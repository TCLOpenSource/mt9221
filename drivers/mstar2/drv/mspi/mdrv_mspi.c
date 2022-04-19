/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////
/// file    mdrv_mspi.c
/// @brief  mspi Driver
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////




#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <asm/io.h>

#include "mdrv_mspi.h"
#include "mhal_mspi.h"

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Description : Set clock polarity of MSPI
/// @return E_MSPI_OK :
/// @return >1 : failed to set clock polarity
//------------------------------------------------------------------------------
static MSPI_ErrorNo _MDrv_CLK_PolaritySetting(MS_U8 u8Pol)
{
    MS_U8 u8PolarityMax;
    MSPI_ErrorNo errnum = E_MSPI_OK;

    u8PolarityMax = HAL_MSPI_CLKConfigMax(E_MSPI_POL);
    if(u8Pol > u8PolarityMax)
        errnum = E_MSPI_PARAM_OVERFLOW;
    else
        HAL_MSPI_SetCLKTime(E_MSPI_POL ,u8Pol);
    return errnum;
}

//------------------------------------------------------------------------------
/// Description : Set clock phase of MSPI
/// @return E_MSPI_OK :
/// @return >1 : failed to set clock phase
//------------------------------------------------------------------------------
static MSPI_ErrorNo _MDrv_CLK_PhaseSetting(MS_U8 u8Pha)
{
    MS_U8 u8PhaseMax;
    MSPI_ErrorNo errnum = E_MSPI_OK;

    u8PhaseMax = HAL_MSPI_CLKConfigMax(E_MSPI_PHA);
    if(u8Pha > u8PhaseMax)
        errnum = E_MSPI_PARAM_OVERFLOW;
    else
        HAL_MSPI_SetCLKTime(E_MSPI_PHA ,u8Pha);
    return errnum;

}

//------------------------------------------------------------------------------
/// Description : Set clock rate of MSPI
/// @return E_MSPI_OK :
/// @return >1 : failed to set clock rate
//------------------------------------------------------------------------------
static MSPI_ErrorNo _MDrv_CLK_ClockSetting(MS_U8 u8Clock)
{
    MS_U8 u8ClockMax;
    MSPI_ErrorNo errnum = E_MSPI_OK;

    u8ClockMax = HAL_MSPI_CLKConfigMax(E_MSPI_CLK);
    if(u8Clock > u8ClockMax)
        errnum = E_MSPI_PARAM_OVERFLOW;
    else
        HAL_MSPI_SetCLKTime(E_MSPI_CLK ,u8Clock);
    return errnum;

}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Description : Set detailed level of MSPI driver debug message
/// @param u8DbgLevel    \b IN  debug level for Serial Flash driver
/// @return TRUE : succeed
/// @return FALSE : failed to set the debug level
//------------------------------------------------------------------------------
MS_BOOL MDrv_MSPI_SetDbgLevel(MS_U8 u8DbgLevel)
{
    u8DbgLevel = u8DbgLevel;

    return TRUE;
}
EXPORT_SYMBOL(MDrv_MSPI_SetDbgLevel);

//------------------------------------------------------------------------------
/// Description : MSPI initial
/// @return E_MSPI_OK :
/// @return >1 : failed to initial
//------------------------------------------------------------------------------
MSPI_ErrorNo MDrv_MSPI_Init(MSPI_CH eChannel)
{
    MSPI_ErrorNo errorno = E_MSPI_OK;
    HAL_MSPI_Init(eChannel);
    //HAL_MSPI_IntEnable(eChannel,TRUE);
    return errorno;
}
EXPORT_SYMBOL(MDrv_MSPI_Init);

MSPI_ErrorNo MDrv_MSPI_Init_Ext(MSPI_CH eChannel)
{
    return MDrv_MSPI_Init(eChannel);
}
EXPORT_SYMBOL(MDrv_MSPI_Init_Ext);
//-------------------------------------------------------------------------------------------------
/// Description : read and write  MSPI
/// @param pData \b IN :pointer to receive data from MSPI read buffer
/// @param u32Size \ b OTU : read data size
/// @return the errorno of operation
//-------------------------------------------------------------------------------------------------
MS_U32  MDrv_MSPI_Read_Write(MSPI_CH eChannel,MS_U8 *pReadData,MS_U8 *pWriteData, MS_U32 u32Size)
{
    MS_U32  u32Index = 0;
    MS_U32  u32TempFrameCnt=0;
    MS_U32  u32TempLastFrameCnt=0;
    MS_BOOL ret = false;
    u32TempFrameCnt = u32Size/MAX_WRITE_BUF_SIZE; //Cut data to frame by max frame size
    u32TempLastFrameCnt = u32Size%MAX_WRITE_BUF_SIZE; //Last data less than a MAX_WRITE_BUF_SIZE fame
    for (u32Index = 0; u32Index < u32TempFrameCnt; u32Index++) {
        ret = HAL_MSPI_Read_Write(eChannel,pReadData+u32Index*MAX_WRITE_BUF_SIZE,pWriteData+u32Index*MAX_WRITE_BUF_SIZE,MAX_WRITE_BUF_SIZE);
        if (!ret) {
            return false;
        }
    }
    if(u32TempLastFrameCnt) {
        ret = HAL_MSPI_Read_Write(eChannel,pReadData+u32TempFrameCnt*MAX_WRITE_BUF_SIZE,pWriteData+u32TempFrameCnt*MAX_WRITE_BUF_SIZE,u32TempLastFrameCnt);
    }
    return ret;
}
EXPORT_SYMBOL(MDrv_MSPI_Read_Write);
//-------------------------------------------------------------------------------------------------
/// Description : read data from MSPI
/// @param pData \b IN :pointer to receive data from MSPI read buffer
/// @param u32Size \ b OTU : read data size
/// @return the errorno of operation
//-------------------------------------------------------------------------------------------------
MSPI_ErrorNo MDrv_MSPI_Read(MSPI_CH eChannel, MS_U8 *pData, MS_U32 u32Size)
{
    MS_U32  u32Index = 0;
    MS_U32  u32TempFrameCnt=0;
    MS_U32  u32TempLastFrameCnt=0;
    int  ret = 0;
    if(pData == NULL) {
        return E_MSPI_NULL;
    }
    u32TempFrameCnt = u32Size/MAX_WRITE_BUF_SIZE; //Cut data to frame by max frame size
    u32TempLastFrameCnt = u32Size%MAX_WRITE_BUF_SIZE; //Last data less than a MAX_WRITE_BUF_SIZE fame
    for (u32Index = 0; u32Index < u32TempFrameCnt; u32Index++) {
        ret = HAL_MSPI_Read(eChannel,pData+u32Index*MAX_WRITE_BUF_SIZE,MAX_WRITE_BUF_SIZE);
        if (!ret) {
            return E_MSPI_OPERATION_ERROR;
        }
    }
    if(u32TempLastFrameCnt) {
        ret = HAL_MSPI_Read(eChannel,pData+u32TempFrameCnt*MAX_WRITE_BUF_SIZE,u32TempLastFrameCnt);
    }
    if (!ret) {
        return E_MSPI_OPERATION_ERROR;
    }
    return E_MSPI_OK;
}
EXPORT_SYMBOL(MDrv_MSPI_Read);

//------------------------------------------------------------------------------
/// Description : read data from MSPI
/// @param pData \b OUT :pointer to write  data to MSPI write buffer
/// @param u32Size \ b OTU : write data size
/// @return the errorno of operation
//------------------------------------------------------------------------------
MSPI_ErrorNo MDrv_MSPI_Write(MSPI_CH eChannel, MS_U8 *pData, MS_U32 u32Size)
{
    MS_U32  u32Index = 0;
    MS_U32  u32TempFrameCnt=0;
    MS_U32  u32TempLastFrameCnt=0;
    MS_BOOL  ret = false;
    u32TempFrameCnt = u32Size/MAX_WRITE_BUF_SIZE; //Cut data to frame by max frame size
    u32TempLastFrameCnt = u32Size%MAX_WRITE_BUF_SIZE; //Last data less than a MAX_WRITE_BUF_SIZE fame
    for (u32Index = 0; u32Index < u32TempFrameCnt; u32Index++) {
        ret = HAL_MSPI_Write(eChannel,pData+u32Index*MAX_WRITE_BUF_SIZE,MAX_WRITE_BUF_SIZE);
        if (!ret) {
            return E_MSPI_OPERATION_ERROR;
        }
    }

    if(u32TempLastFrameCnt) {
        ret = HAL_MSPI_Write(eChannel,pData+u32TempFrameCnt*MAX_WRITE_BUF_SIZE,u32TempLastFrameCnt);
    }
    if (!ret) {
        return E_MSPI_OPERATION_ERROR;
    }
    return E_MSPI_OK;
}
EXPORT_SYMBOL(MDrv_MSPI_Write);

MSPI_ErrorNo MDrv_MSPI_SetReadBufferSize(MSPI_CH eChannel,  MS_U8 u8Size)
{
    HAL_MSPI_SetReadBufferSize( eChannel,  u8Size);
    return E_MSPI_OK;

}
EXPORT_SYMBOL(MDrv_MSPI_SetReadBufferSize);


MSPI_ErrorNo MDrv_MSPI_SetWriteBufferSize(MSPI_CH eChannel,  MS_U8 u8Size)
{
        HAL_MSPI_SetWriteBufferSize( eChannel,  u8Size);
        return E_MSPI_OK;
}
EXPORT_SYMBOL(MDrv_MSPI_SetWriteBufferSize);
//------------------------------------------------------------------------------
/// Description : config spi transfer timing
/// @param ptDCConfig    \b OUT  struct pointer of transfer timing config
/// @return E_MSPI_OK : succeed
/// @return E_MSPI_DCCONFIG_ERROR : failed to config transfer timing
//------------------------------------------------------------------------------
MSPI_ErrorNo MDrv_MSPI_DCConfig(MSPI_CH eChannel, MSPI_DCConfig *ptDCConfig)
{
    MSPI_ErrorNo errnum = E_MSPI_OK;

    if(ptDCConfig == NULL) {
        HAL_MSPI_Reset_DCConfig(eChannel);
        return E_MSPI_OK;
    }
    HAL_MSPI_SetDcTiming(eChannel, E_MSPI_TRSTART ,ptDCConfig->u8TrStart);
    HAL_MSPI_SetDcTiming(eChannel, E_MSPI_TREND ,ptDCConfig->u8TrEnd);
    HAL_MSPI_SetDcTiming(eChannel, E_MSPI_TB ,ptDCConfig->u8TB);
    HAL_MSPI_SetDcTiming(eChannel, E_MSPI_TRW ,ptDCConfig->u8TRW);
    return errnum;
}
EXPORT_SYMBOL(MDrv_MSPI_DCConfig);

//------------------------------------------------------------------------------
/// Description : config spi clock setting
/// @param ptCLKConfig    \b OUT  struct pointer of clock config
/// @return E_MSPI_OK : succeed
/// @return E_MSPI_CLKCONFIG_ERROR : failed to config spi clock
//------------------------------------------------------------------------------
MSPI_ErrorNo  MDrv_MSPI_SetMode(MSPI_CH eChannel, MSPI_Mode_Config_e eMode)
{
    if (eMode >= E_MSPI_MODE_MAX) {
        return E_MSPI_PARAM_OVERFLOW;
    }

    switch (eMode) {
    case E_MSPI_MODE0:
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_POL ,false);
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_PHA ,false);

        break;
    case E_MSPI_MODE1:
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_POL ,false);
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_PHA ,true);
        break;
    case E_MSPI_MODE2:
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_POL ,true);
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_PHA ,false);
        break;
    case E_MSPI_MODE3:
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_POL ,true);
        HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_PHA ,true);
        break;
    default:
        return E_MSPI_OPERATION_ERROR;
    }

    return E_MSPI_OK;
}
EXPORT_SYMBOL(MDrv_MSPI_SetMode);

MSPI_ErrorNo MDrv_MSPI_SetCLK(MSPI_CH eChannel, MS_U8 U8ClockDiv)
{
    MSPI_ErrorNo errnum = E_MSPI_OK;
    HAL_MSPI_SetCLKTiming(eChannel, E_MSPI_CLK ,U8ClockDiv);
    return errnum;

}
EXPORT_SYMBOL(MDrv_MSPI_SetCLK);

// add to sync code from utopia for localdimming to set clk by ini
MSPI_ErrorNo MDrv_MSPI_SetCLKByINI(MSPI_CH eChannel, MS_U32 u32MspiClk)
{
    HAL_MSPI_LD_CLK_Config(eChannel,u32MspiClk);
    return E_MSPI_OK;
}
EXPORT_SYMBOL(MDrv_MSPI_SetCLKByINI);


MS_BOOL MDrv_MSPI_CLOCK_Config(MSPI_CH eChannel, MS_U32 u32MaxClock)
{
    return  HAL_MSPI_CLOCK_Config( eChannel,  u32MaxClock);
}
EXPORT_SYMBOL(MDrv_MSPI_CLOCK_Config);

//------------------------------------------------------------------------------
/// Description : config spi transfer timing
/// @param ptDCConfig    \b OUT  struct pointer of bits of buffer tranferred to slave config
/// @return E_MSPI_OK : succeed
/// @return E_MSPI_FRAMECONFIG_ERROR : failed to config transfered bit per buffer
//------------------------------------------------------------------------------
MSPI_ErrorNo MDrv_MSPI_FRAMEConfig(MSPI_CH eChannel, MSPI_FrameConfig  *ptFrameConfig)
{
    MSPI_ErrorNo errnum = E_MSPI_OK;
    MS_U8 u8Index = 0;

    if(ptFrameConfig == NULL) {
        HAL_MSPI_Reset_FrameConfig(eChannel);
        return E_MSPI_OK;
    }
    // read buffer bit config
    for(u8Index = 0; u8Index < MAX_READ_BUF_SIZE; u8Index++) {
        if(ptFrameConfig->u8RBitConfig[u8Index] > MSPI_FRAME_BIT_MAX) {
            errnum = E_MSPI_PARAM_OVERFLOW;
        } else {
            HAL_MSPI_SetPerFrameSize(eChannel, MSPI_READ_INDEX,  u8Index, ptFrameConfig->u8RBitConfig[u8Index]);
        }
    }
    //write buffer bit config
    for(u8Index = 0; u8Index < MAX_WRITE_BUF_SIZE; u8Index++) {
        if(ptFrameConfig->u8WBitConfig[u8Index] > MSPI_FRAME_BIT_MAX) {
            errnum = E_MSPI_PARAM_OVERFLOW;
        } else {
            HAL_MSPI_SetPerFrameSize(eChannel, MSPI_WRITE_INDEX,  u8Index, ptFrameConfig->u8WBitConfig[u8Index]);
        }
    }
    return errnum;
}
EXPORT_SYMBOL(MDrv_MSPI_FRAMEConfig);

//-------------------------------------------------------
// Description : MSPI Power state
//-------------------------------------------------------
MS_U32 MDrv_MSPI_SetPowerState(void)
{
    return 0;
}
EXPORT_SYMBOL(MDrv_MSPI_SetPowerState);

MSPI_ErrorNo MDrv_MSPI_CLKConfig(MSPI_CLKConfig *ptCLKConfig)
{
    MSPI_ErrorNo errnum = E_MSPI_OK;
    if(ptCLKConfig == NULL)
    {
        HAL_MSPI_Reset_CLKConfig();
        return E_MSPI_OK;
    }

    errnum = _MDrv_CLK_PolaritySetting(ptCLKConfig->BClkPolarity);
    if(errnum != E_MSPI_OK)
        goto ERROR_HANDLE;
    errnum = _MDrv_CLK_PhaseSetting(ptCLKConfig->BClkPhase);
    if(errnum != E_MSPI_OK)
        goto ERROR_HANDLE;
    errnum = _MDrv_CLK_ClockSetting(ptCLKConfig->U8Clock);
    if(errnum != E_MSPI_OK)
        goto ERROR_HANDLE;
    return E_MSPI_OK;

ERROR_HANDLE:
    errnum |= E_MSPI_CLKCONFIG_ERROR;
    return errnum;
}
EXPORT_SYMBOL(MDrv_MSPI_CLKConfig);

void MDrv_MSPI_SlaveEnable(MS_BOOL Enable)
{
  /*1.This Symbol share with User Space but this Chip Select function
      have been implemented in Kernel Space MDrv_MSPI_Read/Write API called
      "HAL_MSPI_SetChipSelect".
  */
}
EXPORT_SYMBOL(MDrv_MSPI_SlaveEnable);

void MDrv_MSPI_SlaveEnable_Channel(MS_U8 u8ch,MS_BOOL Enable)
{
    HAL_MSPI_SetChipSelect(u8ch,  Enable,  E_MSPI_ChipSelect_0);
}
EXPORT_SYMBOL(MDrv_MSPI_SlaveEnable_Channel);

MSPI_ErrorNo MDrv_MSPI_RWBytes(MS_BOOL Direct, MS_U8 u8Bytes)
{
    return HAL_MSPI_RWBytes(Direct,u8Bytes);
}
EXPORT_SYMBOL(MDrv_MSPI_RWBytes);
MSPI_ErrorNo MDrv_MSPI_Read_Channel(MS_U8 u8ch,MS_U8 *pData, MS_U16 u16Size)
{
    return MDrv_MSPI_Read(u8ch,pData,(MS_U32)u16Size);
}
EXPORT_SYMBOL(MDrv_MSPI_Read_Channel);

MSPI_ErrorNo MDrv_MSPI_Write_Channel(MS_U8 u8ch,MS_U8 *pData, MS_U16 u16Size)
{
    return MDrv_MSPI_Write(u8ch,pData,(MS_U32)u16Size);
}
EXPORT_SYMBOL(MDrv_MSPI_Write_Channel);
