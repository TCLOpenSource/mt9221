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

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
//#include "MsCommon.h"
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
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/io.h>

#include "mhal_iic.h"
#include "mhal_iic_reg.h"

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define HWI2C_HAL_RETRY_TIMES       (3)
#define HWI2C_HAL_WAIT_TIMEOUT      (1000)
#define HWI2C_HAL_FUNC()            //{printk("%s\n",  __FUNCTION__);}
#define HWI2C_HAL_INFO(x, args...)  //{printk(x, ##args);}
#define HWI2C_HAL_ERR(x, args...)   //{printk(x, ##args);}

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
static BOOL g_bLastByte[HAL_HWI2C_PORTS];
//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
void MHal_HWI2C_ExtraDelay(U32 u32Us)
{
    /*
    volatile is necessary to avoid optimization
    */
    U32 volatile u32Dummy = 0;
    U32 volatile u32Loop;

    u32Loop = (U32)(50 * u32Us);
    while (u32Loop--)
    {
        u32Dummy++;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_ReadByte
/// @brief \b Function  \b Description: read 1 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b U8
////////////////////////////////////////////////////////////////////////////////
U8 MHal_HWI2C_ReadByte(U32 u32RegAddr)
{
    return ((volatile U8*)(RIU8))[(u32RegAddr << 1) - (u32RegAddr & 1)];
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Read4Byte
/// @brief \b Function  \b Description: read 2 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b U16
////////////////////////////////////////////////////////////////////////////////
U16 MHal_HWI2C_Read2Byte(U32 u32RegAddr)
{
    return ((volatile U16*)(RIU8))[u32RegAddr];
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Read4Byte
/// @brief \b Function  \b Description: read 4 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b U32
////////////////////////////////////////////////////////////////////////////////
U32 MHal_HWI2C_Read4Byte(U32 u32RegAddr)
{
    return (MHal_HWI2C_Read2Byte(u32RegAddr) | MHal_HWI2C_Read2Byte(u32RegAddr + 2) << 16);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_WriteByte
/// @brief \b Function  \b Description: write 1 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u8Val : 1 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_WriteByte(U32 u32RegAddr, U8 u8Val)
{
    if (!u32RegAddr)
    {
        HWI2C_HAL_ERR("%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    ((volatile U8*)(RIU8))[(u32RegAddr << 1) - (u32RegAddr & 1)] = u8Val;
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Write2Byte
/// @brief \b Function  \b Description: write 2 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u16Val : 2 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Write2Byte(U32 u32RegAddr, U16 u16Val)
{
    if (!u32RegAddr)
    {
        HWI2C_HAL_ERR("%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    ((volatile U16*)(RIU8))[u32RegAddr] = u16Val;
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Write4Byte
/// @brief \b Function  \b Description: write 4 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u32Val : 4 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Write4Byte(U32 u32RegAddr, U32 u32Val)
{
    if (!u32RegAddr)
    {
        HWI2C_HAL_ERR("%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    MHal_HWI2C_Write2Byte(u32RegAddr, u32Val & 0x0000FFFF);
    MHal_HWI2C_Write2Byte(u32RegAddr+2, u32Val >> 16);
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_WriteRegBit
/// @brief \b Function  \b Description: write 1 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u8Val : 1 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_WriteRegBit(U32 u32RegAddr, U8 u8Mask, BOOL bEnable)
{
    U8 u8Val = 0;

    if (!u32RegAddr)
    {
        HWI2C_HAL_ERR("%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    u8Val = MHal_HWI2C_ReadByte(u32RegAddr);
    u8Val = (bEnable) ? (u8Val | u8Mask) : (u8Val & ~u8Mask);
    MHal_HWI2C_WriteByte(u32RegAddr, u8Val);
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_WriteByteMask
/// @brief \b Function  \b Description: write data with mask bits
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u8Val : 1 byte data
/// @param <IN>         \b u8Mask : mask bits
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_WriteByteMask(U32 u32RegAddr, U8 u8Val, U8 u8Mask)
{
    if (!u32RegAddr)
    {
        HWI2C_HAL_ERR("%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    u8Val = (MHal_HWI2C_ReadByte(u32RegAddr) & ~u8Mask) | (u8Val & u8Mask);
    MHal_HWI2C_WriteByte(u32RegAddr, u8Val);
    return TRUE;
}

//#####################
//
//  MIIC STD Related Functions
//  Static or Internal use
//
//#####################
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_EnINT
/// @brief \b Function  \b Description: Enable Interrupt
/// @param <IN>         \b bEnable : TRUE: Enable, FALSE: Disable
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok, FALSE: Fail
////////////////////////////////////////////////////////////////////////////////
static BOOL MHal_HWI2C_EnINT(U32 u32PortBaseReg, BOOL bEnable)
{
    HWI2C_HAL_FUNC();
    return MHal_HWI2C_WriteRegBit(REG_HWI2C_MIIC_CFG + u32PortBaseReg, _MIIC_CFG_EN_INT, bEnable);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_SendData
/// @brief \b Function  \b Description: Send 1 byte data to SDA
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_SendData(U32 u32PortBaseReg, U8 u8Data)
{
    HWI2C_HAL_FUNC();

    return (MHal_HWI2C_WriteByte(REG_HWI2C_WDATA + u32PortBaseReg, u8Data));
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_RecvData
/// @brief \b Function  \b Description: Receive 1 byte data from SDA
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b MS_U8 :
////////////////////////////////////////////////////////////////////////////////
U8 MHal_HWI2C_RecvData(U32 u32PortBaseReg)
{
    HWI2C_HAL_FUNC();

    return (MHal_HWI2C_ReadByte(REG_HWI2C_RDATA + u32PortBaseReg));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_GetStae
/// @brief \b Function  \b Description: Get i2c Current State
/// @param <IN>         \b u32PortOffset: HWI2C Port Offset
/// @param <OUT>        \b None
/// @param <RET>        \b HWI2C current status
////////////////////////////////////////////////////////////////////////////////
U8 MHal_HWI2C_GetState(U32 u32PortBaseReg)
{
    U8 cur_state = MHal_HWI2C_ReadByte(REG_HWI2C_CUR_STATE + u32PortBaseReg) & _CUR_STATE_MSK;
    HWI2C_HAL_FUNC();

    if (cur_state <= 0) // 0: idle
        return E_HAL_HWI2C_STATE_IDEL;
    else if (cur_state <= 2) // 1~2:start
        return E_HAL_HWI2C_STATE_START;
    else if (cur_state <= 6) // 3~6:write
        return E_HAL_HWI2C_STATE_WRITE;
    else if (cur_state <= 10) // 7~10:read
        return E_HAL_HWI2C_STATE_READ;
    else if (cur_state <= 11) // 11:interrupt
        return E_HAL_HWI2C_STATE_INT;
    else if (cur_state <= 12) // 12:wait
        return E_HAL_HWI2C_STATE_WAIT;
    else  // 13~15:stop
        return E_HAL_HWI2C_STATE_STOP;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Get_SendAck
/// @brief \b Function  \b Description: Get ack after sending data
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Valid ack, FALSE: No ack
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Get_SendAck(U32 u32PortBaseReg)
{
    HWI2C_HAL_FUNC();

    return (MHal_HWI2C_ReadByte(REG_HWI2C_WDATA_GET + u32PortBaseReg) & _WDATA_GET_ACKBIT) ? FALSE : TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Is_Idle
/// @brief \b Function  \b Description: Check if i2c is idle
/// @param <IN>         \b u32PortOffset: HWI2C Port Offset
/// @param <OUT>        \b None
/// @param <RET>        \b TRUE : idle, FALSE : not idle
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Is_Idle(U32 u32PortBaseReg)
{
    HWI2C_HAL_FUNC();

    return ((MHal_HWI2C_GetState(u32PortBaseReg) == E_HAL_HWI2C_STATE_IDEL) ? TRUE : FALSE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Is_INT
/// @brief \b Function  \b Description: Check if i2c is interrupted
/// @param <IN>         \b u8Status : queried status
/// @param <IN>         \b u8Ch: Channel 0/1
/// @param <OUT>        \b None
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Is_INT(U32 u32PortBaseReg)
{
    HWI2C_HAL_FUNC();

    return (MHal_HWI2C_ReadByte(REG_HWI2C_INT_CTL + u32PortBaseReg) & _INT_CTL) ? TRUE : FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Clear_INT
/// @brief \b Function  \b Description: Enable interrupt for HWI2C
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
static BOOL MHal_HWI2C_Clear_INT(U32 u32PortBaseReg)
{
    HWI2C_HAL_FUNC();

    return MHal_HWI2C_WriteRegBit(REG_HWI2C_INT_CTL + u32PortBaseReg, _INT_CTL, TRUE);
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
void MHal_IIC_Init(void)
{

}

void MHal_IIC_Clock_Select(U8 u8ClockIIC)
{

}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_EnClkStretch
/// @brief \b Function  \b Description: Enable Clock Stretch
/// @param <IN>         \b bEnable : TRUE: Enable, FALSE: Disable
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok, FALSE: Fail
////////////////////////////////////////////////////////////////////////////////
static BOOL MHal_HWI2C_EnClkStretch(U32 u32PortBaseReg, BOOL bEnable)
{
    HWI2C_HAL_FUNC();
    return MHal_HWI2C_WriteRegBit(REG_HWI2C_MIIC_CFG + u32PortBaseReg, _MIIC_CFG_EN_CLKSTR, bEnable);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_EnFilter
/// @brief \b Function  \b Description: Enable Filter
/// @param <IN>         \b bEnable : TRUE: Enable, FALSE: Disable
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok, FALSE: Fail
////////////////////////////////////////////////////////////////////////////////
static BOOL MHal_HWI2C_EnFilter(U32 u32PortBaseReg, BOOL bEnable)
{
    HWI2C_HAL_FUNC();
    return MHal_HWI2C_WriteRegBit(REG_HWI2C_MIIC_CFG + u32PortBaseReg, _MIIC_CFG_EN_FILTER, bEnable);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_EnPushSda
/// @brief \b Function  \b Description: Enable push current for SDA
/// @param <IN>         \b bEnable : TRUE: Enable, FALSE: Disable
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok, FALSE: Fail
////////////////////////////////////////////////////////////////////////////////
static BOOL MHal_HWI2C_EnPushSda(U32 u32PortBaseReg, BOOL bEnable)
{
    HWI2C_HAL_FUNC();
    return MHal_HWI2C_WriteRegBit(REG_HWI2C_MIIC_CFG + u32PortBaseReg, _MIIC_CFG_EN_PUSH1T, bEnable);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Reset
/// @brief \b Function  \b Description: Reset HWI2C state machine
/// @param <IN>         \b bReset : TRUE: Reset FALSE: Not reset
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok, FALSE: Fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Reset(U32 u32PortBaseReg, BOOL bReset)
{
    HWI2C_HAL_FUNC();
    return MHal_HWI2C_WriteRegBit(REG_HWI2C_MIIC_CFG + u32PortBaseReg, _MIIC_CFG_RESET, bReset);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Reset_ALL
/// @brief \b Function  \b Description: Reset ALL HWI2C ENGINE
/// @param <IN>         \b bReset : TRUE: Reset FALSE: Not reset
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok, FALSE: Fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Reset_ALL(U32 u32PortBaseReg, BOOL bReset)
{
    HWI2C_HAL_FUNC();
#if 0
    U16 count = HWI2C_HAL_WAIT_TIMEOUT;
    MHal_HWI2C_WriteRegBit(REG_HWI2C_RESERVE1 + u32PortBaseReg, _MIIC_RESET, !bReset);

    while(count)
    {
        if(((MHal_HWI2C_ReadByte(REG_HWI2C_RESERVE1 + u32PortBaseReg) & _MIIC_RESET)) == !bReset)
            break;

        udelay(1);
        count--;
    }

    if(count == 0)
    {
        HWI2C_HAL_ERR("[%s][%d] Reset Failed!\n");
        return FALSE;
    }
    else
    {
        return TRUE;
    }
#endif
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Master_Enable
/// @brief \b Function  \b Description: Master I2C enable
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_Master_Enable(U32 u32PortBaseReg)
{
    BOOL bRet = TRUE;

    HWI2C_HAL_FUNC();

    /*
    (1) clear interrupt
    */
    MHal_HWI2C_Clear_INT(u32PortBaseReg);

    /*
    (2) reset standard master iic
    */
    MHal_HWI2C_Reset(u32PortBaseReg, TRUE);
    MHal_HWI2C_Reset(u32PortBaseReg, FALSE);

    /*
    (3) configuration
    */
    MHal_HWI2C_EnINT(u32PortBaseReg, TRUE);
    MHal_HWI2C_EnClkStretch(u32PortBaseReg, TRUE);
    MHal_HWI2C_EnFilter(u32PortBaseReg, TRUE);
    MHal_HWI2C_EnPushSda(u32PortBaseReg, TRUE);
    //(4) Disable DMA
    //bRet = MHAL_HWI2C_DMA_Enable(u32PortBaseReg, FALSE);

    return bRet;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_SelectPort
/// @brief \b Function  \b Description: Select HWI2C port
/// @param <IN>         \b None : HWI2C port
/// @param param        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_SelectPort(HAL_HWI2C_PORT ePort)
{
    U8 u8Value1 = 0;

    HWI2C_HAL_FUNC();

    //decide port mask
    if ((ePort>=E_HAL_HWI2C_PORT0_0)&&(ePort<=E_HAL_HWI2C_PORT0_7))//port 0
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT0_0:
                u8Value1 = CHIP_MIIC0_PAD_0;
                break;
            case E_HAL_HWI2C_PORT0_1:
                u8Value1 = CHIP_MIIC0_PAD_1;
                break;
            default:
                return FALSE;
        }
        MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC0, u8Value1, CHIP_MIIC0_PAD_MSK);
    }
    else if ((ePort>=E_HAL_HWI2C_PORT1_0)&&(ePort<=E_HAL_HWI2C_PORT1_7))//port 1
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT1_0:
                u8Value1 = CHIP_MIIC1_PAD_0;
                break;
            case E_HAL_HWI2C_PORT1_1:
                u8Value1 = CHIP_MIIC1_PAD_1;
                break;
            case E_HAL_HWI2C_PORT1_2:
                u8Value1 = CHIP_MIIC1_PAD_2;
                break;
            default:
                return FALSE;
        }
        MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC1, u8Value1, CHIP_MIIC1_PAD_MSK);
    }
    else if ((ePort>=E_HAL_HWI2C_PORT2_0)&&(ePort<=E_HAL_HWI2C_PORT2_7))//port 2
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT2_0:
                u8Value1 = CHIP_MIIC2_PAD_0;
                break;
            case E_HAL_HWI2C_PORT2_1:
                u8Value1 = CHIP_MIIC2_PAD_1;
                break;
            default:
                return FALSE;
        }
        MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC2, u8Value1, CHIP_MIIC2_PAD_MSK);
    }
    else if ((ePort>=E_HAL_HWI2C_PORT3_0)&&(ePort<=E_HAL_HWI2C_PORT3_7))//port 3
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT3_0:
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_DDCR, CHIP_DDCR_PAD_0, CHIP_DDCR_PAD_MSK);
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC3, CHIP_MIIC3_PAD_0, CHIP_MIIC3_PAD_MSK);
                break;
            case E_HAL_HWI2C_PORT3_1:
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_DDCR, CHIP_DDCR_PAD_1, CHIP_DDCR_PAD_MSK);
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC3, CHIP_MIIC3_PAD_0, CHIP_MIIC3_PAD_MSK);
                break;
            case E_HAL_HWI2C_PORT3_2:
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_DDCR, CHIP_DDCR_PAD_0, CHIP_DDCR_PAD_MSK);
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC3, CHIP_MIIC3_PAD_1, CHIP_MIIC3_PAD_MSK);
                break;
            case E_HAL_HWI2C_PORT3_3:
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_DDCR, CHIP_DDCR_PAD_0, CHIP_DDCR_PAD_MSK);
                MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC3, CHIP_MIIC3_PAD_2, CHIP_MIIC3_PAD_MSK);
                break;
            default:
                return FALSE;
        }
    }
    else if ((ePort>=E_HAL_HWI2C_PORT4_0)&&(ePort<=E_HAL_HWI2C_PORT4_7))//port 4
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT4_0:
                u8Value1 = CHIP_MIIC4_PAD_0;
                break;
            case E_HAL_HWI2C_PORT4_1:
                u8Value1 = CHIP_MIIC4_PAD_1;
                break;
            default:
                return FALSE;
        }
        MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC4, u8Value1, CHIP_MIIC4_PAD_MSK);
    }
    else if ((ePort>=E_HAL_HWI2C_PORT5_0)&&(ePort<=E_HAL_HWI2C_PORT5_7)) //port 5
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT5_0:
                u8Value1 = CHIP_MIIC5_PAD_0;
                break;
            case E_HAL_HWI2C_PORT5_1:
                u8Value1 = CHIP_MIIC5_PAD_1;
                break;
            default:
                return FALSE;
        }
        MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC5, u8Value1, CHIP_MIIC5_PAD_MSK);
    }
    else if ((ePort>=E_HAL_HWI2C_PORT6_0)&&(ePort<=E_HAL_HWI2C_PORT6_7)) //port 6
    {
        switch(ePort)
        {
            case E_HAL_HWI2C_PORT6_0:
                u8Value1 = CHIP_MIIC6_PAD_0;
                break;
            case E_HAL_HWI2C_PORT6_1:
                u8Value1 = CHIP_MIIC6_PAD_1;
                break;
            case E_HAL_HWI2C_PORT6_2:
                u8Value1 = CHIP_MIIC6_PAD_2;
                break;
            default:
                return FALSE;
        }
        MHal_HWI2C_WriteByteMask(CHIP_REG_HWI2C_MIIC6, u8Value1, CHIP_MIIC6_PAD_MSK);
    }
    else
    {
        return FALSE;
    }
    return TRUE;
}


BOOL MHal_HWI2C_GetBaseRegByPort(HAL_HWI2C_PORT ePort, U32* pu32PortBaseReg)
{
    HWI2C_HAL_FUNC();

    if((ePort>=E_HAL_HWI2C_PORT0_0)&&(ePort<=E_HAL_HWI2C_PORT0_7))
    {//port 0 : bank register address 0x1118
        *pu32PortBaseReg = (U32)HWI2C_PORT0_REG_BASE;
    }
    else if((ePort>=E_HAL_HWI2C_PORT1_0)&&(ePort<=E_HAL_HWI2C_PORT1_7))
    {//port 1 : bank register address 0x1119
        *pu32PortBaseReg = (U32)HWI2C_PORT1_REG_BASE;
    }
    else if((ePort>=E_HAL_HWI2C_PORT2_0)&&(ePort<=E_HAL_HWI2C_PORT2_7))
    {//port 2 : bank register address 0x111A
        *pu32PortBaseReg = (U32)HWI2C_PORT2_REG_BASE;
    }
    else if((ePort>=E_HAL_HWI2C_PORT3_0)&&(ePort<=E_HAL_HWI2C_PORT3_7))
    {//port 3 : bank register address 0x111B
        *pu32PortBaseReg = (U32)HWI2C_PORT3_REG_BASE;
    }
    else if((ePort>=E_HAL_HWI2C_PORT4_0)&&(ePort<=E_HAL_HWI2C_PORT4_7))
    {//port 4 : bank register address 0x121C
        *pu32PortBaseReg = (U32)HWI2C_PORT4_REG_BASE;
    }
    else if((ePort>=E_HAL_HWI2C_PORT5_0)&&(ePort<=E_HAL_HWI2C_PORT5_7))
    {//port 5 : bank register address 0x121D
        *pu32PortBaseReg = (U32)HWI2C_PORT5_REG_BASE;
    }
    else if((ePort>=E_HAL_HWI2C_PORT6_0)&&(ePort<=E_HAL_HWI2C_PORT6_7))
    {//port 6 : bank register address 0x0x0b00
        *pu32PortBaseReg = (U32)HWI2C_PORT6_REG_BASE;
    }
    else
    {
        *pu32PortBaseReg = (U32)0x00;
        return FALSE;
    }
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_GetPortIdxByPort
/// @brief \b Function  \b Description: Get HWI2C port index by port number
/// @param <IN>         \b ePort : port number
/// @param <OUT>         \b pu8Port :  port index
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_GetPortIdxByPort(HAL_HWI2C_PORT ePort, U8* pu8Port)
{
    HWI2C_HAL_FUNC();

    if((ePort>=E_HAL_HWI2C_PORT0_0)&&(ePort<=E_HAL_HWI2C_PORT0_7))
    {
        *pu8Port = HAL_HWI2C_PORT0;
    }
    else if((ePort>=E_HAL_HWI2C_PORT1_0)&&(ePort<=E_HAL_HWI2C_PORT1_7))
    {
        *pu8Port = HAL_HWI2C_PORT1;
    }
    else if((ePort>=E_HAL_HWI2C_PORT2_0)&&(ePort<=E_HAL_HWI2C_PORT2_7))
    {
        *pu8Port = HAL_HWI2C_PORT2;
    }
    else if((ePort>=E_HAL_HWI2C_PORT3_0)&&(ePort<=E_HAL_HWI2C_PORT3_7))
    {
        *pu8Port = HAL_HWI2C_PORT3;
    }
    else if((ePort>=E_HAL_HWI2C_PORT4_0)&&(ePort<=E_HAL_HWI2C_PORT4_7))
    {
        *pu8Port = HAL_HWI2C_PORT4;
    }
    else if((ePort>=E_HAL_HWI2C_PORT5_0)&&(ePort<=E_HAL_HWI2C_PORT5_7))
    {
        *pu8Port = HAL_HWI2C_PORT5;
    }
    else if((ePort>=E_HAL_HWI2C_PORT6_0)&&(ePort<=E_HAL_HWI2C_PORT6_7))
    {
        *pu8Port = HAL_HWI2C_PORT6;
    }
    else
    {
        *pu8Port = HAL_HWI2C_PORT0;
         return FALSE;
    }
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: MHal_HWI2C_GetPortIdxByPortBaseReg
/// @brief \b Function  \b Description: Get HWI2C port index by base register of port
/// @param <IN>         \b u16Offset : port base register
/// @param <OUT>         \b pu8Port :  port index
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_GetPortIdxByPortBaseReg(U32 u32PortBaseReg, U8* pu8Port)
{
    HWI2C_HAL_FUNC();

    if(u32PortBaseReg==(U32)HWI2C_PORT0_REG_BASE)
    {//port 0 : bank register address 0x1118
        *pu8Port = HAL_HWI2C_PORT0;
            }
    else if(u32PortBaseReg==(U32)HWI2C_PORT1_REG_BASE)
    {//port 1 : bank register address 0x1119
        *pu8Port = HAL_HWI2C_PORT1;
    }
    else if(u32PortBaseReg==(U32)HWI2C_PORT2_REG_BASE)
    {//port 2 : bank register address 0x111A
        *pu8Port = HAL_HWI2C_PORT2;
    }
    else if(u32PortBaseReg==(U32)HWI2C_PORT3_REG_BASE)
    {//port 3 : bank register address 0x111B
        *pu8Port = HAL_HWI2C_PORT3;
    }
    else if(u32PortBaseReg==(U32)HWI2C_PORT4_REG_BASE)
    {//port 4 : bank register address 0x121C
        *pu8Port = HAL_HWI2C_PORT4;
    }
    else if(u32PortBaseReg==(U32)HWI2C_PORT5_REG_BASE)
    {//port 5 : bank register address 0x121D
        *pu8Port = HAL_HWI2C_PORT5;
    }
    else if(u32PortBaseReg==(U32)HWI2C_PORT6_REG_BASE)
    {//port 6 : bank register address 0x0b00
        *pu8Port = HAL_HWI2C_PORT6;
    }
    else
    {
        *pu8Port = HAL_HWI2C_PORT0;
        return FALSE;
    }
    return TRUE;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_SetClk
/// @brief \b Function  \b Description: Set I2C clock
/// @param <IN>         \b u8Clk: clock rate
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_SetClk(U32 u32PortBaseReg, U32 eClkSel)
{
    U16 u16ClkHCnt = 0;
    U16 u16ClkLCnt = 0;
    U16 u16StpCnt = 0;
    U16 u16SdaCnt = 0;
    U16 u16SttCnt = 0;
    U16 u16LchCnt = 0;

    HWI2C_HAL_FUNC();

      /*
        //Xtal = 12M Hz
      */
    switch (eClkSel)
    {
        case 400: // 400 KHz
            u16ClkHCnt =   9; u16ClkLCnt =   13; break;
        case 300: //300 KHz
            u16ClkHCnt =  15; u16ClkLCnt =   17; break;
        case 200: //200 KHz
            u16ClkHCnt =  25; u16ClkLCnt =   27; break;
        case 100: //100 KHz
            u16ClkHCnt =  57; u16ClkLCnt =   59; break;
        case 50: //50 KHz
            u16ClkHCnt =  115; u16ClkLCnt = 117; break;
        case 25: //25 KHz
            u16ClkHCnt =  235; u16ClkLCnt = 237; break;
        default:
            u16ClkHCnt =  15; u16ClkLCnt =  17; break;
    }
    u16SttCnt = 8; u16StpCnt = 8; u16SdaCnt = 5; u16LchCnt = 5;

    MHal_HWI2C_Write2Byte(REG_HWI2C_CKH_CNT + u32PortBaseReg, u16ClkHCnt);
    MHal_HWI2C_Write2Byte(REG_HWI2C_CKL_CNT + u32PortBaseReg, u16ClkLCnt);
    MHal_HWI2C_Write2Byte(REG_HWI2C_STP_CNT + u32PortBaseReg, u16StpCnt);
    MHal_HWI2C_Write2Byte(REG_HWI2C_SDA_CNT + u32PortBaseReg, u16SdaCnt);
    MHal_HWI2C_Write2Byte(REG_HWI2C_STT_CNT + u32PortBaseReg, u16SttCnt);
    MHal_HWI2C_Write2Byte(REG_HWI2C_LTH_CNT + u32PortBaseReg, u16LchCnt);
    //HAL_HWI2C_Write2Byte(REG_HWI2C_TMT_CNT+u32PortBaseReg, 0x0000);
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Start
/// @brief \b Function  \b Description: Send start condition
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_IIC_Start(U32 u32PortBaseReg)
{
    U16 u16Count = HWI2C_HAL_WAIT_TIMEOUT;

    //reset iic
    MHal_HWI2C_WriteRegBit(REG_HWI2C_CMD_START + u32PortBaseReg, _CMD_START, TRUE);

    while ((!MHal_HWI2C_Is_INT(u32PortBaseReg)) && (u16Count > 0))
    {
        udelay(1);
        u16Count--;
    }

    MHal_HWI2C_Clear_INT(u32PortBaseReg);

    while(u16Count)
    {
        if(MHal_HWI2C_GetState(u32PortBaseReg) == E_HAL_HWI2C_STATE_WAIT)
            break;
        udelay(1);
        u16Count--;
    }

    return (u16Count) ? TRUE : FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_Stop
/// @brief \b Function  \b Description: Send Stop condition
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_IIC_Stop(U32 u32PortBaseReg)
{
    U16 u16Count = HWI2C_HAL_WAIT_TIMEOUT;

    MHal_HWI2C_WriteRegBit(REG_HWI2C_CMD_STOP + u32PortBaseReg, _CMD_STOP, TRUE);

    while ((!MHal_HWI2C_Is_Idle(u32PortBaseReg)) && (!MHal_HWI2C_Is_INT(u32PortBaseReg)) && (u16Count > 0))
    {
        u16Count--;
        udelay(1);
    }

    MHal_HWI2C_Clear_INT(u32PortBaseReg);

    while(u16Count)
    {
        if(MHal_HWI2C_GetState(u32PortBaseReg) == E_HAL_HWI2C_STATE_IDEL)
            break;
        udelay(1);
        u16Count--;
    }

    return (u16Count) ? TRUE : FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_ReadRdy
/// @brief \b Function  \b Description: Start byte reading
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_ReadRdy(U32 u32PortBaseReg)
{
    U8 u8Value = 0;
    U8 u8Port;

    if (MHal_HWI2C_GetPortIdxByPortBaseReg(u32PortBaseReg, &u8Port) == FALSE)
        return FALSE;

    u8Value = g_bLastByte[u8Port] ? (_RDATA_CFG_TRIG|_RDATA_CFG_ACKBIT) : (_RDATA_CFG_TRIG);
    g_bLastByte[u8Port] = FALSE;

    return MHal_HWI2C_WriteByte(REG_HWI2C_RDATA_CFG + u32PortBaseReg, u8Value);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_HWI2C_NoAck
/// @brief \b Function  \b Description: generate no ack pulse
/// @param <IN>         \b None :
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE : ok, FALSE : fail
////////////////////////////////////////////////////////////////////////////////
BOOL MHal_HWI2C_NoAck(U32 u32PortBaseReg)
{
    U8 u8Port;

    HWI2C_HAL_FUNC();

    if (MHal_HWI2C_GetPortIdxByPortBaseReg(u32PortBaseReg, &u8Port)==FALSE)
        return FALSE;
    g_bLastByte[u8Port] = TRUE;

    return TRUE;
}

U16 MHal_IIC_SendByte(U32 u32PortBaseReg, U8 u8DataIIC)
{
    U16 u16Count = HWI2C_HAL_WAIT_TIMEOUT;
    U16 u16intCount = HWI2C_HAL_WAIT_TIMEOUT;

    u16intCount = HWI2C_HAL_WAIT_TIMEOUT;
    while(MHal_HWI2C_GetState(u32PortBaseReg) != E_HAL_HWI2C_STATE_WAIT)
    {
        if(u16intCount == 0)
        {
              HWI2C_HAL_ERR("[%s][%d] Clear INT timeout\n", __FUNCTION__, __LINE__);
              goto IIC_SendByte_End;
        }
        u16intCount--;
        udelay(1);
    }

    if (MHal_HWI2C_SendData(u32PortBaseReg, u8DataIIC) == TRUE)
    {
        u16Count = HWI2C_HAL_WAIT_TIMEOUT;
        while (u16Count)
        {
            if (MHal_HWI2C_Is_INT(u32PortBaseReg))
            {
                MHal_HWI2C_Clear_INT(u32PortBaseReg);

                u16intCount = HWI2C_HAL_WAIT_TIMEOUT;
                while(MHal_HWI2C_GetState(u32PortBaseReg) != E_HAL_HWI2C_STATE_WAIT)
                {
                    if(u16intCount == 0)
                    {
                        HWI2C_HAL_ERR("[%s][%d] Clear INT timeout\n", __FUNCTION__, __LINE__);
                        goto IIC_SendByte_End;
                    }
                    u16intCount--;
                    udelay(1);
                }

                udelay(1);

                if (MHal_HWI2C_Get_SendAck(u32PortBaseReg))
                {
                    udelay(1);
                    return TRUE;
                }
                HWI2C_HAL_ERR("No ACK!\n");
                break;
            }
            udelay(1);
            u16Count--;
        }

        if(u16Count == 0)
            HWI2C_HAL_ERR("[%s][%d] no interrupt!\n", __FUNCTION__, __LINE__);
    }

IIC_SendByte_End:
    HWI2C_HAL_ERR("Send byte 0x%X fail!\n", u8DataIIC);
    return FALSE;
}

U16 MHal_IIC_GetByte(U32 u32PortBaseReg, U8* pu8DataIIC) /* auto generate ACK */
{
    U16 u16Count = HWI2C_HAL_WAIT_TIMEOUT;
    U16 u16intCount = HWI2C_HAL_WAIT_TIMEOUT;

    if (!pu8DataIIC)
        return FALSE;

    MHal_HWI2C_ReadRdy(u32PortBaseReg);

    while ((!MHal_HWI2C_Is_INT(u32PortBaseReg)) && (u16Count > 0))
    {
        u16Count--;
        udelay(1);
    }

    MHal_HWI2C_Clear_INT(u32PortBaseReg);

    u16intCount = HWI2C_HAL_WAIT_TIMEOUT;
    while(MHal_HWI2C_GetState(u32PortBaseReg) != E_HAL_HWI2C_STATE_WAIT && (u16Count > 0))
    {
        if(u16intCount == 0)
        {
            HWI2C_HAL_ERR("[%s][%d] Clear INT timeout\n", __FUNCTION__, __LINE__);
            goto IIC_GetByte_End;
        }
        u16intCount--;
            udelay(1);
    }

    if (u16Count)
    {
        //get data before clear int and stop
        *pu8DataIIC = MHal_HWI2C_RecvData(u32PortBaseReg);
        //clear interrupt
        MHal_HWI2C_Clear_INT(u32PortBaseReg);
        u16intCount = HWI2C_HAL_WAIT_TIMEOUT;
        while(MHal_HWI2C_GetState(u32PortBaseReg) != E_HAL_HWI2C_STATE_WAIT)
        {
            if(u16intCount == 0)
            {
                HWI2C_HAL_ERR("[%s][%d] Clear INT timeout\n", __FUNCTION__, __LINE__);
                goto IIC_GetByte_End;
            }
            u16intCount--;
            udelay(1);
        }
        udelay(1);
            return TRUE;
        }

IIC_GetByte_End:
    HWI2C_HAL_ERR("Recv byte fail!\n");
    return FALSE;
}

U32 MHal_Query_MIPS_CLK(void)
{
    unsigned int u32Speed = 0;
    unsigned int u32Count = 0;

    u32Speed = *(volatile u32*)(0xBF22184C);
    u32Count = (u32Speed & 0x00FF) * 12000000; //take 12MHz as reference
    if(((u32Speed&BIT12)==0) && ((u32Speed&BIT13)==0))
    {
        u32Count = u32Count;
    }
    else if(((u32Speed&BIT12)!=0) && ((u32Speed&BIT13)==0))
    {
        u32Count=u32Count>>1;
    }
    else if(((u32Speed&BIT12)==0) && ((u32Speed&BIT13)!=0))
    {
        u32Count=u32Count>>2;
    }
    else
    {
        u32Count=u32Count>>3;
    }
    return u32Count;
}
