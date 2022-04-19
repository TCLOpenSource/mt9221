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

#ifndef _MHAL_MSPI_H_
#define _MHAL_MSPI_H_

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define MSPI_READ_INDEX                0x0
#define MSPI_WRITE_INDEX               0x1
/* check if chip support MSPI*/
//#define HAL_MSPI_HW_Support()          TRUE
#define DEBUG_MSPI(debug_level, x)     do { if (_u8MSPIDbgLevel >= (debug_level)) (x); } while(0)

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------



typedef enum {
    E_MSPI_MODE0, //CPOL = 0,CPHA =0
    E_MSPI_MODE1, //CPOL = 0,CPHA =1
    E_MSPI_MODE2, //CPOL = 1,CPHA =0
    E_MSPI_MODE3, //CPOL = 1,CPHA =1
    E_MSPI_MODE_MAX,
} MSPI_Mode_Config_e;

typedef enum
{
    E_MSPI1,
    E_MSPI2,
    E_MSPI_MAX,
}MSPI_CH;

typedef enum
{
    E_MSPI_ChipSelect_0,
    E_MSPI_ChipSelect_1,
    E_MSPI_ChipSelect_2,
    E_MSPI_ChipSelect_3,
    E_MSPI_ChipSelect_4,
    E_MSPI_ChipSelect_5,
    E_MSPI_ChipSelect_6,
    E_MSPI_ChipSelect_7,
    E_MSPI_ChipSelect_MAX
}MSPI_ChipSelect_e;

typedef enum
{
    E_MSPI_BIT_MSB_FIRST,
    E_MSPI_BIT_LSB_FIRST,
}MSPI_BitSeq_e;

typedef enum _HAL_CLK_Config
{
    E_MSPI_POL,
    E_MSPI_PHA,
    E_MSPI_CLK
}eCLK_config;

typedef enum _HAL_DC_Config
{
    E_MSPI_TRSTART,
    E_MSPI_TREND,
    E_MSPI_TB,
    E_MSPI_TRW
}eDC_config;

typedef struct
{
    MSPI_CH eCurrentCH;;
    MS_U32 VirtMspBaseAddr;
    MS_U32 VirtClkBaseAddr;
} MSPI_BaseAddr_st;

typedef struct
{
    MS_U8 u8TrStart;      //time from trigger to first SPI clock
    MS_U8 u8TrEnd;        //time from last SPI clock to transferred done
    MS_U8 u8TB;           //time between byte to byte transfer
    MS_U8 u8TRW;          //time between last write and first read
} MSPI_DCConfig;

typedef struct
{
    MS_U8 u8WBitConfig[8];      //bits will be transferred in write buffer
    MS_U8 u8RBitConfig[8];      //bits Will be transferred in read buffer
} MSPI_FrameConfig;

typedef struct
{
    MS_U8 U8Clock;
    MS_BOOL BClkPolarity;
    MS_BOOL BClkPhase;
    MS_U32 u32MAXClk;
} MSPI_CLKConfig;


typedef struct
{
    MS_U8 u8ClkSpi_P1;
    MS_U8 u8ClkSpi_P2;
    MS_U8 u8ClkSpi_DIV;
    MS_U32 u32ClkSpi;
}ST_DRV_LD_MSPI_CLK;


typedef struct
{
    MS_BOOL bEnable;
    MSPI_CH eChannel;
    MSPI_Mode_Config_e eMSPIMode;
    MSPI_BaseAddr_st stBaseAddr;
    MSPI_CLKConfig tMSPI_ClockConfig;
    MSPI_DCConfig  tMSPI_DCConfig;
    MSPI_FrameConfig  tMSPI_FrameConfig;
    MSPI_ChipSelect_e eChipSel;
    MSPI_BitSeq_e eBLsbFirst;
    MS_U8 u8MspiBuffSizes;              //spi write buffer size
    void (*MSPIIntHandler)(void);       // interrupt handler
    MS_BOOL BIntEnable;                 // interrupt mode enable or polling mode
    //MS_U8 U8BitMapofConfig;
    //MS_U32 u32DevId;
} MSPI_config;


typedef enum
{
    E_MSPI_DBGLV_NONE,    //disable all the debug message
    E_MSPI_DBGLV_INFO,    //information
    E_MSPI_DBGLV_NOTICE,  //normal but significant condition
    E_MSPI_DBGLV_WARNING, //warning conditions
    E_MSPI_DBGLV_ERR,     //error conditions
    E_MSPI_DBGLV_CRIT,    //critical conditions
    E_MSPI_DBGLV_ALERT,   //action must be taken immediately
    E_MSPI_DBGLV_EMERG,   //system is unusable
    E_MSPI_DBGLV_DEBUG,   //debug-level messages
} MSPI_DbgLv;


typedef enum _MSPI_ERRORNOn {
     E_MSPI_OK = 0
    ,E_MSPI_INIT_FLOW_ERROR =1
    ,E_MSPI_DCCONFIG_ERROR =2
    ,E_MSPI_CLKCONFIG_ERROR =4
    ,E_MSPI_FRAMECONFIG_ERROR =8
    ,E_MSPI_OPERATION_ERROR = 0x10
    ,E_MSPI_PARAM_OVERFLOW = 0x20
    ,E_MSPI_MMIO_ERROR = 0x40
    ,E_MSPI_HW_NOT_SUPPORT = 0x80
    ,E_MSPI_NULL
} MSPI_ErrorNo;

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Description : MSPI initial
/// @return void:
//------------------------------------------------------------------------------
void HAL_MSPI_Init(MSPI_CH eChannel);
void HAL_MSPI_IntEnable(MSPI_CH eChannel,MS_BOOL bEnable);
void HAL_MSPI_SetChipSelect(MSPI_CH eChannel, MS_BOOL Enable, MSPI_ChipSelect_e eCS);
MS_BOOL HAL_MSPI_Read(MSPI_CH eChannel, MS_U8 *pData, MS_U16 u16Size);
MS_BOOL HAL_MSPI_Write(MSPI_CH eChannel, MS_U8 *pData, MS_U16 u16Size);
MS_BOOL HAL_MSPI_SetReadBufferSize(MSPI_CH eChannel,  MS_U8 u8Size);
MS_BOOL HAL_MSPI_SetWriteBufferSize(MSPI_CH eChannel,  MS_U8 u8Size);
MS_U8 HAL_MSPI_Read_Write(MSPI_CH eChannel,  MS_U8 *pReadData,U8 *pWriteData, MS_U8 u8WriteSize);
MS_BOOL HAL_MSPI_Reset_DCConfig(MSPI_CH eChannel);
MS_BOOL HAL_MSPI_Reset_FrameConfig(MSPI_CH eChannel);
MS_BOOL HAL_MSPI_Reset_Ch_CLKConfig(MSPI_CH eChannel);
MS_BOOL HAL_MSPI_Trigger(void);
void HAL_MSPI_SlaveEnable(MSPI_CH eChannel, MS_BOOL Enable);
void HAL_MSPI_SetDcTiming (MSPI_CH eChannel, eDC_config eDCField, MS_U8 u8DCtiming);
void HAL_MSPI_SetCLKTiming(MSPI_CH eChannel, eCLK_config eCLKField, MS_U8 u8CLKVal);
MS_BOOL HAL_MSPI_LD_CLK_Config(MS_U8 u8Chanel,MS_U32 u32MspiClk);

void HAL_MSPI_SetPerFrameSize(MSPI_CH eChannel, MS_BOOL bDirect, MS_U8 u8BufOffset, MS_U8 u8PerFrameSize);
MS_BOOL HAL_MSPI_CLOCK_Config(MSPI_CH eChannel, MS_U32 u32MaxClock);
void HAL_MSPI_SetCLKTime(eCLK_config eCLKField, MS_U8 u8CLKVal);
MS_BOOL HAL_MSPI_Reset_CLKConfig(void);
MS_U8 HAL_MSPI_CLKConfigMax(eCLK_config eCLKField);
MS_BOOL HAL_MSPI_RWBytes(MS_BOOL Direct, MS_U8 u8Bytes);
#endif

