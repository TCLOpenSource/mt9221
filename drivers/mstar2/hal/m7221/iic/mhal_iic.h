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

#ifndef _HAL_IIC_H_
#define _HAL_IIC_H_

#include <asm/types.h>
#include "mdrv_types.h"

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define LOCK_HW_SEM()   //{\
                            MDrv_SEM_Lock(E_SEM_IIC, SEM_WAIT_FOREVER); \
                      }
#define UNLOCK_HW_SEM() //{\
                            MDrv_SEM_Unlock(E_SEM_IIC); \
                                          }

#define HWI2C_SET_RW_BIT(bRead, val) ((bRead) ? ((val) | __BIT0) : ((val) & ~__BIT0))
#define __BIT0                          (0x01)
#define __BIT1                          (0x02)
#define __BIT2                          (0x04)
#define __BIT3                          (0x08)
#define __BIT4                          (0x10)
#define __BIT5                          (0x20)
#define __BIT6                          (0x40)
#define __BIT7                          (0x80)
//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
typedef enum _HAL_HWI2C_STATE
{
    E_HAL_HWI2C_STATE_IDEL = 0,
    E_HAL_HWI2C_STATE_START,
    E_HAL_HWI2C_STATE_WRITE,
    E_HAL_HWI2C_STATE_READ,
    E_HAL_HWI2C_STATE_INT,
    E_HAL_HWI2C_STATE_WAIT,
    E_HAL_HWI2C_STATE_STOP
}
HAL_HWI2C_STATE;

#define HAL_HWI2C_PORTS         4
#define HAL_HWI2C_PORT0         0
#define HAL_HWI2C_PORT1         1
#define HAL_HWI2C_PORT2         2
#define HAL_HWI2C_PORT3         3

typedef enum _HAL_HWI2C_PORT
{
    E_HAL_HWI2C_PORT0_0 = 0, //disable port 0
    E_HAL_HWI2C_PORT0_1,
    E_HAL_HWI2C_PORT0_2,
    E_HAL_HWI2C_PORT0_3,
    E_HAL_HWI2C_PORT0_4,
    E_HAL_HWI2C_PORT0_5,
    E_HAL_HWI2C_PORT0_6,
    E_HAL_HWI2C_PORT0_7,

    E_HAL_HWI2C_PORT1_0,  //disable port 1
    E_HAL_HWI2C_PORT1_1,
    E_HAL_HWI2C_PORT1_2,
    E_HAL_HWI2C_PORT1_3,
    E_HAL_HWI2C_PORT1_4,
    E_HAL_HWI2C_PORT1_5,
    E_HAL_HWI2C_PORT1_6,
    E_HAL_HWI2C_PORT1_7,

    E_HAL_HWI2C_PORT2_0,  //disable port 2
    E_HAL_HWI2C_PORT2_1,
    E_HAL_HWI2C_PORT2_2,
    E_HAL_HWI2C_PORT2_3,
    E_HAL_HWI2C_PORT2_4,
    E_HAL_HWI2C_PORT2_5,
    E_HAL_HWI2C_PORT2_6,
    E_HAL_HWI2C_PORT2_7,

    E_HAL_HWI2C_PORT3_0, //disable port 3
    E_HAL_HWI2C_PORT3_1,
    E_HAL_HWI2C_PORT3_2,
    E_HAL_HWI2C_PORT3_3,
    E_HAL_HWI2C_PORT3_4,
    E_HAL_HWI2C_PORT3_5,
    E_HAL_HWI2C_PORT3_6,
    E_HAL_HWI2C_PORT3_7,

    E_HAL_HWI2C_PORT_NOSUP
}
HAL_HWI2C_PORT;

typedef enum _HAL_HWI2C_CLKSEL
{
    E_HAL_HWI2C_CLKSEL_HIGH = 0,
    E_HAL_HWI2C_CLKSEL_NORMAL,
    E_HAL_HWI2C_CLKSEL_SLOW,
    E_HAL_HWI2C_CLKSEL_VSLOW,
    E_HAL_HWI2C_CLKSEL_USLOW,
    E_HAL_HWI2C_CLKSEL_UVSLOW,
    E_HAL_HWI2C_CLKSEL_NOSUP
}
HAL_HWI2C_CLKSEL;

typedef enum _HAL_HWI2C_CLK
{
    E_HAL_HWI2C_CLK_DIV4 = 1, //750K@12MHz
    E_HAL_HWI2C_CLK_DIV8,     //375K@12MHz
    E_HAL_HWI2C_CLK_DIV16,    //187.5K@12MHz
    E_HAL_HWI2C_CLK_DIV32,    //93.75K@12MHz
    E_HAL_HWI2C_CLK_DIV64,    //46.875K@12MHz
    E_HAL_HWI2C_CLK_DIV128,   //23.4375K@12MHz
    E_HAL_HWI2C_CLK_DIV256,   //11.71875K@12MHz
    E_HAL_HWI2C_CLK_DIV512,   //5.859375K@12MHz
    E_HAL_HWI2C_CLK_DIV1024,  //2.9296875K@12MHz
    E_HAL_HWI2C_CLK_NOSUP
}
HAL_HWI2C_CLK;

typedef enum {
    E_HAL_HWI2C_READ_MODE_DIRECT,                       ///< first transmit slave address + reg address and then start receive the data */
    E_HAL_HWI2C_READ_MODE_DIRECTION_CHANGE,             ///< slave address + reg address in write mode, direction change to read mode, repeat start slave address in read mode, data from device
    E_HAL_HWI2C_READ_MODE_DIRECTION_CHANGE_STOP_START,  ///< slave address + reg address in write mode + stop, direction change to read mode, repeat start slave address in read mode, data from device
    E_HAL_HWI2C_READ_MODE_MAX
}
HAL_HWI2C_ReadMode;

typedef enum _HAL_HWI2C_DMA_ADDRMODE
{
    E_HAL_HWI2C_DMA_ADDR_NORMAL = 0,
    E_HAL_HWI2C_DMA_ADDR_10BIT,
    E_HAL_HWI2C_DMA_ADDR_MAX,
}
HAL_HWI2C_DMA_ADDRMODE;

typedef enum _HAL_HWI2C_DMA_MIUPRI
{
    E_HAL_HWI2C_DMA_PRI_LOW = 0,
    E_HAL_HWI2C_DMA_PRI_HIGH,
    E_HAL_HWI2C_DMA_PRI_MAX,
}
HAL_HWI2C_DMA_MIUPRI;

typedef enum _HAL_HWI2C_DMA_MIUCH
{
    E_HAL_HWI2C_DMA_MIU_CH0 = 0,
    E_HAL_HWI2C_DMA_MIU_CH1,
    E_HAL_HWI2C_DMA_MIU_MAX,
}
HAL_HWI2C_DMA_MIUCH;
//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
extern void MHal_IIC_STR_Record(void);
extern void MHal_IIC_Init(void);
extern void MHal_IIC_Clock_Select(U8 u8ClockIIC);
extern BOOL MHal_IIC_Start(U32 u32PortOffset);
extern BOOL MHal_IIC_Stop(U32 u32PortOffset);
extern U8 MHal_IIC_GetState(U32 u32PortBaseReg);
extern BOOL MHal_HWI2C_NoAck(U32 u32PortOffset);
extern U16 MHal_IIC_SendData(U32 u32PortOffset ,U8 u8DataIIC);
extern U16 MHal_IIC_SendByte(U32 u32PortOffset ,U8 u8DataIIC);
extern U16 MHal_IIC_GetByte(U32 u32PortOffset, U8* pu8DataIIC);
extern BOOL MHal_HWI2C_SetClk(U32 u32PortOffset, U32 eClkSel);
extern BOOL MHal_HWI2C_Master_Enable(U32 u32PortOffset);
extern BOOL MHal_HWI2C_SelectPort(HAL_HWI2C_PORT ePort);
extern U32 MHal_Query_MIPS_CLK(void);
extern BOOL MHal_HWI2C_GetBaseRegByPort(HAL_HWI2C_PORT ePort, U32* pu32PortBaseReg);
extern BOOL MHal_HWI2C_GetPortIdxByPort(HAL_HWI2C_PORT ePort, U8* pu8Port);
extern BOOL MHal_HWI2C_GetPortIdxByPortBaseReg(U32 u32PortBaseReg, U8* pu8Port);
extern BOOL MHal_HWI2C_Reset_ALL(U32 u32PortBaseReg, BOOL bReset);
#endif // _HAL_IIC_H_

