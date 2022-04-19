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
///
/// @file   mdrv_iic.h
/// @brief  IIC Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _MDRV_IIC_IO_H_
#define _MDRV_IIC_IO_H_

#include <asm/types.h>//<asm-mips/types.h>
#include "mdrv_types.h"
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
struct IIC_Param
{
    u8 u8IICType;       /// IICType: 0:SWI2C 1:HWI2C
    U8 u8IdIIC;      	/// IIC ID: Channel 1~7
    U8 u8ClockIIC;   	/// IIC clock speed
    U8 u8SlaveIdIIC;    /// Device slave ID
    U8 u8AddrSizeIIC;	/// Address length in bytes
    U8 *u8AddrIIC;      /// Starting address inside the device
    U8 *u8pbufIIC;     	/// buffer
    U32 u32DataSizeIIC;	/// size of buffer

} __attribute__ ((packed));

#if defined(CONFIG_COMPAT)
struct IIC_Param_Compat
{
    u8 u8IICType;       /// IICType: 0:SWI2C 1:HWI2C
    U8 u8IdIIC;      	/// IIC ID: Channel 1~7
    U8 u8ClockIIC;   	/// IIC clock speed
    U8 u8SlaveIdIIC;    /// Device slave ID
    U8 u8AddrSizeIIC;	/// Address length in bytes
    compat_uptr_t u8AddrIIC;    /// Starting address inside the device
    compat_uptr_t u8pbufIIC;     	/// buffer //(4) not using pointer
    U32 u32DataSizeIIC;	/// size of buffer

} __attribute__ ((packed));

typedef struct IIC_Param_Compat  IIC_Param_Compat;
#endif
typedef struct IIC_Param  IIC_Param;

typedef struct IIC_Param IIC_Param_t;

typedef struct{
    u8 u8ChIdx;         ///Channel index
    u8 u8Enable;        ///Enable
    u16 u16Retries;
    u16 u16PadSCL;      ///Pad(Gpio) number for SCL
    u16 u16PadSDA;      ///Pad(Gpio) number for SDA
    u8 u8Hw_Port;
    u32 u32PadMux;
    u16 u16SpeedKHz;    ///Speed in KHz
    u16 u16DefDelay;
} IIC_BusCfg_t;


//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define IIC_IOC_MAGIC               'u'

#if defined(CONFIG_COMPAT)
#define Compat_MDRV_IIC_INIT               _IO(IIC_IOC_MAGIC, 0)
#define Compat_MDRV_IIC_CLOCK              _IOW(IIC_IOC_MAGIC, 2, IIC_Param_Compat)
#define Compat_MDRV_IIC_ENABLE             _IOW(IIC_IOC_MAGIC, 3, IIC_Param_Compat) 
#define Compat_MDRV_IIC_BUSCFG             _IOW(IIC_IOC_MAGIC, 4, IIC_BusCfg_t)
#endif

#define MDRV_IIC_INIT               _IO(IIC_IOC_MAGIC, 0)
#define MDRV_IIC_SET_PARAM          _IOW(IIC_IOC_MAGIC, 1, IIC_Param_t)
#define MDRV_IIC_CLOCK              _IOW(IIC_IOC_MAGIC, 2, IIC_Param_t)
#define MDRV_IIC_ENABLE             _IOW(IIC_IOC_MAGIC, 3, IIC_Param_t) 
#define MDRV_IIC_BUSCFG             _IOW(IIC_IOC_MAGIC, 4, IIC_BusCfg_t)
#define IIC_IOC_MAXNR               5

#define IIC_MAX_BUF_SIZE            1024
#define IIC_MAX_ADDR_SIZE           128

#define IIC_WR_ADDR_SIZE            8
#define IIC_WR_BUF_SIZE             128		// added for RGB EDID
#define IIC_RD_BUF_SIZE             256		// added for RGB EDID

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
// for software IIC
void MDrv_SW_IIC_SetSpeed(u8 u8ChIIC, u8 u8Speed);
void MDrv_SW_IIC_Init(IIC_BusCfg_t I2CBusCfg[], u8 u8CfgBusNum);
void MDrv_SW_IIC_Init_Bus(IIC_BusCfg_t I2CBusCfg[], u8 u8Bus);
void MDrv_SW_IIC_Init_Setup(void);
void MDrv_SW_IIC_Enable( u8 u8ChIIC, B16 bEnable );
int MDrv_SW_IIC_WriteBytes(u8 u8BusNum, u8 u8SlaveID, u8 u8addrcount, u8* pu8addr, u16 u16size, u8* pu8data);
int MDrv_SW_IIC_WriteBytesStop(u8 u8BusNum, u8 u8SlaveID,u8 AddrCnt, u8* pu8addr, u16 u16size, u8* pBuf, int bGenStop);
int MDrv_SW_IIC_ReadBytes(u8 u8BusNum, u8 u8SlaveID, u8 u8AddrNum, u8* paddr, u16 u16size, u8* pu8data);
int MDrv_SW_IIC_ReadByte(u8 u8BusNum, u8 u8SlaveID, u8 u8RegAddr, u8 *pu8Data);
int MDrv_SW_IIC_WriteByte(u8 u8BusNum, u8 u8SlaveID, u8 u8RegAddr, u8 u8Data);
int MDrv_SW_IIC_Write2Bytes(u8 u8BusNum, u8 u8SlaveID, u8 u8addr, u16 u16data);
u16 MDrv_SW_IIC_Read2Bytes(u8 u8BusNum, u8 u8SlaveID, u8 u8addr);
int MDrv_SW_IIC_Write4Bytes(u8 u8BusNum, u8 u8SlaveID, u32 u32Data, u8 u8EndData);
int MDrv_SW_IIC_WriteByteDirectly(u8 u8BusNum, u8 u8SlaveID, u8 u8Data);
// for hardware IIC
void MDrv_IIC_Init(void);
void MDrv_HW_IIC_Clock_Select(U8 u8ClockIIC);
S32 MDrv_HW_IIC_WriteBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);
S32 MDrv_HW_IIC_ReadBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);
void MDrv_HW_IIC_Init(IIC_BusCfg_t I2CBusCfg[], u8 u8CfgBusNum);
void MDrv_HW_IIC_Resume(void);
#if (defined(CONFIG_MSTAR_TITANIA)||defined(CONFIG_MSTAR_TITANIA2))
#else
B16 MDrv_SW_IIC_ConfigBus(IIC_BusCfg_t* pBusCfg);
int MDrv_SW_IIC_GetSDA(U8 u8ChIIC);
int MDrv_SW_IIC_GetSCL(U8 u8ChIIC);
#endif
#endif
