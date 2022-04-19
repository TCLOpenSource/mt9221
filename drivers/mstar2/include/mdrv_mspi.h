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
/// file    mdrv_mspi.h
/// @brief MSPI drv Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_MSPI_H_
#define _MDRV_MSPI_H_


#include "mdrv_mstypes.h"
#include "mhal_mspi.h"
#include "reg_mspi.h"

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------



MS_BOOL MDrv_MSPI_SetDbgLevel(MS_U8 u8DbgLevel);

MSPI_ErrorNo MDrv_MSPI_Init(MSPI_CH eChannel);
MSPI_ErrorNo MDrv_MSPI_Read(MSPI_CH eChannel, MS_U8 *pData, MS_U32 u32Size);
MSPI_ErrorNo MDrv_MSPI_Write(MSPI_CH eChannel, MS_U8 *pData, MS_U32 u32Size);
MSPI_ErrorNo MDrv_MSPI_SetReadBufferSize(MSPI_CH eChannel,  MS_U8 u8Size);
MSPI_ErrorNo MDrv_MSPI_SetWriteBufferSize(MSPI_CH eChannel,  MS_U8 u8Size);
MS_U32  MDrv_MSPI_Read_Write(MSPI_CH eChannel,MS_U8 *pReadData,MS_U8 *pWriteData, MS_U32 u32Size);
MSPI_ErrorNo MDrv_MSPI_DCConfig(MSPI_CH eChannel, MSPI_DCConfig *ptDCConfig);
MSPI_ErrorNo MDrv_MSPI_SetMode(MSPI_CH eChannel, MSPI_Mode_Config_e eMode);
MSPI_ErrorNo MDrv_MSPI_SetCLK(MSPI_CH eChannel, MS_U8 U8Clock);
MSPI_ErrorNo MDrv_MSPI_SetCLKByINI(MSPI_CH eChannel, MS_U32 u32Clock);
MSPI_ErrorNo MDrv_MSPI_FRAMEConfig(MSPI_CH eChannel, MSPI_FrameConfig *ptFrameConfig);
//void MDrv_MSPI_SlaveEnable(MSPI_CH eChannel, MS_BOOL Enable,MSPI_ChipSelect_e eCS);
MSPI_ErrorNo MDrv_MSPI_CLKConfig(MSPI_CLKConfig *ptCLKConfig);
MSPI_ErrorNo MDrv_MSPI_Init_Ext(MSPI_CH eChannel);
MS_U32 MDrv_MSPI_SetPowerState(void);
MSPI_ErrorNo MDrv_MSPI_RWBytes(MS_BOOL Direct, MS_U8 u8Bytes);
void MDrv_MSPI_SlaveEnable_Channel(MS_U8 u8ch,MS_BOOL Enable);
MSPI_ErrorNo MDrv_MSPI_Read_Channel(MS_U8 u8ch,MS_U8 *pData, MS_U16 u16Size);
MSPI_ErrorNo MDrv_MSPI_Write_Channel(MS_U8 u8ch,MS_U8 *pData, MS_U16 u16Size);
#endif
