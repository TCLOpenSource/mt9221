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
/// @file   halBDMA.h
/// @brief  MStar BDMA HAL DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MHAL_BDMA_H
#define _MHAL_BDMA_H

//=============================================================================
// Includs
//=============================================================================
#include "mosWrapper.h"


//typedef enum _BDMA_Dev
//{
//    E_BDMA_DEV_MIU0
//    ,E_BDMA_DEV_MIU1
//    ,E_BDMA_DEV_SEARCH
//    ,E_BDMA_DEV_CRC32
//    ,E_BDMA_DEV_MEM_FILL
//    ,E_BDMA_DEV_FLASH
//    ,E_BDMA_DEV_DMDMCU
//    ,E_BDMA_DEV_VDMCU
//    ,E_BDMA_DEV_DSP
//    ,E_BDMA_DEV_TSP
//    ,E_BDMA_DEV_1KSRAM_HK51
//    ,E_BDMA_DEV_NOT_SUPPORT
//}BDMA_Dev;
//
//typedef enum _BDMA_SrcDev
//{
//    E_BDMA_SRCDEV_MIU0          = E_BDMA_DEV_MIU0
//    ,E_BDMA_SRCDEV_MIU1         = E_BDMA_DEV_MIU1
//    ,E_BDMA_SRCDEV_MEM_FILL     = E_BDMA_DEV_MEM_FILL
//    ,E_BDMA_SRCDEV_FLASH        = E_BDMA_DEV_FLASH
//    ,E_BDMA_SRCDEV_NOT_SUPPORT  = E_BDMA_DEV_NOT_SUPPORT
//}BDMA_SrcDev;
//
//typedef enum _BDMA_DstDev
//{
//    E_BDMA_DSTDEV_MIU0          = E_BDMA_DEV_MIU0
//    ,E_BDMA_DSTDEV_MIU1         = E_BDMA_DEV_MIU1
//    ,E_BDMA_DSTDEV_SEARCH       = E_BDMA_DEV_SEARCH
//    ,E_BDMA_DSTDEV_CRC32        = E_BDMA_DEV_CRC32
//    ,E_BDMA_DSTDEV_DMDMCU       = E_BDMA_DEV_DMDMCU         //Demod
//    ,E_BDMA_DSTDEV_VDMCU        = E_BDMA_DEV_VDMCU          //VD
//    ,E_BDMA_DSTDEV_DSP          = E_BDMA_DEV_DSP
//    ,E_BDMA_DSTDEV_TSP          = E_BDMA_DEV_TSP
//    ,E_BDMA_DSTDEV_HK51_1KSRAM  = E_BDMA_DEV_1KSRAM_HK51
//    ,E_BDMA_DSTDEV_NOT_SUPPORT  = E_BDMA_DEV_NOT_SUPPORT
//}BDMA_DstDev;

typedef enum _BDMA_Ch
{
    E_BDMA_CH0,
    E_BDMA_CH1
}BDMA_Ch;

typedef struct _BDMA_ChStatus
{
    BOOL bIsBusy;
    BOOL bIsInt;
    BOOL bIsFound;
}BDMA_ChStatus;


BOOL MHal_BDMA_WaitDone(BDMA_Ch channel,U32 u32Timeoutmsec);
BOOL MHal_BDMA_PollingDone(BDMA_Ch channel);
BOOL MHal_BDMA_FlashToMem(BDMA_Ch channel, U32 u32FlashAddr, U32 u32DramAddr, U32 u32Len);
BOOL MHal_BDMA_CalculateCRC32(BDMA_Ch channel, U32 u32DramAddr, U32 u32Len);
U32 MHal_BDMA_GetCRC32(BDMA_Ch channel);
BOOL MHal_BDMA_MemToMem(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len);
BOOL MHal_BDMA_IMItoIMI(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len);
BOOL MHal_BDMA_IMItoMIU(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len);
BOOL MHal_BDMA_MIUtoIMI(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len);
BOOL MHal_BDMA_MIUtoMIU(BDMA_Ch channel, U32 u32Src, U32 u32Dst, U32 u32Len);

BOOL MHal_BDMA_CalculateCRC32FromMIU(BDMA_Ch channel, U32 u32DramAddr, U32 u32Len);
BOOL MHal_BDMA_CalculateCRC32FromIMI(BDMA_Ch channel, U32 u32SramAddr, U32 u32Len);
BOOL MHal_BDMA_PatternSearchFromIMI(BDMA_Ch channel, U32 u32SramAddr, U32 u32Len, U32 u32Pattern);
BOOL MHal_BDMA_IsBusy(BDMA_Ch channel);
BOOL MHal_BDMA_ClearStatus(BDMA_Ch channel);
void MHal_BDMA_Init(void);
#endif
