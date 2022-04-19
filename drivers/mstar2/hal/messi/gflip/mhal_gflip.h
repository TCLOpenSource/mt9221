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
/// @file   mhal_gflip.h
/// @brief  MStar GFLIP Driver DDI HAL Level
/// @author MStar Semiconductor Inc.
/// @attention
/// <b>(OBSOLETED) <em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _HAL_GFLIP_H
#define _HAL_GFLIP_H

#ifdef _HAL_GFLIP_C
#define INTERFACE
#else
#define INTERFACE                               extern
#endif

//=============================================================================
// Includs
//=============================================================================

//=============================================================================
// Defines & Macros
//=============================================================================
#define MAX_GOP_SUPPORT                         3
#define MAX_GOP0_GWIN                           2
#define MAX_GOP1_GWIN                           1
#define MAX_GOP2_GWIN                           1
#define MAX_GOP3_GWIN                           0
#define MAX_GOP4_GWIN                           0
#define MAX_GOP_GWIN                            MAX_GOP0_GWIN

#define GFLIP_GOP_BANKOFFSET                    (0x3)
#define GFLIP_ADDR_ALIGN_RSHIFT                 (4)

#define GFLIP_GOP_IDX_2G                        (0x0)
#define GFLIP_GOP_IDX_2GX                       (0x1)
#define GFLIP_GOP_IDX_1G                        (0x2)
#define GFLIP_GOP_IDX_1GX                       (0x3)
#define GFLIP_GOP_IDX_DWIN                      (0x4)
#define GFLIP_GOP_IDX_INVALID                   (0xFFFFFFFF)

#define GFLIP_GOP_DWIN_BANKID                   0xC

#define GFLIP_GOP_BANK_IDX_0                    (0x0)
#define GFLIP_GOP_BANK_IDX_1                    (0x1)
#define GFLIP_GOP_BANK_IDX_2                    (0x2)

#define GFLIP_GOP_DST_IPSUB                     0x0
#define GFLIP_GOP_DST_IPMAIN                    0x1
#define GFLIP_GOP_DST_OP0                       0x2
#define GFLIP_GOP_DST_VOP                       0x3
#define GFLIP_GOP_DST_VOPSUB                    0x4

#define GFLIP_MULTI_FLIP                        1

#define GOP0_REG_FORM                           ((E_GOP_REG_FORM_2G) + (E_GOP_PAL_SIZE_256))
#define GOP1_REG_FORM                           ((E_GOP_REG_FORM_2G) + (E_GOP_PAL_SIZE_256))
#define GOP2_REG_FORM                           ((E_GOP_REG_FORM_T81G) + (E_GOP_PAL_SIZE_NONE))
#define GOP3_REG_FORM                           (E_GOP_REG_FORM_NONE)
#define GOP4_REG_FORM                           (E_GOP_REG_FORM_NONE)
#define GOP5_REG_FORM                           (E_GOP_REG_FORM_NONE)

#define GS_REG_RESTORE_FUNCTION


#define GOP_TLB_PAGE_SIZE                      0x1000
#define PER_MIU_TLB_ENTRY_COUNT                8
#define TLB_PER_ENTRY_SIZE                     4
#define ADDRESSING_8BYTE_UNIT                  8

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_GOP0 = 0,
    E_GOP1 = 1,
    E_GOP2 = 2,
    E_GOP_Dwin = 3,
    E_GOP_MIXER = 4,
    E_GOP3 = 5,
    E_GOP4 = 6,
    E_GOP5 = 7,
}EN_GOP_TYPE;

//=============================================================================
// HAL Driver Function
//=============================================================================
#define MHal_GFLIP_SetHDRCentralInfo(args...) (FALSE)

INTERFACE MS_BOOL   MHal_GFLIP_IntEnable(MS_U32 u32GopIdx, MS_BOOL bEnable);
INTERFACE MS_BOOL   MHal_GFLIP_IsVSyncInt(MS_U32 u32GopIdx);
INTERFACE MS_U32    MHal_GFLIP_GetIntGopIdx(void);
INTERFACE MS_BOOL MHal_GFLIP_IsGOPACK(MS_U32 u32GopIdx);
INTERFACE void      MHal_GFLIP_Fire(MS_U32 u32GopBitMask);
INTERFACE MS_BOOL MHal_GFLIP_SetFlipToGop(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_BOOL bForceWriteIn);
#ifdef GFLIP_GOP_TLB
INTERFACE MS_BOOL MHal_GFLIP_SetTLBFlipToGop(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_BOOL bForceWriteIn);
#endif
#ifdef	GFLIP_MULTI_FLIP
INTERFACE MS_BOOL MHal_GFLIP_SetMultiFlipToGop(MS_GFLIP_MULTIINFO* pMultiFlipInfo, MS_BOOL bForceWriteIn);
#ifdef GFLIP_GOP_TLB
INTERFACE MS_BOOL MHal_GFLIP_SetTLBMultiFlipToGop(MS_TLB_GFLIP_MULTIINFO* pTLBMultiFlipInfo,MS_BOOL bForceWriteIn);
#endif
INTERFACE void MHal_GFLIP_SetMultiFire(MS_U32 u32GopBitMask,MS_BOOL bForceWriteIn);
#endif
INTERFACE MS_BOOL   MHal_GFLIP_ClearDWINIRQ(GFLIP_DWININT_INFO *pGFlipDWinIntInfo);

INTERFACE MS_BOOL   MHal_GFLIP_HandleVsyncLimitation(MS_U32 u32GopIdx);
INTERFACE MS_BOOL   MHal_GFLIP_RestoreFromVsyncLimitation(MS_U32 u32GopIdx);

INTERFACE MS_BOOL   MHal_GFLIP_IsTagIDBack(MS_U16 u16TagId);
INTERFACE MS_BOOL   MHal_GFLIP_VECaptureEnable(MS_BOOL bEnable);
INTERFACE MS_BOOL   MHal_GFLIP_CheckVEReady(void);
INTERFACE MS_U16    MHal_GFLIP_GetGopDst(MS_U32 u32GopIdx);
INTERFACE MS_U8     MHal_GFLIP_GetFrameIdx(void);
INTERFACE MS_U32    MHal_GFLIP_GetValidGWinIDPerGOPIdx(MS_U32 u32GopIdx, MS_U32 u32GwinIdx);
INTERFACE MS_U8     MHal_GFLIP_GetBankOffset(MS_U32 u32GopIdx, MS_U16 u16BankIdx);
INTERFACE void MHal_GFLIP_WriteGopReg(MS_U32 u32GopIdx, MS_U16 u16BankIdx, MS_U16 u16Addr, MS_U16 u16Val, MS_U16 u16Mask);
INTERFACE void MHal_GFLIP_ReadGopReg(MS_U32 u32GopIdx, MS_U16 u16BankIdx, MS_U16 u16Addr, MS_U16* u16Val);
INTERFACE MS_BOOL MHAL_GFLIP_SetOpmuxInfo(MS_SDR2HDR_INFO* pstSDR2HDRInfo);


#undef INTERFACE
#endif //_HAL_GFLIP_H

