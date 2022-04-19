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
/// @file   mdrv_gflip.h
/// @brief  MediaTek gflip Interface header file
/// @author MediaTek Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_GFLIP_H
#define _MDRV_GFLIP_H
#include "mdrv_gflip_ve_st.h"
#include "mdrv_gflip_st.h"

//CFD header
#include "mdrv_gflip_cfd.h"

#ifdef _MDRV_GFLIP_C
#define INTERFACE
#else
#define INTERFACE extern
#endif

#if defined(__cplusplus)
extern "C" {
#endif

//=============================================================================
// Defines & Macros
//=============================================================================
 //Default enable for T12/J2/, For other IC, please use "make menuconfig" to enable it
#if(defined(CONFIG_MSTAR_TITANIA12) || defined(CONFIG_MSTAR_JANUS2))
#ifndef CONFIG_MSTAR_VE_CAPTURE_SUPPORT
#define CONFIG_MSTAR_VE_CAPTURE_SUPPORT 1
#endif
#endif

#ifndef SYMBOL_WEAK
#define SYMBOL_WEAK __attribute__((weak))
#endif
#ifndef MIN
#define MIN(_a_, _b_)               ((_a_) < (_b_) ? (_a_) : (_b_))
#endif

#define INVAILD_GOP_NUM                 (0xFFUL)
#define PALETTE_SIZE                    (0x100UL)
//=============================================================================
// Type and Structure Declaration
//=============================================================================
typedef struct
{
    MS_PHY64 u32MainAddr; //in
    MS_PHY64 u32SubAddr; //in
    MS_U32 u32TagId; //in
    MS_BOOL bKickOff;// if base address has been set to HW register(may not take effect because of Vs0 not arising)
    MS_U32 u32KickOffStartTime;
    MS_U32 *pdata;
}GFLIP_INFO, *PGFLIP_INFO;

typedef enum
{
    E_GOP_REG_FORM_4G   = 0x0000,
    E_GOP_REG_FORM_T21G = 0x0001,
    E_GOP_REG_FORM_2G   = 0x0002,
    E_GOP_REG_FORM_T81G = 0x0003,
    E_GOP_REG_FORM_U42G = 0x0004,
    E_GOP_REG_FORM_NONE = 0x000F,
    E_GOP_REG_FORM_MASK = 0x000F,
}   EN_GOP_REG_FORM;

// GOP palette type
typedef enum
{
    E_GOP_PAL_SIZE_NONE = 0x0000,
    E_GOP_PAL_SIZE_256  = 0x0100,
    E_GOP_PAL_SIZE_64   = 0x0200,
    E_GOP_PAL_SIZE_MASK = 0x0F00,
}   EN_GOP_PAL_SIZE;

typedef struct gflip_getvsync_data {
    MS_U32 vsync_gop;
    MS_S64 vsync_ts;
}   GFLIP_GETVSYNC_DATA, *PGFLIP_GETVSYNC_DATA;

typedef void (*FP_CFD_GOP_PreProcess)(const STU_CFDAPI_TOP_CONTROL_GOP * pstu_Cfd_api_top, STU_CFDAPI_GOP_PREPROCESS_OUT * pst_Cfd_api_out);
typedef void (*FP_CFD_GOP_PreSDR)(const ST_CFD_GOP_PRESDR_INPUT *pst_Cfd_presdr_input, ST_CFD_GOP_PreSDR_HW_OUTPUT *pst_cfd_presdr_hw_output);
typedef void (*FP_CFD_GOP_TestMode)(STU_CFDAPI_TOP_CONTROL_GOP *pstu_CfdAPI_Top_Param, STU_CFDAPI_TOP_CONTROL_GOP_TESTING *pu8TestCase);

#define CB_MAPI_CFD_GOP_PREPROCESS     0 //MDrv_GFLIP_Register_CB type
#define CB_MAPI_CFD_GOP_PRESDR         1 //MDrv_GFLIP_Register_CB type
#define CB_MApi_DebugCFD_GOP_TestMode  2 //MDrv_GFLIP_Register_CB type
//=============================================================================
// Variable
//=============================================================================
INTERFACE spinlock_t    spinlock_gflip;

//=============================================================================
// Function
//=============================================================================

//Init related:
INTERFACE MS_BOOL MDrv_GFLIP_Init(MS_U32 u32GopIdx);
INTERFACE MS_BOOL MDrv_GFLIP_DeInit(MS_U32 u32GopIdx);

//Irq Relaated(hw process):
INTERFACE MS_BOOL MDrv_GFLIP_ClearIRQ(void);
INTERFACE MS_BOOL MDrv_GFLIP_ProcessIRQ(void);
INTERFACE MS_BOOL MDrv_GFLIP_VEC_ProcessIRQ(void);
INTERFACE MS_BOOL MDrv_GFLIP_VEC_ClearIRQ(void);

//Handle Vsync. Limitation related:
INTERFACE MS_BOOL _MDrv_GFLIP_RestoreFromVsyncLimitation(void);

//Drv Interface related(drv interface):
INTERFACE MS_BOOL _MDrv_GFLIP_SetFlipInfo(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result);
INTERFACE MS_BOOL MDrv_GFLIP_SetTLBFlipInfo(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32MainAddr, MS_PHY64 u32SubAddr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result, MS_BOOL bTLBenable, MS_PHY64 u32TLBAddr);
//#ifdef	GFLIP_MULTI_FLIP
INTERFACE MS_BOOL _MDrv_GFLIP_SetMultiFlipInfo(MS_GFLIP_MULTIINFO* pMultiFlipInfo);
INTERFACE MS_BOOL _MDrv_GFLIP_SetTLBMultiFlipInfo(MS_TLB_GFLIP_MULTIINFO* pTLBMultiFlipInfo);
//#endif
INTERFACE MS_BOOL _MDrv_GFLIP_GetDWinIntInfo(GFLIP_DWININT_INFO *pGFlipDWinIntInfo, MS_BOOL bResetDWinIntInfo,MS_U32 u32Timeout);

INTERFACE MS_BOOL MDrv_GFLIP_SetPixelIDAddr(MS_U32 u32GopIdx, MS_U32 u32GwinIdx, MS_PHY64 u32Addr, MS_U32 u32TagId, MS_U32 * u32QEntry, MS_U32 *u32Result);

INTERFACE MS_BOOL _MDrv_GFLIP_SetGPIO3DPin(MS_PHY64 u32Addr, MS_U32 *u32Result);
INTERFACE MS_BOOL MDrv_GFLIP_SetVECapCurState(MS_BOOL *pbEna);
INTERFACE MS_BOOL MDrv_GFLIP_GetVECapCurState(MS_BOOL *pbEna, MS_U8 *pu8FramCount);
INTERFACE MS_BOOL MDrv_GFLIP_VECapWaitOnFrame(MS_BOOL *pbEna, MS_U8 *pu8FramNum);
INTERFACE void MDrv_GFLIP_SetVECaptureConfig(PMS_GFLIP_VEC_CONFIG pstGflipVECConfig);
INTERFACE void MDrv_GFLIP_GetVECaptureConfig(PMS_GFLIP_VEC_CONFIG pstGflipVECConfig);
INTERFACE MS_BOOL _MDrv_GFLIP_ClearFlipQueue(MS_U32 u32GopIdx,MS_U32 u32GwinIdx);
INTERFACE MS_BOOL MDrv_GFLIP_WaitForVsync(MS_U32 u32GopIdx);
INTERFACE MS_S64 MDrv_GFLIP_WaitForVsync_EX(MS_U32 u32GopIdx);
INTERFACE MS_BOOL MDrv_GFLIP_GetVsync(MS_U32 u32GopIdx);
INTERFACE MS_BOOL _MDrv_GFLIP_SetGwinInfo(MS_GWIN_INFO stGwinInfo);
INTERFACE int MDrv_GFLIP_Suspend(void);
INTERFACE int MDrv_GFLIP_Resume(void);
INTERFACE SYMBOL_WEAK MS_BOOL MDrv_GFLIP_Register_CB(MS_U8 u8Type,void *fpCB);
INTERFACE SYMBOL_WEAK MS_BOOL _MDrv_GFLIP_CSC_Calc(ST_GFLIP_GOP_CSC_PARAM *pstGflipCSCParam);
INTERFACE SYMBOL_WEAK MS_BOOL _MDrv_GFLIP_SetBWPEnFlag(MS_U32 u32GopIdx, MS_BOOL bEnable);
//INTERFACE void MDrv_GFLIP_FilmDriverHWVer1(void);
//INTERFACE void MDrv_GFLIP_FilmDriverHWVer2(void);
#if ( defined (CONFIG_MSTAR_NEW_FLIP_FUNCTION_ENABLE))
INTERFACE MS_U32 MDrv_GFLIP_InitTimer(void);
INTERFACE MS_U32 MDrv_GFLIP_Del_Timer(void);
#endif

#if defined(__cplusplus)
}
#endif

#undef INTERFACE

#endif //_MDRV_GFLIP_H

