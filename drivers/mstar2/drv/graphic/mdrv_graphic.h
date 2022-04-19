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
/// @file   mdrv_graphic.h
/// @brief  MStar graphic Interface header file
/// @author MStar Semiconductor Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_GRAPHIC_H
#define _MDRV_GRAPHIC_H

#ifdef _MDRV_GRAPHIC_H
#define INTERFACE
#else
#define INTERFACE extern
#endif

#if defined(__cplusplus)
extern "C" {
#endif

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

/// GWIN output color domain
typedef enum
{
    DRV_GOPOUT_RGB,     ///< 0: output color RGB
    DRV_GOPOUT_YUV,     ///< 1: output color YUV
} EN_DRV_GOP_OUTPUT_COLOR;

/// Define GOP MIU SEL
typedef enum
{
    /// E_DRV_GOP_SEL_MIU0. gop access miu 0
    E_DRV_GOP_SEL_MIU0    = 0,
    /// E_GOP_SEL_MIU1. gop access miu1
    E_DRV_GOP_SEL_MIU1    = 1,
    /// E_GOP_SEL_MIU2. gop access miu2
    E_DRV_GOP_SEL_MIU2    = 2,
    /// E_GOP_SEL_MIU3. gop access miu3
    E_DRV_GOP_SEL_MIU3    = 3,
} E_DRV_GOP_SEL_TYPE;

typedef enum
{
    E_GOP0 = 0,
    E_GOP1 = 1,
    E_GOP2 = 2,
    E_GOP3 = 3,
    E_GOP4 = 4,
}E_GOP_TYPE;

/// Define GOP destination displayplane type
typedef enum
{
    E_DRV_GOP_DST_IP0       =   0,
    E_DRV_GOP_DST_IP0_SUB   =   1,
    E_DRV_GOP_DST_MIXER2VE  =   2,
    E_DRV_GOP_DST_OP0       =   3,
    E_DRV_GOP_DST_VOP       =   4,
    E_DRV_GOP_DST_IP1       =   5,
    E_DRV_GOP_DST_IP1_SUB   =   6,
    E_DRV_GOP_DST_MIXER2OP  =   7,
    E_DRV_GOP_DST_VOP_SUB   =   8,
    E_DRV_GOP_DST_FRC       =   9,
    E_DRV_GOP_DST_VE        =   10,
    E_DRV_GOP_DST_BYPASS    =   11,
    E_DRV_GOP_DST_OP1       =   12,
    E_DRV_GOP_DST_MIXER2OP1 =  13,
    E_DRV_GOP_DST_DIP       =   14,
    E_DRV_GOP_DST_GOPScaling  = 15,
    E_DRV_GOP_DST_OP_DUAL_RATE   = 16,
    MAX_DRV_GOP_DST_SUPPORT ,
    E_DRV_GOP_DST_INVALID  ,
} EN_DRV_GOP_DST_TYPE;

/// Transparent color format
typedef enum
{
    /// RGB mode transparent color.
    GOPTRANSCLR_FMT0,
   /// YUV mode transparent color.
    GOPTRANSCLR_FMT1,
} EN_DRV_GOP_TRANSCLR_FMT;

typedef enum
{
    /// Color format RGB555 and Blink.
    E_DRV_GOP_COLOR_RGB555_BLINK    =0,
    /// Color format RGB565.
    E_DRV_GOP_COLOR_RGB565          =1,
    /// Color format ARGB4444.
    E_DRV_GOP_COLOR_ARGB4444        =2,
    /// Color format alpha blink.
    E_DRV_GOP_COLOR_2266      =3,
    /// Color format I8 (256-entry palette).
    E_DRV_GOP_COLOR_I8              =4,
    /// Color format ARGB8888.
    E_DRV_GOP_COLOR_ARGB8888        =5,
    /// Color format ARGB1555.
    E_DRV_GOP_COLOR_ARGB1555        =6,
    /// Color format ARGB8888.  - Andriod format
    E_DRV_GOP_COLOR_ABGR8888        =7,
    /// Color format RGB555/YUV422.
     E_DRV_GOP_COLOR_RGB555YUV422    =8,
    /// Color format YUV422.
    E_DRV_GOP_COLOR_YUV422          =9,
    /// Color format ARGB8888.  - Andriod format
    E_DRV_GOP_COLOR_RGBA5551        =10,
    /// Color format ARGB8888.  - Andriod format
    E_DRV_GOP_COLOR_RGBA4444        =11,

    /// Invalid color format.
    E_DRV_GOP_COLOR_INVALID,
} EN_DRV_GOPColorType;

typedef struct
{
    ///panel width.
    MS_U16 u16PanelWidth;
    ///panel height.
    MS_U16 u16PanelHeight;
    ///panel h-start.
    MS_U16 u16PanelHStr;
    ///vsync interrupt flip enable flag.
    MS_BOOL bEnableVsyncIntFlip;
    ///gop frame buffer starting address.
    MS_PHY u32GOPRBAdr;
    ///gop frame buffer length.
    MS_U32 u32GOPRBLen;
    ///gop regdma starting address.
    MS_PHY u32GOPRegdmaAdr;
    ///gop regdma length.
    MS_U32 u32GOPRegdmaLen;
}GOP_InitInfo;

typedef struct
{
    MS_U16 u16HStart;              //!< unit: pix
    MS_U16 u16HEnd;                //!< unit: pix
    MS_U16 u16VStart;              //!< unit: pix
    MS_U16 u16VEnd;                //!< unit: pix
    MS_U32 u16Pitch;               //!< unit: Byte
    MS_U32 u32Addr;                //!< unit: pix
    EN_DRV_GOPColorType clrType;       //!< color format of the buffer
} DRV_GWIN_INFO;


//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define GOP0 0

#define HV_MIRROR_OFF 0
#define HV_MIRROR_EN 1
#define V_MIRROR_ONLY 2
#define H_MIRROR_ONLY 3

//=============================================================================
// Function
//=============================================================================

INTERFACE void _MDrv_GRAPHIC_Init(MS_U8 u8GopIdx);
INTERFACE void _MDrv_GRAPHIC_SetHMirror(MS_U8 u8GOP,MS_BOOL bEnable);
INTERFACE void _MDrv_GRAPHIC_SetVMirror(MS_U8 u8GOP,MS_BOOL bEnable);
INTERFACE void _MDrv_GRAPHIC_OutputColor_EX(MS_U8 u8GOP,EN_DRV_GOP_OUTPUT_COLOR type);
INTERFACE void _MDrv_GRAPHIC_MIUSel(MS_U8 u8GOP, E_DRV_GOP_SEL_TYPE MiuSel);
INTERFACE void _MDrv_GRAPHIC_SetGOPDst(MS_U8 u8GOP, EN_DRV_GOP_DST_TYPE eDsttype, MS_BOOL bOnlyCheck);
INTERFACE void _MDrv_GRAPHIC_EnableTransClr_EX(MS_U8 u8GOP,EN_DRV_GOP_TRANSCLR_FMT eFmt, MS_BOOL bEnable);
INTERFACE void _MDrv_GRAPHIC_SetBlending(MS_U8 u8GOP, MS_U8 u8win, MS_BOOL bEnable, MS_U8 u8coef);
INTERFACE void _MDrv_GOP_SetGwinInfo(MS_U8 u8GOP, MS_U8 u8win,DRV_GWIN_INFO WinInfo);
INTERFACE void _MDrv_GRAPHIC_SetStretchWin(MS_U8 u8GOP,MS_U16 u16x, MS_U16 u16y, MS_U16 u16width, MS_U16 u16height);
INTERFACE void _MDrv_GRAPHIC_SetHScale(MS_U8 u8GOP,MS_BOOL bEnable, MS_U16 u16src, MS_U16 u16dst);
INTERFACE void _MDrv_GRAPHIC_SetVScale(MS_U8 u8GOP,MS_BOOL bEnable, MS_U16 u16src, MS_U16 u16dst);
INTERFACE void _MDrv_GRAPHIC_GWIN_Enable(MS_U8 u8GOP, MS_U8 u8win,MS_BOOL bEnable);
INTERFACE void _MDrv_GRAPHIC_UpdateReg(MS_U8 u8Gop);
INTERFACE void _MDrv_GRAPHIC_SetForceWrite(MS_BOOL bEnable);
INTERFACE void _MDrv_GRAPHIC_GetXCTiming(MS_U16* pu16Width, MS_U16* pu16Height);
INTERFACE MS_BOOL MDrv_GRAPHIC_GetPixelModeSupport(void);
INTERFACE MS_BOOL MDrv_GRAPHIC_IsInit(MS_U8 u8GOPNum);
INTERFACE MS_BOOL MDrv_GRAPHIC_InitGopPart(MS_U8 u8GOPNum);
#if defined(__cplusplus)
}
#endif

#undef INTERFACE

#endif //_MDRV_GRAPHIC_H

