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

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// @file   mhal_graphic.h
// @brief  Graphic Driver Interface
// @author MStar Semiconductor Inc.
//////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define _phy_to_miu_offset(MiuSel, Offset, PhysAddr) if (PhysAddr < ARM_MIU1_BASE_ADDR) \
                                                        {MiuSel = E_DRV_GOP_SEL_MIU0; Offset = PhysAddr;} \

#define MAX_GOP0_GWIN                           1UL
#define MAX_GOP1_GWIN                           1UL
#define MAX_GOP2_GWIN                           1UL
#define MAX_GOP3_GWIN                           1UL
#define MAX_GOP4_GWIN                           1UL
#define MAX_GOP5_GWIN                           0UL
#define MAX_GOP_MUX                             (4UL)

#define GOP_BIT0    0x01
#define GOP_BIT1    0x02
#define GOP_BIT2    0x04
#define GOP_BIT3    0x08
#define GOP_BIT4    0x10
#define GOP_BIT5    0x20
#define GOP_BIT6    0x40
#define GOP_BIT7    0x80
#define GOP_BIT8    0x0100
#define GOP_BIT9    0x0200
#define GOP_BIT10   0x0400
#define GOP_BIT11   0x0800
#define GOP_BIT12   0x1000
#define GOP_BIT13   0x2000
#define GOP_BIT14   0x4000
#define GOP_BIT15   0x8000

#define GOP_REG_WORD_MASK                           0xffff
#define GOP_REG_HW_MASK                             0xff00
#define GOP_REG_LW_MASK                             0x00ff

#define GOP_FIFO_BURST_ALL                          (GOP_BIT8|GOP_BIT9|GOP_BIT10|GOP_BIT11|GOP_BIT12)

#define GOP_FIFO_BURST_MASK                         (GOP_BIT8|GOP_BIT9|GOP_BIT10|GOP_BIT11|GOP_BIT12)
#define GOP_FIFO_THRESHOLD                          0xD0

#define GOP_YUV_TRANSPARENT_ENABLE              GOP_BIT5
#define GOP_RGB_TRANSPARENT_ENABLE              GOP_BIT11

#define GOP_WordUnit                                32
#define GOP_STRETCH_WIDTH_UNIT                2
#define SCALING_MULITPLIER                      0x1000

#define PANEL_WIDTH         3840
#define PANEL_HEIGHT        2160

#define GOP_PD  (0x3C)
#define GOP_MUX_DELTA   (0x1)
#define GOP_MUX0_Offset (0x0)
#define GOP_MUX1_Offset (0x3)
#define GOP_MUX2_Offset (0x6)
#define GOP_MUX3_Offset (0x9)
#define GOP_MUX4_Offset (0xC)

typedef enum
{
    E_GOP_MUX0 = 0,
    E_GOP_MUX1 = 1,
    E_GOP_MUX2 = 2,
    E_GOP_MUX3 = 3,
    E_GOP_MUX4 = 4,
    MAX_GOP_MUX_SUPPORT,
}EN_GOP_MUXSEL;
//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------

void _HAL_GOP_Write16Reg(MS_U32 u32addr, MS_U16 u16val, MS_U16 mask);
void _HAL_GOP_Read16Reg(MS_U32 u32addr, MS_U16* pu16ret);
void _HAL_GOP_Write32Reg(MS_U32 u32addr, MS_U32 u32val);
void _HAL_GOP_Init(MS_U8 u8GOP);
void _HAL_GOP_GWIN_SetDstPlane(MS_U8 u8GOP, EN_DRV_GOP_DST_TYPE eDsttype, MS_BOOL bOnlyCheck);
void _HAL_GOP_SetBlending(MS_U8 u8GOP, MS_U8 u8win, MS_BOOL bEnable, MS_U8 u8coef);
MS_U8 _HAL_GOP_GetBnkOfstByGop(MS_U8 gop, MS_U32 *pBnkOfst);
void _HAL_GOP_SetHMirror(MS_U8 u8GOP,MS_BOOL bEnable);
void _HAL_GOP_SetVMirror(MS_U8 u8GOP,MS_BOOL bEnable);
void _HAL_GOP_OutputColor_EX(MS_U8 u8GOP,EN_DRV_GOP_OUTPUT_COLOR type);
void _HAL_GOP_MIUSel(MS_U8 u8GOP, E_DRV_GOP_SEL_TYPE MiuSel);
void _HAL_GOP_EnableTransClr_EX(MS_U8 u8GOP,EN_DRV_GOP_TRANSCLR_FMT eFmt, MS_BOOL bEnable);
MS_BOOL _HAL_GOP_IsVMirrorOn(MS_U8 u8GOPnum);
void _HAL_GOP_SetGwinInfo(MS_U8 u8GOP, MS_U8 u8win,DRV_GWIN_INFO WinInfo);
void _HAL_GOP_SetStretchWin(MS_U8 u8GOP,MS_U16 u16x, MS_U16 u16y, MS_U16 u16width, MS_U16 u16height);
void _HAL_GOP_SetHScale(MS_U8 u8GOP,MS_BOOL bEnable, MS_U16 u16src, MS_U16 u16dst);
void _HAL_GOP_SetVScale(MS_U8 u8GOP,MS_BOOL bEnable, MS_U16 u16src, MS_U16 u16dst);
void _HAL_GOP_GWIN_Enable(MS_U8 u8GOP, MS_U8 u8win,MS_BOOL bEnable);
void _HAL_GOP_UpdateReg(MS_U8 u8Gop);
void _HAL_GOP_SetForceWrite(MS_BOOL bEnable);
void _HAL_GOP_GettXCTiming(MS_U16* pu16Width, MS_U16* pu16Height);
MS_BOOL MHal_GRAPHIC_GetPixelModeSupport(void);
MS_BOOL MHal_GOP_SetPipeDelay(MS_U8 u8GopIdx);
MS_U8 MHal_GOP_GetMux(EN_GOP_MUXSEL eGopMux);
MS_BOOL MHal_GRAPHIC_IsInit(MS_U8 u8GOPNum);
MS_BOOL MHal_GRAPHIC_InitGopPart(MS_U8 u8GOPNum);
MS_BOOL MHal_GOP_SetOSDBEnable(EN_GOP_MUXSEL eGopMux);
