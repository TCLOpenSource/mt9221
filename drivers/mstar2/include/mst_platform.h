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

#ifndef __MST_PLATFORM_H__
#define __MST_PLATFORM_H__

#define _6BITS      0
#define _8BITS      1
#define _10BITS      2

#define _TTL                0
#define _TCON               1
#define _LVDS               2
#define _RSDS               3

#define _SINGLE      0
#define _DUAL           1
#define _QUAD           2
#define _QUAD_LR           3

// for Lvds output channel,
// REG_321F[7:6]=>D, [5:4]=>C, [3:2]=>B, [1:0]=>A
#define CHANNEL_SWAP(D, C, B, A) ((D<<6)&0xC0)|((C<<4)&0x30)|((B<<2)&0x0C)|(A&0x03)
#define _CH_A                       0
#define _CH_B                       1
#define _CH_C                       2
#define _CH_D                       3

//For titania control setting; REG_MOD40 [2]=>TI/JATA [5]=>P_N_SWAP [6]=>LSB_MSB_SWAP
#define MOD40(TI_JA, P_N_SWAP, L_M_SWAP, DCLK_DLY) (((TI_JA<<2)&0x04)|((P_N_SWAP<<5)&0x20)|((L_M_SWAP<<6)&0x40)|((DCLK_DLY<<7)&0x0F00))
#define _TI                       1
#define _JEIDA                    0
#define _L_M_SWAP_ON              1
#define _L_M_SWAP_OFF             0
#define _P_N_SWAP_ON              1
#define _P_N_SWAP_OFF             0

/*For titania control setting; REG_MOD49[3]=>SWAP_ODD_RB,
          [4]=>SWAP_EVEN_RB,
          [5]=>SWAP_ODD_ML,
          [6]=>SWAP_EVEN_ML,
*/
#define MOD49(EVEN_ML, EVEN_RB, ODD_ML, ODD_RB, BIT_NUM) \
    (((EVEN_ML<<14)&0x4000)|((EVEN_RB<<13)&0x2000)|\
    ((ODD_ML<<12)&0x1000)|((ODD_RB<<11)&0x0800))
#define _10BITS_OP      0
#define _6BITS_OP       1
#define _8BITS_OP       2
#define _ODD_RB_OFF    0
#define _ODD_RB_ON     1
#define _ODD_ML_OFF    0
#define _ODD_ML_ON     1
#define _EVEN_RB_OFF   0
#define _EVEN_RB_ON    1
#define _EVEN_ML_OFF   0
#define _EVEN_ML_ON    1

/*For titania control setting; REG_MOD4A[0]=>ODD_EVEN_SWAP,
            [1]=>SINGLE_DUAL_CHANNEL
            [2]=>INVERT_DATA_ENABLE
            [3]=>INVERT_VSYNC
            [4]=>INVERT_DCLK
            [12]=>INVERT_HSYNC
*/
#define MOD4A(INV_HSYNC, INV_DCLK, INV_VSYNC, INV_DE, SD_CH, OE_SWAP) \
    (((INV_VSYNC<<12)&0x1000)|((INV_VSYNC<<4)&0x10)|\
    ((INV_VSYNC<<3)&0x08)|((INV_DE<<2)&0x04)|\
    ((SD_CH<<1)&0x02)|(OE_SWAP&0x01))
#define _ODD_EVEN_SWAP_OFF        0
#define _ODD_EVEN_SWAP_ON         1
#define _INV_DE_OFF   0
#define _INV_DE_ON    1
#define _INV_VSYNC_OFF         0
#define _INV_VSYNC_ON          1
#define _INV_DCLK_OFF          0
#define _INV_DCLK_ON           1
#define _INV_HSYNC_OFF         0
#define _INV_HSYNC_ON          1

//For titania control setting; REG_MOD4B[0:1]=>0x: 10-bits, 10: 8-bits, 11: 6-bits
#define MOD4B(TI_BIT_MOD) (TI_BIT_MOD&0x03)
#define _TI_10_BITS        0
#define _TI_8_BITS         2
#define _TI_6_BITS         3
//-------------------------------------------------------------------------------------------------
// Board
//-------------------------------------------------------------------------------------------------
typedef struct MST_BOARD_INFO_s
{

} MST_BOARD_INFO_t, *PMST_BOARD_INFO_t;

//-------------------------------------------------------------------------------------------------
// Panel
//-------------------------------------------------------------------------------------------------

typedef struct MST_PANEL_INFO_s
{
    // Basic
    U16 u16HStart; //ursa scaler
    U16 u16VStart; //ursa scaler
    U16 u16Width; //ursa scaler
    U16 u16Height; //ursa scaler
    U16 u16HTotal; //ursa scaler
    U16 u16VTotal; //ursa scaler
    U16 u16DefaultVFreq;
    U8  u8LPLL_Type;
    U8  u8LPLL_Mode;
    U8  u8HSyncWidth;
} MST_PANEL_INFO_t, *PMST_PANEL_INFO_t;



//-------------------------------------------------------------------------------------------------
// Platform
//-------------------------------------------------------------------------------------------------
typedef struct MST_PLATFORM_INFO_s
{
 MST_BOARD_INFO_t board;
    MST_PANEL_INFO_t panel;
} MST_PLATFORM_INFO_t, *PMST_PLATFORM_INFO_t;


#endif // __MST_PLATFORM_H__
