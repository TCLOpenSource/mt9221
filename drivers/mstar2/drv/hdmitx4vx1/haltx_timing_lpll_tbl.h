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

// $Change: 1008272 $
//****************************************************
// Drive Chip           : Raptor
// Excel CodeGen Version: 1.05
// Excel SW      Version: 1.01
// Excel update date    : 11/12/2015 13:40
//****************************************************

#ifndef _HALTX_TIMING_LPLL_TBL_H_
#define _HALTX_TIMING_LPLL_TBL_H_

#ifdef __cplusplus
extern "C"
{
#endif

//#include "MsTypes.h"


////////////////////////////////////////////////////////////////////////////////
#define MS_INIT_LPLL_SET_REG_NUM        3
#define MS_INIT_HDMITX_CD_REG_NUM        2
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
typedef enum
{
    MS_TX_LPLL_ID_8bits_480_60I=0x0,
    MS_TX_LPLL_ID_8bits_640x480_60P=0x1,
    MS_TX_LPLL_ID_8bits_480_60P=0x2,
    MS_TX_LPLL_ID_8bits_1080_60I=0x3,
    MS_TX_LPLL_ID_8bits_720_60P=0x4,
    MS_TX_LPLL_ID_8bits_1080_60P=0x5,
    MS_TX_LPLL_ID_8bits_4K2K_420_30P=0x6,
    MS_TX_LPLL_ID_8bits_4K2K_3D_30P=0x7,
    MS_TX_LPLL_ID_8bits_4K2K_420_60P=0x8,
    MS_TX_LPLL_ID_8bits_4K2K_60P=0x9,
    MS_TX_LPLL_ID_8bits_800x600_60P=0xA,
    MS_TX_LPLL_ID_8bits_848x480_60P=0xB,
    MS_TX_LPLL_ID_8bits_1024x768_60P=0xC,
    MS_TX_LPLL_ID_8bits_1280x768_60P=0xD,
    MS_TX_LPLL_ID_8bits_1280x800_60P=0xE,
    MS_TX_LPLL_ID_8bits_1280x960_60P=0xF,
    MS_TX_LPLL_ID_8bits_1360x768_60P=0x10,
    MS_TX_LPLL_ID_8bits_1400x1050_60P=0x11,
    MS_TX_LPLL_ID_8bits_1440x900_60P=0x12,
    MS_TX_LPLL_ID_8bits_1600x900_60P=0x13,
    MS_TX_LPLL_ID_8bits_1600x1200_60P=0x14,
    MS_TX_LPLL_ID_8bits_1680x1050_60P=0x15,
    MS_TX_LPLL_ID_8bits_1920x1200_60P=0x16,
    MS_TX_LPLL_ID_8bits_2048x1152_60P=0x17,
    MS_TX_LPLL_ID_10bits_480_60I=0x18,
    MS_TX_LPLL_ID_10bits_640x480_60P=0x19,
    MS_TX_LPLL_ID_10bits_480_60P=0x1A,
    MS_TX_LPLL_ID_10bits_1080_60I=0x1B,
    MS_TX_LPLL_ID_10bits_720_60P=0x1C,
    MS_TX_LPLL_ID_10bits_1080_60P=0x1D,
    MS_TX_LPLL_ID_10bits_4K2K_420_30P=0x1E,
    MS_TX_LPLL_ID_10bits_4K2K_3D_30P=0x1F,
    MS_TX_LPLL_ID_10bits_4K2K_420_60P=0x20,
    MS_TX_LPLL_ID_12bits_480_60I=0x21,
    MS_TX_LPLL_ID_12bits_640x480_60P=0x22,
    MS_TX_LPLL_ID_12bits_480_60P=0x23,
    MS_TX_LPLL_ID_12bits_1080_60I=0x24,
    MS_TX_LPLL_ID_12bits_720_60P=0x25,
    MS_TX_LPLL_ID_12bits_1080_60P=0x26,
    MS_TX_LPLL_ID_12bits_4K2K_420_30P=0x27,
    MS_TX_LPLL_ID_12bits_4K2K_3D_30P=0x28,
    MS_TX_LPLL_ID_12bits_4K2K_420_60P=0x29,
    MS_TX_LPLL_ID_16bits_480_60I=0x2A,
    MS_TX_LPLL_ID_16bits_640x480_60P=0x2B,
    MS_TX_LPLL_ID_16bits_480_60P=0x2C,
    MS_TX_LPLL_ID_16bits_1080_60I=0x2D,
    MS_TX_LPLL_ID_16bits_720_60P=0x2E,
    MS_TX_LPLL_ID_16bits_1080_60P=0x2F,
    MS_TX_LPLL_ID_16bits_4K2K_420_30P=0x30,
    MS_TX_LPLL_ID_16bits_4K2K_3D_30P=0x31,
    MS_TX_LPLL_ID_16bits_4K2K_420_60P=0x32,
    MS_TX_LPLL_ID_NUM=0x33,
} E_MS_TX_LPLL_ID_TYPE;

typedef enum
{
    MS_TX_LPLL_TAB_INIT_LPLL_SET,
    MS_TX_LPLL_TAB_INIT_HDMITX_CD,
    MS_TX_LPLL_TAB_NUM,
} E_MS_TX_LPLL_TAB_TYPE;

typedef enum
{
    MS_TX_LPLL_IP_NORMAL,
    MS_TX_LPLL_IP_COMMON
} E_MS_TX_LPLL_IP_TYPE;

typedef struct
{
    MS_U8                 *pData;
    MS_U16                 u16RegNum;
    E_MS_TX_LPLL_IP_TYPE enIPType;
} MS_TX_LPLL_INFO;

extern MS_TX_LPLL_INFO stHALTX_TIMING_LPLL_TBL[MS_TX_LPLL_TAB_NUM];

#ifdef __cplusplus
}
#endif

#undef _DRVADCTBL_H_
#endif
