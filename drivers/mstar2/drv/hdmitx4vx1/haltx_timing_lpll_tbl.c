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

#ifndef _HALTX_TIMING_LPLL_TBL_C_
#define _HALTX_TIMING_LPLL_TBL_C_

#include "haltx_timing_tbl.h"
#include "haltx_timing_lpll_tbl.h"

//****************************************************
// INIT_LPLL_SET
//****************************************************
MS_U8 MST_INIT_LPLL_SET_TBL[MS_INIT_LPLL_SET_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_LPLL_ID_NUM]=
{
 { DRV_REG(REG_LPLL_0F_L), 0xFF, 0x00, /*8bits_480_60I*/
                    0xB7, /*8bits_640x480_60P*/
                    0x00, /*8bits_480_60P*/
                    0x8D, /*8bits_1080_60I*/
                    0xA2, /*8bits_720_60P*/
                    0xA2, /*8bits_1080_60P*/
                    0xA2, /*8bits_4K2K_420_30P*/
                    0xD1, /*8bits_4K2K_3D_30P*/
                    0xD1, /*8bits_4K2K_420_60P*/
                    0xD1, /*8bits_4K2K_60P*/
                    0x33, /*8bits_800x600_60P*/
                    0x33, /*8bits_848x480_60P*/
                    0x52, /*8bits_1024x768_60P*/
                    0xC1, /*8bits_1280x768_60P*/
                    0x12, /*8bits_1280x800_60P*/
                    0x00, /*8bits_1280x960_60P*/
                    0x94, /*8bits_1360x768_60P*/
                    0xA6, /*8bits_1400x1050_60P*/
                    0xC2, /*8bits_1440x900_60P*/
                    0x00, /*8bits_1600x900_60P*/
                    0xAA, /*8bits_1600x1200_60P*/
                    0xF4, /*8bits_1680x1050_60P*/
                    0x63, /*8bits_1920x1200_60P*/
                    0x7F, /*8bits_2048x1152_60P*/
                    0x33, /*10bits_480_60I*/
                    0x5F, /*10bits_640x480_60P*/
                    0x33, /*10bits_480_60P*/
                    0xA, /*10bits_1080_60I*/
                    0x82, /*10bits_720_60P*/
                    0x82, /*10bits_1080_60P*/
                    0x82, /*10bits_4K2K_420_30P*/
                    0x82, /*10bits_4K2K_3D_30P*/
                    0x82, /*10bits_4K2K_420_60P*/
                    0xAA, /*12bits_480_60I*/
                    0x7A, /*12bits_640x480_60P*/
                    0xAA, /*12bits_480_60P*/
                    0xB4, /*12bits_1080_60I*/
                    0xC1, /*12bits_720_60P*/
                    0xC1, /*12bits_1080_60P*/
                    0xC1, /*12bits_4K2K_420_30P*/
                    0xC1, /*12bits_4K2K_3D_30P*/
                    0xC1, /*12bits_4K2K_420_60P*/
                    0x00, /*16bits_480_60I*/
                    0xB7, /*16bits_640x480_60P*/
                    0x00, /*16bits_480_60P*/
                    0x8D, /*16bits_1080_60I*/
                    0xA2, /*16bits_720_60P*/
                    0xA2, /*16bits_1080_60P*/
                    0xA2, /*16bits_4K2K_420_30P*/
                    0xD1, /*16bits_4K2K_3D_30P*/
                    0xD1, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_LPLL_0F_H), 0xFF, 0x00, /*8bits_480_60I*/
                    0xA3, /*8bits_640x480_60P*/
                    0x00, /*8bits_480_60P*/
                    0x97, /*8bits_1080_60I*/
                    0x8B, /*8bits_720_60P*/
                    0x8B, /*8bits_1080_60P*/
                    0x8B, /*8bits_4K2K_420_30P*/
                    0x45, /*8bits_4K2K_3D_30P*/
                    0x45, /*8bits_4K2K_420_60P*/
                    0x45, /*8bits_4K2K_60P*/
                    0x33, /*8bits_800x600_60P*/
                    0x33, /*8bits_848x480_60P*/
                    0x2B, /*8bits_1024x768_60P*/
                    0x78, /*8bits_1280x768_60P*/
                    0xAD, /*8bits_1280x800_60P*/
                    0x00, /*8bits_1280x960_60P*/
                    0xD7, /*8bits_1360x768_60P*/
                    0xC5, /*8bits_1400x1050_60P*/
                    0xE6, /*8bits_1440x900_60P*/
                    0x00, /*8bits_1600x900_60P*/
                    0xAA, /*8bits_1600x1200_60P*/
                    0x42, /*8bits_1680x1050_60P*/
                    0xC4, /*8bits_1920x1200_60P*/
                    0x18, /*8bits_2048x1152_60P*/
                    0x33, /*10bits_480_60I*/
                    0xE9, /*10bits_640x480_60P*/
                    0x33, /*10bits_480_60P*/
                    0x46, /*10bits_1080_60I*/
                    0x3C, /*10bits_720_60P*/
                    0x3C, /*10bits_1080_60P*/
                    0x3C, /*10bits_4K2K_420_30P*/
                    0x3C, /*10bits_4K2K_3D_30P*/
                    0x3C, /*10bits_4K2K_420_60P*/
                    0xAA, /*12bits_480_60I*/
                    0xC2, /*12bits_640x480_60P*/
                    0xAA, /*12bits_480_60P*/
                    0xF, /*12bits_1080_60I*/
                    0x07, /*12bits_720_60P*/
                    0x07, /*12bits_1080_60P*/
                    0x07, /*12bits_4K2K_420_30P*/
                    0x07, /*12bits_4K2K_3D_30P*/
                    0x07, /*12bits_4K2K_420_60P*/
                    0x00, /*16bits_480_60I*/
                    0xA3, /*16bits_640x480_60P*/
                    0x00, /*16bits_480_60P*/
                    0x97, /*16bits_1080_60I*/
                    0x8B, /*16bits_720_60P*/
                    0x8B, /*16bits_1080_60P*/
                    0x8B, /*16bits_4K2K_420_30P*/
                    0x45, /*16bits_4K2K_3D_30P*/
                    0x45, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_LPLL_10_L), 0xFF, 0x40, /*8bits_480_60I*/
                    0x44, /*8bits_640x480_60P*/
                    0x40, /*8bits_480_60P*/
                    0x2E, /*8bits_1080_60I*/
                    0x2E, /*8bits_720_60P*/
                    0x2E, /*8bits_1080_60P*/
                    0x2E, /*8bits_4K2K_420_30P*/
                    0x17, /*8bits_4K2K_3D_30P*/
                    0x17, /*8bits_4K2K_420_60P*/
                    0x17, /*8bits_4K2K_60P*/
                    0x2B, /*8bits_800x600_60P*/
                    0x33, /*8bits_848x480_60P*/
                    0x35, /*8bits_1024x768_60P*/
                    0x2B, /*8bits_1280x768_60P*/
                    0x30, /*8bits_1280x800_60P*/
                    0x40, /*8bits_1280x960_60P*/
                    0x50, /*8bits_1360x768_60P*/
                    0x38, /*8bits_1400x1050_60P*/
                    0x40, /*8bits_1440x900_60P*/
                    0x40, /*8bits_1600x900_60P*/
                    0x2A, /*8bits_1600x1200_60P*/
                    0x2F, /*8bits_1680x1050_60P*/
                    0x23, /*8bits_1920x1200_60P*/
                    0x2C, /*8bits_2048x1152_60P*/
                    0x33, /*10bits_480_60I*/
                    0x36, /*10bits_640x480_60P*/
                    0x33, /*10bits_480_60P*/
                    0x25, /*10bits_1080_60I*/
                    0x25, /*10bits_720_60P*/
                    0x25, /*10bits_1080_60P*/
                    0x25, /*10bits_4K2K_420_30P*/
                    0x25, /*10bits_4K2K_3D_30P*/
                    0x25, /*10bits_4K2K_420_60P*/
                    0x2A, /*12bits_480_60I*/
                    0x2D, /*12bits_640x480_60P*/
                    0x2A, /*12bits_480_60P*/
                    0x1F, /*12bits_1080_60I*/
                    0x1F, /*12bits_720_60P*/
                    0x1F, /*12bits_1080_60P*/
                    0x1F, /*12bits_4K2K_420_30P*/
                    0x1F, /*12bits_4K2K_3D_30P*/
                    0x1F, /*12bits_4K2K_420_60P*/
                    0x40, /*16bits_480_60I*/
                    0x44, /*16bits_640x480_60P*/
                    0x40, /*16bits_480_60P*/
                    0x2E, /*16bits_1080_60I*/
                    0x2E, /*16bits_720_60P*/
                    0x2E, /*16bits_1080_60P*/
                    0x2E, /*16bits_4K2K_420_30P*/
                    0x17, /*16bits_4K2K_3D_30P*/
                    0x17, /*16bits_4K2K_420_60P*/
 },
};

//****************************************************
// INIT_HDMITX_CD
//****************************************************
MS_U8 MST_INIT_HDMITX_CD_TBL[MS_INIT_HDMITX_CD_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_LPLL_ID_NUM]=
{
 { DRV_REG(REG_HDMITX_00_L), 0xFF, 0x04, /*8bits_480_60I*/
                    0x04, /*8bits_640x480_60P*/
                    0x04, /*8bits_480_60P*/
                    0x04, /*8bits_1080_60I*/
                    0x04, /*8bits_720_60P*/
                    0x04, /*8bits_1080_60P*/
                    0x04, /*8bits_4K2K_420_30P*/
                    0x04, /*8bits_4K2K_3D_30P*/
                    0x04, /*8bits_4K2K_420_60P*/
                    0x04, /*8bits_4K2K_60P*/
                    0x04, /*8bits_800x600_60P*/
                    0x04, /*8bits_848x480_60P*/
                    0x04, /*8bits_1024x768_60P*/
                    0x04, /*8bits_1280x768_60P*/
                    0x04, /*8bits_1280x800_60P*/
                    0x04, /*8bits_1280x960_60P*/
                    0x04, /*8bits_1360x768_60P*/
                    0x04, /*8bits_1400x1050_60P*/
                    0x04, /*8bits_1440x900_60P*/
                    0x04, /*8bits_1600x900_60P*/
                    0x04, /*8bits_1600x1200_60P*/
                    0x04, /*8bits_1680x1050_60P*/
                    0x04, /*8bits_1920x1200_60P*/
                    0x04, /*8bits_2048x1152_60P*/
                    0x44, /*10bits_480_60I*/
                    0x44, /*10bits_640x480_60P*/
                    0x44, /*10bits_480_60P*/
                    0x44, /*10bits_1080_60I*/
                    0x44, /*10bits_720_60P*/
                    0x44, /*10bits_1080_60P*/
                    0x44, /*10bits_4K2K_420_30P*/
                    0x44, /*10bits_4K2K_3D_30P*/
                    0x44, /*10bits_4K2K_420_60P*/
                    0x84, /*12bits_480_60I*/
                    0x84, /*12bits_640x480_60P*/
                    0x84, /*12bits_480_60P*/
                    0x84, /*12bits_1080_60I*/
                    0x84, /*12bits_720_60P*/
                    0x84, /*12bits_1080_60P*/
                    0x84, /*12bits_4K2K_420_30P*/
                    0x84, /*12bits_4K2K_3D_30P*/
                    0x84, /*12bits_4K2K_420_60P*/
                    0xC4, /*16bits_480_60I*/
                    0xC4, /*16bits_640x480_60P*/
                    0xC4, /*16bits_480_60P*/
                    0xC4, /*16bits_1080_60I*/
                    0xC4, /*16bits_720_60P*/
                    0xC4, /*16bits_1080_60P*/
                    0xC4, /*16bits_4K2K_420_30P*/
                    0xC4, /*16bits_4K2K_3D_30P*/
                    0xC4, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_04_L), 0x0F, 0x04, /*8bits_480_60I*/
                    0x04, /*8bits_640x480_60P*/
                    0x04, /*8bits_480_60P*/
                    0x04, /*8bits_1080_60I*/
                    0x04, /*8bits_720_60P*/
                    0x04, /*8bits_1080_60P*/
                    0x04, /*8bits_4K2K_420_30P*/
                    0x04, /*8bits_4K2K_3D_30P*/
                    0x04, /*8bits_4K2K_420_60P*/
                    0x04, /*8bits_4K2K_60P*/
                    0x04, /*8bits_800x600_60P*/
                    0x04, /*8bits_848x480_60P*/
                    0x04, /*8bits_1024x768_60P*/
                    0x04, /*8bits_1280x768_60P*/
                    0x04, /*8bits_1280x800_60P*/
                    0x04, /*8bits_1280x960_60P*/
                    0x04, /*8bits_1360x768_60P*/
                    0x04, /*8bits_1400x1050_60P*/
                    0x04, /*8bits_1440x900_60P*/
                    0x04, /*8bits_1600x900_60P*/
                    0x04, /*8bits_1600x1200_60P*/
                    0x04, /*8bits_1680x1050_60P*/
                    0x04, /*8bits_1920x1200_60P*/
                    0x04, /*8bits_2048x1152_60P*/
                    0x05, /*10bits_480_60I*/
                    0x05, /*10bits_640x480_60P*/
                    0x05, /*10bits_480_60P*/
                    0x05, /*10bits_1080_60I*/
                    0x05, /*10bits_720_60P*/
                    0x05, /*10bits_1080_60P*/
                    0x05, /*10bits_4K2K_420_30P*/
                    0x05, /*10bits_4K2K_3D_30P*/
                    0x05, /*10bits_4K2K_420_60P*/
                    0x06, /*12bits_480_60I*/
                    0x06, /*12bits_640x480_60P*/
                    0x06, /*12bits_480_60P*/
                    0x06, /*12bits_1080_60I*/
                    0x06, /*12bits_720_60P*/
                    0x06, /*12bits_1080_60P*/
                    0x06, /*12bits_4K2K_420_30P*/
                    0x06, /*12bits_4K2K_3D_30P*/
                    0x06, /*12bits_4K2K_420_60P*/
                    0x07, /*16bits_480_60I*/
                    0x07, /*16bits_640x480_60P*/
                    0x07, /*16bits_480_60P*/
                    0x07, /*16bits_1080_60I*/
                    0x07, /*16bits_720_60P*/
                    0x07, /*16bits_1080_60P*/
                    0x07, /*16bits_4K2K_420_30P*/
                    0x07, /*16bits_4K2K_3D_30P*/
                    0x07, /*16bits_4K2K_420_60P*/
 },
};

MS_TX_LPLL_INFO stHALTX_TIMING_LPLL_TBL[MS_TX_LPLL_TAB_NUM]=
{
    {*MST_INIT_LPLL_SET_TBL,MS_INIT_LPLL_SET_REG_NUM, MS_TX_LPLL_IP_NORMAL},
    {*MST_INIT_HDMITX_CD_TBL,MS_INIT_HDMITX_CD_REG_NUM, MS_TX_LPLL_IP_NORMAL},
};
#endif
