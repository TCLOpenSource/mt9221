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
// Excel update date    : 9/17/2015 10:21
//****************************************************

#ifndef _HALTX_TIMING_ATOP_TBL_C_
#define _HALTX_TIMING_ATOP_TBL_C_

#include "haltx_timing_tbl.h"
#include "haltx_timing_atop_tbl.h"

//****************************************************
// INIT_SYNTH_CLK
//****************************************************
MS_U8 MST_INIT_SYNTH_CLK_COMMON_TBL[MS_INIT_SYNTH_CLK_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_CHIPTOP_26_L), 0xF0, 0x00,
 },
 { DRV_REG(REG_CHIPTOP_34_L), 0x0F, 0x00,
 },
 { DRV_REG(REG_CHIPTOP_34_L), 0xF0, 0x00,
 },
};

//****************************************************
// INIT_HDMITX_MAC_CLK
//****************************************************
MS_U8 MST_INIT_HDMITX_MAC_CLK_COMMON_TBL[MS_INIT_HDMITX_MAC_CLK_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_HDMITX_MISC_1C_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_HDMITX_MISC_1C_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_HDMITX_MISC_1D_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_HDMITX_MISC_1D_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_HDMITX_MISC_1E_L), 0xFF, 0xFF,
 },
 { DRV_REG(REG_HDMITX_MISC_1E_H), 0xFF, 0xFF,
 },
 { DRV_REG(REG_HDMITX_MISC_1F_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_HDMITX_MISC_1F_H), 0xFF, 0x00,
 },
};

MS_U8 MST_INIT_HDMITX_MAC_CLK_TBL[MS_INIT_HDMITX_MAC_CLK_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_ATOP_ID_NUM]=
{
 { DRV_REG(REG_HDMITX_MISC_52_L), 0x01, 0x00, /*8bits_480_60I*/
                    0x00, /*8bits_480_60P*/
                    0x00, /*8bits_1080_60I*/
                    0x00, /*8bits_720_60P*/
                    0x00, /*8bits_1080_60P*/
                    0x00, /*8bits_4K2K_420_30P*/
                    0x00, /*8bits_4K2K_3D_30P*/
                    0x00, /*8bits_4K2K_420_60P*/
                    0x01, /*8bits_4K2K_60P*/
                    0x00, /*10bits_480_60I*/
                    0x00, /*10bits_480_60P*/
                    0x00, /*10bits_1080_60I*/
                    0x00, /*10bits_720_60P*/
                    0x00, /*10bits_1080_60P*/
                    0x00, /*10bits_4K2K_420_30P*/
                    0x01, /*10bits_4K2K_3D_30P*/
                    0x01, /*10bits_4K2K_420_60P*/
                    0x00, /*12bits_480_60I*/
                    0x00, /*12bits_480_60P*/
                    0x00, /*12bits_1080_60I*/
                    0x00, /*12bits_720_60P*/
                    0x00, /*12bits_1080_60P*/
                    0x00, /*12bits_4K2K_420_30P*/
                    0x01, /*12bits_4K2K_3D_30P*/
                    0x01, /*12bits_4K2K_420_60P*/
                    0x00, /*16bits_480_60I*/
                    0x00, /*16bits_480_60P*/
                    0x00, /*16bits_1080_60I*/
                    0x00, /*16bits_720_60P*/
                    0x00, /*16bits_1080_60P*/
                    0x00, /*16bits_4K2K_420_30P*/
                    0x01, /*16bits_4K2K_3D_30P*/
                    0x01, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_MISC_5D_H), 0x01, 0x00, /*8bits_480_60I*/
                    0x00, /*8bits_480_60P*/
                    0x00, /*8bits_1080_60I*/
                    0x00, /*8bits_720_60P*/
                    0x00, /*8bits_1080_60P*/
                    0x00, /*8bits_4K2K_420_30P*/
                    0x00, /*8bits_4K2K_3D_30P*/
                    0x00, /*8bits_4K2K_420_60P*/
                    0x01, /*8bits_4K2K_60P*/
                    0x00, /*10bits_480_60I*/
                    0x00, /*10bits_480_60P*/
                    0x00, /*10bits_1080_60I*/
                    0x00, /*10bits_720_60P*/
                    0x00, /*10bits_1080_60P*/
                    0x00, /*10bits_4K2K_420_30P*/
                    0x01, /*10bits_4K2K_3D_30P*/
                    0x01, /*10bits_4K2K_420_60P*/
                    0x00, /*12bits_480_60I*/
                    0x00, /*12bits_480_60P*/
                    0x00, /*12bits_1080_60I*/
                    0x00, /*12bits_720_60P*/
                    0x00, /*12bits_1080_60P*/
                    0x00, /*12bits_4K2K_420_30P*/
                    0x01, /*12bits_4K2K_3D_30P*/
                    0x01, /*12bits_4K2K_420_60P*/
                    0x00, /*16bits_480_60I*/
                    0x00, /*16bits_480_60P*/
                    0x00, /*16bits_1080_60I*/
                    0x00, /*16bits_720_60P*/
                    0x00, /*16bits_1080_60P*/
                    0x00, /*16bits_4K2K_420_30P*/
                    0x01, /*16bits_4K2K_3D_30P*/
                    0x01, /*16bits_4K2K_420_60P*/
 },
};

//****************************************************
// INIT_HDMITX_PHY_CLK
//****************************************************
MS_U8 MST_INIT_HDMITX_PHY_CLK_COMMON_TBL[MS_INIT_HDMITX_PHY_CLK_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_HDMITX_PHY_10_L), 0x10, 0x10,
 },
 { DRV_REG(REG_HDMITX_PHY_10_H), 0x01, 0x01,
 },
 { DRV_REG(REG_HDMITX_PHY_10_H), 0x10, 0x10,
 },
 { DRV_REG(REG_HDMITX_PHY_16_L), 0x03, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_16_L), 0x04, 0x04,
 },
 { DRV_REG(REG_HDMITX_PHY_17_L), 0x03, 0x01,
 },
 { DRV_REG(REG_HDMITX_PHY_17_L), 0x04, 0x04,
 },
 { DRV_REG(REG_HDMITX_PHY_18_L), 0x03, 0x02,
 },
 { DRV_REG(REG_HDMITX_PHY_18_L), 0x04, 0x04,
 },
 { DRV_REG(REG_HDMITX_PHY_19_L), 0x03, 0x03,
 },
 { DRV_REG(REG_HDMITX_PHY_19_L), 0x04, 0x04,
 },
 { DRV_REG(REG_HDMITX_PHY_10_L), 0x01, 0x01,
 },
};

MS_U8 MST_INIT_HDMITX_PHY_CLK_TBL[MS_INIT_HDMITX_PHY_CLK_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_ATOP_ID_NUM]=
{
 { DRV_REG(REG_HDMITX_PHY_15_H), 0x30, 0x30, /*8bits_480_60I*/
                    0x30, /*8bits_480_60P*/
                    0x20, /*8bits_1080_60I*/
                    0x20, /*8bits_720_60P*/
                    0x10, /*8bits_1080_60P*/
                    0x10, /*8bits_4K2K_420_30P*/
                    0x10, /*8bits_4K2K_3D_30P*/
                    0x10, /*8bits_4K2K_420_60P*/
                    0x00, /*8bits_4K2K_60P*/
                    0x30, /*10bits_480_60I*/
                    0x30, /*10bits_480_60P*/
                    0x20, /*10bits_1080_60I*/
                    0x20, /*10bits_720_60P*/
                    0x10, /*10bits_1080_60P*/
                    0x10, /*10bits_4K2K_420_30P*/
                    0x00, /*10bits_4K2K_3D_30P*/
                    0x00, /*10bits_4K2K_420_60P*/
                    0x30, /*12bits_480_60I*/
                    0x30, /*12bits_480_60P*/
                    0x20, /*12bits_1080_60I*/
                    0x20, /*12bits_720_60P*/
                    0x10, /*12bits_1080_60P*/
                    0x10, /*12bits_4K2K_420_30P*/
                    0x00, /*12bits_4K2K_3D_30P*/
                    0x00, /*12bits_4K2K_420_60P*/
                    0x20, /*16bits_480_60I*/
                    0x20, /*16bits_480_60P*/
                    0x10, /*16bits_1080_60I*/
                    0x10, /*16bits_720_60P*/
                    0x00, /*16bits_1080_60P*/
                    0x00, /*16bits_4K2K_420_30P*/
                    0x00, /*16bits_4K2K_3D_30P*/
                    0x00, /*16bits_4K2K_420_60P*/
 },
};

//****************************************************
// INIT_HDMITX_ATOP
//****************************************************
MS_U8 MST_INIT_HDMITX_ATOP_COMMON_TBL[MS_INIT_HDMITX_ATOP_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_HDMITX_PHY_3D_H), 0x03, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_3D_L), 0x0F, 0x01,
 },
 { DRV_REG(REG_HDMITX_PHY_3C_L), 0x03, 0x01,
 },
 { DRV_REG(REG_HDMITX_PHY_3C_L), 0xF0, 0x50,
 },
 { DRV_REG(REG_HDMITX_PHY_34_L), 0x3F, 0x03,
 },
 { DRV_REG(REG_HDMITX_PHY_30_L), 0x3F, 0x14,
 },
 { DRV_REG(REG_HDMITX_PHY_2E_H), 0xEA, 0xEA,
 },
 { DRV_REG(REG_HDMITX_PHY_39_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_39_H), 0x0F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_3A_L), 0x70, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_3C_H), 0x03, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_32_L), 0x1F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_36_L), 0x1F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_39_H), 0xF0, 0xf0,
 },
 { DRV_REG(REG_HDMITX_PHY_32_H), 0x3F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_33_L), 0x3F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_33_H), 0x3F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_36_H), 0x3F, 0x00,
 },
 { DRV_REG(REG_HDMITX_PHY_37_H), 0x3F, 0x00,
 },
};

MS_U8 MST_INIT_HDMITX_ATOP_TBL[MS_INIT_HDMITX_ATOP_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_ATOP_ID_NUM]=
{
 { DRV_REG(REG_HDMITX_PHY_38_L), 0x03, 0x03, /*8bits_480_60I*/
                    0x03, /*8bits_480_60P*/
                    0x02, /*8bits_1080_60I*/
                    0x02, /*8bits_720_60P*/
                    0x01, /*8bits_1080_60P*/
                    0x01, /*8bits_4K2K_420_30P*/
                    0x01, /*8bits_4K2K_3D_30P*/
                    0x01, /*8bits_4K2K_420_60P*/
                    0x00, /*8bits_4K2K_60P*/
                    0x03, /*10bits_480_60I*/
                    0x03, /*10bits_480_60P*/
                    0x02, /*10bits_1080_60I*/
                    0x02, /*10bits_720_60P*/
                    0x01, /*10bits_1080_60P*/
                    0x01, /*10bits_4K2K_420_30P*/
                    0x00, /*10bits_4K2K_3D_30P*/
                    0x00, /*10bits_4K2K_420_60P*/
                    0x03, /*12bits_480_60I*/
                    0x03, /*12bits_480_60P*/
                    0x02, /*12bits_1080_60I*/
                    0x02, /*12bits_720_60P*/
                    0x01, /*12bits_1080_60P*/
                    0x01, /*12bits_4K2K_420_30P*/
                    0x00, /*12bits_4K2K_3D_30P*/
                    0x00, /*12bits_4K2K_420_60P*/
                    0x02, /*16bits_480_60I*/
                    0x02, /*16bits_480_60P*/
                    0x01, /*16bits_1080_60I*/
                    0x01, /*16bits_720_60P*/
                    0x00, /*16bits_1080_60P*/
                    0x00, /*16bits_4K2K_420_30P*/
                    0x00, /*16bits_4K2K_3D_30P*/
                    0x00, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_3C_H), 0xC0, 0xC0, /*8bits_480_60I*/
                    0xC0, /*8bits_480_60P*/
                    0x80, /*8bits_1080_60I*/
                    0x80, /*8bits_720_60P*/
                    0x40, /*8bits_1080_60P*/
                    0x40, /*8bits_4K2K_420_30P*/
                    0x40, /*8bits_4K2K_3D_30P*/
                    0x40, /*8bits_4K2K_420_60P*/
                    0x00, /*8bits_4K2K_60P*/
                    0xC0, /*10bits_480_60I*/
                    0xC0, /*10bits_480_60P*/
                    0x80, /*10bits_1080_60I*/
                    0x80, /*10bits_720_60P*/
                    0x40, /*10bits_1080_60P*/
                    0x40, /*10bits_4K2K_420_30P*/
                    0x00, /*10bits_4K2K_3D_30P*/
                    0x00, /*10bits_4K2K_420_60P*/
                    0xC0, /*12bits_480_60I*/
                    0xC0, /*12bits_480_60P*/
                    0x80, /*12bits_1080_60I*/
                    0x80, /*12bits_720_60P*/
                    0x40, /*12bits_1080_60P*/
                    0x40, /*12bits_4K2K_420_30P*/
                    0x00, /*12bits_4K2K_3D_30P*/
                    0x00, /*12bits_4K2K_420_60P*/
                    0x80, /*16bits_480_60I*/
                    0x80, /*16bits_480_60P*/
                    0x40, /*16bits_1080_60I*/
                    0x40, /*16bits_720_60P*/
                    0x00, /*16bits_1080_60P*/
                    0x00, /*16bits_4K2K_420_30P*/
                    0x00, /*16bits_4K2K_3D_30P*/
                    0x00, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_3C_H), 0x30, 0x00, /*8bits_480_60I*/
                    0x00, /*8bits_480_60P*/
                    0x00, /*8bits_1080_60I*/
                    0x00, /*8bits_720_60P*/
                    0x00, /*8bits_1080_60P*/
                    0x00, /*8bits_4K2K_420_30P*/
                    0x00, /*8bits_4K2K_3D_30P*/
                    0x00, /*8bits_4K2K_420_60P*/
                    0x00, /*8bits_4K2K_60P*/
                    0x10, /*10bits_480_60I*/
                    0x10, /*10bits_480_60P*/
                    0x10, /*10bits_1080_60I*/
                    0x10, /*10bits_720_60P*/
                    0x10, /*10bits_1080_60P*/
                    0x10, /*10bits_4K2K_420_30P*/
                    0x10, /*10bits_4K2K_3D_30P*/
                    0x10, /*10bits_4K2K_420_60P*/
                    0x20, /*12bits_480_60I*/
                    0x20, /*12bits_480_60P*/
                    0x20, /*12bits_1080_60I*/
                    0x20, /*12bits_720_60P*/
                    0x20, /*12bits_1080_60P*/
                    0x20, /*12bits_4K2K_420_30P*/
                    0x20, /*12bits_4K2K_3D_30P*/
                    0x20, /*12bits_4K2K_420_60P*/
                    0x30, /*16bits_480_60I*/
                    0x30, /*16bits_480_60P*/
                    0x30, /*16bits_1080_60I*/
                    0x30, /*16bits_720_60P*/
                    0x30, /*16bits_1080_60P*/
                    0x30, /*16bits_4K2K_420_30P*/
                    0x30, /*16bits_4K2K_3D_30P*/
                    0x30, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_34_H), 0x3F, 0x03, /*8bits_480_60I*/
                    0x03, /*8bits_480_60P*/
                    0x03, /*8bits_1080_60I*/
                    0x03, /*8bits_720_60P*/
                    0x03, /*8bits_1080_60P*/
                    0x03, /*8bits_4K2K_420_30P*/
                    0x0c, /*8bits_4K2K_3D_30P*/
                    0x0c, /*8bits_4K2K_420_60P*/
                    0x0c, /*8bits_4K2K_60P*/
                    0x03, /*10bits_480_60I*/
                    0x03, /*10bits_480_60P*/
                    0x03, /*10bits_1080_60I*/
                    0x03, /*10bits_720_60P*/
                    0x03, /*10bits_1080_60P*/
                    0x03, /*10bits_4K2K_420_30P*/
                    0x0c, /*10bits_4K2K_3D_30P*/
                    0x0c, /*10bits_4K2K_420_60P*/
                    0x03, /*12bits_480_60I*/
                    0x03, /*12bits_480_60P*/
                    0x03, /*12bits_1080_60I*/
                    0x03, /*12bits_720_60P*/
                    0x0C, /*12bits_1080_60P*/
                    0x0c, /*12bits_4K2K_420_30P*/
                    0x0c, /*12bits_4K2K_3D_30P*/
                    0x0c, /*12bits_4K2K_420_60P*/
                    0x03, /*16bits_480_60I*/
                    0x03, /*16bits_480_60P*/
                    0x03, /*16bits_1080_60I*/
                    0x03, /*16bits_720_60P*/
                    0xc, /*16bits_1080_60P*/
                    0x0c, /*16bits_4K2K_420_30P*/
                    0x0c, /*16bits_4K2K_3D_30P*/
                    0x0c, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_35_L), 0x3F, 0x03, /*8bits_480_60I*/
                    0x03, /*8bits_480_60P*/
                    0x03, /*8bits_1080_60I*/
                    0x03, /*8bits_720_60P*/
                    0x03, /*8bits_1080_60P*/
                    0x03, /*8bits_4K2K_420_30P*/
                    0x0c, /*8bits_4K2K_3D_30P*/
                    0x0c, /*8bits_4K2K_420_60P*/
                    0x0c, /*8bits_4K2K_60P*/
                    0x03, /*10bits_480_60I*/
                    0x03, /*10bits_480_60P*/
                    0x03, /*10bits_1080_60I*/
                    0x03, /*10bits_720_60P*/
                    0x03, /*10bits_1080_60P*/
                    0x03, /*10bits_4K2K_420_30P*/
                    0x0c, /*10bits_4K2K_3D_30P*/
                    0x0c, /*10bits_4K2K_420_60P*/
                    0x03, /*12bits_480_60I*/
                    0x03, /*12bits_480_60P*/
                    0x03, /*12bits_1080_60I*/
                    0x03, /*12bits_720_60P*/
                    0x0C, /*12bits_1080_60P*/
                    0x0c, /*12bits_4K2K_420_30P*/
                    0x0c, /*12bits_4K2K_3D_30P*/
                    0x0c, /*12bits_4K2K_420_60P*/
                    0x03, /*16bits_480_60I*/
                    0x03, /*16bits_480_60P*/
                    0x03, /*16bits_1080_60I*/
                    0x03, /*16bits_720_60P*/
                    0xc, /*16bits_1080_60P*/
                    0x0c, /*16bits_4K2K_420_30P*/
                    0x0c, /*16bits_4K2K_3D_30P*/
                    0x0c, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_35_H), 0x3F, 0x03, /*8bits_480_60I*/
                    0x03, /*8bits_480_60P*/
                    0x03, /*8bits_1080_60I*/
                    0x03, /*8bits_720_60P*/
                    0x03, /*8bits_1080_60P*/
                    0x03, /*8bits_4K2K_420_30P*/
                    0x0c, /*8bits_4K2K_3D_30P*/
                    0x0c, /*8bits_4K2K_420_60P*/
                    0x0c, /*8bits_4K2K_60P*/
                    0x03, /*10bits_480_60I*/
                    0x03, /*10bits_480_60P*/
                    0x03, /*10bits_1080_60I*/
                    0x03, /*10bits_720_60P*/
                    0x03, /*10bits_1080_60P*/
                    0x03, /*10bits_4K2K_420_30P*/
                    0x0c, /*10bits_4K2K_3D_30P*/
                    0x0c, /*10bits_4K2K_420_60P*/
                    0x03, /*12bits_480_60I*/
                    0x03, /*12bits_480_60P*/
                    0x03, /*12bits_1080_60I*/
                    0x03, /*12bits_720_60P*/
                    0x0C, /*12bits_1080_60P*/
                    0x0c, /*12bits_4K2K_420_30P*/
                    0x0c, /*12bits_4K2K_3D_30P*/
                    0x0c, /*12bits_4K2K_420_60P*/
                    0x03, /*16bits_480_60I*/
                    0x03, /*16bits_480_60P*/
                    0x03, /*16bits_1080_60I*/
                    0x03, /*16bits_720_60P*/
                    0xc, /*16bits_1080_60P*/
                    0x0c, /*16bits_4K2K_420_30P*/
                    0x0c, /*16bits_4K2K_3D_30P*/
                    0x0c, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_30_H), 0x3F, 0x14, /*8bits_480_60I*/
                    0x14, /*8bits_480_60P*/
                    0x14, /*8bits_1080_60I*/
                    0x14, /*8bits_720_60P*/
                    0x14, /*8bits_1080_60P*/
                    0x14, /*8bits_4K2K_420_30P*/
                    0x38, /*8bits_4K2K_3D_30P*/
                    0x38, /*8bits_4K2K_420_60P*/
                    0x38, /*8bits_4K2K_60P*/
                    0x14, /*10bits_480_60I*/
                    0x14, /*10bits_480_60P*/
                    0x14, /*10bits_1080_60I*/
                    0x14, /*10bits_720_60P*/
                    0x14, /*10bits_1080_60P*/
                    0x14, /*10bits_4K2K_420_30P*/
                    0x38, /*10bits_4K2K_3D_30P*/
                    0x38, /*10bits_4K2K_420_60P*/
                    0x14, /*12bits_480_60I*/
                    0x14, /*12bits_480_60P*/
                    0x14, /*12bits_1080_60I*/
                    0x14, /*12bits_720_60P*/
                    0x38, /*12bits_1080_60P*/
                    0x38, /*12bits_4K2K_420_30P*/
                    0x38, /*12bits_4K2K_3D_30P*/
                    0x38, /*12bits_4K2K_420_60P*/
                    0x14, /*16bits_480_60I*/
                    0x14, /*16bits_480_60P*/
                    0x14, /*16bits_1080_60I*/
                    0x14, /*16bits_720_60P*/
                    0x38, /*16bits_1080_60P*/
                    0x38, /*16bits_4K2K_420_30P*/
                    0x38, /*16bits_4K2K_3D_30P*/
                    0x38, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_31_L), 0x3F, 0x14, /*8bits_480_60I*/
                    0x14, /*8bits_480_60P*/
                    0x14, /*8bits_1080_60I*/
                    0x14, /*8bits_720_60P*/
                    0x14, /*8bits_1080_60P*/
                    0x14, /*8bits_4K2K_420_30P*/
                    0x38, /*8bits_4K2K_3D_30P*/
                    0x38, /*8bits_4K2K_420_60P*/
                    0x38, /*8bits_4K2K_60P*/
                    0x14, /*10bits_480_60I*/
                    0x14, /*10bits_480_60P*/
                    0x14, /*10bits_1080_60I*/
                    0x14, /*10bits_720_60P*/
                    0x14, /*10bits_1080_60P*/
                    0x14, /*10bits_4K2K_420_30P*/
                    0x38, /*10bits_4K2K_3D_30P*/
                    0x38, /*10bits_4K2K_420_60P*/
                    0x14, /*12bits_480_60I*/
                    0x14, /*12bits_480_60P*/
                    0x14, /*12bits_1080_60I*/
                    0x14, /*12bits_720_60P*/
                    0x38, /*12bits_1080_60P*/
                    0x38, /*12bits_4K2K_420_30P*/
                    0x38, /*12bits_4K2K_3D_30P*/
                    0x38, /*12bits_4K2K_420_60P*/
                    0x14, /*16bits_480_60I*/
                    0x14, /*16bits_480_60P*/
                    0x14, /*16bits_1080_60I*/
                    0x14, /*16bits_720_60P*/
                    0x38, /*16bits_1080_60P*/
                    0x38, /*16bits_4K2K_420_30P*/
                    0x38, /*16bits_4K2K_3D_30P*/
                    0x38, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_31_H), 0x3F, 0x14, /*8bits_480_60I*/
                    0x14, /*8bits_480_60P*/
                    0x14, /*8bits_1080_60I*/
                    0x14, /*8bits_720_60P*/
                    0x14, /*8bits_1080_60P*/
                    0x14, /*8bits_4K2K_420_30P*/
                    0x38, /*8bits_4K2K_3D_30P*/
                    0x38, /*8bits_4K2K_420_60P*/
                    0x38, /*8bits_4K2K_60P*/
                    0x14, /*10bits_480_60I*/
                    0x14, /*10bits_480_60P*/
                    0x14, /*10bits_1080_60I*/
                    0x14, /*10bits_720_60P*/
                    0x14, /*10bits_1080_60P*/
                    0x14, /*10bits_4K2K_420_30P*/
                    0x38, /*10bits_4K2K_3D_30P*/
                    0x38, /*10bits_4K2K_420_60P*/
                    0x14, /*12bits_480_60I*/
                    0x14, /*12bits_480_60P*/
                    0x14, /*12bits_1080_60I*/
                    0x14, /*12bits_720_60P*/
                    0x38, /*12bits_1080_60P*/
                    0x38, /*12bits_4K2K_420_30P*/
                    0x38, /*12bits_4K2K_3D_30P*/
                    0x38, /*12bits_4K2K_420_60P*/
                    0x14, /*16bits_480_60I*/
                    0x14, /*16bits_480_60P*/
                    0x14, /*16bits_1080_60I*/
                    0x14, /*16bits_720_60P*/
                    0x38, /*16bits_1080_60P*/
                    0x38, /*16bits_4K2K_420_30P*/
                    0x38, /*16bits_4K2K_3D_30P*/
                    0x38, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_3A_L), 0x0F, 0x0F, /*8bits_480_60I*/
                    0x0F, /*8bits_480_60P*/
                    0x0F, /*8bits_1080_60I*/
                    0x0F, /*8bits_720_60P*/
                    0x0F, /*8bits_1080_60P*/
                    0x0F, /*8bits_4K2K_420_30P*/
                    0x08, /*8bits_4K2K_3D_30P*/
                    0x08, /*8bits_4K2K_420_60P*/
                    0x08, /*8bits_4K2K_60P*/
                    0x0F, /*10bits_480_60I*/
                    0x0F, /*10bits_480_60P*/
                    0x0F, /*10bits_1080_60I*/
                    0x0F, /*10bits_720_60P*/
                    0x0F, /*10bits_1080_60P*/
                    0x0F, /*10bits_4K2K_420_30P*/
                    0x08, /*10bits_4K2K_3D_30P*/
                    0x08, /*10bits_4K2K_420_60P*/
                    0x0F, /*12bits_480_60I*/
                    0x0F, /*12bits_480_60P*/
                    0x0F, /*12bits_1080_60I*/
                    0x0F, /*12bits_720_60P*/
                    0x08, /*12bits_1080_60P*/
                    0x08, /*12bits_4K2K_420_30P*/
                    0x08, /*12bits_4K2K_3D_30P*/
                    0x08, /*12bits_4K2K_420_60P*/
                    0x0F, /*16bits_480_60I*/
                    0x0F, /*16bits_480_60P*/
                    0x0F, /*16bits_1080_60I*/
                    0x0F, /*16bits_720_60P*/
                    0x08, /*16bits_1080_60P*/
                    0x08, /*16bits_4K2K_420_30P*/
                    0x08, /*16bits_4K2K_3D_30P*/
                    0x08, /*16bits_4K2K_420_60P*/
 },
 { DRV_REG(REG_HDMITX_PHY_15_H), 0x30, 0x30, /*8bits_480_60I*/
                    0x30, /*8bits_480_60P*/
                    0x20, /*8bits_1080_60I*/
                    0x20, /*8bits_720_60P*/
                    0x10, /*8bits_1080_60P*/
                    0x10, /*8bits_4K2K_420_30P*/
                    0x10, /*8bits_4K2K_3D_30P*/
                    0x10, /*8bits_4K2K_420_60P*/
                    0x00, /*8bits_4K2K_60P*/
                    0x30, /*10bits_480_60I*/
                    0x30, /*10bits_480_60P*/
                    0x20, /*10bits_1080_60I*/
                    0x20, /*10bits_720_60P*/
                    0x10, /*10bits_1080_60P*/
                    0x10, /*10bits_4K2K_420_30P*/
                    0x00, /*10bits_4K2K_3D_30P*/
                    0x00, /*10bits_4K2K_420_60P*/
                    0x30, /*12bits_480_60I*/
                    0x30, /*12bits_480_60P*/
                    0x20, /*12bits_1080_60I*/
                    0x20, /*12bits_720_60P*/
                    0x10, /*12bits_1080_60P*/
                    0x10, /*12bits_4K2K_420_30P*/
                    0x00, /*12bits_4K2K_3D_30P*/
                    0x00, /*12bits_4K2K_420_60P*/
                    0x20, /*16bits_480_60I*/
                    0x20, /*16bits_480_60P*/
                    0x10, /*16bits_1080_60I*/
                    0x10, /*16bits_720_60P*/
                    0x00, /*16bits_1080_60P*/
                    0x00, /*16bits_4K2K_420_30P*/
                    0x00, /*16bits_4K2K_3D_30P*/
                    0x00, /*16bits_4K2K_420_60P*/
 },
};

MS_TX_ATOP_INFO stHALTX_TIMING_ATOP_TBL[MS_TX_ATOP_TAB_NUM]=
{
    {*MST_INIT_SYNTH_CLK_COMMON_TBL,MS_INIT_SYNTH_CLK_COMMON_REG_NUM, MS_TX_ATOP_IP_COMMON},
    {*MST_INIT_HDMITX_MAC_CLK_TBL,MS_INIT_HDMITX_MAC_CLK_REG_NUM, MS_TX_ATOP_IP_NORMAL},
    {*MST_INIT_HDMITX_MAC_CLK_COMMON_TBL,MS_INIT_HDMITX_MAC_CLK_COMMON_REG_NUM, MS_TX_ATOP_IP_COMMON},
    {*MST_INIT_HDMITX_PHY_CLK_TBL,MS_INIT_HDMITX_PHY_CLK_REG_NUM, MS_TX_ATOP_IP_NORMAL},
    {*MST_INIT_HDMITX_PHY_CLK_COMMON_TBL,MS_INIT_HDMITX_PHY_CLK_COMMON_REG_NUM, MS_TX_ATOP_IP_COMMON},
    {*MST_INIT_HDMITX_ATOP_TBL,MS_INIT_HDMITX_ATOP_REG_NUM, MS_TX_ATOP_IP_NORMAL},
    {*MST_INIT_HDMITX_ATOP_COMMON_TBL,MS_INIT_HDMITX_ATOP_COMMON_REG_NUM, MS_TX_ATOP_IP_COMMON},
};
#endif
