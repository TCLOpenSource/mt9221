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
// Excel update date    : 2/5/2016 9:52
//****************************************************

#ifndef _HALTX_TIMING_VX1_TBL_C_
#define _HALTX_TIMING_VX1_TBL_C_

#include "haltx_timing_tbl.h"
#include "haltx_timing_vx1_tbl.h"

//****************************************************
// INIT_Vx1_ANALOG
//****************************************************
MS_U8 MST_INIT_Vx1_ANALOG_COMMON_TBL[MS_INIT_Vx1_ANALOG_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_VBY1_RX_A_43_L), 0x71, 0x21,
 },
 { DRV_REG(REG_VBY1_RX_A_44_L), 0x0F, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_47_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_47_H), 0x0F, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_4F_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_4F_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_45_L), 0xF4, 0x40,
 },
 { DRV_REG(REG_VBY1_RX_A_45_H), 0x0B, 0x09,
 },
 { DRV_REG(REG_VBY1_RX_A_48_H), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_A_50_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_50_H), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_A_51_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_51_H), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_A_52_L), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_A_16_L), 0xFF, 0x33,
 },
 { DRV_REG(REG_VBY1_RX_A_16_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_A_2F_L), 0xFF, 0x02,
 },
 { DRV_REG(REG_VBY1_RX_A_2F_H), 0xFF, 0x00,
 },
};

MS_U8 MST_INIT_Vx1_ANALOG_TBL[MS_INIT_Vx1_ANALOG_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_VX1_ID_NUM]=
{
 { DRV_REG(REG_VBY1_RX_A_2B_L), 0xFF, 0xFF, /*1_LANE*/
                    0xAD, /*2_LANE*/
                    0xAD, /*4_LANE*/
                    0xAD, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2B_H), 0xFF, 0x03, /*1_LANE*/
                    0x42, /*2_LANE*/
                    0x42, /*4_LANE*/
                    0x42, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2C_L), 0xFF, 0xE8, /*1_LANE*/
                    0xAA, /*2_LANE*/
                    0xAA, /*4_LANE*/
                    0xAA, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2C_H), 0xFF, 0x03, /*1_LANE*/
                    0x42, /*2_LANE*/
                    0x42, /*4_LANE*/
                    0x42, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2D_L), 0xFF, 0xEF, /*1_LANE*/
                    0x58, /*2_LANE*/
                    0x58, /*4_LANE*/
                    0x58, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2D_H), 0xFF, 0x07, /*1_LANE*/
                    0x85, /*2_LANE*/
                    0x85, /*4_LANE*/
                    0x85, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2E_L), 0xFF, 0xD8, /*1_LANE*/
                    0xE8, /*2_LANE*/
                    0xE8, /*4_LANE*/
                    0xE8, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_A_2E_H), 0xFF, 0x07, /*1_LANE*/
                    0x67, /*2_LANE*/
                    0x67, /*4_LANE*/
                    0x67, /*8_LANE*/
 },
};

//****************************************************
// INIT_Vx1_DIG
//****************************************************
MS_U8 MST_INIT_Vx1_DIG_COMMON_TBL[MS_INIT_Vx1_DIG_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_VBY1_RX_2F_L), 0xFF, 0x0B,
 },
 { DRV_REG(REG_VBY1_RX_2F_H), 0xEF, 0xE3,
 },
 { DRV_REG(REG_VBY1_RX_26_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_26_H), 0x1F, 0x10,
 },
 { DRV_REG(REG_VBY1_RX_08_L), 0xFF, 0xA3,
 },
 { DRV_REG(REG_VBY1_RX_08_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_20_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_20_H), 0x0F, 0x04,
 },
 { DRV_REG(REG_VBY1_RX_27_L), 0xFF, 0x33,
 },
 { DRV_REG(REG_VBY1_RX_27_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_28_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_28_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_60_L), 0xFF, 0x1A,
 },
 { DRV_REG(REG_VBY1_RX_60_H), 0xFF, 0x1A,
 },
 { DRV_REG(REG_VBY1_RX_61_L), 0xFF, 0xFE,
 },
 { DRV_REG(REG_VBY1_RX_61_H), 0xFF, 0x1A,
 },
 { DRV_REG(REG_VBY1_RX_17_L), 0xFF, 0xA8,
 },
 { DRV_REG(REG_VBY1_RX_17_H), 0xFF, 0xB9,
 },
 { DRV_REG(REG_VBY1_RX_18_L), 0xFF, 0xEC,
 },
 { DRV_REG(REG_VBY1_RX_18_H), 0xFF, 0xFD,
 },
 { DRV_REG(REG_VBY1_RX_1B_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_1B_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_23_H), 0x0F, 0x0A,
 },
 { DRV_REG(REG_VBY1_RX_30_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_30_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_31_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_31_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_32_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_32_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_33_L), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_33_H), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_03_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_03_H), 0xFF, 0xFF,
 },
 { DRV_REG(REG_VBY1_RX_1E_L), 0xFF, 0x4C,
 },
 { DRV_REG(REG_VBY1_RX_1E_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_1F_L), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_1F_H), 0xFF, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_19_H), 0xFF, 0x40,
 },
 { DRV_REG(REG_VBY1_RX_1A_L), 0xFF, 0x11,
 },
 { DRV_REG(REG_VBY1_RX_1A_H), 0x77, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_1D_H), 0xFF, 0x00,
 },
};

MS_U8 MST_INIT_Vx1_DIG_TBL[MS_INIT_Vx1_DIG_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_VX1_ID_NUM]=
{
 { DRV_REG(REG_VBY1_RX_15_L), 0xFF, 0x04, /*1_LANE*/
                    0x04, /*2_LANE*/
                    0x04, /*4_LANE*/
                    0x40, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_15_H), 0xFF, 0x00, /*1_LANE*/
                    0x00, /*2_LANE*/
                    0x06, /*4_LANE*/
                    0x62, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_16_L), 0xFF, 0x04, /*1_LANE*/
                    0x05, /*2_LANE*/
                    0x05, /*4_LANE*/
                    0x51, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_16_H), 0xFF, 0x00, /*1_LANE*/
                    0x00, /*2_LANE*/
                    0x07, /*4_LANE*/
                    0x73, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_19_L), 0xFF, 0x04, /*1_LANE*/
                    0x04, /*2_LANE*/
                    0x04, /*4_LANE*/
                    0x00, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_02_L), 0xFF, 0x01, /*1_LANE*/
                    0x01, /*2_LANE*/
                    0x55, /*4_LANE*/
                    0xFF, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_02_H), 0xFF, 0x01, /*1_LANE*/
                    0x01, /*2_LANE*/
                    0x55, /*4_LANE*/
                    0xFF, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_09_L), 0xFF, 0x01, /*1_LANE*/
                    0x01, /*2_LANE*/
                    0x55, /*4_LANE*/
                    0xFF, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_09_H), 0xFF, 0x01, /*1_LANE*/
                    0x01, /*2_LANE*/
                    0x55, /*4_LANE*/
                    0xFF, /*8_LANE*/
 },
 { DRV_REG(REG_VBY1_RX_1D_L), 0xFF, 0xAA, /*1_LANE*/
                    0xAA, /*2_LANE*/
                    0x22, /*4_LANE*/
                    0x00, /*8_LANE*/
 },
};

//****************************************************
// INIT_Vx1_ERR_CHK
//****************************************************
MS_U8 MST_INIT_Vx1_ERR_CHK_COMMON_TBL[MS_INIT_Vx1_ERR_CHK_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_VBY1_RX_20_H), 0x40, 0x40,
 },
 { DRV_REG(REG_VBY1_RX_24_H), 0x10, 0x10,
 },
 { DRV_REG(REG_VBY1_RX_2F_H), 0x10, 0x00,
 },
 { DRV_REG(REG_VBY1_RX_71_H), 0x08, 0x08,
 },
 { DRV_REG(REG_VBY1_RX_A_1B_H), 0x40, 0x40,
 },
 { DRV_REG(REG_VBY1_RX_A_18_L), 0x30, 0x30,
 },
 { DRV_REG(REG_VBY1_RX_A_19_L), 0x30, 0x30,
 },
 { DRV_REG(REG_VBY1_RX_A_44_L), 0xF0, 0x50,
 },
};

MS_U8 MST_INIT_Vx1_ERR_CHK_TBL[MS_INIT_Vx1_ERR_CHK_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_VX1_ID_NUM]=
{
 { DRV_REG(REG_VBY1_RX_2B_H), 0xFF, 0xF0, /*1_LANE*/
                    0xE0, /*2_LANE*/
                    0xE0, /*4_LANE*/
                    0xE0, /*8_LANE*/
 },
};

MS_TX_VX1_INFO stHALTX_TIMING_VX1_TBL[MS_TX_VX1_TAB_NUM]=
{
    {*MST_INIT_Vx1_ANALOG_TBL,MS_INIT_Vx1_ANALOG_REG_NUM, MS_TX_VX1_IP_NORMAL},
    {*MST_INIT_Vx1_ANALOG_COMMON_TBL,MS_INIT_Vx1_ANALOG_COMMON_REG_NUM, MS_TX_VX1_IP_COMMON},
    {*MST_INIT_Vx1_DIG_TBL,MS_INIT_Vx1_DIG_REG_NUM, MS_TX_VX1_IP_NORMAL},
    {*MST_INIT_Vx1_DIG_COMMON_TBL,MS_INIT_Vx1_DIG_COMMON_REG_NUM, MS_TX_VX1_IP_COMMON},
    {*MST_INIT_Vx1_ERR_CHK_TBL,MS_INIT_Vx1_ERR_CHK_REG_NUM, MS_TX_VX1_IP_NORMAL},
    {*MST_INIT_Vx1_ERR_CHK_COMMON_TBL,MS_INIT_Vx1_ERR_CHK_COMMON_REG_NUM, MS_TX_VX1_IP_COMMON},
};
#endif
