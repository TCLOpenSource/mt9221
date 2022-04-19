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
// Excel update date    : 12/1/2015 11:04
//****************************************************

#ifndef _HALTX_TIMING_3X3MATRIX_TBL_C_
#define _HALTX_TIMING_3X3MATRIX_TBL_C_

#include "haltx_timing_tbl.h"
#include "haltx_timing_3x3matrix_tbl.h"

//****************************************************
// INIT_3x3_MATRIX
//****************************************************
MS_U8 MST_INIT_3x3_MATRIX_COMMON_TBL[MS_INIT_3x3_MATRIX_COMMON_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + REG_DATA_SIZE] =
{
 { DRV_REG(REG_HDMI444To422_70_L), 0x20, 0x00,
 },
 { DRV_REG(REG_HDMI444To422_70_L), 0x02, 0x00,
 },
};

MS_U8 MST_INIT_3x3_MATRIX_TBL[MS_INIT_3x3_MATRIX_REG_NUM][REG_ADDR_SIZE + REG_MASK_SIZE + MS_TX_3X3_MATRIX_ID_NUM]=
{
 { DRV_REG(REG_HDMI444To422_70_H), 0x80, 0x00, /*OFF*/
                 // 0x80, /*Black*/
                    0x00, /*R2Y_SD_Limit*/
                    0x80, /*R2Y_SD_Full*/
                    0x00, /*R2Y_HD_Limit*/
                    0x80, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_70_H), 0x40, 0x00, /*OFF*/
                 // 0x40, /*Black*/
                    0x40, /*R2Y_SD_Limit*/
                    0x40, /*R2Y_SD_Full*/
                    0x40, /*R2Y_HD_Limit*/
                    0x40, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_70_H), 0x20, 0x00, /*OFF*/
                 // 0x20, /*Black*/
                    0x20, /*R2Y_SD_Limit*/
                    0x20, /*R2Y_SD_Full*/
                    0x20, /*R2Y_HD_Limit*/
                    0x20, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_70_L), 0x10, 0x00, /*OFF*/
                 // 0x10, /*Black*/
                    0x10, /*R2Y_SD_Limit*/
                    0x10, /*R2Y_SD_Full*/
                    0x10, /*R2Y_HD_Limit*/
                    0x10, /*R2Y_HD_Full*/
                 // 0x10, /*Y2R_SD_Limit*/
                 // 0x10, /*Y2R_SD_Full*/
                 // 0x10, /*Y2R_HD_Limit*/
                 // 0x10, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_70_L), 0x08, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x00, /*R2Y_SD_Limit*/
                    0x00, /*R2Y_SD_Full*/
                    0x00, /*R2Y_HD_Limit*/
                    0x00, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x40, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x40, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_70_L), 0x04, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x00, /*R2Y_SD_Limit*/
                    0x00, /*R2Y_SD_Full*/
                    0x00, /*R2Y_HD_Limit*/
                    0x00, /*R2Y_HD_Full*/
                 // 0x04, /*Y2R_SD_Limit*/
                 // 0x04, /*Y2R_SD_Full*/
                 // 0x04, /*Y2R_HD_Limit*/
                 // 0x04, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_70_L), 0x01, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x00, /*R2Y_SD_Limit*/
                    0x00, /*R2Y_SD_Full*/
                    0x00, /*R2Y_HD_Limit*/
                    0x00, /*R2Y_HD_Full*/
                 // 0x01, /*Y2R_SD_Limit*/
                 // 0x01, /*Y2R_SD_Full*/
                 // 0x01, /*Y2R_HD_Limit*/
                 // 0x01, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_71_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x0B, /*R2Y_SD_Limit*/
                    0xC2, /*R2Y_SD_Full*/
                    0x0B, /*R2Y_HD_Limit*/
                    0xC2, /*R2Y_HD_Full*/
                 // 0x7C, /*Y2R_SD_Limit*/
                 // 0x62, /*Y2R_SD_Full*/
                 // 0x29, /*Y2R_HD_Limit*/
                 // 0x2C, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_72_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x4A, /*R2Y_SD_Limit*/
                    0x87, /*R2Y_SD_Full*/
                    0x25, /*R2Y_HD_Limit*/
                    0x67, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0xA8, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0xA8, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_73_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0xAB, /*R2Y_SD_Limit*/
                    0xB7, /*R2Y_SD_Full*/
                    0xD0, /*R2Y_HD_Limit*/
                    0xD7, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_74_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x32, /*R2Y_SD_Limit*/
                    0x07, /*R2Y_SD_Full*/
                    0xDA, /*R2Y_HD_Limit*/
                    0xBB, /*R2Y_HD_Full*/
                 // 0x3E, /*Y2R_SD_Limit*/
                 // 0xBF, /*Y2R_SD_Full*/
                 // 0x2A, /*Y2R_HD_Limit*/
                 // 0xDD, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_75_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x59, /*R2Y_SD_Limit*/
                    0x04, /*R2Y_SD_Full*/
                    0xDC, /*R2Y_HD_Limit*/
                    0x75, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0xA8, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0xA8, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_76_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x75, /*R2Y_SD_Limit*/
                    0x64, /*R2Y_SD_Full*/
                    0x4A, /*R2Y_HD_Limit*/
                    0x3F, /*R2Y_HD_Full*/
                 // 0xA8, /*Y2R_SD_Limit*/
                 // 0x70, /*Y2R_SD_Full*/
                 // 0x45, /*Y2R_HD_Limit*/
                 // 0x26, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_77_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x50, /*R2Y_SD_Limit*/
                    0x68, /*R2Y_SD_Full*/
                    0x88, /*R2Y_HD_Limit*/
                    0x99, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_78_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0xA5, /*R2Y_SD_Limit*/
                    0xD6, /*R2Y_SD_Full*/
                    0x6D, /*R2Y_HD_Limit*/
                    0xA6, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0xA8, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0xA8, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_79_L), 0xFF, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x0B, /*R2Y_SD_Limit*/
                    0xC2, /*R2Y_SD_Full*/
                    0x0B, /*R2Y_HD_Limit*/
                    0xC2, /*R2Y_HD_Full*/
                 // 0xEE, /*Y2R_SD_Limit*/
                 // 0x12, /*Y2R_SD_Full*/
                 // 0x44, /*Y2R_HD_Limit*/
                 // 0x76, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_71_H), 0x1F, 0x04, /*OFF*/
                 // 0x00, /*Black*/
                    0x02, /*R2Y_SD_Limit*/
                    0x01, /*R2Y_SD_Full*/
                    0x02, /*R2Y_HD_Limit*/
                    0x01, /*R2Y_HD_Full*/
                 // 0x05, /*Y2R_SD_Limit*/
                 // 0x06, /*Y2R_SD_Full*/
                 // 0x06, /*Y2R_HD_Limit*/
                 // 0x07, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_72_H), 0x1F, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x1E, /*R2Y_SD_Limit*/
                    0x1E, /*R2Y_SD_Full*/
                    0x1E, /*R2Y_HD_Limit*/
                    0x1E, /*R2Y_HD_Full*/
                 // 0x04, /*Y2R_SD_Limit*/
                 // 0x04, /*Y2R_SD_Full*/
                 // 0x04, /*Y2R_HD_Limit*/
                 // 0x04, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_73_H), 0x1F, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x1F, /*R2Y_SD_Limit*/
                    0x1F, /*R2Y_SD_Full*/
                    0x1F, /*R2Y_HD_Limit*/
                    0x1F, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_74_H), 0x1F, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x01, /*R2Y_SD_Limit*/
                    0x01, /*R2Y_SD_Full*/
                    0x00, /*R2Y_HD_Limit*/
                    0x00, /*R2Y_HD_Full*/
                 // 0x1D, /*Y2R_SD_Limit*/
                 // 0x1C, /*Y2R_SD_Full*/
                 // 0x1E, /*Y2R_HD_Limit*/
                 // 0x1D, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_75_H), 0x1F, 0x04, /*OFF*/
                 // 0x00, /*Black*/
                    0x02, /*R2Y_SD_Limit*/
                    0x02, /*R2Y_SD_Full*/
                    0x02, /*R2Y_HD_Limit*/
                    0x02, /*R2Y_HD_Full*/
                 // 0x04, /*Y2R_SD_Limit*/
                 // 0x04, /*Y2R_SD_Full*/
                 // 0x04, /*Y2R_HD_Limit*/
                 // 0x04, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_76_H), 0x1F, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x00, /*R2Y_SD_Limit*/
                    0x00, /*R2Y_SD_Full*/
                    0x00, /*R2Y_HD_Limit*/
                    0x00, /*R2Y_HD_Full*/
                 // 0x1E, /*Y2R_SD_Limit*/
                 // 0x1E, /*Y2R_SD_Full*/
                 // 0x1F, /*Y2R_HD_Limit*/
                 // 0x1F, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_77_H), 0x1F, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x1F, /*R2Y_SD_Limit*/
                    0x1F, /*R2Y_SD_Full*/
                    0x1F, /*R2Y_HD_Limit*/
                    0xFF, /*R2Y_HD_Full*/
                 // 0x00, /*Y2R_SD_Limit*/
                 // 0x00, /*Y2R_SD_Full*/
                 // 0x00, /*Y2R_HD_Limit*/
                 // 0x00, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_78_H), 0x1F, 0x00, /*OFF*/
                 // 0x00, /*Black*/
                    0x1E, /*R2Y_SD_Limit*/
                    0x1E, /*R2Y_SD_Full*/
                    0x1E, /*R2Y_HD_Limit*/
                    0xFE, /*R2Y_HD_Full*/
                 // 0x04, /*Y2R_SD_Limit*/
                 // 0x04, /*Y2R_SD_Full*/
                 // 0x04, /*Y2R_HD_Limit*/
                 // 0x04, /*Y2R_HD_Full*/
 },
 { DRV_REG(REG_HDMI444To422_79_H), 0x1F, 0x04, /*OFF*/
                 // 0x00, /*Black*/
                    0x02, /*R2Y_SD_Limit*/
                    0x01, /*R2Y_SD_Full*/
                    0x02, /*R2Y_HD_Limit*/
                    0x01, /*R2Y_HD_Full*/
                 // 0x06, /*Y2R_SD_Limit*/
                 // 0x08, /*Y2R_SD_Full*/
                 // 0x07, /*Y2R_HD_Limit*/
                 // 0x08, /*Y2R_HD_Full*/
 },
};

MS_TX_3X3_MATRIX_INFO stHALTX_TIMING_3X3MATRIX_TBL[MS_TX_3X3_MATRIX_TAB_NUM]=
{
    {*MST_INIT_3x3_MATRIX_TBL,MS_INIT_3x3_MATRIX_REG_NUM, MS_TX_3X3_MATRIX_IP_NORMAL},
    {*MST_INIT_3x3_MATRIX_COMMON_TBL,MS_INIT_3x3_MATRIX_COMMON_REG_NUM, MS_TX_3X3_MATRIX_IP_COMMON},
};
#endif
