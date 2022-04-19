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

#ifndef _HALTX_TIMING_3X3MATRIX_TBL_H_
#define _HALTX_TIMING_3X3MATRIX_TBL_H_

#ifdef __cplusplus
extern "C"
{
#endif

//#include "MsTypes.h"


////////////////////////////////////////////////////////////////////////////////
#define MS_INIT_3x3_MATRIX_REG_NUM        25
#define MS_INIT_3x3_MATRIX_COMMON_REG_NUM        2
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
typedef enum
{
    MS_TX_3X3_MATRIX_ID_OFF=0x0,
    MS_TX_3X3_MATRIX_ID_Black=0xFF,
    MS_TX_3X3_MATRIX_ID_R2Y_SD_Limit=0x1,
    MS_TX_3X3_MATRIX_ID_R2Y_SD_Full=0x2,
    MS_TX_3X3_MATRIX_ID_R2Y_HD_Limit=0x3,
    MS_TX_3X3_MATRIX_ID_R2Y_HD_Full=0x4,
    MS_TX_3X3_MATRIX_ID_Y2R_SD_Limit=0xFF,
    MS_TX_3X3_MATRIX_ID_Y2R_SD_Full=0xFF,
    MS_TX_3X3_MATRIX_ID_Y2R_HD_Limit=0xFF,
    MS_TX_3X3_MATRIX_ID_Y2R_HD_Full=0xFF,
    MS_TX_3X3_MATRIX_ID_NUM=0x5,
} E_MS_TX_3X3_MATRIX_ID_TYPE;

typedef enum
{
    MS_TX_3X3_MATRIX_TAB_INIT_3x3_MATRIX,
    MS_TX_3X3_MATRIX_COMMON_TAB_INIT_3x3_MATRIX,
    MS_TX_3X3_MATRIX_TAB_NUM,
} E_MS_TX_3X3_MATRIX_TAB_TYPE;

typedef enum
{
    MS_TX_3X3_MATRIX_IP_NORMAL,
    MS_TX_3X3_MATRIX_IP_COMMON
} E_MS_TX_3X3_MATRIX_IP_TYPE;

typedef struct
{
    MS_U8                 *pData;
    MS_U16                 u16RegNum;
    E_MS_TX_3X3_MATRIX_IP_TYPE enIPType;
} MS_TX_3X3_MATRIX_INFO;

extern MS_TX_3X3_MATRIX_INFO stHALTX_TIMING_3X3MATRIX_TBL[MS_TX_3X3_MATRIX_TAB_NUM];

#ifdef __cplusplus
}
#endif

#undef _DRVADCTBL_H_
#endif
