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
///
/// @file   mdrv_hdmitx4vx1.h
/// @brief  hdmitx4vx1 Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_HDMITX4VX1_H_
#define _MDRV_HDMITX4VX1_H_

#include <linux/fs.h>
#include <linux/cdev.h>
#include "mdrv_types.h"
#include <linux/version.h>

#include "mdrv_mstypes.h"
#include "mdrv_hdmitx4vx1_st.h"

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define HDMITX4VX1_DEBUG_ENABLE 1
#if (HDMITX4VX1_DEBUG_ENABLE==1)
#define     HDMITX4VX1_KDBG(_fmt, _args...)        printk("[HDMITX4VX1][%s:%05d] " _fmt, __FUNCTION__, __LINE__, ## _args)
#else
#define     HDMITX4VX1_KDBG(fmt, args...)
#endif


//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

/// Enum for load script
typedef enum
{
    E_TX_TIMING_3X3_MATRIX_AUTO,
    E_TX_TIMING_3X3_MATRIX_OFF,
    E_TX_TIMING_3X3_MATRIX_R2Y_SD_LIMIT,
    E_TX_TIMING_3X3_MATRIX_R2Y_SD_FULL,
    E_TX_TIMING_3X3_MATRIX_R2Y_HD_LIMIT,
    E_TX_TIMING_3X3_MATRIX_R2Y_HD_FULL,
    E_TX_TIMING_3X3_MATRIX_NUM,
}E_TX_TIMING_3X3_MATRIX_TYPE;


//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------

#define E_TX_TIMING_UART_MODE_1     0x01
#define E_TX_TIMING_UART_MODE_2     0x02
#define E_TX_TIMING_UART_MODE_3     0x03
#define VX1_LOCKN_STATE3_TIMEOUT     10
//#define BIT0  0x000001
//#define BIT1  0x000002
//#define BIT2  0x000004
//#define BIT3  0x000008
//#define BIT4  0x000010
//#define BIT5  0x000020
//#define BIT6  0x000040
//#define BIT7  0x000080
//#define BIT8  0x000100
#define HDMITX_AVI_INFO_PKT_VER    0x02
#define HDMITX_AVI_INFO_PKT_LEN    0x0D
#define E_HDMITX_AVI_INFOFRAME      0x82


void HDMITX4VX1_Probe(void);
MS_BOOL HDMITX4VX1_Init(EN_HDMITX4VX1_LOCKN_TYPE enLOCKNType);
MS_BOOL HDMITX4VX1_Video_config(ST_MAILBOX_COMMAND_CONFIQ_VIDEO stVideo);
MS_BOOL HDMITX4VX1_Audio_config(ST_MAILBOX_COMMAND_CONFIQ_AUDIO stAudio);
MS_BOOL HDMITX4VX1_TimingChange_AVmute(ST_MAILBOX_COMMAND_TIMING_CHANGE_AVMUTE stChange);
MS_BOOL HDMITX4VX1_QUERY_4K2KVIC(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_4K2KVIC *get4k2k);
MS_BOOL HDMITX4VX1_QUERY_3D_Structure(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_3DSTRUCT *get3D);
MS_BOOL HDMITX4VX1_QUERY_Num_AudioDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES_NUM *getNUM);
MS_BOOL HDMITX4VX1_QUERY_AudioDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_AUDIODES *getAudio);
MS_BOOL HDMITX4VX1_QUERY_Num_VideoDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES_NUM *getNUM);
MS_BOOL HDMITX4VX1_QUERY_VideoDescriptor(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_VIDEODES *getVideo);
MS_BOOL HDMITX4VX1_QUERY_EDID_RawData(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_EDIDRAWDATA *getRaw);
MS_BOOL HDMITX4VX1_QUERY_RxEdidInfo(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_RX_EDID_INFO *getEDID);
MS_BOOL HDMITX4VX1_QUERY_ColorFormat(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_COLORFMT *getcolor);
MS_BOOL HDMITX4VX1_QUERY_HWInfo(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HWINFO *getHW);
MS_BOOL HDMITX4VX1_QUERY_KSV_BStatus(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_KSV_BSTATUS *getKSVB);
MS_BOOL HDMITX4VX1_QUERY_HDCPKey_Status(ST_MAILBOX_COMMAND_QUERY_EDID_INFO stChange,ST_MAILBOX_COMMAND_REPORT_EDID_INFO_HDCPKEY_STATUS *getKSVB);
MS_BOOL HDMITX4VX1_HDCP_CMD(ST_MAILBOX_COMMAND_HDCP_COMD stHDCP);
int HDMITX4VX1_GetRxStatus(void);
void HDMITX4VX1_loadKeepOutputScript(EN_HDMITX4VX1_VIDEO_TIMING loadtiming,EN_HDMITX4VX1_COLOR_DEPTH loadcolordepth,EN_HDMITX4VX1_OUTPUT_COLOR_TYPE loadoutcolorfmt);
void HDMITX4VX1_ReadRegisterForce(int iSize, ST_HDMITX4VX1_REGISTER *stBuf);
void HDMITX4VX1_WriteRegisterMaskForce(int iSize, ST_HDMITX4VX1_REGISTER *stBuf);
#endif // _MDRV_HDMITX4VX1_H_
