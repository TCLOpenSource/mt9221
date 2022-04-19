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
// @file   mdrv_hdmitx4vx1_io.h
// @brief  hdmitx4vx1 Driver Interface
// @author MStar Semiconductor Inc.
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_HDMITX4VX1_IO_H
#define _MDRV_HDMITX4VX1_IO_H

//=============================================================================
// Includs
//=============================================================================
#include "mdrv_hdmitx4vx1_st.h"
//=============================================================================
// Defines
//=============================================================================
//IO Ctrl defines:
#define HDMITX4VX1_INIT_IOC_NR                          (0)
#define HDMITX4VX1_VIDEO_CONFIG_IOC_NR                  (1)
#define HDMITX4VX1_AUDIO_CONFIG_IOC_NR                  (2)
#define HDMITX4VX1_TIMINGCHANGE_AVMUTE_IOC_NR           (3)
#define HDMITX4VX1_QUERY_4K2KVIC_IOC_NR                 (4)
#define HDMITX4VX1_QUERY_3DSTRUCTURE_IOC_NR             (5)
#define HDMITX4VX1_QUERY_AUDIODESCRIPTOR_NUM_IOC_NR     (6)
#define HDMITX4VX1_QUERY_AUDIODESCRIPTOR_IOC_NR         (7)
#define HDMITX4VX1_QUERY_VIDEDESCRIPTOR_NUM_IOC_NR      (8)
#define HDMITX4VX1_QUERY_VIDEODESCRIPTOR_IOC_NR         (9)
#define HDMITX4VX1_QUERY_EDIDRAWDATA_IOC_NR             (10)
#define HDMITX4VX1_QUERY_RXEDIDINFO_IOC_NR              (11)
#define HDMITX4VX1_QUERY_COLORFORMAT_IOC_NR             (12)
#define HDMITX4VX1_QUERY_HWINFO_IOC_NR                  (13)
#define HDMITX4VX1_QUERY_KSVBSTATUS_IOC_NR              (14)
#define HDMITX4VX1_QUERY_HDCPKEYSTATUS_IOC_NR           (15)
#define HDMITX4VX1_HDCP_COMMAND_IOC_NR                  (16)
#define HDMITX4VX1_GET_RXSTATUS_IOC_NR                  (17)
#define HDMITX4VX1_LOADOUTPUTSCRIPT_IOC_NR              (18)
#define HDMITX4VX1_READREGISTER_IOC_NR                  (19)
#define HDMITX4VX1_WRITEREGISTER_MASK_IOC_NR            (20)


#define HDMITX4VX1_MAX_IOC_NR                           (21)

#define IOCTL_HDMITX4VX1_MAGIC      ('r')
#define IOCTL_HDMITX4VX1_INIT                       _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_INIT_IOC_NR)
#define IOCTL_HDMITX4VX1_VIDEO_CONFIG               _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_VIDEO_CONFIG_IOC_NR)
#define IOCTL_HDMITX4VX1_AUDIO_CONFIG               _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_AUDIO_CONFIG_IOC_NR)
#define IOCTL_HDMITX4VX1_TIMINGCHANGEAVMUTE_CONFIG  _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_TIMINGCHANGE_AVMUTE_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_4K2KVIC              _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_4K2KVIC_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_3DSTRUCTURE          _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_3DSTRUCTURE_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_AUDIODESCRIPTOR_NUM  _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_AUDIODESCRIPTOR_NUM_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_AUDIODESCRIPTOR      _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_AUDIODESCRIPTOR_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_VIDEODESCRIPTOR_NUM  _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_VIDEDESCRIPTOR_NUM_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_VIDEODESCRIPTOR      _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_VIDEODESCRIPTOR_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_EDIDRAWDATA          _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_EDIDRAWDATA_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_RXEDIDINFO           _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_RXEDIDINFO_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_COLORFORMAT          _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_COLORFORMAT_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_HWINFO               _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_HWINFO_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_KSVBSTATUS           _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_KSVBSTATUS_IOC_NR)
#define IOCTL_HDMITX4VX1_QUERY_HDCPKEYSTATUS        _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_QUERY_HDCPKEYSTATUS_IOC_NR)
#define IOCTL_HDMITX4VX1_HDCP_COMMAND               _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_HDCP_COMMAND_IOC_NR)
#define IOCTL_HDMITX4VX1_GET_RXSTATUS               _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_GET_RXSTATUS_IOC_NR)
#define IOCTL_HDMITX4VX1_LOADOUTPUTSCRIPT           _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_LOADOUTPUTSCRIPT_IOC_NR)
#define IOCTL_HDMITX4VX1_READREGISTER               _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_READREGISTER_IOC_NR)
#define IOCTL_HDMITX4VX1_WRITEREGISTER_MASK         _IO(IOCTL_HDMITX4VX1_MAGIC, HDMITX4VX1_WRITEREGISTER_MASK_IOC_NR)




#endif //_MDRV_HDMITX4VX1_IO_H
