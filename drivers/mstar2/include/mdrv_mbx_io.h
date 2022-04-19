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
// @file   mdrv_mbx_io.h
// @brief  Mialbox KMD Driver Interface
// @author MStar Semiconductor Inc.
//
// Driver to initialize and access mailbox.
//     - Provide functions to initialize/de-initialize mailbox
//     - Provide mailbox functional access.
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_MBX_IO_H
#define _MDRV_MBX_IO_H

//=============================================================================
// Includs
//=============================================================================
#include "mdrv_mbx_st.h"

//=============================================================================
// Defines
//=============================================================================
//IO Ctrl defines:
#define MDRV_MBX_IOC_INIT_NR				0
#define MDRV_MBX_IOC_DEINIT_NR			MDRV_MBX_IOC_INIT_NR+1
#define MDRV_MBX_IOC_REGISTERMSG_NR		MDRV_MBX_IOC_DEINIT_NR+1
#define MDRV_MBX_IOC_UNREGISTERMSG_NR	MDRV_MBX_IOC_REGISTERMSG_NR+1
#define MDRV_MBX_IOC_CLEARMSG_NR         MDRV_MBX_IOC_UNREGISTERMSG_NR+1
#define MDRV_MBX_IOC_SENDMSG_NR			MDRV_MBX_IOC_CLEARMSG_NR+1
#define MDRV_MBX_IOC_SENDMSGLOOPBACK_NR MDRV_MBX_IOC_SENDMSG_NR+1
#define MDRV_MBX_IOC_RECVMSG_NR			MDRV_MBX_IOC_SENDMSGLOOPBACK_NR+1
#define MDRV_MBX_IOC_GETMSGQSTATUS_NR	MDRV_MBX_IOC_RECVMSG_NR+1
#define MDRV_MBX_IOC_MBXENABLE_NR		MDRV_MBX_IOC_GETMSGQSTATUS_NR+1
#define MDRV_MBX_IOC_SETINFORMATION_NR   MDRV_MBX_IOC_MBXENABLE_NR+1
#define MDRV_MBX_IOC_GETINFORMATION_NR   MDRV_MBX_IOC_SETINFORMATION_NR+1
#define MDRV_MBX_IOC_GETDRVSTATUS_NR     MDRV_MBX_IOC_GETINFORMATION_NR+1
#define MDRV_MBX_IOC_CHECKMSG_NR		MDRV_MBX_IOC_GETDRVSTATUS_NR+1
#define MDRV_MBX_IOC_REMOVEMSG_NR		MDRV_MBX_IOC_CHECKMSG_NR+1
#define MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO_NR       MDRV_MBX_IOC_REMOVEMSG_NR+1
#define MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO_NR       MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO_NR+1

#define MDRV_MBX_IOC_MAX_NR				MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO_NR +1


// use 'm' as magic number
#define MDRV_MBX_IOC_MAGIC			  'm'
#define MDRV_MBX_IOC_INIT         	  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_INIT_NR, MS_MBX_INIT_INFO)
#define MDRV_MBX_IOC_DEINIT			  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_DEINIT_NR, MS_MBX_SET_BINFO)
#define MDRV_MBX_IOC_REGISTERMSG		  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_REGISTERMSG_NR, MS_MBX_REGISTER_MSG)
#define MDRV_MBX_IOC_UNREGISTERMSG	  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_UNREGISTERMSG_NR, MS_MBX_UNREGISTER_MSG)
#define MDRV_mBX_IOC_CLEARMSG          _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_CLEARMSG_NR, MS_MBX_CLEAR_MSG)
#define MDRV_MBX_IOC_SENDMSG			  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_SENDMSG_NR, MS_MBX_SEND_MSG)
#define MDRV_MBX_IOC_RECVMSG			  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_RECVMSG_NR, MS_MBX_RECV_MSG)
#define MDRV_MBX_IOC_CHECKMSG           _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_CHECKMSG_NR, MS_MBX_RECV_MSG)
#define MDRV_MBX_IOC_REMOVEMSG           _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_REMOVEMSG_NR, MS_MBX_SET_BINFO)
#define MDRV_MBX_IOC_GETMSGQSTATUS	  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_GETMSGQSTATUS_NR, MS_MBX_GET_MSGQSTATUS)
#define MDRV_MBX_IOC_SENDMSGLOOPBACK   _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_SENDMSGLOOPBACK_NR, MS_MBX_SEND_MSG)
#define MDRV_MBX_IOC_MBXENABLE		  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_MBXENABLE_NR, MS_MBX_SET_BINFO)
#define MDRV_MBX_IOC_SETINFORMATION	  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_SETINFORMATION_NR, MS_MBX_CPROSYNC_INFORMATION)
#define MDRV_MBX_IOC_GETINFORMATION	  _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_GETINFORMATION_NR, MS_MBX_CPROSYNC_INFORMATION)
#define MDRV_MBX_IOC_GETDRVSTATUS      _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_MBX_IOC_GETDRVSTATUS_NR, MS_MBX_GET_DRVSTATUS)

#define MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO    _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO_NR, MS_PM_BRICK_TERMINATOR_INFO)
#define MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO    _IOWR(MDRV_MBX_IOC_MAGIC, MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO_NR, MS_PM_BRICK_TERMINATOR_INFO)

#endif //_MDRV_MBX_IO_H
