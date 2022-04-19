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

#ifndef _MDRV_MBX_COMPAT_IO_H
#define _MDRV_MBX_COMPAT_IO_H

//=============================================================================
// Includs
//=============================================================================
#include "mdrv_mbx_compat_st.h"

//=============================================================================
// Defines
//=============================================================================
//IO Ctrl defines:
#define COMPAT_MDRV_MBX_IOC_INIT_NR				0
#define COMPAT_MDRV_MBX_IOC_DEINIT_NR			COMPAT_MDRV_MBX_IOC_INIT_NR+1
#define COMPAT_MDRV_MBX_IOC_REGISTERMSG_NR		COMPAT_MDRV_MBX_IOC_DEINIT_NR+1
#define COMPAT_MDRV_MBX_IOC_UNREGISTERMSG_NR	COMPAT_MDRV_MBX_IOC_REGISTERMSG_NR+1
#define COMPAT_MDRV_MBX_IOC_CLEARMSG_NR         COMPAT_MDRV_MBX_IOC_UNREGISTERMSG_NR+1
#define COMPAT_MDRV_MBX_IOC_SENDMSG_NR			COMPAT_MDRV_MBX_IOC_CLEARMSG_NR+1
#define COMPAT_MDRV_MBX_IOC_SENDMSGLOOPBACK_NR  COMPAT_MDRV_MBX_IOC_SENDMSG_NR+1
#define COMPAT_MDRV_MBX_IOC_RECVMSG_NR			COMPAT_MDRV_MBX_IOC_SENDMSGLOOPBACK_NR+1
#define COMPAT_MDRV_MBX_IOC_GETMSGQSTATUS_NR	COMPAT_MDRV_MBX_IOC_RECVMSG_NR+1
#define COMPAT_MDRV_MBX_IOC_MBXENABLE_NR		COMPAT_MDRV_MBX_IOC_GETMSGQSTATUS_NR+1
#define COMPAT_MDRV_MBX_IOC_SETINFORMATION_NR   COMPAT_MDRV_MBX_IOC_MBXENABLE_NR+1
#define COMPAT_MDRV_MBX_IOC_GETINFORMATION_NR   COMPAT_MDRV_MBX_IOC_SETINFORMATION_NR+1
#define COMPAT_MDRV_MBX_IOC_GETDRVSTATUS_NR     COMPAT_MDRV_MBX_IOC_GETINFORMATION_NR+1
#define COMPAT_MDRV_MBX_IOC_CHECKMSG_NR		    COMPAT_MDRV_MBX_IOC_GETDRVSTATUS_NR+1
#define COMPAT_MDRV_MBX_IOC_REMOVEMSG_NR		COMPAT_MDRV_MBX_IOC_CHECKMSG_NR+1
#define COMPAT_MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO_NR       COMPAT_MDRV_MBX_IOC_REMOVEMSG_NR+1
#define COMPAT_MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO_NR       COMPAT_MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO_NR+1
#define COMPAT_MDRV_MBX_IOC_MAX_NR				COMPAT_MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO_NR +1


// use 'm' as magic number
#define COMPAT_MDRV_MBX_IOC_MAGIC			  'm'
#define COMPAT_MDRV_MBX_IOC_INIT         	  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_INIT_NR, COMPAT_MS_MBX_INIT_INFO)
#define COMPAT_MDRV_MBX_IOC_DEINIT			  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_DEINIT_NR, COMPAT_MS_MBX_SET_BINFO)
#define COMPAT_MDRV_MBX_IOC_REGISTERMSG		  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_REGISTERMSG_NR, COMPAT_MS_MBX_REGISTER_MSG)
#define COMPAT_MDRV_MBX_IOC_UNREGISTERMSG	  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_UNREGISTERMSG_NR, COMPAT_MS_MBX_UNREGISTER_MSG)
#define COMPAT_MDRV_mBX_IOC_CLEARMSG          _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_CLEARMSG_NR, COMPAT_MS_MBX_CLEAR_MSG)
#define COMPAT_MDRV_MBX_IOC_SENDMSG			  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_SENDMSG_NR, COMPAT_MS_MBX_SEND_MSG)
#define COMPAT_MDRV_MBX_IOC_RECVMSG			  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_RECVMSG_NR, COMPAT_MS_MBX_RECV_MSG)
#define COMPAT_MDRV_MBX_IOC_CHECKMSG           _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_CHECKMSG_NR, COMPAT_MS_MBX_RECV_MSG)
#define COMPAT_MDRV_MBX_IOC_REMOVEMSG           _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_REMOVEMSG_NR, COMPAT_MS_MBX_SET_BINFO)
#define COMPAT_MDRV_MBX_IOC_GETMSGQSTATUS	  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_GETMSGQSTATUS_NR, COMPAT_MS_MBX_GET_MSGQSTATUS)
#define COMPAT_MDRV_MBX_IOC_SENDMSGLOOPBACK   _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_SENDMSGLOOPBACK_NR, COMPAT_MS_MBX_SEND_MSG)
#define COMPAT_MDRV_MBX_IOC_MBXENABLE		  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_MBXENABLE_NR, COMPAT_MS_MBX_SET_BINFO)
#define COMPAT_MDRV_MBX_IOC_SETINFORMATION	  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_SETINFORMATION_NR, COMPAT_MS_MBX_CPROSYNC_INFORMATION)
#define COMPAT_MDRV_MBX_IOC_GETINFORMATION	  _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_GETINFORMATION_NR, COMPAT_MS_MBX_CPROSYNC_INFORMATION)
#define COMPAT_MDRV_MBX_IOC_GETDRVSTATUS      _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_MBX_IOC_GETDRVSTATUS_NR, COMPAT_MS_MBX_GET_DRVSTATUS)

#define COMPAT_MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO    _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO_NR, COMPAT_MS_PM_BRICK_TERMINATOR_INFO)
#define COMPAT_MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO    _IOWR(COMPAT_MDRV_MBX_IOC_MAGIC, COMPAT_MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO_NR, COMPAT_MS_PM_BRICK_TERMINATOR_INFO)

#endif //_MDRV_MBX_COMPAT_IO_H
