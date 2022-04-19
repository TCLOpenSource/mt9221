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
// @file   mdrv_mbx_st.h
// @brief  Mialbox KMD Driver Interface
// @author MStar Semiconductor Inc.
//
// Driver to initialize and access mailbox.
//     - Provide functions to initialize/de-initialize mailbox
//     - Provide mailbox functional access.
//////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MDRV_MBX_COMPAT_ST_H
#define _MDRV_MBX_COMPAT_ST_H

//=============================================================================
// Includs
//=============================================================================

//=============================================================================
// Type and Structure Declaration
//=============================================================================
//IO Ctrl struct defines:

typedef struct  __attribute__((packed))
{
    /// Mail message Queue status,
    /// @ref MBX_STATUS_QUEUE_OVER_FLOW
    /// @ref MBX_STATUS_QUEUE_HAS_INSTANT_MSG
    /// @ref MBX_STATUS_QUEUE_HAS_NORMAL_MSG
    compat_long_t status;
    /// pended normal message count in class message queue
    compat_long_t u32NormalMsgCount;
    /// pended Instant message count in class message queue
    compat_long_t u32InstantMsgCount;
}COMPAT_MBX_MSGQ_Status;

//=============================================================================

typedef struct   __attribute__((packed))
{
    MBX_CPU_ID eHKCPU;
    MBX_ROLE_ID eHostRole;
    compat_long_t u32TimeoutMillSecs;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_INIT_INFO, *PCOMPAT_MS_MBX_INIT_INFO;

typedef struct  __attribute__((packed))
{
    MS_BOOL  bInfo;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_SET_BINFO, *PCOMPAT_MS_MBX_SET_BINFO;

typedef struct  __attribute__((packed))
{
    MBX_Class eMsgClass;
    compat_int_t 	u16MsgQueueSize;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_REGISTER_MSG, *PCOMPAT_MS_MBX_REGISTER_MSG;

typedef struct  __attribute__((packed))
{
    MBX_Class eMsgClass;
    MS_BOOL bForceDiscardMsgQueue;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_UNREGISTER_MSG, *PCOMPAT_MS_MBX_UNREGISTER_MSG;

typedef struct  __attribute__((packed))
{
    MBX_Class eMsgClass;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_CLEAR_MSG, *PCOMPAT_MS_MBX_CLEAR_MSG;


typedef struct  __attribute__((packed))
{
    MBX_Class eTargetClass;
    compat_uptr_t pMsg;
    compat_long_t u32WaitMillSecs;
    compat_long_t u32Flag;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_RECV_MSG, *PCOMPAT_MS_MBX_RECV_MSG;

typedef struct __attribute__((packed))
{
    compat_uptr_t pMsg;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_SEND_MSG, *PCOMPAT_MS_MBX_SEND_MSG;

typedef struct  __attribute__((packed))
{
    MBX_Class eTargetClass;
    compat_uptr_t pMsgQueueStatus;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_GET_MSGQSTATUS, *PCOMPAT_MS_MBX_GET_MSGQSTATUS;

typedef struct  __attribute__((packed))
{
    MS_BOOL bEnabled;
    compat_long_t  s32RefCnt;
}COMPAT_MS_MBX_GET_DRVSTATUS, *PCOMPAT_MS_MBX_GET_DRVSTATUS;

typedef struct  __attribute__((packed))
{
    MBX_ROLE_ID eTargetRole;
    compat_uptr_t pU8Info;
    MS_U8 u8Size;
    MBX_Result mbxResult;
}COMPAT_MS_MBX_CPROSYNC_INFORMATION, *PCOMPAT_MS_MBX_CPROSYNC_INFORMATION;

typedef struct  __attribute__((packed))
{
    MS_U8 u8Value;
}COMPAT_MS_PM_BRICK_TERMINATOR_INFO, *PCOMPAT_MS_PM_BRICK_TERMINATOR_INFO;

#endif //_MDRV_MBX_COMPAT_ST_H
