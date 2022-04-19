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
/// @file   mdrv_msos_io.h
/// @brief  kernel mode msos api
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __MDRV_MSOS_IO_H__
#define __MDRV_MSOS_IO_H__

//-------------------------------------------------------------------------------------------------
// [MSOS] Define
//-------------------------------------------------------------------------------------------------
#define MDRV_MSOS_DEV_NAME                 "/dev/semutex"

//-------------------------------------------------------------------------------------------------
// [MSOS] Structure and Enum
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [EVENT] Define
//-------------------------------------------------------------------------------------------------
#define MDRV_EVENT_BUF_SIZE                 (128)

#define MDRV_EVENT_IOC_MAGIC                'E'

#define MDRV_EVENT_IOC_INIT                 _IOWR(MDRV_EVENT_IOC_MAGIC, 0x00, MDRV_EVENT_PARM)
#define MDRV_EVENT_IOC_CREATE               _IOWR(MDRV_EVENT_IOC_MAGIC, 0x01, MDRV_EVENT_PARM)
#define MDRV_EVENT_IOC_DELETE               _IOWR(MDRV_EVENT_IOC_MAGIC, 0x02, MDRV_EVENT_PARM)
#define MDRV_EVENT_IOC_SET                  _IOWR(MDRV_EVENT_IOC_MAGIC, 0x03, MDRV_EVENT_PARM)
#define MDRV_EVENT_IOC_CLEAR                _IOWR(MDRV_EVENT_IOC_MAGIC, 0x04, MDRV_EVENT_PARM)
#define MDRV_EVENT_IOC_WAIT                 _IOWR(MDRV_EVENT_IOC_MAGIC, 0x05, MDRV_EVENT_PARM)

#if defined(CONFIG_COMPAT)
#define MDRV_EVENT_IOC_COMPAT_INIT          _IOWR(MDRV_EVENT_IOC_MAGIC, 0x00, MDRV_EVENT_PARM_COMPAT)
#define MDRV_EVENT_IOC_COMPAT_CREATE        _IOWR(MDRV_EVENT_IOC_MAGIC, 0x01, MDRV_EVENT_PARM_COMPAT)
#define MDRV_EVENT_IOC_COMPAT_DELETE        _IOWR(MDRV_EVENT_IOC_MAGIC, 0x02, MDRV_EVENT_PARM_COMPAT)
#define MDRV_EVENT_IOC_COMPAT_SET           _IOWR(MDRV_EVENT_IOC_MAGIC, 0x03, MDRV_EVENT_PARM_COMPAT)
#define MDRV_EVENT_IOC_COMPAT_CLEAR         _IOWR(MDRV_EVENT_IOC_MAGIC, 0x04, MDRV_EVENT_PARM_COMPAT)
#define MDRV_EVENT_IOC_COMPAT_WAIT          _IOWR(MDRV_EVENT_IOC_MAGIC, 0x05, MDRV_EVENT_PARM_COMPAT)
#endif

//-------------------------------------------------------------------------------------------------
// [EVENT] Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef struct _MDRV_EVENT_PARM
{
    unsigned int    Num;
    char            Name[MDRV_EVENT_BUF_SIZE];
    int             Id;
    unsigned int    Flag;
    unsigned int *  CmpFlag;
    unsigned int    Mode;
    unsigned int    WaitMs;
} MDRV_EVENT_PARM;

#if defined(CONFIG_COMPAT)
typedef struct _MDRV_EVENT_PARM_COMPAT
{
    unsigned int    Num;
    char            Name[MDRV_EVENT_BUF_SIZE];
    int             Id;
    unsigned int    Flag;
    compat_uptr_t   CmpFlag;
    unsigned int    Mode;
    unsigned int    WaitMs;
} MDRV_EVENT_PARM_COMPAT;
#endif

//-------------------------------------------------------------------------------------------------
// [QUEUE] Define
//-------------------------------------------------------------------------------------------------
#define MDRV_QUEUE_BUF_SIZE                 (128)

#define MDRV_QUEUE_IOC_MAGIC                'Q'

#define MDRV_QUEUE_IOC_INIT                 _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x00, MDRV_QUEUE_PARM)
#define MDRV_QUEUE_IOC_CREATE               _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x01, MDRV_QUEUE_PARM)
#define MDRV_QUEUE_IOC_DELETE               _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x02, MDRV_QUEUE_PARM)
#define MDRV_QUEUE_IOC_SEND                 _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x03, MDRV_QUEUE_PARM)
#define MDRV_QUEUE_IOC_RECV                 _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x04, MDRV_QUEUE_PARM)
#define MDRV_QUEUE_IOC_PEEK                 _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x05, MDRV_QUEUE_PARM)

#if defined(CONFIG_COMPAT)
#define MDRV_QUEUE_IOC_COMPAT_INIT          _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x00, MDRV_QUEUE_PARM_COMPAT)
#define MDRV_QUEUE_IOC_COMPAT_CREATE        _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x01, MDRV_QUEUE_PARM_COMPAT)
#define MDRV_QUEUE_IOC_COMPAT_DELETE        _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x02, MDRV_QUEUE_PARM_COMPAT)
#define MDRV_QUEUE_IOC_COMPAT_SEND          _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x03, MDRV_QUEUE_PARM_COMPAT)
#define MDRV_QUEUE_IOC_COMPAT_RECV          _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x04, MDRV_QUEUE_PARM_COMPAT)
#define MDRV_QUEUE_IOC_COMPAT_PEEK          _IOWR(MDRV_QUEUE_IOC_MAGIC, 0x05, MDRV_QUEUE_PARM_COMPAT)
#endif

//-------------------------------------------------------------------------------------------------
// [QUEUE] Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef struct _MDRV_QUEUE_PARM
{
    unsigned int        Num;
    int                 Id;
    unsigned int        QueueSize;
    unsigned int        MsgType;
    unsigned int        MsgAlignSize;
    unsigned int        Attr;
    char                Name[MDRV_QUEUE_BUF_SIZE];
    unsigned char *     MsgData;
    unsigned int        MsgSendSize;
    unsigned int *      MsgRecvSize;
    unsigned int        WaitMs;
} MDRV_QUEUE_PARM;

#if defined(CONFIG_COMPAT)
typedef struct _MDRV_QUEUE_PARM_COMPAT
{
    unsigned int        Num;
    int                 Id;
    unsigned int        QueueSize;
    unsigned int        MsgType;
    unsigned int        MsgAlignSize;
    unsigned int        Attr;
    char                Name[MDRV_QUEUE_BUF_SIZE];
    compat_uptr_t       MsgData;
    unsigned int        MsgSendSize;
    compat_uptr_t       MsgRecvSize;
    unsigned int        WaitMs;
} MDRV_QUEUE_PARM_COMPAT;
#endif
#endif
