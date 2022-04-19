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

/*-----------------------------------------------------------------------------
 * $Author: dtvbm11 $
 * $Date: 2015/04/15 $
 * $RCSfile: cb_low.h,v $
 * $Revision: #1 $
 *---------------------------------------------------------------------------*/

/** @file cb_low.h
 *  This header file declares low-layer support of callback interface.
 */

#ifndef CB2_LOW_H
#define CB2_LOW_H


//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------

#include "lnklist.h"
#include "mt53xx_cb2.h"
#include <linux/wait.h>

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

/// Callback event.
typedef struct _CB_CALLBACK_EVENT_T
{
    DLIST_ENTRY_T(_CB_CALLBACK_EVENT_T) rLink;
    #ifdef CC_MTAL_MULTIPROCESS_CALLBACK
    // This is used for multiple callback reference.
    UINT32              u4BitMask;
    #endif
    // This *MUST* be the last one member of CB_CALLBACK_EVENT_T
    CB_GET_CALLBACK_T   rGetCb;
} CB_CALLBACK_EVENT_T;

typedef struct _CB_EVENT_COUNT_T
{
    UINT32 puCB00IdleFctPid[CB_FCT_NUM];
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK	
	UINT32 puCB01IdleFctPid[CB_FCT_NUM];
	UINT32 puCB02IdleFctPid[CB_FCT_NUM];
#endif
} CB_EVENT_COUNT_T;

#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
#define BITMASK_SINGLE              (1U << 0)
#define BITMASK_MULTI1              (1U << 1)
#define BITMASK_MULTI2              (1U << 2)
#define BITMASK_MULTI3              (1U << 3)
#define BITMASK_MULTI(i)            (1U << (i))
#endif

#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
#define CB_MULTIPROCESS             (1)
#define CB_MULTIPROCESS_REPLACE     (1)
#define CB_MULTIPROCESS_LAST_REPLACE     (1)
#else
#define CB_MULTIPROCESS             (0)
#define CB_MULTIPROCESS_REPLACE     (0)
#endif

//-----------------------------------------------------------------------------
// Inter-file functions
//-----------------------------------------------------------------------------

void _CB_Init(void);
INT32 _CB_RegCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid);
pid_t _CB_GetPid(CB_FCT_ID_T eFctId);
int _CB_GetCbCount(void);
BOOL _CB_ParsingEventQueue(CB_EVENT_COUNT_T *);
INT32 _CB_UnRegCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid);
INT32 _CB_PutEvent(CB_FCT_ID_T eFctId, INT32 i4TagSize, void *pvTag);
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
INT32 _CB_RegMultiCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid);
INT32 _CB_UnRegMultiCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid);
void _CB_FreeEvent(CB_CALLBACK_EVENT_T *prCbEv, UINT32 u4DoneMask);
CB_CALLBACK_EVENT_T *_CB_GetEvent(UINT32 *pu4DoneMask);
#else
CB_CALLBACK_EVENT_T *_CB_GetEvent(void);
#endif
//void _CB_PutThdIntoWQ(void);

extern wait_queue_head_t _rWq;
#define  _CB_PutThdIntoWQ(condition) wait_event_interruptible(_rWq, condition)



void _CB_ClearPendingEventEntries(void);
void _CB_ClearPendingFctEventEntries(CB_FCT_ID_T eFctId);
void _CB_ClearCbIdArray(void);

#if defined(CC_SUPPORT_STR)
int mt53xx_STR_init(void);
#endif

#endif //CB_LOW_H

