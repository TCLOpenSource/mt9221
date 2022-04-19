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
/// @file   mdrv_msos.h
/// @brief  kernel mode msos api
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __MDRV_MSOS_H__
#define __MDRV_MSOS_H__

#include "mdrv_types.h"

//-------------------------------------------------------------------------------------------------
// [MSOS] Define
//-------------------------------------------------------------------------------------------------
#define MDRV_MSOS_DEBUG(fmt, args...)   printk(KERN_ERR "[%s][%d] " fmt, __func__, __LINE__, ## args)

#define MDRV_MSOS_WAIT_FOREVER          (0xFFFFFF00)

//-------------------------------------------------------------------------------------------------
// [MSOS] Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_MSOS_ATTR_PRIORITY,
    E_MSOS_ATTR_FIFO,
} MDRV_MSOS_ATTR;

//-------------------------------------------------------------------------------------------------
// [MSOS] Function and Variable
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
// [EVENT] Define
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [EVENT] Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_AND,
    E_OR,
    E_AND_CLEAR,
    E_OR_CLEAR,
} MDRV_EVENT_MODE;

//-------------------------------------------------------------------------------------------------
//  [EVNET] Function and Variable
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/// Initialize an event group
/// @param  u32GroupNum     \b IN: number of event group
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Init(U32 u32GroupNum);

//-------------------------------------------------------------------------------------------------
/// Create an event group
/// @param  pGroupName      \b IN: event group name
/// @return >=0 : assigned Event Id
/// @return <0 : fail
//-------------------------------------------------------------------------------------------------
S32 MDrv_Event_Create(char *pGroupName);

//-------------------------------------------------------------------------------------------------
/// Delete the event group
/// @param  s32GroupId      \b IN: event group ID
/// @return TRUE : succeed
/// @return FALSE : fail, sb is waiting for the event flag
/// @note event group that are being waited on must not be deleted
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Delete(S32 s32GroupId);

//-------------------------------------------------------------------------------------------------
/// Set the event flag in the specified event group
/// @param  s32GroupId      \b IN: event group ID
/// @param  u32GroupFlag    \b IN: event flag value
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Set(S32 s32GroupId, U32 u32GroupFlag);

//-------------------------------------------------------------------------------------------------
/// Clear the specified event flag in the specified event group
/// @param  s32GroupId      \b IN: event group ID
/// @param  u32GroupFlag    \b IN: event flag value
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Clear(S32 s32GroupId, U32 u32GroupFlag);

//-------------------------------------------------------------------------------------------------
/// Wait for the specified event flag combination from the event group
/// @param  s32GroupId      \b IN: event group ID
/// @param  u32GroupFlag    \b IN: wait event flag value
/// @param  pu32CmpGroupFlag\b OUT: retrieved event flag value
/// @param  eGroupMode      \b IN: E_AND/E_OR/E_AND_CLEAR/E_OR_CLEAR
/// @param  u32WaitMs       \b IN: 0 ~ MDRV_MSOS_WAIT_FOREVER: suspend time (ms) if the event is not ready
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Wait(S32 s32GroupId, U32 u32GroupFlag,
                     U32 *pu32CmpGroupFlag,
                     MDRV_EVENT_MODE eGroupMode,
                     U32 u32WaitMs);

//-------------------------------------------------------------------------------------------------
/// Wait for the specified event flag combination from the event group
/// @param  s32GroupId      \b IN: event group ID
/// @param  u32GroupFlag    \b IN: wait event flag value
/// @param  pu32CmpGroupFlag\b OUT: retrieved event flag value
/// @param  eGroupMode      \b IN: E_AND/E_OR/E_AND_CLEAR/E_OR_CLEAR
/// @param  u32WaitMs       \b IN: 0 ~ MDRV_MSOS_WAIT_FOREVER: suspend time (ms) if the event is not ready
/// @return TRUE : succeed
/// @return FALSE : fail
/// @return -ERESTARTSYS : interrupt by  ERESTARTSYS
//-------------------------------------------------------------------------------------------------
S32 MDrv_Event_Wait_Interrupt(S32 s32GroupId, U32 u32GroupFlag,
                              U32 *pu32CmpGroupFlag,
                              MDRV_EVENT_MODE eGroupMode,
                              U32 u32WaitMs);


//-------------------------------------------------------------------------------------------------
// [QUEUE] Define
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [QUEUE] Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_QUEUE_MSG_FIXED_SIZE,
    E_QUEUE_MSG_VAR_SIZE,
} MDRV_QUEUE_TYPE;

//-------------------------------------------------------------------------------------------------
// [QUEUE] Function and Variable
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/// Initialize a queue
/// @param  u32QueueNum     \b IN: number of queue
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Init(U32 u32QueueNum);

//-------------------------------------------------------------------------------------------------
/// Create a Queue
/// @param  pStartAddr      \b IN: It is useless now, can pass NULL.
/// @param  u32QueueSize    \b IN: queue size (byte unit) : now fixed as
///                                MSOS_QUEUE_SIZE * u32MessageSize
/// @param  eMessageType    \b IN: E_MSG_FIXED_SIZE / E_MSG_VAR_SIZE
/// @param  u32MessageSize  \b IN: message size (byte unit) for E_MSG_FIXED_SIZE
///                                max message size (byte unit) for E_MSG_VAR_SIZE
/// @param  eAttribute      \b IN: E_MSOS_FIFO suspended in FIFO order
/// @param  pQueueName      \b IN: queue name
/// @return assigned message queue ID
/// @return < 0 - fail
//-------------------------------------------------------------------------------------------------
S32 MDrv_Queue_Create(void           *pStartAddr,
                      U32             u32QueueSize,
                      MDRV_QUEUE_TYPE eMessageType,
                      U32             u32MessageSize,
                      MDRV_MSOS_ATTR  eAttribute,
                      char            *pQueueName);

//-------------------------------------------------------------------------------------------------
/// Delete the Queue
/// @param  s32QueueId  \b IN: Queue ID
/// @return TRUE : succeed
/// @return FALSE :  fail
/// @note   It is important that there are not any threads blocked on the queue
///             when this function is called or the behavior is undefined.
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Delete(S32 s32QueueId);

//-------------------------------------------------------------------------------------------------
/// Send a message to the end of the specified queue
/// @param  s32QueueId  \b IN: Queue ID
/// @param  pu8Message  \b IN: ptr to msg to send. NULL ptr is not allowed
/// @param  u32Size     \b IN: msg size (byte)
/// @param  u32WaitMs   \b IN: 0 ~ MDRV_MSOS_WAIT_FOREVER: suspend time (ms) if the queue is full
/// @return TRUE : succeed
/// @return FALSE :  fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Send(S32 s32QueueId, U8 *pu8Message, U32 u32Size, U32 u32WaitMs);

//-------------------------------------------------------------------------------------------------
/// Receive a message from the specified queue
/// @ingroup MsOS_Task
/// @param  s32QueueId      \b IN: Queue ID
/// @param  pu8Message      \b OUT: msg destination
/// @param  u32IntendedSize \b IN: intended msg size (byte unit) to receive:
/// @param  pu32ActualSize  \b OUT: actual msg size (byte unit) received
/// @param  u32WaitMs       \b IN: 0 ~ MDRV_MSOS_WAIT_FOREVER: suspend time (ms) if the queue is empty
/// @return TRUE : succeed
/// @return FALSE :  fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Recv(S32 s32QueueId, U8 *pu8Message, U32 u32IntendedSize, U32 *pu32ActualSize, U32 u32WaitMs);

//-------------------------------------------------------------------------------------------------
/// Receive a message from the specified queue
/// @ingroup MsOS_Task
/// @param  s32QueueId      \b IN: Queue ID
/// @param  pu8Message      \b OUT: msg destination
/// @param  u32IntendedSize \b IN: intended msg size (byte unit) to receive:
/// @param  pu32ActualSize  \b OUT: actual msg size (byte unit) received
/// @return TRUE : succeed
/// @return FALSE :  fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Peek(S32 s32QueueId, U8 *pu8Message, U32 u32IntendedSize, U32 *pu32ActualSize);
#endif
