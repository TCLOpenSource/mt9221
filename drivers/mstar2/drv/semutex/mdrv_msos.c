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
/// file    mdrv_msos.c
/// @brief  kernel mode msos api
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
/* From linux. */
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* From mstar. */
#include "mdrv_msos.h"
#include "mdrv_types.h"

//--------------------------------------------------------------------------------------------------
// [MSOS] Forward declaration
//--------------------------------------------------------------------------------------------------
#define MDRV_MSOS_ID_PREFIX             (0x76540000)
#define MDRV_MSOS_ID_PREFIX_MASK        (0xFFFF0000)
#define MDRV_MSOS_ID_MASK               (0x0000FFFF)

//-------------------------------------------------------------------------------------------------
// [MSOS] Data structure
//-------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// [MSOS] Local variable
//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [MSOS] Golbal variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [MSOS] Local function
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [MSOS] Golbal function
//-------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// [EVENT] Forward declaration
//--------------------------------------------------------------------------------------------------
#define MDRV_EVENT_WORK_QUEUE_NAME      "EventGroupWorkQueue"

#define MDRV_EVENT_LOCK(obj)            spin_lock(obj)
#define MDRV_EVENT_UNLOCK(obj)          spin_unlock(obj)

#define MDRV_EVENT_BUF_SIZE             (128)

#define MDRV_EVENT_CMP_FLAG(flag, bit)  ((flag) &   (bit))
#define MDRV_EVENT_SET_FLAG(flag, bit)  ((flag) |=  (bit))
#define MDRV_EVENT_CLR_FLAG(flag, bit)  ((flag) &=  (~(bit)))

//-------------------------------------------------------------------------------------------------
// [EVENT] Data structure
//-------------------------------------------------------------------------------------------------
typedef struct _WORK_DATA
{
    struct work_struct                  stWork;
    U32                                 u32GroupFlag;
} WORK_DATA;

typedef struct _MDRV_EVENT_INFO
{
    BOOL                                bUsed;
    U32                                 u32Wait;
    U32                                 u32Flag;
    spinlock_t                          stLock;
    wait_queue_head_t                   stWaitQueue;
    WORK_DATA                           stWorkData;
    U8                                  pu8Name[MDRV_EVENT_BUF_SIZE];
} MDRV_EVENT_INFO;

//--------------------------------------------------------------------------------------------------
// [EVENT] Local variable
//--------------------------------------------------------------------------------------------------
static DEFINE_SPINLOCK(_stMDrvEventLock);
static U32                              _u32MDrvEventMax = 0;
static struct workqueue_struct          *_pstWorkQueue = NULL;
static MDRV_EVENT_INFO                  *_pstMDrvEventInfo = NULL;

//-------------------------------------------------------------------------------------------------
// [EVENT] Golbal variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [EVENT] Local function
//-------------------------------------------------------------------------------------------------
static int irqContext(struct work_struct *work)
{
    WORK_DATA       *pstWorkData        = container_of(work, WORK_DATA, stWork);
    MDRV_EVENT_INFO *pstMDrvEventInfo   = container_of(pstWorkData, MDRV_EVENT_INFO, stWorkData);

    MDRV_EVENT_LOCK(&pstMDrvEventInfo->stLock);
    MDRV_EVENT_SET_FLAG(pstMDrvEventInfo->u32Flag, pstWorkData->u32GroupFlag);
    wake_up_all(&pstMDrvEventInfo->stWaitQueue);
    MDRV_EVENT_UNLOCK(&pstMDrvEventInfo->stLock);

    return 0;
}

//-------------------------------------------------------------------------------------------------
// [EVENT] Golbal function
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/// Initialize an event group
/// @param  u32GroupNum     \b IN: number of event group
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Init(U32 u32GroupNum)
{
	void *buf = NULL;
	struct workqueue_struct *wq = NULL;
	/* Check init. */
	MDRV_EVENT_LOCK(&_stMDrvEventLock);
	if ((_pstMDrvEventInfo != NULL) && (_u32MDrvEventMax != 0))	{
		MDRV_MSOS_DEBUG("Already inited.\n");
		MDRV_EVENT_UNLOCK(&_stMDrvEventLock);
		goto MDrv_Event_Init_End;
	} else {
		MDRV_EVENT_UNLOCK(&_stMDrvEventLock);
		buf = kzalloc(sizeof(MDRV_EVENT_INFO) * u32GroupNum, GFP_KERNEL);
		if (buf == NULL) {
			MDRV_MSOS_DEBUG("kzalloc() fail.\n");
			goto MDrv_Event_Init_End;
		}
	}
	wq = create_singlethread_workqueue(MDRV_EVENT_WORK_QUEUE_NAME);

	MDRV_EVENT_LOCK(&_stMDrvEventLock);
	if (_pstMDrvEventInfo == NULL) {
		_u32MDrvEventMax = u32GroupNum;
		_pstWorkQueue = wq;
		_pstMDrvEventInfo = buf;
		MDRV_EVENT_UNLOCK(&_stMDrvEventLock);
	} else {
		MDRV_EVENT_UNLOCK(&_stMDrvEventLock);
		kfree(buf);
		destroy_workqueue(wq);
	}
MDrv_Event_Init_End:
	return ((_pstMDrvEventInfo != NULL) && (_u32MDrvEventMax != 0) && (_pstWorkQueue != NULL));
}

//-------------------------------------------------------------------------------------------------
/// Create an event group
/// @param  pGroupName      \b IN: event group name
/// @return >=0 : assigned Event Id
/// @return <0 : fail
//-------------------------------------------------------------------------------------------------
S32 MDrv_Event_Create(char *pGroupName)
{
    U32 u32Id = 0;
    S32 s32FirstId = -1;

    /* Get id. */
    MDRV_EVENT_LOCK(&_stMDrvEventLock);
    for (u32Id = 0; u32Id < _u32MDrvEventMax; u32Id++)
    {
        if (_pstMDrvEventInfo[u32Id].bUsed == FALSE)
        {
            if (s32FirstId == -1)
            {
                _pstMDrvEventInfo[u32Id].bUsed = TRUE;
                s32FirstId = u32Id;
            }
        }
        else if (strncmp(_pstMDrvEventInfo[u32Id].pu8Name, pGroupName, MDRV_EVENT_BUF_SIZE) == 0)
        {
            if (s32FirstId >= 0)
            {
                _pstMDrvEventInfo[s32FirstId].bUsed = FALSE;
            }
            break;
        }
    }
    MDRV_EVENT_UNLOCK(&_stMDrvEventLock);

    /* Check id & get id. */
    if (u32Id >= _u32MDrvEventMax)
    {
        if (s32FirstId == -1)
        {
            MDRV_MSOS_DEBUG("Resource not enough: id(%d) >= resource(%d)\n",
                            u32Id, _u32MDrvEventMax);
            return -1;
        }
        else
        {
            u32Id = s32FirstId;
        }
    }

    /* Init data. */
    MDRV_EVENT_LOCK(&_stMDrvEventLock);
    _pstMDrvEventInfo[u32Id].u32Wait = 0;
    _pstMDrvEventInfo[u32Id].u32Flag = 0;
    spin_lock_init(&_pstMDrvEventInfo[u32Id].stLock);
    init_waitqueue_head(&_pstMDrvEventInfo[u32Id].stWaitQueue);
    INIT_WORK(&_pstMDrvEventInfo[u32Id].stWorkData.stWork, ((void *)((unsigned long)irqContext)));
    memset(_pstMDrvEventInfo[u32Id].pu8Name, '\0', MDRV_EVENT_BUF_SIZE);
    strncpy(_pstMDrvEventInfo[u32Id].pu8Name, pGroupName, MDRV_EVENT_BUF_SIZE - 1);
    MDRV_EVENT_UNLOCK(&_stMDrvEventLock);

    return (u32Id | MDRV_MSOS_ID_PREFIX);
}

//-------------------------------------------------------------------------------------------------
/// Delete the event group
/// @param  s32GroupId      \b IN: event group ID
/// @return TRUE : succeed
/// @return FALSE : fail, sb is waiting for the event flag
/// @note event group that are being waited on must not be deleted
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Delete(S32 s32GroupId)
{
    /* Check & Get id. */
    if (((s32GroupId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32GroupId & MDRV_MSOS_ID_MASK) >= _u32MDrvEventMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }
    else
    {
        s32GroupId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. */
    if (_pstMDrvEventInfo[s32GroupId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }

    /* Check waiting. */
    if (_pstMDrvEventInfo[s32GroupId].u32Wait > 0)
    {
        MDRV_MSOS_DEBUG("Event was still waiting.\n");
        return FALSE;
    }

    /* Destory data. */
    MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    _pstMDrvEventInfo[s32GroupId].u32Flag = 0;
    _pstMDrvEventInfo[s32GroupId].u32Wait = 0;
    init_waitqueue_head(&_pstMDrvEventInfo[s32GroupId].stWaitQueue);
    memset(_pstMDrvEventInfo[s32GroupId].pu8Name, '\0', MDRV_EVENT_BUF_SIZE);
    MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);

    MDRV_EVENT_LOCK(&_stMDrvEventLock);
    _pstMDrvEventInfo[s32GroupId].bUsed = FALSE;
    MDRV_EVENT_UNLOCK(&_stMDrvEventLock);

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Set the event flag in the specified event group
/// @param  s32GroupId      \b IN: event group ID
/// @param  u32GroupFlag    \b IN: event flag value
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Set(S32 s32GroupId, U32 u32GroupFlag)
{
    /* Check & Get id. */
    if (((s32GroupId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32GroupId & MDRV_MSOS_ID_MASK) >= _u32MDrvEventMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }
    else
    {
        s32GroupId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. */
    if (_pstMDrvEventInfo[s32GroupId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }

    /* workqueue. */
    if (in_interrupt())
    {
        /* Create new kernel thread to set event. */
        _pstMDrvEventInfo[s32GroupId].stWorkData.u32GroupFlag = u32GroupFlag;
        queue_work(_pstWorkQueue, &_pstMDrvEventInfo[s32GroupId].stWorkData.stWork);
    }
    else
    {
        MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
        MDRV_EVENT_SET_FLAG(_pstMDrvEventInfo[s32GroupId].u32Flag, u32GroupFlag);
        wake_up_all(&_pstMDrvEventInfo[s32GroupId].stWaitQueue);
        MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    }

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Clear the specified event flag in the specified event group
/// @param  s32GroupId      \b IN: event group ID
/// @param  u32GroupFlag    \b IN: event flag value
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Event_Clear(S32 s32GroupId, U32 u32GroupFlag)
{
    /* Check & Get id. */
    if (((s32GroupId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32GroupId & MDRV_MSOS_ID_MASK) >= _u32MDrvEventMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }
    else
    {
        s32GroupId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. */
    if (_pstMDrvEventInfo[s32GroupId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }

    MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    MDRV_EVENT_CLR_FLAG(_pstMDrvEventInfo[s32GroupId].u32Flag, u32GroupFlag);
    MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);

    return TRUE;
}

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
                     U32 u32WaitMs)
{
    BOOL bRet = FALSE;
    BOOL bAnd = FALSE;
    BOOL bClr = FALSE;

    /* Check & Get id. */
    if (((s32GroupId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32GroupId & MDRV_MSOS_ID_MASK) >= _u32MDrvEventMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }
    else
    {
        s32GroupId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. */
    if (_pstMDrvEventInfo[s32GroupId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }

    /* Check flag. */
    if (u32GroupFlag == 0)
    {
        MDRV_MSOS_DEBUG("Group flag is zero.\n");
        return FALSE;
    }

    /* Prepare. */
    *pu32CmpGroupFlag = 0;
    switch (eGroupMode)
    {
        case E_AND_CLEAR:
            bClr = TRUE;
        case E_AND:
            bAnd = TRUE;
            break;

        case E_OR_CLEAR:
            bClr = TRUE;
        case E_OR:
            bAnd = FALSE;
            break;

        default:
            MDRV_MSOS_DEBUG("Invalid MDRV_EVENT_MODE.\n");
            return FALSE;
    }

    /* Do wait flag. */
    MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    _pstMDrvEventInfo[s32GroupId].u32Wait++;

    /* Select wait time. */
    if (u32WaitMs == 0)
    {
        /* Nothing to do. */
    }
    else if (u32WaitMs == MDRV_MSOS_WAIT_FOREVER)
    {
        MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
        if (bAnd == TRUE)
        {
            wait_event(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                    ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) == u32GroupFlag));
        }
        else
        {
            wait_event(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                    ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) != 0));
        }
        MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    }
    else
    {
        MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
        if (bAnd == TRUE)
        {
            wait_event_timeout(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                                ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) == u32GroupFlag),
                                msecs_to_jiffies(u32WaitMs));
        }
        else
        {
            wait_event_timeout(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                                ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) != 0),
                                msecs_to_jiffies(u32WaitMs));
        }
        MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    }

    /* Check result. */
    *pu32CmpGroupFlag = MDRV_EVENT_CMP_FLAG(_pstMDrvEventInfo[s32GroupId].u32Flag, u32GroupFlag);
    bRet = (bAnd) ? (*pu32CmpGroupFlag == u32GroupFlag) : (*pu32CmpGroupFlag != 0);

    /* Clear mode. */
    if ((bClr == TRUE) && (bRet == TRUE))
    {
        MDRV_EVENT_CLR_FLAG(_pstMDrvEventInfo[s32GroupId].u32Flag, *pu32CmpGroupFlag);
    }

    /* Do wait flag. */
    _pstMDrvEventInfo[s32GroupId].u32Wait--;
    MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);

    return bRet;
}

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
                              U32 u32WaitMs)
{
    S32 s32Ret = FALSE;
    BOOL bAnd = FALSE;
    BOOL bClr = FALSE;
    BOOL bCond = FALSE;
    U64 u64Interval = 0;
    struct timeval stBegin, stEnd;

    /* Check & Get id. */
    if (((s32GroupId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32GroupId & MDRV_MSOS_ID_MASK) >= _u32MDrvEventMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }
    else
    {
        s32GroupId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. */
    if (_pstMDrvEventInfo[s32GroupId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32GroupId);
        return FALSE;
    }

    /* Check flag. */
    if (u32GroupFlag == 0)
    {
        MDRV_MSOS_DEBUG("Group flag is zero.\n");
        return FALSE;
    }

    /* Prepare. */
    *pu32CmpGroupFlag = 0;
    do_gettimeofday(&stBegin);
    switch (eGroupMode)
    {
        case E_AND_CLEAR:
            bClr = TRUE;
        case E_AND:
            bAnd = TRUE;
            break;

        case E_OR_CLEAR:
            bClr = TRUE;
        case E_OR:
            bAnd = FALSE;
            break;

        default:
            MDRV_MSOS_DEBUG("Invalid MDRV_EVENT_MODE.\n");
            return FALSE;
    }

    /* Do wait flag. */
    MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    _pstMDrvEventInfo[s32GroupId].u32Wait++;

    do
    {
        /* Select wait time. */
        if (u32WaitMs == 0)
        {
            break;
        }
        else if (u32WaitMs == MDRV_MSOS_WAIT_FOREVER)
        {
            MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
            if (bAnd == TRUE)
            {
                s32Ret = wait_event_interruptible(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                                                   ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) == u32GroupFlag));
            }
            else
            {
                s32Ret = wait_event_interruptible(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                                                   ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) != 0));
            }
            MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
        }
        else
        {
            MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
            if (bAnd == TRUE)
            {
                s32Ret = wait_event_interruptible_timeout(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                                                        ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) == u32GroupFlag),
                                                        msecs_to_jiffies(u32WaitMs - u64Interval));
            }
            else
            {
                s32Ret = wait_event_interruptible_timeout(_pstMDrvEventInfo[s32GroupId].stWaitQueue,
                                                        ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) != 0),
                                                        msecs_to_jiffies(u32WaitMs - u64Interval));
            }

            /* Update time. */
            do_gettimeofday(&stEnd);
            u64Interval = ((U64)(stEnd.tv_sec - stBegin.tv_sec) * 1000) + ((U64)(stEnd.tv_usec - stBegin.tv_usec) / 1000);
            MDRV_EVENT_LOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
        }

        /* Update Condation. */
        if (bAnd == TRUE)
        {
            bCond = ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) == u32GroupFlag);
        }
        else
        {
            bCond = ((_pstMDrvEventInfo[s32GroupId].u32Flag & u32GroupFlag) != 0);
        }

        /* Check return value. */
        if (s32Ret == -ERESTARTSYS)
        {
            goto MDrv_Event_Wait_Group_Interrupt_End;
        }

    } while (bCond != TRUE);

    /* Check result. */
    *pu32CmpGroupFlag = MDRV_EVENT_CMP_FLAG(_pstMDrvEventInfo[s32GroupId].u32Flag, u32GroupFlag);
    s32Ret = (bAnd) ? (*pu32CmpGroupFlag == u32GroupFlag) : (*pu32CmpGroupFlag != 0);

    /* Clear mode. */
    if ((bClr == TRUE) && (s32Ret == TRUE))
    {
        MDRV_EVENT_CLR_FLAG(_pstMDrvEventInfo[s32GroupId].u32Flag, *pu32CmpGroupFlag);
    }

MDrv_Event_Wait_Group_Interrupt_End:
    /* Do wait flag. */
    _pstMDrvEventInfo[s32GroupId].u32Wait--;
    MDRV_EVENT_UNLOCK(&_pstMDrvEventInfo[s32GroupId].stLock);
    return s32Ret;
}

//-------------------------------------------------------------------------------------------------
// [EVENT] Export
//-------------------------------------------------------------------------------------------------
EXPORT_SYMBOL(MDrv_Event_Init);
EXPORT_SYMBOL(MDrv_Event_Create);
EXPORT_SYMBOL(MDrv_Event_Delete);
EXPORT_SYMBOL(MDrv_Event_Set);
EXPORT_SYMBOL(MDrv_Event_Clear);
EXPORT_SYMBOL(MDrv_Event_Wait);
EXPORT_SYMBOL(MDrv_Event_Wait_Interrupt);

//--------------------------------------------------------------------------------------------------
// [QUEUE] Forward declaration
//--------------------------------------------------------------------------------------------------
#define MDRV_QUEUE_LOCK(obj)            spin_lock_irqsave(obj, _u32MDrvQueueLockFlag)
#define MDRV_QUEUE_UNLOCK(obj)          spin_unlock_irqrestore(obj, _u32MDrvQueueLockFlag)

#define MDRV_QUEUE_BUF_SIZE             (128)
#define MSOS_QUEUE_SIZE                 (10+1)
#define ALIGN_4(_x_)                    (((_x_) + 3) & ~3)

//-------------------------------------------------------------------------------------------------
// [QUEUE] Data structure
//-------------------------------------------------------------------------------------------------
typedef struct _MDRV_QUEUE_INFO
{
    BOOL                                bUsed;
    U8*                                 pu8Head;
    U8*                                 pu8Tail;
    U8*                                 pu8Read;
    U8*                                 pu8Write;
    U32                                 u32MsgAlign;
    wait_queue_head_t                   stSendSem;
    wait_queue_head_t                   stRecvSem;
    spinlock_t                          stSendMutex;
    spinlock_t                          stRecvMutex;
    U8                                  pu8Name[MDRV_QUEUE_BUF_SIZE];
} MDRV_QUEUE_INFO;

//--------------------------------------------------------------------------------------------------
// [QUEUE] Local variable
//--------------------------------------------------------------------------------------------------
static                                  DEFINE_SPINLOCK(_stMDrvQueueLock);
static unsigned long                    _u32MDrvQueueLockFlag = 0;
static MDRV_QUEUE_INFO *                _pstMDrvQueueInfo = NULL;
static U32                              _u32MDrvQueueMax = 0;

//-------------------------------------------------------------------------------------------------
// [QUEUE] Golbal variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// [QUEUE] Local function
//-------------------------------------------------------------------------------------------------
static BOOL _MDrv_EmptyQueue(MDRV_QUEUE_INFO *pstQueueInfo)
{
    return (pstQueueInfo->pu8Write == pstQueueInfo->pu8Read);
}

static BOOL _MDrv_FullQueue(MDRV_QUEUE_INFO *pstQueueInfo)
{
    U8 *pu8Ptr = NULL;

    pu8Ptr = pstQueueInfo->pu8Write + pstQueueInfo->u32MsgAlign;
    if (pu8Ptr == pstQueueInfo->pu8Tail)
    {
        pu8Ptr = pstQueueInfo->pu8Head;
    }

    return (pu8Ptr == pstQueueInfo->pu8Read);
}

//-------------------------------------------------------------------------------------------------
//  [QUEUE] Golbal function
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/// Initialize a queue
/// @param  u32QueueNum     \b IN: number of queue
/// @return TRUE : succeed
/// @return FALSE : fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Init(U32 u32QueueNum)
{
	void *buf = NULL;
	/* Check init. */
	MDRV_QUEUE_LOCK(&_stMDrvQueueLock);
	if ((_pstMDrvQueueInfo != NULL) && (_u32MDrvQueueMax != 0))	{
		MDRV_MSOS_DEBUG("Already inited.\n");
		MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);
		goto MDrv_Queue_Init_End;
	} else {
		MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);
		buf = kzalloc(sizeof(MDRV_QUEUE_INFO) * u32QueueNum, GFP_ATOMIC);
		if (buf == NULL) {
			MDRV_MSOS_DEBUG("kzalloc() fail.\n");
			goto MDrv_Queue_Init_End;
		}

		MDRV_QUEUE_LOCK(&_stMDrvQueueLock);
		if (_pstMDrvQueueInfo == NULL) {
			_pstMDrvQueueInfo = buf;
			_u32MDrvQueueMax = u32QueueNum;
			MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);
		} else {
			MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);
			kfree(buf);
		}
	}

MDrv_Queue_Init_End:
    return ((_pstMDrvQueueInfo != NULL) && (_u32MDrvQueueMax != 0));
}

//-------------------------------------------------------------------------------------------------
/// Create a Queue
/// @param  pStartAddr      \b IN: It is useless now, can pass NULL.
/// @param  u32QueueSize    \b IN: queue size (byte unit).
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
                      char            *pQueueName)
{
    U32 u32Id = 0;
    S32 s32FirstId = -1;
    U32 u32QueueNum = 0;

    /* Check mode. */
    if (eMessageType == E_QUEUE_MSG_VAR_SIZE)
    {
        MDRV_MSOS_DEBUG("Can't support E_MSG_VAR_SIZE.\n");
        return -1;
    }

    /* Get id. */
    MDRV_QUEUE_LOCK(&_stMDrvQueueLock);
    for (u32Id = 0; u32Id < _u32MDrvQueueMax; u32Id++)
    {
        if (_pstMDrvQueueInfo[u32Id].bUsed == FALSE)
        {
            if (s32FirstId == -1)
            {
                _pstMDrvQueueInfo[u32Id].bUsed = TRUE;
                s32FirstId = u32Id;
            }
        }
        else if (strncmp(_pstMDrvQueueInfo[u32Id].pu8Name, pQueueName, MDRV_QUEUE_BUF_SIZE) == 0)
        {
            if (s32FirstId >= 0)
            {
                _pstMDrvQueueInfo[s32FirstId].bUsed = FALSE;
            }
            break;
        }
    }
    MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);

    /* Check id & get id. */
    if (u32Id >= _u32MDrvQueueMax)
    {
        if (s32FirstId == -1)
        {
            MDRV_MSOS_DEBUG("Resource not enough: id(%d) >= resource(%d)\n",
                            u32Id, _u32MDrvQueueMax);
            return -1;
        }
        else
        {
            u32Id = s32FirstId;
        }
    }

    /* Adjust total of queue size. */
    u32QueueNum = (u32QueueSize / u32MessageSize);

    /* Fix SOPHIA-3818. */
    if (u32QueueNum <= 0)
       u32QueueNum = MSOS_QUEUE_SIZE;
    u32QueueSize = (ALIGN_4(u32MessageSize) * u32QueueNum);
    if ((u32QueueSize / (ALIGN_4(u32MessageSize))) != u32QueueNum)
    {
        MDRV_MSOS_DEBUG("Overflow: u32QueueSize=0x%tX, ALIGN_4(u32MessageSize)=0x%tX, u32QueueNum=0x%tX\n",
                        (size_t)u32QueueSize, (size_t)ALIGN_4(u32MessageSize), (size_t)u32QueueNum);
        return -1;
    }

    /* Check queue init. */
    _pstMDrvQueueInfo[u32Id].pu8Head = kzalloc(u32QueueSize, GFP_KERNEL);
    if (_pstMDrvQueueInfo[u32Id].pu8Head == NULL)
    {
        MDRV_QUEUE_LOCK(&_stMDrvQueueLock);
        _pstMDrvQueueInfo[u32Id].bUsed = FALSE;
        MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);
        MDRV_MSOS_DEBUG("kzalloc() fail : size=%td.\n", (size_t)u32QueueSize);
        return -1;
    }

    /* Init data. */
    _pstMDrvQueueInfo[u32Id].pu8Tail     = _pstMDrvQueueInfo[u32Id].pu8Head + u32QueueSize;
    _pstMDrvQueueInfo[u32Id].pu8Read     = _pstMDrvQueueInfo[u32Id].pu8Head;
    _pstMDrvQueueInfo[u32Id].pu8Write    = _pstMDrvQueueInfo[u32Id].pu8Head;
    _pstMDrvQueueInfo[u32Id].u32MsgAlign = ALIGN_4(u32MessageSize);
    init_waitqueue_head(&_pstMDrvQueueInfo[u32Id].stSendSem);
    init_waitqueue_head(&_pstMDrvQueueInfo[u32Id].stRecvSem);
    spin_lock_init(&_pstMDrvQueueInfo[u32Id].stSendMutex);
    spin_lock_init(&_pstMDrvQueueInfo[u32Id].stRecvMutex);
    memset(_pstMDrvQueueInfo[u32Id].pu8Name, '\0', MDRV_QUEUE_BUF_SIZE);
    strncpy(_pstMDrvQueueInfo[u32Id].pu8Name, pQueueName, MDRV_QUEUE_BUF_SIZE - 1);

    return (u32Id | MDRV_MSOS_ID_PREFIX);
}

//-------------------------------------------------------------------------------------------------
/// Delete the Queue
/// @param  s32QueueId  \b IN: Queue ID
/// @return TRUE : succeed
/// @return FALSE :  fail
/// @note   It is important that there are not any threads blocked on the queue
///             when this function is called or the behavior is undefined.
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Delete(S32 s32QueueId)
{
    /* Check & Get id. */
    if (((s32QueueId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32QueueId & MDRV_MSOS_ID_MASK) >= _u32MDrvQueueMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }
    else
    {
        s32QueueId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. ignore coverity */
    if (_pstMDrvQueueInfo[s32QueueId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }

    MDRV_QUEUE_LOCK(&_stMDrvQueueLock);
    /* Clear data. */
    _pstMDrvQueueInfo[s32QueueId].bUsed = FALSE;
    if (_pstMDrvQueueInfo[s32QueueId].pu8Head != NULL)
    {
        kfree(_pstMDrvQueueInfo[s32QueueId].pu8Head);
        _pstMDrvQueueInfo[s32QueueId].pu8Head= NULL;
    }
    _pstMDrvQueueInfo[s32QueueId].pu8Tail       = NULL;
    _pstMDrvQueueInfo[s32QueueId].pu8Read       = NULL;
    _pstMDrvQueueInfo[s32QueueId].pu8Write      = NULL;
    _pstMDrvQueueInfo[s32QueueId].u32MsgAlign   = 0;
    init_waitqueue_head(&_pstMDrvQueueInfo[s32QueueId].stSendSem);
    init_waitqueue_head(&_pstMDrvQueueInfo[s32QueueId].stRecvSem);
    spin_lock_init(&_pstMDrvQueueInfo[s32QueueId].stSendMutex);
    spin_lock_init(&_pstMDrvQueueInfo[s32QueueId].stRecvMutex);
    memset(_pstMDrvQueueInfo[s32QueueId].pu8Name, '\0', MDRV_QUEUE_BUF_SIZE);
    MDRV_QUEUE_UNLOCK(&_stMDrvQueueLock);

    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Send a message to the end of the specified queue
/// @param  s32QueueId  \b IN: Queue ID
/// @param  pu8Message  \b IN: ptr to msg to send. NULL ptr is not allowed
/// @param  u32Size     \b IN: msg size (byte)
/// @param  u32WaitMs   \b IN: 0 ~ MDRV_MSOS_WAIT_FOREVER: suspend time (ms) if the queue is full
/// @return TRUE : succeed
/// @return FALSE :  fail
//-------------------------------------------------------------------------------------------------
BOOL MDrv_Queue_Send(S32 s32QueueId, U8 *pu8Message, U32 u32Size, U32 u32WaitMs)
{
    BOOL bRet = TRUE;

    /* Check message. */
    if (pu8Message == NULL)
    {
        MDRV_MSOS_DEBUG("pu8Message = NULL.\n");
        return FALSE;
    }

    /* Check & Get id. */
    if (((s32QueueId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32QueueId & MDRV_MSOS_ID_MASK) >= _u32MDrvQueueMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }
    else
    {
        s32QueueId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. ignore coverity */
    if (_pstMDrvQueueInfo[s32QueueId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }

    /* Check size. */
    if (u32Size > _pstMDrvQueueInfo[s32QueueId].u32MsgAlign)
    {
        MDRV_MSOS_DEBUG("Over message size : u32Size(%td) > u32MsgAlign(%td).\n",
                    (size_t)u32Size, (size_t)_pstMDrvQueueInfo[s32QueueId].u32MsgAlign);
        return FALSE;
    }

    MDRV_QUEUE_LOCK(&_pstMDrvQueueInfo[s32QueueId].stSendMutex);

    /* Wait space. */
    if (_MDrv_FullQueue(&_pstMDrvQueueInfo[s32QueueId]) == TRUE)
    {
        MDRV_QUEUE_UNLOCK(&_pstMDrvQueueInfo[s32QueueId].stSendMutex);
        if (u32WaitMs == MDRV_MSOS_WAIT_FOREVER)
        {
            if (wait_event_interruptible(_pstMDrvQueueInfo[s32QueueId].stRecvSem,
                                        (_MDrv_FullQueue(&_pstMDrvQueueInfo[s32QueueId]) == FALSE)) != 0)
                bRet = FALSE;
        }
        else
        {
            if (wait_event_interruptible_timeout(_pstMDrvQueueInfo[s32QueueId].stRecvSem,
                                                (_MDrv_FullQueue(&_pstMDrvQueueInfo[s32QueueId]) == FALSE),
                                                msecs_to_jiffies(u32WaitMs)) <= 0)
                bRet = FALSE;
        }
        MDRV_QUEUE_LOCK(&_pstMDrvQueueInfo[s32QueueId].stSendMutex);

        if (bRet == FALSE)
            goto MDrv_Queue_Send_End;
    }

    /* Send message. */
    _pstMDrvQueueInfo[s32QueueId].pu8Write += _pstMDrvQueueInfo[s32QueueId].u32MsgAlign;
    if (_pstMDrvQueueInfo[s32QueueId].pu8Write == _pstMDrvQueueInfo[s32QueueId].pu8Tail)
    {
        _pstMDrvQueueInfo[s32QueueId].pu8Write = _pstMDrvQueueInfo[s32QueueId].pu8Head;
    }
    memcpy(_pstMDrvQueueInfo[s32QueueId].pu8Write, pu8Message, u32Size);
    wake_up_all(&_pstMDrvQueueInfo[s32QueueId].stSendSem);

MDrv_Queue_Send_End:
    MDRV_QUEUE_UNLOCK(&_pstMDrvQueueInfo[s32QueueId].stSendMutex);
    return bRet;
}

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
BOOL MDrv_Queue_Recv(S32 s32QueueId, U8 *pu8Message, U32 u32IntendedSize, U32 *pu32ActualSize, U32 u32WaitMs)
{
    BOOL bRet = TRUE;

    /* Check & Get id. */
    if (((s32QueueId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32QueueId & MDRV_MSOS_ID_MASK) >= _u32MDrvQueueMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }
    else
    {
        s32QueueId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. ignore coverity */
    if (_pstMDrvQueueInfo[s32QueueId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }

    /* Check size. */
    if (_pstMDrvQueueInfo[s32QueueId].u32MsgAlign != ALIGN_4(u32IntendedSize))
    {
        MDRV_MSOS_DEBUG("Incorrect size\n");
        return FALSE;
    }

    MDRV_QUEUE_LOCK(&_pstMDrvQueueInfo[s32QueueId].stRecvMutex);

    /* Wait data. */
    if (_MDrv_EmptyQueue(&_pstMDrvQueueInfo[s32QueueId]) == TRUE)
    {
        MDRV_QUEUE_UNLOCK(&_pstMDrvQueueInfo[s32QueueId].stRecvMutex);
        if (u32WaitMs == MDRV_MSOS_WAIT_FOREVER)
        {
            if (wait_event_interruptible(_pstMDrvQueueInfo[s32QueueId].stSendSem,
                                        (_MDrv_EmptyQueue(&_pstMDrvQueueInfo[s32QueueId]) == FALSE)) != 0)
                bRet = FALSE;
        }
        else
        {
            if (wait_event_interruptible_timeout(_pstMDrvQueueInfo[s32QueueId].stSendSem,
                                                (_MDrv_EmptyQueue(&_pstMDrvQueueInfo[s32QueueId]) == FALSE),
                                                msecs_to_jiffies(u32WaitMs)) <= 0)
                bRet = FALSE;
        }
        MDRV_QUEUE_LOCK(&_pstMDrvQueueInfo[s32QueueId].stRecvMutex);

        if (bRet == FALSE)
            goto MDrv_Queue_Recv_End;
    }

    /* Recv message. */
    _pstMDrvQueueInfo[s32QueueId].pu8Read += _pstMDrvQueueInfo[s32QueueId].u32MsgAlign;
    if (_pstMDrvQueueInfo[s32QueueId].pu8Read == _pstMDrvQueueInfo[s32QueueId].pu8Tail)
    {
        _pstMDrvQueueInfo[s32QueueId].pu8Read = _pstMDrvQueueInfo[s32QueueId].pu8Head;
    }
    memcpy(pu8Message, _pstMDrvQueueInfo[s32QueueId].pu8Read, u32IntendedSize);
    *pu32ActualSize = u32IntendedSize;
    wake_up_all(&_pstMDrvQueueInfo[s32QueueId].stRecvSem);

MDrv_Queue_Recv_End:
    MDRV_QUEUE_UNLOCK(&_pstMDrvQueueInfo[s32QueueId].stRecvMutex);
    return bRet;
}

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
BOOL MDrv_Queue_Peek(S32 s32QueueId, U8 *pu8Message, U32 u32IntendedSize, U32 *pu32ActualSize)
{
    U8 *pu8Ptr = NULL;

    /* Check & Get id. */
    if (((s32QueueId & MDRV_MSOS_ID_PREFIX_MASK) != MDRV_MSOS_ID_PREFIX) ||
        ((s32QueueId & MDRV_MSOS_ID_MASK) >= _u32MDrvQueueMax))
    {
        MDRV_MSOS_DEBUG("Invalid ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }
    else
    {
        s32QueueId &= MDRV_MSOS_ID_MASK;
    }

    /* Check used. ignore coverity */
    if (_pstMDrvQueueInfo[s32QueueId].bUsed != TRUE)
    {
        MDRV_MSOS_DEBUG("Un-used ID = 0x%tX.\n", (size_t)s32QueueId);
        return FALSE;
    }

    /* Check size. */
    if (_pstMDrvQueueInfo[s32QueueId].u32MsgAlign != ALIGN_4(u32IntendedSize))
    {
        MDRV_MSOS_DEBUG("Incorrect size\n");
        return FALSE;
    }

    /* Queue empty. */
    if (_MDrv_EmptyQueue(&_pstMDrvQueueInfo[s32QueueId]) == TRUE)
    {
        return FALSE;
    }

    /* Reek message. */
    pu8Ptr = (_pstMDrvQueueInfo[s32QueueId].pu8Read + _pstMDrvQueueInfo[s32QueueId].u32MsgAlign);
    if (pu8Ptr == _pstMDrvQueueInfo[s32QueueId].pu8Tail)
    {
        pu8Ptr = _pstMDrvQueueInfo[s32QueueId].pu8Head;
    }
    memcpy(pu8Message, pu8Ptr, u32IntendedSize);
    *pu32ActualSize = u32IntendedSize;
    return TRUE;
}

//-------------------------------------------------------------------------------------------------
// [QUEUE] Export
//-------------------------------------------------------------------------------------------------
EXPORT_SYMBOL(MDrv_Queue_Init);
EXPORT_SYMBOL(MDrv_Queue_Create);
EXPORT_SYMBOL(MDrv_Queue_Delete);
EXPORT_SYMBOL(MDrv_Queue_Send);
EXPORT_SYMBOL(MDrv_Queue_Recv);
EXPORT_SYMBOL(MDrv_Queue_Peek);
