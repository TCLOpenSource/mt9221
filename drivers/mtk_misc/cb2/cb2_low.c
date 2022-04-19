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
 *
 * $Author: brianpc.huang $
 * $Date: 2015/05/29 $
 * $RCSfile: cb2_low.c,v $
 * $Revision: #1 $
 *
 *---------------------------------------------------------------------------*/

/** @file cb2_low.c
 *  This file implements low-layer support of callback interface.
 */


//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------

#include "cb2_low.h"
//#include "x_linux.h"
#include <linux/slab.h>
#include "x_typedef.h"
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>

//-----------------------------------------------------------------------------
// Static variables
//-----------------------------------------------------------------------------
DEFINE_SPINLOCK(mt53xx_cb_lock);

DECLARE_WAIT_QUEUE_HEAD(_rWq);
static DLIST_T(_CB_CALLBACK_EVENT_T) _qCbEvent;

/* Event type to callback thread ownership array */
static pid_t _arCbId[CB_FCT_NUM];
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
static pid_t _arMultiCbId[CB_FCT_NUM][3];
#endif
static BOOL fgInit = FALSE;
static int iStopEventCount = 0;
DECLARE_WAIT_QUEUE_HEAD(qCbStopWaitQueue);
static int iCbCount = 0;

//-----------------------------------------------------------------------------
// Static functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/** Add callback event into event queue.
 *  @param  prCbEv  Pointer to callback event.
 *  @retval CBR_OK          Success.
 *  @retval CBR_ERR_SYS     Internal system error.
 */
//-----------------------------------------------------------------------------
static INT32 _CbPutEvent(CB_CALLBACK_EVENT_T * prCbEv)
{
	unsigned long rState;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	DLIST_INSERT_TAIL(prCbEv, &_qCbEvent, rLink);
	iCbCount++;
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	wake_up_interruptible_all(&_rWq);

	return CBR_OK;
}


//-----------------------------------------------------------------------------
// Inter-file functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/** Initialize callback support.
 */
//-----------------------------------------------------------------------------
void _CB_Init(void)
{
	unsigned long rState;

	if (fgInit) {
		return;
	}

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	fgInit = TRUE;
	DLIST_INIT(&_qCbEvent);
	iCbCount = 0;
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
}

//-----------------------------------------------------------------------------
/** To enable/register a callback function with the related ID.
 *  @param  eFctId  Callback function ID to enable.
 *  @param  i4SubId Callback function Sub ID.
 *  @return The return value of related callback registration function.
 */
//-----------------------------------------------------------------------------

unsigned int isPowerStateRegistered = 0;
unsigned int is2ndPowerStateRegistered = 0;

INT32 _CB_RegCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid)
{
	BOOL fgWakeUp = FALSE;
	unsigned long rState;
#if 0
	if ((_arMultiCbId[eFctId][0] != 0) ||
	    (_arMultiCbId[eFctId][1] != 0) ||
	    (_arMultiCbId[eFctId][2] != 0)) {
		return CBR_ERR_REG;
	}
#endif
	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	if (eFctId < CB_FCT_NUM) {
		if ((_arCbId[eFctId] != (pid_t) 0) &&
		    (_arCbId[eFctId] != rCbPid)) {
			fgWakeUp = TRUE;
		}

		_arCbId[eFctId] = rCbPid;
	}
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
#if 0
#ifndef CONFIG_TV_DRV_VFY
	if (apfnCbRegIf[eFctId] != NULL) {
		return apfnCbRegIf[eFctId] (i4SubId);
	}
#endif
#endif
	if (fgWakeUp) {
		wake_up_interruptible_all(&_rWq);
	}

	/*
	   printk("*** Thread 0x%x registers callback (0x%x, 0x%x)\n",
	   (INT32)current, (INT32)eFctId, (INT32)i4SubId);
	 */
	if(eFctId == CB2_DRV_PDWNC_POWER_STATE)
	{
		isPowerStateRegistered = 1;
	}
	if(eFctId == CB2_DRV_PDWNC_POWER_STATE2)
	{
	  is2ndPowerStateRegistered = 1;
	}
	return CBR_OK;
}

#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
//-----------------------------------------------------------------------------
/** To enable/register a multiple callback function with the related ID.
 *  @param  eFctId  Callback function ID to enable.
 *  @param  i4SubId Callback function Sub ID.
 *  @return The return value of related multiple callback registration function.
 */
//-----------------------------------------------------------------------------
INT32 _CB_RegMultiCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid)
{
	BOOL fgWakeUp = FALSE;
	unsigned long rState;
	INT32 i4Ret = 0;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	if (eFctId < CB_FCT_NUM) {
		if ((_arCbId[eFctId] != (pid_t) 0) &&
		    (_arCbId[eFctId] != rCbPid)) {
			fgWakeUp = TRUE;	// wakeup to leave.
		}

		_arCbId[eFctId] = 0;	// Remove single callback.
		if (_arMultiCbId[eFctId][0] == 0) {
			_arMultiCbId[eFctId][0] = rCbPid;
		} else if (_arMultiCbId[eFctId][1] == 0) {
				_arMultiCbId[eFctId][1] = rCbPid;
		} else if (_arMultiCbId[eFctId][2] == 0) {
			_arMultiCbId[eFctId][2] = rCbPid;
		} else {
#if CB_MULTIPROCESS_LAST_REPLACE
			if ((rCbPid != _arMultiCbId[eFctId][0])
			    && (rCbPid != _arMultiCbId[eFctId][1])
			    && (rCbPid != _arMultiCbId[eFctId][2])) {
				i4Ret = CBR_ERR_REG;
			} else {
				printk
				    ("_CB_RegMultiCbFct multi-fctid used in process(%d)\n",
				     rCbPid);
			}
#else
			i4Ret = CBR_ERR_REG;
#endif
		}
	}
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	if (i4Ret)
		return i4Ret;

#if 0
#ifndef CONFIG_TV_DRV_VFY
	if (apfnCbRegIf[eFctId] != NULL) {
		return apfnCbRegIf[eFctId] (i4SubId);
	}
#endif
#endif
	if (fgWakeUp) {
		wake_up_interruptible_all(&_rWq);
	}

	/*
	   printk("*** Thread 0x%x registers multiple callback (0x%x, 0x%x)\n",
	   (INT32)current, (INT32)eFctId, (INT32)i4SubId);
	 */

	return CBR_OK;
}
#endif

pid_t _CB_GetPid(CB_FCT_ID_T eFctId)
{
	if (eFctId < CB_FCT_NUM) {
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
		if ((current->tgid == _arCbId[eFctId]) ||
		    (current->tgid == _arMultiCbId[eFctId][0]) ||
		    (current->tgid == _arMultiCbId[eFctId][1]) ||
		    (current->tgid == _arMultiCbId[eFctId][2])) {
			return current->tgid;
		}

		if (_arCbId[eFctId])
			return _arCbId[eFctId];
		if (_arMultiCbId[eFctId][0])
			return _arMultiCbId[eFctId][0];
		if (_arMultiCbId[eFctId][1])
			return _arMultiCbId[eFctId][1];
		if (_arMultiCbId[eFctId][2])
			return _arMultiCbId[eFctId][2];
#endif
		return _arCbId[eFctId];
	} else {
		return 0;
	}
}

BOOL _CB_ParsingEventQueue(CB_EVENT_COUNT_T *prCbIdleFctIdInfo)
{
    CB_CALLBACK_EVENT_T *prLast = NULL;
    CB_FCT_ID_T eType;
    unsigned long rState;

    if (prCbIdleFctIdInfo == NULL)
    {
        return FALSE;
    }
    
    for(eType=CB_TIM_TRIGGER;eType<CB_FCT_NUM;eType++)
    {
        prCbIdleFctIdInfo->puCB00IdleFctPid[eType]=0;
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
        prCbIdleFctIdInfo->puCB01IdleFctPid[eType]=0;
        prCbIdleFctIdInfo->puCB02IdleFctPid[eType]=0;	  
#endif
    }
    
    spin_lock_irqsave(&mt53xx_cb_lock, rState);
    DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) 
    {
        if (!prLast)
        {
            break;
        }
        
        if (prLast->rGetCb.eFctId >= CB_FCT_NUM)
        {
            break;
        }
        
        if ((eType>=CB_TIM_TRIGGER) && (eType<CB_FCT_NUM))
        {
            if (eType == prLast->rGetCb.eFctId)
            {
                prCbIdleFctIdInfo->puCB00IdleFctPid[eType]++;
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
                prCbIdleFctIdInfo->puCB01IdleFctPid[eType]++;
                prCbIdleFctIdInfo->puCB02IdleFctPid[eType]++;	  
#endif    
            }
        }
    }
    spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

    return TRUE;
}

int _CB_GetCbCount(void)
{
    unsigned long rState;
    int i4Count;
    //printk("_CB_GetCbCount = %d!\n",iCbCount);
    spin_lock_irqsave(&mt53xx_cb_lock, rState);
    i4Count = iCbCount;
    spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
    return i4Count;
}

#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
//-----------------------------------------------------------------------------
/** To unregister a callback function with the related ID.
 *  @param  eFctId  Callback function ID to enable.
 *  @param  i4SubId Callback function Sub ID.
 *  @return The return value of related callback unregistration function.
 */
//-----------------------------------------------------------------------------
INT32 _CB_UnRegCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	unsigned long rState;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	if ((eFctId < CB_FCT_NUM) && (rCbPid == _arCbId[eFctId])) {
		_arCbId[eFctId] = 0;
	}
	DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
		if (rCbPid == _arCbId[prLast->rGetCb.eFctId]) {
			prLast->u4BitMask &= ~(BITMASK_SINGLE);
			if (prLast->u4BitMask == 0) {
				DLIST_REMOVE(prLast, &_qCbEvent, rLink);
				kfree(prLast);
			}
		}
	}
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	return CBR_OK;
}

//-----------------------------------------------------------------------------
/** To unregister a multiple callback function with the related ID.
 *  @param  eFctId  Callback function ID to enable.
 *  @param  i4SubId Callback function Sub ID.
 *  @return The return value of related callback unregistration function.
 */
//-----------------------------------------------------------------------------
INT32 _CB_UnRegMultiCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	unsigned long rState;
	UINT32 u4Multi = 0;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	if (eFctId < CB_FCT_NUM) {
		if (rCbPid == _arMultiCbId[eFctId][0]) {
			_arMultiCbId[eFctId][0] = 0;
			u4Multi = 1;
		}
		if (rCbPid == _arMultiCbId[eFctId][1]) {
			_arMultiCbId[eFctId][1] = 0;
			u4Multi = 2;
		}
		if (rCbPid == _arMultiCbId[eFctId][2]) {
			_arMultiCbId[eFctId][2] = 0;
			u4Multi = 3;
		}

		DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
			if ((u4Multi != 0) &&
			    (rCbPid ==
			     _arMultiCbId[prLast->rGetCb.eFctId][u4Multi -
								 1])) {
				prLast->u4BitMask &=
				    ~(BITMASK_MULTI(u4Multi));
				if (prLast->u4BitMask == 0) {
					DLIST_REMOVE(prLast, &_qCbEvent,
						     rLink);
					kfree(prLast);
				}
			}
		}
	}
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	return CBR_OK;
}
#else
//-----------------------------------------------------------------------------
/** To unregister a callback function with the related ID.
 *  @param  eFctId  Callback function ID to enable.
 *  @param  i4SubId Callback function Sub ID.
 *  @return The return value of related callback unregistration function.
 */
//-----------------------------------------------------------------------------
INT32 _CB_UnRegCbFct(CB_FCT_ID_T eFctId, INT32 i4SubId, pid_t rCbPid)
{
	unsigned long rState;
	CB_CALLBACK_EVENT_T *prLast = NULL;
	BOOL fgFound;

	if (eFctId >= CB_FCT_NUM)
		return CBR_ERR_REG;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	if (rCbPid != _arCbId[eFctId] || !_arCbId[eFctId])
		goto done;

	// Stop incoming events.
	iStopEventCount++;

	while (1) {
		// Check if this is on event queue.
		fgFound = FALSE;
		DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
			if (!prLast) {
				break;
			}

			if (prLast->rGetCb.eFctId >= CB_FCT_NUM) {
				break;
			}

			if (prLast->rGetCb.eFctId == eFctId) {
				fgFound = TRUE;
				break;
			}

			if (prLast->rLink.pt_next == NULL) {
				break;
			}
		}

		// No more events for eFctId, we are done.
		if (!fgFound)
			break;

		// Wait for events to be consumed. 
		// Must sleep so other thread can working on the events.
		spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
		msleep(10);
		spin_lock_irqsave(&mt53xx_cb_lock, rState);
	}

	// Allow more events to come.
	iStopEventCount--;
	wake_up_all(&qCbStopWaitQueue);

      done:
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	return CBR_OK;
}
#endif

//-----------------------------------------------------------------------------
/** Add callback event into event queue.
 *  @param  eFctId      Callback function ID to notify.
 *  @param  i4TagSize   Data size to callback.
 *  @param  pvTag       Data to callback.
 *  @retval CBR_OK          Success.
 *  @retval CBR_NO_MEM      Out of memory.
 */
//-----------------------------------------------------------------------------
INT32 _CB_PutEvent(CB_FCT_ID_T eFctId, INT32 i4TagSize, void *pvTag)
{
	CB_CALLBACK_EVENT_T *prCbEv;
	UINT32 flags;
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
	UINT32 u4BitMask = 0;
#endif
	if (!fgInit) {
		printk("Error! CB not yet initialized!\n");
		return CBR_ERR_SYS;
	}

	if (in_interrupt()) {
		flags = GFP_ATOMIC;
	} else {
		flags = GFP_KERNEL;

		if (!in_atomic()) {
			wait_event(qCbStopWaitQueue, iStopEventCount == 0);
		}
	}

	prCbEv = (CB_CALLBACK_EVENT_T *)
	    kmalloc(sizeof(CB_CALLBACK_EVENT_T) + i4TagSize, flags);

	if (prCbEv == NULL) {
		return CBR_NO_MEM;
	}
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
	if (_arCbId[eFctId]) {
		u4BitMask = BITMASK_SINGLE;
	} else {
		u4BitMask =
		    (_arMultiCbId[eFctId][0] ? (BITMASK_MULTI1) : 0) |
		    (_arMultiCbId[eFctId][1] ? (BITMASK_MULTI2) : 0) |
		    (_arMultiCbId[eFctId][2] ? (BITMASK_MULTI3) : 0);
	}
	if (!u4BitMask) {
		//No one register callback , skip put event
		printk("No one register callback , skip put event!\n");
		return CBR_OK;
	}

	prCbEv->u4BitMask = u4BitMask;
#endif
	prCbEv->rGetCb.eFctId = eFctId;
	prCbEv->rGetCb.i4TagSize = i4TagSize;
	memcpy((void *) ((UPTR) prCbEv + sizeof(CB_CALLBACK_EVENT_T)),
	       pvTag, i4TagSize);

	_CbPutEvent(prCbEv);
	return CBR_OK;
}

#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
//-----------------------------------------------------------------------------
/** Get the first event from event queue.
 *  @param  pu4DoneMask output parameter to transfer the Done Mask bit.
 *  @return If success and has event in queue, pointer of the callback event.\n
 *          Otherwise, NULL.
 */
//-----------------------------------------------------------------------------
CB_CALLBACK_EVENT_T *_CB_GetEventNoLock(UINT32 * pu4DoneMask)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	BOOL fgFound = FALSE;
	UINT32 u4DoneMask = 0;

	DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
		if (!prLast) {
			break;
		}

		if (prLast->rGetCb.eFctId >= CB_FCT_NUM) {
			break;
		}

		if (current->tgid == _arCbId[prLast->rGetCb.eFctId]) {
			fgFound = TRUE;
			u4DoneMask = BITMASK_SINGLE;
			break;
		} else
		    if ((current->tgid ==
			 _arMultiCbId[prLast->rGetCb.eFctId][0])
			&& (prLast->u4BitMask & BITMASK_MULTI1)) {
			fgFound = TRUE;
			u4DoneMask = BITMASK_MULTI1;
			break;
		} else
		    if ((current->tgid ==
			 _arMultiCbId[prLast->rGetCb.eFctId][1])
			&& (prLast->u4BitMask & BITMASK_MULTI2)) {
			fgFound = TRUE;
			u4DoneMask = BITMASK_MULTI2;
			break;
		} else
		    if ((current->tgid ==
			 _arMultiCbId[prLast->rGetCb.eFctId][2])
			&& (prLast->u4BitMask & BITMASK_MULTI3)) {
			fgFound = TRUE;
			u4DoneMask = BITMASK_MULTI3;
			break;
		}
	}

	if (pu4DoneMask != NULL) {
		*pu4DoneMask = u4DoneMask;
	}

	if (fgFound) {
		return prLast;
	} else {
		return NULL;
	}
}

CB_CALLBACK_EVENT_T *_CB_GetEvent(UINT32 *pu4DoneMask)
{
	unsigned long rState;
	CB_CALLBACK_EVENT_T *prLast = NULL;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	prLast = _CB_GetEventNoLock(pu4DoneMask);
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	return prLast;
}

//-----------------------------------------------------------------------------
/** Remove the event from event queue and free the element.
 *  @param  prCbEv      event element pointer to be removed and free.
 *  @param  u4DoneMask  input parameter to identify the Done Mask bit.
 */
//-----------------------------------------------------------------------------
void _CB_FreeEventNoLock(CB_CALLBACK_EVENT_T * prCbEv, UINT32 u4DoneMask)
{
	prCbEv->u4BitMask &= ~(u4DoneMask);
	if (prCbEv->u4BitMask == 0) {
		//if(prCbEv->rGetCb.eFctId == CB_MTAT_CEC_5V_STATUS_NFY)
		//{
		if (iCbCount > 0) {
			iCbCount--;
		}
		//}
		DLIST_REMOVE(prCbEv, &_qCbEvent, rLink);
		kfree(prCbEv);
	}
}

void _CB_FreeEvent(CB_CALLBACK_EVENT_T *prCbEv, UINT32 u4DoneMask)
{
	unsigned long rState;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	_CB_FreeEventNoLock(prCbEv, u4DoneMask)
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
}
#else
//-----------------------------------------------------------------------------
/** Get the first event from event queue.
 *  @return If success and has event in queue, pointer of the callback event.\n
 *          Otherwise, NULL.
 */
//-----------------------------------------------------------------------------
CB_CALLBACK_EVENT_T *_CB_GetEventNoLock(void)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	BOOL fgFound = FALSE;

	DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
		if (!prLast) {
			break;
		}

		if (prLast->rGetCb.eFctId >= CB_FCT_NUM) {
			break;
		}

		if (current->tgid == _arCbId[prLast->rGetCb.eFctId]) {
			fgFound = TRUE;
			break;
		}

		if (prLast->rLink.pt_next == NULL) {
			break;
		}
	}
	if (fgFound && (prLast != NULL)) {
		DLIST_REMOVE(prLast, &_qCbEvent, rLink);
	}
	if (fgFound) {
		return prLast;
	} else {
		return NULL;
	}
}

CB_CALLBACK_EVENT_T *_CB_GetEvent(void)
{
	unsigned long rState;
	CB_CALLBACK_EVENT_T *prLast = NULL;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	prLast = _CB_GetEventNoLock();
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);

	return prLast;
}
#endif
//-----------------------------------------------------------------------------
/** Put the current thread into wait queue.
 */
//-----------------------------------------------------------------------------
#if 0
void _CB_PutThdIntoWQ(void)
{
	DECLARE_WAITQUEUE(wait, current);

	add_wait_queue(&_rWq, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	schedule();
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&_rWq, &wait);
}
#endif

#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
//-----------------------------------------------------------------------------
/** Clear specified PID entries in event list
 */
//-----------------------------------------------------------------------------
void _CB_ClearPendingEventEntries(void)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	unsigned long rState;
	BOOL fgFound;
	UINT32 u4DoneMask;

	do {
		fgFound = FALSE;

		spin_lock_irqsave(&mt53xx_cb_lock, rState);
		if ((prLast = _CB_GetEventNoLock(&u4DoneMask)) != NULL) {
			fgFound = TRUE;
			_CB_FreeEventNoLock(prLast, u4DoneMask);
		}
		spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
	} while (fgFound);
}

//-----------------------------------------------------------------------------
/** Clear entries with specified eFctId and PID in event list
 */
//-----------------------------------------------------------------------------
void _CB_ClearPendingFctEventEntries(CB_FCT_ID_T eFctId)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	unsigned long rState;
	BOOL fgFound;
	UINT32 u4DoneMask;

	do {
		fgFound = FALSE;

		spin_lock_irqsave(&mt53xx_cb_lock, rState);
		DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
			if (!prLast) {
				break;
			}

			if (prLast->rGetCb.eFctId >= CB_FCT_NUM) {
				break;
			}

			if (prLast->rGetCb.eFctId != eFctId) {
				continue;
			}

			if (current->tgid ==
			    _arCbId[prLast->rGetCb.eFctId]) {
				u4DoneMask = BITMASK_SINGLE;
				fgFound = TRUE;
				break;
			} else if (current->tgid ==
				   _arMultiCbId[prLast->rGetCb.
						eFctId][0]) {
				u4DoneMask = BITMASK_MULTI1;
				fgFound = TRUE;
				break;
			} else if (current->tgid ==
				   _arMultiCbId[prLast->rGetCb.
						eFctId][1]) {
				u4DoneMask = BITMASK_MULTI2;
				fgFound = TRUE;
				break;
			} else if (current->tgid ==
				   _arMultiCbId[prLast->rGetCb.
						eFctId][2]) {
				u4DoneMask = BITMASK_MULTI3;
				fgFound = TRUE;
				break;
			}
		}
		if (fgFound && (prLast != NULL)) {
			_CB_FreeEventNoLock(prLast, u4DoneMask);
		}
		spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
	} while (fgFound);
}
#else
//-----------------------------------------------------------------------------
/** Clear specified PID entries in event list
 */
//-----------------------------------------------------------------------------
void _CB_ClearPendingEventEntries(void)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	unsigned long rState;
	BOOL fgFound;

	do {
		fgFound = FALSE;

		spin_lock_irqsave(&mt53xx_cb_lock, rState);
		DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
			if (!prLast) {
				break;
			}

			if (prLast->rGetCb.eFctId >= CB_FCT_NUM) {
				break;
			}

			if (current->tgid ==
			    _arCbId[prLast->rGetCb.eFctId]) {
				fgFound = TRUE;
				break;
			}

			if (prLast->rLink.pt_next == NULL) {
				break;
			}
		}
		if (fgFound && (prLast != NULL)) {
			DLIST_REMOVE(prLast, &_qCbEvent, rLink);
		}
		spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
	} while (fgFound);
}

//-----------------------------------------------------------------------------
/** Clear entries with specified eFctId and PID in event list
 */
//-----------------------------------------------------------------------------
void _CB_ClearPendingFctEventEntries(CB_FCT_ID_T eFctId)
{
	CB_CALLBACK_EVENT_T *prLast = NULL;
	unsigned long rState;
	BOOL fgFound;

	do {
		fgFound = FALSE;

		spin_lock_irqsave(&mt53xx_cb_lock, rState);
		DLIST_FOR_EACH(prLast, &_qCbEvent, rLink) {
			if (!prLast) {
				break;
			}

			if (prLast->rGetCb.eFctId >= CB_FCT_NUM) {
				break;
			}

			if (current->tgid ==
			    _arCbId[prLast->rGetCb.eFctId]) {
				if (prLast->rGetCb.eFctId == eFctId) {
					fgFound = TRUE;
					break;
				}
			}

			if (prLast->rLink.pt_next == NULL) {
				break;
			}
		}
		if (fgFound && (prLast != NULL)) {
			DLIST_REMOVE(prLast, &_qCbEvent, rLink);
		}
		spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
	} while (fgFound);
}
#endif

//-----------------------------------------------------------------------------
/** Clear specified PID entries in CB PID table
 */
//-----------------------------------------------------------------------------
void _CB_ClearCbIdArray(void)
{
	INT32 i;
	unsigned long rState;

	spin_lock_irqsave(&mt53xx_cb_lock, rState);
	for (i = 0; i < CB_FCT_NUM; i++) {
		if (current->tgid == _arCbId[i]) {
			_arCbId[i] = (pid_t) 0;
		}
#ifdef CC_MTAL_MULTIPROCESS_CALLBACK
		if (current->tgid == _arMultiCbId[i][0]) {
			_arMultiCbId[i][0] = (pid_t) 0;
		}
		if (current->tgid == _arMultiCbId[i][1]) {
			_arMultiCbId[i][1] = (pid_t) 0;
		}
		if (current->tgid == _arMultiCbId[i][2]) {
			_arMultiCbId[i][2] = (pid_t) 0;
		}
#endif
	}
	spin_unlock_irqrestore(&mt53xx_cb_lock, rState);
}
