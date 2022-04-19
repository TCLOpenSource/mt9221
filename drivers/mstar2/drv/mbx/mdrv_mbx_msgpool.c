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
/// file    mdrv_mbx_msgpool.c
/// @brief  MStar MailBox DDI
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _DRV_MBX__MSGPOOL_C

//=============================================================================
// Include Files
//=============================================================================
#include <linux/spinlock.h>
#include <linux/mm.h>

#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"
#include "mdrv_mbx_msgpool.h"

//=============================================================================
// Compile options
//=============================================================================


//=============================================================================
// Local Defines
//=============================================================================
#define MBX_MSGPOOL_PAGES  (14)

//=============================================================================
// Debug Macros
//=============================================================================
#define MSGPOOL_DEBUG
#ifdef MSGPOOL_DEBUG
    #define MSGPOOL_PRINT(fmt, args...)      printk("[MailBox (Driver)][%05d] " fmt, __LINE__, ## args)
    #define MSGPOOL_ASSERT(_cnd, _fmt, _args...)                   \
                                    if (!(_cnd)) {              \
                                        MSGPOOL_PRINT(_fmt, ##_args);  \
                                        while(1);               \
                                    }
#else
    #define MSGPOOL_PRINT(_fmt, _args...)
    #define MSGPOOL_ASSERT(_cnd, _fmt, _args...)
#endif

//----------------------------------------------------------------------------
// Macros
//----------------------------------------------------------------------------
#define MSGPOOL_MSGQ_MAX  E_MBX_CLASS_MAX

//----------------------------------------------------------------------------
// Local Variables
//----------------------------------------------------------------------------
MSGPOOL_MsgPoolInfo _infoMSGP;
MSGPOOL_LastRecvMsgInfo _infoLatestMSGP;
//static MSGPOOL_MsgQMgr _msgQMgrMSGQ[MSGPOOL_MSGQ_MAX];
static DEFINE_SPINLOCK(lock);

//----------------------------------------------------------------------------
// Global Variables
//----------------------------------------------------------------------------
extern MBX_ASYNC_NOTIFIER _mbxAsyncNotifierMBX[MBX_ASYNCNOTIFIER_MAX];

//=============================================================================
// Local Function Prototypes
//=============================================================================

//=============================================================================
// Mailbox Driver MSG POOL Function
static void _MDrv_MSGPOOL_FreeMSGPoolItemQ(MS_S16 s16MsgFirst);
static void _MDrv_MSGPOOL_SetMsgToPoolItem(MS_S16 s16SlotIdx, MBX_Msg* pMbxMsg, MS_U16 u16MsgID);
static void _MDrv_MSGPOOL_GetMsgFromPoolItem(MS_S16 s16SlotIdx, MBX_Msg* pMbxMsg);
static void _MDrv_MSGPOOL_GetMsgIDFromPoolItem(MS_S16 s16SlotIdx, MS_U16 *pu16MsgRecvID);
static MSGPOOL_Result _MDrv_MSGPOOL_AllocateSlot(MS_S16* s16SlotIdx);
static MSGPOOL_Result _MDrv_MSGPOOL_FreeSlot(MS_S16 s16SlotIdx);
static MS_S16 _MDrv_MSGPOOL_GetNextSlot(MS_S16 s16SlotIdx);
static MSGPOOL_Result _MDrv_MSGPOOL_RegisterSlots(MS_U16 u16RegisteSlotNum);
static MSGPOOL_Result _MDrv_MSGPOOL_UnRegisterSlots(MS_U16 u16RegisteSlotNum);

//=============================================================================
// Mailbox Driver MSG Q Function
static void _MDrv_MSGQ_FreeMSG(MSGPOOL_MsgQMgr *pMsgQmgr);

//=============================================================================
// Mailbox Driver MSG POOL Function

//-------------------------------------------------------------------------------------------------
/// Free mail message items in message queue
/// @param  s16MsgFirst                  \b IN: The first message item slot idx for msg q which need free
/// @return void
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_MSGPOOL_FreeMSGPoolItemQ(MS_S16 s16MsgFirst)
{
    MS_S16 s16MsgNext = s16MsgFirst;
    while(IS_VALID_PTR(s16MsgNext))
    {
        s16MsgFirst = s16MsgNext;
	 s16MsgNext = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);

	 _MDrv_MSGPOOL_FreeSlot(s16MsgFirst);
    }
}

//-------------------------------------------------------------------------------------------------
/// set mail message to mail message item
/// @param  s16SlotIdx                  \b IN: The  message item slot idx for setting msg
/// @param  pMbxMsg                  \b IN: The  message to set
/// @return void
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_MSGPOOL_SetMsgToPoolItem(MS_S16 s16SlotIdx, MBX_Msg* pMbxMsg, MS_U16 u16MsgID)
{
    _infoMSGP.pMsgPool[s16SlotIdx].mbxMsg = *pMbxMsg;
    _infoMSGP.pMsgPool[s16SlotIdx].u16MsgID = u16MsgID;
}

//-------------------------------------------------------------------------------------------------
/// get mail message from mail message item
/// @param  s16SlotIdx                  \b IN: The  message item slot idx for getting msg
/// @param  pMbxMsg                  \b OUT: the output msg
/// @return void
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_MSGPOOL_GetMsgFromPoolItem(MS_S16 s16SlotIdx, MBX_Msg* pMbxMsg)
{
    *pMbxMsg = _infoMSGP.pMsgPool[s16SlotIdx].mbxMsg;
}

static void _MDrv_MSGPOOL_GetMsgIDFromPoolItem(MS_S16 s16SlotIdx, MS_U16 *pu16MsgRecvID)
{
    *pu16MsgRecvID = _infoMSGP.pMsgPool[s16SlotIdx].u16MsgID;
}
//-------------------------------------------------------------------------------------------------
/// allocate a free slot for message
/// @param  s16SlotIdx                  \b OUT: The Allocated slot idx
/// @return E_MSGPOOL_ERR_NO_MORE_MEMORY: no more free slots
/// @return E_MSGPOOL_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result _MDrv_MSGPOOL_AllocateSlot(MS_S16* s16SlotIdx)
{
    MS_S16 s16Slot;
    volatile MSGPOOL_MsgPoolItem * pMsgPoolItem = _infoMSGP.pMsgPool;


    if(_infoMSGP.u16FreeSlots<=0)
    {
        return E_MSGPOOL_ERR_NO_MORE_MEMORY;
    }

    for(s16Slot=0; s16Slot<_infoMSGP.u16Slots; s16Slot++)
    {
        if(pMsgPoolItem[s16Slot].u16Usage == FALSE )
        {
            pMsgPoolItem[s16Slot].u16Usage = TRUE;
            pMsgPoolItem[s16Slot].s16Next = INVALID_PTR;
            pMsgPoolItem[s16Slot].u16MsgID = 0;
            _infoMSGP.u16FreeSlots--;
            *s16SlotIdx = s16Slot;
            break;
        }

    }

    return  E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// free a slot for message
/// @param  s16SlotIdx                  \b IN: The free slot idx
/// @return E_MSGPOOL_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result _MDrv_MSGPOOL_FreeSlot(MS_S16 s16SlotIdx)
{
    volatile MSGPOOL_MsgPoolItem * pMsgPoolItem = _infoMSGP.pMsgPool;

    //MSGPOOL_ASSERT((IS_VALID_PTR(s16SlotIdx)), "The slot idx for free is invliad!\n");
    //MSGPOOL_ASSERT((pMsgPoolItem[s16SlotIdx].u16Usage==TRUE), "The slot for free is not used!\n");
    if(!IS_VALID_PTR(s16SlotIdx))
    {
        MSGPOOL_PRINT("[MailBox (Driver)]The slot idx for free is invliad!\n");
        return E_MSGPOOL_UNKNOW_ERROR;
    }
    if(pMsgPoolItem[s16SlotIdx].u16Usage==FALSE)
    {
        MSGPOOL_PRINT("[MailBox (Driver)]The slot for free is not used!\n");
        return E_MSGPOOL_UNKNOW_ERROR;
    }

    pMsgPoolItem[s16SlotIdx].u16Usage = FALSE;
    pMsgPoolItem[s16SlotIdx].s16Next = INVALID_PTR;
    pMsgPoolItem[s16SlotIdx].u16MsgID = 0;
    _infoMSGP.u16FreeSlots++;

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Get Next Slot Idx from input slot idx;
/// @param  s16SlotIdx                  \b IN: The slot idx for get next
/// @return MS_S16:the next slot idx of input idx;
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MS_S16 _MDrv_MSGPOOL_GetNextSlot(MS_S16 s16SlotIdx)
{
    //MSGPOOL_ASSERT((IS_VALID_PTR(s16SlotIdx)), "The slot idx is invliad! %x\n", s16SlotIdx);
    if(!IS_VALID_PTR(s16SlotIdx))
    {
        printk("[MailBox (Driver)]The slot idx is invliad! %x\n", s16SlotIdx);
        return INVALID_PTR;
    }
    return _infoMSGP.pMsgPool[s16SlotIdx].s16Next;
}

//-------------------------------------------------------------------------------------------------
/// set Next Slot Idx to input slot idx;
/// @param  s16PreSlotIdx                  \b IN: The slot idx for set next
/// @param  s16NextSlotIdx                  \b IN: The next slot idx for set
/// @return void
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_MSGPOOL_SetNextSlot(MS_S16 s16PreSlotIdx, MS_S16 s16NextSlotIdx)
{
    _infoMSGP.pMsgPool[s16PreSlotIdx].s16Next = s16NextSlotIdx;
}

//-------------------------------------------------------------------------------------------------
/// register a number of slots from msg pool
/// @param  u16RegisteSlotNum                  \b IN: The register slot number
/// @return E_MSGPOOL_ERR_NO_MORE_MEMORY: no more free slots for register
/// @return E_MSGPOOL_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result _MDrv_MSGPOOL_RegisterSlots(MS_U16 u16RegisteSlotNum)
{
    MS_U16 u16FreeSlots;

    u16FreeSlots = _infoMSGP.u16Slots - _infoMSGP.u16RegistedSlots;

    if(u16RegisteSlotNum > u16FreeSlots) //no enough slots for registe
    {
        return E_MSGPOOL_ERR_NO_MORE_MEMORY;
    }

    //have memory for register:
    _infoMSGP.u16RegistedSlots += u16RegisteSlotNum;

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// un-register a number of slots from msg pool
/// @param  u16UnRegisteSlotNum                  \b IN: The un register slot number
/// @return E_MSGPOOL_ERR_NO_MORE_MEMORY: no more free slots for register
/// @return E_MSGPOOL_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result _MDrv_MSGPOOL_UnRegisterSlots(MS_U16 u16UnRegisteSlotNum)
{
    if(_infoMSGP.u16RegistedSlots < u16UnRegisteSlotNum)
    {
        return E_MSGPOOL_ERR_INVALID_PARAM;
    }

    _infoMSGP.u16RegistedSlots -= u16UnRegisteSlotNum;

    return E_MSGPOOL_SUCCESS;
}

//=============================================================================
// Mailbox Driver MSG Q Function

//-------------------------------------------------------------------------------------------------
/// Free mail message items in message queue
/// @param  s16MsgFirst                  \b IN: The first message item slot idx for msg q which need free
/// @return MS_S16:INVALID_PTR: no more async notifier could be allocated
/// @return MS_S16:Async Notifer ID: between 0-MBX_ASYNCNOTIFIER_MAX
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_MSGQ_FreeMSG(MSGPOOL_MsgQMgr *pMsgQmgr)
{
    _MDrv_MSGPOOL_FreeMSGPoolItemQ(pMsgQmgr->s16MsgFirst);
    _MDrv_MSGPOOL_FreeMSGPoolItemQ(pMsgQmgr->s16InstantMsgFirst);
    _MDrv_MSGPOOL_UnRegisterSlots(pMsgQmgr->u16MsgQSize);

    pMsgQmgr->s16MsgFirst
        = pMsgQmgr->s16MsgEnd
        = pMsgQmgr->s16InstantMsgFirst
        = pMsgQmgr->s16InstantMsgEnd
        = INVALID_PTR;

    pMsgQmgr->u16MsgNum
        = pMsgQmgr->u16InstantMsgNum
        = 0;

    pMsgQmgr->u16MsgQStatus = E_MSGQ_INVALID;
    pMsgQmgr->u16MsgQSize = 0;

    pMsgQmgr->s16MsgQNotifierID = INVALID_PTR;
    pMsgQmgr->s16NextMsgQ = INVALID_PTR;
}

//-------------------------------------------------------------------------------------------------
/// Clear mail message items in message queue
/// @param  s16MsgQIdx                  \b IN: The first message item slot idx for msg q which need free
/// @attention
/// <b>[MXLIB] <em></em></b>
//-------------------------------------------------------------------------------------------------
void _MDrv_MSGQ_ClearMSG(MSGPOOL_MsgQMgr *pMsgQmgr)
{
    _MDrv_MSGPOOL_FreeMSGPoolItemQ(pMsgQmgr->s16MsgFirst);
    _MDrv_MSGPOOL_FreeMSGPoolItemQ(pMsgQmgr->s16InstantMsgFirst);

    pMsgQmgr->s16MsgFirst
        = pMsgQmgr->s16MsgEnd
        = pMsgQmgr->s16InstantMsgFirst
        = pMsgQmgr->s16InstantMsgEnd
        = INVALID_PTR;

    pMsgQmgr->u16MsgNum
        = pMsgQmgr->u16InstantMsgNum
        = 0;

    pMsgQmgr->u16MsgQStatus = E_MSGQ_EMPTY;
}

//----------------------------------------------------------------------------
//=============================================================================
// Mailbox Driver MSG Pool Function
//=============================================================================

//-------------------------------------------------------------------------------------------------
/// Init the message Pool
/// @return E_MSGPOOL_ERR_NO_MORE_MEMORY:allocate pool failed
/// @return E_MSGPOOL_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGPOOL_Init()
{
    //MS_U32 u32PageIdx;
    //struct page *pPage;
    _infoMSGP.pMsgPool = (MSGPOOL_MsgPoolItem * )__get_free_pages(GFP_KERNEL, get_order(MBX_MSGPOOL_PAGES*(0x01<<PAGE_SHIFT)));

    if(_infoMSGP.pMsgPool == NULL)
    {
        return E_MSGPOOL_ERR_NO_MORE_MEMORY;
    }

    _infoMSGP.u16Slots = (MBX_MSGPOOL_PAGES*(0x01<<PAGE_SHIFT))/sizeof(MSGPOOL_MsgPoolItem);
    _infoMSGP.u16FreeSlots = _infoMSGP.u16Slots;
    _infoMSGP.u16RegistedSlots = 0;

    memset((void*)_infoMSGP.pMsgPool, 0x0, (MBX_MSGPOOL_PAGES*(0x01<<PAGE_SHIFT)));

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// DeInit the message Pool
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
void MDrv_MSGPOOL_DeInit()
{
	if (_infoMSGP.pMsgPool == NULL)
		return ;

    free_pages((TYPE_MBX_C_U64)_infoMSGP.pMsgPool, get_order(MBX_MSGPOOL_PAGES*(0x01<<PAGE_SHIFT)));
}


//----------------------------------------------------------------------------
//=============================================================================
// Mailbox Driver MSG Q Function
//=============================================================================

//-------------------------------------------------------------------------------------------------
/// Init the message Q
/// @return E_MSGPOOL_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
#if 0 //msgQmgr is in notifier
MSGPOOL_Result MDrv_MSGQ_Init()
{
    MS_U16 u16MsgQIdx;

    for(u16MsgQIdx = 0; u16MsgQIdx < MSGPOOL_MSGQ_MAX; u16MsgQIdx++)
    {
        _msgQMgrMSGQ[u16MsgQIdx].s16MsgFirst = _msgQMgrMSGQ[u16MsgQIdx].s16MsgEnd
		    = _msgQMgrMSGQ[u16MsgQIdx].s16InstantMsgFirst = _msgQMgrMSGQ[u16MsgQIdx].s16InstantMsgEnd = INVALID_PTR;

        _msgQMgrMSGQ[u16MsgQIdx].u16MsgNum = _msgQMgrMSGQ[u16MsgQIdx].u16InstantMsgNum = 0;

        _msgQMgrMSGQ[u16MsgQIdx].u16MsgQStatus = E_MSGQ_INVALID;
        _msgQMgrMSGQ[u16MsgQIdx].u16MsgQSize = 0;

        _msgQMgrMSGQ[u16MsgQIdx].s16MsgQNotifierID = INVALID_PTR;
        _msgQMgrMSGQ[u16MsgQIdx].s16NextMsgQ = INVALID_PTR;
    }

    return E_MSGPOOL_SUCCESS;
}
#endif
//-------------------------------------------------------------------------------------------------
/// Register message class
/// @param  s16MsgQNotifierID                  \b IN: The Async Notifier ID with the register class
/// @param  s16MsgQFirst                  \b IN: The first msg Q ID of the Async. Notifer
/// @param  s16MsgQID                  \b IN: The Register Msg Class ID
/// @param  u16MsgQSize                  \b IN: The size of Msg Queue with register class
/// @return E_MSGPOOL_ERR_SLOT_AREADY_OPENNED:the class already registered by this Async. Notifier
/// @return E_MSGPOOL_ERR_SLOT_BUSY:the class already registered by another Async. Notifier
/// @return E_MSGPOOL_ERR_NO_MORE_MEMORY:no request msg q size avaiable
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_RegisterMSG(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQNotifierID, MS_S16 s16MsgQFirst, MS_S16 s16MsgQID, MS_U16 u16MsgQSize)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQID]);

    if (pMsgQmgr->u16MsgQStatus != E_MSGQ_INVALID) //already registed by this app
        return E_MSGPOOL_ERR_SLOT_AREADY_OPENNED;

    if (E_MSGPOOL_ERR_NO_MORE_MEMORY == _MDrv_MSGPOOL_RegisterSlots(u16MsgQSize))
        return E_MSGPOOL_ERR_NO_MORE_MEMORY;

    //success registed, then allocate the msg q:
    pMsgQmgr->s16MsgFirst
        = pMsgQmgr->s16MsgEnd
        = pMsgQmgr->s16InstantMsgFirst
        = pMsgQmgr->s16InstantMsgEnd
        = INVALID_PTR;

    pMsgQmgr->u16MsgNum = pMsgQmgr->u16InstantMsgNum = 0;

    pMsgQmgr->u16MsgQStatus = E_MSGQ_EMPTY;
    pMsgQmgr->u16MsgQSize = u16MsgQSize;

    pMsgQmgr->s16MsgQNotifierID = s16MsgQNotifierID;
    pMsgQmgr->s16NextMsgQ = s16MsgQFirst;

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Register message class
/// @param  s16MsgQNotifierID                  \b IN: The Async Notifier ID with the register class
/// @param  s16MsgQFirst                  \b IN: The first msg Q ID of the Async. Notifer
/// @param  s16MsgQID                  \b IN: The Register Msg Class ID
/// @param  bForceDiscardMsgQueue                  \b IN: If TRUE, un-register whatever the status of msg queue
/// @return E_MSGPOOL_ERR_SLOT_NOT_OPENNED:the class have not been registered yet
/// @return E_MSGPOOL_ERR_SLOT_BUSY:the class already registered by another Async. Notifier
/// @return E_MSGPOOL_ERR_HAS_MSG_PENDING: has pendign msg in msg queue for un-register
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_UnRegisterMSG(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQNotifierID,  MS_S16* ps16MsgQFirst, MS_S16 s16MsgQID, MS_BOOL bForceDiscardMsgQueue)
{
    MS_S16 s16MsgQIdx = *ps16MsgQFirst;
    MSGPOOL_MsgQMgr *pMsgQmgrFirst, *pMsgQmgrRegister;

    pMsgQmgrRegister = &(pMbxAsyncNotifier->msgQmgr[s16MsgQID]);

    //if not opened yet?
    if (pMsgQmgrRegister->u16MsgQStatus == E_MSGQ_INVALID)
        return E_MSGPOOL_ERR_SLOT_NOT_OPENNED;

    //if has pending msg and app not force discard:
    if ((!bForceDiscardMsgQueue) && ((pMsgQmgrRegister->u16InstantMsgNum + pMsgQmgrRegister->u16MsgNum) >  0))
        return E_MSGPOOL_ERR_HAS_MSG_PENDING;

    //free msgq and discard msg if has pending:
    //MSGPOOL_ASSERT(IS_VALID_PTR(*ps16MsgQFirst), "Invlid first MSGQ Ptr for un-register!\n");
    if (!IS_VALID_PTR(*ps16MsgQFirst)) {
        printk("[MailBox (Driver)]Invlid first MSGQ Ptr for un-register!\n");
        return E_MSGPOOL_UNKNOW_ERROR;
    }

    if (s16MsgQIdx != s16MsgQID) {
        pMsgQmgrFirst = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);
        while (pMsgQmgrFirst->s16NextMsgQ != s16MsgQID) {
            s16MsgQIdx = pMsgQmgrFirst->s16NextMsgQ;
            pMsgQmgrFirst = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);
        }

        pMsgQmgrFirst->s16NextMsgQ = pMsgQmgrRegister->s16NextMsgQ;
    } else {
        *ps16MsgQFirst = pMsgQmgrRegister->s16NextMsgQ;
    }

    _MDrv_MSGQ_FreeMSG(pMsgQmgrRegister);

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Clear message class
/// @param  s16MsgQID                  \b IN: The Clear Msg Class ID
/// @return E_MSGPOOL_ERR_SLOT_NOT_OPENNED:the class have not been registered yet
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[MXLIB] <em></em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_ClearMSG(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQID)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQID]);

    //if not opened yet?
    if (pMsgQmgr->u16MsgQStatus == E_MSGQ_INVALID)
        return E_MSGPOOL_ERR_SLOT_NOT_OPENNED;

    //Clear msgq:
    _MDrv_MSGQ_ClearMSG(pMsgQmgr);

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// get status of message class
/// @param  s16MsgQNotifierID                  \b IN: The Async Notifier ID with the msg class to get info.
/// @param  s16MsgQID                  \b IN: The Msg Class ID to get info.
/// @param  pMsgQStatus                  \b OUT: the msg class info to put
/// @return E_MSGPOOL_ERR_SLOT_NOT_OPENNED:the class have not been registered yet
/// @return E_MSGPOOL_ERR_SLOT_BUSY:the class already registered by another Async. Notifier
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_GetMsgQStatus(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQID, MBX_MSGQ_Status *pMsgQStatus)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQID]);

    if (pMsgQmgr->u16MsgQStatus== E_MSGQ_INVALID) {
        //already registed by this app
        return E_MSGPOOL_ERR_SLOT_NOT_OPENNED;
    }

    pMsgQStatus->status = 0;

    if (pMsgQmgr->u16MsgQStatus == E_MSGQ_OVERFLOW)
        pMsgQStatus->status |= MBX_STATUS_QUEUE_OVER_FLOW;

    if (pMsgQmgr->u16MsgNum > 0)
        pMsgQStatus->status |= MBX_STATUS_QUEUE_HAS_NORMAL_MSG;

    if (pMsgQmgr->u16InstantMsgNum > 0)
        pMsgQStatus->status |= MBX_STATUS_QUEUE_HAS_INSTANT_MSG;

    pMsgQStatus->u32InstantMsgCount = pMsgQmgr->u16InstantMsgNum;
    pMsgQStatus->u32NormalMsgCount = pMsgQmgr->u16MsgNum;

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// uNRegister message class queue
/// @param  s16MsgQFirst                  \b IN: The first msg Q ID for free
/// @param  bForceDiscardMsgQueue                  \b IN: If TRUE, un-register whatever the status of msg queue
/// @return E_MSGPOOL_ERR_HAS_MSG_PENDING: has pendign msg in msg queue for un-register
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_UnRegisterMSGQ(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQFirst, MS_BOOL bForceDiscardPendingMsg)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;
    MS_S16 s16MsgQNext = s16MsgQFirst;

    if (!bForceDiscardPendingMsg) {
        while (IS_VALID_PTR(s16MsgQNext)) {
            pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQNext]);
            if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) >  0)
                return E_MSGPOOL_ERR_HAS_MSG_PENDING;

            s16MsgQNext = pMsgQmgr->s16NextMsgQ;
        }
    }

    s16MsgQNext = s16MsgQFirst;

    while (IS_VALID_PTR(s16MsgQNext)) {
        s16MsgQFirst = s16MsgQNext;
        pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQFirst]);
        s16MsgQNext = pMsgQmgr->s16NextMsgQ;

        _MDrv_MSGQ_FreeMSG(pMsgQmgr);
    }

    return E_MSGPOOL_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// recv message from msg class queue
/// @param  s16MsgQIdx                  \b IN: The  msg Q ID for recv
/// @param  pMsg                  \b OUT: the msg to put
/// @param  bInstantMsg                  \b IN: get the instant msg or normal msg
/// @return E_MSGPOOL_ERR_NO_MORE_MSG: no msg for recv
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_RecvMsg(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQIdx, MBX_Msg* pMsg, MS_BOOL bInstantMsg, MS_U16 *pu16MsgRecvID)
{
    MS_S16 s16MsgSlotIdx;
    MSGPOOL_Result msgPoolResult;
    MS_U32 flags;
    MSGPOOL_MsgQMgr *pMsgQmgr;

    spin_lock_irqsave(&lock, flags);
    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);
    if (bInstantMsg) {
        //get instant msg:
        if (pMsgQmgr->u16InstantMsgNum > 0) {
            s16MsgSlotIdx = pMsgQmgr->s16InstantMsgFirst;

            if (s16MsgSlotIdx == INVALID_PTR) {
                _MDrv_MSGQ_ClearMSG(pMsgQmgr);
                spin_unlock_irqrestore(&lock, flags);

                return E_MSGPOOL_ERR_NO_MORE_MSG;
            }

            _MDrv_MSGPOOL_GetMsgFromPoolItem(s16MsgSlotIdx, pMsg);
            _MDrv_MSGPOOL_GetMsgIDFromPoolItem(s16MsgSlotIdx, pu16MsgRecvID);
            pMsgQmgr->s16InstantMsgFirst = _MDrv_MSGPOOL_GetNextSlot(s16MsgSlotIdx);
            _MDrv_MSGPOOL_FreeSlot(s16MsgSlotIdx);
            pMsgQmgr->u16InstantMsgNum--;
            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    } else {
        //get normal msg:
        if (pMsgQmgr->u16MsgNum > 0) {
            s16MsgSlotIdx = pMsgQmgr->s16MsgFirst;

            if (s16MsgSlotIdx == INVALID_PTR) {
                _MDrv_MSGQ_ClearMSG(pMsgQmgr);
                spin_unlock_irqrestore(&lock, flags);

                return E_MSGPOOL_ERR_NO_MORE_MSG;
            }

            _MDrv_MSGPOOL_GetMsgFromPoolItem(s16MsgSlotIdx, pMsg);
            _MDrv_MSGPOOL_GetMsgIDFromPoolItem(s16MsgSlotIdx, pu16MsgRecvID);
            pMsgQmgr->s16MsgFirst = _MDrv_MSGPOOL_GetNextSlot(s16MsgSlotIdx);
            _MDrv_MSGPOOL_FreeSlot(s16MsgSlotIdx);
            pMsgQmgr->u16MsgNum--;
            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    }
    spin_unlock_irqrestore(&lock, flags);

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) < pMsgQmgr->u16MsgQSize)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_NORMAL;

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) <= 0)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_EMPTY;

    return msgPoolResult;
}

MSGPOOL_Result MDrv_MSGQ_DequeueMsg(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQIdx, MS_BOOL bInstantMsg, MS_U16 u16MsgRecvID)
{
    MS_S16 s16MsgSlotIdx;
    MSGPOOL_Result msgPoolResult;
    MS_U32 flags;
    MSGPOOL_MsgQMgr *pMsgQmgr;
    MS_U16 u16MsgRecvIDGet = 0;
    MS_S16 s16MsgNext, s16MsgFirst, s16MsgForward;

    spin_lock_irqsave(&lock, flags);
    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);
    if (bInstantMsg) {
        //get instant msg:
        if (pMsgQmgr->u16InstantMsgNum > 0) {
            s16MsgSlotIdx = pMsgQmgr->s16InstantMsgFirst;

            if (s16MsgSlotIdx == INVALID_PTR) {
                _MDrv_MSGQ_ClearMSG(pMsgQmgr);
                spin_unlock_irqrestore(&lock, flags);

                return E_MSGPOOL_ERR_NO_MORE_MSG;
            }

            /* remove the ID of the message */
            s16MsgNext = s16MsgSlotIdx;
            while (IS_VALID_PTR(s16MsgNext)) {
                s16MsgFirst = s16MsgNext;
                s16MsgForward = s16MsgFirst;

                _MDrv_MSGPOOL_GetMsgIDFromPoolItem(s16MsgFirst, &u16MsgRecvIDGet);

                if (u16MsgRecvID == u16MsgRecvIDGet) {
                    if (s16MsgFirst == s16MsgSlotIdx) {
                        /* the msg pool index of the first msg changed */
                        pMsgQmgr->s16InstantMsgFirst = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);
                    } else {
                        /* set forward's next to now's next */
                        s16MsgNext = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);
                        _MDrv_MSGPOOL_SetNextSlot(s16MsgForward, s16MsgNext);
                    }

                    _MDrv_MSGPOOL_FreeSlot(s16MsgFirst);
                    pMsgQmgr->u16InstantMsgNum--;
                    msgPoolResult = E_MSGPOOL_SUCCESS;
                    break;
                }

                s16MsgForward = s16MsgFirst;
                s16MsgNext = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);
            }
            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    } else {
        //get normal msg:
        if (pMsgQmgr->u16MsgNum > 0) {
            s16MsgSlotIdx = pMsgQmgr->s16MsgFirst;

            if (s16MsgSlotIdx == INVALID_PTR) {
                _MDrv_MSGQ_ClearMSG(pMsgQmgr);
                spin_unlock_irqrestore(&lock, flags);

                return E_MSGPOOL_ERR_NO_MORE_MSG;
            }

            /* remove the ID of the message */
            s16MsgNext = s16MsgSlotIdx;
            while (IS_VALID_PTR(s16MsgNext)) {
                s16MsgFirst = s16MsgNext;
                s16MsgForward = s16MsgFirst;

                _MDrv_MSGPOOL_GetMsgIDFromPoolItem(s16MsgFirst, &u16MsgRecvIDGet);

                if (u16MsgRecvID == u16MsgRecvIDGet) {
                    if (s16MsgFirst == s16MsgSlotIdx) {
                        /* the msg pool index of the first msg changed */
                        pMsgQmgr->s16MsgFirst = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);
                    } else {
                        /* set forward's next to now's next */
                        s16MsgNext = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);
                        _MDrv_MSGPOOL_SetNextSlot(s16MsgForward, s16MsgNext);
                    }
                    _MDrv_MSGPOOL_FreeSlot(s16MsgFirst);
                    pMsgQmgr->u16MsgNum--;
                    msgPoolResult = E_MSGPOOL_SUCCESS;
                    break;
                }

                s16MsgForward = s16MsgFirst;
                s16MsgNext = _MDrv_MSGPOOL_GetNextSlot(s16MsgFirst);
            }
            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    }
    spin_unlock_irqrestore(&lock, flags);

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) < pMsgQmgr->u16MsgQSize)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_NORMAL;

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) <= 0)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_EMPTY;

    return msgPoolResult;
}

//-------------------------------------------------------------------------------------------------
/// Check message from msg class queue
/// @param  s16MsgQIdx                  \b IN: The  msg Q ID for check
/// @param  pMsg                  \b OUT: the msg to put
/// @param  bInstantMsg                  \b IN: get the instant msg or normal msg
/// @return E_MSGPOOL_ERR_NO_MORE_MSG: no msg for recv
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
/// <b>[OBAMA] <em>This pMsg contains no message content, only MsgClass available for checking</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_CheckMsg(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQIdx, MBX_Msg* pMsg, MS_BOOL bInstantMsg)
{
    MS_S16 s16MsgSlotIdx;
    MSGPOOL_Result msgPoolResult;
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);
    if (bInstantMsg) {
        //check instant msg:
        if (pMsgQmgr->u16InstantMsgNum > 0) {
            s16MsgSlotIdx = pMsgQmgr->s16InstantMsgFirst;
            _MDrv_MSGPOOL_GetMsgFromPoolItem(s16MsgSlotIdx, pMsg);

            _infoLatestMSGP.bInstantMsg=bInstantMsg;
            _infoLatestMSGP.s16MsgQIdx=s16MsgQIdx;
            _infoLatestMSGP.s16MsgSlotIdx=s16MsgSlotIdx;

            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    } else {
        //check normal msg:
        if (pMsgQmgr->u16MsgNum > 0) {
            s16MsgSlotIdx = pMsgQmgr->s16MsgFirst;
            _MDrv_MSGPOOL_GetMsgFromPoolItem(s16MsgSlotIdx, pMsg);

            _infoLatestMSGP.bInstantMsg=bInstantMsg;
            _infoLatestMSGP.s16MsgQIdx=s16MsgQIdx;
            _infoLatestMSGP.s16MsgSlotIdx=s16MsgSlotIdx;

            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    }

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) < pMsgQmgr->u16MsgQSize)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_NORMAL;

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) <= 0)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_EMPTY;

    return msgPoolResult;
}

//-------------------------------------------------------------------------------------------------
/// remove latest message from msg class queue
/// @return E_MSGPOOL_ERR_NO_MORE_MSG: no msg for recv
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_RemoveLatestMsg(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier)
{
    MS_S16 s16MsgSlotIdx, s16MsgQIdx;
    MS_BOOL bInstantMsg;
    MSGPOOL_Result msgPoolResult;
    MSGPOOL_MsgQMgr *pMsgQmgr;

    s16MsgQIdx = _infoLatestMSGP.s16MsgQIdx;
    bInstantMsg = _infoLatestMSGP.bInstantMsg;
    s16MsgSlotIdx = _infoLatestMSGP.s16MsgSlotIdx;
    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);

    if (bInstantMsg) {
        //check instant msg:
        if (pMsgQmgr->u16InstantMsgNum > 0) {
            pMsgQmgr->s16InstantMsgFirst = _MDrv_MSGPOOL_GetNextSlot(s16MsgSlotIdx);
            _MDrv_MSGPOOL_FreeSlot(s16MsgSlotIdx);
            pMsgQmgr->u16InstantMsgNum--;

            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    } else {
        //check normal msg:
        if (pMsgQmgr->u16MsgNum > 0) {
            pMsgQmgr->s16MsgFirst = _MDrv_MSGPOOL_GetNextSlot(s16MsgSlotIdx);
            _MDrv_MSGPOOL_FreeSlot(s16MsgSlotIdx);
            pMsgQmgr->u16MsgNum--;

            msgPoolResult = E_MSGPOOL_SUCCESS;
        } else {
            msgPoolResult = E_MSGPOOL_ERR_NO_MORE_MSG;
        }
    }

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) < pMsgQmgr->u16MsgQSize)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_NORMAL;

    if ((pMsgQmgr->u16InstantMsgNum + pMsgQmgr->u16MsgNum) <= 0)
        pMsgQmgr->u16MsgQStatus = E_MSGQ_EMPTY;

    return msgPoolResult;
}


//-------------------------------------------------------------------------------------------------
/// Get Next msg q Idx from input msg q idx;
/// @param  s16SlotIdx                  \b IN: The q idx for get next
/// @return MS_S16:the next msg q idx of input idx;
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MS_S16 MDrv_MSGQ_GetNextMsgQ(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQIdx)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);

    return pMsgQmgr->s16NextMsgQ;
}

//-------------------------------------------------------------------------------------------------
/// add message to msg class queue
/// @param  pMsg                  \b IN: the msg to put
/// @return E_MSGPOOL_ERR_NOT_INITIALIZED: no msg not actived yet
/// @return E_MSGPOOL_ERR_NO_MORE_MSG: no msg memory to add
/// @return E_MSGPOOL_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
MSGPOOL_Result MDrv_MSGQ_AddMSG(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MBX_Msg* pMbxMsg, MS_U16 u16MsgID)
{
    MS_S16 s16MsgItemIdx = -1;
    MS_S16 s16MsgQIdx = pMbxMsg->u8MsgClass;
    MS_U32 flags;
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);

    if ((pMsgQmgr->u16MsgQStatus == E_MSGQ_INVALID))
        return E_MSGPOOL_ERR_NOT_INITIALIZED;

    if (pMsgQmgr->u16MsgQStatus == E_MSGQ_OVERFLOW)
        return E_MSGPOOL_ERR_NO_MORE_MEMORY;

    spin_lock_irqsave(&lock, flags);
    if (_MDrv_MSGPOOL_AllocateSlot(&s16MsgItemIdx) == E_MSGPOOL_SUCCESS) {
        _MDrv_MSGPOOL_SetMsgToPoolItem(s16MsgItemIdx, pMbxMsg, u16MsgID);
        if (pMbxMsg->eMsgType == E_MBX_MSG_TYPE_INSTANT) {
            if (pMsgQmgr->u16InstantMsgNum > 0) {
                _MDrv_MSGPOOL_SetNextSlot(pMsgQmgr->s16InstantMsgEnd, s16MsgItemIdx);
                pMsgQmgr->s16InstantMsgEnd = s16MsgItemIdx;
            } else {
                pMsgQmgr->s16InstantMsgFirst
                    = pMsgQmgr->s16InstantMsgEnd
                    = s16MsgItemIdx;
            }

            pMsgQmgr->u16InstantMsgNum++;
        } else {
            if (pMsgQmgr->u16MsgNum > 0) {
                _MDrv_MSGPOOL_SetNextSlot(pMsgQmgr->s16MsgEnd, s16MsgItemIdx);
                pMsgQmgr->s16MsgEnd = s16MsgItemIdx;
            } else {
                pMsgQmgr->s16MsgFirst
                    = pMsgQmgr->s16MsgEnd
                    = s16MsgItemIdx;
            }

            pMsgQmgr->u16MsgNum++;
        }

        if (pMsgQmgr->u16MsgQStatus == E_MSGQ_EMPTY)
            pMsgQmgr->u16MsgQStatus = E_MSGQ_NORMAL;

        if ((pMsgQmgr->u16MsgNum + pMsgQmgr->u16InstantMsgNum) >= pMsgQmgr->u16MsgQSize)
            pMsgQmgr->u16MsgQStatus = E_MSGQ_OVERFLOW;

        spin_unlock_irqrestore(&lock, flags);
        return E_MSGPOOL_SUCCESS;
    }

    spin_unlock_irqrestore(&lock, flags);
    return E_MSGPOOL_ERR_NO_MORE_MEMORY;
}

//-------------------------------------------------------------------------------------------------
/// get NotifierID by msg Q ID
/// @param  s16MsgQIdx                  \b IN: the msg q id
/// @return void
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
MS_S16 MDrv_MSGQ_GetNotiferIDByQID(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQIdx)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);

    return pMsgQmgr->s16MsgQNotifierID;
}

//-------------------------------------------------------------------------------------------------
/// get NotifierID by msg Q ID
/// @param  s16MsgQIdx                  \b IN: the msg q id
/// @return void
/// @attention
/// <b>[OBAMA] <em></em></b>
//-------------------------------------------------------------------------------------------------
MS_U16 MDrv_MSGQ_GetQStatusByQID(MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier, MS_S16 s16MsgQIdx)
{
    MSGPOOL_MsgQMgr *pMsgQmgr;

    pMsgQmgr = &(pMbxAsyncNotifier->msgQmgr[s16MsgQIdx]);

    return pMsgQmgr->u16MsgQStatus;
}
