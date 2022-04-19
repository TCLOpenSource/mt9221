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
/// file    mdrv_mbx.c
/// @brief  MStar MailBox DDI
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _DRV_MBX_C

#define USE_HW_SEM 0
//=============================================================================
// Include Files
//=============================================================================
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/bug.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif

#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"
#include "mhal_mbx.h"
#include "mhal_mbx_interrupt.h"
#include "mdrv_mbx_msgpool.h"
#include "mdrv_types.h"
#include "uapi/linux/psci.h"
#if (defined(CONFIG_MSTAR_HW_SEM) && (CONFIG_MSTAR_HW_SEM == 1) && (USE_HW_SEM == 1))
#include "mdrv_hw_sem.h"
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#endif

#ifdef CONFIG_MSTAR_PM_WDT
#include <chip_arch.h>
#endif
//=============================================================================
// Compile options
//=============================================================================

//=============================================================================
// Local Defines
//=============================================================================
#if (defined(CONFIG_MSTAR_HW_SEM) && (CONFIG_MSTAR_HW_SEM == 1) && (USE_HW_SEM == 1))

#define LOCK_SEND_HW_SEM() { while(MDrv_SEM_Get_Resource(SEMID_MBX_HKCPU2PM_SEND, RESID_HKCPU) != TRUE); }

#define UNLOCK_SEND_HW_SEM() {\
  if (MDrv_SEM_Free_Resource(SEMID_MBX_HKCPU2PM_SEND, RESID_HKCPU) != TRUE) {\
      printk(KERN_EMERG "===ERROR === Hardware Send Semaphore Free Resource\n");\
      BUG();\
  }\
}

#define LOCK_RECV_HW_SEM() { while(MDrv_SEM_Get_Resource(SEMID_MBX_HKCPU2PM_RECV, RESID_HKCPU) != TRUE); }

#define UNLOCK_RECV_HW_SEM() {\
  if (MDrv_SEM_Free_Resource(SEMID_MBX_HKCPU2PM_RECV, RESID_HKCPU) != TRUE) {\
      printk(KERN_EMERG "===ERROR === Hardware Receive Semaphore Free Resource\n");\
      BUG();\
  }\
}

#else

#define LOCK_SEND_HW_SEM()
#define UNLOCK_SEND_HW_SEM()
#define LOCK_RECV_HW_SEM()
#define UNLOCK_RECV_HW_SEM()

#endif /* (defined(CONFIG_MSTAR_HW_SEM) && (CONFIG_MSTAR_HW_SEM == 1) && (USE_HW_SEM == 1)) */

#define MBXCLASS_IN_SIGNAL_MASK 0x0000FF00
#define MBXCLASS_IN_SIGNAL_SHIFT 0x8

//=============================================================================
// Debug Macros
//=============================================================================
#define MBX_DEBUG

#ifdef MBX_DEBUG

#define MBX_PRINT(fmt, args...) printk("[MailBox (Driver)][%05d] " fmt, __LINE__, ## args)
#define MBX_ASSERT(_cnd, _fmt, _args...) if (!(_cnd)) { MBX_PRINT(_fmt, ##_args); }

#else

#define MBX_PRINT(_fmt, _args...)
#define MBX_ASSERT(_cnd, _fmt, _args...)

#endif /* MBX_DEBUG */

//=============================================================================
// Macros
//=============================================================================
static struct mutex _mutexSendMBX;
static struct mutex _mutexSendMBXFRC;
static struct mutex _mutexRecvMBX;
static struct mutex _mutexMBX;
static struct mutex _mutexPM;

#define DRV_MBX_LockSend_Init() mutex_init(&_mutexSendMBX)
#define DRV_MBX_LockSend() mutex_lock(&_mutexSendMBX)
#define DRV_MBX_UnLockSend() mutex_unlock(&_mutexSendMBX)

#define DRV_MBX_LockSend_FRC_Init() mutex_init(&_mutexSendMBXFRC)
#define DRV_MBX_LockSendFRC() mutex_lock(&_mutexSendMBXFRC)
#define DRV_MBX_UnLockSendFRC() mutex_unlock(&_mutexSendMBXFRC)

#define DRV_MBX_LockRecv_Init() mutex_init(&_mutexRecvMBX)
#define DRV_MBX_LockRecv() mutex_lock(&_mutexRecvMBX)
#define DRV_MBX_UnLockRecv() mutex_unlock(&_mutexRecvMBX)

#define DRV_MBX_Lock_Init() mutex_init(&_mutexMBX)
#define DRV_MBX_Lock() mutex_lock(&_mutexMBX)
#define DRV_MBX_UnLock() mutex_unlock(&_mutexMBX)

#define DRV_PM_Lock_Init() mutex_init(&_mutexPM)
#define DRV_PM_Lock() mutex_lock(&_mutexPM)
#define DRV_PM_UnLock() mutex_unlock(&_mutexPM)

MBX_ASYNC_NOTIFIER _mbxAsyncNotifierMBX[MBX_ASYNCNOTIFIER_MAX];
extern MSGPOOL_MsgPoolInfo _infoMSGP;
extern MSGPOOL_LastRecvMsgInfo _infoLatestMSGP;

static MBX_ROLE_ID _eMbxHostRole = E_MBX_ROLE_MAX;
static MBX_ROLE_ID _eMbxCpu2Role[E_MBX_CPU_MAX];
static MBX_CPU_ID _eMbxRole2Cpu[10];
static MS_U32 _u32TimeoutMillSecs = 0;
static MS_U16 _u16MsgIDCounter = 0;

static MS_BOOL _bCalledFromDriver = FALSE;

static struct completion mbx_completion[E_MBX_CLASS_MAX];
static MS_U32 completionInit = 0;
static MS_U32 g_u32AMBXAsyncID;
static MS_S16 _lastNotifierIndex = 0;

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static int mdb_mbx_node_open(struct inode *inode, struct file *file);
static ssize_t mdb_mbx_node_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);

const struct file_operations mdb_mbx_node_operations = {
    .owner      = THIS_MODULE,
    .open       = mdb_mbx_node_open,
    .read       = seq_read,
    .write      = mdb_mbx_node_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#endif /* CONFIG_MSTAR_UTOPIA2K_MDEBUG */

//=============================================================================
// Global Variables
//=============================================================================

//=============================================================================
// Local Function Prototypes
//=============================================================================

//=============================================================================
//Async. Notifier functions:
static MS_S16 _MDrv_MBX_AllocateAsyncNotifier(TYPE_MBX_C_U64 u32AsyncID);
//static MS_S16 _MDrv_MBX_FreeAsyncNotifier(MS_U32 u32AsyncID);
static void _MDrv_MBX_FreeAsyncNotifierByID(MS_U16 u16NotifierID);
static MS_S16 _MDrv_MBX_GetAsyncNotifierIDByAsyncID(TYPE_MBX_C_U64 u32AsyncID);

//=============================================================================
//MBX Config functions:
static MBX_Result _MDrv_MBX_InitConfig(MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs);
static MBX_Result _MDrv_MBX_SetConfig(MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs);

//Util Function
static MS_U32 _MDrv_MBX_GetSystemTime(void); //in ms

//Message Handle functions:
static MBX_Result _MDrv_MBX_SendMsg(MBX_Msg *pMsg, MBX_ROLE_ID eSrcRole);
static void _MDrv_MBX_DequeueMsg(MBX_Class eTargetClass, MS_BOOL bInstantMsg, MS_U16 u16MsgIDRemove);
static MBX_Result _MDrv_MBX_RecvMsg(MS_S16 mbxAsyncNotifierID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32Flag);
static MBX_Result _MDrv_MBX_CheckMsg(MS_S16 mbxAsyncNotifierID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32Flag);

//=============================================================================
//Message interrupt callback function:
static void _MDrv_MBX_MsgRecvCb(MS_S32 s32Irq);
void (*MDrv_MBX_MsgCbFunc)(void);

//=============================================================================
// Local Function
//=============================================================================

//-------------------------------------------------------------------------------------------------
/// Allocate one Async Notifier
/// @param  u32AsyncID                  \b IN: The Async ID
/// @return MS_S16:INVALID_PTR: no more async notifier could be allocated
/// @return MS_S16:Async Notifer ID: between 0-MBX_ASYNCNOTIFIER_MAX
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
static MS_S16 _MDrv_MBX_AllocateAsyncNotifier(TYPE_MBX_C_U64 u32AsyncID)
{
    MS_S16 s16Idx;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_Lock();
    for (s16Idx = 0, pMbxAsyncNotifier = _mbxAsyncNotifierMBX; s16Idx < MBX_ASYNCNOTIFIER_MAX; s16Idx++, pMbxAsyncNotifier++) {
        if (pMbxAsyncNotifier->u16Usage == FALSE) {
            pMbxAsyncNotifier->u32AsyncID = u32AsyncID;
            pMbxAsyncNotifier->u16Usage = TRUE;
            pMbxAsyncNotifier->s16MsgQFirst = INVALID_PTR;
            pMbxAsyncNotifier->bEnable = FALSE; //default disable.
            break;
        } else {
            if (pMbxAsyncNotifier->u32AsyncID == u32AsyncID)
                break;
        }
    }
    DRV_MBX_UnLock();

    if (s16Idx >= MBX_ASYNCNOTIFIER_MAX)
        return INVALID_PTR;

    if (_lastNotifierIndex < s16Idx)
        _lastNotifierIndex = s16Idx;

    return s16Idx;
}

#if 0
//-------------------------------------------------------------------------------------------------
/// Free one Async Notifier
/// @param  u32AsyncID                  \b IN: The Async ID
/// @return MS_S16:INVALID_PTR: no such an async id has been allocated, free fail
/// @return MS_S16:Async Notifer ID: the freeed notifier id by async id
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
static MS_S16 _MDrv_MBX_FreeAsyncNotifier(MS_U32 u32AsyncID)
{
    MS_S16 s16Idx;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_Lock();
    for (s16Idx = 0, pMbxAsyncNotifier = _mbxAsyncNotifierMBX; s16Idx < MBX_ASYNCNOTIFIER_MAX; s16Idx++, pMbxAsyncNotifier++) {
        if (pMbxAsyncNotifier->u32AsyncID == u32AsyncID) {
            pMbxAsyncNotifier->u32AsyncID = NULL;
            pMbxAsyncNotifier->u16Usage = FALSE;
            pMbxAsyncNotifier->bEnable = FALSE; //default disable.
            break;
        }
    }
    DRV_MBX_UnLock();

    if (s16Idx >= MBX_ASYNCNOTIFIER_MAX)
        return INVALID_PTR;

    return s16Idx;
}
#endif

//-------------------------------------------------------------------------------------------------
/// Free one Async Notifier by Notifer ID
/// @param  u16NotifierID                  \b IN: The notifier ID
/// @return void
/// @attention
/// <b>[OBAMA] <em>Use spin lock to protect co-access</em></b>
//-------------------------------------------------------------------------------------------------
static void _MDrv_MBX_FreeAsyncNotifierByID(MS_U16 u16NotifierID)
{
    MBX_ASSERT((u16NotifierID < MBX_ASYNCNOTIFIER_MAX), "Invalid Async Notifier ID: %x\n", u16NotifierID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_Lock();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[u16NotifierID];
    pMbxAsyncNotifier->u32AsyncID = NULL;
    pMbxAsyncNotifier->u16Usage = FALSE;
    pMbxAsyncNotifier->bEnable = FALSE; //default disable.

    if (_lastNotifierIndex == u16NotifierID)
        _lastNotifierIndex--;
    DRV_MBX_UnLock();
}

//-------------------------------------------------------------------------------------------------
/// Get  Notifier ID by Async ID
/// @param  u32AsyncID                  \b IN: The Async ID
/// @return MS_S16:INVALID_PTR: no such an async id has been allocated, get fail
/// @return MS_S16:Async Notifer ID: the notifier id by async id
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
static MS_S16 _MDrv_MBX_GetAsyncNotifierIDByAsyncID(TYPE_MBX_C_U64 u32AsyncID)
{
    MS_S16 s16Idx;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_Lock();
    for (s16Idx = 0, pMbxAsyncNotifier = _mbxAsyncNotifierMBX; s16Idx < MBX_ASYNCNOTIFIER_MAX; s16Idx++, pMbxAsyncNotifier++) {
        if (pMbxAsyncNotifier->u32AsyncID == u32AsyncID) {
            MBX_ASSERT((pMbxAsyncNotifier->u16Usage), "Invalid Usage of allocated Async ID: %x Notifier ID: %x\n", (unsigned int)u32AsyncID, s16Idx);
            DRV_MBX_UnLock();
            return s16Idx;
        }
    }
    DRV_MBX_UnLock();

    return (INVALID_PTR);
}

//=============================================================================
//MBX Config functions:

//-------------------------------------------------------------------------------------------------
// Init Config parameters to Driver Config Parameters
// @param  eHostCPU    \b IN: The config Host CPU ID\n
// @return E_MBX_SUCCESS
// @return E_MBX_ERR_INVALID_PARAM
// @return E_MBX_UNKNOW_ERROR
// @attention
// <b>[MXLIB] <em>Int Config in MDrv_MBX_Init, or After MDrv_MBX_Init</em></b>
//-------------------------------------------------------------------------------------------------
static MBX_Result _MDrv_MBX_InitConfig(MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs)
{
    //Set eMbxHostRole:
    _eMbxHostRole = eHostRole;
    //Set Timeout:
    _u32TimeoutMillSecs = u32TimeoutMillSecs;

    //Set eMbxRole2Cpu, eMbxCpu2Role
    _eMbxRole2Cpu[E_MBX_ROLE_HK] = eHKCPU;
    _eMbxRole2Cpu[E_MBX_ROLE_PM] = E_MBX_CPU_PM;
#if defined(CONFIG_MSTAR_M5621)
    //_eMbxRole2Cpu[E_MBX_CPU_MIPS_VPE1] = E_MBX_CPU_MIPS_VPE1; //it is a wrong usage
#else
    _eMbxRole2Cpu[E_MBX_CPU_MIPS_VPE1] = E_MBX_CPU_MIPS_VPE1;
#endif /* CONFIG_MSTAR_M5621 */
    _eMbxRole2Cpu[E_MBX_ROLE_FRC] = E_MBX_CPU_R2FRC; //frcr2_integration###

    _eMbxCpu2Role[E_MBX_CPU_PM] = E_MBX_ROLE_PM;
    _eMbxCpu2Role[E_MBX_CPU_R2FRC] = E_MBX_ROLE_FRC; //frcr2_integration###

    if (E_MBX_CPU_AEON == eHKCPU) {
        _eMbxRole2Cpu[E_MBX_ROLE_CP] = E_MBX_CPU_MIPS;
        _eMbxCpu2Role[E_MBX_CPU_AEON] = E_MBX_ROLE_HK;
        _eMbxCpu2Role[E_MBX_CPU_MIPS] = E_MBX_ROLE_CP;
    } else if (E_MBX_CPU_MIPS == eHKCPU) {
        _eMbxRole2Cpu[E_MBX_ROLE_CP] = E_MBX_CPU_AEON;
        _eMbxCpu2Role[E_MBX_CPU_AEON] = E_MBX_ROLE_CP;
        _eMbxCpu2Role[E_MBX_CPU_MIPS] = E_MBX_ROLE_HK;
    } else {
        return E_MBX_ERR_INVALID_PARAM;
    }

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
// Config Host CPU ID of Mailbox driver
// @param  eHostCPU    \b IN: The config Host CPU ID\n
// @return E_MBX_SUCCESS
// @return E_MBX_ERR_INVALID_PARAM
// @return E_MBX_ERR_NOT_INITIALIZED
// @return E_MBX_UNKNOW_ERROR
// @attention
// <b>[MXLIB] <em>Pls Set Config in MDrv_MBX_Init, or After MDrv_MBX_Init</em></b>
//-------------------------------------------------------------------------------------------------
static MBX_Result _MDrv_MBX_SetConfig(MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;

    //Config MBX
    if (u32TimeoutMillSecs == 0)
        return E_MBX_ERR_INVALID_PARAM;

    if ((_eMbxHostRole != eHostRole) || (_eMbxRole2Cpu[E_MBX_ROLE_HK] != eHKCPU)) {
        //Config MBX Hardware
        LOCK_RECV_HW_SEM();
        MHAL_MBX_SetConfig(eHostRole);
        UNLOCK_RECV_HW_SEM();

        mbxResult = MHAL_MBXINT_ResetHostCPU(_eMbxRole2Cpu[_eMbxHostRole], _eMbxRole2Cpu[eHostRole]);
        if (E_MBX_SUCCESS != mbxResult)
            return mbxResult;

        return _MDrv_MBX_InitConfig(eHKCPU, eHostRole, u32TimeoutMillSecs);
    }

    _u32TimeoutMillSecs = u32TimeoutMillSecs;

    return mbxResult;
}


//=============================================================================
//Util functions:
// Get System Time in MS:
MS_U32 _MDrv_MBX_GetSystemTime(void)
{
    return jiffies_to_msecs(jiffies);
}

//=============================================================================
//Mail Message handler functions:

//-------------------------------------------------------------------------------------------------
/// Send the message to MailBox
/// @param  pMsg                  \b IN: The msg for sending
/// @param  eSrcCPUID                  \b IN: the src cpu which send the mail message
/// @return E_MBX_ERR_PEER_CPU_NOTREADY:peer cpu is still fetching the mail message
/// @return E_MBX_ERR_PEER_CPU_NOT_ALIVE: fire bit is on while busy bit is off seems like peer cpu not alive
/// @return E_MBX_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em> Use spin lock to protect co-access </em></b>
//-------------------------------------------------------------------------------------------------
static MBX_Result _MDrv_MBX_SendMsg(MBX_Msg *pMsg, MBX_ROLE_ID eSrcRole)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MBXHAL_Fire_Status mbxHalFireStatus;
    MS_U32 u32WaitCnt = 0;
    MS_U32 u32TimeTicket;

#if 1 //no R2 and PM's lock due to incorrect enum usage
    if (pMsg->eRoleID == E_MBX_ROLE_FRC)
        DRV_MBX_LockSendFRC();
    else
        DRV_MBX_LockSend();
#else
    DRV_MBX_LockSend();
#endif

    LOCK_SEND_HW_SEM();
    mbxResult = MHAL_MBX_Fire(pMsg, eSrcRole);
    UNLOCK_SEND_HW_SEM();

    if (E_MBX_SUCCESS == mbxResult) {
        //fire interrupt
        MHAL_MBXINT_Fire(_eMbxRole2Cpu[pMsg->eRoleID], _eMbxRole2Cpu[eSrcRole]);
        u32TimeTicket = _MDrv_MBX_GetSystemTime();
        do {
            LOCK_SEND_HW_SEM();
            mbxResult = MHAL_MBX_GetFireStatus(eSrcRole, pMsg->eRoleID, &mbxHalFireStatus);
            UNLOCK_SEND_HW_SEM();

            if (E_MBX_SUCCESS != mbxResult)
                break;

            if (E_MBXHAL_FIRE_OVERFLOW == mbxHalFireStatus) {
                mbxResult = E_MBX_ERR_PEER_CPU_OVERFLOW;
                break;
            }

            if (E_MBXHAL_FIRE_DISABLED == mbxHalFireStatus) {
                mbxResult = E_MBX_ERR_PEER_CPU_NOTREADY;
                break;
            }

            if (E_MBXHAL_FIRE_SUCCESS == mbxHalFireStatus) {
                mbxResult = E_MBX_SUCCESS;
                break;
            }

            //check If Timeout:
            u32WaitCnt++;
            if (u32WaitCnt >= 0x10000) {
                if ((_MDrv_MBX_GetSystemTime()-u32TimeTicket) >= _u32TimeoutMillSecs) {
                    LOCK_SEND_HW_SEM();
                    MHAL_MBX_SetConfig(_eMbxHostRole);
                    UNLOCK_SEND_HW_SEM();
                    mbxResult = E_MBX_ERR_PEER_CPU_NOT_ALIVE;
                    break;
                }
                u32WaitCnt = 0;
            }
        } while (TRUE);
    }

    if (mbxResult != E_MBX_SUCCESS) {
        LOCK_SEND_HW_SEM();
        MHAL_MBX_ClearStatus(pMsg, eSrcRole);
        UNLOCK_SEND_HW_SEM();
    }

#if 1 //no R2 and PM's lock due to incorrect enum usage
    if (pMsg->eRoleID == E_MBX_ROLE_FRC)
        DRV_MBX_UnLockSendFRC();
    else
        DRV_MBX_UnLockSend();
#else
    DRV_MBX_UnLockSend();
#endif

    return mbxResult;
}

static void _MDrv_MBX_DequeueMsg(MBX_Class eTargetClass, MS_BOOL bInstantMsg, MS_U16 u16MsgIDRemove)
{
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;
    MS_U16 u16QStatus = 0;

    for (pMbxAsyncNotifier = _mbxAsyncNotifierMBX; pMbxAsyncNotifier < (_mbxAsyncNotifierMBX + (_lastNotifierIndex + 1)); pMbxAsyncNotifier++) {
        if (pMbxAsyncNotifier->u16Usage == TRUE) {
            u16QStatus = MDrv_MSGQ_GetQStatusByQID(pMbxAsyncNotifier, eTargetClass);
            if (u16QStatus != E_MSGQ_INVALID) {
                if (pMbxAsyncNotifier->bEnable == TRUE) {
                    MDrv_MSGQ_DequeueMsg(pMbxAsyncNotifier, eTargetClass, bInstantMsg, u16MsgIDRemove);
                }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------
/// Recv the message from mail message queue
/// @param  mbxAsyncNotifierID                  \b IN: Async Notifier ID
/// @param  eTargetClass                  \b IN: The target class for mail message
/// @param  pMsg                  \b OUT: fectch message
/// @param  u32Flag                  \b IN: indicate MBX_CHECK_NORMAL_MSG / MBX_CHECK_INSTANT_MSG / MBX_CHECK_NORMAL_MSG
/// @return E_MBX_ERR_INVALID_PARAM:no any flag setting in u32Flag
/// @return E_MBX_ERR_NO_MORE_MSG: no any mail message in message pool
/// @return E_MBX_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
static MBX_Result _MDrv_MBX_RecvMsg(MS_S16 mbxAsyncNotifierID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32Flag)
{
    MBX_Result mbxResult = E_MBX_ERR_INVALID_PARAM;
    MS_S16 s16MsgQIdx;
    MS_U16 u16MsgRecvID = 0;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    //DRV_MBX_LockRecv();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    if (u32Flag == MBX_CHECK_ALL_MSG_CLASS) {
        //check Instance Msg for All registered class:
        s16MsgQIdx = pMbxAsyncNotifier->s16MsgQFirst;
        if (u32Flag & MBX_CHECK_INSTANT_MSG) {
            while (IS_VALID_PTR(s16MsgQIdx)) {
                mbxResult = MDrv_MSGQ_RecvMsg(pMbxAsyncNotifier, s16MsgQIdx, pMsg, TRUE, &u16MsgRecvID);
                if (mbxResult == E_MBX_SUCCESS) {
                    _MDrv_MBX_DequeueMsg(s16MsgQIdx, TRUE, u16MsgRecvID);
                    _u16MsgIDCounter--;
                    // DRV_MBX_UnLockRecv();
                    return mbxResult;
                }
                s16MsgQIdx = MDrv_MSGQ_GetNextMsgQ(pMbxAsyncNotifier, s16MsgQIdx);
            }
        }

        //check Normal Msg for All registered class:
        s16MsgQIdx = pMbxAsyncNotifier->s16MsgQFirst;

        if (u32Flag & MBX_CHECK_NORMAL_MSG) {
            while (IS_VALID_PTR(s16MsgQIdx)) {
                mbxResult = MDrv_MSGQ_RecvMsg(pMbxAsyncNotifier, s16MsgQIdx, pMsg, FALSE, &u16MsgRecvID);
                if (mbxResult == E_MBX_SUCCESS) {
                    _MDrv_MBX_DequeueMsg(s16MsgQIdx, FALSE, u16MsgRecvID);
                    _u16MsgIDCounter--;
                    //DRV_MBX_UnLockRecv();
                    return mbxResult;
                }

                s16MsgQIdx = MDrv_MSGQ_GetNextMsgQ(pMbxAsyncNotifier, s16MsgQIdx);
            }
        }

        //DRV_MBX_UnLockRecv();
        return mbxResult;
    } else {
        if (!IS_VALID_PTR(MDrv_MSGQ_GetNotiferIDByQID(pMbxAsyncNotifier, eTargetClass))) {
            //DRV_MBX_UnLockRecv();
            return E_MBX_ERR_SLOT_NOT_OPENNED;
        }

        /*
        if(mbxAsyncNotifierID != MDrv_MSGQ_GetNotiferIDByQID(eTargetClass))
        {
            DRV_MBX_UnLockRecv();
            return E_MBX_ERR_SLOT_BUSY;
        }
        */

        if (u32Flag & MBX_CHECK_INSTANT_MSG) {
            //check Instance Msg for All registered class:
            mbxResult = MDrv_MSGQ_RecvMsg(pMbxAsyncNotifier, eTargetClass, pMsg, TRUE, &u16MsgRecvID);
            if (mbxResult == E_MBX_SUCCESS) {
                _MDrv_MBX_DequeueMsg(eTargetClass, TRUE, u16MsgRecvID);
                _u16MsgIDCounter--;
                //DRV_MBX_UnLockRecv();
                return mbxResult;
            }
         }

        if (u32Flag & MBX_CHECK_NORMAL_MSG) {
            //check Instance Msg for All registered class:
            mbxResult = MDrv_MSGQ_RecvMsg(pMbxAsyncNotifier, eTargetClass, pMsg, FALSE, &u16MsgRecvID);
            if (mbxResult == E_MBX_SUCCESS) {
                _MDrv_MBX_DequeueMsg(eTargetClass, FALSE, u16MsgRecvID);
                _u16MsgIDCounter--;
                //DRV_MBX_UnLockRecv();
                return mbxResult;
            }
        }
    }

    //DRV_MBX_UnLockRecv();
    return mbxResult;
}

//-------------------------------------------------------------------------------------------------
/// Check the message from mail message queue
/// @param  mbxAsyncNotifierID                  \b IN: Async Notifier ID
/// @param  eTargetClass                  \b IN: The target class for mail message
/// @param  pMsg                  \b OUT: fectch message
/// @param  u32Flag                  \b IN: indicate MBX_CHECK_NORMAL_MSG / MBX_CHECK_INSTANT_MSG / MBX_CHECK_NORMAL_MSG
/// @return E_MBX_ERR_INVALID_PARAM:no any flag setting in u32Flag
/// @return E_MBX_ERR_NO_MORE_MSG: no any mail message in message pool
/// @return E_MBX_SUCCESS:success
/// @attention
/// <b>[OBAMA] <em> </em></b>
//-------------------------------------------------------------------------------------------------
static MBX_Result  _MDrv_MBX_CheckMsg(MS_S16 mbxAsyncNotifierID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32Flag)
{
    MBX_Result mbxResult = E_MBX_ERR_INVALID_PARAM;
    MS_S16 s16MsgQIdx;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_LockRecv();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];

    if (u32Flag & MBX_CHECK_ALL_MSG_CLASS) {
        //check Instance Msg for All registered class:
        s16MsgQIdx = pMbxAsyncNotifier->s16MsgQFirst;
        if (u32Flag & MBX_CHECK_INSTANT_MSG) {
            while (IS_VALID_PTR(s16MsgQIdx)) {
                mbxResult = MDrv_MSGQ_CheckMsg(pMbxAsyncNotifier, s16MsgQIdx, pMsg, TRUE);
                if (mbxResult == E_MBX_SUCCESS) {
                    DRV_MBX_UnLockRecv();
                    return mbxResult;
                }
                s16MsgQIdx = MDrv_MSGQ_GetNextMsgQ(pMbxAsyncNotifier, s16MsgQIdx);
            }
        }

        //check Normal Msg for All registered class:
        s16MsgQIdx = pMbxAsyncNotifier->s16MsgQFirst;

        if (u32Flag & MBX_CHECK_NORMAL_MSG) {
            while (IS_VALID_PTR(s16MsgQIdx)) {
                mbxResult = MDrv_MSGQ_CheckMsg(pMbxAsyncNotifier, s16MsgQIdx, pMsg, FALSE);
                if(mbxResult == E_MBX_SUCCESS) {
                    DRV_MBX_UnLockRecv();
                    return mbxResult;
                }
                s16MsgQIdx = MDrv_MSGQ_GetNextMsgQ(pMbxAsyncNotifier, s16MsgQIdx);
            }
        }

        DRV_MBX_UnLockRecv();
        return mbxResult;
    }

    if (!IS_VALID_PTR(MDrv_MSGQ_GetNotiferIDByQID(pMbxAsyncNotifier, eTargetClass))) {
        DRV_MBX_UnLockRecv();
        return E_MBX_ERR_SLOT_NOT_OPENNED;
    }

    /*
    if(mbxAsyncNotifierID != MDrv_MSGQ_GetNotiferIDByQID(eTargetClass))
    {
        DRV_MBX_UnLockRecv();
        return E_MBX_ERR_SLOT_BUSY;
    }
    */

    if (u32Flag & MBX_CHECK_INSTANT_MSG) {
        //check Instance Msg for All registered class:
        mbxResult = MDrv_MSGQ_CheckMsg(pMbxAsyncNotifier, eTargetClass, pMsg, TRUE);
        if (mbxResult == E_MBX_SUCCESS) {
            DRV_MBX_UnLockRecv();
            return mbxResult;
        }
     }

    if (u32Flag & MBX_CHECK_NORMAL_MSG) {
        //check Instance Msg for All registered class:
        mbxResult = MDrv_MSGQ_CheckMsg(pMbxAsyncNotifier, eTargetClass, pMsg, FALSE);
    }

    DRV_MBX_UnLockRecv();
    return mbxResult;
}


//=============================================================================
// Mailbox Driver Function
//=============================================================================

//-------------------------------------------------------------------------------------------------
/// When Mail Arrived, the function will be called in interrupt tasklet. it will notify the UMD Driver the msg arrived.
/// @param  s32Irq                  \b IN: The Irq Number which indicated the interrupt is from which to which.
/// @return void
/// @attention
/// <b>[OBAMA] <em>get message class information from signal information. </em></b>
//-------------------------------------------------------------------------------------------------
static void _MDrv_MBX_MsgRecvCb(MS_S32 s32Irq)
{
    MBX_Msg mbxMsg;
    MBX_Result mbxResult;
    MS_BOOL bTrigger;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;
    MS_U16 u16QStatus = 0;
    bool is_class_valid = FALSE;

    struct fasync_struct  **ppTempFasync = NULL;

    //DRV_MBX_LockRecv();
    switch (s32Irq) {
        case MBX_INT_AEON2PM:
        case MBX_INT_AEON2MIPS:
#if defined(CONFIG_MSTAR_M5621)
        case MBX_INT_R2MIPS:
#else
        //case MBX_INT_R2MPIS:
#endif /* CONFIG_MSTAR_M5621 */
            mbxMsg.eRoleID = _eMbxCpu2Role[E_MBX_CPU_AEON];
            break;
        case MBX_INT_MIPS2PM:
        case MBX_INT_MIPS2AEON:
            mbxMsg.eRoleID = _eMbxCpu2Role[E_MBX_CPU_MIPS];
            break;
        case MBX_INT_PM2AEON:
        case MBX_INT_PM2MIPS:
            mbxMsg.eRoleID = _eMbxCpu2Role[E_MBX_CPU_PM];
            break;
        case MBX_INT_FRC2MIPS: //frcr2_integration###
            mbxMsg.eRoleID = _eMbxCpu2Role[E_MBX_CPU_R2FRC];
            break;
        default:
            mbxMsg.eRoleID = 2; //_eMbxCpu2Role[E_MBX_CPU_AEON]; //(### IMPORTANT ###) needs to be reviewed
            //DRV_MBX_UnLockRecv();
            //return;
    }

    LOCK_RECV_HW_SEM();
    mbxResult = MHAL_MBX_Recv(&mbxMsg, _eMbxHostRole);
    UNLOCK_RECV_HW_SEM();

    if (E_MBX_SUCCESS != mbxResult) {
        //receive message failed, just skip it.
        //DRV_MBX_UnLockRecv();
        return;
    }

    /* give an ID for this received message */
    _u16MsgIDCounter++;

    if (_u16MsgIDCounter > _infoMSGP.u16Slots) {
        /* msgpool is full */
        _u16MsgIDCounter--;
        LOCK_RECV_HW_SEM();
        MHAL_MBX_RecvEnd(mbxMsg.eRoleID, _eMbxHostRole, E_MBXHAL_RECV_DISABLED);
        UNLOCK_RECV_HW_SEM();

        return;
    }

    for (pMbxAsyncNotifier = _mbxAsyncNotifierMBX; pMbxAsyncNotifier < (_mbxAsyncNotifierMBX + (_lastNotifierIndex + 1)); pMbxAsyncNotifier++) {
        if (pMbxAsyncNotifier->u16Usage == TRUE) {
            u16QStatus = MDrv_MSGQ_GetQStatusByQID(pMbxAsyncNotifier, mbxMsg.u8MsgClass);
            if (u16QStatus != E_MSGQ_INVALID) {
                if (pMbxAsyncNotifier->bEnable == TRUE) {
                    is_class_valid = TRUE;

                    mbxResult = MDrv_MSGQ_AddMSG(pMbxAsyncNotifier, &mbxMsg, _u16MsgIDCounter); //add to msg queue, maybe failed when overflow.

                    LOCK_RECV_HW_SEM();
                    switch (mbxResult) {
                        case E_MSGPOOL_ERR_NOT_INITIALIZED:
                            MHAL_MBX_RecvEnd(mbxMsg.eRoleID, _eMbxHostRole, E_MBXHAL_RECV_DISABLED);
                            break;
                        //case E_MSGPOOL_ERR_NO_MORE_MSG:
                        case E_MSGPOOL_ERR_NO_MORE_MEMORY:
                            MHAL_MBX_RecvEnd(mbxMsg.eRoleID, _eMbxHostRole, E_MBXHAL_RECV_OVERFLOW);
                            break;
                        default:
                            MHAL_MBX_RecvEnd(mbxMsg.eRoleID, _eMbxHostRole, E_MBXHAL_RECV_SUCCESS);
                            break;
                    }
                    UNLOCK_RECV_HW_SEM();

                    ppTempFasync = (struct fasync_struct  **) &(pMbxAsyncNotifier->async_queue);

                    //DRV_MBX_UnLockRecv();
                    //notify the application:
                    bTrigger = pMbxAsyncNotifier->bTrigger[mbxMsg.u8MsgClass];

                    if (bTrigger == TRUE)
                        kill_fasync(ppTempFasync, SIGIO, POLL_IN | (mbxMsg.u8MsgClass << MBXCLASS_IN_SIGNAL_SHIFT));
                }
            }
        }
    }

    if (is_class_valid == FALSE) {
        //the slot is not registered and not enable yet, just skip the msg.
        _u16MsgIDCounter--;
        LOCK_RECV_HW_SEM();
        MHAL_MBX_RecvEnd(mbxMsg.eRoleID, _eMbxHostRole, E_MBXHAL_RECV_DISABLED);
        UNLOCK_RECV_HW_SEM();
        //DRV_MBX_UnLockRecv();
        return;
    }

    if (MDrv_MBX_MsgCbFunc) //To-do: function pointer array
        MDrv_MBX_MsgCbFunc();

    complete(&(mbx_completion[mbxMsg.u8MsgClass]));
}

/*
  Function: MDrv_MBX_Startup()
  brief: Startup Mailbox driver, including startup Msg Pool, Interrupt...
  Input: None
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NO_MORE_MEMORY-->Not enough memory
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_Startup(void)
{
    //init msg pool:
    if (E_MBX_SUCCESS != (MBX_Result)MDrv_MSGPOOL_Init())
        return E_MBX_ERR_NO_MORE_MEMORY;

    //init msg queue:
    //MDrv_MSGQ_Init();

    //init spin lock:
    //DRV_MBX_LockAsync_Init();
    DRV_MBX_LockSend_Init();
    DRV_MBX_LockSend_FRC_Init();
    DRV_MBX_LockRecv_Init();
    DRV_MBX_Lock_Init();
    DRV_PM_Lock_Init();     //  for MDrv_PM_Get_BrickTerminator_Info  (pm bank dummy register)

    //init async Notifier:
    memset((void*)_mbxAsyncNotifierMBX, 0, (sizeof(MBX_ASYNC_NOTIFIER)*MBX_ASYNCNOTIFIER_MAX));

    return E_MBX_SUCCESS;
}

/*
  Function: MDrv_MBX_Exit()
  brief: Exit Mailbox driver, including exit Msg Pool, Interrupt...
  Input: None
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NO_MORE_MEMORY-->Not enough memory
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_Exit(void)
{
    //DE-INIT MBX pool
    MDrv_MSGPOOL_DeInit();

    //DE-INIT MBX interrupt:
    MHAL_MBXINT_DeInit(_eMbxRole2Cpu[_eMbxHostRole]);

    return E_MBX_SUCCESS;
}

/*
  Function: MDrv_MBX_NotifyMsgRecCbFunc()
  brief: notify MBX receive callback function
  Input: None
  Output: None
  Return Value:
        TRUE-->Successed
        FALSE-->notify fail or repeat notify
*/
int MDrv_MBX_NotifyMsgRecCbFunc(void (*p)(void))
{
    MDrv_MBX_MsgCbFunc = p; //warning messg to avoid repeat
    return TRUE;
}

//-------------------------------------------------------------------------------------------------
/// Init Mailbox driver
/// @param  eHostCPU    \b IN: The configged CPU ID which mbx driver will run on\n
/// @return E_MBX_SUCCESS
/// @return E_MBX_ERR_NOT_INITIALIZED
/// @return E_MBX_ERR_NO_MORE_MEMORY
/// @return E_MBX_UNKNOW_ERROR
/// @attention
/// <b>[MXLIB] <em></em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MDrv_MBX_Init(MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_Init_Async(u32AsyncID, eHKCPU, eHostRole, u32TimeoutMillSecs);

    return mbxResult;
}

MBX_Result MDrv_MBX_Init_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_CPU_ID eHKCPU, MBX_ROLE_ID eHostRole, MS_U32 u32TimeoutMillSecs)
{
    int i = 0;
    MBX_Result result;
    MS_U16 idx = 0;

    DRV_MBX_LockRecv();

    if (INVALID_CPUID(eHKCPU))
        return E_MBX_ERR_INVALID_PARAM;

    if (INVALID_ROLEID(eHostRole))
        return E_MBX_ERR_INVALID_PARAM;

    if (IS_VALID_PTR(_MDrv_MBX_AllocateAsyncNotifier(u32AsyncID))) {
        idx = 0;
        if (E_MBX_ROLE_MAX ==_eMbxHostRole) {
            //Init
            if (E_MBX_SUCCESS != _MDrv_MBX_InitConfig(eHKCPU, eHostRole, u32TimeoutMillSecs)) {
                DRV_MBX_UnLockRecv();
                return E_MBX_ERR_INVALID_PARAM;
            }

            //Set mbx hardware:
            MHAL_MBX_Init(_eMbxHostRole);

            //Set mbx int hardware:
            if (E_MBX_SUCCESS != MHAL_MBXINT_Init(_eMbxRole2Cpu[eHostRole], _MDrv_MBX_MsgRecvCb)) {
                DRV_MBX_UnLockRecv();
                return E_MBX_ERR_INVALID_CPU_ID;
            }
        } else {
            result = _MDrv_MBX_SetConfig(eHKCPU, eHostRole, u32TimeoutMillSecs);
            g_u32AMBXAsyncID = u32AsyncID;
            DRV_MBX_UnLockRecv();
            return result;
        }

#if (defined(CONFIG_MSTAR_HW_SEM) && (CONFIG_MSTAR_HW_SEM == 1) && (USE_HW_SEM == 1))
        MDrv_SEM_Init();
#endif

        if (completionInit == 0) {
            for (i = 0; i<E_MBX_CLASS_MAX; i++)
                init_completion(&(mbx_completion[i]));
            completionInit = 1;
        }
        g_u32AMBXAsyncID = u32AsyncID;

        DRV_MBX_UnLockRecv();

        return E_MBX_SUCCESS;
    }

    DRV_MBX_UnLockRecv();

    return E_MBX_ERR_NO_MORE_MEMORY;
}

/*
  Function: MDrv_MBX_DeInit()
  brief: Deinitialize Mailbox driver
  Input: bForceDiscardPendingMsg--->drop un-processed messages or not
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB driver is not initialized
        MBX_ERR_HAS_MSG_PENDING-->Has unprocessed message.if bForceDiscardPendingMsg is set to TRUE, this  value never be returned
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_DeInit(MS_BOOL bForceDiscardPendingMsg)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_DeInit_Async(u32AsyncID, bForceDiscardPendingMsg);

    return mbxResult;
}

MBX_Result MDrv_MBX_DeInit_Async(TYPE_MBX_C_U64 u32AsyncID, MS_BOOL bForceDiscardPendingMsg)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_LockRecv();

    mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);

    //handle msg q DeInit:
    if (!IS_VALID_PTR(mbxAsyncNotifierID)) {
        DRV_MBX_UnLockRecv();
        return E_MBX_ERR_NOT_INITIALIZED;
    }

    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    if (!bForceDiscardPendingMsg) {
        //false
        if (!IS_VALID_PTR(pMbxAsyncNotifier->s16MsgQFirst)) {
            //no any msg queue.
            DRV_MBX_UnLockRecv();
            return E_MBX_SUCCESS;
        }

        mbxResult = (MBX_Result)MDrv_MSGQ_UnRegisterMSGQ(pMbxAsyncNotifier, pMbxAsyncNotifier->s16MsgQFirst, bForceDiscardPendingMsg);

        if(mbxResult == E_MBX_SUCCESS)
            pMbxAsyncNotifier->s16MsgQFirst = INVALID_PTR; //there is no mail message

        DRV_MBX_UnLockRecv();
        return mbxResult;
    }

    if (!IS_VALID_PTR(pMbxAsyncNotifier->s16MsgQFirst)) {
        //no any msg queue.
        _MDrv_MBX_FreeAsyncNotifierByID(mbxAsyncNotifierID);
    } else {
        mbxResult = (MBX_Result)MDrv_MSGQ_UnRegisterMSGQ(pMbxAsyncNotifier, pMbxAsyncNotifier->s16MsgQFirst, bForceDiscardPendingMsg);

        if(mbxResult == E_MBX_SUCCESS) {
            _MDrv_MBX_FreeAsyncNotifierByID(mbxAsyncNotifierID);
        } else {
            DRV_MBX_UnLockRecv();
            return mbxResult;
        }
    }

    //handle other DeInit:
    DRV_MBX_UnLockRecv();

    return mbxResult;
}

int MDrv_MBX_Suspend(MSTAR_MBX_DEV *pmbx_dev)
{
    MBX_CPU_ID eHostCPU;

    if (!pmbx_dev)
        return -1;

    eHostCPU = _eMbxRole2Cpu[_eMbxHostRole];
    MHAL_MBXINT_Suspend(eHostCPU);

    return 0;
}

int MDrv_MBX_Resume(MSTAR_MBX_DEV *pmbx_dev)
{
    MBX_CPU_ID eHostCPU;

    if (!pmbx_dev)
        return -1;

    if (_eMbxHostRole < E_MBX_ROLE_MAX)
        MHAL_MBX_SetConfig(_eMbxHostRole);

    eHostCPU = _eMbxRole2Cpu[_eMbxHostRole];
    MHAL_MBXINT_Resume(eHostCPU);

    return 0;
}

//-------------------------------------------------------------------------------------------------
/// register for real time signal notification.
/// @param  s32Fd      \b IN: s32Fd for fasync_helper
/// @param  u32AsyncID      \b IN: Async ID
/// @param  mode      \b IN: On/Off
/// @return E_MBX_ERR_INVALID_PARAM: not registered AsyncID
/// @return E_MBX_UNKNOW_ERROR: fasync register failed
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>one AsyncID, one notification queue </em></b>
MBX_Result MDrv_MBX_FASYNC(MS_S32 s32Fd, TYPE_MBX_C_U64 u32AsyncID, MS_S32 s32Mode)
{
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MS_U16 i = 0;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_INVALID_PARAM;

    DRV_MBX_Lock();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    //register the async queue:
    if (fasync_helper(s32Fd, (struct file *)u32AsyncID, s32Mode, &(pMbxAsyncNotifier->async_queue)) < 0) {
        DRV_MBX_UnLock();
        return E_MBX_UNKNOW_ERROR;
    }

    if (completionInit == 0) {
        for (i = 0; i < E_MBX_CLASS_MAX; i++)
            init_completion(&(mbx_completion[i]));
        completionInit = 1;
    }

    DRV_MBX_UnLock();

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// un-register for real time signal notification when application exit
/// @param  u32AsyncID      \b IN: Async ID
/// @return E_MBX_ERR_INVALID_PARAM: not registered AsyncID
/// @return E_MBX_UNKNOW_ERROR: fasync un-register failed
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[OBAMA] <em>one AsyncID, one notification queue </em></b>
MBX_Result MDrv_MBX_ReleaseFASYNC(TYPE_MBX_C_U64 u32AsyncID)
{
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_INVALID_PARAM;

    DRV_MBX_Lock();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
     //un register the async queue:
    if (fasync_helper(-1, (struct file *)u32AsyncID, 0, &(pMbxAsyncNotifier->async_queue)) < 0) {
        DRV_MBX_UnLock();
        return E_MBX_UNKNOW_ERROR;
    }

    DRV_MBX_UnLock();
    return E_MBX_SUCCESS;
}

/*
  Function: MDrv_MBX_RegisterMSG()
  brief: Regisiter Message queue for special class
  Input: u8Class--->target message class
         u16MsgQueueSize-->target message queue size, will be allocated in kernel space
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB Driver is not initialized
        MBX_ERR_NO_MORE_MEMORY-->Not enough memory
        MBX_ERR_SLOT_BUSY-->class has been used by other APP
        MBX_ERR_SLOT_AREADY_OPENNED-->class has been openned by this APP, you do not need to open it again
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if u8Class is to valid or u16MsgQueueSize is not larger than MAX_MBX_QUEUE_SIZE.
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_RegisterMSG(MBX_Class eMsgClass, MS_U16 u16MsgQueueSize)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;

    //DRV_MBX_Lock();
    mbxResult = MDrv_MBX_RegisterMSG_Async(u32AsyncID, eMsgClass, u16MsgQueueSize);
    //DRV_MBX_UnLock();

    return mbxResult;
}

MBX_Result MDrv_MBX_RegisterMSG_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eMsgClass, MS_U16 u16MsgQueueSize)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (INVALID_MBXCLASS(eMsgClass))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    //DRV_MBX_Lock();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    mbxResult = (MBX_Result)MDrv_MSGQ_RegisterMSG(pMbxAsyncNotifier, mbxAsyncNotifierID, pMbxAsyncNotifier->s16MsgQFirst, (MS_S16)eMsgClass, u16MsgQueueSize);

    if (mbxResult == E_MBX_SUCCESS)
        pMbxAsyncNotifier->s16MsgQFirst = (MS_S16)eMsgClass;

    //DRV_MBX_UnLock();

    return mbxResult;
}

/*
  Function: MDrv_MBX_UnRegisterMSG()
  brief: UnRegisiter Message queue for special class
  Input: u8Class--->target message class
         bForceDiscardMsgQueue-->drop un-processed messages or not
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_ERR_SLOT_BUSY-->class has been used by other APP
        MBX_ERR_SLOT_NOT_OPENNED-->this class is never regisitered
        MBX_ERR_HAS_MSG_PENDING-->Has unprocessed message.if bForceDiscardMsgQueue is set to TRUE, this  value never be returned.
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if u8Class is to valid.
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_UnRegisterMSG(MBX_Class eMsgClass, MS_BOOL bForceDiscardMsgQueue)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;

    //DRV_MBX_Lock();
    mbxResult = MDrv_MBX_UnRegisterMSG_Async(u32AsyncID, eMsgClass, bForceDiscardMsgQueue);
    //DRV_MBX_UnLock();

    return mbxResult;
}

MBX_Result MDrv_MBX_UnRegisterMSG_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eMsgClass, MS_BOOL bForceDiscardMsgQueue)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MS_S16 s16MsgQFirst;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (INVALID_MBXCLASS(eMsgClass))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    //DRV_MBX_Lock();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    s16MsgQFirst = pMbxAsyncNotifier->s16MsgQFirst;
    mbxResult = (MBX_Result)MDrv_MSGQ_UnRegisterMSG(pMbxAsyncNotifier, mbxAsyncNotifierID, &s16MsgQFirst, (MS_S16)eMsgClass, bForceDiscardMsgQueue);

    if (mbxResult == E_MBX_SUCCESS)
        pMbxAsyncNotifier->s16MsgQFirst = s16MsgQFirst;

    //DRV_MBX_UnLock();

    return mbxResult;
}

//-------------------------------------------------------------------------------------------------
/// Clear Message queue for special class
/// @param  eMsgClass \b IN: Mail Message Class, @ref MBX_Class
/// @return E_MBX_SUCCESS
/// @return E_MBX_ERR_INVALID_PARAM
/// @return E_MBX_ERR_NOT_INITIALIZED
/// @return E_MBX_ERR_SLOT_NOT_OPENNED
/// @return E_MBX_UNKNOW_ERROR
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MDrv_MBX_ClearMSG(MBX_Class eMsgClass)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_ClearMSG_Async(u32AsyncID, eMsgClass);

    return mbxResult;
}

MBX_Result MDrv_MBX_ClearMSG_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eMsgClass)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (INVALID_MBXCLASS(eMsgClass))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    DRV_MBX_LockRecv();

    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    mbxResult = (MBX_Result)MDrv_MSGQ_ClearMSG(pMbxAsyncNotifier, (MS_S16)eMsgClass);

    DRV_MBX_UnLockRecv();

    return mbxResult;
}

/*
  Function: MDrv_MBX_GetMsgQueueStatus()
  brief: Check the message queue status of a special message class
  Input: targetClass--->target class
  Output: pMsgQueueStatus--> return the queue status
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if targetClass is valid is pMsgQueueStatus is not NULL.
        MBX_ERR_SLOT_BUSY-->class has been used by other APP
        MBX_ERR_SLOT_NOT_OPENNED-->this class is never regisitered
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_GetMsgQueueStatus(MBX_Class eTargetClass, MBX_MSGQ_Status *pMsgQueueStatus)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_GetMsgQueueStatus_Async(u32AsyncID, eTargetClass, pMsgQueueStatus);

    return mbxResult;
}

MBX_Result MDrv_MBX_GetMsgQueueStatus_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eTargetClass, MBX_MSGQ_Status *pMsgQueueStatus)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (INVALID_MBXCLASS(eTargetClass))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    DRV_MBX_LockRecv();

    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    mbxResult = MDrv_MSGQ_GetMsgQStatus(pMbxAsyncNotifier, (MS_S16)eTargetClass, pMsgQueueStatus);

    DRV_MBX_UnLockRecv();

    return mbxResult;
}

/*
  Function: MDrv_MBX_SendMsg()
  brief: send message
  Input: pMsg--->message to send
         bWaitPeerIdle-->wait Peer CPU or not
         u32WaitMillSecs--> if bWaitPeerIdle, indentify the timeout millsecons before return
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if pMsg content is valid or not
        MBX_ERR_SLOT_NOT_OPENNED-->this class is never regisitered
        MBX_ERR_INVALID_CPU_ID-->target cpu defined in pMsg is invalid.
        MBX_ERR_PEER_CPU_BUSY-->peer CPU is still busy
        MBX_ERR_PEER_CPU_NOT_ALIVE-->peer CPU seems not alive
        MBX_ERR_TIME_OUT-->timeout if bWaitPeerIdle is set to TRUE
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_SendMsg(MBX_Msg *pMsg)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;
    MS_BOOL bTrigger;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    /* use fixed trigger value in kernel mode */
    bTrigger = MBX_KERNEL_TRIGGER;
    mbxResult = MDrv_MBX_SendMsg_Async(u32AsyncID, pMsg, bTrigger);

    return mbxResult;
}

MBX_Result MDrv_MBX_SendMsg_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Msg *pMsg, MS_BOOL bTrigger)
{
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    //parameter check:
    if (NULL == pMsg)
        return E_MBX_ERR_INVALID_PARAM;

    if ((pMsg->u8ParameterCount < 0) || (pMsg->u8ParameterCount > MAX_MBX_PARAM_SIZE))
        return E_MBX_ERR_INVALID_PARAM;

    if (INVALID_ROLEID(pMsg->eRoleID))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    if (!(INVALID_MBXCLASS(pMsg->u8MsgClass))) {
        pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
        pMbxAsyncNotifier->bTrigger[pMsg->u8MsgClass] = bTrigger;
    }

    return  _MDrv_MBX_SendMsg(pMsg, _eMbxHostRole);
}


/*
  Function: MDrv_MBX_SendMsgLoopback()
  brief: send message loop back
  Input: pMsg--->message to send
         bWaitPeerIdle-->wait Peer CPU or not
         u32WaitMillSecs--> if bWaitPeerIdle, indentify the timeout millsecons before return
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if pMsg content is valid or not
        MBX_ERR_SLOT_NOT_OPENNED-->this class is never regisitered
        MBX_ERR_INVALID_CPU_ID-->target cpu defined in pMsg is invalid.
        MBX_ERR_PEER_CPU_BUSY-->peer CPU is still busy
        MBX_ERR_PEER_CPU_NOT_ALIVE-->peer CPU seems not alive
        MBX_ERR_TIME_OUT-->timeout if bWaitPeerIdle is set to TRUE
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_SendMsgLoopback(MBX_Msg *pMsg, MBX_ROLE_ID eSrcRoleId)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_SendMsgLoopback_Async(u32AsyncID, pMsg, eSrcRoleId);

    return mbxResult;
}

MBX_Result MDrv_MBX_SendMsgLoopback_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Msg *pMsg, MBX_ROLE_ID eSrcRoleId)
{
    //parameter check:
    if (NULL == pMsg)
        return E_MBX_ERR_INVALID_PARAM;

    if ((pMsg->u8ParameterCount < 0) || (pMsg->u8ParameterCount > MAX_MBX_PARAM_SIZE))
        return E_MBX_ERR_INVALID_PARAM;

    if (INVALID_ROLEID(pMsg->eRoleID))
        return E_MBX_ERR_INVALID_PARAM;

    if (INVALID_ROLEID(eSrcRoleId))
        return E_MBX_ERR_INVALID_PARAM;

    return _MDrv_MBX_SendMsg(pMsg, eSrcRoleId);
}


/*
  Function: MDrv_MBX_RecvMsg()
  brief: recv message
  Input: targetClass--->target class
         bWaitPeerIdle-->wait Peer CPU or not
         u32Flag-->recv flag, has the following possible meaning bits
                       MBX_CHECK_ALL_MSG_CLASS -->recv any message class, this means, the targetClass is useless if this bit is set.
                       MBX_CHECK_INSTANT_MSG-->check INSTANT message
                       MBX_CHECK_NORMAL_MSG->Check Normal message
                       MBX_CHECK_BLOCK_RECV-->block this function call until time out if no message available
  Output: pMsg  message received
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if targetClass, pMsg and u32Flag is valid or not.
        MBX_ERR_SLOT_NOT_OPENNED-->this class is never regisitered
        MBX_ERR_NO_MORE_MSG-->No message is available
        MBX_ERR_TIME_OUT-->timeout if MBX_CHECK_BLOCK_RECV is set to int u32Flag
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_RecvMsg(MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;
    MS_BOOL bTrigger;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    /* use fixed trigger value in kernel mode */
    bTrigger = MBX_KERNEL_TRIGGER;
    mbxResult = MDrv_MBX_RecvMsg_Async(u32AsyncID, eTargetClass, pMsg, u32WaitMillSecs, u32Flag, bTrigger);

    return mbxResult;
}

MBX_Result MDrv_MBX_RecvMsg_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag, MS_BOOL bTrigger)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;
    long ret;

    if (INVALID_MBXCLASS(eTargetClass))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    if (!_bCalledFromDriver) {
        if ((u32Flag != MBX_CHECK_INSTANT_MSG) && (u32Flag != MBX_CHECK_NORMAL_MSG))
            return E_MBX_ERR_INVALID_PARAM;

        _bCalledFromDriver = FALSE;
    }

    ret = wait_for_completion_interruptible_timeout(&(mbx_completion[eTargetClass]), msecs_to_jiffies((u32WaitMillSecs)));
    if (ret == 0) {
        return E_MBX_ERR_TIME_OUT;
    } else if (ret < 0) {
        return E_MBX_UNKNOW_ERROR;
    }

    DRV_MBX_LockRecv();
    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    pMbxAsyncNotifier->bTrigger[eTargetClass] = bTrigger;
    mbxResult = _MDrv_MBX_RecvMsg(mbxAsyncNotifierID, eTargetClass, pMsg, u32Flag);
    DRV_MBX_UnLockRecv();

    return mbxResult;
}

/*
  Function: MDrv_MBX_CheckMsg()
  brief: Check message
  Input: targetClass--->target class
         bWaitPeerIdle-->wait Peer CPU or not
         u32Flag-->recv flag, has the following possible meaning bits
                       MBX_CHECK_ALL_MSG_CLASS -->recv any message class, this means, the targetClass is useless if this bit is set.
                       MBX_CHECK_INSTANT_MSG-->check INSTANT message
                       MBX_CHECK_NORMAL_MSG->Check Normal message
                       MBX_CHECK_BLOCK_RECV-->block this function call until time out if no message available
  Output: pMsg  message received (The function only check the message in the Queue, but no remove message from Q after checking.)
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_ERR_INVALID_PARAM-->Invalid parameter, please check if targetClass, pMsg and u32Flag is valid or not.
        MBX_ERR_SLOT_NOT_OPENNED-->this class is never regisitered
        MBX_ERR_NO_MORE_MSG-->No message is available
        MBX_ERR_TIME_OUT-->timeout if MBX_CHECK_BLOCK_RECV is set to int u32Flag
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_CheckMsg(MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_CheckMsg_Async(u32AsyncID, eTargetClass, pMsg, u32WaitMillSecs, u32Flag);

    return mbxResult;
}

MBX_Result MDrv_MBX_CheckMsg_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MS_U32 u32Idx;

    if (INVALID_MBXCLASS(eTargetClass))
        return E_MBX_ERR_INVALID_PARAM;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return E_MBX_ERR_NOT_INITIALIZED;

    if ((u32Flag & MBX_CHECK_ALL_MSG_CLASS) == 0)
        return E_MBX_ERR_INVALID_PARAM;

    mbxResult = _MDrv_MBX_CheckMsg(mbxAsyncNotifierID, eTargetClass, pMsg, u32Flag);
    if ((mbxResult == E_MBX_ERR_NO_MORE_MSG) && (u32Flag & MBX_CHECK_BLOCK_RECV)) {
        for (u32Idx = 0; u32Idx < u32WaitMillSecs; u32Idx++) {
            mbxResult = _MDrv_MBX_CheckMsg(mbxAsyncNotifierID, eTargetClass, pMsg, u32Flag);

            if (mbxResult != E_MBX_ERR_NO_MORE_MSG)
                break;

            msleep(1);
        }

        if (mbxResult == E_MBX_ERR_NO_MORE_MSG)
            mbxResult = E_MBX_ERR_TIME_OUT;
    }

    return mbxResult;
}
/*
  Function: MDrv_MBX_RemoveLatestMsg()
  brief: Remove the latest received message, this function is used right after check message within message received interrupt handler
  Input: None
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        E_MBX_ERR_NO_MORE_MSG-->no message to be removed
*/
MBX_Result MDrv_MBX_RemoveLatestMsg(TYPE_MBX_C_U64 u32AsyncID)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    DRV_MBX_LockRecv();

    if (!IS_VALID_PTR(mbxAsyncNotifierID)) {
        DRV_MBX_UnLockRecv();
        return E_MBX_ERR_NOT_INITIALIZED;
    }

    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];
    mbxResult = MDrv_MSGQ_RemoveLatestMsg(pMbxAsyncNotifier);

    DRV_MBX_UnLockRecv();

    return mbxResult;
}

//The u32AsyncID is useless since SetInformation could be called before Init.
MBX_Result MDrv_MBX_SetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_SetInformation_Async(u32AsyncID, eTargetRole, pU8Info, u8Size);

    return mbxResult;
}

MBX_Result MDrv_MBX_SetInformation_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size)
{
    if (INVALID_ROLEID(eTargetRole))
        return E_MBX_ERR_INVALID_PARAM;

    return MHAL_MBX_SetInformation(eTargetRole, pU8Info, u8Size);
}

//The u32AsyncID is useless since SetInformation could be called before Init.
MBX_Result MDrv_MBX_GetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_GetInformation_Async(u32AsyncID, eTargetRole, pU8Info, u8Size);

    return mbxResult;
}

MBX_Result MDrv_MBX_GetInformation_Async(TYPE_MBX_C_U64 u32AsyncID, MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size)
{
    if (INVALID_ROLEID(eTargetRole))
        return E_MBX_ERR_INVALID_PARAM;

    return MHAL_MBX_GetInformation(eTargetRole, pU8Info, u8Size);
}

/*
  Function: MDrv_MBX_Enable()
  brief: Enable Message Queue to recv message from peer CPUs, Enable receiving regisited messages in kernel
  Input: bEnable-->Enable or not
  Output: None
  Return Value:
        MBX_SUCCESS-->Successed
        MBX_ERR_NOT_INITIALIZED-->MB drv is not initialized
        MBX_UNKNOW_ERROR-->Other undefined errors
*/
MBX_Result MDrv_MBX_Enable(MS_BOOL bEnable)
{
    MBX_Result mbxResult = E_MBX_SUCCESS;
    TYPE_MBX_C_U64 u32AsyncID;

    /* use fixed fd in kernel mode */
    u32AsyncID = MBX_KERNEL_FD;
    mbxResult = MDrv_MBX_Enable_Async(u32AsyncID, bEnable);

    return mbxResult;
}

MBX_Result MDrv_MBX_Enable_Async(TYPE_MBX_C_U64 u32AsyncID, MS_BOOL bEnable)
{
    MS_S16 mbxAsyncNotifierID;
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);

    DRV_MBX_Lock();
    if (!IS_VALID_PTR(mbxAsyncNotifierID)) {
        DRV_MBX_UnLock();
        return E_MBX_ERR_NOT_INITIALIZED;
    }

    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];

    pMbxAsyncNotifier->bEnable = bEnable;

    DRV_MBX_UnLock();

    return E_MBX_SUCCESS;
}

TYPE_MBX_C_U64 MDrv_MBX_GetAsyncID(void)
{
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    //DRV_MBX_Lock();

    for (pMbxAsyncNotifier = _mbxAsyncNotifierMBX; pMbxAsyncNotifier < _mbxAsyncNotifierMBX + MBX_ASYNCNOTIFIER_MAX; pMbxAsyncNotifier++) {
        if (pMbxAsyncNotifier->u16Usage == TRUE) {
            //DRV_MBX_UnLock();

            return pMbxAsyncNotifier->u32AsyncID;
        }
    }

    //DRV_MBX_UnLock();

    return 0;
}
EXPORT_SYMBOL(MDrv_MBX_GetAsyncID);

MS_BOOL MDrv_MBX_GetEnableStatus(TYPE_MBX_C_U64 u32AsyncID)
{
    MS_S16 mbxAsyncNotifierID = _MDrv_MBX_GetAsyncNotifierIDByAsyncID(u32AsyncID);
    MBX_ASYNC_NOTIFIER *pMbxAsyncNotifier;

    if (!IS_VALID_PTR(mbxAsyncNotifierID))
        return FALSE;

    pMbxAsyncNotifier = &_mbxAsyncNotifierMBX[mbxAsyncNotifierID];

    return pMbxAsyncNotifier->bEnable;
}

void MDrv_MBX_NotifyPMtoSetPowerdown(void)
{
    //write MB RIU to notify 51 to poweroff mips
    MS_U16 grp;

    grp = MHAL_MBX_RegGroup(E_MBX_ROLE_HK,E_MBX_ROLE_PM);
    if (grp == 0xFF)
        return;

    LOCK_SEND_HW_SEM();

#ifdef CONFIG_MSTAR_PM_WDT
    if (kernel_level(__LINE__, __FUNCTION__)) { /* debug mode */
        REG8_MBX_GROUP(grp, 0x0) = 0xDD;
        REG8_MBX_GROUP(grp, 0x1) = 0xDD;
    } else /* release mode */
#endif
    {
        REG8_MBX_GROUP(grp, 0x0) = 0x99;
        REG8_MBX_GROUP(grp, 0x1) = 0x88;
    }

    REG8_MBX_GROUP(grp, 0x2) = 0x77;
    REG8_MBX_GROUP(grp, 0x3) = 0x66;
    REG8_MBX_GROUP(grp, 0x4) = 0x55;
    REG8_MBX_GROUP(grp, 0x5) = 0x44;
    REG8_MBX_GROUP(grp, 0x6) = 0x33;
    REG8_MBX_GROUP(grp, 0x7) = 0x22;
    REG8_MBX_GROUP(grp, 0x8) = 0x11;
    REG8_MBX_GROUP(grp, 0x9) = 0x00;
    REG8_MBX_GROUP(grp, 0xA) = 0xFF;
    REG8_MBX_GROUP(grp, 0xB) = 0xEE;
    REG8_MBX_GROUP(grp, 0xC) = 0xDD;
    REG8_MBX_GROUP(grp, 0xD) = 0xCC;
    REG8_MBX_GROUP(grp, 0xE) = 0xBB;
    REG8_MBX_GROUP(grp, 0xF) = 0xAA;

    UNLOCK_SEND_HW_SEM();
}
EXPORT_SYMBOL(MDrv_MBX_NotifyPMtoSetPowerdown);

void MDrv_MBX_NotifyPMtoSetPowerOff(void)
{
    //write MB RIU to notify 51 to poweroff mips
    MS_U16 grp;

    grp = MHAL_MBX_RegGroup(E_MBX_ROLE_HK,E_MBX_ROLE_PM);
    if (grp == 0xFF)
        return;

    LOCK_SEND_HW_SEM();
#if (defined CONFIG_MP_PLATFORM_ARM_64bit_PORTING)||(defined CONFIG_MP_PLATFORM_ARM_32bit_PORTING)
    extern uint32_t isPSCI;
    if ((TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_OPTEE) && (isPSCI == PSCI_RET_SUCCESS))
#else
    if (TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_OPTEE)
#endif
    {
        printk(" PMtoSetPowerOff OPTEE flow\n");
    } else {
        printk(" PMtoSetPowerOff Not OPTEE flow(TYPE:%d)\n", TEEINFO_TYPTE);
#ifdef CONFIG_MSTAR_PM_WDT
        if (kernel_level(__LINE__, __FUNCTION__)) { /* debug mode */
            REG8_MBX_GROUP(grp, 0x0) = 0xDD;
            REG8_MBX_GROUP(grp, 0x1) = 0xDD;
        } else /* release mode */
#endif
        {
            REG8_MBX_GROUP(grp, 0x0) = 0x99;
            REG8_MBX_GROUP(grp, 0x1) = 0x88;
        }

        REG8_MBX_GROUP(grp, 0x2) = 0x77;
        REG8_MBX_GROUP(grp, 0x3) = 0x66;
        REG8_MBX_GROUP(grp, 0x4) = 0x55;
        REG8_MBX_GROUP(grp, 0x5) = 0x44;
        REG8_MBX_GROUP(grp, 0x6) = 0x33;
        REG8_MBX_GROUP(grp, 0x7) = 0x22;
        REG8_MBX_GROUP(grp, 0x8) = 0x11;
        REG8_MBX_GROUP(grp, 0x9) = 0x00;
        REG8_MBX_GROUP(grp, 0xA) = 0xFF;
        REG8_MBX_GROUP(grp, 0xB) = 0xEE;
        REG8_MBX_GROUP(grp, 0xC) = 0xDD;
        REG8_MBX_GROUP(grp, 0xD) = 0xCC;
        REG8_MBX_GROUP(grp, 0xE) = 0xBB;
        REG8_MBX_GROUP(grp, 0xF) = 0xAA;
    }

    UNLOCK_SEND_HW_SEM();
}
EXPORT_SYMBOL(MDrv_MBX_NotifyPMtoSetPowerOff);

void  MDrv_MBX_NotifyPMPassword(unsigned char passwd[16])
{
    MS_U16 grp;

    grp = MHAL_MBX_RegGroup(E_MBX_ROLE_HK,E_MBX_ROLE_PM);
    if (grp == 0xFF)
        return;

    LOCK_SEND_HW_SEM();
    REG8_MBX_GROUP(grp, 0x0) = passwd[0];
    REG8_MBX_GROUP(grp, 0x1) = passwd[1];
    REG8_MBX_GROUP(grp, 0x2) = passwd[2];
    REG8_MBX_GROUP(grp, 0x3) = passwd[3];
    REG8_MBX_GROUP(grp, 0x4) = passwd[4];
    REG8_MBX_GROUP(grp, 0x5) = passwd[5];
    REG8_MBX_GROUP(grp, 0x6) = passwd[6];
    REG8_MBX_GROUP(grp, 0x7) = passwd[7];
    REG8_MBX_GROUP(grp, 0x8) = passwd[8];
    REG8_MBX_GROUP(grp, 0x9) = passwd[9];
    REG8_MBX_GROUP(grp, 0xA) = passwd[10];
    REG8_MBX_GROUP(grp, 0xB) = passwd[11];
    REG8_MBX_GROUP(grp, 0xC) = passwd[12];
    REG8_MBX_GROUP(grp, 0xD) = passwd[13];
    REG8_MBX_GROUP(grp, 0xE) = passwd[14];
    REG8_MBX_GROUP(grp, 0xF) = passwd[15];
    UNLOCK_SEND_HW_SEM();
}
EXPORT_SYMBOL(MDrv_MBX_NotifyPMPassword);

MS_U8 MDrv_PM_Get_BrickTerminator_Info(void)
{
    MS_U8 u8Info;
    DRV_PM_Lock();
    u8Info = MHAL_PM_Get_BrickTerminator_Info();
    DRV_PM_UnLock();
    return u8Info;
}
EXPORT_SYMBOL(MDrv_PM_Get_BrickTerminator_Info);

void MDrv_PM_Set_BrickTerminator_Info(MS_U8 u8Value)
{
    DRV_PM_Lock();
    MHAL_PM_Set_BrickTerminator_Info(u8Value);
    DRV_PM_UnLock();
}
EXPORT_SYMBOL(MDrv_PM_Set_BrickTerminator_Info);

/* MDrv API sync with utpa */
MBX_Result MDrv_MBX_RegisterMSGWithCallBack(MBX_Class eMsgClass, MS_U16 u16MsgQueueSize, MBX_MAIL_ARRIVE_NOTIFY_FUNC notifier)
{
    return E_MBX_SUCCESS;
}

void MDrv_MBX_SetDbgLevel(MS_U8 u8DbgLevel)
{
}

void MDrv_MBX_SetCallDrvFlag(MS_BOOL bEnable)
{
}

MS_BOOL MDrv_MBX_GetCallDrvFlag(void)
{
    return TRUE;
}

EXPORT_SYMBOL(MDrv_MBX_Init);
EXPORT_SYMBOL(MDrv_MBX_Init_Async);
EXPORT_SYMBOL(MDrv_MBX_DeInit);
EXPORT_SYMBOL(MDrv_MBX_DeInit_Async);
EXPORT_SYMBOL(MDrv_MBX_RegisterMSG);
EXPORT_SYMBOL(MDrv_MBX_RegisterMSG_Async);
EXPORT_SYMBOL(MDrv_MBX_ClearMSG);
EXPORT_SYMBOL(MDrv_MBX_ClearMSG_Async);
EXPORT_SYMBOL(MDrv_MBX_SendMsg);
EXPORT_SYMBOL(MDrv_MBX_SendMsg_Async);
EXPORT_SYMBOL(MDrv_MBX_SendMsgLoopback);
EXPORT_SYMBOL(MDrv_MBX_SendMsgLoopback_Async);
EXPORT_SYMBOL(MDrv_MBX_RecvMsg);
EXPORT_SYMBOL(MDrv_MBX_RecvMsg_Async);
EXPORT_SYMBOL(MDrv_MBX_CheckMsg);
EXPORT_SYMBOL(MDrv_MBX_CheckMsg_Async);
EXPORT_SYMBOL(MDrv_MBX_GetMsgQueueStatus);
EXPORT_SYMBOL(MDrv_MBX_GetMsgQueueStatus_Async);
EXPORT_SYMBOL(MDrv_MBX_RemoveLatestMsg);
EXPORT_SYMBOL(MDrv_MBX_SetInformation);
EXPORT_SYMBOL(MDrv_MBX_SetInformation_Async);
EXPORT_SYMBOL(MDrv_MBX_GetInformation);
EXPORT_SYMBOL(MDrv_MBX_GetInformation_Async);
EXPORT_SYMBOL(MDrv_MBX_Enable);
EXPORT_SYMBOL(MDrv_MBX_Enable_Async);
EXPORT_SYMBOL(MDrv_MBX_GetEnableStatus);
EXPORT_SYMBOL(MDrv_MBX_NotifyMsgRecCbFunc);
EXPORT_SYMBOL(MDrv_MBX_UnRegisterMSG);
EXPORT_SYMBOL(MDrv_MBX_UnRegisterMSG_Async);
EXPORT_SYMBOL(MDrv_MBX_RegisterMSGWithCallBack);
EXPORT_SYMBOL(MDrv_MBX_SetDbgLevel);
EXPORT_SYMBOL(MDrv_MBX_SetCallDrvFlag);
EXPORT_SYMBOL(MDrv_MBX_GetCallDrvFlag);

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static ssize_t mdb_mbx_node_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char tmpString[20] = {};
    MS_U32 i, j;

    if (!count)
    {
        printk("count is 0\n");
        return 0;
    }

    if (count > (sizeof(tmpString) - 1))
    {
        printk("input is too big\n");
        return -EINVAL;
    }

    if (copy_from_user(tmpString, buf, count))
    {
        printk("copy_from_user failed\n");
        return -EFAULT;
    }

    if (!strncmp(tmpString, "help", 4))
    {
        printk("\n--------- MStar MBX Help ---------\n");
        printk("  Get MBX Information:\n");
        printk("    cat /proc/utopia_mdb/mbx\n");
        printk("  Get the MBX Meanings of Return Values:\n");
        printk("    echo return > /proc/utopia_mdb/mbx\n");
        printk("  MBX Reigster Dump:\n");
        printk("    echo register > /proc/utopia_mdb/mbx\n");
    }
    else if (!strncmp(tmpString, "return", 6))
    {
        printk("\n--------------- MStar MBX Return Values ---------------\n");
        printk(" %d  : Success Call\n", E_MBX_SUCCESS);
        printk(" %d  : Error: Not Initialized\n", E_MBX_ERR_NOT_INITIALIZED);
        printk(" %d  : Error: No more Memory, Queue Memory Issue\n", E_MBX_ERR_NO_MORE_MEMORY);
        printk(" %d  : Error: class has been used by other APP\n", E_MBX_ERR_SLOT_BUSY);
        printk(" %d  : Error: class has been openned by this APP, you do not need to open it again\n", E_MBX_ERR_SLOT_AREADY_OPENNED);
        printk(" %d  : Error: class not registered yet\n", E_MBX_ERR_SLOT_NOT_OPENNED);
        printk(" %d  : Error: unknow cpu id\n", E_MBX_ERR_INVALID_CPU_ID);
        printk(" %d  : Error: invalid parameter\n", E_MBX_ERR_INVALID_PARAM);
        printk(" %d  : Error: peer cpu is peek Mail from Hardware, you can not send mail now\n", E_MBX_ERR_PEER_CPU_BUSY);
        printk(" %d  : Error: peer cpu do not alive, Mail never peek out, Need check peer cpu is alive or not\n", E_MBX_ERR_PEER_CPU_NOT_ALIVE);
        printk(" %d : Error: peer cpu not initialized yet, not ready to receive mail message\n", E_MBX_ERR_PEER_CPU_NOTREADY);
        printk(" %d : Error: peer cpu the dedicated class Overflow!\n", E_MBX_ERR_PEER_CPU_OVERFLOW);
        printk(" %d : Error: the mail message has been fetched yet, there has no message in hardware\n", E_MBX_ERR_MSG_ALREADY_FETCHED);
        printk(" %d : Error: time out with dedicated request\n", E_MBX_ERR_TIME_OUT);
        printk(" %d : Error: no mail message in message queue\n", E_MBX_ERR_NO_MORE_MSG);
        printk(" %d : Error: has mail message in queue when un-register mail class or DeInit MailBox Driver\n", E_MBX_ERR_HAS_MSG_PENDING);
        printk(" %d : Error: not implemente yet for request\n", E_MBX_ERR_NOT_IMPLEMENTED);
    }
    else if (!strncmp(tmpString, "register", 8))
    {
        DRV_MBX_Lock();
        printk("\n-------------- MStar MBX Register --------------\n");
        printk("         00|01|02|03|04|05|06|07|08|09|0A|0B|0C|0D|0E|0F\n");
#ifdef REG_MBX_GROUP0
        //printk(" Group0:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group0: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP0, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP1
        //printk(" Group1:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group1: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP1, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP2
        //printk(" Group2:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group2: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP2, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP3
        //printk(" Group3:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group3: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP3, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP4
        //printk(" Group4:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group4: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP4, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP5
        //printk(" Group5:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group5: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP5, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP6
        //printk(" Group6:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group6: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP6, 15));
        //}
        //printk("\n");
#endif
#ifdef REG_MBX_GROUP7
        //printk(" Group7:");
        //for (i = 0; i < 16; i++)
        //{
            printk(" Group7: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 0),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 1),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 2),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 3),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 4),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 5),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 6),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 7),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 8),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 9),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 10),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 11),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 12),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 13),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 14),
                   REG8_MBX_GROUP(REG_MBX_GROUP7, 15));
        //}
        //printk("\n");
#endif
        DRV_MBX_UnLock();
    }
    else if (!strncmp(tmpString, "async", 5))
    {
        for (i = 0; i < MBX_ASYNCNOTIFIER_MAX; i++)
        {
            if (_mbxAsyncNotifierMBX[i].u32AsyncID != 0)
            {
                printk(KERN_ERR "\n MBX _mbxAsyncNotifierMBX[%u]: u32AsyncID = 0x%llx, s16MsgQFirst = 0x%x, u16Usage = 0x%x, bEnable = 0x%x\n",
                       i, (u64) _mbxAsyncNotifierMBX[i].u32AsyncID, _mbxAsyncNotifierMBX[i].s16MsgQFirst, _mbxAsyncNotifierMBX[i].u16Usage, _mbxAsyncNotifierMBX[i].bEnable);
            }
        }
    }
    else if (!strncmp(tmpString, "class", 5))
    {
        for (i = 0; i < MBX_ASYNCNOTIFIER_MAX; i++)
        {
            for (j = 0; j < E_MBX_CLASS_MAX; j++)
            {
                if (_mbxAsyncNotifierMBX[i].msgQmgr[j].u16MsgQStatus != E_MSGQ_INVALID)
                {
                    printk(KERN_ERR "\n MBX _mbxAsyncNotifierMBX[%d].msgQmgr[%d]: s16MsgFirst = 0x%x, s16MsgEnd = 0x%x, s16InstantMsgFirst = 0x%x, s16InstantMsgEnd = 0x%x, u16MsgNum = 0x%x, u16InstantMsgNum = 0x%x, u16MsgQStatus = 0x%x, u16MsgQSize = 0x%x, s16MsgQNotifierID = 0x%x, s16NextMsgQ = 0x%x\n",
                           i, j, _mbxAsyncNotifierMBX[i].msgQmgr[j].s16MsgFirst, _mbxAsyncNotifierMBX[i].msgQmgr[j].s16MsgEnd, _mbxAsyncNotifierMBX[i].msgQmgr[j].s16InstantMsgFirst, _mbxAsyncNotifierMBX[i].msgQmgr[j].s16InstantMsgEnd, _mbxAsyncNotifierMBX[i].msgQmgr[j].u16MsgNum, _mbxAsyncNotifierMBX[i].msgQmgr[j].u16InstantMsgNum, _mbxAsyncNotifierMBX[i].msgQmgr[j].u16MsgQStatus, _mbxAsyncNotifierMBX[i].msgQmgr[j].u16MsgQSize, _mbxAsyncNotifierMBX[i].msgQmgr[j].s16MsgQNotifierID, _mbxAsyncNotifierMBX[i].msgQmgr[j].s16NextMsgQ);
                }
            }
        }
    }
    else if (!strncmp(tmpString, "latest", 6))
    {
        printk(KERN_ERR "\n MBX _infoLatestMSGP: bInstantMsg = 0x%x, s16MsgQIdx = 0x%x, s16MsgSlotIdx = 0x%x\n",
               _infoLatestMSGP.bInstantMsg, _infoLatestMSGP.s16MsgQIdx, _infoLatestMSGP.s16MsgSlotIdx);
    }
    else if (!strncmp(tmpString, "pool", 4))
    {
        printk(KERN_ERR "\n MBX _infoMSGP: u16Slots = 0x%x, u16FreeSlots = 0x%x, u16RegistedSlots = 0x%x\n",
               _infoMSGP.u16Slots, _infoMSGP.u16FreeSlots, _infoMSGP.u16RegistedSlots);
        for (i = 0; i < _infoMSGP.u16Slots; i++)
        {
            if (_infoMSGP.pMsgPool[i].u16Usage != FALSE)
            {
                printk(KERN_ERR "\n MBX _infoMSGP.pMsgPool[%d]: s16Next = 0x%x, u16Usage = 0x%x, u16MsgID=0x%x, _u16MsgIDCounter=0x%x, msgclass=0x%x\n",
                       i, _infoMSGP.pMsgPool[i].s16Next, _infoMSGP.pMsgPool[i].u16Usage, _infoMSGP.pMsgPool[i].u16MsgID, _u16MsgIDCounter, _infoMSGP.pMsgPool[i].mbxMsg.u8MsgClass);
            }
        }
    }
    else if (!strncmp(tmpString, "complete", 8))
    {
        for (i = 0; i < E_MBX_CLASS_MAX; i++)
        {
            printk(KERN_ERR "\n MBX mbx_completion[%d].done = %d\n", i, mbx_completion[i].done);
        }
    }


    return count;
}

static int mdb_mbx_node_show(struct seq_file *m, void *v)
{
    MS_U32 i;

    seq_printf(m, "\n--------- MStar MBX Info ---------\n");

    seq_printf(m, " MStar MBX Debug Usage:\n");
    seq_printf(m, "  echo help > /proc/utopia_mdb/mbx\n\n");

    seq_printf(m, " MStar MBX Return Values:\n");
    seq_printf(m, "  %d  : Success Call\n", E_MBX_SUCCESS);
    seq_printf(m, "  %d  : Error: Not Initialized\n", E_MBX_ERR_NOT_INITIALIZED);
    seq_printf(m, "  %d  : Error: No more Memory, Queue Memory Issue\n", E_MBX_ERR_NO_MORE_MEMORY);
    seq_printf(m, "  %d  : Error: class has been used by other APP\n", E_MBX_ERR_SLOT_BUSY);
    seq_printf(m, "  %d  : Error: class has been openned by this APP, you do not need to open it again\n", E_MBX_ERR_SLOT_AREADY_OPENNED);
    seq_printf(m, "  %d  : Error: class not registered yet\n", E_MBX_ERR_SLOT_NOT_OPENNED);
    seq_printf(m, "  %d  : Error: unknow cpu id\n", E_MBX_ERR_INVALID_CPU_ID);
    seq_printf(m, "  %d  : Error: invalid parameter\n", E_MBX_ERR_INVALID_PARAM);
    seq_printf(m, "  %d  : Error: peer cpu is peek Mail from Hardware, you can not send mail now\n", E_MBX_ERR_PEER_CPU_BUSY);
    seq_printf(m, "  %d  : Error: peer cpu do not alive, Mail never peek out, Need check peer cpu is alive or not\n", E_MBX_ERR_PEER_CPU_NOT_ALIVE);
    seq_printf(m, "  %d : Error: peer cpu not initialized yet, not ready to receive mail message\n", E_MBX_ERR_PEER_CPU_NOTREADY);
    seq_printf(m, "  %d : Error: peer cpu the dedicated class Overflow!\n", E_MBX_ERR_PEER_CPU_OVERFLOW);
    seq_printf(m, "  %d : Error: the mail message has been fetched yet, there has no message in hardware\n", E_MBX_ERR_MSG_ALREADY_FETCHED);
    seq_printf(m, "  %d : Error: time out with dedicated request\n", E_MBX_ERR_TIME_OUT);
    seq_printf(m, "  %d : Error: no mail message in message queue\n", E_MBX_ERR_NO_MORE_MSG);
    seq_printf(m, "  %d : Error: has mail message in queue when un-register mail class or DeInit MailBox Driver\n", E_MBX_ERR_HAS_MSG_PENDING);
    seq_printf(m, "  %d : Error: not implemente yet for request\n", E_MBX_ERR_NOT_IMPLEMENTED);

    DRV_MBX_Lock();
    seq_printf(m, "\n MStar MBX Register:\n");
    seq_printf(m, "         00|01|02|03|04|05|06|07|08|09|0A|0B|0C|0D|0E|0F\n");
#ifdef REG_MBX_GROUP0
    seq_printf(m, " Group0:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP0, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP1
    seq_printf(m, " Group1:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP1, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP2
    seq_printf(m, " Group2:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP2, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP3
    seq_printf(m, " Group3:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP3, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP4
    seq_printf(m, " Group4:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP4, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP5
    seq_printf(m, " Group5:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP5, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP6
    seq_printf(m, " Group6:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP6, i));
    }
    seq_printf(m, "\n");
#endif
#ifdef REG_MBX_GROUP7
    seq_printf(m, " Group7:");
    for (i = 0; i < 16; i++)
    {
        seq_printf(m, " %02X", REG8_MBX_GROUP(REG_MBX_GROUP7, i));
    }
    seq_printf(m, "\n");
#endif
    DRV_MBX_UnLock();

    return 0;
}

static int mdb_mbx_node_open(struct inode *inode, struct file *file)
{
    return single_open(file, mdb_mbx_node_show, NULL);
}
#endif
