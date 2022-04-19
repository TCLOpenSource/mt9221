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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/bug.h>
#include <linux/kthread.h>
#include <linux/mutex.h>

#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"
#include "mdrv_mbx_io.h"
#include "mapi_mbx.h"

#ifdef CONFIG_MSTAR_PM_WDT
#include <chip_arch.h>
#endif


typedef enum _Dynamic_MBX_Index
{
    E_Dynamic_GenerateClass = 0,
    E_Dynamic_QueryCalsss = 1,
} Dynamic_MBX_Index;

extern TYPE_MBX_C_U64 MDrv_MBX_GetAsyncID(void);
extern void _MDrv_MBXIO_IOC_Lock(void);
extern void _MDrv_MBXIO_IOC_UnLock(void);


static MS_BOOL _MApi_MBX_IsMsgQEmpty(MBX_Class eTargetClass)
{
    TYPE_MBX_C_U64 u32ASyncID;
    MBX_MSGQ_Status eMsgQStatus;
    MBX_Result ret = E_MBX_SUCCESS;

    _MDrv_MBXIO_IOC_Lock();
    u32ASyncID = MDrv_MBX_GetAsyncID();
    ret = MDrv_MBX_GetMsgQueueStatus_Async(u32ASyncID, eTargetClass, &eMsgQStatus);
    _MDrv_MBXIO_IOC_UnLock();

    if (ret != E_MBX_SUCCESS)
        return TRUE;

    return (((eMsgQStatus.u32NormalMsgCount+eMsgQStatus.u32InstantMsgCount) > 0) ? FALSE:TRUE);
}

#define Wait_MsgQ_Empty(eTargetClass) { while (_MApi_MBX_IsMsgQEmpty(eTargetClass) == FALSE); }

static MBX_Result MApi_MBX_RegisterQueryMBX(void)
{
    MBX_Result ret = E_MBX_SUCCESS;

    Wait_MsgQ_Empty(E_MBX_CLASS_QUERY_CLASS);
    ret = MApi_MBX_RegisterMSG(E_MBX_CLASS_QUERY_CLASS, 10);

    if ((ret != E_MBX_SUCCESS) && (ret != E_MBX_ERR_SLOT_AREADY_OPENNED))
        return ret;

    Wait_MsgQ_Empty(E_MBX_CLASS_SECUREBOOT_WAIT);
    ret = MApi_MBX_RegisterMSG(E_MBX_CLASS_SECUREBOOT_WAIT, 10);

    if ((ret != E_MBX_SUCCESS) && (ret != E_MBX_ERR_SLOT_AREADY_OPENNED))
        return ret;

    return ret;
}

MBX_Result MApi_MBX_SendMsg(MBX_Msg *pMsg)
{
    TYPE_MBX_C_U64 u32ASyncID;
    MBX_Result ret;

    _MDrv_MBXIO_IOC_Lock();
    u32ASyncID = MDrv_MBX_GetAsyncID();
    ret = MDrv_MBX_SendMsg_Async(u32ASyncID, pMsg, MBX_KERNEL_TRIGGER);
    _MDrv_MBXIO_IOC_UnLock();

    return ret;
}

MBX_Result MApi_MBX_RecvMsg(MBX_Class eTargetClass, MBX_Msg *pMsg, MS_U32 u32WaitMillSecs, MS_U32 u32Flag)
{
    TYPE_MBX_C_U64 u32ASyncID;
    MBX_Result ret;

    _MDrv_MBXIO_IOC_Lock();
    u32ASyncID = MDrv_MBX_GetAsyncID();
    ret = MDrv_MBX_RecvMsg_Async(u32ASyncID, eTargetClass, pMsg, u32WaitMillSecs, u32Flag, MBX_KERNEL_TRIGGER);
    _MDrv_MBXIO_IOC_UnLock();

    return ret;
}

MBX_Result MApi_MBX_UnRegisterMSG(MBX_Class eMsgClass, MS_U16 u16MsgQueueSize)
{
    TYPE_MBX_C_U64 u32ASyncID;
    MBX_Result ret;

    _MDrv_MBXIO_IOC_Lock();
    u32ASyncID = MDrv_MBX_GetAsyncID();
    ret = MDrv_MBX_UnRegisterMSG_Async(u32ASyncID, eMsgClass, u16MsgQueueSize);
    _MDrv_MBXIO_IOC_UnLock();

    return ret;
}

MBX_Result MApi_MBX_RegisterMSG(MBX_Class eMsgClass, MS_U16 u16MsgQueueSize)
{
    TYPE_MBX_C_U64 u32ASyncID;
    MBX_Result ret;

    _MDrv_MBXIO_IOC_Lock();
    u32ASyncID = MDrv_MBX_GetAsyncID();
    ret = MDrv_MBX_RegisterMSG_Async(u32ASyncID, eMsgClass, u16MsgQueueSize);
    _MDrv_MBXIO_IOC_UnLock();

    return ret;
}

static int str_len(char *s)
{
    int len = 0;

    while (*s != 0) {
        len++;
        s++;
    }

    return len;
}

static MBX_Result _MApi_SendDynamicRequest(MBX_ROLE_ID eRoleID, char *name)
{
    MS_U16 i;
    MBX_Msg smsg;
    MBX_Result ret;

    smsg.eRoleID = eRoleID;
    smsg.eMsgType = E_MBX_MSG_TYPE_INSTANT;
    smsg.u8Ctrl = 0;
    smsg.u8MsgClass = E_MBX_CLASS_QUERY_CLASS;
    smsg.u8Index = E_Dynamic_QueryCalsss;
    smsg.u8ParameterCount = str_len(name) + 2; // string end token

    for (i = 0; i < (str_len(name)); i++)
        smsg.u8Parameters[i+1] = (MS_U8) name[i];

    smsg.u8Parameters[(str_len(name))+1] = '\0';

    ret = MApi_MBX_SendMsg(&smsg);

    return ret;
}

static MBX_Result _MApi_RecvDynamicRequest(MS_U8 *pmbx_class)
{
#define MAX_RETRY_RECV (5000)
    MBX_Result ret;
    MBX_Msg rmsg;
    MS_U16 u16Retry = 0;

    do {
        ret = MApi_MBX_RecvMsg(E_MBX_CLASS_SECUREBOOT_WAIT, &rmsg, 1000, MBX_CHECK_INSTANT_MSG);
        u16Retry++;
    } while ((E_MBX_SUCCESS != ret) && (u16Retry < MAX_RETRY_RECV));

    if (ret == E_MBX_SUCCESS) {
        *pmbx_class = rmsg.u8Parameters[0];
    }

    return ret;
}

MBX_Result MApi_MBX_QueryDynamicClass(MBX_ROLE_ID eRoleID, char *name, MS_U8 *pmbx_class)
{
    MBX_Result ret;

    ret = MApi_MBX_RegisterQueryMBX();
    if ((ret != E_MBX_SUCCESS) && (ret != E_MBX_ERR_SLOT_AREADY_OPENNED)) {
        printk(KERN_ERR "[ERROR] %s %d, ret=%d\n", __FUNCTION__, __LINE__, ret);
        return ret;
    }

    ret = _MApi_SendDynamicRequest(eRoleID, name);
    if (ret != E_MBX_SUCCESS) {
        printk(KERN_ERR "[ERROR] %s %d, ret=%d\n", __FUNCTION__, __LINE__, ret);
        return ret;
    }

    ret = _MApi_RecvDynamicRequest(pmbx_class);
    if (ret != E_MBX_SUCCESS) {
        printk(KERN_ERR "[ERROR] %s %d, ret=%d\n", __FUNCTION__, __LINE__, ret);
        return ret;
    }

    return ret;
}

#ifdef CONFIG_MP_R2_STR_ENABLE
#define TIME_OUT 2000
#define MBX_MAX_RETRY_NUM 10
void MApi_MBX_NotifyTeetoSuspend(unsigned long u32TEESTRBOOTFLAG)
{
    const char* pcStrAPP="commonTA";
    MBX_Msg stMsg;
    MBX_Result ret = E_MBX_UNKNOW_ERROR;
    //MBX_Msg stRecvMsg;
    MS_U8 ClassNum = E_MBX_CLASS_PM_WAIT;
    MS_U8 u8RetryCount = 0;

    //TEE STR Handle
    //Send Command : STR_Suspend to TEE

    //1. Register a mailbox class for TEE STR
#if defined(CONFIG_MSTAR_M5621)
    ret = MApi_MBX_QueryDynamicClass(E_MBX_ROLE_CP, pcStrAPP, &ClassNum);
#else
    ret = MApi_MBX_QueryDynamicClass(E_MBX_ROLE_AEON, pcStrAPP, &ClassNum);
#endif /* CONFIG_MSTAR_M5621 */
    if (ret != E_MBX_SUCCESS)
        return FALSE;

    printk("PM: ClassNum = %d ,ret = %d\n", ClassNum, ret);

    //2. Send STR_Suspend to TEE
    memset(&stMsg, 0, sizeof(MBX_Msg));
#if defined(CONFIG_MSTAR_M5621)
    stMsg.eRoleID = E_MBX_ROLE_CP;
#else
    stMsg.eRoleID = E_MBX_ROLE_AEON;
#endif /* CONFIG_MSTAR_M5621 */
    stMsg.eMsgType = E_MBX_MSG_TYPE_INSTANT;
    stMsg.u8Ctrl = 0x0;
    stMsg.u8MsgClass = ClassNum;
    stMsg.u8Index = 0x2c; //EN_CTA_CMD_STR_SUSPEND_NO_ACK
    stMsg.u8ParameterCount = 0xa;
    stMsg.u8Parameters[0] = (unsigned char) (u32TEESTRBOOTFLAG & 0xFF);
    stMsg.u8Parameters[1] = (unsigned char) (u32TEESTRBOOTFLAG >> 8);
    stMsg.u8Parameters[2] = (unsigned char) (u32TEESTRBOOTFLAG >> 16);
    stMsg.u8Parameters[3] = (unsigned char) (u32TEESTRBOOTFLAG >> 24);
    stMsg.u8Parameters[4] = 0;//(unsigned char) (u32TEESTRBOOTFLAG >> 32);
    stMsg.u8Parameters[5] = 0;//(unsigned char) (u32TEESTRBOOTFLAG >> 40);
    stMsg.u8Parameters[6] = 0;//(unsigned char) (u32TEESTRBOOTFLAG >> 48);
    stMsg.u8Parameters[7] = 0;//(unsigned char) (u32TEESTRBOOTFLAG >> 56);
    stMsg.u8Parameters[8] = 0x4;
    stMsg.u8Parameters[9] = 0x0;
    stMsg.u8S0 = 0x0;
    stMsg.u8S1 = 0x0;

    do {
        ret = MApi_MBX_SendMsg(&stMsg);
        printk(KERN_INFO "PM: Send MBX to TEE for STR_Suspend  ... Done \nPM: -> id=%d, type=%d, class=%d, index=%d,  return value = %d\n",
                stMsg.eRoleID, stMsg.eMsgType, stMsg.u8MsgClass, stMsg.u8Index, ret);

        if (ret != E_MBX_SUCCESS) {
            if (u8RetryCount < MBX_MAX_RETRY_NUM) {
                u8RetryCount++;
            } else {
                printk(KERN_INFO "PM: Send msg fail, reach max retry count=%d\n",u8RetryCount);
                break;
            }
        }
    } while (ret != E_MBX_SUCCESS);

#if defined(CONFIG_MSTAR_M5621)
    //MApi_MBX_UnRegisterMSG(ClassNum,10);
#else
    MApi_MBX_UnRegisterMSG(ClassNum,10);
#endif /* CONFIG_MSTAR_M5621 */
    MApi_MBX_UnRegisterMSG(E_MBX_CLASS_SECUREBOOT_WAIT,10);
    MApi_MBX_UnRegisterMSG(E_MBX_CLASS_QUERY_CLASS,10);

#if 0
    //Receive ACK from TEE
    unsigned int err = 0;
    memset(&stRecvMsg, 0, sizeof(MBX_Msg));
    stRecvMsg.u8MsgClass = ClassNum;
    while(E_MBX_SUCCESS != ret) {
        ret = MApi_MBX_RecvMsg(stRecvMsg.u8MsgClass, &stRecvMsg, TIME_OUT, MBX_CHECK_INSTANT_MSG);
        err++;
        if(err == MBX_MAX_RETRY_NUM) {
            printk(KERN_INFO "mbx retry count reached, assume mailbox failed\n");
            return FALSE;
        }
        printk(KERN_INFO "Receive msg fail, retry count=%d\n",err);
    }
#endif
}
#endif /* CONFIG_MP_R2_STR_ENABLE */

EXPORT_SYMBOL(MApi_MBX_SendMsg);
EXPORT_SYMBOL(MApi_MBX_RecvMsg);
EXPORT_SYMBOL(MApi_MBX_RegisterMSG);
EXPORT_SYMBOL(MApi_MBX_UnRegisterMSG);
EXPORT_SYMBOL(MApi_MBX_QueryDynamicClass);
#ifdef CONFIG_MP_R2_STR_ENABLE
EXPORT_SYMBOL(MApi_MBX_NotifyTeetoSuspend);
#endif /* CONFIG_MP_R2_STR_ENABLE */
