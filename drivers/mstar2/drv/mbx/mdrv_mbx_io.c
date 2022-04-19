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
/// file    mdrv_mbx_io.c
/// @brief  MS MailBox ioctrol driver
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//=============================================================================
// Include Files
//=============================================================================
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/string.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include "mdrv_types.h"
#include "mdrv_system.h"
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/of.h>
#endif

//drver header files
#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"
#include "mdrv_mbx_io.h"
#include "mst_devid.h"
#if defined(CONFIG_COMPAT)
#include "mdrv_mbx_compat_st.h"
#include "mdrv_mbx_compat.h"
#include "mdrv_mbx_compat_io.h"
#endif

//=============================================================================
// Local Defines
//=============================================================================
#define     MDRV_MBX_DEVICE_COUNT           1
#define     MDRV_MBX_NAME                           "MSMAILBOX"

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
#define     MBXIO_DEBUG_ENABLE  //
#ifdef      MBXIO_DEBUG_ENABLE
#define     MBXIO_KDBG(_fmt, _args...)        printk(KERN_WARNING _fmt, ## _args)
#define     MBXIO_ASSERT(_con)   do {\
                                                            if (!(_con)) {\
                                                                printk(KERN_CRIT "BUG at %s:%d assert(%s)\n",\
                                                                                                    __FILE__, __LINE__, #_con);\
                                                                BUG();\
                                                            }\
                                                          } while (0)
#else
#define     MBXIO_KDBG(fmt, args...)
#define     MBXIO_ASSERT(arg)
#endif

#if 0 //unnecessary mutex lock here
static struct mutex _mutexMBXIOCTL;
#define DRV_MBX_LockIOCTL_Init()     mutex_init(&_mutexMBXIOCTL)
#define DRV_MBX_LockIOCTL()   mutex_lock(&_mutexMBXIOCTL)
#define DRV_MBX_UnLockIOCTL()   mutex_unlock(&_mutexMBXIOCTL)
#else
static DEFINE_SPINLOCK(lock);
#define DRV_MBX_LockIOCTL_Init()
#define DRV_MBX_LockIOCTL()
#define DRV_MBX_UnLockIOCTL()
#endif

//--------------------------------------------------------------------------------------------------
// IOCtrl functions declaration
//--------------------------------------------------------------------------------------------------
static int _MDrv_MBXIO_Open (struct inode *inode, struct file *filp);
static int _MDrv_MBXIO_Release(struct inode *inode, struct file *filp);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long _MDrv_MBXIO_IOCtl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static long _MDrv_MBXIO_IOCtl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static int _MDrv_MBXIO_FASYNC(int fd, struct file *filp, int mode);
MSYSTEM_STATIC int _MDrv_MBXIO_ModuleInit(void);
MSYSTEM_STATIC void _MDrv_MBXIO_ModuleExit(void);

ssize_t _MDrv_MBXIO_Read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos);
ssize_t _MDrv_MBXIO_Write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos);
#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_MBXIO_IOCtl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

//=============================================================================
// Local Variables: Device handler
//=============================================================================
typedef struct
{
    int s32Major;
    int s32Minor;
    int refCnt;
    struct cdev cdev;
    struct file_operations fops;
    struct mutex mutex;
}MBX_DEV;

static MBX_DEV _devMBX =
{
    .s32Major = MDRV_MAJOR_MSMAILBOX,
    .s32Minor = MDRV_MINOR_MSMAILBOX,
    .refCnt = 0,
    .cdev =
    {
        .kobj = {.name= MDRV_NAME_MSMAILBOX, },
        .owner = THIS_MODULE,
    },
    .fops =
    {
        .open = _MDrv_MBXIO_Open,
        .read = _MDrv_MBXIO_Read,
        .write = _MDrv_MBXIO_Write,
        .release = _MDrv_MBXIO_Release,
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
        .unlocked_ioctl = _MDrv_MBXIO_IOCtl,
	    #if defined(CONFIG_COMPAT)
        .compat_ioctl = _Compat_MDrv_MBXIO_IOCtl,
        #endif
        #else
        .ioctl = _MDrv_MBXIO_IOCtl,
        #endif
        .fasync = _MDrv_MBXIO_FASYNC,
    }
};

static struct class *mbx_class;

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static struct proc_dir_entry *mdb_mbx_proc_entry = NULL;
unsigned int mdb_mbx_node_flag;
extern const struct file_operations mdb_mbx_node_operations;
#endif

//=============================================================================
// Local Function Prototypes
//=============================================================================
static int _MDrv_MBXIO_IOC_Init(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_DeInit(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_RegisterMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_UnRegisterMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_ClearMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_SendMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_SendMsgLoopback(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_RecvMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_CheckMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_RemoveLatestMsg(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_GetMsgQStatus(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_MbxEnable(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_SetInformation(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_GetInformation(struct file *filp, unsigned long arg);
static int _MDrv_MBXIO_IOC_GetDrvStatus(struct file *filp, unsigned long arg);

static int _MDrv_PMIO_IOC_Get_BrickTerminator_Info(struct file *filp, unsigned long arg);
static int _MDrv_PMIO_IOC_Set_BrickTerminator_Info(struct file *filp, unsigned long arg);
//-------------------------------------------------------------------------------------------------
// IOCtrl Driver functions
//-------------------------------------------------------------------------------------------------

void _MDrv_MBXIO_IOC_Lock(void)
{
    DRV_MBX_LockIOCTL();
}

void _MDrv_MBXIO_IOC_UnLock(void)
{
    DRV_MBX_UnLockIOCTL();
}


int _MDrv_MBXIO_IOC_Init(struct file *filp, unsigned long arg)
{
    MS_MBX_INIT_INFO stInitInfo;

    if(copy_from_user(&stInitInfo, (MS_MBX_INIT_INFO __user *)arg, sizeof(MS_MBX_INIT_INFO)))
        return EFAULT;

    stInitInfo.mbxResult = MDrv_MBX_Init_Async((TYPE_MBX_C_U64)filp, stInitInfo.eHKCPU, stInitInfo.eHostRole, stInitInfo.u32TimeoutMillSecs);

    if(copy_to_user((( MS_MBX_INIT_INFO  __user *)arg), &stInitInfo, sizeof(MS_MBX_INIT_INFO)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_DeInit(struct file *filp, unsigned long arg)
{
    MS_MBX_SET_BINFO stSetBInfo;

    if(copy_from_user(&stSetBInfo, (MS_MBX_SET_BINFO  __user *)arg, sizeof(MS_MBX_SET_BINFO)))
        return EFAULT;

    stSetBInfo.mbxResult = MDrv_MBX_DeInit_Async((TYPE_MBX_C_U64)filp, stSetBInfo.bInfo);

     if(copy_to_user((( MS_MBX_SET_BINFO  __user *)arg), &stSetBInfo, sizeof(MS_MBX_SET_BINFO)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_RegisterMsg(struct file *filp, unsigned long arg)
{
    MS_MBX_REGISTER_MSG stRegisterMsg;

    if(copy_from_user(&stRegisterMsg, (MS_MBX_REGISTER_MSG __user *)arg, sizeof(MS_MBX_REGISTER_MSG)))
        return EFAULT;

    stRegisterMsg.mbxResult = MDrv_MBX_RegisterMSG_Async((TYPE_MBX_C_U64)filp, stRegisterMsg.eMsgClass, stRegisterMsg.u16MsgQueueSize);

    if(copy_to_user((( MS_MBX_REGISTER_MSG  __user *)arg), &stRegisterMsg, sizeof(MS_MBX_REGISTER_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_UnRegisterMsg(struct file *filp, unsigned long arg)
{
    MS_MBX_UNREGISTER_MSG stUnRegisterMsg;

    if(copy_from_user(&stUnRegisterMsg, (MS_MBX_UNREGISTER_MSG __user *)arg, sizeof(MS_MBX_UNREGISTER_MSG)))
        return EFAULT;

    stUnRegisterMsg.mbxResult = MDrv_MBX_UnRegisterMSG_Async((TYPE_MBX_C_U64)filp, stUnRegisterMsg.eMsgClass, stUnRegisterMsg.bForceDiscardMsgQueue);

    if(copy_to_user((( MS_MBX_UNREGISTER_MSG  __user *)arg), &stUnRegisterMsg, sizeof(MS_MBX_UNREGISTER_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_ClearMsg(struct file *filp, unsigned long arg)
{
    MS_MBX_CLEAR_MSG stClearMsg;

    if(copy_from_user(&stClearMsg, (MS_MBX_CLEAR_MSG __user *)arg, sizeof(MS_MBX_CLEAR_MSG)))
        return EFAULT;

    stClearMsg.mbxResult = MDrv_MBX_ClearMSG_Async((TYPE_MBX_C_U64)filp, stClearMsg.eMsgClass);

    if(copy_to_user((( MS_MBX_CLEAR_MSG  __user *)arg), &stClearMsg, sizeof(MS_MBX_CLEAR_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_SendMsg(struct file *filp, unsigned long arg)
{
    MBX_Msg mbxMsg;
    MS_MBX_SEND_MSG  stMBXSend;

    if(copy_from_user(&stMBXSend, (MS_MBX_SEND_MSG __user *)arg, sizeof(MS_MBX_SEND_MSG)))
        return EFAULT;

    if (stMBXSend.pMsg == NULL)
        return EFAULT;

    if(copy_from_user(&mbxMsg, stMBXSend.pMsg ,sizeof(MBX_Msg)))
        return EFAULT;

    stMBXSend.mbxResult = MDrv_MBX_SendMsg_Async((TYPE_MBX_C_U64)filp, &mbxMsg, TRUE);

    if(copy_to_user((( MS_MBX_SEND_MSG  __user *)arg), &stMBXSend, sizeof(MS_MBX_SEND_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_SendMsgLoopback(struct file *filp, unsigned long arg)
{
    MS_MBX_SEND_MSG stSendMsg;
    MBX_Msg mbxMsg;

    if(copy_from_user(&stSendMsg, (MS_MBX_SEND_MSG __user *)arg, sizeof(MS_MBX_SEND_MSG)))
        return EFAULT;

    if (stSendMsg.pMsg == NULL)
        return EFAULT;

    if(copy_from_user(&mbxMsg, stSendMsg.pMsg , sizeof(MBX_Msg)))
        return EFAULT;

    stSendMsg.mbxResult = MDrv_MBX_SendMsgLoopback_Async((TYPE_MBX_C_U64)filp, &mbxMsg, stSendMsg.mbxResult);

    if(copy_to_user((( MS_MBX_SEND_MSG  __user *)arg), &stSendMsg, sizeof(MS_MBX_SEND_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_RecvMsg(struct file *filp, unsigned long arg)
{
    MS_MBX_RECV_MSG stRecvMsg;
    MBX_Msg mbxMsg;

    if(copy_from_user(&stRecvMsg, (MS_MBX_RECV_MSG __user *)arg, sizeof(MS_MBX_RECV_MSG)))
        return EFAULT;

    stRecvMsg.mbxResult = MDrv_MBX_RecvMsg_Async((TYPE_MBX_C_U64)filp, stRecvMsg.eTargetClass, &mbxMsg, stRecvMsg.u32WaitMillSecs, stRecvMsg.u32Flag, TRUE);

    if (stRecvMsg.pMsg == NULL)
        return EFAULT;

    if(copy_to_user((TYPE_MBX_C_U64 *)stRecvMsg.pMsg  , &mbxMsg, sizeof(MBX_Msg)))
        return EFAULT;

    if(copy_to_user((( MS_MBX_RECV_MSG  __user *)arg), &stRecvMsg, sizeof(MS_MBX_RECV_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_CheckMsg(struct file *filp, unsigned long arg)
{
    MS_MBX_RECV_MSG stRecvMsg;
    MBX_Msg mbxMsg;

    if(copy_from_user(&stRecvMsg, (MS_MBX_RECV_MSG __user *)arg, sizeof(MS_MBX_RECV_MSG)))
        return EFAULT;

    stRecvMsg.mbxResult = MDrv_MBX_CheckMsg_Async((TYPE_MBX_C_U64)filp, stRecvMsg.eTargetClass, &mbxMsg, stRecvMsg.u32WaitMillSecs, stRecvMsg.u32Flag);

    if (stRecvMsg.pMsg == NULL)
        return EFAULT;

    if(copy_to_user(stRecvMsg.pMsg , &mbxMsg, sizeof(MBX_Msg)))
        return EFAULT;

    if(copy_to_user((( MS_MBX_RECV_MSG  __user *)arg), &stRecvMsg, sizeof(MS_MBX_RECV_MSG)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_RemoveLatestMsg(struct file *filp, unsigned long arg)
{
    MS_MBX_SET_BINFO stSetBInfo;

    if(copy_from_user(&stSetBInfo, (MS_MBX_SET_BINFO  __user *)arg, sizeof(MS_MBX_SET_BINFO)))
        return EFAULT;

    stSetBInfo.mbxResult = MDrv_MBX_RemoveLatestMsg((TYPE_MBX_C_U64)filp);

    if(copy_to_user((( MS_MBX_SET_BINFO  __user *)arg), &stSetBInfo, sizeof(MS_MBX_SET_BINFO)))
        return EFAULT;

    return 0;
}


int _MDrv_MBXIO_IOC_GetMsgQStatus(struct file *filp, unsigned long arg)
{
    MS_MBX_GET_MSGQSTATUS stGetMsgQStatus;
    MBX_MSGQ_Status mbxMsgQueueStatus;


    if(copy_from_user(&stGetMsgQStatus, (MS_MBX_GET_MSGQSTATUS  __user *)arg, sizeof(MS_MBX_GET_MSGQSTATUS)))
        return EFAULT;

    stGetMsgQStatus.mbxResult = MDrv_MBX_GetMsgQueueStatus_Async((TYPE_MBX_C_U64)filp, stGetMsgQStatus.eTargetClass, &mbxMsgQueueStatus);

    if (stGetMsgQStatus.pMsgQueueStatus == NULL)
        return EFAULT;

    if(copy_to_user(stGetMsgQStatus.pMsgQueueStatus , &mbxMsgQueueStatus, sizeof(MBX_MSGQ_Status)))
        return EFAULT;

   if(copy_to_user((( MS_MBX_GET_MSGQSTATUS  __user *)arg), &stGetMsgQStatus, sizeof(MS_MBX_GET_MSGQSTATUS)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_MbxEnable(struct file *filp, unsigned long arg)
{
    MS_MBX_SET_BINFO stSetBInfo;

    if(copy_from_user(&stSetBInfo, (MS_MBX_SET_BINFO  __user *)arg, sizeof(MS_MBX_SET_BINFO)))
        return EFAULT;

    stSetBInfo.mbxResult = MDrv_MBX_Enable_Async((TYPE_MBX_C_U64)filp, stSetBInfo.bInfo);

    if(copy_to_user((( MS_MBX_SET_BINFO  __user *)arg), &stSetBInfo, sizeof(MS_MBX_SET_BINFO)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_SetInformation(struct file *filp, unsigned long arg)
{
    MS_MBX_CPROSYNC_INFORMATION stSetInfo;
    MS_U8 u8Info[MAX_MBX_INFORMATION_SIZE]={0};


    if(copy_from_user(&stSetInfo, (MS_MBX_CPROSYNC_INFORMATION __user *)arg, sizeof(MS_MBX_CPROSYNC_INFORMATION)))
        return EFAULT;

    if(stSetInfo.u8Size > MAX_MBX_INFORMATION_SIZE)
        return EFAULT;

    if (stSetInfo.pU8Info == NULL)
        return EFAULT;

    if(copy_from_user(u8Info, stSetInfo.pU8Info , stSetInfo.u8Size))
        return EFAULT;

    stSetInfo.mbxResult = MDrv_MBX_SetInformation_Async((TYPE_MBX_C_U64)filp, stSetInfo.eTargetRole, u8Info, stSetInfo.u8Size);

    if(copy_to_user((( MS_MBX_CPROSYNC_INFORMATION  __user *)arg), &stSetInfo, sizeof(MS_MBX_CPROSYNC_INFORMATION)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_GetInformation(struct file *filp, unsigned long arg)
{
    MS_MBX_CPROSYNC_INFORMATION stGetInfo;
    MS_U8 u8Info[MAX_MBX_INFORMATION_SIZE];

    if(copy_from_user(&stGetInfo, (MS_MBX_CPROSYNC_INFORMATION __user *)arg, sizeof(MS_MBX_CPROSYNC_INFORMATION)))
        return EFAULT;

    if (stGetInfo.u8Size > MAX_MBX_INFORMATION_SIZE)
        return EFAULT;

    stGetInfo.mbxResult = MDrv_MBX_GetInformation_Async((TYPE_MBX_C_U64)filp, stGetInfo.eTargetRole, u8Info, stGetInfo.u8Size);

    if (stGetInfo.pU8Info == NULL)
        return EFAULT;

    if(copy_to_user(stGetInfo.pU8Info, u8Info, stGetInfo.u8Size))
        return EFAULT;

    if(copy_to_user((( MS_MBX_CPROSYNC_INFORMATION  __user *)arg), &stGetInfo, sizeof(MS_MBX_CPROSYNC_INFORMATION)))
        return EFAULT;

    return 0;
}

int _MDrv_MBXIO_IOC_GetDrvStatus(struct file *filp, unsigned long arg)
{
    MS_MBX_GET_DRVSTATUS stGetDrvStatus;

    stGetDrvStatus.bEnabled = MDrv_MBX_GetEnableStatus((TYPE_MBX_C_U64)filp);

    mutex_lock(&_devMBX.mutex);
    stGetDrvStatus.s32RefCnt = _devMBX.refCnt;
    mutex_unlock(&_devMBX.mutex);

    if(copy_to_user((( MS_MBX_GET_DRVSTATUS __user *)arg), &stGetDrvStatus, sizeof(MS_MBX_GET_DRVSTATUS)))
        return EFAULT;

    return 0;
}

int _MDrv_PMIO_IOC_Get_BrickTerminator_Info(struct file *filp, unsigned long arg)
{
	MS_PM_BRICK_TERMINATOR_INFO	stReadByte;

    stReadByte.u8Value = MDrv_PM_Get_BrickTerminator_Info();

    if (copy_to_user((U8 *) arg, &stReadByte, sizeof(MS_PM_BRICK_TERMINATOR_INFO)))
        return EFAULT;

    return 0;
}

int _MDrv_PMIO_IOC_Set_BrickTerminator_Info(struct file *filp, unsigned long arg)
{
    MS_PM_BRICK_TERMINATOR_INFO	stWriteByte;

	if (copy_from_user(&stWriteByte, (MS_PM_BRICK_TERMINATOR_INFO __user *) arg, sizeof(MS_PM_BRICK_TERMINATOR_INFO)))
        return EFAULT;

    MDrv_PM_Set_BrickTerminator_Info(stWriteByte.u8Value);
    return 0;
}
//-------------------------------------------------------------------------------------------------
// IOCtrl Driver interface functions
//-------------------------------------------------------------------------------------------------
int _MDrv_MBXIO_Open(struct inode *inode, struct file *filp)
{
    mutex_lock(&_devMBX.mutex);

    MBXIO_KDBG("--------->MBX DRIVER OPEN: cnt=%d\n", _devMBX.refCnt);

    _devMBX.refCnt++;
    mutex_unlock(&_devMBX.mutex);

    return 0;
}

ssize_t _MDrv_MBXIO_Read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
    return 0;
}

ssize_t _MDrv_MBXIO_Write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
    return 0;
}

int _MDrv_MBXIO_Release(struct inode *inode, struct file *filp)
{
    mutex_lock(&_devMBX.mutex);

    MBXIO_KDBG("<---------MBX DRIVER CLOSE: cnt=%d\n", _devMBX.refCnt);

    MBXIO_ASSERT(_devMBX.refCnt>0);

    //de-register from signal queue:
    MDrv_MBX_ReleaseFASYNC((TYPE_MBX_C_U64)filp); //call driver interface to un-register signal queue.

    //force to release since application is exit!
    MDrv_MBX_DeInit_Async((TYPE_MBX_C_U64)filp, TRUE);

    if(_devMBX.refCnt > 0)
    {
        _devMBX.refCnt = _devMBX.refCnt -1;
    }
    mutex_unlock(&_devMBX.mutex);

    return 0;
}

int _MDrv_MBXIO_FASYNC(int fd, struct file *filp, int mode)
{
    MBXIO_KDBG("%s is invoked\n", __FUNCTION__);

    if (E_MBX_SUCCESS != MDrv_MBX_FASYNC(fd,(TYPE_MBX_C_U64) filp, mode)) {
         //fasync_helper(fd, filp, mode, &IRDev.async_queue);//call driver interface to register signal queue.
        MBXIO_KDBG("%s is busy\n", __FUNCTION__);
        return -EBUSY;
    }

    MBXIO_KDBG("%s is ok\n", __FUNCTION__);
    return 0;
}

#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_MBXIO_IOCtl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	unsigned int cmd_cpt = 0;

	switch (cmd)
	{
		case COMPAT_MDRV_MBX_IOC_INIT:
		{
			COMPAT_MS_MBX_INIT_INFO __user *data32;
			MS_MBX_INIT_INFO __user *data;

			int ret = 0;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_INIT;

			mdrv_mbx_init_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_init_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;

		case COMPAT_MDRV_MBX_IOC_REGISTERMSG:
		{
			COMPAT_MS_MBX_REGISTER_MSG __user *data32;
			MS_MBX_REGISTER_MSG __user *data;

			int ret;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_REGISTERMSG;

			mdrv_mbx_register_msg_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_register_msg_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;

		case COMPAT_MDRV_MBX_IOC_SENDMSGLOOPBACK:
		case COMPAT_MDRV_MBX_IOC_SENDMSG:
		{
			COMPAT_MS_MBX_SEND_MSG __user *data32;
			MS_MBX_SEND_MSG __user *data;

			int ret = 0;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_SENDMSG;

			mdrv_mbx_sendmsg_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_sendmsg_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;

		case COMPAT_MDRV_MBX_IOC_CHECKMSG:
		case COMPAT_MDRV_MBX_IOC_RECVMSG:
		{
			COMPAT_MS_MBX_RECV_MSG __user *data32;
			MS_MBX_RECV_MSG __user *data;
			int ret = 0;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_RECVMSG;

			mdrv_mbx_recvmsg_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_recvmsg_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;

		case COMPAT_MDRV_MBX_IOC_GETMSGQSTATUS:
		{
			COMPAT_MS_MBX_GET_MSGQSTATUS __user *data32;
			MS_MBX_GET_MSGQSTATUS __user *data;
			int ret = 0;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_GETMSGQSTATUS;

			mdrv_mbx_get_msgqstatus_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_get_msgqstatus_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;

		case COMPAT_MDRV_MBX_IOC_GETINFORMATION:
		case COMPAT_MDRV_MBX_IOC_SETINFORMATION:
		{
			COMPAT_MS_MBX_CPROSYNC_INFORMATION __user *data32;
			MS_MBX_CPROSYNC_INFORMATION __user *data;
			int ret = 0;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_SETINFORMATION;

			mdrv_mbx_crosync_info_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_crosync_info_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;

		case COMPAT_MDRV_MBX_IOC_GETDRVSTATUS:
		{
			COMPAT_MS_MBX_GET_DRVSTATUS __user *data32;
			MS_MBX_GET_DRVSTATUS __user *data;
			int ret = 0;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			cmd_cpt = MDRV_MBX_IOC_GETDRVSTATUS;

			mdrv_mbx_get_drvstatus_compat_get_ion_allaction_data(data32, data);
    		ret = filp->f_op->unlocked_ioctl(filp, cmd_cpt, (unsigned long) data);
			mdrv_mbx_get_drvstatus_compat_put_ion_allaction_data(data32, data);

			return ret;
		}
		break;


		case MDRV_MBX_IOC_DEINIT:
		case MDRV_MBX_IOC_UNREGISTERMSG:
		case MDRV_mBX_IOC_CLEARMSG:
		case MDRV_MBX_IOC_REMOVEMSG:
		case MDRV_MBX_IOC_MBXENABLE:
		case MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO:
		case MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO:
		default:
    		return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
		    break;

	}

	return 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
long _MDrv_MBXIO_IOCtl(struct file *filp, U32 u32Cmd, unsigned long u32Arg)
#else
long _MDrv_MBXIO_IOCtl(struct inode *inode, struct file *filp, U32 u32Cmd, unsigned long u32Arg)
#endif
{
    int err = 0;
    int retval = 0;
    //MS_U32 flags;

    mutex_lock(&_devMBX.mutex);
    if(_devMBX.refCnt <= 0)
    {
        mutex_unlock(&_devMBX.mutex);
        return -EFAULT;
    }
    mutex_unlock(&_devMBX.mutex);

    /* check u32Cmd valid */
    if (MDRV_MBX_IOC_MAGIC!= _IOC_TYPE(u32Cmd))
    {
        return -ENOTTY;
    }

    if (_IOC_NR(u32Cmd) > MDRV_MBX_IOC_MAX_NR)
    {
        MBXIO_KDBG("IOCtl NR Error!!! (Cmd=%x)\n",u32Cmd);
        return -ENOTTY;
    }

    /* verify Access */
    if (_IOC_DIR(u32Cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    else if (_IOC_DIR(u32Cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)u32Arg, _IOC_SIZE(u32Cmd));
    }
    if (err)
    {
        return -EFAULT;
    }

    _MDrv_MBXIO_IOC_Lock();

    /* handle u32Cmd */
    switch(u32Cmd)
    {
        case MDRV_MBX_IOC_INIT:
            retval = _MDrv_MBXIO_IOC_Init(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_DEINIT:
            retval = _MDrv_MBXIO_IOC_DeInit(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_REGISTERMSG:

            //spin_lock_irqsave(&lock, flags);

            retval = _MDrv_MBXIO_IOC_RegisterMsg(filp, u32Arg);

            //spin_unlock_irqrestore(&lock, flags);

            break;
        case MDRV_MBX_IOC_UNREGISTERMSG:

            //spin_lock_irqsave(&lock, flags);

            retval = _MDrv_MBXIO_IOC_UnRegisterMsg(filp, u32Arg);

            //spin_unlock_irqrestore(&lock, flags);

            break;
        case MDRV_mBX_IOC_CLEARMSG:
            retval = _MDrv_MBXIO_IOC_ClearMsg(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_SENDMSG:
            retval = _MDrv_MBXIO_IOC_SendMsg(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_RECVMSG:
            retval = _MDrv_MBXIO_IOC_RecvMsg(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_CHECKMSG:
            retval = _MDrv_MBXIO_IOC_CheckMsg(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_REMOVEMSG:
            retval = _MDrv_MBXIO_IOC_RemoveLatestMsg(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_GETMSGQSTATUS:
            retval = _MDrv_MBXIO_IOC_GetMsgQStatus(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_MBXENABLE:
            retval = _MDrv_MBXIO_IOC_MbxEnable(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_SETINFORMATION:
            retval = _MDrv_MBXIO_IOC_SetInformation(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_GETINFORMATION:
            retval = _MDrv_MBXIO_IOC_GetInformation(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_SENDMSGLOOPBACK:
            retval = _MDrv_MBXIO_IOC_SendMsgLoopback(filp, u32Arg);
            break;
        case MDRV_MBX_IOC_GETDRVSTATUS:
            retval = _MDrv_MBXIO_IOC_GetDrvStatus(filp, u32Arg);
            break;
        case MDRV_PM_IOC_GET_BRICKTERMINATOR_INFO:
            retval = _MDrv_PMIO_IOC_Get_BrickTerminator_Info(filp, u32Arg);
            break;
		case MDRV_PM_IOC_SET_BRICKTERMINATOR_INFO:
            retval = _MDrv_PMIO_IOC_Set_BrickTerminator_Info(filp, u32Arg);
            break;

        default:  /* redundant, as cmd was checked against MAXNR */
	        _MDrv_MBXIO_IOC_UnLock();
			if ((_IOC_NR(u32Cmd) == MDRV_MBX_IOC_INIT_NR) &&
				(_IOC_SIZE(MDRV_MBX_IOC_INIT) != _IOC_SIZE(u32Cmd)))
				MBXIO_ASSERT(0);
			else
                MBXIO_KDBG(" ERROR IOCtl number %x\n ",u32Cmd);
			return -ENOTTY;
    }

    _MDrv_MBXIO_IOC_UnLock();
    return retval;
}
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
static struct dev_pm_ops mbx_pm_ops;
static struct str_waitfor_dev waitfor;
static int mstar_mbx_drv_suspend(struct platform_device *dev, pm_message_t state);
static int mstar_mbx_drv_resume(struct platform_device *dev);

static int of_mbx_drv_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);

    if (WARN_ON(!pdev))
        return -ENODEV;

    if (waitfor.stage1_s_wait)
        wait_for_completion(&(waitfor.stage1_s_wait->power.completion));

    return mstar_mbx_drv_suspend(pdev, dev->power.power_state);
}

static int of_mbx_drv_resume(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);

    if (WARN_ON(!pdev))
        return -ENODEV;

    if (waitfor.stage1_r_wait)
        wait_for_completion(&(waitfor.stage1_r_wait->power.completion));

    return mstar_mbx_drv_resume(pdev);
}
#endif
static MSTAR_MBX_DEV _st_mbxdev={0};
static int mstar_mbx_drv_suspend(struct platform_device *dev, pm_message_t state)
{
    MSTAR_MBX_DEV *mbxdev=(MSTAR_MBX_DEV*)dev->dev.platform_data;

    return MDrv_MBX_Suspend(mbxdev);
}

static int mstar_mbx_drv_resume(struct platform_device *dev)
{
    MSTAR_MBX_DEV *mbxdev=(MSTAR_MBX_DEV*)dev->dev.platform_data;

    return MDrv_MBX_Resume(mbxdev);
}

static int mstar_mbx_drv_probe(struct platform_device *pdev)
{
    int retval = 0;
    pdev->dev.platform_data = &_st_mbxdev;

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    of_mstar_str("Mstar-mbx", &pdev->dev,
		&mbx_pm_ops, &waitfor,
		&of_mbx_drv_suspend,
		&of_mbx_drv_resume,
		NULL, NULL);
#endif

    return retval;
}

static int mstar_mbx_drv_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}

#if defined (CONFIG_OF)
static struct of_device_id mstarmbx_of_device_ids[] = {
         {.compatible = "mstar-mbx"},
         {},
};
#endif

static struct platform_driver Mstar_mbx_driver = {
	.probe 		= mstar_mbx_drv_probe,
	.remove 	= mstar_mbx_drv_remove,
#ifndef CONFIG_MP_MSTAR_STR_OF_ORDER
	.suspend	= mstar_mbx_drv_suspend,
	.resume		= mstar_mbx_drv_resume,
#endif

	.driver = {
#if defined(CONFIG_OF)
                .of_match_table = mstarmbx_of_device_ids,
#endif
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
		.pm     = &mbx_pm_ops,
#endif
		.name	= "Mstar-mbx",
        .owner  = THIS_MODULE,
	}
};
//-------------------------------------------------------------------------------------------------
// Module functions
//-------------------------------------------------------------------------------------------------
MSYSTEM_STATIC int _MDrv_MBXIO_ModuleInit(void)
{
    int s32Ret;
    dev_t  dev;
#ifdef CONFIG_MSTAR_UDEV_NODE
    mbx_class = class_create(THIS_MODULE, MDRV_NAME_MSMAILBOX);
    if (IS_ERR(mbx_class))
    {
        return PTR_ERR(mbx_class);
    }
#endif

    if(_devMBX.s32Major)
    {
        dev = MKDEV(_devMBX.s32Major, _devMBX.s32Minor);
        s32Ret = register_chrdev_region(dev, MDRV_MBX_DEVICE_COUNT, MDRV_MBX_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, _devMBX.s32Minor, MDRV_MBX_DEVICE_COUNT, MDRV_MBX_NAME);
        _devMBX.s32Major = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        MBXIO_KDBG("Unable to get major %d\n", _devMBX.s32Major);
        return s32Ret;
    }

    cdev_init(&_devMBX.cdev, &_devMBX.fops);
    if (0 != (s32Ret= cdev_add(&_devMBX.cdev, dev, MDRV_MBX_DEVICE_COUNT)))
    {
        MBXIO_KDBG("Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_MBX_DEVICE_COUNT);
        return s32Ret;
    }

    /* initial the whole MBX Driver */
    if(E_MBX_SUCCESS != MDrv_MBX_Startup())
    {
        MBXIO_KDBG("Startup MBX Driver Failed! %d\n", _devMBX.s32Major);
        cdev_del(&_devMBX.cdev);
        unregister_chrdev_region(dev, MDRV_MBX_DEVICE_COUNT);
	 return -ENOMEM;
    }

#ifdef CONFIG_MSTAR_UDEV_NODE
    device_create(mbx_class, NULL, dev, NULL, MDRV_NAME_MSMAILBOX);
#endif

    DRV_MBX_LockIOCTL_Init();
    mutex_init(&_devMBX.mutex);

    platform_driver_register(&Mstar_mbx_driver);

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    mdb_mbx_node_flag = 0;
    MDrv_SYS_UtopiaMdbMkdir();

    mdb_mbx_proc_entry = proc_create("utopia_mdb/mbx", (S_IRUGO|S_IWUGO), NULL, &mdb_mbx_node_operations);
    if (mdb_mbx_proc_entry == NULL)
    {
        MBXIO_KDBG("Unable to create proc node\n");
    }
#endif

    return 0;
}


MSYSTEM_STATIC void _MDrv_MBXIO_ModuleExit(void)
{
    /*de-initial the who MBX Driver */
    MDrv_MBX_Exit();

    cdev_del(&_devMBX.cdev);
    unregister_chrdev_region(MKDEV(_devMBX.s32Major, _devMBX.s32Minor), MDRV_MBX_DEVICE_COUNT);
    platform_driver_unregister(&Mstar_mbx_driver);

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    mdb_mbx_node_flag = 0;
    if (mdb_mbx_proc_entry != NULL)
        proc_remove(mdb_mbx_proc_entry);
#endif
}

#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
#else//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
module_init(_MDrv_MBXIO_ModuleInit);
module_exit(_MDrv_MBXIO_ModuleExit);


EXPORT_SYMBOL(_MDrv_MBXIO_IOC_Lock);
EXPORT_SYMBOL(_MDrv_MBXIO_IOC_UnLock);


MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("MSMAILBOX ioctrl driver");
MODULE_LICENSE("GPL");
#endif//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
