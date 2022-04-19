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
/// file    mdrv_cpu_io.c
/// @brief  kernel mode event api
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
/* From linux. */
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kdev_t.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

/* From mstar. */
//#include "mdrv_event.h"
#include "mdrv_cpu_io.h"
#include "mdrv_types.h"
#include "mst_devid.h"

extern unsigned int query_frequency(void);

//--------------------------------------------------------------------------------------------------
//  Forward declaration
//--------------------------------------------------------------------------------------------------
static int  _MDrv_CPU_Open(struct inode *inode, struct file *filp);
static int  _MDrv_CPU_Release(struct inode *inode, struct file *filp);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long _MDrv_CPU_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int  _MDrv_CPU_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if defined(CONFIG_COMPAT)
static long _MDrv_CPU_Compat_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

#define MDRV_CPU_DEVICE_COUNT 1
//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
typedef struct _MDRV_CPU_HANDLE
{
    S32                     s32Major;
    S32                     s32Minor;
    struct cdev             stDevice;
    struct file_operations  stFileOp;
} MDRV_CPU_HANDLE;

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------
static struct class     *_pstCpuClass = NULL;

static MDRV_CPU_HANDLE _stCpuDev =
{
    .s32Major           = MDRV_MAJOR_CPU,
    .s32Minor           = MDRV_MINOR_CPU,
    .stDevice           =
    {
        .kobj           = {.name= MDRV_NAME_CPU, },
        .owner          = THIS_MODULE,
    },
    .stFileOp           =
    {
        .open           = _MDrv_CPU_Open,
        .release        = _MDrv_CPU_Release,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
        .unlocked_ioctl = _MDrv_CPU_Ioctl,
#else
        .ioctl          = _MDrv_CPU_Ioctl,
#endif
#if defined(CONFIG_COMPAT)
        .compat_ioctl   = _MDrv_CPU_Compat_Ioctl,
#endif
    },
};

//-------------------------------------------------------------------------------------------------
//  Golbal variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local function
//-------------------------------------------------------------------------------------------------
static int __init _MDrv_CPU_Init_IO(void)
{
    S32 s32Ret = 0;
    dev_t dev;

    /* Create cleass. */
    _pstCpuClass = class_create(THIS_MODULE, MDRV_NAME_CPU);
    if (IS_ERR(_pstCpuClass))
    {
        MDRV_CPU_DEBUG("class_create() fail: ret=%td.\n", (size_t)PTR_ERR(_pstCpuClass));
        return PTR_ERR(_pstCpuClass);
    }

    /* Register device node. */
    if (_stCpuDev.s32Major)
    {
        dev = MKDEV(_stCpuDev.s32Major, _stCpuDev.s32Minor);
        s32Ret = register_chrdev_region(dev, MDRV_CPU_DEVICE_COUNT, MDRV_NAME_CPU);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, _stCpuDev.s32Minor, MDRV_CPU_DEVICE_COUNT, MDRV_NAME_CPU);
        _stCpuDev.s32Major = MAJOR(dev);
    }

    /* Check XXX_chrdev_region() result. */
    if (s32Ret < 0)
    {
        MDRV_CPU_DEBUG("XXX_chrdev_region() fail: ret=%d Major=%d, Minor=%d.\n",
                        s32Ret, _stCpuDev.s32Major, _stCpuDev.s32Minor);
        class_destroy(_pstCpuClass);
        return s32Ret;
    }

    /* Init device function pointer. */
    cdev_init(&_stCpuDev.stDevice, &_stCpuDev.stFileOp);
    s32Ret = cdev_add(&_stCpuDev.stDevice, dev, MDRV_CPU_DEVICE_COUNT);
    if (s32Ret != 0)
    {
        MDRV_CPU_DEBUG("cdev_add() fail: ret=%d.\n", s32Ret);
        unregister_chrdev_region(dev, MDRV_CPU_DEVICE_COUNT);
        class_destroy(_pstCpuClass);
        return s32Ret;
    }

    /* Create node. */
    device_create(_pstCpuClass, NULL, dev, NULL, MDRV_NAME_CPU);

    return 0;
}

static void __exit _MDrv_CPU_Exit_IO(void)
{
    dev_t dev;

    /* Destory device function pointer. */
    cdev_del(&_stCpuDev.stDevice);

    /* Un-register device node. */
    dev = MKDEV(_stCpuDev.s32Major, _stCpuDev.s32Minor);
    unregister_chrdev_region(dev, MDRV_CPU_DEVICE_COUNT);

    /* Destory cleass. */
    device_destroy(_pstCpuClass, dev);
    class_destroy(_pstCpuClass);
}

static int _MDrv_CPU_Open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int _MDrv_CPU_Release(struct inode *inode, struct file *filp)
{
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long _MDrv_CPU_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int  _MDrv_CPU_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    S32 s32Ret = 0;
    MDRV_CPU_PARM stParm = {{0}};

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if (MDRV_CPU_IOC_MAGIC != _IOC_TYPE(cmd))
    {
        return -ENOTTY;
    }

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        s32Ret = access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        s32Ret = access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (!s32Ret)
    {
        return -EFAULT;
    }

    /* Prepare data. */
    memset(&stParm, 0, sizeof(MDRV_CPU_PARM));
    if (copy_from_user(&stParm, (void __user *)arg, sizeof(MDRV_CPU_PARM)))
        return -EFAULT;

    /* Command. */
    switch (cmd)
    {
        case MDRV_CPU_IOC_QUERY_LIT:
        {
            U32 u32Clock = 0;
            u32Clock = query_frequency();
            return u32Clock;
        }
        case MDRV_CPU_IOC_QUERY_BIG:
        {
            return 0;
        }
        default:
        {
            return -EFAULT;
        }
    }

}

#if defined(CONFIG_COMPAT)
static long _MDrv_CPU_Compat_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
}
#endif

//-------------------------------------------------------------------------------------------------
//  Golbal function
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Early
//-------------------------------------------------------------------------------------------------
module_init(_MDrv_CPU_Init_IO);
module_exit(_MDrv_CPU_Exit_IO);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("CPU");
MODULE_LICENSE("GPL");
