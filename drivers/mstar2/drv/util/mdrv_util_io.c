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
/// @file   mdrv_util_io.c
/// @brief  mdrv_util Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/version.h>
#include "mdrv_util.h"
#include "mst_devid.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/of.h>
#endif

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define MOD_UTIL_DEVICE_COUNT   1

#define MOD_RUTIL_PRI_ADJ       "priority_adjust"

DEFINE_MUTEX(mstar_util_lock);

typedef struct
{
    int s32Major;
    int s32Minor;
    struct cdev cdev;
    struct file_operations fops;
} UTIL_DEV;

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static struct proc_dir_entry *proc_util_dir;

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static long mstar_util_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    mutex_lock(&mstar_util_lock);
    switch (cmd)
    {
        case 0:
            break;
        case IOCTL_MDRV_UTIL_SAVE_CURR_SCHED:
            MDrv_UTIL_SaveCurrentScheduler();
            ret = 0;
            break;
        case IOCTL_MDRV_UTIL_SET_SCHED:
            if (_IOC_SIZE(cmd) != sizeof(struct tune_table))
            {
                printk(KERN_WARNING "[mstarutil] Wrong size of parameter\n");
                return -EINVAL;
            }
            ret = MDrv_UTIL_SetScheduler(arg);
            break;
        case IOCTL_MDRV_UTIL_RESTORE_SCHED:
            ret = MDrv_UTIL_RestoreScheduler();
            break;
        default:
            printk(KERN_WARNING "[mstarutil] Not support command\n");
            ret = -EINVAL;
            break;
    }
    mutex_unlock(&mstar_util_lock);
    return ret;
}

#if defined(CONFIG_COMPAT)
static long mstarutil_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return mstar_util_ioctl(filp,cmd,arg);
}
#endif

static UTIL_DEV util_dev =
{
    .s32Major = MDRV_MAJOR_UTIL,
    .s32Minor = MDRV_MINOR_UTIL,
    .cdev =
    {
        .kobj = {.name= MDRV_NAME_UTIL, },
        .owner = THIS_MODULE,
    },
    .fops =
    {
        .owner = THIS_MODULE,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl = mstar_util_ioctl,
        #else
        .ioctl = mstar_util_ioctl,
        #endif

        #if defined(CONFIG_COMPAT)
        .compat_ioctl = mstarutil_compat_ioctl,
        #endif
    }
};

static int mstar_util_drv_probe(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}

static int mstar_util_drv_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}

#if defined (CONFIG_ARM64)
static struct of_device_id mstarutil_of_device_ids[] =
{
         {.compatible = MDRV_NAME_UTIL},
         {},
};
#endif

static struct platform_driver mstar_util_driver =
{
    .probe      = mstar_util_drv_probe,
    .remove     = mstar_util_drv_remove,
    .driver = {
#if defined(CONFIG_ARM64)
                .of_match_table = mstarutil_of_device_ids,
#endif
                .name   = MDRV_NAME_UTIL,
                .owner  = THIS_MODULE,
    }
};

static int __init mstar_util_drv_init_module(void)
{
    int s32Ret;
    dev_t dev;
    struct proc_dir_entry *entry;

    if (util_dev.s32Major)
    {
        dev = MKDEV(util_dev.s32Major, util_dev.s32Minor);
        s32Ret = register_chrdev_region(dev, MOD_UTIL_DEVICE_COUNT, MDRV_NAME_UTIL);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, util_dev.s32Minor, MOD_UTIL_DEVICE_COUNT, MDRV_NAME_UTIL);
        util_dev.s32Major = MAJOR(dev);
    }

    cdev_init(&util_dev.cdev, &util_dev.fops);
    if (0 != (s32Ret = cdev_add(&util_dev.cdev, dev, MOD_UTIL_DEVICE_COUNT)))
        goto fail;

    if (!(proc_util_dir = proc_mkdir(MDRV_NAME_UTIL, NULL)))
        goto fail;
    if (!(entry = proc_create(MOD_RUTIL_PRI_ADJ, S_IRUSR | S_IWUSR, proc_util_dir, &util_dev.fops)))
        goto fail;

    return platform_driver_register(&mstar_util_driver);

fail:
    unregister_chrdev_region(dev, MOD_UTIL_DEVICE_COUNT);
    return -ENODEV;
}

static void __exit mstar_util_drv_exit_module(void)
{
    if (proc_util_dir)
    {
        remove_proc_entry(MOD_RUTIL_PRI_ADJ, proc_util_dir);
        remove_proc_entry(MDRV_NAME_UTIL, NULL);
        proc_util_dir = NULL;
    }

    cdev_del(&util_dev.cdev);
    unregister_chrdev_region(MKDEV(util_dev.s32Major, util_dev.s32Minor), MOD_UTIL_DEVICE_COUNT);
    platform_driver_unregister(&mstar_util_driver);
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
module_init(mstar_util_drv_init_module);
module_exit(mstar_util_drv_exit_module);
MODULE_LICENSE("GPL");

