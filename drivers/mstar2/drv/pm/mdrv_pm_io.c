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
/// file    mdrv_pm_io.c
/// @brief  PM I/O Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/namei.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/kobject.h>

#include <asm/io.h>
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "mdrv_pm_io.h"
#include "mdrv_pm.h"
#include "mst_devid.h"

//-------------------------------------------------------------------------------------------------
// DEFINE
//-------------------------------------------------------------------------------------------------
#define MDRV_PM_DEVICE_COUNT    (1)
#define MDRV_PM_NAME            "PM"
#define MDRV_PM_DD_NAME         "Mstar-pm"

typedef struct
{
    u32 Address;
    u32 Size;
} PM_DRAM_INFO;

typedef struct
{
    int s32Major;
    int s32Minor;
    struct cdev cdev;
    struct file_operations fops;
} PM_DEV;

//-------------------------------------------------------------------------------------------------
// IOCTL
//-------------------------------------------------------------------------------------------------
static int _MDrv_PM_io_open(struct inode *inode, struct file *filp)
{
    MDRV_PM_INFO("Inside open \n");
    return 0;
}

static int _MDrv_PM_io_release(struct inode *inode, struct file *filp)
{
    MDRV_PM_INFO("Inside close \n");
    return 0;
}

static int _MDrv_PM_Set_Data(struct file *filp, unsigned long arg)
{
    PM_DRAM_INFO stPM_temp;

    if (copy_from_user(&stPM_temp, (PM_DRAM_INFO __user *)arg, sizeof(PM_DRAM_INFO)))
        return EFAULT;
    else
        MDrv_PM_Mapping(PM_MMAP_DATA, (unsigned long long)stPM_temp.Address, (unsigned long long)stPM_temp.Size);

    return 0;
}

static int _MDrv_PM_Set_Dram(struct file *filp, unsigned long arg)
{
    PM_DRAM_INFO stPM_temp;

    if (copy_from_user(&stPM_temp, (PM_DRAM_INFO __user *)arg, sizeof(PM_DRAM_INFO)))
        return EFAULT;
    else
        MDrv_PM_Mapping(PM_MMAP_DRAM, (unsigned long long)stPM_temp.Address, (unsigned long long)stPM_temp.Size);

    return 0;
}

static int _MDrv_PM_Enable_Voice(struct file *filp, unsigned long arg)
{
    int enVoiceWakeup = (int)arg;
    PM_WakeCfg_t buf;
    ssize_t ret = 0;
    memset(&buf,0,sizeof(PM_WakeCfg_t));
    ret = MDrv_PM_Read_Key(PM_KEY_DEFAULT, (char *)&buf);
    buf.bPmWakeEnableCM4 = enVoiceWakeup;
    ret = MDrv_PM_Write_Key(PM_KEY_DEFAULT, (char *)&buf, sizeof(PM_WakeCfg_t));
    return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long _MDrv_PM_io_ioctl(struct file *filp, U32 u32Cmd, unsigned long u32Arg)
#else
static int _MDrv_PM_io_ioctl(struct inode *inode, struct file *filp, unsigned long u32Cmd, unsigned long u32Arg)
#endif
{
    int retval;
    switch (u32Cmd)
    {
#ifndef AMAZON_SET_MMAP_PM_BASE_BY_KERNEL
        case IOCTL_PM_SET_DRAM_CODE_ADDRESS:
            retval = _MDrv_PM_Set_Dram(filp, u32Arg);
            break;

        case IOCTL_PM_SET_DRAM_DATA_ADDRESS:
            retval = _MDrv_PM_Set_Data(filp, u32Arg);
            break;
#endif

        case IOCTL_PM_ENABLE_VOICE_WAKEUP:
            retval = _MDrv_PM_Enable_Voice(filp, u32Arg);
            break;

        default:
            return -ENOTTY;
    }

    return 0;
}

#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_PM_io_ioctlOCtl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    MDRV_PM_DEBUG("\033[32m [%s] \033[0m \n", __FILE__);
    switch (cmd)
    {
        case IOCTL_PM_SET_DRAM_CODE_ADDRESS:
        case IOCTL_PM_SET_DRAM_DATA_ADDRESS:
        case IOCTL_PM_ENABLE_VOICE_WAKEUP:
            return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
        default:
            return -ENOTTY;
    }

    return 0;
}
#endif

PM_DEV _PMDev=
{
    .s32Major=               MDRV_MAJOR_PM,
    .s32Minor=               MDRV_MINOR_PM,
    .cdev=
    {
        .kobj=                  {.name= MDRV_NAME_PM, },
        .owner  =               THIS_MODULE,
    },
    .fops=
    {
        .open=                  _MDrv_PM_io_open,
        .release=               _MDrv_PM_io_release,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl =       _MDrv_PM_io_ioctl,
        #else
        .ioctl =                _MDrv_PM_io_ioctl,
        #endif
        #if defined(CONFIG_COMPAT)
        .compat_ioctl = _Compat_MDrv_PM_io_ioctlOCtl,
        #endif
    },
};

//-------------------------------------------------------------------------------------------------
// DEVICE
//-------------------------------------------------------------------------------------------------
static struct platform_device Mstar_pm_device = {
    .name   =   MDRV_PM_DD_NAME,
    .id     =   0,
    .dev    =   {
    }
};

//-------------------------------------------------------------------------------------------------
// DRIVER
//-------------------------------------------------------------------------------------------------
static int _MDrv_PM_Suspend_Prepare(struct device *dev)
{
    return (MDrv_PM_Suspend(E_PM_STATE_SUSPEND_PRE) != E_PM_OK);
}

static int _MDrv_PM_Suspend_Noirq(struct device *dev)
{
    return (MDrv_PM_Suspend(E_PM_STATE_SUSPEND_NOIRQ) != E_PM_OK);
}

static int _MDrv_PM_Resume_Noirq(struct device *dev)
{
    return (MDrv_PM_Resume(E_PM_STATE_RESUME_NOIRQ) != E_PM_OK);
}

static void _MDrv_PM_Resume_Complete(struct device *dev)
{
    MDrv_PM_Resume(E_PM_STATE_RESUME_COMPLETE);
}

static const struct dev_pm_ops _MDrv_PM_OPS = {
    .prepare        = _MDrv_PM_Suspend_Prepare,
    .suspend_noirq  = _MDrv_PM_Suspend_Noirq,
    .resume_noirq   = _MDrv_PM_Resume_Noirq,
    .complete       = _MDrv_PM_Resume_Complete,
};

static struct of_device_id mstar_pm_of_device_ids[] = {
         {.compatible = "mstar-pm"},
         {},
};

static int _mstar_drv_pm_probe(struct platform_device *pdev)
{
    int retval = 0;
    U8 u8LedPad = 0xFF;
    struct device_node *node = pdev->dev.of_node;
    const struct of_device_id *match = of_match_device(of_match_ptr(mstar_pm_of_device_ids), &pdev->dev);
    if (!(pdev->name) || strcmp(pdev->name, MDRV_PM_DD_NAME) || (pdev->id != 0))
    {
        retval = -ENXIO;
    }

    if (match != NULL)
    {
        if (of_property_read_u8(node, "poweroff_led", &u8LedPad) >= 0)
        {
            if (u8LedPad != 0xFF)
            {
                MDrv_PM_Set_PowerOffLed(u8LedPad);
            }
        }
    }

    return retval;
}

static int _mstar_drv_pm_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver Mstar_pm_driver = {
    .probe          = _mstar_drv_pm_probe,
    .remove         = _mstar_drv_pm_remove,
    .driver         = {
#if defined(CONFIG_OF)
        .of_match_table = mstar_pm_of_device_ids,
#endif
        .pm             = &_MDrv_PM_OPS,
        .name           = MDRV_PM_DD_NAME,
        .owner          = THIS_MODULE,
    }
};

//-------------------------------------------------------------------------------------------------
// NOTIFY
//-------------------------------------------------------------------------------------------------
static int MDrv_PM_Reboot_Notify(struct notifier_block* nb, unsigned long event, void *unused)
{
    int ret = NOTIFY_OK;

    MDRV_PM_INFO("event=%lu\n", event);
    switch (event)
    {
        case SYS_POWER_OFF:
            if (MDrv_PM_Reboot(E_PM_STATE_REBOOT_NOTIFY) != E_PM_OK)
                ret = NOTIFY_BAD;
            break;

        default:
            break;
    }

    return ret;
}

static struct notifier_block Mstar_pm_reboot_notifier = {
    .notifier_call = MDrv_PM_Reboot_Notify,
};

//-------------------------------------------------------------------------------------------------
// MODULE
//-------------------------------------------------------------------------------------------------
static ssize_t mstar_pm_show_WakeCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_DEFAULT, buf);
}

static ssize_t mstar_pm_store_WakeCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_DEFAULT, data, len);
}

static ssize_t mstar_pm_show_PowerDownCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_POWER_DOWN, buf);
}

static ssize_t mstar_pm_store_PowerDownCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_POWER_DOWN, data, len);
}

static ssize_t mstar_pm_show_IrKeyCfg(struct device * device, struct device_attribute * mattr,char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    MDrv_PM_Show_Config(PM_KEY_IR);
    MDrv_PM_Show_Config(PM_KEY_IR_VERSION);
    MDrv_PM_Show_Config(PM_KEY_IR_NORMAL);
    MDrv_PM_Show_Config(PM_KEY_IR_EXT);
    return 0;
}

static ssize_t mstar_pm_store_IrKeyCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_IR, data, len);
}

static ssize_t mstar_pm_show_SARCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_SAR0, buf);
}

static ssize_t mstar_pm_store_SARCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_SAR0, data, len);
}

static ssize_t mstar_pm_show_SARCfg1(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_SAR1, buf);
}

static ssize_t mstar_pm_store_SARCfg1(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_SAR1, data, len);
}

static ssize_t mstar_pm_show_LedCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_LED, buf);
}

static ssize_t mstar_pm_store_LedCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_LED, data, len);
}

static ssize_t mstar_pm_show_WoBTCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_BT, buf);
}

static ssize_t mstar_pm_store_WoBTCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_BT, data, len);
}

static ssize_t mstar_pm_show_WoBTWCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_BTW, buf);
}

static ssize_t mstar_pm_store_WoBTWCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_BTW, data, len);
}

static ssize_t mstar_pm_show_WoEWBSCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_EWBS, buf);
}

static ssize_t mstar_pm_store_WoEWBSCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_EWBS, data, len);
}

static ssize_t mstar_pm_show_WoUSBCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Read_Key(PM_KEY_USB, buf);
}

static ssize_t mstar_pm_store_WoUSBCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    return MDrv_PM_Write_Key(PM_KEY_USB, data, len);
}

static ssize_t mstar_pm_show_PM_Info(struct device * device, struct device_attribute * mattr, char *buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
    MDrv_PM_Show_Config(PM_KEY_DEFAULT);
    MDrv_PM_Show_Config(PM_KEY_POWER_DOWN);
    MDrv_PM_Show_Config(PM_KEY_IR);
    MDrv_PM_Show_Config(PM_KEY_IR_VERSION);
    MDrv_PM_Show_Config(PM_KEY_IR_NORMAL);
    MDrv_PM_Show_Config(PM_KEY_IR_EXT);
    MDrv_PM_Show_Config(PM_KEY_SAR0);
    MDrv_PM_Show_Config(PM_KEY_SAR1);
    MDrv_PM_Show_Config(PM_KEY_LED);
    MDrv_PM_Show_Config(PM_KEY_BT);
    MDrv_PM_Show_Config(PM_KEY_BTW);
    MDrv_PM_Show_Config(PM_KEY_EWBS);
    MDrv_PM_Show_Config(PM_KEY_USB);
    MDrv_PM_Show_Config(PM_KEY_VAD);
    return 0;
}

static ssize_t mstar_pm_store_PM_Info(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    PM_WakeCfg_t    stCfgDef = {0};

    MDRV_PM_DEBUG("Call Trace.\n");
    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
    stCfgDef.u8PmStrMode = *data;
    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (void *)&stCfgDef, sizeof(stCfgDef));
    return len;
}

static ssize_t mstar_pm_show_debug(struct device * device, struct device_attribute * mattr, char *buf)
{
    MDrv_PM_Read_Debug();
    return 0;
}

static ssize_t mstar_pm_store_debug(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    MDrv_PM_Write_Debug(data, len);
    return len;
}

#ifdef CONFIG_AMAZON_WOL
static ssize_t mstar_pm_show_PM_WOL_EN(struct device *device, struct device_attribute *mattr, char *buf)
{
    MDRV_PM_DEBUG("Call Trace.\n");
	MDrv_PM_Show_PM_WOL_EN();
	return 0;
}

static ssize_t mstar_pm_set_PM_WOL_EN(struct device *device, struct device_attribute *mattr, const char *data, size_t len)
{
    MDRV_PM_DEBUG("Call Trace.\n");
	MDrv_PM_SetPM_WOL_EN(data);
	return len;
}
#endif

static DEVICE_ATTR(WakeCfg,         S_IRUGO | S_IWUSR, mstar_pm_show_WakeCfg,       mstar_pm_store_WakeCfg);
static DEVICE_ATTR(PowerDownCfg,    S_IRUGO | S_IWUSR, mstar_pm_show_PowerDownCfg,  mstar_pm_store_PowerDownCfg);
static DEVICE_ATTR(IrKeyCfg,        S_IRUGO | S_IWUSR, mstar_pm_show_IrKeyCfg,      mstar_pm_store_IrKeyCfg);
static DEVICE_ATTR(SARCfg,          S_IRUGO | S_IWUSR, mstar_pm_show_SARCfg,        mstar_pm_store_SARCfg);
static DEVICE_ATTR(SARCfg1,         S_IRUGO | S_IWUSR, mstar_pm_show_SARCfg1,       mstar_pm_store_SARCfg1);
static DEVICE_ATTR(LEDCfg,          S_IRUGO | S_IWUSR, mstar_pm_show_LedCfg,        mstar_pm_store_LedCfg);
static DEVICE_ATTR(WoBTCfg,         S_IRUGO | S_IWUSR, mstar_pm_show_WoBTCfg,       mstar_pm_store_WoBTCfg);
static DEVICE_ATTR(WoBTWCfg,        S_IRUGO | S_IWUSR, mstar_pm_show_WoBTWCfg,      mstar_pm_store_WoBTWCfg);
static DEVICE_ATTR(WoEWBSCfg,       S_IRUGO | S_IWUSR, mstar_pm_show_WoEWBSCfg,     mstar_pm_store_WoEWBSCfg);
static DEVICE_ATTR(WoUSBCfg,        S_IRUGO | S_IWUSR, mstar_pm_show_WoUSBCfg,      mstar_pm_store_WoUSBCfg);
static DEVICE_ATTR(PM_Info,         S_IRUGO | S_IWUSR, mstar_pm_show_PM_Info,       mstar_pm_store_PM_Info);
static DEVICE_ATTR(debug,           S_IRUGO | S_IWUSR, mstar_pm_show_debug,         mstar_pm_store_debug);
#ifdef CONFIG_AMAZON_WOL
static DEVICE_ATTR(PM_WOL_EN,       S_IRUGO | S_IWUSR, mstar_pm_show_PM_WOL_EN,     mstar_pm_set_PM_WOL_EN);
#endif

static struct class *pm_class;

bool disable_background_wakeup = 0;
EXPORT_SYMBOL(disable_background_wakeup);

static ssize_t pm_disable_background_wakeup_show(struct kobject *kobj,
                  struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", disable_background_wakeup);
}

static ssize_t pm_disable_background_wakeup_store(struct kobject *kobj,
                   struct kobj_attribute *attr,
                   const char *buf, size_t n)
{
    uint8_t tmp = 0;

    kstrtou8(buf, 0, &tmp);
    disable_background_wakeup = (bool)tmp;
    return n;
}

static struct kobj_attribute pm_disable_background_wakeup_attr =
    __ATTR(disable_background_wakeup, 0644,
        pm_disable_background_wakeup_show,
        pm_disable_background_wakeup_store);

static struct attribute * g[] = {
    &pm_disable_background_wakeup_attr.attr,
    NULL,
};

static const struct attribute_group attr_group = {
    .attrs = g,
};

MSYSTEM_STATIC int _MDrv_PMIO_ModuleInit(void)
{
    int s32Ret;
    dev_t  dev;
    struct device *pm_dev;

    pm_class = class_create(THIS_MODULE, MDRV_NAME_PM);
    if (IS_ERR(pm_class))
    {
        return PTR_ERR(pm_class);
    }

    if(_PMDev.s32Major)
    {
        dev = MKDEV(_PMDev.s32Major, _PMDev.s32Minor);
        s32Ret = register_chrdev_region(dev, MDRV_PM_DEVICE_COUNT, MDRV_PM_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, _PMDev.s32Minor, MDRV_PM_DEVICE_COUNT, MDRV_PM_NAME);
        _PMDev.s32Major = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        MDRV_PM_ERR("Unable to get major %d\n", _PMDev.s32Major);
        class_destroy(pm_class);
        return s32Ret;
    }

    cdev_init(&_PMDev.cdev, &_PMDev.fops);
    if (0 != (s32Ret= cdev_add(&_PMDev.cdev, dev, MDRV_PM_DEVICE_COUNT)))
    {
        MDRV_PM_ERR("Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_PM_DEVICE_COUNT);
        class_destroy(pm_class);
        return s32Ret;
    }

    pm_dev = device_create(pm_class, NULL, dev, NULL, MDRV_NAME_PM);
    platform_device_register(&Mstar_pm_device);
    platform_driver_register(&Mstar_pm_driver);
    register_reboot_notifier(&Mstar_pm_reboot_notifier);

    /* Path = /sys/devices/virtual/pm/pm/xxx */
    device_create_file(pm_dev, &dev_attr_WakeCfg);
    device_create_file(pm_dev, &dev_attr_PowerDownCfg);
    device_create_file(pm_dev, &dev_attr_IrKeyCfg);
    device_create_file(pm_dev, &dev_attr_SARCfg);
    device_create_file(pm_dev, &dev_attr_SARCfg1);
    device_create_file(pm_dev, &dev_attr_LEDCfg);
    device_create_file(pm_dev, &dev_attr_WoBTCfg);
    device_create_file(pm_dev, &dev_attr_WoBTWCfg);
    device_create_file(pm_dev, &dev_attr_WoEWBSCfg);
    device_create_file(pm_dev, &dev_attr_WoUSBCfg);
    device_create_file(pm_dev, &dev_attr_PM_Info);
    device_create_file(pm_dev, &dev_attr_debug);
#ifdef CONFIG_AMAZON_WOL
    device_create_file(pm_dev, &dev_attr_PM_WOL_EN);
#endif

    s32Ret = sysfs_create_group(power_kobj, &attr_group);
    if (s32Ret) {
        MDRV_PM_ERR("sysfs_create_groups() fail errno=%d.\n", s32Ret);
    }

    /* Init default value. */
    MDrv_PM_Reset_Key(PM_KEY_DEFAULT);
    MDrv_PM_Reset_Key(PM_KEY_POWER_DOWN);
    MDrv_PM_Reset_Key(PM_KEY_IR);
    MDrv_PM_Reset_Key(PM_KEY_SAR0);
    MDrv_PM_Reset_Key(PM_KEY_SAR1);
    MDrv_PM_Reset_Key(PM_KEY_LED);
    MDrv_PM_Reset_Key(PM_KEY_BT);
    MDrv_PM_Reset_Key(PM_KEY_BTW);
    MDrv_PM_Reset_Key(PM_KEY_EWBS);
    MDrv_PM_Reset_Key(PM_KEY_USB);
    return 0;
}

MSYSTEM_STATIC void _MDrv_PMIO_ModuleExit(void)
{
    cdev_del(&_PMDev.cdev);
    unregister_chrdev_region(MKDEV(_PMDev.s32Major, _PMDev.s32Minor), MDRV_PM_DEVICE_COUNT);
    platform_driver_unregister(&Mstar_pm_driver);
    platform_device_unregister(&Mstar_pm_device);
    device_destroy(pm_class, MKDEV(_PMDev.s32Major, _PMDev.s32Minor));
    class_destroy(pm_class);
    sysfs_remove_group(power_kobj, &attr_group);
}

#if !(defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE))
module_init(_MDrv_PMIO_ModuleInit);
module_exit(_MDrv_PMIO_ModuleExit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("PM ioctrl driver");
MODULE_LICENSE("GPL");
#endif
