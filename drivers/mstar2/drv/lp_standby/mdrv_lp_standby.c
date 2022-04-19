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

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "hal_lp_standby.h"

static MS_U8 StandbyModeStatus = false;
//-------------------------------------------------------------------------------------------------
// Define & data type
//-------------------------------------------------------------------------------------------------
#define MSTAR_STANDBY_NAME                  "Mstar-lp-standby"
#define MSTAR_STANDBY_PROC_NAME             "mstar_lp_standby"
#define MSTAR_STANDBY_ON                    "1"
#define MSTAR_STANDBY_OFF                   "0"
#define STRING_LEN                          100


static void MDrv_Standby_Setup(void)
{
    HAL_Standby_Setup();
}

void MDrv_Standby_Restore(void)
{
    HAL_Standby_Restore();
}

static int mstar_standby_proc_show(struct seq_file *m, void *v)
{
    seq_printf(m, "Mstar standby mode procfs!\n");
    return 0;
}

static int mstar_standby_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, mstar_standby_proc_show, NULL);
}

static ssize_t mstar_standby_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    size_t len = STRING_LEN;
    char mbuf[STRING_LEN + 1];

    if (len > count)
        len = count;

    if (copy_from_user (mbuf, buf, len))
    {
        MSTAR_STANDBY_WARN("copy_from_user Error,please check");
        return -EFAULT;
    }

    if (strncmp(mbuf, MSTAR_STANDBY_ON, strlen(MSTAR_STANDBY_ON)) == 0)
    {
        if (StandbyModeStatus == false)
        {
            MDrv_Standby_Setup();
            StandbyModeStatus = true;
        }
    }
    else if (strncmp(mbuf, MSTAR_STANDBY_OFF, strlen(MSTAR_STANDBY_OFF)) == 0)
    {
        if (StandbyModeStatus == true)
        {
            MDrv_Standby_Restore();
            StandbyModeStatus = false;
        }
    }
    else
    {
        MSTAR_STANDBY_WARN("Invalid argument: %s\n", mbuf);
    }

    return count;
}

static const struct file_operations mstar_standby_proc_fops = {
    .owner = THIS_MODULE,
    .open = mstar_standby_proc_open,
    .write = mstar_standby_proc_write,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static int mstar_standby_probe(struct platform_device *pdev)
{
    proc_create(MSTAR_STANDBY_PROC_NAME, 0666, NULL, &mstar_standby_proc_fops);

    return 0;
}

static int mstar_standby_remove(struct platform_device *pdev)
{
    remove_proc_entry(MSTAR_STANDBY_PROC_NAME, NULL);

    return 0;
}

static int mstar_standby_suspend(struct platform_device *dev, pm_message_t state)
{
    int ret = 0;

    MSTAR_STANDBY_DBG("%s\n", __FUNCTION__);
#if defined(CONFIG_AUTO_STR_PD_TABLE_SUPPORT)
    if (StandbyModeStatus == true)
    {
        ret = HAL_PWS_LiteModeSuspend();
    }
#endif

    return ret;
}

static int mstar_standby_resume(struct platform_device *dev)
{
    MSTAR_STANDBY_DBG("%s\n", __FUNCTION__);
#if defined(CONFIG_AUTO_STR_PD_TABLE_SUPPORT)
    if (StandbyModeStatus == true)
    {
        HAL_PWS_LiteModeResume();
    }
#endif

    return 0;
}

#if defined (CONFIG_OF)
static struct of_device_id mstar_standby_of_device_ids[] = {
     {.compatible = MSTAR_STANDBY_NAME},
     {},
};
#endif
static struct platform_driver Mstar_standby_driver = {
    .probe      = mstar_standby_probe,
    .remove     = mstar_standby_remove,
    .suspend    = mstar_standby_suspend,
    .resume     = mstar_standby_resume,

    .driver = {
#if defined(CONFIG_OF)
        .of_match_table = mstar_standby_of_device_ids,
#endif
        .name   = MSTAR_STANDBY_NAME,
        .owner  = THIS_MODULE,
    }
};

static int __init mstar_standby_proc_init(void)
{
    platform_driver_register(&Mstar_standby_driver);

    return 0;
}

static void __exit mstar_standby_proc_exit(void)
{
    platform_driver_unregister(&Mstar_standby_driver);
}

MODULE_AUTHOR("MSTAR");
MODULE_LICENSE("GPL");
module_init(mstar_standby_proc_init);
module_exit(mstar_standby_proc_exit);
