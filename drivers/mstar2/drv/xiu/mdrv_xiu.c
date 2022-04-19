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
/// @file   mdrv_xiu.c
/// @brief  Xiu Timeout Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include "chip_int.h"

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
spinlock_t xiu_timeout_lock;
extern ptrdiff_t mstar_pm_base;
static struct dentry *mdrv_dbg_xiu_dir;
static bool __read_mostly xiu_timeout_enabled;

enum {
	E_XIU_DISABLE = 0,
	E_XIU_ENABLE,
};

//for Linux 3.10.86
#if defined(CONFIG_PM_SLEEP) && !defined(SET_LATE_SYSTEM_SLEEP_PM_OPS)
#define SET_LATE_SYSTEM_SLEEP_PM_OPS(suspend_fn, resume_fn) \
        .suspend_late = suspend_fn, \
        .resume_early = resume_fn, \
        .freeze_late = suspend_fn, \
        .thaw_early = resume_fn, \
        .poweroff_late = suspend_fn, \
        .restore_early = resume_fn,
#endif

//-------------------------------------------------------------------------------------------------
//  XIU Timeout Function
//-------------------------------------------------------------------------------------------------
static int __init xiu_timeout_enabled_setup(char *str)
{
        xiu_timeout_enabled = true;
        pr_info("XIU Timeout Enabled.\n");

        return 1;
}
__setup("xiu_timeout_enabled", xiu_timeout_enabled_setup);

static int xiu_timeout_show(struct seq_file *s, void *data)
{
	seq_printf(s, "\n");
	seq_printf(s,   "####### XIU Timeout Detector #######\n"
			"Self-test:\n"
			"echo test > /sys/kernel/debug/dbg_xiu/xiu_timeout\n");
	seq_printf(s, "\n");
	return 0;
}

static int xiu_timeout_open(struct inode *inode, struct file *file)
{
	return single_open(file, xiu_timeout_show, NULL);
}

static ssize_t xiu_timeout_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[32];
	int i;
	volatile void __iomem *reg = (volatile void __iomem *) mstar_pm_base + 0x340000;


	if (!count || count > 16)// out of bound
		return count;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (!strncmp(buffer,"test",4)) {
		pr_err("XIU Timeout Testing... bank: 0x1AXX\n");
		for(i=0; i<0xFF00; i+=0x1000)
		{
			pr_err("Read bank: 0x%x = 0x%x, and set as 1\n", (unsigned int)(reg + i), (unsigned int)(*(volatile unsigned short *)(reg + i)));
			*(volatile unsigned short *)(reg + i) = 1;
		}

		reg += 0x20000;
		pr_err("XIU Timeout Testing... bank: 0x1BXX\n");
		for(i=0; i<0xFF00; i+=0x1000)
		{
			pr_err("Read bank: 0x%x = 0x%x, and set as 1\n", (unsigned int)(reg + i), (unsigned int)(*(volatile unsigned short *)(reg + i)));
			*(volatile unsigned short *)(reg + i) = 1;
		}

		reg += 0x20000;
		pr_err("XIU Timeout Testing... bank: 0x1CXX\n");
		for(i=0; i<0xFF00; i+=0x1000)
		{
			pr_err("Read bank: 0x%x = 0x%x, and set as 1\n", (unsigned int)(reg + i), (unsigned int)(*(volatile unsigned short *)(reg + i)));
			*(volatile unsigned short *)(reg + i) = 1;
		}
	}

	return count;
}

static const struct file_operations xiu_timeout_fops = {
	.owner      = THIS_MODULE,
	.open       = xiu_timeout_open,
	.read       = seq_read,
	.write      = xiu_timeout_write,
	.llseek     = seq_lseek,
	.release    = single_release,
};

irqreturn_t mdrv_xiu_interrupt(int irq,void *dev_id)
{
	int cnt = 20;
	console_verbose();

	while(cnt-- > 0) {
		pr_emerg("@@@@@@ [%s] XIU timeout hit![0x%x], Addr[0x%04x][0x%04x]\n",
				__func__,
				*(volatile unsigned short *)(mstar_pm_base+0x200204),
				*(volatile unsigned short *)(mstar_pm_base+0x200228),
				*(volatile unsigned short *)(mstar_pm_base+0x200224));
		udelay(50);
	}

	BUG();
}

static void init_xiu_timeout(void)
{
	*(volatile unsigned short *)(mstar_pm_base+0x200250) = 0xffff;
	*(volatile unsigned short *)(mstar_pm_base+0x200254) = 0xffff;
	*(volatile unsigned short *)(mstar_pm_base+0x200264) = 0xffff;
	*(volatile unsigned short *)(mstar_pm_base+0x20026c) = 0xffff;
	*(volatile unsigned short *)(mstar_pm_base+0x200274) = 0xffff;

	//*(volatile unsigned short *)(mstar_pm_base+0x000250) = 0xffff;
	//*(volatile unsigned short *)(mstar_pm_base+0x000254) = 0xffff;
	//*(volatile unsigned short *)(mstar_pm_base+0x000264) = 0xffff;
	//*(volatile unsigned short *)(mstar_pm_base+0x00026c) = 0xffff;
	//*(volatile unsigned short *)(mstar_pm_base+0x000274) = 0xffff;
}

static void set_xiu_timeout(int val)
{
	*(volatile unsigned short *)(mstar_pm_base+0x200200) = val;
	//*(volatile unsigned short *)(mstar_pm_base+0x000200) = val;
}

static int mdrv_xiu_suspend(struct device *dev)
{
	set_xiu_timeout(E_XIU_DISABLE);
	disable_irq(E_FIQ_XIU_TIMEOUT);

	pr_info("XIU suspend, disable xiu timeout\n");
	return 0;
}

static int mdrv_xiu_resume(struct device *dev)
{
	enable_irq(E_FIQ_XIU_TIMEOUT);
	init_xiu_timeout();
	set_xiu_timeout(E_XIU_ENABLE);

	pr_info("XIU resume, enable xiu timeout\n");
	return 0;
}

static int mdrv_xiu_probe(struct platform_device *pdev)
{
	struct dentry *dentry;

	if (request_irq(E_FIQ_XIU_TIMEOUT, mdrv_xiu_interrupt, SA_INTERRUPT, "XIU Timeout", NULL)) {
		pr_err("XIU: interrupt register fail\n");
		return -EBUSY;
	}

	mdrv_dbg_xiu_dir = debugfs_create_dir("dbg_xiu", NULL);
	if(!mdrv_dbg_xiu_dir)
		return -ENOMEM;

	dentry = debugfs_create_file("xiu_timeout", S_IRUGO | S_IWUGO, mdrv_dbg_xiu_dir, NULL, &xiu_timeout_fops);
	if(IS_ERR(dentry))
		return -ENOMEM;

	spin_lock_init(&xiu_timeout_lock);
	init_xiu_timeout();
	set_xiu_timeout(E_XIU_ENABLE);

/*for debug
	pr_info("@@ [%s]: XIU timeout is Enabled. 00[0x%x], 14[0x%x], 15[0x%x], 19[0x%x], 1b[0x%x], 1d[0x%x]\n",
			__func__,
			*(volatile unsigned short *)(mstar_pm_base+0x200200),
			*(volatile unsigned short *)(mstar_pm_base+0x200250),
			*(volatile unsigned short *)(mstar_pm_base+0x200254),
			*(volatile unsigned short *)(mstar_pm_base+0x200264),
			*(volatile unsigned short *)(mstar_pm_base+0x20026c),
			*(volatile unsigned short *)(mstar_pm_base+0x200274));
*/
	pr_info("XIU timeout initialized successfully.\n");
	return 0;
}

static int mdrv_xiu_remove(struct platform_device *pdev)
{
	set_xiu_timeout(E_XIU_DISABLE);
	free_irq(E_FIQ_XIU_TIMEOUT,NULL);
	debugfs_remove_recursive(mdrv_dbg_xiu_dir);
	return 0;
}

static const struct dev_pm_ops xiu_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(mdrv_xiu_suspend, mdrv_xiu_resume)
};

#if defined (CONFIG_OF)
static struct of_device_id mdrv_xiu_of_device_ids[] = {
         {.compatible = "xiu_timeout_int"},
         {},
};
#endif

static struct platform_driver mdrv_xiu_driver = {
	.probe      = mdrv_xiu_probe,
	.remove     = mdrv_xiu_remove,
	.driver = {
		.name   = "xiu_timeout_int",
#if defined(CONFIG_OF)
		.of_match_table = mdrv_xiu_of_device_ids,
#endif
		.owner  = THIS_MODULE,
		.pm	= &xiu_pm_ops
    }
};

static int __init mdrv_xiu_init(void)
{
	if (xiu_timeout_enabled)
		platform_driver_register(&mdrv_xiu_driver);
	return 0;
}

static void __exit mdrv_xiu_exit(void)
{
	if (xiu_timeout_enabled)
		platform_driver_unregister(&mdrv_xiu_driver);
}

subsys_initcall(mdrv_xiu_init);
module_exit(mdrv_xiu_exit);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("XIU Timeout");
MODULE_LICENSE("GPL");
