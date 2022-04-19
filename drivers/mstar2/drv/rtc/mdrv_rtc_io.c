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


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>



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
#include <asm/io.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>





static long mdrv_rtc_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{

	return 0;

}

static ssize_t mdrv_rtc_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{

	return 0;

}

static int mdrv_rtc_open(struct inode *inode, struct file *file)
{
	//printk(KERN_EMERG "==RTC== %s, %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static int mdrv_rtc_release(struct inode *inode, struct file *file)
{
	return 0;
}


static int mdrv_rtc_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{

	return NOTIFY_DONE;
}

static const struct file_operations mstar_rtc_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= mdrv_rtc_write,
	.unlocked_ioctl	= mdrv_rtc_ioctl,
	.open		= mdrv_rtc_open,
	.release	= mdrv_rtc_release,
};

static struct miscdevice mstar_rtc_miscdev = {
	.minor		= RTC_MINOR,
	.name		= "rtc",
	.fops		= &mstar_rtc_fops,
};

static int __init mstar_rtc_init(void)
{

	//printk(KERN_EMERG "==RTC== %s, %d\n", __FUNCTION__, __LINE__);
	printk(KERN_EMERG "[%s] RTC module init!!!!\n", __FUNCTION__);
	return 0;
}

static void __exit mstar_rtc_exit(void)
{
    printk(KERN_EMERG "[%s] RTC module exit!!!!\n", __FUNCTION__);
}

module_init(mstar_rtc_init);
module_exit(mstar_rtc_exit);

MODULE_DESCRIPTION("Mstar RTC Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(RTC_MINOR);


