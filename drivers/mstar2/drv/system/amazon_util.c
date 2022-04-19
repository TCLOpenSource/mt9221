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

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include "board/Board.h"
#include "mdrv_types.h"

extern ptrdiff_t mstar_pm_base;
#define REG_ADDRESS(addr) (*((volatile U16*)(mstar_pm_base + (addr << 1))))

int MDrv_SYS_IsSupportDolbyVision(bool* const pbSupportDolbyVision)
{
	bool bDisableDV = FALSE;
	bool bIgnore = FALSE;
	unsigned long u32RetryCount = 0;
	unsigned long u32Timeout = 1000000;
	unsigned long u32CheckDoneDelayUs = 5000;
	if (pbSupportDolbyVision == NULL)
		return -EINVAL;

	/* address 0x6E *4 = 0x1B8 */
	REG_ADDRESS(0x2050) = (U16)0x21B8;
	while ((REG_ADDRESS(0x2050) & (U16)0x2000UL)) {
		u32RetryCount++;
		if ((u32RetryCount * u32CheckDoneDelayUs) >= u32Timeout)
			return -EBUSY;
		mdelay(u32CheckDoneDelayUs/1000);
	}

	bDisableDV = (REG_ADDRESS(0x2058) & BIT6) >> 6;
	printk(KERN_ERR "[0x2050] = %x\t[0x2058] = %x\t[bDisableDV] = %d\n", REG_ADDRESS(0x2050), REG_ADDRESS(0x2058), bDisableDV);

	/* address 0x6F *4 = 0x1BC */
	REG_ADDRESS(0x2050) = (U16)0x21BC;

	while ((REG_ADDRESS(0x2050) & (U16)0x2000UL)) {
		u32RetryCount++;
		if ((u32RetryCount * u32CheckDoneDelayUs) >= u32Timeout)
			return -EBUSY;
		mdelay(u32CheckDoneDelayUs/1000);
	}

	bIgnore = REG_ADDRESS(0x2058) & BIT0;
	printk(KERN_ERR "[0x2050] = %x\t[0x2058] = %x\t[bIgnore] = %d\n", REG_ADDRESS(0x2050), REG_ADDRESS(0x2058), bIgnore);

	if (bDisableDV & !bIgnore)
		*pbSupportDolbyVision = FALSE;
	else
		*pbSupportDolbyVision = TRUE;

	return 0;
}
EXPORT_SYMBOL(MDrv_SYS_IsSupportDolbyVision);

int MDrv_SYS_IsSupportDolbyAtmos(bool * const pbSupportDolbyAtmos)
{
	bool bDolbyAtmos = FALSE;
	bool bDolbyRevert = FALSE;
	unsigned long u32RetryCount = 0;
	unsigned long u32Timeout = 1000000;
	unsigned long u32CheckDoneDelayUs = 5000;

	if (pbSupportDolbyAtmos == NULL)
		return -EINVAL;

	/* address 0x6F *4 = 0x1BC */
	REG_ADDRESS(0x2050) = (U16)0x21BC;

	while ((REG_ADDRESS(0x2050) & (U16)0x2000UL)) {
		u32RetryCount++;
		if ((u32RetryCount * u32CheckDoneDelayUs) >= u32Timeout)
		    return -EBUSY;
		mdelay(u32CheckDoneDelayUs/1000);
	}

	/* 6F[15]: Dolby ATMOS */
	bDolbyAtmos = ((REG_ADDRESS(0x2058) & BIT15) >> 15);
	/* 6F[21]: Dolby Revert */
	bDolbyRevert = ((REG_ADDRESS(0x205A) & BIT5) >> 5);
	printk(KERN_ERR "[0x2050] = %x\t[0x2058] = %x\t[0x205A] = %x\t[bDolbyAtmos] = %d\t[bDolbyRevert] = %d\n",
		REG_ADDRESS(0x2050), REG_ADDRESS(0x2058), REG_ADDRESS(0x205A), bDolbyAtmos, bDolbyRevert);

	if (!bDolbyAtmos)
		*pbSupportDolbyAtmos = TRUE;
	else {
		if (bDolbyRevert)
		    *pbSupportDolbyAtmos = TRUE;
		else
		    *pbSupportDolbyAtmos = FALSE;
	}
	return 0;
}
EXPORT_SYMBOL(MDrv_SYS_IsSupportDolbyAtmos);

static int show_is_support_dolby_vision(struct seq_file *s, void *data)
{
	bool b_is_support_dolby_vision = false;
	if (0 > MDrv_SYS_IsSupportDolbyVision(&b_is_support_dolby_vision)) {
		printk(KERN_ERR "[%s:%d]Error: MDrv_SYS_IsSupportDolbyVision fail\n", __FUNCTION__, __LINE__);
		return -EIO;
	} else {
		seq_printf(s, "%d", b_is_support_dolby_vision ? 1 : 0);
	}
	return 0;
}
static int open_is_support_dolby_vision(struct inode *inode, struct file *file)
{
	return single_open(file, show_is_support_dolby_vision, NULL);
}
static const struct file_operations fops_is_support_dolby_vision = {
	.open       = open_is_support_dolby_vision,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};
static int show_is_support_dolby_atmos(struct seq_file *s, void *data)
{
	bool b_is_support_dolby_atmos = false;
	if (0 > MDrv_SYS_IsSupportDolbyAtmos(&b_is_support_dolby_atmos)) {
		printk(KERN_ERR "[%s:%d]Error: MDrv_SYS_IsSupportDolbyAtmos fail\n", __FUNCTION__, __LINE__);
		return -EIO;
	} else {
		seq_printf(s, "%d", b_is_support_dolby_atmos ? 1 : 0);
	}
	return 0;
}
static int open_is_support_dolby_atmos(struct inode *inode, struct file *file)
{
	return single_open(file, show_is_support_dolby_atmos, NULL);
}
static const struct file_operations fops_is_support_dolby_atmos = {
	.open       = open_is_support_dolby_atmos,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};
static int __init init_is_support_dolby_vision(void)
{
	struct dentry *d;
	d = debugfs_create_file("is_support_dolby_vision", 0664, NULL, NULL, &fops_is_support_dolby_vision);
	if (!d) {
		pr_err("Failed to create is_support_dolby_vision debug file.\n");
		return -ENOMEM;
	}
	return 0;
}
static int __init init_is_support_dolby_atmos(void)
{
	struct dentry *d;
	d = debugfs_create_file("is_support_dolby_atmos", 0664, NULL, NULL, &fops_is_support_dolby_atmos);
	if (!d) {
		pr_err("Failed to create is_support_dolby_atmos debug file.\n");
		return -ENOMEM;
	}
	return 0;
}

late_initcall(init_is_support_dolby_vision);
late_initcall(init_is_support_dolby_atmos);

#define UART_REG_ADDR (*(volatile unsigned short*)(mstar_pm_base + 0x1C24))

void MDrv_SYS_SWITCH_UART(unsigned int OnOff)
{
	if (OnOff)
		UART_REG_ADDR &= ~(1<<12);
	else
		UART_REG_ADDR |= (1<<12);
}
EXPORT_SYMBOL(MDrv_SYS_SWITCH_UART);

ssize_t write_switch_uart(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int MAX_WRITE_BUFFER = 10;
	char buffer[MAX_WRITE_BUFFER];
	unsigned int OnOff = 0;
	unsigned int garbage = 0;
	if (!count)
		return count;

	if (count >= MAX_WRITE_BUFFER)
		count = MAX_WRITE_BUFFER - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	if (sscanf(buffer, "%d %d", &OnOff, &garbage) == 2) {
		return -EINVAL;
	} else if (sscanf(buffer, "%d", &OnOff) == 1) {
		MDrv_SYS_SWITCH_UART(OnOff);
		return count;
	}

	return -EINVAL;
}

static int show_switch_uart(struct seq_file *s, void *v)
{
	return 0;
}

static int open_switch_uart(struct inode *inode, struct file *file)
{
	return single_open(file, show_switch_uart, NULL);
}

static const struct file_operations fops_switch_uart = {
	.open       = open_switch_uart,
	.write      = write_switch_uart,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int __init init_switch_uart(void)
{
	struct dentry *d;
	d = debugfs_create_file("switch_uart", 0664, NULL, NULL, &fops_switch_uart);
	if (!d) {
		pr_err("Failed to create switch_uart debug file.\n");
		return -ENOMEM;
	}

	return 0;
}
late_initcall(init_switch_uart);


