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
/// @file   Mdrvl_dfc.c
/// @brief  DFC Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/cpu.h>
#include <linux/suspend.h>
#include <linux/delay.h>

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------
#define DFC_INFO(fmt, args...) printk(KERN_INFO "[DFC INFO] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#define DFC_WARN(fmt, args...) printk(KERN_WARNING "[DFC WARNING] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)
#define DFC_ERR(fmt, args...) printk(KERN_ERR "[DFC ERROR] %s:%d " fmt,__FUNCTION__,__LINE__, ##args)

#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define RIU(addr)                          (*((volatile unsigned short *)(mstar_pm_base + (addr << 1))))
#else
#define RIU(addr)                          (*((volatile unsigned short *)(0xFD000000 + (addr << 1))))
#endif

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define PM_RIU_REG_BASE                 0xFD000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define PM_RIU_REG_BASE                 mstar_pm_base
#endif

#define Hand_shrink 0x1033A0
#define STRING_LEN 100

#if (defined CONFIG_MSTAR_CPU_calibrating) || (defined CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
extern atomic_t disable_dvfs;
extern void Mdrv_CpuFreq_All_Lock(char *caller);
extern void Mdrv_CpuFreq_All_UnLock(char *caller);
#endif

static int dfc_proc_show(struct seq_file *m, void *v) {
	seq_printf(m, "DFC proc!\n");
	return 0;
}

static int dfc_proc_open(struct inode *inode, struct  file *file) {
	return single_open(file, dfc_proc_show, NULL);
}
static ssize_t dfc_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
	size_t len = STRING_LEN;
	char mbuf[STRING_LEN + 1];
	int call_count = 0;
	int frequency = 0;

	if (len > count)
		len = count;

	if (copy_from_user (mbuf, buf, len)){
		DFC_WARN("copy_from_user Error,please check");
		return -EFAULT;
	}

	DFC_INFO("Change frequency to %s\n",mbuf);
	frequency = simple_strtol(mbuf,NULL,10);

    if (frequency == 2400 || frequency == 2133)
    {
        *(volatile unsigned short*)(PM_RIU_REG_BASE+ (0x100516UL<<1)) = 0x0000;
    }
    mdelay(10);

#if (defined CONFIG_MSTAR_CPU_calibrating) || (defined CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
	/* Disable DVFS before suspend */
	Mdrv_CpuFreq_All_Lock((char *)__FUNCTION__);
	atomic_set(&disable_dvfs, 1);
	Mdrv_CpuFreq_All_UnLock((char *)__FUNCTION__);
#endif


	extern void _chip_flush_miu_pipe(void);
	if(disable_nonboot_cpus()){
		enable_nonboot_cpus();
		DFC_WARN("FAILED to STOP CPU\n");
		return 0;
	}
	preempt_disable();
	arch_suspend_disable_irqs();
	_chip_flush_miu_pipe();
	BUG_ON(!irqs_disabled());
	while(RIU(Hand_shrink) != 0xfac0){
		if(call_count == 1){
			if (frequency == 1866)
				RIU(Hand_shrink) = 0x1866;
			else if (frequency == 2133)
				RIU(Hand_shrink) = 0x2133;
			else
				RIU(Hand_shrink) = 0x2400;
		}
		udelay(500);
		call_count++;
	}
	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());
	preempt_enable();
	enable_nonboot_cpus();
	RIU(Hand_shrink) = 0;

#if (defined CONFIG_MSTAR_CPU_calibrating) || (defined CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
	/* Enable DVFS after resume, this is error case */
	Mdrv_CpuFreq_All_Lock((char *)__FUNCTION__);
	atomic_set(&disable_dvfs, 0);
	Mdrv_CpuFreq_All_UnLock((char *)__FUNCTION__);
#endif

    mdelay(10);
    if (frequency == 1866)
    {
        *(volatile unsigned short*)(PM_RIU_REG_BASE+ (0x100516UL<<1)) = 0x00c8;
    }

	DFC_INFO("Return\n");

	return count;
}

static const struct file_operations dfc_proc_fops = {
	.owner = THIS_MODULE,
	.open = dfc_proc_open,
	.write = dfc_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init dfc_proc_init(void) {
	proc_create("dfc", 0, NULL, &dfc_proc_fops);
	return 0;
}

static void __exit dfc_proc_exit(void) {
	remove_proc_entry("dfc", NULL);
}

MODULE_LICENSE("GPL");
module_init(dfc_proc_init);
module_exit(dfc_proc_exit);
