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


/* system header files */
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/uaccess.h>
#endif
#include <linux/syscore_ops.h>

static DEFINE_SPINLOCK(rtimestamp_lock);
extern int Mstar_Timer1_GetMs(void);
int Timer1_GetMs(void)
{
#ifdef CONFIG_MP_CHECKPT_BOOT
    return Mstar_Timer1_GetMs();
#else
    return 0;
#endif 
}
#define MAX_SW_TIMESTAMP_SIZE   (1024)
typedef struct _TIME_STAMP {
	unsigned int u4TimeStamp;
	unsigned char *szString;
    unsigned char * szComm;
} TIME_STAMP_T;
static unsigned int _u4TimeStampSize = 0;
static TIME_STAMP_T _arTimeStamp[MAX_SW_TIMESTAMP_SIZE];
TIME_STAMP_T *x_os_drv_get_timestampKernel(int *pu4Size)
{
	*pu4Size = _u4TimeStampSize;
	return _arTimeStamp;
}

EXPORT_SYMBOL(x_os_drv_get_timestampKernel);
/*
static int show_boottime(char *buf, char **start, off_t offset,
	                            int count, int *eof, void *data)
*/
static int show_boottime(struct seq_file *m, void *v)
{
	unsigned int i, u4Size;
	//int l, len = 0;
	//char *pos=buf;
	TIME_STAMP_T *prTimeStamp;
	unsigned int u4Val;

	u4Size = _u4TimeStampSize;
	prTimeStamp = _arTimeStamp;
    
    
	for (i = 0; i < u4Size; i++) {
		u4Val = prTimeStamp[i].u4TimeStamp;
        if(prTimeStamp[i].szComm)
		{
            seq_printf(m, "%6d.%03d s - %s\t%s\n",
			    (int) (u4Val / 1000),
			   (int) (u4Val % 1000), prTimeStamp[i].szString,prTimeStamp[i].szComm);
        }
        else
        {
            seq_printf(m, "%6d.%03d s - %s\n",
			   (int) (u4Val / 1000),
			   (int) (u4Val % 1000), prTimeStamp[i].szString);
        }
    }
	return 0;
}

#if defined(CONFIG_CC_IS_CLANG) && defined(CONFIG_MSTAR_CHIP)
static ssize_t store_timestamp(struct file *file, const char *buffer,
#else
static int store_timestamp(struct file *file, const char *buffer,
#endif
			   size_t count, loff_t * pos)
{
#define TAGSTRING_SIZE 256
	char *buf, *nbuf;
	unsigned long len =
		(count > TAGSTRING_SIZE - 1) ? TAGSTRING_SIZE - 1 : count;

	if (in_interrupt()) {
		pr_err("Don't use store_timestamp in ISR \n");
		return -EINVAL;
	}

	buf = kmalloc(TAGSTRING_SIZE, GFP_ATOMIC);
	if (!buf) {
		pr_err("store_timestamp no buffer \n");
		return -ENOMEM;
	}

	nbuf = kmalloc(strlen(current->comm) + 1, GFP_ATOMIC);
	if (!nbuf) {
		pr_err("%s, %d, kmalloc fail\n", __func__, __LINE__);
		kfree(buf);
		return -ENOMEM;
	}

	strcpy(nbuf, current->comm);
	if (copy_from_user(buf, buffer, len)) {
		kfree(buf);
		kfree(nbuf);
		return -EFAULT;
	}

	buf[len] = 0;
	if ((len > 0) && (buf[len - 1]== '\n'))
		buf[len - 1] = 0x20;//replace newline with space

	spin_lock(&rtimestamp_lock);
	if (_u4TimeStampSize >= MAX_SW_TIMESTAMP_SIZE) {
		spin_unlock(&rtimestamp_lock);
		//pr_err("_u4TimeStampSize exceed maximum size %d\n", MAX_SW_TIMESTAMP_SIZE);
		kfree(buf);
		kfree(nbuf);
		return -E2BIG;
	}

	_arTimeStamp[_u4TimeStampSize].u4TimeStamp = Timer1_GetMs();
	_arTimeStamp[_u4TimeStampSize].szString = buf;
	_arTimeStamp[_u4TimeStampSize].szComm = nbuf;
	_u4TimeStampSize++;
	spin_unlock(&rtimestamp_lock);

	return strnlen(buf, count);
}

void add_timestamp(char *buffer)
{
	char *buf, *nbuf;
	unsigned long count = strlen(buffer);
	unsigned long len =
		(count > TAGSTRING_SIZE - 1) ? TAGSTRING_SIZE - 1 : count;

	if (in_interrupt()) {
		pr_err("Don't use add_timestamp in ISR \n");
		return;
	}

	buf = kmalloc(TAGSTRING_SIZE, GFP_ATOMIC);
	if (!buf) {
		pr_err("add_timestamp no buffer \n");
		return;
	}

	nbuf = kmalloc(strlen(current->comm) + 1, GFP_ATOMIC);
	if (!nbuf) {
		pr_err("%s, %d, kmalloc fail\n", __func__, __LINE__);
		kfree(buf);
		return;
	}

	strcpy(nbuf, current->comm);
	memcpy(buf, buffer, len);
	buf[len] = 0;

	spin_lock(&rtimestamp_lock);
	if (_u4TimeStampSize >= MAX_SW_TIMESTAMP_SIZE) {
		spin_unlock(&rtimestamp_lock);
		/*
		if(printk_ratelimit())
		{
			pr_err("_u4TimeStampSize exceed maximum size %d\n", MAX_SW_TIMESTAMP_SIZE);
			dump_stack();
		}
		*/
		kfree(buf);
		kfree(nbuf);
		return;
	}

	_arTimeStamp[_u4TimeStampSize].u4TimeStamp = Timer1_GetMs();	//BIM_READ32(REG_RW_TIMER2_LOW)
	_arTimeStamp[_u4TimeStampSize].szString = buf;
	_arTimeStamp[_u4TimeStampSize].szComm = nbuf;
	_u4TimeStampSize++;
	spin_unlock(&rtimestamp_lock);

	return;
}

EXPORT_SYMBOL(add_timestamp);

void free_builtin_timestamp(void)
{
	unsigned int i, u4Size;
	TIME_STAMP_T *prTimeStamp;

	spin_lock(&rtimestamp_lock);
	u4Size = _u4TimeStampSize;
	prTimeStamp = _arTimeStamp;
	for (i = 0; i < u4Size; i++) {
		kfree(prTimeStamp[i].szString);
		if(prTimeStamp[i].szComm)
			kfree(prTimeStamp[i].szComm);
	}
	_u4TimeStampSize = 0;
	spin_unlock(&rtimestamp_lock);
}

EXPORT_SYMBOL(free_builtin_timestamp);

static int bt_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_boottime, PDE_DATA(inode));
}

static const struct file_operations bt_proc_fops = {
	.open = bt_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = store_timestamp,
};
static void timestamp_resume(void)
{
    //do nothing
}
static int timestamp_suspend(void)
{
    pr_info("%s\n",__func__);
    free_builtin_timestamp();
    return 0;
}
static struct syscore_ops timestamp_syscore_ops = {
	.resume		= timestamp_resume,
	.suspend	= timestamp_suspend,
};
    
static int __init timestamp_init(void)
{
    struct proc_dir_entry *proc_file __attribute__ ((unused)) = 0;
    proc_create("boottime", 0666, NULL, &bt_proc_fops);
    register_syscore_ops(&timestamp_syscore_ops);    
    return 0;
}
late_initcall(timestamp_init);
