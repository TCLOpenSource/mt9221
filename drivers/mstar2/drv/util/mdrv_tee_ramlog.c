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

#include <linux/fs.h>
#include <linux/hugetlb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/ctype.h>
#include <asm/setup.h>
#include <asm/cacheflush.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#define tee_ramlog_write_rpoint(base,value)	(*(volatile unsigned int *)(base+4) = (value - base))
#define tee_ramlog_write_wpoint(base,value)	(*(volatile unsigned int *)(base) = (value - base))
#define tee_ramlog_read_wpoint(base)		((*(volatile unsigned int *)(base)) + base)
#define tee_ramlog_read_rpoint(base)		((*(volatile unsigned int *)(base+4)) + base)
#define MAX_TASK 5
#define TEE_RAMLOG_DEBUG 0
#define READ_WRITE_START_OFFSET 8

struct ramlog_context {
	struct mutex tee_ramlog_lock;
	unsigned long tee_ramlog_buf_adr;
	unsigned long tee_ramlog_buf_len;
};

static struct ramlog_context ramlog_task_context[MAX_TASK];
static struct task_struct *tee_ramlog_tsk;
static int task_count = 0;
static struct mutex task_count_lock;
extern unsigned long tee_ramlog_addr;
extern unsigned long tee_ramlog_len;

static void tee_ramlog_init_addr(unsigned long buf_adr, unsigned long buf_len)
{
	ramlog_task_context[task_count].tee_ramlog_buf_adr = buf_adr;
	ramlog_task_context[task_count].tee_ramlog_buf_len = buf_len;
	printk("[Module ramlog]%s %d Addr:0x%lx, Length:0x%lx \n",__func__,__LINE__, ramlog_task_context[task_count].tee_ramlog_buf_adr, ramlog_task_context[task_count].tee_ramlog_buf_len);
	return;
}

void tee_ramlog_dump(struct ramlog_context *context)
{
	char* log_buff_read_point;
	char* tmp_point;
	unsigned long log_buff_write_point = tee_ramlog_read_wpoint(context->tee_ramlog_buf_adr);
	log_buff_read_point = (char* )tee_ramlog_read_rpoint(context->tee_ramlog_buf_adr);
	unsigned long bound = context->tee_ramlog_buf_adr + context->tee_ramlog_buf_len;

	if( ((unsigned long)log_buff_read_point > bound) ||
	((unsigned long)log_buff_read_point < context->tee_ramlog_buf_adr) ||
	(log_buff_write_point > bound) ||
	(log_buff_write_point  < context->tee_ramlog_buf_adr)) {
		return ;
	}

	if((unsigned long)log_buff_read_point == log_buff_write_point) {
		return ;
	}

	mutex_lock(&context->tee_ramlog_lock);

	while ((unsigned long)log_buff_read_point != log_buff_write_point) {
		if(isascii(*(log_buff_read_point)))
			printk("%c", *(log_buff_read_point));

		log_buff_read_point ++;
		tmp_point = (char* )(context->tee_ramlog_buf_adr + context->tee_ramlog_buf_len);
		if(log_buff_read_point == tmp_point) {
			tmp_point = (char* )(context->tee_ramlog_buf_adr + 8);
			log_buff_read_point = tmp_point;
		}
	}
	tee_ramlog_write_rpoint(context->tee_ramlog_buf_adr, (unsigned long)log_buff_read_point);

	mutex_unlock(&context->tee_ramlog_lock);
}

void suspend_ramlog_final_dump(void)
{
	printk("[Module ramlog] %s\n",__func__);
	int i = task_count - 1;
	for(i; i >= 0; i--)
	{
		tee_ramlog_dump(&ramlog_task_context[i]);
	}
	printk("[Module ramlog] %s done\n",__func__);
}

static int tee_ramlog_loop(void *p)
{
	struct ramlog_context *context = (struct ramlog_context *) p;
	mutex_init(&context->tee_ramlog_lock);
#if (TEE_RAMLOG_DEBUG == 1)
	printk("[Module ramlog] %s %d Addr:0x%lx, Length:0x%lx\n",__func__,__LINE__,context->tee_ramlog_buf_adr, context->tee_ramlog_buf_len);
#endif
	while(1) {
		tee_ramlog_dump(context);
		msleep(500);
	}
	return 0;
}

int configure_ramlog(unsigned long tee_ramlog_addr, unsigned long tee_ramlog_len, const char* module_name, unsigned int cache_type)
{
	void *ramlog_vaddr;
	int ret = 0;

	if ((tee_ramlog_addr == 0)|| (tee_ramlog_len == 0)) {
		printk("[Module ramlog] Disable. %s %d Addr:0x%lx, Length:0x%lx\n",__func__,__LINE__,tee_ramlog_addr,tee_ramlog_len);
		return -EINVAL;
	}

	if(cache_type == 0)
		ramlog_vaddr = ioremap_nocache(tee_ramlog_addr, tee_ramlog_len);
	else
		ramlog_vaddr = ioremap(tee_ramlog_addr, tee_ramlog_len);

	if (ramlog_vaddr == NULL) {
		printk("[Module ramlog] ioremap failed\n");
		return -ENOMEM;
	}

	mutex_lock(&task_count_lock);
	tee_ramlog_init_addr((unsigned long)ramlog_vaddr, tee_ramlog_len);

	tee_ramlog_tsk = kthread_run(tee_ramlog_loop, &ramlog_task_context[task_count], module_name);
	if (IS_ERR(tee_ramlog_tsk)) {
		printk("[WARN] Failed to create the tee_ramlog_tsk!!!\n");
		tee_ramlog_tsk = NULL;
		goto out;
	}
	task_count++;
out:
	mutex_unlock(&task_count_lock);
	return ret;
}

#if (TEE_RAMLOG_DEBUG == 1)
#define MAX_DMSG_WRITE_BUFFER	64
static ssize_t ramlog_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_DMSG_WRITE_BUFFER];
	long idx;

	if (!count)
		return count;

	if (count >= MAX_DMSG_WRITE_BUFFER)
		count = MAX_DMSG_WRITE_BUFFER - 1;

	/*
	 * Prevent Tainted Scalar Warning:
	 * Buffer can't be tainted because:
	 * 1. The count never exceeds MAX_DMSG_WRITE_BUFFER i.e. buffer size.
	 * 2. copy_from_user returns 0 in case of correct copy.
	 *So, we don't need to sanitize buffer.
	 *
	 */
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (buffer[0] == '/')
		idx = 3;
	else if (strict_strtol(buffer, 0, &idx) != 0)
		return -EINVAL;

	switch (idx) {
	case 0:
		printk("[Module ramlog TEST code]%s %d\n",__func__,__LINE__);
		configure_ramlog(0x5fd80000,0x100000,"_TEST_",0);
		break;
	case 1:
		suspend_ramlog_final_dump();
		break;
	default:
		printk(KERN_CRIT"    echo 0 >  # configure_ramlog\n");
	}

	return count;
}

const struct file_operations proc_ramlog_operations = {
	.write      = ramlog_proc_write,
};
#endif

int tee_ramlog_init(void)
{
	mutex_init(&task_count_lock);
#if (TEE_RAMLOG_DEBUG == 1)
	proc_create("module_ramlog", S_IRUSR | S_IWUSR, NULL, &proc_ramlog_operations);
#endif
	if((tee_ramlog_addr!=0)&&(tee_ramlog_len!=0))
		configure_ramlog(tee_ramlog_addr,tee_ramlog_len,"R2_RAMLOG",0);
	else
		printk("[Module ramlog] tee_ramlog_addr or tee_ramlog_len not set.\n");

	return 0;
}
EXPORT_SYMBOL(configure_ramlog);
EXPORT_SYMBOL(suspend_ramlog_final_dump);
EXPORT_SYMBOL(tee_ramlog_init);
