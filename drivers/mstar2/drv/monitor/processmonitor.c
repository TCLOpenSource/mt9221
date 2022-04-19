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


#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/profile.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <processmonitor.h>

#define MAX_TARGET_NUM   50
#define STRINGIFY(x)     #x
#define MAX_PID_LEN     sizeof(STRINGIFY(PID_MAX_LIMIT))
#define MAX_PROC_SIZE    MAX_TARGET_NUM * (MAX_PID_LEN + 1)

static DEFINE_SPINLOCK(monitorlist_lock);
static struct list_head monitor_list[__MAX_NR_MONITORS];
static struct list_head *process_list;

static int process_count[__MAX_NR_MONITORS]={0};

struct target_process{
	char pid[MAX_PID_LEN];
	struct list_head node;
};

static ssize_t process_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	struct target_process *process_node;
	char pid_buf[MAX_PID_LEN];
	int i;

	memset(pid_buf,0,MAX_PID_LEN);
	if(count > MAX_PID_LEN)
		count = MAX_PID_LEN;
	if(copy_from_user(pid_buf, buf, count))
		return -EFAULT;

	if(pid_buf[count-1] == '\n')
		pid_buf[count-1] = '\0';
	else
		pid_buf[count] = '\0';

	process_node = kmalloc(sizeof(struct target_process),GFP_KERNEL);
	if(process_node == NULL)
	{
		printk (KERN_ERR "\033[1;31m process node allocate memory fail \033[m\n");
		return -ENOMEM;
	}

	strcpy(process_node->pid,pid_buf);
	for(i = 0; i < __MAX_NR_MONITORS; i++){
		if(strcmp(file->f_path.dentry->d_iname,monitor_text[i]) == 0){
			if(process_count[i] >= MAX_TARGET_NUM){
				printk (KERN_ERR "\033[1;31m Too much process be monitored !!\033[m\n");
				return -ENOMEM;
			}
			spin_lock(&monitorlist_lock);
			list_add_tail(&process_node->node,&monitor_list[i]);
			spin_unlock(&monitorlist_lock);
			process_count[i]++;
		}
	}

	return count;
}

static void process_proc_show(struct seq_file *m, void *v)
{
	struct target_process *process_iterator;
	char print_buf[MAX_PROC_SIZE] = {0};

	if(!list_empty(process_list)){
		list_for_each_entry(process_iterator, process_list, node)
		{
			strcat(print_buf,process_iterator->pid);
			strcat(print_buf,",");
		}
		print_buf[strlen(print_buf) - 1] = '\0';
		seq_printf(m, "%s\n",print_buf);
	}else{
		seq_printf(m, "%d\n",0);
	}
}

static int process_proc_open(struct inode *inode, struct file *file)
{
	int i;
	for(i = 0; i < __MAX_NR_MONITORS; i++){
		if(strcmp(file->f_path.dentry->d_iname,monitor_text[i]) == 0) {
			process_list = &monitor_list[i];
		}
	}
	return single_open(file, process_proc_show, NULL);
}

static const struct file_operations process_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= process_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= process_proc_write,
};

static void __init process_monitor_init(void)
{
	struct proc_dir_entry *process_entry;
	struct proc_dir_entry *proc_monitor_dir;
	int i;

	proc_monitor_dir = proc_mkdir("monitor", NULL);
	if(!proc_monitor_dir)
	{
		printk(KERN_ERR "Error creating proc dir: monitor\n");
		return ;
	}

	for(i = 0;i < __MAX_NR_MONITORS;i++){
		process_entry = proc_create(monitor_text[i], S_IRUSR | S_IWUSR, proc_monitor_dir, &process_proc_fops);
		if(!process_entry)
		{
			printk(KERN_ERR "Error creating proc entry: monitor/%s \n",monitor_text[i]);
			continue;
		}
		INIT_LIST_HEAD(&monitor_list[i]);
	}

	printk(KERN_INFO "process moniter initialized\n");
}

device_initcall(process_monitor_init);

bool check_monitored(enum monitor_type monitor,struct task_struct *process)
{
	char task_id[MAX_PID_LEN]={0};
	struct target_process *process_iterator;

	if(!process || !monitor_list[monitor].next ||!monitor_list[monitor].prev)
		return false;

	sprintf(task_id,"%d",task_pid_nr(process));
	spin_lock(&monitorlist_lock);
	list_for_each_entry(process_iterator, &monitor_list[monitor], node)
	{
		if(strcmp(process_iterator->pid,task_id) == 0)
		{
			spin_unlock(&monitorlist_lock);
			return true;
		}
	}
	spin_unlock(&monitorlist_lock);
	return false;
}

bool add_monitor(enum monitor_type monitor,struct task_struct *process)
{
	char task_id[MAX_PID_LEN]={0};
	struct target_process *process_node;

	if(!process || !monitor_list[monitor].next ||!monitor_list[monitor].prev)
		return false;

	if(check_monitored(monitor,process))
		return false;

	if(process_count[monitor] >= MAX_TARGET_NUM){
		printk (KERN_ERR "\033[1;31m Too much process be monitored !!\033[m\n");
		return false;
	}

	process_node = kmalloc(sizeof(struct target_process),GFP_KERNEL);
	if(process_node == NULL)
	{
		printk (KERN_ERR "\033[1;31m process node allocate memory fail \033[m\n");
		return false;
	}

	sprintf(task_id,"%d",task_pid_nr(process));
	strcpy(process_node->pid,task_id);
	spin_lock(&monitorlist_lock);
	list_add_tail(&process_node->node,&monitor_list[monitor]);
	spin_unlock(&monitorlist_lock);
	process_count[monitor]++;
	return true;
}


bool exit_monitor(enum monitor_type monitor,struct task_struct *process)
{
	struct target_process *process_iterator;
	struct target_process *tmp_process;
	char task_id[MAX_PID_LEN]={0};

	if(!process || !monitor_list[monitor].next ||!monitor_list[monitor].prev)
		return false;

	sprintf(task_id,"%d",task_pid_nr(process));
	spin_lock(&monitorlist_lock);
	list_for_each_entry_safe(process_iterator, tmp_process, &monitor_list[monitor], node)
	{
		if(strcmp(process_iterator->pid,task_id) == 0)
		{
			list_del(&process_iterator->node);
			spin_unlock(&monitorlist_lock);
			kfree(process_iterator);
			process_count[monitor]--;
			return true;
		}
	}
	spin_unlock(&monitorlist_lock);
	return false;
}

