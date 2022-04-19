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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <asm/io.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif
#include <asm/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/swap.h>
#include <linux/ptrace.h>
#include <linux/dma-contiguous.h>
#include <linux/version.h>


#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include <linux/vmalloc.h>

#include "mdrv_types.h"
#include "mst_devid.h"
#include "mdrv_system.h"

#include "mdrv_benchmark_optimize.h"

#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
char app_boost_name_list[LIST_MAX_LENGTH]="android.cts.jank.leanback";
char bench_boost_name_list[LIST_MAX_LENGTH]="com.antutu.ABenchMark:com.antutu.benchmark.full:com.antutu.benchmark.bench64:com.antutu.tvbenchmark:com.primatelabs.geekbench:com.ludashi.benchmark";
/*
echo > :app_thread_name > proc/DVFS_SCALLING_LIST/app_boost_name_list
ex: app_thread_name = "com.johnny" echo ":com.johnny" > proc/DVFS_SCALLING_LIST/app_boost_name_list"
*/
#endif
#define DVFS_SCALLING_LIST_DIR "DVFS_SCALLING_LIST"
static struct proc_dir_entry *dvfs_scaling_list;
#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
extern struct task_struct *bench_boost_sensor;
extern struct task_struct *app_boost_sensor;
DEFINE_MUTEX(app_boost_list_lock);
DEFINE_MUTEX(bench_boost_list_lock);
//struct mutex app_boost_list_lock;
//struct mutex bench_boost_list_lock;

static ssize_t app_boost_list_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *ppos){
    char local_buf[LIST_MAX_LENGTH];
    if (!count)
        return count;

    if (count >= LIST_MAX_LENGTH)
        count = LIST_MAX_LENGTH - 1;

    if (copy_from_user(local_buf, buffer, count))
        return -EFAULT;

    local_buf[count] = '\0';

    if (strlen((app_boost_name_list) + strlen(local_buf)) <= LIST_MAX_LENGTH) {
        strcat(app_boost_name_list,local_buf);
        return count;
    } else {
        return -EFAULT;
    }
}
#endif
#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
unsigned long launch_boost_enable = 0; //launching now
static ssize_t launch_boost_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *ppos){
    char local_buf[LIST_MAX_LENGTH];
    if (!count)
        return count;

    if (count >= LIST_MAX_LENGTH)
        count = LIST_MAX_LENGTH - 1;

    if (copy_from_user(local_buf, buffer, count))
        return -EFAULT;

    launch_boost_enable = local_buf[0] - '0';
    return count;
}
#endif
#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
static ssize_t bench_boost_list_write(struct file *file, const char __user *buffer,
                                 size_t count, loff_t *ppos){
    char local_buf[LIST_MAX_LENGTH];
    if (!count)
        return count;

    if (count >= LIST_MAX_LENGTH)
        count = LIST_MAX_LENGTH - 1;

    if (copy_from_user(local_buf, buffer, count))
        return -EFAULT;

    local_buf[count] = '\0';
    if(strlen((bench_boost_name_list) + strlen(local_buf)) <= LIST_MAX_LENGTH) {
       strcat(bench_boost_name_list,local_buf);
       return count;
    } else {
        return -EFAULT;
    }
}
static ssize_t app_boost_list_read(struct file *file, char __user *buf, size_t size, loff_t *ppos){

    printk("app_boost_name_list :%s \n",app_boost_name_list);
    return 0;
}
#endif
#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
static ssize_t launch_boost_read(struct file *file, char __user *buf, size_t size, loff_t *ppos){
    printk("bench boost enable: %d \n",launch_boost_enable);
    return 0;
}
#endif
#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
static ssize_t bench_boost_list_read(struct file *file, char __user *buf, size_t size, loff_t *ppos){

    printk("bench_boost_name_list :%s \n",bench_boost_name_list);
    return 0;
}
#endif
#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
static int launch_boost_open(struct inode *inode, struct file *file){
    return 0;
}
#endif
#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
static int app_boost_list_open(struct inode *inode, struct file *file){
    return 0;
}
static int bench_boost_list_open(struct inode *inode, struct file *file){
    return 0;
}
static const struct file_operations app_boost_list_fops = {
    .owner = THIS_MODULE,
    .write = app_boost_list_write,
    .read = app_boost_list_read,
    .open = app_boost_list_open,
    .llseek = seq_lseek,
};
static const struct file_operations bench_boost_list_fops = {
    .owner = THIS_MODULE,
    .write = bench_boost_list_write,
    .read = bench_boost_list_read,
    .open = bench_boost_list_open,
    .llseek = seq_lseek,
};
#endif
#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
static const struct file_operations launch_boost_fops = {
    .owner = THIS_MODULE,
    .write = launch_boost_write,
    .read = launch_boost_read,
    .open = launch_boost_open,
    .llseek = seq_lseek,
};
#endif
MSYSTEM_STATIC int __init mod_dvfs_scaling_init(void)
{
    struct proc_dir_entry *size;
    struct proc_dir_entry *timeout;
    dvfs_scaling_list = proc_mkdir(DVFS_SCALLING_LIST_DIR, NULL);
#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
    if (NULL==dvfs_scaling_list) {
        printk(KERN_ALERT "Create dir /proc/%s error!\n",DVFS_SCALLING_LIST_DIR);
    }
    timeout = proc_create("app_boost_list", 0644, dvfs_scaling_list, &app_boost_list_fops);
    if (!timeout) {
        printk(KERN_ALERT "Create dir /proc/%s/app_boost_list!\n",DVFS_SCALLING_LIST_DIR);
        return -ENOMEM;
    }
    timeout = proc_create("bench_boost_list", 0644, dvfs_scaling_list, &bench_boost_list_fops);
    if (!timeout){
        printk(KERN_ALERT "Create dir /proc/%s/bench_boost_list error!\n",DVFS_SCALLING_LIST_DIR);
        return -ENOMEM;
    }
#endif
#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
    timeout = proc_create("lanuch_boost_enable",0644,dvfs_scaling_list, &launch_boost_fops);
    if (!timeout){
         printk(KERN_ALERT "Create dir /proc/%s/launch_boost error!\n",DVFS_SCALLING_LIST_DIR);
         return -ENOMEM;
    }
#endif
}
#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
bool foundSpecialTask(char* searchSource, char* pattern,int len) {
    int result = false;
    char * pch = NULL;
#define SEPARATE_SYMBOL ":"
    char tmpbuf[LIST_MAX_LENGTH] = {0};
    char* base = tmpbuf;
    if (len > LIST_MAX_LENGTH)
        len = LIST_MAX_LENGTH;
    strncpy(tmpbuf, searchSource, len);
    while ((pch = strsep(&base, SEPARATE_SYMBOL)) != NULL) {
        if (strstr(pch,pattern)) {
            result = true;
            break;
        }
    }
        return result;
}
EXPORT_SYMBOL(foundSpecialTask);
struct cpu_scaling_list* cpu_scaling_list_delete(struct cpu_scaling_list *head,struct task_struct *delete_node)
{
    struct cpu_scaling_list *now;
    now = head;
    struct cpu_scaling_list *prev = NULL;
    struct cpu_scalling_list *result;
    int found;
    struct pid *pid_now;
    struct task_struct *task_now;
    found =0;
    rcu_read_lock();
    while(now != NULL)
    {
    	pid_now = find_get_pid(now->target_id);
        task_now = get_pid_task(pid_now, PIDTYPE_PID);
        if (task_now) {
        	if (task_now->pid == delete_node->pid) {
            	found = 1;
            	break;
        	}
            put_task_struct(task_now);
        }
        prev = now;
        now = now->next;
        put_pid(pid_now);
    }
    rcu_read_unlock();
    if (found ==1) {
        if (now == head) {
            result = now->next;
            now->next = NULL;
            kfree(now);
        } else {
            prev->next = now->next;
            result = head;
            kfree(now);
        }
        return result;
    }
    return head;
}
struct cpu_scaling_list* cpu_scaling_list_add(struct cpu_scaling_list *head,struct task_struct *add_node)
{
    struct cpu_scaling_list *now;
    struct cpu_scaling_list *prev = NULL;
    now = head;
    struct pid *pid_now;
    struct task_struct *task_now;
    int found = 0;
    if (add_node->comm == NULL || add_node->signal ==NULL || add_node->signal->oom_score_adj != 0) {
        return now;
    }
    if (now == NULL) {
        now = kmalloc(sizeof(struct cpu_scaling_list),GFP_KERNEL);
        if (now != NULL) {
            now->tsk = add_node;
            now->target_id = add_node->pid;
            now->next = NULL;
            if(foundSpecialTask(bench_boost_name_list,add_node->comm,LIST_MAX_LENGTH)) {
#if  defined(CONFIG_CPU_FREQ) && defined(CONFIG_MSTAR_DVFS)
                bench_boost_sensor=kthread_create(bench_boost_check,NULL,"bench_boost_scaling_check");
                if (IS_ERR(bench_boost_sensor))
                {
                    printk("create kthread for bench_boost_sensor fail\n");
                    bench_boost_sensor = NULL;
                }
                else
                {
                    printk(KERN_ALERT"[johnny] create kthread for bench_boost_sensor success\n");
                    wake_up_process(bench_boost_sensor);
                }
#endif
            }
            else {
#if  defined(CONFIG_CPU_FREQ) && defined(CONFIG_MSTAR_DVFS)
                app_boost_sensor=kthread_create(app_boost_check,NULL, "app_boost_scaling_check");
                if (IS_ERR(app_boost_sensor))
                {
                    printk("create kthread for app_boost_sensor fail\n");
                    app_boost_sensor = NULL;
                }
                else
                {
                    wake_up_process(app_boost_sensor);
                }
#endif
            }
            return now;
        }
        else
        {
            printk("cpu_scaling_list_add: kmalloc failed \n");
            return NULL;
        }
    }
    rcu_read_lock();
    while(now != NULL)
    {
    	pid_now = find_get_pid(now->target_id);
        task_now = get_pid_task(pid_now, PIDTYPE_PID);
        if (task_now) {
        	if (task_now->pid == add_node->pid || strcmp(task_now->comm,add_node->comm)==0) {
            	found = 1;
            	break;
        	}
            put_task_struct(task_now);
        }
        prev = now;
        now = now->next;
        put_pid(pid_now);
    }
    rcu_read_unlock();
    if (found == 0) {
        prev->next = kmalloc(sizeof(struct cpu_scaling_list),GFP_KERNEL);
        if (prev->next != NULL) {
            prev->next->tsk = add_node;
            prev->next->target_id = add_node->pid;
            prev->next->next = NULL;
        }
    }
    return head;
}
EXPORT_SYMBOL(cpu_scaling_list_add);
EXPORT_SYMBOL(cpu_scaling_list_delete);
#endif
MSYSTEM_STATIC void __exit mod_dvfs_scaling_exit(void)
{
}
module_init(mod_dvfs_scaling_init);
module_exit(mod_dvfs_scaling_exit);
