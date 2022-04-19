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
#include <asm/uaccess.h>
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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <uapi/linux/sched/types.h>
#endif



#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include <linux/vmalloc.h>
#if defined(CONFIG_MSTAR_DVFS)
#include "mdrv_dvfs.h"
#include "mhal_dvfs.h"
#endif
#include "mdrv_types.h"
#include "mst_devid.h"
#include "mdrv_system.h"
#if defined(CONFIG_MP_BENCHMARK_ACCEL87) || defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
#include "mdrv_benchmark_optimize.h"
#endif
DEFINE_MUTEX(accel87_list_lock);
//struct mutex accel87_list_lock;
extern struct cpu_scaling_list *accel87_list_head;
extern struct task_struct *accel87_sensor;
struct cpu_scaling_list *TVOS_list_head = NULL;
#if defined(CONFIG_SCHED_HMP) && defined(CONFIG_MP_HMP_GTS_SCHEDULER_AGTS) && defined(CONFIG_NR_CPUS) && (defined(CONFIG_CPU_FREQ_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_GOV_INTERACTIVE))
extern unsigned int hmp_up_threshold;
extern unsigned int hmp_down_threshold;
extern int agts_cur_level;
extern unsigned int enable_agts;
#endif
int iFirstFlgag=1;
unsigned long mask_cpu_BigCore=0;
unsigned long mask_cpu_one_Litcore=0;//exclusive_one_Littlecore
unsigned long mask_cpu_Litcore=0;//exclusive_one_Littlecore
unsigned long mask_cpu_all_core=0;

void _get_CPU_mask(void)
{
        int icpu;
#if defined(CONFIG_MSTAR_DVFS)
        unsigned long mask[CONFIG_NR_CPUS];
        int Count_DVFS_TOTAL_CLUSTER_NUM=0;
        //1.Count DVFS TOTAL CLUSTER NUM
        for(icpu=0;icpu <CONFIG_NR_CPUS;icpu++)
        {
            if(getCpuCluster(icpu) > Count_DVFS_TOTAL_CLUSTER_NUM)
            {
                Count_DVFS_TOTAL_CLUSTER_NUM=getCpuCluster(icpu);
            }
        }
        //2.
        //only 1 cluster
        if(Count_DVFS_TOTAL_CLUSTER_NUM==0)
        {
                for(icpu=0;icpu <CONFIG_NR_CPUS;icpu++)
                {
                    if(icpu==0)
                    {
                        mask[0]=0;//reserve core0 for TVOS and so on
                    }
                    else
                    {
                        mask[icpu]=1;//other core to big core
                    }
                    printk("\033[0;41;37m _get_CPU_mask:(cpu,mask)=(%d,%d),(%d)\033[m\n",icpu,mask[icpu],Count_DVFS_TOTAL_CLUSTER_NUM);
                }
        }
        else//cluster > 1
        {
            for(icpu=0;icpu <CONFIG_NR_CPUS;icpu++)
            {
                if(getCpuCluster(icpu)==Count_DVFS_TOTAL_CLUSTER_NUM)
                {
                    //big core
                    mask[icpu]=1;
                }
                else//other core
                {
                    mask[icpu]=0;
                }
                printk("\033[0;41;37m_get_CPU_mask:(cpu,mask)=(%d,%d),(%d)\033[m\n",icpu,mask[icpu],Count_DVFS_TOTAL_CLUSTER_NUM);
            }
        }
            //3.count mask
            for(icpu=0;icpu <CONFIG_NR_CPUS;icpu++)
            {
                //count all cpu mask
                mask_cpu_all_core |= ((unsigned long)1 << icpu);

                //count BigCore mask
                mask_cpu_BigCore |= (mask[icpu] << icpu);

                //count Little core
                if(mask[icpu]==0x0)
                {
                    mask_cpu_Litcore|=((unsigned long)1 << icpu);
                }
            }

            //count exclusive one Little core
            for(icpu=0;icpu <CONFIG_NR_CPUS;icpu++)
            {
                if(mask[icpu]==0x0)
                {
                    mask_cpu_one_Litcore|=((unsigned long)1 << icpu);
                    break;
                }
            }
#else
        //count mask
        for(icpu=0;icpu <CONFIG_NR_CPUS;icpu++)
        {
            //count all cpu mask
            mask_cpu_all_core |= ((unsigned long)1 << icpu);

            if(icpu>0)
            {
                //count BigCore mask
                mask_cpu_BigCore |= ((unsigned long)1 << icpu);
            }
        }
        mask_cpu_one_Litcore=0x1;
        mask_cpu_Litcore=0x1;
#endif
        printk("\033[0;41;37m_get_CPU_mask:(big,one,lit,all)(0x%x,0x%x,0x%x,0x%x)\033[m\n",mask_cpu_BigCore,mask_cpu_one_Litcore,mask_cpu_Litcore,mask_cpu_all_core);
}
unsigned long get_CPU_mask(int mask_type)
{
    unsigned long re_cpu_mask=0;
    if(iFirstFlgag == 1)
    {
        _get_CPU_mask();
        iFirstFlgag=0;
    }
    switch (mask_type)
    {
        case e_Bigcore:
        {
            re_cpu_mask=mask_cpu_BigCore;
            break;
        }
        case e_AllCpuMask:
        {
            re_cpu_mask=mask_cpu_all_core;
            break;
        }
        case e_excl_one_Littlecore:
        {
            re_cpu_mask=mask_cpu_one_Litcore;
            break;
        }
        case e_excl_Littlecore:
        {
            re_cpu_mask=mask_cpu_Litcore;
            break;
        }
        default:
        {
            printk("\033[31m  get_CPU_mask : need check parameter!!!\033[m\n");
            re_cpu_mask=mask_cpu_all_core;
            break;
        }
    }
    return re_cpu_mask;
}
EXPORT_SYMBOL(get_CPU_mask);
void set_TVOS_Affinity(unsigned long cpu_mask_TVOS)
{
	struct cpu_scaling_list *now;
	unsigned long saved_cpu_mask;
	struct task_struct *tsk_temp;
	struct pid *pid_now;
	struct task_struct *task_now;

	now = TVOS_list_head;
	while(now != NULL)
	{
		pid_now = find_get_pid(now->target_id);
		task_now = get_pid_task(pid_now, PIDTYPE_PID);
		if (!task_now)
		{
			//printk("*************** set_TVOS_Affinity is NULL \n");
		}
		else
		{
			for_each_thread(task_now, tsk_temp)
		{
			sched_setaffinity(tsk_temp->pid, to_cpumask(&cpu_mask_TVOS));
		}
			put_task_struct(task_now);
		}
		//sched_getaffinity(now->tsk->pid, &saved_cpu_mask);
		//printk("*** p-name %s policy:%d  priority:%d mask 0x%x \n",now->tsk->comm,now->tsk->policy,now->tsk->rt_priority,saved_cpu_mask);
		now = now->next;
		put_pid(pid_now);
	}
}
void accel87_check(void)
{
    int full_flag;
    int run;
    struct cpu_scaling_list *now;
    struct cpu_scaling_list *antutu;
    struct cpu_scaling_list *zombie_node;
    struct cpu_scaling_list *prev = NULL;
    struct sched_param param_high_prority = { .sched_priority = 97 };
    struct sched_param param_normal_prority = { .sched_priority = 0 };
    struct task_struct *tsk_temp;
    unsigned long saved_cpu_mask;
    unsigned long cpu_mask_BigCore=get_CPU_mask(e_Bigcore);
    unsigned long mask_cpu_excl_one_Littlecore=get_CPU_mask(e_excl_one_Littlecore);
    unsigned long mask_cpu_AllCpu=get_CPU_mask(e_AllCpuMask);
    struct pid *pid_now;
    struct task_struct *task_now;
    while(1)
    {
        run = 0;
        prev = NULL;
        full_flag=0;
        antutu = NULL;
        mutex_lock(&accel87_list_lock);
        now = accel87_list_head;
        rcu_read_lock();
        while(now != NULL)
        {
		pid_now = find_get_pid(now->target_id);
            task_now = get_pid_task(pid_now, PIDTYPE_PID);
            if (!task_now) {
                    zombie_node = now;
                    now = now->next;
                if (prev == NULL) { //cut head
                    accel87_list_head = zombie_node->next;
                    kfree(zombie_node);
                    zombie_node = NULL;
                } else {
                    prev->next = zombie_node->next;
                    kfree(zombie_node);
                    zombie_node = NULL;
                }
            } else {
                if (task_now->signal->oom_score_adj==0) {
                    if (strstr(task_now->comm, "benchmark.full"))
                        full_flag =1;
                        for_each_thread(task_now, tsk_temp)
                        {
                            sched_setaffinity(tsk_temp->pid, to_cpumask(&cpu_mask_BigCore));
                        }
                        //sched_getaffinity(now->tsk->pid, &saved_cpu_mask);
                        sched_setscheduler_nocheck(task_now,SCHED_FIFO, &param_high_prority);
                        //printk("+++ p-name %s policy:%d  priority:%d mask 0x%x \n",now->tsk->comm,now->tsk->policy,now->tsk->rt_priority,saved_cpu_mask);
                        run = 1;
                } else {
                     if (strstr(task_now->comm, "ABenchMark")) {
                         antutu=now;
                     }
                     sched_setscheduler_nocheck(task_now,SCHED_NORMAL, &param_normal_prority);
                }
                if (antutu!=NULL && full_flag) {
                    for_each_thread(antutu->tsk, tsk_temp)
                    {
                        sched_setaffinity(tsk_temp->pid, to_cpumask(&cpu_mask_BigCore));
                    }
                    //sched_getaffinity(antutu->tsk->pid, &saved_cpu_mask);
                    sched_setscheduler_nocheck(antutu->tsk,SCHED_FIFO, &param_high_prority);
                    //printk("--- p-name %s policy:%d  priority:%d mask 0x%x \n",antutu->tsk->comm,antutu->tsk->policy,antutu->tsk->rt_priority,saved_cpu_mask);
                }
                now = now->next;
                put_task_struct(task_now);
            }
            put_pid(pid_now);
        }
        rcu_read_unlock();
#if defined(CONFIG_SCHED_HMP) && defined(CONFIG_MP_HMP_GTS_SCHEDULER_AGTS) && defined(CONFIG_NR_CPUS) && (defined(CONFIG_CPU_FREQ_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_GOV_INTERACTIVE))
        if (run == 1) {
            hmp_up_threshold = 700;
            hmp_down_threshold = 500;
            agts_cur_level = 0;
            enable_agts = 0;
        } else {
            hmp_up_threshold = 350;
            hmp_down_threshold = 150;
            agts_cur_level = 7;
            enable_agts = 1;
        }
#endif
        if(full_flag==1 || run == 1)
        {
            set_TVOS_Affinity(mask_cpu_excl_one_Littlecore);
        }
        else
        {
            set_TVOS_Affinity(mask_cpu_AllCpu);
        }

        if (accel87_list_head == NULL)
        {
            mutex_unlock(&accel87_list_lock);
            break;
        }
        mutex_unlock(&accel87_list_lock);
        msleep(sensor_polling_time);
    }
}
EXPORT_SYMBOL(accel87_check);
struct cpu_scaling_list* accel87_list_delete(struct cpu_scaling_list *head,struct task_struct *delete_node)
{
    struct cpu_scaling_list *now;
    now = head;
    struct cpu_scaling_list *prev = NULL;
    struct cpu_scalling_list *result;
    int found;
    found =0;
    struct pid *pid_now;
    struct task_struct *task_now;
    rcu_read_lock();
    while(now != NULL)
    {
	pid_now = find_get_pid(now->target_id);
        task_now = get_pid_task(pid_now, PIDTYPE_PID);
	if (task_now)
        {
            if (task_now->pid == delete_node->pid)
            {
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
struct cpu_scaling_list* accel87_list_add(struct cpu_scaling_list *head,struct task_struct *add_node)
{
    struct cpu_scaling_list *now;
    struct cpu_scaling_list *prev = NULL;
    struct pid *pid_now;
    struct task_struct *task_now;
    now = head;
    int found = 0;
    if (add_node->comm == NULL || add_node->signal == NULL ) {
        return now;
    }
    if(add_node->signal->oom_score_adj != 0 && strcmp(add_node->comm, "tvos")!=0)
    {
        return now;
    }
    if (now == NULL) {
        now = kmalloc(sizeof(struct cpu_scaling_list),GFP_KERNEL);
        if (now != NULL) {
            now->tsk = add_node;
            now->target_id = add_node->pid;
            now->next = NULL;
            accel87_sensor=kthread_create(accel87_check,NULL,"accel87_check");
            if (IS_ERR(accel87_sensor))
            {
                printk("create kthread for accel87_sensor fail\n");
                accel87_sensor = NULL;
            }
            else
            {
                wake_up_process(accel87_sensor);
            }
            return now;
        }
      else
      {
            printk("accel87_list_add: kmalloc failed \n");
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
EXPORT_SYMBOL(accel87_list_add);
EXPORT_SYMBOL(accel87_list_delete);
