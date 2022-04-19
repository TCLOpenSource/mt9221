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
/// @file   mdrv_util.c
/// @brief  mdrv_util Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/version.h>
#include "mdrv_util.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/sched/signal.h>
#include <uapi/linux/sched/types.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/of.h>
#endif


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
LIST_HEAD(OriginTable);
LIST_HEAD(TuneList);

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
void ClearTuneList(void)
{
       struct tune_node *tune_iterator;
       struct tune_node *tmp_tune;
       list_for_each_entry_safe(tune_iterator, tmp_tune, &TuneList,node)
       {
               list_del(&tune_iterator->node);
               kfree(tune_iterator);
       }
}

void ClearOriginTable(void)
{
       struct origin_node *tmp_origin;
       struct origin_node *origin_iterator;
       list_for_each_entry_safe(origin_iterator, tmp_origin, &OriginTable,node)
       {
               list_del(&origin_iterator->node);
               kfree(origin_iterator);
       }
}

void MDrv_UTIL_SaveCurrentScheduler(void)
{
    struct task_struct *task;
    struct task_struct *gtask;

    do_each_thread(gtask,task)
    {
        struct origin_node* pOriginNode;
        pOriginNode = kmalloc(sizeof(struct origin_node),GFP_KERNEL);
        if(pOriginNode == NULL)
        {
                printk (KERN_INFO "\033[1;31m allocate memory fail \033[m\n");
                if(!list_empty(&OriginTable))
                {
                        ClearOriginTable();
                }
                return;
        }
        pOriginNode->pid = task->pid;
        pOriginNode->prio = task->rt_priority;
        pOriginNode->policy = task->policy;
        list_add(&pOriginNode->node,&OriginTable);
    }
    while_each_thread(gtask,task);
}

int MDrv_UTIL_SetScheduler(unsigned long arg)
{
    int ret = -1;
    struct tune_table TuneTable;
    struct tune_node *tune_iterator;
    struct task_struct *task;
    struct task_struct *gtask;
    struct sched_param target;
    struct tune_task other;

    if(list_empty(&OriginTable))
    {
        printk (KERN_INFO "\033[1;31m Please save current schedule first!! \033[m\n");
        return -EFAULT;
    }

    if (copy_from_user(&TuneTable, (void __user *)arg, sizeof(struct tune_table)))
    {
        printk (KERN_INFO "\033[1;31m user arg is a bad address  \033[m\n");
        return -EFAULT;
    }

    if(!TuneTable.list || !TuneTable.other)
    {
        printk (KERN_INFO "\033[1;31m both list and other can't be empty  \033[m\n");
        return -EFAULT;
    }

    //store set value of other thread
    if (copy_from_user(&other, (void __user *)TuneTable.other, sizeof(struct tune_task)))
    {
        printk (KERN_INFO "\033[1;31m other is a bad address  \033[m\n");
        return -EFAULT;
    }

    //clear last set value of specific thread
    if(!list_empty(&TuneList))
    {
               ClearTuneList();
    }

    //store set value of specific thread
    do
    {
        struct tune_node* pTuneNode;
        pTuneNode = kmalloc(sizeof(struct tune_node),GFP_KERNEL);
        if(pTuneNode == NULL)
        {
                printk (KERN_INFO "\033[1;31m allocate memory fail \033[m\n");
                if(!list_empty(&TuneList))
                {
                        ClearTuneList();
                }
                return -EFAULT;
        }
        if (copy_from_user(&pTuneNode->tune, (void __user *)TuneTable.list, sizeof(struct tune_task)))
        {
            printk (KERN_INFO "\033[1;31m there is a bad node in list  \033[m\n");
            if(!list_empty(&TuneList))
            {
                    ClearTuneList();
            }
            kfree(pTuneNode);
            return -EFAULT;
        }
        list_add(&pTuneNode->node,&TuneList);
        TuneTable.list = pTuneNode->tune.next;
    }while(TuneTable.list);

    //set new status to task
    do_each_thread(gtask,task)
    {
        rcu_read_lock();
        target.sched_priority = other.prio;
        ret = sched_setscheduler(task, other.policy, &target);
        rcu_read_unlock();
        list_for_each_entry(tune_iterator, &TuneList,node)
        {
            if(strcmp(task->comm,tune_iterator->tune.comm) == 0)
            {
                rcu_read_lock();
                target.sched_priority = tune_iterator->tune.prio;
                ret = sched_setscheduler(task, tune_iterator->tune.policy, &target);
                rcu_read_unlock();
                if(ret != 0) printk (KERN_INFO "\033[1;31m Set task(comm = %s , prio = %d , policy = %d) scheduler fail!!\033[m\n",tune_iterator->tune.comm,tune_iterator->tune.prio,tune_iterator->tune.policy);
                break;
            }
        }
    }
    while_each_thread(gtask,task);
    return ret;
}

int MDrv_UTIL_RestoreScheduler(void)
{
    int ret = -1;
    struct origin_node *origin_iterator;
    struct task_struct *task;
    struct sched_param target;

    if(list_empty(&OriginTable))
    {
        printk(KERN_INFO "[mdrv_util] Please save current schedule first!!\n");
        return -EFAULT;
    }

    list_for_each_entry(origin_iterator, &OriginTable,node)
    {
        struct pid * kpid=find_get_pid(origin_iterator->pid);
        rcu_read_lock();
        task = pid_task(kpid,PIDTYPE_PID);
        rcu_read_unlock();
        if (task != NULL)
        {
            rcu_read_lock();
            target.sched_priority = origin_iterator->prio;
            ret = sched_setscheduler(task, origin_iterator->policy, &target);
            rcu_read_unlock();
        }
        else
        {
            printk (KERN_INFO "[mdrv_util] Warning!! pid: %d No such process\n", origin_iterator->pid);
        }
    }

    //clear tune list
    if(!list_empty(&TuneList))
    {
         ClearTuneList();
    }

    //clear origin table
    if(!list_empty(&OriginTable))
    {
         ClearOriginTable();
    }
    return ret;
}

