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

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>

#ifndef __MDRV_DVFS_H__
#include "mdrv_dvfs.h"
#endif

#ifndef __MHAL_DVFS_H__
#include "mhal_dvfs.h"
#endif

#ifdef CONFIG_MSTAR_DVFS_DEBUG
extern void trigger_ir_boost_event(void);
extern bool del_all_boost_client(int cpu_id);
extern int _CPU_calibrating_proc_write(const unsigned long, const unsigned long, int);
static int Pass_Flag = 0;
static int Ret_Cmd = 0;
atomic_t dvfs_test_proc_is_open = ATOMIC_INIT(0);

static int dvfs_test_seq_show(struct seq_file *s, void *v)
{
    if (Pass_Flag == 0)
        seq_printf(s,"CMD: %d, Fail\n",Ret_Cmd);
    else if (Pass_Flag == 1)
        seq_printf(s,"CMD: %d, Pass\n",Ret_Cmd);
    Pass_Flag = 0;
    Ret_Cmd = 0;
    return 0;
}

static int dvfs_test_proc_open(struct inode *inode, struct file *file)
{
    if (atomic_read(&dvfs_test_proc_is_open))
        return -EACCES;

    atomic_set(&dvfs_test_proc_is_open, 1);

    return single_open(file, &dvfs_test_seq_show, NULL);
}

static int dvfs_test_proc_release(struct inode *inode, struct file * file)
{

    WARN_ON(!atomic_read(&dvfs_test_proc_is_open));
    atomic_set(&dvfs_test_proc_is_open, 0);
    return single_release(inode, file);
}

static void clear(void)
{
    int i = 0;
    for_each_online_cpu(i)
    {
        if (i == getCpuCluster(i) )
        {
            del_all_boost_client(i);
            MDrvDvfsSetTemperatureOffset(i,0);
        }
    }
    return;
}

static void setup(int status)
{
    int ret = 0;
    int i = 0;
    for_each_online_cpu(i)
    {
        if ( i == getCpuCluster(i) )
            ret = MDrvDvfsSetStatus(status,i);
    }
    if(ret)
        printk("Not Supprot Status %d\n",status);
    return;
}

#define FREQ_DEVIATION 3000 // tolerance between set freq and exact freq
static int verify_frequency(int target_freq,int cpu)
{
    int ret = 0;
    int freq = 0;
    if ( cpu == getCpuCluster(cpu) )
    {
       freq = MDrvDvfsGetCpuFreq(cpu)*1000;
       printk("target = %d, freq = %d\n",target_freq,freq);
       if (freq <= (target_freq - FREQ_DEVIATION) || freq >= (target_freq + FREQ_DEVIATION) )
           ret = 1;
    }
    return ret;
}

static int verify_status(unsigned int cpu,int status)
{
    int cluster = getCpuCluster(cpu);
    if (status == CONFIG_DVFS_FREEZE_MODE)
        return 0;
    else
        return (MDrvDvfsGetStatus(cluster) != status)?1:0;
}

static int verify_voltage(unsigned int cpu, unsigned freq)
{
    int cluster = getCpuCluster(cpu);
    return MDrvDvfsVerifyVoltage(cluster,freq);
}



/*
1: open file
2: close file
3: set normal
4: set freeze
5: set over temp
6: clear

7: trigger ir
8  trigger app

9: verify ir/app in over temp
10: verify ir in normal/freeze
11: verify app in normal/freeze

12: verify table
13: verify dhrystone
14: verify low loading

*/
#define FREQ_LEVEL 50000 // 50 MHz, for verify dvfs table
#define MAX_DMSG_WRITE_BUFFER 64
static ssize_t dvfs_test_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[MAX_DMSG_WRITE_BUFFER];
    unsigned int garbage = 0;
    unsigned int cmd = 0;
    unsigned int ret = 0;
    unsigned int i = 0;
    unsigned int min_freq = 0, max_freq = 0, cur_freq = 0;

    if (!count)
        return count;

    if (count >= MAX_DMSG_WRITE_BUFFER)
        count = MAX_DMSG_WRITE_BUFFER - 1;

    if (copy_from_user(buffer, buf, count))
        return -EFAULT;

    buffer[count] = '\0';

    if(sscanf(buffer,"%d %d",&cmd, &garbage) == 2)
    {
       printk("usage: echo cmd > /proc/dvfs_test\n");
    }
    else if (sscanf(buffer, "%d", &cmd) == 1)
    {
       switch(cmd)
       {
           case 1:
           {
               char path[MAX_DMSG_WRITE_BUFFER]="";
               if(sscanf(buffer,"%d %s",&cmd,path) == 2)
               {
                   printk("path = %s\n",path);
                   MDrvDvfsOpenFile(path);
               }
               break;
           }
           case 2:
           {
               MDrvDvfsCloseFile();
               break;
           }
           case 3:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_status(i,CONFIG_DVFS_NORMAL_MODE);
                   }
               }
               break;
           }
           case 4:
           {
               setup(CONFIG_DVFS_FREEZE_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_status(i,CONFIG_DVFS_FREEZE_MODE);
                   }
               }
               break;
           }
           case 5:
           {
               setup(CONFIG_DVFS_OVER_TEMPERATURE_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_status(i,CONFIG_DVFS_OVER_TEMPERATURE_MODE);
                   }
               }
               break;
           }
           case 6:
           {
               clear();
               break;
           }
           case 7:
           {
               trigger_ir_boost_event();
               break;
           }
           case 8:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       _CPU_calibrating_proc_write(32, CONFIG_DVFS_CPU_CLOCK_MAX(i), i);
                   }
               }
               break;
           }
           case 9:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                       ret += verify_frequency(CONFIG_DVFS_CPU_CLOCK_PROTECT(i),i);
                   }
               }
               break;
           }
           case 10:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                      ret = verify_frequency(CONFIG_DVFS_CPU_IRBOOST_CLOCK(i),i);
                   }
               }
               break;
           }
           case 11:
           {
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                      ret = verify_frequency(CONFIG_DVFS_CPU_CLOCK_MAX(i),i);
                   }
               }
               break;
           }
           case 12:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                      min_freq = CONFIG_DVFS_CPU_CLOCK_MIN(i);
                      max_freq = CONFIG_DVFS_CPU_CLOCK_MAX(i);
                      for (cur_freq = min_freq; cur_freq <=max_freq; cur_freq += FREQ_LEVEL)
                      {
                          _CPU_calibrating_proc_write(0,cur_freq, i);
                          ret += verify_voltage(i,cur_freq);
                          _CPU_calibrating_proc_write(0,0, i);
                          //printk("freq = %d, pass\n",cur_freq);
                      }
                   }
               }
               clear();
               break;
           }
           case 13:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                   {
                   #if defined(CONFIG_DVFS_DETACH_BOOST_FREQ_AND_MAX_FREQ)
                       ret += verify_frequency(CONFIG_DVFS_CPU_CLOCK_TABLE_MAX(i),i);
                   #else
                       ret += verify_frequency(CONFIG_DVFS_CPU_IRBOOST_CLOCK(i),i);
                   #endif
                   }
               }
               clear();
               break;
            }
           case 14:
           {
               setup(CONFIG_DVFS_NORMAL_MODE);
               for_each_online_cpu(i)
               {
                   if (i == getCpuCluster(i))
                       ret += verify_frequency(CONFIG_DVFS_CPU_CLOCK_MIN(i),i);
               }
               break;
           }
           default:
               printk("unknown cmd\n");
       }

       if(ret)
       {
           Pass_Flag = 0;
           printk("Test Case %d, FAIL\n",cmd);
       }
       else
       {
           Pass_Flag = 1;
           printk("Test Case %d, PASS\n",cmd);
       }
       Ret_Cmd = cmd;
    }
    return count;
}

const struct file_operations proc_dvfs_test_operations = {
        .write    = dvfs_test_proc_write,
        .read     = seq_read,
        .llseek   = seq_lseek,
        .open     = dvfs_test_proc_open,
        .release  = dvfs_test_proc_release,
};


int dvfs_debug_init(void)
{
    struct proc_dir_entry *entry;
    entry = proc_create("dvfs_test",S_IRUSR | S_IWUSR, NULL ,&proc_dvfs_test_operations);
    if (!entry)
        return 0;
    else
        return 1;
}
EXPORT_SYMBOL(dvfs_debug_init);
#endif
