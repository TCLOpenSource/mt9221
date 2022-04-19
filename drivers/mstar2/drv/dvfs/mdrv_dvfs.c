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

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>

#ifndef __MDRV_DVFS_H__
#include "mdrv_dvfs.h"
#endif

DEFINE_MUTEX(MDrvDvfsMutex);
#define MAX_WRITE_BUFFER    64
static atomic_t dvfs_voltage_proc_is_open = ATOMIC_INIT(0);
static atomic_t dvfs_frequency_proc_is_open = ATOMIC_INIT(0);
static atomic_t dvfs_boost_duration_proc_is_open = ATOMIC_INIT(0);

static atomic_t dvfs_over_temp_debug_proc_is_open = ATOMIC_INIT(0);

static unsigned int bootarg_auto_measurement=0;
static struct proc_dir_entry *proc_mstar_dvfs_dir;

unsigned int mstar_dvfs_debug = 0;
unsigned int mstar_dvfs_info = 0;
int MHalDvfsGetStatus(unsigned int cpu) __attribute__((weak));
int MHalDvfsSetStatus(unsigned int status, unsigned int cpu) __attribute__((weak));
void MHalDvfsSetOverTempDebugOffset(unsigned int cpu,int set_offset) __attribute__((weak));
int MHalDvfsGetOverTempDebugOffset(unsigned int cpu) __attribute__((weak));
int MHalDvfsVerifyVoltage(unsigned int cpu, unsigned int freq) __attribute__((weak));
void MHalDvfsOpenFile(char *path) __attribute__((weak));
void MHalDvfsCloseFile(void) __attribute__((weak));
//=================================================================================================
int MDrvDvfsGetCpuCluster(unsigned int cpu)
{
    mutex_lock(&MDrvDvfsMutex);
    int dwCluster = getCpuCluster(cpu);
    mutex_unlock(&MDrvDvfsMutex);

    return dwCluster;
}
//=================================================================================================
int MDrvDvfsGetCpuClusterMainCpu(unsigned int cpu)
{
    mutex_lock(&MDrvDvfsMutex);
    int iCpuNo = getClusterMainCpu(cpu);
    mutex_unlock(&MDrvDvfsMutex);

    return iCpuNo;
}
//=================================================================================================
U32 MDrvDvfsProc(U32 dwInputCpuClock, U8 dwCpu)
{
    U32 dwOutputCpuClock = 0;
    int dwCluster = getCpuCluster(dwCpu);

    if (bootarg_auto_measurement)
        return (MHalDvfsCpuDisplay(dwCluster) * 1000);

    mutex_lock(&MDrvDvfsMutex);
    dwOutputCpuClock = MHalDvfsProc(dwInputCpuClock / 1000, dwCpu);
    mutex_unlock(&MDrvDvfsMutex);

    return (dwOutputCpuClock * 1000);
}

//=================================================================================================
void MDrvDvfsInit(void)
{
    MDrvHalDvfsInit();
}
//=================================================================================================
unsigned int MDrvDvfsVerifyCpuClock(unsigned int dwCpuClock, unsigned int dwCpu)
{
    return MHalDvfsVerifyCpuClock(dwCpuClock, dwCpu);
}
EXPORT_SYMBOL(MDrvDvfsVerifyCpuClock);


//=================================================================================================
void MDrvDvfsOpenFile(char *path)
{
    if (MHalDvfsOpenFile)
        MHalDvfsOpenFile(path);
}
//=================================================================================================
void MDrvDvfsCloseFile(void)
{
    if (MHalDvfsCloseFile)
        MHalDvfsCloseFile();
}


//=================================================================================================
void MDrvDvfsCpuDisplay(U8 dwCpu)
{
    int dwCluster = getCpuCluster(dwCpu);
    MHalDvfsCpuDisplay(dwCluster);
}

//=================================================================================================
void MDrvDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage, U8 dwCpu)
{
#ifdef CONFIG_MSTAR_CPU_calibrating
    MHalDvfsCpuPowerAdjustment(dwCpuPowerVoltage, dwCpu);
#elif defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
    MHalDvfsCpuPowerAdjustment(dwCpuPowerVoltage, dwCpu);
#endif
}

//=================================================================================================
void MDrvDvfsCorePowerAdjustment(U32 dwCorePowerVoltage, U8 dwCpu)
{
#ifdef CONFIG_MSTAR_CPU_calibrating
    MHalDvfsCorePowerAdjustment(dwCorePowerVoltage, dwCpu);
#elif defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
    MHalDvfsCorePowerAdjustment(dwCorePowerVoltage, dwCpu);
#endif
}

//=================================================================================================
U32 MDrvDvfsQueryCpuClock(U32 dwCpuClockType, U8 dwCpu)
{
    return (MHalDvfsQueryCpuClock(dwCpuClockType, dwCpu) * 1000);
}

//=================================================================================================
U32 MDrvDvfsQueryCpuClockByTemperature(U8 dwCpu)
{
    U32 dwOutputCpuClock = 0;

    if (bootarg_auto_measurement)
        return (CPU_AUTO_TEST_MAX_FREQ);

    //mutex_lock(&MDrvDvfsMutex);
    dwOutputCpuClock = MHalDvfsQueryCpuClockByTemperature(dwCpu);
    //mutex_unlock(&MDrvDvfsMutex);

    return (dwOutputCpuClock * 1000);
}
//=================================================================================================
U32 MDrvDvfsGetCpuFreq(U8 dwCpu)
{
    U32 clock = MHalDvfsGetCpuFreq(dwCpu);
    U8  i = 0;

    for(i=0; i < DVFS_MAX_QUERY; i++)
    {
        if (clock == 0)
            clock = MHalDvfsGetCpuFreq(dwCpu);
        else
            return clock;
    }

    if (clock == 0)
    {
        pr_err("\033[32m[DVFS][ERROR] Get Zero Frequency \033[0m\n");
        WARN_ON(1);
        return DVFS_DEFAULT_CLOCK; // print error message, but not return zero frequency to avoid system crash.
    }
    else
    {
        return clock;
    }
}

//=================================================================================================

U32 MDrvDvfsGetOverTemperatureFlag(U8 dwCluster)
{
    return MHalDvfsGetOverTemperatureFlag(dwCluster);
}
//=================================================================================================
U32 MDrvDvfsGetCpuTemperature(U8 dwCpu)
{
    return MHalDvfsGetCpuTemperature(dwCpu);
}
EXPORT_SYMBOL(MDrvDvfsGetCpuTemperature);
//=================================================================================================
U32 MDrvDvfsGetCpuVoltage(U8 dwCpu)
{
	return MHalDvfsGetCpuVoltage(dwCpu);
}
//=================================================================================================
U32 MDrvDvfsGetVoltage(U8 dwCpu)
{
    return MHalDvfsGetVoltage(dwCpu);
}
//=================================================================================================
U32 MDrvDvfsGetSidd(void)
{
    return MHalDvfsGetSidd();
}

//=================================================================================================
U32 MDrvDvfsGetOsc(U8 dwCpu)
{
    return MHalDvfsGetOsc(dwCpu);
}
//=================================================================================================
U32 MDrvDvfsGetCPUPowerType(U8 dwCpu)
{
    return MHalDvfsGetCpuPowerType(dwCpu);
}
//=================================================================================================
U32 MDrvDvfsGetCOREPowerType(U8 dwCpu)
{
    return MHalDvfsGetCorePowerType(dwCpu);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
//=================================================================================================
void MDrvDvfsGetDvfsTable(U8 dwCpu, dvfs_opp_table *opp_table)
{
#if CONFIG_PM_OPP
    MHalDvfsGetDvfsTable(dwCpu, opp_table);
#endif
    return;
}
#endif

//=================================================================================================
U8 MDrvDvfsGetFreqTable(U8 dwCpu, struct cpufreq_frequency_table **freq_table)
{
    return MHalDvfsGetFreqTable(dwCpu, freq_table);
}
//=================================================================================================
static int MDrvDvfsProbe(struct platform_device *pdev)
{
    int wReturnValue = 0;

    if(!(pdev->name) || \
       strcmp(pdev->name, "Mstar_DVFS") || \
       pdev->id != 0)
    {
        wReturnValue = -ENXIO;
    }

    MDrvDvfsInit();

    return wReturnValue;
}

//=================================================================================================
static int MDrvDvfsRemove(struct platform_device *pdev)
{
    return 0;
}

//=================================================================================================
static int MDrvDvfsSuspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

//=================================================================================================
static int MDrvDvfsResume(struct platform_device *dev)
{
    return 0;
}

//=================================================================================================
static struct platform_driver MstarDvfsDriver =
{
    .probe      = MDrvDvfsProbe,
    .remove     = MDrvDvfsRemove,
    .suspend    = MDrvDvfsSuspend,
    .resume     = MDrvDvfsResume,

    .driver =
    {
        .name   = "Mstar_DVFS",
        .owner  = THIS_MODULE,
    }
};
//=================================================================================================
ssize_t dvfs_voltage_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[MAX_WRITE_BUFFER];
    unsigned int set_cpu = 0;
    unsigned int set_voltage = 0;
    unsigned int garbage = 0;

    if (!count)
        return count;

    if (count >= MAX_WRITE_BUFFER)
        count = MAX_WRITE_BUFFER - 1;

    if (copy_from_user(buffer, buf, count))
        return -EFAULT;

    buffer[count] = '\0';

    if(sscanf(buffer,"%d %d %d", &set_cpu, &set_voltage, &garbage) == 3)
    {
       return -EINVAL;
    }
    else if (sscanf(buffer,"%d %d", &set_cpu, &set_voltage) == 2)
    {
        if (set_cpu >= CONFIG_NR_CPUS)
            return -EINVAL;

        printk("Function: %s, Line: %d, Set CPU:%d voltage:%d\n", __func__, __LINE__, set_cpu, set_voltage);
        MHalDvfsCpuPowerAdjustment(set_voltage, set_cpu);
        return count;

    }
    return -EINVAL;

}

static int dvfs_voltage_seq_show(struct seq_file *s, void *v)
{
    int i,j;
    unsigned int volt, volt_sum, volt_min, volt_max;

    for_each_online_cpu(i)
    {
        //Init Volt variables
        volt = volt_sum = 0;
        volt_max = 0;
        volt_min = 9999;
        //Core_Pwr_Online_Det for the voltage
        for(j =0; j<10; j++)
        {
            volt = MDrvDvfsGetCpuVoltage(i);
            volt_sum += volt;
            if(volt < volt_min)
                volt_min = volt;
            if(volt > volt_max)
                volt_max=volt;
        }
        seq_printf(s, "Det_%d_avg:%d:\n", i, volt_sum/10);
        seq_printf(s, "Det_%d_min:%d:\n", i, volt_min);
        seq_printf(s, "Det_%d_max:%d:\n", i, volt_max);
    }
    return 0;
}

static int dvfs_voltage_proc_open(struct inode *inode, struct file *file)
{
    if(atomic_read(&dvfs_voltage_proc_is_open))
        return -EACCES;

    atomic_set(&dvfs_voltage_proc_is_open, 1);

    return single_open(file, &dvfs_voltage_seq_show, NULL);
}

static int dvfs_voltage_proc_release(struct inode *inode, struct file * file)
{
    WARN_ON(!atomic_read(&dvfs_voltage_proc_is_open));
    atomic_set(&dvfs_voltage_proc_is_open, 0);
    return single_release(inode, file);
}

static const struct file_operations proc_dvfs_voltage_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_voltage_proc_open,
    .write      = dvfs_voltage_proc_write,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = dvfs_voltage_proc_release,
};
//=================================================================================================
ssize_t dvfs_frequency_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[MAX_WRITE_BUFFER];
    unsigned int set_cpu = 0;
    unsigned int set_frequency = 0;
    unsigned int garbage = 0;

    if (!count)
        return count;

    if (count >= MAX_WRITE_BUFFER)
        count = MAX_WRITE_BUFFER - 1;

    if (copy_from_user(buffer, buf, count))
        return -EFAULT;

    buffer[count] = '\0';

    if(sscanf(buffer,"%d %d %d", &set_cpu, &set_frequency, &garbage) == 3)
    {
       return -EINVAL;
    }
    else if (sscanf(buffer,"%d %d", &set_cpu, &set_frequency) == 2)
    {
        if (set_cpu >= CONFIG_NR_CPUS)
            return -EINVAL;

        printk("Function: %s, Line: %d, Set CPU:%d freq:%d\n", __func__, __LINE__, set_cpu, set_frequency);
        MHalDvfsCpuClockAdjustment(set_frequency, set_cpu);
        return count;

    }
    return -EINVAL;
}

static int dvfs_frequency_seq_show(struct seq_file *s, void *v)
{
    int i;
    unsigned int freq = 0;
    int dwCluster;

    for_each_online_cpu(i)
    {
        dwCluster = getCpuCluster(i);
        freq = MHalDvfsCpuDisplay(dwCluster);
        seq_printf(s, "CPU_%d_freq:%u:\n", i, freq);
    }
    return 0;
}

static int dvfs_frequency_proc_open(struct inode *inode, struct file *file)
{
    if(atomic_read(&dvfs_frequency_proc_is_open))
        return -EACCES;
    atomic_set(&dvfs_frequency_proc_is_open, 1);

    return single_open(file, &dvfs_frequency_seq_show, NULL);
}

static int dvfs_frequency_proc_release(struct inode *inode, struct file * file)
{
    WARN_ON(!atomic_read(&dvfs_frequency_proc_is_open));
    atomic_set(&dvfs_frequency_proc_is_open, 0);
    return single_release(inode, file);
}


int MDrvDvfsGetTemperatureOffset(unsigned int cpu)
{
    if (MHalDvfsGetOverTempDebugOffset)
        return MHalDvfsGetOverTempDebugOffset(cpu);
    else
        return 0;
}

void MDrvDvfsSetTemperatureOffset(unsigned int cpu,int offset)
{
    if (MHalDvfsSetOverTempDebugOffset)
        MHalDvfsSetOverTempDebugOffset(cpu,offset);
    return;
}

int MDrvDvfsGetStatus(unsigned int cpu)
{
    if(MHalDvfsGetStatus)
        return MHalDvfsGetStatus(cpu);
    else
        return 0;
}

int MDrvDvfsSetStatus(unsigned int status,unsigned int cpu)
{
    if(MHalDvfsSetStatus)
        return MHalDvfsSetStatus(status,cpu);
    else
        return 0;
}

int MDrvDvfsVerifyVoltage(unsigned int cpu, unsigned int freq)
{
    if(MHalDvfsVerifyVoltage)
        return MHalDvfsVerifyVoltage(cpu, freq/1000);
    else
        return 0;
}

static const struct file_operations proc_dvfs_frequency_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_frequency_proc_open,
    .write      = dvfs_frequency_proc_write,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = dvfs_frequency_proc_release,
};
//=================================================================================================
static int dvfs_temp_info_seq_show(struct seq_file *s, void *v)
{
    int i = 0;
    DVFS_THERMAL_INFO thermal_info;

    thermal_info.GPU_TEMP = 0;
    thermal_info.CPU_TEMP = 0;
    thermal_info.INT_TEMP = 0;

    MHalDvfsGetTemperatureInfo(i,&thermal_info);

    if ( (thermal_info.GPU_TEMP != 0) && (thermal_info.INT_TEMP != 0) && (thermal_info.CPU_TEMP != 0) )
    {
        seq_printf(s, "GPU_%d_temp:%d:\n", i, thermal_info.GPU_TEMP);
        seq_printf(s, "CPU_%d_temp:%d:\n", i, thermal_info.CPU_TEMP);
        seq_printf(s, "INT_%d_temp:%d:\n", i, thermal_info.INT_TEMP);
    }
    else
        seq_printf(s, "not support read GPU & Internal T-sensor\n");
    return 0;
}

static int dvfs_temp_info_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, &dvfs_temp_info_seq_show, NULL);
}

static const struct file_operations proc_dvfs_temp_info_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_temp_info_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
//=================================================================================================
static int dvfs_temp_seq_show(struct seq_file *s, void *v)
{
    int i;
    unsigned int temperature = 0;

    for_each_online_cpu(i)
    {
        temperature = MDrvDvfsGetCpuTemperature(i);
        //printk("CPU:%d temp:%d\n", i, temperature);
        seq_printf(s, "CPU_%d_temp:%d:\n", i, temperature);
    }
    return 0;
}

static int dvfs_temp_proc_open(struct inode *inode, struct file *file)
{
    //return seq_open(file, &dvfs_temp_seq_ops);
    return single_open(file, &dvfs_temp_seq_show, NULL);
}

static const struct file_operations proc_dvfs_temp_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_temp_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

//=================================================================================================
static int dvfs_sidd_seq_show(struct seq_file *s, void *v)
{
    unsigned int sidd_val = 0;

    sidd_val = MDrvDvfsGetSidd();
    //printk("Sidd: %u\n", sidd_val);
    seq_printf(s, "Sidd: %u\n", sidd_val);

    return 0;
}

static int dvfs_sidd_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, &dvfs_sidd_seq_show, NULL);
}

static const struct file_operations proc_dvfs_sidd_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_sidd_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = seq_release,
};

//=================================================================================================
static int dvfs_osc_seq_show(struct seq_file *s, void *v)
{
    int i;
    unsigned int osc_val = 0;

    for_each_online_cpu(i)
    {
        osc_val = MDrvDvfsGetOsc(i);
        //printk("CPU:%d Osc:%d\n", i, osc_val);
        seq_printf(s, "CPU: %d osc: %d\n", i, osc_val);
    }

    return 0;
}

static int dvfs_osc_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, &dvfs_osc_seq_show, NULL);
}

static const struct file_operations proc_dvfs_osc_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_osc_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

//=================================================================================================

extern ssize_t dvfs_boost_duration_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
extern int dvfs_boost_duration_seq_show(struct seq_file *s, void *v);

static int dvfs_boost_duration_proc_open(struct inode *inode, struct file *file)
{
    if(atomic_read(&dvfs_boost_duration_proc_is_open))
        return -EACCES;
    atomic_set(&dvfs_boost_duration_proc_is_open, 1);

    return single_open(file, &dvfs_boost_duration_seq_show, NULL);
}

static int dvfs_boost_duration_proc_release(struct inode *inode, struct file * file)
{
    WARN_ON(!atomic_read(&dvfs_boost_duration_proc_is_open));
    atomic_set(&dvfs_boost_duration_proc_is_open, 0);
    return single_release(inode, file);
}

static const struct file_operations proc_dvfs_boost_duration_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_boost_duration_proc_open,
    .write      = dvfs_boost_duration_proc_write,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = dvfs_boost_duration_proc_release,
};
//=================================================================================================
static int dvfs_cpu_power_type_seq_show(struct seq_file *s, void *v)
{
    int i;
    unsigned int power_type = 0;

    for_each_online_cpu(i)
    {
        power_type = MDrvDvfsGetCPUPowerType(i);
        if( power_type == 2 )
        {
            seq_printf(s, "CPU_%d_power_type : FF (%d)\n", i, power_type);
        }
        else
        {
            seq_printf(s, "CPU_%d_power_type : SS/TT (%d)\n", i, power_type);
        }
    }
    return 0;
}

static int dvfs_cpu_power_type_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, &dvfs_cpu_power_type_seq_show, NULL);
}

static const struct file_operations proc_dvfs_cpu_power_type_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_cpu_power_type_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
//=================================================================================================
static int dvfs_core_power_type_seq_show(struct seq_file *s, void *v)
{
    int i;
    unsigned int power_type = 0;

    for_each_online_cpu(i)
    {
        power_type = MDrvDvfsGetCPUPowerType(i);
        if( power_type == 2 )
        {
            seq_printf(s, "CORE_%d_power_type : FF (%d)\n", i, power_type);
        }
        else
        {
            seq_printf(s, "CORE_%d_power_type : SS/TT (%d)\n", i, power_type);
        }
    }
    return 0;
}

static int dvfs_core_power_type_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, &dvfs_core_power_type_seq_show, NULL);
}

static const struct file_operations proc_dvfs_core_power_type_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_core_power_type_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
//=================================================================================================

ssize_t dvfs_over_temp_debug_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char buffer[MAX_WRITE_BUFFER];

    unsigned int set_offset = 0;
    unsigned int garbage = 0;

    if (!count)
        return count;

    if (count >= MAX_WRITE_BUFFER)
        count = MAX_WRITE_BUFFER - 1;

    if (copy_from_user(buffer, buf, count))
        return -EFAULT;

    buffer[count] = '\0';

    if(sscanf(buffer,"%d %d",&set_offset, &garbage) == 2)
    {
       return -EINVAL;
    }
    else if (sscanf(buffer,"%d", &set_offset) == 1)
    {
        printk("Function: %s, Line: %d, Set over-temp debug offset :%d\n", __func__, __LINE__, set_offset);
        if(MHalDvfsSetOverTempDebugOffset)
            MHalDvfsSetOverTempDebugOffset(0,set_offset);
        return count;
    }
    return -EINVAL;

}

static int dvfs_over_temp_debug_seq_show(struct seq_file *s, void *v)
{
    unsigned int offset=0;
    if(MHalDvfsGetOverTempDebugOffset)
        offset = MHalDvfsGetOverTempDebugOffset(0);
    seq_printf(s, "over-temp offset :%d:\n", offset);
    return 0;
}

static int dvfs_over_temp_debug_proc_open(struct inode *inode, struct file *file)
{
    if(atomic_read(&dvfs_over_temp_debug_proc_is_open))
        return -EACCES;

    atomic_set(&dvfs_over_temp_debug_proc_is_open, 1);

    return single_open(file, &dvfs_over_temp_debug_seq_show, NULL);
}

static int dvfs_over_temp_debug_proc_release(struct inode *inode, struct file * file)
{
    WARN_ON(!atomic_read(&dvfs_over_temp_debug_proc_is_open));
    atomic_set(&dvfs_over_temp_debug_proc_is_open, 0);
    return single_release(inode, file);
}

static const struct file_operations proc_dvfs_over_temp_debug_fileops = {
    .owner      = THIS_MODULE,
    .open       = dvfs_over_temp_debug_proc_open,
    .write      = dvfs_over_temp_debug_proc_write,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = dvfs_over_temp_debug_proc_release,
};

static int __init MDrvDvfsModuleInit(void)
{
    struct proc_dir_entry *entry;

    proc_mstar_dvfs_dir = proc_mkdir("mstar_dvfs", NULL);

    if (!proc_mstar_dvfs_dir)
        return -ENOMEM;

    entry = proc_create("voltage", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_voltage_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("frequency", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_frequency_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("temperature_info", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_temp_info_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("temperature", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_temp_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("sidd", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_sidd_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("osc", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_osc_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("boost_duration", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_boost_duration_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("power_type_cpu", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_cpu_power_type_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("power_type_core", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_core_power_type_fileops);
    if (!entry)
        goto fail;

    entry = proc_create("over_temp_debug", S_IRUSR | S_IWUSR, proc_mstar_dvfs_dir, &proc_dvfs_over_temp_debug_fileops);
    if (!entry)
        goto fail;

    return (int) platform_driver_register(&MstarDvfsDriver);

fail:
    return -ENOMEM;
}

//=================================================================================================
static void __exit MDrvDvfsModuleExit(void)
{
    remove_proc_entry("temperature_info", proc_mstar_dvfs_dir);
    remove_proc_entry("voltage", proc_mstar_dvfs_dir);
    remove_proc_entry("temperature", proc_mstar_dvfs_dir);
    remove_proc_entry("sidd", proc_mstar_dvfs_dir);
    remove_proc_entry("osc", proc_mstar_dvfs_dir);
    remove_proc_entry("frequency", proc_mstar_dvfs_dir);
    remove_proc_entry("boost_duration", proc_mstar_dvfs_dir);
    remove_proc_entry("power_type_cpu", proc_mstar_dvfs_dir);
    remove_proc_entry("power_type_core", proc_mstar_dvfs_dir);
    remove_proc_entry("over_temp_debug", proc_mstar_dvfs_dir);
    remove_proc_entry("mstar_dvfs", NULL);

    platform_driver_unregister(&MstarDvfsDriver);
}


//=================================================================================================
static int __init DVFS_auto_measurement(char *str)
{
    if(strcmp(str, "enable") == 0)
    {
        printk("\033[32m auto_measurement enable \033[0m\n");
        bootarg_auto_measurement= 1;
    }
    else
    {
        printk("\033[32m auto_measurement disable \033[0m\n");
        bootarg_auto_measurement = 0;
    }
    MHalDvfsSetAutoMeasurement(bootarg_auto_measurement);
    return 0;
}
early_param("DVFS_MEASURE", DVFS_auto_measurement);

//=================================================================================================
module_init(MDrvDvfsModuleInit);
module_exit(MDrvDvfsModuleExit);

module_param(mstar_dvfs_debug, uint, 0644);
MODULE_PARM_DESC(mstar_debug, "Debug for dvfs");
module_param(mstar_dvfs_info, uint, 0644);
MODULE_PARM_DESC(mstar_info, "Info for dvfs");

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("Mstar DVFS Driver");
MODULE_LICENSE("GPL");

//=================================================================================================
