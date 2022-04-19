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
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/cputype.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/ctype.h>

#include <asm/setup.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>

#include <include/mstar/mstar_chip.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
#include <trace/events/sched.h>
#include <linux/seq_file.h>
#if defined(CONFIG_MP_BENCHMARK_ACCEL87) || defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
#include "mdrv_benchmark_optimize.h"
#endif
#if defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
#include <linux/kthread.h>
#endif

#include "./include/mach/hardware.h"
#include "./include/mach/platform.h"

#include <chip_dvfs_calibrating.h>
#include <linux/version.h>

#ifdef CONFIG_MSTAR_DVFS
#ifndef __MDRV_DVFS_H__
#include "mdrv_dvfs.h"
#include <mach/io.h>
#endif
#endif

#include "mdrv_CPU_cluster_calibrating.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/sched/task.h>
#include <linux/sched/signal.h>
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,18,40)
#include <linux/slab.h>
#include <linux/pm_opp.h>
#include <opp.h>
#include "mhal_dvfs.h"
#endif

#define DVFS_DEBUG KERN_DEBUG
#define DVFS_LOCK_DEBUG KERN_DEBUG

#define BOOST_AGING_TIMEOUT_IN_MS 600000 // 10(mins) * 60 * 1000
#define BOOST_DURATION_CHECK_PERIOD 20 /* check boost duration per 20ms */
#define MAX_DMSG_WRITE_BUFFER	64
#define BENCH_BOOST_CLIENT_ID 32
#define LAUNCH_BOOST_CLIENT_ID 128
#define APP_BOOST_CLIENT_ID 64
#define CM_ID  	IO_ADDRESS(INTEGRATOR_HDR_ID)
#define CM_OSC	IO_ADDRESS(INTEGRATOR_HDR_OSC)
#define CM_STAT IO_ADDRESS(INTEGRATOR_HDR_STAT)
#define CM_LOCK IO_ADDRESS(INTEGRATOR_HDR_LOCK)
#define FREQ_CHECK_LINK_TIME (HZ)
#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
#define DVFS_BLOCK_NONMASTER_CORE 0
#else
#define DVFS_BLOCK_NONMASTER_CORE 1
#endif
struct cpufreq_frequency_table *mstar_freq_table[CONFIG_NR_CPUS];

#define DVFS_LOG_DEBUG(x, args...)                      {if (mstar_debug) \
                                                            printk(KERN_DEBUG x, ##args);}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,18,40)
#define cpumask_of_cpu *cpumask_of

static int set_cpus_allowed(struct task_struct *p, cpumask_t new_mask)
{
	return set_cpus_allowed_ptr(p, &new_mask);
}
#endif
extern unsigned launch_boost_enable;
struct task_struct *boost_duration_check_tsk = NULL;
extern int halTotalClusterNumber;
extern void change_interval(unsigned int old_freq, unsigned int new_freq);
extern struct cpufreq_policy *cpufreq_cpu_data;
#ifndef CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
extern void mstar_update_sched_clock(void);
extern unsigned int SC_MULT;
extern unsigned int SC_SHIFT;
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
extern int getClusterMainCpu(unsigned int cpu);
extern U32 getFreqRiuAddr(unsigned int cpu);
extern unsigned int get_cpu_midr(int cpu);

unsigned int over_temperature_mode[CONFIG_NR_CPUS];
/* str usage */
atomic_t disable_dvfs = ATOMIC_INIT(0);
atomic_t disable_dvfs_debug = ATOMIC_INIT(1);

/* reboot usage */
atomic_t disable_dvfs_reboot = ATOMIC_INIT(0);

/* proc usage */
//static DEFINE_SPINLOCK(set_freq_lock);
static atomic_t proc_is_open = ATOMIC_INIT(0);
static struct cpufreq_driver integrator_driver;
unsigned int current_frequency = 0;
unsigned int register_frequency = 0;
static atomic_t t_sensor_proc_is_open = ATOMIC_INIT(0);


/* on_demand handshake usage */
static atomic_t on_demand_handshake_is_open = ATOMIC_INIT(0);
DECLARE_WAIT_QUEUE_HEAD(DVFS_on_demand_event_waitqueue);
DECLARE_WAIT_QUEUE_HEAD(DVFS_on_demand_event_waitqueue_userspace_return);
EXPORT_SYMBOL(DVFS_on_demand_event_waitqueue);
EXPORT_SYMBOL(DVFS_on_demand_event_waitqueue_userspace_return);
DEFINE_SEMAPHORE(DVFS_on_demand_event_SEM);
DEFINE_SEMAPHORE(DVFS_disable_SEM);

/* this is for muji test cmdq (we don not use SAR to adjust voltage) */
unsigned int start_userspace_ondemand_handshake = 0;
#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
unsigned int using_kernel_i2c_interface = 1;
#else
unsigned int using_kernel_i2c_interface = 0;
#endif
unsigned int ready_to_change_cpufreq[CONFIG_NR_CPUS] = {0};
unsigned int ready_to_change_voltage[CONFIG_NR_CPUS] = {0};
unsigned int ready_to_change_voltage_type[CONFIG_NR_CPUS] = {0};
unsigned int ready_to_change_cpu[CONFIG_NR_CPUS] = {0};
unsigned int ready_to_change_cluster_id[CONFIG_NR_CPUS] = {0};
unsigned int change_cnt[CONFIG_NR_CPUS] = {0};
unsigned int finished_change_cnt[CONFIG_NR_CPUS] = {0};
int voltage_change_result[CONFIG_NR_CPUS] = {0};
bool forcibly_set_target_flag[CONFIG_NR_CPUS] = {0};

unsigned int bootarg_dvfs_disable = 0;
unsigned int bootarg_dvfs_t_sensor_disable = 0;
static unsigned int mstar_debug = 0;
static unsigned int mstar_info = 0;

struct mstar_cpufreq_policy ondemand_timer[CONFIG_NR_CPUS];
EXPORT_SYMBOL(ondemand_timer);
#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE)
static void Mdrv_CPU_T_sensor_Check_callback(unsigned long value);
#if !defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
static struct timer_list Check_T_sensor_timer;
static DEFINE_SPINLOCK(T_sensor_lock);
#endif
#else
static struct timer_list Check_Freq_timer;
static void Mdrv_CPU_Freq_Check_callback(unsigned long value);
#endif

#if defined(CONFIG_MP_DVFS_CPUHOTPLUG_USE_ONLINE_CPU_MAX_LOAD)
DEFINE_MUTEX(mstar_cpuload_lock);
unsigned int mstar_cpu_load_freq[CONFIG_NR_CPUS];
#endif

/* Define cluster-specified locks */
DEFINE_MUTEX(mstar_cpufreq_lock_Little);
DEFINE_MUTEX(mstar_cpufreq_lock_Big);
DEFINE_MUTEX(mstar_voltagesetup);
#if defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
unsigned int t_sensor_monitor_owner_cpu = 0;		/* specify the cpu owning the t_sentor thread */
#endif

static DEFINE_MUTEX(boost_client_mutex);
struct boost_client
{
	unsigned long client_id;
	unsigned long cpu_freq_in_khz;
	unsigned long boost_duration_ms;
	unsigned long jiffies_duration_starttime_ms;
	struct list_head list_head;
};
static struct boost_client *show_boost_client(int cpu_id);
static struct boost_client *find_boost_client(unsigned long client_id, int cpu_id);
static struct boost_client *update_boost_duration(unsigned long client_id, int set_cpu, unsigned int set_duration);
static void check_boost_duration(int cpu_id);

typedef struct {
        unsigned long clk;              // khz
        unsigned long volt;
} _mstar_opp_table;

typedef struct {
        _mstar_opp_table per_cpu_table[DVFS_FREQ_LEVEL_MAX_INDEX];
} mstar_opp_table;

#ifdef CONFIG_MSTAR_DVFS_DEBUG
extern int dvfs_debug_init(void);
#endif

static void Mdrv_CpuFreq_Lock(int cpu_id, char *caller)
{
	if(ondemand_timer[cpu_id].cluster == 0)
	{
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] Locking mstar_cpufreq_lock_Little...\033[m\n", caller);
		mutex_lock(&mstar_cpufreq_lock_Little);
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] Locking mstar_cpufreq_lock_Little... done\033[m\n", caller);
	}
	else if(ondemand_timer[cpu_id].cluster == 1)
	{
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] Locking mstar_cpufreq_lock_Big...\033[m\n", caller);
		mutex_lock(&mstar_cpufreq_lock_Big);
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] Locking mstar_cpufreq_lock_Big... done\033[m\n", caller);
	}
	else
	{
		printk(KERN_ERR "\033[31m[Error] Function = %s, Line = %d, cpu_id: %d, can not get cluster\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id);
		BUG_ON(1);
	}
}

static void Mdrv_CpuFreq_UnLock(int cpu_id, char *caller)
{
	if(ondemand_timer[cpu_id].cluster == 0)
	{
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] UnLocking mstar_cpufreq_lock_Little...\033[m\n", caller);
		mutex_unlock(&mstar_cpufreq_lock_Little);
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] UnLocking mstar_cpufreq_lock_Little done...\033[m\n", caller);
	}
	else if(ondemand_timer[cpu_id].cluster == 1)
	{
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] UnLocking mstar_cpufreq_lock_Big...\033[m\n", caller);
		mutex_unlock(&mstar_cpufreq_lock_Big);
		//printk(DVFS_LOCK_DEBUG "\033[35m[Single: %s] UnLocking mstar_cpufreq_lock_Big done...\033[m\n", caller);
	}
	else
	{
		printk(KERN_ERR "\033[31m[Error] Function = %s, Line = %d, cpu_id: %d, can not get cluster\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id);
		BUG_ON(1);
	}
}

void Mdrv_CpuFreq_All_Lock(char *caller)
{
	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] Locking mstar_cpufreq_lock_Big...\033[m\n", caller);
	mutex_lock(&mstar_cpufreq_lock_Big);
	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] Locking mstar_cpufreq_lock_Big done...\033[m\n", caller);

	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] Locking mstar_cpufreq_lock_Little...\033[m\n", caller);
	mutex_lock(&mstar_cpufreq_lock_Little);
	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] Locking mstar_cpufreq_lock_Little done...\033[m\n", caller);
}

void Mdrv_CpuFreq_All_UnLock(char *caller)
{
	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] UnLocking mstar_cpufreq_lock_Little...\033[m\n", caller);
	mutex_unlock(&mstar_cpufreq_lock_Little);
	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] UnLocking mstar_cpufreq_lock_Little done...\033[m\n", caller);

	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] UnLocking mstar_cpufreq_lock_Big...\033[m\n", caller);
	mutex_unlock(&mstar_cpufreq_lock_Big);
	//printk(DVFS_LOCK_DEBUG "\033[35m[ALL: %s] UnLocking mstar_cpufreq_lock_Big done...\033[m\n", caller);
}

unsigned int get_freq(unsigned int cpu)
{
	return MDrvDvfsGetCpuFreq(cpu) * 1000;
}

/*
 * Validate the speed policy.
 */
static int integrator_verify_policy(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);

	return 0;
}

static unsigned int integrator_get(unsigned int cpu)
{
	cpumask_t cpus_allowed;
	unsigned int current_freq;

	cpus_allowed = current->cpus_allowed;
	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());

	current_freq = get_freq(cpu);

	set_cpus_allowed(current, cpus_allowed);

	return current_freq;
}

ssize_t dvfs_boost_duration_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[MAX_DMSG_WRITE_BUFFER];
	unsigned int set_cpu = 0;
	unsigned int garbage = 0;
	unsigned long idx;
	unsigned long client_id = 0;
	unsigned int set_duration = 0;
	struct boost_client *bc = NULL;

	if (!count)
		return count;

	if (count >= MAX_DMSG_WRITE_BUFFER)
		count = MAX_DMSG_WRITE_BUFFER - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (sscanf(buffer, "%d %lu %d", &set_cpu, &idx, &garbage) == 3)
	{
		return -EINVAL;
	}
	else if (sscanf(buffer,"%d %lu", &set_cpu, &idx) == 2)  // multi-cluster version, can specify cpu, as set_cpu; the idx is combined with client_id and boost_duration
	{
		if (set_cpu >= CONFIG_NR_CPUS)
		{
			printk("\033[35mFunction = %s, Line = %d, multi-cluster version, but the set_cpu:%d is over CONFIG_NR_CPUS:%d\033[m\n", __PRETTY_FUNCTION__, __LINE__, set_cpu, CONFIG_NR_CPUS);
			return -EINVAL;
		}

		// idx[29:22] is the client id, the same as /proc/CPU_calibrating
		client_id = (idx >> 22) & 0xff;

		// idx[21:0] is the cpu boost_duration in ms
		set_duration = idx & 0x3FFFFF;

		if (set_duration > BOOST_AGING_TIMEOUT_IN_MS)
		{
			printk("\033[35mFunction = %s, Line = %d, multi-cluster version, but the set_duration:%d is over BOOST_AGING_TIMEOUT_IN_MS:%d\033[m\n", __PRETTY_FUNCTION__, __LINE__, set_duration, BOOST_AGING_TIMEOUT_IN_MS);
			return -EINVAL;
		}
		set_duration = BOOST_DURATION_CHECK_PERIOD * (set_duration/BOOST_DURATION_CHECK_PERIOD);

		/* find the boost_client, and update the boost_duration */
		bc = update_boost_duration(client_id, set_cpu, set_duration);
		if(bc == NULL)
			return -EINVAL;

		return count;
	}

	printk("\033[35mFunction = %s, Line = %d, unknown setting!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
    return -EINVAL;
}

int dvfs_boost_duration_seq_show(struct seq_file *s, void *v)
{
	int i = 0;

	for_each_online_cpu(i)
	{
		if(i != ondemand_timer[i].cluster_m)
			continue;

		show_boost_client(i);
	}

	return TRUE;
}

static int boost_duration_thread(void* arg)
{
	int i;

	while(1)
	{
		msleep(BOOST_DURATION_CHECK_PERIOD);

		// check if delete a boost_clients using individual boost_duration
		for_each_online_cpu(i)
		{
			if(i != ondemand_timer[i].cluster_m)
				continue;

			check_boost_duration(i);
		}
	}
	return 0;
}

#if defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
static int t_sensor_thread(void* arg)
{
	while(1)
	{
		msleep(1000);

		// this lock should be cluster-specified, or Little Cluster will hang here while Big Cluster hang. we disable this lock first
		// we need to enable this lock, due to T_sensor_check and on_demand will both do MDrvDvfsVoltageSetup(), and thus the change_cnt will not equal!!
		// Now, we use mstar_cpufreq_lock_Little here, because we will only let t_sensor_thread on Little_Cluster
		Mdrv_CpuFreq_Lock(t_sensor_monitor_owner_cpu, (char *)__FUNCTION__);
		if(atomic_read(&disable_dvfs) != 1)
		{
			Mdrv_CPU_T_sensor_Check_callback(0);
		}
		Mdrv_CpuFreq_UnLock(t_sensor_monitor_owner_cpu, (char *)__FUNCTION__);
	}

	return 0;
}
#endif

mstar_opp_table mstar_init_opp_table[CONFIG_NR_CPUS];

static int integrator_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret, opp_cnt, opp_err;
	struct task_struct *t_sensor_tsk;
	int cpu_id;
	struct device *per_cpu_dev = NULL;

	cpu_id = get_cpu();
	put_cpu();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	policy->freq_table = mstar_freq_table[policy->cpu];
	/*
	 * the reason why we not use cpufreq_generic_init() is because
	 * we dont want to set cpumask, this will let policy->cpus be all online cpu
	*/
	ret = cpufreq_table_validate_and_sort(policy);
	if (ret) {
		pr_err("\033[31m%s(%d): invalid frequency table: %d\033[m\n",
				__func__, __LINE__, ret);
		BUG_ON(1);
	}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
	/* the reason why we not use cpufreq_generic_init() is because we dont want to set cpumask, this will let policy->cpus be all online cpu */
	ret = cpufreq_table_validate_and_show(policy, mstar_freq_table[policy->cpu]);
	if(ret)
	{
		printk("\033[31mFunction = %s, Line = %d, invalid frequency table: %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, ret);
		printk("\033[31mFunction = %s, Line = %d, invalid frequency table: %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, ret);
		printk("\033[31mFunction = %s, Line = %d, invalid frequency table: %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, ret);
		printk("\033[31mFunction = %s, Line = %d, invalid frequency table: %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, ret);

		BUG_ON(1);
	}
#else
	ret = cpufreq_frequency_table_cpuinfo(policy, cpufreq_frequency_get_table(policy->cpu));
#endif
	/* re-set default policy and cpuinfo, after cpufreq_frequency_table_cpuinfo() or cpufreq_table_validate_and_show() */
	policy->cur = get_freq(policy->cpu);
	policy->min = policy->cpuinfo.min_freq = CONFIG_DVFS_CPU_CLOCK_MIN(policy->cpu);
#if defined(CONFIG_DVFS_DETACH_BOOST_FREQ_AND_MAX_FREQ)
        policy->max = policy->cpuinfo.max_freq = CONFIG_DVFS_CPU_CLOCK_TABLE_MAX(policy->cpu);
#else
        policy->max = policy->cpuinfo.max_freq = CONFIG_DVFS_CPU_IRBOOST_CLOCK(policy->cpu); //We only allow max freq from kernel is IR boost level, not system maximal freq
#endif
	policy->cpuinfo.transition_latency = TRANSITION_LATENCY;        /* 1 ms, assumed, you can do this by cpufreq_generic_init(), while without cpu_hotplug */

	ondemand_timer[policy->cpu].policy = policy;
	ondemand_timer[policy->cpu].cluster = getCpuCluster(policy->cpu);
	ondemand_timer[policy->cpu].cluster_m = getClusterMainCpu(policy->cpu);
	ondemand_timer[policy->cpu].freq_riu = getFreqRiuAddr(policy->cpu);
	atomic_set(&ondemand_timer[policy->cpu].echo_calibrating_freq, 0);
	INIT_LIST_HEAD(&(ondemand_timer[policy->cpu].boost_head));
    printk("\033[31mFunction = %s, Line = %d, cluster = %d, policy->cpu = %d, cluster_m: %d, max_freq: %u, min_freq: %u\033[m\n",
		__PRETTY_FUNCTION__, __LINE__, ondemand_timer[policy->cpu].cluster, policy->cpu, ondemand_timer[policy->cpu].cluster_m, policy->max, policy->min);

	/* move to here to let the timer works on master cpu of cluster */
	if(policy->cpu == ondemand_timer[policy->cpu].cluster_m) // means it is master CPU of cluster
	{
		atomic_set(&ondemand_timer[policy->cpu].ac_str_cpufreq, policy->cur);
		printk("\033[31mFunction = %s, Line = %d, set cpu%d ac_str_cpufreq: %d KHz\033[m\n", __PRETTY_FUNCTION__, __LINE__, policy->cpu, ondemand_timer[policy->cpu].ac_str_cpufreq.counter);
#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
#if !defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
    	init_timer(&Check_T_sensor_timer);
    	Check_T_sensor_timer.data = 1;
    	Check_T_sensor_timer.function = Mdrv_CPU_T_sensor_Check_callback;
    	Check_T_sensor_timer.expires = jiffies + FREQ_CHECK_LINK_TIME;  // 1 second do once
    	add_timer(&Check_T_sensor_timer);
#else
		if(policy->cpu == t_sensor_monitor_owner_cpu)
		{
			if (!ondemand_timer[policy->cpu].t_sensor_tsk) {
				t_sensor_tsk = kthread_create(t_sensor_thread, NULL, "T_sensor_Check");

				kthread_bind(t_sensor_tsk, policy->cpu);

				if (IS_ERR(t_sensor_tsk)) {
					printk("create kthread for t_sensor temperature observation fail\n");
					ret = PTR_ERR(t_sensor_tsk);
					t_sensor_tsk = NULL;
					goto out;
				}else
					wake_up_process(t_sensor_tsk);

				ondemand_timer[policy->cpu].t_sensor_tsk = t_sensor_tsk;
				printk("\033[31mCreate T sensor on cpu:%d, set str freq to %dkhz\033[m\n", policy->cpu, atomic_read(&ondemand_timer[policy->cpu].ac_str_cpufreq));
			}
			else {
				wake_up_process(ondemand_timer[policy->cpu].t_sensor_tsk);
				printk("\033[31mWake up T sensor on cpu:%d\033[m\n", policy->cpu);
			}

			if(!boost_duration_check_tsk)
			{
				boost_duration_check_tsk = kthread_create(boost_duration_thread, NULL, "Boost_Duration_Check");
				kthread_bind(boost_duration_check_tsk, policy->cpu);

				if (IS_ERR(boost_duration_check_tsk))
				{
					printk("create kthread for boost duration check fail\n");
					ret = PTR_ERR(boost_duration_check_tsk);
					boost_duration_check_tsk = NULL;
					BUG_ON(1);
				}
				else
					wake_up_process(boost_duration_check_tsk);
			}
		}
#endif
#endif // CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
	}

	return 0;
#if defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
out:
    return ret;
#endif
}

typedef struct _IO_CPU_calibrating_INFO
{
	char* MESSAGE_BUFF;
	char CPUID;
	char  MESSAGE_LEN;
	int MID;
}IO_CPU_calibrating_INFO;

//static long CPU_calibrating_proc_ioctl(struct file *filp, unsigned int cmd, IO_CPU_calibrating_INFO* message_buf)
static long CPU_calibrating_proc_ioctl(struct file *filp, unsigned int cmd, unsigned long message_buf)
{
	IO_CPU_calibrating_INFO* bb = (IO_CPU_calibrating_INFO *)message_buf;
	char usr_buf[256];

	if (copy_from_user(usr_buf, bb->MESSAGE_BUFF, bb->MESSAGE_LEN))
	{
		printk(KERN_ERR "setgreq_proc_ioctl error\n");
		return -EFAULT;
	}

	return 0;
}

int _CPU_calibrating_proc_write(const unsigned long, const unsigned long, int);
int __CPU_calibrating_proc_write(const unsigned long, unsigned int);
void write_cpufreq_to_RIU(const unsigned long, int);
static unsigned long read_echo_calibrating_freq(int);
static void write_echo_calibrating_freq(const unsigned long, int);
bool is_any_boost_client_running(int);
static int del_boost_client(unsigned long, int);
static struct boost_client *find_boost_client_with_highest_priority(int cpu_id);

static ssize_t CPU_calibrating_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MAX_DMSG_WRITE_BUFFER];
	unsigned long idx;
	unsigned long cpu_freq_in_khz = 0;
	unsigned long client_id = 0;
	unsigned int set_cpu = 0;
	unsigned int garbage = 0;

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
    {
	    idx = 3;
    }
    else if (sscanf(buffer, "%lu %d %d", &idx, &set_cpu, &garbage) == 3)
    {
       return -EINVAL;
    }
    else if (sscanf(buffer,"%d %lu", &set_cpu, &idx) == 2)	// multi-cluster version, can specify cpu, as set_cpu
    {
        if (set_cpu >= CONFIG_NR_CPUS)
            return -EINVAL;

        // idx[29:22] is the client id
	    client_id = (idx >> 22) & 0xff;

	    // idx[21:0] is the cpu frequency in Khz
	    cpu_freq_in_khz = idx & 0x3FFFFF;

		if (mstar_debug) {
			printk("\033[34m%s %d: set cpu: %d, client_id: %lu, cpu_freq_in_khz: %lu\033[m\n", __PRETTY_FUNCTION__, __LINE__, set_cpu, client_id, cpu_freq_in_khz);
		}
        _CPU_calibrating_proc_write(client_id, cpu_freq_in_khz, set_cpu);

		return count;
    }
	else if (strict_strtol(buffer, 0, &idx) == 0)  //Force change str to decimal conversion (str, base, *converted_num), base is hex, decimal, or ...
	{
		// idx[29:22] is the client id
		client_id = (idx >> 22) & 0xff;

		// idx[21:0] is the cpu frequency in Khz
		cpu_freq_in_khz = idx & 0x3FFFFF;

		for (set_cpu = 0; set_cpu < CONFIG_NR_CPUS; set_cpu ++) {
			if (mstar_debug) {
				printk("\033[34m%s %d: set cpu: %d, client_id: %lu, cpu_freq_in_khz: %lu\033[m\n", __PRETTY_FUNCTION__, __LINE__, set_cpu, client_id, cpu_freq_in_khz);
			}
			_CPU_calibrating_proc_write(client_id, cpu_freq_in_khz, set_cpu);
		}

		return count;
    }

	return -EINVAL;
}

static int CPU_calibrating_seq_show(struct seq_file *s, void *v)
{
	int i, j;
	//unsigned int freq = 0;

	if (mstar_info) {
		for (i = 0; i < halTotalClusterNumber; i ++) {
			for_each_online_cpu(j)
			{
				//freq = query_frequency(i);
				if (ondemand_timer[j].cluster == i) {
#ifdef CONFIG_ARM64
					seq_printf(s, "CPU_part:%x: max_freq: %u | sys_freq: %u | min_freq:%u \n", MIDR_PARTNUM(get_cpu_midr(j)), CONFIG_DVFS_CPU_CLOCK_MAX(j), CONFIG_DVFS_CPU_IRBOOST_CLOCK(j), CONFIG_DVFS_CPU_CLOCK_MIN(j));
#else
					seq_printf(s, "CPU_part:%x: max_freq: %u | sys_freq: %u | min_freq:%u \n", (get_cpu_midr(i) & 0xFFF0), CONFIG_DVFS_CPU_CLOCK_MAX(j), CONFIG_DVFS_CPU_IRBOOST_CLOCK(j), CONFIG_DVFS_CPU_CLOCK_MIN(j));
#endif
					break;
				}
			}
		}
	}

	if (mstar_debug) {
		printk("Total %d %s\n", halTotalClusterNumber, (halTotalClusterNumber==1? "cluster": "clusters"));
		//get_online_cpus();
		for (i = 0; i < halTotalClusterNumber; i ++)
		{
			printk("Cluster %d CPU:", i);
			for_each_online_cpu(j)
			{
				if (ondemand_timer[j].cluster == i)
				printk(" %d", ondemand_timer[j].policy->cpu);
			}
			for_each_online_cpu(j)
			{
				if (ondemand_timer[j].cluster == i) {
					show_boost_client(ondemand_timer[j].policy->cpu);
					break;
				}
			}
			printk("\n");
		}
		//put_online_cpus();
	}
	return 0;
}

static struct boost_client *show_boost_client(int cpu_id)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
    struct list_head *boost_client_head;
    int m = ondemand_timer[cpu_id].cluster_m;
    boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);
	if (list_empty(boost_client_head)) {
	}
	else {
		printk(" -->[boost client for cpu: %d]\n", cpu_id);
		list_for_each_entry(i, boost_client_head, list_head) {
			printk("(client id: %lu, bootst_freq: %lukhz, boost_duration_ms: %lums, start_jiffies: %lums)\n", i->client_id, i->cpu_freq_in_khz, i->boost_duration_ms, i->jiffies_duration_starttime_ms);
		}
		printk("End\n");
	}
	mutex_unlock(&boost_client_mutex);

	return bc;
}

bool is_any_boost_client_running(int cpu_id)
{
    bool i;
    struct list_head *boost_client_head;
    int m = ondemand_timer[cpu_id].cluster_m;
    boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);
	i = !list_empty(boost_client_head);
	mutex_unlock(&boost_client_mutex);
#if 0
    if (i)
        show_boost_client(cpu_id);
#endif

	return i;
}

static int add_boost_client(unsigned long client_id, unsigned long cpu_freq_in_khz, unsigned int cpu_id, void *arg)
{
	struct boost_client *bc = NULL;
	int m = ondemand_timer[cpu_id].cluster_m;

	if ((bc = kmalloc(sizeof(*bc), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	bc->client_id = client_id;
	bc->cpu_freq_in_khz = cpu_freq_in_khz;

	mutex_lock(&boost_client_mutex);
	list_add(&bc->list_head, &(ondemand_timer[m].boost_head));
	bc->boost_duration_ms = 0;
	bc->jiffies_duration_starttime_ms = 0;
	mutex_unlock(&boost_client_mutex);

	return 0;
}

static struct boost_client *find_boost_client(unsigned long client_id, int cpu_id)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
    struct list_head *boost_client_head;
    int m = ondemand_timer[cpu_id].cluster_m;
    boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);
	list_for_each_entry(i, boost_client_head, list_head) {
		if (i->client_id == client_id) {
			bc = i;
		}
	}
	mutex_unlock(&boost_client_mutex);

	return bc;
}

static struct boost_client *update_boost_duration(unsigned long client_id, int set_cpu, unsigned int set_duration)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
    struct list_head *boost_client_head;
    int m = ondemand_timer[set_cpu].cluster_m;
    boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);
	/* find the boost_client, and update the boost_duration */
	list_for_each_entry(i, boost_client_head, list_head) {
		if (i->client_id == client_id) {
			bc = i;
		}
	}

	if(bc == NULL)
	{
		printk("\033[35m[update_boost_duration: set duration to %dms] the set_cpu:%d, client_id:%lu is not existed\033[m\n", set_duration, set_cpu, client_id);
	}
	else
	{
		//printk("\033[35mthe ori_bc_info: bc_client_id: %lu, cpu_freq_in_khz: %lukhz, boost_duration_ms: %lums\033[m\n", bc->client_id, bc->cpu_freq_in_khz, bc->boost_duration_ms);
		//printk("\033[35mfor cpu:%d, now set the duration to %dms\033[m\n", set_cpu, set_duration);
		bc->boost_duration_ms = set_duration;
		bc->jiffies_duration_starttime_ms = jiffies_to_msecs(jiffies);	// once user set the duration, we then start to calculate the diff_jiffies, use jiffies_duration_starttime to check if need del boost_client @ t-sensor callback
	}

	mutex_unlock(&boost_client_mutex);
	return bc;
}

static void check_boost_duration(int cpu_id)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
	struct boost_client **delete_list;
	int delete_cnt, j;
    struct list_head *boost_client_head;
	int first_print;
    int m = ondemand_timer[cpu_id].cluster_m;
    boost_client_head = &(ondemand_timer[m].boost_head);
    if ((delete_list =kmalloc(256 * sizeof(struct boost_client *), GFP_KERNEL)) == NULL)
    {
        pr_notice_once("[%s][%d] warning: kmalloc fail!!!\n",__FUNCTION__,__LINE__);
        return;
    }
	delete_cnt = 0;
	first_print = 0;
	mutex_lock(&boost_client_mutex);

	if (!list_empty(boost_client_head))
	{
		list_for_each_entry(i, boost_client_head, list_head) {
			if(i->jiffies_duration_starttime_ms > 0)
			{
				if( (jiffies_to_msecs(jiffies) - i->jiffies_duration_starttime_ms) > i->boost_duration_ms )
				{
					if(first_print == 0)
					{
						printk(" -->[cpu%d check_boost_duration: delete client]\n", cpu_id);
						first_print = 1;
					}
					printk("(client id: %lu, boost_freq: %lukhz, boost_duration_ms: %lums, start_time: %lums, current_jiffies: %ums)\n",
						i->client_id, i->cpu_freq_in_khz, i->boost_duration_ms, i->jiffies_duration_starttime_ms, jiffies_to_msecs(jiffies));

					// delete this client_id, add it to delete_list[](we will later remove from boost_client_head and free it)
					delete_list[delete_cnt] = i;
					delete_cnt++;
				}
			}
		}
		if(first_print == 1)
			printk("End\n\n");
	}

	for(j = 0; j < delete_cnt; j++)
	{
		list_del(&delete_list[j]->list_head);
		kfree(delete_list[j]);
	}

	if(delete_cnt)	// having remove a client_id, so we need to update the policy->max
	{
		if(!list_empty(boost_client_head))
		{
			bc = list_entry(boost_client_head->next, struct boost_client, list_head);
			list_for_each_entry(i, boost_client_head, list_head)
			{
				if(i->client_id < bc->client_id)
					bc = i;
			}
			write_echo_calibrating_freq(bc->cpu_freq_in_khz, cpu_id);
		}
	}

	mutex_unlock(&boost_client_mutex);
	kfree(delete_list);
	return;
}

static int del_boost_client(unsigned long client_id, int cpu_id)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
	struct list_head *boost_client_head;
	int m = ondemand_timer[cpu_id].cluster_m;
	boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);

	//serach client
	list_for_each_entry(i, boost_client_head, list_head) {
		if (i->client_id == client_id) {
			bc = i;
		}
	}

	if(bc == NULL){
		mutex_unlock(&boost_client_mutex);
		printk(KERN_WARNING "[dvfs_boost] this client is not running\n");
		return 0;
	}

	// remove the client node
	list_del(&bc->list_head);
	mutex_unlock(&boost_client_mutex);

	kfree(bc);

	return 0;
}

bool del_all_boost_client(int cpu_id)
{
	struct boost_client *bc = NULL;
	struct boost_client *bc2 = NULL;
    int m = ondemand_timer[cpu_id].cluster_m;
    struct list_head *boost_client_head;
    boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);
	list_for_each_entry_safe(bc, bc2, boost_client_head, list_head) {
		list_del(&bc->list_head);
		kfree(bc);
	}
	mutex_unlock(&boost_client_mutex);

	return true;
}
EXPORT_SYMBOL(del_all_boost_client);

static bool is_boost_client_running(unsigned long client_id, int cpu_id)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
	struct list_head *boost_client_head;
	int m = ondemand_timer[cpu_id].cluster_m;
	boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);

	//serach client
	list_for_each_entry(i, boost_client_head, list_head) {
		if (i->client_id == client_id) {
			bc = i;
		}
	}
	mutex_unlock(&boost_client_mutex);

	return (bc != NULL) ? true : false;
}

static struct boost_client *find_boost_client_with_highest_priority(int cpu_id)
{
	struct boost_client *i = NULL;
	struct boost_client *bc = NULL;
    struct list_head *boost_client_head;
    int m = ondemand_timer[cpu_id].cluster_m;
    boost_client_head = &(ondemand_timer[m].boost_head);

	mutex_lock(&boost_client_mutex);
	bc = list_entry(boost_client_head->next, struct boost_client, list_head);
	list_for_each_entry(i, boost_client_head, list_head) {
		if(i->client_id < bc->client_id) {
			bc = i;
		}
	}
	mutex_unlock(&boost_client_mutex);

	return bc;
}

static unsigned long read_echo_calibrating_freq(int cpu_id)
{
    return (unsigned long)atomic_read(&(ondemand_timer[cpu_id].echo_calibrating_freq));
}

static void write_echo_calibrating_freq(const unsigned long value, int cpu_id)
{
    atomic_set(&(ondemand_timer[cpu_id].echo_calibrating_freq), value);
}

#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
extern struct cpu_scaling_list *bench_boost_list;
extern struct cpu_scaling_list *app_boost_list;
void bench_boost_check(void);
void app_boost_check(void);
extern struct mutex bench_boost_list_lock;
extern struct mutex app_boost_list_lock;
#endif

#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
atomic_t launch_boost_duration = ATOMIC_INIT(5000);
static struct work_struct launch_boost_start_work;
static struct delayed_work launch_boost_stop_work;
static void launch_boost_start(struct work_struct*);
static void launch_boost_stop(struct work_struct*);
static DECLARE_WORK(launch_boost_start_work, launch_boost_start);
static DECLARE_DELAYED_WORK(launch_boost_stop_work, launch_boost_stop);
atomic_t launch_boost_running = ATOMIC_INIT(0);
void launch_boost_check(void);

void launch_boost_start(struct work_struct *w)
{
#if defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
    int i;
    printk(KERN_INFO "[launch_boost] starts\n");
    for_each_online_cpu(i)
        _CPU_calibrating_proc_write(LAUNCH_BOOST_CLIENT_ID, CONFIG_DVFS_CPU_IRBOOST_CLOCK(i), i);
#elif defined(CONFIG_MSTAR_CPU_calibrating)
    printk(KERN_INFO "[launch_boost] starts\n");
    _CPU_calibrating_proc_write(LAUNCH_BOOST_CLIENT_ID, CONFIG_DVFS_CPU_IRBOOST_CLOCK(0), 0);
#endif
    atomic_set(&launch_boost_running,1);
    return;
}

void launch_boost_stop(struct work_struct *w)
{
#if defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
    int i;
    printk(KERN_INFO "[launch_boost] stops\n");
    for_each_online_cpu(i)
        _CPU_calibrating_proc_write(LAUNCH_BOOST_CLIENT_ID, 0, i);
#elif defined(CONFIG_MSTAR_CPU_calibrating)
    printk(KERN_INFO "[launch_boost] stops\n");
    _CPU_calibrating_proc_write(LAUNCH_BOOST_CLIENT_ID, 0, 0);
#endif
    atomic_set(&launch_boost_running,0);
    return;
}
void launch_boost_check(void)
{
    if (launch_boost_enable && !atomic_read(&launch_boost_running)) {
        schedule_work_on(cpumask_first(cpu_online_mask), &launch_boost_start_work);
        schedule_delayed_work_on(cpumask_first(cpu_online_mask),&launch_boost_stop_work, msecs_to_jiffies(atomic_read(&launch_boost_duration)));
        launch_boost_enable = 0;
    }
    else if (launch_boost_enable && atomic_read(&launch_boost_running)) { // still running, extend duration
        schedule_work_on(cpumask_first(cpu_online_mask), &launch_boost_start_work);
        cancel_delayed_work(&launch_boost_stop_work);
        schedule_delayed_work_on(cpumask_first(cpu_online_mask),&launch_boost_stop_work, msecs_to_jiffies(atomic_read(&launch_boost_duration)));
        launch_boost_enable = 0;
    }
}
#endif

#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
static int ibench_boost_flag = 0;
void bench_boost_check(void)
{
    int run;
    struct cpu_scaling_list *now;
    struct cpu_scaling_list *zombie_node;
    struct cpu_scaling_list *prev = NULL;
    int i;
    struct pid *pid_now;
    struct task_struct *task_now;
    while(1)
    {
        run = 0;
        i = 0;
        prev = NULL;
        mutex_lock(&bench_boost_list_lock);
        now = bench_boost_list;
        rcu_read_lock();
        while(now != NULL)
        {
            pid_now = find_get_pid(now->target_id);
            task_now = get_pid_task(pid_now, PIDTYPE_PID);
            if (!task_now) {
                zombie_node = now;
                now = now->next;
                if (prev == NULL) { //cut head
                    bench_boost_list = zombie_node->next;
                    kfree(zombie_node);
                    zombie_node = NULL;
                } else {
                    prev->next = zombie_node->next;
                    kfree(zombie_node);
                    zombie_node = NULL;
                }
            } else {
                if (task_now->signal->oom_score_adj == 0/*now->tsk->signal->oom_score_adj==0*/)
                    run=1;
                prev = now;
                now = now->next;
                put_task_struct(task_now);
            }
            put_pid(pid_now);
        }
        rcu_read_unlock();
        if (run == 0 && ibench_boost_flag == 1) {
#ifdef CONFIG_MSTAR_DVFS
            for_each_online_cpu(i)
            {
                _CPU_calibrating_proc_write(BENCH_BOOST_CLIENT_ID,0,i);
                ibench_boost_flag = 0;
            }
#endif
        } else if (run == 1 && ibench_boost_flag == 0) {
#ifdef CONFIG_MSTAR_DVFS
            for_each_online_cpu(i)
            {
                _CPU_calibrating_proc_write(BENCH_BOOST_CLIENT_ID,MDrvDvfsQueryCpuClock(CONFIG_DVFS_MAX_CPU_CLOCK, i),i);
                ibench_boost_flag = 1;
            }
#endif
        }

        if (bench_boost_list == NULL)
        {
            mutex_unlock(&bench_boost_list_lock);
            break;
        }
        mutex_unlock(&bench_boost_list_lock);
        msleep(sensor_polling_time);
    }
}

static int iapp_boost_flag = 0;
void app_boost_check(void)
{
    int run;
    struct cpu_scaling_list *now = app_boost_list;
    struct cpu_scaling_list *zombie_node;
    struct cpu_scaling_list *prev = NULL;
    struct pid *pid_now;
    struct task_struct *task_now;
    int i;
    while (1)
    {
        run = 0;
        i = 0;
        prev = NULL;
        mutex_lock(&app_boost_list_lock);
        now = app_boost_list;
        rcu_read_lock();
        while(now != NULL)
        {
        	pid_now = find_get_pid(now->target_id);
            task_now = get_pid_task(pid_now, PIDTYPE_PID);
            if (!task_now) {
                zombie_node = now;
                now = now->next;
                if (prev == NULL) { //cut head
                    app_boost_list = zombie_node->next;
                    kfree(zombie_node);
                    zombie_node = NULL;
                } else {
                    prev->next = zombie_node->next;
                    kfree(zombie_node);
                    zombie_node = NULL;
                }
            } else {
                if (task_now->signal->oom_score_adj==0)
                    run=1;
                prev = now;
                now = now->next;
                put_task_struct(task_now);
           }
           put_pid(pid_now);
        }
        rcu_read_unlock();
        if (run == 0 && iapp_boost_flag == 1) {
#ifdef CONFIG_MSTAR_DVFS
            for_each_online_cpu(i)
            {
                _CPU_calibrating_proc_write(APP_BOOST_CLIENT_ID,0,i);
                iapp_boost_flag = 0;
            }
#endif
        } else if (run == 1 && iapp_boost_flag == 0){
#ifdef CONFIG_MSTAR_DVFS
            for_each_online_cpu(i)
            {
                _CPU_calibrating_proc_write(APP_BOOST_CLIENT_ID, MDrvDvfsQueryCpuClock(CONFIG_DVFS_IR_BOOTS_CPU_CLOCK, i), i);
                iapp_boost_flag = 1;
            }
#endif
        }
        if (app_boost_list == NULL)
        {
            mutex_unlock(&app_boost_list_lock);
            break;
        }
        mutex_unlock(&app_boost_list_lock);
        msleep(sensor_polling_time);
    }
}
#endif

int CPU_Calibrating_Boost_IsEnable(void)
{
	int bEnable = 0;
#if defined(CONFIG_MP_BENCHMARK_LAUNCH_BOOST)
	if (atomic_read(&launch_boost_running) == 1 )
		bEnable = 1;
#endif

#if defined(CONFIG_MP_BENCHMARK_CPU_DVFS_SCALING)
	if (ibench_boost_flag || iapp_boost_flag)
		bEnable = 1;
#endif
	return bEnable;
}
EXPORT_SYMBOL(CPU_Calibrating_Boost_IsEnable);


int _CPU_calibrating_proc_write(
		const unsigned long client_id,
		const unsigned long cpu_freq_in_khz,
                int cpu_id)
{
    struct cpufreq_policy *policy;
	int i;
    bool ret = 0;
    struct boost_client *bc = NULL;

#if (defined CONFIG_MP_DVFS_FORCE_USE_ONE_FREQ) || (defined CONFIG_MP_DVFS_FORCE_PINGO_TEST) || (defined CONFIG_MP_DVFS_VID_ONLY)
    return 0;
#endif

	/* if not cluster_master, return */
	if(cpu_id != ondemand_timer[cpu_id].cluster_m)
		return ret;

	Mdrv_CpuFreq_Lock(cpu_id, (char *)__FUNCTION__);
	if( (!start_userspace_ondemand_handshake) || (atomic_read(&disable_dvfs) == 1) )
	{
		printk("\033[34mFunction = %s, [Return] start_userspace_ondemand_handshake is %d disable_dvfs is %d\033[m\n",
			__PRETTY_FUNCTION__, start_userspace_ondemand_handshake, atomic_read(&disable_dvfs));	// this means utopia is not ready
		ret = -1;
        goto not_change_cpu_freq;
	}
	pr_debug("[dvfs_boost] here comes a client id = %lu and cpu_freq_in_khz = %lu cpu:%d\n", client_id, cpu_freq_in_khz, cpu_id);

	//get_online_cpus();
	for_each_online_cpu(i)
	{
		policy = cpufreq_cpu_get(i);

		if(!policy)
		{
			printk("\033[31mFunction = %s, Line = %d, cpu%d do: [cpu%d] policy is NULL\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, get_cpu(), i);
			put_cpu();
			//put_online_cpus();
			goto not_change_cpu_freq;
		}
		cpufreq_cpu_put(policy);
	}
	//put_online_cpus();

	// extend the timer
	ondemand_timer[cpu_id].jiffies_boost_lasttime = jiffies;
	if (cpu_freq_in_khz == 0x5566) {
		goto not_change_cpu_freq;

	} else if (cpu_freq_in_khz == 0) {
		if (is_boost_client_running(client_id, cpu_id) == true) {
			del_boost_client(client_id, cpu_id);
			if (is_any_boost_client_running(cpu_id) == true) {
				bc = find_boost_client_with_highest_priority(cpu_id);
				write_echo_calibrating_freq(bc->cpu_freq_in_khz, cpu_id);
				goto change_cpu_freq;
			} else {
#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
				goto not_change_cpu_freq;
#else
				write_echo_calibrating_freq(1008000, cpu_id);
				goto change_cpu_freq;
#endif
			}
		} else {
			printk(KERN_WARNING "[dvfs_boost] this client is not running\n");
		}
	} else {
		bc = find_boost_client(client_id, cpu_id);
		// check if a client with same client_id already exists
		if (is_boost_client_running(client_id, cpu_id) == true) {
			goto not_change_cpu_freq;
		} else {
			// registered a boost client with client_id and cpu_freq_in_khz
			add_boost_client(client_id, cpu_freq_in_khz, cpu_id, NULL);
			// echo_calibrating_freq = the cpu_freq_in_khz of the client with highest priority
			bc = find_boost_client_with_highest_priority(cpu_id);
			write_echo_calibrating_freq(bc->cpu_freq_in_khz, cpu_id);
			goto change_cpu_freq;
		}
	}

	// something wrong happened
	ret = -1;
	goto not_change_cpu_freq;

change_cpu_freq:
	__CPU_calibrating_proc_write(read_echo_calibrating_freq(cpu_id), cpu_id);
not_change_cpu_freq:
	Mdrv_CpuFreq_UnLock(cpu_id, (char *)__FUNCTION__);
	return ret;
}

int __CPU_calibrating_proc_write(const unsigned long idx, unsigned int cpu_id)
{
#if (!defined CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE) && (!defined CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH)
	struct cpufreq_policy *policy;
	struct cpufreq_freqs freqs;
#endif
	int i, set_freq = idx;

	if(cpu_id != ondemand_timer[cpu_id].cluster_m)
		return 0;

	if( (!start_userspace_ondemand_handshake) || (atomic_read(&disable_dvfs) == 1) )
	{
		printk("\033[34mFunction = %s, [Return] start_userspace_ondemand_handshake is %d disable_dvfs is %d\033[m\n",
			__PRETTY_FUNCTION__, start_userspace_ondemand_handshake, atomic_read(&disable_dvfs));	// this means utopia is not ready
		return 0;
	}

	if (set_freq > ondemand_timer[cpu_id].sys_max_freq) {
		if (mstar_debug)
			printk("\033[34m%s %d: cpu:%d %dkhz > sys max:%dkhz, set freq to %dkhz\033[m\n",
				__PRETTY_FUNCTION__, __LINE__, cpu_id, set_freq, ondemand_timer[cpu_id].sys_max_freq, ondemand_timer[cpu_id].sys_max_freq);
		set_freq = ondemand_timer[cpu_id].sys_max_freq;
	}

	if (mstar_debug)
	{
		printk("\033[34m%s %d: set cpu:%d %dkhz\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id, set_freq);
	}
	write_cpufreq_to_RIU(set_freq, cpu_id);
	register_frequency = set_freq;

#if (!defined CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE) && (!defined CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH)
	mstar_update_sched_clock();
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE

	//get_online_cpus();
	for_each_online_cpu(i)
	{
        if (ondemand_timer[i].cluster_m == cpu_id)
        {
            // policy->max don't be change
			//ondemand_timer[i].policy->max = set_freq;  // Belong this cluster, we set its max frequency to set_freq.
			ondemand_timer[i].cur_freq = set_freq;
			if (mstar_debug) {
				pr_debug("\033[35m %s %d: cpu_id:%d i:%d policy->max:%d set_freq:%d\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id, i, ondemand_timer[i].policy->max, set_freq);
			}
        }
	}
	//put_online_cpus();
	change_cpus_timer((char *)__FUNCTION__, idx, cpu_id);

	return 0;
}

void write_cpufreq_to_RIU(const unsigned long new_scaling_cur_freq, int cpu_id)
{

//  printk(DVFS_DEBUG "\033[31mFunction = %s, Line = %d, cpu:%d write cpu_freq:%u\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id, (unsigned int)(new_scaling_cur_freq/1000));
#if 1
//  int m = ondemand_timer[cpu_id].cluster_m;
//	reg_writew((new_scaling_cur_freq/1000), ondemand_timer[m].freq_riu);
#else
    MHalDvfsCpuClockAdjustment((new_scaling_cur_freq/1000), cpu_id);
#endif
}

#ifdef CONFIG_MSTAR_DVFS
void MDrvDvfsVoltageSetup(unsigned int dwCpuClock, unsigned int dwVoltage, unsigned int dwVoltageType, unsigned int dwCpu);
void MDrvDvfsVoltageSetup(unsigned int dwCpuClock, unsigned int dwVoltage, unsigned int dwVoltageType, unsigned int dwCpu)
{
	/* We have to lock a lock here for Multi-Cluster Antutu from Maserati.
	 * This is to prevent T_sensor and Big_Cluster calling MDrvDvfsVoltageSetup() at the same time, which will cause finished_change_cnt != change_cnt
	 * You can see Mantis 1139537, the case is T_sensor doing MHalDvfsPradoMonitor() from MDrvDvfsQueryCpuClockByTemperature(), and Big_Cluster doing MDrvDvfsVoltageSetup() at the same time

	 * T_sensor can only lock Little_Cluster_Lock, so Little_Cluster will not having any problem.
	 * The reason of why T_sensor doesnot lock Big_Cluster_Lock is due to Multi-Cluster Antutu will let Big_Cluster too busy to do dvfs_handle, including release Big_Cluster_Lock.
	 */
	long result = 0;
	voltage_change_result[dwCpu] = 0;
	if(start_userspace_ondemand_handshake == 1)
	{
		mutex_lock(&mstar_voltagesetup);
		ready_to_change_cpufreq[dwCpu]      = dwCpuClock;
		ready_to_change_voltage[dwCpu]      = dwVoltage;
		ready_to_change_voltage_type[dwCpu] = dwVoltageType;
		ready_to_change_cpu[dwCpu]          = 1;
		ready_to_change_cluster_id[dwCpu]   = ondemand_timer[dwCpu].cluster;
		change_cnt[dwCpu]++;

		pr_debug("\033[35m[INFO] Data Exchange Count to User Space: %d\033[m\n", change_cnt[dwCpu]);
		pr_debug("\033[35m[INFO] Voltage: %d\033[m\n", dwVoltage);
		pr_debug("\033[35m[INFO] VoltageType: %d\033[m\n", dwVoltageType);
		pr_debug("\033[35m[INFO] CPU: %d Cluster: %d\033[m\n", dwCpu, ondemand_timer[dwCpu].cluster);

		up(&DVFS_on_demand_event_SEM);

                mutex_unlock(&mstar_voltagesetup);
		result = wait_event_timeout(DVFS_on_demand_event_waitqueue_userspace_return, finished_change_cnt[dwCpu]==change_cnt[dwCpu] , MAX_SCHEDULE_TIMEOUT);
		pr_debug("\033[35m[INFO] Data Exchange Count from User Space: %d, %d, %d, %d, %lX\033[m\n", dwVoltage, dwVoltageType, change_cnt[dwCpu], finished_change_cnt[dwCpu], result);
	}
	else
		printk("\033[35m[Return] Function = %s, start_userspace_ondemand_handshake is %d\033[m\n", __PRETTY_FUNCTION__, start_userspace_ondemand_handshake);
}
EXPORT_SYMBOL(MDrvDvfsVoltageSetup);
#endif

#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
static int integrator_set_target_on_demand(struct cpufreq_policy *policy, unsigned int target_freq, unsigned int relation)
{
	cpumask_t cpus_allowed;
	int cpu = policy->cpu;
	int cpu_id = get_cpu();
	put_cpu();

#if DVFS_BLOCK_NONMASTER_CORE
#if defined(CONFIG_CPU_FREQ_GOV_INTERACTIVE)
	/* for interactive mode, the tsk owner cpu will be cpu0, so we only need to check policy->cpu */
	if( ondemand_timer[cpu].cluster_m != cpu )
#else
	/* only master CPU of cluster allowed to adjust cpufreq(due to no mutex for now) */
	if( ondemand_timer[cpu_id].cluster_m != cpu_id || cpu != cpu_id)
#endif
	{
		//printk("\033[35m[LINE: %d] cpu_id is %d, cluster_m is %d, cpu is %d\033[m\n",
		//      __LINE__, cpu_id, ondemand_timer[cpu_id].cluster_m, cpu);
		//printk("\033[35mFunction = %s, Line = %d, cpu:%d set copufreq from %dKHz to %dKHz\033[m\n",
		//      __PRETTY_FUNCTION__, __LINE__, cpu, policy->cur, target_freq);
		return 0;
	}
#endif

	Mdrv_CpuFreq_Lock(cpu_id, (char *)__FUNCTION__);
	if( (!start_userspace_ondemand_handshake) || (atomic_read(&disable_dvfs) == 1) )
	{
		if (atomic_read(&disable_dvfs_debug))
			pr_debug("\033[34mFunction = %s, start_userspace_ondemand_handshake is %d disable_dfvs is %d. Go out, do nothing!!\033[m\n",
                	__PRETTY_FUNCTION__, start_userspace_ondemand_handshake, atomic_read(&disable_dvfs));
		atomic_set(&disable_dvfs_debug, 0);
		goto set_target_out;
	}
	else
	{
		atomic_set(&disable_dvfs_debug, 1);
		/*
		 * Save this threads cpus_allowed mask.
		 */
        if (mstar_debug) {
            pr_debug("\033[35mFunction = %s, Line = %d, cpu:%d set copufreq from %dKHz to %dKHz\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu, policy->cur, target_freq);
        }
		cpus_allowed = current->cpus_allowed;

		/*
		 * Bind to the specified CPU.  When this call returns,
		 * we should be running on the right CPU.
		 */
		set_cpus_allowed(current, cpumask_of_cpu(cpu_id));
#if !defined(CONFIG_CPU_FREQ_GOV_INTERACTIVE)
		// for interactive mode, only cpu0 will go into this set_target driver, so we do not check this
                if(cpu != smp_processor_id())
                {
                        pr_debug("\033[35mFunction = %s, Line = %d, cpu:%d, current task running status: smp_processor_id()=%d\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu, smp_processor_id());
                        set_cpus_allowed(current, cpus_allowed);
                        goto set_target_out;
                }
#endif

		if(target_freq > policy->max)
		{
			printk(KERN_WARNING "\033[31mFunction = %s, Line = %d, adjust target_freq from %dKHz to policy->max %dKHz (over_max case), forcibly_set_target_flag[%d] is %d\033[m\n",
					__PRETTY_FUNCTION__, __LINE__, target_freq, policy->max, cpu_id, forcibly_set_target_flag[cpu_id]);
			target_freq = policy->max;
		}

		if (is_any_boost_client_running(cpu) == true)
		{
			// boost client is running, don't change this.
            goto set_target_out;
		}

		/* update timer, cpu_freq(RIU), and scaling_cur_freq for all cpus */
		change_cpus_timer((char *)__FUNCTION__, target_freq, cpu);

		/*
		 * Restore the CPUs allowed mask.
		 */
		set_cpus_allowed(current, cpus_allowed);

	}

set_target_out:
	Mdrv_CpuFreq_UnLock(cpu_id, (char *)__FUNCTION__);
	return 0;
}
#else
static int integrator_set_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	cpumask_t cpus_allowed;
	int cpu = policy->cpu;
	struct cpufreq_freqs freqs;

	/*
	 * Save this threads cpus_allowed mask.
	 */
	cpus_allowed = current->cpus_allowed;

	/*
	 * Bind to the specified CPU.  When this call returns,
	 * we should be running on the right CPU.
	 */
	set_cpus_allowed(current, cpumask_of_cpu(cpu));
	BUG_ON(cpu != smp_processor_id());

	freqs.old = get_freq(0); //  johnson

	/* icst_hz_to_vco rounds down -- so we need the next
	 * larger freq in case of CPUFREQ_RELATION_L.
	 */
	if (relation == CPUFREQ_RELATION_L)
		target_freq += 999;
	if (target_freq > policy->max)
		target_freq = policy->max;

	freqs.new = get_freq(0); //johnson
	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new) {
		set_cpus_allowed(current, cpus_allowed);
		return 0;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
	cpufreq_freq_transition_begin(policy, &freqs);
#else
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);
#endif

	/*
	 * Restore the CPUs allowed mask.
	 */
	set_cpus_allowed(current, cpus_allowed);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
	cpufreq_freq_transition_end(policy, &freqs, 0);
#else
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
#endif

	return 0;
}
#endif

void change_cpus_timer(char *caller, unsigned int target_freq, unsigned int cpu_id)
{
	int i = 0;
	int ret[CONFIG_NR_CPUS] = {0};
	int ret_value = 1;
	unsigned int ori_target_freq = 0;
	struct cpufreq_policy *other_cpu_policy;
	struct cpufreq_freqs freqs;
    int cid = ondemand_timer[cpu_id].cluster;

	if(bootarg_dvfs_disable)
		return;

	if(!start_userspace_ondemand_handshake)
	{
		printk("\033[35mFunction = %s, cpu:%d, cluster master cpu:%d. Return, start_userspace_ondemand_handshake is %u\033[m\n",
			 __PRETTY_FUNCTION__, cpu_id, ondemand_timer[cpu_id].cluster_m, start_userspace_ondemand_handshake);
		return;
	}

#if DVFS_BLOCK_NONMASTER_CORE
    /* If cpu_id is not the master cpu of cluster, return it.
     * Example: Cluster 1 includes CPU0 and CPU1. The master CPU of cluster 1 is CPU0.
     * So, we just allow CPU0 pass through and then set target freq.
     * This is for performance issue, because we don't hope each CPU can set target freq.
     */
    if(cpu_id != ondemand_timer[cpu_id].cluster_m)
    {
		pr_debug("\033[35mFunction = %s, cpu:%d, cluster master cpu:%d. Return, do nothing\033[m\n", __PRETTY_FUNCTION__, cpu_id, ondemand_timer[cpu_id].cluster_m);
        return;
    }
#endif

    /* If target_freq is 54472, we set ac_str_cpufreq to target_freq.
     * It could be came from STR(472) flow.
     */
	if(target_freq == 54472)
	{
		target_freq = atomic_read(&ondemand_timer[cpu_id].ac_str_cpufreq);
		printk("\033[35mFunction = %s, Line = %d, cpu: %d, set ac_str_cpufreq: %d Khz\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id, target_freq);
	}

	//spin_lock(&set_freq_lock);
	DVFS_LOG_DEBUG( "\033[36mcaller: %s, cluster: %d target_freq: %d KHz\033[m\n", caller, cid, target_freq);
	ori_target_freq = target_freq;

	//get_online_cpus();
	for_each_online_cpu(i)
	{
        if (ondemand_timer[i].cluster == cid) // means the same cluster
        {
	        other_cpu_policy = cpufreq_cpu_get(i);

	        if(!other_cpu_policy) // to avoid some cpus have not had its policy
	        {
	            printk("\033[35mFunction = %s, Line = %d, cpu%d do: [cpu%d] policy is NULL\033[m\n", __PRETTY_FUNCTION__, __LINE__, get_cpu(), i);
                put_cpu();
                ret[i] = 0;
                ret_value = ret_value * ret[i];
                continue;
            }

			// if "Big Cluster", "antutu is running" and "Not from T_sensor", we will not call MDrvDvfsProc().
			if( (getCpuCluster(i) != 0) && (find_boost_client(64, i)) && ( (strcmp(caller, "__CPU_calibrating_proc_write") == 0) || (strcmp(caller, "integrator_set_target_on_demand") == 0)) )
				ret[i] = 0;	// if once here, this time will not do MDrvDvfsProc()
			else
				ret[i] = 1;	// this is OK case

			ret_value = ret_value * ret[i];
		    cpufreq_cpu_put(other_cpu_policy);
        }
	}
	//put_online_cpus();

    if (ret_value)
    {
		/*
		 * set all cpus to new cpu_freq(scaling_cur_freq), and set the timer
		 * always let wait_for_voltage to do voltage_change, adjust target_freq according to Temperature, write adjusted target_freq to RIU, then use new target_freq to set timer, jiffes
		 */
		/* rise voltage first, wait for voltage change(accroding to target_freq) */

		target_freq = MDrvDvfsProc(target_freq, cpu_id);
		DVFS_LOG_DEBUG( "\033[33mFunction = %s, Line = %d, cpu %d the target_freq is changed(by MDrvDvfsProc), from %d KHz to %d KHz\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu_id, ori_target_freq, target_freq);

                if (target_freq == 0)
                {
			DVFS_LOG_DEBUG("Function = %s, Line = %d, ori_target_freq = %d, caller = %s\n", __FUNCTION__, __LINE__,ori_target_freq,caller);
			if ( ori_target_freq == 0 )
                    		target_freq = get_freq(cpu_id);
                        else
				target_freq = ori_target_freq;
			DVFS_LOG_DEBUG("Function = %s, Line = %d, target_freq = %d\n", __FUNCTION__, __LINE__,target_freq);
                }

		//if(voltage_change_result[cpu_id] != 1) // pass in userspace voltage change
		{
#if (!defined CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE) && (!defined CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH)
			mstar_update_sched_clock();
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE

			//get_online_cpus();
			for_each_online_cpu(i)
			{
				if(cid == ondemand_timer[i].cluster) // means the same cluster.
				{
					other_cpu_policy = cpufreq_cpu_get(i);

					if(!other_cpu_policy)
					{
						printk("\033[35mFunction = %s, Line = %d, cpu%d do: [cpu%d] policy is NULL\033[m\n", __PRETTY_FUNCTION__, __LINE__, get_cpu(), i);
						put_cpu();
						BUG_ON(!other_cpu_policy);
					}

					if(other_cpu_policy->cur == target_freq)
					{
						cpufreq_cpu_put(other_cpu_policy);
						continue;
					}

					freqs.cpu = i;
					freqs.old = other_cpu_policy->cur;
					freqs.new = target_freq;

					if (mstar_debug)
					{
						pr_debug("\033[31mFunction = %s, Line = %d, cpu%d do: [cpu%d] adjust cpufreq from %d KHZ to %d KHZ\033[m\n", __PRETTY_FUNCTION__, __LINE__, get_cpu(), i, freqs.old, freqs.new);
						put_cpu();
					}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
					cpufreq_freq_transition_begin(other_cpu_policy, &freqs);
					cpufreq_freq_transition_end(other_cpu_policy, &freqs, 0);
#else
					cpufreq_notify_transition(other_cpu_policy, &freqs, CPUFREQ_PRECHANGE);
					cpufreq_notify_transition(other_cpu_policy, &freqs, CPUFREQ_POSTCHANGE);
#endif
					cpufreq_cpu_put(other_cpu_policy);

					ondemand_timer[i].cur_freq = target_freq;
					trace_cpufreq_change(freqs.cpu, freqs.old, freqs.new); //for meansure
				}

			}
			//put_online_cpus();

		}
	}
	//spin_unlock(&set_freq_lock);
}

#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
static struct freq_attr *mstar_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver integrator_driver = {
	.verify		= integrator_verify_policy,
	.target		= integrator_set_target_on_demand,
	.get		= get_freq,
	.init		= integrator_cpufreq_init,
	.name		= "ondemand",
	.attr		= mstar_cpufreq_attr,
};
#else
static struct cpufreq_driver integrator_driver = {
	.verify		= integrator_verify_policy,
	.target		= integrator_set_target,
	.get		= get_freq,
	.init		= integrator_cpufreq_init,
	.name		= "integrator",
};
#endif

static int CPU_calibrating_proc_open(struct inode *inode, struct file *file)
{
	if (atomic_read(&proc_is_open))
		return -EACCES;

	atomic_set(&proc_is_open, 1);

	return single_open(file, &CPU_calibrating_seq_show, NULL);
}

static int CPU_calibrating_proc_release(struct inode *inode, struct file * file)
{

	WARN_ON(!atomic_read(&proc_is_open));
	atomic_set(&proc_is_open, 0);
        return single_release(inode, file);
}

static int on_demand_handshake_proc_open(struct inode *inode, struct file *file)
{
	if( using_kernel_i2c_interface == 1 )
		return -EACCES;
	if(!bootarg_dvfs_disable)
		start_userspace_ondemand_handshake = 1;

	if(atomic_read(&on_demand_handshake_is_open))
		return -EACCES;

	atomic_set(&on_demand_handshake_is_open, 1);

	return 0;
}

static int on_demand_handshake_proc_release(struct inode *inode, struct file * file)
{
	printk("\033[35mFunction = %s, Line = %d, set start_userspace_ondemand_handshake to be 0\033[m\n", __PRETTY_FUNCTION__, __LINE__);
	start_userspace_ondemand_handshake = 0;

	WARN_ON(!atomic_read(&on_demand_handshake_is_open));
	atomic_set(&on_demand_handshake_is_open, 0);
	return 0;
}

ssize_t on_demand_handshake_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	ON_Demand_From_Userspace from_user_data;
    int i;

	if(!start_userspace_ondemand_handshake)
	{
		printk("\033[35mFunction = %s, start_userspace_ondemand_handshake is %d\033[m\n", __PRETTY_FUNCTION__, start_userspace_ondemand_handshake);
		return  -EFAULT;
	}
	else
	{
		if(!count)
			return count;

		if(count >= MAX_DMSG_WRITE_BUFFER)
			count = MAX_DMSG_WRITE_BUFFER - 1;

		if(copy_from_user(&from_user_data, buf, sizeof(from_user_data)))
                {
                        printk("\033[35mFunction = %s, Line = %d, copy_from_user error\033[m\n", __PRETTY_FUNCTION__,__LINE__);
                        return -EFAULT;
                }

        for (i = 0; i < CONFIG_NR_CPUS; i ++) {
            if (ondemand_timer[i].cluster == from_user_data.from_userspace_cluster_id) {
                ready_to_change_cpu[i] = 0;
                    break;
            }
        }
        if( i >= CONFIG_NR_CPUS )
        {
            printk("\033[35mFunction = %s, Line = %d, Error : cpu%d is not exist! (total %d cpus)\033[m\n", __PRETTY_FUNCTION__,__LINE__,i,CONFIG_NR_CPUS);
            return  -EFAULT;
        }

		finished_change_cnt[i] = from_user_data.from_userspace_finished_change_cnt;
		voltage_change_result[i] = from_user_data.from_userspace_voltage_change_result;

		if(mstar_debug)
		{
			pr_debug("\033[33m%s cluster:%d result is %d, cpu:%d, input:%d, change_cnt:%d wake_up wait_queue for write_down\033[m\n",
				__func__, from_user_data.from_userspace_cluster_id, voltage_change_result[i], i, finished_change_cnt[i], change_cnt[i]);
		}
		wake_up(&DVFS_on_demand_event_waitqueue_userspace_return);	// to wake_up a wait_queue waiting for voltage change

		return count;
	}
}

ssize_t on_demand_handshake_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ON_Demand_To_Userspace to_userspace_data;
	int err;
    int i, cpu = 0;

	if(!start_userspace_ondemand_handshake)
	{
		if(bootarg_dvfs_disable)
		{
			printk("\033[31mFunction = %s, [Warning] DVFS=disable is set @ bootargs, the dvfs_thread will wait forever\033[m\n", __PRETTY_FUNCTION__);
			down_interruptible(&DVFS_disable_SEM);
		}

		printk("\033[35mFunction = %s, start_userspace_ondemand_handshake is %d\033[m\n", __PRETTY_FUNCTION__, start_userspace_ondemand_handshake);
		return  -EFAULT;
	}
	else
	{
		down_interruptible(&DVFS_on_demand_event_SEM);
        for (i = 0; i < CONFIG_NR_CPUS; i ++) {
            if (ready_to_change_cpu[i] == 1) {
                cpu = i;
                break;
            }
        }

        to_userspace_data.to_userspace_cpufreq = ready_to_change_cpufreq[cpu];
        to_userspace_data.to_userspace_voltage = ready_to_change_voltage[cpu];
        to_userspace_data.to_userspace_voltage_type = ready_to_change_voltage_type[cpu];
		to_userspace_data.to_userspace_cluster_id = ready_to_change_cluster_id[cpu];
        to_userspace_data.to_userspace_change_cnt = change_cnt[cpu];

		err = copy_to_user((void *)buf, &to_userspace_data, sizeof(to_userspace_data));

		*ppos += sizeof(to_userspace_data);

		return sizeof(to_userspace_data);
	}
}

static int t_sensor_proc_open(struct inode *inode, struct file *file)
{

	if(atomic_read(&t_sensor_proc_is_open))
		return -EACCES;

	atomic_set(&t_sensor_proc_is_open, 1);

	return 0;
}

static int t_sensor_proc_release(struct inode *inode, struct file * file)
{

	WARN_ON(!atomic_read(&t_sensor_proc_is_open));
	atomic_set(&t_sensor_proc_is_open, 0);
	return 0;
}

ssize_t t_sensor_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[MAX_DMSG_WRITE_BUFFER];
	long set;

	if (!count)
		return count;

	if (count >= MAX_DMSG_WRITE_BUFFER)
		count = MAX_DMSG_WRITE_BUFFER - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	if (strict_strtol(buffer, 0, &set) != 0)
    {  //Force change str to decimal conversion (str, base, *converted_num), base is hex, decimal, or ...
       return -EINVAL;
    }

    switch (set)
    {
        case 0:
            bootarg_dvfs_t_sensor_disable = 0;
        break;
        case 1:
            bootarg_dvfs_t_sensor_disable = 1;
        break;
        default:
        break;
    }

	return count;
}

ssize_t t_sensor_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

    if (mstar_debug) {
        int i;
        for (i = 0; i < CONFIG_NR_CPUS; i ++)
            show_boost_client(i);
        printk("\n");
    }
    printk("T sensor:%s\n",bootarg_dvfs_t_sensor_disable? "disable":"enable");

	return 0;
}

const struct file_operations proc_CPU_calibrating_operations = {
	.write      = CPU_calibrating_proc_write,
	.read      = seq_read,
	.llseek      = seq_lseek,
	.open       = CPU_calibrating_proc_open,
	.release    = CPU_calibrating_proc_release,
  	.unlocked_ioctl  = CPU_calibrating_proc_ioctl,
};

const struct file_operations proc_on_demand_handshake_operations = {
	.open       = on_demand_handshake_proc_open,
	.write      = on_demand_handshake_proc_write,
	.read		= on_demand_handshake_proc_read,
	.release    = on_demand_handshake_proc_release,
};

const struct file_operations proc_t_sensor_operations = {
	.write      = t_sensor_proc_write,
	.read      = t_sensor_proc_read,
	.open       = t_sensor_proc_open,
	.release    = t_sensor_proc_release,
};

#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE)
static void Mdrv_CPU_T_sensor_Check_callback(unsigned long value)
{
	int i = 0,j = 0;
	int cpu_id;
	unsigned long echo_calibrating_freq_tmp = 0;
	unsigned int max_clock;
	int if_do_change_cpus_timer = 0;

	cpu_id = get_cpu();
	put_cpu();

	if(cpu_id != t_sensor_monitor_owner_cpu)
	{
		printk("\033[31mcpu_id is %d, t_sensor_monitor_owner_cpu is %d, not match\033[m\n", cpu_id, t_sensor_monitor_owner_cpu);
		WARN_ON(1);
	}

	/* update boost time, if need */
	for_each_online_cpu(i)
	{
		if(i != ondemand_timer[i].cluster_m)
			continue;

		if (is_any_boost_client_running(i) == true)
		{
			if (jiffies_to_msecs(jiffies - ondemand_timer[i].jiffies_boost_lasttime) > BOOST_AGING_TIMEOUT_IN_MS)
			{
				pr_debug("[dvfs boost] timout happens and delete all of the running clients\n");
				del_all_boost_client(i);
#if !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) && !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) && !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
				__CPU_calibrating_proc_write(1008000, i);
#endif
			}
			else
			{
				// do nothing
			}
		}
		else
		{
			// do nothing
		}
	}
	/* update boost time, if need(done) */

	/* check if we cannot do t_sensort check */
    if (bootarg_dvfs_t_sensor_disable)
        return;

	if(!start_userspace_ondemand_handshake)
	{
#if !defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
		spin_lock(&T_sensor_lock);
		Check_T_sensor_timer.expires = jiffies + FREQ_CHECK_LINK_TIME;
		add_timer(&Check_T_sensor_timer);
		spin_unlock(&T_sensor_lock);
#endif

		pr_debug("\033[34mFunction = %s, [cpu%d] return Mdrv_CPU_T_sensor_Check_callback, start_userspace_ondemand_handshake is %d\033[m\n",
			__PRETTY_FUNCTION__, cpu_id, start_userspace_ondemand_handshake);
		return;
	}
	/* check if we cannot do t_sensort check(done) */

	for_each_online_cpu(i)
	{
		ondemand_timer[i].t_sensor_max_freq = 0;

		// Mask this for resetting slave cpus' max_freq.
		// While leaving boost mode, slave cpus' max_freq should be reset here.
		if(i == ondemand_timer[i].cluster_m)
		{
			DVFS_LOG_DEBUG( "\033[35mFunction = %s, Line = %d, query MDrvDvfsQueryCpuClockByTemperature for cpu %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, i);
			ondemand_timer[i].t_sensor_max_freq = MDrvDvfsQueryCpuClockByTemperature(i);
			DVFS_LOG_DEBUG( "\033[35mFunction = %s, Line = %d, query MDrvDvfsQueryCpuClockByTemperature for cpu %d done\033[m\n", __PRETTY_FUNCTION__, __LINE__, i);
			for_each_online_cpu(j)
			{
				if ( (ondemand_timer[j].cluster_m == i) && (i != j) )
					ondemand_timer[j].t_sensor_max_freq = ondemand_timer[i].t_sensor_max_freq;
			}
		}
		DVFS_LOG_DEBUG("\033[34mFunction = %s, Line = %d, cpu:%d, T_sensor_max_freq: %lu, forcibly_set_target_flag[%d] is %d\033[m\n",
			__PRETTY_FUNCTION__, __LINE__, i, ondemand_timer[i].t_sensor_max_freq, i, forcibly_set_target_flag[i]);
	}

#if (!defined CONFIG_MP_DVFS_FORCE_PINGO_TEST) && (!defined CONFIG_MP_DVFS_VID_ONLY)
	for_each_online_cpu(i)
	{
		if(ondemand_timer[i].t_sensor_max_freq == 0)
			continue;

		max_clock = ondemand_timer[i].policy->cpuinfo.max_freq;

		if (is_any_boost_client_running(i) == true)
		{
			echo_calibrating_freq_tmp = read_echo_calibrating_freq(i);
			if(ondemand_timer[i].t_sensor_max_freq > echo_calibrating_freq_tmp)
			{
				pr_debug("\033[35mFunction = %s, Line = %d, due to boost, change t_sensor_max_freq from %lu to %lu\033[m\n",
					__PRETTY_FUNCTION__, __LINE__, ondemand_timer[i].t_sensor_max_freq, echo_calibrating_freq_tmp);
				ondemand_timer[i].t_sensor_max_freq = echo_calibrating_freq_tmp;
			}
		}
		else if(ondemand_timer[i].t_sensor_max_freq > max_clock)
		{
			DVFS_LOG_DEBUG("\033[35mFunction = %s, Line = %d, normal change, change t_sensor_max_freq from %lu to %d\033[m\n",
				__PRETTY_FUNCTION__, __LINE__, ondemand_timer[i].t_sensor_max_freq, max_clock);
			ondemand_timer[i].t_sensor_max_freq = max_clock;
		}
	}

	//get_online_cpus();
	for_each_online_cpu(i)
	{
		if(ondemand_timer[i].t_sensor_max_freq == 0)
			continue;

		if((ondemand_timer[i].t_sensor_max_freq == 100000) || (ondemand_timer[i].t_sensor_max_freq == 200000))	// this is a special case, only adjust the cpu_voltage only
		{
			ondemand_timer[i].t_sensor_max_freq = ondemand_timer[i].policy->max;
			forcibly_set_target_flag[i] = 5;
		}
		else
		{
			ondemand_timer[i].policy->max = ondemand_timer[i].t_sensor_max_freq;
			struct boost_client *bc = NULL;
			if (is_any_boost_client_running(i) == true) {
				bc = find_boost_client_with_highest_priority(i);
				change_cpus_timer((char *)__FUNCTION__,bc->cpu_freq_in_khz,i);
			}

			if(ondemand_timer[i].t_sensor_max_freq == CONFIG_DVFS_CPU_CLOCK_PROTECT(i)*1000 )		// this is an over-temperature case
			{
				forcibly_set_target_flag[i] = 20;
				DVFS_LOG_DEBUG("\033[31mFunction = %s, Line = %d, T_sensor_max_freq:%lu < [%d] current freq:%d, forcibly_set_target_flag[%d]:%d\033[m\n",
					__func__, __LINE__, ondemand_timer[i].t_sensor_max_freq, i, ondemand_timer[i].policy->cur, i, forcibly_set_target_flag[i]);

				/* This is for big_cluster doing antutu case.
				 * At this case, the big_cluster is very busy, so the dvfs_on_demand will not work normally.
				 * We then do change_cpus_timer directly to decrease the cpufreq and cpuvoltage
				 */
				DVFS_LOG_DEBUG("\033[31mFunction = %s, Line = %d, [Over Temperature Handling Start] changing cpufreq to %lu\033[m\n",
					__PRETTY_FUNCTION__, __LINE__, ondemand_timer[i].t_sensor_max_freq);
				change_cpus_timer((char *)__FUNCTION__, ondemand_timer[i].t_sensor_max_freq, i);
				DVFS_LOG_DEBUG("\033[31mFunction = %s, Line = %d, [Over Temperature Handling Start] changing cpufreq OK!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);

				ondemand_timer[i].over_temperature_mode = 1;
				if_do_change_cpus_timer = 1;
			}
			else
			{
				if (forcibly_set_target_flag[i] > 0)
				{
					forcibly_set_target_flag[i] --;
				}

				if( (ondemand_timer[i].over_temperature_mode) && (ondemand_timer[i].t_sensor_max_freq > ondemand_timer[i].policy->cur) )
				{
					DVFS_LOG_DEBUG("\033[31mFunction = %s, Line = %d, [Over Temperature Handling End] changing cpufreq to %lu\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, ondemand_timer[i].t_sensor_max_freq);
					change_cpus_timer((char *)__FUNCTION__, ondemand_timer[i].t_sensor_max_freq, i);
					DVFS_LOG_DEBUG("\033[31mFunction = %s, Line = %d, [Over Temperature Handling End] changing cpufreq OK!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);

					ondemand_timer[i].over_temperature_mode = 0;
					if_do_change_cpus_timer = 1;
				}
			}

			over_temperature_mode[i] = ondemand_timer[i].over_temperature_mode;

			if( (getCpuCluster(i) != 0) && (find_boost_client(64, i)) && (if_do_change_cpus_timer == 0) )
			{
				printk("\033[31mFunction = %s, Line = %d, [Big Cluster Antutu] changing cpufreq to %lu\033[m\n",
					__PRETTY_FUNCTION__, __LINE__, ondemand_timer[i].t_sensor_max_freq);
				change_cpus_timer((char *)__FUNCTION__, ondemand_timer[i].t_sensor_max_freq, i);
				printk("\033[31mFunction = %s, Line = %d, [Big Cluster Antutu] changing cpufreq OK!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
			}
		}
	}
	//put_online_cpus();
#endif

#if !defined(CONFIG_MP_PLATFORM_T_SENSOR_OBSERVATION)
	spin_lock(&T_sensor_lock);
	Check_T_sensor_timer.expires = jiffies + FREQ_CHECK_LINK_TIME;
	add_timer(&Check_T_sensor_timer);
	spin_unlock(&T_sensor_lock);
#endif
}
#else
static void Mdrv_CPU_Freq_Check_callback(unsigned long value)
{
	int freq, i;
	struct cpufreq_freqs freqs;
	struct cpufreq_policy *policy;
	int cpu_id = get_cpu(); put_cpu();
    int min_clock, max_clock;

    min_clock = CONFIG_DVFS_CPU_CLOCK_MIN(cpu_id);
    max_clock = CONFIG_DVFS_CPU_CLOCK_MAX(cpu_id);

	if(*(volatile u32 *)(0xfd200a00) == 0x3697)
   	{
   		freq = *(volatile u32 *)(0xfd200a04);
   		freq = freq * 1000;

   		if (freq != current_frequency)
   		{
#if (!defined CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE) && (!defined CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH)
			mstar_update_sched_clock();
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
			pr_debug("\033[36m\nFunction = %s, Line = %d, (freq = %d KHZ) != (current_frequency = %d KHZ)\033[m\n", __PRETTY_FUNCTION__, __LINE__, freq, current_frequency);
			if(freq < min_clock)
			{
					printk(KERN_WARNING "\033[36m\n freq %d KHZ < MIN_CPU_FREQ %d KHZ ,not allowed\033[m\n", freq, min_clock);
					return;
			}

			if(freq > max_clock)
			{
					printk(KERN_WARNING "\033[36m\n freq %d KHZ > MAX_CPU_FREQ %d KHZ ,not allowed\033[m\n", freq, max_clock);
					return;
			}

			register_frequency = freq;

			//get_online_cpus();
			for_each_online_cpu(i)
	        {
				policy = cpufreq_cpu_get(i);
	        	freqs.cpu = i;
		        freqs.old = current_frequency;
		        freqs.new = freq;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
				cpufreq_freq_transition_begin(policy, &freqs);
				cpufreq_freq_transition_end(policy, &freqs, 0);
#else
		        cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);
              	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
#endif
            	cpufreq_cpu_put(policy);
            }
			//put_online_cpus();

			current_frequency = freq;
#if (!defined CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE) && (!defined CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH)
			change_interval(freqs.old, freqs.new);
#endif
   		}
   	}
	Check_Freq_timer.expires = jiffies + FREQ_CHECK_LINK_TIME;
	add_timer(&Check_Freq_timer);
}
#endif

static int __init init_procfs_msg(void)
{
    int i, j, k;
    struct cpufreq_frequency_table *freq_table;
    int cpu = get_cpu();
    int size = 0;
    put_cpu();

    memset(&ondemand_timer, 0, sizeof(ondemand_timer));

    //get_online_cpus();
    for_each_possible_cpu(i)
    {
        ondemand_timer[i].cur_freq = get_freq(i);
        ondemand_timer[i].sys_max_freq = CONFIG_DVFS_CPU_CLOCK_MAX(i);
        size = MDrvDvfsGetFreqTable(i,&freq_table);
        if (!freq_table) {
            printk("\033[35m%s: Unable to allocate frequency table \033[m\n", __PRETTY_FUNCTION__);
            //put_online_cpus();
            return -ENOMEM;
        }
        for (j = 0,k = 0; j < size; j++)
        {
            if (freq_table[j].frequency == CPUFREQ_TABLE_END)
            {
                    freq_table[k].frequency = CPUFREQ_TABLE_END;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,40)
                    freq_table[k].index = k;
#endif
                break;
            }

            if (freq_table[j].frequency >  CONFIG_DVFS_CPU_CLOCK_MAX(i))
                continue;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
//          printk("\033[35mFunction = %s, Line = %d, insert ==> %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, freq_table[k].frequency);
#else
            freq_table[k].index = k;
#endif
            k++;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
        printk("\033[35mFunction = %s, [cpu %d] set cpu_%d current freq = %d\033[m\n", __PRETTY_FUNCTION__, cpu, i, ondemand_timer[i].cur_freq);
        mstar_freq_table[i] = freq_table;
#else
        cpufreq_frequency_table_get_attr(&freq_table[0], i);
        pr_debug("\033[35mFunction = %s, [cpu %d] set cpu_%d current freq = %d\033[m\n", __PRETTY_FUNCTION__, cpu, i, ondemand_timer[i].cur_freq);
#endif
    }
    //put_online_cpus();

    cpufreq_register_driver(&integrator_driver);
    proc_create("CPU_calibrating", S_IRUSR | S_IWUSR, NULL, &proc_CPU_calibrating_operations);
    proc_create("on_demand_ctl"  , S_IRUSR | S_IWUSR, NULL, &proc_on_demand_handshake_operations);
    proc_create("t_sensor", S_IRUSR | S_IWUSR, NULL, &proc_t_sensor_operations);
#ifdef CONFIG_MSTAR_DVFS_DEBUG
    if (!dvfs_debug_init())
        pr_debug("\033[35mFunction = %s, dvfs debug node create fail\033[m\n", __PRETTY_FUNCTION__);
#endif
    sema_init(&DVFS_on_demand_event_SEM, 0);

#if !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) && !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) && !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL) && !defined(CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE)
    init_timer(&Check_Freq_timer);
    Check_Freq_timer.data = 1;
    Check_Freq_timer.function = Mdrv_CPU_Freq_Check_callback;
    Check_Freq_timer.expires = jiffies + FREQ_CHECK_LINK_TIME;
    add_timer(&Check_Freq_timer);
#endif

#if CONFIG_PM_OPP
#if 0	// test opp_data
    struct device *cpu_device = NULL;
    struct device_opp *dev_opp = NULL;
    struct opp_table *opp_table;
    struct dev_pm_opp *temp_opp = NULL;
    int opp_data_cnt = 0;
    for_each_online_cpu(i)
    {
        cpu_device = get_cpu_device(i);
        opp_table = _find_opp_table(cpu_device);

        printk("\033[35mFunction = %s, Line = %d, query opp_data for cpu%d, having %d available opp_data\033[m\n", __PRETTY_FUNCTION__, __LINE__, i, dev_pm_opp_get_opp_count(cpu_device));
        opp_data_cnt = 0;
        list_for_each_entry_rcu(temp_opp, &opp_table->opp_list, node) {
            if(temp_opp->available)
                printk("\033[31m    [%d] opp_data: available\033[m\n", opp_data_cnt);

            printk("\033[31m    freq is %u hz\033[m\n", temp_opp->rate);
            printk("\033[31m    volt is %u\033[m\n", temp_opp->u_volt);
            opp_data_cnt++;
        }
    }
#endif
#endif
    return 0;
}

static int __init CPU_calibrating_init(void)
{
    int i;
#if CONFIG_PM_OPP
    struct device *per_cpu_dev = NULL;
    int ret, opp_err;
    int opp_cnt = 0;
#endif
    printk("\033[35mFunction = %s, Line = %d, [cpu %d] do CPU_calibrating_init\033[m\n", __PRETTY_FUNCTION__, __LINE__, get_cpu());
    put_cpu();
#if CONFIG_PM_OPP
    for_each_online_cpu(i)
    {
        MDrvDvfsGetDvfsTable(i, (dvfs_opp_table *)mstar_init_opp_table);
        per_cpu_dev = get_cpu_device(i);
        opp_cnt = 0;
        if (!per_cpu_dev) {
            printk("\033[31mFunction = %s, Line = %d, No cpu device for cpu %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, i);
            BUG_ON(1);
            }

        while (mstar_init_opp_table[i].per_cpu_table[opp_cnt].clk != 0)
        {
            opp_err = dev_pm_opp_add(per_cpu_dev, mstar_init_opp_table[i].per_cpu_table[opp_cnt].clk, mstar_init_opp_table[i].per_cpu_table[opp_cnt].volt);
            if (opp_err) {
                printk("\033[31mFunction = %s, Line = %d, cannot add opp entries for cpu %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, i);
            }
            opp_cnt++;
        }

    }
#endif
	init_procfs_msg();
#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
	start_userspace_ondemand_handshake = 1;
#endif
	return 0;
}

static int __init DVFS_enable(char *str)
{
    if(strcmp(str, "disable") == 0)
    {
        printk("\nDVFS_disable\n");
		bootarg_dvfs_disable = 1;
		sema_init(&DVFS_disable_SEM, 0);
    }
    else
    {
		bootarg_dvfs_disable = 0;
    }
    return 0;
}
early_param("DVFS", DVFS_enable);

module_init(CPU_calibrating_init);

module_param(mstar_debug, uint, 0644);
MODULE_PARM_DESC(mstar_debug, "Debug for dvfs");
module_param(mstar_info, uint, 0644);
MODULE_PARM_DESC(mstar_info, "Info for dvfs");
