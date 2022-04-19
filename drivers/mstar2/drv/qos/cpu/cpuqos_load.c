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
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/irqnr.h>
#include <linux/kernel_stat.h>
#include <linux/jiffies.h>
#include <linux/tick.h>
#include <linux/delay.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/sched/cputime.h>
#else
#include <linux/cputime.h>
#endif
#include "cpuqos_drv.h"

atomic_t cpuqos_cpu_load = ATOMIC_INIT(-1); /* max. is 100 */


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#ifdef arch_idle_time
static cputime64_t get_idle_time(int cpu)
{
    cputime64_t idle;

    idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
    if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
        idle += arch_idle_time(cpu);
    return idle;
}

static u64 get_iowait_time(int cpu)
{
    u64 iowait, iowait_time = -1ULL;

    if (cpu_online(cpu))
        iowait_time = get_cpu_iowait_time_us(cpu, NULL);

    if (iowait_time == -1ULL)
        /* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
        iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
    else
        iowait = usecs_to_cputime64(iowait_time);

    return iowait;
}
#else
static u64 get_idle_time(int cpu)
{
    u64 idle, idle_usecs = -1ULL;

    if (cpu_online(cpu))
        idle_usecs = get_cpu_idle_time_us(cpu, NULL);

    if (idle_usecs == -1ULL)
        /* !NO_HZ or cpu offline so we can rely on cpustat.idle */
        idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
    else
        idle = idle_usecs * NSEC_PER_USEC;

    return idle;
}

static u64 get_iowait_time(int cpu)
{
    u64 iowait, iowait_usecs = -1ULL;

    if (cpu_online(cpu))
        iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

    if (iowait_usecs == -1ULL)
        /* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
        iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
    else
        iowait = iowait_usecs * NSEC_PER_USEC;

    return iowait;
}
#endif
#else
/* kernel 4.9 */
#ifdef arch_idle_time
static cputime64_t get_idle_time(int cpu)
{
    cputime64_t idle;

    idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
    if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
        idle += arch_idle_time(cpu);
    return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
    cputime64_t iowait;

    iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
    if (cpu_online(cpu) && nr_iowait_cpu(cpu))
        iowait += arch_idle_time(cpu);
    return iowait;
}

#else

static u64 get_idle_time(int cpu)
{
    u64 idle, idle_time = -1ULL;

    if (cpu_online(cpu))
        idle_time = get_cpu_idle_time_us(cpu, NULL);

    if (idle_time == -1ULL)
        /* !NO_HZ or cpu offline so we can rely on cpustat.idle */
        idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
    else
        idle = usecs_to_cputime64(idle_time);

    return idle;
}

static u64 get_iowait_time(int cpu)
{
    u64 iowait, iowait_time = -1ULL;

    if (cpu_online(cpu))
        iowait_time = get_cpu_iowait_time_us(cpu, NULL);

    if (iowait_time == -1ULL)
        /* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
        iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
    else
        iowait = usecs_to_cputime64(iowait_time);

    return iowait;
}
#endif
#endif


void cpuqos_get_cpu_load_once(u64 *usage, u64 *total)
{
    int i, j;
    u64 user, nice, system, idle, iowait, irq, softirq, steal;
    u64 guest, guest_nice;
    u64 sum_softirq = 0;
    unsigned int per_softirq_sums[NR_SOFTIRQS] = {0};
	u64 others = 0;

    user = nice = system = idle = iowait =
        irq = softirq = steal = 0;
    guest = guest_nice = 0;

    for_each_possible_cpu(i) {
        user += kcpustat_cpu(i).cpustat[CPUTIME_USER];
        nice += kcpustat_cpu(i).cpustat[CPUTIME_NICE];
        system += kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
        idle += get_idle_time(i);
        iowait += get_iowait_time(i);
        irq += kcpustat_cpu(i).cpustat[CPUTIME_IRQ];
        softirq += kcpustat_cpu(i).cpustat[CPUTIME_SOFTIRQ];
        steal += kcpustat_cpu(i).cpustat[CPUTIME_STEAL];
        guest += kcpustat_cpu(i).cpustat[CPUTIME_GUEST];
        guest_nice += kcpustat_cpu(i).cpustat[CPUTIME_GUEST_NICE];

        for (j = 0; j < NR_SOFTIRQS; j++) {
            unsigned int softirq_stat = kstat_softirqs_cpu(j, i);

            per_softirq_sums[j] += softirq_stat;

            per_softirq_sums[j] += softirq_stat;
            sum_softirq += softirq_stat;
        }
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	*usage = nsec_to_clock_t(user + nice + system);
	others = nsec_to_clock_t(idle + iowait + irq + softirq +
			steal + guest + guest_nice);
#else
	*usage = cputime64_to_clock_t(user + nice + system);
	others = cputime64_to_clock_t(idle + iowait + irq + softirq +
			steal + guest + guest_nice);
#endif
	*total = *usage + others;
}
EXPORT_SYMBOL(cpuqos_get_cpu_load_once);

/* 
 * non-blocking procedure,
 * pass the previous results as args.
 */
u32 cpuqos_update_cpu_load(u64 *used_1, u64 *total_1)
{
	u64 used_2, total_2;
	u64 used, total;
	u32 avg_load;
	cpuqos_get_cpu_load_once(&used_2, &total_2);
	/* ((work_2 - work_1) / (total_2 - total_1) * 100) */
	used = used_2 - *used_1;
	total = total_2 - *total_1;
	avg_load = (used * 100) / total;
	atomic_set(&cpuqos_cpu_load, (int)avg_load);
	*used_1 = used_2;
	*total_1 = total_2;
	return (u32)avg_load;
}
EXPORT_SYMBOL(cpuqos_update_cpu_load);

