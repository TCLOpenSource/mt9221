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

#include <linux/cpufreq.h>
#if (defined CONFIG_MP_DVFS_FORCE_USE_ONE_FREQ) || (defined CONFIG_MP_DVFS_FORCE_PINGO_TEST)
#define MAX_CPU_FREQ 1100000
#elif (defined CONFIG_MP_DVFS_VID_ONLY)
#define MAX_CPU_FREQ 1008123
#else
#define MAX_CPU_FREQ 1400000
#endif
#define MIN_CPU_FREQ 1008000

#if (defined CONFIG_MP_DVFS_FORCE_USE_ONE_FREQ) || (defined CONFIG_MP_DVFS_FORCE_PINGO_TEST) || (defined CONFIG_MP_DVFS_VID_ONLY)
#define TRANSITION_LATENCY 1000000
#else
#define TRANSITION_LATENCY 100000
#endif
// ( TRANSITION_LATENCY / 1000 ) * LATENCY_MULTIPLIER ==> DVFS on_demand sampling rate, > 100000

/* Minimum CLK support, each accross 20 percentage */
enum
{
DC_ZERO, DC_20PT, DC_40PT, DC_60PT, DC_80PT, DC_MAX_NUM
};

/* Voltage for each cpu_load percentage */
#define V_ZERO MIN_CPU_FREQ
#define V_20PT 1200000
#define V_40PT 1200000
#define V_60PT 1300000
#define V_80PT MAX_CPU_FREQ

#define ON_DEMAND_TABLE_INTERVAL (100/DC_MAX_NUM)

#define DVFS_FREQ_LEVEL_MAX_INDEX 32

static struct cpufreq_frequency_table hal_freq_table[DVFS_FREQ_LEVEL_MAX_INDEX] = {
{0, 216000},
{1, 400000},
{2, 600000},
{3, 750000},
{4, 800000},
{5, 850000},
{6, 900000},
{7, 950000},
{8, 1000000},
{9, 1008000},
{10, 1050000},
{11, 1100000},
{12, 1150000},
{13, 1200000},
{14, 1250000},
{15, 1300000},
{16, 1350000},
{17, 1400000},
{18, 1450000},
{19, 1500000},
{20, 1550000},
{21, 1600000},
{22, 1650000},
{23, 1700000},
{24, 1740000},
{25, 1750000},
{26, 1800000},
{27, 1850000},
{28, 1900000},
{29, 1950000},
{30, 2000000},
{31, CPUFREQ_TABLE_END}
};

/* data that will copy to user_space when need change voltage */
typedef struct
{
unsigned int to_userspace_cpufreq;
unsigned int to_userspace_voltage;
unsigned int to_userspace_voltage_type;
unsigned int to_userspace_change_cnt;
unsigned int to_userspace_cluster_id;
} ON_Demand_To_Userspace;

/* data from user space when voltage is changed */
typedef struct
{
unsigned int from_userspace_finished_change_cnt;
int from_userspace_voltage_change_result;
unsigned int from_userspace_cluster_id;
} ON_Demand_From_Userspace;

#ifdef CONFIG_CPU_FREQ
void change_cpus_timer(char *caller, unsigned int target_freq, unsigned int cpu_id);
#endif
