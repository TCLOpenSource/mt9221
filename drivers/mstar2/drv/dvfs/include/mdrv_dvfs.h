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

#ifndef __MDRV_DVFS_H__
#define __MDRV_DVFS_H__

#ifndef __MDRV_TYPES_H__
#include "mdrv_types.h"
#endif

#ifndef __MHAL_DVFS_H__
#include "mhal_dvfs.h"
#endif

#include <linux/version.h>
//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define DVFS_DRV_INFO(x, args...)               //{printk(x, ##args);}
#define DVFS_DRV_DEBUG(x, args...)              //{printk(x, ##args);}

#define CONFIG_DVFS_MAX_CPU_CLOCK                         0
#define CONFIG_DVFS_MIN_CPU_CLOCK                         1
#define CONFIG_DVFS_IR_BOOTS_CPU_CLOCK                    2
#define CONFIG_DVFS_OVER_TEMPERATURE_PROTECT_CPU_CLOCK    3
#define CONFIG_DVFS_TABLE_MAX_CPU_CLOCK                   4


// 1: Enable 0: Disable (M7221/M7622/M7331/C2P)
#define DEBUG_OVER_TEMP           1
#define DVFS_MAX_QUERY            5
#define DVFS_DEFAULT_CLOCK        1000

#define CPU_AUTO_TEST_MAX_FREQ    3000000
//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
U32     MDrvDvfsProc(U32 dwInputCpuClock, U8 dwCpu);
void    MDrvDvfsInit(void);
void    MDrvDvfsCpuDisplay(U8 dwCpu);
void    MDrvDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage, U8 dwCpu);
void    MDrvDvfsCorePowerAdjustment(U32 dwCorePowerVoltage, U8 dwCpu);
U32     MDrvDvfsGetCpuFreq(U8 dwCpu);

U32     MDrvDvfsQueryCpuClock(U32 dwCpuClockType, U8 dwCpu);
U32     MDrvDvfsQueryCpuClockByTemperature(U8 dwCpu);

U32     MDrvDvfsGetOverTemperatureFlag(U8 dwCluster);

U32 MDrvDvfsGetCPUPowerType(U8 dwCpu);
U32 MDrvDvfsGetCpuTemperature(U8 dwCpu);
U32 MDrvDvfsGetVoltage(U8 dwCpu);
U32 MDrvDvfsGetSidd(void);
U32 MDrvDvfsGetOsc(U8 dwCpu);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
void MDrvDvfsGetDvfsTable(U8 dwCpu, dvfs_opp_table *opp_table);
#endif
U8 MDrvDvfsGetFreqTable(U8 dwCpu, struct cpufreq_frequency_table **freq_table);
#endif

int MDrvDvfsGetCpuCluster(unsigned int cpu);
int MDrvDvfsGetCpuClusterMainCpu(unsigned int cpu);

int MDrvDvfsGetStatus(unsigned int cpu);
int MDrvDvfsGetTemperatureOffset(unsigned int cpu);
void MDrvDvfsSetTemperatureOffset(unsigned int cpu,int set_offset);
int MDrvDvfsVerifyVoltage(unsigned int cpu, unsigned int freq);
int MDrvDvfsSetStatus(unsigned int status,unsigned int cpu);

void MDrvDvfsOpenFile(char *path);
void MDrvDvfsCloseFile(void);
