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

#include <linux/printk.h>
#include <linux/string.h>
#include <asm/io.h>

#include "arm-smccc_mstar.h"
#include "mdrv_types.h"
#include "mdrv_tee_general.h"
#include "mdrv_tee_cpu_state.h"

/* General CPU STATE API FOR ARM*/
static unsigned int cpu_freq_from_tee = 0xffffffff;

extern void Chip_Flush_Cache_Range_VA_PA(unsigned long u32VAddr,unsigned long u32PAddr,unsigned long u32Size); //Clean & Invalid L1/L2 cache
extern void Chip_Clean_Cache_Range_VA_PA(unsigned long u32VAddr,unsigned long u32PAddr,unsigned long u32Size); //Clean & Invalid L1/L2 cache

bool TEE_CPUSTATE_SUPPORT(void){
	struct cpu_opt_state cmd_res;
	unsigned long cmd_res_bus_address = virt_to_phys((void *)&cmd_res);
	struct arm_smccc_res_mstar param = {0};

	if(cpu_freq_from_tee != 0xffffffff)
		return cpu_freq_from_tee;

	memset(&cmd_res, 0, sizeof(struct cpu_opt_state));
	cmd_res.cmd = CPU_STATE_QUERY_FREQ_FEATURE_SUPPORT;

	Chip_Flush_Cache_Range_VA_PA((unsigned long) &cmd_res,
					(unsigned long) cmd_res_bus_address,
					sizeof(struct cpu_opt_state));

#ifdef CONFIG_ARM64
	param.a0 = CPU_STATE_SUPPORT;
	param.a1 = cmd_res_bus_address & 0xffffffff;
	param.a2 = (cmd_res_bus_address >> 32) & 0xffffffff;
	param.a3 = 0;
#else
	param.a0 = CPU_STATE_SUPPORT;
	param.a1 = cmd_res_bus_address & 0xffffffff;
	param.a2 = 0;
	param.a3 = 0;
#endif
	if(tee_fast_call_cmd(&param) == 0)
	{
		printk("\033[0;32;31m [TEE] %s %d CPU FREQUERY use TEE flow\033[m\n",__func__,__LINE__);
		cpu_freq_from_tee = 1;
	}
	else{
		printk("\033[0;32;31m [TEE] %s %d CPU FREQUERY Not Support use NORMAL flow\033[m\n",__func__,__LINE__);
		cpu_freq_from_tee = 0;
	}

	return cpu_freq_from_tee;
}

unsigned int TEE_CPUSTATE_QUERY_CPU_FREQ(unsigned int cpu_id,unsigned int *freq)
{
	struct cpu_opt_state cmd_res;
	unsigned long cmd_res_bus_address = virt_to_phys((void *)&cmd_res);
	struct arm_smccc_res_mstar param = {0};

	memset(&cmd_res, 0, sizeof(struct cpu_opt_state));
	cmd_res.cmd = CPU_STATE_QUERY_FREQ;
	cmd_res.data[0] = cpu_id;

	Chip_Flush_Cache_Range_VA_PA((unsigned long) &cmd_res,
					(unsigned long) cmd_res_bus_address,
					sizeof(struct cpu_opt_state));

#ifdef CONFIG_ARM64
	param.a0 = CPU_STATE_SUPPORT;
	param.a1 = cmd_res_bus_address & 0xffffffff;
	param.a2 = (cmd_res_bus_address >> 32) & 0xffffffff;
	param.a3 = 0;
#else
	param.a0 = CPU_STATE_SUPPORT;
	param.a1 = cmd_res_bus_address & 0xffffffff;
	param.a2 = 0;
	param.a3 = 0;
#endif
	if(tee_fast_call_cmd(&param) == 0)
	{
		Chip_Clean_Cache_Range_VA_PA((unsigned long) &cmd_res,
										 (unsigned long) cmd_res_bus_address,
										 sizeof(struct cpu_opt_state));
		*freq = cmd_res.data[1];
	}
	else{
		printk("\033[0;32;31m [TEE] %s %d CPU FREQUERY OPTEE Not Impelement use NORMAL flow\033[m\n",__func__,__LINE__);
		cpu_freq_from_tee = 0;
		return CPU_STATE_RETRY;
	}

	return CPU_STATE_SUCCESS;
}
