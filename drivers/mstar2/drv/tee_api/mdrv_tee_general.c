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

static void cpu_smccc_smc(unsigned long a0, unsigned long a1,
				unsigned long a2, unsigned long a3,
				unsigned long a4, unsigned long a5,
				unsigned long a6, unsigned long a7,
				struct arm_smccc_res_mstar *res);

/* Simple wrapper functions to be able to use a function pointer */
static void cpu_smccc_smc(unsigned long a0, unsigned long a1,
				unsigned long a2, unsigned long a3,
				unsigned long a4, unsigned long a5,
				unsigned long a6, unsigned long a7,
				struct arm_smccc_res_mstar *res)
{
	arm_smccc_smc_mstar(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

unsigned long tee_fast_call_cmd(struct arm_smccc_res_mstar *param)
{
	struct arm_smccc_res_mstar res = {0};

	if(TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_OPTEE){

		cpu_smccc_smc(param->a0, param->a1, param->a2, param->a3, 0, 0, 0, 0, &res);
		memcpy(param, &res, sizeof(struct arm_smccc_res_mstar));

	}
	else{
		printk("\033[0;32;31m [TEE] %s %d NOT TEE Env don't use the API\033[m\n",__func__,__LINE__);
	}
	return res.a0;
}
EXPORT_SYMBOL(tee_fast_call_cmd);
