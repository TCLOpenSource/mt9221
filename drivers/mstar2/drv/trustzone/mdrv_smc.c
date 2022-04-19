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

#include "mdrv_smc.h"
#include <linux/kernel.h>


#define TZ_LLV_DPRINTK(fmt, arg...) //printk(KERN_WARNING"%s:%d " fmt,__FUNCTION__,__LINE__,## args)

void smc_call(struct smc_struct *smc)
{
    volatile unsigned int u32cpsr;
    volatile unsigned int arg1, arg2;

    arg1 = smc->cmd1;
    arg2 = smc->cmd2;

    printk("XX cmd1 = %x, cmd2 = %x, arg1 = %x, arg2 = %x\n", smc->cmd1, smc->cmd2, arg1, arg2);

   __asm__ volatile (
        "mrs %0, cpsr\n\t"
        "push {r0, r4-r12, lr}\n\t"
        "mov r1, %1\n\t"
        "mov r2, %2\n\t"
        "mov  r3, sp  \n\t"
        "smc  0       \n\t"
        "mov sp, r3\n\t"
        "pop {r0, r4-r12, lr}\n\t"
        "mov %1, r1\n\t"
        "mov %2, r2\n\t"
        "msr cpsr, %0\n\t"
        : "+&r" (u32cpsr), "+&r" (arg1), "+&r" (arg2)
        :
        : "r0", "r1", "r2", "r3", "memory");

    smc->cmd1 = arg1;
    smc->cmd2 = arg2;

    printk("XX cmd1 = %x, cmd2 = %x, arg1 = %x, arg2 = %x\n", smc->cmd1, smc->cmd2, arg1, arg2);
}
