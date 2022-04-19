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

/*

typedef enum {
    TZ_LLV_R0,   // = 0, Arm GP R0
    TZ_LLV_R1,
    TZ_LLV_R2,
    TZ_LLV_R3,
    TZ_LLV_R4,
    TZ_LLV_R5,
    TZ_LLV_R6,
    TZ_LLV_R7,
    TZ_LLV_R8,
    TZ_LLV_R9,
    TZ_LLV_R10,
    TZ_LLV_R11,
    TZ_LLV_R12,
    TZ_LLV_R13,  // = 13, Arm Stack Pointer
    TZ_LLV_R14,  // = 14, Arm Link Register
    TZ_LLV_CPSR, // = 15, Arm Current Processor Status Register
    TZ_LLV_SPSR,
    TZ_LLV_REGS_MAX
}TZ_LLV_REGS;
*/
/*
typedef enum {

    TZ_LLV_MODE_SVC,
    TZ_LLV_MODE_USR,
    TZ_LLV_MODE_IRQ,
    TZ_LLV_MODE_FIQ,
    TZ_LLV_MODE_MON,
    TZ_LLV_MODE_MAX

}TZ_LLV_CPU_MODE;
*/
/*
typedef enum {
      TZ_LLV_RES_OK                      //= 0
     ,TZ_LLV_RES_ERR                     //= 1
     ,TZ_LLV_RES_UND                     //= 2
     ,TZ_LLV_RES_MAX
}TZ_LLV_Result;
*/

//#include <linux/spinlock.h>
//#include <linux/mutex.h>
#include <linux/rwsem.h>
struct smc_struct{

    volatile unsigned int cmd1;
    volatile unsigned int cmd2;

    unsigned int smc_flag;
    volatile unsigned int* private_data;
    struct rw_semaphore smc_lock;
};

void smc_call(struct smc_struct *tz);

//TZ_LLV_Result Tz_LLV_Ctrl_Get_Reg(TZ_LLV_REGS reg, int *preg);
//TZ_LLV_Result Tz_LLV_Ctrl_SMC(unsigned int *args_1, unsigned int *args_2, unsigned int *args_3, unsigned int *args_4);

