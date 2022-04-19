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

#ifndef MZC_DRV_H
#define MZC_DRV_H
#include <linux/wait.h>
#include <linux/semaphore.h>

//#define ENABLE_SINGLE_MODE

#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
#error "CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE has not been supported yet!"
#endif

typedef struct cmdq_node {
	unsigned long out_len;
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	wait_queue_head_t wq;
	int used;
#endif
} cmdq_node;

int MDrv_lzc_cmdq_init(void);

void MDrv_lzc_cmdq_exit(void);

#ifdef CONFIG_MP_ZSM
int MDrv_lenc_cmdq_run(unsigned long in_addr, unsigned long out_addr,
        unsigned int *dst_len, u32 *crc32);
#else
int MDrv_lenc_cmdq_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len);
#endif

int MDrv_ldec_cmdq_run(unsigned long in_addr, unsigned long out_addr);

#ifdef CONFIG_MP_MZCCMDQ_HW_SPLIT
void MDrv_ldec_literal_bypass_set(int value);

int MDrv_ldec_cmdq_run_split(unsigned long in_addr1, unsigned long in_addr2, unsigned long out_addr);
#endif

int MDrv_lzc_single_init(void);

void MDrv_lzc_single_exit(void);

#ifdef CONFIG_MP_ZSM
int MDrv_lenc_single_run(unsigned long in_addr, unsigned long out_addr,
        unsigned int *dst_len, u32 *crc32);
#else
int MDrv_lenc_single_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len);
#endif

int MDrv_ldec_single_run(unsigned long in_addr,unsigned long out_addr);

void MDrv_ldec_cycle_info(void);

void MDrv_lenc_cycle_info(void);

#ifdef CONFIG_MP_ZSM
u32 MDrv_lenc_crc32_get(unsigned char *cmem, unsigned int clen);
#endif
#ifdef CONFIG_MP_MZCCMDQ_HYBRID_HW
#ifdef CONFIG_MP_ZSM
int MDrv_lenc_hybrid_cmdq_run(unsigned long in_addr, unsigned long out_addr,
		unsigned int *dst_len, u32 *crc32);
#else
int MDrv_lenc_hybrid_cmdq_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len);
#endif
#ifdef CONFIG_MP_ZSM
int MDrv_lenc_hybrid_single_run(unsigned long in_addr, unsigned long out_addr,
		unsigned int *dst_len, u32 *crc32);
#else
int MDrv_lenc_hybrid_single_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len);
#endif
#endif

#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
#define CMDQ_ELEMENT_NUM	(CONFIG_NR_CPUS)
#else
#define CMDQ_ELEMENT_NUM	(1)
#endif

struct dev_mzc
{
	unsigned int ldec_single_id;
	unsigned int lenc_single_id;
	cmdq_node write_cmdq[CMDQ_ELEMENT_NUM];
	cmdq_node read_cmdq[CMDQ_ELEMENT_NUM];
    spinlock_t r_lock;
    spinlock_t w_lock;
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	spinlock_t r_cmdq_lock;
	spinlock_t w_cmdq_lock;
	struct semaphore r_cmdq_sem;
	struct semaphore w_cmdq_sem;
#endif
};

enum {
	MZC_ERR_NO_ENC_RESOURCE = -1000,
};
#ifdef CONFIG_MP_MZCCMDQ_HYBRID_HW
#define INITIAL_VALUE -1
enum {
	LZO = 0,
	MZC,
	NR_COMP_ALGO,
};
#endif

#endif
