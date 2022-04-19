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

#include "mdrv_mzc_drv.h"
#include "mhal_mzc_hal.h"
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#ifdef CONFIG_ARM
#include <asm/fixmap.h>
#include <mm/mm.h>
#include <asm/pgtable.h>
#include <linux/mm.h>
#include <asm/highmem.h>
#endif
struct dev_mzc empty_dev;

/* The value is for zram to reduce wastage due to unusable space
 * left at end of each zspage
 */
#define MAX_SIZE_FOR_ZRAM	3248

static int enc_out_size_thr = MAX_SIZE_FOR_ZRAM;

#ifdef CONFIG_MP_ZRAM_PERFORMANCE
static inline unsigned long long timediff (struct timeval begin, struct timeval end)
{
	return ((end.tv_sec - begin.tv_sec) * 1000000 + (end.tv_usec - begin.tv_usec));
}

unsigned long long duration_enc = 0, duration_dec = 0;
#endif

static inline unsigned long VA_to_PA(unsigned long VA)
{
#ifdef CONFIG_ARM
	unsigned long PA;
	pte_t *ptep;
	struct page *page;
	if (VA >= (void *)FIXADDR_START || (VA >= PKMAP_ADDR(0) && VA < PKMAP_ADDR(LAST_PKMAP)))
	{
		ptep = pte_offset_kernel(pmd_off_k(VA),VA);
		page = pte_page(*ptep);
		PA = page_to_phys(page) + (VA & (PAGE_SIZE-1));	
	}
	else {
		PA = virt_to_phys(VA);
	}
	return PA;
#else
	return virt_to_phys(VA);
#endif
}


#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
static irqreturn_t MDrv_lzc_cmdq_handler(int irq, void *data) {
	struct dev_mzc *mzc_data = data;
	int outlen;
	int w_output_count, r_output_count;
	int process_id;
	int is_write = 0, is_read = 0;
	int i;
	unsigned int irq_status;

	irq_status = mhal_lzc_cmdq_handler(&w_output_count, &r_output_count, &is_write, &is_read);
	if (is_read == 1) {
		for (i=1; i<=r_output_count; i++) {
			mhal_ldec_cmdq_handler(&outlen,&process_id);
			mzc_data->read_cmdq[process_id].out_len = outlen;
			wake_up(&mzc_data->read_cmdq[process_id].wq);
		}
	}
	if (is_write == 1) {
		for (i=1; i<=w_output_count; i++) {
			mhal_lenc_cmdq_handler(&outlen,&process_id);
			mzc_data->write_cmdq[process_id].out_len = outlen;
			wake_up(&mzc_data->write_cmdq[process_id].wq);
		}
	}

	mhal_lzc_cmdq_clear_irq(irq_status);

	if (!is_write && !is_read) {
		return IRQ_NONE;
	} else {
		return IRQ_HANDLED;
	}
}
#else
static int MDrv_lenc_cmdq_done(cmdq_node *zram_write_cmdq) {
	int target_id;
	int outlen;
	outlen = mhal_lenc_cmdq_done(&target_id);
	if (outlen != 0) {
		zram_write_cmdq[target_id].out_len = outlen;
		return 1;
	}
	else {
		return 0;
	}
}

static int MDrv_ldec_cmdq_done(cmdq_node *zram_read_cmdq) {
	int target_id;
	int outlen;
	outlen = mhal_ldec_cmdq_done(&target_id);
	if (outlen != 0) {
		zram_read_cmdq[target_id].out_len = outlen;
		return 1;
	}
	else {
		return 0;
	}
}
#endif


int MDrv_lzc_cmdq_init(void) {
	int irq_flag = SA_INTERRUPT | IRQF_SHARED | IRQF_ONESHOT;
	int i;
	mhal_regptr_set();
	mhal_acp_common_init();
    mhal_lzc_cmdq_init(enc_out_size_thr);
    printk(KERN_DEBUG"[Debug] Initialize MZC \n");
	spin_lock_init(&empty_dev.r_lock);
	spin_lock_init(&empty_dev.w_lock);
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	spin_lock_init(&empty_dev.r_cmdq_lock);
	spin_lock_init(&empty_dev.w_cmdq_lock);
	sema_init(&empty_dev.r_cmdq_sem, CMDQ_ELEMENT_NUM);
	sema_init(&empty_dev.w_cmdq_sem, CMDQ_ELEMENT_NUM);
	for (i = 0; i < CMDQ_ELEMENT_NUM; i++) {
		init_waitqueue_head(&empty_dev.write_cmdq[i].wq);
		init_waitqueue_head(&empty_dev.read_cmdq[i].wq);
		empty_dev.write_cmdq[i].used = 0;
		empty_dev.read_cmdq[i].used = 0;
	}
	mhal_lzc_irq_mask_all_except_timeout();
	return request_threaded_irq(E_IRQ_LZC,MDrv_lzc_cmdq_handler,NULL,irq_flag,"lzc_irq_handler",&empty_dev);
#else
	mhal_lzc_irq_mask_all();
	return 0;
#endif
}

void MDrv_lzc_cmdq_exit(void) {
	mhal_lzc_cmdq_exit();
}

static int MDrv_lenc_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id, cmdq_node *zram_write_cmdq) {
	zram_write_cmdq[process_id].out_len = 0;
	return mhal_lenc_cmdq_set(VA_to_PA(in_addr),VA_to_PA(out_addr),process_id);
}

static int MDrv_ldec_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id, cmdq_node *zram_read_cmdq) {
	zram_read_cmdq[process_id].out_len = 0;
	return mhal_ldec_cmdq_set(VA_to_PA(in_addr),VA_to_PA(out_addr),process_id);
}

#ifdef CONFIG_MP_MZCCMDQ_HW_SPLIT
void MDrv_ldec_literal_bypass_set(int value) {
	mhal_ldec_literal_bypass_set(value);
}

static int MDrv_ldec_cmdq_set_split(unsigned long in_addr1, unsigned long in_addr2, unsigned long out_addr, unsigned int process_id, cmdq_node *zram_read_cmdq) {
	zram_read_cmdq[process_id].out_len = 0;
	return mhal_ldec_cmdq_set_split(VA_to_PA(in_addr1),VA_to_PA(in_addr2),VA_to_PA(out_addr),process_id);
}
#endif

#ifdef CONFIG_MP_ZSM
int MDrv_lenc_cmdq_run(unsigned long in_addr, unsigned long out_addr,
		unsigned int *dst_len, u32 *crc32)
#else
int MDrv_lenc_cmdq_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len)
#endif
{
	cmdq_node *pCmdq = empty_dev.write_cmdq;
	int ret = 0;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	struct timeval compress_begin, compress_end;
#endif

#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	int wait_ret;
	int preempt_cnt, i;
	int process_id = -1;

	preempt_cnt = preempt_count();
	for (i = 0; i < preempt_cnt; i++)
		preempt_enable();

	down(&empty_dev.w_cmdq_sem);
	spin_lock(&empty_dev.w_cmdq_lock);
	for (i = 0; i < CMDQ_ELEMENT_NUM; i++) {
		if (!pCmdq[i].used) {
			pCmdq[i].used = 1;
			process_id = i;
			break;
		}
	}
	if (process_id == -1) {
		spin_unlock(&empty_dev.w_cmdq_lock);
		up(&empty_dev.w_cmdq_sem);
		pr_err("[MZC][ENC] Can't find an available command element\n");
		for (i = 0; i < preempt_cnt; i++)
			preempt_disable();
		return -2;
	}
	spin_unlock(&empty_dev.w_cmdq_lock);

	spin_lock(&empty_dev.w_lock);
	MDrv_lenc_cmdq_set(in_addr, out_addr, process_id, pCmdq);
#ifdef CONFIG_MP_ZSM
	*crc32 = MDrv_lenc_crc32_get(out_addr, *dst_len);
#endif
	spin_unlock(&empty_dev.w_lock);
	wait_ret = wait_event_timeout(pCmdq[process_id].wq,(pCmdq[process_id].out_len != 0), msecs_to_jiffies(1));
	if (wait_ret == 0) {
		printk(KERN_ALERT"[MZC] Enc: wait_event_timeout\n");
		ret = -1;
	}
	if (pCmdq[process_id].out_len == -1)
		ret = -1;
	*dst_len = pCmdq[process_id].out_len;
	spin_lock(&empty_dev.w_cmdq_lock);
	pCmdq[process_id].used = 0;
	spin_unlock(&empty_dev.w_cmdq_lock);
	up(&empty_dev.w_cmdq_sem);
	for (i = 0; i < preempt_cnt; i++)
		preempt_disable();
#else
	const int process_id = 0;
	spin_lock(&empty_dev.w_lock);
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	do_gettimeofday(&compress_begin);
#endif
	MDrv_lenc_cmdq_set(in_addr, out_addr, process_id, pCmdq);
	while(!MDrv_lenc_cmdq_done(pCmdq))
	{
		;
	}
	if (pCmdq[process_id].out_len == -1) {
		ret = -1;
	}
	*dst_len = pCmdq[process_id].out_len;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	do_gettimeofday(&compress_end);
	duration_enc = timediff(compress_begin, compress_end);
#endif
#ifdef CONFIG_MP_ZSM
	*crc32 = MDrv_lenc_crc32_get(out_addr, *dst_len);
#endif
	spin_unlock(&empty_dev.w_lock);
#endif

	return ret;
}


int MDrv_ldec_cmdq_run(unsigned long in_addr, unsigned long out_addr)
{
	cmdq_node *pCmdq = empty_dev.read_cmdq;
	int ret = 0;
	int retry_count = 3;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	struct timeval decompress_begin, decompress_end;
#endif

#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	int preempt_cnt, i;
	int process_id = -1;

	preempt_cnt = preempt_count();
	for (i = 0; i < preempt_cnt; i++)
		preempt_enable();
	down(&empty_dev.r_cmdq_sem);
	spin_lock(&empty_dev.r_cmdq_lock);
	for (i = 0; i < CMDQ_ELEMENT_NUM; i++) {
		if (!pCmdq[i].used) {
			pCmdq[i].used = 1;
			process_id = i;
			break;
		}
	}
	if (process_id == -1) {
		spin_unlock(&empty_dev.r_cmdq_lock);
		up(&empty_dev.r_cmdq_sem);
		pr_err("[MZC][DEC] Can't find an available command element\n");
		for (i = 0; i < preempt_cnt; i++)
			preempt_disable();
		return -2;
	}
	spin_unlock(&empty_dev.r_cmdq_lock);

DEC_AGAIN:
	spin_lock(&empty_dev.r_lock);
	MDrv_ldec_cmdq_set(in_addr, out_addr, process_id, pCmdq);
	spin_unlock(&empty_dev.r_lock);
	wait_event_timeout(pCmdq[process_id].wq,(pCmdq[process_id].out_len != 0), msecs_to_jiffies(1));
	if (pCmdq[process_id].out_len != PAGE_SIZE) {
		printk(KERN_ALERT" failed !! %lu  try again \n", pCmdq[process_id].out_len);
		if (retry_count) {
			retry_count--;
			goto DEC_AGAIN;
		}
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE)
	 	ret = pCmdq[process_id].out_len;
	spin_lock(&empty_dev.r_cmdq_lock);
	pCmdq[process_id].used = 0;
	spin_unlock(&empty_dev.r_cmdq_lock);
	up(&empty_dev.r_cmdq_sem);
	for (i = 0; i < preempt_cnt; i++)
		preempt_disable();
#else
	const int process_id = 0;

	spin_lock(&empty_dev.r_lock);
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	do_gettimeofday(&decompress_begin);
#endif
DEC_AGAIN:
	MDrv_ldec_cmdq_set(in_addr, out_addr, process_id, pCmdq);
	while(!MDrv_ldec_cmdq_done(pCmdq))
	{
		;
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE) {
		printk(KERN_ALERT" failed !! %lu  try again\n", pCmdq[process_id].out_len);
		if (retry_count) {
			retry_count--;
			goto DEC_AGAIN;
		}
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE)
	    ret = pCmdq[process_id].out_len;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	do_gettimeofday(&decompress_end);
	duration_dec = timediff(decompress_begin, decompress_end);
#endif
	spin_unlock(&empty_dev.r_lock);
#endif

	return ret;
}
#ifdef CONFIG_MP_MZCCMDQ_HW_SPLIT
int MDrv_ldec_cmdq_run_split(unsigned long in_addr1, unsigned long in_addr2, unsigned long out_addr)
{
	cmdq_node *pCmdq = empty_dev.read_cmdq;
	int ret = 0;
	int retry_count = 3;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	struct timeval decompress_begin, decompress_end;
#endif
	const int process_id = 0;

	spin_lock(&empty_dev.r_lock);
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	do_gettimeofday(&decompress_begin);
#endif
DEC_AGAIN:
	MDrv_ldec_cmdq_set_split(in_addr1, in_addr2, out_addr, process_id, pCmdq);
	while(!MDrv_ldec_cmdq_done(pCmdq))
	{
		;
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE) {
		if (retry_count) {
			printk(KERN_ALERT" failed !! %lu  try again\n", pCmdq[process_id].out_len);
			retry_count--;
			goto DEC_AGAIN;
		}
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE)
	    ret = pCmdq[process_id].out_len;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	do_gettimeofday(&decompress_end);
	duration_dec = timediff(decompress_begin, decompress_end);
#endif
	spin_unlock(&empty_dev.r_lock);

	return ret;
}
#endif
#ifdef ENABLE_SINGLE_MODE
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
static irqreturn_t MDrv_lzc_single_handler(int irq, void *data)
{
	struct dev_mzc *mzc_data = data;
	unsigned int process_id;
	int w_outlen, r_outlen;
	int is_write = 0, is_read = 0;

	mhal_lzc_single_handler(&w_outlen, &r_outlen, &is_write, &is_read);
	if (is_write == 1) {
		process_id = empty_dev.lenc_single_id;
		mzc_data->write_cmdq[process_id].out_len = w_outlen;
		wake_up(&mzc_data->write_cmdq[process_id].wq);
	}
	if (is_read == 1) {
		process_id = empty_dev.ldec_single_id;
		mzc_data->read_cmdq[process_id].out_len = r_outlen;
		wake_up(&mzc_data->read_cmdq[process_id].wq);
	}
	if (!is_write && !is_read) {
		return IRQ_NONE;
	} else {
		return IRQ_HANDLED;
	}
}
#else
static int MDrv_lenc_single_done(cmdq_node *zram_write_cmdq) {
	int outlen;
	outlen = mhal_lenc_single_done();
	if (outlen > 0) {
		zram_write_cmdq[empty_dev.ldec_single_id].out_len = outlen;
		return 1;
	}
	else {
		return 0;
	}
}

static int MDrv_ldec_single_done(cmdq_node *zram_read_cmdq) {
	int outlen;
	outlen = mhal_ldec_single_done();
	if (outlen != 0) {
		zram_read_cmdq[empty_dev.ldec_single_id].out_len = outlen;
		return 1;
	}
	else {
		return 0;
	}
}
#endif

int MDrv_lzc_single_init(void) {
	int irq_flag = SA_INTERRUPT | IRQF_SHARED | IRQF_ONESHOT;
	int i;
	mhal_regptr_set();
	mhal_acp_common_init();
	mhal_lzc_single_init(enc_out_size_thr);
	empty_dev.ldec_single_id = 0;
	empty_dev.lenc_single_id = 0;
	spin_lock_init(&empty_dev.r_lock);
	spin_lock_init(&empty_dev.w_lock);
	for (i = 0; i < CMDQ_ELEMENT_NUM; i++) {
		init_waitqueue_head(&empty_dev.write_cmdq[i].wq);
		init_waitqueue_head(&empty_dev.read_cmdq[i].wq);
		empty_dev.write_cmdq[i].used = 0;
		empty_dev.read_cmdq[i].used = 0;
	}
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	return request_threaded_irq(E_IRQ_LZC,MDrv_lzc_single_handler,NULL,irq_flag,"lzc_irq_handler",&empty_dev);
#else
	return 0;
#endif
}

void MDrv_lzc_single_exit(void) {
	mhal_lzc_single_exit();
}

static int MDrv_lenc_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id, cmdq_node *zram_write_cmdq) {
	if (process_id != empty_dev.lenc_single_id) {
	 	return -1;
	} else {	
		zram_write_cmdq[process_id].out_len = 0;
		return mhal_lenc_single_set(VA_to_PA(in_addr),VA_to_PA(out_addr),process_id);
	}
}

static int MDrv_ldec_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id, cmdq_node *zram_read_cmdq) {
	if (process_id != empty_dev.ldec_single_id) {
	 	return -1;
	} else {
		zram_read_cmdq[process_id].out_len = 0;
		return mhal_ldec_single_set(VA_to_PA(in_addr),VA_to_PA(out_addr),process_id);
	}
}

#ifdef CONFIG_MP_ZSM
int MDrv_lenc_single_run(unsigned long in_addr, unsigned long out_addr,
		unsigned int *dst_len, u32 *crc32)
#else
int MDrv_lenc_single_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len)
#endif
{
	cmdq_node *pCmdq = &empty_dev.write_cmdq;
	int ret = 0;
	const int process_id = 0;

	spin_lock(&empty_dev.w_lock);
	MDrv_lenc_single_set(in_addr, out_addr, process_id, pCmdq);
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	int wait_ret;
#ifdef CONFIG_MP_ZSM
	*crc32 = MDrv_lenc_crc32_get(out_addr, *dst_len);
#endif
	spin_unlock(&empty_dev.w_lock);
	wait_ret = wait_event_timeout(pCmdq[process_id].wq,(pCmdq[process_id].out_len != 0), msecs_to_jiffies(1));
	if (wait_ret == 0) {
		printk(KERN_ALERT"[MZC] Enc: wait_event_timeout\n");
		ret = -1;
	}
	if (pCmdq[process_id].out_len == -1)
		ret = -1;
	*dst_len = pCmdq[process_id].out_len;
#else
	while(!MDrv_lenc_single_done(pCmdq))
	{
		;
	}
	if (pCmdq[process_id].out_len == -1) {
		ret = -1;
	}
	*dst_len = pCmdq[process_id].out_len;
#ifdef CONFIG_MP_ZSM
	*crc32 = MDrv_lenc_crc32_get(out_addr, *dst_len);
#endif
	spin_unlock(&empty_dev.w_lock);
#endif

	return ret;
}


int MDrv_ldec_single_run(unsigned long in_addr, unsigned long out_addr)
{
	cmdq_node *pCmdq = &empty_dev.read_cmdq;
	int ret = 0;
	int retry_count = 3;
	const int process_id = 0;

	spin_lock(&empty_dev.r_lock);
DEC_AGAIN:
	MDrv_ldec_single_set(in_addr, out_addr, process_id, pCmdq);
#ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
	spin_unlock(&empty_dev.r_lock);
	wait_event_timeout(pCmdq[process_id].wq,(pCmdq[process_id].out_len != 0), msecs_to_jiffies(1));
	if (pCmdq[process_id].out_len != PAGE_SIZE) {
		printk(KERN_ALERT" failed !! %lu  try again \n", pCmdq[process_id].out_len);
		if (retry_count) {
			retry_count--;
			spin_lock(&empty_dev.r_lock);
			goto DEC_AGAIN;
		}
	}
#else
	while(!MDrv_ldec_single_done(pCmdq))
	{
		;
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE) {
		printk(KERN_ALERT" failed !! %lu  try again \n", pCmdq[process_id].out_len);
		if (retry_count) {
			retry_count--;
			goto DEC_AGAIN;
		}
	}
	if (pCmdq[process_id].out_len != PAGE_SIZE)
	    ret = pCmdq[process_id].out_len;
	spin_unlock(&empty_dev.r_lock);
#endif

	return ret;
}
#endif

void MDrv_ldec_cycle_info(void) {
	int active_cycle;
	int free_cycle;
	int page_count;
	mhal_ldec_cycle_info(&active_cycle,&free_cycle,&page_count);
	printk(KERN_ALERT" ldec active cycle %u   free cycle %u   page count %u  \n",active_cycle,free_cycle,page_count);
}

void MDrv_lenc_cycle_info(void) {
	int active_cycle;
	int free_cycle;
	int page_count;
	mhal_lenc_cycle_info(&active_cycle,&free_cycle,&page_count);
	printk(KERN_ALERT" lenc active cycle %u   free cycle %u   page count %u  \n",active_cycle,free_cycle,page_count);
}

static void MDrv_ldec_state_save(int wait)
{
	mhal_ldec_state_save(wait);
}

static void MDrv_lenc_state_save(int wait)
{
	mhal_lenc_state_save(wait);
}

static int MDrv_ldec_state_check(void)
{
	return mhal_ldec_state_check();
}

static int MDrv_lenc_state_check(void)
{
	return mhal_lenc_state_check();
}

#ifdef CONFIG_MP_ZSM
u32 MDrv_lenc_crc32_get(unsigned char *cmem, unsigned int comp_len)
{
#ifdef CONFIG_MP_MZCCMDQ_HW_SPLIT	
    u32 crc32 = mhal_lenc_crc_get();
#else
    unsigned int crc16 = mhal_lenc_crc_get();
    u32 crc32;
	if (unlikely(comp_len > enc_out_size_thr)) {
		crc32 = crc16 | (cmem[0] << 16) | (cmem[1] << 24);
	} else {
		crc32 = crc16 | (comp_len << 16) | ((cmem[comp_len - 1] >> 4) << 28);
	}
#endif	
    return crc32;
}
#endif
#ifdef CONFIG_MP_MZCCMDQ_HYBRID_HW
#ifdef CONFIG_MP_ZSM
int MDrv_lenc_hybrid_cmdq_run(unsigned long in_addr, unsigned long out_addr,
        unsigned int *dst_len, u32 *crc32)
#else
int MDrv_lenc_hybrid_cmdq_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len)
#endif
{
	cmdq_node *pCmdq = empty_dev.write_cmdq;
	int ret = 0;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
	struct timeval compress_begin, compress_end;
#endif
	const int process_id = 0;

	if (spin_trylock(&empty_dev.w_lock))
	{
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
		do_gettimeofday(&compress_begin);
#endif
		MDrv_lenc_cmdq_set(in_addr, out_addr, process_id, pCmdq);
		while(!MDrv_lenc_cmdq_done(pCmdq))
		{
			;
		}
		if (pCmdq[process_id].out_len == -1)
			ret = -1;
		*dst_len = pCmdq[process_id].out_len;
#ifdef CONFIG_MP_ZRAM_PERFORMANCE
		do_gettimeofday(&compress_end);
		duration_enc = timediff(compress_begin, compress_end);
#endif
#ifdef CONFIG_MP_ZSM
		*crc32 = MDrv_lenc_crc32_get(out_addr, *dst_len);
#endif
		spin_unlock(&empty_dev.w_lock);
		return ret;
	}
	else
	{
		return MZC_ERR_NO_ENC_RESOURCE;
	}
}

#ifdef ENABLE_SINGLE_MODE
#ifdef CONFIG_MP_ZSM
int MDrv_lenc_hybrid_single_run(unsigned long in_addr, unsigned long out_addr,
        unsigned int *dst_len, u32 *crc32)
#else
int MDrv_lenc_hybrid_single_run(unsigned long in_addr, unsigned long out_addr, unsigned int *dst_len)
#endif
{
	cmdq_node *pCmdq = &empty_dev.write_cmdq;
	int ret = 0;
	const int process_id = 0;

	if (spin_trylock(&empty_dev.w_lock)) {
		MDrv_lenc_single_set(in_addr, out_addr, process_id, pCmdq);
		while(!MDrv_lenc_single_done(pCmdq))
		{
			;
		}
		if (pCmdq[process_id].out_len == -1) {
			ret = -1;
		}
		*dst_len = pCmdq[process_id].out_len;
#ifdef CONFIG_MP_ZSM
		*crc32 = MDrv_lenc_crc32_get(out_addr, *dst_len);
#endif
		spin_unlock(&empty_dev.w_lock);
		return ret;
	}
	else
	{
		return MZC_ERR_NO_ENC_RESOURCE;
	}
}
#endif
#endif



static s32 mstar_mzc_suspend(struct platform_device *pDev_st, pm_message_t state)
{
 #ifdef CONFIG_MP_MZCCMDQ_HW_INTERRUPT_MODE
    ktime_t start_time = ktime_get();
    while(MDrv_ldec_state_check() == 2 || MDrv_lenc_state_check() == 2)
    {
        if (ktime_to_ns((ktime_sub(ktime_get(),start_time))) > 1000000) {
             printk(KERN_ALERT" time-out !!lenc %d   ldec %d \n",MDrv_lenc_state_check(),MDrv_ldec_state_check());
             BUG_ON(1);
        }
    }
#else
    int ldec_state, lenc_state;
    MDrv_ldec_state_check();
    MDrv_lenc_state_check();
    ldec_state = MDrv_ldec_state_check();
    lenc_state = MDrv_lenc_state_check();
    if (ldec_state == 1) //output = 1
    {
        MDrv_ldec_state_save(0);
    }
    else if (ldec_state == 2) //input = 1
    {
        MDrv_ldec_state_save(1); //have to wait for HW decode done
    }
    if (lenc_state == 1) //output = 1
    {
        MDrv_lenc_state_save(0);
    }
    else if (lenc_state == 2) //input = 1
    {
        MDrv_lenc_state_save(1); //have to wait for HW decode done
    }
#endif
	return 0;
}

static s32 mstar_mzc_resume(struct platform_device *pdev)
{
#ifndef ENABLE_SINGLE_MODE
	MDrv_lzc_cmdq_init();
#else
	MDrv_lzc_single_init();
#endif
	return 0;
}

static s32 mstar_mzc_probe(struct platform_device *pdev)
{
#ifdef ENABLE_SINGLE_MODE
	MDrv_lzc_single_init();
#else
	MDrv_lzc_cmdq_init();
#endif
	return 0;
}

static s32 mstar_mzc_remove(struct platform_device *pdev)
{
#ifdef ENABLE_SINGLE_MODE
	MDrv_lzc_single_exit();
#else
	MDrv_lzc_cmdq_exit();
#endif
	return 0;
}

static struct platform_driver mzc_driver =
{

	.probe	= mstar_mzc_probe,
	.remove	= mstar_mzc_remove,
    #ifdef CONFIG_PM
    .suspend = mstar_mzc_suspend,
    .resume = mstar_mzc_resume,
    #endif

    .driver  =
    {
        .name = "MZC",
        .owner = THIS_MODULE,
    },
};

static struct platform_device sg_mstar_mzc_st =
{
     .name = "MZC",
     .id = 0,
     .resource = NULL,
     .num_resources = 0,
};

static int __init mzc_dev_init(void){

	return platform_device_register(&sg_mstar_mzc_st);
}

static void __exit mzc_dev_exit(void){
	platform_device_unregister(&sg_mstar_mzc_st);
}

EXPORT_SYMBOL(MDrv_lenc_cmdq_run);
EXPORT_SYMBOL(MDrv_ldec_cmdq_run);
#ifdef CONFIG_MP_MZCCMDQ_HYBRID_HW
EXPORT_SYMBOL(MDrv_lenc_hybrid_cmdq_run);
#endif


module_platform_driver(mzc_driver);
module_init(mzc_dev_init);
module_exit(mzc_dev_exit);

module_param(enc_out_size_thr, int, 0644);
MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("MZC");
MODULE_LICENSE("GPL");

