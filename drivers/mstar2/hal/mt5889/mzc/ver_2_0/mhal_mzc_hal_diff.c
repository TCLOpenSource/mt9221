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


#include "mhal_mzc_hal.h"
#include <uapi/linux/psci.h>
#include <linux/version.h>
//#define ENABLE_DFS
static LZC_REGS comm_reg;
extern volatile unsigned short *regptr[regoffsetnum];
extern ptrdiff_t mstar_pm_base;
extern unsigned long ldec_cmdq_info;
extern unsigned long ldec_output_level;
extern unsigned long lenc_cmdq_info;
extern unsigned long lenc_output_level;

#ifdef CONFIG_MP_PLATFORM_ARM
extern uint32_t isPSCI;
#endif
void mhal_lzc_common_init(int enc_out_size_thr)
{
	// clk & freq
	volatile u16 *regptr;
	regptr = (volatile u16*) ((mstar_pm_base + ((FREQ_BANK) << 9) + MZC_FREQ_REGISTER * 4));
	*regptr = MZC_FULL_SPEED;
	regptr = (volatile u16*) ((mstar_pm_base + ((MZC_CLK_BANK) << 9) + MZC_CLCK_SETTING * 4));
	*regptr |= MZC_CLCK_FIRE;

	// ACP
	comm_reg.reg61 = LZC_REG_READ(ACP_AR);
	comm_reg.reg62 = LZC_REG_READ(ACP_AW);
#if (defined(CONFIG_MP_PLATFORM_ARM) && (LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)))
    if (isPSCI == PSCI_RET_SUCCESS)
    {
        comm_reg.reg_arprot = NON_SECURE_ACCESS;
	    comm_reg.reg_awprot = NON_SECURE_ACCESS;
    }
    else if (isPSCI == PSCI_RET_NOT_SUPPORTED)
    {
        comm_reg.reg_arprot = SECURE_ACCESS;
        comm_reg.reg_awprot = SECURE_ACCESS;
    }
	else
    {
		printk(KERN_ALERT"[MZC] Security state error %d \n",isPSCI);
		BUG_ON(1);
    }
#else
#if defined(CONFIG_TEE) //NON_SECURE
	comm_reg.reg_arprot = NON_SECURE_ACCESS;
	comm_reg.reg_awprot = NON_SECURE_ACCESS;
#else
    comm_reg.reg_arprot = SECURE_ACCESS;
    comm_reg.reg_awprot = SECURE_ACCESS;
#endif
#endif
    comm_reg.reg62 &= ACP_IDLE_SW_FORCE;
	LZC_REG_WRITE(ACP_AW,comm_reg.reg62);
	LZC_REG_WRITE(ACP_AR,comm_reg.reg61);


	// top setting
	comm_reg.reg00 = LZC_REG_READ(CMDQ_TOP_SETTING);
	comm_reg.reg_mzc_soft_rstz = RESET; //set 0
	comm_reg.reg_ldec_soft_rstz = NOT_RESET;
	comm_reg.reg_lenc_soft_rstz = NOT_RESET;
	LZC_REG_WRITE(CMDQ_TOP_SETTING,comm_reg.reg00);
	comm_reg.reg_mzc_soft_rstz = NOT_RESET; //set 1
	comm_reg.reg_ldec_soft_rstz = NOT_RESET;
	comm_reg.reg_lenc_soft_rstz = NOT_RESET;
	LZC_REG_WRITE(CMDQ_TOP_SETTING,comm_reg.reg00);

#ifdef ENABLE_DFS
	// enable DFS
	regptr = (volatile u16*) ((mstar_pm_base + ((L4_AXI_BANK) << 9) + DFS_ENABLE_ADDR * 4));
	*regptr &= ~1;  // disable dfs
	*regptr |= 1;   // enable dfs

	regptr = (volatile u16*) ((mstar_pm_base + ((L4_AXI_BANK) << 9) + DFS_FREQ_DIVISION_ADDR * 4));
	*regptr = 1;    // set to  1/32  mzc_clk.  Formula: (n / 32) * clk_mzc ,    n=[1,2,3, ...., 31]

	regptr = (volatile u16*) ((mstar_pm_base + ((L4_AXI_BANK) << 9) + DFS_UPDATE_ADDR * 4));
	*regptr |= 1;   // set dfs update

	// enable auto HW DFS switching
	// When lzc has loading, it's with full speed. When lzc has no loading, the clock is as DFS setting
	comm_reg.reg02 = LZC_REG_READ(CMDQ_TOP_SETTING_002);
	comm_reg.reg_mzc_fullspeed_mode = 0; // 0: HW control full speed, 1: SW control full speed
	LZC_REG_WRITE(CMDQ_TOP_SETTING_002,comm_reg.reg02);
#endif
	comm_reg.reg02 = LZC_REG_READ(CMDQ_TOP_SETTING_002);
	comm_reg.reg_ldec_clk_gate_en = 1;
    comm_reg.reg_lenc_clk_gate_en = 1;
	comm_reg.reg_mzc_v15_mode_en = 1;
    LZC_REG_WRITE(CMDQ_TOP_SETTING_002,comm_reg.reg02);

	comm_reg.reg45 = LZC_REG_READ(LENC_OUTSIZE_THR);
	comm_reg.reg_lenc_out_size_thr = enc_out_size_thr >> 4;
	LZC_REG_WRITE(LENC_OUTSIZE_THR,comm_reg.reg45);
    LZC_REG_WRITE(MZC_RSV7C,MZC_VERSION_SETTING);
}

void mhal_lenc_cmdq_init(void)
{
	comm_reg.reg40 = LZC_REG_READ(LENC_GENERAL_SETTING);
	comm_reg.reg18 = LZC_REG_READ(LZC_OUTQ_NUM_TH);
	comm_reg.reg1b = LZC_REG_READ(LENC_OUTQ_TIMER_TH_LO);
	comm_reg.reg1c = LZC_REG_READ(LENC_OUTQ_TIMER_TH_HI);
	comm_reg.reg_lenc_op_mode = CMDQ_MODE;
	LZC_REG_WRITE(LENC_GENERAL_SETTING,comm_reg.reg40);
	comm_reg.reg_w1c_lenc_start = 1;
	comm_reg.reg_lenc_outq_num_th = LENC_OUTPUT_THRESHOLD; //set output threshold
	comm_reg.reg_lenc_crc_en = LENC_CRC_DEFAULT_ENABLE_STATE;  //set crc enable or disable
	comm_reg.reg_lenc_crc_mode = LENC_CRC_DEFAULT_MODE; //set crc mode
	comm_reg.reg_lenc_outq_timer_th_lo =  LENC_CMDQ_TIMER_THR_LO; //set timer default
	comm_reg.reg_lenc_outq_timer_th_hi =  LENC_CMDQ_TIMER_THR_HI; //set timer default
	LZC_REG_WRITE(LENC_GENERAL_SETTING,comm_reg.reg40);
	comm_reg.reg_w1c_lenc_start = 0; //due to a write one clear register
	LZC_REG_WRITE(LZC_OUTQ_NUM_TH,comm_reg.reg18);
	LZC_REG_WRITE(LENC_OUTQ_TIMER_TH_LO,comm_reg.reg1b);
	LZC_REG_WRITE(LENC_OUTQ_TIMER_TH_HI,comm_reg.reg1c);
	comm_reg.reg_lenc_timeout_thr_lo = LENC_TIMEOUT_THRESHOLD & 0xffff;
	LZC_REG_WRITE(LENC_TIMEOUT_THR,comm_reg.reg46);
	comm_reg.reg47 = LZC_REG_READ(LENC_TIMEOUT_EN);
	comm_reg.reg_lenc_timeout_thr_hi = LENC_TIMEOUT_THRESHOLD >> 16;
	comm_reg.reg_lenc_cnt_sel = 0; // 0: wrap-around, 1: staurated
	LZC_REG_WRITE(LENC_TIMEOUT_EN,comm_reg.reg47);
	comm_reg.reg50 = LZC_REG_READ(LENC_PAGE_SYNC);
	comm_reg.reg_sync_lenc_reps_full_search = 0;
	LZC_REG_WRITE(LENC_PAGE_SYNC,comm_reg.reg50);
}

void mhal_ldec_cmdq_init(void)
{
	comm_reg.reg1d  = LZC_REG_READ(LDEC_GENERAL_SETTING);
	comm_reg.reg18 = LZC_REG_READ(LZC_OUTQ_NUM_TH);
	comm_reg.reg19 = LZC_REG_READ(LDEC_OUTQ_TIMER_TH_LO);
	comm_reg.reg1a = LZC_REG_READ(LDEC_OUTQ_TIMER_TH_HI);
	comm_reg.reg_ldec_op_mode = CMDQ_MODE;
	LZC_REG_WRITE(LDEC_GENERAL_SETTING,comm_reg.reg1d);
	comm_reg.reg_w1c_ldec_start = 1;
	comm_reg.reg_ldec_outq_num_th = LDEC_OUTPUT_THRESHOLD; //set output threshold
	comm_reg.reg_ldec_crc_en = LDEC_CRC_DEFAULT_ENABLE_STATE;  //set crc enable or disable
	comm_reg.reg_ldec_crc_mode = LDEC_CRC_DEFAULT_MODE; //set crc mode
	comm_reg.reg_ldec_outq_timer_th_lo =  LDEC_CMDQ_TIMER_THR_LO; //set timer default
	comm_reg.reg_ldec_outq_timer_th_hi =  LDEC_CMDQ_TIMER_THR_HI;	 //set timer default
	LZC_REG_WRITE(LDEC_GENERAL_SETTING,comm_reg.reg1d);
	comm_reg.reg_w1c_ldec_start = 0; //due to a write one clear register
	LZC_REG_WRITE(LZC_OUTQ_NUM_TH,comm_reg.reg18);
	LZC_REG_WRITE(LDEC_OUTQ_TIMER_TH_LO,comm_reg.reg19);
	LZC_REG_WRITE(LDEC_OUTQ_TIMER_TH_HI,comm_reg.reg1a);
	comm_reg.reg_ldec_timeout_thr_lo = LDEC_TIMEOUT_THRESHOLD & 0xffff;
	LZC_REG_WRITE(LDEC_TIMEOUT_THR,comm_reg.reg23);
	comm_reg.reg24 = LZC_REG_READ(LDEC_TIMEOUT_EN);
	comm_reg.reg_ldec_timeout_thr_hi = LDEC_TIMEOUT_THRESHOLD >> 16;
	comm_reg.reg_ldec_cnt_sel = 0; // 0: wrap-around, 1: staurated
	LZC_REG_WRITE(LDEC_TIMEOUT_EN,comm_reg.reg24);
}

int mhal_ldec_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id)
{
	ldec_cmdq_info = 0;
	ldec_output_level = 0;
#ifdef CONFIG_ARM
	unsigned long in_addr2 = ((in_addr & 0xfffff000) + 4096);
	LZC_REG_WRITE_32(LDEC_INQ_IN_ST_ADR_LSB_LO, in_addr);
	LZC_REG_WRITE_32(LDEC_INQ_DATA, ((out_addr >> 12) << 8) | (0<<4) | process_id);
	LZC_REG_WRITE_32(LDEC_IN_ST_ADR_4K, (in_addr2 >> 12) | (1 << 24));	
#else
	unsigned long in_addr2 = ((in_addr & 0xffffff000) + 4096);
	LZC_REG_WRITE_32(LDEC_INQ_IN_ST_ADR_LSB_LO, (in_addr & 0xffffffff));
	LZC_REG_WRITE_32(LDEC_INQ_DATA, ((out_addr >> 12) << 8) | ((in_addr & 0xf00000000) >> 28)| process_id);
	LZC_REG_WRITE_32(LDEC_IN_ST_ADR_4K, ((in_addr2 >> 12) & 0xffffff) | (1 << 24));	
#endif
	return 0;
}

int mhal_ldec_cmdq_set_split(unsigned long in_addr1, unsigned long in_addr2, unsigned long out_addr, unsigned int process_id)
{
	ldec_cmdq_info = 0;
	ldec_output_level = 0;
	const int enable = 1;
#ifdef CONFIG_ARM
	LZC_REG_WRITE_32(LDEC_INQ_IN_ST_ADR_LSB_LO, in_addr1);
	LZC_REG_WRITE_32(LDEC_INQ_DATA, ((out_addr >> 12) << 8) | (0<<4) | process_id);
	LZC_REG_WRITE_32(LDEC_IN_ST_ADR_4K, (in_addr2 >> 12) | (enable << 24));	
#else
	LZC_REG_WRITE_32(LDEC_INQ_IN_ST_ADR_LSB_LO, (in_addr1 & 0xffffffff));
	LZC_REG_WRITE_32(LDEC_INQ_DATA, ((out_addr >> 12) << 8) | ((in_addr1 & 0xf00000000) >> 28)| process_id);
	LZC_REG_WRITE_32(LDEC_IN_ST_ADR_4K, ((in_addr2 >> 12) & 0xffffff) | (enable << 24));	
#endif
	return 0;
}

void mhal_ldec_cycle_info(int *active_cycle, int *free_cycle, int *page_count)
{
     comm_reg.reg24 = LZC_REG_READ(LDEC_TIMEOUT_EN);
     comm_reg.reg_ldec_cnt_sel_1 = 0;
     LZC_REG_WRITE(LDEC_TIMEOUT_EN,comm_reg.reg24);
     u32 orig_active_cycle = LZC_REG_READ_32(RO_LDEC_CYC_INFO);
     *active_cycle = orig_active_cycle;
     comm_reg.reg24 = LZC_REG_READ(LDEC_TIMEOUT_EN);
     comm_reg.reg_ldec_cnt_sel_1 = 1;
     LZC_REG_WRITE(LDEC_TIMEOUT_EN,comm_reg.reg24);
     u32 orig_free_cycle = LZC_REG_READ_32(RO_LDEC_CYC_INFO);    
     *free_cycle = orig_free_cycle;
     comm_reg.reg24 = LZC_REG_READ(LDEC_TIMEOUT_EN);
     comm_reg.reg_ldec_cnt_sel_1 = 2;
     LZC_REG_WRITE(LDEC_TIMEOUT_EN,comm_reg.reg24);
     u32 orig_page_count = LZC_REG_READ_32(RO_LDEC_CYC_INFO);     
     *page_count = orig_page_count;
}

void mhal_lenc_cycle_info(int *active_cycle, int *free_cycle, int *page_count)
{

     comm_reg.reg47 = LZC_REG_READ(LENC_TIMEOUT_EN);
     comm_reg.reg_lenc_cnt_sel_1 = 0;
     LZC_REG_WRITE(LENC_TIMEOUT_EN,comm_reg.reg47);
     u32 orig_active_cycle = LZC_REG_READ_32(RO_LENC_CYC_INFO);
     *active_cycle = orig_active_cycle;
     comm_reg.reg47 = LZC_REG_READ(LENC_TIMEOUT_EN);
     comm_reg.reg_lenc_cnt_sel_1 = 1;
     LZC_REG_WRITE(LENC_TIMEOUT_EN,comm_reg.reg47);
     u32 orig_free_cycle = LZC_REG_READ_32(RO_LENC_CYC_INFO);
     *free_cycle = orig_free_cycle;
     comm_reg.reg47 = LZC_REG_READ(LENC_TIMEOUT_EN);
     comm_reg.reg_lenc_cnt_sel_1 = 2;
     LZC_REG_WRITE(LENC_TIMEOUT_EN,comm_reg.reg47);
     u32 orig_page_count = LZC_REG_READ_32(RO_LENC_CYC_INFO);     
     *page_count = orig_page_count;
}

unsigned long mhal_lenc_crc_get(void)
{
	return LZC_REG_READ_32(LENC_CRC_OUT);
}


void mhal_ldec_literal_bypass_set(int value)
{
	if (value < 0 || value > 2)
	{
		BUG_ON(1);
	}
	else
	{
		comm_reg.reg50 = LZC_REG_READ(LENC_PAGE_SYNC);
		comm_reg.reg_sync_lenc_lit_bypass_bins = value;
		LZC_REG_WRITE(LENC_PAGE_SYNC,comm_reg.reg50);
	}
}
