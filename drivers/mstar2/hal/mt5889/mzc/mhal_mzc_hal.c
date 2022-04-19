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
LZC_REGS comm_reg;
volatile unsigned short *regptr[regoffsetnum];
extern ptrdiff_t mstar_pm_base;
unsigned long ldec_cmdq_info = 0;
unsigned long ldec_output_level = 0;
unsigned long lenc_cmdq_info = 0;
unsigned long lenc_output_level = 0;

void mhal_regptr_set(void)
{
	int i;
	for(i=0;i<regoffsetnum;i++)
	{
		regptr[i] = (volatile unsigned short*)((mstar_pm_base + ((LZC_BANK) << 9) + ((i) * 2))) ;
	}

}

void mhal_regptr_unset(void)
{
}

u16 LZC_REG_READ(u16 _offset)
{
	u16 regval = *regptr[_offset];
	return regval;
}

void LZC_REG_WRITE(u16 _offset, u16 val)
{
	*regptr[_offset] = val;
}

void LZC_REG_WRITE_32(u16 _offset, u32 val)
{
	volatile unsigned int *addr32 = (volatile unsigned int *)regptr[_offset];
	*addr32 = val;
}

unsigned int LZC_REG_READ_32(u16 _offset)
{
	u32 regval = *((volatile unsigned int *)regptr[_offset]);
	return regval;
}

void ACP_REG_WRITE(u16 _offset,u16 val)
{
	volatile unsigned short *acpptr = (volatile unsigned short*)(((mstar_pm_base + ((ACP_BANK) << 9) + ((_offset) * 4))));
	*acpptr = val;
}

u16 ACP_REG_READ(u16 _offset)
{
	volatile unsigned short *acpptr = (volatile unsigned short*)(((mstar_pm_base + ((ACP_BANK) << 9) + ((_offset) * 4))));
	unsigned short outputvalue = *acpptr;
	return outputvalue;
}

void ACACHE_REG_WRITE(u16 _offset,u16 val)
{
	volatile unsigned short *acacheptr = (volatile unsigned short*)(((mstar_pm_base + ((ACACHE_BANK) << 9) + ((_offset) * 4))));
	*acacheptr = val;
}

u16 ACACHE_REG_READ(u16 _offset)
{
	volatile unsigned short *acacheptr = (volatile unsigned short*)(((mstar_pm_base + ((ACACHE_BANK) << 9) + ((_offset) * 4))));
	unsigned short outputvalue = *acacheptr;
	return outputvalue;
}


void mhal_acp_common_init(void)
{
    u16 orig_acp_info, acp_set, orig_acache_info, acache_set;
    orig_acp_info = ACP_REG_READ(0x10);
    acp_set = (orig_acp_info & 0xfffe);
    orig_acache_info = ACACHE_REG_READ(0x2b);
    acache_set = (orig_acache_info | 0xff);
    ACP_REG_WRITE(0x10,acp_set);
    ACACHE_REG_WRITE(0x2b,acache_set);
}


static void mhal_lenc_outputcount_read(int *outcount)
{
	*outcount = ((LZC_REG_READ(RO_LENC_OUT_INQ_LV1) >> 8) & 0x1f);
}


static void mhal_ldec_outputcount_read(int *outcount)
{
	*outcount = ((LZC_REG_READ(RO_LDEC_OUT_INQ_LV1) >> 8) & 0x1f);
}



void mhal_lenc_cmdq_handler(int *outlen,int *process_id)
{
	unsigned int cmdq_out_data;
	u16 cmdq_out_data_hi;
	u16 cmdq_out_data_lo;
	int timeout = 0;
	cmdq_out_data = LZC_REG_READ_32(LENC_OUTQ_INFO_LO);
	cmdq_out_data_hi = (cmdq_out_data >> 16); //take last 16 bits
	cmdq_out_data_lo = (cmdq_out_data & 0xffff);
	*process_id = (cmdq_out_data_hi & 0xf); //take first 4 bits
	if ((cmdq_out_data_lo & CMDQ_LENC_NORMAL_END) == CMDQ_LENC_NORMAL_END) { //first bit for normal_end
		u16 out_len = ((cmdq_out_data_lo >> 3) & 0x1fff) ;// take bit 3~15
		*outlen = out_len;
	}
	else {
		if ((cmdq_out_data_lo & CMDQ_LENC_TIME_OUT) == CMDQ_LENC_TIME_OUT) {
			timeout = 1;
			*outlen = -1;
		}
		if ((cmdq_out_data_lo & CMDQ_LENC_SIZE_ERROR) == CMDQ_LENC_SIZE_ERROR) {  //over 3K
			*outlen = PAGE_SIZE;
		}
	}
}

void mhal_ldec_cmdq_handler(int *outlen, int *process_id)
{
	u16 cmdq_out_data;
	int timeout = 0;
	cmdq_out_data = LZC_REG_READ(LDEC_OUTQ_INFO);
	*process_id = cmdq_out_data >> 6; //take last 4 bits
	if ((cmdq_out_data & CMDQ_LDEC_NORMAL_END) == CMDQ_LDEC_NORMAL_END)
	{
		*outlen = PAGE_SIZE;
	}
	else
	{
		*outlen = (cmdq_out_data & CMDQ_LDEC_ERROR_TYPE_BIT_POS);
		if ((cmdq_out_data & CMDQ_LDEC_TIME_OUT) == CMDQ_LDEC_TIME_OUT)
			timeout = 1;
	}
}

unsigned int mhal_lzc_cmdq_handler(int *w_outputcount, int *r_outputcount, int *is_write, int *is_read)
{
	unsigned int irq_type = LZC_REG_READ_32(MZC_ST_IRQ_CPU_LO);
	if (irq_type & (LENC_RELATED)) {
		*is_write = 1;
		mhal_lenc_outputcount_read(w_outputcount);
	}
	if (irq_type & (LDEC_RELATED)) {
		*is_read = 1;
		mhal_ldec_outputcount_read(r_outputcount);
	}

    return irq_type;
}

void mhal_lzc_cmdq_clear_irq(unsigned int irq_status)
{
    LZC_REG_WRITE_32(LZC_IRQ_STATUS_LO, irq_status);
}

void mhal_lzc_cmdq_init(int enc_out_size_thr)
{
	mhal_lzc_common_init(enc_out_size_thr);
	mhal_lenc_cmdq_init();
	mhal_ldec_cmdq_init();
}

void mhal_lzc_cmdq_exit(void)
{
	mhal_regptr_unset();
}

void mhal_lzc_irq_mask_all(void)
{
    LZC_REG_WRITE(MZC_IRQ_MASK_LO,MASK_IRQ_ALL);
    LZC_REG_WRITE(MZC_IRQ_MASK_HI,MASK_IRQ_ALL);
}

void mhal_lzc_irq_mask_all_except_timeout(void)
{
	LZC_REG_WRITE(MZC_IRQ_MASK_LO,MASK_IRQ_ALL_EXCEPT_TIMER_AND_NUMBER);
	LZC_REG_WRITE(MZC_IRQ_MASK_HI,MASK_IRQ_ALL);
}

int mhal_lenc_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id)
{
#ifdef CONFIG_ARM	
	LZC_REG_WRITE_32(LENC_INQ_DATA, ((in_addr>>12)  << 8) | (0 << 4) | process_id);
	LZC_REG_WRITE_32(LENC_INQ_OUT_ST_ADR_LSB_LO, out_addr>>3);
#else
	LZC_REG_WRITE_32(LENC_INQ_DATA, ((in_addr>>12)  << 8) | (out_addr >> 35) << 4 | process_id);
	LZC_REG_WRITE_32(LENC_INQ_OUT_ST_ADR_LSB_LO, ((out_addr >> 3) & 0xffffffff)); //bit 3 ~ bit34
#endif
	return 0;
}


int mhal_lenc_cmdq_done(int *target_id)
{
	if(((LZC_REG_READ(RO_LENC_OUT_INQ_LV1) >> 8) & 0x1f) == 0 && lenc_output_level == 0)
	{
		return 0;
	}
	else if (lenc_output_level == 0) // not from STR
	{
		unsigned int cmdq_out_data;
		u16 cmdq_out_data_hi;
		u16 cmdq_out_data_lo;
		u16 process_id;
		cmdq_out_data = LZC_REG_READ_32(LENC_OUTQ_INFO_LO);
		cmdq_out_data_hi = (cmdq_out_data >> 16); //take last 16 bits
		cmdq_out_data_lo = (cmdq_out_data & 0xffff);
		process_id = (cmdq_out_data_hi & 0xf); //take first 4 bits
		*target_id = process_id;
		if ((cmdq_out_data_lo & CMDQ_LENC_NORMAL_END) == CMDQ_LENC_NORMAL_END) { //first bit for normal_end
			u16 outlen = ((cmdq_out_data_lo >> 3) & 0x1fff) ;// take bit 3~15
			return outlen;
		}
		else if ((cmdq_out_data_lo & CMDQ_LENC_SIZE_ERROR) == CMDQ_LENC_SIZE_ERROR)  //over 3K
		{
			return PAGE_SIZE;;
		}
		else //time_out
		{
			return -1; //should never happen
		}
	}
	else // from STR
	{
		unsigned int cmdq_out_data = lenc_cmdq_info;
		u16 cmdq_out_data_hi;
		u16 cmdq_out_data_lo;
		u16 process_id;
		cmdq_out_data_hi = (cmdq_out_data >> 16); //take last 16 bits
		cmdq_out_data_lo = (cmdq_out_data & 0xffff);
		process_id = (cmdq_out_data_hi & 0xf); //take first 4 bits
		*target_id = process_id;
		if ((cmdq_out_data_lo & CMDQ_LENC_NORMAL_END) == CMDQ_LENC_NORMAL_END) { //first bit for normal_end
			u16 outlen = ((cmdq_out_data_lo >> 3) & 0x1fff) ;// take bit 3~15
			return outlen;
		}
		else if ((cmdq_out_data_lo & CMDQ_LENC_SIZE_ERROR) == CMDQ_LENC_SIZE_ERROR)  //over 3K
		{
			return PAGE_SIZE;;
		}
		else //time_out
		{
			return -1; //should never happen
		}
	}
}

int mhal_ldec_cmdq_done(int *target_id)
{
	if(((LZC_REG_READ(RO_LDEC_OUT_INQ_LV1) >> 8) & 0x1f) == 0  && ldec_output_level == 0)
	{
		return 0;
	}
	else if (ldec_output_level == 0) // not from STR
	{
		u16 cmdq_out_data;
		u16 process_id;
		cmdq_out_data = LZC_REG_READ(LDEC_OUTQ_INFO);
		process_id = cmdq_out_data >> 6; //take last 4 bits
		*target_id = process_id;
		if ((cmdq_out_data & CMDQ_LDEC_NORMAL_END) == CMDQ_LDEC_NORMAL_END)
		{
			return PAGE_SIZE;
		}
		else
		{
			return (cmdq_out_data & CMDQ_LDEC_ERROR_TYPE_BIT_POS);
		}
	}
	else
	{
		u16 cmdq_out_data = ldec_cmdq_info;
		u16 process_id;
		process_id = cmdq_out_data >> 6; //take last 4 bits
		*target_id = process_id;
		if ((cmdq_out_data & CMDQ_LDEC_NORMAL_END) == CMDQ_LDEC_NORMAL_END)
		{
			return PAGE_SIZE;
		}
		else
		{
			return (cmdq_out_data & CMDQ_LDEC_ERROR_TYPE_BIT_POS);
		}
	}
}

static void mhal_lenc_single_handler(u16 irq_type_hi, int *outlen)
{
	if ((irq_type_hi & LENC_IRQ_NORMAL_END) == LENC_IRQ_NORMAL_END) {
		*outlen = LZC_REG_READ(RO_LENC_BYTE_CNT) & 0x1fff;
	} else if ((irq_type_hi & LENC_IRQ_SIZE_ERROR) == LENC_IRQ_SIZE_ERROR) {
		*outlen = PAGE_SIZE;
	} else if ((irq_type_hi & LENC_IRQ_TIMEOUT) == LENC_IRQ_TIMEOUT) {
		*outlen = -1;
	} else {
		*outlen = -1;
	}
	// get CRC
	// LZC_REG_READ(LENC_CRC_OUT);
}

static void mhal_ldec_single_handler(u16 irq_type_hi, int *outlen)
{
	if ((irq_type_hi & LDEC_IRQ_NORMAL_END) == LDEC_IRQ_NORMAL_END) {
		*outlen = PAGE_SIZE;
	} else {
		*outlen = irq_type_hi & SINGLE_MODE_LDEC_IRQ;
	}

	// get CRC
	// LZC_REG_READ(LDEC_CRC_OUT);
}

void mhal_lzc_single_handler(int *w_outlen, int *r_outlen, int *is_write, int *is_read)
{
	//u16 irq_type_lo = LZC_REG_READ(MZC_ST_IRQ_CPU_LO);
	u16 irq_type_hi = LZC_REG_READ(MZC_ST_IRQ_CPU_HI);

	if (irq_type_hi & SINGLE_MODE_LENC_IRQ) {
		*is_write = 1;
		mhal_lenc_single_handler(irq_type_hi, w_outlen);
	}
	if (irq_type_hi & SINGLE_MODE_LDEC_IRQ) {
		*is_read = 1;
		mhal_ldec_single_handler(irq_type_hi, r_outlen);
	}
	LZC_REG_WRITE(LZC_IRQ_STATUS_HI, CLEAR_IRQ(irq_type_hi));
}

void mhal_lzc_single_init(int enc_out_size_thr)
{
	mhal_lzc_common_init(enc_out_size_thr);

	comm_reg.reg1d = LZC_REG_READ(LDEC_GENERAL_SETTING);
	comm_reg.reg40 = LZC_REG_READ(LENC_GENERAL_SETTING);
	comm_reg.reg_lenc_crc_en = LENC_CRC_DEFAULT_ENABLE_STATE;  //set crc enable or disable
	comm_reg.reg_lenc_crc_mode = LENC_CRC_DEFAULT_MODE; //set crc mode
	comm_reg.reg_ldec_crc_en = LDEC_CRC_DEFAULT_ENABLE_STATE;  //set crc enable or disable
	comm_reg.reg_ldec_crc_mode = LDEC_CRC_DEFAULT_MODE; //set crc mode
	LZC_REG_WRITE(LDEC_GENERAL_SETTING,comm_reg.reg1d);
	LZC_REG_WRITE(LENC_GENERAL_SETTING,comm_reg.reg40);

	// mask IRQs
	LZC_REG_WRITE(MZC_IRQ_MASK_LO,MASK_IRQ_ALL);
	LZC_REG_WRITE(MZC_IRQ_MASK_HI,MASK_SINGLE_MODE_IRQ);
}

void mhal_lzc_single_exit(void)
{
	mhal_regptr_unset();
}


int mhal_lenc_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id)
{
	//comm_reg.reg00 = LZC_REG_READ(CMDQ_TOP_SETTING);
	comm_reg.reg40 = LZC_REG_READ(LENC_GENERAL_SETTING);
	comm_reg.reg41 = LZC_REG_READ(LENC_IN_ST_ADR);
	comm_reg.reg42 = LZC_REG_READ(LENC_OUT_ST_ADR_MSB);
	comm_reg.reg43 = LZC_REG_READ(LENC_OUT_ST_ADR_LSB_LO);
	comm_reg.reg44 = LZC_REG_READ(LENC_OUT_ST_ADR_LSB_HI);

	comm_reg.reg_lenc_in_st_adr_hi = (in_addr >> 28);
	comm_reg.reg_lenc_in_st_adr_lo = ((in_addr >> 12) & 0xffff);
	comm_reg.reg_lenc_out_st_adr_lsb_hi = (out_addr >> 19);
	comm_reg.reg_lenc_out_st_adr_lsb_lo = ((out_addr >> 3) & 0xffff) ;
	//comm_reg.reg_lenc_soft_rstz = RESET;
	//LZC_REG_WRITE(CMDQ_TOP_SETTING,comm_reg.reg00);
	//comm_reg.reg_lenc_soft_rstz = NOT_RESET;
	//LZC_REG_WRITE(CMDQ_TOP_SETTING,comm_reg.reg00);
	comm_reg.reg_w1c_lenc_start = 1;
#ifdef CONFIG_ARM
	comm_reg.reg_lenc_out_st_adr_msb = 0;
#else
	comm_reg.reg_lenc_out_st_adr_msb = (out_addr >> 35);
#endif
	comm_reg.reg_lenc_op_mode = SINGLE_MODE;

	LZC_REG_WRITE(LENC_IN_ST_ADR,comm_reg.reg41);
	LZC_REG_WRITE(LENC_OUT_ST_ADR_MSB,comm_reg.reg42);
	LZC_REG_WRITE(LENC_OUT_ST_ADR_LSB_LO,comm_reg.reg43);
	LZC_REG_WRITE(LENC_OUT_ST_ADR_LSB_HI,comm_reg.reg44);
	LZC_REG_WRITE(LENC_GENERAL_SETTING,comm_reg.reg40);
	comm_reg.reg_w1c_lenc_start = 0; //due to a write one clear register
	return 0;
}

int mhal_ldec_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id)
{

	//comm_reg.reg00 = LZC_REG_READ(CMDQ_TOP_SETTING);
	comm_reg.reg1d = LZC_REG_READ(LDEC_GENERAL_SETTING);
	comm_reg.reg1e = LZC_REG_READ(LDEC_IN_ST_ADR_LSB_LO);
	comm_reg.reg1f = LZC_REG_READ(LDEC_IN_ST_ADR_LSB_HI);
	comm_reg.reg20 = LZC_REG_READ(LDEC_OUT_ST_ADR_LO);
	comm_reg.reg21 = LZC_REG_READ(LDEC_OUT_ST_ADR_HI);

	comm_reg.reg_ldec_in_st_adr_lsb_hi = in_addr >> 16;
	comm_reg.reg_ldec_in_st_adr_lsb_lo = in_addr & 0xffff;
	comm_reg.reg_ldec_out_st_adr_hi = (out_addr >> 28);
	comm_reg.reg_ldec_out_st_adr_lo = ((out_addr >> 12) & 0xffff);
	//comm_reg.reg_ldec_soft_rstz = RESET;
	//LZC_REG_WRITE(CMDQ_TOP_SETTING,comm_reg.reg00);
	//comm_reg.reg_ldec_soft_rstz = NOT_RESET;
	//LZC_REG_WRITE(CMDQ_TOP_SETTING,comm_reg.reg00);
	comm_reg.reg_w1c_ldec_start = 1;
#ifdef CONFIG_ARM
	comm_reg.reg_ldec_in_st_adr_msb = 0;
#else
	comm_reg.reg_ldec_in_st_adr_msb = (in_addr >> 32);
#endif
	comm_reg.reg_ldec_op_mode = SINGLE_MODE;

	LZC_REG_WRITE(LDEC_IN_ST_ADR_LSB_LO,comm_reg.reg1e);
	LZC_REG_WRITE(LDEC_IN_ST_ADR_LSB_HI,comm_reg.reg1f);
	LZC_REG_WRITE(LDEC_OUT_ST_ADR_LO,comm_reg.reg20);
	LZC_REG_WRITE(LDEC_OUT_ST_ADR_HI,comm_reg.reg21);
	LZC_REG_WRITE(LDEC_GENERAL_SETTING,comm_reg.reg1d);
	comm_reg.reg_w1c_ldec_start = 0; //due to a write one clear register
	return 0;
}

int mhal_lenc_single_done(void)
{

    u16 irq_type_hi = LZC_REG_READ(MZC_ST_IRQ_CPU_HI);
    u16 outlen;

	if ((irq_type_hi & LENC_IRQ_NORMAL_END) == LENC_IRQ_NORMAL_END) {
        LZC_REG_WRITE(LZC_IRQ_STATUS_HI, CLEAR_IRQ(LENC_IRQ_NORMAL_END));
        outlen = LZC_REG_READ(RO_LENC_BYTE_CNT) & 0x1fff;
	} else if ((irq_type_hi & LENC_IRQ_SIZE_ERROR) == LENC_IRQ_SIZE_ERROR) {
        LZC_REG_WRITE(LZC_IRQ_STATUS_HI, CLEAR_IRQ(LENC_IRQ_SIZE_ERROR));
        outlen = PAGE_SIZE;
	} else if ((irq_type_hi & LENC_IRQ_TIMEOUT) == LENC_IRQ_TIMEOUT) {
        LZC_REG_WRITE(LZC_IRQ_STATUS_HI, CLEAR_IRQ(LENC_IRQ_TIMEOUT));
        outlen = -1;
    } else {
        outlen = 0;
    }

    return outlen;
}

int mhal_ldec_single_done(void)
{
    u16 irq_type_hi = LZC_REG_READ(MZC_ST_IRQ_CPU_HI);
    u16 outlen;

    if ((irq_type_hi & LDEC_IRQ_NORMAL_END) == LDEC_IRQ_NORMAL_END) {
        LZC_REG_WRITE(LZC_IRQ_STATUS_HI, CLEAR_IRQ(LDEC_IRQ_NORMAL_END));
        outlen = PAGE_SIZE;
    } else {
        //LZC_REG_WRITE(LZC_IRQ_STATUS_HI, CLEAR_IRQ(SINGLE_MODE_LDEC_IRQ));
        outlen = irq_type_hi & SINGLE_MODE_LDEC_IRQ;
    }

    return outlen;
}


void mhal_ldec_state_save(int wait)
{
	if (wait)
	{
		while(((LZC_REG_READ(RO_LDEC_OUT_INQ_LV1) >> 8) & 0x1f) == 0)
		{
			;
		}
	}
	ldec_cmdq_info = LZC_REG_READ(LDEC_OUTQ_INFO);
	ldec_output_level = 1;

}

void mhal_lenc_state_save(int wait)
{
	if (wait)
	{
		while(((LZC_REG_READ(RO_LENC_OUT_INQ_LV1) >> 8) & 0x1f) == 0)
		{
			;
		}
	}
	lenc_cmdq_info = LZC_REG_READ_32(LENC_OUTQ_INFO_LO);
	lenc_output_level = 1;
}

int mhal_lenc_state_check(void)
{
	if (((LZC_REG_READ(RO_LENC_OUT_INQ_LV1) >> 8) & 0x1f) != 0)
		return 1;
	if (((LZC_REG_READ(RO_LENC_OUT_INQ_LV1)) & 0x1f) != 0)
		return 2;
	else
		return 0;
}


int mhal_ldec_state_check(void)
{
	if (((LZC_REG_READ(RO_LDEC_OUT_INQ_LV1) >> 8) & 0x1f) != 0)
		return 1;
	if (((LZC_REG_READ(RO_LDEC_OUT_INQ_LV1)) & 0x1f) != 0)
		return 2;
	else
		return 0;
}
