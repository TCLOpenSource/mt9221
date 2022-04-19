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

#ifndef MZC_HAL_DIFF_H
#define MZC_HAL_DIFF_H

typedef unsigned short  U16;

#define LDEC_CRC_OUT 0x0c

#define LENC_CRC_OUT 0x14

#define LDEC_CRC_ROUND_CNT 0x25

#define RO_LDEC_CYC_INFO 0x26

#define RO_LDEC_CRC_OUT 0x28

#define LDEC_IN_ST_ADR_4K 0x2a

#define LDEC_IN_ST_ADR_4K_EN 0x2b

#define LDEC_INQ_INFO 0x2d

#define LDEC_INQ_DEBUG 0x2e



#define LENC_CRC_ROUND_CNT 0x48

#define RO_LENC_CYC_INFO 0x4a

#define RO_LENC_CRC_OUT 0x4c



#define LENC_INQ_INFO 0x57

#define LENC_INQ_DEBUG 0x58

typedef struct _lzc_reg {
    union {
        struct {
            U16 reg_w1c_mzc_start:1;
            U16 reg_mzc_soft_rstz:1;
            U16 reg_ldec_soft_rstz:1;
            U16 reg_lenc_soft_rstz:1;
            U16 reg_mzc_ver_rid:12;
        };
        U16 reg00;
    };

    union {
        struct {
            U16 reg_mzc_pkt_start_marker:8;
            U16 reg_mzc_core_rstz_cnt:5;
            U16 reg_mzc_acp_rstz_cnt:3;
        };
        U16 reg01;
    };

	union {
        struct {
            U16 reg_ldec_clk_gate_en:1;
            U16 reg_lenc_clk_gate_en:1;
            U16 reg_mzc_reset_fsm_idle:1;
            U16 reg_mzc_fullspeed_mode:1;
            U16 reg_mzc_fullspeed_sw_en:1;
            U16 reg_mzc_v15_mode_en:1

        };
        U16 reg02;
    };

    union {
        struct {
            U16 reg_mzc_ext_addr:4;
        };
        U16 reg03;
    };

    union {
        struct {
            U16 reg_ldec_inq_near_empty_th:4;
            U16	reg_ldec_inq_near_full_th:4;
            U16	reg_ldec_outq_near_empty_th:4;
            U16	reg_ldec_outq_near_full_th:4;
        };
        U16 reg04;
    };

	union {
        struct {
            U16 reg_lenc_inq_near_empty_th:4;
            U16	reg_lenc_inq_near_full_th:4;
            U16	reg_lenc_outq_near_empty_th:4;
            U16	reg_lenc_outq_near_full_th:4;
        };
        U16 reg05;
    };

	union {
        struct {
            U16 reg_ro_ldec_inq_lvl:5;
            U16	reg_reg06_reserved:3; //johnny question
            U16	reg_ro_ldec_outq_lvl:5;
        };
        U16 reg06;
    };

	union {
        struct {
            U16 reg_ro_lenc_inq_lvl:5;
            U16	reg_reg07_reserved:3;//johnny question
            U16	reg_ro_lenc_outq_lvl:5;
        };
        U16 reg07;
    };

	union {
        struct {
            U16 reg_ldec_inq_in_st_adr_lsb_lo:16;
        };
        U16 reg08;
    };

	union {
        struct {
            U16 reg_ldec_inq_in_st_adr_lsb_hi:16;
        };
        U16 reg09;
    };

	union {
        struct {
            U16 reg_ldec_inq_id:4;
            U16 reg_ldec_inq_in_st_adr_msb:4;
            U16 reg_ldec_inq_out_st_adr_lo:8;
        };
        U16 reg0a;
    };
	union {
        struct {
            U16 reg_ldec_inq_out_st_adr_hi:16;
        };
        U16 reg0b;
    };

 	union {
        struct {
            U16 reg_ldec_outq_crc_lo:16;
        };
        U16 reg0c;
    };

 	union {
        struct {
            U16 reg_ldec_outq_crc_hi:16;
        };
        U16 reg0d;
    };	

    union {
        struct {
            U16 reg_ldec_outq_info:10;
        };
        U16 reg0e;
    };

    union {
        struct {
            U16 reg_lenc_inq_id:4;
            U16 reg_lenc_inq_out_st_adr_msb:1;
            U16 reg_reg10_reserved:3;	// johnny question
            U16 reg_lenc_inq_in_st_adr_lo:8;
        };
        U16 reg10;
    };

    union {
        struct {
             U16 reg_lenc_inq_in_st_adr_hi:16;
        };
        U16 reg11;
    };
    union {
        struct {
             U16 reg_lenc_inq_out_st_adr_lsb_lo:16;
        };
        U16 reg12;
    };
    union {
        struct {
             U16 reg_lenc_inq_out_st_adr_lsb_hi:16;
        };
        U16 reg13;
    };
    union {
        struct {
             U16 reg_lenc_outq_crc_lo:16;
        };
        U16 reg14;
    };

    union {
        struct {
             U16 reg_lenc_outq_crc_hi:16;
        };
        U16 reg15;
    };   
    
    union {
        struct {
             U16 reg_lenc_outq_info_lo:16;
        };
        U16 reg16;
    };
    union {
        struct {
             U16 reg_lenc_outq_info_hi:4;
        };
        U16 reg17;
    };
    union {
        struct {
             U16 reg_ldec_outq_num_th:4;
             U16 reg_lenc_outq_num_th:4;
        };
        U16 reg18;
    };
    union {
        struct {
             U16 reg_ldec_outq_timer_th_lo:16;
        };
        U16 reg19;
    };
    union {
        struct {
             U16 reg_ldec_outq_timer_th_hi:16;
        };
        U16 reg1a;
    };
    union {
        struct {
             U16 reg_lenc_outq_timer_th_lo:16;
        };
        U16 reg1b;
    };
    union {
        struct {
             U16 reg_lenc_outq_timer_th_hi:16;
        };
        U16 reg1c;
    };
    union {
        struct {
             U16 reg_ldec_op_mode:1;
             U16 reg_reg1d_reserved:6;  //johnny quesetion
             U16 reg_w1c_ldec_start:1;
             U16 reg_ldec_bdma_burst_sel:2;
             U16 reg_ldec_odma_burst_sel:2;
             U16 reg_ldec_crc_en:1;
             U16 reg_ldec_crc_mode:1;
             U16 reg_ldec_crc_round_en:1;
             U16 reg_ldec_crc_per_byte:1;
        };
        U16 reg1d;
    };
    union {
        struct {
            U16 reg_ldec_in_st_adr_lsb_lo:16;
        };
        U16 reg1e;
    };
    union {
        struct {
            U16 reg_ldec_in_st_adr_lsb_hi:16;
        };
        U16 reg1f;
    };
    union {
        struct {
            U16 reg_ldec_out_st_adr_lo:16;
        };
        U16 reg20;
    };
    union {
        struct {
            U16 reg_ldec_out_st_adr_hi:8;
            U16	reg_ldec_in_st_adr_msb:4;
        };
        U16 reg21;
    };
    union {
        struct {
            U16 reg_ldec_out_size_thr:13;
        };
        U16 reg22;
    };
    union {
        struct {
            U16 reg_ldec_timeout_thr_lo:16;
        };
        U16 reg23;
    };
    union {
        struct {
            U16 reg_ldec_timeout_thr_hi:4;
            U16	reg_ldec_timeout_en:1;
            U16	reg_ldec_cnt_sel:1;
            U16 reg_ldec_cnt_sel_1:3;
        };
        U16 reg24;
    };
    union {
        struct {
            U16 reg_ldec_crc_round_cnt:12;
        };
        U16 reg25;
    };    
    union {
        struct {
            U16 reg_ro_ldec_debug_cyc_lo:16;
        };
        U16 reg26;
    };
     union {
        struct {
            U16 reg_ro_ldec_debug_cyc_hi:16;
        };
        U16 reg27;
    };
    union {
        struct {
            U16 reg_ro_ldec_crc_out_lo:16;
        };
        U16 reg28;
    };
    union {
        struct {
            U16 reg_ro_ldec_crc_out_hi:16;
        };
        U16 reg29;
    };
    union {
        struct {
            U16 reg_ldec_inq_in_st_adr2_4k_lsb:16;
        };
        U16 reg2a;
    };
    union {
        struct {
            U16 reg_ldec_inq_in_st_adr2_4k_msb:8;
            U16 reg_ldec_inq_in_st_adr2_en:1
        };
        U16 reg2b;
    };
    union {
        struct {
            U16 reg_ro_ldec_single_cyc_hi:4;
        };
        U16 reg2c;
    };
    union {
        struct {
            U16 reg_ro_ldec_inq_cur_ptr:4;
            U16 reg_ldec_inq_read_ptr:4;
            U16 reg_ldec_inq_read_sel:2;
        };
        U16 reg2d;
    };
    union {
        struct {
            U16 reg_ro_ldec_inq_debug_lo:16;
        };
        U16 reg2e;
    };  
    union {
        struct {
            U16 reg_ro_ldec_inq_debug_hi:16;
        };
        U16 reg2f;
    };   
    union {
        struct {
            U16 reg_mzc_irq_mask_lo:16;
        };
        U16 reg30;
    };
    union {
        struct {
            U16 reg_mzc_irq_mask_hi:16;
        };
        U16 reg31;
    };
    union {
        struct {
            U16 reg_mzc_irq_force_lo:16;
        };
        U16 reg32;
    };
    union {
        struct {
            U16 reg_mzc_irq_force_hi:16;
        };
        U16 reg33;
    };
    union {
        struct {
            U16 reg_mzc_irq_clr0:1;
            U16 reg_mzc_irq_clr1:1;
            U16 reg_mzc_irq_clr2:1;
            U16 reg_mzc_irq_clr3:1;
            U16 reg_mzc_irq_clr4:1;
            U16 reg_mzc_irq_clr5:1;
            U16 reg_mzc_irq_clr6:1;
            U16 reg_mzc_irq_clr7:1;
            U16 reg_mzc_irq_clr8:1;
            U16 reg_mzc_irq_clr9:1;
            U16 reg_mzc_irq_clr10:1;
            U16 reg_mzc_irq_clr11:1;
            U16 reg_mzc_irq_clr12:1;
            U16 reg_mzc_irq_clr13:1;
            U16 reg_mzc_irq_clr14:1;
            U16 reg_mzc_irq_clr15:1;
        };
        U16 reg34;
    };
    union {
        struct {
            U16 reg_mzc_irq_clr16:1;
            U16 reg_mzc_irq_clr17:1;
            U16 reg_mzc_irq_clr18:1;
            U16 reg_mzc_irq_clr19:1;
            U16 reg_mzc_irq_clr20:1;
            U16 reg_mzc_irq_clr21:1;
            U16 reg_mzc_irq_clr22:1;
            U16 reg_mzc_irq_clr23:1;
            U16 reg_mzc_irq_clr24:1;
            U16 reg_mzc_irq_clr25:1;
            U16 reg_mzc_irq_clr26:1;
            U16 reg_mzc_irq_clr27:1;
            U16 reg_mzc_irq_clr28:1;
            U16 reg_mzc_irq_clr29:1;
            U16 reg_mzc_irq_clr30:1;
            U16 reg_mzc_irq_clr31:1;
        };
        U16 reg35;
    };
    union {
        struct {
            U16 reg_mzc_st_irq_cpu_lo:16;
        };
        U16 reg36;
    };
    union {
        struct {
            U16 reg_mzc_st_irq_cpu_hi:16;
        };
        U16 reg37;
    };
    union {
        struct {
            U16 reg_mzc_st_irq_ip_lo:16;
        };
        U16 reg38;
    };
    union {
        struct {
            U16 reg_mzc_st_irq_ip_hi:16;
        };
        U16 reg39;
    };
    union {
        struct {
            U16 reg_ldec_out_lower_bound_adr_lo:16;
        };
        U16 reg3a;
    };
    union {
        struct {
            U16 reg_ldec_out_lower_bound_adr_hi:16;
        };
        U16 reg3b;
    };
    union {
        struct {
            U16 reg_ldec_out_upper_bound_adr_lo:16;
        };
        U16 reg3c;
    };
    union {
        struct {
            U16 reg_ldec_out_upper_bound_adr_hi:16;
        };
        U16 reg3d;
    };
    union {
        struct {
            U16 reg_lenc_op_mode:1;
            U16	reg_lenc_auto_rstz_cnt:6;
            U16	reg_w1c_lenc_start:1;
            U16	reg_lenc_rd_burst_sel:2;
            U16	reg_lenc_wr_burst_sel:2;
            U16	reg_lenc_crc_en:1;
            U16	reg_lenc_crc_mode:1;
            U16	reg_lenc_crc_round_en:1;
        };
        U16 reg40;
    };
    union {
        struct {
            U16 reg_lenc_in_st_adr_lo:16;
        };
        U16 reg41;
    };
    union {
        struct {
            U16 reg_lenc_in_st_adr_hi:8;
            U16	reg_lenc_out_st_adr_msb:1;
        };
        U16 reg42;
    };
    union {
        struct {
            U16 reg_lenc_out_st_adr_lsb_lo:16;
        };
        U16 reg43;
    };
    union {
        struct {
            U16 reg_lenc_out_st_adr_lsb_hi:16;
        };
        U16 reg44;
    };
    union {
        struct {
            U16 reg_reg45_reserved:4; //johnny question
            U16 reg_lenc_out_size_thr:9;
        };
        U16 reg45;
    };
    union {
        struct {
            U16 reg_lenc_timeout_thr_lo:16;
        };
        U16 reg46;
    };
    union {
        struct {
            U16 reg_lenc_timeout_thr_hi:4;
            U16	reg_lenc_timeout_en:1;
            U16	reg_lenc_cnt_sel:1;
            U16 reg_lenc_cnt_sel_1:3;
        };
        U16 reg47;
    };
    union {
        struct {
            U16 reg_ro_lenc_crc_round_cnt:12;
            U16 reg_lenc_dic_cache_disable:1;
            U16 reg_lenc_chain_cache_disable:1;
            U16 reg_lenc_only_park_inout_stage:1;
        };
        U16 reg48;
    };
    union {
        struct {
            U16 reg_ro_lenc_active_cyc_hi:16;
        };
        U16 reg49;
    };
    union {
        struct {
            U16 reg_ro_lenc_debug_cyc_lo:16;
        };
        U16 reg4a;
    };
    union {
        struct {
            U16 reg_ro_lenc_debug_cyc_hi:16;
        };
        U16 reg4b;
    };
    union {
        struct {
            U16 reg_ro_lenc_crc_out_lo:16;
        };
        U16 reg4c;
    };
    union {
        struct {
            U16 reg_ro_lenc_crc_out_hi:16;
        };
        U16 reg4d;
    };
    union {
        struct {
            U16 reg_ro_lenc_single_cyc_hi:4;
        };
        U16 reg4e;
    };
    union {
        struct {
            U16 reg_ro_lenc_crc_out:16;
        };
        U16 reg4f;
    };
    union {
        struct {
            U16 reg_sync_lenc_hash_early_cnt:8;
            U16 reg_sync_lenc_hash_search_num:3;
            U16 reg_sync_lenc_reps_full_search:1;
        	U16 reg_sync_lenc_lit_bypass_bins:2;
        };
        U16 reg50;
    };
    union {
        struct {
            U16 reg_lenc_hash_prime_val:16;
        };
        U16 reg51;
    };
    union {
        struct {
            U16 reg_lenc_lower_addr_lo:16;
        };
        U16 reg52;
    };
    union {
        struct {
            U16 reg_lenc_lower_addr_hi:16;
        };
        U16 reg53;
    };
    union {
        struct {
            U16 reg_lenc_upper_addr_lo:16;
        };
        U16 reg54;
    };
    union {
        struct {
            U16 reg_lenc_upper_addr_hi:16;
        };
        U16 reg55;
    };
    union {
        struct {
            U16 reg_ro_lenc_byte_cnt:13;
        };
        U16 reg56;
    };
    union {
        struct {
            U16 reg_ro_lenc_inq_cur_ptr:4;
            U16 reg_lenc_inq_read_ptr:4;
            U16 reg_lenc_inq_read_sel:1;
        };
        U16 reg57;
    };    
    union {
        struct {
            U16 reg_ro_lenc_inq_debug_lo:16;
        };
        U16 reg58;
    };  
    union {
        struct {
			 U16 reg_ro_lenc_inq_debug_hi:16;
        };
        U16 reg59;
    };  
    union {
        struct {
            U16 reg_acp_outstand:1;
        };
        U16 reg60;
    };
	union {
        struct {
            U16 reg_arcache:4;
            U16	reg_arprot:3;
            U16	reg_aruser:2;
        };
        U16 reg61;
    };
	union {
        struct {
            U16 reg_awcache:4;
            U16	reg_awprot:3;
            U16	reg_awuser:2;
        };
        U16 reg62;
    };
	union {
        struct {
            U16 reg_acp_eff_utilize_bl_en:1;
        };
        U16 reg63;
    };
	union {
        struct {
            U16 reg_acp_arc_utilization_lo:16;
        };
        U16 reg64;
    };
	union {
        struct {
            U16 reg_acp_arc_utilization_hi:16;
        };
        U16 reg65;
    };
	union {
        struct {
            U16 reg_acp_arc_efficiency_lo:16;
        };
        U16 reg66;
    };
	union {
        struct {
            U16 reg_acp_arc_efficiency_hi:16;
        };
        U16 reg67;
    };
	union {
        struct {
            U16 reg_acp_rc_utilization_lo:16;
        };
        U16 reg68;
    };
	union {
        struct {
            U16 reg_acp_rc_utilization_hi:16;
        };
        U16 reg69;
    };
	union {
        struct {
            U16 reg_acp_rc_efficiency_lo:16;
        };
        U16 reg6a;
    };
	union {
        struct {
            U16 reg_acp_rc_efficiency_hi:16;
        };
        U16 reg6b;
    };
	union {
        struct {
            U16 reg_acp_awc_utilization_lo:16;
        };
        U16 reg6c;
    };
	union {
        struct {
            U16 reg_acp_awc_utilization_hi:16;
        };
        U16 reg6d;
    };
	union {
        struct {
            U16 reg_acp_awc_efficiency_lo:16;
        };
        U16 reg6e;
    };
	union {
        struct {
            U16 reg_acp_awc_efficiency_hi:16;
        };
        U16 reg6f;
    };
	union {
        struct {
            U16 reg_acp_wc_utilization_lo:16;
        };
        U16 reg70;
    };
	union {
        struct {
            U16 reg_acp_wc_utilization_hi:16;
        };
        U16 reg71;
    };
    union {
        struct {
            U16 reg_acp_wc_efficiency_lo:16;
        };
        U16 reg72;
    };
    union {
        struct {
            U16 reg_acp_wc_efficiency_hi:16;
        };
        U16 reg73;
    };
    union {
        struct {
            U16 reg_mzc_dbg_sel:8;
        };
        U16 reg78;
    };
    union {
        struct {
            U16 reg_mzc_dbg_bus_out_lo:16;
        };
        U16 reg79;
    };
    union {
        struct {
            U16 reg_mzc_dbg_bus_out_hi:16;
        };
        U16 reg7a;
    };
    union {
        struct {
            U16 reg_mzc_bist_fail:16;
        };
        U16 reg7b;
    };
    union {
        struct {
            U16 reg_mzc_rsv7c_reserved1:2;
            U16 reg_mzc_rsv7c_reserved2:1;
            U16 reg_mzc_rsv7c_reserved3:13;
        };
        U16 reg7c;
    };
    union {
        struct {
            U16 reg_mzc_rsv7d:16;
        };
        U16 reg7d;
    };
} LZC_REGS;

//above is register operation



int mhal_ldec_cmdq_set_split(unsigned long in_addr1, unsigned long in_addr2, unsigned long out_addr, unsigned int process_id);



void mhal_ldec_literal_bypass_set(int value);

#endif
