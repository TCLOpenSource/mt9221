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

#ifndef MZC_HAL_H
#define MZC_HAL_H

#include <linux/wait.h>
#include <linux/irqreturn.h>
#include <linux/zsmalloc.h>
#include <linux/workqueue.h>
#include <asm/io.h>
typedef unsigned short  U16;

#define _BIT(x)                      (1<<(x))
/*bank and irq */

#define regoffsetnum 0x80

#define SINGLE_MODE 0

#define CMDQ_MODE 1

#define CMDQ_LENC_NORMAL_END 1

#define CMDQ_LDEC_NORMAL_END 1

#define CMDQ_LENC_TIME_OUT 4

#define CMDQ_LDEC_TIME_OUT 16

#define RESET 0

#define NOT_RESET 1

#define MZC_VERSION_OFFSET 0x1

#define MZC_VERSION_BANK 0x1e

#define MZC_FREQ_OFFSET01 0x42

#define MZC_FREQ_OFFSET02 0x43

#define LZC_BANK 0x1810

#define ACP_BANK 0x101d

#define FREQ_BANK 0x1033

#define MZC_CLK_BANK 0x1001

#define ACACHE_BANK 0x100e

#define L4_AXI_BANK 0x1018

#define MZC_FREQ_BANK 0x110c

#define MZC_VERSION_MASK 0xff00

#define E_IRQ_LZC 216

#define NONPM_BASE 0x1f000000

#define NON_SECURE_ACCESS 0x3

#define SECURE_ACCESS 0x0

#define MASK_IRQ_ALL 0xffff
#define LDEC_CMDQ_IRQ_IN_NEAR_EMPTY     _BIT(0)
#define LDEC_CMDQ_IRQ_IN_NEAR_FULL      _BIT(1)
#define LDEC_CMDQ_IRQ_IN_OVERFLOW       _BIT(2)
#define LDEC_CMDQ_IRQ_OUT_NEAR_EMPTY    _BIT(3)
#define LDEC_CMDQ_IRQ_OUT_NEAR_FULL     _BIT(4)
#define LDEC_CMDQ_IRQ_OUT_UNDERFLOW     _BIT(5)
#define LENC_CMDQ_IRQ_IN_NEAR_EMPTY     _BIT(6)
#define LENC_CMDQ_IRQ_IN_NEAR_FULL      _BIT(7)
#define LENC_CMDQ_IRQ_IN_OVERFLOW       _BIT(8)
#define LENC_CMDQ_IRQ_OUT_NEAR_EMPTY    _BIT(9)
#define LENC_CMDQ_IRQ_OUT_NEAR_FULL     _BIT(10)
#define LENC_CMDQ_IRQ_OUT_UNDERFLOW     _BIT(11)
#define LDEC_CMDQ_IRQ_TIMER   			_BIT(12)
#define LDEC_CMDQ_IRQ_NUMBER_THRESHOLD  _BIT(13)
#define LENC_CMDQ_IRQ_TIMER   			_BIT(14)
#define LENC_CMDQ_IRQ_NUMBER_THRESHOLD  _BIT(15)
#define LDEC_IRQ_NORMAL_END             _BIT(16-16)
#define LDEC_IRQ_CABAC_ERROR            _BIT(17-16)
#define LDEC_IRQ_SIZE_ERROR             _BIT(18-16)
#define LDEC_IRQ_DICT_ERROR             _BIT(19-16)
#define LDEC_IRQ_TIMEOUT                _BIT(20-16)
#define LDEC_IRQ_PKT_MARKER_ERROR       _BIT(21-16)
#define LENC_IRQ_NORMAL_END             _BIT(22-16)
#define LENC_IRQ_SIZE_ERROR             _BIT(23-16)
#define LENC_IRQ_TIMEOUT                _BIT(24-16)
#define ACP_RRESP_ERROR					_BIT(25-16)
#define ACP_BRESP_ERROR					_BIT(26-16)
#define LDEC_WRITE_INVALID_ERROR		_BIT(27-16)
#define LENC_WRITE_INVALID_ERROR		_BIT(28-16)






#define MASK_IRQ_ALL_EXCEPT_TIMER_AND_NUMBER (MASK_IRQ_ALL &  ~(LDEC_CMDQ_IRQ_TIMER | LDEC_CMDQ_IRQ_NUMBER_THRESHOLD | LENC_CMDQ_IRQ_TIMER | LENC_CMDQ_IRQ_NUMBER_THRESHOLD))

#define MASK_IRQ_ALL_EXCEPT_HW_TIMEOUT (MASK_IRQ_ALL & ~(LENC_IRQ_TIMEOUT | LDEC_IRQ_TIMEOUT))

#define SINGLE_MODE_LENC_IRQ        (LENC_IRQ_NORMAL_END | LENC_IRQ_SIZE_ERROR | LENC_IRQ_TIMEOUT)

#define SINGLE_MODE_LDEC_IRQ        (LDEC_IRQ_NORMAL_END | LDEC_IRQ_CABAC_ERROR | LDEC_IRQ_SIZE_ERROR | LDEC_IRQ_DICT_ERROR | LDEC_IRQ_TIMEOUT | LDEC_IRQ_PKT_MARKER_ERROR)

#define MASK_SINGLE_MODE_IRQ        (~(SINGLE_MODE_LENC_IRQ | SINGLE_MODE_LDEC_IRQ))

#define CLEAR_IRQ(irq)              (irq)

#define LENC_RELATED (LENC_CMDQ_IRQ_TIMER | LENC_CMDQ_IRQ_NUMBER_THRESHOLD)

#define LDEC_RELATED (LDEC_CMDQ_IRQ_TIMER | LDEC_CMDQ_IRQ_NUMBER_THRESHOLD)

#define LENC_CMDQ_TIMER_THR_LO 0xffff

#define LENC_CMDQ_TIMER_THR_HI 0xffff

#define LDEC_CMDQ_TIMER_THR_LO 0xffff

#define LDEC_CMDQ_TIMER_THR_HI 0xffff

#define LENC_TIMEOUT_THRESHOLD (0xfffff)

#define LDEC_TIMEOUT_THRESHOLD (0xfffff)

#define LENC_CRC_DEFAULT_MODE 1

#define LENC_CRC_DEFAULT_ENABLE_STATE 1

#define LENC_OUTPUT_THRESHOLD 0

#define LDEC_CRC_DEFAULT_MODE 0 // 0: input, 1: output

#define LDEC_CRC_DEFAULT_ENABLE_STATE 0

#define LDEC_OUTPUT_THRESHOLD 0

#define CMDQ_LENC_SIZE_ERROR 0x2

#define CMDQ_LDEC_ERROR_TYPE_BIT_POS ((LDEC_IRQ_CABAC_ERROR) | (LDEC_IRQ_SIZE_ERROR) | (LDEC_IRQ_DICT_ERROR) | (LDEC_IRQ_TIMEOUT) | (LDEC_IRQ_PKT_MARKER_ERROR))

#define CLEAR_LENC_IRQ (LENC_RELATED)

#define CLEAR_LDEC_IRQ (LDEC_RELATED)

#define LENC_TIMEOUT (LENC_IRQ_TIMEOUT)

#define LDEC_TIMEOUT (LDEC_IRQ_TIMEOUT)

#define CLEAR_LENC_IRQ_TIMEOUT (LENC_TIMEOUT)

#define CLEAR_LDEC_IRQ_TIMEOUT (LDEC_TIMEOUT)

#define CMDQ_TOP_SETTING 0

#define CMDQ_TOP_SETTING_002 0x2

#define MZC_VERSION_SETTING 0xd

#define RO_LDEC_OUT_INQ_LV1 0x06

#define RO_LENC_OUT_INQ_LV1 0x07

#define LDEC_INQ_IN_ST_ADR_LSB_HI 0x09

#define LDEC_INQ_IN_ST_ADR_LSB_LO 0x08

#define LDEC_INQ_DATA 0x0a

#define LDEC_INQ_OUT_ST_ADR 0x0b

#define LDEC_OUTQ_INFO 0x0e

#define LENC_INQ_DATA 0x10

#define LENC_INQ_IN_ST_ADR 0x11

#define LENC_INQ_OUT_ST_ADR_LSB_LO 0x12

#define LENC_INQ_OUT_ST_ADR_LSB_HI 0x13

#define LENC_OUTQ_INFO_LO 0x16

#define LENC_OUTQ_INFO_HI 0x17

#define LZC_OUTQ_NUM_TH 0x18

#define LDEC_OUTQ_TIMER_TH_LO 0x19

#define LDEC_OUTQ_TIMER_TH_HI 0x1a

#define LENC_OUTQ_TIMER_TH_LO 0x1b

#define LENC_OUTQ_TIMER_TH_HI 0x1c

#define LDEC_GENERAL_SETTING 0x1d

#define LDEC_IN_ST_ADR_LSB_LO 0x1e

#define LDEC_IN_ST_ADR_LSB_HI 0x1f

#define LDEC_OUT_ST_ADR_LO 0x20

#define LDEC_OUT_ST_ADR_HI 0x21

#define LDEC_TIMEOUT_THR 0x23

#define LDEC_TIMEOUT_EN 0x24

#define RO_LDEC_ACTIVE_CYC_LO 0x26

#define RO_LDEC_ACTIVE_CYC_HI 0x27

#define RO_LDEC_FREE_CYC_LO 0x28

#define RO_LDEC_FREE_CYC_HI 0x29

#define RO_LDEC_PAGE_CNT 0x2a

#define LDEC_CRC_OUT 0x2d

#define MZC_IRQ_MASK_HI 0x31

#define MZC_IRQ_MASK_LO 0x30

#define LZC_IRQ_STATUS_LO 0x34

#define LZC_IRQ_STATUS_HI 0x35

#define MZC_ST_IRQ_CPU_LO 0x36

#define MZC_ST_IRQ_CPU_HI 0x37

#define MZC_ST_IRQ_IP_LO 0x38

#define MZC_ST_IRQ_IP_HI 0x39

#define LENC_GENERAL_SETTING 0x40

#define LENC_IN_ST_ADR 0x41

#define LENC_OUT_ST_ADR_MSB 0x42

#define LENC_OUT_ST_ADR_LSB_HI 0x44

#define LENC_OUT_ST_ADR_LSB_LO 0x43

#define LENC_OUTSIZE_THR 0x45

#define LENC_TIMEOUT_THR 0x46

#define LENC_TIMEOUT_EN 0x47

#define RO_LENC_ACTIVE_CYC_LO 0x48

#define RO_LENC_ACTIVE_CYC_HI 0x49

#define RO_LENC_FREE_CYC_LO 0x4a

#define RO_LENC_FREE_CYC_HI 0x4b

#define RO_LENC_PAGE_CNT 0x4c

#define LENC_CRC_OUT 0x4f

#define LENC_PAGE_SYNC 0x50

#define RO_LENC_BYTE_CNT 0x56

#define ACP_OUTSTAND 0x60

#define ACP_AR 0x61

#define ACP_AW 0x62

#define MZC_RSV7C  0x7c

#define MZC_RSV7D  0x7d

#define ACP_IDLE_SW_FORCE 0xdfff

#define MZC_FREQ_REGISTER 0x19

#define MZC_CLCK_SETTING 0x1e

#define MZC_FULL_SPEED 0x0

#define MZC_CLCK_FIRE 0x80

#define DFS_ENABLE_ADDR 0x6a

#define DFS_FREQ_DIVISION_ADDR 0x69

#define DFS_UPDATE_ADDR 0x6b

#define MZC_RESERVED_7C 0x7c

#ifndef GET_LOWORD
#define GET_LOWORD(value)    ((unsigned short)(((unsigned int)(value)) & 0xffff))
#endif
#ifndef GET_HIWORD
#define GET_HIWORD(value)    ((unsigned short)((((unsigned int)(value)) >> 16) & 0xffff))
#endif

//above is register operation

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
            U16 reg_ldec_outq_crc:16;
        };
        U16 reg0c;
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
             U16 reg_lenc_outq_crc:16;
        };
        U16 reg14;
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
        };
        U16 reg24;
    };
    union {
        struct {
            U16 reg_ro_ldec_active_cyc_lo:16;
        };
        U16 reg26;
    };
     union {
        struct {
            U16 reg_ro_ldec_active_cyc_hi:16;
        };
        U16 reg27;
    };
    union {
        struct {
            U16 reg_ro_ldec_free_cyc_lo:16;
        };
        U16 reg28;
    };
    union {
        struct {
            U16 reg_ro_ldec_free_cyc_hi:16;
        };
        U16 reg29;
    };
    union {
        struct {
            U16 reg_ro_ldec_page_cnt:16;
        };
        U16 reg2a;
    };
    union {
        struct {
            U16 reg_ro_ldec_single_cyc_lo:16;
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
            U16 reg_ro_ldec_crc_out:16;
        };
        U16 reg2d;
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
        };
        U16 reg47;
    };
    union {
        struct {
            U16 reg_ro_lenc_active_cyc_lo:16;
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
            U16 reg_ro_lenc_free_cyc_lo:16;
        };
        U16 reg4a;
    };
    union {
        struct {
            U16 reg_ro_lenc_free_cyc_hi:16;
        };
        U16 reg4b;
    };
    union {
        struct {
            U16 reg_ro_lenc_page_cnt:16;
        };
        U16 reg4c;
    };
    union {
        struct {
            U16 reg_ro_lenc_single_cyc_lo:16;
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

void mhal_lzc_cmdq_init(int enc_out_size_thr);

void mhal_lzc_cmdq_exit(void);

int mhal_lenc_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id);

int mhal_ldec_cmdq_set(unsigned long in_addr, unsigned long out_addr, unsigned int process_id);

void mhal_lzc_single_init(int enc_out_size_thr);

void mhal_lzc_single_exit(void);

void mhal_lzc_irq_mask_all_except_timeout(void);

void mhal_lzc_irq_mask_all(void);

int mhal_lenc_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id);

int mhal_ldec_single_set(unsigned long in_addr,unsigned long out_addr, unsigned int process_id);

void mhal_ldec_cycle_info(int *active_cycle, int *free_cycle, int *page_count);

void mhal_lenc_cycle_info(int *active_cycle, int *free_cycle, int *page_count);

int mhal_lenc_cmdq_done(int *target_id);

int mhal_ldec_cmdq_done(int *target_id);

int mhal_lenc_single_done(void);

int mhal_ldec_single_done(void);

unsigned int mhal_lzc_cmdq_handler(int *w_outputcount, int *r_outputcount, int *is_write, int *is_read);

void mhal_lzc_cmdq_clear_irq(unsigned int irq_status);

void mhal_lenc_cmdq_handler(int *outlen,int *process_id);

void mhal_ldec_cmdq_handler(int *outlen, int *process_id);

void mhal_lzc_single_handler(int *w_outlen, int *r_outlen, int *is_write, int *is_read);

void mhal_regptr_set(void);

void mhal_regptr_unset(void);

void mhal_acp_common_init(void);

void mhal_ldec_state_save(int wait);

void mhal_lenc_state_save(int wait);

int mhal_lenc_state_check(void);

int mhal_ldec_state_check(void);

unsigned int mhal_lenc_crc_get(void);

#endif
