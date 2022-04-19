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

#ifndef _MHAL_CLKM_REG_H
#define _MHAL_CLKM_REG_H



#define CLKGEN0 0x100B
#define CLKGEN2 0x100A


#define CLK_GATE_REV 1
#define CLK_GATE_NORMAL 0
#define	NO_GATE_REG	2
#define	HAS_GATE_REG	1
#define NO_SEL_REG  0
#define HAS_SEL_REG 1

#define g_clk_ge_num 0x4

char *g_clk_ge_ptr[] =
{
"clk_miu_256bus_p",
"clk_240_buf",
"clk_216_buf",
"clk_172_buf",
};


#define g_clk_gpd_num 0x8

char *g_clk_gpd_ptr[] =
{
"clk_216_buf",
"clk_192_buf",
"clk_172_buf",
"clk_144_buf",
"clk_108_buf",
"REV",
"REV",
"REV",
};


#define g_clk_njpd_num 0x4

char *g_clk_njpd_ptr[] =
{
"clk_288_buf",
"clk_320_buf",
"clk_192_buf",
"clk_144_buf",
};


#define g_clk_mfe_num 0x4

char *g_clk_mfe_ptr[] =
{
"clk_240_buf",
"clk_123_buf",
"clk_192_buf",
"clk_288_buf",
};


#define g_clk_mvd_num 0x8

char *g_clk_mvd_ptr[] =
{
"clk_216_buf",
"clk_192_buf",
"clk_172_buf",
"clk_144_buf",
"0",
"clk_123_buf",
"0",
"clk_xtal_12M_buf",
};


#define g_clk_vd_mheg5_num 0x8

char *g_clk_vd_mheg5_ptr[] =
{
"clk_240_buf",
"clk_216_buf",
"clk_192_buf",
"clk_xtal_12M_buf",
"clk_320_buf",
"Clk_288_buf",
"clk_432_buf",
"clk_384_buf",
};


#define g_clk_hvd_num 0x8

char *g_clk_hvd_ptr[] =
{
"clk_384_buf",
"clk_345_buf",
"clk_320_buf",
"clk_288_buf",
"clk_240_buf",
"clk_216_buf",
"clk_172_buf",
"clk_432_buf",
};


#define g_clk_hvd_aec_num 0x4

char *g_clk_hvd_aec_ptr[] =
{
"clk_216_buf",
"clk_240_buf",
"clk_288_buf",
"clk_172_buf",
};


#define g_clk_vp8_num 0x4

char *g_clk_vp8_ptr[] =
{
"clk_288_buf",
"clk_240_buf",
"clk_216_buf",
"clk_320_buf",
};


#define g_clk_hvd_idb_num 0x8

char *g_clk_hvd_idb_ptr[] =
{
"clk_432_buf",
"clk_384_buf",
"clk_345_buf",
"clk_480_buf",
"clk_320_buf",
"clk_288_buf",
"clk_240_buf",
"clk_216_buf",
};


#define g_clk_hvd_aec_new_num 0x4

char *g_clk_hvd_aec_new_ptr[] =
{
"clk_288_buf",
"clk_240_buf",
"clk_216_buf",
"clk_320_buf",
};


#define g_clk_evd_num 0x8

char *g_clk_evd_ptr[] =
{
"clk_evdpll_buf",
"clk_miu128pll_buf",
"clk_miu256pll_buf",
"clk_480_buf",
"clk_384_buf",
"clk_320_buf",
"clk_240_buf",
"clk_192_buf",
};


#define g_clk_evdpu_num 0x8

char *g_clk_evdpu_ptr[] =
{
"clk_evdpll_buf",
"clk_miu128pll_buf",
"clk_miu256pll_buf",
"clk_480_buf",
"clk_384_buf",
"clk_320_buf",
"clk_240_buf",
"clk_192_buf",
};


#define g_clk_aesdma_num 0x4

char *g_clk_aesdma_ptr[] =
{
"clk_172_buf",
"clk_288_buf",
"clk_216_buf",
"clk_xtal_12M_buf",
};


#define g_clk_tsp_num 0x4

char *g_clk_tsp_ptr[] =
{
"clk_172_buf",
"clk_144_buf",
"clk_108_buf",
"clk_xtal_12M_buf",
};


#define g_clk_tso_trace_num 0x4

char *g_clk_tso_trace_ptr[] =
{
"clk_216_buf",
"REV",
"REV",
"REV",
};


#define g_clk_tso_in_num 0x6

char *g_clk_tso_in_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
};


#define g_clk_tso_out_num 0x8

char *g_clk_tso_out_ptr[] =
{
"clk_tso_out_div",
"clk_62_buf",
"clk_54_buf",
"clk_ts_src_sel_p",
"clk_ts_src_sel_div8",
"clk_27_buf",
"0",
"clk_demod_ts_internal_p",
};


#define g_clk_tso1_in_num 0x7

char *g_clk_tso1_in_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"REV",
};


#define g_clk_tso2_in_num 0x7

char *g_clk_tso2_in_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"REV",
};


#define g_clk_ts4_num 0x8

char *g_clk_ts4_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"REV",
"REV",
};


#define g_clk_ts5_num 0x8

char *g_clk_ts5_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"REV",
"REV",
};


#define g_clk_ts_mmt_num 0x8

char *g_clk_ts_mmt_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"REV",
"REV",
};


#define g_clk_ts2_num 0x8

char *g_clk_ts2_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"clk_tso_out_p",
"clk_demod_ts_internal_p",
};


#define g_clk_ts1_num 0x8

char *g_clk_ts1_ptr[] =
{
"clk_ts0_buf",
"clk_ts1_buf",
"clk_ts2_buf",
"clk_ts3_buf",
"clk_ts4_buf",
"clk_ts5_buf",
"clk_tso_out_p",
"clk_demod_ts_internal_p",
};


#define g_clk_parser_num 0x2

char *g_clk_parser_ptr[] =
{
"clk_172_buf",
"clk_192_buf",
};


#define g_clk_vd1_num 0x4

char *g_clk_vd1_ptr[] =
{
"clk_vd_p",
"REV",
"VD_TestMode_clk_buf",
"REV",
};


#define g_clk_vd_32fsc_num 0x4

char *g_clk_vd_32fsc_ptr[] =
{
"clk_vd_adc_buf",
"REV",
"REV",
"REV",
};


#define g_clk_vdmcu_num 0x8

char *g_clk_vdmcu_ptr[] =
{
"clk_172_buf",
"clk_160_buf",
"clk_144_buf",
"clk_123_buf",
"clk_108_buf",
"",
"",
"clk_xtal_12M_buf",
};


#define g_clk_r2_secure_num 0x4

char *g_clk_r2_secure_ptr[] =
{
"clk_288_buf",
"clk_240_buf",
"clk_216_buf",
"clk_xtal_12M_buf",
};


#define g_clk_sdio_p_num 0x10

char *g_clk_sdio_p_ptr[] =
{
"clk_xtal_12M_buf",
"clk_20_buf",
"clk_32_buf",
"clk_36_buf",
"clk_40_buf",
"clk_43_2_buf",
"clk_54_buf",
"clk_62_buf",
"clk_72_buf",
"clk_86_buf",
"0",
"clk_sdio_1x_p",
"clk_sdio_2x_p",
"clk_0_3m_buf",
"clk_xtal_12M_buf",
"clk_48_buf",
};


#define g_clk_nfie_p_num 0x10

char *g_clk_nfie_p_ptr[] =
{
"clk_nfie_xtali_sel",
"clk_20_buf",
"clk_32_buf",
"clk_36_buf",
"clk_40_buf",
"clk_43_2_buf",
"clk_54_buf",
"clk_62_buf",
"clk_72_buf",
"clk_86_buf",
"clk_emmc_pll_buf",
"clk_nfie_1x_p",
"clk_nfie_2x_p",
"clk_0_3m_buf",
"clk_nfie_24_sel",
"clk_48_buf",
};


#define g_clk_ecc_num 0x8

char *g_clk_ecc_ptr[] =
{
"clk_xtal_12M_buf",
"clk_54_buf",
"clk_108_buf",
"clk_160_buf",
"clk_216_buf",
"REV",
"REV",
"REV",
};


#define g_clk_vedac_digital_num 0x8

char *g_clk_vedac_digital_ptr[] =
{
"clk_27_buf",
"clk_54_buf",
"clk_108_buf",
"clk_xtal_12M_buf",
"clk_dac_external_buf",
"REV",
"REV",
"REV",
};


#define g_clk_ve_num 0x4

char *g_clk_ve_ptr[] =
{
"clk_27_buf",
"REV",
"clk_xtal_12M_buf",
"REV",
};


#define CLKM_TABLE_NUM	35




typedef struct _clock_table
{

char *g_port_name;;
S32	has_clk_gate;
U32 clk_gate_bank;
U32 clk_gate_reg;
U16 clk_gate_offset;
S32 clk_gate_rev;
S32 has_clk_sel;
U32 clk_sel_bank;
U32 clk_sel_reg;
U16 clksel_offset_start;
U16 clksel_offset_end;
U32 src_num;
S32 handle;
char **g_src_name_ptr;

}clock_table;


typedef struct _clock_table_param
{
clock_table* clock_table_array_ref;
s32 num;
}clock_table_param;


clock_table clock_table_array[CLKM_TABLE_NUM] =
{

{ "g_clk_ge" , HAS_GATE_REG,	0x100B,	0x048 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x048,	2,	3,	g_clk_ge_num,	0,	g_clk_ge_ptr },

{ "g_clk_gpd" , HAS_GATE_REG,	0x1033,	0x028 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x1033,	0x028,	2,	4,	g_clk_gpd_num,	1,	g_clk_gpd_ptr },

{ "g_clk_njpd" , HAS_GATE_REG,	0x100B,	0x035 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x035,	10,	11,	g_clk_njpd_num,	2,	g_clk_njpd_ptr },

{ "g_clk_mfe" , HAS_GATE_REG,	0x1033,	0x018 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x1033,	0x018,	2,	3,	g_clk_mfe_num,	3,	g_clk_mfe_ptr },

{ "g_clk_mvd" , HAS_GATE_REG,	0x100B,	0x039 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x039,	2,	4,	g_clk_mvd_num,	4,	g_clk_mvd_ptr },

{ "g_clk_vd_mheg5" , HAS_GATE_REG,	0x100B,	0x030 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x030,	2,	4,	g_clk_vd_mheg5_num,	5,	g_clk_vd_mheg5_ptr },

{ "g_clk_hvd" , HAS_GATE_REG,	0x100B,	0x031 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x031,	2,	4,	g_clk_hvd_num,	6,	g_clk_hvd_ptr },

{ "g_clk_hvd_aec" , HAS_GATE_REG,	0x100B,	0x034 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x1033,	0x03E,	2,	3,	g_clk_hvd_aec_num,	7,	g_clk_hvd_aec_ptr },

{ "g_clk_vp8" , HAS_GATE_REG,	0x100B,	0x031 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x031,	10,	11,	g_clk_vp8_num,	8,	g_clk_vp8_ptr },

{ "g_clk_hvd_idb" , HAS_GATE_REG,	0x100B,	0x031 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x030,	8,	10,	g_clk_hvd_idb_num,	9,	g_clk_hvd_idb_ptr },

{ "g_clk_hvd_aec_new" , HAS_GATE_REG,	0x100B,	0x034 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x034,	2,	3,	g_clk_hvd_aec_new_num,	10,	g_clk_hvd_aec_new_ptr },

{ "g_clk_evd" , HAS_GATE_REG,	0x100B,	0x034 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x034,	10,	12,	g_clk_evd_num,	11,	g_clk_evd_ptr },

{ "g_clk_evdpu" , HAS_GATE_REG,	0x100B,	0x033 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x033,	10,	12,	g_clk_evdpu_num,	12,	g_clk_evdpu_ptr },

{ "g_clk_aesdma" , HAS_GATE_REG,	0x100A,	0x019 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100A,	0x019,	2,	3,	g_clk_aesdma_num,	13,	g_clk_aesdma_ptr },

{ "g_clk_tsp" , HAS_GATE_REG,	0x100B,	0x02A ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x02A,	2,	3,	g_clk_tsp_num,	14,	g_clk_tsp_ptr },

{ "g_clk_tso_trace" , HAS_GATE_REG,	0x100B,	0x027 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x027,	2,	3,	g_clk_tso_trace_num,	15,	g_clk_tso_trace_ptr },

{ "g_clk_tso_in" , HAS_GATE_REG,	0x100B,	0x027 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x027,	10,	12,	g_clk_tso_in_num,	16,	g_clk_tso_in_ptr },

{ "g_clk_tso_out" , HAS_GATE_REG,	0x100B,	0x07D ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x07D,	10,	12,	g_clk_tso_out_num,	17,	g_clk_tso_out_ptr },

{ "g_clk_tso1_in" , HAS_GATE_REG,	0x100A,	0x010 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100A,	0x010,	2,	4,	g_clk_tso1_in_num,	18,	g_clk_tso1_in_ptr },

{ "g_clk_tso2_in" , HAS_GATE_REG,	0x100A,	0x010 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100A,	0x010,	10,	12,	g_clk_tso2_in_num,	19,	g_clk_tso2_in_ptr },

{ "g_clk_ts4" , HAS_GATE_REG,	0x100A,	0x018 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100A,	0x018,	2,	4,	g_clk_ts4_num,	20,	g_clk_ts4_ptr },

{ "g_clk_ts5" , HAS_GATE_REG,	0x100A,	0x018 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100A,	0x018,	10,	12,	g_clk_ts5_num,	21,	g_clk_ts5_ptr },

{ "g_clk_ts_mmt" , HAS_GATE_REG,	0x100A,	0x019 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100A,	0x019,	10,	12,	g_clk_ts_mmt_num,	22,	g_clk_ts_mmt_ptr },

{ "g_clk_ts2" , HAS_GATE_REG,	0x100B,	0x029 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x029,	2,	4,	g_clk_ts2_num,	23,	g_clk_ts2_ptr },

{ "g_clk_ts1" , HAS_GATE_REG,	0x100B,	0x028 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x028,	10,	12,	g_clk_ts1_num,	24,	g_clk_ts1_ptr },

{ "g_clk_parser" , HAS_GATE_REG,	0x100B,	0x02A ,	4,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x02A,	6,	6,	g_clk_parser_num,	25,	g_clk_parser_ptr },

{ "g_clk_vd1" , HAS_GATE_REG,	0x100B,	0x020 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x020,	10,	11,	g_clk_vd1_num,	26,	g_clk_vd1_ptr },

{ "g_clk_vd_32fsc" , HAS_GATE_REG,	0x100B,	0x023 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x023,	10,	11,	g_clk_vd_32fsc_num,	27,	g_clk_vd_32fsc_ptr },

{ "g_clk_vdmcu" , HAS_GATE_REG,	0x100B,	0x021 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x021,	2,	4,	g_clk_vdmcu_num,	28,	g_clk_vdmcu_ptr },

{ "g_clk_r2_secure" , HAS_GATE_REG,	0x1033,	0x03D ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x1033,	0x03D,	2,	3,	g_clk_r2_secure_num,	29,	g_clk_r2_secure_ptr },

{ "g_clk_sdio_p" , HAS_GATE_REG,	0x100B,	0x069 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x069,	2,	5,	g_clk_sdio_p_num,	30,	g_clk_sdio_p_ptr },

{ "g_clk_nfie_p" , HAS_GATE_REG,	0x100B,	0x064 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x064,	2,	5,	g_clk_nfie_p_num,	31,	g_clk_nfie_p_ptr },

{ "g_clk_ecc" , HAS_GATE_REG,	0x100B,	0x067 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x067,	2,	4,	g_clk_ecc_num,	32,	g_clk_ecc_ptr },

{ "g_clk_vedac_digital" , HAS_GATE_REG,	0x100B,	0x024 ,	8,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x024,	10,	12,	g_clk_vedac_digital_num,	33,	g_clk_vedac_digital_ptr },

{ "g_clk_ve" , HAS_GATE_REG,	0x100B,	0x024 ,	0,	CLK_GATE_NORMAL,	HAS_SEL_REG,	0x100B,	0x024,	2,	3,	g_clk_ve_num,	34,	g_clk_ve_ptr },


};

clock_table_param clkm_param = {clock_table_array,CLKM_TABLE_NUM};

#endif
