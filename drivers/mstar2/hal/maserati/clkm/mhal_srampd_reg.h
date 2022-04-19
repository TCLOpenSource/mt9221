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

#ifndef _MHAL_SRAMPD_REG_H
#define _MHAL_SRAMPD_REG_H


typedef struct _sram_ip_info
{
	char *ip_name;
	u32 bank;
	u32 reg;
	u32 offset;
}sram_ip_info;

typedef struct _sram_pd_table
{
	char *block_name;
	u32 sram_pd_ptr_num;
	sram_ip_info *sram_pd_ptr;
}sram_pd_table;

#define	TOP_num	3


sram_ip_info TOP_ptr[]={
	{"MIU_SRAM_PWR_CTRL",0x1012,0x18,15},
	{"MIU1_SRAM_PWR_CTRL",0x1006,0x18,15},
	{"MIU2_SRAM_PWR_CTRL",0x1620,0x18,15},
};

#define	dig_top_pm_num	3


sram_ip_info dig_top_pm_ptr[]={
	{"ip_PWR_CTRL[0]:mhl_cbus",0x1f,0x6e,9},
	{"ip_PWR_CTRL[1]:mcu51",0x10,0x45,0},
	{"ip_PWR_CTRL[2]:ddc",0x04,0x3e,6},
};

#define	demod0_num	14


sram_ip_info demod0_ptr[]={
	{"demod_1_pwr_ctrl",0x1120,0x48,3},
	{"dvb_frontend_pwr_ctrl",0x1120,0x48,6},
	{"mulan_sram_share_pwr_ctrl",0x1120,0x48,5},
	{"dvbs2_pwr_ctrl",0x1120,0x48,15},
	{"dvbt2_pwr_ctrl",0x1120,0x48,14},
	{"manhattan_sram_share_pwr_ctrl",0x1120,0x48,4},
	{"adcdma_pwr_ctrl",0x1120,0x48,12},
	{"backend_pwr_ctrl",0x1120,0x48,11},
	{"dvbt_pwr_ctrl",0x1120,0x48,8},
	{"isdbt_pwr_ctrl",0x1120,0x48,13},
	{"atsc_pwr_ctrl",0x1120,0x48,9},
	{"dtmb_pwr_ctrl",0x1120,0x48,7},
	{"vif_pwr_ctrl",0x1120,0x48,10},
	{"mcu_pwr_ctrl",0x10,0x45,0},
};

#define	demod1_num	1


sram_ip_info demod1_ptr[]={
	{"demod_1_pwr_ctrl",0x1120,0x48,3},
};

#define	diamond_num	1


sram_ip_info diamond_ptr[]={
	{"PWR_CTRL",0x1018,0x67,0},
};

#define	dvi_num	5


sram_ip_info dvi_ptr[]={
	{"HDMI_PWR_CTRL_P0",0x1739,0x35,1},
	{"HDMI_PWR_CTRL_P1",0x1739,0x35,2},
	{"HDMI_PWR_CTRL_P2",0x1739,0x35,3},
	{"HDMI_PWR_CTRL_P3",0x1739,0x35,4},
	{"HDMI_PWR_CTRL_TOP",0x1739,0x35,0},
};

#define	gpu_num	1


sram_ip_info gpu_ptr[]={
	{"ip00_PWR_CTRL",0x1108,0x43,3},
};

#define	hi_codec_num	2


sram_ip_info hi_codec_ptr[]={
	{"PWR_CTRL_EVD",0x160b,0x01,13},
	{"PWR_CTRL_HVD",0x101b,0x01,13},
};

#define	mheg5_num	15


sram_ip_info mheg5_ptr[]={
	{"ip00_PWR_CTRL:ge",0x1028,0x01,13},
	{"ip01_PWR_CTRL:vd",0x1036,0x7f,9},
	{"ip02_PWR_CTRL:gpd",0x113e,0x6b,14},
	{"ip03_PWR_CTRL:mvd",0x1011,0x21,0},
	{"ip05_PWR_CTRL:evd_r2",0x1633,0x70,0},
	{"ip07_PWR_CTRL:emac",0x1021,0x20,0},
	{"ip08_PWR_CTRL:adc",0x1026,0x76,0},
	{"ip09_PWR_CTRL:uhc0",0x1007,0x05,14},
	{"ip10_PWR_CTRL:uhc1",0x1007,0x45,14},
	{"ip11_PWR_CTRL:uhc2",0x1138,0x05,14},
	{"ip12_PWR_CTRL:bdma",0x103c,0x09,1},
	{"ip13_PWR_CTRL:mailbox",0x103c,0x09,0},
	{"ip14_PWR_CTRL:vd_mcu51",0x10,0x45,0},
	{"ip15_PWR_CTRL:fcie",0x1113,0x39,15},
	{"ip16_PWR_CTRL:sram_usb30",0x1225,0x13,0},
};

#define	tsp_num	16


sram_ip_info tsp_ptr[]={
	{"ip00_PWR_CTRL:dscrmb",0x113c,0x0c,0},
	{"ip01_PWR_CTRL:dscrmb",0x113c,0x0c,1},
	{"ip02_PWR_CTRL:dscrmb",0x113c,0x0c,2},
	{"ip03_PWR_CTRL:dscrmb",0x123c,0x28,7},
	{"ip04_PWR_CTRL:tsp",0x1015,0x7a,9},
	{"ip05_PWR_CTRL:dscrmb",0x122c,0x78,3},
	{"ip06_PWR_CTRL:tso",0x123d,0x30,7},
	{"ip07_PWR_CTRL:sdio",0x120f,0x39,15},
	{"ip08_PWR_CTRL:jpd",0x1232,0x02,15},
	{"ip10_PWR_CTRL:mfe",0x1111,0x50,0},
	{"ip11_PWR_CTRL:sec_r2",0x122a,0x70,0},
	{"ip12_PWR_CTRL:dscrmb",0x113c,0x0c,3},
	{"ip13_PWR_CTRL:dscrmb",0x123c,0x77,11},
	{"ip14_PWR_CTRL:dscrmb",0x173f,0x08,11},
	{"ip19_PWR_CTRL:cmdq",0x1235,0x4c,14},
	{"ip20_PWR_CTRL:cmdq_sub",0x1234,0x4c,14},
};

#define	vivaldi_num	3


sram_ip_info vivaldi_ptr[]={
	{"PWR_CTRL_V9_R2_1",0x1630,0x70,0},
	{"PWR_CTRL_V9_R2",0x1129,0x70,0},
	{"PWR_CTRL",0x112f,0x53,0},
};

#define	fsc_num	32


sram_ip_info fsc_ptr[]={
	{"PWR_CTRL_adc",0x102c,0x3c,0},
	{"PWR_CTRL_afifo",0x102c,0x38,4},
	{"PWR_CTRL_align[0]",0x102c,0x38,12},
	{"PWR_CTRL_align[1]",0x102c,0x38,13},
	{"PWR_CTRL_d2lr",0x102c,0x38,8},
	{"PWR_CTRL_dyn",0x102c,0x3c,4},
	{"PWR_CTRL_fbl",0x102c,0x39,12},
	{"PWR_CTRL_hvsp0[0]",0x102c,0x3a,8},
	{"PWR_CTRL_hvsp0[1]",0x102c,0x3a,9},
	{"PWR_CTRL_hvsp1[0]",0x102c,0x3a,12},
	{"PWR_CTRL_hvsp1[1]",0x102c,0x3a,13},
	{"PWR_CTRL_ipm[0]",0x102c,0x39,0},
	{"PWR_CTRL_ipm[1]",0x102c,0x39,1},
	{"PWR_CTRL_mcm[0]",0x102c,0x3d,0},
	{"PWR_CTRL_mcm[1]",0x102c,0x3d,1},
	{"PWR_CTRL_nr0[0]",0x102c,0x3a,0},
	{"PWR_CTRL_nr0[1]",0x102c,0x3a,1},
	{"PWR_CTRL_nr1[0]",0x102c,0x3a,4},
	{"PWR_CTRL_nr1[1]",0x102c,0x3a,5},
	{"PWR_CTRL_opm[0]",0x102c,0x39,4},
	{"PWR_CTRL_opm[1]",0x102c,0x39,5},
	{"PWR_CTRL_opm[2]",0x102c,0x39,6},
	{"PWR_CTRL_osd[0]",0x102c,0x39,8},
	{"PWR_CTRL_osd[1]",0x102c,0x39,9},
	{"PWR_CTRL_spf0",0x102c,0x3b,0},
	{"PWR_CTRL_spf1",0x102c,0x3b,4},
	{"PWR_CTRL_spt4k[0]",0x102c,0x38,0},
	{"PWR_CTRL_spt4k[1]",0x102c,0x38,1},
	{"PWR_CTRL_vip0[0]",0x102c,0x3b,8},
	{"PWR_CTRL_vip0[1]",0x102c,0x3b,9},
	{"PWR_CTRL_vip1[0]",0x102c,0x3b,12},
	{"PWR_CTRL_vip1[1]",0x102c,0x3b,13},
};

#define	sc_fe_num	82


sram_ip_info sc_fe_ptr[]={
	{"PWRCTRL_SC_FE_DI[0]",0x102f,0x05,0},
	{"PWRCTRL_SC_FE_DI[1]",0x102f,0x05,1},
	{"PWRCTRL_SC_FE_DI[2]",0x102f,0x05,2},
	{"PWRCTRL_SC_FE_DI[3]",0x102f,0x05,3},
	{"PWRCTRL_SC_FE_DI[4]",0x102f,0x05,4},
	{"PWRCTRL_SC_FE_DI[5]",0x102f,0x05,5},
	{"PWRCTRL_SC_FE_DIP[0]",0x102f,0x7e,4},
	{"PWRCTRL_SC_FE_DIP[1]",0x102f,0x7e,5},
	{"PWRCTRL_SC_FE_DIP[2]",0x102f,0x7e,6},
	{"PWRCTRL_SC_FE_DIP[3]",0x102f,0x7e,7},
	{"PWRCTRL_SC_FE_GOP[0]",0x1202,0x3c,11},
	{"PWRCTRL_SC_FE_GOP[1]",0x1205,0x3c,11},
	{"PWRCTRL_SC_FE_GOP[2]",0x1208,0x3c,11},
	{"PWRCTRL_SC_FE_GOP[3]",0x120b,0x3c,11},
	{"PWRCTRL_SC_FE_GOP[4]",0x121b,0x3c,11},
	{"PWRCTRL_SC_FE_HDR[0]",0x102f,0x02,0},
	{"PWRCTRL_SC_FE_HDR[1]",0x102f,0x02,1},
	{"PWRCTRL_SC_FE_HDR[2]",0x102f,0x02,2},
	{"PWRCTRL_SC_FE_HDR[3]",0x102f,0x02,3},
	{"PWRCTRL_SC_FE_HDR[4]",0x102f,0x02,4},
	{"PWRCTRL_SC_FE_HDR[5]",0x102f,0x02,5},
	{"PWRCTRL_SC_FE_HDR[6]",0x102f,0x02,6},
	{"PWRCTRL_SC_FE_HDR[7]",0x102f,0x02,7},
	{"PWRCTRL_SC_FE_HDR[8]",0x102f,0x02,8},
	{"PWRCTRL_SC_FE_HDR[9]",0x102f,0x02,9},
	{"PWRCTRL_SC_FE_HDR[10]",0x102f,0x02,10},
	{"PWRCTRL_SC_FE_HDR[11]",0x102f,0x02,11},
	{"PWRCTRL_SC_FE_HDR[12]",0x102f,0x02,12},
	{"PWRCTRL_SC_FE_IP[0]",0x102f,0x01,0},
	{"PWRCTRL_SC_FE_IP[1]",0x102f,0x01,1},
	{"PWRCTRL_SC_FE_IP[2]",0x102f,0x01,2},
	{"PWRCTRL_SC_FE_IP[3]",0x102f,0x01,3},
	{"PWRCTRL_SC_FE_IP[4]",0x102f,0x01,4},
	{"PWRCTRL_SC_FE_IP[5]",0x102f,0x01,5},
	{"PWRCTRL_SC_FE_IP[6]",0x102f,0x01,6},
	{"PWRCTRL_SC_FE_IP[7]",0x102f,0x01,7},
	{"PWRCTRL_SC_FE_IP[8]",0x102f,0x01,8},
	{"PWRCTRL_SC_FE_IP[9]",0x102f,0x01,9},
	{"PWRCTRL_SC_FE_MVOP[0]",0x1014,0x18,3},
	{"PWRCTRL_SC_FE_MVOP[1]",0x1013,0x18,3},
	{"PWRCTRL_SC_FE_MVOP[2]",0x1014,0x18,4},
	{"PWRCTRL_SC_FE_PDW0[0]",0x102f,0x7f,0},
	{"PWRCTRL_SC_FE_PDW0[1]",0x102f,0x7f,1},
	{"PWRCTRL_SC_FE_PDW0[2]",0x102f,0x7f,2},
	{"PWRCTRL_SC_FE_PDW0[3]",0x102f,0x7f,3},
	{"PWRCTRL_SC_FE_PDW1[0]",0x102f,0x7f,0},
	{"PWRCTRL_SC_FE_PDW1[1]",0x102f,0x7f,1},
	{"PWRCTRL_SC_FE_PDW1[2]",0x102f,0x7f,2},
	{"PWRCTRL_SC_FE_PDW1[3]",0x102f,0x7f,3},
	{"PWRCTRL_SC2_FE_DI[0]",0x102f,0x10,0},
	{"PWRCTRL_SC2_FE_DI[1]",0x102f,0x10,1},
	{"PWRCTRL_SC2_FE_DI[2]",0x102f,0x10,2},
	{"PWRCTRL_SC2_FE_HVSP",0x102f,0x08,0},
	{"PWRCTRL_SC2_FE_IP[0]",0x102f,0x12,0},
	{"PWRCTRL_SC2_FE_IP[1]",0x102f,0x12,1},
	{"PWRCTRL_SC2_FE_IP[2]",0x102f,0x12,2},
	{"PWRCTRL_SC2_FE_IP[3]",0x102f,0x12,3},
	{"PWRCTRL_SC2_FE_IP[4]",0x102f,0x12,4},
	{"PWRCTRL_SC2_FE_IP[5]",0x102f,0x12,5},
	{"PWRCTRL_SC2_FE_IP[6]",0x102f,0x12,6},
	{"PWRCTRL_SC2_FE_OP1[0]",0x102f,0x13,0},
	{"PWRCTRL_SC2_FE_OP1[1]",0x102f,0x13,1},
	{"PWRCTRL_SC2_FE_OP2[0]",0x102f,0x0b,0},
	{"PWRCTRL_SC2_FE_OP2[1]",0x102f,0x0b,1},
	{"PWRCTRL_SC2_FE_OP2[2]",0x102f,0x0b,2},
	{"PWRCTRL_SC2_FE_SCMI[0]",0x102f,0x03,0},
	{"PWRCTRL_SC2_FE_SCMI[1]",0x102f,0x03,1},
	{"PWRCTRL_SC2_FE_SCMI[2]",0x102f,0x03,2},
	{"PWRCTRL_SC2_FE_SCMI[3]",0x102f,0x03,3},
	{"PWRCTRL_SC2_FE_SPF",0x102f,0x0d,0},
	{"PWRCTRL_SC2_FE_VIP[0]",0x102f,0x09,0},
	{"PWRCTRL_SC2_FE_VIP[1]",0x102f,0x09,1},
	{"PWRCTRL_SC2_FE_VIP[2]",0x102f,0x09,2},
	{"PWRCTRL_SC_FE_SCMI[0]",0x102f,0x03,0},
	{"PWRCTRL_SC_FE_SCMI[1]",0x102f,0x03,1},
	{"PWRCTRL_SC_FE_SCMI[2]",0x102f,0x03,2},
	{"PWRCTRL_SC_FE_SCMI[3]",0x102f,0x03,3},
	{"PWRCTRL_SC_FE_SCMI[4]",0x102f,0x03,4},
	{"PWRCTRL_SC_FE_SCMI[5]",0x102f,0x03,5},
	{"PWRCTRL_SC_FE_SCMI[6]",0x102f,0x03,6},
	{"PWRCTRL_SC_FE_SCMI[7]",0x102f,0x03,7},
	{"PWRCTRL_SC_FE_VE",0x103b,0x40,15},
};

#define	sc_be_num	48


sram_ip_info sc_be_ptr[]={
	{"PWRCTRL_SC_BE_2PTO4P",0x102f,0x19,0},
	{"PWRCTRL_SC_BE_DISP_M",0x1032,0x0d,7},
	{"PWRCTRL_SC_BE_FO[0]",0x102f,0x69,0},
	{"PWRCTRL_SC_BE_FO[1]",0x102f,0x69,1},
	{"PWRCTRL_SC_BE_FO[2]",0x102f,0x69,2},
	{"PWRCTRL_SC_BE_FO[3]",0x102f,0x69,3},
	{"PWRCTRL_SC_BE_FO[4]",0x102f,0x6d,0},
	{"PWRCTRL_SC_BE_FO[5]",0x102f,0x6c,0},
	{"PWRCTRL_SC_BE_FO[6]",0x102f,0x6c,1},
	{"PWRCTRL_SC_BE_FO[7]",0x102f,0x6f,0},
	{"PWRCTRL_SC_BE_FO[8]",0x102f,0x65,0},
	{"PWRCTRL_SC_BE_FO[9]",0x102f,0x6f,8},
	{"PWRCTRL_SC_BE_FO[10]",0x102f,0x6e,8},
	{"PWRCTRL_SC_BE_FRCM[0]",0x102f,0x0e,0},
	{"PWRCTRL_SC_BE_FRCM[1]",0x102f,0x0e,1},
	{"PWRCTRL_SC_BE_FRCM[2]",0x102f,0x0e,2},
	{"PWRCTRL_SC_BE_FRCM[3]",0x102f,0x0e,3},
	{"PWRCTRL_SC_BE_FRCM[4]",0x102f,0x0e,4},
	{"PWRCTRL_SC_BE_FRCM[5]",0x102f,0x0e,5},
	{"PWRCTRL_SC_BE_GPLUS",0x102f,0x17,0},
	{"PWRCTRL_SC_BE_HVSP[0]",0x102f,0x08,0},
	{"PWRCTRL_SC_BE_HVSP[1]",0x102f,0x08,1},
	{"PWRCTRL_SC_BE_HVSP[2]",0x102f,0x08,2},
	{"PWRCTRL_SC_BE_LD[0]",0x102f,0x12,0},
	{"PWRCTRL_SC_BE_LD[1]",0x102f,0x12,1},
	{"PWRCTRL_SC_BE_OD",0x102f,0x10,0},
	{"PWRCTRL_SC_BE_OP1_0[0]",0x102f,0x13,0},
	{"PWRCTRL_SC_BE_OP1_0[1]",0x102f,0x13,1},
	{"PWRCTRL_SC_BE_OP1_0[2]",0x102f,0x13,2},
	{"PWRCTRL_SC_BE_OP1_1[0]",0x102f,0x14,0},
	{"PWRCTRL_SC_BE_OP1_1[1]",0x102f,0x14,1},
	{"PWRCTRL_SC_BE_OP2[0]",0x102f,0x0b,0},
	{"PWRCTRL_SC_BE_OP2[1]",0x102f,0x0b,1},
	{"PWRCTRL_SC_BE_OP2[2]",0x102f,0x0b,2},
	{"PWRCTRL_SC_BE_OP2[3]",0x102f,0x0b,3},
	{"PWRCTRL_SC_BE_OP2[4]",0x102f,0x0b,4},
	{"PWRCTRL_SC_BE_OP2[5]",0x102f,0x0b,5},
	{"PWRCTRL_SC_BE_RGBW",0x102f,0x10,1},
	{"PWRCTRL_SC_BE_SPF[0]",0x102f,0x0d,0},
	{"PWRCTRL_SC_BE_SPF[1]",0x102f,0x0d,1},
	{"PWRCTRL_SC_BE_SPF[2]",0x102f,0x0d,2},
	{"PWRCTRL_SC_BE_T3D[0]",0x102f,0x16,0},
	{"PWRCTRL_SC_BE_T3D[1]",0x102f,0x16,1},
	{"PWRCTRL_SC_BE_TGEN[0]",0x102f,0x18,0},
	{"PWRCTRL_SC_BE_TGEN[1]",0x102f,0x18,1},
	{"PWRCTRL_SC_BE_VIP[0]",0x102f,0x09,0},
	{"PWRCTRL_SC_BE_VIP[1]",0x102f,0x09,1},
	{"PWRCTRL_SC_BE_VIP[2]",0x102f,0x09,2},
};

#define	frc_fe_num	19


sram_ip_info frc_fe_ptr[]={
	{"MFC_PWR_CTRL[0]",0x4119,0x01,0},
	{"MFC_PWR_CTRL[1]",0x4119,0x01,1},
	{"MFC_PWR_CTRL[2]",0x4119,0x01,2},
	{"MFC_PWR_CTRL[3]",0x4119,0x01,3},
	{"MFC_PWR_CTRL[4]",0x4119,0x01,4},
	{"MFC_PWR_CTRL[5]",0x4119,0x01,5},
	{"MFC_PWR_CTRL[6]",0x4119,0x01,6},
	{"MFC_PWR_CTRL[7]",0x4119,0x01,7},
	{"FRC_R2_PWR_CTRL",0x4007,0x70,0},
	{"PWR_CTRL_align_0",0x4119,0x06,0},
	{"PWR_CTRL_align_1",0x4119,0x06,1},
	{"PWR_CTRL_d2lr",0x4119,0x06,2},
	{"PWR_CTRL_ipm_0",0x4119,0x06,3},
	{"PWR_CTRL_ipm_1",0x4119,0x06,4},
	{"PWR_CTRL_opm_0",0x4119,0x06,5},
	{"PWR_CTRL_opm_1",0x4119,0x06,6},
	{"PWR_CTRL_opm_2",0x4119,0x06,7},
	{"PWR_CTRL_recv",0x4119,0x06,8},
	{"PWR_CTRL_snr",0x4119,0x05,9},
};

#define SRAM_PD_TABLE_NUM 15
sram_pd_table sram_pd_table_array[SRAM_PD_TABLE_NUM] =
{
	{"TOP",TOP_num,TOP_ptr },
	{"dig_top_pm",dig_top_pm_num,dig_top_pm_ptr },
	{"demod0",demod0_num,demod0_ptr },
	{"demod1",demod1_num,demod1_ptr },
	{"diamond",diamond_num,diamond_ptr },
	{"dvi",dvi_num,dvi_ptr },
	{"gpu",gpu_num,gpu_ptr },
	{"hi_codec",hi_codec_num,hi_codec_ptr },
	{"mheg5",mheg5_num,mheg5_ptr },
	{"tsp",tsp_num,tsp_ptr },
	{"vivaldi",vivaldi_num,vivaldi_ptr },
	{"fsc",fsc_num,fsc_ptr },
	{"sc_fe",sc_fe_num,sc_fe_ptr },
	{"sc_be",sc_be_num,sc_be_ptr },
	{"frc_fe",frc_fe_num,frc_fe_ptr },
};

#endif