/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019-2020 MediaTek Inc.
 * Author: Max Tsai <Max-CH.Tsai@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MTK_BW_H
#define __MTK_BW_H

/*********************************************************************
 *                         Private Define                            *
 *********************************************************************/
#define bw_setb(offset, flag, reg)					\
({									\
	typeof(offset) _offset = (offset);				\
	typeof(flag) _flag = (flag);					\
	typeof(reg) _reg = (reg);					\
	(_flag) ?							\
	writew_relaxed((readw_relaxed(_reg) | (1 << _offset)), _reg) :	\
	writew_relaxed((readw_relaxed(_reg) & ~(1 << _offset)), _reg);	\
})

#define CARBLIST(name) static unsigned int ctrlarblist_##name[]
#define BARBLIST(name) static struct blkarb_config blkarblist_##name[]
#define PORTLIST(name) static unsigned int portlist_##name[]
#define PORTCOUNT(name) (sizeof(portlist_##name) / sizeof(portlist_##name[0]))
#define DOMAIN(name) static const char domain_##name[]
#define PCNT(name) static unsigned int pcnt_##name

#define PORT_DEFINE(name) {\
	#name, \
	sizeof(ctrlarblist_##name)/sizeof(ctrlarblist_##name[0]), \
	ctrlarblist_##name, \
	blkarblist_##name, \
	domain_##name, \
	&pcnt_##name, \
}

/*********************************************************************
 *                         Private Structure                         *
 *********************************************************************/
enum miu_type {
	MIU_DIG,
	MIU_DIG_E,
	MIU_ATOP,
	MIU_CTRL_ARB,
	MIU_BLK_ARB,
};
enum miu_channel {
	MIU_CH0 = 0,
	MIU_CH1,
	MIU_CH_ALL,
};

struct mtk_miu {
	struct device	*dev;
	void __iomem	*base;
	u32		id;
	u8		type;
};

struct mtk_dram_cfg {
	u8	channel;
	u32	clock;
	u32	width;
};

struct mtk_mon {
	u8	mid;
	u64	client_cnt;
	u64	*client;
};

struct mtk_mon_res {
	u64	avg_ocpy;
	u64	max_ocpy;
	u64	avg_bw;
	u64	max_bw;
	u64	time;
}mtk_mon_res_def = {0, 0, 0, 0, 0};

struct bw_res {
	u64     bw;
	u64	util;
	u64	effi;
	u64     mbw;
	u64	mutil;
	u64	meffi;
	u64     *pbw;
}bw_res_def = {0, 0, 0, 0, 0, 0, NULL};

struct blk_bw_res {
	struct bw_res	res;
	unsigned int	mask;
	unsigned int	hpmask;
	int		prior;
	int		maxserivcenum;
	unsigned int	latency;
}blk_bw_res_def = {{0, 0, 0, 0, 0, 0, NULL}, 0, 0, -1, -1, 0};

struct blkarb_config {
        unsigned int    pcount;
        unsigned int    *barb_plist;
};

struct module_map {
        const char		*name;
        unsigned int		larb_num;
        unsigned int		*llist;
        struct blkarb_config	*plist;
	const char		*domain;
	const unsigned int	*pcnt;
};

enum modulelist {
	all,
	bist,
	cpu,
	gpu,
	misc,
	ai,
	audio,
	vdec,
	vdecr2,
	mvop,
	mfdec,
	vd,
	mfe,
	pq,
	od,
	memc,
	dip,
	gop,
	ge,
        MODULE_CNT,
};

enum ctrl_group_id {
	GROUP_0,
	GROUP_1,
	GROUP_2,
	GROUP_3,
	GROUP_4,
	GROUP_5,
	GROUP_6,
	GROUP_7,
	GROUP_8,
	GROUP_9,
	GROUP_10,
	GROUP_11,
	GROUP_12,
	GROUP_13,
	GROUP_14,
	GROUP_15,
        GROUP_CNT,
};

enum blk_port_name {
	//Group 0 (MIU)
	MIU_ALL = 0,
	MIU_MPU_MMU = 14,
	MIU_BIST = 15,
	//Group 1 (AUDIO/VDECR2)
	AUDIO_DSP = 0,
	AUDIO_R20_I = 1,
	AUDIO_R20_D = 2,
	AUDIO_R20_BDMA = 3,
	AUDIO_R21_I = 4,
	AUDIO_R21_D = 5,
	AUDIO_R21_BDMA = 6,
	AUDIO_AU = 7,
	AUDIO_PASDMA = 8,
	AUDIO_R2DMA = 9,
	AUDIO_SEDMA = 10,
	AUDIO_MADDMA = 11,
	AUDIO_VBDMA = 12,
	VDEC_EVD_R2_I = 13,
	VDEC_EVD_R2_D = 14,
	//Group 2 (PERIPHERALS / GRAPHIC / VDEC / MEMC)
	PP_I2C_2_1 = 0,
	PP_I2C_4_3 = 1,
	GP_GE = 2,
	PP_BDMA = 3,
	GP_MVD_PAS = 4,
	PP_USB_2 = 5,
	VDEC_VD = 6,
	VDEC_VD_COMB0 = 7,
	VDEC_VD_COMB1 = 8,
	PP_EMAC = 9,
	GP_MVD_SMDB = 10,
	MEMC_R2_I = 11,
	MEMC_R2_D = 12,
	AI_AIE_2 = 13,
	VDEC_VD_R2_BBU = 14,
	PP_SDIO = 15,
	//Group 3 (PERIPHERALS / TSP / GRAPHIC / VENC)
	PP_DEMOD = 0,
	PP_DEMOD_51 = 1,
	PP_USB_0 = 2,
	TSP_FIQ = 3,
	TSP_ALP = 4,
	GP_JPD = 5,
	TSP_FILEIN = 6,
	TSP_PVR = 7,
	PP_USB_1 = 8,
	PP_FCIE = 9,
	TSP_AESDMA = 10,
	TSP_ORZ = 11,
	TSP_VQ = 12,
	VENC_MFE_0 = 13,
	VENC_MFE_1 = 14,
	TSP_SVQTX = 15,
	//Group 4 (MEMC)
	MEMC_HR23_R = 0,
	MEMC_OPME_F4_R = 1,
	MEMC_FRCM2_R = 5,
	MEMC_OPM0_R = 7,
	MEMC_OPME0_R = 9,
	MEMC_OPMI0_R = 11,
	MEMC_ME_1_R = 13,
	MEMC_HR_R = 14,
	MEMC_ME_2_R = 15,
	//Group 5 (MEMC)
	MEMC_HR23_W = 0,
	MEMC_HVDM_2_W = 1,
	MEMC_HVDM_0_W = 10,
	MEMC_IPM_0_W = 12,
	MEMC_ME_1_W = 14,
	MEMC_HR_W = 15,
	//Group 6 (PQ / GRAPHIC / VDEC)
	PQ_IPM_R = 0,
	PQ_OPM_R = 1,
	PQ_ZNR_R = 2,
	GP_OD_R = 3,
	PQ_DDI_R = 4,
	PQ_ADL_DS_R = 5,
	GP_DIP_Y_R = 6,
	GP_DIP_C_R = 7,
	GP_GOP_0_R = 8,
	GP_GOP_1_R = 9,
	GP_GOP_2_R = 10,
	VDEC_MVOP_Y_C_R = 11,
	PQ_HSE_R = 12,
	VDEC_MFDEC_R = 13,
	VDEC_MVOP_SUB_R = 14,
	GP_ODW_R = 15,
	//Group 7 (PQ / GRAPHIC)
	PQ_IPM_W = 0,
	PQ_OPM_W = 1,
	PQ_ZNR_W = 2,
	GP_OD_W = 3,
	PQ_LD_0_W = 4,
	PQ_LD_1_W = 5,
	PQ_VE_W = 6,
	GP_ODW_W = 7,
	PQ_AUL_W = 8,
	PQ_DDI_W = 9,
	GP_DIP_1_W = 10,
	GP_DIP_0_Y_C_W = 11,
	PQ_HSE_W = 12,
	GP_DIP_2_W = 14,
	PQ_ADC_DMA_W = 15,
	//Group 8 (VDEC)
	VDEC_EVD_BBU = 0,
	VDEC_HVD_BBU = 1,
	VDEC_EVD_0 = 2,
	VDEC_EVD_1 = 4,
	VDEC_HVD_2 = 5,
	VDEC_EVD_2 = 6,
	VDEC_HVD_0 = 7,
	//Group 15
	CPU_R = 0,
	CPU_W = 1,
	GPU_R = 2,
	GPU_W = 3,
};

/*-----------------------------------------------------------------------------
 Master LARB PORT definitions
 -----------------------------------------------------------------------------*/
DOMAIN(all) = "ALL";
PCNT(all) = 1;
CARBLIST(all) = {GROUP_0};
PORTLIST(all) = {
	MIU_ALL,
};
BARBLIST(all) = {
{
	PORTCOUNT(all),
	portlist_all
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(bist) = "OTHER";
PCNT(bist) = 1;
CARBLIST(bist) = {GROUP_0};
PORTLIST(bist) = {
	MIU_BIST,
};
BARBLIST(bist) = {
{
	PORTCOUNT(bist),
	portlist_bist
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(cpu) = "OTHER";
PCNT(cpu) = 2;
CARBLIST(cpu) = {GROUP_15};
PORTLIST(cpu) = {
	CPU_R,
	CPU_W,
};
BARBLIST(cpu) = {
{
	PORTCOUNT(cpu),
	portlist_cpu
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(gpu) = "OTHER";
PCNT(gpu) = 2;
CARBLIST(gpu) = {GROUP_15};
PORTLIST(gpu) = {
	GPU_R,
	GPU_W,
};
BARBLIST(gpu) = {
{
	PORTCOUNT(gpu),
	portlist_gpu
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(misc) = "OTHER";
PCNT(misc) = 22;
CARBLIST(misc) = {GROUP_2, GROUP_3};
PORTLIST(misc2) = {
	PP_I2C_2_1,
	PP_I2C_4_3,
	PP_BDMA,
	GP_MVD_PAS,
	PP_USB_2,
	PP_EMAC,
	GP_MVD_SMDB,
	PP_SDIO,
};
PORTLIST(misc3) = {
	PP_DEMOD,
	PP_DEMOD_51,
	PP_USB_0,
	TSP_FIQ,
	TSP_ALP,
	GP_JPD,
	TSP_FILEIN,
	TSP_PVR,
	PP_USB_1,
	PP_FCIE,
	TSP_AESDMA,
	TSP_ORZ,
	TSP_VQ,
	TSP_SVQTX,
};
BARBLIST(misc) = {
{
	PORTCOUNT(misc2),
	portlist_misc2
},
{
	PORTCOUNT(misc3),
	portlist_misc3
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(audio) = "AUDIO";
PCNT(audio) = 13;
CARBLIST(audio) = {GROUP_1};
PORTLIST(audio) = {
	AUDIO_DSP,
	AUDIO_R20_I,
	AUDIO_R20_D,
	AUDIO_R20_BDMA,
	AUDIO_R21_I,
	AUDIO_R21_D,
	AUDIO_R21_BDMA,
	AUDIO_AU,
	AUDIO_PASDMA,
	AUDIO_R2DMA,
	AUDIO_SEDMA,
	AUDIO_MADDMA,
	AUDIO_VBDMA,
};
BARBLIST(audio) = {
{
        PORTCOUNT(audio),
        portlist_audio
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(vdec) = "CODEC";
PCNT(vdec) = 7;
CARBLIST(vdec) = {GROUP_8};
PORTLIST(vdec) = {
	VDEC_EVD_BBU,
	VDEC_HVD_BBU,
	VDEC_EVD_0,
	VDEC_EVD_1,
	VDEC_HVD_2,
	VDEC_EVD_2,
	VDEC_HVD_0,
};
BARBLIST(vdec) = {
{
	PORTCOUNT(vdec),
	portlist_vdec
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(vdecr2) = "CODEC";
PCNT(vdecr2) = 3;
CARBLIST(vdecr2) = {GROUP_1, GROUP_2};
PORTLIST(vdecr21) = {
	VDEC_EVD_R2_I,
	VDEC_EVD_R2_D,
};
PORTLIST(vdecr22) = {
	VDEC_VD_R2_BBU,
};
BARBLIST(vdecr2) = {
{
	PORTCOUNT(vdecr21),
	portlist_vdecr21
},
{
	PORTCOUNT(vdecr22),
	portlist_vdecr22
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(mvop) = "CODEC";
PCNT(mvop) = 2;
CARBLIST(mvop) = {GROUP_6};
PORTLIST(mvop) = {
	VDEC_MVOP_Y_C_R,
	VDEC_MVOP_SUB_R,
};
BARBLIST(mvop) = {
{
	PORTCOUNT(mvop),
	portlist_mvop
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(mfdec) = "CODEC";
PCNT(mfdec) = 1;
CARBLIST(mfdec) = {GROUP_6};
PORTLIST(mfdec) = {
	VDEC_MFDEC_R,
};
BARBLIST(mfdec) = {
{
	PORTCOUNT(mfdec),
	portlist_mfdec
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(vd) = "CODEC";
PCNT(vd) = 3;
CARBLIST(vd) = {GROUP_2};
PORTLIST(vd) = {
	VDEC_VD,
	VDEC_VD_COMB0,
	VDEC_VD_COMB1,
};
BARBLIST(vd) = {
{
	PORTCOUNT(vd),
	portlist_vd
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(mfe) = "CODEC";
PCNT(mfe) = 2;
CARBLIST(mfe) = {GROUP_3};
PORTLIST(mfe) = {
	VENC_MFE_0,
	VENC_MFE_1,
};
BARBLIST(mfe) = {
{
	PORTCOUNT(mfe),
	portlist_mfe
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(pq) = "PQ";
PCNT(pq) = 16;
CARBLIST(pq) = {GROUP_6, GROUP_7};
PORTLIST(pq6) = {
	PQ_IPM_R,
	PQ_OPM_R,
	PQ_ZNR_R,
	PQ_DDI_R,
	PQ_ADL_DS_R,
	PQ_HSE_R,
};
PORTLIST(pq7) = {
	PQ_IPM_W,
	PQ_OPM_W,
	PQ_ZNR_W,
	PQ_LD_0_W,
	PQ_LD_1_W,
	PQ_VE_W,
	PQ_AUL_W,
	PQ_DDI_W,
	PQ_HSE_W,
	PQ_ADC_DMA_W,
};
BARBLIST(pq) = {
{
	PORTCOUNT(pq6),
	portlist_pq6
},
{
	PORTCOUNT(pq7),
	portlist_pq7
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(od) = "PQ";
PCNT(od) = 4;
CARBLIST(od) = {GROUP_6, GROUP_7};
PORTLIST(od6) = {
	GP_OD_R,
	GP_ODW_R,
};
PORTLIST(od7) = {
	GP_OD_W,
	GP_ODW_W,
};
BARBLIST(od) = {
{
	PORTCOUNT(od6),
	portlist_od6
},
{
	PORTCOUNT(od7),
	portlist_od7
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(memc) = "PQ";
PCNT(memc) = 17;
CARBLIST(memc) = {GROUP_2, GROUP_4, GROUP_5};
PORTLIST(memc2) = {
	MEMC_R2_I,
	MEMC_R2_D,
};
PORTLIST(memc4) = {
	MEMC_HR23_R,
	MEMC_OPME_F4_R,
	MEMC_FRCM2_R,
	MEMC_OPM0_R,
	MEMC_OPME0_R,
	MEMC_OPMI0_R,
	MEMC_ME_1_R,
	MEMC_HR_R,
	MEMC_ME_2_R,
};
PORTLIST(memc5) = {
	MEMC_HR23_W,
	MEMC_HVDM_2_W,
	MEMC_HVDM_0_W,
	MEMC_IPM_0_W,
	MEMC_ME_1_W,
	MEMC_HR_W,
};
BARBLIST(memc) = {
{
	PORTCOUNT(memc2),
	portlist_memc2
},
{
	PORTCOUNT(memc4),
	portlist_memc4
},
{
	PORTCOUNT(memc5),
	portlist_memc5
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(dip) = "GRAPHIC";
PCNT(dip) = 5;
CARBLIST(dip) = {GROUP_6, GROUP_7};
PORTLIST(dip6) = {
	GP_DIP_Y_R,
	GP_DIP_C_R,
};
PORTLIST(dip7) = {
	GP_DIP_1_W,
	GP_DIP_0_Y_C_W,
	GP_DIP_2_W,
};
BARBLIST(dip) = {
{
	PORTCOUNT(dip6),
	portlist_dip6
},
{
	PORTCOUNT(dip7),
	portlist_dip7
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(gop) = "GRAPHIC";
PCNT(gop) = 3;
CARBLIST(gop) = {GROUP_6};
PORTLIST(gop) = {
	GP_GOP_0_R,
	GP_GOP_1_R,
	GP_GOP_2_R,
};
BARBLIST(gop) = {
{
	PORTCOUNT(gop),
	portlist_gop
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(ge) = "GRAPHIC";
PCNT(ge) = 1;
CARBLIST(ge) = {GROUP_2};
PORTLIST(ge) = {
	GP_GE,
};
BARBLIST(ge) = {
{
	PORTCOUNT(ge),
	portlist_ge
},
};
/*----------------------------------------------------------------------------*/
DOMAIN(ai) = "AI";
PCNT(ai) = 1;
CARBLIST(ai) = {GROUP_2};
PORTLIST(ai) = {
	AI_AIE_2,
};
BARBLIST(ai) = {
{
	PORTCOUNT(ai),
	portlist_ai
},
};
/*----------------------------------------------------------------------------*/

#endif /* __MTK_BW_H */
