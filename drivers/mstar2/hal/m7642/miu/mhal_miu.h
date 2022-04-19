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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   Mhal_mtlb.h
/// @author MStar Semiconductor Inc.
/// @brief  MTLB Driver Interface
///////////////////////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
// Linux Mhal_miu.h define start
// -----------------------------------------------------------------------------
#ifndef _HAL_MIU_H_
#define _HAL_MIU_H_

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------
#define MIU_HIT_INTERRUPT       (1)
#ifdef MIU_HIT_INTERRUPT
#if(MIU_HIT_INTERRUPT == 1)
#define MIU_IRQ                  E_IRQEXPL_MIU //E_IRQEXPL_MIU
#endif
#endif

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define _FUNC_NOT_USED()        do {} while ( 0 )

#define MIU_MAX_DEVICE                  (1)
#define MIU_MAX_CHANNEL                 (2)
//Max MIU Group number
#define MIU_MAX_GROUP                   (16)
#define MIU_MAX_GP_CLIENT               (16)
#define MIU_MAX_TBL_CLIENT              (MIU_MAX_GROUP*MIU_MAX_GP_CLIENT)
#define MIU_PAGE_SHIFT                  (12) //Unit for MIU protect
#define MIU_PROTECT_ADDRESS_UNIT        (0x20) //Unit for MIU hitted address
#define MIU_MAX_PROTECT_BLOCK           (8)
#define MIU_MAX_PROTECT_ID_NUM          (16)
#define MIU_MAX_PROTECT_ID_GROUP_NUM    (2)
#define MIU_PROTECT_START_END           (2)
#define MIU_BLOCK0_CLIENT_NUMBER        (16)
#define MIU_BLOCK1_CLIENT_NUMBER        (16)
#define MIU_BLOCK2_CLIENT_NUMBER        (16)
#define MIU_BLOCK3_CLIENT_NUMBER        (16)
#define MIU_BLOCK4_CLIENT_NUMBER        (16)
#define MIU_BLOCK5_CLIENT_NUMBER        (16)
#define MIU_BLOCK6_CLIENT_NUMBER        (16)
#define MIU_BLOCK7_CLIENT_NUMBER        (16)
#define MIU_BLOCK8_CLIENT_NUMBER        (16)
#define MIU_BLOCK9_CLIENT_NUMBER        (16)

#ifndef BIT0
#define BIT0  0x0001
#define BIT1  0x0002
#define BIT2  0x0004
#define BIT3  0x0008
#define BIT4  0x0010
#define BIT5  0x0020
#define BIT6  0x0040
#define BIT7  0x0080
#define BIT8  0x0100
#define BIT9  0x0200
#define BIT10 0x0400
#define BIT11 0x0800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000
#endif

#define ARM_MIU_3G_BUS_BASE 0x100000000

#define MIU_OPM_R_MASK 0x0667
#define MIU_OPM_W_MASK 0x0666
#define MIU_MVD_R_MASK 0x06F6
#define MIU_MVD_W_MASK 0x06F7

//$ MIU0 Request Mask functions
#define _MaskMiuReq_OPM_R( m )     HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT1)

#define _MaskMiuReq_DNRB_W( m )    HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT2)
#define _MaskMiuReq_DNRB_R( m )    HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT3)
#define _MaskMiuReq_DNRB_RW( m )   HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT2|BIT3)

#define _MaskMiuReq_SC_RW( m )     HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT1|BIT2|BIT3)

#define _MaskMiuReq_MVOP_R( m )    HAL_MIU_WriteRegBit(MIU_RQ1L_MASK, m, BIT3)

#define _MaskMiuReq_MVD_R( m )     do { HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT4); HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT5); } while(0)
#define _MaskMiuReq_MVD_W( m )     do { HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT4); HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT5); } while(0)
#define _MaskMiuReq_MVD_RW( m )    do { _MaskMiuReq_MVD_R( m ); _MaskMiuReq_MVD_W( m ); } while (0)

#define _MaskMiuReq_AUDIO_RW( m )  _FUNC_NOT_USED()


//$ MIU1 Request Mask functions
#define _MaskMiu1Req_OPM_R( m )     HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT1)

#define _MaskMiu1Req_DNRB_W( m )    HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT2)
#define _MaskMiu1Req_DNRB_R( m )    HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT3)
#define _MaskMiu1Req_DNRB_RW( m )   HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT2|BIT3)

#define _MaskMiu1Req_SC_RW( m )     HAL_MIU_WriteRegBit(MIU_RQ1H_MASK, m, BIT1|BIT2|BIT3)

#define _MaskMiu1Req_MVOP_R( m )    HAL_MIU_WriteRegBit(MIU_RQ1L_MASK, m, BIT3)

#define _MaskMiu1Req_MVD_R( m )     do { HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT4); HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT5); } while(0)
#define _MaskMiu1Req_MVD_W( m )     do { HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT4); HAL_MIU_WriteRegBit(MIU_RQ3L_MASK, m, BIT5); } while(0)
#define _MaskMiu1Req_MVD_RW( m )    do { _MaskMiuReq_MVD_R( m ); _MaskMiuReq_MVD_W( m ); } while (0)

#define _MaskMiu1Req_AUDIO_RW( m )  _FUNC_NOT_USED()

#define MIU_GET_CLIENT_POS(x)       (x & 0x0F)
#define MIU_GET_CLIENT_GROUP(x)     ((x & 0xF0) >> 4)

//-------------------------------------------------------------------------------------------------
//  Define MIU Client TAble
//-------------------------------------------------------------------------------------------------
#define MIU_CLIENT \
0x0  /*MIU_CLIENT_NONE //none can access*/,\
0x0  /*MIU_CLIENT_DUMMY*/,\
0x0  /*MIU_CLIENT_ADCDVIPLL_W*/,\
0x3A /*MIU_CLIENT_AESDMA_RW*/,\
0x0  /*MIU_CLIENT_AU_R2_RW*/,\
0x23 /*MIU_CLIENT_BDMA_RW*/,\
0x0  /*MIU_CLIENT_DC_R //same as MIU_CLIENT_MVOP_64BIT_R*/,\
0x0  /*MIU_CLIENT_DISP_IPATH_DI_W*/,\
0x0  /*MIU_CLIENT_DISP_IPATH_MR_RW*/,\
0x0  /*MIU_CLIENT_DISP_IPATH_NR_RW*/,\
0x0  /*MIU_CLIENT_DMA2_RW*/,\
0x0  /*MIU_CLIENT_DNRA_RW*/,\
0x0  /*MIU_CLIENT_DSCRMB_RW*/,\
0x0  /*MIU_CLIENT_DVBC_ADC_RW*/,\
0x29 /*MIU_CLIENT_EMAC_RW*/,\
0x39 /*MIU_CLIENT_FCIE_RW*/,\
0x0  /*MIU_CLIENT_FDDECICH_R*/,\
0x0  /*MIU_CLIENT_FDSEICH_R*/,\
0x0  /*MIU_CLIENT_G3D_RW*/,\
0x22 /*MIU_CLIENT_GE_RW*/,\
0x0  /*MIU_CLIENT_GOP_W*/,\
0x68 /*MIU_CLIENT_GOP0_R*/,\
0x69 /*MIU_CLIENT_GOP1_R*/,\
0x6A /*MIU_CLIENT_GOP2_R*/,\
0x0  /*MIU_CLIENT_GOP3_R*/,\
0x0  /*MIU_CLIENT_HISPEED_UART_RW*/,\
0x0  /*MIU_CLIENT_HVD_RW*/,\
0x81 /*MIU_CLIENT_HVD_BBU_R*/,\
0x35 /*MIU_CLIENT_JPD_RW*/,\
0x0  /*MIU_CLIENT_M4VE_ME_R*/,\
0x0  /*MIU_CLIENT_M4VE0_RW*/,\
0x0  /*MIU_CLIENT_M4VE2_RW*/,\
0x0  /*MIU_CLIENT_MAU_RW*/,\
0x0  /*MIU_CLIENT_MAU0_W*/,\
0x0  /*MIU_CLIENT_MAU1_R*/,\
0x3D /*MIU_CLIENT_MFE0_W*/,\
0x3E /*MIU_CLIENT_MFE1_R*/,\
0x0  /*MIU_CLIENT_MHEG5_DCACHE_RW*/,\
0x0  /*MIU_CLIENT_MHEG5_ICACHE_R*/,\
0x0  /*MIU_CLIENT_MHEG5_ICACHE_RW*/,\
0x0  /*MIU_CLIENT_MHEG5_GDMA_RW*/,\
0x0  /*MIU_CLIENT_MIPS_R*/,\
0x0  /*MIU_CLIENT_MIPS_W*/,\
0x0  /*MIU_CLIENT_MIPS_RW*/,\
0x0  /*MIU_CLIENT_MOBF_RW*/,\
0x0  /*MIU_CLIENT_MPIF_RW*/,\
0x2A /*MIU_CLIENT_MVD_RW*/,\
0x24 /*MIU_CLIENT_MVD_BBU_RW*/,\
0x0  /*MIU_CLIENT_MVOP_64BIT_R*/,\
0x0  /*MIU_CLIENT_MVOP_128BIT_R*/,\
0x0  /*MIU_CLIENT_NAND_RW*/,\
0x0  /*MIU_CLIENT_OD_R*/,\
0x0  /*MIU_CLIENT_OD_W*/,\
0x0  /*MIU_CLIENT_OD_LSB_W*/,\
0x0  /*MIU_CLIENT_OD_LSB_R*/,\
0x0  /*MIU_CLIENT_OPW_W*/,\
0x0  /*MIU_CLIENT_OTG_RW*/,\
0x23 /*MIU_CLIENT_PM51_RW*/,\
0x0  /*MIU_CLIENT_PVR_W*/,\
0x0  /*MIU_CLIENT_PVR2_W*/,\
0x0  /*MIU_CLIENT_R2M_R*/,\
0x0  /*MIU_CLIENT_R2M_W*/,\
0x0  /*MIU_CLIENT_RASP0_W*/,\
0x0  /*MIU_CLIENT_RASP1_W*/,\
0x0  /*MIU_CLIENT_RVD_BBU_R*/,\
0x0  /*MIU_CLIENT_RVD_RW*/,\
0x0  /*MIU_CLIENT_SC_DNR_R*/,\
0x0  /*MIU_CLIENT_SC_DNR_W*/,\
0x60 /*MIU_CLIENT_SC_IPMAIN_R*/,\
0x70 /*MIU_CLIENT_SC_IPMAIN_W*/,\
0x0  /*MIU_CLIENT_SC_IPSUB_R*/,\
0x0  /*MIU_CLIENT_SC_IPSUB_W*/,\
0x0  /*MIU_CLIENT_SC_OP_R*/,\
0x0  /*MIU_CLIENT_SC_OPM_R*/,\
0x0  /*MIU_CLIENT_SC_TNR_R*/,\
0x0  /*MIU_CLIENT_SC_TNR_W*/,\
0x0  /*MIU_CLIENT_STRLD_RW*/,\
0x0  /*MIU_CLIENT_TSP_R*/,\
0x0  /*MIU_CLIENT_TSP_W*/,\
0x0  /*MIU_CLIENT_TSP_ORZ_R*/,\
0x0  /*MIU_CLIENT_TSP_ORZ_W*/,\
0x0  /*MIU_CLIENT_USB20_RW*/,\
0x32 /*MIU_CLIENT_USB_UHC0_RW*/,\
0x25 /*MIU_CLIENT_USB_UHC1_RW*/,\
0x25 /*MIU_CLIENT_USB_UHC2_RW*/,\
0x28 /*MIU_CLIENT_VD_COMB_R*/,\
0x27 /*MIU_CLIENT_VD_COMB_W*/,\
0x0  /*MIU_CLIENT_VD_TTX_RW*/,\
0x0  /*MIU_CLIENT_VD_TTXSL_W*/,\
0x0  /*MIU_CLIENT_VD_TTXSK_W*/,\
0x76 /*MIU_CLIENT_VE_W*/,\
0x6F /*MIU_CLIENT_VE_R*/,\
0x0  /*MIU_CLIENT_VIF_ADC_W*/,\
0x17 /*MIU_CLIENT_VIVALDI9_AUDMA_RW*/,\
0x0  /*MIU_CLIENT_VIVALDI9_DECODER_R*/,\
0x0  /*MIU_CLIENT_VIVALDI9_DMA_RW*/,\
0x0  /*MIU_CLIENT_VIVALDI9_LNKLST_R*/,\
0x0  /*MIU_CLIENT_VIVALDI9_MAD_RW*/,\
0x0  /*MIU_CLIENT_VIVALDI9_SE_R*/,\
0x0  /*MIU_CLIENT_MSP_ICACHE_RW*/,\
0x0  /*MIU_CLIENT_DISP_IPATH_DI_RW*/,\
0x0  /*MIU_CLIENT_MVOP1_R*/,\
0x0  /*MIU_CLIENT_LDM_W*/,\
0x0  /*MIU_CLIENT_LDM_R*/,\
0x0  /*MIU_CLIENT_T3D_W*/,\
0x0  /*MIU_CLIENT_T3D_R*/,\
0x0  /*MIU_CLIENT_MIIC0_RW*/,\
0x0  /*MIU_CLIENT_MIIC1_RW*/,\
0x0  /*MIU_CLIENT_MIIC2_W*/,\
0x0  /*MIU_CLIENT_MAXID*/,\
0x0  /*MIU_CLIENT_SC_IPMAIN_RW*/,\
0x0  /*MIU_CLIENT_SC_IPSUB_RW*/,\
0x0  /*MIU_CLIENT_SC_OPMAIN_RW*/,\
0x0  /*MIU_CLIENT_FRC_OSD_RW*/,\
0x0  /*MIU_CLIENT_FRC_IP_R*/,\
0x0  /*MIU_CLIENT_FRC_IP_W*/,\
0x0  /*MIU_CLIENT_FRC_OD_R*/,\
0x0  /*MIU_CLIENT_FRC_OD_W*/,\
0x0  /*MIU_CLIENT_FRC_OPM_R*/,\
0x0  /*MIU_CLIENT_FRC_R2_RW*/,\
0x0  /*MIU_CLIENT_FRC_SC_RW*/,\
0x0  /*MIU_CLIENT_SC_OP_W*/,\
0x0  /*MIU_CLIENT_SECURE_R2_RW*/,\
0x0  /*MIU_CLIENT_SC_2D3D_RW*/,\
0x73 /*MIU_CLIENT_SC_OD_W*/,\
0x63 /*MIU_CLIENT_SC_OD_R*/,\
0x0  /*MIU_CLIENT_SC_LD_RW*/,\
0x2D /*MIU_CLIENT_GPD_RW*/,\
0x0  /*MIU_CLIENT_VP6_RW*/,\
0x2F /*MIU_CLIENT_SDIO_RW*/,\
0x0  /*MIU_CLIENT_G3D0_RW*/,\
0x0  /*MIU_CLIENT_G3D1_RW*/,\
0x0  /*MIU_CLIENT_SECEMAC_RW*/,\
0x38 /*MIU_CLIENT_USB_UHC3_RW*/,\
0x37 /*MIU_CLIENT_TSP_PVR0_W*/,\
0x0  /*MIU_CLIENT_TSP_PVR1_W*/,\
0x0  /*MIU_CLIENT_MAU0_RW*/,\
0x0  /*MIU_CLIENT_MAU1_RW*/,\
0x0  /*MIU_CLIENT_TSP_SEC_W*/,\
0x0  /*MIU_CLIENT_OPM_R*/,\
0x0  /*MIU_CLIENT_USB3_RW*/,\
0x0  /*MIU_CLIENT_SC_DIPW_RW*/,\
0x0  /*MIU_CLIENT_CMD_QUEUE_RW*/,\
0x0  /*MIU_CLIENT_TSO_RW*/,\
0x0  /*MIU_CLIENT_VE_2DMCDI_RW*/,\
0x0  /*MIU_CLIENT_SC_IPSUB2_R*/,\
0x0  /*MIU_CLIENT_SC_IPSUB2_W*/,\
0x20 /*MIU_CLIENT_MIIC_DMA_RW*/,\
0x0  /*MIU_CLIENT_UART_DMA_RW*/,\
0x0  /*MIU_CLIENT_NJPD_RW*/,\
0x0  /*MIU_CLIENT_XD2MIU_RW*/,\
0x0  /*MIU_CLIENT_VD_R2D_RW*/,\
0x0  /*MIU_CLIENT_VD_R2I_R*/,\
0x3B /*MIU_CLIENT_TSP_ORZ_RW*/,\
0x3E /*MIU_CLIENT_MVOP_SUB_R*/,\
0x0  /*MIU_CLIENT_SC_DIPW_W*/,\
0x0  /*MIU_CLIENT_T3D_RW*/,\
0x0  /*MIU_CLIENT_BT_RW*/,\
0x0  /*MIU_CLIENT_VE_VBI_R*/,\
0x0  /*MIU_CLIENT_ARM_RW*/,\
0x0  /*MIU_CLIENT_SC1_OP_R*/,\
0x0  /*MIU_CLIENT_SC1_IPMAIN_RW*/,\
0x0  /*MIU_CLIENT_GOP4_R*/,\
0x0  /*MIU_CLIENT_GOP5_R*/,\
0x0  /*MIU_CLIENT_GMAC_RW*/,\
0x0  /*MIU_CLIENT_SATA_RW*/,\
0x0  /*MIU_CLIENT_SC_LOCALDIMING_RW*/,\
0x0  /*MIU_CLIENT_JPD720P_RW*/,\
0x0  /*MIU_CLIENT_SC_IPM2_R*/,\
0x0  /*MIU_CLIENT_VIVALDI_DSC_R*/,\
0x0  /*MIU_CLIENT_TSP_JPD_RW*/,\
0x0  /*MIU_CLIENT_DEMOD_W*/,\
0x0  /*MIU_CLIENT_DEMOD_R*/,\
0x0  /*MIU_CLIENT_DEMOD_ADCDMA_W*/,\
0x0  /*MIU_CLIENT_GPU_RW*/,\
0x0  /*MIU_CLIENT_PDW1_RW*/,\
0x0  /*MIU_CLIENT_GPO0_PDW0_RW*/,\
0x0  /*MIU_CLIENT_EVD_R*/,\
0x0  /*MIU_CLIENT_EVD_RW*/,\
0x0  /*MIU_CLIENT_SC_FRCL_R*/,\
0x0  /*MIU_CLIENT_SC_FRCR_R*/,\
0x0  /*MIU_CLIENT_VD_VBI_RW // do not use this client; use MIU_CLIENT_VD_TTXSL_W instead*/,\
0x0  /*MIU_CLIENT_VD_MHEG5_ICACHE_RW*/,\
0x0  /*MIU_CLIENT_TSP00_RW*/,\
0x0  /*MIU_CLIENT_TSP01_RW*/,\
0x0  /*MIU_CLIENT_TSP02_RW*/,\
0x3C /*MIU_CLIENT_TSP03_RW*/,\
0x0  /*MIU_CLIENT_TSP04_RW*/,\
0x0  /*MIU_CLIENT_TSP05_RW*/,\
0x0  /*MIU_CLIENT_TSP06_RW*/,\
0x0  /*MIU_CLIENT_VIVALDI9_COMBINE_RW*/,\
0x0  /*MIU_CLIENT_TSP07_RW*/,\
0x0  /*MIU_CLIENT_ISDBT_TDI_R*/,\
0x0  /*MIU_CLIENT_ISDBT_TDI_W*/,\
0x0  /*MIU_CLIENT_FI_Queue0_WR*/,\
0x0  /*MIU_CLIENT_FI_Queue1_WR*/,\
0x0  /*MIU_CLIENT_SWDC_RW*/,\
0x0  /*MIU_CLIENT_ISDB1_RW*/,\
0x0  /*MIU_CLIENT_ISDB2_RW*/,\
0x0  /*MIU_CLIENT_MIIC3_RW*/,\
0x0  /*MIU_CLIENT_SECAU_R2_RW*/,\
0x0  /*MIU_CLIENT_SC_LOCALDIMING_R_RW*/,\
0x0  /*MIU_CLIENT_SC_LOCALDIMING_L_RW*/,\
0x0  /*MIU_CLIENT_SC0_L_RW*/,\
0x0  /*MIU_CLIENT_SC0_R_RW*/,\
0x0  /*MIU_CLIENT_TSP_FIQ_RW*/,\
0x1E /*MIU_CLIENT_EVD_R2D_RW*/,\
0x1D /*MIU_CLIENT_EVD_R2I_R*/,\
0x0  /*MIU_CLIENT_SECHVD_RW*/,\
0x0  /*MIU_CLIENT_SECEVD_RW*/,\
0x6D /*MIU_CLIENT_MFDEC_R*/,\
0x0  /*MIU_CLIENT_SECMFDEC_R*/,\
0x0  /*MIU_CLIENT_MIUTEST_R*/,\
0x0  /*MIU_CLIENT_GOP3_PDW0_RW*/,\
0x0  /*MIU_CLIENT_SC1_OPMAIN_RW*/,\
0x0  /*MIU_CLIENT_SC2_IPSUB_RW*/,\
0x0  /*MIU_CLIENT_SC_IPMAIN2_RW*/,\
0x0  /*MIU_CLIENT_SC2_OPMAIN_RW*/,\
0x0  /*MIU_CLIENT_SC_ODL_RW*/,\
0x0  /*MIU_CLIENT_SC_ODR_RW*/,\
0x0  /*MIU_CLIENT_SC1_IPSUB_RW*/,\
0x80 /*MIU_CLIENT_EVD_BBU_R*/,\
0x0  /*MIU_CLIENT_SC_DWIN_W*/,\
0x0  /*MIU_CLIENT_ZDEC_RW*/,\
0x0  /*MIU_CLIENT_ZDEC_ACP_RW*/,\
0x0  /*MIU_CLIENT_USB30_1_RW*/,\
0x0  /*MIU_CLIENT_USB30_2_RW*/,\
0x0  /*MIU_CLIENT_3RDHVD_RW*/,\
0x0  /*MIU_CLIENT_VP9_RW*/,\
0x0  /*MIU_CLIENT_FRC_R*/,\
0x0  /*MIU_CLIENT_FRCM_W*/,\
0x0  /*MIU_CLIENT_SC_OD_RW*/,\
0x0  /*MIU_CLIENT_SC_OPSUB_W*/,\
0x0  /*MIU_CLIENT_FRCS_W*/,\
0x0  /*MIU_CLIENT_EVD_MTFC_W*/,\
0x0  /*MIU_CLIENT_EVD_MTFY_W*/,\
0x0  /*MIU_CLIENT_ZDEC2_RW*/,\
0x0  /*MIU_CLIENT_SC2_IPMAIN_RW*/,\
0x0  /*MIU_CLIENT_MTF_W*/,\
0x0  /*MIU_CLIENT_DBG_R*/,\
0x0  /*MIU_CLIENT_DS_R*/,\
0x2B /*MIU_CLIENT_FRC_R2*/,\
/*MIU_CLIENT_FRC_R2I_R = MIU_CLIENT_FRC_R2*/\
0x0  /*MIU_CLIENT_MVD_RTO_RW*/,\
0x0  /*MIU_CLIENT_FRC_FSCM2_RW*/,\
0x0  /*MIU_CLIENT_FRC_FSCM3_RW*/,\
0x5C /*MIU_CLIENT_FRC_IPM0_W*/,\
0x0  /*MIU_CLIENT_FRC_IPM1_W*/,\
0x47 /*MIU_CLIENT_FRC_OPM0_R*/,\
0x0  /*MIU_CLIENT_FRC_OPM1_R*/,\
0x49 /*MIU_CLIENT_FRC_OPME0_R*/,\
0x0  /*MIU_CLIENT_FRC_OPME1_R*/,\
0x4B /*MIU_CLIENT_FRC_OPMI0_R*/,\
0x0  /*MIU_CLIENT_FRC_OPMI1_R*/,\
0x5E /*MIU_CLIENT_FRC_ME_W*/,\
0x4D /*MIU_CLIENT_FRC_ME_R*/,\
0x5F /*MIU_CLIENT_FRC_HR_W*/,\
0x4E /*MIU_CLIENT_FRC_HR_R*/,\
0x0  /*MIU_CLIENT_FRC_MI_MERGE_RW*/,\
0x0  /*MIU_CLIENT_MC2D_RW*/,\
0x0  /*MIU_CLIENT_CMD_QUEUE1_RW*/,\
0x0  /*MIU_CLIENT_USB_UHC4_RW*/,\
0x30 /*MIU_CLIENT_DEMOD_RW*/,\
0x0  /*MIU_CLIENT_VE_RW*/,\
0x0  /*MIU_CLIENT_SC_PDW_W*/,\
0x0  /*MIU_CLIENT_VIVALDI9_R2_ARB_RW*/,\
0x0  /*MIU_CLIENT_MCU51_DB_TOOL_RW*/,\
0x0  /*MIU_CLIENT_TSP_RW*/,\
0x0  /*MIU_CLIENT_TSP_ORZ2_RW*/,\
0x0  /*MIU_CLIENT_EVD_BBU_RW*/,\
0x0  /*MIU_CLIENT_DVBC_ADC_W*/,\
0x0  /*MIU_CLIENT_GMAC1_RW*/,\
0x0  /*MIU_CLIENT_MFE_RW*/,\
0x0  /*MIU_CLIENT_VD_R2_L_I_R*/,\
0x0  /*MIU_CLIENT_VD_R2_L_D_RW*/,\
0x0  /*MIU_CLIENT_CA_MIU_CROSSBAR_2_RW*/,\
0x0  /*MIU_CLIENT_TSP08_RW*/,\
0x0  /*MIU_CLIENT_ZDEC_LZDMA_RW*/,\
0x0  /*MIU_CLIENT_EVD2_BBU_R*/,\
0x0  /*MIU_CLIENT_GOP3_DWIN_RW*/,\
0x0  /*MIU_CLIENT_MVOP_256BIT_R*/,\
0x0  /*MIU_CLIENT_MFDEC1_R*/,\
0x0  /*MIU_CLIENT_SC_DYN_SCL_R*/,\
0x0  /*MIU_CLIENT_SC1_OPM_R*/,\
0x0  /*MIU_CLIENT_ZDEC_ACP_W*/,\
0x0  /*MIU_CLIENT_CMD_QUEUE_R*/,\
0x0  /*MIU_CLIENT_VIVALDI9_DECODER_RW*/,\
0x0  /*MIU_CLIENT_DEMOD_ADCDMA_RW*/,\
0x0F /*MIU_CLIENT_MIU_BIST*/,\
0x0  /*MIU_CLIENT_CA_MIU_CROSSBAR_0_RW*/,\
0x0  /*MIU_CLIENT_CA_MIU_CROSSBAR_1_RW*/,\
0x0  /*MIU_CLIENT_SECGMAC_RW*/,\
0x0  /*MIU_CLIENT_AU_R2_1_RW*/,\
0x0  /*MIU_CLIENT_TSO_1_RW*/,\
0x0  /*MIU_CLIENT_TSIO_RW*/,\
0x0  /*MIU_CLIENT_PCIE_OUTBOUND_RW*/,\
0x0  /*MIU_CLIENT_PCIE_INBOUND_RW*/,\
0x0  /*MIU_CLIENT_DDI_0_RW*/,\
0x0  /*MIU_CLIENT_SC_DIPW_1_RW*/,\
0x0  /*MIU_CLIENT_EVD_ENGINE1_RW*/,\
0x0  /*MIU_CLIENT_HVD_ENGINE1_RW*/,\
0x0  /*MIU_CLIENT_DDI_1_RW*/,\
0x0  /*MIU_CLIENT_MFDEC0_1_R*/,\
0x0  /*MIU_CLIENT_MFDEC1_1_R*/,\
0x0  /*MIU_CLIENT_AUTO_DOWNLOAD_R*/,\
0x0  /*MIU_CLIENT_MFEH_R*/,\
0x0  /*MIU_CLIENT_AUDIO_RW*/,\
0x0  /*MIU_CLIENT_OD_RW*/,\
0x0  /*MIU_CLIENT_MVOP1_256BIT_R*/,\
0x0  /*MIU_CLIENT_MVD_256BIT_RW*/,\
0x36 /*MIU_CLIENT_TSP_FILEIN_RW*/,\
0x0  /*MIU_CLIENT_TSP_SEC_RW*/,\
0x0  /*MIU_CLIENT_HDR_L_RW*/,\
0x0  /*MIU_CLIENT_HDR_R_RW*/,\
0x0  /*MIU_CLIENT_MIU_CMD_RW*/,\
0x0  /*MIU_CLIENT_G256_POST_ARB_RW*/,\
0x0  /*MIU_CLIENT_G128_POST_ARB_RW*/,\
0x0  /*MIU_CLIENT_G3_PRE_ARB_RW*/,\
0x0  /*MIU_CLIENT_EVD_2_MIU0_RW*/,\
0x0  /*MIU_CLIENT_EVD_2_MIU1_RW*/,\
0x31 /*MIU_CLIENT_DEMOD_MCU51_WR*/,\
0x0  /*MIU_CLIENT_DEMOD1_WR*/,\
0x0  /*MIU_CLIENT_DEMOD2_WR*/,\
0x0  /*MIU_CLIENT_DEMOD3_WR*/,\
0x0  /*MIU_CLIENT_DEMOD4_WR*/,\
0x0  /*MIU_CLIENT_DEMOD5_WR*/,\
0x0  /*MIU_CLIENT_DEMOD6_WR*/,\
0x0  /*MIU_CLIENT_TSO_TX_RW*/,\
0x0  /*MIU_CLIENT_TSO_RX_RW*/,\
0x0  /*MIU_CLIENT_SC_DIP_RW*/,\
0x0  /*MIU_CLIENT_SC_DIP_DI_RW*/,\
0x0  /*MIU_CLIENT_ADL_RW*/,\
0x10 /*MIU_CLIENT_DSP_CACHE_RW*/,\
0x11 /*MIU_CLIENT_R2_I_RW*/,\
0x12 /*MIU_CLIENT_R2_D_RW*/,\
0x13 /*MIU_CLIENT_R2_BDMA_RW*/,\
0x14 /*MIU_CLIENT_R21_I_R*/,\
0x15 /*MIU_CLIENT_R21_D_RW*/,\
0x16 /*MIU_CLIENT_R21_BDMA_RW*/,\
0x0  /*MIU_CLIENT_DMAL2_RW*/,\
0x1B /*MIU_CLIENT_MADDMA_RW*/,\
0x1A /*MIU_CLIENT_SEDMA_RW*/,\
0x0  /*MIU_CLIENT_DMAL2_W*/,\
0x0  /*MIU_CLIENT_USB30_MERGE_RW*/,\
0x0  /*MIU_CLIENT_MVD_R*/,\
0x0  /*MIU_CLIENT_MIIC0_W*/,\
0x0  /*MIU_CLIENT_PCIE_RW*/,\
0x26 /*MIU_CLIENT_VD_RW*/,\
0x0  /*MIU_CLIENT_AESDMA2_RW*/,\
0x45 /*MIU_CLIENT_FRC_FRCM2_R*/,\
0x0  /*MIU_CLIENT_FRC_FRCM3_R*/,\
0x0  /*MIU_CLIENT_FRC_MI_R*/,\
0x5A /*MIU_CLIENT_FRC_HVDM0_W*/,\
0x51 /*MIU_CLIENT_FRC_HVDM1_W*/,\
0x0  /*MIU_CLIENT_MFDEC2_R*/,\
0x0  /*MIU_CLIENT_MFDEC3_R*/,\
0x6B /*MIU_CLIENT_MVOP1_0_R*/,\
0x0  /*MIU_CLIENT_MVOP1_1_R*/,\
0x0  /*MIU_CLIENT_SC_MGWIN1*/,\
0x62 /*MIU_CLIENT_SC_MCDI_R*/,\
0x0  /*MIU_CLIENT_SC_IPM_R2_R*/,\
0x0  /*MIU_CLIENT_SC2_OPM_R*/,\
0x0  /*MIU_CLIENT_SC2_IPM_R*/,\
0x0  /*MIU_CLIENT_SC_AFBC0_R*/,\
0x0  /*MIU_CLIENT_SC_DS_R*/,\
0x0  /*MIU_CLIENT_SC2_DS_R*/,\
0x6B /*MIU_CLIENT_MVOP0_0_R*/,\
0x0  /*MIU_CLIENT_MVOP0_1_R*/,\
0x66 /*MIU_CLIENT_SC_DIPR_0_R*/,\
0x67 /*MIU_CLIENT_SC_DIPR_1_R*/,\
0x0  /*MIU_CLIENT_SC_DOLBY_R*/,\
0x0  /*MIU_CLIENT_SC_CMDQ_R*/,\
0x0  /*MIU_CLIENT_SC_MGWIN0_R*/,\
0x0  /*MIU_CLIENT_SC_AFBC1_R*/,\
0x71 /*MIU_CLIENT_SC_OPMAIN_W*/,\
0x0  /*MIU_CLIENT_MDWIN_W*/,\
0x7B /*MIU_CLIENT_SC_DIP_0_W*/,\
0x7A /*MIU_CLIENT_SC_DIP_1_W*/,\
0x0  /*MIU_CLIENT_SC_PDW0_W*/,\
0x0  /*MIU_CLIENT_SC_PDW1_W*/,\
0x72 /*MIU_CLIENT_SC_MCDI_W*/,\
0x0  /*MIU_CLIENT_SC_DOLBY_W*/,\
0x0  /*MIU_CLIENT_SC2_IPM_W*/,\
0x7F /*MIU_CLIENT_SC_ABC_DMA_W*/,\
/*MIU_CLIENT_SC_ADC_DMA_W = MIU_CLIENT_SC_ABC_DMA_W*/\
0x82 /*MIU_CLIENT_EVD_0_RW*/,\
0x0  /*MIU_CLIENT_EVD_1_RW*/,\
0x0  /*MIU_CLIENT_EVD_2_RW*/,\
0x0  /*MIU_CLIENT_EVD_3_RW*/,\
0x87 /*MIU_CLIENT_HVD_0_RW*/,\
0x0  /*MIU_CLIENT_HVD_1_RW*/,\
0x85 /*MIU_CLIENT_HVD_2_RW*/,\
0x0  /*MIU_CLIENT_MIU_TEST_RW*/,\
0x33 /*MIU_CLIENT_FIQ_RW //MIU_CLIENT_TSP_FIQ_RW*/,\
0x65 /*MIU_CLIENT_ADL_R*/,\
0x0  /*MIU_CLIENT_TSO_2_RW*/,\
0x0  /*MIU_CLIENT_TSO_3_RW*/,\
0x19 /*MIU_CLIENT_VD_R2DMA_RW*/,\
0x0  /*MIU_CLIENT_MVD1_BBU_RW*/,\
0x0  /*MIU_CLIENT_MVD1_RW*/,\
0x0  /*MIU_CLIENT_PCIE1_OUTBOUND_RW*/,\
0x0  /*MIU_CLIENT_PCIE1_INBOUND_RW*/,\
0x0  /*MIU_CLIENT_EVD_LITE_RW*/,\
0x0  /*MIU_CLIENT_EVD_LITE_ENGINE1_RW*/,\
0x0  /*MIU_CLIENT_HVD_LITE_BBU_R*/,\
0x0  /*MIU_CLIENT_MVD1_RTO_RW*/,\
0x0  /*MIU_CLIENT_TSP09_RW*/,\
0x0  /*MIU_CLIENT_USB30_HS_RW*/,\
0x0  /*MIU_CLIENT_USB30M1_RW*/,\
0x0  /*MIU_CLIENT_USB30M1_HS_RW*/,\
0x0  /*MIU_CLIENT_HDR_OPMAIN_RW*/,\
0x0  /*MIU_CLIENT_HDR_IPMAIN_RW*/,\
0x0  /*MIU_CLIENT_AU_R2_2_RW*/,\
0x0  /*MIU_CLIENT_SCPU_MIU_RW*/,\
0x0  /*MIU_CLIENT_PCIE_1_RW*/,\
0x0  /*MIU_CLIENT_NOE_RW*/,\
0x0  /*MIU_CLIENT_WED0_RW*/,\
0x0  /*MIU_CLIENT_WED1_RW*/,\
0x0  /*MIU_CLIENT_MVD_SUBSYSY_RW*/,\
0x0  /*MIU_CLIENT_GE_W_RW*/,\
0x0  /*MIU_CLIENT_DDI_2_RW*/,\
0x0  /*MIU_CLIENT_IPM_RW*/,\
0x0  /*MIU_CLIENT_USB30_SS_RW*/,\
0x0  /*MIU_CLIENT_PE2_RC_TOP*/,\
0x0  /*MIU_CLIENT_MHE_R*/,\
0x0  /*MIU_CLIENT_MHE_W*/,\
0x0  /*MIU_CLIENT_VIVALDI9_AUBDMA_RW*/,\
0x21 /*MIU_CLIENT_GE_CMDQ_RW*/,\
0x2C /*MIU_CLIENT_FRC_R2D_R*/,\
0x44 /*MIU_CLIENT_FRC_LOGO_R*/,\
0x59 /*MIU_CLIENT_FRC_LOGO_W*/,\
0x0  /*MIU_CLIENT_FRC_HVDM0_R*/,\
0x0  /*MIU_CLIENT_FRC_MV_R*/,\
0x0  /*MIU_CLIENT_FRC_MV_W*/,\
0x0  /*MIU_CLIENT_FRC_MCMV_R*/,\
0x84 /*MIU_CLIENT_EVD1_R*/,\
0x86 /*MIU_CLIENT_EVD2_R*/,\
0x0  /*MIU_CLIENT_HVD1_R*/,\
0x61 /*MIU_CLIENT_SC_OPMAIN_R*/,\
0x0  /*MIU_CLIENT_SC_LD_R*/,\
0x74 /*MIU_CLIENT_SC_LD_W*/,\
0x75 /*MIU_CLIENT_SC_LD1_W*/,\
0x0  /*MIU_CLIENT_SC_HDR_DMA_R*/,\
0x0  /*MIU_CLIENT_SC_HDR_DMA_W*/,\
0xF0 /*MIU_CLIENT_CPU_R*/,\
0xF1 /*MIU_CLIENT_CPU_W*/,\
0xF2 /*MIU_CLIENT_GPU_R*/,\
0xF3 /*MIU_CLIENT_GPU_W*/,\
0x0  /*MIU_CLIENT_TSP_UFSHCI_RW*/,\
0x0  /*MIU_CLIENT_VREC_RW*/,\
0x18 /*MIU_CLIENT_PASDMA_W*/,\
0x34 /*MIU_CLIENT_TSP_ALP_W*/,\
0x3F /*MIU_CLIENT_TSO_R*/,\
0x77 /*MIU_CLIENT_SC_ODW_W*/,\
0x78 /*MIU_CLIENT_SC_AUL_W*/,\
0x0E /*MIU_CLIENT_MPU_MMU_RW*/,\
0x0  /*MIU_CLIENT_DDI_3_RW*/,\
0x0  /*MIU_CLIENT_AUTO_UPLOAD_W*/,\
0x0  /*MIU_CLIENT_AUTO_DOWNLOAD_RW*/,\
0x7E /*MIU_CLIENT_SC_DIP_2_W*/,\
0x0  /*MIU_CLIENT_SC_DIP_3_RW*/,\
0x0  /*MIU_CLIENT_SC_ABF_W0_RW*/,\
0x0  /*MIU_CLIENT_SC_ABF_W1_RW*/,\
0x0  /*MIU_CLIENT_SC2_OP_R*/,\
0x0  /*MIU_CLIENT_SC_DS_RW*/,\
0x0  /*MIU_CLIENT_SC_DIPR_R*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP0*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP1*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP2*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP3*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP4*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP5*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP6*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP7*/,\
0x0  /*MIU_CLIENT_MONITOR_GROUP8*/,\
0x0  /*MIU_CLIENT_MHEG_5_RW*/,\
0x2E /*MIU_CLIENT_R2_BBU_RW*/,\
0x0  /*MIU_CLIENT_MFDEC0_0_R*/,\
0x0  /*MIU_CLIENT_MFDEC1_0_R*/,\
0x0  /*MIU_CLIENT_SC_ODW_R*/,\
0x40 /*MIU_CLIENT_FRC_HR23_R*/,\
0x41 /*MIU_CLIENT_FRC_OPME0_F4_R*/,\
0x4F /*MIU_CLIENT_FRC_BE_ME_R*/,\
0x50 /*MIU_CLIENT_FRC_HR23_W*/,\
0x64 /*MIU_CLIENT_DDI_R*/,\
0x6C /*MIU_CLIENT_HSE_R*/,\
0x79 /*MIU_CLIENT_DDI_W*/,\
0x7C /*MIU_CLIENT_HSE_W*/,\
0x1C /*MIU_CLIENT_VBDMA_RW*/,\
0x90 /*MIU_CLIENT_AIA_R_RW*/,\
0x91 /*MIU_CLIENT_AIA_W_RW*/

#define MIU_CLIENT_NAME \
"NONE",\
"DUMMY",\
"ADCDVIPLL_W",\
"AESDMA_RW",\
"AU_R2_RW",\
"BDMA_RW",\
"DC_R",/*DC_R //same as MVOP_64BIT_R*/\
"DISP_IPATH_DI_W",\
"DISP_IPATH_MR_RW",\
"DISP_IPATH_NR_RW",\
"DMA2_RW",\
"DNRA_RW",\
"DSCRMB_RW",\
"DVBC_ADC_RW",\
"EMAC_RW",\
"FCIE_RW",\
"FDDECICH_R",\
"FDSEICH_R",\
"G3D_RW",\
"GE_RW",\
"GOP_W",\
"GOP0_R",\
"GOP1_R",\
"GOP2_R",\
"GOP3_R",\
"HISPEED_UART_RW",\
"HVD_RW",\
"HVD_BBU_R",\
"JPD_RW",\
"M4VE_ME_R",\
"M4VE0_RW",\
"M4VE2_RW",\
"MAU_RW",\
"MAU0_W",\
"MAU1_R",\
"MFE0_W",\
"MFE1_R",\
"MHEG5_DCACHE_RW",\
"MHEG5_ICACHE_R",\
"MHEG5_ICACHE_RW",\
"MHEG5_GDMA_RW",\
"MIPS_R",\
"MIPS_W",\
"MIPS_RW",\
"MOBF_RW",\
"MPIF_RW",\
"MVD_RW",\
"MVD_BBU_RW",\
"MVOP_64BIT_R",\
"MVOP_128BIT_R",\
"NAND_RW",\
"OD_R",\
"OD_W",\
"OD_LSB_W",\
"OD_LSB_R",\
"OPW_W",\
"OTG_RW",\
"PM51_RW",\
"PVR_W",\
"PVR2_W",\
"R2M_R",\
"R2M_W",\
"RASP0_W",\
"RASP1_W",\
"RVD_BBU_R",\
"RVD_RW",\
"SC_DNR_R",\
"SC_DNR_W",\
"SC_IPMAIN_R",\
"SC_IPMAIN_W",\
"SC_IPSUB_R",\
"SC_IPSUB_W",\
"SC_OP_R",\
"SC_OPM_R",\
"SC_TNR_R",\
"SC_TNR_W",\
"STRLD_RW",\
"TSP_R",\
"TSP_W",\
"TSP_ORZ_R",\
"TSP_ORZ_W",\
"USB20_RW",\
"USB_UHC0_RW",\
"USB_UHC1_RW",\
"USB_UHC2_RW",\
"VD_COMB_R",\
"VD_COMB_W",\
"VD_TTX_RW",\
"VD_TTXSL_W",\
"VD_TTXSK_W",\
"VE_W",\
"VE_R",\
"VIF_ADC_W",\
"VIVALDI9_AUDMA_RW",\
"VIVALDI9_DECODER_R",\
"VIVALDI9_DMA_RW",\
"VIVALDI9_LNKLST_R",\
"VIVALDI9_MAD_RW",\
"VIVALDI9_SE_R",\
"MSP_ICACHE_RW",\
"DISP_IPATH_DI_RW",\
"MVOP1_R",\
"LDM_W",\
"LDM_R",\
"T3D_W",\
"T3D_R",\
"MIIC0_RW",\
"MIIC1_RW",\
"MIIC2_W",\
"MAXID",\
"SC_IPMAIN_RW",\
"SC_IPSUB_RW",\
"SC_OPMAIN_RW",\
"FRC_OSD_RW",\
"FRC_IP_R",\
"FRC_IP_W",\
"FRC_OD_R",\
"FRC_OD_W",\
"FRC_OPM_R",\
"FRC_R2_RW",\
"FRC_SC_RW",\
"SC_OP_W",\
"SECURE_R2_RW",\
"SC_2D3D_RW",\
"SC_OD_W",\
"SC_OD_R",\
"SC_LD_RW",\
"GPD_RW",\
"VP6_RW",\
"SDIO_RW",\
"G3D0_RW",\
"G3D1_RW",\
"SECEMAC_RW",\
"USB_UHC3_RW",\
"TSP_PVR0_W",\
"TSP_PVR1_W",\
"MAU0_RW",\
"MAU1_RW",\
"TSP_SEC_W",\
"OPM_R",\
"USB3_RW",\
"SC_DIPW_RW",\
"CMD_QUEUE_RW",\
"TSO_RW",\
"VE_2DMCDI_RW",\
"SC_IPSUB2_R",\
"SC_IPSUB2_W",\
"MIIC_DMA_RW",\
"UART_DMA_RW",\
"NJPD_RW",\
"XD2MIU_RW",\
"VD_R2D_RW",\
"VD_R2I_R",\
"TSP_ORZ_RW",\
"MVOP_SUB_R",\
"SC_DIPW_W",\
"T3D_RW",\
"BT_RW",\
"VE_VBI_R",\
"ARM_RW",\
"SC1_OP_R",\
"SC1_IPMAIN_RW",\
"GOP4_R",\
"GOP5_R",\
"GMAC_RW",\
"SATA_RW",\
"SC_LOCALDIMING_RW",\
"JPD720P_RW",\
"SC_IPM2_R",\
"VIVALDI_DSC_R",\
"TSP_JPD_RW",\
"DEMOD_W",\
"DEMOD_R",\
"DEMOD_ADCDMA_W",\
"GPU_RW",\
"PDW1_RW",\
"GPO0_PDW0_RW",\
"EVD_R",\
"EVD_RW",\
"SC_FRCL_R",\
"SC_FRCR_R",\
"VD_VBI_RW",/*VD_VBI_RW // do not use this client; use VD_TTXSL_W instead*/\
"VD_MHEG5_ICACHE_RW",\
"TSP00_RW",\
"TSP01_RW",\
"TSP02_RW",\
"TSP03_RW",\
"TSP04_RW",\
"TSP05_RW",\
"TSP06_RW",\
"VIVALDI9_COMBINE_RW",\
"TSP07_RW",\
"ISDBT_TDI_R",\
"ISDBT_TDI_W",\
"FI_Queue0_WR",\
"FI_Queue1_WR",\
"SWDC_RW",\
"ISDB1_RW",\
"ISDB2_RW",\
"MIIC3_RW",\
"SECAU_R2_RW",\
"SC_LOCALDIMING_R_RW",\
"SC_LOCALDIMING_L_RW",\
"SC0_L_RW",\
"SC0_R_RW",\
"TSP_FIQ_RW",\
"EVD_R2D_RW",\
"EVD_R2I_R",\
"SECHVD_RW",\
"SECEVD_RW",\
"MFDEC_R",\
"SECMFDEC_R",\
"MIUTEST_R",\
"GOP3_PDW0_RW",\
"SC1_OPMAIN_RW",\
"SC2_IPSUB_RW",\
"SC_IPMAIN2_RW",\
"SC2_OPMAIN_RW",\
"SC_ODL_RW",\
"SC_ODR_RW",\
"SC1_IPSUB_RW",\
"EVD_BBU_R",\
"SC_DWIN_W",\
"ZDEC_RW",\
"ZDEC_ACP_RW",\
"USB30_1_RW",\
"USB30_2_RW",\
"3RDHVD_RW",\
"VP9_RW",\
"FRC_R",\
"FRCM_W",\
"SC_OD_RW",\
"SC_OPSUB_W",\
"FRCS_W",\
"EVD_MTFC_W",\
"EVD_MTFY_W",\
"ZDEC2_RW",\
"SC2_IPMAIN_RW",\
"MTF_W",\
"DBG_R",\
"DS_R",\
"FRC_R2",\
/*FRC_R2I_R = FRC_R2*/\
"MVD_RTO_RW",\
"FRC_FSCM2_RW",\
"FRC_FSCM3_RW",\
"FRC_IPM0_W",\
"FRC_IPM1_W",\
"FRC_OPM0_R",\
"FRC_OPM1_R",\
"FRC_OPME0_R",\
"FRC_OPME1_R",\
"FRC_OPMI0_R",\
"FRC_OPMI1_R",\
"FRC_ME_W",\
"FRC_ME_R",\
"FRC_HR_W",\
"FRC_HR_R",\
"FRC_MI_MERGE_RW",\
"MC2D_RW",\
"CMD_QUEUE1_RW",\
"USB_UHC4_RW",\
"DEMOD_RW",\
"VE_RW",\
"SC_PDW_W",\
"VIVALDI9_R2_ARB_RW",\
"MCU51_DB_TOOL_RW",\
"TSP_RW",\
"TSP_ORZ2_RW",\
"EVD_BBU_RW",\
"DVBC_ADC_W",\
"GMAC1_RW",\
"MFE_RW",\
"VD_R2_L_I_R",\
"VD_R2_L_D_RW",\
"CA_MIU_CROSSBAR_2_RW",\
"TSP08_RW",\
"ZDEC_LZDMA_RW",\
"EVD2_BBU_R",\
"GOP3_DWIN_RW",\
"MVOP_256BIT_R",\
"MFDEC1_R",\
"SC_DYN_SCL_R",\
"SC1_OPM_R",\
"ZDEC_ACP_W",\
"CMD_QUEUE_R",\
"VIVALDI9_DECODER_RW",\
"DEMOD_ADCDMA_RW",\
"MIU_BIST",\
"CA_MIU_CROSSBAR_0_RW",\
"CA_MIU_CROSSBAR_1_RW",\
"SECGMAC_RW",\
"AU_R2_1_RW",\
"TSO_1_RW",\
"TSIO_RW",\
"PCIE_OUTBOUND_RW",\
"PCIE_INBOUND_RW",\
"DDI_0_RW",\
"SC_DIPW_1_RW",\
"EVD_ENGINE1_RW",\
"HVD_ENGINE1_RW",\
"DDI_1_RW",\
"MFDEC0_1_R",\
"MFDEC1_1_R",\
"AUTO_DOWNLOAD_R",\
"MFEH_R",\
"AUDIO_RW",\
"OD_RW",\
"MVOP1_256BIT_R",\
"MVD_256BIT_RW",\
"TSP_FILEIN_RW",\
"TSP_SEC_RW",\
"HDR_L_RW",\
"HDR_R_RW",\
"MIU_CMD_RW",\
"G256_POST_ARB_RW",\
"G128_POST_ARB_RW",\
"G3_PRE_ARB_RW",\
"EVD_2_MIU0_RW",\
"EVD_2_MIU1_RW",\
"DEMOD_MCU51_WR",\
"DEMOD1_WR",\
"DEMOD2_WR",\
"DEMOD3_WR",\
"DEMOD4_WR",\
"DEMOD5_WR",\
"DEMOD6_WR",\
"TSO_TX_RW",\
"TSO_RX_RW",\
"SC_DIP_RW",\
"SC_DIP_DI_RW",\
"ADL_RW",\
"DSP_CACHE_RW",\
"R2_I_RW",\
"R2_D_RW",\
"R2_BDMA_RW",\
"R21_I_R",\
"R21_D_RW",\
"R21_BDMA_RW",\
"DMAL2_RW",\
"MADDMA_RW",\
"SEDMA_RW",\
"DMAL2_W",\
"USB30_MERGE_RW",\
"MVD_R",\
"MIIC0_W",\
"PCIE_RW",\
"VD_RW",\
"AESDMA2_RW",\
"FRC_FRCM2_R",\
"FRC_FRCM3_R",\
"FRC_MI_R",\
"FRC_HVDM0_W",\
"FRC_HVDM1_W",\
"MFDEC2_R",\
"MFDEC3_R",\
"MVOP1_0_R",\
"MVOP1_1_R",\
"SC_MGWIN1",\
"SC_MCDI_R",\
"SC_IPM_R2_R",\
"SC2_OPM_R",\
"SC2_IPM_R",\
"SC_AFBC0_R",\
"SC_DS_R",\
"SC2_DS_R",\
"MVOP0_0_R",\
"MVOP0_1_R",\
"SC_DIPR_0_R",\
"SC_DIPR_1_R",\
"SC_DOLBY_R",\
"SC_CMDQ_R",\
"SC_MGWIN0_R",\
"SC_AFBC1_R",\
"SC_OPMAIN_W",\
"MDWIN_W",\
"SC_DIP_0_W",\
"SC_DIP_1_W",\
"SC_PDW0_W",\
"SC_PDW1_W",\
"SC_MCDI_W",\
"SC_DOLBY_W",\
"SC2_IPM_W",\
"SC_ABC_DMA_W",\
/*SC_ADC_DMA_W = SC_ABC_DMA_W*/\
"EVD_0_RW",\
"EVD_1_RW",\
"EVD_2_RW",\
"EVD_3_RW",\
"HVD_0_RW",\
"HVD_1_RW",\
"HVD_2_RW",\
"MIU_TEST_RW",\
"FIQ_RW",\
"ADL_R",\
"TSO_2_RW",\
"TSO_3_RW",\
"VD_R2DMA_RW",\
"MVD1_BBU_RW",\
"MVD1_RW",\
"PCIE1_OUTBOUND_RW",\
"PCIE1_INBOUND_RW",\
"EVD_LITE_RW",\
"EVD_LITE_ENGINE1_RW",\
"HVD_LITE_BBU_R",\
"MVD1_RTO_RW",\
"TSP09_RW",\
"USB30_HS_RW",\
"USB30M1_RW",\
"USB30M1_HS_RW",\
"HDR_OPMAIN_RW",\
"HDR_IPMAIN_RW",\
"AU_R2_2_RW",\
"SCPU_MIU_RW",\
"PCIE_1_RW",\
"NOE_RW",\
"WED0_RW",\
"WED1_RW",\
"MVD_SUBSYSY_RW",\
"GE_W_RW",\
"DDI_2_RW",\
"IPM_RW",\
"USB30_SS_RW",\
"PE2_RC_TOP",\
"MHE_R",\
"MHE_W",\
"VIVALDI9_AUBDMA_RW",\
"GE_CMDQ_RW",\
"FRC_R2D_R",\
"FRC_LOGO_R",\
"FRC_LOGO_W",\
"FRC_HVDM0_R",\
"FRC_MV_R",\
"FRC_MV_W",\
"FRC_MCMV_R",\
"EVD1_R",\
"EVD2_R",\
"HVD1_R",\
"SC_OPMAIN_R",\
"SC_LD_R",\
"SC_LD_W",\
"SC_LD1_W",\
"SC_HDR_DMA_R",\
"SC_HDR_DMA_W",\
"CPU_R",\
"CPU_W",\
"GPU_R",\
"GPU_W",\
"TSP_UFSHCI_RW",\
"VREC_RW",\
"PASDMA_W",\
"TSP_ALP_W",\
"TSO_R",\
"SC_ODW_W",\
"SC_AUL_W",\
"MPU_MMU_RW",\
"DDI_3_RW",\
"AUTO_UPLOAD_W",\
"AUTO_DOWNLOAD_RW",\
"SC_DIP_2_W",\
"SC_DIP_3_RW",\
"SC_ABF_W0_RW",\
"SC_ABF_W1_RW",\
"SC2_OP_R",\
"SC_DS_RW",\
"SC_DIPR_R",\
"MIU_CLIENT_MONITOR_GROUP0",\
"MIU_CLIENT_MONITOR_GROUP1",\
"MIU_CLIENT_MONITOR_GROUP2",\
"MIU_CLIENT_MONITOR_GROUP3",\
"MIU_CLIENT_MONITOR_GROUP4",\
"MIU_CLIENT_MONITOR_GROUP5",\
"MIU_CLIENT_MONITOR_GROUP6",\
"MIU_CLIENT_MONITOR_GROUP7",\
"MIU_CLIENT_MONITOR_GROUP8",\
"MHEG_5_RW",\
"R2_BBU_RW",\
"MFDEC0_0_R",\
"MFDEC1_0_R",\
"SC_ODW_R",\
"FRC_HR23_R",\
"FRC_OPME0_F4_R",\
"FRC_BE_ME_R",\
"FRC_HR23_W",\
"DDI_R",\
"HSE_R",\
"DDI_W",\
"HSE_W",\
"VBDMA_RW",\
"AIA_R_RW",\
"AIA_W_RW"

//-------------------------------------------------------------------------------------------------
//  MIU Kernel Protect White List
//-------------------------------------------------------------------------------------------------
#define MIU_KERNEL_PROTECT_WHITE_LIST \
MIU_CLIENT_CPU_R,\
MIU_CLIENT_CPU_W,\
MIU_CLIENT_GPU_R,\
MIU_CLIENT_GPU_W,\
MIU_CLIENT_USB_UHC0_RW,\
MIU_CLIENT_USB_UHC1_RW,\
MIU_CLIENT_USB_UHC3_RW,\
MIU_CLIENT_SDIO_RW,\
MIU_CLIENT_SC_DIP_0_W,\
MIU_CLIENT_FCIE_RW,\
MIU_CLIENT_EMAC_RW,\
MIU_CLIENT_GE_RW,\
MIU_CLIENT_AIA_R_RW,\
MIU_CLIENT_AIA_W_RW

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_MIU_BLOCK_0 = 0,
    E_MIU_BLOCK_1,
    E_MIU_BLOCK_2,
    E_MIU_BLOCK_3,
    E_MIU_BLOCK_4,
    E_MIU_BLOCK_5,
    E_MIU_BLOCK_6,
    E_MIU_BLOCK_7,
    E_MIU_BLOCK_NUM,
} MIU_BLOCK_ID;

typedef enum
{
    E_MIU_ID_GROUP_0 = 0,
    E_MIU_ID_GROUP_1,
    E_MIU_ID_GROUP_BOTH = 16,
    E_MIU_ID_GROUP_NONE,
}MIU_ID_GROUP;

typedef struct
{
    // Order cannot be exchanged
    MS_U16 u16CheckMagicNumber;
    MS_U16 u16DramSize[4];
    MS_U16 u16ProtectIDG0[8];
    MS_U16 u16ProtectIDG1[8];
    MS_U16 u16ProtectIDEnable[8];
    MS_U16 u16ProtectIDGroupSelect;
    MS_U16 u16ProtectAddress[32];
    MS_U16 u16ProtectAddressEnable[2];
}MIU_Protect_Info;

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
MS_U32* HAL_MIU_Kernel_GetDefaultClientID_KernelProtect(void);
MS_BOOL HAL_MIU_Kernel_Protect( MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_PHY phyStart, MS_PHY phyEnd, MS_BOOL bSetFlag);
MS_BOOL HAL_MIU_Kernel_ParseOccupiedResource(void);
MS_BOOL HAL_MIU_Set_ADCDMA_Protect(MS_PHY start_addr);
void HAL_MIU_Remove_ADCDMA_Protect(void);

MS_BOOL HAL_MIU_Kernel_GetProtectInfo(MS_U8 u8MiuDev, MIU_PortectInfo *pInfo);
MS_BOOL HAL_MIU_Save(void);
MS_BOOL HAL_MIU_Restore(void);
MS_BOOL HAL_MIU_Kernel_Dram_ReadSize(MS_U8 MiuID, MIU_DDR_SIZE *pDramSize);
MS_BOOL HAL_MIU_Set_DebugLevel(MIU_DBGLV eDebugLevel);
MS_BOOL HAL_MIU_Enable_Hitkernelpanic(MS_U32 bEnablePanic);
MS_U32  HAL_MIU_Get_Hitkernelpanic(void);
#endif // _HAL_MIU_H_
