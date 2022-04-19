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

#ifndef __CHIP_INT_H__
#define __CHIP_INT_H__

#include <mach/platform.h>

/******************************************************
       CA9 Private Timer
*******************************************************/
#define INT_PPI_VTIMER                  27 // Virtual timer event
#define INT_PPI_FIQ                     28 //GIC PPI FIQ number
#define INT_ID_PTIMER                   29
#define INT_PPI_IRQ                     31 //GIC PPI IRQ number


#define CONFIG_SPI                      1

#if CONFIG_SPI
   #define CONFIG_SPI_IRQ               1
   #define INT_SPI_IRQ_HOST0            63
   #define INT_SPI_IRQ_HOST1            64
   #define INT_SPI_IRQ_HOST2            65
   #define INT_SPI_IRQ_HOST3            66
   #define INT_SPI_FIQ_HOST0            67
   #define INT_SPI_FIQ_HOST1            68
   #define INT_SPI_FIQ_HOST2            69
   #define INT_SPI_FIQ_HOST3            70
   #define INT_SPI_IRQ                  INT_SPI_IRQ_HOST0
   #define SPI_MIN_NUM                  32
   #define SPI_MAX_NUM                  1020

#endif // end of CONFIG_SPI

//Cortex-A9 GIC PPI
#define IRQ_LOCALTIMER                  30
//#define IRQ_LOCALWDOG                   29

//Cortex-A9 PMU(Performance Monitor Unit)
#define CHIP_IRQ_PMU0                   166
#define CHIP_IRQ_PMU1                   168
#define CHIP_IRQ_PMU2                   170
#define CHIP_IRQ_PMU3                   172

extern ptrdiff_t mstar_pm_base;
extern ptrdiff_t mstar_frc_base;

#define REG_INT_BASE_PA                 (0x1F000000UL  + (0x101900UL << 1))
#define REG_INT_BASE                    (mstar_pm_base + (0x101900UL << 1))
#define REG_INT_FRC_BASE_PA             (0x1F800000UL  + (0x000000UL << 1))
#define REG_INT_FRC_BASE                (mstar_frc_base + (0x000000UL << 1))
#define REG_INT_HYP_BASE_PA             (0x1F000000UL  + (0x101000UL << 1))
#define REG_INT_HYP_BASE                (mstar_pm_base + (0x101000UL << 1))

#define NR_IRQS                         384

//IRQ registers
#define REG_IRQ_MASK_L                  (REG_INT_BASE + (0x0034 << 2))
#define REG_IRQ_MASK_H                  (REG_INT_BASE + (0x0035 << 2))
#define REG_IRQ_PENDING_L               (REG_INT_BASE + (0x003c << 2))
#define REG_IRQ_PENDING_H               (REG_INT_BASE + (0x003d << 2))

//IRQ EXP registers
#define REG_IRQ_EXP_MASK_L              (REG_INT_BASE + (0x0036 << 2))
#define REG_IRQ_EXP_MASK_H              (REG_INT_BASE + (0x0037 << 2))
#define REG_IRQ_EXP_PENDING_L           (REG_INT_BASE + (0x003e << 2))
#define REG_IRQ_EXP_PENDING_H           (REG_INT_BASE + (0x003f << 2))

//IRQ HYP registers
#define REG_IRQ_HYP_MASK_L              (REG_INT_HYP_BASE + (0x0034 << 2))
#define REG_IRQ_HYP_MASK_H              (REG_INT_HYP_BASE + (0x0035 << 2))
#define REG_IRQ_HYP_PENDING_L           (REG_INT_HYP_BASE + (0x003c << 2))
#define REG_IRQ_HYP_PENDING_H           (REG_INT_HYP_BASE + (0x003d << 2))

//IRQ SUP registers
#define REG_IRQ_SUP_MASK_L        	(REG_INT_HYP_BASE + (0x0036 << 2))
#define REG_IRQ_SUP_MASK_H          	(REG_INT_HYP_BASE + (0x0037 << 2))
#define REG_IRQ_SUP_PENDING_L       	(REG_INT_HYP_BASE + (0x003e << 2))
#define REG_IRQ_SUP_PENDING_H       	(REG_INT_HYP_BASE + (0x003f << 2))

#if defined(CONFIG_MP_PLATFORM_MSTAR_LEGANCY_INTR)
//FIQ registers
#define REG_FIQ_MASK_L                  (REG_INT_BASE + (0x0024 << 2))
#define REG_FIQ_MASK_H                  (REG_INT_BASE + (0x0025 << 2))
#define REG_FIQ_CLEAR_L                 (REG_INT_BASE + (0x002c << 2))
#define REG_FIQ_CLEAR_H                 (REG_INT_BASE + (0x002d << 2))
#define REG_FIQ_PENDING_L               (REG_INT_BASE + (0x002c << 2))
#define REG_FIQ_PENDING_H               (REG_INT_BASE + (0x002d << 2))

//FIQ EXP registers
#define REG_FIQ_EXP_MASK_L              (REG_INT_BASE + (0x0026 << 2))
#define REG_FIQ_EXP_MASK_H              (REG_INT_BASE + (0x0027 << 2))
#define REG_FIQ_EXP_CLEAR_L             (REG_INT_BASE + (0x002e << 2))
#define REG_FIQ_EXP_CLEAR_H             (REG_INT_BASE + (0x002f << 2))
#define REG_FIQ_EXP_PENDING_L           (REG_INT_BASE + (0x002e << 2))
#define REG_FIQ_EXP_PENDING_H           (REG_INT_BASE + (0x002f << 2))

//FIQ HYP register
#define REG_FIQ_HYP_MASK_L              (REG_INT_HYP_BASE + (0x0024 << 2))
#define REG_FIQ_HYP_MASK_H              (REG_INT_HYP_BASE + (0x0025 << 2))
#define REG_FIQ_HYP_CLEAR_L             (REG_INT_HYP_BASE + (0x002c << 2))
#define REG_FIQ_HYP_CLEAR_H             (REG_INT_HYP_BASE + (0x002d << 2))
#define REG_FIQ_HYP_PENDING_L           (REG_INT_HYP_BASE + (0x002c << 2))
#define REG_FIQ_HYP_PENDING_H           (REG_INT_HYP_BASE + (0x002d << 2))

//FIQ SUP registers
#define REG_FIQ_SUP_MASK_L              (REG_INT_HYP_BASE + (0x0026 << 2))
#define REG_FIQ_SUP_MASK_H              (REG_INT_HYP_BASE + (0x0027 << 2))
#define REG_FIQ_SUP_CLEAR_L             (REG_INT_HYP_BASE + (0x002e << 2))
#define REG_FIQ_SUP_CLEAR_H             (REG_INT_HYP_BASE + (0x002f << 2))
#define REG_FIQ_SUP_PENDING_L           (REG_INT_HYP_BASE + (0x002e << 2))
#define REG_FIQ_SUP_PENDING_H           (REG_INT_HYP_BASE + (0x002f << 2))
#else

//FIQ registers
#define REG_FIQ_MASK_L                  (REG_INT_BASE + (0x0004 << 2))
#define REG_FIQ_MASK_H                  (REG_INT_BASE + (0x0005 << 2))
#define REG_FIQ_CLEAR_L                 (REG_INT_BASE + (0x000c << 2))
#define REG_FIQ_CLEAR_H                 (REG_INT_BASE + (0x000d << 2))
#define REG_FIQ_PENDING_L               (REG_INT_BASE + (0x000c << 2))
#define REG_FIQ_PENDING_H               (REG_INT_BASE + (0x000d << 2))

//FIQ EXP registers
#define REG_FIQ_EXP_MASK_L              (REG_INT_BASE + (0x0006 << 2))
#define REG_FIQ_EXP_MASK_H              (REG_INT_BASE + (0x0007 << 2))
#define REG_FIQ_EXP_CLEAR_L             (REG_INT_BASE + (0x000e << 2))
#define REG_FIQ_EXP_CLEAR_H             (REG_INT_BASE + (0x000f << 2))
#define REG_FIQ_EXP_PENDING_L           (REG_INT_BASE + (0x000e << 2))
#define REG_FIQ_EXP_PENDING_H           (REG_INT_BASE + (0x000f << 2))

#define REG_FIQ_HYP_MASK_L              (REG_INT_HYP_BASE + (0x0004 << 2))
#define REG_FIQ_HYP_MASK_H              (REG_INT_HYP_BASE + (0x0005 << 2))
#define REG_FIQ_HYP_CLEAR_L             (REG_INT_HYP_BASE + (0x000c << 2))
#define REG_FIQ_HYP_CLEAR_H             (REG_INT_HYP_BASE + (0x000d << 2))
#define REG_FIQ_HYP_PENDING_L           (REG_INT_HYP_BASE + (0x000c << 2))
#define REG_FIQ_HYP_PENDING_H           (REG_INT_HYP_BASE + (0x000d << 2))

#endif /* CONFIG_MP_PLATFORM_MSTAR_LEGANCY_INTR */

#ifdef CONFIG_MP_PLATFORM_INT_1_to_1_SPI
#define MSTAR_IRQ_BASE                  32
#define MSTAR_FIQ_BASE                  96
#define MSTAR_IRQ_HYP_BASE              198 //166+32
#define MSTAR_FIQ_HYP_BASE              227 //195+32
#define MSTAR_INT_BASE                  MSTAR_IRQ_BASE
#define MSTAR_CHIP_INT_END              256
#else
#define MSTAR_IRQ_BASE                  192
#define MSTAR_FIQ_BASE                  128
#define MSTAR_IRQ_HYP_BASE              320
#define MSTAR_FIQ_HYP_BASE              256
#define MSTAR_INT_BASE                  MSTAR_FIQ_BASE
#define MSTAR_CHIP_INT_END              384
#endif


/*******************************************************/
/*   THE IRQ AND FIQ ARE NOT COMPLETED.                */
/*   FOR EACH IP OWNER, PLEASE REVIEW IT BY YOURSELF   */
/*******************************************************/
typedef enum
{
    // IRQ
    E_IRQL_START                        = MSTAR_IRQ_BASE,
    E_IRQ_UART0                         = E_IRQL_START + 0,     //IRQ0
    E_IRQ_PM_SLEEP                      = E_IRQL_START + 1,     //IRQ1
    E_IRQ_EVD_R2                        = E_IRQL_START + 2,     //IRQ2
    E_IRQ_MVD                           = E_IRQL_START + 3,     //IRQ3
    E_IRQ_PS                            = E_IRQL_START + 4,     //IRQ4
    E_IRQ_NFIE                          = E_IRQL_START + 5,     //IRQ5
    E_IRQ_USB                           = E_IRQL_START + 6,     //IRQ6
    E_IRQ_UHC                           = E_IRQL_START + 7,     //IRQ7
    E_IRQ_IPLL                          = E_IRQL_START + 8,     //IRQ8
    E_IRQ_EMAC                          = E_IRQL_START + 9,     //IRQ9
    E_IRQ_DISP                          = E_IRQL_START + 10,    //IRQ10
    E_IRQ_MSPI                          = E_IRQL_START + 11,    //IRQ11
    E_IRQ_GE                            = E_IRQL_START + 12,    //IRQ12
    E_IRQ_EVD                           = E_IRQL_START + 13,    //IRQ13
    E_IRQ_COMB                          = E_IRQL_START + 14,    //IRQ14
    E_IRQ_MOD                           = E_IRQL_START + 15,    //IRQ15
    E_IRQL_END                          = 15 + E_IRQL_START,

    E_IRQH_START                        = 16 + E_IRQL_START,
    E_IRQ_TSP2HK                        = E_IRQH_START + 0,     //IRQ16
    E_IRQ_CEC                           = E_IRQH_START + 1,     //IRQ17
    E_IRQ_DISP_FE                       = E_IRQH_START + 2,     //IRQ18
    E_IRQ_DC                            = E_IRQH_START + 3,     //IRQ19
    E_IRQ_GOP                           = E_IRQH_START + 4,     //IRQ20
    E_IRQ_SCDC                          = E_IRQH_START + 5,     //IRQ21
    E_IRQ_OTG                           = E_IRQH_START + 6,     //IRQ22
    E_IRQ_D2B                           = E_IRQH_START + 7,     //IRQ23
    E_IRQ_AUDMA                         = E_IRQH_START + 8,     //IRQ24
//  E_IRQ_RESERVED                      = E_IRQH_START + 9,     //IRQ25
//  E_IRQ_EMMC_OSP                      = E_IRQH_START + 10,    //IRQ26
    E_IRQ_SCM                           = E_IRQH_START + 11,    //IRQ27
    E_IRQ_VBI                           = E_IRQH_START + 12,    //IRQ28
    E_IRQ_MVD2ARM                       = E_IRQH_START + 13,    //IRQ29
    E_IRQ_GPD                           = E_IRQH_START + 14,    //IRQ30
    E_IRQ_TSO                           = E_IRQH_START + 15,    //IRQ31
    E_IRQH_END                          = 15 + E_IRQH_START,

    //IRQEXP
    E_IRQEXPL_START                     = 16 + E_IRQH_START,
    E_IRQEXPL_HVD                       = E_IRQEXPL_START + 0,  //IRQ32
    E_IRQEXPL_USB1                      = E_IRQEXPL_START + 1,  //IRQ33
    E_IRQEXPL_MIU                       = E_IRQEXPL_START + 2,  //IRQ34
    E_IRQEXPL_ERROR                     = E_IRQEXPL_START + 3,  //IRQ35
    E_IRQEXPL_UHC2P2                    = E_IRQEXPL_START + 4,  //IRQ36
    E_IRQEXPL_RTC2                      = E_IRQEXPL_START + 5,  //IRQ37
    E_IRQEXPL_AESDMA                    = E_IRQEXPL_START + 6,  //IRQ38
    E_IRQEXPL_UART1                     = E_IRQEXPL_START + 7,  //IRQ39
    E_IRQEXPL_DISP_MFDEC                = E_IRQEXPL_START + 8,  //IRQ40
    E_IRQEXPL_MSPI_CILINK               = E_IRQEXPL_START + 9,  //IRQ41
    E_IRQEXPL_MIU_SECURITY              = E_IRQEXPL_START + 10, //IRQ42
    E_IRQEXPL_DIPW                      = E_IRQEXPL_START + 11, //IRQ43
    E_IRQEXPL_MIIC2                     = E_IRQEXPL_START + 12, //IRQ44
    E_IRQEXPL_JPD                       = E_IRQEXPL_START + 13, //IRQ45
    E_IRQEXPL_PM_IRQ                    = E_IRQEXPL_START + 14, //IRQ46
    E_IRQEXPL_PKA_ALL                   = E_IRQEXPL_START + 15, //IRQ47
    E_IRQEXPL_END                       = 15 + E_IRQEXPL_START,

    E_IRQEXPH_START                     = 16 + E_IRQEXPL_START,
    E_IRQEXPH_BDMA_MERGE                = E_IRQEXPH_START + 0,  //IRQ48
    E_IRQEXPH_PTS_PARSER                = E_IRQEXPH_START + 1,  //IRQ49
    E_IRQEXPH_UART2MCU                  = E_IRQEXPH_START + 2,  //IRQ50
    E_IRQEXPH_URDMA2MCU                 = E_IRQEXPH_START + 3,  //IRQ51
    E_IRQEXPH_DVI_HDMI_HDCP             = E_IRQEXPH_START + 4,  //IRQ52
    E_IRQEXPH_G3D2MCU                   = E_IRQEXPH_START + 5,  //IRQ53
    E_IRQEXPH_FCIE_TEE                  = E_IRQEXPH_START + 6,  //IRQ54
    E_IRQEXPH_SMART                     = E_IRQEXPH_START + 7,  //IRQ55
    E_IRQEXPH_HDCP_X74                  = E_IRQEXPH_START + 8,  //IRQ56
    E_IRQEXPH_WADR_ERR_INT              = E_IRQEXPH_START + 9,  //IRQ57
    E_IRQEXPH_VE                        = E_IRQEXPH_START + 10, //IRQ58
    E_IRQEXPH_SDIO                      = E_IRQEXPH_START + 11, //IRQ59
    E_IRQEXPH_ACPU                      = E_IRQEXPH_START + 12, //IRQ60
    E_IRQEXPH_MIIC1                     = E_IRQEXPH_START + 13, //IRQ61
    E_IRQEXPH_USB2P3                    = E_IRQEXPH_START + 14, //IRQ62
    E_IRQEXPH_MIIC0                     = E_IRQEXPH_START + 15, //IRQ63
    E_IRQEXPH_END                       = 15 + E_IRQEXPH_START,

    // IRQHYPL
    E_IRQHYPL_START                     = MSTAR_IRQ_HYP_BASE, //SPI192,PPI320
    E_IRQHYPL_MIIC3                     = E_IRQHYPL_START + 0, //IRQ64
    E_IRQHYPL_USB_INT1                  = E_IRQHYPL_START + 1, //IRQ65
    E_IRQHYPL_UHC_INT1                  = E_IRQHYPL_START + 2, //IRQ66
    E_IRQHYPL_UART2                     = E_IRQHYPL_START + 3, //IRQ67
    E_IRQHYPL_BLKPREARB_AUTO_BIST_INT   = E_IRQHYPL_START + 4, //IRQ68
    E_IRQHYPL_UART4                     = E_IRQHYPL_START + 5, //IRQ69
    E_IRQHYPL_MFE                       = E_IRQHYPL_START + 6, //IRQ70
    E_IRQHYPL_CMDQ                      = E_IRQHYPL_START + 7, //IRQ71
    E_IRQHYPL_PCM2MCU                   = E_IRQHYPL_START + 8, //IRQ72
    E_IRQHYPL_SC_HK_INT                 = E_IRQHYPL_START + 9, //IRQ73
    E_IRQHYPL_EARC_TX                   = E_IRQHYPL_START + 10,//IRQ74
    E_IRQHYPL_CI_GPIO                   = E_IRQHYPL_START + 11,//IRQ75
    E_IRQHYPL_CILINK_SOC_IF             = E_IRQHYPL_START + 12,//IRQ76
    E_IRQHYPL_TSP_FI                    = E_IRQHYPL_START + 13,//IRQ77
    E_IRQHYPL_TSEN_0_OV                 = E_IRQHYPL_START + 14,//IRQ78
    E_IRQHYPL_TSEN_1_OV                 = E_IRQHYPL_START + 15,//IRQ79
    E_IRQHYPL_END                       = 15 + E_IRQHYPL_START,

    E_IRQHYPH_START                     = 16 + E_IRQHYPL_START,
    E_IRQEXPH_FRC_FIQTOARM              = E_IRQHYPH_START + 0, //IRQ80
    E_IRQHYPH_FSP_DEMURA_INT            = E_IRQHYPH_START + 1, //IRQ81
    E_IRQHYPH_LZMA                      = E_IRQHYPH_START + 2, //IRQ82
    //E_IRQHYPH_RESERVED                  = E_IRQHYPH_START + 3, //IRQ83
    E_IRQHYPH_ADCPLL_INT                = E_IRQHYPH_START + 4, //IRQ84
    E_IRQHYPH_DC_SUB_INT                = E_IRQHYPH_START + 5, //IRQ85
    E_IRQHYPH_AIA_MDLA_NS_INT           = E_IRQHYPH_START + 6, //IRQ86
    E_IRQHYPH_AIA_CONN_INT              = E_IRQHYPH_START + 7, //IRQ87
    E_IRQHYPH_AIA_DMAC_INT              = E_IRQHYPH_START + 8, //IRQ88
    E_IRQHYPH_AIA_SEC_INT               = E_IRQHYPH_START + 9, //IRQ89
    E_IRQHYPH_AIA_MDLA_S_INT            = E_IRQHYPH_START + 10,//IRQ90
    E_IRQHYPH_DIP2_INT                  = E_IRQHYPH_START + 11,//IRQ91
    E_IRQHYPH_DIP2_SEC_INT              = E_IRQHYPH_START + 12,//IRQ92
    E_IRQHYPH_END                       = 15 + E_IRQHYPH_START,

    E_IRQSUPL_START                     = 16 + E_IRQHYPH_START,
    E_IRQSUPH_START                     = 16 + E_IRQSUPL_START,

    // FIQ
    E_FIQL_START                        = MSTAR_FIQ_BASE,
    E_FIQ_EXTIMER0                      = E_FIQL_START + 0,     //FIQ0
    E_FIQ_EXTIMER1                      = E_FIQL_START + 1,     //FIQ1
    E_FIQ_WDT                           = E_FIQL_START + 2,     //FIQ2
    E_FIQ_USB2P3                        = E_FIQL_START + 3,     //FIQ3
    E_FIQ_MB_AU_SPDIF_TX_CS0            = E_FIQL_START + 4,     //FIQ4
    E_FIQ_MB_AU_SPDIF_TX_CS1            = E_FIQL_START + 5,     //FIQ5
    E_FIQ_MB_DSP2TOMCU0                 = E_FIQL_START + 6,     //FIQ6
    E_FIQ_MB_DSP2TOMCU1                 = E_FIQL_START + 7,     //FIQ7
    E_FIQ_USB                           = E_FIQL_START + 8,     //FIQ8
    E_FIQ_UHC                           = E_FIQL_START + 9,     //FIQ9
    E_FIQ_VE_DONE_TT_IRQ                = E_FIQL_START + 10,    //FIQ10
    E_FIQ_HDMI_NON_PCM                  = E_FIQL_START + 11,    //FIQ11
    E_FIQ_SPDIF_IN_NON_PCM              = E_FIQL_START + 12,    //FIQ12
    E_FIQ_LAN_ESD                       = E_FIQL_START + 13,    //FIQ13
    E_FIQ_SE_DSP2UP                     = E_FIQL_START + 14,    //FIQ14
    E_FIQ_TSP2AEON                      = E_FIQL_START + 15,    //FIQ15
    E_FIQL_END                          = 15 + E_FIQL_START,

    E_FIQH_START                        = 16 + E_FIQL_START,
    E_FIQ_VIVALDI_STR                   = E_FIQH_START + 0,     //FIQ16
    E_FIQ_VIVALDI_PTS                   = E_FIQH_START + 1,     //FIQ17
    E_FIQ_DSP_MIU_PROT                  = E_FIQH_START + 2,     //FIQ18
    E_FIQ_XIU_TIMEOUT                   = E_FIQH_START + 3,     //FIQ19
    E_FIQ_DMDCU2HK_INT                  = E_FIQH_START + 4,     //FIQ20
    E_FIQ_IR                            = E_FIQH_START + 5,     //FIQ21
    E_FIQ_IMI_TOP                       = E_FIQH_START + 6,     //FIQ22
    E_FIQ_VDMCU2HK                      = E_FIQH_START + 7,     //FIQ23
    E_FIQ_LDM_DMA_INT0                  = E_FIQH_START + 8,     //FIQ24
    E_FIQ_LDM_DMA_INT1                  = E_FIQH_START + 9,     //FIQ25
    E_FIQ_PM_SD                         = E_FIQH_START + 10,    //FIQ26
    E_FIQ_MB_AUR2TOMCU0                 = E_FIQH_START + 11,    //FIQ27
    E_FIQ_AFEC_VSYNC                    = E_FIQH_START + 12,    //FIQ28
    E_FIQ_MB_AUR2TOMCU1                 = E_FIQH_START + 13,    //FIQ29
    E_FIQ_MB_AUR2TOMCU2                 = E_FIQH_START + 14,    //FIQ30
    E_FIQ_DSP2ARM                       = E_FIQH_START + 15,    //FIQ31
    E_FIQH_END                          = 15 + E_FIQH_START,

    // FIQEXP
    E_FIQEXPL_START                     = 16 + E_FIQH_START,
    E_FIQEXPL_MB_AUR2TOMCU3             = E_FIQEXPL_START + 0,  //FIQ32
//  E_FIQEXPL_AU_DMA_BUF_INT            = E_FIQEXPL_START + 1,  //FIQ33
    E_FIQEXPL_IR_IN                     = E_FIQEXPL_START + 2,  //FIQ34
//  E_FIQEXPL_PM_SD_CDZ                 = E_FIQEXPL_START + 3,  //FIQ35
    E_FIQEXPL_8051_TO_SECURE_R2         = E_FIQEXPL_START + 4,  //FIQ36
    E_FIQEXPL_8051_TO_ARM_C1            = E_FIQEXPL_START + 5,  //FIQ37
    E_FIQEXPL_8051_TO_ARM               = E_FIQEXPL_START + 6,  //FIQ38
    E_FIQEXPL_EXT_GPIO0                 = E_FIQEXPL_START + 7,  //FIQ39
    E_FIQEXPL_ARM_TO_SECURE_R2          = E_FIQEXPL_START + 8,  //FIQ40
    E_FIQEXPL_EMAC_MAN                  = E_FIQEXPL_START + 9,  //FIQ41
    E_FIQEXPL_ARM_TO_8051               = E_FIQEXPL_START + 10, //FIQ42
    E_FIQEXPL_EXT_GPIO1                 = E_FIQEXPL_START + 11, //FIQ43
    E_FIQEXPL_ARM_C1_TO_SECU            = E_FIQEXPL_START + 12, //FIQ44
    E_FIQEXPL_TIMER2                    = E_FIQEXPL_START + 13, //FIQ45
    E_FIQEXPL_ARM_C1_TO_8051            = E_FIQEXPL_START + 14, //FIQ46
    E_FIQEXPL_EXT_GPIO2                 = E_FIQEXPL_START + 15, //FIQ47
    E_FIQEXPL_END                       = 15 + E_FIQEXPL_START,

    E_FIQEXPH_START                     = 16 + E_FIQEXPL_START,
    E_FIQEXPL_SECURE_R2_TO_ARM_C1       = E_FIQEXPH_START + 0,  //FIQ48
    E_FIQEXPL_SECURE_R2_TO_ARM          = E_FIQEXPH_START + 1,  //FIQ49
    E_FIQEXPL_SECURE_R2_TO_8051         = E_FIQEXPH_START + 2,  //FIQ50
    E_FIQEXPH_USB2P1                    = E_FIQEXPH_START + 3,  //FIQ51
    E_FIQEXPH_VE_VBI_F0                 = E_FIQEXPH_START + 4,  //FIQ52
    E_FIQEXPH_USB2P2                    = E_FIQEXPH_START + 5,  //FIQ53
    E_FIQEXPH_VE_VBI_F1                 = E_FIQEXPH_START + 6,  //FIQ54
    E_FIQEXPH_EXT_GPIO3                 = E_FIQEXPH_START + 7,  //FIQ55
    E_FIQEXPH_EXT_GPIO4                 = E_FIQEXPH_START + 8,  //FIQ56
    E_FIQEXPH_EXT_GPIO5                 = E_FIQEXPH_START + 9,  //FIQ57
    E_FIQEXPH_EXT_GPIO6                 = E_FIQEXPH_START + 10, //FIQ58
    E_FIQEXPH_PWM_RP_I                  = E_FIQEXPH_START + 11, //FIQ59
    E_FIQEXPH_PWM_FP_I                  = E_FIQEXPH_START + 12, //FIQ60
    E_FIQEXPH_PWM_RP_R                  = E_FIQEXPH_START + 13, //FIQ61
    E_FIQEXPH_PWM_FP_R                  = E_FIQEXPH_START + 14, //FIQ62
    E_FIQEXPH_GPIOINIT7                 = E_FIQEXPH_START + 15, //FIQ63
    E_FIQEXPH_END                       = 15 + E_FIQEXPL_START,

    E_FIQHYPL_START                     = MSTAR_FIQ_HYP_BASE,
    E_FIQHYPH_START                     = 16 + E_FIQHYPL_START,
    E_FIQSUPL_START                     = 16 + E_FIQHYPH_START,
    E_FIQSUPH_START                     = 16 + E_FIQSUPL_START,

    E_IRQ_FIQ_ALL                       = 0xFF //all IRQs & FIQs
} InterruptNum;

//array index is SPI number
extern unsigned int spi_to_ppi[NR_IRQS];

// #define REG_IRQ_PENDING_L
#define IRQL_MASK                       0xFFFF
#define IRQ_UART0                       (0x01 << (E_IRQ_UART0               - E_IRQL_START))
#define IRQ_PM_SLEEP                    (0x01 << (E_IRQ_PM_SLEEP            - E_IRQL_START))
#define IRQ_EVD_R2                      (0x01 << (E_IRQ_EVD_R2              - E_IRQL_START))
#define IRQ_MVD                         (0x01 << (E_IRQ_MVD                 - E_IRQL_START))
#define IRQ_PS                          (0x01 << (E_IRQ_PS                  - E_IRQL_START))
#define IRQ_NFIE                        (0x01 << (E_IRQ_NFIE                - E_IRQL_START))
#define IRQ_USB                         (0x01 << (E_IRQ_USB                 - E_IRQL_START))
#define IRQ_UHC                         (0x01 << (E_IRQ_UHC                 - E_IRQL_START))
#define IRQ_MHL_CBUS                    (0x01 << (E_IRQ_MHL_CBUS            - E_IRQL_START))
#define IRQ_EMAC                        (0x01 << (E_IRQ_EMAC                - E_IRQL_START))
#define IRQ_DISP                        (0x01 << (E_IRQ_DISP                - E_IRQL_START))
#define IRQ_GE                          (0x01 << (E_IRQ_GE                  - E_IRQL_START))
#define IRQ_EVD                         (0x01 << (E_IRQ_EVD                 - E_IRQL_START))
#define IRQ_COMB                        (0x01 << (E_IRQ_COMB                - E_IRQL_START))
//#define IRQ_LDM_DMA                     (0x01 << (E_IRQ_LDM_DMA             - E_IRQL_START))
#define IRQ_EXT_GPIO0                   (0x01 << (E_IRQ_EXT_GPIO0           - E_IRQL_START))


// #define REG_IRQ_PENDING_H
#define IRQH_MASK                       0xFFFF
#define IRQ_TSP2HK                      (0x01 << (E_IRQ_TSP2HK              - E_IRQH_START))
#define IRQ_CEC                         (0x01 << (E_IRQ_CEC                 - E_IRQH_START))
#define IRQ_DISP_FE                     (0x01 << (E_IRQ_DISP_FE             - E_IRQH_START))
#define IRQ_DC                          (0x01 << (E_IRQ_DC                  - E_IRQH_START))
#define IRQ_GOP                         (0x01 << (E_IRQ_GOP                 - E_IRQH_START))
#define IRQ_SCDC                        (0x01 << (E_IRQ_SCDC                - E_IRQH_START))
//#define IRQ_LDM_DMA1                  (0x01 << (E_IRQ_LDM_DMA1            - E_IRQH_START))
#define IRQ_D2B                         (0x01 << (IRQ_D2B                   - E_IRQH_START))
#define IRQ_AUDMA                       (0x01 << (E_IRQ_AUDMA               - E_IRQH_START))
#define IRQ_UHC2P3                      (0x01 << (E_IRQ_UHC2P3              - E_IRQH_START))
#define IRQ_EMMC_OSP                    (0x01 << (E_IRQ_EMMC_OSP            - E_IRQH_START))
#define IRQ_SCM                         (0x01 << (E_IRQ_SCM                 - E_IRQH_START))
#define IRQ_VBI                         (0x01 << (E_VBI                     - E_IRQH_START))
#define IRQ_MVD2ARM                     (0x01 << (E_IRQ_MVD2ARM             - E_IRQH_START))
#define IRQ_GPD                         (0x01 << (E_IRQ_GPD                 - E_IRQH_START))
#define IRQ_ADCDVI2RIU                  (0x01 << (E_IRQ_ADCDVI2RIU          - E_IRQH_START))

// #define REG_IRQEXP_PENDING_L
#define IRQEXPL_MASK                    0xFFFF
#define IRQEXPL_HVD                     (0x01 << (E_IRQEXPL_HVD             - E_IRQEXPL_START))
#define IRQEXPL_USB1                    (0x01 << (E_IRQEXPL_USB1            - E_IRQEXPL_START))
#define IRQEXPL_UHC1                    (0x01 << (E_IRQEXPL_UHC1            - E_IRQEXPL_START))
#define IRQEXPL_ERROR                   (0x01 << (E_IRQEXPL_ERROR           - E_IRQEXPL_START))
#define IRQEXPL_USB2P2                  (0x01 << (E_IRQEXPL_USB2P2          - E_IRQEXPL_START))
#define IRQEXPL_UHC2P2                  (0x01 << (E_IRQEXPL_UHC2P2          - E_IRQEXPL_START))
#define IRQEXPL_MHL_ECBUS               (0x01 << (E_IRQEXPL_MHL_ECBUS       - E_IRQEXPL_START))
#define IRQEXPL_UART1                   (0x01 << (E_IRQEXPL_UART1           - E_IRQEXPL_START))
#define IRQEXPL_UART2                   (0x01 << (E_IRQEXPL_UART2           - E_IRQEXPL_START))
#define IRQEXPL_MSPI1                   (0x01 << (E_IRQEXPL_MSPI1           - E_IRQEXPL_START))
#define IRQEXPL_MIU_SECURITY            (0x01 << (E_IRQEXPL_MIU_SECURITY    - E_IRQEXPL_START))
#define IRQEXPL_DIPW                    (0x01 << (E_IRQEXPL_DIPW            - E_IRQEXPL_START))
#define IRQEXPL_MIIC2                   (0x01 << (E_IRQEXPL_MIIC2           - E_IRQEXPL_START))
#define IRQEXPL_JPD                     (0x01 << (E_IRQEXPL_JPD             - E_IRQEXPL_START))
#define IRQEXPL_PM_IRQ                  (0x01 << (E_IRQEXPL_PM_IRQ          - E_IRQEXPL_START))
#define IRQEXPL_MFE                     (0x01 << (E_IRQEXPL_MFE             - E_IRQEXPL_START))

// #define REG_IRQEXP_PENDING_H
#define IRQEXPH_MASK                    0xFFFF
#define IRQEXPH_BDMA_MERGE              (0x01 << (E_IRQEXPH_BDMA_MERGE      - E_IRQEXPH_START))
#define IRQEXPH_PTS_PARSER              (0x01 << (E_IRQEXPH_PTS_PARSER      - E_IRQEXPH_START))
#define IRQEXPH_UART2MCU                (0x01 << (E_IRQEXPH_UART2MCU        - E_IRQEXPH_START))
#define IRQEXPH_URDMA2MCU               (0x01 << (E_IRQEXPH_URDMA2MCU       - E_IRQEXPH_START))
#define IRQEXPH_DVI_HDMI_HDCP           (0x01 << (E_IRQEXPH_DVI_HDMI_HDCP   - E_IRQEXPH_START))
#define IRQEXPH_G3D2MCU                 (0x01 << (E_IRQEXPH_G3D2MCU         - E_IRQEXPH_START))
//#define IRQEXPH_DS                    (0x01 << (E_IRQEXPH_DS              - E_IRQEXPH_START))
//#define IRQEXPH_RESERVED              (0x01 << (E_IRQEXPH_RESERVED        - E_IRQEXPH_START))
#define IRQEXPH_HDCP_X74                (0x01 << (E_IRQEXPH_HDCP_X74        - E_IRQEXPH_START))
#define IRQEXPH_WADR_ERR_INT            (0x01 << (E_IRQEXPH_WADR_ERR_INT    - E_IRQEXPH_START))
//#define IRQEXPH_DCSUB                 (0x01 << (E_IRQEXPH_DCSUB           - E_IRQEXPH_START))
#define IRQEXPH_SDIO                    (0x01 << (E_IRQEXPH_SDIO            - E_IRQEXPH_START))
//#define IRQEXPH_CMDQ                  (0x01 << (E_IRQEXPH_CMDQ            - E_IRQEXPH_START))
#define IRQEXPH_MIIC1                   (0x01 << (E_IRQEXPH_MIIC1           - E_IRQEXPH_START))
#define IRQEXPH_USB2P3                  (0x01 << (E_IRQEXPH_USB2P3          - E_IRQEXPH_START))
#define IRQEXPH_MIIC0                   (0x01 << (E_IRQEXPH_MIIC0           - E_IRQEXPH_START))

// REG_FIQ_MASK_L
//FIQ Low 16 bits
#define FIQL_MASK                       0xFFFF
#define FIQ_EXTIMER0                    (0x01 << (E_FIQ_EXTIMER0            - E_FIQL_START))
#define FIQ_EXTIMER1                    (0x01 << (E_FIQ_EXTIMER1            - E_FIQL_START))
#define FIQ_WDT                         (0x01 << (E_FIQ_WDT                 - E_FIQL_START))
#define FIQ_MB_AU_SPDIF_TX_CS0          (0x01 << (E_FIQ_MB_AU_SPDIF_TX_CS0  - E_FIQL_START))
#define FIQ_MB_AU_SPDIF_TX_CS1          (0x01 << (E_FIQ_MB_AU_SPDIF_TX_CS1  - E_FIQL_START))
#define FIQ_MB_DSP2TOMCU0               (0x01 << (E_FIQ_MB_DSP2TOMCU0       - E_FIQL_START))
#define FIQ_MB_DSP2TOMCU1               (0x01 << (E_FIQ_MB_DSP2TOMCU1       - E_FIQL_START))
#define FIQ_USB                         (0x01 << (E_FIQ_USB                 - E_FIQL_START))
#define FIQ_UHC                         (0x01 << (E_FIQ_UHC                 - E_FIQL_START))
#define FIQ_UHC2P3                      (0x01 << (E_FIQ_UHC2P3              - E_FIQL_START))
#define FIQ_HDMI_NON_PCM                (0x01 << (E_FIQ_HDMI_NON_PCM        - E_FIQL_START))
#define FIQ_SPDIF_IN_NON_PCM            (0x01 << (E_FIQ_SPDIF_IN_NON_PCM    - E_FIQL_START))
#define FIQ_LAN_ESD                     (0x01 << (E_FIQ_LAN_ESD             - E_FIQL_START))
#define FIQ_SE_DSP2UP                   (0x01 << (E_FIQ_SE_DSP2UP           - E_FIQL_START))
#define FIQ_TSP2AEON                    (0x01 << (E_FIQ_TSP2AEON            - E_FIQL_START))

// REG_FIQ_MASK_H
//FIQ High 16 bits
#define FIQH_MASK                       0xFFFF
#define FIQ_VIVALDI_STR                 (0x01 << (E_FIQ_VIVALDI_STR         - E_FIQH_START))
#define FIQ_VIVALDI_PTS                 (0x01 << (E_FIQ_VIVALDI_PTS         - E_FIQH_START))
#define FIQ_DSP_MIU_PROT                (0x01 << (E_FIQ_DSP_MIU_PROT        - E_FIQH_START))
#define FIQ_XIU_TIMEOUT                 (0x01 << (E_FIQ_XIU_TIMEOUT         - E_FIQH_START))
#define FIQ_DMDMCU2HK_INT               (0x01 << (E_FIQ_DMDMCU2HK_INT       - E_FIQH_START))
#define FIQ_IR                          (0x01 << (E_FIQ_IR                  - E_FIQH_START))
#define FIQ_VSYNC_VE4VBI                (0x01 << (E_FIQ_VSYNC_VE4VBI        - E_FIQH_START))
#define FIQ_VDMCU2HK                    (0x01 << (E_FIQ_VDMCU2HK            - E_FIQH_START))
#define FIQ_LDM_DMA_INT0                (0x01 << (E_FIQ_LDM_DMA_INT0        - E_FIQH_START))
#define FIQ_LDM_DMA_INT1                (0x01 << (E_FIQ_LDM_DMA_INT1        - E_FIQH_START))
#define FIQ_PM_SD                       (0x01 << (E_FIQ_PM_SD               - E_FIQH_START))
//#define FIQ_TEMPERATURE_FAIL          (0x01 << (E_FIQ_TEMPERATURE_FAIL    - E_FIQH_START))
#define FIQ_AFEC_VSYNC                  (0x01 << (E_FIQ_AFEC_VSYNC          - E_FIQH_START))
//#define FIQ_DEC_DSP2UP                (0x01 << (E_FIQ_DEC_DSP2UP          - E_FIQH_START))
//#define FIQ_TSO                       (0x01 << (E_FIQ_USB2                - E_FIQH_START))
#define FIQ_DSP2ARM                     (0x01 << (E_FIQ_DSP2ARM             - E_FIQH_START))

// #define REG_FIQEXP_PENDING_L
#define FIQEXPL_MASK                    0xFFFF
//#define E_FIQ_TEMPERATURE_RISE          (0x01 << (E_FIQ_TEMPERATURE_RISE        - E_FIQEXPL_START))
#define FIQEXPL_AU_DMA_BUF_INT          (0x01 << (E_FIQEXPL_AU_DMA_BUF_INT      - E_FIQEXPL_START))
#define FIQEXPL_IR_IN                   (0x01 << (E_FIQEXPL_IR_IN               - E_FIQEXPL_START))
#define FIQEXPL_PM_SD_CDZ               (0x01 << (E_FIQEXPL_PM_SD_CDZ           - E_FIQEXPL_START))
#define FIQEXPL_8051_TO_SECURE_R2       (0x01 << (E_FIQEXPL_8051_TO_SECURE_R2   - E_FIQEXPL_START))
#define FIQEXPL_8051_TO_ARM_C1          (0x01 << (E_FIQEXPL_8051_TO_ARM_C1      - E_FIQEXPL_START))
#define FIQEXPL_8051_TO_ARM             (0x01 << (E_FIQEXPL_8051_TO_ARM         - E_FIQEXPL_START))
#define FIQEXPL_EXT_GPIO0               (0x01 << (E_FIQEXPL_EXT_GPIO0           - E_FIQEXPL_START))
#define FIQEXPL_ARM_TO_SECURE_R2        (0x01 << (E_FIQEXPL_ARM_TO_SECURE_R2    - E_FIQEXPL_START))
#define FIQ_CA9_SCUEVABORT              (0x01 << (E_FIQ_CA9_SCUEVABORT          - E_FIQEXPL_START))
#define FIQEXPL_ARM_TO_8051             (0x01 << (E_FIQEXPL_ARM_TO_8051         - E_FIQEXPL_START))
#define FIQEXPL_EXT_GPIO1               (0x01 << (E_FIQEXPL_EXT_GPIO1           - E_FIQEXPL_START))
#define FIQEXPL_ARM_C1_TO_SECU          (0x01 << (E_FIQEXPL_ARM_C1_TO_SECU      - E_FIQEXPL_START))
#define FIQEXPL_TIMER2                  (0x01 << (E_FIQEXPL_TIMER2              - E_FIQEXPL_START))
#define FIQEXPL_ARM_C1_TO_8051          (0x01 << (E_FIQEXPL_ARM_C1_TO_8051      - E_FIQEXPL_START))
#define FIQEXPL_EXT_GPIO2               (0x01 << (E_FIQEXPL_EXT_GPIO2           - E_FIQEXPL_START))

// #define REG_FIQEXP_PENDING_H
#define FIQEXPL_SECURE_R2_TO_ARM_C1     (0x01 << (E_FIQEXPL_SECURE_R2_TO_ARM_C1 - E_FIQEXPH_START))
//#define FIQEXPL_SECURE_R2_TO_ARM        (0x01 << (E_FIQEXPL_SECURE_R2_TO_ARM    - E_FIQEXPH_START))
#define FIQEXPL_SECURE_R2_TO_8051       (0x01 << (E_FIQEXPL_SECURE_R2_TO_8051   - E_FIQEXPH_START))
#define FIQEXPH_USBP1                   (0x01 << (E_FIQEXPH_USBP1               - E_FIQEXPH_START))
#define FIQEXPH_UHCP1                   (0x01 << (E_FIQEXPH_UHCP1               - E_FIQEXPH_START))
#define FIQEXPH_USBP2                   (0x01 << (E_FIQEXPH_USBP2               - E_FIQEXPH_START))
#define FIQEXPH_UHCP2                   (0x01 << (E_FIQEXPH_UHCP2               - E_FIQEXPH_START))
#define FIQEXPH_EXT_GPIO3               (0x01 << (E_FIQEXPH_EXT_GPIO3           - E_FIQEXPH_START))
#define FIQEXPH_EXT_GPIO4               (0x01 << (E_FIQEXPH_EXT_GPIO4           - E_FIQEXPH_START))
#define FIQEXPH_EXT_GPIO5               (0x01 << (E_FIQEXPH_EXT_GPIO5           - E_FIQEXPH_START))
#define FIQEXPH_EXT_GPIO6               (0x01 << (E_FIQEXPH_EXT_GPIO6           - E_FIQEXPH_START))
#define FIQEXPH_PWM_RP_I                (0x01 << (E_FIQEXPH_PWM_RP_I            - E_FIQEXPH_START))
#define FIQEXPH_PWM_FP_I                (0x01 << (E_FIQEXPH_PWM_FP_I            - E_FIQEXPH_START))
#define FIQEXPH_PWM_RP_R                (0x01 << (E_FIQEXPH_PWM_RP_R            - E_FIQEXPH_START))
#define FIQEXPH_PWM_FP_R                (0x01 << (E_FIQEXPH_PWM_FP_R            - E_FIQEXPH_START))
#define FIQEXPH_GPIOINIT7               (0x01 << (E_FIQEXPH_GPIOINIT7           - E_FIQEXPH_START))

// #define REG_IRQHYP_PENDING_L
#define IRQHYPL_MASK                     0xFFFF
#define IRQHYPL_IRQ_OTG                 (0x01 << (E_IRQHYPL_IRQ_OTG         - E_IRQHYPL_START))
#define IRQHYPL_IRQ_VD_EVD              (0x01 << (E_IRQHYPL_IRQ_VD_EVD      - E_IRQHYPL_START))
#define IRQHYPL_MHL_CBUS                (0x01 << (E_IRQHYPL_MHL_CBUS        - E_IRQHYPL_START))
#define IRQHYPL_GE                      (0x01 << (E_IRQHYPL_GE              - E_IRQHYPL_START))
#define IRQHYPL_CEC                     (0x01 << (E_IRQHYPL_CEC             - E_IRQHYPL_START))
#define IRQHYPL_DISP_FE                 (0x01 << (E_IRQHYPL_DISP_FE         - E_IRQHYPL_START))
#define IRQHYPL_SCDC                    (0x01 << (E_IRQHYPL_SCDC            - E_IRQHYPL_START))
#define IRQHYPL_USB30_HS1_USB           (0x01 << (E_IRQHYPL_USB30_HS1_USB   - E_IRQHYPL_START))
#define IRQHYPL_USB30_HS1_UHC           (0x01 << (E_IRQHYPL_USB30_HS1_UHC   - E_IRQHYPL_START))
#define IRQHYPL_USB30_HS_USB            (0x01 << (E_IRQHYPL_USB30_HS_USB    - E_IRQHYPL_START))
#define IRQHYPL_USB30_HS_UHC            (0x01 << (E_IRQHYPL_USB30_HS_UHC    - E_IRQHYPL_START))
#define IRQHYPL_VP9_HK2VD               (0x01 << (E_IRQHYPL_VP9_HK2VD       - E_IRQHYPL_START))
#define IRQHYPL_ZDEC                    (0x01 << (E_IRQHYPL_ZDEC            - E_IRQHYPL_START))
#define IRQHYPL_TSP_FI                  (0x01 << (E_IRQHYPL_TSP_FI          - E_IRQHYPL_START))
#define IRQHYPL_DISP_SC2                (0x01 << (E_IRQHYPL_DISP_SC2        - E_IRQHYPL_START))
#define IRQEXPL_MSPI_MCARD              (0x01 << (E_IRQHYPL_MSPI_MCARD      - E_IRQHYPL_START))

/* for backward compatibility */
/* #define name_used_in_driver name_defined_here */
#define E_FIQ_VSYNC_VE4VBI      E_FIQEXPH_VE_VBI_F0

#define IRQL_EXP_ALL                    0xFFFF
#define IRQH_EXP_ALL                    0xFFFF
#define FIQL_EXP_ALL                    0xFFFF
#define FIQH_EXP_ALL                    0xFFFF

extern unsigned int interrupt_configs[MSTAR_CHIP_INT_END / 16+1];

void init_chip_spi_config(void);

#endif // #ifndef __CHIP_INT_H__
