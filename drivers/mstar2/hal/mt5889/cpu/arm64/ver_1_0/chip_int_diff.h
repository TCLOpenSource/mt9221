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

#ifndef __CHIP_INT_DIFF_H__
#define __CHIP_INT_DIFF_H__

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
    E_FIQEXPL_REG_HST0TO3               = E_FIQEXPL_START + 4,  //FIQ36
    E_FIQEXPL_REG_HST0TO2               = E_FIQEXPL_START + 5,  //FIQ37
    E_FIQEXPL_REG_HST0TO1               = E_FIQEXPL_START + 6,  //FIQ38
    E_FIQEXPL_EXT_GPIO0                 = E_FIQEXPL_START + 7,  //FIQ39
    E_FIQEXPL_REG_HST1TO3               = E_FIQEXPL_START + 8,  //FIQ40
    E_FIQEXPL_EMAC_MAN                  = E_FIQEXPL_START + 9,  //FIQ41
    E_FIQEXPL_REG_HST1TO0               = E_FIQEXPL_START + 10, //FIQ42
    E_FIQEXPL_EXT_GPIO1                 = E_FIQEXPL_START + 11, //FIQ43
    E_FIQEXPL_REG_HST2TO3               = E_FIQEXPL_START + 12, //FIQ44
    E_FIQEXPL_TIMER2                    = E_FIQEXPL_START + 13, //FIQ45
    E_FIQEXPL_REG_HST2TO0               = E_FIQEXPL_START + 14, //FIQ46
    E_FIQEXPL_EXT_GPIO2                 = E_FIQEXPL_START + 15, //FIQ47
    E_FIQEXPL_END                       = 15 + E_FIQEXPL_START,

    E_FIQEXPH_START                     = 16 + E_FIQEXPL_START,
    E_FIQEXPL_REG_HST3TO2               = E_FIQEXPH_START + 0,  //FIQ48
    E_FIQEXPL_REG_HST3TO1               = E_FIQEXPH_START + 1,  //FIQ49
    E_FIQEXPL_REG_HST3TO0               = E_FIQEXPH_START + 2,  //FIQ50
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

#endif // #ifndef __CHIP_INT_DIFF_H__
