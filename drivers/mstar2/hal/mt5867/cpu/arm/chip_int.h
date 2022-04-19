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
GIC-400
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

//GIC PPI
#define IRQ_LOCALTIMER                  30
#define IRQ_SECURE_LOCALTIMER           29

//PMU(Performance Monitor Unit)
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

#ifndef NR_IRQS
#define NR_IRQS                         384
#endif

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
#define REG_IRQ_SUP_MASK_L              (REG_INT_HYP_BASE + (0x0036 << 2))
#define REG_IRQ_SUP_MASK_H              (REG_INT_HYP_BASE + (0x0037 << 2))
#define REG_IRQ_SUP_PENDING_L           (REG_INT_HYP_BASE + (0x003e << 2))
#define REG_IRQ_SUP_PENDING_H           (REG_INT_HYP_BASE + (0x003f << 2))

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
#define MSTAR_IRQ_HYP_BASE              198
#define MSTAR_FIQ_HYP_BASE              227
#define MSTAR_INT_BASE                  MSTAR_IRQ_BASE
#define MSTAR_CHIP_INT_END              255
#else
#define MSTAR_IRQ_BASE                  192
#define MSTAR_FIQ_BASE                  128
#define MSTAR_IRQ_HYP_BASE              326
#define MSTAR_FIQ_HYP_BASE              259
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
    E_IRQL_START                        = MSTAR_IRQ_BASE,    // nonPM IRQ
    E_IRQ_DCSUB                         = E_IRQL_START + 0,  // 0: dcsub_int
    E_IRQ_PM_SLEEP                      = E_IRQL_START + 1,  // 1: pm_sleep_int
    E_IRQ_AEON                          = E_IRQL_START + 2,  // 2: irq_aeon2hi
    E_IRQ_MVD                           = E_IRQL_START + 3,  // 3: mvd_int
    E_IRQ_PS                            = E_IRQL_START + 4,  // 4: ps_int
    E_IRQ_NFIE                          = E_IRQL_START + 5,  // 5: nfie_int
    E_IRQ_SDIO                          = E_IRQL_START + 6,  // 6: sdio
    //E_IRQ_RESERVED                      = E_IRQL_START + 7,  // 7: reserved
    E_IRQEXPH_SMART                     = E_IRQL_START + 8,  // 8: smart_int
    E_IRQ_EMAC                          = E_IRQL_START + 9,  // 9: emac_int
    E_IRQ_DISP                          = E_IRQL_START + 10, // 10: disp_int, used by "kernel/irq/proc.c"
    E_IRQ_MVD2MIPS                      = E_IRQL_START + 11, // 11: mvd2mips_int
    E_IRQ_HVD                           = E_IRQL_START + 12, // 12: hvd_int
    E_IRQ_EVD                           = E_IRQL_START + 13, // 13: evd_int
    E_IRQ_COMB                          = E_IRQL_START + 14, // 14: comb_int
    E_IRQ_ADCDVI2RIU                    = E_IRQL_START + 15, // 15: adcdvi2riu_int, Not Used
    E_IRQL_END                          = 15 + E_IRQL_START,

    E_IRQH_START                        = 16 + E_IRQL_START,
    E_IRQ_TSP2HK                        = E_IRQH_START + 0,  // 16: tsp2hk_int
    E_IRQ_VE                            = E_IRQH_START + 1,  // 17: ve_int
    E_IRQ_G3D2MCU                       = E_IRQH_START + 2,  // 18: g3d2mcu_irq_dft
    E_IRQ_DC                            = E_IRQH_START + 3,  // 19: dc_int
    E_IRQ_GOP                           = E_IRQH_START + 4,  // 20: gop_int
    E_IRQ_PCM2MCU                       = E_IRQH_START + 5,  // 21: pcm2mcu_int
    E_IRQ_LDM_DMA1                      = E_IRQH_START + 6,  // 22: AUDMA_V2_INTR
   // E_IRQ_MFE                           = E_IRQH_START + 7,  // 23: mfe_int
    E_IRQ_ERROR                         = E_IRQH_START + 8,  // 24: error_resp_int
    E_IRQ_TSO                           = E_IRQH_START + 9,  // 25: tso_int
    E_IRQ_D2B                           = E_IRQH_START + 10, // 26: d2b_int
    E_IRQ_SCM                           = E_IRQH_START + 11, // 27: scm_int
    E_IRQ_VBI                           = E_IRQH_START + 12, // 28: vbi_int
    E_IRQ_USB                           = E_IRQH_START + 13, // 29: usb_int
    E_IRQ_UHC                           = E_IRQH_START + 14, // 30: uhc_int
    E_IRQ_USB1                          = E_IRQH_START + 15, // 31: usb_int1
    E_IRQH_END                          = 15 + E_IRQH_START,

    // FIQ
    E_FIQL_START                        = MSTAR_FIQ_BASE,       // nonPM FIQ
    E_FIQ_EXTIMER0                      = E_FIQL_START + 0,     // 0: int_timer0
    E_FIQ_EXTIMER1                      = E_FIQL_START + 1,     // 1: int_timer1
    E_FIQ_WDT                           = E_FIQL_START + 2,     // 2: int_wdt
    E_FIQ_USB2P3                        = E_FIQL_START + 3,     // 3: dig_dft_bus_out[3]
    E_FIQ_MB_AUR2TOMCU0                 = E_FIQL_START + 4,     // 4: MB_auR2toMCU_INT[0]
    E_FIQ_MB_AUR2TOMCU1                 = E_FIQL_START + 5,     // 5: MB_auR2toMCU_INT[1]
    E_FIQ_MB_DSP2TOMCU0                 = E_FIQL_START + 6,     // 6: MB_DSP2toMCU_INT[0]
    E_FIQ_MB_DSP2TOMCU1                 = E_FIQL_START + 7,     // 7: MB_DSP2toMCU_INT[1]
    //E_FIQ_RESERVED                      = E_FIQL_START + 8,     // 8: reserved
    //E_FIQ_RESERVED                      = E_FIQL_START + 9,     // 9: reserved
    E_FIQ_UHC2P3                        = E_FIQL_START + 10,    // 10: dig_dft_bus_out[10]
    E_FIQ_HDMI_NON_PCM                  = E_FIQL_START + 11,    // 11: HDMI_NON_PCM_MODE_INT_OUT
    E_FIQ_SPDIF_IN_NON_PCM              = E_FIQL_START + 12,    // 12: SPDIF_IN_NON_PCM_INT_OUT
    E_FIQ_LAN_ESD                       = E_FIQL_START + 13,    // 13: lan_esd_int
    E_FIQ_SE_DSP2UP                     = E_FIQL_START + 14,    // 14: SE_DSP2UP_intr
    E_FIQ_TSP2AEON                      = E_FIQL_START + 15,    // 15: tsp2aeon_int
    E_FIQL_END                          = 15 + E_FIQL_START,

    E_FIQH_START                        = 16 + E_FIQL_START,
    E_FIQ_VIVALDI_STR                   = E_FIQH_START + 0,     // 16: vivaldi_str_intr
    E_FIQ_VIVALDI_PTS                   = E_FIQH_START + 1,     // 17: vivaldi_pts_intr
    E_FIQ_DSP_MIU_PROT                  = E_FIQH_START + 2,     // 18: DSP_MIU_PROT_intr
    E_FIQ_XIU_TIMEOUT                   = E_FIQH_START + 3,     // 19: xiu_timeout_int
    E_FIQ_DMDMCU2HK_INT                 = E_FIQH_START + 4,     // 20: dmdmcu2hk_int
    E_FIQ_VSYNC_VE4VBI                  = E_FIQH_START + 5,     // 21: ve_vbi_f0_int
    E_FIQ_FIELD_VE4VBI                  = E_FIQH_START + 6,     // 22: ve_vbi_f1_int
    E_FIQ_VDMCU2HK                      = E_FIQH_START + 7,     // 23: vdmcu2hk_int
    E_FIQ_VE_DONE_TT                    = E_FIQH_START + 8,     // 24: ve_done_TT_irq
    E_FIQ_CCFL                          = E_FIQH_START + 9,     // 25: INT_CCFL
    E_FIQ_PM_SD                         = E_FIQH_START + 10,    // 26: int_in, PM_GPIO7
    E_FIQEXPL_IR_INT                    = E_FIQH_START + 11,    // 27: ir_int
    E_FIQ_AFEC_VSYNC                    = E_FIQH_START + 12,    // 28: AFEC_VSYNC
    //E_FIQ_RESERVED                      = E_FIQH_START + 13,    // 29: reserved
    //E_FIQ_RESERVED                      = E_FIQH_START + 14,    // 30: reserved
    E_FIQ_DSPMIPS                       = E_FIQH_START + 15,    // 31: DSP2MIPS_INT
    E_FIQH_END                          = 15 + E_FIQH_START,

    //IRQEXP
    E_IRQEXPL_START                     = 16 + E_IRQH_START,    // nonPM IRQ, E_IRQH_START = 16
    E_IRQEXPL_UHC1                      = E_IRQEXPL_START + 0,  // 32: uhc_int1
    E_IRQEXPL_USB2                      = E_IRQEXPL_START + 1,  // 33: usb_int2
    E_IRQEXPL_UHC2                      = E_IRQEXPL_START + 2,  // 34: uhc_int2
    //E_IRQEXPL_RESERVED                      = E_IRQEXPL_START + 3,  // 35: reserved
    //E_IRQEXPL_RESERVED                      = E_IRQEXPL_START + 4,  // 36: reserved
    E_IRQEXPL_MIU                       = E_IRQEXPL_START + 5,  // 37: miu_int, miu protect hit
    E_IRQEXPL_UART0                     = E_IRQEXPL_START + 6,  // 38: int_uart0
    E_IRQEXPL_UART1                     = E_IRQEXPL_START + 7,  // 39: int_uart1
    //E_IRQEXPL_RESERVED                     = E_IRQEXPL_START + 8,  // 40: reserved
    //E_IRQEXPL_RESERVED                     = E_IRQEXPL_START + 9,  // 41: reserved
    //E_IRQEXPL_RESERVED                     = E_IRQEXPL_START + 10, // 42: reserved
    //E_IRQEXPL_RESERVED                     = E_IRQEXPL_START + 11, // 43: reserved
    E_IRQEXPL_GE                        = E_IRQEXPL_START + 12, // 44: ge_int
    E_IRQEXPL_MIU_SECURITY              = E_IRQEXPL_START + 13, // 45: miu_security_int
    E_IRQEXPL_MIU_ESD                   = E_IRQEXPL_START + 14, // 46: miu_esd_int
    E_IRQEXPL_MSPI0                     = E_IRQEXPL_START + 15, // 47: mspi0_int
    E_IRQEXPL_END                       = 15 + E_IRQEXPL_START,

    E_IRQEXPH_START                     = 16 + E_IRQEXPL_START, // nonPM IRQ, E_IRQEXPL_START = 32
    E_IRQEXPH_BDMA0                     = E_IRQEXPH_START + 0,  // 48: int_bdma[0]
    E_IRQEXPH_BDMA1                     = E_IRQEXPH_START + 1,  // 49: int_bdma[1]
    E_IRQEXPH_UART2MCU                  = E_IRQEXPH_START + 2,  // 50: uart2mcu_intr
    E_IRQEXPH_URDMA2MCU                 = E_IRQEXPH_START + 3,  // 51: urdma2mcu_intr
    E_IRQEXPH_DVI_HDMI_HDCP             = E_IRQEXPH_START + 4,  // 52: dvi_hdmi_hdcp_int
    E_IRQEXPH_MHL_CBUS                  = E_IRQEXPH_START + 5,  // 53: mhl_cbus_pm_int, Not Used
    E_IRQEXPH_CEC                       = E_IRQEXPH_START + 6,  // 54: cec_int_pm
    E_IRQEXPH_HDCP_IIC                  = E_IRQEXPH_START + 7,  // 55: hdcp_iic_int
    E_IRQEXPH_HDCP_X74                  = E_IRQEXPH_START + 8,  // 56: hdcp_x74_int
    E_IRQEXPH_WADR_ERR_INT              = E_IRQEXPH_START + 9,  // 57: wadr_err_int
    E_IRQEXPH_MIIC0                     = E_IRQEXPH_START + 10, // 58: miic_0_A_int
    E_IRQEXPH_MIIC1                     = E_IRQEXPH_START + 11, // 59: miic_1_A_int
    E_IRQEXPH_ACPU                      = E_IRQEXPH_START + 12, // 60: miic_2_A_int
    E_IRQEXPH_MIIC3                     = E_IRQEXPH_START + 13, // 61: miic_3_A_int
    E_IRQEXPH_JPD_GPD_MERGE             = E_IRQEXPH_START + 14, // 62: jpd_gpd_merge_int
    E_IRQEXPH_EXT_GPIO_MERGE            = E_IRQEXPH_START + 15, // 63: ext_gpio_merge_int
    E_IRQEXPH_END                       = 15 + E_IRQEXPH_START,

    // FIQEXP
    E_FIQEXPL_START                     = 16 + E_FIQH_START,    // nonPM FIQ, E_FIQH_START = 16
    E_FIQEXPL_IR_INT_RC                 = E_FIQEXPL_START + 0,  // 32: ir_int_rc
    E_FIQEXPL_AU_DMA_BUF_INT            = E_FIQEXPL_START + 1,  // 33: AU_DMA_BUFFER_INT_EDGE
    E_FIQEXPL_IR_IN                     = E_FIQEXPL_START + 2,  // 34: ir_in
    E_FIQEXPL_EXTIMER2                  = E_FIQEXPL_START + 3,  // 35: int_timer2
    E_FIQEXPL_8051_TO_SECURE_R2         = E_FIQEXPL_START + 4,  // 36: reg_hst0to3_int
    E_FIQEXPL_8051_TO_ARM_C1            = E_FIQEXPL_START + 5,  // 37: reg_hst0to2_int
    E_FIQEXPL_8051_TO_ARM               = E_FIQEXPL_START + 6,  // 38: reg_hst0to1_int
    E_FIQEXPL_EXT_GPIO0                 = E_FIQEXPL_START + 7,  // 39: dig_dft_bus_out[39], Not Used
    E_FIQEXPL_ARM_TO_SECURE_R2          = E_FIQEXPL_START + 8,  // 40: reg_hst1to3_int
    E_FIQEXPL_ARM_TO_ARM_C1             = E_FIQEXPL_START + 9,  // 41: reg_hst1to2_int
    E_FIQEXPL_ARM_TO_8051               = E_FIQEXPL_START + 10, // 42: reg_hst1to0_int
    E_FIQEXPL_EXT_GPIO1                 = E_FIQEXPL_START + 11, // 43: dig_dft_bus_out[43], Not Used
    E_FIQEXPL_ARM_C1_TO_SECURE_R2       = E_FIQEXPL_START + 12, // 44: reg_hst2to3_int
    E_FIQEXPL_ARM_C1_TO_ARM             = E_FIQEXPL_START + 13, // 45: reg_hst2to1_int
    E_FIQEXPL_ARM_C1_TO_8051            = E_FIQEXPL_START + 14, // 46: reg_hst2to0_int
    E_FIQEXPL_EMAC_MAN_INT              = E_FIQEXPL_START + 15, // 47: emac_man_int
    E_FIQEXPL_END                       = 15 + E_FIQEXPL_START,

    E_FIQEXPH_START                     = 16 + E_FIQEXPL_START, // nonPM FIQ, E_FIQEXPL_START = 32
    E_FIQEXPL_SECURE_R2_TO_ARM_C1       = E_FIQEXPH_START + 0,  // 48: reg_hst3to2_int , no host 3
    E_FIQEXPL_SECURE_R2_TO_ARM          = E_FIQEXPH_START + 1,  // 49: reg_hst3to1_int , no host 3
    E_FIQEXPL_SECURE_R2_TO_8051         = E_FIQEXPH_START + 2,  // 50: reg_hst3to0_int , no host 3
    //E_FIQEXPH_RESERVED                  = E_FIQEXPH_START + 3,  // 51: reserved
    //E_FIQEXPH_RESERVED                  = E_FIQEXPH_START + 4,  // 52: reserved
    //E_FIQEXPH_LDM_DMA_DONE1             = E_FIQEXPH_START + 5,  // 53: ldm_dma_done_int1
    //E_FIQEXPH_LDM_DMA_DONE0             = E_FIQEXPH_START + 6,  // 54: ldm_dma_done_int0
    E_FIQEXPH_AU_SPDIF_TX0              = E_FIQEXPH_START + 7,  // 55: AU_SPDIF_TX_CS_INT[0]
    E_FIQEXPH_AU_SPDIF_TX1              = E_FIQEXPH_START + 8,  // 56: AU_SPDIF_TX_CS_INT[1]
    //E_FIQEXPH_RESERVED              = E_FIQEXPH_START + 9,
    //E_FIQEXPH_RESERVED               = E_FIQEXPH_START + 10,
    E_FIQEXPH_PWM_RP_I                  = E_FIQEXPH_START + 11, // 59: pwm_rp_l_int
    E_FIQEXPH_PWM_FP_I                  = E_FIQEXPH_START + 12, // 60: pwm_fp_l_int
    E_FIQEXPH_PWM_RP_R                  = E_FIQEXPH_START + 13, // 61: pwm_rp_r_int
    E_FIQEXPH_PWM_FP_R                  = E_FIQEXPH_START + 14, // 62: pwm_fp_r_int
    //E_FIQEXPH_RESERVED                  = E_FIQEXPH_START + 15, // 63: reserved
    E_FIQEXPH_END                       = 15 + E_FIQEXPH_START,

    // IRQHYPL
    E_IRQHYPL_START                     = MSTAR_IRQ_HYP_BASE,
    E_IRQHYPL_VD_EVD                    = E_IRQHYPL_START + 0, // 64: irq_vd_evd_r22hi
    E_IRQHYPL_PAS_PTS_COMBINE           = E_IRQHYPL_START + 1, // 65: pas_pts_intrl_combine
    E_IRQHYPL_IMI_TOP                   = E_IRQHYPL_START + 2, // 66: imi_top_irq
    E_IRQHYPL_OTG_INT                   = E_IRQHYPL_START + 3, // 67: otg_int
    E_IRQHYPL_NFIE_TEE_INT              = E_IRQHYPL_START + 4, // 68: nfie_tee_int
    E_IRQHYPL_DIP_INT			        = E_IRQHYPL_START + 5, // 69
    E_IRQHYPL_VIVALDI_AUTO_BIST_INT     = E_IRQHYPL_START + 6, // 70
    E_IRQHYPL_MHEG5_AUTO_BIST_INT       = E_IRQHYPL_START + 7, // 71
    E_IRQHYPL_TSP_AUTO_BIST_INT         = E_IRQHYPL_START + 8, // 72
    E_IRQHYPL_CODEC_AUTO_BIST_INT       = E_IRQHYPL_START + 9, // 73
    E_IRQHYPL_HI_CODEC_AUTO_BIST_INT    = E_IRQHYPL_START + 10,// 74
    E_IRQHYPL_SC_AUTO_BIST_INT          = E_IRQHYPL_START + 11,// 75
    E_IRQHYPL_PM_IRQ_OUT		        = E_IRQHYPL_START + 12,// 76
    E_IRQHYPL_TSEN_0_OV_IRQ             = E_IRQHYPL_START + 13,// 77
    E_IRQHYPL_TSEN_1_OV_IRQ             = E_IRQHYPL_START + 14,// 78
    E_IRQHYPL_TSP_FI_QUEUE              = E_IRQHYPL_START + 15,// 79
    E_IRQHYPL_END                       = 15 + E_IRQHYPL_START,

    // IRQHYPH
    E_IRQHYPH_START                     = 16 + E_IRQHYPL_START,
    E_IRQHYPH_SAR_KP_INT                = E_IRQHYPH_START + 0,
    E_IRQHYPH_MIPI_DSI_INT              = E_IRQHYPH_START + 1,
    E_IRQHYPH_LZMA_INT                  = E_IRQHYPH_START + 2,
    E_IRQHYPH_PAGA_INT_LEVEL            = E_IRQHYPH_START + 3,
    E_IRQHYPH_MOD_DETECT                = E_IRQHYPH_START + 4, // 84:
    E_IRQHYPH_LPLL                      = E_IRQHYPH_START + 5, // 85:
    E_IRQHYPH_ADCPLL                    = E_IRQHYPH_START + 6, // 86:
    E_IRQHYPH_MIIC2                     = E_IRQHYPH_START + 7, // 87:
    //E_IRQHYPH_RESERVED                  = E_IRQHYPH_START + 8, // 88: reserved
    E_IRQHYPH_MSPI2                     = E_IRQHYPH_START + 9, // 89:
    E_IRQHYPH_AESDMA_S_INT              = E_IRQHYPH_START + 10,// 90: aesdma_s_int
    E_IRQHYPH_PKA_ALL                   = E_IRQHYPH_START + 11,// 91:
    E_IRQHYPH_RESERVED12                = E_IRQHYPH_START + 12,// 92:
    E_IRQHYPH_RESERVED13                = E_IRQHYPH_START + 13,// 93:
    E_IRQHYPH_RESERVED14                = E_IRQHYPH_START + 14,// 94:
    E_IRQHYPH_RESERVED15                = E_IRQHYPH_START + 15,// 95:
    E_IRQHYPH_END                       = 15 + E_IRQHYPH_START,

    // IRQSUPL
    E_IRQSUPL_START                     = 16 + E_IRQHYPH_START,
    E_IRQSUPL_RESERVED16                = E_IRQSUPL_START + 0,
    E_IRQSUPL_RESERVED17                = E_IRQSUPL_START + 1,
    E_IRQSUPL_RESERVED18                = E_IRQSUPL_START + 2,
    E_IRQSUPL_RESERVED19                = E_IRQSUPL_START + 3,
    E_IRQSUPL_RESERVED20                = E_IRQSUPL_START + 4,
    E_IRQSUPL_RESERVED21                = E_IRQSUPL_START + 5,
    E_IRQSUPL_RESERVED22                = E_IRQSUPL_START + 6,
    E_IRQSUPL_RESERVED23                = E_IRQSUPL_START + 7,
    E_IRQSUPL_RESERVED24                = E_IRQSUPL_START + 8,
    E_IRQSUPL_RESERVED25                = E_IRQSUPL_START + 9,
    E_IRQSUPL_RESERVED26                = E_IRQSUPL_START + 10,
    E_IRQSUPL_RESERVED27                = E_IRQSUPL_START + 11,
    E_IRQSUPL_RESERVED28                = E_IRQSUPL_START + 12,
    E_IRQSUPL_RESERVED29                = E_IRQSUPL_START + 13,
    E_IRQSUPL_RESERVED30                = E_IRQSUPL_START + 14,
    E_IRQSUPL_RESERVED31                = E_IRQSUPL_START + 15,
    E_IRQSUPL_END                       = 15 + E_IRQSUPL_START,

    // IRQSUPH
    E_IRQSUPH_START                     = 16 + E_IRQSUPL_START,
    E_IRQSUPH_RESERVED32                = E_IRQSUPH_START + 0,
    E_IRQSUPH_RESERVED33                = E_IRQSUPH_START + 1,
    E_IRQSUPH_RESERVED34                = E_IRQSUPH_START + 2,
    E_IRQSUPH_RESERVED35                = E_IRQSUPH_START + 3,
    E_IRQSUPH_RESERVED36                = E_IRQSUPH_START + 4,
    E_IRQSUPH_RESERVED37                = E_IRQSUPH_START + 5,
    E_IRQSUPH_RESERVED38                = E_IRQSUPH_START + 6,
    E_IRQSUPH_RESERVED39                = E_IRQSUPH_START + 7,
    E_IRQSUPH_RESERVED40                = E_IRQSUPH_START + 8,
    E_IRQSUPH_RESERVED41                = E_IRQSUPH_START + 9,
    E_IRQSUPH_RESERVED42                = E_IRQSUPH_START + 10,
    E_IRQSUPH_RESERVED43                = E_IRQSUPH_START + 11,
    E_IRQSUPH_RESERVED44                = E_IRQSUPH_START + 12,
    E_IRQSUPH_RESERVED45                = E_IRQSUPH_START + 13,
    E_IRQSUPH_RESERVED46                = E_IRQSUPH_START + 14,
    E_IRQSUPH_RESERVED47                = E_IRQSUPH_START + 15,
    E_IRQSUPH_END                       = 15 + E_IRQSUPH_START,

    // FIQHYPL
    E_FIQHYPL_START                     =  MSTAR_FIQ_HYP_BASE,
    E_FIQHYPL_RESERVED                  = E_FIQHYPL_START + 0,
    E_FIQHYPL_RESERVED01                = E_FIQHYPL_START + 1,
    E_FIQHYPL_RESERVED02                = E_FIQHYPL_START + 2,
    E_FIQHYPL_RESERVED03                = E_FIQHYPL_START + 3,
    E_FIQHYPL_RESERVED04                = E_FIQHYPL_START + 4,
    E_FIQHYPL_RESERVED05                = E_FIQHYPL_START + 5,
    E_FIQHYPL_RESERVED06                = E_FIQHYPL_START + 6,
    E_FIQHYPL_RESERVED07                = E_FIQHYPL_START + 7,
    E_FIQHYPL_RESERVED08                = E_FIQHYPL_START + 8,
    E_FIQHYPL_RESERVED09                = E_FIQHYPL_START + 9,
    E_FIQHYPL_RESERVED10                = E_FIQHYPL_START + 10,
    E_FIQHYPL_RESERVED11                = E_FIQHYPL_START + 11,
    E_FIQHYPL_RESERVED12                = E_FIQHYPL_START + 12,
    E_FIQHYPL_RESERVED13                = E_FIQHYPL_START + 13,
    E_FIQHYPL_RESERVED14                = E_FIQHYPL_START + 14,
    E_FIQHYPL_RESERVED15                = E_FIQHYPL_START + 15,
    E_FIQHYPL_END                       = 15 + E_FIQHYPL_START,

    // FIQHYPH
    E_FIQHYPH_START                     = 16 + E_FIQHYPL_START,
    E_FIQHYPH_RESERVED00                = E_FIQHYPH_START + 0,
    E_FIQHYPH_RESERVED01                = E_FIQHYPH_START + 1,
    E_FIQHYPH_RESERVED02                = E_FIQHYPH_START + 2,
    E_FIQHYPH_RESERVED03                = E_FIQHYPH_START + 3,
    E_FIQHYPH_RESERVED04                = E_FIQHYPH_START + 4,
    E_FIQHYPH_RESERVED05                = E_FIQHYPH_START + 5,
    E_FIQHYPH_RESERVED06                = E_FIQHYPH_START + 6,
    E_FIQHYPH_RESERVED07                = E_FIQHYPH_START + 7,
    E_FIQHYPH_RESERVED08                = E_FIQHYPH_START + 8,
    E_FIQHYPH_RESERVED09                = E_FIQHYPH_START + 9,
    E_FIQHYPH_RESERVED10                = E_FIQHYPH_START + 10,
    E_FIQHYPH_RESERVED11                = E_FIQHYPH_START + 11,
    E_FIQHYPH_RESERVED12                = E_FIQHYPH_START + 12,
    E_FIQHYPH_RESERVED13                = E_FIQHYPH_START + 13,
    E_FIQHYPH_RESERVED14                = E_FIQHYPH_START + 14,
    E_FIQHYPH_RESERVED15                = E_FIQHYPH_START + 15,
    E_FIQHYPH_END                       = 15 + E_FIQHYPH_START,

    // FIQSUPL
    E_FIQSUPL_START                     = 16 + E_FIQHYPH_START,
    E_FIQSUPL_RESERVED16                = E_FIQSUPL_START + 0,
    E_FIQSUPL_RESERVED17                = E_FIQSUPL_START + 1,
    E_FIQSUPL_RESERVED18                = E_FIQSUPL_START + 2,
    E_FIQSUPL_RESERVED19                = E_FIQSUPL_START + 3,
    E_FIQSUPL_RESERVED20                = E_FIQSUPL_START + 4,
    E_FIQSUPL_RESERVED21                = E_FIQSUPL_START + 5,
    E_FIQSUPL_RESERVED22                = E_FIQSUPL_START + 6,
    E_FIQSUPL_RESERVED23                = E_FIQSUPL_START + 7,
    E_FIQSUPL_RESERVED24                = E_FIQSUPL_START + 8,
    E_FIQSUPL_RESERVED25                = E_FIQSUPL_START + 9,
    E_FIQSUPL_RESERVED26                = E_FIQSUPL_START + 10,
    E_FIQSUPL_RESERVED27                = E_FIQSUPL_START + 11,
    E_FIQSUPL_RESERVED28                = E_FIQSUPL_START + 12,
    E_FIQSUPL_RESERVED29                = E_FIQSUPL_START + 13,
    E_FIQSUPL_RESERVED30                = E_FIQSUPL_START + 14,
    E_FIQSUPL_RESERVED31                = E_FIQSUPL_START + 15,
    E_FIQSUPL_END                       = 15 + E_FIQSUPL_START,

    // FIQSUPH
    E_FIQSUPH_START                     = 16 + E_FIQSUPL_START,
    E_FIQSUPH_RESERVED32                = E_FIQSUPH_START + 0,
    E_FIQSUPH_RESERVED33                = E_FIQSUPH_START + 1,
    E_FIQSUPH_RESERVED34                = E_FIQSUPH_START + 2,
    E_FIQSUPH_RESERVED35                = E_FIQSUPH_START + 3,
    E_FIQSUPH_RESERVED36                = E_FIQSUPH_START + 4,
    E_FIQSUPH_RESERVED37                = E_FIQSUPH_START + 5,
    E_FIQSUPH_RESERVED38                = E_FIQSUPH_START + 6,
    E_FIQSUPH_RESERVED39                = E_FIQSUPH_START + 7,
    E_FIQSUPH_RESERVED40                = E_FIQSUPH_START + 8,
    E_FIQSUPH_RESERVED41                = E_FIQSUPH_START + 9,
    E_FIQSUPH_RESERVED42                = E_FIQSUPH_START + 10,
    E_FIQSUPH_RESERVED43                = E_FIQSUPH_START + 11,
    E_FIQSUPH_RESERVED44                = E_FIQSUPH_START + 12,
    E_FIQSUPH_RESERVED45                = E_FIQSUPH_START + 13,
    E_FIQSUPH_RESERVED46                = E_FIQSUPH_START + 14,
    E_FIQSUPH_RESERVED47                = E_FIQSUPH_START + 15,
    E_FIQSUPH_END                       = 15 + E_FIQSUPH_START,

    E_IRQ_FIQ_ALL                       = 0xFF //all IRQs & FIQs
} InterruptNum;

#define E_FIQEXPL_IR_INT_RC5_RC6 E_FIQEXPL_IR_INT_RC
// REG_FIQ_MASK_L
//FIQ Low 16 bits
#define FIQL_MASK                       0xFFFF
#define FIQ_EXTIMER0                    (0x01 << (E_FIQ_EXTIMER0            - E_FIQL_START))
#define FIQ_EXTIMER1                    (0x01 << (E_FIQ_EXTIMER1            - E_FIQL_START))
#define FIQ_WDT                         (0x01 << (E_FIQ_WDT                 - E_FIQL_START))
#define FIQ_MB_AUR2TOMCU0               (0x01 << (E_FIQ_MB_AUR2TOMCU0       - E_FIQL_START))
#define FIQ_MB_AUR2TOMCU1               (0x01 << (E_FIQ_MB_AUR2TOMCU1       - E_FIQL_START))
#define FIQ_MB_DSP2TOMCU0               (0x01 << (E_FIQ_MB_DSP2TOMCU0       - E_FIQL_START))
#define FIQ_MB_DSP2TOMCU1               (0x01 << (E_FIQ_MB_DSP2TOMCU1       - E_FIQL_START))
//#define FIQ_USB                         (0x01 << (E_FIQ_USB                 - E_FIQL_START))
//#define FIQ_UHC                         (0x01 << (E_FIQ_UHC                 - E_FIQL_START))
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
#define FIQ_VSYNC_VE4VBI                (0x01 << (E_FIQ_VSYNC_VE4VBI        - E_FIQH_START))
#define FIQ_FIELD_VE4VBI                (0x01 << (E_FIQ_FIELD_VE4VBI        - E_FIQH_START))
#define FIQ_VDMCU2HK                    (0x01 << (E_FIQ_VDMCU2HK            - E_FIQH_START))
#define FIQ_VE_DONE_TT                  (0x01 << (E_FIQ_VE_DONE_TT          - E_FIQH_START))
//#define FIQ_RESERVED                    (0x01 << (E_FIQ_RESERVED            - E_FIQH_START))
#define FIQ_PM_SD                       (0x01 << (E_FIQ_PM_SD               - E_FIQH_START))
#define FIQEXPL_IR_INT                  (0x01 << (E_FIQEXPL_IR_INT          - E_FIQH_START))
#define FIQ_AFEC_VSYNC                  (0x01 << (E_FIQ_AFEC_VSYNC          - E_FIQH_START))
//#define FIQ_USB2                        (0x01 << (E_FIQ_USB2                - E_FIQH_START)) 
//#define FIQ_UHC2                        (0x01 << (E_FIQ_UHC2                - E_FIQH_START))
#define FIQ_DSPMIPS                     (0x01 << (E_FIQ_DSPMIPS             - E_FIQH_START))



// #define REG_IRQ_PENDING_L
#define IRQL_MASK                       0xFFFF
#define IRQ_DCSUB                       (0x01 << (E_IRQ_DCSUB                   - E_IRQL_START))
#define IRQ_PM_SLEEP                    (0x01 << (E_IRQ_PM_SLEEP                - E_IRQL_START))
#define IRQ_AEON                        (0x01 << (E_IRQ_AEON                    - E_IRQL_START))
#define IRQ_MVD                         (0x01 << (E_IRQ_MVD                     - E_IRQL_START))
#define IRQ_PS                          (0x01 << (E_IRQ_PS                      - E_IRQL_START))
#define IRQ_NFIE                        (0x01 << (E_IRQ_NFIE                    - E_IRQL_START))
#define IRQ_SDIO                        (0x01 << (E_IRQ_SDIO                    - E_IRQL_START))
//#define IRQ_MIIC4                       (0x01 << (E_IRQ_MIIC4                   - E_IRQL_START))
#define IRQ_SMART                       (0x01 << (E_IRQEXPH_SMART               - E_IRQL_START))
#define IRQ_EMAC                        (0x01 << (E_IRQ_EMAC                    - E_IRQL_START))
#define IRQ_DISP                        (0x01 << (E_IRQ_DISP                    - E_IRQL_START))
#define IRQ_MVD2MIPS                    (0x01 << (E_IRQ_MVD2MIPS                - E_IRQL_START))
#define IRQ_HVD                         (0x01 << (E_IRQ_HVD                     - E_IRQL_START))
#define IRQ_EVD                         (0x01 << (E_IRQ_EVD                     - E_IRQL_START))
#define IRQ_COMB                        (0x01 << (E_IRQ_COMB                    - E_IRQL_START))
//#define IRQ_ADCDVI2RIU                  (0x01 << (E_IRQ_ADCDVI2RIU              - E_IRQL_START))


// #define REG_IRQ_PENDING_H
#define IRQH_MASK                       0xFFFF
#define IRQ_TSP2HK                      (0x01 << (E_IRQ_TSP2HK            - E_IRQH_START))
#define IRQ_VE                          (0x01 << (E_IRQ_VE                - E_IRQH_START))
#define IRQ_G3D2MCU                     (0x01 << (E_IRQ_G3D2MCU           - E_IRQH_START))
#define IRQ_DC                          (0x01 << (E_IRQ_DC                - E_IRQH_START))
#define IRQ_GOP                         (0x01 << (E_IRQ_GOP               - E_IRQH_START))
#define IRQ_PCM2MCU                     (0x01 << (E_IRQ_PCM2MCU           - E_IRQH_START))
#define IRQ_LDM_DMA1                    (0x01 << (E_IRQ_LDM_DMA1          - E_IRQH_START))
//#define IRQ_MFE                         (0x01 << (E_IRQ_MFE               - E_IRQH_START))
#define IRQ_ERROR                       (0x01 << (E_IRQ_ERROR             - E_IRQH_START))
#define IRQ_TSO                         (0x01 << (E_IRQ_TSO               - E_IRQH_START))
#define IRQ_D2B                         (0x01 << (E_IRQ_D2B               - E_IRQH_START))
#define IRQ_SCM                         (0x01 << (E_IRQ_SCM               - E_IRQH_START))
#define IRQ_VBI                         (0x01 << (E_IRQ_VBI               - E_IRQH_START))
#define IRQ_USB                         (0x01 << (E_IRQ_USB               - E_IRQH_START))
#define IRQ_UHC                         (0x01 << (E_IRQ_UHC               - E_IRQH_START))
#define IRQ_USB1                        (0x01 << (E_IRQ_USB1              - E_IRQH_START))

// #define REG_IRQEXP_PENDING_L
#define IRQEXPL_MASK                    0xFFFF
#define IRQEXPL_UHC1                    (0x01 << (E_IRQEXPL_UHC1            - E_IRQEXPL_START))
#define IRQEXPL_USB2                    (0x01 << (E_IRQEXPL_USB2            - E_IRQEXPL_START))
#define IRQEXPL_UHC2                    (0x01 << (E_IRQEXPL_UHC2            - E_IRQEXPL_START))
//#define IRQEXPL_USB3                    (0x01 << (E_IRQEXPL_USB3            - E_IRQEXPL_START))
//#define IRQEXPL_UHC3                    (0x01 << (E_IRQEXPL_UHC3            - E_IRQEXPL_START))
#define IRQEXPL_MIU_INT                 (0x01 << (E_IRQEXPL_MIU		          - E_IRQEXPL_START))
#define IRQEXPL_UART0                   (0x01 << (E_IRQEXPL_UART0           - E_IRQEXPL_START))
#define IRQEXPL_UART1                   (0x01 << (E_IRQEXPL_UART1           - E_IRQEXPL_START))
//#define IRQEXPL_UART2                   (0x01 << (E_IRQEXPL_UART2           - E_IRQEXPL_START))
//#define IRQEXPL_UART3                   (0x01 << (E_IRQEXPL_UART3           - E_IRQEXPL_START))
//#define IRQEXPL_UART4                   (0x01 << (E_IRQEXPL_UART4           - E_IRQEXPL_START))
//#define IRQEXPL_UART5                   (0x01 << (E_IRQEXPL_UART5           - E_IRQEXPL_START))
#define IRQEXPL_GE                      (0x01 << (E_IRQEXPL_GE              - E_IRQEXPL_START))
#define IRQEXPL_MIU_SECURITY            (0x01 << (E_IRQEXPL_MIU_SECURITY    - E_IRQEXPL_START))
#define IRQEXPL_MIU_ESD                 (0x01 << (E_IRQEXPL_MIU_ESD         - E_IRQEXPL_START))
#define IRQEXPL_MSPI0                   (0x01 << (E_IRQEXPL_MSPI0           - E_IRQEXPL_START))


// #define REG_IRQEXP_PENDING_H
#define IRQEXPH_MASK                    0xFFFF
#define IRQEXPH_BDMA0                   (0x01 << (E_IRQEXPH_BDMA0               - E_IRQEXPH_START))
#define IRQEXPH_BDMA1                   (0x01 << (E_IRQEXPH_BDMA1               - E_IRQEXPH_START))
#define IRQEXPH_UART2MCU                (0x01 << (E_IRQEXPH_UART2MCU            - E_IRQEXPH_START))
#define IRQEXPH_URDMA2MCU               (0x01 << (E_IRQEXPH_URDMA2MCU           - E_IRQEXPH_START))
#define IRQEXPH_DVI_HDMI_HDCP           (0x01 << (E_IRQEXPH_DVI_HDMI_HDCP       - E_IRQEXPH_START))
#define IRQEXPH_MHL_CBUS                (0x01 << (E_IRQEXPH_MHL_CBUS            - E_IRQEXPH_START))
#define IRQEXPH_CEC                     (0x01 << (E_IRQEXPH_CEC                 - E_IRQEXPH_START))
#define IRQEXPH_HDCP_IIC                (0x01 << (E_IRQEXPH_HDCP_IIC            - E_IRQEXPH_START))
#define IRQEXPH_HDCP_X74                (0x01 << (E_IRQEXPH_HDCP_X74            - E_IRQEXPH_START))
#define IRQEXPH_WADR_ERR_INT            (0x01 << (E_IRQEXPH_WADR_ERR_INT        - E_IRQEXPH_START))
#define IRQEXPH_MIIC0                   (0x01 << (E_IRQEXPH_MIIC0               - E_IRQEXPH_START))
#define IRQEXPH_MIIC1                   (0x01 << (E_IRQEXPH_MIIC1               - E_IRQEXPH_START))
#define IRQEXPH_ACPU                    (0x01 << (E_IRQEXPH_ACPU                - E_IRQEXPH_START))
#define IRQEXPH_MIIC3                   (0x01 << (E_IRQEXPH_MIIC3               - E_IRQEXPH_START))
#define IRQEXPH_JPD_GPD_MERGE           (0x01 << (E_IRQEXPH_JPD_GPD_MERGE       - E_IRQEXPH_START))
#define IRQEXPH_EXT_GPIO_MERGE          (0x01 << (E_IRQEXPH_EXT_GPIO_MERGE      - E_IRQEXPH_START))


// #define REG_FIQEXP_PENDING_L
#define FIQEXPL_MASK                    0xFFFF
#define FIQEXPL_IR_INT_RC               (0x01 << (E_FIQEXPL_IR_INT_RC           - E_FIQEXPL_START))
#define FIQEXPL_AU_DMA_BUF_INT          (0x01 << (E_FIQEXPL_AU_DMA_BUF_INT      - E_FIQEXPL_START))
#define FIQEXPL_IR_IN                   (0x01 << (E_FIQEXPL_IR_IN               - E_FIQEXPL_START))
#define FIQEXPL_EXTIMER2                (0x01 << (E_FIQEXPL_EXTIMER2            - E_FIQEXPL_START))
#define FIQEXPL_8051_TO_ARM_C1          (0x01 << (E_FIQEXPL_8051_TO_ARM_C1      - E_FIQEXPL_START))
#define FIQEXPL_8051_TO_ARM             (0x01 << (E_FIQEXPL_8051_TO_ARM         - E_FIQEXPL_START))
#define FIQEXPL_8051_TO_SECURE_R2       (0x01 << (E_FIQEXPL_8051_TO_SECURE_R2   - E_FIQEXPL_START))
//#define FIQEXPL_EXT_GPIO0               (0x01 << (E_FIQEXPL_EXT_GPIO0           - E_FIQEXPL_START))
#define FIQEXPL_SECURE_R2_TO_ARM_C1     (0x01 << (E_FIQEXPL_SECURE_R2_TO_ARM_C1 - E_FIQEXPL_START))
#define FIQEXPL_SECURE_R2_TO_ARM        (0x01 << (E_FIQEXPL_SECURE_R2_TO_ARM    - E_FIQEXPL_START))
#define FIQEXPL_SECURE_R2_TO_8051       (0x01 << (E_FIQEXPL_SECURE_R2_TO_8051   - E_FIQEXPL_START))
//#define FIQEXPL_EXT_GPIO1               (0x01 << (E_FIQEXPL_EXT_GPIO1           - E_FIQEXPL_START))
#define FIQEXPL_ARM_TO_ARM_C1           (0x01 << (E_FIQEXPL_ARM_TO_ARM_C1       - E_FIQEXPL_START))
#define FIQEXPL_ARM_TO_SECURE_R2        (0x01 << (E_FIQEXPL_ARM_TO_SECURE_R2    - E_FIQEXPL_START))
#define FIQEXPL_ARM_TO_8051             (0x01 << (E_FIQEXPL_ARM_TO_8051         - E_FIQEXPL_START))
#define FIQEXPL_EMAC_MAN_INT            (0x01 << (E_FIQEXPL_EMAC_MAN_INT        - E_FIQEXPL_START))

// #define REG_FIQEXP_PENDING_H
#define FIQEXPL_ARM_C1_TO_ARM           (0x01 << (E_FIQEXPL_ARM_C1_TO_ARM  - E_FIQEXPH_START))
#define FIQEXPL_ARM_C1_TO_SECU          (0x01 << (E_FIQEXPL_ARM_C1_TO_SECU - E_FIQEXPH_START))
#define FIQEXPL_ARM_C1_TO_8051          (0x01 << (E_FIQEXPL_ARM_C1_TO_8051 - E_FIQEXPH_START))
//#define FIQEXPH_USB1                    (0x01 << (E_FIQEXPH_USB1           - E_FIQEXPH_START))
//#define FIQEXPH_UHC1                    (0x01 << (E_FIQEXPH_UHC1           - E_FIQEXPH_START))
//#define FIQEXPH_LDM_DMA_DONE1           (0x01 << (E_FIQEXPH_LDM_DMA_DONE1  - E_FIQEXPH_START))
//#define FIQEXPH_LDM_DMA_DONE0           (0x01 << (E_FIQEXPH_LDM_DMA_DONE0  - E_FIQEXPH_START))
#define FIQEXPH_AU_SPDIF_TX0            (0x01 << (E_FIQEXPH_AU_SPDIF_TX0   - E_FIQEXPH_START))
#define FIQEXPH_AU_SPDIF_TX1            (0x01 << (E_FIQEXPH_AU_SPDIF_TX1   - E_FIQEXPH_START))
//#define FIQEXPH_USB3                    (0x01 << (E_FIQEXPH_USB3           - E_FIQEXPH_START))
//#define FIQEXPH_UHC3                    (0x01 << (E_FIQEXPH_UHC3           - E_FIQEXPH_START))
#define FIQEXPH_PWM_RP_I                (0x01 << (E_FIQEXPH_PWM_RP_I       - E_FIQEXPH_START))
#define FIQEXPH_PWM_FP_I                (0x01 << (E_FIQEXPH_PWM_FP_I       - E_FIQEXPH_START))
#define FIQEXPH_PWM_RP_R                (0x01 << (E_FIQEXPH_PWM_RP_R       - E_FIQEXPH_START))
#define FIQEXPH_PWM_FP_R                (0x01 << (E_FIQEXPH_PWM_FP_R       - E_FIQEXPH_START))
//#define FIQEXPH_SP12FCIE                (0x01 << (E_FIQEXPH_SP12FCIE       - E_FIQEXPH_START))


// #define REG_IRQHYP_PENDING_L
#define IRQHYPL_MASK                     0xFFFF
#define IRQHYPL_VD_EVD                   (0x01 << (E_IRQHYPL_VD_EVD            - E_IRQHYPL_START))
#define IRQHYPL_PAS_PTS_COMBINE          (0x01 << (E_IRQHYPL_PAS_PTS_COMBINE   - E_IRQHYPL_START))
#define IRQHYPL_IMI_TOP                  (0x01 << (E_IRQHYPL_IMI_TOP           - E_IRQHYPL_START))
#define IRQHYPL_OTG_INT                  (0x01 << (E_IRQHYPL_OTG_INT           - E_IRQHYPL_START))
#define IRQHYPL_NFIE_TEE_INT             (0x01 << (E_IRQHYPL_NFIE_TEE_INT      - E_IRQHYPL_START))
#define IRQHYPL_DIP_INT                  (0x01 << (E_IRQHYPL_DIP_INT          - E_IRQHYPL_START))
#define IRQHYPL_VIVALDI_AUTO_BIST_INT    (0x01 << (E_IRQHYPL_VIVALDI_AUTO_BIST_INT   - E_IRQHYPL_START))
#define IRQHYPL_MHEG5_AUTO_BIST_INT      (0x01 << (E_IRQHYPL_MHEG5_AUTO_BIST_INT        - E_IRQHYPL_START))
#define IRQHYPL_TSP_AUTO_BIST_INT        (0x01 << (E_IRQHYPL_TSP_AUTO_BIST_INT        - E_IRQHYPL_START))
#define IRQHYPL_CODEC_AUTO_BIST_INT      (0x01 << (E_IRQHYPL_CODEC_AUTO_BIST_INT        - E_IRQHYPL_START))
#define IRQHYPL_HI_CODEC_AUTO_BIST_INT   (0x01 << (E_IRQHYPL_HI_CODEC_AUTO_BIST_INT        - E_IRQHYPL_START))
#define IRQHYPL_SC_AUTO_BIST_INT         (0x01 << (E_IRQHYPL_SC_AUTO_BIST_INT        - E_IRQHYPL_START))
#define IRQHYPL_PM_IRQ_OUT               (0x01 << (E_IRQHYPL_PM_IRQ_OUT        - E_IRQHYPL_START))
#define IRQHYPL_TSEN_0_OV_IRQ            (0x01 << (E_IRQHYPL_TSEN_0_OV_IRQ         - E_IRQHYPL_START))
#define IRQHYPL_TSEN_1_OV_IRQ               (0x01 << (E_IRQHYPL_TSEN_1_OV_IRQ        - E_IRQHYPL_START))
#define IRQHYPL_TSP_FI_QUEUE            (0x01 << (E_IRQHYPL_TSP_FI_QUEUE         - E_IRQHYPL_START))

// #define REG_IRQHYP_PENDING_H
#define IRQHYPH_MASK                     0xFFFF
#define IRQHYPH_SAR_KP_INT                 (0x01 << (E_IRQHYPH_SAR_KP_INT               - E_IRQHYPH_START))
#define IRQHYPH_MIPI_DSI_INT               (0x01 << (E_IRQHYPH_MIPI_DSI_INT             - E_IRQHYPH_START))
#define IRQHYPH_LZMA_INT               (0x01 << (E_IRQHYPH_LZMA_INT             - E_IRQHYPH_START))
#define IRQHYPH_PAGA_INT_LEVEL               (0x01 << (E_IRQHYPH_PAGA_INT_LEVEL             - E_IRQHYPH_START))
#define IRQHYPH_MOD_DETECT               (0x01 << (E_IRQHYPH_MOD_DETECT             - E_IRQHYPH_START))
#define IRQHYPH_LPLL                     (0x01 << (E_IRQHYPH_LPLL                   - E_IRQHYPH_START))
#define IRQHYPH_ADCPLL                   (0x01 << (E_IRQHYPH_ADCPLL                 - E_IRQHYPH_START))
#define IRQHYPH_MIIC2                    (0x01 << (E_IRQHYPH_MIIC2                  - E_IRQHYPH_START))
//#define IRQHYPH_RXIU_TIMEOUT_INT         (0x01 << (E_IRQHYPH_RXIU_TIMEOUT_INT       - E_IRQHYPH_START))
#define IRQHYPH_MSPI2                    (0x01 << (E_IRQHYPH_MSPI2                  - E_IRQHYPH_START))
#define IRQHYPH_AESDMA_S_INT             (0x01 << (E_IRQHYPH_AESDMA_S_INT           - E_IRQHYPH_START))
#define IRQHYPH_PKA_ALL                  (0x01 << (E_IRQHYPH_PKA_ALL                - E_IRQHYPH_START))
#define IRQHYPH_RESERVED12               (0x01 << (E_IRQHYPH_RESERVED12             - E_IRQHYPH_START))
#define IRQHYPH_RESERVED13               (0x01 << (E_IRQHYPH_RESERVED13             - E_IRQHYPH_START))
#define IRQHYPH_RESERVED14               (0x01 << (E_IRQHYPH_RESERVED14             - E_IRQHYPH_START))
#define IRQHYPH_RESERVED15               (0x01 << (E_IRQHYPH_RESERVED15             - E_IRQHYPH_START))


// #define REG_FIQHYP_PENDING_L
#define FIQEXPL_MASK                    0xFFFF
#define FIQHYPL_RESERVED                (0x01 << (E_FIQHYPL_RESERVED        - E_FIQHYPL_START))
#define FIQHYPL_RESERVED01              (0x01 << (E_FIQHYPL_RESERVED01      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED02              (0x01 << (E_FIQHYPL_RESERVED02      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED03              (0x01 << (E_FIQHYPL_RESERVED03      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED04              (0x01 << (E_FIQHYPL_RESERVED04      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED05              (0x01 << (E_FIQHYPL_RESERVED05      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED13              (0x01 << (E_FIQHYPL_RESERVED13      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED07              (0x01 << (E_FIQHYPL_RESERVED07      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED08              (0x01 << (E_FIQHYPL_RESERVED08      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED09              (0x01 << (E_FIQHYPL_RESERVED09      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED10              (0x01 << (E_FIQHYPL_RESERVED10      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED11              (0x01 << (E_FIQHYPL_RESERVED11      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED12              (0x01 << (E_FIQHYPL_RESERVED12      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED13              (0x01 << (E_FIQHYPL_RESERVED13      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED14              (0x01 << (E_FIQHYPL_RESERVED14      - E_FIQHYPL_START))
#define FIQHYPL_RESERVED15              (0x01 << (E_FIQHYPL_RESERVED15      - E_FIQHYPL_START))


// #define REG_FIQHYP_PENDING_H
#define FIQEXPH_MASK                    0xFFFF
#define FIQHYPH_RESERVED00              (0x01 << (E_FIQHYPH_RESERVED00         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED01              (0x01 << (E_FIQHYPH_RESERVED01         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED02              (0x01 << (E_FIQHYPH_RESERVED02         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED03              (0x01 << (E_FIQHYPH_RESERVED03         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED04              (0x01 << (E_FIQHYPH_RESERVED04         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED05              (0x01 << (E_FIQHYPH_RESERVED05         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED06              (0x01 << (E_FIQHYPH_RESERVED06         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED07              (0x01 << (E_FIQHYPH_RESERVED07         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED08              (0x01 << (E_FIQHYPH_RESERVED08         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED09              (0x01 << (E_FIQHYPH_RESERVED09         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED10              (0x01 << (E_FIQHYPH_RESERVED10         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED11              (0x01 << (E_FIQHYPH_RESERVED11         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED12              (0x01 << (E_FIQHYPH_RESERVED12         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED13              (0x01 << (E_FIQHYPH_RESERVED13         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED14              (0x01 << (E_FIQHYPH_RESERVED14         - E_FIQHYPH_START))
#define FIQHYPH_RESERVED15              (0x01 << (E_FIQHYPH_RESERVED15         - E_FIQHYPH_START))

#define IRQL_EXP_ALL                    0xFFFF
#define IRQH_EXP_ALL                    0xFFFF
#define FIQL_EXP_ALL                    0xFFFF
#define FIQH_EXP_ALL                    0xFFFF

#define E_FIQ_IR   E_FIQEXPL_IR_IN

extern unsigned int spi_to_ppi[NR_IRQS];
extern unsigned int interrupt_configs[MSTAR_CHIP_INT_END/16+1];

void init_chip_spi_config(void);

#endif // #ifndef __CHIP_INT_H__
