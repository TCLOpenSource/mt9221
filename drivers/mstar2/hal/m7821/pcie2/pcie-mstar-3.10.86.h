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



#ifndef _PCIE_MSTAR_31086_H
#define _PCIE_MSTAR_31086_H

#include "chip_int.h"

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	#include <mstar/mstar_chip.h>

	#if defined(CONFIG_ARM64)
		extern ptrdiff_t mstar_pm_base;
		#define MSTAR_PM_BASE		(mstar_pm_base)
	#else
		#define MSTAR_PM_BASE      0xfd000000UL
	#endif

	#define MSTAR_PCIE_REG_BASE		MSTAR_PM_BASE

#else
#error chip setting
#endif

#define ENABLE_PCIE_PORT0			/* PORT0 is always on*/

#define MIU0_ENABLE			1
#define MIU1_ENABLE			0
#define MIU2_ENABLE			0
#define MIU0_BUS_BASE_ADDR	(MSTAR_MIU0_BUS_BASE)
#define MIU2_BUS_BASE_ADDR	0
#define MIU1_BUS_BASE_ADDR	0
#define MIU0_BUS_LENGTH		((u64)32 * 0x100000000 - 1)
#define MIU1_BUS_LENGTH		0
#define MIU2_BUS_LENGTH		0

#define MSTAR_PCIE_VIRTUAL_IRQ_BASE			E_IRQHYPH_PCIE0
#define MSTAR_PCIE_VIRTUAL_IRQ_COUNT		16

#ifdef ENABLE_PCIE_PORT0
/* PCIE port 0 */
#define MSTAR_PCIE_PORT0_MAC0		(0x3A0000*2)
#define MSTAR_PCIE_PORT0_BASE		(MSTAR_PCIE_REG_BASE + MSTAR_PCIE_PORT0_MAC0)
/* RC TOP */
#define MSTAR_PCIE_PORT0_RC0		(0x160C00*2)
#define MSTAR_PCIE_PORT0_RIU0_BASE	(MSTAR_PCIE_REG_BASE + MSTAR_PCIE_PORT0_RC0)

#define MSTAR_PCIE_PORT0_IRQ		E_IRQEXPL_PE2_RC

#define MSTAR_PCIE_PORT0_NAME		"Mstar-pcie-0"			/* mstar_pcie_port.name */
#define MSTAR_PCIE_PORT0_LANE		1						/* mstar_pcie_port.lane */


#define MSTAR_PCIE_PORT0_IOMEM_BASE		0x08000000UL
#define MSTAR_PCIE_PORT0_IOMEM_LEN		0x08000000UL		/* 128M */

/* USB 3.0 Phy */
#define MSTAR_PCIE_U3PHY_P0_DTOP_BASE   (MSTAR_PM_BASE + (0x00153400<<1))
#define MSTAR_PCIE_U3PHY_P0_ATOP_BASE	(MSTAR_PM_BASE + (0x00153600<<1))
#define MSTAR_PCIE_UTMISS1_P0_BASE      (MSTAR_PM_BASE + (0x00122380<<1))
#define MSTAR_PCIE_UPLL1_P0_BASE        (MSTAR_PM_BASE + (0x00100840<<1))

#endif

/* pcie analog tuning */
#define ATOP_GCR_USB3TX_MAIN_60 			0x32
#define ATOP_GCR_USB3TX_MAIN_PDRV0_61 		0x39

#define ATOP_GCR_USB3TX_LFPS_MAIN_64 		0x3C
#define ATOP_GCR_USB3TX_LFPS_MAIN_PDRV0_65 	0x00

#define ATOP_GCR_USB3TX_LFPS_PRE_66 		0x00
#define ATOP_GCR_USB3TX_LFPS_PRE_PDRV0_67 	0x00

#define ATOP_GCR_USB3TX_PRE_62			0x08
#define ATOP_GCR_USB3TX_PRE_PDRV0_63		0x39

 #define ATOP_TEST_SQH_68					0x01
 #define ATOP_EQ_A1					0x88

#define DTOP_TXDEEMPH_OVRID_EN			(1<<4)

#define UTMISS_REF_CLK_EN_2C				0x18
#define UTMISS_REF_CLK_EN_2B				0x03
#define UTMISS_REF_CLK_EN_2F				0x4A

#define DTOP_TX_SYNTH_SET_60				0x9375
#define DTOP_TX_SYNTH_STEP_61				0x0018
#define DTOP_TX_SYNTH_STEP_FRAC_62			0x7002
#define DTOP_TX_SYNTH_SPAN_63				0x04D8


/* these 3 options maybe m7821 only !? */
#define U3PHY_MCL16							/* m7821 U3 phy special case*/
#define XIU_IO_TIMEOUT_DISABLE				/* m7821 XIU timeout disable */
#define DOUBLE_BUFFER_ENABLE				/* m7821 ACPU double buffer */


#define ENABLE_L3_CACHE

#ifdef ENABLE_L3_CACHE
#define L3_AXI_00_ENABLE					/* ACPU */
#ifdef L3_AXI_00_ENABLE
#define L3_AXI_00_BANK			(MSTAR_PM_BASE + 0x310D00 * 2)
#define L3_AXI_00_OFFSET		(0x19 * 2)		/* 8 bit address */
#define L3_AXI_00_BIT			BIT(0)			/* enable bit */
#endif

/*#define L3_AXI_01_ENABLE*/				/* SCPU */
#ifdef L3_AXI_01_ENABLE
#define L3_AXI_01_BANK			(MSTAR_PM_BASE + 0x113C80 * 2)
#define L3_AXI_01_OFFSET		(0x37 * 2)		/* 8 bit address */
#define L3_AXI_01_BIT			BIT(1)			/* enable bit */
#endif

#endif

#if defined(XIU_IO_TIMEOUT_DISABLE)		/* xiu timeout disable */
#define XIU_IO_TIMEOUT_BANK		(MSTAR_PM_BASE + 0x310D00 * 2)
#define XIU_IO_TIMEOUT_OFFSET 	(0x0F * 2)
#define XIU_IO_TIMEOUT_VAL		0x0000
#endif

#if defined(DOUBLE_BUFFER_ENABLE)		/* double buffer for CPU registers latched & enabled */
#define DOUBLE_BUFFER_BANK	(MSTAR_PM_BASE + 0x310D00 * 2)
#define DOUBLE_BUFFER_OFFSET (0x18 * 2)
#define DOUBLE_BUFFER_BIT	BIT(0)
#endif


#define PCIE_POWERCTRL_GPIO_OFFSET 		0x78
#define PCIE_PERST_GPIO_OFFSET 			0x7A

#define PATCH_BAR					/* the BAR address reflects pcie2axi window setting, RC will not fit... needs to patch */
#define PATCH_SINGLE_DEVFN			/* pci kernel will scan all dev fn combination, this rc will response to some dev fn setting, needs to fix it to single dev fn to prevent kernel confuse with the other random setting */
/*#define PATCH_WED_XIU_BUS_COLLISION*/ /* no xiu access within interrupt handler */
/*#define PATCH_IRQ_DELAY*/				/* irq dispatch delay */
#define LEGACY_INDIRECT_MMIO_MODE 	/* legacy support of K7 outbound indirect mmio mode (MT7615 only) */

/* these 2 options exclude to each other!!! */
#define PATCH_RC_BUSNUM_00			/* RC bus number always 00 */
/*#define PATCH_RC_BUSNUM_POWER_00 */	/* RC number keep 0 before assigned(type1 cfg 18h) */

/*#define PCIE_TEST_MMIO_REGISTERS*/
#define PCIE_ENABLE_5G_LINK
/*#define PCIE_DUMP_REGISTERS */

/*#define PCIE_INT_AFTER_MEM_WRITE_ECO*/
/*#define PCIE_GEN1_LINK_ECO*/
/*#define PCIE_REFCLK_PERFORMANCE_ECO*/

#define PCIE_ENABLE_PPC					/* PCIE enable port power control */

#ifdef PATCH_WED_XIU_BUS_COLLISION
void pcie_set_no_xiu_access(u32 PortIdx, u32 val);
#endif

#endif	/* _PCIE_MSTAR_H */
