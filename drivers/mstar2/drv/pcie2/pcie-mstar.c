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




#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/ktime.h>
#include <asm/mach/pci.h>
/* #include <linux/phy/phy.h> */
#include "pcie-mstar.h"
#include "mstar/mstar_chip.h"

#ifdef CONFIG_MSTAR_ARM_BD_FPGA
#include "pcie-iic.h"
#endif


#define pcie_dbg(dev, fmt, args...) \
	do { if (PCIE_DEBUG) dev_err(dev , fmt , ## args); } while (0)
#define pcie_info(dev, fmt, args...) \
	dev_err(dev , fmt , ## args)
#define pcie_err(dev, fmt, args...) \
	dev_err(dev , fmt , ## args)


#if defined(MSTAR_MIU1_BUS_BASE)
/* #define MIU1_BUS_BASE_ADDR	MSTAR_MIU1_BUS_BASE */
#endif
#if defined(MSTAR_MIU2_BUS_BASE)
/* #define MIU2_BUS_BASE_ADDR	MSTAR_MIU2_BUS_BASE */
#endif


/* PCI CfgWr/CfgRd registers */
#define CFG_HEADER_0 					0x460
#define CFG_HEADER_1					0x464
#define CFG_HEADER_2					0x468
#define CFG_HEADER_3 					0x46c
#define CFG_WDATA						0x470
#define APP_TLP_REQ						0x488
#define APP_CFG_REQ						BIT(0)
#define APP_MSG_REQ						BIT(1)
#define APP_CPL_STATUS					GENMASK(7, 5)
#define CFG_RDATA						0x48c

/* PCI Interrupt registers */
#define INT_MASK 						0x420
#define WDMA_END_MASK					BIT(0)
#define WDMA_PERR_MASK					BIT(1)
#define WDMA_BERR_MASK					BIT(2)
#define RDMA_END_MASK					BIT(4)
#define RDMA_PERR_MASK					BIT(5)
#define RDMA_BERR_MASK					BIT(6)
#define P2B_WERR_MASK					BIT(8)
#define P2B_RERR_MASK					BIT(9)
#define P2B_WR_WIN0_MASK				BIT(10)
#define P2B_WR_WIN1_MASK				BIT(11)
#define B2P_RERR_MASK					BIT(13)
#define B2P_DISCARD_MASK				BIT(14)
#define INTA_MASK						BIT(16)
#define INTB_MASK						BIT(17)
#define INTC_MASK						BIT(18)
#define	INTD_MASK						BIT(19)
#define SERR_MASK						BIT(20)
#define PM_HP_EVENT_MASK				BIT(21)
#define AER_EVENT_MASK					BIT(22)
#define	MSI_MASK						BIT(23)
#define OBFF_IDLE_MASK					BIT(24)
#define OBFF_MASK						BIT(25)
#define CPU_ACTIVE_MASK					BIT(26)
#define LTR_MSG_MASK					BIT(27)
#define LTR_EN_MASK						BIT(28)
#define LEGACY_PM_CHG_MASK				BIT(29)
#define PCIE_L2_ENTRY_WAKE_MASK			BIT(30)
#define MPCIE_CFG_SOFT_MASK				BIT(31)

#ifdef CONFIG_PCIEAER
#define INTX_MASK						(INTA_MASK | 			\
						INTB_MASK | INTC_MASK | INTD_MASK | PCIE_L2_ENTRY_WAKE_MASK | AER_EVENT_MASK)

#else
#define INTX_MASK						(INTA_MASK | 			\
						INTB_MASK | INTC_MASK | INTD_MASK | PCIE_L2_ENTRY_WAKE_MASK)
#endif

#ifdef PCIE_DEBUG_INT_STATUS_STAGE2
#define INTX_MASK_2		(WDMA_PERR_MASK | WDMA_BERR_MASK |		\
						RDMA_PERR_MASK | RDMA_BERR_MASK |		\
						P2B_WERR_MASK | P2B_RERR_MASK	|		\
						B2P_RERR_MASK | B2P_DISCARD_MASK | 		\
						SERR_MASK | MPCIE_CFG_SOFT_MASK)
#endif

#define MSTAR_PCIE_INTX_SHIFT			16

#define INT_STATUS						0x424
#define WDMA_END_STATUS					BIT(0)
#define WDMA_PERR_STATUS				BIT(1)
#define WDMA_BERR_STATUS				BIT(2)
#define RDMA_END_STATUS					BIT(4)
#define RDMA_PERR_STATUS				BIT(5)
#define RDMA_BERR_STATUS				BIT(6)
#define P2B_WERR_STATUS					BIT(8)
#define P2B_RERR_STATUS					BIT(9)
#define P2B_WR_WIN0_STATUS				BIT(10)
#define P2B_WR_WIN1_STATUS				BIT(11)
#define B2P_RERR_STATUS					BIT(13)
#define B2P_DISCARD_STATUS				BIT(14)
#define INTA_STATUS						BIT(16)
#define INTB_STATUS						BIT(17)
#define INTC_STATUS						BIT(18)
#define INTD_STATUS						BIT(19)
#define PM_HP_EVENT_STATUS				BIT(21)
#define AER_EVENT_STATUS				BIT(22)
#define MSI_STATUS						BIT(23)
#define SERR_STATUS						BIT(20)
#define PM_HP_EVENT_STATUS				BIT(21)
#define AER_EVENT_STATUS				BIT(22)
#define OBFF_IDLE_STATUS				BIT(24)
#define OBFF_STATUS						BIT(25)
#define CPU_ACTIVE_STATUS				BIT(26)
#define LTR_MSG_STATUS					BIT(27)
#define LTR_EN_STATUS					BIT(28)
#define LEGACY_PM_CHG_STATUS			BIT(29)
#define PCIE_L2_ENTRY_WAKE_STATUS		BIT(30)
#define MPCIE_CFG_SOFT_STATUS			BIT(31)

#define IMSI_STATUS						0x42c

/* IP Configuration registers */
#define K_GBL_1 						0x000
#define PCIE_IS_RC_ROOT_PORT			(0x04UL<<12)
#define K_CONF_FUNC0_0					0x100
#define K_CONF_FUNC0_1					0x104
#define K_CONF_FUNC0_2					0x108
#define K_CONF_FUNC0_3					0x10C
#define K_CONF_FUNC0_3_MASK				0x0001FE00UL
#define _K_CONF_D1_SUP					BIT(9)
#define _K_CONF_D2_SUP					BIT(10)
#define _K_CONF_PME_SUP					(0x1FUL<<11)
#define _K_CONF_AER_EN					BIT(16)
#define SET_K_CONF_FUNC0_3				(_K_CONF_D1_SUP | \
					_K_CONF_D2_SUP | _K_CONF_PME_SUP | _K_CONF_AER_EN)

#define K_CONF_FUNC0_4					0x110
#define K_CONF_FUNC0_4_MASK				0xFE000000UL
#define _K_CONF_L1S_SUP					BIT(25)
#define _K_CONF_LS_EXIT_LAT_COM			(0x01UL<<26)
#define _K_CONF_LS_EXIT_LAT_SEP			(0x01UL<<29)
#define SET_K_CONF_FUNC0_4 				(_K_CONF_L1S_SUP | \
					_K_CONF_LS_EXIT_LAT_COM | _K_CONF_LS_EXIT_LAT_SEP)

#define K_CONF_FUNC0_12					0x130
#define K_RXL0S_EXIT_TIME_MASK			(0x3FUL << 8)
#define K_RXL0S_EXIT_TIME				(0x08UL << 8)

#define K_FC_VC0_0						0x030
#define SET_K_FC_VC0_0					0x02008020UL

#define K_FC_VC0_1						0x034
#define SET_K_FC_VC0_1					0x00000002UL

#define K_CNT_2							0x014
#define K_CNT_2_MASK					0x00FFF800UL
#define K_CNT_L0S_ENTRY_LAT				0x00
#define K_CNT_L1_ENTRY_LAT				0x00
#define SET_K_CNT_2						(K_CNT_L0S_ENTRY_LAT | K_CNT_L1_ENTRY_LAT)

#define MSTAR_ROOT_PORT					((4 << 12) & 0xf000)
#define MSTAR_VEND_ID					(0x14c3)
#define MSTAR_DEV_ID					(0x5396)
#define MSTAR_SS_VEND_ID				(0x14c3)
#define MSTAR_SS_DEV_ID					(0x5396)
#define MSTAR_RC_CLASS					(0x0604)
#define MSTAR_RC_REVISION				(0x0000)

#define K_CONF_IDS(vend, dev) \
	((vend & 0xffff) | ((dev << 16) \
	& 0xffff0000))

/* PCI MAC registers */
#define PCI_RSTCR 						0x510
#define PCI_PHY_RSTB					BIT(0)
#define PCI_PIPE_SRSTB					BIT(1)
#define PCI_MAC_SRSTB					BIT(2)
#define PCI_CRSTB						BIT(3)
#define PCI_PERSTB						BIT(8)
#define PCI_PIPE_RST_EN					BIT(13)
#define PCI_MAC_RST_EN					BIT(14)
#define PCI_CONF_RST_EN					BIT(15)
#define PCI_RST_DEASSERTED			    (PCI_PHY_RSTB | \
						PCI_PIPE_SRSTB | PCI_MAC_SRSTB | PCI_CRSTB)
#define PCI_LINKDOWN_RST_EN				(PCI_PIPE_RST_EN | \
								PCI_MAC_RST_EN | PCI_CONF_RST_EN)
#define PCI_WAKE_CONTROL				0x52c
#define CLKREQ_N_ENABLE					BIT(0)
#define PCI_LINK_STATUS					0x804
#define PCI_LINKUP						BIT(10)
#define IMSI_ADDR						0x430
#define MSI_VECTOR						0x0c0
#define MSI_IRQS						6
#define PCIE_PORT_MSI_BIT				32
#define MAX_MSI_IRQS					(MSI_IRQS + 1) /* 32 msi irqs for endpoint(b[31:0]) & 1 msi irq for pcie port(b32) */
#define INTX_IRQ_NUM					5
#define AHB2PCIE_BASE0_L				0x438
#define AHB2PCIE_BASE0_H				0x43c
#define AHB2PCIE_BASE1_L				0x440
#define AHB2PCIE_BASE1_H				0x444
#define PCIE2AXI_WIN					0x448
#define WIN_NOPREFETCH					BIT(6)
#define WIN_ENABLE						BIT(7)

#define CFG_BAR0						0x490
#define CFG_BAR1						0x494
#define CFG_BAR2						0x498
#define CFG_BAR3						0x49C
#define CFG_BAR4						0x4A0
#define CFG_BAR5						0x4A4


#define DBG_MODE_SEL					0x518

/* REG_PHYMAC_CONF */
#define REG_PHYMAC_CONF					0x528
#define REC_RXCFG_TXTS2_NUM_MASK		(0x7FUL << 16)
#define REC_RXCFG_TXTS2_NUM				(0x40UL << 16)


#define AHB2PCIE_BASEL(base)		    (base & 0xffffffff)
#define AHB2PCIE_BASEH(base)		    (base >> 32)
#define NOPREFETCH(flag)				((flag & IORESOURCE_PREFETCH) \
											? 0 : 1 << 6)
#define BASE_SIZE(sz)					(sz & 0x1f)

#define PCIE2AXI_SIZE					0x000FFFFFFFFF /* Translated AXI base address (36 bits) */

/* PCI Configuration Transaction Header */
#define CFG_DW0_LENGTH(length)		(length & 0x3ff)
#define CFG_DW0_ATTR(attr)			((attr << 12) & 0x3000)
#define CFG_DW0_EP(ep)		        ((ep << 14) & 0x4000)
#define CFG_DW0_TD(td)				((td << 15) & 0x8000)
#define CFG_DW0_TC(tc)				((tc << 20) & 0x700000)
#define CFG_DW0_TYPE(type)			((type << 24) & 0x1f000000)
#define CFG_DW0_FMT(fmt)			((fmt << 29) & 0xe0000000)
#define CFG_DW1_FBE(fbe)			(fbe & 0x0f)
#define CFG_DW1_LBE(lbe)			((lbe << 4) & 0xf0)
#define CFG_DW1_TAG(tag)		    ((tag << 8) & 0xff00)
#define CFG_DW1_RID(rid)			((rid << 16) & 0xffff0000)
#define CFG_DW2_REGN(regn)			((((regn >> 2) & 0x3f) << 2) | \
								(((regn >> 8) & 0x0f) << 8))
#define CFG_DW2_FUN(fun)		    ((fun << 16) & 0x070000)
#define CFG_DW2_DEV(dev)		    ((dev << 19) & 0xf80000)
#define CFG_DW2_BUS(bus)		    ((bus << 24) & 0xff000000)

#define  CFG_HEADER_DW0(ep, td, type, fmt) \
	(CFG_DW0_LENGTH(1) | CFG_DW0_ATTR(0) | \
	 CFG_DW0_EP(ep) | CFG_DW0_TD(td) | CFG_DW0_TC(0) | \
	  CFG_DW0_TYPE(type) | CFG_DW0_FMT(fmt))

#define  CFG_HEADER_DW1(fbe, tag, rid) \
	(CFG_DW1_FBE(fbe) | CFG_DW1_LBE(0) | \
	 CFG_DW1_TAG(tag) | CFG_DW1_RID(rid))

#define  CFG_HEADER_DW2(regn, fun, dev, bus) \
	(CFG_DW2_REGN(regn) | CFG_DW2_FUN(fun) | \
	 CFG_DW2_DEV(dev) | CFG_DW2_BUS(bus))


struct ms_pcie_irq_map_entry {
	bool used;
	unsigned int irq;
};

static struct ms_pcie_irq_map_entry ms_pcie_irq_map[MSTAR_PCIE_VIRTUAL_IRQ_COUNT];

#ifdef CONFIG_PCI_MSI
struct msi_map_entry {
	bool used;
	u8 index;
	unsigned int irq;
};

#define MSI_MAP_SIZE  (MAX_MSI_IRQS)
#endif

/**
 * struct mstar_pcie_port - PCIe port information
 * @name: PCIe port name
 * @base: IO Mapped Register Base
 * @irq: Interrupt number
 * @port: Port number
 * @lane: Lane count
 * @devfn: Device/Function number
 * @pcie: PCIe host info pointer
 * @irq_domain:  IRQ domain pointer
 * @msi_irq_domain:  MSI IRQ domain pointer
 */
struct mstar_pcie_port {
	char *name;
	int 	portnum;
	void __iomem *base;
	void __iomem *riu_base;
	void __iomem *phyDTOP_base;
	void __iomem *phyATOP_base;
	void __iomem *utmiss_base;
	void __iomem *upll_base;
	unsigned int irq;
	unsigned int pin_irqs[4];
	u32 busnum;
	u32 lane;
#ifdef PATCH_SINGLE_DEVFN
	int devfn;
#endif
#ifdef CONFIG_PCI_MSI
	struct msi_map_entry msi_map[MSI_MAP_SIZE];
	int rp_msi_irq;
#endif
	int Cfg1_18h;
	u32 status;
	ktime_t t_power_on;

	uintptr_t	iomem_base;
	uintptr_t	iomem_len;

	struct device *dev;
	struct resource *bus_range;
	struct resource *io_res;
	struct resource pref_mem;
	struct resource npref_mem;
	struct list_head resources;
	u32 power_ctrl[2];
	u32 perst_ctrl[2];
	u32 clk_src;
	u32 eq;
#if 0 /* MSTAR_PORTING */
	struct irq_domain *irq_domain;
	struct irq_domain *msi_irq_domain;
	DECLARE_BITMAP(msi_irq_in_use, MSI_IRQS);
#endif
#ifdef PATCH_WED_XIU_BUS_COLLISION
	int	no_xiu_access;
#endif
};

struct mstar_pcie_port g_pcie_ports[] = {
	[0] = {
		.name = MSTAR_PCIE_PORT0_NAME,
		.portnum = 0,
		.irq = MSTAR_PCIE_PORT0_IRQ,
		.lane = MSTAR_PCIE_PORT0_LANE,
		.iomem_base = MSTAR_PCIE_PORT0_IOMEM_BASE,
		.iomem_len = MSTAR_PCIE_PORT0_IOMEM_LEN,
#ifdef PATCH_SINGLE_DEVFN
		.devfn = PCI_DEVFN(0, 0),				/* port 0, dev=0, fn=0 */
#endif
		.Cfg1_18h = 0x00000000,				/* default bus address 0 */
		.status	= 0,
	},
#ifdef ENABLE_PCIE_PORT1
	[1] = {
		.name = MSTAR_PCIE_PORT1_NAME,
		.portnum = 1,
		.irq = MSTAR_PCIE_PORT1_IRQ,
		.lane = MSTAR_PCIE_PORT1_LANE,
		.iomem_base = MSTAR_PCIE_PORT1_IOMEM_BASE,
		.iomem_len = MSTAR_PCIE_PORT1_IOMEM_LEN,
#ifdef PATCH_SINGLE_DEVFN
		.devfn = PCI_DEVFN(1, 0),				/* port 1, dev=1, fn=0 */
#endif
		.Cfg1_18h = 0x00000000,				/* default bus address 0 */
		.status = 0,
	},
#endif
};
#define PCIE_PORT_COUNT	ARRAY_SIZE(g_pcie_ports)

static struct platform_device Mstar_pcie_0_device = {
	.name = MSTAR_PCIE_PORT0_NAME,
	.id = 0,
};

#ifdef ENABLE_PCIE_PORT1
static struct platform_device Mstar_pcie_1_device = {
	.name = MSTAR_PCIE_PORT1_NAME,
	.id = 1,
};
#endif

static struct platform_device *Mstar_pcie_platform_devices[] = {
	&Mstar_pcie_0_device,
#ifdef ENABLE_PCIE_PORT1
	&Mstar_pcie_1_device,
#endif
};

struct delayed_work	g_pcie_late_init;
#define PICE_LATE_INIT_DELAY		100


/* MStar PCIe RC TOP registers */
#define REG_SRST_TO_MAC_TOP				0x00
#define REG_DESIGN_OPTION				0x01
#define REG_BIT_LTSSM_EN				BIT(2)
#define REG_BIT_L3_FLUSH_ENABLE			BIT(7)
#define REG_BIT_MASK_INBWRT_AXIBVLD_EN  BIT(8)
#define REG_BIT_MI_DYNAMIC_REQUEST		BIT(9)

#define REG_05							0x05
#define REG_TEST_BUS_1B					0x1B
#define REG_BIT_DEBUG_MASK_DISABLE		BIT(5)

#define REG_36							0x36
#define REG_BIT_DELAY_FLUSH_REQ			BIT(13)
#define REG_BIT_MAC_SRAM_SD_EN			BIT(14)
#define REG_BIT_BRG_SRAM_SD_EN			BIT(15)

#define REG_64BIT_WFUL_ON				0x39
#define REG_BIT_64BIT_WFUL_ON			BIT(1)

#define REG_MIU0_START_ADDR0			0x47
#define REG_MIU0_START_ADDR1			0x48
#define REG_MIU0_START_ADDR2			0x49
#define REG_MIU0_END_ADDR0				0x4A
#define REG_MIU0_END_ADDR1				0x4B
#define REG_MIU0_END_ADDR2				0x4C

#define REG_MIU1_START_ADDR0			0x4D
#define REG_MIU1_START_ADDR1			0x4E
#define REG_MIU1_START_ADDR2			0x4F
#define REG_MIU1_END_ADDR0				0x50
#define REG_MIU1_END_ADDR1				0x51
#define REG_MIU1_END_ADDR2				0x52

#define REG_MIU2_START_ADDR0			0x53
#define REG_MIU2_START_ADDR1			0x54
#define REG_MIU2_START_ADDR2			0x55
#define REG_MIU2_END_ADDR0				0x56
#define REG_MIU2_END_ADDR1				0x57
#define REG_MIU2_END_ADDR2				0x58

#define REG_MIU0_TLB_START_ADDR0		0x59
#define REG_MIU0_TLB_START_ADDR1		0x5A
#define REG_MIU0_TLB_START_ADDR2		0x5B
#define REG_MIU0_TLB_END_ADDR0			0x5C
#define REG_MIU0_TLB_END_ADDR1			0x5D
#define REG_MIU0_TLB_END_ADDR2			0x5E

#define REG_MIU1_TLB_START_ADDR0		0x5F
#define REG_MIU1_TLB_START_ADDR1		0x60
#define REG_MIU1_TLB_START_ADDR2		0x61
#define REG_MIU1_TLB_END_ADDR0			0x62
#define REG_MIU1_TLB_END_ADDR1			0x63
#define REG_MIU1_TLB_END_ADDR2			0x64

#define REG_MIU2_TLB_START_ADDR0		0x65
#define REG_MIU2_TLB_START_ADDR1		0x66
#define REG_MIU2_TLB_START_ADDR2		0x67
#define REG_MIU2_TLB_END_ADDR0			0x68
#define REG_MIU2_TLB_END_ADDR1			0x69
#define REG_MIU2_TLB_END_ADDR2			0x6A


#define REG_AXI2MIU_SETTING				0x74
#define REG_BVALID_LDZ_OPT_MASK			0x03
#define REG_BVALID_DEPENDS_ON_LDZ		0x01
#define REG_MIU_ENABLE_MASK				0x1C
#define REG_BIT_MIU0_ENABLE				BIT(2)
#define REG_BIT_MIU1_ENABLE				BIT(3)
#define REG_BIT_MIU2_ENABLE				BIT(4)
#define REG_BIT_AXI2MI_SRST				BIT(7)

#define REG_CPU2PCIE_ADDR_SEG             0x76
#define REG_ACPU_TO_PCIE_ADR_31_28_MASK   0x0000000F
#define REG_SCPU_TO_PCIE_ADR_31_28_MASK   0x000000F0
#define REG_NOE0_TO_PCIE_ADR_31_28_MASK   0x00000F00
#define REG_NOE1_TO_PCIE_ADR_31_28_MASK   0x0000F000


static void mstar_pcie_irq_map_init(void)
{
	int i;

	pr_info("[PCIE] virtual irq start: %d, count %d\n",
		MSTAR_PCIE_VIRTUAL_IRQ_BASE, MSTAR_PCIE_VIRTUAL_IRQ_COUNT);
	for (i = 0; i < MSTAR_PCIE_VIRTUAL_IRQ_COUNT; i++) {
		ms_pcie_irq_map[i].irq = MSTAR_PCIE_VIRTUAL_IRQ_BASE + i;
		ms_pcie_irq_map[i].used = false;
	}
}

static unsigned int mstar_pcie_irq_alloc(void)
{
	int i;

	for (i = 0; i < MSTAR_PCIE_VIRTUAL_IRQ_COUNT; i++) {
		if (!ms_pcie_irq_map[i].used) {
			ms_pcie_irq_map[i].used = true;
			pr_info("[PCIE] alloc virtual irq %d\n", ms_pcie_irq_map[i].irq);
			return ms_pcie_irq_map[i].irq;
		}
	}

	printk(KERN_ERR "[PCIE] alloc virtual irq failed...\n");
	return 0;
}

#ifdef CONFIG_PCI_MSI
static void mstar_pcie_irq_release(unsigned int irq)
{
	int i;

	for (i = 0; i < MSTAR_PCIE_VIRTUAL_IRQ_COUNT; i++) {
		if (ms_pcie_irq_map[i].used &&  ms_pcie_irq_map[i].irq == irq) {
			pr_info("[PCIE] release virtual irq %d\n", ms_pcie_irq_map[i].irq);
			ms_pcie_irq_map[i].used = false;
			return;
		}
	}

	printk(KERN_ERR "[PCIE] release virtual irq %d failed...\n", irq);
}

static void msi_map_init(struct mstar_pcie_port *port)
{
	struct msi_map_entry *msi_map = port->msi_map;
	int i;

	pcie_dbg(port->dev, "msi_map_init");
	for (i = 0; i < MSI_MAP_SIZE; i++) {
		msi_map[i].used = false;
		msi_map[i].index = i;
		msi_map[i].irq = 0;
	}
	port->rp_msi_irq = 0; 				/* rootport service msi irq number */
}

static struct msi_map_entry *msi_map_get(struct mstar_pcie_port *port)
{
	struct msi_map_entry *msi_map = port->msi_map;
	struct msi_map_entry *retval = NULL;
	int i;
	unsigned int irq_num;

	for (i = 0; i < MSI_MAP_SIZE; i++) {
		if (!msi_map[i].used) {
			irq_num = mstar_pcie_irq_alloc();
			if (irq_num == 0)
				break;

			retval = msi_map + i;
			retval->irq = irq_num;
			retval->used = true;
			break;
		}
	}

	return retval;
}

void msi_map_release(struct msi_map_entry *entry)
{
	if (entry) {
		mstar_pcie_irq_release(entry->irq);
		entry->used = false;
		entry->irq = 0;
	}
}

static unsigned int msi_map_get_irq(struct mstar_pcie_port *port, int index)
{
	struct msi_map_entry *msi_map = port->msi_map;
	int i;

	for (i = 0; i < MSI_MAP_SIZE; i++) {
		if (msi_map[i].used && (msi_map[i].index == index)) {
			return msi_map[i].irq;
		}
	}

	return 0;
}
#endif

static inline u32 pcie_read(struct mstar_pcie_port *port, u32 reg)
{
	return readl(port->base + reg);
}

static inline void pcie_write(struct mstar_pcie_port *port, u32 val, u32 reg)
{
	/* pcie_info(port->dev, "pcie xiu write: offset: %x, val: %x\n", reg, val); */
	writel(val, port->base + reg);
}

static inline u16 pcie_riu_readw(struct mstar_pcie_port *port, u16 reg)
{
	/* reg is 16bits offset value. */
	return readw(port->riu_base + reg*4);
}

static inline void pcie_riu_writew(struct mstar_pcie_port *port, u16 val, u16 reg)
{
	/* pcie_info(port->dev, "pcie riu write: offset: %x, val: %x\n", reg, val); */
	writew(val, port->riu_base + reg*4);
}

static inline bool mstar_pcie_link_is_up(struct mstar_pcie_port *port)
{
	return (pcie_read(port, PCI_LINK_STATUS) &
		PCI_LINKUP) ? 1 : 0;
}

static int mstar_pcie_mapping_sz(u64 size)
{
	int i = 0;

	while (size) {
		size >>= 1;
		i++;
	}
	return i;
}

/**
 * mstar_pcie_check_size - Check if cfg size is valid
 * @szie: PCI cfgrd/cfgwr size
 * @where: PCI cfgrd/cfgwr register number
 * @ByteEn: byte enable field in PCI cfgrd/cfgwr header
 *
 * Return: 'true' on success and 'false' if cfg size is invalid
 */
static bool mstar_pcie_check_size(u32 size, u32 where, u32 *ByteEn)
{
	int i = 0;

	/* cfgrd/cfgwr must be on a DWORD boundary */
	if (size + (where & 3) > 4)
		return false;

	/* find the value for the 1st DW Byte Enables */
	*ByteEn = 0;
	while (i < size) {
		*ByteEn |= 1 << i;
		i++;
	}
	*ByteEn <<= (where & 0x3);

	return true;
}

static int mstar_pcie_check_cfg_cpld(struct mstar_pcie_port *port)
{
	int 	count = 10000;
	u32	val;
	int delay = 10;
	while (1) {
		/* 000: Success ,001: UR ,010: CRS ,100: CA ,111: Completion Timeout */
		val = pcie_read(port, APP_TLP_REQ);

		if (!(val & APP_CFG_REQ)) {
			if (val & APP_CPL_STATUS) {
				pcie_err(port->dev, "check config status %x\n", val);
				return PCIBIOS_SET_FAILED;
			}
			return PCIBIOS_SUCCESSFUL;
		}
		if (!count--) {
			pcie_err(port->dev, "check config timeout\n");
			return PCIBIOS_SET_FAILED;
		}
		udelay(delay);
	}
}

static int mstar_pcie_hw_rd_cfg(struct mstar_pcie_port *port, u32 bus, u32 devfn,
	int where, int size, u32 *val)
{
	int byte_enable = 0, reg, shift = 8 * (where & 3);

	*val = 0;
	/* check if cfgwr data is on a DWORD boundary */
	if (!mstar_pcie_check_size(size, where, &byte_enable))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	/* write PCI Configuration Transaction Header for cfgrd */
	pcie_write(port, CFG_HEADER_DW0(0, 0, 4, 0), CFG_HEADER_0);
	pcie_write(port, CFG_HEADER_DW1(byte_enable, 0, 0), CFG_HEADER_1);
	pcie_write(port, CFG_HEADER_DW2(where, PCI_FUNC(devfn),
		PCI_SLOT(devfn), bus), CFG_HEADER_2);
	/* triget h/w to transmit cfgrd TLP */
	reg = pcie_read(port, APP_TLP_REQ);
	pcie_write(port, reg | APP_CFG_REQ, APP_TLP_REQ);
	/* Check complete condition */
	if (mstar_pcie_check_cfg_cpld(port))
		return PCIBIOS_SET_FAILED;
	/* read cpld payload of cfgrd */
	*val = pcie_read(port, CFG_RDATA);

	if (size == 1)
		*val = (*val >> shift) & 0xff;
	else if (size == 2)
		*val = (*val >> shift) & 0xffff;
	else if (size == 3)
		*val = (*val >> shift) & 0xffffff;

	return PCIBIOS_SUCCESSFUL;
}

static int mstar_pcie_hw_wr_cfg(struct mstar_pcie_port *port, u32 bus, u32 devfn,
	int where, int size, u32 val)

{
    int byte_enable, reg, _val, shift = 8 * (where & 3);

	/* check if cfgwr data is on a DWORD boundary */
	if (!mstar_pcie_check_size(size, where, &byte_enable))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	/* write PCI Configuration Transaction Header for cfgwr */
	pcie_write(port, CFG_HEADER_DW0(0, 0, 4, 2), CFG_HEADER_0);
	pcie_write(port, CFG_HEADER_DW1(byte_enable, 0, 0), CFG_HEADER_1);
	pcie_write(port, CFG_HEADER_DW2(where, PCI_FUNC(devfn),
		PCI_SLOT(devfn), bus), CFG_HEADER_2);

	/* write cfgwr data */
	_val = val << shift;
	pcie_write(port, _val, CFG_WDATA);
	/* triget h/w to transmit cfgwr TLP */
	reg = pcie_read(port, APP_TLP_REQ);
	pcie_write(port, reg | APP_CFG_REQ, APP_TLP_REQ);
	/* Check complete condition */
	return mstar_pcie_check_cfg_cpld(port);
}

#ifdef LEGACY_INDIRECT_MMIO_MODE
static int mstar_pcie_hw_rd_mmio(struct mstar_pcie_port *port, u32 bus, u32 devfn,
	int where, int size, u32 *val)
{
	int byte_enable = 0, reg, shift = 8 * (where & 3);

	*val = 0;
	/* check if cfgwr data is on a DWORD boundary */
	if (!mstar_pcie_check_size(size, where, &byte_enable))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	/* write PCI Configuration Transaction Header for cfgrd */
	pcie_write(port, CFG_HEADER_DW0(0, 0, 0, 0), CFG_HEADER_0);			/*MMIO read 1 DWORD*/
	pcie_write(port, CFG_HEADER_DW1(byte_enable, 0, 0), CFG_HEADER_1);
	pcie_write(port, where, CFG_HEADER_2);
	/* triget h/w to transmit cfgrd TLP */
	reg = pcie_read(port, APP_TLP_REQ);
	pcie_write(port, reg | APP_CFG_REQ, APP_TLP_REQ);
	/* Check complete condition */
	if (mstar_pcie_check_cfg_cpld(port))
		return PCIBIOS_SET_FAILED;
	/* read cpld payload of cfgrd */
	*val = pcie_read(port, CFG_RDATA);

	if (size == 1)
		*val = (*val >> shift) & 0xff;
	else if (size == 2)
		*val = (*val >> shift) & 0xffff;
	else if (size == 3)
		*val = (*val >> shift) & 0xffffff;

	return PCIBIOS_SUCCESSFUL;
}

static int mstar_pcie_hw_wr_mmio(struct mstar_pcie_port *port, u32 bus, u32 devfn,
	int where, int size, u32 val)

{
    int byte_enable, reg, _val, shift = 8 * (where & 3);

	/* check if cfgwr data is on a DWORD boundary */
	if (!mstar_pcie_check_size(size, where, &byte_enable))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	/* write PCI Configuration Transaction Header for cfgwr */
	pcie_write(port, CFG_HEADER_DW0(0, 0, 0, 2), CFG_HEADER_0);			/*MMIO write 1 DWORD*/
	pcie_write(port, CFG_HEADER_DW1(byte_enable, 0, 0), CFG_HEADER_1);
	pcie_write(port, where, CFG_HEADER_2);

	/* write cfgwr data */
	_val = val << shift;
	pcie_write(port, _val, CFG_WDATA);
	/* triget h/w to transmit cfgwr TLP */
	reg = pcie_read(port, APP_TLP_REQ);
	pcie_write(port, reg | APP_CFG_REQ, APP_TLP_REQ);
	/* Check complete condition */
	return mstar_pcie_check_cfg_cpld(port);
}

int mstar_ob_mmio_read(struct pci_bus *bus, u32 bn, unsigned int devfn,
				   int where, int size, u32 *val)
{
	struct mstar_pcie_port *port = (struct mstar_pcie_port *) ((struct pci_sys_data *)bus->sysdata)->private_data;
	u32 ioBase, ioLen = MSTAR_PCIE_PORT0_IOMEM_LEN;

	if (port->portnum == 0)
		ioBase = MSTAR_PCIE_PORT0_IOMEM_BASE;
	else
#ifdef ENABLE_PCIE_PORT1
	if (port->portnum == 1)
		ioBase = MSTAR_PCIE_PORT1_IOMEM_BASE;
	else
#endif
	{
		ioBase = 0; 	/* illeagle base address */
		*val = 0xFFFFFFFF;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/* For compatible with previous interface, use bit31 for enable memory IO and offset is not full address */
	if ((where >= INDIRECT_FLAG) && (where < ioBase))
		where = (where & 0x7FFFFFFF) + ioBase;

	if ((where < ioBase) || (where >= (ioBase+ioLen))) {
		pcie_dbg(port->dev, "[PCIE] ob mmio read with illegal offset 0x%08x !\n", where);
		*val = 0xFFFFFFFF;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return mstar_pcie_hw_rd_mmio(port, bn, devfn, where, size, val);

	/*
	if (size == 1)
		*val = readb((void *)where);
	else
	if (size == 2)
		*val = readw((void *)where);
	else
		*val = readl((void *)where);

	return PCIBIOS_SUCCESSFUL;
	*/
}

int mstar_ob_mmio_write(struct pci_bus *bus, u32 bn, unsigned int devfn,
				   int where, int size, u32 val)
{
	struct mstar_pcie_port *port = (struct mstar_pcie_port *) ((struct pci_sys_data *)bus->sysdata)->private_data;
	u32 ioBase, ioLen = MSTAR_PCIE_PORT0_IOMEM_LEN;

	if (port->portnum == 0)
		ioBase = MSTAR_PCIE_PORT0_IOMEM_BASE;
	else
#ifdef ENABLE_PCIE_PORT1
	if (port->portnum == 1)
		ioBase = MSTAR_PCIE_PORT1_IOMEM_BASE;
	else
#endif
	{
		ioBase = 0; 	/* illeagle base address */
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/* For compatible with previous interface, use bit31 for enable memory IO and offset is not full address */
	if ((where >= INDIRECT_FLAG) && (where < ioBase))
		where = (where & 0x7FFFFFFF) + ioBase;

	if ((where < ioBase) || (where >= (ioBase+ioLen))) {
		pcie_dbg(port->dev, "[PCIE] ob mmio write with illegal offset 0x%08x !\n", where);
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return mstar_pcie_hw_wr_mmio(port, bn, devfn, where, size, val);

	/*
	if (size == 1)
		writeb((u8)val, (void *)where);
	else
	if (size == 2)
		writew((u16)val, (void *)where);
	else
		writel(val, (void *)where);

	return PCIBIOS_SUCCESSFUL;
	*/
}
#endif

/**
 * mstar_pcie_read_config - Read configuration space
 * @bus: PCI Bus structure
 * @devfn: Device/function
 * @where: Offset from base
 * @size: Byte/word/dword
 * @val: Value to be read
 *
 * Return: PCIBIOS_SUCCESSFUL on success
 *	   PCIBIOS_DEVICE_NOT_FOUND on failure
 */
static int mstar_pcie_read_config(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 *val)
{
	u32 bn = bus->number;
	struct mstar_pcie_port *port = (struct mstar_pcie_port *) ((struct pci_sys_data *)bus->sysdata)->private_data;
	int ret;
#ifdef PATCH_RC_BUSNUM_00
	u32 bn_tmp;
#endif

	/* device config read fast return failed if PCIE phy link is not established */
	if ((bn != port->busnum) && !(port->status & PCIE_RC_STATUS_PHY_LINKED)) {
		*val = 0xFFFFFFFF;
		pcie_dbg(port->dev, "ConfigRead [%d:%d:%d] offset: 0x%x, size: 0x%x, val: 0x%x skipped\n",
			bn, PCI_SLOT(devfn), PCI_FUNC(devfn), where, size, *val);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

#ifdef PATCH_SINGLE_DEVFN
	/* exclude all other null functions, */
	/* this IP will return dirty data if pci is scan other slot(dev), fn. */
	/* this path will */
	if ((bn == (port->busnum))&& (devfn != port->devfn)) {
		*val = 0xFFFFFFFF;
		pcie_dbg(port->dev, "[PCIE] Read Cfg error, bn %x, port->busnum %x, devfn %x\n", bn, port->busnum, devfn);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
#endif

#ifdef LEGACY_INDIRECT_MMIO_MODE
	if ((u32)where > 0x1000) {		/* indirect mode, address > 4096 */
		ret = mstar_ob_mmio_read(bus, bn, devfn, where, size, val);
		pcie_dbg(port->dev, "MMIO Read [%d:%d:%d] offset: 0x%x, size: 0x%x, val: 0x%x\n",
					bn, PCI_SLOT(devfn), PCI_FUNC(devfn), where, size, *val);
		return ret;
	}
#endif	/* LEGACY_INDIRECT_MMIO_MODE */

#ifdef PATCH_RC_BUSNUM_00
	/*
		RC power on default bus address 0,
		PRINTK_WITHOUT_KERN_LEVELif primary bus # is not being changed,
		use bus0 as default.
	*/
	bn_tmp = bn;
	if ((bn == port->busnum) && (bn != 0))
		bn = 0;
#endif


#ifdef PATCH_RC_BUSNUM_POWER_00
	/*
		RC power on default bus address 0,
		if primary bus # is not being changed,
		use bus0 as default.
	*/
	if ((bn == port->busnum) && (bn != 0) && (port->Cfg1_18h & 0xFF) == 0x00) {
		pcie_dbg(port->dev, "[PCIE] port%d rd CfgSpace[18h] = %x\n", port->portnum, port->Cfg1_18h);
		bn = 0;
	}
#endif

	ret = mstar_pcie_hw_rd_cfg(port, bn, devfn, where, size,  val);

#ifdef PATCH_RC_BUSNUM_00
	bn = bn_tmp;
	if ((bn == port->busnum) && (bn != 0) && (where == 0x18))
		*val = port->Cfg1_18h;
#endif

	if (ret)
		*val = 0xffffffff;

#ifdef PATCH_BAR
	/*  BAR0, BAR1 is related to PCI2AXI_WIN setting, only EP use this function.
		in RC mode, needs to be patched by software.
		patch BAR 0 to return no use any io memory.	*/
	if ((bn == port->busnum) && (devfn == port->devfn) && (where == 0x10)) {
		pcie_dbg(port->dev, "[PCIE] before Patch BAR 0:%08X\n", *val);
		ret = PCIBIOS_SUCCESSFUL;
		*val = 0xFFFFFFFF;	/* 64bits BAR, non-prefetchable memory */
	}
	/* PATCH BAR 1 to return no use any io memory */
	if ((bn == port->busnum) && (devfn == port->devfn) && (where == 0x14)) {
		pcie_dbg(port->dev, "[PCIE] before Patch BAR 1:%08X\n", *val);
		ret = PCIBIOS_SUCCESSFUL;
		*val = 0xFFFFFFFF;
	}

#endif

	pcie_dbg(port->dev, "ConfigRead [%d:%d:%d] offset: 0x%x, size: 0x%x, val: 0x%x\n",
		bn, PCI_SLOT(devfn), PCI_FUNC(devfn), where, size, *val);

	return ret;
}

/**
 * mstar_pcie_write_config - Write configuration space
 * @bus: PCI Bus structure
 * @devfn: Device/function
 * @where: Offset from base
 * @size: Byte/word/dword
 * @val: Value to be written to device
 *
 * Return: PCIBIOS_SUCCESSFUL on success
 *	   PCIBIOS_DEVICE_NOT_FOUND on failure
 */
static int mstar_pcie_write_config(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 val)
{
	u32 bn = bus->number;
	struct mstar_pcie_port *port = (struct mstar_pcie_port *) ((struct pci_sys_data *)bus->sysdata)->private_data;
	int ret;
#if defined(PATCH_RC_BUSNUM_POWER_00) || defined(PATCH_RC_BUSNUM_00)
	u32 bn_tmp;
#ifdef PATCH_RC_BUSNUM_00
	u32 val_tmp;
#endif
#endif

	/* device config write fast return failed if PCIE phy link is not established */
	if ((bn != port->busnum) && !(port->status & PCIE_RC_STATUS_PHY_LINKED)) {
		pcie_dbg(port->dev, "ConfigWrite [%d:%d:%d] offset: 0x%x, size: 0x%x, val: 0x%x skipped\n",
			bn, PCI_SLOT(devfn), PCI_FUNC(devfn), where, size, val);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

#ifdef LEGACY_INDIRECT_MMIO_MODE
	if ((u32)where > 0x1000) {		/* indirect mode, address > 4096 */
		ret = mstar_ob_mmio_write(bus, bn, devfn, where, size, val);
		pcie_dbg(port->dev, "MMIO Write [%d:%d:%d] offset: 0x%x, size: 0x%x, val: 0x%x\n",
						bn, PCI_SLOT(devfn), PCI_FUNC(devfn), where, size, val);
		return ret;
	}
#endif	/* LEGACY_INDIRECT_MMIO_MODE */


#ifdef PATCH_BAR
	/*  BAR0, BAR1 is related to PCI2AXI_WIN setting, only EP use this function.
		in RC mode, needs to be patched by software.
		patch BAR 0 to return no use any io memory. */
	if ((bn == port->busnum) && (devfn == port->devfn) && (where == 0x10)) {
		pcie_dbg(port->dev, "[PCIE]Write 0 before Patch BAR0:%08X\n", val);
		val = 0x00000000;
	}
	/* PATCH BAR 1 to return no use any io memory */
	if ((bn == port->busnum) && (devfn == port->devfn) && (where == 0x14)) {
		pcie_dbg(port->dev, "[PCIE]Write 0 before Patch BAR1:%08X\n", val);
		val = 0x00000000;
	}

#endif

#ifdef PATCH_RC_BUSNUM_00
	/*
		use bus0 as default.
	*/
	bn_tmp = bn;
	val_tmp = val;
	if ((bn == port->busnum) && (bn != 0)) {
		bn = 0;
		if (where == 0x18)
			val &= 0xFFFFFF00;
	}
#endif

#ifdef PATCH_RC_BUSNUM_POWER_00
	/*
		RC power on default bus address 0,
		if primary bus # is not being changed,
		use bus0 as default.
	*/
	bn_tmp = bn;	/* backup bus number */
	if ((bn == port->busnum) && (bn != 0) && (port->Cfg1_18h & 0xFF) ==  0x00) {
		pcie_dbg(port->dev, "[PCIE] port%d wr CfgSpace[18h] = %x\n", port->portnum, port->Cfg1_18h);
		if (where == 0x18) {
			pcie_dbg(port->dev, "[PCIE] 18h write %x, force RC bus = 0\n", val);
			val &= 0xFFFFFF00;			/* force RC bus # = 0 */
		}
		bn = 0;
	}
#endif

	pcie_dbg(port->dev, "ConfigWrite [%d:%d:%d] offset: 0x%x, size: 0x%x, val: 0x%x\n",
		bn, PCI_SLOT(devfn), PCI_FUNC(devfn), where, size, val);

	ret = mstar_pcie_hw_wr_cfg(port, bn, devfn, where, size, val);

#ifdef PATCH_RC_BUSNUM_00
	/*
		if RC->primary bus # is being changed, update that bus primary #..etc
	*/
	bn = bn_tmp;		/* restore bus number */
	if ((bn == port->busnum) && (where == 0x18) && (ret == PCIBIOS_SUCCESSFUL))
		port->Cfg1_18h = val_tmp;
#endif

#ifdef PATCH_RC_BUSNUM_POWER_00
	/*
		if RC->primary bus # is being changed, update that bus primary #..etc
	*/
	bn = bn_tmp;		/* restore bus number */
	if ((bn == port->busnum) && (where == 0x18) && (ret == PCIBIOS_SUCCESSFUL)) {
		port->Cfg1_18h = val;
		pcie_dbg(port->dev, "[PCIE] port%d update CfgSpace[18h] = %x\n", port->portnum, val);
	}
#endif

	return ret;
}


/* PCIe operations */
static struct pci_ops mstar_pcie_ops = {
	.read  = mstar_pcie_read_config,
	.write = mstar_pcie_write_config,
};

#ifdef CONFIG_PCI_MSI
/* HW Interrupt Chip Descriptor */
static struct irq_chip mstar_msi_irq_chip = {
	.name = "MStar PCIe MSI",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

/* called by arch_setup_msi_irqs in drivers/pci/msi.c */
int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int retval = -EINVAL;
	struct msi_msg msg;
	struct msi_map_entry *map_entry = NULL;
	struct mstar_pcie_port *port = (struct mstar_pcie_port *) ((struct pci_sys_data *)pdev->bus->sysdata)->private_data;

	map_entry = msi_map_get(port);
	if (map_entry == NULL)
		goto exit;

	retval = irq_alloc_desc(map_entry->irq);
	if (retval < 0)
		goto exit;
	irq_set_chip_and_handler(map_entry->irq,
				&mstar_msi_irq_chip,
				handle_simple_irq);

	retval = irq_set_msi_desc(map_entry->irq, desc);
	if (retval < 0)
		goto exit;
	set_irq_flags(map_entry->irq, IRQF_VALID);

	msg.address_lo = (u32)((u64)port->base + MSI_VECTOR);
	/* 32 bit address only */
	msg.address_hi = 0;
	msg.data = map_entry->index;

	write_msi_msg(map_entry->irq, &msg);

	/* patch for root port MSI (PCIE root port service irq) */
	if ((pdev->bus->number == port->busnum) && (pdev->devfn == port->devfn)) {
		pcie_dbg(&pdev->dev, "setup rootport MSI irq=%d\n", map_entry->irq);
		port->rp_msi_irq = map_entry->irq;
	}

	retval = 0;
exit:
	if (retval != 0) {
		if (map_entry) {
			irq_free_desc(map_entry->irq);
			msi_map_release(map_entry);
		}
	}

	return retval;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	struct msi_map_entry *msi_map;
	int i;

	msi_map = g_pcie_ports[0].msi_map;

	g_pcie_ports[0].rp_msi_irq = 0;

	for (i = 0; i < MSI_MAP_SIZE; i++) {
		if ((msi_map[i].used) && (msi_map[i].irq == irq)) {
			irq_free_desc(msi_map[i].irq);
			msi_map_release(msi_map + i);
			break;
		}
	}

#ifdef ENABLE_PCIE_PORT1
	msi_map = g_pcie_ports[1].msi_map;

	g_pcie_ports[1].rp_msi_irq = 0;

	for (i = 0; i < MSI_MAP_SIZE; i++) {
		if ((msi_map[i].used) && (msi_map[i].irq == irq)) {
			irq_free_desc(msi_map[i].irq);
			msi_map_release(msi_map + i);
			break;
		}
	}
#endif

}
#endif

#ifdef CONFIG_PCI_MSI
/**
 * mstar_pcie_enable_msi - Enable MSI support
 * @port: PCIe port information
 */
static void mstar_pcie_enable_msi(struct mstar_pcie_port *port)
{
#ifdef CONFIG_PCI_MSI
	u32 mask;

	pcie_write(port, (u32)((u64)port->base + MSI_VECTOR), IMSI_ADDR);
	mask = pcie_read(port, INT_MASK);
	mask &= ~MSI_MASK;
	pcie_write(port, mask, INT_MASK);
#endif
}
#endif

#ifdef PATCH_WED_XIU_BUS_COLLISION
/**
	\fn void pcie_set_no_xiu_access(u32 val)
*/
void pcie_set_no_xiu_access(u32 PortIdx, u32 val)
{
	/* setting xiu access action during INT */
	/* PortIdx == 0 for port0 */
	/* PortIdx != 0 for port1 */
	/* true - disable, false - enable */

	if (PortIdx == 0) {
		g_pcie_ports[0].no_xiu_access = val;
		if (val) {
			disable_irq(g_pcie_ports[0].irq);
		} else {
			enable_irq(g_pcie_ports[0].irq);
		}

	} else {
#ifdef ENABLE_PCIE_PORT1
		g_pcie_ports[1].no_xiu_access = val;
		if (val) {
			disable_irq(g_pcie_ports[1].irq);
		} else {
			enable_irq(g_pcie_ports[1].irq);
		}
#endif
	}
}
EXPORT_SYMBOL(pcie_set_no_xiu_access);
#endif
/**
 * mstar_pcie_intr_handler - Interrupt Service Handler
 * @irq: IRQ number
 * @data: PCIe port information
 *
 * Return: IRQ_HANDLED on success and IRQ_NONE on failure
 */
static irqreturn_t mstar_pcie_intr_handler(int irq, void *data)
{
	struct mstar_pcie_port *port = (struct mstar_pcie_port *)data;
	u32 mask, status;
	int index = 0;
	unsigned long val;
#ifdef CONFIG_PCI_MSI
	int pos, msi;
#endif


#ifdef PATCH_WED_XIU_BUS_COLLISION

	/* !!! special patch of no access xiu during INT only for mt7615 !!! */

	if (port->no_xiu_access) {
#ifdef PCIE_DEBUG_IRQ
		pcie_dbg(port->dev, "got a INTx with index %d - xiu patch walk around\n", index);
#endif
		generic_handle_irq(port->pin_irqs[0]);  /* mt7615 INTA */
		return IRQ_HANDLED;
	}
#endif


	/* read interrupt decode and mask registers */
	val = pcie_read(port, INT_STATUS);
	mask = pcie_read(port, INT_MASK);

#ifdef PCIE_DEBUG_IRQ
	pcie_dbg(port->dev, "pcie isr, val: 0x%x, mask: 0x%x\n", (u32)val, mask);
#endif

	status = val & ~mask;
	if (!status)
		return IRQ_NONE;

	pcie_write(port, status, INT_STATUS);

	if (status & INTX_MASK) {
		/* handle INTx Interrupt */
		val = ((val & INTX_MASK) >>
			MSTAR_PCIE_INTX_SHIFT);
		while (val) {
			val >>= 1;
			index++;
		}

#ifdef PCIE_DEBUG_IRQ
		pcie_dbg(port->dev, "got a INTx with index %d\n", index);
#endif
#ifdef PATCH_IRQ_DELAY
		mdelay(10);		/* debug only */
#endif
		generic_handle_irq(port->pin_irqs[index-1]);
	}
	if (status & MSI_STATUS) {
		val = pcie_read(port, IMSI_STATUS);
#ifdef CONFIG_PCI_MSI
		while (val > 0) {
			pos = find_first_bit(&val, 32);
			msi = msi_map_get_irq(port, pos);
			/* write back to clear bit (w1c)*/
			pcie_write(port, 1 << pos, IMSI_STATUS);
			val &= ~(1 << pos);

			if (msi != 0)
				generic_handle_irq(msi);
		}
#else
		pcie_err(port->dev, "[PCIE] get a msi intr in msi disable mode\n");
		pcie_write(port, val, IMSI_STATUS);
#endif
	}
#if 0 /* MSTAR_PORTING FIXME */
	/* PCIe port uses the same msi irq for PME, AER and HP*/
	if ((status & PM_HP_EVENT_STATUS) ||
				(status & AER_EVENT_STATUS)) {
		msi = irq_find_mapping(port->msi_irq_domain, PCIE_PORT_MSI_BIT);
		generic_handle_irq(msi);
	}
#endif
	if (status & PCIE_L2_ENTRY_WAKE_STATUS) {
		/* de-assert pe reset */
		if (!(pcie_read(port, PCI_RSTCR) & PCI_PERSTB))
			pcie_write(port, pcie_read(port, PCI_RSTCR) |
				PCI_PERSTB, PCI_RSTCR);
		/* wake up system from L2 state */
		val = pcie_read(port, PCI_WAKE_CONTROL);
		pcie_write(port, val | CLKREQ_N_ENABLE, PCI_WAKE_CONTROL);
		pcie_write(port, val & ~CLKREQ_N_ENABLE, PCI_WAKE_CONTROL);
	}

#ifdef CONFIG_PCIEAER
	if (status & AER_EVENT_STATUS) {
		pcie_dbg(port->dev, "%x - AER error\n", (u32)AER_EVENT_STATUS);
#ifdef CONFIG_PCI_MSI
		if (port->rp_msi_irq)
			generic_handle_irq(port->rp_msi_irq);
		else {
			pcie_info(port->dev, "Err, no MSI irq\n");
		}
#endif
	}
#endif

	/* clear the Interrupt Decode register */
	/*pcie_write(port, status, INT_STATUS);*/

#ifdef PCIE_DEBUG_INT_STATUS_STAGE2
	if (status & INTX_MASK_2) {
			pcie_dbg(port->dev, "irq status 0x%08x - ", status);
		if (status & WDMA_PERR_STATUS) {
			pcie_dbg(port->dev, "%x - wdma ended with error\n", (u32)WDMA_PERR_STATUS);
		} else if (status & WDMA_BERR_STATUS) {
			pcie_dbg(port->dev, "%x - error occurs on AHB/AXI bus wr data\n", (u32)WDMA_BERR_STATUS);
		} else if (status & RDMA_PERR_STATUS) {
			pcie_dbg(port->dev, "%x - DMA with errors on PCIe\n", (u32)RDMA_PERR_STATUS);
		} else if (status & RDMA_BERR_STATUS) {
			pcie_dbg(port->dev, "%x - error occurs on AHB/AXI bus rd data\n", (u32)RDMA_BERR_STATUS);
		} else if (status & P2B_WERR_STATUS) {
			pcie_dbg(port->dev, "%x - request from pcie cannot post on AHB/AXI bus\n", (u32)P2B_WERR_STATUS);
		} else if (status & P2B_RERR_STATUS) {
			pcie_dbg(port->dev, "%x - request from pcie cannot read data from AHB/AXI bus\n", (u32)P2B_RERR_STATUS);
		} else if (status & B2P_RERR_STATUS) {
			pcie_dbg(port->dev, "%x - request from bus cannot be read from pcie\n", (u32)B2P_RERR_STATUS);
		} else if (status & (u32)B2P_DISCARD_STATUS) {
			pcie_dbg(port->dev, "%x - AHB_TIMER cycle ended, remaining data is flushed\n", (u32)B2P_DISCARD_STATUS);
		} else if (status & (u32)SERR_STATUS) {
			pcie_dbg(port->dev, "%x - system error on root port\n", (u32)SERR_STATUS);
		} else if (status & MPCIE_CFG_SOFT_STATUS) {
			pcie_dbg(port->dev, port->dev, "%x - MPCIe enter cfg.soft.stats\n", (u32)MPCIE_CFG_SOFT_STATUS);
		} else {
			pcie_dbg(port->dev, "unknown status\n");
		}
	}
#endif

	return IRQ_HANDLED;
}

/**
 * mstar_pcie_config_init
 * @port: pcie port information
 * return
*/
void mstar_pcie_config_init(struct mstar_pcie_port *port)
{
	struct device *dev = port->dev;

	pcie_info(dev, "PCIE configuration init...\n");
	/* set PCI_K_CONF_FUNC0_0 VID/PID */
	pcie_write(port, (MSTAR_DEV_ID << 16) + MSTAR_VEND_ID, K_CONF_FUNC0_0);
	/* set PCI_K_CONF_FUNC0_1 Class/Revision */
	pcie_write(port, (MSTAR_RC_CLASS << 16) + MSTAR_RC_REVISION, K_CONF_FUNC0_1);
	/* set PCI_K_CONF_FUNC0_2 susbys VID/PID */
	pcie_write(port, (MSTAR_DEV_ID << 16) + MSTAR_VEND_ID, K_CONF_FUNC0_2);
	/* set PCI_K_CONF_FUNC0_3 */
	pcie_write(port, (pcie_read(port, K_CONF_FUNC0_3) & ~K_CONF_FUNC0_3_MASK) \
						| SET_K_CONF_FUNC0_3, K_CONF_FUNC0_3);
	/* set PCI_K_CONF_FUNC0_4 1 lane */
	pcie_write(port, (pcie_read(port, K_CONF_FUNC0_4) & ~K_CONF_FUNC0_4_MASK) \
						| SET_K_CONF_FUNC0_4, K_CONF_FUNC0_4);
	/* Set PCI_K_CONF_FUNC0_12 ? */
	pcie_write(port, (pcie_read(port, K_CONF_FUNC0_12) & ~K_RXL0S_EXIT_TIME_MASK) \
						| K_RXL0S_EXIT_TIME, K_CONF_FUNC0_12);
	/* set PCI_K_FC_VC0_0 */
	pcie_write(port, SET_K_FC_VC0_0, K_FC_VC0_0);
	/* set PCI_K_FC_VC0_1 */
	pcie_write(port, SET_K_FC_VC0_1, K_FC_VC0_1);
	/* set PCI_K_CNT_2 */
	pcie_write(port, (pcie_read(port, K_CNT_2) & ~K_CNT_2_MASK) | SET_K_CNT_2, K_CNT_2);
	/* Set PCI_REG_PHYMAC_CONF ? */
	pcie_write(port, (pcie_read(port, REG_PHYMAC_CONF) & ~REC_RXCFG_TXTS2_NUM_MASK) | REC_RXCFG_TXTS2_NUM, REG_PHYMAC_CONF);
}




static void mstar_pcie_rc_init(struct mstar_pcie_port *port)
{
	u64   tVal;
/*
	// init cpu to pcie base addr
	regVal = readw(port->riu_base +  REG_CPU2PCIE_ADDR_SEG*4);
	printk("[PCIE] cpu to pcie address MSB[31:28](default): %04X\n", regVal & 0xFFFF);
	writew(0x00EE, port->riu_base +  REG_CPU2PCIE_ADDR_SEG*4);
	printk("[PCIE] cpu to pcie address MSB[31:28]         : %04X\n", readw(port->riu_base +  REG_CPU2PCIE_ADDR_SEG*4) & 0xFFFF);
*/
	/* init miu0 address */
	tVal = MIU0_BUS_BASE_ADDR >> 20;
	pcie_riu_writew(port, tVal, REG_MIU0_START_ADDR0);
	pcie_riu_writew(port, tVal, REG_MIU0_TLB_START_ADDR0);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU0_START_ADDR1);
	pcie_riu_writew(port, tVal, REG_MIU0_TLB_START_ADDR1);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU0_START_ADDR2);
	pcie_riu_writew(port, tVal, REG_MIU0_TLB_START_ADDR2);

	tVal = (MIU0_BUS_BASE_ADDR + MIU0_BUS_LENGTH) >> 20;
	pcie_riu_writew(port, tVal, REG_MIU0_END_ADDR0);
	pcie_riu_writew(port, tVal, REG_MIU0_TLB_END_ADDR0);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU0_END_ADDR1);
	pcie_riu_writew(port, tVal, REG_MIU0_TLB_END_ADDR1);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU0_END_ADDR2);
	pcie_riu_writew(port, tVal, REG_MIU0_TLB_END_ADDR2);

#if MIU1_ENABLE
	if ((MIU0_BUS_BASE_ADDR + MIU0_BUS_LENGTH) > MIU1_BUS_BASE_ADDR) {
		pr_err("[PCIE] error: MIU0 range overlap with MIU1\n");
	}
#endif

	/* enable miu0 */
	if (MIU0_ENABLE) {
		pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING) | REG_BIT_MIU0_ENABLE, \
				REG_AXI2MIU_SETTING);
	} else {
		pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING) & ~REG_BIT_MIU0_ENABLE, \
				REG_AXI2MIU_SETTING);
	}

#ifdef PCIE_DEBUG_RC_TOP
	pcie_dbg(port->dev, "[PCIE]: MIU0 base =%lx\n", MIU0_BUS_BASE_ADDR);
	pcie_dbg(port->dev, "[PCIE] AXI2MIU_SETTING=%08X\n", pcie_riu_readw(port, REG_AXI2MIU_SETTING));

	pcie_dbg(port->dev, "[PCIE] MIU0_START_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU0_START_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU0_START_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU0_START_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU0_START_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU0_START_ADDR2));
	pcie_dbg(port->dev, "[PCIE] MIU0_END_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU0_END_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU0_END_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU0_END_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU0_END_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU0_END_ADDR2));

	pcie_dbg(port->dev, "[PCIE] MIU0_TLB_START_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU0_TLB_START_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU0_TLB_START_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU0_TLB_START_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU0_TLB_START_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU0_TLB_START_ADDR2));
	pcie_dbg(port->dev, "[PCIE] MIU0_TLB_END_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU0_TLB_END_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU0_TLB_END_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU0_TLB_END_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU0_TLB_END_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU0_TLB_END_ADDR2));
#endif

	/* init miu1 address */
/* #if defined(MIU1_BUS_BASE_ADDR) */
	tVal = MIU1_BUS_BASE_ADDR>>20;
	pcie_riu_writew(port, tVal, REG_MIU1_START_ADDR0);
	pcie_riu_writew(port, tVal, REG_MIU1_TLB_START_ADDR0);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU1_START_ADDR1);
	pcie_riu_writew(port, tVal, REG_MIU1_TLB_START_ADDR1);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU1_START_ADDR2);
	pcie_riu_writew(port, tVal, REG_MIU1_TLB_START_ADDR2);


	tVal = ((u64)MIU1_BUS_BASE_ADDR + (u64)MIU1_BUS_LENGTH)>>20;
	pcie_riu_writew(port, tVal, REG_MIU1_END_ADDR0);
	pcie_riu_writew(port, tVal, REG_MIU1_TLB_END_ADDR0);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU1_END_ADDR1);
	pcie_riu_writew(port, tVal, REG_MIU1_TLB_END_ADDR1);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU1_END_ADDR2);
	pcie_riu_writew(port, tVal, REG_MIU1_TLB_END_ADDR2);

#if MIU2_ENABLE
	if ((MIU1_BUS_BASE_ADDR + MIU1_BUS_LENGTH) > MIU2_BUS_BASE_ADDR) {
		pr_err("[PCIE] error: MIU1 range overlap with MIU2\n");
	}
#endif

	/* enable miu1 */
	if (MIU1_ENABLE) {
		pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING) | REG_BIT_MIU1_ENABLE, \
				REG_AXI2MIU_SETTING);
	} else {
		pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING) & ~REG_BIT_MIU1_ENABLE, \
				REG_AXI2MIU_SETTING);
	}
/* #endif */

#ifdef PCIE_DEBUG_RC_TOP
	pcie_dbg(port->dev, "[PCIE] MIU1_START_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU1_START_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU1_START_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU1_START_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU1_START_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU1_START_ADDR2));
	pcie_dbg(port->dev, "[PCIE] MIU1_END_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU1_END_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU1_END_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU1_END_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU1_END_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU1_END_ADDR2));

	pcie_dbg(port->dev, "[PCIE] MIU1_TLB_START_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU1_TLB_START_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU1_TLB_START_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU1_TLB_START_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU1_TLB_START_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU1_TLB_START_ADDR2));
	pcie_dbg(port->dev, "[PCIE] MIU1_TLB_END_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU1_TLB_END_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU1_TLB_END_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU1_TLB_END_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU1_TLB_END_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU1_TLB_END_ADDR2));
#endif



	/* init miu2 address */
/* #if defined(MIU2_BUS_BASE_ADDR) */
	tVal = MIU2_BUS_BASE_ADDR >> 20;
	pcie_riu_writew(port, tVal, REG_MIU2_START_ADDR0);
	pcie_riu_writew(port, tVal, REG_MIU2_TLB_START_ADDR0);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU2_START_ADDR1);
	pcie_riu_writew(port, tVal, REG_MIU2_TLB_START_ADDR1);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU2_START_ADDR2);
	pcie_riu_writew(port, tVal, REG_MIU2_TLB_START_ADDR2);

	tVal = (MIU2_BUS_BASE_ADDR + MIU2_BUS_LENGTH)>>20;
	pcie_riu_writew(port, tVal, REG_MIU2_END_ADDR0);
	pcie_riu_writew(port, tVal, REG_MIU2_TLB_END_ADDR0);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU2_END_ADDR1);
	pcie_riu_writew(port, tVal, REG_MIU2_TLB_END_ADDR1);
	tVal >>= 16;
	pcie_riu_writew(port, tVal, REG_MIU2_END_ADDR2);
	pcie_riu_writew(port, tVal, REG_MIU2_TLB_END_ADDR2);

	/* enable miu2 */
	if (MIU2_ENABLE) {
		pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING) | REG_BIT_MIU2_ENABLE, \
				REG_AXI2MIU_SETTING);
	} else {
		pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING) & ~REG_BIT_MIU2_ENABLE, \
				REG_AXI2MIU_SETTING);
	}
/* #endif */

#ifdef PCIE_DEBUG_RC_TOP
	pcie_dbg(port->dev, "[PCIE] MIU2_START_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU2_START_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU2_START_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU2_START_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU2_START_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU2_START_ADDR2));
	pcie_dbg(port->dev, "[PCIE] MIU2_END_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU2_END_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU2_END_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU2_END_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU2_END_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU2_END_ADDR2));

	pcie_dbg(port->dev, "[PCIE] MIU2_TLB_START_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU2_TLB_START_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU2_TLB_START_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU2_TLB_START_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU2_TLB_START_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU2_TLB_START_ADDR2));
	pcie_dbg(port->dev, "[PCIE] MIU2_TLB_END_ADDR0 %x\n", pcie_riu_readw(port, REG_MIU2_TLB_END_ADDR0));
	pcie_dbg(port->dev, "[PCIE] MIU2_TLB_END_ADDR1 %x\n", pcie_riu_readw(port, REG_MIU2_TLB_END_ADDR1));
	pcie_dbg(port->dev, "[PCIE] MIU2_TLB_END_ADDR2 %x\n", pcie_riu_readw(port, REG_MIU2_TLB_END_ADDR2));
#endif

	/*pcie_riu_writew(port, pcie_riu_readw(port, REG_AXI2MIU_SETTING)
						| REG_BIT_AXI2MI_SRST, REG_AXI2MIU_SETTING);
	  printk("[PCIE] AXI2MIU_SETTING %08x\n", pcie_riu_readw(port, REG_AXI2MIU_SETTING));
	*/

	/* set 64bit address */
	pcie_riu_writew(port, REG_BIT_64BIT_WFUL_ON, REG_64BIT_WFUL_ON);

	pcie_dbg(port->dev, "[PCIE] 64bit wful on %x\n", pcie_riu_readw(port, REG_64BIT_WFUL_ON));

#if defined(PCIE_INT_AFTER_MEM_WRITE_ECO)
	pcie_info(port->dev, "[PCIE]Mask InBWr to AXIBVLID\n");
	pcie_riu_writew(port, pcie_riu_readw(port, REG_DESIGN_OPTION) | REG_BIT_MASK_INBWRT_AXIBVLD_EN, REG_DESIGN_OPTION);
	/*pcie_info(port->dev, "[PCIE]BValid depends on LDZ\n");*/
	/*pcie_riu_writew(port, (pcie_riu_readw(port, REG_AXI2MIU_SETTING) & ~REG_BVALID_LDZ_OPT_MASK) | REG_BVALID_DEPENDS_ON_LDZ, REG_AXI2MIU_SETTING);*/
#endif


#ifdef CONFIG_MSTAR_ARM_BD_FPGA
	/* for debug purpose, set debug mask on, testbus select 0x01 */
	pcie_riu_writew(port, (!REG_BIT_DEBUG_MASK_DISABLE)+REG_TEST_BUS_1B, REG_05);

	/* if cpu clock is slow, enable this bit to accomodate the cpu clock. */
	pcie_riu_writew(port, pcie_riu_readw(port, REG_36) | REG_BIT_DELAY_FLUSH_REQ, REG_36);
#endif /* CONFIG_MSTAR_ARM_BD_FPGA */

#ifdef ENABLE_L3_CACHE
	/* ltssem, L3, MI Dynamic */
	pcie_dbg(port->dev, "[PCIE] ltssm enabled, MI Dynamic request, L3 cache enable\n");
#ifdef L3_AXI_00_ENABLE
	/* acpu pcie flush enable, 16bit address, bit 8 */
	writeb(L3_AXI_00_BIT | readb((void *)(L3_AXI_00_BANK) + (L3_AXI_00_OFFSET * 2) - (L3_AXI_00_OFFSET & 0x01)), 	\
			(void *)((L3_AXI_00_BANK) + (L3_AXI_00_OFFSET * 2) - (L3_AXI_00_OFFSET & 0x01)));
	pcie_dbg(port->dev, "[PCIE] acpu L3 flush enabled\n");
#endif

	/* scpu pcie flush enable, 16bit address, bit 1 */
	/*
	writeb(0x02 | readb((void *)((MSTAR_PM_BASE+0x113C80*2)+0x37*4)), (void *)((MSTAR_PM_BASE+0x113C80*2)+0x37*4));
	*/

#ifdef L3_AXI_01_ENABLE
	writeb(L3_AXI_01_BIT | readb((void *)(L3_AXI_01_BANK + (L3_AXI_01_OFFSET * 2) - (L3_AXI_01_OFFSET & 0x01))), 	\
			(void *)((L3_AXI_01_BANK) + (L3_AXI_01_OFFSET * 2) - (L3_AXI_01_OFFSET & 0x01)));
	pcie_dbg(port->dev, "[PCIE] scpu L3 flush enabled\n");
#endif

	/* NOE pcie flush enable, 16bit address */
	/*
	writew(0,      (void *)((MSTAR_PM_BASE+0x100A00*2)+0x72*4));
	writew(0,      (void *)((MSTAR_PM_BASE+0x100A00*2)+0x73*4));
	writew(0x4000, (void *)((MSTAR_PM_BASE+0x121200*2)+0x0A*4));
	writew(0x4000, (void *)((MSTAR_PM_BASE+0x121280*2)+0x0A*4));
	*/
	pcie_riu_writew(port, pcie_riu_readw(port, REG_DESIGN_OPTION) \
		| (REG_BIT_LTSSM_EN + REG_BIT_L3_FLUSH_ENABLE + REG_BIT_MI_DYNAMIC_REQUEST), REG_DESIGN_OPTION);
#else
	/* ltssem MI Dynamic */
	pcie_dbg(port->dev, "[PCIE] ltssm enabled & MI Dynamic request\n");

	pcie_riu_writew(port, pcie_riu_readw(port, REG_DESIGN_OPTION) \
		| (REG_BIT_LTSSM_EN + REG_BIT_MI_DYNAMIC_REQUEST), REG_DESIGN_OPTION);
#endif


    /* disable xiu timeout, maybe m7821 only !? */
#if defined(XIU_IO_TIMEOUT_DISABLE)
	writew(XIU_IO_TIMEOUT_VAL, (void *)(XIU_IO_TIMEOUT_BANK + (XIU_IO_TIMEOUT_OFFSET * 2) - (XIU_IO_TIMEOUT_OFFSET & 0x01)));
	pcie_dbg(port->dev, "[PCIE] XIU IO Timeout disabled\n");
#endif

    /* double buffer setting, for above setting latched & enabled, maybe m7821 only !?*/
#if defined(DOUBLE_BUFFER_ENABLE)
	writeb(DOUBLE_BUFFER_BIT | readb((void *)(DOUBLE_BUFFER_BANK) + (DOUBLE_BUFFER_OFFSET * 2) - (DOUBLE_BUFFER_OFFSET & 0x01)), 	\
			(void *)((DOUBLE_BUFFER_BANK) + (DOUBLE_BUFFER_OFFSET * 2) - (DOUBLE_BUFFER_OFFSET & 0x01)));
	udelay(1);
	/* clear double enable bit */
	writeb((~DOUBLE_BUFFER_BIT) & readb((void *)(DOUBLE_BUFFER_BANK) + (DOUBLE_BUFFER_OFFSET * 2) - (DOUBLE_BUFFER_OFFSET & 0x01)),		\
			(void *)((DOUBLE_BUFFER_BANK) + (DOUBLE_BUFFER_OFFSET * 2) - (DOUBLE_BUFFER_OFFSET & 0x01)));

	pcie_dbg(port->dev, "[PCIE] acpu double buffer enabled\n");
#endif


	/*  XIU
		pcie_write(port, (0xE0000000 + 0x40 + 0x1C), AHB2PCIE_BASE0_L);
		pcie_write(port, 0, AHB2PCIE_BASE0_H);
	*/

}
#ifndef CONFIG_MSTAR_ARM_BD_FPGA
static void mstar_phy_tuning(struct mstar_pcie_port *port)
{

	uintptr_t pU3phyDtopBase = (uintptr_t)port->phyDTOP_base;
	uintptr_t pU3phyAtopBase = (uintptr_t)port->phyATOP_base;
	uintptr_t pUtmissBase = (uintptr_t)port->utmiss_base;
	uintptr_t pUpllBase = (uintptr_t)port->upll_base;

#ifdef PCIE_GEN1_LINK_ECO
    /* PLDA_mac_en*/
    writeb(readb((void *)(pU3phyDtopBase + 0x04 * 2)) | (1<<6), \
										(void *) (pU3phyDtopBase + 0x04 * 2));
    /* PCIE_GEN1_TO_FIFO_EN */
    writew(readw((void *)(pU3phyDtopBase + 0x1A * 2)) | (1<<15), \
										(void *) (pU3phyDtopBase + 0x1A * 2));
#endif
#ifdef PCIE_REFCLK_PERFORMANCE_ECO
    writew(readw((void *)(pUtmissBase + 0x3E * 2)) | (1<<13), \
										(void *) (pUtmissBase + 0x3E * 2));
#endif
#ifdef PCIE_AUTO_RTERM_ECO
	/* PCIE_GEN1_TO_FIFO_EN */
	writew(readw((void *)(pU3phyDtopBase + 0xFE * 2 )) | (1<<12), \
						  (void *) (pU3phyDtopBase + 0xFE * 2));
#endif

	writew((ATOP_GCR_USB3TX_MAIN_60) | (ATOP_GCR_USB3TX_MAIN_PDRV0_61 << 8), \
						(void *)(pU3phyAtopBase + 0x60 * 2));

	/*Tx current */
	writew((ATOP_GCR_USB3TX_LFPS_MAIN_64) | (ATOP_GCR_USB3TX_LFPS_MAIN_PDRV0_65 << 8), \
						(void *)(pU3phyAtopBase + 0x64 * 2));
	/*Tx current */
	writew((ATOP_GCR_USB3TX_LFPS_PRE_66) | (ATOP_GCR_USB3TX_LFPS_PRE_PDRV0_67 << 8), \
						(void *)(pU3phyAtopBase + 0x66 * 2));

	/* Set ATOP+0x34[0] to 1b1 (16-bit addressing) */
	writeb(ATOP_TEST_SQH_68 , (void *) (pU3phyAtopBase + 0x68 * 2));
	writew((ATOP_GCR_USB3TX_PRE_62) | (ATOP_GCR_USB3TX_PRE_PDRV0_63 << 8),  \
						(void *)(pU3phyAtopBase + 0x62 * 2));

	writeb(readb((void *)(pU3phyDtopBase + 0x3D * 2 - 1)) | (DTOP_TXDEEMPH_OVRID_EN), \
			(void *)(pU3phyDtopBase + 0x3D * 2 - 1));

	/* Tune REFCLK */
	writeb(UTMISS_REF_CLK_EN_2C, (void *) (pUtmissBase + 0x2C * 2));          /* set TX Swing bit[6:4] for better REFCLK+- */
	writeb(UTMISS_REF_CLK_EN_2B, (void *) (pUtmissBase + 0x2B * 2 - 1));     /* set TX reference bit[1:0] +50mv */
	writeb(port->clk_src, (void *) (pUtmissBase + 0x2F * 2 - 1));     /*  [30] & [25] to enable PCIE REF clock */

	/* overwrite EQ @PCIE mode in FT/verification/normal function */
	writeb(port->eq, (void *)(pU3phyAtopBase + 0xA1 * 2 - 1));/* TEST_RXPLL[47]=1 for turning on PD demux */

	/* msleep(100); */
}


#if defined(U3PHY_MCL16)
static void Mstar_U3phy_MCL16_init(uintptr_t pU3phyDtopBase, uintptr_t pU3phyAtopBase, uintptr_t pUtmissBase, uintptr_t pUpllBase)
{
	printk("[PCIE] U3 phy MCL_16nm init\n");
    /*========================================= */
    /*        power on							*/
    /*========================================= */
    writeb(0x00, (void *)(pU3phyAtopBase + 0xa1 * 2 - 1)); 	/* disable overwirte eq-str */
    writew(0x0000, (void *)(pU3phyAtopBase + 0x02 * 2)); 	/* power on tx atop */
    writew(0x0000, (void *)(pU3phyAtopBase + 0x00 * 2)); 	/* power on rx atop */

    writew(0x01f2, (void *)(pU3phyDtopBase + 0x12 * 2)); 	/* reg_tx_lock_value */
    writeb(0x09, (void *)(pU3phyAtopBase + 0xa2 * 2));
    writeb(0x80, (void *)(pU3phyAtopBase + 0x02 * 2)); 		/* reg_pd_usb30tx[7:0] */
    writeb(0x00, (void *)(pU3phyAtopBase + 0x04 * 2)); 		/* [0]reg_pd_rxpll_ldo [1]reg_pd_bg [2]reg_pd_portsw */
    writeb(0x01, (void *)(pU3phyAtopBase + 0x80 * 2)); 		/* [0]reg_en_usb3rxpll_phd_ov_en   */
    writeb(0x01, (void *)(pU3phyAtopBase + 0x82 * 2)); 		/* [0]reg_en_usb3rxpll_phd_ov_value*/
    writeb(0x00, (void *)(pU3phyAtopBase + 0x27 * 2 - 1)); 	/* [8]reg_test_vring */

    writew(0x1181, (void *)(pU3phyDtopBase + 0x08 * 2)); 	/* lock value setting */

    /* 2. TX & RX SYNTH SETING */
    writew(0x5002, (void *)(pU3phyDtopBase + 0xc4 * 2));
    writew(0x04D0, (void *)(pU3phyDtopBase + 0xc6 * 2));
    writew(0x0004, (void *)(pU3phyDtopBase + 0xc8 * 2));
    writew(0x1e4f, (void *)(pU3phyDtopBase + 0xc0 * 2));
    writew(0x0016, (void *)(pU3phyDtopBase + 0xc2 * 2));
    writew(0x1e4f, (void *)(pU3phyDtopBase + 0xe0 * 2));
    writew(0x0016, (void *)(pU3phyDtopBase + 0xe2 * 2));
    writew(0x0000, (void *)(pU3phyDtopBase + 0xe4 * 2));
    writew(0x0000, (void *)(pU3phyDtopBase + 0xe6 * 2));
    writew(0x0004, (void *)(pU3phyDtopBase + 0xe8 * 2));

    /* 3. enable PIPE3 interface */
    writew(0x0560, (void *)(pU3phyDtopBase + 0x18 * 2));

    /* 4. Release PHY PD overrade enable */
    writew(0x0000, (void *)(pU3phyAtopBase + 0x3a * 2));

    /* 5. clkgen enable */
    writew(0x0000, (void *)(pU3phyDtopBase + 0x20 * 2));
    writew(0x0000, (void *)(pU3phyDtopBase + 0x22 * 2));

    /* rx_lock_pcie_value */
    writew(0x07d1, (void *)(pU3phyDtopBase + 0x0a * 2));




	/*-------------------------------------------------------*/
#if 1 /* CONFIG_MSTAR_K6 */
/* APHY+0xf1 [3] to 1b1.*/
	writeb(0x08, (void *)(pU3phyDtopBase + 0xF1 * 2 - 1));
/* ([4]=reg_dedicate_pcie_phy) */
	writeb(0x10, (void *)(pU3phyDtopBase + 0x84 * 2));
#endif


	writeb(0x00, (void *)(pU3phyDtopBase + 0x03 * 2 - 1));
	writeb(0xC0, (void *)(pU3phyDtopBase + 0x08 * 2));
	writeb(0x10, (void *)(pU3phyDtopBase + 0x09 * 2-1));

	writew(0x0000, (void *)(pU3phyAtopBase + 0x3A * 2));
	writew(0x0560, (void *)(pU3phyDtopBase + 0x18 * 2));
	writew(0x0309, (void *)(pU3phyAtopBase + 0x3A * 2));
	writeb(readb((void *)(pU3phyAtopBase + 0x03 * 2 - 1)) & 0xbb, \
							(void *)(pU3phyAtopBase + 0x03 * 2 - 1));

	writew(0x0401,  (void *)(pU3phyAtopBase + 0x12 * 2));

		/* -------- New for MS28 --------- */

	writeb(readb((void *)(pU3phyAtopBase + 0x0F * 2 - 1)) & (u8)(~0x04), \
							(void *)(pU3phyAtopBase + 0x0F * 2 - 1));

		/* ------- Add by PCIe PHY SW setting from USB3 PHY ---------*/
		/* rx lock */
	writeb(0xF1, (void *)(pU3phyDtopBase + 0x0A * 2));
	writeb(0x01, (void *)(pU3phyDtopBase + 0x0B * 2 - 1));
		/* tx lock */
	writeb(0xF1, (void *)(pU3phyDtopBase + 0x12 * 2));
	writeb(0x01, (void *)(pU3phyDtopBase + 0x13 * 2 - 1));

		/* -------- PCIe Setting --------- */
	writeb(0xDF, (void *)(pU3phyDtopBase + 0xB4 * 2));

	writeb(readb((void *)(pU3phyDtopBase + 0x3A * 2)) | 0x02, \
					(void *)(pU3phyDtopBase + 0x3A * 2));

	writeb(0x04, (void *) (pU3phyDtopBase + 0x1E*2));
	writeb(0x80, (void *) (pU3phyAtopBase + 0x25 * 2 - 1));      /*  turn on PD demux,  TEST_RXPLL[47]=1 for turning on PD demux */

#if 1        /* CONFIG_MSTAR_K6 */
		/* This should be noticed after K6 (included)*/
		/* clear reg_pd_sqh & reg_pd_sqh_igen before reg_sqh_cal_start */
	writeb(0x00, (void *) (pU3phyAtopBase + 0x6D * 2 - 1)); /* wriu APHY+0x6d  0x00 */
#endif
	writeb(0x01, (void *) (pU3phyAtopBase + 0x6E * 2));
/*  calibrate SQH , [0]=reg_sqh_cal_start */  /*duplicate, called at end*/

	/*override utmi HS_SE0 for REFCLK 100M*/
	writeb(0x03, (void *)(pUtmissBase + 0x00 * 2));/*  [1]=reg_term_override */
	writeb(0x00, (void *)(pUtmissBase + 0x10 * 2));/*  [7]=reg_se0_set*/
	/* IREF_PDN=0 & PD_BG_CURRENT=0 */
	writeb(0x00, (void *)(pUtmissBase + 0x01 * 2 - 1));/*  [6]=IREF_PDN */
	writeb(0x0F, (void *)(pUtmissBase + 0x08 * 2));/*  [7]=PD_BG_CURRENT */

    /* Chris suggests remove this line !!!
    writeb(0x10, (void *)(pUpllBase + 0x01 * 2 - 1));
    */

    /*reg_cal_start wait 10ms and then set 1*/
	msleep(10);

}

#else

static void mstar_phy_init(uintptr_t pU3phyDtopBase, uintptr_t pU3phyAtopBase,
			uintptr_t pUtmissBase, uintptr_t pUpllBase)
{
	writew(0x0104, (void *)(pU3phyAtopBase + 0x06 * 2));
	writew(0x0000, (void *)(pU3phyAtopBase + 0x00 * 2));
/* U3phy initial sequence */
	writew(0x0000, (void *)(pU3phyAtopBase + 0x02 * 2));
/* U3phy initial sequence */
	writew(0x0000, (void *)(pU3phyDtopBase + 0x20 * 2)); /* duplicate 0 */
/*U3phy initial sequence */
	writew(0x0000, (void *)(pU3phyDtopBase + 0x22 * 2)); /* duplicate 0 */
/*U3phy initial sequence */
	writew(readw((void *)(pU3phyAtopBase + 0x02 * 2)) & 0xBBFF, \
								(void *)(pU3phyAtopBase + 0x02 * 2));
#if defined(DTOP_TX_SYNTH_SET_60) /*16 bit addr*/
	writew(DTOP_TX_SYNTH_SET_60, (void *)(pU3phyDtopBase + 0xC0 * 2));
#endif

#if defined(DTOP_TX_SYNTH_STEP_61)
	writew(DTOP_TX_SYNTH_STEP_61, (void *)(pU3phyDtopBase + 0xC2 * 2));
#endif

#if defined(DTOP_TX_SYNTH_STEP_FRAC_62) /*16 bit addr*/
	writew(DTOP_TX_SYNTH_STEP_FRAC_62, (void *)(pU3phyDtopBase + 0xC4 * 2));
#endif

#if defined(DTOP_TX_SYNTH_SPAN_63) /*16 bit addr*/
	writew(DTOP_TX_SYNTH_SPAN_63, (void *)(pU3phyDtopBase + 0xC6 * 2));
#endif
/*Tx lock*/
	writew(readw((void *)(pU3phyAtopBase + 0x0E * 2)) & 0xFBFF, \
								(void *)(pU3phyAtopBase + 0x0E * 2));

#if 1 /* CONFIG_MSTAR_K6 */
/* APHY+0xf1 [3] to 1b1.*/
	writeb(0x08, (void *)(pU3phyDtopBase + 0xF1 * 2 - 1));
/* ([4]=reg_dedicate_pcie_phy) */
	writeb(0x10, (void *)(pU3phyDtopBase + 0x84 * 2));
#endif


	writeb(0x00, (void *)(pU3phyDtopBase + 0x03 * 2 - 1));
	writeb(0xC0, (void *)(pU3phyDtopBase + 0x08 * 2));
	writeb(0x10, (void *)(pU3phyDtopBase + 0x09 * 2-1));

	writew(0x0000, (void *)(pU3phyAtopBase + 0x3A * 2));
	writew(0x0560, (void *)(pU3phyDtopBase + 0x18 * 2));
	writew(0x0309, (void *)(pU3phyAtopBase + 0x3A * 2));
	writeb(readb((void *)(pU3phyAtopBase + 0x03 * 2 - 1)) & 0xbb, \
							(void *)(pU3phyAtopBase + 0x03 * 2 - 1));

	writew(0x0401,  (void *)(pU3phyAtopBase + 0x12 * 2));

		/* -------- New for MS28 --------- */

	writeb(readb((void *)(pU3phyAtopBase + 0x0F * 2 - 1)) & (u8)(~0x04), \
							(void *)(pU3phyAtopBase + 0x0F * 2 - 1));

		/* ------- Add by PCIe PHY SW setting from USB3 PHY ---------*/
		/* rx lock */
	writeb(0xF1, (void *)(pU3phyDtopBase + 0x0A * 2));
	writeb(0x01, (void *)(pU3phyDtopBase + 0x0B * 2 - 1));
		/* tx lock */
	writeb(0xF1, (void *)(pU3phyDtopBase + 0x12 * 2));
	writeb(0x01, (void *)(pU3phyDtopBase + 0x13 * 2 - 1));

		/* -------- PCIe Setting --------- */
	writeb(0xDF, (void *)(pU3phyDtopBase + 0xB4 * 2));

	writeb(readb((void *)(pU3phyDtopBase + 0x3A * 2)) | 0x02, \
					(void *)(pU3phyDtopBase + 0x3A * 2));

	writeb(0x04, (void *) (pU3phyDtopBase + 0x1E*2));
	writeb(0x80, (void *) (pU3phyAtopBase + 0x25 * 2 - 1));      /*  turn on PD demux,  TEST_RXPLL[47]=1 for turning on PD demux */

#if 1        /* CONFIG_MSTAR_K6 */
		/* This should be noticed after K6 (included)*/
		/* clear reg_pd_sqh & reg_pd_sqh_igen before reg_sqh_cal_start */
	writeb(0x00, (void *) (pU3phyAtopBase + 0x6D * 2 - 1)); /* wriu APHY+0x6d  0x00 */
#endif
	writeb(0x01, (void *) (pU3phyAtopBase + 0x6E * 2));
/*  calibrate SQH , [0]=reg_sqh_cal_start */  /*duplicate, called at end*/

	/*override utmi HS_SE0 for REFCLK 100M*/
	writeb(0x03, (void *)(pUtmissBase + 0x00 * 2));/*  [1]=reg_term_override */
	writeb(0x00, (void *)(pUtmissBase + 0x10 * 2));/*  [7]=reg_se0_set*/
	/* IREF_PDN=0 & PD_BG_CURRENT=0 */
	writeb(0x00, (void *)(pUtmissBase + 0x01 * 2 - 1));/*  [6]=IREF_PDN */
	writeb(0x0F, (void *)(pUtmissBase + 0x08 * 2));/*  [7]=PD_BG_CURRENT */

    /* Chris suggests remove this line !!!
    writeb(0x10, (void *)(pUpllBase + 0x01 * 2 - 1));
    */

    /*reg_cal_start wait 10ms and then set 1*/
	msleep(10);
}
#endif

#if 0
static void mstar_u3phy_init(uintptr_t pU3phyDtopBase, uintptr_t pU3phyAtopBase)
{
	writew(0x0104, (void *)(pU3phyAtopBase + 0x06 * 2));
	writew(0x0000, (void *)(pU3phyAtopBase + 0x00 * 2));
	writew(0x0000, (void *)(pU3phyAtopBase + 0x02 * 2));
	writew(0x0000, (void *)(pU3phyAtopBase + 0x3A * 2));

	writew(0x0160, (void *)(pU3phyDtopBase + 0x18 * 2));
	writew(0x0000, (void *)(pU3phyDtopBase + 0x20 * 2));
	writew(0x0000, (void *)(pU3phyDtopBase + 0x22 * 2));

	writew(0x0308, (void *)(pU3phyAtopBase + 0x3A * 2));
	writew(readw((void *)(pU3phyAtopBase + 0x02 * 2)) & 0xBBFF, \
				(void *)(pU3phyAtopBase + 0x02 * 2));

	writew(0x9374, (void *)(pU3phyDtopBase + 0xC0 * 2));
	writew(0x0018, (void *)(pU3phyDtopBase + 0xC2 * 2));
	writew(0x7002, (void *)(pU3phyDtopBase + 0xC4 * 2));
	writew(0x04D8, (void *)(pU3phyDtopBase + 0xC6 * 2));

	writew(0x3932, (void *)(pU3phyAtopBase + 0x60 * 2));
	writew(0x3939, (void *)(pU3phyAtopBase + 0x62 * 2));
	writew(0x3932, (void *)(pU3phyAtopBase + 0x64 * 2));
	writew(0x3939, (void *)(pU3phyAtopBase + 0x66 * 2));
	writew(0x0400, (void *)(pU3phyAtopBase + 0x12 * 2));

	writeb(0x00, (void *)(pU3phyAtopBase + 0xA0 * 2 + 1));
	writeb(0xF4, (void *)(pU3phyDtopBase + 0x12 * 2));
	writew(readw((void *)(pU3phyAtopBase + 0x0E * 2)) & 0xFBFF, \
				(void *)(pU3phyAtopBase + 0x0E * 2));
}

static void mstar_u3phy_ms28_init(uintptr_t pU3phyDtopBase, uintptr_t pU3phyAtopBase,
			uintptr_t pUtmissBase, uintptr_t pUpllBase)
{

#if 1 /* CONFIG_MSTAR_K6 */
	writeb(0x08, (void *)(pU3phyDtopBase + 0xF0 * 2 + 1));  /* APHY+0xf1 [3] to 1b1.*/
	writeb(0x10, (void *)(pU3phyDtopBase + 0x84 * 2));  	/* ([4]=reg_dedicate_pcie_phy) */
#endif
	/* Set ATOP+0x34[0] to 1b1 (16-bit addressing) */
	writeb(0x01, (void *) (pU3phyAtopBase + 0x34 * 4));

	writeb(0x00, (void *)(pU3phyDtopBase + 0x03 * 2 - 1));
	writeb(0xC0, (void *)(pU3phyDtopBase + 0x08 * 2));
	writeb(0x10, (void *)(pU3phyDtopBase + 0x09 * 2-1));


	writew(0x0104, (void *) (pU3phyAtopBase + 0x06 * 2));	/* for Enable 1G clock pass to UTMI */ /*[2] reg_pd_usb3_purt [7:6] reg_gcr_hpd_vsel*/

	/* U3phy initial sequence */
	writew(0x00, (void *)(pU3phyAtopBase));					/* power on rx atop */
	writew(0x00, (void *)(pU3phyAtopBase + 0x02 * 2)); 		/* power on tx atop */

	writew(0x00, (void *)(pU3phyAtopBase + 0x3A * 2));		/* overwrite power on rx/tx atop */
	writew(0x0560, (void *)(pU3phyDtopBase + 0x18 * 2));
	writew(0x00, (void *)(pU3phyDtopBase + 0x20 * 2)); 		/* power on u3_phy clockgen*/
	writew(0x00, (void *)(pU3phyDtopBase + 0x22 * 2)); 		/* power on u3_phy clockgen*/

	writew(0x00, (void *)(pU3phyAtopBase + 0x3A * 2)); 		/* override PD control */

	writew(0x309, (void *)(pU3phyAtopBase + 0x3A * 2)); 	/* [9,8,3] PD_TXCLK_USB3TXPLL, PD_USB3_IBIAS, PD_USB3TXPLL override enable*/
	writeb(readb((void *)(pU3phyAtopBase + 0x03 * 2 - 1)) & 0xbb, \
				(void *)(pU3phyAtopBase + 0x03 * 2 - 1)); 	/* override value 0*/

    /* -- TX current --- */
	writew(0x3932,  (void *)(pU3phyAtopBase + 0x60 * 2));
	writew(0x3939,  (void *)(pU3phyAtopBase + 0x62 * 2));
	writew(0x3932,  (void *)(pU3phyAtopBase + 0x64 * 2));
	writew(0x3939,  (void *)(pU3phyAtopBase + 0x66 * 2));
	writew(0x0400,  (void *)(pU3phyAtopBase + 0x12 * 2));

	/* -------- New for MS28 --------- */
	writeb(0x00, (void *)(pU3phyAtopBase + 0xA1 * 2 - 1));  		/* bit[15] EQ override */
	writeb(readb((void *)(pU3phyAtopBase + 0x0F * 2 - 1)) & (u8)(~0x04), \
				(void *)(pU3phyAtopBase + 0x0F * 2 - 1));  /* 0xF[10]  Fix AEQ RX-reset issue */

	/* ------- Add by PCIe PHY SW setting from USB3 PHY ---------*/
	/* rx lock */
	writeb(0xF1, (void *)(pU3phyDtopBase + 0x0A * 2));   /* [13:0]=reg_rx_lock_value */
	writeb(0x01, (void *)(pU3phyDtopBase + 0x0B * 2 - 1));
	/* tx lock */
	writeb(0xF1, (void *)(pU3phyDtopBase + 0x12 * 2));  /* [13:0]=reg_tx_lock_value */
	writeb(0x01, (void *)(pU3phyDtopBase + 0x13 * 2 - 1));

	/* -------- PCIe Setting --------- */
	writeb(0xDF, (void *)(pU3phyDtopBase + 0xB4 * 2));  /* [0]=reg_pcie_md */
														/* [1]=reg_pcie_new_elastic_buf_en */
	writeb(readb((void *)(pU3phyDtopBase + 0x3D * 2 - 1)) | 0x10, \
				  (void *)(pU3phyDtopBase + 0x3D * 2 - 1));  /* 0x1E bit12 =1; */
	writeb(readb((void *)(pU3phyDtopBase + 0x3A * 2)) | 0x02, \
				  (void *)(pU3phyDtopBase + 0x3A * 2)); 		/* 0x1D bit 1 =1;*/

	writeb(0x0D, (void *)(pU3phyDtopBase + 0x1E * 2));	/* [3]=reg_2port_in_different_mode */
														/*  in projects with u3(P0) and pcie(P1) need to set this bit to bank of P0*/

	writeb(0x04, (void *) (pU3phyDtopBase + 0x1E*2));

	/* Tune REFCLK */
	writeb(0x18, (void *) (pUtmissBase + 0x2C * 2));		/* set TX Swing bit[6:4] for better REFCLK+- */
	writeb(0x03, (void *) (pUtmissBase + 0x2B * 2 - 1));	/* set TX reference bit[1:0] +50mv */
	writeb(0x42, (void *) (pUtmissBase + 0x2F * 2 - 1));	/*  [30] & [25] to enable PCIE REF clock */
	writeb(0x80, (void *) (pU3phyAtopBase + 0x25 * 2 - 1));	/*  turn on PD demux,  TEST_RXPLL[47]=1 for turning on PD demux */

#if 1 	/* CONFIG_MSTAR_K6 */
	/* This should be noticed after K6 (included)*/
	/* clear reg_pd_sqh & reg_pd_sqh_igen before reg_sqh_cal_start */
	writeb(0x00, (void *) (pU3phyAtopBase + 0x6C * 2 + 1)); /* wriu APHY+0x6d  0x00 */
#endif
	writeb(0x01, (void *) (pU3phyAtopBase + 0x6E * 2));			/*  calibrate SQH , [0]=reg_sqh_cal_start */

	/*override utmi HS_SE0 for REFCLK 100M*/
	writeb(0x03, (void *)(pUtmissBase + 0x00 * 2));				/*  [1]=reg_term_override */
	writeb(0x00, (void *)(pUtmissBase + 0x10 * 2));				/*  [7]=reg_se0_set*/
	/* IREF_PDN=0 & PD_BG_CURRENT=0 */
	writeb(0x00, (void *)(pUtmissBase + 0x01 * 2 - 1));			/*  [6]=IREF_PDN */
	writeb(0x0F, (void *)(pUtmissBase + 0x08 * 2));				/*  [7]=PD_BG_CURRENT */

	/* overwrite EQ @PCIE mode in FT/verification/normal function */
	writeb(0x88, (void *)(pU3phyAtopBase + 0xA1 * 2 - 1));	/* TEST_RXPLL[47]=1 for turning on PD demux */

	/* de-emphasis level best is preamp current 8, for best eye diagram */
	writeb(0x08, (void *)(pU3phyAtopBase + 0x62 * 2));

	/* ictrl_txpll=01 is best (2X), original 10 is worst*/
	writeb(0x01, (void *)(pU3phyAtopBase + 0x12 * 2));

	/* upll bw can set to higher IX1.25 could gain TIE 6-8ps? (only in PCIE mode)*/
	/* Chris suggests remove this line !!!
	writeb(0x10, (void *)(pUpllBase + 0x01 * 2 - 1));
	*/

    /*reg_cal_start wait 10ms and then set 1*/
	msleep(10);
	writeb(0x01, (void *)(pU3phyAtopBase + 0x6E * 2));

	/* K6 phy adjust - from analog team */
	/* suggested by analog team after measuring phy Tx/Rx */
	writeb(0x18, (void *)(pU3phyAtopBase + (0x62 * 2)));		/* de-emphasis adjustment */
	writeb(0x58, (void *)(pUtmissBase + (0x2C * 2)));			/* PCIE Tx current adjustment */
	writeb(0x01, (void *)(pU3phyDtopBase + (0xC0 * 2)));		/* adjust synth freq */
	writeb(0x90, (void *)(pU3phyDtopBase + (0xC1 * 2)-1));
	writeb(0x18, (void *)(pU3phyDtopBase + (0xC2 * 2)));

	/* msleep(100); */
}
#endif
#endif

#ifdef PCIE_ENABLE_PPC
/**
	static void mstar_enable_ppc(uintptr_t rc_top_base)
	\desc pcie port power control
		  rc_top_base[0xEE:0xF1] Power control GPIO
		  rc_top_base[0xF2:0xF5] PERST GPIO
*/
uint32_t mstar_pcie_enable_ppc(struct mstar_pcie_port *port)
{
	u16 offset;
	u16 addr_w, bit_num;
	uintptr_t addr;
	u8  value, low_active;

		/* pcie power control GPIO offset */

		addr_w = port->power_ctrl[0];
		addr = (uintptr_t)addr_w << 8;

		addr_w = port->power_ctrl[1];
		addr |= addr_w & 0xFF;

		bit_num = (addr_w >> 8) & 0x7;
		low_active = (u8)((addr_w >> 8) & 0x8);

		if (addr) {
			pcie_info(port->dev, "[PCIE]Power-%x %x %x\n", (unsigned int)addr, bit_num, low_active);

			value = (u8)(1 << bit_num);

			if (low_active) {
				/* assert */
				if (addr & 0x1)
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2 - 1)) & (u8)(~value), (void *)(MSTAR_PM_BASE + addr * 2 - 1));
				else
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2)) & (u8)(~value), (void *)(MSTAR_PM_BASE + addr * 2));
			} else {
				/* deassert */
				if (addr & 0x1)
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2 - 1)) | value, (void *)(MSTAR_PM_BASE + addr * 2 - 1));
				else
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2)) | value, (void *)(MSTAR_PM_BASE + addr * 2));
			}
			return addr;
		}
	return 0;
}

uint32_t mstar_pcie_perst_deassert(struct mstar_pcie_port *port)
{
//	u16 offset;
	u16 addr_w, bit_num;
	uintptr_t addr;
	u8  value, low_active;

		/* pcie power control GPIO offset */


		addr_w = port->perst_ctrl[0];

		addr = (uintptr_t)addr_w << 8;


		addr_w = port->perst_ctrl[1];

		addr |= addr_w & 0xFF;

		bit_num = (addr_w >> 8) & 0x7;
		low_active = (u8)((addr_w >> 8) & 0x8);

		if (addr) {
			pcie_info(port->dev, "[PCIE]PERST-%x %x %x\n", (unsigned int)addr, bit_num, low_active);

			value = (u8)(1 << bit_num);

			if (!low_active) {
				/* deassert */
				if (addr & 0x1)
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2 - 1)) & (u8)(~value), (void *)(MSTAR_PM_BASE + addr * 2 - 1));
				else
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2)) & (u8)(~value), (void *)(MSTAR_PM_BASE + addr * 2));
			} else {
				/* assert */
				if (addr & 0x1)
					writeb(readb((void *)(MSTAR_PM_BASE + addr*2-1)) | value, (void *)(MSTAR_PM_BASE + addr * 2 - 1));
				else
					writeb(readb((void *)(MSTAR_PM_BASE + addr * 2)) | value, (void *)(MSTAR_PM_BASE + addr * 2));
			}
			return addr;
		}
	return 0;
}

#endif

static int mstar_pcie_check_link(struct mstar_pcie_port *port)
{
	int Err_Return = 0;

#if 1
	int no_device_cnt = 0;
	int count = 0;
#endif

	/* check if the link is up or not */
	while (1) {
		mdelay(10);
		if (mstar_pcie_link_is_up(port)) {
			break; 			/* link up & exit */
		} else {
#if 1

			/* determine to quit link retry if no device plug-in */
			if (pcie_read(port, PCI_LINK_STATUS) <= 0x01)	/* link status is detect.quiet(0x00) or detect.active(0x01) */
				no_device_cnt += 1;
			else
				no_device_cnt = 0;

			if ((no_device_cnt == 3) && (count < 3))		/* Break, if first 3 times check of no device */
				goto no_device_break;

			/* link retrain */
			pcie_dbg(port->dev, "retrain link %d\n", count);
			/* rootport reset */
			pcie_write(port, PCI_LINKDOWN_RST_EN, PCI_RSTCR);
			/* de-assert reset signals */
			pcie_write(port, pcie_read(port, PCI_RSTCR) | PCI_RST_DEASSERTED, PCI_RSTCR);
			/* Add MStar RC top init function */

			mstar_pcie_rc_init(port);

			if (++count < 10)
				continue;

no_device_break:
#endif
			pcie_err(port->dev, "[PCIe] No device. %x, %x, %x\n", pcie_read(port, PCI_LINK_STATUS), count, no_device_cnt);

			/* turn off LTSSM_EN to avoid non-stop AER errors status */
			pcie_info(port->dev, "Turn off LTSSM_EN\n");
			pcie_riu_writew(port, pcie_riu_readw(port, REG_DESIGN_OPTION) & (~REG_BIT_LTSSM_EN), REG_DESIGN_OPTION);

			Err_Return = -EINVAL;
			break;
			/* return -EINVAL; */
		}
	}

	if (!Err_Return) {
		pcie_info(port->dev, "PCIE link is up\n");
		port->status |= PCIE_RC_STATUS_PHY_LINKED;
	}

	return Err_Return;
}

/**
 * mstar_pcie_init_hw - Initialize mstar PCIe host hardware
 * @port: PCIe port information
 * @res: Bus Resources
 *
 * Return: '0' on success and error value on failure
 */
static int mstar_pcie_init_hw(struct mstar_pcie_port *port)
{
	int size;
	struct device *dev = port->dev;
	struct resource *pref = &port->pref_mem;
	struct resource *npref = &port->npref_mem;

#ifdef CONFIG_MSTAR_ARM_BD_FPGA
	pcie_dbg(port->dev, "[PCIE] MDrv_IIC_init\n");

	if (PCIE_PHY_INIT(port->portnum)) {
		pcie_dbg(port->dev, "[PCIE] FPGA phy lan[0:3] init return non-00h!!\n");
	}
	/* PCIE_PHY_LoopBack(); */
#else

	/* ASIC phy init */
	pcie_info(port->dev, "[PCIE] phy init(U3 phy)\n");
#if 0
	mstar_u3phy_init((uintptr_t)port->phyDTOP_base, (uintptr_t)port->phyATOP_base);
	mstar_u3phy_ms28_init((uintptr_t)port->phyDTOP_base, (uintptr_t)port->phyATOP_base, \
							(uintptr_t)port->utmiss_base, (uintptr_t)port->upll_base);
#else
#if defined(U3PHY_MCL16)
	Mstar_U3phy_MCL16_init((uintptr_t)port->phyDTOP_base, (uintptr_t)port->phyATOP_base, \
							(uintptr_t)port->utmiss_base, (uintptr_t)port->upll_base);
	mstar_phy_tuning(port);
#else
	mstar_phy_init((uintptr_t)port->phyDTOP_base, (uintptr_t)port->phyATOP_base, \
							(uintptr_t)port->utmiss_base, (uintptr_t)port->upll_base);
	mstar_phy_tuning(port);
#endif

#endif

#endif
	/* set pcie as RC root port*/
	pcie_write(port, PCIE_IS_RC_ROOT_PORT, K_GBL_1);

	/* Add MStar PCI MAC top init function*/
	mstar_pcie_config_init(port);

	pcie_info(dev, "PCIE init hw...\n");
	/* rc internal reset will work when Link state is from link up to link down. */
	pcie_write(port, PCI_LINKDOWN_RST_EN, PCI_RSTCR);
	/* de-assert reset  signals */
	pcie_write(port, pcie_read(port, PCI_RSTCR) |
		PCI_RST_DEASSERTED, PCI_RSTCR);

#if 0
	/* mdelay(100); */
	/* de-assert pe reset */
	pcie_write(port, pcie_read(port, PCI_RSTCR) |
		PCI_PERSTB, PCI_RSTCR);
#endif
	/* Add MStar RC top init function */
	mstar_pcie_rc_init(port);

	/*  init mac top
		set PCI_INT_MASK
		set PCI_IMSI_ADDR ?
		set PCI_1LANE_CAP
		set PCI_REG_WAKE_CONTROL
		set LTSSM enable
	*/

	/* set INT mask */
#ifdef PCIE_DEBUG_INT_STATUS_STAGE2
	pcie_write(port, pcie_read(port, INT_MASK) & ~(INTX_MASK | INTX_MASK_2), INT_MASK);
#else
	pcie_write(port, pcie_read(port, INT_MASK) & ~(INTX_MASK), INT_MASK);
#endif
	/* set AHB to PCIe translation windows for prefetchable and non-prefetchable memory */
	if (pref->flags & IORESOURCE_MEM) {
		size = pref->end - pref->start;
		pcie_write(port, (AHB2PCIE_BASEL(pref->start) |
				NOPREFETCH(IORESOURCE_PREFETCH) | BASE_SIZE(mstar_pcie_mapping_sz(size))),
		AHB2PCIE_BASE1_L);
	#ifdef CONFIG_ARM64  /* Fix compile error in 32bit env.*/
		pcie_write(port, AHB2PCIE_BASEH(pref->start), AHB2PCIE_BASE1_H);
	#else
		pcie_write(port, 0, AHB2PCIE_BASE1_H);
	#endif
	}

	if (npref->flags & IORESOURCE_MEM) {
		size = npref->end - npref->start;
		pcie_write(port, (AHB2PCIE_BASEL(npref->start) |
			NOPREFETCH(0) | BASE_SIZE(mstar_pcie_mapping_sz(size))),
		AHB2PCIE_BASE0_L);

	pcie_dbg(port->dev, "AHB2PCIE BASE = %08X\n", pcie_read(port, AHB2PCIE_BASE0_L));

	#ifdef CONFIG_ARM64  /* Fix compile error in 32bit env. */
		pcie_write(port, AHB2PCIE_BASEH(npref->start), AHB2PCIE_BASE0_H);
	#else
		pcie_write(port, 0, AHB2PCIE_BASE0_H);
	#endif
	}

	/* set PCIe to axi translation memory space.*/
	pcie_write(port, mstar_pcie_mapping_sz(PCIE2AXI_SIZE) | WIN_ENABLE, PCIE2AXI_WIN);
	pcie_dbg(port->dev, "[PCIE] PCIE2AXI_WIN=%08X\n", pcie_read(port, PCIE2AXI_WIN));

#if 0
	/* set debug mode registers */
	pcie_write(port, 0x03030303, DBG_MODE_SEL);
	printk("[PCIE] Set Debug mode register=%08X\n", pcie_read(port, DBG_MODE_SEL));
#endif

#if 0
	pcie_dbg(port->dev, "[PCIE] BAR0=%x", pcie_read(port, CFG_BAR0));
	pcie_dbg(port->dev, "[PCIE] BAR1=%x", pcie_read(port, CFG_BAR1));
	pcie_dbg(port->dev, "[PCIE] BAR2=%x", pcie_read(port, CFG_BAR2));
	pcie_dbg(port->dev, "[PCIE] BAR3=%x", pcie_read(port, CFG_BAR3));
	pcie_dbg(port->dev, "[PCIE] BAR4=%x", pcie_read(port, CFG_BAR4));
	pcie_dbg(port->dev, "[PCIE] BAR5=%x", pcie_read(port, CFG_BAR5));
#endif

#ifdef PCIE_ENABLE_PPC
	mstar_pcie_enable_ppc(port);		/* turn on port power */
	port->t_power_on = ktime_get();		/* get power on time stamp */
#endif

	return 0;
}

static int mstar_pcie_setup(int nr, struct pci_sys_data *sys)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 32)
	struct pci_host_bridge_window *win;
#else
	struct resource_entry *win;
#endif
	struct mstar_pcie_port *port;
	u32 val;

	sys->private_data = (void *) &g_pcie_ports[nr];
	port = (struct mstar_pcie_port *) sys->private_data;

	port->busnum = sys->busnr;
	pcie_info(port->dev, "setup for busnum: %d\n", sys->busnr);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 32)
	list_for_each_entry(win, &port->resources, list)
#else
	list_for_each_entry(win, &port->resources, node)
#endif
	{
		pci_add_resource_offset(
			&sys->resources, win->res, sys->mem_offset);
	}

	/*set irq for RC*/
	mstar_pcie_hw_wr_cfg(port, 0, port->devfn, PCI_INTERRUPT_LINE, 1,  port->irq);

	/*set primary, secondary, and subordinate bus number for RC*/
	mstar_pcie_hw_rd_cfg(port, 0, port->devfn, PCI_PRIMARY_BUS, 4, &val);
	val |= (0xff << 16) | ((sys->busnr + 1) << 8) | sys->busnr;

#ifdef PATCH_RC_BUSNUM_00
	/*Primary bus can only be zero*/
	if (sys->busnr != 0) {
		port->Cfg1_18h = val;
		val &= 0xFFFFFF00;
	}
#endif
	mstar_pcie_hw_wr_cfg(port, 0, port->devfn, PCI_PRIMARY_BUS, 4,  val);

#ifdef CONFIG_PCIEAER
	/* set bridge control SERR# , PLDA needs this both this bit set in COMMAND, bridge control then AER report will enabled */
	mstar_pcie_hw_rd_cfg(port, 0, port->devfn, PCI_BRIDGE_CONTROL, 2, &val);
	val |= 0x0002; /* SERR# */
	mstar_pcie_hw_wr_cfg(port, 0, port->devfn, PCI_BRIDGE_CONTROL, 2, val);
	pcie_dbg(port->dev, "Set SERR# in Bridge control (0x3E) %x\n", val);
#endif


	return 1;
}

static int mstar_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct mstar_pcie_port *port = (struct mstar_pcie_port *) ((struct pci_sys_data *)dev->bus->sysdata)->private_data;

	if (port->pin_irqs[pin-1] == 0)
		port->pin_irqs[pin-1] = mstar_pcie_irq_alloc();

	pcie_info(port->dev, "map [slot: %d: pin %d] to irq %d\n", slot, pin, port->pin_irqs[pin-1]);

	return port->pin_irqs[pin-1];
}

static struct pci_bus *mstar_pcie_scan_bus(int nr,
						  struct pci_sys_data *sys)
{
	return pci_scan_root_bus(NULL, sys->busnr, &mstar_pcie_ops, sys,
				 &sys->resources);
}

/*void *mstar_hw_private[PCIE_PORT_COUNT];*/

static struct hw_pci mstar_pcie_hw = {
	.setup		= mstar_pcie_setup,
	.scan		= mstar_pcie_scan_bus,
	.map_irq	= mstar_pcie_map_irq,
};


static int mstar_pcie_port_init(struct mstar_pcie_port *port)
{
	struct device *dev = port->dev;
	struct resource *mem, *io_res, *bus_range;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 32)
	struct pci_host_bridge_window *win;
#else
	struct resource_entry *win;
#endif
	int err = 0;


	/* register memory resource */
	mem = (struct resource *)devm_kzalloc(dev, sizeof(*mem), GFP_KERNEL);
	mem->name = "Mstar pcie rc memory";
	mem->start = port->iomem_base;
	mem->end = port->iomem_base + port->iomem_len - 1;
	mem->flags = IORESOURCE_MEM;
	memcpy(&port->npref_mem, mem, sizeof(*mem));

	pcie_info(port->dev, "request iomem 0x%x, len: 0x%x\n",
		(unsigned int)port->iomem_base, (unsigned int)port->iomem_len);

	err = request_resource(&iomem_resource, mem);
	if (err) {
		pcie_err(port->dev, "[PCIE] request PCI memory failed(start: 0x%lx, len: 0x%lx)\n",
			port->iomem_base, port->iomem_len);
		goto free_list_entry;
	}
	pci_add_resource(&port->resources, mem);

	/* register io resource */
	io_res = (struct resource *)devm_kzalloc(dev, sizeof(*io_res), GFP_KERNEL);
	*io_res = (struct resource) {
		.name	= "Mstar pcie io",
		.start	= (port->portnum+1) * 0x1000,
		.end	= (port->portnum+2) * 0x1000 - 1,
		.flags	= IORESOURCE_IO,
	};
	port->io_res = io_res;

	pcie_info(port->dev, "request io 0x%x, len: 0x%x\n", (unsigned int)port->io_res->start, (unsigned int)port->io_res->end);

	pci_add_resource(&port->resources, io_res);

	bus_range = (struct resource *)devm_kzalloc(dev, sizeof(*bus_range), GFP_KERNEL);
	*bus_range = (struct resource) {
		.name	= "Mstar pcie bus range",
		.start	= (port->portnum) * 0x40,
		.end	= (port->portnum+1) * 0x40 - 1,
		.flags	= IORESOURCE_BUS,
	};

	pci_add_resource(&port->resources, bus_range);

#if 1
	/* register irq */
	pcie_info(port->dev, "request irq %d\n", port->irq);
	err = request_irq(port->irq, mstar_pcie_intr_handler, IRQF_SHARED, "PCIe", port);
	if (err) {
		pcie_err(dev, "failed to register IRQ: %d\n", err);
		goto free_resources;
	}

#if defined(CONFIG_MP_PLATFORM_GIC_SET_MULTIPLE_CPUS) && defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
    irq_set_affinity_hint(port->irq, cpu_online_mask);
    irq_set_affinity(port->irq, cpu_online_mask);
#endif

#endif

#ifdef CONFIG_PCI_MSI
	mstar_pcie_enable_msi(port);
#endif

	err = mstar_pcie_init_hw(port);
	if (err) {
		pcie_err(dev, "failed to init pcie h/w\n");
		/* return err; */			/* don't report error if link failed. */
	}


	return 0;

free_resources:
free_list_entry:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 32)
	list_for_each_entry(win, &port->resources, list)
#else
	list_for_each_entry(win, &port->resources, node)
#endif
	{
		devm_kfree(dev, win->res);
	}
	pci_free_resource_list(&port->resources);

	return err;
}
static int pcie_parse_dts(struct mstar_pcie_port *port)
{
	struct device_node *dn;
	u16 offset;
	u8 found = 0;

	offset = PCIE_POWERCTRL_GPIO_OFFSET;

	port->power_ctrl[0] = pcie_riu_readw(port, (offset + 0));
	port->power_ctrl[1] = pcie_riu_readw(port, (offset + 1));

	offset = PCIE_PERST_GPIO_OFFSET;

	port->perst_ctrl[0] = pcie_riu_readw(port, (offset + 0));
	port->perst_ctrl[1] = pcie_riu_readw(port, (offset + 1));

	port->clk_src = UTMISS_REF_CLK_EN_2F;

	port->eq = ATOP_EQ_A1;

//	for_each_compatible_node(dn, NULL, "Mstar-pcie2-1")
	for_each_compatible_node(dn, NULL, port->name){

		found = 1;
#if 1
		if(!of_property_read_u32_array(dn, "power_ctrl", port->power_ctrl, 2))
			printk("by dts power_ctrl = %x, %x\n", port->power_ctrl[0], port->power_ctrl[1]);
		else
		{
			printk("by reg power_ctrl = %x, %x\n", port->power_ctrl[0], port->power_ctrl[1]);
		}

		if(!of_property_read_u32_array(dn, "perst_ctrl", port->perst_ctrl, 2))
			printk("by dts perst_ctrl = %x, %x\n", port->perst_ctrl[0], port->perst_ctrl[1]);
		else
		{
			printk("by reg power_ctrl = %x, %x\n", port->power_ctrl[0], port->power_ctrl[1]);
		}
#endif
		if(!of_property_read_u32(dn, "clk_src", &port->clk_src))
			printk("by dts clk_src = %d\n", port->clk_src);
		else
		{
			printk("by hal clk_src = %d\n", port->clk_src);
		}

		if(!of_property_read_u32(dn, "eq", &port->eq))
			printk("by dts eq = %d\n", port->eq);
		else
		{
			printk("by hal eq = %d\n", port->eq);
		}

	}

	if(!found){
			printk("setting param by default values\n");
	}
}

/**
 * mstar_pcie_probe - Probe function
 * @pdev: Platform device pointer
 *
 * Return: '0' on success and error value on failure
 */
static int __init mstar_pcie_probe(struct platform_device *pdev)
{
	struct mstar_pcie_port *port = NULL;
	int err;

	pcie_info(&pdev->dev, "pcie probe...\n");
#ifdef ENABLE_PCIE_PORT0
	if (strncmp(pdev->name, MSTAR_PCIE_PORT0_NAME, strlen(MSTAR_PCIE_PORT0_NAME)) == 0) {
		port = &g_pcie_ports[0];
	} else
#endif
#ifdef ENABLE_PCIE_PORT1
	if (strncmp(pdev->name, MSTAR_PCIE_PORT1_NAME, strlen(MSTAR_PCIE_PORT1_NAME)) == 0) {
		port = &g_pcie_ports[1];
	} else
#endif
	{
		pr_info("[PCIE] No PCIE driver found\n");
		return -ENODEV;
	}

	if (port == NULL) {
		pcie_err(&pdev->dev, "can't not found %s\n", pdev->name);
		return -ENODEV;
	}

	port->dev = &pdev->dev;
	pcie_parse_dts(port);

	/*
	 * parse PCI ranges, configuration bus range and
	 * request their resources
	 */
	INIT_LIST_HEAD(&port->resources);
	err = mstar_pcie_port_init(port);
	if (err) {
		pcie_err(&pdev->dev, "failed to init port (err=%d)\n", err);
		return err;
	}

	platform_set_drvdata(pdev, port);

	/*mstar_hw_private[mstar_pcie_hw.nr_controllers++] = (void *) port;*/
	mstar_pcie_hw.nr_controllers++;

	return 0;
}
/**
 * mstar_pcie_remove - Remove function
 * @pdev: Platform device pointer
 *
 * Return: '0' always
 */
static int mstar_pcie_remove(struct platform_device *pdev)
{
	/* struct mstar_pcie_port *port = (struct mstar_pcie_port *) platform_get_drvdata(pdev); */

	pcie_info(&pdev->dev, "PCIE remove\n");
	return 0;
}

#ifdef PCIE_TEST_MMIO_REGISTERS
void __iomem *pcie_mmio_start_addr;

void pcie_mmio_read_test(void)
{
	u8 	*u8_addr;
	unsigned int 	len, ii;
	unsigned int mt7615reg[] = {0x41F0, 0x1000, 0x1004, 0x1008, 0x4000};

	u32 val;
	u8_addr = (u8 *)pcie_mmio_start_addr;
	len = ARRAY_SIZE(mt7615reg);

	val = 0x02000000;
	pr_info("PCIE MMIO Write DWORD:%08X to mmio %08X\n", val, (u32)u8_addr+mt7615reg[0]);
	writel(val, (u8_addr+mt7615reg[0]));

	pr_info("PCIE MMIO Read DWORD test...%08X\n", (u32)u8_addr);

	for (ii = 0; ii < len; ii++) {
		/* val = *(u8_addr + mt7615reg[ii]); */
		val = readl((void *) (u8_addr + mt7615reg[ii]));

		pr_info("%08x: %08x\n", (unsigned int)mt7615reg[ii], val);
	}
}

void pcie_mmio_write_test(void)
{
	u8 	*u8_addr;
	unsigned int	len, ii;

	u8_addr = (u8 *)pcie_mmio_start_addr;
	len = 0x100;

	pr_info("PCIE MMIO Write byte test...\n");

	for (ii = 0; ii < len; ii++) {
		*(u8_addr + ii) = 0x5A;
	}
	pr_info("write len %x done\n", len);
}
#endif

bool mstar_pcie_change_link_speed(struct pci_dev *pdev, bool isGen2)
{
	u16 val, link_up_spd, link_dn_spd, count;
	struct pci_dev *up_dev, *dn_dev;

	/* skip if current device is not PCI express capable */
	/* or is either a root port or downstream port */
	if (!pci_is_pcie(pdev))
		goto skip;

	if ((pci_pcie_type(pdev) == PCI_EXP_TYPE_DOWNSTREAM) ||
		(pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT))
		goto skip;

	pcie_info(&pdev->dev, "try to change link speed...\n");

	/* initialize upstream/endpoint and downstream/root port device ptr */
	up_dev = pdev;
	dn_dev = pdev->bus->self;

	/* read link status register to find current speed */
	pcie_capability_read_word(up_dev, PCI_EXP_LNKSTA, &link_up_spd);
	link_up_spd &= PCI_EXP_LNKSTA_CLS;
	pcie_info(&pdev->dev, "Endpoint current speed: 0x%x\n", link_up_spd);
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKSTA, &link_dn_spd);
	link_dn_spd &= PCI_EXP_LNKSTA_CLS;
	pcie_info(&pdev->dev, "RC current speed: 0x%x\n", link_dn_spd);

	/* skip if both devices across the link are already trained to gen2 */
	if (isGen2 &&
		(link_up_spd == PCI_EXP_LNKSTA_CLS_5_0GB) &&
		(link_dn_spd == PCI_EXP_LNKSTA_CLS_5_0GB))
		goto skip;
	/* skip if both devices across the link are already trained to gen1 */
	else if (!isGen2 &&
		((link_up_spd == PCI_EXP_LNKSTA_CLS_2_5GB) ||
		(link_dn_spd == PCI_EXP_LNKSTA_CLS_2_5GB)))
		goto skip;

	/* read link capability register to find max speed supported */
	pcie_capability_read_word(up_dev, PCI_EXP_LNKCAP, &link_up_spd);
	link_up_spd &= PCI_EXP_LNKCAP_SLS;
	pcie_info(&pdev->dev, "Endpoint link capability: 0x%x\n", link_up_spd);
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKCAP, &link_dn_spd);
	link_dn_spd &= PCI_EXP_LNKCAP_SLS;
	pcie_info(&pdev->dev, "RC link capability: 0x%x\n", link_dn_spd);

	/* skip if any device across the link is not supporting gen2 speed */
	if (isGen2 &&
		((link_up_spd < PCI_EXP_LNKCAP_SLS_5_0GB) ||
		(link_dn_spd < PCI_EXP_LNKCAP_SLS_5_0GB)))
		goto skip;
	/* skip if any device across the link is not supporting gen1 speed */
	else if (!isGen2 &&
		((link_up_spd < PCI_EXP_LNKCAP_SLS_2_5GB) ||
		(link_dn_spd < PCI_EXP_LNKCAP_SLS_2_5GB)))
		goto skip;

	/* Set Link Speed */
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKCTL2, &val);
	val &= ~PCI_EXP_LNKSTA_CLS;
	if (isGen2)
		val |= PCI_EXP_LNKSTA_CLS_5_0GB;
	else
		val |= PCI_EXP_LNKSTA_CLS_2_5GB;
	pcie_capability_write_word(dn_dev, PCI_EXP_LNKCTL2, val);
	pcie_info(&pdev->dev, "set link speed: 0x%x\n", val);


	/* Retrain the link */
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKCTL, &val);
	val |= PCI_EXP_LNKCTL_RL;
	pcie_capability_write_word(dn_dev, PCI_EXP_LNKCTL, val);
	pcie_info(&pdev->dev, "retrain link: 0x%x done\n", val);

	count = 30;
	val = 0;
	while (count--) {
		mdelay(1);
		pcie_capability_read_word(dn_dev, PCI_EXP_LNKSTA, &link_dn_spd);
		if (val != (link_dn_spd))
			pr_info("[PCIE] read link status %08x\n", link_dn_spd);
		val = link_dn_spd;
	}

	return true;
skip:
	return false;
}


bool mstar_pcie_link_speed(bool isGen2)
{
	struct pci_dev *pdev = NULL;
	bool ret = false;

	for_each_pci_dev(pdev) {
		if (mstar_pcie_change_link_speed(pdev, isGen2))
			ret = true;
	}

	return ret;
}

static void mstar_pcie_enable_features(void)
{
	bool isGen2;
#ifdef PCIE_ENABLE_5G_LINK
		isGen2 = true;
#else
		isGen2 = false;
#endif
	if (!mstar_pcie_link_speed(isGen2))
		pr_info("[PCIE]: Link speed remained.\n");
	else
		pr_info("[PCIE]: Link speed changed.\n");
}

#ifdef CONFIG_PM
static int mstar_pcie_resume_noirq(struct device *dev)
{
	int err = 0;
	struct mstar_pcie_port *port =
		(struct mstar_pcie_port *) platform_get_drvdata(to_platform_device(dev));

	pr_info("[PCIE] resume_noirq start\n");

#ifdef CONFIG_PCI_MSI
	mstar_pcie_enable_msi(port);
#endif

	err = mstar_pcie_init_hw(port);
	if (err) {
		pcie_err(dev, "failed to init pcie h/w\n");
		/* return err; */			/* don't report error if link failed. */
	}

	msleep(PICE_LATE_INIT_DELAY);

#ifdef PCIE_ENABLE_PPC
	mstar_pcie_perst_deassert(port);	/* PERST deassert */
#endif

	mstar_pcie_check_link(port);

	pr_info("[PCIE] change speed 5G\n");
	mstar_pcie_enable_features();

	pr_info("[PCIE] resume_noirq end\n");

	return 0;
}

static const struct dev_pm_ops mstar_pcie_pm_ops = {
	.resume_noirq = mstar_pcie_resume_noirq,
	};
#endif

static struct platform_driver mstar_pcie_0_driver = {
	.driver = {
		.name = MSTAR_PCIE_PORT0_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &mstar_pcie_pm_ops,
#endif
	},
	.probe = mstar_pcie_probe,
	.remove = mstar_pcie_remove,
};

#ifdef ENABLE_PCIE_PORT1
static struct platform_driver mstar_pcie_1_driver = {
	.driver = {
		.name = MSTAR_PCIE_PORT1_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &mstar_pcie_pm_ops,
#endif
	},
	.probe = mstar_pcie_probe,
	.remove = mstar_pcie_remove,
};
#endif

#ifdef PCIE_DUMP_REGISTERS
#define REG_MAC_K_PARA_SIZE 0x60
#define REG_MAC_TOP_SIZE	0x80
#define REG_MAC_TEST_SIZE	0x04
#define REG_RC_TOP_SIZE		0x80
static	u32 reg_mac_k[REG_MAC_K_PARA_SIZE];
static	u32 reg_mac_top[REG_MAC_TOP_SIZE];
static	u32 reg_mac_test[REG_MAC_TEST_SIZE];
static	u32 reg_rc_top[REG_RC_TOP_SIZE];

static void mstar_pcie_dump_registers(struct mstar_pcie_port *port)
{
	u32 i;


	/* init arrary */
	memset((void *)reg_mac_k, 0, sizeof(u32) * ARRAY_SIZE(reg_mac_k));
	memset((void *)reg_mac_top, 0, sizeof(u32) * ARRAY_SIZE(reg_mac_top));
	memset((void *)reg_mac_test, 0, sizeof(u32) * ARRAY_SIZE(reg_mac_test));
	memset((void *)reg_rc_top, 0, sizeof(u32) * ARRAY_SIZE(reg_rc_top));

	/* read MAC k para register */
	for (i = 0; i < REG_MAC_K_PARA_SIZE; i++)
		reg_mac_k[i] = pcie_read(port, i*4);

	/* read MAC TOP register */
	for (i = 0; i < REG_MAC_TOP_SIZE; i++)
		reg_mac_top[i] = pcie_read(port, 0x400+i*4);

	/* read MAC test register */
	for (i = 0; i < REG_MAC_TEST_SIZE; i++)
		reg_mac_test[i] = pcie_read(port, 0x800+i*4);


	/* read RC TOP register */
	for (i = 0; i < REG_RC_TOP_SIZE; i++)
		reg_rc_top[i] = pcie_riu_readw(port, i);

	/* dump registers */

	pr_info("[PCIE] dump MAC k para registers:\n");
	for (i = 0; i < REG_MAC_K_PARA_SIZE; i += 4) {
		/* offset */
		pr_info("%08X:", i*4);
		/* show value */
		pr_info("%08X %08X %08X %08X\n", reg_mac_k[i], reg_mac_k[i+1], reg_mac_k[i+2], reg_mac_k[i+3]);
	}

	pr_info("[PCIE] dump MAC top registers:\n");
	for (i = 0; i < REG_MAC_TOP_SIZE; i += 4) {
		/* offset */
		pr_info("%08X:", 0x400+i*4);
		/* show value */
		pr_info("%08X %08X %08X %08X\n", reg_mac_top[i], reg_mac_top[i+1], reg_mac_top[i+2], reg_mac_top[i+3]);
	}

	pr_info("[PCIE] dump MAC test registers:\n");
	for (i = 0; i < REG_MAC_TEST_SIZE; i += 4) {
		/* offset */
		pr_info("%08X:", 0x800+i*4);
		/* show value */
		pr_info("%08X %08X %08X %08X\n", reg_mac_test[i], reg_mac_test[i+1], reg_mac_test[i+2], reg_mac_test[i+3]);
	}


	pr_info("[PCIE] dump RC top registers:\n");
	for (i = 0; i < REG_RC_TOP_SIZE; i += 4) {
		/* offset*/
		pr_info("%08X:", i*2);
		/* show value */
		pr_info("%04X %04X %04X %04X\n", reg_rc_top[i], reg_rc_top[i+1], reg_rc_top[i+2], reg_rc_top[i+3]);
	}
}
#endif

/* PCIe driver does not allow module unload */
/*static void pcie_late_init(struct work_struct *work)*/
#if (defined(PCIE_INIT_2STAGE) || defined(PCIE_INIT_1STAGE))
static int pcie_late_init(void)
#else
static int pcie_late_init(struct work_struct *work)
#endif
{
	ktime_t t_tmp;

	pr_info("[PCIE] pcie_late_init enter...\n");

	if (mstar_pcie_hw.nr_controllers > 0) {
#ifdef PCIE_ENABLE_PPC
		mstar_pcie_perst_deassert(&g_pcie_ports[0]);	/* PERST deassert */
		t_tmp = ktime_get();
		pr_info("[PCIE]P0 #PERST %dms\n", (u32)ktime_to_ms((ktime_sub(t_tmp, g_pcie_ports[0].t_power_on))));
#ifdef ENABLE_PCIE_PORT1
		mstar_pcie_perst_deassert(&g_pcie_ports[1]);	/* PERST deassert */
		t_tmp = ktime_get();
		pr_info("[PCIE]P1 #PERST %dms\n", (u32)ktime_to_ms((ktime_sub(t_tmp, g_pcie_ports[1].t_power_on))));
#endif
#endif

		mstar_pcie_check_link(&g_pcie_ports[0]);
#ifdef ENABLE_PCIE_PORT1
		mstar_pcie_check_link(&g_pcie_ports[1]);
#endif

#ifdef CONFIG_PCI_MSI
		msi_map_init(&g_pcie_ports[0]);
	#ifdef ENABLE_PCIE_PORT1
		msi_map_init(&g_pcie_ports[1]);
	#endif
#endif
		pci_common_init(&mstar_pcie_hw);
	}

#ifdef PCIE_ENABLE_5G_LINK
	pr_info("[PCIE] change speed 5G\n");
	/* mstar_pcie_change_speed(&g_pcie_ports[0]); */
#endif
	mstar_pcie_enable_features();

#ifdef PCIE_DUMP_REGISTERS
	mstar_pcie_dump_registers(&g_pcie_ports[0]);
#endif

#ifdef PCIE_TEST_MMIO_REGISTERS
	pr_info("[PCIE] Read Command register\n");
	mstar_pcie_hw_rd_cfg(&g_pcie_ports[0], 0x01, 0, 0x04, 4, &val);
	val |= 0x06;
	pr_info("[PCIE] Command register = %08X\n", val);
	mstar_pcie_hw_wr_cfg(&g_pcie_ports[0], 0x01, 0, 0x04, 4, val);

	pcie_mmio_start_addr = ioremap(0xE0000000, 0x2000000);
	pr_info("pcie_mmio_start_addr: %x\n", (unsigned int) pcie_mmio_start_addr);
	pcie_mmio_read_test();
	iounmap(0xE0000000);					/*unmap iomem*/
#endif

#if 0
	pr_info("[PCIE] Call user function\n");
	char *argv[] = {"/bin/sh", "-c", "/bin/busybox insmod /lib/modules/3.10.40/mt_wifi.ko", NULL};
	static char *envp[] = {"HOME=/", "TERM=linux", "PATH=/sbin:/bin:/usr/sbin:/usr/bin", NULL };
	call_usermodehelper(argv[0], argv, envp, UMH_NO_WAIT);

	char *argv1[] = {"/bin/sh", "-c", "/bin/busybox ifconfig ra0 up", NULL};
	call_usermodehelper(argv1[0], argv1, envp, UMH_NO_WAIT);
#endif
	pr_info("[PCIE] pcie_late_init done\n");

	return 0;
}

static int __init pcie_init(void)
{
	int 	ret;
#ifdef PCIE_TEST_MMIO_REGISTERS
	u32 	val;
#endif

	pr_info("[PCIE] driver init start ------ %s -------\n", PCIE_MSTAR_VERSION);
#if defined(PCIE_INIT_1STAGE)
	pr_info("[PCIE] pcie_init 1 stage.");
#elif defined(PCIE_INIT_2STAGE)
	pr_info("[PCIE] pcie_init 2 stage.");
#else
	pr_info("[PCIE] pcie_init with late init.");
#endif


#ifdef FPGA_PCIE_PHY_IIC
	PCIE_IIC_INIT();
#endif

#if 1
#ifdef PCIE_DEBUG_INT_STATUS_STAGE2
	pr_info("[PCIE] interrupt status stage 2 enabled.\n");
#endif
#ifdef PATCH_IRQ_DELAY
	pr_info("[PCIE] irq dispatch delay 10ms\n");
#endif
#ifdef ENABLE_L3_CACHE
	pr_info("[PCIE] HW L3 Cache on\n");
#endif
#if defined(PCIE_INT_AFTER_MEM_WRITE_ECO)
	pr_info("[PCIE] e01-irq trigger after InB Wr done.\n");
#endif
#ifdef PCIE_GEN1_LINK_ECO
    /* fixes the issue of link failure because of clock phase difference */
    pr_info("[PCIE] e02-GEN1_LINK on\n");
#endif
#ifdef PCIE_REFCLK_PERFORMANCE_ECO
	pr_info("[PCIE] e03-REF clock performance\n");
#endif

#ifdef PATCH_BAR
	pr_info("[PCIE] RC BAR address patch is on\n");
#endif
#ifdef PATCH_SINGLE_DEVFN
	pr_info("[PCIE] single DEVFN of pcie port patch is on\n");
#endif
#endif

#ifdef PCIE_DEBUG_TEST_BUS
	pr_info("[PCIE] Enable test bus\n");
	writeb(0x10, (void *)MSTAR_PM_BASE + (0x101E24<<1));		/*bit[5:4] test bus out*/
	writeb(0x40, (void *)MSTAR_PM_BASE + (0x101EEB<<1) - 1);	/*bit[14] test bus enable*/
	writeb(0x0F, (void *)MSTAR_PM_BASE + (0x101EEE<<1));		/*[5:0] reg_test_bus24b_Sel*/
	/* PCIE port 1 */
	writeb(0x0C, (void *)MSTAR_PM_BASE + (0x171141<<1) - 1);	/*PCIE0 in mheg5*/
	writeb(0x00, (void *)MSTAR_PM_BASE + (0x14080A<<1));		/*PCIE0 reg_debug_sel*/

#endif

	mstar_pcie_hw.nr_controllers = 0;
	/* mstar_pcie_hw.private_data = mstar_hw_private; */

	g_pcie_ports[0].base = (void *) MSTAR_PCIE_PORT0_BASE;
	g_pcie_ports[0].riu_base = (void *) MSTAR_PCIE_PORT0_RIU0_BASE;
	g_pcie_ports[0].phyDTOP_base = (void *) MSTAR_PCIE_U3PHY_P0_DTOP_BASE;
	g_pcie_ports[0].phyATOP_base = (void *) MSTAR_PCIE_U3PHY_P0_ATOP_BASE;
	g_pcie_ports[0].utmiss_base = (void *) MSTAR_PCIE_UTMISS1_P0_BASE;
	g_pcie_ports[0].upll_base = (void *) MSTAR_PCIE_UPLL1_P0_BASE;
	g_pcie_ports[0].t_power_on = ktime_set(0, 0);			/* set default time stamp */

#ifdef ENABLE_PCIE_PORT1
	g_pcie_ports[1].base = (void *) MSTAR_PCIE_PORT1_BASE;
	g_pcie_ports[1].riu_base = (void *) MSTAR_PCIE_PORT1_RIU1_BASE;
	g_pcie_ports[1].phyDTOP_base = (void *) MSTAR_PCIE_U3PHY_P1_DTOP_BASE;
	g_pcie_ports[1].phyATOP_base = (void *) MSTAR_PCIE_U3PHY_P1_ATOP_BASE;
	g_pcie_ports[1].utmiss_base = (void *) MSTAR_PCIE_UTMISS1_P1_BASE;
	g_pcie_ports[1].upll_base = (void *) MSTAR_PCIE_UPLL1_P1_BASE;
	g_pcie_ports[1].t_power_on = ktime_set(0, 0);			/* set default time stamp */
#endif

#ifdef PATCH_WED_XIU_BUS_COLLISION
	g_pcie_ports[0].no_xiu_access = false;				/* enable xiu accesss in interrupt */
#ifdef ENABLE_PCIE_PORT1
	g_pcie_ports[1].no_xiu_access = false;				/* enable xiu accesss in interrupt */
#endif
#endif

	mstar_pcie_irq_map_init();

	ret = platform_add_devices(Mstar_pcie_platform_devices, ARRAY_SIZE(Mstar_pcie_platform_devices));
	if (ret) {
		pr_info("[PCIE] Fail to add the platform device %d\n", ret);
		return ret;
	}

	ret = platform_driver_register(&mstar_pcie_0_driver);
	if (ret) {
		pr_info("[PCIE] Fail to add pcie-0 platform driver %d\n", ret);
		return ret;
	}

#ifdef ENABLE_PCIE_PORT1
	ret = platform_driver_register(&mstar_pcie_1_driver);
	if (ret) {
		pr_info("[PCIE] Fail to add pcie-1 platform driver %d\n", ret);
		return ret;
	}
#endif


#if defined(PCIE_INIT_1STAGE)
	msleep(PICE_LATE_INIT_DELAY);
	pcie_late_init();
#elif defined(PCIE_INIT_2STAGE)
	/* PCIe init in 2 stage mode */
#else
	/* PCIe late init is default option */
	if (mstar_pcie_hw.nr_controllers > 0) {
		INIT_DELAYED_WORK(&g_pcie_late_init, pcie_late_init);
		schedule_delayed_work(&g_pcie_late_init, msecs_to_jiffies(PICE_LATE_INIT_DELAY));
	}
#endif

	return 0;
}

MODULE_AUTHOR("Mstar PCIe host driver");
MODULE_DESCRIPTION("MStar PCIe host controller driver");
MODULE_LICENSE("GPL v2");

/**
 * Module init and exit
 */
#if defined(PCIE_INIT_1STAGE)
	fs_initcall(pcie_init);
#elif defined(PCIE_INIT_2STAGE)
	subsys_initcall(pcie_init);
	fs_initcall(pcie_late_init);
#else
	/* pcie late init is default option */
	module_init(pcie_init);
#endif
