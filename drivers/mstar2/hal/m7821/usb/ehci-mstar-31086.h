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


#ifndef _EHCI_MSTAR_31086_H
#define _EHCI_MSTAR_31086_H

#if defined(CONFIG_ARM)
#include "../cpu/arm/chip_int.h"
#elif defined(CONFIG_ARM64)
#include "../cpu/arm64/chip_int.h"
#endif

//------ General constant definition ---------------------------------
#define DISABLE	0
#define ENABLE	1

#define BIT0	(1U<<0)
#define BIT1	(1U<<1)
#define BIT2	(1U<<2)
#define BIT3	(1U<<3)
#define BIT4	(1U<<4)
#define BIT5	(1U<<5)
#define BIT6	(1U<<6)
#define BIT7	(1U<<7)
//--------------------------------------------------------------------


#define ENABLE_THIRD_EHC
#define ENABLE_FOURTH_EHC

#ifndef CONFIG_MSTAR_XUSB_PCIE_PLATFORM
#define ENABLE_FIFTH_EHC
#endif

#define ENABLE_IRQ_REMAP
#if defined(ENABLE_IRQ_REMAP)
	#define MSTAR_EHC1_IRQ		E_IRQ_UHC		
	#define MSTAR_EHC2_IRQ		E_IRQEXPL_UHC1	

	#if defined(ENABLE_THIRD_EHC)
		#define MSTAR_EHC3_IRQ		E_IRQEXPL_UHC2P2
	#endif
	#if defined(ENABLE_FOURTH_EHC)
		#define MSTAR_EHC4_IRQ		E_IRQHYPL_USB30_HS_UHC
	#endif
	#if defined(ENABLE_FIFTH_EHC)
		#define MSTAR_EHC5_IRQ		E_IRQHYPL_USB30_HS1_UHC	
	#endif
	/* 2 XHCI roots */
	#if  defined(CONFIG_USB_XHCI_HCD) || defined(CONFIG_USB_XHCI_HCD_MODULE) 
		#define MSTAR_XHCI_IRQ		E_IRQ_USB30
		#define MSTAR_XHCI2_IRQ		E_IRQHYPH_USB30_SS_INT_P1
	#endif
#endif
//--------------------------------------------------------------------


//------ RIU base addr and bus-addr to phy-addr translation ----------
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	#include <mstar/mstar_chip.h>

	#if defined(CONFIG_ARM64)
		extern ptrdiff_t mstar_pm_base;
		#define MSTAR_PM_BASE		(mstar_pm_base)
		#define _MSTAR_USB_BASEADR	(mstar_pm_base + 0x00200000ULL)
	#else
		#define MSTAR_PM_BASE      0xfd000000UL
		#define _MSTAR_USB_BASEADR 0xfd200000UL
	#endif

	#define MIU0_BUS_BASE_ADDR 	MSTAR_MIU0_BUS_BASE
	#define MIU1_BUS_BASE_ADDR	MSTAR_MIU1_BUS_BASE

	#if 0 && defined(CONFIG_ARM64)
		#define MIU0_PHY_BASE_ADDR	(0x00000000ULL)
		/* MIU0 4G */
		#define MIU0_SIZE		(0x100000000ULL)

		#define MIU1_PHY_BASE_ADDR	(0x100000000ULL)
		/* MIU1 1G, PHY space is full, set to 0 */
		#define MIU1_SIZE		(0x00000000ULL)

		#define ENABLE_USB_NEW_MIU_SLE	1
		#define USB_MIU_SEL0	((u8) 0xF0U)
		#define USB_MIU_SEL1	((u8) 0xEFU)
		#define USB_MIU_SEL2	((u8) 0xEFU)
		#define USB_MIU_SEL3	((u8) 0xEFU)
		#define EHC_DMA_BIT_MASK	DMA_BIT_MASK(33)
		#define XHC_DMA_BIT_MASK	DMA_BIT_MASK(33)
	#elsif 0 // ARM32
		#define MIU0_PHY_BASE_ADDR	(0x00000000ULL)
		/* MIU0 2G */
		#define MIU0_SIZE		(0x80000000ULL)

		#define MIU1_PHY_BASE_ADDR	(0x80000000ULL)
		/* MIU1 1G */
		#define MIU1_SIZE		(0x40000000ULL)

		#define ENABLE_USB_NEW_MIU_SLE	1
		#define USB_MIU_SEL0	((u8) 0x70U)
		#define USB_MIU_SEL1	((u8) 0xB8U)
		#define USB_MIU_SEL2	((u8) 0xEFU)
		#define USB_MIU_SEL3	((u8) 0xEFU)
	#else
		#define SHORT_ADDR_MAX	(0xFFFFFFFFULL)
		#define MIU_SZ0G	((unsigned long long)(0))
		#define MIU_SZ1G	((unsigned long long)(SZ_1G))
		#define MIU_SZ2G	(MIU_SZ1G << 1)
		#define MIU_SZ4G	(MIU_SZ1G << 2)
		#define MIU0_SIZE	((MIU0_BUS_BASE_ADDR > SHORT_ADDR_MAX) ? \
						MIU_SZ4G : MIU_SZ2G)
		#define MIU1_SIZE	((MIU0_SIZE >= MIU_SZ4G) ? MIU_SZ0G : \
						(MIU1_BUS_BASE_ADDR > SHORT_ADDR_MAX ? \
						MIU_SZ2G : MIU_SZ1G))
		#define MIU0_PHY_BASE_ADDR	(0ULL)
		#define MIU1_PHY_BASE_ADDR	(MIU0_PHY_BASE_ADDR + MIU0_SIZE)

		#define ENABLE_USB_NEW_MIU_SLE	1
		#define PHY2REG(BASE, SIZE) \
				((((BASE + SIZE - 1) / SZ_256M) << 4) | \
				(BASE / SZ_256M))
		#define USB_MIU_SEL0	((u8) PHY2REG(MIU0_PHY_BASE_ADDR, MIU0_SIZE))
		#define USB_MIU_SEL1	((u8) ((MIU1_SIZE == MIU_SZ0G) ? 0xEFU : \
						PHY2REG(MIU1_PHY_BASE_ADDR, MIU1_SIZE)))
		#define USB_MIU_SEL2	((u8) 0xEFU)
		#define USB_MIU_SEL3	((u8) 0xEFU)

		#define EHC_DMA_BIT_MASK	((MIU0_BUS_BASE_ADDR > SHORT_ADDR_MAX) ? \
							DMA_BIT_MASK(33) : DMA_BIT_MASK(32))
		#define XHC_DMA_BIT_MASK	EHC_DMA_BIT_MASK
	#endif

	#define MIU_FAIL_ADDR ULONG_MAX

	#define IN_MIU_BUS(A, NUM)	\
		(((A)>=MIU##NUM##_BUS_BASE_ADDR)&&((A)<=(MIU##NUM##_BUS_BASE_ADDR+MIU##NUM##_SIZE-1)))

	#define IN_MIU_PA(A, NUM)	\
		(((A)>=MIU##NUM##_PHY_BASE_ADDR)&&((A)<=(MIU##NUM##_PHY_BASE_ADDR+MIU##NUM##_SIZE-1)))

	#define PA_MIU(A, NUM)	\
		((A)-MIU##NUM##_BUS_BASE_ADDR+MIU##NUM##_PHY_BASE_ADDR)

	#define BUS_MIU(A, NUM)	\
		((A)-MIU##NUM##_PHY_BASE_ADDR+MIU##NUM##_BUS_BASE_ADDR)

	#if defined(MIU2_BUS_BASE_ADDR)
		#define _BUS2PA(A)	\
			(IN_MIU_BUS(A, 0) ? PA_MIU(A, 0) : (IN_MIU_BUS(A, 1)? PA_MIU(A, 1): (IN_MIU_BUS(A, 2)? PA_MIU(A, 2) : MIU_FAIL_ADDR)))

		#define PA2BUS(A)	\
			(IN_MIU_PA(A, 0) ? BUS_MIU(A, 0) : (IN_MIU_PA(A, 1)? BUS_MIU(A, 1): (IN_MIU_PA(A, 2)? BUS_MIU(A, 2) : MIU_FAIL_ADDR)))
	#else
		#define _BUS2PA(A)	\
			(IN_MIU_BUS(A, 0) ? PA_MIU(A, 0) : (IN_MIU_BUS(A, 1)? PA_MIU(A, 1): MIU_FAIL_ADDR))

		#define PA2BUS(A)	\
			(IN_MIU_PA(A, 0) ? BUS_MIU(A, 0) : (IN_MIU_PA(A, 1)? BUS_MIU(A, 1): MIU_FAIL_ADDR))
	#endif

	#define DEBUG_BUS_PA_PATCH
	#ifdef DEBUG_BUS_PA_PATCH
	static inline uintptr_t BUS2PA(uintptr_t addr)
	{
		uintptr_t ret;

		ret = _BUS2PA(addr);
		if (ret == MIU_FAIL_ADDR)
		{
			printk("[USB WARNING] unsupported bus address 0x%lx\n", (unsigned long) addr);
			printk("MSTAR_MIU0_BUS_BASE:%lx\n", (unsigned long) MSTAR_MIU0_BUS_BASE);
			printk("MSTAR_MIU1_BUS_BASE:%lx\n", (unsigned long) MSTAR_MIU1_BUS_BASE);
			#if defined(MSTAR_MIU2_BUS_BASE)
			printk("MSTAR_MIU2_BUS_BASE:%lx\n", (unsigned long) MSTAR_MIU2_BUS_BASE);
			#endif
			BUG();
		}
		return (ret);
	}
	#else
		#define BUS2PA(A) _BUS2PA(A)
	#endif
#else
#error chip setting
#endif

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)|| defined(CONFIG_MIPS)
#define BUS_PA_PATCH
#endif
//--------------------------------------------------------------------


//------ UTMI, USBC and UHC base address -----------------------------
//---- Port0
#define _MSTAR_UTMI0_BASE	(_MSTAR_USB_BASEADR+(0x3A80*2))
#define _MSTAR_UHC0_BASE	(_MSTAR_USB_BASEADR+(0x2400*2))
#define _MSTAR_USBC0_BASE	(_MSTAR_USB_BASEADR+(0x0700*2))
#define _MSTAR_BC0_BASE		(_MSTAR_USB_BASEADR+(0x23600*2))
//---- Port1
#if !defined(DISABLE_SECOND_EHC)
	#define _MSTAR_UTMI1_BASE	(_MSTAR_USB_BASEADR+(0x3A00*2))
	#define _MSTAR_UHC1_BASE	(_MSTAR_USB_BASEADR+(0x0D00*2))
	#define _MSTAR_USBC1_BASE	(_MSTAR_USB_BASEADR+(0x0780*2))
	#define _MSTAR_BC1_BASE		(_MSTAR_USB_BASEADR+(0x23620*2))
#endif
//---- Port2
#ifdef ENABLE_THIRD_EHC
	#define _MSTAR_UTMI2_BASE	(_MSTAR_USB_BASEADR+(0x3900*2))
	#define _MSTAR_UHC2_BASE	(_MSTAR_USB_BASEADR+(0x13900*2))
	#define _MSTAR_USBC2_BASE	(_MSTAR_USB_BASEADR+(0x13800*2))
	#define _MSTAR_BC2_BASE		(_MSTAR_USB_BASEADR+(0x23640*2))
#endif
//---- Port3
#ifdef ENABLE_FOURTH_EHC
	#define _MSTAR_UTMI3_BASE	(_MSTAR_USB_BASEADR+(0x22300*2))
	#define _MSTAR_UHC3_BASE	(_MSTAR_USB_BASEADR+(0x53000*2))
	#define _MSTAR_USBC3_BASE	(_MSTAR_USB_BASEADR+(0x53200*2))
	#define _MSTAR_EHC3_COMP_PORT (_MSTAR_USB_BASEADR+(0x90000*2)+0x430)
	#define _MSTAR_EHC3_COMP_U3TOP_BASE  _MSTAR_U3TOP0_BASE
	#define _MSTAR_EHC3_COMP_PORT_INDEX  0
	#define _MSTAR_BC3_BASE		(_MSTAR_USB_BASEADR+(0x23680*2))
#endif
//---- Port4
#ifdef ENABLE_FIFTH_EHC
	#define _MSTAR_UTMI4_BASE	(_MSTAR_USB_BASEADR+(0x22380*2))
	#define _MSTAR_UHC4_BASE	(_MSTAR_USB_BASEADR+(0x53100*2))
	#define _MSTAR_USBC4_BASE	(_MSTAR_USB_BASEADR+(0x53280*2))
	#define _MSTAR_EHC4_COMP_PORT (_MSTAR_USB_BASEADR+(0xf0000*2)+0x430)
	#define _MSTAR_EHC4_COMP_U3TOP_BASE  _MSTAR_U3TOP1_BASE
	#define _MSTAR_EHC4_COMP_PORT_INDEX  1
	#define _MSTAR_BC4_BASE		(_MSTAR_USB_BASEADR+(0x236A0*2))
#endif
//--------------------------------------------------------------------


//------ Hardware ECO enable switch ----------------------------------
//---- 1. cross point
#define ENABLE_LS_CROSS_POINT_ECO
#define LS_CROSS_POINT_ECO_OFFSET	(0x04*2)
#define LS_CROSS_POINT_ECO_BITSET	BIT6

//---- 2. power noise
#define ENABLE_PWR_NOISE_ECO

//---- 3. tx/rx reset clock gating cause XIU timeout
#define ENABLE_TX_RX_RESET_CLK_GATING_ECO
#define TX_RX_RESET_CLK_GATING_ECO_OFFSET	(0x04*2)
#define TX_RX_RESET_CLK_GATING_ECO_BITSET	BIT5

//---- 8. fix pv2mi bridge mis-behavior
#define ENABLE_PV2MI_BRIDGE_ECO

//---- 9. change to 55 interface
#define ENABLE_UTMI_55_INTERFACE

//---- 10. 240's phase as 120's clock
/* bit<3> for 240's phase as 120's clock set 1, bit<4> for 240Mhz in mac 0 for faraday 1 for etron */
#define ENABLE_UTMI_240_AS_120_PHASE_ECO

//---- 14. HS connection fail problem (Gate into VFALL state)
#define ENABLE_HS_CONNECTION_FAIL_INTO_VFALL_ECO

//---- 15. Enable UHC Preamble ECO function
#define ENABLE_UHC_PREAMBLE_ECO

//---- 16. Don't close RUN bit when device disconnect
#define ENABLE_UHC_RUN_BIT_ALWAYS_ON_ECO

//---- 18. Extra HS SOF after bus reset
#define ENABLE_UHC_EXTRA_HS_SOF_ECO

//---- 19. Not yet support MIU lower bound address subtraction ECO (for chips which use ENABLE_USB_NEW_MIU_SLE)
//#define DISABLE_MIU_LOW_BOUND_ADDR_SUBTRACT_ECO

//---- 20. UHC speed type report should be reset by device disconnection
#define ENABLE_DISCONNECT_SPEED_REPORT_RESET_ECO

//---- 21. Port Change Detect (PCD) is triggered by babble. Pulse trigger will not hang this condition.
/* 1'b0: level trigger
 * 1'b1: one-pulse trigger
 */
#define ENABLE_BABBLE_PCD_ONE_PULSE_TRIGGER_ECO

//---- 22. generation of hhc_reset_u
/* 1'b0: hhc_reset is_u double sync of hhc_reset
 * 1'b1: hhc_reset_u is one-pulse of hhc_reset
 */
#define ENABLE_HC_RESET_FAIL_ECO

//---- 23. EHCI keeps running when device is disconnected
//#define ENABLE_DISCONNECT_HC_KEEP_RUNNING_ECO

//---- 24. DMA interrupt after the write back of qTD
// don't enable it, it make split transaction very slow
//#define ENABLE_INT_AFTER_WRITE_DMA_ECO

//---- 25. Do SRAM clock gating automatically to save power
#define ENABLE_SRAM_CLK_GATING_ECO
//--------------------------------------------------------------------


//------ Software patch enable switch --------------------------------
//---- 1. flush MIU pipe
#define _USB_T3_WBTIMEOUT_PATCH 1

//---- 2. data structure (qtd ,...) must be 128-byte aligment
#define _USB_128_ALIGMENT 1

//---- 3. tx/rx reset clock gating cause XIU timeout
#define _USB_XIU_TIMEOUT_PATCH 0

//---- 4. short packet lose interrupt without IOC
#define _USB_SHORT_PACKET_LOSE_INT_PATCH 0

//---- 5. QH blocking in MDATA condition, split zero-size data
#define _USB_SPLIT_MDATA_BLOCKING_PATCH 1

//---- 6. DM always keep high issue
#if !defined(ENABLE_HS_DM_KEEP_HIGH_ECO) // if without ECO solution, use SW patch.
	#define _USB_HS_CUR_DRIVE_DM_ALLWAYS_HIGH_PATCH 0
#else
	#define _USB_HS_CUR_DRIVE_DM_ALLWAYS_HIGH_PATCH 0
#endif

//---- 7. clear port eanble when device disconnect while bus reset
#define _USB_CLEAR_PORT_ENABLE_AFTER_FAIL_RESET_PATCH 1

//---- 8. mstar host only supports "Throttle Mode" in split translation
#define _USB_TURN_ON_TT_THROTTLE_MODE_PATCH 1

//---- 9. lower squelch level to cover weak cable link
#define _USB_ANALOG_RX_SQUELCH_PATCH 0

//---- 10. high speed reset chirp patch
//#define _USB_HS_CHIRP_PATCH 1

//---- 11. friendly customer patch
#define _USB_FRIENDLY_CUSTOMER_PATCH 1

//---- 12. enabe PVCI i_miwcplt wait for mi2uh_last_done_z
#define _USB_MIU_WRITE_WAIT_LAST_DONE_Z_PATCH 1

//---- 13. usb disconnection not reset usb speed report
#define _USB_SPEED_REPORT_RESET_PATCH 0

//---- 14. enable new chrip design, disonnect reg_2A, chrip detect reg_42
#define ENABLE_NEW_HW_CHRIP_PATCH
//--------------------------------------------------------------------


//------ Reduce EOF1 to 12us for performance imporvement -------------
/* Enlarge EOP1 from 12us to 16us for babble prvention under hub case.
 * However, we keep the "old config name". 20130121
 */
#define ENABLE_12US_EOF1
//--------------------------------------------------------------------

#define _UTMI_PWR_SAV_MODE_ENABLE 1

//---- Setting PV2MI bridge read/write burst size to minimum
#define _USB_MINI_PV2MI_BURST_SIZE 1

//--------------------------------------------------------------------
#define _USB_UTMI_DPDM_SWAP_P0 0
#define _USB_UTMI_DPDM_SWAP_P1 0

//------ Titania3_series_start_ehc flag ------------------------------
// Use low word as flag
#define EHCFLAG_NONE			0x0
#define EHCFLAG_DPDM_SWAP		0x1
#define EHCFLAG_TESTPKG			0x2
#define EHCFLAG_DOUBLE_DATARATE		0x4
#define EHCFLAG_ENABLE_OTG		0x8

// Use high word as data
#define EHCFLAG_DDR_MASK		0xF0000000
#define EHCFLAG_DDR_x15			0x10000000 //480MHz x1.5
#define EHCFLAG_DDR_x18			0x20000000 //480MHz x1.8
//--------------------------------------------------------------------

//------ UTMI disconnect level parameters ---------------------------------
// disc: bit[7:4] 0x00: 550mv, 0x20: 575, 0x40: 600, 0x60: 625
// squelch: bit[3:0] 4'b0010 => 100mv
#define UTMI_DISCON_LEVEL_2A	(0x60 | 0x02)
#define UTMI_CHIRP_DCT_LEVEL_42	(0x80)

//------ UTMI eye diagram parameters ---------------------------------
#if 0
	// for 40nm
	#define UTMI_EYE_SETTING_2C	(0x98)
	#define UTMI_EYE_SETTING_2D	(0x02)
	#define UTMI_EYE_SETTING_2E	(0x10)
	#define UTMI_EYE_SETTING_2F	(0x01)
#elif 0
	// for 40nm after Agate, use 55nm setting7
	#define UTMI_EYE_SETTING_2C	(0x90)
	#define UTMI_EYE_SETTING_2D	(0x03)
	#define UTMI_EYE_SETTING_2E	(0x30)
	#define UTMI_EYE_SETTING_2F	(0x81)
#elif 0
	// for 40nm after Agate, use 55nm setting6
	#define UTMI_EYE_SETTING_2C	(0x10)
	#define UTMI_EYE_SETTING_2D	(0x03)
	#define UTMI_EYE_SETTING_2E	(0x30)
	#define UTMI_EYE_SETTING_2F	(0x81)
#elif 0
	// for 40nm after Agate, use 55nm setting5
	#define UTMI_EYE_SETTING_2C	(0x90)
	#define UTMI_EYE_SETTING_2D	(0x02)
	#define UTMI_EYE_SETTING_2E	(0x30)
	#define UTMI_EYE_SETTING_2F	(0x81)
#elif 0
	// for 40nm after Agate, use 55nm setting4
	#define UTMI_EYE_SETTING_2C	(0x90)
	#define UTMI_EYE_SETTING_2D	(0x03)
	#define UTMI_EYE_SETTING_2E	(0x00)
	#define UTMI_EYE_SETTING_2F	(0x81)
#elif 0
	// for 40nm after Agate, use 55nm setting3
	#define UTMI_EYE_SETTING_2C	(0x10)
	#define UTMI_EYE_SETTING_2D	(0x03)
	#define UTMI_EYE_SETTING_2E	(0x00)
	#define UTMI_EYE_SETTING_2F	(0x81)
#elif 0
	// for 40nm after Agate, use 55nm setting2
	#define UTMI_EYE_SETTING_2C	(0x90)
	#define UTMI_EYE_SETTING_2D	(0x02)
	#define UTMI_EYE_SETTING_2E	(0x00)
	#define UTMI_EYE_SETTING_2F	(0x81)
#else
	// for 40nm after Agate, use 55nm setting1, the default
	#define UTMI_EYE_SETTING_2C	(0x10)
	#define UTMI_EYE_SETTING_2D	(0x02)
	#define UTMI_EYE_SETTING_2E	(0x00)
	#define UTMI_EYE_SETTING_2F	(0x81)
#endif

//------ eFuse setting ---------------------------------
//#define MSTAR_EFUSE_RTERM

//------ USB MAC sram power saving setting ---------------------------------
#ifdef CONFIG_MSTAR_SRAMPD
#define USB_MAC_SRAM_POWER_DOWN_ENABLE
#define USB_MAC_SRAM_CTRL_ADDR(h) ((h)->usbc_base+0x0a*2)
#define USB_MAC_SRAM_CTRL_BIT(h) (0x4000)
#endif

//------ BDMA SW patch setting ---------------------------------
#ifdef CONFIG_MSTAR_BDMA
#define USB_MSTAR_BDMA
#define EHCI_CHECK_MIU1 1
#define MIU0_RIU_BASE (MSTAR_PM_BASE+0x101200*2)
#define MIU1_RIU_BASE (MSTAR_PM_BASE+0x100600*2)
#define MIU_DRAMOBF_READY_OFFSET (0x2a*2)
#define MIU_DRAMOBF_READY_BIT (1 << 15)
#define MIU_64BIT_CIPHER_OFFSET (0xd8*2)
#define MIU_64BIT_CIPHER_BIT (1 << 10)
#endif
#endif	/* _EHCI_MSTAR_31086_H */
