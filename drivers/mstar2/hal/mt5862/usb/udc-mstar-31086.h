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

//------------------------------------------------------------------------------
// FILE
//      ms_otg.h
//
// DESCRIPTION
//
// HISTORY
//
//------------------------------------------------------------------------------
#ifndef _UDC_MSTAR_31086_H
#define _UDC_MSTAR_31086_H

#if defined(CONFIG_ARM)
#include "../cpu/arm/chip_int.h"
#endif

#if defined(CONFIG_ARM64)
#include "../cpu/arm64/chip_int.h"
#endif

#define ENABLE_IRQ_REMAP
#if defined(ENABLE_IRQ_REMAP)
	#define MSTAR_OTG_IRQ		E_IRQHYPL_OTG_INT
#endif


#include <mstar/mstar_chip.h>

#if defined(CONFIG_ARM64)
	#define RIU_BASE	(mstar_pm_base + 0x00200000ULL)
#else
	#define MSTAR_PM_BASE	   0xfd000000UL
	#define RIU_BASE 0xfd200000UL
#endif


#define MIU0_BUS_BASE_ADDR 	MSTAR_MIU0_BUS_BASE
#define MIU1_BUS_BASE_ADDR	MSTAR_MIU1_BUS_BASE
#if defined(MSTAR_MIU2_BUS_BASE)
#define MIU2_BUS_BASE_ADDR	MSTAR_MIU2_BUS_BASE
#endif


#define MIU0_PHY_BASE_ADDR	(0x00000000ULL)
/* MIU0 4G*/
#define MIU0_SIZE		(0x100000000ULL)

#define MIU1_PHY_BASE_ADDR	(0x100000000ULL)
/* MIU1 0G*/
#define MIU1_SIZE		(0x00000000ULL)

#define ENABLE_OTG_USB_NEW_MIU_SLE	1
#define USB_MIU_SEL0	((u8) 0xF0U)
#define USB_MIU_SEL1	((u8) 0xEFU)
#define USB_MIU_SEL2	((u8) 0xEFU)	// Set Upper bound < Lower bound to disable MIU2
#define USB_MIU_SEL3	((u8) 0xEFU)	// Set Upper bound < Lower bound to disable MIU3


#define BUS2PA(A)	\
	((((A)>=MIU0_BUS_BASE_ADDR)&&((A)<(MIU0_BUS_BASE_ADDR+MIU0_SIZE)))?	\
		((A)-MIU0_BUS_BASE_ADDR+MIU0_PHY_BASE_ADDR):	\
		((((A)>= MIU1_BUS_BASE_ADDR)&&((A)<MIU1_BUS_BASE_ADDR+MIU1_SIZE))?	\
			((A)-MIU1_BUS_BASE_ADDR+MIU1_PHY_BASE_ADDR):	\
			(0xFFFFFFFF)))

#define PA2BUS(A)	\
	((((A)>=MIU0_PHY_BASE_ADDR)&&((A)<(MIU0_PHY_BASE_ADDR+MIU0_SIZE)))?	\
		((A)-MIU0_PHY_BASE_ADDR+MIU0_BUS_BASE_ADDR):	\
		((((A)>= MIU1_PHY_BASE_ADDR)&&((A)<MIU1_PHY_BASE_ADDR+MIU1_SIZE))?	\
			((A)-MIU1_PHY_BASE_ADDR+MIU1_BUS_BASE_ADDR):	\
			(0xFFFFFFFF)))


/******************************************************************************
 * SW patch config
 ******************************************************************************/
//***** for Cedric C3 patch*****//
//#define DRAM_MORE_THAN_1G_PATCH	//Clippers HW switch miu0/miu1 at 1G, if miu0 > 1G, HW will access wrong place
/******************************************************************************
 * HW eco config
 ******************************************************************************/
#define AHB_WDATA_CHANGES_WITHOUT_HREADY_ECO
#define SHORTPKT_STATUS_CLEAR_ECO
#define	MONKEY_TEST_ECO	//avoid device can not enter high speed mode
#define ENABLE_TX_RX_RESET_ECO
#define DMA_RXTX_INT_LAST_DONE_ECO
#define XIU_ACCESS_FAILED_WITH_ECO //(when clk_mcu<120MHz)

//---- change to 55 interface
#define ENABLE_UTMI_55_INTERFACE
/******************************************************************************
 * HW config
 ******************************************************************************/
#define MAX_USB_DMA_CHANNEL  2
#define MAX_EP_NUMBER	4

static const char ep0name [] = "ep0";
static const char *const ep_name[] = {
    ep0name,                                /* everyone has ep0 */
	"ep1in-bulk", "ep2out-bulk", "ep3in-int",
};
#define MSB250X_ENDPOINTS ARRAY_SIZE(ep_name)

//---USB device switch pad to CID enable
#define CID_ENABLE
#define ENABLE_SQUELCH_LEVEL
//------ UTMI squelch level parameters ---------------------------------
// disc: bit[7:4] 0x00: 550mv, 0x20: 575, 0x40: 600, 0x60: 625
// squelch: bit[3:0] 4'b0010 => 100mv
#define UTMI_squelch_LEVEL_2A	(0x02)

//------UTMI, USBC and OTG base address -----------------------------
#define UTMI_BASE_ADDR     (RIU_BASE+(0x3a80*2))
#define USBC_BASE_ADDR     (RIU_BASE+(0x0700*2))
#define OTG0_BASE_ADDR     (RIU_BASE+(0x21500*2))
#define BC_BASE_ADDR	   	 (RIU_BASE+(0x23600*2))
#endif  /* define _UDC_MSTAR_31086_H */
