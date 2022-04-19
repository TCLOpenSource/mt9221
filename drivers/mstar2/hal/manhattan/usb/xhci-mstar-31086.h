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


#ifndef _XHCI_MSTAR_31086_H
#define _XHCI_MSTAR_31086_H

#include <ehci-mstar-31086.h>

// ----- Don't modify it !----------
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
#define XHCI_PA_PATCH   1
#else
#define XHCI_PA_PATCH   0
#endif
#define XHCI_FLUSHPIPE_PATCH  1
//------------------------------

#define XHCI_CHIRP_PATCH  1
#define ENABLE_XHCI_SSC   1

#if (ENABLE_XHCI_SSC)
#define XHCI_SSC_TX_SYNTH_SET_C0			0x9374
#define XHCI_SSC_TX_SYNTH_SET_C2			0x18
#define XHCI_SSC_TX_SYNTH_STEP_C4			0x7002
#define XHCI_SSC_TX_SYNTH_SPAN_C6			0x04D8
#endif

#define XHCI_TX_SWING_PATCH  1

//------ for test -----------------
//#define XHCI_CURRENT_SHARE_PATCH 0   //Only for USB3; will cause USB2 chirp handshake fail.
#define XHCI_ENABLE_DEQ  0
#define XHCI_ENABLE_TESTBUS  0
//--------------------------------

//Inter packet delay setting for all chips
#define XHCI_IPACKET_DELAY_PATCH

#define XHCI_DISABLE_COMPLIANCE
#define XHCI_DISABLE_TESTMODE
#define XHCI_SSDISABLED_PATCH
#define XHCI_HC_RESET_PATCH

#define MSTAR_LOST_SLOT_PATCH 0

#define XHCI_TX_ERR_EVENT_PATCH

#define XHCI_ENABLE_PPC

//--------  Setting option  -----------

#define XHCI_ENABLE_240MHZ

#define XHCI_ENABLE_LASTDOWNZ

//--------------------------------


// --------- ECO option ---------
#define XHCI_ENABLE_LOOPBACK_ECO
#define LOOPBACK_ECO_OFFSET		0x20*2
#define LOOPBACK_ECO_BIT		BIT4|BIT5

//--------------------------------


//--------  U3 PHY IP  -----------
#define XHCI_PHY_MS28

#ifdef XHCI_PHY_MS28
#define GCR_USB3RX0_RCTRL		(0x08*2)
#define GCR_USB3TX0_RT			(0x10*2)
#define GCR_USB3RX1_RCTRL		(0x08*2)
#define GCR_USB3TX1_RT			(0x10*2)

#define USB30RX0_EFUSE_BITOFFSET	8
#define USB30TX0_EFUSE_BITOFFSET	0
#define USB30RX1_EFUSE_BITOFFSET	24
#define USB30TX1_EFUSE_BITOFFSET	16
#endif

#define XHCI_COMPANION

#if defined(XHCI_2PORTS)
#define XHCI_SSDISABLE_POWERDOWN_PATCH
#endif

//--------------------------------

#if defined(CONFIG_ARM64)
	extern ptrdiff_t mstar_pm_base;
	#define _MSTAR_PM_BASE         (mstar_pm_base)
#elif defined(CONFIG_ARM)
	#define _MSTAR_PM_BASE         0xFD000000
#else
	#define _MSTAR_PM_BASE         0xBF000000
#endif

#define _MSTAR_U3PHY_DTOP_BASE (_MSTAR_USB_BASEADR+(0x11C00*2))
#define _MSTAR_U3PHY_ATOP_BASE (_MSTAR_USB_BASEADR+(0x22100*2))
#define _MSTAR_U3UTMI_BASE     0
#define _MSTAR_U3TOP_BASE      (_MSTAR_USB_BASEADR+(0x22500*2))
#define _MSTAR_XHCI_BASE       (_MSTAR_USB_BASEADR+(0x90000*2))
#define _MSTAR_U3BC_BASE       0

#define XHC_HSPORT_OFFSET	0x420
#define XHC_SSPORT_OFFSET	0x430

#if defined(XHCI_PHY_EFUSE)
	#define RTERM_XHC_BANK		(0x50*4)	//bank 0x50
#endif

#endif	/* _XHCI_MSTAR_31086_H */

