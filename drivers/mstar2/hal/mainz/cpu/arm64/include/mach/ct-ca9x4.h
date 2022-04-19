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

#ifndef __MACH_CT_CA9X4_H
#define __MACH_CT_CA9X4_H
/*
* Physical base addresses
*/
#define CT_CA9X4_CLCDC		(0x10020000)
#define CT_CA9X4_AXIRAM		(0x10060000)
#define CT_CA9X4_DMC		(0x100e0000)
#define CT_CA9X4_SMC		(0x100e1000)
#define CT_CA9X4_SCC		(0x100e2000)
#define CT_CA9X4_SP804_TIMER	(0x100e4000)
#define CT_CA9X4_SP805_WDT	(0x100e5000)
#define CT_CA9X4_TZPC		(0x100e6000)
#define CT_CA9X4_GPIO		(0x100e8000)
#define CT_CA9X4_FASTAXI	(0x100e9000)
#define CT_CA9X4_SLOWAXI	(0x100ea000)
#define CT_CA9X4_TZASC		(0x100ec000)
#define CT_CA9X4_CORESIGHT	(0x10200000)
#define CT_CA9X4_MPIC		(0x1e000000)
#define CT_CA9X4_SYSTIMER	(0x1e004000)
#define CT_CA9X4_SYSWDT		(0x1e007000)
#define CT_CA9X4_L2CC		(0x1e00a000)
#define CT_CA9X4_TIMER0		(CT_CA9X4_SP804_TIMER + 0x000)
#define CT_CA9X4_TIMER1		(CT_CA9X4_SP804_TIMER + 0x020)
#define A9_MPCORE_SCU		(CT_CA9X4_MPIC + 0x0000)
#define A9_MPCORE_GIC_CPU	(CT_CA9X4_MPIC + 0x0100)
#define A9_MPCORE_GIT		(CT_CA9X4_MPIC + 0x0200)
#define A9_MPCORE_TWD		(CT_CA9X4_MPIC + 0x0600)
#define A9_MPCORE_GIC_DIST	(CT_CA9X4_MPIC + 0x1000)
/*
* Interrupts.  Those in {} are for AMBA devices
*/
#define IRQ_CT_CA9X4_CLCDC	{ 76 }
#define IRQ_CT_CA9X4_DMC	{ -1 }
#define IRQ_CT_CA9X4_SMC	{ 77, 78 }
#define IRQ_CT_CA9X4_TIMER0	80
#define IRQ_CT_CA9X4_TIMER1	81
#define IRQ_CT_CA9X4_GPIO	{ 82 }
#define IRQ_CT_CA9X4_PMU_CPU0	92
#define IRQ_CT_CA9X4_PMU_CPU1	93
#define IRQ_CT_CA9X4_PMU_CPU2	94
#define IRQ_CT_CA9X4_PMU_CPU3	95
extern struct ct_desc ct_ca9x4_desc;
#endif
