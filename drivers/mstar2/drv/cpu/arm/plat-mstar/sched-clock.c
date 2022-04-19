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

/*
 *  linux/arch/arm/arm-boards/plat-mstar/sched-clock.c
 *
 *  Copyright (C) 1999 - 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/clocksource.h>

#include <asm/sched_clock.h>
#include <plat/sched_clock.h>

#include <linux/version.h>
#include <asm/sched_clock.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
#include <linux/sched_clock.h>
#endif

extern struct clocksource clocksource_timer;

#if defined(CONFIG_MP_CA7_QUAD_CORE_PATCH)


#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <mach/io.h>
#include <mach/timex.h>
#include <asm/irq.h>
#include <linux/timer.h>
#include <plat/localtimer.h>
#include "chip_int.h"


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
u64 arch_counter_read(struct clocksource *cs)
#else
cycle_t arch_counter_read(struct clocksource *cs)
#endif
{
	return arch_counter_get_cntpct();
}
#endif

#if !defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
#ifdef CONFIG_MSTAR_CPU_calibrating
static DEFINE_CLOCK_DATA(cd);
#endif/*CONFIG_MSTAR_CPU_calibrating*/
#endif/*MP_STATIC_TIMER_CLOCK_SOURCE */
static void __iomem *ctr;

//clocks_calc_mult_shift(m, s, 450MHz, NSEC_PER_SEC, 8)
#if 0
#define SC_MULT		2386092942u
#define SC_SHIFT	30
#else
u32 SC_MULT=0;
u32 SC_SHIFT=0;
#endif

#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
#if defined(CONFIG_MSTAR_CLIPPERS) || defined(CONFIG_MSTAR_MONACO) || defined(CONFIG_MSTAR_MUSTANG) || defined(CONFIG_MSTAR_M3822)
static DEFINE_SPINLOCK(sched_clock_lock);
#define RTC1_TIMER_BASE (0xFD002600)
#define RTC1_TIMER_READ_MASK (0x10)
#define RTC1_TIMER_COUNTER_1_OFFEST (0x0b << 2)
#define RTC1_TIMER_COUNTER_2_OFFEST (0x0c << 2)
#define RTC1_TIMER_COUNTER_3_OFFEST (0x0d << 2)
#define RTC1_TIMER_COUNTER_4_OFFEST (0x0e << 2)
#endif // CONFIG_MSTAR_CLIPPERS || CONFIG_MSTAR_MONACO
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE

#if defined(CONFIG_MP_PLATFORM_CPU_SETTING)
#if defined CONFIG_MSTAR_CPU_calibrating
extern unsigned int current_frequency;
extern unsigned int register_frequency;
extern void clocks_calc_mult_shift(u32 *mult, u32 *shift, u32 from, u32 to, u32 maxsec);
#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
static void mstar_update_sched_clock(void);
#else
void mstar_update_sched_clock(void);
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
#endif //CONFIG_MSTAR_CPU_calibrating
#endif // MP_PLATFORM_CPU_SETTING

#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
unsigned int RTC1_timer_read32(void)
{
	unsigned int temp;
	unsigned int flags;
	unsigned int cnt1=10;
	spin_lock_irqsave(&sched_clock_lock ,flags);
	temp = *(volatile unsigned int *)(RTC1_TIMER_BASE);
	*(volatile unsigned int *)(RTC1_TIMER_BASE)= (temp|RTC1_TIMER_READ_MASK); //Read enable
	while(1) {
		if((*(volatile unsigned int *)(RTC1_TIMER_BASE) & RTC1_TIMER_READ_MASK) == 0)
			break;
	}
	u32 cyc =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
	u32 cyc2 =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
	u32 cyc1 =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
	u32 cyc3 =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);

	if((cyc==cyc1)  && (cyc2 == cyc3) )
	{
		cyc = (cyc << 16) + cyc2;
	}
	else
	{
		while(cnt1)
		{
			cyc =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
			cyc2 =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
			cyc1 =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
			cyc3 =*(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
			cnt1--;
			if((cyc==cyc1)  && (cyc2 == cyc3) )
			{
				cyc = (cyc << 16) + cyc2;
				break;
			}
		}
	}

	spin_unlock_irqrestore(&sched_clock_lock,flags);
	return cyc;
}

#if defined(CONFIG_MSTAR_CLIPPERS) || defined(CONFIG_MSTAR_MONACO)
unsigned long long RTC1_timer_read64(void)
{
	unsigned int flags;
	unsigned int temp;
	unsigned int cnt2=10;
	u64 cyc=0;
	spin_lock_irqsave(&sched_clock_lock ,flags);

	temp = *(volatile unsigned int *)(RTC1_TIMER_BASE);
	*(volatile unsigned int *)(RTC1_TIMER_BASE)= (temp|RTC1_TIMER_READ_MASK); //Read enable
	while(1) {
		if((*(volatile unsigned int *)(RTC1_TIMER_BASE) & RTC1_TIMER_READ_MASK) == 0)
			break;
	}
	u32 cyc1 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_4_OFFEST);
	u32 cyc3 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_3_OFFEST);
	u32 cyc5 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
	u32 cyc7 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
	u32 cyc2 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_4_OFFEST);
	u32 cyc4 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_3_OFFEST);
	u32 cyc6 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
	u32 cyc8 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
	if((cyc1==cyc2)&&(cyc3==cyc4)&&(cyc5==cyc6)&&(cyc7==cyc8))
	{
		cyc = cyc1;
		cyc = (cyc << 16) + cyc3;
		cyc = (cyc << 16) + cyc5;
		cyc = (cyc << 16) + cyc7;
	}
	else
	{
		while(cnt2)
		{
			cyc1 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_4_OFFEST);
			cyc3 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_3_OFFEST);
			cyc5 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
			cyc7 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
			cyc2 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_4_OFFEST);
			cyc4 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_3_OFFEST);
			cyc6 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_2_OFFEST);
			cyc8 = *(volatile unsigned int *)(RTC1_TIMER_BASE + RTC1_TIMER_COUNTER_1_OFFEST);
			cnt2--;
			if((cyc1==cyc2)&&(cyc3==cyc4)&&(cyc5==cyc6)&&(cyc7==cyc8))
			{
				cyc = cyc1;
				cyc = (cyc << 16) + cyc3;
				cyc = (cyc << 16) + cyc5;
				cyc = (cyc << 16) + cyc7;
				break;
			}
		}
	}
	spin_unlock_irqrestore(&sched_clock_lock,flags);
	return cyc;
}
#endif // CONFIG_MSTAR_CLIPPERS || CONFIG_MSTAR_MONACO
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
static u64 notrace mstar_sched_clock(void)
#else
static u32 notrace mstar_sched_clock(void)
#endif
{
	u32 cyc;
#if defined(CONFIG_MP_CA7_QUAD_CORE_PATCH)
#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
        cyc = RTC1_timer_read32();
#else
        cyc = arch_counter_read(NULL);
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
#else
#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
		cyc = RTC1_timer_read32();
#else
	if (ctr)
		cyc = readl(ctr);
	 else
		cyc = 0;
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
#endif // defined(CONFIG_MP_CA7_QUAD_CORE_PATCH)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
	return (u64)cyc;
#else
	return cyc;
#endif
}

extern void notrace update_sched_clock(void);
#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
static void mstar_update_sched_clock(void)
#else
void mstar_update_sched_clock(void)
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
{
        u32 shift,mult;
#if defined(CONFIG_MP_CA7_QUAD_CORE_PATCH)
#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
	u32 cyc =RTC1_timer_read32();
	update_sched_clock();
#else
	u32 cyc = arch_counter_read(NULL);
	update_sched_clock();
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
#else
#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
	u32 cyc =RTC1_timer_read32();
	update_sched_clock();
#else
	u32 cyc = readl(ctr);
	update_sched_clock();
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
#endif // defined(CONFIG_MP_CA7_QUAD_CORE_PATCH)

#if !defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
#ifdef CONFIG_MSTAR_CPU_calibrating
    if (register_frequency)
	{
        clocks_calc_mult_shift(&mult, &shift,register_frequency /2 * 1000, NSEC_PER_SEC, 0);

        cd.mult = mult;
        cd.shift = shift;
    }
#endif // CONFIG_MSTAR_CPU_calibrating
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
}

#if defined(CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE)
void __init mstar_sched_clock_init(void *reg, unsigned long rate)
{
	//printk("SC_MULT  = %u  ,SC_SHIFT  = %u \n",SC_MULT,SC_SHIFT);
	ctr = reg;
	spin_lock_init(&sched_clock_lock);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
	sched_clock_register(mstar_sched_clock, 32, rate);
#else
	setup_sched_clock(mstar_sched_clock, 32, rate);
#endif
}
#else
void __init mstar_sched_clock_init(void __iomem *reg, unsigned long rate)
{
	printk("SC_MULT  = %u  ,SC_SHIFT  = %u , %lu \n",SC_MULT,SC_SHIFT , (unsigned long)reg );
	ctr = reg;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
	sched_clock_register(mstar_sched_clock, 32, rate);
#else
	setup_sched_clock(mstar_sched_clock, 32, rate);
#endif
}
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
