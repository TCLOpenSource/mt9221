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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clkdev.h>
#include <asm/smp_twd.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/sched_clock.h>
#else
#include <asm/sched_clock.h>
#include <asm/irq.h>
#endif
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/of_irq.h>

#include <mach/io.h>
#include <mach/timex.h>
#include <linux/timer.h>
#include <plat/localtimer.h>
#include "chip_int.h"
#include <plat/sched_clock.h>


#include <linux/clk-provider.h>
//------------------------------------------------------------------------------
//  Macros
//------------------------------------------------------------------------------

//Mstar PIU Timer
#define TIMER_ENABLE			(0x1)
#define TIMER_TRIG				(0x2)
#define TIMER_INTERRUPT		    (0x100)
#define TIMER_CLEAR			    (0x4)
#define TIMER_CAPTURE       	(0x8)
#define ADDR_TIMER_MAX_LOW 	    (0x2<<2)
#define ADDR_TIMER_MAX_HIGH 	(0x3<<2)
#define PIU_TIMER_FREQ_KHZ	    (12000)

//ARM Global Timer
static int GLB_TIMER_FREQ_KHZ;  // PERICLK = CPUCLK/2
static unsigned long interval;
static int ORI_FREQ_KHZ;

#ifdef CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE
#define RTC1_TIMER_DISABLE  (0x0)
#define RTC1_TIMER_ENABLE   (0x3)
#define RTC1_TIMER_FREQ_CW_OFFEST_L (0x1)
#define RTC1_TIMER_FREQ_CW_OFFEST_H (0x2)
#define RTC1_TIMER_MATCH_VAL_OFFEST_1 (0x7) 
#define RTC1_TIMER_MATCH_VAL_OFFEST_2 (0x8)
#define RTC1_TIMER_MATCH_VAL_OFFEST_3 (0x9)
#define RTC1_TIMER_MATCH_VAL_OFFEST_4 (0xa)
#define CLK_PM_SLEEP_OFFEST (0x22)
#define CLK_EXT_XTALI_BUF (0x4)
static int STATIC_TIMER_FREQ_KHZ;
extern unsigned long long RTC1_timer_read64(void);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
extern u64 arch_counter_read(struct clocksource *cs);
#else
extern cycle_t arch_counter_read(struct clocksource *cs);
#endif

/*
 * xtal: fixed 12M 
 * mpl: fixed 216M
 * pll, variable, 
 * ex: 1.6G --> 2717908.8 / 1.6 = 0x19EB85
 * bank 0x110C, reg 0x60 = 0xEB85
 *              reg 0x61 = 0x19
 * reverse: 2717908.8 / (reg 0x61 << 16 + reg 0x60) * 1000 = 1600(MHz)
 * --> 2717908800 / (reg 0x61 << 16 + reg 0x60) = 1600(MHz)
 *
 * return unit: MHz
 */
#define XTAL 0x0000 /* 12M */
#define MPL  0x0400 /* 216M */
#define PLL  0x8000 /* freq. depends on setting */
int query_frequency(unsigned int cpu_id)
{
#ifdef CONFIG_MSTAR_ARM_BD_FPGA
	return 12;
#else
	unsigned short freq_type, freq_type1;
	unsigned int freq;
	u16 reg_val = 0;
	unsigned int mcu_arm_bank = 0x101d00;
	unsigned int mstar_pm_base = 0x1f000000;

	freq_type = reg_readw(mstar_pm_base + mcu_arm_bank * 2 + 0xF8 * 2);
	freq_type1 = reg_readw(mstar_pm_base + mcu_arm_bank * 2 + 0xFA * 2);
	if(freq_type & 0x1)
	{
		/* CA55 */
		reg_val = reg_readw(mstar_pm_base + ((mcu_arm_bank + 0xf6)<<1));
		reg_val = reg_val & ~0x01;
		reg_writew(reg_val, (mstar_pm_base + ((mcu_arm_bank + 0xf6)<<1)));

		reg_val = reg_readw(mstar_pm_base + ((mcu_arm_bank + 0xf6)<<1));
		reg_val = reg_val | 0x01;
		reg_writew(reg_val, (mstar_pm_base + ((mcu_arm_bank + 0xf6)<<1)));
                udelay(300);
                while( ((reg_readw((mstar_pm_base + ((mcu_arm_bank + 0xf6)<<1))) & 0x3) != 0x3) );
		freq = reg_readw(mstar_pm_base + ((mcu_arm_bank + 0xf4)<<1));
		freq *= 16;
		freq *= 1000;
		freq /= 83333;
		//printk("\033[0;32;31m [Kernel] %s %d CPU:%d frequency : %ldM \033[m\n", __func__, __LINE__, cpu_id, freq);
		return freq;
	}
	else if (freq_type1 & 0x1)
		return 216;
	else
		return 24;//chenyang request for this case modify to 24M
#endif
}
EXPORT_SYMBOL(query_frequency);

#define USE_GLOBAL_TIMER 1

static unsigned int evt_timer_cnt;
static unsigned int clksrc_base;
static unsigned int clkevt_base;

struct clocksource clocksource_timer = {
	.name		= "timer1",
	.rating		= 200,
	.read           = arch_counter_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};
EXPORT_SYMBOL(clocksource_timer);

void __init chip_clocksource_init(unsigned int base)
{

	struct clocksource *cs = &clocksource_timer;
	clksrc_base = base;
#ifdef CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE 
	  
	CLRREG16(clksrc_base, RTC1_TIMER_DISABLE);  
	OUTREG16(clksrc_base+(RTC1_TIMER_FREQ_CW_OFFEST_L<<2),0x0);
	OUTREG16(clksrc_base+(RTC1_TIMER_FREQ_CW_OFFEST_H<<2),0x0);

	//set max period
					  	  
	OUTREG16(clksrc_base+(RTC1_TIMER_MATCH_VAL_OFFEST_1<<2),0xFFFF);
	OUTREG16(clksrc_base+(RTC1_TIMER_MATCH_VAL_OFFEST_2<<2),0xFFFF);
	OUTREG16(clksrc_base+(RTC1_TIMER_MATCH_VAL_OFFEST_3<<2),0xFFFF);
	OUTREG16(clksrc_base+(RTC1_TIMER_MATCH_VAL_OFFEST_4<<2),0xFFFF); 
										      
	SETREG16(clksrc_base, RTC1_TIMER_ENABLE);	  
												  	 
	/*
	 * calculate the value of mult    
	 * cycle = ( time(ns) * mult ) >> shift
	 * PERICLK = CPUCLK / 2
	 */
	cs->mult = clocksource_khz2mult(STATIC_TIMER_FREQ_KHZ, cs->shift);
#elif USE_GLOBAL_TIMER 
	PERI_W(GT_CONTROL,0x1); //Enable

	/*
	 * calculate the value of mult    
	 * cycle = ( time(ns) * mult ) >> shift
	 * PERICLK = CPUCLK / 2
	 */
	cs->mult = clocksource_khz2mult(GLB_TIMER_FREQ_KHZ, cs->shift);
#else // USE_GLOBAL_TIMER
	/* setup timer 1 as free-running clocksource */
	//make sure timer 1 is disable
	CLRREG16(clksrc_base, TIMER_ENABLE);

	//set max period
	OUTREG16(clksrc_base+(0x2<<2),0xffff);
	OUTREG16(clksrc_base+(0x3<<2),0xffff);

	//enable timer 1
	SETREG16(clksrc_base, TIMER_ENABLE);

	/*
	 * TODO: need to double check 
	 * calculate the value of mult
	 * cycle = ( time(ns) * mult ) >> shift
     * Mstar timer => 12Mhz,
	 */
	cs->mult = clocksource_khz2mult(GLB_TIMER_FREQ_KHZ, cs->shift);  
#endif // CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE

        #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
                __clocksource_register(cs);
        #else
                clocksource_register(cs);
        #endif

}

/*
 * IRQ handler for the timer
 */
static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	//printk("t");
	/* clear the interrupt */
	evt_timer_cnt=INREG16(clkevt_base+(0x3<<2));
	OUTREG16(clkevt_base+(0x3<<2),evt_timer_cnt);

	//enable timer
    //SETREG16(clkevt_base, TIMER_TRIG);//default

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
void timer_set_mode(enum clock_event_state mode,
                struct clock_event_device *evt)
#else
void timer_set_mode(enum clock_event_mode mode,
                struct clock_event_device *evt)
#endif
{
        unsigned short ctl=TIMER_INTERRUPT;
        switch (mode) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
                case CLOCK_EVT_STATE_PERIODIC:
#else
                case CLOCK_EVT_MODE_PERIODIC:
#endif
                        interval = (PIU_TIMER_FREQ_KHZ*1000 / HZ)  ;
                        OUTREG16(clkevt_base + ADDR_TIMER_MAX_LOW, (interval &0xffff));
                        OUTREG16(clkevt_base + ADDR_TIMER_MAX_HIGH, (interval >>16));
                        ctl|=TIMER_ENABLE;
                        SETREG16(clkevt_base, ctl);
                        break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
                case CLOCK_EVT_STATE_ONESHOT:
#else
                case CLOCK_EVT_MODE_ONESHOT:
#endif
                        /* period set, and timer enabled in 'next_event' hook */
                        ctl|=TIMER_TRIG;
                        SETREG16(clkevt_base, ctl);
                        break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
                case CLOCK_EVT_STATE_SHUTDOWN:
                        CLRREG16(clkevt_base, TIMER_ENABLE);
                        break;
#else
                case CLOCK_EVT_MODE_UNUSED:
                case CLOCK_EVT_MODE_SHUTDOWN:
#endif
                default:
                        break;
        }
}

static int timer_set_next_event(unsigned long next, 
		struct clock_event_device *evt)
{
	//stop timer
	//OUTREG16(clkevt_base, 0x0);

	//set period
	OUTREG16(clkevt_base + ADDR_TIMER_MAX_LOW, (next &0xffff));
	OUTREG16(clkevt_base + ADDR_TIMER_MAX_HIGH, (next >>16));

	//enable timer
	SETREG16(clkevt_base, TIMER_TRIG|TIMER_INTERRUPT);//default

	return 0;
}

static struct clock_event_device clockevent_timer = {
	.name		= "timer0",
	.shift		= 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,49)
	.set_mode       = timer_set_mode,
#endif
        .set_next_event	= timer_set_next_event,
	.rating		= 300,
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
	.cpumask	= cpu_possible_mask,
#else
	.cpumask	= cpu_all_mask,
#endif
};

static struct irqaction timer_irq = {
        .name           = "timer",
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,49)
        .flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
#else
        .flags          = IRQF_TIMER | IRQF_IRQPOLL,
#endif
        .handler        = timer_interrupt,
        .dev_id         = &clockevent_timer,
};

void __init chip_clockevents_init(unsigned int base,unsigned int irq)
{
	struct clock_event_device *evt = &clockevent_timer;

	clkevt_base = base;

	evt->irq = irq;
	/* PIU Timer FRE = 12Mhz */
	evt->mult = div_sc(PIU_TIMER_FREQ_KHZ, NSEC_PER_MSEC, evt->shift); 
	evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
	evt->min_delta_ns = clockevent_delta2ns(0xf, evt);

	setup_irq(irq, &timer_irq);
	clockevents_register_device(evt);
}


#ifdef CONFIG_HAVE_ARM_TWD
static DEFINE_TWD_LOCAL_TIMER(twd_local_timer, chip_MPCORE_TWD, IRQ_LOCALTIMER);

static int __init chip_twd_init(void)
{
    int err = twd_local_timer_register(&twd_local_timer);
    if (err)
        pr_err("twd_local_timer_register failed %d\n", err);
    return err;
}
#else
#define chip_twd_init()  do {} while(0)
#endif

static int __init chip_init_timer_clocks(void)
{
    struct clk *fclk;
    struct clk *armperi_clk;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    fclk = clk_register_fixed_rate(NULL, "fclk", NULL, 0, GLB_TIMER_FREQ_KHZ * 1000 * 2);
#else
    fclk = clk_register_fixed_rate(NULL, "fclk", NULL, CLK_IS_ROOT, GLB_TIMER_FREQ_KHZ * 1000 * 2);
#endif

    armperi_clk = clk_register_fixed_rate(NULL, "arm_peri", "fclk", 0, GLB_TIMER_FREQ_KHZ * 1000);

    /* TODO: use static lookup table? */
    clk_register_clkdev(fclk, "fclk", NULL);
    clk_register_clkdev(armperi_clk, NULL, "smp_twd");

    return 0;
}

extern u32 SC_MULT;
extern u32 SC_SHIFT;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32) && defined(CONFIG_OF)
static int __init chip_init_timer(struct device_node *np)
#else
void __init chip_init_timer(void)
#endif
{
	u32 shift,mult;
	int err;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
#ifdef CONFIG_OF
	int i=0, irq;
	for (;;) {
		irq = irq_of_parse_and_map (np , i++);
		if (!irq)
			break;
	}
#endif
#endif
	// PERIPHCLK = CPU Clock / 2,
	// div 2 later,when CONFIG_GENERIC_CLOCKEVENTS
	// clock event will handle this value
#ifdef CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE // Enable STATIC_TIMER 
	OUTREG16(chip_BASE_REG_PM_SLEEP_PA+(CLK_PM_SLEEP_OFFEST<<2),CLK_EXT_XTALI_BUF); // enable clk_int_xtali_buf
#ifdef CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH  // Both enable STATIC_TIMER and GLOBAL_TIMER_12MHZ
	GLB_TIMER_FREQ_KHZ= 12000;
#endif
	STATIC_TIMER_FREQ_KHZ= 12000;  //12MHZ
	printk("\033[35mStatic Timer Frequency = %d MHz\033[m\n", STATIC_TIMER_FREQ_KHZ/1000);
#else
#ifdef CONFIG_MP_GLOBAL_TIMER_12MHZ_PATCH // Enable GLOBAL_TIMER_12MHZ
	GLB_TIMER_FREQ_KHZ= 12000;
#else
	GLB_TIMER_FREQ_KHZ=(query_frequency(0)*1000/2); 
#endif
#endif
  ORI_FREQ_KHZ= query_frequency(0)*1000;
	printk("\033[35mGlobal Timer Frequency = %d MHz\033[m\n", GLB_TIMER_FREQ_KHZ/1000);
	printk("\033[35mCPU Clock Frequency = %d MHz\033[m\n", query_frequency(0));
#ifdef CONFIG_MP_ANTUTU_BENCHMARK_RISE_PERFORMANCE
	  GLB_TIMER_FREQ_KHZ=(1600*1000/2); 
    ORI_FREQ_KHZ= 1600*1000;
    	printk("\033[35mGlobal Timer Frequency = %d MHz\033[m\n", GLB_TIMER_FREQ_KHZ/1000);
	printk("\033[35mCPU Clock Frequency = %d MHz\033[m\n", query_frequency(0));
#endif // CONFIG_MP_ANTUTU_BENCHMARK_RISE_PERFORMANCE

	

	chip_init_timer_clocks();

#ifdef CONFIG_MP_STATIC_TIMER_CLOCK_SOURCE 
	clocks_calc_mult_shift(&mult, &shift,
			STATIC_TIMER_FREQ_KHZ * 1000, NSEC_PER_SEC, 0);
	printk("fre = %d, mult= %u, shift= %d\n",
			STATIC_TIMER_FREQ_KHZ * 1000, mult, shift);
	SC_SHIFT=shift;
	SC_MULT=mult;

	chip_clocksource_init(chip_BASE_REG_RTC1_PA);
	chip_clockevents_init(chip_BASE_REG_TIMER0_PA,E_FIQ_EXTIMER0);
	mstar_sched_clock_init((void *)(chip_READ_REG_RTC1_VA),STATIC_TIMER_FREQ_KHZ * 1000);
#else
	clocks_calc_mult_shift(&mult, &shift,
			GLB_TIMER_FREQ_KHZ * 1000, NSEC_PER_SEC, 0);
	printk("fre = %d, mult= %d, shift= %d\n",
			GLB_TIMER_FREQ_KHZ * 1000, mult, shift);
	SC_SHIFT=shift;
	SC_MULT=mult;

	mstar_sched_clock_init((void __iomem *)(PERI_VIRT + 0x200),
			GLB_TIMER_FREQ_KHZ * 1000);

	chip_clocksource_init(chip_BASE_REG_TIMER1_PA);
	chip_clockevents_init(chip_BASE_REG_TIMER0_PA,E_FIQ_EXTIMER0);
#endif

	err = chip_twd_init();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
#ifdef CONFIG_OF
	return err;
#endif
#endif
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
CLOCKSOURCE_OF_DECLARE(mstar_twd_timer, "arm,mstar-timer-workaround", chip_init_timer);
#endif

