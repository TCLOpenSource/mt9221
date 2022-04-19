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
 *  linux/arch/arm/plat-versatile/hotplug.c
 *
 *  Copyright (C) 2010 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/completion.h>
#include <linux/version.h>
#include <asm/cacheflush.h>

#include <plat/hotplug.h>
#include <asm/cputype.h>


static DECLARE_COMPLETION(cpu_killed);


static unsigned int get_smp_ctrl_mask(void)
{
    switch (read_cpuid_id() & 0xfff0)
    {
        case 0xb020: /* 11mpcore */
            return 0x20;
        case 0xc090: /* Cortex-A9 */
            return 0x40;
        default:
            return 0;
    }
}
extern void Chip_Flush_Cache_All(void);

static inline void cpu_enter_lowpower(void)
{
    unsigned int v, smp_ctrl = get_smp_ctrl_mask();

    flush_cache_all();
    //Chip_Flush_Cache_All();
    dsb();
    asm volatile(
    /*
     * Turn off coherency
     */
    "   mrc p15, 0, %0, c1, c0, 1\n"
    "   bic %0, %0, %1\n"
    "   mcr p15, 0, %0, c1, c0, 1\n"
    /* DSB */
    "       mcr     p15, 0, %2, c7, c10, 4\n"
    /* Disable D-cache */
    "   mrc p15, 0, %0, c1, c0, 0\n"
    "   bic %0, %0, #0x04\n"
    "   mcr p15, 0, %0, c1, c0, 0\n"
      : "=&r" (v)
      : "r" (smp_ctrl), "r" (0)
      : "memory");
    isb();
}

static inline void cpu_leave_lowpower(void)
{
    unsigned int v, smp_ctrl = get_smp_ctrl_mask();

    flush_cache_all();
    //Chip_Flush_Cache_All();
    dsb();
    asm volatile(   "mrc    p15, 0, %0, c1, c0, 0\n"
    "   orr %0, %0, #0x04\n"
    "   mcr p15, 0, %0, c1, c0, 0\n"
    "   mrc p15, 0, %0, c1, c0, 1\n"
    "   orr %0, %0, %1\n"
    "   mcr p15, 0, %0, c1, c0, 1\n"
      : "=&r" (v)
      : "r" (smp_ctrl)
      : "memory");
    isb();
}


#if LINUX_VERSION_CODE > KERNEL_VERSION(3,1,10)
int platform_mstar_cpu_kill(unsigned int cpu)
#else
int platform_cpu_kill(unsigned int cpu)
#endif
{
    return wait_for_completion_timeout(&cpu_killed, 5000);
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,1,10)
void platform_mstar_cpu_die(unsigned int cpu)
#else
void platform_cpu_die(unsigned int cpu)
#endif
{
#ifdef DEBUG
    unsigned int this_cpu = hard_smp_processor_id();

    if (cpu != this_cpu) {
        printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
               this_cpu, cpu);
        BUG();
    }
#endif

	printk(KERN_NOTICE "CPU%u: shutdown\n", cpu);
	complete(&cpu_killed);

	cpu_enter_lowpower();

	   /*
     * there is no power-control hardware on this platform, so all
     * we can do is put the core into WFI; this is safe as the calling
     * code will have already disabled interrupts
     */
    for (;;) {
        /*
         * here's the WFI
         */
        extern volatile int pen_release ;
        asm volatile("wfi" : : : "memory");

		//extern volatile int pen_release ;
        if (pen_release == cpu) {
            /*
             * OK, proper wakeup, we're done
             */
            break;
        }

        /*
         * getting here, means that we have come out of WFI without
         * having been woken up - this shouldn't happen
         *
         * The trouble is, letting people know about this is not really
         * possible, since we are currently running incoherently, and
         * therefore cannot safely call printk() or anything else
         */
#ifdef DEBUG
        printk("CPU%u: spurious wakeup call\n", cpu);
#endif
    }
    cpu_leave_lowpower();
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,1,10)
int platform_mstar_cpu_disable(unsigned int cpu)
#else
int platform_cpu_disable(unsigned int cpu)
#endif
{
    /*
     * we don't allow CPU 0 to be shutdown (it is still too special
     * e.g. clock tick interrupts)
     */
    return cpu == 0 ? -EPERM : 0;
}
