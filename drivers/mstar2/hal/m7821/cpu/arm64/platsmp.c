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
 * Spin Table SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <asm/cpu_ops.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>

static phys_addr_t cpu_release_addr[NR_CPUS];
extern ptrdiff_t mstar_pm_base;

phys_addr_t magic_number_address;

/*
 * Write secondary_holding_pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	void *start = (void *)&secondary_holding_pen_release;
	unsigned long size = sizeof(secondary_holding_pen_release);

	secondary_holding_pen_release = val;
	__flush_dcache_area(start, size);
}

int __cpuinit mstar_smp_spin_table_boot_cpu(unsigned int cpu)
{
	unsigned long timeout;

	/*
	 * Update the pen release flag.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send an event, causing the secondaries to read pen_release.
	 */
	sev();

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		if (secondary_holding_pen_release == INVALID_HWID)
			break;
        udelay(10);
	}

    return secondary_holding_pen_release != INVALID_HWID ? -ENOSYS : 0;
}

void __cpuinit mstar_smp_spin_table_postboot_cpu(void)
{
    write_pen_release(INVALID_HWID);
}

static int __init mstar_smp_spin_table_init_cpu(struct device_node *dn, unsigned int cpu)
{
	/*
	 * Determine the address from which the CPU is polling.
	 */
	if (of_property_read_u64(dn, "cpu-release-addr",
				 &cpu_release_addr[cpu])) {
		pr_err("CPU %d: missing or invalid cpu-release-addr property\n",
		       cpu);

		return -1;
	}

	return 0;
}

int mstar_smp_spin_table_prepare_cpu(unsigned int cpu)
{
    ptrdiff_t secondary_lo_addr_reg;
    ptrdiff_t secondary_hi_addr_reg;
    ptrdiff_t secondary_magic_reg;

    //printk("\033[31mFunction = %s, Line = %d, for cpu %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, cpu);
    if (!cpu_release_addr[cpu])
    return -ENODEV;

    //printk("\033[35mFunction = %s, Line = %d, write to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, (unsigned long)cpu_release_addr[cpu]);
    secondary_lo_addr_reg = mstar_pm_base + (cpu_release_addr[cpu] << 1) ;
    secondary_hi_addr_reg = secondary_lo_addr_reg + 4;
    secondary_magic_reg = secondary_lo_addr_reg + 8;

    writel_relaxed((__pa(secondary_holding_pen) & 0x000000000000ffff), (void*)secondary_lo_addr_reg);
    writel_relaxed((__pa(secondary_holding_pen) >> 16), (void*)secondary_hi_addr_reg);
    writel_relaxed(0xbabe, (void*)secondary_magic_reg);

    magic_number_address = secondary_magic_reg;

    __flush_dcache_area((void *)secondary_lo_addr_reg, sizeof(phys_addr_t));
    __flush_dcache_area((void *)secondary_hi_addr_reg, sizeof(phys_addr_t));
    __flush_dcache_area((void *)secondary_magic_reg, sizeof(unsigned short));

    /*
    * Send an event to wake up the secondary CPU.
    */
    sev();

    return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
#if defined(CONFIG_MP_MSTAR_STR_BASE)
static DECLARE_COMPLETION(cpu_killed);
extern void Chip_Flush_Cache_All_Single(void);
extern void fpsimd_clear_state(void);
#endif
void mstar_smp_spin_table_die_cpu(unsigned int cpu)
{
#if defined(CONFIG_MP_MSTAR_STR_BASE)
    complete(&cpu_killed);
    fpsimd_clear_state();
    Chip_Flush_Cache_All_Single();
    for (;;) {
        wfe();
        if (secondary_holding_pen_release == cpu_logical_map(cpu)) {
            break;
        }
    }

    /*
     * Do not return to the idle loop - jump back to the secondary
     * cpu initialisation.  There's some initialisation which needs
     * to be repeated to undo the effects of taking the CPU offline.
     * ps. x29 is the frame pointer.
     *
     * from 3.10.40 smp.c
     */
    __asm__("mov sp, %0\n"
            "mov x29, #0\n"
            "b secondary_start_kernel"
            :: "r" (secondary_data.stack));
#endif
}

int mstar_smp_spin_table_kill_cpu(unsigned int cpu)
{
#if defined(CONFIG_MP_MSTAR_STR_BASE)
    return wait_for_completion_timeout(&cpu_killed, 5000);
#else
    return 0;
#endif
}

int mstar_smp_spin_table_disable_cpu(unsigned int cpu)
{
#if defined(CONFIG_MP_MSTAR_STR_BASE)
    /*
     * we don't allow CPU 0 to be shutdown (it is still too special
     * e.g. clock tick interrupts)
     */
    return cpu == 0 ? -EPERM : 0;
#else
    return 0;
#endif
}
#endif

#ifdef CONFIG_ARM64_CPU_SUSPEND
int mstar_smp_spin_table_suspend_cpu(unsigned long arg)
{
#if defined(CONFIG_MP_MSTAR_STR_BASE)
    extern int sleep_suspend_ret(void);
    return sleep_suspend_ret();
#else
    return 0;
#endif
}

#endif

const struct cpu_operations mstar_smp_spin_table __initconst = {
	.name		= "mstar-spin-table",
	.cpu_init 	= mstar_smp_spin_table_init_cpu,
	.cpu_prepare	= mstar_smp_spin_table_prepare_cpu,
	.cpu_boot   = mstar_smp_spin_table_boot_cpu,
	.cpu_postboot = mstar_smp_spin_table_postboot_cpu,
#ifdef CONFIG_HOTPLUG_CPU
    .cpu_die    = mstar_smp_spin_table_die_cpu,
    .cpu_kill    = mstar_smp_spin_table_kill_cpu,
    .cpu_disable    = mstar_smp_spin_table_disable_cpu,
#endif
#ifdef CONFIG_ARM64_CPU_SUSPEND
    .cpu_suspend = mstar_smp_spin_table_suspend_cpu,
#endif

};
