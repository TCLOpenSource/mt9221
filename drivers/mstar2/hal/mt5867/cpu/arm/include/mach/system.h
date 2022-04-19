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
 *  arch/arm/mach-vexpress/include/mach/system.h
 *
 *  Copyright (C) 2003 ARM Limited
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
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H
#include "mdrv_pm.h"

#define REG_PM_MISC_BASE     0xfd005C00
    #define PM_RST_OFSET (0x52 * 2)
        #define PM_RST_BIT (1 << 12)
#define REG_WDT_BASE         0xfd006000
#define MST_XTAL_CLOCK_HZ   (12000000UL) 

#define REG_PM_POR_STATUS_BASE (0xfd000C00)
#define PM_POR_OFSET (0x6 * 2)

#define MAGIC_PANIC 0x9631

#define REG_PMRTC_BASE (0xfd002400UL)

#define WAKEUP_TIME 5 //  second
// RTC0
#define REG_PMRTC_CTRL       ((REG_PMRTC_BASE + 0x00UL))
    #define PMRTC_CTRL_READ_EN                  (1 << 4)
    #define PMRTC_CTRL_INT_MASK                 (1 << 5)
    #define PMRTC_CTRL_INT_CLEAR                (1 << 7)
#define REG_PMRTC_MATCH_VAL_0  ((REG_PMRTC_BASE + 0x1CUL))
#define REG_PMRTC_MATCH_VAL_1  ((REG_PMRTC_BASE + 0x20UL))
#define REG_PMRTC_MATCH_VAL_2  ((REG_PMRTC_BASE + 0x24UL))
#define REG_PMRTC_MATCH_VAL_3  ((REG_PMRTC_BASE + 0x28UL))
#define REG_PMRTC_CNT_0        ((REG_PMRTC_BASE + 0x2CUL))
#define REG_PMRTC_CNT_1        ((REG_PMRTC_BASE + 0x30UL))
#define REG_PMRTC_CNT_2        ((REG_PMRTC_BASE + 0x34UL))
#define REG_PMRTC_CNT_3        ((REG_PMRTC_BASE + 0x38UL))

extern int is_panic;
extern void  Chip_Flush_Cache_All(void);

#define writel(y, x) *(volatile unsigned int*)(x)=(y)
#define readl(x) (*(volatile unsigned int*)(x))

static void dram_self_refresh(void)
{
    unsigned int counter = 0;
    PM_WakeCfg_t stCfgDef = {0};

    // config RTC0
    writel(readl(REG_PMRTC_CTRL)|PMRTC_CTRL_READ_EN, REG_PMRTC_CTRL);
    udelay(100);
    counter = readl(REG_PMRTC_CNT_1) << 16 | \
              readl(REG_PMRTC_CNT_0);
    //pr_emerg("counter = 0x%x\n", counter);

    // set wakeup source RTC0
    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
    stCfgDef.bPmWakeEnableRTC0 = 1;
    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (void *)&stCfgDef, sizeof(stCfgDef));

    MDrv_PM_Panic();

    // write magic value for sboot
    writel(MAGIC_PANIC, REG_PM_POR_STATUS_BASE + PM_POR_OFSET);

    writel(readl(REG_PMRTC_CTRL)|PMRTC_CTRL_INT_CLEAR, REG_PMRTC_CTRL);
    writel(((counter + WAKEUP_TIME) & 0x0000FFFF) >> 0 , REG_PMRTC_MATCH_VAL_0);
    writel(((counter + WAKEUP_TIME) & 0xFFFF0000) >> 16, REG_PMRTC_MATCH_VAL_1);

    writel(0x0, REG_PMRTC_MATCH_VAL_2);
    writel(0x0, REG_PMRTC_MATCH_VAL_3);

    writel(readl(REG_PMRTC_CTRL) & ~PMRTC_CTRL_INT_MASK, REG_PMRTC_CTRL);
    // release pm51 reset
    writel(readl(REG_PM_MISC_BASE + PM_RST_OFSET) | PM_RST_BIT, REG_PM_MISC_BASE + PM_RST_OFSET);
}
static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_MTK_PSTORE
    if(is_panic == 1)
    {
        dram_self_refresh();
        while(1);
    }
#endif
    *(volatile unsigned long*)(REG_PM_MISC_BASE + 0xB8) = 0x79;    // reg_top_sw_rst   
#if 0 
    // set WDT count down timer
    *(volatile unsigned long*)(REG_WDT_BASE + 0x10) = BOOT_DELAY_MILLISECOND & 0x0000FFFF;               
    *(volatile unsigned long*)(REG_WDT_BASE + 0x14) = BOOT_DELAY_MILLISECOND  >> 16  ;
#endif 
}

#endif
