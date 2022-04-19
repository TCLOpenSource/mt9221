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
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 1999,2000 MIPS Technologies, Inc.  All rights reserved.
 *
 * ########################################################################
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * ########################################################################
 *
 * Reset the MIPS boards.
 *
 */
#include <linux/pm.h>

#include <asm/io.h>
#include <asm/reboot.h>
#include <asm/mips-boards/generic.h>
#if defined(CONFIG_MIPS_ATLAS)
#include <asm/mips-boards/atlas.h>
#endif

#if defined(CONFIG_MSTAR_TITANIA3) || \
    defined(CONFIG_MSTAR_TITANIA10) || \
    defined(CONFIG_MSTAR_TITANIA4) || \
    defined(CONFIG_MSTAR_URANUS4) || \
    defined(CONFIG_MSTAR_TITANIA8) || \
    defined(CONFIG_MSTAR_TITANIA9) || \
    defined(CONFIG_MSTAR_JANUS2) || \
    defined(CONFIG_MSTAR_TITANIA11) || \
    defined(CONFIG_MSTAR_TITANIA12) || \
    defined(CONFIG_MSTAR_TITANIA13) || \
    defined(CONFIG_MSTAR_AMBER1) || \
    defined(CONFIG_MSTAR_KRONUS) || \
    defined(CONFIG_MSTAR_KAPPA) || \
    defined(CONFIG_MSTAR_KRITI) || \
    defined(CONFIG_MSTAR_AMBER6) || \
    defined(CONFIG_MSTAR_AMBER7) || \
    defined(CONFIG_MSTAR_AMETHYST) || \
    defined(CONFIG_MSTAR_KAISERIN) || \
    defined(CONFIG_MSTAR_AMBER5) || \
    defined(CONFIG_MSTAR_EMERALD) || \
    defined(CONFIG_MSTAR_NUGGET) || \
	defined(CONFIG_MSTAR_NIKON) || \
	defined(CONFIG_MSTAR_KENYA) || \
	defined(CONFIG_MSTAR_KERES) || \
	defined(CONFIG_MSTAR_MILAN)
#define REG_WDT_BASE    0xbf006000
#elif defined(CONFIG_MSTAR_EUCLID)
#define REG_WDT_BASE    0xbf0078c0
#else // CONFIG_MSTAR_TITANIA2
#define REG_WDT_BASE    0xbf8078c0
#endif

static void mips_machine_restart(char *command);
static void mips_machine_halt(void);
#if defined(CONFIG_MIPS_ATLAS)
static void atlas_machine_power_off(void);
#endif

#if defined(CONFIG_MSTAR_TITANIA3) || \
    defined(CONFIG_MSTAR_TITANIA10) || \
    defined(CONFIG_MSTAR_TITANIA4) || \
    defined(CONFIG_MSTAR_URANUS4) || \
    defined(CONFIG_MSTAR_TITANIA9) || \
    defined(CONFIG_MSTAR_JANUS2) || \
    defined(CONFIG_MSTAR_TITANIA11) || \
    defined(CONFIG_MSTAR_TITANIA13) || \
    defined(CONFIG_MSTAR_AMBER1) || \
    defined(CONFIG_MSTAR_KRONUS) || \
    defined(CONFIG_MSTAR_KAPPA) || \
    defined(CONFIG_MSTAR_KRITI) || \
    defined(CONFIG_MSTAR_AMBER6) || \
    defined(CONFIG_MSTAR_AMBER7) || \
    defined(CONFIG_MSTAR_AMETHYST) || \
    defined(CONFIG_MSTAR_KAISERIN) || \
    defined(CONFIG_MSTAR_EMERALD) || \
    defined(CONFIG_MSTAR_NUGGET) || \
    defined(CONFIG_MSTAR_KENYA) || \
    defined(CONFIG_MSTAR_KERES) || \
    defined(CONFIG_MSTAR_MILAN)
#define REG_PM_SLEEP_BASE    0xbf001C00
#elif defined(CONFIG_MSTAR_TITANIA8) || defined(CONFIG_MSTAR_TITANIA12) || \
    defined(CONFIG_MSTAR_AMBER5) || \
	defined(CONFIG_MSTAR_NIKON)
#define REG_PM_MSIC_BASE	 0xbf005C00
#define REG_PM_SLEEP_BASE    REG_PM_MSIC_BASE
#endif
static void mips_machine_restart(char *command)
{
#if 0
	unsigned int __iomem *softres_reg =
		ioremap(SOFTRES_REG, sizeof(unsigned int));

	__raw_writel(GORESET, softres_reg);
#elif defined(CONFIG_MSTAR_TITANIA3) || \
      defined(CONFIG_MSTAR_TITANIA10) || \
      defined(CONFIG_MSTAR_TITANIA4) || \
      defined(CONFIG_MSTAR_URANUS4) || \
      defined(CONFIG_MSTAR_TITANIA8) || \
      defined(CONFIG_MSTAR_TITANIA9) || \
      defined(CONFIG_MSTAR_JANUS2) || \
      defined(CONFIG_MSTAR_TITANIA11) || \
      defined(CONFIG_MSTAR_TITANIA12) || \
      defined(CONFIG_MSTAR_TITANIA13) || \
      defined(CONFIG_MSTAR_AMBER1) || \
      defined(CONFIG_MSTAR_KRONUS) || \
      defined(CONFIG_MSTAR_KAPPA) || \
      defined(CONFIG_MSTAR_KRITI) || \
      defined(CONFIG_MSTAR_AMBER6) || \
      defined(CONFIG_MSTAR_AMBER7) || \
      defined(CONFIG_MSTAR_AMETHYST) || \
      defined(CONFIG_MSTAR_KAISERIN) || \
      defined(CONFIG_MSTAR_AMBER5) || \
      defined(CONFIG_MSTAR_EMERALD) || \
      defined(CONFIG_MSTAR_NUGGET) || \
	  defined(CONFIG_MSTAR_NIKON) || \
	  defined(CONFIG_MSTAR_KENYA) || \
	  defined(CONFIG_MSTAR_KERES) || \
	  defined(CONFIG_MSTAR_MILAN)

#if 1
    // fire reset for global chip, active by password (8'h79)
    // 1. set to default first
    *(volatile unsigned long*)(REG_PM_SLEEP_BASE + 0xB8) = 0xff;    // reg_top_sw_rst
    // 2. active by password (8'h79)
    *(volatile unsigned long*)(REG_PM_SLEEP_BASE + 0xB8) = 0x79;    // reg_top_sw_rst
#else
    unsigned int nSecond = 1;
    unsigned int nCount = nSecond * 12000000;  // 12MHz

    *(volatile unsigned long*)(REG_WDT_BASE + 0x00)   = 0x01;                      // WDT re-start
    *(volatile unsigned long*)(REG_WDT_BASE + 0x10)   = nCount & 0xFFFF;           // WDT_PERIOD_L
    *(volatile unsigned long*)(REG_WDT_BASE + 0x14)   = (nCount >> 16) & 0xFFFF;   // WDT_PERIOD_H
#endif

#else // CONFIG_MSTAR_TITANIA2 // TODO: please implement "#elif defined(CONFIG_MSTAR_EUCLID)"
	*(volatile unsigned long*)(REG_WDT_BASE)        = 0x5678;   //WDT_KEY
	*(volatile unsigned long*)(REG_WDT_BASE + 4)    = 0xFFFE;   //WDT_SEL
	*(volatile unsigned long*)(REG_WDT_BASE + 12)   = 0x6;      //WDT_CTRL
	*(volatile unsigned long*)(REG_WDT_BASE + 8)    = 0x10;     //WDT_INT_SEL
#endif
}

static void mips_machine_halt(void)
{
#if 0
	unsigned int __iomem *softres_reg =
		ioremap(SOFTRES_REG, sizeof(unsigned int));

	__raw_writel(GORESET, softres_reg);
#elif defined(CONFIG_MSTAR_TITANIA3) || \
   	  defined(CONFIG_MSTAR_TITANIA10) || \
      defined(CONFIG_MSTAR_TITANIA4) || \
      defined(CONFIG_MSTAR_URANUS4) || \
      defined(CONFIG_MSTAR_TITANIA8) || \
      defined(CONFIG_MSTAR_TITANIA9) || \
      defined(CONFIG_MSTAR_JANUS2) || \
      defined(CONFIG_MSTAR_TITANIA11) || \
      defined(CONFIG_MSTAR_TITANIA12) || \
      defined(CONFIG_MSTAR_TITANIA13) || \
      defined(CONFIG_MSTAR_AMBER1) || \
      defined(CONFIG_MSTAR_KRONUS) || \
      defined(CONFIG_MSTAR_KAPPA) || \
      defined(CONFIG_MSTAR_KRITI) || \
      defined(CONFIG_MSTAR_AMBER6) || \
      defined(CONFIG_MSTAR_AMBER7) || \
      defined(CONFIG_MSTAR_AMETHYST) || \
      defined(CONFIG_MSTAR_KAISERIN) || \
      defined(CONFIG_MSTAR_AMBER5) || \
      defined(CONFIG_MSTAR_EMERALD) || \
      defined(CONFIG_MSTAR_NUGGET) || \
	  defined(CONFIG_MSTAR_NIKON) || \
	  defined(CONFIG_MSTAR_KENYA) || \
	  defined(CONFIG_MSTAR_KERES) || \
	  defined(CONFIG_MSTAR_MILAN)

#if 1
    // fire reset for global chip, active by password (8'h79)
    // 1. set to default first
    *(volatile unsigned long*)(REG_PM_SLEEP_BASE + 0xB8) = 0xff;    // reg_top_sw_rst
    // 2. active by password (8'h79)
    *(volatile unsigned long*)(REG_PM_SLEEP_BASE + 0xB8) = 0x79;    // reg_top_sw_rst
#else
    unsigned int nSecond = 1;
    unsigned int nCount = nSecond * 12000000;  // 12MHz

    *(volatile unsigned long*)(REG_WDT_BASE + 0x00)   = 0x01;                      // WDT re-start
    *(volatile unsigned long*)(REG_WDT_BASE + 0x10)   = nCount & 0xFFFF;           // WDT_PERIOD_L
    *(volatile unsigned long*)(REG_WDT_BASE + 0x14)   = (nCount >> 16) & 0xFFFF;   // WDT_PERIOD_H
#endif

#else // CONFIG_MSTAR_TITANIA2 // TODO: please implement "#elif defined(CONFIG_MSTAR_EUCLID)"
	*(volatile unsigned long*)(REG_WDT_BASE)        = 0x5678;   //WDT_KEY
	*(volatile unsigned long*)(REG_WDT_BASE + 4)    = 0xFFFE;   //WDT_SEL
	*(volatile unsigned long*)(REG_WDT_BASE + 12)   = 0x6;      //WDT_CTRL
	*(volatile unsigned long*)(REG_WDT_BASE + 8)    = 0x10;     //WDT_INT_SEL
#endif
}

#if defined(CONFIG_MIPS_ATLAS)
static void atlas_machine_power_off(void)
{
	unsigned int __iomem *psustby_reg = ioremap(ATLAS_PSUSTBY_REG, sizeof(unsigned int));

	writew(ATLAS_GOSTBY, psustby_reg);
}
#endif

void mips_reboot_setup(void)
{
	_machine_restart = mips_machine_restart;
	_machine_halt = mips_machine_halt;
#if defined(CONFIG_MIPS_ATLAS)
	pm_power_off = atlas_machine_power_off;
#endif
#if defined(CONFIG_MIPS_MALTA) || defined(CONFIG_MIPS_SEAD)
	pm_power_off = mips_machine_halt;
#endif
}
