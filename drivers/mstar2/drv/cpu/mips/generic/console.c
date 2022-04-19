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
 * Putting things on the screen/serial line using YAMONs facilities.
 */
#include <linux/console.h>
#include <linux/init.h>
//#include <linux/serial_reg.h>
#include <asm/io.h>

#ifdef CONFIG_MIPS_ATLAS
#include <asm/mips-boards/atlas.h>

#ifdef CONFIG_CPU_LITTLE_ENDIAN
#define PORT(offset) (ATLAS_UART_REGS_BASE     + ((offset)<<3))
#else
#define PORT(offset) (ATLAS_UART_REGS_BASE + 3 + ((offset)<<3))
#endif

#elif defined(CONFIG_MIPS_SEAD)

#include <asm/mips-boards/sead.h>

#ifdef CONFIG_CPU_LITTLE_ENDIAN
#define PORT(offset) (SEAD_UART0_REGS_BASE     + ((offset)<<3))
#else
#define PORT(offset) (SEAD_UART0_REGS_BASE + 3 + ((offset)<<3))
#endif

#else

#define PORT(offset) (0x3f8 + (offset))

#endif

#ifdef CONFIG_MSTAR_TITANIA2
#include "../arch/mips/mips-boards/titania2/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF801300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA3
#include "../arch/mips/mips-boards/titania3/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA10
#include "../arch/mips/mips-boards/titania10/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_EUCLID
#include "../arch/mips/mips-boards/euclid/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_OBERON
#include "../arch/mips/mips-boards/oberon/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF808c80 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_URANUS4
#include "../arch/mips/mips-boards/uranus4/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA4
#include "../arch/mips/mips-boards/titania4/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA8
#include "../arch/mips/mips-boards/titania8/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA9
#include "../arch/mips/mips-boards/titania9/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA12
#include "../arch/mips/mips-boards/titania12/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA13
#include "../arch/mips/mips-boards/titania13/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_AMBER1
#include "../arch/mips/mips-boards/amber1/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_AMBER6
#include "../arch/mips/mips-boards/amber6/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_AMBER7
#include "../arch/mips/mips-boards/amber7/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_AMETHYST
#include "../arch/mips/mips-boards/amethyst/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_TITANIA11
#include "../arch/mips/mips-boards/titania11/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_JANUS2
#include "../arch/mips/mips-boards/janus2/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KRONUS
#include "../arch/mips/mips-boards/kronus/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KAISERIN
#include "../arch/mips/mips-boards/kaiserin/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KAPPA
#include "../arch/mips/mips-boards/kappa/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KRITI
#include "../arch/mips/mips-boards/kriti/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KENYA
#include "../arch/mips/mips-boards/kenya/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KERES
#include "../arch/mips/mips-boards/keres/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_KELTIC
#include "../arch/mips/mips-boards/keltic/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_AMBER5
#include "../arch/mips/mips-boards/amber5/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_EMERALD
#include "../arch/mips/mips-boards/emerald/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_NUGGET
#include "../arch/mips/mips-boards/nugget/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_NIKON
#include "../arch/mips/mips-boards/nikon/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

#ifdef CONFIG_MSTAR_MILAN
#include "../arch/mips/mips-boards/milan/serial_reg.h"
#define UART_REG(addr) *((volatile unsigned int*)(0xBF201300 + ((addr)<< 2)))
#endif

static u32 UART16550_READ(u8 addr)
{
    u32 data;
    //u32 dummy;

    if (addr>80) //ERROR: Not supported
    {
        //printk("R: %d\n",addr);
        //while(1);
        return(0);
    }

	data = UART_REG(addr);

    //dummy = UART_REG(3) ;

    return(data);
}

static void UART16550_WRITE(u8 addr, u8 data)
{
    if (addr>80) //ERROR: Not supported
    {
        printk("W: %d\n",addr);
        //while(1);
        return;
    }

	UART_REG(addr) = data;

    //data = UART_REG(3) ;
}

static inline unsigned int serial_in(int offset)
{
	//return inb(PORT(offset));
	return UART16550_READ(offset);
}

static inline void serial_out(int offset, int value)
{
	//outb(value, PORT(offset));
	UART16550_WRITE(offset, value);
}

int prom_putchar(char c)
{
	while ((serial_in(UART_LSR) & UART_LSR_THRE) == 0)
		;
	serial_out(UART_TX, c);

	return 1;
}
