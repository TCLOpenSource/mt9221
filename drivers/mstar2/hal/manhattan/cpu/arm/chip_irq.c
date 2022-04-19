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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/types.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <linux/irqchip/arm-gic.h>

#include "chip_int.h"

/*
 * 2 bits/per interrupt
 * b'00: level
 * b'10: edge
 */
unsigned int interrupt_configs[MSTAR_CHIP_INT_END / 16+1] =
{
    0x00000000, /*   0~ 15 sgi, don't care */
    0x00000000, /*  16~ 31 ppi, don't care */
    0x00000000, /*  32~ 47 spi, set level for mstar irq */
    0x00000000, /*  64~ 79 spi, set level for mstar irq */
    0x00000000, /*  64~ 79 spi, set level for mstar irq */
    0x00000000, /*  80~ 95 spi, set level for mstar irq */
    0xAAAAAAAA, /*  96~111 spi, set edge for mstar fiq */
    0xAAAAAAAA, /* 112~127 spi, set edge for mstar fiq */
    0xAAAAAAAA, /* 128~143 spi, set edge for mstar fiq */
    0xAAAAAAAA, /* 144~159 spi, set edge for mstar fiq */
                /* set the rest by init_chip_spi_config() */
};

#define BIT_PER_IRQ     2
#define IRQ_PER_UINT    16
#define EDGE            2
#define LEVEL           0

static inline void set_edge(unsigned int irq)
{
    interrupt_configs[irq/IRQ_PER_UINT] |= (EDGE << ((irq % IRQ_PER_UINT) * BIT_PER_IRQ));
}

void init_chip_spi_config(void)
{
    set_edge(170); /* clock switch interrupt */
    set_edge(171); /* riu/xiu timerout interrupt */
    set_edge(172); /* scu event abort interrupt */

    /* neon/fpu exception flag */
    set_edge(174);
    set_edge(175);
    set_edge(176);
    set_edge(177);
    set_edge(178);
    set_edge(179);
    set_edge(180);

    /* neon/fpu exception flag */
    set_edge(183);
    set_edge(184);
    set_edge(185);
    set_edge(186);
    set_edge(187);
    set_edge(188);
    set_edge(189);

    /* neon/fpu exception flag */
    set_edge(192);
    set_edge(193);
    set_edge(194);
    set_edge(195);
    set_edge(196);
    set_edge(197);
    set_edge(198);

    /* neon/fpu exception flag */
    set_edge(201);
    set_edge(202);
    set_edge(203);
    set_edge(204);
    set_edge(205);
    set_edge(206);
    set_edge(207);
}

/* Clear FIQ (Clear is not supported for IRQ) */
void chip_irq_ack(unsigned int irq)
{
    unsigned short tmp;

    if(irq < 16)
    {
        tmp = (0x01 << irq);
        reg_writew(tmp, REG_INT_BASE_PA + (0x2c << 2));
    }
    else if((irq >= 16) && (irq < 32))
    {
        tmp = (0x01 << (irq - 16));
        reg_writew(tmp, REG_INT_BASE_PA + (0x2d << 2));
    }
    else if( (irq >= 32) && (irq < 48))
    {
      tmp = (0x01) << (irq - 32);
        reg_writew(tmp, REG_INT_BASE_PA + (0x2e << 2));
    }
    else if( (irq >= 48) && (irq < 64))
    {
        tmp = (0x01) << (irq - 48);
        reg_writew(tmp, REG_INT_BASE_PA + (0x2f << 2));
    }
    else if( (irq >= 128) && (irq < 144))
    {
        tmp = (0x01) << (irq - 128);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x2c << 2));
    }
     else if((irq >= 144) && (irq < 160))
    {
        tmp = (0x01 << (irq - 144));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x2d << 2));
    }
    else if( (irq >= 160) && (irq < 176))
    {
      tmp = (0x01) << (irq - 160);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x2e << 2));
    }
    else if( (irq >= 176) && (irq < 192))
    {
        tmp = (0x01) << (irq - 176);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x2f << 2));
    }
}

/* Mask IRQ/FIQ */
void chip_irq_mask(unsigned int irq)
{
    unsigned short tmp;

    if(irq <16)
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x24 << 2));
        tmp |= (0x01) << irq;
        reg_writew(tmp, REG_INT_BASE_PA + (0x24 << 2));
    }
    else if((irq >= 16) && (irq < 32))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x25 << 2));
        tmp |= (0x01) << (irq - 16);
        reg_writew(tmp, REG_INT_BASE_PA + (0x25 << 2));
    }
    else if((irq >= 32) && (irq < 48))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x26 << 2));
        tmp |= (0x01) << (irq - 32);
        reg_writew(tmp, REG_INT_BASE_PA + (0x26 << 2));
    }
    else if((irq >= 48) && (irq < 64))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x27 << 2));
        tmp |= (0x01) << (irq - 48);
        reg_writew(tmp, REG_INT_BASE_PA + (0x27 << 2));
    }
    else if((irq >= 64) && (irq < 80))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x34 << 2));
        tmp |= (0x01) << (irq - 64);
        reg_writew(tmp, REG_INT_BASE_PA + (0x34 << 2));
    }
    else if((irq >= 80) && (irq < 96))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x35 << 2));
        tmp |= (0x01) << (irq - 80);
        reg_writew(tmp, REG_INT_BASE_PA + (0x35 << 2));
    }
    else if((irq >= 96) && (irq < 112))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x36 << 2));
        tmp |= (0x01) << (irq - 96);
        reg_writew(tmp, REG_INT_BASE_PA + (0x36 << 2));
    }
    else if((irq >= 112) && (irq < 128))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x37 << 2));
        tmp |= (0x01) << (irq - 112);
        reg_writew(tmp, REG_INT_BASE_PA + (0x37 << 2));
    }
	else if((irq >= 128) && (irq < 144))
	{
		tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x24 << 2));
		tmp |= (0x01) << (irq - 128);
		reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x24 << 2));
	}
	else if((irq >= 144) && (irq < 160))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x25 << 2));
        tmp |= (0x01) << (irq - 144);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x25 << 2));
    }
    else if((irq >= 160) && (irq < 176))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x26 << 2));
        tmp |= (0x01) << (irq - 160);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x26 << 2));
    }
    else if((irq >= 176) && (irq < 192))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x27 << 2));
        tmp |= (0x01) << (irq - 176);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x27 << 2));
    }
    else if((irq >= 192) && (irq < 208))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x34 << 2));
        tmp |= (0x01) << (irq - 192);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x34 << 2));
    }
    else if((irq >= 208) && (irq < 224))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x35 << 2));
        tmp |= (0x01) << (irq - 208);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x35 << 2));
    }
    else if((irq >= 224) && (irq < 240))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x36 << 2));
        tmp |= (0x01) << (irq - 224);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x36 << 2));
    }
    else if((irq >= 240) && (irq < 256))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x37 << 2));
        tmp |= (0x01) << (irq - 240);
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x37 << 2));
    }


}

/* Un-Mask IRQ/FIQ */
void chip_irq_unmask(unsigned int irq)
{
   unsigned short tmp;

    //printk(KERN_WARNING "chip_irq_unmask(irq=%d)\n",irq);

    if(irq < 16)
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x24 << 2));
        tmp &= ~((0x01) << irq);
        reg_writew(tmp, REG_INT_BASE_PA + (0x24 << 2));
    }

    else if((irq >= 16) && (irq < 32))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x25 << 2));
        tmp &= ~((0x01) << (irq - 16));
        reg_writew(tmp, REG_INT_BASE_PA + (0x25 << 2));
    }
    else if((irq >= 32) && (irq < 48))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x26 << 2));
        tmp &= ~((0x01) << (irq - 32));
        reg_writew(tmp, REG_INT_BASE_PA + (0x26 << 2));
    }
    else if((irq >= 48) && (irq < 64))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x27 << 2));
        tmp &= ~((0x01) << (irq - 48));
        reg_writew(tmp, REG_INT_BASE_PA + (0x27 << 2));
    }
    else if((irq >= 64) && (irq < 80))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x34 << 2));
        tmp &= ~((0x01) << (irq - 64));
        reg_writew(tmp, REG_INT_BASE_PA + (0x34 << 2));
    }
    else if((irq >= 80) && (irq < 96))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x35 << 2));
        tmp &= ~((0x01) << (irq - 80));
        reg_writew(tmp, REG_INT_BASE_PA + (0x35 << 2));
    }
    else if((irq >= 96) && (irq < 112))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x36 << 2));
        tmp &= ~((0x01) << (irq - 96));
        reg_writew(tmp, REG_INT_BASE_PA + (0x36 << 2));
    }
    else if((irq >= 112) && (irq < 128))
    {
        tmp = reg_readw(REG_INT_BASE_PA + (0x37 << 2));
        tmp &= ~((0x01) << (irq - 112));
        reg_writew(tmp, REG_INT_BASE_PA + (0x37 << 2));
    }
    else if((irq >= 128) && (irq < 144))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x24 << 2));
	tmp &= ~ ((0x01) << (irq - 128));
	reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x24 << 2));
    }
    else if((irq >= 144) && (irq < 160))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x25 << 2));
        tmp &= ~ ((0x01) << (irq - 144));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x25 << 2));
    }
    else if((irq >= 160) && (irq < 176))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x26 << 2));
        tmp &= ~ ((0x01) << (irq - 160));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x26 << 2));
    }
    else if((irq >= 176) && (irq < 192))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x27 << 2));
        tmp &= ~ ((0x01) << (irq - 176));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x27 << 2));
    }
    else if((irq >= 192) && (irq < 208))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x34 << 2));
        tmp &= ~ ((0x01) << (irq - 192));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x34 << 2));
    }
    else if((irq >= 208) && (irq < 224))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x35 << 2));
        tmp &= ~ ((0x01) << (irq - 208));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x35 << 2));
    }
    else if((irq >= 224) && (irq < 240))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x36 << 2));
        tmp &= ~ ((0x01) << (irq - 224));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x36 << 2));
    }
    else if((irq >= 240) && (irq < 256))
    {
        tmp = reg_readw(REG_INT_HYP_BASE_PA + (0x37 << 2));
        tmp &= ~ ((0x01) << (irq - 240));
        reg_writew(tmp, REG_INT_HYP_BASE_PA + (0x37 << 2));
    }
}


void __iomem *_gic_cpu_base_addr = (void __iomem *) (PERI_VIRT + 0x2000);
void __iomem *_gic_dist_base_addr = (void __iomem *) (PERI_VIRT + 0x1000);

extern void gic_dist_init(unsigned int gic_nr, void __iomem *base, unsigned int irq_start);
extern void gic_cpu_init(unsigned int gic_nr, void __iomem *base);
extern void arm_interrupt_chain_setup(int chain_num);

void __init chip_init_irq(void)
{
#ifndef CONFIG_MP_PLATFORM_MSTAR_LEGANCY_INTR
    unsigned long temp;
#endif

#if 1
    gic_init(0, 29, _gic_dist_base_addr, _gic_cpu_base_addr);
#else
    gic_dist_init(0, gic_dist_base_addr, 29);
    gic_cpu_init(0, gic_cpu_base_addr);
#endif


#if defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
#elif defined(CONFIG_MP_PLATFORM_MSTAR_LEGANCY_INTR)
    arm_interrupt_chain_setup(INT_PPI_IRQ);
#else
    //GIC Interrupt Set Enable Register for MSTAR controller
    temp=PERI_R(GIC_DIST_SET_EANBLE);
    temp= temp | (0x01 << INT_PPI_IRQ );
    PERI_W(GIC_DIST_SET_EANBLE,temp);
#endif /* CONFIG_MP_PLATFORM_MSTAR_LEGANCY_INTR */

}

// switch FIQ/IRQ merge bit
int __init init_irq_fiq_merge(void)
{
    //unsigned short tmp;

    //tmp = reg_readw(0x1f000000 + (0x123964 << 1));
    //tmp &= 0xFFDF;
    //tmp |= 0x0050;
    //reg_writew(tmp, (0x1f000000 + (0x123964 << 1)));

    return 0;
}
