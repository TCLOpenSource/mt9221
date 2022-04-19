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
#include <linux/of.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/io.h>
#include <asm/setup.h>
#include <chip_int.h>
#include <irqchip.h>


#if defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
/*
 * 2 bits/per interrupt
 * b'00: level
 * b'10: edge
 */
/*irq 181 190 199 208 is active low, someone who want to use need to set reg to convert the signal*/
unsigned int interrupt_configs[MSTAR_CHIP_INT_END / 16+1] =
{
    0x00000000, /*   0~ 15 sgi, don't care */
    0x00000000, /*  16~ 31 ppi, don't care */
    0x00000000, /*  32~ 47 spi, set level for mstar irq */
    0x00000000, /*  64~ 79 spi, set level for mstar irq */
    0x00000000, /*  64~ 79 spi, set level for mstar irq */
    0x00000000, /*  80~ 95 spi, set level for mstar irq */
    0x00000000, /*  96~111 spi, set level for mstar fiq */
    0x00000000, /* 112~127 spi, set level for mstar fiq */
    0x00000000, /* 128~143 spi, set level for mstar fiq */
    0x00000000, /* 144~159 spi, set level for mstar fiq */
    0xA2A00000, /* 160~175 spi, set edge/level for mstar fiq */
    0x0AAA82AA, /* 176~191 spi, set edge/level for mstar fiq */
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

static inline void set_level(unsigned int irq)
{
    interrupt_configs[irq/IRQ_PER_UINT] |= (LEVEL << ((irq % IRQ_PER_UINT) * BIT_PER_IRQ));
}
void init_chip_spi_config(void)
{
	int i = 0;
#if 0
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
#endif
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

    /*
	208:CTI exception flag(active low),209: PMU exception event flag, 210 - 229:HYP IRQ
	230 - 236: HYP FIQ, 237: SUP IRQ
    */
    for(i = 208; i <= 237; i++ )
	set_level(i);
}
#endif/*MP_PLATFORM_INT_1_to_1_SPI*/

/* Clear FIQ (Clear is not supported for IRQ) */
void chip_irq_ack(unsigned int irq)
{
    u16 tmp = 0;

    if(irq < 16)
    {
        tmp = (0x01 << irq);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x2c << 2)) );
    }
    else if((irq >= 16) && (irq < 32))
    {
        tmp = (0x01 << (irq - 16));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x2d << 2)) );
    }
    else if( (irq >= 32) && (irq < 48))
    {
        tmp = (0x01) << (irq - 32);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x2e << 2)) );
    }
    else if( (irq >= 48) && (irq < 64))
    {
        tmp = (0x01) << (irq - 48);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x2f << 2)) );
    }
    else if( (irq >= 128) && (irq < 144))
    {
        tmp = (0x01) << (irq - 128);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x2c << 2)) );
    }
     else if((irq >= 144) && (irq < 160))
    {
        tmp = (0x01 << (irq - 144));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x2d << 2)) );
    }
    else if( (irq >= 160) && (irq < 176))
    {
        tmp = (0x01) << (irq - 160);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x2e << 2)) );
    }
    else if( (irq >= 176) && (irq < 192))
    {
        tmp = (0x01) << (irq - 176);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x2f << 2)) );
    }
}

/* Mask IRQ/FIQ */
void chip_irq_mask(unsigned int irq)
{
    u16 tmp = 0;

    if(irq <16)
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x24 << 2)));
        tmp |= (0x01) << irq;
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x24 << 2)) );
    }
    else if((irq >= 16) && (irq < 32))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x25 << 2)));
        tmp |= (0x01) << (irq - 16);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x25 << 2)) );
    }
    else if((irq >= 32) && (irq < 48))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x26 << 2)));
        tmp |= (0x01) << (irq - 32);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x26 << 2)) );
    }
    else if((irq >= 48) && (irq < 64))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x27 << 2)));
        tmp |= (0x01) << (irq - 48);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x27 << 2)) );
    }
    else if((irq >= 64) && (irq < 80))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x34 << 2)));
        tmp |= (0x01) << (irq - 64);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x34 << 2)) );
    }
    else if((irq >= 80) && (irq < 96))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x35 << 2)));
        tmp |= (0x01) << (irq - 80);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x35 << 2)) );
    }
    else if((irq >= 96) && (irq < 112))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x36 << 2)));
        tmp |= (0x01) << (irq - 96);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x36 << 2)) );
    }
    else if((irq >= 112) && (irq < 128))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x37 << 2)));
        tmp |= (0x01) << (irq - 112);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x37 << 2)) );
    }
    else if((irq >= 128) && (irq < 144))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x24 << 2)));
        tmp |= (0x01) << (irq - 128);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x24 << 2)) );
    }
    else if((irq >= 144) && (irq < 160))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x25 << 2)));
        tmp |= (0x01) << (irq - 144);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x25 << 2)) );
    }
    else if((irq >= 160) && (irq < 176))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x26 << 2)));
        tmp |= (0x01) << (irq - 160);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x26 << 2)) );
    }
    else if((irq >= 176) && (irq < 192))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x27 << 2)));
        tmp |= (0x01) << (irq - 176);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x27 << 2)) );
    }
    else if((irq >= 192) && (irq < 208))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x34 << 2)));
        tmp |= (0x01) << (irq - 192);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x34 << 2)) );
    }
    else if((irq >= 208) && (irq < 224))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x35 << 2)));
        tmp |= (0x01) << (irq - 208);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x35 << 2)) );
    }
    else if((irq >= 224) && (irq < 240))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x36 << 2)));
        tmp |= (0x01) << (irq - 224);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x36 << 2)) );
    }
    else if((irq >= 240) && (irq < 256))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x37 << 2)));
        tmp |= (0x01) << (irq - 240);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x37 << 2)) );
    }
}

/* Un-Mask IRQ/FIQ */
void chip_irq_unmask(unsigned int irq)
{
    u16 tmp = 0;

    //printk(KERN_WARNING "chip_irq_unmask(irq=%d)\n",irq);

    if(irq < 16)
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x24 << 2)));
        tmp &= ~((0x01) << irq);
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x24 << 2)) );
    }
    else if((irq >= 16) && (irq < 32))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x25 << 2)));
        tmp &= ~((0x01) << (irq - 16));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x25 << 2)) );
    }
    else if((irq >= 32) && (irq < 48))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x26 << 2)));
        tmp &= ~((0x01) << (irq - 32));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x26 << 2)) );
    }
    else if((irq >= 48) && (irq < 64))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x27 << 2)));
        tmp &= ~((0x01) << (irq - 48));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x27 << 2)) );
    }
    else if((irq >= 64) && (irq < 80))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x34 << 2)));
        tmp &= ~((0x01) << (irq - 64));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x34 << 2)) );
    }
    else if((irq >= 80) && (irq < 96))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x35 << 2)));
        tmp &= ~((0x01) << (irq - 80));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x35 << 2)) );
    }
    else if((irq >= 96) && (irq < 112))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x36 << 2)));
        tmp &= ~((0x01) << (irq - 96));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x36 << 2)) );
    }
    else if((irq >= 112) && (irq < 128))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_BASE + (0x37 << 2)));
        tmp &= ~((0x01) << (irq - 112));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_BASE + (0x37 << 2)) );
    }
    else if((irq >= 128) && (irq < 144))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x24 << 2)));
	tmp &= ~ ((0x01) << (irq - 128));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x24 << 2)) );
    }
    else if((irq >= 144) && (irq < 160))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x25 << 2)));
        tmp &= ~ ((0x01) << (irq - 144));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x25 << 2)) );
    }
    else if((irq >= 160) && (irq < 176))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x26 << 2)));
        tmp &= ~ ((0x01) << (irq - 160));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x26 << 2)) );
    }
    else if((irq >= 176) && (irq < 192))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x27 << 2)));
        tmp &= ~ ((0x01) << (irq - 176));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x27 << 2)) );
    }
    else if((irq >= 192) && (irq < 208))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x34 << 2)));
        tmp &= ~ ((0x01) << (irq - 192));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x34 << 2)) );
    }
    else if((irq >= 208) && (irq < 224))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x35 << 2)));
        tmp &= ~ ((0x01) << (irq - 208));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x35 << 2)) );
    }
    else if((irq >= 224) && (irq < 240))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x36 << 2)));
        tmp &= ~ ((0x01) << (irq - 224));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x36 << 2)) );
    }
    else if((irq >= 240) && (irq < 256))
    {
        tmp = readw_relaxed((void __iomem *)(REG_INT_HYP_BASE + (0x37 << 2)));
        tmp &= ~ ((0x01) << (irq - 240));
        writew_relaxed(tmp, (void __iomem *)(REG_INT_HYP_BASE + (0x37 << 2)) );
    }
}

/* Since kenrnel run on EL1, it forbidden to access secure bank  */
int __init init_irq_fiq_merge(void)
{
#if 0
    u16 tmp = 0;

    tmp = readw_relaxed((void __iomem *)(mstar_pm_base + (0x123964UL << 1)));
    tmp &= 0xFF8F;
    tmp |= 0x0050;
    writew_relaxed(tmp, (void __iomem *)(mstar_pm_base + (0x123964UL << 1)));
#endif

    return 0;
}

extern int __init gic_of_init(struct device_node *node, struct device_node *parent);
IRQCHIP_DECLARE(mstar_a53_gic, "arm,cortex-a15-gic", gic_of_init);


