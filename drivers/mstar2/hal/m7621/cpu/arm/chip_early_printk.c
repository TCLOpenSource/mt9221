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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/kernel.h>


#define UART_RX                     (0)  // In:  Receive buffer (DLAB=0)
#define UART_TX                     (0)  // Out: Transmit buffer (DLAB=0)
#define UART_DLL                    (0)  // Out: Divisor Latch Low (DLAB=1)
#define UART_DLM                    (1)  // Out: Divisor Latch High (DLAB=1)
#define UART_IER                    (1)  // Out: Interrupt Enable Register
#define UART_IIR                    (2)  // In:  Interrupt ID Register
#define UART_FCR                    (2)  // Out: FIFO Control Register
#define UART_LCR                    (3)  // Out: Line Control Register
#define UART_LSR                    (5)  // In:  Line Status Register
#define UART_MSR                    (6)  // In:  Modem Status Register
#define UART_USR                    (7)


// UART_LSR(5)
// Line Status Register
#define UART_LSR_DR                 0x01          // Receiver data ready
#define UART_LSR_OE                 0x02          // Overrun error indicator
#define UART_LSR_PE                 0x04          // Parity error indicator
#define UART_LSR_FE                 0x08          // Frame error indicator
#define UART_LSR_BI                 0x10          // Break interrupt indicator
#define UART_LSR_THRE               0x20          // Transmit-hold-register empty
#define UART_LSR_TEMT               0x40          // Transmitter empty

/* 1F000000 => FD000000 */
/* UART 0 : base + 0x1f201300 */
/* UART 1 : base + 0x1f220c00 */
/* UART 2 : base + 0x1f220d00 */
extern  volatile u64     is_paging_init;
#define UART_REG(addr) *((volatile unsigned short*)(0xfd201300UL + ((addr)<< 3)))
#define DIRECT_UART_READ_REG(reg, addr) do{  \
reg = *((volatile unsigned short*)(0xfd201300UL + ((addr)<< 3))); \
}while(0)

#define DIRECT_UART_WRITE_REG(reg, addr) do{  \
*((volatile unsigned short*)(0xfd201300UL + ((addr)<< 3))) = reg; \
}while(0)

/*------------------------------------------------------------------------------
Local Function Prototypes
-------------------------------------------------------------------------------*/
int prom_putchar(char c);

static u32 UART16550_READ(u8 addr)
{
u32 data;

if (addr>80) //ERROR: Not supported
{
return(0);
}

data = UART_REG((unsigned short)addr);
return(data);
}

static void UART16550_WRITE(u8 addr, u8 data)
{
if (addr>80) //ERROR: Not supported
{
//printk("W: %d\n",addr);
return;
}
UART_REG((unsigned short)addr) = data;
}

static inline unsigned int serial_in(int offset)
{
return UART16550_READ((u8)offset);
}

static inline void serial_out(int offset, int value)
{
UART16550_WRITE((u8)offset, (u8)value);
}

int prom_putchar(char c)
{

//volatile int i=0;

while ((serial_in(UART_LSR) & UART_LSR_THRE) == 0)
;
// for ( i=0;i<1000;i++)
// {
//     serial_in(UART_LSR);
// }
serial_out(UART_TX, c);

return 1;
}

static char early_printk_buf[1024];
// only use before prepare_page_table()
void early_putstr(const char *fmt, ...)
{
char c;
va_list args;
char* ptr;
u16 reg;

va_start(args, fmt);
vscnprintf(early_printk_buf, sizeof(early_printk_buf), fmt, args);
va_end(args);
ptr = early_printk_buf;


while ((c = *ptr++) != '\0') {
if (c == '\n')
{
DIRECT_UART_READ_REG(reg, UART_LSR);
while ((reg & UART_LSR_THRE) == 0)
{
DIRECT_UART_READ_REG(reg, UART_LSR);
}
//DIRECT_UART_REG(0) = 'e';
DIRECT_UART_WRITE_REG('\r', 0);
}
DIRECT_UART_READ_REG(reg, UART_LSR);
while ((reg & UART_LSR_THRE) == 0)
{
DIRECT_UART_READ_REG(reg, UART_LSR);
}
DIRECT_UART_WRITE_REG(c, 0);

//DIRECT_UART_REG(0) = 'e';
}
}

static void chip_printch(int n)
{
u16 reg;

do {
DIRECT_UART_READ_REG(reg, UART_LSR);
} while (!(reg & UART_LSR_THRE));
DIRECT_UART_WRITE_REG(n, 0);
}

static void chip_early_write(const char *s, unsigned n)
{
while (n-- > 0) {
if (*s == '\n')
chip_printch('\r');
chip_printch(*s);
s++;
}
}

void __init chip_early_map_io(void);
int chip_early_vprintk(const char *fmt, va_list ap)
{
char buf[512];
int n = vscnprintf(buf, sizeof(buf), fmt, ap);
static bool riu_mapped = false;

if (!riu_mapped) {
chip_early_map_io();
riu_mapped = true;
}

chip_early_write(buf, n);

return n;
}
