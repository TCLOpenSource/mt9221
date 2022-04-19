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
 * Copyright (C) 2000 MIPS Technologies, Inc.  All rights reserved.
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
 * This is the interface to the remote debugger stub.
 */
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>

#include <asm/serial.h>
#include <asm/io.h>

static struct serial_state rs_table[] = {
	SERIAL_PORT_DFNS	/* Defined in serial.h */
};

static struct async_struct kdb_port_info = {0};

int (*generic_putDebugChar)(char);
char (*generic_getDebugChar)(void);

static __inline__ unsigned int serial_in(struct async_struct *info, int offset)
{
	return inb(info->port + offset);
}

static __inline__ void serial_out(struct async_struct *info, int offset,
				int value)
{
	outb(value, info->port+offset);
}

int rs_kgdb_hook(int tty_no, int speed) {
	int t;
	struct serial_state *ser = &rs_table[tty_no];

	kdb_port_info.state = ser;
	kdb_port_info.magic = SERIAL_MAGIC;
	kdb_port_info.port = ser->port;
	kdb_port_info.flags = ser->flags;

	/*
	 * Clear all interrupts
	 */
	serial_in(&kdb_port_info, UART_LSR);
	serial_in(&kdb_port_info, UART_RX);
	serial_in(&kdb_port_info, UART_IIR);
	serial_in(&kdb_port_info, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	serial_out(&kdb_port_info, UART_LCR, UART_LCR_WLEN8);	/* reset DLAB */
	if (kdb_port_info.flags & ASYNC_FOURPORT) {
		kdb_port_info.MCR = UART_MCR_DTR | UART_MCR_RTS;
		t = UART_MCR_DTR | UART_MCR_OUT1;
	} else {
		kdb_port_info.MCR
			= UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2;
		t = UART_MCR_DTR | UART_MCR_RTS;
	}

	kdb_port_info.MCR = t;		/* no interrupts, please */
	serial_out(&kdb_port_info, UART_MCR, kdb_port_info.MCR);

	/*
	 * and set the speed of the serial port
	 */
	if (speed == 0)
		speed = 9600;

	t = kdb_port_info.state->baud_base / speed;
	/* set DLAB */
	serial_out(&kdb_port_info, UART_LCR, UART_LCR_WLEN8 | UART_LCR_DLAB);
	serial_out(&kdb_port_info, UART_DLL, t & 0xff);/* LS of divisor */
	serial_out(&kdb_port_info, UART_DLM, t >> 8);  /* MS of divisor */
	/* reset DLAB */
	serial_out(&kdb_port_info, UART_LCR, UART_LCR_WLEN8);

	return speed;
}

int putDebugChar(char c)
{
	return generic_putDebugChar(c);
}

char getDebugChar(void)
{
	return generic_getDebugChar();
}

int rs_putDebugChar(char c)
{

	if (!kdb_port_info.state) { 	/* need to init device first */
		return 0;
	}

	while ((serial_in(&kdb_port_info, UART_LSR) & UART_LSR_THRE) == 0)
		;

	serial_out(&kdb_port_info, UART_TX, c);

	return 1;
}

char rs_getDebugChar(void)
{
	if (!kdb_port_info.state) { 	/* need to init device first */
		return 0;
	}

	while (!(serial_in(&kdb_port_info, UART_LSR) & 1))
		;

	return serial_in(&kdb_port_info, UART_RX);
}
