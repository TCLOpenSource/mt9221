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
 * $Id: //DAILEO/Columbus/IPCamera/source/iNfinity/iNfinity_ROM/source/include/platform.h#4 $
 * $Header: //DAILEO/Columbus/IPCamera/source/iNfinity/iNfinity_ROM/source/include/platform.h#4 $
 * $Date: 2015/06/10 $
 * $DateTime: 2015/06/10 16:00:37 $
 * $Change: 1251790 $
 * $File: //DAILEO/Columbus/IPCamera/source/iNfinity/iNfinity_ROM/source/include/platform.h $
 * $Revision: #4 $
 */

#ifndef PLATEFORM_H
#define PLATEFORM_H

#define SetDebugFlag(x) (REG(REG_ADDR_DEBUG)=x)

#define BOOTROM_VERSION_ASIC

/*==========================================================================
    //CA4 Physical Address Mapping
===========================================================================*/

#define IMI_BASE_ADDR           0x00000000
#define RIU_BASE_ADDR           0x1f000000
#ifdef BOOTROM_VERSION_ZEBU
  #define REG_ADDR_DEBUG            MIU0_START_ADDR
#else
  #define REG_ADDR_DEBUG            REG_ADDR_BASE_MAILBOX
#endif
#define FLAG_INIT_UART_BUSY         0x0BF1

/*==========================================================================
    Put the configuration base address of each IP here.
===========================================================================*/
#define GET_REG8_ADDR(x, y)  	      (x+(y)*2-((y)&1))
#define GET_REG16_ADDR(x, y)  	      (x+(y)*4)
#define GET_REG_OFFSET(x, y)  	      ((x)*0x200+(y)*0x4)

#define REG_ADDR_BASE_CHIPTOP         GET_REG8_ADDR( RIU_BASE_ADDR, 0x1E00 )
#define REG_ADDR_BASE_CLKGEN          GET_REG8_ADDR( RIU_BASE_ADDR, 0x40200 )

#define REG_ADDR_BASE_PM_UART0        GET_REG8_ADDR( RIU_BASE_ADDR, 0xC00 )
#define REG_ADDR_BASE_PM_SLEEP        GET_REG8_ADDR( RIU_BASE_ADDR, 0xE00 )
#define REG_ADDR_BASE_WDT             GET_REG8_ADDR( RIU_BASE_ADDR, 0x3000 )
#define REG_ADDR_BASE_TIMER0          GET_REG8_ADDR( RIU_BASE_ADDR, 0x3020 )
#define REG_ADDR_BASE_TIMER1          GET_REG8_ADDR( RIU_BASE_ADDR, 0x3040 )
#define REG_ADDR_BASE_TIMER2          GET_REG8_ADDR( RIU_BASE_ADDR, 0x3060 )

//#define REG_ADDR_BASE_MIUPLL          GET_REG8_ADDR( RIU_BASE_ADDR, 0x103100 )
#define REG_ADDR_BASE_MAILBOX         GET_REG8_ADDR( RIU_BASE_ADDR, 0x103380 )
#define REG_ADDR_BASE_INTC            GET_REG8_ADDR( RIU_BASE_ADDR, 0x1019C0 )
#define REG_ADDR_BASE_CPUINT          GET_REG8_ADDR( RIU_BASE_ADDR, 0x100540 )
#define REG_ADDR_BASE_VBDMA           GET_REG8_ADDR( RIU_BASE_ADDR, 0x151A00 )

#if 0
#if defined(BOOTROM_VERSION_FPGA)
    #define UART_BAUDRATE       115200
    #define UART_CLK            27000000
    #define TIMER_OSC           27000000
#elif defined(BOOTROM_VERSION_ASIC)
    #define UART_BAUDRATE       115200
    #define UART_CLK            172800000
    #define TIMER_OSC           12000000
#elif defined(BOOTROM_VERSION_ZEBU)
    #define UART_BAUDRATE       38400
    #define UART_CLK            12000000
    #define TIMER_OSC           12000000
#else
    #error "incorrect version!!"
#endif
#endif

#if defined(BOOTROM_VERSION_FPGA)
    #define UART_BAUDRATE       38400
    #define UART_CLK            12000000
    #define TIMER_OSC           12000000
#elif defined(BOOTROM_VERSION_ASIC)
    #define UART_BAUDRATE       115200
    #define UART_CLK            24000000
    #define TIMER_OSC           12000000
#elif defined(BOOTROM_VERSION_ZEBU)
    #define UART_BAUDRATE       38400
    #define UART_CLK            12000000
    #define TIMER_OSC           12000000
#else
    #error "incorrect version!!"
#endif


#endif //PLATEFORM_H
