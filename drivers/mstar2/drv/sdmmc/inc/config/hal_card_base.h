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

/***************************************************************************************************************
 *
 * FileName hal_card_base.h (for G2p)
 *     @author jeremy.wang (2010/10/14)
 * Desc:
 *	   This header file is independent.
 * 	   We could put different hal_card_base.h in different build code folder but the same name.
 *	   We want to distinguish between this and others settings but the same project.
 * 	   Specific Define is freedom for each project, but we need to put it at inner code of project define.
 *
 *	   For Base Setting:
 * 	   (1) BASE Specific Define for Each Project
 * 	   (2) BASE Project/OS/CPU Define
 * 	   (3) BASE Project/OS/CPU and Specific Option Define
 * 	   (4) BASE TYPE Define
 * 	   (5) BASE Debug System
 *
 * 	   P.S.	D_XX for define and D_XX__ two under line("__") to distinguish define and its define option.
 *
 ***************************************************************************************************************/

#ifndef __HAL_CARD_BASE_H
#define __HAL_CARD_BASE_H


//***********************************************************************************************************
// (1) BASE Specific Define for Each Project
//***********************************************************************************************************
//#define D_ICVER             D_ICVER_00
//#define D_BDVER             0

//***********************************************************************************************************
// (2) BASE Project/OS/CPU Define
//***********************************************************************************************************
#ifdef  CONFIG_MSTAR_EAGLE
#define D_PROJECT	        D_PROJECT__EAGLE
#endif
#ifdef CONFIG_MSTAR_EDISON
#define D_PROJECT           D_PROJECT__EDISON
#endif
#ifdef CONFIG_MSTAR_EIFFEL
#define D_PROJECT           D_PROJECT__EIFFEL
#endif
#ifdef CONFIG_MSTAR_NIKE
#define D_PROJECT           D_PROJECT__NIKE
#endif
#ifdef CONFIG_MSTAR_KAISER
#define D_PROJECT           D_PROJECT__KAISER
#endif
#ifdef CONFIG_MSTAR_KAISERS
#define D_PROJECT           D_PROJECT__KAISERS
#endif

#ifdef CONFIG_MSTAR_EINSTEIN
#define D_PROJECT           D_PROJECT__EINSTEIN
#endif
#ifdef CONFIG_MSTAR_MADISON
#define D_PROJECT           D_PROJECT__MADISON
#endif
#ifdef CONFIG_MSTAR_MONACO
#define D_PROJECT           D_PROJECT__MONACO
#endif
#ifdef CONFIG_MSTAR_CLIPPERS
#define D_PROJECT           D_PROJECT__CLIPPERS
#endif
#ifdef CONFIG_MSTAR_MUJI
#define D_PROJECT           D_PROJECT__MUJI
#endif
#ifdef CONFIG_MSTAR_MONET
#define D_PROJECT           D_PROJECT__MONET
#endif
#ifdef CONFIG_MSTAR_MANHATTAN
#define D_PROJECT           D_PROJECT__MANHATTAN
#endif
#ifdef CONFIG_MSTAR_KANO
#define D_PROJECT           D_PROJECT__KANO
#endif
#ifdef CONFIG_MSTAR_MASERATI
#define D_PROJECT           D_PROJECT__MASERATI
#endif
#define D_OS                D_OS__LINUX
#define D_CPU               D_CPU__ARM
#define D_PRODUCT           D_TV

//***********************************************************************************************************
// (3) BASE Project/OS/CPU and Specific Option Define
//***********************************************************************************************************
// Project Option Define
//-----------------------------------------------------------------------------------------------------------
#define D_PROJECT__CB2			 1
#define D_PROJECT__G2			 2
#define D_PROJECT__EAGLE		 3
#define D_PROJECT__EIFFEL		 4
#define D_PROJECT__EDISON		 5
#define D_PROJECT__NIKE			 6
#define D_PROJECT__KAISER		 7
#define D_PROJECT__EINSTEIN		 8
#define D_PROJECT__KAISERS		 9
#define D_PROJECT__MADISON		10
#define D_PROJECT__MONACO		11
#define D_PROJECT__CLIPPERS		12
#define D_PROJECT__MUJI			13
#define D_PROJECT__MONET		14
#define D_PROJECT__MANHATTAN	15
#define D_PROJECT__KANO			16
#define D_PROJECT__MASERATI		17

// OS Option
//-----------------------------------------------------------------------------------------------------------
#define D_OS__LINUX       1
#define D_OS__UBOOT       2
#define D_OS__WINCE       3
#define D_OS__EBOOT       4

// CPU Option
//-----------------------------------------------------------------------------------------------------------
#define D_CPU__ARM        1
#define D_CPU__MIPS       2

// Product Option Define
//-----------------------------------------------------------------------------------------------------------
#define D_PHONE           1
#define D_TV              2

// IC Version Option
//-----------------------------------------------------------------------------------------------------------
#define D_ICVER_00      0
#define D_ICVER_01      1
#define D_ICVER_02      2
#define D_ICVER_03      3
#define D_ICVER_04      4
#define D_ICVER_05      5
#define D_ICVER_06      6
#define D_ICVER_07      7
#define D_ICVER_08      8
#define D_ICVER_09      9

//***********************************************************************************************************
// (4) BASE Type Define
//***********************************************************************************************************

typedef unsigned char		BOOL_T;								  // 1 bytes
typedef unsigned char		U8_T;								  // 1 bytes
typedef unsigned short		U16_T;                                // 2 bytes
typedef unsigned int		U32_T;                                // 4 bytes
typedef unsigned long		ULONG_T;                              // 4 byte in 32 bit, 8 bytes in 64 bits
typedef unsigned long long	U64_T;
typedef signed char			S8_T;                                 // 1 byte
typedef signed short		S16_T;                                // 2 bytes
typedef signed int			S32_T;                                // 4 bytes
typedef signed long			SLONG_T;                              // 4 or 8 bytes
typedef unsigned int		U32;

#define FALSE	0
#define TRUE	1

#define BIT0 0x0001
#define BIT1 0x0002
#define BIT2 0x0004
#define BIT3 0x0008
#define BIT4 0x0010
#define BIT5 0x0020
#define BIT6 0x0040
#define BIT7 0x0080
#define BIT8 0x0100
#define BIT9 0x0200

#define BIT00 0x0001
#define BIT01 0x0002
#define BIT02 0x0004
#define BIT03 0x0008
#define BIT04 0x0010
#define BIT05 0x0020
#define BIT06 0x0040
#define BIT07 0x0080
#define BIT08 0x0100
#define BIT09 0x0200
#define BIT10 0x0400
#define BIT11 0x0800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000
#define BIT16 0x00010000
#define BIT17 0x00020000
#define BIT18 0x00040000
#define BIT19 0x00080000
#define BIT20 0x00100000
#define BIT21 0x00200000
#define BIT22 0x00400000
#define BIT23 0x00800000
#define BIT24 0x01000000
#define BIT25 0x02000000
#define BIT26 0x04000000
#define BIT27 0x08000000
#define BIT28 0x10000000
#define BIT29 0x20000000
#define BIT30 0x40000000
#define BIT31 0x80000000

typedef enum
{
	EV_IP_FCIE1     = 0,
	EV_IP_FCIE2     = 1,

} IPEmType;

typedef enum
{
	EV_REG_BANK    = 0,
	EV_CIFC_BANK   = 1,
	EV_CIFD_BANK   = 2,

} IPBankEmType;

typedef enum
{
	EV_PORT_SD     = 0,
	EV_PORT_SDIO1  = 1,
	EV_PORT_SDIO2  = 2,
	EV_PORT_MS     = 3,
	EV_PORT_SMXD   = 4,
	EV_PORT_CF     = 5,
	EV_PORT_NANDT  = 6,

} PortEmType;

typedef enum
{
	EV_OK	= 0,
	EV_FAIL	= 1,

} RetEmType;



//***********************************************************************************************************
// (5) BASE Debug System
//***********************************************************************************************************

//###########################################################################################################
#if (D_OS == D_OS__LINUX)
//###########################################################################################################

#include <linux/kernel.h>
#define prtstring(s)	printk(s)
#define prtUInt(v) 		printk("%u", v)
#define prtU8(v)		printk("0x%02X", v)
#define prtU8Hex(v)		printk("0x%02X", v)
#define prtU16Hex(v)	printk("0x%04X", v)
#define prtU32Hex(v)	printk("0x%08X", v)
//###########################################################################################################
#else
//###########################################################################################################
/*
extern void uart_write_string(U8 *ptr);
extern void uart_write_U8_T_hex(U8 c);
extern void uart_write_U32_T_hex(U32 val););

#define prtstring(s)	uart_write_string(s)
#define prtUInt(v) 		uart_write_U32_hex(v)
#define prtU8(v)		uart_write_U8_hex(v)
#define prtU8Hex(v)		uart_write_U8_hex(v)
#define prtU16Hex(v)	uart_write_U32_hex(v)
#define prtU32Hex(v)	uart_write_U32_hex(v)
*/

//###########################################################################################################
#endif



#endif //End of __HAL_CARD_BASE_H


