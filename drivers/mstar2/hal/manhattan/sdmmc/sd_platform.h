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


// this file use for manhattan only

#if defined(CONFIG_ARM)
	#define A_RIU_PM_BASE                     (IO_ADDRESS(0x1F000000UL))
	#define A_RIU_BASE                        (IO_ADDRESS(0x1F200000UL))
#elif defined(CONFIG_ARM64)
	extern ptrdiff_t   mstar_pm_base;
	#define A_RIU_PM_BASE                     ((uintptr_t)(mstar_pm_base))
	#define A_RIU_BASE                        ((uintptr_t)(mstar_pm_base+0x200000))
#endif



#define BANK_PM_SLEEP      	GET_CARD_REG_ADDR(A_RIU_PM_BASE, 	0x0700) //			0x000E x 0x80 =
#define BANK_PM_GPIO       	GET_CARD_REG_ADDR(A_RIU_PM_BASE, 	0x0780) //			0x000F x 0x80 =

#define REG_BANK_MIU2		GET_CARD_REG_ADDR(A_RIU_BASE,		0x0300) //			0x0006 x 0x80 =
#define REG_BANK_MIU		GET_CARD_REG_ADDR(A_RIU_BASE,		0x0900) //			0x0012 x 0x80 =

#define REG_BANK_CHIPTOP	GET_CARD_REG_ADDR(A_RIU_BASE, 		0x0F00) //			0x001E x 0x80 = 0x0F00
#define CLKGEN_BANK			GET_CARD_REG_ADDR(A_RIU_BASE, 		0x0580) //			0x000B x 0x80 = 0x0580
//#define RIU_BASE_SDIO     GET_CARD_REG_ADDR(A_RIU_BASE, 	   0x10780) // SDIO		0x020F x 0x80 = 0x10780
#define REG_BANK_FCIE		GET_CARD_REG_ADDR(A_RIU_BASE,	   0x08980) // FCIE		0x0113 x 0x80
#define	REG_BANK_GPIO 		GET_CARD_REG_ADDR(A_RIU_BASE,		0x1580) // GPIO		0x002B x 0x80 =
//#define REG_BANK_SDIO_PLL	GET_CARD_REG_ADDR(A_RIU_BASE,	   0x11F00) // SDIO_PLL 0x023E x 0x80 = 0x11F00
#define REG_BANK_EMMC_PLL	GET_CARD_REG_ADDR(A_RIU_BASE,	   0x11F80) // EMMC_PLL 0x023F x 0x80 = 0x11F80


////////////////////////////////////////////////////////////////////////////////////////////////////
// BANK_PM_GPIO BANK 0x000F
////////////////////////////////////////////////////////////////////////////////////////////////////
#define PMGPIO_REG_07			GET_CARD_REG_ADDR(BANK_PM_GPIO, 0x07) // MST215C & 072B CDZ, Muji & Manhattan pin to pin

#define PMGPIO7_OEN				BIT00 // 0: output, 1: input
#define PMGPIO7_IN				BIT02


////////////////////////////////////////////////////////////////////////////////////////////////////
// CHIPTOP BANK 0x101E
////////////////////////////////////////////////////////////////////////////////////////////////////
#define CHIPTOP_0A			GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x0A)
#define CHIPTOP_50			GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x50)
#define CHIPTOP_5A			GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x5A)
#define CHIPTOP_64   		GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x64)
#define CHIPTOP_6E   		GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x6E)
#define CHIPTOP_6F			GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x6F)
#define CHIPTOP_7B   		GET_CARD_REG_ADDR(REG_BANK_CHIPTOP, 0x7B)


////////////////////////////////////////////////////////////////////////////////////////////////////
// FCIE BANK 0x1113
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
// MIU2 BANK 0x1006
////////////////////////////////////////////////////////////////////////////////////////////////////
#define MIU2_69					GET_CARD_REG_ADDR(REG_BANK_MIU2, 0x69)
#define MIU2_PROTECT			BIT00

#define MIU2_7A					GET_CARD_REG_ADDR(REG_BANK_MIU2, 0x7A)
#define MIU_SELECT_BY_SDIO		BIT10
#define MIU_SELECT_BY_FCIE		BIT09


////////////////////////////////////////////////////////////////////////////////////////////////////
// MIU BANK 0x1012
////////////////////////////////////////////////////////////////////////////////////////////////////
#define MIU_69					GET_CARD_REG_ADDR(REG_BANK_MIU, 0x69)
#define MIU_PROTECT				BIT00


////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO BANK 0x102B
////////////////////////////////////////////////////////////////////////////////////////////////////
#define CHIP_GPIO_47		GET_CARD_REG_ADDR(REG_BANK_GPIO, 0x47) // manhattan MST232A POWER_CTRL
#define TGPIO1_OEN			BIT1 // low --> output
#define TGPIO1_OUT			BIT0 // low power on, high power off

#define CHIP_GPIO_52		GET_CARD_REG_ADDR(REG_BANK_GPIO, 0x52) // manhattan MST232A CDZ
#define GPIO88_OEN			BIT1 // high --> input
#define GPIO88_IN			BIT2 // low --> insert, high --> remove ?


////////////////////////////////////////////////////////////////////////////////////////////////////
// CLKGEN BANK 0x100B
////////////////////////////////////////////////////////////////////////////////////////////////////
#define CLKGEN_FCIE			GET_CARD_REG_ADDR(CLKGEN_BANK, 0x64)
#define CLKGEN_SDIO			GET_CARD_REG_ADDR(CLKGEN_BANK, 0x69)

#define   _48M					(0xF<<2)
#define   _43M					(0x5<<2)
#define   _36M					(0x3<<2)
#define   _32M					(0x2<<2)
#define   _27M					(0x1<<2) // 20M
#define   _18M					(0x1<<2) // 20M
#define   _300K					(0xD<<2)

// clock gen for FCIE use only
#define FCIE_CLK_F 48000000
#define FCIE_CLK_5 43200000
#define FCIE_CLK_4 40000000
#define FCIE_CLK_3 36000000
#define FCIE_CLK_2 32000000
#define FCIE_CLK_1 20000000
#define FCIE_CLK_D   300000

#define FCIE_CLK_GATING		BIT0 // 0: gate, 1, open
#define FCIE_CLK_INVERSE	BIT1
#define FCIE_CLK_SOURCE_MSK (BIT2|BIT3|BIT4|BIT5)
#define FCIE_CLK_SOURCE_SEL BIT6 // 0: xtal, 1: clk_fcie_p1


////////////////////////////////////////////////////////////////////////////////////////////////////
// SDIO_PLL BANK 0x123E
////////////////////////////////////////////////////////////////////////////////////////////////////
//#define SDIO_PLL_REG_1A			GET_CARD_REG_ADDR(REG_BANK_SDIO_PLL, 0x1A)
//#define BIT_EMMC_TEST			BIT11

////////////////////////////////////////////////////////////////////////////////////////////////////
// EMMC_PLL BANK 0x123F
////////////////////////////////////////////////////////////////////////////////////////////////////
#define EMMC_PLL_REG_1A			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x1A)
#define BIT_EMMC_TEST			BIT11

#define EMMC_PLL_REG_68			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x68)
#define EMMC_PLL_REG_6D			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x6D)
#define EMMC_PLL_REG_70			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x70)
#define EMMC_PLL_REG_71			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x71)
#define EMMC_PLL_REG_73			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x73)
#define EMMC_PLL_REG_74			GET_CARD_REG_ADDR(REG_BANK_EMMC_PLL, 0x74)


////////////////////////////////////////////////////////////////////////////////////////////////////
// ?????
////////////////////////////////////////////////////////////////////////////////////////////////////


#define	SDR50			1
#define	SDR104			2
#define	DDR50			3
//#define 	SDBUS			SDR50


