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

#ifndef UFS_MSTAR_PLTFRM_H_
#define UFS_MSTAR_PLTFRM_H_

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <mach/io.h>
#include "chip_int.h"

#define CAP_MSTAR_MAX_RX_PWM_GEAR	3
#define CAP_MSTAR_MAX_TX_PWM_GEAR	3
#define CAP_MSTAR_MAX_RX_HS_GEAR	3
#define CAP_MSTAR_MAX_TX_HS_GEAR	3

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#ifndef BIT0
#define BIT0					(0x01 << 0)
#define BIT1					(0x01 << 1)
#define BIT2					(0x01 << 2)
#define BIT3					(0x01 << 3)
#define BIT4					(0x01 << 4)
#define BIT5					(0x01 << 5)
#define BIT6					(0x01 << 6)
#define BIT7					(0x01 << 7)
#define BIT8					(0x01 << 8)
#define BIT9					(0x01 << 9)
#define BIT10					(0x01 << 10)
#define BIT11					(0x01 << 11)
#define BIT12					(0x01 << 12)
#define BIT13					(0x01 << 13)
#define BIT14					(0x01 << 14)
#define BIT15					(0x01 << 15)
#endif


#define UFS_IRQ_NUM				E_IRQ_UFS

#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define RIU_PM_BASE				((uintptr_t)(mstar_pm_base))
#elif defined(CONFIG_ARM)
#define RIU_PM_BASE				(IO_ADDRESS(0x1F000000UL))
#elif defined(CONFIG_MIPS)
#define RIU_PM_BASE				(0xBF000000)
#endif

#define REG_UFSHCI_ADDR			(RIU_PM_BASE+(0x380200<<1))
#define REG_UFSHCI_MSTAR_ADDR	(RIU_PM_BASE+(0x301200<<1))
#define REG_UFSHCI_PAD_ADDR 	(RIU_PM_BASE+(0x302100<<1))
#define REG_CHIPTOP_ADDR		(RIU_PM_BASE+(0x101E00<<1))
#define REG_CLKGEN_ADDR			(RIU_PM_BASE+(0x100B00<<1))
#define REG_CLKGENB_ADDR		(RIU_PM_BASE+(0x103300<<1))
#define REG_EFUSE_ADDR			(RIU_PM_BASE+(0x002000<<1))

#define REG(Reg_Addr)			(*(volatile unsigned short*)(Reg_Addr))
#define REG_OFFSET_SHIFT_BITS	2
#define GET_REG_ADDR(x, y)		(x+((y)<<REG_OFFSET_SHIFT_BITS))
#define REG_WRITE_UINT16(reg_addr, val)		REG(reg_addr) = val
#define REG_READ_UINT16(reg_addr, val)		val = REG(reg_addr)
#define REG_SET_BITS_UINT16(reg_addr, val)	REG(reg_addr) |= (val)
#define REG_CLR_BITS_UINT16(reg_addr, val)	REG(reg_addr) &= ~(val)

#define REG_UFSHCI_MI0_START_L	GET_REG_ADDR(REG_UFSHCI_MSTAR_ADDR, 0x22)
#define REG_UFSHCI_MI0_START_H	GET_REG_ADDR(REG_UFSHCI_MSTAR_ADDR, 0x23)
#define REG_UFSHCI_MI0_END_L	GET_REG_ADDR(REG_UFSHCI_MSTAR_ADDR, 0x24)
#define REG_UFSHCI_MI0_END_H	GET_REG_ADDR(REG_UFSHCI_MSTAR_ADDR, 0x25)
#define REG_UFSHCI_RESET_N		GET_REG_ADDR(REG_UFSHCI_MSTAR_ADDR, 0x44)

#define REG_CLKGENB_0x3E		GET_REG_ADDR(REG_CLKGENB_ADDR, 0x3E)

#define REG_BANK3021_0x00		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x00)
#define REG_BANK3021_0x01		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x01)
#define REG_BANK3021_0x02		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x02)
#define REG_BANK3021_0x03		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x03)
#define REG_BANK3021_0x05		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x05)
#define REG_BANK3021_0x06		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x06)
#define REG_BANK3021_0x14		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x14)
#define REG_BANK3021_0x19		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x19)
#define REG_BANK3021_0x1A		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x1A)
#define REG_BANK3021_0x1C		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x1C)
#define REG_BANK3021_0x1F		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x1F)
#define REG_BANK3021_0x22		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x22)
#define REG_BANK3021_0x3D		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x3D)
#define REG_BANK3021_0x3E		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x3E)
#define REG_BANK3021_0x4A		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x4A)
#define REG_BANK3021_0x48		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x48)
#define REG_BANK3021_0x69		GET_REG_ADDR(REG_UFSHCI_PAD_ADDR, 0x69)


#define REG_BANKeFuse_0x28		GET_REG_ADDR(REG_EFUSE_ADDR, 0x28)
#define REG_BANKeFuse_0x2D		GET_REG_ADDR(REG_EFUSE_ADDR, 0x2D)

#define REG_CKG_UFSHCI			GET_REG_ADDR(REG_CLKGEN_ADDR, 0x6A)
#define BIT_UFSHCI_CLK_SELECT	BIT5
#define UFSHCI_CLK_MASK			(BIT4|BIT3|BIT2|BIT1|BIT0)
#define UFSHCI_CLK_150M			(0<<0)
#define UFSHCI_SW_DEFAULT_CLK	UFSHCI_CLK_150M

extern int ufs_mstar_pltfrm_init(void);
extern int ufs_mstar_pltfrm_clock(u16 u16ClkParam, u32 u32_gear, u32 u32_hs_rate);
#endif /* UFS_MSTAR_PLTFRM_H_ */
