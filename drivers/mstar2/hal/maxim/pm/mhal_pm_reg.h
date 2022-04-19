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

#ifndef _REG_PM_H_
#define _REG_PM_H_

#include <mstar/mstar_chip.h>
#include "mdrv_types.h"


#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define PM_RIU_REG_BASE                 0xFD000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define PM_RIU_REG_BASE                 mstar_pm_base
#endif

#define REG_PM_SLEEP_BASE               (0x0e00UL)
#define REG_PM_MISC_BASE                (0x2e00UL)
#define REG_MCU_BASE                    (0x1000UL)
#define REG_MCU_CACHE_BASE              (0x2b00UL)
#define REG_MBX_BASE                    (0x103300UL)

//------------------------------------------------------------------------------
// pm sleep Reg
//------------------------------------------------------------------------------
#define REG_PM_MCU_CLK                  (REG_PM_SLEEP_BASE + (0x20UL << 1))
#define REG_PM_LOCK                     (REG_PM_SLEEP_BASE + (0x12UL << 1))
#define REG_PM_DUMMY_WAKEUP_SOURCE      (REG_PM_SLEEP_BASE + (0x39UL << 1))
#define REG_PM_WK_MASK                  (REG_PM_SLEEP_BASE + (0x08UL << 1))

//------------------------------------------------------------------------------
// pm misc Reg
//------------------------------------------------------------------------------
#define REG_PM_CPU_SW_RST               (REG_PM_MISC_BASE + (0x29UL << 1))
#define REG_PM_CPU_SW_RST_P             (REG_PM_MISC_BASE + (0x29UL << 1) + 1)
#define REG_PM_RST_CPU0_PASSWORD        (REG_PM_MISC_BASE + (0x2aUL << 1))


//------------------------------------------------------------------------------
// mcu Reg
//------------------------------------------------------------------------------
#define REG_MCU_SRAM_START_ADDR_H       (REG_MCU_BASE + (0x00UL << 1))
#define REG_MCU_SRAM_END_ADDR_H         (REG_MCU_BASE + (0x01UL << 1))
#define REG_MCU_SRAM_START_ADDR_L       (REG_MCU_BASE + (0x02UL << 1))
#define REG_MCU_SRAM_END_ADDR_L         (REG_MCU_BASE + (0x03UL << 1))
#define REG_MCU_DRAM_START_ADDR_1       (REG_MCU_BASE + (0x04UL << 1))
#define REG_MCU_DRAM_END_ADDR_1         (REG_MCU_BASE + (0x05UL << 1))
#define REG_MCU_DRAM_START_ADDR_0       (REG_MCU_BASE + (0x06UL << 1))
#define REG_MCU_DRAM_END_ADDR_0         (REG_MCU_BASE + (0x07UL << 1))
#define REG_MCU_CONFIG                  (REG_MCU_BASE + (0x0cUL << 1))
#define REG_MCU_IR_POWERON_KEY		(REG_MCU_BASE + (0x53UL << 1)) // High byte

//------------------------------------------------------------------------------
// 8051 cache Reg
//------------------------------------------------------------------------------
#define REG_ICACHE_SDRAM_CODE_MAP       (REG_MCU_CACHE_BASE + (0x40UL << 1))
#define REG_MCU_CACHE_CONFIG            (REG_MCU_CACHE_BASE + (0x50UL << 1))

//------------------------------------------------------------------------------
// Mailbox bank
//------------------------------------------------------------------------------
#define REG_MBX_0                       (REG_MBX_BASE + (0x40UL << 1))
#define REG_MBX_1                       (REG_MBX_BASE + (0x41UL << 1))
#define REG_MBX_2                       (REG_MBX_BASE + (0x42UL << 1))
#define REG_MBX_3                       (REG_MBX_BASE + (0x43UL << 1))

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#endif // _REG_PM_H_
