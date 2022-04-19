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

#define MSTAR_MIU0_BUS_BASE                      0x20000000
extern phys_addr_t MIU1_BASE;
extern phys_addr_t MIU2_BASE;
#define MSTAR_MIU1_BUS_BASE                      MIU1_BASE
#define MSTAR_MIU1_BUS_BASE_L                    0xA0000000UL
#define MSTAR_MIU1_BUS_BASE_H                    0x300000000UL
#define MSTAR_MIU2_BUS_BASE                      MIU2_BASE
#define MSTAR_MIU2_BUS_BASE_L                    0xFFFFFFFFUL
#define MSTAR_MIU2_BUS_BASE_H                    0xFFFFFFFFFUL


#define ARM_MIU0_BUS_BASE                      MSTAR_MIU0_BUS_BASE
#define ARM_MIU1_BUS_BASE                      MSTAR_MIU1_BUS_BASE
#define ARM_MIU2_BUS_BASE                      MSTAR_MIU2_BUS_BASE

#define ARM_MIU3_BUS_BASE                      0xFFFFFFFFFFFFFFFFUL

#define ARM_MIU0_BASE_ADDR                     0x00000000UL
#define ARM_MIU1_BASE_ADDR                     0x80000000UL
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFFFFFFFFFFUL
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFFFFFFFFFFUL

#define MST_XTAL_CLOCK_HZ   (12000000UL)

#define REG_WDT_BASE         (mstar_pm_base + 0x006000)
#define REG_WDT_SET(x)      ( REG_WDT_BASE + 2*x )
#define REG_WDT_CLR         REG_WDT_SET(0x00UL)
#define REG_WDT_RST         REG_WDT_SET(0x02UL)
#define REG_WDT_INT         REG_WDT_SET(0x03UL)
#define REG_WDT_MAX         REG_WDT_SET(0x04UL)
#define REG_WDT_MAX2        REG_WDT_SET(0x05UL)

#define WDT_CLR_RESET_FLAG  BIT(0)
#define WDT_RST             BIT(0)
#define WDT_CLOSE_KEY       (0x0)

#define WDT_GET_TIMER() ((*((volatile unsigned short *)(REG_WDT_MAX))) | ((*((volatile unsigned short *)(REG_WDT_MAX2))) << 16))
#define WDT_CLEAR_RST_FLAG() ((*((volatile unsigned char *)(REG_WDT_RST))) = WDT_CLR_RESET_FLAG)
#define WDT_SET_TIMER(x) \
((*((volatile unsigned short *)(REG_WDT_MAX))) = (unsigned short)(((x) * (MST_XTAL_CLOCK_HZ)) & 0x0000FFFFUL), \
(*((volatile unsigned short *)(REG_WDT_MAX2))) = (unsigned short)(((x) * (MST_XTAL_CLOCK_HZ)) >> 16))
#define WDT_REFRESH() (*((volatile unsigned char *)(REG_WDT_CLR))) = WDT_RST;

extern unsigned int query_frequency(unsigned int cpu_id);
extern ptrdiff_t mstar_pm_base;
