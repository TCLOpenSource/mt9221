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

#ifndef _CHIP_SETUP_H_
#define _CHIP_SETUP_H_

#ifdef CONFIG_ARM_LPAE
#define ARM_MIU0_BASE              0x20000000UL
#define ARM_MIU1_BASE               0x200000000ULL
#define MIU1_BUS_BASE               0x200000000ULL
#else
#define ARM_MIU0_BASE               0x20000000UL
#define ARM_MIU1_BASE               0xA0000000UL
#define MIU1_BUS_BASE               0xA0000000UL
#endif

#define HIGH_MEM_BUS_BASE           (0x50000000)
#define HIGH_MEM_PHY_BASE           (0x10000000)
#define HIGH_MEM_MAX_BUS_ADDR       (0x80000000)
#define RIU_VIRT_BASE               (0xFD000000)

#define REG(addr) (*(volatile unsigned int *)(addr))

#define SECOND_MAGIC_NUMBER_ADRESS	(PAGE_OFFSET + 0x8000)
#define SECOND_START_ADDR		(PAGE_OFFSET + 0x8004)

void* Chip_mphy_cachevirt( unsigned long mphyaddr );
void* Chip_mphy_noncachevirt( unsigned long mphyaddr );
unsigned long Chip_mphy_bus( unsigned long phyaddress );
unsigned long Chip_bus_mphy( unsigned long busaddress );

extern void _chip_flush_miu_pipe(void); //flush miu pipe
extern void Chip_Flush_Memory(void);  //flush miu pipe
extern void Chip_Read_Memory(void) ;  //flush miu pipe
extern void Chip_Flush_Memory_Range(unsigned long pAddress , unsigned long  size);  //flush miu pipe
extern void Chip_Read_Memory_Range(unsigned long pAddress , unsigned long  size ) ;  //flush miu pipe


extern void Chip_Flush_Cache_Range(unsigned long u32Addr, unsigned long u32Size); //Clean & Invalid L1/L2 cache
extern void Chip_Clean_Cache_Range(unsigned long u32Addr, unsigned long u32Size); //Clean L1/L2 cache, keep valid
extern void Chip_Inv_Cache_Range(unsigned long u32Addr, unsigned long u32Size);  //Invalid L1/L2 cache

extern void Chip_Flush_Cache_Range_VA_PA(unsigned long u32VAddr,unsigned long u32PAddr,unsigned long u32Size); //Clean & Invalid L1/L2 cache
extern void Chip_Clean_Cache_Range_VA_PA(unsigned long u32VAddr, unsigned long u32PAddr,unsigned long u32Size); //Clean L1/L2 cache, keep valid
extern void Chip_Inv_Cache_Range_VA_PA(unsigned long u32VAddr, unsigned long u32PAddr,unsigned long u32Size);  //Invalid L1/L2 cache
extern void Chip_Flush_Cache_All(void); //Clean & Invalid L1/L2 cache
extern void Chip_Flush_Cache_All_Single(void);

extern unsigned int Chip_Query_CLK(void);
extern unsigned int Chip_Query_Rev(void);
extern void Chip_Query_L2_Config(void);

#endif

