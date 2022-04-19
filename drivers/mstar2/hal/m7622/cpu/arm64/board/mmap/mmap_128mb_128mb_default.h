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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    mmap_128mb_128mb_default.h
/// @brief  Memory mapping for 128MB+128MB RAM with project Obama
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////
//#include <linux/undefconf.h>
//#include "../../../../../include/linux/undefconf.h"
#ifndef _MS_MMAP_128MB_128MB_DEFAULT_H_
#define _MS_MMAP_128MB_128MB_DEFAULT_H_

// Memory alignment
#define MemAlignUnit                    64UL
#define MemAlign(n, unit)               ((((n)+(unit)-1)/(unit))*(unit))

#define MIU_DRAM_LEN				    (0x10000000)

#define MIU0_LEN					    (0x8000000)
#define MIU1_LEN					    (0x8000000)


#define COP_CODE_START				    (0x00000000)
#ifdef CONFIG_MSTAR_RESERVE_MEM_FOR_AEON
#define COP_CODE_LEN CONFIG_MSTAR_RESERVE_MEM_FOR_AEON_SIZE
#else
#define COP_CODE_LEN				    (0x0)
#endif

// Linux kernel space
#define LINUX_MEM_AVAILABLE		        (COP_CODE_START+COP_CODE_LEN)
#define LINUX_MEM_BASE_ADR 		        (LINUX_MEM_AVAILABLE)
#define LINUX_MEM_GAP_CHK  		        (LINUX_MEM_BASE_ADR-LINUX_MEM_AVAILABLE)
#define LINUX_MEM_LEN				    (0x27C0000UL) //8MB-256K


#define EMAC_MEM_AVAILABLE			    (LINUX_MEM_AVAILABLE + LINUX_MEM_LEN)
#define EMAC_MEM_ADR				    (EMAC_MEM_AVAILABLE)
#define EMAC_MEM_GAP_CHK				(EMAC_MEM_BASE_ADR-EMAC_MEM_AVAILABLE)
#define EMAC_MEM_LEN					(0x100000UL)


#define MPOOL_AVAILABLE				    (EMAC_MEM_AVAILABLE + EMAC_MEM_LEN)
#define MPOOL_ADR					    MemAlign(MPOOL_AVAILABLE, 4096)
#define MPOOL_GAP_CHK				    (MPOOL_ADR-MPOOL_AVAILABLE)
#define MPOOL_LEN					    (MIU_DRAM_LEN-MPOOL_ADR)

#endif


