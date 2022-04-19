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
/// @file   Mhal_mtlb.h
/// @author MStar Semiconductor Inc.
/// @brief  MTLB Driver Interface
///////////////////////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
// Linux Mhal_mtlb.h define start
// -----------------------------------------------------------------------------
#ifndef __DRV_MTLB__
#define __DRV_MTLB__

#include "mdrv_mtlb.h"

//-------------------------------------------------------------------------------------------------
//  Define Enable or Compiler Switches
//-------------------------------------------------------------------------------------------------
#define MTLB_ERR(fmt, args...)   printk(KERN_ERR "mtlb hal error %s:%d" fmt,__FUNCTION__,__LINE__,## args)

//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------
#define TLB_Entry_BYTE_SIZE             4           /* tlb entry size = 4 bytes */
#define TLB_MAP_PAGE_SIZE               0x1000      /* tlb map per page size = 0x1000 bytes */

#define TLB_TAG_BIT_SHIFT               26
#define TLB_MIU_BIT_SHIFT               19

#define MTLB_RIU_REG_BASE                   0xFD000000
#define MTLB_RIU_BANK                       0x1628

/*8 bits address*/
#define REG_TLB_MODE                        0x00
    #define GE_TLB_FLUSH                        0x0008
#define REG_TLB_ENABLE                      0x02
    #define GE_TLB_ENABLE                        0x0001
#define REG_TLB_TAG                         0x20

#define TLB_GE_TAG_VALUE                          0x28         // (MTLB) GE Tag Value


void MHal_MTLB_Get_HWInfo(mtlb_hardware_info * hwinfo);
int MHal_MTLB_Get_HWStatus(mtlb_hardware_status * hwstatus);
int MHal_MTLB_TableSize(void);
int MHal_MTLB_Init(void);
int MHal_MTLB_Reserve_Memory(void);
int MHal_MTLB_Get_ZeroPage(unsigned long * u32miu0pa, unsigned long * u32miu1pa, unsigned long * u32miu2pa);
int MHal_MTLB_Enable(mtlb_tlbclient_enable * tlb_client_enable);
void MHal_MTLB_Mapping_Start(unsigned long *flag);
void MHal_MTLB_Mapping_End(unsigned long * flag, u8 u8miu, unsigned long start, unsigned long size);
int MHal_MTLB_Mapping(u8 u8miu, u32 u32va, u32 u32pa, bool paInc, u32 u32size);
int MHal_MTLB_Mapping_Address_Support(unsigned long * u32pa, unsigned long nents, unsigned char * miu);
int MHal_MTLB_Dump(u32 *u32va, u32 *u32pa);
#endif
// -----------------------------------------------------------------------------
// Linux Mhal_mtlb.h End
// -----------------------------------------------------------------------------


