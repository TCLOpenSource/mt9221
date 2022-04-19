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


#ifndef _MDRV_MPOOL_ST_H_
#define _MDRV_MPOOL_ST_H_


//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
// MPOOL_IOC_INFO

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#ifdef CONFIG_MP_NEW_UTOPIA_32BIT
typedef struct
{
    u64                u32Addr;
    u64                u32Size;
    u64                u32Interval;
    unsigned char      u8MiuSel;
} DrvMPool_Info_t;


typedef struct
{
    size_t             u32AddrVirt;
    u64                u32AddrPhys;
    u64                u32Size;
} DrvMPool_Flush_Info_t;

typedef struct
{
    u64                LX_MEM_ADDR;
    u64                LX_MEM_LENGTH;
    u64                LX_MEM2_ADDR;
    u64                LX_MEM2_LENGTH;
    u64                EMAC_ADDR;
    u64                EMAC_LENGTH;
    u64                DRAM_ADDR;
    u64                DRAM_LENGTH;
} De_Sys_Info_t;

typedef struct
{
    u64                u32lxAddr;
    u64                u32lxSize;
    u64                u32lx2Addr;
    u64                u32lx2Size;
} DrvMPool_Kernel_Info_t;

typedef struct{
    unsigned int       ASID;
    unsigned int       global;
    u64                u32AddrVirt;
    unsigned int       rwx;
    unsigned int       mask;
} DrvMPool_Watchpt_Info_t;

typedef struct{
    unsigned int       wvr0;
    unsigned int       wvr1;
    unsigned int       wcr0;
    unsigned int       wcr1;
} DrvMPool_Wcvr_Info_t;
#else
typedef struct
{
    unsigned int                u32Addr;
    unsigned int                u32Size;
    unsigned int                u32Interval;
    unsigned char               u8MiuSel;
} DrvMPool_Info_t;

typedef struct
{
    unsigned int                u32AddrVirt;
    unsigned int                u32AddrPhys;
    unsigned int                u32Size;
} DrvMPool_Flush_Info_t;

typedef struct
{
    unsigned int                LX_MEM_ADDR;
    unsigned int                LX_MEM_LENGTH;
    unsigned int                LX_MEM2_ADDR;
    unsigned int                LX_MEM2_LENGTH;
    unsigned int                EMAC_ADDR;
    unsigned int                EMAC_LENGTH;
    unsigned int                DRAM_ADDR;
    unsigned int                DRAM_LENGTH;
} De_Sys_Info_t;

typedef struct
{
    unsigned int                u32lxAddr;
    unsigned int                u32lxSize;
    unsigned int                u32lx2Addr;
    unsigned int                u32lx2Size;
} DrvMPool_Kernel_Info_t;

typedef struct{
    unsigned int                ASID;
    unsigned int                global;
    unsigned int                u32AddrVirt;
    unsigned int                rwx;
    unsigned int                mask;
} DrvMPool_Watchpt_Info_t;

typedef struct{
    unsigned int                wvr0;
    unsigned int                wvr1;
    unsigned int                wcr0;
    unsigned int                wcr1;
} DrvMPool_Wcvr_Info_t;
#endif //CONFIG_MP_NEW_UTOPIA_32BIT
#elif defined(CONFIG_ARM64) //64bit kernel and 64bit physical
#if defined(CONFIG_COMPAT)
typedef struct
{
    compat_size_t                u32AddrVirt;
    compat_u64                   u32AddrPhys;
    compat_u64                   u32Size;
} DrvMPool_Flush_Info_t32;
#endif

typedef struct
{
    u64                u32Addr;
    u64                u32Size;
    u64                u32Interval;
    unsigned char      u8MiuSel;
} DrvMPool_Info_t;


typedef struct
{
    size_t             u32AddrVirt;
    u64                u32AddrPhys;
    u64                u32Size;
} DrvMPool_Flush_Info_t;

typedef struct
{
    u64                LX_MEM_ADDR;
    u64                LX_MEM_LENGTH;
    u64                LX_MEM2_ADDR;
    u64                LX_MEM2_LENGTH;
    u64                EMAC_ADDR;
    u64                EMAC_LENGTH;
    u64                DRAM_ADDR;
    u64                DRAM_LENGTH;
} De_Sys_Info_t;

typedef struct
{
    u64                u32lxAddr;
    u64                u32lxSize;
    u64                u32lx2Addr;
    u64                u32lx2Size;
} DrvMPool_Kernel_Info_t;

typedef struct{
    unsigned int       ASID;
    unsigned int       global;
    u64                u32AddrVirt;
    unsigned int       rwx;
    unsigned int       mask;
} DrvMPool_Watchpt_Info_t;

typedef struct{
    unsigned int       wvr0;
    unsigned int       wvr1;
    unsigned int       wcr0;
    unsigned int       wcr1;
} DrvMPool_Wcvr_Info_t;
#endif //CONFIG_ARM

#endif // _MDRV_MPOOL_ST_H_

