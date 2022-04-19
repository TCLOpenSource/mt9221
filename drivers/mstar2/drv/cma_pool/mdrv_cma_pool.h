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
/// @file   cma_mpool_manager.h
/// @brief  CMA mpool Manager interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __DRV_MCMA_H__
#define __DRV_MCMA_H__
#include "mdrv_cma_pool_io.h"
#include "mdrv_cma_pool_st.h"
#include "mhal_miu.h"

#define size_1M (1<<20)

#ifdef CONFIG_MSTAR_MIUSLITS
#define SLIT_NUM   (256)
#define CMA_HEAP_LENGTH_ALIGNMENT ((pageblock_nr_pages)*(PAGE_SIZE))
#endif

#define MIU_BLOCK_NUM MIU_MAX_PROTECT_BLOCK

#ifdef CONFIG_MP_CMA_PATCH_MBOOT_STR_USE_CMA
void str_reserve_mboot_cma_buffer(void);
void str_release_mboot_cma_buffer(void);
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
#define global_page_state global_zone_page_state
#endif

typedef enum
{
    MIU_BLOCK_IDLE = 0,
    MIU_BLOCK_BUSY
}MIU_PROTECT_BLOCK_STATUS;

struct cma_buffer{
    bool freed;
    int mapped;
    void *cpu_addr;
    struct page * page;
    pid_t pid;
    //bus address
    unsigned long start_pa;
    unsigned long length;
    struct list_head list;
};

struct cma_allocation_list{
    unsigned long min_start;
    unsigned long max_end;
    unsigned long using_count;

    unsigned long freed_count;
    struct list_head list_head;
};

/* every MIU will have one MIU_ProtectRanges for it */
struct MIU_ProtectRanges
{
    unsigned char miu;											// this protect_info is for which MIU
    MIU_PROTECT_BLOCK_STATUS miuBlockStatus[MIU_BLOCK_NUM];		// BLOCK_STATUS: used or available

    unsigned int krange_num;									// count of used block
    struct list_head list_head;									// list for every protect_info (MIU_ProtectRange)
    struct mutex lock;
};

/* each protect_info(block) of a MIU */
typedef struct
{
	unsigned char miuBlockIndex;								// this protect_info is using which block
	unsigned long start_pa;
	unsigned long length;
	struct list_head list_node;
} MIU_ProtectRange;

typedef struct
{
    unsigned char miuBlockIndex;
    unsigned long start_pa;
    unsigned long length;
}MIU_ProtectRange_Record;

typedef struct
{
    MIU_ProtectRange_Record old;
    MIU_ProtectRange_Record new;
}MIU_ProtectRange_pair;

struct cma_memory_info{
    char miu;
    unsigned int heap_id;
    struct device *dev;
    struct cma_allocation_list alloc_list;
    struct MIU_ProtectRanges * pranges;

    struct list_head list_node;
    struct mutex lock;
};

#ifdef  CONFIG_MP_CMA_PATCH_POOL_UTOPIA_TO_KERNEL
enum type_states
{
    TYPE_STATES_USER = (1<<0),//for user
    TYPE_STATES_KERNEL= (1<<1),//for kernel
    TYPE_STATES_NUM,
};
enum type_kermode_flag
{
    KERNEL_MODE_CMA_POOL_USER_SPACE_MAP_CACHE = (1<<1),

};
struct kernel_cma_pool_vma_item
{
	struct list_head list_node;//node for vma_list
	pid_t pid;
	unsigned long flags;//
	struct vm_area_struct *vma;//for kernel mode user space va use.
};
#endif

struct heap_proc_info
{
    unsigned int heap_id;
    unsigned long base_phy;
    struct vm_area_struct *vma;//user mode cma heap user VA
    bool vma_cached;
    struct list_head list_node;
#ifdef CONFIG_MP_CMA_PATCH_POOL_UTOPIA_TO_KERNEL
    struct vm_struct *kernel_vm;//"vm_struct" for kernel,can only set value for once.
    enum type_states  type_states;//user utopia map or kernel utopia map
    struct list_head vma_list;//kernel mode cma heap user VA list for each process
	struct mutex vma_list_lock;
#endif
};

typedef struct
{
    pid_t pid;

    unsigned int temp_heap_id;

    struct list_head list_heap_proc_info;
    struct mutex lock;
}filp_private;

typedef struct
{
    unsigned char miu;
    int krange_change_num;
    MIU_ProtectRange_pair pair[KERN_CHUNK_NUM];
}MIU_KERN_PRange_Info_user;
#endif

