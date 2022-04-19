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

#ifndef __MDRV_IPA_POOL_H_
#define __MDRV_IPA_POOL_H_
#include <linux/idr.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#ifdef CONFIG_ARM64
#define U64X_FMT "%lx"
#else
#define U64X_FMT "%llx"
#endif

enum miu_type
{
    MIU0=0,
    MIU1,
    MIU2,
    MIU_MAX
};

struct miu_addr
{
    int miu;
    unsigned long miu_offset;
};

struct IPA_pool{
    struct list_head node;
    struct list_head refs;
    struct IPAPool_device *device;
    struct mutex lock;
    int hid;
    int id;//notice that this value not means pool handle id !!!!
    unsigned int flags;
    unsigned long heap_off;
    unsigned long len;
    phys_addr_t phyaddr;
    int count;
    char *name;
    struct idr idr_poolref;
    struct dentry * dbg_dir;
    struct dentry * dbg_entry;
};

#define MAX_CONFLICT_WAKE_UP_COUNTS (10)
struct IPA_pool_conflict_detail
{
    struct file *conflict_client_filp;
    unsigned long offset_in_heap;   //offset in heap space, unit:BYTE
    unsigned long length;   
};
struct IPA_pool_conflict 
{
    struct IPA_pool_conflict_detail  conflict_detail[MAX_CONFLICT_WAKE_UP_COUNTS];
    unsigned long conflict_counts;
};

#define IPAPOOLFLG_CPU_ADDR_TYPE_MASK 0x0F
#define IPAPOOLFLG_KERNEL_ADDR         0x10   //map to kernel address
#define IPAPOOLFLG_INVALID_ONLY        0x20   //invalid cache only

extern int IPA_pool_init(
                           int heap_id, unsigned int flags,
                           unsigned long heap_off,
                           unsigned long pool_len,char *name,
                           int *pool_id);
extern int IPA_pool_deinit(int pool_id);
extern int IPA_pool_get_mem(int pool_id,
                           unsigned long pool_offset,  unsigned long len,  unsigned long timeout);
extern int IPA_pool_put_mem(
                           int pool_id,
                           unsigned long pool_offset,  unsigned long len);
extern int IPA_pool_map(
                           int pool_id,
                           unsigned long pool_offset, unsigned long len,
                           unsigned long flags,unsigned long *paddr);
extern int IPA_pool_unmap(
                           unsigned long va, unsigned long len);
extern int IPA_pool_dcache_flush(
    unsigned long va, unsigned long len,
    unsigned long flags);

void str_reserve_mboot_ipa_str_pool_buffer();
void str_release_mboot_ipa_str_pool_buffer();

#endif
