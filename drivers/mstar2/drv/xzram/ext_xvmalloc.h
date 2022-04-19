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

/*
 * xvmalloc memory allocator
 *
 * Copyright (C) 2008, 2009, 2010  Nitin Gupta
 *
 * This code is released using a dual license strategy: BSD/GPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of 3-clause BSD License
 * Released under the terms of GNU General Public License Version 2.0
 */

#ifndef _EXT_XV_MALLOC_H_
#define _EXT_XV_MALLOC_H_

#include <linux/types.h>

#define EXT_XVPOOL_INTERNAL_NO_LIMIT 0
#define EXT_XVPOOL_EXTERNAL          1
#define EXT_XVPOOL_INTERNAL          2

#define XZRAM_PAGE_ATTR (GFP_NOIO | __GFP_HIGHMEM)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#ifndef __GFP_REPEAT
#define __GFP_REPEAT ___GFP_RETRY_MAYFAIL
#endif
#endif

struct ext_xv_pool_attr{
     u8 pool_id;
     u8 type;
     unsigned int ext_pool_size;
     unsigned int ext_pool_start;
};

struct ext_xv_backup_page{
     void __iomem *page;
     struct ext_xv_backup_page *next_backup_page;
};

struct ext_xv_backuplist{
     struct ext_xv_backup_page *backup_page;
     unsigned int cnt;
};

struct migration{
    unsigned int on_migration;
    unsigned int read_cnt;
    unsigned int write_cnt;
};

struct ext_xv_pool;

struct ext_xv_pool *ext_xv_create_pool(struct ext_xv_pool_attr *attr, u8 pool_id);
void ext_xv_destroy_pool(struct ext_xv_pool *pool);
void __iomem *ext_xv_get_ptr_atomic(void __iomem *page, u16 offset, u32 type, struct ext_xv_pool *pool);
void ext_xv_put_ptr_atomic(void *ptr, u32 type, struct ext_xv_pool *pool);
int ext_xv_malloc(struct ext_xv_pool *pool, u32 size, struct page **page,
			u32 *offset, gfp_t flags);
void ext_xv_free(struct ext_xv_pool *pool, struct page *page, u32 offset);

u32 ext_xv_get_object_size(void *obj);
u64 ext_xv_get_total_size_bytes(struct ext_xv_pool *pool);
bool ext_xv_is_ext_pool(struct ext_xv_pool *pool);
bool ext_xv_is_dyn_int_pool(struct ext_xv_pool *pool);
u64 ext_xv_show_total_pages(struct ext_xv_pool *pool);

void __iomem**  ext_xv_pool_migration(struct ext_xv_pool *pool, u32 type, void __iomem **tmp_array, unsigned int *tmp_xbitmap, struct ext_xv_backuplist *backuplist);
void __iomem** ext_xv_create_pool_array(struct ext_xv_pool *pool, u32 type, unsigned int *tmp_xbitmap);
void ext_xv_delete_pool_array(struct ext_xv_pool *pool, void __iomem **tmp_array);
unsigned int * ext_xv_create_bitmap(struct ext_xv_pool *pool);
void ext_xv_delete_bitmap(unsigned int *bitmap);
unsigned int ext_xv_add_page_to_backuplist(struct ext_xv_backuplist *backuplist, int cnt);
void ext_xv_set_migration(struct ext_xv_pool *pool);
void ext_xv_clear_migration(struct ext_xv_pool *pool);
void ext_xv_add_read_cnt(struct ext_xv_pool *pool, int cnt);
void ext_xv_sub_read_cnt(struct ext_xv_pool *pool, int cnt);
void ext_xv_add_write_cnt(struct ext_xv_pool *pool, int cnt);
void ext_xv_sub_write_cnt(struct ext_xv_pool *pool, int cnt);
void ext_xv_set_migration(struct ext_xv_pool *pool);
void ext_xv_clear_migration(struct ext_xv_pool *pool);
unsigned int ext_xv_get_migration(struct ext_xv_pool *pool);
unsigned int ext_xv_get_read_cnt(struct ext_xv_pool *pool);
unsigned int ext_xv_get_write_cnt(struct ext_xv_pool *pool);
void ext_xv_bitmap_duplicate(struct ext_xv_pool *pool, unsigned int *bitmap);
void ext_xv_show_block_info(struct ext_xv_pool *pool);

#endif
