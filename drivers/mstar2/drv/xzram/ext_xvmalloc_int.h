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

#ifndef _XV_MALLOC_INT_H_
#define _XV_MALLOC_INT_H_

#include <linux/kernel.h>
#include <linux/types.h>

/* User configurable params */

/* Must be power of two */
#ifdef CONFIG_64BIT
#define XV_ALIGN_SHIFT 3
#else
#define XV_ALIGN_SHIFT	2
#endif
#define XV_ALIGN	(1 << XV_ALIGN_SHIFT)
#define XV_ALIGN_MASK	(XV_ALIGN - 1)

/* This must be greater than sizeof(link_free) */
#define XV_MIN_ALLOC_SIZE	32
#define XV_MAX_ALLOC_SIZE	(PAGE_SIZE - XV_ALIGN)

/*
 * Free lists are separated by FL_DELTA bytes
 * This value is 3 for 4k pages and 4 for 64k pages, for any
 * other page size, a conservative (PAGE_SHIFT - 9) is used.
 */
#if PAGE_SHIFT == 16
#define FL_DELTA_SHIFT 4
#else
#define FL_DELTA_SHIFT (PAGE_SHIFT - 9)
#endif
#define FL_DELTA	(1 << FL_DELTA_SHIFT)
#define FL_DELTA_MASK	(FL_DELTA - 1)
#define NUM_FREE_LISTS	((XV_MAX_ALLOC_SIZE - XV_MIN_ALLOC_SIZE) \
				/ FL_DELTA + 1)

#define MAX_FLI		DIV_ROUND_UP(NUM_FREE_LISTS, BITS_PER_LONG)

/* End of user params */

enum blockflags {
	BLOCK_FREE,
	PREV_FREE,
	__NR_BLOCKFLAGS,
};

#define FLAGS_MASK	XV_ALIGN_MASK
#define PREV_MASK	(~FLAGS_MASK)

struct freelist_entry {
	struct page *page;
	u16 offset;
	u16 pad;
	u32 num;
};

struct link_free {
	struct page *prev_page;
	struct page *next_page;
	u16 prev_offset;
	u16 next_offset;
};

struct block_header {
	union {
		/* This common header must be XV_ALIGN bytes */
		u8 common[XV_ALIGN];
		struct {
			u16 size;
			u16 prev;
		};
	};
	struct link_free link;
};

struct ext_xv_pool {
	ulong flbitmap;
	ulong slbitmap[MAX_FLI];
	u64 total_pages;	/* stats */
        u8  pool_id;
	struct freelist_entry freelist[NUM_FREE_LISTS];
	spinlock_t lock;
        struct list_head list;
        struct xv_ext_memory_attr *ext_pool_attr;
        u8  type;
  struct migration xv_migration;
};

struct xv_ext_memory_attr{
        void __iomem *ba_start;
        void __iomem **ext_xv_pool_array;
        u32 *xbitmap;
        unsigned int size;
};

#endif
