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

#ifndef _UAPI_MMA_IOCTL_H_
#define _UAPI_MMA_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define MAX_NAME_SIZE	16
#define UUID_SIZE		16
#define MAX_TAG_NUM     8

#define MMA_DEV_NAME "/dev/mma"

struct  mma_alloc_data {
    union {
        char tag_name[MAX_NAME_SIZE];
        char heap_name[MAX_NAME_SIZE];
    };
    __s32 pipeid;
    __u64 addr;
    __u32 len;
    __s32 dmabuf_fd;
    __u32 bSecure; //whethe secure buffer
    char  miu_select;
};

struct mma_alloc_data_v2 {
	union {
	char tag_name[MAX_NAME_SIZE];
	char heap_name[MAX_NAME_SIZE];
	};
	int pipeid;
	__u64 addr;
	__u32 len;
	int dmabuf_fd;
	__u32 bSecure; //whethe secure buffer
	char miu_select;
	__u32 flag;
};

struct mma_map_iova_data {
	__u64 addr;
	int dmabuf_fd;
};


struct  mma_buftag_data {
    char tag_name[MAX_NAME_SIZE];
    __u32 heaptype;
    __u32 miu_number;
    __u32 max_size;
};


struct mma_reserve_iova_data {
    char space_tag[MAX_NAME_SIZE];
    char buf_tag_array[MAX_TAG_NUM][MAX_NAME_SIZE];
    __u64 base_addr; //return start address
    __u64 size; //reserve size
    __u32 buf_tag_num;
};

struct mma_fd_data {
    __s32 dmabuf_fd;
    __u32 name; //global name
    __u64 vaddr;  //virtual address
    __u32 offset; //map offset
    __u32 len; //map length
    __u32 bCached; //whethe cached
    __u32 unused;
    __u64 addr;//IOVA or PA
};

struct mma_ion_handle_data {
    int handle;
    __s32 dmabuf_fd;
};
struct mma_va_iova {
    char tag_name[MAX_NAME_SIZE];
    __u64 addr;
    __u32 len;
    __u64 vaddr;
    int   fd;
};

#define MMA_IOC_MAGIC  'M'

#define MMA_IOC_RESERVE_IOVA_SPACE    _IOWR(MMA_IOC_MAGIC, 0, struct mma_reserve_iova_data)

#define MMA_IOC_FREE_IOVA_SPACE       _IOW(MMA_IOC_MAGIC, 1, struct mma_reserve_iova_data)

#define MMA_IOC_GET_PIPEID             _IOR(MMA_IOC_MAGIC, 2, struct mma_alloc_data)

#define MMA_IOC_ALLOC                 _IOWR(MMA_IOC_MAGIC, 3, struct mma_alloc_data)

#define MMA_IOC_AUTHORIZE             _IOW(MMA_IOC_MAGIC, 4, struct mma_alloc_data)

#define MMA_IOC_FREE                  _IOW(MMA_IOC_MAGIC, 5, struct mma_fd_data)

#define MMA_IOC_SET_CACHE_FLAG        _IOW(MMA_IOC_MAGIC, 6, struct mma_fd_data)

#define MMA_IOC_UNMAP                 _IOWR(MMA_IOC_MAGIC, 7,struct mma_fd_data)

#define MMA_IOC_FLUSH                 _IOW(MMA_IOC_MAGIC, 8, struct mma_fd_data)

#define MMA_IOC_EXPORT                _IOWR(MMA_IOC_MAGIC, 9,struct mma_fd_data)

#define MMA_IOC_IMPORT                _IOWR(MMA_IOC_MAGIC, 10, struct mma_fd_data)

#define MMA_IOC_GET_MEMINFO           _IOWR(MMA_IOC_MAGIC, 11,struct mma_alloc_data)

#define MMA_IOC_GET_HEAPINFO          _IOWR(MMA_IOC_MAGIC, 12,struct mma_alloc_data)

#define MMA_IOC_QUERY_BUFTAG          _IOWR(MMA_IOC_MAGIC, 13,struct mma_buftag_data)

#define MMA_IOC_UNAUTHORIZE             _IOWR(MMA_IOC_MAGIC, 14,struct mma_fd_data)

#define MMA_IOC_PHYSICAL_AUTHORIZE     _IOW(MMA_IOC_MAGIC, 15, struct mma_alloc_data)

#define MMA_IOC_PHYSICAL_UNAUTHORIZE   _IOW(MMA_IOC_MAGIC, 16, struct mma_alloc_data)

#define MMA_IOC_QUERY_PIPELINE_ID      _IOWR(MMA_IOC_MAGIC, 17, struct mma_alloc_data)

#define MMA_IOC_QUERY_GLOBAL_NAME      _IOWR(MMA_IOC_MAGIC, 18, struct mma_fd_data)

#define MMA_IOC_ALLOC_V2                _IOWR(MMA_IOC_MAGIC, 19, struct mma_alloc_data_v2)

#define MMA_IOC_MAP_IOVA                _IOWR(MMA_IOC_MAGIC, 20, struct mma_map_iova_data)

#define MMA_IOC_ION_IMPORT              _IOWR(MMA_IOC_MAGIC, 21, struct mma_ion_handle_data)

#define MMA_IOC_ION_FREE                _IOWR(MMA_IOC_MAGIC, 22, struct mma_ion_handle_data)

#define MMA_IOC_VA2IOVA                _IOWR(MMA_IOC_MAGIC, 23, struct  mma_va_iova)

#define MMA_IOC_PUT_PIPEID             _IOW(MMA_IOC_MAGIC, 24, struct mma_alloc_data)

#define MMA_IOC_RESIZE                 _IOWR(MMA_IOC_MAGIC, 25, struct mma_alloc_data_v2)

#define MMA_IOC_SUPPORT                _IOWR(MMA_IOC_MAGIC, 26, int)

#endif
