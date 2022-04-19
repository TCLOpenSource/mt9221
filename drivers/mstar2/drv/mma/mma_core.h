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

#ifndef _MMA_CORE_H_
#define _MMA_CORE_H_

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/kref.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/printk.h>
#include <linux/dcache.h>
#include <linux/idr.h>
#include <linux/fs.h>

#include "mma_common.h"


#define CHECK_POINTER(pointer,result)      \
        if (IS_ERR_OR_NULL(pointer))        \
        {\
            printk(KERN_ERR "%s  %d,pointer is NULL or Error\n",__FUNCTION__, __LINE__);\
            return result;\
        } \

#define CHECK_RETURN(result)      \
        if (result < 0)   \
        {\
            printk(KERN_ERR "%s  %d, Failed!\n",__FUNCTION__, __LINE__);\
            return result;\
        }\

#define PRINT_AND_RETURN(err) \
        do { \
            printk(KERN_ERR "%s  %d, failed\n",__FUNCTION__, __LINE__);\
            return err; \
        }while(0)


#define MMA_FLAG_IOVA    (1<< 0) //buffer need IOVA, 1:yes, 0:no
#define MMA_FLAG_DMA     (1<< 1)  //buffer allocate from dma zone, 1:yes, 0:no
#define MMA_FLAG_CACHE   (1<< 2) //buffer 1:cache ,0 :uncache
#define MMA_FLAG_2XIOVA  (1<< 3) //1:mapping double size iova


struct mma_buf_handle {
	char buf_tag[MAX_NAME_SIZE];
	struct delayed_work work;
	struct dma_buf *dmabuf;
	struct dma_buf *db_ion;
	struct list_head buf_list_node;
	struct list_head dfree_list_node;
	struct mma_device *mma_dev;
    struct sg_table *sgt;
	struct dentry *entry;
	size_t length;
	int dmabuf_fd;  //dmabuf fd
	u64 addr; //buffer address
	bool is_iova; //whether addr is iova
	bool is_secure; //whethe secure buffer
	int miu_select; //which miu
	int ref; //reference count
	int global_name;
    void* kvaddr;
	pid_t tpid;
    u32 serial;
    u32 alloc_time;
    u32 pipe_id;
    u32 auth_count;// authorize ref count
    u32 kmap_cnt;
    struct mutex handle_lock;
    int tee_map;
    struct page **pages;
    u32 num_pages;
    int flag;
};

struct mma_space_handle {
	struct mma_reserve_iova_data data;
	struct list_head list_node;
};

struct mma_userdev_info {
	pid_t tpid;
	pid_t mmap_tpid;
	struct semaphore sem;
	void *private_data;
	void *tmp;
};

/**
 * struct mma_device - mma device node private data
 * @misc_dev:	the misc device
 * @head:	list of allocator
 * @lock:	list and secure pointer mutex
 * @secure: pointer to secure functions helpers
 */
struct mma_device {
	struct miscdevice misc_dev;
	struct mutex buf_lock;
	struct list_head buf_list_head;
	struct list_head space_list_head;

	struct list_head dfree_list_head;
	struct list_head physical_buf_list_head;
	struct list_head physical_dfree_list_head;

	struct kobject *mma_kobj;
	struct list_head debugfs_list_head;

	struct idr global_name_idr;
	struct file* file_ion;
	struct kref ref;
    struct dentry *debug_root;
	struct dentry *buf_debug_root;
    u32 record_alloc_time;
    u32 calltrace_enable;
    struct workqueue_struct *mma_wq;
};

extern struct mma_device mma_dev;
#define MMA_DEBUG(fmt, args...) \
        if(mma_dev.calltrace_enable == 2) \
            printk(KERN_INFO"[IOMMU][%s][%d][pid:%d] " fmt,__FUNCTION__,__LINE__,current->tgid, ## args)

struct mma_buf_handle* __mma_create_buf_handle(u32 length, const char* buf_tag);

bool __is_mma_dmabuf(struct dma_buf *dmabuf);
int	__mma_alloc_internal(const char* buf_tag, u32 size, bool secure, u64* addr_out, int* dmabuf_fd, int flag);
int __mma_reserve_iova_internal(struct mma_reserve_iova_data *data);
int __mma_free_iova_internal(const char *space_tag);
int __mma_set_cache_flag(int dmabuf_fd, bool cached);
void* __mma_kmap_internel(struct dma_buf *db, unsigned long offset, size_t len);
int __mma_kunmap_internel(struct dma_buf *db, void* vaddr, size_t len);

struct mma_buf_handle* __mma_find_buf_handle(void *vaddr);
int __mma_get_space_tag(const char* buf_tag, char** space_tag);
int __mma_handle_release(struct mma_buf_handle *handle);
int __mma_support(void);

#endif
