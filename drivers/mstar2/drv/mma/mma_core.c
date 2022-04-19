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
 * smaf.c
 *
 * Copyright (C) Linaro SA 2015
 * Author: Benjamin Gaignard <benjamin.gaignard at linaro.org> for Linaro.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/ion.h>
#include <linux/notifier.h>
#include <linux/syscalls.h>
#include <linux/debugfs.h>
#include <linux/cma.h>
#include <asm/cacheflush.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>

#include "mma_core.h"
#include "mma_tee_inf.h"
#include "mma_of.h"
#include "mma_api.h"
#include "mma_ion.h"
#include "mma_debugfs.h"
#include "MsTypes.h"
extern struct CMA_BootArgs_Config cma_config[MAX_CMA_AREAS];

struct mma_device mma_dev;
static int mma_support = 0;

union mma_ioctl_data{
	struct mma_fd_data fd;
	struct mma_alloc_data allocation;
	struct mma_reserve_iova_data reservation;
	struct mma_buftag_data buftag;
	struct mma_alloc_data_v2 alloc_v2;
	struct mma_map_iova_data map;
	struct mma_ion_handle_data handle;
	struct mma_va_iova va;
	int support;
};

struct mma_flag {
	int miu;
	int ion_flag;
	int zone_flag;
	u32 heap_mask;
	enum mma_of_heap_type heap_type;
	char tag[MAX_NAME_SIZE];
	char* space_tag;
	u64 addr;
	u64 max_size;
	u32 _2x;
	int db_ion_fd;
	struct dma_buf *db_ion;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	u32 map_time_us;
	struct timeval tv1;
	struct timeval tv2;
	struct timeval tv3;
};

static void mma_dma_buf_release(struct dma_buf *dmabuf);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
static void mma_pages_sync_for_device(struct device *dev, struct page *page,
			       size_t size, enum dma_data_direction dir)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, page, size, 0);
	/*
	 * This is not correct - sg_dma_address needs a dma_addr_t that is valid
	 * for the targeted device, but this works on the currently targeted
	 * hardware.
	 */
	sg_dma_address(&sg) = page_to_phys(page);
	dma_sync_sg_for_device(dev, &sg, 1, dir);
}
#endif

static struct sg_table *mma_map_dma_buf(struct dma_buf_attachment *attachment,
					 enum dma_data_direction direction)
{
	struct dma_buf_attachment *db_attachment;
	struct dma_buf *dmabuf = attachment->dmabuf;
	struct mma_buf_handle *handle = dmabuf->priv;
	struct sg_table *sgt;

	CHECK_POINTER(handle->db_ion, NULL);

	db_attachment = (struct dma_buf_attachment *)attachment->priv;
	if(IS_ERR_OR_NULL(db_attachment)){
		printk(KERN_ERR "%s  %d,db_attachment NULL\n",__FUNCTION__, __LINE__);
		return NULL;
	}
	sgt = dma_buf_map_attachment(db_attachment, direction);

	CHECK_POINTER(sgt, NULL);

	return sgt;
}

static void mma_unmap_dma_buf(struct dma_buf_attachment *attachment,
			       struct sg_table *sgt,
			       enum dma_data_direction direction)
{
	struct dma_buf_attachment *db_attachment;
	struct dma_buf *dmabuf = attachment->dmabuf;
	struct mma_buf_handle *handle = dmabuf->priv;

	if(handle->db_ion == NULL)
		return;

	db_attachment = (struct dma_buf_attachment *)attachment->priv;
	if(IS_ERR_OR_NULL(db_attachment)){
		printk(KERN_ERR "%s  %d,db_attachment NULL\n",__FUNCTION__, __LINE__);
		return;
	}
	dma_buf_unmap_attachment(db_attachment, sgt, direction);
}

static int mma_dma_buf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct mma_buf_handle *handle = dmabuf->priv;
	MMA_DEBUG("map user tag=%s,addr=0x%llx ,size=0x%zx\n", handle->buf_tag,handle->addr, handle->length);
	return dma_buf_mmap(handle->db_ion, vma, vma->vm_pgoff);
}

int __mma_handle_release(struct mma_buf_handle *handle)
{
	int ret;
	enum mma_addr_type addr_type;
	struct mma_range_t range_out;
	struct mma_buf_handle *tmp = NULL;
	struct task_struct *task = current;
	struct file *file;
	struct ion_buffer *buffer = NULL;
	struct scatterlist *sg = NULL;
	int i = 0;

	if(!handle)
		return -1;
	MMA_DEBUG("tag:%s,size=0x%zx ,va=%lx,addr=0x%llx\n",
		handle->buf_tag, handle->length, (unsigned long)handle->kvaddr, handle->addr);
	if(handle->kvaddr != NULL) {
		__mma_kunmap_internel(handle->dmabuf, handle->kvaddr, handle->length);
		handle->kvaddr = NULL;
	}

	if(handle->is_secure && handle->auth_count != 0) {
		ret = mma_tee_unauthorize(handle->addr,handle->length,handle->buf_tag,handle->pipe_id,&range_out);
		if (MMA_UNAUTH_DELAY_FREE == ret) {
			mutex_lock(&mma_dev.buf_lock);
			list_for_each_entry(tmp, &mma_dev.dfree_list_head, dfree_list_node) {
				if(handle == tmp){
					mutex_unlock(&mma_dev.buf_lock);
					goto out;
				}
			}
			list_add(&handle->dfree_list_node, &mma_dev.dfree_list_head);
			mutex_unlock(&mma_dev.buf_lock);
			goto out;
		} else if (MMA_UNAUTH_FREE_RANGE == ret){
			handle->auth_count = 0;
			mutex_lock(&mma_dev.buf_lock);
			list_for_each_entry(tmp, &mma_dev.dfree_list_head, dfree_list_node) {
				if((tmp->addr >= range_out.start) &&(tmp->addr < (range_out.start+ range_out.size))) {
					list_del(&tmp->dfree_list_node);
					mutex_unlock(&mma_dev.buf_lock);
					mma_dma_buf_release(tmp->dmabuf);
				}
			}
			mutex_unlock(&mma_dev.buf_lock);
		}
		handle->auth_count = 0;
	}

	mutex_lock(&mma_dev.buf_lock);
	if(handle->global_name >= 0){
		idr_remove(&mma_dev.global_name_idr, handle->global_name);
		handle->global_name = -1;
	}

	list_for_each_entry(tmp, &mma_dev.buf_list_head, buf_list_node) {
		if(tmp == handle) {
			list_del(&handle->buf_list_node);
			break;
		}
	}
	mutex_unlock(&mma_dev.buf_lock);

	if(handle->tee_map == 0){
		handle->tee_map = -1;
		cancel_delayed_work_sync(&handle->work);
	}
	mutex_lock(&handle->handle_lock);
	if(handle->is_iova)
	{
		addr_type = mma_tee_addr_type(handle->addr);
		ret = mma_tee_unmap(addr_type, handle->addr, &range_out);
		handle->is_iova = 0;
	}

	if(handle->entry != NULL){
		debugfs_remove(handle->entry);
		handle->entry = NULL;
	}

	if(handle->pages) {
		if(handle->num_pages) {
			for(i = 0;i < handle->num_pages;i++){
				put_page(handle->pages[i]);
			}
		}
		kfree(handle->pages);
		handle->pages = NULL;
		if(handle->sgt){
			sg_free_table(handle->sgt);
			kfree(handle->sgt);
			handle->sgt = NULL;
		}
	}

	if (handle->db_ion) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
		buffer = (struct ion_buffer *)handle->db_ion->priv;
		if (buffer->flags & ION_FLAG_CACHED){
			ion_set_buffer_cached(handle->db_ion, 0);
			if(handle->sgt){
				for_each_sg(handle->sgt->sgl, sg, handle->sgt->nents, i)
					mma_pages_sync_for_device(NULL, sg_page(sg), sg->length,DMA_BIDIRECTIONAL);
			}
		}
#endif

		handle->sgt = NULL;
		file = handle->db_ion->file;
		if (!file)
			goto free;
		if(task->flags & PF_KTHREAD){
			__fput_sync(file);
		}else{
			handle->db_ion->ops->release(handle->db_ion);
			handle->db_ion->priv = NULL;
			dma_buf_put(handle->db_ion);
		}
		handle->db_ion = NULL;
	}
free:
	mutex_unlock(&handle->handle_lock);
	return 1;

out:
	return 0;
}

static void mma_dma_buf_release(struct dma_buf *dmabuf)
{
	int ret = 0;
	struct mma_buf_handle *handle = dmabuf->priv;

	if(!handle)
		return;
	MMA_DEBUG("tag:%s,size=0x%zx ,va=%p,addr=0x%llx\n",
		handle->buf_tag, handle->length, handle->kvaddr, handle->addr);

	ret = __mma_handle_release(handle);
	if(ret){
		kfree(handle);
		dmabuf->priv = NULL;
	}
	return;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
static int mma_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					 enum dma_data_direction direction)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	CHECK_POINTER(handle->db_ion, -EINVAL);

	return dma_buf_begin_cpu_access(handle->db_ion, direction);
}

static int mma_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
					enum dma_data_direction direction)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	if (handle->db_ion)
		dma_buf_end_cpu_access(handle->db_ion, direction);
	return 0;
}
#else
static int mma_dma_buf_begin_cpu_access(struct dma_buf *dmabuf, size_t start,
					 size_t len,
					 enum dma_data_direction direction)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	CHECK_POINTER(handle->db_ion, -EINVAL);

	return dma_buf_begin_cpu_access(handle->db_ion,
					start, len, direction);
}

static void mma_dma_buf_end_cpu_access(struct dma_buf *dmabuf, size_t start,
					size_t len,
					enum dma_data_direction direction)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	if (handle->db_ion)
		dma_buf_end_cpu_access(handle->db_ion, start, len, direction);
}


#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
static void *mma_dma_buf_kmap_atomic(struct dma_buf *dmabuf,
				      unsigned long offset)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	CHECK_POINTER(handle->db_ion, NULL);

	return dma_buf_kmap_atomic(handle->db_ion, offset);
}

static void mma_dma_buf_kunmap_atomic(struct dma_buf *dmabuf,
				       unsigned long offset, void *ptr)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	if(handle->db_ion == NULL)
		return;

	dma_buf_kunmap_atomic(handle->db_ion, offset, ptr);
}
#endif

static void *mma_dma_buf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	CHECK_POINTER(handle->db_ion, NULL);


	return dma_buf_kmap(handle->db_ion, offset);
}

static void mma_dma_buf_kunmap(struct dma_buf *dmabuf, unsigned long offset,
				void *ptr)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	if(handle->db_ion == NULL)
		return;

	dma_buf_kunmap(handle->db_ion, offset, ptr);
}

static void *mma_dma_buf_vmap(struct dma_buf *dmabuf)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	if(handle->db_ion == NULL)
		return NULL;

	return dma_buf_vmap(handle->db_ion);
}

static void mma_dma_buf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct mma_buf_handle *handle = dmabuf->priv;

	if(handle->db_ion == NULL)
		return;

	dma_buf_vunmap(handle->db_ion, vaddr);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static int mma_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *attach)
{
	struct mma_buf_handle *handle = dmabuf->priv;
	struct dma_buf_attachment *db_attach;

	CHECK_POINTER(handle->db_ion, -EFAULT);

	db_attach = dma_buf_attach(handle->db_ion, attach->dev);

	attach->priv = (void *)db_attach;
	return IS_ERR(db_attach);
}
#else
static int mma_attach(struct dma_buf *dmabuf, struct device *dev,
		       struct dma_buf_attachment *attach)
{
	struct mma_buf_handle *handle = dmabuf->priv;
	struct dma_buf_attachment *db_attach;

	CHECK_POINTER(handle->db_ion, -EFAULT);

	db_attach = dma_buf_attach(handle->db_ion, dev);
	attach->priv = (void *)db_attach;
	return IS_ERR(db_attach);
}
#endif
static void mma_detach(struct dma_buf *dmabuf,
			struct dma_buf_attachment *attach)
{
	struct mma_buf_handle *handle = dmabuf->priv;
	struct dma_buf_attachment *db_attachment;

	if(handle->db_ion == NULL)
		return;
	db_attachment = (struct dma_buf_attachment *)attach->priv;
	if(IS_ERR_OR_NULL(db_attachment)){
		printk(KERN_ERR "%s  %d,db_attachment NULL\n",__FUNCTION__, __LINE__);
		return;
	}
	dma_buf_detach(handle->db_ion, db_attachment);
}

static struct dma_buf_ops mma_dma_buf_ops = {
	.attach = mma_attach,
	.detach = mma_detach,
	.map_dma_buf = mma_map_dma_buf,
	.unmap_dma_buf = mma_unmap_dma_buf,
	.release = mma_dma_buf_release,
	.begin_cpu_access = mma_dma_buf_begin_cpu_access,
	.end_cpu_access = mma_dma_buf_end_cpu_access,
	.mmap = mma_dma_buf_mmap,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
	.kmap_atomic = mma_dma_buf_kmap_atomic,
	.kunmap_atomic = mma_dma_buf_kunmap_atomic,
	.kmap = mma_dma_buf_kmap,
	.kunmap = mma_dma_buf_kunmap,
#else
	.map = mma_dma_buf_kmap,
	.unmap = mma_dma_buf_kunmap,
#endif
	.vunmap = mma_dma_buf_vunmap,
};

int mma_maxima_check(const char* buf_tag, u32 size, u64 max)
{
	struct mma_buf_handle* handle;
	u64 total = 0;
	mutex_lock(&mma_dev.buf_lock);
	list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
		if(!strncmp(handle->buf_tag,buf_tag,MAX_NAME_SIZE))
			total += handle->length;
	}
	mutex_unlock(&mma_dev.buf_lock);
	total += size;
	if(total > max){
		if(mma_dev.calltrace_enable == 2) {
			mutex_lock(&mma_dev.buf_lock);
			list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
				if(!strncmp(handle->buf_tag, buf_tag, MAX_NAME_SIZE))
					printk(KERN_ERR"buf_tag: %s, addr: 0x%llx, size: 0x%zx, file_count: %ld\n",
						buf_tag, handle->addr, handle->length, file_count(handle->dmabuf->file));
			}
			mutex_unlock(&mma_dev.buf_lock);
		}
		printk(KERN_ERR"%s  %d, Failed! buf_tag=%s,total=0x%llx > max=0x%llx\n",__FUNCTION__, __LINE__,buf_tag,total,max);
		return -1;
	}else
		return 0;
}

static void do_TEEMap(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct mma_buf_handle *handle = container_of(dwork, struct mma_buf_handle, work);
	int ret = 0;
	u64 addr = 0;
	char* space_tag = NULL;
	struct timeval tv0, tv1;
	u32 maptime = 0;

	if(IS_ERR_OR_NULL(handle) || IS_ERR_OR_NULL(handle->sgt))
		return;

	if(mma_dev.record_alloc_time){
		memset(&tv0, 0, sizeof(struct timeval));
		memset(&tv1, 0, sizeof(struct timeval));
		do_gettimeofday(&tv0);
	}

	mutex_lock(&handle->handle_lock);
	if(handle->tee_map == 0)
		handle->tee_map = 1;
	else
		goto out;

	if(handle->addr > 0){
		goto out;
	}

	__mma_get_space_tag(handle->buf_tag, &space_tag);

	ret = mma_tee_map(space_tag, handle->sgt, &addr,
            handle->flag & MMA_FLAG_2XIOVA,handle->buf_tag);
	if(ret != 0){
		printk(KERN_ERR"%s  %d,bugtag=%s Failed! \n",__FUNCTION__, __LINE__,handle->buf_tag);
		goto out;
	}

	handle->is_iova = addr & IOVA_START_ADDR;
	handle->addr = addr;
out:
	if(mma_dev.record_alloc_time){
		do_gettimeofday(&tv1);
		maptime = (tv1.tv_sec - tv0.tv_sec)*1000*1000 + (tv1.tv_usec - tv0.tv_usec);
		printk(KERN_ERR "%s  %d,addr =0x%llx,  maptime(us) =%d!\n",__FUNCTION__, __LINE__,addr,maptime);
	}
	mutex_unlock(&handle->handle_lock);
	return;
}

bool __is_mma_dmabuf(struct dma_buf *dmabuf)
{
	return dmabuf->ops == &mma_dma_buf_ops;
}


struct mma_buf_handle* __mma_create_buf_handle(u32 length, const char* buf_tag)
{
	int fd;
    static u64 serial_num = 0;
	struct mma_buf_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	CHECK_POINTER(handle, ERR_PTR(ENOMEM));

#if defined(DEFINE_DMA_BUF_EXPORT_INFO)

	DEFINE_DMA_BUF_EXPORT_INFO(info);
	info.ops = &mma_dma_buf_ops;
	info.size = length;
	info.flags = O_RDWR;
	info.priv = handle;

	handle->dmabuf = dma_buf_export(&info);
#else

	handle->dmabuf = dma_buf_export(handle, &mma_dma_buf_ops, length,
				O_RDWR);
#endif
	if (IS_ERR(handle->dmabuf)) {
		kfree(handle);
		PRINT_AND_RETURN(ERR_PTR(-EINVAL));
	}

	handle->length = length;
	strncpy(handle->buf_tag, buf_tag, MAX_NAME_SIZE);
	handle->buf_tag[MAX_NAME_SIZE - 1] = '\0';
	fd = dma_buf_fd(handle->dmabuf, O_CLOEXEC);
	if (fd < 0) {
		dma_buf_put(handle->dmabuf);
		PRINT_AND_RETURN(ERR_PTR(fd));
	}

	handle->dmabuf_fd = fd;
	handle->mma_dev = &mma_dev;
    handle->tpid = current->tgid;
    handle->global_name = -1;
    handle->kvaddr = NULL;
    handle->serial = serial_num++;
    handle->entry = NULL;
    handle->kmap_cnt = 0;
	mutex_init(&handle->handle_lock);
	return handle;
}


int __mma_get_space_tag(const char* buf_tag, char** space_tag)
{
	int i;
	struct mma_space_handle* handle;

	CHECK_POINTER(buf_tag, -EINVAL);
	CHECK_POINTER(space_tag, -EINVAL);

	*space_tag = NULL;
	if(list_empty(&mma_dev.space_list_head))
		return 0;
	list_for_each_entry(handle, &mma_dev.space_list_head, list_node) {
		for(i=0;i<handle->data.buf_tag_num;i++) {
			if(!strncmp(buf_tag, handle->data.buf_tag_array[i],MAX_NAME_SIZE)) {
				*space_tag = handle->data.space_tag;
				return 0;
			}
		}
	}
	return 0;
}

int	__mma_alloc_internal(const char* buf_tag, u32 size, bool secure, u64* addr_out, int* dmabuf_fd, int flag)
{
	int ret;
	struct mma_buf_handle *buf_handle = NULL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	struct ion_buffer *buffer;
#endif
	struct mma_flag *data = NULL;

	CHECK_POINTER(buf_tag, -EINVAL);
	CHECK_POINTER(dmabuf_fd, -EINVAL);
	CHECK_POINTER(addr_out, -EINVAL);

	data = kzalloc(sizeof(struct mma_flag), GFP_KERNEL);
	CHECK_POINTER(data, -ENOMEM);

	strncpy(data->tag, buf_tag, MAX_NAME_SIZE);
	data->tag[MAX_NAME_SIZE - 1] = '\0';
	data->_2x = flag & MMA_FLAG_2XIOVA;

	if(mma_dev.calltrace_enable == 1)
		dump_stack();

	do_gettimeofday(&data->tv1);

	ret = mma_of_get_buftag_info(data->tag, &data->heap_type, &data->miu,
			&data->max_size, &data->zone_flag);
	if (ret) {
		printk(KERN_ERR "%s  %d,get_buftag Failed!\n",__FUNCTION__, __LINE__);
		kfree(data);
		return -EINVAL;
	}

	ret = mma_maxima_check(data->tag, size, data->max_size);
	if (ret) {
		printk(KERN_ERR "%s  %d,oversize Failed!\n",__FUNCTION__, __LINE__);
		kfree(data);
		return -EINVAL;
	}

	if(flag & MMA_FLAG_DMA)
		data->zone_flag = 0;// allocate from dma zone
	ret = mma_ion_get_id_flag(data->heap_type, data->miu, data->zone_flag,
			secure, &data->heap_mask, &data->ion_flag, size, data->tag);
	if (ret) {
		printk(KERN_ERR "%s  %d,get ion flag Failed!\n",__FUNCTION__, __LINE__);
		kfree(data);
		return -EINVAL;
	}


ALLOCATE_AGAIN:
	ret = mma_ion_alloc(&mma_dev,size, data->heap_mask, data->ion_flag, &data->db_ion_fd);
	if(ret < 0){
		if(data->ion_flag & ION_FLAG_DMAZONE){
			data->ion_flag &= ~ION_FLAG_DMAZONE;
			printk(KERN_ERR "%s  %d,tag:%s,size=0x%x DMAZONE alloc Failed!fallback to high zone\n",
				__FUNCTION__, __LINE__,data->tag,size);
			goto ALLOCATE_AGAIN;
		}else{
			printk(KERN_ERR "%s  %d,ion allocate Failed!\n",__FUNCTION__, __LINE__);
			kfree(data);
			return ret;
		}
	}

	data->db_ion = dma_buf_get(data->db_ion_fd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	ksys_close(data->db_ion_fd);
#else
	sys_close(data->db_ion_fd);
#endif

	if (IS_ERR(data->db_ion)) {
		kfree(data);
        PRINT_AND_RETURN(-EINVAL);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	buffer = (struct ion_buffer *)data->db_ion->priv;
	if (IS_ERR(buffer)) {
		dma_buf_put(data->db_ion);
		kfree(data);
		PRINT_AND_RETURN(-EINVAL);
	}

	data->sgt = buffer->sg_table;
	if (IS_ERR(data->sgt)) {
		dma_buf_put(data->db_ion);
		kfree(data);
		PRINT_AND_RETURN(-EINVAL);
	}

#else

	data->attach = dma_buf_attach(data->db_ion, mma_dev.misc_dev.this_device);
	if (IS_ERR(data->attach)) {
		dma_buf_put(data->db_ion);
		kfree(data);
		PRINT_AND_RETURN(-EINVAL);
	}

	data->sgt = dma_buf_map_attachment(data->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(data->sgt)) {
		dma_buf_detach(data->db_ion, data->attach);
		dma_buf_put(data->db_ion);
		kfree(data);
		PRINT_AND_RETURN(-EINVAL);
	}

#endif

	__mma_get_space_tag(data->tag, &data->space_tag);
	do_gettimeofday(&data->tv2);

	if(data->heap_type == HEAP_TYPE_IOMMU
		|| data->heap_type == HEAP_TYPE_CMA_IOMMU
		|| data->heap_type == HEAP_TYPE_CARVEOUT) {
			if(flag & MMA_FLAG_IOVA)
				ret = mma_tee_map(data->space_tag, data->sgt, &data->addr, data->_2x, data->tag);
			data->miu = 0;
	}else{
		printk("%s  %d, heap_type invalid\n",__FUNCTION__, __LINE__);
		dma_buf_put(data->db_ion);
		kfree(data);
		return -EINVAL;
	}
	do_gettimeofday(&data->tv3);
	data->map_time_us = (data->tv3.tv_sec - data->tv2.tv_sec)*1000*1000 +
			(data->tv3.tv_usec - data->tv2.tv_usec);

	if (ret < 0) {
		if(data->attach)
			dma_buf_detach(data->db_ion, data->attach);
		dma_buf_put(data->db_ion);
		kfree(data);
		PRINT_AND_RETURN(ret);
	}

	buf_handle = __mma_create_buf_handle(size, data->tag);
	if (IS_ERR(buf_handle)) {
		if(data->attach)
			dma_buf_detach(data->db_ion, data->attach);
		dma_buf_put(data->db_ion);
		kfree(data);
		PRINT_AND_RETURN(-EINVAL);
	}

	buf_handle->db_ion = data->db_ion;
	buf_handle->addr = data->addr;
	buf_handle->is_secure = false;
	buf_handle->is_iova = data->addr & IOVA_START_ADDR;
	buf_handle->auth_count = 0;
	buf_handle->miu_select = data->miu;
	buf_handle->sgt = data->sgt;
	buf_handle->flag = flag;
	mutex_lock(&mma_dev.buf_lock);
	list_add(&buf_handle->buf_list_node, &mma_dev.buf_list_head);
	mutex_unlock(&mma_dev.buf_lock);
	*dmabuf_fd = buf_handle->dmabuf_fd;
	*addr_out = data->addr;

	mma_creat_buffer_file(buf_handle, mma_dev.buf_debug_root);
	if(data->attach)
		dma_buf_detach(data->db_ion, data->attach);

	if(flag & MMA_FLAG_CACHE)
		ion_set_buffer_cached(data->db_ion, 1);

	if(!(flag & MMA_FLAG_IOVA)){
		buf_handle->tee_map = 0;
		INIT_DELAYED_WORK(&buf_handle->work, do_TEEMap);
		if(mma_dev.mma_wq)
			queue_delayed_work(mma_dev.mma_wq, &buf_handle->work,msecs_to_jiffies(2));
	}else{
		buf_handle->tee_map = 1;
	}
	do_gettimeofday(&data->tv3);
	buf_handle->alloc_time = (data->tv3.tv_sec - data->tv1.tv_sec)*1000*1000 +
			(data->tv3.tv_usec - data->tv1.tv_usec);
	if(mma_dev.record_alloc_time) {
		printk(KERN_ERR "%s  %d,buftag=%s,size=0x%x allocate time=%d us,map time=%d us\n",
                __FUNCTION__, __LINE__,data->tag,size,buf_handle->alloc_time,data->map_time_us);
	}
	MMA_DEBUG("tag:%s,size=0x%x ,fd=%d,addr=0x%llx\n", data->tag, size, buf_handle->dmabuf_fd, data->addr);
	kfree(data);
	return 0;
}


struct mma_space_handle* __mma_find_space_handle(const char* space_tag)
{
	struct mma_space_handle* handle;
	list_for_each_entry(handle, &mma_dev.space_list_head, list_node) {
		if(!strncmp(handle->data.space_tag, space_tag, MAX_NAME_SIZE))
			return handle;
	}
	return NULL;
}

struct mma_buf_handle* __mma_find_buf_handle(void *vaddr)
{
	struct mma_buf_handle* handle;
	mutex_lock(&mma_dev.buf_lock);
	list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
		if(handle->kvaddr == vaddr){
			mutex_unlock(&mma_dev.buf_lock);
			return handle;
		}
	}
	mutex_unlock(&mma_dev.buf_lock);
	return NULL;
}


int __mma_reserve_iova_internal(struct mma_reserve_iova_data *data)
{
	int i,ret = 0;
	struct mma_space_handle *handle;

	handle = __mma_find_space_handle(data->space_tag);
	if(handle != NULL) {
		data->base_addr = handle->data.base_addr;
	} else {
		ret = mma_tee_reserve_space(MMA_ADDR_TYPE_IOVA, data->space_tag, data->size, &data->base_addr);
        if( ret < 0) {
			PRINT_AND_RETURN(ret);
        }

		handle = kzalloc(sizeof(*handle), GFP_KERNEL);
		if(handle == NULL){
			mma_tee_free_space(MMA_ADDR_TYPE_IOVA, data->space_tag);
			PRINT_AND_RETURN(-ENOMEM);
		}
		memcpy(&handle->data, data, sizeof(*data));
	    //for(i=0;i<handle->data.buf_tag_num;i++)
            //printk("buf_tag = %s\n", handle->data.buf_tag_array[i]);
		mutex_lock(&mma_dev.buf_lock);
		list_add(&handle->list_node, &mma_dev.space_list_head);
		mutex_unlock(&mma_dev.buf_lock);
	}
	return 0;
}

int __mma_free_iova_internal(const char *space_tag)
{
	int ret = 0;
	struct mma_space_handle *handle;

	handle = __mma_find_space_handle(space_tag);
	CHECK_POINTER(handle, -EINVAL);

	ret = mma_tee_free_space(MMA_ADDR_TYPE_IOVA, space_tag);
    if( ret < 0) {
        PRINT_AND_RETURN(ret);
    }
	mutex_lock(&mma_dev.buf_lock);
	list_del(&handle->list_node);
    mutex_unlock(&mma_dev.buf_lock);
	kfree(handle);
	return 0;
}

void* __mma_kmap_internel(struct dma_buf *db, unsigned long offset, size_t len)
{
    int ret;
    void* vaddr;
	unsigned long page_offset = offset >> PAGE_SHIFT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    ret = dma_buf_begin_cpu_access(db, DMA_BIDIRECTIONAL);
#else
    ret = dma_buf_begin_cpu_access(db, offset, len, DMA_BIDIRECTIONAL);
#endif
    if(ret < 0) {
        return NULL;
    }

    vaddr = dma_buf_kmap(db, page_offset);
    return vaddr;
}

int __mma_kunmap_internel(struct dma_buf *db, void* vaddr, size_t len)
{
    dma_buf_kunmap(db, 0, vaddr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    dma_buf_end_cpu_access(db, DMA_BIDIRECTIONAL);
#else
    dma_buf_end_cpu_access(db, 0, len, DMA_BIDIRECTIONAL);
#endif

    return 0;
}

int __mma_set_cache_flag(int dmabuf_fd, bool cached)
{
    struct dma_buf *db = dma_buf_get(dmabuf_fd);
if (IS_ERR(db))
		return PTR_ERR(db);

    struct mma_buf_handle *handle = db->priv;
    ion_set_buffer_cached(handle->db_ion, cached);
    dma_buf_put(db);
    return 0;
}

int __mma_support(void)
{
	return mma_support;
}

#define MAX_MPU_NR    2
extern unsigned long lx_mem_addr;// = PHYS_OFFSET;
extern unsigned long lx_mem_size;// = INVALID_PHY_ADDR; //default setting


int get_cma_addr(char *name,int miu,unsigned long *start,unsigned long *size)
{
	int i=0;
	if((NULL == name)||(NULL == start)||(NULL == size)){
		printk("%s bad parameters\n",__FUNCTION__);
		return -1;
        }
	for(i = 0; i < MAX_CMA_AREAS;i++)
	{
		if(strstr(cma_config[i].name,name) != NULL
			&&cma_config[i].miu == miu)
		{
			*start = cma_config[i].start;
			*size = cma_config[i].size;
                        //printk("%s name %s start %lx size %lx\n",__FUNCTION__,name,*start,*size);
			return 0;
		}
	}
	printk("not find %s\n",name);
	return -1;
}


typedef struct lx_range_node{
        struct list_head list;
        uint64_t start;
        uint64_t length;
}lx_range_node;
LIST_HEAD(lx_layout);
int lx_add(uint64_t start,uint64_t length,struct list_head *head)
{
        struct list_head *pos;
        lx_range_node *pos_node=NULL,*node =NULL,*pre_node = NULL;

	if(length <= 0||start <= 0)
		return -1;

        pos = head->next;
        while(pos != head){
                pos_node = list_entry(pos,struct lx_range_node,list);
                if(pos_node->start > start){
                        if(pos_node->start < start + length){
                                printk("erro node : pre :0x%llx 0x%llx node 0x%llx 0x%llx\n",
                                        pos_node->start,pos_node->length,start,length);
                                return -1;
                        }
                        if(start + length == pos_node->start){
                                pos_node->start = start;
                                pos_node->length += length;
                                if((pos->prev != head)){
                                        pre_node = list_entry(pos->prev,struct lx_range_node,list);
                                        if(pre_node->start + pre_node->length == pos_node->start)
                                        {
                                                pos_node->start = pre_node->start;
                                                pos_node->length += pre_node->length;
                                                list_del(pos->prev);
                                                kfree(pre_node);
                                        }
                                }
                                return 0;
                        }
                        break;
                }
                pos=pos->next;
        }

        if(pos->prev != head){
                pre_node = list_entry(pos->prev,struct lx_range_node,list);
                if(pre_node->start + pre_node->length == start){
                        pre_node->length += length;
                        return 0;
                }
		if(pre_node->start+pre_node->length >start)
			return -1;
        }

        node = kmalloc(sizeof(*node),GFP_KERNEL);
        if(node == NULL )
                return -1;

        node->start = start;
        node->length = length;
        list_add_tail(&(node->list),pos);

        return 0;
}
int lx_remove(uint64_t start,uint64_t length,struct list_head *head)
{
	struct list_head *pos = NULL,*free_pos=NULL;
	lx_range_node *pos_node = NULL;
	pos = head->next;
	while(pos != head){
		pos_node = list_entry(pos,struct lx_range_node,list);
		if((pos_node->start <= start+length)
		   && (pos_node->start+pos_node->length >= start+length)){
			list_del(pos);
			if(pos_node->start < start )
				lx_add(pos_node->start,start - pos_node->start,head);
			if(pos_node->start+pos_node->length > start+length)
				lx_add(start+length,pos_node->start+pos_node->length-(start+length),head);
			kfree(pos_node);
			return 0;
		}
		pos = pos->next;
	}
	return -1;
}
void dump_lx_list(struct list_head *head)
{
	struct list_head *pos=NULL;
	struct lx_range_node *pos_node=NULL;
	printk("%s begin\n",__FUNCTION__);

	pos = head->next;
	while(pos != head)
	{
		pos_node = list_entry(pos,struct lx_range_node,list);
		printk("node:start 0x%llx , 0x%llx\n",pos_node->start,pos_node->length);
		pos=pos->next;
	}
	printk("%s end\n\n",__FUNCTION__);

}
void free_lx_list(struct list_head *head)
{
	struct list_head *pos=NULL,*free_pos=NULL;
	struct lx_range_node *pos_node=NULL;
	pos = head->next;
	printk("%s begin\n",__FUNCTION__);
	while(pos != head)
	{
		pos_node = list_entry(pos,struct lx_range_node,list);
		printk("del node:start 0x%llx , 0x%llx\n",pos_node->start,pos_node->length);
		free_pos = pos;
		pos=pos->next;
		list_del(free_pos);
		kfree(pos_node);
	}
	printk("%s end\n\n",__FUNCTION__);

}

int __mma_set_mpu_area(void)
{
#ifdef CONFIG_MP_MMA_MPU_SEC
    int i;
    struct mma_range_t mpu_range[MAX_MPU_NR] ={0};
    unsigned long pstart_pfn, pend_pfn;

    for(i=0;i<MAX_MPU_NR;i++) {
        mma_get_mpu_area(i, &pstart_pfn,&pend_pfn);
        if(pstart_pfn>=pend_pfn)
            break;
        mpu_range[i].start = __pfn_to_phys(pstart_pfn);
        mpu_range[i].size = __pfn_to_phys(pend_pfn-pstart_pfn);
        printk("mma_get_mpu_area: i=%d, start=%llx, size=%lld\n", i,mpu_range[i].start, mpu_range[i].size);
    }
	mma_tee_set_mpu_area(MMA_ADDR_TYPE_IOVA, mpu_range, i);
#else
    struct mma_range_t mpu_range[MAX_MPU_NR]={0};
    unsigned long start = 0,size = 0;
    struct list_head *pos = NULL;
    struct lx_range_node *pos_node = NULL;

    if(lx_mem_addr != INVALID_PHY_ADDR)
	lx_add(lx_mem_addr,lx_mem_size,&lx_layout);
    if(lx_mem2_addr != INVALID_PHY_ADDR)
	lx_add(lx_mem2_addr,lx_mem2_size,&lx_layout);
    if(0 == get_cma_addr("OTHERS",0,&start,&size))
    {
		lx_remove(start,size,&lx_layout);
    }
    if(0 == get_cma_addr("OTHERS",1,&start,&size))
    {
		lx_remove(start,size,&lx_layout);
    }
    pos = lx_layout.next;
    if(pos != &lx_layout){
		pos_node = list_entry(pos,struct lx_range_node,list);
        mpu_range[0].start = pos_node->start;
        mpu_range[0].size = pos_node->length;
    }
   pos = pos->next;
   if(pos != &lx_layout){
	pos_node = list_entry(pos,struct lx_range_node,list);
        mpu_range[1].start = pos_node->start;
        mpu_range[1].size = pos_node->length;
    }
	dump_lx_list(&lx_layout);
	free_lx_list(&lx_layout);
#ifdef CONFIG_MMA_MPU_PATCH
	//patch begin for mmap not ready
	mpu_range[0].start = 0x20200000;
	mpu_range[0].size =  0x3000000;
	mpu_range[1].start = 0x23200000;
	mpu_range[1].size =  0x3000000;
	//patch end
#endif
	printk("%llx %llx %llx %llx\n",mpu_range[0].start,mpu_range[0].size,mpu_range[1].start,mpu_range[1].size);
	if(mma_tee_set_mpu_area(MMA_ADDR_TYPE_IOVA, mpu_range, 2)==0){
#ifdef CONFIG_MP_MMA_INT_ENABLE
		_MDrv_IOMMU_RegisterInterrupt();
		printk("enable iommu interrupt!!!!!\n");
#else
		printk("disable iommu interrupt!!!!!\n");
#endif
		mma_support = 1;
	}

#endif

    return 0;
}


static long mma_userdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
    int dir;
    int cleanup_fd = -1;
    char *cleanup_tag = NULL;
	struct mma_userdev_info *info = file->private_data;
	union mma_ioctl_data *data;

    if(arg == 0)
		return  -EINVAL;

	data = kzalloc(sizeof(union mma_ioctl_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	down(&info->sem);

	dir = _IOC_DIR(cmd);

	if (_IOC_SIZE(cmd) > sizeof(union mma_ioctl_data)) {
           ret = -EFAULT;
           goto exit;
       }

	if (dir & _IOC_WRITE) {
		if (copy_from_user(data, (void __user *)arg, _IOC_SIZE(cmd))){
			ret = -EFAULT;
            goto exit;
        }
     }
	switch (cmd) {
		case MMA_IOC_ALLOC:
		{
			ret = __mma_alloc_internal(data->allocation.tag_name, data->allocation.len, data->allocation.bSecure, &data->allocation.addr,
											&data->allocation.dmabuf_fd, MMA_FLAG_IOVA);
            if(ret < 0) {
				ret = -ENOMEM;
                goto exit;
            }
            cleanup_fd = data->allocation.dmabuf_fd;

			break;
		}
		case MMA_IOC_ALLOC_V2:
		{
			ret = __mma_alloc_internal(data->alloc_v2.tag_name, data->alloc_v2.len, data->alloc_v2.bSecure, &data->alloc_v2.addr,
											&data->alloc_v2.dmabuf_fd, data->alloc_v2.flag);
			if(ret < 0) {
				ret = -ENOMEM;
				goto exit;
			}
			cleanup_fd = data->allocation.dmabuf_fd;

			break;
		}
		case MMA_IOC_MAP_IOVA:
		{
			ret = mma_map_iova(&data->map.addr, data->map.dmabuf_fd);
			if(ret < 0) {
				goto exit;
			}
			break;
		}

		case MMA_IOC_RESERVE_IOVA_SPACE:
		{
			ret = __mma_reserve_iova_internal(&data->reservation);
			if(ret < 0) {
				ret = -ENOMEM;
                goto exit;
             }
			break;
		}
		case MMA_IOC_FREE_IOVA_SPACE:
		{
			ret = __mma_free_iova_internal(data->reservation.space_tag);
			if(ret < 0) {
                goto exit;
         }
			break;
		}
		case MMA_IOC_GET_PIPEID:
		{
			ret = mma_get_pipeid(&data->allocation.pipeid);
            if(ret < 0) {
                goto exit;
            }
			break;
		}
		case MMA_IOC_PUT_PIPEID:
		{
			ret = mma_put_pipeid(data->allocation.pipeid);
			if (ret < 0) {
				goto exit;
			}
			break;
		}
		case MMA_IOC_AUTHORIZE:
		{
			ret = mma_buffer_authorize(data->allocation.dmabuf_fd, data->allocation.pipeid);
           if(ret < 0) {
               goto exit;
            }
			break;
		}
		case MMA_IOC_UNAUTHORIZE:
		{
			ret = mma_buffer_unauthorize(data->fd.dmabuf_fd);
			if(ret < 0) {
               goto exit;
			}
			break;
		}

		case MMA_IOC_FREE:
		{
			ret = mma_free(data->fd.dmabuf_fd);
            if(ret < 0) {
                goto exit;
            }
			break;
		}
		case MMA_IOC_SET_CACHE_FLAG:
		{
			ret = __mma_set_cache_flag(data->fd.dmabuf_fd, data->fd.bCached);
            if(ret < 0) {
                goto exit;
            }
			break;
		}
		case MMA_IOC_FLUSH:
		{
			ret = mma_flush((void *)(unsigned long)(data->fd.vaddr), data->fd.len);
            if(ret < 0) {
                goto exit;
            }

			break;
		}
		case MMA_IOC_EXPORT:
		{
			ret = mma_export_globalname(data->fd.dmabuf_fd);
            if(ret < 0) {
                goto exit;
            }

			data->fd.name = ret;
			break;
		}
		case MMA_IOC_IMPORT:
		{
			ret = mma_import_globalname(data->fd.name);
            if(ret < 0) {
                goto exit;
            }

			data->fd.dmabuf_fd = ret;
			break;
		}
		case MMA_IOC_GET_MEMINFO:
		{
			struct mma_meminfo_t meminfo;
			ret = mma_get_meminfo(data->allocation.dmabuf_fd, &meminfo);
            if(ret < 0) {
                goto exit;
            }

            //printk("kernel : meminfo: addr = %llx, size =%d\n", meminfo.addr, meminfo.size);

			data->allocation.addr = meminfo.addr;
			data->allocation.len = meminfo.size;
			data->allocation.bSecure = meminfo.secure;
			data->allocation.miu_select = meminfo.miu_select;
			break;
		}
		case MMA_IOC_GET_HEAPINFO:
		{
			struct mma_heapinfo_t heapinfo;

			ret = mma_get_heapinfo(data->allocation.tag_name, &heapinfo);
            if(ret < 0) {
                goto exit;
            }

			data->allocation.addr = heapinfo.base_addr;
			data->allocation.len = heapinfo.size;
			strncpy(data->allocation.heap_name, heapinfo.name, MAX_NAME_SIZE);
			data->allocation.heap_name[MAX_NAME_SIZE - 1] = '\0';
			break;
		}
        case MMA_IOC_QUERY_BUFTAG:
		{
			ret = mma_query_buf_tag(data->buftag.tag_name, &data->buftag.heaptype,
                                    &data->buftag.miu_number, &data->buftag.max_size);
            if(ret < 0) {
                goto exit;
            }
			break;
		}
		case MMA_IOC_PHYSICAL_AUTHORIZE:
		{
			ret = mma_physical_buffer_authorize(data->allocation.tag_name,data->allocation.addr, data->allocation.len,data->allocation.pipeid);
			if(ret < 0) {
				goto exit;
			}

			break;
		}
		case MMA_IOC_PHYSICAL_UNAUTHORIZE:
		{
			ret = mma_physical_buffer_unauthorize(data->allocation.addr,false);
			if(ret < 0) {
				goto exit;
			}

			break;
		}
		case MMA_IOC_QUERY_PIPELINE_ID:
		{
			ret = mma_pipelineID_query(data->allocation.addr, &data->allocation.pipeid);
			if(ret < 0) {
				goto exit;
			}

			break;
		}
		case MMA_IOC_QUERY_GLOBAL_NAME:
		{
			u64 buf_start;
			ret = mma_globalname_query(data->fd.addr, &data->fd.name, &buf_start);
			if(ret < 0) {
				goto exit;
			}
			data->fd.addr = buf_start;
			break;
		}
		case MMA_IOC_ION_IMPORT:
		{
			ret = mma_import_handle(data->handle.dmabuf_fd, &data->handle.handle);
			if(ret < 0) {
				goto exit;
			}
			break;
		}
		case MMA_IOC_ION_FREE:
		{
			ret = mma_free_handle(data->handle.handle);
			if(ret < 0) {
				goto exit;
			}
			break;
		}
		case MMA_IOC_VA2IOVA:
		{
			ret = mma_va2iova(data->va.tag_name,&data->va.addr, (unsigned long)data->va.vaddr, data->va.len, &data->va.fd);
			if(ret < 0) {
				goto exit;
			}
			cleanup_fd = data->va.fd;

			break;
		}
		case MMA_IOC_RESIZE:
		{
			ret = mma_resize(data->alloc_v2.dmabuf_fd, data->alloc_v2.len, &data->alloc_v2.addr);
			if(ret < 0) {
				goto exit;
			}
			break;
		}
		case MMA_IOC_SUPPORT:
		{
			data->support = mma_support;
			break;
		}
		default:
			ret -EINVAL;
            goto exit;
	}
	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, data, _IOC_SIZE(cmd))) {
			if(cleanup_fd > 0)
				 mma_free(cleanup_fd); //if copy_to_user fail, free the buffer
		if(cleanup_tag != NULL)
			__mma_free_iova_internal(cleanup_tag); //if copy_to_user fail, free the iova space
		printk("copy_to_user failed\n");
		ret = -EFAULT;
        }
     }
exit:
    up(&info->sem);
	kfree(data);
	return ret;
}


int mma_userdev_open(struct inode *inode, struct file *file)
{
	struct mma_userdev_info *info;

	CHECK_POINTER(inode, -ENOMEM);
	CHECK_POINTER(file, -ENOMEM);

	info = kmalloc(sizeof(*info), GFP_KERNEL);

	info->tpid = current->tgid;
	sema_init(&info->sem, 1);
	info->mmap_tpid = 0;
	file->private_data = (void *)info;

	return 0;
}

int mma_userdev_release(struct inode *inode, struct file *file)
{
	struct mma_userdev_info *info = file->private_data;
	down(&info->sem);

	//need add garbage collection


	up(&info->sem);
	file->private_data = NULL;
	kfree(info);

	return 0;
}

static const struct file_operations mma_userdev_fops = {
	.owner = THIS_MODULE,
	.open = mma_userdev_open,
	.release = mma_userdev_release,
	.unlocked_ioctl = mma_userdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mma_userdev_ioctl,
#endif
};
static int mma_tee_init(void);
static int mma_driver_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
		pr_info("%s(%d): PM_POST_SUSPEND\n", __func__, __LINE__);
		mma_tee_close_session(false);
		mma_tee_init();
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block mma_pm_notifer = {
	.notifier_call = mma_driver_event,
};

static int mma_tee_init(void)
{
	mma_tee_open_session();
	__mma_set_mpu_area();
	mma_optee_ta_store_buf_tags();
	return 0;
}
late_initcall(mma_tee_init);

static int __init mma_driver_init(void)
{
	int ret = 0;

	mma_dev.misc_dev.minor = MISC_DYNAMIC_MINOR;
	mma_dev.misc_dev.name  = "mma";
	mma_dev.misc_dev.fops  = &mma_userdev_fops;

	/* register misc device */
	ret = misc_register(&mma_dev.misc_dev);
	if (ret < 0)
		return ret;

	mma_dev.file_ion = NULL;
	mutex_init(&mma_dev.buf_lock);
	INIT_LIST_HEAD(&mma_dev.buf_list_head);
	INIT_LIST_HEAD(&mma_dev.dfree_list_head);
	INIT_LIST_HEAD(&mma_dev.space_list_head);
	INIT_LIST_HEAD(&mma_dev.physical_buf_list_head);
	INIT_LIST_HEAD(&mma_dev.physical_dfree_list_head);

	idr_init(&mma_dev.global_name_idr);

	/*create debugfs*/
	mma_debugfs_init(&mma_dev);

	mma_dev.mma_wq = system_highpri_wq;
	if (mma_dev.mma_wq == NULL)
		printk(KERN_ERR"MMA workqueue fail.\n");
#ifdef CONFIG_PM
	register_pm_notifier(&mma_pm_notifer);
#endif

#ifdef MODULE
    printk("Load mma.ko success.\n");
#endif

	return 0;
}
module_init(mma_driver_init);

static void __exit mma_driver_deinit(void)
{
    mma_ion_close(&mma_dev);
    mma_tee_close_session(true);
	misc_deregister(&mma_dev.misc_dev);
    mma_debugfs_destroy(&mma_dev);
}
module_exit(mma_driver_deinit);

MODULE_DESCRIPTION("Mstar Memory Allocation Framework");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("mstar semiconductor>");
