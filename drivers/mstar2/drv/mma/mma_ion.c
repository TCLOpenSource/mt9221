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

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ion.h>
#include <linux/mm.h>
#include <linux/syscalls.h>

#include "mma_core.h"
#include "mma_of.h"
#include "mma_ion.h"
#ifdef CONFIG_MP_MMA_CMA_ENABLE
extern int ion_mma_cma_miu0_heap_id;
extern int ion_mma_cma_miu1_heap_id;
extern int ion_mma_cma_miu2_heap_id;
#endif
#ifdef CONFIG_MP_MMA_UMA_WITH_NARROW
extern u64 mma_dma_zone_size;
#endif

#ifdef CONFIG_MP_IOMMU_DISCRETE_CMA
extern int iommu_discrete_cma_miu0_heap_id;
extern int iommu_discrete_cma_miu1_heap_id;
extern int iommu_discrete_cma_miu2_heap_id;
#endif

#define MMA_FLUSH_SIZE 0x3200000

int mma_ion_open(struct mma_device* mma_dev)
{
    if(mma_dev->file_ion == NULL) {
        struct file* filep = filp_open(ION_DEV_NAME, O_RDWR, 0);
        if (IS_ERR(filep)) {
            printk("mma_ion_open failed, err %ld", PTR_ERR(filep));
            mma_dev->file_ion = NULL;
            goto error;
        }
        mma_dev->file_ion = filep;
    }
    return 0;

error:
    return -ENODEV;
}

int mma_ion_close(struct mma_device* mma_dev)
{
    if(mma_dev->file_ion){
        int ret = filp_close(mma_dev->file_ion, NULL);
        if (ret < 0)
            goto error;

        mma_dev->file_ion = NULL;
    }
    return 0;

error:
    return -EBUSY;
}

static int mma_ion_ioctl(struct mma_device* mma_dev, uint req, void *arg)
{
    int ret;
    mm_segment_t old_fs;

    if(mma_dev->file_ion == NULL) {
        ret = mma_ion_open(mma_dev);
        CHECK_RETURN(ret);
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    ret = file_ioctl(mma_dev->file_ion, req, (unsigned long )arg);
    set_fs(old_fs);
    return ret;
}

int mma_ion_alloc(struct mma_device* mma_dev, size_t size, unsigned int heap_mask,
                  int flag, int *dmabuf_fd)
{
    int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,12,0)
    struct ion_allocation_data *alloc_data;

    CHECK_POINTER(dmabuf_fd, -EINVAL)

	alloc_data = kzalloc(sizeof(struct ion_allocation_data), GFP_KERNEL);
	CHECK_POINTER(alloc_data, -ENOMEM)

    alloc_data->len = size;
    alloc_data->heap_id_mask = heap_mask;
    alloc_data->flags = flag;
    ret = mma_ion_ioctl(mma_dev, ION_IOC_ALLOC, (void *)alloc_data);
	*dmabuf_fd = alloc_data->fd;
	kfree(alloc_data);
    CHECK_RETURN(ret);

    if (*dmabuf_fd < 0)
        PRINT_AND_RETURN(-EINVAL);

#else
    struct ion_allocation_data alloc_data = {0};
    struct ion_fd_data fd_data = {0};
    struct ion_handle_data handle_data = {0};

    CHECK_POINTER(dmabuf_fd, -EINVAL)

    alloc_data.len = size;
    alloc_data.align = MMA_ALIGN;
    alloc_data.heap_id_mask = heap_mask;
    alloc_data.flags = flag;
    ret = mma_ion_ioctl(mma_dev, ION_IOC_ALLOC, &alloc_data);
    CHECK_RETURN(ret);


    fd_data.handle = alloc_data.handle;
    handle_data.handle = alloc_data.handle;

    ret = mma_ion_ioctl(mma_dev, ION_IOC_SHARE, &fd_data);
    CHECK_RETURN(ret);

    if (fd_data.fd < 0) {
        mma_ion_ioctl(mma_dev, ION_IOC_FREE, &handle_data);
        PRINT_AND_RETURN(-EINVAL);
    }
    *dmabuf_fd = fd_data.fd;

    ret = mma_ion_ioctl(mma_dev, ION_IOC_FREE, &handle_data); //dec reference count
    CHECK_RETURN(ret);
#endif

    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
int mma_ion_import(int fd, ion_user_handle_t* handle) {
    struct dma_buf* dmabuf = dma_buf_get(fd);
    struct mma_buf_handle* buf_handle = NULL;
    struct dma_buf* db_ion = NULL;
    struct ion_fd_data fd_data = {0};
    int ret = -1, db_ion_fd = -1;

    if(IS_ERR_OR_NULL(dmabuf))
        return -1;

    buf_handle = (struct mma_buf_handle*) dmabuf->priv;
    db_ion = buf_handle->db_ion;
    db_ion_fd = dma_buf_fd(db_ion, O_CLOEXEC);
    db_ion = dma_buf_get(db_ion_fd);
    fd_data.fd = db_ion_fd;
    ret = mma_ion_ioctl(&mma_dev, ION_IOC_IMPORT, &fd_data);
    CHECK_RETURN(ret);

    *handle = fd_data.handle;
    buf_handle->tpid = current->tgid;
    dma_buf_put(dmabuf);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
    ksys_close(db_ion_fd);
#else
    sys_close(db_ion_fd);
#endif
    return 0;
}

int mma_ion_free(ion_user_handle_t handle) {
    int ret = -1;
    struct ion_handle_data handle_data = {0};
    handle_data.handle = handle;

    ret = mma_ion_ioctl(&mma_dev, ION_IOC_FREE, &handle_data);
    CHECK_RETURN(ret);

    return 0;
}
#endif

int mma_ion_query_heap(struct mma_device* mma_dev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,12,0)
    
        
    
#endif
    return 0;
}


int mma_ion_get_id_flag(int heap_type, int miu,int zone_flag, bool secure, int* heap_mask,
		int* ion_flag, u32 size, char* buf_tag)
{
    int flag = 0;
    if(miu > 2)
        return -EINVAL;
    //heap_type = HEAP_TYPE_CMA;
    if(heap_type == HEAP_TYPE_IOMMU) {
        *heap_mask = 1 << ION_HEAP_TYPE_SYSTEM;//ION_HEAP_SYSTEM_MASK;
        if(miu == 0)
            flag = ION_FLAG_MIU0;
        else if(miu == 1)
            flag = ION_FLAG_MIU1;
        else
            flag = ION_FLAG_MIU2;
    } else if (heap_type == HEAP_TYPE_CMA) {
#ifdef CONFIG_MP_MMA_CMA_ENABLE
        if(miu == 0 && ion_mma_cma_miu0_heap_id) {
            *heap_mask = 1<<ion_mma_cma_miu0_heap_id;
        } else if (miu == 1 && ion_mma_cma_miu1_heap_id) {
            *heap_mask = 1<<ion_mma_cma_miu1_heap_id;
        } else if (miu == 2 && ion_mma_cma_miu2_heap_id){
            *heap_mask = 1<<ion_mma_cma_miu2_heap_id;
        }else{
            printk("mma_ion_get_id_flag miu=%d,miu0 heap id=%d,miu1 heap id=%d,miu2 heap id=%d \n",
                miu,ion_mma_cma_miu0_heap_id,ion_mma_cma_miu1_heap_id,ion_mma_cma_miu2_heap_id);
            return -EINVAL;
        }
        flag = ION_FLAG_CONTIGUOUS;
#else
        printk("MMA CMA not support\n");
        return -EINVAL;
#endif
    } else if (heap_type == HEAP_TYPE_CMA_IOMMU) {
#ifdef CONFIG_MP_IOMMU_DISCRETE_CMA
        if(miu == 0 && iommu_discrete_cma_miu0_heap_id) {
            *heap_mask = 1<<iommu_discrete_cma_miu0_heap_id;
        } else if (miu == 1 && iommu_discrete_cma_miu1_heap_id) {
            *heap_mask = 1<<iommu_discrete_cma_miu1_heap_id;
        } else if (miu == 2 && iommu_discrete_cma_miu2_heap_id){
            *heap_mask = 1<<iommu_discrete_cma_miu2_heap_id;
        }else{
            printk(KERN_ERR"mma_ion_get_id_flag miu=%d,miu0 heap id=%d,miu1 heap id=%d,miu2 heap id=%d \n",
                miu,iommu_discrete_cma_miu0_heap_id,iommu_discrete_cma_miu1_heap_id,iommu_discrete_cma_miu2_heap_id);
            printk(KERN_ERR"IOMMU buftag=%s, check dts,CMA and MMAP !!!\n",buf_tag);
            *heap_mask = 1 << ION_HEAP_TYPE_SYSTEM;//try ion system heap
        }
        flag = ION_FLAG_IOMMU_CMA;
#else
        printk(KERN_ERR"IOMMU DISCRETE CMA not support !!!\n");
        printk(KERN_ERR"IOMMU buftag=%s, check dts and config !!!\n",buf_tag);
        *heap_mask = 1 << ION_HEAP_TYPE_SYSTEM;//try ion system heap
#endif
    }else if(heap_type == HEAP_TYPE_CARVEOUT){
		*heap_mask = 1 << ION_HEAP_TYPE_IOMMU_CARVEOUT;
	}else
		return -EINVAL;

#ifdef CONFIG_MP_MMA_UMA_WITH_NARROW
    if(!zone_flag && mma_dma_zone_size > 0)
        flag |= ION_FLAG_DMAZONE;
#endif

    if(secure)
        flag |= ION_FLAG_SECURE;

	if(strstr(buf_tag,"mali") == NULL)
		flag |= ION_FLAG_IOMMU_FLUSH;

    *ion_flag = flag;
    return 0;
}
