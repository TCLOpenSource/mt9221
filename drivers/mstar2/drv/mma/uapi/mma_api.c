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

#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <cutils/log.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdarg.h>

#include <sys/types.h>

#include <pthread.h>

#include "mma_ioctl.h"
#include "mma_api.h"


#define CHECK_POINTER(pointer,result)      \
        if(NULL == pointer)        \
        {\
            printf("%s  %d,pointer is NULL or Error\n",__FUNCTION__, __LINE__);\
            return result;\
        } \

#define CHECK_RETURN(result)      \
        if (result < 0)   \
        {\
            printf("%s  %d, Failed, result = %d!\n",__FUNCTION__, __LINE__, result);\
            return result;\
        }\


static int mma_fd = -1;
static int reference_count = 0;
static pthread_mutex_t mma_mutex = PTHREAD_MUTEX_INITIALIZER;


static void __mma_check_open(void)
{
	pthread_mutex_lock(&mma_mutex);
	if (-1 == mma_fd) {
		mma_fd = open(MMA_DEV_NAME, O_RDWR);
	}
	pthread_mutex_unlock(&mma_mutex);
}

int  mma_open(void)
{
	pthread_mutex_lock(&mma_mutex);
    if (-1 == mma_fd) {
		mma_fd = open(MMA_DEV_NAME, O_RDWR);
		if(mma_fd < 0) {
		    printf("open mma dev failed\n");
			return -ENODEV;
		}
    }
    reference_count ++;
	pthread_mutex_unlock(&mma_mutex);
	return 0;
}

int  mma_release(void)
{
	pthread_mutex_lock(&mma_mutex);
    if(--reference_count == 0) {
        printf("mma_release\n");
        if(mma_fd > 0)
            close(mma_fd);
    }
	pthread_mutex_unlock(&mma_mutex);
    return 0;
}

static int __mma_alloc_internal(const char* buf_tag, __u32 size, __u8 secure, __u64* addr_out, int* dmabuf_fd)
{
    int ret;
    struct mma_alloc_data data;

	__mma_check_open();
	CHECK_POINTER(buf_tag, -EINVAL);
	CHECK_POINTER(dmabuf_fd, -EINVAL);

	data.len = size;
    strncpy(data.tag_name, buf_tag, MAX_NAME_SIZE);
    data.tag_name[MAX_NAME_SIZE - 1] = '\0';
    data.bSecure = secure;

    ret = ioctl(mma_fd, MMA_IOC_ALLOC, &data);
	CHECK_RETURN(ret);

    *dmabuf_fd = data.dmabuf_fd;
	*addr_out = data.addr;
    return 0;
}

int	mma_alloc(const char* buf_tag, __u32 size, __u64* addr_out, int* dmabuf_fd)
{
	return __mma_alloc_internal(buf_tag, size, 0, addr_out, dmabuf_fd);
}

int	mma_alloc_sec(const char* buf_tag, __u32 size, __u64* addr_out, int* dmabuf_fd)
{
	return __mma_alloc_internal(buf_tag, size, 1, addr_out, dmabuf_fd);
}

int mma_reserve_iova_space(const char *space_tag, __u64 size, __u64* addr_out, int tag_num,  ...)
{
    int ret;
	va_list ap;
	int i;
	char *temp;
    struct mma_reserve_iova_data data;
	CHECK_POINTER(space_tag, -EINVAL);
	if(tag_num > MAX_TAG_NUM)
        return -EINVAL;

	//fill data
	strncpy(data.space_tag, space_tag, MAX_NAME_SIZE);
	data.space_tag[MAX_NAME_SIZE - 1] = '\0';
	data.size = size;
	data.buf_tag_num = tag_num;

	va_start(ap, tag_num);
	for (i = 0; i < tag_num; i++) {
		temp = va_arg(ap, char*);
		strncpy(data.buf_tag_array[i], temp, MAX_NAME_SIZE);
		data.buf_tag_array[i][MAX_NAME_SIZE - 1] = '\0';
	}
	va_end(ap);

	__mma_check_open();

    ret = ioctl(mma_fd, MMA_IOC_RESERVE_IOVA_SPACE, &data);
	CHECK_RETURN(ret);
	*addr_out = data.base_addr;
	return 0;
}

int mma_free_iova_space(const char *space_tag)
{
    struct mma_reserve_iova_data data;

	CHECK_POINTER(space_tag, -EINVAL);
	__mma_check_open();

    strncpy(data.space_tag, space_tag, MAX_NAME_SIZE);
    data.space_tag[MAX_NAME_SIZE - 1] = '\0';

    return ioctl(mma_fd, MMA_IOC_FREE_IOVA_SPACE, &data);
}

int mma_get_pipeid(int *pipeid)
{
    int ret, id;
    
    struct mma_alloc_data data;
	__mma_check_open();

    ret = ioctl(mma_fd, MMA_IOC_GET_PIPEID, &data);
	CHECK_RETURN(ret);

    *pipeid = data.pipeid;
    return 0;
}

int mma_put_pipeid(int pipeid) {
    int ret;
    struct mma_alloc_data data;

    __mma_check_open();

    data.pipeid = pipeid;
    ret = ioctl(mma_fd, MMA_IOC_PUT_PIPEID, &data);
    CHECK_RETURN(ret);

    return 0;
}

int mma_buffer_authorize(int dmabuf_fd, int pipeid)
{
    struct mma_alloc_data data;

	__mma_check_open();

	data.dmabuf_fd = dmabuf_fd;
    data.pipeid = pipeid;

    return ioctl(mma_fd, MMA_IOC_AUTHORIZE, &data);
}

int  mma_buffer_unauthorize(int dmabuf_fd)
{
    struct mma_fd_data data;

	__mma_check_open();
	data.dmabuf_fd = dmabuf_fd;

    return ioctl(mma_fd, MMA_IOC_UNAUTHORIZE, &data);
}

int  mma_free(int dmabuf_fd)
{
    struct mma_fd_data data;
    int ret;

    if((ret = mma_buffer_unauthorize(dmabuf_fd)) != 0)
        return ret;
	__mma_check_open();
	data.dmabuf_fd = dmabuf_fd;

    return ioctl(mma_fd, MMA_IOC_FREE, &data);
}

void* mma_map(int dmabuf_fd, __u8 cached, __u32 offset, __u32 size)
{
	int ret;
    struct mma_fd_data data;

	__mma_check_open();

	data.dmabuf_fd = dmabuf_fd;
	data.bCached = cached;

	ret = ioctl(mma_fd, MMA_IOC_SET_CACHE_FLAG, &data);
	if (ret < 0)
		return NULL;
    
	void* addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dmabuf_fd, offset);
	if(addr == MAP_FAILED){
		printf(" %s  %d, Failed, :%s\n", __FUNCTION__, __LINE__,strerror(errno));
		return NULL;
	}

	return (void*)addr;
}

int mma_flush(void* vadrr, __u32 size)
{
	struct mma_fd_data data;

	__mma_check_open();
	data.vaddr = (__u64)vadrr;
	data.len = size;

	return ioctl(mma_fd, MMA_IOC_FLUSH, &data);
}

int mma_unmap (void* vadrr, __u32 size)
{
    return munmap(vadrr, size);
}

int mma_export_globalname(int dmabuf_fd)
{
    int ret;
    struct mma_fd_data data;

	__mma_check_open();
	data.dmabuf_fd= dmabuf_fd;

    ret = ioctl(mma_fd, MMA_IOC_EXPORT, &data);
	CHECK_RETURN(ret);

	return data.name;
}

int mma_import_globalname(int name)
{
    int ret;
    struct mma_fd_data data;

	__mma_check_open();
	data.name = name;

    ret = ioctl(mma_fd, MMA_IOC_IMPORT, &data);
	CHECK_RETURN(ret);

	return data.dmabuf_fd;
}

int mma_get_meminfo(int dmabuf_fd, struct mma_meminfo_t *mem_info)
{
    int ret;
    struct mma_alloc_data data;

	__mma_check_open();
	data.dmabuf_fd = dmabuf_fd;

    ret = ioctl(mma_fd, MMA_IOC_GET_MEMINFO, &data);
	CHECK_RETURN(ret);

    printf("mma_get_meminfo: addr = %llx, size =%d\n", data.addr, data.len);  
	mem_info->addr = data.addr;
	mem_info->size= data.len;
	mem_info->secure = data.bSecure;
    return 0;
}

int mma_get_heapinfo(const char* buf_tag, struct mma_heapinfo_t* heap_info)
{
    int ret;
    struct mma_alloc_data data;

	__mma_check_open();
    strncpy(data.tag_name, buf_tag, MAX_NAME_SIZE);
    data.tag_name[MAX_NAME_SIZE - 1] = '\0';

    ret = ioctl(mma_fd, MMA_IOC_GET_HEAPINFO, &data);
	CHECK_RETURN(ret);

    strncpy(heap_info->name, data.heap_name, MAX_NAME_SIZE);
    heap_info->name[MAX_NAME_SIZE - 1] = '\0';
	heap_info->base_addr = data.addr;
	heap_info->size = data.len;
    return 0;
}

int mma_query_buf_tag(const char* buf_tag, __u32* heap_type, __u32* miu_number, __u32* max_size)
{
    int ret;
    struct mma_buftag_data data;

    __mma_check_open();
    strncpy(data.tag_name, buf_tag, MAX_NAME_SIZE);
    data.tag_name[MAX_NAME_SIZE - 1] = '\0';

    ret = ioctl(mma_fd, MMA_IOC_QUERY_BUFTAG, &data);
	CHECK_RETURN(ret);

    *heap_type = data.heaptype;
    *miu_number = data.miu_number;
    *max_size = data.max_size;
    return 0;
}
int mma_physical_buffer_authorize(const char* buf_tag,__u64 addr, __u32 size,__u32 pipeid)
{
    struct mma_alloc_data data;

    __mma_check_open();

    data.addr = addr;
    data.len = size;
    data.pipeid = pipeid;
    strncpy(data.tag_name, buf_tag, MAX_NAME_SIZE);
    data.tag_name[MAX_NAME_SIZE - 1] = '\0';

    return ioctl(mma_fd, MMA_IOC_PHYSICAL_AUTHORIZE, &data);
}

int mma_physical_buffer_unauthorize(__u64 addr)
{
    struct mma_alloc_data data;

    __mma_check_open();

    data.addr = addr;

    return ioctl(mma_fd, MMA_IOC_PHYSICAL_UNAUTHORIZE, &data);
}
int mma_pipelineID_query(__u64 addr, __u32* u32pipelineID)
{
    int ret;
    struct mma_alloc_data data;

    __mma_check_open();

    data.addr = addr;

    ret = ioctl(mma_fd, MMA_IOC_QUERY_PIPELINE_ID, &data);
    CHECK_RETURN(ret);

    *u32pipelineID = data.pipeid;
    return ret;
}

int mma_import_handle(int fd, int* handle) {
    int ret = -1;
    struct mma_ion_handle_data data = {0};

    __mma_check_open();
    data.dmabuf_fd = fd;

    ret = ioctl(mma_fd, MMA_IOC_ION_IMPORT, &data);
    CHECK_RETURN(ret);

    *handle = data.handle;
    return 0;
}

int mma_free_handle(int handle) {
    int ret = -1;
    struct mma_ion_handle_data data = {0};

    __mma_check_open();
    data.handle= handle;

    ret = ioctl(mma_fd, MMA_IOC_ION_FREE, &data);
    CHECK_RETURN(ret);

    return 0;
}

int	mma_alloc_v2(const char* buf_tag, unsigned int size, unsigned long long* addr_out, int* dmabuf_fd,int flag)
{
	int ret;
	struct mma_alloc_data_v2 data;

	__mma_check_open();
	CHECK_POINTER(buf_tag, -EINVAL);
	CHECK_POINTER(dmabuf_fd, -EINVAL);

	data.len = size;
	strncpy(data.tag_name, buf_tag, MAX_NAME_SIZE);
	data.bSecure = 0;
	data.flag = flag;

	ret = ioctl(mma_fd, MMA_IOC_ALLOC_V2, &data);
	CHECK_RETURN(ret);

	*dmabuf_fd = data.dmabuf_fd;
	*addr_out = data.addr;
	return 0;
}

int mma_resize(int fd, unsigned int newsize, unsigned long long* addr_out)
{
	int ret;
	struct mma_alloc_data_v2 data;

	__mma_check_open();
	CHECK_POINTER(addr_out, -EINVAL);
	memset(&data, 0, sizeof(data));

	data.len = newsize;
	data.dmabuf_fd = fd;

	ret = ioctl(mma_fd, MMA_IOC_RESIZE, &data);
	CHECK_RETURN(ret);

	*addr_out = data.addr;
	return 0;
}

int mma_support(int *status)
{
	int ret;
	int support = 0;
	CHECK_POINTER(status, -EINVAL);
	ret = ioctl(mma_fd, MMA_IOC_SUPPORT, &support);
	CHECK_RETURN(ret);
	*status = support;
	return 0;
}
