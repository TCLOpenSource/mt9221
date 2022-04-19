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

#include <linux/module.h>
#include <asm/timex.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "mma_api.h"

#define CHECK_POINTER(pointer,result)      \
        if (IS_ERR_OR_NULL(pointer))        \
        {\
            printk("%s  %d,pointer is NULL or Error, pointer = %ld!\n",__FUNCTION__, __LINE__, PTR_ERR(pointer));\
            return result;\
        } \

#define CHECK_RETURN(result)      \
        if (result < 0)   \
        {\
            printk("%s  %d, Failed, result = %d!\n",__FUNCTION__, __LINE__, result);\
            return result;\
        }\


#define PRINT printk

int test_alloc(int size)
{
    int ret, dmabuf_fd;
    u64 addr;

    ret = mma_alloc("vdec_es", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_free(dmabuf_fd);

    CHECK_RETURN(ret);
    return 0;
}

int test_conti_alloc(int size)
{
    int ret = 0;
    int dmabuf_fd1, dmabuf_fd2;
    u64 addr;

    PRINT("Starting continuous allocate memory test...\n");
    ret = mma_alloc("vdec_es", size, &addr, &dmabuf_fd1);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd1);

    ret = mma_alloc("vdec_fb", size, &addr, &dmabuf_fd2);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd2);

    ret = mma_free(dmabuf_fd1);
    CHECK_RETURN(ret);

    ret = mma_free(dmabuf_fd2);
    CHECK_RETURN(ret);

    return 0;
}

int test_buftag(void)
{
    int ret;
    __u32 max_size;
    __u32 heaptype;
    __u32 miu;
    ret = mma_query_buf_tag("xc_main", &heaptype, &miu, &max_size);
    PRINT("ret = %d,heaptype =%d, miu=%d, max_size =%d\n", ret, heaptype, miu, max_size);

    ret = mma_query_buf_tag("vdec_fb", &heaptype, &miu, &max_size);
    PRINT("ret = %d,heaptype =%d, miu=%d, max_size =%d\n", ret, heaptype, miu, max_size);

    return 0;
}



int test_alloc_sec(int size)
{
    int ret, dmabuf_fd, pipeid;
    u64 addr;

    ret = mma_alloc_sec("vdec_es", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_get_pipeid(&pipeid);
    PRINT("pipeid =%d\n", pipeid);

    ret = mma_buffer_authorize(dmabuf_fd, pipeid);
    CHECK_RETURN(ret);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}


int test_alloc_size(void)
{
    int ret, dmabuf_fd, i;
    u64 addr;
    static int allocationSizes[] = {1024, 4*1024, 64*1024, 1024*1024, 4*1024*1024, 64*1024*1024};
    for (i=0;i<(sizeof(allocationSizes)/sizeof(int)); i++) {
        PRINT("test_alloc_size: size =%d\n", allocationSizes[i]);
        ret = test_alloc(allocationSizes[i]);
        CHECK_RETURN(ret);
    }

    return 0;
}

int test_illegal_alloc(void)
{
    int ret, dmabuf_fd, i;
    u64 addr;
	PRINT("test_illegal_alloc\n");

    //test 0 size
    ret = mma_alloc("vdec_es", 0, &addr, &dmabuf_fd);
    PRINT("test 0 size: ret =%d\n", ret);

    //test non-exist buf_tag
    ret = mma_alloc("TEST", 1024*1024, &addr, &dmabuf_fd);
    PRINT("test non-exist buf_tag: ret =%d\n", ret);

    //test null addr
    ret = mma_alloc("vdec_es", 1024*1024, NULL,NULL);
    PRINT("test null addr: ret =%d\n", ret);


    ret = mma_alloc("vdec_es", 1024*1024, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    //test illegal dmabuf_fd
    ret = mma_free(0);
    PRINT("test illegal dmabuf_fd: ret =%d\n", ret);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);

    //test duplicate free
    ret = mma_free(dmabuf_fd);
    PRINT("test duplicate free: ret =%d\n", ret);
    return 0;
}


int test_map(int size)
{
    int ret, dmabuf_fd;
    u64 addr;
    void *vaddr = NULL;

    ret = mma_alloc("vdec_es", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    vaddr = mma_map( dmabuf_fd, true, 0, size);
    if(vaddr == NULL) {
        mma_free(dmabuf_fd);
        PRINT("%s  %d,pointer is NULL or Error!\n",__FUNCTION__, __LINE__);
        return -1;
    }
    PRINT("vaddr = %p\n", vaddr);
    memset(vaddr, 0x10, size);

    void *tmp_addr = kzalloc(size, GFP_KERNEL);
    if(tmp_addr == NULL) {
        mma_unmap(vaddr, size);
        mma_free(dmabuf_fd);
        PRINT("%s  %d,pointer is NULL or Error!\n",__FUNCTION__, __LINE__);
        return -1;
    }

    memset(tmp_addr, 0x10, size);
    PRINT("tmp_addr = %p\n", tmp_addr);

    ret = memcmp(vaddr, tmp_addr, size);
    if(ret == 0)
        PRINT("memcmp sucess\n");
    else
        PRINT("memcmp failed\n");

    kfree(tmp_addr);
    ret = mma_unmap(vaddr, size);
    CHECK_RETURN(ret);
    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}


int test_map_cache(int size, u8 cached)
{
    int ret, dmabuf_fd;
    u64 addr;

    ret = mma_alloc("vdec_es", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    void *vaddr = mma_map( dmabuf_fd, cached, 0, size);
    if (!vaddr)
        return NULL;

    PRINT("vaddr = %p\n", vaddr);
    memset(vaddr,0x55,size);

    mma_unmap(vaddr, size);
    mma_free(dmabuf_fd);
    return 0;
}

int test_reserve_iova(unsigned int size)
{
    int ret, dmabuf_fd;
    u64 addr, base;

    ret = mma_reserve_iova_space("vdec", size, &base, 2, "vdec_es", "vdec_fb");
    CHECK_RETURN(ret);
    PRINT("base = %llx, size =%d\n", base, size);

    ret = mma_alloc("vdec_es", 1024*1024, &addr, &dmabuf_fd);
    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);

    ret = mma_free_iova_space("vdec");
    CHECK_RETURN(ret);

    return 0;
}

int test_export_import(int size)
{
    int ret, dmabuf_fd,import_fd, globalname;
    u64 addr;
    struct mma_meminfo_t mem_info;

    ret = mma_alloc("vdec_es", size, &addr, &dmabuf_fd);
    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_get_meminfo(dmabuf_fd,&mem_info);
    CHECK_RETURN(ret);
    PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);

    globalname = mma_export_globalname(dmabuf_fd);
    PRINT("globalname =%d\n", globalname);

    import_fd = mma_import_globalname(globalname);
    PRINT("import_fd =%d\n",import_fd);

    ret = mma_get_meminfo(import_fd,&mem_info);
    CHECK_RETURN(ret);
    PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);

    //ret = mma_free(import_fd);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);

    return 0;
}


static int __init mma_test_init(void)
{
    int i;
    PRINT(KERN_ALERT "mma testing\n");
    mma_open();
    test_buftag();

    //test_alloc(1024*1024);
    //test_reserve_iova(512*1024*1024);

    //test_alloc_sec(60*1024*1024);

    //test_conti_alloc(1024*1024);

    //for(i =0;i<1000;i++)
        //test_alloc_size();

    //test_map(8*1024);

    //test_export_import(1024*1024);

    //test_map_cache(4096, 1);
    //test_map_cache(4096, 0);

    msleep(1000*30);
    mma_release();

    return -1; /* Fail will directly unload the module */
}

static void __exit mma_test_exit(void)
{
	PRINT(KERN_ALERT "test exit\n");
}

module_init(mma_test_init)
module_exit(mma_test_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("richard.du");
MODULE_DESCRIPTION("mma test");

