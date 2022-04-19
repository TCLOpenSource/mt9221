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

#include <stdio.h>
#include <string.h>
#include <utils/Log.h>
#include <malloc.h>

#include "mma_api.h"

#include <drvSYS.h>
#include <MsCommon.h>
#include <MsOS.h>
#include "drvBDMA.h"


#define PRINT printf

#define CHECK_RETURN(result)      \
        if (result < 0)   \
        {\
            PRINT("%s  %d, Failed, result = %d!\n",__FUNCTION__, __LINE__, result);\
            return result;\
        }\

#define MMA_IOVA_START  0x200000000
#define MMA_FLAG_IOVA (1<< 0) //buffer need IOVA, 1:yes, 0:no
#define MMA_FLAG_DMA (1<< 1)  //buffer allocate from dma zone, 1:yes, 0:no
#define MMA_FLAG_CACHE (1<< 2) //buffer 1:cache ,0 :uncache
#define MMA_FLAG_2XIOVA  (1<< 3) //1:mapping double size iova

int test_alloc(__u32 size)
{
    int ret, dmabuf_fd;
    __u64 addr;

#if 1
    //cma
    ret = mma_alloc("VDEC_ES", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);
    
    ret = MDrv_BDMA_PatternFill(addr, size, 0x55555555, E_BDMA_DEV_MIU0);
    if(E_BDMA_OK!=ret) {
        PRINT("MDrv_BDMA_PatternFill Failed, result = %d!\n", ret);
    }
    sleep(10);
    #else
    void *vaddr = mma_map( dmabuf_fd, 0, 0, size);
    if(vaddr!=NULL) {
        memset(vaddr, 0x55, size);
        PRINT("vaddr = %p\n", vaddr);
    }
    mma_flush(vaddr, size);
    sleep(10);

    mma_unmap(vaddr, size);
    #endif


    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}

int test_resize(void)
{
    int ret, dmabuf_fd = -1;
    unsigned long long addr = 0;
    int flag = 0;
    void *vaddr = NULL ;
    int status = 0;
    unsigned int size = 15 * 1024 * 1024;
    unsigned int newsize = 16 * 1024 * 1024;
    struct mma_meminfo_t mem_info;

    ret = mma_support(&status);
    CHECK_RETURN(ret);
    if(status != 1 ){
        PRINT("not support IOMMU\n");
        return -1;
    }
    ret = mma_alloc_v2("vdec_es", size, &addr, &dmabuf_fd,flag);
    CHECK_RETURN(ret);

    PRINT("mma_alloc addr = %llx, dmabuf_fd =%d,size=0x%x\n", addr, dmabuf_fd,size);

    ret = mma_resize(dmabuf_fd, newsize, &addr);
    CHECK_RETURN(ret);

    //addr == 0,because no MMA_FLAG_IOVA flag
    PRINT("mma_resize addr = %llx, dmabuf_fd =%d,newsize=0x%x\n", addr, dmabuf_fd,newsize);

    memset(&mem_info, 0, sizeof(mem_info));
    ret = mma_get_meminfo(dmabuf_fd, &mem_info);
    CHECK_RETURN(ret);
    PRINT("mem_info addr = %llx,size=0x%x\n", mem_info.addr, mem_info.size);
    if(mem_info.size != newsize || mem_info.addr < MMA_IOVA_START){
        return -1;
    }

    vaddr = mma_map(dmabuf_fd, 0, 0, newsize);
    PRINT("vaddr = 0x%lx\n", (unsigned long)vaddr);
    if(vaddr){
        memset(vaddr,0,newsize);
        mma_unmap(vaddr, newsize);
    }else
        return -1;
    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    PRINT("free success\n");
    sleep(1);

    dmabuf_fd = -1;
    flag = MMA_FLAG_IOVA;
    ret = mma_alloc_v2("vdec_es", size, &addr, &dmabuf_fd,flag);
    CHECK_RETURN(ret);

    PRINT("mma_alloc addr = %llx, dmabuf_fd =%d,size=0x%x\n", addr, dmabuf_fd,size);

    ret = mma_resize(dmabuf_fd, newsize, &addr);
    CHECK_RETURN(ret);

    PRINT("mma_resize addr = %llx, dmabuf_fd =%d,newsize=0x%x\n", addr, dmabuf_fd,newsize);

    memset(&mem_info, 0, sizeof(mem_info));
    ret = mma_get_meminfo(dmabuf_fd, &mem_info);
    CHECK_RETURN(ret);

    PRINT("mem_info addr = %llx,size=0x%x\n", mem_info.addr, mem_info.size);
    if(mem_info.size != newsize || mem_info.addr < MMA_IOVA_START){
        return -1;
    }

    vaddr = mma_map(dmabuf_fd, 0, 0, newsize);
    PRINT("vaddr = %p\n", vaddr);

    PRINT("addr = %llx, dmabuf_fd =%d,size=0x%x\n", addr, dmabuf_fd,size);
    if(vaddr){
        memset(vaddr,0,newsize);
        mma_unmap(vaddr, newsize);
    }else
        return -1;

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    PRINT("free success\n");
    sleep(1);

    flag = MMA_FLAG_IOVA | MMA_FLAG_2XIOVA;
    ret = mma_alloc_v2("vdec_es", size, &addr, &dmabuf_fd,flag);
    CHECK_RETURN(ret);

    PRINT("mma_alloc addr = %llx, dmabuf_fd =%d,size=0x%x\n", addr, dmabuf_fd,size);

    ret = mma_resize(dmabuf_fd, newsize, &addr);
    CHECK_RETURN(ret);

    PRINT("mma_resize addr = %llx, dmabuf_fd =%d,newsize=0x%x\n", addr, dmabuf_fd,newsize);

    memset(&mem_info, 0, sizeof(mem_info));
    ret = mma_get_meminfo(dmabuf_fd, &mem_info);
    CHECK_RETURN(ret);

    PRINT("mem_info addr = %llx,size=0x%x\n", mem_info.addr, mem_info.size);
    if(mem_info.size != newsize || mem_info.addr < MMA_IOVA_START){
        return -1;
    }

    vaddr = mma_map(dmabuf_fd, 0, 0, newsize);
    PRINT("vaddr = %p\n", vaddr);

    if(vaddr){
        memset(vaddr,0,newsize);
        mma_unmap(vaddr, newsize);
    }else
        return -1;

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    PRINT("free success\n");
    sleep(1);

    dmabuf_fd = -1;
    flag = MMA_FLAG_2XIOVA;
    ret = mma_alloc_v2("vdec_es", size, &addr, &dmabuf_fd,flag);
    CHECK_RETURN(ret);

    PRINT("mma_alloc addr = %llx, dmabuf_fd =%d,size=0x%x\n", addr, dmabuf_fd,size);

    ret = mma_resize(dmabuf_fd, newsize, &addr);
    CHECK_RETURN(ret);

    //addr == 0,because no MMA_FLAG_IOVA flag
    PRINT("mma_resize addr = %llx, dmabuf_fd =%d,newsize=0x%x\n", addr, dmabuf_fd,newsize);

    memset(&mem_info, 0, sizeof(mem_info));
    ret = mma_get_meminfo(dmabuf_fd, &mem_info);
    CHECK_RETURN(ret);

    PRINT("mem_info addr = %llx,size=0x%x\n", mem_info.addr, mem_info.size);
    if(mem_info.size != newsize || mem_info.addr < MMA_IOVA_START){
        return -1;
    }

    vaddr = mma_map(dmabuf_fd, 0, 0, newsize);
    PRINT("vaddr = %p\n", vaddr);

    if(vaddr){
        memset(vaddr,0,newsize);
        mma_unmap(vaddr, newsize);
    }else
        return -1;

    ret = mma_unmap(vaddr, newsize);
    PRINT("sleep\n");
    sleep(1000);
    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}

int test_bdma_fill(int size)
{
    int ret, dmabuf_fd;
    __u64 addr;

    ret = mma_alloc("VDEC_ES", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);
    
    ret = MDrv_BDMA_PatternFill(addr, size, 0x55555555, E_BDMA_DEV_MIU0);
    if(E_BDMA_OK!=ret) {
        PRINT("MDrv_BDMA_PatternFill Failed, result = %d!\n", ret);
    }

    void *vaddr = mma_map( dmabuf_fd, 0, 0, size);

    PRINT("vaddr = %p\n", vaddr);
    if(vaddr!=NULL) {

        void *tmp_addr = malloc(size);
        if(tmp_addr!=NULL) {
            memset(tmp_addr, 0x55, size);
            PRINT("tmp_addr = %p\n", tmp_addr);
        }
        ret = memcmp(vaddr, tmp_addr, size);
        if(ret == 0)
            PRINT("memcmp sucess\n");
        else
            PRINT("memcmp failed\n");

        free(tmp_addr);
    }
    ret = mma_unmap(vaddr, size);
    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}

int test_alloc_sec(int size)
{
    int ret, dmabuf_fd, pipeid;
    __u64 addr;

    ret = mma_alloc_sec("VDEC_ES", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    //ret = mma_get_pipeid(&pipeid);
    //PRINT("pipeid =%d\n", pipeid);

    //ret = mma_buffer_authorize(dmabuf_fd, pipeid);
   // CHECK_RETURN(ret);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}

int test_multi_process_alloc()
{
    pid_t pid;
    char *message;
    int n;

    printf("test_multi_process pid =%d\n", getpid());
    pid = fork();
    switch(pid)
    {
    case -1:
        perror("fork failed");
       // exit(1);
    case 0:
        message = "This is the child";
        n = 5;
        break;
    default:
        message = "This is the parent";
        n = 3;
        break;
    }

    for(; n > 0; n--) {
        puts(message);
        test_alloc(1024*1024);
        //teest_pipeid();
        sleep(1);
    }
   // exit(0);
   return 0;
}

int test_alloc_size(void)
{
    int ret, dmabuf_fd, i;
    __u64 addr;
    static int allocationSizes[] = {1024, 4*1024, 64*1024, 1024*1024, 4*1024*1024, 32*1024*1024};
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
    __u64 addr;
	PRINT("test_illegal_alloc\n");

    //test 0 size
    ret = mma_alloc("VDEC_ES", 0, &addr, &dmabuf_fd);
    PRINT("test 0 size: ret =%d\n", ret);

    //test non-exist buf_tag
    ret = mma_alloc("TEST", 1024*1024, &addr, &dmabuf_fd);
    PRINT("test non-exist buf_tag: ret =%d\n", ret);

    //test null addr
    ret = mma_alloc("VDEC_ES", 1024*1024, NULL,NULL);
    PRINT("test null addr: ret =%d\n", ret);


    ret = mma_alloc("VDEC_ES", 1024*1024, &addr, &dmabuf_fd);
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
    __u64 addr;

    ret = mma_alloc("VDEC_ES", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    void *vaddr = mma_map( dmabuf_fd, 0, 0, size);

    PRINT("vaddr = %p\n", vaddr);
    if(vaddr!=NULL) {
        memset(vaddr, 0x10, size);

        void *tmp_addr = malloc(size);
        if(tmp_addr!=NULL) {
            memset(tmp_addr, 0x10, size);
            PRINT("tmp_addr = %p\n", tmp_addr);
        }
        ret = memcmp(vaddr, tmp_addr, size);
        if(ret == 0)
            PRINT("memcmp sucess\n");
        else
            PRINT("memcmp failed\n");

        free(tmp_addr);
    }
    ret = mma_unmap(vaddr, size);
    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
    return 0;
}


int test_map_cache(int size, __u8 cached)
{
    int ret, dmabuf_fd;
    __u64 addr;

    ret = mma_alloc("VDEC_ES", size, &addr, &dmabuf_fd);
    CHECK_RETURN(ret);

    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    void *vaddr = mma_map( dmabuf_fd, cached, 0, size);

    PRINT("vaddr = %p\n", vaddr);
    memset(vaddr,0x55,size);

    sleep(30);
    return 0;
}

int test_reserve_iova(unsigned int size)
{
    int ret, dmabuf_fd;
    __u64 addr, base;

    ret = mma_reserve_iova_space("vdec", size, &base, 2, "VDEC_ES", "VDEC_FB");
   CHECK_RETURN(ret);
   PRINT("base = %llx, size =%lld\n", base, size);

     ret = mma_reserve_iova_space("xc", size, &base, 1, "XC_FB1");
    CHECK_RETURN(ret);
    PRINT("base = %llx, size =%lld\n", base, size);

    ret = mma_alloc("VDEC_ES", 1024*1024, &addr, &dmabuf_fd);
    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);

    ret = mma_alloc("XC_FB1", 1024*1024, &addr, &dmabuf_fd);
    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);

        ret = mma_alloc("VDEC_FB0", 1024*1024, &addr, &dmabuf_fd);
    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);

    ret = mma_free_iova_space("vdec");
    CHECK_RETURN(ret);

        ret = mma_free_iova_space("xc");
    CHECK_RETURN(ret);
    return 0;
}

int test_export_import(__u32 size)
{
    int ret, dmabuf_fd, globalname;
    __u64 addr;
    struct mma_meminfo_t mem_info;
    memset(&mem_info, 0, sizeof(mem_info));

    ret = mma_alloc("VDEC_ES", size, &addr, &dmabuf_fd);
    PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

    ret = mma_get_meminfo(dmabuf_fd,&mem_info);
    CHECK_RETURN(ret);
    PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);  

    globalname = mma_export_globalname(dmabuf_fd);
    PRINT("globalname =%d\n", globalname);

    int import_fd = mma_import_globalname(globalname);
    PRINT("import_fd =%d\n",import_fd);

    memset(&mem_info, 0, sizeof(mem_info));

    ret = mma_get_meminfo(import_fd,&mem_info);
    CHECK_RETURN(ret);
    PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);

    //ret = mma_free(import_fd);
    
    ret = mma_free(dmabuf_fd);
    CHECK_RETURN(ret);
  
    
    return 0;
}

int teest_pipeid()
{
    int pipeid;
    mma_get_pipeid(&pipeid);
    PRINT("mma_get_pipeid: id = %d\n",pipeid);
    return 0;
}


/*
int test_buffer_transfer()
{
    pid_t pid;
    char *message;
    int ret, dmabuf_fd;
    __u64 addr;
    int name;
    char globaname[3];
   struct mma_meminfo_t mem_info;
    printf("test_buffer_transfer pid =%d\n", getpid());

    pid = fork();
    switch(pid)
    {
    case -1:
        perror("fork failed");
        exit(1);
    case 0:
        PRINT("This is the child\n");
        sleep(10);
        property_get("mma_globaname", globaname, "0");
        name = atoi(globaname);
        PRINT("import globaname =%d\n", name);
        dmabuf_fd = mma_import_globalname(name);
        CHECK_RETURN(dmabuf_fd);
        PRINT("import dmabuf_fd =%d\n", dmabuf_fd);

        ret = mma_get_meminfo(dmabuf_fd,&mem_info);
        CHECK_RETURN(ret);
        PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);

        close(dmabuf_fd);
        break;
    default:
        PRINT("This is the parent\n");
        ret = mma_alloc("VDEC_ES", 1024*1024, &addr, &dmabuf_fd);
        CHECK_RETURN(ret);
        PRINT("addr = %llx, dmabuf_fd =%d\n", addr, dmabuf_fd);

        ret = mma_get_meminfo(dmabuf_fd,&mem_info);
        CHECK_RETURN(ret);
        PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);

        name = mma_export_globalname(dmabuf_fd);
        PRINT("globalname =%d\n", name);

        snprintf(globaname, 3, "%d", name);
        property_set("mma_globaname", globaname);
        sleep(30);
        mma_free(dmabuf_fd);
        break;
        }

    exit(0);
    return 0;
}

int test_ion_buffer(int size)
{
    int ion_fd, dmabuf_fd;
    struct mma_meminfo_t mem_info;

    ion_fd = ion_open();

    ion_alloc_fd(ion_fd,size, 0, ION_HEAP_SYSTEM_MASK, 0, &dmabuf_fd);
    PRINT("dmabuf_fd =%d\n", dmabuf_fd);

    mma_get_meminfo(dmabuf_fd, &mem_info);
    PRINT("meminfo: addr = %llx, size =%d\n", mem_info.addr, mem_info.size);

    close(dmabuf_fd);
    return 0;
}
*/
void bdma_init()
{
    MDrv_SYS_GlobalInit();
    MsOS_MPool_Init();
    MsOS_Init();
    MDrv_BDMA_Init(0x80000000UL);
}

int main(int argc, const char* argv[])
{
    struct mma_heapinfo_t heap_info;
    int i;

    bdma_init();
    mma_open();

    //mma_get_heapinfo("VDEC_ES", &heap_info);
    //PRINT("heap_info: name=%s, addr = %llx, size =%d\n", heap_info.name, heap_info.base_addr, heap_info.size);
    //test_reserve_iova(512*1024*1024);

    //test_alloc_sec(50*1024*1024);
    
    test_bdma_fill(12*1024);

    //test_map(1024*1024);

    //test_export_import(1024*1024);

    //test_multi_process_alloc();
    //test_buffer_transfer();
    //test_alloc_sec(64*1024*1024);

    //for(i=0;i<100;i++)
    //test_alloc_size();

    //test_illegal_alloc();
    //test_map_cache(4096, 1);
    //test_map_cache(4096, 0);
    //test_map(1024*1024);
    //test_buffer_transfer();
    //test_ion_buffer(1024*1024);

    sleep(5);
    mma_release();
	return -1; /* Fail will directly unload the module */
}
