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


#ifndef _MMA_USER_API_H_
#define _MMA_USER_API_H_

struct mma_meminfo_t {
      __u64 addr; //buffer地址
      __u32 size; //buffer size
      char iova; //是否是iova地址
      char secure; //是否是安全buffer
      char miu_select;//此buffer在哪个miu上
};


struct mma_heapinfo_t {
	__u64 base_addr; //返回此heap的base address。
	__u64 size; //返回此heap的buffer size.
	char name[16]; //返回此heap的name.
};

/*
mma_open
功能说明：open mma
入参说明：无。
出参说明：无
返回值:   0：代表成功
         负数：代表失败，表示不支持mstar memory allocator操作。 注：此返回值可用来实现ip driver 的向前兼容设计，当返回负值时，代表此chip 或kernel不支持mma分配，则fallback 调用原来的cma api分配。
*/
int  mma_open(void);

/*
mma_release
功能说明：释放mma资源
入参说明：无。
出参说明：无
返回值：0：代表成功
         负数：代表失败
*/

int  mma_release(void);

/*
mma_alloc
功能说明：分配普通buffer
入参说明：size： length of memory, in bytes
		buf_tag:  buffer tag,  每个IP可根据buffer的类别，给buffer设定特殊的tag，如”VDEC_ES”。Mmb driver会根据这个tag决定从哪个ion heap分配buffer。对于有带宽分流考量的ip driver，可以使用不同的buf_tag进行miu区分，如”vdec_fb0”, ”vdec_fb1” 来分别代表从哪颗miu分配。
出参说明：dmabuf_fd：为dma_buf fd , 用户可以调用标准的mmap函数map此fd得到virtual address。
返回值：物理地址或IOVA地址
*/
int mma_alloc(const char* buf_tag, __u32 size, __u64* addr_out, int* dmabuf_fd);

/*
mma_alloc_sec
功能说明：分配安全buffer
入参说明：size： length of memory, in bytes
  		buf_tag:  buffer tag,  每个IP可根据buffer的类别，给buffer设定特殊的tag，如”VDEC_ES”。 
出参说明：dmabuf_fd：为dma_buf fd , 用户可以调用标准的mmap函数map此fd得到virtual address。
返回值： 物理地址或IOVA地址。
*/
int mma_alloc_sec(const char* buf_tag, __u32 size, __u64* addr_out, int* dmabuf_fd);

/*
mma_reserve_iova_space
功能说明：某些IP有IOVA 空间范围的限制,例如VDEC要求所有buffer在512M IOVA空间内。 此函数用来设定这些特殊IP的IOVA空间范围。如果IP没有特定的地址范围限制，则不要调用此函数。
入参说明：space_tag：代表地址空间的tag，如“VDEC
          Size：代表范围的大小，如512M，以M为单位。
出参说明：base_addr_out：返回这个space tag地址空间的base addr。如ION TA 决定把2G～2.5G IOVA分配给VDEC，则返回2G
返回值： 0：代表成功
         负数：代表失败
*/
int mma_reserve_iova_space(const char *space_tag, __u64 size, __u64* addr_out, int tag_num, ...);

/*
mma_free_iova_space

功能说明：某些IP有IOVA 空间范围的限制,例如VDEC要求所有buffer在512M IOVA空间内。 此函数用来设定这些特殊IP的IOVA空间范围。如果IP没有特定的地址范围限制，则不要调用此函数。
入参说明：space_tag：代表地址空间的tag，如“VDEC
返回值： 0：代表成功
         负数：代表失败
*/
int mma_free_iova_space(const char *space_tag);


/*
mma_get_pipeid
功能说明：获取pipeid
入参说明：NA
出参说明：NA
返回值： pipeid
*/
int mma_get_pipeid(int *pipeid);

int mma_put_pipeid(int pipeid);
/*
mma_buffer_authorize
功能说明：对分配的安全buffer进行授权，从而相应IP可以访问。
入参说明： dmabuf_fd :调用mma_alloc或mma_alloc_sec所返回的dmabuf_fd。
	pipe_id: tee端根据pipe id和buffer信息决定相应的权限。如果传NULL，则为默认权限。
出参说明：无
返回值： 0：代表成功
                负数：代表失败
*/
int mma_buffer_authorize(int dmabuf_fd, int pipeid);

/*
mma_free
功能说明：释放ion buffer.
入参说明：dmabuf_fd： 调用mma_alloc或mma_alloc_sec所返回的dmabuf_fd。
出参说明：无
返回值：0：代表成功
         负数：代表失败
*/
int  mma_free(int dmabuf_fd);


/*
mma_map
功能说明：获取cpu virtual address.
入参说明：dmabuf_fd： 调用mma_alloc或mma_alloc_sec所返回的dmabuf_fd。
		cached：是否需要cache
		Offset：相对于buffer起始地址的offset
size：需要map的size。
出参说明：无
返回值：NULL：代表失败
         正数：Vritual address。
*/
void* mma_map(int dmabuf_fd, __u8 cached, __u32 offset, __u32 size);

/*
mma_unmap
功能说明：获取cpu virtual address
入参说明：virt_adrr：通过map得到的虚拟地址
size：需要umap的size。
出参说明：无
返回值：0：代表成功
         负数：代表失败
*/
int mma_unmap (void* vadrr, __u32 size);

/*
mma_flush
功能说明：对于map为cached类型buffer，刷D-Cache到内存
入参说明：virt_adrr：通过map得到的虚拟地址
Size：需要flush的size。
出参说明：无
返回值：0：代表成功
         负数：代表失败
*/
int mma_flush(void* vadrr, __u32 size);

/*
mma_export_globalname
功能说明：根据dmabuf_fd export global name
入参说明：dmabuf_fd： 调用mma_alloc或mma_alloc_sec所返回的dmabuf_fd
出参说明：无
返回值： global name
*/
int mma_export_globalname(int dmabuf_fd);

/*
mma_import_globalname
功能说明：根据dmabuf_fd export global name
入参说明：name： global name
出参说明：无
返回值： dmabuf_fd.
*/
int mma_import_globalname(int name);

/*
mma_getinfo
功能说明：根据ion_handle得到此块buffer相关信息.
入参说明：dmabuf_fd： 代表此buffer的fd。	
出参说明：mem_info: buffer 相关信息。
返回值： 0：代表成功
         负数：代表失败
*/
int mma_get_meminfo(int dmabuf_fd, struct mma_meminfo_t *mem_info);

/*
mma_get_heapinfo
功能说明：根据bufTag得到对应heap的base address 和size.
入参说明：buf_tag：每个IP可根据buffer的类别，给buffer设定特殊的tag。
出参说明：heap_base： 返回此heap的base address。
          heap_size: 返回此heap的buffer size.
heap_name: 返回此heap的name.
返回值： 0：代表成功
         负数：代表失败
*/
int mma_get_heapinfo(const char* buf_tag, struct mma_heapinfo_t* heap_info);

/*
功能说明：check mma 是否支持此buf_tag
入参说明：buf_tag：每个IP可根据buffer的类别，给buffer设定特殊的tag。

出参说明：max_size: 此buf_tag 单次分配最大的buffer
          heap_type:  HEAP_TYPE_IOMMU = 0, HEAP_TYPE_CMA= 1,  HEAP_TYPE_NON_MMA = 2
          miu_number: UMA_OR_MIU0 = 0, MIU_1 = 1

返回值： 0：代表支持此buf_tag
         负数：代表失败,表示不支持此buf_tag.

*/
int mma_query_buf_tag(const char* buf_tag, __u32* heap_type, __u32* miu_number, __u32* max_size);

/*
mma_buffer_unauthorize
功能说明：对分配的安全buffer进行取消授权。
入参说明： dmabuf_fd :调用mma_alloc或mma_alloc_sec所返回的dmabuf_fd。

出参说明：无
返回值： 0：代表成功
                负数：代表失败
*/

int mma_buffer_unauthorize(int dmabuf_fd);

/*
mma_cma_buffer_authorize
功能说明：对指定address进行授权，从而相应IP可以访问，只适用于CMA，不需要调用mma_alloc
入参说明： buf_tag：每个IP可根据buffer的类别，给buffer设定特殊的tag。
           addr： 需要授权的address
           size： length of memory, in bytes
	       pipe_id: tee端根据pipe id和buffer信息决定相应的权限。如果传NULL，则为默认权限。

出参说明：无
返回值： 0：代表成功
         负数：代表失败
*/
int mma_physical_buffer_authorize(const char* buf_tag,__u64 addr, __u32 size,__u32 pipe_id);

/*
mma_cma_buffer_unauthorize
功能说明：对buffer进行取消授权，只适用CMA
入参说明：addr： 需要取消授权的address

出参说明：无
返回值： 0：代表成功
         负数：代表失败
*/
int mma_physical_buffer_unauthorize(__u64 addr);

/*
功能说明：查询address对应的pipelineID
入参说明：addr：需要查询的address

出参说明：u32pipelineID: address对应的pipelineID
返回值： 0：代表支持此buf_tag
         负数：代表失败,表示不支持此buf_tag.
*/
int mma_pipelineID_query(__u64 addr, __u32 * u32pipelineID);

int mma_import_handle(int fd, int* handle);

int mma_free_handle(int handle);
int	mma_alloc_v2(const char* buf_tag, unsigned int size, unsigned long long* addr_out, int* dmabuf_fd,int flag);
int mma_resize(int fd, unsigned int newsize, unsigned long long* addr_out);
int mma_support(int *status);

#endif
