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


#ifndef _MMA_KERNEL_API_H_
#define _MMA_KERNEL_API_H_

struct mma_meminfo_t {
      __u64 addr; //buffer��ַ
      __u32 size; //buffer size
      bool iova; //�Ƿ���iova��ַ
      bool secure; //�Ƿ��ǰ�ȫbuffer
      char miu_select;//��buffer���ĸ�miu��
};


struct mma_heapinfo_t {
	__u64 base_addr; //���ش�heap��base address��
	__u64 size; //���ش�heap��buffer size.
	char name[16]; //���ش�heap��name.
};

/*
mma_open
����˵����open mma
���˵�����ޡ�
����˵������
����ֵ:   0�������ɹ�
         ����������ʧ�ܣ���ʾ��֧��mstar memory allocator������ ע���˷���ֵ������ʵ��ip driver ����ǰ������ƣ������ظ�ֵʱ��������chip ��kernel��֧��mma���䣬��fallback ����ԭ����cma api���䡣
*/
int  mma_open(void);

/*
mma_release
����˵�����ͷ�mma��Դ
���˵�����ޡ�
����˵������
����ֵ��0�������ɹ�
         ����������ʧ��
*/

int  mma_release(void);

/*
mma_alloc
����˵����������ͨbuffer
���˵����buf_tag:  buffer tag,  ÿ��IP�ɸ���buffer����𣬸�buffer�趨�����tag����"VDEC_ES"��Mma driver��������tag�������ĸ�ion heap����buffer�������д�������������ip driver������ʹ�ò�ͬ��buf_tag����miu���֣���"vdec_fb0", "vdec_fb1" ���ֱ�������Ŀ�miu���䡣
		  size�� length of memory, in bytes
����˵����addr_out���ѷ���buffer��������ַ��IOVA��ַ��
          dmabuf_fd��Ϊdma_buf fd , �û����Ե��ñ�׼��mmap����map��fd�õ�virtual address��
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_alloc(const char* buf_tag, u32 size, u64* addr_out, int* dmabuf_fd);

/*
mma_alloc_sec
����˵�������䰲ȫbuffer
���˵����buf_tag:  buffer tag,  ÿ��IP�ɸ���buffer����𣬸�buffer�趨�����tag����"VDEC_ES"��Mma driver��������tag�������ĸ�ion heap����buffer�������д�������������ip driver������ʹ�ò�ͬ��buf_tag����miu���֣���"vdec_fb0", "vdec_fb1" ���ֱ�������Ŀ�miu���䡣
		  size�� length of memory, in bytes
����˵����addr_out���ѷ���buffer��������ַ��IOVA��ַ��
          dmabuf_fd��Ϊdma_buf fd , �û����Ե��ñ�׼��mmap����map��fd�õ�virtual address��
����ֵ�� 0�������ɹ�
         ����������ʧ��

*/
int mma_alloc_sec(const char* buf_tag, u32 size, u64* addr_out, int* dmabuf_fd);

/*
mma_reserve_iova_space
����˵����ĳЩIP��IOVA �ռ䷶Χ������,����VDECҪ������buffer��512M IOVA�ռ��ڡ� �˺��������趨��Щ����IP��IOVA�ռ䷶Χ�����IPû���ض��ĵ�ַ��Χ���ƣ���Ҫ���ô˺�����
���˵����space_tag��������ַ�ռ��tag���硰VDEC
          Size��������Χ�Ĵ�С����512M����MΪ��λ��
����˵����base_addr_out���������space tag��ַ�ռ��base addr����ION TA ������2G��2.5G IOVA�����VDEC���򷵻�2G
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_reserve_iova_space(const char *space_tag, u64 size, u64* addr_out, int tag_num, ...);

/*
mma_free_iova_space

����˵����ĳЩIP��IOVA �ռ䷶Χ������,����VDECҪ������buffer��512M IOVA�ռ��ڡ� �˺��������趨��Щ����IP��IOVA�ռ䷶Χ�����IPû���ض��ĵ�ַ��Χ���ƣ���Ҫ���ô˺�����
���˵����space_tag��������ַ�ռ��tag���硰VDEC
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_free_iova_space(const char *space_tag);


/*
mma_get_pipeid
����˵������ȡpipeid
���˵�����ޡ�
����ֵ��pipeid
*/
int mma_get_pipeid(int *pipeid);

int mma_put_pipeid(int pipeid);

/*
mma_buffer_authorize
����˵�����Է���İ�ȫbuffer������Ȩ���Ӷ���ӦIP���Է��ʡ�
���˵���� dmabuf_fd :����mma_alloc��mma_alloc_sec�����ص�dmabuf_fd��
	pipe_id: tee�˸���pipe id��buffer��Ϣ������Ӧ��Ȩ�ޡ������NULL����ΪĬ��Ȩ�ޡ�
                   
����˵������
����ֵ�� 0�������ɹ�
                ����������ʧ��
*/

int mma_buffer_authorize(int dmabuf_fd, u32 pipe_id);

/*
mma_buffer_unauthorize
����˵�����Է���İ�ȫbuffer����ȡ����Ȩ��
���˵���� dmabuf_fd :����mma_alloc��mma_alloc_sec�����ص�dmabuf_fd��
                   
����˵������
����ֵ�� 0�������ɹ�
                ����������ʧ��
*/

int mma_buffer_unauthorize(int dmabuf_fd);


/*
mma_free
����˵�����ͷ�ion buffer.
���˵����dmabuf_fd�� ����mma_alloc��mma_alloc_sec�����ص�dmabuf_fd��
����˵������
����ֵ��0�������ɹ�
         ����������ʧ��
*/
int  mma_free(int dmabuf_fd);


/*
mma_map
����˵������ȡcpu virtual address.
���˵����dmabuf_fd�� ����mma_alloc��mma_alloc_sec�����ص�dmabuf_fd��
		cached���Ƿ���Ҫcache
		Offset�������buffer��ʼ��ַ��offset
size����Ҫmap��size��
����˵������
����ֵ��NULL������ʧ��
         ������Vritual address��
*/
void* mma_map(int dmabuf_fd, bool cached, u32 offset, u32 size);

/*
mma_unmap
����˵������ȡcpu virtual address
���˵����virt_adrr��ͨ��map�õ��������ַ
size����Ҫumap��size��
����˵������
����ֵ��0�������ɹ�
         ����������ʧ��
*/
int mma_unmap (void* vadrr, u32 size);

/*
mma_flush
����˵��������mapΪcached����buffer��ˢD-Cache���ڴ�
���˵����virt_adrr��ͨ��map�õ��������ַ
Size����Ҫflush��size��
����˵������
����ֵ��0�������ɹ�
         ����������ʧ��
*/
int mma_flush(void* vadrr, u32 size);

/*
mma_export_globalname
����˵��������dmabuf_fd export global name
���˵����dmabuf_fd�� ����mma_alloc��mma_alloc_sec�����ص�dmabuf_fd
����˵������
����ֵ�� global name
*/
int mma_export_globalname(int dmabuf_fd);

/*
mma_import_globalname
����˵��������dmabuf_fd export global name
���˵����name�� global name
����˵������
����ֵ�� dmabuf_fd.
*/
int mma_import_globalname(int name);

/*
mma_getinfo
����˵��������ion_handle�õ��˿�buffer�����Ϣ.
���˵����dmabuf_fd�� ������buffer��fd��	
����˵����mem_info: buffer �����Ϣ��
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_get_meminfo(int dmabuf_fd, struct mma_meminfo_t *mem_info);

/*
mma_get_heapinfo
����˵��������bufTag�õ���Ӧheap��base address ��size.
���˵����buf_tag��ÿ��IP�ɸ���buffer����𣬸�buffer�趨�����tag��
����˵����heap_base�� ���ش�heap��base address��
          heap_size: ���ش�heap��buffer size.
heap_name: ���ش�heap��name.
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_get_heapinfo(const char* buf_tag, struct mma_heapinfo_t* heap_info);

/*
����˵����check mma �Ƿ�֧�ִ�buf_tag
���˵����buf_tag��ÿ��IP�ɸ���buffer����𣬸�buffer�趨�����tag��

����˵����max_size: ��buf_tag ���η�������buffer
          heap_type:  HEAP_TYPE_IOMMU = 0, HEAP_TYPE_CMA= 1,  HEAP_TYPE_NON_MMA = 2
          miu_number: UMA_OR_MIU0 = 0, MIU_1 = 1
����ֵ�� 0������֧�ִ�buf_tag
         ����������ʧ��,��ʾ��֧�ִ�buf_tag.

*/
int mma_query_buf_tag(const char* buf_tag, u32* heap_type, u32* miu_number, u32* max_size);

/*
mma_physical_buffer_authorize
����˵������ָ��address������Ȩ���Ӷ���ӦIP���Է��ʣ�ֻ������physical address������Ҫ����mma_alloc
���˵���� buf_tag��ÿ��IP�ɸ���buffer����𣬸�buffer�趨�����tag��
           addr�� ��Ҫ��Ȩ��address
           size�� length of memory, in bytes
	       pipe_id: tee�˸���pipe id��buffer��Ϣ������Ӧ��Ȩ�ޡ������NULL����ΪĬ��Ȩ�ޡ�

����˵������
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_physical_buffer_authorize(const char* buf_tag,u64 addr, u32 size,u32 pipe_id);

/*
mma_physical_buffer_unauthorize
����˵������buffer����ȡ����Ȩ��ֻ����physical address
���˵����addr�� ��Ҫȡ����Ȩ��address
          force�� �Ƿ�Ҫǿ��ȡ����������authorize count
����˵������
����ֵ�� 0�������ɹ�
         ����������ʧ��
*/
int mma_physical_buffer_unauthorize(u64 addr,bool force);

/*
����˵������ѯaddress��Ӧ��pipelineID
���˵����addr����Ҫ��ѯ��address

����˵����u32pipelineID: address��Ӧ��pipelineID
����ֵ�� 0��������ȡ�ɹ�
         ����������ʧ��
*/
int mma_pipelineID_query(u64 addr, u32 * u32pipelineID);

/*
����˵������ѯaddress��Ӧ��globalname
���˵����addr����Ҫ��ѯ��address

����˵����name: address��Ӧ��globalname
          buf_start: address��Ӧ�ķ���ʱ����ʼ��ַ
����ֵ�� 0��������ȡ�ɹ�
         ����������ʧ��.
*/
int mma_globalname_query(u64 addr, u32* name, u64 *buf_start);
int	mma_alloc_v2(const char* buf_tag, u32 size, u64* addr_out, int* dmabuf_fd, int flag);
int	mma_map_iova(u64* addr_out, int dmabuf_fd);
void mma_dmabuf_put(struct dma_buf *db);
int	mma_va2iova(char* buf_tag, u64* addr_out, unsigned long va, u32 size, int* dmabuf_fd);
int mma_import_handle(int fd, int* handle);
int mma_free_handle(int handle);
//-------------------------------------------------------------------------------------------------
/// mma_iommu_support
/// check iommu support
/// @return 1 : support
///         0 : not support
//-------------------------------------------------------------------------------------------------
int mma_iommu_support(void);

//-------------------------------------------------------------------------------------------------
/// mma_resize
/// resize mma buffer
/// @param  fd        \b IN: buffer fd
/// @param  newsize   \b IN: new buffer size
/// @param  addr      \b out: buffer address
/// @return 0    : succee
///        other : error
//-------------------------------------------------------------------------------------------------
int mma_resize(int fd, unsigned int newsize, u64 *addr);

#endif