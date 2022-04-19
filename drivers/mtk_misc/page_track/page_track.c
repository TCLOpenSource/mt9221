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
#if defined(CONFIG_PROC_FS) && defined(CONFIG_STACKTRACE)
#define pr_fmt(fmt) "page_track: " fmt
//#define DEBUG 1
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/slub_def.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <trace/events/kmem.h>
#include <linux/fdtable.h>
#include <linux/stat.h>
#include <linux/mount.h>
#include <linux/module.h>
#include <linux/radix-tree.h>
#include <linux/memblock.h>
#include <linux/ioport.h>
#include <linux/timekeeping.h>
#include <linux/bitops.h>
#include <linux/version.h>
#if BITS_PER_LONG<64
#include <asm/sections.h>
#endif
#include <linux/vmalloc.h>
#include <linux/kdebug.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>

//for 64bit,in each stacktrace entry,sym in module is 28bit,in kernel is up to kernel text size
#define PAGE_TRACK_STACK_SIZE ((4*28)+BITS_PER_BYTE-1)/BITS_PER_BYTE
#define PAGE_TRACK_STACK_MAX_DEPTH (10)
#define TRACK_COUNT_PER_PAGE (3)
#define MAX_MODULE_SIZE (MODULES_END-MODULES_VADDR)

typedef int (*page_pfn_cb)(unsigned long pfn,unsigned char data);
struct ram_res
{
	const char *name;
	unsigned long start_pfn;
	unsigned long end_pfn;
};
struct _page_track_info
{
	unsigned char enc_stack[PAGE_TRACK_STACK_SIZE];
	unsigned long when;//unit is 0.1ms,max circle is 119 hrs on arm32
	unsigned long data;	//addr or order
};
struct kmem_cache_data
{
	unsigned long addr;
	unsigned int pid;
};
struct kmem_page_data
{
	unsigned int order;
	unsigned int index;
	unsigned int pid;
};
struct page_track_info
{
	struct _page_track_info track_info[TRACK_COUNT_PER_PAGE];
	spinlock_t lock;
};

enum
{
	KMEMC_ALLOC,
	KMEMC_FREE,
	KMEM_KFREE,
	KMEMC_ALLOC_NODE,
	PAGE_ALLOC,
	PAGE_FREE,
};
static struct kmem_cache *page_track_cache;
static char b_page_track=0;
static RADIX_TREE(page_track_tree,GFP_KERNEL);
static DEFINE_RWLOCK(page_track_tree_lock);
static unsigned long mem_start_pfn=0;
static unsigned char enc_stack_mod_bitlen=0;
static unsigned char enc_stack_kern_bitlen=0;
static void * delete_page_track_slot(unsigned long pfn,unsigned char block);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,14,0)
static __init int _match_res_name(struct resource *input_res,struct ram_res *res)
#else
static __init int _match_res_name(u64 start,u64 end,struct ram_res *res)
#endif
{
#if LINUX_VERSION_CODE >KERNEL_VERSION(4,14,0)	
	u64 start=input_res->start;
#endif
	struct resource *iomem_res=lookup_resource(&iomem_resource,start);	
	struct resource *p;
		
	if(!iomem_res)return 0;
	pr_debug("%s:%llx-%llx\n",iomem_res->name,iomem_res->start,iomem_res->end);
	if(!strcmp(iomem_res->name,"System RAM"))
	{
		p=iomem_res->child;
		if(!p)return 0;
		for(;p;p=p->sibling)
		{
			if(!strcmp(res->name,p->name))
			{
				pr_debug("%s:%llx-%llx\n",p->name,p->start,p->end);
				res->start_pfn=PHYS_PFN(p->start);
				res->end_pfn=PHYS_PFN(p->end);
				return 1;
			}
		}
   }
	return 0;
}
static __init int _get_kernel_phys(struct ram_res *res)
{
	int ret;
	struct memblock_region *reg;
	ret=walk_iomem_res_desc(IORES_DESC_NONE,0,memblock_start_of_DRAM(), memblock_end_of_DRAM(),res,
				(void *)_match_res_name);
	if(ret)
	{
		for_each_memblock(reserved, reg) 
		{
			unsigned long start_pfn=memblock_region_reserved_base_pfn(reg);
			unsigned long end_pfn=memblock_region_reserved_end_pfn(reg)-1;
			pr_debug("Reserved %lx~%lx\n",start_pfn,end_pfn);
			if((res->start_pfn>=start_pfn) && (res->start_pfn<end_pfn) && (res->end_pfn>end_pfn))
			{
				pr_debug("change end_pfn of %s from %lx to %lx\n",res->name,res->end_pfn,end_pfn);
				res->end_pfn=end_pfn;
			}	
		}
	}
	return ret;
}
static int _init_pfn_slot(unsigned long pfn,unsigned char block)
{
	int ret;
	unsigned long flags;
	struct page_track_info *info;

	info=kmem_cache_alloc(page_track_cache, GFP_KERNEL);
	if(!info)
	{
		pr_err("can not allocate mem for page_track_info!\n");
		return -ENOMEM;
	}
	spin_lock_init(&info->lock);
	if(block)
		write_lock_irqsave(&page_track_tree_lock,flags);
	ret=radix_tree_insert(&page_track_tree,pfn-mem_start_pfn,info);
	if(block)
		write_unlock_irqrestore(&page_track_tree_lock,flags); 
	if(ret)
	{
		kfree(info);
		pr_err("radix_tree_insert fails:%d\n",ret);
		return ret;
	}

	return 0;
}

static __init void _init_enc_stack_bitlen(void)
{
	unsigned int order_width=fls_long(MAX_ORDER-1);
	unsigned int index_width=MAX_ORDER;
	
	
	pr_debug("order_width=%d,index_width=%d,max_order=%d\n",order_width,index_width,MAX_ORDER);
	enc_stack_mod_bitlen=fls_long(MAX_MODULE_SIZE-1)+1;
	enc_stack_kern_bitlen=fls_long((unsigned long)_etext-
	           (unsigned long)_text-1)+1;
	pr_debug("bitlen for mod sym=%d,for kernel=%d\n",enc_stack_mod_bitlen,
			enc_stack_kern_bitlen);
}

static __init int _cb_on_each_pfn(page_pfn_cb pfn_cb,unsigned char data)
{
    struct ram_res	kernel_code={"Kernel code",0,0};
    struct ram_res	kernel_data={"Kernel data",0,0};
	int ret=0;
	struct memblock_region *reg;
	
	
	(void)_get_kernel_phys(&kernel_code);
	(void)_get_kernel_phys(&kernel_data);
	
    pr_alert("exclude %s:%lx-%lx\n",kernel_code.name,kernel_code.start_pfn,kernel_code.end_pfn);
    pr_alert("exclude %s:%lx-%lx\n",kernel_data.name,kernel_data.start_pfn,kernel_data.end_pfn);
	
	//get the kernel code and data
	
	for_each_memblock(memory, reg) {
		unsigned long start = memblock_region_memory_base_pfn(reg);
		unsigned long end = memblock_region_memory_end_pfn(reg);
	    unsigned long pfn;

		if (start >= end)
			continue;
		
		for(pfn=start;pfn<end;pfn++)
		{
			if(pfn==kernel_code.start_pfn)
			{
				pfn=kernel_code.end_pfn;
				continue;
			}
			else if(pfn==kernel_data.start_pfn)
			{
				pfn=kernel_data.end_pfn;
				continue;
			}
			ret=pfn_cb(pfn,data);
		}
	}
	pr_debug("%s done...\n",__func__);
	return ret;	
}
static __init int init_radix_slot(void)
{
	return _cb_on_each_pfn(_init_pfn_slot,0);
}
#if !defined(CONFIG_MODULES)
static __init int exclude_pfn_slot(unsigned long pfn,unsigned char block)
{	
	struct page *head_page=compound_head(pfn_to_page(pfn));
	if(PageSlab(head_page) && (head_page->slab_cache==page_track_cache))
	{
		if(delete_page_track_slot(pfn,block))
		{
			pr_debug("delete pfn %lx-%s\n",pfn,head_page->slab_cache->name);
		}	
	}
	return 0;
}
static __init int exclude_radix_slot(void)
{
	pr_debug("shrink page_track_cache=%d\n",kmem_cache_shrink(page_track_cache));
	return _cb_on_each_pfn(exclude_pfn_slot,0);
}
#endif
static inline unsigned int _get_skip_depth(unsigned char type)
{
	switch(type)
	{
		case KMEMC_FREE:
	    case KMEMC_ALLOC:	
			return 4;
		case KMEM_KFREE:
		case KMEMC_ALLOC_NODE:
			return 5;			
		case PAGE_FREE:
			return 4;
		case PAGE_ALLOC:
			return 4;
        default:
            break;		
	}
    return 3;	
}
static inline void __enc_stack_trace(unsigned char *enc_stack,unsigned long bits_start,
		unsigned long func_ptr)
{
	unsigned char *byte;
	unsigned bit_index=0;
	unsigned char bit_in_byte=0;
	unsigned char enc_stack_bitlen=(func_ptr&0x1)?enc_stack_kern_bitlen:enc_stack_mod_bitlen;
	//pr_debug("bits start=%d,enc_stack_bitlen=%d\n",bits_start,enc_stack_bitlen);
	
	if((bits_start+enc_stack_bitlen)>=(PAGE_TRACK_STACK_SIZE*BITS_PER_BYTE))return;
	
	for(byte=enc_stack+(bits_start/BITS_PER_BYTE);bit_index<enc_stack_bitlen;bit_index++)
	{
		bit_in_byte=(bits_start+bit_index)%BITS_PER_BYTE;
		if(func_ptr&BIT(bit_index))
			byte[0]|=BIT(bit_in_byte);
		else
			byte[0]&=~(BIT(bit_in_byte));
		if((0==(bit_in_byte+1)%BITS_PER_BYTE))byte++;
	}
}
static inline void __dec_stack_trace(unsigned char *enc_stack,unsigned long *stack_trace)
{
	unsigned char *byte;
	unsigned bit_index=0;
	unsigned char bit_in_byte=0;
	unsigned long func_ptr=0;
	unsigned stack_index=0;
	unsigned bits_start=0;
	unsigned char enc_stack_bitlen;
	
	//pr_debug("%s\n",__func__);
	for(stack_index=0;stack_index<PAGE_TRACK_STACK_MAX_DEPTH;stack_index++)
	{
		for(byte=enc_stack+(bits_start/BITS_PER_BYTE),bit_index=0,func_ptr=0,enc_stack_bitlen=enc_stack_mod_bitlen;
			bit_index<enc_stack_bitlen;bit_index++)
		{
			if((bits_start+bit_index)>=PAGE_TRACK_STACK_SIZE*BITS_PER_BYTE)goto done;
			bit_in_byte=(bits_start+bit_index)%BITS_PER_BYTE;
			
			if(byte[0] &BIT(bit_in_byte))
			{
				if(bit_index==0)enc_stack_bitlen=enc_stack_kern_bitlen;
				func_ptr|=BIT(bit_index);
			}
			else
				func_ptr&=~BIT(bit_index);
			if(0==((bit_in_byte+1)%BITS_PER_BYTE))byte++;
		}
		//pr_debug("%d:%lx\n",stack_index,func_ptr);
		if(func_ptr &0x1)
		{
			func_ptr>>=1;
			func_ptr+=(unsigned long)_text;
		}
		else if(func_ptr)
		{
			func_ptr>>=1;
			func_ptr+=MODULES_VADDR;
		}
		stack_trace[stack_index]=func_ptr;
		bits_start+=enc_stack_bitlen;
	}
done:
	for(;stack_index<PAGE_TRACK_STACK_MAX_DEPTH;stack_index++)
		stack_trace[stack_index]=ULONG_MAX;
}
static inline void _enc_stack_trace(struct stack_trace *trace,unsigned char *enc_stack)
{
	int i;
	unsigned long offset;
	unsigned long enc_bit_index=0;
	unsigned char min_bit_len=min(enc_stack_kern_bitlen,enc_stack_mod_bitlen);
	
	for (i = 0; i < trace->nr_entries; i++)
	{
		if(core_kernel_text(trace->entries[i]))
		{
			offset=trace->entries[i]-(unsigned long)_text;
			if(offset>(BIT(enc_stack_kern_bitlen-1)-1))
				if((trace->entries[i]>=(unsigned long)_sinittext) &&
					(trace->entries[i]<=(unsigned long)_einittext))
					continue;//kernel init case
				else WARN(1,"invalid kernel text addr %lx from %lx!",trace->entries[i],(unsigned long)_text);
			else
			{
				offset<<=1;
				offset|=0x1;
				//pr_debug("%d:kernel addr %lx,offset=%lx\n",i,trace->entries[i],offset);
				__enc_stack_trace(enc_stack,enc_bit_index,offset);
				enc_bit_index+=enc_stack_kern_bitlen;
			}	
		}
		else if((trace->entries[i]>=MODULES_VADDR) && (trace->entries[i]<MODULES_END))
		{
			offset=trace->entries[i]-MODULES_VADDR;
			if(offset>(BIT(enc_stack_mod_bitlen-1)-1))
				WARN(1,"invalid mod text addr %lx from %lx!",(unsigned long)trace->entries[i],MODULES_VADDR);
			else
			{
				offset<<=1;
				//pr_debug("%d:module addr %lx,offset=%lx\n",i,trace->entries[i],offset);
				__enc_stack_trace(enc_stack,enc_bit_index,offset);
				enc_bit_index+=enc_stack_mod_bitlen;
			}	
		}
		else if(trace->entries[i]==ULONG_MAX)
		{
			__enc_stack_trace(enc_stack,enc_bit_index,
			min_bit_len==enc_stack_kern_bitlen);
			enc_bit_index+=min_bit_len;
		}	
		else
			continue;
		//	WARN(1,"invalid stack addr %pS(%lx)!",(void *)trace->entries[i],trace->entries[i]);
	}
	for (;i < PAGE_TRACK_STACK_MAX_DEPTH;i++)
	{
		__enc_stack_trace(enc_stack,enc_bit_index,min_bit_len==enc_stack_kern_bitlen);
		enc_bit_index+=min_bit_len;
	}
}
static inline unsigned long _enc_page_track_kmem_data(struct kmem_cache_data * data)
{
#if BITS_PER_LONG==64
	unsigned long enc_data=0;
	
	enc_data|=data->addr &((UL(1) << VA_BITS)-1);//addrs;
	enc_data |=(unsigned long)(data->pid)<<VA_BITS;//pid
	return enc_data;
#else
	return data->addr;
#endif
}
static struct kmem_cache_data _dec_page_track_kmem_data(unsigned long data)
{
#if BITS_PER_LONG==64	
	struct kmem_cache_data dec_data = {0};
	
	dec_data.addr=data|~((UL(1) << VA_BITS)-1);
	dec_data.pid=data>>VA_BITS;
	return dec_data;
#else
  
	struct kmem_cache_data dec_data = {0};
	dec_data.addr=data;
	return dec_data;
#endif	
}

#ifndef CONFIG_CC_IS_CLANG
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
static inline unsigned long _enc_page_track_page_data(struct kmem_page_data * data)
{
	unsigned long enc_data=0;
	unsigned int order_width=fls_long(MAX_ORDER-1);
	unsigned int index_width=MAX_ORDER;
	
	enc_data|=data->order;
	enc_data |=(unsigned long)(data->index)<<order_width;
	enc_data |=(unsigned long)(data->pid)<<(order_width+index_width);
	return enc_data;
}

#ifndef CONFIG_CC_IS_CLANG
#pragma GCC diagnostic pop
#endif

static struct kmem_page_data _dec_page_track_page_data(unsigned long data)
{
	struct kmem_page_data dec_data = {0};
	unsigned int order_width=fls_long(MAX_ORDER-1);
	unsigned int index_width=MAX_ORDER;
	
	dec_data.order=data&((1<<order_width)-1);
	dec_data.index=(data>>order_width)&((1<<index_width)-1);
	dec_data.pid=data>>(order_width+index_width);
	return dec_data;
}
static inline void _update_perpage_track_info(struct page_track_info *track,void *data,
      unsigned long type)
{
   struct _page_track_info *track_info=NULL;
   unsigned long entries[PAGE_TRACK_STACK_MAX_DEPTH];
   struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries =PAGE_TRACK_STACK_MAX_DEPTH,
		.skip = 3
	};
	unsigned long flags;
	unsigned char track_index=0;
	unsigned long min_when=ULONG_MAX;
	unsigned long enc_data;
#if BITS_PER_LONG < 64
	s64 time_us;
#endif	
	trace.skip=_get_skip_depth(type);
	if((type==PAGE_ALLOC) || (type==PAGE_FREE))
		enc_data=_enc_page_track_page_data((struct kmem_page_data *)data);
    else
		enc_data=_enc_page_track_kmem_data((struct kmem_cache_data *)data);

	spin_lock_irqsave(&track->lock,flags);
	for(track_index=0;track_index<TRACK_COUNT_PER_PAGE;track_index++)
	{
		if(track->track_info[track_index].when<min_when)
		{
			min_when=track->track_info[track_index].when;
			track_info=track->track_info+track_index;
		}
	}
	#if BITS_PER_LONG < 64
	time_us=ktime_to_us(ktime_get());
	track_info->when=(unsigned long)((time_us>>7)&ULONG_MAX);	
	#else
	track_info->when=(unsigned long)(ktime_to_us(ktime_get()) &ULONG_MAX);
	#endif
	track_info->data=enc_data;
//	trace.entries=track_info->stack_trace;
//	memset(entries,0xff,sizeof(entries));
	save_stack_trace(&trace);
	_enc_stack_trace(&trace,track_info->enc_stack);
	spin_unlock_irqrestore(&track->lock,flags);
}
static struct page_track_info *_get_track_info(unsigned long index)
{
/*only protect track info get/delete.
do not consider thread A get and use,thread b delete &free case.
since the module memory should not get into alloc/free
*/	
	unsigned long flags;
	struct page_track_info *track;
	read_lock_irqsave(&page_track_tree_lock,flags);
	track=radix_tree_lookup(&page_track_tree,index);
	read_unlock_irqrestore(&page_track_tree_lock,flags);
	return track;
}
static void kmem_cache_alloc_check(void *data,unsigned long call_site, const void *ptr, 
	size_t bytes_req, size_t bytes_alloc, gfp_t gfp_flags)
{
	unsigned long pfn=page_to_pfn(virt_to_page(ptr));
	struct page_track_info *track=NULL;
	unsigned int count=PAGE_ALIGN(bytes_alloc)>>PAGE_SHIFT;
	struct kmem_cache_data cache_data = {0};
	
	if(!ptr)return;
	if(pfn>=mem_start_pfn)
	{
		while(count--)
		{
			//if(bEnableDebug)pr_info("update %ld for %px \n",pfn,ptr);
			track=_get_track_info(pfn-mem_start_pfn);
			WARN(!track,"can not get track for pfn %lx\n",pfn);
			if(track)
			{
				cache_data.addr=(unsigned long)ptr;
				cache_data.pid=task_pid_nr(current);
				_update_perpage_track_info(track,&cache_data,(unsigned long)data);
			}
			pfn++;
		}
	}
}
static void kmem_cache_alloc_node_check(void *data,unsigned long call_site, const void *ptr,
		 size_t bytes_req, size_t bytes_alloc,
		 gfp_t gfp_flags, int node)
{
	kmem_cache_alloc_check(data,call_site,ptr,bytes_req,bytes_alloc,gfp_flags);
}
static void kmem_cache_free_check(void *data,unsigned long call_site, const void *ptr)
{
	unsigned long pfn=page_to_pfn(virt_to_page(ptr));
	struct page *head_page=virt_to_head_page(ptr);
	struct page_track_info *track=NULL;
	unsigned int count=1;
	struct kmem_cache_data cache_data = {0};
	
	if(!ptr)return;
	//only check kfree since the page maybe invalid when trace_kmem_cache_free is called
	if((unsigned long)data==KMEM_KFREE)
	{	
		WARN_ON(!PageSlab(head_page));
		if(PageSlab(head_page))
		{
			count=head_page->slab_cache?1:PAGE_ALIGN(head_page->slab_cache->object_size)>>PAGE_SHIFT;
		}
	}	
	if(pfn>=mem_start_pfn)
	{
		while(count--)
		{
			track=_get_track_info(pfn-mem_start_pfn);
			WARN(!track,"can not get track for pfn %lx\n",pfn);
			if(track)
			{
				cache_data.addr=(unsigned long)ptr;
				cache_data.pid=task_pid_nr(current);
				_update_perpage_track_info(track,&cache_data,(unsigned long)data);
			}
			pfn++;
		}
	}
}
static void kfree_check(void *data,unsigned long call_site, const void *ptr)
{
	struct page *page;
	if(ZERO_OR_NULL_PTR(ptr))return;
	
	page = virt_to_head_page(ptr);
	if (PageSlab(page))kmem_cache_free_check(data,call_site,ptr);
}
static void mm_page_free_check(void *data,struct page *page, unsigned int order)
{
	unsigned long pfn=page_to_pfn(page);
	struct page_track_info *track=NULL;
	unsigned int count=1<<order;
	unsigned int index=0;
	struct kmem_page_data page_data = {0};
	
	if(!page)return;
	if(pfn>=mem_start_pfn)
	{
			while(count--)
			{
				track=_get_track_info(pfn-mem_start_pfn);
				WARN(!track,"can not get track for pfn %lx\n",pfn);
				if(track)
				{
					page_data.order=order;
					page_data.index=index;
					page_data.pid=task_pid_nr(current);
					_update_perpage_track_info(track,&page_data,(unsigned long)data);
				}
				pfn++;
				index++;
			}
	}
}
static void mm_page_alloc_check(void *data,struct page *page, unsigned int order,
			gfp_t gfp_flags, int migratetype)
{
	unsigned long pfn=page_to_pfn(page);
	struct page_track_info *track=NULL;
	unsigned int count=1<<order;
	unsigned int index=0;
	struct kmem_page_data page_data = {0};
	
	if(!page)return;
	if(pfn>=mem_start_pfn)
	{
		while(count--)
		{
			track=_get_track_info(pfn-mem_start_pfn);
			WARN(!track,"can not get track for pfn %lx-%d-%px-%d\n",pfn,count,page,PageSlab(page));
			if(track)
			{
				page_data.order=order;
				page_data.index=index;
				page_data.pid=task_pid_nr(current);
				_update_perpage_track_info(track,&page_data,(unsigned long)data);
			}	
			pfn++;			
			index++;			
		}
	}
}
static void _print_page_track_stack_trace(unsigned char *enc_stack)
{
	int i;
	unsigned long stacktrace[PAGE_TRACK_STACK_MAX_DEPTH];
	__dec_stack_trace(enc_stack,stacktrace);

	for (i = 0; i < PAGE_TRACK_STACK_MAX_DEPTH; i++)
	{
		//pr_alert("\t%pS\n",stacktrace[i]);
		if(ULONG_MAX!=stacktrace[i])pr_alert("\t<%lx> %pS\n",stacktrace[i],(void*)stacktrace[i]);
	}	
}
static void * delete_page_track_slot(unsigned long pfn,unsigned char block)
{
	struct page_track_info *track=NULL;
	unsigned long flags;
	if(pfn>mem_start_pfn)
	{	
		if(block)write_lock_irqsave(&page_track_tree_lock,flags);
		track=radix_tree_delete(&page_track_tree,pfn-mem_start_pfn);
		if(block)write_unlock_irqrestore(&page_track_tree_lock,flags); 
	}
	return track;
}
#if defined(CONFIG_MODULES)
static int module_event_notify(struct notifier_block *self, unsigned long action,
                 void *v)
{
    struct module *mod=v;
	struct vm_struct *mod_vm;
	unsigned int index=0;
	unsigned long pfn;
	int ret;
	struct page_track *track;
	
    if(action==MODULE_STATE_LIVE)
    {
		pr_debug("%s:%px-%x\n",mod->name,mod->core_layout.base,mod->core_layout.size);
		mod_vm=find_vm_area(mod->core_layout.base);
		if(mod_vm)
		{
			for(;index<mod_vm->nr_pages;index++)
			{
				pfn=page_to_pfn(mod_vm->pages[index]);
				if((track=delete_page_track_slot(pfn,1)))
				{
					//at the time,mem alloc had beed done.so no need to protect page_track when free
					if(track)kmem_cache_free(page_track_cache,track);
					pr_debug("%s:remove pfn %ld\n",mod->name,pfn);
				}
			}
			pr_debug("%s:addr=%px pages=%x\n",mod->name,mod_vm->addr,mod_vm->nr_pages);
		}	
    }
	if(action==MODULE_STATE_GOING)
	{
		pr_debug("%s-going:%px-%x\n",mod->name,mod->core_layout.base,mod->core_layout.size);
		mod_vm=find_vm_area(mod->core_layout.base);
		for(index=0;mod_vm && index<mod_vm->nr_pages;index++)
		{
			pfn=page_to_pfn(mod_vm->pages[index]);
			if(pfn>mem_start_pfn)
			{
				ret=_init_pfn_slot(pfn,1);
				pr_debug("%s:int pfn %ld again,ret=%d\n",mod->name,pfn,ret);
			}
		}
	}
    return NOTIFY_OK;
}
static struct notifier_block module_event_nb = {
    .notifier_call = module_event_notify,
};
#endif
void print_page_track_info(struct page *page,unsigned int around_count)
{
	unsigned long pfn=page_to_pfn(page);
	struct page_track_info *track=NULL;
	unsigned char track_index=0;
	struct _page_track_info *track_info=NULL;
	struct kmem_cache_data cache_data = {0};
	struct kmem_page_data page_data = {0};
	
	unsigned long flags;
	s64 time_us;
	#if BITS_PER_LONG < 64
	unsigned int time_units;
	#endif 
	
	pr_alert("original page %px(%lx) track info:\n",page,page_to_pfn(page));
	for(pfn=page_to_pfn(page)-around_count;pfn<=page_to_pfn(page)+around_count;pfn++)
	{
		if(pfn>=mem_start_pfn)
		{
			track=_get_track_info(pfn-mem_start_pfn);
		}
		//WARN(!track,"can not get track for pfn %ld\n",pfn);
		if(track)
		{
			#if BITS_PER_LONG < 64
			time_us=ktime_to_us(ktime_get());
			time_units=1<<7;
			pr_alert("page %px(%lx) track info:now %ld/128 us\n",pfn_to_page(pfn),pfn,
					(unsigned long)((time_us>>7) &ULONG_MAX));			
			#else
			time_us=ktime_to_us(ktime_get());
			pr_alert("page %px(%lx) track info:now %lld.%06ld s\n",pfn_to_page(pfn),pfn,time_us/USEC_PER_SEC,(unsigned long)(time_us%USEC_PER_SEC));
			#endif
			for(track_index=0;track_index<TRACK_COUNT_PER_PAGE;track_index++)
			{
				track_info=track->track_info+track_index;
				spin_lock_irqsave(&track->lock,flags);
				cache_data=_dec_page_track_kmem_data(track_info->data);
				page_data=_dec_page_track_page_data(track_info->data);
				pr_alert("\t%d:%lx at  %ld.%06ld s\n",track_index,track_info->data,track_info->when/USEC_PER_SEC,(unsigned long)(track_info->when%USEC_PER_SEC));				
				pr_alert("\t\taddr=%lx,pid=%d\n",cache_data.addr,cache_data.pid);
				pr_alert("\t\torder=%d,index=%d,pid=%d\n",page_data.order,page_data.index,page_data.pid);				
				_print_page_track_stack_trace(track_info->enc_stack);
				spin_unlock_irqrestore(&track->lock,flags);
			}		
		}
		pr_alert("========================================\n\n");
	}	
}
EXPORT_SYMBOL_GPL(print_page_track_info);
static __init int _enable_kmem_trace(void)
{
    int ret=0;
	ret=register_trace_kmem_cache_free(kmem_cache_free_check,(void *)KMEMC_FREE);
	if(ret)goto fail1;
 	ret=register_trace_kmem_cache_alloc(kmem_cache_alloc_check,(void *)KMEMC_ALLOC);
	if(ret)goto fail2;
	ret=register_trace_kmalloc(kmem_cache_alloc_check,(void *)KMEMC_ALLOC);
	if(ret)goto fail3;
	ret=register_trace_mm_page_alloc(mm_page_alloc_check,(void *)PAGE_ALLOC);
	if(ret)goto fail4;

	ret=register_trace_mm_page_free(mm_page_free_check,(void *)PAGE_FREE);   
	if(ret)goto fail5;   
	ret=register_trace_kfree(kfree_check,(void *)KMEM_KFREE); 
	if(ret)goto fail6;
	
	ret=register_trace_kmalloc_node(kmem_cache_alloc_node_check,(void *)KMEMC_ALLOC_NODE);   
	if(ret)goto fail7;   
	ret=register_trace_kmem_cache_alloc_node(kmem_cache_alloc_node_check,(void *)KMEMC_ALLOC_NODE); 
	if(ret)goto fail8;	
	
	return ret;
fail7:
	unregister_trace_kfree(kfree_check,NULL); 
fail8:
	unregister_trace_kmalloc_node(kmem_cache_alloc_node_check,NULL); 	
fail6:
	unregister_trace_mm_page_free(mm_page_free_check,NULL);	
fail5:
	unregister_trace_mm_page_alloc(mm_page_alloc_check,NULL);	
fail4:
	unregister_trace_kmalloc(kmem_cache_alloc_check,NULL);
fail3:
	unregister_trace_kmem_cache_alloc(kmem_cache_alloc_check,NULL);
fail2:
	unregister_trace_kmem_cache_free(kmem_cache_free_check,NULL);
fail1:
	return ret;
}
static ssize_t selftest_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
	void *ptr1,*ptr2;
	
	ptr1=kmalloc(0x800,GFP_KERNEL);
	pr_alert("kmalloc %px,pfn:%ld count=%d\n",ptr1,page_to_pfn(virt_to_page(ptr1)),PAGE_ALIGN(0x800)>>PAGE_SHIFT);
	print_page_track_info(virt_to_page(ptr1),0);
	ptr2=alloc_pages(GFP_KERNEL,1);
	pr_alert("alloc order 1 %px\n",ptr2);
	print_page_track_info(ptr2,1);
	//free_pages((unsigned long)ptr1,compound_order(virt_to_head_page(ptr1)));
	kfree(ptr1);
	pr_alert("kfree %px\n",ptr1);
	print_page_track_info(virt_to_page(ptr1),0);
	__free_pages(ptr2,1);
	pr_alert("free_pages %px\n",ptr2);
	print_page_track_info(ptr2,1);
    tracing_off();
    return count;    
}
static ssize_t selftest_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
   unsigned long entries[PAGE_TRACK_STACK_MAX_DEPTH];
   int i;
   struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries =PAGE_TRACK_STACK_MAX_DEPTH,
		.skip = 2
	};
	unsigned char enc_stack[PAGE_TRACK_STACK_SIZE];
	save_stack_trace(&trace);
	//print_stack_trace(&trace,0);
	for(i=0;i<trace.nr_entries;i++)
		pr_info("%d:<%lx>%pS\n",i,trace.entries[i],(void *)trace.entries[i]);
	_enc_stack_trace(&trace,enc_stack);
	for(i=0;i<PAGE_TRACK_STACK_SIZE;i++)
		pr_info("%d:%02X\n",i,enc_stack[i]);
	_print_page_track_stack_trace(enc_stack);
    return 0;
}
static struct kobject *page_track_kobj=NULL;
static struct kobj_attribute ST_attr=__ATTR_RW(selftest);
static const struct attribute *page_track_attrs[] = {
	&ST_attr.attr,
	NULL,
};
static __init int page_track_sysfs_init(void)
{
    int error=0;
    page_track_kobj=kobject_create_and_add("page_track", kernel_kobj);
    if(!page_track_kobj)
    {
        return -ENOMEM;
    }
    error = sysfs_create_files(page_track_kobj, page_track_attrs);
    if (error)
    {
        kobject_put(page_track_kobj);
        return error;
    }
    return error;
}
static struct page * _get_page_from_addr(unsigned long addr)
{
	unsigned long flags, paddr;
	local_irq_save(flags);
#if BITS_PER_LONG>=64
	asm volatile (
			"  AT S1E1R, %1\n"
			"  ISB\n"
			"  MRS %0, PAR_EL1\n"
			: "=r" (paddr) : "r" (addr) : "cc");
#else
	asm volatile (
			"  mcr p15, 0, %1, c7, c8, 0\n"
			"  ISB\n"
			"  mrc p15, 0, %0, c7, c4, 0\n"
			: "=r" (paddr) : "r" (addr) : "cc");	
#endif
	local_irq_restore(flags);
	pr_debug("PAR is %lx\n",paddr);
	if (paddr & 1)
		return NULL;
	if(sizeof(paddr)>4)
	{
		paddr &=0xffffffffffff;//bit 47:12
	}	
	return phys_to_page(paddr);
}
static void _dump_possibletrack_info(unsigned long addr,const char *name)
{
	struct page *page;
#if BITS_PER_LONG<64
	if (addr < TASK_SIZE || addr > -256UL)
#else	
    if (addr < VA_START || addr > -256UL)
#endif		
		return;
	page=_get_page_from_addr(addr);
	if(page)
	{
		pr_alert("\n%s: %#lx:\n", name, addr);
		print_page_track_info(page,0);
	}
}
static void _page_track_dump_ondie(struct pt_regs *regs)
{
	unsigned int i;

#if BITS_PER_LONG>=64
	for (i = 0; i < 30; i++) {
		char name[11];
		snprintf(name, sizeof(name), "X%u's page", i);
		_dump_possibletrack_info(regs->regs[i],name);
	}
	_dump_possibletrack_info(regs->sp,"SP's page");
#else
	for (i = 0; i < 10; i++) {
		char name[11];
		snprintf(name, sizeof(name), "R%u's page", i);
		_dump_possibletrack_info(regs->uregs[i],name);
	}
	_dump_possibletrack_info(regs->ARM_fp,"FP's page");
	_dump_possibletrack_info(regs->ARM_ip,"IP's page");
	_dump_possibletrack_info(regs->ARM_fp,"FP's page");
	_dump_possibletrack_info(regs->ARM_sp,"SP's page");
#endif	
}
static int page_track_exceptions_notify(struct notifier_block *nb,
				    unsigned long val, void *data)
{
	struct die_args *pargs;
	struct pt_regs *excp_regs;
	
	pr_debug("die_cb:%ld-%px\n",val,data);
	if(DIE_OOPS==val)
	{
		pargs=(struct die_args *)data;
		excp_regs=pargs->regs;
		_page_track_dump_ondie(excp_regs);
	}
	return NOTIFY_OK;
}

static struct notifier_block page_track_exceptions_nb = {
	.notifier_call = page_track_exceptions_notify,
};

static __init void _init_page_track_obj(void * ptr)
{
	memset(ptr,0,sizeof(struct page_track_info));
}
static __init int page_track_init(void)
{
    int ret=0;
    if(b_page_track)
    {
		//actually,ctor means no merge.but we also add SLAB_NEVER_MERGE in case of somebody will remove ctor
		page_track_cache=kmem_cache_create("page_track_rec", sizeof(struct page_track_info),
                    0,SLAB_PANIC,_init_page_track_obj);
        mem_start_pfn=PFN_UP(memblock_start_of_DRAM());
		pr_debug("mem_start_pfn=%lx\n",mem_start_pfn);
		_init_enc_stack_bitlen();
		ret=init_radix_slot();
		if(!ret)
		{
			#if !defined(CONFIG_MODULES)
			(void)exclude_radix_slot();
			#endif
			ret=_enable_kmem_trace();
			if(ret)
			{
				pr_err("enable_kmem_trace fails:%d\n",ret);
				return ret;
			}
			#if defined(CONFIG_MODULES)
			register_module_notifier(&module_event_nb);
			#endif
			page_track_sysfs_init();
			return register_die_notifier(&page_track_exceptions_nb);
		}

        return ret;
    }
    return 0;
}
late_initcall(page_track_init);

static int __init setup_page_track(char *str)
{
    if (*str++ != '=' || !*str)
        return -EINVAL;
    if(!strcmp(str,"on") || *str=='1')
		b_page_track=1;
    return 1;
}
__setup("page_track", setup_page_track);
#endif
