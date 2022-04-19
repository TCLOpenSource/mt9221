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

#ifdef CONFIG_SYSFS 
#define pr_fmt(fmt) "oom_diag: " fmt 
#include <linux/stacktrace.h>
#include <linux/kallsyms.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/ftrace.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/init.h>
#include <asm/setup.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/slub_def.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include <linux/oom.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/vmstat.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/kasan.h>
#include <linux/jhash.h>

#include "mm/slab.h"
#include "utils.h"

#define ENABLE_SELF_TEST 1

//only dump the vmalloc when its alloc size>SINGLE_VMALLOC_CHECK_SIZE_THD
#define SINGLE_VMALLOC_CHECK_SIZE_THD (vmalloc_dump_thd) 
#define DROP_VMALLOC_RECORDS_FULL (10) 
//only count vmalloc when vmalloc size >MEM_TOAL/COUNT_VMALLOC_RATIO_THD
#define COUNT_VMALLOC_RATIO_THD (vmalloc_dump_ratio)

#define STACK_HASH_SEED 0x9747b28c


static DEFINE_MUTEX(oom_diag_mutex);
static void *count_vm_buf=NULL;
static unsigned int vm_buf_size=(PAGE_SIZE*2);//default 2 page in case of showing slub full trace
static atomic_t skip_oom_diag=ATOMIC_INIT(0);

static bool sort_vmalloc_by_size=false;//vmalloc records sort by call times by default
module_param(sort_vmalloc_by_size,bool ,0644);

static unsigned int vmalloc_dump_thd=0x100000;//default is 1M
module_param(vmalloc_dump_thd,int ,0644);

static unsigned int vmalloc_dump_ratio=1000;//1/1000
module_param(vmalloc_dump_ratio,int ,0644);

struct vmalloc_count
{
    unsigned int pagenum;
    unsigned int count;
    unsigned long caller;
};
struct slab_count_dist
{
    unsigned int size;
    struct kmem_cache *slab;
};
struct slab_caller_dist
{
    unsigned int count;
    struct track * alloc_track;
#ifdef CONFIG_STACKTRACE	
    u32 stack_hash;
#else
    unsigned long stack_hash;
#endif
};
static void dump_vm_record(struct vmalloc_count *record,
            unsigned int len,unsigned int size_thd)
{
    unsigned int index=0;
    
    while(index<len)
    {
        if(size_thd && (record[index].pagenum<<PAGE_SHIFT)<size_thd)
        {
            index++;
            continue;
        }
        pr_err("%08d:vmalloc total %08u KB in %08d times by %pS(%lx) \n",
            index,record[index].pagenum<<(PAGE_SHIFT-10),
            record[index].count,(void *)record[index].caller,
            record[index].caller);
        index++;    
    }
}
static int cmp_vm_record_by_caller(const void *a, const void *b)
{
    const struct vmalloc_count *pre = a, *next = b;
    int i=0;
    //sort by caller
    i=pre->caller>next->caller?-1:pre->caller<next->caller;
    return i;
}
static int cmp_vm_record_by_count(const void *a, const void *b)
{
    const struct vmalloc_count *pre = a, *next = b;
    int i=0;
    //sort by caller,count
    if(sort_vmalloc_by_size)i=0;
    else i=pre->count>next->count?-1:pre->count<next->count;
    
    if(i==0)
        i=pre->pagenum>next->pagenum?-1:pre->pagenum<next->pagenum;
    if(sort_vmalloc_by_size && (i==0))
        i=pre->count>next->count?-1:pre->count<next->count;
    return i;
}
static unsigned int _do_unique_sort_by_caller(struct vmalloc_count *record,unsigned int len)
{
    unsigned int index=1;
    unsigned int uniq_index=0;
    unsigned int dup_count=0;
    unsigned char need_memmove=0;
    unsigned int end=len;
    
    sort(record, len, sizeof(struct vmalloc_count),cmp_vm_record_by_caller, NULL);
    while(index<end)
    {
        if(record[index-1].caller==record[index].caller)
        {
            record[uniq_index].pagenum+=record[index].pagenum;
            record[uniq_index].count+=record[index].count;
            need_memmove=1;
            dup_count++;    
        }
        else
        {
            if(need_memmove)
            {
                memmove(record+uniq_index+1,record+index,(end-index)*sizeof(record[0]));
                end-=index-uniq_index-1;
                index=uniq_index+1;
            }
            uniq_index=index;
            need_memmove=0;
        }   
        index++;
    }
    return len-dup_count;
}
static void _do_sort_by_count(struct vmalloc_count *record,unsigned int len)
{
    sort(record, len, sizeof(struct vmalloc_count),cmp_vm_record_by_count, NULL);   
}
static void drop_vm_record(struct vmalloc_count *record,
    unsigned int start,unsigned int len)
{
    unsigned int count=0;
    unsigned int size=0;
    unsigned int index=start;
    while(index<(start+len))
    {
        count+=record[index].count;
        size+=record[index].pagenum;
        index++;
    }
    pr_err("drop the last %d records includes %u KB in %d allocs\n",
        len,size<<(PAGE_SHIFT-10),count);
}   
static unsigned long _count_vmalloc(void)
{
    unsigned int record_count=vm_buf_size/sizeof(struct vmalloc_count);
    struct vmalloc_count *vm_record=(struct vmalloc_count *)count_vm_buf;
    unsigned int index=0;
    unsigned long vm_addr=MODULES_VADDR;
    struct vm_struct *vm;
    unsigned long unmaped_hole=0;
    unsigned long total_vmalloc_pages=0;
    bool need_partial_sort=record_count>(DROP_VMALLOC_RECORDS_FULL*2);
    struct sysinfo sysinfo;
    si_meminfo(&sysinfo);
    
    memset(vm_record,0,record_count*sizeof(vm_record[0]));

    while(1)
    {
        if((vm_addr>=MODULES_END) && (vm_addr<VMALLOC_START))
        {
            vm_addr=VMALLOC_START;
        }
        if(vm_addr>=VMALLOC_END)
            break;
        vm=find_vm_area((void *)vm_addr);
        if(!vm)
        {
            vm_addr+=PAGE_SIZE;
            unmaped_hole+=PAGE_SIZE;
            if(unmaped_hole>100*1024*1024)
            {
                if(vm_addr<MODULES_END)vm_addr=VMALLOC_START;
                else break;
            }   
        }
        else
        {
            unmaped_hole=0;
            vm_addr+=vm->size;
            if(!vm->nr_pages)continue;
            total_vmalloc_pages+=vm->nr_pages;
            if(vm_addr<VMALLOC_START)continue;//do not count modules caller
            vm_record[index].pagenum=vm->nr_pages;
            barrier();
            //ensure vm is valid at this moment
            vm_record[index].caller=(unsigned long)vm->caller;
            if(!__kernel_text_address(vm_record[index].caller))continue;
            vm_record[index].count=1;
            index++;
            if(index==record_count)
            {
                index=_do_unique_sort_by_caller(vm_record,record_count);
                if(index==record_count)//no available space,consume it
                {
                    _do_sort_by_count(vm_record,record_count);
                    if(need_partial_sort)
                    {
                        drop_vm_record(vm_record,
                        record_count-DROP_VMALLOC_RECORDS_FULL,
                          DROP_VMALLOC_RECORDS_FULL);
                        index-=DROP_VMALLOC_RECORDS_FULL;                       
                    }
                    else
                    {
                        pr_err("dump the dropped vmalloc record:\n");
                        dump_vm_record(vm_record,record_count,0);
                        index=0;
                    }
                }
            }
        }
    }
    pr_err("vmalloc used %ld/%ld KB\n",
          total_vmalloc_pages<<(PAGE_SHIFT-10)
          ,sysinfo.totalram<<(PAGE_SHIFT-10));
    
    if(COUNT_VMALLOC_RATIO_THD && (total_vmalloc_pages
        <(sysinfo.totalram/COUNT_VMALLOC_RATIO_THD)))
    {
           // pr_err("thd %ld \n",sysinfo.totalram/COUNT_VMALLOC_RATIO_THD);
            return total_vmalloc_pages;
    }
    index=_do_unique_sort_by_caller(vm_record,index);
    _do_sort_by_count(vm_record,index);
    dump_vm_record(vm_record,index,SINGLE_VMALLOC_CHECK_SIZE_THD);
    return total_vmalloc_pages;
}
#if defined(CONFIG_SLUB_DEBUG) && !defined(CONFIG_DEBUG_KMEMLEAK)
#define DROP_CALLER_NUM_FULL 10
#define TOP_CALLER_NUM 15
#define TOP_N_SLAB 10
#define DROP_SLAB_NUM_FULL 10
#define DUMP_FULL_STACK_NUM 3
#define CHECK_FULL_STACK_SLAB_NUM (TOP_N_SLAB/3) //only compare full stack
//only count slub when unreclaim size >MEM_TOAL/COUNT_SLUB_RATIA_THD
#define COUNT_SLUB_RATIO_THD (slub_dump_ratio)
//only check slub when its used size>SINGLE_SLUB_CHECK_SIZE_THD
#define SINGLE_SLUB_CHECK_SIZE_THD (slub_dump_thd) 

static unsigned int slub_dump_thd=0xa00000;//default is 10M
module_param(slub_dump_thd,int ,0644);

static unsigned int slub_dump_ratio=10;//default is 1/10
module_param(slub_dump_ratio,int ,0644);

static int cmp_slub_stackhash(const void *a, const void *b)
{
    const struct slab_caller_dist *pre = a, *next = b;
    int i=0;
    //sort by stack_hash
    i=pre->stack_hash>next->stack_hash?-1:pre->stack_hash<next->stack_hash;
    return i;
}
static int cmp_slub_caller_count(const void *a, const void *b)
{
    const struct slab_caller_dist *pre = a, *next = b;
    int i=0;
    //sort by count
    i=pre->count>next->count?-1:pre->count<next->count;
    return i;
}
static unsigned int _remove_dup_caller(struct slab_caller_dist *record,unsigned int len)
{
    unsigned int index=1;
    unsigned int uniq_index=0;
    unsigned int dup_count=0;
    unsigned char need_memmove=0;
    unsigned int end=len;
    while(index<end)
    {
        if(record[index-1].stack_hash==record[index].stack_hash)
        {
            record[uniq_index].count+=record[index].count;
            need_memmove=1;
            dup_count++;    
        }
        else
        {
            if(need_memmove)
            {
                memmove(record+uniq_index+1,record+index,(end-index)*sizeof(record[0]));
                end-=index-uniq_index-1;
                index=uniq_index+1;
            }
            uniq_index=index;
            need_memmove=0;
        }   
        index++;
    }
    return len-dup_count;
}
static void dump_slab_caller_dist(struct slab_caller_dist *record,
    unsigned int start,unsigned int end,bool show_full_stack)
{
    unsigned int index=start;
    
    while(index<end)
    {
        pr_err("\t%08d:allocate %u times by %pS\n",
            index,record[index].count,(void *)(record[index].alloc_track->addr));
        if(show_full_stack && (index<start+DUMP_FULL_STACK_NUM))
			show_slub_track(record[index].alloc_track,"oom_diag: ",8);
        index++;    
    }
}
static void drop_slub_caller_record(struct slab_caller_dist *record,
    unsigned int start,unsigned int len)
{
    unsigned int count=0;
    unsigned int index=start;
    while(index<(start+len))
    {
        count+=record[index].count;
        index++;
    }
    if(count!=len)
        pr_debug("\tdrop the last %d records includes %d allocs\n",
            len,count);     
}
static inline u32 hash_stack(unsigned long *entries, unsigned int size)
{
    return jhash2((u32 *)entries,
           size * sizeof(unsigned long) / sizeof(u32),
              STACK_HASH_SEED);
}
static unsigned int check_object_on_page(struct kmem_cache *s,struct page *page,
struct slab_caller_dist *record,unsigned int start,
unsigned int end,bool check_full_stack)
{
    void *p;
    void *addr = page_address(page);
    struct track *alloc,*free;
    unsigned int index=0;
    bool need_partial_sort=end>(TOP_CALLER_NUM*2);
    
    pr_debug("\t addr=%px,objects=%d,start=%d,end=%d\n",addr,
          page->objects,start,end);

	for (p = page->freelist; p; p = get_freepointer(s, p))//correction
	{
        alloc=get_slub_track(s,p,TRACK_ALLOC);
        free=get_slub_track(s,p,TRACK_FREE);
		if(alloc->when>=free->when)
		{
			alloc->when=free->when-1;
		}
	}
    for_each_object_in_page(p,page,s,addr)
    {
        alloc=get_slub_track(s,p,TRACK_ALLOC);
        free=get_slub_track(s,p,TRACK_FREE);
        if(alloc->when>=free->when)//now the object is in use
        {
			record[start+index].alloc_track=alloc;
        #ifdef CONFIG_STACKTRACE
            if(check_full_stack)
            {   
            #if BITS_PER_LONG>=64   
                record[start+index].stack_hash=hash_stack(alloc->addrs,TRACK_ADDRS_COUNT);
            #else
                record[start+index].stack_hash=hash_stack(alloc->addrs,TRACK_ADDRS_COUNT/2);            
            #endif
            }
            else
            record[start+index].stack_hash=alloc->addr;             
        #else
            record[start+index].stack_hash=alloc->addr;
		#endif
            record[start+index].count=1;
            index++;
        }
        if((index+start)==end)//full
        {
            pr_debug("record full...,try to remove duplicated record\n");
            sort(record,end, sizeof(record[0]),
                cmp_slub_stackhash, NULL);
            //remove the duplicated caller
            start=_remove_dup_caller(record,end);
            pr_debug("after remove duplicated.count=%d\n",start);
            index=0;
            if((index+start)==end)//non-exist duplicated caller
            {
                pr_debug("non-duplicated record,need to drop\n");
                sort(record,end, sizeof(record[0]),
                cmp_slub_caller_count, NULL);
                //drop the last DROP_CALLER_NUM_FULL
                if(need_partial_sort)
                {
                    drop_slub_caller_record(record,start-DROP_CALLER_NUM_FULL,DROP_CALLER_NUM_FULL);
                    start-=DROP_CALLER_NUM_FULL;
                }
                else
                {
                    drop_slub_caller_record(record,0,end);
                    start=0;                    
                }
            }
        }
    }
    return start+index;
}
static void check_slab_by_caller(struct kmem_cache *s,bool cmp_full_stack)
{
    unsigned long index=0;
    struct kmem_cache_node *n;
    int node;
    struct slab_caller_dist *record=(struct slab_caller_dist *)count_vm_buf;
    unsigned int record_count=vm_buf_size/sizeof(struct slab_caller_dist);

    pr_err("check slub %s caller distribute by %s(unit:%u B):\n",
        cache_name(s),cmp_full_stack?"full_stack ":"last caller ",s->size);
    memset(record,0,record_count*sizeof(record[0]));
    kasan_disable_current();
    kmem_cache_shrink(s);
    for_each_kmem_cache_node(s, node, n) 
    {
        unsigned long flags;
        struct page *page;

        if (!atomic_long_read(&n->nr_slabs))
            continue;

        spin_lock_irqsave(&n->list_lock, flags);
        list_for_each_entry(page, &n->partial, lru)
        {
            index=check_object_on_page(s,page,record,
			     index,record_count,cmp_full_stack);
        }
        list_for_each_entry(page, &n->full, lru)
        {
            index=check_object_on_page(s,page,record,
			   index,record_count,cmp_full_stack);
        }
        spin_unlock_irqrestore(&n->list_lock, flags);
    }
    sort(record,index,sizeof(record[0]),
                cmp_slub_stackhash, NULL);
    //remove the duplicated caller
    index=_remove_dup_caller(record,index);
    sort(record,index, sizeof(record[0]),
                cmp_slub_caller_count, NULL);           
    dump_slab_caller_dist(record,0,TOP_CALLER_NUM<index?TOP_CALLER_NUM:index,cmp_full_stack);
	kasan_enable_current();	
}
static void dump_slab_record(struct slab_count_dist *record,
    unsigned int start,unsigned int len)
{
    unsigned int index=start;
    
    while(index<len)
    {
        pr_err("%08d:%12s slab %32s use %08u KB by %u objects\n",
            index,(record[index].slab->flags&SLAB_RECLAIM_ACCOUNT)?"Reclaimable":"UnReclaimable",
            cache_name(record[index].slab),
            record[index].size/1024,record[index].size/record[index].slab->size);
        index++;    
    }
}
static int cmp_slab_record_by_size(const void *a, const void *b)
{
    const struct slab_count_dist *pre = a, *next = b;
    int i=0;
    //sort by size
    i=pre->size>next->size?-1:pre->size<next->size;
    return i;
}
static void _count_slabs(void)
{
    struct kmem_cache *s = NULL;
    struct slabinfo s_info;
    unsigned int record_count=vm_buf_size/sizeof(struct slab_count_dist);
    struct slab_count_dist *slab_record=(struct slab_count_dist *)count_vm_buf; 
    unsigned int index=0,dump_count,success=0;
    struct kmem_cache *top_kmem_caches[TOP_N_SLAB];
    bool need_partial_sort=record_count>(TOP_N_SLAB*2);
    struct sysinfo sysinfo;
    unsigned long total_slub_size;
    si_meminfo(&sysinfo);

    #if LINUX_VERSION_CODE > KERNEL_VERSION(4,13,0)
    total_slub_size=global_node_page_state(NR_SLAB_UNRECLAIMABLE);
    #else
    total_slub_size=global_page_state(NR_SLAB_UNRECLAIMABLE);
    #endif
    pr_err("slub used %ld/%ld KB\n",
          total_slub_size<<(PAGE_SHIFT-10),sysinfo.totalram<<(PAGE_SHIFT-10));
    
    if(COUNT_SLUB_RATIO_THD && (total_slub_size
        <sysinfo.totalram/COUNT_SLUB_RATIO_THD))return;
    mutex_lock(&slab_mutex);
    //firstly,we sort the slab by used size
    #if 0//LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
    list_for_each_entry(s, &slab_root_caches, root_caches_node)
    #else
    list_for_each_entry(s, &slab_caches, list)
    #endif
    {
        get_slabinfo(s,&s_info);
        slab_record[index].size=s_info.active_objs*s->size;
        if(!slab_record[index].size)continue;
        slab_record[index].slab=s;
        index++;
        if(index==record_count)
        {
            sort(slab_record, index, sizeof(struct slab_count_dist),
                cmp_slab_record_by_size, NULL);
            if(need_partial_sort)
            {
                dump_slab_record(slab_record,index-DROP_SLAB_NUM_FULL,DROP_SLAB_NUM_FULL);
                index-=DROP_SLAB_NUM_FULL;              
            }
            else
            {
                dump_slab_record(slab_record,0,index);
                index=0;
            }
        }
    }
    if(index!=record_count)
    {
        sort(slab_record, index, sizeof(struct slab_count_dist),
                cmp_slab_record_by_size, NULL);
    }
    dump_count=index<TOP_N_SLAB?index:TOP_N_SLAB;
    for(index=0;index<dump_count;index++)
    {
        if(slab_record[index].size<SINGLE_SLUB_CHECK_SIZE_THD)
        {
            pr_err("%08d:%s used %u KB less than %d KB\n",index,
                cache_name(slab_record[index].slab),
                slab_record[index].size>>10,SINGLE_SLUB_CHECK_SIZE_THD>>10);
            break;
        }
        top_kmem_caches[index]=slab_record[index].slab;
    }
    dump_slab_record(slab_record,0,index);
    dump_count=index;
    //secondly,check the TOP_N_SLAB slab by caller is possible
    for(index=0;index<dump_count;index++)
    {
        if (!(top_kmem_caches[index]->flags & SLAB_STORE_USER))continue;
        check_slab_by_caller(top_kmem_caches[index],
             success++<CHECK_FULL_STACK_SLAB_NUM);
    }

    mutex_unlock(&slab_mutex);
}
#else
static void _count_slabs(void)
{
    pr_err("can not count slabs due to CONFIG_SLUB_DEBUG is not set or \
    KMEMLEAK is on\n");
}   
#endif
static unsigned long get_vm_state(enum zone_stat_item item)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,13,0)
    return global_zone_page_state(item);
#else
    return global_page_state(item);
#endif
}
#define COUNT_ANON_PAGES_RATIO (60)
#define ANON_PAGE3_TOP_NUM (10)
struct anon_page_dist
{
	struct task_struct *tsk;
	unsigned long pagenum;
};
static int cmp_anon_page_by_size(const void *a, const void *b)
{
    const struct anon_page_dist *pre = a, *next = b;
    int i=0;
    //sort by size
    i=pre->pagenum>next->pagenum?-1:pre->pagenum<next->pagenum;
    return i;
}


static void _count_anons(void)
{
	unsigned int record_count=vm_buf_size/sizeof(struct anon_page_dist);
    struct task_struct *p;
    struct anon_page_dist *anon_record=(struct anon_page_dist *)count_vm_buf; 
    unsigned int index=0;
	unsigned int i=0;
	
	rcu_read_lock();
    for_each_process(p) {
		if (p->flags & (PF_KTHREAD | PF_WQ_WORKER))continue;
		if(!p->mm)
		{
			pr_warn("mm in %s-%d is null\n",p->comm,
				task_pid_nr(p));
			continue;
		}
		anon_record[index].tsk=p;
		anon_record[index].pagenum=get_mm_counter(p->mm, MM_ANONPAGES);
		index++;
		if(index==record_count)
		{
            sort(anon_record, index, sizeof(struct anon_page_dist),
                cmp_anon_page_by_size, NULL);
			index=record_count/2;//drop the 2nd half
		}
    }
	if(index!=record_count)
	{
            sort(anon_record, index, sizeof(struct anon_page_dist),
                cmp_anon_page_by_size, NULL);
	}
	for(i=0;i<index && i<ANON_PAGE3_TOP_NUM;i++)
	{
		if(i==0)
			pr_err("anon page distribution:\n");
		pr_err("\t%d:%08u KB anon_mem in %s-%d\n",i,
			anon_record[i].pagenum<<(PAGE_SHIFT-10),
			anon_record[i].tsk->comm,task_pid_nr(anon_record[i].tsk)
			);
	}
    rcu_read_unlock();	
}
struct mem_usage_dist
{
    const char *mem_type;
    unsigned long usage_kb;
};
#define UPDATE_MEM_USAGE(idx,type,value) \
    if(idx<record_count) \
    {\
        mem_usage[idx].mem_type=type; \
        mem_usage[idx].usage_kb=value; \
        idx++; \
    }
static int cmp_mem_usage_by_size(const void *a, const void *b)
{
    const struct mem_usage_dist  *pre = a, *next = b;
    int i=0;
    //sort by size
    i=pre->usage_kb>next->usage_kb?-1:pre->usage_kb<next->usage_kb;
    return i;
}   
static void _calculate_mem(unsigned long vmalloc_pagenum)
{
    struct sysinfo sysinfo;
    unsigned long slab_reclam;
    unsigned long slab_unreclam;
    unsigned long active,inactive;
    unsigned long direct_alloc;
    unsigned long anons,files;
    unsigned int record_count=vm_buf_size/sizeof(struct mem_usage_dist);
    struct mem_usage_dist *mem_usage=(struct mem_usage_dist *)count_vm_buf;
    unsigned int index=0;
    
    si_meminfo(&sysinfo);
    pr_err("MemTotal:%lu KB\n", sysinfo.totalram<<(PAGE_SHIFT - 10));
    pr_err("MemFree: %lu KB %lu%%\n", sysinfo.freeram<<(PAGE_SHIFT - 10),
        (sysinfo.freeram*100)/sysinfo.totalram);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,13,0)
    slab_unreclam=global_node_page_state(NR_SLAB_UNRECLAIMABLE);
    slab_reclam=global_node_page_state(NR_SLAB_RECLAIMABLE);    
#else
    slab_unreclam=global_page_state(NR_SLAB_UNRECLAIMABLE);
    slab_reclam=global_page_state(NR_SLAB_RECLAIMABLE);
#endif
    UPDATE_MEM_USAGE(index,"URSlab",slab_unreclam<<(PAGE_SHIFT-10));
    UPDATE_MEM_USAGE(index,"Vmalloc",vmalloc_pagenum<<(PAGE_SHIFT-10));
    UPDATE_MEM_USAGE(index,"PageTables", get_vm_state(NR_PAGETABLE)<<(PAGE_SHIFT-10));
    UPDATE_MEM_USAGE(index,"KernelStack",get_vm_state(NR_KERNEL_STACK_KB));
  
    active=global_node_page_state(NR_ACTIVE_ANON)+
            global_node_page_state(NR_ACTIVE_FILE);
    inactive=global_node_page_state(NR_INACTIVE_ANON)+
            global_node_page_state(NR_INACTIVE_FILE);
    direct_alloc=sysinfo.totalram-sysinfo.freeram-slab_unreclam-slab_reclam
            -vmalloc_pagenum-get_vm_state(NR_PAGETABLE)
            -get_vm_state(NR_BOUNCE)-active-inactive
            -global_node_page_state(NR_UNEVICTABLE);
    #ifndef CONFIG_VMAP_STACK
    direct_alloc-=(get_vm_state(NR_KERNEL_STACK_KB)>>(PAGE_SHIFT-10));
    #endif 
    anons=global_node_page_state(NR_ACTIVE_ANON)
        +global_node_page_state(NR_INACTIVE_ANON);
    files=global_node_page_state(NR_ACTIVE_FILE)
        +global_node_page_state(NR_INACTIVE_FILE);

    UPDATE_MEM_USAGE(index,"DirectAlloc",direct_alloc<<(PAGE_SHIFT-10));
    UPDATE_MEM_USAGE(index,"Anon",anons<<(PAGE_SHIFT-10));
    UPDATE_MEM_USAGE(index,"File",files<<(PAGE_SHIFT-10));
    UPDATE_MEM_USAGE(index,"SHM",sysinfo.sharedram<<(PAGE_SHIFT-10));
    sort(mem_usage,index, sizeof(struct mem_usage_dist),cmp_mem_usage_by_size, NULL);
    while(index>0)
    {
        pr_err("%08u KB(%03ld%%) consumed by %s\n",mem_usage[index-1].usage_kb,
           (mem_usage[index-1].usage_kb*100)/(sysinfo.totalram<<(PAGE_SHIFT - 10)),mem_usage[index-1].mem_type);
        index--;  
    }
    if(((anons*100)/sysinfo.totalram)>COUNT_ANON_PAGES_RATIO)
    {
        _count_anons();
    }       
}
static void oom_diag(void)
{
    unsigned long vmalloc_pagenum;
    if(in_atomic())return;
    mutex_lock(&oom_diag_mutex);
    if(!atomic_read(&skip_oom_diag))
    {   
        pr_err("start to do oom diagnostic\n");
        vmalloc_pagenum=_count_vmalloc();
        _count_slabs();
        _calculate_mem(vmalloc_pagenum);
    }
    mutex_unlock(&oom_diag_mutex);  
}
#if ENABLE_SELF_TEST
static ssize_t selftest_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    void *ptr;
    unsigned long num=0;
    unsigned long opt;
    
    if(kstrtoul(buf,0,&opt))
       return -EINVAL;
    do
    {
        if(opt==0)ptr=vmalloc(0x100000);
        else if(opt==1)ptr=kmalloc(0x2000,GFP_KERNEL);
        else if(opt==2)ptr=alloc_pages(GFP_HIGHUSER_MOVABLE,0);
        if(!ptr)break;
        num++;      
    }while(1);
    pr_info("success count=%ld\n",num);
    return count;    
}
#endif
static ssize_t oom_diag_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
    oom_diag();
    return 0;
}
static ssize_t buf_size_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    unsigned long buf_size=memparse(buf,NULL);
    ssize_t retval=count;
    if(buf_size>(10<<20))return -EINVAL;
    mutex_lock(&oom_diag_mutex);
    if(count_vm_buf)vfree(count_vm_buf);
    atomic_set(&skip_oom_diag,1);
    if(!buf_size)
    {
        vm_buf_size=buf_size;
        count_vm_buf=NULL;
        goto done;
    }
    count_vm_buf=vmalloc(buf_size);
    if(!count_vm_buf)
    {
        retval=-ENOMEM;
        vm_buf_size=0;
    }
    else
    {
        vm_buf_size=buf_size;
        atomic_set(&skip_oom_diag,0);
    }
done:   
    mutex_unlock(&oom_diag_mutex);
    return retval;
}
static ssize_t buf_size_show(struct kobject *kobj,struct kobj_attribute *pattr, char *buf)
{
    return snprintf(buf,PAGE_SIZE,"%u KB\n",vm_buf_size>>10);
}

static int oom_notifier(struct notifier_block *nb,
   unsigned long action, void *data)
{
    oom_diag();
    return NOTIFY_OK;
}
static struct notifier_block oom_nb = {
    .notifier_call = oom_notifier,
};
static int vmalloc_oom_notifier(struct notifier_block *nb,
   unsigned long action, void *data)
{
    pr_err("start to count vmalloc usage\n");
    (void)_count_vmalloc();
    return NOTIFY_OK;
}
static struct notifier_block vmalloc_oom_nb = {
    .notifier_call = vmalloc_oom_notifier,
};    
static struct kobject *oom_diag_kobj=NULL;
#if ENABLE_SELF_TEST
static struct kobj_attribute selftest_attr=__ATTR_WO(selftest);
#endif
static struct kobj_attribute buf_size_attr=__ATTR_RW(buf_size);
static struct kobj_attribute oom_diag_attr=__ATTR_RO(oom_diag);
static const struct attribute *oom_diag_attrs[] = {
#if ENABLE_SELF_TEST    
    &selftest_attr.attr,
#endif  
    &buf_size_attr.attr,
    &oom_diag_attr.attr,
    NULL,
};
static __init int oom_diag_init(void)
{
    int error=0;

    oom_diag_kobj=kobject_create_and_add("oom_diag", kernel_kobj);
    if(!oom_diag_kobj)
    {
        return -ENOMEM;
    }
    error = sysfs_create_files(oom_diag_kobj, oom_diag_attrs);
    if (error)
    {
        kobject_put(oom_diag_kobj);
        return error;
    }
    count_vm_buf=vmalloc(vm_buf_size);
    if(count_vm_buf)
    {
        register_oom_notifier(&oom_nb);
        register_vmap_purge_notifier(&vmalloc_oom_nb);		
    }
    else
    {
        error=-ENOMEM;
    }
    return error;
}

device_initcall(oom_diag_init);
#endif