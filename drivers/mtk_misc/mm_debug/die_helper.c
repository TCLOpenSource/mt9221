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

#define pr_fmt(fmt) "die_helper: " fmt
//#define DEBUG 1
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/slub_def.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/kasan.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/kdebug.h>
#include <linux/rmap.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <asm/sections.h>
#include <linux/memblock.h>
#include <linux/kallsyms.h>
#include "utils.h"
#include "mm/slab.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,15,0)
#include <linux/sched/signal.h>
#endif

#define DUMP_REG_AROUND_BYTES (128)
#define IS_ADDR_VALID(x) (_get_phys_addr((unsigned long)(x))!=PHYS_ADDR_MAX)

#define DUMP_ON_PANIC (1)

static atomic_t on_die=ATOMIC_INIT(0);
static struct pt_regs saved_die_regs;
#ifdef MODULE
typedef void * (*pfn_fixup_red_left)(struct kmem_cache *s, void *p);
typedef struct vm_struct * (*pfn_find_vm_area)(const void *addr);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,11,0)
typedef void (*pfn_rmap_walk)(struct page *page, struct rmap_walk_control *rwc);
#else
typedef int (*pfn_rmap_walk)(struct page *page, struct rmap_walk_control *rwc);
#endif
#ifdef CONFIG_KASAN
typedef void (*pfn_kasan_enable_current)(void);
typedef void (*pfn_kasan_disable_current)(void);
static pfn_kasan_enable_current  kasan_enable_current_pfn=NULL;
static pfn_kasan_disable_current kasan_disable_current_pfn=NULL;
#endif
static pfn_fixup_red_left fixup_red_left_pfn=NULL;
static pfn_find_vm_area find_vm_area_pfn=NULL;
static pfn_rmap_walk rmap_walk_pfn=NULL;
static int _kernel_sym_init(void)
{
    fixup_red_left_pfn=(pfn_fixup_red_left)kallsyms_lookup_name("fixup_red_left");
    pr_err("fixup_red_left_pfn=%pS\n",fixup_red_left_pfn);
    if(__module_address((unsigned long)fixup_red_left_pfn))goto fixup_red_left_err;
    find_vm_area_pfn=(pfn_find_vm_area)kallsyms_lookup_name("find_vm_area");
    pr_err("find_vm_area_pfn=%pS\n",find_vm_area_pfn);
    if(__module_address((unsigned long)find_vm_area_pfn))goto find_vm_area_err;
    rmap_walk_pfn=(pfn_rmap_walk)kallsyms_lookup_name("rmap_walk");
    pr_err("rmap_walk_pfn=%pS\n",rmap_walk_pfn);    
    if(__module_address((unsigned long)rmap_walk_pfn))goto rmap_walk_err;

#ifdef CONFIG_KASAN
    kasan_enable_current_pfn=(pfn_kasan_enable_current)kallsyms_lookup_name
                ("kasan_enable_current");
    pr_err("kasan_enable_current_pfn=%pS\n",kasan_enable_current_pfn);
    if(__module_address((unsigned long)kasan_enable_current_pfn))
        goto kasan_enable_current_err;  
    kasan_disable_current_pfn=(pfn_kasan_disable_current)kallsyms_lookup_name
                ("kasan_disable_current");
    pr_err("kasan_disable_current_pfn=%pS\n",kasan_disable_current_pfn);
    if(__module_address((unsigned long)kasan_disable_current_pfn))
        goto kasan_disable_current_err;     
#endif    
    return 0;
#ifdef CONFIG_KASAN
kasan_disable_current_err:
    kasan_disable_current_pfn=NULL; 
kasan_enable_current_err:
    kasan_enable_current_pfn=NULL;
#endif    
rmap_walk_err:
    rmap_walk_pfn=NULL;
find_vm_area_err:
    find_vm_area_pfn=NULL;
fixup_red_left_err:
    fixup_red_left_pfn=NULL;

    return 1;
}
void *fixup_red_left(struct kmem_cache *s, void *p)
{
    if(fixup_red_left_pfn)return fixup_red_left_pfn(s,p);
    return NULL;
}
struct vm_struct * find_vm_area(const void *addr)
{
    if(find_vm_area_pfn)return find_vm_area_pfn(addr);
    return NULL;
}
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,11,0)
void rmap_walk(struct page *page, struct rmap_walk_control *rwc)
#else
int rmap_walk(struct page *page, struct rmap_walk_control *rwc)
#endif
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,11,0)
    if(rmap_walk_pfn)rmap_walk_pfn(page,rwc);
#else
    if(rmap_walk_pfn)return rmap_walk_pfn(page,rwc);
    return 0;
#endif  
}
#ifdef CONFIG_KASAN
void kasan_enable_current(void)
{
    if(kasan_enable_current_pfn)kasan_enable_current_pfn();
}
void kasan_disable_current(void)
{
    if(kasan_disable_current_pfn)kasan_disable_current_pfn();
}
#endif
#else
static inline int _kernel_sym_init(void){return 0;}
#endif
static  void *_show_slub_info(void *object,phys_addr_t pa_addr)
{
    struct page *page=NULL,*head_page=NULL;
    struct kmem_cache *s=NULL;

    if((pa_addr==PHYS_ADDR_MAX) ||
    !pfn_valid(PHYS_PFN(pa_addr)))
    {
        pr_emerg("%px is invalid due to phys %pa\n",object,&pa_addr);
        return NULL;
    }
    page=phys_to_page(pa_addr);         
    if(IS_ADDR_VALID(page))
    {
        head_page=compound_head(page);
        if(!IS_ADDR_VALID(head_page))
        {
            pr_emerg("head_page %px for pa %pa is invalid\n",
                        head_page,&pa_addr);
            return NULL;
        }
        else
        {   
            pr_emerg("%pa:Flags %#lx(%pGp)\n",&pa_addr,head_page->flags, 
                        &head_page->flags);
        }                       
    }
    else
    {
        pr_emerg("page %px for pa %pa is invalid\n",page,&pa_addr);
        return NULL;
    }
    if(head_page && PageSlab(head_page))
    {
        s=head_page->slab_cache;    
        if (s && 
               IS_ADDR_VALID(s))
            {
                object=nearest_obj(s,head_page,object);
                pr_emerg("object %px was from slab %s(%x)\n",
                object,cache_name(s),s->size);
                #if defined(CONFIG_SLUB_DEBUG)
                pr_emerg("object %px is free?%c\n",object,
                   obj_in_free(s,object,head_page)?'Y':'N');
                if(s->flags & SLAB_STORE_USER)
                {
                    struct track *slub_alloc,*slub_free;
                    slub_alloc=get_slub_track(s,object,TRACK_ALLOC);
                    slub_free=get_slub_track(s,object,TRACK_FREE);
                    if(IS_ADDR_VALID(slub_alloc))
                    {
                        pr_emerg("object %px allocated when %lu,pid %d on c%d,trace:\n",
                        object,slub_alloc->when,
                        slub_alloc->pid,slub_alloc->cpu);
                        show_slub_track(slub_alloc,"die_helper: ",4);
                    }
                    if(IS_ADDR_VALID(slub_free))
                    {
                        pr_emerg("object %px freed when %lu,pid %d on c%d,trace:\n",
                        object,slub_free->when,
                        slub_free->pid,slub_free->cpu);
                        show_slub_track(slub_free,"die_helper:  ",4);
                    }
                }
                #endif
                return object;
            }
            else
            {
                pr_emerg("%px was allocated from slab,but slab %px is corrupt\n",
                    object,s);              
            }           
    }
    return NULL;
}
static void *_get_prev_object(void *object)
{
    struct page *head_page=NULL,*prev_page=NULL;
    void *prev_obj=NULL;
    struct kmem_cache *s;
    phys_addr_t pa_addr;
    
    head_page=virt_to_head_page(object);
    s=head_page->slab_cache;
    prev_obj=object-s->size;
    pa_addr=_get_phys_addr((unsigned long)prev_obj);
    if((pa_addr!=PHYS_ADDR_MAX) 
            && pfn_valid(PHYS_PFN(pa_addr)))
    {
        prev_page=virt_to_head_page(prev_obj);
        if(IS_ADDR_VALID(prev_page))
        {
            if(PageSlab(prev_page))
            {
                if(IS_ADDR_VALID(prev_page->slab_cache))
                {
                    if(prev_page->slab_cache!=s)//different slab
                    {
                        prev_obj=nearest_obj(prev_page->slab_cache,prev_page,object);
                    }
                    return prev_obj;
                }
                else
                {
                    pr_emerg("%px was allocated from slab,but slab %px is corrupt\n",
                        prev_obj,prev_page->slab_cache);                            
                }   
            }
            else
                    pr_emerg("%px is not from slab\n",
                            prev_obj);                      
        }   
    }
    else
        pr_emerg("%px is not from slab\n",
                            prev_obj);
    return NULL;                            
}
static void _dump_reg_info(unsigned long addr,const char *name)
{
    char buf[16];
    unsigned long tmp=0;
    unsigned long val=0;
    int i=0;
    phys_addr_t pa_addr=0;
    void *object,*prev_obj;
    
    pa_addr=_get_phys_addr(addr);
    if(pa_addr!=PHYS_ADDR_MAX)
    {
        pr_emerg("%s: va %#lx-pa %pa:\n",name,addr,&pa_addr);
        
        //dump memory content
        tmp=addr/sizeof(buf);
        tmp*=sizeof(buf);
        
        for(tmp-=DUMP_REG_AROUND_BYTES;
            tmp<addr+DUMP_REG_AROUND_BYTES;
            tmp+=sizeof(buf))
        {
            if(_get_phys_addr(tmp)==PHYS_ADDR_MAX)continue;
            if(!probe_kernel_read(buf,(const void *)tmp,sizeof(buf)))
            {
                pr_cont("%#lx:",tmp);
                for(i=0;i<sizeof(buf)/sizeof(long);i++)
                {
                    //if(0==i%sizeof(long))pr_cont(" ");
                    //pr_cont("%02x",buf[i]);
                    memcpy(&val,buf+i*sizeof(long),sizeof(long));
                #if BITS_PER_LONG>=64
                    if(core_kernel_text(tmp))
                        pr_cont("%08lx %08lx ",val&0xffffffff,val>>32);                        
                    else
                        pr_cont("%016lx ",val);
                #else
                    pr_cont("%08lx ",val);
                #endif
                }
                for(i=0;i<sizeof(buf);i++)
                {
                    if(0==i%sizeof(long))pr_cont(" ");
                    if((buf[i]>=0x20) &&
                        (buf[i]<=0x7e))
                        pr_cont("%c",buf[i]);
                    else
                        pr_cont(".");   
                }
                pr_cont("\n");              
            }               
        }
        //check slab
        object=_show_slub_info((void *)addr,pa_addr);
        if(object)
        {
            prev_obj=_get_prev_object(object);
            if(prev_obj)
                _show_slub_info(prev_obj,
                    _get_phys_addr((unsigned long)prev_obj));
        }
    }
}
static void _dump_available_mem_onpanic(struct pt_regs *regs)
{
    unsigned int i;
    
    kasan_disable_current();
#if BITS_PER_LONG>=64
//nested die will hang @64bit,so we print all regs to 
//get more information 
    for (i = 0; i < 30; i+=2) {
        pr_emerg("X%02u: %016llx X%02u: %016llx\n",i,
        regs->regs[i],i+1,regs->regs[i+1]);
    }
    pr_emerg("LR:%pS(%px)\n",(void *)regs->regs[30],(void *)regs->regs[30]);
    pr_emerg("PC:%pS(%px)\n",(void *)regs->pc,(void *)regs->pc);    
    pr_emerg("SP:%pS(%px)\n",(void *)regs->sp,(void *)regs->sp);    

    for (i = 0; i < 30; i++) {
        char name[11];
        snprintf(name, sizeof(name), "X%u's info", i);
        _dump_reg_info(regs->regs[i],name);
    }
    _dump_reg_info(regs->sp,"SP's info");
    _dump_reg_info(regs->regs[30],"LR's info");
    _dump_reg_info(regs->pc,"PC's info");
#else
    
    for (i = 0; i < 10; i++) {
        char name[11];
        snprintf(name, sizeof(name), "R%u's info", i);
        _dump_reg_info(regs->uregs[i],name);
    }
    _dump_reg_info(regs->ARM_fp,"FP's info");
    _dump_reg_info(regs->ARM_ip,"IP's info");
    _dump_reg_info(regs->ARM_sp,"SP's info");
    _dump_reg_info(regs->ARM_lr,"LR's info");
    _dump_reg_info(regs->ARM_pc,"PC's info");   
#endif  
    kasan_enable_current(); 
}
#if 0
static void _search_task_by_mm(struct mm_struct *mm)
{
    struct task_struct *g, *p;
    rcu_read_lock();
    for_each_process_thread(g, p) {
        if(p->mm==mm)
            pr_emerg("%s-%d:got mm %px\n",p->comm,p->pid,mm);
    }
    rcu_read_unlock();
}
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,11,0)
static bool rmap_one(struct page *page, struct vm_area_struct *vma,
     unsigned long addr, void *arg)
#else
static int rmap_one(struct page *page, struct vm_area_struct *vma,
     unsigned long addr, void *arg)
#endif
{
    phys_addr_t pa;
    pa=page_to_phys(page);
    pr_emerg("%s:physical %pa is mapped as %lx in vma %px mm %px\n",
    (char *)arg,&pa,addr,vma,vma->vm_mm);
    pr_emerg("\tvma %px:flags %lx,prot %lx",vma,vma->vm_flags,
    (unsigned long)pgprot_val(vma->vm_page_prot));
    _search_task_by_mm(vma->vm_mm);
    return 1;
}
#endif
static void _dump_reg_rmap(unsigned long addr,const char *name)
{
    struct module *mod;
    phys_addr_t pa_addr=0;  
#if DUMP_ON_PANIC  
    struct vm_struct *vm;   
#endif
#if 0
    struct page *page;
    struct rmap_walk_control rc=
    {
        .arg=(void *)name,
        .rmap_one = rmap_one,
    };
#endif
    //pr_emerg("%s-%s\n",__func__,name);
    pa_addr=_get_phys_addr(addr);
    if((pa_addr!=PHYS_ADDR_MAX) && pfn_valid(PHYS_PFN(pa_addr)))
    {
        pr_emerg("%s\n",name);
        //show module layout if involved
        mod=__module_address(addr);
        if(mod)
        {
            if(within_module_init(addr,mod))
            {
                pr_emerg("in module %s,init range %px-%px\n",mod->name,
            mod->init_layout.base,
            mod->init_layout.base+mod->init_layout.size);
            }
            else
            {
                pr_emerg("in module %s,range %px-%px\n",mod->name,
            mod->core_layout.base,
            mod->core_layout.base+mod->core_layout.size);
            }
        }
#if DUMP_ON_PANIC
//check vmalloc
        if(((addr>=MODULES_VADDR) && (addr<MODULES_END))
            || ((addr>=VMALLOC_START) && (addr<VMALLOC_END)))
        {
            vm=find_vm_area((void *)addr);
            if(vm && (_get_phys_addr((unsigned long)vm)!=PHYS_ADDR_MAX))
                pr_emerg("vmalloc addr %lx allocated by \n\
            \t%pS in range %lx-%lx from phys %pa\n",
            addr,vm->caller,(unsigned long)vm->addr,(unsigned long)vm->addr+vm->size,&vm->phys_addr);
        }
#endif
#if 0      
        page=phys_to_page(pa_addr);
        rmap_walk(page,&rc);
#endif
    }
}
static void _dump_reg_rmap_onpanic(struct pt_regs *regs)
{
    int i;
    char name[30];
    pr_emerg("%s\n",__func__);
#if BITS_PER_LONG>=64
    for (i = 0; i < 30; i++) {
        snprintf(name, sizeof(name), "X%u's rmap-%lx", 
        i,(unsigned long)regs->regs[i]);
        _dump_reg_rmap(regs->regs[i],name);
    }
    snprintf(name, sizeof(name), "SP's rmap-%llx",regs->sp); 
    _dump_reg_rmap(regs->sp,name);
#else
    for (i = 0; i < 10; i++) {
        snprintf(name, sizeof(name), "R%u's rmap-%lx", 
        i,(unsigned long)regs->uregs[i]);
        _dump_reg_rmap(regs->uregs[i],name);
    }
    snprintf(name, sizeof(name),"FP's rmap-%lx",regs->ARM_fp); 
    _dump_reg_rmap(regs->ARM_fp,name);
    snprintf(name, sizeof(name),"IP's rmap-%lx",regs->ARM_ip);  
    _dump_reg_rmap(regs->ARM_ip,name);
    snprintf(name, sizeof(name),"FP's rmap-%lx",regs->ARM_fp);
    _dump_reg_rmap(regs->ARM_fp,name);
    snprintf(name, sizeof(name),"SP's rmap-%lx",regs->ARM_sp);  
    _dump_reg_rmap(regs->ARM_sp,name);
#endif      
}
static void _dump_misc_info(void)
{
#ifndef MODULE  
    struct memblock_region *region;
    phys_addr_t start,end;
    
    pr_emerg("kernel text:%px-%px\n",_text,_etext);
    pr_emerg("kernel init:%px-%px\n",__init_begin,__init_end);
    pr_emerg("System RAM:\n");
    for_each_memblock(memory, region) 
    {
        start=PFN_PHYS(memblock_region_memory_base_pfn(region));
        end=PFN_PHYS(memblock_region_memory_end_pfn(region))-1;
        pr_emerg("\t%s:%pa-%pa\n",
        memblock_is_nomap(region)?"reserved":"memory",
        &start,&end);
    }
#endif
}
static int die_notify(struct notifier_block *nb,
                    unsigned long val, void *data)
{
    struct die_args *pargs;
    if(atomic_add_return(1,&on_die)==1)
    {
        if(DIE_OOPS==val)
        {
            pargs=(struct die_args *)data;
            memcpy(&saved_die_regs,pargs->regs,sizeof(saved_die_regs));
#if !DUMP_ON_PANIC  
    _dump_misc_info();
    _dump_available_mem_onpanic(&saved_die_regs);           
    _dump_reg_rmap_onpanic(&saved_die_regs);
#endif          
        }
    }
    else
    {
        pr_emerg("die_helper is ongoing %d\n",atomic_read(&on_die));    
    }
    atomic_dec(&on_die);
    return NOTIFY_OK;
}
//check if slab,if so,dump previous/after slub objects and caller
//check rmap and kernel map(vmalloc) for this physical address
//dump memory content of each possiable addr pointed by register

static struct notifier_block die_helper_nb = {
    .notifier_call = die_notify,
     /* we need to be notified first */ 
    .priority = 0x7fffffff
};
static int die_helper_on_panic(struct notifier_block *self, unsigned long v, void *p)
{
#if DUMP_ON_PANIC   
    _dump_misc_info();
    _dump_available_mem_onpanic(&saved_die_regs);           
    _dump_reg_rmap_onpanic(&saved_die_regs);
#endif  
    return 0;
}

static struct notifier_block die_helper_panic_nb = {
    .notifier_call = die_helper_on_panic,
    /* we need to be notified first */
    .priority = 0x7fffffff
};
static char b_die_helper=1;
static __init int die_helper_init(void)
{
    if(!b_die_helper)return 0;

    if(_kernel_sym_init())
    {
        pr_err("can not find symbol...disabled\n");
        return 0;
    }
    memset(&saved_die_regs,0,sizeof(saved_die_regs));
    atomic_notifier_chain_register(&panic_notifier_list,
       &die_helper_panic_nb);
    return register_die_notifier(&die_helper_nb);
}
static void __exit die_helper_exit(void)
{
    if(!b_die_helper)return;
    unregister_die_notifier(&die_helper_nb);
}
static int __init setup_die_helper(char *str)
{
    if (*str++ != '=' || !*str)
        return -EINVAL;
    if(!strcmp(str,"on") || *str=='1')
        b_die_helper=1;
    return 1;
}
__setup("die_helper", setup_die_helper);
module_init(die_helper_init);
module_exit(die_helper_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mediatek");

