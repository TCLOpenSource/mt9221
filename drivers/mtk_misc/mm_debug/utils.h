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

#ifndef MM_DEBUG_UTILS_H
#define MM_DEBUG_UTILS_H 1
#include <linux/version.h>
#if BITS_PER_LONG<64
#include <asm/uaccess.h>
#endif
#include <linux/bit_spinlock.h>

#ifndef PHYS_ADDR_MAX
#define PHYS_ADDR_MAX (ULONG_MAX)
#endif
#if defined(CONFIG_SLUB_DEBUG)
#define for_each_object_in_page(p,page,s,addr) \
    for (p = nearest_obj(s, page,addr);\
        p < (addr + (page->objects) * s->size); \
        p += s->size)//steal from for_each_object in slub.c
/*
 * Tracking user of a slab.
 */
#define TRACK_ADDRS_COUNT 16
struct track {
    unsigned long addr; /* Called from address */
#ifdef CONFIG_STACKTRACE
    unsigned long addrs[TRACK_ADDRS_COUNT]; /* Called from address */
#endif
    int cpu;        /* Was running on cpu */
    int pid;        /* Pid context */
    unsigned long when; /* When did the operation occur */
};

enum track_item { TRACK_ALLOC, TRACK_FREE };

struct track *get_slub_track(struct kmem_cache *s, void *object,
    enum track_item alloc);
static inline phys_addr_t _get_phys_addr(unsigned long addr);
void show_slub_track(struct track  *track,const char *prefix,int spaces);
static inline void *get_freepointer(struct kmem_cache *s, void *object)
{
    phys_addr_t pa_addr=_get_phys_addr((unsigned long)(object + s->offset));

    if((pa_addr!=PHYS_ADDR_MAX) 
            && pfn_valid(PHYS_PFN(pa_addr)))
        return *(void **)(object + s->offset);
    return NULL;    
}
static inline bool obj_in_free(struct kmem_cache *s, void *object,struct page *page)
{
    void *p;
    if(!bit_spin_trylock(PG_locked, &page->flags))//can not lock,return N
    {
        pr_err("can not lock page to check %px\n",object);
        return false;
    }
    for (p = page->freelist; p; p = get_freepointer(s, p))//correction
    {
        if(p==object)
        {
            bit_spin_unlock(PG_locked, &page->flags);
            return true;
        }   
    }
    bit_spin_unlock(PG_locked, &page->flags);   
    return false;
}
#endif

static inline phys_addr_t _get_phys_addr(unsigned long addr)
{
    unsigned long flags, paddr;
#if BITS_PER_LONG<64    
    unsigned int ua_flags;
#endif  
    local_irq_save(flags);
#if BITS_PER_LONG>=64
    asm volatile (
            "  AT S1E1R, %1\n"
            "  ISB\n"
            "  MRS %0, PAR_EL1\n"
            : "=r" (paddr) : "r" (addr) : "cc");
#else
    if(addr<TASK_SIZE)
        ua_flags = uaccess_save_and_enable();
    asm volatile (
            "  mcr p15, 0, %1, c7, c8, 0\n"
            "  ISB\n"
            "  mrc p15, 0, %0, c7, c4, 0\n"
            : "=r" (paddr) : "r" (addr) : "cc");
    if(addr<TASK_SIZE)
        uaccess_restore(ua_flags);          
#endif
    local_irq_restore(flags);
    pr_debug("PAR is %lx\n",paddr);
    if (paddr & 1)
        return PHYS_ADDR_MAX;
    if(sizeof(paddr)>4)
    {
        paddr &=0xffffffffffff;//bit 47:12
    }
    
    //if(out_paddr)*out_paddr=paddr;
    return (paddr &(PAGE_MASK))|(addr &(PAGE_SIZE-1));
    //if(pfn_valid(__phys_to_pfn(paddr))
    //  return phys_to_page(paddr);
    //return NULL;
}
#endif
