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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_cma_mpool.c
/// @brief  CMA mpool interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <linux/version.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/dma-contiguous.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/swap.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include <linux/vmalloc.h>

#include <ion.h>
#include "mdrv_types.h"
#include "mst_devid.h"
#include "mdrv_system.h"
#include "mdrv_miu_protect.h"
#include "mdrv_miu.h"
#include "mhal_miu.h"

#include <linux/profile.h>//profile_munmap need this header file
#include <linux/ptrace.h>//ptrace_may_access function need this header file

#ifdef CONFIG_MP_MMA_ENABLE
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define PHYSICAL_START_INIT     UL(-1)
#define PHYSICAL_END_INIT       0
#define INVALID_MIU          0xFF
#define INVALID_PID          0
#define MAX_ALLOC_TRY        30
#define MOD_CMAPOOL_DEVICE_COUNT     1
#define FREE_FROM_BEGINNING       0x01UL
#define FREE_FROM_END             0x10UL
#define MOD_MIUPROTECT_NAME             "miuprotect"
#define ION_DEVICE_NAME              "/dev/ion"
#define MIU_PROTECT_ENABLE 1
#define MIU_PROTECT_DISABLE 0
#define MIU_BLOCK_NUM     MIU_MAX_PROTECT_BLOCK

#define MCMA_BUG_ON(cond)  \
do { \
      if(cond) {\
        printk(CMA_ERR "MCMA_BUG in %s @ %d \n", __FUNCTION__, __LINE__); \
	  }\
      BUG_ON(cond); \
   } while(0)

#define MCMA_CUST_ERR(fmt, args...) printk(CMA_ERR "MCMA error %s:%d " fmt,__FUNCTION__,__LINE__,## args)
#define MCMA_CUST_WARNING(fmt, args...) printk(CMA_ERR "MCMA warning %s:%d " fmt,__FUNCTION__,__LINE__,## args)

//#define DBUG_UTOPIA_TO_KERNEL_CODE 1

#ifdef DBUG_UTOPIA_TO_KERNEL_CODE
#define MCMA_CUST_DEBUG(fmt, args...) printk(CMA_ERR "MCMA debug %s:%d " fmt,__FUNCTION__,__LINE__,## args)
#else
#define MCMA_CUST_DEBUG(fmt, args...)
#endif

struct MIU_ProtectRanges
{
    unsigned char miu;
    MIU_PROTECT_BLOCK_STATUS miuBlockStatus[MIU_BLOCK_NUM];

    unsigned int krange_num;
    struct list_head list_head;
    struct mutex lock;
};

static inline void phy_to_MiuOffset(unsigned long phy_addr, unsigned int *miu, unsigned long *offset)
{
    *miu = INVALID_MIU;

    if(phy_addr >= ARM_MIU2_BUS_BASE)
    {
        *miu = 2;
        *offset = phy_addr - ARM_MIU2_BUS_BASE;
    }
    else if(phy_addr >= ARM_MIU1_BUS_BASE)
    {
        *miu = 1;
        *offset = phy_addr - ARM_MIU1_BUS_BASE;
    }
    else if(phy_addr >= ARM_MIU0_BUS_BASE)
    {
        *miu = 0;
        *offset = phy_addr - ARM_MIU0_BUS_BASE;
    }
	else
		printk(CMA_ERR "\033[35mFunction = %s, Line = %d, Error, Unknown MIU, for phy_addr is 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, phy_addr);
}


static struct MIU_ProtectRanges glob_miu_kranges[KERN_CHUNK_NUM]; //record kernel protect ranges on 3 MIUs
static DEFINE_MUTEX(heap_info_lock);
static struct list_head heap_info_list; //record heap related info
static unsigned char *kernal_protect_client_id = NULL;
static atomic_t kprotect_enabled = ATOMIC_INIT(1);


int idleBlockIndx(struct MIU_ProtectRanges * pranges)
{
    int index = 0;

    for(index = 0; index < MIU_BLOCK_NUM; ++index)
    {
        if(pranges->miuBlockStatus[index] == MIU_BLOCK_IDLE)
        {
            return index;
        }
    }

    return -1;
}


void miuprotect_dumpKRange(unsigned int miu)
{


//#ifdef CMA_POOL_KERNEL_PROTECT_DUMP
    struct MIU_ProtectRanges *pranges = &(glob_miu_kranges[miu]);
    MIU_ProtectRange *range = NULL;

    mutex_lock(&pranges->lock);
    list_for_each_entry(range, &pranges->list_head, list_node)
    {
        printk(CMA_ERR "range start 0x%lX to 0x%lX, length 0x%lX miublock_index %d\n",
            range->start_pa, (range->start_pa+range->length), range->length, range->miuBlockIndex);
    }
    mutex_unlock(&pranges->lock);
//#endif

}


static bool _miu_kernel_protect(unsigned char miuBlockIndex, unsigned char *pu8ProtectId,
    unsigned long start, unsigned long end, int flag)
{
    bool ret = true;

    if(atomic_read(&kprotect_enabled) > 0)
        ret = MDrv_MIU_Kernel_Protect(miuBlockIndex, kernal_protect_client_id, start, end, flag);
    else
        printk(CMA_ERR "ignore kernel protect\n");

    return ret;
}


//???ﻹ??Ҫ??һЩ???????飬 �� ??֤????��?Ĳ????Ƿ??????ǻ?????Ҫ????
//????????????protect??????2??,
//????ʱ?????϶????ٺ????е?protect??ĳ????��???ĲŶ?

/* when alloc from cma heap, call this API to deleteKRange of this allocted buffer */
 int miuprotect_deleteKRange(unsigned long buffer_start_pa, unsigned long buffer_length)
{

    struct MIU_ProtectRanges * pranges ;
    MIU_ProtectRange * range,  * r_front = NULL, * r_back= NULL;
    MIU_ProtectRange old;
    unsigned long r_front_len = 0, r_back_len = 0;
    int miuBlockIndex = -1;
    bool find = false, protect_ret = false;
    int ret = 0;
    unsigned int miu;
    unsigned long offset;


    if(buffer_length == 0)
        return 0;

    phy_to_MiuOffset(buffer_start_pa,&miu,&offset);
    
    pranges = &(glob_miu_kranges[miu]);

    /*
         * kernel protect range( before allocate buffer)
         *
         * |--------------------------------|
         *
         * kernel protect range(buffer location in this range, after buffer allocated)
         *  r_front        allocated buffer    r_back
         *
         * |------|=============|-------|
         *
         * case: r_front = 0; r_back = 0; r_front=r_back=0;
         */
    mutex_lock(&pranges->lock);
    list_for_each_entry(range, &pranges->list_head, list_node)
    {
        if((buffer_start_pa >= range->start_pa)
            && ((buffer_start_pa+buffer_length) <= (range->start_pa+range->length)))
        {
            find = true;
            old.start_pa = range->start_pa;
            old.length = range->length;
            old.miuBlockIndex = range->miuBlockIndex;
            break;
        }
    }

    if(!find)
    {
       ret = -EINVAL;
       MCMA_CUST_ERR("not find the buffer: start_pa %lx length %lu\n", buffer_start_pa, buffer_length);
       goto DELETE_KRANGE_DONE;
    }

    r_front_len = buffer_start_pa - range->start_pa;
    r_back_len = range->start_pa + range->length - (buffer_start_pa + buffer_length);

    if((r_front_len != 0) && (r_back_len != 0))
    {
        miuBlockIndex = idleBlockIndx(pranges);
        if(miuBlockIndex < 0)
        {
           ret = -ENXIO;
           MCMA_CUST_ERR("no idle miu protect block in miu %d\n", (int)miu);
           goto DELETE_KRANGE_DONE;
        }

        r_back = (MIU_ProtectRange *)kzalloc(sizeof(MIU_ProtectRange), GFP_KERNEL);
        if(!r_back)
        {
           ret = -ENOMEM;
           printk(CMA_ERR "no memory\n");
           goto DELETE_KRANGE_DONE;
        }

        r_front = range;
        r_front->length = r_front_len;

        r_back->start_pa = buffer_start_pa + buffer_length;
        r_back->length = r_back_len;
        r_back->miuBlockIndex = miuBlockIndex;
        INIT_LIST_HEAD(&r_back->list_node);
        list_add(&r_back->list_node, &r_front->list_node);
        pranges->krange_num++;
    }
    else if(r_front_len != 0) //and (r_back_len == 0)
    {
        r_front = range;
        r_front->length = r_front_len;
    }
    else if(r_back_len != 0) //and (r_front_len == 0)
    {
        r_back = range;
        r_back->start_pa = buffer_start_pa + buffer_length;
        r_back->length = r_back_len;
    }
    else //((r_front_len == 0) && (r_back_len == 0))
    {
        list_del(&range->list_node);
        kfree(range);
        pranges->krange_num--;
    }

    protect_ret = _miu_kernel_protect(old.miuBlockIndex, kernal_protect_client_id, old.start_pa,
        old.start_pa + old.length, MIU_PROTECT_DISABLE);
    MCMA_BUG_ON(!protect_ret);
    pranges->miuBlockStatus[old.miuBlockIndex] = MIU_BLOCK_IDLE;

    if(r_front)
    {
        protect_ret = _miu_kernel_protect(r_front->miuBlockIndex, kernal_protect_client_id,
            r_front->start_pa, r_front->start_pa+r_front->length, MIU_PROTECT_ENABLE);
        MCMA_BUG_ON(!protect_ret);
        pranges->miuBlockStatus[r_front->miuBlockIndex] = MIU_BLOCK_BUSY;
    }

    if(r_back)
    {
        protect_ret = _miu_kernel_protect(r_back->miuBlockIndex, kernal_protect_client_id,
            r_back->start_pa, r_back->start_pa+r_back->length, MIU_PROTECT_ENABLE);
        MCMA_BUG_ON(!protect_ret);
        pranges->miuBlockStatus[r_back->miuBlockIndex] = MIU_BLOCK_BUSY;
    }

DELETE_KRANGE_DONE:
    mutex_unlock(&pranges->lock);
    return ret;

}



//???ﻹ??Ҫ??һЩ???????飬 �� ??֤????��?Ĳ????Ƿ??????ǻ?????Ҫ????
//????????????protect??????2??,
//????ʱ?????϶????ٺ????е?protect??ĳ????��???ĲŶ?

/*

to be discuss:
Ҫ?????????ıȽϺ???:
legacy cma heap??ion cma hep????ά???Լ?????Ҫadd/delete KRange.
ע????ʵ??(????ͨ??cma cache,delay free)??memory??free/allocҲ??
һ????Ҫadd/delete KRange??????ά????cma heap��??,????cma heap???Լ?Ҫά?????Լ????Ѿ???alloc?ߵ?memory????Щ

miuprotect moduleֻ????Ҫ????ʲô????ʲô???????ж? index?Ƿ񹻣??Լ???ͬ??protect?Ƿ?????idx
*/
/* when free to cma heap, call this API to add KRange of this allocted buffer */
int miuprotect_addKRange(unsigned long buffer_start_pa, unsigned long buffer_length)
{
	
    unsigned long offset;
    struct MIU_ProtectRanges *pranges;
    unsigned int miu;
    MIU_ProtectRange *r_prev = NULL, *r_next= NULL;
    MIU_ProtectRange *range;
    int miuBlockIndex = -1;
    bool protect_ret = false;
    int ret = 0;

#ifdef CONFIG_MP_CMA_PATCH_DEBUG_STATIC_MIU_PROTECT
	/*
 	 * When CONFIG_MP_CMA_PATCH_DEBUG_STATIC_MIU_PROTECT is enabled,
 	 * range shoule never be changed
 	 */
	return 0;
#endif

    if(buffer_length == 0)
        return 0;

    phy_to_MiuOffset(buffer_start_pa,&miu,&offset);
    pranges = &(glob_miu_kranges[miu]);

    /*
         * kernel protect range (before freed buffer)
         *      r_prev       allocated buffer     r_next
         * |-------------|====================|------------|
         *
         * kernel protect range(freed buffer location in this range)
         *   r_prev   freed buffer    r_next
         * |--------|?-------------?|-------|
         *
    */
    mutex_lock(&pranges->lock);
    list_for_each_entry(range, &pranges->list_head, list_node)	// find this miu all kernel_protect setting(range)
    {
        if((range->start_pa + range->length) <= buffer_start_pa)
        {
			//printk("\033[35mFunction = %s, Line = %d, find r_prev form 0x%lX to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, range->start_pa, (range->start_pa + range->length));
            r_prev = range;
            continue;	// should be continue, we are going to find a nearest one k_range before this buffer
        }
    }

    if(r_prev)	// find a kernel_protect range before this buffer
    {
        if(!list_is_last(&r_prev->list_node,&pranges->list_head))
        {
            r_next = container_of(r_prev->list_node.next, MIU_ProtectRange, list_node);		// if prev_krange is not the last one, the next one krange will be r_next
			//printk("\033[35mFunction = %s, Line = %d, find r_next form 0x%lX to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, r_next->start_pa, (r_next->start_pa + r_next->length));
        }
    }
    else		// no kernel_protect range before this buffer ==> all k_range is behind this buffer
    {
        if(list_empty(&pranges->list_head))
            r_next = NULL;
        else
            r_next = list_first_entry(&pranges->list_head, MIU_ProtectRange, list_node);	// r_next will be first krange
    }

    //till now, find the prev range and next range of buffer freed
    if(r_prev && r_next)
    {
        if(((r_prev->start_pa + r_prev->length) == buffer_start_pa)
            && ((buffer_start_pa + buffer_length) == r_next->start_pa))	// the buffer is just the hole between r_prev and r_next
        {
			// disable r_prev
            protect_ret = _miu_kernel_protect(r_prev->miuBlockIndex, kernal_protect_client_id,
                r_prev->start_pa, r_prev->start_pa + r_prev->length, MIU_PROTECT_DISABLE);
            MCMA_BUG_ON(!protect_ret);

			// disable r_next
            protect_ret = _miu_kernel_protect(r_next->miuBlockIndex, kernal_protect_client_id,
                r_next->start_pa, r_next->start_pa + r_next->length, MIU_PROTECT_DISABLE);
            MCMA_BUG_ON(!protect_ret);
            pranges->miuBlockStatus[r_next->miuBlockIndex] = MIU_BLOCK_IDLE;	// mark a k_range is available

            r_prev->length += (r_next->length + buffer_length);				// extend the r_prev length, and protect it
            protect_ret = _miu_kernel_protect(r_prev->miuBlockIndex, kernal_protect_client_id,
                r_prev->start_pa, r_prev->start_pa + r_prev->length, MIU_PROTECT_ENABLE);
            MCMA_BUG_ON(!protect_ret);

            list_del(&r_next->list_node);
            kfree(r_next);
            pranges->krange_num--;

            goto ADD_KRANGE_DONE;
        }
    }

    if(r_prev)
    {
        if((r_prev->start_pa + r_prev->length) == buffer_start_pa)
        {
            protect_ret = _miu_kernel_protect(r_prev->miuBlockIndex, kernal_protect_client_id,
                r_prev->start_pa, r_prev->start_pa + r_prev->length, MIU_PROTECT_DISABLE);
            MCMA_BUG_ON(!protect_ret);

            r_prev->length += buffer_length;
            protect_ret = _miu_kernel_protect(r_prev->miuBlockIndex, kernal_protect_client_id,
                r_prev->start_pa, r_prev->start_pa + r_prev->length, MIU_PROTECT_ENABLE);
            MCMA_BUG_ON(!protect_ret);

            goto ADD_KRANGE_DONE;
        }
    }

    if(r_next)
    {
        if((buffer_start_pa + buffer_length) == r_next->start_pa)
        {
			protect_ret = _miu_kernel_protect(r_next->miuBlockIndex, kernal_protect_client_id,
                r_next->start_pa, r_next->start_pa + r_next->length, MIU_PROTECT_DISABLE);
            MCMA_BUG_ON(!protect_ret);

            r_next->start_pa = buffer_start_pa;
            r_next->length += buffer_length;
            protect_ret = _miu_kernel_protect(r_next->miuBlockIndex, kernal_protect_client_id,
                r_next->start_pa, r_next->start_pa + r_next->length, MIU_PROTECT_ENABLE);
            MCMA_BUG_ON(!protect_ret);

            goto ADD_KRANGE_DONE;
        }
    }

	// use a new k_range for this buffer
    miuBlockIndex = idleBlockIndx(pranges);
    if(miuBlockIndex < 0)
    {
       ret = -ENXIO;
       MCMA_CUST_ERR("no idle miu protect block in miu %d\n", (int)miu);
       goto ADD_KRANGE_DONE;
    }
	printk(CMA_DEBUG "\033[35mFunction = %s, Line = %d, use a new k_range for this buffer, miu_protect %d for 0x%lX to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, miuBlockIndex, buffer_start_pa, (buffer_start_pa+buffer_length));

    range = (MIU_ProtectRange *)kzalloc(sizeof(MIU_ProtectRange), GFP_KERNEL);
    if(!range)
    {
       ret = -ENOMEM;
       printk(CMA_ERR "no memory\n");
       goto ADD_KRANGE_DONE;
    }
    range->start_pa = buffer_start_pa;
    range->length = buffer_length;
    range->miuBlockIndex = miuBlockIndex;
    INIT_LIST_HEAD(&range->list_node);
    if(r_prev)
        list_add(&range->list_node, &r_prev->list_node);
    else
        list_add(&range->list_node, &pranges->list_head);

    protect_ret = _miu_kernel_protect(range->miuBlockIndex, kernal_protect_client_id,
        range->start_pa, range->start_pa + range->length, MIU_PROTECT_ENABLE);
    MCMA_BUG_ON(!protect_ret);
    pranges->miuBlockStatus[range->miuBlockIndex] = MIU_BLOCK_BUSY;
    pranges->krange_num++;

ADD_KRANGE_DONE:
    mutex_unlock(&pranges->lock);
    return ret;

}




/*  this API for the case, in same miu, two adjacent lx exist, they can share same miu protect block
  *              LX_MEM                   LX2_MEM
  *  |-----------------| ------------------|
  */
static int _insertKRange(int miu_index, unsigned long lx_addr, unsigned long lx_length)
{
    MIU_ProtectRange *krange = NULL;

    if(!list_empty(&glob_miu_kranges[miu_index].list_head))
    {
        krange= list_entry(glob_miu_kranges[miu_index].list_head.prev, MIU_ProtectRange, list_node);
        if((krange->start_pa+krange->length) == lx_addr)
        {
            _miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
                krange->start_pa, krange->start_pa+krange->length, MIU_PROTECT_DISABLE);

            krange->length += lx_length;
            _miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
                krange->start_pa, krange->start_pa+krange->length, MIU_PROTECT_ENABLE);
            return 0;
        }
    }
    return -1;
}



static void init_glob_miu_kranges(void)
{
    unsigned long offset = 0;
    MIU_ProtectRange *krange = NULL;
    int i = 0, miu_index = 0;

    for(i = 0; i < KERN_CHUNK_NUM; ++i)
    {
        glob_miu_kranges[i].miu = i;
        memset(glob_miu_kranges[i].miuBlockStatus, 0, sizeof(unsigned char)*MIU_BLOCK_NUM);
        glob_miu_kranges[i].krange_num = 0;
        mutex_init(&glob_miu_kranges[i].lock);
        INIT_LIST_HEAD(&glob_miu_kranges[i].list_head);
    }

    if(lx_mem_size != INVALID_PHY_ADDR)
    {
        phy_to_MiuOffset(PHYS_OFFSET, &miu_index, &offset);

        krange = (MIU_ProtectRange *)kzalloc(sizeof(MIU_ProtectRange), GFP_KERNEL);
        MCMA_BUG_ON(!krange);
        INIT_LIST_HEAD(&krange->list_node);
        krange->start_pa = PHYS_OFFSET;
        krange->length = lx_mem_size;
        krange->miuBlockIndex = glob_miu_kranges[miu_index].krange_num;	// use miu_index's kernel protect
        // kernel protect block index start with 0

		printk(CMA_DEBUG "\033[35mFunction = %s, Line = %d, [INIT] for LX0 kprotect: from 0x%lX to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, krange->start_pa, krange->start_pa+krange->length);
		_miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
            krange->start_pa, krange->start_pa+krange->length, MIU_PROTECT_ENABLE);
        glob_miu_kranges[miu_index].miuBlockStatus[krange->miuBlockIndex] = MIU_BLOCK_BUSY;

        glob_miu_kranges[miu_index].krange_num++;						// next miu_index's kernel protect id
        list_add_tail(&krange->list_node, &glob_miu_kranges[miu_index].list_head);
    }

    if(lx_mem2_size != INVALID_PHY_ADDR)
    {
        phy_to_MiuOffset(lx_mem2_addr, &miu_index, &offset);

        if(_insertKRange(miu_index, lx_mem2_addr, lx_mem2_size))
        {
            krange = (MIU_ProtectRange *)kzalloc(sizeof(MIU_ProtectRange), GFP_KERNEL);
            MCMA_BUG_ON(!krange);
            INIT_LIST_HEAD(&krange->list_node);
            krange->start_pa = lx_mem2_addr;
            krange->length = lx_mem2_size;
            krange->miuBlockIndex = glob_miu_kranges[miu_index].krange_num;
            //kernel protect block index start with 0

			printk(CMA_DEBUG "\033[35mFunction = %s, Line = %d, [INIT] for LX1 kprotect: from 0x%lX to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, krange->start_pa, krange->start_pa+krange->length);
            _miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
                krange->start_pa, krange->start_pa+krange->length, MIU_PROTECT_ENABLE);
            glob_miu_kranges[miu_index].miuBlockStatus[krange->miuBlockIndex] = MIU_BLOCK_BUSY;

            glob_miu_kranges[miu_index].krange_num++;
            list_add_tail(&krange->list_node, &glob_miu_kranges[miu_index].list_head);
        }
    }

    if(lx_mem3_size != INVALID_PHY_ADDR)
    {
        phy_to_MiuOffset(lx_mem3_addr, &miu_index, &offset);

        if(_insertKRange(miu_index, lx_mem3_addr, lx_mem3_size))
        {
            krange = (MIU_ProtectRange *)kzalloc(sizeof(MIU_ProtectRange), GFP_KERNEL);
            MCMA_BUG_ON(!krange);
            INIT_LIST_HEAD(&krange->list_node);
            krange->start_pa = lx_mem3_addr;
            krange->length = lx_mem3_size;
            krange->miuBlockIndex = glob_miu_kranges[miu_index].krange_num;
            //kernel protect block index start with 0
            _miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
                krange->start_pa, krange->start_pa+krange->length, MIU_PROTECT_ENABLE);
            glob_miu_kranges[miu_index].miuBlockStatus[krange->miuBlockIndex] = MIU_BLOCK_BUSY;

            glob_miu_kranges[miu_index].krange_num++;

            list_add_tail(&krange->list_node, &glob_miu_kranges[miu_index].list_head);
        }
    }
	if(lx_mem4_size != INVALID_PHY_ADDR)
    {
        phy_to_MiuOffset(lx_mem4_addr, &miu_index, &offset);

		printk("\033[35mFunction = %s, Line = %d, Insert KProtect for LX4 @ MIU: %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, miu_index);
        if(_insertKRange(miu_index, lx_mem4_addr, lx_mem4_size))	// we first check if LX4 can be combined to an existed protect_block(krange), if not, we add a new protect_block(krange)
        {
            krange = (MIU_ProtectRange *)kzalloc(sizeof(MIU_ProtectRange), GFP_KERNEL);
            MCMA_BUG_ON(!krange);
            INIT_LIST_HEAD(&krange->list_node);
            krange->start_pa = lx_mem4_addr;
            krange->length = lx_mem4_size;
            krange->miuBlockIndex = glob_miu_kranges[miu_index].krange_num;
            //kernel protect block index start with 0

			printk("\033[35mFunction = %s, Line = %d, [INIT] for LX4 kprotect: from 0x%lX to 0x%lX, using block %u\033[m\n", __PRETTY_FUNCTION__, __LINE__, krange->start_pa, krange->start_pa + krange->length, krange->miuBlockIndex);
            _miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
                krange->start_pa, krange->start_pa+krange->length, MIU_PROTECT_ENABLE);
            glob_miu_kranges[miu_index].miuBlockStatus[krange->miuBlockIndex] = MIU_BLOCK_BUSY;

            glob_miu_kranges[miu_index].krange_num++;

            list_add_tail(&krange->list_node, &glob_miu_kranges[miu_index].list_head);
        }
    }


}



//??Ҫ????suspend&resume????


static int miu_protect_suspend(struct device *dev)
{

    int miu_index= 0;
    MIU_ProtectRange *krange = NULL;
    for (miu_index = 0; miu_index < KERN_CHUNK_NUM; miu_index++)
    {
        list_for_each_entry(krange, &glob_miu_kranges[miu_index].list_head, list_node)
        {
            printk(CMA_DEBUG "--cmapool suspend--[miu: %d][block: %d][start: %lx][length: %lx]---\n",
                    miu_index, krange->miuBlockIndex, krange->start_pa, krange->length);
        }
    }
	MDrv_MIU_Save();

    return 0;
}

static int miu_protect_resume(struct device *dev)
{
    int miu_index= 0;
    MIU_ProtectRange *krange = NULL;
    for (miu_index = 0; miu_index < KERN_CHUNK_NUM; miu_index++)
    {
        list_for_each_entry(krange, &glob_miu_kranges[miu_index].list_head, list_node)
        {
            printk(CMA_DEBUG "--cmapool resume--[miu: %d][block: %d][start: %lx][length: %lx]---\n",
                    miu_index, krange->miuBlockIndex, krange->start_pa, krange->length);
            _miu_kernel_protect(krange->miuBlockIndex, kernal_protect_client_id,
                    krange->start_pa, krange->start_pa + krange->length, MIU_PROTECT_ENABLE);
        }
    }
	MDrv_MIU_Restore();

    return 0;
}

static int miu_protect_freeze(struct device *dev)
{
    return 0;
}

static int miu_protect_thaw(struct device *dev)
{
    return 0;
}

static int miu_protect_restore(struct device *dev)
{
    return 0;
}

static int miu_protect_probe(struct platform_device *pdev)
{
    pdev->dev.platform_data = NULL;
    return 0;
}

static int miu_protect_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data = NULL;
    return 0;
}

static const struct dev_pm_ops miu_protect_pm_ops =
{
    .suspend = miu_protect_suspend,
    .resume  = miu_protect_resume,
    .freeze  = miu_protect_freeze,
    .thaw    = miu_protect_thaw,
    .restore = miu_protect_restore,
};

static struct platform_driver Mstar_miuprotect_driver = {
    .probe   = miu_protect_probe,
    .remove  = miu_protect_remove,

    .driver = {
        .name   = MOD_MIUPROTECT_NAME,
        .owner  = THIS_MODULE,
        .pm     = &miu_protect_pm_ops,
    }
};
static struct platform_device miu_protect_dev ={
    .name   = MOD_MIUPROTECT_NAME,
    .id     = 0,
};


//
/*
kprotect_enable
?????е?code??ԭ????
ֻ??????kprotect_enabled??ֵ??Ȼ??ɶҲ???? 

_miu_kernel_protect?????е?code??ԭ????
kprotect_enabledΪ0?Ļ?????enable area????ʱ??bypass,disable area????ʱҲ??bypass,
????????Ҫ??ʲô ?ı䣬????ʲô?????? ??
???Ƿ񲻷??????ǵ???Ҫ ???Ƿ?Ҫ?????????? :

 if(true  == flag__enable_change_to_disable)
{
Miu protect???ص?????code,????memory??alloc/free??״̬?仯?Ĺ?????
????Ҫ?µ?area??kernel protect??enable????bypass,????Ҫ?µ?area??kernel protect?
?̬??enable??disable?Ļ???disable??
}
    if(true  ==  flag__disable_change_to_enable)
{
????cma heap???????е?alloc???????Ҵ?ʱ????��??????????
kernel protect?ķ?Χ
}

===>20170720update:talk with leo,??ԭ��?ļ????����???һ?£? ???????? ??
ֻ??????kprotect_enabled??ֵ??Ȼ??ɶҲ???? ??
Ȼ??_miu_kernel_protect??kprotect_enabledΪ0?Ļ????򲻹?ʲô????(to enable,to disable )??bypass.
??Ϊ??????debug ???ܣ?ֻ??ע?䶯֮???? ?????Ĳ?????



*/
static ssize_t kprotect_enable(struct file *file, const char __user *user_buf, size_t size, loff_t *ppos)
{
    int len = 2;
    char buf[2];

    if(size > 2)
	    len = 2;

    if(__copy_from_user(buf, user_buf, len))
    {
	    return -EFAULT;
    }

    if(buf[0]=='0')
    {
	    atomic_set(&kprotect_enabled, 0);
    }
    else if(buf[0]=='1')
    {
	    atomic_set(&kprotect_enabled, 1);
    }

    if(atomic_read(&kprotect_enabled) > 0)
        printk(CMA_ERR "kernel protect enabled\n");
    else
       printk(CMA_ERR "kernel protect disabled\n");

    return size;
}


int kprotect_status(struct seq_file *m, void *v)
{
    int kprotect_value;
    char buf[2];

	kprotect_value = atomic_read(&kprotect_enabled);
    if(kprotect_value == 0)
        buf[0] = '0';
    else
        buf[0] = '1';

    //seq_write(m, (const void *)buf, 1);
    if(kprotect_value > 0)
        printk(CMA_ERR "kernel protect enabled\n");
    else
        printk(CMA_ERR "kernel protect disabled\n");

    return 0;
}


static int miu_kernel_protect_open(struct inode *inode, struct file *file)
{
	return single_open(file, kprotect_status, inode->i_private);
}

static const struct file_operations kprotect_fops = {
	.owner		= THIS_MODULE,
	.open		= miu_kernel_protect_open,
	.read		= seq_read,
	.write		= kprotect_enable,
 	.llseek		= seq_lseek,
	.release	= single_release,
};


#ifdef CONFIG_ARM_LPAE
extern phys_addr_t lx_mem2_addr;
extern phys_addr_t lx_mem3_addr;
#else
extern unsigned long lx_mem2_addr;
extern unsigned long lx_mem3_addr;
#endif
extern unsigned long lx_mem_addr;// = PHYS_OFFSET;
extern unsigned long lx_mem_size;// = INVALID_PHY_ADDR; //default setting
extern unsigned long lx_mem2_size;// = INVALID_PHY_ADDR; //default setting
extern unsigned long lx_mem3_size;// = INVALID_PHY_ADDR; //default setting
MSYSTEM_STATIC int __init mod_miuprotect_init(void)
{

    struct dentry * read_file = NULL;
    struct dentry *   debug_root; 
    MCMA_BUG_ON(FALSE == MDrv_MIU_Kernel_Init());
    kernal_protect_client_id = MDrv_MIU_Kernel_GetDefaultClientID_KernelProtect();
    init_glob_miu_kranges();		// to set kernel_protect for each lxmem
    INIT_LIST_HEAD(&heap_info_list);


    debug_root = debugfs_create_dir("miu_protect", NULL);
    if (!debug_root)
    {
		printk( "miu protect: failed to create debugfs root directory.\n");
    }
    else
    {
        read_file = debugfs_create_file("kprotect_state", 0664, debug_root,
						NULL, &kprotect_fops);
        if (!read_file)
            printk("miu protect: failed to create debugfs file kprotect_state\n");

    }
    platform_device_register(&miu_protect_dev);
    platform_driver_register(&Mstar_miuprotect_driver);

    return 0;
}

MSYSTEM_STATIC void __exit mod_miuprotect_exit(void)
{
    platform_device_unregister(&miu_protect_dev);
    platform_driver_unregister(&Mstar_miuprotect_driver);
	
    return;

}

#ifdef CONFIG_MSTAR_MIUPROTECT
module_init(mod_miuprotect_init);
module_exit(mod_miuprotect_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("MIUPROTECT driver");
MODULE_LICENSE("GPL");
#endif
#endif
