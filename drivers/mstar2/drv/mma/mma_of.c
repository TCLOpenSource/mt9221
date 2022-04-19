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

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/types.h>
#include <linux/slab.h>
#include "mma_of.h"
#include "mma_common.h"
#include <linux/version.h>
//#ifdef CONFIG_OF

static dram_type_info mma_type_info={
.mma_type = ERR__TYPR,
};

static mpu_cma_info mpu_cma_size={
.cma_size_in_miu = {-1,-1,-1},
.cma_addr_in_miu = {-1,-1,-1},
.is_scan_dt = 0,
};

static struct device_node *mma_buf_tag = NULL;
static LIST_HEAD(buftag_list);
u64 __init dt_mma_next_cell(int s, __be32 **cellp)
{
    __be32 *p = *cellp;

    *cellp = p + s;
    return of_read_number(p, s);
}

int __init early_init_dt_scan_mma_type(unsigned long node, const char *uname,
                     int depth, void *data)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    int l = 0,reg_l=0;
#else
    unsigned long l = 0,reg_l=0;
#endif
    char *p = NULL;
    __be32 *reg=NULL;
    dram_type_info *type_info = data;

//pr_notice("search \"dram_type\", depth: %d, uname: %s\n", depth, uname);

    if (!type_info ||(strcmp(uname, "dram_type") != 0 ))
        return 0;

    p = (char *)of_get_flat_dt_prop(node, "dram_type", &l);

    if (p == NULL || l <= 0)
            return 0;

    pr_notice("dram_type is: %s\n",p);
    if(strncmp(p,"UMA",4) == 0){

        p = NULL;
        p = (char *)of_get_flat_dt_prop(node,"asymmetric_dram",&l);
        if(p == NULL || *p != '1'){
            type_info->mma_type = UMA;
            return 1;
        }
        type_info->mma_type = UMA_WIDE_NARROW_DRAM;
        type_info->uma.asymmetric_dram = 1;
        reg = (__be32 *)of_get_flat_dt_prop(node,"wide_range",&reg_l);
        if(reg != NULL){
        type_info->uma.wide_range.start = dt_mma_next_cell(2,&reg);
        type_info->uma.wide_range.size = dt_mma_next_cell(2,&reg);
    }
    reg = NULL;
        reg = (__be32 *)of_get_flat_dt_prop(node,"narrow_range",&reg_l);
        if(reg == NULL){
            type_info->uma.narrow_range.start = 0;
            type_info->uma.narrow_range.size = 0;
            return 1;
        }
        type_info->uma.narrow_range.start = dt_mma_next_cell(2,&reg);
        type_info->uma.narrow_range.size = dt_mma_next_cell(2,&reg);
        pr_notice("l:%llx,%llx,s:%llx,%llx\n",type_info->uma.wide_range.start,
              type_info->uma.wide_range.size,type_info->uma.narrow_range.start,
              type_info->uma.narrow_range.size);

    }
    else if(strncmp(p,"NUMA",5)==0){
        type_info->mma_type = NUMA;
        reg =NULL;
        reg = (__be32 *)of_get_flat_dt_prop(node,"miu_number",&reg_l);
        if(reg == NULL)
            return 1;
       type_info->numa.miu_number = dt_mma_next_cell(1,&reg);

        reg =NULL;
        reg = (__be32 *)of_get_flat_dt_prop(node,"miu0_range",&reg_l);
        if(reg == NULL)
            return 1;
        type_info->numa.miu[0].start = dt_mma_next_cell(2,&reg);
        type_info->numa.miu[0].size = dt_mma_next_cell(2,&reg);
    pr_notice("miu:%llx,%llx,\n",type_info->numa.miu[0].start,
              type_info->numa.miu[0].size);

        reg =NULL;
        reg = (__be32 *)of_get_flat_dt_prop(node,"miu1_range",&reg_l);
        if(reg == NULL)
            return 1;
        type_info->numa.miu[1].start = dt_mma_next_cell(2,&reg);
        type_info->numa.miu[1].size = dt_mma_next_cell(2,&reg);
    pr_notice("miu:%llx,%llx,\n",type_info->numa.miu[1].start,
              type_info->numa.miu[1].size);

        reg =NULL;
        reg = (__be32 *)of_get_flat_dt_prop(node,"miu2_range",&reg_l);
        if(reg == NULL)
            return 1;
        type_info->numa.miu[2].start = dt_mma_next_cell(2,&reg);
        type_info->numa.miu[2].size = dt_mma_next_cell(2,&reg);
    pr_notice("miu:%llx,%llx,\n",type_info->numa.miu[2].start,
              type_info->numa.miu[2].size);

    }
    else {
        pr_notice("err: mma_type:%s\n",p);
        return 0;
    }


    /* break now */
    return 1;
}

mma_miu_config_type mma_of_get_dram_type(){
    if(mma_type_info.mma_type != ERR__TYPR)
        return mma_type_info.mma_type;

    of_scan_flat_dt(early_init_dt_scan_mma_type,&mma_type_info);
    return mma_type_info.mma_type;

}

int  mma_of_get_miu_number(){
    if(mma_type_info.mma_type == ERR__TYPR)
        of_scan_flat_dt(early_init_dt_scan_mma_type,&mma_type_info);

    if(mma_type_info.mma_type == NUMA)
        return mma_type_info.numa.miu_number;

    return 1; //default return 1
}

int mma_of_get_miu_range(int miu, struct  mma_range_t* range){
    if(miu < 0)
        return -1;

    if(mma_type_info.mma_type == ERR__TYPR)
        of_scan_flat_dt(early_init_dt_scan_mma_type,&mma_type_info);

    if(mma_type_info.mma_type == NUMA &&
       mma_type_info.numa.miu_number > miu ){
       range->start = mma_type_info.numa.miu[miu].start;
       range->size = mma_type_info.numa.miu[miu].size;
       return 0;
    }
    return -1;
}

int mma_of_asymmetric_dram(){
    if(mma_type_info.mma_type == ERR__TYPR)
        of_scan_flat_dt(early_init_dt_scan_mma_type,&mma_type_info);

    if(mma_type_info.mma_type == UMA_WIDE_NARROW_DRAM)
        return mma_type_info.uma.asymmetric_dram;

    return 0;
}

int mma_of_get_wide_range(struct mma_range_t* range){

    if(mma_type_info.mma_type == ERR__TYPR)
        of_scan_flat_dt(early_init_dt_scan_mma_type,&mma_type_info);

    if(mma_type_info.mma_type == UMA &&
       mma_type_info.uma.asymmetric_dram ){
       range->start = mma_type_info.uma.wide_range.start;
       range->size = mma_type_info.uma.wide_range.size;
       return 0;
    }
    return -1;
}

int mma_of_get_narrow_range(struct mma_range_t* range){

    if(mma_type_info.mma_type == ERR__TYPR)
        of_scan_flat_dt(early_init_dt_scan_mma_type,&mma_type_info);

    if(mma_type_info.mma_type == UMA &&
       mma_type_info.uma.asymmetric_dram ){
       range->start = mma_type_info.uma.narrow_range.start;
       range->size = mma_type_info.uma.narrow_range.size;
       return 0;
    }
    return -1;
}

int __init early_init_dt_scan_mpu_cma_info(unsigned long node, const char *uname,
                     int depth, void *data)
{

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    int l = 0;
#else
    unsigned long l = 0;
#endif
    __be32 *reg = NULL;
    mpu_cma_info *miu = data;

//  pr_notice("search \"miu_info\", depth: %d, uname: %s\n", depth, uname);

    if (!miu ||(strcmp(uname, "mpu_cma_info") != 0 ))
        return 0;

    reg = (__be32 *)of_get_flat_dt_prop(node, "cma_size_in_miu0", &l);
    if (reg != NULL ){
         miu->cma_size_in_miu[0] = dt_mma_next_cell(2,&reg);
    }
    reg = NULL;
    reg = (__be32 *)of_get_flat_dt_prop(node, "cma_size_in_miu1", &l);
    if (reg != NULL ){
         miu->cma_size_in_miu[1] = dt_mma_next_cell(2,&reg);
    }
    reg = NULL;
    reg = (__be32 *)of_get_flat_dt_prop(node, "cma_size_in_miu2", &l);
    if (reg != NULL ){
         miu->cma_size_in_miu[2] = dt_mma_next_cell(2,&reg);
    }

    reg = NULL;
    reg = (__be32 *)of_get_flat_dt_prop(node, "cma_addr_in_miu0", &l);
    if (reg != NULL ){
         miu->cma_addr_in_miu[0] = dt_mma_next_cell(2,&reg);
    }
    reg = NULL;
    reg = (__be32 *)of_get_flat_dt_prop(node, "cma_addr__in_miu1", &l);
    if (reg != NULL ){
         miu->cma_addr_in_miu[1] = dt_mma_next_cell(2,&reg);
    }
    reg = NULL;
    reg = (__be32 *)of_get_flat_dt_prop(node, "cma_addr_in_miu2", &l);
    if (reg != NULL ){
         miu->cma_addr_in_miu[2] = dt_mma_next_cell(2,&reg);
    }
    miu->is_scan_dt = 1;
    /* break now */
    return 1;
}

uint64_t mma_of_get_cma_size(int miu){
    if(miu < 0 || miu >= 3)
        return -1;
    if(mpu_cma_size.is_scan_dt == 0)
        of_scan_flat_dt(early_init_dt_scan_mpu_cma_info,&mpu_cma_size);
    return mpu_cma_size.cma_size_in_miu[miu];
}

uint64_t mma_of_get_cma_addr(int miu){
    if(miu < 0 || miu >= 3)
        return -1;
    if(mpu_cma_size.is_scan_dt == 0)
        of_scan_flat_dt(early_init_dt_scan_mpu_cma_info,&mpu_cma_size);
    return mpu_cma_size.cma_addr_in_miu[miu];
}


int mma_of_get_buftag_info( const char *buf_tag, enum mma_of_heap_type *heap_type ,
                             int* miu,uint64_t *maxsize, int* zone_flag)
{
   struct device_node *np = NULL;
   uint32_t len = 0;
   __be32 *p = NULL;
   __be64 *p64 = NULL;
    if(buf_tag == NULL || heap_type == NULL||miu == NULL||maxsize == NULL)
       return -1;

    if(mma_buf_tag == NULL){
        mma_buf_tag = of_find_node_by_name(NULL,"buf_tag");

        if(mma_buf_tag == NULL)
             return -1;
         if(mma_buf_tag->child == NULL){
             of_node_put(mma_buf_tag);
             return -1;
        }

    }

    np = mma_buf_tag->child;
    for (; np; np = np->sibling)
        if(np->name && (of_node_cmp(np->name, buf_tag) == 0)
                         && of_node_get(np))
             break;
    if(np == NULL)
        return -1;

    p = (__be32 *)of_get_property(np, "heaptype",&len);
    if(p != NULL)
        *heap_type = be32_to_cpup(p);

    p = NULL;
    p = (__be32 *)of_get_property(np, "miu",&len);
    if(p != NULL)
        *miu = be32_to_cpup(p);

    p64 = NULL;
    p64 = (__be64 *)of_get_property(np, "max_size",&len);
    if(p64 != NULL)
        *maxsize = be64_to_cpup(p64);
    else
       *maxsize = -1;

    if(zone_flag){
        p=NULL;
        p = (__be32 *)of_get_property(np,"normal_zone",&len);
        if(p != NULL)
            *zone_flag = be32_to_cpup(p);
        else
            *zone_flag = 0;
    }
    of_node_put(np);

    return 0;
}

struct list_head* mma_get_buftags()
{
   struct device_node *np = NULL;
   uint32_t len = 0;
   __be32 *p = NULL;
   __be64 *p64 = NULL;
   struct buf_tag_info *buf_info;

    if(!list_empty(&buftag_list))
        return &buftag_list;

    if(mma_buf_tag == NULL){
        mma_buf_tag = of_find_node_by_name(NULL,"buf_tag");

        if(mma_buf_tag == NULL)
             return NULL;
         if(mma_buf_tag->child == NULL){
             of_node_put(mma_buf_tag);
             return NULL;
        }

    }

    np = mma_buf_tag->child;
    for (; np; np = np->sibling){
        if(!np->name)
            return NULL;

         of_node_get(np);

         buf_info = kzalloc(sizeof(struct buf_tag_info),GFP_KERNEL);
         if(!buf_info)
             return NULL;
        strncpy(buf_info->name,np->name,64);
        buf_info->name[63] = '\0';
        p = (__be32 *)of_get_property(np, "heaptype",&len);
        if(p != NULL)
            buf_info->heap_type = be32_to_cpup(p);

        p = NULL;
        p = (__be32 *)of_get_property(np, "miu",&len);
        if(p != NULL)
            buf_info->miu = be32_to_cpup(p);

        p64 = NULL;
        p64 = (__be64 *)of_get_property(np, "max_size",&len);
        if(p64 != NULL)
           buf_info->maxsize = be64_to_cpup(p64);

        list_add_tail(&(buf_info->list),&buftag_list);
        of_node_put(np);
    }
   return &buftag_list;
}
//#endif
