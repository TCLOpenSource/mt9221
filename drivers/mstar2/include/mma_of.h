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

#ifndef _MMA_OF_H
#define _MMA_OF_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/errno.h>
#include "mma_common.h"

typedef enum mma_of_heap_type {
    HEAP_TYPE_IOMMU = 0,
    HEAP_TYPE_CMA,
    HEAP_TYPE_CMA_IOMMU,       //CMA with MPA for external domain. alloc cma_page for iommu
    HEAP_TYPE_CARVEOUT,
    HEAP_TYPE_MAX,
}heap_type;

typedef enum mma_miu_config_type
{
    ERR__TYPR=0,
    UMA,
    UMA_WIDE_NARROW_DRAM,
    NUMA
}mma_miu_config_type;

typedef struct miu_info{
    uint32_t miu_nu;
    uint64_t start_add;
    uint64_t leng;
}miu_info;

typedef struct uma_info{
    uint8_t asymmetric_dram;
    struct mma_range_t wide_range;
    struct mma_range_t narrow_range;
}uma_info;

typedef struct numa_info{
     uint8_t miu_number;
     struct mma_range_t miu[3];
}numa_info;

typedef struct dram_type_info
{
    mma_miu_config_type mma_type;
    union{
        numa_info numa;
        uma_info uma;
    } ;
}dram_type_info;

typedef struct mpu_cma_info{
    uint64_t cma_size_in_miu[3];
    uint64_t cma_addr_in_miu[3];
    uint8_t is_scan_dt;
}mpu_cma_info;

typedef struct buf_tag_info{
    uint32_t heap_type;
    uint32_t miu;
    uint64_t maxsize;
    char name[64];
    struct list_head list;
}buf_tag_info;

//#ifdef CONFIG_OF
/*
description£ºget buf_tag info
para_in£ºbuf_tag
para_out£ºheap_type: "cma" or "system"
para_out£ºmiu: which miu
return:  0: success
         negative£ºfailed
*/
int mma_of_get_buftag_info( const char *buf_tag, enum mma_of_heap_type *heap_type,
                             int* miu,uint64_t *maxsize, int* zone_flag);

/*
description£ºget dram type
para_in£ºmiu number
para_out£ºna
return:  NULL: failed
         "uma" or "numa"
*/
mma_miu_config_type  mma_of_get_dram_type(void);


/*
description£ºget miu nunber
para_in£ºna
para_out£ºna
return:  number of miu
*/
int mma_of_get_miu_number(void);

/*
description£ºget miu range
para_in£ºmiu number
para_out£ºrange
return:  0: success
         negative£ºfailed
*/
int mma_of_get_miu_range(int miu, struct mma_range_t* range);

/*
description£ºwhether dram is  asymmetric, that is have wide and narrow bus-width
para_in£ºna
para_out£ºna
return:  1: asymmetric dram
		 0: symmetric_dram
*/
int mma_of_asymmetric_dram(void);

/*
description£ºget wide and narrow
para_in: na
para_out£ºrange
return:  0: success
         negative£ºfailed
*/
int mma_of_get_wide_range(struct mma_range_t* range);
int mma_of_get_narrow_range(struct mma_range_t* range);

/*
description£ºget cma size in each miu
para_in£ºmiu number
para_out£ºna
return:  size of cma
*/
uint64_t mma_of_get_cma_size(int miu);

/*
description£ºget mpu size in each miu
para_in£ºmiu number
para_out£ºna
return:  size of mpu
*/
uint64_t mma_of_get_cma_addr(int miu);

/*
description£ºget buf tags
para_in£º
para_out£º
return:  list of buf tag (struct buf_tag_info)
*/
struct list_head* mma_get_buftags(void);
//#endif
#endif
