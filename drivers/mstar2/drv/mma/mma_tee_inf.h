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

#ifndef _MMA_TEE_INTERFACE_H_
#define _MMA_TEE_INTERFACE_H_

#include "mma_common.h"
#include "mma_tee_inf.h"
#include "mma_core.h"
/* this structure use to communication with tee, convert sg_table to this structure*/
struct mma_tee_page_table{
    u64 size;
    u32 pa_num;                    /* number of list entries */
    struct mma_range_t pa_list[1];      /* page list */
};

/* this structure use to communication with tee */
struct mma_tee_secure_info {
	char uuid[UUID_SIZE];
    char buf_tag[MAX_NAME_SIZE];
};

enum mma_addr_type{
    MMA_ADDR_TYPE_IOVA,
    MMA_ADDR_TYPE_PA,
    MMA_ADDR_TYPE_MAX
};

enum mma_unauth_status_e{
    MMA_UNAUTH_SUCCESS,
    MMA_UNAUTH_DELAY_FREE,
    MMA_UNAUTH_FREE_RANGE,
    MMA_UNAUTH_MAX
};
typedef struct buf_tag{
    uint32_t heap_type;
    uint32_t miu;
    uint64_t maxsize;
    char name[64];
}buf_tag;

typedef struct {
	uint32_t type;
	uint32_t is_secure;
	uint64_t pa_num;
	uint64_t size;
	uint32_t flag;
	uint64_t addr; //for buffer reallocate,old buffer iova
	char buffer_tag[MAX_NAME_SIZE];
} tee_map_args_v2;

typedef struct {
	uint32_t aid[8];
	uint32_t aid_num;
	char buffer_tag[MAX_NAME_SIZE];
} tee_map_lock;

static inline enum mma_addr_type mma_tee_addr_type( u64 addr)
{
    if(addr >= IOVA_START_ADDR)
        return MMA_ADDR_TYPE_IOVA;
    else
        return MMA_ADDR_TYPE_PA;
}

/*
description£ºTA open session
para_in£ºna
para_out£ºna
return:  0: success
         negative£ºfailed
*/
int mma_tee_open_session(void);


/*
description£ºTA close session
para_in£ºna
para_out: na
return:  0: success
         negative£ºfailed
*/
int  mma_tee_close_session(bool closed);

/*
description£ºmap iova
para_in£º
		space_tag: reserved tag
		pagetbl: scatter list
		_2x: double size iova
		buf_tag: buf tag
para_out: 
		va_out: iova addr
return:  0: success
         negative£ºfailed
*/
int mma_tee_map(const char* space_tag, struct sg_table *sgt, u64* va_out, u32 _2x, const char *buf_tag);

/*
description: unmap iova
para_in£º
		type: iova or pa
		va: iova to be unmap
para_out: 
		va_out: range to be freed by cma.  iova ignore this.
return:  MMA_UNMAP_SUCCESS: success
		 MMA_UNMAP_DELAY_FREE: the page should't be freed
		 MMA_UNMAP_FREE_RANGE: free page base on the va_out range.
*/

int mma_tee_unmap (enum mma_addr_type type, uint64_t va, struct mma_range_t *va_out);

/*
description£ºauthorize va, before calling this, hw ip can't be access secure buffer
para_in£º
		type: iova or pa
		va: iova addreess
		buf_tag: buffer tag
		pipe_id: pipe id represent by uuid
para_out: na
return:  0: success
         negative£ºfailed
*/
int mma_tee_authorize ( u64 va,u32 buffer_size, const char* buf_tag, uint32_t pipe_id);

int mma_tee_unauthorize(u64 va,u32 buffer_size,const char* buf_tag,
                       uint32_t pipe_id,struct mma_range_t *rang_out);

/*
description£ºreserve iova space
para_in£º
		type: iova or pa
		space_tag: space name
		size: size to be reserved
para_out: 
		va_out: base address of reserved iova space
return:  0: success
         negative£ºfailed
*/
int mma_tee_reserve_space (enum mma_addr_type type, const char *space_tag,  u64 size, u64* va_out);

/*
description£ºfree reserved iova space
para_in£º
		type: iova or pa
		space_tag: space name
para_out: na
return:  0: success
         negative£ºfailed
*/
int mma_tee_free_space (enum mma_addr_type type, const char* space_tag);

/*
description£ºset mpu range
para_in£º
		type: iova or pa
		range: mpu range array
		num: num of array
para_out: na
return:  0: success
         negative£ºfailed
*/
int mma_tee_set_mpu_area (enum mma_addr_type type, struct mma_range_t *range, uint8_t num);


int mma_tee_debug(uint32_t debug_type,uint8_t *buf1,uint8_t size1,
                       uint8_t *buf2,uint8_t size2,uint8_t *buf3,uint8_t size3);
int mma_tee_get_pipeid (int* pipeid);

int mma_tee_put_pipeid(int pipeid);

int mma_optee_ta_store_buf_tags(void);

int mma_tee_pipelineID_query (u64 addr,uint32_t* pipeline_id);
int mma_tee_lockdebug(tee_map_lock *lock);

#endif
