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

#ifndef _MMA_ION_H_
#define _MMA_ION_H_

#include <linux/version.h>
#include "mma_common.h"

/*
description£ºopen ion file
para_in£ºmma_dev
para_out£ºna
return:  0: success
         negative£ºfailed
*/
int mma_ion_open(struct mma_device* mma_dev);

/*
description£ºclose ion file
para_in£º
		mma_dev: mma devuce structure
para_out£ºna
return:  0: success
         negative£ºfailed
*/

int mma_ion_close(struct mma_device* mma_dev);


/*
description£ºalloc buffer by calling ion interface
para_in£º
		mma_dev: mma devuce structure
		size: size of buffer
para_out£ºheap_type: "cma" or "system"
para_out£ºmiu: which miu
return:  0: success
         negative£ºfailed
*/
int mma_ion_alloc(struct mma_device* mma_dev, size_t size, unsigned int heap_mask,
                  int flag, int* dmabuf_fd);


/*
description£ºget heap info
para_in£ºbuf_tag
para_out£ºheap_type: "cma" or "system"
para_out£ºmiu: which miu
return:  0: success
         negative£ºfailed
*/
int mma_ion_query_heap(struct mma_device* mma_dev);


/*
description£ºget heap info
para_in£ºbuf_tag
para_out£ºheap_type: "cma" or "system"
para_out£ºmiu: which miu
return:  0: success
         negative£ºfailed
*/
int  mma_ion_get_id_flag(int heap_type, int miu,int zone_flag, bool secure, int* heap_id, int* flag, u32 size, char* buf_tag);


#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
int mma_ion_import(int fd, ion_user_handle_t* handle);
int mma_ion_free(ion_user_handle_t handle);
#endif
#endif
