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

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/kallsyms.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/tee_kernel_api.h>
#include <linux/tee_client_api.h>
#include <linux/tee_drv.h>

#include "static_ta_Mma.h"
#include "mma_common.h"
#include "mma_tee_inf.h"
#include "mma_of.h"

struct mma_optee_contex {
    //struct mutex lock;
    struct dentry *debug_root;
    TEEC_Context ctx;
    TEEC_Session session;
    bool session_initialized;
};
#ifdef CONFIG_OPTEE
extern atomic_t STR;
extern atomic_t SMC_UNLOCKED;
#endif

#ifdef CONFIG_TEE_2_4
extern int optee_version;
#else
static int optee_version = 1;
#endif

static struct mma_optee_contex mma_ctx;

static int mma_init (void);

static void uuid_to_octets(uint8_t d[TEE_IOCTL_UUID_LEN], const TEEC_UUID *s)
{
	d[0] = s->timeLow >> 24;
	d[1] = s->timeLow >> 16;
	d[2] = s->timeLow >> 8;
	d[3] = s->timeLow;
	d[4] = s->timeMid >> 8;
	d[5] = s->timeMid;
	d[6] = s->timeHiAndVersion >> 8;
	d[7] = s->timeHiAndVersion;
	memcpy(d + 8, s->clockSeqAndNode, sizeof(s->clockSeqAndNode));
}
extern struct tee_shm *tee_client_shm_alloc(struct tee_context *ctx, size_t size, u32 flags);
extern void *tee_client_shm_get_va(struct tee_shm *shm, size_t offs);
extern void tee_shm_free(struct tee_shm *shm);

typedef struct {
	void *buffer;
	size_t size;
	uint32_t flags;
	/*
	 * Implementation-Defined
	 */
	int id;
	size_t alloced_size;
	void *shadow_buffer;
	int registered_fd;
} TEEC_SharedMemory_2;


static TEEC_Result teec_allocatesharedmemory(TEEC_Context *ctx, TEEC_SharedMemory_2 *shm)
{

	struct tee_shm *_shm = NULL;
	void *shm_va = NULL;
	size_t len;
	struct tee_context *context = (struct tee_context *) ctx->ctx;

	shm->shadow_buffer = shm->buffer = NULL;
	len = shm->size;
	if (!len)
		len = 8;
	_shm = tee_client_shm_alloc(context, len, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(_shm)) {
		printk(KERN_ERR "\033[0;32;31m %s %d: tee_client_shm_alloc Fail,len=0x%zx \033[m\n",__func__,__LINE__,len);
		return TEEC_ERROR_OUT_OF_MEMORY;
	}

	shm_va = tee_client_shm_get_va(_shm, 0);
	if (IS_ERR(shm_va)){
		printk(KERN_ERR "\033[0;32;31m %s %d: tee_client_shm_get_va Fail\033[m\n",__func__,__LINE__);
		return TEEC_ERROR_OUT_OF_MEMORY;
	}
	shm->buffer = shm_va;
	shm->shadow_buffer = (void*)_shm;
	shm->alloced_size = len;
	shm->registered_fd = -1;

	return TEEC_SUCCESS;

}

static void teec_releasesharedmemory(TEEC_SharedMemory_2 *shm)
{
	if(shm->shadow_buffer)
		tee_shm_free((struct tee_shm *)shm->shadow_buffer);
	shm->id = -1;
	shm->shadow_buffer = NULL;
	shm->buffer = NULL;
	shm->registered_fd = -1;
}

static TEEC_Result teec_pre_process_tmpref(TEEC_Context *ctx,
			uint32_t param_type, TEEC_TempMemoryReference *tmpref,
			struct tee_param *param,
			TEEC_SharedMemory_2 *shm)
{
	TEEC_Result res;

	switch (param_type) {
	case TEEC_MEMREF_TEMP_INPUT:
		param->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
		shm->flags = TEEC_MEM_INPUT;
		break;
	case TEEC_MEMREF_TEMP_OUTPUT:
		param->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
		shm->flags = TEEC_MEM_OUTPUT;
		break;
	case TEEC_MEMREF_TEMP_INOUT:
		param->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
		shm->flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT;
		break;
	default:
		return TEEC_ERROR_BAD_PARAMETERS;
	}
	shm->size = tmpref->size;

	res = teec_allocatesharedmemory(ctx, shm);
	if (res != TEEC_SUCCESS)
		return res;

	memcpy(shm->buffer, tmpref->buffer, tmpref->size);
	param->u.memref.size = tmpref->size;
	//Workaround, shm->shadow_buffer is shm object.
	param->u.memref.shm = (struct tee_shm *)shm->shadow_buffer;
	return TEEC_SUCCESS;
}

static TEEC_Result teec_pre_process_operation(TEEC_Context *ctx,
			TEEC_Operation *operation,
			struct tee_param *params,
			TEEC_SharedMemory_2 *shms)
{
	TEEC_Result res;
	size_t n;

	memset(shms, 0, sizeof(TEEC_SharedMemory_2) *
			TEEC_CONFIG_PAYLOAD_REF_COUNT);
	if (!operation) {
		memset(params, 0, sizeof(struct tee_param) *
				  TEEC_CONFIG_PAYLOAD_REF_COUNT);
		return TEEC_SUCCESS;
	}

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		uint32_t param_type;

		param_type = TEEC_PARAM_TYPE_GET(operation->paramTypes, n);
		switch (param_type) {
		case TEEC_NONE:
			params[n].attr = param_type;
			break;
		case TEEC_VALUE_INPUT:
		case TEEC_VALUE_OUTPUT:
		case TEEC_VALUE_INOUT:
			params[n].attr = param_type;
			params[n].u.value.a = operation->params[n].value.a;
			params[n].u.value.b = operation->params[n].value.b;
			break;
		case TEEC_MEMREF_TEMP_INPUT:
		case TEEC_MEMREF_TEMP_OUTPUT:
		case TEEC_MEMREF_TEMP_INOUT:
			res = teec_pre_process_tmpref(ctx, param_type,
				&operation->params[n].tmpref, params + n,
				shms + n);
			if (res != TEEC_SUCCESS)
				return res;
			break;
		default:
			printk(KERN_ERR "\033[1;31m[%s] param_type not support!\033[m\n", __FUNCTION__);
			return TEEC_ERROR_BAD_PARAMETERS;
		}
	}

	return TEEC_SUCCESS;
}


static void teec_post_process_tmpref(uint32_t param_type,
			TEEC_TempMemoryReference *tmpref,
			struct tee_param *param,
			TEEC_SharedMemory_2 *shm)
{
	if (param_type != TEEC_MEMREF_TEMP_INPUT) {
		if (param->u.memref.size <= tmpref->size && tmpref->buffer)
			memcpy(tmpref->buffer, shm->buffer,
			       param->u.memref.size);

		tmpref->size = param->u.memref.size;
	}
}

static void teec_post_process_operation(TEEC_Operation *operation,
			struct tee_param *params,
			TEEC_SharedMemory_2 *shms)
{
	size_t n;

	if (!operation)
		return;

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		uint32_t param_type;

		param_type = TEEC_PARAM_TYPE_GET(operation->paramTypes, n);
		switch (param_type) {
		case TEEC_VALUE_INPUT:
			break;
		case TEEC_VALUE_OUTPUT:
		case TEEC_VALUE_INOUT:
			operation->params[n].value.a = params[n].u.value.a;
			operation->params[n].value.b = params[n].u.value.b;
			break;
		case TEEC_MEMREF_TEMP_INPUT:
		case TEEC_MEMREF_TEMP_OUTPUT:
		case TEEC_MEMREF_TEMP_INOUT:
			teec_post_process_tmpref(param_type,
				&operation->params[n].tmpref, params + n,
				shms + n);
			break;
		default:
			break;
		}
	}
}


static void teec_free_temp_refs(TEEC_Operation *operation,
			TEEC_SharedMemory_2 *shms)
{
	size_t n;

	if (!operation)
		return;

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		switch (TEEC_PARAM_TYPE_GET(operation->paramTypes, n)) {
		case TEEC_MEMREF_TEMP_INPUT:
		case TEEC_MEMREF_TEMP_OUTPUT:
		case TEEC_MEMREF_TEMP_INOUT:
			teec_releasesharedmemory(shms + n);
			break;
		default:
			break;
		}
	}
}

static int _optee_match(struct tee_ioctl_version_data *data,
				const void *vers)
{
	return 1;
}

static TEEC_Result mma_tee_invoke_2x(TEEC_Session *session,
			       uint32_t commandID,
			       TEEC_Operation *operation,
			       uint32_t *return_origin)
{
	void *buf = NULL;
	struct tee_ioctl_invoke_arg *arg;
	struct tee_param *params;
	TEEC_Result res;
	uint32_t eorig;
	TEEC_SharedMemory_2 *shm;
	int rc;
	size_t size;

	if (!session) {
		eorig = TEEC_ORIGIN_API;
		res = TEEC_ERROR_BAD_PARAMETERS;
		return res;
	}
	shm = vzalloc(sizeof(TEEC_SharedMemory_2) * TEEC_CONFIG_PAYLOAD_REF_COUNT);
	if (!shm) {
		eorig = TEEC_ORIGIN_API;
		res = TEEC_ERROR_OUT_OF_MEMORY;
		return res;
	}
	size = (sizeof(struct tee_ioctl_invoke_arg) +
			TEEC_CONFIG_PAYLOAD_REF_COUNT *
				sizeof(struct tee_param)) / sizeof(uint64_t);
	buf = vzalloc(size);
	if (!buf) {
		vfree(shm);
		eorig = TEEC_ORIGIN_API;
		res = TEEC_ERROR_OUT_OF_MEMORY;
		return res;
	}

	arg = (struct tee_ioctl_invoke_arg *)buf;
	arg->num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;
	params = (struct tee_param *)(arg + 1);

	arg->session = session->fd;
	arg->func = commandID;

	if (operation) {
		operation->session = session;
	}
	res = teec_pre_process_operation(&mma_ctx.ctx, operation, params, shm);
	if (res != TEEC_SUCCESS) {
		eorig = TEEC_ORIGIN_API;
		goto out_free_temp_refs;
	}
	rc = tee_client_invoke_func(mma_ctx.ctx.ctx, arg, params);

	if (rc) {
		eorig = TEEC_ORIGIN_COMMS;
		res = rc;
		goto out_free_temp_refs;
	}
	res = arg->ret;
	eorig = arg->ret_origin;
	teec_post_process_operation(operation, params, shm);
out_free_temp_refs:
	teec_free_temp_refs(operation, shm);
	if (return_origin)
		*return_origin = eorig;
	vfree(buf);
	vfree(shm);
	return res;
}

static int mma_init_session_2x(void)
{
    struct tee_ioctl_open_session_arg *arg;
    struct tee_param *param;
    int rc;
	TEEC_Result res;
    TEEC_UUID uuid = MSTAR_INTERNAL_MMA_MANAGER_TA_UUID;
    struct tee_ioctl_version_data vers = {
        .impl_id = TEE_OPTEE_CAP_TZ,
        .impl_caps = TEE_IMPL_ID_OPTEE,
        .gen_caps = TEE_GEN_CAP_GP,
    };
	void *data;
    if (mma_ctx.session_initialized)
        return 0;

    arg = kzalloc(sizeof(struct tee_ioctl_open_session_arg), GFP_KERNEL);
    if (!arg)
        return -ENOMEM;

    param = kzalloc(sizeof(struct tee_param), GFP_KERNEL);
    if (!param) {
        kfree(arg);
        return -ENOMEM;
    }
    uuid_to_octets(arg->uuid, &uuid);
    arg->clnt_login = TEE_IOCTL_LOGIN_PUBLIC;

    mma_ctx.ctx.ctx = tee_client_open_context(NULL, _optee_match, NULL, &vers);

    if (IS_ERR(mma_ctx.ctx.ctx)) {
        printk(KERN_ERR "\033[1;31m[%s] context is NULL\033[m\n", __FUNCTION__);
        kfree(arg);
        kfree(param);
        return -EINVAL;
    }

    rc = tee_client_open_session(mma_ctx.ctx.ctx, arg, param);
    if (rc == TEEC_SUCCESS) {
        mma_ctx.session.fd = arg->session;
    }else{
        tee_client_close_context(mma_ctx.ctx.ctx);
        kfree(arg);
        kfree(param);
        return -EINVAL;
    }
    kfree(arg);
    kfree(param);

    res = mma_init();
    if (res != TEEC_SUCCESS) {
        printk(KERN_DEBUG "mma_tee_init failed %x\n", res);
        //TEEC_FinalizeContext(&mma_ctx.ctx);
        //return -EINVAL;
    }

    mma_ctx.session_initialized = true;
    return 0;
}

static void mma_destroy_session_2x(void)
{
    if (!mma_ctx.session_initialized)
        return;

	tee_client_close_session(mma_ctx.ctx.ctx, mma_ctx.session.fd);
	tee_client_close_context(mma_ctx.ctx.ctx);
    mma_ctx.session_initialized = false;
}

static TEEC_Result mma_tee_invoke_1x(TEEC_Session *session,
			       uint32_t commandID,
			       TEEC_Operation *operation,
			       uint32_t *return_origin)
{
	return TEEC_InvokeCommand(session, commandID, operation, return_origin);
}




static int mma_init_session_1x(void)
{
    TEEC_Result res;
    uint32_t err_origin;
    TEEC_UUID uuid = MSTAR_INTERNAL_MMA_MANAGER_TA_UUID;

    if (mma_ctx.session_initialized)
        return 0;

    res = TEEC_InitializeContext(NULL, &mma_ctx.ctx);
    if (res != TEEC_SUCCESS) {
        printk (KERN_ERR "TEEC_InitializeContext failed %d\n", res);
        return -EINVAL;
    }

    res = TEEC_OpenSession(&mma_ctx.ctx, &mma_ctx.session, &uuid,
            TEEC_LOGIN_PUBLIC, NULL, NULL, &err_origin);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "TEEC_OpenSession failed %d\n", res);
        TEEC_FinalizeContext(&mma_ctx.ctx);
        return -EINVAL;
    }

    res = mma_init();
    if (res != TEEC_SUCCESS) {
        printk(KERN_DEBUG "mma_tee_init failed %x\n", res);
        //TEEC_FinalizeContext(&mma_ctx.ctx);
        //return -EINVAL;
    }

    mma_ctx.session_initialized = true;
    return 0;
}

static void mma_destroy_session_1x(void)
{
    if (!mma_ctx.session_initialized)
        return;

    TEEC_CloseSession(&mma_ctx.session);
    TEEC_FinalizeContext(&mma_ctx.ctx);
    mma_ctx.session_initialized = false;
}

static TEEC_Result mma_tee_invoke(TEEC_Session *session,
			       uint32_t commandID,
			       TEEC_Operation *operation,
			       uint32_t *return_origin)
{
    TEEC_Result ret = TEEC_SUCCESS;

    if(1 == optee_version){
        ret = mma_tee_invoke_1x(session, commandID, operation, return_origin);
    }else if(optee_version > 1){
        ret = mma_tee_invoke_2x(session, commandID, operation, return_origin);
    }else{
        printk(KERN_ERR "mma_tee_invoke fail:optee_version = %d\n",optee_version);
        ret = TEEC_ERROR_GENERIC;
    }
    return ret;
}

static int mma_init_session(void)

{
    if(1 == optee_version){
        return mma_init_session_1x();
    }else if(optee_version > 1){
        return mma_init_session_2x();
    }else{
        printk(KERN_ERR "mma_init_session fail:optee_version = %d\n",optee_version);
        return -EINVAL;
    }
}

static void mma_destroy_session(void)
{
    if(1 == optee_version){
        return mma_destroy_session_1x();
    }else if(optee_version > 1){
        return mma_destroy_session_2x();
    }else{
        printk(KERN_ERR "mma_destroy_session fail:optee_version = %d\n",optee_version);
        return;
    }
}

static int mma_init (void){

    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_NONE,
                     TEEC_NONE,
                     TEEC_NONE,
                     TEEC_NONE);

    res = mma_tee_invoke(&mma_ctx.session,TEE_MMA_INIT,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        //printk(KERN_ERR "failed to init mma  res:0x%x\n",res);
        return res;
    }

    return 0;
}


int mma_tee_map(const char* space_tag, struct sg_table *sgt,
                 u64* va_out, u32 _2x, const char *buf_tag)
{
    TEEC_Operation *op;
    TEEC_Result res;
    int ret = -1;
    uint32_t err_origin = 0;
    tee_map_args_v2 *args;
    char tag[MAX_NAME_SIZE] = {0};
    unsigned int i = 0;
    struct mma_range_t *pa_list = NULL;
    struct scatterlist *s;

    if(sgt == NULL || va_out == NULL)
        return -EINVAL;

    ret = mma_init_session();
    CHECK_RETURN(ret);

    args = kzalloc(sizeof(tee_map_args_v2), GFP_KERNEL);
    if (!args)
        return -ENOMEM;
    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op) {
        kfree(args);
        return -ENOMEM;
    }
    pa_list = vmalloc(sgt->nents * sizeof(struct mma_range_t));
    if (!pa_list) {
        kfree(op);
        kfree(args);
        return -ENOMEM;
    }
    args->size = 0;
    args->pa_num = sgt->nents;

    for_each_sg(sgt->sgl, s, sgt->nents, i) {
        pa_list[i].start = page_to_phys(sg_page(s))+s->offset;
        //printk("addr:%llx,szie:%x index %u\n",pa_list[i].start,s->length,i);
        pa_list[i].size = s->length;
        args->size += s->length;
    }
    if(space_tag!=NULL){
        strncpy(tag,space_tag,MAX_NAME_SIZE);
        tag[MAX_NAME_SIZE - 1] = '\0';
    }
    if(buf_tag != NULL){
        strncpy(args->buffer_tag, buf_tag, MAX_NAME_SIZE);
        args->buffer_tag[MAX_NAME_SIZE - 1] = '\0';
    }

    args->type = MMA_ADDR_TYPE_IOVA;
    args->is_secure = 0;
    args->flag = 0;
    if(_2x)
        args->flag |= TEE_MMA_FLAG_2XIOVA;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_MEMREF_TEMP_OUTPUT);

    op->params[0].tmpref.buffer = (uint8_t *)args;
    op->params[0].tmpref.size = sizeof(tee_map_args_v2);
    op->params[1].tmpref.buffer =(uint8_t *)tag;
    op->params[1].tmpref.size = MAX_NAME_SIZE;
    op->params[2].tmpref.buffer = (uint8_t *)pa_list;
    op->params[2].tmpref.size = sgt->nents * sizeof(struct mma_range_t);
    op->params[3].tmpref.buffer = (uint8_t *)va_out;
    op->params[3].tmpref.size = sizeof(uint64_t);

    res = mma_tee_invoke(&mma_ctx.session, MAP_IOVA,
                 op, &err_origin);

    vfree(pa_list);
    kfree(op);
    kfree(args);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  MAP res:0x%x err: 0x%x\n",
               res, err_origin);

        return res;
    }

    return 0;
}

int  mma_tee_unmap(enum mma_addr_type type,uint64_t va,struct mma_range_t *va_out)
{
    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;
    int ret =-1;

    ret = mma_init_session();
	CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_MEMREF_TEMP_INOUT,
                     TEEC_NONE);

    op->params[0].value.a = type;
    op->params[1].tmpref.buffer = (uint8_t *)(&va);
    op->params[1].tmpref.size = sizeof(uint64_t);
    op->params[2].tmpref.buffer = (uint8_t *)(va_out);
    op->params[2].tmpref.size = sizeof(*va_out);

    res = mma_tee_invoke(&mma_ctx.session, UNMAP_IOVA,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  UMAP res:0x%x err: 0x%x\n",
               res, err_origin);

        return res;
    }

    return 0;
}

int  mma_tee_set_mpu_area(enum mma_addr_type type,struct mma_range_t *range, uint8_t num)
{
    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;
    int ret =-1;
   if(range == NULL)
       return -EINVAL;

    ret = mma_init_session();
	CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_NONE,
                     TEEC_NONE);

    op->params[0].value.a = type;
    op->params[1].tmpref.buffer = (uint8_t *)range;
    op->params[1].tmpref.size = num * sizeof(*range);

    res = mma_tee_invoke(&mma_ctx.session, SET_MPU_AREA,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to SetSecure_range  res:0x%x err: 0x%x\n",
               res, err_origin);
        mma_destroy_session();
        return res;
    }

    return 0;
}
int mma_tee_authorize ( u64 va,u32 buffer_size,const char* buf_tag,
                       uint32_t pipe_id)
{
    TEEC_Operation *op;
    TEEC_Result res;
    int ret =-1;
    uint32_t err_origin;
   // char tag[MAX_NAME_SIZE]={0};
    TEE_SvpMmaPipeInfo svp_pipe_info;
    if(buf_tag == NULL )
        return -EINVAL;

    ret = mma_init_session();
	CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    memset(&svp_pipe_info,0,sizeof(svp_pipe_info));
    strncpy(svp_pipe_info.buffer_tag,buf_tag,MAX_NAME_SIZE);
    svp_pipe_info.buffer_tag[MAX_NAME_SIZE - 1] = '\0';
    svp_pipe_info.u32bufferTagLen = MMA_MIN(strlen(buf_tag),MAX_NAME_SIZE);
    svp_pipe_info.u64BufferAddr = va;
    svp_pipe_info.u32BufferSize = buffer_size;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
                     TEEC_VALUE_INPUT,
                     TEEC_NONE,
                     TEEC_NONE);


    op->params[0].tmpref.buffer = (uint8_t *)(&svp_pipe_info);
    op->params[0].tmpref.size = sizeof(svp_pipe_info);
    op->params[1].value.a = pipe_id;

    res = mma_tee_invoke(&mma_ctx.session, VA_AUTHORIZE,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  SET_authorize res:0x%x err: 0x%x\n",
                res, err_origin);

        return res;
    }

    return 0;
}
int mma_tee_unauthorize(u64 va,u32 buffer_size,const char* buf_tag,
                       uint32_t pipe_id,struct mma_range_t *rang_out)

{
    TEEC_Operation *op;
    TEEC_Result res;
    int ret =-1;
    uint32_t err_origin;
    TEE_SvpMmaPipeInfo svp_pipe_info;

    if(buf_tag == NULL||rang_out == NULL )
        return -EINVAL;

    ret = mma_init_session();
	CHECK_RETURN(ret);
    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    memset(&svp_pipe_info,0,sizeof(svp_pipe_info));
    strncpy(svp_pipe_info.buffer_tag,buf_tag,MAX_NAME_SIZE);
    svp_pipe_info.buffer_tag[MAX_NAME_SIZE - 1] = '\0';
    svp_pipe_info.u32bufferTagLen = MMA_MIN(strlen(buf_tag),MAX_NAME_SIZE);
    svp_pipe_info.u64BufferAddr = va;
    svp_pipe_info.u32BufferSize = buffer_size;


    op->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
                     TEEC_VALUE_INPUT,
                     TEEC_MEMREF_TEMP_OUTPUT,
                     TEEC_NONE);

    op->params[0].tmpref.buffer = (uint8_t *)(&svp_pipe_info);
    op->params[0].tmpref.size = sizeof(svp_pipe_info);
    op->params[1].value.a = pipe_id;
    op->params[2].tmpref.buffer = (uint8_t *)(rang_out);
    op->params[2].tmpref.size = sizeof(*rang_out);

    res = mma_tee_invoke(&mma_ctx.session, VA_UNAUTHORIZE,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  SET_unauthorize res:0x%x err: 0x%x\n",
                res, err_origin);

        return res;
    }

    return 0;
}
int  mma_tee_reserve_space(enum mma_addr_type type,const char  *space_tag,
                         u64 size,u64 *va_out)
{
    TEEC_Operation *op;
    TEEC_Result res;
    int ret =-1;
    uint32_t err_origin;
    char tag[MAX_NAME_SIZE]={0};

    if(space_tag == NULL || va_out == NULL)
        return -EINVAL;

    ret = mma_init_session();
	CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    strncpy(tag,space_tag,MAX_NAME_SIZE);
    tag[MAX_NAME_SIZE - 1] = '\0';
    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_MEMREF_TEMP_OUTPUT);

    op->params[0].value.a = type;
    op->params[1].tmpref.buffer = (uint8_t *)tag;
    op->params[1].tmpref.size = MAX_NAME_SIZE;
    op->params[2].tmpref.buffer = (uint8_t *)(&size);
    op->params[2].tmpref.size = sizeof(uint64_t);
    op->params[3].tmpref.buffer = (uint8_t *)va_out;
    op->params[3].tmpref.size = sizeof(uint64_t);

    res = mma_tee_invoke(&mma_ctx.session, RESERVE_IOVA_SPACE,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  RESERVE res:0x%x err: 0x%x\n",
               res, err_origin);

        return res;
    }

    return 0;
}
int mma_tee_free_space (enum mma_addr_type type, const char* space_tag){

    TEEC_Operation *op;
    TEEC_Result res;
    int ret =-1;
    uint32_t err_origin;
    char tag[MAX_NAME_SIZE] = {0};

   if(space_tag == NULL)
       return -EINVAL;

    ret = mma_init_session();
	CHECK_RETURN(ret);


    strncpy(tag,space_tag,MAX_NAME_SIZE);
    tag[MAX_NAME_SIZE - 1] = '\0';
    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT,
                     TEEC_MEMREF_TEMP_INPUT,
                     TEEC_NONE,
                     TEEC_NONE);


    op->params[0].value.a = type;
    op->params[1].tmpref.buffer = (uint8_t *)space_tag;
    op->params[1].tmpref.size = MAX_NAME_SIZE;

    res = mma_tee_invoke(&mma_ctx.session,FREE_IOVA_SPACE,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to free_space  res:0x%x err: 0x%x\n",
               res, err_origin);

        return res;
    }

    return 0;
}

int  mma_tee_debug(uint32_t debug_type,uint8_t *buf1,uint8_t size1,
                       uint8_t *buf2,uint8_t size2,uint8_t *buf3,uint8_t size3)
{
    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;
    int ret =-1;
    ret = mma_init_session();
	CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;


    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT,
                     TEEC_NONE,
                     TEEC_NONE,
                     TEEC_NONE);


    op->params[0].value.a = debug_type;
    op->params[1].tmpref.buffer = (uint8_t *)buf1;
    op->params[1].tmpref.size = size1;
    op->params[2].tmpref.buffer = (uint8_t *)buf2;
    op->params[2].tmpref.size = size2;
    op->params[3].tmpref.buffer = (uint8_t *)buf3;
    op->params[3].tmpref.size = size3;


    res = mma_tee_invoke(&mma_ctx.session, DEBUG,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  DEBUG res:0x%x err: 0x%x\n",
               res, err_origin);

        return res;
    }

    return 0;
}

int mma_tee_lockdebug(tee_map_lock *lock)
{
    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin = 0;
    int ret = -1;

    CHECK_POINTER(lock,-ENOMEM);

    ret = mma_init_session();
    CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
                     TEEC_NONE,
                     TEEC_NONE,
                     TEEC_NONE);


    op->params[0].tmpref.buffer = (uint8_t *)lock;
    op->params[0].tmpref.size = sizeof(tee_map_lock);
	printk(KERN_ERR "LOCK_DEBUG tag=%s,num=%d,aid=%d,%d,%d,%d,%d,%d,%d,%d\n",
        lock->buffer_tag,lock->aid_num,lock->aid[0],lock->aid[1],lock->aid[2],
        lock->aid[3],lock->aid[4],lock->aid[5],lock->aid[6],lock->aid[7]);

    res = mma_tee_invoke(&mma_ctx.session, LOCK_DEBUG,
                 op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  LOCK_DEBUG res:0x%x err: 0x%x\n",
               res, err_origin);

        return res;
    }

    return 0;
}


int mma_tee_get_pipeid (int* pipeid){

    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;
    int ret =-1;
    ret = mma_init_session();
	CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_OUTPUT,
                     TEEC_NONE,
                     TEEC_NONE,
                     TEEC_NONE);

    res = mma_tee_invoke(&mma_ctx.session,GENERATE_PIPELINE_ID,
                 op, &err_origin);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to get pipeid  res:0x%x err: 0x%x\n",
               res, err_origin);
        kfree(op);
        return res;
    }
    *pipeid = op->params[0].value.a;
    printk("mma_tee_get_pipeid: pipeid =%d\n",*pipeid);
    kfree(op);
    return 0;
}

int mma_tee_put_pipeid(int pipeid) {
    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;
    int ret = -1;

    ret = mma_init_session();
    CHECK_RETURN(ret);

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INOUT, TEEC_NONE, TEEC_NONE, TEEC_NONE);
    op->params[0].value.a = pipeid;

    res = mma_tee_invoke(&mma_ctx.session,RELEASE_PIPELINE_ID, op, &err_origin);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to release pipeid  res:0x%x err: 0x%x\n", res, err_origin);
        return res;
    }

    return 0;
}

int mma_tee_pipelineID_query (u64 addr,uint32_t* pipeline_id)
{
    /*TEEC_Operation op;
    TEEC_Result res;
    int ret =-1;
    uint32_t err_origin;
    TEE_SvpMmaPipeInfo svp_pipe_info;

    ret = mma_init_session();
    CHECK_RETURN(ret);

    memset(&svp_pipe_info,0,sizeof(svp_pipe_info));
    svp_pipe_info.u64BufferAddr = addr;

    memset(&op, 0, sizeof(op));

    op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
                     TEEC_VALUE_OUTPUT,
                     TEEC_NONE,
                     TEEC_NONE);

    op.params[0].tmpref.buffer = (uint8_t *)(&svp_pipe_info);
    op.params[0].tmpref.size = sizeof(svp_pipe_info);

    res = mma_tee_invoke(&mma_ctx.session, QUERY_PIPELINE_ID,
                 &op, &err_origin);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  SET_authorize res:0x%x err: 0x%x\n",
                res, err_origin);

        return res;
    }
    *pipeline_id = op.params[1].value.a;*/
    return 0;
}

static struct buf_tag *dump_buftags(int *num)
{
    struct list_head *head = NULL,*pos = NULL;
    buf_tag_info *buf=NULL;
    buf_tag *ret_tags = NULL;
    int count=0;

    head = mma_get_buftags();
    if(!head || list_empty(head))
       return NULL;

    list_for_each(pos,head){
	count++;
    }
    ret_tags = (struct buf_tag *)vmalloc(sizeof(struct buf_tag)*count);
    if(!ret_tags)
       return NULL;

    count = 0;
    list_for_each_entry(buf,head,list){
        ret_tags[count].heap_type = buf->heap_type;
        if(ret_tags[count].heap_type == HEAP_TYPE_CARVEOUT)
            ret_tags[count].heap_type = HEAP_TYPE_IOMMU;
        ret_tags[count].miu = buf->miu;
        ret_tags[count].maxsize = buf->maxsize;
        memcpy(ret_tags[count].name,buf->name,64);
        count++;
    }
    *num = count;
    return ret_tags;
}
int mma_optee_ta_store_buf_tags()
{
    TEEC_Operation *op;
    TEEC_Result res;
    uint32_t err_origin;
    struct buf_tag *buf =NULL;
    int size = 0;
    int ret =-1;
    buf = dump_buftags(&size);
    if(!buf)
        return 0;

    ret = mma_init_session();
    if(ret < 0){
        vfree(buf);
        return -1;
    }

    op = kzalloc(sizeof(*op), GFP_KERNEL);
    if (!op)
        return -ENOMEM;

    op->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
                     TEEC_NONE,
                     TEEC_NONE,
                     TEEC_NONE);


    op->params[0].tmpref.buffer = (uint8_t *)buf;
    op->params[0].tmpref.size = size*sizeof(struct buf_tag);


    res = mma_tee_invoke(&mma_ctx.session, STORE_BUF_TAGS,
                 op, &err_origin);

    vfree(buf);
    kfree(op);
    if (res != TEEC_SUCCESS) {
        printk(KERN_ERR "failed to  STORE_BUF_TAGS res:0x%x err: 0x%x\n",
               res, err_origin);
        return res;
    }
    return 0;

}

/* test */
static int mma_optee_ta_dump_status(struct seq_file *s, void *unused)
{
#if 0
    struct  mma_range_t iova_rg,pa_list;
    uint64_t iova=0;
    printk("%s\n",__FUNCTION__);
    uint32_t type=0;
    uint8_t buf1[2],buf2[2],buf3[2];

    TEEC_UUID pipe_id;
    uint8_t buffer_tag[2];
    struct sg_table sgt;
        mma_tee_map(1,"123",&sgt,1,&iova);
        mma_tee_debug(type,buf1,2,buf2,2,buf3,2);
        mma_tee_authorize(1,123,"123","1234");
        mma_tee_umap(1,iova,&pa_list);
        mma_tee_set_secure_range(1,&iova_rg,2);
        mma_tee_free_space(1,"123");
        mma_tee_reserve_space(1,buffer_tag,2,&iova);
#endif
        return 0;
}

static int mma_optee_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, mma_optee_ta_dump_status, inode->i_private);
}

static const struct file_operations mma_debug_fops = {
    .open    = mma_optee_debug_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release,
};
int mma_tee_open_session(void){
    mma_ctx.debug_root = debugfs_create_dir("mma-optee", NULL);
    debugfs_create_file("dump", S_IRUGO, mma_ctx.debug_root,
                &mma_ctx, &mma_debug_fops);

    mma_ctx.session_initialized = false;

    mma_init_session();
    return 0;
}

int mma_tee_close_session(bool closed){
    mma_destroy_session();
    if(closed)
        debugfs_remove_recursive(mma_ctx.debug_root);
    return 0;

}
#if 0
static int __init mma_optee_init(void)
{
    //mutex_init(&mma_ctx.lock);

    mma_ctx.debug_root = debugfs_create_dir("mma-optee", NULL);
    debugfs_create_file("dump", S_IRUGO, mma_ctx.debug_root,
                &mma_ctx, &mma_debug_fops);

    mma_ctx.session_initialized = false;
    return 0;
}
module_init(mma_optee_init);

static void __exit mma_optee_exit(void)
{

    mma_destroy_session();
    debugfs_remove_recursive(mma_ctx.debug_root);
}
module_exit(mma_optee_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMA for OP-TEE");
MODULE_AUTHOR("Mstar");
#endif
