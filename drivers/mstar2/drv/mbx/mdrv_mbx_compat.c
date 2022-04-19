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
/// file    mdrv_mbx_io.c
/// @brief  MS MailBox ioctrol driver
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//=============================================================================
// Include Files
//=============================================================================
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/string.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

//drver header files
#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"
#include "mdrv_mbx_io.h"
#include "mst_devid.h"
#if defined(CONFIG_COMPAT)
#include "mdrv_mbx_compat_st.h"
#endif


int mdrv_mbx_sendmsg_compat_get_ion_allaction_data(COMPAT_MS_MBX_SEND_MSG __user *data32, MS_MBX_SEND_MSG __user *data)
{
	COMPAT_MS_MBX_SEND_MSG temp;
	MBX_Msg *ptr_msg;
	int err;

	err = get_user(temp.mbxResult, &data32->mbxResult);
	err = get_user(ptr_msg, &data32->pMsg);
	temp.pMsg = compat_ptr(ptr_msg);

	err = put_user(temp.mbxResult, &data->mbxResult);
	err = put_user(temp.pMsg, &data->pMsg);

	return err;
}

int mdrv_mbx_sendmsg_compat_put_ion_allaction_data(COMPAT_MS_MBX_SEND_MSG __user *data32, MS_MBX_SEND_MSG __user *data)
{
	MS_MBX_SEND_MSG temp;
	int err;

	err = get_user(temp.mbxResult, &data->mbxResult);
	err = put_user(temp.mbxResult, &data32->mbxResult);
	return err;
}

int mdrv_mbx_recvmsg_compat_get_ion_allaction_data(COMPAT_MS_MBX_RECV_MSG __user *data32, MS_MBX_RECV_MSG __user *data)
{
    COMPAT_MS_MBX_RECV_MSG temp;
	MBX_Msg *ptr_msg;
	int err;

	err = get_user(temp.eTargetClass, &data32->eTargetClass);
	err = get_user(temp.u32WaitMillSecs, &data32->u32WaitMillSecs);
	err = get_user(temp.u32Flag, &data32->u32Flag);
	err = get_user(temp.mbxResult, &data32->mbxResult);
	err = get_user(ptr_msg, &data32->pMsg);
	temp.pMsg = compat_ptr(ptr_msg);

	err = put_user(temp.eTargetClass, &data->eTargetClass);
	err = put_user(temp.u32WaitMillSecs, &data->u32WaitMillSecs);
	err = put_user(temp.u32Flag, &data->u32Flag);
	err = put_user(temp.mbxResult, &data->mbxResult);
	err = put_user(temp.pMsg, &data->pMsg);

	return err;
}

int mdrv_mbx_recvmsg_compat_put_ion_allaction_data(COMPAT_MS_MBX_RECV_MSG __user *data32, MS_MBX_RECV_MSG __user *data)
{
	MS_MBX_RECV_MSG temp = {};
	int err;

	err = get_user(temp.eTargetClass, &data->eTargetClass);
	err = get_user(temp.u32WaitMillSecs, &data->u32WaitMillSecs);
	err = get_user(temp.u32Flag, &data->u32Flag);
	err = get_user(temp.mbxResult, &data->mbxResult);

	err = put_user(temp.u32WaitMillSecs, &data32->u32WaitMillSecs);
	err = put_user(temp.u32Flag, &data32->u32Flag);
	err = put_user(temp.mbxResult, &data32->mbxResult);
	err = put_user(temp.pMsg, &data32->pMsg);

	return err;
}

int mdrv_mbx_init_compat_get_ion_allaction_data(COMPAT_MS_MBX_INIT_INFO __user *data32, MS_MBX_INIT_INFO __user *data)
{
    COMPAT_MS_MBX_INIT_INFO temp;
	int err;

	err = get_user(temp.eHKCPU, &data32->eHKCPU);
	err = get_user(temp.eHostRole, &data32->eHostRole);
	err = get_user(temp.u32TimeoutMillSecs, &data32->u32TimeoutMillSecs);
	err = get_user(temp.mbxResult, &data32->mbxResult);

	err = put_user(temp.eHKCPU, &data->eHKCPU);
	err = put_user(temp.eHostRole, &data->eHostRole);
	err = put_user(temp.u32TimeoutMillSecs, &data->u32TimeoutMillSecs);
	err = put_user(temp.mbxResult, &data->mbxResult);

	return err;
}

int mdrv_mbx_init_compat_put_ion_allaction_data(COMPAT_MS_MBX_INIT_INFO __user *data32, MS_MBX_INIT_INFO __user *data)
{
	MS_MBX_INIT_INFO temp;
	int err;

	err = get_user(temp.eHKCPU, &data->eHKCPU);
	err = get_user(temp.eHostRole, &data->eHostRole);
	err = get_user(temp.u32TimeoutMillSecs, &data->u32TimeoutMillSecs);
	err = get_user(temp.mbxResult, &data->mbxResult);

	err = put_user(temp.eHKCPU, &data32->eHKCPU);
	err = put_user(temp.eHostRole, &data32->eHostRole);
	err = put_user(temp.u32TimeoutMillSecs, &data32->u32TimeoutMillSecs);
	err = put_user(temp.mbxResult, &data32->mbxResult);

	return err;
}

int mdrv_mbx_register_msg_compat_get_ion_allaction_data(COMPAT_MS_MBX_REGISTER_MSG __user *data32, MS_MBX_REGISTER_MSG __user *data)
{
    COMPAT_MS_MBX_REGISTER_MSG temp;
	int err;

	err = get_user(temp.eMsgClass, &data32->eMsgClass);
	err = get_user(temp.u16MsgQueueSize, &data32->u16MsgQueueSize);
	err = get_user(temp.mbxResult, &data32->mbxResult);

	err = put_user(temp.eMsgClass, &data->eMsgClass);
	err = put_user(temp.u16MsgQueueSize, &data->u16MsgQueueSize);
	err = put_user(temp.mbxResult, &data->mbxResult);

	return err;
}

int mdrv_mbx_register_msg_compat_put_ion_allaction_data(COMPAT_MS_MBX_REGISTER_MSG __user *data32, MS_MBX_REGISTER_MSG __user *data)
{
	MS_MBX_REGISTER_MSG temp;
	int err;

	err = get_user(temp.eMsgClass, &data->eMsgClass);
	err = get_user(temp.u16MsgQueueSize, &data->u16MsgQueueSize);
	err = get_user(temp.mbxResult, &data->mbxResult);

	err = put_user(temp.eMsgClass, &data32->eMsgClass);
	err = put_user(temp.u16MsgQueueSize, &data32->u16MsgQueueSize);
	err = put_user(temp.mbxResult, &data32->mbxResult);

	return err;
}

int mdrv_mbx_get_msgqstatus_compat_get_ion_allaction_data(COMPAT_MS_MBX_GET_MSGQSTATUS __user *data32, MS_MBX_GET_MSGQSTATUS __user *data)
{
    COMPAT_MS_MBX_GET_MSGQSTATUS temp;
	COMPAT_MBX_MSGQ_Status *ptr_msg;
	int err;

	err = get_user(temp.eTargetClass, &data32->eTargetClass);
	err = get_user(temp.mbxResult, &data32->mbxResult);
	err = get_user(ptr_msg, &data32->pMsgQueueStatus);
	temp.pMsgQueueStatus = compat_ptr(ptr_msg);

	err = put_user(temp.eTargetClass, &data->eTargetClass);
	err = put_user(temp.mbxResult, &data->mbxResult);
	err = put_user(temp.pMsgQueueStatus, &data->pMsgQueueStatus);

	return err;
}

int mdrv_mbx_get_msgqstatus_compat_put_ion_allaction_data(COMPAT_MS_MBX_GET_MSGQSTATUS __user *data32, MS_MBX_GET_MSGQSTATUS __user *data)
{
	MS_MBX_GET_MSGQSTATUS temp;
	int err;

	err = get_user(temp.eTargetClass, &data->eTargetClass);
	err = get_user(temp.mbxResult, &data->mbxResult);

	err = put_user(temp.eTargetClass, &data32->eTargetClass);
	err = put_user(temp.mbxResult, &data32->mbxResult);

	return err;
}

int mdrv_mbx_crosync_info_compat_get_ion_allaction_data(COMPAT_MS_MBX_CPROSYNC_INFORMATION __user *data32, MS_MBX_CPROSYNC_INFORMATION __user *data)
{
	COMPAT_MS_MBX_CPROSYNC_INFORMATION temp;
	MS_U8 *ptr_msg;
	int err;

	err = get_user(temp.eTargetRole, &data32->eTargetRole);
	err = get_user(temp.u8Size, &data32->u8Size);
	err = get_user(temp.mbxResult, &data32->mbxResult);
	err = get_user(ptr_msg, &data32->pU8Info);
	temp.pU8Info = compat_ptr(ptr_msg);

	err = put_user(temp.eTargetRole, &data->eTargetRole);
	err = put_user(temp.u8Size, &data->u8Size);
	err = put_user(temp.mbxResult, &data->mbxResult);
	err = put_user(temp.pU8Info, &data->pU8Info);

	return err;
}

int mdrv_mbx_crosync_info_compat_put_ion_allaction_data(COMPAT_MS_MBX_CPROSYNC_INFORMATION __user *data32, MS_MBX_CPROSYNC_INFORMATION __user *data)
{
	MS_MBX_CPROSYNC_INFORMATION temp;
	int err;

	err = get_user(temp.eTargetRole, &data->eTargetRole);
	err = get_user(temp.u8Size, &data->u8Size);
	err = get_user(temp.mbxResult, &data->mbxResult);


	err = put_user(temp.eTargetRole, &data32->eTargetRole);
    err = put_user(temp.u8Size, &data32->u8Size);
	err = put_user(temp.mbxResult, &data32->mbxResult);

	return err;
}

int mdrv_mbx_get_drvstatus_compat_get_ion_allaction_data(COMPAT_MS_MBX_GET_DRVSTATUS __user *data32, MS_MBX_GET_DRVSTATUS __user *data)
{
    COMPAT_MS_MBX_GET_DRVSTATUS temp;
	int err;

	err = get_user(temp.bEnabled, &data32->bEnabled);
	err = get_user(temp.s32RefCnt, &data32->s32RefCnt);

	err = put_user(temp.bEnabled, &data->bEnabled);
	err = put_user(temp.s32RefCnt, &data->s32RefCnt);

	return err;
}

int mdrv_mbx_get_drvstatus_compat_put_ion_allaction_data(COMPAT_MS_MBX_GET_DRVSTATUS __user *data32, MS_MBX_GET_DRVSTATUS __user *data)
{
	MS_MBX_GET_DRVSTATUS temp;
	int err;

	err = get_user(temp.bEnabled, &data->bEnabled);
	err = get_user(temp.s32RefCnt, &data->s32RefCnt);

	err = put_user(temp.bEnabled, &data32->bEnabled);
	err = put_user(temp.s32RefCnt, &data32->s32RefCnt);

	return err;
}


EXPORT_SYMBOL(mdrv_mbx_sendmsg_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_sendmsg_compat_put_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_recvmsg_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_recvmsg_compat_put_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_init_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_init_compat_put_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_register_msg_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_register_msg_compat_put_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_get_msgqstatus_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_get_msgqstatus_compat_put_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_crosync_info_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_crosync_info_compat_put_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_get_drvstatus_compat_get_ion_allaction_data);
EXPORT_SYMBOL(mdrv_mbx_get_drvstatus_compat_put_ion_allaction_data);


