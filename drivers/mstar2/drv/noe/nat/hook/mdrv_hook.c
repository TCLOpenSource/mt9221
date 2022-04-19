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
/// @file   hook.c
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------



#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include "mdrv_hwnat.h"

//--------------------------------------------------------------------------------------------------
//  Global Variable
//--------------------------------------------------------------------------------------------------


struct net_device   *dst_port[MAX_IF_NUM];
EXPORT_SYMBOL(dst_port);

struct foe_entry *ppe_virt_foe_base_tmp;
EXPORT_SYMBOL(ppe_virt_foe_base_tmp);

int (*noe_nat_hook_rx)(struct sk_buff *skb) = NULL;
EXPORT_SYMBOL(noe_nat_hook_rx);

int (*noe_nat_hook_tx)(struct sk_buff *skb, int gmac_no) = NULL;
EXPORT_SYMBOL(noe_nat_hook_tx);

void (*ppe_dev_register_hook)(struct net_device *dev);
EXPORT_SYMBOL(ppe_dev_register_hook);

void (*ppe_dev_unregister_hook)(struct net_device *dev);
EXPORT_SYMBOL(ppe_dev_unregister_hook);

//--------------------------------------------------------------------------------------------------
//  Global Function
//--------------------------------------------------------------------------------------------------



void  MDrv_NOE_NAT_Set_Magic_Tag_Zero(struct sk_buff *skb)
{
    if ((FOE_MAGIC_TAG_HEAD(skb) == FOE_MAGIC_PCI) ||
        (FOE_MAGIC_TAG_HEAD(skb) == FOE_MAGIC_WLAN) ||
        (FOE_MAGIC_TAG_HEAD(skb) == FOE_MAGIC_GE)) {
        if (IS_SPACE_AVAILABLE_HEAD(skb))
            FOE_MAGIC_TAG_HEAD(skb) = 0;
    }
    if ((FOE_MAGIC_TAG_TAIL(skb) == FOE_MAGIC_PCI) ||
        (FOE_MAGIC_TAG_TAIL(skb) == FOE_MAGIC_WLAN) ||
        (FOE_MAGIC_TAG_TAIL(skb) == FOE_MAGIC_GE)) {
        if (IS_SPACE_AVAILABLE_TAIL(skb))
            FOE_MAGIC_TAG_TAIL(skb) = 0;
    }
}

void MDrv_NOE_NAT_Check_Magic_Tag(struct sk_buff *skb)
{
    if (IS_SPACE_AVAILABLE_HEAD(skb)) {
        FOE_MAGIC_TAG_HEAD(skb) = 0;
        FOE_AI_HEAD(skb) = UN_HIT;
    }
    if (IS_SPACE_AVAILABLE_TAIL(skb)) {
        FOE_MAGIC_TAG_TAIL(skb) = 0;
        FOE_AI_TAIL(skb) = UN_HIT;
    }
}

void MDrv_NOE_NAT_Set_Headroom_Zero(struct sk_buff *skb)
{
    if (skb->cloned != 1) {
        if (IS_MAGIC_TAG_PROTECT_VALID_HEAD(skb) ||
            (FOE_MAGIC_TAG(skb) == FOE_MAGIC_PPE)) {
            if (IS_SPACE_AVAILABLE_HEAD(skb))
                memset(FOE_INFO_START_ADDR_HEAD(skb), 0, FOE_INFO_LEN);
        }
    }
}

void MDrv_NOE_NAT_Set_Tailroom_Zero(struct sk_buff *skb)
{
    if (skb->cloned != 1) {
        if (IS_MAGIC_TAG_PROTECT_VALID_TAIL(skb) ||
            (FOE_MAGIC_TAG(skb) == FOE_MAGIC_PPE)) {
            if (IS_SPACE_AVAILABLE_TAIL(skb))
                memset(FOE_INFO_START_ADDR_TAIL(skb), 0, FOE_INFO_LEN);
        }
    }
}

void MDrv_NOE_NAT_Copy_Headroom(u8 *data, struct sk_buff *skb)
{
    memcpy(data, skb->head, FOE_INFO_LEN);
}

void MDrv_NOE_NAT_Copy_Tailroom(u8 *data, int size, struct sk_buff *skb)
{
    memcpy((data + size - FOE_INFO_LEN), (skb_end_pointer(skb) - FOE_INFO_LEN), FOE_INFO_LEN);
}

void MDrv_NOE_NAT_Set_Dma_Ops(struct device *dev, bool coherent)
{
#if defined(CONFIG_ARM64)
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
    if(coherent)
        set_dma_ops(dev, &coherent_swiotlb_dma_ops);
    else
        set_dma_ops(dev, &noncoherent_swiotlb_dma_ops);
    #else
    arch_setup_dma_ops(dev, 0, 0, NULL, coherent);
    #endif
#endif
}

EXPORT_SYMBOL(MDrv_NOE_NAT_Set_Dma_Ops);

