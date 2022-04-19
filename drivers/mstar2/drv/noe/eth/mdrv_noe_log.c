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
/// @file   MDRV_NOE_LOG.c
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include "mdrv_noe.h"

//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------

#define SKB_DUMP_LAYER_II  0
#if SKB_DUMP_LAYER_II
#define SKB_START_OFFSET (-14) /* dump Layer II*/
#else
#define SKB_START_OFFSET (0)  /* dump Layer III*/
#endif

#define SKB_END_OFFSET  (32) // (40)



//--------------------------------------------------------------------------------------------------
//  Local Variable
//--------------------------------------------------------------------------------------------------

static struct net_device *_noe_dev = NULL;

void MDrv_NOE_LOG_Set_Level(unsigned char level)
{
    struct END_DEVICE *ei_local = NULL;

    if (_noe_dev == NULL)
        return;

    ei_local = netdev_priv(_noe_dev);

    if (ei_local->log_level == level)
        return;

    ei_local->log_level = level;
    NOE_MSG_DBG("Set Log Level = 0x%x \n", ei_local->log_level);
    if (level >= E_MDRV_NOE_MSG_CTRL_DUMP) {
        MHal_NOE_Set_DBG(E_NOE_HAL_LOG_DBG);
    }
    else {
        MHal_NOE_Set_DBG(E_NOE_HAL_LOG_NONE);
    }

}

unsigned char MDrv_NOE_LOG_Get_Level(void)
{
    struct END_DEVICE *ei_local = NULL;

    if (_noe_dev == NULL)
        return E_MDRV_NOE_MSG_CTRL_NONE;

    ei_local = netdev_priv(_noe_dev);

    return ei_local->log_level;
}

void MDrv_NOE_LOG_Init(struct net_device *dev)
{
    _noe_dev = dev;
}

void MDrv_NOE_LOG_Dump_Skb(struct sk_buff* sk)
{
    int num = sk->len; //(sk->len < SKB_END_OFFSET)? sk->len : SKB_END_OFFSET;
    unsigned char *i;
    unsigned int j = 0;
    NOE_MSG_DUMP("[%s][%d] =========> \n",__FUNCTION__,__LINE__);
#if 0
    NOE_MSG_DUMP("skb_dump: from %s with len %d (%d) headroom=%d tailroom=%d\n",
            sk->dev? sk->dev->name:"ip stack", sk->len, sk->truesize,
            skb_headroom(sk),skb_tailroom(sk));
#endif
    NOE_MSG_DUMP("%s: head = 0x%p, data = 0x%p with len %d (%d)\n",
            sk->dev? sk->dev->name:" ",
            sk->head, sk->data, sk->len, sk->truesize);
    for(i = sk->data + SKB_START_OFFSET; i < sk->data + num; i++) {

        if ((j % 16) == 8)
            NOE_MSG_DUMP("\t");
        if ((j % 16) == 0)
            NOE_MSG_DUMP("\n");

        NOE_MSG_DUMP("%02x ", *(sk->data + j));
        j++;
    }
    NOE_MSG_DUMP("\n");
}

