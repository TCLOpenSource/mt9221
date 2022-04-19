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
/// @file   MDRV_NOE_DMA.h
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef _MDRV_NOE_DMA_H_
#define _MDRV_NOE_DMA_H_




struct parse_result {
    /* layer2 header */
    u8 dmac[6];
    u8 smac[6];

    /* vlan header */
    u16 vlan_tag;
    u16 vlan1_gap;
    u16 vlan1;
    u16 vlan2_gap;
    u16 vlan2;
    u16 vlan_layer;

    /* pppoe header */
    u32 pppoe_gap;
    u16 ppp_tag;
    u16 pppoe_sid;

    /* layer3 header */
    u16 eth_type;
    struct iphdr iph;
    struct ipv6hdr ip6h;

    /* layer4 header */
    struct tcphdr th;
    struct udphdr uh;

    u32 pkt_type;
    u8 is_mcast;
};

int MDrv_NOE_PDMA_Init_Rx(struct net_device *dev);
int MDrv_NOE_PDMA_Init_Tx(struct net_device *dev);
void MDrv_NOE_PDMA_Deinit_Rx(struct net_device *dev);
void MDrv_NOE_PDMA_Deinit_Tx(struct net_device *dev);
int MDrv_NOE_PDMA_Start_Xmit(struct sk_buff *skb, struct net_device *dev, int gmac_no);
int MDrv_NOE_PDMA_Xmit_Housekeeping(struct net_device *netdev, int budget);

int MDrv_NOE_QDMA_Init_Rx(struct net_device *dev);
int MDrv_NOE_QDMA_Init_Tx(struct net_device *dev);
void MDrv_NOE_QDMA_Deinit_Rx(struct net_device *dev);
void MDrv_NOE_QDMA_Deinit_Tx(struct net_device *dev);
int MDrv_NOE_QDMA_Start_Xmit(struct sk_buff *skb, struct net_device *dev, int gmac_no);
int MDrv_NOE_QDMA_Xmit_Housekeeping(struct net_device *netdev, int budget);
int MDrv_NOE_QDMA_Ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);


#endif /* _MDRV_NOE_DMA_H_ */
