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
/// @file   MDRV_NOE.h
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_NOE_H_
#define _MDRV_NOE_H_
//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/if_ether.h>
#include <linux/fs.h>
#include <linux/mii.h>
#include <linux/uaccess.h>
#include <linux/tcp.h>
#include <net/ipv6.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <linux/in.h>
#include <linux/ppp_defs.h>
#include <linux/if_pppox.h>
#include <linux/netdevice.h>
#include <linux/if_vlan.h>
#include <linux/ppp_defs.h>
#include <linux/delay.h>
#include <linux/sched.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
/* LRO support */
#include <linux/inet_lro.h>
#include <asm-generic/pci-dma-compat.h>
#else
#include <linux/pci-dma-compat.h>
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION... */

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_net.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <linux/dma-mapping.h>

#include <linux/kthread.h>
#include <linux/prefetch.h>

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>


#include "mdrv_noe_def.h"
#include "mdrv_mstypes.h"
#include "mhal_noe.h"
#include "mhal_noe_dma.h"
#include "mdrv_noe_dma.h"
#include "mdrv_noe_config.h"
#include "mdrv_noe_proc.h"
#include "mdrv_noe_lro.h"
#include "mdrv_noe_mac.h"
#include "mdrv_noe_ioctl.h"
#include "mdrv_noe_utils.h"
#include "mdrv_noe_log.h"

//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------
#define FE_RESET_POLLING_MS (5000)

#define DEV_NAME        "eth2"
#define DEV2_NAME       "eth3"

#define IP4_ADDR_LEN        16
#define FE_DEFAULT_LAN_IP   "10.10.10.254"
#define NOE_NAPI_WEIGHT 64

#define NOE_DRV_STRING "NOE_DRV"
#define NOE_COMPATIBLE_DEV_ID "eth,noe"
#define NOE_COMPATIBLE_SWITCH "noe,switch"
#define NOE_COMPATIBLE_ETHSYS "noe,ethsys"


#ifdef CONFIG_MSTAR_ARM_BD_FPGA
#ifdef NOE_CFG_PIN_MUX_SEL
#undef NOE_CFG_PIN_MUX_SEL
#define NOE_CFG_PIN_MUX_SEL (E_NOE_SEL_PIN_MUX_GE1_TO_PM_GE2_TO_CHIPTOP)
#endif
#define FPGA_NOE_MAC_SPEED E_NOE_SPEED_100  // SPEED_1000
#define FPGA_NOE_PHY_SPEED E_NOE_SPEED_10
#define FPGA_NOE_PHY_DUPLEX E_NOE_DUPLEX_FULL
#endif /* CONFIG_MSTAR_ARM_BD_FPGA */


//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------

enum noe_clks_map {
    E_NOE_CLK_ETHIF,
    E_NOE_CLK_ESW,
    E_NOE_CLK_GP0,
    E_NOE_CLK_GP1,
    E_NOE_CLK_GP2,
    E_NOE_CLK_SGMII_TX250M,
    E_NOE_CLK_SGMII_RX250M,
    E_NOE_CLK_SGMII_CDR_REF,
    E_NOE_CLK_SGMII_CDR_FB,
    E_NOE_CLK_TRGPLL,
    E_NOE_CLK_SGMIPLL,
    E_NOE_CLK_ETH1PLL,
    E_NOE_CLK_ETH2PLL,
    E_NOE_CLK_MAX
};

struct lro_counters {
    u32 lro_aggregated;
    u32 lro_flushed;
    u32 lro_no_desc;
};

struct lro_para_struct {
    unsigned int lan_ip1;
};

struct PSEUDO_ADAPTER {
    struct net_device *primary_dev;
    struct net_device *pseudo_dev;
    struct net_device_stats stat;
    struct mii_if_info mii_info;
};


struct noe_stats {
    unsigned int min_free_txd;
};


struct END_DEVICE {
    struct device *dev;
    unsigned int tx_cpu_owner_idx0;
#ifdef CONFIG_NOE_RW_PDMAPTR_FROM_VAR
    unsigned int rx_calc_idx[MAX_RX_RING_NUM];
#endif
    unsigned int tx_ring_full;
    unsigned int tx_full;   /* NOTE(Nelso): unused, can remove */

    /* PDMA TX  PTR */
    dma_addr_t phy_tx_ring0;

    /* QDMA TX  PTR */
    struct platform_device *qdma_pdev;
    struct sk_buff *free_skb[NUM_TX_DESC];
    unsigned int tx_dma_ptr;
    unsigned int tx_cpu_ptr;
    unsigned int tx_cpu_idx;
    unsigned int rls_cpu_idx;
    atomic_t  free_txd_num;
    unsigned int free_txd_head;
    unsigned int free_txd_tail;
    struct QDMA_txdesc *txd_pool;
    dma_addr_t phy_txd_pool;
    unsigned int txd_pool_info[NUM_TX_DESC];
    struct QDMA_txdesc *free_head;
    unsigned int phy_free_head;
    unsigned int *free_page_head;
    dma_addr_t phy_free_page_head;
    struct PDMA_rxdesc *qrx_ring;
    dma_addr_t phy_qrx_ring;

    /* TSO */
    unsigned int skb_txd_num;

    /* MT7623 workaround */
    struct work_struct reset_task;

    /* workqueue_bh */
    struct work_struct rx_wq;

    /* tasklet_bh */
    struct tasklet_struct rx_tasklet;

    struct sk_buff *skb_free[NUM_TX_DESC];
    unsigned int free_idx;

    struct net_device_stats stat;   /* The new statistics table. */
    spinlock_t page_lock;   /* spin_lock for cr access critial section */
    spinlock_t irq_lock;    /* spin_lock for isr critial section */
    struct PDMA_txdesc *tx_ring0;
    struct PDMA_rxdesc *rx_ring[MAX_RX_RING_NUM];
    dma_addr_t phy_rx_ring[MAX_RX_RING_NUM];
    void *netrx_skb_data[MAX_RX_RING_NUM][NUM_RX_DESC];
    struct sk_buff *netrx0_skbuf[NUM_RX_DESC];

    /* napi */
    struct napi_struct napi;
    struct napi_struct napi_rx;
    struct napi_struct napi_tx;
    struct net_device dummy_dev;

    /* clock control */
    struct clk  *clks[E_NOE_CLK_MAX];

    /* GE1 support */
    struct net_device *netdev;
    /* GE2 support */
    struct net_device *pseudo_dev;
    unsigned int is_pseudo;

    struct mii_if_info mii_info;
#if NETDEV_LRO_SUPPORTED
    struct lro_counters lro_counters;
    struct net_lro_mgr lro_mgr;
    struct net_lro_desc lro_arr[8];
#endif /* NETDEV_LRO_SUPPORTED */
    struct vlan_group *vlgrp;

    /* virtual base addr from device tree */
    void __iomem *ethdma_sysctl_base;

    unsigned int irq0;
    unsigned int irq1;
    unsigned int irq2;

    unsigned int features;
    unsigned int architecture;

    /* IP address */
    char lan_ip4_addr[IP4_ADDR_LEN];

    /* Function pointers */
    int (*ei_start_xmit)(struct sk_buff *skb, struct net_device *netdev, int gmac_no);
    int (*ei_xmit_housekeeping)(struct net_device *netdev, int budget);
    int (*ei_eth_recv)(struct net_device *dev, struct napi_struct *napi, int budget);
    int (*ei_fill_tx_desc)(struct net_device *dev, unsigned long *tx_cpu_owner_idx, struct sk_buff *skb, int gmac_no);

    struct task_struct *kreset_task;
    unsigned int fe_reset_times;

    EN_NOE_SEL_PIN_MUX pin_mux;
    unsigned char log_level;
    struct noe_stats stats;
    unsigned char irq_num;
    unsigned char sys_inited;
};

//-------------------------------------------------------------------------------------------------
//  Functions
//-------------------------------------------------------------------------------------------------
void MDrv_NOE_Set_Lro_Ip(char *lan_ip_addr);
static inline void *MDrv_NOE_SkbData_Alloc(size_t size, gfp_t flags)
{
#ifdef CONFIG_ETH_SLAB_ALLOC_SKB
    return kmalloc(size, flags);
#else
    return netdev_alloc_frag(size);
#endif
}

static inline void MDrv_NOE_SkbData_Free(void *addr)
{
#ifdef CONFIG_ETH_SLAB_ALLOC_SKB
    kfree(addr);
#else
    skb_free_frag(addr);
#endif
}

static inline struct sk_buff *MDrv_NOE_Skb_Build(void *data, unsigned int frag_size)
{
#ifdef CONFIG_ETH_SLAB_ALLOC_SKB
    return build_skb(data, 0);
#else
    return build_skb(data, frag_size);
#endif
}


#endif /* _MDRV_NOE_H_ */
