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
/// @file   devEMAC.c
/// @brief  EMAC Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/init.h>
#include <generated/autoconf.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/pci.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#if defined(CONFIG_MIPS)
#include <asm/mips-boards/prom.h>
#include "mhal_chiptop_reg.h"
#elif defined(CONFIG_ARM)
#include <prom.h>
#include <asm/mach/map.h>
#elif defined(CONFIG_ARM64)
#include <asm/arm-boards/prom.h>
#include <asm/mach/map.h>
#endif

#include "mdrv_types.h"
#include "mst_platform.h"
//#include "mdrv_system.h"
#include "chip_int.h"
#include "mhal_emac_v3.h"
#include "mdrv_emac_v3.h"
#include "mdrv_system_st.h"

#ifdef CHIP_FLUSH_READ
#include "chip_setup.h"
#endif
#ifdef CONFIG_EMAC_SUPPLY_RNG
#include <linux/input.h>
#include <random.h>
#include "mhal_rng_reg.h"
#endif
#include "mdrv_pm.h"

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
static struct net_device *emac_dev;
/* slt-test */
static bool is_slt_test_running = false;
static bool is_slt_test_passed = true;
static const int slt_packet_len = 1280;
static bool is_wol_woc_wake = false;
//-------------------------------------------------------------------------------------------------
//  EMAC Function
//-------------------------------------------------------------------------------------------------
static int  MDev_EMAC_tx (struct sk_buff *skb, struct net_device *dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static void MDev_EMAC_timer_callback(struct timer_list *t);
#else
static void MDev_EMAC_timer_callback( unsigned long value );
#endif
static int  MDev_EMAC_SwReset(struct net_device *dev);
static void MDev_EMAC_Send_PausePkt(struct net_device* dev);

#ifdef RX_ZERO_COPY
static void MDev_EMAC_RX_DESC_Init_zero_copy(struct net_device *dev);
static void MDev_EMAC_RX_DESC_close_zero_copy(struct net_device *dev);
#else
static void MDev_EMAC_RX_DESC_Init_memcpy(struct net_device *dev);
static void MDev_EMAC_RX_DESC_Reset_memcpy(struct net_device *dev);
#endif

#ifdef RX_NAPI
static void MDEV_EMAC_ENABLE_RX_REG(void);
static void MDEV_EMAC_DISABLE_RX_REG(void);
static int  MDev_EMAC_RX_napi_poll(struct napi_struct *napi, int budget);
#endif

#ifndef NEW_TX_QUEUE
#ifdef EMAC_XIU16_SW_PROTECT
static int MDev_EMAC_CheckTSR(struct net_device *dev);
#else
static int MDev_EMAC_CheckTSR(void);
#endif /* EMAC_XIU16_SW_PROTECT */
#endif

#ifdef TX_NAPI
static void MDEV_EMAC_ENABLE_TX_REG(void);
static void MDEV_EMAC_DISABLE_TX_REG(void);
static int  MDev_EMAC_TX_napi_poll(struct napi_struct *napi, int budget);
#endif

#ifdef TX_DESC_MODE
static void MDev_EMAC_TX_Desc_Reset(struct net_device *dev);
static void MDev_EMAC_TX_Desc_Mode_Set(struct net_device *dev);
static void MDev_EMAC_TX_Desc_Close(struct net_device *dev);
#endif

#ifdef TX_SOFTWARE_QUEUE
static void _MDev_EMAC_tx_reset_TX_SW_QUEUE(struct net_device* netdev);
#endif

static int MDev_EMAC_open(struct net_device *dev);
static int MDev_EMAC_close(struct net_device *dev);

void MDrv_EMAC_DumpMem(phys_addr_t addr, u32 len);
//-------------------------------------------------------------------------------------------------
// PHY MANAGEMENT
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Access the PHY to determine the current Link speed and Mode, and update the
// MAC accordingly.
// If no link or auto-negotiation is busy, then no changes are made.
// Returns:  0 : OK
//              -1 : No link
//              -2 : AutoNegotiation still in progress
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_update_linkspeed (struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 bmsr, bmcr, lpa;
#ifdef CONFIG_EMAC_EPHY_LPA_FORCE_SPEED
    u32 adv, neg;
#endif /* CONFIG_EMAC_EPHY_LPA_FORCE_SPEED */
    u32 speed, duplex;
#ifdef EMAC_XIU16_SW_PROTECT
    u8 BCE_speed_old, BCE_duplex_old;
    BCE_speed_old = LocPtr->ThisBCE.speed;
    BCE_duplex_old = LocPtr->ThisBCE.duplex;
#endif /* EMAC_XIU16_SW_PROTECT */

#ifdef CONFIG_EMAC_PHY_RESTART_AN
    u32 hcd_link_st_ok, an_100t_link_st = 0;
    static unsigned int phy_restart_cnt = 0;
#endif /* CONFIG_EMAC_PHY_RESTART_AN */

    // Link status is latched, so read twice to get current value //
    MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
    MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
    if (!(bmsr & BMSR_LSTATUS)){
    #ifdef CONFIG_MSTAR_EEE
        MHal_EMAC_Disable_EEE();

        if (LocPtr->PreLinkStatus == 1)
        {
            MHal_EMAC_Reset_EEE();
        }
    #endif

        LocPtr->PreLinkStatus = 0;

        return -1;          //no link //
    }

    MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMCR, &bmcr);

    if (bmcr & BMCR_ANENABLE)
    {               //AutoNegotiation is enabled //
        if (!(bmsr & BMSR_ANEGCOMPLETE))
        {
            EMAC_DBG("==> AutoNegotiation still in progress\n");
            return -2;
        }

#ifdef CONFIG_EMAC_EPHY_LPA_FORCE_SPEED
        /* Get Link partner and advertisement from the PHY not from the MAC */
        MHal_EMAC_read_phy(LocPtr->phyaddr, MII_ADVERTISE, &adv);
        MHal_EMAC_read_phy(LocPtr->phyaddr, MII_LPA, &lpa);

        /* For Link Parterner adopts force mode and EPHY used,
         * EPHY LPA reveals all zero value.
         * EPHY would be forced to Full-Duplex mode.
         */
        if (!lpa)
        {
            /* 100Mbps Full-Duplex */
            if (bmcr & BMCR_SPEED100)
                lpa |= LPA_100FULL;
            else /* 10Mbps Full-Duplex */
                lpa |= LPA_10FULL;
        }

        neg = adv & lpa;

        if (neg & LPA_100FULL)
        {
            speed = SPEED_100;
            duplex = DUPLEX_FULL;
        }
        else if (neg & LPA_100HALF)
        {
            speed = SPEED_100;
            duplex = DUPLEX_HALF;
        }
        else if (neg & LPA_10FULL)
        {
            speed = SPEED_10;
            duplex = DUPLEX_FULL;
        }
        else if (neg & LPA_10HALF)
        {
            speed = SPEED_10;
            duplex = DUPLEX_HALF;
        }
        else
        {
            speed = SPEED_10;
            duplex = DUPLEX_HALF;
            EMAC_DBG("%s: No speed and mode found (LPA=0x%x, ADV=0x%x)\n", __FUNCTION__, lpa, adv);
        }
#else
        MHal_EMAC_read_phy (LocPtr->phyaddr, MII_LPA, &lpa);
        if (lpa & LPA_100FULL)
        {
            speed = SPEED_100;
            duplex = DUPLEX_FULL;
        }
        else if (lpa & LPA_100HALF)
        {
            speed = SPEED_100;
            duplex = DUPLEX_HALF;
        }
        else if (lpa & LPA_10FULL)
        {
            speed = SPEED_10;
            duplex = DUPLEX_FULL;
        }
        else
        {
            speed = SPEED_10;
            duplex = DUPLEX_HALF;
        }

#endif /* CONFIG_EMAC_EPHY_LPA_FORCE_SPEED */
    }
    else
    {
        speed = (bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10;
        duplex = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
    }

#ifdef EMAC_XIU16_SW_PROTECT
    LocPtr->ThisBCE.speed = (speed == SPEED_100) ? EMAC_SPEED_100 : 0;
    LocPtr->ThisBCE.duplex = (duplex == DUPLEX_FULL) ? 2 : 1;

    if ((BCE_speed_old != LocPtr->ThisBCE.speed) || (BCE_duplex_old != LocPtr->ThisBCE.duplex))
    {
        MHal_EMAC_update_speed_duplex(speed,duplex);

        if (speed == SPEED_100)
            LocPtr->ThisUVE.CFG_val_golden |= EMAC_SPD;
        else
            LocPtr->ThisUVE.CFG_val_golden &= ((~EMAC_SPD) & 0x3FFFUL);

        if (duplex == DUPLEX_FULL)
            LocPtr->ThisUVE.CFG_val_golden |= EMAC_FD;
        else
            LocPtr->ThisUVE.CFG_val_golden &= ((~EMAC_FD) & 0x3FFFUL);
    }
    else if (LocPtr->ThisUVE.link_sp_updated == 0)
    {
        MHal_EMAC_update_speed_duplex(speed,duplex);
        LocPtr->ThisUVE.link_sp_updated = 1;
    }
#else
    // Update the MAC //
    MHal_EMAC_update_speed_duplex(speed,duplex);
#endif /* EMAC_XIU16_SW_PROTECT */

#ifdef CONFIG_MSTAR_EEE
    /*TX idle, enable EEE*/
    if((MDev_EMAC_IS_TX_IDLE()) && (speed == SPEED_100) && (duplex == DUPLEX_FULL))
    {
        if (LocPtr->PreLinkStatus == 0)
        {
            MHal_EMAC_Enable_EEE(300);
        }
        else
        {
            MHal_EMAC_Enable_EEE(0);
        }
    }
#endif

    LocPtr->PreLinkStatus = 1;

#ifdef CONFIG_EMAC_PHY_RESTART_AN
    if (speed == SPEED_100) {
        MHal_EMAC_read_phy(LocPtr->phyaddr, 0x21, &hcd_link_st_ok);
        MHal_EMAC_read_phy(LocPtr->phyaddr, 0x22, &an_100t_link_st);
        if (((hcd_link_st_ok & 0x100) && !(an_100t_link_st & 0x300)) || (!(hcd_link_st_ok & 0x100) && ((an_100t_link_st & 0x300) == 0x200))) {
            phy_restart_cnt++;
            if (phy_restart_cnt > 10) {
                EMAC_DBG("MDev_EMAC_update_linkspeed: restart AN process\n");
                MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, 0x1200UL);
                phy_restart_cnt = 0;
            }
        } else {
            phy_restart_cnt = 0;
        }
    }
#endif /* CONFIG_EMAC_PHY_RESTART_AN */

    return 0;
}

static int MDev_EMAC_get_info(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#ifdef CONFIG_EMAC_EPHY_LPA_FORCE_SPEED
    u32 bmsr, bmcr;
    u32 uRegStatus = 0;
    u32 adv, lpa, neg;
#else
    u32 bmsr, bmcr, LocPtrA;
    u32 uRegStatus =0;
#endif /* CONFIG_EMAC_EPHY_LPA_FORCE_SPEED */

    // Link status is latched, so read twice to get current value //
    MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
    MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
    if (!(bmsr & BMSR_LSTATUS)){
        uRegStatus &= ~ETHERNET_TEST_RESET_STATE;
        uRegStatus |= ETHERNET_TEST_NO_LINK; //no link //
    }
    MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMCR, &bmcr);

    if (bmcr & BMCR_ANENABLE)
    {
        //AutoNegotiation is enabled //
        if (!(bmsr & BMSR_ANEGCOMPLETE))
        {
            uRegStatus &= ~ETHERNET_TEST_RESET_STATE;
            uRegStatus |= ETHERNET_TEST_AUTO_NEGOTIATION; //AutoNegotiation //
        }
        else
        {
            uRegStatus &= ~ETHERNET_TEST_RESET_STATE;
            uRegStatus |= ETHERNET_TEST_LINK_SUCCESS; //link success //
        }

#ifdef CONFIG_EMAC_EPHY_LPA_FORCE_SPEED
        /* Get Link partner and advertisement from the PHY not from the MAC */
        MHal_EMAC_read_phy(LocPtr->phyaddr, MII_ADVERTISE, &adv);
        MHal_EMAC_read_phy(LocPtr->phyaddr, MII_LPA, &lpa);

        /* For Link Parterner adopts force mode and EPHY used,
         * EPHY LPA reveals all zero value.
         * EPHY would be forced to Full-Duplex mode.
         */
        if (!lpa)
        {
            /* 100Mbps Full-Duplex */
            if (bmcr & BMCR_SPEED100)
                lpa |= LPA_100FULL;
            else /* 10Mbps Full-Duplex */
                lpa |= LPA_10FULL;
        }

        neg = adv & lpa;

        if (neg & LPA_100FULL)
        {
            uRegStatus |= ETHERNET_TEST_SPEED_100M;
            uRegStatus |= ETHERNET_TEST_DUPLEX_FULL;
        }
        else if (neg & LPA_100HALF)
        {
            uRegStatus |= ETHERNET_TEST_SPEED_100M;
            uRegStatus &= ~ETHERNET_TEST_DUPLEX_FULL;
        }
        else if (neg & LPA_10FULL)
        {
            uRegStatus &= ~ETHERNET_TEST_SPEED_100M;
            uRegStatus |= ETHERNET_TEST_DUPLEX_FULL;
        }
        else if (neg & LPA_10HALF)
        {
            uRegStatus &= ~ETHERNET_TEST_SPEED_100M;
            uRegStatus &= ~ETHERNET_TEST_DUPLEX_FULL;
        }
        else
        {
            uRegStatus &= ~ETHERNET_TEST_SPEED_100M;
            uRegStatus &= ~ETHERNET_TEST_DUPLEX_FULL;
        }
#else
        MHal_EMAC_read_phy (LocPtr->phyaddr, MII_LPA, &LocPtrA);
        if ((LocPtrA & LPA_100FULL) || (LocPtrA & LPA_100HALF))
        {
            uRegStatus |= ETHERNET_TEST_SPEED_100M; //SPEED_100//
        }
        else
        {
            uRegStatus &= ~ETHERNET_TEST_SPEED_100M; //SPEED_10//
        }

        if ((LocPtrA & LPA_100FULL) || (LocPtrA & LPA_10FULL))
        {
            uRegStatus |= ETHERNET_TEST_DUPLEX_FULL; //DUPLEX_FULL//
        }
        else
        {
            uRegStatus &= ~ETHERNET_TEST_DUPLEX_FULL; //DUPLEX_HALF//
        }
#endif /* CONFIG_EMAC_EPHY_LPA_FORCE_SPEED */
    }
    else
    {
        if(bmcr & BMCR_SPEED100)
        {
            uRegStatus |= ETHERNET_TEST_SPEED_100M; //SPEED_100//
        }
        else
        {
            uRegStatus &= ~ETHERNET_TEST_SPEED_100M; //SPEED_10//
        }

        if(bmcr & BMCR_FULLDPLX)
        {
            uRegStatus |= ETHERNET_TEST_DUPLEX_FULL; //DUPLEX_FULL//
        }
        else
        {
            uRegStatus &= ~ETHERNET_TEST_DUPLEX_FULL; //DUPLEX_HALF//
        }
    }

    return uRegStatus;
}

//-------------------------------------------------------------------------------------------------
//Program the hardware MAC address from dev->dev_addr.
//-------------------------------------------------------------------------------------------------
void MDev_EMAC_update_mac_address (struct net_device *dev)
{
    u32 value;
    value = (dev->dev_addr[3] << 24) | (dev->dev_addr[2] << 16) | (dev->dev_addr[1] << 8) |(dev->dev_addr[0]);
    MHal_EMAC_Write_SA1L(value);
    value = (dev->dev_addr[5] << 8) | (dev->dev_addr[4]);
    MHal_EMAC_Write_SA1H(value);

    /* set mac addr to phy for wol */
    MHal_EMAC_set_wol_mac_addr(dev->dev_addr[5], dev->dev_addr[4], dev->dev_addr[3], dev->dev_addr[2], dev->dev_addr[1], dev->dev_addr[0]);
}

//-------------------------------------------------------------------------------------------------
// ADDRESS MANAGEMENT
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Set the ethernet MAC address in dev->dev_addr
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_get_mac_address (struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    char addr[6];
    u32 HiAddr, LoAddr;

    // Check if bootloader set address in Specific-Address 1 //
    HiAddr = MHal_EMAC_get_SA1H_addr();
    LoAddr = MHal_EMAC_get_SA1L_addr();

    addr[0] = (LoAddr & 0xffUL);
    addr[1] = (LoAddr & 0xff00UL) >> 8;
    addr[2] = (LoAddr & 0xff0000UL) >> 16;
    addr[3] = (LoAddr & 0xff000000UL) >> 24;
    addr[4] = (HiAddr & 0xffUL);
    addr[5] = (HiAddr & 0xff00UL) >> 8;

    if (is_valid_ether_addr (addr))
    {
        EMAC_DBG("SA1_Valid!!!\n");
        memcpy (LocPtr->ThisBCE.sa1, &addr, 6);
        memcpy (dev->dev_addr, &addr, 6);
        return;
    }
    // Check if bootloader set address in Specific-Address 2 //
    HiAddr = MHal_EMAC_get_SA2H_addr();
    LoAddr = MHal_EMAC_get_SA2L_addr();
    addr[0] = (LoAddr & 0xffUL);
    addr[1] = (LoAddr & 0xff00UL) >> 8;
    addr[2] = (LoAddr & 0xff0000UL) >> 16;
    addr[3] = (LoAddr & 0xff000000UL) >> 24;
    addr[4] = (HiAddr & 0xffUL);
    addr[5] = (HiAddr & 0xff00UL) >> 8;

    if (is_valid_ether_addr (addr))
    {
        EMAC_DBG("SA2_Valid!!!\n");
        memcpy (LocPtr->ThisBCE.sa1, &addr, 6);
        memcpy (dev->dev_addr, &addr, 6);
        return;
    }
    else
    {
        EMAC_DBG("ALL Not Valid!!!\n");
        LocPtr->ThisBCE.sa1[0] = MY_MAC[0];
        LocPtr->ThisBCE.sa1[1] = MY_MAC[1];
        LocPtr->ThisBCE.sa1[2] = MY_MAC[2];
        LocPtr->ThisBCE.sa1[3] = MY_MAC[3];
        LocPtr->ThisBCE.sa1[4] = MY_MAC[4];
        LocPtr->ThisBCE.sa1[5] = MY_MAC[5];
        memcpy (dev->dev_addr, LocPtr->ThisBCE.sa1, 6);
    }
}

#ifdef URANUS_ETHER_ADDR_CONFIGURABLE
//-------------------------------------------------------------------------------------------------
// Store the new hardware address in dev->dev_addr, and update the MAC.
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_set_mac_address (struct net_device *dev, void *addr)
{
    struct sockaddr *address = addr;
    if (!is_valid_ether_addr (address->sa_data))
        return -EADDRNOTAVAIL;

    memcpy (dev->dev_addr, address->sa_data, dev->addr_len);
    MDev_EMAC_update_mac_address (dev);
    return 0;
}
#endif

//-------------------------------------------------------------------------------------------------
// Mstar Multicast hash rule
//-------------------------------------------------------------------------------------------------
//Hash_index[5] = da[5] ^ da[11] ^ da[17] ^ da[23] ^ da[29] ^ da[35] ^ da[41] ^ da[47]
//Hash_index[4] = da[4] ^ da[10] ^ da[16] ^ da[22] ^ da[28] ^ da[34] ^ da[40] ^ da[46]
//Hash_index[3] = da[3] ^ da[09] ^ da[15] ^ da[21] ^ da[27] ^ da[33] ^ da[39] ^ da[45]
//Hash_index[2] = da[2] ^ da[08] ^ da[14] ^ da[20] ^ da[26] ^ da[32] ^ da[38] ^ da[44]
//Hash_index[1] = da[1] ^ da[07] ^ da[13] ^ da[19] ^ da[25] ^ da[31] ^ da[37] ^ da[43]
//Hash_index[0] = da[0] ^ da[06] ^ da[12] ^ da[18] ^ da[24] ^ da[30] ^ da[36] ^ da[42]
//-------------------------------------------------------------------------------------------------

static void MDev_EMAC_sethashtable(unsigned char *addr)
{
    u32 mc_filter[2];
    u32 uHashIdxBit;
    u32 uHashValue;
    u32 i;
    u32 tmpcrc;
    u32 uSubIdx;
    u64 macaddr;
    u64 mac[6];

    uHashValue = 0;
    macaddr = 0;

    // Restore mac //
    for(i = 0; i < 6;  i++)
    {
        mac[i] =(u64)addr[i];
    }

    // Truncate mac to u64 container //
    macaddr |=  mac[0] | (mac[1] << 8) | (mac[2] << 16);
    macaddr |=  (mac[3] << 24) | (mac[4] << 32) | (mac[5] << 40);

    // Caculate the hash value //
    for(uHashIdxBit = 0; uHashIdxBit < 6;  uHashIdxBit++)
    {
        tmpcrc = (macaddr & (0x1UL << uHashIdxBit)) >> uHashIdxBit;
        for(i = 1; i < 8;  i++)
        {
            uSubIdx = uHashIdxBit + (i * 6);
            tmpcrc = tmpcrc ^ ((macaddr >> uSubIdx) & 0x1);
        }
        uHashValue |= (tmpcrc << uHashIdxBit);
    }

    mc_filter[0] = MHal_EMAC_ReadReg32( REG_ETH_HSL);
    mc_filter[1] = MHal_EMAC_ReadReg32( REG_ETH_HSH);

    // Set the corrsponding bit according to the hash value //
    if(uHashValue < 32)
    {
        mc_filter[0] |= (0x1UL <<  uHashValue);
        MHal_EMAC_WritReg32( REG_ETH_HSL, mc_filter[0] );
    }
    else
    {
        mc_filter[1] |= (0x1UL <<  (uHashValue - 32));
        MHal_EMAC_WritReg32( REG_ETH_HSH, mc_filter[1] );
    }
}

//-------------------------------------------------------------------------------------------------
//Enable/Disable promiscuous and multicast modes.
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_set_rx_mode (struct net_device *dev)
{
#ifdef EMAC_XIU16_SW_PROTECT
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#endif /* EMAC_XIU16_SW_PROTECT */
    u32 uRegVal;
    struct netdev_hw_addr *ha;

    uRegVal  = MHal_EMAC_Read_CFG();

    if (dev->flags & IFF_PROMISC)
    {
        // Enable promiscuous mode //
        uRegVal |= EMAC_CAF;
    }
    else if (dev->flags & (~IFF_PROMISC))
    {
        // Disable promiscuous mode //
        uRegVal &= ~EMAC_CAF;
    }
    MHal_EMAC_Write_CFG(uRegVal);

    if (dev->flags & IFF_ALLMULTI)
    {
        // Enable all multicast mode //
        MHal_EMAC_update_HSH(-1,-1);
        uRegVal |= EMAC_MTI;
    }
    else if (dev->flags & IFF_MULTICAST)
    {
        // Enable specific multicasts//
        MHal_EMAC_update_HSH(0,0);
        netdev_for_each_mc_addr(ha, dev)
        {
            MDev_EMAC_sethashtable(ha->addr);
        }
        uRegVal |= EMAC_MTI;
    }
    else if (dev->flags & ~(IFF_ALLMULTI | IFF_MULTICAST))
    {
        // Disable all multicast mode//
        MHal_EMAC_update_HSH(0,0);
        uRegVal &= ~EMAC_MTI;
    }

    MHal_EMAC_Write_CFG(uRegVal);

#ifdef EMAC_XIU16_SW_PROTECT
    if ((uRegVal & EMAC_BR & 0x3FFFUL) == 0)
    {
        printk(KERN_DEBUG "%s: change CFG golden from 0x%x to 0x%x\n", __FUNCTION__, LocPtr->ThisUVE.CFG_val_golden, uRegVal);
        LocPtr->ThisUVE.CFG_val_golden = uRegVal;
    }
#endif /* EMAC_XIU16_SW_PROTECT */
}
//-------------------------------------------------------------------------------------------------
// IOCTL
//-------------------------------------------------------------------------------------------------
/* slt-test */
static void mstar_emac_slt_test_tx(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    dma_addr_t skb_addr;
    int i, j, err_cnt_rx, err_cnt_tx, err_cnt_col;
    char *pskb;
    u32 reg_val;
    u32 config3_value = 0;

    printk("slt-test-eth: start mstar_emac_slt_test_tx\n");

    /* stop kernel tx packet */
    netif_stop_queue(dev);

    /* enable copy all frame mode */
    reg_val = MHal_EMAC_Read_CFG();
    reg_val |= 0x00000010UL;
    MHal_EMAC_Write_CFG(reg_val);

    /* set delay interrupt to 1 packet 1 time */
#ifdef RX_DELAY_INTERRUPT
    config3_value = (0x01010000UL | EMAC_INT_DELAY_MODE_EN);
#endif
#ifdef RX_CHECKSUM
    config3_value = config3_value | RX_CHECKSUM_ENABLE;
#endif
#ifdef RX_DESC_MODE
    config3_value = config3_value | SOFTWARE_DESCRIPTOR_ENABLE;
#endif
#ifdef HW_TX_CHECKSUM
    dev->features |= NETIF_F_IP_CSUM;
    config3_value = config3_value | TX_CHECKSUM_ENABLE;
#endif
    MHal_EMAC_Write_Network_config_register3(config3_value);

    /* prepare test packet */
    skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
    pskb = skb_addr;
    for (j = 0; j < 5; j++) {
        for (i = 0; i < 256; i++) {
            *pskb = i;
            pskb++;
        }
    }

    /* EMAC error count, read clear */
    err_cnt_rx = (MHal_EMAC_Read_ALE() + MHal_EMAC_Read_ELR() + MHal_EMAC_Read_SEQE() + MHal_EMAC_Read_ROVR() + MHal_EMAC_Read_SE() + MHal_EMAC_Read_RJB());
    err_cnt_tx = (MHal_EMAC_Read_TUE() + MHal_EMAC_Read_CSE() + MHal_EMAC_Read_SQEE());
    err_cnt_col = (MHal_EMAC_Read_LCOL() + MHal_EMAC_Read_ECOL() + MHal_EMAC_Read_SCOL() + MHal_EMAC_Read_MCOL());

    is_slt_test_passed = true;
    //MDrv_EMAC_DumpMem(skb_addr, slt_packet_len);
    MHal_EMAC_Write_TAR(skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE);
    MHal_EMAC_Write_TCR(slt_packet_len);

    printk("slt-test-eth: end mstar_emac_slt_test_tx\n");
}

#ifdef EMAC_NEW_WOC
static u8 woc_filter_index = 0;
static u8 woc_sram_pattern[20][128] = {0};

u8 mstar_emac_woc_index_get(void)
{
    return woc_filter_index;
}

void mstar_emac_woc_index_set(u8 index)
{
    woc_filter_index = index;
}

int mstar_emac_woc_sram_read(struct net_device *dev)
{
    u32 i, j, temp;
    u8 val;

    /* set to sram mode */
    val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET);
    val &= 0xFC;
    MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET, val);

    for (i = 0; i < WOC_PATTERN_BYTES_MAX; i++) {
        /* set pattern byte offset */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_CTRL_OFFSET);
        val &= 0x80;
        val |= (i & 0x7F);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_CTRL_OFFSET, val);

        /* latch address to offset */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_H_SRAM0_ADDR_LATCH);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET, val);

        for (j = 0; j < WOC_FILTER_NUM_MAX; j++) {
            woc_sram_pattern[j][i] = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, (REG_WOC_SRAM0_DOUT_OFFSET + j));
        }
    }

    return 0;
}

int mstar_emac_woc_sram_write(struct net_device *dev)
{
    u32 i, j, delay;
    u8 val;

    /* set to sram mode */
    val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET);
    val &= 0xFC;
    val |= 0x02;
    MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET, val);

    for (i = 0; i < WOC_PATTERN_BYTES_MAX; i++) {
        delay = 0;

        /* set pattern byte offset */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_CTRL_OFFSET);
        val &= 0x80;
        val |= (i & 0x7F);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_CTRL_OFFSET, val);

        /* latch address to offset */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_H_SRAM0_ADDR_LATCH);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET, val);

        for(j = 0; j < WOC_FILTER_NUM_MAX; j++) {
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, (REG_WOC_CHK_LENGTH_0_OFFSET + j), woc_sram_pattern[j][i]);
        }

        if ((i == ETH_PROTOCOL_NUM_OFFSET) || (i == ETH_PROTOCOL_DEST_PORT_H_OFFSET) || (i == ETH_PROTOCOL_DEST_PORT_L_OFFSET)) {
            /* enable all pattern byte check */
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_0_7_OFFSET, 0xff);
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_8_15_OFFSET, 0xff);
            val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET);
            val |= 0x0f;
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET, val);
        } else {
            /* disable all pattern byte check */
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_0_7_OFFSET, 0x0);
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_8_15_OFFSET, 0x0);
            val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET);
            val &= 0xf0;
            MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET, val);
        }

        /* write to sram */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_H_SRAM0_WE);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET, val);
        do {
            mdelay(1);
            val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET);
            val &= (0x1 << BIT_WOC_CTRL_L_WOL_WRDY);

            delay++;
            if (delay == 20) {
                printk("mstar_emac_woc_write_sram write sram0 failed 0x%x\n", val);
                goto write_sram_fail;
            }
        } while (val == 0);

        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_H_WOL_WRDY_CR);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET, val);
    }

    return 0;

write_sram_fail:
    return -1;
}

int mstar_emac_woc_protocol_port_set(struct net_device *dev, char protocol_type, int *port_array, int count)
{
    int i;

    if (woc_filter_index >= WOC_FILTER_NUM_MAX) {
        printk("already set 20 packets!\n");
        return -1;
    }

    if ((protocol_type != UDP_PROTOCOL) && (protocol_type != TCP_PROTOCOL)) {
        printk("Protocol type is wrong(UDP = 17, TCP = 6).!\n");
        return -1;
    }

    mstar_emac_woc_sram_read(dev);

    for (i = 0; i < count; i++) {
        /* byte23 is udp/tcp protocol number. */
        woc_sram_pattern[woc_filter_index][ETH_PROTOCOL_NUM_OFFSET] = protocol_type;

        /* byte36 and byte37 is udp/tcp destination port. */
        woc_sram_pattern[woc_filter_index][ETH_PROTOCOL_DEST_PORT_H_OFFSET] = (char)(port_array[i] >> 8);
        woc_sram_pattern[woc_filter_index][ETH_PROTOCOL_DEST_PORT_L_OFFSET] = (char)(port_array[i] & 0xff);
        woc_filter_index++;
    }

    mstar_emac_woc_sram_write(dev);

    return woc_filter_index - count;
}

int mstar_emac_woc_detect_len_set(struct net_device *dev, int len)
{
    u8 i, val;
    u32 bit_mask = 0;

    /* set to check length mode */
    val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET);
    val |= 0x03;
    MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET, val);

    for (i = 0; i < mstar_emac_woc_index_get(); i++) {
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, (REG_WOC_CHK_LENGTH_0_OFFSET + i), len);
        bit_mask |= (0x1 << i);
    }

    /* enable pattern length check */
    MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_0_7_OFFSET, (bit_mask & 0xffUL));
    MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_8_15_OFFSET, (bit_mask & 0xff00UL) >> 8);
    val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET);
    val &= 0xf0;
    val |= ((bit_mask & 0xf0000UL) >> 16);
    MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET, val);

    return 0;
}

void mstar_emac_woc_config(struct net_device *dev, bool enable)
{
    u8 val = 0;

    if (enable) {
        mstar_emac_woc_detect_len_set(dev, WOC_PATTERN_CHECK_LEN_MAX);

        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_L_WOL_EN);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET, val);

        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_L_STORAGE_EN);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET, val);

        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY2, REG_WOL_MODE_OFFSET);
        val |= (0x1 << BIT_WOL_MODE_WOL_EN);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY2, REG_WOL_MODE_OFFSET, val);
    } else {
        /* not disable wol here, this is for woc */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET);
        val &= ~(0x1 << BIT_WOC_CTRL_L_WOL_EN);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET, val);

        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET);
        val &= ~(0x1 << BIT_WOC_CTRL_L_STORAGE_EN);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_L_OFFSET, val);

        /* set to check length mode */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET);
        val |= 0x01;
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_SRAM_STATE_OFFSET, val);

        /* disable all pattern byte check */
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_0_7_OFFSET, 0x0);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_8_15_OFFSET, 0x0);
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET);
        val &= 0xf0;
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_PAT_EN_16_19_OFFSET, val);
    }
}

void mstar_emac_woc_auto_load_trigger(struct net_device *dev)
{
    u8 val_det_l, val_det_m, val_det_h, val;

    val_det_l = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_DET_LA_0_7_OFFSET);
    val_det_m = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_DET_LA_8_15_OFFSET);
    val_det_h = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_DET_LA_16_19_OFFSET);
    val_det_h &= 0x0f;

    if ((val_det_l != 0) || (val_det_m != 0) || (val_det_h != 0)) {
        /* pattern match hit */
        /* trigger autoload */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_H_WOL_STORAGE_AUTOLOAD);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET, val);
        /* clear interrupt */
        val = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET);
        val |= (0x1 << BIT_WOC_CTRL_H_WOL_INT_CLEAR);
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY3, REG_WOC_CTRL_H_OFFSET, val);
        printk("mstar_emac_woc_auto_load_trigger: trigger auto load\n");
    }
}
#endif /* EMAC_NEW_WOC */

//-------------------------------------------------------------------------------------------------
// Enable/Disable MDIO
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_mdio_read (struct net_device *dev, int phy_id, int location)
{
    u32 value;
    MHal_EMAC_read_phy (phy_id, location, &value);
    return value;
}

static void MDev_EMAC_mdio_write (struct net_device *dev, int phy_id, int location, int value)
{
    MHal_EMAC_write_phy (phy_id, location, value);
}

//-------------------------------------------------------------------------------------------------
//ethtool support.
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_ethtool_ioctl (struct net_device *dev, void *useraddr)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 ethcmd;
    int res = 0;

    if (copy_from_user (&ethcmd, useraddr, sizeof (ethcmd)))
        return -EFAULT;

    spin_lock_irq (&LocPtr->irq_lock);

    switch (ethcmd)
    {
        case ETHTOOL_GSET:
        {
            struct ethtool_cmd ecmd = { ETHTOOL_GSET };
            res = mii_ethtool_gset (&LocPtr->mii, &ecmd);
            if (copy_to_user (useraddr, &ecmd, sizeof (ecmd)))
                res = -EFAULT;
            break;
        }
        case ETHTOOL_SSET:
        {
            struct ethtool_cmd ecmd;
            if (copy_from_user (&ecmd, useraddr, sizeof (ecmd)))
                res = -EFAULT;
            else
                res = mii_ethtool_sset (&LocPtr->mii, &ecmd);
            break;
        }
        case ETHTOOL_NWAY_RST:
        {
            res = mii_nway_restart (&LocPtr->mii);
            break;
        }
        case ETHTOOL_GLINK:
        {
            struct ethtool_value edata = { ETHTOOL_GLINK };
            edata.data = mii_link_ok (&LocPtr->mii);
            if (copy_to_user (useraddr, &edata, sizeof (edata)))
                res = -EFAULT;
            break;
        }
        default:
            res = -EOPNOTSUPP;
    }
    spin_unlock_irq (&LocPtr->irq_lock);
    return res;
}

//-------------------------------------------------------------------------------------------------
// User-space ioctl interface.
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_ioctl (struct net_device *dev, struct ifreq *rq, int cmd)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    struct mii_ioctl_data *data = if_mii(rq);
    u32 tmpVal = 0;

    if (!netif_running(dev))
    {
        rq->ifr_metric = ETHERNET_TEST_INIT_FAIL;
    }

    switch (cmd)
    {
        case SIOCGMIIPHY:
            data->phy_id = (LocPtr->phyaddr & 0x1FUL);
            return 0;

        case SIOCDEVPRIVATE:
            rq->ifr_metric = (MDev_EMAC_get_info(emac_dev)|LocPtr->initstate);
            return 0;

        case SIOCDEVON:
            MHal_EMAC_Power_On_Clk();
#ifdef CONFIG_EMAC_TR_PN_SWAP
            if (LocPtr->is_tr_pn_swap == true)
                MHal_EMAC_Swap_TR_PN();
#endif /* CONFIG_EMAC_TR_PN_SWAP */
            return 0;

        case SIOCDEVOFF:
            MHal_EMAC_Power_Off_Clk();
            return 0;

        case SIOCGMIIREG:
            // check PHY's register 1.
            if((data->reg_num & 0x1fUL) == 0x1UL)
            {
                // PHY's register 1 value is set by timer callback function.
                spin_lock_irq(&LocPtr->irq_lock);
                data->val_out = LocPtr->phy_status_register;
                spin_unlock_irq(&LocPtr->irq_lock);
            }
            else
            {
                MHal_EMAC_read_phy((LocPtr->phyaddr & 0x1FUL), (data->reg_num & 0x1fUL), &tmpVal);
                data->val_out = tmpVal;
            }
            return 0;

        case SIOCSMIIREG:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            MHal_EMAC_write_phy((LocPtr->phyaddr & 0x1FUL), (data->reg_num & 0x1fUL), data->val_in);
            return 0;

        case SIOCETHTOOL:
            return MDev_EMAC_ethtool_ioctl (dev, (void *) rq->ifr_data);

        /* slt-test */
        case SIOC_SLT_TEST_AN_RUN:
        {
            struct ioctl_slt_para_cmd slt_cmd;
            u32 timeout = 0;
            u32 i;

            if (copy_from_user(&slt_cmd, rq->ifr_data, sizeof(slt_cmd))) {
                printk(KERN_ERR "slt-test-eth: copy from user fail\n");
                return -EFAULT;
            }

            printk("slt-test-eth: test adv all with auto-nego %d\n", slt_cmd.test_count);

            for (i = 0; i < slt_cmd.test_count; i++) {
                is_slt_test_running = true;
                is_slt_test_passed = false;
                timeout = 0;
                mstar_emac_slt_test_tx(dev);
                while (is_slt_test_running == true) {
                    mdelay(1);
                    if (timeout > 2000) {
                        printk(KERN_ERR "slt-test-eth: adv all timeout 2 secs, no rx: %d\n", i);
                        break;
                    }
                    timeout++;
                }
                if ((is_slt_test_running == false) && (is_slt_test_passed == true)) {
                    printk("slt-test-eth: test adv all with auto-nego passed: %d\n", i);
                } else {
                    printk(KERN_ERR "slt-test-eth: test adv all with auto-nego failed: %d\n", i);
                    is_slt_test_running = false;
                    is_slt_test_passed = false;
                    slt_cmd.failed_count = i;
                    if (copy_to_user (rq->ifr_data, &slt_cmd, sizeof(slt_cmd)))
                        printk(KERN_ERR "slt-test-eth: copy from user fail\n");
                    return -EFAULT;
                }
            }

            return 0;
        }

        case SIOC_SLT_TEST_FS_RUN:
        {
            struct ioctl_slt_para_cmd slt_cmd;
            u32 timeout = 0;
            u32 adv_orig = 0;
            u32 i;

            if (copy_from_user(&slt_cmd, rq->ifr_data, sizeof(slt_cmd))) {
                printk(KERN_ERR "slt-test-eth: copy from user fail\n");
                return -EFAULT;
            }

            MHal_EMAC_read_phy(LocPtr->phyaddr, MII_ADVERTISE, &adv_orig);

            /* test adv speed 100 start */
            printk("slt-test-eth: test adv speed 100: %d\n", slt_cmd.test_count);

            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_RESET);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, (ADVERTISE_CSMA | ADVERTISE_100FULL | ADVERTISE_100HALF));
            mdelay(10);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_CTRL1000, 0x0);
            mdelay(10);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_SPEED100 | BMCR_FULLDPLX));

            LocPtr->ThisBCE.connected = 0;

            LocPtr->Link_timer.expires = jiffies + (EMAC_CHECK_LINK_TIME / 1000000);

            if (!(timer_pending(&LocPtr->Link_timer)))
                add_timer(&LocPtr->Link_timer);

            timeout = 0;

            while (LocPtr->ThisBCE.connected == 0) {
                mdelay(1);
                if (timeout > 5000) {
                    printk(KERN_ERR "slt-test-eth: adv speed 100 timeout 5 secs, no connection\n");
                    is_slt_test_running = false;
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, adv_orig);
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));
                    return -EFAULT;
                }
                timeout++;
            }

            for (i = 0; i < slt_cmd.test_count; i++) {
                is_slt_test_running = true;
                is_slt_test_passed = false;
                timeout = 0;
                mstar_emac_slt_test_tx(dev);
                while (is_slt_test_running == true) {
                    mdelay(1);
                    if (timeout > 2000) {
                        printk(KERN_ERR "slt-test-eth: adv speed 100 timeout 2 secs, no rx: %d\n", i);
                        break;
                    }
                    timeout++;
                }
                if ((is_slt_test_running == false) && (is_slt_test_passed == true)) {
                    printk("slt-test-eth: test adv speed 100 passed: %d\n", i);
                } else {
                    printk(KERN_ERR "slt-test-eth: test adv speed 100 failed: %d\n", i);
                    is_slt_test_running = false;
                    is_slt_test_passed = false;
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, adv_orig);
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));
                    slt_cmd.failed_count = i;
                    if (copy_to_user (rq->ifr_data, &slt_cmd, sizeof(slt_cmd)))
                        printk(KERN_ERR "slt-test-eth: copy from user fail\n");
                    return -EFAULT;
                }
            }

            /* test adv speed 10 start */
            printk("slt-test-eth: test adv speed 10: %d\n", slt_cmd.test_count);

            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_RESET);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, (ADVERTISE_CSMA | ADVERTISE_10FULL | ADVERTISE_10HALF));
            mdelay(10);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_CTRL1000, 0x0);
            mdelay(10);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_SPEED10 | BMCR_FULLDPLX));

            LocPtr->ThisBCE.connected = 0;

            LocPtr->Link_timer.expires = jiffies + (EMAC_CHECK_LINK_TIME / 1000000);

            if (!(timer_pending(&LocPtr->Link_timer)))
                add_timer(&LocPtr->Link_timer);

            timeout = 0;

            while (LocPtr->ThisBCE.connected == 0) {
                mdelay(1);
                if (timeout > 5000) {
                    printk(KERN_ERR "slt-test-eth: adv speed 100 timeout 5 secs, no connection\n");
                    is_slt_test_running = false;
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, adv_orig);
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));
                    return -EFAULT;
                }
                timeout++;
            }

            for (i = 0; i < slt_cmd.test_count; i++) {
                is_slt_test_running = true;
                is_slt_test_passed = false;
                timeout = 0;
                mstar_emac_slt_test_tx(dev);
                while (is_slt_test_running == true) {
                    mdelay(1);
                    if (timeout > 2000) {
                        printk(KERN_ERR "slt-test-eth: adv speed 10 timeout 2 secs, no rx: %d\n", i);
                        break;
                    }
                    timeout++;
                }
                if ((is_slt_test_running == false) && (is_slt_test_passed == true)) {
                    printk("slt-test-eth: test adv speed 10 passed: %d\n", i);
                } else {
                    printk(KERN_ERR "slt-test-eth: test adv speed 10 failed: %d\n", i);
                    is_slt_test_running = false;
                    is_slt_test_passed = false;
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, adv_orig);
                    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));
                    slt_cmd.failed_count = i;
                    if (copy_to_user (rq->ifr_data, &slt_cmd, sizeof(slt_cmd)))
                        printk(KERN_ERR "slt-test-eth: copy from user fail\n");
                    return -EFAULT;
                }
            }

            return 0;
        }

        case SIOC_SLT_TEST_CLR:
        {
            printk("slt-test-eth: clear test environment\n");
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_RESET);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, (ADVERTISE_CSMA | ADVERTISE_ALL));
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_CTRL1000, 0x0);
            MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));
            is_slt_test_running = false;
            return 0;
        }

#ifdef EMAC_NEW_WOC
        case SIOC_SET_WOP_CMD:
        {
#ifdef CONFIG_COMPAT
            struct ioctl_woc_para_cmd32 __user *req_ifr_data32;
            struct ioctl_woc_para_cmd32 woc_cmd32;
#endif /* CONFIG_COMPAT */
            struct ioctl_woc_para_cmd __user *req_ifr_data;
            struct ioctl_woc_para_cmd woc_cmd;
            void __user *woc_port_array;
            u32 *port_array;
            int retval;

#ifdef CONFIG_COMPAT
            if (is_compat_task()) {
                req_ifr_data32 = compat_ptr(ptr_to_compat(rq->ifr_data));

                if (copy_from_user(&woc_cmd32, req_ifr_data32,
                    sizeof(struct ioctl_woc_para_cmd32))) {
                    printk(KERN_ERR "req_ifr_data32: 0x%p fail!\n", req_ifr_data32);
                    return -EFAULT;
                }

                port_array = kmalloc((sizeof(u32) * woc_cmd32.port_count), GFP_KERNEL);
                if (!port_array) {
                    printk(KERN_ERR "port_array kmalloc fail!\n");
                    return -EFAULT;
                }

                woc_port_array = compat_ptr(woc_cmd32.port_array);
                if (!woc_port_array) {
                    printk(KERN_ERR "null woc_port_array!\n");
                    kfree(port_array);
                    return -EFAULT;
                }

                retval = copy_from_user(port_array, woc_port_array, (sizeof(u32) * woc_cmd32.port_count));
                if (retval) {
                    printk(KERN_ERR "wop_port_array: 0x%p fail!"
                           "count32 = %d, ret = %d \n",
                           woc_port_array,
                           woc_cmd32.port_count, retval);
                    kfree(port_array);

                    return -EFAULT;
                }

                retval = mstar_emac_woc_protocol_port_set(dev,
                            woc_cmd32.protocol_type, port_array,
                            woc_cmd32.port_count);
            } else
#endif /* CONFIG_COMPAT */
            {
                req_ifr_data = rq->ifr_data;

                if (copy_from_user(&woc_cmd, req_ifr_data,
                    sizeof(struct ioctl_woc_para_cmd))) {
                    printk(KERN_ERR "req_ifr_data: 0x%p fail! \n",
                                req_ifr_data);
                    return -EFAULT;
                }

                port_array = kmalloc((sizeof(u32) * woc_cmd.port_count),
                                     GFP_KERNEL);
                if (!port_array) {
                    printk(KERN_ERR "port_array kmalloc fail! \n");
                    return -EFAULT;
                }

                woc_port_array = woc_cmd.port_array;
                if (!woc_port_array) {
                    printk(KERN_ERR "null woc_port_array!\n");
                    kfree(port_array);
                    return -EFAULT;
                }

                retval = copy_from_user(port_array, woc_port_array,
                         (sizeof(u32) * woc_cmd.port_count));
                if (retval) {
                    printk(KERN_ERR "woc_port_array: 0x%p fail! "
                                "count = %d, ret = %d \n",
                                woc_port_array, woc_cmd.port_count,
                                retval);
                    kfree(port_array);

                    return -EFAULT;
                }

                retval = mstar_emac_woc_protocol_port_set(dev,
                         woc_cmd.protocol_type, port_array,
                         woc_cmd.port_count);
            }

            kfree(port_array);

            return retval;
        }
        case SIOC_CLR_WOP_CMD:
            mstar_emac_woc_index_set(0);
            return 0;
#endif /* EMAC_NEW_WOC */

        default:
            return -EOPNOTSUPP;
    }
}
//-------------------------------------------------------------------------------------------------
// MAC
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//Initialize and start the Receiver and Transmit subsystems
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_start (struct net_device *dev)
{
    struct EMAC_private *LocPtr;
    u32 uRegVal;

#ifdef RX_ZERO_COPY
    MDev_EMAC_RX_DESC_Init_zero_copy(dev);
#else
    MDev_EMAC_RX_DESC_Reset_memcpy(dev);
#endif

#ifdef TX_DESC_MODE
    MDev_EMAC_TX_Desc_Reset(dev);
    MDev_EMAC_TX_Desc_Mode_Set(dev);
#else
#ifdef TX_SOFTWARE_QUEUE
    _MDev_EMAC_tx_reset_TX_SW_QUEUE(dev);
#else
    LocPtr = (struct EMAC_private*) netdev_priv(dev);
    LocPtr->tx_index = 0;
#endif
#endif



    // Enable Receive and Transmit //
    uRegVal = MHal_EMAC_Read_CTL();
    uRegVal |= (EMAC_RE | EMAC_TE);
    MHal_EMAC_Write_CTL(uRegVal);

#ifdef EMAC_XIU16_SW_PROTECT
    if ((uRegVal & EMAC_LBL) == 0) {
        printk(KERN_DEBUG "%s: change CTL golden from 0x%x to 0x%x\n", __FUNCTION__, LocPtr->ThisUVE.CTL_val_golden, uRegVal);
        LocPtr->ThisUVE.CTL_val_golden = uRegVal;
    } else {
        printk(KERN_WARNING, "%s: EMAC_LBL is pulled high, 0x%x\n", __FUNCTION__, uRegVal);
    }
#endif /* EMAC_XIU16_SW_PROTECT */
}

//-------------------------------------------------------------------------------------------------
// Open the ethernet interface
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_open (struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 uRegVal;
    int ret;

#ifdef RX_NAPI
    EMAC_DBG("MDev_EMAC_open: stat=0x%lx, stat_sch=0x%x\n", LocPtr->napi_rx.state, NAPI_STATE_SCHED);
    if (!test_bit(NAPI_STATE_SCHED, &LocPtr->napi_rx.state)) {
        napi_disable(&LocPtr->napi_rx);
        EMAC_DBG("napi_disable RRXX!\n");
    }

    napi_enable(&LocPtr->napi_rx);
    EMAC_DBG("napi_enable RX!!!\n");
    LocPtr->full_budge_count = 0;
    LocPtr->max_polling = 0;
    LocPtr->polling_count = 0;
    LocPtr->ROVR_count = 0;
#endif

#ifdef TX_NAPI
    if(LocPtr->napi_tx.state == NAPI_STATE_SCHED)
    {
        napi_disable(&LocPtr->napi_tx);
        EMAC_DBG("napi_disable TX!!!\n");
    }

    napi_enable(&LocPtr->napi_tx);
    EMAC_DBG("napi_enable TX!!!\n");
#endif

#ifdef EMAC_XIU16_SW_PROTECT
    EMAC_DBG("XIU16_SW_PROTECT!!!\n");
#endif /* EMAC_XIU16_SW_PROTECT */

    spin_lock_irq (&LocPtr->irq_lock);
    ret = MDev_EMAC_update_linkspeed(dev);
    spin_unlock_irq (&LocPtr->irq_lock);

    if (!is_valid_ether_addr (dev->dev_addr))
       return -EADDRNOTAVAIL;

    uRegVal = MHal_EMAC_Read_CTL();
    uRegVal |= EMAC_CSR;
    MHal_EMAC_Write_CTL(uRegVal);

#ifdef EMAC_XIU16_SW_PROTECT
    if ((uRegVal & EMAC_LBL) == 0) {
        printk(KERN_DEBUG "%s: change CTL golden from 0x%x to 0x%x\n", __FUNCTION__, LocPtr->ThisUVE.CTL_val_golden, uRegVal);
        LocPtr->ThisUVE.CTL_val_golden = uRegVal;
    } else {
        printk(KERN_WARNING, "%s: EMAC_LBL is pulled high, 0x%x\n", __FUNCTION__, uRegVal);
    }
#endif /* EMAC_XIU16_SW_PROTECT */

    // Enable PHY interrupt //
    MHal_EMAC_enable_phyirq ();

    // Enable MAC interrupts //
    MHal_EMAC_Write_IDR(EMAC_INT_ALL);
#ifndef RX_DELAY_INTERRUPT
    uRegVal = EMAC_INT_RCOM | EMAC_INT_ENABLE;
    MHal_EMAC_Write_IER(uRegVal);
#else
    MHal_EMAC_Write_IER(EMAC_INT_ENABLE);
#endif

    LocPtr->ep_flag |= EP_FLAG_OPEND;

    MDev_EMAC_set_rx_mode(dev);
    MDev_EMAC_start (dev);
    netif_start_queue (dev);

#ifdef TX_ZERO_COPY
    LocPtr->tx_desc_start_free = 1;
#endif

    if (!(timer_pending(&LocPtr->Link_timer)))
        add_timer(&LocPtr->Link_timer);

#ifdef TX_ZERO_COPY
    if (!(timer_pending(&LocPtr->TX_free_timer)))
        add_timer(&LocPtr->TX_free_timer);
#endif


    /* check if network linked */
    if (-1 == ret)
    {
        netif_carrier_off(dev);
        LocPtr->ThisBCE.connected = 0;
    }
    else if(0 == ret)
    {
        netif_carrier_on(dev);
        LocPtr->ThisBCE.connected = 1;
    }

    return 0;
}
//-------------------------------------------------------------------------------------------------
// Close the interface
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_close (struct net_device *dev)
{
    u32 uRegVal;
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

#ifdef RX_NAPI
    napi_disable(&LocPtr->napi_rx);
#endif

#ifdef TX_NAPI
    napi_disable(&LocPtr->napi_tx);
#endif

#ifdef TX_ZERO_COPY
    LocPtr->tx_desc_start_free = 0;
#endif

    //Disable Receiver and Transmitter //
    uRegVal = MHal_EMAC_Read_CTL();
    uRegVal &= ~(EMAC_TE | EMAC_RE);
    MHal_EMAC_Write_CTL(uRegVal);

#ifdef EMAC_XIU16_SW_PROTECT
    if ((uRegVal & EMAC_LBL) == 0) {
        printk(KERN_DEBUG "%s: change CTL golden from 0x%x to 0x%x\n", __FUNCTION__, LocPtr->ThisUVE.CTL_val_golden, uRegVal);
        LocPtr->ThisUVE.CTL_val_golden = uRegVal;
    } else {
        printk(KERN_WARNING, "%s: EMAC_LBL is pulled high, 0x%x\n", __FUNCTION__, uRegVal);
    }
#endif /* EMAC_XIU16_SW_PROTECT */

    // Disable PHY interrupt //
    MHal_EMAC_disable_phyirq ();
#ifndef RX_DELAY_INTERRUPT
    //Disable MAC interrupts //
    uRegVal = EMAC_INT_RCOM | EMAC_INT_ENABLE;
    MHal_EMAC_Write_IDR(uRegVal);
#else
    MHal_EMAC_Write_IDR(EMAC_INT_ENABLE);
#endif
#ifdef RX_ZERO_COPY
    MDev_EMAC_RX_DESC_close_zero_copy(dev);
#endif
    netif_stop_queue (dev);
    del_timer_sync(&LocPtr->Link_timer);
#ifdef TX_ZERO_COPY
    del_timer_sync(&LocPtr->TX_free_timer);
#endif
#ifdef TX_ZERO_COPY
    LocPtr->ThisBCE.connected = 0;
#endif
    LocPtr->ep_flag &= (~EP_FLAG_OPEND);

#ifdef TX_DESC_MODE
    MDev_EMAC_TX_Desc_Reset(dev);
    #ifdef TX_ZERO_COPY
    MDev_EMAC_TX_Desc_Close(dev);
    #endif
#endif

#ifdef TX_SOFTWARE_QUEUE
    _MDev_EMAC_tx_reset_TX_SW_QUEUE(dev);
#endif

    return 0;
}

//-------------------------------------------------------------------------------------------------
// Update the current statistics from the internal statistics registers.
//-------------------------------------------------------------------------------------------------
static struct net_device_stats * MDev_EMAC_stats (struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    int ale, lenerr, seqe, lcol, ecol;

    spin_lock_irq (&LocPtr->irq_lock);

    if (netif_running (dev))
    {
        LocPtr->stats.rx_packets += MHal_EMAC_Read_OK();            /* Good frames received */
        ale = MHal_EMAC_Read_ALE();
        LocPtr->stats.rx_frame_errors += ale;                       /* Alignment errors */
        lenerr = MHal_EMAC_Read_ELR();
        LocPtr->stats.rx_length_errors += lenerr;                   /* Excessive Length or Undersize Frame error */
        seqe = MHal_EMAC_Read_SEQE();
        LocPtr->stats.rx_crc_errors += seqe;                        /* CRC error */
        LocPtr->stats.rx_fifo_errors += MHal_EMAC_Read_ROVR();
        LocPtr->stats.rx_errors += ale + lenerr + seqe + MHal_EMAC_Read_SE() + MHal_EMAC_Read_RJB();
        LocPtr->stats.tx_packets += MHal_EMAC_Read_FRA();           /* Frames successfully transmitted */
        LocPtr->stats.tx_fifo_errors += MHal_EMAC_Read_TUE();       /* Transmit FIFO underruns */
        LocPtr->stats.tx_carrier_errors += MHal_EMAC_Read_CSE();    /* Carrier Sense errors */
        LocPtr->stats.tx_heartbeat_errors += MHal_EMAC_Read_SQEE(); /* Heartbeat error */
        lcol = MHal_EMAC_Read_LCOL();
        ecol = MHal_EMAC_Read_ECOL();
        LocPtr->stats.tx_window_errors += lcol;                     /* Late collisions */
        LocPtr->stats.tx_aborted_errors += ecol;                    /* 16 collisions */
        LocPtr->stats.collisions += MHal_EMAC_Read_SCOL() + MHal_EMAC_Read_MCOL() + lcol + ecol;
    }

    spin_unlock_irq (&LocPtr->irq_lock);

    return &LocPtr->stats;
}

static int MDev_EMAC_TxReset(void)
{
    u32 val = MHal_EMAC_Read_CTL() & 0x000001FFUL;

    MHal_EMAC_Write_CTL((val & ~EMAC_TE));
    MHal_EMAC_Write_TCR(0);
    MHal_EMAC_Write_CTL((MHal_EMAC_Read_CTL() | EMAC_TE));
    return 0;
}

#ifdef EMAC_XIU16_SW_PROTECT
static int MDev_EMAC_TxReset_CTL(void)
{
    u32 val = MHal_EMAC_Read_CTL() & 0x000001FFUL;
    MHal_EMAC_Write_CTL((val & ~(EMAC_TE)));
    val = MHal_EMAC_Read_CTL() & 0x000001FFUL;
    MHal_EMAC_Write_CTL((val | EMAC_TE));
    return 0;
}
#endif /* EMAC_XIU16_SW_PROTECT */

static u8 pause_pkt[] =
{
    //DA - multicast
    0x01, 0x80, 0xC2, 0x00, 0x00, 0x01,
    //SA
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //Len-Type
    0x88, 0x08,
    //Ctrl code
    0x00, 0x01,
    //Ctrl para 8192
    0x20, 0x00
};

void MDrv_EMAC_DumpMem(phys_addr_t addr, u32 len)
{
    u8 *ptr = (u8 *)addr;
    u32 i;

    printk(KERN_ERR "===== Dump %lx, len %d(%02x) =====\n",
           (long unsigned int)ptr, len, len);
    printk(KERN_ERR "              00 01 02 03 04 05 06 07  08 09 0a 0b 0c 0d 0e 0f\n");
    for (i = 0; i < len; i++) {
        printk(KERN_ERR "%lx(%02x): %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n",
               (long unsigned int)ptr, i,
               *ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3), *(ptr + 4), *(ptr + 5), *(ptr + 6), *(ptr + 7),
               *(ptr + 8), *(ptr + 9), *(ptr + 10), *(ptr + 11), *(ptr + 12), *(ptr + 13), *(ptr + 14), *(ptr + 15));
        ptr += 16;
        i += 15;
    }
    printk(KERN_ERR "\n");
}
#if 0
//Background send
static int MDev_EMAC_BGsend(struct net_device* dev, phys_addr_t addr, int len )
{
    dma_addr_t skb_addr;
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

#ifdef NEW_TX_QUEUE
    if (MHal_EMAC_New_TX_QUEUE_OVRN_Get() == 1)
        return NETDEV_TX_BUSY;
#else
    if (NETDEV_TX_OK != MDev_EMAC_CheckTSR())
        return NETDEV_TX_BUSY;
#endif

    skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
    LocPtr->tx_index ++;
    LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;

    memcpy((void*)skb_addr,(void *)addr, len);

    LocPtr->stats.tx_bytes += len;

#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
    if((unsigned int)skb_addr < 0xC0000000UL)
    {
        Chip_Flush_Memory_Range((unsigned int)skb_addr&0x0FFFFFFFUL, len);
    }
    else
    {
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    }
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    #else
        //#ERROR
    #endif
#endif

    //Set address of the data in the Transmit Address register //
    MHal_EMAC_Write_TAR(skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE);

    // Set length of the packet in the Transmit Control register //
    MHal_EMAC_Write_TCR(len);

    return NETDEV_TX_OK;
}

static void MDev_EMAC_Send_PausePkt(struct net_device* dev)
{
    u32 val = MHal_EMAC_Read_CTL() & 0x000001FFUL;

    //Disable Rx
    MHal_EMAC_Write_CTL((val & ~EMAC_RE));
    memcpy(&pause_pkt[6], dev->dev_addr, 6);
    MDev_EMAC_BGsend(dev, (phys_addr_t)pause_pkt, sizeof(pause_pkt));
    //Enable Rx
    MHal_EMAC_Write_CTL((MHal_EMAC_Read_CTL() | EMAC_RE));
}
#endif
//-------------------------------------------------------------------------------------------------
//Patch for losing small-size packet when running SMARTBIT
//-------------------------------------------------------------------------------------------------
#if 0
#ifdef CONFIG_MP_ETHERNET_MSTAR_ICMP_ENHANCE
static void MDev_EMAC_Period_Retry(struct sk_buff *skb, struct net_device* dev)
{
    u32 xval;
    u32 uRegVal;

    xval = MHal_EMAC_ReadReg32(REG_ETH_CFG);

    if((skb->len <= PACKET_THRESHOLD) && !(xval & EMAC_SPD) && !(xval & EMAC_FD))
    {
        txcount++;
    }
    else
    {
        txcount = 0;
    }

    if(txcount > TXCOUNT_THRESHOLD)
    {
        uRegVal  = MHal_EMAC_Read_CFG();
        uRegVal  |= 0x00001000UL;
        MHal_EMAC_Write_CFG(uRegVal);
    }
    else
    {
        uRegVal = MHal_EMAC_Read_CFG();
        uRegVal &= ~(0x00001000UL);
        MHal_EMAC_Write_CFG(uRegVal);
    }
}
#endif

int MDev_EMAC_enqueue(struct skb_node **queue_head, struct skb_node **queue_tail, struct sk_buff *skb)
{
    struct skb_node *pskb_node = (struct skb_node *)kmalloc(sizeof(struct skb_node), GFP_ATOMIC);

    if(!pskb_node)
    {
        EMAC_DBG("*pskb_node kmalloc fail\n");
        BUG_ON(1);
        return 1;
    }

    /*Reset the content of pskb_node*/
    memset(pskb_node, 0x0, sizeof(struct skb_node));

    pskb_node->skb = skb;
    pskb_node->next = *queue_head;
    pskb_node->prev = NULL;

    if(!(*queue_tail))
    {
        (*queue_tail) = pskb_node;
    }

    (*queue_tail)->queuesize = (*queue_tail)->queuesize + 1;

    if(!(*queue_head))
    {
        (*queue_head) = pskb_node;
    }
    else
    {
        (*queue_head)->prev = pskb_node;
    }

    (*queue_head) = pskb_node;

    return 0;
}

struct sk_buff* MDev_EMAC_dequeue(struct skb_node **queue_head, struct skb_node **queue_tail)
{
    struct sk_buff *skb;

    if(!(*queue_tail))
    {
        return NULL;
    }
    else
    {
        skb = (*queue_tail)->skb;

        /* If the size of queue is equal to 1*/
        if(*queue_head == *queue_tail)
        {
            kfree(*queue_tail);
            *queue_tail = NULL;
            *queue_head = NULL;
        }
        else
        {
            (*queue_tail)->prev->queuesize = (*queue_tail)->queuesize - 1;
            *queue_tail = (*queue_tail)->prev;
            kfree((*queue_tail)->next);
            (*queue_tail)->next = NULL;
        }
    }

    return skb;
}

void MDev_EMAC_bottom_tx_task(struct work_struct *work)
{
    struct sk_buff *skb;
    unsigned long flags;
    dma_addr_t skb_addr;
    struct EMAC_private *LocPtr = container_of(work, struct EMAC_private, tx_task);
    struct net_device *dev = LocPtr->dev;


    spin_lock_irqsave(LocPtr->lock, flags);

#ifdef EMAC_XIU16_SW_PROTECT
    while(NETDEV_TX_OK == MDev_EMAC_CheckTSR(dev))
#else
    while(NETDEV_TX_OK == MDev_EMAC_CheckTSR())
#endif /* EMAC_XIU16_SW_PROTECT */
    {
        skb = MDev_EMAC_dequeue(&(LocPtr->tx_queue_head), &(LocPtr->tx_queue_tail));

        if(skb == NULL)
        {
            break;
        }
        else
        {

            skb_addr = get_tx_addr();
            memcpy((void*)skb_addr, skb->data, skb->len);

        #ifdef CONFIG_MP_ETHERNET_MSTAR_ICMP_ENHANCE
            MDev_EMAC_Period_Retry(skb, dev);
        #endif
            LocPtr->stats.tx_bytes += skb->len;

        #ifdef CHIP_FLUSH_READ
            #if defined(CONFIG_MIPS)
            if((unsigned int)skb_addr < 0xC0000000UL)
            {
                Chip_Flush_Memory_Range((unsigned int)skb_addr&0x0FFFFFFFUL, skb->len);
            }
            else
            {
                Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
            }
            #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
                Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
            #else
                //#ERROR
            #endif
        #endif

            //MDrv_EMAC_DumpMem(tx_fifo->skb_physaddr,skb->len);

            //Set address of the data in the Transmit Address register //
            MHal_EMAC_Write_TAR(skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE);

            // Set length of the packet in the Transmit Control register //
            MHal_EMAC_Write_TCR(skb->len);

            dev->trans_start = jiffies;
            dev_kfree_skb_irq(skb);
        }
    }
    spin_unlock_irqrestore(LocPtr->lock, flags);
}
#endif

#ifdef TX_ZERO_COPY
#ifdef TX_DESC_MODE
//-------------------------------------------------------------------------------------------------
//Transmit packet.
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_TX_Desc_Reset(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

    LocPtr->tx_desc_write_index = 0;
    LocPtr->tx_desc_read_index = 0;
    LocPtr->tx_desc_queued_number = 0;
    LocPtr->tx_desc_count = 0;
    LocPtr->tx_desc_full_count = 0;
}

static void MDev_EMAC_TX_Desc_Mode_Set(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

    MHal_EMAC_Write_LOW_PRI_TX_DESC_BASE(LocPtr->TX_LP_DESC_BASE - MIU0_BUS_BASE);
    MHal_EMAC_Write_LOW_PRI_TX_DESC_THRESHOLD(TX_LOW_PRI_DESC_NUMBER|EMAC_RW_TX_DESC_EN_W);
    return;
}

static void MDev_EMAC_TX_Desc_Close(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

    u32 tx_desc_index;

    for(tx_desc_index = 0; tx_desc_index < TX_LOW_PRI_DESC_NUMBER; tx_desc_index++)
    {
        if(LocPtr->tx_desc_sk_buff_list[tx_desc_index])
            dev_kfree_skb_any(LocPtr->tx_desc_sk_buff_list[tx_desc_index]);
        LocPtr->tx_desc_sk_buff_list[tx_desc_index] = NULL;
    }
    MHal_EMAC_Write_LOW_PRI_TX_DESC_THRESHOLD(0x0);

    return;
}

static u32 MDev_EMAC_TX_Free_sk_buff(struct net_device *dev, u32 work_done)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 free_sk_buffer_number, hw_queue_number, free_count, flags;

    if(!LocPtr->tx_desc_start_free)
        return 0;

    spin_lock_irqsave(&LocPtr->tx_lock, flags);

    free_count = 0;
    hw_queue_number = MHal_EMAC_Read_LOW_PRI_TX_DESC_QUEUED() & 0x03FFFUL;

    if(LocPtr->tx_desc_queued_number > 8 + hw_queue_number)
    {
        free_sk_buffer_number = LocPtr->tx_desc_queued_number - hw_queue_number - 8;
    }
    else
    {
        free_sk_buffer_number = 0;
    }

    while(free_sk_buffer_number > free_count)
    {
        //GMAC_DBG("tx_desc_sk_buff_list = 0x%zx, tx_desc_list = 0x%zx, tx_desc_read_index = 0x%zx\n", (size_t)LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_read_index], (size_t)LocPtr->tx_desc_list[LocPtr->tx_desc_read_index].addr, LocPtr->tx_desc_read_index);

        //dma_unmap_single(&dev->dev, LocPtr->tx_desc_list[LocPtr->tx_desc_read_index].addr + MIU0_BUS_BASE, LocPtr->tx_desc_list[LocPtr->tx_desc_read_index].low_tag & 0x03FFFUL, DMA_TO_DEVICE);
        dev_kfree_skb_any(LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_read_index]);

        LocPtr->stats.tx_bytes += (LocPtr->tx_desc_list[LocPtr->tx_desc_read_index].low_tag & 0x03FFFUL);
        LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_read_index]=NULL;
        LocPtr->tx_desc_read_index++;
        if(TX_LOW_PRI_DESC_NUMBER == LocPtr->tx_desc_read_index)
        {
            LocPtr->tx_desc_read_index = 0;
        }
//        EMAC_DBG("Free Next No.%d, MHal_EMAC_Read_LOW_PRI_TX_DESC_PTR() = 0x%zx\n", LocPtr->tx_desc_read_index, MHal_EMAC_Read_LOW_PRI_TX_DESC_PTR() & 0x03FFFUL);

        LocPtr->tx_desc_queued_number--;

        free_count++;
#ifdef TX_NAPI
        if(free_count + work_done > TX_NAPI_WEIGHT)
            break;
#endif
    }
    spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
    //MHal_GMAC_Write_IDR(GMAC_INT_TCOM);

    return free_count;
}


static int MDev_EMAC_tx (struct sk_buff *skb, struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    struct netdev_queue *txq = netdev_get_tx_queue(dev, 0);
#endif
    unsigned long flags;
    dma_addr_t skb_addr;

    //if((MHal_EMAC_Read_LOW_PRI_TX_DESC_QUEUED() & 0x03FFFUL) > 0x40)
    //{
        //return NETDEV_TX_BUSY;
    //}

    if (skb->len > EMAC_TX_MAX_LEN)
    {
        EMAC_DBG("Wrong Tx len:%u\n", skb->len);
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    if (MHal_EMAC_LOW_PRI_TX_DESC_MODE_OVRN_Get() == 1 )
    {
        EMAC_DBG("TX_DESC_MODE_OVRN\n");
        netif_stop_queue(dev);
        //MDev_EMAC_TX_Free_sk_buff(dev, 0);
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    MDev_EMAC_TX_Free_sk_buff(dev,0);

    spin_lock_irqsave(&LocPtr->tx_lock, flags);

    skb_addr =(u32)__virt_to_phys(skb->data);

#ifdef CHIP_FLUSH_READ
    Chip_Flush_Cache_Range((unsigned long)skb->data, skb->len);
#endif

    LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_write_index]= skb;
    LocPtr->tx_desc_list[LocPtr->tx_desc_write_index].addr = skb_addr - MIU0_BUS_BASE;
    if(LocPtr->tx_desc_write_index == TX_LOW_PRI_DESC_NUMBER - 1)
    {
        LocPtr->tx_desc_list[LocPtr->tx_desc_write_index].low_tag = (skb->len & 0x3FFFUL) | EMAC_TX_DESC_WRAP;
#ifdef CHIP_FLUSH_READ
        Chip_Flush_Cache_Range((unsigned long)(&(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index])), sizeof(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index]));
#endif

        LocPtr->tx_desc_write_index = 0;
    }
    else
    {
        LocPtr->tx_desc_list[LocPtr->tx_desc_write_index].low_tag = skb->len & 0x3FFFUL;
#ifdef CHIP_FLUSH_READ
        Chip_Flush_Cache_Range((unsigned long)(&(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index])), sizeof(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index]));
#endif
        LocPtr->tx_desc_write_index++;
    }

    if(LocPtr->tx_desc_count % 2 == 0)
    {
        //EMAC_DBG("0x%zx\n", LocPtr->tx_desc_trigger_flag);
        MHal_EMAC_Write_LOW_PRI_TX_DESC_TRANSMIT0(0x1);
    }
    else
    {
        //EMAC_DBG("0x%zx\n", LocPtr->tx_desc_trigger_flag);
        MHal_EMAC_Write_LOW_PRI_TX_DESC_TRANSMIT1(0x1);
    }

    LocPtr->tx_desc_count++;
    LocPtr->tx_desc_queued_number++;
    LocPtr->stats.tx_bytes += skb->len;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    txq->trans_start = jiffies;
#else
    dev->trans_start = jiffies;
#endif
/*
    if(LocPtr->tx_desc_write_index > TX_DESC_DELAY_FREE)
    {
        LocPtr->tx_desc_start_free = true;
    }

    if(LocPtr->tx_desc_write_index > TX_DESC_DELAY_FREE && LocPtr->tx_desc_start_free)
    {
        dev_kfree_skb_any(LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_write_index - TX_DESC_DELAY_FREE - 1]);
        LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_write_index - TX_DESC_DELAY_FREE - 1] = NULL;
    }
    else if(LocPtr->tx_desc_start_free)
    {
        dev_kfree_skb_any(LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_write_index + TX_LOW_PRI_DESC_NUMBER - TX_DESC_DELAY_FREE - 1]);
        LocPtr->tx_desc_sk_buff_list[LocPtr->tx_desc_write_index + TX_LOW_PRI_DESC_NUMBER - TX_DESC_DELAY_FREE - 1] = NULL;
    }
*/

    out_unlock:
    spin_unlock_irqrestore(&LocPtr->tx_lock, flags);

    MDev_EMAC_TX_Free_sk_buff(dev,0);

    return NETDEV_TX_OK;
}
#endif
#ifdef TX_SOFTWARE_QUEUE
//-------------------------------------------------------------------------------------------------
//Transmit packet.
//-------------------------------------------------------------------------------------------------
//  read skb from TX_SW_QUEUE to HW,
//  !!!! NO SPIN LOCK INSIDE !!!!
static int MDev_EMAC_GetTXFIFOIdle(void)
{

    u32 tsrval = 0;
    u8  avlfifo[8] = {0};
    u8  avlfifoidx;
    u8  avlfifoval = 0;

    tsrval = MHal_EMAC_Read_TSR();
    avlfifo[0] = ((tsrval & EMAC_IDLETSR) != 0)? 1 : 0;
    avlfifo[1] = ((tsrval & EMAC_BNQ)!= 0)? 1 : 0;
    avlfifo[2] = ((tsrval & EMAC_TBNQ) != 0)? 1 : 0;
    avlfifo[3] = ((tsrval & EMAC_FBNQ) != 0)? 1 : 0;
    avlfifo[4] = ((tsrval & EMAC_FIFO1IDLE) !=0)? 1 : 0;
    avlfifo[5] = ((tsrval & EMAC_FIFO2IDLE) != 0)? 1 : 0;
    avlfifo[6] = ((tsrval & EMAC_FIFO3IDLE) != 0)? 1 : 0;
    avlfifo[7] = ((tsrval & EMAC_FIFO4IDLE) != 0)? 1 : 0;

    avlfifoval = 0;
    for(avlfifoidx = 0; avlfifoidx < 8; avlfifoidx++)
    {
        avlfifoval += avlfifo[avlfifoidx];
    }

    if (avlfifoval > 4)
    {
        return avlfifoval-4;
    }
    return 0;
}

static void _MDev_EMAC_tx_read_TX_SW_QUEUE(int txIdleCount,struct net_device *dev,int intr)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
//  int txIdleCount=MDev_EMAC_GetTXFIFOIdle();

    while(txIdleCount>0){

        struct tx_ring *txq=&(LocPtr->tx_swq[LocPtr->tx_rdidx]);
        if(txq->used == TX_DESC_WROTE)
        {
            Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
            //Chip_Flush_Memory_Range((size_t)txq->skb->data,txq->skb->len);
            MHal_EMAC_Write_TAR(txq->skb_physaddr - MIU0_BUS_BASE);
            MHal_EMAC_Write_TCR(txq->skb->len);
            txq->used=TX_DESC_READ;
            LocPtr->tx_rdidx ++;
            if(TX_SW_QUEUE_SIZE==LocPtr->tx_rdidx)
            {
                LocPtr->tx_rdwrp++;
                LocPtr->tx_rdidx =0;
            }
        }else{
            break;
        }
        txIdleCount--;
    }

}

//  clear skb from TX_SW_QUEUE
//  !!!! NO SPIN LOCK INSIDE !!!!
static void _MDev_EMAC_tx_clear_TX_SW_QUEUE(int txIdleCount,struct net_device *dev,int intr)
{

    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    int clearcnt=0;
    int fifoCount=0;//LocPtr->tx_rdidx-LocPtr->tx_clidx;
    if(0==txIdleCount)return;

    fifoCount=LocPtr->tx_rdidx-LocPtr->tx_clidx;
    if(fifoCount<0)fifoCount=LocPtr->tx_rdidx+(TX_SW_QUEUE_SIZE-LocPtr->tx_clidx);

    /*
     * "fifoCount" is the count of the packets that has been sent to the EMAC HW.
     * "(TX_FIFO_SIZE-txIdleCount)" is the packet count that has not yet been sent out completely by EMAC HW
     */
    clearcnt = fifoCount-(TX_FIFO_SIZE-txIdleCount);
    if((clearcnt > TX_FIFO_SIZE) || (clearcnt < 0)){
        printk(KERN_ERR"fifoCount in _MDev_EMAC_tx_clear_TX_SW_QUEUE() ERROR!! fifoCount=%d intr=%d, %d, %d, %d, %d\n",fifoCount,intr,LocPtr->tx_rdidx,LocPtr->tx_clidx,txIdleCount,TX_FIFO_SIZE);
    }

    while(clearcnt>0)
    {
        struct tx_ring *txq=&(LocPtr->tx_swq[LocPtr->tx_clidx]);
        if(TX_DESC_READ==txq->used)
        {

            dma_unmap_single(&dev->dev, txq->skb_physaddr, txq->skb->len, DMA_TO_DEVICE);

            LocPtr->stats.tx_bytes += txq->skb->len;

            dev_kfree_skb_any(txq->skb);

            txq->used = TX_DESC_CLEARED;
            txq->skb=NULL;
            LocPtr->tx_clidx++;
            if(TX_SW_QUEUE_SIZE==LocPtr->tx_clidx)
            {
                LocPtr->tx_clwrp++;
                LocPtr->tx_clidx =0;
            }
        }else{
            break;
        }
        clearcnt--;
    }
}

static void _MDev_EMAC_tx_reset_TX_SW_QUEUE(struct net_device* netdev)
{
    struct EMAC_private *LocPtr;
    u32 i=0;
    LocPtr = (struct EMAC_private*) netdev_priv(netdev);
    for (i=0;i<TX_SW_QUEUE_SIZE;i++)
    {
        if(LocPtr->tx_swq[i].skb != NULL)
        {
            dma_unmap_single(&netdev->dev, LocPtr->tx_swq[i].skb_physaddr, LocPtr->tx_swq[i].skb->len, DMA_TO_DEVICE);
            dev_kfree_skb_any(LocPtr->tx_swq[i].skb);

        }
        LocPtr->tx_swq[i].skb = NULL;
        LocPtr->tx_swq[i].used = TX_DESC_CLEARED;
        LocPtr->tx_swq[i].skb_physaddr = 0;
    }
    LocPtr->tx_clidx = 0;
    LocPtr->tx_wridx = 0;
    LocPtr->tx_rdidx = 0;
    LocPtr->tx_clwrp = 0;
    LocPtr->tx_wrwrp = 0;
    LocPtr->tx_rdwrp = 0;
    LocPtr->tx_swq_full_cnt=0;
}

static int MDev_EMAC_tx (struct sk_buff *skb, struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    unsigned long flags;
    dma_addr_t skb_addr;

    spin_lock_irqsave(&LocPtr->tx_lock, flags);

    if (skb->len > EMAC_TX_MAX_LEN)
    {
        EMAC_DBG("Wrong Tx len:%u\n", skb->len);
        spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    {
        int txIdleCount=0;//MDev_EMAC_GetTXFIFOIdle();
        //FIFO full, loop until HW empty then try again
        //This is an abnormal condition as the upper network tx_queue should already been stopped by "netif_stop_queue(dev)" in code below
        if( LocPtr->tx_swq[LocPtr->tx_wridx].used > TX_DESC_CLEARED)
        {
            printk(KERN_ERR"ABNORMAL !! %d, %d, %d, %d\n",LocPtr->tx_wridx,LocPtr->tx_rdidx,LocPtr->tx_clidx, LocPtr->tx_swq[LocPtr->tx_wridx].used );
            BUG();
            /*
            txIdleCount=MDev_EMAC_GetTXFIFOIdle();
            while(0==txIdleCount)
            {
                txIdleCount=MDev_EMAC_GetTXFIFOIdle();
            }
            _MDev_EMAC_tx_clear_TX_SW_QUEUE(txIdleCount,dev,TX_SW_QUEUE_IN_GENERAL_TX);
            */
        }

        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);

        //map skbuffer for DMA
        skb_addr = dma_map_single(&dev->dev, skb->data, skb->len, DMA_TO_DEVICE);

        if (dma_mapping_error(&dev->dev, skb_addr))
        {
            dev_kfree_skb_any(skb);
            printk(KERN_ERR"ERROR!![%s]%d\n",__FUNCTION__,__LINE__);
            dev->stats.tx_dropped++;

            goto out_unlock;
        }

        LocPtr->tx_swq[LocPtr->tx_wridx].skb = skb;
        LocPtr->tx_swq[LocPtr->tx_wridx].skb_physaddr= skb_addr;
        LocPtr->tx_swq[LocPtr->tx_wridx].used = TX_DESC_WROTE;
        LocPtr->tx_wridx ++;
        if(TX_SW_QUEUE_SIZE==LocPtr->tx_wridx)
        {
            LocPtr->tx_wridx=0;
            LocPtr->tx_wrwrp++;
        }


        //if FIFO is full, netif_stop_queue
        if( LocPtr->tx_swq[LocPtr->tx_wridx].used > TX_DESC_CLEARED)
        {
            LocPtr->tx_swq_full_cnt++;
            netif_stop_queue(dev);
        }

        // clear & read to HW FIFO
        txIdleCount = MDev_EMAC_GetTXFIFOIdle();

        _MDev_EMAC_tx_clear_TX_SW_QUEUE(txIdleCount,dev,TX_SW_QUEUE_IN_GENERAL_TX);
        _MDev_EMAC_tx_read_TX_SW_QUEUE(txIdleCount,dev,TX_SW_QUEUE_IN_GENERAL_TX);
    }

out_unlock:
    spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
    return NETDEV_TX_OK;
}
#endif
#else
#ifdef NEW_TX_QUEUE
static int MDev_EMAC_tx (struct sk_buff *skb, struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    struct netdev_queue *txq = netdev_get_tx_queue(dev, 0);
#endif
    unsigned long flags;
    dma_addr_t skb_addr;

#ifdef EMAC_10T_RANDOM_WAVEFORM
    if(LocPtr->private_flag & EMAC_PRIVATE_FLAG_10T_RANDOM)
    {
        return NETDEV_TX_BUSY;
    }
#endif

    //spin_lock_irqsave(&LocPtr->tx_lock, flags);
    if (skb->len > EMAC_TX_MAX_LEN)
    {
        EMAC_DBG("Wrong Tx len:%u\n", skb->len);
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    if (MHal_EMAC_New_TX_QUEUE_OVRN_Get() == 1)
    {
        EMAC_DBG("New_TX_QUEUE_OVRN\n");
        //netif_stop_queue(dev);    /*Will fail in TX UDP Test*/
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    spin_lock_irqsave(&LocPtr->tx_lock, flags);
    skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
    LocPtr->tx_index ++;
    LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;
    spin_unlock_irqrestore(&LocPtr->tx_lock, flags);


    if (!skb_addr)
    {
        EMAC_DBG("Can not get memory from EMAC area\n");
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return -ENOMEM;
    }

    memcpy((void*)skb_addr, skb->data, skb->len);


    LocPtr->stats.tx_bytes += skb->len;

#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
    if((u32)skb_addr < 0xC0000000UL)
    {
        Chip_Flush_Memory_Range((unsigned int)skb_addr&0x0FFFFFFFUL, skb->len);
    }
    else
    {
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    }
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    #else
        #ERROR
    #endif
#endif

    MHal_EMAC_Write_TAR(skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE);
    MHal_EMAC_Write_TCR(skb->len);

    //netif_stop_queue (dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    txq->trans_start = jiffies;
#else
    dev->trans_start = jiffies;
#endif
    dev_kfree_skb_irq(skb);
    //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
    return NETDEV_TX_OK;
}
#elif defined(TX_DESC_MODE)
//-------------------------------------------------------------------------------------------------
//Transmit packet.
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_TX_Desc_Reset(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

    LocPtr->tx_desc_write_index = 0;
    LocPtr->tx_desc_read_index = 0;
    LocPtr->tx_desc_queued_number = 0;
    LocPtr->tx_desc_count = 0;
    LocPtr->tx_desc_full_count = 0;
    LocPtr->tx_index = 0;
    MHal_EMAC_Write_LOW_PRI_TX_DESC_THRESHOLD(0x0);
}

static void MDev_EMAC_TX_Desc_Mode_Set(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);

    MHal_EMAC_Write_LOW_PRI_TX_DESC_BASE(LocPtr->TX_LP_DESC_BASE - MIU0_BUS_BASE);
    MHal_EMAC_Write_LOW_PRI_TX_DESC_THRESHOLD(TX_LOW_PRI_DESC_NUMBER|EMAC_RW_TX_DESC_EN_W);
    return;
}

static int MDev_EMAC_tx (struct sk_buff *skb, struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    struct netdev_queue *txq = netdev_get_tx_queue(dev, 0);
#endif
    unsigned long flags;
    dma_addr_t skb_addr;

#ifdef EMAC_10T_RANDOM_WAVEFORM
    if(LocPtr->private_flag & EMAC_PRIVATE_FLAG_10T_RANDOM)
    {
        return NETDEV_TX_BUSY;
    }
#endif

    if (skb->len > EMAC_TX_MAX_LEN)
    {
        EMAC_DBG("Wrong Tx len:%u\n", skb->len);
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    if (MHal_EMAC_LOW_PRI_TX_DESC_MODE_OVRN_Get() == 1)
    {
        EMAC_DBG("TX_DESC_MODE_OVRN\n");
        netif_stop_queue(dev);
        //spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    spin_lock_irqsave(&LocPtr->tx_lock, flags);

    skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
    LocPtr->tx_index ++;
    LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;

    memcpy((void*)skb_addr, skb->data, skb->len);

    if (!skb_addr)
    {
        EMAC_DBG("Can not get memory from EMAC area\n");
        spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return -ENOMEM;
    }

    // Store packet information (to free when Tx completed) //
    LocPtr->stats.tx_bytes += skb->len;


#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
    if((unsigned int)skb_addr < 0xC0000000UL)
    {
        Chip_Flush_Memory_Range((unsigned int)skb_addr&0x0FFFFFFFUL, skb->len);
    }
    else
    {
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    }
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        Chip_Flush_Memory_Range((unsigned int)skb_addr&0x0FFFFFFFUL, skb->len);
        //Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    #else
        #ERROR
    #endif
#endif

    LocPtr->tx_desc_list[LocPtr->tx_desc_write_index].addr = skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE;

    if(LocPtr->tx_desc_write_index == TX_LOW_PRI_DESC_NUMBER - 1)
    {
        LocPtr->tx_desc_list[LocPtr->tx_desc_write_index].low_tag = (skb->len & 0x3FFFUL) | EMAC_TX_DESC_WRAP;
#ifdef CHIP_FLUSH_READ
    Chip_Flush_Memory_Range((u32)(&(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index])) & 0x0FFFFFFF, sizeof(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index]));
#endif

        LocPtr->tx_desc_write_index = 0;
    }
    else
    {
        LocPtr->tx_desc_list[LocPtr->tx_desc_write_index].low_tag = skb->len & 0x3FFFUL;
#ifdef CHIP_FLUSH_READ
    Chip_Flush_Memory_Range((u32)(&(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index])) & 0x0FFFFFFF, sizeof(LocPtr->tx_desc_list[LocPtr->tx_desc_write_index]));
#endif
        LocPtr->tx_desc_write_index++;
    }

    if(LocPtr->tx_desc_count % 2 == 0)
    {
        //EMAC_DBG("0x%zx\n", LocPtr->tx_desc_trigger_flag);
        MHal_EMAC_Write_LOW_PRI_TX_DESC_TRANSMIT0(0x1);
    }
    else
    {
        //EMAC_DBG("0x%zx\n", LocPtr->tx_desc_trigger_flag);
        MHal_EMAC_Write_LOW_PRI_TX_DESC_TRANSMIT1(0x1);
    }

    LocPtr->tx_desc_count++;

    spin_unlock_irqrestore(&LocPtr->tx_lock, flags);

    //netif_stop_queue (dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    txq->trans_start = jiffies;
#else
    dev->trans_start = jiffies;
#endif
    dev_kfree_skb_irq(skb);

    return NETDEV_TX_OK;
}
#else
//-------------------------------------------------------------------------------------------------
//Transmit packet.
//-------------------------------------------------------------------------------------------------
#ifdef EMAC_XIU16_SW_PROTECT
static int MDev_EMAC_CheckTSR(struct net_device *dev)
#else
static int MDev_EMAC_CheckTSR(void)
#endif /* EMAC_XIU16_SW_PROTECT */
{
#ifdef EMAC_XIU16_SW_PROTECT
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#endif /* EMAC_XIU16_SW_PROTECT */
    u32 tsrval = 0;

    #ifdef TX_QUEUE_4
    u8  avlfifo[8] = {0};
    u8  avlfifoidx;
    u8  avlfifoval = 0;

    //for (check = 0; check < EMAC_CHECK_CNT; check++)
    {
        tsrval = MHal_EMAC_Read_TSR();

        avlfifo[0] = ((tsrval & EMAC_IDLETSR) != 0)? 1 : 0;
        avlfifo[1] = ((tsrval & EMAC_BNQ)!= 0)? 1 : 0;
        avlfifo[2] = ((tsrval & EMAC_TBNQ) != 0)? 1 : 0;
        avlfifo[3] = ((tsrval & EMAC_FBNQ) != 0)? 1 : 0;
        avlfifo[4] = ((tsrval & EMAC_FIFO1IDLE) !=0)? 1 : 0;
        avlfifo[5] = ((tsrval & EMAC_FIFO2IDLE) != 0)? 1 : 0;
        avlfifo[6] = ((tsrval & EMAC_FIFO3IDLE) != 0)? 1 : 0;
        avlfifo[7] = ((tsrval & EMAC_FIFO4IDLE) != 0)? 1 : 0;

        avlfifoval = 0;

        for(avlfifoidx = 0; avlfifoidx < 8; avlfifoidx++)
        {
            avlfifoval += avlfifo[avlfifoidx];
        }

#ifdef EMAC_XIU16_SW_PROTECT
        if (avlfifoval > 4) {
            LocPtr->ThisUVE.tsrErrCnt = 0;
            return NETDEV_TX_OK;
        }
#else
        if (avlfifoval > 4)
            return NETDEV_TX_OK;
#endif /* EMAC_XIU16_SW_PROTECT */
    }
    #else
    //for (check = 0; check < EMAC_CHECK_CNT; check++)
    {
        tsrval = MHal_EMAC_Read_TSR();
        if ((tsrval & EMAC_IDLETSR) || (tsrval & EMAC_BNQ))
            return NETDEV_TX_OK;
    }
    #endif

#ifndef TX_SOFTWARE_QUEUE
    //EMAC_DBG("Err CheckTSR:0x%x\n", tsrval);
    //MDev_EMAC_TxReset();
#endif

#ifdef EMAC_XIU16_SW_PROTECT
    LocPtr->ThisUVE.tsrErrCnt++;
    if (LocPtr->ThisUVE.tsrErrCnt > 500000) {
        EMAC_DBG("Err CheckTSR:0x%x for %d times, do tx reset\n", tsrval, LocPtr->ThisUVE.tsrErrCnt);
        MDev_EMAC_TxReset_CTL();
    }
#endif /* EMAC_XIU16_SW_PROTECT */

    return NETDEV_TX_BUSY;
}

static int MDev_EMAC_tx (struct sk_buff *skb, struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    struct netdev_queue *txq = netdev_get_tx_queue(dev, 0);
#endif
    unsigned long flags;
    dma_addr_t skb_addr;

#ifdef EMAC_10T_RANDOM_WAVEFORM
    if(LocPtr->private_flag & EMAC_PRIVATE_FLAG_10T_RANDOM)
    {
        return NETDEV_TX_BUSY;
    }
#endif

    spin_lock_irqsave(&LocPtr->tx_lock, flags);

    if (skb->len > EMAC_TX_MAX_LEN)
    {
        EMAC_DBG("Wrong Tx len:%u\n", skb->len);
        spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

#ifdef EMAC_XIU16_SW_PROTECT
    if (NETDEV_TX_OK != MDev_EMAC_CheckTSR(dev))
#else
    if (NETDEV_TX_OK != MDev_EMAC_CheckTSR())
#endif /* EMAC_XIU16_SW_PROTECT */
    {
        spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return NETDEV_TX_BUSY; //check
    }


    skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
    LocPtr->tx_index ++;
    LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;

    memcpy((void*)skb_addr, skb->data, skb->len);

    if (!skb_addr)
    {
        EMAC_DBG("Can not get memory from EMAC area\n");
        spin_unlock_irqrestore(&LocPtr->tx_lock, flags);
        return -ENOMEM;
    }

    // Store packet information (to free when Tx completed) //
    LocPtr->stats.tx_bytes += skb->len;


#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
    if((unsigned int)skb_addr < 0xC0000000UL)
    {
        Chip_Flush_Memory_Range((unsigned int)skb_addr&0x0FFFFFFFUL, skb->len);
    }
    else
    {
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    }
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        Chip_Flush_Memory_Range(0, 0xFFFFFFFFUL);
    #else
        #ERROR
    #endif
#endif
    //Moniter TX packet
    //MDrv_EMAC_DumpMem(skb_addr, skb->len);

    //Set address of the data in the Transmit Address register //
    MHal_EMAC_Write_TAR(skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE);

    // Set length of the packet in the Transmit Control register //
    MHal_EMAC_Write_TCR(skb->len);

    //netif_stop_queue (dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
    txq->trans_start = jiffies;
#else
    dev->trans_start = jiffies;
#endif
    dev_kfree_skb_irq(skb);
    spin_unlock_irqrestore(&LocPtr->tx_lock, flags);

    return NETDEV_TX_OK;
}
#endif
#endif

#ifdef RX_ZERO_COPY
extern void Chip_Flush_Miu_Pipe(void);
//-------------------------------------------------------------------------------------------------
// Extract received frame from buffer descriptors and sent to upper layers.
// (Called from interrupt context)
// (Enable RX software discriptor)
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_RX_DESC_Init_zero_copy(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    struct sk_buff *skb = NULL;
    u32 skb_addr;
    u32 rx_desc_index;
    int retry_cnt = 0;
    u32 tmp_desc_addr = 0;
#ifdef EMAC_RX_ADDRESS_34BIT
    u32 tmp_desc_high_tag = 0;
#endif

    MDev_EMAC_RX_DESC_close_zero_copy(dev);

    memset((void *)LocPtr->RX_DESC_BASE + LocPtr->RAM_VA_PA_OFFSET, 0x00UL, RX_DESC_TABLE_SIZE);

    for(rx_desc_index = 0; rx_desc_index < RX_DESC_NUMBER; rx_desc_index++)
    {
        skb = alloc_skb(RX_BUFF_ENTRY_SIZE, GFP_ATOMIC);

        while (!skb) {
            if (retry_cnt > 10) {
                EMAC_DBG("MDev_EMAC_RX_DESC_Init_zero_copy: alloc skb fail for %d times!\n", retry_cnt);
                BUG_ON(1);
            }
            msleep(10);
            skb = alloc_skb(RX_BUFF_ENTRY_SIZE, GFP_ATOMIC);
            retry_cnt++;
        }
        retry_cnt = 0;

#ifdef CHIP_FLUSH_READ
        Chip_Flush_Cache_Range((unsigned long)(skb->data), RX_BUFF_ENTRY_SIZE);
#endif

#if defined(CONFIG_MIPS)
        skb_addr = (u32)virt_to_phys(skb->data) - MIU0_BUS_BASE;
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
#ifdef MAC_UMA
    skb_addr = (u32)__virt_to_phys(skb->data) - MSTAR_MIU0_BUS_BASE;
#else
        if((skb_addr = (u32)__virt_to_phys(skb->data)) >= MSTAR_MIU1_BUS_BASE)
        {
            skb_addr -= MSTAR_MIU1_BUS_BASE;

    #ifdef EMAC_RX_ADDRESS_32BIT
            skb_addr |= EMAC_MIU1_ADDR;
    #endif

    #ifdef EMAC_RX_ADDRESS_34BIT
            tmp_desc_high_tag |= EMAC_MIU1_ADDR;
    #endif
        }
        else
        {
            skb_addr -= MSTAR_MIU0_BUS_BASE;

    #ifdef EMAC_RX_ADDRESS_32BIT
            skb_addr &= ~EMAC_MIU1_ADDR;
    #endif

    #ifdef EMAC_RX_ADDRESS_34BIT
            tmp_desc_high_tag &= ~EMAC_MIU1_ADDR;
    #endif
        }
#endif
#endif

        LocPtr->rx_desc_sk_buff_list[rx_desc_index] = skb;
        if(rx_desc_index < (RX_DESC_NUMBER - 1))
        {
            tmp_desc_addr = (u32)skb_addr;
        }
        else
        {
            tmp_desc_addr = (u32)skb_addr + EMAC_DESC_WRAP;
        }

        /* Set descriptor in one time */
        LocPtr->rx_desc_list[rx_desc_index].addr = tmp_desc_addr;
#ifdef EMAC_RX_ADDRESS_34BIT
        LocPtr->rx_desc_list[rx_desc_index].size = tmp_desc_high_tag;
#endif

#ifdef CHIP_FLUSH_READ
        #if defined(CONFIG_MIPS)
            Chip_Flush_Memory_Range((u32)(&(LocPtr->rx_desc_list[rx_desc_index].addr)) & 0x0FFFFFFF, sizeof(LocPtr->rx_desc_list[rx_desc_index].addr));
        #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
            //Chip_Flush_Cache_Range_VA_PA((unsigned long)(&(LocPtr->rx_desc_list[rx_desc_index].addr)),(unsigned long)(&(LocPtr->rx_desc_list[rx_desc_index].addr)) - LocPtr->RAM_VA_PA_OFFSET ,sizeof(LocPtr->rx_desc_list[rx_desc_index].addr));
        #else
            //#ERROR
        #endif
#endif
        skb = NULL;
    }

#ifdef CHIP_FLUSH_READ
    Chip_Flush_Miu_Pipe();
#endif

    LocPtr->rx_desc_read_index = 0;
    LocPtr->rx_desc_free_index = 0;
    LocPtr->rx_desc_free_number = RX_DESC_NUMBER;

    MHal_EMAC_Write_RBQP(LocPtr->RX_DESC_BASE - MIU0_BUS_BASE);
    MHal_EMAC_Write_BUFF(EMAC_CLEAR_BUFF);

    return;
}

static void MDev_EMAC_RX_DESC_close_zero_copy(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 rx_desc_index;

    for(rx_desc_index = 0; rx_desc_index < RX_DESC_NUMBER; rx_desc_index++)
    {
        if(LocPtr->rx_desc_sk_buff_list[rx_desc_index])
            dev_kfree_skb_any(LocPtr->rx_desc_sk_buff_list[rx_desc_index]);
        LocPtr->rx_desc_sk_buff_list[rx_desc_index] = NULL;
    }

    return;
}

#ifdef RX_NAPI
static int MDev_EMAC_rx (struct net_device *dev, u32 field)
#else
static int MDev_EMAC_rx (struct net_device *dev)
#endif
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 pktlen=0;
    u32 received_number=0;
    struct sk_buff *skb = NULL;
    int retry_cnt = 0;
    u32 tmp_desc_addr = 0;
#ifdef EMAC_RX_ADDRESS_34BIT
    u32 tmp_desc_high_tag = 0;
#endif
    int ret = 0;

    do {
#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
        Chip_Read_Memory_Range((u32)(&(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr)) & 0x0FFFFFFF, sizeof(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr));
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        //Chip_Inv_Cache_Range((unsigned long)(&(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr)), sizeof(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr));
    #else
        //#ERROR
    #endif
#endif
        if (!((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr) & EMAC_DESC_DONE))
            break;

        if (LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index] == NULL) {
            LocPtr->stats.rx_dropped++;
            goto RX_recieve_end;
        }

#ifdef EMAC_GMAC_DESCRIPTOR
        pktlen = ((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].high_tag & 0x7UL) << 11) | (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag& 0x7ffUL);    /* Length of frame including FCS */
#else
        pktlen = LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & 0x7ffUL;    /* Length of frame including FCS */
#endif

        if (pktlen > EMAC_RX_MAX_LEN)
        {
            EMAC_DBG("Warning: RX large packet (pktlen = %d)", pktlen);
        }
        else if (pktlen < 64)
        {
            EMAC_DBG("RX Packet RX length Error!!(pktlen = %d)(EMAC_MTU = %d)(EMAC_MIN = %d)", pktlen, EMAC_MTU, 64);
            dev_kfree_skb_any(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
            LocPtr->stats.rx_length_errors++;
            LocPtr->stats.rx_errors++;
            LocPtr->stats.rx_dropped++;

            goto RX_recieve_end;
        }

#ifdef CHIP_FLUSH_READ
#if defined(CONFIG_MIPS)
        if((u32)LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index] < 0xC0000000UL)
            Chip_Read_Memory_Range((u32)(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]) & 0x0FFFFFFFUL, pktlen);
        else
            Chip_Read_Memory_Range(0, 0xFFFFFFFFUL);
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        Chip_Inv_Cache_Range(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data, pktlen);
#else
        #ERROR
#endif
#endif

        /* slt-test */
        if (is_slt_test_running == true) {
            u32 reg_val;
            u32 config3_value = 0;
            int err_cnt_rx = 0;
            int err_cnt_tx = 0;
            int err_cnt_col = 0;
            dma_addr_t skb_addr;

            /* rx packet len is include 4 bytes crc */
            if (pktlen != (slt_packet_len + 4)) {
                printk(KERN_ERR "slt-test-eth: not test packet, skip it\n");
                dev_kfree_skb_any(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
                goto RX_recieve_end;
            }

            /* get test packet */
            skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
            LocPtr->tx_index++;
            LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;

            /* disable copy all frame mode */
            reg_val = MHal_EMAC_Read_CFG();
            reg_val &= ~(0x00000010UL);
            MHal_EMAC_Write_CFG(reg_val);

            printk("slt-test-eth: rx packet\n");
            pktlen -= 4;
            //MDrv_EMAC_DumpMem(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data, pktlen);

            /* EMAC error count */
            err_cnt_rx = (MHal_EMAC_Read_ALE() + MHal_EMAC_Read_ELR() + MHal_EMAC_Read_SEQE() + MHal_EMAC_Read_ROVR() + MHal_EMAC_Read_SE() + MHal_EMAC_Read_RJB());
            err_cnt_tx = (MHal_EMAC_Read_TUE() + MHal_EMAC_Read_CSE() + MHal_EMAC_Read_SQEE());
            err_cnt_col = (MHal_EMAC_Read_LCOL() + MHal_EMAC_Read_ECOL() + MHal_EMAC_Read_SCOL() + MHal_EMAC_Read_MCOL());

            /* compare tx rx packet data */
            if ((memcmp(skb_addr, LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data, slt_packet_len) != 0) || (err_cnt_rx != 0) || (err_cnt_tx != 0) || (err_cnt_col != 0)) {
                printk(KERN_ERR "slt-test-eth: rx failed, rx_err: %d, tx_err: %d, col_err: %d\n", err_cnt_rx, err_cnt_tx, err_cnt_col);
                MDrv_EMAC_DumpMem(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data, pktlen);
                is_slt_test_passed = false;
            } else {
                printk("slt-test-eth: passed\n");
            }

            dev_kfree_skb_any(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);

            /* recovery settings */
#ifdef RX_DELAY_INTERRUPT
            config3_value = DELAY_INTERRUPT_CONFIG;
#endif
#ifdef RX_CHECKSUM
            config3_value = config3_value | RX_CHECKSUM_ENABLE;
#endif
#ifdef RX_DESC_MODE
            config3_value = config3_value | SOFTWARE_DESCRIPTOR_ENABLE;
#endif
#ifdef HW_TX_CHECKSUM
            dev->features |= NETIF_F_IP_CSUM;
            config3_value = config3_value | TX_CHECKSUM_ENABLE;
#endif
            MHal_EMAC_Write_Network_config_register3(config3_value);

            is_slt_test_running = false;
            netif_start_queue(dev);

            goto RX_recieve_end;
        }
        pktlen = pktlen - 4;

        skb_put(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index], pktlen);

	LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->protocol = eth_type_trans (LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index], dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
	LocPtr->last_rx_time = jiffies;
#else
	dev->last_rx = jiffies;
#endif

#ifdef RX_CHECKSUM
#ifdef RX_FINE_CHECKSUM
        /* Use EMAC HW checksum function */
        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_DESC_IPV4) {
            if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_DESC_IP_CSUM) {
                /* IP length should not be bigger than mtu(1500) */
                if (be16_to_cpu((*(unsigned short *)(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data + 2))) > 1500) {
                    LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
                    //LocPtr->stats.rx_dropped += 1;
                    //EMAC_DBG("\x1b[37;41m Receive invalid packet due to IP length is too big(%d) \x1b[0m\n", be16_to_cpu((*(unsigned short *)(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data + 2))));
                    //EMAC_DBG("EMAC_DESC_TCP=0x%08x, EMAC_DESC_UDP=0x%08x, EMAC_DESC_IP_CSUM=0x%08x, EMAC_DESC_TCP_UDP_CSUM=0x%08x\n", EMAC_DESC_TCP, EMAC_DESC_UDP, EMAC_DESC_IP_CSUM, EMAC_DESC_TCP_UDP_CSUM);
                    //EMAC_DBG("pktlen=%d, des.size=0x%08x\n", pktlen, dlist->descriptors[LocPtr->rxBuffIndex].size);
                    //MDrv_EMAC_DumpMem(skb->data, skb->len);
                    //kfree_skb(skb);
                    //goto ERRSUM;
                } else {
                    LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_UNNECESSARY;
                }
            } else {
                LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
                //LocPtr->stats.rx_dropped += 1;
                //EMAC_DBG("\x1b[37;41m Receive invalid TCP packet due to error IP checksum \x1b[0m\n");
                //EMAC_DBG("EMAC_DESC_TCP=0x%08x, EMAC_DESC_UDP=0x%08x, EMAC_DESC_IP_CSUM=0x%08x, EMAC_DESC_TCP_UDP_CSUM=0x%08x\n", EMAC_DESC_TCP, EMAC_DESC_UDP, EMAC_DESC_IP_CSUM, EMAC_DESC_TCP_UDP_CSUM);
                //EMAC_DBG("pktlen=%d, des.size=0x%08x\n", pktlen, dlist->descriptors[LocPtr->rxBuffIndex].size);
                //MDrv_EMAC_DumpMem(skb->data, skb->len);
                //kfree_skb(skb);
                //goto ERRSUM;
            }

            if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_DESC_TCP) {
                /* only check non-fragmented packet, both MF and OFFSET are zero */
                if (((*(unsigned char *)(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data + 6)) & 0x2f) == 0) {
                    if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_DESC_TCP_UDP_CSUM) {
                        LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_UNNECESSARY;
                    } else {
                        LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
                        //LocPtr->stats.rx_dropped += 1;
                        //EMAC_DBG("\x1b[37;41m Receive invalid packet due to error TCP checksum \x1b[0m\n");
                        //EMAC_DBG("EMAC_DESC_TCP=0x%08x, EMAC_DESC_UDP=0x%08x, EMAC_DESC_IP_CSUM=0x%08x, EMAC_DESC_TCP_UDP_CSUM=0x%08x\n", EMAC_DESC_TCP, EMAC_DESC_UDP, EMAC_DESC_IP_CSUM, EMAC_DESC_TCP_UDP_CSUM);
                        //EMAC_DBG("pktlen=%d, des.size=0x%08x\n", pktlen, dlist->descriptors[LocPtr->rxBuffIndex].size);
                        //MDrv_EMAC_DumpMem(skb->data, skb->len);
                        //kfree_skb(skb);
                        //goto ERRSUM;
                    }
                } else {
                    LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
                }
            } else if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_DESC_UDP) {
                /* only check non-fragmented packet, both MF and OFFSET are zero */
                if (((*(unsigned char *)(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data + 6)) & 0x2f) == 0) {
                    /* unnecessary to check if UDP checksum is zero */
                    if ((*(unsigned short *)(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data + 26)) == 0x0) {
                        LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_UNNECESSARY;
                    } else {
                        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_DESC_TCP_UDP_CSUM) {
                            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_UNNECESSARY;
                        } else {
                            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
                            //LocPtr->stats.rx_dropped += 1;
                            //EMAC_DBG("\x1b[37;41m Receive invalid packet due to error UDP checksum \x1b[0m\n");
                            //EMAC_DBG("EMAC_DESC_TCP=0x%08x, EMAC_DESC_UDP=0x%08x, EMAC_DESC_IP_CSUM=0x%08x, EMAC_DESC_TCP_UDP_CSUM=0x%08x\n", EMAC_DESC_TCP, EMAC_DESC_UDP, EMAC_DESC_IP_CSUM, EMAC_DESC_TCP_UDP_CSUM);
                            //EMAC_DBG("pktlen=%d, des.size=0x%08x\n", pktlen, dlist->descriptors[LocPtr->rxBuffIndex].size);
                            //MDrv_EMAC_DumpMem(skb->data, skb->len);
                            //kfree_skb(skb);
                            //goto ERRSUM;
                        }
                    }
                } else {
                    LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
                }
            } else {
                LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
            }
        } else {
            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
        }
#else
#ifdef EMAC_GMAC_DESCRIPTOR
        if(((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_TCP ) || (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_UDP )) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_IP_CSUM) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_TCP_UDP_CSUM) )
            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_UNNECESSARY;
        else
            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
#else
        if(((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_TCP ) || (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_UDP )) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_IP_CSUM) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_TCP_UDP_CSUM) )
            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_UNNECESSARY;
        else
            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->ip_summed = CHECKSUM_NONE;
#endif
#endif
#endif

        /* nr_frags should be 0, and the value of it might be changed by EMAC HW due to EMAC HW issue */
        if (skb_shinfo(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index])->nr_frags != 0) {
            printk("\n");
            printk("[%s %s %d] rx_desc_read_index = %d\n", __FILE__, __FUNCTION__, __LINE__, LocPtr->rx_desc_read_index);
            printk("[%s %s %d] pktlen     = %d\n", __FILE__, __FUNCTION__, __LINE__, pktlen);
            printk("[%s %s %d] head_p     = 0x%p\n", __FILE__, __FUNCTION__, __LINE__, LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->head);
            printk("[%s %s %d] data_p     = 0x%p\n", __FILE__, __FUNCTION__, __LINE__, LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->data);
            printk("[%s %s %d] tail_p     = 0x%p\n", __FILE__, __FUNCTION__, __LINE__, LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->tail);
            printk("[%s %s %d] end_p      = 0x%p\n", __FILE__, __FUNCTION__, __LINE__, LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->end);
            printk("[%s %s %d] nr_frags_p = 0x%p\n", __FILE__, __FUNCTION__, __LINE__, &(skb_shinfo(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index])->nr_frags));
            printk("[%s %s %d] nr_frags = 0x%x\n", __FILE__, __FUNCTION__, __LINE__, skb_shinfo(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index])->nr_frags);
            printk("\n");
            MDrv_EMAC_DumpMem(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->head, 0x2000);
            printk("\n");
            memset(&(skb_shinfo(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index])->nr_frags), 0x0, 0x28);
            memset(&(skb_shinfo(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index])->nr_frags) + 0x24, 0x01, 0x1);
            MDrv_EMAC_DumpMem(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]->head, 0x1000);

            dev_kfree_skb_any(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);

            LocPtr->stats.rx_length_errors++;
            LocPtr->stats.rx_errors++;
            LocPtr->stats.rx_dropped++;

            goto RX_recieve_end;
        }

#ifdef RX_NAPI
#ifdef RX_GRO
        if (field == 2)
            ret = netif_rx(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
        else
            ret = napi_gro_receive(&LocPtr->napi_rx,LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
#else
        if (field == 2)
            ret = netif_rx(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
        else
            ret = netif_receive_skb(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
#endif
#else
        ret = netif_rx(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_read_index]);
#endif

        if ((ret == NET_RX_DROP) || (ret == GRO_DROP)) {
            LocPtr->stats.rx_dropped++;
            if (printk_ratelimit()) {
                EMAC_NOTI("receive skb failed\n");
            }
        }

        LocPtr->stats.rx_bytes += pktlen;

#ifdef EMAC_GMAC_DESCRIPTOR
        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag& EMAC_MULTICAST)
            LocPtr->stats.multicast++;
#else
        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_MULTICAST)
            LocPtr->stats.multicast++;
#endif

RX_recieve_end:

        received_number++;
        LocPtr->rx_desc_free_number--;
        if (LocPtr->rx_desc_read_index == RX_DESC_NUMBER - 1)
            LocPtr->rx_desc_read_index = 0;
        else
            LocPtr->rx_desc_read_index++;

#ifdef RX_NAPI
        if (field == 2) {
            /* leave loop, or ISR of RX will always keep in running */
            //if (received_number >= (RX_DESC_NUMBER - 1))
            if (received_number >= 350)
                break;
        } else {
            if (received_number >= LocPtr->napi_rx.weight)
                break;
        }
#endif
    }while(1);

    while (LocPtr->rx_desc_free_number < RX_DESC_NUMBER) {
        tmp_desc_addr = 0;
#ifdef EMAC_RX_ADDRESS_34BIT
        tmp_desc_high_tag = 0;
#endif
        //LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_free_index] = dev_alloc_skb(RX_BUFF_ENTRY_SIZE);
        skb = alloc_skb(RX_BUFF_ENTRY_SIZE, GFP_ATOMIC);
        //skb_reserve(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_free_index], 16);

        u32 no_memory_flag;

        no_memory_flag = false;
        retry_cnt = 0;

        while (!skb) {
            if (retry_cnt > 10) {
                EMAC_DBG("MDev_EMAC_rx: alloc skb fail for %d times!\n", retry_cnt);
                no_memory_flag = true;
                mdelay(100);
                break;
            }
            skb = alloc_skb(RX_BUFF_ENTRY_SIZE, GFP_ATOMIC);
            retry_cnt++;
        }

        if (no_memory_flag) {
            LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_free_index] = NULL;
            tmp_desc_addr = LocPtr->rx_drop_base;
        } else {

#ifdef CHIP_FLUSH_READ
        Chip_Flush_Cache_Range((unsigned long)(skb->data), RX_BUFF_ENTRY_SIZE);
#endif

        LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_free_index] = skb;
        skb = NULL;

#if defined(CONFIG_MIPS)
            tmp_desc_addr = (u32)virt_to_phys(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_free_index]->data);
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
            tmp_desc_addr = (u32)__virt_to_phys(LocPtr->rx_desc_sk_buff_list[LocPtr->rx_desc_free_index]->data);
#endif
        }
#ifdef MAC_UMA
        tmp_desc_addr -= MSTAR_MIU0_BUS_BASE;
#else
        if (tmp_desc_addr >= MSTAR_MIU1_BUS_BASE) {
            tmp_desc_addr -= MSTAR_MIU1_BUS_BASE;

#ifdef EMAC_RX_ADDRESS_32BIT
            tmp_desc_addr |= EMAC_MIU1_ADDR;
#endif

#ifdef EMAC_RX_ADDRESS_34BIT
            tmp_desc_high_tag |= EMAC_MIU1_ADDR;
#endif
        } else {
            tmp_desc_addr -= MSTAR_MIU0_BUS_BASE;

#ifdef EMAC_RX_ADDRESS_32BIT
            tmp_desc_addr &= ~EMAC_MIU1_ADDR;
#endif

#ifdef EMAC_RX_ADDRESS_34BIT
            tmp_desc_high_tag &= ~EMAC_MIU1_ADDR;
#endif
        }
#endif

        /* Set descriptor in one time */
        if (LocPtr->rx_desc_free_index == (RX_DESC_NUMBER - 1))
            tmp_desc_addr |= EMAC_DESC_WRAP;

        LocPtr->rx_desc_list[LocPtr->rx_desc_free_index].addr = tmp_desc_addr;
#ifdef EMAC_RX_ADDRESS_34BIT
        LocPtr->rx_desc_list[LocPtr->rx_desc_free_index].size = tmp_desc_high_tag;
#endif

#ifdef CHIP_FLUSH_READ
#if defined(CONFIG_MIPS)
        Chip_Flush_Memory_Range((u32)(&(LocPtr->rx_desc_list[LocPtr->rx_desc_free_index].addr)) & 0x0FFFFFFF, sizeof(LocPtr->rx_desc_list[LocPtr->rx_desc_free_index].addr));
#endif
#endif

#ifdef CHIP_FLUSH_READ
        Chip_Flush_Miu_Pipe();
#endif
        if (LocPtr->rx_desc_free_index < (RX_DESC_NUMBER - 1))
            LocPtr->rx_desc_free_index++;
        else
            LocPtr->rx_desc_free_index = 0;

        LocPtr->rx_desc_free_number++;
    }

    return received_number;
}
#else //#ifdef RX_SOFTWARE_DESCRIPTOR

//-------------------------------------------------------------------------------------------------
// Extract received frame from buffer descriptors and sent to upper layers.
// (Called from interrupt context)
// (Disable RX software discriptor)
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_RX_DESC_Init_memcpy(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 rx_desc_index;

    memset((u8*)LocPtr->RAM_VA_PA_OFFSET + LocPtr->RX_BUFFER_BASE, 0x00UL, RX_BUFF_SIZE);

    for (rx_desc_index = 0; rx_desc_index < RX_DESC_NUMBER; rx_desc_index++) {
        if (rx_desc_index < (RX_DESC_NUMBER - 1)) {
            LocPtr->rx_desc_list[rx_desc_index].addr = LocPtr->RX_BUFFER_BASE - MIU0_BUS_BASE + RX_BUFF_ENTRY_SIZE * rx_desc_index;
#ifdef YMY_MAC_2BYTE_ALIGNMENT
            LocPtr->rx_desc_list[rx_desc_index].addr &= ~0x2UL;
#endif
        } else {
            LocPtr->rx_desc_list[rx_desc_index].addr = (LocPtr->RX_BUFFER_BASE - MIU0_BUS_BASE + RX_BUFF_ENTRY_SIZE * rx_desc_index) | EMAC_DESC_WRAP;
        }
#ifdef YMY_MAC_2BYTE_ALIGNMENT
            LocPtr->rx_desc_list[rx_desc_index].size |= 0x1 >> 18;
#endif
    }

    return;
}

static void MDev_EMAC_RX_DESC_Reset_memcpy(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 rx_desc_index;

    for(rx_desc_index = 0; rx_desc_index < RX_DESC_NUMBER; rx_desc_index++)
    {
        LocPtr->rx_desc_list[rx_desc_index].addr &= ~EMAC_DESC_DONE;
    }

    // Program address of descriptor list in Rx Buffer Queue register //
    MHal_EMAC_Write_RBQP(LocPtr->RX_DESC_BASE - MIU0_BUS_BASE);

    //Reset buffer index//
    LocPtr->rx_desc_read_index = 0;

    return;
}

#ifdef RX_NAPI
static int MDev_EMAC_rx (struct net_device *dev, u32 field)
#else
static int MDev_EMAC_rx (struct net_device *dev)
#endif
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    unsigned char *p_recv;
    u32 pktlen;
    int ret = 0;
    u32 received_number=0;
    struct sk_buff *skb;

#ifndef RX_DELAY_INTERRUPT
    u32 uRegVal=0;
    int count = 0;
#endif

    // If any Ownership bit is 1, frame received.
    //while ( (dlist->descriptors[LocPtr->rxBuffIndex].addr )& EMAC_DESC_DONE)
    do
    {
#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
        Chip_Read_Memory_Range((u32)(&(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index])) & 0x0FFFFFFFUL, sizeof(struct rx_descriptor));
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        //Chip_Inv_Cache_Range_VA_PA((u32)(&LocPtr->rx_desc_list[LocPtr->rx_desc_read_index]),
        //    (u32)(&LocPtr->rx_desc_list[LocPtr->rx_desc_read_index]) - LocPtr->RAM_VA_PA_OFFSET ,sizeof(struct rx_descriptor));
    #else
        #ERROR
    #endif
#endif
        if(!((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr) & EMAC_DESC_DONE))
        {
                break;
        }

        p_recv = (char *) ((((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr) & 0xFFFFFFFFUL) + LocPtr->RAM_VA_PA_OFFSET + MIU0_BUS_BASE) & ~(EMAC_DESC_DONE | EMAC_DESC_WRAP));
#ifdef EMAC_GMAC_DESCRIPTOR
        pktlen = ((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].high_tag & 0x7UL) << 11) | (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag& 0x7ffUL);    /* Length of frame including FCS */
#else
        pktlen = LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & 0x7ffUL;    /* Length of frame including FCS */
#endif

        if (pktlen > EMAC_RX_MAX_LEN)
        {
            EMAC_DBG("Warning: RX large packet (pktlen = %d)", pktlen);
        }
        else if (pktlen < 64)
        {
            EMAC_DBG("Packet RX too small!!(pktlen = %d)", pktlen);

            LocPtr->stats.rx_length_errors++;
            LocPtr->stats.rx_errors++;
            LocPtr->stats.rx_dropped++;

            goto RX_recieve_memcpy_end;
        }

        /* slt-test */
        if (is_slt_test_running == true) {
            u32 reg_val;
            u32 config3_value = 0;
            int err_cnt_rx = 0;
            int err_cnt_tx = 0;
            int err_cnt_col = 0;
            dma_addr_t skb_addr;

            /* rx packet len is include 4 bytes crc */
            if (pktlen != (slt_packet_len + 4)) {
                printk(KERN_ERR "slt-test-eth: not test packet, skip it\n");
                goto RX_recieve_memcpy_end;
            }

            /* get test packet */
            skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
            LocPtr->tx_index++;
            LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;

            /* disable copy all frame mode */
            reg_val = MHal_EMAC_Read_CFG();
            reg_val &= ~(0x00000010UL);
            MHal_EMAC_Write_CFG(reg_val);

            printk("slt-test-eth: rx packet\n");
            pktlen -= 4;
            //MDrv_EMAC_DumpMem(p_recv, pktlen);

            /* EMAC error count */
            err_cnt_rx = (MHal_EMAC_Read_ALE() + MHal_EMAC_Read_ELR() + MHal_EMAC_Read_SEQE() + MHal_EMAC_Read_ROVR() + MHal_EMAC_Read_SE() + MHal_EMAC_Read_RJB());
            err_cnt_tx = (MHal_EMAC_Read_TUE() + MHal_EMAC_Read_CSE() + MHal_EMAC_Read_SQEE());
            err_cnt_col = (MHal_EMAC_Read_LCOL() + MHal_EMAC_Read_ECOL() + MHal_EMAC_Read_SCOL() + MHal_EMAC_Read_MCOL());

            /* compare tx rx packet data */
            if ((memcmp(skb_addr, p_recv, slt_packet_len) != 0) || (err_cnt_rx != 0) || (err_cnt_tx != 0) || (err_cnt_col != 0)) {
                printk(KERN_ERR "slt-test-eth: rx failed, rx_err: %d, tx_err: %d, col_err: %d\n", err_cnt_rx, err_cnt_tx, err_cnt_col);
                printk(KERN_ERR "++++++++++++ p_recv +++++++++++\n");
                MDrv_EMAC_DumpMem(p_recv, slt_packet_len);
                printk(KERN_ERR "++++++++++++ skb_addr +++++++++++\n");
                MDrv_EMAC_DumpMem(skb_addr, slt_packet_len);
                is_slt_test_passed = false;
            } else {
                printk("slt-test-eth: passed\n");
            }

            /* recovery settings */
#ifdef RX_DELAY_INTERRUPT
            config3_value = DELAY_INTERRUPT_CONFIG;
#endif
#ifdef RX_CHECKSUM
            config3_value = config3_value | RX_CHECKSUM_ENABLE;
#endif
#ifdef RX_DESC_MODE
            config3_value = config3_value | SOFTWARE_DESCRIPTOR_ENABLE;
#endif
#ifdef HW_TX_CHECKSUM
            dev->features |= NETIF_F_IP_CSUM;
            config3_value = config3_value | TX_CHECKSUM_ENABLE;
#endif
            MHal_EMAC_Write_Network_config_register3(config3_value);

            is_slt_test_running = false;
            netif_start_queue(dev);

            goto RX_recieve_memcpy_end;
        }

        skb = alloc_skb (pktlen + 6, GFP_ATOMIC);

        if (skb != NULL)
        {
            skb_reserve (skb, 2);
    #ifdef CHIP_FLUSH_READ
        #if defined(CONFIG_MIPS)
           if((u32)p_recv < 0xC0000000UL)
           {
               Chip_Read_Memory_Range((u32)(p_recv) & 0x0FFFFFFFUL, pktlen);
           }
           else
           {
               Chip_Read_Memory_Range(0, 0xFFFFFFFFUL);
           }
        #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
           Chip_Inv_Cache_Range_VA_PA((size_t)p_recv,(size_t)p_recv - LocPtr->RAM_VA_PA_OFFSET ,pktlen);
        #else
            #ERROR
        #endif
    #endif
            memcpy(skb_put(skb, pktlen), p_recv, pktlen);
            skb->protocol = eth_type_trans (skb, dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
	    LocPtr->last_rx_time = jiffies;
#else
	    dev->last_rx = jiffies;
#endif
            LocPtr->stats.rx_bytes += pktlen;

            if (is_wol_woc_wake) {
                pm_wakeup_event(LocPtr->ddev, 50);
            }

    #ifdef RX_CHECKSUM
        #ifdef EMAC_GMAC_DESCRIPTOR
        if(((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_TCP ) || (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_UDP )) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_IP_CSUM) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_DESC_TCP_UDP_CSUM) )
        {
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        }
        else
        {
            skb->ip_summed = CHECKSUM_NONE;
        }
        #else
        if(((LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_TCP ) || (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_UDP )) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_IP_CSUM) && \
            (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size& EMAC_DESC_TCP_UDP_CSUM) )
        {
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        }
        else
        {
            skb->ip_summed = CHECKSUM_NONE;
        }
        #endif
    #endif

        #ifdef RX_NAPI
        #ifdef RX_GRO
            if (field == 2)
                ret = netif_rx(skb);
            else
                ret = napi_gro_receive(&LocPtr->napi_rx, skb);
        #else
            if (field == 2)
                ret = netif_rx(skb);
            else
                ret = netif_receive_skb(skb);
        #endif
        #else
            ret = netif_rx(skb);
        #endif

            if ((ret == NET_RX_DROP) || (ret == GRO_DROP)) {
                LocPtr->stats.rx_dropped++;
                if (printk_ratelimit()) {
                    EMAC_NOTI("receive skb failed\n");
                }
            }

            received_number++;
        }
        else
        {
            EMAC_DBG("alloc_skb fail!!!\n");
            //LocPtr->stats.rx_dropped += 1;
        }

    #ifdef EMAC_GMAC_DESCRIPTOR
        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].low_tag & EMAC_MULTICAST)
        {
            LocPtr->stats.multicast++;
        }
    #else
        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_MULTICAST)
        {
            LocPtr->stats.multicast++;
#ifdef YMY_MAC_HASH_ADDRESS
            EMAC_DBG("multicast!!!\n");
#endif
        }
    #endif

#ifdef YMY_MAC_TAG
        if (LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].size & EMAC_TAG_MATCH)
        {
            EMAC_DBG("TAG!!!\n");
        }
#endif

RX_recieve_memcpy_end:

        LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr  &= ~EMAC_DESC_DONE;  /* reset ownership bit */
#ifdef CHIP_FLUSH_READ
    #if defined(CONFIG_MIPS)
        Chip_Flush_Memory_Range((u32)(&(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr)) & 0x0FFFFFFFUL, sizeof(LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr));
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        //Chip_Flush_Cache_Range_VA_PA((unsigned long)(&LocPtr->rx_desc_list[LocPtr->rx_desc_read_index]),
        //(unsigned long)(&LocPtr->rx_desc_list[LocPtr->rx_desc_read_index]) - LocPtr->RAM_VA_PA_OFFSET ,sizeof(struct rx_descriptor));
    #else
        #ERROR
    #endif
#endif

        //wrap after last buffer //
        LocPtr->rx_desc_read_index++;
        if (LocPtr->rx_desc_read_index == RX_DESC_NUMBER)
        {
            LocPtr->rx_desc_read_index = 0;
        }

    #ifdef RX_NAPI
        if (field == 2)
        {
            /* leave loop, or ISR of RX will always keep in running */
            //if (received_number >= (RX_DESC_NUMBER - 1))
            if (received_number >= 350)
                break;
        }
        else
        {
            if(received_number >= LocPtr->napi_rx.weight) {
                break;
            }
        }
    #endif

    }while(1);
    return received_number;
}
#endif //#ifdef RX_SOFTWARE_DESCRIPTOR

#ifdef RX_NAPI
// Enable MAC interrupts
static void MDEV_EMAC_ENABLE_RX_REG(void)
{
    u32 uRegVal;
    //printk( KERN_ERR "[EMAC] %s\n" , __FUNCTION__);
#ifdef EMAC_XIU16_SW_PROTECT
    int cnt = 0;
    u32 EMAC_INT_INV = ((~(EMAC_INT_ENABLE & 0x1FFF)) & 0x1FFF);
    MHal_EMAC_Write_RSR(EMAC_BNA);
    MHal_EMAC_Write_RSR(EMAC_DNA);
    MHal_EMAC_Write_RSR(EMAC_RSROVR);
#endif /* EMAC_XIU16_SW_PROTECT */

#ifndef RX_DELAY_INTERRUPT
    // disable MAC interrupts
    uRegVal = EMAC_INT_RCOM | EMAC_INT_ENABLE;
    MHal_EMAC_Write_IER(uRegVal);
#else
    uRegVal = EMAC_INT_ENABLE;
    MHal_EMAC_Write_IER(uRegVal);
    // enable delay interrupt
    uRegVal = MHal_EMAC_Read_Network_config_register3();
    uRegVal |= 0x00000080UL;
    MHal_EMAC_Write_Network_config_register3(uRegVal);
#endif

#ifdef EMAC_XIU16_SW_PROTECT
    uRegVal = MHal_EMAC_Read_IMR();
    while ((uRegVal & 0x1FFF) != EMAC_INT_INV) {
        printk(KERN_DEBUG "Interrupt Enable ERROR (EMAC_INT_INV = 0x%x, EMAC_INT_ENABLE = 0x%x, IMR = 0x%x)\n", EMAC_INT_INV, EMAC_INT_ENABLE, uRegVal);
        MHal_EMAC_Write_IDR(0xFFFF);
        MHal_EMAC_Write_IER(EMAC_INT_ENABLE);
        cnt++;
        if (cnt > 10) {
            EMAC_DBG("Enable interrupt fail over %d times\n", cnt);
            break;
        }
        uRegVal = MHal_EMAC_Read_IMR();
    }
#endif /* EMAC_XIU16_SW_PROTECT */
}

// Disable MAC interrupts
static void MDEV_EMAC_DISABLE_RX_REG(void)
{
    u32 uRegVal;
#ifdef EMAC_XIU16_SW_PROTECT
    int cnt = 0;
#endif /* EMAC_XIU16_SW_PROTECT */
    //printk( KERN_ERR "[EMAC] %s\n" , __FUNCTION__);
#ifndef RX_DELAY_INTERRUPT
    // Enable MAC interrupts
    uRegVal = EMAC_INT_RCOM | EMAC_INT_ENABLE;
    MHal_EMAC_Write_IDR(uRegVal);
#else
    uRegVal = 0xFFFF;
    MHal_EMAC_Write_IDR(uRegVal);
    // disable delay interrupt
    uRegVal = MHal_EMAC_Read_Network_config_register3();
    uRegVal &= ~(0x00000080UL);
    MHal_EMAC_Write_Network_config_register3(uRegVal);
#endif

#ifdef EMAC_XIU16_SW_PROTECT
    uRegVal = MHal_EMAC_Read_IMR();
    while ((uRegVal & 0x1FFF) != 0x1FFF) {
        printk(KERN_DEBUG "Interrupt Enable ERROR (EMAC_INT_ENABLE = 0x%x, IMR = 0x%x)\n", EMAC_INT_ENABLE, uRegVal);
        MHal_EMAC_Write_IDR(0xFFFF);
        cnt++;
        if (cnt > 10) {
            EMAC_DBG("Enable interrupt fail over %d times\n", cnt);
            break;
        }
        uRegVal = MHal_EMAC_Read_IMR();
    }
#endif /* EMAC_XIU16_SW_PROTECT */
}

static int MDev_EMAC_RX_napi_poll(struct napi_struct *napi, int budget)
{
    struct EMAC_private  *LocPtr = container_of(napi, struct EMAC_private, napi_rx);
    struct net_device *dev = LocPtr->dev;
    int work_done = 0;
    unsigned long flags;

    spin_lock(&LocPtr->rx_lock);
    work_done = MDev_EMAC_rx(dev, 0);
    spin_unlock(&LocPtr->rx_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
    if (work_done < budget && napi_complete_done(napi, work_done)) {
        spin_lock_irqsave(&LocPtr->irq_lock, flags);
        // enable MAC interrupt
        MDEV_EMAC_ENABLE_RX_REG();
        spin_unlock_irqrestore(&LocPtr->irq_lock, flags);
    }
#else
    if (work_done < budget) {
        spin_lock_irqsave(&LocPtr->irq_lock, flags);
	__napi_complete(napi);
        // enable MAC interrupt
        MDEV_EMAC_ENABLE_RX_REG();
        spin_unlock_irqrestore(&LocPtr->irq_lock, flags);
    }

#endif

    return work_done;
}
#endif

#ifdef TX_NAPI
// Enable MAC interrupts
static void MDEV_EMAC_ENABLE_TX_REG(void)
{
    MHal_EMAC_Write_IER(EMAC_INT_TCOM);
}

// Disable MAC interrupts
static void MDEV_EMAC_DISABLE_TX_REG(void)
{
    MHal_EMAC_Write_IDR(EMAC_INT_TCOM);
}

static int MDev_EMAC_TX_napi_poll(struct napi_struct *napi, int budget)
{
    struct EMAC_private  *LocPtr = container_of(napi, struct EMAC_private, napi_tx);
    struct net_device *dev = LocPtr->dev;
    unsigned long flags = 0;
    int work_done = 0;
    int count = 0;

    //EMAC_DBG("MDev_EMAC_TX_napi_poll, budget = %d\n", budget);

    while(work_done < budget)
    {
        //EMAC_DBG("work_done = %d, count = %d\n", work_done, count);

        if((count = MDev_EMAC_TX_Free_sk_buff(dev, work_done)) == 0)
            break;

        work_done += count;
    }

    /* If budget not fully consumed, exit the polling mode */
    if (work_done < budget) {
        napi_complete(napi);
        // enable MAC interrupt
        spin_lock_irqsave(&LocPtr->irq_lock, flags);
        MDEV_EMAC_ENABLE_TX_REG();
        spin_unlock_irqrestore(&LocPtr->irq_lock, flags);
    }
    else
    {
//        EMAC_DBG("TX Over Budget!!! \n");
    }

    return work_done;
}
#endif

#ifdef TX_COM_ENABLE
irqreturn_t MDev_EMAC_interrupt(int irq,void *dev_id)
{
    struct net_device *dev = (struct net_device *) dev_id;
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 intstatus = 0;
    u32 delay_int_status = 0;
    unsigned long flags;
#ifdef TX_SOFTWARE_QUEUE
    int txIdleCount=0;//MDev_EMAC_GetTXFIFOIdle();
#endif

#ifndef RX_ZERO_COPY
    u32 wp = 0;
#endif

    spin_lock_irqsave(&LocPtr->irq_lock, flags);
    //MAC Interrupt Status register indicates what interrupts are pending.
    //It is automatically cleared once read.
    delay_int_status = MHal_EMAC_Read_Delay_interrupt_status();
    intstatus = MHal_EMAC_Read_ISR() & (~(MHal_EMAC_Read_IMR())) & EMAC_INT_MASK;

#ifndef RX_ZERO_COPY
    wp = MHal_EMAC_Read_Network_config_register2() & 0x00100000UL;
    if(wp)
    {
        EMAC_DBG("EMAC HW write invalid address");
    }
#endif

    //while((delay_int_status & 0x8000UL) || (intstatus = (MHal_EMAC_Read_ISR() & ~MHal_EMAC_Read_IMR() & EMAC_INT_MASK )) )
    {
        if (intstatus & EMAC_INT_RBNA)
        {
            EMAC_DBG("RBNA!!!!\n");
            LocPtr->stats.rx_dropped ++;
            //GmacThisUVE.flagRBNA = 1;
            //write 1 clear
            MHal_EMAC_Write_RSR(EMAC_BNA);
        }

        // Transmit complete //
        if (intstatus & EMAC_INT_TCOM)
        {

            // The TCOM bit is set even if the transmission failed. //
            if (intstatus & (EMAC_INT_TUND | EMAC_INT_RTRY))
            {
                LocPtr->stats.tx_errors += 1;
                if(intstatus & EMAC_INT_TUND)
                {
                    //write 1 clear
                    MHal_EMAC_Write_TSR(EMAC_UND);

                    //Reset TX engine
                    MDev_EMAC_TxReset();
                    EMAC_DBG ("Transmit TUND error, TX reset\n");
                }
            }

        #ifdef TX_DESC_MODE
        #ifdef TX_NAPI
            /* Receive packets are processed by poll routine. If not running start it now. */
            if (napi_schedule_prep(&LocPtr->napi_tx)) {
                MDEV_EMAC_DISABLE_TX_REG();
                __napi_schedule(&LocPtr->napi_tx);
            }
        #else
            MDev_EMAC_TX_Free_sk_buff(dev, 0);
        #endif
            if (((LocPtr->ep_flag & EP_FLAG_SUSPENDING)==0) && netif_queue_stopped (dev) && (LocPtr->tx_desc_queued_number < TX_DESC_REFILL_NUMBER));
                netif_wake_queue(dev);
        #elif defined(TX_SOFTWARE_QUEUE)
            if (((LocPtr->ep_flag & EP_FLAG_SUSPENDING)==0) && netif_queue_stopped (dev))
                netif_wake_queue(dev);
            LocPtr->tx_irqcnt++;
            txIdleCount=MDev_EMAC_GetTXFIFOIdle();
            while(txIdleCount>0 && (LocPtr->tx_rdidx != LocPtr->tx_wridx))
            {

                _MDev_EMAC_tx_clear_TX_SW_QUEUE(txIdleCount,dev,TX_SW_QUEUE_IN_IRQ);
                _MDev_EMAC_tx_read_TX_SW_QUEUE(txIdleCount,dev,TX_SW_QUEUE_IN_IRQ);
                txIdleCount=MDev_EMAC_GetTXFIFOIdle();
            }
        #else
            if (((LocPtr->ep_flag & EP_FLAG_SUSPENDING)==0) && netif_queue_stopped (dev))
                netif_wake_queue(dev);
        #endif
        }

        if(intstatus & EMAC_INT_DONE)
        {
            LocPtr->ThisUVE.flagISR_INT_DONE = 0x01UL;
        }

        //RX Overrun //
        if(intstatus & EMAC_INT_ROVR)
        {
            EMAC_DBG("ROVR!!!!\n");
            LocPtr->stats.rx_dropped++;
            LocPtr->ROVR_count++;
            //write 1 clear
            MHal_EMAC_Write_RSR(EMAC_RSROVR);

            if (LocPtr->ROVR_count >= 6)
            {
                MDev_EMAC_SwReset(dev);
            }
        }
        else
        {
            LocPtr->ROVR_count = 0;
        }

        // Receive complete //
        if(delay_int_status & 0x8000UL)
        {
        #ifdef RX_NAPI
            /* Receive packets are processed by poll routine. If not running start it now. */
            if (napi_schedule_prep(&LocPtr->napi_rx)) {
                MDEV_EMAC_DISABLE_RX_REG();
                __napi_schedule(&LocPtr->napi_rx);
            }
        #else
            MDev_EMAC_rx(dev);
        #endif
        }
        //delay_int_status = MHal_EMAC_Read_Delay_interrupt_status();
    }
    spin_unlock_irqrestore(&LocPtr->irq_lock, flags);
    return IRQ_HANDLED;
}
#else
irqreturn_t MDev_EMAC_interrupt(int irq,void *dev_id)
{
    struct net_device *dev = (struct net_device *) dev_id;
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 intstatus = 0;
    u32 delay_int_status = 0;
    unsigned long flags;
#ifdef TX_SOFTWARE_QUEUE
    int txIdleCount=0;//MDev_EMAC_GetTXFIFOIdle();
#endif

#ifndef RX_ZERO_COPY
    u32 wp = 0;
#endif

#ifdef TX_ZERO_COPY
    MDev_EMAC_TX_Free_sk_buff(dev,0);
#endif

    spin_lock_irqsave(&LocPtr->irq_lock, flags);
    //MAC Interrupt Status register indicates what interrupts are pending.
    //It is automatically cleared once read.
    delay_int_status = MHal_EMAC_Read_Delay_interrupt_status();
    intstatus = MHal_EMAC_Read_ISR() & (~(MHal_EMAC_Read_IMR())) & EMAC_INT_MASK;

#ifndef RX_ZERO_COPY
    wp = MHal_EMAC_Read_Network_config_register2() & 0x00100000UL;
    if(wp)
    {
        EMAC_DBG("EMAC HW write invalid address");
    }
#endif

    //while((delay_int_status & 0x8000UL) || (intstatus = (MHal_EMAC_Read_ISR() & ~MHal_EMAC_Read_IMR() & EMAC_INT_MASK )) )
    {
        if (((LocPtr->ep_flag & EP_FLAG_SUSPENDING)==0) && netif_queue_stopped (dev))
        {
            netif_wake_queue(dev);
        }

#if defined(TX_QUEUE_EMPTY) && defined(TX_ZERO_COPY)
        if (intstatus & EMAC_INT_TXQ_EMPTY)
        {
            MDev_EMAC_TX_Free_sk_buff(dev,0);
        }
#endif

        if (intstatus & EMAC_INT_RBNA)
        {
            EMAC_DBG("RBNA!!!! read:%d, desc=0x%x\n", LocPtr->rx_desc_read_index, LocPtr->rx_desc_list[LocPtr->rx_desc_read_index].addr);
            LocPtr->stats.rx_dropped ++;
            //write 1 clear
            MHal_EMAC_Write_RSR(EMAC_BNA);
#ifdef RX_NAPI
            MDev_EMAC_rx(dev, 2);
#endif
        }

        // The TCOM bit is set even if the transmission failed. //
        if (intstatus & (EMAC_INT_TUND | EMAC_INT_RTRY))
        {
            LocPtr->stats.tx_errors += 1;

            if(intstatus & EMAC_INT_TUND)
            {
                //write 1 clear
                MHal_EMAC_Write_TSR(EMAC_UND);

                //Reset TX engine
                MDev_EMAC_TxReset();
                EMAC_DBG ("Transmit TUND error, TX reset\n");
            }
            else
            {
                EMAC_DBG("EMAC_INT_RTRY!!!!\n");
            }
        }

        if(intstatus & EMAC_INT_DONE)
        {
            LocPtr->ThisUVE.flagISR_INT_DONE = 0x01UL;
        }

        //RX Overrun //
        if(intstatus & EMAC_INT_ROVR)
        {
            EMAC_DBG("ROVR!!!!\n");
            LocPtr->stats.rx_dropped++;
            //write 1 clear
            MHal_EMAC_Write_RSR(EMAC_RSROVR);
        }

        // Receive complete //
        if(delay_int_status & 0x8000UL)
        {
            //MDev_EMAC_TX_Free_sk_buff(dev, 0);
        #ifdef RX_NAPI
            /* Receive packets are processed by poll routine. If not running start it now. */
            if (napi_schedule_prep(&LocPtr->napi_rx)) {
                MDEV_EMAC_DISABLE_RX_REG();
                __napi_schedule(&LocPtr->napi_rx);
            }
        #else
            MDev_EMAC_rx(dev);
        #endif
        }
        //delay_int_status = MHal_EMAC_Read_Delay_interrupt_status();
    }
    spin_unlock_irqrestore(&LocPtr->irq_lock, flags);
    return IRQ_HANDLED;
}
#endif
//-------------------------------------------------------------------------------------------------
// EMAC Hardware register set
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_ScanPhyAddr(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    unsigned char addr = 0;
    u32 value = 0;

    MHal_EMAC_enable_mdi();
    do
    {
        MHal_EMAC_read_phy(addr, MII_BMSR, &value);
        if (0 != value && 0x0000FFFFUL != value)
        {
            EMAC_DBG("[ PHY Addr ] ==> :%u\n", addr);
            break;
        }
    }while(++addr && addr < 32);
    MHal_EMAC_disable_mdi();
    LocPtr->phyaddr = addr;

    if (LocPtr->phyaddr >= 32)
    {
        EMAC_DBG("Wrong PHY Addr and reset to 0\n");
        LocPtr->phyaddr = 0;
        return -1;
    }
    return 0;
}

static void MDev_EMAC_Force_10(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 val;

    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_RESET);
    mdelay(10);
    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_ADVERTISE, ADVERTISE_CSMA|ADVERTISE_10HALF|ADVERTISE_10FULL);
    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_CTRL1000, 0x0);
    mdelay(10);
    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_ANENABLE|BMCR_ANRESTART);
}

static void MDev_EMAC_Force_100(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 val;

    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_RESET);
    mdelay(10);
    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_CTRL1000, 0x0);
    mdelay(10);
    MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, BMCR_ANENABLE|BMCR_ANRESTART);
}

static void MDev_EMAC_Force_Speed(struct net_device *dev)
{
#ifdef CONFIG_EMAC_FORCE_10
    MDev_EMAC_Force_10(dev);
#endif
#ifdef CONFIG_EMAC_FORCE_100
    MDev_EMAC_Force_100(dev);
#endif
}

static u32 MDev_EMAC_HW_Reg_init(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*) netdev_priv(dev);
    u32 word_ETH_CFG = 0x00000800UL;
    u32 word_ETH_CTL;
    u32 config3_Value = 0;
    u32 uNegPhyVal = 0;
#ifdef HW_TX_CHECKSUM
    u32 config4_Value = 0;
#endif
#ifdef CONFIG_MSTAR_JUMBO_PACKET
    u32 config5_Value = 0;
#endif

    if (!LocPtr->ThisUVE.initedEMAC)
    {
        MHal_EMAC_Power_On_Clk();
#ifdef CONFIG_EMAC_TR_PN_SWAP
        if (LocPtr->is_tr_pn_swap == true)
            MHal_EMAC_Swap_TR_PN();
#endif /* CONFIG_EMAC_TR_PN_SWAP */

        MHal_EMAC_Write_Network_config_register2(CONFIG2_VAL);

        if (MDev_EMAC_ScanPhyAddr(dev) < 0)
            return -1;

#ifdef CONFIG_MSTAR_ARM_BD_FPGA
        MDev_EMAC_Force_Speed(dev);
#endif
    }

#ifdef RX_DELAY_INTERRUPT
    config3_Value = DELAY_INTERRUPT_CONFIG;//0xFF050080;
#endif

#ifdef RX_CHECKSUM
    config3_Value = config3_Value | RX_CHECKSUM_ENABLE;
#endif

#ifdef RX_DESC_MODE
    config3_Value = config3_Value | SOFTWARE_DESCRIPTOR_ENABLE;
#endif

#ifdef HW_TX_CHECKSUM
    dev->features |= NETIF_F_IP_CSUM;
    config3_Value = config3_Value | TX_CHECKSUM_ENABLE;
#endif

    MHal_EMAC_Write_Network_config_register3(config3_Value);

#ifdef HW_TX_CHECKSUM_IPV6
    dev->features |= NETIF_F_IPV6_CSUM;
    config4_Value = MHal_EMAC_Read_Network_config_register4() | TX_V6_CHECKSUM_ENABLE;
    MHal_EMAC_Write_Network_config_register4(config4_Value);
#endif

#ifdef CONFIG_MSTAR_JUMBO_PACKET
    config5_Value = MHal_EMAC_Read_Network_config_register5() | TX_CHECKSUM_ENABLE;
    config5_Value &= 0xffff0000UL;
    config5_Value |= TX_JUMBO_FRAME_ENABLE;
    MHal_EMAC_Write_Network_config_register5(config5_Value);
#endif

#ifdef NEW_TX_QUEUE
    MHal_EMAC_New_TX_QUEUE_Enable();
    MHal_EMAC_New_TX_QUEUE_Threshold_Set(TX_BUFF_ENTRY_NUMBER);
#endif /* NEW_TX_QUEUE */

#ifdef INT_MASK_MODE
    MHal_EMAC_INT_MASK_DIVSEL_Set(INT_MASK_DIVSEL);
    MHal_EMAC_INT_MASK_CNT_Set(INT_MASK_CNT);
    MHal_EMAC_INT_MASK_Enable();
#endif /* INT_MASK_MODE */

    if(!(LocPtr->ep_flag & EP_FLAG_SUSPENDING))
        MDev_EMAC_get_mac_address (dev);    // Get ethernet address and store it in dev->dev_addr //

    MDev_EMAC_update_mac_address (dev); // Program ethernet address into MAC //

    if (!LocPtr->ThisUVE.initedEMAC)
    {
#ifdef CONFIG_EMAC_PHY_RESTART_AN
        MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, 0x1000UL);
        MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, 0x1000UL);
        //MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, 0x1200UL);
#else
        //MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, 0x9000UL);
        MHal_EMAC_write_phy(LocPtr->phyaddr, MII_BMCR, 0x1000UL);
#endif /* CONFIG_EMAC_PHY_RESTART_AN */
        // IMPORTANT: Run NegotiationPHY() before writing REG_ETH_CFG.
        uNegPhyVal = MHal_EMAC_NegotiationPHY( LocPtr->phyaddr );
        if(uNegPhyVal == 0x01UL)
        {
            LocPtr->ThisUVE.flagMacTxPermit = 0x01UL;
            LocPtr->ThisBCE.duplex = 1;

        }
        else if(uNegPhyVal == 0x02UL)
        {
            LocPtr->ThisUVE.flagMacTxPermit = 0x01UL;
            LocPtr->ThisBCE.duplex = 2;
        }

        // ETH_CFG Register -----------------------------------------------------
        word_ETH_CFG = 0x00000800UL;        // Init: CLK = 0x2
        // (20070808) IMPORTANT: REG_ETH_CFG:bit1(FD), 1:TX will halt running RX frame, 0:TX will not halt running RX frame.
        // If always set FD=0, no CRC error will occur. But throughput maybe need re-evaluate.
        // IMPORTANT: (20070809) NO_MANUAL_SET_DUPLEX : The real duplex is returned by "negotiation"
        if(LocPtr->ThisBCE.speed     == EMAC_SPEED_100) word_ETH_CFG |= 0x00000001UL;
        if(LocPtr->ThisBCE.duplex    == 2)              word_ETH_CFG |= 0x00000002UL;
        if(LocPtr->ThisBCE.cam       == 1)              word_ETH_CFG |= 0x00000200UL;
        if(LocPtr->ThisBCE.rcv_bcast == 0)              word_ETH_CFG |= 0x00000020UL;
        if(LocPtr->ThisBCE.rlf       == 1)              word_ETH_CFG |= 0x00000100UL;

        MHal_EMAC_Write_CFG(word_ETH_CFG);

#ifdef EMAC_XIU16_SW_PROTECT
        if ((word_ETH_CFG & EMAC_BR & 0x3FFFUL) == 0)
        {
            printk(KERN_DEBUG "%s: change CFG golden from 0x%x to 0x%x\n ", __FUNCTION__, LocPtr->ThisUVE.CFG_val_golden, word_ETH_CFG);
            LocPtr->ThisUVE.CFG_val_golden = word_ETH_CFG;
        }
#endif /* EMAC_XIU16_SW_PROTECT */

        // ETH_CTL Register -----------------------------------------------------

        if(LocPtr->ThisBCE.wes == 1)
        {
            word_ETH_CTL = MHal_EMAC_Read_CTL();
            word_ETH_CTL |= 0x00000080UL;
            MHal_EMAC_Write_CTL(word_ETH_CTL);
#ifdef EMAC_XIU16_SW_PROTECT
            if ((word_ETH_CTL & EMAC_LBL) == 0)
            {
                printk(KERN_DEBUG "%s: change CTL golden from 0x%x to 0x%x\n", __FUNCTION__, LocPtr->ThisUVE.CTL_val_golden, word_ETH_CTL);
                LocPtr->ThisUVE.CTL_val_golden = word_ETH_CTL;
            }
            else
            {
                printk(KERN_WARNING "%s: EMAC_LBL is pulled high, 0x%x\n", __FUNCTION__, word_ETH_CTL);
            }
#endif /* EMAC_XIU16_SW_PROTECT */
        }

        LocPtr->ThisUVE.flagPowerOn = 1;
        LocPtr->ThisUVE.initedEMAC  = 1;
    }

    MHal_EMAC_HW_init();
    return 0;
}


//-------------------------------------------------------------------------------------------------
// EMAC init Variable
//-------------------------------------------------------------------------------------------------
extern phys_addr_t memblock_start_of_DRAM(void);
extern phys_addr_t memblock_size_of_first_region(void);

static phys_addr_t MDev_EMAC_MemInit(struct net_device *dev)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
    phys_addr_t alloRAM_PA_BASE;
    phys_addr_t alloRAM_SIZE;
    phys_addr_t *alloRAM_VA_BASE;

    get_boot_mem_info(EMAC_MEM, &alloRAM_PA_BASE, &alloRAM_SIZE);

#if defined (CONFIG_ARM64)
    alloRAM_PA_BASE = memblock_start_of_DRAM() + memblock_size_of_first_region();
#endif

    alloRAM_VA_BASE = (phys_addr_t *)ioremap_wc(alloRAM_PA_BASE, alloRAM_SIZE); //map buncing buffer from PA to VA

    EMAC_DBG("alloRAM_VA_BASE = 0x%zx alloRAM_PA_BASE= 0x%zx  alloRAM_SIZE= 0x%zx\n", (size_t)alloRAM_VA_BASE, (size_t)alloRAM_PA_BASE, (size_t)alloRAM_SIZE);
    BUG_ON(!alloRAM_VA_BASE);

    memset((phys_addr_t *)alloRAM_VA_BASE,0x00UL,alloRAM_SIZE);

    LocPtr->RAM_VA_BASE       = ((phys_addr_t)alloRAM_VA_BASE + 0x3FFFUL) & ~0x3FFFUL;   // IMPORTANT: Let lowest 14 bits as zero.
    LocPtr->RAM_PA_BASE       = ((phys_addr_t)alloRAM_PA_BASE + 0x3FFFUL) & ~0x3FFFUL;   // IMPORTANT: Let lowest 14 bits as zero.
    LocPtr->RAM_VA_PA_OFFSET  = LocPtr->RAM_VA_BASE - LocPtr->RAM_PA_BASE;  // IMPORTANT_TRICK_20070512
    LocPtr->RX_DESC_BASE      = LocPtr->RAM_PA_BASE;
#ifndef RX_ZERO_COPY
#ifdef YMY_MAC_2BYTE_ALIGNMENT
    LocPtr->RX_BUFFER_BASE    = LocPtr->RAM_PA_BASE + RX_DESC_TABLE_SIZE + 0x2;
#else /* YMY_MAC_2BYTE_ALIGNMENT */
    LocPtr->RX_BUFFER_BASE    = LocPtr->RAM_PA_BASE + RX_DESC_TABLE_SIZE;
#endif /* YMY_MAC_2BYTE_ALIGNMENT */
#ifdef TX_DESC_MODE
    LocPtr->TX_LP_DESC_BASE   = ((LocPtr->RX_BUFFER_BASE + RX_BUFF_SIZE) + 0x3FFFUL) & ~0x3FFFUL;
#ifndef TX_ZERO_COPY
    LocPtr->TX_BUFFER_BASE    = LocPtr->TX_LP_DESC_BASE + TX_LOW_PRI_DESC_TABLE_SIZE;
#endif /* TX_ZERO_COPY */
#else /* TX_DESC_MODE */
#ifndef TX_ZERO_COPY
    LocPtr->TX_BUFFER_BASE    = LocPtr->RX_BUFFER_BASE + RX_BUFF_SIZE;
#endif /* TX_ZERO_COPY */
#endif /* TX_DESC_MODE */
#else /* RX_ZERO_COPY */
    LocPtr->rx_drop_base      = LocPtr->RAM_PA_BASE + RX_DESC_TABLE_SIZE;
#ifdef TX_DESC_MODE
    LocPtr->TX_LP_DESC_BASE   = ((LocPtr->rx_drop_base + RX_BUFF_ENTRY_SIZE) + 0x3FFFUL) & ~0x3FFFUL;
#ifndef TX_ZERO_COPY
    LocPtr->TX_BUFFER_BASE    = LocPtr->TX_LP_DESC_BASE + TX_LOW_PRI_DESC_TABLE_SIZE;
#endif /* TX_ZERO_COPY */
#else /* TX_DESC_MODE */
#ifndef TX_ZERO_COPY
    LocPtr->TX_BUFFER_BASE    = LocPtr->rx_drop_base + RX_BUFF_ENTRY_SIZE;
#endif /* TX_ZERO_COPY */
#endif /* TX_DESC_MODE */
#endif /* RX_ZERO_COPY */

    EMAC_DBG("RAM_VA_BASE       = 0x%zx\n", (size_t)LocPtr->RAM_VA_BASE);
    EMAC_DBG("RAM_PA_BASE       = 0x%zx\n", (size_t)LocPtr->RAM_PA_BASE);
    EMAC_DBG("RAM_VA_PA_OFFSET  = 0x%zx\n", (size_t)LocPtr->RAM_VA_PA_OFFSET);
    EMAC_DBG("RX_DESC_BASE      = 0x%zx\n", (size_t)LocPtr->RX_DESC_BASE);
#ifndef RX_ZERO_COPY
    EMAC_DBG("RX_BUFFER_BASE    = 0x%zx\n", (size_t)LocPtr->RX_BUFFER_BASE);
#endif
#ifdef TX_DESC_MODE
    EMAC_DBG("TX_LP_DESC_BASE   = 0x%zx\n", (size_t)LocPtr->TX_LP_DESC_BASE);
#endif
#ifndef TX_ZERO_COPY
    EMAC_DBG("TX_BUFFER_BASE    = 0x%zx\n", (size_t)LocPtr->TX_BUFFER_BASE);
#endif

#ifdef RX_DESC_MODE
    LocPtr->rx_desc_list = (struct rx_descriptor *)(LocPtr->RX_DESC_BASE + LocPtr->RAM_VA_PA_OFFSET);
#endif

#ifdef TX_DESC_MODE
    LocPtr->tx_desc_list = (struct tx_descriptor *)(LocPtr->TX_LP_DESC_BASE + LocPtr->RAM_VA_PA_OFFSET);
#endif

#ifndef RX_ZERO_COPY
    MDev_EMAC_RX_DESC_Init_memcpy(dev);
#endif

#ifdef YMY_MAC_MEMORY_PROTECT
    EMAC_DBG("MHal_EMAC_Write_Protect(%d, %d)\n", (size_t)alloRAM_PA_BASE - MIU0_BUS_BASE, alloRAM_SIZE);
    MHal_EMAC_Write_Protect(alloRAM_PA_BASE - MIU0_BUS_BASE, alloRAM_SIZE);
#endif

    memset(&LocPtr->ThisBCE,0x00UL,sizeof(BasicConfigEMAC));
    memset(&LocPtr->ThisUVE,0x00UL,sizeof(UtilityVarsEMAC));

    LocPtr->ThisBCE.wes         = 0;                    // 0:Disable, 1:Enable (WR_ENABLE_STATISTICS_REGS)
    LocPtr->ThisBCE.duplex      = 2;                    // 1:Half-duplex, 2:Full-duplex
    LocPtr->ThisBCE.cam         = 0;                    // 0:No CAM, 1:Yes
    LocPtr->ThisBCE.rlf         = 0;                    // 0:No, 1:Yes receive long frame(1522)
    LocPtr->ThisBCE.rcv_bcast   = 1;
    LocPtr->ThisBCE.speed       = EMAC_SPEED_100;
    LocPtr->ThisBCE.connected   = 0;
    return (phys_addr_t)alloRAM_VA_BASE;
}

//-------------------------------------------------------------------------------------------------
// Initialize the ethernet interface
// @return TRUE : Yes
// @return FALSE : FALSE
//-------------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static const struct net_device_ops mstar_lan_netdev_ops = {
        .ndo_open               = MDev_EMAC_open,
        .ndo_stop               = MDev_EMAC_close,
        .ndo_start_xmit         = MDev_EMAC_tx,
        .ndo_set_mac_address    = MDev_EMAC_set_mac_address,
        .ndo_set_rx_mode        = MDev_EMAC_set_rx_mode,
        .ndo_do_ioctl           = MDev_EMAC_ioctl,
        .ndo_get_stats          = MDev_EMAC_stats,
};

static int MDev_EMAC_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
#ifdef CONFIG_EMAC_EPHY_LPA_FORCE_SPEED
    u32 bmcr, bmsr, adv, lpa, neg = 0;
#endif /* CONFIG_EMAC_EPHY_LPA_FORCE_SPEED */

    mii_ethtool_gset (&LocPtr->mii, cmd);

#ifdef CONFIG_EMAC_EPHY_LPA_FORCE_SPEED
    if (cmd->autoneg == AUTONEG_ENABLE) {
        MHal_EMAC_read_phy(LocPtr->phyaddr, MII_BMSR, &bmsr);
        MHal_EMAC_read_phy(LocPtr->phyaddr, MII_BMSR, &bmsr);
        if (bmsr & BMSR_ANEGCOMPLETE) {
            /* For Link Parterner adopts force mode and EPHY used,
             * EPHY LPA reveals all zero value.
             * EPHY would be forced to Full-Duplex mode.
             */
            if (cmd->lp_advertising == 0) {
                MHal_EMAC_read_phy(LocPtr->phyaddr, MII_BMCR, &bmcr);
                if (bmcr & BMCR_SPEED100)
                    lpa = LPA_100FULL;
                else
                    lpa = LPA_10FULL;

                MHal_EMAC_read_phy(LocPtr->phyaddr, MII_ADVERTISE, &adv);
                neg = adv & lpa;

                if (neg & LPA_100FULL)
                {
                    ethtool_cmd_speed_set(cmd, SPEED_100);
                }
                else
                {
                    ethtool_cmd_speed_set(cmd, SPEED_10);
                }
                cmd->duplex = 0x01;
                LocPtr->mii.full_duplex = cmd->duplex;
            }
        }
    }
#endif /* CONFIG_EMAC_EPHY_LPA_FORCE_SPEED */

    return 0;
}

static int MDev_EMAC_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    mii_ethtool_sset (&LocPtr->mii, cmd);

    return 0;
}

static int MDev_EMAC_nway_reset(struct net_device *dev)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    mii_nway_restart (&LocPtr->mii);

    return 0;
}

static u32 MDev_EMAC_get_link(struct net_device *dev)
{
    u32 u32data;
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    u32data = mii_link_ok (&LocPtr->mii);

    return u32data;
}

static void MDev_EMAC_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
    strlcpy(info->driver, EMAC_DRVNAME, sizeof(info->driver));
    strlcpy(info->version, EMAC_DRV_VERSION, sizeof(info->version));
    return;
}

static u64 *mstar_emac_stats[EMAC_STATS_STRING_LEN]; //Read only, don't write

static void MDev_EMAC_ethtool_stats_init(struct net_device *dev)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    mstar_emac_stats[0] = (u64 *)&LocPtr->stats.rx_packets;
    mstar_emac_stats[1] = (u64 *)&LocPtr->stats.rx_bytes;
    mstar_emac_stats[2] = (u64 *)&LocPtr->stats.rx_errors;
    mstar_emac_stats[3] = (u64 *)&LocPtr->stats.rx_dropped;
    mstar_emac_stats[4] = (u64 *)&LocPtr->stats.rx_length_errors;
    mstar_emac_stats[5] = (u64 *)&LocPtr->stats.rx_over_errors;
    mstar_emac_stats[6] = (u64 *)&LocPtr->stats.rx_crc_errors;
    mstar_emac_stats[7] = (u64 *)&LocPtr->stats.rx_frame_errors;
    mstar_emac_stats[8] = (u64 *)&LocPtr->stats.rx_fifo_errors;
    mstar_emac_stats[9] = (u64 *)&LocPtr->stats.rx_missed_errors;
    mstar_emac_stats[10] = (u64 *)&LocPtr->stats.rx_compressed;
    mstar_emac_stats[11] = (u64 *)&LocPtr->stats.tx_packets;
    mstar_emac_stats[12] = (u64 *)&LocPtr->stats.tx_bytes;
    mstar_emac_stats[13] = (u64 *)&LocPtr->stats.tx_errors;
    mstar_emac_stats[14] = (u64 *)&LocPtr->stats.tx_dropped;
    mstar_emac_stats[15] = (u64 *)&LocPtr->stats.tx_aborted_errors;
    mstar_emac_stats[16] = (u64 *)&LocPtr->stats.tx_carrier_errors;
    mstar_emac_stats[17] = (u64 *)&LocPtr->stats.tx_fifo_errors;
    mstar_emac_stats[18] = (u64 *)&LocPtr->stats.tx_heartbeat_errors;
    mstar_emac_stats[19] = (u64 *)&LocPtr->stats.tx_window_errors;
    mstar_emac_stats[20] = (u64 *)&LocPtr->stats.tx_compressed;
    mstar_emac_stats[21] = (u64 *)&LocPtr->stats.multicast;
    mstar_emac_stats[22] = (u64 *)&LocPtr->stats.collisions;

}

static void MDev_EMAC_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
    char *p = (char *)data;
    unsigned int i;

    switch(stringset)
    {
        case ETH_SS_TEST:
            return;
        case ETH_SS_STATS:
            for (i = 0; i < EMAC_STATS_STRING_LEN; i++)
            {
                snprintf(p, ETH_GSTRING_LEN, mstar_emac_stat_string[i]);
                p += ETH_GSTRING_LEN;
            }
            return;
        case ETH_SS_PRIV_FLAGS:
#ifdef EMAC_10T_RANDOM_WAVEFORM
            for (i = 0; i < EMAC_PRIV_FLAGS_STRING_LEN; i++)
            {
                snprintf(p, ETH_GSTRING_LEN, mstar_emac_private_flag_string[i]);
                p += ETH_GSTRING_LEN;
            }
#endif
            return;
        default:
            return;
    }
}

static void MDev_EMAC_get_ethtool_stats(struct net_device *dev, struct ethtool_stats *e_state, u64 *data)
{
    unsigned int i;
    //EMAC_DBG("MDev_GMAC_get_ethtool_stats\n");

    MDev_EMAC_stats(dev);

    for(i = 0; i < EMAC_STATS_STRING_LEN; i++)
    {
        data[i] = *(u32 *)mstar_emac_stats[i];
    }
}

static int MDev_EMAC_sset_count(struct net_device *dev, int type)
{
    //EMAC_DBG("MDev_EMAC_sset_count\n");
    switch(type)
    {
        case ETH_SS_TEST:
            return EMAC_TEST_STRING_LEN;
        case ETH_SS_STATS:
            return EMAC_STATS_STRING_LEN;
        case ETH_SS_PRIV_FLAGS:
            return EMAC_PRIV_FLAGS_STRING_LEN;
        default:
            return -EOPNOTSUPP;
    }
}

#ifdef EMAC_GMAC_DESCRIPTOR
#define EMAC_REGS_LEN 0xC0
#else
#define EMAC_REGS_LEN 0x80
#endif

static int MDev_EMAC_get_regs_len(struct net_device *dev)
{
    return EMAC_REGS_LEN * sizeof(u32);
}

static u32 MDev_EMAC_get_regs_transform(u32 reg_data)
{
    return ((reg_data & 0x000000FF) << 8) |  ((reg_data & 0x0000FF00) >> 8) | ((reg_data & 0x00FF0000) << 8) | ((reg_data & 0xFF000000) >> 8);
}

static void MDev_EMAC_get_regs(struct net_device *dev, struct ethtool_regs *regs, void *data)
{
    u32 *data_32 = data;
    u32 register_index;
    u32 address_counter;

    register_index = 0;
    address_counter = 0;

    while(register_index < 0x40)
    {
        data_32[register_index] = MDev_EMAC_get_regs_transform(MHal_EMAC_ReadReg32(0x4UL * address_counter));
        register_index++;
        address_counter++;
    }

    address_counter = 0;

    while(register_index < 0x80)
    {
        data_32[register_index] = MDev_EMAC_get_regs_transform(MHal_EMAC_ReadReg32(0x4UL * address_counter + 0x100UL));
        register_index++;
        address_counter++;
    }

#ifdef EMAC_GMAC_DESCRIPTOR
    address_counter = 0;

    while(register_index < 0xC0)
    {
        data_32[register_index] = MDev_EMAC_get_regs_transform(MHal_EMAC_ReadReg32(0x4UL * address_counter + 0x400UL));
        register_index++;
        address_counter++;
    }
#endif

}

static int MDev_EMAC_get_coalesce(struct net_device *dev, struct ethtool_coalesce *e_coalesce)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
    u32 register_config;

    register_config = MHal_EMAC_Read_Network_config_register3();

    e_coalesce->rx_coalesce_usecs           = (register_config & (~EMAC_INT_DELAY_TIME_MASK)) >> EMAC_INT_DELAY_TIME_SHIFT;
    e_coalesce->rx_max_coalesced_frames     = (register_config & (~EMAC_INT_DELAY_NUMBER_MASK)) >> EMAC_INT_DELAY_NUMBER_SHIFT;
#ifdef RX_NAPI
    e_coalesce->rx_max_coalesced_frames_irq = LocPtr->napi_rx.weight;
#endif
    e_coalesce->use_adaptive_rx_coalesce    = true;

    return 0;
}

static int MDev_EMAC_set_coalesce(struct net_device *dev, struct ethtool_coalesce *e_coalesce)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
    u32 register_config;

    if(e_coalesce->rx_coalesce_usecs > 0xFF)
        return -EINVAL;
    if(e_coalesce->rx_max_coalesced_frames > 0xFF)
        return -EINVAL;
    if(e_coalesce->rx_max_coalesced_frames_irq > 0x40)
        return -EINVAL;

    register_config = MHal_EMAC_Read_Network_config_register3();
    register_config = register_config & EMAC_INT_DELAY_TIME_MASK & EMAC_INT_DELAY_NUMBER_MASK;
    register_config = register_config | e_coalesce->rx_coalesce_usecs << EMAC_INT_DELAY_TIME_SHIFT;
    register_config = register_config | e_coalesce->rx_max_coalesced_frames << EMAC_INT_DELAY_NUMBER_SHIFT;
    MHal_EMAC_Write_Network_config_register3(register_config);

#ifdef RX_NAPI
    LocPtr->napi_rx.weight = e_coalesce->rx_max_coalesced_frames_irq;
#endif

    return 0;
}

static void MDev_EMAC_get_ringparam(struct net_device *dev, struct ethtool_ringparam *e_ringparam)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    e_ringparam->rx_max_pending = RX_DESC_NUMBER;
    e_ringparam->tx_max_pending = TX_BUFF_ENTRY_NUMBER;
    e_ringparam->rx_pending = LocPtr->rx_ring_entry_number;
    e_ringparam->tx_pending = LocPtr->tx_ring_entry_number;

    return;
}

static int MDev_EMAC_set_ringparam(struct net_device *dev, struct ethtool_ringparam *e_ringparam)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    if(e_ringparam->rx_pending > RX_DESC_NUMBER)
        return -EINVAL;
    if(e_ringparam->tx_pending > TX_BUFF_ENTRY_NUMBER)
        return -EINVAL;

    LocPtr->rx_ring_entry_number = e_ringparam->rx_pending;
    LocPtr->tx_ring_entry_number = e_ringparam->tx_pending;

    return 0;
}

static u32 MDev_EMAC_get_msglevel(struct net_device *dev)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
    return LocPtr->msglvl;
}

static void MDev_EMAC_set_msglevel(struct net_device *dev, u32 data)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
    LocPtr->msglvl = data;
    return;
}

#ifdef EMAC_10T_RANDOM_WAVEFORM

static int MDev_EMAC_10T_RANDOM(void *arg)
{
    struct net_device *dev = arg;
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);
    int i;
    dma_addr_t skb_addr;
    char random_bytes[64];

    del_timer_sync(&LocPtr->Link_timer);

    MHal_EMAC_Force_10();

    while(LocPtr->private_flag & EMAC_PRIVATE_FLAG_10T_RANDOM)
    {
#ifdef EMAC_XIU16_SW_PROTECT
        if (NETDEV_TX_OK != MDev_EMAC_CheckTSR(dev))
#else
        if (NETDEV_TX_OK != MDev_EMAC_CheckTSR())
#endif /* EMAC_XIU16_SW_PROTECT */
            continue;

        printk("Random bytes: ");
        get_random_bytes(random_bytes, sizeof(char)*64);
        for(i = 0; i < 64 ;i++)
            printk("%x", random_bytes[i]);
        printk("\n");

        skb_addr = LocPtr->TX_BUFFER_BASE + LocPtr->RAM_VA_PA_OFFSET + TX_BUFF_ENTRY_SIZE * LocPtr->tx_index;
        LocPtr->tx_index ++;
        LocPtr->tx_index = LocPtr->tx_index % TX_BUFF_ENTRY_NUMBER;

        memcpy((void*)skb_addr, random_bytes, 64);

        MHal_EMAC_Write_TAR(skb_addr - LocPtr->RAM_VA_PA_OFFSET - MIU0_BUS_BASE);
        MHal_EMAC_Write_TCR(0x40);

    }

    if (!(timer_pending(&LocPtr->Link_timer)))
        add_timer(&LocPtr->Link_timer);

    return 0;
}

static int MDev_EMAC_set_private_flags(struct net_device *dev, u32 data)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    printk("set_private_flags data = %x\n", data);

    if(data & EMAC_PRIVATE_FLAG_10T_RANDOM)
    {
        if(!(LocPtr->private_flag & EMAC_PRIVATE_FLAG_10T_RANDOM))
        {
            LocPtr->private_flag |= EMAC_PRIVATE_FLAG_10T_RANDOM;

            LocPtr->ewave_task = kthread_create(MDev_EMAC_10T_RANDOM, dev, "10T_RANDOM");

            wake_up_process(LocPtr->ewave_task);
        }
    }
    else
    {
        if(LocPtr->private_flag & EMAC_PRIVATE_FLAG_10T_RANDOM)
        {
            LocPtr->private_flag &= ~EMAC_PRIVATE_FLAG_10T_RANDOM;

            kthread_stop(LocPtr->ewave_task);
            LocPtr->ewave_task = NULL;
        }
    }

    return 0;
}

static u32 MDev_EMAC_get_private_flags(struct net_device *dev)
{
    struct EMAC_private *LocPtr =(struct EMAC_private *) netdev_priv(dev);

    return LocPtr->private_flag;
}

#endif

static const struct ethtool_ops mstar_ethtool_ops = {
    .get_settings       = MDev_EMAC_get_settings,
    .set_settings       = MDev_EMAC_set_settings,
    .nway_reset         = MDev_EMAC_nway_reset,
    .get_link           = MDev_EMAC_get_link,
    .get_drvinfo        = MDev_EMAC_get_drvinfo,
    .get_sset_count     = MDev_EMAC_sset_count,
    .get_ethtool_stats  = MDev_EMAC_get_ethtool_stats,
    .get_strings        = MDev_EMAC_get_strings,
    .get_regs           = MDev_EMAC_get_regs,
    .get_regs_len       = MDev_EMAC_get_regs_len,
    .get_coalesce       = MDev_EMAC_get_coalesce,
    .set_coalesce       = MDev_EMAC_set_coalesce,
    .get_ringparam      = MDev_EMAC_get_ringparam,
    //.set_ringparam      = MDev_EMAC_set_ringparam,
    .get_msglevel       = MDev_EMAC_get_msglevel,
    .set_msglevel       = MDev_EMAC_set_msglevel,
#ifdef EMAC_10T_RANDOM_WAVEFORM
    .set_priv_flags     = MDev_EMAC_set_private_flags,
    .get_priv_flags     = MDev_EMAC_get_private_flags,
#endif
};
#endif
//-------------------------------------------------------------------------------------------------
// Restar the ethernet interface
// @return TRUE : Yes
// @return FALSE : FALSE
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_SwReset(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private *) netdev_priv(dev);
    u32 oldCFG, oldCTL;
    u32 retval;

    MDev_EMAC_get_mac_address (dev);
    oldCFG = MHal_EMAC_Read_CFG();
    oldCTL = MHal_EMAC_Read_CTL() & ~(EMAC_TE | EMAC_RE);

    netif_stop_queue (dev);

    retval = MHal_EMAC_Read_Network_config_register2();
    MHal_EMAC_Write_Network_config_register2(retval & 0x00000FFFUL);
    MHal_EMAC_Write_Network_config_register2(retval);

    MDev_EMAC_HW_Reg_init(dev);
    MHal_EMAC_Write_CFG(oldCFG);
    MHal_EMAC_Write_CTL(oldCTL);
    MHal_EMAC_enable_mdi ();
    MDev_EMAC_update_mac_address (dev); // Program ethernet address into MAC //
    MDev_EMAC_update_linkspeed (dev);
    MHal_EMAC_Write_IER(EMAC_INT_ENABLE);
    MDev_EMAC_start (dev);
    MDev_EMAC_set_rx_mode(dev);
    netif_start_queue (dev);
    LocPtr->ROVR_count = 0;
#ifdef HW_TX_CHECKSUM
    retval = MHal_EMAC_Read_Network_config_register3() | TX_CHECKSUM_ENABLE;
    MHal_EMAC_Write_Network_config_register3(retval);
#endif
    return 0;
}

//-------------------------------------------------------------------------------------------------
// Detect MAC and PHY and perform initialization
//-------------------------------------------------------------------------------------------------
#if defined (CONFIG_OF)
static struct of_device_id mstaremac_of_device_ids[] = {
         {.compatible = "mstar-emac"},
         {},
};
#endif
//-------------------------------------------------------------------------------------------------
// EMAC Timer set for Receive function
//-------------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static void MDev_EMAC_timer_callback(struct timer_list *t)
{
    unsigned long value = EMAC_LINK_TMR;
#else
static void MDev_EMAC_timer_callback(unsigned long value)
{
#endif
    int ret = 0;
    struct EMAC_private *LocPtr = (struct EMAC_private *) netdev_priv(emac_dev);
    static u32 bmsr, time_count = 0;
#ifdef EMAC_XIU16_SW_PROTECT
    u32 uRegValCTL;
    u32 uRegValCFG;
#endif /* EMAC_XIU16_SW_PROTECT */

#ifndef INT_JULIAN_D
    if (EMAC_RX_TMR == value)
    {
        MHal_EMAC_timer_callback(value);
        return;
    }
#endif

#ifdef EMAC_XIU16_SW_PROTECT
    uRegValCTL = MHal_EMAC_Read_CTL();
    uRegValCFG = MHal_EMAC_Read_CFG();
    if ((uRegValCTL | EMAC_MPE) != (LocPtr->ThisUVE.CTL_val_golden | EMAC_MPE) || uRegValCFG != LocPtr->ThisUVE.CFG_val_golden)
    {
        printk(KERN_DEBUG "%s: Wrong CTL = 0x%x, CTL golden = 0x%x\n ", __FUNCTION__, uRegValCTL, LocPtr->ThisUVE.CTL_val_golden);
        printk(KERN_DEBUG "%s: Wrong CFG = 0x%x, CFG golden = 0x%x\n ", __FUNCTION__, uRegValCFG, LocPtr->ThisUVE.CFG_val_golden);
        MHal_EMAC_Write_CFG(LocPtr->ThisUVE.CFG_val_golden);

        uRegValCTL &= ((~EMAC_TE) & (~EMAC_RE) & (~EMAC_LBL) & 0x1FFUL);
        MHal_EMAC_Write_CTL(uRegValCTL);
        MHal_EMAC_Write_TCR(0);
        uRegValCTL = MHal_EMAC_Read_CTL();
        uRegValCTL |= ((EMAC_TE | EMAC_RE) & 0x1FFUL);
        MHal_EMAC_Write_CTL(uRegValCTL);
    }
#endif /* EMAC_XIU16_SW_PROTECT */

#ifdef TX_ZERO_COPY
    MDev_EMAC_TX_Free_sk_buff(emac_dev,0);
#endif

    spin_lock_irq (&LocPtr->irq_lock);
    ret = MDev_EMAC_update_linkspeed(emac_dev);
    spin_unlock_irq (&LocPtr->irq_lock);
    if (0 == ret)
    {
        if (!LocPtr->ThisBCE.connected)
        {
            LocPtr->ThisBCE.connected = 1;
            netif_carrier_on(emac_dev);
        }

        // Link status is latched, so read twice to get current value //
        MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
        MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
        time_count = 0;
        spin_lock_irq (&LocPtr->irq_lock);
        LocPtr->phy_status_register = bmsr;
        spin_unlock_irq (&LocPtr->irq_lock);
        // Normally, time out sets 1 Sec.
        LocPtr->Link_timer.expires = jiffies + EMAC_CHECK_LINK_TIME;
    }
    else    //no link
    {
        if(LocPtr->ThisBCE.connected) {
            LocPtr->ThisBCE.connected = 0;
            netif_carrier_off(emac_dev);
        }
        // If disconnected is over 3 Sec, the real value of PHY's status register will report to application.
        if(time_count > CONFIG_DISCONNECT_DELAY_S*10) {
            // Link status is latched, so read twice to get current value //
            MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
            MHal_EMAC_read_phy (LocPtr->phyaddr, MII_BMSR, &bmsr);
            spin_lock_irq (&LocPtr->irq_lock);
            LocPtr->phy_status_register = bmsr;
            spin_unlock_irq (&LocPtr->irq_lock);
            // Report to kernel.
            netif_carrier_off(emac_dev);
            LocPtr->ThisBCE.connected = 0;
            // Normally, time out is set 1 Sec.
            LocPtr->Link_timer.expires = jiffies + EMAC_CHECK_LINK_TIME;
        }
        else if(time_count <= CONFIG_DISCONNECT_DELAY_S*10){
            time_count++;
            // Time out is set 100ms. Quickly checks next phy status.
            LocPtr->Link_timer.expires = jiffies + (EMAC_CHECK_LINK_TIME / 10);
        }
    }

    if (!(timer_pending(&LocPtr->Link_timer)))
        add_timer(&LocPtr->Link_timer);
}

#ifdef TX_ZERO_COPY
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static void MDev_EMAC_tx_free_timer_callback(struct timer_list *t)
{
    struct EMAC_private *LocPtr = form_timer(LocPtr, t, TX_free_timer);
#else
static void MDev_EMAC_tx_free_timer_callback(unsigned long value)
{
    struct EMAC_private *LocPtr = (struct EMAC_private *) netdev_priv(value);
#endif
    MDev_EMAC_TX_Free_sk_buff(value,0);

    LocPtr->Link_timer.expires = jiffies + (EMAC_CHECK_LINK_TIME / 1000000);
    if (!(timer_pending(&LocPtr->TX_free_timer)))
        add_timer(&LocPtr->TX_free_timer);
}
#endif

//-------------------------------------------------------------------------------------------------
// EMAC MACADDR Setup
//-------------------------------------------------------------------------------------------------

#define MACADDR_FORMAT "XX:XX:XX:XX:XX:XX"

static int __init macaddr_auto_config_setup(char *addrs)
{
    if (strlen(addrs) == strlen(MACADDR_FORMAT)
        && ':' == addrs[2]
        && ':' == addrs[5]
        && ':' == addrs[8]
        && ':' == addrs[11]
        && ':' == addrs[14]
       )
    {
        addrs[2]  = '\0';
        addrs[5]  = '\0';
        addrs[8]  = '\0';
        addrs[11] = '\0';
        addrs[14] = '\0';

        MY_MAC[0] = (u8)simple_strtoul(&(addrs[0]),  NULL, 16);
        MY_MAC[1] = (u8)simple_strtoul(&(addrs[3]),  NULL, 16);
        MY_MAC[2] = (u8)simple_strtoul(&(addrs[6]),  NULL, 16);
        MY_MAC[3] = (u8)simple_strtoul(&(addrs[9]),  NULL, 16);
        MY_MAC[4] = (u8)simple_strtoul(&(addrs[12]), NULL, 16);
        MY_MAC[5] = (u8)simple_strtoul(&(addrs[15]), NULL, 16);

        /* set back to ':' or the environment variable would be destoried */ // REVIEW: this coding style is dangerous
        addrs[2]  = ':';
        addrs[5]  = ':';
        addrs[8]  = ':';
        addrs[11] = ':';
        addrs[14] = ':';
    }

    return 1;
}

__setup("macaddr=", macaddr_auto_config_setup);

//-------------------------------------------------------------------------------------------------
// EMAC register reboot notifier
//-------------------------------------------------------------------------------------------------
static int _MDev_EMAC_reboot_handler(struct notifier_block* nb, unsigned long event, void *unused)
{
    struct EMAC_private *LocPtr;
    u8 u8RegVal;

    EMAC_DBG("[%s][%d] event=%lu\n", __func__, __LINE__, event);
    switch (event)
    {
        case SYS_POWER_OFF:
            if (!emac_dev) {
                EMAC_DBG("no emac dev\n");
                return NOTIFY_OK;
            }

            LocPtr = netdev_priv(emac_dev);

            if (LocPtr->ep_flag & EP_FLAG_OPEND) {
                /* write reg_emac_dummy1[10] to 0, if ifconfig up */
                u8RegVal = MHal_EMAC_ReadReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1));
                u8RegVal &= ~(0x04);
                MHal_EMAC_WritReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1), u8RegVal);
#ifdef EMAC_NEW_WOC
                mstar_emac_woc_config(emac_dev, true);
#endif /* EMAC_NEW_WOC */
            } else {
                /* write reg_emac_dummy1[10] to 1, if ifconfig down */
                u8RegVal = MHal_EMAC_ReadReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1));
                u8RegVal |= (0x04);
                MHal_EMAC_WritReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1), u8RegVal);
#ifdef EMAC_NEW_WOC
                mstar_emac_woc_config(emac_dev, false);
#endif /* EMAC_NEW_WOC */
            }
            break;

        default:
            break;
    }

    return NOTIFY_OK;
}

static struct notifier_block _MDev_EMAC_reboot_notifier = {
    .notifier_call = _MDev_EMAC_reboot_handler,
};
//-------------------------------------------------------------------------------------------------
// EMAC init module
//-------------------------------------------------------------------------------------------------
static int MDev_EMAC_init(struct platform_device *pdev)
{
    struct EMAC_private *LocPtr;
    struct device_node *np = NULL;

    if(emac_dev)
        return -1;

    emac_dev = alloc_etherdev(sizeof(*LocPtr));
    LocPtr = netdev_priv(emac_dev);

    if (emac_dev == NULL)
    {
        EMAC_DBG( KERN_ERR "No EMAC dev mem!\n" );
        return -ENOMEM;
    }

    if (LocPtr == NULL)
    {
        EMAC_DBG("LocPtr fail\n");
        return -ENOMEM;
    }

#ifdef RX_NAPI
    netif_napi_add(emac_dev, &LocPtr->napi_rx, MDev_EMAC_RX_napi_poll, RX_NAPI_WEIGHT);
#endif

#ifdef TX_NAPI
    netif_napi_add(emac_dev, &LocPtr->napi_tx, MDev_EMAC_TX_napi_poll, TX_NAPI_WEIGHT);
#endif

    LocPtr->ddev = &pdev->dev;
    LocPtr->dev                 = emac_dev;
    LocPtr->phy_status_register = 0x78c9UL;         // 0x78c9: link is down.
    LocPtr->PreLinkStatus       = 0;
    LocPtr->initstate           = 0;
    LocPtr->phyaddr             = 0;
    spin_lock_init (&LocPtr->irq_lock);
    spin_lock_init (&LocPtr->tx_lock);
    spin_lock_init (&LocPtr->rx_lock);
    ether_setup (emac_dev);
    emac_dev->base_addr     = (long) REG_ADDR_BASE;
    emac_dev->irq           = MAC_IRQ;
    emac_dev->netdev_ops    = &mstar_lan_netdev_ops;

    np = pdev->dev.of_node;
    if (!np) {
        EMAC_DBG("fail to find node\n");
        goto end;
    }

#ifdef CONFIG_EMAC_TR_PN_SWAP
    if (of_property_read_bool(np, "tr-pn-swap") == true) {
        EMAC_DBG("do tr pn swap setting\n");
        LocPtr->is_tr_pn_swap = true;
    } else {
        LocPtr->is_tr_pn_swap = false;
    }
#endif /* CONFIG_EMAC_TR_PN_SWAP */

    if(MDev_EMAC_MemInit(emac_dev) == 0)
    {
        EMAC_DBG("Memery init fail!!\n");
        goto end;
    }

    LocPtr->rx_ring_entry_number = RX_DESC_NUMBER;
    LocPtr->tx_ring_entry_number = TX_BUFF_ENTRY_NUMBER;

    if (MDev_EMAC_HW_Reg_init(emac_dev) < 0)
    {
        EMAC_DBG("Hardware Register init fail!!\n");
        goto end;
    }

    //Support for ethtool //
    LocPtr->mii.dev = emac_dev;
    LocPtr->mii.mdio_read = MDev_EMAC_mdio_read;
    LocPtr->mii.mdio_write = MDev_EMAC_mdio_write;
    LocPtr->mii.phy_id = LocPtr->phyaddr;

    emac_dev->ethtool_ops = &mstar_ethtool_ops;
    MDev_EMAC_ethtool_stats_init(emac_dev);

    //Install the interrupt handler //
    //Notes: Modify linux/kernel/irq/manage.c  /* interrupt.h */
    if (request_irq(emac_dev->irq, MDev_EMAC_interrupt, SA_INTERRUPT, emac_dev->name, emac_dev))
        goto end;

#if defined(CONFIG_MP_PLATFORM_GIC_SET_MULTIPLE_CPUS) && defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
    irq_set_affinity_hint(emac_dev->irq, cpu_online_mask);
    irq_set_affinity(emac_dev->irq, cpu_online_mask);
#endif

    //Determine current link speed //
    spin_lock_irq (&LocPtr->irq_lock);
    (void) MDev_EMAC_update_linkspeed (emac_dev);
    spin_unlock_irq (&LocPtr->irq_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
    timer_setup(&LocPtr->Link_timer, MDev_EMAC_timer_callback, 0);
#else
    init_timer(&LocPtr->Link_timer);
    LocPtr->Link_timer.data = EMAC_LINK_TMR;
    LocPtr->Link_timer.function = MDev_EMAC_timer_callback;
#endif
    LocPtr->Link_timer.expires = jiffies + EMAC_CHECK_LINK_TIME/20;

#ifdef TX_ZERO_COPY
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
    timer_setup(&LocPtr->TX_free_timer, MDev_EMAC_tx_free_timer_callback, 0);
#else
    init_timer(&LocPtr->TX_free_timer);
    LocPtr->TX_free_timer.data = emac_dev;
    LocPtr->TX_free_timer.function = MDev_EMAC_tx_free_timer_callback;
#endif
    LocPtr->TX_free_timer.expires = jiffies + (EMAC_CHECK_LINK_TIME / 1000000);
#endif

    register_reboot_notifier(&_MDev_EMAC_reboot_notifier);

    return register_netdev (emac_dev);

end:
    free_netdev(emac_dev);
    emac_dev = 0;
    LocPtr->initstate = ETHERNET_TEST_INIT_FAIL;
    EMAC_DBG( KERN_ERR "Init EMAC error!\n" );
    return -1;
}
//-------------------------------------------------------------------------------------------------
// EMAC exit module
//-------------------------------------------------------------------------------------------------
static void MDev_EMAC_exit(void)
{
    if (emac_dev)
    {
        unregister_netdev(emac_dev);
        free_netdev(emac_dev);
    }
}

static int mstar_emac_drv_suspend(struct platform_device *dev, pm_message_t state)
{
    struct net_device *netdev=(struct net_device*)dev->dev.platform_data;
    struct EMAC_private *LocPtr;
    u32 uRegVal;
    u8 u8RegVal;
    printk(KERN_INFO "mstar_emac_drv_suspend\n");
    if(!netdev)
    {
        return -1;
    }

    is_wol_woc_wake = false;

    LocPtr = (struct EMAC_private*) netdev_priv(netdev);
    LocPtr->ep_flag |= EP_FLAG_SUSPENDING;
    netif_stop_queue (netdev);

    disable_irq(netdev->irq);
    del_timer_sync(&LocPtr->Link_timer);

    //Disable Receiver and Transmitter //
    uRegVal = MHal_EMAC_Read_CTL();
    uRegVal &= ~(EMAC_TE | EMAC_RE);
    MHal_EMAC_Write_CTL(uRegVal);

#ifdef EMAC_XIU16_SW_PROTECT
    if ((uRegVal & EMAC_LBL) == 0)
    {
        printk(KERN_DEBUG "%s: change CTL golden from 0x%x to 0x%x\n", __FUNCTION__, LocPtr->ThisUVE.CTL_val_golden, uRegVal);
        LocPtr->ThisUVE.CTL_val_golden = uRegVal;
    }
    else
    {
        printk(KERN_WARNING "%s: EMAC_LBL is pulled high, 0x%x\n", __FUNCTION__, uRegVal);
    }
#endif /* EMAC_XIU16_SW_PROTECT */

    // Disable PHY interrupt //
    MHal_EMAC_disable_phyirq ();
#ifndef RX_DELAY_INTERRUPT
    //Disable MAC interrupts //
    uRegVal = EMAC_INT_RCOM | EMAC_INT_ENABLE;
    MHal_EMAC_Write_IDR(uRegVal);
#else
    MHal_EMAC_Write_IDR(EMAC_INT_ENABLE);
#endif
    //MHal_EMAC_Power_Off_Clk();

    if (LocPtr->ep_flag & EP_FLAG_OPEND)
    {
        MDev_EMAC_close(netdev);
        LocPtr->ep_flag |= EP_FLAG_SUSPENDING_OPEND;
        /* write reg_emac_dummy1[10] to 0, if ifconfig up */
        u8RegVal = MHal_EMAC_ReadReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1));
        u8RegVal &= ~(0x04);
        MHal_EMAC_WritReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1), u8RegVal);
#ifdef EMAC_NEW_WOC
        mstar_emac_woc_config(netdev, true);
#endif /* EMAC_NEW_WOC */
    }
    else
    {
        /* write reg_emac_dummy1[10] to 1, if ifconfig down */
        u8RegVal = MHal_EMAC_ReadReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1));
        u8RegVal |= (0x04);
        MHal_EMAC_WritReg8(REG_EMAC0_BANK, (REG_EMAC_JULIAN_0138 + 1), u8RegVal);
#ifdef EMAC_NEW_WOC
        mstar_emac_woc_config(netdev, false);
#endif /* EMAC_NEW_WOC */
    }

#ifdef CONFIG_EMAC_STR_SLOW_SPEED
    MHal_EMAC_AN_10T();
#endif /* CONFIG_EMAC_STR_SLOW_SPEED */

    return 0;
}

#ifdef EMAC_WOC
static void mstar_emac_woc_packet_get(struct net_device *dev)
{
    struct EMAC_private *LocPtr = (struct EMAC_private*)netdev_priv(dev);
    u8 wakeReason = 0;
    u8 wocSramIndex = 0;
    u8 wocWriteVal = 0;
    u8 wocPacketLength_L = 0;
    u8 wocPacketLength_H = 0;
    u16 wocPacketLength = 0;
    u8 wocPacket[MAX_WOC_PKT_BUFF_DATA_LEN + 1] = {0};
    u8 i = 0, packetIndex = 0;
    struct sk_buff *skb = NULL;

    /* get wake up reason */
    wakeReason = MHal_EMAC_ReadReg8(REG_PMSLEEP_BANK, REG_WKUP_REASON_OFFSET);
    if ((wakeReason & 0x0f) == VAL_WKUP_REASON_WOC) {
        EMAC_DBG("wake up by WOC, 0x%x\n", wakeReason);
        wocSramIndex = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_INDEX_OFFSET);
        /* three SRAM buffers ring index: 00->01->10->00 */
        if ((wocSramIndex & 0x03) == 0x01)
            wocSramIndex = 0x00;
        else if ((wocSramIndex & 0x03) == 0x02)
            wocSramIndex = 0x40;
        else if ((wocSramIndex & 0x03) == 0x00)
            wocSramIndex = 0x80;
        else
            goto woc_drop_packet;

        /* get packet length, include 4 bytes CRC */
        wocWriteVal = wocSramIndex | 0x3f;
        MHal_EMAC_WritReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_PTR_OFFSET, wocWriteVal);
        wocPacketLength_L = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_DATA_L_OFFSET);
        wocPacketLength_H = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_DATA_H_OFFSET);
        wocPacketLength = (wocPacketLength_H & 0x07) << 8;
        wocPacketLength |= wocPacketLength_L;
        EMAC_DBG("get packet length 0x%x from 0x%x\n", wocPacketLength, wocWriteVal);
        wocPacketLength -= 4;
        /* HW stores 126 bytes in SRAM only */
        if (wocPacketLength <= MAX_WOC_PKT_BUFF_DATA_LEN) {
            packetIndex = 0;
            /* read once for two bytes */
            for (i = 0; i <= (MAX_WOC_PKT_BUFF_DATA_LEN >> 1); i++) {
                /* get packet data */
                wocWriteVal = wocSramIndex | i;
                MHal_EMAC_WritReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_PTR_OFFSET, wocWriteVal);
                wocPacket[packetIndex] = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_DATA_L_OFFSET);
                packetIndex++;
                if (packetIndex >= wocPacketLength)
                    break;
                wocPacket[packetIndex] = MHal_EMAC_ReadReg8(REG_EPHY_ALBANY2, REG_WOC_PKT_BUFF_DATA_H_OFFSET);
                packetIndex++;
                if (packetIndex >= wocPacketLength)
                    break;
            }

            //MDrv_EMAC_DumpMem(wocPacket, wocPacketLength);
            /* send to kernel network stack */
            skb = alloc_skb(wocPacketLength + 6, GFP_ATOMIC);
            if (skb != NULL) {
                skb_reserve(skb, 2);
                memcpy(skb_put(skb, wocPacketLength), wocPacket, wocPacketLength);
                skb->protocol = eth_type_trans(skb, dev);
                dev->last_rx = jiffies;
                LocPtr->stats.rx_bytes += wocPacketLength;
                skb->ip_summed = CHECKSUM_NONE;
                netif_rx(skb);
            } else {
                EMAC_DBG("allocate skb fail\n");
                goto woc_drop_packet;
            }
        } else
            goto woc_drop_packet;
    }

    return;

woc_drop_packet:
    LocPtr->stats.rx_dropped += 1;
    EMAC_DBG("skip woc packet\n");
}
#endif /* EMAC_WOC */

static int mstar_emac_drv_resume(struct platform_device *dev)
{
    struct net_device *netdev = (struct net_device*)dev->dev.platform_data;
    struct EMAC_private *LocPtr = (struct EMAC_private *) netdev_priv(netdev);
    phys_addr_t alloRAM_PA_BASE;
    phys_addr_t alloRAM_SIZE;
    u32 retval;
    u8 wake_src;

    printk(KERN_INFO "mstar_emac_drv_resume\n");

#ifdef CONFIG_EMAC_STR_SLOW_SPEED
    MHal_EMAC_AN_100T();
#endif /* CONFIG_EMAC_STR_SLOW_SPEED */

    if(!netdev)
    {
        return -1;
    }

    wake_src = MDrv_PM_GetWakeupSource();
    /* check wakeup reason */
    if ((wake_src != PM_WAKEUPSRC_WOC) &&
        (wake_src != PM_WAKEUPSRC_WOL)) {
        is_wol_woc_wake = false;
    } else {
        printk(KERN_INFO "mstar_emac_drv_resume: wakeup by 0x%x\n", wake_src);
        is_wol_woc_wake = true;
        pm_wakeup_event(LocPtr->ddev, 50);
    }

#ifdef EMAC_WOC
    mstar_emac_woc_packet_get(netdev);
#endif /* EMAC_WOC */

#ifdef EMAC_NEW_WOC
    mstar_emac_woc_config(netdev, false);
#endif /* EMAC_NEW_WOC */

    LocPtr->ThisUVE.initedEMAC = 0;
    MDev_EMAC_HW_Reg_init(netdev);
    LocPtr->ep_flag &= ~EP_FLAG_SUSPENDING;

    enable_irq(netdev->irq);
    if(LocPtr->ep_flag & EP_FLAG_SUSPENDING_OPEND)
    {
        if(0>MDev_EMAC_open(netdev))
        {
            printk(KERN_WARNING "Driver Emac: open failed after resume\n");
        }
        LocPtr->ep_flag &= ~EP_FLAG_SUSPENDING_OPEND;
#ifdef EMAC_NEW_WOC
        mstar_emac_woc_auto_load_trigger(netdev);
#endif /* EMAC_NEW_WOC */
    }
    return 0;
}

static int mstar_emac_drv_probe(struct platform_device *pdev)
{
    int retval=0;

#ifdef CONFIG_EMAC_PHY_RESTART_AN
    EMAC_DBG("EMAC V3: without any phy restart anto-nego patch\n");
#endif /* CONFIG_EMAC_PHY_RESTART_AN */

    retval = MDev_EMAC_init(pdev);
    if(!retval)
    {
        pdev->dev.platform_data=emac_dev;
    }
    return retval;
}

static int mstar_emac_drv_remove(struct platform_device *pdev)
{
    MDev_EMAC_exit();
    pdev->dev.platform_data=NULL;
    return 0;
}

static struct platform_driver Mstar_emac_driver = {
    .probe      = mstar_emac_drv_probe,
    .remove     = mstar_emac_drv_remove,
    .suspend    = mstar_emac_drv_suspend,
    .resume     = mstar_emac_drv_resume,
    .driver = {
        .name   = "Mstar-emac",
#if defined(CONFIG_OF)
        .of_match_table = mstaremac_of_device_ids,
#endif
        .owner  = THIS_MODULE,
    }
};

static int __init mstar_emac_drv_init_module(void)
{
    emac_dev=NULL;
    return platform_driver_register(&Mstar_emac_driver);
}

static void __exit mstar_emac_drv_exit_module(void)
{
    platform_driver_unregister(&Mstar_emac_driver);
    emac_dev=NULL;
    return;
}


module_init(mstar_emac_drv_init_module);
module_exit(mstar_emac_drv_exit_module);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("EMAC Ethernet driver");
MODULE_LICENSE("GPL");
