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
/// @file   MDRV_NOE.c
/// @brief  NOE Driver
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

#include "mdrv_mstypes.h"
#include "chip_int.h"
#include "mdrv_noe.h"
#include "mdrv_noe_phy.h"

//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
//  Global Variable and Functions
//--------------------------------------------------------------------------------------------------



static void _MDrv_NOE_MAC_Set_Link(struct END_DEVICE *ei_local)
{
    MS_BOOL bAnSupported = MHal_NOE_Support_Auto_Polling();
    struct st_drv_phy_config phy_cfg[E_NOE_GE_MAC_MAX];
    phy_cfg[E_NOE_GE_MAC1].ge = E_NOE_GE_MAC1;
    phy_cfg[E_NOE_GE_MAC2].ge = E_NOE_GE_MAC2;

    if(ei_local->architecture & GE1_RGMII_AN) {
        phy_cfg[E_NOE_GE_MAC1].mii_force_mode = (bAnSupported == TRUE)?NOE_DISABLE:NOE_ENABLE;
        if (bAnSupported == FALSE) {
            phy_cfg[E_NOE_GE_MAC1].speed = E_NOE_SPEED_1000;
            phy_cfg[E_NOE_GE_MAC1].duplex = E_NOE_DUPLEX_FULL;
        }
        if (phy_cfg[E_NOE_GE_MAC1].mii_force_mode == NOE_DISABLE) {
            phy_cfg[E_NOE_GE_MAC1].speed = E_NOE_SPEED_INVALID;
            phy_cfg[E_NOE_GE_MAC1].duplex = E_NOE_DUPLEX_INVALID;
        }
        #ifdef CONFIG_MSTAR_ARM_BD_FPGA
        phy_cfg[E_NOE_GE_MAC1].mii_force_mode = NOE_ENABLE;
        phy_cfg[E_NOE_GE_MAC1].speed = FPGA_NOE_PHY_SPEED;
        phy_cfg[E_NOE_GE_MAC1].duplex = FPGA_NOE_PHY_DUPLEX;
        #endif /*CONFIG_MSTAR_ARM_BD_FPGA*/
    }

    if (ei_local->features & FE_GE2_SUPPORT) {
        if (ei_local->architecture & GE2_RGMII_AN) {
            phy_cfg[E_NOE_GE_MAC2].mii_force_mode = (bAnSupported == TRUE)?NOE_DISABLE:NOE_ENABLE;
            if (bAnSupported == FALSE) {
                phy_cfg[E_NOE_GE_MAC2].speed = E_NOE_SPEED_1000;
                phy_cfg[E_NOE_GE_MAC2].duplex = E_NOE_DUPLEX_FULL;
            }
            if (phy_cfg[E_NOE_GE_MAC2].mii_force_mode == NOE_DISABLE) {
                phy_cfg[E_NOE_GE_MAC2].speed = E_NOE_SPEED_INVALID;
                phy_cfg[E_NOE_GE_MAC2].duplex = E_NOE_DUPLEX_INVALID;
            }
            #ifdef CONFIG_MSTAR_ARM_BD_FPGA
            phy_cfg[E_NOE_GE_MAC2].mii_force_mode = NOE_ENABLE;
            phy_cfg[E_NOE_GE_MAC2].speed = FPGA_NOE_PHY_SPEED;
            phy_cfg[E_NOE_GE_MAC2].duplex = FPGA_NOE_PHY_DUPLEX;
            #endif /*CONFIG_MSTAR_ARM_BD_FPGA*/
        }
    }

    if (ei_local->features & FE_GE2_SUPPORT) {
        if (MDrv_NOE_SYS_IS_AN_DEINITED(ei_local->sys_inited)) {
            MDrv_NOE_PHY_Set_Config(CONFIG_MAC_TO_GIGAPHY_MODE_ADDR2, &phy_cfg[E_NOE_GE_MAC2]);
            MHal_NOE_Force_Link_Mode(E_NOE_GE_MAC2, phy_cfg[E_NOE_GE_MAC2].speed, phy_cfg[E_NOE_GE_MAC2].duplex);
        }
    }

    if (ei_local->architecture & GE1_RGMII_AN) {
        if (MDrv_NOE_SYS_IS_AN_DEINITED(ei_local->sys_inited)) {
            MDrv_NOE_PHY_Set_Config(CONFIG_MAC_TO_GIGAPHY_MODE_ADDR, &phy_cfg[E_NOE_GE_MAC1]);
            MHal_NOE_Force_Link_Mode(E_NOE_GE_MAC1, phy_cfg[E_NOE_GE_MAC1].speed, phy_cfg[E_NOE_GE_MAC1].duplex);
        }
    }

    if (bAnSupported == TRUE) {
        if ((phy_cfg[E_NOE_GE_MAC1].mii_force_mode != NOE_ENABLE) || (phy_cfg[E_NOE_GE_MAC2].mii_force_mode != NOE_ENABLE))
            MHal_NOE_Set_Auto_Polling(E_NOE_SEL_ENABLE);
    }
    else
        MHal_NOE_Set_Auto_Polling(E_NOE_SEL_DISABLE);

    MDrv_NOE_SYS_SET_BIT(ei_local->sys_inited, NOE_SYS_INITED_AN);
}


void MDrv_NOE_MAC_Init(struct net_device *dev)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);
    _MDrv_NOE_MAC_Set_Link(ei_local);
}


void MDrv_NOE_MAC_Detect_Link_Status(struct net_device *dev)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);
    struct noe_mac_link_status status;


    if (MHal_NOE_Get_Link_Intr_Status()) {
        MHal_NOE_Disable_Link_Intr();
        MHal_NOE_MAC_Get_Link_Status(E_NOE_GE_MAC1, &status);

        if (!netif_carrier_ok(dev)) {
            if (status.link_up == TRUE) {
                MHal_NOE_AN_Speed_Judgment();
                netif_carrier_on(dev);
            }
        }
        else {
            if (status.link_up == FALSE) {
                netif_carrier_off(dev);
            }
            else {
                MHal_NOE_AN_Speed_Judgment();
            }
        }

        if (ei_local->features & FE_GE2_SUPPORT) {
            if (ei_local->pseudo_dev != NULL) {
                MHal_NOE_MAC_Get_Link_Status(E_NOE_GE_MAC2, &status);
                if (!netif_carrier_ok(ei_local->pseudo_dev)) {
                    if (status.link_up == TRUE)
                        netif_carrier_on(ei_local->pseudo_dev);
                }
                else {
                    if (status.link_up == FALSE)
                        netif_carrier_off(ei_local->pseudo_dev);
                }
            }
        }
        MHal_NOE_Enable_Link_Intr();
    }

}



