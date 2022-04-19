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
/// @file   MDRV_NOE_ETHTOOL.c
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include "mdrv_noe.h"


//--------------------------------------------------------------------------------------------------
//  Local Variable
//--------------------------------------------------------------------------------------------------
static struct net_device *ethtool_dev = NULL;





unsigned char MDrv_NOE_ETHTOOL_Get_Phy_Address(void)
{
    struct END_DEVICE *ei_local;

    if (ethtool_dev != NULL) {
        ei_local = netdev_priv(ethtool_dev);
        if (!ei_local->netdev)
            return 0;
        return ei_local->mii_info.phy_id;
    }
    return 0;
}

u32 MDrv_NOE_ETHTOOL_Get_Link(struct net_device *dev)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);
    return mii_link_ok(&ei_local->mii_info);
}

int MDrv_NOE_ETHTOOL_Set_Settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);
    mii_ethtool_sset(&ei_local->mii_info, cmd);
    return 0;
}

int MDrv_NOE_ETHTOOL_Get_Settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);
    mii_ethtool_gset(&ei_local->mii_info, cmd);
    return 0;
}

static int _MDrv_NOE_ETHTOOL_MDIO_Read(struct net_device *dev, int phy_id, int location)
{
    unsigned int result;
    struct END_DEVICE *ei_local = netdev_priv(dev);

    MHal_NOE_Read_Mii_Mgr((unsigned int)ei_local->mii_info.phy_id, (unsigned int)location, &result);
    NOE_MSG_DUMP("\n%s mii.o query= phy_id:%d\n",dev->name, phy_id);
    NOE_MSG_DUMP("address:%d retval:%x\n", location, result);
    return (int)result;
}


static void _MDrv_NOE_ETHTOOL_MDIO_Write(struct net_device *dev, int phy_id, int location, int value)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);

    MHal_NOE_Write_Mii_Mgr((unsigned int)ei_local->mii_info.phy_id, (unsigned int)location, (unsigned int)value);
    NOE_MSG_DUMP("mii.o write= phy_id:%d\n", phy_id);
    NOE_MSG_DUMP("address:%d value:%x\n", location, value);
}



u32 MDrv_NOE_ETHTOOL_Virt_Get_Link(struct net_device *dev)
{
    struct PSEUDO_ADAPTER *pseudo = netdev_priv(dev);
    struct END_DEVICE *ei_local = netdev_priv(ethtool_dev);

    if (ei_local->features & FE_GE2_SUPPORT)
        return mii_link_ok(&pseudo->mii_info);
    else
        return 0;
}


int MDrv_NOE_ETHTOOL_Virt_Set_Settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct PSEUDO_ADAPTER *pseudo = netdev_priv(dev);
    struct END_DEVICE *ei_local = netdev_priv(ethtool_dev);

    if (ei_local->features & FE_GE2_SUPPORT)
        mii_ethtool_sset(&pseudo->mii_info, cmd);
    return 0;
}


int MDrv_NOE_ETHTOOL_Virt_Get_Settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct PSEUDO_ADAPTER *pseudo = netdev_priv(dev);
    struct END_DEVICE *ei_local = netdev_priv(ethtool_dev);

    if (ei_local->features & FE_GE2_SUPPORT)
        mii_ethtool_gset(&pseudo->mii_info, cmd);
    return 0;
}

static int _MDrv_NOE_ETHTOOL_MDIO_Virt_Read(struct net_device *dev, int phy_id, int location)
{
    unsigned int result;
    struct PSEUDO_ADAPTER *pseudo = netdev_priv(dev);
    struct END_DEVICE *ei_local = netdev_priv(ethtool_dev);

    if (ei_local->features & FE_GE2_SUPPORT) {
        MHal_NOE_Read_Mii_Mgr((unsigned int)pseudo->mii_info.phy_id, (unsigned int)location, &result);
        NOE_MSG_DUMP("%s mii.o query= phy_id:%d,\n", dev->name, phy_id);
        NOE_MSG_DUMP("address:%d retval:%d\n", location, result);
        return (int)result;
    }
    return 0;
}

static void _MDrv_NOE_ETHTOOL_MDIO_Virt_Write(struct net_device *dev, int phy_id, int location, int value)
{
    struct PSEUDO_ADAPTER *pseudo = netdev_priv(dev);
    struct END_DEVICE *ei_local = netdev_priv(ethtool_dev);

    if (ei_local->features & FE_GE2_SUPPORT) {
        MHal_NOE_Write_Mii_Mgr((unsigned int)pseudo->mii_info.phy_id, (unsigned int)location, (unsigned int)value);
    }

    NOE_MSG_DUMP("mii.o write= phy_id:%d\n", phy_id);
    NOE_MSG_DUMP("address:%d value:%d\n)", location, value);
}

void MDrv_NOE_ETHTOOL_Init(struct net_device *dev)
{
    struct END_DEVICE *ei_local = netdev_priv(dev);

    /* store global info */
    ethtool_dev = dev;
    /* init mii structure */
    ei_local->mii_info.dev = dev;
    ei_local->mii_info.mdio_read = _MDrv_NOE_ETHTOOL_MDIO_Read;
    ei_local->mii_info.mdio_write = _MDrv_NOE_ETHTOOL_MDIO_Write;
    ei_local->mii_info.phy_id_mask = 0x1f;
    ei_local->mii_info.reg_num_mask = 0x1f;
    ei_local->mii_info.supports_gmii = mii_check_gmii_support(&ei_local->mii_info);

    /* TODO:   phy_id: 0~4 */
    ei_local->mii_info.phy_id = CONFIG_MAC_TO_GIGAPHY_MODE_ADDR;
}

void MDrv_NOE_ETHTOOL_Virt_Init(struct net_device *dev)
{
    struct PSEUDO_ADAPTER *p_pseudo_ad = netdev_priv(dev);

    /* init mii structure */
    p_pseudo_ad->mii_info.dev = dev;
    p_pseudo_ad->mii_info.mdio_read = _MDrv_NOE_ETHTOOL_MDIO_Virt_Read;
    p_pseudo_ad->mii_info.mdio_write = _MDrv_NOE_ETHTOOL_MDIO_Virt_Write;
    p_pseudo_ad->mii_info.phy_id_mask = 0x1f;
    p_pseudo_ad->mii_info.reg_num_mask = 0x1f;
    p_pseudo_ad->mii_info.phy_id = CONFIG_MAC_TO_GIGAPHY_MODE_ADDR2;
    p_pseudo_ad->mii_info.supports_gmii = mii_check_gmii_support(&p_pseudo_ad->mii_info);

}


