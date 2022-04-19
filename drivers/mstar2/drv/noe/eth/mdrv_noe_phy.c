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
/// @file   MDRV_NOE_PHY.c
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include "mdrv_noe.h"
#include "mdrv_noe_phy.h"

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
typedef void (*PFN_SET_CFG)(u32, struct st_drv_phy_config *);

typedef enum {
    E_NOE_PHY_ID_MARVELL = 0,
    E_NOE_PHY_ID_VTSS,
    E_NOE_PHY_ID_ATHEROS,
    E_NOE_PHY_ID_REALTEK,
    E_NOE_PHY_ID_MSTAR,
    /* add id below */

    /* add id above */
    E_NOE_PHY_ID_MAX,
    E_NOE_PHY_ID_INVALID = E_NOE_PHY_ID_MAX,
}EN_NOE_PHY_ID;

//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//  Local Functions
//--------------------------------------------------------------------------------------------------
static void _MDrv_NOE_PHY_Set_Vtss_Config(u32 phy_address, struct st_drv_phy_config *info);
static void _MDrv_NOE_PHY_Set_Marvell_Config(u32 phy_address, struct st_drv_phy_config *info);
static void _MDrv_NOE_PHY_Set_Atheros_Config(u32 phy_address, struct st_drv_phy_config *info);
static void _MDrv_NOE_PHY_Set_Realtek_Config(u32 phy_address, struct st_drv_phy_config *info);
static void _MDrv_NOE_PHY_Set_Mstar_Config(u32 phy_address, struct st_drv_phy_config *info);

//--------------------------------------------------------------------------------------------------
//  Local Variable
//--------------------------------------------------------------------------------------------------
static PFN_SET_CFG _pfn_set_config[E_NOE_PHY_ID_MAX] = {
    [E_NOE_PHY_ID_MARVELL] = _MDrv_NOE_PHY_Set_Marvell_Config,
    [E_NOE_PHY_ID_VTSS] = _MDrv_NOE_PHY_Set_Vtss_Config,
    [E_NOE_PHY_ID_ATHEROS] = _MDrv_NOE_PHY_Set_Atheros_Config,
    [E_NOE_PHY_ID_REALTEK] = _MDrv_NOE_PHY_Set_Realtek_Config,
    [E_NOE_PHY_ID_MSTAR] = _MDrv_NOE_PHY_Set_Mstar_Config,
};


static EN_NOE_PHY_ID _MDrv_NOE_PHY_Get_PHY_ID(u32 phy_address)
{
    u32 phy_id0 = 0, phy_id1 = 0;

    if (MHal_NOE_Read_Mii_Mgr(phy_address, MII_PHYSID1, &phy_id0) != E_NOE_RET_TRUE) {
        NOE_MSG_ERR("\n Read PhyID 1 is Fail!!  0x%x\n", phy_id0);
        phy_id0 = 0;
    }
    if (MHal_NOE_Read_Mii_Mgr(phy_address, MII_PHYSID2, &phy_id1) != E_NOE_RET_TRUE) {
        NOE_MSG_ERR("\n Read PhyID 1 is Fail!!  0x%x\n", phy_id1);
        phy_id1 = 0;
    }

    if ((phy_id0 == EV_MARVELL_PHY_ID0) && (phy_id1 == EV_MARVELL_PHY_ID1))
        return E_NOE_PHY_ID_MARVELL;

    if ((phy_id0 == EV_ATHEROS_PHY_ID0) && (phy_id1 == EV_ATHEROS_PHY_ID1))
        return E_NOE_PHY_ID_ATHEROS;

    if ((phy_id0 == EV_VTSS_PHY_ID0) && (phy_id1 == EV_VTSS_PHY_ID1))
        return E_NOE_PHY_ID_VTSS;

    if ((phy_id0 == EV_REALTEK_PHY_ID0) && ((phy_id1 & EV_REALTEK_PHY_ID1) == EV_REALTEK_PHY_ID1))
        return E_NOE_PHY_ID_REALTEK;

    if ((phy_id0 == EV_MSTAR_PHY_ID0) && ((phy_id1 & EV_MSTAR_PHY_ID1) == EV_MSTAR_PHY_ID1))
        return E_NOE_PHY_ID_MSTAR;



    return E_NOE_PHY_ID_INVALID;

}

static void _MDrv_NOE_PHY_Set_Vtss_Config(u32 phy_address, struct st_drv_phy_config *info)
{
    u32 reg_value;
    MHal_NOE_Write_Mii_Mgr(phy_address, 31, 1);
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_NCONFIG, &reg_value);
    NOE_MSG_DBG("Vitesse phy skew: %x --> ", reg_value);
    reg_value |= (0x3 << 12);
    reg_value &= ~(0x3 << 14);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_NCONFIG, reg_value);
    MHal_NOE_Write_Mii_Mgr(phy_address, 31, 0);
}

static void _MDrv_NOE_PHY_Set_Marvell_Config(u32 phy_address, struct st_drv_phy_config *info)
{
    u32 reg_value;
    if (phy_address == CONFIG_MAC_TO_GIGAPHY_MODE_ADDR2) {
        MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
        /* turn off 1000Base-T Advertisement
         * (9.9=1000Full, 9.8=1000Half)
         */
        reg_value &= ~(3 << 8);
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);

        MHal_NOE_Read_Mii_Mgr(phy_address, MII_NWAYTEST, &reg_value);
        /* Add delay to RX_CLK for RXD Outputs */
        reg_value |= 1 << 7;
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_NWAYTEST, reg_value);

        MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
        reg_value |= 1 << 15;   /* PHY Software Reset */
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);
    }
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
    /* turn off 1000Base-T Advertisement
     * (9.9=1000Full, 9.8=1000Half)
     */
    reg_value &= ~(3 << 8);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);

    /*10Mbps, debug */
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, 0x461);

    MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
    reg_value |= 1 << 9;    /* restart AN */
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);

}

static void _MDrv_NOE_PHY_Set_Atheros_Config(u32 phy_address, struct st_drv_phy_config *info)
{
    u32 reg_value;
    if (info->mii_force_mode == NOE_ENABLE) {
        MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
        reg_value &= ~(3<<8); //turn off 1000Base-T Advertisement  (9.9=1000Full, 9.8=1000Half)
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);

        /*10Mbps, debug*/
        reg_value = ADVERTISE_PAUSE_CAP|ADVERTISE_10FULL|ADVERTISE_10HALF|ADVERTISE_CSMA;
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, reg_value);

        MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
        reg_value |= BMCR_ANENABLE; //restart AN
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);
    }
    else {
        /* Set Auto Polling Mode */
        MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
        reg_value |= (3<<8);
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);

        reg_value = ADVERTISE_PAUSE_CAP|ADVERTISE_100FULL|ADVERTISE_100HALF|ADVERTISE_10FULL|ADVERTISE_10HALF|ADVERTISE_CSMA;
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, reg_value);

        MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
        reg_value |= (BMCR_ANENABLE | BMCR_ANRESTART) ;
        MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);
    }


}

static void _MDrv_NOE_PHY_Set_Realtek_Config(u32 phy_address, struct st_drv_phy_config *info)
{
    u32 reg_value;
    /* Set Speed */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_ADVERTISE, &reg_value);
    reg_value |= ADVERTISE_10FULL | ADVERTISE_10HALF;
    reg_value |= ADVERTISE_100HALF | ADVERTISE_100FULL;
    reg_value |= ADVERTISE_PAUSE_CAP;
    reg_value |= ADVERTISE_CSMA;
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, reg_value);

    MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
    reg_value |= (ADVERTISE_1000HALF | ADVERTISE_1000FULL);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);


    /* restart AN */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
    reg_value |= (BMCR_ANENABLE | BMCR_ANRESTART);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);


    /* Add delay to RX_CLK for RXD Outputs */
    MHal_NOE_Write_Mii_Mgr(phy_address, 0x1F, 0x0007);
    MHal_NOE_Write_Mii_Mgr(phy_address, 0x1E, 0x00A4);
    MHal_NOE_Write_Mii_Mgr(phy_address, 0x1C, 0xAD91);
    MHal_NOE_Write_Mii_Mgr(phy_address, 0x1F, 0x0000);

    MHal_NOE_Read_Mii_Mgr(phy_address, 25, &reg_value);
    MHal_NOE_Write_Mii_Mgr(phy_address, 25, 0x400UL);
    MHal_NOE_Read_Mii_Mgr(phy_address, 25, &reg_value);

    /* Disable pause ability */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_ADVERTISE, &reg_value);
    reg_value &= ~(ADVERTISE_PAUSE_CAP);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, reg_value);

    /* restart AN */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
    reg_value &= ~(BMCR_LOOPBACK);
    reg_value |= BMCR_ANENABLE | BMCR_ANRESTART;
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);
}

static void _MDrv_NOE_PHY_Set_Mstar_Config(u32 phy_address, struct st_drv_phy_config *info)
{
    /* Add delay to RX_CLK for RXD Outputs */

    u32 reg_value;
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_ADVERTISE, &reg_value);
    reg_value |= ADVERTISE_10FULL | ADVERTISE_10HALF;
    reg_value |= ADVERTISE_100HALF | ADVERTISE_100FULL;
    reg_value |= ADVERTISE_PAUSE_CAP;
    reg_value |= ADVERTISE_CSMA;
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, reg_value);

#if defined(CONFIG_GMAC_GPHY_TO_EPHY)
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
    reg_value &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);
#else
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);
    reg_value |= (ADVERTISE_1000HALF | ADVERTISE_1000FULL);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_CTRL1000, reg_value);
#endif

    /* restart AN */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
    reg_value |= (BMCR_ANENABLE | BMCR_ANRESTART);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);

    /* Disable pause ability */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_ADVERTISE, &reg_value);
    reg_value &= ~(ADVERTISE_PAUSE_CAP);
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_ADVERTISE, reg_value);

    /* restart AN */
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_BMCR, &reg_value);
    reg_value &= ~(BMCR_LOOPBACK);
    reg_value |= BMCR_ANENABLE | BMCR_ANRESTART;
    MHal_NOE_Write_Mii_Mgr(phy_address, MII_BMCR, reg_value);

    MHal_NOE_Read_Mii_Mgr(phy_address, MII_ADVERTISE, &reg_value);
    MHal_NOE_Read_Mii_Mgr(phy_address, MII_CTRL1000, &reg_value);

    MHal_NOE_Write_Mii_Mgr(phy_address, 0x00, 0x1000);


}

void MDrv_NOE_PHY_Set_Config(u32 phy_addr, struct st_drv_phy_config *info)
{
    EN_NOE_PHY_ID id;
    MHal_NOE_Init_Mii_Mgr(info->ge, phy_addr, info->mii_force_mode);

    id = _MDrv_NOE_PHY_Get_PHY_ID(phy_addr);
    NOE_MSG_DBG("[GE%d] phy id: %d, phy addr: %d\n", (info->ge +1), id, phy_addr);

    if (id == E_NOE_PHY_ID_INVALID) {
        NOE_MSG_ERR("Invalid phy id  in GE%d\n", (info->ge +1));
        return;
    }
    _pfn_set_config[id](phy_addr, info);
}




