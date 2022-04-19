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
/// @file   MDRV_NOE_IOCTL.c
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _MDRV_NOE_IOCTL_H_
#define _MDRV_NOE_IOCTL_H_


//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------

/* ioctl commands */
#define NOE_SW_IOCTL                  0x89F0
#define NOE_ESW_REG_READ                0x89F1
#define NOE_ESW_REG_WRITE               0x89F2
#define NOE_MII_READ                    0x89F3
#define NOE_MII_WRITE                   0x89F4
#define NOE_ESW_INGRESS_RATE            0x89F5
#define NOE_ESW_EGRESS_RATE         0x89F6
#define NOE_ESW_PHY_DUMP                0x89F7
#define NOE_QDMA_REG_READ               0x89F8
#define NOE_QDMA_REG_WRITE          0x89F9
#define NOE_QDMA_QUEUE_MAPPING        0x89FA
#define NOE_QDMA_READ_CPU_CLK         0x89FB
#define NOE_MII_READ_CL45             0x89FC
#define NOE_MII_WRITE_CL45            0x89FD
#define NOE_QDMA_SFQ_WEB_ENABLE       0x89FE
#define NOE_SET_LAN_IP              0x89FF

/* switch ioctl commands */
#define SW_IOCTL_SET_EGRESS_RATE        0x0000
#define SW_IOCTL_SET_INGRESS_RATE       0x0001
#define SW_IOCTL_SET_VLAN               0x0002
#define SW_IOCTL_DUMP_VLAN              0x0003
#define SW_IOCTL_DUMP_TABLE             0x0004
#define SW_IOCTL_ADD_L2_ADDR            0x0005
#define SW_IOCTL_DEL_L2_ADDR            0x0006
#define SW_IOCTL_ADD_MCAST_ADDR         0x0007
#define SW_IOCTL_DEL_MCAST_ADDR         0x0008
#define SW_IOCTL_DUMP_MIB               0x0009
#define SW_IOCTL_ENABLE_IGMPSNOOP       0x000A
#define SW_IOCTL_DISABLE_IGMPSNOOP      0x000B
#define SW_IOCTL_SET_PORT_TRUNKING      0x000C
#define SW_IOCTL_GET_PORT_TRUNKING      0x000D
#define SW_IOCTL_SET_PORT_MIRROR        0x000E
#define SW_IOCTL_GET_PHY_STATUS         0x000F
#define SW_IOCTL_READ_REG               0x0010
#define SW_IOCTL_WRITE_REG              0x0011
#define SW_IOCTL_QOS_EN                 0x0012
#define SW_IOCTL_QOS_SET_TABLE2TYPE     0x0013
#define SW_IOCTL_QOS_GET_TABLE2TYPE     0x0014
#define SW_IOCTL_QOS_SET_PORT2TABLE     0x0015
#define SW_IOCTL_QOS_GET_PORT2TABLE     0x0016
#define SW_IOCTL_QOS_SET_PORT2PRI       0x0017
#define SW_IOCTL_QOS_GET_PORT2PRI       0x0018
#define SW_IOCTL_QOS_SET_DSCP2PRI       0x0019
#define SW_IOCTL_QOS_GET_DSCP2PRI       0x001a
#define SW_IOCTL_QOS_SET_PRI2QUEUE      0x001b
#define SW_IOCTL_QOS_GET_PRI2QUEUE      0x001c
#define SW_IOCTL_QOS_SET_QUEUE_WEIGHT   0x001d
#define SW_IOCTL_QOS_GET_QUEUE_WEIGHT   0x001e
#define SW_IOCTL_SET_PHY_TEST_MODE      0x001f
#define SW_IOCTL_GET_PHY_REG            0x0020
#define SW_IOCTL_SET_PHY_REG            0x0021
#define SW_IOCTL_VLAN_TAG               0x0022



#define REG_ESW_WT_MAC_MFC              0x0010
#define REG_ESW_ISC                     0x0018
#define REG_ESW_WT_MAC_ATA1             0x0074
#define REG_ESW_WT_MAC_ATA2             0x0078
#define REG_ESW_WT_MAC_ATWD             0x007C
#define REG_ESW_WT_MAC_ATC              0x0080
#define REG_ESW_TABLE_TSRA1             0x0084
#define REG_ESW_TABLE_TSRA2             0x0088
#define REG_ESW_TABLE_ATRD              0x008C
#define REG_ESW_VLAN_VTCR               0x0090
#define REG_ESW_VLAN_VAWD1              0x0094
#define REG_ESW_VLAN_VAWD2              0x0098
#define REG_ESW_VLAN_ID_BASE            0x0100

#define REG_ESW_VLAN_MEMB_BASE          0x0070
#define REG_ESW_TABLE_SEARCH            0x0024
#define REG_ESW_TABLE_STATUS0           0x0028
#define REG_ESW_TABLE_STATUS1           0x002C
#define REG_ESW_TABLE_STATUS2           0x0030
#define REG_ESW_WT_MAC_AD0              0x0034
#define REG_ESW_WT_MAC_AD1              0x0038
#define REG_ESW_WT_MAC_AD2              0x003C

/*10/100 phy cal*/
#define NOE_VBG_IEXT_CALIBRATION        0x0040
#define NOE_TXG_R50_CALIBRATION     0x0041
#define NOE_TXG_OFFSET_CALIBRATION  0x0042
#define NOE_TXG_AMP_CALIBRATION     0x0043




#define REG_ESW_MAX                     0x00FC
#define REG_HQOS_MAX                    0x3FFF

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------

struct ra_mii_ioctl_data {
    unsigned int  phy_id;
    unsigned int  reg_num;
    unsigned int  val_in;
    unsigned int  val_out;
    unsigned int  port_num;
    unsigned int  dev_addr;
    unsigned int  reg_addr;
};


struct noe_reg {
    unsigned int off;
    unsigned int val;
};

#endif /* _MDRV_NOE_IOCTL_H_ */
