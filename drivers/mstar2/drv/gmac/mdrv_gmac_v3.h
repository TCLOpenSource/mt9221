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
//
// * Copyright (c) 2006 - 2007 Mstar Semiconductor, Inc.
// This program is free software.
// You can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation;
// either version 2 of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   GMAC.h
/// @author MStar Semiconductor Inc.
/// @brief  GMAC Driver Interface
///////////////////////////////////////////////////////////////////////////////////////////////////


// -----------------------------------------------------------------------------
// Linux GMAC.h define start
// -----------------------------------------------------------------------------

#ifndef __DRV_GMAC_H_
#define __DRV_GMAC_H_

#define GMAC_DBG(fmt, args...)              {printk("Mstar_gmac: "fmt, ##args);}
#define GMAC_INFO                           {printk("Line:%u\n", __LINE__);}
#define GMAC_DRVNAME                        "mstar gmac"
#define GMAC_DRV_VERSION                    "3.0.0"
#define GMAC_TEST_STRING_LEN                0
//-------------------------------------------------------------------------------------------------
//  Define Enable or Compiler Switches
//-------------------------------------------------------------------------------------------------
#define GMAC_MTU                            (1518)
#define GMAC_TX_MAX_LEN                     (1580) //hw limit
#define GMAC_RX_MAX_LEN                     (1522) //hw limit
//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------
#define GMAC_EP_FLAG_OPEND                  0X00000001UL
#define GMAC_EP_FLAG_SUSPENDING             0X00000002UL
#define GMAC_EP_FLAG_SUSPENDING_OPEND       0X00000004UL
#define GMAC_EP_FLAG_ROVR_RESET             0X00000008UL

#define GMAC_ETHERNET_TEST_NO_LINK          0x00000000UL
#define GMAC_ETHERNET_TEST_AUTO_NEGOTIATION 0x00000001UL
#define GMAC_ETHERNET_TEST_LINK_SUCCESS     0x00000002UL
#define GMAC_ETHERNET_TEST_RESET_STATE      0x00000003UL
#define GMAC_ETHERNET_TEST_SPEED_100M       0x00000004UL
#define GMAC_ETHERNET_TEST_DUPLEX_FULL      0x00000008UL
#define GMAC_ETHERNET_TEST_INIT_FAIL        0x00000010UL

#define GMAC_RX_TMR                         (0)
#define GMAC_LINK_TMR                       (1)

#define GMAC_CHECK_LINK_TIME                msecs_to_jiffies(1000)
#define GMAC_CHECK_CNT                      (500000)
#define GMAC_RTL_8210                       (0x1CUL)
//--------------------------------------------------------------------------------------------------
//  Global variable
//--------------------------------------------------------------------------------------------------
u8 GMAC_MY_MAC[6] = { 0x00, 0x55, 0x66, 0x00, 0x00, 0x01 };
//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
#define GMAC_STATS_STRING_LEN 23

static char mstar_gmac_stat_string[GMAC_STATS_STRING_LEN][ETH_GSTRING_LEN]=
{
    {"rx_packets"},
    {"rx_bytes"},
    {"rx_errors"},
    {"rx_dropped"},
    {"rx_length_errors"},
    {"rx_over_errors"},
    {"rx_crc_errors"},
    {"rx_frame_errors"},
    {"rx_fifo_errors"},
    {"rx_missed_errors"},
    {"rx_compressed"},
    {"tx_packets"},
    {"tx_bytes"},
    {"tx_errors"},
    {"tx_dropped"},
    {"tx_aborted_errors"},
    {"tx_carrier_errors"},
    {"tx_fifo_errors"},
    {"tx_heartbeat_errors"},
    {"tx_window_errors"},
    {"tx_compressed"},
    {"multicast"},
    {"collisions"}
};

#define GMAC_PRIV_FLAGS_STRING_LEN          0
#ifdef TX_DESC_MODE
struct tx_descriptor
{
  u32   addr;
  u32   low_tag;
  u32   reserve0;
  u32   reserve1;
};
#endif

#ifdef TX_SOFTWARE_QUEUE
struct tx_ring
{
    u8 used;
    struct sk_buff *skb;        /* holds skb until xmit interrupt completes */
    dma_addr_t skb_physaddr;    /* phys addr from pci_map_single */
};
#endif

#ifdef RX_DESC_MODE
struct rx_descriptor
{
  u32   addr;
  u32   low_tag;
  u32   high_tag;
  u32   reserve;
};
#endif

struct _BasicConfigGMAC
{
    u8 connected;           // 0:No, 1:Yes    <== (20070515) Wait for Julian's reply
    u8 speed;               // 10:10Mbps, 100:100Mbps
    // ETH_CTL Register:
    u8 wes;                 // 0:Disable, 1:Enable (WR_ENABLE_STATISTICS_REGS)
    // ETH_CFG Register:
    u8 duplex;              // 1:Half-duplex, 2:Full-duplex
    u8 cam;                 // 0:No CAM, 1:Yes
    u8 rcv_bcast;           // 0:No, 1:Yes
    u8 rlf;                 // 0:No, 1:Yes receive long frame(1522)
    // MAC Address:
    u8 sa1[6];              // Specific Addr 1 (MAC Address)
    u8 sa2[6];              // Specific Addr 2
    u8 sa3[6];              // Specific Addr 3
    u8 sa4[6];              // Specific Addr 4
};
typedef struct _BasicConfigGMAC BasicConfigGMAC;

struct _UtilityVarsGMAC
{
    u32 cntChkINTCounter;
    u32 readIdxRBQP;        // Reset = 0x00000000
    u32 rxOneFrameAddr;     // Reset = 0x00000000 (Store the Addr of "ReadONE_RX_Frame")
    // Statistics Counters : (accumulated)
    u32 cntREG_ETH_FRA;
    u32 cntREG_ETH_SCOL;
    u32 cntREG_ETH_MCOL;
    u32 cntREG_ETH_OK;
    u32 cntREG_ETH_SEQE;
    u32 cntREG_ETH_ALE;
    u32 cntREG_ETH_DTE;
    u32 cntREG_ETH_LCOL;
    u32 cntREG_ETH_ECOL;
    u32 cntREG_ETH_TUE;
    u32 cntREG_ETH_CSE;
    u32 cntREG_ETH_RE;
    u32 cntREG_ETH_ROVR;
    u32 cntREG_ETH_SE;
    u32 cntREG_ETH_ELR;
    u32 cntREG_ETH_RJB;
    u32 cntREG_ETH_USF;
    u32 cntREG_ETH_SQEE;
    // Interrupt Counter :
    u32 cntHRESP;           // Reset = 0x0000
    u32 cntROVR;            // Reset = 0x0000
    u32 cntLINK;            // Reset = 0x0000
    u32 cntTIDLE;           // Reset = 0x0000
    u32 cntTCOM;            // Reset = 0x0000
    u32 cntTBRE;            // Reset = 0x0000
    u32 cntRTRY;            // Reset = 0x0000
    u32 cntTUND;            // Reset = 0x0000
    u32 cntTOVR;            // Reset = 0x0000
    u32 cntRBNA;            // Reset = 0x0000
    u32 cntRCOM;            // Reset = 0x0000
    u32 cntDONE;            // Reset = 0x0000
    // Flags:
    u8  flagMacTxPermit;    // 0:No,1:Permitted.  Initialize as "permitted"
    u8  flagISR_INT_RCOM;
    u8  flagISR_INT_RBNA;
    u8  flagISR_INT_DONE;
    u8  flagPowerOn;        // 0:Poweroff, 1:Poweron
    u8  initedGMAC;         // 0:Not initialized, 1:Initialized.
    u8  flagRBNA;
    // Misc Counter:
    u32 cntRxFrames;        // Reset = 0x00000000 (Counter of RX frames,no matter it's me or not)
    u32 cntReadONE_RX;      // Counter for ReadONE_RX_Frame
    u32 cntCase20070806;
    u32 cntChkToTransmit;
    // Misc Variables:
    u32 mainThreadTasks;    // (20071029_CHARLES) b0=Poweroff,b1=Poweron
#ifdef GMAC_XIU16_SW_PROTECT
    // Link Speed Updated
    u8 link_sp_updated;
    // Register Golden Value:
    u32 CFG_val_golden;
    u32 CTL_val_golden;
    // TX Error Count:
    u32 tsrErrCnt;
#endif /* GMAC_XIU16_SW_PROTECT */
};
typedef struct _UtilityVarsGMAC UtilityVarsGMAC;

struct GMAC_private
{
    struct net_device *dev;
    struct net_device_stats stats;
    struct mii_if_info mii;             /* ethtool support */
    struct timer_list Link_timer;
#ifdef TX_ZERO_COPY
    struct timer_list TX_free_timer;
#endif
    u32 padmux_type;
    u32 hardware_type;
    u32 initstate;
    u32 msglvl;
    u32 private_flag;
    struct task_struct *ewave_task;
    BasicConfigGMAC ThisBCE;
    UtilityVarsGMAC ThisUVE;
    /* Memory */
    phys_addr_t     RAM_VA_BASE;
    phys_addr_t     RAM_PA_BASE;
    phys_addr_t     RAM_VA_PA_OFFSET;
    phys_addr_t     RX_DESC_BASE;
#ifndef RX_ZERO_COPY
    phys_addr_t     RX_BUFFER_BASE;
#else
    phys_addr_t     rx_drop_base;
#endif
#ifdef TX_DESC_MODE
    phys_addr_t     TX_LP_DESC_BASE;
    phys_addr_t     TX_HP_DESC_BASE;
#endif
#ifndef TX_ZERO_COPY
    phys_addr_t     TX_BUFFER_BASE;
#endif
    /* PHY */
    u8  phyaddr;
    u32 phy_status_register;
    u32 phy_type;             /* type of PHY (PHY_ID) */
    spinlock_t irq_lock;                /* lock for MDI interface */
    spinlock_t tx_lock;                 /* lock for MDI interface */
    spinlock_t rx_lock;
    short phy_media;                    /* media interface type */
    /* Transmit */
    u32 tx_index;
    u32 tx_ring_entry_number;
#ifdef TX_DESC_MODE
    struct tx_descriptor *tx_desc_list;
    struct sk_buff *tx_desc_sk_buff_list[TX_LOW_PRI_DESC_NUMBER];
    u32 tx_desc_write_index;
    u32 tx_desc_read_index;
    u32 tx_desc_queued_number;
    u32 tx_desc_count;
    u32 tx_desc_full_count;
    u32 tx_desc_start_free;
#endif
#ifdef TX_SOFTWARE_QUEUE
    struct tx_ring tx_swq[TX_SW_QUEUE_SIZE];
    unsigned int tx_rdidx;              /* TX_SW_QUEUE read to hw index */
    unsigned int tx_wridx;              /* TX_SW_QUEUE write index */
    unsigned int tx_clidx;              /* TX_SW_QUEUE clear index */

    unsigned int tx_rdwrp;              /* TX_SW_QUEUE read to hw index wrap*/
    unsigned int tx_wrwrp;              /* TX_SW_QUEUE write index wrap*/
    unsigned int tx_clwrp;              /* TX_SW_QUEUE clear index wrap */
    unsigned int tx_swq_full_cnt;       /* TX_SW_QUEUE full stopped count*/

    unsigned int irqcnt;
    unsigned int tx_irqcnt;
#endif
#ifdef TX_NAPI
    struct napi_struct napi_tx;
#endif
    /* Receive */
    u32 ROVR_count;
    u32 full_budge_count;
    u32 polling_count;
    u32 max_polling;
    u32 rx_ring_entry_number;
#ifdef RX_DESC_MODE
    struct rx_descriptor *rx_desc_list;
    u32 rx_desc_read_index;
#ifdef RX_ZERO_COPY
    struct sk_buff *rx_desc_sk_buff_list[RX_DESC_NUMBER];
    u32 rx_desc_free_index;
    u32 rx_desc_free_number;
#endif
#endif
#ifdef RX_NAPI
    struct napi_struct napi_rx;
#endif
    /* suspend/resume */
    u32 ep_flag;
};

#endif
// -----------------------------------------------------------------------------
// Linux GMAC.h End
// -----------------------------------------------------------------------------
