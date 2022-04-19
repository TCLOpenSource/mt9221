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
/// @file   EMAC.h
/// @author MStar Semiconductor Inc.
/// @brief  EMAC Driver Interface
///////////////////////////////////////////////////////////////////////////////////////////////////


// -----------------------------------------------------------------------------
// Linux EMAC.h define start
// -----------------------------------------------------------------------------

#ifndef __DRV_EMAC_H_
#define __DRV_EMAC_H_

#define EMAC_DBG(fmt, args...)              {printk("Mstar_emac: "fmt, ##args);}
#define EMAC_ERR(fmt, args...)              {printk(KERN_ERR "Mstar_emac: "fmt, ##args);}
#define EMAC_NOTI(fmt, args...)             {printk(KERN_NOTICE "Mstar_emac: "fmt, ##args);}
#define EMAC_INFO                           {printk("Line:%u\n", __LINE__);}
#define EMAC_DRVNAME                        "mstar emac"
#define EMAC_DRV_VERSION                    "3.0.0"
#define EMAC_TEST_STRING_LEN                0
//-------------------------------------------------------------------------------------------------
//  Define Enable or Compiler Switches
//-------------------------------------------------------------------------------------------------
#define EMAC_MTU                            (1518)
#define EMAC_TX_MAX_LEN                     (1580) //hw limit
#define EMAC_RX_MAX_LEN                     (1522) //hw limit
//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------
#define EP_FLAG_OPEND                       0X00000001UL
#define EP_FLAG_SUSPENDING                  0X00000002UL
#define EP_FLAG_SUSPENDING_OPEND            0X00000004UL

#define ETHERNET_TEST_NO_LINK               0x00000000UL
#define ETHERNET_TEST_AUTO_NEGOTIATION      0x00000001UL
#define ETHERNET_TEST_LINK_SUCCESS          0x00000002UL
#define ETHERNET_TEST_RESET_STATE           0x00000003UL
#define ETHERNET_TEST_SPEED_100M            0x00000004UL
#define ETHERNET_TEST_DUPLEX_FULL           0x00000008UL
#define ETHERNET_TEST_INIT_FAIL             0x00000010UL

#define EMAC_RX_TMR                         (0)
#define EMAC_LINK_TMR                       (1)

#define EMAC_CHECK_LINK_TIME                msecs_to_jiffies(1000)
#define EMAC_CHECK_CNT                      (500000)
#define RTL_8210                            (0x1CUL)
//--------------------------------------------------------------------------------------------------
//  Global variable
//--------------------------------------------------------------------------------------------------
u8 MY_MAC[6] = { 0x00UL, 0x55UL, 0x66UL, 0x00UL, 0x00UL, 0x01UL };
//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
#define EMAC_STATS_STRING_LEN 23

static char mstar_emac_stat_string[EMAC_STATS_STRING_LEN][ETH_GSTRING_LEN]=
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
#ifdef EMAC_10T_RANDOM_WAVEFORM
#define EMAC_PRIV_FLAGS_STRING_LEN      1
#define EMAC_PRIVATE_FLAG_10T_RANDOM    0x1UL

static char mstar_emac_private_flag_string[EMAC_PRIV_FLAGS_STRING_LEN][ETH_GSTRING_LEN]=
{
    {"10T_Random"},
};
#else
#define EMAC_PRIV_FLAGS_STRING_LEN      0
#endif


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
#ifdef EMAC_GMAC_DESCRIPTOR
struct rx_descriptor
{
  u32   addr;
  u32   low_tag;
  u32   high_tag;
  u32   reserve;
};
#else
struct rx_descriptor
{
  u32   addr;
  u32   size;
#ifdef EMAC_64BIT_MOBF_DESCRIPTOR_TABLE_128BIT_ALIGNMENT_PATCH
  u32   dummy1;
  u32   dummy2;
#endif
};
#endif
#endif

/* slt-test */
/* auto-nego */
#define SIOC_SLT_TEST_AN_RUN (SIOCDEVPRIVATE + 13)
/* force speed */
#define SIOC_SLT_TEST_FS_RUN (SIOCDEVPRIVATE + 14)
/* clear test env */
#define SIOC_SLT_TEST_CLR (SIOCDEVPRIVATE + 15)

#ifdef EMAC_NEW_WOC
#define UDP_PROTOCOL 17
#define TCP_PROTOCOL 6
#define SIOC_SET_WOP_CMD (SIOCDEVPRIVATE + 11)
#define SIOC_CLR_WOP_CMD (SIOCDEVPRIVATE + 12)

#define WOC_PATTERN_BYTES_MAX 128
#define WOC_FILTER_NUM_MAX 20
#define WOC_PATTERN_CHECK_LEN_MAX 38

#define ETH_PROTOCOL_NUM_OFFSET 23
#define ETH_PROTOCOL_DEST_PORT_H_OFFSET 36
#define ETH_PROTOCOL_DEST_PORT_L_OFFSET 37

struct ioctl_woc_para_cmd
{
    u8 protocol_type;
    u8 port_count;
    u32 *port_array;
};

#ifdef CONFIG_COMPAT
struct ioctl_woc_para_cmd32
{
    u8 protocol_type;
    u8 port_count;
    u32 port_array;
};
#endif /* CONFIG_COMPAT */
#endif /* EMAC_NEW_WOC */

struct ioctl_slt_para_cmd
{
    u32 test_count;
    u32 failed_count;
};

struct _BasicConfigEMAC
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
typedef struct _BasicConfigEMAC BasicConfigEMAC;

struct _UtilityVarsEMAC
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
    u8  initedEMAC;         // 0:Not initialized, 1:Initialized.
    u8  flagRBNA;
    // Misc Counter:
    u32 cntRxFrames;        // Reset = 0x00000000 (Counter of RX frames,no matter it's me or not)
    u32 cntReadONE_RX;      // Counter for ReadONE_RX_Frame
    u32 cntCase20070806;
    u32 cntChkToTransmit;
    // Misc Variables:
    u32 mainThreadTasks;    // (20071029_CHARLES) b0=Poweroff,b1=Poweron
#ifdef EMAC_XIU16_SW_PROTECT
    // Link Speed Updated
    u8 link_sp_updated;
    // Register Golden Value:
    u32 CFG_val_golden;
    u32 CTL_val_golden;
    // TX Error Count:
    u32 tsrErrCnt;
#endif /* EMAC_XIU16_SW_PROTECT */
};
typedef struct _UtilityVarsEMAC UtilityVarsEMAC;

struct EMAC_private
{
    struct net_device *dev;
    struct device *ddev;
    struct net_device_stats stats;
    struct mii_if_info mii;             /* ethtool support */
    struct timer_list Link_timer;
#ifdef TX_ZERO_COPY
    struct timer_list TX_free_timer;
#endif
    u32 PreLinkStatus;
    u32 initstate;
    u32 msglvl;
    u32 private_flag;
    struct task_struct *ewave_task;
    BasicConfigEMAC ThisBCE;
    UtilityVarsEMAC ThisUVE;
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
#ifdef CONFIG_EMAC_TR_PN_SWAP
    bool is_tr_pn_swap;
#endif /* CONFIG_EMAC_TR_PN_SWAP */
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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
    long last_rx_time;
#endif
};

#endif
// -----------------------------------------------------------------------------
// Linux EMAC.h End
// -----------------------------------------------------------------------------
