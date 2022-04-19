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
/// @file   MDRV_NOE_PROC.h
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_NOE_PROC_H_
#define _MDRV_NOE_PROC_H_


#define NOE_PROC_LRO_STATS          "lro_stats"
#define NOE_PROC_TSO_LEN            "tso_len"
#define NOE_PROC_DIR                "noe"
#define NOE_PROC_SKBFREE            "skb_free"
#define NOE_PROC_TX_RING            "tx_ring"
#define NOE_PROC_RX_RING            "rx_ring"
#define NOE_PROC_LRO_RX_RING1       "lro_rx_ring1"
#define NOE_PROC_LRO_RX_RING2       "lro_rx_ring2"
#define NOE_PROC_LRO_RX_RING3       "lro_rx_ring3"
#define NOE_PROC_NUM_OF_TXD         "num_of_txd"
#define NOE_PROC_TSO_LEN            "tso_len"
#define NOE_PROC_LRO_STATS          "lro_stats"
#define NOE_PROC_HW_LRO_STATS       "hw_lro_stats"
#define NOE_PROC_HW_LRO_AUTO_TLB    "hw_lro_auto_tlb"
#define NOE_PROC_HW_IO_COHERENT     "hw_iocoherent"
#define NOE_PROC_GMAC               "gmac"
#define NOE_PROC_GMAC2              "gmac2"
#define NOE_PROC_CP0                "cp0"
#define NOE_PROC_QSCH               "qsch"
#define NOE_PROC_READ_VAL           "regread_value"
#define NOE_PROC_WRITE_VAL          "regwrite_value"
#define NOE_PROC_ADDR               "reg_addr"
#define NOE_PROC_CTL                "procreg_control"
#define NOE_PROC_RXDONE_INTR        "rxdone_intr_count"
#define NOE_PROC_ESW_INTR           "esw_intr_count"
#define NOE_PROC_ESW_CNT            "esw_cnt"
#define NOE_PROC_ETH_CNT            "eth_cnt"
#define NOE_PROC_SNMP               "snmp"
#define NOE_PROC_SET_LAN_IP         "set_lan_ip"
#define NOE_PROC_SCHE               "schedule" /* TASKLET_WORKQUEUE_SW */
#define NOE_PROC_QDMA               "qdma"
#define NOE_PROC_INT_DBG            "int_dbg"
#define NOE_PROC_PIN_MUX            "mux"
#define NOE_PROC_LOG_CTRL           "log"

struct mdrv_noe_proc_intr {
    unsigned int RX_COHERENT_CNT;
    unsigned int RX_DLY_INT_CNT;
    unsigned int TX_COHERENT_CNT;
    unsigned int TX_DLY_INT_CNT;
    unsigned int RING3_RX_DLY_INT_CNT;
    unsigned int RING2_RX_DLY_INT_CNT;
    unsigned int RING1_RX_DLY_INT_CNT;
    unsigned int RXD_ERROR_CNT;
    unsigned int ALT_RPLC_INT3_CNT;
    unsigned int ALT_RPLC_INT2_CNT;
    unsigned int ALT_RPLC_INT1_CNT;
    unsigned int RX_DONE_INT3_CNT;
    unsigned int RX_DONE_INT2_CNT;
    unsigned int RX_DONE_INT1_CNT;
    unsigned int RX_DONE_INT0_CNT;
    unsigned int TX_DONE_INT3_CNT;
    unsigned int TX_DONE_INT2_CNT;
    unsigned int TX_DONE_INT1_CNT;
    unsigned int TX_DONE_INT0_CNT;
};



struct PDMA_LRO_AUTO_TLB_INFO0_T {
    unsigned int DTP:16;
    unsigned int STP:16;
};

struct PDMA_LRO_AUTO_TLB_INFO1_T {
    unsigned int SIP0:32;
};

struct PDMA_LRO_AUTO_TLB_INFO2_T {
    unsigned int SIP1:32;
};

struct PDMA_LRO_AUTO_TLB_INFO3_T {
    unsigned int SIP2:32;
};

struct PDMA_LRO_AUTO_TLB_INFO4_T {
    unsigned int SIP3:32;
};

struct PDMA_LRO_AUTO_TLB_INFO5_T {
    unsigned int VLAN_VID0:32;
};

struct PDMA_LRO_AUTO_TLB_INFO6_T {
    unsigned int VLAN_VID1:16;
    unsigned int VLAN_VID_VLD:4;
    unsigned int CNT:12;
};

struct PDMA_LRO_AUTO_TLB_INFO7_T {
    unsigned int DW_LEN:32;
};

struct PDMA_LRO_AUTO_TLB_INFO8_T {
    unsigned int DIP_ID:2;
    unsigned int IPV6:1;
    unsigned int IPV4:1;
    unsigned int RESV:27;
    unsigned int VALID:1;
};

struct PDMA_LRO_AUTO_TLB_INFO {
    struct PDMA_LRO_AUTO_TLB_INFO0_T auto_tlb_info0;
    struct PDMA_LRO_AUTO_TLB_INFO1_T auto_tlb_info1;
    struct PDMA_LRO_AUTO_TLB_INFO2_T auto_tlb_info2;
    struct PDMA_LRO_AUTO_TLB_INFO3_T auto_tlb_info3;
    struct PDMA_LRO_AUTO_TLB_INFO4_T auto_tlb_info4;
    struct PDMA_LRO_AUTO_TLB_INFO5_T auto_tlb_info5;
    struct PDMA_LRO_AUTO_TLB_INFO6_T auto_tlb_info6;
    struct PDMA_LRO_AUTO_TLB_INFO7_T auto_tlb_info7;
    struct PDMA_LRO_AUTO_TLB_INFO8_T auto_tlb_info8;
};



int MDrv_NOE_Update_Tso_Len(int tso_len);
int MDrv_NOE_PROC_Init(struct net_device *dev);
void MDrv_NOE_PROC_Exit(void);
#if FE_HW_LRO
int MDrv_NOE_LRO_PROC_Init(struct proc_dir_entry *proc_reg_dir, struct net_device *dev);
void MDrv_NOE_LRO_PROC_Exit(struct proc_dir_entry *proc_reg_dir);
void MDrv_NOE_LRO_PROC_Update_Flush_Stats(unsigned int ring_num, struct PDMA_rxdesc *rx_ring);
void MDrv_NOE_LRO_PROC_Update_Stats(unsigned int ring_num, struct PDMA_rxdesc *rx_ring);
#else
static inline int MDrv_NOE_LRO_PROC_Init(struct proc_dir_entry *proc_reg_dir, struct net_device *dev) { return 0;}
static inline void MDrv_NOE_LRO_PROC_Exit(struct proc_dir_entry *proc_reg_dir) {}
static inline void MDrv_NOE_LRO_PROC_Update_Flush_Stats(unsigned int ring_num, struct PDMA_rxdesc *rx_ring) {}
static inline void MDrv_NOE_LRO_PROC_Update_Stats(unsigned int ring_num, struct PDMA_rxdesc *rx_ring) {}
#endif /* FE_HW_LRO */

#endif /* _MDRV_NOE_PROC_H_ */
