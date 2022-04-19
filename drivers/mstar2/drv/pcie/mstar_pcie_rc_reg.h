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


#ifndef _MSTAR_PCIE_RC_HEADER
#define _MSTAR_PCIE_RC_HEADER

/*
 * ============================================================================
 * INCLUDE HEADERS
 * ============================================================================
 */
#include "mstar_pcie_addrmap.h"

/*
 * ============================================================================
 * LOCAL VARIABLES DEFINITIONS
 * ============================================================================
 */

/*
 * ============================================================================
 * CONSTANT DEFINITIONS
 * ============================================================================
 */

/****  PCIE RC MAC SYSTEM REGISTERS  ****/
#define REG_PCIE_GENERATION_SEL     (0x0000*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_PCIE_GENERATION_SEL_MASK      0x00000001U
#define REG_PERFORMANCE_DATA_OUT0   (0x0001*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_PERFORMANCE_DATA_OUT0_MASK    0x0000FFFFU
#define REG_PERFORMANCE_DATA_OUT1   (0x0002*4 + (REG_RC_BASE))		/* 2 BITS */
		#define REG_PERFORMANCE_DATA_OUT1_MASK    0x00000003U
#define REG_PERFORMANCE_DATA_CNTVAL (0x0002*4 + (REG_RC_BASE))		/* 12 BITS */
		#define REG_PERFORMANCE_DATA_CNTVAL_MASK  0x0000FFF0U
#define REG_LINK_TRAINING_ENABLE    (0x0003*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_LINK_TRAINING_ENABLE_MASK     0x00000001U
#define REG_RC_RST_CTRL             (0x0004*4 + (REG_RC_BASE))
		#define REG_CORE_RST                  (1<<0)
		#define REG_PIPE_RST                  (1<<1)
		#define REG_AXI_RST                   (1<<2)
		#define REG_APB_RST                   (1<<3)
		#define REG_MGMT_RST                  (1<<4)
		#define REG_MGMT_STICKY_RST           (1<<5)
		#define REG_PHY_RST                   (1<<6)
		#define REG_HOT_RST                   (1<<8)
#define REG_DEBUG_DATA_OUT          (0x0005*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_DEBUG_DATA_OUT_MASK           0x0000FFFFU
#define REG_LTSSM_STATE             (0x0006*4 + (REG_RC_BASE))		/* 6 BITS */
		#define REG_LTSSM_STATE_MASK              0x0000003FU
#define REG_MEM_SD_EN               (0x0007*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_MEM_SD_EN_MASK                0x00000001U
#define REG_PERFORMANCE_DATA_CTRL   (0x0007*4 + (REG_RC_BASE))
		#define REG_PERFORMANCE_DATA_SEL          0x00001F00U
		#define REG_PERFORMANCE_DATA_CNTEN        0x00002000U
		#define REG_PERFORMANCE_DATA_CNTCLR       0x00004000U
#define REG_PCIE_IBWP_LOCK          (0x0008*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_PCIE_IBWP_LOCK_MASK           0x00000001U
#define REG_CLK_AXI_SEL             (0x0009*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_CLK_AXI_SEL_MASK              0x00000001U


/****  PCIE RC MAC CONFIGURATION & STATUS REGISTER  ****/
#define REG_CONFIG_ENABLE           (0x0014*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_CONFIG_ENABLE_MASK            0x00000001U
#define REG_LATCH_EN_CTRL           (0x0015*4 + (REG_RC_BASE))
		#define REG_LINK_STATUS_LATCH_EN          (1<<0)
		#define REG_LINK_POWER_STATE_LATCH_EN     (1<<1)
		#define REG_LTSSM_STATE_LATCH_EN          (1<<2)
#define REQ_PM_TRANSITION_L23_READY (0x0016*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REQ_PM_TRANSITION_L23_READY_MASK  0x00000001U
#define REG_CLIENT_REQ_EXIT_L2      (0x0016*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_CLIENT_REQ_EXIT_L2_MASK       0x00000002U
#define REG_POWER_STATE_CHANGE_ACK  (0x0017*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_POWER_STATE_CHANGE_ACK_MASK   0x00000001U
#define REG_LINK_STATUS             (0x0018*4 + (REG_RC_BASE))		/* 2 BITS */
		#define REG_LINK_STATUS_MASK              0x00000003U
		#define DL_INITIALIZATION_COMPLETED       0x03
#define REG_NEGOTIATED_SPEED        (0x0018*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_NEGOTIATED_SPEED_MASK         0x00000008U
#define REG_LINK_POWER_STATE        (0x0018*4 + (REG_RC_BASE))		/* 4 BITS */
		#define REG_LINK_POWER_STATE_MASK         0x000000F0U
#define REG_MAX_READ_REQ_SIZE       (0x0018*4 + (REG_RC_BASE))		/* 3 BITS */
		#define REG_MAX_READ_REQ_SIZE_MASK        0x00000700U
#define REG_MAX_PAYLOAD_SIZE        (0x0018*4 + (REG_RC_BASE))		/* 3 BITS */
		#define REG_MAX_PAYLOAD_SIZE_MASK         0x00007000U
#define REG_RCB_STATUS              (0x0018*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_RCB_STATUS_MASK               0x00008000U
#define REG_FUNCTION_STATUS         (0x0019*4 + (REG_RC_BASE))		/* 4 BITS */
		#define REG_FUNCTION_STATUS_MASK          0x0000000FU
#define REG_FUNCTION_POWER_STATE    (0x001A*4 + (REG_RC_BASE))		/* 3 BITS */
		#define REG_FUNCTION_POWER_STATE_MASK     0x00000007U
#define REG_LTR_MECHANISM_ENABLE    (0x001B*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_LTR_MECHANISM_ENABLE_MASK     0x00000010U
#define REG_OBFF_ENABLE             (0x001B*4 + (REG_RC_BASE))		/* 2 BITS */
		#define REG_OBFF_ENABLE_MASK              0x00000060U
#define REG_TPH_REQUESTER_ENABLE    (0x001B*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_TPH_REQUESTER_ENABLE_MASK     0x00001000U
#define REG_TPH_ST_MODE             (0x001B*4 + (REG_RC_BASE))		/* 3 BITS */
		#define REG_TPH_ST_MODE_MASK              0x0000E000U
#define REG_TPH_REQUESTER_ENABLE    (0x001B*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_TPH_REQUESTER_ENABLE_MASK     0x00001000U
#define REG_CORRECTABLE_ERROR_IN    (0x001C*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_CORRECTABLE_ERROR_IN_MASK     0x00000001U
#define REG_UNCORRECTABLE_ERROR_IN  (0x001D*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_UNCORRECTABLE_ERROR_IN_MASK   0x00000001U


/****  MSG RECEIVER HEADER & DATA REGISTERS  ****/
#define REG_IB_MSG_HEADER0          (0x0020*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER0_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER1          (0x0021*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER1_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER2          (0x0022*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER2_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER3          (0x0023*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER3_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER4          (0x0024*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER4_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER5          (0x0025*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER5_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER6          (0x0026*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER6_MASK           0x0000FFFFU
#define REG_IB_MSG_HEADER7          (0x0027*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_HEADER7_MASK           0x0000FFFFU
#define REG_IB_MSG_DATA0            (0x0028*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA0_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA1            (0x0029*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA1_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA2            (0x002A*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA2_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA3            (0x002B*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA3_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA4            (0x002C*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA4_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA5            (0x002D*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA5_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA6            (0x002E*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA6_MASK             0x0000FFFFU
#define REG_IB_MSG_DATA7            (0x002F*4 + (REG_RC_BASE))		/* 16 BITS */
		#define REG_IB_MSG_DATA7_MASK             0x0000FFFFU


/****  MSG RECEIVER CONTROL & STATUS REGISTERS  ****/
#define REG_IB_MSG_POP_HEADER       (0x0030*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_IB_MSG_POP_HEADER_MASK        0x00000001U
#define REG_IB_MSG_POP_DATA         (0x0030*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_IB_MSG_POP_DATA_MASK          0x00000010U
#define REG_IB_MSG_HEADER_RST       (0x0031*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_IB_MSG_HEADER_RST_MASK        0x00000001U
#define REG_IB_MSG_DATA_RST         (0x0031*4 + (REG_RC_BASE))		/* 1 BITS */
		#define REG_IB_MSG_DATA_RST_MASK          0x00000010U
#define REG_IB_MSG_CTRL             (0x0032*4 + (REG_RC_BASE))
		#define REG_IB_MSG_HEADER_EMPTY           (1<<0)
		#define REG_IB_MSG_HEADER_UNDERFLOW       (1<<3)
		#define REG_IB_MSG_HEADER_FULL            (1<<4)
		#define REG_IB_MSG_HEADER_OVERFLOW        (1<<7)
		#define REG_IB_MSG_DATA_EMPTY             (1<<8)
		#define REG_IB_MSG_DATA_UNDERFLOW         (1<<11)
		#define REG_IB_MSG_DATA_FULL              (1<<12)
		#define REG_IB_MSG_DATA_OVERFLOW          (1<<15)
#define REG_IB_MSG_HEADR_WRPTR      (0x0033*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_IB_MSG_HEADR_WRPTR_MASK       0x0000001FU
#define REG_IB_MSG_HEADR_RDPTR      (0x0033*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_IB_MSG_HEADR_RDPTR_MASK       0x00000F80U
#define REG_IB_MSG_DATA_WRPTR       (0x0034*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_IB_MSG_DATA_WRPTR_MASK        0x0000001FU
#define REG_IB_MSG_DATA_RDPTR       (0x0034*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_IB_MSG_DATA_RDPTR_MASK        0x00000F80U


/****  INT STATUS REGISTER  ****/
#define REG_INT_STATUS0             (0x0040*4 + (REG_RC_BASE))		/* 12 BITS */
		#define REG_INT_STATUS0_MASK              0x00000FFFU
        #define REG_INT_RX_ELEC_IDLE              (1<<0)
        #define REG_INT_RX_ELEC_IDLE_N            (1<<1)
        #define REG_INT_LINK_ENTRY_L1             (1<<2)
        #define REG_INT_LINK_ENTRY_L2             (1<<3)
        #define REG_INT_LINKDW_RESET_OUT          (1<<4)
        #define REG_INT_LOCAL_INTERRUPT           (1<<5)
        #define REG_INT_POWER_STATE_CHANGE        (1<<7)
        #define REG_INT_FATAL_ERROR_OUT           (1<<8)
        #define REG_INT_NON_FATAL_ERROR_OUT       (1<<9)
        #define REG_INT_CORRECTABLE_ERROR_OUT     (1<<10)
        #define REG_INT_PHY_INTERRUPT             (1<<11)
#define REG_INT_STATUS1             (0x0041*4 + (REG_RC_BASE))		/* 8 BITS */
		#define REG_INT_STATUS1_MASK              0x000000FFU
        #define REG_INT_INTA_OUT                  (1<<0)
        #define REG_INT_INTB_OUT                  (1<<1)
        #define REG_INT_INTC_OUT                  (1<<2)
        #define REG_INT_INTD_OUT                  (1<<3)
        #define REG_INT_AXI2MI_BRIDGE             (1<<4)
        #define REG_INT_MSI_QUEUE                 (1<<5)
        #define REG_INT_MSI_VTABLE                (1<<6)
        #define REG_INT_MSG_RCV                   (1<<7)
#define REG_INT_RAW_STATUS0         (0x0042*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_INT_RAW_STATUS0_MASK          0x0000001FU

/****  INT CLEAR REGISTER  ****/
#define REG_INT_CLR0                (0x0044*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_INT_CLR0_MASK                 0x0000001FU

/****  INT MASK REGISTER  ****/
#define REG_INT_MSK0                (0x0046*4 + (REG_RC_BASE))		/* 12 BITS */
		#define REG_INT_MSK0_MASK                 0x00000FFFU
#define REG_INT_MSK1                (0x0047*4 + (REG_RC_BASE))		/* 8 BITS */
		#define REG_INT_MSK1_MASK                 0x000000FFU

/****  INT CONTROL REGISTER  ****/
#define REG_INT_LATCH_EN0           (0x0048*4 + (REG_RC_BASE))		/* 5 BITS */
		#define REG_INT_LATCH_EN0_MASK            0x0000001FU

#endif