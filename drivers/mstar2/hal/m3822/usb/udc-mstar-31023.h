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

//------------------------------------------------------------------------------
// FILE
//      ms_otg.h
//
// DESCRIPTION
//
// HISTORY
//
//------------------------------------------------------------------------------
#ifndef _UDC_MSTAR_31023_H
#define _UDC_MSTAR_31023_H

/******************************************************************************
 * SW patch config
 ******************************************************************************/
//***** for Cedric C3 patch*****//
#define DRAM_MORE_THAN_1G_PATCH	//Clippers HW switch miu0/miu1 at 1G, if miu0 > 1G, HW will access wrong place
/******************************************************************************
 * HW eco config
 ******************************************************************************/
#define AHB_WDATA_CHANGES_WITHOUT_HREADY_ECO
#define SHORTPKT_STATUS_CLEAR_ECO
#define	MONKEY_TEST_ECO	//avoid device can not enter high speed mode
#define ENABLE_TX_RX_RESET_ECO
#define DMA_RXTX_INT_LAST_DONE_ECO
#define XIU_ACCESS_FAILED_WITH_ECO //(when clk_mcu<120MHz)

//---- change to 55 interface
#define ENABLE_UTMI_55_INTERFACE
/******************************************************************************
 * HW config
 ******************************************************************************/
#define MAX_USB_DMA_CHANNEL  2
#define MAX_EP_NUMBER	4

static const char ep0name [] = "ep0";
static const char *const ep_name[] = {
    ep0name,                                /* everyone has ep0 */
	"ep1in-bulk", "ep2out-bulk", "ep3in-int",
};
#define MSB250X_ENDPOINTS ARRAY_SIZE(ep_name)

//---USB device switch pad to CID enable
#define CID_ENABLE

//------RIU, UTMI, USBC and OTG base address -----------------------------
/*---port2 support device---*/
#define RIU_BASE           0xfd200000
#define UTMI_BASE_ADDR     GET_UDC_REG_ADDR(RIU_BASE, 0x3800)
#define USBC_BASE_ADDR     GET_UDC_REG_ADDR(RIU_BASE, 0x13800)
#define OTG0_BASE_ADDR     GET_UDC_REG_ADDR(RIU_BASE, 0x12600)
#define BC_BASE_ADDR	   GET_UDC_REG_ADDR(RIU_BASE, 0x53880)
#endif  /* define _UDC_MSTAR_31023_H */
