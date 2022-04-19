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
/// @file   MDRV_NOE.h
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_NOE_NAT_H_
#define _MDRV_NOE_NAT_H_

#ifdef CONFIG_NOE_NAT_HW
//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------

#include "mdrv_hwnat.h"
//--------------------------------------------------------------------------------------------------
//  Constant definition
//--------------------------------------------------------------------------------------------------
extern int (*noe_nat_hook_rx)(struct sk_buff *skb);
extern int (*noe_nat_hook_tx)(struct sk_buff *skb, int gmac_no);
#define  NOE_HWNAT_HOOK_RX(skb)             noe_nat_hook_rx(skb)
#define  NOE_HWNAT_HOOK_TX(skb,no)          noe_nat_hook_tx(skb, no)
#define  IS_VALID_NOE_HWNAT_HOOK_RX         (noe_nat_hook_rx != NULL)
#define  IS_VALID_NOE_HWNAT_HOOK_TX         (noe_nat_hook_tx != NULL)
#define  IS_NOT_VALID_NOE_HWNAT_HOOK_RX         (noe_nat_hook_rx == NULL)
#define  IS_NOT_VALID_NOE_HWNAT_HOOK_TX         (noe_nat_hook_tx == NULL)
#define  NOE_HWNAT_HOOK_RX_INIT             (noe_nat_hook_rx = NULL)
#define  NOE_HWNAT_HOOK_TX_INIT             (noe_nat_hook_tx = NULL)

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Functions
//-------------------------------------------------------------------------------------------------

#endif /* CONFIG_NOE_NAT_HW  */
#endif /* _MDRV_NOE_H_ */
