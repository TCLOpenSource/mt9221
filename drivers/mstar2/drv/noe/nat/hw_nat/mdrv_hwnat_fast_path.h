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
/// @file   MDRV_NOE_NAT.h
/// @brief  NOE NAT Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MDRV_HWNAT_FASTPATH_H
#define _MDRV_HWNAT_FASTPATH_H

inline int32_t MDrv_HWNAT_Pptp_Lan(struct sk_buff *skb) {return 1;}
inline int32_t MDrv_HWNAT_Pptp_Wan(struct sk_buff *skb) {return 1;}
inline int32_t MDrv_HWNAT_L2tp_Lan(struct sk_buff *skb) {return 1;}
inline int32_t MDrv_HWNAT_L2tp_Wan(struct sk_buff *skb) {return 1;}
inline int32_t MDrv_HWNAT_Pptp_Lan_Parse_Layer(struct sk_buff * skb) {return 1;}
inline int32_t MDrv_HWNAT_Pptp_Wan_Parse_Layer(struct sk_buff * skb) {return 1;}
inline int32_t MDrv_HWNAT_L2tp_Wan_Parse_Layer(struct sk_buff * skb) {return 1;}
inline void MDrv_HWNAT_Pptp_L2tp_Update_Entry(uint8_t protocol, uint32_t addr,uint32_t src_ip, uint32_t foe_hash_index){}
inline int32_t MDrv_HWNAT_Send_Hash_Pkt(struct sk_buff *pskb) {return 1;}
inline int32_t MDrv_HWNAT_L2tp_Send_Hash_Pkt(struct sk_buff *pskb) {return 1;}
inline int32_t MDrv_HWNAT_Pptp_L2tp_Init(void) {return 1;}
inline int32_t MDrv_HWNAT_Pptp_L2tp_Clean(void) {return 1;}
#endif  /* _MDRV_HWNAT_FASTPATH_H */
