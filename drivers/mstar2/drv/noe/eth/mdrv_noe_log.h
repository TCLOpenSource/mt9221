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
/// @file   MDRV_NOE_LOG.h
/// @brief  NOE Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _MDRV_NOE_LOG_H_
#define _MDRV_NOE_LOG_H_


typedef enum {
    E_MDRV_NOE_MSG_CTRL_NONE    = 0,
    E_MDRV_NOE_MSG_CTRL_ERR,
    E_MDRV_NOE_MSG_CTRL_DBG,
    E_MDRV_NOE_MSG_CTRL_DUMP,
}EN_MDRV_NOE_MSG_CTRL;


#define MDRV_NOE_MSG(type, format , args...)    \
    do{                                         \
        if (MDrv_NOE_LOG_Get_Level() >= type)    \
        {                                       \
            printk(format , ##args  );          \
        } \
    }while(0);

#define NOE_MSG_ERR(format, args...)  MDRV_NOE_MSG(E_MDRV_NOE_MSG_CTRL_ERR, format, ##args)
#define NOE_MSG_DBG(format, args...)  MDRV_NOE_MSG(E_MDRV_NOE_MSG_CTRL_DBG, format, ##args)
#define NOE_MSG_DUMP(format, args...) MDRV_NOE_MSG(E_MDRV_NOE_MSG_CTRL_DUMP, format, ##args)
#define NOE_MSG_MUST(format, args...) printk(format, ##args)


void MDrv_NOE_LOG_Set_Level(unsigned char level);
unsigned char MDrv_NOE_LOG_Get_Level(void);
void MDrv_NOE_LOG_Init(struct net_device *dev);
void MDrv_NOE_LOG_Dump_Skb(struct sk_buff* sk);
#endif /* _MDRV_NOE_LOG_H_ */
