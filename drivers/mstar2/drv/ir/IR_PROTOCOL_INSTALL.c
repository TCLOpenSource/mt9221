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

//<MStar Software>

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include "IR_PROTOCOL_INSTALL.h"
#ifdef SUPPORT_MULTI_PROTOCOL
#include "IR_PROTOCOL_NONE.c"
#include "IR_PROTOCOL_NEC.c"
#include "IR_PROTOCOL_RC5.c"
#include "IR_PROTOCOL_OCN.c"

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
DRV_IR_PROTOCOL_TYPE* _GetProtocolEntry(IR_PROCOCOL_TYPE eProtocol)
{
    extern DRV_IR_PROTOCOL_TYPE GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_NONE);
    extern DRV_IR_PROTOCOL_TYPE GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_RC5);
    extern DRV_IR_PROTOCOL_TYPE GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_NEC);
    extern DRV_IR_PROTOCOL_TYPE GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_PZ_OCN);

    switch(eProtocol)
    {
        case E_IR_PROTOCOL_NONE:
            return &GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_NONE);
            break;
        case E_IR_PROTOCOL_NEC:
            return &GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_NEC);
            break;
        case E_IR_PROTOCOL_RC5:
            return &GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_RC5);
            break;
        case E_IR_PROTOCOL_PZ_OCN:
            return &GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_PZ_OCN);
            break;
        default:
            printk("not support protocol\n");
    }

    return &GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_NONE);
}
#endif
