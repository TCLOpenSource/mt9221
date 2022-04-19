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

#ifndef IR_PROTOCOL_COMMON_H
#define IR_PROTOCOL_COMMON_H

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include "mdrv_types.h"
#include "mdrv_ir_st.h"
//#define SUPPORT_MULTI_PROTOCOL
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define SHOT_BUF_MAX    100
#define PROTOCOL_SUPPORT_MAX    E_IR_PROTOCOL_MAX-1
#define SHOTLIST_EMPTY()    (_u8ShotHeadIdx==_u8ShotTailIdx)
#define SHOTLIST_FULL()     ((_u8ShotHeadIdx+1)%SHOT_BUF_MAX ==_u8ShotTailIdx)
#define IR_TIME_UPD(time, tolerance)   ((U32)(((double)time)*((double)1+tolerance)))
#define IR_TIME_LOB(time, tolerance)   ((U32)(((double)time)*((double)1-tolerance)))
#define LAST_KEY_PROTOCOL(x)   (x==_eLastKeyProtocol)


#define _GET_IR_PROTOCOL_ENTRY(var, Name) var##Name
#define GET_IR_PROTOCOL_ENTRY(Name) _GET_IR_PROTOCOL_ENTRY(IR_PROTOCOL_ENTRY_,Name)
//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_IR_KEY_STATE_PRESS = 0,
    E_IR_KEY_STATE_REPEAT,
    E_IR_KEY_STATE_RELEASE,
} IR_KEY_STATE_e;

typedef enum
{
    E_IR_DECODE_ERR=0,
    E_IR_DECODE_DATA_SHORTAGE,
    E_IR_DECODE_DATA_OK,
} IR_DECODE_STATUS;

typedef BOOL                 drv_ir_protocol_findleadcode(U8 *pu8StartIndex);
typedef IR_DECODE_STATUS     drv_ir_protocol_praseprotocol(U32 *pu32CustCode, U16 *pu16KeyCode, U8 *pu8State, U8 *pu8Reserved);

typedef struct
{
    const char                          *name;          // ir protocol name
    IR_PROCOCOL_TYPE                    etype;          //ir enum value
    drv_ir_protocol_findleadcode        *findleadcode; //check if lead code exist
    drv_ir_protocol_praseprotocol       *parseprotocol;//parse by specific protocol
    U8                                  u8LeadCodeMinCount;//the least shot count to identify lead code
    U32                                 u32Timeout;
} DRV_IR_PROTOCOL_TYPE;
//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
extern IR_PROCOCOL_TYPE _eLastKeyProtocol;
//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Global Functions
//-------------------------------------------------------------------------------------------------
U8 _MulProtCommon_GetShotDataSize(void);
BOOL _MulProtCommon_PeekShot(U8 u8Offset, U8 u8Count, U32 *pu32Value, BOOL *pbNegtive);
U8 _MulProtCommon_LSB2MSB(U8 u8OrgData);
void _MulProtCommon_dumpShotData(void);

extern unsigned long _MDrv_IR_GetSystemTime(void);
extern int printk(const char *fmt, ...);
//-------------------------------------------------------------------------------------------------
// Global Functions for Mdrv_ir.c (should not be called in private protocol)
//-------------------------------------------------------------------------------------------------
void _Mdrv_MulProtCommon_ShotDataReset(void);
BOOL _Mdrv_MulProtCommon_AddShot(U32 u32Value, BOOL bNegtive);
#endif

