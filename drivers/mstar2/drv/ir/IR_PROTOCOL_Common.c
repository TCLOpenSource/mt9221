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


//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include "IR_PROTOCOL_Common.h"
#ifdef SUPPORT_MULTI_PROTOCOL
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#define SHOTLIST_EMPTY()    (_u8ShotHeadIdx==_u8ShotTailIdx)
#define SHOTLIST_FULL()     ((_u8ShotHeadIdx+1)%SHOT_BUF_MAX ==_u8ShotTailIdx)

#ifndef NULL
#define NULL 0
#endif
//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
U8 _u8ShotHeadIdx=0;
U8 _u8ShotTailIdx=0;
U32 _u32ShotValue[SHOT_BUF_MAX];
BOOL _bNegShot[SHOT_BUF_MAX];

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
U8 _MulProtCommon_GetShotDataSize(void)
{
    if(_u8ShotHeadIdx>=_u8ShotTailIdx)
        return _u8ShotHeadIdx- _u8ShotTailIdx;
    else
        return (SHOT_BUF_MAX-_u8ShotTailIdx)+_u8ShotHeadIdx;
}

BOOL _MulProtCommon_PeekShot(U8 u8Offset, U8 u8Count, U32 *pu32Value, BOOL *pbNegtive)
{
    U8 u8Index;
    if(pu32Value==NULL||pbNegtive==NULL)
    {
        //printk("Null parameter!\n");
        return FALSE;
    }

    if((u8Offset+u8Count)>_MulProtCommon_GetShotDataSize())//peek data out of bound
    {
        return FALSE;
    }
    else
    {
        for(u8Index=u8Offset; u8Index< u8Offset+u8Count; u8Index++)
        {
            if((_u8ShotTailIdx+u8Index)>=SHOT_BUF_MAX)
            {
                pu32Value[u8Index-u8Offset] =_u32ShotValue[(_u8ShotTailIdx+u8Index)%SHOT_BUF_MAX];
                pbNegtive[u8Index-u8Offset] = _bNegShot[(_u8ShotTailIdx+u8Index)%SHOT_BUF_MAX];
            }
            else
            {
                pu32Value[u8Index-u8Offset] =_u32ShotValue[_u8ShotTailIdx+u8Index];
                pbNegtive[u8Index-u8Offset] = _bNegShot[_u8ShotTailIdx+u8Index];
            }
        }
        return TRUE;
    }
}

U8 _MulProtCommon_LSB2MSB(U8 u8OrgData)
{
    U8 u8Index;
    U8 u8NewData=0;

    for(u8Index=0; u8Index<8; u8Index++)
    {
        u8NewData <<=1;
        u8NewData = u8NewData|(u8OrgData&0x1UL);
        u8OrgData >>=1;
    }

    return u8NewData;
}

void _Mdrv_MulProtCommon_ShotDataReset(void)
{
    _u8ShotHeadIdx=0;
    _u8ShotTailIdx=0;
}

BOOL _Mdrv_MulProtCommon_AddShot(U32 u32Value, BOOL bNegtive)
{
    if(SHOTLIST_FULL())
    {
        printk("Error buffer full\n");
        return FALSE;
    }
    else
    {
        _u32ShotValue[_u8ShotHeadIdx]= u32Value;
        _bNegShot[_u8ShotHeadIdx] = bNegtive;
        _u8ShotHeadIdx= (_u8ShotHeadIdx+1)%SHOT_BUF_MAX;
        return TRUE;
    }
}

void _MulProtCommon_dumpShotData(void)
{
    printk("-----dump data----\n");
    if(!SHOTLIST_EMPTY())
    {
        U8 u8Index=0;
        U32 u32ShotValue;
        BOOL bNegShot;
        while(_MulProtCommon_PeekShot(u8Index, 1, &u32ShotValue, &bNegShot)==TRUE)
        {
            if(bNegShot==TRUE)
                printk("N[%d]: %ld\n", u8Index, u32ShotValue);
            else
                printk("P[%d]: %ld\n", u8Index, u32ShotValue);

            u8Index++;
        }
    }
    printk("-----dump end----\n");
}
#endif
