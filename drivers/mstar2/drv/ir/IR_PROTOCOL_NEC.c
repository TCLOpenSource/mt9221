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
#include "IR_PROTOCOL_NEC.h"

//-------------------------------------------------------------------------------------------------
//  Debug Defines
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
static U32 _u32NecLastDecodeData=0;
static unsigned long _ulNecPreDecodeTime=0;
static U32 _u32NecData;
static U8 _u8NecBits=0;

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static IR_DECODE_STATUS _ParseRepeatCodeNec(void)//check if repeat symbol
{
    U8 u8Index;
    U32 u32ShotValue[IR_NEC_REPEAT_SHOT_COUNT];
    BOOL bNegShot[IR_NEC_REPEAT_SHOT_COUNT];

    if(_MulProtCommon_GetShotDataSize()<IR_NEC_REPEAT_SHOT_COUNT)
    {
        return E_IR_DECODE_DATA_SHORTAGE;
    }

    for(u8Index=0; u8Index<= _MulProtCommon_GetShotDataSize()-IR_NEC_REPEAT_SHOT_COUNT; u8Index++)
    {
        if(_MulProtCommon_PeekShot(u8Index, IR_NEC_REPEAT_SHOT_COUNT, u32ShotValue, bNegShot)==TRUE)
        {
            //first shot must be positive and at least 24.5ms delay (24.5 = 110-(9+4.5+32*2.25))
            if(bNegShot[0]== FALSE && u32ShotValue[0] >= IR_TIME_LOB(IR_TIME_NEC_REPEAT_DELAY, IR_TIME_NEC_TOLERENCE) &&
               u32ShotValue[1] >= IR_TIME_LOB(IR_TIME_NEC_REPEAT_SHOT_1, IR_TIME_NEC_TOLERENCE) &&
               u32ShotValue[1] <= IR_TIME_UPD(IR_TIME_NEC_REPEAT_SHOT_1, IR_TIME_NEC_TOLERENCE) &&
               u32ShotValue[2] >= IR_TIME_LOB(IR_TIME_NEC_REPEAT_SHOT_2, IR_TIME_NEC_TOLERENCE) &&
               u32ShotValue[2] <= IR_TIME_UPD(IR_TIME_NEC_REPEAT_SHOT_2, IR_TIME_NEC_TOLERENCE) &&
               u32ShotValue[3] >= IR_TIME_LOB(IR_TIME_NEC_REPEAT_SHOT_3, IR_TIME_NEC_TOLERENCE) &&
               u32ShotValue[3] <= IR_TIME_UPD(IR_TIME_NEC_REPEAT_SHOT_3, IR_TIME_NEC_TOLERENCE))
            {
                return E_IR_DECODE_DATA_OK;
            }
        }
        else
        {
            return E_IR_DECODE_ERR;
        }
    }

    return E_IR_DECODE_ERR;
}

static void _NecRAW2Format(U32 u32RawData, U32 *pu32CustCode, U16 *pu16KeyCode, U8 *pu8State, U8 *pu8Reserved)
{
    BOOL bRepeatKey=FALSE;

    if(LAST_KEY_PROTOCOL(E_IR_PROTOCOL_NEC))
    {
        if(u32RawData==_u32NecLastDecodeData)
        {
            bRepeatKey=TRUE;
        }

        if(bRepeatKey==TRUE)
        {
            *pu8State = E_IR_KEY_STATE_REPEAT;
        }
    }
    else
    {
        *pu8State = E_IR_KEY_STATE_PRESS;
    }

    *pu32CustCode = _MulProtCommon_LSB2MSB((u32RawData>>24)&0xFFUL)<<8 | _MulProtCommon_LSB2MSB((u32RawData>>16)&0xFFUL);
    *pu16KeyCode = _MulProtCommon_LSB2MSB((u32RawData>>8)&0xFFUL);
    return;
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
BOOL _ParseLeadCodeNec(U8 *pu8StartIndex)
{
    U8 u8Index;
    U32 u32ShotValue[IR_NEC_LEADCODE_SHOT_COUNT];
    BOOL bNegshot[IR_NEC_LEADCODE_SHOT_COUNT];
    U8 u8Datasize=0;

    u8Datasize = _MulProtCommon_GetShotDataSize();
    if(u8Datasize< IR_NEC_LEADCODE_SHOT_COUNT)
    {
        return FALSE;
    }

    for(u8Index=0; u8Index<= u8Datasize-(IR_NEC_LEADCODE_SHOT_COUNT); u8Index++)
    {
        if(_MulProtCommon_PeekShot(u8Index, IR_NEC_LEADCODE_SHOT_COUNT, u32ShotValue, bNegshot)==TRUE)
        {
            if(bNegshot[0]==TRUE)
            {
                continue;//1st shot must be positive
            }

            if(bNegshot[1]==TRUE && bNegshot[2]==FALSE)//2nd shot must be negtive
            {
                if((u32ShotValue[1] > IR_TIME_LOB(IR_TIME_NEC_HEADER_CODE, IR_TIME_NEC_LEADCODE_TOLERENCE) && u32ShotValue[1] < IR_TIME_UPD(IR_TIME_NEC_HEADER_CODE, IR_TIME_NEC_LEADCODE_TOLERENCE)) &&
                   (u32ShotValue[2] > IR_TIME_LOB(IR_TIME_NEC_HEADER_OFF_CODE, IR_TIME_NEC_LEADCODE_TOLERENCE) && u32ShotValue[2] < IR_TIME_UPD(IR_TIME_NEC_HEADER_OFF_CODE, IR_TIME_NEC_LEADCODE_TOLERENCE)))
                {
                    *pu8StartIndex = u8Index;
                    return TRUE;
                }
            }
        }
        else
        {
            return FALSE;
        }
    }

    return FALSE;

}


IR_DECODE_STATUS _ParseNEC(U32 *pu32CustCode, U16 *pu16KeyCode, U8 *pu8State, U8 *pu8Reserved)
{
    IR_DECODE_STATUS eDecodeStatu = E_IR_DECODE_ERR;
    U32 u32ShotValue;
    BOOL bNegShot=FALSE;
    BOOL bLastShotN=FALSE;
    U32 u32LastNValue=0;

    if (_MDrv_IR_GetSystemTime() - _ulNecPreDecodeTime> (IR_TIME_NEC_TIMEOUT-IR_TIME_UPD(IR_TIME_NEC_REPEAT_SHOT_1+IR_TIME_NEC_REPEAT_SHOT_2+IR_TIME_NEC_REPEAT_SHOT_3, IR_TIME_NEC_LEADCODE_TOLERENCE))/1000)//>110ms timeout
    {
        _u32NecLastDecodeData=0;
        _u32NecData=0;
        _u8NecBits=0;
    }

    _ulNecPreDecodeTime = _MDrv_IR_GetSystemTime();
    if(_u32NecLastDecodeData!=0)//decode success last time, should check if repeat symbol
    {
        if((eDecodeStatu = _ParseRepeatCodeNec())==E_IR_DECODE_DATA_OK)
        {
            _NecRAW2Format(_u32NecLastDecodeData, pu32CustCode, pu16KeyCode, pu8State, pu8Reserved);
        }

        return eDecodeStatu;
    }
    else
    {
        U8 u8StartIndex;

        if(_MulProtCommon_GetShotDataSize()<IR_NEC_LEADCODE_SHOT_COUNT)
        {
            return E_IR_DECODE_DATA_SHORTAGE;
        }

        if(_ParseLeadCodeNec(&u8StartIndex)==FALSE)
        {
            return E_IR_DECODE_ERR;
        }

        while(_MulProtCommon_PeekShot(u8StartIndex+IR_NEC_LEADCODE_SHOT_COUNT+_u8NecBits*2, 1, &u32ShotValue, &bNegShot)==TRUE)//only use peek here
        {
            if(bNegShot==TRUE)//Negtive
            {
                bLastShotN = TRUE;
                u32LastNValue = u32ShotValue;
            }
            else//positive
            {
                if(bLastShotN==TRUE)//A complete logic is: N+P
                {
                    if(u32LastNValue+u32ShotValue >= IR_TIME_LOB(IR_TIME_NEC_LOGI_0, IR_TIME_NEC_TOLERENCE) && u32LastNValue+u32ShotValue <= IR_TIME_UPD(IR_TIME_NEC_LOGI_0, IR_TIME_NEC_TOLERENCE))//logical 0
                    {
                        _u32NecData<<=1;
                        _u32NecData |=0;
                        _u8NecBits+=1;
                    }
                    else if(u32LastNValue+u32ShotValue >= IR_TIME_LOB(IR_TIME_NEC_LOGI_1, IR_TIME_NEC_TOLERENCE) && u32LastNValue+u32ShotValue <= IR_TIME_UPD(IR_TIME_NEC_LOGI_1, IR_TIME_NEC_TOLERENCE))//logical 1
                    {
                        _u32NecData <<=1;
                        _u32NecData |=1;
                        _u8NecBits+=1;
                    }
                    else
                    {
                        _u32NecLastDecodeData=0;
                        _u32NecData=0;
                        _u8NecBits=0;
                        return E_IR_DECODE_ERR;
                    }
                }

                bLastShotN=FALSE;
                u32LastNValue = 0;
            }

            u8StartIndex++;
        }

        if(_u8NecBits==IR_NEC_BITS && bLastShotN==TRUE)
        {
            _NecRAW2Format(_u32NecData, pu32CustCode, pu16KeyCode, pu8State, pu8Reserved);
            _u32NecLastDecodeData=_u32NecData;
            _u32NecData=0;
            _u8NecBits=0;
            return E_IR_DECODE_DATA_OK;
        }

        return E_IR_DECODE_DATA_SHORTAGE;
    }

}

DRV_IR_PROTOCOL_TYPE GET_IR_PROTOCOL_ENTRY(E_IR_PROTOCOL_NEC) =
{
    .name               ="IR_PROTOCOL_NEC",
    .etype              =E_IR_PROTOCOL_NEC,
    .findleadcode       =_ParseLeadCodeNec,
    .parseprotocol      =_ParseNEC,
    .u8LeadCodeMinCount = IR_NEC_LEADCODE_SHOT_COUNT,
    .u32Timeout         = IR_TIME_UPD(IR_TIME_NEC_TIMEOUT, IR_TIME_NEC_TOLERENCE),
};

