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
/// file    mhal_mbx.c
/// @brief  MStar MailBox DDI
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _HAL_MBX_C

//=============================================================================
// Include Files
//=============================================================================
#include <linux/version.h>
#include <linux/kernel.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,1,10)
#include <asm/barrier.h>
#endif /*LINUX_VERSION_CODE*/

#include "mdrv_mstypes.h"
#include <asm/barrier.h>
#include "mdrv_mbx.h"
#include "mhal_mbx.h"

//=============================================================================
// Compile options
//=============================================================================


//=============================================================================
// Local Defines
//=============================================================================

//=============================================================================
// Debug Macros
//=============================================================================
#define MBXHAL_DEBUG
#ifdef MBXHAL_DEBUG
    #define MBXHAL_PRINT(fmt, args...)      printk("[MailBox (HAL Driver)][%05d] " fmt, __LINE__, ## args)
    #define MBXHAL_ASSERT(_cnd, _fmt, _args...)                   \
                                    if (!(_cnd)) {              \
                                        MBXHAL_PRINT(_fmt, ##_args);  \
                                        while(1);               \
                                    }
#else
    #define MBXHAL_PRINT(_fmt, _args...)
    #define MBXHAL_ASSERT(_cnd, _fmt, _args...)
#endif

//=============================================================================
// Macros
//=============================================================================

//=============================================================================
// Local Variables
//=============================================================================
static MS_U16 _u16MbxGroupIdMBXHAL[E_MBX_ROLE_MAX][E_MBX_ROLE_MAX] =
                                        { {0xFF, REG_MBX_GROUP0, REG_MBX_GROUP4, REG_MBX_GROUP0}, {REG_MBX_GROUP1, 0xFF, 0xFF, 0xFF}, {REG_MBX_GROUP3, 0xFF, 0xFF, 0xFF} };

//=============================================================================
// Global Variables
//=============================================================================

//=============================================================================
// Local Function Prototypes
//=============================================================================
static void _MHAL_MBX_FireMsg (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole);
static void _MHAL_MBX_RecvMsg(MBX_Msg* pMbxMsg, MBX_ROLE_ID eDstRole);

//=============================================================================
// Local Function
//=============================================================================
//-------------------------------------------------------------------------------------------------
/// Fire Msg to MailBox hardware.
/// @param  eSrcCPUID                  \b IN: The Firer CPUID
/// @return void
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
void _MHAL_MBX_FireMsg (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole)
{
    MS_S32 s32Idx;

    /* fill mail box register. */
    mb();
    //REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcCPUID], REG8_MBX_CTRL) = pMbxMsg->u8Ctrl;
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID], REG8_MBX_MAIL_CLASS) = pMbxMsg->u8MsgClass;
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID], REG8_MBX_MAIL_IDX) = pMbxMsg->u8Index;
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID], REG8_MBX_PARAMETER_CNT) = pMbxMsg->u8ParameterCount;
    mb();

    for(s32Idx=0;  s32Idx<pMbxMsg->u8ParameterCount; s32Idx++)
    {
        REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID], s32Idx+REG8_MBX_PARAMETER_S) = pMbxMsg->u8Parameters[s32Idx];
        mb();
    }

    //REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcCPUID], REG8_MBX_STATE_0) = pMbxMsg->u8S0;
    //REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eSrcCPUID], REG8_MBX_STATE_1) = pMbxMsg->u8S1;
}

//-------------------------------------------------------------------------------------------------
/// Recv Msg From MailBox hardware.
/// @param  pMbxMsg                  \b INOUT: The Recv CPUID, and where mail to put
/// @return void
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
void _MHAL_MBX_RecvMsg(MBX_Msg* pMbxMsg, MBX_ROLE_ID eDstRole)
{
    MS_S32 s32Idx;

    mb();
    pMbxMsg->u8Ctrl = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], REG8_MBX_CTRL);
    pMbxMsg->u8MsgClass = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], REG8_MBX_MAIL_CLASS);
    pMbxMsg->u8Index = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], REG8_MBX_MAIL_IDX);
    pMbxMsg->u8ParameterCount = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], REG8_MBX_PARAMETER_CNT);
    mb();

    for(s32Idx=0;  s32Idx<10; s32Idx++)
    {
        pMbxMsg->u8Parameters[s32Idx] = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], s32Idx+REG8_MBX_PARAMETER_S);
        mb();
    }

    pMbxMsg->u8S0 = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], REG8_MBX_STATE_0);
    pMbxMsg->u8S1 = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole], REG8_MBX_STATE_1);
}

//=============================================================================
// Mailbox HAL Driver Function
//=============================================================================

//-------------------------------------------------------------------------------------------------
/// Init MailBox hardware.
/// @param  eHostRole                  \b IN: The host Role ID
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MHAL_MBX_Init (MBX_ROLE_ID eHostRole)
{
    return MHAL_MBX_SetConfig(eHostRole);
}

//-------------------------------------------------------------------------------------------------
/// Set MailBox Group Regs.
/// @param  eHostRole                  \b IN: The host Role ID
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[MXLIB] <em>Can't be called Before MHAL_MBX_Init </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MHAL_MBX_SetConfig(MBX_ROLE_ID eHostRole)
{
    MS_S32 s32MailIdx;

    /* clear host mail box register. */
    if(E_MBX_ROLE_HK == eHostRole) //It is HouseKeeping
    {
        for(s32MailIdx=0; s32MailIdx<16; s32MailIdx++)
        {
            REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[E_MBX_ROLE_HK][E_MBX_ROLE_CP], s32MailIdx) = 0x00;
            REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[E_MBX_ROLE_HK][E_MBX_ROLE_PM], s32MailIdx) = 0x00;
            mb();
        }
    }
    else
    {
        for(s32MailIdx=0; s32MailIdx<16; s32MailIdx++)
        {
            REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eHostRole][E_MBX_ROLE_HK], s32MailIdx) = 0x00;
            mb();
        }
    }

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Set Information to dedicated Mailbox HW Regs.
/// @param  eTargetRole \b IN: The correpsonded CPU ID for which MBX HW Group will be used for Set
/// @param  pU8Info \b IN: The Information which need to set
/// @param  u8Size \b IN: The Size(bytes) of pU8Info
/// @return E_MBX_SUCCESS
/// @return E_MBX_UNKNOW_ERROR
/// @attention
/// <b>[MXLIB] <em>It is used for speciall need when Init-Coprocesser</em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result  MHAL_MBX_SetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size)
{
    MS_S32 s32Idx;

    mb();
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 0+REG8_MBX_PARAMETER_S) = 0x00;
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 1+REG8_MBX_PARAMETER_S) = 0x00;
    mb();

    for(s32Idx=0; s32Idx<u8Size; s32Idx++)
    {
        REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], s32Idx+2+REG8_MBX_PARAMETER_S) = pU8Info[s32Idx];
        mb();
    }

    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 0+REG8_MBX_PARAMETER_S) = 0x78;
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 1+REG8_MBX_PARAMETER_S) = 0x56;
    mb();

    while((REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 0+REG8_MBX_PARAMETER_S) == 0x78)
            && (REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 1+REG8_MBX_PARAMETER_S) == 0x56));

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Get Information from dedicated Mailbox HW Regs.
/// @param  eTargetRole \b IN: The correpsonded CPU ID for which MBX HW Group will be used for Get
/// @param  pU8Info \b OUT: The Information Where we get to put
/// @param  u8Size \b IN: The Size(bytes) need to get
/// @return E_MBX_SUCCESS
/// @return E_MBX_UNKNOW_ERROR
/// @attention
/// <b>[MXLIB] <em>It is used for speciall need when Init-Coprocesser.</em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result  MHAL_MBX_GetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size)
{
    MS_S32 s32Idx;

    mb();
    while((REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 0+REG8_MBX_PARAMETER_S) != 0x78)
            || (REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 1+REG8_MBX_PARAMETER_S) != 0x56));

    for(s32Idx=0; s32Idx<u8Size; s32Idx++)
    {
        pU8Info[s32Idx] = REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], s32Idx+2+REG8_MBX_PARAMETER_S);
        mb();
    }

    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 0+REG8_MBX_PARAMETER_S) = 0x00;
    REG8_MBX_GROUP(_u16MbxGroupIdMBXHAL[eTargetRole][E_MBX_ROLE_HK], 1+REG8_MBX_PARAMETER_S) = 0x00;

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Fire Mail to MailBox hardware.
/// @param  eSrcRole                  \b IN: The Firer RoleID
/// @return E_MBX_ERR_PEER_CPU_NOTREADY: THE Peer CPU in fetching the msg
/// @return E_MBX_ERR_PEER_CPU_NOT_ALIVE: The Peer CPU Looks like not alive
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MHAL_MBX_Fire (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole)
{
#define TIMEOUT_CHK_BUSY 500000

    MS_BOOL bIPState;
	MS_U16  time_chk = TIMEOUT_CHK_BUSY;

    /* check paramter. */
    if(0xFF == _u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID])
    {
        return E_MBX_ERR_INVALID_PARAM;
    }

    /* check send bit in CONTROL register. */
    mb();
    bIPState = _FIRE(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);

    if(bIPState > 0)
    {
        bIPState = _BUSY(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
        if(bIPState > 0)
        {//the mail is in process
            MBXHAL_PRINT("[Fire Mail Fail] : co-processor is not ready!\n");
            return  E_MBX_ERR_PEER_CPU_NOTREADY;
        }
        else
        {//mail not processed yet!
            MBXHAL_PRINT("[Fire Mail Fail] : co-processor is not alive!\n");
            return  E_MBX_ERR_PEER_CPU_NOT_ALIVE;
        }
    }

	//Check Send MBX Busy Status bit
	mb();
	do
	{
        bIPState = _BUSY(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
	} while ((bIPState>0) && ((time_chk--)>0));

	if (time_chk == 0)
	{
		MBXHAL_PRINT("[Fire Mail Fail] : co-processor is still busy!\n");
		return E_MBX_ERR_PEER_CPU_BUSY;
	} 

    /* clear status1 register. */
    _S1_C(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
    // Clear Instant Setting:
    _INSTANT_C(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
    mb();

	//set message busy bit to prevent 
    _BUSY_S(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
		
    /* fill mail box register. */
    _MHAL_MBX_FireMsg(pMbxMsg, eSrcRole);

    /* set instant message attribute */
    if(pMbxMsg->eMsgType == E_MBX_MSG_TYPE_INSTANT)
    {
        _INSTANT_S(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
        mb();
    }

    _FIRE_S(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
    mb();

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Clear HW  Status register
/// @param  eSrcRole                  \b IN: The Firer Src Role
/// @param  eDstRole                  \b IN: The Firer Dst Role
/// @param  pFireStatus               \b OUT: THE Fire Status
/// @return void
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
void MHAL_MBX_ClearStatus (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole)
{
    /* clear status1 register. */
    _S1_C(_u16MbxGroupIdMBXHAL[eSrcRole][pMbxMsg->eRoleID]);
}

//-------------------------------------------------------------------------------------------------
/// Get HW Fire Status
/// @param  eSrcRole                  \b IN: The Firer Src Role
/// @param  eDstRole                  \b IN: The Firer Dst Role
/// @param  pFireStatus               \b OUT: THE Fire Status
/// @return E_MBX_ERR_INVALID_PARAM
/// @return E_MBX_SUCCESS
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MHAL_MBX_GetFireStatus(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Fire_Status *pFireStatus)
{
    /* check paramter. */
    if((0xFF == _u16MbxGroupIdMBXHAL[eSrcRole][eDstRole])
        || (NULL == pFireStatus))
    {
        return E_MBX_ERR_INVALID_PARAM;
    }

    mb();
    //still on Firing:
    if(_FIRE(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]))
    {
        *pFireStatus = E_MBXHAL_FIRE_ONGOING;
    }
    else if(_OVERFLOW(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]))
    {    //If Overflow:
        *pFireStatus = E_MBXHAL_FIRE_OVERFLOW;
    }
    else if(_DISABLED(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]))
    {   //If Disabled:
        *pFireStatus = E_MBXHAL_FIRE_DISABLED;
    }
    else
    {
        *pFireStatus = E_MBXHAL_FIRE_SUCCESS;
    }

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Recv Mail From MailBox hardware.
/// @param  pMbxMsg                  \b INOUT: The Recv Src RoleID, and where mail to put
/// @param  eDstRole                  \b IN: The Recv Dst RoleID
/// @return E_MBX_ERR_MSG_ALREADY_FETCHED: THE Msg has been fetched
/// @return E_MBX_ERR_INVALID_PARAM: THE Input Src Role & Dst Role is incorrect
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MHAL_MBX_Recv(MBX_Msg* pMbxMsg, MBX_ROLE_ID eDstRole)
{
    MS_BOOL bIPState;

    /* check paramter. */
    if(0xFF == _u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole])
    {
        return E_MBX_ERR_INVALID_PARAM;
    }

    mb();
    /* check send bit in CONTROL register. */
    bIPState = _FIRE(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole]);
    if(bIPState <= 0) //the message already fetched!
        return  E_MBX_ERR_MSG_ALREADY_FETCHED;

    /* set busy bit */
    _BUSY_S(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole]);
    mb();

    bIPState = _INSTANT(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole]) | _READBACK(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole]);
    if(bIPState)
        pMbxMsg->eMsgType= E_MBX_MSG_TYPE_INSTANT;
    else
        pMbxMsg->eMsgType= E_MBX_MSG_TYPE_NORMAL;

    /* recv mail msg */
    _MHAL_MBX_RecvMsg(pMbxMsg, eDstRole);

    /* clear busy bit */
    //_BUSY_C(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole]);
    /* clear send bit */
    //_FIRE_C(_u16MbxGroupIdMBXHAL[pMbxMsg->eRoleID][eDstRole]);

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// End Recv Mail From MailBox hardware. Set Status for FeedBack
/// @param  eSrcRole                  \b IN: The Recv Src RoleID
/// @param  eDstRole                  \b IN: The Recv Dst RoleID
/// @param  eRecvSatus                  \b IN: The Recv Status
/// @return E_MBX_ERR_INVALID_PARAM: THE Input Src Role & Dst Role is incorrect
/// @return E_MBX_SUCCESS: success
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MBX_Result MHAL_MBX_RecvEnd(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Recv_Status eRecvSatus)
{
    /* check paramter. */
    if(0xFF == _u16MbxGroupIdMBXHAL[eSrcRole][eDstRole])
    {
        return E_MBX_ERR_INVALID_PARAM;
    }

    switch(eRecvSatus)
    {
        case E_MBXHAL_RECV_OVERFLOW:
            //Set OverFlow & Error:
            _OVERFLOW_S(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]);
            _ERR_S(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]);
            mb();
            break;
        case E_MBXHAL_RECV_DISABLED:
            //Set Disable & Error:
            _DISABLED_S(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]);
            _ERR_S(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]);
            mb();
            break;
        default:
            break;
    }

    /* clear busy bit */
    _BUSY_C(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]);
    /* clear send bit */
    _FIRE_C(_u16MbxGroupIdMBXHAL[eSrcRole][eDstRole]);
    mb();

    return E_MBX_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
/// Get register group for specified src and dst role
/// @param  eSrcRole                  \b IN: The Recv Src RoleID
/// @param  eDstRole                  \b IN: The Recv Dst RoleID
/// @return Register group
/// @return
/// @attention
/// <b>[MXLIB] <em> </em></b>
//-------------------------------------------------------------------------------------------------
MS_U16 MHAL_MBX_RegGroup(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole)
{
    return _u16MbxGroupIdMBXHAL[eSrcRole][eDstRole];
}

MS_U8 MHAL_PM_Get_BrickTerminator_Info(void)
{
    return MHAL_PM_REG();
}

void MHAL_PM_Set_BrickTerminator_Info(MS_U8 u8Value)
{
    MHAL_PM_REG() = u8Value;
}

