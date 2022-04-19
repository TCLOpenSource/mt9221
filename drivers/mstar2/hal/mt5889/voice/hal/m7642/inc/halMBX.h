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
/// @file   halMBX.h
/// @brief  MStar Mailbox HAL Driver DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _HAL_MBX_H
#define _HAL_MBX_H

#ifdef _HAL_MBX_C
#define INTERFACE
#else
#define INTERFACE extern
#endif

//=============================================================================
// Includs
//=============================================================================

//=============================================================================
// Defines & Macros

//=============================================================================
//busy bit Set/Clear/Get
#define   _BUSY_S(arg)  {\
                            U8 val; \
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_BUSY;\
                         }

#define   _BUSY_C(arg)  {\
                            U8 val; \
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_BUSY;\
                         }

#define   _BUSY(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_BUSY);

//////////////////////////////////////////////////////////////
//error bit Set/Clear/Get
#define   _ERR_S(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_ERROR;\
                         }

#define   _ERR_C(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_ERROR;\
                         }

#define   _ERR(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_ERROR)

//////////////////////////////////////////////////////////////
//disabled bit Set/Clear/Get
#define   _DISABLED_S(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_DISABLED;\
                         }

#define   _DISABLED_C(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_DISABLED;\
                         }

#define   _DISABLED(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_DISABLED)

////////////////////////////////////////////////////////////////////////
//overflow bit Set/Clear/Get
#define   _OVERFLOW_S(arg)  {\
                                U8 val;\
                                val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                                REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_OVERFLOW;\
                              }

#define   _OVERFLOW_C(arg)  {\
                                U8 val;\
                                val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                                REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_OVERFLOW;\
                              }

#define   _OVERFLOW(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_OVERFLOW)

////////////////////////////////////////////////////////////////////////
//status bit clear
#define   _S1_C(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~(MBX_STATE1_DISABLED | MBX_STATE1_OVERFLOW | MBX_STATE1_ERROR | MBX_STATE1_BUSY);\
                        }

////////////////////////////////////////////////////////////////////////
//fire bit Set/Clear/Get
#define   _FIRE_S(arg)  {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
                            REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_FIRE;\
                         }

#define   _FIRE_C(arg)  {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
                            REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_FIRE;\
                         }

#define   _FIRE(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_FIRE)

////////////////////////////////////////////////////////////////////////
//readback bit Set/Clear/Get
#define   _READBACK_S(arg)   {\
                                  U8 val;\
                                  val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
                                  REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_READBACK;\
                               }

#define   _READBACK_C(arg)   {\
                                  U8 val;\
                                  val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
                                  REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_READBACK;\
                               }

#define   _READBACK(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_READBACK)

////////////////////////////////////////////////////////////////////////
//instant bit Set/Clear/Get
#define   _INSTANT_S(arg)   {\
                                  U8 val;\
                                  val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
                                  REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_INSTANT;\
                              }

#define   _INSTANT_C(arg)   {\
                                  U8 val;\
                                  val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
                                  REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_INSTANT;\
                              }

#define   _INSTANT(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_INSTANT)



//////////////////////////////////////////////////////////////
//state0 error bit Set/Clear/Get
#define   _STATE0_ERR_S(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_0);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_0) = val | MBX_STATE0_ERROR;\
                         }

#define   _STATE0_ERR_C(arg)   {\
                            U8 val;\
                            val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_0);\
                            REG8_MBX_GROUP(arg, REG8_MBX_STATE_0) = val & ~MBX_STATE0_ERROR;\
                         }

#define   _STATE0_ERR(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_0) & MBX_STATE0_ERROR)

//=============================================================================
// Type and Structure Declaration
//=============================================================================

//=============================================================================
// Enums
/// MBX HAL Recv Status Define
typedef enum
{
    /// Recv Success
    E_MBXHAL_RECV_SUCCESS = 0,
    /// Recv Error: OverFlow
    E_MBXHAL_RECV_OVERFLOW = 1,
    /// Recv Error: Not Enabled
    E_MBXHAL_RECV_DISABLED = 2,
} MBXHAL_Recv_Status;

/// MBX HAL Fire Status Define
typedef enum
{
    /// Fire Success
    E_MBXHAL_FIRE_SUCCESS = 0,
    /// Still Firing
    E_MBXHAL_FIRE_ONGOING = 1,
    /// Fire Error: Overflow:
    E_MBXHAL_FIRE_OVERFLOW = 2,
    /// Fire Error: Not Enabled
    E_MBXHAL_FIRE_DISABLED = 3,
} MBXHAL_Fire_Status;

/// MBX HAL Fire Cmd Status Define
typedef enum
{
    /// Cmd Success
    E_MBXHAL_CMD_SUCCESS = 0,
    /// Cmd Error
    E_MBXHAL_CMD_ERROR = 1,
} MBXHAL_Cmd_Status;

//=============================================================================
// Mailbox HAL Driver Function
//=============================================================================

INTERFACE MBX_Result MHAL_VOC_MBX_ClearAll (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole);

INTERFACE MBX_Result MHAL_VOC_MBX_Init(MBX_ROLE_ID eHostRole);
INTERFACE MBX_Result MHAL_VOC_MBX_SetConfig(MBX_ROLE_ID eHostRole);

INTERFACE MBX_Result MHAL_VOC_MBX_Fire(MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole);
INTERFACE void MHAL_VOC_MBX_ClearStatus (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole);
INTERFACE MBX_Result MHAL_VOC_MBX_GetFireStatus(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Fire_Status *pFireStatus);
INTERFACE MBX_Result MHAL_VOC_MBX_GetCmdStatus(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Cmd_Status *pCmdStatus);

INTERFACE MBX_Result MHAL_VOC_MBX_Recv(MBX_Msg* pMbxMsg, MBX_ROLE_ID eDstRole);
INTERFACE MBX_Result MHAL_VOC_MBX_RecvEnd(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Recv_Status eRecvSatus);
INTERFACE MBX_Result MHAL_VOC_MBX_RecvEndExt(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, BOOL bSuccess);

#undef INTERFACE
#endif //_HAL_MBX_H

