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
/// @file   mhal_mbx.h
/// @brief  MStar Mailbox Driver DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b>(OBSOLETED) <em>legacy interface is only used by MStar proprietary Mail Message communication\n
/// It's API level for backward compatible and will be remove in the next version.\n
/// Please refer @ref drvGE.h for future compatibility.</em></b>
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
#include "mhal_mbx_reg.h"

//=============================================================================
// Defines & Macros
//=============================================================================
//busy bit Set/Clear/Get
#define   _BUSY_S(arg)  {\
MS_U8 val; \
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_BUSY;\
mb();\
}

#define   _BUSY_C(arg)  {\
MS_U8 val; \
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_BUSY;\
mb();\
}


#define   _BUSY(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_BUSY);

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _BUSY_S_FRC(arg)  {\
MS_U8 val; \
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_BUSY;\
mb();\
}

#define   _BUSY_C_FRC(arg)  {\
MS_U8 val; \
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_BUSY;\
mb();\
}


#define   _BUSY_FRC(arg)    (REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_BUSY);
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

//////////////////////////////////////////////////////////////
//error bit Set/Clear/Get
// Non PM group
#define   _ERR_S(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_ERROR;\
mb();\
}

#define   _ERR_C(arg)    {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_ERROR;\
mb();\
}


#define   _ERR(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_ERROR)

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
// FRC group
#define   _ERR_S_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_ERROR;\
mb();\
}

#define   _ERR_C_FRC(arg)    {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_ERROR;\
mb();\
}


#define   _ERR_FRC(arg)    (REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_ERROR)
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

//////////////////////////////////////////////////////////////
//disabled bit Set/Clear/Get
#define   _DISABLED_S(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_DISABLED;\
mb();\
}

#define   _DISABLED_C(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_DISABLED;\
mb();\
}

#define   _DISABLED(arg)    (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_DISABLED)

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _DISABLED_S_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_DISABLED;\
mb();\
}

#define   _DISABLED_C_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_DISABLED;\
mb();\
}

#define   _DISABLED_FRC(arg)    (REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_DISABLED)
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

////////////////////////////////////////////////////////////////////////
//overflow bit Set/Clear/Get

#define   _OVERFLOW_S(arg)  {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_OVERFLOW;\
mb();\
}


#define   _OVERFLOW_C(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_OVERFLOW;\
mb();\
}


#define   _OVERFLOW(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_OVERFLOW)

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _OVERFLOW_S_FRC(arg)  {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val | MBX_STATE1_OVERFLOW;\
mb();\
}


#define   _OVERFLOW_C_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val & ~MBX_STATE1_OVERFLOW;\
mb();\
}


#define   _OVERFLOW_FRC(arg)   (REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) & MBX_STATE1_OVERFLOW)
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

////////////////////////////////////////////////////////////////////////
//status bit clear
#define   _S1_C(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_STATE_1) = val & ~(MBX_STATE1_DISABLED | MBX_STATE1_OVERFLOW | MBX_STATE1_ERROR | MBX_STATE1_BUSY);\
mb();\
}

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _S1_C_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_STATE_1) = val & ~(MBX_STATE1_DISABLED | MBX_STATE1_OVERFLOW | MBX_STATE1_ERROR | MBX_STATE1_BUSY);\
mb();\
}
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

////////////////////////////////////////////////////////////////////////
//fire bit Set/Clear/Get

#define   _FIRE_S(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_FIRE;\
mb();\
}


#define   _FIRE_C(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_FIRE;\
mb();\
}


#define   _FIRE(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_FIRE)

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _FIRE_S_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_FIRE;\
mb();\
}


#define   _FIRE_C_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_FIRE;\
mb();\
}


#define   _FIRE_FRC(arg)   (REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_FIRE)
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

////////////////////////////////////////////////////////////////////////
//readback bit Set/Clear/Get
#define   _READBACK_S(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_READBACK;\
mb();\
}

#define   _READBACK_C(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_READBACK;\
mb();\
}


#define   _READBACK(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_READBACK)

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _READBACK_S_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_READBACK;\
mb();\
}

#define   _READBACK_C_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_READBACK;\
mb();\
}


#define   _READBACK_FRC(arg)   (REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_READBACK)
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

////////////////////////////////////////////////////////////////////////
//instant bit Set/Clear/Get
#define   _INSTANT_S(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_INSTANT;\
mb();\
}


#define   _INSTANT_C(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_INSTANT;\
mb();\
}

#define   _INSTANT(arg)   (REG8_MBX_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_INSTANT)

#ifdef CONFIG_MP_PLATFORM_FRC_MAPPING
#define   _INSTANT_S_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) = val | MBX_CTRL_INSTANT;\
mb();\
}


#define   _INSTANT_C_FRC(arg)   {\
MS_U8 val;\
mb();\
val = REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL);\
mb();\
REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) = val & ~MBX_CTRL_INSTANT;\
mb();\
}

#define   _INSTANT_FRC(arg)   (REG8_MBX_FRC_GROUP(arg, REG8_MBX_CTRL) & MBX_CTRL_INSTANT)
#endif /* CONFIG_MP_PLATFORM_FRC_MAPPING */

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

//=============================================================================
// Mailbox HAL Driver Function
//=============================================================================

INTERFACE MBX_Result MHAL_MBX_Init(MBX_ROLE_ID eHostRole);
INTERFACE MBX_Result MHAL_MBX_SetConfig(MBX_ROLE_ID eHostRole);

INTERFACE MBX_Result MHAL_MBX_SetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size);
INTERFACE MBX_Result MHAL_MBX_GetInformation(MBX_ROLE_ID eTargetRole, MS_U8 *pU8Info, MS_U8 u8Size);

INTERFACE MBX_Result MHAL_MBX_Fire(MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole);
INTERFACE void       MHAL_MBX_ClearStatus (MBX_Msg* pMbxMsg, MBX_ROLE_ID eSrcRole);
INTERFACE MBX_Result MHAL_MBX_GetFireStatus(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Fire_Status *pFireStatus);

INTERFACE MBX_Result MHAL_MBX_Recv(MBX_Msg* pMbxMsg, MBX_ROLE_ID eDstRole);
INTERFACE MBX_Result MHAL_MBX_RecvEnd(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole, MBXHAL_Recv_Status eRecvSatus);
INTERFACE MS_U16 MHAL_MBX_RegGroup(MBX_ROLE_ID eSrcRole, MBX_ROLE_ID eDstRole);

INTERFACE MS_U8 MHAL_PM_Get_BrickTerminator_Info(void);
INTERFACE void MHAL_PM_Set_BrickTerminator_Info(MS_U8 u8Value);

#undef INTERFACE
#endif //_HAL_MBX_H
