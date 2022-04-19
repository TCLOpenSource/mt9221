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
/// @file   mhal_mbx_interrupt.h
/// @brief  MStar Mailbox Interrupt HAL DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b>(OBSOLETED) <em>legacy interface is only used by MStar proprietary Mail Message communication\n
/// It's API level for backward compatible and will be remove in the next version.\n
/// Please refer @ref drvGE.h for future compatibility.</em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MHAL_MBX_INTERRUPT_H
#define _MHAL_MBX_INTERRUPT_H

#ifdef _MHAL_MBX_INTERRUPT_C
#define INTERFACE
#else
#define INTERFACE extern
#endif

//=============================================================================
// Includs
//=============================================================================
#include "mhal_mbx_interrupt_reg.h"
#include "chip_int.h"

//=============================================================================
// Defines & Macros
//=============================================================================
#define E_FIQ_INT_AEON_TO_8051  (0xFF-0) 
#define E_FIQ_INT_MIPS_TO_8051   E_FIQEXPL_ARM_TO_8051 //VALID //E_FIQ_INT_ARM_TO_8051
#define E_FIQ_INT_8051_TO_AEON  (0xFF-1)
#define E_FIQ_INT_MIPS_TO_AEON   (0xFF-2)
#define E_FIQ_INT_8051_TO_MIPS   E_FIQEXPL_8051_TO_ARM //VALID //E_FIQ_INT_8051_TO_ARM
#define E_FIQ_INT_AEON_TO_MIPS   (0xFF-3)
#define E_FIQ_INT_FRCR2_TO_MIPS   (0xFF-4)//frcr2_integration###

#define MBX_INT_AEON2PM     E_FIQ_INT_AEON_TO_8051
#define MBX_INT_PM2AEON     E_FIQ_INT_8051_TO_AEON
#define MBX_INT_PM2MIPS     (E_FIQ_INT_8051_TO_MIPS-E_FIQEXPL_START)
#define MBX_INT_MIPS2PM     (E_FIQ_INT_MIPS_TO_8051-E_FIQEXPL_START)
#define MBX_INT_MIPS2AEON   E_FIQ_INT_MIPS_TO_AEON
#define MBX_INT_AEON2MIPS   E_FIQ_INT_AEON_TO_MIPS
#define MBX_INT_R2MIPS     (E_FIQEXPL_SECURE_R2_TO_ARM-E_FIQEXPH_START)
#define MBX_INT_FRC2MIPS   E_FIQ_INT_FRCR2_TO_MIPS //frcr2_integration###

//=============================================================================
// Type and Structure Declaration
//=============================================================================
typedef void (*MBX_MSGRECV_CB_FUNC)(MS_S32 s32Irq);

//=============================================================================
// Function
//=============================================================================
INTERFACE MBX_Result MHAL_MBXINT_Init (MBX_CPU_ID eHostCPU, MBX_MSGRECV_CB_FUNC pMBXRecvMsgCBFunc);
INTERFACE void MHAL_MBXINT_DeInit (MBX_CPU_ID eHostCPU);
INTERFACE MBX_Result MHAL_MBXINT_ResetHostCPU (MBX_CPU_ID ePrevCPU, MBX_CPU_ID eConfigCpu);
INTERFACE MBX_Result MHAL_MBXINT_Enable (MS_S32 s32Fiq, MS_BOOL bEnable);
INTERFACE MBX_Result MHAL_MBXINT_Fire (MBX_CPU_ID eDstCPUID, MBX_CPU_ID eSrcCPUID);
INTERFACE MBX_Result MHAL_MBXINT_Clear (MS_S32 s32Fiq);
INTERFACE MBX_Result MHAL_MBXINT_Suspend(MBX_CPU_ID eHostCPU);
INTERFACE MBX_Result MHAL_MBXINT_Resume(MBX_CPU_ID eHostCPU);
#undef INTERFACE

#endif //_MHAL_MBX_INTERRUPT_H

