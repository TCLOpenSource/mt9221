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
/// @file   mhal_mbx_reg.h
/// @brief  MStar Mailbox Driver DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b>(OBSOLETED) <em>legacy interface is only used by MStar proprietary Mail Message communication\n
/// It's API level for backward compatible and will be remove in the next version.\n
/// Please refer @ref drvGE.h for future compatibility.</em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MHAL_MBX_REG_H
#define _MHAL_MBX_REG_H

//=============================================================================
// Includs
//=============================================================================
#include "mdrv_types.h"

//=============================================================================
// Defines & Macros
//=============================================================================

#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define RIU_MAP (mstar_pm_base+0x200000UL)
extern ptrdiff_t mstar_frc_base;
#define RIU_FRC_MAP (mstar_frc_base)
#else
extern ptrdiff_t mstar_pm_base;
#define RIU_MAP (mstar_pm_base+0x200000UL)
extern ptrdiff_t mstar_frc_base;
#define RIU_FRC_MAP (mstar_frc_base)
#endif

#define RIU8    ((unsigned char  volatile *) RIU_MAP)
#define RIU8_FRC    ((unsigned char  volatile *) RIU_FRC_MAP)

#define REG_MBX_BASE                (0x19C0UL) //(0x103380-0x100000)/2
#if 1//frcr2_integration###
//###############################
//
//  FRC MBX Dummy Register Allocation
//  Bank : 0x400300
//  Reg16_offset : 0x40 ~ 47 (HKCPU to FRCR2)
//  Reg16_offset : 0x48 ~ 4F (FRCR2 to HKCPU)
//
//###############################
#define REG_MBX_FRC_BASE            (0x1C0UL)//(0x1801C0UL) //(0x400380-0x100000)/2
#define REG_MBX_FRC_OFST            (REG_MBX_FRC_BASE - REG_MBX_BASE)
#endif
#define MBX_REG8(gid, addr)         RIU8[((gid+REG_MBX_BASE)<<2) + ((addr) * 2) - ((addr) & 1)]
#define MBX_FRC_REG8(gid, addr)     RIU8_FRC[((gid+REG_MBX_FRC_BASE)<<2) + ((addr) * 2) - ((addr) & 1)]

//Reg8 defines:
#define REG8_MBX_CTRL               0x0000UL
#define MBX_CTRL_FIRE             BIT(0)
#define MBX_CTRL_READBACK         BIT(1)
#define MBX_CTRL_INSTANT          BIT(2)
#define REG8_MBX_MAIL_CLASS          0x0001UL
#define REG8_MBX_MAIL_IDX            0x0002UL
#define REG8_MBX_PARAMETER_CNT       0x0003UL
#define REG8_MBX_PARAMETER_S         0x0004UL
#define REG8_MBX_PARAMETER_E         0x000DUL
#define REG8_MBX_STATE_0              0x000EUL
#define REG8_MBX_STATE_1              0x000FUL
#define MBX_STATE1_DISABLED          BIT(4)
#define MBX_STATE1_OVERFLOW           BIT(5)
#define MBX_STATE1_ERROR              BIT(6)
#define MBX_STATE1_BUSY               BIT(7)

#define REG_MBX_GROUP0      0x00UL
#define REG_MBX_GROUP1      0x08UL
#define REG_MBX_GROUP2      0x10UL
#define REG_MBX_GROUP3      0x18UL
#define REG_MBX_GROUP4      0x20UL
#define REG_MBX_GROUP5      0x28UL
#if 1//frcr2_integration###
#define REG_MBX_GROUP6      (0x00UL)// + REG_MBX_FRC_OFST)//(HKCPU to FRCR2)
#define REG_MBX_GROUP7      (0x08UL)// + REG_MBX_FRC_OFST)//(FRCR2 to HKCPU)
#endif

#define REG8_MBX_GROUP(gid, addr) MBX_REG8(gid, addr)
#define REG8_MBX_FRC_GROUP(gid, addr) 	MBX_FRC_REG8(gid, addr)

#define REG8_MBX_MIPS(addr)        MBX_REG8(REG_MBX_GROUP1, addr) //group0 allocated for mips
#define REG8_MBX_AEON(addr)        MBX_REG8(REG_MBX_GROUP0, addr) //group1 allocated for aeon
#define REG8_MBX_PM(addr)          MBX_REG8(REG_MBX_GROUP3, addr) //group3 allocated for pm

//  for MDrv_PM_Get_BrickTerminator_Info  (pm bank dummy register)
// 0x0E h0053 BIT[15:8]
#if defined(CONFIG_ARM64)
#define RIU_MAP_MBX (mstar_pm_base)
#else
#define RIU_MAP_MBX     0xFD000000UL
#endif
#define REG_PM_BASE   (0x0EA7UL)
#define MHAL_PM_REG()	((volatile MS_U8*)(RIU_MAP_MBX))[(REG_PM_BASE << 1) - (REG_PM_BASE & 1)]


#endif //__MHAL_MBX_REG_H
