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
/// file    regMIU.h
/// @brief  MIU Control Register Definition
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _REG_MIU_H_
#define _REG_MIU_H_


//-------------------------------------------------------------------------------------------------
//  Hardware Capability
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define BIT(x) (1UL << (x))
#define BITS_RANGE(range)                       (BIT(((1)?range)+1) - BIT((0)?range))
#define BITS_RANGE_VAL(x, range)                ((x & BITS_RANGE(range)) >> ((0)?range))

#define MIU_REG_BASE                            (0x1200UL)
#define MIU1_REG_BASE                           (0x0600UL)
#define MIU2_REG_BASE                           (0x62000UL)
#define PM_REG_BASE                             (0x1E00UL)
#define MIU_ATOP_BASE                           (0x10D00UL)
#define MIU1_ATOP_BASE                          (0x61600UL)
#define MIU2_ATOP_BASE                          (0x62100UL)
#define CHIP_TOP_BASE                           (0x1E00UL)
#define MIU_ARB_REG_BASE                        (0x61500UL)
#define MIU1_ARB_REG_BASE                       (0x62200UL)
#define MIU2_ARB_REG_BASE                       (0x62300UL)
#define MIU_ARBB_REG_BASE                       (0x52000UL)
#define MIU1_ARBB_REG_BASE                      (0x52100UL)
#define MIU2_ARBB_REG_BASE                      (0x52200UL)

#define MIU_RQ                                  (MIU_REG_BASE+0x40UL)

#define MIU_PROTECT_EN                          (MIU_REG_BASE+0xD2UL)
#define MIU_PROTECT_DDR_SIZE                    (MIU_REG_BASE+0xD3UL)
#define MIU_PROTECT_DDR_SIZE_MASK               BITS_RANGE(11:8)
#define MIU_PROTECT_DDR_32MB                    (0x50UL)
#define MIU_PROTECT_DDR_64MB                    (0x60UL)
#define MIU_PROTECT_DDR_128MB                   (0x70UL)
#define MIU_PROTECT_DDR_256MB                   (0x80UL)
#define MIU_PROTECT_DDR_512MB                   (0x90UL)
#define MIU_PROTECT_DDR_1024MB                  (0xA0UL)
#define MIU_PROTECT_DDR_2048MB                  (0xB0UL)

#define MIU_PROTECT0_ID0                        (MIU_REG_BASE+0x2EUL)
#define MIU_PROTECT_EN_INTERNAL                 (MIU_REG_BASE+0xD2UL)
#define MIU_PROTECT0_ID_ENABLE                  (MIU_REG_BASE+0x20UL)
#define MIU_PROTECT0_START                      (MIU_REG_BASE+0xC0UL)

#define REG_MIU_PROTECT_LOADDR                  (0x6DUL << 1)	//0xDE
#define REG_MIU_PROTECT_HIADDR                  (0x6EUL << 1)	//0xDE
#define REG_MIU_GROUP_PRIORITY                  (0x7FUL << 1)
#define REG_MIU_PROTECT_STATUS                  (0x6FUL << 1)	//0xDE

// MIU selection registers
#define REG_MIU_SEL0                            (MIU_REG_BASE+0xf0UL)  //0x12F0
#define REG_MIU_SEL1                            (MIU_REG_BASE+0xf2UL)  //0x12F1
#define REG_MIU_SEL2                            (MIU_REG_BASE+0xf4UL)  //0x12F2
#define REG_MIU_SEL3                            (MIU_REG_BASE+0xf6UL)  //0x12F3
#define REG_MIU_SEL6                            (MIU_ARBB_REG_BASE+0xF0UL)  //0x615F0

//MIU1
#define MIU1_PROTECT_EN                          (MIU1_REG_BASE+0xD2UL)
#define MIU1_PROTECT_DDR_SIZE                    (MIU1_REG_BASE+0xD3UL)
#define MIU1_PROTECT0_ID0                        (MIU1_REG_BASE+0x2EUL)
#define MIU1_PROTECT0_ID_ENABLE                  (MIU1_REG_BASE+0x20UL)
#define MIU1_PROTECT0_START                      (MIU1_REG_BASE+0xC0UL)
#define MIU1_RQ                                  (MIU1_REG_BASE+0x40UL)

// MIU1 selection registers
#define REG_MIU1_SEL0                            (MIU1_REG_BASE+0xf0UL)  //0x6F0
#define REG_MIU1_SEL6                            (MIU1_ARBB_REG_BASE+0xF0UL)  //0x622F0

//Protection Status
#define REG_MIU_PROTECT_LOG_CLR                 (BIT0)
#define REG_MIU_PROTECT_HIT_FALG                (BIT4)
#define REG_MIU_PROTECT_HIT_ID                  14:8
#define REG_MIU_PROTECT_HIT_NO                  7:5

//MIU Bus Width
#define MIU_BW_CTRL                             (MIU_ARBB_REG_BASE+0xE0UL)  //0x70
#define MIU1_BW_CTRL                             (MIU1_ARBB_REG_BASE+0xE0UL)  //0x70

#endif // _REG_MIU_H_
