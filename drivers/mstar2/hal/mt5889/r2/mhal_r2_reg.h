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

#ifndef _REG_R2_H_
#define _REG_R2_H_

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
/* From linux. */
#include <linux/types.h>

/* From mstar. */
#include "mdrv_types.h"
#include "mdrv_r2.h"
#include "mhal_r2.h"

//--------------------------------------------------------------------------------------------------
//  Forward declaration
//--------------------------------------------------------------------------------------------------
extern ptrdiff_t mstar_pm_base;

#if defined(CONFIG_MSTAR_FRC_R2)
#if defined(CONFIG_MP_PLATFORM_FRC_MAPPING)
extern ptrdiff_t mstar_frc_base;
#else
#error "CONFIG_MSTAR_FRC_R2 depend on CONFIG_MP_PLATFORM_FRC_MAPPING"
#endif
#endif

//-------------------------------------------------------------------------------------------------
//  Base
//-------------------------------------------------------------------------------------------------
#define BASE_PM                         (mstar_pm_base)
#define BASE_NON_PM                     (mstar_pm_base + 0x200000UL)
#define BASE_NON_PM_OFFSET              (0x1000UL)

#if defined(CONFIG_MSTAR_FRC_R2)
#define BASE_FRC_R2                     (mstar_frc_base)
#define BASE_FRC_R2_OFFSET              (0x4000UL)
#endif

//-------------------------------------------------------------------------------------------------
//  Bank
//-------------------------------------------------------------------------------------------------
#define BANK_CLKGEN1                    (0x1033UL - BASE_NON_PM_OFFSET)
#define BANK_FRC_R2                     (0x4007UL - BASE_FRC_R2_OFFSET)
#define BANK_FRC_MAU                    (0x4008UL - BASE_FRC_R2_OFFSET)
#define BANK_FRC_MAU2                   (0x4009UL - BASE_FRC_R2_OFFSET)
#define BANK_FRC_UMA                    (0x400BUL - BASE_FRC_R2_OFFSET)

//-------------------------------------------------------------------------------------------------
//  Offset
//-------------------------------------------------------------------------------------------------
#define REG_CLKGEN1_FRC_R2              ((BANK_CLKGEN1  << 8) + (0x0030UL << 1) + 0)
#define REG_CLKGEN1_FRC_MCU             ((BANK_CLKGEN1  << 8) + (0x0030UL << 1) + 1)

#define REG_FRC_R2_STOP                 ((BANK_FRC_R2   << 8) + (0x0040UL << 1) + 0)
#define REG_FRC_R2_SDR_LO_INST_BASE     ((BANK_FRC_R2   << 8) + (0x0041UL << 1) + 0)
#define REG_FRC_R2_SDR_HI_INST_BASE     ((BANK_FRC_R2   << 8) + (0x0042UL << 1) + 0)
#define REG_FRC_R2_SDR_LO_DATA_BASE     ((BANK_FRC_R2   << 8) + (0x0043UL << 1) + 0)
#define REG_FRC_R2_SDR_HI_DATA_BASE     ((BANK_FRC_R2   << 8) + (0x0044UL << 1) + 0)
#define REG_FRC_R2_RIU_BASE             ((BANK_FRC_R2   << 8) + (0x0045UL << 1) + 0)
#define REG_FRC_R2_SPI_BASE             ((BANK_FRC_R2   << 8) + (0x0048UL << 1) + 0)
#define REG_FRC_R2_DQMEM_BASE           ((BANK_FRC_R2   << 8) + (0x004EUL << 1) + 0)
#define REG_FRC_R2_QMEM_MASK_HIGH       ((BANK_FRC_R2   << 8) + (0x0050UL << 1) + 0)
#define REG_FRC_R2_IO1_BASE             ((BANK_FRC_R2   << 8) + (0x0055UL << 1) + 0)
#define REG_FRC_R2_SPI_BASE1            ((BANK_FRC_R2   << 8) + (0x0056UL << 1) + 0)
#define REG_FRC_R2_SPACE_EN             ((BANK_FRC_R2   << 8) + (0x0058UL << 1) + 0)

#define REG_FRC_R2_MAU                  ((BANK_FRC_MAU  << 8) + (0x0001UL << 1) + 0)
#define REG_FRC_R2_MAU_LV2              ((BANK_FRC_MAU2 << 8) + (0x0001UL << 1) + 0)

#define REG_FRC_R2_UMA_REMAP_DATA_2_LO  ((BANK_FRC_UMA  << 8) + (0x000CUL << 1) + 0)
#define REG_FRC_R2_UMA_REMAP_DATA_2_HI  ((BANK_FRC_UMA  << 8) + (0x000DUL << 1) + 0)
#define REG_FRC_R2_UMA_REMAP_DATA_3_LO  ((BANK_FRC_UMA  << 8) + (0x000EUL << 1) + 0)
#define REG_FRC_R2_UMA_REMAP_DATA_3_HI  ((BANK_FRC_UMA  << 8) + (0x000FUL << 1) + 0)
#endif
