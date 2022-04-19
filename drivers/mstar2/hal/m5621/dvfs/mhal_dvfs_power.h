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

#ifndef __MHAL_DVFS_POWER_H__
#define __MHAL_DVFS_POWER_H__

#ifndef __MDRV_TYPES_H__
#include "mdrv_types.h"
#endif

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define CONFIG_DVFS_CPU_POWER_I2C_ENABLE        1
#define CONFIG_DVFS_CORE_POWER_I2C_ENABLE       0
#define CONFIG_DVFS_CPU_POWER_GPIO_ENABLE       1
#define CONFIG_DVFS_CORE_POWER_GPIO_ENABLE      0

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_SWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_SWIIC == 1)))
#define CONFIG_DVFS_POWER_I2C_SET               0x00    // SWI2C BUS0
#else
#define CONFIG_DVFS_POWER_I2C_SET               0x03    // HWI2C Port
#endif

#define CONFIG_DVFS_CPU_POWER                   0
#define CONFIG_DVFS_CORE_POWER                  1
#define CONFIG_DVFS_STR_INIT                    0xFE

#if (CONFIG_DVFS_CPU_POWER_I2C_ENABLE)
#define CONFIG_DVFS_POWER_I2C_ADDR_CPU          0x80    //0x80

#define CONFIG_DVFS_CPU_POWER_SHIFT_PRADO       (69 -1)
#define CONFIG_DVFS_CPU_POWER_DEFAULT           110

#define DVFS_LOW_BOUND_VOLTAGE                  85
#define DVFS_HIGH_BOUND_VOLTAGE                 130
#define CONFIG_DVFS_CPU_POWER_MAX               (DVFS_HIGH_BOUND_VOLTAGE+2)
#define CONFIG_DVFS_CPU_POWER_STEP              3
#endif

#if (CONFIG_DVFS_CORE_POWER_I2C_ENABLE)
#define CONFIG_DVFS_POWER_I2C_ADDR_CORE         0x82    //0x82
#define CONFIG_DVFS_CORE_POWER_SHIFT_PRADO      (69 - 2)
#define CONFIG_DVFS_CORE_POWER_SHIFT            94
#define CONFIG_DVFS_CORE_POWER_DEFAULT          100
#endif

#define CONFIG_DVFS_CHIP_ID_UNKNOWN             0xFF
#define CONFIG_DVFS_CHIP_ID_PRADA               0x79
#define CONFIG_DVFS_CHIP_ID_PRADO               0x9A

#define DVFS_GPIO_IN             0
#define DVFS_GPIO_OUTPUT_LOW     1
#define DVFS_GPIO_OUTPUT_HIGH    2
#define DVFS_GPIO_NONE           3
//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
U32 SysDvfsPowerInit(U32 dwCluster);
U32 SysDvfsCpuPowerInit(void);
U32 SysDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage,U32 dwCluster);
U32 SysDvfsCorePowerInit(void);
U32 SysDvfsCorePowerAdjustment(U32 dwCorePowerVoltage, U32 dwCluster);

#endif // __MHAL_DVFS_POWER_H__

