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
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>

#include <mstar/mstar_chip.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/module.h>

#include "mdrv_dvfs.h"

#ifndef __MHAL_DVFS_H__
#include "mhal_dvfs.h"
#endif

#ifndef __MHAL_DVFS_POWER_H__
#include "mhal_dvfs_power.h"
#endif

#include "chip_dvfs_calibrating.h"

#include <linux/platform_device.h>
#include <linux/pm.h>
#include <asm/cputype.h>
#include <linux/slab.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
#include <linux/pm_opp.h>
#endif
#include <linux/cpu.h>
DEFINE_MUTEX(MDrvDvfsCpuTempMutex);
DEFINE_MUTEX(MHalDvfsGetCpuFreqMutex);
extern unsigned int get_cpu_midr(int cpu);
#define CONFIG_DVFS_CPU_CLOCK_DISPLAY_ENABLE 1
extern void MDrvDvfsVoltageSetup(unsigned int dwCpuClock, unsigned int dwVoltage, unsigned int dwVoltageType, unsigned int dwCpu);
int halTotalClusterNumber;

static U32 gDVFS_TEMP_OFFSET = 0;
static U32 gDVFS_auto_measurement=0;
static U16 u16CpuClockSet[DVFS_CLUSTER_NUM] = {0,0};

static volatile MSTAR_DVFS_REG_INFO *DvfsRegInfo = 0;
static MSTAR_DVFS_INFO hMstarDvfsInfo[DVFS_CLUSTER_NUM] =
{
    {
        .bDvfsInitOk                    = 0,
        .bDvfsModeChange                = 0,
        .dwMaxCpuClockByTemperature     = 0,

        .dwFinalCpuClock                = 0,
        .dwFinalCpuPowerVoltage         = 0,
        .dwFinalCorePowerVoltage        = 0,

        .dwCpuTemperature               = 0,
        .dwRefTemperature               = 0,
        .dwAvgCpuTempCounter            = 0,

        .bSystemResumeFlag              = 0,
        .dwResetCounter                 = 0,
        .dwTemperatureCounter           = 0,
        .dwBootTimeCounter              = 0,
        .dwOverTemperatureFlag          = 0,

        .dwCpuPartId                    = 0xd03,        //CA53 PartNum
        .dwClusterCpuMask               = 0x0,          //This will be used in classify_cpu_cluster for cluster mask.
        .dwFreqRiuAddr                  = 0x1f200a04,   //bus address
        .dwSpecialCpuClkddr             = 0x1f200a0c,   //bus address
        .dwAnaMiscBank                  = 0x110c00,
        .dwMcuArmBank                   = 0x101d00,
        .dwPradoProtect                 = 0,
        .dwPradoCounter                 = 0,

        .DvfsModeInfo =
        {
            //CA53
            {
                //Normal
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1000,
                    .dwMinimumCpuClock          = 1000,
                    .dwMaximumCpuClock          = 1300,
                    .dwIRBoostCpuClock          = 1200,
                    .dwAntutuProtectedCpuClock  = 1300,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 105,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1000,
                        .dwUpperCpuClock        = 1000,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1200,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1301MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1301,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1301MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1301,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // ENG6 HCP+
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 100,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1200,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 108,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // ENG6 HCP
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 100,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1400,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 108,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },

                },
            },
            {
                // ENG4 HCP
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1000,
                    .dwMinimumCpuClock          = 1000,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 100,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1000,
                        .dwUpperCpuClock        = 1000,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1200,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1000MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1000MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // raptor
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 216,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 100,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1400,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 108,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 216,
                        .dwUpperCpuClock        = 216,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 92,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 92,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 600MHz - 1000MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 1000,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 600MHz - 1000MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 1000,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // amazon
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 100,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1400,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 113,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 108,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 600MHz - 1000MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 1000,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 600MHz - 1000MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 1000,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
        },
    },
    {
        .bDvfsInitOk                    = 0,
        .bDvfsModeChange                = 0,

        .dwMaxCpuClockByTemperature     = 0,
        .dwFinalCpuClock                = 0,

        .dwFinalCpuPowerVoltage         = 0,
        .dwFinalCorePowerVoltage        = 0,

        .dwCpuTemperature               = 0,
        .dwRefTemperature               = 0,
        .dwAvgCpuTempCounter            = 0,

        .bSystemResumeFlag              = 0,
        .dwResetCounter                 = 0,
        .dwTemperatureCounter           = 0,
        .dwBootTimeCounter              = 0,
        .dwOverTemperatureFlag          = 0,

        .dwCpuPartId                    = 0xd08, //CA72
        .dwClusterCpuMask               = 0x0, //This will be used in classify_cpu_cluster for cluster mask.
        .dwFreqRiuAddr                  = 0x1f200a14, //bus address
        .dwSpecialCpuClkddr             = 0x1f200a1c, //bus address
        .dwAnaMiscBank                  = 0x111600,
        .dwMcuArmBank                   = 0x110900,
        .dwPradoProtect                 = 0,
        .dwPradoCounter                 = 0,

        .DvfsModeInfo =
        {
            //CA72
            {
                //Normal
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 850,
                    .dwMinimumCpuClock          = 850,
                    .dwMaximumCpuClock          = 1100,
                    .dwIRBoostCpuClock          = 1000,
                    .dwAntutuProtectedCpuClock  = 1100,
                    .dwProtectedCpuClock        = 750,
                    .dwDefaultCpuPower          = 105,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 950,
                        .dwUpperCpuClock        = 1000,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 750,
                        .dwUpperCpuClock        = 750,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 850MHz - 950MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 850,
                            .dwUpperCpuClock    = 950,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 950MHz - 1000MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 950,
                            .dwUpperCpuClock    = 1000,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1100MHz - 1101MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1101,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 850MHz - 950MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 850,
                            .dwUpperCpuClock    = 950,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 950MHz - 1000MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 950,
                            .dwUpperCpuClock    = 1000,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1000MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1100MHz - 1101MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1101,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // ENG6 HCP+
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1740,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1700,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 103,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 98,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1200,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1740MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1740,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1740MHz - 1741MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1740,
                            .dwUpperCpuClock    = 1741,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 140,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 140,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 135,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1740MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1740,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1740MHz - 1741MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1740,
                            .dwUpperCpuClock    = 1741,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 140,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 140,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 135,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // ENG6 HCP
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1900,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1700,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 103,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 110,
                    .dwUpperLevelTemperature    = 125,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 98,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1400,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1700MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1700,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1900MHz - 1901MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1700,
                            .dwUpperCpuClock    = 1901,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 150,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 150,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 150,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1700MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1700,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1700MHz - 1701MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1700,
                            .dwUpperCpuClock    = 1901,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 135,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 135,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 130,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // ENG4 HCP
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1000,
                    .dwMinimumCpuClock          = 1000,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 103,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1000,
                        .dwUpperCpuClock        = 1000,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 98,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1200,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 125,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 125,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 120,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1000MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 125,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 125,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1000MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1000,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 125,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 125,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // raptor
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 216,
                    .dwMaximumCpuClock          = 1700,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1700,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 103,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 98,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1400,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 216,
                        .dwUpperCpuClock        = 216,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 92,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 92,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 600MHz - 900MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 900,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 900MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 900,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1700MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1700,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1900MHz - 1901MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1700,
                            .dwUpperCpuClock    = 1901,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 150,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 150,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 150,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 600MHz - 900MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 900,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 900MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 900,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1700MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1700,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1700MHz - 1701MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1700,
                            .dwUpperCpuClock    = 1701,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 135,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 135,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 130,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
            {
                // amazon
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 103,
                    .dwDefaultCorePower         = 97,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE, //CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature    = 120,
                    .dwUpperLevelTemperature    = 135,
                    .dwMaxLevelTemperature      = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1100,
                        .dwUpperCpuClock        = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 103,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 98,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1400,
                        .dwUpperCpuClock        = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsLightModeInfo =
                {
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                        .dwUpperCpuClock        = CONFIG_DVFS_LIGHTMODE_DISABLE,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 85,
                            .dwCorePower        = 85,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 850,
                        .dwUpperCpuClock        = 850,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 550,
                        .dwUpperGpuClock        = 550,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 92,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 600MHz - 900MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 900,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 900MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 900,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 216MHz - 400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 216,
                            .dwUpperCpuClock    = 400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 400MHz - 600MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 400,
                            .dwUpperCpuClock    = 600,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 600MHz - 900MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 600,
                            .dwUpperCpuClock    = 900,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 85,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 900MHz - 1100MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 900,
                            .dwUpperCpuClock    = 1100,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1100MHz - 1200MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1200,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 98,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1300MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1200,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 103,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1400MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1400,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 113,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 108,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1401MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1401,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 550,
                            .dwUpperGpuClock    = 550,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 92,
                            },
                        },
                    },
                },
            },
        },
    },
};

#define DVFS_HAL_TOTAL_CLUSTER_NUM  (sizeof(hMstarDvfsInfo)/sizeof(MSTAR_DVFS_INFO))


//=================================================================================================
int getCpuCluster(unsigned int cpu)
{
    U8 cluster = 0;
    for (cluster = 0; cluster < DVFS_HAL_TOTAL_CLUSTER_NUM; cluster ++)
    {
        if (hMstarDvfsInfo[cluster].dwClusterCpuMask & (0x01 << cpu))
        {
            return cluster;
        }
    }

    DVFS_HAL_INFO("%s %d: ERROR can't find cluster:%d\n",__func__,__LINE__, cluster);
    return cluster;
}

//=================================================================================================
int getClusterMainCpu(unsigned int cpu)
{
    int cluster = getCpuCluster(cpu);
    int i = 0;
    for (i = 0; i < CONFIG_NR_CPUS; i ++)
    {
        if (hMstarDvfsInfo[cluster].dwClusterCpuMask >> i & 0x1)
        {
            return i;
        }
    }

    DVFS_HAL_INFO("%s %d: cpu:%d ERROR can't find master cpu\n",__func__,__LINE__, cpu);
    //only one cluster, return 0
    return 0;
}

//=================================================================================================
U32 getFreqRiuAddr(unsigned int cpu)
{
    int cluster = getCpuCluster(cpu);

    return hMstarDvfsInfo[cluster].dwFreqRiuAddr;
}

//=================================================================================================
U32 MHalDvfsProc(U32 dwCpuClock, U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    //Check Specific Register to Check DVFS Running State (0x1005_00 = 0x3697)
    if(DvfsRegInfo->dvfs_reg[dwCluster].reg_vid_dvfs_id == CONFIG_DVFS_ENABLE_PATTERN)
    {
        if(hMstarDvfsInfo[dwCluster].bDvfsInitOk == 0)
        {
            //Initial DVFS Default Settings and Data Structure
            if(MHalDvfsInit(dwCluster) == TRUE)
            {
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_INIT_MODE;
            }
            else
            {
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
                goto _MHalDvfsProcExit;
            }
        }

        //Get CPU Temperature by PM_SAR
        //MHalDvfsCpuTemperature(dwCluster);

        if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
        {
            //Initial Mode
            //Use Default CPU Clock in Init Mode
            MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock, dwCpu);
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
            {
                //Change to Over-Temperature Mode
                hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
            }
            else
            {
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_BOOT_MODE;  //CONFIG_DVFS_NORMAL_MODE;
            }

            hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
        }
        else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
        {
            //Boot Mode
            if(hMstarDvfsInfo[dwCluster].dwBootTimeCounter > CONFIG_DVFS_BOOT_MODE_TIME)
            {
                //Change to Normal Mode
                hMstarDvfsInfo[dwCluster].dwBootTimeCounter = 0;

                //Use Default CPU Clock in Boot Mode
                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwLowerCpuClock, dwCpu);

                //Change to Normal Mode
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwLowerCpuClock;
            }
            else
            {
                //Use Maximum CPU Clock in Boot Mode
                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwUpperCpuClock, dwCpu);
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwUpperCpuClock;
            }
        }
        else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
        {
            //Over-Temperature Mode
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwMaxLevelTemperature)
            {
                //Retry 10 Times to Confirm the State of Over Temperature
                if(hMstarDvfsInfo[dwCluster].dwResetCounter < CONFIG_DVFS_RESET_MAX_COUNT)
                {
                    DVFS_HAL_DEBUG("\033[1;31m[DVFS] Over Temperature Protection: Count = %d / Temperature = %d\033[0m\n",
                        (unsigned int) hMstarDvfsInfo[dwCluster].dwResetCounter, (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                    hMstarDvfsInfo[dwCluster].dwResetCounter ++;
                }
                else
                {
                    DVFS_HAL_INFO("\033[1;31m[DVFS] Current Temperature: %d\033[0m\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                    DVFS_HAL_INFO("\033[1;31m[DVFS] Over Temperature Mode: SYSTEM RESET\033[0m\n");

                    //Trigger a WDT Reset
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x00;
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003008 << 1)) = 0x00;
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x05;
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003000 << 1)) = 0x01;

                    while(1);
                }
            }
//            else if(hMstarDvfsInfo[dwCluster].dwCpuTemperature <
//                    hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerLevelTemperature)
//            {
                //Return to Normal Mode
//                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

//                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
//                DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);

//                MHalDvfsCpuClockAdjustment(dwCpuClock, dwCpu);
//                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwCpuClock;

//                hMstarDvfsInfo[dwCluster].dwResetCounter = 0;
//            }
            else
            {
                //Keep at Over-Temperature Mode
                hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;

                DVFS_HAL_DEBUG("\033[1;31m[DVFS] Current Temperature: %d\033[0m\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("\033[1;31m[DVFS] Over-Temperature Mode: CPU Clock: %dMHz\033[0m\n", hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);

                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock, dwCpu);
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;

                hMstarDvfsInfo[dwCluster].dwResetCounter = 0;
            }
        }
        else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_FREEZE_MODE)
        {
            //Freeze Mode
            if((hMstarDvfsInfo[dwCluster].dwCpuTemperature >= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
               (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
            {
                //Return to Normal Mode
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);
            }
            else
            {
                //Keep at Freeze Mode
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Freeze Mode: CPU Clock: %dMHz\n", dwCpuClock);
            }

            //dwCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;

            MHalDvfsCpuClockAdjustment(dwCpuClock, dwCpu);
            hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwCpuClock;
        }
        else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_LIGHT_MODE)
        {
            DVFS_HAL_INFO("\033[32m[DVFS] eason.lien [%s, %d]\033[0m\n", __FUNCTION__, __LINE__);
            //Light Mode
            MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsCpuInfo.dwLowerCpuClock, dwCpu);
/*
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
            {
                //Change to Over-Temperature Mode
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
            }
            else
            {
                if (DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk == CONFIG_DVFS_CPU_CLOCK_LIGHT_MODE)
                {
                    DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_LIGHT_MODE;
                }
                else
                {
                    DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                }
            }
*/
            hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsCpuInfo.dwLowerCpuClock;

        }
        else
        {
            //Normal Mode
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
            {
                //Change to Over-Temperature Mode
                hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Over-Temperature Mode: CPU Clock: %dMHz\n", hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);

                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock, dwCpu);

                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
            }
            else if((hMstarDvfsInfo[dwCluster].dwCpuTemperature < hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature) && \
                    (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
            {
                //Change to Freeze Mode
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Freeze Mode: CPU Clock: %dMHz\n", dwCpuClock);

                MHalDvfsCpuClockAdjustment(dwCpuClock, dwCpu);
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwCpuClock;
            }
            else
            {
                if(DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk != CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT)
                {
                    U32 dwRegisterValue = 0;
                    dwRegisterValue = DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk;
                    if ((dwRegisterValue == CONFIG_DVFS_CPU_CLOCK_LIGHT_MODE)  && \
                        (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsCpuInfo.dwLowerCpuClock != CONFIG_DVFS_LIGHTMODE_DISABLE))
                    {
                        DVFS_HAL_INFO("\033[32m[DVFS] eason.lien [%s, %d] \033[0m\n", __FUNCTION__, __LINE__);
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_LIGHT_MODE;
                        MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsCpuInfo.dwLowerCpuClock, dwCpu);
                        hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwRegisterValue;

                    }
                    else if((dwRegisterValue >= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMinimumCpuClock) && \
                            (dwRegisterValue <= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock))
                    {
                        //Special Clock Mode
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_SPECIAL_CLOCK_MODE;

                        DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                        DVFS_HAL_DEBUG("[DVFS] Special Clock Mode: CPU Clock: %dMHz\n", dwRegisterValue);

                        MHalDvfsCpuClockAdjustment(dwRegisterValue, dwCpu);

                        DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                        hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwRegisterValue;
                    }
                    else
                    {
                        U32 dwClockLevel = 0;
                        //Return to  Normal Mode
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                        dwClockLevel = MHalDvfsSearchCpuClockLevel(dwCpuClock, dwCpu);
                        //DVFS_HAL_DEBUG("[DVFS] dwClockLevel: %d dwCpuClock: %d \n", dwClockLevel, dwCpuClock);
                        DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock);


                        MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock, dwCpu);

                        DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                        hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock;
                    }
                }
                else
                {
                    U32 dwClockLevel = 0;
                    //Keep at Normal Mode
                    DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                    dwClockLevel = MHalDvfsSearchCpuClockLevel(dwCpuClock, dwCpu);
                    DVFS_HAL_DEBUG("[DVFS] dwClockLevel: %d dwCpuClock: %d \n", dwClockLevel, dwCpuClock);
                    DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                    DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock);

                    MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock, dwCpu);

                    DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                    hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock;
                }
            }
        }

        //MsOS_ReleaseMutex(_s32SAR_Dvfs_Mutex);
    }
    else
    {
        //Disable DVFS
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb0) << 1)) = 0;
    }

_MHalDvfsProcExit:

//  hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwCpuClock;// = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;
    DVFS_HAL_DEBUG("[DVFS] Current DVFS State: %d\n", (unsigned int) DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state);
    DVFS_HAL_DEBUG("[DVFS] hMstarDvfsInfo[%d].dwFinalCpuClock: %d\n", dwCluster, hMstarDvfsInfo[dwCluster].dwFinalCpuClock);

    return hMstarDvfsInfo[dwCluster].dwFinalCpuClock;
}

//=================================================================================================
U32 MHalDvfsInit(U8 dwCluster)
{
    U32 bDvfsInitStatus = TRUE;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb2) << 1)) &= ~(0x01);

    hMstarDvfsInfo[dwCluster].bDvfsInitOk = 0;
    hMstarDvfsInfo[dwCluster].bDvfsModeChange = 0;
    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = 0;
    hMstarDvfsInfo[dwCluster].dwFinalCpuClock = 0;
    hMstarDvfsInfo[dwCluster].dwFinalCpuPowerVoltage = 0;
    hMstarDvfsInfo[dwCluster].dwFinalCorePowerVoltage = 0;
    hMstarDvfsInfo[dwCluster].dwCpuTemperature = 0;
    hMstarDvfsInfo[dwCluster].dwRefTemperature = 0;
    hMstarDvfsInfo[dwCluster].dwAvgCpuTempCounter = 0;
    hMstarDvfsInfo[dwCluster].dwAvgCpuTempBuffer = 0;
    hMstarDvfsInfo[dwCluster].bSystemResumeFlag = 0;
    hMstarDvfsInfo[dwCluster].dwResetCounter = 0;
    hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
    hMstarDvfsInfo[dwCluster].dwBootTimeCounter = 0;
    hMstarDvfsInfo[dwCluster].dwPradoProtect = 0;
    hMstarDvfsInfo[dwCluster].dwPradoCounter = 0;
    hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 0;

    //Get Reference Level of 25-degree Temperature in eFuse
    MHalDvfsCpuTemperature(dwCluster);

    u16CpuClockSet[dwCluster] = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;

    //Init Test Bus to Measure CPU Clock
    MHalDvfsCpuDisplay(dwCluster);

#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
    if(SysDvfsPowerInit(dwCluster) == TRUE)
    {
        hMstarDvfsInfo[dwCluster].bDvfsInitOk = 1;
    }
    else
    {
        hMstarDvfsInfo[dwCluster].bDvfsInitOk = 0;
        bDvfsInitStatus = FALSE;
    }
#else
    hMstarDvfsInfo[dwCluster].bDvfsInitOk = 1;
#endif

    return bDvfsInitStatus;
}

//=================================================================================================
void MDrvHalDvfsInit(void)
{
    int i = 0;
    for (i = 0; i < DVFS_HAL_TOTAL_CLUSTER_NUM; i ++)
    {
        MHalDvfsCpuDisplayInit(i);
        MHalDvfsCpuTemperature(i);
        MHalDvfsRefTemperature(i);
        MHalDvfsCpuPowerInit(i);
        MHalDvfsCorePowerInit(i);
    }
}
//=================================================================================================
U32 MHalDvfsGetCpuFreq(U8 dwCpu)
{
    U32 dwRegisterValue = 0;
    int dwCluster = getCpuCluster(dwCpu);
    mutex_lock(&MHalDvfsGetCpuFreqMutex);
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) << 1));
    dwRegisterValue = dwRegisterValue & ~0x01;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) << 1)) = dwRegisterValue;

    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) << 1));
    dwRegisterValue = dwRegisterValue | 0x01;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) << 1)) = dwRegisterValue;

    udelay(CONFIG_DVFS_CLOCK_DELAY_US);

    //CPU Clock = (reg_calc_cnt_report(DEC) / 83333) * 16
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf4) << 1));
    dwRegisterValue *= 16;
    dwRegisterValue *= 1000;
    dwRegisterValue /= 83333;
    mutex_unlock(&MHalDvfsGetCpuFreqMutex);
    return dwRegisterValue;
}
//=================================================================================================
int MHalDvfsCpuDisplay(U8 dwCluster)
{
    U32     dwRegisterValue = 0;
    U32     u32RetryCount=0;
#ifdef CONFIG_DVFS_CPU_CLOCK_DISPLAY_ENABLE
    if( dwCluster == 0)
        dwRegisterValue = MHalDvfsGetCpuFreq(0);
    else
        dwRegisterValue = MHalDvfsGetCpuFreq(2);

    if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state != CONFIG_DVFS_INIT_MODE)
        && (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state != CONFIG_DVFS_BOOT_MODE))
    {
        while(!((dwRegisterValue <= u16CpuClockSet[dwCluster] + CONFIG_DVFS_CPU_CLOCK_ACCURACY_BOUNDARY)
            && (dwRegisterValue >= u16CpuClockSet[dwCluster] - CONFIG_DVFS_CPU_CLOCK_ACCURACY_BOUNDARY)))
        {
            if( dwCluster == 0)
                dwRegisterValue = MHalDvfsGetCpuFreq(0);
            else
                dwRegisterValue = MHalDvfsGetCpuFreq(2);

            u32RetryCount++;
            if (u32RetryCount > 5)
            {
                // CPU Clock check timeout, show info and setup original clock
                DVFS_HAL_DEBUG("[DVFS][Error] Check CPU Clock not correct, Clock = %d]]\n", dwRegisterValue);
                dwRegisterValue = u16CpuClockSet[dwCluster];
                break;
            }
        }
    }
    DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock = dwRegisterValue;

    DVFS_HAL_DEBUG("[DVFS] CPU Clock: %dMHz\n", (unsigned int) dwRegisterValue);
#endif
    return dwRegisterValue;
}

//=================================================================================================
void MHalDvfsCpuTemperature(U8 dwCluster)
{
    S32     dwTempData = 0;

    mutex_lock(&MDrvDvfsCpuTempMutex);
    if(hMstarDvfsInfo[dwCluster].dwRefTemperature == 0)
    {
        MHalDvfsRefTemperature(dwCluster);
    }

    //Read CH8 of PM_SAR to Get CPU Temperature
    if (dwCluster == 0)
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001420 << 1)) &= ~0x1;
    }
    else
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001420 << 1)) |= 0x01;
    }
    udelay(CONFIG_DVFS_CLOCK_DELAY_US);
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) |= (0x01 << 14);
    dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00148e << 1));

    if(dwTempData == 0)
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) = 0x0A20;

        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) |= (0x01 << 14);

        dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00148e << 1));
    }

    hMstarDvfsInfo[dwCluster].dwAvgCpuTempBuffer += dwTempData;
    hMstarDvfsInfo[dwCluster].dwAvgCpuTempCounter ++;
    if((hMstarDvfsInfo[dwCluster].dwAvgCpuTempCounter >= CONFIG_DVFS_DATA_COUNT) || \
       (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE))
    {
        S32    dwTempValue = 0;

        DVFS_HAL_DEBUG("[DVFS] [[====================================\n");

        MHalDvfsCpuDisplay(dwCluster);

        DVFS_HAL_DEBUG("[DVFS] Reference Temperature Data (%d): 0x%04x\n", dwCluster, (unsigned int) hMstarDvfsInfo[dwCluster].dwRefTemperature);

        if(hMstarDvfsInfo[dwCluster].dwAvgCpuTempBuffer != 0)
        {
            dwTempValue = hMstarDvfsInfo[dwCluster].dwAvgCpuTempBuffer / hMstarDvfsInfo[dwCluster].dwAvgCpuTempCounter;

            DVFS_HAL_DEBUG("[DVFS] Current Temperature Sensor Data (%d): 0x%04x\n", dwCluster, (unsigned int) dwTempValue);

            if(hMstarDvfsInfo[dwCluster].dwRefTemperature >= dwTempValue)
            {
                dwTempData = (((hMstarDvfsInfo[dwCluster].dwRefTemperature - dwTempValue) * 1280) + CONFIG_DVFS_T_SENSOR_SHIFT);
            }
            else
            {
                dwTempData = ((dwTempValue - hMstarDvfsInfo[dwCluster].dwRefTemperature) * 1280);
                dwTempData = (CONFIG_DVFS_T_SENSOR_SHIFT - dwTempData);
            }

            hMstarDvfsInfo[dwCluster].dwCpuTemperature = (dwTempData / 1000);
            DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_temp = hMstarDvfsInfo[dwCluster].dwCpuTemperature;
        }

        DVFS_HAL_DEBUG("[DVFS] Temperature (%d): %d\n", dwCluster, hMstarDvfsInfo[dwCluster].dwCpuTemperature);

        hMstarDvfsInfo[dwCluster].dwCpuTemperature += gDVFS_TEMP_OFFSET;
        hMstarDvfsInfo[dwCluster].dwAvgCpuTempBuffer = 0;
        hMstarDvfsInfo[dwCluster].dwAvgCpuTempCounter = 0;

        DVFS_HAL_DEBUG("[DVFS] CPU Power: %d0mV\n", hMstarDvfsInfo[dwCluster].dwFinalCpuPowerVoltage);
        DVFS_HAL_DEBUG("[DVFS] Core Power: %d0mV\n", hMstarDvfsInfo[dwCluster].dwFinalCorePowerVoltage);

        DVFS_HAL_DEBUG("[DVFS] ====================================]]\n");
    }
    mutex_unlock(&MDrvDvfsCpuTempMutex);
}

//=================================================================================================
void MHalDvfsCpuClockAdjustment(U32 dwCpuClock, U8 dwCpu)
{
    U32     dwRegisterValue = 0;
    U32     dwTempCpuClock  = 0;
    int     dwCluster       = getCpuCluster(dwCpu);
    static  bool bLowClkFg[2];

    dwTempCpuClock = (dwCpuClock - (dwCpuClock % 4));
    if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock != dwTempCpuClock) || (hMstarDvfsInfo[dwCluster].bDvfsModeChange == 1))
    {
        if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock < dwCpuClock) && (gDVFS_auto_measurement == 0))
        {
            MHalDvfsPowerControl(dwCpuClock, dwCpu);
        }

        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        if (dwCpuClock < 700)
        {
            if (bLowClkFg[dwCluster] == 0)
            {
                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfc) << 1));
                dwRegisterValue = (dwRegisterValue & (~0x01));
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfc) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfa) << 1));
                dwRegisterValue = (dwRegisterValue | 0x01);
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfa) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1));
                dwRegisterValue = (dwRegisterValue & (~0x01));
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0x2a) << 1));
                dwRegisterValue = (dwRegisterValue | 0x02);
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0x2a) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1));
                dwRegisterValue = (dwRegisterValue | 0x01);
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1)) = dwRegisterValue;

                bLowClkFg[dwCluster] = 1;
            }

            //Adjust User Defined CPU Clock
            dwRegisterValue = ((3623878UL / (dwCpuClock << 1)) * 1000);
        }
        else
        {
            if (bLowClkFg[dwCluster] == 1)
            {
                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfc) << 1));
                dwRegisterValue = (dwRegisterValue & (~0x01));
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfc) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfa) << 1));
                dwRegisterValue = (dwRegisterValue | 0x01);
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfa) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1));
                dwRegisterValue = (dwRegisterValue & (~0x01));
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0x2a) << 1));
                dwRegisterValue = (dwRegisterValue & (~0x02));
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0x2a) << 1)) = dwRegisterValue;

                dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1));
                dwRegisterValue = (dwRegisterValue | 0x01);
                *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8) << 1)) = dwRegisterValue;

                bLowClkFg[dwCluster] = 0;
            }

            //Adjust User Defined CPU Clock
            dwRegisterValue = ((3623878UL / dwCpuClock) * 1000);
        }
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa4) << 1))
                            = (dwRegisterValue & 0xFFFF);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa6) << 1))
                            = ((dwRegisterValue >> 16) & 0xFFFF);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb0) << 1))
                            = 0x01;   //switch to LPF control
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xaa) << 1))
                            = 0x06;   //mu[2:0]
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xae) << 1))
                            = 0x08;   //lpf_update_cnt[7:0]

        //Set LPF is Low to High
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb2) << 1))
                            |= (0x01 << 12);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa8) << 1))
                            = 0x00;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa8) << 1))
                            = 0x01;

        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock >= dwCpuClock) && (gDVFS_auto_measurement == 0))
        {
            MHalDvfsPowerControl(dwCpuClock, dwCpu);
        }

        u16CpuClockSet[dwCluster] = dwCpuClock;
        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock = dwCpuClock;


        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 0;
    }
}

//=================================================================================================
U32 MHalDvfsSearchCpuClockLevel(U32 dwCpuClock, U8 dwCpu)
{
    U32     dwLoopCounter = 0;
    U32     dwCpuLevel = 0;
    int     dwCluster = getCpuCluster(dwCpu);

    //Confirm Corresponding Level by User Defined CPU Clock
    for(dwLoopCounter = 0; dwLoopCounter < CONFIG_DVFS_POWER_CTL_SEGMENT; dwLoopCounter++)
    {
        if((dwCpuClock >= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsFreezeModeInfo[dwLoopCounter].DvfsCpuInfo.dwLowerCpuClock) && \
           (dwCpuClock < hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsFreezeModeInfo[dwLoopCounter].DvfsCpuInfo.dwUpperCpuClock)  && \
           (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
        {
            dwCpuLevel = dwLoopCounter;
            return dwCpuLevel;
        }
        else if((dwCpuClock >= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwLoopCounter].DvfsCpuInfo.dwLowerCpuClock) && \
           (dwCpuClock < hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwLoopCounter].DvfsCpuInfo.dwUpperCpuClock))
        {
            dwCpuLevel = dwLoopCounter;
            return dwCpuLevel;
        }
    }
    return dwCpuLevel;
}

//=================================================================================================
U32 MHalDvfsVerifyCpuClock(U32 dwCpuClock, U8 dwCpu)
{
    U32     dwLoopCounter = 0;
    int     dwCluster = getCpuCluster(dwCpu);
    //Confirm Corresponding Level by User Defined CPU Clock
    for(dwLoopCounter = 0; dwLoopCounter < CONFIG_DVFS_POWER_CTL_SEGMENT; dwLoopCounter++)
    {
        if ((dwCpuClock == hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsFreezeModeInfo[dwLoopCounter].DvfsCpuInfo.dwLowerCpuClock))
        {
            return 1;
        }
    }
    return 0;
}

//=================================================================================================
void MHalDvfsPowerControl(U32 dwCpuClock, U8 dwCpu)
{
    U32     dwClockLevel = 0;
    int     dwCluster = getCpuCluster(dwCpu);

    dwClockLevel = MHalDvfsSearchCpuClockLevel(dwCpuClock, dwCpu);
    if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
    {
        //Init Mode
        //Adjust CPU Power
        MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower, dwCpu);

        //Adjust Core Power
        MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type].dwCorePower, dwCpu);
    }
    else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
    {
        //Boot Mode
        //Adjust CPU Power
        MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsBootModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower, dwCpu);

        //Adjust Core Power
        MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsBootModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type].dwCorePower, dwCpu);
    }
    else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_LIGHT_MODE)
    {
        //LIGHT Mode
        DVFS_HAL_DEBUG("\033[32m [DVFS] eason.lien \0330[0m\n");

        //Adjust CPU Power
        MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower, dwCpu);

        //Adjust Core Power
        MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type].dwCorePower, dwCpu);
    }
    else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_FREEZE_MODE)
    {
        //Freeze Mode
        if(dwClockLevel == CONFIG_DVFS_CPU_CLOCK_DISABLE)
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuPower, dwCpu);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCorePower, dwCpu);
        }
        else
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsFreezeModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower, dwCpu);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsFreezeModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type].dwCorePower, dwCpu);
        }
    }
    else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
    {
        //Over-Temperature Mode
        //Adjust CPU Power
        MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower, dwCpu);

        //Adjust Core Power
        MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type].dwCorePower, dwCpu);
    }
    else
    {
        //Normal Mode and Special Clock Mode
        if(dwClockLevel == CONFIG_DVFS_CPU_CLOCK_DISABLE)
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuPower, dwCpu);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCorePower, dwCpu);
        }
        else
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower, dwCpu);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type].dwCorePower, dwCpu);
        }
    }
}

//=================================================================================================
void MHalDvfsPradoMonitor(U8 dwCpu)
{

#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
        int     dwCluster = getCpuCluster(dwCpu);
        SysDvfsPradoProtect(dwCluster);
#else
        MDrvDvfsVoltageSetup(0, 0, 0xfc, dwCpu);
#endif

}

//=================================================================================================
void MHalDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage, U8 dwCpu)
{
    int     dwCluster = getCpuCluster(dwCpu);

    if(hMstarDvfsInfo[dwCluster].bSystemResumeFlag != 0)
    {
        DVFS_HAL_DEBUG("\033[1;31m[DVFS] Re-init Power Control Flow\033[0m\n");

#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
        SysDvfsCpuPowerInit();
        DVFS_HAL_DEBUG("[DVFS][Kernel IIC] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
#else
        MDrvDvfsVoltageSetup(0, 0, 0xFE, dwCpu);
        DVFS_HAL_DEBUG("[DVFS][Utopia IIC] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
#endif
        hMstarDvfsInfo[dwCluster].bSystemResumeFlag = 0;
    }
    else
    {
        #if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
            SysDvfsCpuPowerAdjustment(dwCpuPowerVoltage, dwCluster);
            DVFS_HAL_DEBUG("[DVFS][Kernel IIC] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
        #else
            MDrvDvfsVoltageSetup(0, dwCpuPowerVoltage, 0, dwCpu);
            DVFS_HAL_DEBUG("[DVFS][Utopia IIC] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
        #endif
    }
    //DVFS_HAL_DEBUG("[DVFS] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
    hMstarDvfsInfo[dwCluster].dwFinalCpuPowerVoltage = dwCpuPowerVoltage;
}

//=================================================================================================
void MHalDvfsCorePowerAdjustment(U32 dwCorePowerVoltage, U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);

#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
    SysDvfsCorePowerAdjustment(dwCorePowerVoltage, dwCluster);
#else
    MDrvDvfsVoltageSetup(0, dwCorePowerVoltage, 1, dwCpu);
#endif
    DVFS_HAL_DEBUG("[DVFS] Core Power: %d\n", (unsigned int) dwCorePowerVoltage);
    hMstarDvfsInfo[dwCluster].dwFinalCorePowerVoltage = dwCorePowerVoltage;
}

//=================================================================================================
U32  MHalDvfsQueryCpuClock(U32 dwCpuClockType, U8 dwCpu)
{
    U32     dwOutputCpuClock = 0;
    int     dwCluster = getCpuCluster(dwCpu);

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    if(dwCpuClockType == CONFIG_DVFS_MAX_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
    }
    else if(dwCpuClockType == CONFIG_DVFS_MIN_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMinimumCpuClock;
    }
    else if(dwCpuClockType == CONFIG_DVFS_IR_BOOTS_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
        u16CpuClockSet[dwCluster] = dwOutputCpuClock;
    }
    else if(dwCpuClockType == CONFIG_DVFS_OVER_TEMPERATURE_PROTECT_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
    }
    else
    {
        dwOutputCpuClock = CONFIG_DVFS_CPU_CLOCK_DISABLE;
    }

    return dwOutputCpuClock;
}

//=================================================================================================
U32 MHalDvfsQueryCpuClockByTemperature(U8 dwCpu)
{
//  S32     dwCpuTemperature = 0;
    int    dwCluster = getCpuCluster(dwCpu);

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    if(DvfsRegInfo->dvfs_reg[dwCluster].reg_vid_dvfs_id == CONFIG_DVFS_ENABLE_PATTERN)
    {
        if(hMstarDvfsInfo[dwCluster].bDvfsModeChange == 1)
        {
            goto _MHalDvfsQueryCpuClockByTemperatureExit;
        }

        if(hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature == 0)
        {
            hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;
        }

        if(hMstarDvfsInfo[dwCluster].bDvfsInitOk == 0)
        {
            if(MHalDvfsInit(dwCluster) == TRUE)
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
            }
            else
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;
            }
        }
        else
        {
            MHalDvfsCpuTemperature(dwCluster);   //Get current CPU temperature
            if (dwCluster == 1)
            {
                MHalDvfsPradoMonitor(dwCluster);
                if ((*(volatile U16 *) (CONFIG_REGISTER_BASE_ADDRESS + (0x100514 << 1))) == 0x1)
                {
                    DVFS_HAL_INFO("\033[31m [DVFS] Cluster 1 Prado protect \033[0m \n");
                    hMstarDvfsInfo[dwCluster].dwPradoProtect = 1;
                    hMstarDvfsInfo[dwCluster].dwPradoCounter = 0;
                    DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
                }
                else if (hMstarDvfsInfo[dwCluster].dwPradoProtect == 1)
                {
                    if(hMstarDvfsInfo[dwCluster].dwPradoCounter < CONFIG_DVFS_AVERAGE_PRADO_COUNT)
                    {
                        DVFS_HAL_INFO("\033[31m [DVFS] Prado Over-Temperature Counter: %d \033[0m \n", (unsigned int) hMstarDvfsInfo[dwCluster].dwPradoCounter);
                        hMstarDvfsInfo[dwCluster].dwPradoCounter++;
                    }
                    else
                    {
                        DVFS_HAL_INFO("\033[31m [DVFS] prado protect disable \033[0m \n");
                        hMstarDvfsInfo[dwCluster].dwPradoProtect = 0;
                        hMstarDvfsInfo[dwCluster].dwPradoCounter = 0;
                    }
                }
            }

            if ((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100516 << 1)) == CONFIG_DVFS_CPU_CLOCK_LIGHT_MODE)  && \
                (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state != CONFIG_DVFS_LIGHT_MODE)  && \
                (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsCpuInfo.dwLowerCpuClock != CONFIG_DVFS_LIGHTMODE_DISABLE))
            {
                DVFS_HAL_INFO("\033[32m[DVFS] eason.lien [%s, %d] \033[0m\n", __FUNCTION__, __LINE__);
                DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state  = CONFIG_DVFS_NORMAL_MODE;
                DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk = CONFIG_DVFS_CPU_CLOCK_LIGHT_MODE;
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_LIGHT_MODE;
                hMstarDvfsInfo[dwCluster].bDvfsModeChange            = 1;
            }
            else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
            }
            else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwMaxLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("\033[1;31m[DVFS] Over Temperature Protection: %d\033[0m\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Maximum Level Threshold Temperature in Over-Temperature Mode
                        DVFS_HAL_INFO("\033[1;31m[DVFS] Current Temperature: %d\033[0m\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                        DVFS_HAL_INFO("\033[1;31m[DVFS] Over Temperature Mode: SYSTEM RESET\033[0m\n");

                        //Trigger a WDT Reset
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x00;
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003008 << 1)) = 0x00;
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x05;
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003000 << 1)) = 0x01;

                        while(1);
                    }
                }
                else if (hMstarDvfsInfo[dwCluster].dwPradoProtect == 1)
                {
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                }
                else if(hMstarDvfsInfo[dwCluster].dwCpuTemperature <
                        hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_LOW_TEMP_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Over-Temperature->Normal Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Return to Normal Mode
                        hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 0;
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Over-Temperature Mode
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                }
            }
            else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_HIGH_TEMP_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Boot->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Over-Temperature Mode
                        hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
                        //hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }

                    hMstarDvfsInfo[dwCluster].dwBootTimeCounter = 0;
                }
                else
                {
                    //Keep at Boot Mode
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;

                    hMstarDvfsInfo[dwCluster].dwBootTimeCounter ++;
                    if(hMstarDvfsInfo[dwCluster].dwBootTimeCounter > CONFIG_DVFS_BOOT_MODE_TIME)
                    {
                        //Return to Normal Mode
//                      DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        if(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature == CONFIG_DVFS_TEMPERATURE_DISABLE)
                        {
                            DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        }
                        else
                        {
                            DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;
                        }

                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Boot Mode->Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
                    }
                    else
                    {
                        DVFS_HAL_DEBUG("[DVFS] Boot Mode Counter: %d\n", hMstarDvfsInfo[dwCluster].dwBootTimeCounter);
                    }
                }
            }
            else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_FREEZE_MODE)
            {
                if((hMstarDvfsInfo[dwCluster].dwCpuTemperature >= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
                   (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Freeze->Normal Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Return to Normal Mode
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                }
            }
            else if (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_LIGHT_MODE)
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Light Mode ->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Over-Temperature Mode
                        hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    if (*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100516 << 1)) == CONFIG_DVFS_CPU_CLOCK_LIGHT_MODE)
                    {
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_LIGHT_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsLightModeInfo.DvfsCpuInfo.dwLowerCpuClock;
                    }
                    else if (*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100516 << 1)) != CONFIG_DVFS_LIGHTMODE_DISABLE)
                    {
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
                    }
                }
            }
            else if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_NORMAL_MODE) || (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_SPECIAL_CLOCK_MODE))
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_HIGH_TEMP_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Normal->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Normal Mode
                        hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else if((hMstarDvfsInfo[dwCluster].dwCpuTemperature < hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature) && \
                        (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Normal->Freeze Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Freeze Threshold Temperature in Normal Mode
                        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Freeze Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Normal Mode
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                }
            }
            else
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
            }
        }
    }

_MHalDvfsQueryCpuClockByTemperatureExit:

    DVFS_HAL_DEBUG("[DVFS] Current DVFS State: %d\n", (unsigned int) DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state);
    DVFS_HAL_DEBUG("[DVFS] Current Valid CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

    return hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature;
}

//=================================================================================================
void MHalDvfsCpuDisplayInit(U8 dwCluster)
{
#ifdef CONFIG_DVFS_CPU_CLOCK_DISPLAY_ENABLE

    U32     dwRegisterValue = 0;

    //Init Test Bus to Measure CPU Clock
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) << 1));
    dwRegisterValue = dwRegisterValue | 0x01;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) << 1)) = dwRegisterValue;
#endif
}
//=================================================================================================
U32 MHalDvfsGetOverTemperatureFlag(U8 dwCluster)
{
   return hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag;
}

//=================================================================================================
void MHalDvfsRefTemperature(U8 dwCluster)
{
    U32     dwRegisterValue = 0;

    //Read 25-degree Reference Level in eFuse
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x0144;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x2144;
    while((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) & (0x01 << 13)) != 0);

    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002058 << 1));
    dwRegisterValue >>= 6;

    //If no data existed in eFuse, set the default reference level is 400
    hMstarDvfsInfo[dwCluster].dwRefTemperature = dwRegisterValue;
    if(dwRegisterValue == 0)
    {
        hMstarDvfsInfo[dwCluster].dwRefTemperature = 400;
    }
}
//=================================================================================================
U32 MHalDvfsGetCpuTemperature(U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }
    MHalDvfsCpuTemperature(dwCluster);
    DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_temp = hMstarDvfsInfo[dwCluster].dwCpuTemperature;
    return DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_temp;
}

//=================================================================================================
U32 MHalDvfsGetCpuVoltage(U8 dwCpu)
{
    S32     dwTempData = 0;
    U32     dwRegisterBackup = 0;

    //Read CH8 of PM_SAR to Get CPU Voltage
    mutex_lock(&MDrvDvfsCpuTempMutex);

    //Save the original setting for Temperature
    dwRegisterBackup = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1));
    //Change SAR to single mode
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) = 0x0AB0;
    //Read the voltage for the CORE_PWR_ONLINE_DET[0~3]
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001404 << 1)) = 0x100+ dwCpu;

    udelay(CONFIG_DVFS_CLOCK_DELAY_US);


    //tooggle to update sar code data
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) |= (0x01 << 14);

    dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001480 << 1));

    //Recover the Status
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001404 << 1)) = 0x0;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) = dwRegisterBackup;

    mutex_unlock(&MDrvDvfsCpuTempMutex);

    return dwTempData*100*(10/3)/1024; //result for *3.3/1024
}
//=================================================================================================
void MHalDvfsSetOverTempDebugOffset(unsigned int cpu,int set_offset)
{
    gDVFS_TEMP_OFFSET = set_offset;
}
int MHalDvfsGetOverTempDebugOffset(unsigned int cpu)
{
    return gDVFS_TEMP_OFFSET;
}

//=================================================================================================
U32 MHalDvfsGetVoltage(U8 dwCpu)
{
    U32    dwClockLevel = 0;
    int    dwCluster = getCpuCluster(dwCpu);

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    dwClockLevel = MHalDvfsSearchCpuClockLevel(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock, dwCpu);

    return hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower;
}

//=================================================================================================
U32 MHalDvfsGetSidd(void)
{
    U32    dwRegisterValue = 0;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x01ac;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x21ac;
    while((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) & (0x01 << 13)) != 0);
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002058 << 1));
    dwRegisterValue += ((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00205a << 1))) << 16);
    dwRegisterValue = dwRegisterValue & 0x3ff;
    return dwRegisterValue;
}

//=================================================================================================
U32 MHalDvfsGetOsc(U8 dwCpu)
{
    U32    dwRegisterValue = 0;
    int    dwCluster = getCpuCluster(dwCpu);

    if (dwCluster == 0)
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x01ac;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x21ac;
        while((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) & (0x01 << 13)) != 0);
        dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002058 << 1));
        dwRegisterValue += ((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00205a << 1))) << 16);
        dwRegisterValue = (dwRegisterValue >> 10) & 0x3ff;
    }
    else
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x01b4;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x21b4;
        while((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) & (0x01 << 13)) != 0);
        dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002058 << 1));
        dwRegisterValue += ((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00205a << 1))) << 16);
        dwRegisterValue = (dwRegisterValue >> 2) & 0x3ff;
    }

    return dwRegisterValue;
}

//=================================================================================================
void MHalDvfsGetDvfsTable(U8 dwCpu, dvfs_opp_table *opp_table)
{
    U32    dwClockLevel = 0;
    int    dwCluster = getCpuCluster(dwCpu);
    U8     dwLoop = 0;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    for (dwLoop = 0; dwLoop < CONFIG_DVFS_POWER_CTL_SEGMENT; dwLoop++)
    {
        opp_table[dwCpu].per_cpu_table[dwLoop].clk = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwLoop].DvfsCpuInfo.dwLowerCpuClock * 1000000;
        opp_table[dwCpu].per_cpu_table[dwLoop].volt = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwLoop].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower * 10000;
        DVFS_HAL_INFO("eason.lien opp_table[%d][%d].clk: %d ", dwCpu, dwLoop, opp_table[dwCpu].per_cpu_table[dwLoop].clk);
    }
}

//=================================================================================================
U8 MHalDvfsGetFreqTable(U8 dwCpu, struct cpufreq_frequency_table **freq_table)
{
    U8 dwLoop = 0;
    *freq_table = kzalloc(sizeof(struct cpufreq_frequency_table) * DVFS_FREQ_LEVEL_MAX_INDEX, GFP_KERNEL);
    for (dwLoop = 0; dwLoop < DVFS_FREQ_LEVEL_MAX_INDEX; dwLoop++)
    {
        if (hal_freq_table[dwLoop].frequency == CPUFREQ_TABLE_END )
        {
            (*freq_table)[dwLoop].frequency = CPUFREQ_TABLE_END;
            break;
        }
        (*freq_table)[dwLoop].frequency = hal_freq_table[dwLoop].frequency;
    }

    for (dwLoop=0;dwLoop < DVFS_FREQ_LEVEL_MAX_INDEX;dwLoop++)
    {
        //printk("freq_table: cpu= %d, index = %d, frequency = %d\n",dwCpu,dwLoop,(*freq_table)[dwLoop].frequency );
        DVFS_HAL_DEBUG("freq_table: cpu= %d, index = %d, frequency = %d\n",dwCpu,dwLoop,(*freq_table)[dwLoop].frequency );
    }
    return DVFS_FREQ_LEVEL_MAX_INDEX;
}

//=================================================================================================
void MHalDvfsSetAutoMeasurement(U32 auto_measurement)
{
    gDVFS_auto_measurement = auto_measurement;
}

//=================================================================================================
U32 MHalDvfsGetCpuPowerType(U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }
    return DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type;
}

//=================================================================================================
U32 MHalDvfsGetCorePowerType(U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }
    return DvfsRegInfo->dvfs_reg[dwCluster].reg_core_pwr_type;
}
//=================================================================================================
void MHalDvfsCpuPowerInit(U8 dwCluster)
{
    //TBD
}

//=================================================================================================
void MHalDvfsCorePowerInit(U8 dwCluster)
{
    //TBD
}
//=================================================================================================
void MHalDvfsGetTemperatureInfo(U8 dwCpu, DVFS_THERMAL_INFO *thermal_info)
{
   // not support
}

//=================================================================================================
static struct platform_device mstar_dvfs_dev =
{
    .name   = "mstar_dvfs",
    .id     = 0,
};

static int mstar_dvfs_drv_suspend(struct device *dev)
{
    U8 cluster = 0;
    DVFS_HAL_DEBUG("[DVFS] Enter Suspend Mode\n");

//  MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock);

    if (NULL == DvfsRegInfo)
    {
        DVFS_HAL_DEBUG("\033[34mFunction = %s, DvfsRegInfo was not initialized(NULL).\033[m\n", __PRETTY_FUNCTION__);
		return 0;
    }

    for (cluster = 0; cluster < DVFS_HAL_TOTAL_CLUSTER_NUM; cluster ++)
    {
        DvfsRegInfo->dvfs_reg[cluster].reg_vid_dvfs_id = 0;
    }
    return 0;
}

static int mstar_dvfs_drv_resume(struct device *dev)
{
    U8 cluster = 0;

    DVFS_HAL_DEBUG("\033[31mFunction = %s, Line = %d, [DVFS] Enter Resume Mode\033[m\n", __PRETTY_FUNCTION__, __LINE__);

    halTotalClusterNumber = DVFS_HAL_TOTAL_CLUSTER_NUM;
    for (cluster = 0; cluster < DVFS_HAL_TOTAL_CLUSTER_NUM; cluster ++)
    {
        //Only one cluster
        MHalDvfsInit(cluster);
        hMstarDvfsInfo[cluster].bSystemResumeFlag = 1;

        DvfsRegInfo->dvfs_reg[cluster].reg_cur_dvfs_state = CONFIG_DVFS_INIT_MODE;
    }
    return 0;
}

static int mstar_dvfs_drv_freeze(struct device *dev)
{
    return 0;
}

static int mstar_dvfs_drv_thaw(struct device *dev)
{
    return 0;
}

static int mstar_dvfs_drv_restore(struct device *dev)
{
    return 0;
}

static int mstar_dvfs_drv_probe(struct platform_device *pdev)
{
    pdev->dev.platform_data = NULL;
    return 0;
}

static int mstar_dvfs_drv_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data = NULL;
    return 0;
}

static const struct dev_pm_ops mstar_dvfs_dev_pm_ops =
{
    .suspend = mstar_dvfs_drv_suspend,
    .resume = mstar_dvfs_drv_resume,
    .freeze = mstar_dvfs_drv_freeze,
    .thaw = mstar_dvfs_drv_thaw,
    .restore = mstar_dvfs_drv_restore,
};

static struct platform_driver mstar_dvfs_driver =
{
    .probe = mstar_dvfs_drv_probe,
    .remove = mstar_dvfs_drv_remove,

    .driver =
    {
        .name = "mstar_dvfs",
        .owner = THIS_MODULE,
        .pm = &mstar_dvfs_dev_pm_ops,
    }
};

static void __init classify_cpu_cluster(void)
{
	int i, j;
    for (j = 0; j < DVFS_HAL_TOTAL_CLUSTER_NUM; j ++) {
        for_each_online_cpu(i) {
#ifdef CONFIG_ARM64
            if (MIDR_PARTNUM(get_cpu_midr(i)) == hMstarDvfsInfo[j].dwCpuPartId) {
#else
            if ( ((get_cpu_midr(i) & 0xFFF0) >> 4) == hMstarDvfsInfo[j].dwCpuPartId ) {
#endif
                hMstarDvfsInfo[j].dwClusterCpuMask |= (0x1<<i);
            }
        }
        DVFS_HAL_INFO("Cluster:%d CpuMask:%x\n", j, hMstarDvfsInfo[j].dwClusterCpuMask);
    }

}
#ifdef CONFIG_THERMAL_GOV_POWER_ALLOCATOR

#define SOC_SENSOR "SENSOR_TEMP_SOC"
enum cluster_type {
    CLUSTER_LITTLE = 0,
    CLUSTER_BIG,
    NUM_CLUSTERS
};

struct cluster_power_coefficients {
    int dyn_coeff;
};

struct cluster_power_coefficients cluster_data[] = {
    [CLUSTER_LITTLE] = {
        .dyn_coeff = 221,
    },
    [CLUSTER_BIG] = {
        .dyn_coeff = 818,
    },

};

struct mstar_sensor {
    unsigned long prev_temp;
    struct thermal_zone_device *tzd;
    struct cpumask cluster[NUM_CLUSTERS];
    struct thermal_cooling_device *cdevs[NUM_CLUSTERS];
};
static struct mstar_sensor mstar_temp_sensor;


/* Temperatures on power over-temp-and-voltage curve (C) */
static const int vt_temperatures[] = { 30, 50, 75, 100, 125};

/* Voltages on power over-temp-and-voltage curve (mV) */
static const int *big_vt_voltages;
static const int *little_vt_voltages;
#define POWER_TABLE_NUM_TEMP 5
#define BIG_POWER_TABLE_NUM_VOLT 6
#define LITTLE_POWER_TABLE_NUM_VOLT 6
#define NORMAL_CASE_MAX_FREQUENCY 1400000000
/* Voltages on power over-temp-and-voltage curve (mV) */
static const int SS_big_vt_voltages[BIG_POWER_TABLE_NUM_VOLT] = { 850000, 950000, 1030000, 1080000, 1130000, 1150000};
static const int SS_little_vt_voltages[LITTLE_POWER_TABLE_NUM_VOLT] = { 850000, 950000, 100000, 1030000, 1080000, 1130000};

static const int TT_big_vt_voltages[BIG_POWER_TABLE_NUM_VOLT] = { 850000, 950000, 1030000, 1080000, 1130000, 1150000};
static const int TT_little_vt_voltages[LITTLE_POWER_TABLE_NUM_VOLT] = { 850000, 950000, 100000, 1030000, 1080000, 1130000};

static const int FF_big_vt_voltages[BIG_POWER_TABLE_NUM_VOLT] = { 850000, 900000, 980000, 1030000, 1080000, 1100000};
static const int FF_little_vt_voltages[LITTLE_POWER_TABLE_NUM_VOLT] = { 850000, 950000, 103000, 1080000, 1130000, 1150000};

static unsigned int *big_power_table[BIG_POWER_TABLE_NUM_VOLT];
static unsigned int *little_power_table[BIG_POWER_TABLE_NUM_VOLT];

static const unsigned int
SS_big_power_table[BIG_POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   30     50     75     100     125 */
        {  5000, 15000, 32000,  75000, 169000 },  /*  850 mV */
        {  8000, 20000, 43000,  97000, 216000 },  /*  950 mV */
        { 15800, 26400, 54800, 122800, 266000 },  /* 1030 mV */
        { 19400, 32200, 65200, 142600, 305800 },  /* 1080 mV */
        { 24600, 39800, 78400, 164800, 356000 },  /* 1130 mV */
        { 27000, 43000, 84000, 174000, 378000 },  /* 1150 mV */
};
static const unsigned int
SS_little_power_table[LITTLE_POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   30     50     75    100     125 */
        {  5000,  9000, 18000, 39000,  86000 },  /*  850 mV */
        {  7000, 11000, 23000, 51000, 111000 },  /*  950 mV */
        {  8000, 13000, 27000, 58000, 125000 },  /* 1000 mV */
        {  9200, 14800, 29400, 63400, 135800 },  /* 1030 mV */
        { 11200, 17800, 34600, 73600, 156200 },  /* 1080 mV */
        { 13800, 21400, 41200, 85200, 181200 },  /* 1130 mV */
};

static const unsigned int
TT_big_power_table[BIG_POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   30     50      75     100     125 */
        { 15000, 28000,  65000, 148000, 323000 },  /*  850 mV */
        { 21000, 40000,  89000, 194000, 419000 },  /*  950 mV */
        { 31200, 55000, 116400, 249400, 523800 },  /* 1030 mV */
        { 40600, 67400, 139000, 292000, 608800 },  /* 1080 mV */
        { 53400, 85000, 168800, 343000, 712000 },  /* 1130 mV */
        { 59000, 93000, 182000, 365000, 756000 },  /* 1150 mV */
};
static const unsigned int
TT_little_power_table[LITTLE_POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   30     50     75     100     125 */
        { 10000, 16000, 36000,  78000, 162000 },  /*  850 mV */
        { 15000, 23000, 48000, 103000, 209000 },  /*  950 mV */
        { 18000, 28000, 58000, 119000, 240000 },  /* 1000 mV */
        { 21000, 32200, 64600, 131600, 262200 },  /* 1030 mV */
        { 27200, 39800, 76800, 154400, 302800 },  /* 1080 mV */
        { 35400, 49600, 92800, 182600, 351200 },  /* 1130 mV */
};

static const unsigned int
FF_big_power_table[BIG_POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*    30      50      75     100      125 */
        {  42000,  68000, 153000, 332000,  680000 },  /*  850 mV */
        {  63000,  83000, 183000, 391000,  796000 },  /*  900 mV */
        {  74000, 100100, 218000, 465000,  956000 },  /*  980 mV */
        {  87200, 142000, 303800, 610800, 1223600 },  /* 1030 mV */
        { 105800, 178600, 359800, 730400, 1468000 },  /* 1080 mV */
        { 115000, 195000, 387000, 784000, 1576000 },  /* 1100 mV */
};
static const unsigned int
FF_little_power_table[LITTLE_POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   30     50      75     100     125 */
        { 21000, 38000,  84000, 171000, 340000 },  /*  850 mV */
        { 32000, 59000, 119000, 240000, 464000 },  /*  950 mV */
        { 50400, 81200, 175000, 328800, 605600 },  /* 1030 mV */
        { 62600,118600, 205600, 382400, 701600 },  /* 1080 mV */
        { 78400,142600, 239200, 449600, 822000 },  /* 1130 mV */
        { 86000,145000, 256000, 480000, 874000 },  /* 1150 mV */
};

static u32 interpolate(int value, const int *x, const unsigned int *y, int len)
{
    u64 tmp64;
    u32 dx;
    u32 dy;
    int i, ret;

    if (value <= x[0])
        return y[0];
    if (value >= x[len - 1])
        return y[len - 1];

    for (i = 1; i < len - 1; i++) {
        /* If value is identical, no need to interpolate */
        if (value == x[i])
            return y[i];
        if (value < x[i])
            break;
    }

    /* Linear interpolation between the two (x,y) points */
    dy = y[i] - y[i - 1];
    dx = x[i] - x[i - 1];

    tmp64 = value - x[i - 1];
    tmp64 *= dy;
    do_div(tmp64, dx);
    ret = y[i - 1] + tmp64;

    return ret;
}

static int get_static_power(cpumask_t *cpumask, int interval,
        unsigned long u_volt, u32 *power)
{
    struct thermal_zone_device *tzd = mstar_temp_sensor.tzd;
    int temperature = 0;
    int low_idx = 0, high_idx = 0;
    int i;
    enum cluster_type cluster =
                topology_physical_package_id(cpumask_any(cpumask));

    if (tzd){
        temperature = tzd->temperature;
        do_div(temperature, 1000);
    }

    if (cluster == CLUSTER_BIG){
        for (i = 0; i < BIG_POWER_TABLE_NUM_VOLT; i++) {
            if (u_volt <= big_vt_voltages[BIG_POWER_TABLE_NUM_VOLT - 1 - i])
                high_idx = BIG_POWER_TABLE_NUM_VOLT - 1 - i;

            if (u_volt >= big_vt_voltages[i])
                low_idx = i;
        }

        if (low_idx == high_idx) {
            *power = interpolate(temperature,
                    vt_temperatures,
                    &big_power_table[low_idx][0],
                    POWER_TABLE_NUM_TEMP);
        } else {
            unsigned long dvt =
                big_vt_voltages[high_idx] - big_vt_voltages[low_idx];
            unsigned long power1, power2;

            power1 = interpolate(temperature,
                    vt_temperatures,
                    &big_power_table[high_idx][0],
                    POWER_TABLE_NUM_TEMP);

            power2 = interpolate(temperature,
                    vt_temperatures,
                    &big_power_table[low_idx][0],
                    POWER_TABLE_NUM_TEMP);

            *power = (power1 - power2) * (u_volt - big_vt_voltages[low_idx]);
            do_div(*power, dvt);
            *power += power2;
        }
    } else if (cluster == CLUSTER_LITTLE) {
        for (i = 0; i < LITTLE_POWER_TABLE_NUM_VOLT; i++) {
            if (u_volt <= little_vt_voltages[LITTLE_POWER_TABLE_NUM_VOLT - 1 - i])
                high_idx = LITTLE_POWER_TABLE_NUM_VOLT - 1 - i;

            if (u_volt >= little_vt_voltages[i])
                low_idx = i;
        }

        if (low_idx == high_idx) {
            *power = interpolate(temperature,
                    vt_temperatures,
                    &little_power_table[low_idx][0],
                    POWER_TABLE_NUM_TEMP);
        } else {
            unsigned long dvt =
                little_vt_voltages[high_idx] - little_vt_voltages[low_idx];
            unsigned long power1, power2;

            power1 = interpolate(temperature,
                    vt_temperatures,
                    &little_power_table[high_idx][0],
                    POWER_TABLE_NUM_TEMP);

            power2 = interpolate(temperature,
                    vt_temperatures,
                    &little_power_table[low_idx][0],
                    POWER_TABLE_NUM_TEMP);

            *power = (power1 - power2) * (u_volt - little_vt_voltages[low_idx]);
            do_div(*power, dvt);
            *power += power2;
        }
    }

    // convert to mw
    do_div(*power, 1000);
    return 0;
}
U32 GetCpuTemperature(U8 dwCluster)
{
    U32     dwTempData = 0;

    mutex_lock(&MDrvDvfsCpuTempMutex);
    if(hMstarDvfsInfo[dwCluster].dwRefTemperature == 0)
    {
        MHalDvfsRefTemperature(dwCluster);
    }

    //Read CH8 of PM_SAR to Get CPU Temperature
    if (dwCluster == 0)
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001420 << 1)) &= ~0x1;;
    }
    else
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001420 << 1)) |= 0x01;
    }
    udelay(CONFIG_DVFS_CLOCK_DELAY_US);
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) |= (0x01 << 14);
    dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00148e << 1));

    if(dwTempData == 0)
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) = 0x0A20;

        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) |= (0x01 << 14);

        dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00148e << 1));
    }

    if(hMstarDvfsInfo[dwCluster].dwRefTemperature >= dwTempData)
    {
        dwTempData = (((hMstarDvfsInfo[dwCluster].dwRefTemperature - dwTempData) * 1280) + CONFIG_DVFS_T_SENSOR_SHIFT);
    }
    else
    {
        dwTempData = ((dwTempData - hMstarDvfsInfo[dwCluster].dwRefTemperature) * 1280);
        dwTempData = (CONFIG_DVFS_T_SENSOR_SHIFT - dwTempData);
    }

    mutex_unlock(&MDrvDvfsCpuTempMutex);
    return ((dwTempData) / 1000);
}

static int get_temp(void *data,int *temp)
{
    U32 temperature_0 = 0,temperature_1 = 0;
    temperature_0 = GetCpuTemperature(0); //get cluster 0 temperature
    temperature_1 = GetCpuTemperature(1); //get cluster 1 temperature
    *temp = max(temperature_0,temperature_1);
    *temp *=1000;
    return 0;
}

static struct thermal_zone_of_device_ops mstar_of_ops = {
    .get_temp = get_temp,
};

static int mstar_thermal_probe(struct platform_device *pdev)
{
    struct mstar_sensor *sensor_data = &mstar_temp_sensor;
    struct device_node *np;
    int cpu;
    int i, j;
    platform_set_drvdata(pdev, sensor_data);
    for_each_possible_cpu(cpu) {
        int cluster_id = topology_physical_package_id(cpu);
        if (cluster_id > NUM_CLUSTERS) {
            dev_warn(&pdev->dev, "Cluster id: %d > %d\n",
                 cluster_id, NUM_CLUSTERS);
            goto error;
        }

        cpumask_set_cpu(cpu, &sensor_data->cluster[cluster_id]);
    }

    for (i = 0, j = 0; i < NUM_CLUSTERS; i++) {
        char node[16];

        snprintf(node, 16, "cluster%d", i);
        np = of_find_node_by_name(NULL, node);

        if (!np) {
            dev_info(&pdev->dev, "Node not found: %s\n", node);
            continue;
        }
        sensor_data->cdevs[j] =
            of_cpufreq_power_cooling_register(np,
                              &sensor_data->cluster[i],
                              cluster_data[i].dyn_coeff,
                              get_static_power);

        if (IS_ERR(sensor_data->cdevs[i])) {
            dev_warn(&pdev->dev,
                "Error registering cooling device: %s\n", node);
            continue;
        }
        else {
            dev_warn(&pdev->dev,
                "Registering cooling device: %s\n", node);
        }
        j++;
    }

    sensor_data->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, sensor_data,
                                                     &mstar_of_ops);
    if (IS_ERR(sensor_data->tzd)) {
        dev_warn(&pdev->dev, "Error registering sensor: %ld\n", PTR_ERR(sensor_data->tzd));
        return PTR_ERR(sensor_data->tzd);
    }
    return 0;

error:
    return -ENODEV;
}

static int mstar_thermal_remove(struct platform_device *pdev)
{
    struct mstar_sensor *sensor = platform_get_drvdata(pdev);

    thermal_zone_device_unregister(sensor->tzd);
    platform_set_drvdata(pdev, NULL);

    return 0;
}

static struct of_device_id mstar_thermal_of_match[] = {
    { .compatible = "mstar-thermal" },
    {},
};
MODULE_DEVICE_TABLE(of, mstar_thermal_of_match);

static struct platform_driver mstar_thermal_platdrv = {
    .driver = {
        .name       = "mstar-thermal",
        .owner      = THIS_MODULE,
        .of_match_table = mstar_thermal_of_match,
    },
    .probe  = mstar_thermal_probe,
    .remove = mstar_thermal_remove,
};


static int __init mstar_power_allocation_init(void)
{
    int i = 0;
    int j = 0;
    int cpu;
    for (i = 0; i < BIG_POWER_TABLE_NUM_VOLT ; i++)
    {
        big_power_table[i] = kzalloc(sizeof(unsigned int) * POWER_TABLE_NUM_TEMP ,GFP_KERNEL);
        if (!big_power_table[i])
        {
            DVFS_HAL_DEBUG("power_allocation big_power_table kzalloc fail\n");
	    return 0;
        }
        little_power_table[i] = kzalloc(sizeof(unsigned int) * POWER_TABLE_NUM_TEMP ,GFP_KERNEL);
        if (!little_power_table[i])
        {
            DVFS_HAL_DEBUG("power_allocation little_power_table kzalloc fail\n");
            return 0;
        }
        DVFS_HAL_INFO("DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type:%d\n",DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type);
        if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 0)// SS Corner IC
        {
            memcpy(big_power_table[i],SS_big_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
            memcpy(little_power_table[i],SS_little_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
        }
        else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)// TT Corner IC
        {
             memcpy(big_power_table[i],TT_big_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
             memcpy(little_power_table[i],TT_little_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
        }
        else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 2)// FF Corner IC
        {
            memcpy(big_power_table[i],FF_big_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
            memcpy(little_power_table[i],FF_little_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
        }
        else
        {
            DVFS_HAL_DEBUG("Power_allocation error, Check corner IC information\n");
        }
        for (j = 0 ; j < POWER_TABLE_NUM_TEMP ; j++)
        {
            DVFS_HAL_INFO("big_power_table[%d][%d]:%d\n",i,j,big_power_table[i][j]);
            DVFS_HAL_INFO("little_power_table[%d][%d]:%d\n",i,j,little_power_table[i][j]);
        }
    }
    big_vt_voltages = kzalloc(sizeof(int) * BIG_POWER_TABLE_NUM_VOLT ,GFP_KERNEL);
    if (!big_vt_voltages)
    {
        DVFS_HAL_DEBUG("power_allocation big_vt_voltages kzalloc fail\n");
        return 0;
    }
    little_vt_voltages = kzalloc(sizeof(int) * LITTLE_POWER_TABLE_NUM_VOLT ,GFP_KERNEL);
    if (!little_vt_voltages)
    {
        DVFS_HAL_DEBUG("power_allocation little_vt_voltages kzalloc fail\n");
        return 0;
    }
    if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 0)// SS Corner IC
    {
        memcpy(big_vt_voltages,SS_big_vt_voltages,sizeof(int)*BIG_POWER_TABLE_NUM_VOLT);
        memcpy(little_vt_voltages,SS_little_vt_voltages,sizeof(int)*LITTLE_POWER_TABLE_NUM_VOLT);
    }
    else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)// TT Corner IC
    {
        memcpy(big_vt_voltages,TT_big_vt_voltages,sizeof(int)*BIG_POWER_TABLE_NUM_VOLT);
        memcpy(little_vt_voltages,TT_little_vt_voltages,sizeof(int)*LITTLE_POWER_TABLE_NUM_VOLT);
    }
    else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 2)// FF Corner IC
    {
        memcpy(big_vt_voltages,FF_big_vt_voltages,sizeof(int)*BIG_POWER_TABLE_NUM_VOLT);
        memcpy(little_vt_voltages,FF_little_vt_voltages,sizeof(int)*LITTLE_POWER_TABLE_NUM_VOLT);
    }
    else
    {
        DVFS_HAL_DEBUG("Power_allocation error, Check corner IC information\n");
    }
    for (j = 0 ; j < POWER_TABLE_NUM_TEMP ; j++)
    {
        DVFS_HAL_INFO("big_vt_voltages[%d]:%d\n",i,big_vt_voltages[j]);
		DVFS_HAL_INFO("little_vt_voltages[%d]:%d\n",i,little_vt_voltages[j]);
    }

    for_each_possible_cpu(cpu)
    {
        struct device *dev = NULL;
        struct dev_pm_opp *opp;
	unsigned long freq;
        dev = get_cpu_device(cpu);
        for(freq = 0; opp = dev_pm_opp_find_freq_ceil(dev, &freq),!IS_ERR(opp); freq++)
	{
	    if (freq > NORMAL_CASE_MAX_FREQUENCY ) //disable > 1.4Ghz special case
	    {
                DVFS_HAL_INFO("little_vt_voltages[%d]:%d\n",i,little_vt_voltages[j]);
                dev_pm_opp_disable(dev, freq);
	    }
	}
    }
    platform_driver_register(&mstar_thermal_platdrv);
    DVFS_HAL_INFO("Register power_allocation driver");
    return 0;
}

#endif //CONFIG_THERMAL_GOV_POWER_ALLOCATOR

static int __init mstar_dvfs_init(void)
{
    halTotalClusterNumber = DVFS_HAL_TOTAL_CLUSTER_NUM;
    classify_cpu_cluster();
    platform_device_register(&mstar_dvfs_dev);
    platform_driver_register(&mstar_dvfs_driver);
    return 0;
}

static void __init mstar_dvfs_exit(void)
{
    platform_device_unregister(&mstar_dvfs_dev);
    platform_driver_unregister(&mstar_dvfs_driver);
}
#ifdef CONFIG_THERMAL_GOV_POWER_ALLOCATOR
late_initcall(mstar_power_allocation_init);
#endif

//i2c drive (Level 3) rmust earlier than dvfs driver
//core_initcall(mstar_dvfs_init);       // Level 1
subsys_initcall(mstar_dvfs_init);       // Level 4

module_exit(mstar_dvfs_exit);

//=================================================================================================
