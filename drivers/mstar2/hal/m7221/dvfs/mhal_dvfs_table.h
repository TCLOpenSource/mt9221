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

#ifndef __MHAL_DVFS_TABLE__
#define __MHAL_DVFS_TABLE__
#define CONFIG_DVFS_CORNER_CHIP_MAX                     0x03
#define CONFIG_DVFS_POWER_CTL_SEGMENT                   4
#define DVFS_CLUSTER_NUM                                2
#define CONFIG_DVFS_PACKAGE_SEGMENT                     2
#define CONFIG_DVFS_TEMPERATURE_DISABLE                 0xFFFF
#define CONFIG_DVFS_CPU_CLOCK_DISABLE                   0xFFFF
typedef struct
{
    U32     dwLowerCpuClock;    //Lower Level Threshold CPU Clock
    U32     dwUpperCpuClock;    //Upper Level Threshold CPU Clock
} MSTAR_DVFS_CPU_INFO;

typedef struct
{
    U32     dwLowerGpuClock;    //Lower Level Threshold GPU Clock
    U32     dwUpperGpuClock;    //Upper Level Threshold GPU Clock
} MSTAR_DVFS_GPU_INFO;

typedef struct
{
    U32     dwCpuPower;         //The Value of CPU Power
    U32     dwCorePower;        //The Value of Core Power
} MSTAR_DVFS_POWER_INFO;

typedef struct
{
    S32     dwLowerFreezeTemperature;   //Lower Level Threshold Temperature for Freeze Mode
    S32     dwUpperFreezeTemperature;   //Upper Level Threshold Temperature for Freeze Mode
    S32     dwLowerLevelTemperature;    //Lower Level Threshold Temperature
    S32     dwUpperLevelTemperature;    //Upper Level Threshold Temperature
    S32     dwMaxLevelTemperature;      //Maximum Level Threshold Temperature
} MSTAR_DVFS_TEMPERATURE_INFO;

typedef struct
{
    MSTAR_DVFS_CPU_INFO         DvfsCpuInfo;                //CPU Clock
    MSTAR_DVFS_GPU_INFO         DvfsGpuInfo;                //GPU Clock
    MSTAR_DVFS_POWER_INFO       DvfsPowerInfo[CONFIG_DVFS_CORNER_CHIP_MAX]; //Power Settings for Corner Chip
} MSTAR_DVFS_ENTRY_INFO;

typedef struct
{
    U32     dwDefaultCpuClock;         //Default CPU Clock
    U32     dwMinimumCpuClock;         //Minimum CPU Clock
    U32     dwMaximumCpuClock;         //Maximum CPU Clock
    U32     dwIRBoostCpuClock;         //IRBoost CPU Clock
    U32     dwFreezeMaximumCpuClock;   //Maximum CPU Clock in Freeze Mode
    U32     dwNormalMaximumCpuClock;   //Maximum CPU CLock in Normal Mode
    U32     dwAntutuProtectedCpuClock; //Protect Mode Antutu Clock
    U32     dwProtectedCpuClock;       //Protect Mode CPU Clock
    U32     dwDefaultCpuPower;         //Default CPU Power
    U32     dwDefaultCorePower;        //Default Core Power
    U32     dwMaximumCpuPower;         //Maximum CPU Power
    U32     dwMinimumCpuPower;         //Mininum CPU Power
} MSTAR_DVFS_SYS_INFO;

typedef struct
{
    MSTAR_DVFS_SYS_INFO         DvfsSysInfo;                //System Configuration
    MSTAR_DVFS_TEMPERATURE_INFO DvfsTemperatureInfo;        //Temperature
    MSTAR_DVFS_ENTRY_INFO       DvfsInitModeInfo;           //Initial Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsBootModeInfo;           //Boot Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsOverTemperatureModeInfo;//Over-temperature Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsFreezeModeInfo[CONFIG_DVFS_POWER_CTL_SEGMENT];  //Freeze Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsNormalModeInfo[CONFIG_DVFS_POWER_CTL_SEGMENT];  //Normal Mode
} MSTAR_DVFS_MODE_INFO;

typedef struct
{
    U32     bDvfsInitOk;
    U32     bDvfsModeChange;
    U32     dwMaxCpuClockByTemperature;
    U32     dwFinalCpuClock;
    U32     dwFinalCpuPowerVoltage;
    U32     dwFinalCorePowerVoltage;
    S32     dwCpuTemperature;
    U32     dwRefTemperature;
    U32     dwAvgCpuTempCounter;
    U32     dwAvgCpuTempBuffer;
    U32     bSystemResumeFlag;
    U32     dwResetCounter;
    U32     dwTemperatureCounter;
    U32     dwBootTimeCounter;
    U32     dwOverTemperatureFlag;
    U32     dwCpuPartId;
    U32     dwClusterCpuMask;
    U32     dwFreqRiuAddr;
    U32     dwSpecialCpuClkddr;
    U32     dwAnaMiscBank;
    U32     dwMcuArmBank;
    U32     dwPradoProtect;
    U32     dwPradoCounter;
    U32     dwClusterPowerIndex;
    MSTAR_DVFS_MODE_INFO    DvfsModeInfo[CONFIG_DVFS_PACKAGE_SEGMENT];
} MSTAR_DVFS_INFO;

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
        .dwCpuPartId                    = 0xd03,
        .dwClusterCpuMask               = 0x0,
        .dwFreqRiuAddr                  = 0x1f200a04,
        .dwSpecialCpuClkddr             = 0x1f200a0c,
        .dwAnaMiscBank                  = 0x110c00,
        .dwMcuArmBank                   = 0x101d00,
        .dwPradoProtect                 = 0,
        .dwPradoCounter                 = 0,
        .dwClusterPowerIndex            = 0,

        .DvfsModeInfo =
        {
            //CA53
            {
                //U01
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1050,
                    .dwMinimumCpuClock          = 1050,
                    .dwMaximumCpuClock          = 1400,
                    .dwIRBoostCpuClock          = 1300,
                    .dwFreezeMaximumCpuClock    = 1400,
                    .dwNormalMaximumCpuClock    = 1400,
                    .dwAntutuProtectedCpuClock  = 1400,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 105,
                    .dwDefaultCorePower         = 105,
                    .dwMaximumCpuPower          = 120,
                    .dwMinimumCpuPower          = 95,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature   = 120,
                    .dwUpperLevelTemperature   = 135,
                    .dwMaxLevelTemperature   = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1050,
                        .dwUpperCpuClock        = 1050,
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
                            .dwCorePower        = 105,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 105,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 100,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1300,
                        .dwUpperCpuClock        = 1300,
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
                            .dwCorePower        = 105,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 105,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 100,
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
                            .dwCorePower        = 105,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 105,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 100,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1050MHz - 1199MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1050,
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
                                .dwCpuPower     = 105,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1200MHz - 1299MHz
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
                                .dwCpuPower     = 110,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1399MHz
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
                                .dwCpuPower     = 115,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1400MHz
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
                                .dwCpuPower     = 120,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1050MHz - 1199MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1050,
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
                                .dwCpuPower     = 105,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1200MHz - 1299MHz
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
                                .dwCpuPower     = 110,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1399MHz
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
                                .dwCpuPower     = 115,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1400MHz
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
                                .dwCpuPower     = 120,
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                },
            },
            {
                //U02
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1450,
                    .dwIRBoostCpuClock          = 1350,
                    .dwFreezeMaximumCpuClock    = 1450,
                    .dwNormalMaximumCpuClock    = 1450,
                    .dwAntutuProtectedCpuClock  = 1450,
                    .dwProtectedCpuClock        = 950,
                    .dwDefaultCpuPower          = 105,
                    .dwDefaultCorePower         = 102,
                    .dwMaximumCpuPower          = 120,
                    .dwMinimumCpuPower          = 95,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature   = 120,
                    .dwUpperLevelTemperature   = 135,
                    .dwMaxLevelTemperature   = 160,
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
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 102,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 102,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1350,
                        .dwUpperCpuClock        = 1350,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 116,
                            .dwCorePower        = 102,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 116,
                            .dwCorePower        = 102,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 112,
                            .dwCorePower        = 97,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 950,
                        .dwUpperCpuClock        = 950,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 102,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 102,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1100MHz - 1249MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1250,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1250MHz - 1349MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1250,
                            .dwUpperCpuClock    = 1350,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1350MHz - 1449MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1350,
                            .dwUpperCpuClock    = 1450,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 112,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1450MHz - 1450MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1450,
                            .dwUpperCpuClock    = 1451,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1100MHz - 1249MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1250,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1250MHz - 1349MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1250,
                            .dwUpperCpuClock    = 1350,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1350MHz - 1449MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1350,
                            .dwUpperCpuClock    = 1450,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 112,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1450MHz - 1450MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1450,
                            .dwUpperCpuClock    = 1451,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
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
        .dwCpuPartId                    = 0xd09,
        .dwClusterCpuMask               = 0x0,
        .dwFreqRiuAddr                  = 0x1f200a14,
        .dwSpecialCpuClkddr             = 0x1f200a1c,
        .dwAnaMiscBank                  = 0x313a00,
        .dwMcuArmBank                   = 0x313300,
        .dwPradoProtect                 = 0,
        .dwPradoCounter                 = 0,
        .dwClusterPowerIndex            = 0,

        .DvfsModeInfo =
        {
            //CA73
            {
                //U01
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1500,
                    .dwIRBoostCpuClock          = 1400,
                    .dwFreezeMaximumCpuClock    = 1500,
                    .dwNormalMaximumCpuClock    = 1500,
                    .dwAntutuProtectedCpuClock  = 1500,
                    .dwProtectedCpuClock        = 900,
                    .dwDefaultCpuPower          = 105,
                    .dwDefaultCorePower         = 105,
                    .dwMaximumCpuPower          = 120,
                    .dwMinimumCpuPower          = 95,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature   = 120,
                    .dwUpperLevelTemperature   = 135,
                    .dwMaxLevelTemperature   = 160,
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
                            .dwCpuPower         = 105,
                            .dwCorePower        = 105,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 105,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 100,
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
                            .dwCorePower        = 105,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 105,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 100,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 900,
                        .dwUpperCpuClock        = 900,
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
                            .dwCorePower        = 105,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 105,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 100,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1100MHz - 1249MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1250,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1250MHz - 1399MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1250,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1499MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1500,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1500MHz - 1500MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1500,
                            .dwUpperCpuClock    = 1501,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1100MHz - 1249MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1100,
                            .dwUpperCpuClock    = 1250,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1250MHz - 1399MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1250,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1400MHz - 1499MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1500,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1500MHz - 1500MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1500,
                            .dwUpperCpuClock    = 1501,
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
                                .dwCorePower    = 105,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 105,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 100,
                            },
                        },
                    },
                },
            },
            {
                //U02
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1150,
                    .dwMinimumCpuClock          = 1150,
                    .dwMaximumCpuClock          = 1550,
                    .dwIRBoostCpuClock          = 1500,
                    .dwFreezeMaximumCpuClock    = 1550,
                    .dwNormalMaximumCpuClock    = 1550,
                    .dwAntutuProtectedCpuClock  = 1550,
                    .dwProtectedCpuClock        = 1000,
                    .dwDefaultCpuPower          = 105,
                    .dwDefaultCorePower         = 102,
                    .dwMaximumCpuPower          = 120,
                    .dwMinimumCpuPower          = 95,
                },
                .DvfsTemperatureInfo =
                {
                    .dwLowerFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwUpperFreezeTemperature   = CONFIG_DVFS_TEMPERATURE_DISABLE,
                    .dwLowerLevelTemperature   = 120,
                    .dwUpperLevelTemperature   = 135,
                    .dwMaxLevelTemperature   = 160,
                },
                .DvfsInitModeInfo =
                {
                    //Initial Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1150,
                        .dwUpperCpuClock        = 1150,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 102,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 105,
                            .dwCorePower        = 102,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 97,
                        },
                    },
                },
                .DvfsBootModeInfo =
                {
                    //Boot Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1500,
                        .dwUpperCpuClock        = 1500,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 116,
                            .dwCorePower        = 102,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 116,
                            .dwCorePower        = 102,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 112,
                            .dwCorePower        = 97,
                        },
                    },
                },
                .DvfsOverTemperatureModeInfo =
                {
                    //Over Temperature Mode
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock        = 1000,
                        .dwUpperCpuClock        = 1000,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 102,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 102,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 97,
                        },
                    },
                },
                .DvfsFreezeModeInfo =
                {
                    {
                        //Freeze Mode: 1150MHz - 1299MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1150,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1300MHz - 1499MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1500,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1500MHz - 1549MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1500,
                            .dwUpperCpuClock    = 1550,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 112,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1550MHz - 1550MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1550,
                            .dwUpperCpuClock    = 1551,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                },
                .DvfsNormalModeInfo =
                {
                    {
                        //Normal Mode: 1150MHz - 1299MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1150,
                            .dwUpperCpuClock    = 1300,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1300MHz - 1499MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1300,
                            .dwUpperCpuClock    = 1500,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1500MHz - 1549MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1500,
                            .dwUpperCpuClock    = 1550,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 116,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 112,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                    {
                        //Normal Mode: 1550MHz - 1550MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1550,
                            .dwUpperCpuClock    = 1551,
                        },
                        .DvfsGpuInfo =
                        {
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 102,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 97,
                            },
                        },
                    },
                },
            },
        },
    },
};
#endif
