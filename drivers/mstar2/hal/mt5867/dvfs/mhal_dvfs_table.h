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
#define CONFIG_DVFS_POWER_CTL_SEGMENT                   5
#define DVFS_CLUSTER_NUM                                1
#define CONFIG_DVFS_PACKAGE_SEGMENT                     1
#define CONFIG_DVFS_TEMPERATURE_DISABLE                 0xFFFF
#define CONFIG_DVFS_CPU_CLOCK_DISABLE                   0xFFFF

#define DVFS_DEVICE_TREE_PARSE_ENABLE                   1

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
    U32     dwMinimumCpuPower;         //Mininum CPU Powe
} MSTAR_DVFS_SYS_INFO;

typedef struct
{
    MSTAR_DVFS_SYS_INFO         DvfsSysInfo;                //System Configuration
    MSTAR_DVFS_TEMPERATURE_INFO DvfsTemperatureInfo;        //Temperature
    MSTAR_DVFS_ENTRY_INFO       DvfsInitModeInfo;           //Initial Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsBootModeInfo;           //Boot Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsOverTemperatureModeInfo;//Over-temperature Mode
#if (DVFS_DEVICE_TREE_PARSE_ENABLE == 1)
    MSTAR_DVFS_ENTRY_INFO       *DvfsFreezeModeInfo;
    MSTAR_DVFS_ENTRY_INFO       *DvfsNormalModeInfo;
#else
    MSTAR_DVFS_ENTRY_INFO       DvfsFreezeModeInfo[CONFIG_DVFS_POWER_CTL_SEGMENT];  //Freeze Mode
    MSTAR_DVFS_ENTRY_INFO       DvfsNormalModeInfo[CONFIG_DVFS_POWER_CTL_SEGMENT];  //Normal Mode
#endif
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
    U32     dwAnaMiscBank;
    U32     dwMcuArmBank;

#if (DVFS_DEVICE_TREE_PARSE_ENABLE == 1)
    MSTAR_DVFS_MODE_INFO    *DvfsModeInfo;
#else
    MSTAR_DVFS_MODE_INFO    DvfsModeInfo[CONFIG_DVFS_PACKAGE_SEGMENT];
#endif
} MSTAR_DVFS_INFO;


#if (DVFS_DEVICE_TREE_PARSE_ENABLE == 1)
static MSTAR_DVFS_INFO *hMstarDvfsInfo = NULL;
#define DVFS_DTS_READ_U32(node, name, value, pos) \
    if (!of_property_read_u32(node, name, &value)) \
        pos = value; \
    else \
    {   \
        DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: %s\033[m\n",name); \
        bRet = FALSE; \
    }

#define DVFS_DTS_READ_STRING(node, name, value, pos) \
    if (!of_property_read_string(node, name, &value)) \
        pos = value; \
    else \
    {   \
        DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: %s\033[m\n",name); \
        bRet = FALSE; \
    }

#define DVFS_MIN_CLOCK_LEVEL(node,cluster,package,type,level,prop)   \
    if (type == CONFIG_DVFS_FREEZE_MODE) \
        DVFS_DTS_READ_U32(node,"min-clock", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsFreezeModeInfo[level].DvfsCpuInfo.dwLowerCpuClock) \
    else if (type == CONFIG_DVFS_NORMAL_MODE)   \
        DVFS_DTS_READ_U32(node,"min-clock", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsCpuInfo.dwLowerCpuClock) 

#define DVFS_MAX_CLOCK_LEVEL(node,cluster,package,type,level,prop)   \
    if (type == CONFIG_DVFS_FREEZE_MODE) \
        DVFS_DTS_READ_U32(node,"max-clock", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsFreezeModeInfo[level].DvfsCpuInfo.dwUpperCpuClock) \
    else if (type == CONFIG_DVFS_NORMAL_MODE)   \
        DVFS_DTS_READ_U32(node,"max-clock", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsCpuInfo.dwUpperCpuClock) 

#define DVFS_POWER_INIT(node,cluster,package,type,level,prop) \
    if (type == CONFIG_DVFS_FREEZE_MODE) \
    { \
        DVFS_DTS_READ_U32(node,"power-ss", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsFreezeModeInfo[level].DvfsPowerInfo[0].dwCpuPower)  \
        DVFS_DTS_READ_U32(node,"power-ss", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsFreezeModeInfo[level].DvfsPowerInfo[1].dwCpuPower)  \
        DVFS_DTS_READ_U32(node,"power-ff", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsFreezeModeInfo[level].DvfsPowerInfo[2].dwCpuPower)  \
    } \
    else if (type == CONFIG_DVFS_NORMAL_MODE)   \
    { \
        DVFS_DTS_READ_U32(node,"power-ss", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[0].dwCpuPower)  \
        DVFS_DTS_READ_U32(node,"power-ss", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[1].dwCpuPower)  \
        DVFS_DTS_READ_U32(node,"power-ff", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[2].dwCpuPower)  \
    }

#define DVFS_TABLE_PAIR_INIT_LEVEL(node, cluster, package, type,level,prop) \
    DVFS_MIN_CLOCK_LEVEL(node,cluster,package,type,level,prop); \
    DVFS_MAX_CLOCK_LEVEL(node,cluster,package,type,level,prop); \
    DVFS_POWER_INIT(node,cluster,package,type,level,prop);


#define DVFS_SYS_INIT(node, cluster,package,prop) \
    DVFS_DTS_READ_U32(child, "default-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwDefaultCpuClock) \
    DVFS_DTS_READ_U32(child, "min-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwMinimumCpuClock) \
    DVFS_DTS_READ_U32(child, "max-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwMaximumCpuClock) \
    DVFS_DTS_READ_U32(child, "max-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwFreezeMaximumCpuClock) \
    DVFS_DTS_READ_U32(child, "max-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwNormalMaximumCpuClock) \
    DVFS_DTS_READ_U32(child, "boost-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwIRBoostCpuClock) \
    DVFS_DTS_READ_U32(child, "antutu-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwAntutuProtectedCpuClock) \
    DVFS_DTS_READ_U32(child, "protect-clock", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwProtectedCpuClock) \
    DVFS_DTS_READ_U32(child, "default-cpu", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwDefaultCpuPower) \
    DVFS_DTS_READ_U32(child, "max-cpu", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwMaximumCpuPower) \
    DVFS_DTS_READ_U32(child, "min-cpu", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwMinimumCpuPower)


#define DVFS_TEMP_INIT(node, cluster,package,prop) \
    DVFS_DTS_READ_U32(child, "freeze_lower", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsTemperatureInfo.dwLowerFreezeTemperature) \
    DVFS_DTS_READ_U32(child, "freeze_upper", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsTemperatureInfo.dwUpperFreezeTemperature) \
    DVFS_DTS_READ_U32(child, "normal_lower", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsTemperatureInfo.dwLowerLevelTemperature) \
    DVFS_DTS_READ_U32(child, "normal_upper", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsTemperatureInfo.dwUpperLevelTemperature) \
    DVFS_DTS_READ_U32(child, "reset", prop,hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsTemperatureInfo.dwMaxLevelTemperature)

#else
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

        .dwCpuPartId                    = 0xd03,        //CA53 (MT5862)
        .dwClusterCpuMask               = 0x0,          //This will be used in classify_cpu_cluster for cluster mask.
        .dwFreqRiuAddr                  = 0x1F200A04,   //bus address
        .dwAnaMiscBank                  = 0x110c00,
        .dwMcuArmBank                   = 0x101d00,

        .DvfsModeInfo =
        {
            //CA55
            {
                //Normal
                .DvfsSysInfo =
                {
                    .dwDefaultCpuClock          = 1100,
                    .dwMinimumCpuClock          = 1100,
                    .dwMaximumCpuClock          = 1450,
                    .dwIRBoostCpuClock          = 1400,
                    .dwFreezeMaximumCpuClock    = 1450,
                    .dwNormalMaximumCpuClock    = 1450,
                    .dwAntutuProtectedCpuClock  = 1450,
                    .dwProtectedCpuClock        = 850,
                    .dwDefaultCpuPower          = 100,
                    .dwDefaultCorePower         = 95,
                    .dwMaximumCpuPower          = 120,
                    .dwMinimumCpuPower          = 90,
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
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 98,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 100,
                            .dwCorePower        = 98,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 95,
                            .dwCorePower        = 93,
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
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 98,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 115,
                            .dwCorePower        = 98,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 110,
                            .dwCorePower        = 93,
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
                        .dwLowerGpuClock        = 600,
                        .dwUpperGpuClock        = 600,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 98,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 98,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower         = 90,
                            .dwCorePower        = 93,
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
                                .dwCpuPower     = 100,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 93,
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
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 93,
                            },
                        },
                    },
                    {
                        //Freeze Mode: 1400MHz - 1449MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
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
                                .dwCpuPower     = 115,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 93,
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
                            //Freeze Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 93,
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
                                .dwCpuPower     = 100,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 100,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 95,
                                .dwCorePower    = 93,
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
                            .dwLowerGpuClock    = 600,
                            .dwUpperGpuClock    = 600,
                        },
                        .DvfsPowerInfo =
                        {
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 105,
                                .dwCorePower    = 93,
                            },
                        },

                    },
                    {
                        //Normal Mode: 1400MHz - 1449MHz
                        .DvfsCpuInfo =
                        {
                            .dwLowerCpuClock    = 1400,
                            .dwUpperCpuClock    = 1449,
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
                                .dwCpuPower     = 115,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 110,
                                .dwCorePower    = 93,
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
                            //Normal Mode
                            {
                                //SS Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 98,
                            },
                            {
                                //TT Corner Chip
                                .dwCpuPower     = 120,
                                .dwCorePower    = 98,
                            },
                            {
                                //FF Corner Chip
                                .dwCpuPower     = 115,
                                .dwCorePower    = 93,
                            },
                        },
                    },
                },
            },
        },
    },

#if (DVFS_CLUSTER_NUM == 2)
    {
        /* TBD */
    },
#endif
};
#endif
#endif


