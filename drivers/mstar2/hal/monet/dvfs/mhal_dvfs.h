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

#ifndef __MHAL_DVFS_H__
#define __MHAL_DVFS_H__

#ifndef __MDRV_TYPES_H__
#include "mdrv_types.h"
#endif

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define DVFS_HAL_INFO(x, args...)                       {printk(KERN_INFO x, ##args);}
#define DVFS_HAL_DEBUG(x, args...)                      {printk(KERN_DEBUG x, ##args);}

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define CONFIG_REGISTER_BASE_ADDRESS                    0xfd000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define CONFIG_REGISTER_BASE_ADDRESS                    mstar_pm_base
#endif

#define CONFIG_DVFS_ENABLE_PATTERN                      0x3697
#define CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT           0x2454

#define CONFIG_DVFS_DATA_COUNT                          5
#define CONFIG_DVFS_AVERAGE_COUNT                       5
#define CONFIG_DVFS_RESET_MAX_COUNT                     5
#define CONFIG_DVFS_CLOCK_DELAY_US                      100
#define CONFIG_DVFS_MUTEX_WAIT_TIME                     50
#define CONFIG_DVFS_BOOT_MODE_TIME                      30

#define CONFIG_DVFS_CORNER_CHIP_SS                      0x00
#define CONFIG_DVFS_CORNER_CHIP_TT                      0x01
#define CONFIG_DVFS_CORNER_CHIP_FF                      0x02
#define CONFIG_DVFS_CORNER_CHIP_MAX                     0x03
#define CONFIG_DVFS_CORNER_CHIP_UNKNOWN                 0xFF

#define CONFIG_DVFS_INIT_MODE                           0x00
#define CONFIG_DVFS_FREEZE_MODE                         0x01
#define CONFIG_DVFS_NORMAL_MODE                         0x02
#define CONFIG_DVFS_OVER_TEMPERATURE_MODE               0x03
#define CONFIG_DVFS_SPECIAL_CLOCK_MODE                  0x04
#define CONFIG_DVFS_BOOT_MODE                           0x05
#define CONFIG_DVFS_CORNER_CHIP_UNKNOWN                 0xFF

#define CONFIG_DVFS_CPU_CLOCK_MAX                       MDrvDvfsQueryCpuClock(CONFIG_DVFS_MAX_CPU_CLOCK)
#define CONFIG_DVFS_CPU_CLOCK_MIN                       MDrvDvfsQueryCpuClock(CONFIG_DVFS_MIN_CPU_CLOCK)
#define CONFIG_DVFS_CPU_CLOCK_SPECIAL                   100
#define CONFIG_DVFS_CPU_CLOCK_DISABLE                   0xFFFF
#define CONFIG_DVFS_TEMPERATURE_DISABLE                 0xFFFF

#define CONFIG_DVFS_PACKAGE_SEGMENT                     3
#define CONFIG_DVFS_POWER_CTL_SEGMENT                   4

#define CONFIG_DVFS_T_SENSOR_SHIFT                      27000

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
typedef struct {
    U32 GPU_TEMP;
    U32 CPU_TEMP;
    U32 INT_TEMP;
} DVFS_THERMAL_INFO;

typedef struct  __attribute__((packed))
{
    U32     reg_vid_dvfs_id: 16;        //0x1005_00[15:0]: Identify VID Only or VID with DVFS
    U32     reg_reserved_00: 16;        //0x1005_00[31:16]: Reserved

    U32     reg_cur_cpu_clock: 12;      //0x1005_02[11:0]: Current Value of CPU Clock
    U32     reg_reserved_01: 20;        //0x1005_02[31:12]: Reserved

    U32     reg_cur_cpu_temp: 8;        //0x1005_04[7:0]: Current Value of CPU Temperature
    U32     reg_reserved_02: 24;        //0x1005_04[31:8]: Reserved

    U32     reg_vid_enable: 1;          //0x1005_06[0]: Enable VID Flow
    U32     reg_dvfs_enable: 1;         //0x1005_06[1]: Enable DVFS Flow
    U32     reg_chip_package: 2;        //0x1005_06[3:2]: Chip Package Type
    U32     reg_cpu_pwr_type: 2;        //0x1005_06[5:4]: CPU Power Type
    U32     reg_core_pwr_type: 2;       //0x1005_06[7:6]: Core Power Type
    U32     reg_cpu_pwr_ctl: 2;         //0x1005_06[9:8]: IO Control Mode for CPU Power
    U32     reg_core_pwr_ctl: 2;        //0x1005_06[11:10]: IO Control Mode for Core Power
    U32     reg_cur_dvfs_state: 3;      //0x1005_06[14:12]: Current DVFS Running State
    U32     reg_reserved_03_1: 17;      //0x1005_06[31:15]: Reserved

    U32     reg_low_high_temp_en: 1;    //0x1005_08[0]: Enable Lower/Upper Level Threshold of Temperature
    U32     reg_max_temp_en: 1;         //0x1005_08[1]: Enable Maximum Level Threshold of Temperature
    U32     reg_reserved_04_0: 6;       //0x1005_08[7:2]: Reserved
    U32     reg_low_level_temp: 8;      //0x1005_08[15:8]: Lower Level Threshold of Temperature
    U32     reg_reserved_04_1: 16;      //0x1005_08[31:16]: Reserved

    U32     reg_high_level_temp: 8;     //0x1005_0A[7:0]: Upper Level Threshold of Temperature
    U32     reg_max_level_temp: 8;      //0x1005_0A[15:8]: Maximum Level Threshold of Temperature
    U32     reg_reserved_05: 16;        //0x1005_0A[31:0]: Reserved

    U32     reg_reserved_06: 32;        //0x1005_0C[31:0]: Reserved

    U32     reg_special_cpu_clk: 16;    //0x1005_0E[15:0]: User-defined Specific CPU Clock
    U32     reg_reserved_07: 16;        //0x1005_0E[31:16]: Reserved

} MSTAR_DVFS_REG_INFO;

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
    U32     dwDefaultCpuClock;      //Default CPU Clock
    U32     dwMinimumCpuClock;      //Minimum CPU Clock
    U32     dwMaximumCpuClock;      //Maximum CPU Clock
    U32     dwProtectedCpuClock;    //Protect Mode CPU Clock
    U32     dwDefaultCpuPower;      //Default CPU Power
    U32     dwDefaultCorePower;     //Default Core Power

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

    MSTAR_DVFS_MODE_INFO    DvfsModeInfo[CONFIG_DVFS_PACKAGE_SEGMENT];

} MSTAR_DVFS_INFO;

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
U32  MHalDvfsProc(U32 dwCpuClock);
U32  MHalDvfsInit(void);
void MHalDvfsCpuDisplay(void);
void MHalDvfsCpuTemperature(void);
void MHalDvfsCpuClockAdjustment(U32 dwCpuClock);
U32  MHalDvfsSearchCpuClockLevel(U32 dwCpuClock);
void MHalDvfsPowerControl(U32 dwCpuClock);
void MHalDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage);
void MHalDvfsCorePowerAdjustment(U32 dwCorePowerVoltage);
U32  MHalDvfsQueryCpuClock(U32 dwCpuClockType);
U32  MHalDvfsQueryCpuClockByTemperature(void);
U32  MHalDvfsGetCpuFreq(U8 dwCpu);

void MHalDvfsCpuDisplayInit(void);
void MHalDvfsRefTemperature(void);
void MHalDvfsCpuPowerInit(void);
void MHalDvfsCorePowerInit(void);
U32 MHalDvfsGetCpuPowerType(U8 dwCpu);
U32 MHalDvfsGetCorePowerType(U8 dwCpu);

U8 MHalDvfsGetFreqTable(U8 dwCpu, struct cpufreq_frequency_table **freq_table);
U32 MHalDvfsGetOverTemperatureFlag(U8 dwCluster);
void MHalDvfsGetTemperatureInfo(U8 dwCpu, DVFS_THERMAL_INFO *thermal_info);
#endif
