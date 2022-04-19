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
#include <linux/slab.h>
#include <linux/cpufreq.h>

#include <mstar/mstar_chip.h>

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

DEFINE_MUTEX(MHalDvfsGetCpuFreqMutex);
extern void MDrvDvfsVoltageSetup(unsigned int dwCpuClock, unsigned int dwVoltage, unsigned int dwVoltageType);

static volatile MSTAR_DVFS_REG_INFO *DvfsRegInfo = 0;
static MSTAR_DVFS_INFO hMstarDvfsInfo =
{
    .bDvfsInitOk = 0,
    .bDvfsModeChange = 0,

    .dwMaxCpuClockByTemperature = 0,
    .dwFinalCpuClock = 0,

    .dwFinalCpuPowerVoltage = 0,
    .dwFinalCorePowerVoltage = 0,

    .dwCpuTemperature = 0,
    .dwRefTemperature = 0,
    .dwAvgCpuTempCounter = 0,

    .bSystemResumeFlag = 0,
    .dwResetCounter = 0,
    .dwTemperatureCounter = 0,
    .dwBootTimeCounter = 0,
    .dwOverTemperatureFlag = 0,

    .DvfsModeInfo =
    {
        {
            //BGA Package
            .DvfsSysInfo =
            {
                .dwDefaultCpuClock = 1050,
                .dwMinimumCpuClock = 1050,
                .dwMaximumCpuClock = 1400,
                .dwProtectedCpuClock = 850,
                .dwDefaultCpuPower = 100,
                .dwDefaultCorePower = 100,
            },
            .DvfsTemperatureInfo =
            {
                .dwLowerFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                .dwUpperFreezeTemperature = CONFIG_DVFS_TEMPERATURE_DISABLE,
                .dwLowerLevelTemperature = 120,
                .dwUpperLevelTemperature = 135,
                .dwMaxLevelTemperature = 160,
            },
            .DvfsInitModeInfo =
            {
                //Initial Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 1050,    //BGA: 1050MHz
                    .dwUpperCpuClock = 1050,    //BGA: 1050MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //BGA: 500MHz
                    .dwUpperGpuClock = 500,     //BGA: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 100,      //BGA: 1.00V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 100,      //BGA: 1.00V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 100,      //BGA: 1.00V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                },
            },
            .DvfsBootModeInfo =
            {
                //Boot Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 1050,    //BGA: 1050MHz
                    .dwUpperCpuClock = 1400,    //BGA: 1400MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //BGA: 500MHz
                    .dwUpperGpuClock = 500,     //BGA: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 120,      //BGA: 1.20V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 120,      //BGA: 1.20V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 110,      //BGA: 1.10V
                        .dwCorePower = 95,      //BGA: 0.95V
                    },
                },
            },
            .DvfsOverTemperatureModeInfo =
            {
                //Over Temperature Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 850,     //BGA: 850MHz
                    .dwUpperCpuClock = 850,     //BGA: 850MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //BGA: 500MHz
                    .dwUpperGpuClock = 500,     //BGA: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 95,       //BGA: 0.95V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 95,       //BGA: 0.95V
                        .dwCorePower = 100,     //BGA: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 90,       //BGA: 0.90V
                        .dwCorePower = 95,      //BGA: 0.95V
                    },
                },
            },
            .DvfsFreezeModeInfo =
            {
                {
                    //Freeze Mode: 1050MHz - 1200MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1050,
                        .dwUpperCpuClock = 1200,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 95,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Freeze Mode: 1200MHz - 1300MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1200,
                        .dwUpperCpuClock = 1300,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Freeze Mode: 1300MHz - 1400MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1300,
                        .dwUpperCpuClock = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Freeze Mode: 1400MHz - 1400MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1400,
                        .dwUpperCpuClock = 1401,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 95,
                        },
                    },
                },
            },
            .DvfsNormalModeInfo =
            {
                {
                    //Normal Mode: 1050MHz - 1200MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1050,
                        .dwUpperCpuClock = 1200,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 95,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1200MHz - 1300MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1200,
                        .dwUpperCpuClock = 1300,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1300MHz - 1400MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1300,
                        .dwUpperCpuClock = 1400,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1400MHz - 1400MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1400,
                        .dwUpperCpuClock = 1401,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Normal Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 95,
                        },
                    },
                },
            },
        },
        {
            //Old QFP Package
            .DvfsSysInfo =
            {
                .dwDefaultCpuClock = 852,
                .dwMinimumCpuClock = 852,
                .dwMaximumCpuClock = 1150,
                .dwProtectedCpuClock = 848,
                .dwDefaultCpuPower = 110,
                .dwDefaultCorePower = 100,
            },
            .DvfsTemperatureInfo =
            {
                .dwLowerFreezeTemperature = 70, //CONFIG_DVFS_TEMPERATURE_DISABLE
                .dwUpperFreezeTemperature = 85, //CONFIG_DVFS_TEMPERATURE_DISABLE
                .dwLowerLevelTemperature = 120,
                .dwUpperLevelTemperature = 135,
                .dwMaxLevelTemperature = 160,
            },
            .DvfsInitModeInfo =
            {
                //Initial Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 852, //QFP: 852MHz
                    .dwUpperCpuClock = 852, //QFP: 852MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500, //QFP: 500MHz
                    .dwUpperGpuClock = 500, //QFP: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 110,  //QFP: 1.10V
                        .dwCorePower = 100, //QFP: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 110,  //QFP: 1.10V
                        .dwCorePower = 100, //QFP: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 110,  //QFP: 1.10V
                        .dwCorePower = 100, //QFP: 1.00V
                    },
                },
            },
            .DvfsBootModeInfo =
            {
                //Boot Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 852,     //QFP: 852MHz
                    .dwUpperCpuClock = 1150,    //QFP: 1150MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //QFP: 500MHz
                    .dwUpperGpuClock = 500,     //QFP: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 120,      //QFP: 1.20V
                        .dwCorePower = 100,     //QFP: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 120,      //QFP: 1.20V
                        .dwCorePower = 100,     //QFP: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 110,      //QFP: 1.10V
                        .dwCorePower = 95,      //QFP: 0.95V
                    },
                },
            },
            .DvfsOverTemperatureModeInfo =
            {
                //Over Temperature Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 848, //QFP: 848MHz
                    .dwUpperCpuClock = 848, //QFP: 848MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500, //QFP: 500MHz
                    .dwUpperGpuClock = 500, //QFP: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 100,  //QFP: 1.00V
                        .dwCorePower = 95,  //QFP: 0.95V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 100,  //QFP: 1.00V
                        .dwCorePower = 95,  //QFP: 0.95V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 90,   //QFP: 0.90V
                        .dwCorePower = 95,  //QFP: 0.95V
                    },
                },
            },
            .DvfsFreezeModeInfo =
            {
                {
                    //Freeze Mode: 852MHz - 950MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 852,
                        .dwUpperCpuClock = 950,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                    },
                },
                {
                    //Freeze Mode: 950MHz - 1050MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 950,
                        .dwUpperCpuClock = 1050,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                    },
                },
                {
                    //Freeze Mode: 1050MHz - 1150MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1050,
                        .dwUpperCpuClock = 1150,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                    },
                },
                {
                    //Freeze Mode: 1150MHz - 1150MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1150,
                        .dwUpperCpuClock = 1151,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 125,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 125,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 125,
                            .dwCorePower = 100,
                        },
                    },
                },
            },
            .DvfsNormalModeInfo =
            {
                {
                    //Normal Mode: 852MHz - 950MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 852,
                        .dwUpperCpuClock = 950,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 95,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 950MHz - 1050MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 950,
                        .dwUpperCpuClock = 1050,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1050MHz - 1150MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1050,
                        .dwUpperCpuClock = 1150,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 115,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1150MHz - 1150MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1150,
                        .dwUpperCpuClock = 1151,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Normal Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 120,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 95,
                        },
                    },
                },
            },
        },
        {
            //New QFP Package
            .DvfsSysInfo =
            {
                .dwDefaultCpuClock = 1000,
                .dwMinimumCpuClock = 1000,
                .dwMaximumCpuClock = 1294,
                .dwProtectedCpuClock = 900,
                .dwDefaultCpuPower = 110,
                .dwDefaultCorePower = 100,
            },
            .DvfsTemperatureInfo =
            {
                .dwLowerFreezeTemperature = 90, //CONFIG_DVFS_TEMPERATURE_DISABLE
                .dwUpperFreezeTemperature = 100, //CONFIG_DVFS_TEMPERATURE_DISABLE
                .dwLowerLevelTemperature = 120,
                .dwUpperLevelTemperature = 135,
                .dwMaxLevelTemperature = 160,
            },
            .DvfsInitModeInfo =
            {
                //Initial Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 1000,    //New QFP: 1000MHz
                    .dwUpperCpuClock = 1000,    //New QFP: 1000MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //New QFP: 500MHz
                    .dwUpperGpuClock = 500,     //New QFP: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 110,      //New QFP: 1.10V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 110,      //New QFP: 1.10V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 110,      //New QFP: 1.10V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                },
            },
            .DvfsBootModeInfo =
            {
                //Boot Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 1294,    //New QFP: 1294MHz
                    .dwUpperCpuClock = 1294,    //New QFP: 1294MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //New QFP: 500MHz
                    .dwUpperGpuClock = 500,     //New QFP: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 127,      //New QFP: 1.27V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 127,      //New QFP: 1.27V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 127,      //New QFP: 1.27V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                },
            },
            .DvfsOverTemperatureModeInfo =
            {
                //Over Temperature Mode
                .DvfsCpuInfo =
                {
                    .dwLowerCpuClock = 900,     //New QFP: 900MHz
                    .dwUpperCpuClock = 900,     //New QFP: 900MHz
                },
                .DvfsGpuInfo =
                {
                    .dwLowerGpuClock = 500,     //New QFP: 500MHz
                    .dwUpperGpuClock = 500,     //New QFP: 500MHz
                },
                .DvfsPowerInfo =
                {
                    {
                        //SS Corner Chip
                        .dwCpuPower = 100,      //New QFP: 1.00V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                    {
                        //TT Corner Chip
                        .dwCpuPower = 100,      //New QFP: 1.00V
                        .dwCorePower = 100,     //New QFP: 1.00V
                    },
                    {
                        //FF Corner Chip
                        .dwCpuPower = 95,       //New QFP: 0.95V
                        .dwCorePower = 95,      //New QFP: 0.95V
                    },
                },
            },
            .DvfsFreezeModeInfo =
            {
                {
                    //Freeze Mode: 1000MHz - 1100MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1000,
                        .dwUpperCpuClock = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                    },
                },
                {
                    //Freeze Mode: 1100MHz - 12000MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1100,
                        .dwUpperCpuClock = 1200,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 117,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 117,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 117,
                            .dwCorePower = 100,
                        },
                    },
                },
                {
                    //Freeze Mode: 1200MHz - 1294MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1200,
                        .dwUpperCpuClock = 1294,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 122,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 122,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 122,
                            .dwCorePower = 100,
                        },
                    },
                },
                {
                    //Freeze Mode: 1294MHz - 1295MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1294,
                        .dwUpperCpuClock = 1295,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Freeze Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 127,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 127,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 127,
                            .dwCorePower = 100,
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
                        .dwLowerCpuClock = 1000,
                        .dwUpperCpuClock = 1100,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 100,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1100MHz - 1200MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1100,
                        .dwUpperCpuClock = 1200,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 105,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1200MHz - 1294MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1200,
                        .dwUpperCpuClock = 1294,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        {
                            //SS Corner Chip
                            .dwCpuPower = 117,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 117,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 95,
                        },
                    },
                },
                {
                    //Normal Mode: 1294MHz - 1294MHz
                    .DvfsCpuInfo =
                    {
                        .dwLowerCpuClock = 1294,
                        .dwUpperCpuClock = 1295,
                    },
                    .DvfsGpuInfo =
                    {
                        .dwLowerGpuClock = 500,
                        .dwUpperGpuClock = 500,
                    },
                    .DvfsPowerInfo =
                    {
                        //Normal Mode
                        {
                            //SS Corner Chip
                            .dwCpuPower = 122,
                            .dwCorePower = 100,
                        },
                        {
                            //TT Corner Chip
                            .dwCpuPower = 122,
                            .dwCorePower = 100,
                        },
                        {
                            //FF Corner Chip
                            .dwCpuPower = 110,
                            .dwCorePower = 95,
                        },
                    },
                },
            },
        },
    },
};

//=================================================================================================
U32 MHalDvfsProc(U32 dwCpuClock)
{
//  U32     dwRegisterValue = 0;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    //Check Specific Register to Check DVFS Running State (0x1005_00 = 0x3697)
    if(DvfsRegInfo->reg_vid_dvfs_id == CONFIG_DVFS_ENABLE_PATTERN)
    {
        if(hMstarDvfsInfo.bDvfsInitOk == 0)
        {
            //Initial DVFS Default Settings and Data Structure
            if(MHalDvfsInit() == TRUE)
            {
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_INIT_MODE;
            }
            else
            {
                hMstarDvfsInfo.dwFinalCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
                goto _MHalDvfsProcExit;
            }
        }

        //Get CPU Temperature by PM_SAR
        MHalDvfsCpuTemperature();

        if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
        {
            //Initial Mode

            //Use Default CPU Clock in Init Mode
            MHalDvfsCpuClockAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock);

            if((hMstarDvfsInfo.dwCpuTemperature >= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
               (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
            {
                //Change to Normal Mode
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
            }
            else if(hMstarDvfsInfo.dwCpuTemperature > DvfsRegInfo->reg_high_level_temp)
            {
                //Change to Over-Temperature Mode
               hMstarDvfsInfo.dwOverTemperatureFlag = 1;
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
            }
            else
            {
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_BOOT_MODE;  //CONFIG_DVFS_NORMAL_MODE;
            }

            hMstarDvfsInfo.dwFinalCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
        }
        else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
        {
            //Boot Mode

            if(hMstarDvfsInfo.dwBootTimeCounter > CONFIG_DVFS_BOOT_MODE_TIME)
            {
                //Change to Normal Mode
                hMstarDvfsInfo.dwBootTimeCounter = 0;

                //Use Default CPU Clock in Boot Mode
                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwLowerCpuClock);

                //Change to Normal Mode
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                hMstarDvfsInfo.dwFinalCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwLowerCpuClock;
            }
            else
            {
                //Use Maximum CPU Clock in Boot Mode
                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwUpperCpuClock);
                hMstarDvfsInfo.dwFinalCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsBootModeInfo.DvfsCpuInfo.dwUpperCpuClock;
            }
        }
        else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
        {
            //Over-Temperature Mode
            if(hMstarDvfsInfo.dwCpuTemperature > DvfsRegInfo->reg_max_level_temp)
            {
                //Retry 10 Times to Confirm the State of Over Temperature
                if(hMstarDvfsInfo.dwResetCounter < CONFIG_DVFS_RESET_MAX_COUNT)
                {
                    DVFS_HAL_DEBUG("\033[1;31m[DVFS] Over Temperature Protection: Count = %d / Temperature = %d\033[0m\n", (unsigned int) hMstarDvfsInfo.dwResetCounter, (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                    hMstarDvfsInfo.dwResetCounter ++;
                }
                else
                {
                    DVFS_HAL_INFO("\033[1;31m[DVFS] Current Temperature: %d\033[0m\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                    DVFS_HAL_INFO("\033[1;31m[DVFS] Over Temperature Mode: SYSTEM RESET\033[0m\n");

                    //Trigger a WDT Reset
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x00;
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003008 << 1)) = 0x00;
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x05;
                    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003000 << 1)) = 0x01;

                    while(1);
                }
            }
            else if(hMstarDvfsInfo.dwCpuTemperature < DvfsRegInfo->reg_low_level_temp)
            {
                //Return to Normal Mode
                hMstarDvfsInfo.dwOverTemperatureFlag = 0;
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);

                MHalDvfsCpuClockAdjustment(dwCpuClock);
                hMstarDvfsInfo.dwFinalCpuClock = dwCpuClock;

                hMstarDvfsInfo.dwResetCounter = 0;
            }
            else
            {
                //Keep at Over-Temperature Mode
                hMstarDvfsInfo.dwOverTemperatureFlag = 1;
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;

                DVFS_HAL_DEBUG("\033[1;31m[DVFS] Current Temperature: %d\033[0m\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                DVFS_HAL_DEBUG("\033[1;31m[DVFS] Over-Temperature Mode: CPU Clock: %dMHz\033[0m\n", hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);

                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);
                hMstarDvfsInfo.dwFinalCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;

                hMstarDvfsInfo.dwResetCounter = 0;
            }
        }
        else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_FREEZE_MODE)
        {
            //Freeze Mode
            if((hMstarDvfsInfo.dwCpuTemperature >= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
               (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
            {
                //Return to Normal Mode
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);
            }
            else
            {
                //Keep at Freeze Mode
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Freeze Mode: CPU Clock: %dMHz\n", dwCpuClock);
            }

            MHalDvfsCpuClockAdjustment(dwCpuClock);
            hMstarDvfsInfo.dwFinalCpuClock = dwCpuClock;
        }
        else
        {
            //Normal Mode
            if(hMstarDvfsInfo.dwCpuTemperature >= DvfsRegInfo->reg_high_level_temp)
            {
                //Change to Over-Temperature Mode
                hMstarDvfsInfo.dwOverTemperatureFlag = 1;
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Over-Temperature Mode: CPU Clock: %dMHz\n", hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);

                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);

                hMstarDvfsInfo.dwFinalCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
            }
            else if((hMstarDvfsInfo.dwCpuTemperature < hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature) && \
                    (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
            {
                //Change to Freeze Mode
                DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Freeze Mode: CPU Clock: %dMHz\n", dwCpuClock);

                MHalDvfsCpuClockAdjustment(dwCpuClock);
                hMstarDvfsInfo.dwFinalCpuClock = dwCpuClock;
            }
            else
            {
                if(DvfsRegInfo->reg_special_cpu_clk != CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT)
                {
                    U32 dwRegisterValue = 0;

                    dwRegisterValue = DvfsRegInfo->reg_special_cpu_clk;

                    if((dwRegisterValue >= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMinimumCpuClock) && \
                       (dwRegisterValue <= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock))
                    {
                        //Special Clock Mode
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_SPECIAL_CLOCK_MODE;

                        DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                        DVFS_HAL_DEBUG("[DVFS] Special Clock Mode: CPU Clock: %dMHz\n", dwRegisterValue);

                        MHalDvfsCpuClockAdjustment(dwRegisterValue);

                        DvfsRegInfo->reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                        hMstarDvfsInfo.dwFinalCpuClock = dwRegisterValue;
                    }
                    else
                    {
                        //Return to  Normal Mode
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                        DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);

                        MHalDvfsCpuClockAdjustment(dwCpuClock);

                        DvfsRegInfo->reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                        hMstarDvfsInfo.dwFinalCpuClock = dwCpuClock;
                    }
                }
                else
                {
                    //Keep at Normal Mode
                    DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;

                    DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                    DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);

                    MHalDvfsCpuClockAdjustment(dwCpuClock);

                    DvfsRegInfo->reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                    hMstarDvfsInfo.dwFinalCpuClock = dwCpuClock;
                }
            }
        }

        //MsOS_ReleaseMutex(_s32SAR_Dvfs_Mutex);
    }
    else
    {
        //Disable DVFS
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110cb0 << 1)) = 0;
    }

_MHalDvfsProcExit:

//  hMstarDvfsInfo.dwFinalCpuClock = dwCpuClock;// = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;
    DVFS_HAL_DEBUG("[DVFS] hMstarDvfsInfo.dwFinalCpuClock: %d\n", hMstarDvfsInfo.dwFinalCpuClock);

    return hMstarDvfsInfo.dwFinalCpuClock;
}

//=================================================================================================
U32 MHalDvfsInit(void)
{
    U32 bDvfsInitStatus = TRUE;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110cb2 << 1)) &= ~(0x01);

    hMstarDvfsInfo.bDvfsInitOk = 0;
    hMstarDvfsInfo.bDvfsModeChange = 0;
    hMstarDvfsInfo.dwMaxCpuClockByTemperature = 0;
    hMstarDvfsInfo.dwFinalCpuClock = 0;
    hMstarDvfsInfo.dwFinalCpuPowerVoltage = 0;
    hMstarDvfsInfo.dwFinalCorePowerVoltage = 0;
    hMstarDvfsInfo.dwCpuTemperature = 0;
    hMstarDvfsInfo.dwRefTemperature = 0;
    hMstarDvfsInfo.dwAvgCpuTempCounter = 0;
    hMstarDvfsInfo.dwAvgCpuTempBuffer = 0;
    hMstarDvfsInfo.bSystemResumeFlag = 0;
    hMstarDvfsInfo.dwResetCounter = 0;
    hMstarDvfsInfo.dwTemperatureCounter = 0;
    hMstarDvfsInfo.dwBootTimeCounter = 0;
    hMstarDvfsInfo.dwOverTemperatureFlag = 0;

    //Get Reference Level of 25-degree Temperature in eFuse
    MHalDvfsCpuTemperature();

    //Init Test Bus to Measure CPU Clock
    MHalDvfsCpuDisplay();

#if defined(CONFIG_MSTAR_IIC)
    if(SysDvfsPowerInit() == TRUE)
    {
        hMstarDvfsInfo.bDvfsInitOk = 1;
    }
    else
    {
        hMstarDvfsInfo.bDvfsInitOk = 0;
        bDvfsInitStatus = FALSE;
    }
#else
    hMstarDvfsInfo.bDvfsInitOk = 1;
#endif

    return bDvfsInitStatus;
}
//=================================================================================================
U32 MHalDvfsGetCpuFreq(U8 dwCpu)
{
    U32 dwRegisterValue = 0;
    mutex_lock(&MHalDvfsGetCpuFreqMutex);
    if(hMstarDvfsInfo.bDvfsInitOk == 0)
    {
        //Init Test Bus to Measure CPU Clock
        dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101896 << 1));
        dwRegisterValue &= ~(0x07);
        dwRegisterValue |= 0x01;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101896 << 1)) = dwRegisterValue;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eea << 1)) = 0;
        dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eea << 1));
        dwRegisterValue &= ~(0x07);
        dwRegisterValue |= 0x04;
        dwRegisterValue &= ~(0x01 << 4);
        dwRegisterValue &= ~(0x01 << 5);
        dwRegisterValue &= ~(0x01 << 6);
        dwRegisterValue |= (0x01 << 14);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eea << 1)) = dwRegisterValue;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eee << 1)) = 0x001F;
//      *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e62 << 1)) = 0;
//      *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e62 << 1)) = 1;
    }

    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e62 << 1)) = 0;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e62 << 1)) = 1;
    udelay(CONFIG_DVFS_CLOCK_DELAY_US);
    udelay(CONFIG_DVFS_CLOCK_DELAY_US);

    //CPU Clock = Register 0x101E_64[15:0] * 12 / 1000 * 4
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e64 << 1));
    dwRegisterValue *= 12;
    dwRegisterValue /= 1000;
    dwRegisterValue *= 4;
    mutex_unlock(&MHalDvfsGetCpuFreqMutex);
    return dwRegisterValue;
}

//=================================================================================================
void MHalDvfsCpuDisplay(void)
{
    U32  dwRegisterValue = 0;

    dwRegisterValue = MHalDvfsGetCpuFreq(0);
    DvfsRegInfo->reg_cur_cpu_clock = dwRegisterValue;

    DVFS_HAL_DEBUG("[DVFS] CPU Clock: %dMHz\n", (unsigned int) dwRegisterValue);
}

//=================================================================================================
void MHalDvfsCpuTemperature(void)
{
    S32     dwTempData = 0;

    if(hMstarDvfsInfo.dwRefTemperature == 0)
    {
        U32     dwRegisterValue = 0;

        //Read 25-degree Reference Level in eFuse
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x0144;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x2144;
        while((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) & (0x01 << 13)) != 0);

        dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002058 << 1));
        dwRegisterValue >>= 6;

        //If no data existed in eFuse, set the default reference level is 400
        hMstarDvfsInfo.dwRefTemperature = dwRegisterValue;
        if(dwRegisterValue == 0)
        {
            hMstarDvfsInfo.dwRefTemperature = 400;
        }
    }

    //Read CH8 of PM_SAR to Get CPU Temperature
    dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00148e << 1));
    if(dwTempData == 0)
    {
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) = 0x0A20;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x001400 << 1)) |= (0x01 << 14);
        dwTempData = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00148e << 1));
    }

    hMstarDvfsInfo.dwAvgCpuTempBuffer += dwTempData;
    hMstarDvfsInfo.dwAvgCpuTempCounter ++;
    if((hMstarDvfsInfo.dwAvgCpuTempCounter >= CONFIG_DVFS_DATA_COUNT) || \
       (DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE))
    {
//      U32    dwTempCounter = 0;
        S32    dwTempValue = 0;

        DVFS_HAL_DEBUG("[DVFS] [[====================================\n");

        MHalDvfsCpuDisplay();

        DVFS_HAL_DEBUG("[DVFS] Reference Temperature Data (CPU): 0x%04x\n", (unsigned int) hMstarDvfsInfo.dwRefTemperature);
//      for(dwTempCounter = 0; dwTempCounter < CONFIG_DVFS_DATA_COUNT; dwTempCounter ++)
//      {
//          //DVFS_HAL_DEBUG(" - %d: 0x%04x\n", dwTempCounter, dwAvgCpuTempBuffer[dwTempCounter]);
//          dwTempValue += hMstarDvfsInfo.dwAvgCpuTempBuffer[dwTempCounter];
//      }
//      dwTempValue /= CONFIG_DVFS_DATA_COUNT;

        if(hMstarDvfsInfo.dwAvgCpuTempBuffer != 0)
        {
            dwTempValue = hMstarDvfsInfo.dwAvgCpuTempBuffer / hMstarDvfsInfo.dwAvgCpuTempCounter;

            DVFS_HAL_DEBUG("[DVFS] Current Temperature Sensor Data (CPU): 0x%04x\n", (unsigned int) dwTempValue);

            if(hMstarDvfsInfo.dwRefTemperature >= dwTempValue)
            {
                dwTempData = (((hMstarDvfsInfo.dwRefTemperature - dwTempValue) * 1280) + CONFIG_DVFS_T_SENSOR_SHIFT);
            }
            else
            {
                dwTempData = ((dwTempValue - hMstarDvfsInfo.dwRefTemperature) * 1280);
                dwTempData = (CONFIG_DVFS_T_SENSOR_SHIFT - dwTempData);
            }

            hMstarDvfsInfo.dwCpuTemperature = (dwTempData / 1000);
            DvfsRegInfo->reg_cur_cpu_temp = hMstarDvfsInfo.dwCpuTemperature;
        }

        DVFS_HAL_DEBUG("[DVFS] Temperature (CPU): %d\n", hMstarDvfsInfo.dwCpuTemperature);

        hMstarDvfsInfo.dwAvgCpuTempBuffer = 0;
        hMstarDvfsInfo.dwAvgCpuTempCounter = 0;

        DVFS_HAL_DEBUG("[DVFS] CPU Power: %d0mV\n", hMstarDvfsInfo.dwFinalCpuPowerVoltage);
        DVFS_HAL_DEBUG("[DVFS] Core Power: %d0mV\n", hMstarDvfsInfo.dwFinalCorePowerVoltage);

        DVFS_HAL_DEBUG("[DVFS] ====================================]]\n");
    }
}

//=================================================================================================
void MHalDvfsCpuClockAdjustment(U32 dwCpuClock)
{
    U32     dwRegisterValue = 0;
    U32     dwTempCpuClock = 0;

    dwTempCpuClock = (dwCpuClock - (dwCpuClock % 4));

    if((DvfsRegInfo->reg_cur_cpu_clock != dwTempCpuClock) || (hMstarDvfsInfo.bDvfsModeChange == 1))
    {
        if(DvfsRegInfo->reg_cur_cpu_clock < dwCpuClock)
        {
            MHalDvfsPowerControl(dwCpuClock);
        }

        //Adjust User Defined CPU Clock
        dwRegisterValue = ((3623878UL / dwCpuClock) * 1000);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110ca4 << 1)) = (dwRegisterValue & 0xFFFF);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110ca6 << 1)) = ((dwRegisterValue >> 16) & 0xFFFF);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110cb0 << 1)) = 0x01;   //switch to LPF control
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110caa << 1)) = 0x06;   //mu[2:0]
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110cae << 1)) = 0x08;   //lpf_update_cnt[7:0]

        //Set LPF is Low to High
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110cb2 << 1)) |= (0x01 << 12);
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110ca8 << 1)) = 0x00;
        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x110ca8 << 1)) = 0x01;
        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        if(DvfsRegInfo->reg_cur_cpu_clock >= dwCpuClock)
        {
            MHalDvfsPowerControl(dwCpuClock);
        }

        DvfsRegInfo->reg_cur_cpu_clock = dwCpuClock;
        hMstarDvfsInfo.bDvfsModeChange = 0;
    }
}

//=================================================================================================
U32 MHalDvfsSearchCpuClockLevel(U32 dwCpuClock)
{
    U32     dwLoopCounter = 0;
    U32     dwCpuLevel = CONFIG_DVFS_CPU_CLOCK_DISABLE;

    //Confirm Corresponding Level by User Defined CPU Clock
    for(dwLoopCounter = 0; dwLoopCounter < CONFIG_DVFS_POWER_CTL_SEGMENT; dwLoopCounter++)
    {
        if((dwCpuClock >= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsNormalModeInfo[dwLoopCounter].DvfsCpuInfo.dwLowerCpuClock) && \
           (dwCpuClock < hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsNormalModeInfo[dwLoopCounter].DvfsCpuInfo.dwUpperCpuClock))
        {
            dwCpuLevel = dwLoopCounter;
            break;
        }
    }

    return dwCpuLevel;
}

//=================================================================================================
void MHalDvfsPowerControl(U32 dwCpuClock)
{
    U32     dwClockLevel = 0;

    dwClockLevel = MHalDvfsSearchCpuClockLevel(dwCpuClock);
    if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
    {
        //Init Mode
        //Adjust CPU Power
        MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsPowerInfo[DvfsRegInfo->reg_cpu_pwr_type].dwCpuPower);

        //Adjust Core Power
        MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsPowerInfo[DvfsRegInfo->reg_core_pwr_type].dwCorePower);
    }
    else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
    {
        //Boot Mode
        if((hMstarDvfsInfo.dwCpuTemperature < hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
           (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsFreezeModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->reg_cpu_pwr_type].dwCpuPower);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsFreezeModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->reg_core_pwr_type].dwCorePower);
        }
        else
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsBootModeInfo.DvfsPowerInfo[DvfsRegInfo->reg_cpu_pwr_type].dwCpuPower);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsBootModeInfo.DvfsPowerInfo[DvfsRegInfo->reg_core_pwr_type].dwCorePower);
        }
    }
    else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_FREEZE_MODE)
    {
        //Freeze Mode
        if(dwClockLevel == CONFIG_DVFS_CPU_CLOCK_DISABLE)
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCpuPower);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCorePower);
        }
        else
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsFreezeModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->reg_cpu_pwr_type].dwCpuPower);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsFreezeModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->reg_core_pwr_type].dwCorePower);
        }
    }
    else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
    {
        //Over-Temperature Mode
        //Adjust CPU Power
        MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[DvfsRegInfo->reg_cpu_pwr_type].dwCpuPower);

        //Adjust Core Power
        MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[DvfsRegInfo->reg_core_pwr_type].dwCorePower);
    }
    else
    {
        //Normal Mode and Special Clock Mode
        if(dwClockLevel == CONFIG_DVFS_CPU_CLOCK_DISABLE)
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCpuPower);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCorePower);
        }
        else
        {
            //Adjust CPU Power
            MHalDvfsCpuPowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->reg_cpu_pwr_type].dwCpuPower);

            //Adjust Core Power
            MHalDvfsCorePowerAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsPowerInfo[DvfsRegInfo->reg_core_pwr_type].dwCorePower);
        }
    }
}

//=================================================================================================
void MHalDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage)
{
    if(hMstarDvfsInfo.bSystemResumeFlag != 0)
    {
        DVFS_HAL_DEBUG("\033[1;31m[DVFS] Re-init Power Control Flow\033[0m\n");

#if defined(CONFIG_MSTAR_IIC)
        SysDvfsCpuPowerInit();
#else
        MDrvDvfsVoltageSetup(0, 0, 0xFE);
#endif
        hMstarDvfsInfo.bSystemResumeFlag = 0;
    }

#if defined(CONFIG_MSTAR_IIC)
    SysDvfsCpuPowerAdjustment(dwCpuPowerVoltage);
#else
    MDrvDvfsVoltageSetup(0, dwCpuPowerVoltage, 0);
#endif

    DVFS_HAL_DEBUG("[DVFS] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
    hMstarDvfsInfo.dwFinalCpuPowerVoltage = dwCpuPowerVoltage;
}

//=================================================================================================
void MHalDvfsCorePowerAdjustment(U32 dwCorePowerVoltage)
{
#if defined(CONFIG_MSTAR_IIC)
    SysDvfsCorePowerAdjustment(dwCorePowerVoltage);
#else
    MDrvDvfsVoltageSetup(0, dwCorePowerVoltage, 1);
#endif
    DVFS_HAL_DEBUG("[DVFS] Core Power: %d\n", (unsigned int) dwCorePowerVoltage);
    hMstarDvfsInfo.dwFinalCorePowerVoltage = dwCorePowerVoltage;
}

//=================================================================================================
U32  MHalDvfsQueryCpuClock(U32 dwCpuClockType)
{
    U32     dwOutputCpuClock = 0;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    if(dwCpuClockType == CONFIG_DVFS_MAX_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
    }
    else if(dwCpuClockType == CONFIG_DVFS_MIN_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMinimumCpuClock;
    }
    else if(dwCpuClockType == CONFIG_DVFS_OVER_TEMPERATURE_PROTECT_CPU_CLOCK)
    {
        dwOutputCpuClock = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
    }
    else
    {
        dwOutputCpuClock = CONFIG_DVFS_CPU_CLOCK_DISABLE;
    }

    return dwOutputCpuClock;
}

//=================================================================================================
U32 MHalDvfsQueryCpuClockByTemperature(void)
{
//  S32     dwCpuTemperature = 0;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    if(DvfsRegInfo->reg_vid_dvfs_id == CONFIG_DVFS_ENABLE_PATTERN)
    {
        if(hMstarDvfsInfo.bDvfsModeChange == 1)
        {
            goto _MHalDvfsQueryCpuClockByTemperatureExit;
        }

        if(hMstarDvfsInfo.dwMaxCpuClockByTemperature == 0)
        {
            hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;
        }

        if(hMstarDvfsInfo.bDvfsInitOk == 0)
        {
            if(MHalDvfsInit() == TRUE)
            {
                hMstarDvfsInfo.dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                hMstarDvfsInfo.bDvfsModeChange = 1;
            }
            else
            {
                hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;
            }
        }
        else
        {
            MHalDvfsCpuTemperature();   //Get current CPU temperature

            if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
            {
                hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
            }
            else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
            {
                if(hMstarDvfsInfo.dwCpuTemperature > DvfsRegInfo->reg_max_level_temp)
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("\033[1;31m[DVFS] Over Temperature Protection: %d\033[0m\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Maximum Level Threshold Temperature in Over-Temperature Mode
                        DVFS_HAL_INFO("\033[1;31m[DVFS] Current Temperature: %d\033[0m\n", (unsigned int) hMstarDvfsInfo.dwCpuTemperature);
                        DVFS_HAL_INFO("\033[1;31m[DVFS] Over Temperature Mode: SYSTEM RESET\033[0m\n");

                        //Trigger a WDT Reset
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x00;
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003008 << 1)) = 0x00;
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x00300a << 1)) = 0x05;
                        *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x003000 << 1)) = 0x01;

                        while(1);
                    }
                }
                else if(hMstarDvfsInfo.dwCpuTemperature < DvfsRegInfo->reg_low_level_temp)
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Over-Temperature->Normal Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Return to Normal Mode
                        hMstarDvfsInfo.dwOverTemperatureFlag = 0;
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Over-Temperature Mode
                    hMstarDvfsInfo.dwOverTemperatureFlag = 1;
                    hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                }
            }
            else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
            {
                if(hMstarDvfsInfo.dwCpuTemperature >= DvfsRegInfo->reg_high_level_temp)
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Boot->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Over-Temperature Mode
                        hMstarDvfsInfo.dwOverTemperatureFlag = 1;
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
                        //hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.bDvfsModeChange = 1;
                        hMstarDvfsInfo.dwTemperatureCounter = 0;
                    }

                    hMstarDvfsInfo.dwBootTimeCounter = 0;
                }
                else if((hMstarDvfsInfo.dwCpuTemperature >= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
                   (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Boot->Normal Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Return to Normal Mode
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.bDvfsModeChange = 1;
                        hMstarDvfsInfo.dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Boot Mode
                    hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;

                    hMstarDvfsInfo.dwBootTimeCounter ++;
                    if(hMstarDvfsInfo.dwBootTimeCounter > CONFIG_DVFS_BOOT_MODE_TIME)
                    {
                        //Return to Normal Mode
                        if(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature == CONFIG_DVFS_TEMPERATURE_DISABLE)
                        {
                            DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        }
                        else
                        {
                            DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;
                        }
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Boot Mode->Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.bDvfsModeChange = 1;
                    }
                    else
                    {
                        DVFS_HAL_DEBUG("[DVFS] Boot Mode Counter: %d\n", hMstarDvfsInfo.dwBootTimeCounter);
                    }
                }
            }
            else if(DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_FREEZE_MODE)
            {
                if((hMstarDvfsInfo.dwCpuTemperature >= hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature) && \
                   (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Freeze->Normal Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Return to Normal Mode
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.bDvfsModeChange = 1;
                        hMstarDvfsInfo.dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                }
            }
            else if((DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_NORMAL_MODE) || (DvfsRegInfo->reg_cur_dvfs_state == CONFIG_DVFS_SPECIAL_CLOCK_MODE))
            {
                if(hMstarDvfsInfo.dwCpuTemperature >= DvfsRegInfo->reg_high_level_temp)
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Normal->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Normal Mode
                        hMstarDvfsInfo.dwOverTemperatureFlag = 1;
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_OVER_TEMPERATURE_MODE;
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.dwTemperatureCounter = 0;
                    }
                }
                else if((hMstarDvfsInfo.dwCpuTemperature < hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature) && \
                        (hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
                {
                    if(hMstarDvfsInfo.dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Normal->Freeze Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo.dwTemperatureCounter);
                        hMstarDvfsInfo.dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Freeze Threshold Temperature in Normal Mode
                        DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_FREEZE_MODE;
                        hMstarDvfsInfo.dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Freeze Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo.bDvfsModeChange = 1;
                        hMstarDvfsInfo.dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Normal Mode
                    hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                }
            }
            else
            {
                hMstarDvfsInfo.dwMaxCpuClockByTemperature = hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
            }
        }
    }

_MHalDvfsQueryCpuClockByTemperatureExit:

    DVFS_HAL_DEBUG("[DVFS] Current DVFS State: %d\n", (unsigned int) DvfsRegInfo->reg_cur_dvfs_state);
    DVFS_HAL_DEBUG("[DVFS] Current Valid CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo.dwMaxCpuClockByTemperature);

    return hMstarDvfsInfo.dwMaxCpuClockByTemperature;
}

//=================================================================================================
void MHalDvfsCpuDisplayInit(void)
{
    U32     dwRegisterValue = 0;

    //Init Test Bus to Measure CPU Clock
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101896 << 1));
    dwRegisterValue &= ~(0x07);
    dwRegisterValue |= 0x01;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101896 << 1)) = dwRegisterValue;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eea << 1)) = 0;
    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eea << 1));
    dwRegisterValue &= ~(0x07);
    dwRegisterValue |= 0x04;
    dwRegisterValue &= ~(0x01 << 4);
    dwRegisterValue &= ~(0x01 << 5);
    dwRegisterValue &= ~(0x01 << 6);
    dwRegisterValue |= (0x01 << 14);
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eea << 1)) = dwRegisterValue;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101eee << 1)) = 0x001F;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e62 << 1)) = 0;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x101e62 << 1)) = 1;
}
//=================================================================================================
U32 MHalDvfsGetOverTemperatureFlag(U8 dwCluster)
{
   return hMstarDvfsInfo.dwOverTemperatureFlag;
}
//=================================================================================================
void MHalDvfsRefTemperature(void)
{
    U32     dwRegisterValue = 0;

    //Read 25-degree Reference Level in eFuse
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x0144;
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) = 0x2144;
    while((*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002050 << 1)) & (0x01 << 13)) != 0);

    dwRegisterValue = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x002058 << 1));
    dwRegisterValue >>= 6;

    //If no data existed in eFuse, set the default reference level is 400
    hMstarDvfsInfo.dwRefTemperature = dwRegisterValue;
    if(dwRegisterValue == 0)
    {
        hMstarDvfsInfo.dwRefTemperature = 400;
    }
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

void MHalDvfsCpuPowerInit(void)
{
    //TBD
}

//=================================================================================================
void MHalDvfsCorePowerInit(void)
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
    DVFS_HAL_DEBUG("[DVFS] Enter Suspend Mode\n");

//  MHalDvfsCpuClockAdjustment(hMstarDvfsInfo.DvfsModeInfo[DvfsRegInfo->reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock);

    DvfsRegInfo->reg_vid_dvfs_id = 0;

    return 0;
}

static int mstar_dvfs_drv_resume(struct device *dev)
{
    DVFS_HAL_DEBUG("[DVFS] Enter Resume Mode\n");

    MHalDvfsInit();
    hMstarDvfsInfo.bSystemResumeFlag = 1;

    DvfsRegInfo->reg_cur_dvfs_state = CONFIG_DVFS_INIT_MODE;

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

static int __init mstar_dvfs_init(void)
{
    platform_device_register(&mstar_dvfs_dev);
    platform_driver_register(&mstar_dvfs_driver);
    return 0;
}

static void __init mstar_dvfs_exit(void)
{
    platform_device_unregister(&mstar_dvfs_dev);
    platform_driver_unregister(&mstar_dvfs_driver);
}

core_initcall(mstar_dvfs_init);
module_exit(mstar_dvfs_exit);

//=================================================================================================
