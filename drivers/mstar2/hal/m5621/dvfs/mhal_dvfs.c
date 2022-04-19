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
#include <linux/module.h>

#include <linux/thermal.h>
#include <linux/cpu_cooling.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
#if defined(CONFIG_ENERGY_MODEL)
#include <linux/energy_model.h>
#endif
#endif

#include <linux/pm_opp.h>
#include <linux/cpu.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#include <mstar/mstar_chip.h>

#include "mdrv_dvfs.h"
#include "mdrv_CPU_cluster_calibrating.h"

#ifndef __MHAL_DEVS_TABLE_H__
#include "mhal_dvfs_table.h"
#endif

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

extern struct mstar_cpufreq_policy ondemand_timer[CONFIG_NR_CPUS];

DEFINE_MUTEX(MDrvDvfsCpuTempMutex);
DEFINE_MUTEX(MHalDvfsGetCpuFreqMutex);
DEFINE_MUTEX(MHalDvfsWriteMutex);
DEFINE_MUTEX(MHalDvfsInitMutex);

static struct file *gFile = NULL;
static loff_t gPos;
static char gDVFS_BUFFER[100];
static mm_segment_t gFS;
static int gBeClosed = 1;

extern unsigned int get_cpu_midr(int cpu);
#define CONFIG_DVFS_CPU_CLOCK_DISPLAY_ENABLE 1
extern void MDrvDvfsVoltageSetup(unsigned int dwCpuClock, unsigned int dwVoltage, unsigned int dwVoltageType, unsigned int dwCpu);
int halTotalClusterNumber;

static U32 gDVFS_TEMP_OFFSET = 0;
static U32 gDVFS_auto_measurement=0;
static U16 u16CpuClockSet[DVFS_CLUSTER_NUM] = {0};
static volatile MSTAR_DVFS_REG_INFO *DvfsRegInfo = 0;
#define DVFS_HAL_TOTAL_CLUSTER_NUM  (sizeof(hMstarDvfsInfo)/sizeof(MSTAR_DVFS_INFO))


int MHalDvfsSetStatus(unsigned int status, unsigned int cpu)
{
    int dwCluster = getCpuCluster(cpu);
    int temp =  0;
    int switch_on = 0;
    int offset = 0;
    MHalDvfsSetOverTempDebugOffset(0,0);
    temp = MHalDvfsGetCpuTemperature(cpu);
    switch (status)
    {
        case CONFIG_DVFS_FREEZE_MODE: // freeze mode;
            if (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature == CONFIG_DVFS_TEMPERATURE_DISABLE)
                printk("not support status %d\n",status);
            else
            {
                switch_on = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature;
                while( MHalDvfsGetStatus(dwCluster) != status )
                {
                    MHalDvfsSetOverTempDebugOffset(dwCluster,switch_on-temp-5);
                    msleep(100);
                }
            }
            break;
        case CONFIG_DVFS_OVER_TEMPERATURE_MODE: // over temp;
            while ( MHalDvfsGetStatus(cpu) != status)
            {
                switch_on = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature;
                MHalDvfsSetOverTempDebugOffset(dwCluster,switch_on -temp + 5);
                msleep(100);
            }
            break;
        case CONFIG_DVFS_NORMAL_MODE: // normal;
            while ( MHalDvfsGetStatus(cpu) != status)
            {
                if (MHalDvfsGetStatus(cpu) == CONFIG_DVFS_FREEZE_MODE)
                {
                    switch_on = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperFreezeTemperature;
                    MHalDvfsSetOverTempDebugOffset(cpu,switch_on - temp + 10);
                }
                else if (MHalDvfsGetStatus(cpu) == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
                {
                    switch_on = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerLevelTemperature;
                    MHalDvfsSetOverTempDebugOffset(cpu, switch_on - temp - 10 );
                    msleep(1000);
                }
            }
            break;
    }
    return 0;
}


int MHalDvfsVerifyVoltage(unsigned int cpu, unsigned int freq)
{
    int dwCluster = getCpuCluster(cpu);
    int level = MHalDvfsSearchCpuClockLevel(freq,cpu);
    int cur_vol = MHalDvfsGetCpuVoltage(cpu);
    int set_vol = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[level].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower;
    printk("set_vol = %d, cur_vol = %d\n", set_vol*10 , cur_vol);
    return (abs(cur_vol-set_vol*10)<=20)?0:1;
}


int MHalDvfsGetStatus(unsigned int cpu)
{
    int dwCluster = getCpuCluster(cpu);
    return DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state;
}

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

    DVFS_HAL_INFO("%s %d: ERROR cluster:%d is illegal , froce to be cluster0\n",__func__,__LINE__, cluster);
    return 0;
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
//================================================================================================
void getCpuBoundInfo(unsigned int dwCluster,unsigned *min, unsigned *max)
{
    if ( gDVFS_auto_measurement == 1 )
    {
        *min = DVFS_LOW_BOUND_VOLTAGE;
        *max = DVFS_HIGH_BOUND_VOLTAGE;
    }
    else
    {
        *max = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuPower;
        *min = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMinimumCpuPower;
    }

    return 1;
}

//================================================================================================
U32 MHalDvfsRegisterRead(U32 dwRegister )
{
    return( *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + ( dwRegister << 1)));
}
//=================================================================================================
void MHalDvfsRegisterWrite(U32 dwRegister , U32 u32Set )
{
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (dwRegister << 1)) = u32Set;

}
//=================================================================================================
void MHalDvfsRegisterWrite_SetBit(U32 dwRegister , int iSetBit )
{
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (dwRegister << 1)) |= (0x1<<iSetBit);
}
//=================================================================================================
void MHalDvfsRegisterWrite_ClearBit(U32 dwRegister , int iClearBit )
{
    *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (dwRegister << 1)) &= ~(0x1<<iClearBit);
}
//=================================================================================================
void MHalDvfsChangeMode(U8 u8Mode)
{
    U8 dwCluster = 0;
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    for(dwCluster = 0; dwCluster < DVFS_HAL_TOTAL_CLUSTER_NUM ; dwCluster++)
    {
        if (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == u8Mode)
            break;
        else
        {
            hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 0;

            DVFS_HAL_DEBUG("[DVFS]Cluser %d, Change Mode from %d to %d\n", dwCluster, DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state,u8Mode);
            if (hMstarDvfsInfo[dwCluster].bDvfsInitOk == 0)
            {
                MHalDvfsInit(dwCluster);
            }
            else if (u8Mode == CONFIG_DVFS_FREEZE_MODE )
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
            }
            else if (u8Mode == CONFIG_DVFS_NORMAL_MODE)
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
            }
            else if (u8Mode == CONFIG_DVFS_BOOT_MODE)
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
            }
            else if (u8Mode == CONFIG_DVFS_OVER_TEMPERATURE_MODE)
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
            }
            DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = u8Mode;
            hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
        }
    }

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
                MHalDvfsChangeMode(CONFIG_DVFS_INIT_MODE);
            }
            else
            {
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
                goto _MHalDvfsProcExit;
            }
        }

        //Get CPU Temperature by PM_SAR
        MHalDvfsCpuTemperature(dwCluster);

        if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
        {
            //Initial Mode
            //Use Default CPU Clock in Init Mode
            MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock, dwCpu);
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
            {
                //Change to Over-Temperature Mode
                MHalDvfsChangeMode(CONFIG_DVFS_OVER_TEMPERATURE_MODE);
            }
            else
            {
                MHalDvfsChangeMode(CONFIG_DVFS_BOOT_MODE);
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
                MHalDvfsChangeMode(CONFIG_DVFS_NORMAL_MODE);

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
            else
            {
                //Keep at Over-Temperature Mode
                MHalDvfsChangeMode(CONFIG_DVFS_OVER_TEMPERATURE_MODE);

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
                MHalDvfsChangeMode(CONFIG_DVFS_NORMAL_MODE);

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", dwCpuClock);
            }
            else
            {
                //Keep at Freeze Mode
                MHalDvfsChangeMode(CONFIG_DVFS_FREEZE_MODE);

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Freeze Mode: CPU Clock: %dMHz\n", dwCpuClock);
            }

            //dwCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;

            MHalDvfsCpuClockAdjustment(dwCpuClock, dwCpu);
            hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwCpuClock;
        }
        else
        {
            //Normal Mode
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
            {
                //Change to Over-Temperature Mode
                MHalDvfsChangeMode(CONFIG_DVFS_OVER_TEMPERATURE_MODE);

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Over-Temperature Mode: CPU Clock: %dMHz\n", hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock);

                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock, dwCpu);

                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
            }
            else if((hMstarDvfsInfo[dwCluster].dwCpuTemperature < hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature) && \
                    (hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerFreezeTemperature != CONFIG_DVFS_TEMPERATURE_DISABLE))
            {
                //Change to Freeze Mode
                MHalDvfsChangeMode(CONFIG_DVFS_FREEZE_MODE);

                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Freeze Mode: CPU Clock: %dMHz\n", dwCpuClock);

                MHalDvfsCpuClockAdjustment(dwCpuClock, dwCpu);
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = dwCpuClock;
            }
            else
            {
                U32 dwClockLevel = 0;
                //Keep at Normal Mode
                MHalDvfsChangeMode(CONFIG_DVFS_NORMAL_MODE);

                dwClockLevel = MHalDvfsSearchCpuClockLevel(dwCpuClock, dwCpu);
                DVFS_HAL_DEBUG("[DVFS] dwClockLevel: %d dwCpuClock: %d \n", dwClockLevel, dwCpuClock);
                DVFS_HAL_DEBUG("[DVFS] Current Temperature: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwCpuTemperature);
                DVFS_HAL_DEBUG("[DVFS] Normal Mode: CPU Clock: %dMHz\n", hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock);

                MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock, dwCpu);

                DvfsRegInfo->dvfs_reg[dwCluster].reg_special_cpu_clk = CONFIG_DVFS_DYNAMIC_CLOCK_ADJUST_INIT;
                hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwClockLevel].DvfsCpuInfo.dwLowerCpuClock;
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
    if (hMstarDvfsInfo[dwCluster].dwFinalCpuClock == 0)
    {
        DVFS_HAL_DEBUG("[DVFS] Current DVFS State: %d\n", (unsigned int) DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state);
        DVFS_HAL_DEBUG("[DVFS] hMstarDvfsInfo[%d].dwFinalCpuClock: %d\n", dwCluster, hMstarDvfsInfo[dwCluster].dwFinalCpuClock);
    }
    else
    {
        hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock;
    }


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

    mutex_lock(&MHalDvfsInitMutex);

    if (hMstarDvfsInfo[dwCluster].bDvfsInitOk == 1)
    {
        mutex_unlock(&MHalDvfsInitMutex);
        return bDvfsInitStatus;
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
    hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 0;

    //Get Reference Level of 25-degree Temperature in eFuse
    MHalDvfsCpuTemperature(dwCluster);

    //Init Test Bus to Measure CPU Clock
    MHalDvfsCpuDisplay(dwCluster);

    u16CpuClockSet[dwCluster] = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;

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

    mutex_unlock(&MHalDvfsInitMutex);
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

    // Add for query cpu temperature
    MHalDvfsRegisterWrite(0x001432, MHalDvfsRegisterRead(0x001432) & (~0x3 << 6) );
    MHalDvfsRegisterWrite_SetBit(0x000EC8,DVFS_BIT10);
    MHalDvfsRegisterWrite(0x000E5E,0x0004);
    MHalDvfsRegisterWrite(0x001420,0x0000);
}

//=================================================================================================

U32 MHalDvfsGetCpuFreq(U8 dwCpu)
{
    U32 dwRegisterValue = 0;
    int dwCluster = getCpuCluster(dwCpu);
    mutex_lock(&MHalDvfsGetCpuFreqMutex);
    MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0x90),DVFS_BIT14);
    MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0x90),DVFS_BIT14);
    usleep_range(1000,1000);

    // Wait calculate done , reg_calc_done
    while( (MHalDvfsRegisterRead((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0x90)) & 0xc000 ) != 0xc000 );

    dwRegisterValue = MHalDvfsRegisterRead((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0x92));
    dwRegisterValue *= 16;
    dwRegisterValue *= 1000;
    dwRegisterValue /= 83333;
    mutex_unlock(&MHalDvfsGetCpuFreqMutex);
    return dwRegisterValue;
}
//=================================================================================================
int MHalDvfsCpuDisplay(U8 dwCluster)
{
    U32 dwRegisterValue = 0;
    U32 u32RetryCount = 0;
#ifdef CONFIG_DVFS_CPU_CLOCK_DISPLAY_ENABLE
    if (dwCluster == 0)
        dwRegisterValue = MHalDvfsGetCpuFreq(0);

    if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state != CONFIG_DVFS_INIT_MODE)
        && (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state != CONFIG_DVFS_BOOT_MODE))
    {
        while(!((dwRegisterValue <= u16CpuClockSet[dwCluster] + CONFIG_DVFS_CPU_CLOCK_ACCURACY_BOUNDARY)
            && (dwRegisterValue >= u16CpuClockSet[dwCluster] - CONFIG_DVFS_CPU_CLOCK_ACCURACY_BOUNDARY)))
        {
            if (dwCluster == 0)
                dwRegisterValue = MHalDvfsGetCpuFreq(0);

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

    MHalDvfsRegisterWrite(0x001400,0x0A20);
    MHalDvfsRegisterWrite_SetBit(0x001400,DVFS_BIT14);
    dwTempData = MHalDvfsRegisterRead(0x00148e);

    if(dwTempData == 0)
    {
        MHalDvfsRegisterWrite(0x001400,0x0A20);
        MHalDvfsRegisterWrite_SetBit(0x001400,DVFS_BIT14);
        dwTempData = MHalDvfsRegisterRead(0x00148e);
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

    if (!IS_ERR(gFile))
    {
        mutex_lock(&MHalDvfsWriteMutex);
        if (!gBeClosed)
        {
            snprintf(gDVFS_BUFFER,sizeof(gDVFS_BUFFER),"|cpu = %d|state = %d%c",dwCpu,DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state, '\0');
            vfs_write(gFile, gDVFS_BUFFER, strlen(gDVFS_BUFFER), &gPos);
        }
        mutex_unlock(&MHalDvfsWriteMutex);
    }

    dwTempCpuClock = (dwCpuClock - (dwCpuClock % 4));
    if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock != dwTempCpuClock) || (hMstarDvfsInfo[dwCluster].bDvfsModeChange == 1))
    {
        if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock < dwCpuClock) && (gDVFS_auto_measurement == 0))
        {
            MHalDvfsPowerControl(dwCpuClock, dwCpu);
        }

        usleep_range(1000,1000);

        if (!IS_ERR(gFile))
        {
            mutex_lock(&MHalDvfsWriteMutex);
            if (!gBeClosed)
            {
                snprintf(gDVFS_BUFFER,sizeof(gDVFS_BUFFER),"|clock = %d%c",dwCpuClocki, '\0');
                vfs_write(gFile, gDVFS_BUFFER, strlen(gDVFS_BUFFER), &gPos);
            }
            mutex_unlock(&MHalDvfsWriteMutex);
        }

        if (dwCpuClock < 700)
        {
            if (bLowClkFg[dwCluster] == 0)
            {
                MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfc),DVFS_BIT0);
                MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfa),DVFS_BIT0);
                MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8),DVFS_BIT0);
                MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0x2a),DVFS_BIT1);
                MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8),DVFS_BIT0);

                bLowClkFg[dwCluster] = 1;
            }

            //Adjust User Defined CPU Clock
            dwRegisterValue = ((3623878UL / (dwCpuClock << 1)) * 1000);
        }
        else
        {
            if (bLowClkFg[dwCluster] == 1)
            {
                MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfc),DVFS_BIT0);
                MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xfa),DVFS_BIT0);
                MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8),DVFS_BIT0);
                MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0x2a),DVFS_BIT1);
                MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8),DVFS_BIT0);

                bLowClkFg[dwCluster] = 0;
            }

            //Adjust User Defined CPU Clock
            dwRegisterValue = ((3623878UL / dwCpuClock) * 1000);
        }

        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa4),(dwRegisterValue & 0xFFFF));
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa6),((dwRegisterValue >> 16) & 0xFFFF));
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb0),0x01); //switch to LPF control
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xaa),0x06); //mu[2:0]
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xae),0x08); //lpf_update_cnt[7:0]

        //Set LPF is Low to High
        MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb2),DVFS_BIT12);
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa8),0x00);
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa8),0x01);

        usleep_range(1000,1000);

        if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock >= dwCpuClock) && (gDVFS_auto_measurement == 0))
        {
            MHalDvfsPowerControl(dwCpuClock, dwCpu);
        }

        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock = dwCpuClock;
        u16CpuClockSet[dwCluster] = dwCpuClock;

        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 0;
    }
    if (!IS_ERR(gFile))
    {
        mutex_lock(&MHalDvfsWriteMutex);
        if (!gBeClosed)
        {
            snprintf(gDVFS_BUFFER,sizeof(gDVFS_BUFFER),"\n%c", '\0');
            vfs_write(gFile, gDVFS_BUFFER, strlen(gDVFS_BUFFER), &gPos);
        }
        mutex_unlock(&MHalDvfsWriteMutex);
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
void MHalDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage, U8 dwCpu)
{
    int     dwCluster = getCpuCluster(dwCpu);

    if(hMstarDvfsInfo[dwCluster].bSystemResumeFlag != 0)
    {
        DVFS_HAL_DEBUG("\033[1;31m[DVFS] Re-init Power Control Flow\033[0m\n");

#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
        //SysDvfsCpuPowerInit();
#else
        MDrvDvfsVoltageSetup(0, 0, 0xFE, dwCpu);
#endif
        hMstarDvfsInfo[dwCluster].bSystemResumeFlag = 0;
    }
    else
    {
        if (!IS_ERR(gFile))
        {
            mutex_lock(&MHalDvfsWriteMutex);
            if (!gBeClosed)
            {
                snprintf(gDVFS_BUFFER,sizeof(gDVFS_BUFFER),"|voltage = %d%c",dwCpuPowerVoltage, '\0');
                vfs_write(gFile, gDVFS_BUFFER, strlen(gDVFS_BUFFER), &gPos);
            }
            mutex_unlock(&MHalDvfsWriteMutex);
        }
        #if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
            SysDvfsCpuPowerAdjustment(dwCpuPowerVoltage, dwCluster);
        #else
            MDrvDvfsVoltageSetup(0, dwCpuPowerVoltage, 0, dwCpu);
        #endif
    }
    DVFS_HAL_DEBUG("[DVFS] CPU Power: %d\n", (unsigned int) dwCpuPowerVoltage);
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
        if(hMstarDvfsInfo[dwCluster].bDvfsModeChange == 1 && gDVFS_auto_measurement == 0 )
        //if(hMstarDvfsInfo[dwCluster].bDvfsModeChange == 1 )
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

            if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
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
                else if(hMstarDvfsInfo[dwCluster].dwCpuTemperature <
                        hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerLevelTemperature)
                {
                if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Over-Temperature->Normal Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        MHalDvfsChangeMode(CONFIG_DVFS_NORMAL_MODE);
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Over-Temperature Mode
                    MHalDvfsChangeMode(CONFIG_DVFS_OVER_TEMPERATURE_MODE);
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                }
            }
            else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
            if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Boot->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Over-Temperature Mode
                        MHalDvfsChangeMode(CONFIG_DVFS_OVER_TEMPERATURE_MODE);
                        //hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                        MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock, dwCpu);
                        hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
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
                            MHalDvfsChangeMode(CONFIG_DVFS_NORMAL_MODE);
                        }
                        else
                        {
                            MHalDvfsChangeMode(CONFIG_DVFS_FREEZE_MODE);
                        }

                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Boot Mode->Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

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
                        MHalDvfsChangeMode(CONFIG_DVFS_NORMAL_MODE);
                        DVFS_HAL_DEBUG("[DVFS] Normal Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
                }
            }
            else if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_NORMAL_MODE) || (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_SPECIAL_CLOCK_MODE))
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_AVERAGE_COUNT)
                    {
                        DVFS_HAL_DEBUG("[DVFS] Normal->Over-Temperature Mode Counter: %d\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwTemperatureCounter);
                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter ++;
                    }
                    else
                    {
                        //Upper Level Threshold Temperature in Normal Mode
                        MHalDvfsChangeMode(CONFIG_DVFS_OVER_TEMPERATURE_MODE);
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                        DVFS_HAL_DEBUG("[DVFS] Over Temperature Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                        MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock, dwCpu);
                        hMstarDvfsInfo[dwCluster].dwFinalCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
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
                        MHalDvfsChangeMode(CONFIG_DVFS_FREEZE_MODE);
                        hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = CONFIG_DVFS_CPU_CLOCK_SPECIAL;
                        DVFS_HAL_DEBUG("[DVFS] Freeze Mode, Support CPU Clock: %dMHz\n", (unsigned int) hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature);

                        hMstarDvfsInfo[dwCluster].dwTemperatureCounter = 0;
                    }
                }
                else
                {
                    //Keep at Normal Mode
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
                }
            }
            else
            {
                hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
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
    MHalDvfsRegisterWrite(0x002050,0x0144);
    MHalDvfsRegisterWrite(0x002050,0x2144);
    while( (MHalDvfsRegisterRead(0x002050) & (0x01<<13) ) != 0);
    dwRegisterValue = MHalDvfsRegisterRead(0x002058);
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
    dwRegisterBackup = MHalDvfsRegisterRead(0x001400);

    //Change SAR to single mode
    MHalDvfsRegisterWrite(0x001400,0x0AB0);
    //Read the voltage for the CORE_PWR_ONLINE_DET[0~3]
    MHalDvfsRegisterWrite(0x001404,0x102);

    usleep_range(1000,1000);

    //tooggle to update sar code data
    MHalDvfsRegisterWrite_SetBit(0x001400,DVFS_BIT14);

    dwTempData = MHalDvfsRegisterRead(0x001480);

    //Recover the Status
    MHalDvfsRegisterWrite(0x001404,0x0);
    MHalDvfsRegisterWrite(0x001400,dwRegisterBackup);

    mutex_unlock(&MDrvDvfsCpuTempMutex);

    return dwTempData*100*(10/3)/1024; //result for *3.3/1024
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
    MHalDvfsRegisterWrite(0x002050,0x01ac);
    MHalDvfsRegisterWrite(0x002050,0x21ac);
    while( (MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0 );
    dwRegisterValue = MHalDvfsRegisterRead(0x002058);
    dwRegisterValue += (MHalDvfsRegisterRead(0x00205a) << 16);
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
        MHalDvfsRegisterWrite(0x002050,0x01ac);
        MHalDvfsRegisterWrite(0x002050,0x21ac);
        while( (MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0 );
        dwRegisterValue = MHalDvfsRegisterRead(0x002058);
        dwRegisterValue += (MHalDvfsRegisterRead(0x00205a) << 16);
        dwRegisterValue = (dwRegisterValue >> 10) & 0x3ff;
    }
    else
    {
        MHalDvfsRegisterWrite(0x002050,0x01b4);
        MHalDvfsRegisterWrite(0x002050,0x21b4);
        while( (MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0 );
        dwRegisterValue = MHalDvfsRegisterRead(0x002058);
        dwRegisterValue += (MHalDvfsRegisterRead(0x00205a) << 16);
        dwRegisterValue = (dwRegisterValue >> 2) & 0x3ff;
    }

    return dwRegisterValue;
}

//=================================================================================================
U8 MHalDvfsGetFreqTable(U8 dwCpu, struct cpufreq_frequency_table **freq_table)
{
    U8  dwCluster = getCpuCluster(dwCpu);
    U8  idx = 0;
    U8  u8Size = 0;
    u8  gap = 50; // 50MHz
    u32 max = 0;
    u32 min = 0;
    u32 freq = 0;

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    max = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMaximumCpuClock;
    min = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;

    u8Size = (max - min)/gap + 2;
    *freq_table = kzalloc(sizeof(struct cpufreq_frequency_table) * u8Size, GFP_KERNEL);
    for (freq = min; freq <= max ; freq +=gap)
    {
        (*freq_table)[idx].frequency = freq * 1000;
        idx++;
    }

    (*freq_table)[idx].frequency = CPUFREQ_TABLE_END;

    for (idx = 0;idx < u8Size; idx++)
    {
        //printk("freq_table: cpu= %d, index = %d, frequency = %d\n",dwCpu,idx,(*freq_table)[idx].frequency );
        DVFS_HAL_DEBUG("freq_table: cpu= %d, index = %d, frequency = %d\n",dwCpu,idx,(*freq_table)[idx].frequency );
    }

    return u8Size;
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
    // add over temperature mode in opp_table
    opp_table[dwCpu].per_cpu_table[0].clk = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsOverTemperatureModeInfo.DvfsCpuInfo.dwLowerCpuClock * 1000000;
    opp_table[dwCpu].per_cpu_table[0].volt = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower* 10000;
    printk("opp_table[%d][%d].clk: %lu  volt: %lu \n", dwCpu, 0, opp_table[dwCpu].per_cpu_table[0].clk,opp_table[dwCpu].per_cpu_table[0].volt);

    // add normal mode in opp_table
    for (dwLoop = 1; dwLoop < CONFIG_DVFS_POWER_CTL_SEGMENT + 1; dwLoop++)
    {
        opp_table[dwCpu].per_cpu_table[dwLoop].clk = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwLoop-1].DvfsCpuInfo.dwLowerCpuClock * 1000000;
        opp_table[dwCpu].per_cpu_table[dwLoop].volt = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsNormalModeInfo[dwLoop-1].DvfsPowerInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type].dwCpuPower * 10000;
        printk("opp_table[%d][%d].clk: %lu  volt: %lu \n", dwCpu, dwLoop, opp_table[dwCpu].per_cpu_table[dwLoop].clk,opp_table[dwCpu].per_cpu_table[dwLoop].volt);
    }
}
//=================================================================================================
void MHalDvfsGetTemperatureInfo(U8 dwCpu, DVFS_THERMAL_INFO *thermal_info)
{
   // not support
}
//=================================================================================================
#if defined(CONFIG_THERMAL_GOV_STEP_WISE) || defined(CONFIG_THERMAL_GOV_POWER_ALLOCATOR)

#define SOC_SENSOR "SENSOR_TEMP_SOC"
enum cluster_type {
    CLUSTER_LITTLE = 0,
    NUM_CLUSTERS
};

struct cluster_power_coefficients {
    int dyn_coeff;
};

struct cluster_power_coefficients SS_cluster_data[] = {
    [CLUSTER_LITTLE] = {
        .dyn_coeff = 159,
    },
};

struct cluster_power_coefficients FF_cluster_data[] = {
    [CLUSTER_LITTLE] = {
        .dyn_coeff = 166,
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
static const int vt_temperatures[] = { 50, 75, 100, 115, 130};

/* Voltages on power over-temp-and-voltage curve (mV) */
static const int *vt_voltages;
#define POWER_TABLE_NUM_TEMP 5
#define POWER_TABLE_NUM_VOLT 6
/* Voltages on power over-temp-and-voltage curve (mV) */
static const int SS_vt_voltages[POWER_TABLE_NUM_VOLT] = { 900000, 1000000, 1050000, 1100000, 1150000, 1250000};

static const int TT_vt_voltages[POWER_TABLE_NUM_VOLT] = { 900000, 1000000, 1050000, 1100000, 1150000, 1250000};

static const int FF_vt_voltages[POWER_TABLE_NUM_VOLT] = { 850000, 950000,  1000000, 1050000, 1100000, 1200000};

static unsigned int *power_table[POWER_TABLE_NUM_VOLT];

static const unsigned int
SS_power_table[POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   50     75    100      115     130 */
        {10000, 23000,  55000,  94000,  159000},  /*  900 mV */
        {15000, 32000,  72000,  120000, 196000},  /*  1000 mV */
        {20000, 39000,  85000,  136000, 223000},  /* 1050 mV */
        {26000, 49000,  102000, 166000, 255000},  /* 1100 mV */
        {36000, 62000,  120000, 185000, 294000},  /* 1150 mV */
        {71000, 111000, 195000, 280000, 429000},  /* 1250 mV */
};

static const unsigned int
TT_power_table[POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   50     75    100      115     130 */
        {10000, 23000,  55000,  94000,  159000},  /*  900 mV */
        {15000, 32000,  72000,  120000, 196000},  /*  1000 mV */
        {20000, 39000,  85000,  136000, 223000},  /* 1050 mV */
        {26000, 49000,  102000, 166000, 255000},  /* 1100 mV */
        {36000, 62000,  120000, 185000, 294000},  /* 1150 mV */
        {71000, 111000, 195000, 280000, 429000},  /* 1250 mV */

};

static const unsigned int
FF_power_table[POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   50     75    100      115     130 */
        {21000, 51000,  129000, 222000, 410000},  /*  850 mV */
        {28000, 70000,  172000, 282000, 500000},  /*  950 mV */
        {33000, 79000,  182000, 306000, 545000},  /* 1000 mV */
        {42000, 95000,  220000, 369000, 652000},  /* 1050 mV */
        {50000, 108000, 241000, 403000, 720000},  /* 1100 mV */
        {80000, 160000, 332000, 554000, 1043000},  /* 1200 mV */

};

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
#if defined(CONFIG_ENERGY_MODEL)
int active_power(unsigned long *power, unsigned long *freq, int cpu)
{
    unsigned long MainCpu = getClusterMainCpu(cpu);
    int i, ret = 0;
    u32 freq_mhz, voltage_mv, num_opps;
    u64 capacitance = 0;
    u64 est_power;
    struct dev_pm_opp *opp = NULL;

    /* check cpu device */
    struct device *dev = get_cpu_device(MainCpu);
    if (!dev) {
        pr_err("No cpu device for cpu %d\n",cpu);
        return -1;
    }
    /* check opp table */
    num_opps = dev_pm_opp_get_opp_count(dev);
    if (num_opps <= 0) {
        pr_err("size of opp table should not be zero\n");
        return -1;
    }
    /* based corner chip type to set dynamic power coefficient */
    if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 0 || DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)
        capacitance = SS_cluster_data[i].dyn_coeff;
    else
        capacitance = FF_cluster_data[i].dyn_coeff;

    rcu_read_lock();
    *freq = (*freq) * 1000;
    opp = dev_pm_opp_find_freq_ceil(dev, freq);
    printk("[opp] freq from opp = %d Hz\n",*freq);

    freq_mhz = (*freq) / 1000000;
    voltage_mv = dev_pm_opp_get_voltage(opp) / 1000;
    rcu_read_unlock();
    /*
     * Do the multiplication with MHz and millivolt so as
     * to not overflow.
     */
    est_power = capacitance * freq_mhz * voltage_mv * voltage_mv;
    do_div(est_power, 1000000000);

    /* frequency is stored in power_table in KHz */
    *freq = (*freq) / 1000;

    /* power is stored in mW */
    *power = est_power;

    printk("[energy model] freq = %d KHz, power = %d mw\n",*freq, *power);
    return 0;
}
#endif
#endif


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

int get_static_power(cpumask_t *cpumask, int interval,
        unsigned long u_volt, u32 *power)
{
    struct thermal_zone_device *tzd = mstar_temp_sensor.tzd;
    int temperature = 0;
    int low_idx = 0, high_idx = 0;
    int i;

    if (tzd){
        temperature = tzd->temperature;
        do_div(temperature, 1000);
    }

    for (i = 0; i < POWER_TABLE_NUM_VOLT; i++) {
        if (u_volt <= vt_voltages[POWER_TABLE_NUM_VOLT - 1 - i])
            high_idx = POWER_TABLE_NUM_VOLT - 1 - i;

        if (u_volt >= vt_voltages[i])
            low_idx = i;
    }

    if (low_idx == high_idx) {
        *power = interpolate(temperature,
                vt_temperatures,
                &power_table[low_idx][0],
                POWER_TABLE_NUM_TEMP);
    } else {
        unsigned long dvt =
            vt_voltages[high_idx] - vt_voltages[low_idx];
        unsigned long power1, power2;

        power1 = interpolate(temperature,
                vt_temperatures,
                &power_table[high_idx][0],
                POWER_TABLE_NUM_TEMP);

        power2 = interpolate(temperature,
                vt_temperatures,
                &power_table[low_idx][0],
                POWER_TABLE_NUM_TEMP);

        *power = (power1 - power2) * (u_volt - vt_voltages[low_idx]);
        do_div(*power, dvt);
        *power += power2;
    }
    // convert to mw
    do_div(*power, 1000);
    return 0;
}
EXPORT_SYMBOL(get_static_power);

static int get_temp(void *data,int *temp)
{
    U32 max = 0;
    U32 temperature = 0;
    U32 i;
    for_each_online_cpu(i)
    {
        temperature = MHalDvfsGetCpuTemperature(i);
        if (temperature > max)
        max = temperature;
    }
    *temp = max * 1000;
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
    struct thermal_cooling_device *cdev;
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }

    platform_set_drvdata(pdev, sensor_data);
    for_each_possible_cpu(cpu) {
        if ( cpu < CONFIG_NR_CPUS)
        {
            int cluster_id = topology_physical_package_id(cpu);
            if (cluster_id > NUM_CLUSTERS) {
                dev_warn(&pdev->dev, "Cluster id: %d > %d\n",
                    cluster_id, NUM_CLUSTERS);
                    goto error;
            }

            cpumask_set_cpu(cpu, &sensor_data->cluster[cluster_id]);
        }
    }

    for (i = 0, j = 0; i < NUM_CLUSTERS; i++) {
        char node[16];

        snprintf(node, 16, "cluster%d", i);
        np = of_find_node_by_name(NULL, node);

        if (!np) {
            dev_info(&pdev->dev, "Node not found: %s\n", node);
            continue;
        }

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
#if defined(CONFIG_ENERGY_MODEL)
        struct em_data_callback energy_data;
        energy_data.active_power = active_power;
        if(em_register_perf_domain(&(sensor_data->cluster[i]),CONFIG_DVFS_POWER_CTL_SEGMENT,&energy_data))
            printk("[energy model] register energy model fail\n");
#endif
#endif

        if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 0 || DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)// SS/TT Corner IC
		{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
            cdev = of_cpufreq_cooling_register(ondemand_timer[getClusterMainCpu(i)].policy);
            if(cdev == NULL)
                printk("[cooling device] register cooling device fail\n");
            else
                sensor_data->cdevs[j] = cdev;
#else
            sensor_data->cdevs[j] =
            of_cpufreq_power_cooling_register(np,
                              &sensor_data->cluster[i],
                              SS_cluster_data[i].dyn_coeff,
                              get_static_power);
#endif
        }
        else // FF Corner IC
		{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
            cdev = of_cpufreq_cooling_register(ondemand_timer[getClusterMainCpu(i)].policy);
            if(cdev == NULL)
                printk("[cooling device] register cooling device fail\n");
            else
                sensor_data->cdevs[j] = cdev;
#else
            sensor_data->cdevs[j] =
            of_cpufreq_power_cooling_register(np,
                              &sensor_data->cluster[i],
                              FF_cluster_data[i].dyn_coeff,
                              get_static_power);
#endif
        }

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
    { .compatible = "mtk-thermal" },
    {},
};
MODULE_DEVICE_TABLE(of, mstar_thermal_of_match);

static struct platform_driver mstar_thermal_platdrv = {
    .driver = {
        .name       = "mtk-thermal",
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

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100500 << 1));
    }
    for (i = 0; i < POWER_TABLE_NUM_VOLT ; i++)
    {
        power_table[i] = kzalloc(sizeof(unsigned int) * POWER_TABLE_NUM_TEMP ,GFP_KERNEL);
        if (!power_table[i])
        {
            printk("power_allocation power_table kzalloc fail\n");
	    return 0;
        }
        printk("DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type:%d\n",DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type);
        if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 0)// SS Corner IC
        {
            memcpy(power_table[i],SS_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
        }
        else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)// TT Corner IC
        {
             memcpy(power_table[i],TT_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
        }
        else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 2)// FF Corner IC
        {
            memcpy(power_table[i],FF_power_table[i],sizeof(unsigned int)*POWER_TABLE_NUM_TEMP);
        }
        else
        {
            printk("Power_allocation error, Check corner IC information\n");
        }
        for (j = 0 ; j < POWER_TABLE_NUM_TEMP ; j++)
        {
            printk("power_table[%d][%d]:%d\n",i,j,power_table[i][j]);
        }
    }
    vt_voltages = kzalloc(sizeof(int) * POWER_TABLE_NUM_VOLT ,GFP_KERNEL);
    if (!vt_voltages)
    {
	printk("power_allocation vt_voltages kzalloc fail\n");
	return 0;
    }
    if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 0)// SS Corner IC
    {
        memcpy(vt_voltages,SS_vt_voltages,sizeof(int)*POWER_TABLE_NUM_VOLT);
    }
    else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)// TT Corner IC
    {
        memcpy(vt_voltages,TT_vt_voltages,sizeof(int)*POWER_TABLE_NUM_VOLT);
    }
    else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 2)// FF Corner IC
    {
        memcpy(vt_voltages,FF_vt_voltages,sizeof(int)*POWER_TABLE_NUM_VOLT);
    }
    else
    {
        printk("Power_allocation error, Check corner IC information\n");
    }
    for (j = 0 ; j < POWER_TABLE_NUM_VOLT ; j++)
    {
        printk("vt_voltages[%d]:%d\n",i,vt_voltages[j]);
    }

    platform_driver_register(&mstar_thermal_platdrv);
    printk("Register power_allocation driver");
    return 0;
}

#endif //CONFIG_THERMAL_GOV_POWER_ALLOCATOR
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
void MHalDvfsSetOverTempDebugOffset(unsigned int cpu,int set_offset)
{
    gDVFS_TEMP_OFFSET = set_offset;
}
int MHalDvfsGetOverTempDebugOffset(unsigned int cpu)
{
    return gDVFS_TEMP_OFFSET;
}

//=================================================================================================
void MHalDvfsOpenFile(char *path)
{
    gFile = filp_open(path, O_RDWR | O_CREAT | O_APPEND,0644);
    gFS = get_fs();
    set_fs(KERNEL_DS);
    gBeClosed = 0;
}
//=================================================================================================
void MHalDvfsCloseFile(void)
{
    mutex_lock(&MHalDvfsWriteMutex);
    filp_close(gFile, NULL);
    set_fs(gFS);
    gBeClosed = 1;
    gFile = NULL;
    mutex_unlock(&MHalDvfsWriteMutex);
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
        hMstarDvfsInfo[cluster].bDvfsInitOk = 0;
        DvfsRegInfo->dvfs_reg[cluster].reg_vid_dvfs_id = 0;
    }
    return 0;
}

static int mstar_dvfs_drv_resume(struct device *dev)
{
    U8 cluster = 0;
    DVFS_HAL_DEBUG("[DVFS] Enter Resume Mode\n");
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
            if (i < CONFIG_NR_CPUS)
            {
#ifdef CONFIG_ARM64
                if (MIDR_PARTNUM(get_cpu_midr(i)) == hMstarDvfsInfo[j].dwCpuPartId) {
#else
                if (get_cpu_midr(i) == hMstarDvfsInfo[j].dwCpuPartId) {
#endif
                    hMstarDvfsInfo[j].dwClusterCpuMask |= (0x1<<i);
                }
            }
        }
        printk("Cluster:%d CpuMask:%x\n", j, hMstarDvfsInfo[j].dwClusterCpuMask);
    }

}

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
#if defined(CONFIG_THERMAL_GOV_STEP_WISE) || defined(CONFIG_THERMAL_GOV_POWER_ALLOCATOR)
late_initcall(mstar_power_allocation_init);
#endif


//i2c drive (Level 3) rmust earlier than dvfs driver
//core_initcall(mstar_dvfs_init);    // Level 1
subsys_initcall(mstar_dvfs_init);    // Level 4
module_exit(mstar_dvfs_exit);

//=================================================================================================
