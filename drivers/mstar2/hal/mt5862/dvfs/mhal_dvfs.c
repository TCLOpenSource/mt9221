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

#include "mdrv_dvfs.h"
#include "mdrv_CPU_cluster_calibrating.h"

#ifndef __MHAL_DVFS_H__
#include "mhal_dvfs.h"
#endif

#ifndef __MHAL_DVFS_POWER_H__
#include "mhal_dvfs_power.h"
#endif

#ifndef __MHAL_DVFS_TABLE_H__
#include "mhal_dvfs_table.h"
#endif

#include "chip_dvfs_calibrating.h"

#include <linux/platform_device.h>
#include <linux/pm.h>
#include <asm/cputype.h>

#include <linux/slab.h>

extern struct mstar_cpufreq_policy ondemand_timer[CONFIG_NR_CPUS];

DEFINE_MUTEX(MDrvDvfsCpuTempMutex);
DEFINE_MUTEX(MHalDvfsGetCpuFreqMutex);
DEFINE_MUTEX(MHalDvfsWriteMutex);

static struct file *gFile = NULL;
static loff_t gPos;
static char gDVFS_BUFFER[100];
static mm_segment_t gFS;
static int gBeClosed = 1;
static U32 bEnable = 1;

extern unsigned int get_cpu_midr(int cpu);
#define CONFIG_DVFS_CPU_CLOCK_DISPLAY_ENABLE 1
extern void MDrvDvfsVoltageSetup(unsigned int dwCpuClock, unsigned int dwVoltage, unsigned int dwVoltageType, unsigned int dwCpu);

static U32 gDVFS_TEMP_OFFSET = 0;

int halTotalClusterNumber;

static U32 gDVFS_auto_measurement=0;
static U16 u16CpuClockSet[DVFS_CLUSTER_NUM];
static volatile MSTAR_DVFS_REG_INFO *DvfsRegInfo = 0;
static U32 u32DefaultClock = 0;

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
    return (abs(cur_vol-set_vol*30)<=20)?0:1;
}


int MHalDvfsGetStatus(unsigned int cpu)
{
    int dwCluster = getCpuCluster(cpu);
    return DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state;
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

    DVFS_HAL_INFO("%s %d: ERROR can't find cluster:%d\n",__func__,__LINE__, cluster);
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
//=================================================================================================
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
U32 MHalDvfsVidProtectCheck(U8 dwCpu)
{
    int dwCluster = getCpuCluster(dwCpu);
    if (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_INIT_MODE)
    {
        MHalDvfsInit(dwCluster);
        MHalDvfsCpuTemperature(dwCluster);
        hMstarDvfsInfo[dwCluster].bDvfsModeChange = 1;
        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state = CONFIG_DVFS_NORMAL_MODE;
        u32DefaultClock = MHalDvfsCpuDisplay(dwCluster);
    }

    if ( DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_NORMAL_MODE)
    {
        MHalDvfsCpuClockAdjustment(u32DefaultClock,dwCpu);
    }
    else
    {
        MHalDvfsCpuClockAdjustment(hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock,dwCpu);
    }
    return MHalDvfsCpuDisplay(dwCluster);
}
//=================================================================================================
U32 MHalDvfsProc(U32 dwCpuClock, U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);

    #if defined(CONFIG_MSTAR_DVFS_PROTECT_ONLY)
    dwCpuClock = MHalDvfsVidProtectCheck(dwCpu);
    MHalDvfsCpuTemperature(dwCluster);
    return dwCpuClock;
    #endif

    if (bEnable == 0)
        return 1100;


    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
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
                    MHalDvfsRegisterWrite(0x00300a,0x00);
                    MHalDvfsRegisterWrite(0x003008,0x00);
                    MHalDvfsRegisterWrite(0x00300a,0x05);
                    MHalDvfsRegisterWrite(0x003000,0x01);

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
        else
        {
            //Normal Mode
            if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
               hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
            {
                //Change to Over-Temperature Mode
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

                    if((dwRegisterValue >= hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwMinimumCpuClock) && \
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
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb0),0);
    }

_MHalDvfsProcExit:

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
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
    }

    MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb2),DVFS_BIT0);

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

    u16CpuClockSet[dwCluster] = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwDefaultCpuClock;

    //Init Test Bus to Measure CPU Clock
    MHalDvfsCpuDisplay(dwCluster);

    if (bEnable)
    {
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
    }
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
    U32  dwRegisterValue = 0;
    int dwCluster = getCpuCluster(dwCpu);
    mutex_lock(&MHalDvfsGetCpuFreqMutex);
    MHalDvfsRegisterWrite_ClearBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) , DVFS_BIT0 );

    udelay(CONFIG_DVFS_CLOCK_SHORT_DELAY_US);

    MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6) , DVFS_BIT0 );

    udelay(CONFIG_DVFS_CLOCK_DELAY_US);

    // Wait calculate done , reg_calc_done
    while( (MHalDvfsRegisterRead((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6)) & 0x3 ) != 0x3 );

    //CPU Clock = 192 * reg_calc_cnt_report(DEC) / 1000 (MHz)
    dwRegisterValue = MHalDvfsRegisterRead((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf4));
    dwRegisterValue *= 192;
    dwRegisterValue /= 1000;
    mutex_unlock(&MHalDvfsGetCpuFreqMutex);
    return dwRegisterValue;
}
//=================================================================================================
int MHalDvfsCpuDisplay(U8 dwCluster)
{
    U32  dwRegisterValue = 0;
    U32  u32RetryCount=0;
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
    int  iLoop;

    mutex_lock(&MDrvDvfsCpuTempMutex);
    if(hMstarDvfsInfo[dwCluster].dwRefTemperature == 0)
    {
        MHalDvfsRefTemperature(dwCluster);
    }

    for(iLoop = 0 ; iLoop < CONFIG_DVFS_DATA_COUNT ; iLoop++ )
    {
        //Read CH8 of PM_SAR to Get CPU Temperature (use external t-sensor)
        MHalDvfsRegisterWrite_ClearBit(0x001420,DVFS_BIT0);

        MHalDvfsRegisterWrite(0x001400,0x0A20);

        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        MHalDvfsRegisterWrite_SetBit(0x001400,DVFS_BIT14);

        dwTempData = MHalDvfsRegisterRead(0x00148e);

        if(dwTempData == 0)
        {
            MHalDvfsRegisterWrite(0x001400,0x0A20);

            udelay(CONFIG_DVFS_CLOCK_DELAY_US);

            MHalDvfsRegisterWrite_SetBit(0x001400,DVFS_BIT14);

            dwTempData = MHalDvfsRegisterRead(0x00148e);
        }

        hMstarDvfsInfo[dwCluster].dwAvgCpuTempBuffer += dwTempData;
        hMstarDvfsInfo[dwCluster].dwAvgCpuTempCounter ++;
    }
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
                dwTempData = (((hMstarDvfsInfo[dwCluster].dwRefTemperature - dwTempValue) * 1250) + CONFIG_DVFS_T_SENSOR_SHIFT);
            }
            else
            {
                dwTempData = ((dwTempValue - hMstarDvfsInfo[dwCluster].dwRefTemperature) * 1250);
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
    U32     dwLowFreq;
    U32     dwHighFreq;
    int     iLeftShit = 0;
    int     dwCluster       = getCpuCluster(dwCpu);
    U32     dwCurrentCpuClock = DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock;

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

    if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock != dwCpuClock) || (hMstarDvfsInfo[dwCluster].bDvfsModeChange == 1))
    {
        if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock < dwCpuClock) && (gDVFS_auto_measurement == 0))
        {
            MHalDvfsPowerControl(dwCpuClock, dwCpu);
        }
        udelay(CONFIG_DVFS_CLOCK_DELAY_US);

        if (!IS_ERR(gFile))
        {
            mutex_lock(&MHalDvfsWriteMutex);
            if (!gBeClosed)
            {
                snprintf(gDVFS_BUFFER,sizeof(gDVFS_BUFFER),"|clock = %d%c",dwCpuClock, '\0');
                vfs_write(gFile, gDVFS_BUFFER, strlen(gDVFS_BUFFER), &gPos);
            }
            mutex_unlock(&MHalDvfsWriteMutex);
        }

        // 1. Set CPU Clock Mux to ARMPLL clk
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf8),0x1);

        dwRegisterValue = MHalDvfsRegisterRead((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0x2e));
        if( dwCpuClock < FREQ_600 )
        {
            /* Range : 180M ~ 599M */
            // Div4
            dwRegisterValue |= 0x7;
            iLeftShit = 2;
        }
        else
        {
            /* Range : 600M ~ 2.5G */
            // Div1
            dwRegisterValue &= ~(0x7);
            dwRegisterValue |= 0x1;
            iLeftShit = 0;
        }
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0x2e),dwRegisterValue);

        if( dwCurrentCpuClock < dwCpuClock )                                                         // Start from Low to High
        {
            dwLowFreq = dwCurrentCpuClock;
            dwHighFreq = dwCpuClock;
        }
        else
        {                                                                                            // Start from High to Low
            dwLowFreq = dwCpuClock;
            dwHighFreq = dwCurrentCpuClock;
        }

        // Set LPF_LOW
        // Range : 600M ~ 2.5G  : Calculate register dac of Current Freq = 3623875 /low_clkfreq_GHz
        // Range : 180M ~ 599M : Calculate register dac of Current Freq = 3623875 /(low_clkfreq_GHz*4)
        dwRegisterValue = ((3623878UL /(dwLowFreq<<iLeftShit)) * 1000);
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa0),(dwRegisterValue & 0xFFFF));
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa2),(dwRegisterValue >> 16) & 0xFFFF);

        // Set LPF_HIGH
        // Range : 600M ~ 2.5G  : Calculate register dac of Current Freq = 3623875 /high_clkfreq_GHz
        // Range : 180M ~ 599M : Calculate register dac of Current Freq = 3623875 /(high_clkfreq_GHz*4)
        dwRegisterValue = ((3623878UL /(dwHighFreq<<iLeftShit)) * 1000);
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa4),(dwRegisterValue & 0xFFFF));
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa6),(dwRegisterValue >> 16) & 0xFFFF);

        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xaa),0x6);                // Set freq tune step , reg_lpu_mu
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xae),0x8);                // Set interval between 2 freq , reg_lpu_update_cnt
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb0),0x1);                // reg_lpf_use_hw
        MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb0),DVFS_BIT8);   // reg_lpf_set_low_auto

        if( dwCurrentCpuClock < dwCpuClock )
        {
            MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb2),0x1330);         // Start from Low to High ,  reg_lpf_low2high
        }
        else
        {
            MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xb2),0x0330);         // Start from High to Low ,  reg_lpf_low2high
        }
        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa8),0x1);                // LPF enable , reg_lpf_enable
        udelay(CONFIG_DVFS_CLOCK_SHORT_DELAY_US);

        // Wait LPF setting done , reg_lpf_low2high_done
        while( ( MHalDvfsRegisterRead((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xba))&0x1) == 0 );

        MHalDvfsRegisterWrite((hMstarDvfsInfo[dwCluster].dwAnaMiscBank + 0xa8),0);                  // LPF disable , reg_lpf_enable

        if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock >= dwCpuClock) && (gDVFS_auto_measurement == 0))
        {
            MHalDvfsPowerControl(dwCpuClock, dwCpu);
        }

        u16CpuClockSet[dwCluster] = dwCpuClock;
        DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_cpu_clock = dwCpuClock;
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
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
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
        if( gDVFS_auto_measurement == 1 )
        {
            dwOutputCpuClock = CPU_AUTO_TEST_MAX_FREQ;
        }
        else
        {
            dwOutputCpuClock = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
        }
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
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
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
                        MHalDvfsRegisterWrite(0x00300a,0x00);
                        MHalDvfsRegisterWrite(0x003008,0x00);
                        MHalDvfsRegisterWrite(0x00300a,0x05);
                        MHalDvfsRegisterWrite(0x003000,0x01);

                        while(1);
                    }
                }
                else if(hMstarDvfsInfo[dwCluster].dwCpuTemperature <
                        hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwLowerLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_TEMPERATURE_LOWER_COUNT)
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
                    hMstarDvfsInfo[dwCluster].dwOverTemperatureFlag = 1;
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwProtectedCpuClock;
                }
            }
            else if(DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_BOOT_MODE)
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_TEMPERATURE_COUNT)
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
                    hMstarDvfsInfo[dwCluster].dwMaxCpuClockByTemperature = hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsSysInfo.dwIRBoostCpuClock;
                }
            }
            else if((DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_NORMAL_MODE) || (DvfsRegInfo->dvfs_reg[dwCluster].reg_cur_dvfs_state == CONFIG_DVFS_SPECIAL_CLOCK_MODE))
            {
                if(hMstarDvfsInfo[dwCluster].dwCpuTemperature >=
                   hMstarDvfsInfo[dwCluster].DvfsModeInfo[DvfsRegInfo->dvfs_reg[dwCluster].reg_chip_package].DvfsTemperatureInfo.dwUpperLevelTemperature)
                {
                    if(hMstarDvfsInfo[dwCluster].dwTemperatureCounter < CONFIG_DVFS_TEMPERATURE_COUNT)
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
    //Init Test Bus to Measure CPU Clock
    MHalDvfsRegisterWrite_SetBit((hMstarDvfsInfo[dwCluster].dwMcuArmBank + 0xf6),DVFS_BIT0);
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

    while((MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0);

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
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
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

    //Read the voltage for the CORE_PWR_ONLINE_DET[0]
    MHalDvfsRegisterWrite(0x001404,0x100);

    udelay(CONFIG_DVFS_CLOCK_DELAY_US);

    //tooggle to update sar code data
    MHalDvfsRegisterWrite_SetBit(0x001400,DVFS_BIT14);
    dwTempData = MHalDvfsRegisterRead(0x001480);

    //Recover the Status
    MHalDvfsRegisterWrite(0x001404,0);
    MHalDvfsRegisterWrite(0x001400,dwRegisterBackup);

    mutex_unlock(&MDrvDvfsCpuTempMutex);

    // Formula : Voltage = DAC * 3.3 / 1024 (V) , return XXXX mV
    return dwTempData*3300/1024;
}

//=================================================================================================
U32 MHalDvfsGetVoltage(U8 dwCpu)
{
    U32    dwClockLevel = 0;
    int    dwCluster = getCpuCluster(dwCpu);

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
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

    while((MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0);

    dwRegisterValue = MHalDvfsRegisterRead(0x002058);
    dwRegisterValue += ( MHalDvfsRegisterRead(0x00205a) << 16);
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

        while((MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0);

        dwRegisterValue = MHalDvfsRegisterRead(0x002058);
        dwRegisterValue += ( MHalDvfsRegisterRead(0x00205a) << 16);
        dwRegisterValue = (dwRegisterValue >> 10) & 0x3ff;
    }
    else
    {
        MHalDvfsRegisterWrite(0x002050,0x01b4);
        MHalDvfsRegisterWrite(0x002050,0x21b4);

        while((MHalDvfsRegisterRead(0x002050) & (0x01 << 13)) != 0);

        dwRegisterValue = MHalDvfsRegisterRead(0x002058);
        dwRegisterValue += ( MHalDvfsRegisterRead(0x00205a) << 16);
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
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
    }
    return DvfsRegInfo->dvfs_reg[dwCluster].reg_cpu_pwr_type;
}

//=================================================================================================
U32 MHalDvfsGetCorePowerType(U8 dwCpu)
{
    int    dwCluster = getCpuCluster(dwCpu);
    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
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
#if (DVFS_DEVICE_TREE_PARSE_ENABLE == 1)
U32 MHal_dvfs_variable_init(struct device_node *node, U32 cluster)
{
    U32 prop = 0;
    U32 bRet = TRUE;
    struct device_node *child = of_find_node_by_name(node, "cpu-info");
    DVFS_DTS_READ_U32(child, "partid", prop, hMstarDvfsInfo[cluster].dwCpuPartId);
    DVFS_DTS_READ_U32(child, "mask", prop, hMstarDvfsInfo[cluster].dwClusterCpuMask);
    DVFS_DTS_READ_U32(child, "freqaddr", prop, hMstarDvfsInfo[cluster].dwFreqRiuAddr);
    DVFS_DTS_READ_U32(child, "anabank", prop, hMstarDvfsInfo[cluster].dwAnaMiscBank);
    DVFS_DTS_READ_U32(child, "mcubank", prop, hMstarDvfsInfo[cluster].dwMcuArmBank);


    hMstarDvfsInfo[cluster].bDvfsInitOk = 0;
    hMstarDvfsInfo[cluster].bDvfsModeChange = 0;
    hMstarDvfsInfo[cluster].dwMaxCpuClockByTemperature = 0;

    hMstarDvfsInfo[cluster].dwFinalCpuClock = 0;
    hMstarDvfsInfo[cluster].dwFinalCpuPowerVoltage = 0;
    hMstarDvfsInfo[cluster].dwFinalCorePowerVoltage = 0;

    hMstarDvfsInfo[cluster].dwCpuTemperature = 0;
    hMstarDvfsInfo[cluster].dwRefTemperature = 0;
    hMstarDvfsInfo[cluster].dwAvgCpuTempCounter = 0;

    hMstarDvfsInfo[cluster].bSystemResumeFlag = 0;
    hMstarDvfsInfo[cluster].dwResetCounter = 0;
    hMstarDvfsInfo[cluster].dwTemperatureCounter = 0;
    hMstarDvfsInfo[cluster].dwBootTimeCounter = 0;
    hMstarDvfsInfo[cluster].dwOverTemperatureFlag = 0;
    return bRet;
}

U32 MHal_dvfs_table_init(struct device_node *node, U32 cluster, U32 package)
{
    U32 prop = 0, level = 0, i = 0;
    U32 bRet = TRUE;
    char temp[100];
    struct device_node *child = of_find_node_by_name(node, "clock-info");
    struct device_node *gchild = NULL, *ggchild = NULL;
    DVFS_DTS_READ_U32(child, "level", prop, level);
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsFreezeModeInfo = (MSTAR_DVFS_ENTRY_INFO *)kmalloc(sizeof(MSTAR_DVFS_ENTRY_INFO)*level,GFP_KERNEL);
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo = (MSTAR_DVFS_ENTRY_INFO *)kmalloc(sizeof(MSTAR_DVFS_ENTRY_INFO)*level,GFP_KERNEL);

    gchild = of_find_node_by_name(child, "pairs");
    for(i = 0; i < level; i++)
    {
        memset(temp,0,100);
        snprintf(temp, 10, "setting_%d", i);
        ggchild = of_find_node_by_name(gchild, temp);

        DVFS_TABLE_PAIR_INIT_LEVEL(ggchild, cluster,package, CONFIG_DVFS_FREEZE_MODE, i,prop);
        DVFS_TABLE_PAIR_INIT_LEVEL(ggchild, cluster,package, CONFIG_DVFS_NORMAL_MODE, i,prop);

    }
    return bRet;
}

U32 MHal_sysinfo_init(struct device_node *node, U32 cluster, U32 package)
{
    U32 prop = 0;
    U32 bRet = TRUE;
    struct device_node *child = NULL;
    child = of_find_node_by_name(node, "sys-info");
    DVFS_SYS_INIT(child, cluster, package,prop);
    return bRet;
}

U32 MHal_tempinfo_init(struct device_node *node, U32 cluster, U32 package)
{
    U32 prop = 0;
    U32 bRet = TRUE;
    struct device_node *child = NULL;
    child = of_find_node_by_name(node, "temp-info");
    DVFS_TEMP_INIT(child, cluster, package,prop);

    // overtemp mode
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsOverTemperatureModeInfo.DvfsCpuInfo.dwLowerCpuClock = \
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsOverTemperatureModeInfo.DvfsCpuInfo.dwUpperCpuClock = \
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwProtectedCpuClock;

    DVFS_DTS_READ_U32(child, "power-ss", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[0].dwCpuPower);
    DVFS_DTS_READ_U32(child, "power-ss", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[1].dwCpuPower);
    DVFS_DTS_READ_U32(child, "power-ff", prop, hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsOverTemperatureModeInfo.DvfsPowerInfo[2].dwCpuPower);

    return bRet;
}


U32 MHal_mode_init(struct device_node *node, U32 cluster, U32 package)
{
    U32 prop = 0,level = 0;
    U32 bRet = TRUE;


    // init mode
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsInitModeInfo.DvfsCpuInfo.dwLowerCpuClock = \
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsInitModeInfo.DvfsCpuInfo.dwUpperCpuClock = \
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwDefaultCpuClock;

    level = MHalDvfsSearchCpuClockLevel(hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwDefaultCpuClock, cluster);
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsInitModeInfo.DvfsPowerInfo[0].dwCpuPower = \
        hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[0].dwCpuPower;

    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsInitModeInfo.DvfsPowerInfo[1].dwCpuPower = \
        hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[1].dwCpuPower;

    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsInitModeInfo.DvfsPowerInfo[2].dwCpuPower = \
        hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[2].dwCpuPower;

    // boot mode
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsBootModeInfo.DvfsCpuInfo.dwLowerCpuClock = \
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsBootModeInfo.DvfsCpuInfo.dwUpperCpuClock = \
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwIRBoostCpuClock;

    level = MHalDvfsSearchCpuClockLevel(hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsSysInfo.dwIRBoostCpuClock, cluster);
    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsBootModeInfo.DvfsPowerInfo[0].dwCpuPower = \
        hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[0].dwCpuPower;

    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsBootModeInfo.DvfsPowerInfo[1].dwCpuPower = \
        hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[1].dwCpuPower;

    hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsBootModeInfo.DvfsPowerInfo[2].dwCpuPower = \
        hMstarDvfsInfo[cluster].DvfsModeInfo[package].DvfsNormalModeInfo[level].DvfsPowerInfo[2].dwCpuPower;

    return bRet;
}

U8 MHal_dvfs_parsing(void)
{
    struct device_node *node, *child, *gchild, *ggchild;
    U32 prop = 0, package = 0, cluster = 0;
    char *string = NULL;
    U8  bRet = TRUE, i = 0, j = 0;
    U8  temp[100];

    if(DvfsRegInfo == NULL)
    {
        DvfsRegInfo = (volatile MSTAR_DVFS_REG_INFO *)(CONFIG_REGISTER_BASE_ADDRESS + (DVFS_REG_BANK << 1));
    }

    node = of_find_node_by_name(NULL, "soc-dvfs");
    DVFS_DTS_READ_U32(node, "enable", prop, bEnable);

    DVFS_DTS_READ_U32(node, "package", prop, package);
    DVFS_DTS_READ_U32(node, "cluster", prop, cluster);

    hMstarDvfsInfo = (MSTAR_DVFS_INFO *)kmalloc(sizeof(MSTAR_DVFS_INFO)*cluster,GFP_KERNEL);

    for(i = 0 ; i < cluster; i++)
    {
        hMstarDvfsInfo[i].DvfsModeInfo = (MSTAR_DVFS_MODE_INFO *)kmalloc(sizeof(MSTAR_DVFS_MODE_INFO)*package,GFP_KERNEL);
        memset(temp,0,100);
        snprintf(temp, 10, "cluster_%d", i);
        child = of_find_node_by_name(node, temp);
        MHal_dvfs_variable_init(child,i);

        for(j = 0; j < package; j++)
        {
            memset(temp,0,100);
            snprintf(temp, 13, "table-info_%d", j);
            gchild = of_find_node_by_name(child, temp);
            MHal_dvfs_table_init(gchild, i, j);

            MHal_sysinfo_init(gchild,i,j);
            MHal_tempinfo_init(gchild,i,j);
            MHal_mode_init(gchild,i,j);
        }

    }
    return bRet;
}
#endif
//=================================================================================================
static struct platform_device mstar_dvfs_dev =
{
    .name   = "mstar_dvfs",
    .id     = 0,
};
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
        .dyn_coeff = 149,
    },
};

struct cluster_power_coefficients FF_cluster_data[] = {
    [CLUSTER_LITTLE] = {
        .dyn_coeff = 159,
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
static const int SS_vt_voltages[POWER_TABLE_NUM_VOLT] = { 900000, 1000000, 1050000, 1100000, 1150000, 1200000};

static const int TT_vt_voltages[POWER_TABLE_NUM_VOLT] = { 900000, 1000000, 1050000, 1100000, 1150000, 1200000};

static const int FF_vt_voltages[POWER_TABLE_NUM_VOLT] = { 900000, 950000,  1000000, 1050000, 1100000, 1150000};

static unsigned int *power_table[POWER_TABLE_NUM_VOLT];

static const unsigned int
SS_power_table[POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   50     75    100      115     130 */
        {10000, 28000,  68000, 114000, 186000},  /*  900 mV */
        {16000, 37000,  86000, 140000, 230000},  /* 1000 mV */
        {20000, 44000, 100000, 161000, 261000},  /* 1050 mV */
        {26000, 53000, 114000, 181000, 293000},  /* 1100 mV */
        {36000, 68000, 136000, 214000, 340000},  /* 1150 mV */
        {50000, 85000, 161000, 245000, 389000},  /* 1200 mV */
};

static const unsigned int
TT_power_table[POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   50     75    100      115     130 */
        {10000, 28000,  68000, 114000, 186000},  /*  900 mV */
        {16000, 37000,  86000, 140000, 230000},  /* 1000 mV */
        {20000, 44000, 100000, 161000, 261000},  /* 1050 mV */
        {26000, 53000, 114000, 181000, 293000},  /* 1100 mV */
        {36000, 68000, 136000, 214000, 340000},  /* 1150 mV */
        {50000, 85000, 161000, 245000, 389000},  /* 1200 mV */
};

static const unsigned int
FF_power_table[POWER_TABLE_NUM_VOLT][POWER_TABLE_NUM_TEMP] = {
        /*   50     75    100      115     130 */
        {26000, 63000,  148000, 240000, 393000},   /*  900 mV */
        {30000, 73000,  165000, 269000, 445000},   /*  950 mV */
        {34000, 82000,  184000, 300000, 505000},   /* 1000 mV */
        {41000, 95000,  212000, 348000, 585000},   /* 1050 mV */
        {52000, 113000, 244000, 402000, 688000},   /* 1100 mV */
        {63000, 131000, 278000, 455000, 787000},  /* 1150 mV */

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
    struct thermal_cooling_device *cdev;
    int i, j;
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
        memcpy((void *)vt_voltages,SS_vt_voltages,sizeof(int)*POWER_TABLE_NUM_VOLT);
    }
    else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 1)// TT Corner IC
    {
        memcpy((void *)vt_voltages,TT_vt_voltages,sizeof(int)*POWER_TABLE_NUM_VOLT);
    }
    else if (DvfsRegInfo->dvfs_reg[0].reg_cpu_pwr_type == 2)// FF Corner IC
    {
        memcpy((void *)vt_voltages,FF_vt_voltages,sizeof(int)*POWER_TABLE_NUM_VOLT);
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
#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)
    SysDvfsPowerSuspend();
#endif
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
#if defined(CONFIG_MSTAR_IIC) && defined(CONFIG_MSTAR_DVFS_KERNEL_IIC)

#else
        hMstarDvfsInfo[cluster].bSystemResumeFlag = 1;
#endif
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
    #if (DVFS_DEVICE_TREE_PARSE_ENABLE == 1)
    MHal_dvfs_parsing();
    #endif
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
            if(i < CONFIG_NR_CPUS)
            {
#ifdef CONFIG_ARM64
                if (MIDR_PARTNUM(get_cpu_midr(i)) == hMstarDvfsInfo[j].dwCpuPartId) {
#else
                if ( ((get_cpu_midr(i) & 0xFFF0) >> 4) == hMstarDvfsInfo[j].dwCpuPartId ) {
#endif
                    hMstarDvfsInfo[j].dwClusterCpuMask |= (0x1<<i);
                }
            }
        }
        DVFS_HAL_INFO("Cluster:%d CpuMask:%x\n", j, hMstarDvfsInfo[j].dwClusterCpuMask);
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

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,0)
core_initcall(mstar_dvfs_init);
#else
subsys_initcall(mstar_dvfs_init);
#endif
module_exit(mstar_dvfs_exit);

//=================================================================================================
