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

#include "mdrv_dvfs.h"

#ifndef __MHAL_DVFS_H__
#include "mhal_dvfs.h"
#endif

#ifndef __MHAL_DVFS_POWER_H__
#include "mhal_dvfs_power.h"
#endif

#ifndef __MHAL_DVFS_TABLE_H__
#include "mhal_dvfs_table.h"
#endif

#include "../../gpio/mdrv_gpio.h"

#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/slab.h>
//=================================================================================================
static U32 dwPowerChipId = CONFIG_DVFS_CHIP_ID_UNKNOWN;
static U32 bDvfsPowerInitStatus = FALSE;
static unsigned int g_u32VoltageUpper[DVFS_CLUSTER_NUM];
static unsigned int g_u32VoltageLower[DVFS_CLUSTER_NUM];

#if CONFIG_DVFS_CPU_POWER_GPIO_ENABLE
static unsigned int num_of_gpio = 0;
static unsigned int control_level = 0;
static unsigned int *gpio_num = NULL;
static unsigned int *power_control = NULL;
static unsigned int **gpio_control = NULL;
static U32 bDvfsPowerInfoParse = FALSE;
#endif

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_SWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_SWIIC == 1)))
extern S32 MDrv_SW_IIC_WriteBytes(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf);
extern S32 MDrv_SW_IIC_ReadBytes(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf);
#endif

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_HWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_HWIIC == 1)))
extern S32 MDrv_HW_IIC_WriteBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);
extern S32 MDrv_HW_IIC_ReadBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);
#endif
////////////////////////////////////////////////////////////////////////////////
S32 MDrv_DVFS_IIC_Write(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf)
{
    S32 s32Ret = -1;

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_SWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_SWIIC == 1)))
    s32Ret = MDrv_SW_IIC_WriteBytes(u8ChIIC, u8SlaveID, u8AddrCnt, pu8Addr, u32BufLen, pu8Buf);
#endif

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_HWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_HWIIC == 1)))
    s32Ret = MDrv_HW_IIC_WriteBytes(u8ChIIC,u8SlaveID, u8AddrCnt, pu8Addr, u32BufLen, pu8Buf);
#endif

    return(s32Ret);
}
////////////////////////////////////////////////////////////////////////////////
S32 MDrv_DVFS_IIC_Read(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf)
{
    S32 s32Ret = -1;

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_SWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_SWIIC == 1)))
    s32Ret = MDrv_SW_IIC_ReadBytes(u8ChIIC, u8SlaveID, u8AddrCnt, pu8Addr, u32BufLen, pu8Buf);
#endif

#if ((defined(CONFIG_MSTAR_DVFS_KERNEL_HWIIC) && (CONFIG_MSTAR_DVFS_KERNEL_HWIIC == 1)))
    s32Ret = MDrv_HW_IIC_ReadBytes(u8ChIIC,u8SlaveID, u8AddrCnt, pu8Addr, u32BufLen, pu8Buf);
#endif

    return(s32Ret);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: SysDvfsInit
/// @brief \b Function  \b Description: Read T-Sensor to Handle DVFS Flow
/// @param <IN>         \b None:
/// @param <OUT>        \b None:
/// @param <RET>        \b None:
/// @param <GLOBAL>     \b None:
////////////////////////////////////////////////////////////////////////////////
void SysDvfsPowerSuspend(void)
{
    bDvfsPowerInitStatus = false;
}
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsPowerInit(U8 byCluster)
{
    if (bDvfsPowerInitStatus == TRUE)
        return TRUE;

    if (SysDvfsCpuPowerInit() == TRUE)
    {
        //SysDvfsCpuPowerAdjustment(CONFIG_DVFS_CPU_POWER_DEFAULT(byCluster), byCluster);
    }
    else
    {
        bDvfsPowerInitStatus = FALSE;
        return bDvfsPowerInitStatus;
    }

    bDvfsPowerInitStatus = TRUE;
    return bDvfsPowerInitStatus;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: SysDvfsCpuPowerAdjustment
/// @brief \b Function  \b Description: Update Output Voltage Level in External Power Chip
/// @param <IN>         \b None:
/// @param <OUT>        \b None:
/// @param <RET>        \b None:
/// @param <GLOBAL>     \b None:
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage, U8 byCluster)
{
    U32 bDvfsCpuPowerAdjStatus = TRUE;
    U16 u16ControlType = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100508 << 1));
    u16ControlType = u16ControlType & 0x03;
    if (u16ControlType == 0)
    {
#if CONFIG_DVFS_CPU_POWER_I2C_ENABLE
        U32  dwRegisterValue = 0;
        U32  dwOriginalCpuPowerVoltage = 0;
        U32  dwSourceRegisterSetting = 0;
        U32  dwTargetRegisterSetting = 0;

        U8   byTargetRegAddress[5] =
                {
                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF
                };
        U8   byTargetData[5] =
                {
                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF
                };

        if(byCluster >= DVFS_CLUSTER_NUM)
        {
            return FALSE;
        }
        getCpuBoundInfo(byCluster,&g_u32VoltageLower[byCluster],&g_u32VoltageUpper[byCluster]);
        if( dwPowerChipId == CONFIG_DVFS_CHIP_ID_UNKNOWN )
        {
            SysDvfsCpuPowerInit();
        }
        if(dwCpuPowerVoltage < CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byCluster))
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] dwCpuPowerVoltage is smaller than CONFIG_DVFS_CPU_POWER_SHIFT_PRADO!\033[m\n");
            return FALSE;
        }
        else if(dwCpuPowerVoltage > g_u32VoltageUpper[byCluster])
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] dwCpuPowerVoltage is larger than CONFIG_DVFS_CPU_POWER_MAX!\033[m\n");
            return FALSE;
        }

        if (dwPowerChipId == CONFIG_DVFS_CHIP_ID_PRADO)
        {
            byTargetRegAddress[0] = 0x10;
            byTargetRegAddress[1] = (0x06 << 1);
            if (MDrv_DVFS_IIC_Read(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byCluster), 2, byTargetRegAddress, 2, byTargetData) > 0)
            {
                dwOriginalCpuPowerVoltage = (unsigned int) byTargetData[1] + CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byCluster);
                DVFS_HAL_DEBUG("\033[33m[INFO] Orginal CPU Power: (%d)%d0 mV\033[m\n", byCluster, (unsigned int) dwOriginalCpuPowerVoltage);
            }
            else
            {
                DVFS_HAL_DEBUG("\033[37m[ERROR] I2C Read Bytes with CONFIG_DVFS_POWER_SWI2C_ADDR_CPU Fail!\033[m\n");
            }

            if ((dwOriginalCpuPowerVoltage < g_u32VoltageLower[byCluster]) || (dwOriginalCpuPowerVoltage > g_u32VoltageUpper[byCluster]))
            {
                DVFS_HAL_DEBUG("[ERROR] Original Voltage (%d)%d0 mV is invalid\n",byCluster,dwOriginalCpuPowerVoltage);
                dwOriginalCpuPowerVoltage = DVFS_HIGH_BOUND_VOLTAGE;
            }
            dwSourceRegisterSetting = (dwOriginalCpuPowerVoltage - CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byCluster));
            dwTargetRegisterSetting = (dwCpuPowerVoltage - CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byCluster));

            if (bDvfsPowerInitStatus == TRUE)
            {
                if (dwCpuPowerVoltage > dwOriginalCpuPowerVoltage)
                {
                    for (;dwSourceRegisterSetting <= dwTargetRegisterSetting; dwSourceRegisterSetting += CONFIG_DVFS_CPU_POWER_STEP(byCluster))
                    {
                        //Set CPU Voltage
                        dwRegisterValue = dwSourceRegisterSetting;
                        byTargetRegAddress[0] = 0x10;
                        byTargetRegAddress[1] = (0x06 << 1);
                        byTargetRegAddress[2] = 0x10;
                        byTargetRegAddress[3] = dwRegisterValue;
                        if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byCluster), 0, byTargetRegAddress, 4, byTargetRegAddress) > 0)
                        {
                            DVFS_HAL_DEBUG("\033[37m[INFO] Change to Voltage: (%d)%d0 mV (0x%X)\033[m\n", byCluster, (unsigned int) dwCpuPowerVoltage, (unsigned int) dwRegisterValue);
                        }
                        else
                        {
                            DVFS_HAL_DEBUG("\033[37m[ERROR] kernel I2C Write Failed\033[m\n");
                        }
                    }
                }
                else if (dwCpuPowerVoltage < dwOriginalCpuPowerVoltage)
                {
                    for (;dwSourceRegisterSetting >= dwTargetRegisterSetting; dwSourceRegisterSetting -= CONFIG_DVFS_CPU_POWER_STEP(byCluster))
                    {
                        //Set CPU Voltage
                        dwRegisterValue = dwSourceRegisterSetting;
                        byTargetRegAddress[0] = 0x10;
                        byTargetRegAddress[1] = (0x06 << 1);
                        byTargetRegAddress[2] = 0x10;
                        byTargetRegAddress[3] = dwRegisterValue;
                        if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byCluster), 0, byTargetRegAddress, 4, byTargetRegAddress) > 0)
                        {
                            DVFS_HAL_DEBUG("\033[37m[INFO] Change to Voltage: (%d)%d0 mV (0x%X)\033[m\n", byCluster, (unsigned int) dwCpuPowerVoltage, (unsigned int) dwRegisterValue);
                        }
                        else
                        {
                            DVFS_HAL_DEBUG("\033[37m[ERROR] kernel I2C Write Failed\033[m\n");
                        }
                    }
                }
                else
                {
                    DVFS_HAL_DEBUG("\033[37m[INFO] No Need to Change CPU Power\033[m\n");
                    return bDvfsCpuPowerAdjStatus;
                }
            }

            if (dwSourceRegisterSetting != dwTargetRegisterSetting)
            {
                //Set CPU Voltage
                dwRegisterValue = (dwCpuPowerVoltage - CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byCluster));
                byTargetRegAddress[0] = 0x10;
                byTargetRegAddress[1] = (0x06 << 1);
                byTargetRegAddress[2] = 0x10;
                byTargetRegAddress[3] = dwRegisterValue;
                if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byCluster), 0, byTargetRegAddress, 4, byTargetRegAddress) > 0)
                {
                    DVFS_HAL_DEBUG("\033[37m[INFO] Change to Voltage: (%d)%d0 mV (0x%X)\033[m\n", byCluster, (unsigned int) dwCpuPowerVoltage, (unsigned int) dwRegisterValue);
                }
                else
                {
                    DVFS_HAL_DEBUG("\033[37m[ERROR] kernel I2C Write Failed\033[m\n");
                }
            }
        }
        else if (dwPowerChipId == CONFIG_DVFS_CHIP_ID_SY8824C)
        {
           byTargetData[0] = (((100 * dwCpuPowerVoltage) - 7625) / 123) | 0x80;
           byTargetRegAddress[0] = 0x00;
           if(MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET,CONFIG_DVFS_POWER_SWI2C_ADDR_SY8824C_CPU(byCluster), 1, byTargetRegAddress, 1, byTargetData) > 0 )
           {
               DVFS_HAL_DEBUG("\033[37m[INFO] Change to Voltage: %d0 mV (0x%X)\033[m\n", (unsigned int) dwCpuPowerVoltage, (unsigned int) dwRegisterValue);
           }
           else
           {
               DVFS_HAL_DEBUG("\033[37m[ERROR] kernel I2C Write Failed\033[m\n");
           }
        }
#endif
    }
    else if (u16ControlType == 1)
    {
#if CONFIG_DVFS_CPU_POWER_GPIO_ENABLE

        if (bDvfsPowerInfoParse == TRUE)
        {
            U8 u8PowerIndex = 0, u8GPIOIndex = 0, u8Mode = 0;

            for( u8PowerIndex = 0; u8PowerIndex < control_level; u8PowerIndex++)
            {
                if( power_control[u8PowerIndex] == dwCpuPowerVoltage)
                {
                    for( u8GPIOIndex = 0; u8GPIOIndex < num_of_gpio; u8GPIOIndex++)
                    {
                        u8Mode = gpio_control[u8GPIOIndex][u8PowerIndex];
                        switch(u8Mode)
                        {
                            case DVFS_GPIO_IN:
                                MDrv_GPIO_Set_Input(gpio_num[u8GPIOIndex]);
                                break;
                            case DVFS_GPIO_OUTPUT_LOW:
                                MDrv_GPIO_Set_Low(gpio_num[u8GPIOIndex]);
                                break;
                            case DVFS_GPIO_OUTPUT_HIGH:
                                MDrv_GPIO_Set_High(gpio_num[u8GPIOIndex]);
                                break;
                            default:
                                DVFS_HAL_DEBUG("\033[37m[ERROR] GPIO Mode is not defined\033[m\n");
                                bDvfsCpuPowerAdjStatus = FALSE;
                        }
                    }
                }
            }
        }
        else
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Do not parsing dts information\033[m\n");
            bDvfsCpuPowerAdjStatus = FALSE;
        }
#endif
    }
    return bDvfsCpuPowerAdjStatus;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: SysDvfsCorePowerAdjustment
/// @brief \b Function  \b Description: Update Output Voltage Level in External Power Chip
/// @param <IN>         \b None:
/// @param <OUT>        \b None:
/// @param <RET>        \b None:
/// @param <GLOBAL>     \b None:
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsCorePowerAdjustment(U32 dwCorePowerVoltage, U8 byCluster)
{
    U32 bDvfsCorePowerAdjStatus = TRUE;
#if CONFIG_DVFS_CORE_POWER_I2C_ENABLE

#endif
    return bDvfsCorePowerAdjStatus;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: SysDvfsCpuPowerInit
/// @brief \b Function  \b Description: The Init Flow of  External Power Chip
/// @param <IN>         \b None:
/// @param <OUT>        \b None:
/// @param <RET>        \b None:
/// @param <GLOBAL>     \b None:
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsCpuPowerInit(void)
{
    U32 bDvfsCpuPowerInitStatus = TRUE;
    U16 u16ControlType = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100508 << 1));
    u16ControlType = u16ControlType & 0x03;
    if (u16ControlType == 0)
    {
    #if CONFIG_DVFS_CPU_POWER_I2C_ENABLE
        U8  byTargetRegAddress[5] =
            {
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF
            };
        U8  byTargetData[5] =
            {
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF
            };
        U8   byloop;
        U32  dwRegisterValue=0;

    #if ((CONFIG_MSTAR_DVFS_KERNEL_SWIIC == 1) && (defined(CONFIG_MSTAR_DVFS_KERNEL_SWIIC)))
        DVFS_HAL_DEBUG("[INFO] DVFS is using kernel sw i2c interface.\n");
    #endif

    #if ((CONFIG_MSTAR_DVFS_KERNEL_HWIIC == 1) && (defined(CONFIG_MSTAR_DVFS_KERNEL_HWIIC)))
        DVFS_HAL_DEBUG("[INFO] DVFS is using kernel hw i2c interface.\n");
    #endif
        for (byloop=0;byloop<1;byloop++)    // Only 1 PM IC
        {
            byTargetRegAddress[0] = 0;
            if (MDrv_DVFS_IIC_Read(CONFIG_DVFS_POWER_I2C_SET,CONFIG_DVFS_POWER_SWI2C_ADDR_SY8824C_CPU(byloop), 1, byTargetRegAddress, 1, byTargetData) > 0 )
            {
                // SY8824C
                dwPowerChipId = CONFIG_DVFS_CHIP_ID_SY8824C;

                byTargetData[0] = (((100 * CONFIG_DVFS_CPU_POWER_DEFAULT(byloop)) - 7625) / 123) | 0x80;
                byTargetRegAddress[0] = 0x00;
                if(MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET,CONFIG_DVFS_POWER_SWI2C_ADDR_SY8824C_CPU(byloop), 1, byTargetRegAddress, 1, byTargetData) < 0 )
                {
                    DVFS_HAL_DEBUG("[ERROR] I2C_Enter_I2C Failed\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }
            }
            else
            {
                if( dwPowerChipId == CONFIG_DVFS_CHIP_ID_UNKNOWN )    // Avoid Prodo bug (can't init twice) while STR
                {
                    //dwPowerChipId = CONFIG_DVFS_CHIP_ID_PRADO;
                    // Prodo
                    byTargetRegAddress[0] = 0x53;
                    byTargetRegAddress[1] = 0x45;
                    byTargetRegAddress[2] = 0x52;
                    byTargetRegAddress[3] = 0x44;
                    byTargetRegAddress[4] = 0x42;
                    if(MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 5, byTargetRegAddress) < 0  )
                    {
                        //DVFS_HAL_DEBUG("[ERROR] I2C_Enter_I2C Failed\n");
                        printk("\033[31mFunction = %s, Line = %d, [ERROR] I2C_Enter_I2C Failed\033[m\n", __PRETTY_FUNCTION__, __LINE__);
                        bDvfsCpuPowerInitStatus = FALSE;
                    }
                }

                byTargetRegAddress[0] = 0x7F;
                if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
                {
                    //DVFS_HAL_DEBUG("[ERROR] I2C_USE_CFG Failed\n");
                    printk("\033[31mFunction = %s, Line = %d, [ERROR] I2C_USE_CFG Failed\033[m\n", __PRETTY_FUNCTION__, __LINE__);
                    bDvfsCpuPowerInitStatus = FALSE;
                }

                byTargetRegAddress[0] = 0x7D;
                if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
                {
                    DVFS_HAL_DEBUG("[ERROR] I2C_OUT_NO_DELAY Failed\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }

                byTargetRegAddress[0] = 0x50;
                if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
                {
                    DVFS_HAL_DEBUG("[ERROR] I2C_AD_BYTE_EN0 Failed\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }

                byTargetRegAddress[0] = 0x55;
                if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
                {
                    DVFS_HAL_DEBUG("[ERROR] I2C_DA_BYTE_EN1 Failed\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }

                byTargetRegAddress[0] = 0x35;
                if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
                {
                    DVFS_HAL_DEBUG("[ERROR] I2C_USE Failed\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }

                byTargetRegAddress[0] = 0x10;
                byTargetRegAddress[1] = 0xc0;
                if (MDrv_DVFS_IIC_Read(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 2, byTargetRegAddress, 2, byTargetData) > 0 )
                {
                    DVFS_HAL_DEBUG("[INFO] MStar Power IC Chip ID: %x%x\n", (unsigned int) byTargetData[0], (unsigned int) byTargetData[1]);
                    dwPowerChipId = (unsigned int) byTargetData[1];
                }
                else
                {
                    DVFS_HAL_DEBUG("\033[33m[ERROR] MStar Power IC Chip ID read fail \033[m\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }
                if( dwPowerChipId == CONFIG_DVFS_CHIP_ID_PRADO )
                {
                    //Set Default CPU Voltage
                    dwRegisterValue = (CONFIG_DVFS_CPU_POWER_DEFAULT(byloop) - CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byloop));
                    byTargetRegAddress[0] = 0x10;
                    byTargetRegAddress[1] = (0x06 << 1);
                    byTargetRegAddress[2] = 0x10;
                    byTargetRegAddress[3] = dwRegisterValue;
                    if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 4, byTargetRegAddress) > 0 )
                    {
                        DVFS_HAL_DEBUG("\033[37m[INFO] Change to Voltage: %d0 mV (0x%X)\033[m\n", (unsigned int) CONFIG_DVFS_CPU_POWER_DEFAULT(byloop), (unsigned int) dwRegisterValue);
                    }
                    else
                    {
                        DVFS_HAL_DEBUG("[ERROR] kernel I2C Write Failed\n");
                    }

                    //Set OTP Level
                    byTargetRegAddress[0] = 0x10;
                    byTargetRegAddress[1] = (0x05 << 1);
                    byTargetRegAddress[2] = 0x40;
                    byTargetRegAddress[3] = 0x00;
                    if(MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET,CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 4, byTargetRegAddress) < 0 )
                    {
                        DVFS_HAL_DEBUG("\033[37m[ERROR] kernel I2C Write Failed\033[m\n");
                    }

                    //Set Password
                    byTargetRegAddress[0] = 0x10;
                    byTargetRegAddress[1] = (0x0C << 1);
                    byTargetRegAddress[2] = 0xbe;
                    byTargetRegAddress[3] = 0xaf;
                    if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 4, byTargetRegAddress) < 0 )
                    {
                        DVFS_HAL_DEBUG("\033[37m[ERROR] kernel I2C Write Failed\033[m\n");
                    }
                }
            }
        }

        *(volatile U16 *)CONFIG_REG_DVFS_CPU_POWER_STATUS = CONFIG_DVFS_DYNAMIC_POWER_ADJUST_INIT;
    #endif
    }
    else
    {
    #if CONFIG_DVFS_CPU_POWER_GPIO_ENABLE
        struct device_node *np, *child, *gchild;
        int i,j,prop;
        char node_name[16];
        const char *gpio_mode = NULL;
        u16ControlType = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100502 << 1));
        u16ControlType = u16ControlType & (0x07 << 12);
        u16ControlType = (u16ControlType >> 12);


        if (bDvfsPowerInfoParse == TRUE)
            return TRUE;

        np = of_find_node_by_name(NULL, "mstar_dvfs");
        if (np == NULL)
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get devide node: mstar_dvfs\033[m\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }

        if (u16ControlType == 1)
            child = of_find_node_by_name(np, "dvfs_package0");
        else
            child = of_find_node_by_name(np, "dvfs_package1");

        if (child == NULL)
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get devide node: dvfs_package\033[m\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }

        if (!of_property_read_u32(child, "control-number", &prop))
            num_of_gpio = prop;
        else
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: control-number\033[m\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }

        if (!of_property_read_u32(child, "power_level", &prop))
            control_level = prop;
        else
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: power_level\033[m\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }

        gpio_num = (unsigned int *)kmalloc(sizeof(unsigned int)*num_of_gpio,GFP_KERNEL);
        power_control = (unsigned int *)kmalloc(sizeof(unsigned int)*control_level,GFP_KERNEL);
        gpio_control = (unsigned int **)kmalloc(sizeof(unsigned int *)*num_of_gpio,GFP_KERNEL);

        if ( (gpio_num == NULL) || (power_control == NULL) || (gpio_control == NULL) )
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get memory\033[m\n");
            return FALSE;
        }

        if (!of_property_read_u32_array(child, "power-status", power_control,control_level)){}
        else
        {
            DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: power-status\033[m\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }

        for(i = 0;i < num_of_gpio; i++)
        {
            gpio_control[i] = (unsigned int *)kmalloc(sizeof(unsigned int)*control_level,GFP_KERNEL);
            if ( gpio_control[i] == NULL )
            {
                DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get memory\033[m\n");
                return FALSE;
            }

            snprintf(node_name, 16, "control_status%d", i);

            gchild = of_find_node_by_name(child, node_name);

            if (!of_property_read_u32(gchild, "gpio-num", &prop))
                gpio_num[i] = prop;
            else
            {
                DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: gpio-num\033[m\n");
                bDvfsCpuPowerInitStatus = FALSE;
            }

            for( j = 0; j < control_level; j++)
            {
                if (!of_property_read_string_index(gchild, "gpio-mode", j , &gpio_mode))
                {
                    if (strcmp(gpio_mode, "input") == 0)
                        gpio_control[i][j] = DVFS_GPIO_IN;
                    else if (strcmp(gpio_mode, "output_low") == 0)
                        gpio_control[i][j] = DVFS_GPIO_OUTPUT_LOW;
                    else if (strcmp(gpio_mode, "output_high") == 0)
                        gpio_control[i][j] = DVFS_GPIO_OUTPUT_HIGH;
                    else
                        gpio_control[i][j] = DVFS_GPIO_NONE;
                }
                else
                {
                    DVFS_HAL_DEBUG("\033[37m[ERROR] Fail to get property: gpio-mode\033[m\n");
                    bDvfsCpuPowerInitStatus = FALSE;
                }
            }
        }

        *(volatile U16 *)CONFIG_REG_DVFS_CPU_POWER_STATUS = CONFIG_DVFS_DYNAMIC_POWER_ADJUST_INIT;
        bDvfsPowerInfoParse = TRUE;

        //Debug Use
        /*
        printk("[DVFS] Number of GPIO = %d\n",num_of_gpio);
        printk("[DVFS] Control_Level = %d\n",control_level);

        for(i = 0; i < num_of_gpio; i++)
            printk("[DVFS] GPIO %d = %d\n",i,gpio_num[i]);
        for(j = 0; j < control_level; j++)
            printk("[DVFS] Power %d = %d\n",j,power_control[j]);

        for(i = 0; i < num_of_gpio; i++)
            for(j = 0; j < control_level; j++)
                printk("[DVFS] GPIO Control[%d][%d] = %d\n",i,j,gpio_control[i][j]);
        */
    #endif
    }

    return bDvfsCpuPowerInitStatus;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: SysDvfsCorePowerInit
/// @brief \b Function  \b Description: The Init Flow of  External Power Chip
/// @param <IN>         \b None:
/// @param <OUT>        \b None:
/// @param <RET>        \b None:
/// @param <GLOBAL>     \b None:
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsCorePowerInit(void)
{
    U32 bDvfsCorePowerInitStatus = TRUE;
#if CONFIG_DVFS_CORE_POWER_I2C_ENABLE

#endif

#if CONFIG_DVFS_CORE_POWER_GPIO_ENABLE
#error "No Support CPU Power Adjustment by GPIO in M7622 Platform"
#endif
    return bDvfsCorePowerInitStatus;
}

