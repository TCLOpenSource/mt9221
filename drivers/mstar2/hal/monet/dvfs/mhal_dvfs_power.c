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

#include <linux/platform_device.h>
#include <linux/pm.h>

//=================================================================================================
static U32 dwPowerChipId = CONFIG_DVFS_CHIP_ID_UNKNOWN;

extern void MDrv_IIC_Init(void);
extern S32 MDrv_SW_IIC_Write(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf);
extern S32 MDrv_SW_IIC_Read(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf);

//#if (CONFIG_DVFS_CPU_POWER_I2C_ENABLE | CONFIG_DVFS_CORE_POWER_I2C_ENABLE)
//extern MS_BOOL MApi_SWI2C_ReadBytes(U16 u16BusNumSlaveID, U8 u8AddrNum, U8* paddr, U16 u16size, U8* pu8data);
//extern MS_BOOL MApi_SWI2C_WriteBytes(U16 u16BusNumSlaveID, U8 u8addrcount, U8* pu8addr, U16 u16size, U8* pu8data);
//S32 MDrv_SW_IIC_Write(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf);
//S32 MDrv_SW_IIC_Read(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf);
//#endif

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: SysDvfsInit
/// @brief \b Function  \b Description: Read T-Sensor to Handle DVFS Flow
/// @param <IN>         \b None:
/// @param <OUT>        \b None:
/// @param <RET>        \b None:
/// @param <GLOBAL>     \b None:
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsPowerInit(void)
{
    U32 bDvfsPowerInitStatus = TRUE;

    if(SysDvfsCpuPowerInit() == TRUE)
    {
        SysDvfsCpuPowerAdjustment(CONFIG_DVFS_CPU_POWER_DEFAULT);
    }
    else
    {
        bDvfsPowerInitStatus = FALSE;
        return bDvfsPowerInitStatus;
    }

    if(SysDvfsCorePowerInit() == TRUE)
    {
        SysDvfsCorePowerAdjustment(CONFIG_DVFS_CORE_POWER_DEFAULT);
    }
    else
    {
        bDvfsPowerInitStatus = FALSE;
        return bDvfsPowerInitStatus;
    }

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
U32 SysDvfsCpuPowerAdjustment(U32 dwCpuPowerVoltage)
{
    U32 bDvfsCpuPowerAdjStatus = TRUE;
#if CONFIG_DVFS_CPU_POWER_I2C_ENABLE
    U32 dwRegisterValue = 0;

    U8  byTargetRegAddress[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };
    U8  byTargetData[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };

    if(dwPowerChipId == CONFIG_DVFS_CHIP_ID_PRADO)
    {
        //Set CPU Voltage
        dwRegisterValue = (dwCpuPowerVoltage - CONFIG_DVFS_CPU_POWER_SHIFT_PRADO);
        byTargetRegAddress[0] = 0x10;
        byTargetRegAddress[1] = (0x06 << 1);
        byTargetRegAddress[2] = 0x10;
        byTargetRegAddress[3] = dwRegisterValue;
        if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 4, byTargetRegAddress, 0, byTargetData) != -1)
        {
            DVFS_HAL_DEBUG("[INFO] Change to Voltage: %d mv (0x%X)\n", (unsigned int) dwCpuPowerVoltage, (unsigned int) dwRegisterValue);
        }
        else
        {
            DVFS_HAL_DEBUG("[ERROR] Software I2C Write Failed\n");
            bDvfsCpuPowerAdjStatus = FALSE;
        }
    }
#endif
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
U32 SysDvfsCorePowerAdjustment(U32 dwCorePowerVoltage)
{
    U32 bDvfsCorePowerAdjStatus = TRUE;
#if CONFIG_DVFS_CORE_POWER_I2C_ENABLE
    U32  dwRegisterValue = 0;

    U8  byTargetRegAddress[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };
    U8  byTargetData[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };

    if(dwPowerChipId == CONFIG_DVFS_CHIP_ID_PRADO)
    {
        //Set Core Voltage
        dwRegisterValue = (dwCorePowerVoltage - CONFIG_DVFS_CORE_POWER_SHIFT_PRADO);
        byTargetRegAddress[0] = 0x10;
        byTargetRegAddress[1] = (0x06 << 1);
        byTargetRegAddress[2] = 0x10;
        byTargetRegAddress[3] = dwRegisterValue;
        if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 4, byTargetRegAddress, 0, byTargetData) != -1)
        {
            DVFS_HAL_DEBUG("[INFO] Change to Voltage: %d mv (0x%X)\n", (unsigned int) dwCorePowerVoltage, (unsigned int) dwRegisterValue);
        }
        else
        {
            DVFS_HAL_DEBUG("[ERROR] Software I2C Write Failed\n");
            bDvfsCorePowerAdjStatus = FALSE;
        }
    }
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
#if CONFIG_DVFS_CPU_POWER_I2C_ENABLE
    U8  byTargetRegAddress[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };
    U8  byTargetData[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };

    byTargetRegAddress[0] = 0x53;
    byTargetRegAddress[1] = 0x45;
    byTargetRegAddress[2] = 0x52;
    byTargetRegAddress[3] = 0x44;
    byTargetRegAddress[4] = 0x42;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 5, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_Enter_I2C Failed\n");
        bDvfsCpuPowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x7F;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_USE_CFG Failed\n");
        bDvfsCpuPowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x7D;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_OUT_NO_DELAY Failed\n");
        bDvfsCpuPowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x50;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_AD_BYTE_EN0 Failed\n");
        bDvfsCpuPowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x55;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_DA_BYTE_EN1 Failed\n");
        bDvfsCpuPowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x35;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_USE Failed\n");
        bDvfsCpuPowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x10;
    byTargetRegAddress[1] = 0xc0;
    if(MDrv_SW_IIC_Read(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 2, byTargetRegAddress, 2, byTargetData) != -1)
    {
        DVFS_HAL_DEBUG("[INFO] MStar Power IC Chip ID: %x%x\n", (unsigned int) byTargetData[0], (unsigned int) byTargetData[1]);
        dwPowerChipId = (unsigned int) byTargetData[1];
    }
    else
    {
        bDvfsCpuPowerInitStatus = FALSE;
    }

    if(dwPowerChipId == CONFIG_DVFS_CHIP_ID_PRADO)
    {
        //Set OTP Level
        byTargetRegAddress[0] = 0x10;
        byTargetRegAddress[1] = (0x05 << 1);
        byTargetRegAddress[2] = 0x40;
        byTargetRegAddress[3] = 0x00;
        if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 4, byTargetRegAddress, 0, byTargetData) == -1)
        {
            DVFS_HAL_DEBUG("[ERROR] Software I2C Write Failed\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }

        //Set Password
        byTargetRegAddress[0] = 0x10;
        byTargetRegAddress[1] = (0x0C << 1);
        byTargetRegAddress[2] = 0xbe;
        byTargetRegAddress[3] = 0xaf;
        if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CPU_ID, 4, byTargetRegAddress, 0, byTargetData) == -1)
        {
            DVFS_HAL_DEBUG("[ERROR] Software I2C Write Failed\n");
            bDvfsCpuPowerInitStatus = FALSE;
        }
    }

//    SysDvfsCpuPowerAdjustment(CONFIG_DVFS_CPU_POWER_DEFAULT);

    //*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x100510 << 1)) = CONFIG_DVFS_DYNAMIC_POWER_ADJUST_INIT;
#endif
#if CONFIG_DVFS_CPU_POWER_GPIO_ENABLE
#error "No Support CPU Power Adjustment by GPIO in Monet Platform"
#endif

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
    U8  byTargetRegAddress[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };
    U8  byTargetData[5] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };

    byTargetRegAddress[0] = 0x53;
    byTargetRegAddress[1] = 0x45;
    byTargetRegAddress[2] = 0x52;
    byTargetRegAddress[3] = 0x44;
    byTargetRegAddress[4] = 0x42;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 5, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_Enter_I2C Failed\n");
        bDvfsCorePowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x7F;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_USE_CFG Failed\n");
        bDvfsCorePowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x7D;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_OUT_NO_DELAY Failed\n");
        bDvfsCorePowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x50;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_AD_BYTE_EN0 Failed\n");
        bDvfsCorePowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x55;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_DA_BYTE_EN1 Failed\n");
        bDvfsCorePowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x35;
    if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 1, byTargetRegAddress, 0, byTargetData) == -1)
    {
        DVFS_HAL_DEBUG("[ERROR] I2C_USE Failed\n");
        bDvfsCorePowerInitStatus = FALSE;
    }

    byTargetRegAddress[0] = 0x10;
    byTargetRegAddress[1] = 0xc0;
    if(MDrv_SW_IIC_Read(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 2, byTargetRegAddress, 2, byTargetData) != -1)
    {
        DVFS_HAL_DEBUG("[INFO] MStar Power IC Chip ID: %x%x\n", (unsigned int) byTargetData[0], (unsigned int) byTargetData[1]);
        dwPowerChipId = (unsigned int) byTargetData[1];
    }
    else
    {
        bDvfsCorePowerInitStatus = FALSE;
    }

    if(dwPowerChipId == CONFIG_DVFS_CHIP_ID_PRADO)
    {
        //Set OTP Level
        byTargetRegAddress[0] = 0x10;
        byTargetRegAddress[1] = (0x05 << 1);
        byTargetRegAddress[2] = 0x40;
        byTargetRegAddress[3] = 0x00;
        if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 4, byTargetRegAddress, 0, byTargetData) == -1)
        {
            DVFS_HAL_DEBUG("[ERROR] Software I2C Write Failed\n");
            bDvfsCorePowerInitStatus = FALSE;
        }

        //Set Password
        byTargetRegAddress[0] = 0x10;
        byTargetRegAddress[1] = (0x0C << 1);
        byTargetRegAddress[2] = 0xbe;
        byTargetRegAddress[3] = 0xaf;
        if(MDrv_SW_IIC_Write(CONFIG_DVFS_POWER_SWI2C_BUS, CONFIG_DVFS_POWER_SWI2C_CORE_ID, 4, byTargetRegAddress, 0, byTargetData) == -1)
        {
            DVFS_HAL_DEBUG("[ERROR] Software I2C Write Failed\n");
            bDvfsCorePowerInitStatus = FALSE;
        }
    }

//    SysDvfsCorePowerAdjustment(CONFIG_DVFS_CORE_POWER_DEFAULT);
#endif
#if CONFIG_DVFS_CORE_POWER_GPIO_ENABLE
#error "No Support CPU Power Adjustment by GPIO in Monet Platform"
#endif
    return bDvfsCorePowerInitStatus;
}

