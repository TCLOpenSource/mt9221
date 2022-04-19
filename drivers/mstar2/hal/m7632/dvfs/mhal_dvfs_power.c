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
#include <linux/of.h>
#include <linux/slab.h>

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

#include <linux/platform_device.h>
#include <linux/pm.h>

#if CONFIG_MSTAR_PWM
#include "mdrv_pwm.h"
#endif

//=================================================================================================
#if (DVFS_DEVICE_TREE_PARSE_ENABLE != 1)
static unsigned int g_u32VoltageUpper[DVFS_CLUSTER_NUM];
static unsigned int g_u32VoltageLower[DVFS_CLUSTER_NUM];
#else
#define MAX_CLUSTER_NUM 2
static unsigned int *g_u32VoltageUpper = NULL;
static unsigned int *g_u32VoltageLower = NULL;
#endif

DEFINE_MUTEX(MHalDvfsPowerInitMutex);

extern S32 MDrv_HW_IIC_WriteBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);
extern S32 MDrv_HW_IIC_ReadBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);
extern void MDrv_GPIO_Set_High(U8 u8IndexGPIO);
extern void MDrv_GPIO_Set_Low(U8 u8IndexGPIO);
extern void MDrv_GPIO_Set_Input(U8 u8IndexGPIO);


static U32 dwPowerChipId = CONFIG_DVFS_CHIP_ID_UNKNOWN;

static E_CPU_POWER_TYPE cpu_power_type = E_CPU_POWER_I2C;
static U32 bDvfsPowerInitStatus = FALSE;
static U32 bParseDone = FALSE;
static U32 bConfigByDTS = FALSE;

// PWM Control
static U32 PWM_Base = 0;
static U32 PWM_Level = 0;
static U32 PWM_Channel = 0;
static U32 PWM_Offset = 0;

// GPIO Control
static U32 GPIO_Level = 0;
static U32 *GPIO_Vol = NULL;
static U32 *GPIO_Num = NULL;
static GPIO_MAPPING *GPIO_Mapping = NULL;

////////////////////////////////////////////////////////////////////////////////
S32 MDrv_DVFS_IIC_Write(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf)
{
	S32 s32Ret = -1;
	s32Ret = MDrv_HW_IIC_WriteBytes(u8ChIIC,u8SlaveID, u8AddrCnt, pu8Addr, u32BufLen, pu8Buf);
	return(s32Ret);
}
////////////////////////////////////////////////////////////////////////////////
S32 MDrv_DVFS_IIC_Read(U8 u8ChIIC, U8 u8SlaveID, U8 u8AddrCnt, U8* pu8Addr, U32 u32BufLen, U8* pu8Buf)
{
	S32 s32Ret = -1;
	s32Ret = MDrv_HW_IIC_ReadBytes(u8ChIIC,u8SlaveID, u8AddrCnt, pu8Addr, u32BufLen, pu8Buf);
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
        dwPowerChipId = CONFIG_DVFS_CHIP_ID_UNKNOWN;
}
////////////////////////////////////////////////////////////////////////////////
U32 SysDvfsPowerInit(U8 byCluster)
{
	U32 bRet = TRUE;
    mutex_lock(&MHalDvfsPowerInitMutex);
	if (bDvfsPowerInitStatus == FALSE) {
        bDvfsPowerInitStatus = SysDvfsCpuPowerInit();
        bRet = bDvfsPowerInitStatus;
    }
	mutex_unlock(&MHalDvfsPowerInitMutex);
	return bRet;
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

	if (bDvfsPowerInitStatus == FALSE)
		SysDvfsPowerInit(byCluster);

	if (cpu_power_type == E_CPU_POWER_I2C) {
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

		if ( (g_u32VoltageLower == NULL) || (g_u32VoltageUpper == NULL) ) {
			g_u32VoltageLower = (unsigned int *)kmalloc(sizeof(unsigned int)*MAX_CLUSTER_NUM,GFP_KERNEL);
			g_u32VoltageUpper = (unsigned int *)kmalloc(sizeof(unsigned int)*MAX_CLUSTER_NUM,GFP_KERNEL);
		}
		getCpuBoundInfo(byCluster,&g_u32VoltageLower[byCluster],&g_u32VoltageUpper[byCluster]);

		if(dwCpuPowerVoltage < CONFIG_DVFS_CPU_POWER_SHIFT_PRADO(byCluster))
		{
			pr_err("\033[37m[DVFS][ERROR] dwCpuPowerVoltage is smaller than CONFIG_DVFS_CPU_POWER_SHIFT_PRADO!\033[m\n");
			return FALSE;
		}
		else if(dwCpuPowerVoltage > g_u32VoltageUpper[byCluster])
		{
			pr_err("\033[37m[DVFS][ERROR] dwCpuPowerVoltage is larger than CONFIG_DVFS_CPU_POWER_MAX!\033[m\n");
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
				pr_err("\033[37m[DVFS][ERROR] I2C Read Bytes with CONFIG_DVFS_POWER_SWI2C_ADDR_CPU Fail!\033[m\n");
			}

			if ((dwOriginalCpuPowerVoltage < g_u32VoltageLower[byCluster]) || (dwOriginalCpuPowerVoltage > g_u32VoltageUpper[byCluster]))
			{
				pr_err("[DVFS][ERROR] Original Voltage (%d)%d0 mV is invalid\n",byCluster,dwOriginalCpuPowerVoltage);
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
							pr_err("\033[37m[DVFS][ERROR] kernel I2C Write Failed\033[m\n");
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
							pr_err("\033[37m[DVFS][ERROR] kernel I2C Write Failed\033[m\n");
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
					pr_err("\033[37m[DVFS][ERROR] kernel I2C Write Failed\033[m\n");
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
			   pr_err("\033[37m[DVFS][ERROR] kernel I2C Write Failed\033[m\n");
			}
		}
	}
	else if (cpu_power_type == E_CPU_POWER_PWM)
	{
		DVFS_HAL_DEBUG("[DVFS] Cpu Power Adjustment by PWM PM IC\n");
#if CONFIG_MSTAR_PWM
		if ( (dwCpuPowerVoltage <= PWM_Base) && (dwCpuPowerVoltage >= PWM_Base - PWM_Level -1) )
		{
			if (PWM_Channel == 1)
				MDrv_PWM_DutyCycle(E_PVR_PWM_CH1, PWM_Base - dwCpuPowerVoltage);
			else
				MDrv_PWM_DutyCycle(E_PVR_PWM_CH0, PWM_Base - dwCpuPowerVoltage);
			DVFS_HAL_DEBUG("[INFO] Change to Voltage: %d mv (0x%X)\n", (unsigned int) dwCpuPowerVoltage, (unsigned int)(PWM_Base - dwCpuPowerVoltage) );
			msleep(1);
		}
		else
		{
			pr_err("[DVFS][DVFS] Cpu power is not valid for PWM PM IC(%d)\n", dwCpuPowerVoltage);
			bDvfsCpuPowerAdjStatus = FALSE;
		}
#else
		bDvfsCpuPowerAdjStatus = FALSE;
#endif
	}
	else if (cpu_power_type == E_CPU_POWER_GPIO)
	{
		DVFS_HAL_DEBUG("[DVFS] Cpu Power Adjustment by GPIO\n");
		U32 Idx = 0,Idx2 = 0;
		for(Idx = 0; Idx < GPIO_Level; Idx++) {
			if (GPIO_Vol[Idx] == dwCpuPowerVoltage) {
				for(Idx2=0; Idx2 < GPIO_Num[Idx]; Idx2++) {
					switch(GPIO_Mapping[Idx].mode[Idx2]) {
						case E_GPIO_INPUT:
							MDrv_GPIO_Set_Input(GPIO_Mapping[Idx].num[Idx2]);
							break;
						case E_GPIO_OUTPUT_LOW:
							MDrv_GPIO_Set_Low(GPIO_Mapping[Idx].num[Idx2]);
							break;
						case E_GPIO_OUTPUT_HIGH:
							MDrv_GPIO_Set_High(GPIO_Mapping[Idx].num[Idx2]);
							break;
						default:
							break;
					}
				}
			}
		}
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
	return bDvfsCorePowerAdjStatus;
}


U32 SysDvfsCpuPowerParsing(void)
{
	U32 bDvfsCpuPowerConfigByDTS = TRUE;
	U32 Idx = 0, Idx2 = 0;
	struct device_node *np, *child,*gchild,*ggchild, *parm;
	U32 prop = 0;
	const char *temp;
	char name[100];
	np = of_find_node_by_name(NULL, "soc-dvfs");
	if (np == NULL)
	{
		pr_err("\033[37m[DVFS][ERROR] Fail to get devide node: soc-dvfs\033[m\n");
		bDvfsCpuPowerConfigByDTS = FALSE;
	}

	child = of_find_node_by_name(np, "cluster_0");
	if (child == NULL)
	{
		pr_err("\033[37m[DVFS][ERROR] Fail to get devide node: cluster_0\033[m\n");
		bDvfsCpuPowerConfigByDTS = FALSE;
	}

	gchild = of_find_node_by_name(child, "table-info_0");
	if (gchild == NULL)
	{
		pr_err("\033[37m[DVFS][ERROR] Fail to get devide node: table-info_0\033[m\n");
		bDvfsCpuPowerConfigByDTS = FALSE;
	}

	ggchild = of_find_node_by_name(gchild, "power-info");
	if (ggchild == NULL)
	{
		pr_err("\033[37m[DVFS][ERROR] Fail to get devide node: power_info\033[m\n");
		bDvfsCpuPowerConfigByDTS = FALSE;
	}

	if (bDvfsCpuPowerConfigByDTS == TRUE) {

		if ( !of_property_read_string(ggchild, "type", &temp) )
			pr_emerg("[DVFS] CPU Power: %s\n", temp);
		else {
			pr_err("\033[37m[DVFS][ERROR] Fail to get property: source\033[m\n");
			bDvfsCpuPowerConfigByDTS = FALSE;
		}

		parm = of_find_node_by_name(ggchild, "parm");
		if (parm == NULL) {
			pr_err("\033[37m[DVFS][ERROR] Fail to get devide node: parm\033[m\n");
			bDvfsCpuPowerConfigByDTS = FALSE;
		}

		if ( strcmp(temp, "i2c") == 0 )
			bDvfsCpuPowerConfigByDTS = FALSE;
	}

	if (bDvfsCpuPowerConfigByDTS == TRUE) {
		if (strcmp(temp, "pwm") == 0) {
			/*
			power-info {
				type = "pwm";
				parm {
					channel     = <0>;
					power-base  = <129>;
					power-level = <48>;
					offset      = <0>;
				};
			};
			*/
			if (!of_property_read_u32(parm, "channel", &prop))
				PWM_Channel = prop;
			else {
				pr_err("\033[37m[DVFS][ERROR] Fail to get property: channel\033[m\n");
				bDvfsCpuPowerConfigByDTS = FALSE;
			}

			if (!of_property_read_u32(parm, "power-base", &prop))
				PWM_Base = prop;
			else {
				pr_err("\033[37m[DVFS][ERROR] Fail to get property: power-base\033[m\n");
				bDvfsCpuPowerConfigByDTS = FALSE;
			}

			if (!of_property_read_u32(parm, "power-level", &prop))
				PWM_Level = prop;
			else {
				pr_err("\033[37m[DVFS][ERROR] Fail to get property: power-level\033[m\n");
				bDvfsCpuPowerConfigByDTS = FALSE;
			}

			if (!of_property_read_u32(parm, "offset", &prop))
				PWM_Offset = prop;
			else {
				pr_err("\033[37m[DVFS][ERROR] Fail to get property: offset\033[m\n");
				bDvfsCpuPowerConfigByDTS = FALSE;
			}

			if (bDvfsCpuPowerConfigByDTS == TRUE) {
				pr_emerg("\033[32m [DVFS] channel = %d, base = %d, level = %d, offset = %d \033[0m\n", PWM_Channel,PWM_Base,PWM_Level,PWM_Offset);
				cpu_power_type = E_CPU_POWER_PWM;
			}
		}
		else if (strcmp(temp, "gpio") == 0) {
			/*
			power-info {
				type = "gpio";
				parm {
					level = <3>;
					control_0 {
						vol = <90>;
						num = <2>;
						gpio_num =  <51 52>;
						gpio_mode = <0 0>;
					};
					control_1 {
						vol = <95>;
						num = <2>;
						gpio_num =  <51 52>;
						gpio_mode = <0 1>;
					};
					control_2 {
						vol = <100>;
						num = <2>;
						gpio_num =  <51 52>;
						gpio_mode = <1 1>;
					};
				};
			}
			*/
			if (!of_property_read_u32(parm, "level", &prop))
				GPIO_Level = prop;
			else {
					pr_err("\033[37m[DVFS][ERROR] Fail to get property: level\033[m\n");
					bDvfsCpuPowerConfigByDTS = FALSE;
			}

			GPIO_Vol = (U32 *)kmalloc(sizeof(U32)*GPIO_Level,GFP_KERNEL);
			GPIO_Num = (U32 *)kmalloc(sizeof(U32)*GPIO_Level,GFP_KERNEL);
			GPIO_Mapping = (GPIO_MAPPING *)kmalloc(sizeof(GPIO_MAPPING)*GPIO_Level,GFP_KERNEL);

			for(Idx = 0; Idx < GPIO_Level; Idx++) {
				memset(name,0,100);
				snprintf(name, 10, "control_%d", Idx);
				ggchild = of_find_node_by_name(parm, name);
				if (ggchild == NULL)
				{
					pr_err("\033[37m[DVFS][ERROR] Fail to get devide node: %s\033[m\n", name);
					bDvfsCpuPowerConfigByDTS = FALSE;
				}

				if (!of_property_read_u32(ggchild, "vol", &prop))
					GPIO_Vol[Idx] = prop;
				else {
					pr_err("\033[37m[DVFS][ERROR] Fail to get property: vol\033[m\n");
					bDvfsCpuPowerConfigByDTS = FALSE;
				}

				if (!of_property_read_u32(ggchild, "num", &prop))
					GPIO_Num[Idx] = prop;
				else {
					pr_err("\033[37m[DVFS][ERROR] Fail to get property: num\033[m\n");
					bDvfsCpuPowerConfigByDTS = FALSE;
				}

				GPIO_Mapping[Idx].num = (U32 *)kmalloc(sizeof(U32)*GPIO_Num[Idx], GFP_KERNEL);
				GPIO_Mapping[Idx].mode = (E_GPIO_MODE *)kmalloc(sizeof(E_GPIO_MODE)*GPIO_Num[Idx], GFP_KERNEL);

				if ( of_property_read_u32_array(ggchild , "gpio_num",GPIO_Mapping[Idx].num ,GPIO_Num[Idx]) ) {
					pr_err("\033[37m[DVFS][ERROR] Fail to get property: gpio_num\033[m\n");
					bDvfsCpuPowerConfigByDTS = FALSE;
				}

				if ( of_property_read_u32_array(ggchild , "gpio_mode",GPIO_Mapping[Idx].mode ,GPIO_Num[Idx]) ) {
					pr_err("\033[37m[DVFS][ERROR] Fail to get property: gpio_mode\033[m\n");
					bDvfsCpuPowerConfigByDTS = FALSE;
				}
			}

			if (bDvfsCpuPowerConfigByDTS == TRUE) {
				pr_emerg("\033[32m [DVFS] GPIO_Level = %d \033[0m\n",GPIO_Level);
				for (Idx = 0; Idx < GPIO_Level; Idx++) {
					pr_emerg("\033[32m [DVFS] GPIO_Vol = %d \033[0m\n", GPIO_Vol[Idx]);
					pr_emerg("\033[32m [DVFS] GPIO_Num = %d \033[0m\n", GPIO_Num[Idx]);
					for (Idx2 = 0; Idx2 < GPIO_Num[Idx]; Idx2++) {
						pr_emerg("\033[32m [DVFS] num = %d \033[0m\n", GPIO_Mapping[Idx].num[Idx2]);
						pr_emerg("\033[32m [DVFS] mode = %d \033[0m\n", GPIO_Mapping[Idx].mode[Idx2]);
					}
				}
				cpu_power_type = E_CPU_POWER_GPIO;
			}
		}
	}
	return bDvfsCpuPowerConfigByDTS;
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
	U32 dwRegisterValue = 0;
	U16 u16Temp = 0;
	U8  byTargetRegAddress[5] =
		{
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF
		};
	U8  byTargetData[5] =
		{
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF
		};
	U8   byloop;
#if CONFIG_MSTAR_PWM
	PWM_ChNum num = E_PVR_PWM_CH1;
#endif
	if (bParseDone == FALSE) {
		bConfigByDTS = SysDvfsCpuPowerParsing();
		bParseDone = TRUE;
	}

	if ( (cpu_power_type == E_CPU_POWER_PWM) && (bConfigByDTS == TRUE) ) {
#if CONFIG_MSTAR_PWM
		if (PWM_Channel == 1)
			num = E_PVR_PWM_CH1;
		else
			num = E_PVR_PWM_CH0;

		MDrv_PWM_Period(num, 0x2F);
		MDrv_PWM_DutyCycle(num, 0x1D);
		MDrv_PWM_Shift(num, PWM_Offset);
		MDrv_PWM_AutoCorrect(num, TRUE);

		u16Temp = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x322992 << 1));
		*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x322992 << 1)) = u16Temp | BIT4;       // reg_pwm_dac1_mode
		u16Temp = *(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x3229D0 << 1));
		*(volatile U16 *)(CONFIG_REGISTER_BASE_ADDRESS + (0x3229D0 << 1)) = u16Temp & ~(BIT9);    // reg_pwm_dac_oen
#else
		bDvfsCpuPowerInitStatus = FALSE;
#endif
	}
	else if ( (cpu_power_type == E_CPU_POWER_GPIO) && (bConfigByDTS == TRUE) ) {
		bDvfsCpuPowerInitStatus = TRUE;
	}
	else {
		pr_debug("\033[32m [DVFS] CPU Power Control By I2C \033[0m\n");

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
				if( dwPowerChipId == CONFIG_DVFS_CHIP_ID_UNKNOWN )
				{
					// Prodo
					byTargetRegAddress[0] = 0x53;
					byTargetRegAddress[1] = 0x45;
					byTargetRegAddress[2] = 0x52;
					byTargetRegAddress[3] = 0x44;
					byTargetRegAddress[4] = 0x42;
					if(MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 5, byTargetRegAddress) < 0  )
					{
						pr_err("\033[31mFunction = %s, Line = %d, [ERROR] I2C_Enter_I2C Failed\033[m\n", __PRETTY_FUNCTION__, __LINE__);
						bDvfsCpuPowerInitStatus = FALSE;
					}
				}

				byTargetRegAddress[0] = 0x7F;
				if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
				{
					pr_err("\033[31mFunction = %s, Line = %d, [ERROR] I2C_USE_CFG Failed\033[m\n", __PRETTY_FUNCTION__, __LINE__);
					bDvfsCpuPowerInitStatus = FALSE;
				}

				byTargetRegAddress[0] = 0x7D;
				if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
				{
					pr_err("[DVFS][ERROR] I2C_OUT_NO_DELAY Failed\n");
					bDvfsCpuPowerInitStatus = FALSE;
				}

				byTargetRegAddress[0] = 0x50;
				if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
				{
					pr_err("[DVFS][ERROR] I2C_AD_BYTE_EN0 Failed\n");
					bDvfsCpuPowerInitStatus = FALSE;
				}

				byTargetRegAddress[0] = 0x55;
				if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
				{
					pr_err("[DVFS][ERROR] I2C_DA_BYTE_EN1 Failed\n");
					bDvfsCpuPowerInitStatus = FALSE;
				}

				byTargetRegAddress[0] = 0x35;
				if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 1, byTargetRegAddress) < 0 )
				{
					pr_err("[DVFS][ERROR] I2C_USE Failed\n");
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
					pr_err("\033[33m[DVFS][ERROR] MStar Power IC Chip ID read fail \033[m\n");
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
						pr_err("[DVFS][ERROR] kernel I2C Write Failed\n");
					}

					//Set OTP Level
					byTargetRegAddress[0] = 0x10;
					byTargetRegAddress[1] = (0x05 << 1);
					byTargetRegAddress[2] = 0x40;
					byTargetRegAddress[3] = 0x00;
					if(MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET,CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 4, byTargetRegAddress) < 0 )
					{
						pr_err("\033[37m[DVFS][ERROR] kernel I2C Write Failed\033[m\n");
					}

					//Set Password
					byTargetRegAddress[0] = 0x10;
					byTargetRegAddress[1] = (0x0C << 1);
					byTargetRegAddress[2] = 0xbe;
					byTargetRegAddress[3] = 0xaf;
					if (MDrv_DVFS_IIC_Write(CONFIG_DVFS_POWER_I2C_SET, CONFIG_DVFS_POWER_SWI2C_ADDR_CPU(byloop), 0, byTargetRegAddress, 4, byTargetRegAddress) < 0 )
					{
						pr_err("\033[37m[DVFS][ERROR] kernel I2C Write Failed\033[m\n");
					}
				}
			}
		}
		if (bDvfsCpuPowerInitStatus == TRUE)
			cpu_power_type = E_CPU_POWER_I2C;
	}

	*(volatile U16 *)CONFIG_REG_DVFS_CPU_POWER_STATUS = CONFIG_DVFS_DYNAMIC_POWER_ADJUST_INIT;
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
	return bDvfsCorePowerInitStatus;
}

