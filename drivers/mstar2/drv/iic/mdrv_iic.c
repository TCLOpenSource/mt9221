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

#define _API_SWI2C_C_

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/time.h>

#include "../gpio/mdrv_gpio.h"
#include "mdrv_iic.h"
#include "mdrv_sem.h"
#include "mhal_iic.h"
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define __I2C_BUS(scl, sda, dly)    scl, sda, dly
#define I2C_BUS( bus )			  __I2C_BUS( bus )
#define COUNTOF( array )			(sizeof(array) / sizeof((array)[0]))

#define I2C_CHECK_PIN_DUMMY	  5000
#define PULL_HIGH				   1
#define PULL_LOW					0
#define SWI2C_READ			  0
#define SWI2C_WRITE			 1
#define I2C_ACKNOWLEDGE		 PULL_LOW
#define I2C_NON_ACKNOWLEDGE	 PULL_HIGH
#define I2C_ACCESS_DUMMY_TIME   7

#define HIBYTE(value)  ((u8)((value) / 0x100))
#define LOBYTE(value)  ((u8)(value))

struct mutex gBusLocks[IIC_BUS_MAX];
struct mutex gAdapterLock;

#define IIC_WR_RETRY_TIMES         3
struct mutex gPortLocks[HWI2C_PORTS];
static HWI2C_PortCfg gp_HWI2CinitCfg[HWI2C_PORTS];

EXPORT_SYMBOL(gPortLocks);

//#define LOCK_HW_SEM()   {\
							MDrv_SEM_Lock(E_SEM_IIC, SEM_WAIT_FOREVER); \
					  }
//#define UNLOCK_HW_SEM() {\
							MDrv_SEM_Unlock(E_SEM_IIC); \
					  }
//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static SWI2C_DbgLvl _gSWI2CDbgLevel = E_SWI2C_DBGLVL_WARNING;

static IIC_BusCfg_t g_IICBus[IIC_BUS_MAX];
static u8 u8BusAllocNum = 0;
static u8 u8BusSel = 0;
static SWI2C_ReadMode g_I2CReadMode[IIC_BUS_MAX];

static u32 u32DelayCount[IIC_BUS_MAX];
static u32 u32CpuSpeedMHz;
static u8 g_u8CfgBusNum = 0;

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------
#define IIC_DBG_ENABLE           0

#if IIC_DBG_ENABLE
#define IIC_DBG(_f)             (_f)
#else
#define IIC_DBG(_f)
#endif

#if 0
#define LINE_DBG()             printf("IIC %d\n", __LINE__)
#else
#define LINE_DBG()
#endif

#define IIC_PRINT(fmt, args...)    //printk("[IIC][%05d] " fmt, __LINE__, ## args)

#define SWI2C_DBG_FUNC()               if (_gSWI2CDbgLevel >= E_SWI2C_DBGLVL_ALL) \
                                        {printk(KERN_INFO "\t====   %s   ====\n", __FUNCTION__);}
#define SWI2C_DBG_INFO(x, args...)     if (_gSWI2CDbgLevel >= E_SWI2C_DBGLVL_INFO ) \
                                        {printk(KERN_INFO "[%s]: ", __FUNCTION__); printk(KERN_INFO x, ##args);}
#define SWI2C_DBG_ERR(x, args...)      if (_gSWI2CDbgLevel >= E_SWI2C_DBGLVL_ERROR) \
                                        {printk(KERN_ERR "[%s]: ", __FUNCTION__); printk( KERN_ERR x, ##args);}
#define SWI2C_DBG_WARN(x, args...)     if (_gSWI2CDbgLevel >= E_SWI2C_DBGLVL_WARNING) \
                                        {printk(KERN_DBG "[%s]: ", __FUNCTION__); printk(KERN_DBG x, ##args);}

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

static long long timespec_diff_us(struct timespec start, struct timespec end)
{
    struct timespec temp;
    if ((end.tv_nsec - start.tv_nsec) < 0) {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return ((long long)temp.tv_sec * 1000000) + (temp.tv_nsec / 1000);
}

//-------------------------------------------------------------------------------------------------
/// Delay for u32Us microseconds
/// @param  u32Us  \b IN: delay 0 ~ 999 us
/// @return None
/// @note   implemented by "busy waiting". Plz call DelayTask directly for ms-order delay
//-------------------------------------------------------------------------------------------------
static void DelayTaskUs (u32 u32Us)
{
    struct timespec TS1, TS2;
    getrawmonotonic(&TS1);
    do
    {
        getrawmonotonic(&TS2);
    }while(timespec_diff_us(TS1,TS2)< u32Us);
}

//-----------------------------------------------------------------------------------------------
/// Delay Poll for u32Us microseconds
/// @param  u32Us  \b IN: delay 0 ~ 999 us
/// @return None
/// @note   implemented by "busy waiting". Plz call DelayTask directly for ms-order delay
//------------------------------------------------------------------------------------------------
static void DelayTaskUs_Poll(u32 u32Us)
{
    DelayTaskUs(u32Us);
}

static void iic_delay(u8 u8BusNum)
{
	u32 volatile u32Loop=u32DelayCount[u8BusNum];//henry.wu why volatile?

	while(u32Loop--)
	{
		#ifdef __mips__
		__asm__ __volatile__ ("nop");
		#endif

		#ifdef __AEONR2__
		__asm__ __volatile__ ("l.nop");
		#endif

		#ifdef __arm__
		__asm__ __volatile__ ("mov r0, r0");
		#endif
	}
}

static inline void pin_scl_set_input(u8 u8BusNum)
{
	//MDrv_GPIO_Set_Input( g_IICBus[u8BusNum].u16PadSCL );
	gpio_direction_input(g_IICBus[u8BusNum].u16PadSCL);
}

static inline void pin_scl_set_high(u8 u8BusNum)
{
	//MDrv_GPIO_Set_High( g_IICBus[u8BusNum].u16PadSCL );
	gpio_direction_input(g_IICBus[u8BusNum].u16PadSCL);
}

static inline void pin_scl_set_low(u8 u8BusNum)
{
	//MDrv_GPIO_Set_Low( g_IICBus[u8BusNum].u16PadSCL );
	gpio_set_value(g_IICBus[u8BusNum].u16PadSCL, 0);
}

static int pin_scl_get_level(u8 u8BusNum)
{
	//return MDrv_GPIO_Pad_Read( g_IICBus[u8BusNum].u16PadSCL ) ? PULL_HIGH : PULL_LOW;
	return gpio_get_value(g_IICBus[u8BusNum].u16PadSCL) ? PULL_HIGH : PULL_LOW;
}

static void pin_scl_check_high(u8 u8BusNum)
{
	u16 u16Retries = g_IICBus[u8BusNum].u16Retries;

	pin_scl_set_high(u8BusNum);

	while (u16Retries--)
	{
		if(pin_scl_get_level(u8BusNum) == PULL_HIGH)
			break;
	}
}

static void pin_sda_set_input(u8 u8BusNum)
{
	//MDrv_GPIO_Set_Input( g_IICBus[u8BusNum].u16PadSDA );
	gpio_direction_input(g_IICBus[u8BusNum].u16PadSDA);
}

static void pin_sda_set_high(u8 u8BusNum)
{
	//MDrv_GPIO_Set_Input( g_IICBus[u8BusNum].u16PadSDA );
	gpio_direction_input(g_IICBus[u8BusNum].u16PadSDA);
}

static void pin_sda_set_low(u8 u8BusNum)
{
	//MDrv_GPIO_Set_Low( g_IICBus[u8BusNum].u16PadSDA );
	gpio_set_value(g_IICBus[u8BusNum].u16PadSDA, 0);
}

static int pin_sda_get_level(u8 u8BusNum)
{
	//return MDrv_GPIO_Pad_Read( g_IICBus[u8BusNum].u16PadSDA ) ? PULL_HIGH : PULL_LOW;
	return gpio_get_value(g_IICBus[u8BusNum].u16PadSDA) ? PULL_HIGH : PULL_LOW;
}

static bool pin_sda_check_high(u8 u8BusNum)
{
    u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
    bool bRet = TRUE;

    pin_sda_set_high(u8BusNum);
    while (u16Retries--)
    {
        if(pin_sda_get_level(u8BusNum) == PULL_HIGH)
            break;
    }
    return bRet;
}

static bool pin_scl_check_low(u8 u8BusNum)
{
    u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
    bool bRet = TRUE;

    pin_scl_set_low(u8BusNum);
    while (u16Retries--)
    {
        if(pin_scl_get_level(u8BusNum) == PULL_LOW)
            break;
    }
    return bRet;
}

/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
// I2C start signal.
// <comment>
//  SCL ________
//              \_________
//  SDA _____
//           \____________
//
// Return value: None
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
static bool IIC_Start(u8 u8BusNum)
{
	bool bStatus = TRUE;    // success status
	u16 u16Dummy = I2C_CHECK_PIN_DUMMY;

	pin_sda_check_high(u8BusNum);
	iic_delay(u8BusNum);

	pin_scl_check_high(u8BusNum);
	iic_delay(u8BusNum);

	// check pin error
	pin_scl_set_input(u8BusNum);
	pin_sda_set_input(u8BusNum);

	if ((pin_scl_get_level(u8BusNum) == PULL_LOW) || (pin_sda_get_level(u8BusNum) == PULL_LOW))
	{
		pin_scl_set_high(u8BusNum);
		pin_sda_set_high(u8BusNum);
		bStatus = FALSE;
	}
	else // success
	{
		pin_sda_set_low(u8BusNum);
		while (u16Dummy--)
		{
			if(pin_sda_get_level(u8BusNum) == PULL_LOW)
				break;
			if (u16Dummy==0)
			{
				return FALSE;
			}
		}
	DelayTaskUs_Poll(5);
		pin_scl_set_low(u8BusNum);
	}

	return bStatus;     //vain
}

////////////////////////////////////////////////////////////////////////////////
// I2C stop signal.
// <comment>
//              ____________
//  SCL _______/
//                 _________
//  SDA __________/
////////////////////////////////////////////////////////////////////////////////
static void IIC_Stop(u8 u8BusNum)
{
	u16 u16Dummy = I2C_CHECK_PIN_DUMMY;
	pin_scl_set_low(u8BusNum);
	iic_delay(u8BusNum);
	pin_sda_set_low(u8BusNum);

	iic_delay(u8BusNum);
	pin_scl_set_input(u8BusNum);
	while (u16Dummy--)
	{
		if(pin_scl_get_level(u8BusNum) == PULL_HIGH)
			break;
	}
	DelayTaskUs_Poll(5);
	pin_sda_set_input(u8BusNum);
	iic_delay(u8BusNum);
}

/******************************************************************************/
///Send 1 bytes data
///@param u8dat \b IN: 1 byte data to send
/******************************************************************************/
static bool SendByte(u8 u8BusNum, u8 u8dat)   // Be used int IIC_SendByte
{
	u8	u8Mask = 0x80;
	int bAck; // acknowledge bit

	while ( u8Mask )
	{
		if (u8dat & u8Mask)
		{
			pin_sda_check_high(u8BusNum);
		}
		else
		{
			pin_sda_set_low(u8BusNum);
		}

		iic_delay(u8BusNum);
		pin_scl_check_high(u8BusNum);
		iic_delay(u8BusNum);
		pin_scl_set_low(u8BusNum);

		u8Mask >>= 1; // next
	}

	// recieve acknowledge
	pin_sda_set_input(u8BusNum);
	iic_delay(u8BusNum);
	pin_scl_check_high(u8BusNum);

	iic_delay(u8BusNum);
	bAck = pin_sda_get_level(u8BusNum); // recieve acknowlege
	pin_scl_set_low(u8BusNum);

	iic_delay(u8BusNum);

	//for I2c waveform sharp
	if (bAck)
	pin_sda_set_high(u8BusNum);
	else
		pin_sda_set_low(u8BusNum);

	pin_sda_set_input(u8BusNum);

	iic_delay(u8BusNum);
	iic_delay(u8BusNum);
	iic_delay(u8BusNum);
	iic_delay(u8BusNum);

	return (bAck)? TRUE: FALSE;
}

/******************************************************************************/
///Send 1 bytes data, this function will retry 5 times until success.
///@param u8dat \b IN: 1 byte data to send
///@return bool:
///- TRUE: Success
///- FALSE: Fail
/******************************************************************************/
static bool IIC_SendByte(u8 u8BusNum, u8 u8dat)
{
    u8 i;
    //SWI2C_DBG_INFO("IIC[%d] writing byte 0x%x\n", u8BusNum, u8dat);
    if (SendByte(u8BusNum, u8dat) == I2C_ACKNOWLEDGE)
        return TRUE;

    SWI2C_DBG_INFO("IIC[%d] write byte 0x%x fail!!\n", u8BusNum, u8dat);
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
// I2C access start.
//
// Arguments: u8SlaveID - Slave ID (Address)
//            trans_t - I2C_TRANS_WRITE/I2C_TRANS_READ
////////////////////////////////////////////////////////////////////////////////
static bool IIC_AccessStart(u8 u8BusNum, u8 u8SlaveID, u8 trans_t)
{
	u16 u16Retries;

	SWI2C_DBG_FUNC();

	if (trans_t == SWI2C_READ) // check i2c read or write
	{
		u8SlaveID |= BIT0;
	}
	else
	{
		u8SlaveID &= ~BIT0;
	}

	u16Retries = 1;
	while (u16Retries--)
	{
		if ( IIC_Start(u8BusNum) == FALSE)
		{
			continue;
		}

		if ( IIC_SendByte(u8BusNum, u8SlaveID) == TRUE )  // check acknowledge
		{
			return TRUE;
		}

		IIC_Stop(u8BusNum);
	}
	return FALSE;
}
/******************************************************************************/
///Get 1 bytes data, this function will retry 5 times until success.
///@param *u8dat \b IN: pointer to 1 byte data buffer for getting data
///@return bool:
///- TRUE: Success
///- FALSE: Fail
/******************************************************************************///
//static bool IIC_GetByte(u8* pu8data)    // Auto generate ACK
static u8 IIC_GetByte (u8 u8BusNum, u16 bAck)
{
	u8 ucReceive = 0;
	u8 ucMask = 0x80;

	pin_sda_set_input(u8BusNum);

	while ( ucMask )
	{
		iic_delay(u8BusNum);
		pin_scl_check_high(u8BusNum);
		iic_delay(u8BusNum);

		if (pin_sda_get_level(u8BusNum) == PULL_HIGH)
		{
			ucReceive |= ucMask;
		}
		ucMask >>= 1; // next

		pin_scl_set_low(u8BusNum);
		pin_scl_check_low(u8BusNum);
	}
	if (bAck)
	{
		pin_sda_set_low(u8BusNum);     // acknowledge
	}
	else
	{
		pin_sda_check_high(u8BusNum);  // non-acknowledge
	}
	iic_delay(u8BusNum);
	pin_scl_check_high(u8BusNum);
	iic_delay(u8BusNum);
	pin_scl_set_low(u8BusNum);
	iic_delay(u8BusNum);
	iic_delay(u8BusNum);
	iic_delay(u8BusNum);
	return ucReceive;
}

static bool IIC_CfgSpeedParam(u8 u8BusNum, u32 u32Speed_K)
{
#define DELAY_CNT(SpeedKHz)  ((u32FactorDelay/(SpeedKHz))-((u32Parameter1+u32AdjParam)-((SpeedKHz)/u32AdjParam))+((1<<((u32Parameter2-SpeedKHz)/40))))

	u32 u32FactorDelay = 50400UL;
	u32 u32FactorAdjust = 11040UL;
	u32 u32ParamBase1 = 130UL;
	u32 u32Parameter1 = 130UL;
	u32 u32Parameter2 = 440UL;
	u32 u32AdjParam;

	//(1) assign primary parameters
	u32FactorDelay = u32CpuSpeedMHz * 100;
	u32FactorAdjust = (u32CpuSpeedMHz>=312) ? 10000UL :13000UL;
	if (u32CpuSpeedMHz > 0)
	{
    	u32AdjParam = u32FactorAdjust/u32CpuSpeedMHz;
	}
	else
	{
		SWI2C_DBG_ERR("%s, Error parameter u32CpuSpeedMHz=%d",__FUNCTION__, u32CpuSpeedMHz);
		return FALSE;
	}
    u32Parameter2 = 440UL;
    //(2) assign base for parameter 1
    if(u32CpuSpeedMHz>=1000) u32ParamBase1 = 150UL;
    else if(u32CpuSpeedMHz>=900) u32ParamBase1 = 140UL;
    else if(u32CpuSpeedMHz>=780) u32ParamBase1 = 135UL;
    else if(u32CpuSpeedMHz>=720) u32ParamBase1 = 130UL;
    else if(u32CpuSpeedMHz>=650) u32ParamBase1 = 125UL;
    else if(u32CpuSpeedMHz>=600) u32ParamBase1 = 110UL;
    else if(u32CpuSpeedMHz>=560) u32ParamBase1 = 100UL;
    else if(u32CpuSpeedMHz>=530) u32ParamBase1 = 95UL;
    else if(u32CpuSpeedMHz>=500) u32ParamBase1 = 90UL;
    else if(u32CpuSpeedMHz>=480) u32ParamBase1 = 85UL;
    else if(u32CpuSpeedMHz>=430) u32ParamBase1 = 80UL;
    else if(u32CpuSpeedMHz>=400) u32ParamBase1 = 75UL;
    else if(u32CpuSpeedMHz>=384) u32ParamBase1 = 70UL;
    else if(u32CpuSpeedMHz>=360) u32ParamBase1 = 65UL;
    else if(u32CpuSpeedMHz>=336) u32ParamBase1 = 60UL;
    else if(u32CpuSpeedMHz>=312) u32ParamBase1 = 40UL;
    else if(u32CpuSpeedMHz>=240) u32ParamBase1 = 10UL;
    else if(u32CpuSpeedMHz>=216) u32ParamBase1 = 0UL;
    else u32ParamBase1 = 0UL;
    //(3) compute parameter 1 by base
    if(u32Speed_K>=350) u32Parameter1 = u32ParamBase1;  //400K level
    else if(u32Speed_K>=250) u32Parameter1 = u32ParamBase1 + 10; //300K evel
    else if(u32Speed_K>=150) u32Parameter1 = u32ParamBase1 + 60; //200K level
    else if(u32Speed_K>=75) u32Parameter1 = u32ParamBase1 + 250; //100K level //160
    else u32Parameter1 = u32ParamBase1 + 560; //50K level
    //(4) compute delay counts
    if ((u32Speed_K>0) && (u32AdjParam>0))
    {
    	u32DelayCount[u8BusNum] = DELAY_CNT(u32Speed_K);
    }
	else
	{
		SWI2C_DBG_ERR("%s, Error parameter u32Speed_K=%d , u32AdjParam=%d",__FUNCTION__,  u32Speed_K, u32AdjParam);
		return FALSE;
	}

	return TRUE;
}

/******************************************************************************/
///I2C Initialize: set I2C Clock and enable I2C
/******************************************************************************/
static bool IIC_ConfigBus(u8 u8BusNum, IIC_BusCfg_t* pSWI2CBusCfg)
{
	SWI2C_DBG_FUNC();

	//check resources
	if ((u8BusAllocNum > IIC_BUS_MAX) || (u8BusNum >= IIC_BUS_MAX))
		return FALSE;
	if (!pSWI2CBusCfg)
		return FALSE;

	//config IIC bus settings
	g_IICBus[u8BusNum].u16PadSCL = pSWI2CBusCfg->u16PadSCL;
	g_IICBus[u8BusNum].u16PadSDA = pSWI2CBusCfg->u16PadSDA;
	g_IICBus[u8BusNum].u16Retries = pSWI2CBusCfg->u16Retries;
	g_IICBus[u8BusNum].u16DefDelay = pSWI2CBusCfg->u16DefDelay;
	u8BusAllocNum++;

	SWI2C_DBG_INFO("[IIC_ConfigBus]: g_I2CBus[%d].padSCL   = %d\n", u8BusNum, g_IICBus[u8BusNum].u16PadSCL);
	SWI2C_DBG_INFO("[IIC_ConfigBus]: g_I2CBus[%d].padSDA   = %d\n", u8BusNum, g_IICBus[u8BusNum].u16PadSDA);
	SWI2C_DBG_INFO("[IIC_ConfigBus]: g_I2CBus[%d].defDelay = %d\n", u8BusNum, g_IICBus[u8BusNum].u16DefDelay);
	SWI2C_DBG_INFO("[IIC_ConfigBus]: g_I2CBus[%d].retries = %d\n", u8BusNum, g_IICBus[u8BusNum].u16Retries);
	SWI2C_DBG_INFO("[IIC_ConfigBus]: u8BusAllocNum = %d\n", u8BusAllocNum);
	return TRUE;
}

static bool  IIC_UseBus( u8 u8BusNum )
{
	bool bReturn = FALSE;
	if ( u8BusNum < COUNTOF( g_IICBus ) )
	{
		u8BusSel = u8BusNum;
		bReturn = IIC_CfgSpeedParam(u8BusNum, g_IICBus[u8BusNum].u16DefDelay);
		//SWI2C_DBG_INFO("[IIC_UseBus]: u8BusSel = %d delay= %d\n", u8BusSel, u32DelayCount[u8BusNum]);
	}
	return bReturn;
}

static u32 MDrv_SW_IIC_Speed_Setting(u8 u8BusNum, u32 u32Speed_K)
{
	u32 u32OriginalValue;

	LOCK_HW_SEM();
	IIC_MUTEX_LOCK(u8BusNum);
	SWI2C_DBG_FUNC();

	u32OriginalValue = g_IICBus[u8BusNum].u16DefDelay;
	g_IICBus[u8BusNum].u16DefDelay = u32Speed_K;
	if (IIC_UseBus(u8BusNum) == FALSE)
	{
		g_IICBus[u8BusNum].u16DefDelay = (u8) u32OriginalValue;
	}
	SWI2C_DBG_INFO("[MDrv_SW_IIC_Speed_Setting]: u8BusNum = %d, u32Speed_K = %d\n", u8BusNum,u32Speed_K);
	IIC_MUTEX_UNLOCK(u8BusNum);
	UNLOCK_HW_SEM();
	return u32OriginalValue;
}

//-------------------------------------------------------------------------------------------------
/// IIC Set Speed by Index
/// @param  u8ChIIC            \b IN:  channel index
/// @param  u8Speed            \b IN:  u8Speed index
/// @return None
///@brief obsoleted, reserved for compatibility
//-------------------------------------------------------------------------------------------------
void MDrv_SW_IIC_SetSpeed(U8 u8ChIIC, U8 u8Speed)
{
    if( (u8ChIIC <= IIC_NUM_OF_HW) || (u8ChIIC >= IIC_NUM_OF_MAX) )
    {
        return;
        //SWII_DELAY(u8ChIIC) = u8Speed;
    }

    //switch(SWII_DELAY(u8ChIIC))
    switch(u8Speed)
    {
        case 1:
            g_IICBus[u8ChIIC].u16SpeedKHz = 400; //KHz
            break;
        case 2:
            g_IICBus[u8ChIIC].u16SpeedKHz = 300; //KHz
            break;
        case 3:
            g_IICBus[u8ChIIC].u16SpeedKHz = 200; //KHz
            break;
        case 4:
            g_IICBus[u8ChIIC].u16SpeedKHz = 100; //KHz
            break;
        default:
            g_IICBus[u8ChIIC].u16SpeedKHz = 100; //KHz
            break;
    }

}

static bool MDrv_SW_IIC_SetBusReadMode(u8 u8BusNum, SWI2C_ReadMode eReadMode)
{
	SWI2C_DBG_FUNC();

	if(eReadMode>=E_SWI2C_READ_MODE_MAX)
		return FALSE;
	g_I2CReadMode[u8BusNum] = eReadMode;
	return TRUE;
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
/*
 * config single i2c bus
 * 
 * */
void MDrv_SW_IIC_Init_Bus(IIC_BusCfg_t *SWI2CCBusCfg, u8 u8Bus)
{
	/* Store SWI2CCBusCfg for FastBoot Resume */
	g_IICBus[u8Bus] = *SWI2CCBusCfg;

	/* Config Bus */
	IIC_MUTEX_CREATE(u8Bus);
	IIC_ConfigBus(u8Bus, SWI2CCBusCfg);
	MDrv_SW_IIC_Speed_Setting(u8Bus, SWI2CCBusCfg->u16DefDelay);
	MDrv_SW_IIC_SetBusReadMode(u8Bus, E_SWI2C_READ_MODE_DIRECTION_CHANGE);
	IIC_Stop(u8Bus);
}
EXPORT_SYMBOL(MDrv_SW_IIC_Init_Bus);

/*
 * Setup mutex and CPU Speed
 *
 * */
void MDrv_SW_IIC_Init_Setup(void)
{
	g_u8CfgBusNum = IIC_BUS_MAX;

	//Get CPU clock & delay parameters

	//FIXME: how to query CPU current speed in kernel?
	//u32CpuSpeedMHz = (u32)(MDrv_CPU_QueryClock()/1000000UL);
	u32CpuSpeedMHz = 1000;
	SWI2C_DBG_INFO("@@@@@@ u32CpuSpeedMHz= %d MHz\n",(int)u32CpuSpeedMHz);
	BUG_ON(u32CpuSpeedMHz == 0);

	// create mutex to protect access specially for ISP Programming
	IIC_MUTEX_S_CREATE();
}
EXPORT_SYMBOL(MDrv_SW_IIC_Init_Setup);

void MDrv_SW_IIC_Init(IIC_BusCfg_t SWI2CCBusCfg[], u8 u8CfgBusNum)
{
	u8 u8Bus;
	SWI2C_DBG_FUNC();

	MDrv_SW_IIC_Init_Setup();

	for(u8Bus = 0; u8Bus < u8CfgBusNum; u8Bus++)
	{
		MDrv_SW_IIC_Init_Bus(SWI2CCBusCfg, u8Bus);
	}

}
EXPORT_SYMBOL(MDrv_SW_IIC_Init);

//-------------------------------------------------------------------------------------------------
/// IIC master initialization
/// @return None
/// @note   Hardware IIC. Called only once at system initialization
/// @brief reserved for HW IIC
//-------------------------------------------------------------------------------------------------
void MDrv_IIC_Init(void)
{
	MHal_IIC_Init();
}
EXPORT_SYMBOL(MDrv_IIC_Init);
//-------------------------------------------------------------------------------------------------
/// IIC Bus Enable or Disable
/// @param  u8ChIIC            \b IN:  channel index
/// @param  bEnable            \b IN:  enable
/// @return None
//-------------------------------------------------------------------------------------------------
void MDrv_SW_IIC_Enable( U8 u8ChIIC, B16 bEnable )
{
    g_IICBus[u8ChIIC].u8Enable = bEnable;
}

/******************************************************************************/
///Write bytes, be able to write 1 byte or several bytes to several register offsets in same slave address.
///@param u16BusNumSlaveID \b IN: Bus Number (high byte) and Slave ID (Address) (low byte)
///@param u8addrcount \b IN:  register NO to write, this parameter is the NO of register offsets in pu8addr buffer,
///it should be 0 when *pu8addr = NULL.
///@param *pu8addr \b IN: pointer to a buffer containing target register offsets to write
///@param u16size \b IN: Data length (in byte) to write
///@param *pu8data \b IN: pointer to the data buffer for write
///@return int:
///- TRUE: Success
///- FALSE: Fail
/******************************************************************************/
int MDrv_SW_IIC_WriteBytes(u8 u8BusNum, u8 u8SlaveID, u8 AddrCnt, u8* pu8addr, u16 u16size, u8* pBuf)
{

	int bRet = -EIO;
	u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
	u8 AddrCnt_temp=0;
	u16 u16size_temp=0;
	u8* pu8addr_temp=(u8*)NULL;
	u8* pBuf_temp=(u8*)NULL;
	SWI2C_DBG_FUNC();

	if(u8BusAllocNum == 0)
		return -ENOENT;

	LOCK_HW_SEM();
	IIC_MUTEX_S_LOCK();
	IIC_MUTEX_LOCK(u8BusNum);
	if (IIC_UseBus(u8BusNum) == FALSE)
	{
		IIC_MUTEX_UNLOCK(u8BusNum);
		IIC_MUTEX_S_UNLOCK();
                UNLOCK_HW_SEM();
		return -ENOENT;
	}

	while (u16Retries--)
	{
		AddrCnt_temp = AddrCnt;
		u16size_temp = u16size;
		pu8addr_temp =  (u8*)pu8addr;
		pBuf_temp	=  (u8*)pBuf;
		if (IIC_AccessStart(u8BusNum, u8SlaveID, SWI2C_WRITE) == FALSE)
		{
			if( u16Retries )
				continue;
			else
				goto fail;
		}

		while (AddrCnt_temp)
		{
			AddrCnt_temp--;
			if (IIC_SendByte(u8BusNum,*pu8addr_temp) == FALSE)
			{
				goto fail;
			}
			pu8addr_temp++;
		}
		while (u16size_temp) // loop of writting data
		{
			u16size_temp-- ;
			if (IIC_SendByte(u8BusNum,*pBuf_temp) == FALSE)
			{
				goto fail;
			}
			pBuf_temp++; // next byte pointer
		}
		bRet = 0;
		break;
fail:
		IIC_Stop(u8BusNum);
		bRet = -EIO;
	}

	IIC_Stop(u8BusNum);
	IIC_MUTEX_UNLOCK(u8BusNum);
	IIC_MUTEX_S_UNLOCK();
	UNLOCK_HW_SEM();

    return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_WriteBytes);

 /******************************************************************************/
 ///Write bytes with Stop control
 ///@param u16BusNumSlaveID \b IN: Bus Number (high byte) and Slave ID (Address) (low byte)
 ///@param AddrCnt \b IN:  register NO to write, this parameter is the NO of register offsets in pu8addr buffer,
 ///it should be 0 when *pu8addr = NULL.
 ///@param *pu8addr \b IN: pointer to a buffer containing target register offsets to write
 ///@param u16size \b IN: Data length (in byte) to write
 ///@param *pu8data \b IN: pointer to the data buffer for write
 ///@param bGenStop \b IN: control stop to be generated by result
 ///@return bool:
 ///- TRUE: Success
 ///- FALSE: Fail
 /******************************************************************************/
int MDrv_SW_IIC_WriteBytesStop(u8 u8BusNum, u8 u8SlaveID, u8 AddrCnt, u8* pu8addr, u16 u16size, u8* pBuf, int bGenStop)
{
	int bRet = -EIO;
	u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
	u8 AddrCnt_temp=0;
	u16 u16size_temp=0;
	u8* pu8addr_temp=(u8*)NULL;
	u8* pBuf_temp=(u8*)NULL;
	SWI2C_DBG_FUNC();

	if (u8BusAllocNum == 0)
		return -ENOENT;

	LOCK_HW_SEM();
	IIC_MUTEX_S_LOCK();
	IIC_MUTEX_LOCK(u8BusNum);
	if (IIC_UseBus(u8BusNum) == FALSE)
	{
		//IIC_MUTEX_UNLOCK(u8BusNum);
		//IIC_MUTEX_S_UNLOCK();
                UNLOCK_HW_SEM();
		return -ENOENT;
	}

	while (u16Retries--)
	{
		AddrCnt_temp = AddrCnt;
		u16size_temp = u16size;
		pu8addr_temp = (u8*)pu8addr;
		pBuf_temp	= (u8*)pBuf;
		if (IIC_AccessStart(u8BusNum, u8SlaveID, SWI2C_WRITE) == FALSE)
		{
			if(u16Retries)
				continue;
			else
				goto fail;
		}

		while (AddrCnt_temp)
		{
			AddrCnt_temp--;
			if (IIC_SendByte(u8BusNum, *pu8addr_temp) == FALSE)
			{
				goto fail;
			}
			pu8addr_temp++;
		}
		while (u16size_temp) // loop of writting data
		{
			u16size_temp-- ;
			if (IIC_SendByte(u8BusNum, *pBuf_temp) == FALSE)
			{
				goto fail;
			}
			pBuf_temp++; // next byte pointer
		 }
		bRet = 0;
		break;
fail:
		IIC_Stop(u8BusNum);
		bRet = -EIO;
	}
	

	if (bRet >= 0)
	{
		if (bGenStop)
		{
			IIC_Stop(u8BusNum);
		}
	}
	else
	{
		IIC_Stop(u8BusNum);
	}

	IIC_MUTEX_UNLOCK(u8BusNum);
	IIC_MUTEX_S_UNLOCK();
	UNLOCK_HW_SEM();

	return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_WriteBytesStop);

 /******************************************************************************/
///Read bytes, be able to read 1 byte or several bytes from several register offsets in same slave address.
///@param u16BusNumSlaveID \b IN: Bus Number (high byte) and Slave ID (Address) (low byte)
///@param u8AddrNum \b IN:  register NO to read, this parameter is the NO of register offsets in pu8addr buffer,
///it should be 0 when *paddr = NULL.
///@param *paddr \b IN: pointer to a buffer containing target register offsets to read
///@param u16size \b IN: Data length (in byte) to read
///@param *pu8data \b IN: pointer to retun data buffer.
///@return bool:
///- TRUE: Success
///- FALSE: Fail
/******************************************************************************/
int MDrv_SW_IIC_ReadBytes(u8 u8BusNum, u8 u8SlaveID, u8 ucSubAdr, u8* paddr, u16 ucBufLen, u8* pBuf)
{
	int bRet = -EIO;
	u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
	u8* pu8addr_temp=(u8*)NULL;
	u8* paddr_temp=(u8*)NULL;
	u16 ucBufLen_temp=0;
	u8 ucSubAdr_temp=0;
	SWI2C_DBG_FUNC();

	if(u8BusAllocNum == 0)
		return -ENOENT;

	LOCK_HW_SEM();
	IIC_MUTEX_S_LOCK();
	IIC_MUTEX_LOCK(u8BusNum);
	if (IIC_UseBus(u8BusNum) == FALSE)
	{
		IIC_MUTEX_UNLOCK(u8BusNum);
		IIC_MUTEX_S_UNLOCK();
                UNLOCK_HW_SEM();
		return -ENOENT;
	}

	while (u16Retries--)
	{
		ucBufLen_temp = ucBufLen;
		ucSubAdr_temp = ucSubAdr;
		pu8addr_temp  = (u8*)pBuf;
		paddr_temp	= (u8*)paddr;
		if((g_I2CReadMode[u8BusNum]!=E_SWI2C_READ_MODE_DIRECT) && (ucSubAdr_temp>0) && (paddr_temp))
		{
			if (IIC_AccessStart(u8BusNum, u8SlaveID, SWI2C_WRITE) == FALSE)
			{
				if( u16Retries )
					continue;
				else
					goto fail;
			}

			while (ucSubAdr_temp)
			{
				ucSubAdr_temp--;
				if (IIC_SendByte(u8BusNum,*paddr_temp) == FALSE)
				{
					goto fail;
				}
				paddr_temp++;
			}

			if(g_I2CReadMode[u8BusNum] == E_SWI2C_READ_MODE_DIRECTION_CHANGE_STOP_START)
			{
				IIC_Stop(u8BusNum);
			}
		}

		if (IIC_AccessStart(u8BusNum, u8SlaveID, SWI2C_READ) == FALSE)
		{
			if( u16Retries )
				continue;
			else
				goto fail;
		}

		while (ucBufLen_temp--) // loop to burst read
		{
			*pu8addr_temp = IIC_GetByte(u8BusNum, ucBufLen_temp); // receive byte
			pu8addr_temp++; // next byte pointer
		}
		bRet = 0;
		break;
fail:
		IIC_Stop(u8BusNum);
		bRet = -EIO;
	}

	IIC_Stop(u8BusNum);
	//IIC_UnuseBus(u8BusNum);
	IIC_MUTEX_UNLOCK(u8BusNum);
	IIC_MUTEX_S_UNLOCK();
	UNLOCK_HW_SEM();

	return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_ReadBytes);

/******************************************************************************/
///Read 1 byte through IIC
///@param u16BusNumSlaveID \b IN: Bus Number (high byte) and Slave ID (Address) (low byte)
///@param u8RegAddr \b IN: Target register offset to read
///@param *pu8Data \b IN: pointer to 1 byte return data.
///@return bool:
///- TRUE: Success
///- FALSE: Fail
/******************************************************************************/
int MDrv_SW_IIC_ReadByte(u8 u8BusNum, u8 u8SlaveID, u8 u8RegAddr, u8 *pu8Data)
{
	SWI2C_DBG_FUNC();
	return MDrv_SW_IIC_ReadBytes(u8BusNum, u8SlaveID, 1, &u8RegAddr,1, pu8Data);
}
EXPORT_SYMBOL(MDrv_SW_IIC_ReadByte);

/******************************************************************************/
///Write 1 byte through IIC
///@param u16BusNumSlaveID \b IN: Bus Number (high byte) and Slave ID (Address) (low byte)
///@param u8RegAddr \b IN: Target register offset to write
///@param u8Data \b IN: 1 byte data to write
///@return bool:
///- TRUE: Success
///- FALSE: Fail
/******************************************************************************/
int MDrv_SW_IIC_WriteByte(u8 u8BusNum, u8 u8SlaveID, u8 u8RegAddr, u8 u8Data)
{
	SWI2C_DBG_FUNC();
	return( MDrv_SW_IIC_WriteBytes(u8BusNum, u8SlaveID, 1, &u8RegAddr, 1, &u8Data) );
}
EXPORT_SYMBOL(MDrv_SW_IIC_WriteByte);

//------------------------------------------------------------------------
int MDrv_SW_IIC_Write2Bytes(u8 u8BusNum, u8 u8SlaveID, u8 u8addr, u16 u16data)
{
	u8 u8Data[2];

	SWI2C_DBG_FUNC();

	u8Data[0] = (u16data>>8) & 0xFF;
	u8Data[1] = (u16data) & 0xFF;
	return (MDrv_SW_IIC_WriteBytes(u8BusNum, u8SlaveID, 1, &u8addr, 2, u8Data));
}
EXPORT_SYMBOL(MDrv_SW_IIC_Write2Bytes);

u16 MDrv_SW_IIC_Read2Bytes(u8 u8BusNum, u8 u8SlaveID, u8 u8addr)
{
	u8 u8Data[2]={0, 0};
	SWI2C_DBG_FUNC();

	MDrv_SW_IIC_ReadBytes(u8BusNum, u8SlaveID, 1, &u8addr, 2, u8Data);
	return ( (((u16)u8Data[0]) << 8) | u8Data[1] );
}
EXPORT_SYMBOL(MDrv_SW_IIC_Read2Bytes);

int MDrv_SW_IIC_WriteByteDirectly(u8 u8BusNum, u8 u8SlaveID, u8 u8Data)
{
	int bRet = -EIO;

	SWI2C_DBG_FUNC();

	if (u8BusAllocNum == 0)
		return -ENOENT;

	LOCK_HW_SEM();
	IIC_MUTEX_S_LOCK();
	IIC_MUTEX_LOCK(u8BusNum);
	if (IIC_UseBus(u8BusNum) == FALSE)
	{
		//IIC_MUTEX_UNLOCK(u8BusNum);
		//IIC_MUTEX_S_UNLOCK();
                UNLOCK_HW_SEM();
		return -ENOENT;
	}

	IIC_Start(u8BusNum);
	if (IIC_SendByte(u8BusNum, (u8SlaveID & ~BIT0)) == FALSE)
		goto fail;
	if (IIC_SendByte(u8BusNum, u8Data) == FALSE)
		goto fail;

	bRet = 0;

fail:
	IIC_Stop(u8BusNum);
	//IIC_UnuseBus(u8BusNum);
	IIC_MUTEX_UNLOCK(u8BusNum);
	IIC_MUTEX_S_UNLOCK();
	UNLOCK_HW_SEM();

	return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_WriteByteDirectly);

int MDrv_SW_IIC_Write4Bytes(u8 u8BusNum, u8 u8SlaveID, u32 u32Data, u8 u8EndData)
{
	int bRet = -EIO;

	SWI2C_DBG_FUNC();

	if (u8BusAllocNum == 0)
		return -ENOENT;

	LOCK_HW_SEM();
	IIC_MUTEX_S_LOCK();
	IIC_MUTEX_LOCK(u8BusNum);
	if (IIC_UseBus(u8BusNum) == FALSE)
	{
		IIC_MUTEX_UNLOCK(u8BusNum);
		IIC_MUTEX_S_UNLOCK();
                UNLOCK_HW_SEM();
		return -ENOENT;
	}

	IIC_Start(u8BusNum);
	//send address
	if(IIC_SendByte(u8BusNum,(u8SlaveID & ~BIT0))==FALSE)
		goto fail;

	//send 4 bytes
	if(IIC_SendByte(u8BusNum,(u8)((u32Data)>>24) ) == FALSE )
		goto fail;
	if(IIC_SendByte(u8BusNum,(u8)((u32Data)>>16) ) == FALSE )
		goto fail;
	if(IIC_SendByte(u8BusNum,(u8)((u32Data)>>8) ) == FALSE )
		goto fail;
	if(IIC_SendByte(u8BusNum,(u8)((u32Data)>>0) ) == FALSE )
		goto fail;
	if(IIC_SendByte(u8BusNum,u8EndData)==FALSE)
		goto fail;

	bRet = 0;

fail:

	IIC_Stop(u8BusNum);
	//IIC_UnuseBus(u8BusNum);
	IIC_MUTEX_UNLOCK(u8BusNum);
	IIC_MUTEX_S_UNLOCK();
	UNLOCK_HW_SEM();

    return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_Write4Bytes);

int MDrv_SW_IIC_WriteByteArrayDirectly(u8 u8BusNum, u8 u8SlaveID, u16 u16size, u8* pu8Data, int stop)
{
    int bRet = -EIO;
    u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
    u16 u16size_temp=0;
    u8* pu8Data_temp=(u8*)NULL;

    SWI2C_DBG_FUNC();

    if (u8BusAllocNum == 0)
        return -ENOENT;

    if (IIC_UseBus(u8BusNum) == FALSE)
    {
        return -ENOENT;
    }

    while (u16Retries--)
    {
        u16size_temp = u16size;
        pu8Data_temp = (u8*)pu8Data;

        if (IIC_AccessStart(u8BusNum, u8SlaveID, SWI2C_WRITE) == FALSE)
        {
            if( u16Retries )
                continue;
            else
                goto fail;
        }

        while (u16size_temp)
        {
            u16size_temp--;
            if (IIC_SendByte(u8BusNum, *pu8Data_temp) == FALSE) {
                goto fail;
            }
            pu8Data_temp++;
        }
        bRet = u16size;
        break;
fail:
        IIC_Stop(u8BusNum);
        bRet = -EIO;
    }

    if (stop)
    {
        IIC_Stop(u8BusNum);
    }

    return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_WriteByteArrayDirectly);

int MDrv_SW_IIC_ReadByteArrayDirectly(u8 u8BusNum, u8 u8SlaveID, u16 u16size, u8* pu8Data, int stop)
{
    int bRet = -EIO;
    u16 u16Retries = g_IICBus[u8BusNum].u16Retries;
    u8* pu8Data_temp = (u8*)NULL;
    u16 u16size_temp = 0;

    SWI2C_DBG_FUNC();

    if (u8BusAllocNum == 0)
        return -ENOENT;

    if (IIC_UseBus(u8BusNum) == FALSE)
    {
        return -ENOENT;
    }

    while (u16Retries--)
    {
        u16size_temp = u16size;
        pu8Data_temp  = (u8*)pu8Data;

        if (IIC_AccessStart(u8BusNum, u8SlaveID, SWI2C_READ) == FALSE)
        {
            if( u16Retries )
                continue;
            else
            {
                goto fail;
            }
        }

        while(u16size_temp)
        {
            u16size_temp--;
            *pu8Data_temp = IIC_GetByte((u8BusNum), u16size_temp); // receive byte, generate
            pu8Data_temp++; // next byte pointer
        }
        bRet = u16size;
        break;
fail:
        IIC_Stop(u8BusNum);
        bRet = u16size;
    }

    if (stop)
    {
        IIC_Stop(u8BusNum);
    }

    return bRet;
}
EXPORT_SYMBOL(MDrv_SW_IIC_ReadByteArrayDirectly);
#undef _API_SWI2C_C_

//-------------------------------------------------------------------------------------------------
/// IIC clock selection
/// @param  u8ClockIIC            \b IN:  clock selection
/// @return None
/// @note
//-------------------------------------------------------------------------------------------------
void MDrv_HW_IIC_Clock_Select(U8 u8ClockIIC)
{
    MHal_IIC_Clock_Select(u8ClockIIC);
}
EXPORT_SYMBOL(MDrv_HW_IIC_Clock_Select);

//-------------------------------------------------------------------------------------------------
/// Write data to an IIC device
/// @param  u8SlaveIdIIC            \b IN:  device slave ID
/// @param  u8AddrSizeIIC           \b IN:  address length in bytes
/// @param  pu8AddrIIC              \b IN:  pointer to the start address of the device
/// @param  u32BufSizeIIC           \b IN:  number of bytes to be written
/// @param  pu8BufIIC               \b IN:  pointer to write data buffer
/// @return TRUE(succeed), FALSE(fail)
/// @note   Not allowed in interrupt context
//-------------------------------------------------------------------------------------------------
S32 MDrv_HW_IIC_WriteBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC)
{
    U32     u32I = 0;
    U8      u8I = 0;
    B16     bReturnIIC = TRUE;
    S32     s32RetCountIIC = -1;

    U32     uAddrCntBkp;
    U32     uSizeBkp;
    U8      *pRegAddrBkp;
    U8      *pDataBkp;
    U16     retryCount = IIC_WR_RETRY_TIMES;

    U32     u32BaseReg;

    HWIIC_MUTEX_LOCK(u8Port);

    MHal_HWI2C_GetBaseRegByPort(gp_HWI2CinitCfg[u8Port].ePort, &u32BaseReg);

HW_IIC_Write_Start:
    uAddrCntBkp = u8AddrSizeIIC;
    pRegAddrBkp = pu8AddrIIC;
    uSizeBkp    = u32BufSizeIIC;
    pDataBkp    = pu8BufIIC;

    if(MHal_IIC_Start(u32BaseReg) == FALSE)
    {
        IIC_DBG(printk("%s Start condition failed\n", __FUNCTION__));
        goto HW_IIC_Write_End;
    }

    udelay(5);

    if (FALSE == MHal_IIC_SendByte(u32BaseReg, HWI2C_SET_RW_BIT(FALSE, u8SlaveIdIIC)))
    {
        bReturnIIC = FALSE;
        s32RetCountIIC = -2;
        goto HW_IIC_Write_End;
    }

    for (u8I = 0; u8I < u8AddrSizeIIC; u8I++)
    {
        if (FALSE == MHal_IIC_SendByte(u32BaseReg, pu8AddrIIC[u8I]))
        {
            bReturnIIC = FALSE;
            s32RetCountIIC = -3;
            goto HW_IIC_Write_End;
        }
    }

    for (u32I = 0; u32I < u32BufSizeIIC; u32I++)
    {
        if (FALSE == MHal_IIC_SendByte(u32BaseReg, pu8BufIIC[u32I]))
        {
            bReturnIIC = FALSE;
            s32RetCountIIC = -4;
            goto HW_IIC_Write_End;
        }
    }

    s32RetCountIIC = u32BufSizeIIC;

HW_IIC_Write_End:
    MHal_IIC_Stop(u32BaseReg);
    if((retryCount > 0) && (s32RetCountIIC < 0))
    {
        MHal_HWI2C_Reset_ALL(u32BaseReg, TRUE);
        MHal_HWI2C_Reset_ALL(u32BaseReg, FALSE);
        IIC_DBG(printk("[%s][%d] Reset IIC \n", __FUNCTION__, __LINE__));
        retryCount--;
        goto HW_IIC_Write_Start;
    }

    if(retryCount == 0)
        printk(KERN_DEBUG "[IIC] Write to slave ID 0x%x failed. Please check slave id or port is correct \n", u8SlaveIdIIC);

    usleep_range(60, 120);

    IIC_DBG(printk("MDrv_IIC_Write() --> s32RetCountIIC=%d \n", s32RetCountIIC));

    HWIIC_MUTEX_UNLOCK(u8Port);
    return s32RetCountIIC;
}
EXPORT_SYMBOL(MDrv_HW_IIC_WriteBytes);

//-------------------------------------------------------------------------------------------------
/// Write data to an IIC device
/// @param  u8Port                  \b IN:  HWI2C Port id
/// @param  u8SlaveIdIIC            \b IN:  Device Slave id
/// @param  u16BufSizeIIC           \b IN:  number of bytes to be written
/// @param  pu8BufIIC               \b IN:  pointer to write data buffer
/// @param  stop                    \b IN:  generic i2c stop flag
/// @return i2cBufSize: Success, Error code(nagative value): fail
/// @note   This Fuction is only used by generic i2c driver
//-------------------------------------------------------------------------------------------------
S32 MDrv_HW_IIC_WriteByteArrayDirectly(U8 u8Port, U8 u8SlaveIdIIC, U16 u16BufSizeIIC, U8 *pu8BufIIC, int stop)
{
    U16     u16I = 0;
    B16     bReturnIIC = TRUE;
    S32     s32RetCountIIC = -1;

    U16     uSizeBkp;
    U8      *pDataBkp;
    U16     retryCount = IIC_WR_RETRY_TIMES;

    U32     u32BaseReg;

    MHal_HWI2C_GetBaseRegByPort(gp_HWI2CinitCfg[u8Port].ePort, &u32BaseReg);

HW_IIC_Write_Start:
    uSizeBkp    = u16BufSizeIIC;
    pDataBkp    = pu8BufIIC;

    if(MHal_IIC_Start(u32BaseReg) == FALSE)
    {
        IIC_DBG(printk("%s Start condition failed\n", __FUNCTION__));
        goto HW_IIC_Write_End;
    }

    udelay(5);

    if (FALSE == MHal_IIC_SendByte(u32BaseReg, HWI2C_SET_RW_BIT(FALSE, u8SlaveIdIIC)))
    {
        bReturnIIC = FALSE;
        s32RetCountIIC = -2;
        goto HW_IIC_Write_End;
    }

    for (u16I = 0; u16I < u16BufSizeIIC; u16I++)
    {
        if (FALSE == MHal_IIC_SendByte(u32BaseReg, pu8BufIIC[u16I]))
        {
            bReturnIIC = FALSE;
            s32RetCountIIC = -4;
            goto HW_IIC_Write_End;
        }
    }

    s32RetCountIIC = u16BufSizeIIC;

HW_IIC_Write_End:
    if((retryCount > 0) && (s32RetCountIIC < 0))
    {
        //MHal_HWI2C_Reset_ALL(u32BaseReg, TRUE);
        //MHal_HWI2C_Reset_ALL(u32BaseReg, FALSE);
        //IIC_DBG(printk("[%s][%d] Reset IIC \n", __FUNCTION__, __LINE__));
        retryCount--;
        MHal_IIC_Stop(u32BaseReg);
        goto HW_IIC_Write_Start;
    }
    else
    {
        if(stop)
            MHal_IIC_Stop(u32BaseReg);
    }

    if(retryCount == 0)
        printk(KERN_DEBUG "[IIC] Write to slave ID 0x%x failed. Please check slave id or port is correct \n", u8SlaveIdIIC);

    usleep_range(60, 120);

    IIC_DBG(printk("MDrv_IIC_Write() --> s32RetCountIIC=%d \n", s32RetCountIIC));
    return s32RetCountIIC;
}
EXPORT_SYMBOL(MDrv_HW_IIC_WriteByteArrayDirectly);

//-------------------------------------------------------------------------------------------------
/// Read data from an IIC device
/// @param  u8SlaveIdIIC            \b IN:  device slave ID
/// @param  u8AddrSizeIIC           \b IN:  address length in bytes
/// @param  pu8AddrIIC              \b IN:  ptr to the start address inside the device
/// @param  u32BufSizeIIC           \b IN:  number of bytes to be read
/// @param  pu8BufIIC               \b OUT: pointer to read data buffer
/// @return TRUE : succeed
/// @return FALSE : fail
/// @note   Not allowed in interrupt context
//-------------------------------------------------------------------------------------------------
S32 MDrv_HW_IIC_ReadBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC)
{
    U32     u32I;
    U8      u8I;
    B16     bReturnIIC = TRUE;
    S32     s32RetCountIIC = -1;

    U8      uAddrCntBkp;
    U32     uBufSizeBkp;
    U8      *pRegAddrBkp;
    U8      *pBufBkp;
    U16     retryCount = IIC_WR_RETRY_TIMES;

    U32     u32BaseReg;

    HWIIC_MUTEX_LOCK(u8Port);

    MHal_HWI2C_GetBaseRegByPort(gp_HWI2CinitCfg[u8Port].ePort, &u32BaseReg);

HW_IIC_Read_Start:
    uAddrCntBkp = u8AddrSizeIIC;
    pRegAddrBkp = pu8AddrIIC;
    uBufSizeBkp = u32BufSizeIIC;
    pBufBkp     = pu8BufIIC;

    if(MHal_IIC_Start(u32BaseReg) == FALSE)
    {
        IIC_DBG(printk("%s Start condition failed\n", __FUNCTION__));
        goto HW_IIC_Read_End;
    }

    udelay(5);

    if (FALSE == MHal_IIC_SendByte(u32BaseReg, HWI2C_SET_RW_BIT(FALSE, u8SlaveIdIIC)))
    {
        bReturnIIC = FALSE;
        s32RetCountIIC = -2;
        goto HW_IIC_Read_End;
    }

    for (u8I = 0; u8I < u8AddrSizeIIC; u8I++)
    {
        if (FALSE == MHal_IIC_SendByte(u32BaseReg, pu8AddrIIC[u8I]))
        {
            bReturnIIC = FALSE;
            s32RetCountIIC = -3;
            goto HW_IIC_Read_End;
        }
    }

    MHal_IIC_Stop(u32BaseReg);

    udelay(5);

    if (MHal_IIC_Start(u32BaseReg) == FALSE)
        goto HW_IIC_Read_End;

    udelay(5);

    if (FALSE == MHal_IIC_SendByte(u32BaseReg, HWI2C_SET_RW_BIT(TRUE, u8SlaveIdIIC)))
    {
        bReturnIIC = FALSE;
        s32RetCountIIC = -4;
        goto HW_IIC_Read_End;
    }

    for (u32I = 0; u32I < u32BufSizeIIC; u32I++)
    {
        if (u32I == (u32BufSizeIIC-1))
        {
            MHal_HWI2C_NoAck(u32BaseReg);
        }

        if (FALSE == MHal_IIC_GetByte(u32BaseReg, &pu8BufIIC[u32I]))
        {
            bReturnIIC = FALSE;
            s32RetCountIIC = -5;
            goto HW_IIC_Read_End;
        }
    }

    s32RetCountIIC = u32BufSizeIIC;

HW_IIC_Read_End:
    MHal_IIC_Stop(u32BaseReg);
    if((retryCount > 0) && (s32RetCountIIC < 0))
    {
        MHal_HWI2C_Reset_ALL(u32BaseReg, TRUE);
        MHal_HWI2C_Reset_ALL(u32BaseReg, FALSE);
        IIC_DBG(printk("[%s][%d] Reset IIC \n", __FUNCTION__, __LINE__));
        retryCount--;
        goto HW_IIC_Read_Start;
    }

    if(retryCount == 0)
        printk(KERN_DEBUG "[IIC] Read from slave ID 0x%x failed. Please check slave id or port is correct \n", u8SlaveIdIIC);

    usleep_range(60, 120);

    IIC_DBG(printk("MDrv_IIC_Read() --> s32RetCountIIC=%d \n", s32RetCountIIC));

    HWIIC_MUTEX_UNLOCK(u8Port);
    return s32RetCountIIC;
}
EXPORT_SYMBOL(MDrv_HW_IIC_ReadBytes);

//-------------------------------------------------------------------------------------------------
/// Read data from an IIC device
/// @param  u8Port                  \b IN:  HWI2C Port id
/// @param  u8SlaveIdIIC            \b IN:  Device Slave id
/// @param  u16BufSizeIIC           \b IN:  number of bytes to be read
/// @param  pu8BufIIC               \b OUT: pointer to read data buffer
/// @param  stop                    \b IN:  generic i2c stop flag
/// @return i2cBufSize : Success, Error code(nagative value) : fail
/// @note   This Fuction is only used by generic i2c driver
//-------------------------------------------------------------------------------------------------
S32 MDrv_HW_IIC_ReadByteArrayDirectly(U8 u8Port, U8 u8SlaveIdIIC, U16 u16BufSizeIIC, U8 *pu8BufIIC, int stop)
{
    U16     u16I;
    B16     bReturnIIC = TRUE;
    S32     s32RetCountIIC = -1;

    U16     uBufSizeBkp;
    U8      *pBufBkp;
    U16     retryCount = IIC_WR_RETRY_TIMES;

    U32     u32BaseReg;

    MHal_HWI2C_GetBaseRegByPort(gp_HWI2CinitCfg[u8Port].ePort, &u32BaseReg);

HW_IIC_Read_Start:
    uBufSizeBkp = u16BufSizeIIC;
    pBufBkp     = pu8BufIIC;

    if(MHal_IIC_Start(u32BaseReg) == FALSE)
    {
        IIC_DBG(printk("%s Start condition failed\n", __FUNCTION__));
        goto HW_IIC_Read_End;
    }

    udelay(5);

    if (FALSE == MHal_IIC_SendByte(u32BaseReg, HWI2C_SET_RW_BIT(TRUE, u8SlaveIdIIC)))
    {
        bReturnIIC = FALSE;
        s32RetCountIIC = -4;
        goto HW_IIC_Read_End;
    }

    for (u16I = 0; u16I < u16BufSizeIIC; u16I++)
    {
        if (u16I == (u16BufSizeIIC-1))
        {
            MHal_HWI2C_NoAck(u32BaseReg);
        }

        if (FALSE == MHal_IIC_GetByte(u32BaseReg, &pu8BufIIC[u16I]))
        {
            bReturnIIC = FALSE;
            s32RetCountIIC = -5;
            goto HW_IIC_Read_End;
        }
    }

    s32RetCountIIC = u16BufSizeIIC;

HW_IIC_Read_End:
    if((retryCount > 0) && (s32RetCountIIC < 0))
    {
        //MHal_HWI2C_Reset_ALL(u32BaseReg, TRUE);
        //MHal_HWI2C_Reset_ALL(u32BaseReg, FALSE);
        //IIC_DBG(printk("[%s][%d] Reset IIC \n", __FUNCTION__, __LINE__));
        retryCount--;
        MHal_IIC_Stop(u32BaseReg);
        goto HW_IIC_Read_Start;
    }
    else
    {
        if(stop)
            MHal_IIC_Stop(u32BaseReg);
    }

    if(retryCount == 0)
        printk(KERN_DEBUG "[IIC] Read from slave ID 0x%x failed. Please check slave id or port is correct \n", u8SlaveIdIIC);

    usleep_range(60, 120);

    IIC_DBG(printk("MDrv_IIC_Read() --> s32RetCountIIC=%d \n", s32RetCountIIC));
    return s32RetCountIIC;
}
EXPORT_SYMBOL(MDrv_HW_IIC_ReadByteArrayDirectly);

void MDrv_HW_IIC_Resume(void)
{
    U8      u8I;
    U32 u32BaseReg;

    for(u8I = 0; u8I<HWI2C_PORTS; u8I++)
    {
        if(gp_HWI2CinitCfg[u8I].bEnable)
        {
            MHal_IIC_STR_Record();
            /*
	          (1) Select Pad Mux
            */
            MHal_HWI2C_SelectPort(gp_HWI2CinitCfg[u8I].ePort);

            MHal_HWI2C_GetBaseRegByPort(gp_HWI2CinitCfg[u8I].ePort, &u32BaseReg);
            /*
	          (2)	Set Clock Speed
            */
            MHal_HWI2C_SetClk(u32BaseReg, gp_HWI2CinitCfg[u8I].eSpeed);

            //set read mode
            //MHAL_HWI2C_SetReadMode(u32Offset,psCfg->eReadMode);
            /*
	          (3) Master Enable
            */
            MHal_HWI2C_Master_Enable(u32BaseReg);
        }
    }
}
//-------------------------------------------------------------------------------------------------
/// IIC initialization
/// @param  None
/// @return None
//-------------------------------------------------------------------------------------------------
void MDrv_HW_IIC_Init(IIC_BusCfg_t I2CBusCfg[], u8 u8CfgBusNum)
{
	  U8 u8PortIdx;
    U32 u32BaseReg;

	IIC_DBG(printk(">>  MDrv_HW_IIC_Init  %s %d  \n ",__FILE__,__LINE__));

    if(MHal_HWI2C_GetPortIdxByPort((HAL_HWI2C_PORT)I2CBusCfg->u32PadMux, &u8PortIdx))
    {
        gp_HWI2CinitCfg[u8PortIdx].ePort = I2CBusCfg->u32PadMux;
        gp_HWI2CinitCfg[u8PortIdx].eSpeed = I2CBusCfg->u16SpeedKHz;
        gp_HWI2CinitCfg[u8PortIdx].bEnable = TRUE;
    }
    else
    {
        //do nothing
        printk("[HWIIC] Not support this port. %s, %d\n", __FUNCTION__, __LINE__);
        return;
    }
	/* move to probe function to avoid utpa using mutex without calling init
    //HWIIC_MUTEX_CREATE(u8PortIdx);

    /*
	  (1) Select Pad Mux
    */
    MHal_HWI2C_SelectPort(I2CBusCfg->u32PadMux);

    MHal_HWI2C_GetBaseRegByPort(gp_HWI2CinitCfg[u8PortIdx].ePort, &u32BaseReg);
    /*
	  (2)	Set Clock Speed
    */
    MHal_HWI2C_SetClk(u32BaseReg, I2CBusCfg->u16SpeedKHz);

    //set read mode
    //MHAL_HWI2C_SetReadMode(u32Offset,psCfg->eReadMode);
    /*
	  (3) Master Enable
    */
    MHal_HWI2C_Master_Enable(u32BaseReg);
}
EXPORT_SYMBOL(MDrv_HW_IIC_Init);
