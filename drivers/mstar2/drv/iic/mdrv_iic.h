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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_iic.h
/// @brief  IIC Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DRV_IIC_H_
#define _DRV_IIC_H_

#include <asm/types.h>
#include "mdrv_types.h"
#include "mdrv_iic_io.h"
#include "mhal_iic.h"
//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------
#define	IIC_NUM_OF_MAX				(16)

#if 1
#define	IIC_NUM_OF_HW				(0)
#define IIC_NUM_OF_SYS_EEPROM		(1)
#else
#define	IIC_NUM_OF_HW				(1)
#define IIC_NUM_OF_SYS_EEPROM		(2)
#define IIC_NUM_OF_NEC_MICOM		(3)
#define IIC_NUM_OF_AUDIO_AMP		(4)

#define IIC_NUM_OF_RGB_EDID			(8)
#define IIC_NUM_OF_HDMI_A_EDID		(9)
#define IIC_NUM_OF_HDMI_B_EDID		(10)
#define IIC_NUM_OF_HDMI_C_EDID		(11)
#define IIC_NUM_OF_HDMI_D_EDID		(12)
#endif
//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------

#define TRUE                            1
#define FALSE                           0

#define MSTAR_SWIIC                     (0)
#define MSTAR_HWIIC                     (1)

#define IIC_BUS_MAX                     IIC_NUM_OF_MAX
#define HWI2C_PORTS                     HAL_HWI2C_PORTS

extern struct mutex gBusLocks[IIC_BUS_MAX];
extern struct mutex gAdapterLock;
#define IIC_MUTEX_CREATE(_P_)           mutex_init(&gBusLocks[_P_])
#define IIC_MUTEX_LOCK(_P_)             mutex_lock(&gBusLocks[_P_])
#define IIC_MUTEX_UNLOCK(_P_)           mutex_unlock(&gBusLocks[_P_])
#define IIC_MUTEX_DELETE(_P_)

#define IIC_MUTEX_S_CREATE()            mutex_init(&gAdapterLock)
#define IIC_MUTEX_S_LOCK()              mutex_lock(&gAdapterLock)
#define IIC_MUTEX_S_UNLOCK()            mutex_unlock(&gAdapterLock)
#define IIC_MUTEX_S_DELETE()

extern struct mutex gPortLocks[HWI2C_PORTS];
#define HWIIC_MUTEX_CREATE(_P_)         mutex_init(&gPortLocks[_P_])
#define HWIIC_MUTEX_LOCK(_P_)           mutex_lock(&gPortLocks[_P_])
#define HWIIC_MUTEX_UNLOCK(_P_)         mutex_unlock(&gPortLocks[_P_])
#define HWIIC_MUTEX_DELETE(_P_)

// Speed Mapping: Base on the Driver loading, maybe you must to use the oscilloscope to measure this Value// Speed Mapping has been refined, and you can specify the IIC speed X (KHz) driectly by X.
#define SWI2C_SPEED_MAPPING_400K          400
#define SWI2C_SPEED_MAPPING_350K          350
#define SWI2C_SPEED_MAPPING_300K          300
#define SWI2C_SPEED_MAPPING_250K          250
#define SWI2C_SPEED_MAPPING_200K          200
#define SWI2C_SPEED_MAPPING_150K          150
#define SWI2C_SPEED_MAPPING_100K          100
#define SWI2C_SPEED_MAPPING_50K           50

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

///Define SWI2C read mode
typedef enum {    
	E_SWI2C_READ_MODE_DIRECT,                 			///< first transmit slave address + reg address and then start receive the data */    
	E_SWI2C_READ_MODE_DIRECTION_CHANGE,		///< slave address + reg address in write mode, direction change to read mode, repeat start slave address in read mode, data from device    
	E_SWI2C_READ_MODE_DIRECTION_CHANGE_STOP_START,  ///< slave address + reg address in write mode + stop, direction change to read mode, repeat start slave address in read mode, data from device   
	E_SWI2C_READ_MODE_MAX
} SWI2C_ReadMode;

///Define SWI2C debug level
typedef enum _SWI2C_DbgLvl{    
	E_SWI2C_DBGLVL_NONE = 0,		/// no debug message    
	E_SWI2C_DBGLVL_WARNING,		/// show warning only    
	E_SWI2C_DBGLVL_ERROR,		/// show error only    
	E_SWI2C_DBGLVL_INFO, 			/// show error & informaiton    
	E_SWI2C_DBGLVL_ALL,			/// show error, information & funciton name
}SWI2C_DbgLvl;

/// I2C select master port
typedef enum _HWI2C_PORT
{
    E_HWI2C_PORT_0 = 0, /// port 0_0 //disable port 0
    E_HWI2C_PORT0_1,    /// port 0_1
    E_HWI2C_PORT0_2,    /// port 0_2
    E_HWI2C_PORT0_3,    /// port 0_3
    E_HWI2C_PORT0_4,    /// port 0_4
    E_HWI2C_PORT0_5,    /// port 0_5
    E_HWI2C_PORT0_6,    /// port 0_6
    E_HWI2C_PORT0_7,    /// port 0_7

    E_HWI2C_PORT_1 = 8, /// port 1_0 //disable port 1
    E_HWI2C_PORT1_1,    /// port 1_1
    E_HWI2C_PORT1_2,    /// port 1_2
    E_HWI2C_PORT1_3,    /// port 1_3
    E_HWI2C_PORT1_4,    /// port 1_4
    E_HWI2C_PORT1_5,    /// port 1_5
    E_HWI2C_PORT1_6,    /// port 1_6
    E_HWI2C_PORT1_7,    /// port 1_7

    E_HWI2C_PORT_2 = 16,/// port 2_0 //disable port 2
    E_HWI2C_PORT2_1,    /// port 2_1
    E_HWI2C_PORT2_2,    /// port 2_2
    E_HWI2C_PORT2_3,    /// port 2_3
    E_HWI2C_PORT2_4,    /// port 2_4
    E_HWI2C_PORT2_5,    /// port 2_5
    E_HWI2C_PORT2_6,    /// port 2_6
    E_HWI2C_PORT2_7,    /// port 2_7

    E_HWI2C_PORT_3 = 24,/// port 3_0 //disable port 3
    E_HWI2C_PORT3_1,    /// port 3_1
    E_HWI2C_PORT3_2,    /// port 3_2
    E_HWI2C_PORT3_3,    /// port 3_3
    E_HWI2C_PORT3_4,    /// port 3_4
    E_HWI2C_PORT3_5,    /// port 3_5
    E_HWI2C_PORT3_6,    /// port 3_6
    E_HWI2C_PORT3_7,    /// port 3_7

    E_HWI2C_PORT_4 = 32,/// port 4_0 //disable port 4
    E_HWI2C_PORT4_1,    /// port 4_1
    E_HWI2C_PORT4_2,    /// port 4_2
    E_HWI2C_PORT4_3,    /// port 4_3
    E_HWI2C_PORT4_4,    /// port 4_4
    E_HWI2C_PORT4_5,    /// port 4_5
    E_HWI2C_PORT4_6,    /// port 4_6
    E_HWI2C_PORT4_7,    /// port 4_7

    E_HWI2C_PORT_5 = 40,/// port 5_0 //disable port 5
    E_HWI2C_PORT5_1,    /// port 5_1
    E_HWI2C_PORT5_2,    /// port 5_2
    E_HWI2C_PORT5_3,    /// port 5_3
    E_HWI2C_PORT5_4,    /// port 5_4
    E_HWI2C_PORT5_5,    /// port 5_5
    E_HWI2C_PORT5_6,    /// port 5_6
    E_HWI2C_PORT5_7,    /// port 5_7

    E_HWI2C_PORT_NOSUP  /// non-support port
}HWI2C_PORT;

/// I2C port config
typedef struct _HWI2C_PortCfg
{
    HWI2C_PORT          ePort;          /// number
    U16                 eSpeed;         /// clock speed
    BOOL                bEnable;        /// enable
}HWI2C_PortCfg;

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------

int MDrv_SW_IIC_WriteByteArrayDirectly(u8 u8BusNum, u8 u8SlaveID, u16 u16size, u8* pu8Data, int stop);
int MDrv_SW_IIC_ReadByteArrayDirectly(u8 u8BusNum, u8 u8SlaveID, u16 u16size, u8* pu8Data, int stop);
S32 MDrv_HW_IIC_ReadByteArrayDirectly(U8 u8Port, U8 u8SlaveIdIIC, U16 u16BufSizeIIC, U8 *pu8BufIIC, int stop);
S32 MDrv_HW_IIC_WriteByteArrayDirectly(U8 u8Port, U8 u8SlaveIdIIC, U16 u16BufSizeIIC, U8 *pu8BufIIC, int stop);
#endif // _DRV_IIC_H_

