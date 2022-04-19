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

#ifndef __DEV_SYSTEM_H__
#define __DEV_SYSTEM_H__

/* Use 'S' as magic number */
#define SYS_IOC_MAGIC           'S'

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

/// System output pad switch type
typedef enum
{
    E_SYS_PAD_MSD5010_SM2_IIC2_DEV,                                     ///< 5010 SM2, IIC2
    E_SYS_PAD_MSD5011_SM2_IIC2_DEV,                                     ///< 5011 SM2, IIC2
    E_SYS_PAD_MSD5015_GPIO_DEV,                                         ///< 5015 GPIO
    E_SYS_PAD_MSD5018_SM2_DEV,                                          ///< 5018 SM2
} SYS_PadType_DEV;


//------------------------------------------------------------------------------
// Data structure
//------------------------------------------------------------------------------
// DEV_SYS_IOC_SWITCH_PAD
typedef struct
{
    SYS_PadType_DEV             PadType;
    MS_U32                         bSuccess;
} DevSys_Switch_Pad;


//-------------------------------------------------------------------------------------------------
//  ioctl method
//-------------------------------------------------------------------------------------------------
// MS_BOOL MDrv_System_Init(void);
#define DEV_SYS_IOC_INIT                _IOWR(SYS_IOC_MAGIC, 0x00, MS_U32)

// MS_BOOL MDrv_System_SwitchPad(SYS_PadType ePadType);
#define DEV_SYS_IOC_SWITCH_PAD          _IOWR(SYS_IOC_MAGIC, 0x01, DevSys_Switch_Pad)

// void MDrv_System_WDTEnable(MS_BOOL bEnable);
#define DEV_SYS_IOC_WDT_ENABLE          _IOW (SYS_IOC_MAGIC, 0x02, MS_U32)

// void MDrv_System_WDTClear(void);
#define DEV_SYS_IOC_WDT_CLEAR           _IO  (SYS_IOC_MAGIC, 0x03)

// MS_BOOL MDrv_System_WDTLastStatus(void);
#define DEV_SYS_IOC_WDT_LASTSTATUS      _IOWR(SYS_IOC_MAGIC, 0x04, MS_U32)

// void MDrv_System_WDTSetTime(MS_U32 u32Ms);
#define DEV_SYS_IOC_WDT_SETTIME         _IOW (SYS_IOC_MAGIC, 0x05, MS_U32)

// void MDrv_System_ResetChip(void);
#define DEV_SYS_IOC_RESET_CHIP          _IO  (SYS_IOC_MAGIC, 0x06)

// void MDrv_System_ResetCPU(void);
#define DEV_SYS_IOC_RESET_CPU           _IO  (SYS_IOC_MAGIC, 0x07)

#endif // #ifndef __DEV_SYSTEM_H__
