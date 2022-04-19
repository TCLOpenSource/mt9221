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
/// @file   Mdrv_mtlb.h
/// @brief  MTLB Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __DRV_PM_H__
#define __DRV_PM_H__

//-------------------------------------------------------------------------------------------------
//  Define
//-------------------------------------------------------------------------------------------------
/* Print type. */
#define MDRV_PM_ERR(fmt, args...)   printk(KERN_ERR     "[%s][%d] " fmt, __func__, __LINE__, ## args)
#define MDRV_PM_INFO(fmt, args...)  printk(KERN_INFO    "[%s][%d] " fmt, __func__, __LINE__, ## args)
#define MDRV_PM_DEBUG(fmt, args...) printk(KERN_DEBUG   "[%s][%d] " fmt, __func__, __LINE__, ## args)

/* Using by MDrv_PM_Mapping(), MDrv_PM_Unmapping(). */
#define PM_MMAP_DATA                        (0x01)
#define PM_MMAP_DRAM                        (0x02)

/* Using by struct PM_WakeCfg_t. */
#define MAX_BUF_WAKE_IR                     (32)
#define MAX_BUF_WAKE_IR2                    (16)
#define MAX_BUF_WAKE_MAC_ADDRESS            (6)
#define CRC_KERNEL_BUF                      (3)
#define PM_MAX_BUF_WAKE_IR_WAVEFORM_ADDR    (4)
#define PM_MAX_BUF_RESERVERD                (4)

/* Using by struct PM_WakeCfg_t member u8PmStrMode. */
#define PM_DC_MODE                          (0x01)
#define PM_STR_MODE                         (0x02)

/* Using by setting wakeup source disable. */
#define PM_SOURCE_DISABLE                   (0xFF)

/* Using by wakeup reason. */
#define PM_WAKEUPSRC_NONE                   (0x00)
#define PM_WAKEUPSRC_USB                    (0x10)
#define PM_WAKEUPSRC_IR                     (0x01)
#define PM_WAKEUPSRC_DVI                    (0x02)
#define PM_WAKEUPSRC_DVI2                   (0x03)
#define PM_WAKEUPSRC_CEC                    (0x04)
#define PM_WAKEUPSRC_CEC_PORT1              (0x14)
#define PM_WAKEUPSRC_CEC_PORT2              (0x24)
#define PM_WAKEUPSRC_CEC_PORT3              (0x34)
#define PM_WAKEUPSRC_SAR                    (0x05)
#define PM_WAKEUPSRC_ESYNC                  (0x06)
#define PM_WAKEUPSRC_SYNC                   (0x07)
#define PM_WAKEUPSRC_RTC                    (0x08)
#define PM_WAKEUPSRC_RTC2                   (0x09)
#define PM_WAKEUPSRC_AVLINK                 (0x0A)
#define PM_WAKEUPSRC_VOICE                  (0x1A)
#define PM_WAKEUPSRC_VAD                    (0x2A)
#define PM_WAKEUPSRC_UART                   (0x0B)
#define PM_WAKEUPSRC_GPIO                   (0x0C)
#define PM_WAKEUPSRC_GPIO_WOWLAN            (0x1C)
#define PM_WAKEUPSRC_GPIO_WOBT              (0x2C)
#define PM_WAKEUPSRC_GPIO_WOEWBS            (0x3C)
#define PM_WAKEUPSRC_MHL                    (0x0D)
#define PM_WAKEUPSRC_WOL                    (0x0E)
#define PM_WAKEUPSRC_WOC                    (0x0F)

/* Using by identify function switch case. */
#define PM_KEY_DEFAULT                      (0x0000)
#define PM_KEY_POWER_DOWN                   (0x0001)
#define PM_KEY_SAR0                         (0x0002)
#define PM_KEY_LED                          (0x0003)
#define PM_KEY_IR_VERSION                   (0x0004)
#define PM_KEY_SAR1                         (0x0005)
#define PM_KEY_RTC                          (0x0006)
#define PM_KEY_BT                           (0x0012)
#define PM_KEY_EWBS                         (0x0013)
#define PM_KEY_USB                          (0x0014)
#define PM_KEY_BTW                          (0x0015)
#define PM_KEY_IR_NORMAL                    (0x0016)
#define PM_KEY_IR_EXT                       (0x0017)
#define PM_KEY_IR                           (0x0018)
#define PM_KEY_VAD                          (0x0019)

/* Using by struct PM_PowerDownCfg_t member u8PowerDownMode. */
#define E_PM_STANDBY                        (0x00)
#define E_PM_SLEEP                          (0x01)
#define E_PM_DEEP_SLEEP                     (0x02)
#define E_PM_NORMAL                         (0x03)

/* Using by struct PM_PowerDownCfg_t member u8WakeAddress. */
#define E_PM_LAST_TWOSTAGE_POWERDOWN        (3)

/* Using by struct PM_IrInfoCfg_t member u8IrVersion. */
#define IR_NO_INI_VERSION_NUM               (0x10)
#define IR_INI_VERSION_NUM                  (0x20)
#define IR_INI_VERSION_NUM_V30              (0x30)

/* Using by struct PM_IrInfoCfg_t. */
#define IR_SUPPORT_PM_KEYS_MAX              (32)
/* 1 byte protocol + 2 bytes headcode + 1 byte keycode. */
#define IR_POWER_KEY_NORMAL_SIZE_MAX        (4 * IR_SUPPORT_PM_KEYS_MAX)
/* 1 byte protocol + 4 bytes headcode + 2 byte keycode. */
#define IR_POWER_KEY_EXTEND_SIZE_MAX        (7 * IR_SUPPORT_PM_KEYS_MAX)

//-------------------------------------------------------------------------------------------------
//  Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_PM_FAIL       = 0,
    E_PM_OK         = 1,
    E_PM_TIMEOUT    = 2,
} PM_Result;

typedef enum
{
    /* STI. */
    E_PM_STATE_SUSPEND_PRE = 0x00,
    E_PM_STATE_SUSPEND,
    E_PM_STATE_SUSPEND_LATE,
    E_PM_STATE_SUSPEND_NOIRQ,
    E_PM_STATE_RESUME_NOIRQ,
    E_PM_STATE_RESUME_EARLY,
    E_PM_STATE_RESUME,
    E_PM_STATE_RESUME_COMPLETE,
    E_PM_STATE_REBOOT_NOTIFY,
    E_PM_STATE_POWER_OFF_PRE,
    E_PM_STATE_POWER_OFF,

    /* TODO :Remove. */
    E_PM_STATE_STORE_INFO,
    E_PM_STATE_SUSPEND_PREPARE,
    E_PM_STATE_POWER_OFF_AC,
    E_PM_STATE_POWER_OFF_DC,
    E_PM_STATE_POWER_ON_DC,
    E_PM_STATE_DC_OFF = E_PM_STATE_POWER_OFF_AC,
    E_PM_STATE_STR_OFF = E_PM_STATE_POWER_OFF_DC,
    E_PM_STATE_STR_ON = E_PM_STATE_POWER_ON_DC,
} PM_STATE;

typedef struct
{
    u8 bPmWakeEnableIR         : 1;
    u8 bPmWakeEnableSAR        : 1;
    u8 bPmWakeEnableGPIO0      : 1;
    u8 bPmWakeEnableGPIO1      : 1;
    u8 bPmWakeEnableUART1      : 1;
    u8 bPmWakeEnableSYNC       : 1;
    u8 bPmWakeEnableESYNC      : 1;
    u8 bPmWakeEnableRTC0       : 1;

    u8 bPmWakeEnableRTC1       : 1;
    u8 bPmWakeEnableDVI0       : 1;
    u8 bPmWakeEnableDVI2       : 1;
    u8 bPmWakeEnableCEC        : 1;
    u8 bPmWakeEnableAVLINK     : 1;
    u8 bPmWakeEnableMHL        : 1;
    u8 bPmWakeEnableWOL        : 1;
    u8 bPmWakeEnableCM4        : 1;

    u8 u8PmWakeIR[MAX_BUF_WAKE_IR];         //For PM IR Wake-up key define
    u8 u8PmWakeIR2[MAX_BUF_WAKE_IR2];       //For PM IR Wake-up key 2 define
    u8 u8PmWakeMACAddress[MAX_BUF_WAKE_MAC_ADDRESS];    //For PM WOL Wake-up MAC Addr define

    u8 u8PmStrMode;                         //For STR Power Mode, defined in EN_PM_STR_MODE
    u8 bLxCRCMiu[CRC_KERNEL_BUF];           //For STR CRC check, TRUE for MIU1, FALSE for MIU0
    u32 u32LxCRCAddress[CRC_KERNEL_BUF];    //For STR CRC check, memory address
    u32 u32LxCRCSize[CRC_KERNEL_BUF];       //For STR CRC check, memory size
    u8 u8PmWakeEnableWOWLAN;
    u8 u8PmWakeWOWLANPol;
    u8 u8PmWakeKeyShotCountAddr[PM_MAX_BUF_WAKE_IR_WAVEFORM_ADDR]; //For PM IR Wake-up key waveform define
    u8 u8HdmiByPass;
    u8 u8Reserved[PM_MAX_BUF_RESERVERD];
} PM_WakeCfg_t;

typedef struct
{
    u8 u8PowerDownMode;
    u8 u8WakeAddress;
} PM_PowerDownCfg_t ;

typedef struct
{
    u8 u8IrVersion;
    u8 u8NormalKeyNumber;
    u8 au8IrNormalKey[IR_POWER_KEY_NORMAL_SIZE_MAX];
    u8 u8ExtendKeyNumber;
    u8 au8IrExtendKey[IR_POWER_KEY_EXTEND_SIZE_MAX];
} PM_IrInfoCfg_t;

typedef struct
{
    u8 u8UpBnd;     //upper bound
    u8 u8LoBnd;     //low bound
} SAR_BndCfg;

typedef struct
{
    u8 u8SARChID;
    SAR_BndCfg tSARChBnd;
    u8 u8KeyLevelNum;
    u8 u8KeyThreshold[8];
    u8 u8KeyCode[8];
} SAR_RegCfg;

typedef struct
{
    u8 u8GpioNum;   /* Disable: 0xFF. */
    u8 u8Polarity;
} PM_WoBT_WakeCfg;

typedef struct
{
    u8 u8GpioNum;   /* Disable: 0xFF. */
    u8 u8Polarity;
} PM_WoEWBS_WakeCfg;

// TODO: Remove.
typedef struct
{
    PM_WakeCfg_t stPMWakeCfg;
    PM_PowerDownCfg_t stPMPowerDownCfg;
} PM_Cfg_t;

extern bool disable_background_wakeup;

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
/* Mmap buffer. */
void        MDrv_PM_Mapping(u16 u16Buf, unsigned long long ullAddr, unsigned long long ullSize);
void        MDrv_PM_Unmapping(u16 u16Buf);
/* Config from/to Local. */
void        MDrv_PM_Reset_Key(u16 u16Key);
ssize_t     MDrv_PM_Read_Key(u16 u16Key, const char *buf);
ssize_t     MDrv_PM_Write_Key(u16 u16Key, const char *buf, size_t size);
void        MDrv_PM_Show_Config(u16 u16Key);
/* Flush Local to Dram. */
void        MDrv_PM_Flush_Config(void);
/* Pm. */
PM_Result   MDrv_PM_Suspend(PM_STATE eState);
PM_Result   MDrv_PM_Resume(PM_STATE eState);
PM_Result   MDrv_PM_Reboot(PM_STATE eState);
PM_Result   MDrv_PM_Panic(void);
/* Other. */
u8          MDrv_PM_GetWakeupSource(void);
u8          MDrv_PM_GetPowerOnKey(void);
void        MDrv_PM_CleanPowerOnKey(void);
ssize_t     MDrv_PM_Set_IRCfg(const u8 *buf, size_t size);
void        MDrv_PM_WakeIrqMask(u8 mask);
u8          MDrv_PM_Get_PowerOffLed(void);
void        MDrv_PM_Set_PowerOffLed(u8 u8LedPad);
void        MDrv_PM_WDT_RST(void);

#ifdef      CONFIG_MSTAR_SYSFS_BACKLIGHT
PM_Result   MDrv_PM_TurnoffBacklight(void);
#endif

#ifdef      CONFIG_AMAZON_WOL
void        MDrv_PM_Show_PM_WOL_EN(void);
void        MDrv_PM_SetPM_WOL_EN(const char *buf);
#endif

/* Debug. */
void        MDrv_PM_Read_Debug(void);
void        MDrv_PM_Write_Debug(const char *buf, size_t size);
#endif
