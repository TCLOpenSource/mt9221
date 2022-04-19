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
#include "hal_lp_standby.h"
#include "mhal_lp_standby_mode_table.h"
#include "mdrv_gpio.h"

//-------------------------------------------------------------------------------------------------
// Define & data type
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define PM_RIU_REG_BASE                 0xFD000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
static ptrdiff_t _g_MapBase = 0;
#define PM_RIU_REG_BASE                 mstar_pm_base
#endif

#if defined(CONFIG_MSTAR_STANDBY_TABLE_SUPPORT)
MS_U8 *u8StandbyModeBuf = NULL;
#endif
#if defined(CONFIG_MSTAR_STANDBY_INV_TABLE_SUPPORT)
MS_U8 *u8StandbyModeInvBuf = NULL;
#endif
#if defined(CONFIG_MSTAR_STANDBY_MIU_TABLE_SUPPORT)
MS_U8 *u8StandbyModeMiuBuf = NULL;
#endif
#if defined(CONFIG_MSTAR_STANDBY_PD_TABLE_SUPPORT)
MS_U8 *u8StandbyModePdBuf = NULL;
#endif
#if defined(CONFIG_MSTAR_STANDBY_PRE_SETTING_TABLE_SUPPORT)
MS_U8 *u8StandbyModePreSettingBuf = NULL;
#endif

static u8 HAL_Standby_ReadByte(u32 u32RegAddr)
{
    return ((volatile u8*)(_g_MapBase))[(u32RegAddr << 1) - (u32RegAddr & 1)];
}

static u16 HAL_Standby_Read2Byte(u32 u32RegAddr)
{
    return ((volatile u16*)(_g_MapBase))[u32RegAddr];
}

static void HAL_Standby_Write2Byte(u32 u32RegAddr, u16 u16Val)
{
    if (!u32RegAddr)
    {
        MSTAR_STANDBY_ERR("Invalid reg addr: 0x%08x\n", u32RegAddr);
        return;
    }

    ((volatile MS_U16*)(_g_MapBase))[u32RegAddr] = u16Val;
}

static void HAL_Standby_WriteByte(u32 u32RegAddr, u8 u8Val)
{
    if (!u32RegAddr)
    {
        MSTAR_STANDBY_ERR("Invalid reg addr: 0x%08x\n", u32RegAddr);
        return;
    }

    ((volatile u8*)(_g_MapBase))[(u32RegAddr << 1) - (u32RegAddr & 1)] = u8Val;
}

void HAL_Standby_RegisterSet(STANDBY_MODE_TABLE RegSetting)
{
    MS_U32 addr = RegSetting.u32RegAddr;
    MS_U16 value = RegSetting.u16RegData;
    MS_U16 valueExt = RegSetting.u16RegDataExt;
    E_REG_HANDLE handle = RegSetting.handle;
    E_REG_SIZE size = RegSetting.size;

    switch (handle)
    {
        case REG_MASK:
            if (size == REG_2Bytes)
                HAL_Standby_Write2Byte(addr, (HAL_Standby_Read2Byte(addr) & (~value)) | valueExt);
            else
                HAL_Standby_WriteByte(addr, (HAL_Standby_ReadByte(addr) & ~((MS_U8)(value & 0xFF))) | (MS_U8)(valueExt & (value & 0xFF)));
            break;
        case REG_SET:
            if (size == REG_2Bytes)
                HAL_Standby_Write2Byte(addr, value);
            else
                HAL_Standby_WriteByte(addr, (MS_U8)(value & 0xFF));
            break;
        case REG_TOGGLE:
            if (size == REG_2Bytes)
            {
                HAL_Standby_Write2Byte(addr, HAL_Standby_Read2Byte(addr) | value);
                HAL_Standby_Write2Byte(addr, HAL_Standby_Read2Byte(addr) & (~value));
            }
            else
            {
                HAL_Standby_WriteByte(addr, HAL_Standby_ReadByte(addr) | (MS_U8)(value & 0xFF));
                HAL_Standby_WriteByte(addr, HAL_Standby_ReadByte(addr) & (MS_U8)((~value) & 0xFF));
            }
            break;
        case REG_TOGGLE_RESTORE:
            if (size == REG_2Bytes)
            {
                HAL_Standby_Write2Byte(addr, HAL_Standby_Read2Byte(addr) | value);
                HAL_Standby_Write2Byte(addr, HAL_Standby_Read2Byte(addr) & (~value));
                HAL_Standby_Write2Byte(addr, HAL_Standby_Read2Byte(addr) | value);
            }
            else
            {
                HAL_Standby_WriteByte(addr, HAL_Standby_ReadByte(addr) | (MS_U8)(value & 0xFF));
                HAL_Standby_WriteByte(addr, HAL_Standby_ReadByte(addr) & (MS_U8)((~value) & 0xFF));
                HAL_Standby_WriteByte(addr, HAL_Standby_ReadByte(addr) | (MS_U8)(value & 0xFF));
            }
            break;
    }
}

static bool HAL_Standby_IsFirstIndex(const STANDBY_MODE_TABLE *table, int index, STANDBY_MODE_TABLE RegSetting)
{
    int i;

    for (i = 0; i <= index; i++)
    {
        if ((table[i].u32RegAddr == RegSetting.u32RegAddr) && (table[i].u16RegData == RegSetting.u16RegData))
        {
            if (i == index)
                return true;
            else
                return false;
        }
    }

    return false;
}

static int HAL_Standby_PrepareBuf(void)
{
    if (_g_MapBase <= 0)
        _g_MapBase = mstar_pm_base;

#if defined(CONFIG_MSTAR_STANDBY_TABLE_SUPPORT)
    if (u8StandbyModeBuf == NULL)
    {
        u8StandbyModeBuf = kmalloc_array(STANDBY_MODE_BUF_SIZE, sizeof(MS_U8), GFP_KERNEL);
        if (u8StandbyModeBuf == NULL)
        {
            MSTAR_STANDBY_ERR("kmalloc table buffer fail\n");
            return -ENOMEM;
        }
        memset(u8StandbyModeBuf, 0x0, STANDBY_MODE_BUF_SIZE * sizeof(MS_U8));
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_INV_TABLE_SUPPORT)
    if (u8StandbyModeInvBuf == NULL)
    {
        u8StandbyModeInvBuf = kmalloc_array(STANDBY_MODE_INV_BUF_SIZE, sizeof(MS_U8), GFP_KERNEL);
        if (u8StandbyModeInvBuf == NULL)
        {
            MSTAR_STANDBY_ERR("kmalloc invert table buffer fail\n");
            return -ENOMEM;
        }
        memset(u8StandbyModeInvBuf, 0x0, STANDBY_MODE_INV_BUF_SIZE * sizeof(MS_U8));
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_MIU_TABLE_SUPPORT)
    if (u8StandbyModeMiuBuf == NULL)
    {
        u8StandbyModeMiuBuf = kmalloc_array(STANDBY_MODE_MIU_BUF_SIZE, sizeof(MS_U8), GFP_KERNEL);
        if (u8StandbyModeMiuBuf == NULL)
        {
            MSTAR_STANDBY_ERR("kmalloc miu str table buffer fail\n");
            return -ENOMEM;
        }
        memset(u8StandbyModeMiuBuf, 0x0, STANDBY_MODE_MIU_BUF_SIZE * sizeof(MS_U8));
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_PRE_SETTING_TABLE_SUPPORT)
    if (u8StandbyModePreSettingBuf == NULL)
    {
        u8StandbyModePreSettingBuf = kmalloc_array(STANDBY_MODE_PRE_SETTING_BUF_SIZE, sizeof(MS_U8), GFP_KERNEL);
        if (u8StandbyModePreSettingBuf == NULL)
        {
            MSTAR_STANDBY_ERR("kmalloc pre-setting table buffer fail\n");
            return -ENOMEM;
        }
        memset(u8StandbyModePreSettingBuf, 0x0, STANDBY_MODE_PRE_SETTING_BUF_SIZE * sizeof(MS_U8));
    }
#endif

    return 0;
}

static void HAL_Standby_ReleaseBuf(void)
{
#if defined(CONFIG_MSTAR_STANDBY_TABLE_SUPPORT)
    if (u8StandbyModeBuf != NULL)
    {
        kfree(u8StandbyModeBuf);
        u8StandbyModeBuf = NULL;
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_INV_TABLE_SUPPORT)
    if (u8StandbyModeInvBuf != NULL)
    {
        kfree(u8StandbyModeInvBuf);
        u8StandbyModeInvBuf = NULL;
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_MIU_TABLE_SUPPORT)
    if (u8StandbyModeMiuBuf != NULL)
    {
        kfree(u8StandbyModeMiuBuf);
        u8StandbyModeMiuBuf = NULL;
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_PRE_SETTING_TABLE_SUPPORT)
    if (u8StandbyModePreSettingBuf != NULL)
    {
        kfree(u8StandbyModePreSettingBuf);
        u8StandbyModePreSettingBuf = NULL;
    }
#endif
}

static void HAL_Standby_SetupTable(const STANDBY_MODE_TABLE *StandbyModeTable, int Tablesize, MS_U8 *SaveBuf)
{
    int i;
    MS_U16 index = 0;
    MS_U16 u16RegVal = 0;

    for (i = 0; i < Tablesize; i++)
    {
        if (StandbyModeTable[i].u32RegAddr == 0)
        {
            MSTAR_STANDBY_WARN("ignore invalid address(index: %d)\n", i);
            continue;
        }
        if (HAL_Standby_IsFirstIndex(StandbyModeTable, i, StandbyModeTable[i]))
        {
            // Backup register settings
            if (StandbyModeTable[i].size == REG_2Bytes)
            {
                u16RegVal = HAL_Standby_Read2Byte(StandbyModeTable[i].u32RegAddr);
                SaveBuf[index++] = (u16RegVal & 0xFF00) >> 8;
                SaveBuf[index++] = u16RegVal & 0xFF;
                MSTAR_STANDBY_DBG("save addr = 0x%0x, val = 0x%04x\n", StandbyModeTable[i].u32RegAddr, u16RegVal);
            }
            else
            {
                u16RegVal = HAL_Standby_ReadByte(StandbyModeTable[i].u32RegAddr);
                SaveBuf[index++] = u16RegVal & 0xFF;
                MSTAR_STANDBY_DBG("save addr = 0x%0x, val = 0x%02x\n", StandbyModeTable[i].u32RegAddr, u16RegVal);
            }
        }

        // Ignore toggle
        if ((StandbyModeTable[i].handle == REG_MASK) || (StandbyModeTable[i].handle == REG_SET))
            HAL_Standby_RegisterSet(StandbyModeTable[i]);
    }
#if defined(MSTAR_STANDBY_DEBUG)
    // For debug
    HAL_Standby_WriteByte(MSTAR_STANDBY_DEBUG_REG, 0x0);
#endif
}

void HAL_Standby_Setup(void)
{
    if (HAL_Standby_PrepareBuf())
        return;

    // Disable backlight
    MDrv_GPIO_Set_Low(MSTAR_STANDBY_BACKLIGHT_GPIO);

#if defined(CONFIG_MSTAR_STANDBY_PRE_SETTING_TABLE_SUPPORT)
    HAL_Standby_SetupTable(standby_mode_pre_setting_table, STANDBY_MODE_PRE_SETTING_TABLE_SIZE, u8StandbyModePreSettingBuf);
#endif

#if defined(CONFIG_MSTAR_STANDBY_TABLE_SUPPORT)
    HAL_Standby_SetupTable(standby_mode_table, STANDBY_MODE_TABLE_SIZE, u8StandbyModeBuf);
#endif

#if defined(CONFIG_MSTAR_STANDBY_INV_TABLE_SUPPORT)
    HAL_Standby_SetupTable(standby_mode_inv_restore_table, STANDBY_MODE_INV_TABLE_SIZE, u8StandbyModeInvBuf);
#endif

#if defined(CONFIG_MSTAR_STANDBY_MIU_TABLE_SUPPORT)
    HAL_Standby_SetupTable(standby_mode_miu_str_table, STANDBY_MODE_MIU_TABLE_SIZE, u8StandbyModeMiuBuf);
#endif
}

static void HAL_Standby_RestoreTable(const STANDBY_MODE_TABLE *StandbyModeTable, int TableSize, MS_U8 *SaveBuf, int BufSize, bool IsInvTable)
{
    int i;
    MS_U16 index = 0;
    MS_U16 u16RegVal = 0;
    STANDBY_MODE_TABLE RegSetting;

    if (IsInvTable == false)
    {
        // Restore register settings
        for (i = 0; i < TableSize; i++)
        {
            if (StandbyModeTable[i].u32RegAddr == 0)
            {
                MSTAR_STANDBY_WARN("ignore invalid address(index: %d)\n", i);
                continue;
            }
            if (HAL_Standby_IsFirstIndex(StandbyModeTable, i, StandbyModeTable[i]))
            {
                if (StandbyModeTable[i].size == REG_2Bytes)
                {
                    u16RegVal = SaveBuf[index++] << 8;
                    u16RegVal += SaveBuf[index++];
                    MSTAR_STANDBY_DBG("Restore addr = 0x%0x, val = 0x%04x\n", StandbyModeTable[i].u32RegAddr, u16RegVal);
                }
                else
                {
                    u16RegVal = SaveBuf[index++];
                    MSTAR_STANDBY_DBG("Restore addr = 0x%0x, val = 0x%02x\n", StandbyModeTable[i].u32RegAddr, u16RegVal);
                }
            }

#if defined(MSTAR_STANDBY_DEBUG)
            // For debug
            HAL_Standby_WriteByte(MSTAR_STANDBY_DEBUG_REG, i);
#endif

            memset(&RegSetting, 0x0, sizeof(STANDBY_MODE_TABLE));
            RegSetting.u32RegAddr = StandbyModeTable[i].u32RegAddr;
            RegSetting.handle = StandbyModeTable[i].handle;
            RegSetting.size = StandbyModeTable[i].size;
            if (StandbyModeTable[i].handle == REG_MASK)
            {
                RegSetting.u16RegData = StandbyModeTable[i].u16RegData;
                RegSetting.u16RegDataExt = u16RegVal;
            }
            else
            {
                RegSetting.u16RegData = u16RegVal;
            }

            HAL_Standby_RegisterSet(RegSetting);
            udelay(StandbyModeTable[i].u32DelayUs);
        }
    }
    else
    {
        index = BufSize - 1;
        for (i = TableSize - 1; i >= 0; i--)
        {
            if (StandbyModeTable[i].u32RegAddr == 0)
            {
                MSTAR_STANDBY_WARN("ignore invalid address(index: %d)\n", i);
                continue;
            }
            if (HAL_Standby_IsFirstIndex(StandbyModeTable, i, StandbyModeTable[i]))
            {
                if (StandbyModeTable[i].size == REG_2Bytes)
                {
                    u16RegVal = SaveBuf[index--];
                    u16RegVal += SaveBuf[index--] << 8;
                    MSTAR_STANDBY_DBG("Restore addr = 0x%0x, val = 0x%04x\n", StandbyModeTable[i].u32RegAddr, u16RegVal);
                }
                else
                {
                    u16RegVal = SaveBuf[index--];
                    MSTAR_STANDBY_DBG("Restore addr = 0x%0x, val = 0x%02x\n", StandbyModeTable[i].u32RegAddr, u16RegVal);
                }
            }

#if defined(MSTAR_STANDBY_DEBUG)
            // For debug
            HAL_Standby_WriteByte(MSTAR_STANDBY_DEBUG_REG, i);
#endif

            memset(&RegSetting, 0x0, sizeof(STANDBY_MODE_TABLE));
            RegSetting.u32RegAddr = StandbyModeTable[i].u32RegAddr;
            RegSetting.handle = StandbyModeTable[i].handle;
            RegSetting.size = StandbyModeTable[i].size;
            if (StandbyModeTable[i].handle == REG_MASK)
            {
                RegSetting.u16RegData = StandbyModeTable[i].u16RegData;
                RegSetting.u16RegDataExt = u16RegVal;
            }
            else
            {
                RegSetting.u16RegData = u16RegVal;
            }

            HAL_Standby_RegisterSet(RegSetting);
            udelay(StandbyModeTable[i].u32DelayUs);
        }
    }
}

void HAL_Standby_Restore(void)
{
#if defined(CONFIG_MSTAR_STANDBY_MIU_TABLE_SUPPORT)
    HAL_Standby_RestoreTable(standby_mode_miu_str_table, STANDBY_MODE_MIU_TABLE_SIZE, u8StandbyModeMiuBuf,
                                STANDBY_MODE_MIU_BUF_SIZE, false);
#endif

#if defined(CONFIG_MSTAR_STANDBY_INV_TABLE_SUPPORT)
    HAL_Standby_RestoreTable(standby_mode_inv_restore_table, STANDBY_MODE_INV_TABLE_SIZE, u8StandbyModeInvBuf,
                                STANDBY_MODE_INV_BUF_SIZE, true);
#endif

#if defined(CONFIG_MSTAR_STANDBY_TABLE_SUPPORT)
    HAL_Standby_RestoreTable(standby_mode_table, STANDBY_MODE_TABLE_SIZE, u8StandbyModeBuf, STANDBY_MODE_BUF_SIZE,
                                false);
#endif

#if defined(CONFIG_MSTAR_STANDBY_RST_TABLE_SUPPORT)
    // SW Reset
    for (i = 0; i < STANDBY_MODE_RST_TABLE_SIZE; i++)
    {
        HAL_Standby_RegisterSet(standby_mode_reset_table[i].u32RegAddr, standby_mode_reset_table[i].u16RegData,
                                standby_mode_reset_table[i].handle, standby_mode_reset_table[i].size);
    }
#endif

#if defined(CONFIG_MSTAR_STANDBY_PRE_SETTING_TABLE_SUPPORT)
    HAL_Standby_RestoreTable(standby_mode_pre_setting_table, STANDBY_MODE_PRE_SETTING_TABLE_SIZE,
                                u8StandbyModePreSettingBuf, STANDBY_MODE_PRE_SETTING_BUF_SIZE, false);
#endif

    // Enable backlight
    MDrv_GPIO_Set_High(MSTAR_STANDBY_BACKLIGHT_GPIO);
    HAL_Standby_ReleaseBuf();
}

#if defined(CONFIG_MSTAR_STANDBY_PD_TABLE_SUPPORT)
int HAL_Standby_Suspend(void)
{
    // Prepare buffer
    if (u8StandbyModePdBuf == NULL)
    {
        u8StandbyModePdBuf = kmalloc_array(STANDBY_MODE_PD_BUF_SIZE, sizeof(MS_U8), GFP_KERNEL);
        if (u8StandbyModePdBuf == NULL)
        {
            MSTAR_STANDBY_ERR("kmalloc power down table buffer fail\n");
            return -ENOMEM;
        }
    }

    HAL_Standby_SetupTable(standby_mode_pd_table, STANDBY_MODE_PD_TABLE_SIZE, u8StandbyModePdBuf);

    return 0;
}

void HAL_Standby_Resume(void)
{
    HAL_Standby_RestoreTable(standby_mode_pd_table, STANDBY_MODE_PD_TABLE_SIZE, u8StandbyModePdBuf, false);

    if (u8StandbyModePdBuf != NULL)
    {
        kfree(u8StandbyModePdBuf);
        u8StandbyModePdBuf = NULL;
    }
}
#endif
