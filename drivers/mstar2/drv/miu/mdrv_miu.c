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
/// @file   Mdrvl_miu.c
/// @brief  MIU Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/of.h>
#endif
#include "mdrv_miu.h"
#include "mhal_miu.h"
#include "chip_int.h"

//-------------------------------------------------------------------------------------------------
//  Function prototype with weak symbol
//-------------------------------------------------------------------------------------------------

MS_BOOL HAL_MIU_Kernel_Protect(MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_PHY phy64Start, MS_PHY phy64End, MS_BOOL bSetFlag) __attribute__((weak));

MS_BOOL HAL_MIU_Kernel_ParseOccupiedResource(void) __attribute__((weak));
MS_U32* HAL_MIU_Kernel_GetDefaultClientID_KernelProtect(void) __attribute__((weak));

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------
//static DEFINE_SPINLOCK(miu_lock);
static MS_BOOL bMiuInit = FALSE;
static MS_U32 mstar_debug = E_MIU_DBGLV_ERR;
static MS_U32 miu_hit_panic = 0;
static MS_U32 u32MIUHitCount = 0;
static MS_U32 u32HitInterrupt = 0;
static MS_U32 u32HitIRQDynCtrl = 1;
static MS_U32 u32HitMaxCount = 100;
static MS_BOOL bGotHitPanicInBootargs = FALSE;

//-------------------------------------------------------------------------------------------------
//  Local functions
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: MDrv_MIU_Kernel_Init
/// @brief \b Function  \b Description: parse occupied resource to software structure
/// @param None         \b IN :
/// @param None         \b OUT :
/// @param MS_BOOL      \b RET
/// @param None         \b GLOBAL :
////////////////////////////////////////////////////////////////////////////////

MS_BOOL MDrv_MIU_Kernel_Init(void)
{
    MS_BOOL ret = FALSE;

    if(bMiuInit == TRUE)
        return TRUE;

    /* Parse the used client ID in hardware into software data structure */
    if(HAL_MIU_Kernel_ParseOccupiedResource)
    {
        ret = HAL_MIU_Kernel_ParseOccupiedResource();
    }
    else
    {
        ret = FALSE;
    }

    bMiuInit = TRUE;

    return ret;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: MDrv_MIU_Kernel_GetDefaultClientID_KernelProtect()
/// @brief \b Function \b Description:  Get default client id array pointer for protect kernel
/// @param <RET>           \b     : The pointer of Array of client IDs
////////////////////////////////////////////////////////////////////////////////
MS_U32* MDrv_MIU_Kernel_GetDefaultClientID_KernelProtect(void)
{
    if( HAL_MIU_Kernel_GetDefaultClientID_KernelProtect )
    {
        return HAL_MIU_Kernel_GetDefaultClientID_KernelProtect();
    }
    else
    {
        return NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: MDrv_MIU_Kernel_Protect()
/// @brief \b Function \b Description:  Enable/Disable MIU Protection mode
/// @param u8Blockx        \b IN     : MIU Block to protect (0 ~ 3)
/// @param *pu8ProtectId   \b IN     : Allow specified client IDs to write
/// @param u32Start        \b IN     : Starting address(bus address)
/// @param u32End          \b IN     : End address(bus address)
/// @param bSetFlag        \b IN     : Disable or Enable MIU protection
///                                      - -Disable(0)
///                                      - -Enable(1)
/// @param <OUT>           \b None    :
/// @param <RET>           \b None    :
/// @param <GLOBAL>        \b None    :
////////////////////////////////////////////////////////////////////////////////
MS_BOOL MDrv_MIU_Kernel_Protect( MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_PHY u64BusStart, MS_PHY u64BusEnd, MS_BOOL bSetFlag)
{
    MS_BOOL Result = FALSE;

    /*Case of former MIU protect*/
    if((u8Blockx >= E_PROTECT_0))
    {
        if(HAL_MIU_Kernel_Protect)
        {
            Result = HAL_MIU_Kernel_Protect(u8Blockx, pu32ProtectId, u64BusStart, u64BusEnd, bSetFlag);
        }
    }

    return Result;
}

MS_BOOL MDrv_MIU_Save(void)
{
    MS_BOOL Result = FALSE;
    Result = HAL_MIU_Save();
    return Result;
}

MS_BOOL MDrv_MIU_Restore(void)
{
    MS_BOOL Result = FALSE;
    Result = HAL_MIU_Restore();
    return Result;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: MDrv_MIU_Kernel_GetProtectInfo()
/// @brief \b Function  \b Description:  This function for querying client ID info
/// @param u8MiuDev     \b IN   : select MIU0 or MIU1
/// @param eClientID    \b IN   : Client ID
/// @param pInfo        \b OUT  : Client Info
/// @param None \b RET:   0: Fail 1: Ok
////////////////////////////////////////////////////////////////////////////////
MS_BOOL MDrv_MIU_Kernel_GetProtectInfo(MS_U8 u8MiuDev, MIU_PortectInfo *pInfo)
{
    return HAL_MIU_Kernel_GetProtectInfo(u8MiuDev, pInfo);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: MDrv_MIU_Slits()
/// @brief \b Function \b Description:  Enable/Disable MIU Slit
/// @param u8Blockx        \b IN     : MIU Block to protect (E_MIU_SLIT_0)
/// @param u32Start        \b IN     : Starting address(bus address)
/// @param u32End          \b IN     : End address(bus address)
/// @param bSetFlag        \b IN     : Disable or Enable MIU protection
///                                      - -Disable(0)
///                                      - -Enable(1)
/// @param <OUT>           \b None    :
/// @param <RET>           \b None    :
/// @param <GLOBAL>        \b None    :
////////////////////////////////////////////////////////////////////////////////
MS_BOOL MDrv_MIU_Slits(MS_U8 u8Blockx, MS_PHY u64SlitsStart, MS_PHY u64SlitsEnd, MS_BOOL bSetFlag)
{
    MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "Not Support MDrv_MIU_Slits\n");
    return TRUE;
}

MS_BOOL MDrv_MIU_Set_ADCDMA_Protect(MS_PHY start_addr)
{
    return HAL_MIU_Set_ADCDMA_Protect(start_addr);
}
EXPORT_SYMBOL(MDrv_MIU_Set_ADCDMA_Protect);

void MDrv_MIU_Remove_ADCDMA_Protect(void)
{
    HAL_MIU_Remove_ADCDMA_Protect();
}
EXPORT_SYMBOL(MDrv_MIU_Remove_ADCDMA_Protect);

#ifndef CONFIG_MSTAR_UTOPIA2K_BUILTIN
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: MDrv_MIU_Dram_ReadSize()
/// @brief \b Function \b Description:  Get DRAM size
/// @param MiuID       \b IN     : MIU to be queried
/// @param pDramSize   \b OUT    : DRAM size enumeration
////////////////////////////////////////////////////////////////////////////////
MS_BOOL MDrv_MIU_Dram_ReadSize(MS_U8 MiuID, MIU_DDR_SIZE *pDramSize)
{
    return HAL_MIU_Kernel_Dram_ReadSize(MiuID, pDramSize);
}

EXPORT_SYMBOL(MDrv_MIU_Dram_ReadSize);
#endif

#if defined(MIU_HIT_INTERRUPT) && (MIU_HIT_INTERRUPT == 1)
irqreturn_t MDrv_MIU_ProtectInterrupt(int irq, void *devid);

irqreturn_t MDrv_MIU_ProtectInterrupt(int irq, void *devid)
{
    MS_U8 u8MiuDev;
    MIU_PortectInfo pInfo;

    u32MIUHitCount++;
    if( u32MIUHitCount > u32HitMaxCount)
    {
        u32HitInterrupt = 0;
        disable_irq_nosync(MIU_IRQ);
        return IRQ_HANDLED;
    }

    for(u8MiuDev = 0; u8MiuDev < MIU_MAX_DEVICE; u8MiuDev++)
    {
        HAL_MIU_Kernel_GetProtectInfo(u8MiuDev, &pInfo);
    }

    return IRQ_HANDLED;
}
#endif

static int mstar_miu_drv_probe(struct platform_device *pdev)
{
    if( MDrv_MIU_Kernel_Init() == FALSE )
    {
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU Init fail\n");
    }

    if(u32HitIRQDynCtrl == 1)
    {
#if defined(MIU_HIT_INTERRUPT) && (MIU_HIT_INTERRUPT == 1)
        if (request_irq(MIU_IRQ, MDrv_MIU_ProtectInterrupt, SA_INTERRUPT, "MIU_IRQ", NULL))
        {
            MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU IRQ registration ERROR\n");
        }
        else
        {
            u32HitInterrupt = 1;
            MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU IRQ registration OK\n");
        }
#endif
    }
    miu_hit_panic = HAL_MIU_Get_Hitkernelpanic();

    return 0;
}

static int mstar_miu_drv_remove(struct platform_device *pdev)
{
    return 0;
}

static int mstar_miu_drv_suspend(struct platform_device *dev, pm_message_t state)
{
#ifndef CONFIG_MSTAR_CMAPOOL
    MDrv_MIU_Kernel_Init();
    HAL_MIU_Save();
#endif
    return 0;
}

static int mstar_miu_drv_resume(struct platform_device *dev)
{
#ifndef CONFIG_MSTAR_CMAPOOL
    HAL_MIU_Restore();
#endif
    return 0;
}

#if defined (CONFIG_OF)
static struct of_device_id mstarmiu_of_device_ids[] = {
         {.compatible = "mstar-miu"},
         {},
};
#endif

static struct platform_driver Mstar_miu_driver = {
    .probe      = mstar_miu_drv_probe,
    .remove     = mstar_miu_drv_remove,
    .suspend    = mstar_miu_drv_suspend,
    .resume     = mstar_miu_drv_resume,
    .driver     = {
    .name       = "Mstar-miu",
#if defined(CONFIG_OF)
    .of_match_table = mstarmiu_of_device_ids,
#endif
    .owner  = THIS_MODULE,
    }
};

static int __init mstar_miu_drv_init_module(void)
{
//    retval = platform_driver_register(&Mstar_miu_driver);
    int ret = 0;
    ret = platform_driver_register(&Mstar_miu_driver);
    if (ret)
    {
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "Register Mstar MIU Platform Driver Failed!\n");
    }
    return ret;
}

static void __exit mstar_miu_drv_exit_module(void)
{
    platform_driver_unregister(&Mstar_miu_driver);
}

static int __init MIU_Set_DebugLevel(char *str)
{
    if(strcmp(str, "0") == 0)
    {
        mstar_debug = E_MIU_DBGLV_NONE;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_NONE);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : SILENT");
    }
    else if(strcmp(str, "1") == 0)
    {
        mstar_debug = E_MIU_DBGLV_EMERG;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_EMERG);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : EMERGENCE");
    }
    else if(strcmp(str, "2") == 0)
    {
        mstar_debug = E_MIU_DBGLV_ALERT;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_ALERT);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : ALERT");
    }
    else if(strcmp(str, "3") == 0)
    {
        mstar_debug = E_MIU_DBGLV_CRIT;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_CRIT);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : CRITICAL");
    }
    else if(strcmp(str, "4") == 0)
    {
        mstar_debug = E_MIU_DBGLV_ERR;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_ERR);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : ERROR");
    }
    else if(strcmp(str, "5") == 0)
    {
        mstar_debug = E_MIU_DBGLV_WARNING;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_WARNING);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : WARNING");
    }
    else if(strcmp(str, "6") == 0)
    {
        mstar_debug = E_MIU_DBGLV_NOTICE;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_NOTICE);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : NOTICE");
    }
    else if(strcmp(str, "7") == 0)
    {
        mstar_debug = E_MIU_DBGLV_INFO;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_INFO);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : INFORMATION");
    }
    else if(strcmp(str, "8") == 0)
    {
        mstar_debug = E_MIU_DBGLV_DEBUG;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_DEBUG);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : DEBUG");
    }
    return 0;
}
static int __init MIU_Enable_Hitkernelpanic(char *str)
{
    if(strcmp(str, "ON") == 0 || strcmp(str, "1") == 0)
    {
        miu_hit_panic = 1;
        HAL_MIU_Enable_Hitkernelpanic(miu_hit_panic);
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU HIT KERNEL PANIC=ON");
    }
    else if(strcmp(str, "OFF") == 0 || strcmp(str, "0") == 0)
    {
        miu_hit_panic = 0;
        HAL_MIU_Enable_Hitkernelpanic(miu_hit_panic);
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU HIT KERNEL PANIC=OFF");
    }
    else
    {
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "Set KERNEL PANIC FAIL");
    }

    return 0;
}
static int __init MIU_Enable_HitInterupt(char *str)
{
    if(strcmp(str, "OFF") == 0)
    {
        u32HitIRQDynCtrl = 0;
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU HIT KERNEL INTERRUPT=OFF");
    }
    return 0;
}
static int __init MIU_Hit_MaxCount(char *str)
{
    u32HitMaxCount = (MS_U32)simple_strtol(str, NULL, 10);
    MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "SET MIU HIT MAX COUNT TO %lu", (unsigned long int)u32HitMaxCount);
    return 0;
}
early_param("MIU_DEBUG_LEVEL", MIU_Set_DebugLevel);
early_param("MIU_HIT_PANIC", MIU_Enable_Hitkernelpanic);
early_param("MIU_HIT_INTERRUPT", MIU_Enable_HitInterupt);
early_param("MIU_HIT_MAX_COUNT", MIU_Hit_MaxCount);

module_init(mstar_miu_drv_init_module);
module_exit(mstar_miu_drv_exit_module);

static int set_mstar_debug(const char *val, const struct kernel_param *kp)
{
    char valcp[16];
    char *s;

    strncpy(valcp, val, 16);
    valcp[15] = '\0';

    s = strstrip(valcp);

    if(strcmp(s, "0") == 0)
    {
        mstar_debug = E_MIU_DBGLV_NONE;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_NONE);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : SILENT");
    }
    else if(strcmp(s, "1") == 0)
    {
        mstar_debug = E_MIU_DBGLV_EMERG;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_EMERG);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : EMERGENCE");
    }
    else if(strcmp(s, "2") == 0)
    {
        mstar_debug = E_MIU_DBGLV_ALERT;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_ALERT);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : ALERT");
    }
    else if(strcmp(s, "3") == 0)
    {
        mstar_debug = E_MIU_DBGLV_CRIT;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_CRIT);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : CRITICAL");
    }
    else if(strcmp(s, "4") == 0)
    {
        mstar_debug = E_MIU_DBGLV_ERR;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_ERR);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : ERROR");
    }
    else if(strcmp(s, "5") == 0)
    {
        mstar_debug = E_MIU_DBGLV_WARNING;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_WARNING);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : WARNING");
    }
    else if(strcmp(s, "6") == 0)
    {
        mstar_debug = E_MIU_DBGLV_NOTICE;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_NOTICE);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : NOTICE");
    }
    else if(strcmp(s, "7") == 0)
    {
        mstar_debug = E_MIU_DBGLV_INFO;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_INFO);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : INFORMATION");
    }
    else if(strcmp(s, "8") == 0)
    {
        mstar_debug = E_MIU_DBGLV_DEBUG;
        HAL_MIU_Set_DebugLevel(E_MIU_DBGLV_DEBUG);
        MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU LOG LEVEL : DEBUG");
    }
    else
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "WRONG MIU LOG LEVEL");
    }

    return 0;
}

static int set_miu_hit_panic(const char *val, const struct kernel_param *kp)
{
    char valcp[16];
    char *s;

    strncpy(valcp, val, 16);
    valcp[15] = '\0';

    s = strstrip(valcp);

    if(strcmp(s, "0") == 0)
    {
        miu_hit_panic = 0;
        HAL_MIU_Enable_Hitkernelpanic(miu_hit_panic);
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU HIT KERNEL PANIC=OFF");
    }
    else if(strcmp(s, "1") == 0)
    {
        miu_hit_panic = 1;
        HAL_MIU_Enable_Hitkernelpanic(miu_hit_panic);
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU HIT KERNEL PANIC=ON");
    }
    else
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "WRONG MIU HIT PANIC PARAMETER");
    }

    return 0;
}

static int set_miu_hit_max_count(const char *val, const struct kernel_param *kp)
{
    char valcp[16];
    char *s;

    strncpy(valcp, val, 16);
    valcp[15] = '\0';

    s = strstrip(valcp);

    u32HitMaxCount = (MS_U32)simple_strtol(s, NULL, 10);

    MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "SET MIU HIT MAX COUNT TO %lu", (unsigned long int)u32HitMaxCount);

    return 0;
}

#if defined(MIU_HIT_INTERRUPT) && (MIU_HIT_INTERRUPT == 1)
static int set_miu_hit_interrupt(const char *val, const struct kernel_param *kp)
{
    char valcp[16];
    char *s;

    strncpy(valcp, val, 16);
    valcp[15] = '\0';

    s = strstrip(valcp);

    if(strcmp(s, "0") == 0)
    {
        if( u32HitInterrupt == 0 )
        {
            MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "HIT LOG INTERRUPT Already OFF\n");
        }
        else
        {
            free_irq(MIU_IRQ, NULL);
            u32HitInterrupt = 0;
            u32MIUHitCount = 0;
            MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "HIT LOG INTERRUPT OFF\n");
        }
    }
    else if(strcmp(s, "1") == 0)
    {
        if( u32HitInterrupt == 1 )
        {
            MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "HIT LOG INTERRUPT Already ON\n");
        }
        else
        {
            if( u32MIUHitCount > u32HitMaxCount)
            {
                free_irq(MIU_IRQ, NULL);
            }
            u32MIUHitCount = 0;

            if (request_irq(MIU_IRQ, MDrv_MIU_ProtectInterrupt, SA_INTERRUPT, "MIU_IRQ", NULL))
            {
                MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU IRQ registration ERROR\n");
            }
            else
            {
                u32HitInterrupt = 1;
                MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "MIU IRQ registration OK\n");
            }
            MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "HIT LOG INTERRUPT ON\n");
        }
    }
    else
    {
        MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "WRONG MIU HIT INTERRUPT PARAMETER\n");
    }

    return 0;
}

static const struct kernel_param_ops mstar_hit_interrupt_ops = {
    .set    = set_miu_hit_interrupt,
    .get    = param_get_int,
};
module_param_cb(miu_hit_interrupt, &mstar_hit_interrupt_ops, &u32HitInterrupt, 0644);

#endif

static const struct kernel_param_ops mstar_debug_ops = {
    .set    = set_mstar_debug,
    .get    = param_get_int,
};

static const struct kernel_param_ops miu_hit_panic_ops = {
    .set    = set_miu_hit_panic,
    .get    = param_get_int,
};

static const struct kernel_param_ops miu_hit_max_count_ops = {
    .set    = set_miu_hit_max_count,
    .get    = param_get_int,
};

module_param_cb(miu_debug_level, &mstar_debug_ops, &mstar_debug, 0644);
module_param_cb(miu_hit_panic, &miu_hit_panic_ops, &miu_hit_panic, 0644);
module_param_cb(miu_hit_max_count, &miu_hit_max_count_ops, &u32HitMaxCount, 0644);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("MIU driver");
MODULE_LICENSE("GPL");
