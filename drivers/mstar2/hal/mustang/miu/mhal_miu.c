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
/// @file   Mhal_mtlb.c
/// @brief  MTLB Driver
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include files
//-------------------------------------------------------------------------------------------------
#include <linux/irq.h>
#include <linux/printk.h>
#include "MsTypes.h"
#include "mdrv_types.h"
#include "mdrv_miu.h"
#include "regMIU.h"
#include "mhal_miu.h"
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include "mdrv_system.h"

//-------------------------------------------------------------------------------------------------
//  Define
//-------------------------------------------------------------------------------------------------
#define MIU_HAL_ERR(fmt, args...)   printk(KERN_ERR "miu hal error %s:%d" fmt,__FUNCTION__,__LINE__,## args)

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Macros
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Variable
//-------------------------------------------------------------------------------------------------
//static MS_U32 _gMIU_MapBase = 0xBF200000;      //default set to MIPS platfrom
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
static MS_U32 _gMIU_MapBase = 0xFD200000UL;   //default set to arm 32bit platfrom
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
static ptrdiff_t _gMIU_MapBase;
#endif

//-------------------------------------------------------------------------------------------------
//  MIU Debug Message Level
//-------------------------------------------------------------------------------------------------
static MS_U32 mstar_debug = E_MIU_DBGLV_ERR;
static MS_U32 miu_hit_panic = 0;

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_ReadByte
/// @brief \b Function  \b Description: read 1 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b MS_U8
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////
static MS_U8 HAL_MIU_ReadByte(MS_U32 u32RegAddr)
{
#if defined(CONFIG_ARM64)
	_gMIU_MapBase = (mstar_pm_base + 0x00200000UL);
#endif
    return ((volatile MS_U8*)(_gMIU_MapBase))[(u32RegAddr << 1) - (u32RegAddr & 1)];
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_Read4Byte
/// @brief \b Function  \b Description: read 2 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b MS_U16
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////
static MS_U16 HAL_MIU_Read2Byte(MS_U32 u32RegAddr)
{
#if defined(CONFIG_ARM64)
	_gMIU_MapBase = (mstar_pm_base + 0x00200000UL);
#endif
    return ((volatile MS_U16*)(_gMIU_MapBase))[u32RegAddr];
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_WriteByte
/// @brief \b Function  \b Description: write 1 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u8Val : 1 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok FALSE: Fail
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////
static MS_BOOL HAL_MIU_WriteByte(MS_U32 u32RegAddr, MS_U8 u8Val)
{
    if (!u32RegAddr)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

#if defined(CONFIG_ARM64)
	_gMIU_MapBase = (mstar_pm_base + 0x00200000UL);
#endif
    ((volatile MS_U8*)(_gMIU_MapBase))[(u32RegAddr << 1) - (u32RegAddr & 1)] = u8Val;
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_Write2Byte
/// @brief \b Function  \b Description: write 2 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u16Val : 2 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok FALSE: Fail
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////
static MS_BOOL HAL_MIU_Write2Byte(MS_U32 u32RegAddr, MS_U16 u16Val)
{
    if (!u32RegAddr)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

#if defined(CONFIG_ARM64)
	_gMIU_MapBase = (mstar_pm_base + 0x00200000UL);
#endif
    ((volatile MS_U16*)(_gMIU_MapBase))[u32RegAddr] = u16Val;
    return TRUE;
}

static void HAL_MIU_Write2BytesBit(MS_U32 u32RegOffset, MS_BOOL bEnable, MS_U16 u16Mask)
{
    MS_U16 val = HAL_MIU_Read2Byte(u32RegOffset);
    val = (bEnable) ? (val | u16Mask) : (val & ~u16Mask);
    HAL_MIU_Write2Byte(u32RegOffset, val);
}

#define GET_HIT_BLOCK(regval)       ((regval & 0xE0) >> 5)
#define GET_HIT_CLIENT(regval)      (regval >> 8)

MS_BOOL HAL_MIU_Kernel_GetProtectInfo(MS_U8 u8MiuDev, MIU_PortectInfo *pInfo)
{
    MS_U16 ret = 0;
    MS_U16 loaddr = 0;
    MS_U16 hiaddr = 0;
    MS_U32 u32Address = 0;
    MS_U32 u32Reg = (u8MiuDev) ? MIU1_REG_BASE : MIU_REG_BASE;

    if (!pInfo)
        return FALSE;

    ret = HAL_MIU_Read2Byte(u32Reg+REG_MIU_PROTECT_STATUS);
    loaddr = HAL_MIU_Read2Byte(u32Reg+REG_MIU_PROTECT_LOADDR);
    hiaddr = HAL_MIU_Read2Byte(u32Reg+REG_MIU_PROTECT_HIADDR);

    pInfo->bHit = false;

    if (REG_MIU_PROTECT_HIT_FALG & ret)
    {
        pInfo->bHit = TRUE;

        pInfo->u8Block = (MS_U8)GET_HIT_BLOCK(ret);
        pInfo->u8Group = (MS_U8)(GET_HIT_CLIENT(ret) >> 4);
        pInfo->u8ClientID = (MS_U8)(GET_HIT_CLIENT(ret) & 0x0F);
        u32Address = (MS_U32)((hiaddr << 16) | loaddr) ;
        u32Address = u32Address * MIU_PROTECT_ADDRESS_UNIT;

        if(printk_ratelimit() == TRUE)
        {
            MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "MIU%u Block:%u Group:%u ClientID:%u Hitted_Address:0x%x<->0x%x\n", u8MiuDev,
            pInfo->u8Block, pInfo->u8Group, pInfo->u8ClientID, u32Address, u32Address + MIU_PROTECT_ADDRESS_UNIT - 1);
        }

        BUG_ON((miu_hit_panic==1));

        //clear log
        HAL_MIU_Write2BytesBit(u32Reg+REG_MIU_PROTECT_STATUS, TRUE, REG_MIU_PROTECT_LOG_CLR);
        HAL_MIU_Write2BytesBit(u32Reg+REG_MIU_PROTECT_STATUS, FALSE, REG_MIU_PROTECT_LOG_CLR);
    }

    return TRUE;
}

MIU_Bank MIU_NormalBankInfo[MIU_MAX_DEVICE];
MIU_BWBank MIU_BWBankInfo[MIU_MAX_DEVICE][2];
//MIU_BWBankInfo[MIU_MAX_DEVICE][0] for ARB Base
//MIU_BWBankInfo[MIU_MAX_DEVICE][1] for ARBB Base

MS_BOOL HAL_MIU_Save(void)
{
    int index;
//protect ID enable
    for( index = 0; index < 4; index++ )
    {
        MIU_NormalBankInfo[0].u16ProtectIDEn[index] = HAL_MIU_Read2Byte(MIU_PROTECT0_ID_ENABLE+index*2); 
        MIU_NormalBankInfo[1].u16ProtectIDEn[index] = HAL_MIU_Read2Byte(MIU1_PROTECT0_ID_ENABLE+index*2);
    }
//protect ID
    for( index = 0; index < MIU_MAX_PROTECT_ID; index++ )
    {
        MIU_NormalBankInfo[0].u8GroupClient[index] = HAL_MIU_ReadByte(MIU_PROTECT0_ID0+index);
        MIU_NormalBankInfo[1].u8GroupClient[index] = HAL_MIU_ReadByte(MIU1_PROTECT0_ID0+index);
    }
//RQ
    for( index = 0; index < 0x40; index++ )
    {
        MIU_NormalBankInfo[0].u16RQ[index] = HAL_MIU_Read2Byte( MIU_RQ+index*2 );
        MIU_NormalBankInfo[1].u16RQ[index] = HAL_MIU_Read2Byte( MIU1_RQ+index*2 );
    }
//miu protect addr
    for( index = 0; index < 9; index++ )
    {
        MIU_NormalBankInfo[0].u16ProtectAddr[index] = HAL_MIU_Read2Byte(MIU_PROTECT0_START+index*2);
        MIU_NormalBankInfo[1].u16ProtectAddr[index] = HAL_MIU_Read2Byte(MIU1_PROTECT0_START+index*2);
    }
//After Protect ID & Addr, enable miu protect
    MIU_NormalBankInfo[0].u16ProtectEn = HAL_MIU_Read2Byte(MIU_PROTECT_EN_INTERNAL);
    MIU_NormalBankInfo[1].u16ProtectEn = HAL_MIU_Read2Byte(MIU1_PROTECT_EN);
//miu select
    for( index = 0; index < 6; index++ )
    {
        MIU_NormalBankInfo[0].u16SelMIU[index] = HAL_MIU_Read2Byte(REG_MIU_SEL0+index*2);
        MIU_NormalBankInfo[1].u16SelMIU[index] = HAL_MIU_Read2Byte(REG_MIU1_SEL0+index*2);
    }
//group priority
    MIU_NormalBankInfo[0].u16GroupPriority = HAL_MIU_Read2Byte(MIU_REG_BASE+REG_MIU_GROUP_PRIORITY);
    MIU_NormalBankInfo[1].u16GroupPriority = HAL_MIU_Read2Byte(MIU1_REG_BASE+REG_MIU_GROUP_PRIORITY);
    MIU_BWBankInfo[0][0].u16GroupPriority = HAL_MIU_Read2Byte( MIU_ARB_REG_BASE+REG_MIU_GROUP_PRIORITY );
    MIU_BWBankInfo[1][0].u16GroupPriority = HAL_MIU_Read2Byte( MIU1_ARB_REG_BASE+REG_MIU_GROUP_PRIORITY );

//BWRQ
    for( index = 0; index < 0x20; index++ )
    {
        MIU_BWBankInfo[0][0].u16BWRQ[index] = HAL_MIU_Read2Byte( MIU_ARB_REG_BASE+index*2 );
        MIU_BWBankInfo[0][1].u16BWRQ[index] = HAL_MIU_Read2Byte( MIU_ARBB_REG_BASE+index*2 );
        MIU_BWBankInfo[1][0].u16BWRQ[index] = HAL_MIU_Read2Byte( MIU1_ARB_REG_BASE+index*2 );
        MIU_BWBankInfo[1][1].u16BWRQ[index] = HAL_MIU_Read2Byte( MIU1_ARBB_REG_BASE+index*2 );
    }
//BW ctrl
    for( index = 0; index <= 0x0D; index++ )
    {
        MIU_BWBankInfo[0][0].u16BWCtrl[index] = HAL_MIU_Read2Byte( MIU_BW_CTRL+index*2 );
        MIU_BWBankInfo[1][0].u16BWCtrl[index] = HAL_MIU_Read2Byte( MIU1_BW_CTRL+index*2 );
    }
//BW select MIU
    for( index = 0; index < 2; index++ )
    {
        MIU_BWBankInfo[0][0].u16BWSelMIU[index] = HAL_MIU_Read2Byte( REG_MIU_SEL6+index*2 );
        MIU_BWBankInfo[1][0].u16BWSelMIU[index] = HAL_MIU_Read2Byte( REG_MIU1_SEL6+index*2 );
    }
    return TRUE;
}

MS_BOOL HAL_MIU_Restore(void)
{
    int index;

// First Disable miu protect
    HAL_MIU_Write2BytesBit(MIU_PROTECT_EN, FALSE, BIT3 | BIT2 | BIT1 | BIT0);
    HAL_MIU_Write2BytesBit(MIU1_PROTECT_EN, FALSE, BIT3 | BIT2 | BIT1 | BIT0);
//protect ID enable
    for( index = 0; index < 4; index++ )
    {
        HAL_MIU_Write2Byte( MIU_PROTECT0_ID_ENABLE+index*2, MIU_NormalBankInfo[0].u16ProtectIDEn[index] );
        HAL_MIU_Write2Byte( MIU1_PROTECT0_ID_ENABLE+index*2, MIU_NormalBankInfo[1].u16ProtectIDEn[index] );
    }
//protect ID
    for( index = 0; index < MIU_MAX_PROTECT_ID; index++ )
    {
        HAL_MIU_WriteByte( MIU_PROTECT0_ID0+index, MIU_NormalBankInfo[0].u8GroupClient[index] );
        HAL_MIU_WriteByte( MIU1_PROTECT0_ID0+index, MIU_NormalBankInfo[1].u8GroupClient[index] );
    }
//RQ
    for( index = 0; index < 0x40; index++ )
    {
        HAL_MIU_Write2Byte( MIU_RQ+index*2, MIU_NormalBankInfo[0].u16RQ[index] );
        HAL_MIU_Write2Byte( MIU1_RQ+index*2, MIU_NormalBankInfo[1].u16RQ[index] );
    }
//miu protect addr
    for( index = 0; index < 9; index++ )
    {
        HAL_MIU_Write2Byte( MIU_PROTECT0_START+index*2, MIU_NormalBankInfo[0].u16ProtectAddr[index] );
        HAL_MIU_Write2Byte( MIU1_PROTECT0_START+index*2, MIU_NormalBankInfo[1].u16ProtectAddr[index] );
    }
//After Protect ID & Addr, enable miu protect
    HAL_MIU_Write2Byte( MIU_PROTECT_EN_INTERNAL, MIU_NormalBankInfo[0].u16ProtectEn );
    HAL_MIU_Write2Byte( MIU1_PROTECT_EN, MIU_NormalBankInfo[1].u16ProtectEn );
//miu select
    for( index = 0; index < 6; index++ )
    {
        HAL_MIU_Write2Byte( REG_MIU_SEL0+index*2, MIU_NormalBankInfo[0].u16SelMIU[index] );
        HAL_MIU_Write2Byte( REG_MIU1_SEL0+index*2, MIU_NormalBankInfo[1].u16SelMIU[index] );
    }
//group priority
    HAL_MIU_Write2Byte( MIU_REG_BASE+REG_MIU_GROUP_PRIORITY, MIU_NormalBankInfo[0].u16GroupPriority );
    HAL_MIU_Write2Byte( MIU1_REG_BASE+REG_MIU_GROUP_PRIORITY, MIU_NormalBankInfo[1].u16GroupPriority );
    HAL_MIU_Write2Byte( MIU_ARB_REG_BASE+REG_MIU_GROUP_PRIORITY, MIU_BWBankInfo[0][0].u16GroupPriority );
    HAL_MIU_Write2Byte( MIU1_ARB_REG_BASE+REG_MIU_GROUP_PRIORITY, MIU_BWBankInfo[1][0].u16GroupPriority );

//BWRQ
    for( index = 0; index < 0x20; index++ )
    {
        HAL_MIU_Write2Byte( MIU_ARB_REG_BASE+index*2, MIU_BWBankInfo[0][0].u16BWRQ[index] );
        HAL_MIU_Write2Byte( MIU_ARBB_REG_BASE+index*2, MIU_BWBankInfo[0][1].u16BWRQ[index] );
        HAL_MIU_Write2Byte( MIU1_ARB_REG_BASE+index*2, MIU_BWBankInfo[1][0].u16BWRQ[index] );
        HAL_MIU_Write2Byte( MIU1_ARBB_REG_BASE+index*2, MIU_BWBankInfo[1][1].u16BWRQ[index] );
    }
//BW ctrl
    for( index = 0; index <= 0x0D; index++ )
    {
        HAL_MIU_Write2Byte( MIU_BW_CTRL+index*2, MIU_BWBankInfo[0][0].u16BWCtrl[index] );
        HAL_MIU_Write2Byte( MIU1_BW_CTRL+index*2, MIU_BWBankInfo[1][0].u16BWCtrl[index] );
    }
//BW select MIU
    for( index = 0; index < 2; index++ )
    {
        HAL_MIU_Write2Byte( REG_MIU_SEL6+index*2, MIU_BWBankInfo[0][0].u16BWSelMIU[index] );
        HAL_MIU_Write2Byte( REG_MIU1_SEL6+index*2, MIU_BWBankInfo[1][0].u16BWSelMIU[index] );
    }

    return TRUE;
}

MS_BOOL HAL_MIU_Kernel_Dram_ReadSize(MS_U8 MiuID, MIU_DDR_SIZE *pDramSize)
{
    MS_U32 regAddr;

    switch(MiuID)
    {
        case 0:
            regAddr = MIU_PROTECT_DDR_SIZE;
            break;
        case 1:
            regAddr = MIU1_PROTECT_DDR_SIZE;
            break;
        default:
            return FALSE;
    }

    switch( (HAL_MIU_ReadByte(regAddr)&0xF0)>>4 )
    {
        case 0x1:
            *pDramSize = E_MIU_DDR_2MB;
            break;
        case 0x2:
            *pDramSize = E_MIU_DDR_4MB;
            break;
        case 0x3:
            *pDramSize = E_MIU_DDR_8MB;
            break;
        case 0x4:
            *pDramSize = E_MIU_DDR_16MB;
            break;
        case 0x5:
            *pDramSize = E_MIU_DDR_32MB;
            break;
        case 0x6:
            *pDramSize = E_MIU_DDR_64MB;
            break;
        case 0x7:
            *pDramSize = E_MIU_DDR_128MB;
            break;
        case 0x8:
            *pDramSize = E_MIU_DDR_256MB;
            break;
        case 0x9:
            *pDramSize = E_MIU_DDR_512MB;
            break;
        case 0xA:
            *pDramSize = E_MIU_DDR_1024MB;
            break;
        case 0xB:
            *pDramSize = E_MIU_DDR_2048MB;
            break;
        default:
            return FALSE;
    }

    return TRUE;
}

MS_BOOL HAL_MIU_Set_DebugLevel(MIU_DBGLV eDebugLevel)
{
    mstar_debug = eDebugLevel;
    return TRUE;
}

MS_BOOL HAL_MIU_Enable_Hitkernelpanic(MS_U32 bEnablePanic)
{
    miu_hit_panic = bEnablePanic;
    return TRUE;
}

MS_U32 HAL_MIU_Get_Hitkernelpanic(void)
{
    return miu_hit_panic;
}

MS_BOOL HAL_MIU_Set_ADCDMA_Protect(MS_PHY start_addr)
{
    MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "not support %s\n", __FUNCTION__);
    return FALSE;
}

void HAL_MIU_Remove_ADCDMA_Protect(void)
{
}
