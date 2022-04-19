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
#define MIU_CLIENT_GP0              \
    MIU_CLIENT_NONE,                \
    MIU_CLIENT_DUMMY,  \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_SECURE_R2_RW,        \
    MIU_CLIENT_DUMMY,          \
    MIU_CLIENT_DUMMY,           \
    MIU_CLIENT_PM51_RW,             \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_USB_UHC0_RW,         \
    MIU_CLIENT_USB_UHC1_RW,         \
    MIU_CLIENT_USB_UHC2_RW,         \
    MIU_CLIENT_MVD_BBU_RW,          \
    MIU_CLIENT_EMAC_RW,             \
    MIU_CLIENT_BDMA_RW,             \
    MIU_CLIENT_MIUTEST_R

#define MIU_CLIENT_GP1              \
    MIU_CLIENT_DUMMY,     \
    MIU_CLIENT_DEMOD_W,             \
    MIU_CLIENT_DUMMY,             \
    MIU_CLIENT_UART_DMA_RW,         \
    MIU_CLIENT_MIIC_DMA_RW,         \
    MIU_CLIENT_JPD720P_RW,          \
    MIU_CLIENT_DUMMY,             \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_TSP_R,               \
    MIU_CLIENT_TSP_PVR0_W,          \
    MIU_CLIENT_SDIO_RW,             \
    MIU_CLIENT_VD_TTXSL_W,           \
    MIU_CLIENT_VD_COMB_W,           \
    MIU_CLIENT_VD_COMB_R,           \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP2              \
    MIU_CLIENT_GE_RW,               \
    MIU_CLIENT_HVD_BBU_R,           \
    MIU_CLIENT_HVD_RW,              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_MVD_RW,              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_GPD_RW,              \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_NAND_RW,             \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_DSCRMB_RW,           \
    MIU_CLIENT_DUMMY,          \
    MIU_CLIENT_TSP_ORZ_W,           \
    MIU_CLIENT_TSP_ORZ_R,           \
    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP3              \
    MIU_CLIENT_AUDIO_RW,               \
    MIU_CLIENT_EVD_R2D_RW,               \
    MIU_CLIENT_EVD_R2I_R,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP4              \
    MIU_CLIENT_EVD_RW,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP5              \
    MIU_CLIENT_SC_IPMAIN_RW,        \
    MIU_CLIENT_SC_OPMAIN_RW,        \
    MIU_CLIENT_MVOP_128BIT_R,       \
    MIU_CLIENT_MFDEC_R,             \
    MIU_CLIENT_DUMMY,          \
    MIU_CLIENT_GOP0_R,              \
    MIU_CLIENT_GOP1_R,              \
    MIU_CLIENT_GOP2_R,              \
    MIU_CLIENT_DBG_R,               \
    MIU_CLIENT_DS_R,                \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_SC_DIPW_RW,          \
    MIU_CLIENT_SC_LOCALDIMING_RW,   \
    MIU_CLIENT_SC_OD_RW,            \
    MIU_CLIENT_GOP3_PDW0_RW,        \
    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP6              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP7              \
    MIU_CLIENT_MIPS_RW,             \
    MIU_CLIENT_G3D_RW,              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP0              \
    MIU_CLIENT_NONE,                \
    MIU_CLIENT_DUMMY,  \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_SECURE_R2_RW,        \
    MIU_CLIENT_DUMMY,          \
    MIU_CLIENT_DUMMY,           \
    MIU_CLIENT_PM51_RW,             \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_USB_UHC0_RW,         \
    MIU_CLIENT_USB_UHC1_RW,         \
    MIU_CLIENT_USB_UHC2_RW,         \
    MIU_CLIENT_MVD_BBU_RW,          \
    MIU_CLIENT_EMAC_RW,             \
    MIU_CLIENT_BDMA_RW,             \
    MIU_CLIENT_MIUTEST_R

#define MIU1_CLIENT_GP1              \
    MIU_CLIENT_DUMMY,     \
    MIU_CLIENT_DEMOD_W,             \
    MIU_CLIENT_DUMMY,             \
    MIU_CLIENT_UART_DMA_RW,         \
    MIU_CLIENT_MIIC_DMA_RW,         \
    MIU_CLIENT_JPD720P_RW,          \
    MIU_CLIENT_DUMMY,             \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_TSP_R,               \
    MIU_CLIENT_TSP_PVR0_W,          \
    MIU_CLIENT_SDIO_RW,             \
    MIU_CLIENT_VD_TTXSL_W,           \
    MIU_CLIENT_VD_COMB_W,           \
    MIU_CLIENT_VD_COMB_R,           \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP2              \
    MIU_CLIENT_GE_RW,               \
    MIU_CLIENT_HVD_BBU_R,           \
    MIU_CLIENT_HVD_RW,              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_MVD_RW,              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_GPD_RW,              \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_NAND_RW,             \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_DSCRMB_RW,           \
    MIU_CLIENT_DUMMY,          \
    MIU_CLIENT_TSP_ORZ_W,           \
    MIU_CLIENT_TSP_ORZ_R,           \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP3              \
    MIU_CLIENT_AUDIO_RW,               \
    MIU_CLIENT_EVD_R2D_RW,               \
    MIU_CLIENT_EVD_R2I_R,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP4              \
    MIU_CLIENT_EVD_RW,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP5              \
    MIU_CLIENT_SC_IPMAIN_RW,        \
    MIU_CLIENT_SC_OPMAIN_RW,        \
    MIU_CLIENT_MVOP_128BIT_R,       \
    MIU_CLIENT_MFDEC_R,             \
    MIU_CLIENT_DUMMY,          \
    MIU_CLIENT_GOP0_R,              \
    MIU_CLIENT_GOP1_R,              \
    MIU_CLIENT_GOP2_R,              \
    MIU_CLIENT_DBG_R,               \
    MIU_CLIENT_DS_R,                \
    MIU_CLIENT_DUMMY,              \
    MIU_CLIENT_SC_DIPW_RW,          \
    MIU_CLIENT_SC_LOCALDIMING_RW,   \
    MIU_CLIENT_SC_OD_RW,            \
    MIU_CLIENT_GOP3_PDW0_RW,        \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP6              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY

#define MIU1_CLIENT_GP7              \
    MIU_CLIENT_MIPS_RW,             \
    MIU_CLIENT_G3D_RW,              \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY,               \
    MIU_CLIENT_DUMMY


#define IDNUM_KERNELPROTECT (MIU_MAX_PROTECT_ID)
//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
static const eMIUClientID clientTbl[MIU_MAX_DEVICE][MIU_MAX_TBL_CLIENT] =
{
    {
        MIU_CLIENT_GP0,
        MIU_CLIENT_GP1,
        MIU_CLIENT_GP2,
        MIU_CLIENT_GP3,
        MIU_CLIENT_GP4,
        MIU_CLIENT_GP5,
        MIU_CLIENT_GP6,
        MIU_CLIENT_GP7
    },
    {
        MIU1_CLIENT_GP0,
        MIU1_CLIENT_GP1,
        MIU1_CLIENT_GP2,
        MIU1_CLIENT_GP3,
        MIU1_CLIENT_GP4,
        MIU1_CLIENT_GP5,
        MIU1_CLIENT_GP6,
        MIU1_CLIENT_GP7,
    }
};

static MS_U32 clientId_KernelProtect[MIU_MAX_PROTECT_ID] =
{
    MIU_CLIENT_MIPS_RW,   //0
    MIU_CLIENT_NAND_RW,
    MIU_CLIENT_USB_UHC0_RW,
    MIU_CLIENT_USB_UHC1_RW,
    MIU_CLIENT_USB_UHC2_RW,  // 4
    MIU_CLIENT_G3D_RW,
    MIU_CLIENT_SDIO_RW,
    MIU_CLIENT_SC_DIPW_RW,
#ifdef CONFIG_MSTAR_IPAPOOL
    MIU_CLIENT_NONE,
#else
    MIU_CLIENT_GE_RW,
#endif
    MIU_CLIENT_NONE,   // 9
    MIU_CLIENT_NONE,
    MIU_CLIENT_NONE,
    MIU_CLIENT_NONE,   // 12
    MIU_CLIENT_NONE,
    MIU_CLIENT_NONE,
    MIU_CLIENT_NONE
};


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_CHIP_MIU_0 = 0,
    E_CHIP_MIU_1,
    E_CHIP_MIU_2,
    E_CHIP_MIU_3,
    E_CHIP_MIU_NUM,
} CHIP_MIU_ID;

//-------------------------------------------------------------------------------------------------
//  Macros
//-------------------------------------------------------------------------------------------------
#define _phy_to_miu_offset(MiuSel, Offset, PhysAddr) if (PhysAddr < ARM_MIU1_BASE_ADDR) \
                                                        {MiuSel = E_CHIP_MIU_0; Offset = PhysAddr;} \
                                                     else \
                                                         {MiuSel = E_CHIP_MIU_1; Offset = PhysAddr - ARM_MIU1_BASE_ADDR;}
#define _miu_offset_to_phy(MiuSel, Offset, PhysAddr) if (MiuSel == E_CHIP_MIU_0) \
                                                        {PhysAddr = Offset;} \
                                                     else \
                                                         {PhysAddr = Offset + ARM_MIU1_BASE_ADDR;}

#define MIU_HAL_ERR(fmt, args...)   printk(KERN_ERR "miu hal error %s:%d" fmt,__FUNCTION__,__LINE__,## args)

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

static MS_BOOL IDEnables[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][MIU_MAX_PROTECT_ID] = {{{0},{0},{0},{0}},{{0},{0},{0},{0}}}; //ID enable for protect block 0~3
static MS_U32 IDs[MIU_MAX_DEVICE][MIU_MAX_PROTECT_ID] = {{0}, {0}}; //IDs for protection

//-------------------------------------------------------------------------------------------------
//  MIU Debug Message Level
//-------------------------------------------------------------------------------------------------
static MS_U32 mstar_debug = E_MIU_DBGLV_ERR;
static MS_U32 miu_hit_panic = 0;

//-------------------------------------------------------------------------------------------------
//  MTLB HAL internal function
//-------------------------------------------------------------------------------------------------
MS_PHY HAL_MIU_BA2PA(MS_PHY u64BusAddr)
{
    MS_PHY u64PhyAddr = 0x0UL;

    // pa = ba - offset
    if( (u64BusAddr >= ARM_MIU0_BUS_BASE) && (u64BusAddr < ARM_MIU1_BUS_BASE) )	// MIU0
        u64PhyAddr = u64BusAddr - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
    else if( (u64BusAddr >= ARM_MIU1_BUS_BASE) && (u64BusAddr < ARM_MIU2_BUS_BASE))	// MIU1
        u64PhyAddr = u64BusAddr - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;
    else
        u64PhyAddr = u64BusAddr - ARM_MIU2_BUS_BASE + ARM_MIU2_BASE_ADDR;	// MIU2

    return u64PhyAddr;
}

static MS_S16 HAL_MIU_GetClientInfo(MS_U8 u8MiuDev, eMIUClientID eClientID)
{
    MS_S16 idx;

    if (MIU_MAX_DEVICE <= u8MiuDev)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Wrong MIU device:%u\n", u8MiuDev);
        return (-1);
    }

    for (idx = 0; idx < MIU_MAX_TBL_CLIENT; idx++)
        if (eClientID == clientTbl[u8MiuDev][idx])
            return idx;
    return (-1);
}

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

static void HAL_MIU_SetProtectID(MS_U32 u32Reg, MS_U8 u8MiuDev, MS_U32 u32ClientID)
{
    MS_S16 sVal = HAL_MIU_GetClientInfo(u8MiuDev, (eMIUClientID)u32ClientID);
    MS_S16 sIDVal;

    if (0 > sVal)
        sVal = 0;

    sIDVal = HAL_MIU_ReadByte(u32Reg);
    sIDVal &= 0x80;
    sIDVal |= sVal;
    HAL_MIU_WriteByte(u32Reg, sIDVal);
}

static MS_BOOL HAL_MIU_WriteRegBit(MS_U32 u32RegAddr, MS_U8 u8Mask, MS_BOOL bEnable)
{
    MS_U8 u8Val = HAL_MIU_ReadByte(u32RegAddr);
    if (!u32RegAddr)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    u8Val = HAL_MIU_ReadByte(u32RegAddr);
    u8Val = (bEnable) ? (u8Val | u8Mask) : (u8Val & ~u8Mask);
    HAL_MIU_WriteByte(u32RegAddr, u8Val);
    return TRUE;
}

static MS_BOOL HAL_MIU_SetGroupID(MS_U8 u8MiuSel, MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_U32 u32RegAddrID, MS_U32 u32RegAddrIDenable)
{
    MS_U32 u32index0, u32index1;
    MS_U32 u32ID;
    MS_U8 u8isfound0, u8isfound1;
    MS_U16 u16idenable;

    //reset IDenables for protect u8Blockx
    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        IDEnables[u8MiuSel][u8Blockx][u32index0] = 0;
    }

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        u32ID = pu32ProtectId[u32index0];

        //Unused ID
        if(u32ID == 0)
            continue;

        u8isfound0 = FALSE;

        for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID; u32index1++)
        {
            if(IDs[u8MiuSel][u32index1] == u32ID)
            {
                //ID reused former setting
                IDEnables[u8MiuSel][u8Blockx][u32index1] = 1;
                u8isfound0 = TRUE;
                break;
            }
        }

        //Need to create new ID in IDs
        if(u8isfound0 != TRUE)
        {
            u8isfound1 = FALSE;

            for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID; u32index1++)
            {
                if(IDs[u8MiuSel][u32index1] == 0)
                {
                    IDs[u8MiuSel][u32index1] = u32ID;
                    IDEnables[u8MiuSel][u8Blockx][u32index1] = 1;
                    u8isfound1 = TRUE;
                    break;
                }
            }

            //ID overflow
            if(u8isfound1 == FALSE)
                return FALSE;
        }
    }

    u16idenable = 0;

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        if(IDEnables[u8MiuSel][u8Blockx][u32index0] == 1)
            u16idenable |= (1<<u32index0);
    }

    HAL_MIU_Write2Byte(u32RegAddrIDenable, u16idenable);

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        HAL_MIU_SetProtectID(u32RegAddrID + u32index0, u8MiuSel, IDs[u8MiuSel][u32index0]);
    }

    return TRUE;
}

static MS_BOOL HAL_MIU_ResetGroupID(MS_U8 u8MiuSel, MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_U32 u32RegAddrID, MS_U32 u32RegAddrIDenable)
{
    MS_U32 u32index0, u32index1;
    MS_U8 u8isIDNoUse;
    MS_U16 u16idenable;

    //reset IDenables for protect u8Blockx
    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        IDEnables[u8MiuSel][u8Blockx][u32index0] = 0;
    }

    u16idenable = 0x0;

    HAL_MIU_Write2Byte(u32RegAddrIDenable, u16idenable);

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        u8isIDNoUse  = FALSE;

        for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_BLOCK; u32index1++)
        {
            if(IDEnables[u8MiuSel][u32index1][u32index0] == 1)
            {
                //protect ID is still be used
                u8isIDNoUse  = FALSE;
                break;
            }
            u8isIDNoUse  = TRUE;
        }

        if(u8isIDNoUse == TRUE)
            IDs[u8MiuSel][u32index0] = 0;
    }

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID; u32index0++)
    {
        HAL_MIU_SetProtectID(u32RegAddrID + u32index0, u8MiuSel, IDs[u8MiuSel][u32index0]);
    }

    return TRUE;
}

static void HAL_MIU_Write2BytesBit(MS_U32 u32RegOffset, MS_BOOL bEnable, MS_U16 u16Mask)
{
    MS_U16 val = HAL_MIU_Read2Byte(u32RegOffset);
    val = (bEnable) ? (val | u16Mask) : (val & ~u16Mask);
    HAL_MIU_Write2Byte(u32RegOffset, val);
}

//-------------------------------------------------------------------------------------------------
//  MTLB HAL function
//-------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: HAL_MIU_Kernel_GetDefaultClientID_KernelProtect()
/// @brief \b Function \b Description:  Get default client id array pointer for protect kernel
/// @param <RET>           \b     : The pointer of Array of client IDs
////////////////////////////////////////////////////////////////////////////////
MS_U32* HAL_MIU_Kernel_GetDefaultClientID_KernelProtect(void)
{
    if(IDNUM_KERNELPROTECT > 0)
        return  (MS_U32 *)&clientId_KernelProtect[0];

    return NULL;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_Kernel_ParseOccupiedResource
/// @brief \b Function  \b Description: Parse occupied resource to software structure
/// @return             \b 0: Fail 1: OK
////////////////////////////////////////////////////////////////////////////////
MS_BOOL HAL_MIU_Kernel_ParseOccupiedResource(void)
{
    MS_U8  u8MiuSel;
    MS_U8  u8Blockx;
    MS_U8  u8ClientID;
    MS_U16 u16idenable;
    MS_U32 u32index;
    MS_U32 u32RegAddr;
    MS_U32 u32RegAddrIDenable;

    for(u8MiuSel = E_MIU_0; u8MiuSel < MIU_MAX_DEVICE; u8MiuSel++)
    {
        for(u8Blockx = E_MIU_BLOCK_0; u8Blockx < E_MIU_BLOCK_NUM; u8Blockx++)
        {
            if(u8MiuSel == E_MIU_0)
            {
                u32RegAddr = MIU_PROTECT0_ID0;

                switch (u8Blockx)
                {
                    case E_MIU_BLOCK_0:
                        u32RegAddrIDenable = MIU_PROTECT0_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_1:
                        u32RegAddrIDenable = MIU_PROTECT1_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_2:
                        u32RegAddrIDenable = MIU_PROTECT2_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_3:
                        u32RegAddrIDenable = MIU_PROTECT3_ID_ENABLE;
                        break;
                    default:
                        return false;
                }
            }
            else if(u8MiuSel == E_MIU_1)
            {
                u32RegAddr = MIU1_PROTECT0_ID0;

                switch (u8Blockx)
                {
                    case E_MIU_BLOCK_0:
                        u32RegAddrIDenable = MIU1_PROTECT0_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_1:
                        u32RegAddrIDenable = MIU1_PROTECT1_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_2:
                        u32RegAddrIDenable = MIU1_PROTECT2_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_3:
                        u32RegAddrIDenable = MIU1_PROTECT3_ID_ENABLE;
                        break;
                    default:
                        return false;
                }
            }
            else if(u8MiuSel == E_MIU_2)
            {
                u32RegAddr = MIU2_PROTECT0_ID0;

                switch (u8Blockx)
                {
                case E_MIU_BLOCK_0:
                     u32RegAddrIDenable = MIU2_PROTECT0_ID_ENABLE;
                     break;
                 case E_MIU_BLOCK_1:
                     u32RegAddrIDenable = MIU2_PROTECT1_ID_ENABLE;
                     break;
                 case E_MIU_BLOCK_2:
                     u32RegAddrIDenable = MIU2_PROTECT2_ID_ENABLE;
                     break;
                 case E_MIU_BLOCK_3:
                     u32RegAddrIDenable = MIU2_PROTECT3_ID_ENABLE;
                     break;
                 default:
                     return false;
                }
            }
            else if(u8MiuSel == E_MIU_3)
            {
                u32RegAddr = MIU3_PROTECT0_ID0;

                switch (u8Blockx)
                {
                case E_MIU_BLOCK_0:
                     u32RegAddrIDenable = MIU3_PROTECT0_ID_ENABLE;
                     break;
                 case E_MIU_BLOCK_1:
                     u32RegAddrIDenable = MIU3_PROTECT1_ID_ENABLE;
                     break;
                 case E_MIU_BLOCK_2:
                     u32RegAddrIDenable = MIU3_PROTECT2_ID_ENABLE;
                     break;
                 case E_MIU_BLOCK_3:
                     u32RegAddrIDenable = MIU3_PROTECT3_ID_ENABLE;
                     break;
                 default:
                     return false;
                }
            }
            else
            {
                MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s not support MIU%u!\n", __FUNCTION__, u8MiuSel );
                return FALSE;
            }

            u16idenable = HAL_MIU_Read2Byte(u32RegAddrIDenable);
            for(u32index = 0; u32index < MIU_MAX_PROTECT_ID; u32index++)
            {
                IDEnables[u8MiuSel][u8Blockx][u32index] = ((u16idenable >> u32index) & 0x1)? 1: 0;
            }
        }//for(u8Blockx = E_MIU_BLOCK_0; u8Blockx < E_MIU_BLOCK_NUM; u8Blockx++)

        for(u32index = 0; u32index < MIU_MAX_PROTECT_ID; u32index++)
        {
            u8ClientID = HAL_MIU_ReadByte(u32RegAddr + u32index) & 0x7F;
            IDs[u8MiuSel][u32index] = clientTbl[u8MiuSel][u8ClientID];
        }
    }//for(u8MiuSel = E_MIU_0; u8MiuSel < E_MIU_NUM; u8MiuSel++)

    return TRUE;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: HAL_MIU_Kernel_Protect()
/// @brief \b Function \b Description:  Enable/Disable MIU Protection mode
/// @param u8Blockx        \b IN     : MIU Block to protect (0 ~ 4)
/// @param *pu8ProtectId   \b IN     : Allow specified client IDs to write
/// @param u32Start        \b IN     : Starting bus address
/// @param u32End          \b IN     : End bus address
/// @param bSetFlag        \b IN     : Disable or Enable MIU protection
///                                      - -Disable(0)
///                                      - -Enable(1)
/// @param <OUT>           \b None    :
/// @param <RET>           \b None    :
/// @param <GLOBAL>        \b None    :
////////////////////////////////////////////////////////////////////////////////
MS_BOOL HAL_MIU_Kernel_Protect(
        MS_U8    u8Blockx,
        MS_U32    *pu32ProtectId,
        MS_PHY   u64BusStart,
        MS_PHY   u64BusEnd,
        MS_BOOL  bSetFlag
        )
{
    MS_U32 u32RegAddr;
    MS_U32 u32Reg;
    MS_U32 u32RegAddrStar;
    MS_U32 u32RegAddrMSB;
    MS_U32 u32RegAddrIDenable;
    MS_U32 u32MiuProtectEn;
    MS_PHY phy64StartOffset;
    MS_PHY phy64EndOffset;
    MS_U16 u16Data;
    MS_U16 u16Data1;
    MS_U16 u16Data2;
    MS_U8  u8Data;
    MS_U8  u8MiuSel;
    MS_PHY phy64Start;
    MS_PHY phy64End;

    phy64Start = HAL_MIU_BA2PA(u64BusStart);
    phy64End = HAL_MIU_BA2PA(u64BusEnd);

    // Get MIU selection and offset
    _phy_to_miu_offset(u8MiuSel, phy64EndOffset, phy64End - 1)
    _phy_to_miu_offset(u8MiuSel, phy64StartOffset, phy64Start)

    phy64Start = phy64StartOffset;
    phy64End = phy64EndOffset + 1;

    // Incorrect Block ID
    if(u8Blockx >= E_MIU_BLOCK_NUM)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Err: Out of the number of protect device\n");
        return false;
    }
    else if(((phy64Start & ((1 << MIU_PAGE_SHIFT) -1)) != 0) || ((phy64End & ((1 << MIU_PAGE_SHIFT) -1)) != 0))
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Err: Protected address should be aligned to 8KB\n");
        return false;
    }
    else if(phy64Start >= phy64End)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Err: Start address is equal to or more than end address\n");
        return false;
    }


    //write_enable
    u8Data = 1 << u8Blockx;
    if(u8MiuSel == E_CHIP_MIU_0)
    {
        u32RegAddrMSB = MIU_PROTECT0_MSB;
        u16Data1 = HAL_MIU_Read2Byte(u32RegAddrMSB);

        u32RegAddr = MIU_PROTECT0_ID0;
        u32MiuProtectEn=MIU_PROTECT_EN;
        u32Reg = MIU_REG_BASE;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStar = MIU_PROTECT0_START;
                u32RegAddrIDenable = MIU_PROTECT0_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFFF0);
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStar = MIU_PROTECT1_START;
                u32RegAddrIDenable = MIU_PROTECT1_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFF0F);
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStar = MIU_PROTECT2_START;
                u32RegAddrIDenable = MIU_PROTECT2_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xF0FF);
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStar = MIU_PROTECT3_START;
                u32RegAddrIDenable = MIU_PROTECT3_ID_ENABLE;
                u16Data2 = (u16Data1 & 0x0FFF);
                break;
            default:
                return false;
        }
    }
    else if(u8MiuSel == E_CHIP_MIU_1)
    {
        u32RegAddrMSB = MIU1_PROTECT0_MSB;
        u16Data1 = HAL_MIU_Read2Byte(u32RegAddrMSB);

        u32RegAddr = MIU1_PROTECT0_ID0;
        u32MiuProtectEn=MIU1_PROTECT_EN;
        u32Reg = MIU1_REG_BASE;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStar = MIU1_PROTECT0_START;
                u32RegAddrIDenable = MIU1_PROTECT0_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFFF0);
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStar = MIU1_PROTECT1_START;
                u32RegAddrIDenable = MIU1_PROTECT1_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFF0F);
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStar = MIU1_PROTECT2_START;
                u32RegAddrIDenable = MIU1_PROTECT2_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xF0FF);
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStar = MIU1_PROTECT3_START;
                u32RegAddrIDenable = MIU1_PROTECT3_ID_ENABLE;
                u16Data2 = (u16Data1 & 0x0FFF);
                break;
            default:
                return false;
        }
     }
    else if(u8MiuSel == E_CHIP_MIU_2)
    {
        u32RegAddrMSB = MIU2_PROTECT0_MSB;
        u16Data1 = HAL_MIU_Read2Byte(u32RegAddrMSB);

        u32RegAddr = MIU2_PROTECT0_ID0;
        u32MiuProtectEn=MIU2_PROTECT_EN;
        u32Reg = MIU2_REG_BASE;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStar = MIU2_PROTECT0_START;
                u32RegAddrIDenable = MIU2_PROTECT0_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFFF0);
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStar = MIU2_PROTECT1_START;
                u32RegAddrIDenable = MIU2_PROTECT1_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFF0F);
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStar = MIU2_PROTECT2_START;
                u32RegAddrIDenable = MIU2_PROTECT2_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xF0FF);
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStar = MIU2_PROTECT3_START;
                u32RegAddrIDenable = MIU2_PROTECT3_ID_ENABLE;
                u16Data2 = (u16Data1 & 0x0FFF);
                break;
            default:
                return false;
        }
    }
    else if(u8MiuSel == E_CHIP_MIU_3)
    {
        u32RegAddrMSB = MIU3_PROTECT0_MSB;
        u16Data1 = HAL_MIU_Read2Byte(u32RegAddrMSB);

        u32RegAddr = MIU3_PROTECT0_ID0;
        u32MiuProtectEn=MIU3_PROTECT_EN;
        u32Reg = MIU3_REG_BASE;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStar = MIU3_PROTECT0_START;
                u32RegAddrIDenable = MIU3_PROTECT0_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFFF0);
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStar = MIU3_PROTECT1_START;
                u32RegAddrIDenable = MIU3_PROTECT1_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xFF0F);
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStar = MIU3_PROTECT2_START;
                u32RegAddrIDenable = MIU3_PROTECT2_ID_ENABLE;
                u16Data2 = (u16Data1 & 0xF0FF);
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStar = MIU3_PROTECT3_START;
                u32RegAddrIDenable = MIU3_PROTECT3_ID_ENABLE;
                u16Data2 = (u16Data1 & 0x0FFF);
                break;
            default:
                return false;
        }
    }
    else
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s not support MIU%u!\n", __FUNCTION__, u8MiuSel );
        return FALSE;
    }

    // Disable MIU protect
    HAL_MIU_WriteRegBit(u32MiuProtectEn,u8Data,DISABLE);

    if ( bSetFlag )
    {
        // Set Protect IDs
        if(HAL_MIU_SetGroupID(u8MiuSel, u8Blockx, pu32ProtectId, u32RegAddr, u32RegAddrIDenable) == FALSE)
        {
            return FALSE;
        }

        // Set BIT29,30 of start/end address
        u16Data2 = u16Data2 | (MS_U16)((phy64Start >> 29) << (u8Blockx*4));
        u16Data1 = u16Data2 | (MS_U16)(((phy64End - 1) >> 29) << (u8Blockx*4+2));
        HAL_MIU_Write2Byte(u32RegAddrMSB, u16Data1);

        // Start Address
        u16Data = (MS_U16)(phy64Start >> MIU_PAGE_SHIFT);   //8k/unit
        HAL_MIU_Write2Byte(u32RegAddrStar , u16Data);

        // End Address
        u16Data = (MS_U16)((phy64End >> MIU_PAGE_SHIFT)-1);   //8k/unit;
        HAL_MIU_Write2Byte(u32RegAddrStar + 2, u16Data);

        // Enable MIU protect
        HAL_MIU_WriteRegBit(u32MiuProtectEn, u8Data, ENABLE);
    }
    else
    {
        // Reset Protect IDs
        HAL_MIU_ResetGroupID(u8MiuSel, u8Blockx, pu32ProtectId, u32RegAddr, u32RegAddrIDenable);
    }

    // clear log
    HAL_MIU_Write2BytesBit(u32Reg+REG_MIU_PROTECT_STATUS, TRUE, REG_MIU_PROTECT_LOG_CLR);
    HAL_MIU_Write2BytesBit(u32Reg+REG_MIU_PROTECT_STATUS, FALSE, REG_MIU_PROTECT_LOG_CLR);

    return TRUE;
}

#define GET_HIT_BLOCK(regval)       ((regval & 0xE0) >> 5)
#define GET_HIT_CLIENT(regval)      (regval >> 8)

MS_BOOL HAL_MIU_Kernel_GetProtectInfo(MS_U8 u8MiuDev, MIU_PortectInfo *pInfo)
{
    MS_U16 ret = 0;
    MS_U16 loaddr = 0;
    MS_U16 hiaddr = 0;
    MS_U32 u32Address = 0;
    MS_U32 u32Reg;

    if(u8MiuDev == E_MIU_0)
    {
        u32Reg = MIU_REG_BASE;
    }
    else if(u8MiuDev == E_MIU_1)
    {
        u32Reg = MIU1_REG_BASE;
    }
    else if(u8MiuDev == E_MIU_2)
    {
        u32Reg = MIU2_REG_BASE;
    }
    else if(u8MiuDev == E_MIU_3)
    {
        u32Reg = MIU3_REG_BASE;
    }
    else
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s not support MIU%u!\n", __FUNCTION__, u8MiuDev );
        return FALSE;
    }

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

MS_U16 MIU_PROTECT_EN_T[4];
MS_U16 MIU_PROTECT0_ID_ENABLE_T[4];
MS_U16 MIU_PROTECT1_ID_ENABLE_T[4];
MS_U16 MIU_PROTECT2_ID_ENABLE_T[4];
MS_U16 MIU_PROTECT3_ID_ENABLE_T[4];
MS_U16 MIU_PROTECT0_START_T[4];
MS_U16 MIU_PROTECT1_START_T[4];
MS_U16 MIU_PROTECT2_START_T[4];
MS_U16 MIU_PROTECT3_START_T[4];
MS_U16 MIU_PROTECT0_END_T[4];
MS_U16 MIU_PROTECT1_END_T[4];
MS_U16 MIU_PROTECT2_END_T[4];
MS_U16 MIU_PROTECT3_END_T[4];
MS_U16 MIU_GROUPCLIENT[4][MIU_MAX_PROTECT_ID];


MS_BOOL HAL_MIU_Save(void)
{
	MIU_PROTECT_EN_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT_EN);
	MIU_PROTECT_EN_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT_EN);
	MIU_PROTECT_EN_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT_EN);
	MIU_PROTECT_EN_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT_EN);

	MIU_PROTECT0_ID_ENABLE_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT0_ID_ENABLE);
	MIU_PROTECT0_ID_ENABLE_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT0_ID_ENABLE);
	MIU_PROTECT0_ID_ENABLE_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT0_ID_ENABLE);
	MIU_PROTECT0_ID_ENABLE_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT0_ID_ENABLE);

	MIU_PROTECT1_ID_ENABLE_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT1_ID_ENABLE);
	MIU_PROTECT1_ID_ENABLE_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT1_ID_ENABLE);
	MIU_PROTECT1_ID_ENABLE_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT1_ID_ENABLE);
	MIU_PROTECT1_ID_ENABLE_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT1_ID_ENABLE);

	MIU_PROTECT2_ID_ENABLE_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT2_ID_ENABLE);
	MIU_PROTECT2_ID_ENABLE_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT2_ID_ENABLE);
	MIU_PROTECT2_ID_ENABLE_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT2_ID_ENABLE);
	MIU_PROTECT2_ID_ENABLE_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT2_ID_ENABLE);

	MIU_PROTECT3_ID_ENABLE_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT3_ID_ENABLE);
	MIU_PROTECT3_ID_ENABLE_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT3_ID_ENABLE);
	MIU_PROTECT3_ID_ENABLE_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT3_ID_ENABLE);
	MIU_PROTECT3_ID_ENABLE_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT3_ID_ENABLE);

	MIU_PROTECT0_START_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT0_START);
	MIU_PROTECT0_START_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT0_START);
	MIU_PROTECT0_START_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT0_START);
	MIU_PROTECT0_START_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT0_START);

	MIU_PROTECT1_START_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT1_START);
	MIU_PROTECT1_START_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT1_START);
	MIU_PROTECT1_START_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT1_START);
	MIU_PROTECT1_START_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT1_START);

	MIU_PROTECT2_START_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT2_START);
	MIU_PROTECT2_START_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT2_START);
	MIU_PROTECT2_START_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT2_START);
	MIU_PROTECT2_START_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT2_START);

	MIU_PROTECT3_START_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT3_START);
	MIU_PROTECT3_START_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT3_START);
	MIU_PROTECT3_START_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT3_START);
	MIU_PROTECT3_START_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT3_START);

	MIU_PROTECT0_END_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT0_START+2);
	MIU_PROTECT0_END_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT0_START+2);
	MIU_PROTECT0_END_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT0_START+2);
	MIU_PROTECT0_END_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT0_START+2);

	MIU_PROTECT1_END_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT1_START+2);
	MIU_PROTECT1_END_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT1_START+2);
	MIU_PROTECT1_END_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT1_START+2);
	MIU_PROTECT1_END_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT1_START+2);

	MIU_PROTECT2_END_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT2_START+2);
	MIU_PROTECT2_END_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT2_START+2);
	MIU_PROTECT2_END_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT2_START+2);
	MIU_PROTECT2_END_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT2_START+2);

	MIU_PROTECT3_END_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT3_START+2);
	MIU_PROTECT3_END_T[1] = HAL_MIU_Read2Byte(MIU1_PROTECT3_START+2);
	MIU_PROTECT3_END_T[2] = HAL_MIU_Read2Byte(MIU2_PROTECT3_START+2);
	MIU_PROTECT3_END_T[3] = HAL_MIU_Read2Byte(MIU3_PROTECT3_START+2);

	int index;
	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		MIU_GROUPCLIENT[0][index] = HAL_MIU_ReadByte(MIU_PROTECT0_ID0+index);
	}

	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		MIU_GROUPCLIENT[1][index] = HAL_MIU_ReadByte(MIU1_PROTECT0_ID0+index);
	}

	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		MIU_GROUPCLIENT[2][index] = HAL_MIU_ReadByte(MIU2_PROTECT0_ID0+index);
	}

	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		MIU_GROUPCLIENT[3][index] = HAL_MIU_ReadByte(MIU3_PROTECT0_ID0+index);
	}

}

MS_BOOL HAL_MIU_Restore(void)
{

	HAL_MIU_Write2BytesBit(MIU_PROTECT_EN, FALSE, BIT3 | BIT2 | BIT1 | BIT0);
	HAL_MIU_Write2BytesBit(MIU1_PROTECT_EN, FALSE, BIT3 | BIT2 | BIT1 | BIT0);
	HAL_MIU_Write2BytesBit(MIU2_PROTECT_EN, FALSE, BIT3 | BIT2 | BIT1 | BIT0);
	HAL_MIU_Write2BytesBit(MIU3_PROTECT_EN, FALSE, BIT3 | BIT2 | BIT1 | BIT0);

	HAL_MIU_Write2Byte(MIU_PROTECT0_ID_ENABLE,MIU_PROTECT0_ID_ENABLE_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT0_ID_ENABLE,MIU_PROTECT0_ID_ENABLE_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT0_ID_ENABLE,MIU_PROTECT0_ID_ENABLE_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT0_ID_ENABLE,MIU_PROTECT0_ID_ENABLE_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT1_ID_ENABLE,MIU_PROTECT1_ID_ENABLE_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT1_ID_ENABLE,MIU_PROTECT1_ID_ENABLE_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT1_ID_ENABLE,MIU_PROTECT1_ID_ENABLE_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT1_ID_ENABLE,MIU_PROTECT1_ID_ENABLE_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT2_ID_ENABLE,MIU_PROTECT2_ID_ENABLE_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT2_ID_ENABLE,MIU_PROTECT2_ID_ENABLE_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT2_ID_ENABLE,MIU_PROTECT2_ID_ENABLE_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT2_ID_ENABLE,MIU_PROTECT2_ID_ENABLE_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT3_ID_ENABLE,MIU_PROTECT3_ID_ENABLE_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT3_ID_ENABLE,MIU_PROTECT3_ID_ENABLE_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT3_ID_ENABLE,MIU_PROTECT3_ID_ENABLE_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT3_ID_ENABLE,MIU_PROTECT3_ID_ENABLE_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT0_START,MIU_PROTECT0_START_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT0_START,MIU_PROTECT0_START_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT0_START,MIU_PROTECT0_START_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT0_START,MIU_PROTECT0_START_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT1_START,MIU_PROTECT1_START_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT1_START,MIU_PROTECT1_START_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT1_START,MIU_PROTECT1_START_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT1_START,MIU_PROTECT1_START_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT2_START,MIU_PROTECT2_START_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT2_START,MIU_PROTECT2_START_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT2_START,MIU_PROTECT2_START_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT2_START,MIU_PROTECT2_START_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT3_START,MIU_PROTECT3_START_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT3_START,MIU_PROTECT3_START_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT3_START,MIU_PROTECT3_START_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT3_START,MIU_PROTECT3_START_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT0_START+2,MIU_PROTECT0_END_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT0_START+2,MIU_PROTECT0_END_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT0_START+2,MIU_PROTECT0_END_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT0_START+2,MIU_PROTECT0_END_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT1_START+2,MIU_PROTECT1_END_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT1_START+2,MIU_PROTECT1_END_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT1_START+2,MIU_PROTECT1_END_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT1_START+2,MIU_PROTECT1_END_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT2_START+2,MIU_PROTECT2_END_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT2_START+2,MIU_PROTECT2_END_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT2_START+2,MIU_PROTECT2_END_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT2_START+2,MIU_PROTECT2_END_T[3]);

	HAL_MIU_Write2Byte(MIU_PROTECT3_START+2,MIU_PROTECT3_END_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT3_START+2,MIU_PROTECT3_END_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT3_START+2,MIU_PROTECT3_END_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT3_START+2,MIU_PROTECT3_END_T[3]);

	int index;
	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		HAL_MIU_WriteByte(MIU_PROTECT0_ID0+index,MIU_GROUPCLIENT[0][index]);
	}

	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		HAL_MIU_WriteByte(MIU1_PROTECT0_ID0+index,MIU_GROUPCLIENT[1][index]);
	}

	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		HAL_MIU_WriteByte(MIU2_PROTECT0_ID0+index,MIU_GROUPCLIENT[2][index]);
	}

	for(index = 0; index < MIU_MAX_PROTECT_ID; index++){
		HAL_MIU_WriteByte(MIU3_PROTECT0_ID0+index,MIU_GROUPCLIENT[3][index]);
	}

	HAL_MIU_Write2Byte(MIU_PROTECT_EN,MIU_PROTECT_EN_T[0]);
	HAL_MIU_Write2Byte(MIU1_PROTECT_EN,MIU_PROTECT_EN_T[1]);
	HAL_MIU_Write2Byte(MIU2_PROTECT_EN,MIU_PROTECT_EN_T[2]);
	HAL_MIU_Write2Byte(MIU3_PROTECT_EN,MIU_PROTECT_EN_T[3]);

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
        case 2:
            regAddr = MIU2_PROTECT_DDR_SIZE;
            break;
        case 3:
            regAddr = MIU3_PROTECT_DDR_SIZE;
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
