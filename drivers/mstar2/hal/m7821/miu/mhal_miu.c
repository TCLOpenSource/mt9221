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
#include <linux/delay.h>
#include "MsTypes.h"
#include "mdrv_types.h"
#include "mdrv_miu.h"
#include "regMIU.h"
#include "mhal_miu.h"
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include "mdrv_system.h"

#if defined(CONFIG_MIPS)
#include <asm/mips-boards/prom.h>
#include "mhal_chiptop_reg.h"
#elif defined(CONFIG_ARM)
#include <prom.h>
#include <asm/mach/map.h>
#include <asm/system.h>
#elif defined(CONFIG_ARM64)
#include <asm/arm-boards/prom.h>
#include <asm/mach/map.h>
#endif
#include "chip_setup.h"
#include <linux/bug.h>

//-------------------------------------------------------------------------------------------------
//  Define
//-------------------------------------------------------------------------------------------------
#define MIU_CLIENT_GP_DUMMY              \
/* 0 */    MIU_CLIENT_DUMMY, \
/* 1 */    MIU_CLIENT_DUMMY, \
/* 2 */    MIU_CLIENT_DUMMY, \
/* 3 */    MIU_CLIENT_DUMMY, \
/* 4 */    MIU_CLIENT_DUMMY, \
/* 5 */    MIU_CLIENT_DUMMY, \
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_DUMMY, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_DUMMY, \
/*13 */    MIU_CLIENT_DUMMY, \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP0  \
/* 0 */    MIU_CLIENT_NONE, \
/* 1 */    MIU_CLIENT_DUMMY, \
/* 2 */    MIU_CLIENT_DUMMY, \
/* 3 */    MIU_CLIENT_DUMMY, \
/* 4 */    MIU_CLIENT_DUMMY, \
/* 5 */    MIU_CLIENT_DUMMY,\
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_DUMMY, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_DUMMY, \
/*13 */    MIU_CLIENT_DUMMY,  \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP1  \
/* 0 */    MIU_CLIENT_DSP_CACHE_RW, \
/* 1 */    MIU_CLIENT_R2_I_RW, \
/* 2 */    MIU_CLIENT_R2_D_RW, \
/* 3 */    MIU_CLIENT_R2_BDMA_RW, \
/* 4 */    MIU_CLIENT_R21_I_R, \
/* 5 */    MIU_CLIENT_R21_D_RW,\
/* 6 */    MIU_CLIENT_R21_BDMA_RW, \
/* 7 */    MIU_CLIENT_DMAL2_RW, \
/* 8 */    MIU_CLIENT_MADDMA_RW, \
/* 9 */    MIU_CLIENT_SEDMA_RW, \
/*10 */    MIU_CLIENT_DMAL2_W, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_VREC_RW, \
/*13 */    MIU_CLIENT_DUMMY,  \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP2  \
/* 0 */    MIU_CLIENT_USB30_MERGE_RW, \
/* 1 */    MIU_CLIENT_PM51_RW, \
/* 2 */    MIU_CLIENT_USB20_RW, \
/* 3 */    MIU_CLIENT_EVD_R2D_RW, \
/* 4 */    MIU_CLIENT_MVD_R, \
/* 5 */    MIU_CLIENT_EMAC_RW, \
/* 6 */    MIU_CLIENT_BDMA_RW, \
/* 7 */    MIU_CLIENT_MIIC0_W, \
/* 8 */    MIU_CLIENT_PCIE_RW, \
/* 9 */    MIU_CLIENT_VD_RW, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_MVD_RW, \
/*12 */    MIU_CLIENT_GPD_RW, \
/*13 */    MIU_CLIENT_NAND_RW, \
/*14 */    MIU_CLIENT_EVD_R2D_RW,\
/*15 */    MIU_CLIENT_EVD_R2I_R

#define MIU_CLIENT_GP3  \
/* 0 */    MIU_CLIENT_SECURE_R2_RW, \
/* 1 */    MIU_CLIENT_JPD_RW, \
/* 2 */    MIU_CLIENT_TSP_ORZ_W, \
/* 3 */    MIU_CLIENT_TSP_ORZ_R, \
/* 4 */    MIU_CLIENT_AESDMA2_RW, \
/* 5 */    MIU_CLIENT_MFE0_W, \
/* 6 */    MIU_CLIENT_MFE1_R, \
/* 7 */    MIU_CLIENT_SDIO_RW, \
/* 8 */    MIU_CLIENT_DSCRMB_RW, \
/* 9 */    MIU_CLIENT_TSP_FIQ_RW, \
/*10 */    MIU_CLIENT_TSP_SEC_RW, \
/*11 */    MIU_CLIENT_TSP_FILEIN_RW, \
/*12 */    MIU_CLIENT_TSO_RW, \
/*13 */    MIU_CLIENT_DEMOD_RW, \
/*14 */    MIU_CLIENT_FRC_R2_RW, \
/*15 */    MIU_CLIENT_TSP_UFSHCI_RW

#define MIU_CLIENT_GP4  \
/* 0 */    MIU_CLIENT_DUMMY, \
/* 1 */    MIU_CLIENT_DUMMY, \
/* 2 */    MIU_CLIENT_DUMMY, \
/* 3 */    MIU_CLIENT_DUMMY, \
/* 4 */    MIU_CLIENT_DUMMY,\
/* 5 */    MIU_CLIENT_FRC_FRCM2_R, \
/* 6 */    MIU_CLIENT_FRC_FRCM3_R, \
/* 7 */    MIU_CLIENT_FRC_OPM0_R, \
/* 8 */    MIU_CLIENT_FRC_OPM1_R, \
/* 9 */    MIU_CLIENT_FRC_OPME0_R, \
/*10 */    MIU_CLIENT_FRC_OPME1_R, \
/*11 */    MIU_CLIENT_FRC_OPMI0_R, \
/*12 */    MIU_CLIENT_FRC_OPMI1_R, \
/*13 */    MIU_CLIENT_FRC_ME_R, \
/*14 */    MIU_CLIENT_FRC_HR_R, \
/*15 */    MIU_CLIENT_FRC_MI_R

/* FRC Support MIU1,2 only */
#define MIU_CLIENT_GP5  \
/* 0 */    MIU_CLIENT_DUMMY, \
/* 1 */    MIU_CLIENT_DUMMY, \
/* 2 */    MIU_CLIENT_DUMMY, \
/* 3 */    MIU_CLIENT_DUMMY, \
/* 4 */    MIU_CLIENT_DUMMY, \
/* 5 */    MIU_CLIENT_DUMMY, \
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_DUMMY, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_FRC_HVDM0_W, \
/*11 */    MIU_CLIENT_FRC_HVDM1_W, \
/*12 */    MIU_CLIENT_FRC_IPM0_W, \
/*13 */    MIU_CLIENT_FRC_IPM1_W, \
/*14 */    MIU_CLIENT_FRC_ME_W, \
/*15 */    MIU_CLIENT_FRC_HR_W

#define MIU_CLIENT_GP6  \
/* 0 */    MIU_CLIENT_SC_IPMAIN_RW, \
/* 1 */    MIU_CLIENT_SC_OPMAIN_RW, \
/* 2 */    MIU_CLIENT_MFDEC2_R, \
/* 3 */    MIU_CLIENT_MFDEC3_R, \
/* 4 */    MIU_CLIENT_MVOP1_0_R, \
/* 5 */    MIU_CLIENT_MVOP1_1_R, \
/* 6 */    MIU_CLIENT_GOP0_R, \
/* 7 */    MIU_CLIENT_GOP3_R, \
/* 8 */    MIU_CLIENT_SC_MGWIN1, \
/* 9 */    MIU_CLIENT_SC_MCDI_R, \
/*10 */    MIU_CLIENT_T3D_R, \
/*11 */    MIU_CLIENT_SC_IPM_R2_R, \
/*12 */    MIU_CLIENT_SC2_OPM_R, \
/*13 */    MIU_CLIENT_SC_OD_R, \
/*14 */    MIU_CLIENT_SC2_IPM_R, \
/*15 */    MIU_CLIENT_SC_AFBC0_R

#define MIU_CLIENT_GP7  \
/* 0 */    MIU_CLIENT_SC_DS_R, \
/* 1 */    MIU_CLIENT_SC2_DS_R, \
/* 2 */    MIU_CLIENT_MFDEC_R, \
/* 3 */    MIU_CLIENT_MFDEC1_R, \
/* 4 */    MIU_CLIENT_MVOP0_0_R, \
/* 5 */    MIU_CLIENT_MVOP0_1_R, \
/* 6 */    MIU_CLIENT_GOP1_R, \
/* 7 */    MIU_CLIENT_GOP2_R, \
/* 8 */    MIU_CLIENT_GOP4_R, \
/* 9 */    MIU_CLIENT_SC_DIPR_0_R, \
/*10 */    MIU_CLIENT_SC_DIPR_1_R, \
/*11 */    MIU_CLIENT_SC_DOLBY_R, \
/*12 */    MIU_CLIENT_VE_R, \
/*13 */    MIU_CLIENT_SC_CMDQ_R, \
/*14 */    MIU_CLIENT_SC_MGWIN0_R, \
/*15 */    MIU_CLIENT_SC_AFBC1_R

#define MIU_CLIENT_GP8  \
/* 0 */    MIU_CLIENT_SC_IPMAIN_W, \
/* 1 */    MIU_CLIENT_SC_OPMAIN_W, \
/* 2 */    MIU_CLIENT_MDWIN_W, \
/* 3 */    MIU_CLIENT_DUMMY, \
/* 4 */    MIU_CLIENT_DUMMY, \
/* 5 */    MIU_CLIENT_SC_DIP_0_W, \
/* 6 */    MIU_CLIENT_SC_DIP_1_W, \
/* 7 */    MIU_CLIENT_SC_PDW0_W, \
/* 8 */    MIU_CLIENT_SC_PDW1_W, \
/* 9 */    MIU_CLIENT_SC_MCDI_W, \
/*10 */    MIU_CLIENT_T3D_W, \
/*11 */    MIU_CLIENT_SC_DOLBY_W, \
/*12 */    MIU_CLIENT_VE_W, \
/*13 */    MIU_CLIENT_SC_OD_W, \
/*14 */    MIU_CLIENT_SC2_IPM_W, \
/*15 */    MIU_CLIENT_SC_ABC_DMA_W

#define MIU_CLIENT_GP9  \
/* 0 */    MIU_CLIENT_EVD_0_RW, \
/* 1 */    MIU_CLIENT_EVD_1_RW, \
/* 2 */    MIU_CLIENT_EVD_2_RW, \
/* 3 */    MIU_CLIENT_EVD_3_RW, \
/* 4 */    MIU_CLIENT_EVD_BBU_R, \
/* 5 */    MIU_CLIENT_HVD_0_RW, \
/* 6 */    MIU_CLIENT_HVD_1_RW, \
/* 7 */    MIU_CLIENT_HVD_2_RW, \
/* 8 */    MIU_CLIENT_HVD_BBU_R, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_DUMMY, \
/*13 */    MIU_CLIENT_DUMMY, \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_GE_RW

#define MIU_CLIENT_GP15  \
/* 0 */    MIU_CLIENT_CPU_R, \
/* 1 */    MIU_CLIENT_CPU_W, \
/* 2 */    MIU_CLIENT_GPU_R, \
/* 3 */    MIU_CLIENT_GPU_W, \
/* 4 */    MIU_CLIENT_DUMMY, \
/* 5 */    MIU_CLIENT_DUMMY, \
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_DUMMY, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_DUMMY, \
/*13 */    MIU_CLIENT_DUMMY, \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_DUMMY

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
#ifdef CONFIG_CMA
#define IDNUM_KERNELPROTECT (9)
#else
#define IDNUM_KERNELPROTECT (8)
#endif

static const eMIUClientID clientTbl[MIU_MAX_DEVICE][MIU_MAX_TBL_CLIENT] =
{
    {MIU_CLIENT_GP0, MIU_CLIENT_GP1, MIU_CLIENT_GP2, MIU_CLIENT_GP3, MIU_CLIENT_GP4, MIU_CLIENT_GP5, MIU_CLIENT_GP6, MIU_CLIENT_GP7, MIU_CLIENT_GP8, MIU_CLIENT_GP9, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP15}
    ,{MIU_CLIENT_GP0, MIU_CLIENT_GP1, MIU_CLIENT_GP2, MIU_CLIENT_GP3, MIU_CLIENT_GP4, MIU_CLIENT_GP5, MIU_CLIENT_GP6, MIU_CLIENT_GP7, MIU_CLIENT_GP8, MIU_CLIENT_GP9, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP_DUMMY, MIU_CLIENT_GP15}
};

static MS_U32 clientId_KernelProtect[MIU_MAX_PROTECT_ID] =
{
    MIU_CLIENT_CPU_R, MIU_CLIENT_CPU_W, MIU_CLIENT_GPU_R, MIU_CLIENT_GPU_W,
    MIU_CLIENT_NAND_RW, MIU_CLIENT_USB30_MERGE_RW, MIU_CLIENT_USB20_RW, MIU_CLIENT_SDIO_RW
#ifdef CONFIG_CMA
    ,MIU_CLIENT_GE_RW
#endif
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
                                                         {MiuSel = E_CHIP_MIU_2; Offset = PhysAddr - ARM_MIU2_BASE_ADDR;}

#define _miu_offset_to_phy(MiuSel, Offset, PhysAddr) if (MiuSel == E_CHIP_MIU_0) \
                                                        {PhysAddr = Offset;} \
                                                     else \
                                                         {PhysAddr = Offset + ARM_MIU2_BASE_ADDR;}

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

static MS_BOOL IDEnables[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][MIU_MAX_PROTECT_ID]=
{{{0},{0},{0},{0},{0},{0},{0},{0}},
 {{0},{0},{0},{0},{0},{0},{0},{0}}}; //ID enable for protect block 0~7
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
	if( (u64BusAddr >= ARM_MIU0_BUS_BASE) && (u64BusAddr < ARM_MIU1_BUS_BASE) )	// UMA
		u64PhyAddr = u64BusAddr - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
	else
		u64PhyAddr = u64BusAddr - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;

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

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_BDMA_Write4Byte
/// @brief \b Function  \b Description: write 4 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b u32Val : 4 byte data
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok FALSE: Fail
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////
MS_BOOL HAL_MIU_Write4Byte(MS_U32 u32RegAddr, MS_U32 u32Val)
{
    if (!u32RegAddr)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    HAL_MIU_Write2Byte(u32RegAddr, u32Val & 0x0000FFFF);
    HAL_MIU_Write2Byte(u32RegAddr+2, u32Val >> 16);
    return TRUE;
}

static void HAL_MIU_SetProtectID(MS_U32 u32Reg, MS_U8 u8MiuDev, MS_U32 u32ClientID)
{
    MS_S16 sVal = HAL_MIU_GetClientInfo(u8MiuDev, (eMIUClientID)u32ClientID);
    MS_S16 sIDVal;

    if (0 > sVal)
        sVal = 0;

    sIDVal = HAL_MIU_ReadByte(u32Reg);
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

    u16idenable = 0x0UL;

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
MS_U32* HAL_MIU_Kernel_GetDefaultClientID_KernelProtect()
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
        for(u8Blockx = E_MIU_BLOCK_0; u8Blockx < MIU_PROTECT_BLOCK_NUMBER; u8Blockx++)
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
                    case E_MIU_BLOCK_4:
                        u32RegAddrIDenable = MIU_PROTECT4_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_5:
                        u32RegAddrIDenable = MIU_PROTECT5_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_6:
                        u32RegAddrIDenable = MIU_PROTECT6_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_7:
                        u32RegAddrIDenable = MIU_PROTECT7_ID_ENABLE;
                        break;
                    default:
                        return false;
                }
            }
            else if(u8MiuSel == E_MIU_1)
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
                    case E_MIU_BLOCK_4:
                        u32RegAddrIDenable = MIU2_PROTECT4_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_5:
                        u32RegAddrIDenable = MIU2_PROTECT5_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_6:
                        u32RegAddrIDenable = MIU2_PROTECT6_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_7:
                        u32RegAddrIDenable = MIU2_PROTECT7_ID_ENABLE;
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
                IDEnables[u8MiuSel][u8Blockx][u32index] = ((u16idenable >> u32index) & 0x1UL)? 1: 0;
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
/// @param *pu32ProtectId   \b IN     : Allow specified client IDs to write
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
    MS_U32   *pu32ProtectId,
    MS_PHY   u64BusStart,
    MS_PHY   u64BusEnd,
    MS_BOOL  bSetFlag
    )
{
    MS_U32 u32RegAddr;
    MS_U32 u32RegAddrStar;
    MS_U32 u32RegAddrIDenable;
    MS_U32 u32MiuProtectEn;
    MS_PHY phy64StartOffset;
    MS_PHY phy64EndOffset;
    MS_U32 u32Data;
    MS_U8  u8Data;
    MS_U8  u8MiuSel;
    MS_PHY phy64Start;
    MS_PHY phy64End;

    phy64Start = HAL_MIU_BA2PA(u64BusStart);
    phy64End = HAL_MIU_BA2PA(u64BusEnd);

    // Get MIU selection and offset
    _phy_to_miu_offset(u8MiuSel, phy64EndOffset, phy64End - 1);	// use "u32End - 1" to avoid selecting wrong u8MiuSel and u32EndOffset, ex: 0xE0000000 will be MIU2_END and MIU3_START
    _phy_to_miu_offset(u8MiuSel, phy64StartOffset, phy64Start);

    phy64Start = phy64StartOffset;
    phy64End = phy64EndOffset + 1;	// due to we use u32End - 1 @_phy_to_miu_offset for getting u32EndOffset

    // Incorrect Block ID
    if(u8Blockx >= E_MIU_BLOCK_NUM)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Out of the number of protect device\n");
        return false;
    }
    else if(((phy64Start & ((1 << MIU_PAGE_SHIFT) -1)) != 0) || ((phy64End & ((1 << MIU_PAGE_SHIFT) -1)) != 0))
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Protected address should be aligned to 8KB\n");
        return false;
    }
    else if(phy64Start >= phy64End)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Start address is equal to or more than end address\n");
        return false;
    }


    //write_enable
    u8Data = 1 << u8Blockx;
    if(u8MiuSel == E_CHIP_MIU_0)
    {
        u32RegAddr = MIU_PROTECT0_ID0;
        u32MiuProtectEn=MIU_PROTECT_EN;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStar = MIU_PROTECT0_START;
                u32RegAddrIDenable = MIU_PROTECT0_ID_ENABLE;
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStar = MIU_PROTECT1_START;
                u32RegAddrIDenable = MIU_PROTECT1_ID_ENABLE;
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStar = MIU_PROTECT2_START;
                u32RegAddrIDenable = MIU_PROTECT2_ID_ENABLE;
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStar = MIU_PROTECT3_START;
                u32RegAddrIDenable = MIU_PROTECT3_ID_ENABLE;
                break;
            case E_MIU_BLOCK_4:
                u32RegAddrStar = MIU_PROTECT4_START;
                u32RegAddrIDenable = MIU_PROTECT4_ID_ENABLE;
                break;
            case E_MIU_BLOCK_5:
                u32RegAddrStar = MIU_PROTECT5_START;
                u32RegAddrIDenable = MIU_PROTECT5_ID_ENABLE;
                break;
            case E_MIU_BLOCK_6:
                u32RegAddrStar = MIU_PROTECT6_START;
                u32RegAddrIDenable = MIU_PROTECT6_ID_ENABLE;
                break;
            case E_MIU_BLOCK_7:
                u32RegAddrStar = MIU_PROTECT7_START;
                u32RegAddrIDenable = MIU_PROTECT7_ID_ENABLE;
                break;
            default:
                return false;
        }
    }
    else if(u8MiuSel == E_CHIP_MIU_1)
    {
        u32RegAddr = MIU2_PROTECT0_ID0;
        u32MiuProtectEn=MIU2_PROTECT_EN;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStar = MIU2_PROTECT0_START;
                u32RegAddrIDenable = MIU2_PROTECT0_ID_ENABLE;
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStar = MIU2_PROTECT1_START;
                u32RegAddrIDenable = MIU2_PROTECT1_ID_ENABLE;
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStar = MIU2_PROTECT2_START;
                u32RegAddrIDenable = MIU2_PROTECT2_ID_ENABLE;
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStar = MIU2_PROTECT3_START;
                u32RegAddrIDenable = MIU2_PROTECT3_ID_ENABLE;
                break;
            case E_MIU_BLOCK_4:
                u32RegAddrStar = MIU2_PROTECT4_START;
                u32RegAddrIDenable = MIU2_PROTECT4_ID_ENABLE;
                break;
            case E_MIU_BLOCK_5:
                u32RegAddrStar = MIU2_PROTECT5_START;
                u32RegAddrIDenable = MIU2_PROTECT5_ID_ENABLE;
                break;
            case E_MIU_BLOCK_6:
                u32RegAddrStar = MIU2_PROTECT6_START;
                u32RegAddrIDenable = MIU2_PROTECT6_ID_ENABLE;
                break;
            case E_MIU_BLOCK_7:
                u32RegAddrStar = MIU2_PROTECT7_START;
                u32RegAddrIDenable = MIU2_PROTECT7_ID_ENABLE;
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

        // Start Address
        u32Data = (MS_U32)(phy64Start >> MIU_PAGE_SHIFT);   //8k/unit
        HAL_MIU_Write4Byte(u32RegAddrStar , u32Data);

        // End Address
        u32Data = (MS_U32)((phy64End >> MIU_PAGE_SHIFT)-1);   //8k/unit;
        HAL_MIU_Write4Byte(u32RegAddrStar + 4, u32Data);

        // Enable MIU protect
        HAL_MIU_WriteRegBit(u32MiuProtectEn, u8Data, ENABLE);
    }
    else
    {
        // Reset Protect IDs
        HAL_MIU_ResetGroupID(u8MiuSel, u8Blockx, pu32ProtectId, u32RegAddr, u32RegAddrIDenable);
    }

    // clear log
    HAL_MIU_Write2BytesBit(u32RegAddr+REG_MIU_PROTECT_STATUS, TRUE, REG_MIU_PROTECT_LOG_CLR);
    HAL_MIU_Write2BytesBit(u32RegAddr+REG_MIU_PROTECT_STATUS, FALSE, REG_MIU_PROTECT_LOG_CLR);

    return TRUE;
}

#define MIU_BANK_SIZE (0x80)
MS_U16 MIU_PROTECT_EN_T[MIU_MAX_DEVICE];
MS_U16 MIU_PROTECT_BANK[MIU_MAX_DEVICE][MIU_BANK_SIZE];

MS_U16 MIU_SLIT_PROTECT0_ID_ENABLE_T[MIU_MAX_DEVICE];
MS_U16 MIU_SLIT_LOW_BOUND_T[MIU_MAX_DEVICE];
MS_U16 MIU_SLIT_HIGH_BOUND_T[MIU_MAX_DEVICE];
MS_U16 MIU_SLIT_CTL_T[MIU_MAX_DEVICE];
MS_U8 Slit_FlagGroup[MIU_MAX_DEVICE][SLIT_NUM] = {{0}, {0}};

MS_U16 MIU_ARBB_T[MIU_MAX_BANK_DEVICE_NUM][MIU_BANK_SIZE];
MS_U16 MIU_BLOCK_ARBITER[MIU_CLIENT_GROUP_NUM][MIU_BANK_SIZE/2];
MS_U16 MIU_BWBankInfo[MIU_MAX_BANK_DEVICE_NUM][2];

MS_BOOL HAL_MIU_Save(void)
{
//    MS_U16 u16Data;
    MS_U16 u16idx;

    // Enable MIU protect and slits protect
    MIU_PROTECT_EN_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT_EN);
    MIU_PROTECT_EN_T[1] = HAL_MIU_Read2Byte(MIU2_PROTECT_EN);
    for(u16idx = 0; u16idx < MIU_BANK_SIZE ; u16idx++)
    {
        MIU_PROTECT_BANK[0][u16idx] = HAL_MIU_Read2Byte(MIU_PROTECT_BASE+u16idx*2);
        MIU_PROTECT_BANK[1][u16idx] = HAL_MIU_Read2Byte(MIU2_PROTECT_BASE+u16idx*2);

        MIU_ARBB_T[0][u16idx] = HAL_MIU_Read2Byte(MIU_ARBB_REG_BASE+u16idx*2);
        MIU_ARBB_T[1][u16idx] = HAL_MIU_Read2Byte(MIU1_ARBB_REG_BASE+u16idx*2);
        MIU_ARBB_T[2][u16idx] = HAL_MIU_Read2Byte(MIU2_ARBB_REG_BASE+u16idx*2);
    }

    for(u16idx = 0; u16idx < (MIU_BANK_SIZE/2) ; u16idx++)
    {
        MIU_BLOCK_ARBITER[1][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP1+u16idx*2);
        MIU_BLOCK_ARBITER[2][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP2+u16idx*2);
        MIU_BLOCK_ARBITER[3][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP3+u16idx*2);
        MIU_BLOCK_ARBITER[4][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP4+u16idx*2);
        MIU_BLOCK_ARBITER[5][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP5+u16idx*2);
        MIU_BLOCK_ARBITER[6][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP6+u16idx*2);
        MIU_BLOCK_ARBITER[7][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP7+u16idx*2);
        MIU_BLOCK_ARBITER[8][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP8+u16idx*2);
        MIU_BLOCK_ARBITER[9][u16idx] = HAL_MIU_Read2Byte(MIU_BLOCK_ARBITER_GP9+u16idx*2);
    }

/*****Remove MIU Slit function*****

    // MIU2 can't use slits because all MIU2 is used by kernel, so no memory to mapping sram
    // ID enable
    MIU_SLIT_PROTECT0_ID_ENABLE_T[0] = HAL_MIU_Read2Byte(REG_MIU0_SLIT_PROTECT0_ID_ENABLE);
    MIU_SLIT_PROTECT0_ID_ENABLE_T[1] = HAL_MIU_Read2Byte(REG_MIU1_SLIT_PROTECT0_ID_ENABLE);

    // Low Bound
    MIU_SLIT_LOW_BOUND_T[0] = HAL_MIU_Read2Byte(REG_MIU0_SLIT_LOW_BOUND);
    MIU_SLIT_LOW_BOUND_T[1] = HAL_MIU_Read2Byte(REG_MIU1_SLIT_LOW_BOUND);

    // High bound
    MIU_SLIT_HIGH_BOUND_T[0] = HAL_MIU_Read2Byte(REG_MIU0_SLIT_HIGH_BOUND);
    MIU_SLIT_HIGH_BOUND_T[1] = HAL_MIU_Read2Byte(REG_MIU1_SLIT_HIGH_BOUND);

    //Slits
    u16Data = HAL_MIU_Read2Byte(REG_MIU0_SLIT_DBG_CTRL);
    u16Data &= ~(BIT5 | BIT6 | BIT7);
    u16Data |= REG_MIU_SLIT_DBG_SEL;
    HAL_MIU_Write2Byte(REG_MIU0_SLIT_DBG_CTRL, u16Data);
    u16Data = HAL_MIU_Read2Byte(REG_MIU1_SLIT_DBG_CTRL);
    u16Data &= ~(BIT5 | BIT6 | BIT7);
    u16Data |= REG_MIU_SLIT_DBG_SEL;
    HAL_MIU_Write2Byte(REG_MIU1_SLIT_DBG_CTRL, u16Data);

    HAL_MIU_WriteRegBit(REG_MIU0_SLIT_DBG_CTRL, REG_MIU_SLIT_DBG_EN, ENABLE);
    HAL_MIU_WriteRegBit(REG_MIU1_SLIT_DBG_CTRL, REG_MIU_SLIT_DBG_EN, ENABLE);

    for(u16idx = 0; u16idx < SLIT_NUM; u16idx++)
    {
        HAL_MIU_Write2Byte(REG_MIU0_SLIT_DBG_ADDR_BASE, u16idx);
        HAL_MIU_Write2Byte(REG_MIU1_SLIT_DBG_ADDR_BASE, u16idx);

        Slit_FlagGroup[0][u16idx] = HAL_MIU_Read2Byte(REG_MIU0_SLIT_DBG_VALUE);
        Slit_FlagGroup[1][u16idx] = HAL_MIU_Read2Byte(REG_MIU1_SLIT_DBG_VALUE);
    }

    HAL_MIU_WriteRegBit(REG_MIU0_SLIT_DBG_CTRL, REG_MIU_SLIT_DBG_EN, DISABLE);
    HAL_MIU_WriteRegBit(REG_MIU1_SLIT_DBG_CTRL, REG_MIU_SLIT_DBG_EN, DISABLE);

*****Remove MIU Slit function*****/

    return TRUE;
}

MS_BOOL HAL_MIU_Restore(void)
{
//    MS_U16 u16Data;
    MS_U16 u16idx;

    HAL_MIU_Write2Byte(MIU_PROTECT_EN, 0x0);
    HAL_MIU_Write2Byte(MIU2_PROTECT_EN, 0x0);

    for(u16idx = 1; u16idx < MIU_BANK_SIZE ; u16idx++)
    {
        HAL_MIU_Write2Byte(MIU_PROTECT_BASE+u16idx*2, MIU_PROTECT_BANK[0][u16idx]);
        HAL_MIU_Write2Byte(MIU2_PROTECT_BASE+u16idx*2, MIU_PROTECT_BANK[1][u16idx]);

        HAL_MIU_Write2Byte(MIU_ARBB_REG_BASE+u16idx*2, MIU_ARBB_T[0][u16idx]);
        HAL_MIU_Write2Byte(MIU1_ARBB_REG_BASE+u16idx*2, MIU_ARBB_T[1][u16idx]);
        HAL_MIU_Write2Byte(MIU2_ARBB_REG_BASE+u16idx*2, MIU_ARBB_T[2][u16idx]);
}

    for(u16idx = 0; u16idx < (MIU_BANK_SIZE/2); u16idx++)
    {
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP1+u16idx*2, MIU_BLOCK_ARBITER[1][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP2+u16idx*2, MIU_BLOCK_ARBITER[2][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP3+u16idx*2, MIU_BLOCK_ARBITER[3][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP4+u16idx*2, MIU_BLOCK_ARBITER[4][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP5+u16idx*2, MIU_BLOCK_ARBITER[5][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP6+u16idx*2, MIU_BLOCK_ARBITER[6][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP7+u16idx*2, MIU_BLOCK_ARBITER[7][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP8+u16idx*2, MIU_BLOCK_ARBITER[8][u16idx]);
        HAL_MIU_Write2Byte(MIU_BLOCK_ARBITER_GP9+u16idx*2, MIU_BLOCK_ARBITER[9][u16idx]);
    }

/*****Remove MIU Slit function*****

    // ID enable
    HAL_MIU_Write2Byte(REG_MIU0_SLIT_PROTECT0_ID_ENABLE, MIU_SLIT_PROTECT0_ID_ENABLE_T[0]);
    HAL_MIU_Write2Byte(REG_MIU1_SLIT_PROTECT0_ID_ENABLE, MIU_SLIT_PROTECT0_ID_ENABLE_T[1]);

    // Low Bound
    HAL_MIU_Write2Byte(REG_MIU0_SLIT_LOW_BOUND , MIU_SLIT_LOW_BOUND_T[0]);
    HAL_MIU_Write2Byte(REG_MIU1_SLIT_LOW_BOUND , MIU_SLIT_LOW_BOUND_T[1]);

    // High bound
    HAL_MIU_Write2Byte(REG_MIU0_SLIT_HIGH_BOUND, MIU_SLIT_HIGH_BOUND_T[0]);
    HAL_MIU_Write2Byte(REG_MIU1_SLIT_HIGH_BOUND, MIU_SLIT_HIGH_BOUND_T[1]);

    //Slits
    HAL_MIU_Write2Byte(REG_MIU0_SLIT_ADDR_BASE, 0x0);
    HAL_MIU_Write2Byte(REG_MIU1_SLIT_ADDR_BASE, 0x0);
    //update 511 range
    for(u16idx = 0; u16idx < SLIT_NUM; u16idx++)
    {
        u16Data = Slit_FlagGroup[0][u16idx] << 13;
        u16Data |= (REG_MIU_SLIT_UPDATE_EN | BIT0);
        HAL_MIU_Write2Byte(REG_MIU0_SLIT_UPDATE_CTRL, u16Data);

        u16Data = Slit_FlagGroup[1][u16idx] << 13;
        u16Data |= (REG_MIU_SLIT_UPDATE_EN | BIT0);
        HAL_MIU_Write2Byte(REG_MIU0_SLIT_UPDATE_CTRL, u16Data);
    }
    HAL_MIU_WriteRegBit(REG_MIU0_SLIT_UPDATE_CTRL, (MS_U8)REG_MIU_SLIT_UPDATE_VALUE, DISABLE);
    HAL_MIU_WriteRegBit(REG_MIU1_SLIT_UPDATE_CTRL, (MS_U8)REG_MIU_SLIT_UPDATE_VALUE, DISABLE);

    // Enable MIU protect
    HAL_MIU_Write2Byte(MIU_PROTECT_EN,MIU_PROTECT_EN_T[0]);
    HAL_MIU_Write2Byte(MIU2_PROTECT_EN,MIU_PROTECT_EN_T[1]);

*****Remove MIU Slit function*****/

    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_SlitInit
/// @brief \b Function  \b Description: Initialize slit sram
/// @return             \b 0: Fail 1: OK
////////////////////////////////////////////////////////////////////////////////
MS_BOOL HAL_MIU_SlitInit(void)
{
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: HAL_MIU_SetSlitRange()
/// @brief \b Function \b Description:  Enable/Disable MIU Slit range
/// @param u8Blockx        \b IN     : MIU Block to protect (5)
/// @param *pu8ProtectId   \b IN     : Allow specified client IDs to write
/// @param u64BusStart     \b IN     : Starting bus address
/// @param u64BusEnd       \b IN     : End bus address
/// @param bSetFlag        \b IN     : Disable or Enable MIU protection
///                                      - -Disable(0)
///                                      - -Enable(1)
/// @param <OUT>           \b None    :
/// @param <RET>           \b None    :
/// @param <GLOBAL>        \b None    :
////////////////////////////////////////////////////////////////////////////////
MS_BOOL HAL_MIU_SetSlitRange(
                          MS_U8    u8Blockx,
                          MS_U32   *pu32ProtectId,
                          MS_PHY   u64BusStart,
                          MS_PHY   u64BusEnd,
                          MS_BOOL  bSetFlag
                         )
{
/*****Remove MIU Slit function*****
    MS_U32 u32RegAddrIDenable;
    MS_U32 u32RegProtectEn;
    MS_U32 u32RegLowBound;
    MS_U32 u32RegHighBound;
    MS_U32 u32RegAddrBase;
    MS_U32 u32RegUpdateCtrl;
    MS_U32 u32RegUpdateLength;
    MS_PHY u64LowBoundOffset;
    MS_PHY u64HighBoundOffset;
    MS_PHY u64PhyStart;
    MS_PHY u64PhyEnd;
    MS_U32 u32RegId;
    MS_U32 u32RegProtectLog;
    MS_U32 u32RegSlitId0Ctl;
    MS_U16 u16Data;
    MS_U8  u8MiuSel_start;
    MS_U8  u8MiuSel_end;
    MS_U8  u8MiuSel;

    u64PhyStart = HAL_MIU_BA2PA(u64BusStart);
    u64PhyEnd = HAL_MIU_BA2PA(u64BusEnd);

    // Get MIU selection and offset
    _phy_to_miu_offset(u8MiuSel_start, u64HighBoundOffset, u64PhyEnd);
    _phy_to_miu_offset(u8MiuSel_end, u64LowBoundOffset, u64PhyStart);

    if(u8MiuSel_start != u8MiuSel_end)
    {
        MIU_HAL_ERR("u8MiuSel_start %d u8MiuSel_end %d not equal\n", u8MiuSel_start, u8MiuSel_end);
        return FALSE;
    }
    else
        u8MiuSel = u8MiuSel_start;

    // Incorrect Block ID
    if((u8Blockx < E_SLIT_0) || (u8Blockx >= E_MIU_BLOCK_NUM))
    {
        MIU_HAL_ERR("u8Blockx = %d Out of the number of protect device\n", u8Blockx );
        return FALSE;
    }
    else if(((u64PhyStart & ((1UL << MIU_SLIT_SHIFT) -1)) != 0) || ((u64PhyEnd & ((1UL << MIU_SLIT_SHIFT) -1)) != 0))
    {
        MIU_HAL_ERR("u64PhyStart = 0x%llx\n", (unsigned long long)u64PhyStart);
        MIU_HAL_ERR("u64PhyEnd = 0x%llx\n", (unsigned long long)u64PhyEnd);
        MIU_HAL_ERR("Protected address should be aligned to 1MB\n");
        return FALSE;
    }
    else if(u64PhyStart >= u64PhyEnd)
    {
        MIU_HAL_ERR("u64StartOffset = 0x%llx\n", (unsigned long long)u64PhyStart);
        MIU_HAL_ERR("u64PhyEnd = 0x%llx\n", (unsigned long long)u64PhyEnd);
        MIU_HAL_ERR("Start address is equal to or more than end address\n");
        return FALSE;
    }

    if(u8MiuSel == E_CHIP_MIU_0)
    {
        u32RegLowBound = REG_MIU0_SLIT_LOW_BOUND;
        u32RegHighBound = REG_MIU0_SLIT_HIGH_BOUND;
        u32RegAddrIDenable = REG_MIU0_SLIT_PROTECT0_ID_ENABLE;
        u32RegProtectEn = REG_MIU0_SLIT_PROTECT_EN;
        u32RegId = MIU_PROTECT0_ID0;
        u32RegProtectLog = REG_MIU_PROTECT_LOG;
        u32RegSlitId0Ctl = REG_MIU0_SLIT_RW_ID0;
        u32RegAddrBase = REG_MIU0_SLIT_ADDR_BASE;
        u32RegUpdateLength = REG_MIU0_SLIT_UPDATE_LENGTH;
        u32RegUpdateCtrl = REG_MIU0_SLIT_UPDATE_CTRL;
    }
    else if(u8MiuSel == E_CHIP_MIU_1)
    {
        u32RegLowBound = REG_MIU1_SLIT_LOW_BOUND;
        u32RegHighBound = REG_MIU1_SLIT_HIGH_BOUND;
        u32RegAddrIDenable = REG_MIU1_SLIT_PROTECT0_ID_ENABLE;
        u32RegProtectEn = REG_MIU1_SLIT_PROTECT_EN;
        u32RegId = MIU2_PROTECT0_ID0;
        u32RegProtectLog = REG_MIU2_PROTECT_LOG;
        u32RegSlitId0Ctl = REG_MIU1_SLIT_RW_ID0;
        u32RegAddrBase = REG_MIU1_SLIT_ADDR_BASE;
        u32RegUpdateLength = REG_MIU1_SLIT_UPDATE_LENGTH;
        u32RegUpdateCtrl = REG_MIU1_SLIT_UPDATE_CTRL;
    }
    else
    {
        MIU_HAL_ERR("%s not support MIU%u!\n", __FUNCTION__, u8MiuSel );
        return FALSE;
    }

    // Offset software block ID to get hardware one
    u8Blockx = u8Blockx - E_MIU_SLIT_0 + E_MIU_BLOCK_7 + 1;

    // Disable MIU protect
    HAL_MIU_WriteRegBit(u32RegProtectEn+1, (MS_U8)(BIT13>>8), DISABLE);

    if(bSetFlag)
    {
        // Set Protect IDs
        if(HAL_MIU_SetGroupID(u8MiuSel, u8Blockx, pu32ProtectId, u32RegId, u32RegAddrIDenable) == FALSE)
        {
            return FALSE;
        }

        // Low Bound
        u16Data = (MS_U16)(u64LowBoundOffset >> MIU_SLIT_SHIFT);   //1MB unit
        HAL_MIU_Write2Byte(u32RegLowBound , u16Data);

        // High bound
        u16Data = (MS_U16)((u64HighBoundOffset >> MIU_SLIT_SHIFT) - 1);   //1MB unit
        HAL_MIU_Write2Byte(u32RegHighBound, u16Data);

        // Enable MIU protect
        HAL_MIU_WriteRegBit(u32RegProtectEn+1, (MS_U8)(BIT13>>8), ENABLE);
    }
    else
    {
        // Reset Slits Table
        HAL_MIU_Write2Byte(u32RegAddrBase, 0x0);

        //update 511 range to 0
        HAL_MIU_Write2Byte(u32RegUpdateLength, SLIT_NUM-1);
        //enable update table
        HAL_MIU_WriteRegBit(u32RegUpdateCtrl+1, (MS_U8)(REG_MIU_SLIT_UPDATE_EN>>8), ENABLE);

        // Reset Protect IDs
        HAL_MIU_ResetGroupID(u8MiuSel, u8Blockx, NULL, u32RegId, u32RegAddrIDenable);
    }

    // clear log
    HAL_MIU_Write2BytesBit(u32RegProtectLog, TRUE, REG_MIU_PROTECT_LOG_CLR);
    HAL_MIU_Write2BytesBit(u32RegProtectLog, FALSE, REG_MIU_PROTECT_LOG_CLR);
*****Remove MIU Slit function*****/
    return TRUE;
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
MS_BOOL HAL_MIU_Slits(MS_U8 u8Blockx, MS_PHY u64SlitsStart, MS_PHY u64SlitsEnd, MS_BOOL bSetFlag)
{
/*****Remove MIU Slit function*****
    MS_PHY u64PhyStart;
    MS_PHY u64PhyEnd;
    MS_PHY u64StartOffset;
    MS_PHY u64EndOffset;
    MS_U32 u32RegAddr0;
    MS_U32 u32RegLowBound;
    MS_U32 u32RegHighBound;
    MS_U32 u32RegUpdateCtrl;
    MS_U32 u32RegUpdateLength;
    MS_U32 u32RegAddrBase;
    MS_U16 u16LowBoundidx;
    MS_U16 u16HighBoundidx;
    MS_U16 u16Headidx;
    MS_U16 u16Tailidx;
    MS_U16 u16Val;
    MS_U8  u8MiuSel_start;
    MS_U8  u8MiuSel_end;
    MS_U8  u8MiuSel;

    u64PhyStart = HAL_MIU_BA2PA(u64SlitsStart);
    u64PhyEnd = HAL_MIU_BA2PA(u64SlitsEnd);

    // Get MIU selection and offset
    _phy_to_miu_offset(u8MiuSel_end, u64EndOffset, u64PhyEnd);
    _phy_to_miu_offset(u8MiuSel_start, u64StartOffset, u64PhyStart);

    //_phy_to_miu_offset(u8MiuSel_start, u64EndOffset, u64PhyEnd);
    //_phy_to_miu_offset(u8MiuSel_end, u64StartOffset, u64PhyStart);

    if(u8MiuSel_start != u8MiuSel_end)
    {
        MIU_HAL_ERR("u8MiuSel_start %d u8MiuSel_end %d not equal ===\n", u8MiuSel_start, u8MiuSel_end);
        return FALSE;
    }
    else
        u8MiuSel = u8MiuSel_start;

    // Incorrect Block ID
    if((u8Blockx < E_SLIT_0) || (u8Blockx >= E_MIU_BLOCK_NUM))
    {
        MIU_HAL_ERR("u8Blockx = %d Out of the number of protect device\n", u8Blockx );
        return FALSE;
    }
    else if(((u64SlitsStart & ((1 << MIU_SLIT_SHIFT) -1)) != 0) || ((u64SlitsEnd & ((1 << MIU_SLIT_SHIFT) -1)) != 0))
    {
        MIU_HAL_ERR("slit address should be aligned to 1MB\n");
        return FALSE;
    }
    else if(u64PhyStart >= u64PhyEnd)
    {
        MIU_HAL_ERR("u64StartOffset = 0x%llx\n", (unsigned long long)u64PhyStart);
        MIU_HAL_ERR("u64PhyEnd = 0x%llx\n", (unsigned long long)u64PhyEnd);
        MIU_HAL_ERR("Start address is equal to or more than end address\n");
        return FALSE;
    }

    if(u8MiuSel == E_CHIP_MIU_0)
    {
        u32RegAddr0 = REG_MIU0_SLIT_RW_ID0;
        u32RegLowBound = REG_MIU0_SLIT_LOW_BOUND;
        u32RegHighBound = REG_MIU0_SLIT_HIGH_BOUND;
        u32RegUpdateLength = REG_MIU0_SLIT_UPDATE_LENGTH;
        u32RegUpdateCtrl = REG_MIU0_SLIT_UPDATE_CTRL;
        u32RegAddrBase = REG_MIU0_SLIT_ADDR_BASE;
    }
    else if(u8MiuSel == E_CHIP_MIU_1)
    {
        u32RegAddr0 = REG_MIU1_SLIT_RW_ID0;
        u32RegLowBound = REG_MIU1_SLIT_LOW_BOUND;
        u32RegHighBound = REG_MIU1_SLIT_HIGH_BOUND;
        u32RegUpdateLength = REG_MIU1_SLIT_UPDATE_LENGTH;
        u32RegUpdateCtrl = REG_MIU1_SLIT_UPDATE_CTRL;
        u32RegAddrBase = REG_MIU1_SLIT_ADDR_BASE;
    }
    else
    {
        MIU_HAL_ERR("%s not support MIU%u!\n", __FUNCTION__, u8MiuSel );
        return FALSE;
    }

    // Get low bound base idx, an index means 1MB entry
    u16LowBoundidx = HAL_MIU_Read2Byte(u32RegLowBound);
    u16HighBoundidx = HAL_MIU_Read2Byte(u32RegHighBound);

    // Get head and tail index, an index means 1MB entry
    u16Headidx = u64StartOffset >> MIU_SLIT_SHIFT;
    u16Tailidx = (u64EndOffset >> MIU_SLIT_SHIFT) - 1;

    if(u16Headidx < u16LowBoundidx)
    {
        MIU_HAL_ERR("u16Headidx=%d shuold not be less than u16LowBoundidx=%d\n",u16Headidx, u16LowBoundidx);
        return FALSE;
    }
    else if(u16Tailidx > u16HighBoundidx)
    {
        MIU_HAL_ERR("u16Tailidx=%d shuold not be larger than u16HighBoundidx=%d\n",u16Tailidx, u16HighBoundidx);
        return FALSE;
    }

    // Offset head and tail index with "u16LowBoundidx"
    u16Headidx = u16Headidx - u16LowBoundidx;
    u16Tailidx = u16Tailidx - u16LowBoundidx;

    // Reset Slits Table
    HAL_MIU_Write2Byte(u32RegAddrBase, u16Headidx);

    //update (u16Tailidx-u16Headidx) range to 1
    u16Val = REG_MIU_SLIT_UPDATE_VALUE | (u16Tailidx-u16Headidx);
    HAL_MIU_Write2Byte(u32RegUpdateLength, u16Val);

    //enable update table
    HAL_MIU_WriteRegBit(u32RegUpdateCtrl+1, (MS_U8)(REG_MIU_SLIT_UPDATE_EN>>8), ENABLE);
    HAL_MIU_WriteRegBit(u32RegUpdateCtrl+1, (MS_U8)(REG_MIU_SLIT_UPDATE_EN>>8), DISABLE);
*****Remove MIU Slit function*****/
    return TRUE;
}

void HAL_MIU_DumpMem(phys_addr_t addr, MS_U32 len)
{
    MS_U8 *ptr = (MS_U8 *)addr;
    MS_U32 i;

    MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "\n ===== Dump@%lx =====\n", (long unsigned int)ptr);
    for (i=0; i<len; i++)
    {
        if ((u32)i%0x10UL ==0)
        {
            MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "%lx: ", (long unsigned int)ptr);
        }
        if (*ptr < 0x10UL)
        {
            MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "0%x ", *ptr);
        }
        else
        {
            MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "%x ", *ptr);
        }
        if ((u32)i%0x10UL == 0x0fUL)
        {
            MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "\n");
        }
        ptr++;
    }
    MIU_DEBUG_LOG(KERN_NOTICE, E_MIU_DBGLV_NOTICE, "\n");
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
            regAddr = MIU2_PROTECT_DDR_SIZE;
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

#define GET_HIT_CLIENT(regval)      (regval >> 8)

MS_BOOL HAL_MIU_Kernel_GetProtectInfo(MS_U8 u8MiuDev, MIU_PortectInfo *pInfo)
{
    MS_U16 ret = 0;
    MS_U16 loaddr = 0;
    MS_U16 hiaddr = 0;
    MS_U16 hitblock = 0;
    MS_U32 u32Address = 0;
    MS_U32 u32Reg;

    if(u8MiuDev == E_MIU_0)
    {
        u32Reg = MIU_PROTECT_BASE;
    }
    else if(u8MiuDev == E_MIU_1)
    {
        u32Reg = MIU1_PROTECT_BASE;
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
    hitblock = (HAL_MIU_Read2Byte(u32Reg+REG_MIU_PROTECT_BLOCK) & 0xF);

    pInfo->bHit = false;

    if (REG_MIU_PROTECT_HIT_FALG & ret)
    {
        pInfo->bHit = TRUE;

        pInfo->u8Block = (MS_U8)(hitblock);
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
