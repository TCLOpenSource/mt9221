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
#include <asm/barrier.h>

#if defined(CONFIG_MIPS)
#include <asm/mips-boards/prom.h>
#include "mhal_chiptop_reg.h"
#elif defined(CONFIG_ARM)
#include <prom.h>
#include <asm/mach/map.h>
#elif defined(CONFIG_ARM64)
#include <asm/arm-boards/prom.h>
#include <asm/mach/map.h>
#endif
#include "chip_setup.h"
#include <linux/bug.h>

//-------------------------------------------------------------------------------------------------
//  Define
//-------------------------------------------------------------------------------------------------
#define MIU_HAL_ERR(fmt, args...)   printk(KERN_ERR "miu hal error %s:%d" fmt,__FUNCTION__,__LINE__,## args)
#define MIU_HAL_WARN(fmt, args...)   printk(KERN_ERR "miu hal warning %s:%d" fmt,__FUNCTION__,__LINE__,## args)

//[MIU][HAL][005] Update MIU Client Table [START]
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
/* 3 */    MIU_CLIENT_DUMMY,\
/* 4 */    MIU_CLIENT_SECURE_R2_RW,\
/* 5 */    MIU_CLIENT_DUMMY, \
/* 6 */    MIU_CLIENT_DUMMY,\
/* 7 */    MIU_CLIENT_PM51_RW, \
/* 8 */    MIU_CLIENT_MHEG_5_RW, \
/* 9 */    MIU_CLIENT_USB_UHC0_RW, \
/*10 */    MIU_CLIENT_USB_UHC1_RW, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_MVD_BBU_RW, \
/*13 */    MIU_CLIENT_EMAC_RW, \
/*14 */    MIU_CLIENT_BDMA_RW, \
/*15 */    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP1  \
/* 0 */    MIU_CLIENT_DUMMY, \
/* 1 */    MIU_CLIENT_DEMOD_RW, \
/* 2 */    MIU_CLIENT_DUMMY, \
/* 3 */    MIU_CLIENT_JPD720P_RW, \
/* 4 */    MIU_CLIENT_DUMMY, \
/* 5 */    MIU_CLIENT_DUMMY,\
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_TSP_ORZ_W, \
/* 9 */    MIU_CLIENT_TSP_ORZ_R, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_VD_TTXSL_W, \
/*12 */    MIU_CLIENT_VD_COMB_W, \
/*13 */    MIU_CLIENT_VD_COMB_R,  \
/*14 */    MIU_CLIENT_ZDEC_RW, \
/*15 */    MIU_CLIENT_ZDEC_ACP_RW

#define MIU_CLIENT_GP2  \
/* 0 */    MIU_CLIENT_DUMMY, \
/* 1 */    MIU_CLIENT_GE_RW, \
/* 2 */    MIU_CLIENT_MIIC0_RW, \
/* 3 */    MIU_CLIENT_UART_DMA_RW, \
/* 4 */    MIU_CLIENT_MVD_RW, \
/* 5 */    MIU_CLIENT_DUMMY, \
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_DUMMY, \
/* 9 */    MIU_CLIENT_NAND_RW, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_DSCRMB_RW, \
/*12 */    MIU_CLIENT_DUMMY, \
/*13 */    MIU_CLIENT_TSP_SEC_RW, \
/*14 */    MIU_CLIENT_TSP_FILEIN_RW,\
/*15 */    MIU_CLIENT_TSO_RW

#define MIU_CLIENT_GP3  \
/* 0 */    MIU_CLIENT_AUDIO_RW, \
/* 1 */    MIU_CLIENT_EVD_R2D_RW, \
/* 2 */    MIU_CLIENT_EVD_R2I_R, \
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

#define MIU_CLIENT_GP4  \
/* 0 */    MIU_CLIENT_HVD_BBU_R, \
/* 1 */    MIU_CLIENT_DUMMY, \
/* 2 */    MIU_CLIENT_HVD_0_RW, \
/* 3 */    MIU_CLIENT_EVD_BBU_RW, \
/* 4 */    MIU_CLIENT_EVD_RW,\
/* 5 */    MIU_CLIENT_DUMMY, \
/* 6 */    MIU_CLIENT_R2_BBU_RW, \
/* 7 */    MIU_CLIENT_DUMMY, \
/* 8 */    MIU_CLIENT_DUMMY, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_DUMMY, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_DUMMY, \
/*13 */    MIU_CLIENT_DUMMY, \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_DUMMY

#define MIU_CLIENT_GP5  \
/* 0 */    MIU_CLIENT_SC_IPMAIN_R, \
/* 1 */    MIU_CLIENT_SC_OPMAIN_R, \
/* 2 */    MIU_CLIENT_DS_R, \
/* 3 */    MIU_CLIENT_OD_R, \
/* 4 */    MIU_CLIENT_SC_LD_R, \
/* 5 */    MIU_CLIENT_AUTO_DOWNLOAD_R, \
/* 6 */    MIU_CLIENT_DUMMY, \
/* 7 */    MIU_CLIENT_SC_ODW_R, \
/* 8 */    MIU_CLIENT_GOP0_R, \
/* 9 */    MIU_CLIENT_GOP1_R, \
/*10 */    MIU_CLIENT_GOP2_R, \
/*11 */    MIU_CLIENT_MVOP0_0_R, \
/*12 */    MIU_CLIENT_SC_HDR_DMA_R, \
/*13 */    MIU_CLIENT_MFDEC0_0_R, \
/*14 */    MIU_CLIENT_MVOP0_1_R, \
/*15 */    MIU_CLIENT_VE_R

#define MIU_CLIENT_GP6  \
/* 0 */    MIU_CLIENT_SC_IPMAIN_W, \
/* 1 */    MIU_CLIENT_SC_OPMAIN_W, \
/* 2 */    MIU_CLIENT_DUMMY, \
/* 3 */    MIU_CLIENT_OD_W, \
/* 4 */    MIU_CLIENT_SC_LD_W, \
/* 5 */    MIU_CLIENT_SC_LD1_W, \
/* 6 */    MIU_CLIENT_VE_W, \
/* 7 */    MIU_CLIENT_SC_ODW_W, \
/* 8 */    MIU_CLIENT_AUTO_UPLOAD_W, \
/* 9 */    MIU_CLIENT_DUMMY, \
/*10 */    MIU_CLIENT_SC_PDW_W, \
/*11 */    MIU_CLIENT_DUMMY, \
/*12 */    MIU_CLIENT_SC_HDR_DMA_W, \
/*13 */    MIU_CLIENT_DUMMY, \
/*14 */    MIU_CLIENT_DUMMY, \
/*15 */    MIU_CLIENT_SC_ADC_DMA_W

#define MIU_CLIENT_GP15  \
/* 0 */    MIU_CLIENT_MIPS_RW, \
/* 1 */    MIU_CLIENT_G3D_RW, \
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

//[MIU][HAL][005] Update MIU Client Table [END]

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
#define IDNUM_KERNELPROTECT (7)

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
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP_DUMMY,
        MIU_CLIENT_GP15,
    }
};


//[MIU][HAL][006] Check data structure of MIU client table [END]

static MS_U32 clientId_KernelProtect[MIU_MAX_PROTECT_ID_NUM] =
{
    MIU_CLIENT_MIPS_RW,     //MIPS W
    MIU_CLIENT_NAND_RW,
    MIU_CLIENT_USB_UHC0_RW, //USB0
    MIU_CLIENT_USB_UHC1_RW, //USB1
    MIU_CLIENT_G3D_RW,      //GPU
    MIU_CLIENT_EMAC_RW,
    MIU_CLIENT_GE_RW,
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

static MS_BOOL IDEnables[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][MIU_MAX_PROTECT_ID_NUM];
static MS_U32 IDs[MIU_MAX_DEVICE][MIU_MAX_PROTECT_ID_GROUP_NUM][MIU_MAX_PROTECT_ID_NUM];
MS_U32 ProtectGroupSelect[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK] = {{1,1,1,1,1,1,1,1}};
MS_U32 ProtectAddr[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][MIU_PROTECT_START_END];   //MIU_PROTECT_START_END : 0->start, 1->end

//-------------------------------------------------------------------------------------------------
//  MIU Debug Message Level
//-------------------------------------------------------------------------------------------------
static MS_U32 mstar_debug = E_MIU_DBGLV_ERR;
static MS_U32 miu_hit_panic = 0;

//index 0 ~ MIU_MAX_PROTECT_ID_NUM-1 : IDs
//index MIU_MAX_PROTECT_ID_NUM : ID's number
//index MIU_MAX_PROTECT_ID_NUM+1 : start address
//index MIU_MAX_PROTECT_ID_NUM+2 : end address
//-------------------------------------------------------------------------------------------------
//  MTLB HAL internal function
//-------------------------------------------------------------------------------------------------
MS_PHY HAL_MIU_BA2PA(MS_PHY u64BusAddr)
{
    MS_PHY u64PhyAddr = 0x0UL;

    // pa = ba - offset
    if( (u64BusAddr >= ARM_MIU0_BUS_BASE) && (u64BusAddr < ARM_MIU1_BUS_BASE) ) // MIU0
        u64PhyAddr = u64BusAddr - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;

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
/// @brief \b Function  \b Name: HAL_MIU_Read2Byte
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
/// @brief \b Function  \b Name: HAL_MIU_Read4Byte
/// @brief \b Function  \b Description: read 4 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b MS_U16
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////
MS_U32 HAL_MIU_Read4Byte(MS_U32 u32RegAddr)
{
    return (HAL_MIU_Read2Byte(u32RegAddr) | HAL_MIU_Read2Byte(u32RegAddr+2) << 16);
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
/// @brief \b Function  \b Name: HAL_MIU_Write2BytesBit
/// @brief \b Function  \b Description: write 2 Byte data bit
/// @param <IN>         \b u32RegAddr: register address
/// @param <IN>         \b bEnable : write enable ro disable at mask
/// @param <IN>         \b u16Mask : Mask
/// @param <OUT>        \b None :
/// @param <RET>        \b TRUE: Ok FALSE: Fail
/// @param <GLOBAL>     \b None :
////////////////////////////////////////////////////////////////////////////////

static void HAL_MIU_Write2BytesBit(MS_U32 u32RegOffset, MS_BOOL bEnable, MS_U16 u16Mask)
{
    MS_U16 val = HAL_MIU_Read2Byte(u32RegOffset);
    val = (bEnable) ? (val | u16Mask) : (val & ~u16Mask);
    HAL_MIU_Write2Byte(u32RegOffset, val);
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

//[[KERNEL PROTECT]]
static void HAL_MIU_SetProtectID(MS_U32 u32Reg, MS_U8 u8MiuDev, MS_U32 u32ClientID)
{
    MS_S16 sVal = HAL_MIU_GetClientInfo(u8MiuDev, (eMIUClientID)u32ClientID);
    MS_S16 sIDVal;

    if (0 > sVal)
        sVal = 0;

    HAL_MIU_WriteByte(u32Reg, sVal & 0xFF);
}

static MS_BOOL HAL_MIU_CheckProtectID_KernelProtect(MS_U32 *pu32ProtectId)
{
    MS_U32  u32index;

    for(u32index = 0; u32index < MIU_MAX_PROTECT_ID_NUM; u32index++)
    {
        if(pu32ProtectId[u32index] != clientId_KernelProtect[u32index])
        {
            return FALSE;
        }
    }

    return TRUE;
}

static MS_U8 HAL_MIU_SelIDGroup(MS_U8 u8MiuSel, MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_BOOL bisKernellist)
{
    MS_BOOL bprotectgroup;
    MS_U32 u32ID;
    MS_U32 u32SelIdGroup;
    MS_U32 u32IdIndex;
    MS_U32 u32index;
    MS_U8  u8isfound0, u8isfound1;

    //if it is kernel protect, Select Group 0 for kernel protect
    if(bisKernellist == TRUE)
        return E_MIU_ID_GROUP_0;

    //if it is not kernel protect, select which protect group
    for(u32SelIdGroup = 0; u32SelIdGroup < MIU_MAX_PROTECT_ID_GROUP_NUM; u32SelIdGroup++)
    {
        bprotectgroup = TRUE;
        for(u32IdIndex = 0; u32IdIndex < MIU_MAX_PROTECT_ID_NUM; u32IdIndex++)
        {
            u32ID = pu32ProtectId[u32IdIndex];

            if(u32ID == 0)
                continue;

            u8isfound0 = FALSE;
            //check if repeat
            for(u32index = 0; u32index < MIU_MAX_PROTECT_ID_NUM; u32index++)
            {
                if(IDs[u8MiuSel][u32SelIdGroup][u32index] == u32ID)
                {
                    u8isfound0 = TRUE;
                    break;
                }
            }
            //check if protect list overflow
            if(u8isfound0 != TRUE)
            {
                u8isfound1 = FALSE;
                for(u32index = 0; u32index < MIU_MAX_PROTECT_ID_NUM; u32index++)
                {
                    if(IDs[u8MiuSel][u32SelIdGroup][u32index] == 0)
                    {
                        u8isfound1 = TRUE;
                        break;
                    }
                }
                //ID overflow
                if(u8isfound1 == FALSE)
                {
                    //this protect group can not protect all protect id
                    bprotectgroup = FALSE;
                    break;
                }
            }
        }
        if(bprotectgroup)
        {
            return u32SelIdGroup;
        }
    }
    MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Err: No protect ID group can use.\n");
    return E_MIU_ID_GROUP_NONE;
}

static MS_BOOL HAL_MIU_SetGroupID(MS_U8 u8MiuSel, MS_U8 u8Blockx, MS_U32 *pu32ProtectId, MS_U32 u32RegAddr, MS_U32 u32RegAddrG1, MS_U32 u32RegAddrIDenable, MS_U32 u32SelIdGroup, MS_BOOL bisKernellist)
{
    MS_U8 u8isfound0, u8isfound1;
    MS_U16 u16idenable;
    MS_U32 u32ID, u32RegID;
    MS_U32 u32index0, u32index1;
    static MS_U32 IDs_tmp[MIU_MAX_PROTECT_ID_NUM];

    switch(u32SelIdGroup)
    {
        case E_MIU_ID_GROUP_0:
            u32RegID = u32RegAddr;
            break;
        case E_MIU_ID_GROUP_1:
            u32RegID = u32RegAddrG1;
            break;
        default:
            return FALSE;
    }

    if(bisKernellist == TRUE)
    {
        for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
        {
            IDs_tmp[u32index0] = IDs[u8MiuSel][u32SelIdGroup][u32index0];

            //reset IDenables for protect u8Blockx
            IDEnables[u8MiuSel][u8Blockx][u32index0] = 0;

            u32ID = pu32ProtectId[u32index0];

            //Unused ID
            if(u32ID == 0)
                continue;
            IDs[u8MiuSel][u32SelIdGroup][u32index0] = u32ID;
            IDEnables[u8MiuSel][u8Blockx][u32index0] = 1;
        }
        for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
        {
            u8isfound0 = FALSE;
            for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID_NUM; u32index1++)
            {
                if(IDs_tmp[u32index0] == IDs[u8MiuSel][u32SelIdGroup][u32index1])
                {
                    IDEnables[u8MiuSel][u8Blockx][u32index1] = 1;
                    u8isfound0 = TRUE;
                    break;
                }
            }
            //Need to create new ID in IDs
            if(u8isfound0 != TRUE)
            {
                u8isfound1 = FALSE;

                for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID_NUM; u32index1++)
                {
                    if(IDs[u8MiuSel][u32SelIdGroup][u32index1] == 0)
                    {
                        IDs[u8MiuSel][u32SelIdGroup][u32index1] = IDs_tmp[u32index0];
                        IDEnables[u8MiuSel][u8Blockx][u32index1] = 1;
                        u8isfound1 = TRUE;
                        break;
                    }
                }

                //ID overflow
                if(u8isfound1 == FALSE)
                {
                    MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, " MIU PROTECT WARNING: NO ENOUGH PROTECT CLIENT SPACE CAN USE!!!\n");
                    break;
                }
            }
        }
    }
    else
    {
        //reset IDenables for protect u8Blockx
        for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
        {
            IDEnables[u8MiuSel][u8Blockx][u32index0] = 0;
        }

        for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
        {
            u32ID = pu32ProtectId[u32index0];

            //Unused ID
            if(u32ID == 0)
                continue;

            u8isfound0 = FALSE;

            for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID_NUM; u32index1++)
            {
                if(IDs[u8MiuSel][u32SelIdGroup][u32index1] == u32ID)
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

                for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID_NUM; u32index1++)
                {
                    if(IDs[u8MiuSel][u32SelIdGroup][u32index1] == 0)
                    {
                        IDs[u8MiuSel][u32SelIdGroup][u32index1] = u32ID;
                        IDEnables[u8MiuSel][u8Blockx][u32index1] = 1;
                        u8isfound1 = TRUE;
                        break;
                    }
                }

                //ID overflow
                if(u8isfound1 == FALSE)
                    break;
            }
        }
    }

    u16idenable = 0;

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
    {
        if(IDEnables[u8MiuSel][u8Blockx][u32index0] == 1)
            u16idenable |= (1<<u32index0);
    }

    HAL_MIU_Write2Byte(u32RegAddrIDenable, u16idenable);

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
    {
        HAL_MIU_SetProtectID(u32RegID + u32index0, u8MiuSel, IDs[u8MiuSel][u32SelIdGroup][u32index0]);
    }

    return TRUE;
}

static MS_BOOL HAL_MIU_ResetGroupID(MS_U8 u8MiuSel, MS_U8 u8Blockx, MS_U32 u32RegAddr, MS_U32 u32RegAddrG1, MS_U32 u32RegAddrIDenable, MS_U32 u32MiuProtectGroupSel, MS_U32 u32SelIdGroup)
{
    MS_U8 u8isIDNoUse;
    MS_U8 u8Data;
    MS_U16 u16idenable;
    MS_U32 u32RegID;
    MS_U32 u32index0, u32index1;

    switch(u32SelIdGroup)
    {
        case E_MIU_ID_GROUP_0:
            break;
        case E_MIU_ID_GROUP_1:
            u32RegID = u32RegAddrG1;
            break;
        default:
            return FALSE;
    }

    //reset IDenables for protect u8Blockx
    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
    {
        IDEnables[u8MiuSel][u8Blockx][u32index0] = 0;
    }

    u16idenable = 0x0UL;

    HAL_MIU_Write2Byte(u32RegAddrIDenable, u16idenable);

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
    {
        u8isIDNoUse  = FALSE;

        for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_BLOCK; u32index1++)
        {
            if(ProtectGroupSelect[u8MiuSel][u32index1] == u32SelIdGroup)
            {
                if(IDEnables[u8MiuSel][u32index1][u32index0] == 1)
                {
                    //protect ID is still be used
                    u8isIDNoUse  = FALSE;
                    break;
                }
                u8isIDNoUse  = TRUE;
            }
        }

        if(u8isIDNoUse == TRUE)
            IDs[u8MiuSel][u32SelIdGroup][u32index0] = 0;
    }

    for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
    {
        HAL_MIU_SetProtectID(u32RegID + u32index0, u8MiuSel, IDs[u8MiuSel][u32SelIdGroup][u32index0]);
    }

    u8Data = 1 << u8Blockx;
    ProtectGroupSelect[u8MiuSel][u8Blockx] = 0;
    HAL_MIU_WriteRegBit(u32MiuProtectGroupSel, u8Data, 0);

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
        MS_U32   *pu32ProtectId,
        MS_PHY   u64BusStart,
        MS_PHY   u64BusEnd,
        MS_BOOL  bSetFlag
        )
{
    MS_U32  u32RegAddr;
    MS_U32  u32RegAddrG1;
    MS_U32  u32RegAddrStart;
    MS_U32  u32RegAddrIDenable;
    MS_U32  u32MiuProtectEn;
    MS_U32  u32MiuProtectGroupSel;
    MS_U32  u32Data;
    MS_PHY  phy64StartOffset;
    MS_PHY  phy64EndOffset;
    MS_U8   u8Data;
    MS_U8   u8MiuSel;
    MS_PHY  phy64Start;
    MS_PHY  phy64End;
    MS_U32  u32SelIdGroup = 0;
    MS_BOOL bisKernellist;

    phy64Start = HAL_MIU_BA2PA(u64BusStart);
    phy64End = HAL_MIU_BA2PA(u64BusEnd);

    // Get MIU selection and offset
    _phy_to_miu_offset(u8MiuSel, phy64EndOffset, phy64End - 1); // use "u32End - 1" to avoid selecting wrong u8MiuSel and u32EndOffset, ex: 0xE0000000 will be MIU2_END and MIU3_START
    _phy_to_miu_offset(u8MiuSel, phy64StartOffset, phy64Start)

    phy64Start = phy64StartOffset;
    phy64End = phy64EndOffset + 1;  // due to we use u32End - 1 @_phy_to_miu_offset for getting u32EndOffset

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

    if(u8MiuSel >= MIU_MAX_DEVICE)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s not support MIU%u!\n", __FUNCTION__, u8MiuSel );
        return false;
    }

    //Check if Protect ID is kernel protect or not
    bisKernellist = HAL_MIU_CheckProtectID_KernelProtect(pu32ProtectId);

    //write_enable
    u8Data = 1 << u8Blockx;
    if(u8MiuSel == E_CHIP_MIU_0)
    {
        u32RegAddr = MIU_PROTECT0_ID0;
        u32RegAddrG1 = MIU_PROTECT0_GROUP1_ID0;
        u32MiuProtectEn = MIU_PROTECT_EN;
        u32MiuProtectGroupSel = MIU_PROTECT_GROUP_SEL;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStart = MIU_PROTECT0_START;
                u32RegAddrIDenable = MIU_PROTECT0_ID_ENABLE;
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStart = MIU_PROTECT1_START;
                u32RegAddrIDenable = MIU_PROTECT1_ID_ENABLE;
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStart = MIU_PROTECT2_START;
                u32RegAddrIDenable = MIU_PROTECT2_ID_ENABLE;
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStart = MIU_PROTECT3_START;
                u32RegAddrIDenable = MIU_PROTECT3_ID_ENABLE;
                break;
            case E_MIU_BLOCK_4:
                u32RegAddrStart = MIU_PROTECT4_START;
                u32RegAddrIDenable = MIU_PROTECT4_ID_ENABLE;
                break;
            case E_MIU_BLOCK_5:
                u32RegAddrStart = MIU_PROTECT5_START;
                u32RegAddrIDenable = MIU_PROTECT5_ID_ENABLE;
                break;
            case E_MIU_BLOCK_6:
                u32RegAddrStart = MIU_PROTECT6_START;
                u32RegAddrIDenable = MIU_PROTECT6_ID_ENABLE;
                break;
            case E_MIU_BLOCK_7:
                u32RegAddrStart = MIU_PROTECT7_START;
                u32RegAddrIDenable = MIU_PROTECT7_ID_ENABLE;
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
        //select ID Group
        u32SelIdGroup = HAL_MIU_SelIDGroup(u8MiuSel, u8Blockx, pu32ProtectId, bisKernellist);

        if(u32SelIdGroup == E_MIU_ID_GROUP_NONE)
        {
            return FALSE;
        }

        // Set Protect IDs
        if(HAL_MIU_SetGroupID(u8MiuSel, u8Blockx, pu32ProtectId, u32RegAddr, u32RegAddrG1, u32RegAddrIDenable, u32SelIdGroup, bisKernellist) == FALSE)
        {
            return FALSE;
        }

        if(bisKernellist == TRUE)
        {
            if( ((MS_U32)(phy64Start >> MIU_PAGE_SHIFT) ) != HAL_MIU_Read4Byte(u32RegAddrStart) || \
                ((MS_U32)((phy64End >> MIU_PAGE_SHIFT)-1)) != HAL_MIU_Read4Byte((u32RegAddrStart + 4)) )
            {
                MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, " MIU PROTECT WARNING: BLOCK%d IS COVER BY KERNEL PROTECT!!!\n", (int)u8Blockx);
            }
        }

        // Start Address
        u32Data = (MS_U32)(phy64Start >> MIU_PAGE_SHIFT);   //8k/unit
        HAL_MIU_Write4Byte(u32RegAddrStart , u32Data);
        ProtectAddr[u8MiuSel][u8Blockx][0] = u32Data;

        // End Address
        u32Data = (MS_U32)((phy64End >> MIU_PAGE_SHIFT)-1);   //8k/unit;
        HAL_MIU_Write4Byte(u32RegAddrStart + 4, u32Data);
        ProtectAddr[u8MiuSel][u8Blockx][1] = u32Data;

        // Select MIU protect group
        ProtectGroupSelect[u8MiuSel][u8Blockx] = u32SelIdGroup;
        HAL_MIU_WriteRegBit(u32MiuProtectGroupSel, u8Data, ProtectGroupSelect[u8MiuSel][u8Blockx]);

        // Enable MIU protect
        HAL_MIU_WriteRegBit(u32MiuProtectEn, u8Data, ENABLE);
    }
    else
    {
        //select ID Group
        u32SelIdGroup = ProtectGroupSelect[u8MiuSel][u8Blockx];
        // Reset Protect IDs
        HAL_MIU_ResetGroupID(u8MiuSel, u8Blockx, u32RegAddr, u32RegAddrG1, u32RegAddrIDenable, u32MiuProtectGroupSel, u32SelIdGroup);
    }

    // clear log
    HAL_MIU_Write2BytesBit(u32MiuProtectEn + REG_MIU_PROTECT_STATUS, TRUE, REG_MIU_PROTECT_LOG_CLR);
    HAL_MIU_Write2BytesBit(u32MiuProtectEn + REG_MIU_PROTECT_STATUS, FALSE, REG_MIU_PROTECT_LOG_CLR);

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

MS_U16 MIU_PROTECT_EN_T[MIU_MAX_DEVICE];
MS_U16 MIU_PROTECT_ID_ENABLE_T[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK];
MS_U16 MIU_PROTECT_START_T[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][2];
MS_U16 MIU_PROTECT_END_T[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][2];
MS_U16 MIU_GROUPCLIENT[MIU_MAX_DEVICE][MIU_MAX_PROTECT_ID_GROUP_NUM][MIU_MAX_PROTECT_ID_NUM];
MS_U16 MIU_GROUP_CLIENT_SELECT[MIU_MAX_DEVICE];
MS_U16 MIU_RQ_DATA[MIU_MAX_DEVICE][MIU_MAX_GROUP/2][0x80];
MS_U16 MIU_SELECT[MIU_MAX_CLIENT_GROUP_NUM];
MS_U16 u16grouppriority;
MS_U16 MIU_BW_CRTL[0x0E];

MS_BOOL HAL_MIU_Save(void)
{
    MS_U16 u16idx;
    MS_U16 u16idx1;

    // Enable MIU protect
    MIU_PROTECT_EN_T[0] = HAL_MIU_Read2Byte(MIU_PROTECT_EN);

    MIU_GROUP_CLIENT_SELECT[0] = HAL_MIU_ReadByte(MIU_PROTECT_GROUP_SEL);
    for(u16idx = 0; u16idx < MIU_MAX_PROTECT_BLOCK ; u16idx++)
    {
        u16idx1 = u16idx * 4;

        //protect ID enable
        MIU_PROTECT_ID_ENABLE_T[0][u16idx] = HAL_MIU_Read2Byte(MIU_PROTECT0_ID_ENABLE+u16idx*2);

        //miu protect addr
        MIU_PROTECT_START_T[0][u16idx][0] = HAL_MIU_Read2Byte(MIU_PROTECT0_START+u16idx1*2);
        MIU_PROTECT_START_T[0][u16idx][1] = HAL_MIU_Read2Byte(MIU_PROTECT0_START+(u16idx1+1)*2);

        MIU_PROTECT_END_T[0][u16idx][0] = HAL_MIU_Read2Byte(MIU_PROTECT0_START+(u16idx1+2)*2);
        MIU_PROTECT_END_T[0][u16idx][1] = HAL_MIU_Read2Byte(MIU_PROTECT0_START+(u16idx1+3)*2);
    }

    //protect ID
    for(u16idx = 0; u16idx < MIU_MAX_PROTECT_ID_NUM; u16idx++){
        MIU_GROUPCLIENT[0][0][u16idx] = HAL_MIU_ReadByte(MIU_PROTECT0_ID0 + u16idx);
        MIU_GROUPCLIENT[0][1][u16idx] = HAL_MIU_ReadByte(MIU_PROTECT0_GROUP1_ID0 + u16idx);
    }

    //RQ
    for( u16idx = 0; u16idx < 0x80; u16idx++ )
    {
        MIU_RQ_DATA[0][0][u16idx] = HAL_MIU_Read2Byte( MIU_RQ_E + u16idx*2 );
        MIU_RQ_DATA[0][1][u16idx] = HAL_MIU_Read2Byte( MIU_RQ_F + u16idx*2 );
        MIU_RQ_DATA[0][2][u16idx] = HAL_MIU_Read2Byte( MIU_RQ_G + u16idx*2 );
        MIU_RQ_DATA[0][3][u16idx] = HAL_MIU_Read2Byte( MIU_RQ_H + u16idx*2 );
    }

    //miu select
    for( u16idx = 0; u16idx < 6; u16idx++ )
    {
        MIU_SELECT[u16idx] = HAL_MIU_Read2Byte(REG_MIU_SEL0+u16idx*2);
    }
    for( u16idx = 0; u16idx < 2; u16idx++ )
    {
        MIU_SELECT[u16idx+6] = HAL_MIU_Read2Byte( REG_MIU_SEL6+u16idx*2 );
    }

    //group priority
        u16grouppriority = HAL_MIU_Read2Byte( MIU_ARBB_REG_BASE+REG_MIU_GROUP_PRIORITY );

    //BW ctrl
    for( u16idx = 0; u16idx < 0x0E; u16idx++ )
    {
        MIU_BW_CRTL[u16idx] = HAL_MIU_Read2Byte( MIU_BW_CTRL+u16idx*2 );
    }

    return TRUE;
}

MS_BOOL HAL_MIU_Restore(void)
{
    MS_U16 u16idx;
    MS_U16 u16idx1;

    //After Protect ID & Addr, enable miu protect
    HAL_MIU_Write2Byte(MIU_PROTECT_EN, 0x0);

    for( u16idx = 0; u16idx < 0x80; u16idx++ )
    {
        HAL_MIU_Write2Byte(MIU_RQ_E + u16idx*2, MIU_RQ_DATA[0][0][u16idx]);
        HAL_MIU_Write2Byte(MIU_RQ_F + u16idx*2, MIU_RQ_DATA[0][1][u16idx]);
        HAL_MIU_Write2Byte(MIU_RQ_G + u16idx*2, MIU_RQ_DATA[0][2][u16idx]);
        HAL_MIU_Write2Byte(MIU_RQ_H + u16idx*2, MIU_RQ_DATA[0][3][u16idx]);
    }

    for( u16idx = 0; u16idx < 6; u16idx++ )
    {
        HAL_MIU_Write2Byte(REG_MIU_SEL0+u16idx*2, MIU_SELECT[u16idx]);
    }
    for( u16idx = 0; u16idx < 2; u16idx++ )
    {
        HAL_MIU_Write2Byte(REG_MIU_SEL6+u16idx*2, MIU_SELECT[u16idx+6]);
    }

    HAL_MIU_Write2Byte(MIU_ARBB_REG_BASE+REG_MIU_GROUP_PRIORITY, u16grouppriority);

    for( u16idx = 0; u16idx < 0x0E; u16idx++ )
    {
        HAL_MIU_Write2Byte(MIU_BW_CTRL+u16idx*2, MIU_BW_CRTL[u16idx]);
    }

    for( u16idx = 0; u16idx < ( MIU_MAX_PROTECT_BLOCK); u16idx++)
    {
        u16idx1 = u16idx * 4;

        HAL_MIU_Write2Byte(MIU_PROTECT0_ID_ENABLE+u16idx*2, MIU_PROTECT_ID_ENABLE_T[0][u16idx]);

        HAL_MIU_Write2Byte(MIU_PROTECT0_START+u16idx1*2, MIU_PROTECT_START_T[0][u16idx][0]);
        HAL_MIU_Write2Byte(MIU_PROTECT0_START+(u16idx1+1)*2, MIU_PROTECT_START_T[0][u16idx][1]);

        HAL_MIU_Write2Byte(MIU_PROTECT0_START+(u16idx1+2)*2, MIU_PROTECT_END_T[0][u16idx][0]);
        HAL_MIU_Write2Byte(MIU_PROTECT0_START+(u16idx1+3)*2, MIU_PROTECT_END_T[0][u16idx][1]);
    }

    for(u16idx = 0; u16idx < MIU_MAX_PROTECT_ID_NUM; u16idx++){
        HAL_MIU_WriteByte(MIU_PROTECT0_ID0 + u16idx,MIU_GROUPCLIENT[0][0][u16idx]);
        HAL_MIU_WriteByte(MIU_PROTECT0_GROUP1_ID0 + u16idx,MIU_GROUPCLIENT[0][1][u16idx]);
    }

    HAL_MIU_WriteByte(MIU_PROTECT_GROUP_SEL, MIU_GROUP_CLIENT_SELECT[0]);

    // Enable MIU protect
    HAL_MIU_Write2Byte(MIU_PROTECT_EN,MIU_PROTECT_EN_T[0]);

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
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: HAL_MIU_Kernel_ParseOccupiedResource
/// @brief \b Function  \b Description: Parse occupied resource to software structure
/// @return             \b 0: Fail 1: OK
////////////////////////////////////////////////////////////////////////////////
MS_BOOL HAL_MIU_Kernel_ParseOccupiedResource(void)
{

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
