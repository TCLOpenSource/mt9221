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
#include <linux/slab.h>
#include "MsTypes.h"
#include "mdrv_types.h"
#include "mdrv_miu.h"
#include "regMIU.h"
#include "mhal_miu.h"
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include "mdrv_system.h"

#include <string.h>

#ifdef CHIP_FLUSH_READ
#include "chip_setup.h"
#endif
//-------------------------------------------------------------------------------------------------
//  Define
//-------------------------------------------------------------------------------------------------

#define MIU_HIT_INFOLEN 128
#define IDNUM_KERNELPROTECT (MIU_MAX_PROTECT_ID_NUM)
//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
static const MS_U32 clientTbl[MIU_MAX_DEVICE][MIU_CLIENT_MAX_NUMBER] =
{
    { MIU_CLIENT }
};

const char* clientTbl_name[MIU_MAX_DEVICE][MIU_CLIENT_MAX_NUMBER] =
{
    {MIU_CLIENT_NAME}
};

static MS_U32 clientId_KernelProtect[MIU_MAX_PROTECT_ID_NUM] =
{
    MIU_KERNEL_PROTECT_WHITE_LIST
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
#define MIU_HAL_WARN(fmt, args...)   printk(KERN_ERR "miu hal warning %s:%d" fmt,__FUNCTION__,__LINE__,## args)

//-------------------------------------------------------------------------------------------------
//  Local Variable
//-------------------------------------------------------------------------------------------------
//static MS_U32 _gMIU_MapBase = 0xBF200000;      //default set to MIPS platfrom
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
static MS_U32 _gMIU_MapBase = 0xFD200000UL;
#define _gPM_MapBase 0xFD000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
static ptrdiff_t _gMIU_MapBase;
#define _gPM_MapBase mstar_pm_base
#endif

static MS_BOOL IDEnables[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][MIU_MAX_PROTECT_ID_NUM];
static MS_U32 IDs[MIU_MAX_DEVICE][MIU_MAX_PROTECT_ID_GROUP_NUM][MIU_MAX_PROTECT_ID_NUM];
MS_U32 ProtectGroupSelect[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK] = {{0,0,0,0,0,0,0,0}};
MS_U32 ProtectAddr[MIU_MAX_DEVICE][MIU_MAX_PROTECT_BLOCK][MIU_PROTECT_START_END];   //MIU_PROTECT_START_END : 0->start, 1->end

//-------------------------------------------------------------------------------------------------
//  MIU Debug Message Level
//-------------------------------------------------------------------------------------------------
static MS_U32 mstar_debug = E_MIU_DBGLV_ERR;
static MS_U32 miu_hit_panic = 1;

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

//    u64PhyAddr = u64BusAddr - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
//3G patch
    if(u64BusAddr >= ARM_MIU_3G_BUS_BASE)
        u64PhyAddr = u64BusAddr - ARM_MIU_3G_BUS_BASE + ARM_MIU0_BASE_ADDR;
    else
        u64PhyAddr = u64BusAddr - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;

//NO NON-UMA mode
#if 0
    if( (u64BusAddr >= ARM_MIU0_BUS_BASE) && (u64BusAddr < ARM_MIU1_BUS_BASE) )	// MIU0
        u64PhyAddr = u64BusAddr - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
    else if( (u64BusAddr >= ARM_MIU1_BUS_BASE) && (u64BusAddr < ARM_MIU2_BUS_BASE))	// MIU1
        u64PhyAddr = u64BusAddr - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;
    else
        u64PhyAddr = u64BusAddr - ARM_MIU2_BUS_BASE + ARM_MIU2_BASE_ADDR;	// MIU2
#endif

    return u64PhyAddr;
}

static MS_S16 HAL_MIU_GetClientInfo(MS_U8 u8MiuDev, eMIUClientID eClientID)
{
    if (MIU_MAX_DEVICE <= u8MiuDev)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Wrong MIU device:%u\n", u8MiuDev);
        return (-1);
    }

    if(eClientID == MIU_CLIENT_DUMMY)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Not support client ID:%u!\n", eClientID);
        return (-1);
    }

    if(eClientID != MIU_CLIENT_NONE && clientTbl[u8MiuDev][eClientID] == 0x0)
        return (-1);

    return clientTbl[u8MiuDev][eClientID];
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
/// @brief \b Function  \b Name: HAL_MIU_Read4Byte
/// @brief \b Function  \b Description: read 4 Byte data
/// @param <IN>         \b u32RegAddr: register address
/// @param <OUT>        \b None :
/// @param <RET>        \b MS_U32
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

MS_BOOL HAL_MIU_PM_Write2Byte(MS_U32 u32RegAddr, MS_U16 u16Val)
{
    if (!u32RegAddr)
    {
        MIU_HAL_ERR("%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    ((volatile MS_U16*)(_gPM_MapBase))[u32RegAddr] = u16Val;
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

MS_BOOL HAL_MIU_PM_Write4Byte(MS_U32 u32RegAddr, MS_U32 u32Val)
{
    if (!u32RegAddr)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "%s reg error!\n", __FUNCTION__);
        return FALSE;
    }

    HAL_MIU_PM_Write2Byte(u32RegAddr, u32Val & 0x0000FFFF);
    HAL_MIU_PM_Write2Byte(u32RegAddr+2, u32Val >> 16);
    return TRUE;
}

MS_BOOL HAL_MIU_GetSWClientInfo(MS_U8 u8MiuDev, MS_U64 *pu64HWClientID, MS_U32 u32HWClientIDNumber)
{
    MS_U32 u32index;
    MS_U64 idx;

    if (u8MiuDev >= MIU_MAX_DEVICE)
    {
        MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "Wrong MIU device:%u!\n", u8MiuDev);
        return FALSE;
    }
    for(idx = 0; idx < u32HWClientIDNumber; idx++)
    {
        if(*(pu64HWClientID+idx)>= MIU_MAX_TBL_CLIENT)
            *(pu64HWClientID + idx) = 0x0;
        else
        {
            for(u32index = 0; u32index < MIU_CLIENT_MAX_NUMBER; u32index++)
            {
                if( clientTbl[u8MiuDev][u32index] == *(pu64HWClientID+idx) )
                {
                    *(pu64HWClientID + idx) = u32index;
                    break;
                }
            }
            if( u32index == MIU_CLIENT_MAX_NUMBER )
            {
                MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, "Wrong MIU Client ID:0x%llX!\n", *(pu64HWClientID+idx));
                return FALSE;
            }
        }
    }
    return TRUE;
}

static void HAL_MIU_SetProtectID(MS_U32 u32Reg, MS_U8 u8MiuDev, MS_U32 u32ClientID)
{
    MS_S16 sVal = HAL_MIU_GetClientInfo(u8MiuDev, (eMIUClientID)u32ClientID);

    if (0 > sVal)
        sVal = 0;

    HAL_MIU_WriteByte(u32Reg, sVal & 0xFF);
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

static MS_BOOL HAL_MIU_CheckProtectID_KernelProtect(MS_U32 *pu32ProtectId)
{
    MS_BOOL bisKernellist = TRUE;
    MS_U32  u32index;

    for(u32index = 0; u32index < MIU_MAX_PROTECT_ID_NUM; u32index++)
    {
        if(pu32ProtectId[u32index] != clientId_KernelProtect[u32index])
        {
            bisKernellist = FALSE;
            break;
        }
    }

    return bisKernellist;
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
                if(HAL_MIU_GetClientInfo(u8MiuSel,IDs[u8MiuSel][u32SelIdGroup][u32index]) == HAL_MIU_GetClientInfo(u8MiuSel,u32ID))
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
    MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, " Err: No protect ID group can use.\n");
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
            if(u32ID == 0 || HAL_MIU_GetClientInfo(u8MiuSel, (eMIUClientID)u32ID) <= 0)
                continue;
            IDs[u8MiuSel][u32SelIdGroup][u32index0] = u32ID;
            IDEnables[u8MiuSel][u8Blockx][u32index0] = 1;
        }
        for(u32index0 = 0; u32index0 < MIU_MAX_PROTECT_ID_NUM; u32index0++)
        {
            u8isfound0 = FALSE;
            for(u32index1 = 0; u32index1 < MIU_MAX_PROTECT_ID_NUM; u32index1++)
            {
                if(HAL_MIU_GetClientInfo(u8MiuSel,IDs[u8MiuSel][u32SelIdGroup][u32index1]) == HAL_MIU_GetClientInfo(u8MiuSel,u32ID))
                {
                    IDEnables[u8MiuSel][u8Blockx][u32index0] = 1;
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
            if(u32ID == 0 || HAL_MIU_GetClientInfo(u8MiuSel, (eMIUClientID)u32ID) <= 0)
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
            u32RegID = u32RegAddr;
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
    MS_U8  u8ProtectGroupSel;
    MS_U16 u16idenable;
    MS_U32 u32index;
    MS_U32 u32index2;
    MS_U32 u32RegAddr;
    MS_U32 u32RegAddrG1;
    MS_U32 u32RegAddrStart;
    MS_U32 u32RegAddrIDenable;
    MS_U32 u32RegProtectGroupSel;

    for(u8MiuSel = E_MIU_0; u8MiuSel < MIU_MAX_DEVICE; u8MiuSel++)
    {
        for(u8Blockx = E_MIU_BLOCK_0; u8Blockx < MIU_MAX_PROTECT_BLOCK; u8Blockx++)
        {
            if(u8MiuSel == E_MIU_0)
            {
                u32RegAddr = MIU_PROTECT0_ID0;
                u32RegAddrG1 = MIU_PROTECT0_GROUP1_ID0;
                u32RegAddrStart = MIU_PROTECT0_START;
                u32RegProtectGroupSel = MIU_PROTECT_GROUP_SEL;

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
                u32RegAddr = MIU1_PROTECT0_ID0;
                u32RegAddrG1 = MIU1_PROTECT0_GROUP1_ID0;
                u32RegAddrStart = MIU1_PROTECT0_START;
                u32RegProtectGroupSel = MIU1_PROTECT_GROUP_SEL;

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
                    case E_MIU_BLOCK_4:
                        u32RegAddrIDenable = MIU1_PROTECT4_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_5:
                        u32RegAddrIDenable = MIU1_PROTECT5_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_6:
                        u32RegAddrIDenable = MIU1_PROTECT6_ID_ENABLE;
                        break;
                    case E_MIU_BLOCK_7:
                        u32RegAddrIDenable = MIU1_PROTECT7_ID_ENABLE;
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

            //ID Enables
            u16idenable = HAL_MIU_Read2Byte(u32RegAddrIDenable);
            for(u32index = 0; u32index < MIU_MAX_PROTECT_ID_NUM; u32index++)
            {
                IDEnables[u8MiuSel][u8Blockx][u32index] = ((u16idenable >> u32index) & 0x1UL)? 1: 0;
            }

            //Start address for u8Blockx
            ProtectAddr[u8MiuSel][u8Blockx][0] = HAL_MIU_Read4Byte(u32RegAddrStart + u8Blockx * 0x8);
            //End address for u8Blockx
            ProtectAddr[u8MiuSel][u8Blockx][1] = HAL_MIU_Read4Byte(u32RegAddrStart + u8Blockx * 0x8 + 0x4);
        }//for(u8Blockx = E_MIU_BLOCK_0; u8Blockx < E_MIU_BLOCK_NUM; u8Blockx++)

        //IDs
        for(u32index = 0; u32index < MIU_MAX_PROTECT_ID_NUM; u32index++)
        {
            u8ClientID = HAL_MIU_ReadByte(u32RegAddr + u32index);
            for(u32index2 = 0; u32index2 < MIU_CLIENT_MAX_NUMBER; u32index2++)
            {
                if( clientTbl[u8MiuSel][u32index2] == u8ClientID )
                {
                    IDs[u8MiuSel][0][u32index] = u32index2;
                    break;
                }
            }
            u8ClientID = HAL_MIU_ReadByte(u32RegAddrG1 + u32index);
            for(u32index2 = 0; u32index2 < MIU_CLIENT_MAX_NUMBER; u32index2++)
            {
                if( clientTbl[u8MiuSel][u32index2] == u8ClientID )
                {
                    IDs[u8MiuSel][1][u32index] = u32index2;
                    break;
                }
            }
        }

        //ID Group Selects
        u8ProtectGroupSel = HAL_MIU_ReadByte(u32RegProtectGroupSel);
        for(u8Blockx = E_MIU_BLOCK_0; u8Blockx < E_MIU_BLOCK_NUM; u8Blockx++)
        {
            ProtectGroupSelect[u8MiuSel][u8Blockx] = ( (u8ProtectGroupSel >> u8Blockx) & 0x1);
        }
    }//for(u8MiuSel = E_MIU_0; u8MiuSel < E_MIU_NUM; u8MiuSel++)

    return TRUE;
}

static MS_U64 HAL_MIU_UMAOffset(MS_U32 u32MiuSel)
{
    switch (u32MiuSel)
    {
        case 0:
        {
            return 0;
        }
        case 1:
        {
            MS_U16 regVal = HAL_MIU_Read2Byte(REG_MIU_SIZE_CONFIG);
            MS_U32 miu_size_config = (regVal>>8)&0xF;

            switch(miu_size_config)
            {
                case 0x9:
                    return 256* 0x100000ULL; //256MB
                case 0xA:
                    return 512* 0x100000ULL; //512MB
                case 0xB:
                    return 1024* 0x100000ULL; //1GB
                case 0xC:
                    return 2048* 0x100000ULL; //2GB
                case 0xD:
                    return 4096* 0x100000ULL; //4GB
                default:
                    MIU_DEBUG_LOG(KERN_CRIT, E_MIU_DBGLV_CRIT, "incorrect MIU size config value\n");
            }
            return 0;
        }
        default:
        {
            MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "incorrect MIU index\n");
        }
    }

    return 0;
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
    MS_U8   u8MiuSel = E_CHIP_MIU_0;
    MS_PHY  phy64Start;
    MS_PHY  phy64End;
    MS_U32  u32SelIdGroup = 0;
    MS_BOOL bisKernellist;

    phy64Start = HAL_MIU_BA2PA(u64BusStart);
    phy64End = HAL_MIU_BA2PA(u64BusEnd);

//NO NON-UMA mode
#if 0
    // Get MIU selection and offset
    _phy_to_miu_offset(u8MiuSel, phy64EndOffset, phy64End - 1); // use "u32End - 1" to avoid selecting wrong u8MiuSel and u32EndOffset, ex: 0xE0000000 will be MIU2_END and MIU3_START
    _phy_to_miu_offset(u8MiuSel, phy64StartOffset, phy64Start)

    phy64Start = phy64StartOffset;
    phy64End = phy64EndOffset + 1;  // due to we use u32End - 1 @_phy_to_miu_offset for getting u32EndOffset

    // For UMA offset
    phy64Start += HAL_MIU_UMAOffset(u8MiuSel);
    phy64End += HAL_MIU_UMAOffset(u8MiuSel);
#endif

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
    else if(u8MiuSel == E_CHIP_MIU_1)
    {
        u32RegAddr = MIU1_PROTECT0_ID0;
        u32RegAddrG1 = MIU1_PROTECT0_GROUP1_ID0;
        u32MiuProtectEn = MIU1_PROTECT_EN;
        u32MiuProtectGroupSel = MIU1_PROTECT_GROUP_SEL;

        switch (u8Blockx)
        {
            case E_MIU_BLOCK_0:
                u32RegAddrStart = MIU1_PROTECT0_START;
                u32RegAddrIDenable = MIU1_PROTECT0_ID_ENABLE;
                break;
            case E_MIU_BLOCK_1:
                u32RegAddrStart = MIU1_PROTECT1_START;
                u32RegAddrIDenable = MIU1_PROTECT1_ID_ENABLE;
                break;
            case E_MIU_BLOCK_2:
                u32RegAddrStart = MIU1_PROTECT2_START;
                u32RegAddrIDenable = MIU1_PROTECT2_ID_ENABLE;
                break;
            case E_MIU_BLOCK_3:
                u32RegAddrStart = MIU1_PROTECT3_START;
                u32RegAddrIDenable = MIU1_PROTECT3_ID_ENABLE;
                break;
            case E_MIU_BLOCK_4:
                u32RegAddrStart = MIU1_PROTECT4_START;
                u32RegAddrIDenable = MIU1_PROTECT4_ID_ENABLE;
                break;
            case E_MIU_BLOCK_5:
                u32RegAddrStart = MIU1_PROTECT5_START;
                u32RegAddrIDenable = MIU1_PROTECT5_ID_ENABLE;
                break;
            case E_MIU_BLOCK_6:
                u32RegAddrStart = MIU1_PROTECT6_START;
                u32RegAddrIDenable = MIU1_PROTECT6_ID_ENABLE;
                break;
            case E_MIU_BLOCK_7:
                u32RegAddrStart = MIU1_PROTECT7_START;
                u32RegAddrIDenable = MIU1_PROTECT7_ID_ENABLE;
                break;
            default:
                return false;
        }
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
                //close warning log
                //MIU_DEBUG_LOG(KERN_WARNING, E_MIU_DBGLV_WARNING, " MIU PROTECT WARNING: BLOCK%d IS COVER BY KERNEL PROTECT!!!\n", (int)u8Blockx);
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
    MS_U64 u64Address = 0;
    MS_U32 u32Reg;
    MS_U32 u32RegTZPC;
    MS_U32 u32SercureRangeNSIndex;
    MS_U32 u32SercureRangeLogErrorCode;
    MS_U64 u64SWClientID;
    MS_U64 pu64HWClientID[1];

    if(u8MiuDev == E_MIU_0)
    {
        u32Reg = MIU_PROTECT_BASE;
        u32RegTZPC = TZPC_MIU_BASE;
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
    u32SercureRangeNSIndex = (HAL_MIU_Read2Byte(u32RegTZPC+REG_SECURERANGE_HIT_SECURE_NS_INDEX) >> 7);
    u32SercureRangeLogErrorCode = (HAL_MIU_Read2Byte(u32RegTZPC+REG_SECURERANGE_LOG_ERROR_CODE) & 0xF);

    pInfo->bHit = false;

    if (REG_MIU_PROTECT_HIT_FALG & ret)
    {
        char buf[MIU_HIT_INFOLEN] = {};
        pInfo->bHit = TRUE;

        pInfo->u8Block = (MS_U8)(hitblock);
        pInfo->u8Group = (MS_U8)(GET_HIT_CLIENT(ret) >> 4);
        pInfo->u8ClientID = (MS_U8)(GET_HIT_CLIENT(ret) & 0x0F);
        u64Address = (MS_U64)((hiaddr << 16) | loaddr) ;
        u64Address = u64Address * MIU_PROTECT_ADDRESS_UNIT;

        pu64HWClientID[0] = (MS_U64)GET_HIT_CLIENT(ret);
        if( HAL_MIU_GetSWClientInfo(0, pu64HWClientID, 1) == FALSE)
            u64SWClientID = 0x0;
        else
            u64SWClientID = pu64HWClientID[0];
        if ( pInfo->u8Block == 0xF )
        {
            snprintf(buf, MIU_HIT_INFOLEN, "MIU%u Read Overbound ClientID:0x%X IP_Name:%s Hitted_Address:0x%lX<->0x%lX\n", u8MiuDev,
            (unsigned int)GET_HIT_CLIENT(ret), clientTbl_name[u8MiuDev][u64SWClientID], u64Address, u64Address + MIU_PROTECT_ADDRESS_UNIT - 1);
        }
        else if ( pInfo->u8Block == 0xE )
        {
            snprintf(buf, MIU_HIT_INFOLEN, "MIU%u Write Overbound ClientID:0x%X IP_Name:%s Hitted_Address:0x%lX<->0x%lX\n", u8MiuDev,
            (unsigned int)GET_HIT_CLIENT(ret), clientTbl_name[u8MiuDev][u64SWClientID], u64Address, u64Address + MIU_PROTECT_ADDRESS_UNIT - 1);
        }
        else
        {
            snprintf(buf, MIU_HIT_INFOLEN, "MIU%u Block:%u ClientID:0x%X IP_Name:%s Hitted_Address:0x%lX<->0x%lX\n", u8MiuDev,
            pInfo->u8Block, (unsigned int)GET_HIT_CLIENT(ret), clientTbl_name[u8MiuDev][u64SWClientID], u64Address, u64Address + MIU_PROTECT_ADDRESS_UNIT - 1);
        }

        if(printk_ratelimit() == TRUE)
            MIU_DEBUG_LOG(KERN_EMERG, E_MIU_DBGLV_EMERG, "%s", buf);

        if (miu_hit_panic==1)
            panic("MTK panic, %s", buf);

        //clear log
        HAL_MIU_Write2BytesBit(u32Reg+REG_MIU_PROTECT_STATUS, TRUE, REG_MIU_PROTECT_LOG_CLR);
        HAL_MIU_Write2BytesBit(u32Reg+REG_MIU_PROTECT_STATUS, FALSE, REG_MIU_PROTECT_LOG_CLR);
    }

    return TRUE;
}

MS_U8 MIU_DRAMSIZE[MIU_MAX_DEVICE];
MS_U8 MIU_DRAMSIZE_RESERVED_3F;

MIU_Protect_Info *MIU_PROTECT_INFO;

MS_BOOL HAL_MIU_Save(void)
{
    MS_U16 u16idx;
    MS_U16 u16idx1;

   if(MIU_PROTECT_INFO == NULL)
        MIU_PROTECT_INFO = (MIU_Protect_Info*)kmalloc( 1 * sizeof(MIU_Protect_Info), GFP_KERNEL);

    HAL_MIU_PM_Write4Byte(REG_PM_PROTECT_INFO_ADDR, virt_to_phys(MIU_PROTECT_INFO) );

    MIU_PROTECT_INFO[0].u16CheckMagicNumber = 0x1234;

    // Enable MIU protect
    MIU_PROTECT_INFO[0].u16ProtectAddressEnable[0] = HAL_MIU_Read2Byte(MIU_PROTECT_EN);
    MIU_PROTECT_INFO[0].u16ProtectAddressEnable[1] = HAL_MIU_Read2Byte(MIU_PROTECT_EN + 0x2);

    // Protect ID Group Select
    MIU_PROTECT_INFO[0].u16ProtectIDGroupSelect = HAL_MIU_Read2Byte(MIU_PROTECT_GROUP_SEL);

    for(u16idx = 0; u16idx < MIU_MAX_PROTECT_BLOCK ; u16idx++)
    {
        u16idx1 = u16idx * 4;

        // Protect ID Enable
        MIU_PROTECT_INFO[0].u16ProtectIDEnable[u16idx] = HAL_MIU_Read2Byte(MIU_PROTECT0_ID_ENABLE+u16idx*2);

        // MIU0 Protect Address Start
        MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 0] = HAL_MIU_Read2Byte(MIU_PROTECT0_START + u16idx1 * 2 );
        MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 1] = HAL_MIU_Read2Byte(MIU_PROTECT0_START + (u16idx1+1)*2 );
        // MIU0 Protect Address End
        MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 2] = HAL_MIU_Read2Byte(MIU_PROTECT0_START + (u16idx1+2)*2 );
        MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 3] = HAL_MIU_Read2Byte(MIU_PROTECT0_START + (u16idx1+3)*2 );
    }

    for(u16idx = 0; u16idx < MIU_MAX_PROTECT_ID_NUM/2; u16idx++)
    {
        // Protect ID
        MIU_PROTECT_INFO[0].u16ProtectIDG1[u16idx] = HAL_MIU_Read2Byte(MIU_PROTECT0_GROUP1_ID0+ u16idx * 2);
        MIU_PROTECT_INFO[0].u16ProtectIDG0[u16idx] = HAL_MIU_Read2Byte(MIU_PROTECT0_ID0 + u16idx * 2);
    }

    for(u16idx = 0; u16idx < 4; u16idx++)
    {
        // Protect Dram Size
        MIU_PROTECT_INFO[0].u16DramSize[u16idx] = HAL_MIU_Read2Byte( MIU_PROTECT_DRAM_SIZE + u16idx * 2);
    }

#ifdef CHIP_FLUSH_READ
    Chip_Flush_Cache_Range((unsigned long)(MIU_PROTECT_INFO), (unsigned long)sizeof(MIU_PROTECT_INFO));
#endif

    //dram size
    MIU_DRAMSIZE[0] = HAL_MIU_ReadByte(MIU_PROTECT_DDR_SIZE);

    return TRUE;
}

MS_BOOL HAL_MIU_Restore(void)
{
    MS_U16 u16idx;
    MS_U16 u16idx1;

    //dram size
    HAL_MIU_WriteByte(MIU_PROTECT_DDR_SIZE, MIU_DRAMSIZE[0]);

    // Disable MIU protect
    HAL_MIU_Write2Byte(MIU_PROTECT_EN, 0x0);

    for(u16idx = 0; u16idx < 4; u16idx++)
    {
        // Protect Dram Size
        HAL_MIU_Write2Byte( MIU_PROTECT_DRAM_SIZE + u16idx * 2, MIU_PROTECT_INFO[0].u16DramSize[u16idx]);
    }

    for(u16idx = 0; u16idx < MIU_MAX_PROTECT_ID_NUM/2; u16idx++)
    {
        // Protect ID
        HAL_MIU_Write2Byte(MIU_PROTECT0_ID0 + u16idx * 2, MIU_PROTECT_INFO[0].u16ProtectIDG0[u16idx]);
        HAL_MIU_Write2Byte(MIU_PROTECT0_GROUP1_ID0 + u16idx * 2, MIU_PROTECT_INFO[0].u16ProtectIDG1[u16idx]);
    }

    // Protect ID Group Select
    HAL_MIU_Write2Byte(MIU_PROTECT_GROUP_SEL, MIU_PROTECT_INFO[0].u16ProtectIDGroupSelect);

    for( u16idx = 0; u16idx < ( MIU_MAX_PROTECT_BLOCK ); u16idx++)
    {
        u16idx1 = u16idx * 4;

        // Protect ID Enable
        HAL_MIU_Write2Byte(MIU_PROTECT0_ID_ENABLE+u16idx*2, MIU_PROTECT_INFO[0].u16ProtectIDEnable[u16idx]);

        // MIU0 Protect Address Start
        HAL_MIU_Write2Byte(MIU_PROTECT0_START + u16idx1*2, MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 0]);
        HAL_MIU_Write2Byte(MIU_PROTECT0_START + (u16idx1+1)*2, MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 1]);
        // MIU0 Protect Address End
        HAL_MIU_Write2Byte(MIU_PROTECT0_START + (u16idx1+2)*2, MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 2]);
        HAL_MIU_Write2Byte(MIU_PROTECT0_START + (u16idx1+3)*2, MIU_PROTECT_INFO[0].u16ProtectAddress[u16idx1 + 3]);

    }

    // Enable MIU protect
    HAL_MIU_Write2Byte(MIU_PROTECT_EN, MIU_PROTECT_INFO[0].u16ProtectAddressEnable[0]);

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
//// for UMA patch, use reserved register to store real dram size,
//// and read this reserved register in kernel and utopia
//            regAddr = MIU1_PROTECT_DDR_SIZE;
            regAddr = MIU1_REG_RESERVED_3F;
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
        case 0xC:
            *pDramSize = E_MIU_DDR_4096MB;
            break;
        case 0xD:
            *pDramSize = E_MIU_DDR_8192MB;
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
    MIU_DEBUG_LOG(KERN_ERR, E_MIU_DBGLV_ERR, "no need patch\n");
    return TRUE;
}

void HAL_MIU_Remove_ADCDMA_Protect(void)
{
}
