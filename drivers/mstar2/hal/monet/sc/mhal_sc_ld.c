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

#if defined (CONFIG_HAS_LD)  
//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
/*
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/version.h>
#include "reg_sc.h"
*/
#include <linux/types.h>
#include <mstar/mstar_chip.h>
#include "mhal_sc_ld.h"
#include "reg_sc_ld.h"


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define BYTE            U8
#define WORD            U16
#define DWORD           U32


//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static void MHal_LDWriteByte(DWORD u32Reg, BYTE u8Val )
{
    if(u32Reg%2)
    {
        REG_WH((u32Reg-1), u8Val);
    }
    else
    {
        REG_WL(u32Reg, u8Val);
    }
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static BYTE MHal_LDReadByte(DWORD u32Reg )
{
    if(u32Reg%2)
    {
        u32Reg = u32Reg-1 ;
        return ((REG_RR(u32Reg) & 0xFF00)>>8);
    }
    else
    {
        return (REG_RR(u32Reg) & 0xFF);
    }
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static void MHal_LDWriteByteMask(DWORD u32Reg, BYTE u8Val, BYTE u8Mask )
{
	BYTE u8Data = MHal_LDReadByte(u32Reg);
    if(u32Reg%2)
    {
        REG_WH((u32Reg-1), (u8Data&(~u8Mask))|(u8Val&u8Mask));
    }
    else
    {
        REG_WL(u32Reg, (u8Data&(~u8Mask))|(u8Val&u8Mask));
    }
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static BYTE MHal_LDReadByteMask(DWORD u32Reg, BYTE u8Mask )
{
    if(u32Reg%2)
    {
        u32Reg = u32Reg-1 ;
        return (((REG_RR(u32Reg) & 0xFF00)>>8) & u8Mask );
    }
    else
    {
        return ((REG_RR(u32Reg) & 0xFF) & u8Mask );
    }
}


//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static void MHal_LDWrite2Byte(DWORD u32Reg, WORD u16Val )
{
    REG_W2B(u32Reg, u16Val);
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static WORD MHal_LDRead2Byte(DWORD u32Reg )
{
    return REG_RR(u32Reg) ;
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static void MHal_LDWrite2ByteMask(DWORD u32Reg, WORD u16Val, WORD u16Mask )
{
    WORD u16Data=0 ;
    u16Data = REG_RR(u32Reg);
    u16Data = (u16Data & (0xFFFF-u16Mask))|(u16Val &u16Mask);
    REG_W2B(u32Reg, u16Data);
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static WORD MHal_LDRead2ByteMask(DWORD u32Reg, WORD u16Mask )
{
    return (REG_RR(u32Reg) & u16Mask);
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static void MHal_LDWriteRegBit(DWORD u32Reg, BOOL bEnable, BYTE u8Mask )
{
    BYTE u8Data=0 ;
    u8Data = MHal_LDReadByte(u32Reg);
    u8Data = (bEnable)?(u8Data | (u8Mask))
					   :(u8Data & ~(u8Mask));
    MHal_LDWriteByte(u32Reg, u8Data);
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static BOOL MHal_LDReadRegBit(DWORD u32Reg, BYTE u8Mask )
{
    return ((MHal_LDReadByte(u32Reg) & u8Mask)==u8Mask);
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static void MHal_LDWrite4Byte(DWORD u32Reg, DWORD u32Val )
{	
    REG_W2B(u32Reg, u32Val&0xFFFF);
	REG_W2B((u32Reg+2), (u32Val&0xFFFF0000)>>16);
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
static DWORD MHal_LDRead4Byte(DWORD u32Reg )
{
    return (REG_RR((u32Reg+2))<<16)+REG_RR(u32Reg) ;
}


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

// h44[7] reg_dummy10
BOOL MHal_LD_Get_SW_WriteLDBEn(void)
{
    return MHal_LDReadRegBit(REG_2E88, _BIT7);
}

// h44[12] reg_dummy10
BOOL MHal_LD_Get_SW_AlgorithmEn(void)
{
    return MHal_LDReadRegBit(REG_2E89, _BIT4);
}

// h44[13] reg_dummy10
BOOL MHal_LD_Get_SW_ReadLDFEn(void)
{
    return MHal_LDReadRegBit(REG_2E89, _BIT5);
}

// h76[11:8] reg_dummy0
U8 MHal_LD_Get_SW_BacklightInGamma(void)
{
    return MHal_LDReadByte(REG_2EED) & 0x0F;
}

// h76[15:12] reg_dummy0
U8 MHal_LD_Get_SW_BacklightOutGamma(void)
{
    return (MHal_LDReadByte(REG_2EED) >> 4) & 0x0F;
}

//***************************MHal_LDAlgo_PostProcess****************************************//
// h41[7:0] reg_bl_width_led
U8 MHal_LD_Get_LED_BacklightWidth(void)
{
    return MHal_LDReadByte(REG_2E82)+1;
}

// h41[15:8] reg_bl_height_led
U8 MHal_LD_Get_LED_BacklightHeight(void)
{
    return MHal_LDReadByte(REG_2E83)+1;
}

// h22[11:0] reg_dummy0
U16  MHal_LD_Get_SW_PulseId(void)
{
    return MHal_LDRead2Byte(REG_2E44) & 0x0FFF;
}

// h22[15:12] reg_dummy0
U8 MHal_LD_Get_SW_PulseMode(void)
{
    return MHal_LDReadByte(REG_2E45) >> 4;
}

// h23[7:0] reg_dummy1
U8 MHal_LD_Get_SW_PulseLEDIntensity(void)
{
    return MHal_LDReadByte(REG_2E46);
}

// h23[7:0] reg_dummy1
U8 MHal_LD_Get_SW_PulseLDBIntensity(void)
{
    return MHal_LDReadByte(REG_2E47);
}

// h72[3:1] reg_lsf_out_mod
void MHal_LD_Set_LSFOutMode(U8 u8Mode)
{
    return MHal_LDWriteByteMask(REG_2EE4, u8Mode << 1, 0x0E);
}

U8 MHal_LD_Get_LSFOutMode(void)
{
    return (MHal_LDReadByte(REG_2EE4) & 0x0E) >> 1;
}

//***************************MHal_LDAlgo_LocalDimmingStrength****************************************//
// h25[7:0] reg_dummy3
U8 MHal_LD_Get_SW_LocalDimmingStrength(void)
{
    return MHal_LDReadByte(REG_2E4A);
}

// h76[7:0] reg_dummy0
U8 MHal_LD_Get_SW_MinClampValue(void)
{
    return MHal_LDReadByte(REG_2E86);
}


//***************************MHal_LDAlgo_GlobalDimmingStrength****************************************//
// h25[15:8] reg_dummy3
U8 MHal_LD_Get_SW_GlobalDimmingStrength(void)
{
    return MHal_LDReadByte(REG_2E4B);
}


//***************************MHal_LDAlgo_ScalingDown****************************************//
// h03[4:0] reg_bl_width
U8 MHal_LD_Get_LDF_BacklightWidth(void)
{
    return (MHal_LDReadByte(REG_2E06) & 0x1F)+1;
}

// h03[9:5] reg_bl_height
U8 MHal_LD_Get_LDF_BacklightHeight(void)
{
    return ((MHal_LDRead2Byte(REG_2E06) & 0x03E0) >> 5)+1;
}

// h1f[3:0] reg_alpha
U8 MHal_LD_Get_LDF_DCMaxAlpha(void)
{
    return MHal_LDReadByte(REG_2E3E) & 0x0F;
}

// 
U8 MHal_LD_Get_SW_MaxThreshold(void)
{
    return MHal_LDReadByte(REG_2E88);
}

// h1f[8] reg_write_dc_max_en
BOOL MHal_LD_Get_LDF_WriteDCMaxEn(void)
{
    return MHal_LDReadRegBit(REG_2E3F, _BIT0);
}

// h1f[9] reg_write_dc_max_of_en
BOOL MHal_LD_Get_LDF_WriteDCMaxOFEn(void)
{
    return MHal_LDReadRegBit(REG_2E3F, _BIT1);
}


//***************************MHal_LDAlgo_TemporalFilter****************************************//
// h24[15:8] reg_dummy2
U8 MHal_LD_Get_SW_TemporalFilterStrengthDn(void)
{
    return MHal_LDReadByte(REG_2E49);
}

// h43[15:8] reg_dummy
U8 MHal_LD_Get_SW_TemporalFilterStrengthUp(void)
{
    return MHal_LDReadByte(REG_2E87);
}

// h57[7:0] reg_dummy5
U8 MHal_LD_Get_SW_TemporalFilterLowTh(void)
{
    return MHal_LDReadByte(REG_2E4E);
}

// h57[15:8] reg_dummy5
U8 MHal_LD_Get_SW_TemporalFilterHighTh(void)
{
    return MHal_LDReadByte(REG_2E4F);
}


//***************************MHal_LDAlgo_SpatialFilter****************************************//
// h24[7:0] reg_dummy2
U8 MHal_LD_Get_SW_SpatialFilterStrength(void)
{
    return MHal_LDReadByte(REG_2E48);
}

// h58[7:0] reg_dummy6
U8 MHal_LD_Get_SW_SpatialFilterStrength2(void)
{
    return MHal_LDReadByte(REG_2E4C);
}

// h58[15:8] reg_dummy6
U8 MHal_LD_Get_SW_SpatialFilterStrength3(void)
{
    return MHal_LDReadByte(REG_2E4D);
}

// 
U8 MHal_LD_Get_SW_SpatialFilterStrength4(void)
{
    return MHal_LDReadByte(REG_2E84);
}

// 
U8 MHal_LD_Get_SW_SpatialFilterStrength5(void)
{
    return MHal_LDReadByte(0x2E85);
}

// h44[15:14] reg_dummy10
U8 MHal_LD_Get_LEDType(void)
{
    return (MHal_LDReadByte(REG_2E89) >> 6) & 0x3;
}


//***************************MHal_LD_Load_LDF_FrameStatistics****************************************//
// h04[25:0] reg_baseaddr0_l
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_L0(void)
{
    return (MHal_LDRead4Byte(REG_2E08) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h06[25:0] reg_baseaddr1_l
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_L1(void)
{
    return (MHal_LDRead4Byte(REG_2E0C) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h08[25:0] reg_baseaddr0_r
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_R0(void)
{
    return (MHal_LDRead4Byte(REG_2E10) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h0a[25:0] reg_baseaddr1_r
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_R1(void)
{
    return (MHal_LDRead4Byte(REG_2E14) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h40[4] reg_read_addr_idx (readonly)
BOOL MHal_LD_Get_CurFrameIdx(void)
{
    return MHal_LDReadRegBit(REG_2E80, _BIT4);
}


//***************************MHal_LD_Output_LDB_FrameBacklight****************************************//
// h4D[25:0] reg_baseaddr0_l
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_L0(void)
{
    return (MHal_LDRead4Byte(REG_2E9A) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h4F[25:0] reg_baseaddr1_l
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_L1(void)
{
    return (MHal_LDRead4Byte(REG_2E9E) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h51[25:0] reg_baseaddr0_r
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_R0(void)
{
    return (MHal_LDRead4Byte(REG_2EA2) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h53[25:0] reg_baseaddr1_r
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_R1(void)
{
    return (MHal_LDRead4Byte(REG_2EA6) & 0x03FFFFFF)*MHAL_LD_MIU_BUS;
}

// h63[4:0] reg_bl_width_led
U8 MHal_LD_Get_LDB_BacklightWidth(void)
{
    return (MHal_LDReadByte(REG_2EC6) & 0x1F)+1;
}

// h37[1] reg_edge2d_en
BOOL MHal_LD_Get_Edge2DEn(void)
{
    return MHal_LDReadRegBit(REG_2E6E, _BIT1);
}

#endif

