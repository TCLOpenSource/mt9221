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

#ifndef _MHAL_SC_LD_H_
#define _MHAL_SC_LD_H_

#include "mdrv_types.h"

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
// h44[7] reg_dummy10
BOOL MHal_LD_Get_SW_WriteLDBEn(void);

// h44[12] reg_dummy10
BOOL MHal_LD_Get_SW_AlgorithmEn(void);

// h44[13] reg_dummy10
BOOL MHal_LD_Get_SW_ReadLDFEn(void);

// h76[11:8] reg_dummy0
U8 MHal_LD_Get_SW_BacklightInGamma(void);

// h76[15:12] reg_dummy0
U8 MHal_LD_Get_SW_BacklightOutGamma(void);

//***************************MDrv_LDAlgo_PostProcess****************************************//
// h41[7:0] reg_bl_width_led
U8 MHal_LD_Get_LED_BacklightWidth(void);

// h41[15:8] reg_bl_height_led
U8 MHal_LD_Get_LED_BacklightHeight(void);

// h22[11:0] reg_dummy0
U16  MHal_LD_Get_SW_PulseId(void);

// h22[15:12] reg_dummy0
U8 MHal_LD_Get_SW_PulseMode(void);

// h23[7:0] reg_dummy1
U8 MHal_LD_Get_SW_PulseLEDIntensity(void);

// h23[7:0] reg_dummy1
U8 MHal_LD_Get_SW_PulseLDBIntensity(void);

// h72[3:1] reg_lsf_out_mod
void MHal_LD_Set_LSFOutMode(U8 u8Mode);

U8 MHal_LD_Get_LSFOutMode(void);

//***************************MDrv_LDAlgo_LocalDimmingStrength****************************************//
// h25[7:0] reg_dummy3
U8 MHal_LD_Get_SW_LocalDimmingStrength(void);

// h76[7:0] reg_dummy0
U8 MHal_LD_Get_SW_MinClampValue(void);


//***************************MDrv_LDAlgo_GlobalDimmingStrength****************************************//
// h25[15:8] reg_dummy3
U8 MHal_LD_Get_SW_GlobalDimmingStrength(void);


//***************************MDrv_LDAlgo_ScalingDown****************************************//
// h03[4:0] reg_bl_width
U8 MHal_LD_Get_LDF_BacklightWidth(void);

// h03[9:5] reg_bl_height
U8 MHal_LD_Get_LDF_BacklightHeight(void);

// h1f[3:0] reg_alpha
U8 MHal_LD_Get_LDF_DCMaxAlpha(void);

// 
U8 MHal_LD_Get_SW_MaxThreshold(void);

// h1f[8] reg_write_dc_max_en
BOOL MHal_LD_Get_LDF_WriteDCMaxEn(void);

// h1f[9] reg_write_dc_max_of_en
BOOL MHal_LD_Get_LDF_WriteDCMaxOFEn(void);


//***************************MDrv_LDAlgo_TemporalFilter****************************************//
// h24[15:8] reg_dummy2
U8 MHal_LD_Get_SW_TemporalFilterStrengthDn(void);

// h43[15:8] reg_dummy
U8 MHal_LD_Get_SW_TemporalFilterStrengthUp(void);

// h57[7:0] reg_dummy5
U8 MHal_LD_Get_SW_TemporalFilterLowTh(void);

// h57[15:8] reg_dummy5
U8 MHal_LD_Get_SW_TemporalFilterHighTh(void);


//***************************MDrv_LDAlgo_SpatialFilter****************************************//
// h24[7:0] reg_dummy2
U8 MHal_LD_Get_SW_SpatialFilterStrength(void);

// h58[7:0] reg_dummy6
U8 MHal_LD_Get_SW_SpatialFilterStrength2(void);

// h58[15:8] reg_dummy6
U8 MHal_LD_Get_SW_SpatialFilterStrength3(void);

// 
U8 MHal_LD_Get_SW_SpatialFilterStrength4(void);

// 
U8 MHal_LD_Get_SW_SpatialFilterStrength5(void);

// h44[15:14] reg_dummy10
U8 MHal_LD_Get_LEDType(void);


//***************************MDrv_LD_Load_LDF_FrameStatistics****************************************//
// h04[25:0] reg_baseaddr0_l
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_L0(void);

// h06[25:0] reg_baseaddr1_l
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_L1(void);

// h08[25:0] reg_baseaddr0_r
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_R0(void);

// h0a[25:0] reg_baseaddr1_r
U32 MHal_LD_Get_LDF_FrameBufBaseAddr_R1(void);

// h40[4] reg_read_addr_idx (readonly)
BOOL MHal_LD_Get_CurFrameIdx(void);


//***************************MDrv_LD_Output_LDB_FrameBacklight****************************************//
// h4D[25:0] reg_baseaddr0_l
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_L0(void);

// h4F[25:0] reg_baseaddr1_l
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_L1(void);

// h51[25:0] reg_baseaddr0_r
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_R0(void);

// h53[25:0] reg_baseaddr1_r
U32 MHal_LD_Get_LDB_FrameBufBaseAddr_R1(void);

// h63[4:0] reg_bl_width_led
U8 MHal_LD_Get_LDB_BacklightWidth(void);

// h37[1] reg_edge2d_en
BOOL MHal_LD_Get_Edge2DEn(void);

#endif // _MHAL_SC_LD_H_

