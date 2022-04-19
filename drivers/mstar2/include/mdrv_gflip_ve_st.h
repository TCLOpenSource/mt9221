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

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// @file   mdrv_gflip_ve_st.h
// @brief  GFlip KMD Driver Interface
// @author MediaTek Inc.
//////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MDRV_GFLIP_VE_ST_H
#define _MDRV_GFLIP_VE_ST_H

//=============================================================================
// Macros
//=============================================================================
#define VE_VEC_CONFIG_LENGTH_1STVERSION     16 //DO NOT CHANGE THIS, it is the structure length of the first version
#define VE_VEC_CONFIG_VERSION               0 //Version number for compatibility with kernel and VE driver

//=============================================================================
// Type and Structure Declaration
//=============================================================================
typedef enum
{
    MS_VEC_ISR_GOP_OP,
    MS_VEC_ISR_VE,
    MS_VEC_ISR_MAXNUM,
} MS_VEC_ISR_TYPE;

typedef enum
{
    MS_VEC_CONFIG_INIT,
    MS_VEC_CONFIG_ENABLE,
} MS_VEC_CONFIG_TYPE;

//IO Ctrl struct defines:
typedef struct
{
    MS_BOOL bEnable;      //< InOut, VE capture enable state
    MS_U8   u8FrameCount; //< Out, Current captured frame number,value range: 0~3
    MS_U8   u8Result;     //< Out, Function return status
}MS_GFLIP_VEC_STATE, *PMS_GFLIP_VEC_STATE;

typedef struct
{
    MS_U16  u16Version;   //< Version number for this structure
    MS_U16  u16Length;    //< Length of this structure, unit: byte
    MS_U8   u8Result;     //< Out, Function return status
    MS_BOOL bInterlace;   //< If the VEC capture source is interlace
    MS_U8   u8MaxFrameNumber_P;     //< The max frame number for progressive capture. This setting should obey with VE driver
    MS_U8   u8MaxFrameNumber_I;     //< The max frame number for Interlace capture. This setting should obey with VE driver
    MS_VEC_ISR_TYPE     eIsrType;   //< The ISR type of VEC capture
    MS_VEC_CONFIG_TYPE eConfigType; //< The ISR type of VEC capture
}MS_GFLIP_VEC_CONFIG, *PMS_GFLIP_VEC_CONFIG;

#endif //_MDRV_GFLIP_VE_ST_H
