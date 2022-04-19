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

#ifndef IR_FORMAT_H
#define IR_FORMAT_H

//-------------------------------------------------------------------------------------------
// Customer IR Specification parameter define (Please modify them by IR SPEC)
//-------------------------------------------------------------------------------------------
#define IR_MODE_SEL                 IR_TYPE_SWDECODE_MODE//IR_TYPE_FULLDECODE_MODE

// IR Header code define
#define IR_HEADER_CODE0         0x0EUL//0x80    // Custom 0
#define IR_HEADER_CODE1         0x0EUL//0x7F    // Custom 1

#define IR_HEADER_CODE3         0x07UL//0x80    // Custom 0
#define IR_HEADER_CODE4         0x07UL//0x7F    // Custom 1

// IR Timing define
#define IR_HEADER_CODE_TIME     4500//9000    // us
#define IR_OFF_CODE_TIME        4500    // us
#define IR_OFF_CODE_RP_TIME     2500    // us
#define IR_LOGI_01H_TIME        560     // us
#define IR_LOGI_0_TIME          1120    // us
#define IR_LOGI_1_TIME          2240    // us
#define IR_TIMEOUT_CYC          140000  // us

#define IR_HEADER_CODE_TIME2     2400//1120//9000    // us
#define IR_OFF_CODE_TIME2        1600//2240    // us
#define IR_OFF_CODE_RP_TIME2     2500    // us
#define IR_LOGI_01H_TIME2        560     // us
#define IR_LOGI_0_TIME2          1125    // us
#define IR_LOGI_1_TIME2          2250    // us
#define IR_TIMEOUT_CYC2          6000//140000  // us


// IR Format define
#define IRKEY_DUMY              0xFFUL
#define IRDA_KEY_MAPPING_POWER  IRKEY_POWER
#define IR_SWFIFO_MODE          ENABLE

typedef enum _IrCommandType
{
    IRKEY_NUM_0             = 0x00,
    IRKEY_NUM_1             = 0x01,
    IRKEY_NUM_2             = 0x02,
    IRKEY_NUM_3             = 0x03,
    IRKEY_NUM_4             = 0x04,
    IRKEY_NUM_5             = 0x05,
    IRKEY_NUM_6             = 0x06,
    IRKEY_NUM_7             = 0x07,
    IRKEY_NUM_8             = 0x08,
    IRKEY_NUM_9             = 0x09,

    IRKEY_POWER             = 0x0C,
    IRKEY_MENU              = 0x11,
    IRKEY_HOME              = 0x12,
    IRKEY_AV                = 0x13,
    IRKEY_MUTE              = 0x14,

    IRKEY_UP                = 0x42,
    IRKEY_DOWN              = 0x43,
    IRKEY_LEFT              = 0X44,
    IRKEY_RIGHT             = 0x45,
    IRKEY_SELECT            = 0x46,

    IRKEY_SLIDE_L1             = 0x90,
    IRKEY_SLIDE_L2             = 0x91,
    IRKEY_SLIDE_L3             = 0x92,
    IRKEY_SLIDE_L4             = 0x93,
    IRKEY_SLIDE_L5             = 0x94,
    IRKEY_SLIDE_L6             = 0x95,

    IRKEY_SLIDE_R1             = 0x96,
    IRKEY_SLIDE_R2             = 0x97,
    IRKEY_SLIDE_R3             = 0x98,
    IRKEY_SLIDE_R4             = 0x99,
    IRKEY_SLIDE_R5             = 0x9A,
    IRKEY_SLIDE_R6             = 0x9B,

    IRKEY_TV_RADIO          = IRKEY_DUMY,
    IRKEY_CHANNEL_LIST      = IRKEY_DUMY-1,
    IRKEY_CHANNEL_FAV_LIST  = IRKEY_DUMY-2,
    IRKEY_CHANNEL_RETURN    = IRKEY_DUMY-3,
    IRKEY_CHANNEL_PLUS      = IRKEY_DUMY-4,
    IRKEY_CHANNEL_MINUS     = IRKEY_DUMY-5,

    IRKEY_AUDIO             = IRKEY_DUMY-6,
    IRKEY_VOLUME_PLUS       = IRKEY_DUMY-7,
    IRKEY_VOLUME_MINUS      = IRKEY_DUMY-8,
    IRKEY_EXIT              = IRKEY_DUMY-9,

    IRKEY_PAGE_UP           = IRKEY_DUMY-10,
    IRKEY_PAGE_DOWN         = IRKEY_DUMY-11,
    IRKEY_CLOCK             = IRKEY_DUMY-12,

    IRKEY_INFO              = IRKEY_DUMY-13,
    IRKEY_RED               = IRKEY_DUMY-14,
    IRKEY_GREEN             = IRKEY_DUMY-15,
    IRKEY_YELLOW            = IRKEY_DUMY-16,
    IRKEY_BLUE              = IRKEY_DUMY-17,
    IRKEY_MTS               = IRKEY_DUMY-18,
    IRKEY_NINE_LATTICE      = IRKEY_DUMY-19,
    IRKEY_TTX               = IRKEY_DUMY-20,
    IRKEY_CC                = IRKEY_DUMY-21,
    IRKEY_INPUT_SOURCE      = IRKEY_DUMY-22,
    IRKEY_CRADRD            = IRKEY_DUMY-23,
    IRKEY_ZOOM              = IRKEY_DUMY-24,
    IRKEY_DASH              = IRKEY_DUMY-25,
    IRKEY_SLEEP             = IRKEY_DUMY-26,
    IRKEY_EPG               = IRKEY_DUMY-27,
    IRKEY_PIP               = IRKEY_DUMY-28,

  	IRKEY_MIX               = IRKEY_DUMY-29,
    IRKEY_INDEX             = IRKEY_DUMY-30,
    IRKEY_HOLD              = IRKEY_DUMY-31,
    IRKEY_PREVIOUS          = IRKEY_DUMY-32,
    IRKEY_NEXT              = IRKEY_DUMY-33,
    IRKEY_BACKWARD          = IRKEY_DUMY-34,
    IRKEY_FORWARD           = IRKEY_DUMY-35,
    IRKEY_PLAY              = IRKEY_DUMY-36,
    IRKEY_RECORD            = IRKEY_DUMY-37,
    IRKEY_STOP              = IRKEY_DUMY-38,
    IRKEY_PAUSE             = IRKEY_DUMY-39,

    IRKEY_SIZE              = IRKEY_DUMY-40,
    IRKEY_REVEAL            = IRKEY_DUMY-41,
    IRKEY_SUBCODE           = IRKEY_DUMY-42,
}IrCommandType;

typedef enum _IrCommandType_Slide
{

    IRKEY_DESIL1         = 0x01,
    IRKEY_DESIL2         = 0x02,
    IRKEY_DESIL3         = 0x03,
    IRKEY_DESIL4         = 0x04,
    IRKEY_DESIL5         = 0x05,
    IRKEY_DESIL6         = 0x06,

    IRKEY_REVERSE1   = 0x09,
    IRKEY_REVERSE2   = 0x0A,
    IRKEY_REVERSE3   = 0x0B,
    IRKEY_REVERSE4   = 0x0C,
    IRKEY_REVERSE5   = 0x0D,
    IRKEY_REVERSE6   = 0x0E,
}IrCommandType_Slide;

//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// IR system parameter define for H/W setting (Please don't modify them)
//-------------------------------------------------------------------------------------------
#define IR_CKDIV_NUM                        ((XTAL_CLOCK_FREQ)/1000000)
#define IR_CKDIV_NUM_BOOT                   XTAL_CLOCK_FREQ
#define IR_CLK_BOOT                         (XTAL_CLOCK_FREQ/1000000)
#define IR_CLK                              IR_CLK_BOOT

#define irGetMinCnt_BOOT(time, tolerance)   ((u32)(((double)time*((double)IR_CLK_BOOT)/(IR_CKDIV_NUM_BOOT+1))*((double)1-tolerance)))
#define irGetMaxCnt_BOOT(time, tolerance)   ((u32)(((double)time*((double)IR_CLK_BOOT)/(IR_CKDIV_NUM_BOOT+1))*((double)1+tolerance)))
#define irGetMinCnt(time, tolerance)        ((u32)(((double)time*((double)IR_CLK)/(IR_CKDIV_NUM))*((double)1-tolerance)))
#define irGetMaxCnt(time, tolerance)        ((u32)(((double)time*((double)IR_CLK)/(IR_CKDIV_NUM))*((double)1+tolerance)))

#define irGetCnt_BOOT(time)                 ((u32)((double)time*((double)IR_CLK_BOOT)/(IR_CKDIV_NUM_BOOT+1)))
#define irGetCnt(time)                      ((u32)((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1)))

// 12Mhz
#define IR_RP_TIMEOUT_BOOT      irGetCnt_BOOT(IR_TIMEOUT_CYC)
#define IR_HDC_UPB_BOOT         irGetMaxCnt_BOOT(IR_HEADER_CODE_TIME, 0.2)
#define IR_HDC_LOB_BOOT         irGetMinCnt_BOOT(IR_HEADER_CODE_TIME, 0.2)
#define IR_OFC_UPB_BOOT         irGetMaxCnt_BOOT(IR_OFF_CODE_TIME, 0.2)
#define IR_OFC_LOB_BOOT         irGetMinCnt_BOOT(IR_OFF_CODE_TIME, 0.2)
#define IR_OFC_RP_UPB_BOOT      irGetMaxCnt_BOOT(IR_OFF_CODE_RP_TIME, 0.2)
#define IR_OFC_RP_LOB_BOOT      irGetMinCnt_BOOT(IR_OFF_CODE_RP_TIME, 0.2)
#define IR_LG01H_UPB_BOOT       irGetMaxCnt_BOOT(IR_LOGI_01H_TIME, 0.35)
#define IR_LG01H_LOB_BOOT       irGetMinCnt_BOOT(IR_LOGI_01H_TIME, 0.3)
#define IR_LG0_UPB_BOOT         irGetMaxCnt_BOOT(IR_LOGI_0_TIME, 0.2)
#define IR_LG0_LOB_BOOT         irGetMinCnt_BOOT(IR_LOGI_0_TIME, 0.2)
#define IR_LG1_UPB_BOOT         irGetMaxCnt_BOOT(IR_LOGI_1_TIME, 0.2)
#define IR_LG1_LOB_BOOT         irGetMinCnt_BOOT(IR_LOGI_1_TIME, 0.2)

// 90Mhz
#define IR_RP_TIMEOUT           irGetCnt(IR_TIMEOUT_CYC)
#define IR_HDC_UPB              irGetMaxCnt(IR_HEADER_CODE_TIME, 0.2)
#define IR_HDC_LOB              irGetMinCnt(IR_HEADER_CODE_TIME, 0.2)
#define IR_OFC_UPB              irGetMaxCnt(IR_OFF_CODE_TIME, 0.2)
#define IR_OFC_LOB              irGetMinCnt(IR_OFF_CODE_TIME, 0.2)
#define IR_OFC_RP_UPB           irGetMaxCnt(IR_OFF_CODE_RP_TIME, 0.2)
#define IR_OFC_RP_LOB           irGetMinCnt(IR_OFF_CODE_RP_TIME, 0.2)
#define IR_LG01H_UPB            irGetMaxCnt(IR_LOGI_01H_TIME, 0.35)
#define IR_LG01H_LOB            irGetMinCnt(IR_LOGI_01H_TIME, 0.3)
#define IR_LG0_UPB              irGetMaxCnt(IR_LOGI_0_TIME, 0.2)
#define IR_LG0_LOB              irGetMinCnt(IR_LOGI_0_TIME, 0.2)
#define IR_LG1_UPB              irGetMaxCnt(IR_LOGI_1_TIME, 0.2)
#define IR_LG1_LOB              irGetMinCnt(IR_LOGI_1_TIME, 0.2)

// 12Mhz
#define IR_RP_TIMEOUT_BOOT2      irGetCnt_BOOT(IR_TIMEOUT_CYC2)
#define IR_HDC_UPB_BOOT2         irGetMaxCnt_BOOT(IR_HEADER_CODE_TIME2, 0.2)
#define IR_HDC_LOB_BOOT2         irGetMinCnt_BOOT(IR_HEADER_CODE_TIME2, 0.2)
#define IR_OFC_UPB_BOOT2         irGetMaxCnt_BOOT(IR_OFF_CODE_TIME2, 0.2)
#define IR_OFC_LOB_BOOT2         irGetMinCnt_BOOT(IR_OFF_CODE_TIME2, 0.2)
#define IR_OFC_RP_UPB_BOOT2      irGetMaxCnt_BOOT(IR_OFF_CODE_RP_TIME2, 0.2)
#define IR_OFC_RP_LOB_BOOT2      irGetMinCnt_BOOT(IR_OFF_CODE_RP_TIME2, 0.2)
#define IR_LG01H_UPB_BOOT2       irGetMaxCnt_BOOT(IR_LOGI_01H_TIME2, 0.35)
#define IR_LG01H_LOB_BOOT2       irGetMinCnt_BOOT(IR_LOGI_01H_TIME2, 0.3)
#define IR_LG0_UPB_BOOT2         irGetMaxCnt_BOOT(IR_LOGI_0_TIME2, 0.2)
#define IR_LG0_LOB_BOOT2         irGetMinCnt_BOOT(IR_LOGI_0_TIME2, 0.2)
#define IR_LG1_UPB_BOOT2         irGetMaxCnt_BOOT(IR_LOGI_1_TIME2, 0.2)
#define IR_LG1_LOB_BOOT2         irGetMinCnt_BOOT(IR_LOGI_1_TIME2, 0.2)

// 90Mhz
#define IR_RP_TIMEOUT2           irGetCnt(IR_TIMEOUT_CYC2)
#define IR_HDC_UPB2              irGetMaxCnt(IR_HEADER_CODE_TIME2, 0.3)
#define IR_HDC_LOB2              irGetMinCnt(IR_HEADER_CODE_TIME2, 0.3)
#define IR_OFC_UPB2              irGetMaxCnt(IR_OFF_CODE_TIME2, 0.2)
#define IR_OFC_LOB2              irGetMinCnt(IR_OFF_CODE_TIME2, 0.2)
#define IR_OFC_RP_UPB2           irGetMaxCnt(IR_OFF_CODE_RP_TIME2, 0.2)
#define IR_OFC_RP_LOB2           irGetMinCnt(IR_OFF_CODE_RP_TIME2, 0.2)
#define IR_LG01H_UPB2            irGetMaxCnt(IR_LOGI_01H_TIME2, 0.35)
#define IR_LG01H_LOB2            irGetMinCnt(IR_LOGI_01H_TIME2, 0.3)
#define IR_LG0_UPB2              irGetMaxCnt(IR_LOGI_0_TIME2, 0.2)
#define IR_LG0_LOB2              irGetMinCnt(IR_LOGI_0_TIME2, 0.2)
#define IR_LG1_UPB2              irGetMaxCnt(IR_LOGI_1_TIME2, 0.2)
#define IR_LG1_LOB2              irGetMinCnt(IR_LOGI_1_TIME2, 0.2)
//-------------------------------------------------------------------------------------------

#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
#define IR_EVENT_TIMEOUT 140
#endif

#endif

