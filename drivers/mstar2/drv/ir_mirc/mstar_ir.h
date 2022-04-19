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

#ifndef _MSTAR_IR_H_
#define _MSTAR_IR_H_

#define XTAL_CLOCK_FREQ             12000000    //12 MHz
#define IR_CKDIV_NUM ((XTAL_CLOCK_FREQ+500000)/1000000)
#define IR_CLK   (XTAL_CLOCK_FREQ/1000000)
#define irGetCnt(time) ((u32)((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1)))

#define irGetMinCnt(time, tolerance)        ((u32)(((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1))*((double)1-tolerance)))
#define irGetMaxCnt(time, tolerance)        ((u32)(((double)time*((double)IR_CLK)/(IR_CKDIV_NUM+1))*((double)1+tolerance)))

//IR MSTAR TV Headcode
#define IR_DEFAULT_CUSTOMER_CODE0         0x7F80UL    // Custom 0
#define IR_DEFAULT_CUSTOMER_CODE1         0x0000UL    // Custom 1

// IR HW NEC Timing define
#define IR_HEADER_CODE_TIME     9000    // us
#define IR_OFF_CODE_TIME        4500    // us
#define IR_OFF_CODE_RP_TIME     2500    // us
#define IR_LOGI_01H_TIME        560     // us
#define IR_LOGI_0_TIME          1120    // us
#define IR_LOGI_1_TIME          2240    // us
#define IR_TIMEOUT_CYC          140000  // us

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

//#######################################
//#     (1) ==> Operation on 4/3 MHz
//#######################################
#define IR_RC_CLK_DIV(ClkMhz)       (ClkMhz*3/4-1)
#define IR_RC_LGPS_THR(UnitTime)    ((UnitTime))            //(UnitTime)*(4/3)*(3/4)
#define IR_RC_INTG_THR(UnitTime)    (((UnitTime)*2/3-7)/8) //((UnitTime)*(4/3)*(1/2)-7)/8
#define IR_RC_WDOG_CNT(UnitTime)    ((UnitTime)/768)        //(UnitTime)*(4/3)/1024
#define IR_RC_TMOUT_CNT(UnitTime)   ((UnitTime)/1536)       //(UnitTime)*(4/3)/2048
#define IR_RC6_LDPS_THR(UnitTime)   ((UnitTime)*8/9-31)     //(UnitTime)*(4/3)*(2/3)-31  RC6 leading pulse threshold * 32
#define IR_RC6_LGPS_MAR(UnitTime)   ((UnitTime)*2/3)        //(UnitTime)*(4/3)*(1/2)

//HW RC5 Timming Define
#define IR_RC5_LONGPULSE_THR     1778    //us
#define IR_RC5_LONGPULSE_MAR     192     //us only for RC6????
#define IR_RC5_INTG_THR_TIME     887     //us
#define IR_RC5_WDOG_TIME         24576   // us ???
#define IR_RC5_TIMEOUT_TIME      24576   // us ???
#define IR_RC5_POWER_WAKEUP_KEY  0xffUL
#define IR_RC5_BITS             14

//HW RC6 Timming Define
#define IR_RC6_LONGPULSE_THR     889    //us 2t
#define IR_RC6_LONGPULSE_MAR     444     //us only for RC6????
#define IR_RC6_INTG_THR_TIME     444     //us t
#define IR_RC6_WDOG_TIME         25752  // us ??? IR_RC6_LONGPULSE_THR*29bit=58t   25781
#define IR_RC6_TIMEOUT_TIME      25752   // us ??? 58t
#define IR_RC6_POWER_WAKEUP_KEY  0xffUL
#define IR_RC6_BITS             21
#define IR_RC6_LEADPULSE        2667  //6t  RC6 leading pulse threshold * 32


#define IRFLAG_IRENABLE         0x00000001UL
#define IRFLAG_HWINITED         0x00000002UL

#endif
