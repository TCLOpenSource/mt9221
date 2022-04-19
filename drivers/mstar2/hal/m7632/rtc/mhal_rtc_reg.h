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





#ifndef _REG_RTC_H
#define _REG_RTC_H

#include "mdrv_types.h"

//------------------------------------------------------------------------------
// PIU_MISC Reg
//------------------------------------------------------------------------------
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define RIU_MAP 0xFD000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define RIU_MAP           (mstar_pm_base)
#endif

#define INTR_CTL_BASE               (0x00101900UL)
#define REG_HST0_IRQ_STS_15_0       (INTR_CTL_BASE+0x1CUL*2)
#define REG_HST0_IRQ_MASK_15_0_     (INTR_CTL_BASE+0x14UL*2)
   #define PM_SLEEP_INT_STS			(0x0002UL)
   #define PM_SLEEP_INT_MASK		(0x0002UL)

#define REG_RTC_BASE_0              (0x1200UL)
#define REG_RTC_BASE_2              (0x1300UL)

#define REG_SLEEP_BASE          (0x0E00UL)
#define SLEEP_DUMMY_REG	        (0x0070UL)

#define REG_RTC_CTRL_REG        (0x0000UL)
    #define RTC_SOFT_RSTZ_BIT       (0x0001UL)//BIT0
    #define RTC_CNT_EN_BIT          (0x0002UL)//BIT1
    #define RTC_WRAP_EN_BIT         (0x0004UL)//BIT2
    #define RTC_LOAD_EN_BIT         (0x0008UL)//BIT3
    #define RTC_READ_EN_BIT         (0x0010UL)//BIT4
    #define RTC_INT_MASK_BIT        (0x0020UL)//BIT5
    #define RTC_INT_FORCE_BIT       (0x0040UL)//BIT6
    #define RTC_INT_CLEAR_BIT       (0x0080UL)//BIT7
    #define RTC_INT_RAW_BIT         (0x0100UL)//BIT8
    #define RTC_INT_STATUS_BIT      (0x0200UL)//BIT9
#define REG_RTC_FREQ_CW         (0x0002UL)    //BIT0-BIT31
#define REG_RTC_LOAD_VAL        (0x0006UL)    //BIT0-BIT31
#define REG_RTC_MATCH_VAL_L       (0x000EUL)    //BIT0-BIT31
#define REG_RTC_MATCH_VAL_H       (0x0012UL)    //BIT0-BIT31
#define REG_RTC_INT             (0x0000UL)
    #define RTC_RAW_INT_BIT         (0x0100UL)//BIT0
    #define RTC_INT_BIT             (0x0200UL)//BIT1
#define REG_RTC_CNT             (0x0016UL)    //BIT0-BIT31

// PM
#define PM_REG_BASE             (0x0700UL*2)
#define REG_PM_CKG_RTC          (PM_REG_BASE + 0x22UL*2+0)

#define REG_WK_IRQ_MASK             (PM_REG_BASE + 0x08*2+0)
    #define  RTC_WK_SRC	            (0x0080UL)//BIT7

#define REG_RTC_DUMMY0          (0x0020UL)
#define REG_RTC_DUMMY1          (0x0022UL)
#endif  // _REG_RTC_H

