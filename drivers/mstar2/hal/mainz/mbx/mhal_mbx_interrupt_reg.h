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
/// @file   mhal_mbx_interrupt_reg.h
/// @brief  MStar Mailbox Driver DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b>(OBSOLETED) <em>legacy interface is only used by MStar proprietary Mail Message communication\n
/// It's API level for backward compatible and will be remove in the next version.\n
/// Please refer @ref drvGE.h for future compatibility.</em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MHAL_MBX_INTERRUPT_REG_H
#define _MHAL_MBX_INTERRUPT_REG_H

//=============================================================================
// Includs
//=============================================================================
#include "mdrv_types.h"

extern ptrdiff_t mstar_pm_base;

//=============================================================================
// Defines & Macros
//=============================================================================
//#define REG_INT_BASE                     (0xBF203200)
#define INT_REG(address)                 (*((volatile MS_U16 *)(REG_INT_BASE + ((address)<<2) )))

#define INTNO_BASE_32                       32
#define INTNO_BASE_47                       47
#define INTNO_PM_ARM                        38
#define INTNO_ARM_PM                        42
#define INTNO_R2_ARM                        48



// interrupt index address
#define REG_INT_FIQMASK_PM                  0x06UL
#define REG_INT_FIQMASK_ARM                 0x26UL
#define REG_INT_FIQMASK_ARM_H               0x27UL
#define REG_INT_FIQMASK_ARM_EXP             0x2EUL
#define REG_INT_FIQMASK_ARM_EXP_H           0x2FUL
#define REG_INT_FIQCLEAR_PM                 0x0EUL
#define REG_INT_FIQCLEAR_ARM                0x2EUL

#define REG_INT_FIQFINALSTATUS_PM           0x0EUL
#define REG_INT_FIQFINALSTATUS_ARM          0x2EUL

    #define _BIT_PM_ARM              (1<<(INTNO_PM_ARM-INTNO_BASE_32))
    #define _BIT_ARM_PM              (1<<(INTNO_ARM_PM-INTNO_BASE_32))
	#define _BIT_R2_ARM              (1<<(INTNO_R2_ARM-INTNO_BASE_47))


#define REG_CPU_INT_BASE               (mstar_pm_base + (0x100540UL<<1))
#define CPU_INT_REG(address)             (*((volatile MS_U16 *)(REG_CPU_INT_BASE + ((address)<<2) )))
#define REG_INT_PMFIRE                   0x0000UL
    #define INT_PM_ARM                   BIT0

#define REG_INT_ARMFIRE                  0x0002UL
    #define INT_ARM_PM                   BIT0
    #define INT_ARM_Core1                BIT1
    #define INT_ARM_R2                   BIT2
#if 1//frcr2_integration###
//################
//
// FRC - FIQ
//
// Host0: to nonPM_intr
// Host1: frc_r2
//
#define REG_FRC_INT_BASE                (REG_INT_FRC_BASE + (0x500UL<<1))
#define FRC_INT_REG(address)            (*((volatile MS_U16 *)(REG_FRC_INT_BASE + ((address)<<2) )))
//Non-PM interrupt controller
#define INTNO_IRQ_BASE_48               48
#define INTNO_IRQ_FRC_ARM               54//IRQ (irq_fiq2mips)
#define REG_INT_IRQMASK_ARM_H           0x37UL
#define _BIT_FRCFIQ_ARM             (1<<(INTNO_IRQ_FRC_ARM-INTNO_IRQ_BASE_48))
//FRC interrupt controller
#define INTNO_FRCIRQ_BASE_00            0
#define INTNO_FRCIRQ_HST1TOHST0         3//FIQ (reg_hst1to0_int)
#define REG_FRCINT_FIQMASK_HST0         0x04UL
#define _BIT_FRCFIQ_HST0            (1<<(INTNO_FRCIRQ_HST1TOHST0-INTNO_FRCIRQ_BASE_00))
#define REG_FRCINT_FIQCLR_HST0          0x0CUL
#define REG_FRCINT_FIQSTS_HST0          0x0CUL
//################
//
// FRC CPU Interrupt
//
#define REG_FRC_CPUINT_BASE             (REG_INT_FRC_BASE + (0x640UL<<1))
#define FRC_CPU_INT_REG(address)         (*((volatile MS_U16 *)(REG_FRC_CPUINT_BASE + ((address)<<2) )))
#define REG_FRCINT_HKCPUFIRE            0x0000UL //hst0 to hst1
#define INT_HKCPU_FRCR2             BIT0
#define REG_FRCINT_FRCR2FIRE            0x0002UL //hst1 to hst0
#define INT_FRCR2_HKCPU             BIT0
#endif
#endif //_MHAL_MBX_INTERRUPT_REG_H

