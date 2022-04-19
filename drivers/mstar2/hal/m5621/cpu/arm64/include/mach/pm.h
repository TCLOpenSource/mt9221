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

#ifndef __ASM_ARCH_MADISON_PM_H
#define __ASM_ARCH_MADISON_PM_H

#include <linux/version.h>

/*------------------------------------------------------------------------------
Constant
-------------------------------------------------------------------------------*/
#define PMU_WAKEUP_ADDR_REGL            (0x0EA4 << 1)    //0x1F001CE0
#define PMU_WAKEUP_ADDR_LSHIFT          0x00000000
#define PMU_WAKEUP_ADDR_REGH            (0x0EA4 << 1)    //0x1F001CE4
#define PMU_WAKEUP_ADDR_HSHIFT          0x00000008


#define WAKEUP_ADDR_MASK                0x0000FFF0
#define WAKEUP_FLAG_MASK                0x0000000F
#define WAKEUP_FLAG_INVALID             0
#define WAKEUP_FLAG_SLEPT               1
#define WAKEUP_FLAG_WKUP                2

/* Definition for ARM CPU */
#define Mode_USR                        0x10
#define Mode_FIQ                        0x11
#define Mode_IRQ                        0x12
#define Mode_SVC                        0x13
#define Mode_ABT                        0x17
#define Mode_UND                        0x1B
#define Mode_SYS                        0x1F
#define I_Bit                           0x80
#define F_Bit                           0x40

/* Define how many bytes in a word */
#define WORD_SIZE                       4
#define MSTAR_SLEEP_MAGIC               0x4D535452 /*MSTR*/
#define WAKEUP_SAVE_ADDR                0XFFFFFFC000000000

/* Constants used to calculate data size backup in menory */
#define SLEEPSTATE_DATA_START           0
#define SLEEPSTATE_MAGIC                (SLEEPSTATE_DATA_START)
#define SLEEPSTATE_SVC_R4               (SLEEPSTATE_MAGIC       + WORD_SIZE)
#define SLEEPSTATE_SVC_R5               (SLEEPSTATE_SVC_R4      + WORD_SIZE)
#define SLEEPSTATE_SVC_R6               (SLEEPSTATE_SVC_R5      + WORD_SIZE)
#define SLEEPSTATE_SVC_R7               (SLEEPSTATE_SVC_R6      + WORD_SIZE)
#define SLEEPSTATE_SVC_R8               (SLEEPSTATE_SVC_R7      + WORD_SIZE)
#define SLEEPSTATE_SVC_R9               (SLEEPSTATE_SVC_R8      + WORD_SIZE)
#define SLEEPSTATE_SVC_R10              (SLEEPSTATE_SVC_R9      + WORD_SIZE)
#define SLEEPSTATE_SVC_R11              (SLEEPSTATE_SVC_R10     + WORD_SIZE)
#define SLEEPSTATE_SVC_R12              (SLEEPSTATE_SVC_R11     + WORD_SIZE)
#define SLEEPSTATE_SVC_SP               (SLEEPSTATE_SVC_R12     + WORD_SIZE)
#define SLEEPSTATE_SVC_SPSR             (SLEEPSTATE_SVC_SP      + WORD_SIZE)
#define SLEEPSTATE_SVC_CPSR             (SLEEPSTATE_SVC_SPSR    + WORD_SIZE)
#define SLEEPSTATE_FIQ_SPSR             (SLEEPSTATE_SVC_CPSR    + WORD_SIZE)
#define SLEEPSTATE_FIQ_R8               (SLEEPSTATE_FIQ_SPSR    + WORD_SIZE)
#define SLEEPSTATE_FIQ_R9               (SLEEPSTATE_FIQ_R8      + WORD_SIZE)
#define SLEEPSTATE_FIQ_R10              (SLEEPSTATE_FIQ_R9      + WORD_SIZE)
#define SLEEPSTATE_FIQ_R11              (SLEEPSTATE_FIQ_R10     + WORD_SIZE)
#define SLEEPSTATE_FIQ_R12              (SLEEPSTATE_FIQ_R11     + WORD_SIZE)
#define SLEEPSTATE_FIQ_SP               (SLEEPSTATE_FIQ_R12     + WORD_SIZE)
#define SLEEPSTATE_FIQ_LR               (SLEEPSTATE_FIQ_SP      + WORD_SIZE)
#define SLEEPSTATE_ABT_SPSR             (SLEEPSTATE_FIQ_LR      + WORD_SIZE)
#define SLEEPSTATE_ABT_SP               (SLEEPSTATE_ABT_SPSR    + WORD_SIZE)
#define SLEEPSTATE_ABT_LR               (SLEEPSTATE_ABT_SP      + WORD_SIZE)
#define SLEEPSTATE_IRQ_SPSR             (SLEEPSTATE_ABT_LR      + WORD_SIZE)
#define SLEEPSTATE_IRQ_SP               (SLEEPSTATE_IRQ_SPSR    + WORD_SIZE)
#define SLEEPSTATE_IRQ_LR               (SLEEPSTATE_IRQ_SP      + WORD_SIZE)
#define SLEEPSTATE_UND_SPSR             (SLEEPSTATE_IRQ_LR      + WORD_SIZE)
#define SLEEPSTATE_UND_SP               (SLEEPSTATE_UND_SPSR    + WORD_SIZE)
#define SLEEPSTATE_UND_LR               (SLEEPSTATE_UND_SP      + WORD_SIZE)
#define SLEEPSTATE_SYS_SP               (SLEEPSTATE_UND_LR      + WORD_SIZE)
#define SLEEPSTATE_SYS_LR               (SLEEPSTATE_SYS_SP      + WORD_SIZE)
#define SLEEPSTATE_CHKSUM               (SLEEPSTATE_SYS_LR      + WORD_SIZE)
//Neon
#define NEON_DREG_NUM                   32
#define SLEEPSTATE_NEONREG              (SLEEPSTATE_CHKSUM      + WORD_SIZE)
#define SLEEPSTATE_DATA_END             (SLEEPSTATE_NEONREG     + (WORD_SIZE * 2) * NEON_DREG_NUM)
#define SLEEPDATA_SIZE                  ((SLEEPSTATE_DATA_END - SLEEPSTATE_DATA_START) / WORD_SIZE)

#define AC_ON_CHECK_REG                 (0x1F000000 + (0x0E70 << 1))

#ifndef BIT0
#define BIT0                            (0x01 << 0)
#define BIT1                            (0x01 << 1)
#define BIT2                            (0x01 << 2)
#define BIT3                            (0x01 << 3)
#define BIT4                            (0x01 << 4)
#define BIT5                            (0x01 << 5)
#define BIT6                            (0x01 << 6)
#define BIT7                            (0x01 << 7)
#define BIT8                            (0x01 << 8)
#define BIT9                            (0x01 << 9)
#define BIT10                           (0x01 << 10)
#define BIT11                           (0x01 << 11)
#define BIT12                           (0x01 << 12)
#define BIT13                           (0x01 << 13)
#define BIT14                           (0x01 << 14)
#define BIT15                           (0x01 << 15)
#define BIT16                           (0x01 << 16)
#define BIT17                           (0x01 << 17)
#define BIT18                           (0x01 << 18)
#define BIT19                           (0x01 << 19)
#define BIT20                           (0x01 << 20)
#define BIT21                           (0x01 << 21)
#define BIT22                           (0x01 << 22)
#define BIT23                           (0x01 << 23)
#define BIT24                           (0x01 << 24)
#define BIT25                           (0x01 << 25)
#define BIT26                           (0x01 << 26)
#define BIT27                           (0x01 << 27)
#define BIT28                           (0x01 << 28)
#define BIT29                           (0x01 << 29)
#define BIT30                           (0x01 << 30)
#define BIT31                           (0x01 << 31)
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,10,86)
#define STRSAVE_ITEM_SIZE 8
#define sleep_saved_magic 0
#define sleep_save (sleep_saved_magic+1)
#define sleep_save_spt (sleep_save+10)
#define sleep_save_ret_addr (sleep_save_spt+1 )
#define sleep_save_wakup_vir (sleep_save_ret_addr+1)
#define sleep_save_wakup_phy (sleep_save_wakup_vir+1)
#define sleep_save_regs (sleep_save_wakup_phy+1)
#define sleep_save2 (sleep_save_regs+14)
#define sleep_save_wakup_save (sleep_save2+1)
#define sleep_save_L2CTLR_EL1 (sleep_save_wakup_save+1)
#define sleep_save_L2ECTLR_EL1 (sleep_save_L2CTLR_EL1+1)
#define sleep_save_L2ACTLR_EL1 (sleep_save_L2ECTLR_EL1+1)
#define sleep_save_PAR_EL1 (sleep_save_L2ACTLR_EL1+1)
#define sleep_save_CPUACTLR_EL1 (sleep_save_PAR_EL1+1)
#define sleep_save_CPUECTLR_EL1 (sleep_save_CPUACTLR_EL1+1)
#define sleep_save_CPUMERRSR_EL1 (sleep_save_CPUECTLR_EL1+1)
#define sleep_save_TPIDR_EL1 (sleep_save_CPUMERRSR_EL1+1)
#define sleep_save_CNTP_CVAL_EL0 (sleep_save_TPIDR_EL1+1)
#define sleep_save_CNTP_TVAL_EL0 (sleep_save_CNTP_CVAL_EL0+1)
#define sleep_save_CNTV_CVAL_EL0 (sleep_save_CNTP_TVAL_EL0+1)
#define sleep_save_CNTV_TVAL_EL0 (sleep_save_CNTV_CVAL_EL0+1)
#define sleep_save_CNTP_CTL_EL0 (sleep_save_CNTV_TVAL_EL0+1)
#define sleep_save_CNTV_CTL_EL0 (sleep_save_CNTP_CTL_EL0+1)
#define sleep_save_CNTKCTL_EL1 (sleep_save_CNTV_CTL_EL0+1)
#define STRSAVE_AREA_COUNT (sleep_save_CNTKCTL_EL1+1)
#endif

#ifdef __ASSEMBLY__
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,10,86)
.extern strsavearea
.macro STSV_ADR, reg, id
ADR_L \reg, strsavearea+(\id*STRSAVE_ITEM_SIZE)
.endm
#else
.macro STSV_ADR, reg, id
ADR \reg, \id
.endm
#endif
#endif

#endif /* __ASM_ARCH_MADISON_PM_H */
