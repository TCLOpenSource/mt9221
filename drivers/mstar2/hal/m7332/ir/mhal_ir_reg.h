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
/// @file   drvIR.h
/// @brief  IR Driver Interface
/// @author MStar Semiconductor Inc.
///
/// Driver to initialize and access IR.
///     - Provide functions to initialize IR timing, and enable IR interrupt.
///     - Provide IR ISR.
///     - Provide IR callback function registration for AP.
///     - Provide function to get IR key.
///
/// \b [Example]
/// @code
///
/// // Initalize the IR in the boot time.
/// MDrv_IR_Init();
///
/// // *****************************************************************************
///
/// // Set the delay time of IR. First repeat key code is sent after one second.
/// // The following repeat key code is sent after 0.5 seconds.
/// MDrv_IR_SetDelayTime(1000, 500);
///
/// // Please refer to the following diagram. Assume that when users press and hold
/// // IR button, the repeat key is sent every 200ms.
/// // The 1st press is sent, and the return repeat code is 0.
/// // The 5th repeat key is sent because of the 1st delay time is 1000ms.
/// // The 8th repeat key is sent because of the 2nd delay time is 500ms, and
/// // the time between the 5th and the 8th repeat key is 600ms which is greater
/// // than 500ms.
/// // Note: Do not support RELEASE event.
///
/// @endcode
///
/// @image html IR_delay.JPG "IR delay time"
///
/// @code
/// // *****************************************************************************
///
/// // Set the callback function. The function MApi_IR_SetCallback is called if
/// // the IR interrupt is generated and the delay time setting is matched.
/// void MApi_IR_SetCallback(U8 *pu8Key, U8 *pu8Flg);
///
/// MDrv_IR_Set_Callback(MApi_IR_SetCallback);
///
/// // *****************************************************************************
///
/// // Polling & get the IR key directly. Users can call the MDrv_IR_GetKey to get
/// // the IR key if it returns TRUE.
/// U8 u8Key, u8Flg;
///
/// MDrv_IR_GetKey(&u8Key, &u8Flg);
///
/// @endcode
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MHAL_IR_REG_H_
#define _MHAL_IR_REG_H_


//Define IR  IRQ Vector
#define INT_NUM_IR_ALL (E_FIQEXPL_IR_IN) 

// Define IRQ registers
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_IRQ_BASE            0xFD003200
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_IRQ_BASE           (mstar_pm_base + 0x00003200UL)
#endif

#define REG_IRQ_MASK_IR         (REG_IRQ_BASE + 0x14UL)
    #define IRQ_UNMASK_IR       0xF7FFUL

// Define IrDa registers
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_IR_BASE             0xFD007B00
#elif defined(CONFIG_ARM64)
#define REG_IR_BASE            (mstar_pm_base + 0x00007B00UL)
#endif
///////////////////////////////////////////////////////////////////////////////
#define REG_IR_CTRL             (0x0000UL*4 + (REG_IR_BASE))
    #define IR_SEPR_EN              0x0200UL
    #define IR_TIMEOUT_CHK_EN       0x0100UL
    #define IR_INV                  0x80UL
    #define IR_INT_MASK             0x40UL
    #define IR_RPCODE_EN            0x20UL
    #define IR_LG01H_CHK_EN         0x10UL
    #define IR_DCODE_PCHK_EN        0x08UL
    #define IR_CCODE_CHK_EN         0x04UL
    #define IR_LDCCHK_EN            0x02UL
    #define IR_EN                   0x01UL
#define REG_IR_HDC_UPB          (0x0001UL*4 + (REG_IR_BASE))
#define REG_IR_HDC_LOB          (0x0002UL*4 + (REG_IR_BASE))
#define REG_IR_OFC_UPB          (0x0003UL*4 + (REG_IR_BASE))
#define REG_IR_OFC_LOB          (0x0004UL*4 + (REG_IR_BASE))
#define REG_IR_OFC_RP_UPB       (0x0005UL*4 + (REG_IR_BASE))
#define REG_IR_OFC_RP_LOB       (0x0006UL*4 + (REG_IR_BASE))
#define REG_IR_LG01H_UPB        (0x0007UL*4 + (REG_IR_BASE))
#define REG_IR_LG01H_LOB        (0x0008UL*4 + (REG_IR_BASE))
#define REG_IR_LG0_UPB          (0x0009UL*4 + (REG_IR_BASE))
#define REG_IR_LG0_LOB          (0x000AUL*4 + (REG_IR_BASE))
#define REG_IR_LG1_UPB          (0x000BUL*4 + (REG_IR_BASE))
#define REG_IR_LG1_LOB          (0x000CUL*4 + (REG_IR_BASE))
#define REG_IR_SEPR_UPB         (0x000DUL*4 + (REG_IR_BASE))
#define REG_IR_SEPR_LOB         (0x000EUL*4 + (REG_IR_BASE))
#define REG_IR_TIMEOUT_CYC_L    (0x000FUL*4 + (REG_IR_BASE))
#define REG_IR_TIMEOUT_CYC_H_CODE_BYTE  (0x0010UL*4 + (REG_IR_BASE))
    #define IR_CCB_CB               0x9F00UL//ir_ccode_byte+ir_code_bit_num
#define REG_IR_SEPR_BIT_FIFO_CTRL       (0x0011UL*4 + (REG_IR_BASE))
#define REG_IR_CCODE            (0x0012UL*4 + (REG_IR_BASE))
#define REG_IR_GLHRM_NUM        (0x0013UL*4 + (REG_IR_BASE))
#define REG_IR_CKDIV_NUM_KEY_DATA       (0x0014UL*4 + (REG_IR_BASE))
#define REG_IR_SHOT_CNT_L       (0x0015UL*4 + (REG_IR_BASE))
#define REG_IR_SHOT_CNT_H_FIFO_STATUS   (0x0016UL*4 + (REG_IR_BASE))
    #define IR_RPT_FLAG             0x0100UL
    #define IR_FIFO_EMPTY           0x0200UL
#define REG_IR_FIFO_RD_PULSE    (0x0018UL*4 + (REG_IR_BASE))
#define REG_IR_CCODE1           (0x0020UL*4 + (REG_IR_BASE))
#define REG_IR_CCODE1_CHK_EN    (0x0021UL*4 + (REG_IR_BASE))
    #define IR_CCODE1_CHK_EN    0x01UL
//for RC5 HW mode
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_IR_RC_BASE             0xFD007A00
#elif defined(CONFIG_ARM64)
#define REG_IR_RC_BASE            (mstar_pm_base + 0x00007A00UL)
#endif

#define REG_IR_RC_CTRL             (0x0000UL*4 + (REG_IR_RC_BASE))
#define IR_RC_AUTOCONFIG              (1<<5)
#define IR_RC_FIFO_CLEAR              (1<<6)
#define IR_RC_FIFO_WFIRST             (1<<7)
#define IR_RC_EN                      (1<<8)
#define IR_RC6_EN                     (1<<9)
#define IR_RC5EXT_EN                  (1<<10)
#define IR_RC_WKUP_EN                 (1<<11)
#define IR_RCIN_INV                   (1<<15)
#define REG_IR_RC_LONGPULSE_THR     (0x0001UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_LONGPULSE_MAR     (0x0002UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_CLK_INT_THR       (0x0003UL*4 + (REG_IR_RC_BASE))
#define IR_RC6_ECO_EN                  (1<<0)
#define REG_IR_RC_WD_TIMEOUT_CNT    (0x0004UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_COMP_KEY1_KEY2    (0x0005UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_KEY_COMMAND_ADD   (0x0006UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_KEY_MISC          (0x0007UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_KEY_FIFO_STATUS   (0x0008UL*4 + (REG_IR_RC_BASE))
#define IR_RC_FIFO_EMPTY			(1<<0)
#define IR_RC_TIMEOUT_FLAG 			(1<<1)
#define IR_RC_FIFO_FULL             (1<<2)
#define REG_IR_RC_FIFO_RD_PULSE     (0x0009UL*4 + (REG_IR_RC_BASE))
#define REG_IR_RC_CMP_RCKEY         (0x000AUL*4 + (REG_IR_RC_BASE))
#define IR_RC_POWER_WAKEUP_EN      (1<<8)

#endif
