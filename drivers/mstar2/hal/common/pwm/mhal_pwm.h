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

#ifndef _HAL_PWM_H_
#define _HAL_PWM_H_

#include "mdrv_types.h"

////////////////////////////////////////////////////////////////////////////////
/// @file HALPWM.h
/// @author MStar Semiconductor Inc.
/// @brief Pulse Width Modulation hal
////////////////////////////////////////////////////////////////////////////////

#define PWM_Num     5       /* Max. 6 */

////////////////////////////////////////////////////////////////////////////////
// Macro for utility
////////////////////////////////////////////////////////////////////////////////
#define MST_MACRO_START     do {
#define MST_MACRO_END       } while (0)

#define UNUSED( var )       ((void)(var))

////////////////////////////////////////////////////////////////////////////////
// Macro for bitwise
////////////////////////////////////////////////////////////////////////////////
#define _BITMASK(loc_msb, loc_lsb) ((1U << (loc_msb)) - (1U << (loc_lsb)) + (1U << (loc_msb)))
#define BITMASK(x) _BITMASK(1?x, 0?x)
#define BITFLAG(loc) (1U << (loc))

#define SETBIT(REG, BIT)   ((REG) |= (1UL << (BIT)))
#define CLRBIT(REG, BIT)   ((REG) &= ~(1UL << (BIT)))
#define GETBIT(REG, BIT)   (((REG) >> (BIT)) & 0x01UL)
#define COMPLEMENT(a)      (~(a))
#define BITS(_bits_, _val_)         ((BIT(((1)?_bits_)+1)-BIT(((0)?_bits_))) & (_val_<<((0)?_bits_)))
#define BMASK(_bits_)               (BIT(((1)?_bits_)+1)-BIT(((0)?_bits_)))
#define READ_BYTE(_reg)             (*(volatile U8*)(_reg))
#define READ_WORD(_reg)             (*(volatile U16*)(_reg))
#define WRITE_BYTE(_reg, _val)      { (*((volatile U8*)(_reg))) = (U8)(_val); }
#define WRITE_WORD(_reg, _val)      { (*((volatile U16*)(_reg))) = (U16)(_val); }

typedef enum _HAL_PM_PWM_ChNum
{
    E_HAL_PM_PWM_CH0,
    E_HAL_PM_PWM_CH1,
    E_HAL_PM_PWM_CH2,
}HAL_PM_PWM_ChNum;

////////////////////////////////////////////////////////////////////////////////////////
// Extern function
////////////////////////////////////////////////////////////////////////////////
BOOL HAL_PWM_Init(void);
BOOL HAL_PWM_Oen(PWM_ChNum index, BOOL letch);
BOOL HAL_PWM_UnitDiv(U16 u16DivPWM);
void HAL_PWM_Period(PWM_ChNum index, U32 u32PeriodPWM);
void HAL_PWM_DutyCycle(PWM_ChNum index, U32 u32DutyPWM);
void HAL_PWM_Div(PWM_ChNum index, U16 u16DivPWM);
void HAL_PWM_Polarity(PWM_ChNum index, BOOL bPolPWM);
void HAL_PWM_VDBen(PWM_ChNum index, BOOL bVdbenPWM);
void HAL_PWM_Vrest(PWM_ChNum index, BOOL bRstPWM);
void HAL_PWM_DBen(PWM_ChNum index, BOOL bdbenPWM);
void HAL_PWM_RstMux(PWM_ChNum index, BOOL bMuxPWM);
void HAL_PWM_RstCnt(PWM_ChNum index, U8 u8RstCntPWM);
void HAL_PWM_BypassUnit(PWM_ChNum index, BOOL bBypassPWM);
BOOL HAL_PWM01_CntMode(PWM_CntMode CntMode);
BOOL HAL_PWM23_CntMode(PWM_CntMode CntMode);
BOOL HAL_PWM67_CntMode(PWM_CntMode CntMode);
BOOL HAL_PWM_Shift(PWM_ChNum index, U32 u32ShiftPWM);
void HAL_PWM_IMPULSE_EN(PWM_ChNum index, BOOL bdbenPWM);
void HAL_PWM_ODDEVEN_SYNC(PWM_ChNum index, BOOL bdbenPWM);
void HAL_PWM_Nvsync(PWM_ChNum index, BOOL bNvsPWM);
void HAL_PWM_Align(PWM_ChNum index, BOOL bAliPWM);

void HAL_PM_PWM_Enable(HAL_PM_PWM_ChNum index, BOOL b_en);
void HAL_PM_PWM_Period(HAL_PM_PWM_ChNum index, U16 u16PeriodPWM);
void HAL_PM_PWM_DutyCycle(HAL_PM_PWM_ChNum index, U16 u16DutyPWM);
void HAL_PM_PWM_Div(HAL_PM_PWM_ChNum index, U8 u8DivPWM);
void HAL_PM_PWM_Polarity(HAL_PM_PWM_ChNum index, BOOL bPolPWM);
void HAL_PM_PWM_DBen(HAL_PM_PWM_ChNum index, BOOL bdbenPWM);
void HAL_PM_PWM_Reset(HAL_PM_PWM_ChNum index, BOOL bresetPWM);
void HAL_PWM_AutoCorrect(PWM_ChNum index, BOOL bCorrect);
BOOL HAL_PWM_GetOen(PWM_ChNum index);
U32 HAL_PWM_GetPeriod(PWM_ChNum index);
U32 HAL_PWM_GetDutyCycle(PWM_ChNum index);
BOOL HAL_PWM_GetPolarity(PWM_ChNum index);

#endif // _HAL_PWM_H_

