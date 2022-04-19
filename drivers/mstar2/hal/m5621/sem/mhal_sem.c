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


#include <linux/kernel.h>
#include "mhal_reg_sem.h"
#include "mhal_sem.h"

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define REG16(addr) *((volatile u16*)((REGISTER_BASE_ADDRESS + ((REG_SEM_BASE * 0x100) << 1) + ((addr)<< 2))))
#define SEM_ID_GP0  \
    E_SEM_USER0,    \
    E_SEM_GE0,      \
    E_SEM_GE1,      \
    E_SEM_BDMA,     \
    E_SEM_PM,       \
    E_SEM_TEE,      \
    E_SEM_AESDMA0,  \
    E_SEM_AESDMA1
//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------
const eSemId SemIdTbl[REG_SEM_MAX_NUM] =
{
    SEM_ID_GP0
};

s16 HAL_SEM_GetSemId(eSemId eSemId)
{
    u8 idx;

    for (idx = 0; idx < REG_SEM_MAX_NUM; idx++)
        if (eSemId == SemIdTbl[idx])
            return idx;
    return (-1);
}

u32 HAL_SEM_Get_Num(void)
{
    return REG_SEM_MAX_NUM;
}

bool HAL_SEM_Get_Resource(u8 u8SemID, u16 u16ResId)
{
    if (u8SemID > REG_SEM_MAX_NUM)
        return false;
    REG16(u8SemID)= u16ResId;
    return (u16ResId == REG16(u8SemID))? true: false;
}

bool HAL_SEM_Free_Resource(u8 u8SemID, u16 u16ResId)
{
    if (u8SemID > REG_SEM_MAX_NUM)
        return false;
    if (u16ResId != REG16(u8SemID))
        return false;
    REG16(u8SemID)= 0x00;
    return true;
}

bool HAL_SEM_Reset_Resource(u8 u8SemID)
{
    if (u8SemID > REG_SEM_MAX_NUM)
        return false;
    REG16(u8SemID)= 0x00;
    return true;
}

bool HAL_SEM_Get_ResourceID(u8 u8SemID, u16* pu16ResId)
{
    if (u8SemID > REG_SEM_MAX_NUM)
        return false;
    *pu16ResId = REG16(u8SemID);
    return true;
}
