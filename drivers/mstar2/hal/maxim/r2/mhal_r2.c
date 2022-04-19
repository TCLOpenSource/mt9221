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

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/delay.h>
#include <linux/printk.h>

#include "mdrv_types.h"
#include "mdrv_r2.h"
#include "mhal_r2.h"
#include "mhal_r2_reg.h"

#include "mdrv_system.h"

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static U8 MHal_R2_Read_Reg8(ptrdiff_t pBase, U32 u32Reg)
{
    return *((volatile U8 *)(pBase + (u32Reg << 1)) - (u32Reg & 1));
}

static U16 MHal_R2_Read_Reg16(ptrdiff_t pBase, U32 u32Reg)
{
    return *((volatile U16 *)(pBase + (u32Reg << 1)));
}

static void MHal_R2_Write_Reg8(ptrdiff_t pBase, U32 u32Reg, U8 u8Val)
{
    *((volatile U8 *)(pBase + (u32Reg << 1) - (u32Reg & 1))) = u8Val;
}

static void MHal_R2_Write_Reg16(ptrdiff_t pBase, U32 u32Reg, U16 u16Val)
{
    *((volatile U16 *)(pBase + (u32Reg << 1))) = u16Val;
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_MSTAR_FRC_R2)
void MHal_FRC_R2_Disable(void)
{
    /* Disable R2 -> Stop(Reset). */
    MHal_R2_Write_Reg8(BASE_FRC_R2, REG_FRC_R2_STOP, 0x00UL);
    mdelay(15);
    MHAL_R2_DEBUG("Disable done.\n");
}

void MHal_FRC_R2_Enable(U32 u32Addr)
{
    /* (0) Disable R2 -> Stop(Reset). */

    /* (1) Set FRC clocks: */
    /* (1-1) R2 MCU(RIU). */
    MHal_R2_Write_Reg8(BASE_NON_PM, REG_CLKGEN1_FRC_MCU, 0x00UL);
    MHal_R2_Write_Reg8(BASE_NON_PM, REG_CLKGEN1_FRC_MCU, 0x20UL);
    /* (1-2) FRC-R2 clcok. */
    MHal_R2_Write_Reg8(BASE_NON_PM, REG_CLKGEN1_FRC_R2, 0x08UL);

    /* (2) Set CPU SDR base: */
    /* I Fetch Offset - Low. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SDR_LO_INST_BASE, (u32Addr & 0x0000FFFFUL));
    /* I Fetch Offset - High. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SDR_HI_INST_BASE, (u32Addr >> 16));
    /* D Fetch Offset - Low. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SDR_LO_DATA_BASE, (u32Addr & 0x0000FFFFUL));
    /* D Fetch Offset - High. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SDR_HI_DATA_BASE, (u32Addr >> 16));

    /* (3) Set MAU Mapping. */
    if (u32Addr < ARM_MIU1_BASE_ADDR)
    {
        MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_MAU, 0x8800UL);
        MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_MAU_LV2, 0x8400UL);
    }
    else
    {
        MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_MAU, 0x8900UL);
        MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_MAU_LV2, 0x8B00UL);
    }

    /* (4) Set RIU base. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_RIU_BASE, 0xFA00UL);

    /* (5) Set SPI/UART base. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_IO1_BASE, 0xF800UL);
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SPI_BASE, 0xF900UL);
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SPI_BASE1, 0xF900UL);
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_DQMEM_BASE, 0xFB00UL);
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_QMEM_MASK_HIGH, 0xFF00UL);

    /* (6) Set IO space enable (UART, RIU) with QMEM space disabled. */
    MHal_R2_Write_Reg16(BASE_FRC_R2, REG_FRC_R2_SPACE_EN, 0x0003UL);

    /* (7) Enable R2 -> Start. */
    MHal_R2_Write_Reg8(BASE_FRC_R2, REG_FRC_R2_STOP, 0x24UL);
    MHal_R2_Write_Reg8(BASE_FRC_R2, REG_FRC_R2_STOP, 0x27UL);

    MHAL_R2_DEBUG("Enable done.\n");
}
#endif
