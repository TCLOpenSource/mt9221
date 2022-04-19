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
/// file    drv_system_io.c
/// @brief  System Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/types.h>

#include "mdrv_types.h"
#include "mst_platform.h"
#include "mhal_system.h"
#if defined(CONFIG_MMC_MSTAR_MMC_EMMC) && CONFIG_MMC_MSTAR_MMC_EMMC
extern bool mstar_mci_exit_checkdone_ForCI(void);
#endif

// NOTE
// PE capability :
// PAD as output
//  PE = 0 / 1, don't care
// PAD as input
//  PE = 0, Hi-Z (input disable)
//  PE = 1, internal pull up
//
// for PAD = PCM_A(output), PE don't care
// for PAD = PCM_D(input/output), PE must be 1

#define PAD_PCM_D_ENABLE() \
    REG_ADDR(REG_CHIP_PCM_PE)   |= REG_CHIP_PCM_PE_MASK;

#define PAD_PCM_A_ENABLE() \
    REG_ADDR(REG_CHIP_PCM_PE1)   |= REG_CHIP_PCM_PE1_MASK;

#define PAD_PCM_D_DISABLE() \
    REG_ADDR(REG_CHIP_PCM_PE)   &= ~REG_CHIP_PCM_PE_MASK;

#define PAD_PCM_A_DISABLE() \
    REG_ADDR(REG_CHIP_PCM_PE1)   &= ~REG_CHIP_PCM_PE1_MASK;

static U16 u16TmpReg[8];

#if defined(CONFIG_MMC_MSTAR_MMC_EMMC) && CONFIG_MMC_MSTAR_MMC_EMMC
BOOL MHal_PCMCIA_WaitEMMCDone(U32 u32loopCnt)
{
    return mstar_mci_exit_checkdone_ForCI();
}
#endif

void MHal_PCMCIA_SetPad(BOOL bRestore)
{
    if(bRestore == FALSE)
    {
        // Switch to PCMCIA
        u16TmpReg[6] = REG_ADDR( REG_CHIP_NAND_MODE ) & REG_CHIP_NAND_MODE_MASK;
        REG_ADDR( REG_CHIP_NAND_MODE ) &= ~REG_CHIP_NAND_MODE_MASK;

        //u16Bypass = REG_ADDR(REG_CHIP_PCM_NAND_BYPASS) & REG_CHIP_PCM_NAND_BYPASS_MASK;
        u16TmpReg[0] = REG_ADDR(REG_CHIP_PCM_NAND_BYPASS) & REG_CHIP_PCM_NAND_BYPASS_MASK;
        REG_ADDR(REG_CHIP_PCM_NAND_BYPASS) |= REG_CHIP_PCM_NAND_BYPASS_ENABLE;

        //u16RegVal3 = REG_ADDR(REG_EMMC_CONFIG) & REG_EMMC_CONFIG_MASK;
        u16TmpReg[7] = REG_ADDR(REG_EMMC_CONFIG2) & REG_EMMC_CONFIG2_MASK;
        REG_ADDR(REG_EMMC_CONFIG2) &= ~REG_EMMC_CONFIG2_MASK;

        //u16RegVal1 = REG_ADDR(REG_CHIP_PCMCFG) & REG_CHIP_PCMCFG_MASK;
        u16TmpReg[3] = REG_ADDR(REG_CHIP_PCMCFG) & REG_CHIP_PCMCFG_MASK;
        REG_ADDR(REG_CHIP_PCMCFG) |= REG_CHIP_PCMCFG_CTRL_EN;

        //u16RegVal4 = REG_ADDR(REG_SD_CONFIG2) & REG_SD_CONFIG2_MASK;
        //u16TmpReg[4] = REG_ADDR(REG_SD_CONFIG2) & REG_SD_CONFIG2_MASK;
        //REG_ADDR(REG_SD_CONFIG2) &= ~REG_SD_CONFIG2_MASK;

        // disable PCM_A PE for NEOTION CAM issue (mantis id:0365320)
        PAD_PCM_A_DISABLE();
        PAD_PCM_D_ENABLE();
    }
    else
    {
        // restore padmux to original
        PAD_PCM_D_DISABLE();

        REG_ADDR(REG_EMMC_CONFIG2) = (REG_ADDR(REG_EMMC_CONFIG2) & ~REG_EMMC_CONFIG2_MASK) | u16TmpReg[7];

        REG_ADDR(REG_CHIP_PCMCFG) = (REG_ADDR(REG_CHIP_PCMCFG) & ~REG_CHIP_PCMCFG_MASK) | u16TmpReg[3];

        //REG_ADDR(REG_SD_CONFIG2) = (REG_ADDR(REG_SD_CONFIG2) & ~REG_SD_CONFIG2_MASK) | u16TmpReg[4];

        REG_ADDR(REG_CHIP_PCM_NAND_BYPASS) = 
            (REG_ADDR(REG_CHIP_PCM_NAND_BYPASS) & ~(REG_CHIP_PCM_NAND_BYPASS_MASK)) | u16TmpReg[0];;

        // Switch to Nand
        REG_ADDR( REG_CHIP_NAND_MODE ) |= u16TmpReg[6]; //(BIT0|BIT2);

    }

}
