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
/// @file   drvBDMA.c
/// @brief  MStar BDMA DRV Driver DDI
/// @author MStar Semiconductor Inc.
/// @attention
/// <b><em></em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////

#define _DRV_BDMA_C

/* Kernel includes. */
#include "mosWrapper.h"

/* driver includes. */
#include "drvBDMA.h"
#include "halBDMA.h"

#define BDMA_MSG(fmt, args...) MOS_DBG_PRINT(MOS_DBG_LEVEL_BDMA, "[BDMA]" fmt, ##args)
#define BDMA_ERROR(fmt, args...) MOS_DBG_ERROR(fmt, ##args);


BOOL MDrv_BDMA_Copy(U32 u32Src, U32 u32Dst, U32 u32Len, BDMA_Mode_e eMode)
{
    BDMA_Ch ch;
    switch(eMode)
    {
      case E_BDMA_CPtoHK:
        ch = E_BDMA_CH0;
        MHal_BDMA_IMItoMIU(ch, u32Src, u32Dst, u32Len);
        break;
      case E_BDMA_HKtoCP:
        ch = E_BDMA_CH1;
        MHal_BDMA_MIUtoIMI(ch, u32Src, u32Dst, u32Len);
        break;
      case E_BDMA_CPtoCP:
        ch = E_BDMA_CH0;
        MHal_BDMA_IMItoIMI(ch, u32Src, u32Dst, u32Len);
        break;
      case E_BDMA_HKtoHK:
      	ch = E_BDMA_CH1;
        MHal_BDMA_MIUtoMIU(ch, u32Src, u32Dst, u32Len);
        break;
      default:
        BDMA_ERROR("MDrv_BDMA_Copy: mode %d failed\n", eMode);
        return FALSE;
    }
    MHal_BDMA_PollingDone(ch);

    return TRUE;

}


U32 MDrv_BDMA_Crc32(U32 u32Src, U32 u32Len, BDMA_Dev eDev)
{
    BDMA_Ch ch = E_BDMA_CH1;
    switch(eDev)
    {
      case E_BDMA_DEV_MIU:
        MHal_BDMA_CalculateCRC32FromMIU(ch, u32Src, u32Len);
        break;
      case E_BDMA_DEV_IMI:
        MHal_BDMA_CalculateCRC32FromIMI(ch, u32Src, u32Len);
        break;
    }
    MHal_BDMA_PollingDone(ch);

    return MHal_BDMA_GetCRC32(ch);
}

BOOL MDrv_BDMA_PatternSearch(U32 u32Src, U32 u32Len, U32 u32Pattern, BDMA_Dev eDev)
{
    BDMA_Ch ch = E_BDMA_CH1;
    switch(eDev)
    {
      case E_BDMA_DEV_IMI:
        return MHal_BDMA_PatternSearchFromIMI(ch, u32Src, u32Len, u32Pattern);
      default:
        BDMA_ERROR("MDrv_BDMA_PatternSearch: Dev %d not support\n", eDev);
        return FALSE;
    }
}

