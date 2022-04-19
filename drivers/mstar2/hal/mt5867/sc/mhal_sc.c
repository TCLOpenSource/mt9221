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
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/version.h>
#include "mdrv_mstypes.h"
#include "reg_sc.h"
#include "mhal_sc.h"

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
static void _HAL_SC_ClearLsr(MS_U8 u8SCID)
{
    SC_OR(u8SCID, UART_LSR_CLR, UART_LSR_CLR_FLAG);
    SC_AND(u8SCID, UART_LSR_CLR, ~(UART_LSR_CLR_FLAG));
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
MS_BOOL HAL_SC_GetIntTxLevelAndGWT(MS_U8 u8SCID, HAL_SC_TX_LEVEL_GWT_INT *pstTxLevelGWT)
{
 MS_U8 u8RegData;

    u8RegData = SC_READ(u8SCID,UART_GWT_INT);

    //init
    pstTxLevelGWT->bTxLevelInt = FALSE;
    pstTxLevelGWT->bCWT_RxFail = FALSE;
    pstTxLevelGWT->bCWT_TxFail = FALSE;
    pstTxLevelGWT->bCGT_RxFail = FALSE;
    pstTxLevelGWT->bCGT_TxFail = FALSE;
    pstTxLevelGWT->bBGT_Fail = FALSE;
    pstTxLevelGWT->bBWT_Fail = FALSE;

    if (u8RegData & UART_GWT_TX_LEVEL_INT)
        pstTxLevelGWT->bTxLevelInt = TRUE;

    if (u8RegData & UART_GWT_CWT_RX_FAIL)
        pstTxLevelGWT->bCWT_RxFail = TRUE;

    if (u8RegData & UART_GWT_CWT_TX_FAIL)
        pstTxLevelGWT->bCWT_TxFail = TRUE;

    if (u8RegData & UART_GWT_CGT_RX_FAIL)
        pstTxLevelGWT->bCGT_RxFail = TRUE;

    if (u8RegData & UART_GWT_CGT_TX_FAIL)
        pstTxLevelGWT->bCGT_TxFail = TRUE;

    if (u8RegData & UART_GWT_BGT_FAIL)
        pstTxLevelGWT->bBGT_Fail = TRUE;

    if (u8RegData & UART_GWT_BWT_FAIL)
        pstTxLevelGWT->bBWT_Fail = TRUE;

   return TRUE;
}

void HAL_SC_ClearIntTxLevelAndGWT(MS_U8 u8SCID)
{
    SC_WRITE(u8SCID, UART_CTRL2, SC_READ(u8SCID, UART_CTRL2) | UART_CTRL2_FLAG_CLEAR);
    SC_WRITE(u8SCID, UART_CTRL2, SC_READ(u8SCID, UART_CTRL2) & (~UART_CTRL2_FLAG_CLEAR));
}

MS_BOOL HAL_SC_IsPendingINT(MS_U8 IIRReg)
{
    return TRUE;
}

MS_U8 HAL_SC_GetLsr(MS_U8 u8SCID)
{
    MS_U8 u8Data;

    u8Data = SC_READ(u8SCID,UART_LSR);

    _HAL_SC_ClearLsr(u8SCID);
    //for debug test option for Fix read clear register timing issue
    //printk(KERN_EMERG "mark u8Data=%X\n",u8Data);

    return u8Data;
}

MS_BOOL HAL_SC_CheckIntRstToIoEdgeFail(MS_U8 u8SCID)
{
    MS_U8 u8RegData;

    u8RegData = SC_READ(u8SCID,UART_SCFC);
    if (u8RegData & UART_SCFC_RST_TO_IO_EDGE_DET_FAIL)
      return TRUE;
    else
    return FALSE;
}

void HAL_SC_MaskIntRstToIoEdgeFail(MS_U8 u8SCID)
{
    SC_WRITE(u8SCID, UART_CTRL15, SC_READ(u8SCID, UART_CTRL15) | UART_CTRL15_RST_TO_IO_EDGE_DET_FAIL_MASK);
}

MS_BOOL HAL_SC_IsBwtInsteadExtCwt(MS_U8 u8SCID)
{
    //no support
    return FALSE;
}

void HAL_SC_RstToIoEdgeDetCtrl(MS_U8 u8SCID, MS_BOOL bEnable)
{
    if (bEnable)
    {
        SC_OR(u8SCID, UART_SCFC, UART_SCFC_RST_TO_IO_EDGE_DET_EN);
    }
    else
{
        SC_AND(u8SCID, UART_SCFC, ~(UART_SCFC_RST_TO_IO_EDGE_DET_EN));
    }
}

void HAL_SC_ResetPadCtrl(MS_U8 u8SCID, MS_BOOL eLevelHigh)
{
    if (eLevelHigh)
    {
        SC_OR(u8SCID, UART_SCCR, UART_SCCR_RST);//Pull RSTIN high
    }
    else
    {
        SC_AND(u8SCID, UART_SCCR, ~UART_SCCR_RST);//Pull RSTIN low
    }
}

MS_U8 HAL_SC_GetTxFifoCnt(MS_U8 u8SCID)
{
    MS_U8 u8Reg;

    u8Reg = SC_READ(u8SCID, UART_TX_FIFO_COUNT);

    return (u8Reg & 0x3F);
}

MS_U8 HAL_SC_GetTxLevel(MS_U8 u8SCID)
{
    MS_U8 u8Reg;
    MS_U8 u8TxLevel = 0x00;

    u8Reg = SC_READ(u8SCID, UART_CTRL2);
    u8Reg = u8Reg & UART_CTRL2_TX_LEVEL;

    switch (u8Reg)
    {
        case UART_CTRL2_TX_LEVEL_5_TO_4:
            u8TxLevel = 5;
            break;

        case UART_CTRL2_TX_LEVEL_9_TO_8:
        default:
            u8TxLevel = 9;
            break;

        case UART_CTRL2_TX_LEVEL_17_TO_16:
            u8TxLevel = 17;
            break;

        case UART_CTRL2_TX_LEVEL_31_TO_30:
            u8TxLevel = 31;
            break;
    }

    return u8TxLevel;
}
