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

#ifndef _MHAL_SC_H_
#define _MHAL_SC_H_

#include "mdrv_mstypes.h"

//-------------------------------------------------------------------------------------------------
//  Driver Capability
//-------------------------------------------------------------------------------------------------
#define SC_DEV_NUM                  1               // number of device
#define SC_IRQ                      E_IRQ_SMART//E_INT_IRQ_UART1 //E_IRQ_UART1
#define SC_IRQ2                     E_IRQ_UART2

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define SC_READ(id, addr)           (!(id) ? UART1_READ(addr) : UART2_READ(addr))
#define SC_WRITE(id, addr, val)     if (!(id)) { UART1_WRITE(addr, val);} else { UART2_WRITE(addr, val);}
#define SC_OR(id,addr, val)         if (!(id)) { UART1_OR(addr, val);   } else { UART2_OR(addr, val);   }
#define SC_AND(id,addr, val)        if (!(id)) { UART1_AND(addr, val);  } else { UART2_AND(addr, val);  }
#define SC_XOR(id,addr, val)        if (!(id)) { UART1_XOR(addr, val);  } else { UART2_XOR(addr, val);  }

#define SC_3M_UARTDIV(d)            (((d*9/5)>>1)+((d*9/5)&0x1UL))        // 3M/d=43.2M/(16*uart_div) => div=d*3/5
#define SC_4P5M_UARTDIV(d)          (((d*6/5)>>1)+((d*6/5)&0x1UL))        // 4.5M/d=43.2M/(16*uart_div) => div=d*3/5
#define SC_6M_UARTDIV(d)            (((d*9/10)>>1)+((d*9/10)&0x1UL))      // 6M/d=43.2M/(16*uart_div) => div=d*9/20


//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------
typedef struct
{
    MS_BOOL bTxLevelInt;
    MS_BOOL bCGT_TxFail;
    MS_BOOL bCGT_RxFail;
    MS_BOOL bCWT_TxFail;
    MS_BOOL bCWT_RxFail;
    MS_BOOL bBGT_Fail;
    MS_BOOL bBWT_Fail;
}HAL_SC_TX_LEVEL_GWT_INT;

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
MS_BOOL HAL_SC_GetIntTxLevelAndGWT(MS_U8 u8SCID, HAL_SC_TX_LEVEL_GWT_INT *pstTxLevelGWT);
void HAL_SC_ClearIntTxLevelAndGWT(MS_U8 u8SCID);
MS_BOOL HAL_SC_IsPendingINT(MS_U8 IIRReg);
MS_U8 HAL_SC_GetLsr(MS_U8 u8SCID);
MS_BOOL HAL_SC_CheckIntRstToIoEdgeFail(MS_U8 u8SCID);
void HAL_SC_MaskIntRstToIoEdgeFail(MS_U8 u8SCID);
MS_BOOL HAL_SC_IsBwtInsteadExtCwt(MS_U8 u8SCID);
void HAL_SC_RstToIoEdgeDetCtrl(MS_U8 u8SCID, MS_BOOL bEnable);
void HAL_SC_ResetPadCtrl(MS_U8 u8SCID, MS_BOOL eLevelHigh);
MS_U8 HAL_SC_GetTxFifoCnt(MS_U8 u8SCID);
MS_U8 HAL_SC_GetTxLevel(MS_U8 u8SCID);

#endif // _MHAL_TEMP_H_

