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

//=============================================================================
// Include Files
//=============================================================================
#include "mosWrapper.h"
#include "halCHIPTOP.h"
#include "halCPUINT.h"

//=============================================================================
// Debug Macros
//=============================================================================
#define PM_MISC_BASE_ADDR         GET_REG8_ADDR(RIU_BASE_ADDR, 0x2E00 )

#define REG_ADDR_BASE_INTC        GET_REG8_ADDR( RIU_BASE_ADDR, 0x1019C0 )
#define REQ_INTC_FIQ_MASK_47_32   GET_REG16_ADDR(REG_ADDR_BASE_INTC, 0x06)

#define REG_ADDR_BASE_PM_SLEEP    GET_REG8_ADDR( RIU_BASE_ADDR, 0xE00 )
#define REG_PM_UART_PAD           GET_REG16_ADDR(REG_ADDR_BASE_PM_SLEEP, 0x76)

#define INSREG8(addr, mask, val)    OUTREG8(addr, ((INREG8(addr)&(~(mask))) | val))
#define INSREG16(addr, mask, val)   OUTREG16(addr, ((INREG16(addr)&(~(mask))) | val))

#define WRIU16(x, y)            OUTREG16(GET_REG8_ADDR(RIU_BASE_ADDR, x), y)
#define IRIU16(x, y, Z)         INSREG16(GET_REG8_ADDR(RIU_BASE_ADDR, x), y, Z)
#define IRIU8(x, y, Z)         INSREG8(GET_REG8_ADDR(RIU_BASE_ADDR, x), y, Z)
//=============================================================================
// CM4 HAL Driver Function
//=============================================================================
void MHal_CM4_Init (void)
{
// AC_ON
//  $display("PAGASIM_START");
 // WRIU16(0x032506, 0x0021); // PAGANIN PLL 396 MHz
  WRIU16(0x032502, 0x0001); // [8]reg_dsppll_pd_dft PAGANINIPLL setup
  mdelay(1);

  // AC ON
  WRIU16(0x0325F0, 0x0001); // XTAIL ON


  // CM4 TSV & SYSTICK
  WRIU16(0x0325F8, 0x1000); // [8]REG_CLK_CM4_TSV_EN; [10] REG_CLK_CM4_SYS_EN ; [12] REG_CLK_CM4_SYSTICK_GF_SEL


  // IMI CLK  XTAIL -> DFS OFF -> PLL -> DFS_ON
  WRIU16(0x0325F4, 0x0380); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  mdelay(1);

  WRIU16(0x0325F2, 0x023F); // [5:0] REG_CLK_IMI_DFS_DIV; [9] REG_CLK_IMI_DFS_NOBYPASS
  WRIU16(0x0325F2, 0x033F); // [5:0] REG_CLK_IMI_DFS_DIV; [9] REG_CLK_IMI_DFS_NOBYPASS ; [8] REG_CLK_IMI_DFS_UPDATE
  WRIU16(0x0325F2, 0x023F); // [5:0] REG_CLK_IMI_DFS_DIV; [9] REG_CLK_IMI_DFS_NOBYPASS ; [8] REG_CLK_IMI_DFS_UPDATE
  mdelay(1);

  WRIU16(0x0325F8, 0x1500);  // [8]REG_CLK_CM4_TSV_EN; [10] REG_CLK_CM4_SYS_EN ; [12] REG_CLK_CM4_SYSTICK_GF_SEL

  WRIU16(0x0325F4, 0x0388); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  mdelay(1);

  WRIU16(0x0325F4, 0x0398); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL

  WRIU16(0x0325F2, 0x021F); // [5:0] REG_CLK_IMI_DFS_DIV; [9] REG_CLK_IMI_DFS_NOBYPASS
  WRIU16(0x0325F2, 0x031F); // [5:0] REG_CLK_IMI_DFS_DIV; [9] REG_CLK_IMI_DFS_NOBYPASS ; [8] REG_CLK_IMI_DFS_UPDATE
  WRIU16(0x0325F2, 0x021F); // [5:0] REG_CLK_IMI_DFS_DIV; [9] REG_CLK_IMI_DFS_NOBYPASS ; [8] REG_CLK_IMI_DFS_UPDATE

  WRIU16(0x0325F8, 0x1580); // [3] REG_CLK_CM4_DFS_EN ; [4]REG_CLK_CM4_GF_SEL  ; [7]REG_CLK_CM4_EN
  mdelay(1);

  WRIU16(0x0325F6, 0x023F); // [5:0] REG_CLK_CM4_DFS_DIV; [9] REG_CLK_CM4_DFS_NOBYPASS
  WRIU16(0x0325F6, 0x033F); // [5:0] REG_CLK_CM4_DFS_DIV; [9] REG_CLK_CM4_DFS_NOBYPASS ; [8] REG_CLK_CM4_DFS_UPDATE
  WRIU16(0x0325F6, 0x023F); // [5:0] REG_CLK_CM4_DFS_DIV; [9] REG_CLK_CM4_DFS_NOBYPASS ; [8] REG_CLK_CM4_DFS_UPDATE

  WRIU16(0x0325F8, 0x1590);  // [3] REG_CLK_CM4_DFS_EN ; [4]REG_CLK_CM4_GF_SEL  ; [7]REG_CLK_CM4_EN
  mdelay(1);
  IRIU8(0x0325e6,0x02,0x02); // REG_CLK_CM4_CORE_DFS_EN
  WRIU16(0x0325F8, 0x1598); // [3] REG_CLK_CM4_DFS_EN ; [4]REG_CLK_CM4_GF_SEL  ; [7]REG_CLK_CM4_EN

  WRIU16(0x0325F6, 0x021F); // [5:0] REG_CLK_CM4_DFS_DIV; [9] REG_CLK_CM4_DFS_NOBYPASS
  WRIU16(0x0325F6, 0x031F); // [5:0] REG_CLK_CM4_DFS_DIV; [9] REG_CLK_CM4_DFS_NOBYPASS ; [8] REG_CLK_CM4_DFS_UPDATE
  WRIU16(0x0325F6, 0x021F); // [5:0] REG_CLK_CM4_DFS_DIV; [9] REG_CLK_CM4_DFS_NOBYPASS ; [8] REG_CLK_CM4_DFS_UPDATE

  WRIU16(0x032538, 0x0000); //power on audio sram/imi
  CLRREG8(GET_REG8_ADDR(RIU_BASE_ADDR, 0x32527), 0x80);
  IRIU16(0x032522, 0x4000,0x4000); // same freq mode for bridge of IMI/MIU
}

void MHal_CM4_Halt (void)
{
    CLRREG16(GET_REG16_ADDR(PM_MISC_BASE_ADDR, 0x27), 0x10); //reset cm4 cpu
    MHal_CPUINT_Clear(E_CPUINT_CP, E_CPUINT_HK0);
    CLRREG16(REQ_INTC_FIQ_MASK_47_32, 0x100); //fiq40 HK0 to CP
}
void MHal_CM4_Run (void)
{
    SETREG16(GET_REG16_ADDR(PM_MISC_BASE_ADDR, 0x27), 0x10);
}

void MHal_CM4_DcOn (void)
{
  //U16 u16Val;
  // default not set uart pad
//  u16Val = INREG16(REG_PM_UART_PAD);
//  u16Val &= ~(7<<6);
//  OUTREG16(REG_PM_UART_PAD, (u16Val |(0x1<<6)));

  CLRREG16(REQ_INTC_FIQ_MASK_47_32, 0x100); //fiq40 HK0 to CP

  WRIU16(0x0325F4, 0x0198); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0188); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0189); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0199); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0399); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL

  // bdma stop
  IRIU16(0x151a00, 0x0010, 0x0000);
  IRIU16(0x151a00, 0x0010, 0x0010);
  // bridge rst
  IRIU16(0x112e52, 0x003f, 0x000f);
  IRIU16(0x112e52, 0x003f, 0x001f);
  IRIU16(0x112e52, 0x003f, 0x003f);
  IRIU16(0x112e52, 0x003f, 0x001f);
  IRIU16(0x112e52, 0x003f, 0x000f);
  IRIU16(0x112e52, 0x003f, 0x0000);
  // bdma stop release
  IRIU16(0x151a00, 0x0010, 0x0000);

  /* miu mask*/
  WRIU16(0x310206, 0x0000); //wriu -w 0x310206 0x0000 // MASK 20190528
}

void MHal_CM4_DcOff (void)
{
  WRIU16(0x0325F4, 0x0199); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL  [9:8] 0/1 -> [4] 0 -> [1:0] 0/1 -> [4] 1 ->  [9:8] 3
  WRIU16(0x0325F4, 0x0189); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0188); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0198); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
  WRIU16(0x0325F4, 0x0398); // IMI CLK [9:8]REG_CLK_IMI_GF_PRE_SEL ; [7] REG_CLK_IMI_EN ; [4]REG_CLK_IMI_GF_SEL
}

void MHal_CM4_InitOff(void)
{
  WRIU16(0x032502, 0x017d); //wriu –w 0x032502 0x017d      // CM4 PLL PD
  WRIU16(0x0325F0, 0x0000);//wriu –w 0x0325F0 0x0000      // XTAIL off
  WRIU16(0x032538, 0xffff);//wriu –w 0x032538 0xffff         // power off audio sram/imi
  IRIU8(0x032527, 0x80, 0x80); //wriu –b  0x032527 0x80 0x80 
  WRIU16(0x0325f8, 0x0000); //wriu –w 0x0325f8 0x0000       // cm4 clk off
  WRIU16(0x0325f4, 0x0000);//wriu –w 0x0325f4 0x0000       // imi clk off
  WRIU16(0x032530, 0x0000);//wriu –w 0x032530 0x0000      // NF synthizer off

  IRIU8(0x0325e6,0x02,0x00); // REG_CLK_CM4_CORE_DFS_EN
}

void MHal_CM4_Seamless(BOOL bEn)
{
  IRIU8(0x03231C, 0x1, (bEn?0x1:0x0));
}

void MHal_CM4_UartEnable(BOOL bEn)
{
    U16 u16Val;
    if (bEn)
    {
        u16Val = INREG16(REG_PM_UART_PAD);
        u16Val &= ~(7<<6);
        OUTREG16(REG_PM_UART_PAD, (u16Val |(0x1<<6)));
    }
    else
    {
        u16Val = INREG16(REG_PM_UART_PAD);
        u16Val &= ~(7<<6);
        OUTREG16(REG_PM_UART_PAD, (u16Val |(0x0<<6)));
    }
}
