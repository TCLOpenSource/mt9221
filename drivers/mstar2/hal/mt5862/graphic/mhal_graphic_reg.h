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

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// @file   mhal_graphic_reg.h
// @brief  Graphic Driver Interface
// @author MStar Semiconductor Inc.
//////////////////////////////////////////////////////////////////////////////////////////////////


//=============================================================================
// Defines & Macros
//=============================================================================
#define BMASK(bits)                             (BIT(((1)?bits)+1)-BIT(((0)?bits)))
#define GOP_REG_VAL(x)                          (1<<x)

#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define RIU_MAP                                 (mstar_pm_base+0x200000UL)
#else
#define RIU_MAP                                 0xFD200000UL
#endif

#define RIU                                     ((unsigned short volatile *) RIU_MAP)

#define GOP_WRITE2BYTE(addr, val)    { RIU[addr] = val; }
#define GOP_READ2BYTE(addr)            RIU[addr]


//----------------------------------------------------------------------------
// MIU Reg
//----------------------------------------------------------------------------
#define GOP_MIU_CLIENT_GOP0     0x5
#define GOP_MIU_CLIENT_GOP1     0x6
#define GOP_MIU_CLIENT_GOP2     0x7
#define GOP_MIU_CLIENT_GOP3     0x8
#define GOP_MIU_CLIENT_GOP4     0xff

//----------------------------------------------------------------------------
// GOP related GOP Reg Defines
//----------------------------------------------------------------------------
#define GOP_REG_DIRECT_BASE                            (0x20200)
#define GOP_REG(bk, reg)                    (GOP_REG_DIRECT_BASE+((MS_U32)(bk)<<8) + (reg) * 2)

#define GOP_OFFSET_WR                       8
#define GOP_VAL_WR                          GOP_REG_VAL(GOP_OFFSET_WR)
#define GOP_OFFSET_FWR                      9
#define GOP_VAL_FWR                         GOP_REG_VAL(GOP_OFFSET_FWR)


#define GOP_4G_OFST                           0x0
#define GOP_2G_OFST                           0x3
#define GOP_1G_OFST                           0x6
#define GOP_1GX_OFST                          0x9
#define GOP_DW_OFST                           0xC

#define GOP_4G_CTRL0                        GOP_REG(0, 0x00)
#define GOP_HMIRROR_EN                      GOP_BIT12
#define GOP_VMIRROR_EN                      GOP_BIT13
#define GOP_4G_CTRL1                        GOP_REG(0, 0x01)
#define GOP_4G_RATE                         GOP_REG(0, 0x02)
#define GOP_4G_RDMA_HT                      GOP_REG(GOP_4G_OFST, 0x0e)
#define GOP_4G_HS_PIPE                      (GOP_REG(GOP_4G_OFST, 0x0f))
#define GOP_4G_BW                           GOP_REG(GOP_4G_OFST, 0x19)
#define GOP_4G_SRAM_BORROW                  GOP_REG(GOP_4G_OFST, 0x1D)
#define GOP_4G_MIU_SEL                      GOP_REG(GOP_4G_OFST, 0x1F)
#define GOP_4G_STRCH_HSZ                    GOP_REG(GOP_4G_OFST, 0x30)
#define GOP_4G_STRCH_VSZ                    GOP_REG(GOP_4G_OFST, 0x31)
#define GOP_4G_STRCH_HSTR                   GOP_REG(GOP_4G_OFST, 0x32)
#define GOP_4G_STRCH_VSTR                   GOP_REG(GOP_4G_OFST, 0x34)
#define GOP_4G_HSTRCH                       GOP_REG(0, 0x35)
#define GOP_4G_VSTRCH                       GOP_REG(0, 0x36)
#define GOP_4G_HSTRCH_INI                   GOP_REG(GOP_4G_OFST, 0x38)
#define GOP_4G_VSTRCH_INI                   GOP_REG(GOP_4G_OFST, 0x39)
#define GOP_4G_OLDADDR                      GOP_REG(GOP_4G_OFST, 0x3b)
#define GOP_4G_MULTI_ALPHA                  (GOP_REG(GOP_4G_OFST, 0x3c))
    #define GOP_OP_VDE_SEL                  (GOP_BIT10)
#define GOP_4G_BANK_FWR                     (GOP_REG(GOP_4G_OFST, 0x50))
    #define GOP_PIXEL_BASE_EN               (GOP_BIT7)
#define GOP_MUX                             (GOP_REG(GOP_4G_OFST, 0x7e))
#define GOP_BAK_SEL                         GOP_REG(GOP_4G_OFST, 0x7f)

//MUX Setting
#define GOP_MUX_SHIFT                        (0x2)
#define GOP_REGMUX_MASK                      BMASK((GOP_MUX_SHIFT-1):0)

#define GOP_4G_GWIN0_CTRL(id)               GOP_REG(GOP_4G_OFST+1, 0x00 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_RBLK_L(id)              GOP_REG(GOP_4G_OFST+1, 0x01 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_RBLK_H(id)              GOP_REG(GOP_4G_OFST+1, 0x02 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_HSTR(id)                     GOP_REG(GOP_4G_OFST+1, 0x04 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_HEND(id)                     GOP_REG(GOP_4G_OFST+1, 0x05 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_VSTR(id)                     GOP_REG(GOP_4G_OFST+1, 0x06 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_VEND(id)                     GOP_REG(GOP_4G_OFST+1, 0x08 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_RBLK_HSIZE(id)          GOP_REG(GOP_4G_OFST+1, 0x09 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_RBLK_SIZE_L(id)         GOP_REG(GOP_4G_OFST+1, 0x10 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_RBLK_SIZE_H(id)         GOP_REG(GOP_4G_OFST+1, 0x11 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_GWIN_ALPHA01(id)             GOP_REG(GOP_4G_OFST+1, 0x0A + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_VSTR_L(id)              GOP_REG(GOP_4G_OFST+1, 0x0C + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_VSTR_H(id)              GOP_REG(GOP_4G_OFST+1, 0x0D + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_HSTR(id)                GOP_REG(GOP_4G_OFST+1, 0x0E + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_HVSTOP_L(id)            GOP_REG(GOP_4G_OFST+1, 0x14 + (0x20*((id)%MAX_GOP0_GWIN)))
#define GOP_4G_DRAM_HVSTOP_H(id)            GOP_REG(GOP_4G_OFST+1, 0x15 + (0x20*((id)%MAX_GOP0_GWIN)))

#define GOP_4G_CTRL_CSC                     GOP_REG(GOP_4G_OFST+2, 0x20)
//----------------------------------------------------------------------------
// SC related Reg Defines
//----------------------------------------------------------------------------
#define XC_REG_DIRECT_BASE                  (0x30000)
#define XC_REG(BK, REG)                     (XC_REG_DIRECT_BASE+((MS_U32)(BK)<<8) + (REG) * 2)
#define XC_BK00_06_L                        XC_REG(0x00, 0x06)
#define XC_BK10_04_L                        XC_REG(0x10, 0x04)
#define XC_BK10_05_L                        XC_REG(0x10, 0x05)
#define XC_BK10_06_L                        XC_REG(0x10, 0x06)
#define XC_BK10_07_L                        XC_REG(0x10, 0x07)

//----------------------------------------------------------------------------
// ChipTop Reg
//----------------------------------------------------------------------------
#define GOP_CKG_REG_BASE                   (0x00B00)
#define GOP_GOP0CLK                        (GOP_CKG_REG_BASE+(0x40<<1))
#define GOP_G0_DISABLE_CLK                 (GOP_BIT0)
#define GOP_G1_DISABLE_CLK                 (GOP_BIT8)

#define GOP_GOP2CLK                        (GOP_CKG_REG_BASE+(0x41<<1))
#define GOP_G2_DISABLE_CLK                 (GOP_BIT0)
