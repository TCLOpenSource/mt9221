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
/// @file   mhal_gflip_reg.h
/// @brief  MStar gflip reg defines
/// @author MStar Semiconductor Inc.
/// @attention
/// <b><em>The g(graphics, gop/ge) flip will have related regs with GOP/GE</em></b>
///////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MHAL_GFLIP_REG_H
#define _MHAL_GFLIP_REG_H

//=============================================================================
// Includs
//=============================================================================

//=============================================================================
// Defines & Macros
//=============================================================================
#define BMASK(bits)                             (BIT(((1)?bits)+1)-BIT(((0)?bits)))
#define BITS(bits, val)                         ((BIT(((1)?bits)+1)-BIT(((0)?bits))) & (val<<((0)?bits)))
//----------------------------------------------------------------------------
// GFLIP related Reg Defines
//----------------------------------------------------------------------------
extern ptrdiff_t mstar_pm_base;
#define RIU_MAP                                 (mstar_pm_base+0x200000UL)

#define RIU                                     ((unsigned short volatile *) RIU_MAP)
#define RIU8                                    ((unsigned char  volatile *) RIU_MAP)

//----------------------------------------------------------------------------
// GFLIP related GOP Reg Defines
//----------------------------------------------------------------------------
#define REG_GOP_BASE                            (0x20200)
#define REG_MIU0_BASE                           (0x01200)
#define REG_MIU1_BASE                           (0x00600)
#define REG_MIU2_BASE                           (0x62000)
#define GOP_REG(addr, bk)                       GOP_DIR_REG(addr,REG_GOP_BASE+(((MS_U16)bk)<<8))
#define GOP_DIR_REG(addr,REG_GOP_DIR_BASE)      RIU[(addr<<1)+REG_GOP_DIR_BASE]
#define MIU0_REG(addr)                          GOP_DIR_REG(addr,REG_MIU0_BASE)
#define MIU1_REG(addr)                          GOP_DIR_REG(addr,REG_MIU1_BASE)
#define MIU2_REG(addr)                          GOP_DIR_REG(addr,REG_MIU2_BASE)
#define GOP_MIU_REG                             0x7D
#define GOP_MIU_CLIENT                          (BIT(5)|BIT(6)|BIT(7)|BIT(8))

#define REG_GOP_DRAM_RBLK_STR_L(widx)           (0x0001+(widx<<5))
#define REG_GOP_DRAM_RBLK_STR_H(widx)           (0x0002+(widx<<5))
#define REG_GOP_DRAM_RBLK_STR_EX(widx)          (0x0003+(widx<<5))

#define REG_GOP_3DOSD_SUB_RBLK_L(widx)          (0x001E + (widx<<5))
#define REG_GOP_3DOSD_SUB_RBLK_H(widx)          (0x001F + (widx<<5))
#define REG_GOP_3DOSD_SUB_RBLK_EX(widx)         (0x001D + (widx<<5))

#define REG_GOP4G_GWIN_CTRL0(widx)              (0x0000+(widx<<5))
    #define GOP_GWIN_ENABLE_MASK                BIT(0)

#define REG_GOP4G_CTRL0                         0x0000
    #define GOP_RST_MASK                        BIT(0)

#define REG_GOP_CTRL1                           0x0001
    #define GOP_DST_MASK                        BMASK(3:0)

#define REG_GOP_4G_PALDATA_L                    (0x0003)
#define REG_GOP_4G_PALDATA_H                    (0x0004)
#define REG_GOP_4G_PALCTRL                      (0x0005)
#define REG_GOP_4G_OLDADDR                      (0x003B)

#define REG_GOP_INT                             0x0008
    #define GOP_INTMASK_VS0                     BIT(0)
    #define GOP_INTMASK_VS1                     BIT(1)
    #define GOP_INT_VS0                         BIT(8)
    #define GOP_INT_VS1                         BIT(9)

#define REG_GOP_BANK_SEL_EX                     0x007c
    #define GOP1GX0_INT_FLAG                    BMASK(0:0)
    #define GOP1GX0_WR_ACK                      BMASK(4:4)

#define REG_GOP_MUX                             0x007e
    #define GOPD_INT_FLAG_MASK                  BMASK(12:12)
    #define GOPD_INT_FLAG                       BIT(12)
    #define GOPD_WR_ACK                         BIT(13)

#define REG_GOP_BANK_SEL                        0x007f
    #define GOP_BANK_SEL                        BMASK(3:0)
    #define GOP_INT_FLAG_MASK                   BMASK(7:4)
    #define GOP2G_INT_FLAG                      BIT(4)
    #define GOP2GX_INT_FLAG                     BIT(5)
    #define GOP1G_INT_FLAG                      BIT(6)
    #define GOP1GX_INT_FLAG                     BIT(7)
    #define GOP_WR                              BIT(8)
    #define GOP_FWR                             BIT(9)
    #define GOP_BK_WR                           BIT(10)
    #define GOP_FCLR                            BIT(11)
    #define GOP2G_WR_ACK                        BIT(12)
    #define GOP2GX_WR_ACK                       BIT(13)
    #define GOP1G_WR_ACK                        BIT(14)
    #define GOP1GX_WR_ACK                       BIT(15)

#define REG_GOP_DWIN_INT                        0x0002
    #define GOP_DWIN_INTMASK_WADR               BIT(3)
    #define GOP_DWIN_INTMASK_PROG               BIT(4)
    #define GOP_DWIN_INTMASK_TF                 BIT(5)
    #define GOP_DWIN_INTMASK_BF                 BIT(6)
    #define GOP_DWIN_INTMASK_VS                 BIT(7)
    #define GOP_DWIN_INT_WADR                   BIT(11)
    #define GOP_DWIN_INT_PROG                   BIT(12)
    #define GOP_DWIN_INT_TF                     BIT(13)
    #define GOP_DWIN_INT_BF                     BIT(14)
    #define GOP_DWIN_INT_VS                     BIT(15)

#define REG_GOP_TLB_MAIN_ADDR_L                 0x0058
#define REG_GOP_TLB_MAIN_ADDR_H                 0x0059
#define REG_GOP_TLB_SUB_ADDR_L                  0x005a
#define REG_GOP_TLB_SUB_ADDR_H                  0x005b

//----------------------------------------------------------------------------
// GFLIP related chiptop Reg Defines
//----------------------------------------------------------------------------
#define CKG_REG_BASE                            0xB00
#define CKG_REG(addr)                           RIU[(addr<<1)+CKG_REG_BASE]

/* GOP0 and GOP1 CLK */
#define GOP_GOPCLK                              0x40
    #define CKG_GOPG0_DISABLE_CLK               ~(BIT(0))
    #define CKG_GOPG0_MASK                      BMASK(4:2)
    #define CKG_GOPG0_ODCLK                     BITS(4:2, 0)
    #define CKG_GOPG0_IDCLK2                    BITS(4:2, 1)
    #define CKG_GOPG0_IDCLK1                    BITS(4:2, 2)

    #define CKG_GOPG1_DISABLE_CLK               ~(BIT(8))
    #define CKG_GOPG1_MASK                      BMASK(12:10)
    #define CKG_GOPG1_ODCLK                     BITS(12:10, 0)
    #define CKG_GOPG1_IDCLK2                    BITS(12:10, 1)
    #define CKG_GOPG1_IDCLK1                    BITS(12:10, 2)

/* GOP2 and GOPDWIN CLK */
#define GOP_GOP2CLK                             0x41
    #define CKG_GOPG2_DISABLE_CLK               ~(BIT(0))
    #define CKG_GOPG2_MASK                      BMASK(4:2)
    #define CKG_GOPG2_ODCLK                     BITS(4:2, 0)
    #define CKG_GOPG2_IDCLK2                    BITS(4:2, 1)
    #define CKG_GOPG2_IDCLK1                    BITS(4:2, 2)

    #define CKG_GOPD_MASK                       BMASK(12:10)
    #define CKG_GOPD_CLK_ODCLK                  BMASK(12:10)

/* GOP3 CLK*/
#define GOP_GOP3CLK                             0x42
    #define CKG_GOPG3_MASK                      BMASK(4:2)
    #define CKG_GOPG3_ODCLK                     BITS(4:2, 0)
    #define CKG_GOPG3_IDCLK2                    BITS(4:2, 1)
    #define CKG_GOPG3_IDCLK1                    BITS(4:2, 2)

/* GOP4 CLK*/
#define GOP_GOP4CLK                             0x7E
    #define CKG_GOPG4_MASK                      BMASK(12:10)
    #define CKG_GOPG4_ODCLK                     BITS(12:10, 0)
    #define CKG_GOPG4_IDCLK2                    BITS(12:10, 1)
    #define CKG_GOPG4_IDCLK1                    BITS(12:10, 2)

    #define CKG_GOPD_DISABLE_CLK                ~(BIT(8))

#define GOP_SRAMCLK             0x43
/* GOP SRAM CLK*/
#define GOP_SCLCLK              0x44
#define GOP_LB_SRAMCLK          0x45
//----------------------------------------------------------------------------
// GFLIP related GE Reg Defines
//----------------------------------------------------------------------------
#define REG_GE_BASE                             (0x2800)
#define GE_REG(addr)                            RIU[(addr<<1)+REG_GE_BASE]

#define REG_GE_FMT_BLT                          0x01
    #define GE_EN_CMDQ                          BIT(0)
    #define GE_EN_VCMDQ                         BIT(1)

#define REG_GE_VQ_FIFO_STATUS_L                 0x0004
#define REG_GE_VQ_FIFO_STATUS_H                 0x0005

#define REG_GE_STATUS                           0x0007
    #define GE_BUSY                             BIT(0)
    #define GE_CMDQ1_STATUS                     BMASK(7:3)
    #define GE_CMDQ2_STATUS                     BMASK(15:11)

#define REG_GE_TAG                              0x0032

//----------------------------------------------------------------------------
// GFLIP related VE Reg Defines, Used for VE capture
//----------------------------------------------------------------------------
#define REG_VE_BASE                             (0x3B00)
#define VE_REG(addr)                            RIU[(addr<<1)+REG_VE_BASE]

#define REG_VE_CTRL                             0x00
    #define BIT_VE_ENABLE                       BIT(0)
    #define BIT_VE_RESET                        BIT(4)
    #define BIT_VE_REG_RELOAD                   BIT(5)

#define REG_VE_STATE                            0x7F
    #define BIT_VE_START_DOWNSCALING            BIT(0)

#define REG_VE_COUNTER                          0x70
    #define BIT_VE_FIELD_IDX                    BMASK(15:14)
    #define BIT_VE_FIELD_IDX_SHIFT_BITS         14

//----------------------------------------------------------------------------
// GFLIP related GS Reg Defines
//----------------------------------------------------------------------------
#define CKG_GS_BASE                           0x33D00
#define GS_REG(addr)                           RIU[(addr<<1)+CKG_GS_BASE]
#define REG_GS_VSP_SRAM                  0x0D

#define SC_REG_BASE                            0x2F00
#define SC_REG(addr)                           RIU[(addr<<1)+SC_REG_BASE]
#define GOP_SC_BANKSEL                          0x0
#define GOP_SC_GOPEN                            0x0
#define GOP_SC1_GOPEN                           0x80
#define GOP_SC_IP2SC                            0x5
#define GOP_SC_VOP2BLENDING_L                   0x6
#define GOP_SC_VOP2BLENDING_H                   0x6
#define GOP_SC_OCBANKSEL                        0x37
#define GOP_SC_OCMIXER_ALPHA                    0x28

#define GOP_SC_MIUBANKSEL                       0x7F
    #define GOP_SC_MIUSEL_HW_SW                 0x10
    #define GOP_SC_MIUSEL_L                     0x11
    #define GOP_SC_MIUSEL_H                     0x18

#define GOP_SC_GOPBLENDING                      GOP_SC_GOPEN
#define GOP_SC_GOPBLENDING_L                    GOP_SC_VOP2BLENDING_L
#define GOP_SC_GOPBLENDING_H                    GOP_SC_VOP2BLENDING_H

#define GOP_SC_DUALRATE_BLENDING_BNK            0xC9
    #define GOP_SC_DUALRATE_BLENDING0           0x50
    #define GOP_SC_DUALRATE_BLENDING1           0x51
    #define GOP_SC_DUALRATE_BLENDING2           0x52
#define GOP_SC_HSYNC_SHIFT_BNK                  0xCB
    #define GOP_SC_HSYNC_SHIFT                  0x48

#define AFBC_REG_BASE                            0x13100
#define AFBC_REG(addr)                           RIU[((addr)<<1)+AFBC_REG_BASE]

#define GOP_AFBC_CLK                            0x46
    #define CKG_AFBCCLK_DISABLE_CLK   (GOP_BIT0)
    #define CKG_AFBCCLK_432           (0 << 2)
    #define CKG_AFBCCLK_216           (1 << 2)

#define REG_AFBC_CORE_EN(id)                    AFBC_REG(0x00+(0x20*id))
#define REG_AFBC_ADDR_L(id)                     AFBC_REG(0x01+(0x20*id))
#define REG_AFBC_ADDR_H(id)                     AFBC_REG(0x02+(0x20*id))
#define REG_AFBC_FMT(id)                        AFBC_REG(0x0C+(0x20*id))
#define REG_AFBC_WIDTH(id)                      AFBC_REG(0x0A+(0x20*id))
#define REG_AFBC_HEIGHT(id)                     AFBC_REG(0x0B+(0x20*id))
#define REG_AFBC_RESP(id)                       AFBC_REG(0x0F+(0x20*id))
#define REG_AFBC_MIU                            AFBC_REG(0x43)
#define REG_AFBC_TRIGGER                        AFBC_REG(0x50)


#define SC_DIRECT_BNK                           0x30000L
#define SC_BNK_REG(bnk, addr)                   RIU[((addr)<<1)+(bnk<<8)+(SC_DIRECT_BNK)]

#define GOP_MANUAL_CROP_BNK                     0x70L
    #define REG_GOP_CROP_PRECALDONE(gop_id)     SC_BNK_REG(GOP_MANUAL_CROP_BNK, (0x0 + 0x20*(gop_id))) // (0x0 + 0x20*(gop_id))
    #define REG_GOP_CROP_ORIHSTART(gop_id)      SC_BNK_REG(GOP_MANUAL_CROP_BNK, (0x7 + 0x20*(gop_id))) // (0x7 + 0x20*(gop_id))
    #define REG_GOP_CROP_ORIHEND(gop_id)        SC_BNK_REG(GOP_MANUAL_CROP_BNK, (0x8 + 0x20*(gop_id))) // (0x8 + 0x20*(gop_id))

#define MIU0_ARB_BASE                                0x52000
#define MIU1_ARB_BASE                                0x52100
#define MIU2_ARB_BASE                                0x52200

#define MIU0_ARB_REG(addr)                           RIU[(addr<<1)+MIU0_ARB_BASE]
#define MIU1_ARB_REG(addr)                           RIU[(addr<<1)+MIU1_ARB_BASE]
#define MIU2_ARB_REG(addr)                           RIU[(addr<<1)+MIU2_ARB_BASE]

#define GOP_SC_MIU_ARBITOR                                0x03

#endif //_MHAL_GFLIP_REG_H

