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
/// file    regSERFLASH.h
/// @brief  Serial Flash Register Definition
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _REG_SERFLASH_H_
#define _REG_SERFLASH_H_

//#include "mhal_chiptop_reg.h"

//-------------------------------------------------------------------------------------------------
//  Hardware Capability
//-------------------------------------------------------------------------------------------------

// !!! Uranus Serial Flash Notes: !!!
//  - The clock of DMA & Read via XIU operations must be < 3*CPU clock
//  - The clock of DMA & Read via XIU operations are determined by only REG_ISP_CLK_SRC; other operations by REG_ISP_CLK_SRC only
//  - DMA program can't run on DRAM, but in flash ONLY
//  - DMA from SPI to DRAM => size/DRAM start/DRAM end must be 8-B aligned


//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------

// BASEADDR & BK
#define BASEADDR_RIU                0xFD000000  // TODO: <-@@@ CHIP SPECIssFIC
#define BASEADDR_XIU                0x14000000  // FLASH0 // TODO: <-@@@ CHIP SPECIFIC
#define BASEADDR_NonPMBankRIU       0xFD200000
extern ptrdiff_t mstar_pm_base;
// #define BASEADDR_XIU                0xB8000000  // FLASH1 // TODO: use define instead of hard code e.g. CONFIG_SERFLASH1_SEL
#define BK_ISP                      0x1000
#define BK_FSP                      0x1200
#define BK_QSP                      0x1400
#define BK_PIU                      0x7800
#define BK_MHEG5                    0x1E00
#define BK_PMSLP                    0x1C00
#define BK_CLK0                     0x1600
#define BK_BDMA                     0x1200
#define BK_QSPI                     0x2E00
#define BK_PMGPIO                0x1E00

//----- Chip flash -------------------------
#define REG_SPI_BASE                0x7FUL

//Bit operation
#define BITS(_bits_, _val_)         ((BIT(((1)?_bits_)+1)-BIT(((0)?_bits_))) & (_val_<<((0)?_bits_)))
#define BMASK(_bits_)               (BIT(((1)?_bits_)+1)-BIT(((0)?_bits_)))

// ISP_CMD

#define REG_ISP_PASSWORD            0x00UL // ISP / XIU read / DMA mutual exclusive
#define REG_ISP_SPI_COMMAND         0x01UL
    // please refer to the serial flash datasheet
    #define ISP_SPI_CMD_READ                BITS(7:0, 0x03)
    #define ISP_SPI_CMD_FASTREAD            BITS(7:0, 0x0B)
    #define ISP_SPI_CMD_RDID                BITS(7:0, 0x9F)
    #define ISP_SPI_CMD_WREN                BITS(7:0, 0x06)
    #define ISP_SPI_CMD_WRDI                BITS(7:0, 0x04)
    #define ISP_SPI_CMD_SE                  BITS(7:0, 0x20)
    #define ISP_SPI_CMD_32BE                BITS(7:0, 0x52)
    #define ISP_SPI_CMD_64BE                BITS(7:0, 0xD8)
    #define ISP_SPI_CMD_CE                  BITS(7:0, 0xC7)
    #define ISP_SPI_CMD_PP                  BITS(7:0, 0x02)
    #define ISP_SPI_CMD_RDSR                BITS(7:0, 0x05)
    #define ISP_SPI_CMD_RDSR2               BITS(7:0, 0x35) // support for new WinBond Flash
    #define ISP_SPI_CMD_WRSR                BITS(7:0, 0x01)
    #define ISP_SPI_CMD_DP                  BITS(7:0, 0xB9)
    #define ISP_SPI_CMD_RDP                 BITS(7:0, 0xAB)
    #define ISP_SPI_CMD_RES                 BITS(7:0, 0xAB)
    #define ISP_SPI_CMD_REMS                BITS(7:0, 0x90)
    #define ISP_SPI_CMD_REMS4               BITS(7:0, 0xCF) // support for new MXIC Flash
    #define ISP_SPI_CMD_PARALLEL            BITS(7:0, 0x55)
    #define ISP_SPI_CMD_EN4K                BITS(7:0, 0xA5)
    #define ISP_SPI_CMD_EX4K                BITS(7:0, 0xB5)
    /* MXIC Individual Block Protection Mode */
    #define ISP_SPI_CMD_WPSEL               BITS(7:0, 0x68)
    #define ISP_SPI_CMD_SBLK                BITS(7:0, 0x36)
    #define ISP_SPI_CMD_SBULK               BITS(7:0, 0x39)
    #define ISP_SPI_CMD_RDSCUR              BITS(7:0, 0x2B)
    #define ISP_SPI_CMD_RDBLOCK             BITS(7:0, 0x3C)
    #define ISP_SPI_CMD_GBLK                BITS(7:0, 0x7E)
    #define ISP_SPI_CMD_GBULK               BITS(7:0, 0x98)
#define REG_ISP_SPI_ADDR_L          0x02UL // A[15:0]
#define REG_ISP_SPI_ADDR_H          0x03UL // A[23:16]
#define REG_ISP_SPI_WDATA           0x04UL
    #define ISP_SPI_WDATA_DUMMY     BITS(7:0, 0xFFUL)
#define REG_ISP_SPI_RDATA           0x05UL
#define REG_ISP_SPI_CLKDIV          0x06UL // clock = CPU clock / this div
    #define ISP_SPI_CLKDIV2         BIT(0)
    #define ISP_SPI_CLKDIV4         BIT(2)
    #define ISP_SPI_CLKDIV8         BIT(6)
    #define ISP_SPI_CLKDIV16        BIT(7)
    #define ISP_SPI_CLKDIV32        BIT(8)
    #define ISP_SPI_CLKDIV64        BIT(9)
    #define ISP_SPI_CLKDIV128       BIT(10)
#define REG_ISP_DEV_SEL             0x07UL
#define REG_ISP_SPI_CECLR           0x08UL
    #define ISP_SPI_CECLR                   BITS(0:0, 1)
#define REG_ISP_SPI_RDREQ           0x0CUL
    #define ISP_SPI_RDREQ                   BITS(0:0, 1)
#define REG_ISP_SPI_ENDIAN          0x0FUL
#define REG_ISP_SPI_RD_DATARDY      0x15UL
    #define ISP_SPI_RD_DATARDY_MASK     BMASK(0:0)
    #define ISP_SPI_RD_DATARDY              BITS(0:0, 1)
#define REG_ISP_SPI_WR_DATARDY      0x16UL
    #define ISP_SPI_WR_DATARDY_MASK     BMASK(0:0)
    #define ISP_SPI_WR_DATARDY              BITS(0:0, 1)
#define REG_ISP_SPI_WR_CMDRDY       0x17UL
    #define ISP_SPI_WR_CMDRDY_MASK      BMASK(0:0)
    #define ISP_SPI_WR_CMDRDY               BITS(0:0, 1)
#define REG_ISP_TRIGGER_MODE        0x2aUL
#define REG_ISP_CHIP_SEL            0x36UL
    #define SFSH_CHIP_SEL_MASK          BMASK(6:0)
    #define SFSH_CHIP_SEL_FLASH1            BIT(0)
    #define SFSH_CHIP_SEL_FLASH2            BIT(1)
    #define SFSH_CHIP_SEL_SPI_DEV1          BIT(2)
    #define SFSH_CHIP_SEL_SPI_DEV2          BIT(3)
    #define SFSH_CHIP_SEL_SPI_DEV3          BIT(4)
    #define SFSH_CHIP_SEL_SPI_DEV4          BIT(5)
    #define SFSH_CHIP_SEL_SPI_DEV5          BIT(6)
//    #define SFSH_CHIP_SEC_MASK          BMASK(7:0)          // 0x00FF // TODO: review this define
    #define SFSH_CHIP_SEL_MODE_SEL_MASK BMASK(7:7)
    #define SFSH_CHIP_SEL_RIU               BITS(7:7, 1)    // 0x0080
    #define SFSH_CHIP_SEL_XIU               BITS(7:7, 0)    // 0x0000
#define REG_ISP_CHIP_RST            0x3FUL // SPI clock source  [0]:gate  [1]:inv  [4:2]:clk_sel  000:Xtal clock 001:27MHz 010:36MHz 011:43.2MHz 100:54MHz 101:72MHz 110:86MHz 111:108MHz  [5]:0:xtal 1:clk_Sel
    #define SFSH_CHIP_RESET_MASK          BMASK(2:2)
    #define SFSH_CHIP_RESET                 BITS(2:2, 0)
    #define SFSH_CHIP_NOTRESET              BITS(2:2, 1)
    #define SFSH_CHIP_ARBITER_MASK          BMASK(1:1)
    #define SFSH_CHIP_ARBITER_RESET         BITS(1:1, 0)
    #define SFSH_CHIP_ARBITER_NOTRESET      BITS(1:1, 1)

#define REG_SPI_CS_TIME            0x71
     #define SFSH_CS_DESEL_MASK          BMASK(3:0)
    #define SFSH_CS_DESEL_TWO            BITS(3:0, 1)
    #define SFSH_CS_SETUP_MASK          BMASK(7:4)
    #define SFSH_CS_SETUP_TWO            BITS(7:4, 1)
    #define SFSH_CS_HOLD_MASK          BMASK(11:8)
    #define SFSH_CS_HOLD_TWO            BITS(11:8, 1)

#define REG_ISP_SPI_MODE            0x72
    #define SFSH_CHIP_FAST_MASK          BMASK(3:0) // SPI CMD [0x0B]
    #define SFSH_CHIP_FAST_ENABLE           BITS(3:0, 1)
    #define SFSH_CHIP_FAST_DISABLE          BITS(0:0, 0)
    #define SFSH_CHIP_2XREAD_MASK        BMASK(3:0) // SPI CMD [0xBB]
    #define SFSH_CHIP_2XREAD_ENABLE         BITS(3:0, 2)
    #define SFSH_CHIP_2XREAD_DISABLE        BITS(3:0, 0)
    #define SFSH_CHIP_4XREAD_MASK        BMASK(3:0) // SPI CMD [0xEB]
    #define SFSH_CHIP_4XREAD_ENABLE         BITS(3:0, 11)
    #define SFSH_CHIP_4XREAD_DISABLE        BITS(3:0, 0)
#define REG_ISP_SPI_CHIP_SELE       0x7A
    #define SFSH_CHIP_SELE_MASK          BMASK(1:0) // only for secure booting = 0;
    #define SFSH_CHIP_SELE_EXT1             BITS(1:0, 0)
    #define SFSH_CHIP_SELE_EXT2             BITS(1:0, 1)
    #define SFSH_CHIP_SELE_EXT3             BITS(1:0, 2)
#define REG_ISP_SPI_CHIP_SELE_BUSY  0x7B
    #define SFSH_CHIP_SELE_BUSY_MASK     BMASK(0:0)
    #define SFSH_CHIP_SELE_SWITCH           BITS(0:0, 1)
    #define SFSH_CHIP_SELE_DONE             BITS(0:0, 0)

// PIU_DMA

#define REG_PIU_DMA_STATUS          0x10 // [1]done [2]busy [8:15]state
    #define PIU_DMA_DONE_MASK           BMASK(0:0)
    #define PIU_DMA_DONE                    BITS(0:0, 1)
    #define PIU_DMA_BUSY_MASK           BMASK(1:1)
    #define PIU_DMA_BUSY                    BITS(1:1, 1)
    #define PIU_DMA_STATE_MASK          BMASK(15:8)
#define REG_PIU_SPI_CLK_SRC         0x26 // SPI clock source  [0]:gate  [1]:inv  [4:2]:clk_sel  000:Xtal clock 001:27MHz 010:36MHz 011:43.2MHz 100:54MHz 101:72MHz 110:86MHz 111:108MHz  [5]:0:xtal 1:clk_Sel
    #define PIU_SPI_RESET_MASK          BMASK(8:8)
    #define PIU_SPI_RESET                   BITS(8:8, 1)
    #define PIU_SPI_NOTRESET                BITS(8:8, 0)
    #define PSCS_DISABLE_MASK           BMASK(0:0)
    #define PSCS_INVERT_MASK            BMASK(1:1)
    #define PSCS_CLK_SEL_MASK           BMASK(4:2)
    #define PSCS_CLK_SEL_XTAL               BITS(4:2, 0)
    #define PSCS_CLK_SEL_27MHZ              BITS(4:2, 1)
    #define PSCS_CLK_SEL_36MHZ              BITS(4:2, 2)
    #define PSCS_CLK_SEL_43MHZ              BITS(4:2, 3)
    #define PSCS_CLK_SEL_54MHZ              BITS(4:2, 4)
    #define PSCS_CLK_SEL_72MHZ              BITS(4:2, 5)
    #define PSCS_CLK_SEL_86MHZ              BITS(4:2, 6)
    #define PSCS_CLK_SEL_108MHZ             BITS(4:2, 7)
    #define PSCS_CLK_SRC_SEL_MASK       BMASK(5:5)
    #define PSCS_CLK_SRC_SEL_XTAL           BITS(5:5, 0)
    #define PSCS_CLK_SRC_SEL_CLK            BITS(5:5, 1)
#define REG_PIU_DMA_SPISTART_L      0x70 // [15:0]
#define REG_PIU_DMA_SPISTART_H      0x71 // [23:16]
#define REG_PIU_DMA_DRAMSTART_L     0x72 // [15:0]  in unit of B; must be 8B aligned
#define REG_PIU_DMA_DRAMSTART_H     0x73 // [23:16]
#define REG_PIU_DMA_SIZE_L          0x74 // [15:0]  in unit of B; must be 8B aligned
#define REG_PIU_DMA_SIZE_H          0x75 // [23:16]
#define REG_PIU_DMA_CMD             0x76
    #define PIU_DMA_CMD_FIRE            0x0001
    #define PIU_DMA_CMD_LE              0x0000
    #define PIU_DMA_CMD_BE              0x0020

// Serial Flash Register // please refer to the serial flash datasheet
#define SF_SR_WIP_MASK                  BMASK(0:0)
#define SF_SR_WEL_MASK                  BMASK(1:1)
#define SF_SR_BP_MASK                   BMASK(5:2) // BMASK(4:2) is normal case but SERFLASH_TYPE_MX25L6405 use BMASK(5:2)
#define SF_SR_PROG_ERASE_ERR_MASK       BMASK(6:6)
#define SF_SR_SRWD_MASK                 BMASK(7:7)
    #define SF_SR_SRWD                      BITS(7:7, 1)
    #define SF_SR_QUALD                     BITS(6:6, 1)

// PM_SLEEP CMD.
#define REG_PM_CKG_SPI              0x20 // Ref spec. before using these setting.
    #define PM_SPI_CLK_SEL_MASK         BMASK(13:10)
    #define PM_SPI_CLK_XTALI                BITS(13:10, 0)
    #define PM_SPI_CLK_54MHZ                BITS(13:10, 1)
    #define PM_SPI_CLK_86MHZ                BITS(13:10, 2)
    #define PM_SPI_CLK_108MHZ               BITS(13:10, 3)
    #define PM_SPI_CLK_SWITCH_MASK      BMASK(14:14)
    #define PM_SPI_CLK_SWITCH_OFF           BITS(14:14, 0)
    #define PM_SPI_CLK_SWITCH_ON            BITS(14:14, 1)

#define REG_PM_SPI_WPN              0x01
    #define PM_SPI_WPN_SWITCH_MASK      BMASK(0:0)
    #define PM_SPI_WPN_SWITCH_OUT       BITS(0:0, 0)
    #define PM_SPI_WPN_SWITCH_IN            BITS(0:0, 1)
    #define PM_SPI_WPN_OUT_DAT_MASK     BMASK(1:1)
    #define PM_SPI_WPN_OUT_HIGH         BITS(1:1, 1)
    #define PM_SPI_WPN_OUT_LOW          BITS(1:1, 0)


#define REG_PM_CHK_51MODE           0x53
    #define PM_51_ON_SPI                BITS(0:0, 0x1)
    #define PM_51_ONT_ON_SPI            BITS(0:0, 0x0)
// For Power Consumption

#define REG_PM_GPIO_SPICZ_OEN       0x17
#define REG_PM_GPIO_SPICK_OEN       0x18
#define REG_PM_GPIO_SPIDI_OEN       0x19
#define REG_PM_GPIO_SPIDO_OEN       0x1A
#define REG_PM_SPI_IS_GPIO          0x35
    #define PM_SPI_GPIO_MASK            BMASK(3:0)
    #define PM_SPI_IS_GPIO                  BITS(3:0, 0xF)
    #define PM_SPI_NOT_GPIO                 BITS(3:0, 0x0)
#define PM_SPI_HOLD_GPIO_MASK            BMASK(14:14)
#define PM_SPI_HOLD_IS_GPIO                  BITS(14:14, 1)
#define PM_SPI_HOLD_NOT_GPIO                 BITS(14:14, 0)
#define PM_SPI_WP_GPIO_MASK            BMASK(7:7)
#define PM_SPI_WP_IS_GPIO                  BITS(7:7, 1)
#define PM_SPI_WP_NOT_GPIO                 BITS(7:7, 0)

#define REG_PM_SPI_PAD_SET         0x76
    #define PM_SPI_PAD_MASK                 BMASK(10:10)
    #define PM_SPI_PAD_EN                   BITS(10:10, 0x1)
    #define PM_SPI_PAD_DIS                  BITS(10:10, 0x0)

#define REG_PM_GPIO_WPN         0x01
#define REG_PM_GPIO_HOLD            0x02
#define PM_SPI_GPIO_OEN_MASK        BMASK(0:0)
#define PM_SPI_GPIO_OEN_EN      BITS(0:0, 0)
#define PM_SPI_GPIO_OEN_DIS     BITS(0:0, 1)
#define PM_SPI_GPIO_HIGH_MASK       BMASK(1:1)
#define PM_SPI_GPIO_HIGH            BITS(1:1, 1)

// CLK_GEN0
#define CLK0_CKG_SPI_MASK           BMASK(5:2)
#define CLK0_CKG_SPI_XTALI              BITS(5:2, 0)
#define CLK0_CKG_SPI_54MHZ              BITS(5:2, 1)
#define CLK0_CKG_SPI_86MHZ              BITS(5:2, 2)
#define CLK0_CKG_SPI_108MHZ             BITS(5:2, 3)
#define CLK0_CLK_SWITCH_MASK        BMASK(6:6)
#define CLK0_CLK_SWITCH_OFF           BITS(6:6, 0)
#define CLK0_CLK_SWITCH_ON            BITS(6:6, 1)

#define QUAD_EN                      BITS(6:6, 1)
#define QUAD_DIS                     BITS(6:6, 0)

// please refer to the serial flash datasheet
#define SPI_CMD_READ        (0x03)
#define SPI_CMD_FASTREAD    (0x0B)
#define SPI_CMD_RDID        (0x9F)
#define SPI_CMD_WREN        (0x06)
#define SPI_CMD_WRDI        (0x04)
#define SPI_CMD_SE          (0x20)
#define SPI_CMD_32BE        (0x52)
#define SPI_CMD_64BE        (0xD8)
#define SPI_CMD_CE          (0xC7)
#define SPI_CMD_PP          (0x02)
#define SPI_CMD_RDSR        (0x05)
#define SPI_CMD_RDSR2       (0x15)
// support for new WinBond Flash
#define SPI_CMD_WRSR        (0x01)
#define SPI_CMD_QPI         (0x35)
#define SPI_CMD_QPI_RST (0xF5)
#define SPI_WRIET_BUSY        (0x01)
#define SPI_CMD_WRSR_TWO        (0x31)
#define SPI_CMD_WRSR_THREE   (0x11)
  // FSP Register
#define REG_FSP_WDB0    0x60
#define REG_FSP_WDB0_MASK       BMASK(7:0)
#define REG_FSP_WDB0_DATA(d)    BITS(7:0, d)
#define REG_FSP_WDB1    0x60
#define REG_FSP_WDB1_MASK       BMASK(15:8)
#define REG_FSP_WDB1_DATA(d)    BITS(15:8, d)
#define REG_FSP_WDB2    0x61
#define REG_FSP_WDB2_MASK       BMASK(7:0)
#define REG_FSP_WDB2_DATA(d)    BITS(7:0, d)
#define REG_FSP_WDB3    0x61
#define REG_FSP_WDB3_MASK       BMASK(15:8)
#define REG_FSP_WDB3_DATA(d)    BITS(15:8, d)
#define REG_FSP_WDB4    0x62
#define REG_FSP_WDB4_MASK       BMASK(7:0)
#define REG_FSP_WDB4_DATA(d)    BITS(7:0, d)
#define REG_FSP_WDB5    0x62
#define REG_FSP_WDB5_MASK       BMASK(15:8)
#define REG_FSP_WDB5_DATA(d)    BITS(15:8, d)
#define REG_FSP_WDB6    0x63
#define REG_FSP_WDB6_MASK       BMASK(7:0)
#define REG_FSP_WDB6_DATA(d)    BITS(7:0, d)
#define REG_FSP_WDB7    0x63
#define REG_FSP_WDB7_MASK       BMASK(15:8)
#define REG_FSP_WDB7_DATA(d)    BITS(15:8, d)
#define REG_FSP_WDB8    0x64
#define REG_FSP_WDB8_MASK       BMASK(7:0)
#define REG_FSP_WDB8_DATA(d)    BITS(7:0, d)
#define REG_FSP_WDB9    0x64
#define REG_FSP_WDB9_MASK       BMASK(15:8)
#define REG_FSP_WDB9_DATA(d)    BITS(15:8, d)
#define REG_FSP_RDB0    0x65
#define REG_FSP_RDB1    0x65
#define REG_FSP_RDB2    0x66
#define REG_FSP_RDB3    0x66
#define REG_FSP_RDB4    0x67
#define REG_FSP_RDB5    0x67
#define REG_FSP_RDB6    0x68
#define REG_FSP_RDB7    0x68
#define REG_FSP_RDB8    0x69
#define REG_FSP_RDB9    0x69

#define FSP_OFFSET          0x60
#define REG_FSP_WD0         (FSP_OFFSET+0x00)
#define REG_FSP_WD0_MASK    BMASK(7:0)
    #define FSP_WD0(byte)   BITS(7:0,(byte))
#define REG_FSP_WD1         (FSP_OFFSET+0x00UL)
#define REG_FSP_WD1_MASK    BMASK(15:8)
    #define FSP_WD1(byte)   BITS(15:8,(byte))
#define REG_FSP_WD2         (FSP_OFFSET+0x01UL)
#define REG_FSP_WD2_MASK    BMASK(7:0)
    #define FSP_WD2(byte)   BITS(7:0,(byte))
#define REG_FSP_WD3         (FSP_OFFSET+0x01UL)
#define REG_FSP_WD3_MASK    BMASK(15:8)
    #define FSP_WD3(byte)   BITS(15:8,(byte))
#define REG_FSP_WD4         (FSP_OFFSET+0x02UL)
#define REG_FSP_WD4_MASK    BMASK(7:0)
    #define FSP_WD4(byte)   BITS(7:0,(byte))
#define REG_FSP_WD5         (FSP_OFFSET+0x02UL)
#define REG_FSP_WD5_MASK    BMASK(15:8)
    #define FSP_WD5(byte)       BITS(15:8,(byte))
#define REG_FSP_WD6         (FSP_OFFSET+0x03UL)
#define REG_FSP_WD6_MASK    BMASK(7:0)
    #define FSP_WD6(byte)   BITS(7:0,(byte))
#define REG_FSP_WD7         (FSP_OFFSET+0x03UL)
#define REG_FSP_WD7_MASK    BMASK(15:8)
    #define FSP_WD7(byte)   BITS(15:8,(byte))
#define REG_FSP_WD8         (FSP_OFFSET+0x04UL)
#define REG_FSP_WD8_MASK    BMASK(7:0)
    #define FSP_WD8(byte)   BITS(7:0,(byte))
#define REG_FSP_WD9         (FSP_OFFSET+0x04UL)
#define REG_FSP_WD9_MASK    BMASK(15:8)
    #define FSP_WD9(byte)   BITS(15:8,(byte))

#define REG_FSP_RD0         (FSP_OFFSET+0x05UL)
#define REG_FSP_RD0_MASK    BMASK(7:0)
    #define FSP_RD0(byte)   BITS(7:0,(byte))
#define REG_FSP_RD1         (FSP_OFFSET+0x05UL)
#define REG_FSP_RD1_MASK    BMASK(15:8)
    #define FSP_RD1(byte)   BITS(15:8,(byte))
#define REG_FSP_RD2         (FSP_OFFSET+0x06UL)
#define REG_FSP_RD2_MASK    BMASK(7:0)
    #define FSP_RD2(byte)   BITS(7:0,(byte))
#define REG_FSP_RD3         (FSP_OFFSET+0x06UL)
#define REG_FSP_RD3_MASK    BMASK(15:8)
    #define FSP_RD3(byte)   BITS(15:8,(byte))
#define REG_FSP_RD4         (FSP_OFFSET+0x07UL)
#define REG_FSP_RD4_MASK    BMASK(7:0)
    #define FSP_RD4(byte)   BITS(7:0,(byte))
#define REG_FSP_RD5         (FSP_OFFSET+0x07UL)
#define REG_FSP_RD5_MASK    BMASK(15:8)
    #define FSP_RD5(byte)   BITS(15:8,(byte))
#define REG_FSP_RD6         (FSP_OFFSET+0x08UL)
#define REG_FSP_RD6_MASK    BMASK(7:0)
    #define FSP_RD6(byte)   BITS(7:0,(byte))
#define REG_FSP_RD7         (FSP_OFFSET+0x08UL)
#define REG_FSP_RD7_MASK    BMASK(15:8)
    #define FSP_RD7(byte)   BITS(15:8,(byte))
#define REG_FSP_RD8         (FSP_OFFSET+0x09UL)
#define REG_FSP_RD8_MASK    BMASK(7:0)
    #define FSP_RD8(byte)   BITS(7:0,(byte))
#define REG_FSP_RD9         (FSP_OFFSET+0x09UL)
#define REG_FSP_RD9_MASK    BMASK(15:8)
    #define FSP_RD9(byte)   BITS(15:8,(byte))

#define REG_FSP_WBF_SIZE   0x6a
#define REG_FSP_WBF_SIZE0_MASK  BMASK(3:0)
#define REG_FSP_WBF_SIZE0(s)    BITS(3:0,s)
#define REG_FSP_WBF_SIZE1_MASK  BMASK(7:4)
#define REG_FSP_WBF_SIZE1(s)    BITS(7:4,s)
#define REG_FSP_WBF_SIZE2_MASK  BMASK(11:8)
#define REG_FSP_WBF_SIZE2(s)    BITS(11:8,s)
#define REG_FSP_RBF_SIZE   0x6b
#define REG_FSP_RBF_SIZE0_MASK  BMASK(3:0)
#define REG_FSP_RBF_SIZE0(s)    BITS(3:0,s)
#define REG_FSP_RBF_SIZE1_MASK  BMASK(7:4)
#define REG_FSP_RBF_SIZE1(s)    BITS(7:4,s)
#define REG_FSP_RBF_SIZE2_MASK  BMASK(11:8)
#define REG_FSP_RBF_SIZE2(s)    BITS(11:8,s)

#define REG_FSP_CTRL   0x6c
#define REG_FSP_ENABLE_MASK     BMASK(0:0)
#define REG_FSP_ENABLE          BITS(0:0,1)
#define REG_FSP_DISABLE         BITS(0:0,0)
#define REG_FSP_RESET_MASK      BMASK(1:1)
#define REG_FSP_RESET           BITS(1:1,0)
#define REG_FSP_NRESET          BITS(1:1,1)
#define REG_FSP_INT_MASK        BMASK(2:2)
#define REG_FSP_INT             BITS(2:2,1)
#define REG_FSP_INT_OFF         BITS(2:2,0)
#define REG_FSP_CHK_MASK        BMASK(3:3)
#define REG_FSP_CHK             BITS(3:3,1)
#define REG_FSP_CHK_OFF         BITS(3:3,0)
#define REG_FSP_STS_BUSY         BITS(10:8,0)
#define REG_FSP_STS_MASK         BMASK(10:8)
#define REG_FSP_RDSR_MASK       BMASK(12:11)
#define REG_FSP_1STCMD          BITS(12:11,0)
#define REG_FSP_2NDCMD          BITS(12:11,1)
#define REG_FSP_3THCMD          BITS(12:11,2)
#define REG_FSP_FSCHK_MASK        BMASK(13:13)
#define REG_FSP_FSCHK_ON          BITS(13:13,1)
#define REG_FSP_FSCHK_OFF         BITS(13:13,0)
#define REG_FSP_3THCMD_MASK        BMASK(14:14)
#define REG_FSP_3THCMD_ON          BITS(14:14,1)
#define REG_FSP_3THCMD_OFF         BITS(14:14,0)
#define REG_FSP_2NDCMD_MASK        BMASK(15:15)
#define REG_FSP_2NDCMD_ON          BITS(15:15,1)
#define REG_FSP_2NDCMD_OFF         BITS(15:15,0)
#define REG_FSP_TRIGGER   0x6d
#define REG_FSP_TRIGGER_MASK        BMASK(0:0)
#define REG_FSP_FIRE             BITS(0:0,1)
#define REG_FSP_DONE_FLAG   0x6e
#define REG_FSP_DONE_FLAG_MASK        BMASK(0:0)
#define REG_FSP_DONE             BITS(0:0,1)
#define REG_FSP_DONE_CLR   0x6f
#define REG_FSP_DONE_CLR_MASK        BMASK(0:0)
#define REG_FSP_CLR             BITS(0:0,1)
        // Serial Flash Register // please refer to the serial flash datasheet
#define SF_SR_WIP_MASK                  BMASK(0:0)
#define SF_SR_WEL_MASK                  BMASK(1:1)
#define SF_SR_BP_MASK                   BMASK(5:2)
        // BMASK(4:2) is normal case but SERFLASH_TYPE_MX25L6405 use BMASK(5:2)
#define SF_SR_PROG_ERASE_ERR_MASK       BMASK(6:6)
#define SF_SR_SRWD_MASK                 BMASK(7:7)
#define SF_SR_SRWD_DIS                      BITS(7:7, 0)
#define REG_FSP_WRITEDATA_SIZE     0x4
#define REG_FSP_MAX_WRITEDATA_SIZE 0xA
#define REG_FSP_READDATA_SIZE      0xA
#define FLASH_PAGE_SIZE            256
#define FLASH_OIP                  1
//QSPI
#define REG_SPI_BURST_WRITE 0x0A
#define REG_SPI_DISABLE_BURST 0x03
#define REG_SPI_ENABLE_BURST  0x01
#define REG_SPI_TO_HARDWARE  0x02

//FSP_CTRL0
#define FSP_ENABLE          BIT(0)
#define FSP_NOT_RESET       BIT(1)
#define FSP_INT_ENABLE      BIT(2)
#define FSP_AUTOCHK_ENABLE  BIT(3)
//FSP_CTRL1
#define FSP_RDSR_FIRST                 0x00UL
#define FSP_RDSR_SECOND                0x01UL
#define FSP_RDSR_THIRD                 0x10UL
#define FSP_FLASH_STATUS_AUTO_CHK      BIT(5)
#define FSP_THIRD_CMD_ENABLE           BIT(6)
#define FSP_SECOND_CMD_ENABLE          BIT(7)
//FSP_TRIGGER
#define FSP_TRIGGER_ENABLE          BIT(0)
//FSP_CLEAR
#define FSP_CLEAR_DONE_FLAG         BIT(0)
#define CLK_REG_MCU_CLK_SEL         0x10
#define REG_CLK0_CKG_SPI            0x16UL
#endif // _REG_SERFLASH_H_
