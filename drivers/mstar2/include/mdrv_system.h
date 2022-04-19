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
/// @file   drvSystem.h
/// @brief  System Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////
#include "mdrv_system_st.h"
#include "mst_platform.h"

#ifndef _DRV_SYSTEM_H_
#define _DRV_SYSTEM_H_

#if defined(CONFIG_MSTAR_KENYA) || defined(CONFIG_MSTAR_AMBER1) || defined(CONFIG_MSTAR_KERES)
   #define DEFINE_IN_MSTAR_CHIP_H 1
   #include <include/mstar/mstar_chip.h>
#endif

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define SYS_BOARD_NAME_MAX          32                                  ///< Maximum length of board name
#define SYS_PLATFORM_NAME_MAX       32                                  ///< Maximum length of playform name

#if 1
#define REG_SW_RESET_CPU_AEON                   0x1086
    //---------------------------------------------
    // definition for REG_SW_RESET_CPU_AEON   //reg[0x1086]
    #define AEON_SW_RESET                           BIT0

#define AEON_SPI_ADDR0                          0x0FFE

#if defined(CONFIG_MSTAR_TITANIA2)
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF800000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x1E26
#define AEON_CLK_ENABLE                         0x00
#define AEON_CLK_DISABLE                        0x40
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0006
    #define MIU_PROTECT_4                       0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x2B40
#define REG_AEON_C_FIQ_MASK_H                   0x2B42
#define REG_AEON_C_IRQ_MASK_L                   0x2B58
#define REG_AEON_C_IRQ_MASK_H                   0x2B5A

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x0010

#define MIPS_MIU0_BUS_BASE                      0x00000000

#elif defined(CONFIG_MSTAR_EUCLID)
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x1E24
#define AEON_CLK_ENABLE                         0x0000
#define AEON_CLK_DISABLE                        0x4000
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0006
    #define MIU_PROTECT_4                       0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x11940
#define REG_AEON_C_FIQ_MASK_H                   0x11942
#define REG_AEON_C_IRQ_MASK_L                   0x11958
#define REG_AEON_C_IRQ_MASK_H                   0x1195A

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x0030

#define MIPS_MIU0_BUS_BASE                      0x00000000

#elif defined(CONFIG_MSTAR_TITANIA3) || defined(CONFIG_MSTAR_TITANIA10)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x6948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x0010

#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x40000000

#elif defined(CONFIG_MSTAR_URANUS4)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1E0C
	#define REG_CHIP_NAND_MODE_MASK				0x000F

#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x40000000

#elif defined(CONFIG_MSTAR_TITANIA4)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x0010

#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x40000000

#elif defined(CONFIG_MSTAR_TITANIA8) || \
	  defined(CONFIG_MSTAR_TITANIA12) || \
      defined(CONFIG_MSTAR_AMBER2)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0040  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0080

#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x60000000

#elif defined(CONFIG_MSTAR_AMBER5)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0040

#define REG_SD_CONFIG2                          0x1EB4
    #define REG_SD_CONFIG2_MASK                 0x0020 //reg_sd_config & reg_sd_config2
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030 //AD pads & Ctrl pads enable
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00F0

#define MIPS_MIU0_BUS_BASE                      0x40000000
#define MIPS_MIU1_BUS_BASE                      0x60000000

#elif defined(CONFIG_MSTAR_TITANIA9) || \
      defined(CONFIG_MSTAR_TITANIA13) || \
      defined(CONFIG_MSTAR_AMBER6) || \
      defined(CONFIG_MSTAR_AMBER7) || \
      defined(CONFIG_MSTAR_AMETHYST) || \
      defined(CONFIG_MSTAR_EMERALD) || \
      defined(CONFIG_MSTAR_NUGGET) || \
	  defined(CONFIG_MSTAR_NIKON) || \
	  defined(CONFIG_MSTAR_MILAN)
#define REG_ADDR(addr)                          (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
 #define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

    #if (defined(CONFIG_MSTAR_TITANIA9) || defined(CONFIG_MSTAR_TITANIA13))
#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x0010
	#elif defined(CONFIG_MSTAR_EMERALD)
    #define REG_CHIP_NAND_MODE						0x1EA0
    	#define REG_CHIP_NAND_MODE_MASK				0x0300
    	#define REG_CHIP_NAND_MODE_PCMA				0x0200
    	#define REG_CHIP_NAND_MODE_PCMD				0x0100
    #define REG_CHIP_PCMCFG                         0x1E9E
        #define REG_CHIP_PCMCFG_MASK                0xC000
        #define REG_CHIP_PCMCFG_CTRL_EN             0xC000 //AD pads & Ctrl pads enable
    #define REG_SD_CONFIG2                          0x1EAE
        #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2

    #define REG_CHIP_PCM_PE                         0x1E16
        #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
        #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
    #define REG_CHIP_PCM_PE1                        0x1E18
        #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
    #define REG_CHIP_PCM_D_PE                       0x1E1A
        #define REG_CHIP_PCM_D_PE_MASK              0x0003

	#elif defined(CONFIG_MSTAR_NUGGET)
    #define REG_CHIP_NAND_MODE						0x1EA0
    	#define REG_CHIP_NAND_MODE_MASK				0x0380
//    	#define REG_CHIP_NAND_MODE_PCMA				0x0200
//    	#define REG_CHIP_NAND_MODE_PCMD				0x0100
    #define REG_CHIP_PCMCFG                         0x1E9E
        #define REG_CHIP_PCMCFG_MASK                0xC000
        #define REG_CHIP_PCMCFG_CTRL_EN             0xC000 //AD pads & Ctrl pads enable
    #define REG_SD_CONFIG2                          0x1EAE
        #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2

    #define REG_CHIP_PCM_PE                         0x1E16
        #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
        #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
    #define REG_CHIP_PCM_PE1                        0x1E18
        #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
    #define REG_CHIP_PCM_D_PE                       0x1E1A
        #define REG_CHIP_PCM_D_PE_MASK              0x0003

	#elif defined(CONFIG_MSTAR_NIKON)
    #define REG_CHIP_NAND_MODE						0x1EA0
    	#define REG_CHIP_NAND_MODE_MASK				0x0380
//    	#define REG_CHIP_NAND_MODE_PCMA				0x0200
//    	#define REG_CHIP_NAND_MODE_PCMD				0x0100
    #define REG_CHIP_PCMCFG                         0x1E9E
        #define REG_CHIP_PCMCFG_MASK                0xC000
        #define REG_CHIP_PCMCFG_CTRL_EN             0xC000 //AD pads & Ctrl pads enable
    #define REG_SD_CONFIG2                          0x1EAE
        #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2

    #define REG_CHIP_PCM_PE                         0x1E16
        #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
        #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
    #define REG_CHIP_PCM_PE1                        0x1E18
        #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
    #define REG_CHIP_PCM_D_PE                       0x1E1A
        #define REG_CHIP_PCM_D_PE_MASK              0x0003

	#elif defined(CONFIG_MSTAR_MILAN)
    #define REG_CHIP_NAND_MODE						0x1EA0
    	#define REG_CHIP_NAND_MODE_MASK				0x0380
//    	#define REG_CHIP_NAND_MODE_PCMA				0x0200
//    	#define REG_CHIP_NAND_MODE_PCMD				0x0100
    #define REG_CHIP_PCMCFG                         0x1E9E
        #define REG_CHIP_PCMCFG_MASK                0xC000
        #define REG_CHIP_PCMCFG_CTRL_EN             0xC000 //AD pads & Ctrl pads enable
    #define REG_SD_CONFIG2                          0x1EAE
        #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2

    #define REG_CHIP_PCM_PE                         0x1E16
        #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
        #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
    #define REG_CHIP_PCM_PE1                        0x1E18
        #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
    #define REG_CHIP_PCM_D_PE                       0x1E1A
        #define REG_CHIP_PCM_D_PE_MASK              0x0003

    #else
#define REG_CHIP_NAND_MODE						0x1EA0
	#define REG_CHIP_NAND_MODE_MASK				0x0300
    #define REG_CHIP_NAND_MODE_PCMA				0x0200
	#define REG_CHIP_NAND_MODE_PCMD				0x0100
    #define REG_CHIP_PF_MODE			        0x1EDE
    #define REG_CHIP_PF_MODE_MASK				0x0010
    #endif

    #define MIPS_MIU0_BUS_BASE                      0x00000000
    #define MIPS_MIU1_BUS_BASE                      0x60000000

#elif defined(CONFIG_MSTAR_TITANIA11)
#define REG_ADDR(addr)                          (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0040
	#define REG_CHIP_NAND_MODE_PCMD				0x0080


#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x40000000
#elif defined(CONFIG_MSTAR_JANUS2)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)              (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0


#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x60000000

#elif defined(CONFIG_MSTAR_KRONUS)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                          (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1E0C
	#define REG_CHIP_NAND_MODE_MASK				0x000F


#define MIPS_MIU0_BUS_BASE                      0x00000000
#define MIPS_MIU1_BUS_BASE                      0x40000000

#elif defined(CONFIG_MSTAR_KAISERIN) || defined(CONFIG_MSTAR_KAPPA) || defined(CONFIG_MSTAR_KELTIC)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                          (*((volatile U16*)(0xBF200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x0B24
#define AEON_CLK_ENABLE                         0x0084 //CLK 172.8MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x12C0
#define MIU_PROTECT3_ID0                        0x12D6
#define MIU_PROTECT3_START_ADDR_H               0x12D8
#define MIU_PROTECT3_END_ADDR_H                 0x12DA
    #define MIU_CLI_AEON_RW                     0x0005
    #define MIU_PROTECT_4                       0x0008
#define MIU_PROTECT_4                           0x0008
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
    #define STOP_AEON                           0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1948
#define REG_AEON_C_FIQ_MASK_H                   0x194A
#define REG_AEON_C_IRQ_MASK_L                   0x1968
#define REG_AEON_C_IRQ_MASK_H                   0x196A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE			0x1E0C
#define REG_CHIP_NAND_MODE_MASK			0x000F

#if defined(CONFIG_MSTAR_KAISERIN) || defined(CONFIG_MSTAR_KAPPA) || defined(CONFIG_MSTAR_KELTIC)
   #include <include/mstar/mstar_chip.h>
   #define MIPS_MIU0_BUS_BASE                   MSTAR_MIU0_BUS_BASE
   #define MIPS_MIU1_BUS_BASE                   MSTAR_MIU1_BUS_BASE
#else
   #define MIPS_MIU0_BUS_BASE                   0x00000000
   #define MIPS_MIU1_BUS_BASE                   0x60000000
#endif

#elif defined(CONFIG_MSTAR_AMBER3)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030 //AD pads & Ctrl pads enable
#define REG_SD_CONFIG                           0x1EB4
    #define REG_SD_CONFIG_MASK                  0x0020
#define REG_SD_CONFIG2                          0x1E38 //U02 added
    #define REG_SD_CONFIG2_MASK                 0x1000
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00F0

#define ARM_MIU0_BUS_BASE                      0x40000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x60000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_EAGLE)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1E16
	#define REG_CHIP_NAND_MODE_MASK				0x7000
	#define REG_CHIP_NAND_MODE_PCMA				0x2000
	#define REG_CHIP_NAND_MODE_PCMD				0x1000

#define REG_CHIP_PCMCFG                         0x1E9E
    #define REG_CHIP_PCMCFG_MASK                0xC000
    #define REG_CHIP_PCMCFG_CTRL_EN             0xC000 //AD pads & Ctrl pads enable

#define REG_EMMC_CONFIG                         0x1EBA
    #define REG_EMMC_CONFIG_MASK                0xC000

#define REG_CHIP_PCM_PE                         0x1E82
    #define REG_CHIP_PCM_PE_MASK                0xFFFF
#define REG_CHIP_PCM_PE1                        0x1E84
    #define REG_CHIP_PCM_PE1_MASK               0x03FF
#define REG_CHIP_PCM_D_PE                       0x1E18
    #define REG_CHIP_PCM_D_PE_MASK              0x00FF

#define REG_CHIP_PCM_NAND_BYPASS                0x1E86
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0002
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0002
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define ARM_MIU0_BUS_BASE                      0x40000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_AGATE)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030 //AD pads & Ctrl pads enable
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00F0

#define REG_CHIP_PCM_PE                         0x1E1C
    #define REG_CHIP_PCM_PE_MASK                0xFFFF
#define REG_CHIP_PCM_PE1                        0x1E1E
    #define REG_CHIP_PCM_PE1_MASK               0x03FF
#define REG_CHIP_PCM_D_PE                       0x1E1A
    #define REG_CHIP_PCM_D_PE_MASK              0x00FF

#define REG_CHIP_PCM_NAND_BYPASS                0x1E20
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define ARM_MIU0_BUS_BASE                      0x40000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_EDISON)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030 //AD pads & Ctrl pads enable

#define REG_SD_CONFIG2                          0x1EB4
    #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00C0

#define REG_CHIP_PCM_NAND_BYPASS                0x1E20
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define REG_CHIP_PCM_PE                         0x1E12
    #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
    #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
#define REG_CHIP_PCM_PE1                        0x1E14
    #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
#define REG_CHIP_PCM_D_PE                       0x1E16
    #define REG_CHIP_PCM_D_PE_MASK              0x0003

#define ARM_MIU0_BUS_BASE                      0x40000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_EINSTEIN) || defined(CONFIG_MSTAR_NAPOLI) || defined(CONFIG_MSTAR_MUNICH)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030 //AD pads & Ctrl pads enable

#define REG_SD_CONFIG2                          0x1EB4
    #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00C0

#define REG_CHIP_PCM_NAND_BYPASS                0x1E20
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define REG_CHIP_PCM_PE                         0x1E12
    #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
    #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
#define REG_CHIP_PCM_PE1                        0x1E14
    #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
#define REG_CHIP_PCM_D_PE                       0x1E16
    #define REG_CHIP_PCM_D_PE_MASK              0x0003

#define ARM_MIU0_BUS_BASE                      0x20000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_EINSTEIN3) || \
      defined(CONFIG_MSTAR_MONACO)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080  //Board 116A
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030 //AD pads & Ctrl pads enable

#define REG_SD_CONFIG2                          0x1EB4
    #define REG_SD_CONFIG2_MASK                 0x0F00 //reg_sd_config & reg_sd_config2
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00C0

#define REG_CHIP_PCM_NAND_BYPASS                0x1E20
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define REG_CHIP_PCM_PE                         0x1E12
    #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
    #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
#define REG_CHIP_PCM_PE1                        0x1E14
    #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
#define REG_CHIP_PCM_D_PE                       0x1E16
    #define REG_CHIP_PCM_D_PE_MASK              0x0003

#define ARM_MIU0_BUS_BASE                      0x20000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU2_BUS_BASE                      0xE0000000
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF

#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000
#define ARM_MIU2_BASE_ADDR                     0xc0000000
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_MUJI)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_KANO)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_CURRY)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_C2P)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_K6)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_K6Lite)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_K7U)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MASERATI)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MAXIM)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7621)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7622)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M5621)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7821)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MAINZ)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MONET)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MOONEY)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7221)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7322)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7332)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7632)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M7642)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MT5862)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MT5867)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MT5889)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MT5872)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_M3822)
        #include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MANHATTAN)
	#include <include/mstar/mstar_chip.h>

#elif defined(CONFIG_MSTAR_MESSI)
#define ARM_MIU0_BUS_BASE                      0x20000000UL
#define ARM_MIU1_BUS_BASE                      0xA0000000UL
#define ARM_MIU2_BUS_BASE                      0xFFFFFFFFUL
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFFUL

#define ARM_MIU0_BASE_ADDR                     0x00000000UL
#define ARM_MIU1_BASE_ADDR                     0x80000000UL
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFFUL
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFFUL

#elif defined(CONFIG_MSTAR_MUSTANG)
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFFUL

#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFFUL
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFFUL
//----------------------------------------------------

#define MSTAR_MIU0_BUS_BASE                      0x20000000
#define MSTAR_MIU1_BUS_BASE                      0xA0000000
#define MSTAR_MIU2_BUS_BASE                      0xE0000000


#define ARM_MIU0_BUS_BASE                        MSTAR_MIU0_BUS_BASE
#define ARM_MIU1_BUS_BASE                        MSTAR_MIU1_BUS_BASE
#define ARM_MIU2_BUS_BASE                      0xFFFFFFFFUL

#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFFUL


#elif defined(CONFIG_MSTAR_EIFFEL)
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x01C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x0030
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0030

#define REG_SD_CONFIG2                          0x1EB4
    #define REG_SD_CONFIG2_MASK                 0x0F00
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00C0

#define REG_CHIP_PCM_NAND_BYPASS                0x1E20
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define REG_CHIP_PCM_PE                         0x1E12
    #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
    #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
#define REG_CHIP_PCM_PE1                        0x1E14
    #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
#define REG_CHIP_PCM_D_PE                       0x1E16
    #define REG_CHIP_PCM_D_PE_MASK              0x0003

#define ARM_MIU0_BUS_BASE                       0x20000000
#define ARM_MIU1_BUS_BASE                       0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#elif defined(CONFIG_MSTAR_NIKE) || \
      defined(CONFIG_MSTAR_CLIPPERS) || \
      defined(CONFIG_MSTAR_MESSI) || \
      defined(CONFIG_MSTAR_MADISON)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1E16
	#define REG_CHIP_NAND_MODE_MASK				0x7000
	#define REG_CHIP_NAND_MODE_PCMA				0x2000
	#define REG_CHIP_NAND_MODE_PCMD				0x1000
#define REG_CHIP_PCMCFG                         0x1E9E
    #define REG_CHIP_PCMCFG_MASK                0xC000
    #define REG_CHIP_PCMCFG_CTRL_EN             0xC000 //AD pads & Ctrl pads enable

#define REG_SD_CONFIG2                          0x1EB6
    #define REG_SD_CONFIG2_MASK                 0x0F00
#define REG_EMMC_CONFIG                         0x1EBA
    #define REG_EMMC_CONFIG_MASK                0xC000

#define REG_CHIP_PCM_PE                         0x1E82
    #define REG_CHIP_PCM_PE_MASK                0xFFFF
#define REG_CHIP_PCM_PE1                        0x1E84
    #define REG_CHIP_PCM_PE1_MASK               0x03FF
#define REG_CHIP_PCM_D_PE                       0x1E18 // PAD_PCM_A[7:0] PE Control
    #define REG_CHIP_PCM_D_PE_MASK              0x00FF

#define REG_CHIP_PCM_NAND_BYPASS                0x1E86
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0002
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0002
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define ARM_MIU0_BUS_BASE                       0x20000000
#define ARM_MIU1_BUS_BASE                       0xA0000000
#define ARM_MIU0_BASE_ADDR                      0x00000000
#define ARM_MIU1_BASE_ADDR                      0x80000000

#define ARM_MIU2_BUS_BASE                       0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                       0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                      0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                      0xFFFFFFFF

#elif defined(CONFIG_MSTAR_MIAMI)
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_NAND_MODE						0x1EDE
	#define REG_CHIP_NAND_MODE_MASK				0x00C0
	#define REG_CHIP_NAND_MODE_PCMA				0x0080
	#define REG_CHIP_NAND_MODE_PCMD				0x0040
#define REG_CHIP_PCMCFG                         0x1EC8
    #define REG_CHIP_PCMCFG_MASK                0x00F0
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0050 //AD pads & Ctrl pads enable

#define REG_SD_CONFIG2                          0x1EB4
    #define REG_SD_CONFIG2_MASK                 0x0F00
#define REG_EMMC_CONFIG                         0x1EDC
    #define REG_EMMC_CONFIG_MASK                0x00C0

#define REG_CHIP_PCM_PE                         0x1E12
    #define REG_CHIP_PCM_PE_MASK                0x00FF // PCM_D
    #define REG_CHIP_PCM_CTL_MASK               0xFF00 // PCM_CTRL (except PCM_CD, PCM_RESET)
#define REG_CHIP_PCM_PE1                        0x1E14
    #define REG_CHIP_PCM_PE1_MASK               0x00FF // PCM_A
#define REG_CHIP_PCM_D_PE                       0x1E16
    #define REG_CHIP_PCM_D_PE_MASK              0x0003

#define REG_CHIP_PCM_NAND_BYPASS                0x1E20
    #define REG_CHIP_PCM_NAND_BYPASS_MASK       0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_ENABLE     0x0100
    #define REG_CHIP_PCM_NAND_BYPASS_DISABLE    0x0000

#define ARM_MIU0_BUS_BASE                       0x20000000
#define ARM_MIU1_BUS_BASE                       0xA0000000
#define ARM_MIU0_BASE_ADDR                      0x00000000
#define ARM_MIU1_BASE_ADDR                      0x80000000

#define ARM_MIU2_BUS_BASE                       0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                       0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                      0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                      0xFFFFFFFF

#elif defined(CONFIG_MSTAR_KAISER) || \
      defined(CONFIG_MSTAR_KAISERS)
//Fix me!!Needs to be reivewed!!
#define REG_ADDR(addr)                         (*((volatile U16*)(0xFD200000 + (addr << 1))))
#define REG_CKG_AEONTS0                         0x3360
#define AEON_CLK_ENABLE                         0x0000 //CLK 216MHz
#define AEON_CLK_DISABLE                        0x0001
#define MIU_PROTECT_EN                          0x1230
#define MIU_PROTECT3_ID0                        0x1232
#define MIU_PROTECT3_START_ADDR_H               0x1234
#define MIU_PROTECT3_END_ADDR_H                 0x1236
#define MIU_CLI_AEON_RW                         0x0004
#define MIU_PROTECT_4                           0x0001
#define MBX_AEON_JUDGEMENT                      0x33DE
#define MHEG5_CPU_STOP                          0x0FE6
#define STOP_AEON                               0x0001
#define MHEG5_REG_IOWINORDER                    0x0F80
#define REG_AEON_C_FIQ_MASK_L                   0x1148
#define REG_AEON_C_FIQ_MASK_H                   0x114A
#define REG_AEON_C_IRQ_MASK_L                   0x1168
#define REG_AEON_C_IRQ_MASK_H                   0x116A
#define REG_MAU0_MIU0_SIZE                      0x1842

#define REG_CHIP_PCMCFG                         0x1E0E
    #define REG_CHIP_PCMCFG_MASK                0x0003
    #define REG_CHIP_PCMCFG_CTRL_EN             0x0001 //AD pads & Ctrl pads enable

#define REG_SD_CONFIG2                          0x1E0C
    #define REG_SD_CONFIG2_MASK                 0xC000 //reg_sd_mode

#define ARM_MIU0_BUS_BASE                      0x20000000
#define ARM_MIU1_BUS_BASE                      0xA0000000
#define ARM_MIU0_BASE_ADDR                     0x00000000
#define ARM_MIU1_BASE_ADDR                     0x80000000

#define ARM_MIU2_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU3_BUS_BASE                      0xFFFFFFFF
#define ARM_MIU2_BASE_ADDR                     0xFFFFFFFF
#define ARM_MIU3_BASE_ADDR                     0xFFFFFFFF

#endif

#if defined(CONFIG_MSTAR_EUCLID) || \
    defined(CONFIG_MSTAR_URANUS4) || \
    defined(CONFIG_MSTAR_TITANIA2) || \
    defined(CONFIG_MSTAR_TITANIA4) || \
    defined(CONFIG_MSTAR_KRONUS) || \
    defined(CONFIG_MSTAR_KAISERIN) || \
    defined(CONFIG_MSTAR_KAPPA) || \
    defined(CONFIG_MSTAR_KELTIC) || \
    defined(CONFIG_MSTAR_TITANIA11) || \
    defined(CONFIG_MSTAR_AMBER2)

    #define MS_MIU_INTERVAL 0x08000000

#elif defined(CONFIG_MSTAR_TITANIA3) || defined(CONFIG_MSTAR_TITANIA10)

    #define MS_MIU_INTERVAL 0x10000000

#elif defined(CONFIG_MSTAR_TITANIA8) || \
       defined(CONFIG_MSTAR_TITANIA12) || \
       defined(CONFIG_MSTAR_TITANIA9) || \
       defined(CONFIG_MSTAR_TITANIA13) || \
       defined(CONFIG_MSTAR_AMBER5) || \
       defined(CONFIG_MSTAR_AMBER6) || \
       defined(CONFIG_MSTAR_AMBER7) || \
       defined(CONFIG_MSTAR_AMETHYST) || \
       defined(CONFIG_MSTAR_JANUS2) || \
       defined(CONFIG_MSTAR_EMERALD) || \
       defined(CONFIG_MSTAR_NUGGET) || \
	   defined(CONFIG_MSTAR_NIKON) || \
	   defined(CONFIG_MSTAR_MILAN)

    #define MS_MIU_INTERVAL 0x20000000

#elif defined(CONFIG_MSTAR_AMBER3) || \
      defined(CONFIG_MSTAR_EAGLE) || \
      defined(CONFIG_MSTAR_AGATE) || \
      defined(CONFIG_MSTAR_EDISON) || \
      defined(CONFIG_MSTAR_EINSTEIN) || \
      defined(CONFIG_MSTAR_NAPOLI) || \
      defined(CONFIG_MSTAR_EINSTEIN3) || \
      defined(CONFIG_MSTAR_EIFFEL) || \
      defined(CONFIG_MSTAR_NIKE) || \
      defined(CONFIG_MSTAR_KAPPA) || \
      defined(CONFIG_MSTAR_KELTIC) || \
      defined(CONFIG_MSTAR_KAISER) || \
	  defined(CONFIG_MSTAR_KAISERS)

    #define MS_MIU_INTERVAL 0x60000000

#elif defined(CONFIG_MSTAR_MADISON) || \
      defined(CONFIG_MSTAR_MESSI) || \
      defined(CONFIG_MSTAR_CLIPPERS) || \
      defined(CONFIG_MSTAR_MUNICH) || \
      defined(CONFIG_MSTAR_MIAMI) || \
      defined(CONFIG_MSTAR_MONACO) || \
      defined(CONFIG_MSTAR_MUJI) || \
      defined(CONFIG_MSTAR_KANO) || \
      defined(CONFIG_MSTAR_CURRY) || \
      defined(CONFIG_MSTAR_C2P) || \
      defined(CONFIG_MSTAR_K6) || \
      defined(CONFIG_MSTAR_K6Lite) || \
      defined(CONFIG_MSTAR_K7U) || \
      defined(CONFIG_MSTAR_MASERATI) || \
      defined(CONFIG_MSTAR_MAXIM) || \
      defined(CONFIG_MSTAR_M7621) || \
      defined(CONFIG_MSTAR_M7622) || \
      defined(CONFIG_MSTAR_M5621) || \
      defined(CONFIG_MSTAR_M7821) || \
      defined(CONFIG_MSTAR_MAINZ) || \
      defined(CONFIG_MSTAR_MONET) || \
      defined(CONFIG_MSTAR_MUSTANG) || \
      defined(CONFIG_MSTAR_MOONEY) || \
      defined(CONFIG_MSTAR_M7221) || \
      defined(CONFIG_MSTAR_M7322) || \
      defined(CONFIG_MSTAR_M7632) || \
      defined(CONFIG_MSTAR_M7332) || \
      defined(CONFIG_MSTAR_M7642) || \
      defined(CONFIG_MSTAR_MT5862) || \
      defined(CONFIG_MSTAR_MT5867) || \
      defined(CONFIG_MSTAR_MT5889) || \
      defined(CONFIG_MSTAR_MT5872) || \
      defined(CONFIG_MSTAR_M3822)

    #define MS_MIU_INTERVAL 0x80000000

#elif defined(CONFIG_MSTAR_MANHATTAN)
    #define MS_MIU_INTERVAL 0x80000000

#elif defined(DEFINE_IN_MSTAR_CHIP_H)

#else
     #error unknown chip!!
#endif

#define AEON_REG_CTRL                           0x0FF0

    //---------------------------------------------
    // definition for AEON_REG_CTRL   //reg[0x0FF0]
    #define AEON_CTRL_EN                            BIT0
    #define AEON_CTRL_RST                           BIT1
    #define AEON_DWB_SWAP                           BIT3

#define REG_PCMCIA_PCM_MEM_IO_CMD           0x00
#define REG_PCMCIA_ADDR0                    0x02
#define REG_PCMCIA_ADDR1                    0x03
#define REG_PCMCIA_WRITE_DATA               0x04
#define REG_PCMCIA_FIRE_READ_DATA_CLEAR     0x06
#define REG_PCMCIA_READ_DATA                0x08
#define REG_PCMCIA_READ_DATA_DONE_BUS_IDLE  0x09
#define REG_PCMCIA_INT_MASK_CLEAR           0x0A
#define REG_PCMCIA_INT_MASK_CLEAR1          0x0B
#define REG_PCMCIA_STAT_INT_RAW_INT         0x0E
#define REG_PCMCIA_STAT_INT_RAW_INT1        0x0F
#define REG_PCMCIA_MODULE_VCC_OOB           0x10
#define PCMCIA_ATTRIBMEMORY_READ            0x03
#define PCMCIA_ATTRIBMEMORY_WRITE           0x04
#define PCMCIA_IO_READ                      0x05
#define PCMCIA_IO_WRITE                     0x06
#if defined(CONFIG_MSTAR_M7322)
    #define REG_PCMCIA_BASE                     0x60940//note: pcm bank 0x1609 only support M7322
#else
    #define REG_PCMCIA_BASE                     0x3440
#endif
#define PCMCIA_FIRE_COMMAND         BIT0
#define PCMCIA_CLEAN_STATE_RD_DONE  BIT1
#define PCMCIA_STATE_RD_DONE        BIT0
#define PCMCIA_STATE_BUS_IDLE       BIT1
#define PCMCIA_STATE_WD_DONE        BIT4
#define PCMCIA_MAX_DETECT_COUNT     1
#define PCMCIA_MAX_POLLING_COUNT    20000
#define MAX_PCMCIA_CONFIGS              6       //!< The maximum number of configurations supported by a PCMCIA card
#define MAX_PCMCIA_STRLEN               (20)    //!< The maximum name of vendor/manufacturer/info strings
#define MAX_CIS_SIZE                    0x100   //!< The maximum size of a CIS, that is understood by this driver
#define PCMCIA_HW_TIMEOUT               3000
#define PCMCIA_HW_MAX_RETRY_COUNT       1000      //  PCMCIA hardware register maximum access times
#define PCMCIA_IRQ_ENABLE               1       // FIXME_ALEC

#endif

//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

/// System output pad switch type
typedef enum
{
    E_SYS_PAD_MSD5010_SM2_IIC2,                                         ///< 5010 SM2, IIC2
    E_SYS_PAD_MSD5011_SM2_IIC2,                                         ///< 5011 SM2, IIC2
    E_SYS_PAD_MSD5015_GPIO,                                             ///< 5015 GPIO
    E_SYS_PAD_MSD5018_SM2,                                              ///< 5018 SM2
} SYS_PadType;

/// System information
typedef struct
{
    /// Software information
    struct
    {
        U8                          Board[SYS_BOARD_NAME_MAX];          ///< Board name
        U8                          Platform[SYS_PLATFORM_NAME_MAX];    ///< Platform name
    } SWLib;
} SYS_Info;

/// Memory mapping type
typedef enum
{
    E_SYS_MMAP_LINUX_BASE,      //0
    E_SYS_MMAP_BIN_MEM,         //1
    E_SYS_MMAP_MAD_BASE,        //2
    E_SYS_MMAP_SCALER_DNR_BUF,  //3
    E_SYS_MMAP_RLD_BUF,         //4
    E_SYS_MMAP_MVD_SW,          //5
    E_SYS_MMAP_VD_3DCOMB,       //6
    E_SYS_MMAP_VE,              //7
    E_SYS_MMAP_TTX_BUF,         //8
    E_SYS_MMAP_MPOOL,           //9
    E_SYS_MMAP_EMAC_MEM,        //10
    E_SYS_MMAP_LINUX_MEM,       //11
    E_SYS_MMAP_SVD,             //12
    E_SYS_MMAP_SVD_ALL,         //13
    E_SYS_MMAP_MVD_FB,          //14
    E_SYS_MMAP_MVD_BS,          //15
    E_SYS_MMAP_POSD0_MEM,       //16
    E_SYS_MMAP_POSD1_MEM,       //17
    E_SYS_MMAP_TSP, // samuel, 20081107 //18
    E_SYS_MMAP_AUDIO_CLIP_MEM, // samuel, 20081107  //19
    E_SYS_MMAP_MBOX_SHM,                //20
    E_SYS_MMAP_CHAKRA_SUBSYSTEM,        //21
    E_SYS_MMAP_CHAKRA_FW,           //22
#ifdef CONFIG_MSTAR_KIP
    E_SYS_MMAP_AEON_SHM,               // 23
#endif
    E_SYS_MMAP_NUMBER,          //23

} SYS_Memory_Mapping;


//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
BOOL MDrv_System_Init(void);
//BOOL MDrv_System_SwitchPad(SYS_PadType ePadType);
void MDrv_System_WDTEnable(BOOL bEnable);
void MDrv_System_WDTClear(void);
BOOL MDrv_System_WDTLastStatus(void);
void MDrv_System_WDTSetTime(U32 u32Ms);
void MDrv_System_ResetChip(void);
void MDrv_System_ResetCPU(void);


PMST_PANEL_INFO_t MDrv_SYS_GetPanelInfo(void);

void MDrv_SYS_PowerDown(int src);
void MDrv_SYS_PD_ADC_R(B16 bStatus);
void MDrv_SYS_PD_ADC_G(B16 bStatus);
void MDrv_SYS_PD_ADC_B(B16 bStatus);
void MDrv_SYS_PD_ADC_Y(B16 bStatus);
void MDrv_SYS_PD_GMC_P(B16 bStatus);
void MDrv_SYS_PD_GMC_Y(B16 bStatus);
void MDrv_SYS_PD_GMC_C(B16 bStatus);
void MDrv_SYS_PD_CVBS_Buffer(B16 bStatus);
void MDrv_SYS_PD_DAC_CVBS(B16 bStatus);
void MDrv_SYS_PD_DAC(B16 bStatus);
void MDrv_SYS_PD_FB_DAC(B16 bStatus);
void MDrv_SYS_PD_DAC_RGB(B16 bStatus);
void MDrv_SYS_PD_Audio(B16 bStatus);
void MDrv_SYS_PD_LVDS(B16 bStatus);
void MDrv_SYS_PD_VD(B16 bStatus);
void MDrv_SYS_PD_SVD(B16 bStatus);
void MDrv_SYS_PD_MVD_M4V(B16 bStatus);
void MDrv_SYS_PD_TSP(B16 bStatus);
void MDrv_SYS_PD_VE(B16 bStatus);
void MDrv_SYS_PD_RVD(B16 bStatus);
void MDrv_SYS_PD_STRLD(B16 bStatus);
void MDrv_SYS_PD_AEON(B16 bStatus);
void MDrv_SYS_PD_GOPG2(B16 bStatus);


unsigned int MDrv_SYS_GetDRAMLength(void);

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
extern int MDrv_SYS_GetMMAP(int type, unsigned int *addr, unsigned int *len);
#elif defined(CONFIG_ARM64)
extern int MDrv_SYS_GetMMAP(int type, u64 *addr, u64 *len);
#endif

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)

U32 MDrv_SYS_SetPanelInfo(U32 arg);
void MDrv_SYS_GetPanelRes(U32 arg);
void MDrv_SYS_GetPanelHStart(U32 argv);
void MDrv_SYS_GetGFXGOPPipelineDelay(U32 argv);

void MDrv_SYS_ReadGeneralRegister(U32 arg);
void MDrv_SYS_WriteGeneralRegister(U32 arg);
void MDrv_SYS_LoadAeon(U32 arg);
void MDrv_SYS_ResetAeon(U32 arg);
void MDrv_SYS_EnableAeon(void);
void MDrv_SYS_DumpAeonMessage(void);
void MDrv_SYS_DisableAeon(void);
void MDrv_SYS_SwitchUart(U32 arg);
U32 MDrv_SYS_IsAeonEnable(U32 arg);

void MDrv_SYS_SetNexusPID(U32 argv);
void MDrv_SYS_GetNexusPID(U32 argv);

U32 MDrv_SYS_PCMCIA_WRITE(U32 arg,bool bKernelCopy);
U32 MDrv_SYS_PCMCIA_READ(U32 arg,bool bKernelCopy);
U32 MDrv_SYS_PCMCIA_WRITE_DATA(U32 arg);
U32 MDrv_SYS_PCMCIA_READ_DATA(U32 arg,bool bKernelCopy);

void MDrv_SYS_GetMBoxShareMemory(U32 argv);

void MDrv_SYS_GetMsBinInfo(U32 argv);
void MDrv_SYS_GetMIU1Base(U32 argv);
void MDrv_SYS_GetMIU1BusBase(U32 argv);
void MDrv_SYS_ForceUpgradeOADByDRAM(U32 arg);
void MDrv_SYS_ForceUpgradeENVByDRAM(U32 arg);
void MDrv_SYS_PrintMsg(U32 arg);

U32 MDrv_SYS_GetRawUART(U32 arg);
void MDrv_SYS_ReloadAeon( U32 arg ) ;
U32 MDrv_SYS_Timer(U32 arg) ;
U32 MDrv_SYS_RegOP(U32 arg);
extern void MDrv_SYS_MMAP_Dump( void ) ;
U32 MDrv_SYS_HotelMode(U32 arg) ;
U32 MDrv_SYS_HotelModePrintf(U32 arg) ;

void MDrv_SYS_ChangeUart( U32 arg );

void MDrv_SYS_SetGFXGOPIndex(U32 argv);
void MDrv_SYS_GetGFXGOPIndex(U32 argv);

void MDrv_SYS_SetDisplayControllerSeparated(U32 argv);
U32  MDrv_SYS_GetDisplayControllerSeparated(void);
void MDrv_SYS_IsDisplayControllerSeparated(U32 argv);

void MDrv_SYS_SetNexus(U32 argv);
void MDrv_SYS_HasNexus(U32 argv);


#elif defined(CONFIG_ARM64)

U32 MDrv_SYS_SetPanelInfo(unsigned long arg);
void MDrv_SYS_GetPanelRes(unsigned long arg);
void MDrv_SYS_GetPanelHStart(unsigned long argv);
void MDrv_SYS_GetGFXGOPPipelineDelay(unsigned long argv);

void MDrv_SYS_ReadGeneralRegister(unsigned long arg);
void MDrv_SYS_WriteGeneralRegister(unsigned long arg);
void MDrv_SYS_LoadAeon(unsigned long arg);
void MDrv_SYS_ResetAeon(unsigned long arg);
void MDrv_SYS_EnableAeon(void);
void MDrv_SYS_DumpAeonMessage(void);
void MDrv_SYS_DisableAeon(void);
void MDrv_SYS_SwitchUart(unsigned long arg);
U32 MDrv_SYS_IsAeonEnable(unsigned long arg);

void MDrv_SYS_SetNexusPID(unsigned long argv);
void MDrv_SYS_GetNexusPID(unsigned long argv);

U32 MDrv_SYS_PCMCIA_WRITE(unsigned long arg,bool bKernelCopy);
U32 MDrv_SYS_PCMCIA_READ(unsigned long arg,bool bKernelCopy);
U32 MDrv_SYS_PCMCIA_WRITE_DATA(unsigned long arg);
U32 MDrv_SYS_PCMCIA_READ_DATA(unsigned long arg,bool bKernelCopy);

void MDrv_SYS_GetMBoxShareMemory(unsigned long argv);

void MDrv_SYS_GetMsBinInfo(unsigned long argv);
void MDrv_SYS_GetMIU1Base(unsigned long argv);
void MDrv_SYS_GetMIU1BusBase(unsigned long argv);
void MDrv_SYS_ForceUpgradeOADByDRAM(unsigned long arg);
void MDrv_SYS_ForceUpgradeENVByDRAM(unsigned long arg);
void MDrv_SYS_PrintMsg(unsigned long arg);

U32 MDrv_SYS_GetRawUART(unsigned long arg);
void MDrv_SYS_ReloadAeon(unsigned long arg ) ;
U32 MDrv_SYS_Timer(unsigned long arg) ;
U32 MDrv_SYS_RegOP(unsigned long arg);
extern void MDrv_SYS_MMAP_Dump( void ) ;
U32 MDrv_SYS_HotelMode(unsigned long arg) ;
U32 MDrv_SYS_HotelModePrintf(unsigned long arg) ;

void MDrv_SYS_ChangeUart(unsigned long arg );

void MDrv_SYS_SetGFXGOPIndex(unsigned long argv);
void MDrv_SYS_GetGFXGOPIndex(unsigned long argv);

void MDrv_SYS_SetDisplayControllerSeparated(unsigned long argv);
U32  MDrv_SYS_GetDisplayControllerSeparated(void);
void MDrv_SYS_IsDisplayControllerSeparated(unsigned long argv);

void MDrv_SYS_SetNexus(unsigned long argv);
void MDrv_SYS_HasNexus(unsigned long argv);


#endif

void MDrv_SYS_ReadMemory(void);
void MDrv_SYS_FlushMemory(void);

unsigned int MDrv_SYS_GetPowerStates(void);
unsigned int MDrv_SYS_GetGPIOIR(void);
unsigned int MDrv_SYS_GetGPIOIRType(void);

void MDrv_SYS_UtopiaMdbMkdir(void);

#endif // _DRV_SYSTEM_H_

