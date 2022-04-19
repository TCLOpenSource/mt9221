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

#ifndef _REG_GPIO_H_
#define _REG_GPIO_H_

//-------------------------------------------------------------------------------------------------
//  Hardware Capability
//-------------------------------------------------------------------------------------------------
#define GPIO_UNIT_NUM               202
#define GPIO_OEN_INVERSE            1
#define GPIO_NUM_OENIVS             178 //Oen Invert Start
#define GPIO_NUM_OENIVE             187 //OEN Invert End

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_MIPS_BASE               0xFD000000      //Use 8 bit addressing
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_MIPS_BASE           (mstar_pm_base)
#endif

#define REG_ALL_PAD_IN              (0x101ea1UL)      //set all pads (except SPI) as input
#define REG_LVDS_BASE               (0x103200UL)
#define REG_LVDS_BANK               REG_LVDS_BASE

#define PAD_VID0	0
#define PAD_VID1	1
#define PAD_VID2	2
#define PAD_DDCA_CK	3
#define PAD_DDCA_DA	4
#define PAD_IRIN	5
#define PAD_PWM_PM	6
#define PAD_GPIO0_PM	7
#define PAD_GPIO1_PM	8
#define PAD_GPIO2_PM	9
#define PAD_USB_CTRL	10
#define PAD_GPIO5_PM	11
#define PAD_GPIO6_PM	12
#define PAD_GPIO7_PM	13
#define PAD_GPIO8_PM	14
#define PAD_GPIO9_PM	15
#define PAD_GPIO10_PM	16
#define PAD_GPIO11_PM	17
#define PAD_GPIO12_PM	18
#define PAD_GPIO13_PM	19
#define PAD_GPIO14_PM	20
#define PAD_DDCDA_CK	21
#define PAD_DDCDA_DA	22
#define PAD_DDCDB_CK	23
#define PAD_DDCDB_DA	24
#define PAD_WOL_INT_OUT	25
#define PAD_SAR0	26
#define PAD_SAR1	27
#define PAD_SAR2	28
#define PAD_SAR3	29
#define PAD_SAR4	30
#define PAD_VPLUGIN	31
#define PAD_ARC0	32
#define PAD_DDCR_CK	33
#define PAD_DDCR_DA	34
#define PAD_GPIO0	35
#define PAD_GPIO1	36
#define PAD_GPIO2	37
#define PAD_GPIO3	38
#define PAD_GPIO4	39
#define PAD_GPIO5	40
#define PAD_GPIO6	41
#define PAD_GPIO7	42
#define PAD_GPIO8	43
#define PAD_GPIO9	44
#define PAD_GPIO10	45
#define PAD_GPIO11	46
#define PAD_GPIO12	47
#define PAD_GPIO13	48
#define PAD_GPIO14	49
#define PAD_I2S_IN_BCK	50
#define PAD_I2S_IN_SD	51
#define PAD_I2S_IN_WS	52
#define PAD_I2S_OUT_BCK	53
#define PAD_I2S_OUT_MCK	54
#define PAD_I2S_OUT_SD	55
#define PAD_I2S_OUT_WS	56
#define PAD_PWM0	57
#define PAD_PWM1	58
#define PAD_PWM2	59
#define PAD_SPDIF_IN	60
#define PAD_SPDIF_OUT	61
#define PAD_SPI1_CK	62
#define PAD_SPI1_DI	63
#define PAD_TGPIO0	64
#define PAD_TGPIO1	65
#define PAD_TGPIO2	66
#define PAD_TGPIO3	67
#define PAD_TS0_CLK	68
#define PAD_TS0_D0	69
#define PAD_TS0_SYNC	70
#define PAD_TS0_VLD	71
#define PAD_MIC_BCK	72
#define PAD_MIC_SD0	73
#define PAD_MIC_SD1	74
#define PAD_MIC_SD2	75
#define PADA_CVBS1      181
#define PADA_CVBS_OUT1	183
#define PAD_MOD_TX_N01	201


/* analog gpio */
/*
#define PADA_RIN0P	178
#define PADA_CVBS0	179
#define PADA_GIN0P	180
#define PADA_GIN0M	182
#define PADA_BIN0P	184
#define PADA_HSYNC0	185
#define PADA_VSYNC0	186
#define PADA_VCOM	187
*/

#define GPIO_OEN                    0   //set o to nake output
#define GPIO_ODN                    1

#define IN_HIGH                     1   //input high
#define IN_LOW                      0   //input low

#define OUT_HIGH                    1   //output high
#define OUT_LOW                     0   //output low

#define MHal_GPIO_REG(addr)         (*(volatile U8*)(REG_MIPS_BASE + (((addr) & ~1) << 1) + (addr & 1)))

#define RIU     ((U16 volatile *) REG_MIPS_BASE)
#define RIU8    ((U8  volatile *) REG_MIPS_BASE)

#define MST_MACRO_START     do {
#define MST_MACRO_END       } while (0)

#define MDrv_ReadRegBit( u32Reg, u8Mask )                                               \
        (RIU8[(u32Reg) * 2 - ((u32Reg) & 1)] & (u8Mask))

#define MDrv_WriteRegBit( u32Reg, bEnable, u8Mask )                                     \
    MST_MACRO_START                                                                     \
    U32 u32Reg8 = ((u32Reg) * 2) - ((u32Reg) & 1);                                   \
    RIU8[u32Reg8] = (bEnable) ? (RIU8[u32Reg8] |  (u8Mask)) :                           \
                                (RIU8[u32Reg8] & ~(u8Mask));                            \
    MST_MACRO_END

#define MDrv_WriteByte( u32Reg, u8Val )                                                 \
    MST_MACRO_START                                                                     \
    RIU8[((u32Reg) * 2) - ((u32Reg) & 1)] = u8Val;                                      \
    MST_MACRO_END

#define MDrv_ReadByte( u32Reg )                                                         \
        (RIU8[(u32Reg) * 2 - ((u32Reg) & 1)])

#define MDrv_Read2Byte( u32Reg )                                                        \
        (RIU8[(u32Reg)])

#define MDrv_Write2Byte( u32Reg, u16Val )                                               \
    MST_MACRO_START                                                                     \
    if ((u32Reg) & 0x01)                                                                \
    {                                                                                   \
        RIU8[((u32Reg) * 2) - 1] = (U8)((u16Val));                                   \
        RIU8[((u32Reg) + 1) * 2] = (U8)((u16Val) >> 8);                              \
    }                                                                                   \
    else                                                                                \
    {                                                                                   \
        RIU[u32Reg] = u16Val;                                                           \
    }                                                                                   \
    MST_MACRO_END
//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

#endif // _REG_GPIO_H_

