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
#define GPIO_UNIT_NUM               182

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_MIPS_BASE               0xFD000000      //Use 8 bit addressing
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_MIPS_BASE           (mstar_pm_base)
#endif

#define REG_ALL_PAD_IN              (0x3229e0UL)      //set all pads (except SPI) as input
#define REG_LVDS_BASE               (0x103200UL)
#define REG_LVDS_BANK               REG_LVDS_BASE

#define PAD_DDCA_CK	1
#define PAD_DDCA_DA	2
#define PAD_IRIN	3
#define PAD_PWM_PM	4
#define PAD_CEC0	5
#define PAD_GPIO0_PM	6
#define PAD_GPIO1_PM	7
#define PAD_GPIO2_PM	8
#define PAD_USB_CTRL	9
#define PAD_GPIO5_PM	10
#define PAD_GPIO6_PM	11
#define PAD_GPIO7_PM	12
#define PAD_GPIO8_PM	13
#define PAD_GPIO9_PM	14
#define PAD_GPIO10_PM	15
#define PAD_GPIO11_PM	16
#define PAD_GPIO12_PM	17
#define PAD_GPIO13_PM	18
#define PAD_GPIO14_PM	19
#define PAD_GPIO15_PM	20
#define PAD_GPIO16_PM	21
#define PAD_GPIO17_PM	22
#define PAD_GPIO18_PM	23
#define PAD_GPIO19_PM	24
#define PAD_GPIO20_PM	25
#define PAD_GPIO21_PM	26
#define PAD_GPIO22_PM	27
#define PAD_GPIO23_PM	28
#define PAD_GPIO24_PM	29
#define PAD_GPIO25_PM	30
#define PAD_GPIO26_PM	31
#define PAD_DDCDA_CK	34
#define PAD_DDCDA_DA	35
#define PAD_DDCDB_CK	36
#define PAD_DDCDB_DA	37
#define PAD_DDCDC_CK	38
#define PAD_DDCDC_DA	39
#define PAD_DDCDD_CK	40
#define PAD_DDCDD_DA	41
#define PAD_SAR0	42
#define PAD_SAR1	43
#define PAD_SAR2	44
#define PAD_SAR3	45
#define PAD_SAR4	46
#define PAD_VPLUGIN	47
#define PAD_VID0	50
#define PAD_VID1	51
#define PAD_VID2	52
#define PAD_WOL_INT_OUT	53
#define PAD_DDCR_CK	54
#define PAD_DDCR_DA	55
#define PAD_GPIO0	60
#define PAD_GPIO8	62
#define PAD_GPIO9	63
#define PAD_GPIO10	64
#define PAD_GPIO11	65
#define PAD_GPIO12	66
#define PAD_GPIO13	67
#define PAD_GPIO16	70
#define PAD_I2S_IN_BCK	73
#define PAD_I2S_IN_DIN0	74
#define PAD_I2S_IN_DIN1	75
#define PAD_I2S_IN_DIN2	76
#define PAD_I2S_IN_WCK	77
#define PAD_I2S_OUT_BCK	78
#define PAD_I2S_OUT_MCK	79
#define PAD_I2S_OUT_SD0	80
#define PAD_I2S_OUT_SD1	81
#define PAD_I2S_OUT_SD2	82
#define PAD_I2S_OUT_WCK	83
#define PAD_LD_SPI_CK	84
#define PAD_LD_SPI_CS	85
#define PAD_LD_SPI_MISO	86
#define PAD_LD_SPI_MOSI	87
#define PAD_MIC_BCK	88
#define PAD_MIC_SD0	89
#define PAD_MIC_SD1	90
#define PAD_MIC_SD2	91
#define PAD_PCM2_CD_N	92
#define PAD_PCM2_CE_N	93
#define PAD_PCM2_IRQA_N	94
#define PAD_PCM2_RESET	95
#define PAD_PCM2_WAIT_N	96
#define PAD_PCM_A0	97
#define PAD_PCM_A1	98
#define PAD_PCM_A2	99
#define PAD_PCM_A3	100
#define PAD_PCM_A4	101
#define PAD_PCM_A5	102
#define PAD_PCM_A6	103
#define PAD_PCM_A7	104
#define PAD_PCM_A8	105
#define PAD_PCM_A9	106
#define PAD_PCM_A10	107
#define PAD_PCM_A11	108
#define PAD_PCM_A12	109
#define PAD_PCM_A13	110
#define PAD_PCM_A14	111
#define PAD_PCM_CD_N	112
#define PAD_PCM_CE_N	113
#define PAD_PCM_D0	114
#define PAD_PCM_D1	115
#define PAD_PCM_D2	116
#define PAD_PCM_D3	117
#define PAD_PCM_D4	118
#define PAD_PCM_D5	119
#define PAD_PCM_D6	120
#define PAD_PCM_D7	121
#define PAD_PCM_IORD_N	122
#define PAD_PCM_IOWR_N	123
#define PAD_PCM_IRQA_N	124
#define PAD_PCM_OE_N	125
#define PAD_PCM_REG_N	126
#define PAD_PCM_RESET	127
#define PAD_PCM_WAIT_N	128
#define PAD_PCM_WE_N	129
#define PAD_PWM0	130
#define PAD_PWM1	131
#define PAD_PWM2	132
#define PAD_SPDIF_IN	133
#define PAD_SPDIF_OUT	134
#define PAD_TCON0	135
#define PAD_TCON1	136
#define PAD_TCON2	137
#define PAD_TCON3	138
#define PAD_TCON4	139
#define PAD_TGPIO0	140
#define PAD_TGPIO1	141
#define PAD_TGPIO2	142
#define PAD_TGPIO3	143
#define PAD_TS0_CLK	144
#define PAD_TS0_D0	145
#define PAD_TS0_D1	146
#define PAD_TS0_D2	147
#define PAD_TS0_D3	148
#define PAD_TS0_D4	149
#define PAD_TS0_D5	150
#define PAD_TS0_D6	151
#define PAD_TS0_D7	152
#define PAD_TS0_SYNC	153
#define PAD_TS0_VLD	154
#define PAD_TS1_CLK	155
#define PAD_TS1_D0	156
#define PAD_TS1_D1	157
#define PAD_TS1_D2	158
#define PAD_TS1_D3	159
#define PAD_TS1_D4	160
#define PAD_TS1_D5	161
#define PAD_TS1_D6	162
#define PAD_TS1_D7	163
#define PAD_TS1_SYNC	164
#define PAD_TS1_VLD	165
#define PAD_TS2_CLK	166
#define PAD_TS2_D0	167
#define PAD_TS2_D1	168
#define PAD_TS2_D2	169
#define PAD_TS2_D3	170
#define PAD_TS2_D4	171
#define PAD_TS2_D5	172
#define PAD_TS2_D6	173
#define PAD_TS2_D7	174
#define PAD_TS2_SYNC	175
#define PAD_TS2_VLD	176
#define PAD_HDMIRX_ARCTX	181

/* analog gpio */
/*
#define PADA_RIN0P	182
#define PADA_RIN1P	183
#define PADA_GIN0P	184
#define PADA_GIN1P	185
#define PADA_GIN0M	186
#define PADA_GIN1M	187
#define PADA_BIN0P	188
#define PADA_BIN1P	189
#define PADA_HSYNC0	190
#define PADA_VSYNC0	191
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

