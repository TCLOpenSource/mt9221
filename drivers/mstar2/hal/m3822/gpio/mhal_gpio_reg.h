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
#define GPIO_UNIT_NUM               194

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

#define PAD_VID0                    0
#define PAD_VID1                    1
#define PAD_DDCA_CK                 2
#define PAD_DDCA_DA                 3
#define PAD_IRIN                    4
#define PAD_PWM_PM                  5
#define PAD_CEC0                    6
#define PAD_DDCDA_CK                7
#define PAD_DDCDA_DA                8
#define PAD_DDCDB_CK                9
#define PAD_DDCDB_DA                10
#define PAD_DDCDC_CK                11
#define PAD_DDCDC_DA                12
#define PAD_HOTPLUGA_HDMI20_5V      13
#define PAD_HOTPLUGB_HDMI20_5V      14
#define PAD_HOTPLUGC_HDMI20_5V      15
#define PAD_NAND_REZ                16
#define PAD_NAND_CLE                17
#define PAD_NAND_ALE                18
#define PAD_NAND_CEZ                19
#define PAD_NAND_WPZ                20
#define PAD_NAND_WEZ                21
#define PAD_NAND_RBZ                22
#define PAD_HDMIRX_ARCTX            23
#define PAD_GPIO0                   24
#define PAD_GPIO1                   25
#define PAD_GPIO2                   26
#define PAD_GPIO3                   27
#define PAD_GPIO4                   28
#define PAD_GPIO5                   29
#define PAD_GPIO6                   30
#define PAD_USB_DEBUG_GPIO          31
#define PAD_DDCR_CK                 32
#define PAD_DDCR_DA                 33
#define PAD_LD_SPI_CS               34
#define PAD_LD_SPI_CK               35
#define PAD_LD_SPI_MOSI             36
#define PAD_LD_SPI_MISO             37
#define PAD_I2S_OUT_WS              38
#define PAD_I2S_OUT_MCK             39
#define PAD_I2S_OUT_BCK             40
#define PAD_I2S_OUT_SD              41
#define PAD_I2S_OUT_SD1             42
#define PAD_I2S_OUT_SD2             43
#define PAD_I2S_OUT_SD3             44
#define PAD_PCM_A0                  45
#define PAD_PCM_A1                  46
#define PAD_PCM_A2                  47
#define PAD_PCM_A3                  48
#define PAD_PCM_A4                  49
#define PAD_PCM_A5                  50
#define PAD_PCM_A6                  51
#define PAD_PCM_A7                  52
#define PAD_PCM_A8                  53
#define PAD_PCM_A9                  54
#define PAD_PCM_A10                 55
#define PAD_PCM_A11                 56
#define PAD_PCM_A12                 57
#define PAD_PCM_A13                 58
#define PAD_PCM_A14                 59
#define PAD_PCM_D0                  60
#define PAD_PCM_D1                  61
#define PAD_PCM_D2                  62
#define PAD_PCM_D3                  63
#define PAD_PCM_D4                  64
#define PAD_PCM_D5                  65
#define PAD_PCM_D6                  66
#define PAD_PCM_D7                  67
#define PAD_PCM_CD_N                68
#define PAD_PCM_CE_N                69
#define PAD_PCM_IORD_N              70
#define PAD_PCM_IOWR_N              71
#define PAD_PCM_IRQA_N              72
#define PAD_PCM_OE_N                73
#define PAD_PCM_REG_N               74
#define PAD_PCM_RESET               75
#define PAD_PCM_WAIT_N              76
#define PAD_PCM_WE_N                77
#define PAD_PWM0                    78
#define PAD_PWM1                    79
#define PAD_PWM2                    80
#define PAD_SPDIF_OUT               81
#define PAD_TCON0                   82
#define PAD_TCON1                   83
#define PAD_TCON2                   84
#define PAD_TCON3                   85
#define PAD_TCON4                   86
#define PAD_TGPIO0                  87
#define PAD_TGPIO1                  88
#define PAD_TGPIO2                  89
#define PAD_TGPIO3                  90
#define PAD_TS0_CLK                 91
#define PAD_TS0_SYNC                92
#define PAD_TS0_VLD                 93
#define PAD_TS0_D0                  94
#define PAD_TS0_D1                  95
#define PAD_TS0_D2                  96
#define PAD_TS0_D3                  97
#define PAD_TS0_D4                  98
#define PAD_TS0_D5                  99
#define PAD_TS0_D6                  100
#define PAD_TS0_D7                  101
#define PAD_TS1_CLK                 102
#define PAD_TS1_SYNC                103
#define PAD_TS1_VLD                 104
#define PAD_TS1_D0                  105
#define PAD_TS1_D1                  106
#define PAD_TS1_D2                  107
#define PAD_TS1_D3                  108
#define PAD_TS1_D4                  109
#define PAD_TS1_D5                  110
#define PAD_TS1_D6                  111
#define PAD_TS1_D7                  112
#define PAD_TS2_CLK                 113
#define PAD_TS2_SYNC                114
#define PAD_TS2_VLD                 115
#define PAD_TS2_D0                  116
#define PAD_MOD_TX_P00              117
#define PAD_MOD_TX_N00              118
#define PAD_MOD_TX_P01              119
#define PAD_MOD_TX_N01              120
#define PAD_MOD_TX_P02              121
#define PAD_MOD_TX_N02              122
#define PAD_MOD_TX_P03              123
#define PAD_MOD_TX_N03              124
#define PAD_MOD_TX_P04              125
#define PAD_MOD_TX_N04              126
#define PAD_MOD_TX_P05              127
#define PAD_MOD_TX_N05              128
#define PAD_MOD_TX_P06              129
#define PAD_MOD_TX_N06              130
#define PAD_MOD_TX_P07              131
#define PAD_MOD_TX_N07              132
#define PAD_MOD_TX_P08              133
#define PAD_MOD_TX_N08              134
#define PAD_MOD_TX_P09              135
#define PAD_MOD_TX_N09              136
#define PAD_MOD_TX_P10              137
#define PAD_MOD_TX_N10              138
#define PAD_MOD_TX_P11              139
#define PAD_MOD_TX_N11              140
#define PAD_MOD_TX_P12              141
#define PAD_MOD_TX_N12              142
#define PAD_MOD_TX_P13              143
#define PAD_MOD_TX_N13              144
#define PAD_LINEIN_L0               145
#define PAD_LINEIN_R0               146
#define PAD_LINEIN_L1               147
#define PAD_LINEIN_R1               148
#define PAD_RX0N                    149
#define PAD_RX0P                    150
#define PAD_RX1N                    151
#define PAD_RX1P                    152
#define PAD_RX2N                    153
#define PAD_RX2P                    154
#define PAD_RXCN                    155
#define PAD_RXCP                    156
#define PAD_B_RX0N                  157
#define PAD_B_RX0P                  158
#define PAD_B_RX1N                  159
#define PAD_B_RX1P                  160
#define PAD_B_RX2N                  161
#define PAD_B_RX2P                  162
#define PAD_B_RXCN                  163
#define PAD_B_RXCP                  164
#define PAD_C_RX0N                  165
#define PAD_C_RX0P                  166
#define PAD_C_RX1N                  167
#define PAD_C_RX1P                  168
#define PAD_C_RX2N                  169
#define PAD_C_RX2P                  170
#define PAD_C_RXCN                  171
#define PAD_C_RXCP                  172
#define PAD_SAR0                    173
#define PAD_SAR1                    174
#define PAD_SAR2                    175
#define PADA_RIN0P                  176
#define PADA_RIN1P                  177
#define PADA_RIN2P                  178
#define PADA_GIN0P                  179
#define PADA_GIN1P                  180
#define PADA_GIN2P                  181
#define PADA_GIN0M                  182
#define PADA_GIN1M                  183
#define PADA_GIN2M                  184
#define PADA_BIN0P                  185
#define PADA_BIN1P                  186
#define PADA_BIN2P                  187
#define PADA_HSYNC0                 188
#define PADA_HSYNC1                 189
#define PADA_HSYNC2                 190
#define PADA_VSYNC0                 191
#define PADA_VSYNC1                 192
#define PADA_VSYNC2                 193


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

