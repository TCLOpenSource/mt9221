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
#define GPIO_UNIT_NUM               208

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

#define PAD_PM_SPI_CK               1
#define PAD_PM_SPI_DI               2
#define PAD_PM_SPI_DO               3
#define PAD_IRIN                    4
#define PAD_CEC0                    5
#define PAD_PWM_PM                  6
#define PAD_DDCA_CK                 7
#define PAD_DDCA_DA                 8
#define PAD_GPIO0_PM                9
#define PAD_GPIO1_PM                10
#define PAD_GPIO2_PM                11
#define PAD_GPIO3_PM                12
#define PAD_GPIO4_PM                13
#define PAD_GPIO5_PM                14
#define PAD_GPIO6_PM                15
#define PAD_GPIO7_PM                16
#define PAD_GPIO8_PM                17
#define PAD_GPIO9_PM                18
#define PAD_GPIO10_PM               19
#define PAD_GPIO11_PM               20
#define PAD_GPIO12_PM               21
#define PAD_GPIO13_PM               22
#define PAD_GPIO14_PM               23
#define PAD_GPIO15_PM               24
#define PAD_GPIO16_PM               25
#define PAD_LED0                    26
#define PAD_LED1                    27
#define PAD_HOTPLUGA                28
#define PAD_HOTPLUGB                29
#define PAD_HOTPLUGC                30
#define PAD_HOTPLUGD                31
#define PAD_HOTPLUGA_HDMI20_5V      32
#define PAD_HOTPLUGB_HDMI20_5V      33
#define PAD_HOTPLUGC_HDMI20_5V      34
#define PAD_HOTPLUGD_HDMI20_5V      35
#define PAD_DDCDA_CK                36
#define PAD_DDCDA_DA                37
#define PAD_DDCDB_CK                38
#define PAD_DDCDB_DA                39
#define PAD_DDCDC_CK                40
#define PAD_DDCDC_DA                41
#define PAD_DDCDD_CK                42
#define PAD_DDCDD_DA                43
#define PAD_SAR0                    44
#define PAD_SAR1                    45
#define PAD_SAR2                    46
#define PAD_SAR3                    47
#define PAD_SAR4                    48
#define PAD_VPLUGIN                 49
#define PAD_VID0                    50
#define PAD_VID1                    51
#define PAD_VID2                    52
#define PAD_VID3                    53
#define PAD_WOL_INT_OUT             54
#define PAD_ARC0                    55
#define PAD_DDCR_CK                 56
#define PAD_DDCR_DA                 57
#define PAD_DIM0                    58
#define PAD_DIM1                    59
#define PAD_DIM2                    60
#define PAD_DIM3                    61
#define PAD_GPIO0                   62
#define PAD_GPIO1                   63
#define PAD_GPIO2                   64
#define PAD_GPIO3                   65
#define PAD_GPIO4                   66
#define PAD_GPIO5                   67
#define PAD_GPIO6                   68
#define PAD_GPIO7                   69
#define PAD_GPIO8                   70
#define PAD_GPIO9                   71
#define PAD_GPIO10                  72
#define PAD_GPIO11                  73
#define PAD_GPIO12                  74
#define PAD_GPIO13                  75
#define PAD_GPIO14                  76
#define PAD_GPIO15                  77
#define PAD_GPIO16                  78
#define PAD_GPIO17                  79
#define PAD_GPIO18                  80
#define PAD_GPIO19                  81
#define PAD_GPIO20                  82
#define PAD_GPIO21                  83
#define PAD_GPIO22                  84
#define PAD_GPIO23                  85
#define PAD_GPIO24                  86
#define PAD_GPIO25                  87
#define PAD_GPIO26                  88
#define PAD_GPIO27                  89
#define PAD_GPIO28                  90
#define PAD_GPIO29                  91
#define PAD_GPIO30                  92
#define PAD_GPIO31                  93
#define PAD_GPIO32                  94
#define PAD_GPIO33                  95
#define PAD_GPIO34                  96
#define PAD_GPIO35                  97
#define PAD_GPIO36                  98
#define PAD_GPIO37                  99
#define PAD_I2S_IN_BCK              100
#define PAD_I2S_IN_SD               101
#define PAD_I2S_IN_WS               102
#define PAD_I2S_OUT_BCK             103
#define PAD_I2S_OUT_MCK             104
#define PAD_I2S_OUT_SD              105
#define PAD_I2S_OUT_SD1             106
#define PAD_I2S_OUT_SD2             107
#define PAD_I2S_OUT_SD3             108
#define PAD_I2S_OUT_WS              109
#define PAD_PCM2_CD_N               110
#define PAD_PCM2_CE_N               111
#define PAD_PCM2_IRQA_N             112
#define PAD_PCM2_RESET              113
#define PAD_PCM2_WAIT_N             114
#define PAD_PCM_A0                  115
#define PAD_PCM_A1                  116
#define PAD_PCM_A2                  117
#define PAD_PCM_A3                  118
#define PAD_PCM_A4                  119
#define PAD_PCM_A5                  120
#define PAD_PCM_A6                  121
#define PAD_PCM_A7                  122
#define PAD_PCM_A8                  123
#define PAD_PCM_A9                  124
#define PAD_PCM_A10                 125
#define PAD_PCM_A11                 126
#define PAD_PCM_A12                 127
#define PAD_PCM_A13                 128
#define PAD_PCM_A14                 129
#define PAD_PCM_CD_N                130
#define PAD_PCM_CE_N                131
#define PAD_PCM_D0                  132
#define PAD_PCM_D1                  133
#define PAD_PCM_D2                  134
#define PAD_PCM_D3                  135
#define PAD_PCM_D4                  136
#define PAD_PCM_D5                  137
#define PAD_PCM_D6                  138
#define PAD_PCM_D7                  139
#define PAD_PCM_IORD_N              140
#define PAD_PCM_IOWR_N              141
#define PAD_PCM_IRQA_N              142
#define PAD_PCM_OE_N                143
#define PAD_PCM_REG_N               144
#define PAD_PCM_RESET               145
#define PAD_PCM_WAIT_N              146
#define PAD_PCM_WE_N                147
#define PAD_PWM0                    148
#define PAD_PWM1                    149
#define PAD_PWM2                    150
#define PAD_PWM3                    151
#define PAD_PWM4                    152
#define PAD_SPDIF_IN                153
#define PAD_SPDIF_OUT               154
#define PAD_SPI1_CK                 155
#define PAD_SPI1_DI                 156
#define PAD_SPI2_CK                 157
#define PAD_SPI2_DI                 158
#define PAD_TGPIO0                  159
#define PAD_TGPIO1                  160
#define PAD_TGPIO2                  161
#define PAD_TGPIO3                  162
#define PAD_TS0_CLK                 163
#define PAD_TS0_D0                  164
#define PAD_TS0_D1                  165
#define PAD_TS0_D2                  166
#define PAD_TS0_D3                  167
#define PAD_TS0_D4                  168
#define PAD_TS0_D5                  169
#define PAD_TS0_D6                  170
#define PAD_TS0_D7                  171
#define PAD_TS0_SYNC                172
#define PAD_TS0_VLD                 173
#define PAD_TS1_CLK                 174
#define PAD_TS1_D0                  175
#define PAD_TS1_D1                  176
#define PAD_TS1_D2                  177
#define PAD_TS1_D3                  178
#define PAD_TS1_D4                  179
#define PAD_TS1_D5                  180
#define PAD_TS1_D6                  181
#define PAD_TS1_D7                  182
#define PAD_TS1_SYNC                183
#define PAD_TS1_VLD                 184
#define PAD_TS2_CLK                 185
#define PAD_TS2_D0                  186
#define PAD_TS2_D1                  187
#define PAD_TS2_D2                  188
#define PAD_TS2_D3                  189
#define PAD_TS2_D4                  190
#define PAD_TS2_D5                  191
#define PAD_TS2_D6                  192
#define PAD_TS2_D7                  193
#define PAD_TS2_SYNC                194
#define PAD_TS2_VLD                 195
#define PAD_TS3_CLK                 196
#define PAD_TS3_D0                  197
#define PAD_TS3_D1                  198
#define PAD_TS3_D2                  199
#define PAD_TS3_D3                  200
#define PAD_TS3_D4                  201
#define PAD_TS3_D5                  202
#define PAD_TS3_D6                  203
#define PAD_TS3_D7                  204
#define PAD_TS3_SYNC                205
#define PAD_TS3_VLD                 206
#define PAD_VSYNC_Like              207

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
