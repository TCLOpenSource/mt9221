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
#define GPIO_UNIT_NUM               140
#define GPIO_OEN_INVERSE            0
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

/* VID PADs are not open in use as default */
/*
#define PAD_VID0	0
#define PAD_VID1	1
#define PAD_VID2	2
*/
#define PAD_DDCA_CK	3
#define PAD_DDCA_DA	4
#define PAD_IRIN	5
#define PAD_PWM0_PM	6
#define PAD_PWM1_PM	7
#define PAD_GPIO0_PM	9
#define PAD_GPIO1_PM	10
#define PAD_GPIO2_PM	11
#define PAD_USB_CTRL	12
#define PAD_GPIO5_PM	13
#define PAD_GPIO6_PM	14
#define PAD_GPIO7_PM	15
#define PAD_GPIO8_PM	16
#define PAD_PM_SPI_MOSI	17
#define PAD_PM_SPI_MISO	18
#define PAD_PM_SPI_CK	19
#define PAD_PM_SPI_CS	20
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
#define PAD_DDCR_CK	35
#define PAD_DDCR_DA	36
#define PAD_GPIO0	37
#define PAD_GPIO1	38
#define PAD_GPIO2	39
#define PAD_GPIO3	40
#define PAD_GPIO4	41
#define PAD_GPIO5	42
#define PAD_GPIO6	43
#define PAD_GPIO7	44
#define PAD_GPIO8	45
#define PAD_GPIO9	46
#define PAD_GPIO10	47
#define PAD_GPIO11	48
#define PAD_GPIO12	49
#define PAD_I2S_IN_BCK	50
#define PAD_I2S_IN_SD	51
#define PAD_I2S_IN_WS	52
#define PAD_I2S_OUT_BCK	53
#define PAD_I2S_OUT_MCK	54
#define PAD_I2S_OUT_SD	55
#define PAD_I2S_OUT_SD1	56
#define PAD_I2S_OUT_SD2	57
#define PAD_I2S_OUT_WS	58
#define PAD_PWM0	59
#define PAD_PWM1	60
#define PAD_PWM2	61
#define PAD_SPDIF_IN	62
#define PAD_SPDIF_OUT	63
#define PAD_SPI0_SCK	64
#define PAD_SPI0_MOSI	65
#define PAD_TGPIO2	66
#define PAD_TGPIO3	67
#define PAD_TS0_CLK	68
#define PAD_TS0_D0	69
#define PAD_TS0_D1	70
#define PAD_TS0_D2	71
#define PAD_TS0_D3	72
#define PAD_TS0_D4	73
#define PAD_TS0_D5	74
#define PAD_TS0_D6	75
#define PAD_TS0_D7	76
#define PAD_TS0_SYNC	77
#define PAD_TS0_VLD	78
#define PAD_MIC_BCK	79
#define PAD_MIC_SD0	80
#define PAD_MIC_SD1	81
#define PAD_MIC_SD2	82
#define PAD_DISEQC_OUT	83
#define PAD_DISEQC_IN	84
#define PAD_NAND_CLE	85
#define PAD_NAND_ALE	86
#define PAD_NAND_WEZ	87
#define PAD_NAND_WPZ	88
#define PAD_SD_CMD	89
#define PAD_SD_CLK	90
#define PAD_TS1_CLK	91
#define PAD_TS1_D0	92
#define PAD_TS1_D1	93
#define PAD_TS1_D2	94
#define PAD_TS1_D3	95
#define PAD_TS1_D4	96
#define PAD_TS1_D5	97
#define PAD_TS1_D6	98
#define PAD_TS1_D7	99
#define PAD_TS1_SYNC	100
#define PAD_TS1_VLD	101
#define PAD_PCM_A0	102
#define PAD_PCM_A1	103
#define PAD_PCM_A2	104
#define PAD_PCM_A3	105
#define PAD_PCM_A4	106
#define PAD_PCM_A5	107
#define PAD_PCM_A6	108
#define PAD_PCM_A7	109
#define PAD_PCM_A8	110
#define PAD_PCM_A9	111
#define PAD_PCM_A10	112
#define PAD_PCM_A11	113
#define PAD_PCM_A12	114
#define PAD_PCM_A13	115
#define PAD_PCM_A14	116
#define PAD_PCM_D0	117
#define PAD_PCM_D1	118
#define PAD_PCM_D2	119
#define PAD_PCM_D3	120
#define PAD_PCM_D4	121
#define PAD_PCM_D5	122
#define PAD_PCM_D6	123
#define PAD_PCM_D7	124
#define PAD_PCM_IOWR_N	125
#define PAD_PCM_WE_N	126
#define PAD_PCM_IRQA_N	127
#define PAD_PCM_WAIT_N	128
#define PAD_PCM_RESET	129
#define PAD_PCM_CD_N	130
#define PAD_PCM_CE_N	131
#define PAD_PCM_OE_N	132
#define PAD_PCM_IORD_N	133
#define PAD_PCM_REG_N	134
#define PAD_SPI0_SCZ	135
#define PAD_SPI0_MISO	136
/*
#define PAD_PWM_DAC0	137
#define PAD_PWM_DAC1	138
*/
#define PAD_HDMIRX_ARCTX	139

/* analog gpio */
/*
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

