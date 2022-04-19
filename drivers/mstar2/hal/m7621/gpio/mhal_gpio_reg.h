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
#define GPIO_UNIT_NUM               207

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

#define PAD_PM_SPI_CK				1
#define PAD_PM_SPI_DI				2
#define PAD_PM_SPI_DO				3
#define PAD_IRIN				    4
#define PAD_CEC0			    	5
#define PAD_PWM_PM			    	6
#define PAD_DDCA_CK			    	7
#define PAD_DDCA_DA			    	8
#define PAD_GPIO0_PM				9
#define PAD_GPIO1_PM				10
#define PAD_GPIO2_PM				11
#define PAD_GPIO3_PM				12
#define PAD_GPIO5_PM				13
#define PAD_GPIO6_PM				14
#define PAD_GPIO7_PM				15
#define PAD_GPIO8_PM				16
#define PAD_GPIO9_PM				17
#define PAD_GPIO10_PM				18
#define PAD_GPIO11_PM				19
#define PAD_GPIO12_PM				20
#define PAD_GPIO13_PM				21
#define PAD_GPIO14_PM				22
#define PAD_GPIO15_PM				23
#define PAD_GPIO16_PM				24
#define PAD_GPIO17_PM				25
#define PAD_GPIO18_PM				26
#define PAD_GPIO19_PM				27
#define PAD_GPIO20_PM				28
#define PAD_GPIO21_PM				29
#define PAD_GPIO22_PM				30
#define PAD_GPIO23_PM				31
#define PAD_GPIO24_PM				32
#define PAD_LED0			    	33
#define PAD_LED1				    34
#define PAD_HOTPLUGA				35
#define PAD_HOTPLUGB				36
#define PAD_HOTPLUGC				37
#define PAD_HOTPLUGD				38
#define PAD_HOTPLUGA_HDMI20_5V		39
#define PAD_HOTPLUGB_HDMI20_5V		40
#define PAD_HOTPLUGC_HDMI20_5V		41
#define PAD_HOTPLUGD_HDMI20_5V		42
#define PAD_DDCDA_CK				43
#define PAD_DDCDA_DA				44
#define PAD_DDCDB_CK				45
#define PAD_DDCDB_DA				46
#define PAD_DDCDC_CK				47
#define PAD_DDCDC_DA				48
#define PAD_DDCDD_CK				49
#define PAD_DDCDD_DA				50
#define PAD_SAR0				    51
#define PAD_SAR1				    52
#define PAD_SAR2			    	53
#define PAD_SAR3			    	54
#define PAD_SAR4			    	55
#define PAD_VPLUGIN			    	56
#define PAD_VID0			    	57
#define PAD_VID1			    	58
#define PAD_VID2			    	59
#define PAD_VID3			    	60
#define PAD_WOL_INT_OUT				61
#define PAD_DDCR_CK				    62
#define PAD_DDCR_DA			    	63
#define PAD_DIM0		    		64
#define PAD_DIM1		    		65
#define PAD_DIM2		    		66
#define PAD_DIM3		    		67
#define PAD_GPIO0		    		68
#define PAD_GPIO1		    		69
#define PAD_GPIO2			    	70
#define PAD_GPIO3			    	71
#define PAD_GPIO4			    	72
#define PAD_GPIO5			       	73
#define PAD_GPIO6			    	74
#define PAD_GPIO7		    		75
#define PAD_GPIO8			    	76
#define PAD_GPIO9				    77
#define PAD_GPIO10	    			78
#define PAD_GPIO11		    		79
#define PAD_GPIO12			    	80
#define PAD_GPIO13				    81
#define PAD_GPIO14				    82
#define PAD_GPIO15				    83
#define PAD_GPIO16				    84
#define PAD_GPIO17				    85
#define PAD_GPIO18				    86
#define PAD_GPIO19				    87
#define PAD_GPIO20				    88
#define PAD_GPIO21				    89
#define PAD_GPIO22				    90
#define PAD_GPIO23				    91
#define PAD_GPIO24				    92
#define PAD_GPIO25				    93
#define PAD_GPIO26				    94
#define PAD_GPIO27				    95
#define PAD_GPIO28				    96
#define PAD_GPIO29				    97
#define PAD_GPIO30				    98
#define PAD_GPIO31				    99
#define PAD_GPIO36				    100
#define PAD_GPIO37				    101
#define PAD_HDMIRX_ARCTX			102
#define PAD_I2S_IN_BCK				103
#define PAD_I2S_IN_SD				104
#define PAD_I2S_IN_WS				105
#define PAD_I2S_OUT_BCK				106
#define PAD_I2S_OUT_MCK				107
#define PAD_I2S_OUT_SD				108
#define PAD_I2S_OUT_SD1				109
#define PAD_I2S_OUT_SD2				110
#define PAD_I2S_OUT_SD3				111
#define PAD_I2S_OUT_WS				112
#define PAD_PCM2_CD_N				113
#define PAD_PCM2_CE_N				114
#define PAD_PCM2_IRQA_N				115
#define PAD_PCM2_RESET				116
#define PAD_PCM2_WAIT_N				117
#define PAD_PCM_A0			    	118
#define PAD_PCM_A1	    			119
#define PAD_PCM_A2	    			120
#define PAD_PCM_A3	    			121
#define PAD_PCM_A4	    			122
#define PAD_PCM_A5	    			123
#define PAD_PCM_A6	    			124
#define PAD_PCM_A7	    			125
#define PAD_PCM_A8	    			126
#define PAD_PCM_A9	    			127
#define PAD_PCM_A10	    			128
#define PAD_PCM_A11	    			129
#define PAD_PCM_A12		    		130
#define PAD_PCM_A13		    		131
#define PAD_PCM_A14		    		132
#define PAD_PCM_CD_N	   			133
#define PAD_PCM_CE_N	    		134
#define PAD_PCM_D0		    		135
#define PAD_PCM_D1		    		136
#define PAD_PCM_D2		    		137
#define PAD_PCM_D3		    		138
#define PAD_PCM_D4		    		139
#define PAD_PCM_D5		    		140
#define PAD_PCM_D6		    		141
#define PAD_PCM_D7		    		142
#define PAD_PCM_IORD_N				143
#define PAD_PCM_IOWR_N				144
#define PAD_PCM_IRQA_N				145
#define PAD_PCM_OE_N				146
#define PAD_PCM_REG_N				147
#define PAD_PCM_RESET				148
#define PAD_PCM_WAIT_N				149
#define PAD_PCM_WE_N				150
#define PAD_PWM0			    	151
#define PAD_PWM1			    	152
#define PAD_PWM2    				153
#define PAD_PWM3    				154
#define PAD_PWM4    				155
#define PAD_SPDIF_IN				156
#define PAD_SPDIF_OUT				157
#define PAD_TGPIO0	    			158
#define PAD_TGPIO1		    		159
#define PAD_TGPIO2		     		160
#define PAD_TGPIO3		    		161
#define PAD_TS0_CLK		    		162
#define PAD_TS0_D0		    		163
#define PAD_TS0_D1		    		164
#define PAD_TS0_D2		    		165
#define PAD_TS0_D3		    		166
#define PAD_TS0_D4		    		167
#define PAD_TS0_D5		    		168
#define PAD_TS0_D6			    	169
#define PAD_TS0_D7			    	170
#define PAD_TS0_SYNC				171
#define PAD_TS0_VLD		    		172
#define PAD_TS1_CLK		    		173
#define PAD_TS1_D0		    		174
#define PAD_TS1_D1		    		175
#define PAD_TS1_D2		    		176
#define PAD_TS1_D3			    	177
#define PAD_TS1_D4			    	178
#define PAD_TS1_D5				    179
#define PAD_TS1_D6		    		180
#define PAD_TS1_D7		    		181
#define PAD_TS1_SYNC				182
#define PAD_TS1_VLD		    		183
#define PAD_TS2_CLK		    		184
#define PAD_TS2_D0		    		185
#define PAD_TS2_D1			    	186
#define PAD_TS2_D2				    187
#define PAD_TS2_D3	    			188
#define PAD_TS2_D4	    			189
#define PAD_TS2_D5	    			190
#define PAD_TS2_D6	    			191
#define PAD_TS2_D7	    			192
#define PAD_TS2_SYNC				193
#define PAD_TS2_VLD		    		194
#define PAD_TS3_CLK		    		195
#define PAD_TS3_D0		    		196
#define PAD_TS3_D1		    		197
#define PAD_TS3_D2		    		198
#define PAD_TS3_D3		    		199
#define PAD_TS3_D4		    		200
#define PAD_TS3_D5		    		201
#define PAD_TS3_D6		    		202
#define PAD_TS3_D7		    		203
#define PAD_TS3_SYNC				204
#define PAD_TS3_VLD			    	205
#define PAD_VSYNC_Like				206

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
