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

#ifndef _REG_IIC_H_
#define _REG_IIC_H_


//-------------------------------------------------------------------------------------------------
//  Hardware Capability
//-------------------------------------------------------------------------------------------------
//#define IIC_UNIT_NUM               2

//-------------------------------------------------------------------------------------------------
//  Macro and Define
//-------------------------------------------------------------------------------------------------
#define MHal_IIC_DELAY()            { udelay(1000); udelay(1000); udelay(1000); udelay(1000); udelay(1000); }//delay 5ms

extern ptrdiff_t mstar_pm_base;

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define RIU_MAP                    0xfd000000
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define RIU_MAP                    mstar_pm_base
#endif
#define RIU8                            ((unsigned char volatile *) RIU_MAP)
//############################
//
//IP bank address : for pad mux in chiptop
//
//############################
#define CHIP_REG_BASE                   (0x101E00)
#define PMSLEEP_REG_BASE                (0x0E00)
#define CLKGEN1_REG_BASE                (0x103300)
#define CLKGEN2_REG_BASE                (0x100A00)

//for port 0
#define CHIP_REG_HWI2C_MIIC0            (CHIP_REG_BASE+ (0x6E*2))
    #define CHIP_MIIC0_PAD_0            (0)
    #define CHIP_MIIC0_PAD_1            (__BIT0) //PAD_GPIO28/PAD_GPIO29
    #define CHIP_MIIC0_PAD_MSK          (__BIT0)

//for port 1
#define CHIP_REG_HWI2C_MIIC1            (CHIP_REG_BASE+ (0x6E*2))
    #define CHIP_MIIC1_PAD_0            (0)
    #define CHIP_MIIC1_PAD_1            (__BIT1) //PAD_TGPIO2/PAD_TGPIO3
    #define CHIP_MIIC1_PAD_2            (__BIT2) //PAD_GPIO36/PAD_GPIO37
    #define CHIP_MIIC1_PAD_MSK          (__BIT1|__BIT2)

//for port 2
#define CHIP_REG_HWI2C_MIIC2            (CHIP_REG_BASE+ (0x6E*2))
    #define CHIP_MIIC2_PAD_0            (0)
    #define CHIP_MIIC2_PAD_1            (__BIT3) //PAD_I2S_IN_BCK/PAD_I2S_IN_SD
    #define CHIP_MIIC2_PAD_MSK          (__BIT3)

//for port 3-1
#define CHIP_REG_HWI2C_DDCR             (CHIP_REG_BASE+ (0x57*2))
    #define CHIP_DDCR_PAD_0             (0)
    #define CHIP_DDCR_PAD_1             (__BIT1) //PAD_DDCR_CK/PAD_DDCR_DA
    #define CHIP_DDCR_PAD_MSK           (__BIT1|__BIT0)
//for port3-2
#define CHIP_REG_HWI2C_MIIC3            (CHIP_REG_BASE+ (0x6F*2)+1)
    #define CHIP_MIIC3_PAD_0            (0)
    #define CHIP_MIIC3_PAD_1            (__BIT1) //PAD_GPIO36/PAD_GPIO37
    #define CHIP_MIIC3_PAD_2            (__BIT2) //PAD_GPIO19/PAD_GPIO20
    #define CHIP_MIIC3_PAD_MSK          (__BIT1|__BIT2)

//for port 4
#define CHIP_REG_HWI2C_MIIC4            (CHIP_REG_BASE+ (0x6F*2))
    #define CHIP_MIIC4_PAD_0            (0)
    #define CHIP_MIIC4_PAD_1            (__BIT0)  //PAD_GPIO30/PAD_GPIO31
    #define CHIP_MIIC4_PAD_MSK          (__BIT0)

//for port 5
#define CHIP_REG_HWI2C_MIIC5            (CHIP_REG_BASE+ (0x6F*2))
    #define CHIP_MIIC5_PAD_0            (0)
    #define CHIP_MIIC5_PAD_1            (__BIT1) //PAD_TGPIO0/PAD_TGPIO1
    #define CHIP_MIIC5_PAD_MSK          (__BIT1)

//for port 6
#define CHIP_REG_HWI2C_MIIC6            (PMSLEEP_REG_BASE+ (0x64*2)+1)
    #define CHIP_MIIC6_PAD_0            0
    #define CHIP_MIIC6_PAD_1            (__BIT6) //PAD_GPIO2_PM/PAD_GPIO9_PM
    #define CHIP_MIIC6_PAD_2            (__BIT7) //PAD_VID0/PAD_VID1
    #define CHIP_MIIC6_PAD_MSK          (__BIT6|__BIT7)

//pad mux configuration
#define CHIP_REG_ALLPADIN               (CHIP_REG_BASE+0xA1)
    #define CHIP_ALLPAD_IN              (__BIT7)

#if 0
#define CHIP_REG_HWI2C_MIIC0_CLK              (CLKGEN1_REG_BASE + (0x30) * 2)
    #define CHIP_REG_HWI2C_MIIC0_CLK_72M      0
    #define CHIP_REG_HWI2C_MIIC0_CLK_XTAL     (__BIT2)
    #define CHIP_REG_HWI2C_MIIC0_CLL_36M      (__BIT3)
    #define CHIP_REG_HWI2C_MIIC0_CLK_54M      (__BIT2 | __BIT3)
    #define CHIP_REG_HWI2C_MIIC0_CLK_MSK      (__BIT0 | __BIT1 | __BIT2 | __BIT3)

#define CHIP_REG_HWI2C_MIIC1_CLK              (CLKGEN1_REG_BASE + (0x30) * 2)
    #define CHIP_REG_HWI2C_MIIC1_CLK_72M      0
    #define CHIP_REG_HWI2C_MIIC1_CLK_XTAL     (__BIT6)
    #define CHIP_REG_HWI2C_MIIC1_CLL_36M      (__BIT7)
    #define CHIP_REG_HWI2C_MIIC1_CLK_54M      (__BIT6 | __BIT7)
    #define CHIP_REG_HWI2C_MIIC1_CLK_MSK      (__BIT4 | __BIT5 | __BIT6 | __BIT7)

#define CHIP_REG_HWI2C_MIIC2_CLK              (CLKGEN1_REG_BASE + (0x30) * 2 + 1)
    #define CHIP_REG_HWI2C_MIIC2_CLK_72M      0
    #define CHIP_REG_HWI2C_MIIC2_CLK_XTAL     (__BIT2)
    #define CHIP_REG_HWI2C_MIIC2_CLL_36M      (__BIT3)
    #define CHIP_REG_HWI2C_MIIC2_CLK_54M      (__BIT2 | __BIT3)
    #define CHIP_REG_HWI2C_MIIC2_CLK_MSK      (__BIT0 | __BIT1 | __BIT2 | __BIT3)

#define CHIP_REG_HWI2C_MIIC3_CLK              (CLKGEN2_REG_BASE + (0x3A) * 2)
    #define CHIP_REG_HWI2C_MIIC3_CLK_72M      0
    #define CHIP_REG_HWI2C_MIIC3_CLK_XTAL     (__BIT2)
    #define CHIP_REG_HWI2C_MIIC3_CLL_36M      (__BIT3)
    #define CHIP_REG_HWI2C_MIIC3_CLK_54M      (__BIT2 | __BIT3)
    #define CHIP_REG_HWI2C_MIIC3_CLK_MSK      (__BIT0 | __BIT1 | __BIT2 | __BIT3)

#define CHIP_REG_HWI2C_MIIC4_CLK              (CLKGEN2_REG_BASE + (0x3A) * 2)
    #define CHIP_REG_HWI2C_MIIC4_CLK_72M      0
    #define CHIP_REG_HWI2C_MIIC4_CLK_XTAL     (__BIT6)
    #define CHIP_REG_HWI2C_MIIC4_CLL_36M      (__BIT7)
    #define CHIP_REG_HWI2C_MIIC4_CLK_54M      (__BIT6 | __BIT7)
    #define CHIP_REG_HWI2C_MIIC4_CLK_MSK      (__BIT4 | __BIT5 | __BIT6 | __BIT7)

#define CHIP_REG_HWI2C_MIIC5_CLK              (PMSLEEP_REG_BASE + (0x26) * 2 + 1)
    #define CHIP_REG_HWI2C_MIIC5_CLK_72M      0
    #define CHIP_REG_HWI2C_MIIC5_CLK_XTAL     (__BIT6)
    #define CHIP_REG_HWI2C_MIIC5_CLL_36M      (__BIT7)
    #define CHIP_REG_HWI2C_MIIC5_CLK_54M      (__BIT6 | __BIT7)
    #define CHIP_REG_HWI2C_MIIC5_CLK_MSK      (__BIT4 | __BIT5 | __BIT6 | __BIT7)
#endif

//############################
//
//IP bank address : for independent port
//
//############################
//Standard mode
#define HWI2C_REG_BASE                  (0) //0x1(11800) + offset ==> default set to port 0
#define HWI2C_PORT0_REG_BASE            (0x111800)
#define HWI2C_PORT1_REG_BASE            (0x111900)
#define HWI2C_PORT2_REG_BASE            (0x111A00)
#define HWI2C_PORT3_REG_BASE            (0x111B00)
#define HWI2C_PORT4_REG_BASE            (0x121C00)
#define HWI2C_PORT5_REG_BASE            (0x121D00)
#define HWI2C_PORT6_REG_BASE            (0x000B00)

//#################
//#
//#  For Non-PM HWI2C
//#
//STD mode
#define REG_HWI2C_MIIC_CFG              (HWI2C_REG_BASE+0x00*2)
    #define _MIIC_CFG_RESET             (__BIT0)
    #define _MIIC_CFG_EN_DMA            (__BIT1)
    #define _MIIC_CFG_EN_INT            (__BIT2)
    #define _MIIC_CFG_EN_CLKSTR         (__BIT3)
    #define _MIIC_CFG_EN_TMTINT         (__BIT4)
    #define _MIIC_CFG_EN_FILTER         (__BIT5)
    #define _MIIC_CFG_EN_PUSH1T         (__BIT6)
    #define _MIIC_CFG_RESERVED          (__BIT7)
#define REG_HWI2C_CMD_START             (HWI2C_REG_BASE+0x01*2)
    #define _CMD_START                  (__BIT0)
#define REG_HWI2C_CMD_STOP              (HWI2C_REG_BASE+0x01*2+1)
    #define _CMD_STOP                   (__BIT0)
#define REG_HWI2C_WDATA                 (HWI2C_REG_BASE+0x02*2)
#define REG_HWI2C_WDATA_GET             (HWI2C_REG_BASE+0x02*2+1)
    #define _WDATA_GET_ACKBIT           (__BIT0)
#define REG_HWI2C_RDATA                 (HWI2C_REG_BASE+0x03*2)
#define REG_HWI2C_RDATA_CFG             (HWI2C_REG_BASE+0x03*2+1)
    #define _RDATA_CFG_TRIG             (__BIT0)
    #define _RDATA_CFG_ACKBIT           (__BIT1)
#define REG_HWI2C_INT_CTL               (HWI2C_REG_BASE+0x04*2)
    #define _INT_CTL                    (__BIT0) //write this register to clear int
#define REG_HWI2C_CUR_STATE             (HWI2C_REG_BASE+0x05*2) //For Debug
    #define _CUR_STATE_MSK              (__BIT4|__BIT3|__BIT2|__BIT1|__BIT0)
#define REG_HWI2C_INT_STATUS            (HWI2C_REG_BASE+0x05*2+1) //For Debug
    #define _INT_STARTDET               (__BIT0)
    #define _INT_STOPDET                (__BIT1)
    #define _INT_RXDONE                 (__BIT2)
    #define _INT_TXDONE                 (__BIT3)
    #define _INT_CLKSTR                 (__BIT4)
    #define _INT_SCLERR                 (__BIT5)
#define REG_HWI2C_STP_CNT               (HWI2C_REG_BASE+0x08*2)
#define REG_HWI2C_CKH_CNT               (HWI2C_REG_BASE+0x09*2)
#define REG_HWI2C_CKL_CNT               (HWI2C_REG_BASE+0x0A*2)
#define REG_HWI2C_SDA_CNT               (HWI2C_REG_BASE+0x0B*2)
#define REG_HWI2C_STT_CNT               (HWI2C_REG_BASE+0x0C*2)
#define REG_HWI2C_LTH_CNT               (HWI2C_REG_BASE+0x0D*2)
#define REG_HWI2C_TMT_CNT               (HWI2C_REG_BASE+0x0E*2)
#define REG_HWI2C_SCLI_DELAY            (HWI2C_REG_BASE+0x0F*2)
    #define _SCLI_DELAY                 (__BIT2|__BIT1|__BIT0)

#define REG_HWI2C_RESERVE0              (HWI2C_REG_BASE+0x10*2)
#define REG_HWI2C_RESERVE1              (HWI2C_REG_BASE+0x10*2+1)
    #define _MIIC_RESET                 (__BIT0)

//DMA mode
#define REG_HWI2C_DMA_CFG               (HWI2C_REG_BASE+0x20*2)
    #define _DMA_CFG_RESET              (__BIT1)
    #define _DMA_CFG_INTEN              (__BIT2)
    #define _DMA_CFG_MIURST             (__BIT3)
    #define _DMA_CFG_MIUPRI             (__BIT4)
#define REG_HWI2C_DMA_MIU_ADR           (HWI2C_REG_BASE+0x21*2) // 4 bytes
#define REG_HWI2C_DMA_CTL               (HWI2C_REG_BASE+0x23*2)
//    #define _DMA_CTL_TRIG               (__BIT0)
//    #define _DMA_CTL_RETRIG             (__BIT1)
    #define _DMA_CTL_TXNOSTOP           (__BIT5) //miic transfer format, 1: S+data..., 0: S+data...+P
    #define _DMA_CTL_RDWTCMD            (__BIT6) //miic transfer format, 1:read, 0:write
    #define _DMA_CTL_MIUCHSEL           (__BIT7) //0: miu0, 1:miu1
#define REG_HWI2C_DMA_TXR               (HWI2C_REG_BASE+0x24*2)
    #define _DMA_TXR_DONE               (__BIT0)
#define REG_HWI2C_DMA_CMDDAT0           (HWI2C_REG_BASE+0x25*2) // 8 bytes
#define REG_HWI2C_DMA_CMDDAT1           (HWI2C_REG_BASE+0x25*2+1)
#define REG_HWI2C_DMA_CMDDAT2           (HWI2C_REG_BASE+0x26*2)
#define REG_HWI2C_DMA_CMDDAT3           (HWI2C_REG_BASE+0x26*2+1)
#define REG_HWI2C_DMA_CMDDAT4           (HWI2C_REG_BASE+0x27*2)
#define REG_HWI2C_DMA_CMDDAT5           (HWI2C_REG_BASE+0x27*2+1)
#define REG_HWI2C_DMA_CMDDAT6           (HWI2C_REG_BASE+0x28*2)
#define REG_HWI2C_DMA_CMDDAT7           (HWI2C_REG_BASE+0x28*2+1)
#define REG_HWI2C_DMA_CMDLEN            (HWI2C_REG_BASE+0x29*2)
    #define _DMA_CMDLEN_MSK             (__BIT2|__BIT1|__BIT0)
#define REG_HWI2C_DMA_DATLEN            (HWI2C_REG_BASE+0x2A*2) // 4 bytes
#define REG_HWI2C_DMA_TXFRCNT           (HWI2C_REG_BASE+0x2C*2) // 4 bytes
#define REG_HWI2C_DMA_SLVADR            (HWI2C_REG_BASE+0x2E*2)
    #define _DMA_SLVADR_10BIT_MSK       0x3FF //10 bits
    #define _DMA_SLVADR_NORML_MSK       0x7F //7 bits
#define REG_HWI2C_DMA_SLVCFG            (HWI2C_REG_BASE+0x2E*2+1)
    #define _DMA_10BIT_MODE             (__BIT2)
#define REG_HWI2C_DMA_CTL_TRIG          (HWI2C_REG_BASE+0x2F*2)
    #define _DMA_CTL_TRIG               (__BIT0)
#define REG_HWI2C_DMA_CTL_RETRIG        (HWI2C_REG_BASE+0x2F*2+1)
    #define _DMA_CTL_RETRIG             (__BIT0)

//#################
//#
//#  For PM HWI2C
//#
//STD mode
#define REG_HWI2C_MIIC_CFG_PM           (HWI2C_REG_BASE_PM+0x00*2)
    #define _MIIC_CFG_RESET             (__BIT0)
    #define _MIIC_CFG_EN_DMA            (__BIT1)
    #define _MIIC_CFG_EN_INT            (__BIT2)
    #define _MIIC_CFG_EN_CLKSTR         (__BIT3)
    #define _MIIC_CFG_EN_TMTINT         (__BIT4)
    #define _MIIC_CFG_EN_FILTER         (__BIT5)
    #define _MIIC_CFG_EN_PUSH1T         (__BIT6)
    #define _MIIC_CFG_RESERVED          (__BIT7)
#define REG_HWI2C_CMD_START_PM          (HWI2C_REG_BASE_PM+0x01*2)
    #define _CMD_START                  (__BIT0)
#define REG_HWI2C_CMD_STOP_PM           (HWI2C_REG_BASE_PM+0x01*2+1)
    #define _CMD_STOP                   (__BIT0)
#define REG_HWI2C_WDATA_PM              (HWI2C_REG_BASE_PM+0x02*2)
#define REG_HWI2C_WDATA_GET_PM          (HWI2C_REG_BASE_PM+0x02*2+1)
    #define _WDATA_GET_ACKBIT           (__BIT0)
#define REG_HWI2C_RDATA_PM              (HWI2C_REG_BASE_PM+0x03*2)
#define REG_HWI2C_RDATA_CFG_PM          (HWI2C_REG_BASE_PM+0x03*2+1)
    #define _RDATA_CFG_TRIG             (__BIT0)
    #define _RDATA_CFG_ACKBIT           (__BIT1)
#define REG_HWI2C_INT_CTL_PM            (HWI2C_REG_BASE_PM+0x04*2)
    #define _INT_CTL                    (__BIT0) //write this register to clear int
#define REG_HWI2C_CUR_STATE_PM          (HWI2C_REG_BASE_PM+0x05*2) //For Debug
    #define _CUR_STATE_MSK              (__BIT4|__BIT3|__BIT2|__BIT1|__BIT0)
#define REG_HWI2C_INT_STATUS_PM         (HWI2C_REG_BASE_PM+0x05*2+1) //For Debug
    #define _INT_STARTDET               (__BIT0)
    #define _INT_STOPDET                (__BIT1)
    #define _INT_RXDONE                 (__BIT2)
    #define _INT_TXDONE                 (__BIT3)
    #define _INT_CLKSTR                 (__BIT4)
    #define _INT_SCLERR                 (__BIT5)
#define REG_HWI2C_STP_CNT_PM            (HWI2C_REG_BASE_PM+0x08*2)
#define REG_HWI2C_CKH_CNT_PM            (HWI2C_REG_BASE_PM+0x09*2)
#define REG_HWI2C_CKL_CNT_PM            (HWI2C_REG_BASE_PM+0x0A*2)
#define REG_HWI2C_SDA_CNT_PM            (HWI2C_REG_BASE_PM+0x0B*2)
#define REG_HWI2C_STT_CNT_PM            (HWI2C_REG_BASE_PM+0x0C*2)
#define REG_HWI2C_LTH_CNT_PM            (HWI2C_REG_BASE_PM+0x0D*2)
#define REG_HWI2C_TMT_CNT_PM            (HWI2C_REG_BASE_PM+0x0E*2)
#define REG_HWI2C_SCLI_DELAY_PM         (HWI2C_REG_BASE_PM+0x0F*2)
    #define _SCLI_DELAY                 (__BIT2|__BIT1|__BIT0)

//DMA mode
#define REG_HWI2C_DMA_CFG_PM            (HWI2C_REG_BASE_PM+0x20*2)
    #define _DMA_CFG_RESET              (__BIT1)
    #define _DMA_CFG_INTEN              (__BIT2)
    #define _DMA_CFG_MIURST             (__BIT3)
    #define _DMA_CFG_MIUPRI             (__BIT4)
#define REG_HWI2C_DMA_MIU_ADR_PM        (HWI2C_REG_BASE_PM+0x21*2) // 4 bytes
#define REG_HWI2C_DMA_CTL_PM            (HWI2C_REG_BASE_PM+0x23*2)
    //    #define _DMA_CTL_TRIG               (__BIT0)
    //    #define _DMA_CTL_RETRIG             (__BIT1)
    #define _DMA_CTL_TXNOSTOP           (__BIT5) //miic transfer format, 1: S+data..., 0: S+data...+P
    #define _DMA_CTL_RDWTCMD            (__BIT6) //miic transfer format, 1:read, 0:write
    #define _DMA_CTL_MIUCHSEL           (__BIT7) //0: miu0, 1:miu1
#define REG_HWI2C_DMA_TXR_PM            (HWI2C_REG_BASE_PM+0x24*2)
    #define _DMA_TXR_DONE               (__BIT0)
#define REG_HWI2C_DMA_CMDDAT0_PM        (HWI2C_REG_BASE_PM+0x25*2) // 8 bytes
#define REG_HWI2C_DMA_CMDDAT1_PM        (HWI2C_REG_BASE_PM+0x25*2+1)
#define REG_HWI2C_DMA_CMDDAT2_PM        (HWI2C_REG_BASE_PM+0x26*2)
#define REG_HWI2C_DMA_CMDDAT3_PM        (HWI2C_REG_BASE_PM+0x26*2+1)
#define REG_HWI2C_DMA_CMDDAT4_PM        (HWI2C_REG_BASE_PM+0x27*2)
#define REG_HWI2C_DMA_CMDDAT5_PM        (HWI2C_REG_BASE_PM+0x27*2+1)
#define REG_HWI2C_DMA_CMDDAT6_PM        (HWI2C_REG_BASE_PM+0x28*2)
#define REG_HWI2C_DMA_CMDDAT7_PM        (HWI2C_REG_BASE_PM+0x28*2+1)
#define REG_HWI2C_DMA_CMDLEN_PM         (HWI2C_REG_BASE_PM+0x29*2)
    #define _DMA_CMDLEN_MSK             (__BIT2|__BIT1|__BIT0)
#define REG_HWI2C_DMA_DATLEN_PM         (HWI2C_REG_BASE_PM+0x2A*2) // 4 bytes
#define REG_HWI2C_DMA_TXFRCNT_PM        (HWI2C_REG_BASE_PM+0x2C*2) // 4 bytes
#define REG_HWI2C_DMA_SLVADR_PM         (HWI2C_REG_BASE_PM+0x2E*2)
    #define _DMA_SLVADR_10BIT_MSK       0x3FF //10 bits
    #define _DMA_SLVADR_NORML_MSK       0x7F //7 bits
#define REG_HWI2C_DMA_SLVCFG_PM         (HWI2C_REG_BASE_PM+0x2E*2+1)
    #define _DMA_10BIT_MODE             (__BIT2)
#define REG_HWI2C_DMA_CTL_TRIG_PM       (HWI2C_REG_BASE_PM+0x2F*2)
    #define _DMA_CTL_TRIG               (__BIT0)
#define REG_HWI2C_DMA_CTL_RETRIG_PM     (HWI2C_REG_BASE_PM+0x2F*2+1)
    #define _DMA_CTL_RETRIG             (__BIT0)

/*
#define REG_IIC_CTRL                0x00UL
#define REG_IIC_CLK_SEL             0x01UL
#define REG_IIC_WDATA               0x02UL
#define REG_IIC_RDATA               0x03UL
#define REG_IIC_STATUS              0x04UL                                // reset, clear and status

#define MHal_IIC_REG(addr)          (*(volatile U32*)(REG_IIC_BASE + ((addr)<<2)))

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_CHIP_BASE               0xFD203C00
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_CHIP_BASE           (mstar_pm_base + 0x00203C00UL)
#endif

#define REG_IIC_ALLPADIN            0x50UL
#define REG_IIC_MODE                0x57UL
#define REG_DDCR_GPIO_SEL           0x70UL
#define MHal_CHIP_REG(addr)         (*(volatile U32*)(REG_CHIP_BASE + ((addr)<<2)))


//the definitions of GPIO reg set to initialize
#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
#define REG_ARM_BASE                0xFD000000//Use 8 bit addressing
#elif defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_CHIP_BASE           (mstar_pm_base)
#endif

//#define REG_ALL_PAD_IN            ((0x0f50<<1) )   //set all pads (except SPI) as input
#define REG_ALL_PAD_IN              (0x101ea1UL)   //set all pads (except SPI) as input

//the definitions of GPIO reg set to make output
#define PAD_DDCR_CK                 173
#define REG_PAD_DDCR_CK_SET         (0x101eaeUL)
#define REG_PAD_DDCR_CK_OEN         (0x102b87UL)
#define REG_PAD_DDCR_CK_IN          (0x102b87UL)
#define REG_PAD_DDCR_CK_OUT         (0x102b87UL)
#define PAD_DDCR_DA                 172
#define REG_PAD_DDCR_DA_SET         (0x101eaeUL)
#define REG_PAD_DDCR_DA_OEN         (0x102b86UL)
#define REG_PAD_DDCR_DA_IN          (0x102b86UL)
#define REG_PAD_DDCR_DA_OUT         (0x102b86UL)

#define PAD_TGPIO2 181
#define REG_PAD_TGPIO2_SET
#define REG_PAD_TGPIO2_OEN (0x102b8fUL)
#define REG_PAD_TGPIO2_IN (0x102b8fUL)
#define REG_PAD_TGPIO2_OUT (0x102b8fUL)
#define PAD_TGPIO3 182
#define REG_PAD_TGPIO3_SET
#define REG_PAD_TGPIO3_OEN (0x102b90UL)
#define REG_PAD_TGPIO3_IN (0x102b90UL)
#define REG_PAD_TGPIO3_OUT (0x102b90UL)

#define PAD_I2S_OUT_SD1 101
#define REG_PAD_I2S_OUT_SD1_SET
#define REG_PAD_I2S_OUT_SD1_OEN (0x102b3fUL)
#define REG_PAD_I2S_OUT_SD1_IN (0x102b3fUL)
#define REG_PAD_I2S_OUT_SD1_OUT (0x102b3fUL)
#define PAD_SPDIF_IN 95
#define REG_PAD_SPDIF_IN_SET
#define REG_PAD_SPDIF_IN_OEN (0x102b39UL)
#define REG_PAD_SPDIF_IN_IN (0x102b39UL)
#define REG_PAD_SPDIF_IN_OUT (0x102b39UL)

#define PAD_I2S_IN_WS 92
#define REG_PAD_I2S_IN_WS_SET
#define REG_PAD_I2S_IN_WS_OEN (0x102b36UL)
#define REG_PAD_I2S_IN_WS_IN (0x102b36UL)
#define REG_PAD_I2S_IN_WS_OUT (0x102b36UL)
#define PAD_I2S_IN_BCK 93
#define REG_PAD_I2S_IN_BCK_SET
#define REG_PAD_I2S_IN_BCK_OEN (0x102b37UL)
#define REG_PAD_I2S_IN_BCK_IN (0x102b37UL)
#define REG_PAD_I2S_IN_BCK_OUT (0x102b37UL)

#define PAD_I2S_OUT_SD3 103
#define REG_PAD_I2S_OUT_SD3_SET
#define REG_PAD_I2S_OUT_SD3_OEN (0x102b41UL)
#define REG_PAD_I2S_OUT_SD3_IN (0x102b41UL)
#define REG_PAD_I2S_OUT_SD3_OUT (0x102b41UL)
#define PAD_I2S_OUT_SD2 102
#define REG_PAD_I2S_OUT_SD2_SET
#define REG_PAD_I2S_OUT_SD2_OEN (0x102b40UL)
#define REG_PAD_I2S_OUT_SD2_IN (0x102b40UL)
#define REG_PAD_I2S_OUT_SD2_OUT (0x102b40UL)

#define PAD_GPIO_PM9 9
#define REG_PAD_GPIO_PM9_SET
#define REG_PAD_GPIO_PM9_OEN (0x0f12UL)
#define REG_PAD_GPIO_PM9_IN (0x0f12UL)
#define REG_PAD_GPIO_PM9_OUT (0x0f12UL)
#define PAD_GPIO_PM8 8
#define REG_PAD_GPIO_PM8_SET
#define REG_PAD_GPIO_PM8_OEN (0x0f10UL)
#define REG_PAD_GPIO_PM8_IN (0x0f10UL)
#define REG_PAD_GPIO_PM8_OUT (0x0f10UL)

#define MHal_GPIO_REG(addr)             (*(volatile U8*)(REG_ARM_BASE + (((addr) & ~1)<<1) + (addr & 1)))
//-------------------------------------------------------------------------------------------------
//  Type and Structure
//-------------------------------------------------------------------------------------------------

typedef struct
{
    U32 SclOenReg;
    U8  SclOenBit;

    U32 SclOutReg;
    U8  SclOutBit;

    U32 SclInReg;
    U8  SclInBit;

    U32 SdaOenReg;
    U8  SdaOenBit;

    U32 SdaOutReg;
    U8  SdaOutBit;

    U32 SdaInReg;
    U8  SdaInBit;

    U8  DefDelay;
}IIC_Bus_t;

*/

#endif // _REG_IIC_H_

