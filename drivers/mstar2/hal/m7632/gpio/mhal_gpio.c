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

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
//#include "MsCommon.h"
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <asm/io.h>

#include "mhal_gpio.h"
#include "mhal_gpio_reg.h"


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#define _CONCAT( a, b )     a##b
#define CONCAT( a, b )      _CONCAT( a, b )

/*
#define BIT0    BIT(0)
#define BIT1    BIT(1)
#define BIT2    BIT(2)
#define BIT3    BIT(3)
#define BIT4    BIT(4)
#define BIT5    BIT(5)
#define BIT6    BIT(6)
#define BIT7    BIT(7)
*/

// Dummy
#define GPIO999_OEN     0, 0
#define GPIO999_OUT     0, 0
#define GPIO999_IN      0, 0

//[UTOPIA_GPIO_INFO] RobotS_AutoCheckStart
#define GPIO1_PAD PAD_DDCA_CK
#define GPIO1_OEN 0x0494, BIT1
#define GPIO1_OUT 0x0494, BIT2
#define GPIO1_IN  0x0494, BIT0

#define GPIO2_PAD PAD_DDCA_DA
#define GPIO2_OEN 0x0494, BIT5
#define GPIO2_OUT 0x0494, BIT6
#define GPIO2_IN  0x0494, BIT4

#define GPIO3_PAD PAD_IRIN
#define GPIO3_OEN 0x0f26, BIT2
#define GPIO3_OUT 0x0f26, BIT2
#define GPIO3_IN  0x0f26, BIT2

#define GPIO4_PAD PAD_PWM_PM
#define GPIO4_OEN 0x0f28, BIT0
#define GPIO4_OUT 0x0f28, BIT1
#define GPIO4_IN  0x0f28, BIT2

#define GPIO5_PAD PAD_CEC0
#define GPIO5_OEN 0x0f2a, BIT0
#define GPIO5_OUT 0x0f2a, BIT1
#define GPIO5_IN  0x0f2a, BIT2

#define GPIO6_PAD PAD_GPIO0_PM
#define GPIO6_OEN 0x0f00, BIT0
#define GPIO6_OUT 0x0f00, BIT1
#define GPIO6_IN  0x0f00, BIT2

#define GPIO7_PAD PAD_GPIO1_PM
#define GPIO7_OEN 0x0f02, BIT0
#define GPIO7_OUT 0x0f02, BIT1
#define GPIO7_IN  0x0f02, BIT2

#define GPIO8_PAD PAD_GPIO2_PM
#define GPIO8_OEN 0x0f04, BIT0
#define GPIO8_OUT 0x0f04, BIT1
#define GPIO8_IN  0x0f04, BIT2

#define GPIO9_PAD PAD_USB_CTRL
#define GPIO9_OEN 0x0f06, BIT0
#define GPIO9_OUT 0x0f06, BIT1
#define GPIO9_IN  0x0f06, BIT2

#define GPIO10_PAD PAD_GPIO5_PM
#define GPIO10_OEN 0x0f0a, BIT0
#define GPIO10_OUT 0x0f0a, BIT1
#define GPIO10_IN  0x0f0a, BIT2

#define GPIO11_PAD PAD_GPIO6_PM
#define GPIO11_OEN 0x0f0c, BIT0
#define GPIO11_OUT 0x0f0c, BIT1
#define GPIO11_IN  0x0f0c, BIT2

#define GPIO12_PAD PAD_GPIO7_PM
#define GPIO12_OEN 0x0f0e, BIT0
#define GPIO12_OUT 0x0f0e, BIT1
#define GPIO12_IN  0x0f0e, BIT2

#define GPIO13_PAD PAD_GPIO8_PM
#define GPIO13_OEN 0x0f10, BIT0
#define GPIO13_OUT 0x0f10, BIT1
#define GPIO13_IN  0x0f10, BIT2

#define GPIO14_PAD PAD_GPIO9_PM
#define GPIO14_OEN 0x0f12, BIT0
#define GPIO14_OUT 0x0f12, BIT1
#define GPIO14_IN  0x0f12, BIT2

#define GPIO15_PAD PAD_GPIO10_PM
#define GPIO15_OEN 0x0f14, BIT0
#define GPIO15_OUT 0x0f14, BIT1
#define GPIO15_IN  0x0f14, BIT2

#define GPIO16_PAD PAD_GPIO11_PM
#define GPIO16_OEN 0x0f16, BIT0
#define GPIO16_OUT 0x0f16, BIT1
#define GPIO16_IN  0x0f16, BIT2

#define GPIO17_PAD PAD_GPIO12_PM
#define GPIO17_OEN 0x0f18, BIT0
#define GPIO17_OUT 0x0f18, BIT1
#define GPIO17_IN  0x0f18, BIT2

#define GPIO18_PAD PAD_GPIO13_PM
#define GPIO18_OEN 0x0f1a, BIT0
#define GPIO18_OUT 0x0f1a, BIT1
#define GPIO18_IN  0x0f1a, BIT2

#define GPIO19_PAD PAD_GPIO14_PM
#define GPIO19_OEN 0x0f1c, BIT0
#define GPIO19_OUT 0x0f1c, BIT1
#define GPIO19_IN  0x0f1c, BIT2

#define GPIO20_PAD PAD_GPIO15_PM
#define GPIO20_OEN 0x0f1e, BIT0
#define GPIO20_OUT 0x0f1e, BIT1
#define GPIO20_IN  0x0f1e, BIT2

#define GPIO21_PAD PAD_GPIO16_PM
#define GPIO21_OEN 0x0f20, BIT0
#define GPIO21_OUT 0x0f20, BIT1
#define GPIO21_IN  0x0f20, BIT2

#define GPIO22_PAD PAD_GPIO17_PM
#define GPIO22_OEN 0x0f48, BIT0
#define GPIO22_OUT 0x0f48, BIT1
#define GPIO22_IN  0x0f48, BIT2

#define GPIO23_PAD PAD_GPIO18_PM
#define GPIO23_OEN 0x0f4a, BIT0
#define GPIO23_OUT 0x0f4a, BIT1
#define GPIO23_IN  0x0f4a, BIT2

#define GPIO24_PAD PAD_GPIO19_PM
#define GPIO24_OEN 0x0f4c, BIT0
#define GPIO24_OUT 0x0f4c, BIT1
#define GPIO24_IN  0x0f4c, BIT2

#define GPIO25_PAD PAD_GPIO20_PM
#define GPIO25_OEN 0x0f4e, BIT0
#define GPIO25_OUT 0x0f4e, BIT1
#define GPIO25_IN  0x0f4e, BIT2

#define GPIO26_PAD PAD_GPIO21_PM
#define GPIO26_OEN 0x0f50, BIT0
#define GPIO26_OUT 0x0f50, BIT1
#define GPIO26_IN  0x0f50, BIT2

#define GPIO27_PAD PAD_GPIO22_PM
#define GPIO27_OEN 0x0f52, BIT0
#define GPIO27_OUT 0x0f52, BIT1
#define GPIO27_IN  0x0f52, BIT2

#define GPIO28_PAD PAD_GPIO23_PM
#define GPIO28_OEN 0x0f54, BIT0
#define GPIO28_OUT 0x0f54, BIT1
#define GPIO28_IN  0x0f54, BIT2

#define GPIO29_PAD PAD_GPIO24_PM
#define GPIO29_OEN 0x0f56, BIT0
#define GPIO29_OUT 0x0f56, BIT1
#define GPIO29_IN  0x0f56, BIT2

#define GPIO30_PAD PAD_GPIO25_PM
#define GPIO30_OEN 0x0f58, BIT0
#define GPIO30_OUT 0x0f58, BIT1
#define GPIO30_IN  0x0f58, BIT2

#define GPIO31_PAD PAD_GPIO26_PM
#define GPIO31_OEN 0x0f5a, BIT0
#define GPIO31_OUT 0x0f5a, BIT1
#define GPIO31_IN  0x0f5a, BIT2

#define GPIO32_PAD PAD_GPIO27_PM
#define GPIO32_OEN 0x0f5c, BIT0
#define GPIO32_OUT 0x0f5c, BIT1
#define GPIO32_IN  0x0f5c, BIT2

#define GPIO33_PAD PAD_GPIO28_PM
#define GPIO33_OEN 0x0f5e, BIT0
#define GPIO33_OUT 0x0f5e, BIT1
#define GPIO33_IN  0x0f5e, BIT2

#define GPIO34_PAD PAD_DDCDA_CK
#define GPIO34_OEN 0x0496, BIT1
#define GPIO34_OUT 0x0496, BIT2
#define GPIO34_IN  0x0496, BIT0

#define GPIO35_PAD PAD_DDCDA_DA
#define GPIO35_OEN 0x0496, BIT5
#define GPIO35_OUT 0x0496, BIT6
#define GPIO35_IN  0x0496, BIT4

#define GPIO36_PAD PAD_DDCDB_CK
#define GPIO36_OEN 0x0497, BIT1
#define GPIO36_OUT 0x0497, BIT2
#define GPIO36_IN  0x0497, BIT0

#define GPIO37_PAD PAD_DDCDB_DA
#define GPIO37_OEN 0x0497, BIT5
#define GPIO37_OUT 0x0497, BIT6
#define GPIO37_IN  0x0497, BIT4

#define GPIO38_PAD PAD_DDCDC_CK
#define GPIO38_OEN 0x0498, BIT1
#define GPIO38_OUT 0x0498, BIT2
#define GPIO38_IN  0x0498, BIT0

#define GPIO39_PAD PAD_DDCDC_DA
#define GPIO39_OEN 0x0498, BIT5
#define GPIO39_OUT 0x0498, BIT6
#define GPIO39_IN  0x0498, BIT4

#define GPIO40_PAD PAD_DDCDD_CK
#define GPIO40_OEN 0x0499, BIT1
#define GPIO40_OUT 0x0499, BIT2
#define GPIO40_IN  0x0499, BIT0

#define GPIO41_PAD PAD_DDCDD_DA
#define GPIO41_OEN 0x0499, BIT5
#define GPIO41_OUT 0x0499, BIT6
#define GPIO41_IN  0x0499, BIT4

#define GPIO42_PAD PAD_SAR0
#define GPIO42_OEN 0x1423, BIT0
#define GPIO42_OUT 0x1424, BIT0
#define GPIO42_IN  0x1425, BIT0

#define GPIO43_PAD PAD_SAR1
#define GPIO43_OEN 0x1423, BIT1
#define GPIO43_OUT 0x1424, BIT1
#define GPIO43_IN  0x1425, BIT1

#define GPIO44_PAD PAD_SAR2
#define GPIO44_OEN 0x1423, BIT2
#define GPIO44_OUT 0x1424, BIT2
#define GPIO44_IN  0x1425, BIT2

#define GPIO45_PAD PAD_SAR3
#define GPIO45_OEN 0x1423, BIT3
#define GPIO45_OUT 0x1424, BIT3
#define GPIO45_IN  0x1425, BIT3

#define GPIO46_PAD PAD_SAR4
#define GPIO46_OEN 0x1423, BIT4
#define GPIO46_OUT 0x1424, BIT4
#define GPIO46_IN  0x1425, BIT4

#define GPIO47_PAD PAD_VPLUGIN
#define GPIO47_OEN 0x1423, BIT5
#define GPIO47_OUT 0x1424, BIT5
#define GPIO47_IN  0x1425, BIT5

#define GPIO48_PAD PAD_SAR6
#define GPIO48_OEN 0x1423, BIT6
#define GPIO48_OUT 0x1424, BIT6
#define GPIO48_IN  0x1425, BIT6

#define GPIO49_PAD PAD_SAR7
#define GPIO49_OEN 0x1423, BIT7
#define GPIO49_OUT 0x1424, BIT7
#define GPIO49_IN  0x1425, BIT7

#define GPIO50_PAD PAD_VID0
#define GPIO50_OEN 0x2e84, BIT1
#define GPIO50_OUT 0x2e84, BIT0
#define GPIO50_IN  0x2e84, BIT2

#define GPIO51_PAD PAD_VID1
#define GPIO51_OEN 0x2e85, BIT1
#define GPIO51_OUT 0x2e85, BIT0
#define GPIO51_IN  0x2e85, BIT2

#define GPIO52_PAD PAD_VID2
#define GPIO52_OEN 0x0f22, BIT0
#define GPIO52_OUT 0x0f22, BIT1
#define GPIO52_IN  0x0f22, BIT2

#define GPIO53_PAD PAD_WOL_INT_OUT
#define GPIO53_OEN 0x2e82, BIT1
#define GPIO53_OUT 0x2e82, BIT0
#define GPIO53_IN  0x2e82, BIT2

#define GPIO54_PAD PAD_DDCR_CK
#define GPIO54_OEN 0x322ee2, BIT1
#define GPIO54_OUT 0x322ee2, BIT0
#define GPIO54_IN  0x322ee2, BIT2

#define GPIO55_PAD PAD_DDCR_DA
#define GPIO55_OEN 0x322ee0, BIT1
#define GPIO55_OUT 0x322ee0, BIT0
#define GPIO55_IN  0x322ee0, BIT2

#define GPIO56_PAD PAD_DIM0
#define GPIO56_OEN 0x322f40, BIT1
#define GPIO56_OUT 0x322f40, BIT0
#define GPIO56_IN  0x322f40, BIT2

#define GPIO57_PAD PAD_DIM1
#define GPIO57_OEN 0x322f42, BIT1
#define GPIO57_OUT 0x322f42, BIT0
#define GPIO57_IN  0x322f42, BIT2

#define GPIO58_PAD PAD_DIM2
#define GPIO58_OEN 0x322f44, BIT1
#define GPIO58_OUT 0x322f44, BIT0
#define GPIO58_IN  0x322f44, BIT2

#define GPIO59_PAD PAD_DIM3
#define GPIO59_OEN 0x322f46, BIT1
#define GPIO59_OUT 0x322f46, BIT0
#define GPIO59_IN  0x322f46, BIT2

#define GPIO60_PAD PAD_GPIO0
#define GPIO60_OEN 0x322b00, BIT1
#define GPIO60_OUT 0x322b00, BIT0
#define GPIO60_IN  0x322b00, BIT2

#define GPIO61_PAD PAD_GPIO1
#define GPIO61_OEN 0x322b02, BIT1
#define GPIO61_OUT 0x322b02, BIT0
#define GPIO61_IN  0x322b02, BIT2

#define GPIO62_PAD PAD_GPIO8
#define GPIO62_OEN 0x322b10, BIT1
#define GPIO62_OUT 0x322b10, BIT0
#define GPIO62_IN  0x322b10, BIT2

#define GPIO63_PAD PAD_GPIO9
#define GPIO63_OEN 0x322b12, BIT1
#define GPIO63_OUT 0x322b12, BIT0
#define GPIO63_IN  0x322b12, BIT2

#define GPIO64_PAD PAD_GPIO10
#define GPIO64_OEN 0x322b14, BIT1
#define GPIO64_OUT 0x322b14, BIT0
#define GPIO64_IN  0x322b14, BIT2

#define GPIO65_PAD PAD_GPIO11
#define GPIO65_OEN 0x322b16, BIT1
#define GPIO65_OUT 0x322b16, BIT0
#define GPIO65_IN  0x322b16, BIT2

#define GPIO66_PAD PAD_GPIO12
#define GPIO66_OEN 0x322b18, BIT1
#define GPIO66_OUT 0x322b18, BIT0
#define GPIO66_IN  0x322b18, BIT2

#define GPIO67_PAD PAD_GPIO13
#define GPIO67_OEN 0x322b1a, BIT1
#define GPIO67_OUT 0x322b1a, BIT0
#define GPIO67_IN  0x322b1a, BIT2

#define GPIO68_PAD PAD_GPIO14
#define GPIO68_OEN 0x322b1c, BIT1
#define GPIO68_OUT 0x322b1c, BIT0
#define GPIO68_IN  0x322b1c, BIT2

#define GPIO69_PAD PAD_GPIO15
#define GPIO69_OEN 0x322b1e, BIT1
#define GPIO69_OUT 0x322b1e, BIT0
#define GPIO69_IN  0x322b1e, BIT2

#define GPIO70_PAD PAD_GPIO16
#define GPIO70_OEN 0x322b20, BIT1
#define GPIO70_OUT 0x322b20, BIT0
#define GPIO70_IN  0x322b20, BIT2

#define GPIO71_PAD PAD_GPIO17
#define GPIO71_OEN 0x322b22, BIT1
#define GPIO71_OUT 0x322b22, BIT0
#define GPIO71_IN  0x322b22, BIT2

#define GPIO72_PAD PAD_GPIO18
#define GPIO72_OEN 0x322b24, BIT1
#define GPIO72_OUT 0x322b24, BIT0
#define GPIO72_IN  0x322b24, BIT2

#define GPIO73_PAD PAD_I2S_IN_BCK
#define GPIO73_OEN 0x322e00, BIT1
#define GPIO73_OUT 0x322e00, BIT0
#define GPIO73_IN  0x322e00, BIT2

#define GPIO74_PAD PAD_I2S_IN_DIN0
#define GPIO74_OEN 0x322e04, BIT1
#define GPIO74_OUT 0x322e04, BIT0
#define GPIO74_IN  0x322e04, BIT2

#define GPIO75_PAD PAD_I2S_IN_DIN1
#define GPIO75_OEN 0x322e06, BIT1
#define GPIO75_OUT 0x322e06, BIT0
#define GPIO75_IN  0x322e06, BIT2

#define GPIO76_PAD PAD_I2S_IN_DIN2
#define GPIO76_OEN 0x322e08, BIT1
#define GPIO76_OUT 0x322e08, BIT0
#define GPIO76_IN  0x322e08, BIT2

#define GPIO77_PAD PAD_I2S_IN_WCK
#define GPIO77_OEN 0x322e02, BIT1
#define GPIO77_OUT 0x322e02, BIT0
#define GPIO77_IN  0x322e02, BIT2

#define GPIO78_PAD PAD_I2S_OUT_BCK
#define GPIO78_OEN 0x322e20, BIT1
#define GPIO78_OUT 0x322e20, BIT0
#define GPIO78_IN  0x322e20, BIT2

#define GPIO79_PAD PAD_I2S_OUT_MCK
#define GPIO79_OEN 0x322e22, BIT1
#define GPIO79_OUT 0x322e22, BIT0
#define GPIO79_IN  0x322e22, BIT2

#define GPIO80_PAD PAD_I2S_OUT_SD0
#define GPIO80_OEN 0x322e26, BIT1
#define GPIO80_OUT 0x322e26, BIT0
#define GPIO80_IN  0x322e26, BIT2

#define GPIO81_PAD PAD_I2S_OUT_SD1
#define GPIO81_OEN 0x322e28, BIT1
#define GPIO81_OUT 0x322e28, BIT0
#define GPIO81_IN  0x322e28, BIT2

#define GPIO82_PAD PAD_I2S_OUT_SD2
#define GPIO82_OEN 0x322e2a, BIT1
#define GPIO82_OUT 0x322e2a, BIT0
#define GPIO82_IN  0x322e2a, BIT2

#define GPIO83_PAD PAD_I2S_OUT_WCK
#define GPIO83_OEN 0x322e24, BIT1
#define GPIO83_OUT 0x322e24, BIT0
#define GPIO83_IN  0x322e24, BIT2

#define GPIO84_PAD PAD_LD_SPI_CK
#define GPIO84_OEN 0x322f26, BIT1
#define GPIO84_OUT 0x322f26, BIT0
#define GPIO84_IN  0x322f26, BIT2

#define GPIO85_PAD PAD_LD_SPI_CS
#define GPIO85_OEN 0x322f22, BIT1
#define GPIO85_OUT 0x322f22, BIT0
#define GPIO85_IN  0x322f22, BIT2

#define GPIO86_PAD PAD_LD_SPI_MISO
#define GPIO86_OEN 0x322f20, BIT1
#define GPIO86_OUT 0x322f20, BIT0
#define GPIO86_IN  0x322f20, BIT2

#define GPIO87_PAD PAD_LD_SPI_MOSI
#define GPIO87_OEN 0x322f24, BIT1
#define GPIO87_OUT 0x322f24, BIT0
#define GPIO87_IN  0x322f24, BIT2

#define GPIO88_PAD PAD_MIC_BCK
#define GPIO88_OEN 0x322e40, BIT1
#define GPIO88_OUT 0x322e40, BIT0
#define GPIO88_IN  0x322e40, BIT2

#define GPIO89_PAD PAD_MIC_SD0
#define GPIO89_OEN 0x322e42, BIT1
#define GPIO89_OUT 0x322e42, BIT0
#define GPIO89_IN  0x322e42, BIT2

#define GPIO90_PAD PAD_MIC_SD1
#define GPIO90_OEN 0x322e44, BIT1
#define GPIO90_OUT 0x322e44, BIT0
#define GPIO90_IN  0x322e44, BIT2

#define GPIO91_PAD PAD_MIC_SD2
#define GPIO91_OEN 0x322e46, BIT1
#define GPIO91_OUT 0x322e46, BIT0
#define GPIO91_IN  0x322e46, BIT2

#define GPIO92_PAD PAD_PCM2_CD_N
#define GPIO92_OEN 0x322d66, BIT1
#define GPIO92_OUT 0x322d66, BIT0
#define GPIO92_IN  0x322d66, BIT2

#define GPIO93_PAD PAD_PCM2_CE_N
#define GPIO93_OEN 0x322d68, BIT1
#define GPIO93_OUT 0x322d68, BIT0
#define GPIO93_IN  0x322d68, BIT2

#define GPIO94_PAD PAD_PCM2_IRQA_N
#define GPIO94_OEN 0x322d62, BIT1
#define GPIO94_OUT 0x322d62, BIT0
#define GPIO94_IN  0x322d62, BIT2

#define GPIO95_PAD PAD_PCM2_RESET
#define GPIO95_OEN 0x322d60, BIT1
#define GPIO95_OUT 0x322d60, BIT0
#define GPIO95_IN  0x322d60, BIT2

#define GPIO96_PAD PAD_PCM2_WAIT_N
#define GPIO96_OEN 0x322d64, BIT1
#define GPIO96_OUT 0x322d64, BIT0
#define GPIO96_IN  0x322d64, BIT2

#define GPIO97_PAD PAD_PCM_A0
#define GPIO97_OEN 0x322d00, BIT1
#define GPIO97_OUT 0x322d00, BIT0
#define GPIO97_IN  0x322d00, BIT2

#define GPIO98_PAD PAD_PCM_A1
#define GPIO98_OEN 0x322d02, BIT1
#define GPIO98_OUT 0x322d02, BIT0
#define GPIO98_IN  0x322d02, BIT2

#define GPIO99_PAD PAD_PCM_A2
#define GPIO99_OEN 0x322d04, BIT1
#define GPIO99_OUT 0x322d04, BIT0
#define GPIO99_IN  0x322d04, BIT2

#define GPIO100_PAD PAD_PCM_A3
#define GPIO100_OEN 0x322d06, BIT1
#define GPIO100_OUT 0x322d06, BIT0
#define GPIO100_IN  0x322d06, BIT2

#define GPIO101_PAD PAD_PCM_A4
#define GPIO101_OEN 0x322d08, BIT1
#define GPIO101_OUT 0x322d08, BIT0
#define GPIO101_IN  0x322d08, BIT2

#define GPIO102_PAD PAD_PCM_A5
#define GPIO102_OEN 0x322d0a, BIT1
#define GPIO102_OUT 0x322d0a, BIT0
#define GPIO102_IN  0x322d0a, BIT2

#define GPIO103_PAD PAD_PCM_A6
#define GPIO103_OEN 0x322d0c, BIT1
#define GPIO103_OUT 0x322d0c, BIT0
#define GPIO103_IN  0x322d0c, BIT2

#define GPIO104_PAD PAD_PCM_A7
#define GPIO104_OEN 0x322d0e, BIT1
#define GPIO104_OUT 0x322d0e, BIT0
#define GPIO104_IN  0x322d0e, BIT2

#define GPIO105_PAD PAD_PCM_A8
#define GPIO105_OEN 0x322d10, BIT1
#define GPIO105_OUT 0x322d10, BIT0
#define GPIO105_IN  0x322d10, BIT2

#define GPIO106_PAD PAD_PCM_A9
#define GPIO106_OEN 0x322d12, BIT1
#define GPIO106_OUT 0x322d12, BIT0
#define GPIO106_IN  0x322d12, BIT2

#define GPIO107_PAD PAD_PCM_A10
#define GPIO107_OEN 0x322d14, BIT1
#define GPIO107_OUT 0x322d14, BIT0
#define GPIO107_IN  0x322d14, BIT2

#define GPIO108_PAD PAD_PCM_A11
#define GPIO108_OEN 0x322d16, BIT1
#define GPIO108_OUT 0x322d16, BIT0
#define GPIO108_IN  0x322d16, BIT2

#define GPIO109_PAD PAD_PCM_A12
#define GPIO109_OEN 0x322d18, BIT1
#define GPIO109_OUT 0x322d18, BIT0
#define GPIO109_IN  0x322d18, BIT2

#define GPIO110_PAD PAD_PCM_A13
#define GPIO110_OEN 0x322d1a, BIT1
#define GPIO110_OUT 0x322d1a, BIT0
#define GPIO110_IN  0x322d1a, BIT2

#define GPIO111_PAD PAD_PCM_A14
#define GPIO111_OEN 0x322d1c, BIT1
#define GPIO111_OUT 0x322d1c, BIT0
#define GPIO111_IN  0x322d1c, BIT2

#define GPIO112_PAD PAD_PCM_CD_N
#define GPIO112_OEN 0x322d50, BIT1
#define GPIO112_OUT 0x322d50, BIT0
#define GPIO112_IN  0x322d50, BIT2

#define GPIO113_PAD PAD_PCM_CE_N
#define GPIO113_OEN 0x322d52, BIT1
#define GPIO113_OUT 0x322d52, BIT0
#define GPIO113_IN  0x322d52, BIT2

#define GPIO114_PAD PAD_PCM_D0
#define GPIO114_OEN 0x322d20, BIT1
#define GPIO114_OUT 0x322d20, BIT0
#define GPIO114_IN  0x322d20, BIT2

#define GPIO115_PAD PAD_PCM_D1
#define GPIO115_OEN 0x322d22, BIT1
#define GPIO115_OUT 0x322d22, BIT0
#define GPIO115_IN  0x322d22, BIT2

#define GPIO116_PAD PAD_PCM_D2
#define GPIO116_OEN 0x322d24, BIT1
#define GPIO116_OUT 0x322d24, BIT0
#define GPIO116_IN  0x322d24, BIT2

#define GPIO117_PAD PAD_PCM_D3
#define GPIO117_OEN 0x322d26, BIT1
#define GPIO117_OUT 0x322d26, BIT0
#define GPIO117_IN  0x322d26, BIT2

#define GPIO118_PAD PAD_PCM_D4
#define GPIO118_OEN 0x322d28, BIT1
#define GPIO118_OUT 0x322d28, BIT0
#define GPIO118_IN  0x322d28, BIT2

#define GPIO119_PAD PAD_PCM_D5
#define GPIO119_OEN 0x322d2a, BIT1
#define GPIO119_OUT 0x322d2a, BIT0
#define GPIO119_IN  0x322d2a, BIT2

#define GPIO120_PAD PAD_PCM_D6
#define GPIO120_OEN 0x322d2c, BIT1
#define GPIO120_OUT 0x322d2c, BIT0
#define GPIO120_IN  0x322d2c, BIT2

#define GPIO121_PAD PAD_PCM_D7
#define GPIO121_OEN 0x322d2e, BIT1
#define GPIO121_OUT 0x322d2e, BIT0
#define GPIO121_IN  0x322d2e, BIT2

#define GPIO122_PAD PAD_PCM_IORD_N
#define GPIO122_OEN 0x322d44, BIT1
#define GPIO122_OUT 0x322d44, BIT0
#define GPIO122_IN  0x322d44, BIT2

#define GPIO123_PAD PAD_PCM_IOWR_N
#define GPIO123_OEN 0x322d46, BIT1
#define GPIO123_OUT 0x322d46, BIT0
#define GPIO123_IN  0x322d46, BIT2

#define GPIO124_PAD PAD_PCM_IRQA_N
#define GPIO124_OEN 0x322d4a, BIT1
#define GPIO124_OUT 0x322d4a, BIT0
#define GPIO124_IN  0x322d4a, BIT2

#define GPIO125_PAD PAD_PCM_OE_N
#define GPIO125_OEN 0x322d42, BIT1
#define GPIO125_OUT 0x322d42, BIT0
#define GPIO125_IN  0x322d42, BIT2

#define GPIO126_PAD PAD_PCM_REG_N
#define GPIO126_OEN 0x322d4e, BIT1
#define GPIO126_OUT 0x322d4e, BIT0
#define GPIO126_IN  0x322d4e, BIT2

#define GPIO127_PAD PAD_PCM_RESET
#define GPIO127_OEN 0x322d40, BIT1
#define GPIO127_OUT 0x322d40, BIT0
#define GPIO127_IN  0x322d40, BIT2

#define GPIO128_PAD PAD_PCM_WAIT_N
#define GPIO128_OEN 0x322d4c, BIT1
#define GPIO128_OUT 0x322d4c, BIT0
#define GPIO128_IN  0x322d4c, BIT2

#define GPIO129_PAD PAD_PCM_WE_N
#define GPIO129_OEN 0x322d48, BIT1
#define GPIO129_OUT 0x322d48, BIT0
#define GPIO129_IN  0x322d48, BIT2

#define GPIO130_PAD PAD_PWM0
#define GPIO130_OEN 0x322f60, BIT1
#define GPIO130_OUT 0x322f60, BIT0
#define GPIO130_IN  0x322f60, BIT2

#define GPIO131_PAD PAD_PWM1
#define GPIO131_OEN 0x322f62, BIT1
#define GPIO131_OUT 0x322f62, BIT0
#define GPIO131_IN  0x322f62, BIT2

#define GPIO132_PAD PAD_PWM2
#define GPIO132_OEN 0x322f64, BIT1
#define GPIO132_OUT 0x322f64, BIT0
#define GPIO132_IN  0x322f64, BIT2

#define GPIO133_PAD PAD_SPDIF_IN
#define GPIO133_OEN 0x322e62, BIT1
#define GPIO133_OUT 0x322e62, BIT0
#define GPIO133_IN  0x322e62, BIT2

#define GPIO134_PAD PAD_SPDIF_OUT
#define GPIO134_OEN 0x322e60, BIT1
#define GPIO134_OUT 0x322e60, BIT0
#define GPIO134_IN  0x322e60, BIT2

#define GPIO135_PAD PAD_TCON0
#define GPIO135_OEN 0x322f00, BIT1
#define GPIO135_OUT 0x322f00, BIT0
#define GPIO135_IN  0x322f00, BIT2

#define GPIO136_PAD PAD_TCON1
#define GPIO136_OEN 0x322f02, BIT1
#define GPIO136_OUT 0x322f02, BIT0
#define GPIO136_IN  0x322f02, BIT2

#define GPIO137_PAD PAD_TCON2
#define GPIO137_OEN 0x322f04, BIT1
#define GPIO137_OUT 0x322f04, BIT0
#define GPIO137_IN  0x322f04, BIT2

#define GPIO138_PAD PAD_TCON3
#define GPIO138_OEN 0x322f06, BIT1
#define GPIO138_OUT 0x322f06, BIT0
#define GPIO138_IN  0x322f06, BIT2

#define GPIO139_PAD PAD_TCON4
#define GPIO139_OEN 0x322f08, BIT1
#define GPIO139_OUT 0x322f08, BIT0
#define GPIO139_IN  0x322f08, BIT2

#define GPIO140_PAD PAD_TGPIO0
#define GPIO140_OEN 0x322b80, BIT1
#define GPIO140_OUT 0x322b80, BIT0
#define GPIO140_IN  0x322b80, BIT2

#define GPIO141_PAD PAD_TGPIO1
#define GPIO141_OEN 0x322b82, BIT1
#define GPIO141_OUT 0x322b82, BIT0
#define GPIO141_IN  0x322b82, BIT2

#define GPIO142_PAD PAD_TGPIO2
#define GPIO142_OEN 0x322b84, BIT1
#define GPIO142_OUT 0x322b84, BIT0
#define GPIO142_IN  0x322b84, BIT2

#define GPIO143_PAD PAD_TGPIO3
#define GPIO143_OEN 0x322b86, BIT1
#define GPIO143_OUT 0x322b86, BIT0
#define GPIO143_IN  0x322b86, BIT2

#define GPIO144_PAD PAD_TS0_CLK
#define GPIO144_OEN 0x322c10, BIT1
#define GPIO144_OUT 0x322c10, BIT0
#define GPIO144_IN  0x322c10, BIT2

#define GPIO145_PAD PAD_TS0_D0
#define GPIO145_OEN 0x322c00, BIT1
#define GPIO145_OUT 0x322c00, BIT0
#define GPIO145_IN  0x322c00, BIT2

#define GPIO146_PAD PAD_TS0_D1
#define GPIO146_OEN 0x322c02, BIT1
#define GPIO146_OUT 0x322c02, BIT0
#define GPIO146_IN  0x322c02, BIT2

#define GPIO147_PAD PAD_TS0_D2
#define GPIO147_OEN 0x322c04, BIT1
#define GPIO147_OUT 0x322c04, BIT0
#define GPIO147_IN  0x322c04, BIT2

#define GPIO148_PAD PAD_TS0_D3
#define GPIO148_OEN 0x322c06, BIT1
#define GPIO148_OUT 0x322c06, BIT0
#define GPIO148_IN  0x322c06, BIT2

#define GPIO149_PAD PAD_TS0_D4
#define GPIO149_OEN 0x322c08, BIT1
#define GPIO149_OUT 0x322c08, BIT0
#define GPIO149_IN  0x322c08, BIT2

#define GPIO150_PAD PAD_TS0_D5
#define GPIO150_OEN 0x322c0a, BIT1
#define GPIO150_OUT 0x322c0a, BIT0
#define GPIO150_IN  0x322c0a, BIT2

#define GPIO151_PAD PAD_TS0_D6
#define GPIO151_OEN 0x322c0c, BIT1
#define GPIO151_OUT 0x322c0c, BIT0
#define GPIO151_IN  0x322c0c, BIT2

#define GPIO152_PAD PAD_TS0_D7
#define GPIO152_OEN 0x322c0e, BIT1
#define GPIO152_OUT 0x322c0e, BIT0
#define GPIO152_IN  0x322c0e, BIT2

#define GPIO153_PAD PAD_TS0_SYNC
#define GPIO153_OEN 0x322c12, BIT1
#define GPIO153_OUT 0x322c12, BIT0
#define GPIO153_IN  0x322c12, BIT2

#define GPIO154_PAD PAD_TS0_VLD
#define GPIO154_OEN 0x322c14, BIT1
#define GPIO154_OUT 0x322c14, BIT0
#define GPIO154_IN  0x322c14, BIT2

#define GPIO155_PAD PAD_TS1_CLK
#define GPIO155_OEN 0x322c30, BIT1
#define GPIO155_OUT 0x322c30, BIT0
#define GPIO155_IN  0x322c30, BIT2

#define GPIO156_PAD PAD_TS1_D0
#define GPIO156_OEN 0x322c20, BIT1
#define GPIO156_OUT 0x322c20, BIT0
#define GPIO156_IN  0x322c20, BIT2

#define GPIO157_PAD PAD_TS1_D1
#define GPIO157_OEN 0x322c22, BIT1
#define GPIO157_OUT 0x322c22, BIT0
#define GPIO157_IN  0x322c22, BIT2

#define GPIO158_PAD PAD_TS1_D2
#define GPIO158_OEN 0x322c24, BIT1
#define GPIO158_OUT 0x322c24, BIT0
#define GPIO158_IN  0x322c24, BIT2

#define GPIO159_PAD PAD_TS1_D3
#define GPIO159_OEN 0x322c26, BIT1
#define GPIO159_OUT 0x322c26, BIT0
#define GPIO159_IN  0x322c26, BIT2

#define GPIO160_PAD PAD_TS1_D4
#define GPIO160_OEN 0x322c28, BIT1
#define GPIO160_OUT 0x322c28, BIT0
#define GPIO160_IN  0x322c28, BIT2

#define GPIO161_PAD PAD_TS1_D5
#define GPIO161_OEN 0x322c2a, BIT1
#define GPIO161_OUT 0x322c2a, BIT0
#define GPIO161_IN  0x322c2a, BIT2

#define GPIO162_PAD PAD_TS1_D6
#define GPIO162_OEN 0x322c2c, BIT1
#define GPIO162_OUT 0x322c2c, BIT0
#define GPIO162_IN  0x322c2c, BIT2

#define GPIO163_PAD PAD_TS1_D7
#define GPIO163_OEN 0x322c2e, BIT1
#define GPIO163_OUT 0x322c2e, BIT0
#define GPIO163_IN  0x322c2e, BIT2

#define GPIO164_PAD PAD_TS1_SYNC
#define GPIO164_OEN 0x322c32, BIT1
#define GPIO164_OUT 0x322c32, BIT0
#define GPIO164_IN  0x322c32, BIT2

#define GPIO165_PAD PAD_TS1_VLD
#define GPIO165_OEN 0x322c34, BIT1
#define GPIO165_OUT 0x322c34, BIT0
#define GPIO165_IN  0x322c34, BIT2

#define GPIO166_PAD PAD_TS2_CLK
#define GPIO166_OEN 0x322c50, BIT1
#define GPIO166_OUT 0x322c50, BIT0
#define GPIO166_IN  0x322c50, BIT2

#define GPIO167_PAD PAD_TS2_D0
#define GPIO167_OEN 0x322c40, BIT1
#define GPIO167_OUT 0x322c40, BIT0
#define GPIO167_IN  0x322c40, BIT2

#define GPIO168_PAD PAD_TS2_D1
#define GPIO168_OEN 0x322c42, BIT1
#define GPIO168_OUT 0x322c42, BIT0
#define GPIO168_IN  0x322c42, BIT2

#define GPIO169_PAD PAD_TS2_D2
#define GPIO169_OEN 0x322c44, BIT1
#define GPIO169_OUT 0x322c44, BIT0
#define GPIO169_IN  0x322c44, BIT2

#define GPIO170_PAD PAD_TS2_D3
#define GPIO170_OEN 0x322c46, BIT1
#define GPIO170_OUT 0x322c46, BIT0
#define GPIO170_IN  0x322c46, BIT2

#define GPIO171_PAD PAD_TS2_D4
#define GPIO171_OEN 0x322c48, BIT1
#define GPIO171_OUT 0x322c48, BIT0
#define GPIO171_IN  0x322c48, BIT2

#define GPIO172_PAD PAD_TS2_D5
#define GPIO172_OEN 0x322c4a, BIT1
#define GPIO172_OUT 0x322c4a, BIT0
#define GPIO172_IN  0x322c4a, BIT2

#define GPIO173_PAD PAD_TS2_D6
#define GPIO173_OEN 0x322c4c, BIT1
#define GPIO173_OUT 0x322c4c, BIT0
#define GPIO173_IN  0x322c4c, BIT2

#define GPIO174_PAD PAD_TS2_D7
#define GPIO174_OEN 0x322c4e, BIT1
#define GPIO174_OUT 0x322c4e, BIT0
#define GPIO174_IN  0x322c4e, BIT2

#define GPIO175_PAD PAD_TS2_SYNC
#define GPIO175_OEN 0x322c52, BIT1
#define GPIO175_OUT 0x322c52, BIT0
#define GPIO175_IN  0x322c52, BIT2

#define GPIO176_PAD PAD_TS2_VLD
#define GPIO176_OEN 0x322c54, BIT1
#define GPIO176_OUT 0x322c54, BIT0
#define GPIO176_IN  0x322c54, BIT2

#define GPIO177_PAD PAD_TS3_CLK
#define GPIO177_OEN 0x322c70, BIT1
#define GPIO177_OUT 0x322c70, BIT0
#define GPIO177_IN  0x322c70, BIT2

#define GPIO178_PAD PAD_TS3_D0
#define GPIO178_OEN 0x322c60, BIT1
#define GPIO178_OUT 0x322c60, BIT0
#define GPIO178_IN  0x322c60, BIT2

#define GPIO179_PAD PAD_TS3_SYNC
#define GPIO179_OEN 0x322c72, BIT1
#define GPIO179_OUT 0x322c72, BIT0
#define GPIO179_IN  0x322c72, BIT2

#define GPIO180_PAD PAD_TS3_VLD
#define GPIO180_OEN 0x322c74, BIT1
#define GPIO180_OUT 0x322c74, BIT0
#define GPIO180_IN  0x322c74, BIT2

/* analog gpio
 * must add __GPIO(X) in below gpio_table
 */
/*
#define GPIO182_PAD PADA_RIN0P
#define GPIO182_OEN 0x102536, BIT0
#define GPIO182_OUT 0x102537, BIT0
#define GPIO182_IN  0x10253C, BIT0

#define GPIO183_PAD PADA_RIN1P
#define GPIO183_OEN 0x102536, BIT1
#define GPIO183_OUT 0x102537, BIT1
#define GPIO183_IN  0x10253C, BIT1

#define GPIO184_PAD PADA_GIN0P
#define GPIO184_OEN 0x102538, BIT0
#define GPIO184_OUT 0x102539, BIT0
#define GPIO184_IN  0x10253D, BIT0

#define GPIO185_PAD PADA_GIN1P
#define GPIO185_OEN 0x102538, BIT1
#define GPIO185_OUT 0x102539, BIT1
#define GPIO185_IN  0x10253D, BIT1

#define GPIO186_PAD PADA_GIN0M
#define GPIO186_OEN 0x102538, BIT4
#define GPIO186_OUT 0x102539, BIT4
#define GPIO186_IN  0x10253D, BIT4

#define GPIO187_PAD PADA_GIN1M
#define GPIO187_OEN 0x102538, BIT5
#define GPIO187_OUT 0x102539, BIT5
#define GPIO187_IN  0x10253D, BIT5

#define GPIO188_PAD PADA_BIN0P
#define GPIO188_OEN 0x10253A, BIT0
#define GPIO188_OUT 0x10253B, BIT0
#define GPIO188_IN  0x10253E, BIT0

#define GPIO189_PAD PADA_BIN1P
#define GPIO189_OEN 0x10253A, BIT1
#define GPIO189_OUT 0x10253B, BIT1
#define GPIO189_IN  0x10253E, BIT1

#define GPIO190_PAD PADA_HSYNC0
#define GPIO190_OEN 0x10255A, BIT0
#define GPIO190_OUT 0x10255A, BIT6
#define GPIO190_IN  0x10255C, BIT0

#define GPIO191_PAD PADA_VSYNC0
#define GPIO191_OEN 0x10255A, BIT3
#define GPIO191_OUT 0x10255B, BIT1
#define GPIO191_IN  0x10255C, BIT3
*/

/* ext_gpio_int, and access it via u8 */
#define GPIO_EXT0_MSK  0x10194c, BIT7
#define GPIO_EXT0_POL  0x101954, BIT7
#define GPIO_EXT0_CLR  0x10195c, BIT7
#define GPIO_EXT0_STS  0x10195c, BIT7

#define GPIO_EXT1_MSK  0x10194d, BIT3
#define GPIO_EXT1_POL  0x101955, BIT3
#define GPIO_EXT1_CLR  0x10195d, BIT3
#define GPIO_EXT1_STS  0x10195d, BIT3

#define GPIO_EXT2_MSK  0x10194d, BIT7
#define GPIO_EXT2_POL  0x101955, BIT7
#define GPIO_EXT2_CLR  0x10195d, BIT7
#define GPIO_EXT2_STS  0x10195d, BIT7

#define GPIO_EXT3_MSK  0x10194e, BIT7
#define GPIO_EXT3_POL  0x101956, BIT7
#define GPIO_EXT3_CLR  0x10195e, BIT7
#define GPIO_EXT3_STS  0x10195e, BIT7

#define GPIO_EXT4_MSK  0x10194f, BIT0
#define GPIO_EXT4_POL  0x101957, BIT0
#define GPIO_EXT4_CLR  0x10195f, BIT0
#define GPIO_EXT4_STS  0x10195f, BIT0

#define GPIO_EXT5_MSK  0x10194f, BIT1
#define GPIO_EXT5_POL  0x101957, BIT1
#define GPIO_EXT5_CLR  0x10195f, BIT1
#define GPIO_EXT5_STS  0x10195f, BIT1

#define GPIO_EXT6_MSK  0x10194f, BIT2
#define GPIO_EXT6_POL  0x101957, BIT2
#define GPIO_EXT6_CLR  0x10195f, BIT2
#define GPIO_EXT6_STS  0x10195f, BIT2

#define GPIO_EXT7_MSK  0x10194f, BIT7
#define GPIO_EXT7_POL  0x101957, BIT7
#define GPIO_EXT7_CLR  0x10195f, BIT7
#define GPIO_EXT7_STS  0x10195f, BIT7

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
#if (GPIO_PM_INT_SUPPORTED)

#define PM_INT_COUNT   33

#define BIT_PM_GPIO_INT_MASK          BIT4
#define BIT_PM_GPIO_INT_FORCE         BIT5
#define BIT_PM_GPIO_INT_CLR           BIT6
#define BIT_PM_GPIO_INT_POLARITY      BIT7
#define BIT_PM_GPIO_INT_FINAL_STATUS  BIT0
/* pm_sleep_int in PM IRQ page in interrupt table */
#define BIT_PM_PMSLEEP_IRQ_MASK       BIT2

#define E_INT_PM2HOST  E_INT_IRQ_PM

/* PM WK FIQ page in interrupt table */
/* GPIO number, NO need to +1 */
const U16 PM_gpio_IntPad[PM_INT_COUNT] =
{
    3,    /* PAD_IRIN */
    4,    /* PAD_PWM_PM */
    5,    /* PAD_CEC0 */
    6,    /* PAD_GPIO0_PM */
    7,    /* PAD_GPIO1_PM */
    8,    /* PAD_GPIO2_PM */
    9,    /* PAD_USB_CTRL */
    10,   /* PAD_GPIO5_PM */
    11,   /* PAD_GPIO6_PM */
    12,   /* PAD_GPIO7_PM */
    13,   /* PAD_GPIO8_PM */
    14,   /* PAD_GPIO9_PM */
    15,   /* PAD_GPIO10_PM */
    16,   /* PAD_GPIO11_PM */
    17,   /* PAD_GPIO12_PM */
    18,   /* PAD_GPIO13_PM */
    19,   /* PAD_GPIO14_PM */
    20,   /* PAD_GPIO15_PM */
    21,   /* PAD_GPIO16_PM */
    22,   /* PAD_GPIO17_PM */
    23,   /* PAD_GPIO18_PM */
    24,   /* PAD_GPIO19_PM */
    25,   /* PAD_GPIO20_PM */
    26,   /* PAD_GPIO21_PM */
    27,   /* PAD_GPIO22_PM */
    28,   /* PAD_GPIO23_PM */
    29,   /* PAD_GPIO24_PM */
    30,   /* PAD_GPIO25_PM */
    31,   /* PAD_GPIO26_PM */
    32,   /* PAD_GPIO27_PM */
    33,   /* PAD_GPIO28_PM */
    52,   /* PAD_VID2 */
    53    /* PAD_WOL_INT_OUT */
};

const U32 PM_gpio_IRQreg[PM_INT_COUNT] =
{
    0x0f26,    /* PAD_IRIN */
    0x0f28,    /* PAD_PWM_PM */
    0x0f2a,    /* PAD_CEC0 */
    0x0f00,    /* PAD_GPIO0_PM */
    0x0f02,    /* PAD_GPIO1_PM */
    0x0f04,    /* PAD_GPIO2_PM */
    0x0f06,    /* PAD_USB_CTRL */
    0x0f0a,    /* PAD_GPIO5_PM */
    0x0f0c,    /* PAD_GPIO6_PM */
    0x0f0e,    /* PAD_GPIO7_PM */
    0x0f10,    /* PAD_GPIO8_PM */
    0x0f12,    /* PAD_GPIO9_PM */
    0x0f14,    /* PAD_GPIO10_PM */
    0x0f16,    /* PAD_GPIO11_PM */
    0x0f18,    /* PAD_GPIO12_PM */
    0x0f1a,    /* PAD_GPIO13_PM */
    0x0f1c,    /* PAD_GPIO14_PM */
    0x0f1e,    /* PAD_GPIO15_PM */
    0x0f20,    /* PAD_GPIO16_PM */
    0x0f48,    /* PAD_GPIO17_PM */
    0x0f4a,    /* PAD_GPIO18_PM */
    0x0f4c,    /* PAD_GPIO19_PM */
    0x0f4e,    /* PAD_GPIO20_PM */
    0x0f50,    /* PAD_GPIO21_PM */
    0x0f52,    /* PAD_GPIO22_PM */
    0x0f54,    /* PAD_GPIO23_PM */
    0x0f56,    /* PAD_GPIO24_PM */
    0x0f58,    /* PAD_GPIO25_PM */
    0x0f5a,    /* PAD_GPIO26_PM */
    0x0f5c,    /* PAD_GPIO27_PM */
    0x0f5e,    /* PAD_GPIO28_PM */
    0x0f22,    /* PAD_VID2 */
    0x0f3c     /* PAD_WOL_INT_OUT */
};

const U32 PM_IRQ_SRC[1] =
{
    0x2b28
};
#endif

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
#define INT_COUNT   8

/* ext_gpio_int in nonPM FIQ page in interrupt table */
/* must check gpio_func_mux page in chiptop table */
/* extend pad mux in below struct */
const int gpio_IntPad[INT_COUNT]=
{
    132,     //PAD_PWM2
    133,     //PAD_SPDIF_IN
    79,      //PAD_I2S_OUT_MCK
    95,      //PAD_PCM2_RESET
    63,      //PAD_GPIO9
    64,      //PAD_GPIO10
    65,      //PAD_GPIO11
    66       //PAD_GPIO12
};

const int gpio_IRQnum[INT_COUNT]= {
				E_FIQEXPL_EXT_GPIO0, /*E_INT_FIQ_GPIO0,*/
				E_FIQEXPL_EXT_GPIO1, /*E_INT_FIQ_GPIO1,*/
				E_FIQEXPL_EXT_GPIO2, /*E_INT_FIQ_GPIO2,*/
				E_FIQEXPH_EXT_GPIO3, /*E_INT_FIQ_GPIO3,*/
				E_FIQEXPH_EXT_GPIO4, /*E_INT_FIQ_GPIO4,*/
				E_FIQEXPH_EXT_GPIO5, /*E_INT_FIQ_GPIO5,*/
				E_FIQEXPH_EXT_GPIO6, /*E_INT_FIQ_GPIO6,*/
				E_FIQEXPH_GPIOINIT7, /*E_INT_FIQ_GPIO7,*/
                                            };

irq_handler_t  gpio_irq_pCallback[INT_COUNT];
void *gpio_irq_dev_id[INT_COUNT];


static const struct gpio_setting
{
    U32 r_oen;
    U8  m_oen;
    U32 r_out;
    U8  m_out;
    U32 r_in;
    U8  m_in;
} gpio_table[] =
{
#define __GPIO__(_x_)   { CONCAT(CONCAT(GPIO, _x_), _OEN),   \
                          CONCAT(CONCAT(GPIO, _x_), _OUT),   \
                          CONCAT(CONCAT(GPIO, _x_), _IN) }
#define __GPIO(_x_)     __GPIO__(_x_)

//
// !! WARNING !! DO NOT MODIFIY !!!!
//
// These defines order must match following
// 1. the PAD name in GPIO excel
// 2. the perl script to generate the package header file
//
    //__GPIO(999), // 0 is not used

    __GPIO(999), __GPIO(1), __GPIO(2), __GPIO(3), __GPIO(4),
    __GPIO(5), __GPIO(6), __GPIO(7), __GPIO(8), __GPIO(9),
    __GPIO(10), __GPIO(11), __GPIO(12), __GPIO(13), __GPIO(14),
    __GPIO(15), __GPIO(16), __GPIO(17), __GPIO(18), __GPIO(19),
    __GPIO(20), __GPIO(21), __GPIO(22), __GPIO(23), __GPIO(24),
    __GPIO(25), __GPIO(26), __GPIO(27), __GPIO(28), __GPIO(29),
    __GPIO(30), __GPIO(31), __GPIO(32), __GPIO(33), __GPIO(34),
    __GPIO(35), __GPIO(36), __GPIO(37), __GPIO(38), __GPIO(39),
    __GPIO(40), __GPIO(41), __GPIO(42), __GPIO(43), __GPIO(44),
    __GPIO(45), __GPIO(46), __GPIO(47), __GPIO(48), __GPIO(49),
    __GPIO(50), __GPIO(51), __GPIO(52), __GPIO(53), __GPIO(54),
    __GPIO(55), __GPIO(56), __GPIO(57), __GPIO(58), __GPIO(59),
    __GPIO(60), __GPIO(61), __GPIO(62), __GPIO(63), __GPIO(64),
    __GPIO(65), __GPIO(66), __GPIO(67), __GPIO(68), __GPIO(69),
    __GPIO(70), __GPIO(71), __GPIO(72), __GPIO(73), __GPIO(74),
    __GPIO(75), __GPIO(76), __GPIO(77), __GPIO(78), __GPIO(79),
    __GPIO(80), __GPIO(81), __GPIO(82), __GPIO(83), __GPIO(84),
    __GPIO(85), __GPIO(86), __GPIO(87), __GPIO(88), __GPIO(89),
    __GPIO(90), __GPIO(91), __GPIO(92), __GPIO(93), __GPIO(94),
    __GPIO(95), __GPIO(96), __GPIO(97), __GPIO(98), __GPIO(99),
    __GPIO(100), __GPIO(101), __GPIO(102), __GPIO(103), __GPIO(104),
    __GPIO(105), __GPIO(106), __GPIO(107), __GPIO(108), __GPIO(109),
    __GPIO(110), __GPIO(111), __GPIO(112), __GPIO(113), __GPIO(114),
    __GPIO(115), __GPIO(116), __GPIO(117), __GPIO(118), __GPIO(119),
    __GPIO(120), __GPIO(121), __GPIO(122), __GPIO(123), __GPIO(124),
    __GPIO(125), __GPIO(126), __GPIO(127), __GPIO(128), __GPIO(129),
    __GPIO(130), __GPIO(131), __GPIO(132), __GPIO(133), __GPIO(134),
    __GPIO(135), __GPIO(136), __GPIO(137), __GPIO(138), __GPIO(139),
    __GPIO(140), __GPIO(141), __GPIO(142), __GPIO(143), __GPIO(144),
    __GPIO(145), __GPIO(146), __GPIO(147), __GPIO(148), __GPIO(149),
    __GPIO(150), __GPIO(151), __GPIO(152), __GPIO(153), __GPIO(154),
    __GPIO(155), __GPIO(156), __GPIO(157), __GPIO(158), __GPIO(159),
    __GPIO(160), __GPIO(161), __GPIO(162), __GPIO(163), __GPIO(164),
    __GPIO(165), __GPIO(166), __GPIO(167), __GPIO(168), __GPIO(169),
    __GPIO(170), __GPIO(171), __GPIO(172), __GPIO(173), __GPIO(174),
    __GPIO(175), __GPIO(176), __GPIO(177), __GPIO(178), __GPIO(179),
    __GPIO(180)
};

static const struct gpio_irq_setting
{
    U32  msk_reg;
    U16  m_msk;
    U32  pol_reg;
    U16  m_pol;
    U32  clr_reg;
    U16  m_clr;
    U32  sts_reg;
    U16  m_sts;
} gpio_irq_table[] =
{
#define __GPIOIRQ__(_x_)   { CONCAT(CONCAT(GPIO_EXT, _x_), _MSK),   \
                          CONCAT(CONCAT(GPIO_EXT, _x_), _POL),   \
                          CONCAT(CONCAT(GPIO_EXT, _x_), _CLR),  \
                          CONCAT(CONCAT(GPIO_EXT, _x_), _STS)}
#define __GPIO_EXT(_x_)     __GPIOIRQ__(_x_)

   __GPIO_EXT(0),__GPIO_EXT(1),__GPIO_EXT(2),__GPIO_EXT(3),
   __GPIO_EXT(4),__GPIO_EXT(5),__GPIO_EXT(6),__GPIO_EXT(7)
};

spinlock_t   gpio_spinlock;
//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
#if (GPIO_PM_INT_SUPPORTED)
static void (*_PMGPIOCallback)(void);
static irq_handler_t pm_gpio_irq(int irq, void *data)
{
    U8 i;
    for(i=0; i < PM_INT_COUNT; i++)
    {
        if(MHal_GPIO_ReadRegBit((PM_gpio_IRQreg[i]+0x1),BIT_PM_GPIO_INT_FINAL_STATUS))
        {
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 1, BIT_PM_GPIO_INT_MASK);
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i],1, BIT_PM_GPIO_INT_CLR);
            _PMGPIOCallback();
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 0,  BIT_PM_GPIO_INT_MASK);
        }
    }
   return IRQ_HANDLED;
}
#endif

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

//the functions of this section set to initialize
void MHal_GPIO_Init(void)
{
    MHal_GPIO_REG(REG_ALL_PAD_IN) &= ~BIT0;
}

void MHal_GPIO_WriteRegBit(U32 u32Reg, U8 u8Enable, U8 u8BitMsk)
{
    if(u8Enable)
        MHal_GPIO_REG(u32Reg) |= u8BitMsk;
    else
        MHal_GPIO_REG(u32Reg) &= (~u8BitMsk);
}

U8 MHal_GPIO_ReadRegBit(U32 u32Reg, U8 u8BitMsk)
{
    return ((MHal_GPIO_REG(u32Reg)&u8BitMsk)? 1 : 0);
}

void MHal_GPIO_Pad_Set(U32 u32IndexGPIO)
{

}
void MHal_GPIO_Pad_Oen(U32 u32IndexGPIO)
{
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
}

void MHal_GPIO_Pad_Odn(U32 u32IndexGPIO)
{
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
}

U8 MHal_GPIO_Pad_Level(U32 u32IndexGPIO)
{
    return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_in)&gpio_table[u32IndexGPIO].m_in)? 1 : 0);
}

U8 MHal_GPIO_Pad_InOut(U32 u32IndexGPIO)
{
    return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen)&gpio_table[u32IndexGPIO].m_oen)? 1 : 0);
}

void MHal_GPIO_Pull_High(U32 u32IndexGPIO)
{
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_out) |= gpio_table[u32IndexGPIO].m_out;
}

void MHal_GPIO_Pull_Low(U32 u32IndexGPIO)
{
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_out) &= (~gpio_table[u32IndexGPIO].m_out);
}

void MHal_GPIO_Set_High(U32 u32IndexGPIO)
{
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_out) |= gpio_table[u32IndexGPIO].m_out;
}

void MHal_GPIO_Set_Low(U32 u32IndexGPIO)
{
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_out) &= (~gpio_table[u32IndexGPIO].m_out);
}

void MHal_GPIO_Set_Input(U32 u32IndexGPIO)
{
	MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
}

U8 MHal_GPIO_Get_Level(U32 u32IndexGPIO)
{
	return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_in) & gpio_table[u32IndexGPIO].m_in) ? 1 : 0);
}

int MHal_GPIO_Get_Interrupt_Num(U32 u32IndexGPIO)
{
	U8 i;
	for(i = 0; i < INT_COUNT; i++)
	{
		if((gpio_IntPad[i] == u32IndexGPIO))
		{
			return gpio_IRQnum[i];
		}
	}
	return -1;
}

u32 MHal_GPIO_Get_Pins_Count(void)
{
	return __Sizeof_GPIO_Pins();
}

int MHal_GPIO_Get_Pin_Status_Array(U32* pGPIOPinStatusList, U32 u32PinCount, U32 *upRetPinCount)
{
	int gpio;

	if (!pGPIOPinStatusList || !upRetPinCount)
		return -1;// invalid value EINVLD;

	if (u32PinCount == 0)
		return -1;

	if (u32PinCount > __Sizeof_GPIO_Pins())
		u32PinCount = __Sizeof_GPIO_Pins();

	for (gpio = 0; gpio < u32PinCount; gpio++)
	{
		int oenIndex = gpio*2;
		int outIndex = gpio*2 + 1;
		pGPIOPinStatusList[oenIndex] = MHal_GPIO_Pad_InOut(gpio);
		pGPIOPinStatusList[outIndex] = MHal_GPIO_Get_Level(gpio);
#if 0
		//debug statements for GPIO STR
		if (gpio == 16)
		{
			printk("[GPIO Suspend]Mute pin status: Oen->%u Out->%u\n",
				pGPIOPinStatusList[oenIndex], pGPIOPinStatusList[outIndex]);
		}
#endif
	}

	*upRetPinCount = gpio;
	return 0;

}
int MHal_GPIO_Get_Pin_Status(U32 u32IndexGPIO, U8 *upOen, U8 *upLevel)
{
	if (!upOen || !upLevel)
	{
		return -1;//-EINVL
	}
	if (u32IndexGPIO >= __Sizeof_GPIO_Pins())
	{
		return -1;//-EINVAL
	}

	*upOen = MHal_GPIO_Pad_InOut(u32IndexGPIO);
	*upLevel = MHal_GPIO_Get_Level(u32IndexGPIO);

	return 0;
}
int MHal_GPIO_Set_Pin_Status_Array(U32* pGPIOPinStatusList, U32 u32PinCount, U32 *upRetPinCount, U32* pin_disable)
{
	int gpio;

	if (!pGPIOPinStatusList || !upRetPinCount)
		return -1;// invalid value EINVLD;

	if (u32PinCount == 0)
		return -1;

	if (u32PinCount > __Sizeof_GPIO_Pins())
		u32PinCount = __Sizeof_GPIO_Pins();

	for (gpio = 0; gpio < u32PinCount; gpio++)
	{
		int oenIndex = gpio*2;
		int outIndex = gpio*2 + 1;

                if (pin_disable[gpio] == 1)
                        continue;

		/* oen first */
		if (pGPIOPinStatusList[oenIndex])
		{
			/* 1: input (output disabled), oen toggle on */
			MHal_GPIO_Pad_Odn(gpio);
		}
		else
		{
			/* 0: output enable */
			MHal_GPIO_Pad_Oen(gpio);
		}

		/* then out */
		if (pGPIOPinStatusList[outIndex])
		{
			/* out toggle on */
			MHal_GPIO_Pull_High(gpio);
		}
		else
		{
			MHal_GPIO_Pull_Low(gpio);
		}
#if 0
		//debug statements for GPIO STR
		if (gpio == 16)
		{
			u8 oen;
			u8 out;
			printk("[GPIO Resume]Mute pin status in memory: Oen->%u Out->%u\n",
				pGPIOPinStatusList[oenIndex], pGPIOPinStatusList[outIndex]);

			MHal_GPIO_Get_Pin_Status(gpio, &oen, &out);
			printk("[GPIO Resume]Mute pin status in RIU: Oen->%u Out->%u\n",
				oen, out);
		}
#endif
	}

	*upRetPinCount = gpio;

	return 0;
}
int MHal_GPIO_Set_Pin_Status(U32 u32IndexGPIO, U8 uOen, U8 uOut)
{
	if (u32IndexGPIO >= __Sizeof_GPIO_Pins())
	{
		return -1;//-EINVAL
	}

	if (uOen)
	{
		/* oen toggle on */
		MHal_GPIO_Pad_Odn(u32IndexGPIO);
	}
	else
	{
		MHal_GPIO_Pad_Oen(u32IndexGPIO);
	}

	if (uOut)
	{
		/* out toggle on */
		MHal_GPIO_Pull_High(u32IndexGPIO);
	}
	else
	{
		MHal_GPIO_Pull_Low(u32IndexGPIO);
	}

return 0;
}


int MHal_GPIO_Enable_Interrupt(int gpio_num, unsigned long gpio_edge_type, irq_handler_t pCallback, void *dev_id)
{
    U8 i;
    for(i = 0; i < INT_COUNT; i++)
    {
        if(gpio_IntPad[i] == gpio_num)
        {
            if(gpio_edge_type == IRQF_TRIGGER_RISING)
            {
              MHal_GPIO_WriteRegBit(gpio_irq_table[i].pol_reg, 0, gpio_irq_table[i].m_pol);   //set fiq polarity to
            }
            else if(gpio_edge_type == IRQF_TRIGGER_FALLING)
            {
              MHal_GPIO_WriteRegBit(gpio_irq_table[i].pol_reg, 1, gpio_irq_table[i].m_pol);   //set fiq polarity to
            }
            else
            {
               printk("Trigger Type not support\n");
               return -1;
            }

            /* write 1 to clear irq status */
            MHal_GPIO_WriteRegBit(gpio_irq_table[i].clr_reg, 1, gpio_irq_table[i].m_clr);

            if(request_irq(gpio_IRQnum[i], (irq_handler_t)pCallback, 0x0, "GPIO_NONPM", dev_id))
            {
                printk("request_irq fail\n");
                return -EBUSY;
            }
            MHal_GPIO_Pad_Odn(gpio_num);

            /* write 1 to clear irq status again */
            MHal_GPIO_WriteRegBit(gpio_irq_table[i].clr_reg, 1, gpio_irq_table[i].m_clr);
        }
    }

#if (GPIO_PM_INT_SUPPORTED)
    for(i=0; i<PM_INT_COUNT; i++)
    {
        if(PM_gpio_IntPad[i] == gpio_num)
        {
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 1,  BIT_PM_GPIO_INT_CLR);
            _PMGPIOCallback = (void (*)(void))pCallback;
            if(gpio_edge_type == IRQF_TRIGGER_RISING)
            {
              MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 0, BIT_PM_GPIO_INT_POLARITY);   //set fiq polarity to
            }
            else if(gpio_edge_type == IRQF_TRIGGER_FALLING)
            {
              MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 1, BIT_PM_GPIO_INT_POLARITY);   //set fiq polarity to
            }
            else
            {
               printk("Trigger Type not support\n");
               return -1;
            }
            if(request_irq(E_IRQEXPL_PM_IRQ, (irq_handler_t)pm_gpio_irq, 0x0, "GPIO_PM", dev_id))
            {
                printk("request_irq fail\n");
                return -EBUSY;
            }
            MHal_GPIO_Pad_Odn(gpio_num);
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 1,  BIT_PM_GPIO_INT_CLR);
            MHal_GPIO_WriteRegBit(PM_IRQ_SRC[0], 0, BIT_PM_PMSLEEP_IRQ_MASK);
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 0,  BIT_PM_GPIO_INT_MASK);
        }
    }
#endif
    return 0;
}

int MHal_GPIO_Disable_Interrupt(int gpio_num, void *dev_id)
{
    U8 i;

    for(i = 0; i < INT_COUNT; i++)
    {
      if(gpio_IntPad[i] == gpio_num)
      {
         MHal_GPIO_WriteRegBit(gpio_irq_table[i].clr_reg, 1, gpio_irq_table[i].m_clr);
         MHal_GPIO_WriteRegBit(gpio_irq_table[i].msk_reg, 1, gpio_irq_table[i].m_msk);
         free_irq(gpio_IRQnum[i], dev_id);
         gpio_irq_pCallback[i] = NULL;
         gpio_irq_dev_id[i] = NULL;
      }
    }

#if (GPIO_PM_INT_SUPPORTED)
    for(i=0; i<PM_INT_COUNT; i++)
    {
        if(PM_gpio_IntPad[i] == gpio_num)
        {
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 1,  BIT_PM_GPIO_INT_MASK);
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 1,  BIT_PM_GPIO_INT_CLR);
            free_irq(E_IRQEXPL_PM_IRQ, dev_id);
        }
    }
#endif

    return 0;
}
