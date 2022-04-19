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
/* VID PADs are not open in use as default */
/*
#define GPIO0_PAD PAD_VID0
#define GPIO0_OEN 0x2e84, BIT1
#define GPIO0_OUT 0x2e84, BIT0
#define GPIO0_IN  0x2e84, BIT2

#define GPIO1_PAD PAD_VID1
#define GPIO1_OEN 0x2e85, BIT1
#define GPIO1_OUT 0x2e85, BIT0
#define GPIO1_IN  0x2e85, BIT2

#define GPIO2_PAD PAD_VID2
#define GPIO2_OEN 0x0f22, BIT0
#define GPIO2_OUT 0x0f22, BIT1
#define GPIO2_IN  0x0f22, BIT2
*/

#define GPIO3_PAD PAD_DDCA_CK
#define GPIO3_OEN 0x0494, BIT1
#define GPIO3_OUT 0x0494, BIT2
#define GPIO3_IN  0x0494, BIT0

#define GPIO4_PAD PAD_DDCA_DA
#define GPIO4_OEN 0x0494, BIT5
#define GPIO4_OUT 0x0494, BIT6
#define GPIO4_IN  0x0494, BIT4

#define GPIO5_PAD PAD_IRIN
#define GPIO5_OEN 0x0f26, BIT2
#define GPIO5_OUT 0x0f26, BIT2
#define GPIO5_IN  0x0f26, BIT2

#define GPIO6_PAD PAD_PWM0_PM
#define GPIO6_OEN 0x0f28, BIT0
#define GPIO6_OUT 0x0f28, BIT1
#define GPIO6_IN  0x0f28, BIT2

#define GPIO7_PAD PAD_PWM1_PM
#define GPIO7_OEN 0x0f48, BIT0
#define GPIO7_OUT 0x0f48, BIT1
#define GPIO7_IN  0x0f48, BIT2

#define GPIO9_PAD PAD_GPIO0_PM
#define GPIO9_OEN 0x0f00, BIT0
#define GPIO9_OUT 0x0f00, BIT1
#define GPIO9_IN  0x0f00, BIT2

#define GPIO10_PAD PAD_GPIO1_PM
#define GPIO10_OEN 0x0f02, BIT0
#define GPIO10_OUT 0x0f02, BIT1
#define GPIO10_IN  0x0f02, BIT2

#define GPIO11_PAD PAD_GPIO2_PM
#define GPIO11_OEN 0x0f04, BIT0
#define GPIO11_OUT 0x0f04, BIT1
#define GPIO11_IN  0x0f04, BIT2

#define GPIO12_PAD PAD_USB_CTRL
#define GPIO12_OEN 0x0f06, BIT0
#define GPIO12_OUT 0x0f06, BIT1
#define GPIO12_IN  0x0f06, BIT2

#define GPIO13_PAD PAD_GPIO5_PM
#define GPIO13_OEN 0x0f0a, BIT0
#define GPIO13_OUT 0x0f0a, BIT1
#define GPIO13_IN  0x0f0a, BIT2

#define GPIO14_PAD PAD_GPIO6_PM
#define GPIO14_OEN 0x0f0c, BIT0
#define GPIO14_OUT 0x0f0c, BIT1
#define GPIO14_IN  0x0f0c, BIT2

#define GPIO15_PAD PAD_GPIO7_PM
#define GPIO15_OEN 0x0f0e, BIT0
#define GPIO15_OUT 0x0f0e, BIT1
#define GPIO15_IN  0x0f0e, BIT2

#define GPIO16_PAD PAD_GPIO8_PM
#define GPIO16_OEN 0x0f10, BIT0
#define GPIO16_OUT 0x0f10, BIT1
#define GPIO16_IN  0x0f10, BIT2

#define GPIO17_PAD PAD_PM_SPI_MOSI
#define GPIO17_OEN 0x0f32, BIT0
#define GPIO17_OUT 0x0f32, BIT1
#define GPIO17_IN  0x0f32, BIT2

#define GPIO18_PAD PAD_PM_SPI_MISO
#define GPIO18_OEN 0x0f34, BIT0
#define GPIO18_OUT 0x0f34, BIT1
#define GPIO18_IN  0x0f34, BIT2

#define GPIO19_PAD PAD_PM_SPI_CK
#define GPIO19_OEN 0x0f30, BIT0
#define GPIO19_OUT 0x0f30, BIT1
#define GPIO19_IN  0x0f30, BIT2

#define GPIO20_PAD PAD_PM_SPI_CS
#define GPIO20_OEN 0x0f2e, BIT0
#define GPIO20_OUT 0x0f2e, BIT1
#define GPIO20_IN  0x0f2e, BIT2

#define GPIO21_PAD PAD_DDCDA_CK
#define GPIO21_OEN 0x0496, BIT1
#define GPIO21_OUT 0x0496, BIT2
#define GPIO21_IN  0x0496, BIT0

#define GPIO22_PAD PAD_DDCDA_DA
#define GPIO22_OEN 0x0496, BIT5
#define GPIO22_OUT 0x0496, BIT6
#define GPIO22_IN  0x0496, BIT4

#define GPIO23_PAD PAD_DDCDB_CK
#define GPIO23_OEN 0x0497, BIT1
#define GPIO23_OUT 0x0497, BIT2
#define GPIO23_IN  0x0497, BIT0

#define GPIO24_PAD PAD_DDCDB_DA
#define GPIO24_OEN 0x0497, BIT5
#define GPIO24_OUT 0x0497, BIT6
#define GPIO24_IN  0x0497, BIT4

#define GPIO25_PAD PAD_WOL_INT_OUT
#define GPIO25_OEN 0x2e82, BIT1
#define GPIO25_OUT 0x2e82, BIT0
#define GPIO25_IN  0x2e82, BIT2

#define GPIO26_PAD PAD_SAR0
#define GPIO26_OEN 0x1423, BIT0
#define GPIO26_OUT 0x1424, BIT0
#define GPIO26_IN  0x1425, BIT0

#define GPIO27_PAD PAD_SAR1
#define GPIO27_OEN 0x1423, BIT1
#define GPIO27_OUT 0x1424, BIT1
#define GPIO27_IN  0x1425, BIT1

#define GPIO28_PAD PAD_SAR2
#define GPIO28_OEN 0x1423, BIT2
#define GPIO28_OUT 0x1424, BIT2
#define GPIO28_IN  0x1425, BIT2

#define GPIO29_PAD PAD_SAR3
#define GPIO29_OEN 0x1423, BIT3
#define GPIO29_OUT 0x1424, BIT3
#define GPIO29_IN  0x1425, BIT3

#define GPIO30_PAD PAD_SAR4
#define GPIO30_OEN 0x1423, BIT4
#define GPIO30_OUT 0x1424, BIT4
#define GPIO30_IN  0x1425, BIT4

#define GPIO31_PAD PAD_VPLUGIN
#define GPIO31_OEN 0x1423, BIT5
#define GPIO31_OUT 0x1424, BIT5
#define GPIO31_IN  0x1425, BIT5

#define GPIO35_PAD PAD_DDCR_CK
#define GPIO35_OEN 0x12109a, BIT1
#define GPIO35_OUT 0x12109a, BIT3
#define GPIO35_IN  0x12109a, BIT5

#define GPIO36_PAD PAD_DDCR_DA
#define GPIO36_OEN 0x12109a, BIT0
#define GPIO36_OUT 0x12109a, BIT2
#define GPIO36_IN  0x12109a, BIT4

#define GPIO37_PAD PAD_GPIO0
#define GPIO37_OEN 0x101e5c, BIT0
#define GPIO37_OUT 0x101e56, BIT0
#define GPIO37_IN  0x101e50, BIT0

#define GPIO38_PAD PAD_GPIO1
#define GPIO38_OEN 0x101e5c, BIT1
#define GPIO38_OUT 0x101e56, BIT1
#define GPIO38_IN  0x101e50, BIT1

#define GPIO39_PAD PAD_GPIO2
#define GPIO39_OEN 0x101e5c, BIT2
#define GPIO39_OUT 0x101e56, BIT2
#define GPIO39_IN  0x101e50, BIT2

#define GPIO40_PAD PAD_GPIO3
#define GPIO40_OEN 0x101e5c, BIT3
#define GPIO40_OUT 0x101e56, BIT3
#define GPIO40_IN  0x101e50, BIT3

#define GPIO41_PAD PAD_GPIO4
#define GPIO41_OEN 0x101e5c, BIT4
#define GPIO41_OUT 0x101e56, BIT4
#define GPIO41_IN  0x101e50, BIT4

#define GPIO42_PAD PAD_GPIO5
#define GPIO42_OEN 0x101e5c, BIT5
#define GPIO42_OUT 0x101e56, BIT5
#define GPIO42_IN  0x101e50, BIT5

#define GPIO43_PAD PAD_GPIO6
#define GPIO43_OEN 0x101e5c, BIT6
#define GPIO43_OUT 0x101e56, BIT6
#define GPIO43_IN  0x101e50, BIT6

#define GPIO44_PAD PAD_GPIO7
#define GPIO44_OEN 0x101e5c, BIT7
#define GPIO44_OUT 0x101e56, BIT7
#define GPIO44_IN  0x101e50, BIT7

#define GPIO45_PAD PAD_GPIO8
#define GPIO45_OEN 0x101e5d, BIT0
#define GPIO45_OUT 0x101e57, BIT0
#define GPIO45_IN  0x101e51, BIT0

#define GPIO46_PAD PAD_GPIO9
#define GPIO46_OEN 0x101e5d, BIT1
#define GPIO46_OUT 0x101e57, BIT1
#define GPIO46_IN  0x101e51, BIT1

#define GPIO47_PAD PAD_GPIO10
#define GPIO47_OEN 0x101e5d, BIT2
#define GPIO47_OUT 0x101e57, BIT2
#define GPIO47_IN  0x101e51, BIT2

#define GPIO48_PAD PAD_GPIO11
#define GPIO48_OEN 0x101e5d, BIT3
#define GPIO48_OUT 0x101e57, BIT3
#define GPIO48_IN  0x101e51, BIT3

#define GPIO49_PAD PAD_GPIO12
#define GPIO49_OEN 0x101e5d, BIT4
#define GPIO49_OUT 0x101e57, BIT4
#define GPIO49_IN  0x101e51, BIT4

#define GPIO50_PAD PAD_I2S_IN_BCK
#define GPIO50_OEN 0x101e6e, BIT1
#define GPIO50_OUT 0x101e6c, BIT1
#define GPIO50_IN  0x101e6a, BIT1

#define GPIO51_PAD PAD_I2S_IN_SD
#define GPIO51_OEN 0x101e6e, BIT2
#define GPIO51_OUT 0x101e6c, BIT2
#define GPIO51_IN  0x101e6a, BIT2

#define GPIO52_PAD PAD_I2S_IN_WS
#define GPIO52_OEN 0x101e6e, BIT0
#define GPIO52_OUT 0x101e6c, BIT0
#define GPIO52_IN  0x101e6a, BIT0

#define GPIO53_PAD PAD_I2S_OUT_BCK
#define GPIO53_OEN 0x101e6e, BIT7
#define GPIO53_OUT 0x101e6c, BIT7
#define GPIO53_IN  0x101e6a, BIT7

#define GPIO54_PAD PAD_I2S_OUT_MCK
#define GPIO54_OEN 0x101e6e, BIT6
#define GPIO54_OUT 0x101e6c, BIT6
#define GPIO54_IN  0x101e6a, BIT6

#define GPIO55_PAD PAD_I2S_OUT_SD
#define GPIO55_OEN 0x101e6f, BIT0
#define GPIO55_OUT 0x101e6d, BIT0
#define GPIO55_IN  0x101e6b, BIT0

#define GPIO56_PAD PAD_I2S_OUT_SD1
#define GPIO56_OEN 0x101e6f, BIT1
#define GPIO56_OUT 0x101e6d, BIT1
#define GPIO56_IN  0x101e6b, BIT1

#define GPIO57_PAD PAD_I2S_OUT_SD2
#define GPIO57_OEN 0x101e6f, BIT2
#define GPIO57_OUT 0x101e6d, BIT2
#define GPIO57_IN  0x101e6b, BIT2

#define GPIO58_PAD PAD_I2S_OUT_WS
#define GPIO58_OEN 0x101e6e, BIT5
#define GPIO58_OUT 0x101e6c, BIT5
#define GPIO58_IN  0x101e6a, BIT5

#define GPIO59_PAD PAD_PWM0
#define GPIO59_OEN 0x121096, BIT0
#define GPIO59_OUT 0x121098, BIT0
#define GPIO59_IN  0x121094, BIT0

#define GPIO60_PAD PAD_PWM1
#define GPIO60_OEN 0x121096, BIT1
#define GPIO60_OUT 0x121098, BIT1
#define GPIO60_IN  0x121094, BIT1

#define GPIO61_PAD PAD_PWM2
#define GPIO61_OEN 0x121096, BIT2
#define GPIO61_OUT 0x121098, BIT2
#define GPIO61_IN  0x121094, BIT2

#define GPIO62_PAD PAD_SPDIF_IN
#define GPIO62_OEN 0x101e6e, BIT3
#define GPIO62_OUT 0x101e6c, BIT3
#define GPIO62_IN  0x101e6a, BIT3

#define GPIO63_PAD PAD_SPDIF_OUT
#define GPIO63_OEN 0x101e6e, BIT4
#define GPIO63_OUT 0x101e6c, BIT4
#define GPIO63_IN  0x101e6a, BIT4

#define GPIO64_PAD PAD_SPI0_SCK
#define GPIO64_OEN 0x101e5f, BIT2
#define GPIO64_OUT 0x101e59, BIT2
#define GPIO64_IN  0x101e53, BIT2

#define GPIO65_PAD PAD_SPI0_MOSI
#define GPIO65_OEN 0x101e5f, BIT1
#define GPIO65_OUT 0x101e59, BIT1
#define GPIO65_IN  0x101e53, BIT1

#define GPIO66_PAD PAD_TGPIO2
#define GPIO66_OEN 0x12109c, BIT0
#define GPIO66_OUT 0x12109c, BIT4
#define GPIO66_IN  0x12109d, BIT0

#define GPIO67_PAD PAD_TGPIO3
#define GPIO67_OEN 0x12109c, BIT1
#define GPIO67_OUT 0x12109c, BIT5
#define GPIO67_IN  0x12109d, BIT1

#define GPIO68_PAD PAD_TS0_CLK
#define GPIO68_OEN 0x101e68, BIT2
#define GPIO68_OUT 0x101e64, BIT2
#define GPIO68_IN  0x101e62, BIT2

#define GPIO69_PAD PAD_TS0_D0
#define GPIO69_OEN 0x101e68, BIT3
#define GPIO69_OUT 0x101e64, BIT3
#define GPIO69_IN  0x101e62, BIT3

#define GPIO70_PAD PAD_TS0_D1
#define GPIO70_OEN 0x101e68, BIT4
#define GPIO70_OUT 0x101e64, BIT4
#define GPIO70_IN  0x101e62, BIT4

#define GPIO71_PAD PAD_TS0_D2
#define GPIO71_OEN 0x101e68, BIT5
#define GPIO71_OUT 0x101e64, BIT5
#define GPIO71_IN  0x101e62, BIT5

#define GPIO72_PAD PAD_TS0_D3
#define GPIO72_OEN 0x101e68, BIT6
#define GPIO72_OUT 0x101e64, BIT6
#define GPIO72_IN  0x101e62, BIT6

#define GPIO73_PAD PAD_TS0_D4
#define GPIO73_OEN 0x101e68, BIT7
#define GPIO73_OUT 0x101e64, BIT7
#define GPIO73_IN  0x101e62, BIT7

#define GPIO74_PAD PAD_TS0_D5
#define GPIO74_OEN 0x101e69, BIT0
#define GPIO74_OUT 0x101e65, BIT0
#define GPIO74_IN  0x101e63, BIT0

#define GPIO75_PAD PAD_TS0_D6
#define GPIO75_OEN 0x101e69, BIT1
#define GPIO75_OUT 0x101e65, BIT1
#define GPIO75_IN  0x101e63, BIT1

#define GPIO76_PAD PAD_TS0_D7
#define GPIO76_OEN 0x101e69, BIT2
#define GPIO76_OUT 0x101e65, BIT2
#define GPIO76_IN  0x101e63, BIT2

#define GPIO77_PAD PAD_TS0_SYNC
#define GPIO77_OEN 0x101e68, BIT1
#define GPIO77_OUT 0x101e64, BIT1
#define GPIO77_IN  0x101e62, BIT1

#define GPIO78_PAD PAD_TS0_VLD
#define GPIO78_OEN 0x101e68, BIT0
#define GPIO78_OUT 0x101e64, BIT0
#define GPIO78_IN  0x101e62, BIT0

#define GPIO79_PAD PAD_MIC_BCK
#define GPIO79_OEN 0x101e5d, BIT7
#define GPIO79_OUT 0x101e57, BIT7
#define GPIO79_IN  0x101e51, BIT7

#define GPIO80_PAD PAD_MIC_SD0
#define GPIO80_OEN 0x101e5e, BIT0
#define GPIO80_OUT 0x101e58, BIT0
#define GPIO80_IN  0x101e52, BIT0

#define GPIO81_PAD PAD_MIC_SD1
#define GPIO81_OEN 0x101e5e, BIT1
#define GPIO81_OUT 0x101e58, BIT1
#define GPIO81_IN  0x101e52, BIT1

#define GPIO82_PAD PAD_MIC_SD2
#define GPIO82_OEN 0x101e5e, BIT2
#define GPIO82_OUT 0x101e58, BIT2
#define GPIO82_IN  0x101e52, BIT2

#define GPIO83_PAD PAD_DISEQC_OUT
#define GPIO83_OEN 0x101e5e, BIT3
#define GPIO83_OUT 0x101e58, BIT3
#define GPIO83_IN  0x101e52, BIT3

#define GPIO84_PAD PAD_DISEQC_IN
#define GPIO84_OEN 0x101e5e, BIT4
#define GPIO84_OUT 0x101e58, BIT4
#define GPIO84_IN  0x101e52, BIT4

#define GPIO85_PAD PAD_NAND_CLE
#define GPIO85_OEN 0x101e88, BIT4
#define GPIO85_OUT 0x101e89, BIT4
#define GPIO85_IN  0x101e8A, BIT4

#define GPIO86_PAD PAD_NAND_ALE
#define GPIO86_OEN 0x101e88, BIT5
#define GPIO86_OUT 0x101e89, BIT5
#define GPIO86_IN  0x101e8A, BIT5

#define GPIO87_PAD PAD_NAND_WEZ
#define GPIO87_OEN 0x101e88, BIT6
#define GPIO87_OUT 0x101e89, BIT6
#define GPIO87_IN  0x101e8A, BIT6

#define GPIO88_PAD PAD_NAND_WPZ
#define GPIO88_OEN 0x101e88, BIT7
#define GPIO88_OUT 0x101e89, BIT7
#define GPIO88_IN  0x101e8A, BIT7

#define GPIO89_PAD PAD_SD_CMD
#define GPIO89_OEN 0x101e5e, BIT5
#define GPIO89_OUT 0x101e58, BIT5
#define GPIO89_IN  0x101e52, BIT5

#define GPIO90_PAD PAD_SD_CLK
#define GPIO90_OEN 0x101e5e, BIT6
#define GPIO90_OUT 0x101e58, BIT6
#define GPIO90_IN  0x101e52, BIT6

#define GPIO91_PAD PAD_TS1_CLK
#define GPIO91_OEN 0x101e4e, BIT1
#define GPIO91_OUT 0x101e4c, BIT1
#define GPIO91_IN  0x101e4a, BIT1

#define GPIO92_PAD PAD_TS1_D0
#define GPIO92_OEN 0x101e4e, BIT3
#define GPIO92_OUT 0x101e4c, BIT3
#define GPIO92_IN  0x101e4a, BIT3

#define GPIO93_PAD PAD_TS1_D1
#define GPIO93_OEN 0x101e4e, BIT4
#define GPIO93_OUT 0x101e4c, BIT4
#define GPIO93_IN  0x101e4a, BIT4

#define GPIO94_PAD PAD_TS1_D2
#define GPIO94_OEN 0x101e4e, BIT5
#define GPIO94_OUT 0x101e4c, BIT5
#define GPIO94_IN  0x101e4a, BIT5

#define GPIO95_PAD PAD_TS1_D3
#define GPIO95_OEN 0x101e4e, BIT6
#define GPIO95_OUT 0x101e4c, BIT6
#define GPIO95_IN  0x101e4a, BIT6

#define GPIO96_PAD PAD_TS1_D4
#define GPIO96_OEN 0x101e4e, BIT7
#define GPIO96_OUT 0x101e4c, BIT7
#define GPIO96_IN  0x101e4a, BIT7

#define GPIO97_PAD PAD_TS1_D5
#define GPIO97_OEN 0x101e4f, BIT0
#define GPIO97_OUT 0x101e4d, BIT0
#define GPIO97_IN  0x101e4b, BIT0

#define GPIO98_PAD PAD_TS1_D6
#define GPIO98_OEN 0x101e4f, BIT1
#define GPIO98_OUT 0x101e4d, BIT1
#define GPIO98_IN  0x101e4b, BIT1

#define GPIO99_PAD PAD_TS1_D7
#define GPIO99_OEN 0x101e4f, BIT2
#define GPIO99_OUT 0x101e4d, BIT2
#define GPIO99_IN  0x101e4b, BIT2

#define GPIO100_PAD PAD_TS1_SYNC
#define GPIO100_OEN 0x101e4e, BIT2
#define GPIO100_OUT 0x101e4c, BIT2
#define GPIO100_IN  0x101e4a, BIT2

#define GPIO101_PAD PAD_TS1_VLD
#define GPIO101_OEN 0x101e4e, BIT0
#define GPIO101_OUT 0x101e4c, BIT0
#define GPIO101_IN  0x101e4a, BIT0

#define GPIO102_PAD PAD_PCM_A0
#define GPIO102_OEN 0x101e7f, BIT3
#define GPIO102_OUT 0x101e79, BIT3
#define GPIO102_IN  0x101e73, BIT3

#define GPIO103_PAD PAD_PCM_A1
#define GPIO103_OEN 0x101e7f, BIT2
#define GPIO103_OUT 0x101e79, BIT2
#define GPIO103_IN  0x101e73, BIT2

#define GPIO104_PAD PAD_PCM_A2
#define GPIO104_OEN 0x101e7f, BIT0
#define GPIO104_OUT 0x101e79, BIT0
#define GPIO104_IN  0x101e73, BIT0

#define GPIO105_PAD PAD_PCM_A3
#define GPIO105_OEN 0x101e7e, BIT7
#define GPIO105_OUT 0x101e78, BIT7
#define GPIO105_IN  0x101e72, BIT7

#define GPIO106_PAD PAD_PCM_A4
#define GPIO106_OEN 0x101e7e, BIT6
#define GPIO106_OUT 0x101e78, BIT6
#define GPIO106_IN  0x101e72, BIT6

#define GPIO107_PAD PAD_PCM_A5
#define GPIO107_OEN 0x101e7e, BIT4
#define GPIO107_OUT 0x101e78, BIT4
#define GPIO107_IN  0x101e72, BIT4

#define GPIO108_PAD PAD_PCM_A6
#define GPIO108_OEN 0x101e7e, BIT3
#define GPIO108_OUT 0x101e78, BIT3
#define GPIO108_IN  0x101e72, BIT3

#define GPIO109_PAD PAD_PCM_A7
#define GPIO109_OEN 0x101e7e, BIT2
#define GPIO109_OUT 0x101e78, BIT2
#define GPIO109_IN  0x101e72, BIT2

#define GPIO110_PAD PAD_PCM_A8
#define GPIO110_OEN 0x101e7d, BIT4
#define GPIO110_OUT 0x101e77, BIT4
#define GPIO110_IN  0x101e71, BIT4

#define GPIO111_PAD PAD_PCM_A9
#define GPIO111_OEN 0x101e7d, BIT2
#define GPIO111_OUT 0x101e77, BIT2
#define GPIO111_IN  0x101e71, BIT2

#define GPIO112_PAD PAD_PCM_A10
#define GPIO112_OEN 0x101e7c, BIT6
#define GPIO112_OUT 0x101e76, BIT6
#define GPIO112_IN  0x101e70, BIT6

#define GPIO113_PAD PAD_PCM_A11
#define GPIO113_OEN 0x101e7d, BIT0
#define GPIO113_OUT 0x101e77, BIT0
#define GPIO113_IN  0x101e71, BIT0

#define GPIO114_PAD PAD_PCM_A12
#define GPIO114_OEN 0x101e7e, BIT1
#define GPIO114_OUT 0x101e78, BIT1
#define GPIO114_IN  0x101e72, BIT1

#define GPIO115_PAD PAD_PCM_A13
#define GPIO115_OEN 0x101e7d, BIT5
#define GPIO115_OUT 0x101e77, BIT5
#define GPIO115_IN  0x101e71, BIT5

#define GPIO116_PAD PAD_PCM_A14
#define GPIO116_OEN 0x101e7d, BIT6
#define GPIO116_OUT 0x101e77, BIT6
#define GPIO116_IN  0x101e71, BIT6

#define GPIO117_PAD PAD_PCM_D0
#define GPIO117_OEN 0x101e7f, BIT4
#define GPIO117_OUT 0x101e79, BIT4
#define GPIO117_IN  0x101e73, BIT4

#define GPIO118_PAD PAD_PCM_D1
#define GPIO118_OEN 0x101e7f, BIT5
#define GPIO118_OUT 0x101e79, BIT5
#define GPIO118_IN  0x101e73, BIT5

#define GPIO119_PAD PAD_PCM_D2
#define GPIO119_OEN 0x101e7f, BIT6
#define GPIO119_OUT 0x101e79, BIT6
#define GPIO119_IN  0x101e73, BIT6

#define GPIO120_PAD PAD_PCM_D3
#define GPIO120_OEN 0x101e7c, BIT0
#define GPIO120_OUT 0x101e76, BIT0
#define GPIO120_IN  0x101e70, BIT0

#define GPIO121_PAD PAD_PCM_D4
#define GPIO121_OEN 0x101e7c, BIT1
#define GPIO121_OUT 0x101e76, BIT1
#define GPIO121_IN  0x101e70, BIT1

#define GPIO122_PAD PAD_PCM_D5
#define GPIO122_OEN 0x101e7c, BIT2
#define GPIO122_OUT 0x101e76, BIT2
#define GPIO122_IN  0x101e70, BIT2

#define GPIO123_PAD PAD_PCM_D6
#define GPIO123_OEN 0x101e7c, BIT3
#define GPIO123_OUT 0x101e76, BIT3
#define GPIO123_IN  0x101e70, BIT3

#define GPIO124_PAD PAD_PCM_D7
#define GPIO124_OEN 0x101e7c, BIT4
#define GPIO124_OUT 0x101e76, BIT4
#define GPIO124_IN  0x101e70, BIT4

#define GPIO125_PAD PAD_PCM_IOWR_N
#define GPIO125_OEN 0x101e7d, BIT3
#define GPIO125_OUT 0x101e77, BIT3
#define GPIO125_IN  0x101e71, BIT3

#define GPIO126_PAD PAD_PCM_WE_N
#define GPIO126_OEN 0x101e7d, BIT7
#define GPIO126_OUT 0x101e77, BIT7
#define GPIO126_IN  0x101e71, BIT7

#define GPIO127_PAD PAD_PCM_IRQA_N
#define GPIO127_OEN 0x101e7e, BIT0
#define GPIO127_OUT 0x101e78, BIT0
#define GPIO127_IN  0x101e72, BIT0

#define GPIO128_PAD PAD_PCM_WAIT_N
#define GPIO128_OEN 0x101e7e, BIT5
#define GPIO128_OUT 0x101e78, BIT5
#define GPIO128_IN  0x101e72, BIT5

#define GPIO129_PAD PAD_PCM_RESET
#define GPIO129_OEN 0x101e7f, BIT7
#define GPIO129_OUT 0x101e79, BIT7
#define GPIO129_IN  0x101e73, BIT7

#define GPIO130_PAD PAD_PCM_CD_N
#define GPIO130_OEN 0x101e80, BIT0
#define GPIO130_OUT 0x101e7a, BIT0
#define GPIO130_IN  0x101e74, BIT0

#define GPIO131_PAD PAD_PCM_CE_N
#define GPIO131_OEN 0x101e7c, BIT5
#define GPIO131_OUT 0x101e76, BIT5
#define GPIO131_IN  0x101e70, BIT5

#define GPIO132_PAD PAD_PCM_OE_N
#define GPIO132_OEN 0x101e7c, BIT7
#define GPIO132_OUT 0x101e76, BIT7
#define GPIO132_IN  0x101e70, BIT7

#define GPIO133_PAD PAD_PCM_IORD_N
#define GPIO133_OEN 0x101e7d, BIT1
#define GPIO133_OUT 0x101e77, BIT1
#define GPIO133_IN  0x101e71, BIT1

#define GPIO134_PAD PAD_PCM_REG_N
#define GPIO134_OEN 0x101e7f, BIT1
#define GPIO134_OUT 0x101e79, BIT1
#define GPIO134_IN  0x101e73, BIT1

#define GPIO135_PAD PAD_SPI0_SCZ
#define GPIO135_OEN 0x101e5e, BIT7
#define GPIO135_OUT 0x101e58, BIT7
#define GPIO135_IN  0x101e52, BIT7

#define GPIO136_PAD PAD_SPI0_MISO
#define GPIO136_OEN 0x101e5f, BIT0
#define GPIO136_OUT 0x101e59, BIT0
#define GPIO136_IN  0x101e53, BIT0

/*
#define GPIO137_PAD PAD_PWM_DAC0
#define GPIO137_OEN 0x101e5d, BIT5
#define GPIO137_OUT 0x101e57, BIT5
#define GPIO137_IN  0x101e51, BIT5

#define GPIO138_PAD PAD_PWM_DAC1
#define GPIO138_OEN 0x101e5d, BIT6
#define GPIO138_OUT 0x101e57, BIT6
#define GPIO138_IN  0x101e51, BIT6
*/

#define GPIO139_PAD PAD_HDMIRX_ARCTX
#define GPIO139_OEN 0x101efe, BIT2
#define GPIO139_OUT 0x101efe, BIT1
#define GPIO139_IN  0x101efe, BIT3

/* analog gpio
 * must add __GPIO(X) in below gpio_table
 */
/*
*/

/* ext_gpio_int, and access it via u8 */
#define GPIO_EXT0_MSK  0x10196f, BIT7
#define GPIO_EXT0_POL  0x101977, BIT7
#define GPIO_EXT0_CLR  0x10197f, BIT7
#define GPIO_EXT0_STS  0x10197f, BIT7

#define GPIO_EXT1_MSK  0x10196f, BIT7
#define GPIO_EXT1_POL  0x101977, BIT7
#define GPIO_EXT1_CLR  0x10197f, BIT7
#define GPIO_EXT1_STS  0x10197f, BIT7

#define GPIO_EXT2_MSK  0x10196f, BIT7
#define GPIO_EXT2_POL  0x101977, BIT7
#define GPIO_EXT2_CLR  0x10197f, BIT7
#define GPIO_EXT2_STS  0x10197f, BIT7

#define GPIO_EXT3_MSK  0x10196f, BIT7
#define GPIO_EXT3_POL  0x101977, BIT7
#define GPIO_EXT3_CLR  0x10197f, BIT7
#define GPIO_EXT3_STS  0x10197f, BIT7

#define GPIO_EXT4_MSK  0x10196f, BIT7
#define GPIO_EXT4_POL  0x101977, BIT7
#define GPIO_EXT4_CLR  0x10197f, BIT7
#define GPIO_EXT4_STS  0x10197f, BIT7

#define GPIO_EXT5_MSK  0x10196f, BIT7
#define GPIO_EXT5_POL  0x101977, BIT7
#define GPIO_EXT5_CLR  0x10197f, BIT7
#define GPIO_EXT5_STS  0x10197f, BIT7

#define GPIO_EXT6_MSK  0x10196f, BIT7
#define GPIO_EXT6_POL  0x101977, BIT7
#define GPIO_EXT6_CLR  0x10197f, BIT7
#define GPIO_EXT6_STS  0x10197f, BIT7

#define GPIO_EXT7_MSK  0x10196f, BIT7
#define GPIO_EXT7_POL  0x101977, BIT7
#define GPIO_EXT7_CLR  0x10197f, BIT7
#define GPIO_EXT7_STS  0x10197f, BIT7

//-------------------------------------------------------------------------------------------------
//  Local Structures
//-------------------------------------------------------------------------------------------------





//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
#if (GPIO_PM_INT_SUPPORTED)

#define PM_INT_COUNT   11

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
    5,    /* PAD_IRIN */
    6,    /* PAD_PWM0_PM */
    9,    /* PAD_GPIO0_PM */
    10,   /* PAD_GPIO1_PM */
    11,   /* PAD_GPIO2_PM */
    12,   /* PAD_USB_CTRL */
    13,   /* PAD_GPIO5_PM */
    14,   /* PAD_GPIO6_PM */
    15,   /* PAD_GPIO7_PM */
    16,   /* PAD_GPIO8_PM */
    25    /* PAD_WOL_INT_OUT */
};

const U32 PM_gpio_IRQreg[PM_INT_COUNT] =
{
    0x0f26,    /* PAD_IRIN */
    0x0f28,    /* PAD_PWM0_PM */
    0x0f00,    /* PAD_GPIO0_PM */
    0x0f02,    /* PAD_GPIO1_PM */
    0x0f04,    /* PAD_GPIO2_PM */
    0x0f06,    /* PAD_USB_CTRL */
    0x0f0a,    /* PAD_GPIO5_PM */
    0x0f0c,    /* PAD_GPIO6_PM */
    0x0f0e,    /* PAD_GPIO7_PM */
    0x0f10,    /* PAD_GPIO8_PM */
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
    44,     //PAD_GPIO7
    45,     //PAD_GPIO8
    46,     //PAD_GPIO9
    47,     //PAD_GPIO10
    37,     //PAD_GPIO0
    38,     //PAD_GPIO1
    39,     //PAD_GPIO2
    40      //PAD_GPIO3
};

const int gpio_IRQnum[INT_COUNT]=
{
	GPIO_IRQNUM
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

    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(3), __GPIO(4),
    __GPIO(5), __GPIO(6), __GPIO(7), __GPIO(999), __GPIO(9),
    __GPIO(10), __GPIO(11), __GPIO(12), __GPIO(13), __GPIO(14),
    __GPIO(15), __GPIO(16), __GPIO(17), __GPIO(18), __GPIO(19),
    __GPIO(20), __GPIO(21), __GPIO(22), __GPIO(23), __GPIO(24),
    __GPIO(25), __GPIO(26), __GPIO(27), __GPIO(28), __GPIO(29),
    __GPIO(30), __GPIO(31), __GPIO(999), __GPIO(999), __GPIO(999),
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
    __GPIO(135), __GPIO(136), __GPIO(999), __GPIO(999), __GPIO(139)
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
    MHal_GPIO_REG(REG_ALL_PAD_IN) &= ~BIT7;
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
#if defined(GPIO_OEN_INVERSE) && (GPIO_OEN_INVERSE)
    if ((u32IndexGPIO >= GPIO_NUM_OENIVS) && (u32IndexGPIO <= GPIO_NUM_OENIVE))
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
    else
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
#else
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
#endif /* GPIO_OEN_INVERSE */
}

void MHal_GPIO_Pad_Odn(U32 u32IndexGPIO)
{
#if defined(GPIO_OEN_INVERSE) && (GPIO_OEN_INVERSE)
    if ((u32IndexGPIO >= GPIO_NUM_OENIVS) && (u32IndexGPIO <= GPIO_NUM_OENIVE))
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
    else
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
#else
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
#endif /* GPIO_OEN_INVERSE */
}

U8 MHal_GPIO_Pad_Level(U32 u32IndexGPIO)
{
    return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_in)&gpio_table[u32IndexGPIO].m_in)? 1 : 0);
}

U8 MHal_GPIO_Pad_InOut(U32 u32IndexGPIO)
{
#if defined(GPIO_OEN_INVERSE) && (GPIO_OEN_INVERSE)
    if ((u32IndexGPIO >= GPIO_NUM_OENIVS) && (u32IndexGPIO <= GPIO_NUM_OENIVE))
        return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen)&gpio_table[u32IndexGPIO].m_oen)? 0 : 1);
    else
        return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen)&gpio_table[u32IndexGPIO].m_oen)? 1 : 0);
#else
    return ((MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen)&gpio_table[u32IndexGPIO].m_oen)? 1 : 0);
#endif /* GPIO_OEN_INVERSE */
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
#if defined(GPIO_OEN_INVERSE) && (GPIO_OEN_INVERSE)
    if ((u32IndexGPIO >= GPIO_NUM_OENIVS) && (u32IndexGPIO <= GPIO_NUM_OENIVE))
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
    else
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
#else
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
#endif /* GPIO_OEN_INVERSE */
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_out) |= gpio_table[u32IndexGPIO].m_out;
}

void MHal_GPIO_Set_Low(U32 u32IndexGPIO)
{
#if defined(GPIO_OEN_INVERSE) && (GPIO_OEN_INVERSE)
    if ((u32IndexGPIO >= GPIO_NUM_OENIVS) && (u32IndexGPIO <= GPIO_NUM_OENIVE))
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
    else
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
#else
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
#endif /* GPIO_OEN_INVERSE */
    MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_out) &= (~gpio_table[u32IndexGPIO].m_out);
}

void MHal_GPIO_Set_Input(U32 u32IndexGPIO)
{
#if defined(GPIO_OEN_INVERSE) && (GPIO_OEN_INVERSE)
    if ((u32IndexGPIO >= GPIO_NUM_OENIVS) && (u32IndexGPIO <= GPIO_NUM_OENIVE))
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) &= (~gpio_table[u32IndexGPIO].m_oen);
    else
        MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
#else
	MHal_GPIO_REG(gpio_table[u32IndexGPIO].r_oen) |= gpio_table[u32IndexGPIO].m_oen;
#endif /* GPIO_OEN_INVERSE */
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
            if(request_irq(gpio_IRQnum[i], (irq_handler_t)pCallback, 0x0, "GPIO_NONPM", dev_id))
            {
                printk("request_irq fail\n");
                return -EBUSY;
            }
            MHal_GPIO_Pad_Odn(gpio_num);
        }
    }

/*
   current no E_IRQEXPL_PM_IRQ , need owner to fix it 
*/
#if 0
#if (GPIO_PM_INT_SUPPORTED)
    for(i=0; i<PM_INT_COUNT; i++)
    {
        if(PM_gpio_IntPad[i] == gpio_num)
        {
            MHal_GPIO_WriteRegBit(PM_IRQ_SRC[0], 0, BIT_PM_PMSLEEP_IRQ_MASK);
            MHal_GPIO_WriteRegBit(PM_gpio_IRQreg[i], 0,  BIT_PM_GPIO_INT_MASK);
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
        }
    }
#endif
#endif
    return 0;
}

int MHal_GPIO_Disable_Interrupt(int gpio_num)
{
    U8 i;

    for(i = 0; i < INT_COUNT; i++)
    {
      if(gpio_IntPad[i] == gpio_num)
      {
         MHal_GPIO_WriteRegBit(gpio_irq_table[i].msk_reg, 1, gpio_irq_table[i].m_msk);
         gpio_irq_pCallback[i] = NULL;
         gpio_irq_dev_id[i] = NULL;
      }
    }

    return 0;
}
