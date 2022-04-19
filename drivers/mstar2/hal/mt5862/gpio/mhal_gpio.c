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

#define GPIO6_PAD PAD_PWM_PM
#define GPIO6_OEN 0x0f28, BIT0
#define GPIO6_OUT 0x0f28, BIT1
#define GPIO6_IN  0x0f28, BIT2

#define GPIO7_PAD PAD_GPIO0_PM
#define GPIO7_OEN 0x0f00, BIT0
#define GPIO7_OUT 0x0f00, BIT1
#define GPIO7_IN  0x0f00, BIT2

#define GPIO8_PAD PAD_GPIO1_PM
#define GPIO8_OEN 0x0f02, BIT0
#define GPIO8_OUT 0x0f02, BIT1
#define GPIO8_IN  0x0f02, BIT2

#define GPIO9_PAD PAD_GPIO2_PM
#define GPIO9_OEN 0x0f04, BIT0
#define GPIO9_OUT 0x0f04, BIT1
#define GPIO9_IN  0x0f04, BIT2

#define GPIO10_PAD PAD_USB_CTRL
#define GPIO10_OEN 0x0f06, BIT0
#define GPIO10_OUT 0x0f06, BIT1
#define GPIO10_IN  0x0f06, BIT2

#define GPIO11_PAD PAD_GPIO5_PM
#define GPIO11_OEN 0x0f0a, BIT0
#define GPIO11_OUT 0x0f0a, BIT1
#define GPIO11_IN  0x0f0a, BIT2

#define GPIO12_PAD PAD_GPIO6_PM
#define GPIO12_OEN 0x0f0c, BIT0
#define GPIO12_OUT 0x0f0c, BIT1
#define GPIO12_IN  0x0f0c, BIT2

#define GPIO13_PAD PAD_GPIO7_PM
#define GPIO13_OEN 0x0f0e, BIT0
#define GPIO13_OUT 0x0f0e, BIT1
#define GPIO13_IN  0x0f0e, BIT2

#define GPIO14_PAD PAD_GPIO8_PM
#define GPIO14_OEN 0x0f10, BIT0
#define GPIO14_OUT 0x0f10, BIT1
#define GPIO14_IN  0x0f10, BIT2

#define GPIO15_PAD PAD_GPIO9_PM
#define GPIO15_OEN 0x0f12, BIT0
#define GPIO15_OUT 0x0f12, BIT1
#define GPIO15_IN  0x0f12, BIT2

#define GPIO16_PAD PAD_GPIO10_PM
#define GPIO16_OEN 0x0f14, BIT0
#define GPIO16_OUT 0x0f14, BIT1
#define GPIO16_IN  0x0f14, BIT2

#define GPIO17_PAD PAD_GPIO11_PM
#define GPIO17_OEN 0x0f16, BIT0
#define GPIO17_OUT 0x0f16, BIT1
#define GPIO17_IN  0x0f16, BIT2

#define GPIO18_PAD PAD_GPIO12_PM
#define GPIO18_OEN 0x0f18, BIT0
#define GPIO18_OUT 0x0f18, BIT1
#define GPIO18_IN  0x0f18, BIT2

#define GPIO19_PAD PAD_GPIO13_PM
#define GPIO19_OEN 0x0f1a, BIT0
#define GPIO19_OUT 0x0f1a, BIT1
#define GPIO19_IN  0x0f1a, BIT2

#define GPIO20_PAD PAD_GPIO14_PM
#define GPIO20_OEN 0x0f1c, BIT0
#define GPIO20_OUT 0x0f1c, BIT1
#define GPIO20_IN  0x0f1c, BIT2

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

#define GPIO32_PAD PAD_ARC0
#define GPIO32_OEN 0x101efe, BIT2
#define GPIO32_OUT 0x101efe, BIT1
#define GPIO32_IN  0x101efe, BIT3

#define GPIO33_PAD PAD_DDCR_CK
#define GPIO33_OEN 0x12109a, BIT1
#define GPIO33_OUT 0x12109a, BIT3
#define GPIO33_IN  0x12109a, BIT5

#define GPIO34_PAD PAD_DDCR_DA
#define GPIO34_OEN 0x12109a, BIT0
#define GPIO34_OUT 0x12109a, BIT2
#define GPIO34_IN  0x12109a, BIT4

#define GPIO35_PAD PAD_GPIO0
#define GPIO35_OEN 0x101e5c, BIT0
#define GPIO35_OUT 0x101e56, BIT0
#define GPIO35_IN  0x101e50, BIT0

#define GPIO36_PAD PAD_GPIO1
#define GPIO36_OEN 0x101e5c, BIT1
#define GPIO36_OUT 0x101e56, BIT1
#define GPIO36_IN  0x101e50, BIT1

#define GPIO37_PAD PAD_GPIO2
#define GPIO37_OEN 0x101e5c, BIT2
#define GPIO37_OUT 0x101e56, BIT2
#define GPIO37_IN  0x101e50, BIT2

#define GPIO38_PAD PAD_GPIO3
#define GPIO38_OEN 0x101e5c, BIT3
#define GPIO38_OUT 0x101e56, BIT3
#define GPIO38_IN  0x101e50, BIT3

#define GPIO39_PAD PAD_GPIO4
#define GPIO39_OEN 0x101e5c, BIT4
#define GPIO39_OUT 0x101e56, BIT4
#define GPIO39_IN  0x101e50, BIT4

#define GPIO40_PAD PAD_GPIO5
#define GPIO40_OEN 0x101e5c, BIT5
#define GPIO40_OUT 0x101e56, BIT5
#define GPIO40_IN  0x101e50, BIT5

#define GPIO41_PAD PAD_GPIO6
#define GPIO41_OEN 0x101e5c, BIT6
#define GPIO41_OUT 0x101e56, BIT6
#define GPIO41_IN  0x101e50, BIT6

#define GPIO42_PAD PAD_GPIO7
#define GPIO42_OEN 0x101e5c, BIT7
#define GPIO42_OUT 0x101e56, BIT7
#define GPIO42_IN  0x101e50, BIT7

#define GPIO43_PAD PAD_GPIO8
#define GPIO43_OEN 0x101e5d, BIT0
#define GPIO43_OUT 0x101e57, BIT0
#define GPIO43_IN  0x101e51, BIT0

#define GPIO44_PAD PAD_GPIO9
#define GPIO44_OEN 0x101e5d, BIT1
#define GPIO44_OUT 0x101e57, BIT1
#define GPIO44_IN  0x101e51, BIT1

#define GPIO45_PAD PAD_GPIO10
#define GPIO45_OEN 0x101e5d, BIT2
#define GPIO45_OUT 0x101e57, BIT2
#define GPIO45_IN  0x101e51, BIT2

#define GPIO46_PAD PAD_GPIO11
#define GPIO46_OEN 0x101e5d, BIT3
#define GPIO46_OUT 0x101e57, BIT3
#define GPIO46_IN  0x101e51, BIT3

#define GPIO47_PAD PAD_GPIO12
#define GPIO47_OEN 0x101e5d, BIT4
#define GPIO47_OUT 0x101e57, BIT4
#define GPIO47_IN  0x101e51, BIT4

#define GPIO48_PAD PAD_GPIO13
#define GPIO48_OEN 0x101e5d, BIT5
#define GPIO48_OUT 0x101e57, BIT5
#define GPIO48_IN  0x101e51, BIT5

#define GPIO49_PAD PAD_GPIO14
#define GPIO49_OEN 0x101e5d, BIT6
#define GPIO49_OUT 0x101e57, BIT6
#define GPIO49_IN  0x101e51, BIT6

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

#define GPIO56_PAD PAD_I2S_OUT_WS
#define GPIO56_OEN 0x101e6e, BIT5
#define GPIO56_OUT 0x101e6c, BIT5
#define GPIO56_IN  0x101e6a, BIT5

#define GPIO57_PAD PAD_PWM0
#define GPIO57_OEN 0x121096, BIT0
#define GPIO57_OUT 0x121098, BIT0
#define GPIO57_IN  0x121094, BIT0

#define GPIO58_PAD PAD_PWM1
#define GPIO58_OEN 0x121096, BIT1
#define GPIO58_OUT 0x121098, BIT1
#define GPIO58_IN  0x121094, BIT1

#define GPIO59_PAD PAD_PWM2
#define GPIO59_OEN 0x121096, BIT2
#define GPIO59_OUT 0x121098, BIT2
#define GPIO59_IN  0x121094, BIT2

#define GPIO60_PAD PAD_SPDIF_IN
#define GPIO60_OEN 0x101e6e, BIT3
#define GPIO60_OUT 0x101e6c, BIT3
#define GPIO60_IN  0x101e6a, BIT3

#define GPIO61_PAD PAD_SPDIF_OUT
#define GPIO61_OEN 0x101e6e, BIT4
#define GPIO61_OUT 0x101e6c, BIT4
#define GPIO61_IN  0x101e6a, BIT4

#define GPIO62_PAD PAD_SPI1_CK
#define GPIO62_OEN 0x101e90, BIT0
#define GPIO62_OUT 0x101e90, BIT4
#define GPIO62_IN  0x101e91, BIT0

#define GPIO63_PAD PAD_SPI1_DI
#define GPIO63_OEN 0x101e90, BIT1
#define GPIO63_OUT 0x101e90, BIT5
#define GPIO63_IN  0x101e91, BIT1

#define GPIO64_PAD PAD_TGPIO0
#define GPIO64_OEN 0x12109c, BIT0
#define GPIO64_OUT 0x12109c, BIT4
#define GPIO64_IN  0x12109d, BIT0

#define GPIO65_PAD PAD_TGPIO1
#define GPIO65_OEN 0x12109c, BIT1
#define GPIO65_OUT 0x12109c, BIT5
#define GPIO65_IN  0x12109d, BIT1

#define GPIO66_PAD PAD_TGPIO2
#define GPIO66_OEN 0x12109c, BIT2
#define GPIO66_OUT 0x12109c, BIT6
#define GPIO66_IN  0x12109d, BIT2

#define GPIO67_PAD PAD_TGPIO3
#define GPIO67_OEN 0x12109c, BIT3
#define GPIO67_OUT 0x12109c, BIT7
#define GPIO67_IN  0x12109d, BIT3

#define GPIO68_PAD PAD_TS0_CLK
#define GPIO68_OEN 0x101e68, BIT2
#define GPIO68_OUT 0x101e64, BIT2
#define GPIO68_IN  0x101e62, BIT2

#define GPIO69_PAD PAD_TS0_D0
#define GPIO69_OEN 0x101e68, BIT3
#define GPIO69_OUT 0x101e64, BIT3
#define GPIO69_IN  0x101e62, BIT3

#define GPIO70_PAD PAD_TS0_SYNC
#define GPIO70_OEN 0x101e68, BIT1
#define GPIO70_OUT 0x101e64, BIT1
#define GPIO70_IN  0x101e62, BIT1

#define GPIO71_PAD PAD_TS0_VLD
#define GPIO71_OEN 0x101e68, BIT0
#define GPIO71_OUT 0x101e64, BIT0
#define GPIO71_IN  0x101e62, BIT0

#define GPIO72_PAD PAD_MIC_BCK
#define GPIO72_OEN 0x101e5e, BIT2
#define GPIO72_OUT 0x101e58, BIT2
#define GPIO72_IN  0x12101e, BIT3

#define GPIO73_PAD PAD_MIC_SD0
#define GPIO73_OEN 0x101e5d, BIT7
#define GPIO73_OUT 0x101e57, BIT7
#define GPIO73_IN  0x12101e, BIT0

#define GPIO74_PAD PAD_MIC_SD1
#define GPIO74_OEN 0x101e5e, BIT0
#define GPIO74_OUT 0x101e58, BIT0
#define GPIO74_IN  0x12101e, BIT1

#define GPIO75_PAD PAD_MIC_SD2
#define GPIO75_OEN 0x101e5e, BIT1
#define GPIO75_OUT 0x101e58, BIT1
#define GPIO75_IN  0x12101e, BIT2

#define GPIO181_PAD PADA_CVBS1
#define GPIO181_OEN 0x102742, BIT1
#define GPIO181_OUT 0x102743, BIT1
#define GPIO181_IN  0x102744, BIT1

#define GPIO183_PAD PADA_CVBS_OUT1
#define GPIO183_OEN 0x102748, BIT0
#define GPIO183_OUT 0x102749, BIT0
#define GPIO183_IN  0x10274a, BIT0

#define GPIO198_PAD PAD_MOD_TX_P00
#define GPIO198_OEN 0x111e84, BIT0
#define GPIO198_OUT 0x111e88, BIT0
#define GPIO198_IN  0x111e8c, BIT0

#define GPIO199_PAD PAD_MOD_TX_N00
#define GPIO199_OEN 0x111e84, BIT1
#define GPIO199_OUT 0x111e88, BIT1
#define GPIO199_IN  0x111e8c, BIT1

#define GPIO200_PAD PAD_MOD_TX_P01
#define GPIO200_OEN 0x111e84, BIT2
#define GPIO200_OUT 0x111e88, BIT2
#define GPIO200_IN  0x111e8c, BIT2

#define GPIO201_PAD PAD_MOD_TX_N01
#define GPIO201_OEN 0x111e84, BIT3
#define GPIO201_OUT 0x111e88, BIT3
#define GPIO201_IN  0x111e8c, BIT3

/* analog gpio
 * must add __GPIO(X) in below gpio_table
 */
/*
#define GPIO178_PAD PADA_RIN0P
#define GPIO178_OEN 0x102536, BIT0
#define GPIO178_OUT 0x102537, BIT0
#define GPIO178_IN  0x10253C, BIT0

#define GPIO179_PAD PADA_CVBS0
#define GPIO179_OEN 0x102742, BIT0
#define GPIO179_OUT 0x102743, BIT0
#define GPIO179_IN  0x102744, BIT0

#define GPIO180_PAD PADA_GIN0P
#define GPIO180_OEN 0x102538, BIT0
#define GPIO180_OUT 0x102539, BIT0
#define GPIO180_IN  0x10253D, BIT0

#define GPIO181_PAD PADA_CVBS1
#define GPIO181_OEN 0x102742, BIT1
#define GPIO181_OUT 0x102743, BIT1
#define GPIO181_IN  0x102744, BIT1

#define GPIO182_PAD PADA_GIN0M
#define GPIO182_OEN 0x102538, BIT4
#define GPIO182_OUT 0x102539, BIT4
#define GPIO182_IN  0x10253D, BIT4

#define GPIO184_PAD PADA_BIN0P
#define GPIO184_OEN 0x10253A, BIT0
#define GPIO184_OUT 0x10253B, BIT0
#define GPIO184_IN  0x10253E, BIT0

#define GPIO185_PAD PADA_HSYNC0
#define GPIO185_OEN 0x10255A, BIT0
#define GPIO185_OUT 0x10255A, BIT6
#define GPIO185_IN  0x10255C, BIT0

#define GPIO186_PAD PADA_VSYNC0
#define GPIO186_OEN 0x10255A, BIT3
#define GPIO186_OUT 0x10255B, BIT1
#define GPIO186_IN  0x10255C, BIT3

#define GPIO187_PAD PADA_VCOM
#define GPIO187_OEN 0x102742, BIT2
#define GPIO187_OUT 0x102743, BIT2
#define GPIO187_IN  0x102744, BIT2
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

#define PM_INT_COUNT   31

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
    42,     //PAD_GPIO7
    43,     //PAD_GPIO8
    48,     //PAD_GPIO13
    49,     //PAD_GPIO14
    34,     //PAD_GPIO0
    35,     //PAD_GPIO1
    36,     //PAD_GPIO2
    37      //PAD_GPIO3
};

const int gpio_IRQnum[INT_COUNT]= {
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE,
				E_IRQEXPH_EXT_GPIO_MERGE
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

    __GPIO(0), __GPIO(1), __GPIO(2), __GPIO(3), __GPIO(4),
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
    __GPIO(75), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(181), __GPIO(999), __GPIO(183), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(999),
    __GPIO(999), __GPIO(999), __GPIO(999), __GPIO(198), __GPIO(199),
    __GPIO(200), __GPIO(201)
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

/*
 *no E_IRQEXPL_PM_IRQ, use E_IRQHYPL_PM_IRQ_OUT
 */
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
            if(request_irq(E_IRQHYPL_PM_IRQ_OUT, (irq_handler_t)pm_gpio_irq, 0x0, "GPIO_PM", dev_id))
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
            free_irq(E_IRQHYPL_PM_IRQ_OUT, dev_id);
        }
    }
#endif

    return 0;
}
