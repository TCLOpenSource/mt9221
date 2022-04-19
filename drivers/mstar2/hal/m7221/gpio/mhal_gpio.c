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

#define GPIO0_PAD PAD_IRIN
#define GPIO0_OEN 0x0f26, BIT0
#define GPIO0_OUT 0x0f26, BIT1
#define GPIO0_IN  0x0f26, BIT2

#define GPIO1_PAD PAD_CEC0
#define GPIO1_OEN 0x0f2a, BIT0
#define GPIO1_OUT 0x0f2a, BIT1
#define GPIO1_IN  0x0f2a, BIT2

#define GPIO2_PAD PAD_PWM_PM
#define GPIO2_OEN 0x0f28, BIT0
#define GPIO2_OUT 0x0f28, BIT1
#define GPIO2_IN  0x0f28, BIT2

#define GPIO3_PAD PAD_DDCA_CK
#define GPIO3_OEN 0x0494, BIT1
#define GPIO3_OUT 0x0494, BIT2
#define GPIO3_IN  0x0494, BIT0

#define GPIO4_PAD PAD_DDCA_DA
#define GPIO4_OEN 0x0494, BIT5
#define GPIO4_OUT 0x0494, BIT6
#define GPIO4_IN  0x0494, BIT4

#define GPIO5_PAD PAD_GPIO0_PM
#define GPIO5_OEN 0x0f00, BIT0
#define GPIO5_OUT 0x0f00, BIT1
#define GPIO5_IN  0x0f00, BIT2

#define GPIO6_PAD PAD_GPIO1_PM
#define GPIO6_OEN 0x0f02, BIT0
#define GPIO6_OUT 0x0f02, BIT1
#define GPIO6_IN  0x0f02, BIT2

#define GPIO7_PAD PAD_GPIO2_PM
#define GPIO7_OEN 0x0f04, BIT0
#define GPIO7_OUT 0x0f04, BIT1
#define GPIO7_IN  0x0f04, BIT2

#define GPIO8_PAD PAD_USB_CTRL
#define GPIO8_OEN 0x0f06, BIT0
#define GPIO8_OUT 0x0f06, BIT1
#define GPIO8_IN  0x0f06, BIT2

#define GPIO9_PAD PAD_GPIO5_PM
#define GPIO9_OEN 0x0f0a, BIT0
#define GPIO9_OUT 0x0f0a, BIT1
#define GPIO9_IN  0x0f0a, BIT2

#define GPIO10_PAD PAD_GPIO6_PM
#define GPIO10_OEN 0x0f0c, BIT0
#define GPIO10_OUT 0x0f0c, BIT1
#define GPIO10_IN  0x0f0c, BIT2

#define GPIO11_PAD PAD_GPIO7_PM
#define GPIO11_OEN 0x0f0e, BIT0
#define GPIO11_OUT 0x0f0e, BIT1
#define GPIO11_IN  0x0f0e, BIT2

#define GPIO12_PAD PAD_GPIO8_PM
#define GPIO12_OEN 0x0f10, BIT0
#define GPIO12_OUT 0x0f10, BIT1
#define GPIO12_IN  0x0f10, BIT2

#define GPIO13_PAD PAD_GPIO9_PM
#define GPIO13_OEN 0x0f12, BIT0
#define GPIO13_OUT 0x0f12, BIT1
#define GPIO13_IN  0x0f12, BIT2

#define GPIO14_PAD PAD_GPIO10_PM
#define GPIO14_OEN 0x0f14, BIT0
#define GPIO14_OUT 0x0f14, BIT1
#define GPIO14_IN  0x0f14, BIT2

#define GPIO15_PAD PAD_GPIO11_PM
#define GPIO15_OEN 0x0f16, BIT0
#define GPIO15_OUT 0x0f16, BIT1
#define GPIO15_IN  0x0f16, BIT2

#define GPIO16_PAD PAD_GPIO12_PM
#define GPIO16_OEN 0x0f18, BIT0
#define GPIO16_OUT 0x0f18, BIT1
#define GPIO16_IN  0x0f18, BIT2

#define GPIO17_PAD PAD_HOTPLUGA
#define GPIO17_OEN 0x0e4e, BIT0
#define GPIO17_OUT 0x0e4e, BIT4
#define GPIO17_IN  0x0e4f, BIT0

#define GPIO18_PAD PAD_HOTPLUGB
#define GPIO18_OEN 0x0e4e, BIT1
#define GPIO18_OUT 0x0e4e, BIT5
#define GPIO18_IN  0x0e4f, BIT1

#define GPIO19_PAD PAD_HOTPLUGC
#define GPIO19_OEN 0x0e4e, BIT2
#define GPIO19_OUT 0x0e4e, BIT6
#define GPIO19_IN  0x0e4f, BIT2

#define GPIO20_PAD PAD_HOTPLUGA_HDMI20_5V
#define GPIO20_OEN 0x010218, BIT5
#define GPIO20_OUT 0x010218, BIT4
#define GPIO20_IN  0x010218, BIT6

#define GPIO21_PAD PAD_HOTPLUGB_HDMI20_5V
#define GPIO21_OEN 0x010318, BIT5
#define GPIO21_OUT 0x010318, BIT4
#define GPIO21_IN  0x010318, BIT6

#define GPIO22_PAD PAD_HOTPLUGC_HDMI20_5V
#define GPIO22_OEN 0x010418, BIT5
#define GPIO22_OUT 0x010418, BIT4
#define GPIO22_IN  0x010418, BIT6

#define GPIO23_PAD PAD_DDCDA_CK
#define GPIO23_OEN 0x0496, BIT1
#define GPIO23_OUT 0x0496, BIT2
#define GPIO23_IN  0x0496, BIT0

#define GPIO24_PAD PAD_DDCDA_DA
#define GPIO24_OEN 0x0496, BIT5
#define GPIO24_OUT 0x0496, BIT6
#define GPIO24_IN  0x0496, BIT4

#define GPIO25_PAD PAD_DDCDB_CK
#define GPIO25_OEN 0x0497, BIT1
#define GPIO25_OUT 0x0497, BIT2
#define GPIO25_IN  0x0497, BIT0

#define GPIO26_PAD PAD_DDCDB_DA
#define GPIO26_OEN 0x0497, BIT5
#define GPIO26_OUT 0x0497, BIT6
#define GPIO26_IN  0x0497, BIT4

#define GPIO27_PAD PAD_DDCDC_CK
#define GPIO27_OEN 0x0498, BIT1
#define GPIO27_OUT 0x0498, BIT2
#define GPIO27_IN  0x0498, BIT0

#define GPIO28_PAD PAD_DDCDC_DA
#define GPIO28_OEN 0x0498, BIT5
#define GPIO28_OUT 0x0498, BIT6
#define GPIO28_IN  0x0498, BIT4

#define GPIO29_PAD PAD_SAR0
#define GPIO29_OEN 0x1423, BIT0
#define GPIO29_OUT 0x1424, BIT0
#define GPIO29_IN  0x1425, BIT0

#define GPIO30_PAD PAD_SAR1
#define GPIO30_OEN 0x1423, BIT1
#define GPIO30_OUT 0x1424, BIT1
#define GPIO30_IN  0x1425, BIT1

#define GPIO31_PAD PAD_SAR2
#define GPIO31_OEN 0x1423, BIT2
#define GPIO31_OUT 0x1424, BIT2
#define GPIO31_IN  0x1425, BIT2

#define GPIO32_PAD PAD_SAR3
#define GPIO32_OEN 0x1423, BIT3
#define GPIO32_OUT 0x1424, BIT3
#define GPIO32_IN  0x1425, BIT3

#define GPIO33_PAD PAD_SAR4
#define GPIO33_OEN 0x1423, BIT4
#define GPIO33_OUT 0x1424, BIT4
#define GPIO33_IN  0x1425, BIT4

#define GPIO34_PAD PAD_VPLUGIN
#define GPIO34_OEN 0x1423, BIT5
#define GPIO34_OUT 0x1424, BIT5
#define GPIO34_IN  0x1425, BIT5

#define GPIO35_PAD PAD_VID0
#define GPIO35_OEN 0x2e84, BIT1
#define GPIO35_OUT 0x2e84, BIT0
#define GPIO35_IN  0x2e84, BIT2

#define GPIO36_PAD PAD_VID1
#define GPIO36_OEN 0x2e85, BIT1
#define GPIO36_OUT 0x2e85, BIT0
#define GPIO36_IN  0x2e85, BIT2

#define GPIO37_PAD PAD_VID2
#define GPIO37_OEN 0x0f22, BIT0
#define GPIO37_OUT 0x0f22, BIT1
#define GPIO37_IN  0x0f22, BIT2

#define GPIO38_PAD PAD_VID3
#define GPIO38_OEN 0x0f24, BIT0
#define GPIO38_OUT 0x0f24, BIT1
#define GPIO38_IN  0x0f24, BIT2

#define GPIO39_PAD PAD_WOL_INT_OUT
#define GPIO39_OEN 0x2e82, BIT1
#define GPIO39_OUT 0x2e82, BIT0
#define GPIO39_IN  0x2e82, BIT2

#define GPIO40_PAD PAD_I2S_IN_BCK
#define GPIO40_OEN 0x0f3e, BIT0
#define GPIO40_OUT 0x0f3e, BIT1
#define GPIO40_IN  0x0f3e, BIT2

#define GPIO41_PAD PAD_I2S_IN_WS
#define GPIO41_OEN 0x0f40, BIT0
#define GPIO41_OUT 0x0f40, BIT1
#define GPIO41_IN  0x0f40, BIT2

#define GPIO42_PAD PAD_I2S_IN_MCK
#define GPIO42_OEN 0x0f42, BIT0
#define GPIO42_OUT 0x0f42, BIT1
#define GPIO42_IN  0x0f42, BIT2

#define GPIO43_PAD PAD_I2S_IN_SD0
#define GPIO43_OEN 0x0f44, BIT0
#define GPIO43_OUT 0x0f44, BIT1
#define GPIO43_IN  0x0f44, BIT2

#define GPIO44_PAD PAD_I2S_IN_SD1
#define GPIO44_OEN 0x0f46, BIT0
#define GPIO44_OUT 0x0f46, BIT1
#define GPIO44_IN  0x0f46, BIT2

#define GPIO45_PAD PAD_CILINK_DEMOD0_CLK
#define GPIO45_OEN 0x102b13, BIT1
#define GPIO45_OUT 0x102b13, BIT0
#define GPIO45_IN  0x102b13, BIT2

#define GPIO46_PAD PAD_CILINK_DEMOD1_CLK
#define GPIO46_OEN 0x102b1a, BIT1
#define GPIO46_OUT 0x102b1a, BIT0
#define GPIO46_IN  0x102b1a, BIT2

#define GPIO47_PAD PAD_CILINK_INT
#define GPIO47_OEN 0x102b12, BIT1
#define GPIO47_OUT 0x102b12, BIT0
#define GPIO47_IN  0x102b12, BIT2

#define GPIO48_PAD PAD_CILINK_RX0_D0
#define GPIO48_OEN 0x102b18, BIT1
#define GPIO48_OUT 0x102b18, BIT0
#define GPIO48_IN  0x102b18, BIT2

#define GPIO49_PAD PAD_CILINK_RX0_D1
#define GPIO49_OEN 0x102b19, BIT1
#define GPIO49_OUT 0x102b19, BIT0
#define GPIO49_IN  0x102b19, BIT2

#define GPIO50_PAD PAD_CILINK_RX1_D0
#define GPIO50_OEN 0x102ba1, BIT1
#define GPIO50_OUT 0x102ba1, BIT0
#define GPIO50_IN  0x102ba1, BIT2

#define GPIO51_PAD PAD_CILINK_RX1_D1
#define GPIO51_OEN 0x102ba2, BIT1
#define GPIO51_OUT 0x102ba2, BIT0
#define GPIO51_IN  0x102ba2, BIT2

#define GPIO52_PAD PAD_CILINK_RX_CLK
#define GPIO52_OEN 0x102b17, BIT1
#define GPIO52_OUT 0x102b17, BIT0
#define GPIO52_IN  0x102b17, BIT2

#define GPIO53_PAD PAD_CILINK_SPI_CLK
#define GPIO53_OEN 0x102b0e, BIT1
#define GPIO53_OUT 0x102b0e, BIT0
#define GPIO53_IN  0x102b0e, BIT2

#define GPIO54_PAD PAD_CILINK_SPI_CS
#define GPIO54_OEN 0x102b11, BIT1
#define GPIO54_OUT 0x102b11, BIT0
#define GPIO54_IN  0x102b11, BIT2

#define GPIO55_PAD PAD_CILINK_SPI_MISO
#define GPIO55_OEN 0x102b10, BIT1
#define GPIO55_OUT 0x102b10, BIT0
#define GPIO55_IN  0x102b10, BIT2

#define GPIO56_PAD PAD_CILINK_SPI_MOSI
#define GPIO56_OEN 0x102b0f, BIT1
#define GPIO56_OUT 0x102b0f, BIT0
#define GPIO56_IN  0x102b0f, BIT2

#define GPIO57_PAD PAD_CILINK_TX0_D0
#define GPIO57_OEN 0x102b15, BIT1
#define GPIO57_OUT 0x102b15, BIT0
#define GPIO57_IN  0x102b15, BIT2

#define GPIO58_PAD PAD_CILINK_TX0_D1
#define GPIO58_OEN 0x102b16, BIT1
#define GPIO58_OUT 0x102b16, BIT0
#define GPIO58_IN  0x102b16, BIT2

#define GPIO59_PAD PAD_CILINK_TX1_D0
#define GPIO59_OEN 0x102b1b, BIT1
#define GPIO59_OUT 0x102b1b, BIT0
#define GPIO59_IN  0x102b1b, BIT2

#define GPIO60_PAD PAD_CILINK_TX1_D1
#define GPIO60_OEN 0x102ba0, BIT1
#define GPIO60_OUT 0x102ba0, BIT0
#define GPIO60_IN  0x102ba0, BIT2

#define GPIO61_PAD PAD_CILINK_TX_CLK
#define GPIO61_OEN 0x102b14, BIT1
#define GPIO61_OUT 0x102b14, BIT0
#define GPIO61_IN  0x102b14, BIT2

#define GPIO62_PAD PAD_DDCR_CK
#define GPIO62_OEN 0x102b87, BIT1
#define GPIO62_OUT 0x102b87, BIT0
#define GPIO62_IN  0x102b87, BIT2

#define GPIO63_PAD PAD_DDCR_DA
#define GPIO63_OEN 0x102b86, BIT1
#define GPIO63_OUT 0x102b86, BIT0
#define GPIO63_IN  0x102b86, BIT2

#define GPIO64_PAD PAD_GPIO2
#define GPIO64_OEN 0x102b00, BIT1
#define GPIO64_OUT 0x102b00, BIT0
#define GPIO64_IN  0x102b00, BIT2

#define GPIO65_PAD PAD_GPIO3
#define GPIO65_OEN 0x102b01, BIT1
#define GPIO65_OUT 0x102b01, BIT0
#define GPIO65_IN  0x102b01, BIT2

#define GPIO66_PAD PAD_GPIO4
#define GPIO66_OEN 0x102b02, BIT1
#define GPIO66_OUT 0x102b02, BIT0
#define GPIO66_IN  0x102b02, BIT2

#define GPIO67_PAD PAD_GPIO5
#define GPIO67_OEN 0x102b03, BIT1
#define GPIO67_OUT 0x102b03, BIT0
#define GPIO67_IN  0x102b03, BIT2

#define GPIO68_PAD PAD_GPIO9
#define GPIO68_OEN 0x102b04, BIT1
#define GPIO68_OUT 0x102b04, BIT0
#define GPIO68_IN  0x102b04, BIT2

#define GPIO69_PAD PAD_GPIO10
#define GPIO69_OEN 0x102b05, BIT1
#define GPIO69_OUT 0x102b05, BIT0
#define GPIO69_IN  0x102b05, BIT2

#define GPIO70_PAD PAD_GPIO11
#define GPIO70_OEN 0x102b06, BIT1
#define GPIO70_OUT 0x102b06, BIT0
#define GPIO70_IN  0x102b06, BIT2

#define GPIO71_PAD PAD_GPIO12
#define GPIO71_OEN 0x102b07, BIT1
#define GPIO71_OUT 0x102b07, BIT0
#define GPIO71_IN  0x102b07, BIT2

#define GPIO72_PAD PAD_GPIO19
#define GPIO72_OEN 0x102b08, BIT1
#define GPIO72_OUT 0x102b08, BIT0
#define GPIO72_IN  0x102b08, BIT2

#define GPIO73_PAD PAD_GPIO20
#define GPIO73_OEN 0x102b09, BIT1
#define GPIO73_OUT 0x102b09, BIT0
#define GPIO73_IN  0x102b09, BIT2

#define GPIO74_PAD PAD_GPIO25
#define GPIO74_OEN 0x102b0a, BIT1
#define GPIO74_OUT 0x102b0a, BIT0
#define GPIO74_IN  0x102b0a, BIT2

#define GPIO75_PAD PAD_GPIO26
#define GPIO75_OEN 0x102b0b, BIT1
#define GPIO75_OUT 0x102b0b, BIT0
#define GPIO75_IN  0x102b0b, BIT2

#define GPIO76_PAD PAD_GPIO30
#define GPIO76_OEN 0x102b0c, BIT1
#define GPIO76_OUT 0x102b0c, BIT0
#define GPIO76_IN  0x102b0c, BIT2

#define GPIO77_PAD PAD_GPIO31
#define GPIO77_OEN 0x102b0d, BIT1
#define GPIO77_OUT 0x102b0d, BIT0
#define GPIO77_IN  0x102b0d, BIT2

#define GPIO78_PAD PAD_HDMIRX_ARCTX
#define GPIO78_OEN 0x110320, BIT1
#define GPIO78_OUT 0x110320, BIT0
#define GPIO78_IN  0x110320, BIT2

#define GPIO79_PAD PAD_I2S_OUT_BCK
#define GPIO79_OEN 0x102b38, BIT1
#define GPIO79_OUT 0x102b38, BIT0
#define GPIO79_IN  0x102b38, BIT2

#define GPIO80_PAD PAD_I2S_OUT_MCK
#define GPIO80_OEN 0x102b39, BIT1
#define GPIO80_OUT 0x102b39, BIT0
#define GPIO80_IN  0x102b39, BIT2

#define GPIO81_PAD PAD_I2S_OUT_SD
#define GPIO81_OEN 0x102b3b, BIT1
#define GPIO81_OUT 0x102b3b, BIT0
#define GPIO81_IN  0x102b3b, BIT2

#define GPIO82_PAD PAD_I2S_OUT_SD1
#define GPIO82_OEN 0x102b3c, BIT1
#define GPIO82_OUT 0x102b3c, BIT0
#define GPIO82_IN  0x102b3c, BIT2

#define GPIO83_PAD PAD_I2S_OUT_SD2
#define GPIO83_OEN 0x102b3d, BIT1
#define GPIO83_OUT 0x102b3d, BIT0
#define GPIO83_IN  0x102b3d, BIT2

#define GPIO84_PAD PAD_I2S_OUT_WS
#define GPIO84_OEN 0x102b3a, BIT1
#define GPIO84_OUT 0x102b3a, BIT0
#define GPIO84_IN  0x102b3a, BIT2

#define GPIO85_PAD PAD_PWM0
#define GPIO85_OEN 0x102b88, BIT1
#define GPIO85_OUT 0x102b88, BIT0
#define GPIO85_IN  0x102b88, BIT2

#define GPIO86_PAD PAD_PWM1
#define GPIO86_OEN 0x102b89, BIT1
#define GPIO86_OUT 0x102b89, BIT0
#define GPIO86_IN  0x102b89, BIT2

#define GPIO87_PAD PAD_PWM2
#define GPIO87_OEN 0x102b8a, BIT1
#define GPIO87_OUT 0x102b8a, BIT0
#define GPIO87_IN  0x102b8a, BIT2

#define GPIO88_PAD PAD_PWM3
#define GPIO88_OEN 0x102b8b, BIT1
#define GPIO88_OUT 0x102b8b, BIT0
#define GPIO88_IN  0x102b8b, BIT2

#define GPIO89_PAD PAD_SD_CLK
#define GPIO89_OEN 0x102bfa, BIT1
#define GPIO89_OUT 0x102bfa, BIT0
#define GPIO89_IN  0x102bfa, BIT2

#define GPIO90_PAD PAD_SD_CMD
#define GPIO90_OEN 0x102bfb, BIT1
#define GPIO90_OUT 0x102bfb, BIT0
#define GPIO90_IN  0x102bfb, BIT2

#define GPIO91_PAD PAD_SD_D0
#define GPIO91_OEN 0x102bac, BIT1
#define GPIO91_OUT 0x102bac, BIT0
#define GPIO91_IN  0x102bac, BIT2

#define GPIO92_PAD PAD_SD_D1
#define GPIO92_OEN 0x102bad, BIT1
#define GPIO92_OUT 0x102bad, BIT0
#define GPIO92_IN  0x102bad, BIT2

#define GPIO93_PAD PAD_SD_D2
#define GPIO93_OEN 0x102bae, BIT1
#define GPIO93_OUT 0x102bae, BIT0
#define GPIO93_IN  0x102bae, BIT2

#define GPIO94_PAD PAD_SD_D3
#define GPIO94_OEN 0x102bf9, BIT1
#define GPIO94_OUT 0x102bf9, BIT0
#define GPIO94_IN  0x102bf9, BIT2

#define GPIO95_PAD PAD_SPDIF_IN
#define GPIO95_OEN 0x102b36, BIT1
#define GPIO95_OUT 0x102b36, BIT0
#define GPIO95_IN  0x102b36, BIT2

#define GPIO96_PAD PAD_SPDIF_OUT
#define GPIO96_OEN 0x102b37, BIT1
#define GPIO96_OUT 0x102b37, BIT0
#define GPIO96_IN  0x102b37, BIT2

#define GPIO97_PAD PAD_TGPIO0
#define GPIO97_OEN 0x102b8d, BIT1
#define GPIO97_OUT 0x102b8d, BIT0
#define GPIO97_IN  0x102b8d, BIT2

#define GPIO98_PAD PAD_TGPIO1
#define GPIO98_OEN 0x102b8e, BIT1
#define GPIO98_OUT 0x102b8e, BIT0
#define GPIO98_IN  0x102b8e, BIT2

#define GPIO99_PAD PAD_TGPIO2
#define GPIO99_OEN 0x102b8f, BIT1
#define GPIO99_OUT 0x102b8f, BIT0
#define GPIO99_IN  0x102b8f, BIT2

#define GPIO100_PAD PAD_TGPIO3
#define GPIO100_OEN 0x102b90, BIT1
#define GPIO100_OUT 0x102b90, BIT0
#define GPIO100_IN  0x102b90, BIT2

#define GPIO101_PAD PAD_TS1_CLK
#define GPIO101_OEN 0x102b27, BIT1
#define GPIO101_OUT 0x102b27, BIT0
#define GPIO101_IN  0x102b27, BIT2

#define GPIO102_PAD PAD_TS1_D0
#define GPIO102_OEN 0x102b31, BIT1
#define GPIO102_OUT 0x102b31, BIT0
#define GPIO102_IN  0x102b31, BIT2

#define GPIO103_PAD PAD_TS1_D1
#define GPIO103_OEN 0x102b30, BIT1
#define GPIO103_OUT 0x102b30, BIT0
#define GPIO103_IN  0x102b30, BIT2

#define GPIO104_PAD PAD_TS1_D2
#define GPIO104_OEN 0x102b2f, BIT1
#define GPIO104_OUT 0x102b2f, BIT0
#define GPIO104_IN  0x102b2f, BIT2

#define GPIO105_PAD PAD_TS1_D3
#define GPIO105_OEN 0x102b2e, BIT1
#define GPIO105_OUT 0x102b2e, BIT0
#define GPIO105_IN  0x102b2e, BIT2

#define GPIO106_PAD PAD_TS1_D4
#define GPIO106_OEN 0x102b2d, BIT1
#define GPIO106_OUT 0x102b2d, BIT0
#define GPIO106_IN  0x102b2d, BIT2

#define GPIO107_PAD PAD_TS1_D5
#define GPIO107_OEN 0x102b2c, BIT1
#define GPIO107_OUT 0x102b2c, BIT0
#define GPIO107_IN  0x102b2c, BIT2

#define GPIO108_PAD PAD_TS1_D6
#define GPIO108_OEN 0x102b2b, BIT1
#define GPIO108_OUT 0x102b2b, BIT0
#define GPIO108_IN  0x102b2b, BIT2

#define GPIO109_PAD PAD_TS1_D7
#define GPIO109_OEN 0x102b2a, BIT1
#define GPIO109_OUT 0x102b2a, BIT0
#define GPIO109_IN  0x102b2a, BIT2

#define GPIO110_PAD PAD_TS1_SYNC
#define GPIO110_OEN 0x102b28, BIT1
#define GPIO110_OUT 0x102b28, BIT0
#define GPIO110_IN  0x102b28, BIT2

#define GPIO111_PAD PAD_TS1_VLD
#define GPIO111_OEN 0x102b29, BIT1
#define GPIO111_OUT 0x102b29, BIT0
#define GPIO111_IN  0x102b29, BIT2

#define GPIO112_PAD PAD_TS2_CLK
#define GPIO112_OEN 0x102b35, BIT1
#define GPIO112_OUT 0x102b35, BIT0
#define GPIO112_IN  0x102b35, BIT2

#define GPIO113_PAD PAD_TS2_D0
#define GPIO113_OEN 0x102b32, BIT1
#define GPIO113_OUT 0x102b32, BIT0
#define GPIO113_IN  0x102b32, BIT2

#define GPIO114_PAD PAD_TS2_SYNC
#define GPIO114_OEN 0x102b34, BIT1
#define GPIO114_OUT 0x102b34, BIT0
#define GPIO114_IN  0x102b34, BIT2

#define GPIO115_PAD PAD_TS2_VLD
#define GPIO115_OEN 0x102b33, BIT1
#define GPIO115_OUT 0x102b33, BIT0
#define GPIO115_IN  0x102b33, BIT2

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


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
#define INT_COUNT   8

const int gpio_IntPad[INT_COUNT]=
{

    89,     //PAD_SD_CLK
    64,     //PAD_GPIO2
    90,     //PAD_SD_CMD
    91,     //PAD_SD_D0
    75,     //PAD_GPIO26
    73,     //PAD_GPIO20
    95,     //PAD_SPDIF_IN
    82,     //PAD_I2S_OUT_SD1
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
    __GPIO(75), __GPIO(76), __GPIO(77), __GPIO(78), __GPIO(79),
    __GPIO(80), __GPIO(81), __GPIO(82), __GPIO(83), __GPIO(84),
    __GPIO(85), __GPIO(86), __GPIO(87), __GPIO(88), __GPIO(89),
    __GPIO(90), __GPIO(91), __GPIO(92), __GPIO(93), __GPIO(94),
    __GPIO(95), __GPIO(96), __GPIO(97), __GPIO(98), __GPIO(99),
    __GPIO(100), __GPIO(101), __GPIO(102), __GPIO(103), __GPIO(104),
    __GPIO(105), __GPIO(106), __GPIO(107), __GPIO(108), __GPIO(109),
    __GPIO(110), __GPIO(111), __GPIO(112), __GPIO(113), __GPIO(114),
    __GPIO(115)
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
	if (u32IndexGPIO > __Sizeof_GPIO_Pins())
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
	if (u32IndexGPIO >__Sizeof_GPIO_Pins())
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
    printk("MHal_GPIO_Enable_Interrupt : %d\n\n", gpio_num);

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

    return 0;
}

