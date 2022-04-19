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

#ifndef __DEV_ID_H__
#define __DEV_ID_H__

//-------------------------------------------------------------------------------------------------
// Device major/minior definition
//-------------------------------------------------------------------------------------------------
#define MDRV_NAME_CI                    "ci"
#define MDRV_MAJOR_CI                   0x80
#define MDRV_MINOR_CI                   0x00

#define MDRV_NAME_CREADER               "creader"
#define MDRV_MAJOR_CREADER              0x81
#define MDRV_MINOR_CREADER              0x00

#define MDRV_NAME_DAC                   "dac"
#define MDRV_MAJOR_DAC                  0x82
#define MDRV_MINOR_DAC                  0x00

#define MDRV_NAME_DEB                   "deb"
#define MDRV_MAJOR_DEB                  0x83
#define MDRV_MINOR_DEB                  0x00

#define MDRV_NAME_DVBC                  "dvbc"
#define MDRV_MAJOR_DVBC                 0x85
#define MDRV_MINOR_DVBC                 0x00

#define MDRV_NAME_EMAC                  "emac"
#define MDRV_MAJOR_EMAC                 0x86
#define MDRV_MINOR_EMAC                 0x00

#define MDRV_NAME_GE                    "ge"
#define MDRV_MAJOR_GE                   0x87
#define MDRV_MINOR_GE                   0x00

#define MDRV_NAME_HDMI                  "hdmi"
#define MDRV_MAJOR_HDMI                 0x89
#define MDRV_MINOR_HDMI                 0x00

#define MDRV_NAME_IIC                   "iic"
#define MDRV_MAJOR_IIC                  0x8a
#define MDRV_MINOR_IIC                  0x00

#define MDRV_NAME_M4VD                  "m4vd"
#define MDRV_MAJOR_M4VD                 0x8b
#define MDRV_MINOR_M4VD                 0x00

#define MDRV_NAME_MAD                   "mad"
#define MDRV_MAJOR_MAD                  0x8c
#define MDRV_MINOR_MAD                  0x00

#define MDRV_NAME_MIU                   "miu"
#define MDRV_MAJOR_MIU                  0x8d
#define MDRV_MINOR_MIU                  0x00

#define MDRV_NAME_MVD                   "mvd"
#define MDRV_MAJOR_MVD                  0x8e
#define MDRV_MINOR_MVD                  0x00

#define MDRV_NAME_POWER                 "power"
#define MDRV_MAJOR_POWER                0x8f
#define MDRV_MINOR_POWER                0x00


#define MDRV_NAME_SMART                 "smart"
#define MDRV_MAJOR_SMART                0x92
#define MDRV_MINOR_SMART                0x00

#define MDRV_NAME_SCALER                "scaler"
#define MDRV_MAJOR_SCALER               0x93
#define MDRV_MINOR_SCALER               0x00

#define MDRV_NAME_SYSTEM                "system"
#define MDRV_MAJOR_SYSTEM               0x94
#define MDRV_MINOR_SYSTEM               0x00

#define MDRV_NAME_TSP                   "tsp"
#define MDRV_MAJOR_TSP                  0x95
#define MDRV_MINOR_TSP                  0x00

#define MDRV_NAME_TVENCODER             "tvencoder"
#define MDRV_MAJOR_TVENCODER            0x96
#define MDRV_MINOR_TVENCODER            0x00

#define MDRV_NAME_UART                  "uart"
#define MDRV_MAJOR_UART                 0x97
#define MDRV_MINOR_UART                 0x00

#define MDRV_NAME_USB                   "usb"
#define MDRV_MAJOR_USB                  0x98
#define MDRV_MINOR_USB                  0x00

#define MDRV_NAME_EEPROM                "eeprom"
#define MDRV_MAJOR_EEPROM               0x99
#define MDRV_MINOR_EEPROM               0x00

#define MDRV_NAME_FLASH                 "flash"
#define MDRV_MAJOR_FLASH                0x9a
#define MDRV_MINOR_FLASH                0x00

#define MDRV_NAME_GPIO                  "gpio"
#define MDRV_MAJOR_GPIO                 0x9b
#define MDRV_MINOR_GPIO                 0x00

#define MDRV_NAME_IR                    "ir"
#define MDRV_MAJOR_IR                   0x9c
#define MDRV_MINOR_IR                   0x00

#define MDRV_NAME_SOFTWARE_IR           "software_ir"
#define MDRV_MAJOR_SOFTWARE_IR          0x9d
#define MDRV_MINOR_SOFTWARE_IR          0x00

#define MDRV_NAME_MPOOL                 "mpool"
#define MDRV_MAJOR_MPOOL                0x9e
#define MDRV_MINOR_MPOOL                0x00

#define MDRV_NAME_MVOP                  "mvop"
#define MDRV_MAJOR_MVOP                 0x9f
#define MDRV_MINOR_MVOP                 0x00

#define MDRV_NAME_VD                    "vd"
#define MDRV_MAJOR_VD                   0xa0
#define MDRV_MINOR_VD                   0x00

#define MDRV_NAME_MCU8051               "8051"
#define MDRV_MAJOR_MCU8051              0xa1
#define MDRV_MINOR_MCU8051              0x00

#define MDRV_NAME_MICOM                 "8051lite"
#define MDRV_MAJOR_MICOM                0xa2
#define MDRV_MINOR_MICOM                0x00

#define MDRV_NAME_AEON                  "aeon"
#define MDRV_MAJOR_AEON                 0xa3
#define MDRV_MINOR_AEON                 0x00

#define MDRV_NAME_PWM                   "pwm"
#define MDRV_MAJOR_PWM                  0xa4
#define MDRV_MINOR_PWM                  0x00

#define MDRV_NAME_RLD                   "rld"
#define MDRV_MAJOR_RLD                  0xa5
#define MDRV_MINOR_RLD                  0x00

#define MDRV_NAME_TTX                   "ttx"
#define MDRV_MAJOR_TTX                  0xaa
#define MDRV_MINOR_TTX                  0x00

#define MDRV_NAME_MLINK                 "mlink"
#define MDRV_MAJOR_MLINK                0xab
#define MDRV_MINOR_MLINK                0x00

#define MDRV_NAME_CC                   "cc"
#define MDRV_MAJOR_CC                  0xac
#define MDRV_MINOR_CC                  0x00

#define MDRV_NAME_JPEG                  "jpeg"
#define MDRV_MAJOR_JPEG                 0xad
#define MDRV_MINOR_JPEG                 0x00

#define MDRV_NAME_H264                   "h264"
#define MDRV_MAJOR_H264                   0xae
#define MDRV_MINOR_H264                   0x00

#define MDRV_NAME_MSMAILBOX             "msmailbox"
#define MDRV_MAJOR_MSMAILBOX            0xaf
#define MDRV_MINOR_MSMAILBOX            0x00

#define MDRV_NAME_GOP                   "gop"
#define MDRV_MAJOR_GOP                  0xe7
#define MDRV_MINOR_GOP                  0x00

#define MDRV_NAME_FUSION                "fusion"
#define MDRV_MAJOR_FUSION               0xed
#define MDRV_MINOR_FUSION               0x00

#define MDRV_NAME_MALI                  "mali"
#define MDRV_MAJOR_MALI                 0xee
#define MDRV_MINOR_MALI                 0x00

#define MDRV_NAME_UMP                   "ump"
#define MDRV_MAJOR_UMP                  0xef
#define MDRV_MINOR_UMP                  0x00

#define MDRV_NAME_AVS                   "avs"
#define MDRV_MAJOR_AVS                  0xf1
#define MDRV_MINOR_AVS                  0x00

#define MDRV_NAME_SEM                  "sem"
#define MDRV_MAJOR_SEM                  0xf2
#define MDRV_MINOR_SEM                  0x00

#define MDRV_NAME_RVD                  "rvd"
#define MDRV_MAJOR_RVD                  0xf3
#define MDRV_MINOR_RVD                  0x00

#define MDRV_NAME_MEDIACODEC           "mediacodec"
#define MDRV_MAJOR_MEDIACODEC          0xf4
#define MDRV_MINOR_MEDIACODEC          0x00

#define MDRV_NAME_DMA           "dma"
#define MDRV_MAJOR_DMA          0xf5
#define MDRV_MINOR_DMA         0x00

#define MDRV_NAME_FIMWARE               "fw"
#define MDRV_MAJOR_FIRMWARE             0xf6
#define MDRV_MINOR_FIRMWARE             0x00

#define MDRV_NAME_MIOMAP                 "miomap"
#define MDRV_MAJOR_MIOMAP                0xb0
#define MDRV_MINOR_MIOMAP                0x00

#define MDRV_NAME_GFLIP                 "gflip"
#define MDRV_MAJOR_GFLIP                0xea
#define MDRV_MINOR_GFLIP                0x00

#define MDRV_NAME_RFID                 "rfid"
#define MDRV_MAJOR_RFID                0xb1
#define MDRV_MINOR_RFID                0x00

#define MDRV_NAME_SWREG                "swreg"
#define MDRV_MAJOR_SWREG               0xb2
#define MDRV_MINOR_SWREG               0x00

#define MDRV_NAME_VPOOL                "vpool"
#define MDRV_MAJOR_VPOOL               0xb3
#define MDRV_MINOR_VPOOL               0x00

#define MDRV_NAME_MPIF                 "mpif"
#define MDRV_MAJOR_MPIF               	0xb4
#define MDRV_MINOR_MPIF                 0x00

#define MDRV_NAME_SEMUTEX              "semutex"
#define MDRV_MAJOR_SEMUTEX              0xb5
#define MDRV_MINOR_SEMUTEX              0x00

#define MDRV_NAME_CMAPOOL              "cmapool"
#define MDRV_MAJOR_CMAPOOL              0xb6
#define MDRV_MINOR_CMAPOOL              0x00

#define MDRV_NAME_IPAPOOL              "ipapool"
#define MDRV_MAJOR_IPAPOOL              0xb7
#define MDRV_MINOR_IPAPOOL              0x00

#define MDRV_NAME_TUNER                "tuner"
#define MDRV_MAJOR_TUNER                0xb8
#define MDRV_MINOR_TUNER                0x00

#define MDRV_NAME_PM              		"pm"
#define MDRV_MAJOR_PM              		0xb9
#define MDRV_MINOR_PM              		0x00

#define MDRV_NAME_LDM              "localdimming"
#define MDRV_MAJOR_LDM              0xba
#define MDRV_MINOR_LDM              0x00

#define MDRV_NAME_CLKM              "clkm"
#define MDRV_MAJOR_CLKM              0xbb
#define MDRV_MINOR_CLKM              0xfa

#define MDRV_NAME_CPU                 "cpu"
#define MDRV_MAJOR_CPU                0xbe
#define MDRV_MINOR_CPU                0x00

#define MDRV_NAME_HDMITX4VX1                "hdmitx4vx1"
#define MDRV_MAJOR_HDMITX4VX1               0xc0
#define MDRV_MINOR_HDMITX4VX1               0x00

#define MDRV_NAME_TEESHM              "teeshm"
#define MDRV_MAJOR_TEESHM              0xc1
#define MDRV_MINOR_TEESHM              0x00

#define MDRV_NAME_TUNER_DVBS            "tunerS"
#define MDRV_MAJOR_TUNER_DVBS           0xc2
#define MDRV_MINOR_TUNER_DVBS           0x00

#define MDRV_NAME_UTIL                  "mstar_util"
#define MDRV_MAJOR_UTIL                 0xc3
#define MDRV_MINOR_UTIL                 0x00

#define MDRV_NAME_DISH                  "dish"
#define MDRV_MAJOR_DISH                 0xc4
#define MDRV_MINOR_DISH                 0x00

#endif // __DEV_ID_H__
