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

#ifndef __PRANA_DEVICE_H__
#define __PRANA_DEVICE_H__

#define DEVICE_NULL ((device_t)0)

typedef unsigned long device_t;

//
// device PM event
//
typedef enum
{
    E_EVT_SUSPEND   = 1,
    E_EVT_SHUTDOWN  = 2,
} eDEVEVT;

//
// poll types
//
#define POLLIN      0x01
#define POLLOUT     0x02
#define POLLHUP     0x04

//
// device I/O operations
//
struct devio
{
    int     (*open) (device_t dev, int mode);
    int     (*close)(device_t dev);
//    int     (*read) (device_t dev, char *buf, size_t len, int off);
    int     (*read) (device_t dev, char *buf, MS_U32 len, int off);
//    int     (*write)(device_t dev, const char *buf, size_t len, int off);
    int     (*write)(device_t dev, const char *buf, MS_U32 len, int off);
    int     (*poll) (device_t dev, int types);
    int     (*ioctl)(device_t dev, int request, unsigned long arg);
    int     (*event)(eDEVEVT evt);
};


#define DEVICE_MAGIC    (('D' << 24) | ('E' << 16) | ('V' << 8) | 'I')

//
// device structure
//
struct device
{
    int             magic;              // magic number
    int             refcnt;             // reference count
    int             flags;              // device characteristics
    struct devio    *devio;             // device i/o table
};

#define device_valid(dev) ((dev)->magic == DEVICE_MAGIC)
#endif /* __PRANA_DEVICE_H__ */

//MSTAR Module ID
/*******************************************************************************
!!!! WARNING NOT ALLOW MODIFY EXISTED VALUE !!!!
!!!! ONLY ALLOW ADD NEW VALUE AFTER END OF THIS LIST !!!!
*******************************************************************************/
#define MS_MODULE_HW          0x0000

#define MS_MODULE_CHIPTOP     0x0011
#define MS_MODULE_MIU         0x0012
#define MS_MODULE_WDT         0x003C

#define MS_MODULE_ACE         0x0101
#define MS_MODULE_AUDIO       0x0102
#define MS_MODULE_AVD         0x0103
#define MS_MODULE_BDMA        0x0104
#define MS_MODULE_DLC         0x0105
#define MS_MODULE_FLASH       0x0106
#define MS_MODULE_GE          0x0107
#define MS_MODULE_GOP         0x0108
#define MS_MODULE_HVD         0x0109
#define MS_MODULE_HWI2C       0x010a
#define MS_MODULE_IR          0x010b
#define MS_MODULE_IRQ         0x010c
#define MS_MODULE_JPD         0x010d
#define MS_MODULE_MBX         0x010e
#define MS_MODULE_MVD         0x010f
#define MS_MODULE_MVOP        0x0120
#define MS_MODULE_PQ          0x0121
#define MS_MODULE_RVD         0x0122
#define MS_MODULE_SC          0x0123
#define MS_MODULE_TSP         0x0124
#define MS_MODULE_UART        0x0125
#define MS_MODULE_VPU         0x0126
#define MS_MODULE_XC          0x0127
#define MS_MODULE_PNL         0x0128
#define MS_MODULE_ISP         0x0129
#define MS_MODULE_PFSH        0x012a
#define MS_MODULE_GPIO        0x012b
#define MS_MODULE_PM          0x012c
#define MS_MODULE_VBI         0x012d
#define MS_MODULE_PCMCIA      0x012e
#define MS_MODULE_PIU         0x012f
#define MS_MODULE_MHEG5       0x0130
#define MS_MODULE_SEM         0x02f0
#define MS_MODULE_PWM         0x1032
#define MS_MODULE_DMD         0x1033
#define MS_MODULE_PWS         0x1034
