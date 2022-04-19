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

#ifndef _STANDBY_MODE_TABLE_H_
#define _STANDBY_MODE_TABLE_H_

#include "MsTypes.h"

#define CONFIG_MSTAR_STANDBY_INV_TABLE_SUPPORT

#define STANDBY_MODE_TEST_OK_KERNEL

#define STANDBY_MODE_INV_TABLE_SIZE             84
#define STANDBY_MODE_INV_BUF_SIZE               85


typedef enum {
    REG_MASK,
    REG_SET,
    // for reset
    REG_TOGGLE,         // 1 -> 0 or 0 -> 1
    REG_TOGGLE_RESTORE, // 1 -> 0 -> 1 or 0 -> 1 -> 0
} E_REG_HANDLE;

typedef enum {
    REG_1Byte,
    REG_2Bytes,
} E_REG_SIZE;

typedef struct {
    MS_U32 u32RegAddr;
    MS_U16 u16RegData;
    MS_U16 u16RegDataExt;
    E_REG_HANDLE handle;
    E_REG_SIZE size;
    MS_U32 u32DelayUs;
} STANDBY_MODE_TABLE;

static const STANDBY_MODE_TABLE standby_mode_inv_restore_table[] =
{
    {0x103340, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b41, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b46, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b47, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b44, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b45, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b6b, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x103330, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x10331e, 0x01, 0x00, REG_SET, REG_1Byte},
    {0x101e39, 0x00, 0x00, REG_SET, REG_1Byte},
    {0x100a32, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b6c, 0x11, 0x00, REG_MASK, REG_1Byte},
    {0x100b62, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b68, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b63, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b69, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b67, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b72, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b60, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x112ce9, 0x80, 0x80, REG_MASK, REG_1Byte},
    {0x112ce6, 0x03, 0x03, REG_MASK, REG_1Byte},
    {0x112ce0, 0x30, 0x30, REG_MASK, REG_1Byte},
    {0x112ce1, 0x10, 0x10, REG_MASK, REG_1Byte},
    {0x112cea, 0x08, 0x08, REG_MASK, REG_1Byte},
    {0x112cdc, 0x30, 0x30, REG_MASK, REG_1Byte},
    {0x112cdd, 0x33, 0x33, REG_MASK, REG_1Byte},
    {0x112cda, 0x03, 0x03, REG_MASK, REG_1Byte},
    {0x112ce3, 0x03, 0x03, REG_MASK, REG_1Byte},
    {0x112cec, 0xd0, 0xd0, REG_MASK, REG_1Byte},
    {0x100b54, 0x11, 0x11, REG_MASK, REG_1Byte},
    {0x110abc, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b57, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110ac8, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110ac9, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110aa7, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110aa8, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110aa9, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110a9f, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x110aa0, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x10335a, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b56, 0x33, 0x33, REG_MASK, REG_1Byte},
    {0x100a58, 0x61, 0x61, REG_MASK, REG_1Byte},
    {0x100a59, 0x0c, 0x0c, REG_MASK, REG_1Byte},
    {0x100a5a, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100a5b, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100a5c, 0x41, 0x41, REG_MASK, REG_1Byte},
    {0x100a5e, 0x41, 0x41, REG_MASK, REG_1Byte},
    {0x100a55, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100a5d, 0x08, 0x08, REG_MASK, REG_1Byte},
    {0x100a5f, 0x08, 0x08, REG_MASK, REG_1Byte},
    {0x103324, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x103325, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x103326, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x103327, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b4a, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b49, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100ba6, 0x11, 0x11, REG_MASK, REG_1Byte},
    {0x100ba5, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100ba8, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100ba9, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100baa, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100bb2, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x133fa6, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100baf, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b9c, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x111eb1, 0x11, 0x00, REG_SET, REG_1Byte},
    {0x111eb2, 0x1111, 0x00, REG_SET, REG_2Bytes},
    {0x100b98, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x122d01, 0x10, 0x10, REG_MASK, REG_1Byte},
    {0x100b90, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x101430, 0x18, 0x00, REG_SET, REG_1Byte},
    {0x101431, 0x18, 0x00, REG_SET, REG_1Byte},
    {0x100b29, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b32, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b33, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b2a, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x10337a, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b42, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x10331e, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b2d, 0x21, 0x21, REG_MASK, REG_1Byte},
    {0x100a0e, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b24, 0x01, 0x01, REG_MASK, REG_1Byte},
    {0x100b07, 0x21, 0x21, REG_MASK, REG_1Byte},
    {0x100ae8, 0x01, 0x01, REG_MASK, REG_1Byte},
};
#endif

