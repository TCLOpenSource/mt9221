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
#if defined(CONFIG_MSTAR_MIU_AUTO_STR)
#define CONFIG_MSTAR_STANDBY_MIU_TABLE_SUPPORT
#endif

#define STANDBY_MODE_TEST_OK_KERNEL

#define STANDBY_MODE_INV_TABLE_SIZE             115
#define STANDBY_MODE_INV_BUF_SIZE               129
#define STANDBY_MODE_MIU_TABLE_SIZE             1
#define STANDBY_MODE_MIU_BUF_SIZE               2


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
    {0x100b41, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b46, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b47, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b44, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b45, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b6b, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x103330, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x10331e, 0x01, 0x00, REG_SET, REG_1Byte, 1},
    {0x101e39, 0x00, 0x00, REG_SET, REG_1Byte, 1},
    {0x100a33, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100b6c, 0x01, 0x10, REG_MASK, REG_1Byte, 1},
    {0x100b62, 0x01, 0x0d, REG_MASK, REG_1Byte, 1},
    {0x100b61, 0x00, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b68, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b63, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b6c, 0x10, 0x00, REG_MASK, REG_1Byte, 1},
    {0x100b69, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b67, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b72, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b60, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b54, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x110abc, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100b54, 0x10, 0x10, REG_MASK, REG_1Byte, 1},
    {0x100b57, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110ac8, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110ac9, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110aa7, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110aa8, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110aa9, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110a9f, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x110aa0, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x10335a, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b56, 0x03, 0x13, REG_MASK, REG_1Byte, 1},
    {0x100b56, 0x30, 0x33, REG_MASK, REG_1Byte, 1},
    {0x100a58, 0x60, 0xfd, REG_MASK, REG_1Byte, 1},
    {0x100a59, 0x0c, 0x1c, REG_MASK, REG_1Byte, 1},
    {0x100a5a, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100a5b, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100a5c, 0x01, 0x5d, REG_MASK, REG_1Byte, 1},
    {0x100a5c, 0x40, 0x5d, REG_MASK, REG_1Byte, 1},
    {0x100a5e, 0x01, 0x5d, REG_MASK, REG_1Byte, 1},
    {0x100a5e, 0x40, 0x5d, REG_MASK, REG_1Byte, 1},
    {0x100a55, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100a58, 0x01, 0xfd, REG_MASK, REG_1Byte, 1},
    {0x100a5d, 0x08, 0xef, REG_MASK, REG_1Byte, 1},
    {0x100a5f, 0x08, 0xef, REG_MASK, REG_1Byte, 1},
    {0x103324, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x103325, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x103326, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x103327, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100b4a, 0x01, 0x0d, REG_MASK, REG_1Byte, 1},
    {0x100b49, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100ba6, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100ba5, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100ba8, 0x01, 0x21, REG_MASK, REG_1Byte, 1},
    {0x100ba9, 0x01, 0x21, REG_MASK, REG_1Byte, 1},
    {0x100baa, 0x01, 0x21, REG_MASK, REG_1Byte, 1},
    {0x100bb2, 0x01, 0x21, REG_MASK, REG_1Byte, 1},
    {0x133fa6, 0x01, 0xff, REG_MASK, REG_1Byte, 1},
    {0x100baf, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b9c, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x15231e, 0x01, 0x00, REG_SET, REG_1Byte, 1},
    {0x152320, 0x0101, 0x00, REG_SET, REG_2Bytes, 1},
    {0x152322, 0x0101, 0x00, REG_SET, REG_2Bytes, 1},
    {0x152324, 0x0100, 0x00, REG_SET, REG_2Bytes, 1},
    {0x152342, 0x01, 0x00, REG_SET, REG_1Byte, 1},
    {0x152344, 0x0101, 0x00, REG_SET, REG_2Bytes, 1},
    {0x152380, 0x01, 0x00, REG_SET, REG_1Byte, 1},
    {0x1523e0, 0x0101, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111eb1, 0x11, 0x00, REG_SET, REG_1Byte, 1},
    {0x111eb2, 0x1111, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111e74, 0x0001, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111ee0, 0x0000, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111e70, 0x0000, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111e00, 0x0000, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111e02, 0x0000, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111e84, 0xffff, 0x00, REG_SET, REG_2Bytes, 1},
    {0x111e86, 0xffff, 0x00, REG_SET, REG_2Bytes, 1},
    {0x100b98, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x122d01, 0x10, 0x10, REG_MASK, REG_1Byte, 1},
    {0x100b90, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b86, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x101430, 0x18, 0x00, REG_SET, REG_1Byte, 1},
    {0x101431, 0x18, 0x00, REG_SET, REG_1Byte, 1},
    {0x100b29, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b32, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b33, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b2a, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x10337a, 0x01, 0x0d, REG_MASK, REG_1Byte, 1},
    {0x100b2f, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b42, 0x01, 0x1d, REG_MASK, REG_1Byte, 1},
    {0x100b2d, 0x21, 0x21, REG_MASK, REG_1Byte, 1},
    {0x100a0e, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b24, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x100b07, 0x21, 0x21, REG_MASK, REG_1Byte, 1},
    {0x100ae8, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x1132c0, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x032506, 0x0218, 0x00, REG_SET, REG_2Bytes, 1},
//    {0x0325e6, 0x02, 0x00, REG_MASK, REG_1Byte, 1},
//    {0x032527, 0x80, 0x80, REG_MASK, REG_1Byte, 1},
//    {0x032538, 0xfffe, 0x00, REG_SET, REG_2Bytes, 1},
    {0x411907, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x411904, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x41190e, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x411905, 0x40, 0x40, REG_MASK, REG_1Byte, 1},
    {0x411905, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x411908, 0x01, 0x01, REG_MASK, REG_1Byte, 1},
    {0x412107, 0x10, 0x10, REG_MASK, REG_1Byte, 1},
    {0x371a66, 0x49, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x371a62, 0x01, 0x00, REG_SET, REG_1Byte, 1},   // aai block
    {0x371a60, 0x38, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x371a68, 0x09, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x371a6a, 0x03, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x371a00, 0x00, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x371ac0, 0x00, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x371a04, 0x00, 0x00, REG_SET, REG_1Byte, 1},   // aia block 
    {0x110b62, 0x7d, 0x00, REG_SET, REG_1Byte, 1},   // aia block
    {0x110b63, 0x01, 0x00, REG_SET, REG_1Byte, 1},   // aia block
};

static const STANDBY_MODE_TABLE standby_mode_miu_str_table[] =
{
    {0x152bd2, 0x0001, 0x00, REG_SET, REG_2Bytes, 1},
};

static const STANDBY_MODE_TABLE standby_mode_board_check_table[] =
{
    {0x001e03, 0x01, 0x00, REG_SET, REG_1Byte, 1},
};
#endif


