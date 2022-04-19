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

/* keymap-mstar-tv.h - Keytable for mstar_tv Remote Controller
 *
 * keymap imported from ir-keymaps.c
 *
 * Copyright (c) 2010 by Mauro Carvalho Chehab <mchehab@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include "../ir_core.h"
#include "../ir_common.h"

/*
 * Jimmy Hsu <jimmy.hsu@mstarsemi.com>
 * this is the remote control that comes with the mstar smart tv
 * which based on STAOS standard.
 */

static struct key_map_table metz_rm18[] = {
    { 0x00, KEY_0 },
    { 0x01, KEY_1 },
    { 0x02, KEY_2 },
    { 0x03, KEY_3 },
    { 0x04, KEY_4 },
    { 0x05, KEY_5 },
    { 0x06, KEY_6 },
    { 0x07, KEY_7 },
    { 0x08, KEY_8 },
    { 0x09, KEY_9 },
    { 0x0A, KEY_POWER },
    { 0x11, KEY_BLUE },
    { 0x12, KEY_YELLOW },
    //{ 0x13, KEY_WHITE},
    { 0x14, KEY_GREEN },
    { 0x15, KEY_RED },
    { 0x16, KEY_UP },
    { 0x17, KEY_DOWN },
    { 0x18, KEY_LEFT },
    { 0x19, KEY_RIGHT },
    { 0x1A, KEY_VOLUMEUP },
    { 0x1B, KEY_VOLUMEDOWN },
    { 0x20, KEY_BACK },
    { 0x22, KEY_MUTE },
    { 0x24, KEY_EPG },        // (C)EPG
    { 0x26, KEY_OK },         // KEY_OK
    { 0x27, KEY_EXIT },
    { 0x28, KEY_REWIND },
    { 0x2A, KEY_PLAY },
    { 0x2B, KEY_STOP },
    { 0x2C, KEY_RECORD },     // DVR
    { 0x2D, KEY_PAGEUP },
    { 0x2E, KEY_PAGEDOWN },
    { 0x2F, KEY_HOME},
    // 2nd IR controller.
};

static struct key_map_list metz_rm18_map = {
    .map = {
        .scan    = metz_rm18,
        .size    = ARRAY_SIZE(metz_rm18),
        .name    = NAME_KEYMAP_METZ_RM18_TV,
        .headcode     = NUM_KEYMAP_METZ_RM18_TV,
    }
};

int init_key_map_metz_rm18(void)
{
    return MIRC_Map_Register(&metz_rm18_map);
}
EXPORT_SYMBOL(init_key_map_metz_rm18);

void exit_key_map_metz_rm18(void)
{
    MIRC_Map_UnRegister(&metz_rm18_map);
}
EXPORT_SYMBOL(exit_key_map_metz_rm18);
