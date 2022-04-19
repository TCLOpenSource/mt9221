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

static struct key_map_table beko_rc5_tv[] = {
    { 0x000C, KEY_POWER },
    { 0x0001, KEY_1 }, //KEYCODE_1
    { 0x0002, KEY_2 }, //KEYCODE_2
    { 0x0003, KEY_3 }, //KEYCODE_3
    { 0x0004, KEY_4 }, //KEYCODE_4
    { 0x0005, KEY_5 }, //KEYCODE_5
    { 0x0006, KEY_6 }, //KEYCODE_6
    { 0x0007, KEY_7 }, //KEYCODE_7
    { 0x0008, KEY_8 }, //KEYCODE_8
    { 0x0009, KEY_9 }, //KEYCODE_9
    { 0x0000, KEY_0 }, //KEYCODE_0
    { 0x0037, KEY_RED },
    { 0x0036, KEY_GREEN },
    { 0x0032, KEY_YELLOW },
    { 0x0034, KEY_BLUE },
    { 0x0016, KEY_UP },
    { 0x0017, KEY_DOWN },
    { 0x0013, KEY_LEFT },
    { 0x0012, KEY_RIGHT },
    { 0x0035, KEY_ENTER },
    { 0x0020, KEY_CHANNELUP },
    { 0x0021, KEY_CHANNELDOWN },
    { 0x0010, KEY_VOLUMEUP },
    { 0x0011, KEY_VOLUMEDOWN },
    { 0x002E, KEY_PAGEUP },
    { 0x00A7, KEY_PAGEDOWN },
    //{ 0x000817, KEY_HOME},
    { 0x0019, KEY_MENU },
    { 0x001B, KEY_BACK },
    { 0x000D, KEY_MUTE },
    { 0x000A, KEY_RECORD },     // DVR
    //{ 0x000842, KEY_HELP },       // GUIDE
    { 0x0033, KEY_INFO },
    //{ 0x000840, KEY_KP0 },        // WINDOW
    { 0x0038, KEY_KP1 },        // TV_INPUT
    { 0x0029, KEY_REWIND },
    { 0x002C, KEY_FORWARD },
    { 0x0014, KEY_PREVIOUSSONG },
    { 0x0015, KEY_NEXTSONG },
    { 0x000E, KEY_PLAY },
    { 0x000B, KEY_PAUSE },
    { 0x001A, KEY_STOP },
    //{ 0x000844, KEY_AUDIO },      // (C)SOUND_MODE
    //{ 0x000856, KEY_CAMERA },     // (C)PICTURE_MODE
    //{ 0x00084C, KEY_ZOOM },       // (C)ASPECT_RATIO
    { 0x0022, KEY_CHANNEL },    // (C)CHANNEL_RETURN
    //{ 0x000845, KEY_SLEEP },      // (C)SLEEP
    { 0x0025, KEY_EPG },        // (C)EPG
    //{ 0x000810, KEY_LIST },       // (C)LIST
    { 0x0030, KEY_SUBTITLE },   // (C)SUBTITLE
    { 0x0024, KEY_FN_F1 },      // (C)MTS
    //{ 0x00084E, KEY_FN_F2 },      // (C)FREEZE
    { 0x003C, KEY_FN_F3 },      // (C)TTX
    //{ 0x000809, KEY_FN_F4 },      // (C)CC
    { 0x000F, KEY_FN_F5 },      // (C)TV_SETTING
    //{ 0x000808, KEY_FN_F6 },      // (C)SCREENSHOT
    //{ 0x00080B, KEY_F1 },         // MSTAR_BALANCE
    { 0x0026, KEY_F2 },         // MSTAR_INDEX
    //{ 0x000800, KEY_F3 },         // MSTAR_HOLD
    //{ 0x00080C, KEY_F4 },         // MSTAR_UPDATE
    //{ 0x00084F, KEY_F5 },         // MSTAR_REVEAL
    //{ 0x00085E, KEY_F6 },         // MSTAR_SUBCODE
    //{ 0x000843, KEY_F7 },         // MSTAR_SIZE
    //{ 0x00085F, KEY_F8 },         // MSTAR_CLOCK
    //{ 0x0008FE, KEY_POWER2 },     // FAKE_POWER
    //{ 0x0008FF, KEY_OK },         // KEY_OK
    { 0x0031, KEY_F },         // KEY_F

    // 2nd IR controller.
};

static struct key_map_list mstar_beko_rc5_map = {
    .map = {
        .scan    = beko_rc5_tv,
        .size    = ARRAY_SIZE(beko_rc5_tv),
        .name    = NAME_KEYMAP_BEKO_RC5_TV,
        .headcode    = NUM_KEYMAP_BEKO_RC5_TV,
    }
};

int init_key_map_beko_rc5_tv(void)
{
    return MIRC_Map_Register(&mstar_beko_rc5_map);
}
EXPORT_SYMBOL(init_key_map_beko_rc5_tv);

void exit_key_map_beko_rc5_tv(void)
{
    MIRC_Map_UnRegister(&mstar_beko_rc5_map);
}
EXPORT_SYMBOL(exit_key_map_beko_rc5_tv);
