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

static struct key_map_table rc5_tv[] = {
    { 0x00080C, KEY_POWER },
    { 0x000800, KEY_0 },
    { 0x000801, KEY_1 },
    { 0x000802, KEY_2 },
    { 0x000803, KEY_3 },
    { 0x000804, KEY_4 },
    { 0x000805, KEY_5 },
    { 0x000806, KEY_6 },
    { 0x000807, KEY_7 },
    { 0x000808, KEY_8 },
    { 0x000809, KEY_9 },
    { 0x000837, KEY_RED },
    { 0x000836, KEY_GREEN },
    { 0x000832, KEY_YELLOW },
    { 0x000834, KEY_BLUE },
    { 0x000812, KEY_UP },
    { 0x000813, KEY_DOWN },
    { 0x000815, KEY_LEFT },
    { 0x000816, KEY_RIGHT },
    { 0x000814, KEY_ENTER },
    { 0x000820, KEY_CHANNELUP },
    { 0x000821, KEY_CHANNELDOWN },
    { 0x000810, KEY_VOLUMEUP },
    { 0x000811, KEY_VOLUMEDOWN },
    //{ 0x000803, KEY_PAGEUP },
    //{ 0x000805, KEY_PAGEDOWN },
    //{ 0x000817, KEY_HOME},
    { 0x000835, KEY_MENU },
    { 0x000833, KEY_BACK },
    { 0x00080d, KEY_MUTE },
    { 0x00080D, KEY_RECORD },     // DVR
    { 0x000842, KEY_HELP },       // GUIDE
    { 0x000814, KEY_INFO },
    { 0x000840, KEY_KP0 },        // WINDOW
    { 0x000804, KEY_KP1 },        // TV_INPUT
    { 0x00080E, KEY_REWIND },
    { 0x000812, KEY_FORWARD },
    { 0x000802, KEY_PREVIOUSSONG },
    { 0x00081E, KEY_NEXTSONG },
    { 0x00082e, KEY_PLAY },
    { 0x000830, KEY_PAUSE },
    { 0x00082f, KEY_STOP },
    { 0x000844, KEY_AUDIO },      // (C)SOUND_MODE
    { 0x000856, KEY_CAMERA },     // (C)PICTURE_MODE
    { 0x00084C, KEY_ZOOM },       // (C)ASPECT_RATIO
    { 0x00085C, KEY_CHANNEL },    // (C)CHANNEL_RETURN
    { 0x000845, KEY_SLEEP },      // (C)SLEEP
    { 0x00084A, KEY_EPG },        // (C)EPG
    { 0x000810, KEY_LIST },       // (C)LIST
    { 0x000853, KEY_SUBTITLE },   // (C)SUBTITLE
    { 0x000841, KEY_FN_F1 },      // (C)MTS
    { 0x00084E, KEY_FN_F2 },      // (C)FREEZE
    { 0x00080A, KEY_FN_F3 },      // (C)TTX
    { 0x000809, KEY_FN_F4 },      // (C)CC
    { 0x00081C, KEY_FN_F5 },      // (C)TV_SETTING
    { 0x000808, KEY_FN_F6 },      // (C)SCREENSHOT
    { 0x00080B, KEY_F1 },         // MSTAR_BALANCE
    { 0x000818, KEY_F2 },         // MSTAR_INDEX
    { 0x000800, KEY_F3 },         // MSTAR_HOLD
    { 0x00080C, KEY_F4 },         // MSTAR_UPDATE
    { 0x00084F, KEY_F5 },         // MSTAR_REVEAL
    { 0x00085E, KEY_F6 },         // MSTAR_SUBCODE
    { 0x000843, KEY_F7 },         // MSTAR_SIZE
    { 0x00085F, KEY_F8 },         // MSTAR_CLOCK
    { 0x0008FE, KEY_POWER2 },     // FAKE_POWER
    { 0x0008FF, KEY_OK },         // KEY_OK

    // 2nd IR controller.
};

static struct key_map_list mstar_rc5_map = {
    .map = {
        .scan    = rc5_tv,
        .size    = ARRAY_SIZE(rc5_tv),
        .name    = NAME_KEYMAP_RC5_TV,
        .headcode    = NUM_KEYMAP_RC5_TV,
    }
};

int init_key_map_rc5_tv(void)
{
    return MIRC_Map_Register(&mstar_rc5_map);
}
EXPORT_SYMBOL(init_key_map_rc5_tv);

void exit_key_map_rc5_tv(void)
{
    MIRC_Map_UnRegister(&mstar_rc5_map);
}
EXPORT_SYMBOL(exit_key_map_rc5_tv);
