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

#ifndef KEY_ASSISTANT
#define KEY_ASSISTANT  0x247
#endif

/*
 * Jimmy Hsu <jimmy.hsu@mstarsemi.com>
 * this is the remote control that comes with the mstar smart tv
 * which based on STAOS standard.
 */

static struct key_map_table mstar_tv[] = {
    { 0x46, KEY_POWER },
    { 0x50, KEY_0 },
    { 0x49, KEY_1 },
    { 0x55, KEY_2 },
    { 0x59, KEY_3 },
    { 0x4D, KEY_4 },
    { 0x51, KEY_5 },
    { 0x5D, KEY_6 },
    { 0x48, KEY_7 },
    { 0x54, KEY_8 },
    { 0x58, KEY_9 },
    { 0x47, KEY_RED },
    { 0x4B, KEY_GREEN },
    { 0x57, KEY_YELLOW },
    { 0x5B, KEY_BLUE },
    { 0x52, KEY_UP },
    { 0x13, KEY_DOWN },
    { 0x06, KEY_LEFT },
    { 0x1A, KEY_RIGHT },
    { 0x0F, KEY_ENTER },
    { 0x1F, KEY_CHANNELUP },
    { 0x19, KEY_CHANNELDOWN },
    { 0x16, KEY_VOLUMEUP },
    { 0x15, KEY_VOLUMEDOWN },
    { 0x03, KEY_PAGEUP },
    { 0x05, KEY_PAGEDOWN },
    { 0x17, KEY_HOME},
    { 0x07, KEY_MENU },
    { 0x1B, KEY_BACK },
    { 0x5A, KEY_MUTE },
    { 0x0D, KEY_RECORD },     // DVR
    { 0x42, KEY_HELP },       // GUIDE
    { 0x14, KEY_INFO },
    { 0x40, KEY_KP0 },        // WINDOW
    { 0x04, KEY_KP1 },        // TV_INPUT
    { 0x21, KEY_ASSISTANT },  // ASSISTANT
    { 0x27, KEY_POUND },      // POUND
    { 0x0E, KEY_REWIND },
    { 0x12, KEY_FORWARD },
    { 0x02, KEY_PREVIOUSSONG },
    { 0x1E, KEY_NEXTSONG },
    { 0x01, KEY_PLAY },
    { 0x1D, KEY_PAUSE },
    { 0x11, KEY_STOP },
    { 0x44, KEY_AUDIO },      // (C)SOUND_MODE
    { 0x56, KEY_CAMERA },     // (C)PICTURE_MODE
    { 0x4C, KEY_ZOOM },       // (C)ASPECT_RATIO
    { 0x5C, KEY_CHANNEL },    // (C)CHANNEL_RETURN
    { 0x45, KEY_SLEEP },      // (C)SLEEP
    { 0x4A, KEY_EPG },        // (C)EPG
    { 0x10, KEY_LIST },       // (C)LIST
    { 0x53, KEY_SUBTITLE },   // (C)SUBTITLE
    { 0x41, KEY_FN_F1 },      // (C)MTS
    { 0x26, KEY_FN_F1 },      // (C)MTS I/II
    { 0x4E, KEY_FN_F2 },      // (C)FREEZE
    { 0x0A, KEY_FN_F3 },      // (C)TTX
    { 0x23, KEY_FN_F3 },      // (C)TTX   //TEXT
    { 0x09, KEY_FN_F4 },      // (C)CC
    { 0x1C, KEY_FN_F5 },      // (C)TV_SETTING
    { 0x08, KEY_FN_F6 },      // (C)SCREENSHOT
    { 0x0B, KEY_F1 },         // MSTAR_BALANCE
    { 0x18, KEY_F2 },         // MSTAR_INDEX
    { 0x00, KEY_F3 },         // MSTAR_HOLD
    { 0x0C, KEY_F4 },         // MSTAR_UPDATE
    { 0x4F, KEY_F5 },         // MSTAR_REVEAL
    { 0x5E, KEY_F6 },         // MSTAR_SUBCODE
    { 0x43, KEY_F7 },         // MSTAR_SIZE
    { 0x5F, KEY_F8 },         // MSTAR_CLOCK
    { 0xFE, KEY_POWER2 },     // FAKE_POWER
    { 0xFF, KEY_OK },         // KEY_OK
    { 0x22, KEY_F12 },         // KEY_F12
    { 0x24, KEY_FAVORITES },  // FAV
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
    { 0x25, KEY_AUDIO_DESC }, // AD
#endif
    // 2nd IR controller.
};

static struct key_map_list mstar_tv_map = {
    .map = {
        .scan    = mstar_tv,
        .size    = ARRAY_SIZE(mstar_tv),
        .name    = NAME_KEYMAP_MSTAR_TV,
        .headcode     = NUM_KEYMAP_MSTAR_TV,
    }
};

int init_key_map_mstar_tv(void)
{
    return MIRC_Map_Register(&mstar_tv_map);
}
EXPORT_SYMBOL(init_key_map_mstar_tv);

void exit_key_map_mstar_tv(void)
{
    MIRC_Map_UnRegister(&mstar_tv_map);
}
EXPORT_SYMBOL(exit_key_map_mstar_tv);
