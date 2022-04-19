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

/* keymap-panasonic.h - Keytable for mstar_tv Remote Controller
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
 * this is the remote control that comes with the panasonic  smart tv
 * which based on panasonic  standard.
 */

static struct key_map_table p7051_stb[] = {
    { 0x5057, KEY_POWER },

    { 0x3030, KEY_0 },
    { 0x3031, KEY_1 },
    { 0x3032, KEY_2 },
    { 0x3033, KEY_3 },
    { 0x3034, KEY_4 },
    { 0x3035, KEY_5 },
    { 0x3036, KEY_6 },
    { 0x3037, KEY_7 },
    { 0x3038, KEY_8 },
    { 0x3039, KEY_9 },

    { 0x5550, KEY_UP },
    { 0x444E, KEY_DOWN },
    { 0x4C45, KEY_LEFT },
    { 0x5249, KEY_RIGHT },
    { 0x454E, KEY_ENTER },
    //{ 0x454E, KEY_OK },         // KEY_OK

    { 0x562B, KEY_VOLUMEUP },
    { 0x562D, KEY_VOLUMEDOWN },

    { 0x4D45, KEY_HOME},
    { 0x5250, KEY_MENU },
    { 0x5254, KEY_BACK },
    { 0x4633, KEY_KP1 },        // TV_INPUT
};

static struct key_map_list p7051_stb_map = {
    .map = {
        .scan    = p7051_stb,
        .size    = ARRAY_SIZE(p7051_stb),
        .name    = NAME_KEYMAP_P7051_STB,
        .headcode     = NUM_KEYMAP_P7051_STB,
    }
};

int init_key_map_p7051_stb(void)
{
    return MIRC_Map_Register(&p7051_stb_map);
}
EXPORT_SYMBOL(init_key_map_p7051_stb);

void exit_key_map_p7051_stb(void)
{
    MIRC_Map_UnRegister(&p7051_stb_map);
}
EXPORT_SYMBOL(exit_key_map_p7051_stb);
