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

/* keymap-tcl-tv.h - Keytable for tcl_tv Remote Controller
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



#include <linux/module.h>
#include <linux/kernel.h>
#include "../ir_core.h"
#include "../ir_common.h"
/*
 * Jimmy Hsu <jimmy.hsu@mstarsemi.com>
 * this is the remote control that comes with the tcl smart tv
 * which based on STAOS standard.
 */

static struct key_map_table tcl_rca_tv[] = {
    // 1st IR controller.
    { 0x54, KEY_POWER },
    { 0x3E, KEY_0 },
    { 0x8C, KEY_1 },
    { 0x4C, KEY_2 },
    { 0xCC, KEY_3 },
    { 0x2C, KEY_4 },
    { 0xAC, KEY_5 },
    { 0x6C, KEY_6 },
    { 0xEC, KEY_7 },
    { 0x1C, KEY_8 },
    { 0x9C, KEY_9 },
    { 0x00, KEY_RED },
    { 0x17, KEY_GREEN },
    { 0x27, KEY_YELLOW },
    { 0x1B, KEY_BLUE },
    { 0x9A, KEY_UP },
    { 0x1A, KEY_DOWN },
    { 0x6A, KEY_LEFT },
    { 0xEA, KEY_RIGHT },
    { 0x2F, KEY_ENTER },
    { 0xB4, KEY_CHANNELUP },
    { 0x34, KEY_CHANNELDOWN },
    { 0xF4, KEY_VOLUMEUP },
    { 0x74, KEY_VOLUMEDOWN },
    { 0x10, KEY_HOME },
    { 0x60, KEY_BACK },
    { 0x37, KEY_MUTE },
    { 0x8B, KEY_INFO },
    { 0xC5, KEY_KP0 },        // TV_INPUT
    { 0x48, KEY_CAMERA },     // (C)PICTURE_MODE
    { 0x09, KEY_ZOOM },       // (C)ASPECT RATIO

    // 2nd IR controller.
};

static struct key_map_list tcl_tv_rca_map = {
    .map = {
        .scan    = tcl_rca_tv,
        .size    = ARRAY_SIZE(tcl_rca_tv),
        .name    = NAME_KEYMAP_TCL_RCA_TV,
        .headcode     = NUM_KEYMAP_TCL_RCA_TV,
    }
};

int init_key_map_tcl_rca_tv(void)
{
    return MIRC_Map_Register(&tcl_tv_rca_map);
}
EXPORT_SYMBOL(init_key_map_tcl_rca_tv);

void exit_key_map_tcl_rca_tv(void)
{
    MIRC_Map_UnRegister(&tcl_tv_rca_map);
}
EXPORT_SYMBOL(exit_key_map_tcl_rca_tv);
