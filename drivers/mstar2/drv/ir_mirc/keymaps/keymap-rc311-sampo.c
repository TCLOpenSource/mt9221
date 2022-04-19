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

/* tcl-tv.h - Keytable for tcl_tv Remote Controller
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
 * this is the remote control that comes with the tcl smart tv
 * which based on STAOS standard.
 */
//customer 0x003F
static struct key_map_table sampo0[] = {
    //yuanerlei 2018-2-9 start
    { 0x01, KEY_POWER },
    { 0x11, KEY_0 },
    { 0x08, KEY_1 },
    { 0x09, KEY_2 },
    { 0x0A, KEY_3 },
    { 0x0B, KEY_4 },
    { 0x0C, KEY_5 },
    { 0x0D, KEY_6 },
    { 0x0E, KEY_7 },
    { 0x0F, KEY_8 },
    { 0x10, KEY_9 },
    { 0x23, KEY_RED },
    { 0x2E, KEY_GREEN },
    { 0x38, KEY_YELLOW },
    { 0x39, KEY_BLUE },
    { 0x2B, KEY_UP },
    { 0x2A, KEY_DOWN },
    { 0x2C, KEY_LEFT },
    { 0x2D, KEY_RIGHT },
    { 0x29, KEY_ENTER },
    { 0x13, KEY_CHANNELUP },
    { 0x14, KEY_CHANNELDOWN },
    { 0x06, KEY_VOLUMEUP },
    { 0x07, KEY_VOLUMEDOWN },
    { 0x1B, KEY_MENU },
    { 0x26, KEY_BACK },
    { 0x05, KEY_MUTE },
   // { 0x806855, KEY_EXIT },
    { 0x28, KEY_RECORD },     // REC
    { 0x03, KEY_INFO },
    { 0x3A, KEY_EPG },        // (C)EPG
//    { 0x8068D5, KEY_MEDIA },
//    { 0x806846, KEY_ARCHIVE },
//    { 0x806881, KEY_CONTEXT_MENU },
/*    { 0x25, KEY_USB },
    { 0x1E, KEY_SLEEP},
    { 0x18, KEY_DATA},// 1
    { 0x8068CC, KEY_LIST},
    { 0x30, KEY_ZOOM},
    { 0x20, KEY_CAMERA},// PIC_MODE
    { 0x19, KEY_AUDIO},
    //{ 0x80684b, KEY_SUBTITLE},
    { 0x31, KEY_STOP},
    { 0x2F, KEY_PAUSE},
    { 0x17, KEY_PLAY},
    { 0x1D, KEY_FORWARD},//>>
    { 0x1A, KEY_REWIND},//<<
    //{ 0x8068D7, KEY_TIME}, //look back
    //{ 0x8068D9, KEY_MALL},
    { 0x02, KEY_KP1 },
    { 0x04, KEY_PRE_CH },
    { 0x3B, KEY_F2 },
*/
    //yuanerlei --2018-2-9 end
};

//customer :0x0A2D
static struct key_map_table sampo1[] = {
    // 1st IR controller.
    { 0x1C, KEY_HOME },
/*  { 0x38, KEY_QIY},
    { 0x0E, KEY_COLOUR},// av
    { 0x0B, KEY_TV},
    { 0x02, KEY_FAC_HDMI1},
    { 0x13, KEY_NEXTSONG},
    { 0x12, KEY_PREVIOUSSONG},
    { 0x2B, KEY_BROWSER},
    { 0x1D, KEY_MIR },
*/
};
static struct key_map_list rc311_samp_map0 = {
    .map = {
        .scan    = sampo0,
        .size    = ARRAY_SIZE(sampo0),
        .name    = "sampo0",
        .headcode     = NUM_KEYMAP_RC311_SAMPO0,
    }
};
static struct key_map_list rc311_samp_map1 = {
    .map = {
        .scan    = sampo1,
        .size    = ARRAY_SIZE(sampo1),
        .name    = "sampo1",
        .headcode     = NUM_KEYMAP_RC311_SAMPO1,
    }
};
int init_key_map_rc311_samp(void)
{
    int ret = 0;
    ret = MIRC_Map_Register(&rc311_samp_map0);
    if(ret < 0)
    {
        IRDBG_ERR("Register Map rc311_samp_map0 Failed\n");
    }
    ret = MIRC_Map_Register(&rc311_samp_map1);
    if(ret < 0)
    {
        IRDBG_ERR("Register Map rc311_samp_map1 Failed\n");
    }
    return ret;
}
EXPORT_SYMBOL(init_key_map_rc311_samp);

void exit_key_map_rc311_samp(void)
{
    MIRC_Map_UnRegister(&rc311_samp_map0);
    MIRC_Map_UnRegister(&rc311_samp_map1);
}
EXPORT_SYMBOL(exit_key_map_rc311_samp);
