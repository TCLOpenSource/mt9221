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

#ifndef _IR_TS_H
#define _IR_TS_H
enum IR_TIME_STAGE{
    MDRV_IR_ISR=0,
    MIRC_DATA_STORE,
    MIRC_Data_WAKEUP,
    MIRC_DATA_CTRL_THREAD,
    MIRC_DATA_GET,
    MIRC_DECODE_LIST_ENTRY,
    MIRC_DIFF_PROTOCOL,
    MIRC_DATA_MATCH_SHOTCOUNT,
    MIRC_SEND_KEY_EVENT,
    MIRC_KEYCODE_FROM_MAP,
    MIRC_MAX,
};
enum START_END{
    TIME_START=0,
    TIME_END,
};
struct time_range
{
    unsigned long start_time;
    unsigned long end_time;
    unsigned long diff_time;
};
struct time_stage
{
    unsigned int keycode;
    u8 flag;
    struct time_range time_MDrv_IR_ISR;
    struct time_range time_MIRC_Data_Get;
    struct time_range time_MIRC_Decode_list_entry;
    struct time_range time_MIRC_Diff_protocol;
    struct time_range time_MIRC_Keycode_From_Map;
    struct time_range time_MIRC_Data_Ctrl_Thread;
    struct time_range time_MIRC_Data_Store;
    struct time_range time_MIRC_Data_Match_shotcount;
    struct time_range time_MIRC_Send_key_event;
    struct time_range time_MIRC_Data_Wakeup;
    struct list_head list;
};
void gettime(int flag,u8 stage);
#endif
