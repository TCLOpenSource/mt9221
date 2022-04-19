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

#ifndef _IR_CONFIG_H_
#define _IR_CONFIG_H_
#include "mstar_ir.h"
#include "ir_core.h"
#include "ir_common.h"

/****************** For Customer Config  Start [can modify by customers] ********************/

//Number of IR should this chip supported
#define IR_SUPPORT_NUM 1

//Add & Modify Customer IR with Differ Headcode Here
static IR_Profile_t ir_config[IR_SUPPORT_NUM]=
{
//   protocol_type, Headcode, IRSpeed ,enable
    {IR_TYPE_NEC,NUM_KEYMAP_MSTAR_TV,0,1},           // Mstar IR customer code
    //{IR_TYPE_METZ,NUM_KEYMAP_METZ_RM18_TV,0,1},           // metz rm18 IR customer code
    //{IR_TYPE_METZ,NUM_KEYMAP_METZ_RM19_TV,0,1},           // metz rm19 IR customer code
    //{IR_TYPE_TOSHIBA,NUM_KEYMAP_SKYWORTH_TV,0,1},        //skyworth toshiba ir
    //{IR_TYPE_NEC,NUM_KEYMAP_CHANGHONG_TV,0,1},           // changhong_RL78B /Toshiba CT-90436 IR customer code
    //{IR_TYPE_NEC,NUM_KEYMAP_HISENSE_TV,0,1},           // Hisense IR customer code
    //{IR_TYPE_RCA,NUM_KEYMAP_TCL_RCA_TV,0,1},           // TCL RCA  customer code
    //{IR_TYPE_P7051,NUM_KEYMAP_P7051_STB,0,1},       // Panasonic 7051 IR customer code
    //{IR_TYPE_RC5,NUM_KEYMAP_RC5_TV,0,1},           // RC5 customer code
    //{IR_TYPE_RC6,NUM_KEYMAP_KATHREIN_TV,0,1},           //Kathrein RC6 customer code
    //{IR_TYPE_KONKA,NUM_KEYMAP_KONKA_TV,2,1},
    //{IR_TYPE_BEKO_RC5,NUM_KEYMAP_BEKO_RC5_TV,0,1},           //Beko RC5 customer code
};

//IR Debug level for customer setting
static IR_DBG_LEVEL_e ir_dbglevel = IR_DBG_ERR;

//IR Speed level for customer setting
static IR_SPEED_LEVEL_e ir_speed = IR_SPEED_FAST_H;

/****************** For Customer Config  End [can modify by customers] ********************/

#endif
