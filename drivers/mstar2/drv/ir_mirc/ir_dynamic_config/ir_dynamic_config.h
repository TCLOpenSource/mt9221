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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    ir_dynamic_config.h
/// @brief  Load IR config from INI config file
/// @author Vick.Sun@MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __IR_DYNAMIC_CONFIG__
#define __IR_DYNAMIC_CONFIG__

#include <linux/input.h>

#define IR_CONFIG_PATH_LEN 256
/***********************************************************************************/
//&step2:  IR  protocols Type enum
typedef enum{
    IR_TYPE_NEC = 0,        /* NEC protocol */
    IR_TYPE_RC5,            /* RC5 protocol*/
    IR_TYPE_RC6,            /* RC6 protocol*/
    IR_TYPE_RCMM,           /* RCMM protocol*/
    IR_TYPE_KONKA,          /* Konka protocol*/
    IR_TYPE_HAIER,          /* Haier protocol*/
    IR_TYPE_RCA,            /*TCL RCA protocol**/
    IR_TYPE_P7051,          /*Panasonic 7051 protocol**/
    IR_TYPE_TOSHIBA,        /*Toshiba protocol*/
    IR_TYPE_RC5X,           /* RC5 ext protocol*/
    IR_TYPE_RC6_MODE0,      /* RC6  mode0 protocol*/
    IR_TYPE_METZ,           /*Metz remote control unit protocol*/
    IR_TYPE_PANASONIC,      /* PANASONIC protocol*/
    IR_TYPE_RC311,          /*RC311(RC-328ST) remote control */
    IR_TYPE_BEKO_RC5,           /* Beko protocal(customized RC5) */
    IR_TYPE_SHARP,      /* Sharp protocol*/
    IR_TYPE_MAX,
}IR_Type_e;

typedef enum{
    HW_DECODE = 0,
    SW_DECODE,
}DECODE_Type_e;


//Description  of IR
typedef struct IR_Profile_s {
    IR_Type_e eIRType;
    u32 u32HeadCode;
    u32 u32IRSpeed;
    u8 u8Enable;
}IR_Profile_t;

struct key_map_table {
    u32 scancode;
    u32 keycode;
};

struct key_map {
    struct key_map_table    *scan;
    unsigned int        size;   /* Max number of entries */
    const char      *name;
    u32              headcode;
};

struct key_map_list {
    struct list_head     list;
    struct key_map map;
};

#define KEYMAP_PAIR(x)      {#x,x}
#define IRPROTOCOL_PAIR(x)  {#x,IR_TYPE_##x}
typedef struct {
    char *key;
    u32 value;
}key_value_t;

//load ir_config_ini for pm51
#if defined(CONFIG_MSTAR_PM)
#define IR_HEAD16_KEY_PM_CFG_OFFSET   5
#define IR_HEAD32_KEY_PM_CFG_OFFSET   25
#define IR_SUPPORT_PM_NUM_MAX         32

typedef struct IR_PM51_Profile_s {
    IR_Type_e eIRType;
    u32 u32Header;
    u32 u32PowerKey;
}IR_PM51_Profile_t;

typedef struct by_pass_kernel_keymap_Profile_s {
    DECODE_Type_e eDECType;
}by_pass_kernel_keymap_Profile_t;


extern ssize_t MDrv_PM_Set_IRCfg(const u8 *buf, size_t size);
#endif


int MIRC_Map_Register(struct key_map_list *map);
void MIRC_Map_UnRegister(struct key_map_list *map);
int MIRC_IRCustomer_Add(IR_Profile_t *stIr);
int MIRC_IRCustomer_Remove(IR_Profile_t *stIr);
int MIRC_IRCustomer_Init(void);
void mstar_ir_reinit(void);
u32 MIRC_Get_Keycode(u32 keymapnum, u32 scancode);
unsigned int get_ir_keycode(unsigned int scancode);

#endif //__IR_DYNAMIC_CONFIG__
