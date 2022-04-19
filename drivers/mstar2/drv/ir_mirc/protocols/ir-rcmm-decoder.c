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

/* ir-tcl-rcmm-decoder.c - handle RCMM IR Pulse/Space protocol
 *
 * Copyright (C) 2010 by Mauro Carvalho Chehab <mchehab@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/bitrev.h>
#include <linux/module.h>
#include "../ir_core.h"
#include "../ir_common.h"

static u8 u8InitFlag_rcmm = FALSE;
static IR_RCMM_Spec_t rcmm;
static unsigned long  KeyTime;

static inline u8 to_databits(u32 duration)
{
    u8 data = 0xff;

    if (eq_margin(duration, RCMM_BIT_00_LWB, RCMM_BIT_00_UPB)){
        data = 0;
    }
    else if (eq_margin(duration, RCMM_BIT_01_LWB, RCMM_BIT_01_UPB)){
        data = 1;
    }
    else if (eq_margin(duration, RCMM_BIT_10_LWB, RCMM_BIT_10_UPB)){
        data = 2;
    }
    else if (eq_margin(duration, RCMM_BIT_11_LWB, RCMM_BIT_11_UPB)){
        data = 3;
    }
    return data;
}
static inline u32 data32bits2code(u32 data)
{
    u32 code = 0;
    int i = 0;
    for(i = 0; i < 16 ;i++)
    {
        code |= (data&3);
        code <<= 2;
        data >>= 2;
    }

    return code;
}


/**
 * ir_rcmm_decode() - Decode one RCMM pulse or space
 * @dev:    the struct rc_dev descriptor of the device
 * @duration:   the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_rcmm_decode(struct mstar_ir_dev*dev, struct ir_raw_data ev)
{
    IR_RCMM_Spec_t *data = &rcmm;
    struct ir_scancode *sc = &dev->raw->this_sc;

    u8 u8Cmd= 0;
    u8 u8Addr= 0;
    u8 u8RepeatFlag= 0;
    int i = 0;

    static volatile u32 data_bits;
    static volatile u32 curr_bit;
    static volatile u32 total_bit;

    if (!(dev->enabled_protocols & (1<<IR_TYPE_RCMM)))
        return 0;

    switch (data->eStatus)
    {
        case STATE_INACTIVE:
            if (!ev.pulse)
            {
                break;
            }

            if (eq_margin(ev.duration, RCMM_HEADER_PULSE_LWB, RCMM_HEADER_PULSE_UPB))
            {
                data->eStatus = STATE_HEADER_SPACE;
                return 0;
            }
            else
                break;

        case STATE_HEADER_SPACE:
            if (ev.pulse)
                break;
            if (eq_margin(ev.duration, RCMM_HEADER_SPACE_LWB, RCMM_HEADER_SPACE_UPB))
            {
                IRDBG_INFO("[RCMM] Start decode\n");
                data_bits = 0;
                curr_bit = 0;
                total_bit = 0;
                data->eStatus = STATE_BIT_SPACE;
                return 0;
            }
            else{
                data->eStatus = STATE_INACTIVE;
            }

            break;
        case STATE_BIT_SPACE :
            if (!ev.pulse)
            {
                IRDBG_INFO("error !ev.pulse\n");
                data->eStatus = STATE_INACTIVE;
                break;
            }
            data->eStatus = STATE_BIT_PULSE;
            break;
        case STATE_BIT_PULSE:
            if (ev.pulse)
            {
                IRDBG_INFO("error ev.pulse\n");
                data->eStatus = STATE_INACTIVE;
                break;
            }
            {
                u8 temp = to_databits(ev.duration);
                if(temp == 0xff){
                    IRDBG_INFO("error ev.duration:%d\n",ev.duration);
                    data->eStatus = STATE_INACTIVE;
                    break;
                }else{
                    data->eStatus = STATE_BIT_SPACE;
                }
                data_bits |= temp<<curr_bit;
                if(curr_bit == 0){
                    if(data_bits == 0)
                        total_bit = RCMM_NBITS;
                    else
                    {
                        IRDBG_INFO("Not Support IR\n");
                    }
                }
                curr_bit+=2;
                if(curr_bit >= total_bit){

                    if(total_bit == RCMM_NBITS){
                        static u32 last_scancode = 0;
                        u32 code = data32bits2code(data_bits);
                        u8RepeatFlag = (last_scancode == code)?1:0;
                        last_scancode = code;
                        IRDBG_WRN("code = %08x\n",code);
                        code>>=2;
                        u8Cmd = code&0xff;
                        code >>= 8;
                        u8Addr = code&0x3f;


                       IRDBG_INFO("[RCMM] u8Addr = %x u8Cmd = %x u8RepeatFlag = %x\n",u8Addr,u8Cmd,u8RepeatFlag);
                       IRDBG_INFO("[RCMM] TIME =%ld\n",(MIRC_Get_System_Time()-KeyTime));
                       KeyTime = MIRC_Get_System_Time();

                        for (i= 0;i<dev->support_num;i++)
                        {
                            if((u8Addr== dev->support_ir[i].u32HeadCode)&&(dev->support_ir[i].eIRType == IR_TYPE_RCMM)&&(dev->support_ir[i].u8Enable == TRUE))
                            {
                                u32 speed = dev->support_ir[i].u32IRSpeed;
                                if (((speed != 0)&&( data->u8RepeatTimes >= (speed - 1)))
                                        || ((speed == 0)&&(data->u8RepeatTimes >= dev->speed)))
                                {
                                    sc->scancode_protocol = (1<<IR_TYPE_RCMM);
#ifdef CONFIG_MIRC_INPUT_DEVICE
                                    sc->scancode = (u8Addr<<8) | u8Cmd;
                                    dev->map_num = dev->support_ir[i].u32HeadCode;
                                    dev->raw->u8RepeatFlag = u8RepeatFlag;
#else
                                    sc->scancode = (u8Cmd<<8) | u8RepeatFlag;
#endif
                                    memset(data,0,sizeof(IR_RCMM_Spec_t));
                                    return 1;
                                }
                            }
                        }
                        if(u8RepeatFlag)
                             ++data->u8RepeatTimes;
                        else
                            data->u8RepeatTimes = 0;

                    }
                    else{
                        IRDBG_INFO("Not Support IR\n");
                    }
                    data->eStatus = STATE_INACTIVE;
                }
            }
             break;
             default:break;
        }
    return 0;
}

static struct ir_decoder_handler rcmm_handler =
{
    .protocols  = (1<<IR_TYPE_RCMM),
    .decode     = ir_rcmm_decode,
};

int rcmm_decode_init(void)
{
    if(u8InitFlag_rcmm == FALSE)
    {
        memset(&rcmm,0,sizeof(IR_RCMM_Spec_t));
        MIRC_Decoder_Register(&rcmm_handler);
        IR_PRINT("[IR Log] rcmm Spec protocol init\n");
        u8InitFlag_rcmm = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] rcmm Spec Protocol Has been Initialized\n");

    }
    return 0;
}

void rcmm_decode_exit(void)
{
    if(u8InitFlag_rcmm == TRUE)
    {
        MIRC_Decoder_UnRegister(&rcmm_handler);
        u8InitFlag_rcmm = FALSE;
    }
}
