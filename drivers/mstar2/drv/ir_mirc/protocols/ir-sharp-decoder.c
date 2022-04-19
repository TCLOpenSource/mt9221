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

/* ir-sharp-decoder.c - handle NEC IR Pulse/Space protocol
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

#define IR_SWDECODE_MODE_BUF_LEN    100

struct sharp_ir_key_t{
    u32 syscode, databit, judgebit, checkbit, map_num;
};

static u8 u8InitFlag_sharp = FALSE;
static IR_sharp_Spec_t sharp;
static u8 previous_pulse = 0;
static u32 previous_duration = 0;
static u8 get_key_status = 0;
static struct sharp_ir_key_t front_key = {0}, back_key = {0};

u8 reverse_8bit(u8 x)
{
    x = (((x & 0xaa) >> 1) | ((x & 0x55) << 1));
    x = (((x & 0xcc) >> 2) | ((x & 0x33) << 2));
    return((x >> 4) | (x << 4));
}

u16 reverse_16bit(u16 x)
{
    x = (((x & 0xaaaa) >> 1) | ((x & 0x5555) << 1));
    x = (((x & 0xcccc) >> 2) | ((x & 0x3333) << 2));
    x = (((x & 0xf0f0) >> 4) | ((x & 0x0f0f) << 4));
    return((x >> 8) | (x << 8));
}

/**
 * ir_sharp_decode() - Decode one SHARP pulse or space
 * @dev:	the struct rc_dev descriptor of the device
 * @duration:	the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_sharp_decode(struct mstar_ir_dev *dev, struct ir_raw_data ev)
{
    IR_sharp_Spec_t *data = &sharp;
    struct ir_scancode *sc = &dev->raw->this_sc;
    struct ir_scancode *prevsc = &dev->raw->prev_sc;
    struct sharp_ir_key_t current_key = {0};
    u32 intervalT = 0;
    int i = 0;

    if (!(dev->enabled_protocols & (1<<IR_TYPE_SHARP)))
        return -EINVAL;
    switch (data->eStatus)
    {
        case STATE_INACTIVE:
            if( ev.duration < SHARP_BIT_1_HIGH )
            {
                IRDBG_INFO("[SHARP] INACTIVE, duration in range\n");
                /* Get legal duration, Save current pulse info */
                previous_pulse = ev.pulse;
                previous_duration = ev.duration;
                data->u8BitCount = 0;
                data->u32DataBits = 0;
                data->eStatus = STATE_BIT_DATA;
                return 0;
            }
            else if(ev.duration >= SHARP_BIT_1_HIGH)
            {
                IRDBG_INFO("[SHARP] INACTIVE, duration NOT in range\n");
                return -EINVAL;
            }
            break;
        case STATE_BIT_DATA:
            /* Collect Intervel and check bit.
             *   1. T = Th + Tl
             *   2. Judge T to logic 1/0
             *   3. parse bit data
             *      1) paese system code as header
             *      2) parse data code
             *      3) parse last 2 bit
             */
            intervalT = 0;
            if( previous_pulse == ev.pulse )
            {
                /* Wrong! pulse sequence should be 10101... */
                IRDBG_ERR("[SHARP] BIT_DATA, pulse wrong! %d->%d\n", previous_pulse, ev.pulse);
                goto bit_data_reset;
            }

            if( ev.pulse == 1 && ev.duration < SHARP_BIT_0_LOW)
            {
                /* Get Th here, update current previous pulse info */
                previous_pulse = ev.pulse;
                previous_duration = ev.duration;
                return 0;
            }
            else if( ev.pulse == 0 && ev.duration >= SHARP_BIT_0_LOW && ev.duration < SHARP_BIT_1_HIGH )
            {
                /* We got both Th and Tl here */
                intervalT = previous_duration+ ev.duration;

                /* Update current previous pulse info */
                previous_pulse = ev.pulse;
                previous_duration = ev.duration;
            }
            else
            {
                /* The duration is incorrect, reset */
                IRDBG_INFO("[SHARP] BIT_DATA, duration not in range\n");
                goto bit_data_reset;
            }

            /* Transfer pulse to logic 1/0 */
            if( intervalT >= SHARP_BIT_0_LOW && intervalT < SHARP_BIT_0_HIGH)
            {
                data->u8BitCount++;
                data->u32DataBits <<= 1;
            }
            else if( intervalT >= SHARP_BIT_1_LOW && intervalT < SHARP_BIT_1_HIGH)
            {
                data->u8BitCount++;
                data->u32DataBits <<= 1;
                data->u32DataBits |= 1;
            }
            else
            {
                IRDBG_INFO("[SHARP] BIT_DATA, interval not in range!\n");
                goto bit_data_reset;
            }

            /* We parse code while we got 15 bit datas */
            if(data->u8BitCount == SHARP_NBITS)
            {
                IRDBG_INFO("[SHARP] get 15 bit data->u32DataBits = %x\n",data->u32DataBits);
                /* Separate system code(head), data code, check bit and judge bit */
                current_key.syscode = (data->u32DataBits & 0x7c00) >> 10;
                current_key.databit = (data->u32DataBits & 0x03fc) >> 2;
                current_key.checkbit = (data->u32DataBits & 0x0002) >> 1;
                current_key.judgebit = data->u32DataBits & 0x0001;
                IRDBG_INFO("[SHARP] Get Keycode: %x_%x_%x_%x\n",current_key.syscode, current_key.databit, current_key.checkbit, current_key.judgebit);
            }
            else
            {
                /* TODO:
                 * In sharp IR protocol, it mentioned key off time,
                 * We can add timer here to decide clear data or not.
                 */
                return 0;
            }

            /*
             * simple check key
             * 1. check the check bit and judge bit first
             * 2. check system code
             */
            /* Check is system code(header) correct */
            for (i= 0;i<dev->support_num;i++)
            {
                if((current_key.syscode == dev->support_ir[i].u32HeadCode) && (dev->support_ir[i].eIRType == IR_TYPE_SHARP))
                {
                    current_key.map_num = dev->support_ir[i].u32HeadCode;
                }
            }
            if(current_key.map_num == 0)
            {
                IRDBG_INFO("[SHARP] system code fail:[%x]\n",current_key.syscode);
                goto bit_data_reset;
            }

            /* check key is front or back key */
            /* get_key_status:
             *  00(0): we have no key
             *  01(1): we have back key
             *  10(2): we have front key
             *  11(3): we have both key
             * */
            if(get_key_status != 3)
            {
                if(current_key.checkbit == 1 && current_key.judgebit== 0)
                {
                    IRDBG_INFO("[SHARP] get Front key\n");
                    get_key_status |= 0x2;
                    front_key = current_key;
                }
                else if(current_key.checkbit == 0 && current_key.judgebit == 1)
                {
                    IRDBG_INFO("[SHARP] get Back key\n");
                    get_key_status |= 0x1;
                    back_key = current_key;
                }
                else
                {
                    IRDBG_INFO("[SHARP] get key judge/check bit fail:[%x][%x]\n",current_key.checkbit, current_key.judgebit);
                    goto bit_data_reset;
                }
            }

            /* Checking data */
            IRDBG_INFO("[SHARP] get_key_status=[%d]\n", get_key_status);
            if(get_key_status == 3)
            {
                IRDBG_INFO("[SHARP] get both key, [%x]:[%x]\n", front_key.databit, ~(back_key.databit)&0x000000ff);
                /* we get both d and d', we can check data now */
                if( !((front_key.databit&0x000000ff)^((~back_key.databit)&0x000000ff)) )
                {
                    IRDBG_INFO("[SHARP] two key match! send [%x] key! \n", reverse_8bit(front_key.databit));
                    sc->scancode = reverse_8bit(front_key.databit);
                    sc->scancode_protocol = (1<<IR_TYPE_SHARP);
                    dev->map_num = front_key.map_num;
                    dev->raw->u8RepeatFlag = 0;

                    data->u8BitCount = 0;
                    data->u32DataBits = 0;
                    previous_pulse = 0;
                    previous_duration = 0;
                    data->eStatus = STATE_INACTIVE;
                    get_key_status = 0;
                    memset(&front_key, 0, sizeof(front_key));
                    memset(&back_key, 0, sizeof(back_key));
                    return 1;
                }
                else
                {
                    IRDBG_INFO("[SHARP] two key NOT match! [%x]:[%x]\n", front_key.databit, back_key.databit);
                }
                get_key_status = 0;
                memset(&front_key, 0, sizeof(front_key));
                memset(&back_key, 0, sizeof(back_key));
            }
            /* Decode Done */
            IRDBG_INFO("[SHARP] decode finish, curren status, get_key_status=%d\n",get_key_status);
        default:
            data->eStatus = STATE_INACTIVE;
            return -EINVAL;
    }
bit_data_reset:
    data->u8BitCount = 0;
    data->u32DataBits = 0;
    previous_pulse = 0;
    previous_duration = 0;
    data->eStatus = STATE_INACTIVE;
    return 0;
}

static struct ir_decoder_handler sharp_handler = {
    .protocols	= (1<<IR_TYPE_SHARP),
    .decode		= ir_sharp_decode,
};

int sharp_decode_init(void)
{
    if(u8InitFlag_sharp == FALSE)
    {
        memset(&sharp,0,sizeof(IR_sharp_Spec_t));
        MIRC_Decoder_Register(&sharp_handler);
        IR_PRINT("[IR Log] Sharp Spec Protocol Init\n");
        u8InitFlag_sharp = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] Sharp Spec Protocol Has been Initialized\n");
    }
    return 0;
}

void sharp_decode_exit(void)
{
    if(u8InitFlag_sharp == TRUE)
    {
        MIRC_Decoder_UnRegister(&sharp_handler);
        u8InitFlag_sharp = FALSE;
    }
}
