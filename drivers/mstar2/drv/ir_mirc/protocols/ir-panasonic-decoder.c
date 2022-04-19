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

/* ir-panasonic-decoder.c - handle PANASONIC IR Pulse/Space protocol
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

static u8 u8InitFlag_panasonic = FALSE;
static u32 speed = 0;
static unsigned long  KeyTime = 0;
static unsigned long KeyTime_repeat = 0;
static IR_PANASONIC_Spec_t panasonic;

/**
 * ir_panasonic_decode() - Decode one PANASONIC pulse or space
 * @dev:    the struct rc_dev descriptor of the device
 * @duration:   the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_panasonic_decode(struct mstar_ir_dev *dev, struct ir_raw_data ev)
{
    IR_PANASONIC_Spec_t *data = &panasonic;
    struct ir_scancode *sc = &dev->raw->this_sc;
    struct ir_scancode *prevsc = &dev->raw->prev_sc;
    u8 i = 0;
    u8 u8Custom_1 = 0;
    u8 u8Custom_2 = 0;
    u8 u8Custom_3 = 0;
    u8 u8Custom_K = 0;
    u8 u8Data = 0;
    u8 u8Verify = 0;
    u32 u32Headcode = 0;

    if(!(dev->enabled_protocols & (1<<IR_TYPE_PANASONIC)))
        return -EINVAL;
    switch (data->eStatus)
    {

    case STATE_INACTIVE:
        if(!ev.pulse)
            break;
        if(eq_margin(ev.duration,PANASONIC_HEADER_PULSE_LWB,PANASONIC_HEADER_PULSE_UPB))
        {
            data->u8BitCount = 0;
            data->u64DataBits = 0;
            data->eStatus = STATE_HEADER_SPACE;
            return 0;
        }
        else
            break;

    case STATE_HEADER_SPACE:
        if(ev.pulse)
            break;
        if(eq_margin(ev.duration,PANASONIC_HEADER_SPACE_LWB,PANASONIC_HEADER_SPACE_UPB))
        {
            data->eStatus = STATE_BIT_PULSE;
            return 0;
        }
        break;

    case STATE_BIT_PULSE:
        if(!ev.pulse)
            break;
        if(!eq_margin(ev.duration,PANASONIC_BIT_PULSE_LWB,PANASONIC_BIT_PULSE_UPB))
            break;

        data->eStatus = STATE_BIT_SPACE;
        return 0;

    case STATE_BIT_SPACE:
        if(ev.pulse)
            break;
        data->u64DataBits <<= 1;

        if(eq_margin(ev.duration,PANASONIC_BIT_1_SPACE_LWB,PANASONIC_BIT_1_SPACE_UPB))
            data->u64DataBits |= 1;
        else if(!eq_margin(ev.duration,PANASONIC_BIT_0_SPACE_LWB,PANASONIC_BIT_0_SPACE_UPB))
            break;
        data->u8BitCount++;

        if(data->u8BitCount == PANASONIC_NBITS)
        {
            u8Custom_1  = bitrev8((data->u64DataBits >> 40) & 0xff);
            u8Custom_2  = bitrev8((data->u64DataBits >> 32) & 0xff);
            u8Custom_3  = bitrev8((data->u64DataBits >> 24) & 0xff);
            u8Custom_K  = bitrev8((data->u64DataBits >> 16) & 0xff);
            u8Data      = bitrev8((data->u64DataBits >>  8) & 0xff);
            u8Verify    = bitrev8((data->u64DataBits >>  0) & 0xff);
            u32Headcode = ((((u32)u8Custom_2) << 16) | (((u32)u8Custom_1) << 8)) | u8Custom_3;

            IRDBG_INFO("[PANASONIC] u32Headcode = %x u8Custom_K = %x u8Data = %x u8Verify = %x\n",u32Headcode,u8Custom_K,u8Data,u8Verify);
            for(i= 0;i<dev->support_num;i++)
            {
                if(dev->support_ir[i].eIRType == IR_TYPE_PANASONIC)
                {
                    if((u32Headcode == dev->support_ir[i].u32HeadCode) && (dev->support_ir[i].u8Enable == TRUE))
                    {
                        if((u8Data ^ (u8Custom_K + 0x80)) == u8Verify)
                        {
#ifdef CONFIG_MIRC_INPUT_DEVICE
                            sc->scancode = (u8Custom_K << 8) | u8Data;
                            sc->scancode_protocol = (1<<IR_TYPE_PANASONIC);
                            speed = dev->support_ir[i].u32IRSpeed;
                            dev->map_num = dev->support_ir[i].u32HeadCode;
#else
                            sc->scancode = (u8Custom_K << 16) | (u8Data << 8) | 0x00;
                            sc->scancode |= (0x01UL << 28);
                            sc->scancode_protocol = (1<<IR_TYPE_PANASONIC);
                            speed = dev->support_ir[i].u32IRSpeed;
#endif
                            KeyTime_repeat = MIRC_Get_System_Time() - KeyTime;
                            IRDBG_INFO("KeyTime_repeat = %lu\n",KeyTime_repeat);
                            KeyTime = MIRC_Get_System_Time();
                            data->eStatus = STATE_INACTIVE;
                            if(prevsc->scancode_protocol == (1<<IR_TYPE_PANASONIC) && ((u32)KeyTime_repeat <= PANASONIC_REPEAT_TINEOUT))
                            {
                                if(((speed != 0)&&( data->u8RepeatTimes >= (speed - 1))) || ((speed == 0)&&(data->u8RepeatTimes >= dev->speed)))
                                {
#ifdef CONFIG_MIRC_INPUT_DEVICE
                                    dev->raw->u8RepeatFlag = 1;
#else
                                    sc->scancode = scancode|0x01;//repeat
#endif
                                    return 1;
                                }
                                data->u8RepeatTimes ++;
                                IRDBG_INFO("[PANASONIC] repeattimes =%d \n",data->u8RepeatTimes);
                                return 0;
                            }
                            dev->raw->u8RepeatFlag = 0;
                            data->u8RepeatTimes = 0;
                            return 1;
                        }
                    }
                }
            }
        }
        else
        {
            data->eStatus = STATE_BIT_PULSE;
        }

        return 0;
    default:
        break;
    }

    data->eStatus = STATE_INACTIVE;
    return -EINVAL;
}

static struct ir_decoder_handler panasonic_handler = {
    .protocols  = (1<<IR_TYPE_PANASONIC),
    .decode     = ir_panasonic_decode,
};

int panasonic_decode_init(void)
{
    if(u8InitFlag_panasonic == FALSE)
    {
        KeyTime = 0;
        memset(&panasonic,0,sizeof(IR_PANASONIC_Spec_t));
        MIRC_Decoder_Register(&panasonic_handler);
        IR_PRINT("[IR Log] PANASONIC Spec Protocol Init\n");
        u8InitFlag_panasonic = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] PANASONIC Spec Protocol Has been Initialized\n");
    }
    return 0;
}

void panasonic_decode_exit(void)
{
    if(u8InitFlag_panasonic == TRUE)
    {
        MIRC_Decoder_UnRegister(&panasonic_handler);
        u8InitFlag_panasonic = FALSE;
    }
}
