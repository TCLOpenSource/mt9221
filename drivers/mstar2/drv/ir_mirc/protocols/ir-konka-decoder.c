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

/* ir-konka-decoder.c - handle KONKA IR Pulse/Space protocol
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

static u8 u8InitFlag_konka = FALSE;
static u32 speed = 0;
static unsigned long  KeyTime = 0;
static IR_KONKA_Spec_t konka;

/**
 * ir_konka_decode() - Decode one KONKA pulse or space
 * @dev:    the struct rc_dev descriptor of the device
 * @duration:    the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_konka_decode(struct mstar_ir_dev *dev, struct ir_raw_data ev)
{
    IR_KONKA_Spec_t *data = &konka;
    struct ir_scancode *sc = &dev->raw->this_sc;
    struct ir_scancode *prevsc = &dev->raw->prev_sc;
    u8 i = 0;
    u8 u8Cuscode = 0;
    u8 u8Datacode = 0;

    if (!(dev->enabled_protocols & (1<<IR_TYPE_KONKA)))
        return -EINVAL;
    switch (data->eStatus)
    {

    case STATE_INACTIVE:
        if (!ev.pulse)
            break;
        if (eq_margin(ev.duration,KONKA_HEADER_PULSE_LWB,KONKA_HEADER_PULSE_UPB))
        {
            data->u8BitCount = 0;
            data->eStatus = STATE_HEADER_SPACE;
            //IRDBG_INFO("KONKA_HEADER_PULSE\n");
            return 0;
        }
        else
            break;

    case STATE_HEADER_SPACE:
        if (ev.pulse)
            break;
        if (eq_margin(ev.duration,KONKA_HEADER_SPACE_LWB,KONKA_HEADER_SPACE_UPB))
        {
            data->eStatus = STATE_BIT_PULSE;
            return 0;
        }
        break;

    case STATE_BIT_PULSE:
        if (!ev.pulse)
            break;
        if (!eq_margin(ev.duration,KONKA_BIT_PULSE_LWB,KONKA_BIT_PULSE_UPB))
            break;

        if(data->u8BitCount == KONKA_NBITS)
            data->eStatus = STATE_BIT_TRAILER;
        else
            data->eStatus = STATE_BIT_SPACE;
        return 0;

    case STATE_BIT_SPACE:
        if (ev.pulse)
            break;
        data->u16DataBits <<= 1;

        if (eq_margin(ev.duration,KONKA_BIT_1_SPACE_LWB,KONKA_BIT_1_SPACE_UPB))
            data->u16DataBits |= 1;
        else if (!eq_margin(ev.duration,KONKA_BIT_0_SPACE_LWB,KONKA_BIT_0_SPACE_UPB))
            break;
        data->u8BitCount++;
        data->eStatus = STATE_BIT_PULSE;
        return 0;

    case STATE_BIT_TRAILER:
        if (ev.pulse)
            break;
        if (!eq_margin(ev.duration,KONKA_BIT_TRAILER_LWB,KONKA_BIT_TRAILER_UPB))
            break;

        u8Cuscode  = (data->u16DataBits >> 8) & 0xff;
        u8Datacode = (data->u16DataBits >> 0) & 0xff;

        IRDBG_INFO("[KONKA] u8Cuscode = %x u8Datacode = %x\n",u8Cuscode,u8Datacode);
        for (i= 0;i<dev->support_num;i++)
        {
            if((dev->support_ir[i].eIRType == IR_TYPE_KONKA) &&(dev->support_ir[i].u8Enable == TRUE))
            {
                if(u8Cuscode == dev->support_ir[i].u32HeadCode)
                {
#ifdef CONFIG_MIRC_INPUT_DEVICE
                    sc->scancode = (u8Cuscode << 8) |  u8Datacode;
                    sc->scancode_protocol = (1<<IR_TYPE_KONKA);
                    speed = dev->support_ir[i].u32IRSpeed;
                    dev->map_num = dev->support_ir[i].u32HeadCode;
                    dev->raw->u8RepeatFlag = 0;
#else
                    sc->scancode = (u8Datacode << 8) | 0x00;
                    sc->scancode |= (0x01UL << 28);
                    sc->scancode_protocol = (1<<IR_TYPE_KONKA);
                    speed = dev->support_ir[i].u32IRSpeed;
#endif
                    if(prevsc->scancode_protocol == (1<<IR_TYPE_KONKA) && ((u32)(MIRC_Get_System_Time()- KeyTime) <= KONKA_REPEAT_TIMEOUT))
                    {
                        if(((speed != 0)&&( data->u8RepeatTimes >= (speed - 1))) || ((speed == 0)&&(data->u8RepeatTimes >= dev->speed)))
                        {
#ifdef CONFIG_MIRC_INPUT_DEVICE
                            dev->raw->u8RepeatFlag = 1;
#else
                            sc->scancode |= 0x01;//repeat
#endif
                        }
                        data->u8RepeatTimes ++;
                    }
                    KeyTime = MIRC_Get_System_Time();
                    memset(data,0,sizeof(IR_KONKA_Spec_t));
                    return 1;
                }
            }
        }
        return 0;
    default:
        break;
    }

    data->eStatus = STATE_INACTIVE;
    return -EINVAL;
}

static struct ir_decoder_handler konka_handler = {
    .protocols    = (1<<IR_TYPE_KONKA),
    .decode        = ir_konka_decode,
};

int konka_decode_init(void)
{
    if(u8InitFlag_konka == FALSE)
    {
        KeyTime = 0;
        memset(&konka,0,sizeof(IR_KONKA_Spec_t));
        MIRC_Decoder_Register(&konka_handler);
        IR_PRINT("[IR Log] KONKA Spec Protocol Init\n");
        u8InitFlag_konka = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] KONKA Spec Protocol Has been Initialized\n");
    }
    return 0;
}

void konka_decode_exit(void)
{
    if(u8InitFlag_konka == TRUE)
    {
        MIRC_Decoder_UnRegister(&konka_handler);
        u8InitFlag_konka = FALSE;
    }
}
