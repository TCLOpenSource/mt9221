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

/* ir-tcl-rc5-decoder.c - handle RCA IR Pulse/Space protocol
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

static u8 u8InitFlag_rc5 = FALSE;
static u32 speed = 0;
static u8 pre_toggle;
static unsigned long  KeyTime;
static IR_RC5_Spec_t rc5;

static u32 databits_to_data(u64 data_bits,u8 bits,u8 reverse)
{
    u32 ret_data=0;
    u8 i = 0;
    u8 number = bits>>1;
    for(i=1;i<number+1;i++)
    {
        ret_data <<= 1;
        if(reverse)
        {
            if(((data_bits>>(bits-i*2))&0x03)==1)
                ret_data |=0x01;
        }
        else
        {
            if(((data_bits>>(bits-i*2))&0x03)==2)
                ret_data |=0x01;

        }
    }
    if(bits%2)
    {
        ret_data <<= 1;
        if(reverse)
        {
            if((data_bits&0x01) == 0)
                ret_data |=0x01;
        }
        else
        {
            if(data_bits&0x01)
                ret_data |=0x01;
        }
    }
    return ret_data;
}

/**
 * ir_rc5_decode() - Decode one RCA pulse or space
 * @dev:       the struct rc_dev descriptor of the device
 * @duration:  the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_rc5_decode(struct mstar_ir_dev*dev, struct ir_raw_data ev)
{
    IR_RC5_Spec_t *data = &rc5;
    struct ir_scancode *sc = &dev->raw->this_sc;
    struct ir_scancode *prevsc = &dev->raw->prev_sc;

    u8 i = 0;
    u8 u8Addr = 0;
    u8 u8Toggle =0;
    u8 u8Cmd =0;
    u32 u32Data;


    if (!(dev->enabled_protocols & (1<<IR_TYPE_RC5)))
        return -EINVAL;

    switch (data->eStatus)
    {

    case STATE_INACTIVE:
        if (!ev.pulse)
            break;
        //match RC5 start bit1  1
        if (eq_margin(ev.duration, RC5_BIT_MIN_LWB, RC5_BIT_MIN_UPB))
        {
            data->u8BitCount = 0;
            data->u64DataBits = 0;
            data->eStatus = STATE_BIT_DATA;
            return 0;
        }
        //match RC5 start bit1 with bit2  0_High
        else if(eq_margin(ev.duration, RC5_BIT_MAX_LWB, RC5_BIT_MAX_UPB))
        {
            data->u8BitCount = 1;
            data->u64DataBits = 1;
            data->eStatus = STATE_BIT_DATA;
            return 0;
        }
        else
            break;

    case STATE_BIT_DATA:
        if (eq_margin(ev.duration, RC5_BIT_MIN_LWB, RC5_BIT_MIN_UPB))
        {
            data->u8BitCount ++;
            data->u64DataBits <<= 1;
            if(!ev.pulse)
            {
                data->u64DataBits |= 1;
            }
        }
        else if(eq_margin(ev.duration, RC5_BIT_MAX_LWB, RC5_BIT_MAX_UPB))
        {
            data->u8BitCount += 2;
            data->u64DataBits <<= 2;
            if(!ev.pulse)
            {
                data->u64DataBits |= 3;
            }
        }
        else
            break;
        if(data->u8BitCount >= (RC5_NBITS*2 -3))
        {
            //IRDBG_INFO("[RC5] data->u64DataBits = %llx\n",data->u64DataBits);
            u32Data = databits_to_data(data->u64DataBits,data->u8BitCount,0);
            //IRDBG_INFO("[RC5] u32Data = %x\n",u32Data);

            u8Cmd    = (u32Data & 0x0003F) >> 0;
            u8Addr   = (u32Data & 0x007C0) >> 6;
            u8Toggle = (u32Data & 0x00800) ? 1 : 0;
            u8Cmd   += (u32Data & 0x01000) ? 0 : 0x40;

            IRDBG_INFO("[RC5] u8Addr = %x u8Cmd = %x u8Toggle = %x\n",u8Addr,u8Cmd,u8Toggle);

            for (i= 0;i<dev->support_num;i++)
            {
                if((u8Addr== dev->support_ir[i].u32HeadCode)&&(dev->support_ir[i].eIRType == IR_TYPE_RC5)&&(dev->support_ir[i].u8Enable == TRUE))
                {
#ifdef CONFIG_MIRC_INPUT_DEVICE
                    sc->scancode = (u8Addr<<8)| u8Cmd;
                    sc->scancode_protocol = (1<<IR_TYPE_RC5);
                    speed = dev->support_ir[i].u32IRSpeed;
                    dev->map_num = dev->support_ir[i].u32HeadCode;
#else
                    sc->scancode = (u8Cmd<<8) |0x00;
                    sc->scancode |= (0x01UL << 28);
                    sc->scancode_protocol = (1<<IR_TYPE_RCA);
                    speed = dev->support_ir[i].u32IRSpeed;
#endif
                    IRDBG_INFO("[RC5] TIME =%ld\n",(MIRC_Get_System_Time()-KeyTime));
                    KeyTime = MIRC_Get_System_Time();
                    data->eStatus = STATE_INACTIVE;
                    if((prevsc->scancode_protocol == (1 << IR_TYPE_RC5))
                            && (prevsc->scancode == sc->scancode)
                            && ((MIRC_Get_System_Time()-KeyTime) < 225))
                    {
                        if(pre_toggle == u8Toggle)
                        {
                            if(((speed != 0)&&( data->u8RepeatTimes >= (speed - 1)))
                                    || ((speed == 0)&&(data->u8RepeatTimes >= dev->speed)))
                            {
#ifdef CONFIG_MIRC_INPUT_DEVICE
                                dev->raw->u8RepeatFlag = 1;
#else
                                sc->scancode = scancode|0x01;//repeat
#endif
                                return 1;
                            }
                            data->u8RepeatTimes ++;
                            IRDBG_INFO("[RCA] repeattimes =%d \n",data->u8RepeatTimes);
                            return 0;
                        }
                    }
                    pre_toggle = u8Toggle;
                    dev->raw->u8RepeatFlag = 0;
                    data->u8RepeatTimes = 0;
                    return 1;
                }
            }
        }
        else
        {
            data->eStatus = STATE_BIT_DATA;
        }
        return 0;
    default:
        break;
    }

    data->eStatus = STATE_INACTIVE;
    return -EINVAL;
}

static struct ir_decoder_handler rc5_handler = {
    .protocols  = (1<<IR_TYPE_RC5),
    .decode     = ir_rc5_decode,
};

int rc5_decode_init(void)
{
    if(u8InitFlag_rc5 == FALSE)
    {
        memset(&rc5,0,sizeof(IR_RC5_Spec_t));
        MIRC_Decoder_Register(&rc5_handler);
        IR_PRINT("[IR Log] RC5 Spec protocol init\n");
        u8InitFlag_rc5 = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] RC5 Spec Protocol Has been Initialized\n");

    }
    return 0;
}

void rc5_decode_exit(void)
{
    if(u8InitFlag_rc5 == TRUE)
    {
        MIRC_Decoder_UnRegister(&rc5_handler);
        u8InitFlag_rc5 = FALSE;
    }
}

