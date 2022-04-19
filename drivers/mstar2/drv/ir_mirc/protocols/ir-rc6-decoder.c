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

/* ir-tcl-rc6-decoder.c - handle RCA IR Pulse/Space protocol
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

static u8 u8InitFlag_rc6 = FALSE;
static IR_RC6_Spec_t rc6;
static unsigned long  KeyTime = 0;
static u32 u32preData = 0;

static u32 databits_to_data(u64 data_bits,u8 bits)
{
    u32 ret_data=0;
    u8 i = 0;
    u8 number = bits>>1;
    for(i=1;i<number+1;i++)
    {
        ret_data <<= 1;
        if(((data_bits>>(bits-i*2))&0x03)==1)
            ret_data |=0x01;
    }
    if(bits%2)
    {
        ret_data <<= 1;
        if((data_bits&0x01) == 0)
            ret_data |=0x01;
    }
    return ret_data;
}

/**
 * ir_rc6_decode() - Decode one RCA pulse or space
 * @dev:	the struct rc_dev descriptor of the device
 * @duration:	the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_rc6_decode(struct mstar_ir_dev*dev, struct ir_raw_data ev)
{
    IR_RC6_Spec_t *data = &rc6;
    struct ir_scancode *sc = &dev->raw->this_sc;
    u8 i = 0;
    u16 u16Addr = 0;
    u8 u8Cmd = 0;
    u32 u32Data = 0;
    u8 u8Toggle= 0;
    u8 u8RepeatTimes= 0;

    if (!(dev->enabled_protocols & (1<<IR_TYPE_RC6)))
        return -EINVAL;

    switch (data->eStatus)
    {

    case STATE_INACTIVE:
        if (!ev.pulse)
            break;
        if (eq_margin(ev.duration, RC6_HEADER_PULSE_LWB, RC6_HEADER_PULSE_UPB))
        {
            data->u8BitCount = 0;
            data->eStatus = STATE_HEADER_SPACE;
            return 0;
        }
        else
            break;

    case STATE_HEADER_SPACE:
        if (ev.pulse)
            break;
        if (eq_margin(ev.duration, RC6_HEADER_SPACE_LWB, RC6_HEADER_SPACE_UPB))
        {
            u8RepeatTimes = data->u8RepeatTimes;
            memset(data,0,sizeof(IR_RC6_Spec_t));
            data->u8RepeatTimes = u8RepeatTimes;
            data->eStatus = STATE_BIT_DATA;
            return 0;
        }
        break;

    case STATE_BIT_DATA:
        if (eq_margin(ev.duration, RC6_BIT_MIN_LWB, RC6_BIT_MIN_UPB))
        {
            data->u8BitCount ++;
            data->u64DataBits <<= 1;
            if(!ev.pulse)
            {
                data->u64DataBits |= 0x01;
            }
        }
        else if(eq_margin(ev.duration, RC6_BIT_MAX_LWB, RC6_BIT_MAX_UPB))
        {
            data->u8BitCount += 2;
            data->u64DataBits <<= 2;
            if(!ev.pulse)
            {
                data->u64DataBits |= 0x03;
            }
        }
        else if(eq_margin(ev.duration, RC6_TRAILER_MAX_LOB, RC6_TRAILER_MAX_UPB))
        {
            data->u8BitCount += 3;
            data->u64DataBits <<= 3;
            if(!ev.pulse)
            {
                data->u64DataBits |= 0x07;
            }
        }
        else
        {
            IRDBG_INFO("[RC6] Error Bit break Shotcount =%d !!!\n",ev.duration);
            break;
        }

        if(!data->u8DataFlag)
        {
            if(data->u8BitCount >= TRAILER_BITS)
            {
                data->u8RC6Mode = databits_to_data(((data->u64DataBits)>>(data->u8BitCount - MODE_BITS)),MODE_BITS);
                data->u8DataFlag =1;
                data->u8BitCount= data->u8BitCount - TRAILER_BITS;
                data->u64DataBits &= ~((~0)<<(data->u8BitCount));
            }
        }
        else
        {
            //data handle
            if((data->u8RC6Mode&0x07) == 0)//MODE_0  command + keycode
            {
                if(data->u8BitCount >= (RC6_MODE0_NBITS*2 -1))//Need 16 bit data
                {
                    u32Data = databits_to_data(data->u64DataBits,data->u8BitCount);
                    data->eStatus = STATE_INACTIVE;
                    IRDBG_INFO("[RC6] u8RC6Mode: MODE_0\n");
                    IRDBG_INFO("[RC6] u32Data =%x \n", u32Data);

                    u16Addr  = (u32Data  >> 8) & 0xFF;
                    u8Cmd    = u32Data & 0xFF;
                    goto ret_ok;
                }
            }
            else if((data->u8RC6Mode&0x07) == 6)
            {
                if(data->u8BitCount >= (RC6_MODE6A_32_NBITS*2 -1)) //Need 32bit data //MODE_6A 2bytes command +2bytes keycode [key value=last byte]
                {
                    u32Data = databits_to_data(data->u64DataBits,data->u8BitCount);
                    data->eStatus = STATE_INACTIVE;
                    IRDBG_INFO("[RC6] u8RC6Mode: MODE_6A\n");
                    IRDBG_INFO("[RC6] u32Data =%x \n", u32Data);

                    u16Addr  = u32Data  >> 16;
                    u8Toggle = (u32Data & 0x8000) ? 1 : 0;
                    u8Cmd    = u32Data & 0xFF;
                    goto ret_ok;
                }
            }
            else if((data->u8RC6Mode&0x07) == 1)
            {
                IRDBG_ERR("[RC6] u8RC6Mode =%x Decoder Not Support\n", data->u8RC6Mode);
                return -EINVAL;
            }
            else
            {
                //for RC6 else mode [unrealized]
                IRDBG_ERR("[RC6] u8RC6Mode =%x Decoder Not Support\n", data->u8RC6Mode);
                data->eStatus  = STATE_INACTIVE;
                return -EINVAL;
            }
            return -EINVAL;
ret_ok:
            for (i= 0;i<dev->support_num;i++)
            {
                if((u16Addr== dev->support_ir[i].u32HeadCode)&&((dev->support_ir[i].eIRType == IR_TYPE_RC6_MODE0)||(dev->support_ir[i].eIRType == IR_TYPE_RC6))&&(dev->support_ir[i].u8Enable == TRUE))
                {
                    u32 speed = dev->support_ir[i].u32IRSpeed;
                    if (((speed != 0)&&( data->u8RepeatTimes >= (speed - 1)))
                            || ((speed == 0)&&(data->u8RepeatTimes >= dev->speed)))
                    {
                            if((u32)(MIRC_Get_System_Time()- KeyTime) <= 130)
                            {
                                if(u32Data == u32preData)
                                {
                                    dev->raw->u8RepeatFlag = 1;
                                }
                            }
                            KeyTime = MIRC_Get_System_Time();
                            u32preData = u32Data;
#ifdef CONFIG_MIRC_INPUT_DEVICE
                            sc->scancode = (u16Addr<<8)| u8Cmd;
                            sc->scancode_protocol = (1<<IR_TYPE_RC6);
                            dev->map_num = dev->support_ir[i].u32HeadCode;
#else
                            sc->scancode = (u8Cmd<<8) |0x00;
                            sc->scancode |= (0x01UL << 28);
                            sc->scancode_protocol = (1<<IR_TYPE_RC6);
#endif
                            memset(data,0,sizeof(IR_RC6_Spec_t));
                            return 1;
                    }
                }
            }
            u8RepeatTimes = ++data->u8RepeatTimes;
            IRDBG_INFO("[RC6] repeattimes =%d \n",data->u8RepeatTimes);
            memset(data,0,sizeof(IR_RC6_Spec_t));
            data->u8RepeatTimes = u8RepeatTimes;
        }
        return -EINVAL;
    default:
        break;
    }

    data->eStatus = STATE_INACTIVE;
    return -EINVAL;
}

static struct ir_decoder_handler rc6_handler = {
    .protocols = (1<<IR_TYPE_RC6),
    .decode    = ir_rc6_decode,
};

int rc6_decode_init(void)
{
    if(u8InitFlag_rc6 == FALSE)
    {
        memset(&rc6,0,sizeof(IR_RC6_Spec_t));
        MIRC_Decoder_Register(&rc6_handler);
        IR_PRINT("[IR Log] RC6 Spec protocol init\n");
        u8InitFlag_rc6 = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] RC6 Spec Protocol Has been Initialized\n");

    }
    return 0;
}

void rc6_decode_exit(void)
{
    if(u8InitFlag_rc6 == TRUE)
    {
        MIRC_Decoder_UnRegister(&rc6_handler);
        u8InitFlag_rc6 = FALSE;
    }
}

