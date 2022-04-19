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

/* ir-tcl-rca-decoder.c - handle RCA IR Pulse/Space protocol
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

static u8 u8InitFlag_rca = FALSE;
static unsigned long  KeyTime;
static IR_RCA_Spec_t rca;
static u8 u8FirstPress;
static u32 u32RcaSpeed;

#define TCL_FACTORY_DRIVER    1
#if (TCL_FACTORY_DRIVER == 1)

#define FactoryCustomerCodeMax  10
#define FACTORY_KEY_TIMEOUT     500  //ms

int factory_key_pretime = 0;
int factory_key_curtime = 0;

extern struct input_dev *irFactoryInputdev;

static unsigned int factory_customer_code_map[FactoryCustomerCodeMax]=
{
    5, 6, 7, 8, 9,  11, 12, 13, 14, 15
};

static int is_factory_customer_code(unsigned int customer_code)
{
    int i = 0;
    for(i = 0; i < FactoryCustomerCodeMax; i++){
        if(factory_customer_code_map[i] == customer_code)
            break;
    }
    if(i >= FactoryCustomerCodeMax){
        return 0;
    }else{
        return 1;
    }
}

static int report_factory_key(struct mstar_ir_dev*dev, int keycode,int value)
{
    input_event(dev->irFactoryInputdev, EV_FACTKEYREADER, keycode, value + 1);
    input_sync(dev->irFactoryInputdev);
    input_event(dev->irFactoryInputdev, EV_FACTKEYREADER, keycode, value);
    input_sync(dev->irFactoryInputdev);
}

#endif

static u8 bitrev_n(u8 value,u8 n)
{
    u8 i = 0;
    u8 ret = 0;
    for(i = 0;i<n;i++)
    {
        ret = ret<<1;
        if((value>>i)&0x01)
        {
            ret |= 1;
        }
    }
    return ret;
}

/**
 * ir_rca_decode() - Decode one RCA pulse or space
 * @dev:    the struct rc_dev descriptor of the device
 * @duration:   the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_rca_decode(struct mstar_ir_dev*dev, struct ir_raw_data ev)
{
    IR_RCA_Spec_t *data = &rca;
    struct ir_scancode *sc = &dev->raw->this_sc;
    u8 i = 0;
    u8 u8Addr = 0;
    u8 u8AddrInv = 0;
    u8 u8Cmd = 0;
    u8 u8CmdInv = 0;
    u8 u8Repeat = 0;
#if (TCL_FACTORY_DRIVER == 1)
    int factory_customer_code = 0;
    static int factory_precustomer_code = 0;
    int factory_value = 0;
    int factory_keycode = 0;
    static int factory_prekeycode = 0;
#endif
    if (!(dev->enabled_protocols & (1<<IR_TYPE_RCA)))
        return -EINVAL;

    switch (data->eStatus)
    {

    case STATE_INACTIVE:
        if (!ev.pulse)
            break;

        if (eq_margin(ev.duration, RCA_HEADER_PULSE_LWB, RCA_HEADER_PULSE_UPB))
        {
            data->u8BitCount = 0;
            data->u32DataBits = 0;
            data->eStatus = STATE_HEADER_SPACE;
            return 0;
        }
        else
            break;

    case STATE_HEADER_SPACE:
        if (ev.pulse)
            break;

        if (eq_margin(ev.duration, RCA_HEADER_SPACE_LWB, RCA_HEADER_SPACE_UPB)) {
            data->eStatus = STATE_BIT_PULSE;
            return 0;
        }
        break;

    case STATE_BIT_PULSE:
        if (!ev.pulse)
            break;

        if (!eq_margin(ev.duration, RCA_BIT_PULSE_LWB,RCA_BIT_PULSE_UPB))
            break;

        data->eStatus = STATE_BIT_SPACE;
        return 0;

    case STATE_BIT_SPACE:
        if (ev.pulse)
            break;
        data->u32DataBits >>= 1;
        if (eq_margin(ev.duration, RCA_BIT_1_SPACE_LWB, RCA_BIT_1_SPACE_UPB))
            data->u32DataBits |= 0x80000000;
        else if (!eq_margin(ev.duration, RCA_BIT_0_SPACE_LWB, RCA_BIT_0_SPACE_UPB))
            break;
        data->u8BitCount++;

        if (data->u8BitCount == RCA_NBITS)
        {
            data->u32DataBits = data->u32DataBits>>8;
            u8AddrInv     = bitrev_n(((data->u32DataBits)&0x0F), 4);
            u8CmdInv      = bitrev_n((((data->u32DataBits)>>4)&0xFF),8);
            u8Addr  = bitrev_n((((data->u32DataBits)>>12)&0x0F),4);
            u8Cmd   = bitrev_n((((data->u32DataBits)>>16)&0xFF),8);
            printk("[RCA] u8Addr = %x u8AddrInv = %x u8Cmd = %x u8CmdInv = %x\n",u8Addr,u8AddrInv,u8Cmd,u8CmdInv);
            IRDBG_INFO("[RCA] TIME =%ld\n",(MIRC_Get_System_Time()-KeyTime));

            KeyTime = MIRC_Get_System_Time();
#ifdef CONFIG_MIRC_INPUT_DEVICE
    #if (TCL_FACTORY_DRIVER == 1)
            factory_customer_code = u8Addr;
            factory_keycode = u8Cmd;

            if(is_factory_customer_code(factory_customer_code)  == 1)//tcl factory key is true
            {
                factory_value = factory_value | (factory_customer_code<< 16);

                if(factory_key_curtime == 0) //first time press factory key
                {
                    //IRDBG_INFO("[RCA:factory] first time press factory key \n");
                    factory_key_curtime = KeyTime;
                    factory_key_pretime = factory_key_curtime;
                    report_factory_key(dev, factory_keycode, factory_value);
                } else {
                    factory_key_curtime = KeyTime;

                    if((factory_key_curtime - factory_key_pretime) < FACTORY_KEY_TIMEOUT) //no timeout
                    {
                        if(factory_precustomer_code == factory_customer_code && factory_prekeycode == factory_keycode){// the same key action
                            //IRDBG_INFO("[RCA:factory] ignore the repeat key action between 500ms\n");
                        } else {
                            report_factory_key(dev, factory_keycode, factory_value);
                            factory_key_pretime = factory_key_curtime;
                        }
                    }else{
                        report_factory_key(dev, factory_keycode,factory_value);
                        factory_key_pretime = factory_key_curtime;
                    }
                }
                factory_precustomer_code = factory_customer_code;
                factory_prekeycode = factory_keycode;

                return 0;
            }
    #endif
#endif

            for (i= 0;i<dev->support_num;i++)
            {
                if(((u8Addr<<8 | u8AddrInv) == dev->support_ir[i].u32HeadCode)&&(dev->support_ir[i].eIRType == IR_TYPE_RCA)&&(dev->support_ir[i].u8Enable == TRUE))
                {
                    if (u8Cmd == (u8)~u8CmdInv)
                    {
                        data->eStatus = STATE_INACTIVE;
                        data->u32DataBits = 0;
                        data->u8BitCount = 0;
                        if((MIRC_Get_System_Time()-KeyTime) <= RCA_REPEAT_TIMEOUT)
                        {
                            KeyTime = MIRC_Get_System_Time();
                            if (((u32RcaSpeed != 0)&&( data->u8RepeatTimes >= (u32RcaSpeed - 1)))
                            || ((u32RcaSpeed == 0)&&(data->u8RepeatTimes >= dev->speed)))
                            {
                                u8Repeat = 1;
                            }
                            else
                            {
                                data->u8RepeatTimes ++;
                                IRDBG_INFO("[RCA] repeattimes =%d \n",data->u8RepeatTimes);
                                return -EINVAL;
                            }
                        }
                        else
                        {
                            u32RcaSpeed = dev->support_ir[i].u32IRSpeed;
                            KeyTime = MIRC_Get_System_Time();
                            u8Repeat = 0;
                            data->u8RepeatTimes = 0;
                        }
                        #ifdef CONFIG_MIRC_INPUT_DEVICE
                        sc->scancode = (u8Addr<<16) | (u8AddrInv<<8) | u8Cmd;
                        sc->scancode_protocol = (1<<IR_TYPE_RCA);
                        dev->map_num = dev->support_ir[i].u32HeadCode;
                        dev->raw->u8RepeatFlag = u8Repeat;
                        #else
                        sc->scancode = (u8Cmd<<8) |(u8Repeat&0x01);
                        sc->scancode |= (0x01UL << 28);
                        sc->scancode_protocol = (1<<IR_TYPE_RCA);
                        #endif
                        return 1;
                    }
                    else
                    {
                       IRDBG_ERR("[RCA] parse keycode = u8Cmd fail\n");
                       memset(data,0,sizeof(IR_RCA_Spec_t));
                       data->eStatus = STATE_INACTIVE;
                       return -EINVAL;
                    }
                }
            }
            break;
        }
        else
            data->eStatus = STATE_BIT_PULSE;
        return 0;
    default:
        break;
    }

    data->eStatus = STATE_INACTIVE;
    return -EINVAL;
}

static struct ir_decoder_handler rca_handler = {
    .protocols  = (1<<IR_TYPE_RCA),
    .decode     = ir_rca_decode,
};

int rca_decode_init(void)
{
    if(u8InitFlag_rca == FALSE)
    {
        memset(&rca,0,sizeof(IR_RCA_Spec_t));
        MIRC_Decoder_Register(&rca_handler);
        IR_PRINT("[IR Log] RCA Spec protocol init\n");
        u8InitFlag_rca = TRUE;
    }
    else
    {
        IR_PRINT("[IR Log] RCA Spec Protocol Has been Initialized\n");

    }
    return 0;
}

void rca_decode_exit(void)
{
    if(u8InitFlag_rca == TRUE)
    {
        MIRC_Decoder_UnRegister(&rca_handler);
        u8InitFlag_rca = FALSE;
    }
}

