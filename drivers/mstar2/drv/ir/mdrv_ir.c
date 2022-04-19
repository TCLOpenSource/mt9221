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
/// file    drvIR.c
/// @brief  IR Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
//#include "MsCommon.h"
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>  //added
#include <linux/timer.h> //added
#include <linux/types.h> //added
#include <asm/io.h>
#include <linux/semaphore.h>
#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
#ifdef CONFIG_ARM
#include <linux/version.h>
#elif CONFIG_ARM64
#include <generated/uapi/linux/version.h>
#else
#include <version.h>
#endif
#include <linux/mutex.h>
#include <freezer.h>
#include <media/rc-core.h>
#endif
#ifdef CONFIG_MSTAR_IR_POWER_KEY_LONG_PRESS
#include <linux/reboot.h>
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/namei.h>
#endif

#ifdef CONFIG_MSTAR_DYNAMIC_IR
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#include "board/Board.h"
#include "mst_devid.h"
#include "mhal_ir_reg.h"
//#include "mdrv_ir_io.h"
#include "mdrv_ir.h"
#include "chip_int.h"
#include "mdrv_system.h"
#include "../gpio/mdrv_gpio.h"

#if defined(IR_TYPE_RCMM) && (IR_TYPE_SEL == IR_TYPE_RCMM)
#include "IR_RCMM.h"
#elif (IR_TYPE_SEL == IR_TYPE_OLD)
#include "IR_MSTAR_OLD.h"
#elif (IR_TYPE_SEL == IR_TYPE_NEW)
#include "IR_MSTAR_NEW.h"
#elif (IR_TYPE_SEL == IR_TYPE_MSTAR_DTV)
#include "IR_MSTAR_DTV.h"
#elif (IR_TYPE_SEL == IR_TYPE_RC_V16)
#include "IR_MSTAR_RC_V16.h"
#elif (IR_TYPE_SEL == IR_TYPE_MSTAR_RAW)
#include "IR_MSTAR_RAW.h"
#elif (IR_TYPE_SEL == IR_TYPE_CUS03_DTV)
#include "IR_CUS03_DTV.h"
#elif (IR_TYPE_SEL == IR_TYPE_MSTAR_FANTASY)
#include "IR_MSTAR_FANTASY.h"
#elif (IR_TYPE_SEL == IR_TYPE_MSTAR_SZ1)
#include "IR_MSTAR_SZ1.h"
#elif (IR_TYPE_SEL==IR_TYPE_CUS08_RC5)
#include "IR_CUS08_RC5.h"
#elif (IR_TYPE_SEL==IR_TYPE_CUS21SH)
#include "IR_MSTAR_CUS21SH.h"
#elif (IR_TYPE_SEL == IR_TYPE_TOSHIBA)
#include "IR_MSTAR_CUS22T.h"
#elif (defined(IR_TYPE_CHANGHONG) && IR_TYPE_SEL == IR_TYPE_CHANGHONG)
#include "IR_CHANGHONG.h"
#elif (defined(IR_TYPE_HAIER) && IR_TYPE_SEL == IR_TYPE_HAIER)
#include "IR_HAIER.h"
#elif (defined(IR_TYPE_HISENSE) && IR_TYPE_SEL == IR_TYPE_HISENSE)
#include "IR_HISENSE.h"
#elif (IR_TYPE_SEL == IR_TYPE_KONKA)
#include "IR_KONKA.h"
#elif (defined(IR_TYPE_SKYWORTH) && IR_TYPE_SEL == IR_TYPE_SKYWORTH)
#include "IR_SKYWORTH.h"
#elif (defined(IR_TYPE_TCL) && IR_TYPE_SEL==IR_TYPE_TCL)
#include "IR_TCL.h"
#elif (defined(IR_TYPE_HWRC5) && IR_TYPE_SEL==IR_TYPE_HWRC5)
#include "IR_MSTAR_HWRC5.h"
#elif (defined(IR_TYPE_SWRC6) && IR_TYPE_SEL == IR_TYPE_SWRC6)
#include "IR_MSTAR_RC6.h"
#else
#include "IR_MSTAR_DTV.h"
#endif

#define SWIR_FILEOPS 17

#ifdef CONFIG_MSTAR_PM_SWIR
#include "mdrv_mstypes.h"
#include "mdrv_mbx.h"

#include <fcntl.h>
#include <linux/signal.h>
#include <unistd.h>
#endif


#ifdef CONFIG_IR_SUPPLY_RNG
#include <linux/input.h>
#include <random.h>
#include "mhal_rng_reg.h"
#endif

#ifndef INT_NUM_IR_ALL
#define INT_NUM_IR_ALL (E_FIQ_IR)
#endif

#include "IR_PROTOCOL_INSTALL.h"
#ifdef SUPPORT_MULTI_PROTOCOL
#ifdef IR_MODE_SEL
#undef IR_MODE_SEL
#define IR_MODE_SEL IR_TYPE_SWDECODE_MODE
#endif
#ifndef IR_INT_NP_EDGE_TRIG
#define IR_INT_NP_EDGE_TRIG
#endif
#endif

#if ((defined(IR_SWFIFO_MODE) && (IR_SWFIFO_MODE==ENABLE)) && ((IR_MODE_SEL!=IR_TYPE_FULLDECODE_MODE)&& (IR_MODE_SEL!=IR_TYPE_RAWDATA_MODE)))
#define IR_SWFIFO_DECODE_MODE ENABLE
#else
#define IR_SWFIFO_DECODE_MODE DISABLE
#endif

extern IRModHandle IRDev;

#ifdef CONFIG_MSTAR_SOFTWARE_IR_MODULE
extern int take_over_by_software_ir(unsigned char, unsigned char, unsigned char, struct input_dev *, struct ir_input_state *);
extern int software_ir_enable();
extern int take_over_by_software_ir_dfb(unsigned char, unsigned char);
extern int software_ir_processing_undone();
extern int set_software_ir_processing_undone();
#endif

static irqreturn_t  MDrv_IR_ISRParseKey(void);

#ifdef CONFIG_MSTAR_DYNAMIC_IR
static int ir_proc_debug_open(struct inode *inode, struct file *filp);
static ssize_t ir_proc_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *pos);
#endif

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
#define REG(addr)                   (*(volatile u32 *)(addr))
//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#ifdef CONFIG_MSTAR_DYNAMIC_IR
static U8 scan2key_map[16][0xFF] = {0xFF};
static U8 debug_flag = 0;
/*
Max support 16 nec rc
*/
static U8 headcode0array[16];
static U8 headcode1array[16];
static U32 headcode_powerkey_combine[16];  // higher 16bits are SCANCODE, lower 16bits are HEADCODE
static U8 g_headcodecount = 0;

#define GETLINESTRING_ERR 0
#define GETLINESTRING_LINEEND 1
#define GETLINESTRING_FILEEND 2

typedef struct {
    char keystring[16];
    int keycode;
}str2key_map_t;

static const str2key_map_t str2key_map[] =
{
    { "KEY_POWER", KEY_POWER },
    { "KEY_MUTE", KEY_MUTE },
    { "KEY_HOME", KEY_HOME },
    { "KEY_ENTER", KEY_ENTER },
    { "KEY_UP", KEY_UP },
    { "KEY_DOWN", KEY_DOWN },
    { "KEY_LEFT", KEY_LEFT },
    { "KEY_RIGHT", KEY_RIGHT },
    { "KEY_BACK", KEY_BACK},
    { "KEY_MENU", KEY_MENU },
    { "KEY_VOLUMEDOWN", KEY_VOLUMEDOWN },
    { "KEY_VOLUMEUP", KEY_VOLUMEUP },
    { "KEY_CHANNELUP", KEY_CHANNELUP},
    { "KEY_CHANNELDOWN", KEY_CHANNELDOWN},
    { "KEY_1", KEY_1 },
    { "KEY_2", KEY_2 },
    { "KEY_3", KEY_3 },
    { "KEY_4", KEY_4 },
    { "KEY_5", KEY_5 },
    { "KEY_6", KEY_6 },
    { "KEY_7", KEY_7 },
    { "KEY_8", KEY_8 },
    { "KEY_9", KEY_9 },
    { "KEY_0", KEY_0 },
    { "KEY_RED", KEY_RED },
    { "KEY_GREEN", KEY_GREEN },
    { "KEY_YELLOW", KEY_YELLOW },
    { "KEY_BLUE", KEY_BLUE },
    { "KEY_BACKSPACE", KEY_BACKSPACE },
    { "KEY_TAB", KEY_TAB },
    { "KEY_Q", KEY_Q },
    { "KEY_W", KEY_W },
    { "KEY_E", KEY_E },
    { "KEY_R", KEY_R },
    { "KEY_T", KEY_T },
    { "KEY_Y", KEY_Y },
    { "KEY_U", KEY_U },
    { "KEY_I", KEY_I },
    { "KEY_O", KEY_O },
    { "KEY_P", KEY_P },
    { "KEY_LEFTBRACE", KEY_LEFTBRACE },
    { "KEY_RIGHTBRACE", KEY_RIGHTBRACE },
    { "KEY_LEFTCTRL", KEY_LEFTCTRL },
    { "KEY_A", KEY_A },
    { "KEY_S", KEY_S },
    { "KEY_D", KEY_D },
    { "KEY_F", KEY_F },
    { "KEY_G", KEY_G },
    { "KEY_H", KEY_H },
    { "KEY_J", KEY_J },
    { "KEY_K", KEY_K },
    { "KEY_L", KEY_L },
    { "KEY_LEFTSHIFT", KEY_LEFTSHIFT },
    { "KEY_Z", KEY_Z },
    { "KEY_X", KEY_X },
    { "KEY_C", KEY_C },
    { "KEY_V", KEY_V },
    { "KEY_B", KEY_B },
    { "KEY_N", KEY_N },
    { "KEY_M", KEY_M },
    { "KEY_DOT", KEY_DOT },
    { "KEY_ESC", KEY_ESC },
    { "KEY_SLASH", KEY_SLASH },
    { "KEY_RIGHTSHIFT", KEY_RIGHTSHIFT },
    { "KEY_LEFTALT", KEY_LEFTALT },
    { "KEY_SPACE", KEY_SPACE },
    { "KEY_CAPSLOCK", KEY_CAPSLOCK },
    { "KEY_F1", KEY_F1 },
    { "KEY_F2", KEY_F2 },
    { "KEY_F3", KEY_F3 },
    { "KEY_F4", KEY_F4 },
    { "KEY_F5", KEY_F5 },
    { "KEY_F6", KEY_F6 },
    { "KEY_F7", KEY_F7 },
    { "KEY_F8", KEY_F8 },
    { "KEY_F9", KEY_F9 },
    { "KEY_F10", KEY_F10 },
    { "KEY_NUMLOCK", KEY_NUMLOCK },
    { "KEY_SCROLLLOCK", KEY_SCROLLLOCK },
    { "KEY_KP7", KEY_KP7 },
    { "KEY_KP8", KEY_KP8 },
    { "KEY_KP9", KEY_KP9 },
    { "KEY_KPMINUS", KEY_KPMINUS },
    { "KEY_KP4", KEY_KP4 },
    { "KEY_KP5", KEY_KP5 },
    { "KEY_KP6", KEY_KP6 },
    { "KEY_KPPLUS", KEY_KPPLUS },
    { "KEY_KP1", KEY_KP1 },
    { "KEY_KP2", KEY_KP2 },
    { "KEY_KP3", KEY_KP3 },
    { "KEY_KP0", KEY_KP0 },
    { "KEY_KPDOT", KEY_KPDOT },
    { "KEY_F11", KEY_F11 },
    { "KEY_F12", KEY_F12 },
    { "KEY_KPENTER", KEY_KPENTER },
    { "KEY_RIGHTCTRL", KEY_RIGHTCTRL },
    { "KEY_KPSLASH", KEY_KPSLASH },
    { "KEY_SYSRQ", KEY_SYSRQ },
    { "KEY_LINEFEED", KEY_LINEFEED },
    { "KEY_PAGEUP", KEY_PAGEUP },
    { "KEY_END", KEY_END },
    { "KEY_PAGEDOWN", KEY_PAGEDOWN },
    { "KEY_INSERT", KEY_INSERT },
    { "KEY_DELETE", KEY_DELETE },
    { "KEY_KPEQUAL", KEY_KPEQUAL },
    { "KEY_KPPLUSMINUS", KEY_KPPLUSMINUS },
    { "KEY_PAUSE", KEY_PAUSE },
    { "KEY_SCALE", KEY_SCALE },
    { "KEY_STOP", KEY_STOP },
    { "KEY_CALC", KEY_CALC },
    { "KEY_SETUP", KEY_SETUP },
    { "KEY_SLEEP", KEY_SLEEP },
    { "KEY_WAKEUP", KEY_WAKEUP },
    { "KEY_F13", KEY_F13 },    //183
    { "KEY_F14", KEY_F14 },    //184
    { "KEY_F15", KEY_F15 },    //185
    { "KEY_F16", KEY_F16 },    //186
    { "KEY_F17", KEY_F17 },    //187
    { "KEY_F18", KEY_F18 },    //188
    { "KEY_F19", KEY_F19 },    //189
    { "KEY_F20", KEY_F20 },    //190
    { "KEY_F21", KEY_F21 },    //191
    { "KEY_F22", KEY_F22 },    //192
    { "KEY_F23", KEY_F23 },    //193
    { "KEY_F24", KEY_F24 },    //194
    { "KEY_RESERVED", KEY_RESERVED },
};

static const struct file_operations multi_ir_proc_fops = {
    .open = ir_proc_debug_open,
    //.read = ?;
    .write = ir_proc_debug_write,
};

static int ir_proc_debug_show(struct seq_file *seq, void *ptr)
{
    seq_puts(seq, debug_flag?"1":"0");
    return 0;
}

static ssize_t ir_proc_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *pos)
{
    if(debug_flag) printk(KERN_DEBUG "[IR] ir_proc_debug_write ENTRY\n");
    char mode;
    if(count>0){
        if(get_user(mode, buff))
            return -EFAULT;
        debug_flag = mode == '0' ? 0 : 1;
    }

    printk(KERN_DEBUG "[IR] debug_flag = %d\n", debug_flag);

    return count;
}


static int ir_proc_debug_open(struct inode *inode, struct file *filp)
{
    if(debug_flag) printk(KERN_DEBUG "[IR] ir_proc_debug_open ENTRY\n");
    return single_open(filp, ir_proc_debug_show, NULL);
}

/*
return 0xFF means error.
*/
static U8 char_to_u8(char c)
{
    int bitu8;
    char bitchar = c;
    if (bitchar >= '0' && bitchar <= '9')
    {
        bitu8 = bitchar - '0';
    }
    else if (bitchar >= 'A' && bitchar <= 'F')
    {
        bitu8 = 10 + (bitchar - 'A'); // cast to HEX
    }
    else if (bitchar >= 'a' && bitchar <= 'f')
    {
        bitu8 = 10 + (bitchar - 'a');
    }
    else
    {
        printk(KERN_ERR "[IR] %s: param error\n", __FUNCTION__);
        bitu8 = 0xFF;
    }
    return bitu8;
}

static int get_line(struct file *file, int *ptotalcount, char buf[256], int *pbufcount)
{
    int i;
    memset(buf, 0, 256);
    for (i = 0; i < 256; i++)
    {
        if (kernel_read(file, *ptotalcount, buf + i, 1) < 0) {
            return GETLINESTRING_ERR;
        }
        *ptotalcount = *ptotalcount + 1;

        if(*(buf + i) == ' ') // ignore all space
        {
            i--;
            continue;
        }

        if (*(buf + i) == '\n'/* || *(buf + i) == '\r'*/) //Line end.
        {
            if(*(buf + i -1) == '\r')
            {
                i--;
            }
            *pbufcount = i;
            /*
            Set the last byte to 0, so that we can print it use %s.
            */
            *(buf + i) = 0;

            //printk("%s : %s\n", __FUNCTION__, buf);
            if(likely('#' != buf[0]))
            {
                if(debug_flag) printk(KERN_DEBUG "[IR] readline : %s\n", buf);
                return GETLINESTRING_LINEEND;
            }
            else
            {   // Drop this line if it start with '#', and read next line continue
                *pbufcount = 0;
                i=-1;   // i++ will execute in the loop, so set it to -1
                memset(buf, 0, 256);
                continue;
            }
        }else if (*(buf + i) == 0)
        {
            /*
            File end.
            */
            *pbufcount = i;
            return GETLINESTRING_FILEEND;
        }
    }
    printk(KERN_ERR "[IR] GETLINESTRING_ERR : This line is too long!!!\n");
    return GETLINESTRING_ERR;
}

u32 getPowerKeyAndHeadCode(u32 index)
{
#if (IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
    //printk("FULLDECODE, use this combination : 0x%x\n", headcode_powerkey_combine[0]);
    return headcode_powerkey_combine[0];
#else
    if(index >= 16)
    {
        printk(KERN_ERR "[IR] getPowerKeyAndHeadCode : index is larger than 16");
        return 0xDEADBEAF;
    }
    return headcode_powerkey_combine[index];
#endif
}

static int parse_config_section(struct file* filp, unsigned int *total_count)
{
    int get_line_ret = 0;
    U8 buffer[256] = {0};
    U32 char_count = 0;
    U8 ir_index = 0;

    if(debug_flag){printk(KERN_DEBUG "[IR] %s: ENTRY\n", __FUNCTION__);}

    do{
        get_line_ret = get_line(filp, total_count, buffer, &char_count);
        if (unlikely(get_line_ret == GETLINESTRING_ERR)) {
                printk(KERN_ERR "[IR] %s: read error!\n", __FUNCTION__);
                return -1;
        }else if(unlikely(get_line_ret == GETLINESTRING_FILEEND))
        {
            printk(KERN_ERR "[IR] Empty config file, please check it\n");
            return -2;
        }

        if(0 == memcmp(buffer,"headcodebegin",strlen("headcodebegin")))
        {
            if(debug_flag){printk(KERN_DEBUG "[IR] %s: headcode begin\n", __FUNCTION__);}
            break;
        }
    }while(1);

    do{
        get_line_ret = get_line(filp, total_count, buffer, &char_count);
        if (unlikely(get_line_ret == GETLINESTRING_ERR))
        {       printk(KERN_ERR "[IR] file read line error!\n");
                return -1;
        }else if(unlikely(get_line_ret == GETLINESTRING_FILEEND))
        {
            if(debug_flag){printk("[IR] Read complete.\n");}
            break;
        }

        if(0 == memcmp(buffer,"headcodeend",strlen("headcodeend")))
        {
            if(debug_flag){printk(KERN_DEBUG "[IR] %s: headcode end\n", __FUNCTION__);}
            break;
        }

        if(unlikely(char_count!=8))
        {
            printk(KERN_ERR "[IR] %s: char_count != 8, ERROR!\n", __FUNCTION__);
            //return -3;
            continue;
        }
        ir_index = char_to_u8(buffer[3]);  // '0' 'x '0' '?'

        if(unlikely(0xFF == ir_index))
        {
            if(debug_flag){printk(KERN_DEBUG "[IR] %s: ir_index ERROR\n", __FUNCTION__);}
            continue;
        }

        headcode0array[ir_index] = (char_to_u8(buffer[4])<<4) + char_to_u8(buffer[5]);
        headcode1array[ir_index] = (char_to_u8(buffer[6])<<4) + char_to_u8(buffer[7]);

        if(debug_flag){printk(KERN_DEBUG "[IR] %s: headcode0array[%d] = 0x%x\n", __FUNCTION__, ir_index, headcode0array[ir_index]);}
        if(debug_flag){printk(KERN_DEBUG "[IR] %s: headcode1array[%d] = 0x%x\n", __FUNCTION__, ir_index, headcode1array[ir_index]);}

        g_headcodecount++;
    }while(1);

    REG(REG_IR_CCODE) = ((u16)headcode0array[0]<<8) | headcode1array[0];
    _u8IRHeaderCode0 = headcode0array[0];
    _u8IRHeaderCode1 = headcode1array[0];

    return 0;
}

static U8 string2keycode(char *str)
{
    // str2key_map
    unsigned char map_size;
    unsigned char loop_count=0;
    unsigned char str_length =0;
    char *tmp_str = NULL;
    map_size = sizeof(str2key_map)/sizeof(str2key_map_t);

    if(NULL == str)
    {
        printk("[IR] %s: null pointer\n", __FUNCTION__);
        return 0xFF;
    }

    tmp_str = str;
    while(1)
    {
        if('}' == *tmp_str)
            break;
        ++str_length;
        ++tmp_str;
    }

    if(unlikely(str_length == 0))
    {
        printk(KERN_ERR "[IR] %s: Key string error\n", __FUNCTION__);
        return 0xFF;
    }

    for(loop_count=0; loop_count<map_size; loop_count++)
    {
        if(!strncmp(str, str2key_map[loop_count].keystring, str_length))
        {
            return str2key_map[loop_count].keycode;
        }
    }
    printk("[IR] %s: Can't match key for %s\n", __FUNCTION__, str);
    return 0xFF;
}

static int parse_key_section(struct file* filp, unsigned int *total_count)
{
    int get_line_ret = 0;
    unsigned int char_count = 0;
    unsigned char scan_code, key_code;
    unsigned char ir_index = 0;
    unsigned char *key_string;
    unsigned char buffer[256] = {0};

    memset(scan2key_map, 0xF, sizeof(scan2key_map));

    do{
        get_line_ret = get_line(filp, total_count, buffer, &char_count);
        if (unlikely(get_line_ret == GETLINESTRING_ERR))
        {       printk("[IR] %s: file read line fail\n", __FUNCTION__);
                return -1;
        }else if(unlikely(get_line_ret == GETLINESTRING_FILEEND))
        {
            if(debug_flag) printk("[IR] Read complete.\n");
            break;
        }

        ir_index = char_to_u8(buffer[4]); // '{', '0', 'x', '0', '?'
        if(unlikely(ir_index == 0xFF || g_headcodecount<ir_index))
        {
            if(debug_flag) printk("[IR] ir_index is wrong! (0x%x)\n", ir_index);
            continue;
        }

        U8 scancode_high = char_to_u8(buffer[5]);
        U8 scancode_low = char_to_u8(buffer[6]);
        if(unlikely(scancode_high == 0xFF || scancode_low == 0xFF))
        {
            printk(KERN_ERR "[IR] read scancode error\n");
            continue;
        }
        //scan_code = (unsigned char)((char_to_u8(buffer[5])<<4)&0xFF);
        //scan_code += char_to_u8(buffer[6]);
        scan_code = ((scancode_high<<4)&0xF0) | (scancode_low&0x0F);
        key_string = &(buffer[8]);
        //printk("key_string = %s\n", key_string);
        key_code = string2keycode(key_string);
        if(unlikely(key_code == 0xFF))
        {
            printk(KERN_ERR "[IR] %s: string2keycode error, key_code=%d\n", __FUNCTION__, key_code);
            continue;
        }
        if(debug_flag) printk("[IR] [index=%d]scan_code(0x%x) -> key_code(0x%x)\n", ir_index, scan_code, key_code);
        if(unlikely(key_code == KEY_POWER))
        {
            unsigned short current_headcode = ((u16)headcode0array[ir_index]<<8) | headcode1array[ir_index];
            headcode_powerkey_combine[ir_index] = ((scan_code&0xFFFF)<<16) + (current_headcode&0xFFFF);
            if(debug_flag) printk("headcode_powerkey_combine[%d] = 0x%x\n", ir_index, headcode_powerkey_combine[ir_index]);
        }

        //scan2key_map[ir_index][scan_code] = (ir_index<<8) + key_code;
        scan2key_map[ir_index][scan_code] = key_code;
    }while(1);

    return 0;
}

static void proc_init()
{
    struct proc_dir_entry *proc_entry=NULL;
    struct proc_dir_entry *debug_file = NULL;
    proc_entry = proc_mkdir("ir", NULL);
    if(unlikely(NULL == proc_entry))
    {
        printk(KERN_ERR "[IR] %s: create proc directory error!\n", __FUNCTION__);
        return;
    }

    debug_file = proc_create("multi_ir_debug", 0x644, proc_entry, &multi_ir_proc_fops);
    if(unlikely(NULL == debug_file))
    {
        printk(KERN_ERR "[IR] %s: create debug file error!\n", __FUNCTION__);
        return;
    }
}

static void dynamic_ir_mode_init(void)
{
    struct file *filp;
    unsigned int total_count = 0;
    /*
    ir config begin
    */

    proc_init();

    filp = filp_open("/system/etc/irkey.cfg", O_RDONLY, 0);
    if (IS_ERR(filp)) {
        printk(KERN_ERR "[IR] %s: file open ir config file fail\n", __FUNCTION__);
        return;
    }
    // printk("open irkey.cfg success\n");

    /*
    * Add by Shaoqing.Mei
    */
    if(parse_config_section(filp, &total_count))
    {// error
        printk(KERN_ERR "[IR] parse_config_section error, skip key section\n");
    }else{
        if(unlikely(g_headcodecount == 0))
        {
            printk(KERN_ERR "[IR] No headcode has been setup correct! please check the config file\n");
        }else{
            parse_key_section(filp, &total_count);
        }
    }

    filp_close(filp, NULL);
}

/*
Ir dynamic mode part end.
*/
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static U8 IR_bRepKey;
static U8 IR_KeyValue;
static U8 IR_HeaderCode1 = IR_HEADER_CODE0;
static U8 IR_HeaderCode2 = IR_HEADER_CODE1;
static U8 IR_TimeoutCycle;
static U8 IR_EventTimeout;

static U32 Shotcnt[32];
#endif

static U8 _u8IRHeaderCode0 = IR_HEADER_CODE0;
static U8 _u8IRHeaderCode1 = IR_HEADER_CODE1;
static U8 ir_irq_depth;


#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
#ifndef IR2_HEADER_CODE0
#define IR2_HEADER_CODE0 0xffUL
#endif
#ifndef IR2_HEADER_CODE1
#define IR2_HEADER_CODE1 0xffUL
#endif
static U8 _u8IR2HeaderCode0 = IR2_HEADER_CODE0;
static U8 _u8IR2HeaderCode1 = IR2_HEADER_CODE1;
#endif

#if defined(IR_TYPE_HISENSE) && (IR_TYPE_SEL == IR_TYPE_HISENSE)
static U8 _u8FactoryIRHeaderCode0 = IR_FHEADER_CODE0;
static U8 _u8FactoryIRHeaderCode1 = IR_FHEADER_CODE1;
static U8 _u8PCCommandHeaderCode0 = IR_PHEADER_CODE0;
static U8 _u8PCCommandHeaderCode1 = IR_PHEADER_CODE1;
#elif defined(IR_TYPE_HAIER) && (IR_TYPE_SEL == IR_TYPE_HAIER)
static U8 _u8FactoryIRHeaderCode0 = IR_HEADER_CODE3;
static U8 _u8FactoryIRHeaderCode1 = IR_HEADER_CODE4;
#elif (IR_TYPE_SEL == IR_TYPE_TOSHIBA)
static U8 _u8FactoryIRHeaderCode0 = IR_FHEADER_CODE0_0;
static U8 _u8FactoryIRHeaderCode1 = IR_FHEADER_CODE1_0;

#define    IR_REPEAT_SKIP_COUNT    2

#else
static U8 _u8FactoryIRHeaderCode0 = 0x00UL;
static U8 _u8FactoryIRHeaderCode1 = 0x00UL;
#endif

#define IR_RAW_DATA_NUM            4
//#define IR_FILTER_REPEAT_NUM    1

#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
#define IR_SWDECODE_MODE_BUF_LEN        100
#endif

#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
#if(IR_TYPE_SEL != IR_TYPE_RCMM)
static U32  _u32IRData[IR_SWDECODE_MODE_BUF_LEN];
static U32  _u32IRCount=0;
#endif
#endif

#ifdef CONFIG_MSTAR_IR_POWER_KEY_LONG_PRESS
static unsigned long _u16IRLongPressPrevTime = 0;
#endif

#if(IR_MODE_SEL == IR_TYPE_SWDECODE_KON_MODE)// pulse width modulation format: Header(1)+CusterCode(8)+Data(8)+Stop(1)
#define IR_KON_DATA_BITS          18       //
#define IR_KON_DETECT_START         BIT0   // start to detect IR int
#define IR_KON_DETECT_END             BIT1   // end to detect IR int
#define IR_KON_COUNTER_ERROR        BIT6   // for temp use
#define IR_KON_REPEATE_TIMEOUT      BIT5
static volatile U8 g_u8IrKonFlag;            // for store bit var
static volatile U8 g_u8IrKonBits;                // for store bit count
static volatile U16 g_u16IrKonCounter;         // for read counter from register
static volatile U16 g_u16IrKonData;    // store shift data
static volatile U16 g_u16IrKonDecode;    // store shift data
static U8 g_u8IrKonRepeatCount;    // for repeat
static U8 g_u8IrKonPreKey;
static unsigned long g_u8IrKonRepeatTimeout;
#define IR_KON_PWS3_HEADER_CNT_LB       3000
#define IR_KON_PWS3_HEADER_CNT_UB       4000
#define IR_KON_PWS3_LOGIC0_CNT_LB       1500
#define IR_KON_PWS3_LOGIC0_CNT_UB       2500
#define IR_KON_PWS3_LOGIC1_CNT_LB       2500
#define IR_KON_PWS3_LOGIC1_CNT_UB       3500
#define IR_KON_PWS3_STOP_CNT_LB       4000
#define IR_KON_PWS3_STOP_CNT_UB       5000
#define IR_KON_PWS3_REPEATE_TIMEOUT     150
#endif

#if 0
#define DEBUG_IR(x) (x)
#else
#define DEBUG_IR(x)
#endif

#if 0
#define DEBUG_IR_INPUT_DEVICE(x) (x)
#else
#define DEBUG_IR_INPUT_DEVICE(x)
#endif

#ifdef CONFIG_MP_DEBUG_TOOL_CHANGELIST
char g_sChangeList[] = KERM_CL;
#endif

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_IR_KEY_PROPERTY_INIT,
    E_IR_KEY_PROPERTY_1st,
    E_IR_KEY_PROPERTY_FOLLOWING
} IRKeyProperty;

#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
struct mstar_ir
{
    struct rc_dev *dev;
};
#endif

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------
static U8 bIRPass = 0;
static pid_t MasterPid = 0;

#ifdef CONFIG_MSTAR_SOFTWARE_IR_MODULE
U8 u8Key_for_mdrv_software_ir = 0;
U8 u8RepeatFlag_for_mdrv_software_ir = 0;
#endif

#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
struct mstar_ir *ir;
struct completion key_completion;
#endif

#if defined(CONFIG_MSTAR_GPIO) && defined(CONFIG_MSTAR_IR_GPIO_TOGGLE)
struct completion ir_gpio_completion;
#endif
//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Local Variables
//-------------------------------------------------------------------------------------------------
#if (IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
static U8   _u8IRRawModeBuf[IR_RAW_DATA_NUM];
static U32  _u8IRRawModeCount;
static unsigned long  _ulPrevKeyTime;
#endif

#ifdef SUPPORT_MULTI_PROTOCOL
struct {
     wait_queue_head_t inq;
     struct semaphore sem;
     long long data; // 0=nothing, defined in fantasy protocol
} fantasy_protocol;
#else
struct {
    wait_queue_head_t inq;
    struct semaphore sem;
    U32 data; // 0=nothing, defined in fantasy protocol
} fantasy_protocol;
#endif

#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)

#if(IR_TYPE_SEL == IR_TYPE_RCMM)
#define MAX_RCBYTE_LEN  4
static BOOL StartDecodeFlag = FALSE;
static U8 RCBitsCnt;
static U32 tgbits = 0;
static U8 RCByte[MAX_RCBYTE_LEN];
static U8 _u8IrPreRcmmData[MAX_RCBYTE_LEN];
static U16 u16CustomerID;
static BOOL UpDataFlage = FALSE;
static U8 RCMode;
static unsigned long  _ulPrevKeyTime;

#endif


#if ((IR_TYPE_SEL != IR_TYPE_CUS21SH) && (IR_TYPE_SEL != IR_TYPE_TOSHIBA))
#if ((IR_TYPE_SEL != IR_TYPE_CUS08_RC5) && (IR_TYPE_SEL != IR_TYPE_RCMM)&&(IR_TYPE_SEL != IR_TYPE_SWRC6))
static U32  _u32IRData[IR_SWDECODE_MODE_BUF_LEN];
#endif
#else
static U16  _u16IRData[IR_SWDECODE_MODE_BUF_LEN];
#endif
//static U32  _u32IRCount=0;

#if (IR_TYPE_SEL == IR_TYPE_HISENSE || IR_TYPE_SEL == IR_TYPE_MSTAR_DTV || IR_TYPE_SEL == IR_TYPE_CHANGHONG)
static U8   _u8IRHeadReceived=0;
static U8   _u8IRRepeateDetect=0;
static U8   _u8IRRepeated=0;
static U8   _u8IRRepeatedNum=0;
static U8   _u8IRType=0;

#elif (IR_TYPE_SEL == IR_TYPE_CUS08_RC5)
static U16  _u16IrRc5Data=0;          // for store shift ir data
static U16  _u16PreIrRc5Data=0;          // for store previous ir data
static U8 _u8IrRc5Bits=0;                // for store bit count
static U8 _u8IrRc5LastBit=0;//IR_RC5_LAST_BIT;            // for store bit var
static unsigned long  _ulPrevKeyTime;

#elif(IR_TYPE_SEL == IR_TYPE_TCL)
static BOOL _brepeat_flag;
static unsigned long  _ulPrevKeyTime;
static unsigned long  _ulPrevKeyTimeFirstPress;    //qiyx add for "OK" Long Press Key

#elif (IR_TYPE_SEL == IR_TYPE_CUS21SH)
static U8   _u8IRReceiveDetect = 0;
static U8   _u8IRRepeateDetect = 0;
static U16  _u16IRKeyMap = 0;
static BOOL _bKeyValueHit = FALSE;

/////////
U8 g_bIrDetect;//!<IR command detect flag

U8 g_ucIrRepeatSkipCnt = 0; //IR_REPEAT_SKIP_COUNT;
U32 gIRTimeOutCount = 0;
static U16 _u16BufferCurrent=0;
static U16 _u16BufferPrev=0;
static U8 LastKeyData=0xFF;
static U8 IsRepeatData=0;
static U8 keyDatArrayWTIdx=0 ;
static U8 keyDatArrayRDIdx=0 ;
static BOOL _bExpectedTCome=TRUE;
static BOOL    FirstRXFlag=FALSE;
BOOL   SecondRXFlag=FALSE;
static U32 RxTimeOutCount;
static U32 CurrentTime;
static BOOL RxInProcess=FALSE;
static U32 _u32KeyOffTimeOutCount;
static BOOL ReceivingMode=FALSE;
static BOOL SetToStandbyMode=FALSE;

U8 g_u8Key;
U8 g_u8Flag;

////////

#elif (IR_TYPE_SEL == IR_TYPE_TOSHIBA)
static U32  _u32IRKeyMap = 0;
static BOOL _bRxInProcess = FALSE;
static BOOL _bIrRepeat = FALSE;
static BOOL _bIrDetect = FALSE;
static U32    _u32KeyOffTimeOutCount = 0;
static U8    _u8IrRepeatSkipCnt = IR_REPEAT_SKIP_COUNT;

#elif (defined(IR_TYPE_SWRC6) && IR_TYPE_SEL == IR_TYPE_SWRC6)
//define args
enum RC6_STATE {
    STATE_INACTIVE = 0,
    STATE_HEADER_SPACE,
    STATE_RC6_DATA,
    STATE_TRAILER,
};
#define MODE_BITS 12
#define DATA_BITS 16
static U8  status = STATE_INACTIVE;
static U8  rc6_bits=0;
static U32 rc6_databits=0;
static U8 rc6_data[4] ={0};
static U8 data_count =0;
static U8 data_flag = 0;
static U8 rc6_mode=0;
static U8 done_flag =0;
static U8 rc6_databits_to_data(U32 data_bits,U8 bits,U8 reverse)
{
    U8 ret_data=0;
    U8 i = 0;
    U8 number = bits>>1;
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
void _MDrv_IR_Decode_SWRC6(U16 shotcount, U8 flag)
{
    switch(status)
    {
        case STATE_INACTIVE:
            if(flag)
                break;
            if (shotcount>IR_RC6_HDC_LOB&&shotcount<IR_RC6_HDC_UPB)
            {
                status = STATE_HEADER_SPACE;
                return;
            }
            else
                break;
        case STATE_HEADER_SPACE:
            if(!flag)
                break;
            if (shotcount>IR_RC6_HDC_SPACE_LOB&&shotcount<IR_RC6_HDC_SPACE_UPB)
            {
                rc6_bits=0;
                rc6_databits=0;
                memset(rc6_data,0,sizeof(rc6_data));
                data_count =0;
                data_flag = 0;
                rc6_mode=0;
                done_flag =0;
                status = STATE_RC6_DATA;
                return;
            }
            else
            {
                //printf("[Lwc Debug Msg] error 1\n",0);
                break;
            }
        case STATE_RC6_DATA:
            if (shotcount>IR_RC6_BIT_MIN_LOB && shotcount<IR_RC6_BIT_MIN_UPB)
            {
                rc6_bits++;
                rc6_databits <<= 1;
                if(flag)
                    rc6_databits |=0x01;
            }
            else if (shotcount>IR_RC6_BIT_MAX_LOB && shotcount<IR_RC6_BIT_MAX_UPB)
            {
                rc6_bits += 2;
                rc6_databits <<= 2;
                if(flag)
                    rc6_databits |=0x03;
            }
            else// error bits
            {
                //printf("[Lwc Debug Msg] error 2,PulseShot=%d\n",shotcount);
                break;
            }
            if(!data_flag)
            {
                if(rc6_bits>=7)//mode bits over
                {
                    rc6_mode = rc6_databits_to_data(rc6_databits,rc6_bits,0);
                    status = STATE_TRAILER;
                }
            }
            else
            {
                //data handle
                if(rc6_mode ==0)// command + keycode
                {
                    if(data_count == 1)
                    {
                        if(rc6_bits>= DATA_BITS-1)
                        {
                            rc6_data[data_count] = rc6_databits_to_data(rc6_databits,rc6_bits,1);
                            done_flag =1;
                            status = STATE_INACTIVE;
                            return;
                        }
                    }
                }
                else if(rc6_mode ==1)//2bytes command +2bytes keycode [key value=last byte]
                {
                    if(data_count == 3)
                    {
                        if(rc6_bits>= DATA_BITS-1)
                        {
                            rc6_data[data_count] = rc6_databits_to_data(rc6_databits,rc6_bits,1);
                            done_flag =1;
                            status = STATE_INACTIVE;
                            return;
                        }
                    }

                }
                else
                {
                    //for RC6 else mode [unrealized]
                    status = STATE_INACTIVE;
                    return;
                }
                if(rc6_bits >= DATA_BITS)
                {
                    rc6_data[data_count] = rc6_databits_to_data((rc6_databits>>(rc6_bits-DATA_BITS)),DATA_BITS,1);
                    rc6_bits = rc6_bits - DATA_BITS;
                    rc6_databits &= ~((~0)<<rc6_bits);
                    data_count++;
                }
            }
            return ;
       case STATE_TRAILER:
            if (shotcount>IR_RC6_BIT_MIN_LOB&&shotcount<IR_RC6_BIT_MIN_UPB)
            {
                rc6_bits += 1;
                rc6_databits <<= 1;
                if(flag)
                    rc6_databits |=0x01;
            }
            else if (shotcount>IR_RC6_TRAILER_MIN_LOB&&shotcount<IR_RC6_TRAILER_MIN_UPB)
            {
                rc6_bits += 2;
                rc6_databits <<= 2;
                if(flag)
                    rc6_databits |=0x03;
            }
            else if (shotcount>IR_RC6_TRAILER_MAX_LOB&&shotcount<IR_RC6_TRAILER_MAX_UPB)
            {
                rc6_bits += 3;
                rc6_databits <<= 3;
                if(flag)
                    rc6_databits |=0x07;
            }
            else
            {
                //printf("[Lwc Debug Msg] error 3\n",0);
                break;
            }
            if(rc6_bits>= MODE_BITS)
            {
                data_flag =1;
                rc6_bits= rc6_bits - MODE_BITS;
                rc6_databits &= ~((~0)<<rc6_bits);
                status = STATE_RC6_DATA;
            }
            return;
       default :
            //printf("[Lwc Debug Msg] error 4\n",0);
            break;
     }
     status = STATE_INACTIVE;
     return ;
}

#endif
#endif

static U32  _u32_1stDelayTimeMs;
static U32  _u32_2ndDelayTimeMs;
static IRKeyProperty _ePrevKeyProperty;

#if (IR_MODE_SEL != IR_TYPE_HWRC_MODE)
static U8   _u8PrevKeyCode;
#endif
static U8   _u8PrevSystemCode;

#if !defined(IR_TYPE_SKYWORTH) || (IR_TYPE_SEL != IR_TYPE_SKYWORTH)
#if (IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
static unsigned long  _ulPrevKeyTime;
#endif
#endif

#if defined(IR_TYPE_HAIER) && (IR_TYPE_SEL == IR_TYPE_HAIER)
static BOOL _btoshiba_code;
static BOOL _brepeat_flag;
static BOOL _bBeginRepeat_flag;
static U8     _Repeat_Num;
#endif
#if defined(IR_TYPE_SKYWORTH) && (IR_TYPE_SEL == IR_TYPE_SKYWORTH )
static BOOL _brepeat_flag;
static U8 _u8BeforeHeader=0;
static unsigned long  _ulPrevKeyTime;
static BOOL _bIRStart;
static BOOL    _bskyworth_shuttle_code;
static BOOL _bskyworth_normal_code;
#endif

#ifdef SUPPORT_MULTI_PROTOCOL
static IR_PROCOCOL_TYPE _eCurentProtocol=E_IR_PROTOCOL_NONE;
IR_PROCOCOL_TYPE _eLastKeyProtocol=E_IR_PROTOCOL_NONE;//last used protocol of processing key success without timeout
static unsigned long long _u64LastData=0;//last rawdata of processing key success without timeout
static unsigned long _ulPreShotTime=0;
static IR_PROCOCOL_TYPE _eDetectList[PROTOCOL_SUPPORT_MAX];
static struct timer_list    _key_MultiProtocol_timer;
#endif
//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static void _MDrv_IR_Timing(void);
#if (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
irqreturn_t _MDrv_IR_RC_ISR(int irq, void *dev_id);
#else
irqreturn_t _MDrv_IR_ISR(int irq, void *dev_id);
#endif
static BOOL _MDrv_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag);
unsigned long _MDrv_IR_GetSystemTime(void);

#if ((IR_TYPE_SEL == IR_TYPE_TOSHIBA) || (IR_TYPE_SEL == IR_TYPE_CUS21SH))
static unsigned long _MDrv_IR_DiffTimeFromNow(unsigned long time)
{
    return (((unsigned long)((jiffies)*(1000/HZ))) - time);
}

void ResetKeyoffTimer(void)
{
    _u32KeyOffTimeOutCount = _MDrv_IR_GetSystemTime();
}

U32 GetKeyoffTimer(void)
{
    return (_MDrv_IR_GetSystemTime() - _u32KeyOffTimeOutCount);
}
#endif

#if (IR_TYPE_SEL == IR_TYPE_TOSHIBA)

#if (IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
static U8 _MDrv_IR_CUS22T_ParseKey(U8 u8KeyData)
{
    U8 u8RetVal = 0xFFUL;

    //printk("Linux: %s:%s:%d u8KeyData:%x\n",__FILE__,__FUNCTION__,__LINE__,u8KeyData);

    switch(u8KeyData)
    {
        case CUS22T_IRKEY_TV_RADIO          : u8RetVal = IRKEY_TV_RADIO;           break;
        case CUS22T_IRKEY_CHANNEL_LIST      : u8RetVal = IRKEY_CHANNEL_LIST;       break;
        case CUS22T_IRKEY_CHANNEL_FAV_LIST: u8RetVal = IRKEY_CHANNEL_FAV_LIST; break;
        case CUS22T_IRKEY_CHANNEL_RETURN  : u8RetVal = IRKEY_CHANNEL_RETURN;   break;
        case CUS22T_IRKEY_CHANNEL_PLUS      : u8RetVal = IRKEY_CHANNEL_PLUS;       break;
        case CUS22T_IRKEY_CHANNEL_MINUS   : u8RetVal = IRKEY_CHANNEL_MINUS;    break;

        case CUS22T_IRKEY_AUDIO           : u8RetVal = IRKEY_AUDIO;            break;
        case CUS22T_IRKEY_VOLUME_PLUS      : u8RetVal = IRKEY_VOLUME_PLUS;       break;
        case CUS22T_IRKEY_VOLUME_MINUS      : u8RetVal = IRKEY_VOLUME_MINUS;       break;

        case CUS22T_IRKEY_UP              : u8RetVal = IRKEY_UP;               break;
        case CUS22T_IRKEY_POWER           : u8RetVal = IRKEY_POWER;            break;
        case CUS22T_IRKEY_EXIT              : u8RetVal = IRKEY_EXIT;               break;
        case CUS22T_IRKEY_MENU              : u8RetVal = IRKEY_MENU;               break;
        case CUS22T_IRKEY_DOWN              : u8RetVal = IRKEY_DOWN;               break;
        case CUS22T_IRKEY_LEFT              : u8RetVal = IRKEY_LEFT;               break;
        case CUS22T_IRKEY_SELECT          : u8RetVal = IRKEY_SELECT;           break;
        case CUS22T_IRKEY_RIGHT           : u8RetVal = IRKEY_RIGHT;            break;

        case CUS22T_IRKEY_NUM_0           : u8RetVal = IRKEY_NUM_0;            break;
        case CUS22T_IRKEY_NUM_1           : u8RetVal = IRKEY_NUM_1;            break;
        case CUS22T_IRKEY_NUM_2           : u8RetVal = IRKEY_NUM_2;            break;
        case CUS22T_IRKEY_NUM_3           : u8RetVal = IRKEY_NUM_3;            break;
        case CUS22T_IRKEY_NUM_4           : u8RetVal = IRKEY_NUM_4;            break;
        case CUS22T_IRKEY_NUM_5           : u8RetVal = IRKEY_NUM_5;            break;
        case CUS22T_IRKEY_NUM_6           : u8RetVal = IRKEY_NUM_6;            break;
        case CUS22T_IRKEY_NUM_7           : u8RetVal = IRKEY_NUM_7;            break;
        case CUS22T_IRKEY_NUM_8           : u8RetVal = IRKEY_NUM_8;            break;
        case CUS22T_IRKEY_NUM_9           : u8RetVal = IRKEY_NUM_9;            break;

        case CUS22T_IRKEY_MUTE              : u8RetVal = IRKEY_MUTE;               break;
        case CUS22T_IRKEY_PAGE_UP          : u8RetVal = IRKEY_PAGE_UP;           break;
        //case CUS22T_IRKEY_PAGE_DOWN        : u8RetVal = IRKEY_PAGE_DOWN;         break;
        case CUS22T_IRKEY_CLOCK           : u8RetVal = IRKEY_CLOCK;            break;

        case CUS22T_IRKEY_INFO              : u8RetVal = IRKEY_INFO;               break;
        case CUS22T_IRKEY_RED              : u8RetVal = IRKEY_RED;               break;
        case CUS22T_IRKEY_GREEN           : u8RetVal = IRKEY_GREEN;            break;
        case CUS22T_IRKEY_YELLOW          : u8RetVal = IRKEY_YELLOW;           break;
        case CUS22T_IRKEY_BLUE              : u8RetVal = IRKEY_BLUE;               break;
        case CUS22T_IRKEY_MTS              : u8RetVal = IRKEY_MTS;               break;
        //case CUS22T_IRKEY_NINE_LATTICE    : u8RetVal = IRKEY_NINE_LATTICE;     break;
#if defined(DVB_SYSTEM)
        case CUS22T_IRKEY_TTX              : u8RetVal = IRKEY_TTX;               break;
#elif defined(ATSC_SYSTEM)
        case CUS22T_IRKEY_CC              : u8RetVal = IRKEY_CC;               break;
#endif
        case CUS22T_IRKEY_INPUT_SOURCE      : u8RetVal = IRKEY_INPUT_SOURCE;       break;
        //case CUS22T_IRKEY_CRADRD            : u8RetVal = IRKEY_CRADRD;             break;
    //      case CUS22T_IRKEY_PICTURE         : u8RetVal = IRKEY_PICTURE;          break;
        case CUS22T_IRKEY_ZOOM              : u8RetVal = IRKEY_ZOOM;               break;
        //case CUS22T_IRKEY_DASH            : u8RetVal = IRKEY_DASH;             break;
        case CUS22T_IRKEY_SLEEP           : u8RetVal = IRKEY_SLEEP;            break;
        case CUS22T_IRKEY_EPG              : u8RetVal = IRKEY_EPG;               break;
        //case CUS22T_IRKEY_PIP             : u8RetVal = IRKEY_PIP;              break;

        //case CUS22T_IRKEY_MIX             : u8RetVal = IRKEY_MIX;              break;
        //case CUS22T_IRKEY_INDEX            : u8RetVal = IRKEY_INDEX;             break;
        case CUS22T_IRKEY_HOLD              : u8RetVal = IRKEY_HOLD;               break;
        //case CUS22T_IRKEY_PREVIOUS        : u8RetVal = IRKEY_PREVIOUS;         break;
        //case CUS22T_IRKEY_NEXT            : u8RetVal = IRKEY_NEXT;             break;
        //case CUS22T_IRKEY_BACKWARD        : u8RetVal = IRKEY_BACKWARD;         break;
        //case CUS22T_IRKEY_FORWARD         : u8RetVal = IRKEY_FORWARD;          break;
        //case CUS22T_IRKEY_PLAY            : u8RetVal = IRKEY_PLAY;             break;
        //case CUS22T_IRKEY_RECORD            : u8RetVal = IRKEY_RECORD;             break;
        //case CUS22T_IRKEY_STOP            : u8RetVal = IRKEY_STOP;             break;
        //case CUS22T_IRKEY_PAUSE            : u8RetVal = IRKEY_PAUSE;             break;

        case CUS22T_IRKEY_SIZE              : u8RetVal = IRKEY_SIZE;               break;
        case CUS22T_IRKEY_REVEAL          : u8RetVal = IRKEY_REVEAL;           break;
        //case CUS22T_IRKEY_SUBCODE         : u8RetVal = IRKEY_SUBCODE;          break;

        case CUS22T_40BE_3D             : u8RetVal = IRKEY_3D;                  break;
        case CUS22T_IRKEY_QUICK         : u8RetVal = IRKEY_QUICK;               break;
        case CUS22T_IRKEY_RETURN        : u8RetVal = IRKEY_RETURN;              break;
        //default                     : u8RetVal = IRKEY_DUMMY;              break;
    }
    return u8RetVal;
}

#elif (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)

static U8 _MDrv_IR_CUS22T_Customer_ParseKey(U32 u32KeyData)
{
    U8 u8RetVal = CUS22T_IRKEY_DUMY;

    u32KeyData &= 0xFFFFFF00UL;

    //printk("Linux: %s:%s:%d u32KeyData:%x",__FILE__,__FUNCTION__,__LINE__,u32KeyData);

    switch(u32KeyData)
    {
        case CUS22T_SW_IRKEY_TV_RADIO        : u8RetVal = IRKEY_TV_RADIO;         break;
        case CUS22T_SW_IRKEY_CHANNEL_LIST    : u8RetVal = IRKEY_CHANNEL_LIST;     break;
        case CUS22T_SW_IRKEY_CHANNEL_FAV_LIST: u8RetVal = IRKEY_CHANNEL_FAV_LIST; break;
        case CUS22T_SW_IRKEY_CHANNEL_RETURN  : u8RetVal = IRKEY_CHANNEL_RETURN;   break;
        case CUS22T_SW_IRKEY_CHANNEL_PLUS    : u8RetVal = IRKEY_CHANNEL_PLUS;     break;
        case CUS22T_SW_IRKEY_CHANNEL_MINUS   : u8RetVal = IRKEY_CHANNEL_MINUS;    break;

        case CUS22T_SW_IRKEY_AUDIO           : u8RetVal = IRKEY_AUDIO;            break;
        case CUS22T_SW_IRKEY_VOLUME_PLUS     : u8RetVal = IRKEY_VOLUME_PLUS;      break;
        case CUS22T_SW_IRKEY_VOLUME_MINUS    : u8RetVal = IRKEY_VOLUME_MINUS;     break;

        case CUS22T_SW_IRKEY_UP              : u8RetVal = IRKEY_UP;               break;
        case CUS22T_SW_IRKEY_POWER           : u8RetVal = IRKEY_POWER;            break;
        case CUS22T_SW_IRKEY_EXIT            : u8RetVal = IRKEY_EXIT;             break;
        case CUS22T_SW_IRKEY_MENU            : u8RetVal = IRKEY_MENU;             break;
        case CUS22T_SW_IRKEY_DOWN            : u8RetVal = IRKEY_DOWN;             break;
        case CUS22T_SW_IRKEY_LEFT            : u8RetVal = IRKEY_LEFT;             break;
        case CUS22T_SW_IRKEY_SELECT          : u8RetVal = IRKEY_SELECT;           break;
        case CUS22T_SW_IRKEY_RIGHT           : u8RetVal = IRKEY_RIGHT;            break;

        case CUS22T_SW_IRKEY_NUM_0           : u8RetVal = IRKEY_NUM_0;            break;
        case CUS22T_SW_IRKEY_NUM_1           : u8RetVal = IRKEY_NUM_1;            break;
        case CUS22T_SW_IRKEY_NUM_2           : u8RetVal = IRKEY_NUM_2;            break;
        case CUS22T_SW_IRKEY_NUM_3           : u8RetVal = IRKEY_NUM_3;            break;
        case CUS22T_SW_IRKEY_NUM_4           : u8RetVal = IRKEY_NUM_4;            break;
        case CUS22T_SW_IRKEY_NUM_5           : u8RetVal = IRKEY_NUM_5;            break;
        case CUS22T_SW_IRKEY_NUM_6           : u8RetVal = IRKEY_NUM_6;            break;
        case CUS22T_SW_IRKEY_NUM_7           : u8RetVal = IRKEY_NUM_7;            break;
        case CUS22T_SW_IRKEY_NUM_8           : u8RetVal = IRKEY_NUM_8;            break;
        case CUS22T_SW_IRKEY_NUM_9           : u8RetVal = IRKEY_NUM_9;            break;

        case CUS22T_SW_IRKEY_MUTE            : u8RetVal = IRKEY_MUTE;             break;
        case CUS22T_SW_IRKEY_PAGE_UP         : u8RetVal = IRKEY_PAGE_UP;          break;
        //case CUS22T_SW_IRKEY_PAGE_DOWN       : u8RetVal = IRKEY_PAGE_DOWN;        break;
        case CUS22T_SW_IRKEY_CLOCK           : u8RetVal = IRKEY_CLOCK;            break;

        case CUS22T_SW_IRKEY_INFO            : u8RetVal = IRKEY_INFO;             break;
        case CUS22T_SW_IRKEY_RED             : u8RetVal = IRKEY_RED;              break;
        case CUS22T_SW_IRKEY_GREEN           : u8RetVal = IRKEY_GREEN;            break;
        case CUS22T_SW_IRKEY_YELLOW          : u8RetVal = IRKEY_YELLOW;           break;
        case CUS22T_SW_IRKEY_BLUE            : u8RetVal = IRKEY_BLUE;             break;
        case CUS22T_SW_IRKEY_MTS             : u8RetVal = IRKEY_MTS;              break;
        //case CUS22T_SW_IRKEY_NINE_LATTICE    : u8RetVal = IRKEY_NINE_LATTICE;     break;

        case CUS22T_SW_IRKEY_INPUT_SOURCE    : u8RetVal = IRKEY_INPUT_SOURCE;     break;
        //case CUS22T_SW_IRKEY_CRADRD          : u8RetVal = IRKEY_CRADRD;           break;
        case CUS22T_SW_IRKEY_PICTURE         : u8RetVal = IRKEY_PICTURE;          break;
        case CUS22T_SW_IRKEY_ZOOM            : u8RetVal = IRKEY_ZOOM;             break;
        //case CUS22T_SW_IRKEY_DASH            : u8RetVal = IRKEY_DASH;             break;
        case CUS22T_SW_IRKEY_SLEEP           : u8RetVal = IRKEY_SLEEP;            break;
        case CUS22T_SW_IRKEY_EPG             : u8RetVal = IRKEY_EPG;              break;
        //case CUS22T_SW_IRKEY_PIP             : u8RetVal = IRKEY_PIP;              break;

        //case CUS22T_SW_IRKEY_MIX             : u8RetVal = IRKEY_MIX;              break;
        //case CUS22T_SW_IRKEY_INDEX           : u8RetVal = IRKEY_INDEX;            break;
        //case CUS22T_SW_IRKEY_HOLD            : u8RetVal = IRKEY_HOLD;             break;
        //case CUS22T_SW_IRKEY_PREVIOUS        : u8RetVal = IRKEY_PREVIOUS;         break;
        //case CUS22T_SW_IRKEY_NEXT            : u8RetVal = IRKEY_NEXT;             break;
        //case CUS22T_SW_IRKEY_BACKWARD        : u8RetVal = IRKEY_BACKWARD;         break;
        //case CUS22T_SW_IRKEY_FORWARD         : u8RetVal = IRKEY_FORWARD;          break;
        //case CUS22T_SW_IRKEY_PLAY            : u8RetVal = IRKEY_PLAY;             break;
        //case CUS22T_SW_IRKEY_RECORD          : u8RetVal = IRKEY_RECORD;           break;
        //case CUS22T_SW_IRKEY_STOP            : u8RetVal = IRKEY_STOP;             break;
        //case CUS22T_SW_IRKEY_PAUSE           : u8RetVal = IRKEY_PAUSE;            break;

        case CUS22T_SW_IRKEY_TEXT               : u8RetVal = IRKEY_TTX;                 break;
        case CUS22T_SW_IRKEY_MEDIA_PLAYER       : u8RetVal = IRKEY_MEDIA_PLAYER;        break;
        case CUS22T_SW_IRKEY_SUBTITLE           : u8RetVal = IRKEY_SUBCODE;             break;

        case CUS22T_SW_IRKEY_REW                : u8RetVal = IRKEY_BACKWARD;            break;
        case CUS22T_SW_IRKEY_PLAY               : u8RetVal = IRKEY_PLAY;                break;
        case CUS22T_SW_IRKEY_FF                 : u8RetVal = IRKEY_FORWARD;             break;
        case CUS22T_SW_IRKEY_SKIP_MINUS         : u8RetVal = IRKEY_PREVIOUS;            break;
        case CUS22T_SW_IRKEY_STOP               : u8RetVal = IRKEY_STOP;                break;
        case CUS22T_SW_IRKEY_PAUSE              : u8RetVal = IRKEY_PAUSE;               break;
        case CUS22T_SW_IRKEY_SKIP_PLUS          : u8RetVal = IRKEY_NEXT;                break;

        case CUS22T_SW_IRKEY_SIZE            : u8RetVal = IRKEY_SIZE;             break;
        //case CUS22T_SW_IRKEY_REVEAL          : u8RetVal = IRKEY_REVEAL;           break;
        //case CUS22T_SW_IRKEY_SUBCODE         : u8RetVal = IRKEY_SUBCODE;          break;

        case CUS22T_SW_IRKEY_3D                 : u8RetVal = IRKEY_3D;                  break;
        case CUS22T_SW_IRKEY_QUICK              : u8RetVal = IRKEY_QUICK;               break;
        case CUS22T_SW_IRKEY_RETURN             : u8RetVal = IRKEY_RETURN;              break;
        case CUS22T_SW_IRKEY_TV_INPUT           : u8RetVal = IRKEY_TV_INPUT;            break;


        case CUS22T_SW_F_40BF_PIP_ON_OFF        : u8RetVal = F_40BF_PIP_ON_OFF;         break;
        case CUS22T_SW_F_40BF_TUNE_DOWN_SEARCH  : u8RetVal = F_40BF_TUNE_DOWN_SEARCH;   break;
        case CUS22T_SW_F_40BF_TUNE_UP_SEARCH    : u8RetVal = F_40BF_TUNE_UP_SEARCH;     break;
        case CUS22T_SW_F_40BF_MEM               : u8RetVal = F_40BF_MEM;                break;
        case CUS22T_SW_F_40BF_COLOR_SYS_CHECK   : u8RetVal = F_40BF_COLOR_SYS_CHECK;    break;
        case CUS22T_SW_F_40BF_SOUND_SYS_CHECK   : u8RetVal = F_40BF_SOUND_SYS_CHECK;    break;
        case CUS22T_SW_F_40BF_E_D_MODE          : u8RetVal = F_40BF_E_D_MODE;           break;


        default                             : u8RetVal = IRKEY_DUMY;            break;
    }
    printk("==>%x\n",u8RetVal);
    return u8RetVal;
}

static U8 _MDrv_IR_CUS22T_Factory_ParseKey(U32 u32KeyData)
{
    U8 u8RetVal = CUS22T_IRKEY_DUMY;

    printk("[IR] Linux: %s:%s:%d u32KeyData:%x\n",__FILE__,__FUNCTION__,__LINE__,u32KeyData);

    if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40E2_COMMAND)
    {
        u32KeyData &= 0xFFFFFF00UL;

        switch(u32KeyData)
        {
            case CUS22T_SW_F_40E2_CONTRAST        : u8RetVal = F_40E2_CONTRAST    ;   break;
            case CUS22T_SW_F_40E2_BRIGHT          : u8RetVal = F_40E2_BRIGHT    ;   break;
            case CUS22T_SW_F_40E2_COLOR           : u8RetVal = F_40E2_COLOR        ;   break;
            case CUS22T_SW_F_40E2_TINT            : u8RetVal = F_40E2_TINT        ;   break;
            case CUS22T_SW_F_40E2_SHARPNESS       : u8RetVal = F_40E2_SHARPNESS ;   break;
            case CUS22T_SW_F_40E2_VOLUME          : u8RetVal = F_40E2_VOLUME    ;   break;
            case CUS22T_SW_F_40E2_BASS            : u8RetVal = F_40E2_BASS        ;   break;
            case CUS22T_SW_F_40E2_TREBLE          : u8RetVal = F_40E2_TREBLE    ;   break;
            case CUS22T_SW_F_40E2_BALANCE         : u8RetVal = F_40E2_BALANCE    ;   break;
            case CUS22T_SW_F_40E2_RCUT            : u8RetVal = F_40E2_RCUT        ;   break;
            case CUS22T_SW_F_40E2_GCUT            : u8RetVal = F_40E2_GCUT        ;   break;
            case CUS22T_SW_F_40E2_BCUT            : u8RetVal = F_40E2_BCUT        ;   break;
            case CUS22T_SW_F_40E2_RDRV            : u8RetVal = F_40E2_RDRV        ;   break;
            case CUS22T_SW_F_40E2_BDRV            : u8RetVal = F_40E2_BDRV        ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40E3_COMMAND)
    {
        u32KeyData &= 0xFFFFFF00UL;

        switch(u32KeyData)
        {
            case CUS22T_SW_F_40E3_RCUT            : u8RetVal = F_40E3_RCUT        ;   break;
            case CUS22T_SW_F_40E3_GCUT            : u8RetVal = F_40E3_GCUT        ;   break;
            case CUS22T_SW_F_40E3_BCUT            : u8RetVal = F_40E3_BCUT        ;   break;
            case CUS22T_SW_F_40E3_RDRV_GDRV       : u8RetVal = F_40E3_RDRV_GDRV ;   break;
            case CUS22T_SW_F_40E3_BDRV            : u8RetVal = F_40E3_BDRV      ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40EA_COMMAND)
    {
        u32KeyData &= 0xFFFFFF00;

        switch(u32KeyData)
        {
            case CUS22T_SW_F_40EA_INTERNAL_PATTERN_OFF  : u8RetVal = F_40EA_INTERNAL_PATTERN_OFF ;   break;
            case CUS22T_SW_F_40EA_RED_RASTER            : u8RetVal = F_40EA_RED_RASTER           ;   break;
            case CUS22T_SW_F_40EA_GREEN_RASTER          : u8RetVal = F_40EA_GREEN_RASTER         ;   break;
            case CUS22T_SW_F_40EA_BLUE_RASTER           : u8RetVal = F_40EA_BLUE_RASTER          ;   break;
            case CUS22T_SW_F_40EA_BLACK_RASTER          : u8RetVal = F_40EA_BLACK_RASTER         ;   break;
            case CUS22T_SW_F_40EA_WHITE_RASTER          : u8RetVal = F_40EA_WHITE_RASTER         ;   break;
            case CUS22T_SW_F_40EA_AGING_MODE_1          : u8RetVal = F_40EA_AGING_MODE_1         ;   break;
            case CUS22T_SW_F_40EA_AGINE_MODE_2          : u8RetVal = F_40EA_AGINE_MODE_2         ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40EB_COMMAND)
    {

        u32KeyData &= 0xFFFFFF00UL;

        switch(u32KeyData)
        {
            case CUS22T_SW_F_40EB_DIRECT_CH                 : u8RetVal = F_40EB_DIRECT_CH               ;   break;
            case CUS22T_SW_F_40EB_DIRECT_RF                 : u8RetVal = F_40EB_DIRECT_RF                ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT1         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT1       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT2         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT2       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT3         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT3       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT5         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT5       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT6         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT6       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT7         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT7       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT8         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT8       ;   break;
            case CUS22T_SW_F_40EB_DIRECT_VIDEO_EXT9         : u8RetVal = F_40EB_DIRECT_VIDEO_EXT9       ;   break;
            case CUS22T_SW_F_40EB_SIDE_SHARED_AUDEO_EXT2    : u8RetVal = F_40EB_SIDE_SHARED_AUDEO_EXT2  ;   break;
            case CUS22T_SW_F_40EB_SIDE_SHARED_AUDIO_EXT3    : u8RetVal = F_40EB_SIDE_SHARED_AUDIO_EXT3  ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40EE_COMMAND)
    {
        u32KeyData &= 0xFFFFFF00UL;

        switch(u32KeyData)
        {
            case CUS22T_SW_F_40EE_ALL_CH                     : u8RetVal = F_40EE_ALL_CH                     ;   break;
            case CUS22T_SW_F_40EE_M_MODE                     : u8RetVal = F_40EE_M_MODE                     ;   break;
            case CUS22T_SW_F_40EE_INITIALIZATION             : u8RetVal = F_40EE_INITIALIZATION             ;   break;
            case CUS22T_SW_F_40EE_PICTURE_MODE_DYNAMIC       : u8RetVal = F_40EE_PICTURE_MODE_DYNAMIC     ;   break;
            case CUS22T_SW_F_40EE_M_OSD_OFF                  : u8RetVal = F_40EE_M_OSD_OFF                 ;   break;
            case CUS22T_SW_F_40EE_TJP_FACTORY_SIGNAL_SETTING : u8RetVal = F_40EE_TJP_FACTORY_SIGNAL_SETTING   ;   break;
            case CUS22T_SW_F_40EE_3D_ON_SIDE_BY_SIDE         : u8RetVal = F_40EE_3D_ON_SIDE_BY_SIDE         ;   break;
            case CUS22T_SW_F_40EE_DATA_1_UP                  : u8RetVal = F_40EE_DATA_1_UP                 ;   break;
            case CUS22T_SW_F_40EE_DATA_1_DN                  : u8RetVal = F_40EE_DATA_1_DN                 ;   break;
            case CUS22T_SW_F_40EE_DATA_4_UP                  : u8RetVal = F_40EE_DATA_4_UP                 ;   break;
            case CUS22T_SW_F_40EE_DATA_4_DN                  : u8RetVal = F_40EE_DATA_4_DN                 ;   break;
            case CUS22T_SW_F_40EE_2D_3D_CONVERSION           : u8RetVal = F_40EE_2D_3D_CONVERSION         ;   break;
            case CUS22T_SW_F_40EE_AUDIO_BALANCE_TOGGLE       : u8RetVal = F_40EE_AUDIO_BALANCE_TOGGLE     ;   break;
            case CUS22T_SW_F_40EE_LED_CHECK                  : u8RetVal = F_40EE_LED_CHECK                 ;   break;
            case CUS22T_SW_F_40EE_4_3_STRETCH_TOGGLE         : u8RetVal = F_40EE_4_3_STRETCH_TOGGLE         ;   break;
            case CUS22T_SW_F_40EE_DNR_SELECT                 : u8RetVal = F_40EE_DNR_SELECT                 ;   break;
            case CUS22T_SW_F_40EE_AUTO_BRIGHTNESS_SENSOR     : u8RetVal = F_40EE_AUTO_BRIGHTNESS_SENSOR   ;   break;
            case CUS22T_SW_F_40EE_AUTO_BRIGHTNESS_SENSOR_DETECT_VALUE_DISPLAY : u8RetVal = F_40EE_AUTO_BRIGHTNESS_SENSOR_DETECT_VALUE_DISPLAY   ;   break;
            case CUS22T_SW_F_40EE_EDID_DATA_DOWNLOAD         : u8RetVal = F_40EE_EDID_DATA_DOWNLOAD         ;   break;
            case CUS22T_SW_F_40EE_STABLE_SOUND_ON_OFF        : u8RetVal = F_40EE_STABLE_SOUND_ON_OFF     ;   break;
            case CUS22T_SW_F_40EE_CLEAR_SCAN_100_ON_OFF      : u8RetVal = F_40EE_CLEAR_SCAN_100_ON_OFF   ;   break;
            case CUS22T_SW_F_40EE_CLEAR_SCAN_200_PRO_ON_OFF  : u8RetVal = F_40EE_CLEAR_SCAN_200_PRO_ON_OFF   ;   break;
            case CUS22T_SW_F_40EE_MEDIA_PLAYER_MOVIE         : u8RetVal = F_40EE_MEDIA_PLAYER_MOVIE         ;   break;
            case CUS22T_SW_F_40EE_BASE_BOOST_ON_OFF          : u8RetVal = F_40EE_BASE_BOOST_ON_OFF         ;   break;
            case CUS22T_SW_F_40EE_BLACK_WHITE_LEVEL          : u8RetVal = F_40EE_BLACK_WHITE_LEVEL         ;   break;
            case CUS22T_SW_F_40EE_NEWZEALAND_PRESET          : u8RetVal = F_40EE_NEWZEALAND_PRESET         ;   break;
            case CUS22T_SW_F_40EE_DLNA_TEST                  : u8RetVal = F_40EE_DLNA_TEST                 ;   break;
            case CUS22T_SW_F_40EE_COLOR_TEMPERATURE          : u8RetVal = F_40EE_COLOR_TEMPERATURE       ;   break;
            case CUS22T_SW_F_40EE_BLACK_LIGHT_CHECK          : u8RetVal = F_40EE_BLACK_LIGHT_CHECK       ;   break;
            case CUS22T_SW_F_40EE_DEFAULT_MAC_IP             : u8RetVal = F_40EE_DEFAULT_MAC_IP             ;   break;
            case CUS22T_SW_F_40EE_MAC_ADDRESS_DELETE         : u8RetVal = F_40EE_MAC_ADDRESS_DELETE         ;   break;
            case CUS22T_SW_F_40EE_ADC_ADJUSTMENT             : u8RetVal = F_40EE_ADC_ADJUSTMENT             ;   break;
            case CUS22T_SW_F_40EE_DUAL_1                     : u8RetVal = F_40EE_DUAL_1                     ;   break;
            case CUS22T_SW_F_40EE_MONO_MODE                  : u8RetVal = F_40EE_MONO_MODE                 ;   break;
            case CUS22T_SW_F_40EE_STEREO_MODE                : u8RetVal = F_40EE_STEREO_MODE             ;   break;
            case CUS22T_SW_F_40EE_DUAL_2                     : u8RetVal = F_40EE_DUAL_2                     ;   break;
            case CUS22T_SW_F_40EE_HDMI_2_AUDIO               : u8RetVal = F_40EE_HDMI_2_AUDIO            ;   break;
            case CUS22T_SW_F_40EE_HDMI_2_AUDIO_SELECT        : u8RetVal = F_40EE_HDMI_2_AUDIO_SELECT     ;   break;
            case CUS22T_SW_F_40EE_S_MODE                     : u8RetVal = F_40EE_S_MODE                  ;   break;
            case CUS22T_SW_F_40EE_SIDE_PANEL_SELECT          : u8RetVal = F_40EE_SIDE_PANEL_SELECT       ;   break;
            case CUS22T_SW_F_40EE_PICTURE_SIZE_POSITION      : u8RetVal = F_40EE_PICTURE_SIZE_POSITION   ;   break;
            case CUS22T_SW_F_40EE_MPEG_NR                    : u8RetVal = F_40EE_MPEG_NR                 ;   break;
            case CUS22T_SW_F_40EE_SIGNAL_BOOSTER_ON_OFF      : u8RetVal = F_40EE_SIGNAL_BOOSTER_ON_OFF   ;   break;
            case CUS22T_SW_F_40EE_PICTURE_RESET              : u8RetVal = F_40EE_PICTURE_RESET           ;   break;
            case CUS22T_SW_F_40EE_DIGITAL_AUDIO_OUT          : u8RetVal = F_40EE_DIGITAL_AUDIO_OUT       ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40F0_COMMAND)
    {
        switch(u32KeyData)
        {
            case CUS22T_SW_F_40F0_EEPROM_MEM_DATE_UPDATE    : u8RetVal = F_40F0_EEPROM_MEM_DATE_UPDATE  ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40F1_COMMAND)
    {
        switch(u32KeyData)
        {
            case CUS22T_SW_F_40F1_EEPROM_MEM_DATE_UPDATE    : u8RetVal = F_40F1_EEPROM_MEM_DATE_UPDATE  ;   break;
        }
    }
    else if((u32KeyData & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40F2_COMMAND)
    {
        switch(u32KeyData)
        {
            case CUS22T_SW_F_40F2_EEPROM_MEM_DATE_UPDATE    : u8RetVal = F_40F2_EEPROM_MEM_DATE_UPDATE  ;   break;
        }
    }
    else if((u32KeyData & 0xFF00FFFFUL) == CUS22T_SW_IRKEY_CF00_COMMAND)
    {
        switch(u32KeyData)
        {
            case CUS22T_SW_F_CFXX_WHITE_WINDOW_ON   : u8RetVal = F_CFXX_WHITE_WINDOW_ON ;   break;
        }
    }
    else if((u32KeyData & 0xFF000000UL) == CUS22T_SW_IRKEY_D000_COMMAND)
    {
        switch(u32KeyData)
        {
            case CUS22T_SW_F_D0XX_WHITE_WINDOW_OFF  : u8RetVal = F_D0XX_WHITE_WINDOW_ON ;   break;
        }
    }

    return u8RetVal;
}

static U8 _MDrv_IR_CUS22T_ParseKey(U32 u32KeyData)
{
    U8 u8RetVal = CUS22T_IRKEY_DUMY;
    U32 u32key = CUS22T_IRKEY_DUMY;

    u32key = (((u32KeyData & 0x000000FFUL) << 24)
                | ((u32KeyData & 0x0000FF00UL) << 8)
                | ((u32KeyData & 0x00FF0000UL) >> 8)
                | ((u32KeyData & 0xFF000000UL) >> 24)
                );

    //printk("Linux: %s:%s:%d u32key:%x\n",__FILE__,__FUNCTION__,__LINE__,u32key);

    if(((u32key & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40BE_COMMAND)
        || ((u32key & 0xFFFF0000UL) == CUS22T_SW_IRKEY_40BF_COMMAND)
        )
    {
        u8RetVal = _MDrv_IR_CUS22T_Customer_ParseKey(u32key);
    }
    else
    {
        u8RetVal = _MDrv_IR_CUS22T_Factory_ParseKey(u32key);
    }

    return u8RetVal;
}

static BOOL _MDrv_IR_CUS22T_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
    if(_bIrDetect)
    {
        _bIrDetect = FALSE;

        if(_bIrRepeat)
        {
            if(_u8IrRepeatSkipCnt > 0)
            {
                _u8IrRepeatSkipCnt--;
                _bIrRepeat = FALSE;
                _bRxInProcess = FALSE;
                ResetKeyoffTimer();

                return FALSE;
            }
            else
            {
                *pu8Key = _MDrv_IR_CUS22T_ParseKey(_u32IRKeyMap);//IS_SW_FACTORY_CODE()? IR_FACTORY_KEY: IR_CODE;
                *pu8Flag = _bIrRepeat;
            }
        }
        else
        {
            *pu8Key = _MDrv_IR_CUS22T_ParseKey(_u32IRKeyMap);//IS_SW_FACTORY_CODE()? IR_FACTORY_KEY: IR_CODE;
            *pu8Flag = _bIrRepeat;
            _u8IrRepeatSkipCnt = IR_REPEAT_SKIP_COUNT;
        }

        _bIrRepeat = FALSE;
        _bRxInProcess = FALSE;
        ResetKeyoffTimer();

        return TRUE;
    }
        return FALSE;
}
#endif

#endif

//-------------------------------------------------------------------------------------------------
/// Translate from IR key to internal key.
/// @param  u8KeyData  \b IN: IR key value.
/// @return translated internal IR key.
//-------------------------------------------------------------------------------------------------
#if (IR_TYPE_SEL == IR_TYPE_CUS21SH)
static U8 _MDrv_IR_CUS21SH_ParseKey(U16 u16KeyData)
{
    U8 u8RetVal;
    //printk("Linux: %s:%s:%d u8KeyData:%x\n",__FILE__,__FUNCTION__,__LINE__,u16KeyData);

    switch(u16KeyData)
    {
        //case SH_IRKEY_TV_RADIO        : u8RetVal = IRKEY_TV_RADIO;         break;
        case SH_IRKEY_CHANNEL_LIST    : u8RetVal = IRKEY_CHANNEL_LIST;     break;
        case SH_IRKEY_CHANNEL_FAV_LIST: u8RetVal = IRKEY_CHANNEL_FAV_LIST; break;
        case SH_IRKEY_CHANNEL_RETURN  : u8RetVal = IRKEY_CHANNEL_RETURN;   break;
        case SH_IRKEY_CHANNEL_PLUS    : u8RetVal = IRKEY_CHANNEL_PLUS;     break;
        case SH_IRKEY_CHANNEL_MINUS   : u8RetVal = IRKEY_CHANNEL_MINUS;    break;

        case SH_IRKEY_AUDIO           : u8RetVal = IRKEY_AUDIO;            break;
        case SH_IRKEY_VOLUME_PLUS     : u8RetVal = IRKEY_VOLUME_PLUS;      break;
        case SH_IRKEY_VOLUME_MINUS    : u8RetVal = IRKEY_VOLUME_MINUS;     break;

        case SH_IRKEY_UP              : u8RetVal = IRKEY_UP;               break;
        case SH_IRKEY_POWER           : u8RetVal = IRKEY_POWER;            break;
        case SH_IRKEY_EXIT            : u8RetVal = IRKEY_EXIT;             break;
        case SH_IRKEY_MENU            : u8RetVal = IRKEY_MENU;             break;
        case SH_IRKEY_DOWN            : u8RetVal = IRKEY_DOWN;             break;
        case SH_IRKEY_LEFT            : u8RetVal = IRKEY_LEFT;             break;
        case SH_IRKEY_SELECT          : u8RetVal = IRKEY_SELECT;           break;
        case SH_IRKEY_RIGHT           : u8RetVal = IRKEY_RIGHT;            break;

        case SH_IRKEY_NUM_0           : u8RetVal = IRKEY_NUM_0;            break;
        case SH_IRKEY_NUM_1           : u8RetVal = IRKEY_NUM_1;            break;
        case SH_IRKEY_NUM_2           : u8RetVal = IRKEY_NUM_2;            break;
        case SH_IRKEY_NUM_3           : u8RetVal = IRKEY_NUM_3;            break;
        case SH_IRKEY_NUM_4           : u8RetVal = IRKEY_NUM_4;            break;
        case SH_IRKEY_NUM_5           : u8RetVal = IRKEY_NUM_5;            break;
        case SH_IRKEY_NUM_6           : u8RetVal = IRKEY_NUM_6;            break;
        case SH_IRKEY_NUM_7           : u8RetVal = IRKEY_NUM_7;            break;
        case SH_IRKEY_NUM_8           : u8RetVal = IRKEY_NUM_8;            break;
        case SH_IRKEY_NUM_9           : u8RetVal = IRKEY_NUM_9;            break;

        case SH_IRKEY_MUTE            : u8RetVal = IRKEY_MUTE;             break;
        case SH_IRKEY_PAGE_UP         : u8RetVal = IRKEY_PAGE_UP;          break;
        //case SH_IRKEY_PAGE_DOWN       : u8RetVal = IRKEY_PAGE_DOWN;        break;
        case SH_IRKEY_CLOCK           : u8RetVal = IRKEY_CLOCK;            break;

        case SH_IRKEY_INFO            : u8RetVal = IRKEY_INFO;             break;
        case SH_IRKEY_RED             : u8RetVal = IRKEY_RED;              break;
        case SH_IRKEY_GREEN           : u8RetVal = IRKEY_GREEN;            break;
        case SH_IRKEY_YELLOW          : u8RetVal = IRKEY_YELLOW;           break;
        case SH_IRKEY_BLUE            : u8RetVal = IRKEY_BLUE;             break;
        case SH_IRKEY_MTS             : u8RetVal = IRKEY_MTS;              break;
        //case SH_IRKEY_NINE_LATTICE    : u8RetVal = IRKEY_NINE_LATTICE;     break;
#if defined(DVB_SYSTEM)
        case SH_IRKEY_TTX             : u8RetVal = IRKEY_TTX;              break;
#elif defined(ATSC_SYSTEM)
        case SH_IRKEY_CC              : u8RetVal = IRKEY_CC;               break;
#endif
        case SH_IRKEY_INPUT_SOURCE    : u8RetVal = IRKEY_INPUT_SOURCE;     break;
        //case SH_IRKEY_CRADRD          : u8RetVal = IRKEY_CRADRD;           break;
    //    case SH_IRKEY_PICTURE         : u8RetVal = IRKEY_PICTURE;          break;
        case SH_IRKEY_ZOOM            : u8RetVal = IRKEY_ZOOM;             break;
        //case SH_IRKEY_DASH            : u8RetVal = IRKEY_DASH;             break;
        case SH_IRKEY_SLEEP           : u8RetVal = IRKEY_SLEEP;            break;
        case SH_IRKEY_EPG             : u8RetVal = IRKEY_EPG;              break;
        //case SH_IRKEY_PIP             : u8RetVal = IRKEY_PIP;              break;

          //case SH_IRKEY_MIX             : u8RetVal = IRKEY_MIX;              break;
        //case SH_IRKEY_INDEX           : u8RetVal = IRKEY_INDEX;            break;
        case SH_IRKEY_HOLD            : u8RetVal = IRKEY_HOLD;             break;
        //case SH_IRKEY_PREVIOUS        : u8RetVal = IRKEY_PREVIOUS;         break;
        //case SH_IRKEY_NEXT            : u8RetVal = IRKEY_NEXT;             break;
        //case SH_IRKEY_BACKWARD        : u8RetVal = IRKEY_BACKWARD;         break;
        //case SH_IRKEY_FORWARD         : u8RetVal = IRKEY_FORWARD;          break;
        //case SH_IRKEY_PLAY            : u8RetVal = IRKEY_PLAY;             break;
        //case SH_IRKEY_RECORD          : u8RetVal = IRKEY_RECORD;           break;
        //case SH_IRKEY_STOP            : u8RetVal = IRKEY_STOP;             break;
        //case SH_IRKEY_PAUSE           : u8RetVal = IRKEY_PAUSE;            break;

        case SH_IRKEY_SIZE            : u8RetVal = IRKEY_SIZE;             break;
        case SH_IRKEY_REVEAL          : u8RetVal = IRKEY_REVEAL;           break;
        //case SH_IRKEY_SUBCODE         : u8RetVal = IRKEY_SUBCODE;          break;

        //default                    : u8RetVal = IRKEY_DUMMY;            break;
    }
    return u8RetVal;
}

BOOL _MDrv_SH_IR_CheckKey(void)
{
    U8 j;
    U8 DataPlusExDat0 = 0;
    U8 DataPlusExDat1 = 0;
    U8 u8TableIdx = 0;
    BOOL DataMatched = FALSE;
    BOOL SystemBitOK = FALSE;
    BOOL DetectBitOK = FALSE;




    CurrentTime = _MDrv_IR_GetSystemTime();

    // 1
    if(_u32IRCount == 15)
    {
        // 1->a
        _bExpectedTCome = TRUE;
        _u16IRData[keyDatArrayWTIdx]=_u16IRKeyMap;
        keyDatArrayWTIdx++;
        keyDatArrayWTIdx%=IR_SWDECODE_MODE_BUF_LEN;
    }
    else
    {
        // 1->b
        _bExpectedTCome = FALSE;
        if(_u32IRCount >= 11)
        {
            // 1->b->b->6
            ResetKeyoffTimer();
        }
        // 1->b->a->7
        RxInProcess = FALSE;
        SetToStandbyMode = TRUE;

        _u32IRCount = 0;
        _u16IRKeyMap = 0;

        return FALSE;
    }

    if(ReceivingMode && _bExpectedTCome)
    {
        if(keyDatArrayRDIdx != keyDatArrayWTIdx)
        {
            _u16BufferCurrent=_u16IRData[keyDatArrayRDIdx];

            //printk("IR: %s:%d %x,%x\n",__FUNCTION__,__LINE__,_u16BufferCurrent,keyDatArray[keyDatArrayRDIdx]);
            keyDatArrayRDIdx++;
            keyDatArrayRDIdx%=IR_SWDECODE_MODE_BUF_LEN;

            if(((_u16BufferCurrent&0x1FUL)==0x01UL)||((_u16BufferCurrent&0x1FUL)==0x11UL)||((_u16BufferCurrent&0x1FUL)==0x1EUL))
            {
                SystemBitOK=TRUE;
            }
            else
            {
                 FirstRXFlag=FALSE;
                 _u16BufferPrev=0;
                 _u16BufferCurrent=0;
                 ResetKeyoffTimer();
                 return FALSE;
            }
            if(SystemBitOK)
            {
                SystemBitOK = FALSE;
                if((((_u16BufferCurrent>>13)&0x03UL)==0x01UL)||(((_u16BufferCurrent>>13)&0x03UL)==0x02UL))
                {
                    DetectBitOK=TRUE;
                }
                else
                {
                    FirstRXFlag=FALSE;
                    _u16BufferPrev=0;
                    _u16BufferCurrent=0;
                    ResetKeyoffTimer();
                    return FALSE;
                }
            }
            if(DetectBitOK)
            {
                DetectBitOK=FALSE;
                if(FirstRXFlag==TRUE)
                {
                    if(1)//(_u16BufferCurrent & 0x4000) == 0x0400)
                    {
                        SecondRXFlag=TRUE;
                    }
                    else
                    {
                        FirstRXFlag = FALSE;
                        SecondRXFlag = FALSE;
                        _u16BufferCurrent = 0;
                        _u16BufferPrev = 0;
                    }
                }
                else
                {
                    if((_u16BufferCurrent & 0x4000UL) == 0x0000UL)
                    {
                        FirstRXFlag = TRUE;
                        _u16BufferPrev = _u16BufferCurrent;
                    }
                    _u16BufferCurrent=0;
                    ResetKeyoffTimer();
                }
            }
            if(SecondRXFlag)
            {
                SecondRXFlag=FALSE;

                DataPlusExDat0=(U8)((_u16BufferCurrent&0x1FE0UL)>>5);
                DataPlusExDat1=(U8)((_u16BufferPrev&0x1FE0UL)>>5);
                //printk("## Data0=0x%x,Data1=0x%x,Cur=0x%x,Prev=0x%x(%d,%d)\n",DataPlusExDat0,DataPlusExDat1,_u16BufferCurrent,_u16BufferPrev,keyDatArrayRDIdx,keyDatArrayWTIdx);
                if((U8)DataPlusExDat0==(U8)(~DataPlusExDat1))
                {
                    DataMatched=TRUE;
                }
                if(DataMatched)
                {
                    g_bIrDetect=TRUE;

                    FirstRXFlag=FALSE;

                    //DebugIRCode = _u16BufferPrev;
                    u8TableIdx = _MDrv_IR_CUS21SH_ParseKey(_u16BufferPrev);

                    if(LastKeyData==u8TableIdx)
                    {
                        IsRepeatData=1;
                    }
                    else
                    {
                        IsRepeatData=0;
                    }
                    g_u8Key=u8TableIdx;
                    g_u8Flag=IsRepeatData;
                    LastKeyData=u8TableIdx;
                    _u16BufferCurrent=0;
                    _u16BufferPrev=0;

                    //printf("## Repeat=0x%bx, index=0x%bx,key=0x%x",IsRepeatData,u8Key,IR_CUS21SH_RAWDATA_TABLE[u8TableIdx]);
                    //printf("\n");
                    #if 0//SHA Standby Using Codes
                    if((gStandbyInto==TRUE)&&(g_u8Key!=IRKEY_POWER))
                    {
                        return MSRET_ERROR;
                    }
                    #endif
                    //printk("IR: %s:%d %x,%x\n",__FUNCTION__,__LINE__,g_u8Key,IsRepeatData);
                    return TRUE;
                }
                else
                {
                    _u16BufferPrev=_u16BufferCurrent;
                    _u16BufferCurrent=0;
                    ResetKeyoffTimer();
                }
            }
        }
        else
        {

            if((CurrentTime-_u32KeyOffTimeOutCount)>100)
            {
                //printk("IR: %s:%d (%d ,%d)\n",__FUNCTION__,__LINE__,CurrentTime,_u32KeyOffTimeOutCount);
                SetToStandbyMode=TRUE;
            }
            if(SetToStandbyMode)
            {
                SetToStandbyMode=FALSE;
                ReceivingMode=FALSE;
                keyDatArrayRDIdx=0;
                keyDatArrayWTIdx=0;
                for(j=0;j<10;j++)
                {
                    _u16IRData[j]=0;
                }
                FirstRXFlag=FALSE;
                _u16BufferPrev=0;
                _u16BufferCurrent=0;
                LastKeyData=0xFFUL;
                //ExpectedTCome=TRUE;

                _u32IRCount=0;
                _u16IRKeyMap=0;
                RxInProcess=FALSE;

            }
        }
    }
    return FALSE;
}

//-------------------------------------------------------------------------------------------------
/// Get IR key.
/// @param pu8Key  \b IN: Return IR key value.
/// @param pu8Flag \b IN: Return IR repeat code.
///
/// @return TRUE:  Success
/// @return FALSE: Failure
//-------------------------------------------------------------------------------------------------
static BOOL _MDrv_SH_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
#define SH_IR_REPEAT_THRESHOLD  200
#define SH_IR_REPEAT_THRESHOLD_PLAYBACK  900    //600
#define SH_IR_REPEAT_DELAY  4

    if(_MDrv_SH_IR_CheckKey() == TRUE)//FALSE)
    {
        *pu8Key = g_u8Key;
        *pu8Flag = IsRepeatData;

        if (g_u8Flag)
        {
            U32 u32RepeatThreshold = SH_IR_REPEAT_THRESHOLD;

            if (_MDrv_IR_DiffTimeFromNow(gIRTimeOutCount) < u32RepeatThreshold)
            {//keep press case
                gIRTimeOutCount= _MDrv_IR_GetSystemTime();
                _u8IRRepeateDetect= 1;
                g_ucIrRepeatSkipCnt++;
                if(g_ucIrRepeatSkipCnt < SH_IR_REPEAT_DELAY) //fine tune IR repeat speed
                {//first delay when keep press
                    *pu8Flag = FALSE;
                    //*pu8Key = u8Key;
                    return FALSE;
                }
                else
                {//repeat key case
                    g_ucIrRepeatSkipCnt = (SH_IR_REPEAT_DELAY+1); //to avoid overflow
                    *pu8Flag = g_u8Flag;
                    *pu8Key = g_u8Key;
                    return TRUE;
                }
            }
            else
            {//press frequently case (press-release-press-release...)
                gIRTimeOutCount= _MDrv_IR_GetSystemTime();
                if (0)//_u8IRRepeateDetect == 0)
                {
                    *pu8Flag = FALSE;
                    //*pu8Key = u8Key;
                    return FALSE;
                }
                *pu8Key=g_u8Key;
                *pu8Flag=g_u8Flag;
                _u8IRRepeateDetect = 0;
                g_ucIrRepeatSkipCnt= 0;
                return TRUE;
            }
        }
        else
        {//no repeat key case (maybe first loop of keep press or only press one time)
            gIRTimeOutCount= _MDrv_IR_GetSystemTime();
            *pu8Flag = g_u8Flag;
            *pu8Key = g_u8Key;
            _u8IRRepeateDetect = 0;
            g_ucIrRepeatSkipCnt= 0;
            return TRUE;
        }
    }
    return FALSE;
}
#endif

#if (IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
static U8 _MDrv_IR_ParseKey(U8 u8KeyData)
{
    U8 u8RetVal;

    if ( bIRPass ) return KEYCODE_DUMMY;

    switch(u8KeyData)
    {
    case IRKEY_TV_RADIO        : u8RetVal = KEYCODE_TV_RADIO;         break;
    case IRKEY_CHANNEL_LIST    : u8RetVal = KEYCODE_CHANNEL_LIST;     break;
    case IRKEY_CHANNEL_FAV_LIST: u8RetVal = KEYCODE_CHANNEL_FAV_LIST; break;
    case IRKEY_CHANNEL_RETURN  : u8RetVal = KEYCODE_CHANNEL_RETURN;   break;
    case IRKEY_CHANNEL_PLUS    : u8RetVal = KEYCODE_CHANNEL_PLUS;     break;
    case IRKEY_CHANNEL_MINUS   : u8RetVal = KEYCODE_CHANNEL_MINUS;    break;

    case IRKEY_AUDIO           : u8RetVal = KEYCODE_AUDIO;            break;
    case IRKEY_VOLUME_PLUS     : u8RetVal = KEYCODE_VOLUME_PLUS;      break;
    case IRKEY_VOLUME_MINUS    : u8RetVal = KEYCODE_VOLUME_MINUS;     break;

    case IRKEY_UP              : u8RetVal = KEYCODE_UP;               break;
    case IRKEY_POWER           : u8RetVal = KEYCODE_POWER;            break;
    case IRKEY_EXIT            : u8RetVal = KEYCODE_EXIT;             break;
    case IRKEY_MENU            : u8RetVal = KEYCODE_MENU;             break;
    case IRKEY_DOWN            : u8RetVal = KEYCODE_DOWN;             break;
    case IRKEY_LEFT            : u8RetVal = KEYCODE_LEFT;             break;
    case IRKEY_SELECT          : u8RetVal = KEYCODE_SELECT;           break;
    case IRKEY_RIGHT           : u8RetVal = KEYCODE_RIGHT;            break;

    case IRKEY_NUM_0           : u8RetVal = KEYCODE_NUMERIC_0;        break;
    case IRKEY_NUM_1           : u8RetVal = KEYCODE_NUMERIC_1;        break;
    case IRKEY_NUM_2           : u8RetVal = KEYCODE_NUMERIC_2;        break;
    case IRKEY_NUM_3           : u8RetVal = KEYCODE_NUMERIC_3;        break;
    case IRKEY_NUM_4           : u8RetVal = KEYCODE_NUMERIC_4;        break;
    case IRKEY_NUM_5           : u8RetVal = KEYCODE_NUMERIC_5;        break;
    case IRKEY_NUM_6           : u8RetVal = KEYCODE_NUMERIC_6;        break;
    case IRKEY_NUM_7           : u8RetVal = KEYCODE_NUMERIC_7;        break;
    case IRKEY_NUM_8           : u8RetVal = KEYCODE_NUMERIC_8;        break;
    case IRKEY_NUM_9           : u8RetVal = KEYCODE_NUMERIC_9;        break;

    case IRKEY_MUTE            : u8RetVal = KEYCODE_MUTE;             break;
    case IRKEY_PAGE_UP         : u8RetVal = KEYCODE_PAGE_UP;          break;
    case IRKEY_PAGE_DOWN       : u8RetVal = KEYCODE_PAGE_DOWN;        break;
    case IRKEY_CLOCK           : u8RetVal = KEYCODE_CLOCK;            break;

    case IRKEY_INFO            : u8RetVal = KEYCODE_INFO;             break;
    case IRKEY_RED             : u8RetVal = KEYCODE_RED;              break;
    case IRKEY_GREEN           : u8RetVal = KEYCODE_GREEN;            break;
    case IRKEY_YELLOW          : u8RetVal = KEYCODE_YELLOW;           break;
    case IRKEY_BLUE            : u8RetVal = KEYCODE_BLUE;             break;
    case IRKEY_MTS             : u8RetVal = KEYCODE_MTS;              break;
    case IRKEY_NINE_LATTICE    : u8RetVal = KEYCODE_NINE_LATTICE;     break;
#if defined(DVB_SYSTEM)
    case IRKEY_TTX             : u8RetVal = KEYCODE_TTX;              break;
#elif defined(ATSC_SYSTEM)
    case IRKEY_CC              : u8RetVal = KEYCODE_CC;               break;
#endif
    case IRKEY_INPUT_SOURCE    : u8RetVal = KEYCODE_INPUT_SOURCE;     break;
    case IRKEY_CRADRD          : u8RetVal = KEYCODE_CRADRD;           break;
//    case IRKEY_PICTURE         : u8RetVal = KEYCODE_PICTURE;          break;
    case IRKEY_ZOOM            : u8RetVal = KEYCODE_ZOOM;             break;
    case IRKEY_DASH            : u8RetVal = KEYCODE_DASH;             break;
    case IRKEY_SLEEP           : u8RetVal = KEYCODE_SLEEP;            break;
    case IRKEY_EPG             : u8RetVal = KEYCODE_EPG;              break;
    case IRKEY_PIP             : u8RetVal = KEYCODE_PIP;              break;

      case IRKEY_MIX             : u8RetVal = KEYCODE_MIX;              break;
    case IRKEY_INDEX           : u8RetVal = KEYCODE_INDEX;            break;
    case IRKEY_HOLD            : u8RetVal = KEYCODE_HOLD;             break;
    case IRKEY_PREVIOUS        : u8RetVal = KEYCODE_PREVIOUS;         break;
    case IRKEY_NEXT            : u8RetVal = KEYCODE_NEXT;             break;
    case IRKEY_BACKWARD        : u8RetVal = KEYCODE_BACKWARD;         break;
    case IRKEY_FORWARD         : u8RetVal = KEYCODE_FORWARD;          break;
    case IRKEY_PLAY            : u8RetVal = KEYCODE_PLAY;             break;
    case IRKEY_RECORD          : u8RetVal = KEYCODE_RECORD;           break;
    case IRKEY_STOP            : u8RetVal = KEYCODE_STOP;             break;
    case IRKEY_PAUSE           : u8RetVal = KEYCODE_PAUSE;            break;

    case IRKEY_SIZE            : u8RetVal = KEYCODE_SIZE;             break;
    case IRKEY_REVEAL          : u8RetVal = KEYCODE_REVEAL;           break;
    case IRKEY_SUBCODE         : u8RetVal = KEYCODE_SUBCODE;          break;

    default                    : u8RetVal = KEYCODE_DUMMY;            break;
    }

    return u8RetVal;
}
#endif

//-------------------------------------------------------------------------------------------------
/// Set the timing of IrDa at BOOT stage.
/// @return None
//-------------------------------------------------------------------------------------------------
static void _MDrv_IR_Timing(void)
{
    // header code upper bound
    REG(REG_IR_HDC_UPB) = IR_HDC_UPB;

    // header code lower bound
    REG(REG_IR_HDC_LOB) = IR_HDC_LOB;

    // off code upper bound
    REG(REG_IR_OFC_UPB) = IR_OFC_UPB;

    // off code lower bound
    REG(REG_IR_OFC_LOB) = IR_OFC_LOB;

    // off code repeat upper bound
    REG(REG_IR_OFC_RP_UPB) = IR_OFC_RP_UPB;

    // off code repeat lower bound
    REG(REG_IR_OFC_RP_LOB) = IR_OFC_RP_LOB;

    // logical 0/1 high upper bound
    REG(REG_IR_LG01H_UPB) = IR_LG01H_UPB;

    // logical 0/1 high lower bound
    REG(REG_IR_LG01H_LOB) = IR_LG01H_LOB;

    // logical 0 upper bound
    REG(REG_IR_LG0_UPB) = IR_LG0_UPB;

    // logical 0 lower bound
    REG(REG_IR_LG0_LOB) = IR_LG0_LOB;

    // logical 1 upper bound
    REG(REG_IR_LG1_UPB) = IR_LG1_UPB;

    // logical 1 lower bound
    REG(REG_IR_LG1_LOB) = IR_LG1_LOB;

    // timeout cycles
    REG(REG_IR_TIMEOUT_CYC_L) = IR_RP_TIMEOUT & 0xFFFFUL;

    #if(IR_MODE_SEL == IR_TYPE_HWRC_MODE)
    REG(REG_IR_RC_LONGPULSE_THR) = IR_RC_LONGPULSE_THR;
    REG(REG_IR_RC_LONGPULSE_MAR) = IR_RC_LONGPULSE_MAR;
    REG(REG_IR_RC_CLK_INT_THR) = (IR_RC_INTG_THR(IR_RC_INTG_THR_TIME)<<1) + (IR_RC_CLK_DIV(IR_CLK)<<8) + IR_RC6_ECO_EN;
    REG(REG_IR_RC_WD_TIMEOUT_CNT) = IR_RC_WDOG_CNT(IR_RC_WDOG_TIME) + (IR_RC_TMOUT_CNT(IR_RC_TIMEOUT_TIME)<<8);
    REG(REG_IR_TIMEOUT_CYC_H_CODE_BYTE) = (IR_RC5_BITS<<8) | 0x30UL;//[6:4]=011 : timeout be clear at Key Data Code check pass
    #else
    //set up ccode bytes and code bytes/bits num
    REG(REG_IR_TIMEOUT_CYC_H_CODE_BYTE) = IR_CCB_CB | 0x30UL | ((IR_RP_TIMEOUT >> 16) & 0x0FUL);
    #endif
    #ifdef SUPPORT_MULTI_PROTOCOL
    REG(REG_IR_CKDIV_NUM_KEY_DATA) = IR_CKDIV_NUM-1;   // clock divider
    #else
    REG(REG_IR_CKDIV_NUM_KEY_DATA) = IR_CKDIV_NUM;   // clock divider
    #endif
}

unsigned long _MDrv_IR_GetSystemTime(void)
{
    return((unsigned long)((jiffies)*(1000/HZ)));
    //return 0;
}

#if((IR_MODE_SEL==IR_TYPE_RAWDATA_MODE) || (IR_MODE_SEL==IR_TYPE_FULLDECODE_MODE)||IR_MODE_SEL==IR_TYPE_HWRC_MODE || (IR_SWFIFO_DECODE_MODE==ENABLE))
static void _MDrv_IR_ClearFIFO(void)
{
    unsigned long i;
    unsigned long fifo_status_addr,fifo_read_pulse_addr,fifo_data_addr;
    unsigned long fifo_empty_flag;

    if((IR_MODE_SEL==IR_TYPE_RAWDATA_MODE) || (IR_MODE_SEL==IR_TYPE_FULLDECODE_MODE))
    {
        fifo_status_addr = REG_IR_SHOT_CNT_H_FIFO_STATUS;
        fifo_read_pulse_addr = REG_IR_FIFO_RD_PULSE;
        fifo_data_addr = REG_IR_CKDIV_NUM_KEY_DATA;
        fifo_empty_flag = IR_FIFO_EMPTY;
    }
    else
    {
        fifo_status_addr = REG_IR_RC_KEY_FIFO_STATUS;
        fifo_read_pulse_addr = REG_IR_RC_FIFO_RD_PULSE;
        fifo_data_addr = REG_IR_RC_KEY_COMMAND_ADD;
        fifo_empty_flag = IR_RC_FIFO_EMPTY;
    }

    // Empty the FIFO
    for(i=0; i<16; i++)  //FIFO maxinum depth is 16 bytes(typically, SW decode mode will use 16bytes)
    {
        U8 u8Garbage;

        if(REG(fifo_status_addr) & fifo_empty_flag)
            break;

        u8Garbage = REG(fifo_data_addr) >> 8;
        REG(fifo_read_pulse_addr) |= 0x0001UL; //read
    }

}
#endif

#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
#define MaxQueue 100
static struct queue
{
    u32 item[MaxQueue];
    int front;
    int rear;
} q;

static DEFINE_SEMAPHORE(queue_lock);
void enqueue(u32 data) {
    if (down_trylock(&queue_lock))
       return;

    if (q.rear == ((q.front + 1) % MaxQueue)) {
        DEBUG_IR_INPUT_DEVICE(printk("queue is full\n"));
    }  else {
        q.rear = (q.rear + 1) % MaxQueue;
        q.item[q.rear] = data;
    }

    up(&queue_lock);
}

u32 dequeue(void) {
    u32 data = 0xFFFFUL;

    down(&queue_lock);

    if (q.front == q.rear) {
        DEBUG_IR_INPUT_DEVICE(printk("queue is empty\n"));
    } else {
        q.front = (q.front + 1) % MaxQueue;
        data = q.item[q.front];
    }

    up(&queue_lock);
    return data;
}

int key_dispatch(void *p) {
    static u32 prev_scancode = 0xFFFFUL;

    DEBUG_IR_INPUT_DEVICE(printk("[IR] key_dispatch thread start\n"));
    //daemonize("ir_key_dispatch");
    while(1) {
        int ret;
        u32 scancode;

        try_to_freeze();

        DEBUG_IR_INPUT_DEVICE(printk("xxxxx 0\n"));
        if (prev_scancode == 0xFFFFUL) {
            DEBUG_IR_INPUT_DEVICE(printk("xxxxx 1\n"));
            ret = wait_for_completion_interruptible(&key_completion);
        } else {
            DEBUG_IR_INPUT_DEVICE(printk("xxxxx 2\n"));
            // Depend on different IR to wait timeout.
            // or IR_TYPE_MSTAR_DTV, 160 is better, because ISR need such time to get another ir key.
            //
            // NOTE:
            // Too small, you will find the repeat function in android don't work. (up immediately)
            // It will become down->up->down->down.....(not continue down)
            // In input driver(2.6.35), over REP_DELAY(250 msecs) will auto-repeat, and every REP_PERIOD(33 msecs) will send repeat key.
            // In input driver(3.0.20), over REP_DELAY(500 msecs) will auto-repeat, and every REP_PERIOD(125 msecs) will send repeat key.
            // In android, over DEFAULT_LONG_PRESS_TIMEOUT(500 mesc) will auto-repeat, and every KEY_REPEAT_DELAY(50 mesc) will send repeat key.
            ret = wait_for_completion_interruptible_timeout(&key_completion, msecs_to_jiffies(IR_EVENT_TIMEOUT));
        }
        if (ret < 0) {
            DEBUG_IR_INPUT_DEVICE(printk("completion interruptible\n"));
            continue;
        }

        scancode = dequeue();
        if ((prev_scancode != 0xFFFFUL) && (scancode == 0xFFFFUL)) {
            DEBUG_IR_INPUT_DEVICE(printk("xxxxx 3\n"));
            rc_keyup(ir->dev);
        } else if ((prev_scancode != 0xFFFFUL) && (scancode != 0xFFFFUL)) {
           DEBUG_IR_INPUT_DEVICE(printk("xxxxx 4\n"));

           if ((scancode != prev_scancode)) {
               DEBUG_IR_INPUT_DEVICE(printk("xxxxx 5\n"));
               rc_keyup(ir->dev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
               rc_keydown_notimeout(ir->dev, RC_TYPE_NEC, scancode, 0);
#else
               rc_keydown_notimeout(ir->dev, scancode, 0);
#endif
           }
        } else if ((prev_scancode == 0xFFFFUL) && (scancode != 0xFFFFUL)) {
            DEBUG_IR_INPUT_DEVICE(printk("xxxxx 6\n"));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
            rc_keydown_notimeout(ir->dev, RC_TYPE_NEC, scancode, 0);
#else
            rc_keydown_notimeout(ir->dev, scancode, 0);
#endif
        }

        DEBUG_IR_INPUT_DEVICE(printk("xxxxx 7, scancode=%d\n", scancode));
        prev_scancode = scancode;
    }

    DEBUG_IR_INPUT_DEVICE(printk("[IR] key_dispatch thread end\n"));
    return 0;
}
#endif // CONFIG_MSTAR_IR_INPUT_DEVICE

#if defined(CONFIG_MSTAR_GPIO) && defined(CONFIG_MSTAR_IR_GPIO_TOGGLE)
static struct workqueue_struct *gpio_toggle_workqueue;
int gpio_toggle(void *p)
{
    //daemonize("ir_key_dispatch");
    while(1)
    {
        unsigned int gpio_ir = MDrv_SYS_GetGPIOIR();
        wait_for_completion_interruptible(&ir_gpio_completion);
        if (gpio_ir != 0)
        {
            if (MDrv_SYS_GetGPIOIRType() == 1) // High active
            {
                MDrv_GPIO_Set_High(gpio_ir);
                msleep(120);
                MDrv_GPIO_Set_Low(gpio_ir);
            }
            else
            {
                MDrv_GPIO_Set_Low(gpio_ir);
                msleep(120);
                MDrv_GPIO_Set_High(gpio_ir);
            }
        }
    }

    return 0;
}

static DECLARE_WORK(gpio_toggle_thread, gpio_toggle);
void gpio_toggle_trigger(void)
{
    complete(&ir_gpio_completion);
}
#endif



#ifdef CONFIG_MSTAR_PM_SWIR
static int swir_pm_status = 0;
#define IR_MBX_TIMEOUT 1500
#define IR_MBX_QUEUESIZE        8

typedef enum
{
    /// mbx command for initialization
    E_IR_CPUTo51_CMD_INIT=0x00,
    /// mbx command for configuration
    E_IR_CPUTo51_CMD_CONFIG,
    /// mbx command for key code
    E_IR_CPUTo51_CMD_KEYCODE,
    /// mbx command for set callback
    E_IR_CPUTo51_CMD_SETCLBK,
    /// mbx command for library version
    E_IR_CPUTo51_CMD_LIBVER,
    /// mbx command for status
    E_IR_CPUTo51_CMD_STATUS,
    /// mbx command for enable
    E_IR_CPUTo51_CMD_ENABLE,
    /// mbx command for driver info
    E_IR_CPUTo51_CMD_INFO,
    /// mbx command for sw init
    E_IR_CPUTo51_CMD_SWIR_INT,
    /// mbx command for sw ir key code
    E_IR_CPUTo51_CMD_SWIR_KEYCODE,
} IR_CPUTo51CmdIdx;

/// emurate IR mailbox commands ack from mcu51 to cpu
typedef enum
{
    //(1) Acknowledge from MCU51
    /// ack mbx command for initialization
    E_IR_51ToCPU_CMD_ACK_INIT=0x00,
    /// ack mbx command for configuration
    E_IR_51ToCPU_CMD_ACK_CONFIG,
    /// ack mbx command for key code
    E_IR_51ToCPU_CMD_ACK_KEYCODE,
    /// ack mbx command for set callback
    E_IR_51ToCPU_CMD_ACK_SETCLBK,
    /// ack mbx command for library version
    E_IR_51ToCPU_CMD_ACK_LIBVER,
    ///ack mbx command for status
    E_IR_51ToCPU_CMD_ACK_STATUS,
    ///ack mbx command for enable
    E_IR_51ToCPU_CMD_ACK_ENABLE,
    ///ack mbx command for driver info
    E_IR_51ToCPU_CMD_ACK_INFO,

    //(2) Notification from MCU51
    ///notification mbx command for key code
    E_IR_51ToCPU_CMD_KEYCODE,

} IR_51ToCPUCmdIdx;

/// emurate ack flags
typedef enum
{
    /// ack flag for null
    E_IR_ACKFLG_NULL         = 0,
    /// ack flag for wait initialization
    E_IR_ACKFLG_WAIT_INIT    = (1<<0),
    /// ack flag for wait configuration
    E_IR_ACKFLG_WAIT_CONFIG  = (1<<1),
    /// ack flag for wait key code
    E_IR_ACKFLG_WAIT_KEYCODE = (1<<2),
    /// ack flag for wait set callback
    E_IR_ACKFLG_WAIT_SETCLBK = (1<<3),
    /// ack flag for wait library version
    E_IR_ACKFLG_WAIT_LIBVER  = (1<<4),
    /// ack flag for wait status
    E_IR_ACKFLG_WAIT_STATUS  = (1<<5),
    /// ack flag for wait enable
    E_IR_ACKFLG_WAIT_ENABLE  = (1<<6),
    /// ack flag for wait driver info
    E_IR_ACKFLG_WAIT_INFO    = (1<<7),
} IR_AckFlags;

typedef enum
{
    /// IR result for failure
    E_IR_FAIL =0,
    /// IR result for OK
    E_IR_OK = 1,

} IR_Result;

typedef struct
{
    //MS_U8 u8ModeSel;          /// IR mode selection
    MS_U8 u8Ctrl0;          /// IR enable control 0
    MS_U8 u8Ctrl1;          /// IR enable control 1
    MS_U8 u8Clk_mhz;        /// IR required clock
    MS_U8 u8HdrCode0;       /// IR Header code 0
    MS_U8 u8HdrCode1;       /// IR Header code 1
    MS_U8 u8CCodeBytes;     /// Customer codes: 1 or 2 bytes
    MS_U8 u8CodeBits;       /// Code bits: 0x00~0x7F
    MS_U8 u8HdrCode20;      /// 2nd IR Header code 0
    MS_U8 u8HdrCode21;      /// 2nd IR Header code 1

} IR_RegCfg;

static IR_AckFlags gIRAckFlags;
static MBX_Result gMBXResult;
static IR_Result gIRResult;

static void _MDrv_IR_MailBoxHandler(MBX_Class eMsgClass, MBX_MSGQ_Status *pMsgQueueStatus)
{
    MBX_Msg MB_Command;

    memset((void*)&MB_Command, 0, sizeof(MBX_Msg));
    gMBXResult = MDrv_MBX_RecvMsg_Async(SWIR_FILEOPS, eMsgClass, &MB_Command, IR_MBX_TIMEOUT/*ms*/, MBX_CHECK_NORMAL_MSG);
    if (gMBXResult == E_MBX_ERR_TIME_OUT)
    {
        printk(KERN_EMERG "[IR] Handler Timeout!\n");
        return;
    }

    if (gMBXResult == E_MBX_SUCCESS)
    {
        if ((MB_Command.u8Ctrl != 0) && (MB_Command.u8Ctrl != 1))
        {
            gIRResult = E_IR_FAIL;
            printk(KERN_EMERG "[IR] Not Implemented!\n");
            return;
        }
        //printk(KERN_EMERG "Get IR command: 0x%02x.\n", MB_Command.u8Index);
        //printk(KERN_EMERG "Parameter[0]=%d\n",  MB_Command.u8Parameters[0]);
        gIRResult = E_IR_FAIL;
        switch (MB_Command.u8Index)
        {
            case E_IR_51ToCPU_CMD_ACK_INIT:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_INIT);
                break;
            case E_IR_51ToCPU_CMD_ACK_CONFIG:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_CONFIG);
                break;
            case E_IR_51ToCPU_CMD_ACK_KEYCODE:
                gIRResult = E_IR_OK;
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_KEYCODE);
                break;
            case E_IR_51ToCPU_CMD_ACK_SETCLBK:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_SETCLBK);
                break;
            case E_IR_51ToCPU_CMD_ACK_LIBVER:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_LIBVER);
                break;
            case E_IR_51ToCPU_CMD_ACK_STATUS:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_STATUS);
                break;
            case E_IR_51ToCPU_CMD_ACK_ENABLE:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_ENABLE);
                break;
            case E_IR_51ToCPU_CMD_ACK_INFO:
                gIRResult = (IR_Result)MB_Command.u8Parameters[0];
                gIRAckFlags &= (IR_AckFlags)(~E_IR_ACKFLG_WAIT_INFO);
                break;
            default:
                break;
        }

    }

    return;

}

int MDrv_PMSWIR_ReceiveMsg(void)
{
    MBX_MSGQ_Status MbxMsgQStatus;

    _MDrv_IR_MailBoxHandler(E_MBX_CLASS_IRKEY_NOWAIT, &MbxMsgQStatus);

    return TRUE;
}
#endif

#ifdef SUPPORT_MULTI_PROTOCOL
inline static void _MDrv_MultiProtocol_RestartTimer(void)
{
    timer_pending(&_key_MultiProtocol_timer);
    mod_timer(&_key_MultiProtocol_timer, jiffies + msecs_to_jiffies(50));
}

inline static void _MDrv_MultiProtocol_StartTimer(void)
{
    DRV_IR_PROTOCOL_TYPE *pstProtocol;
    pstProtocol = _GetProtocolEntry(_eCurentProtocol);
    timer_pending(&_key_MultiProtocol_timer);
    mod_timer(&_key_MultiProtocol_timer, jiffies + msecs_to_jiffies(pstProtocol->u32Timeout/100));
}

static void _MDrv_MultiProtocol_Cb(unsigned long params)
{
    if(_u64LastData==0||_u64LastData&E_IR_KEY_STATE_RELEASE)
    {
        return;
    }

    while(down_trylock(&fantasy_protocol.sem)!=0)
    {
        _MDrv_MultiProtocol_RestartTimer();
    }

    if (fantasy_protocol.data != 0)
    {
        _MDrv_MultiProtocol_RestartTimer();
    }
    else
    {
        _u64LastData = ((_u64LastData>>4)<<4)|E_IR_KEY_STATE_RELEASE;
        fantasy_protocol.data = _u64LastData;
    }
    up(&fantasy_protocol.sem);
    wake_up_interruptible(&fantasy_protocol.inq);

}

static void _MDrv_MultiProtocol_Init(void)
{
   init_timer(&_key_MultiProtocol_timer);
   _key_MultiProtocol_timer.expires = jiffies + msecs_to_jiffies(0xFFFFFFFFUL);
   _key_MultiProtocol_timer.function = _MDrv_MultiProtocol_Cb;
   add_timer(&_key_MultiProtocol_timer);
}
#endif

#ifdef SUPPORT_MULTI_PROTOCOL
static IR_PROCOCOL_TYPE _MDrv_MultiProtocol_GetProByLeadCode(void)
{
    U8 u8Index;
    U8 u8StartIdx=0;
    BOOL bResult = FALSE;
    IR_PROCOCOL_TYPE eTemp=E_IR_PROTOCOL_NONE;
    DRV_IR_PROTOCOL_TYPE *pstProtocol;

    for(u8Index=0; u8Index<PROTOCOL_SUPPORT_MAX; u8Index++)
    {
        if(_eDetectList[u8Index]==E_IR_PROTOCOL_NONE)
            continue;

        pstProtocol = _GetProtocolEntry(_eDetectList[u8Index]);
        bResult = pstProtocol->findleadcode(&u8StartIdx);
        if(bResult==TRUE)
        {
            eTemp = _eDetectList[u8Index];
            if(u8Index!=0 && _eDetectList[0]!=E_IR_PROTOCOL_NONE)
            {//switch newest protocol to list head
                _eDetectList[u8Index] = _eDetectList[0];
                _eDetectList[0] = eTemp;
            }
            return eTemp;//return detect result
        }
    }

    return E_IR_PROTOCOL_NONE;
}

static IR_DECODE_STATUS _MDrv_MultiProtocol_ParseShotCount(IR_PROCOCOL_TYPE eProtocol, unsigned long long *pu64RawData)
{
    IR_DECODE_STATUS eResult = E_IR_DECODE_ERR;
    DRV_IR_PROTOCOL_TYPE *pstProtocol;
    U32 u32CustCode =0;
    U16 u16KeyCode = 0;
    U8 u8State = 0;
    U8 u8Reserved = 0;

    if(eProtocol>E_IR_PROTOCOL_NONE && eProtocol<E_IR_PROTOCOL_MAX)
    {
        pstProtocol = _GetProtocolEntry(eProtocol);
        eResult = pstProtocol->parseprotocol(&u32CustCode, &u16KeyCode, &u8State, &u8Reserved);
    }

    if(eResult ==E_IR_DECODE_DATA_OK)
    {
        *pu64RawData = (((unsigned long long)u8Reserved)<<56)|(((unsigned long long)u32CustCode)<<24) | (u16KeyCode<<8) | u8State;
    }
    return eResult;
}

static U8 _MDrv_MultiProtocol_GetMaxLeadCodeShot(void)
{
    U8 u8Max=0;
    U8 i;
    IR_PROCOCOL_TYPE eProtocol;
    DRV_IR_PROTOCOL_TYPE *pstProtocol;

    for(i=0; i<PROTOCOL_SUPPORT_MAX; i++)
    {
        eProtocol =_eDetectList[i];
        if(eProtocol!= E_IR_PROTOCOL_NONE)
        {
            pstProtocol = _GetProtocolEntry(eProtocol);
            if(pstProtocol->u8LeadCodeMinCount > u8Max)
            {
                u8Max =pstProtocol->u8LeadCodeMinCount;
            }
        }
    }

    return u8Max;
}
#endif

#ifdef CONFIG_MSTAR_DYNAMIC_IR
u8 translate_scancode(u16 tempScancode)
{
#if (IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
    //printk("%s: 0x%x ----> 0x%x\n", __FUNCTION__, tempScancode, scan2key_map[0][tempScancode]);
    return scan2key_map[0][tempScancode&0xFF];
#else
    int ir_index = (tempScancode>>8)&0x0F;
    return scan2key_map[ir_index][tempScancode&0xFF];
#endif
}
#endif

//-------------------------------------------------------------------------------------------------
/// ISR when receive IR key.
/// @return None
//-------------------------------------------------------------------------------------------------
#if (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
irqreturn_t _MDrv_IR_RC_ISR(int irq, void *dev_id) {

    U8 u8Key=0, u8RepeatFlag=0;
    U8 u8System = 0;
    BOOL bHaveKey = FALSE;

    if (bHaveKey = _MDrv_IR_GetKey(&u8Key, &u8System, &u8RepeatFlag)) {
        if (down_trylock(&fantasy_protocol.sem) == 0) {
            if (fantasy_protocol.data == 0) {
                fantasy_protocol.data = (u8Key<<8 | u8RepeatFlag);
                fantasy_protocol.data |= (0x01UL << 28);
            }

            up(&fantasy_protocol.sem);
            wake_up_interruptible(&fantasy_protocol.inq);
        }
    }
    //printk("HaveKey=%d, KEY=%d, RepeatFalg=%d\n", bHaveKey, u8Key, u8RepeatFlag);

    return IRQ_HANDLED;
}

#else
irqreturn_t _MDrv_IR_ISR(int irq, void *dev_id) {
#if (IR_SWFIFO_DECODE_MODE==ENABLE)
    while ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & IR_FIFO_EMPTY) != IR_FIFO_EMPTY)
    {
        MDrv_IR_ISRParseKey();
        REG(REG_IR_FIFO_RD_PULSE) |= 0x0001;
    }
    return IRQ_HANDLED;
#else
    return MDrv_IR_ISRParseKey();
#endif
}

static irqreturn_t  MDrv_IR_ISRParseKey(void)
{
    U8 u8Key=0, u8RepeatFlag=0;
    U8 u8System = 0;

#ifdef SUPPORT_MULTI_PROTOCOL
    U32 u32ShotValue;
    BOOL bNegShot;
    unsigned long long u64Data=0;
    DRV_IR_PROTOCOL_TYPE *pstProtocol;

    pstProtocol = _GetProtocolEntry(_eCurentProtocol);
    if (_MDrv_IR_GetSystemTime() - _ulPreShotTime> pstProtocol->u32Timeout/1000)
    {
        _Mdrv_MulProtCommon_ShotDataReset();
        _eCurentProtocol=E_IR_PROTOCOL_NONE;
        _eLastKeyProtocol= E_IR_PROTOCOL_NONE;
        _u64LastData=0;
    }

    u32ShotValue = ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)&0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
    bNegShot = (REG(REG_IR_SHOT_CNT_H_FIFO_STATUS))&0x10UL?FALSE:TRUE;
    if(_MulProtCommon_GetShotDataSize()!=0)
    {
        _Mdrv_MulProtCommon_AddShot(u32ShotValue, bNegShot);
    }
    else
    {
        if(bNegShot==FALSE)//First shot must be p-shot
        {
            _Mdrv_MulProtCommon_AddShot(u32ShotValue, bNegShot);
        }
    }

    if(_eCurentProtocol== E_IR_PROTOCOL_NONE &&
        _MulProtCommon_GetShotDataSize() >= _MDrv_MultiProtocol_GetMaxLeadCodeShot())
    {
        _eCurentProtocol = _MDrv_MultiProtocol_GetProByLeadCode();
        if(_eCurentProtocol==E_IR_PROTOCOL_NONE)//fail
        {
            //printk("parse lead code fail\n");
            //_MulProtCommon_dumpShotData();
            _Mdrv_MulProtCommon_ShotDataReset();//reset all data
        }
    }

    if(_eCurentProtocol!=E_IR_PROTOCOL_NONE)
    {
        IR_DECODE_STATUS eResult = _MDrv_MultiProtocol_ParseShotCount(_eCurentProtocol, &u64Data);
        if(eResult==E_IR_DECODE_ERR)
        {
            //printk("parse error, _eCurentProtocol=%d\n", _eCurentProtocol);
            //_MulProtCommon_dumpShotData();
            _eLastKeyProtocol=E_IR_PROTOCOL_NONE;
            _Mdrv_MulProtCommon_ShotDataReset();
        }
        else if(eResult == E_IR_DECODE_DATA_SHORTAGE)
        {
            //continue to get next shot
        }
        else if(eResult == E_IR_DECODE_DATA_OK)
        {
            //printk("parse shot count ok, u64RawData=0x%016llx\n", u64Data);
            //_MulProtCommon_dumpShotData();
            _Mdrv_MulProtCommon_ShotDataReset();
            _eLastKeyProtocol= _eCurentProtocol;
            while(down_trylock(&fantasy_protocol.sem)!=0)
            {
                printk("busy wait semaphore\n");//loop if can't get semaphore
            }

            if(fantasy_protocol.data!=0)
            {
                printk("unprocessed fantasy protocol data!!!!\n");
            }
            else
            {
                _u64LastData = u64Data;
                fantasy_protocol.data = u64Data;
                _MDrv_MultiProtocol_StartTimer();
            }
            up(&fantasy_protocol.sem);
            wake_up_interruptible(&fantasy_protocol.inq);
        }
    }

    _ulPreShotTime = _MDrv_IR_GetSystemTime();
    return IRQ_HANDLED;

#endif

#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
    #if(IR_TYPE_SEL == IR_TYPE_RCMM)
    U16 u16IrCounter;
    static unsigned long PreTime;
    u16IrCounter = ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)&0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
    if ((_MDrv_IR_GetSystemTime() - PreTime > IR_TIMEOUT_CYC/1000))//reset
    {
        RCBitsCnt = 0;
        _u8PrevKeyCode = 0xffUL;
    }
    //i++;
    //if(i % 2)
       // printk("%d:%d\n",i,u16IrCounter);
    //return IRQ_HANDLED;

    if(P25_MIN < u16IrCounter && u16IrCounter < P25_MAX)
    {
        tgbits = 0x00UL;
        RCByte[0] = 0x00UL;
        RCByte[1] = 0x00UL;
        RCByte[2] = 0x00UL;
        RCByte[3] = 0x00UL;
        RCBitsCnt = 0;
        RCMode = 0;

        StartDecodeFlag = TRUE;
        //printk("START\n");
        //head code start
    }
    else if( (P16_MIN < u16IrCounter && u16IrCounter < P16_MAX) && StartDecodeFlag) //! it is 00 bit sequence
    {
        tgbits = 0x00UL;
        //printk("00\n");
        //_u32IrRcmmData <<= 2;
        //_u32IrRcmmData |= tgbits;
        RCByte[RCBitsCnt>>3] <<= 2;
        RCByte[RCBitsCnt>>3] |= tgbits;
        RCBitsCnt += 2;
    }
    else if( (P22_MIN < u16IrCounter && u16IrCounter< P22_MAX) && StartDecodeFlag) //! it is 01 bit sequence
    {
        tgbits = 0x01UL;
        //printk("01\n");
        //_u32IrRcmmData <<= 2;
        //_u32IrRcmmData |= tgbits;
        RCByte[RCBitsCnt>>3] <<= 2;
        RCByte[RCBitsCnt>>3] |= tgbits;
        RCBitsCnt += 2;
    }
    else if( (P28_MIN < u16IrCounter && u16IrCounter < P28_MAX) && StartDecodeFlag) //! it is 10 bit sequence
    {
        tgbits = 0x02UL;
        //printk("10\n");
        //_u32IrRcmmData <<= 2;
        //_u32IrRcmmData |= tgbits;
        RCByte[RCBitsCnt>>3] <<= 2;
        RCByte[RCBitsCnt>>3] |= tgbits;
        RCBitsCnt += 2;

    }
    else if( (P34_MIN < u16IrCounter && u16IrCounter < P34_MAX) && StartDecodeFlag) //! it is 11 bit sequence
    {
        tgbits = 0x03UL;
        //printk("11\n");
        //_u32IrRcmmData <<= 2;
        //_u32IrRcmmData |= tgbits;
        RCByte[RCBitsCnt>>3] <<= 2;
        RCByte[RCBitsCnt>>3] |= tgbits;
        RCBitsCnt += 2;
    }
    else
    {
        StartDecodeFlag = FALSE;
        RCBitsCnt = 0;
        UpDataFlage = FALSE;
        tgbits = 0x00UL;
        RCByte[0] = 0x00UL;
        RCByte[1] = 0x00UL;
        RCByte[2] = 0x00UL;
        RCByte[3] = 0x00UL;
    }

    /*
    if( RCBitsCnt == 12)
    {
        printk("12\n");
        if(RCByte[0] & 0xC0)
        {
            RCMode = RCMMBASIC_MODE;

            tgbits = 1 << ((RCByte[0]&0xC0) >> 6);
            if(tgbits == RC_MODE)
            {
                RCBitsCnt = 0;
                u16CustomerID = 0;
                RCMode = 0;
                UpDataFlage = FALSE;
                tgbits = 0x00;
                RCByte[0] = 0x00;
                RCByte[1] = 0x00;
                RCByte[2] = 0x00;
                RCByte[3] = 0x00;

            }

            RCMode |= tgbits; //BASIC_Mouse, //BASIC_keyboard, //BASIC_joystick
            UpDataFlage = TRUE;
            //SaveRCMMIRData();
        }
    }*/
    if(RCBitsCnt == 24)
    {
        if(RCByte[0] & 0x20UL)
        {
            RCMode |= RCMMOEM_LONGID_MODE;

            tgbits = (RCByte[1]&0x0CUL) >> 2;
            RCMode |= 1<<tgbits; //OEM_LONGID_RC, //OEM_LONGID_Mouse, //OEM_LONGID_keyboard, //OEM_LONGID_joystick
        }
        else if( ((RCByte[0]&0xFCUL)>>2) == 0x03UL )
        {
            RCMode |= RCMMOEM_SHORTID_MODE;

            tgbits = (RCByte[1]&0x0CUL) >> 2;
            RCMode |= 1<<tgbits; //OEM_SHORTID_RC, //OEM_SHORTID_Mouse, //OEM_SHORTID_keyboard, //OEM_SHORTID_joystick
        }

        if( (RCMode & MOUSE_MODE) || (RCMode & KEYBOARD_MODE) )
        {
            StartDecodeFlag = FALSE;
            UpDataFlage = TRUE;
        }

    }
    else if(RCBitsCnt >= 32)
    {
        if( (RCMode & RC_MODE) || (RCMode & JOYSTICK_MODE) )
        {
            StartDecodeFlag = FALSE;
            UpDataFlage = TRUE;
        }
        else
        {
            RCBitsCnt = 0;
            u16CustomerID = 0;
            RCMode = 0;
            UpDataFlage = FALSE;
            tgbits = 0x00UL;
            RCByte[0] = 0x00UL;
            RCByte[1] = 0x00UL;
            RCByte[2] = 0x00UL;
            RCByte[3] = 0x00UL;
        }
    }
    PreTime = _MDrv_IR_GetSystemTime();
    #elif (IR_TYPE_SEL == IR_TYPE_SKYWORTH)
    U16 tmp;
    static unsigned long PreTime;
    unsigned long CurrentTime = _MDrv_IR_GetSystemTime();
    if ((CurrentTime - PreTime > IR_TIMEOUT_CYC/1000))//reset
    {
        _u32IRCount = 0;
        _brepeat_flag=FALSE;
        _u8PrevKeyCode = 0xfaUL;  //invalidate key code
        _bskyworth_normal_code = FALSE;
        _bskyworth_shuttle_code = FALSE;
    }
    tmp=(((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)) & 0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
    if(tmp>10000)
    {
        _u32IRCount=0;
        _bIRStart = TRUE;
    }

    if(TRUE == _bIRStart)
    {
        if(!_bskyworth_normal_code && !_bskyworth_shuttle_code)
        {
            if(tmp > (IR_OFC_LOB) && tmp < (IR_OFC_UPB))
            {
                _u8BeforeHeader=_u32IRCount;
                _u32IRCount=1;
                _bskyworth_normal_code = TRUE;
                _bskyworth_shuttle_code = FALSE;
            }
            else if(tmp > (IR_OFC_LOB2) && tmp < (IR_OFC_UPB2))
            {
                _u32IRCount=1;
                _bskyworth_normal_code = FALSE;
                _bskyworth_shuttle_code = TRUE;
            }
        }
        if(_u32IRCount >= IR_SWDECODE_MODE_BUF_LEN)
        {
            _u32IRCount = 0;
        }
        _u32IRData[_u32IRCount++] = tmp;
        if((_u32IRCount==3) && (TRUE == _bskyworth_normal_code))
        {
            if (((_u32IRData[1] > (IR_HDC_LOB)) && (_u32IRData[1] < (IR_HDC_UPB)))&&
                (_u32IRData[2] > IR_LG1_LOB && _u32IRData[2] < IR_LG1_UPB)&&
                (_u8BeforeHeader==1))//repeat key, 150
            {
                _brepeat_flag = TRUE;
            }
        }
    }
    PreTime = _MDrv_IR_GetSystemTime();

    #elif(IR_TYPE_SEL == IR_TYPE_TCL)
    U32 u32PulseCounter;
    u32PulseCounter = (((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)) & 0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
    if(_u32IRCount >= IR_SWDECODE_MODE_BUF_LEN)
    {
        _u32IRCount = 0;
    }
    _u32IRData[_u32IRCount++] = u32PulseCounter;
    if (u32PulseCounter>27000) {
        _u32IRCount=0;
        _u32IRData[_u32IRCount++] = u32PulseCounter;
    } else if (u32PulseCounter>(IR_OFC_RP_LOB)&&u32PulseCounter<(IR_OFC_RP_UPB)) {
        _brepeat_flag=TRUE;
        _u32IRCount=0;
        _u32IRData[_u32IRCount++] = u32PulseCounter;
    } else if(u32PulseCounter>(IR_HDC_LOB)&&u32PulseCounter<(IR_HDC_UPB)) {
        _u32IRCount=1;
    _u32IRData[_u32IRCount++] = u32PulseCounter;
    }

    #elif(IR_TYPE_SEL == IR_TYPE_HAIER)
    U16 tmp;

    static unsigned long ulPreTime;
    if ((_MDrv_IR_GetSystemTime() - ulPreTime > IR_TIMEOUT_CYC/1000))//reset
    {
        _u32IRCount = 0;
        _btoshiba_code=FALSE;
        _brepeat_flag=FALSE;
        _u8PrevKeyCode = 0;
        _bBeginRepeat_flag = 0;
        _Repeat_Num = 0;
    }
    tmp=(((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)) & 0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
    if(_u32IRCount >= IR_SWDECODE_MODE_BUF_LEN)
    {
        _u32IRCount = 0;
    }
    _u32IRData[_u32IRCount++] = tmp;
    if(tmp>10000)
    {
        _u32IRCount=0;
        _u32IRData[_u32IRCount++] = tmp;
    }
    else if(tmp> (IR_HDC_LOB)&&tmp < (IR_HDC_UPB))
    {
        _u32IRCount=1;
        _u32IRData[_u32IRCount++] = tmp;
    }

    if(_u32IRCount==2)
    {
        if ((_u32IRData[1] > (IR_HDC_LOB)) && (_u32IRData[1] < (IR_HDC_UPB)))
        {
            _btoshiba_code=TRUE;
        }
        else
        {
            _btoshiba_code=FALSE;
            _bBeginRepeat_flag = FALSE;
            _Repeat_Num = 0;
        }
    }
    else if(_u32IRCount==3)
    {
        if (_u32IRData[2] > IR_LG1_LOB && _u32IRData[2] < IR_LG1_UPB)   //repeat key
            _brepeat_flag=TRUE;
    }
    /*if(!btoshiba_code&&_u32IRCount>18)
    {
        for(tmp=0;tmp<18;tmp++)
            printk("<<<%d data %d\n",tmp,_u32IRData[tmp]);
        _u32IRCount=0;
    }
    else if(btoshiba_code&&_u32IRCount>33)
    {
        for(tmp=0;tmp<34;tmp++)
            printk("<<<%d data %d\n",tmp,_u32IRData[tmp]);
        _u32IRCount=0;
    }*/
    ulPreTime = _MDrv_IR_GetSystemTime();

    #elif (IR_TYPE_SEL == IR_TYPE_CUS08_RC5)

    U16 u16IrCounter,u16N_Shot;

    u16IrCounter=((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)&0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
    u16N_Shot=(REG(REG_IR_SHOT_CNT_H_FIFO_STATUS))&0x10UL;

    if ((u16IrCounter > ONE_BIT_UB) || (u16IrCounter < HALF_BIT_LB))    // if ir counter so big, then reset and return
    {
        _u8IrRc5LastBit = TRUE;
        _u16IrRc5Data = 0;
        _u8IrRc5Bits = 0;
    }
    else
    {
        if (u16IrCounter >= HALF_BIT_LB && u16IrCounter <= HALF_BIT_UB)     //detect
        {
            _u8IrRc5Bits++;
            if (_u8IrRc5LastBit)
            {
                _u8IrRc5LastBit = FALSE;
                _u16IrRc5Data <<= 1;
                if (u16N_Shot==0) // Is nshot
                    _u16IrRc5Data |= 1;
            }
            else
            {
                _u8IrRc5LastBit = TRUE;
            }
        }
        else if (u16IrCounter >= ONE_BIT_LB && u16IrCounter <= ONE_BIT_UB)
        {
            if (_u8IrRc5LastBit)
            {
                _u16IrRc5Data <<= 1;
                if (u16N_Shot==0) // Is nshot
                    _u16IrRc5Data |= 1;
                _u8IrRc5Bits += 2;
            }
            else    //error stste
            {
                _u16IrRc5Data = 0;
                _u8IrRc5Bits = 0;
            }
        }

        if (_u8IrRc5Bits == IR_RC5_DATA_BITS)
        {
            if(u16N_Shot==0)
            {
                _u16IrRc5Data <<= 1;
                _u8IrRc5Bits++;
            }
        }
    }

       #elif (defined(IR_TYPE_SWRC6) && IR_TYPE_SEL == IR_TYPE_SWRC6)
       //for decode
       static unsigned long ulPreTime;
       U16 u16IrCounter = 0;
       U8 u8N_Shot = 0;
       U8 i =0;
       if ((_MDrv_IR_GetSystemTime() - ulPreTime > IR_TIMEOUT_CYC/1000))//||_u8IRRepeated) //timeout or not handler yet
       {
           //reinit args
           rc6_bits=0;
           rc6_databits=0;
           memset(rc6_data,0,sizeof(rc6_data));
           data_count =0;
           data_flag = 0;
           rc6_mode=0;
           done_flag =0;
           status = STATE_RC6_DATA;
       }
       u16IrCounter=((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)&0xFUL) << 16) | ((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);
       u8N_Shot=(REG(REG_IR_SHOT_CNT_H_FIFO_STATUS))&0x10UL?1:0;

       _MDrv_IR_Decode_SWRC6(u16IrCounter,u8N_Shot);

       ulPreTime = _MDrv_IR_GetSystemTime();

       #elif (IR_TYPE_SEL == IR_TYPE_HISENSE || IR_TYPE_SEL == IR_TYPE_MSTAR_DTV || IR_TYPE_SEL == IR_TYPE_CHANGHONG)
       static unsigned long ulPreTime;
       U32 tmp;

       if ((_MDrv_IR_GetSystemTime() - ulPreTime > IR_TIMEOUT_CYC/1000))//||_u8IRRepeated) //timeout or not handler yet
       {
           _u32IRCount = 0;
             _u8IRRepeateDetect=0;
             _u8IRRepeated=0;
             _u8IRRepeatedNum=0;
             _u8IRHeadReceived=0;
       }
       if (_u32IRCount <IR_SWDECODE_MODE_BUF_LEN)
       {
        tmp=((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & 0xFUL) << 16) | REG(REG_IR_SHOT_CNT_L);
        if(tmp<IR_LG0_LOB)//error signal
        {
            _u32IRCount = 0;
            _u8IRRepeateDetect=0;
            _u8IRRepeated=0;
            _u8IRRepeatedNum=0;
            _u8IRHeadReceived=0;
        }

        if( tmp>(IR_HDC_LOB))//Head received
        {
            #if (IR_SWFIFO_DECODE_MODE==DISABLE)
            _u32IRCount=0;
            #endif
            _u8IRHeadReceived=1;
        }
        /*
        printk("\n[%d][%d][%d]\n",tmp,IR_OFC_RP_LOB,IR_OFC_RP_UPB);
        //eg haier repeate: tmp|IR_OFC_RP_LOB|IR_OFC_RP_UPB
        [92715][1846][2769]
        [4655][1846][2769]
        [2047][1846][2769]

        //eg mstar dtv repeate tmp|IR_OFC_RP_LOB|IR_OFC_RP_UPB
        [97137][1846][2769]
        [2581][1846][2769]
        so mstar dtv  has no IR_OFC_UPB code
        */
        if(_u8IRHeadReceived)//begin data received
        {
            _u32IRData[_u32IRCount++] = tmp;
            if(_u8IRRepeateDetect)
            {
                if( tmp>(IR_OFC_RP_LOB) && tmp<(IR_OFC_RP_UPB) )
                {
                    if(_u32IRCount<4)//for quickly change ir
                    {
                        _u8IRRepeated=1;
                        _u8IRHeadReceived=0;
                    }
                }
            }

        }

    }
    ulPreTime = _MDrv_IR_GetSystemTime();

    #elif (IR_TYPE_SEL == IR_TYPE_CUS21SH)

    U32 tmp;

    #if (1)
    tmp = 0;
    tmp |=  ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & 0xFUL) << 16);
    tmp |=  (REG(REG_IR_SHOT_CNT_L) & 0xFFFFUL);


    if(tmp >= 420L && tmp < 4000L)
    {
        if(tmp  >=1580L && tmp < 4000L)
        {
            _u16IRKeyMap|=(1<<_u32IRCount);
        }
        _u32IRCount++;
        //ReceivingMode=TRUE;
        RxInProcess=TRUE;
        //RxTimeOutCount = _MDrv_IR_GetSystemTime();
        //ResetKeyoffTimer();
    }
    else if(tmp >= 4000L)
    {
        _bExpectedTCome = FALSE;
        SetToStandbyMode = TRUE;
        ReceivingMode = TRUE;
        _u32IRCount=0;
        _u16IRKeyMap=0;
    }


    //if(BufIdx >= 15)
        //printk("IR: %s:%d %x,%x\n",__FUNCTION__,__LINE__,_u32IRCount,_u16IRKeyMap);
    #else
    static unsigned long ulPreTime;

    if (_MDrv_IR_GetSystemTime() - ulPreTime > IR_TIMEOUT_CYC/1000) //timeout
    {
        _u32IRCount = 0;
        _u8IRReceiveDetect = 0;
        _u8IRRepeateDetect = 0;
        _u16IRKeyMap = 0;
    }

    if (_u32IRCount < IR_SWDECODE_MODE_BUF_LEN)
    {
        tmp = ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & 0xFUL) << 16) | (REG(REG_IR_SHOT_CNT_L) & 0xFFFFUL);

        if(tmp < 420L)  //Error signal
        {
            _u32IRCount = 0;
            _u8IRReceiveDetect = 0;
            _u8IRRepeateDetect = 0;
            _u16IRKeyMap = 0;
        }
        else if(tmp >= 420L)    //Data received
        {
            if(_u32IRCount < 16)
            {
                if((tmp >= 1580L) && (tmp < 4000L))
                {
                    _u16IRKeyMap |= (1 << (/*15 -*/ _u32IRCount - 1));
                }
            }
            else if(_u32IRCount == 16)
            {
                //_u16IRKeyMap = _u16IRKeyMap << 1;
            }

            _u32IRData[_u32IRCount++] = tmp;
        }

        if(tmp >= 4000L)   //Head received
        {
            if(_u8IRReceiveDetect == 1)
            {
                if(_u8IRRepeateDetect == 1)
                {
                    _u32IRCount = 0;
                    _u8IRReceiveDetect = 0;
                    _u8IRRepeateDetect = 0;
                    _u16IRKeyMap = 0;
                }
                _u8IRRepeateDetect = 1;
            }
            _u8IRReceiveDetect = 1;
        }

        ulPreTime = _MDrv_IR_GetSystemTime();
    }
    #endif

    #elif (IR_TYPE_SEL == IR_TYPE_TOSHIBA)

    U32 tmp;


    tmp = 0;
    tmp |=  ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & 0xFUL) << 16);
    tmp |=  (REG(REG_IR_SHOT_CNT_L) & 0xFFFFUL);


    if(_bRxInProcess == FALSE)
    {
        if(_u32IRCount == 0)
        {
            //_u32CurrentTime = _MDrv_IR_GetSystemTime();
            if(GetKeyoffTimer() > 100)
            {
                if(tmp > 1800L && tmp < 2600L)//Repeat Negative Period 2.25ms After Leader signal.
                {
                    _bIrDetect = TRUE;
                    _bIrRepeat = TRUE;
                    _u32IRCount = 0;
                    ResetKeyoffTimer();//IR_SW_REPEAT_TIMEOUT;
                    _bRxInProcess = FALSE;
                    goto SW_ISR_LAST;
                }
            }

            //if(g_wIrPulseCounter < HEADER_UPPER_BOND && g_wIrPulseCounter > HEADER_LOWER_BOND)
            if(tmp > 4000L && tmp < 5000L)//Leader code - Low 4500us(4.5ms)
            {
                _bRxInProcess = TRUE;
                ResetKeyoffTimer();//IR_SW_REPEAT_TIMEOUT;
                _u32IRCount = 0;
                _u32IRKeyMap = 0;
                return IRQ_HANDLED;
            }
            else
            {
                _u32IRCount = 0;
                _bRxInProcess = FALSE;
                _bIrDetect = FALSE;
                _bIrRepeat = FALSE;
                return IRQ_HANDLED;
            }
        }
        else
        {
            _u32IRCount = 0;
            _bRxInProcess = TRUE;
        }
    }
    else
    {
        //if((g_wIrPulseCounter > 400) && (g_wIrPulseCounter < 1400))//Negative period 560us(5.6ms)
        if(tmp < 1400)//Negative period 560us(5.6ms)
        {
            //Decode Bit
            _u32IRCount ++;
        }
        //else if((g_wIrPulseCounter > 1399) && (g_wIrPulseCounter < 2750))//Negative period 2250us(2.25ms)
        else if((tmp >= 1400) && (tmp < 2750))//Negative period 2250us(2.25ms)
        {
            //Decode Bit
            _u32IRKeyMap|=(1<<_u32IRCount);
            _u32IRCount++;
        }
        else
        {
            //Pulse Count Error!. => Reset
            _u32IRCount = 0;
        }

        if(_u32IRCount >= 32)//IR_SW_COUNT_BIT_NUM)
        {
            _u32IRCount = 0;
            _bIrDetect = TRUE;
            _bRxInProcess = FALSE;
        }
    }

    #else
    static unsigned long ulPreTime;
    if (_MDrv_IR_GetSystemTime() - ulPreTime > IR_TIMEOUT_CYC/1000) //timeout
    {
        _u32IRCount = 0;
    }

    if (_u32IRCount <IR_SWDECODE_MODE_BUF_LEN)
    {
        _u32IRData[_u32IRCount++] = ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & 0xFUL) << 16) | REG(REG_IR_SHOT_CNT_L);
    }
    ulPreTime = _MDrv_IR_GetSystemTime();
    #endif
#elif (IR_MODE_SEL == IR_TYPE_SWDECODE_KON_MODE)
    // get a ir counter value
    g_u16IrKonCounter=((REG(REG_IR_SHOT_CNT_L))&0xFFFFUL);

 //   printk("<<<%d \n",g_u16IrKonCounter);

    g_u8IrKonFlag |= IR_KON_COUNTER_ERROR;
    // IR_KONKA_DETECT_START means detected header code(here g_u8IrKonkaBits==1), and ready to detect system/data code
    if ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)) & 0x07UL)
    {

    }
    else if (g_u16IrKonCounter > IR_KON_PWS3_HEADER_CNT_LB && g_u16IrKonCounter < IR_KON_PWS3_HEADER_CNT_UB)
    {
        g_u8IrKonFlag  |= IR_KON_DETECT_START;
        g_u16IrKonData = 0;
        g_u8IrKonBits = 1;
        return IRQ_HANDLED;
    }
    else if (g_u8IrKonFlag & IR_KON_DETECT_START )
    {
        if (g_u16IrKonCounter > IR_KON_PWS3_LOGIC0_CNT_LB && g_u16IrKonCounter < IR_KON_PWS3_STOP_CNT_UB)
            g_u8IrKonFlag &= ~IR_KON_COUNTER_ERROR;
    }

    if (g_u8IrKonFlag & IR_KON_COUNTER_ERROR)
    {
        g_u8IrKonFlag &= ~IR_KON_DETECT_START;
        g_u16IrKonData = 0;
        g_u8IrKonBits = 0;
        return IRQ_HANDLED;
    }

    if (g_u8IrKonBits > 0)
    {
        if (g_u16IrKonCounter > IR_KON_PWS3_LOGIC1_CNT_LB && g_u16IrKonCounter < IR_KON_PWS3_LOGIC1_CNT_UB)
        {
            g_u16IrKonData <<= 1;
            g_u16IrKonData |= 1;
            g_u8IrKonBits++;
        }
        else if (g_u16IrKonCounter > IR_KON_PWS3_LOGIC0_CNT_LB && g_u16IrKonCounter <  IR_KON_PWS3_LOGIC0_CNT_UB)
        {
            g_u16IrKonData <<= 1;
            g_u8IrKonBits++;
        }
        else if (g_u16IrKonCounter > IR_KON_PWS3_STOP_CNT_LB && g_u16IrKonCounter <  IR_KON_PWS3_STOP_CNT_UB)
        {
            g_u8IrKonBits++;
            if(g_u8IrKonBits >= IR_KON_DATA_BITS)
            {
                if(!(g_u8IrKonFlag & IR_KON_DETECT_END))
                {
                    g_u8IrKonFlag |= IR_KON_DETECT_END;
                    g_u16IrKonDecode = g_u16IrKonData;
                }
                g_u8IrKonBits = 0;
            #ifndef CONFIG_MSTAR_IR_INPUT_DEVICE
                if (_MDrv_IR_GetSystemTime()-g_u8IrKonRepeatTimeout > IR_KON_PWS3_REPEATE_TIMEOUT)
                    g_u8IrKonFlag |= IR_KON_REPEATE_TIMEOUT;
                else
                    g_u8IrKonFlag &= (~IR_KON_REPEATE_TIMEOUT);
            #else
                g_u8IrKonFlag |= IR_KON_REPEATE_TIMEOUT;
            #endif
                g_u8IrKonRepeatTimeout = _MDrv_IR_GetSystemTime();
            }
            else
            {
                 g_u8IrKonFlag &= ~IR_KON_DETECT_START;
                g_u16IrKonData = 0;
                g_u8IrKonBits = 0;
            }
        }
    }
#endif  //#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
#ifdef CONFIG_IR_SUPPLY_RNG
    {
        unsigned int u32Temp;
        unsigned short u16Temp;

        u16Temp = MIPS_REG(REG_RNG_OUT);
        memcpy((unsigned char *)&u32Temp+0, &u16Temp, 2);
        u16Temp = MIPS_REG(REG_RNG_OUT);
        memcpy((unsigned char *)&u32Temp+2, &u16Temp, 2);
        add_input_randomness(EV_MSC, MSC_SCAN, u32Temp);
    }
#endif
    //regs = NULL;
#if (IR_TYPE_SEL == IR_TYPE_TOSHIBA)
SW_ISR_LAST:
#endif

    BOOL bHaveKey = FALSE;
    if (bHaveKey = _MDrv_IR_GetKey(&u8Key, &u8System, &u8RepeatFlag)) {
#if CONFIG_MSTAR_IR_FANTASY_MODE
        if (down_trylock(&fantasy_protocol.sem) == 0) {
            if (fantasy_protocol.data == 0) {
#if defined(IR_TYPE_RCMM) && (IR_TYPE_SEL == IR_TYPE_RCMM)
                fantasy_protocol.data = (u8Key<<8 | u8RepeatFlag);
                fantasy_protocol.data |=(u8System<<16);
                fantasy_protocol.data |= (0x01UL << 28); //set fantasy class to IR

#elif defined(IR_TYPE_HISENSE) && (IR_TYPE_SEL == IR_TYPE_HISENSE)//for hisense need suport 2 ir head code ;
                U32 hisense_factory_ir = 0;
                if (u8RepeatFlag&FCTORY_HEADER_RECEIVE) { //sec ir
                    u8RepeatFlag &= (~FCTORY_HEADER_RECEIVE);
                    hisense_factory_ir = (0x04UL << 28);
                } else if(u8RepeatFlag&PCCOMMAND_HEADER_RECEIVE) { //third ir
                    u8RepeatFlag &= (~PCCOMMAND_HEADER_RECEIVE);
                    hisense_factory_ir = (0x08UL << 28);
                } else { //first ir
                    hisense_factory_ir = (0x01UL << 28);
                }
                fantasy_protocol.data = (u8Key << 8 | u8RepeatFlag);
                fantasy_protocol.data |= hisense_factory_ir;
#elif defined(IR_TYPE_TCL) && (IR_TYPE_SEL == IR_TYPE_TCL)
                fantasy_protocol.data = (u8Key << 8 | u8RepeatFlag);
                fantasy_protocol.data |= (0x06UL << 28);
                fantasy_protocol.data |= (u8System << 16);
#else
                fantasy_protocol.data = (u8Key << 8 | u8RepeatFlag);
                fantasy_protocol.data |= (0x01UL << 28); //set fantasy class to IR
#endif
            }

            up(&fantasy_protocol.sem);
            wake_up_interruptible(&fantasy_protocol.inq);
        }
#endif
#ifdef CONFIG_MSTAR_SOFTWARE_IR_MODULE
        if (software_ir_enable()) {
            u8Key_for_mdrv_software_ir = u8Key;
            u8RepeatFlag_for_mdrv_software_ir = u8RepeatFlag;

            take_over_by_software_ir(u8Key, u8System, u8RepeatFlag, ir->dev, &ir->ir);
        }
#endif

#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
        enqueue((u8System << 8) | u8Key);
        complete(&key_completion);
#endif
    }
    //printk("HaveKey=%d, KEY=%d, RepeatFalg=%d\n", bHaveKey, u8Key, u8RepeatFlag);


#ifdef CONFIG_MSTAR_IR_POWER_KEY_LONG_PRESS
    if ((u8Key == CONFIG_MSTAR_IR_POWER_KEY) && (u8RepeatFlag == 1)) { //only judge "power key" & repoeat for long press
        if (_u16IRLongPressPrevTime == 0) {
            _u16IRLongPressPrevTime = _MDrv_IR_GetSystemTime();
        }

        if (_u16IRLongPressPrevTime > _MDrv_IR_GetSystemTime()) {
            _u16IRLongPressPrevTime = _MDrv_IR_GetSystemTime();
        }

        if ((_MDrv_IR_GetSystemTime() - _u16IRLongPressPrevTime) > CONFIG_MSTAR_IR_LONG_PRESS_TIMEOUT) {
            _u16IRLongPressPrevTime = 0;

            printk(KERN_EMERG "\n[IR][%s]: Power Key Long Press , System is being to reset!!!!!!!\n", __FUNCTION__);
            //system reboot
            kernel_restart(NULL);
        }
    } else {
        _u16IRLongPressPrevTime = 0;
    }
#endif

    return IRQ_HANDLED;
}
#endif

#if (IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
//-------------------------------------------------------------------------------------------------
/// Get IR key. It is a non-blocking function.
/// @param pu8Key  \b IN: Return IR key value.
/// @param pu8Flag \b IN: Return IR repeat code.
///
/// @return TRUE:  Success
/// @return FALSE: No key or repeat key is faster than the specified period
//-------------------------------------------------------------------------------------------------
static BOOL _MDrv_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
#ifndef CONFIG_MSTAR_IR_INPUT_DEVICE
    static unsigned long  _ulPrevKeyRepeatTime;
#endif
    static BOOL  _bCheckQuickRepeat;
    BOOL bRet=FALSE;
    *pu8System = 0;

#ifdef CONFIG_MSTAR_DYNAMIC_IR
    U16 scancode = 0;
#endif
    if(REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & IR_FIFO_EMPTY)
    {
        _bCheckQuickRepeat = 0;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        IR_bRepKey = _bCheckQuickRepeat;
#endif
        return FALSE;
    }

    if(((_MDrv_IR_GetSystemTime() - _ulPrevKeyTime) >= IR_TIMEOUT_CYC/1000))
    {
        *pu8Key = REG(REG_IR_CKDIV_NUM_KEY_DATA) >> 8;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        IR_KeyValue = *pu8Key;
#endif
        REG(REG_IR_FIFO_RD_PULSE) |= 0x0001UL; //read
        #ifdef __arm__
    mdelay(10);
        #else
        udelay(1000*10);
        #endif
        #if(IR_TYPE_SEL == IR_TYPE_TOSHIBA)
        *pu8Key = _MDrv_IR_CUS22T_ParseKey(*pu8Key);
        #endif
#ifdef CONFIG_MSTAR_DYNAMIC_IR
        scancode = (U16)*pu8Key;
        if(debug_flag) printk(KERN_DEBUG "[IR] scancode = 0x%x\n", scancode);
        *pu8Key = translate_scancode(scancode);
        if(debug_flag) printk(KERN_DEBUG "[IR] keycode = 0x%x\n", pu8Key);
#endif
       _u8PrevKeyCode = *pu8Key;
        *pu8Flag = 0;
        _ulPrevKeyTime = _MDrv_IR_GetSystemTime();
        _ePrevKeyProperty = E_IR_KEY_PROPERTY_INIT;
        _bCheckQuickRepeat = 0;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        IR_bRepKey = _bCheckQuickRepeat;
#endif
        _MDrv_IR_ClearFIFO();
        return TRUE;
    }
    else
    {
        if(_bCheckQuickRepeat==0)
        {
            _bCheckQuickRepeat = 1;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
            IR_bRepKey = _bCheckQuickRepeat;
#endif
            _ulPrevKeyTime = _MDrv_IR_GetSystemTime();
            _MDrv_IR_ClearFIFO();
            return FALSE;
        }
#ifdef CONFIG_MSTAR_DYNAMIC_IR
        scancode = (U16)REG(REG_IR_CKDIV_NUM_KEY_DATA) >> 8;
        if(debug_flag) printk(KERN_DEBUG "[IR] scancode = 0x%x\n", scancode);
        *pu8Key = translate_scancode(scancode);
        if(debug_flag) printk(KERN_DEBUG "[IR] keycode = 0x%x\n", *pu8Key);
#endif
        *pu8Key = REG(REG_IR_CKDIV_NUM_KEY_DATA) >> 8;
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
        IR_KeyValue = *pu8Key;
#endif
        *pu8Flag = (REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & IR_RPT_FLAG)? 1 : 0;
        REG(REG_IR_FIFO_RD_PULSE) |= 0x0001UL; //read
        bRet = FALSE;
        _ulPrevKeyTime = _MDrv_IR_GetSystemTime();
        if ( (*pu8Flag == 1) && ( *pu8Key == _u8PrevKeyCode ))
        {
#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
            bRet = TRUE;
#else
            unsigned long i = _MDrv_IR_GetSystemTime();
            if( _ePrevKeyProperty == E_IR_KEY_PROPERTY_INIT)
            {
                _u8PrevKeyCode     = *pu8Key;
                _ulPrevKeyRepeatTime    = i;
                _ePrevKeyProperty  = E_IR_KEY_PROPERTY_1st;
            }
            else if(_ePrevKeyProperty == E_IR_KEY_PROPERTY_1st)
            {
                if( (i - _ulPrevKeyRepeatTime) > _u32_1stDelayTimeMs)
                {
                    _ulPrevKeyRepeatTime = i;
                    _ePrevKeyProperty  = E_IR_KEY_PROPERTY_FOLLOWING;
                    bRet = TRUE;
                }
            }
            else //E_IR_KEY_PROPERTY_FOLLOWING
            {
                if( (i - _ulPrevKeyRepeatTime) > _u32_2ndDelayTimeMs)
                {
                    _ulPrevKeyRepeatTime = i;
                    bRet = TRUE;
                }
            }
#endif // CONFIG_MSTAR_IR_INPUT_DEVICE
        }
    }

    // Empty the FIFO
    _MDrv_IR_ClearFIFO();
    return bRet;

}

#elif (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
//-------------------------------------------------------------------------------------------------
/// Get IR key. It is a non-blocking function.
/// @param pu8Key  \b IN: Return IR key value.
/// @param pu8Flag \b IN: Return IR repeat code.
///
/// @return TRUE:  Success
/// @return FALSE: No key or repeat key is faster than the specified period
//-------------------------------------------------------------------------------------------------
static BOOL _MDrv_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
    U8 u8KeyAddr,u8KeyCmd;
    BOOL bRet=FALSE;
    *pu8System = 0;

    if(REG(REG_IR_RC_KEY_FIFO_STATUS) & IR_RC_FIFO_EMPTY)
    {
        return FALSE;
    }

    u8KeyAddr = REG(REG_IR_RC_KEY_COMMAND_ADD)&0x3fUL;//RC5: {2'b0,toggle,address[4:0]} reg[7:0]
    u8KeyCmd = (REG(REG_IR_RC_KEY_COMMAND_ADD)&0x3f00UL)>>8;//RC5: {repeat,1'b0,command[13:8]} reg[15:8]
    *pu8Flag = (REG(REG_IR_RC_KEY_COMMAND_ADD)&0x8000UL)>>15;//repeat
    *pu8Key = u8KeyCmd;

    //printk("REG_IR_RC_KEY_FIFO_STATUS = 0x%x\n",REG(REG_IR_RC_KEY_FIFO_STATUS));
    //printk("REG_IR_RC_KEY_COMMAND_ADD = 0x%x\n",REG(REG_IR_RC_KEY_COMMAND_ADD));
    //printk("@@@ RC5 protocol: Addr = 0x%x, Cmd = 0x%x \n",u8KeyAddr,u8KeyCmd);
    //printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    REG(REG_IR_RC_FIFO_RD_PULSE)|=0x1UL;

    bRet=TRUE;
    // Empty the FIFO
    _MDrv_IR_ClearFIFO();
    return bRet;

}

#elif (IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
//-------------------------------------------------------------------------------------------------
/// Get IR key.
/// @param pu8Key  \b IN: Return IR key value.
/// @param pu8Flag \b IN: Return IR repeat code.
///
/// @return TRUE:  Success
/// @return FALSE: Failure
//-------------------------------------------------------------------------------------------------
static BOOL flag_repeat = TRUE;
static U8  prev_key_rpt = 0x00UL;

static BOOL _MDrv_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
    BOOL bRet = FALSE;

    u32 i, j;
    *pu8System = 0;

    if ((flag_repeat==TRUE) && ((REG(REG_IR_SHOT_CNT_H_FIFO_STATUS)&IR_RPT_FLAG) == IR_RPT_FLAG))
    {
         *pu8Key  = prev_key_rpt;
         *pu8Flag = 1;
         bRet = TRUE;
         goto  Raw_Repeat;
    }

    flag_repeat=TRUE;

    for (j=0; j<IR_RAW_DATA_NUM; j++)
    {
        if ( REG(REG_IR_SHOT_CNT_H_FIFO_STATUS) & IR_FIFO_EMPTY)  // check FIFO empty
            break;

        _u8IRRawModeBuf[_u8IRRawModeCount++] = REG(REG_IR_CKDIV_NUM_KEY_DATA) >> 8;
        REG(REG_IR_FIFO_RD_PULSE) |= 0x0001UL; //read

        for(i=0;i<5;i++); //Delay

#ifdef CONFIG_MSTAR_DYNAMIC_IR
        if(_u8IRRawModeCount == IR_RAW_DATA_NUM)
        {
            do{
                int headcodeindex = 0;;
                _u8IRRawModeCount = 0;
                for (headcodeindex = 0; headcodeindex < g_headcodecount; ++headcodeindex)
                {
                    if (_u8IRRawModeBuf[0] != headcode0array[headcodeindex])
                        continue;
                    if (_u8IRRawModeBuf[1] != headcode1array[headcodeindex])
                        continue;
                    break;
                }

                if(_u8IRRawModeBuf[2] == (U8)(~_u8IRRawModeBuf[3]))
                {
                    U16 scancode = (U16)((headcodeindex << 8) + _u8IRRawModeBuf[2]);
                    if(debug_flag) printk(KERN_DEBUG "[IR] scancode = 0x%x\n", scancode);
                    *pu8Key = translate_scancode(scancode);
                    if(debug_flag) printk(KERN_DEBUG "[IR] keycode = 0x%x\n", *pu8Key);
                    *pu8Flag = 0;
                    bRet = TRUE;
                    flag_repeat = FALSE;
                    prev_key_rpt = *pu8Key;
                    break;
                }
            }while(0);
        }

#else

        if(_u8IRRawModeCount == IR_RAW_DATA_NUM)
        {
            _u8IRRawModeCount = 0;
            if( (_u8IRRawModeBuf[0]==_u8IRHeaderCode0) &&
                (_u8IRRawModeBuf[1]==_u8IRHeaderCode1) )
            {
                if(_u8IRRawModeBuf[2] == (U8)(~_u8IRRawModeBuf[3]))
                {
                    *pu8Key = _MDrv_IR_ParseKey(_u8IRRawModeBuf[2]);    // translate for MIPS
                    *pu8Flag = 0;
                    bRet = TRUE;
                    flag_repeat = FALSE;
                    prev_key_rpt = *pu8Key;
                    break;
                }
            }
        }
#endif
    }

Raw_Repeat:
    // Empty the FIFO
    //_MDrv_IR_ClearFIFO();
    REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= (0x1UL <<15);

    return bRet;
}

#elif(IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
//-------------------------------------------------------------------------------------------------
/// Get IR key.
/// @param pu8Key  \b IN: Return IR key value.
/// @param pu8Flag \b IN: Return IR repeat code.
///
/// @return TRUE:  Success
/// @return FALSE: Failure
//-------------------------------------------------------------------------------------------------
static BOOL _MDrv_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
    BOOL bRet = FALSE;

#if(IR_TYPE_SEL == IR_TYPE_RCMM)
    unsigned long i;
#else
#if ((IR_TYPE_SEL != IR_TYPE_CUS21SH) && (IR_TYPE_SEL != IR_TYPE_TOSHIBA))
    #if (IR_TYPE_SEL != IR_TYPE_CUS08_RC5)
    U32 u8Byte, u8Bit;
#if(IR_TYPE_SEL == IR_TYPE_SKYWORTH)
    U8 u8IRSwModeByte;
    U8 u8IRSwModeBuf[IR_RAW_DATA_NUM];
#else
    U8 u8IRSwModeBuf[IR_RAW_DATA_NUM];
#endif
    U32 *pu32IRData = NULL;
    *pu8System = 0;
    #else
    unsigned long i;
    #endif
#endif
#endif

#if(IR_TYPE_SEL == IR_TYPE_RCMM)

    if(UpDataFlage)
    {
        //printk("GetKey\n");
        UpDataFlage = FALSE;

        switch(RCMode)
        {
            case RCMMOEM_LONGID_MODE|RC_MODE:
            {
                if((_u8IrPreRcmmData[0] == RCByte[0]) && (_u8IrPreRcmmData[1] == RCByte[1]) && (_u8IrPreRcmmData[2] == RCByte[2]) && (_u8IrPreRcmmData[3] == RCByte[3]))
                {
                    *pu8Flag = TRUE;
                }

                u16CustomerID = ((RCByte[0] & 0x1FUL) << 4) | ((RCByte[1] & 0xF0UL) >> 4);

                if(u16CustomerID == 0x007DUL)
                {
                    *pu8Key = RCByte[3];
                    if(*pu8Key>=30 && *pu8Key<=132)
                        *pu8Key = RCByte[3]+111;
                     //printk("0x%x:,%d\n",*pu8Key,*pu8Key);
                    *pu8System = (RCByte[2] & 0x7FUL);

                    _u8IrPreRcmmData[0] = RCByte[0];
                    _u8IrPreRcmmData[1] = RCByte[1];
                    _u8IrPreRcmmData[2] = RCByte[2];
                    _u8IrPreRcmmData[3] = RCByte[3];

                    RCByte[0] = 0x0000UL;
                    RCByte[1] = 0x0000UL;
                    RCByte[2] = 0x0000UL;
                    RCByte[3] = 0x0000UL;

                    RCMode = 0;
                    RCBitsCnt = 0;
                    bRet = TRUE;
                }

                break;
            }
            case RCMMOEM_LONGID_MODE|KEYBOARD_MODE:
            {
                if((_u8IrPreRcmmData[0] == RCByte[0]) && (_u8IrPreRcmmData[1] == RCByte[1]) && (_u8IrPreRcmmData[2] == RCByte[2]))
                {
                    *pu8Flag = TRUE;
                }

                u16CustomerID = ((RCByte[0] & 0x1FUL) << 4) | ((RCByte[1] & 0xF0UL) >> 4);
                if(u16CustomerID == 0x007DUL)
                {
                    if(_u8PrevKeyCode==61)
                    {
                        *pu8Key = (RCByte[2] & 0x7FUL) + 0x3DUL;
                        if(*pu8Key>=97&&*pu8Key<=99)
                            *pu8Key += 150;
                    }
                    else
                    {
                        *pu8Key = RCByte[2] & 0x7FUL;
                    }
                    //printk("0x%x:,%d\n",*pu8Key,*pu8Key);
                    _u8IrPreRcmmData[0] = RCByte[0];
                    _u8IrPreRcmmData[1] = RCByte[1];
                    _u8IrPreRcmmData[2] = RCByte[2];

                    RCByte[0] = 0x0000UL;
                    RCByte[1] = 0x0000UL;
                    RCByte[2] = 0x0000UL;
                    RCByte[3] = 0x0000UL;

                    RCBitsCnt = 0;
                    bRet = TRUE;
                }


            break;
            }

            case RCMMOEM_LONGID_MODE|MOUSE_MODE:
            break;
            case RCMMOEM_LONGID_MODE|JOYSTICK_MODE:
            break;
            default:
                bRet = FALSE;
                break;
        }

    }

    if(bRet)
    {
        //printk("_u8PrevKeyCode=%d,*pu8Key=%d,_ePrevKeyProperty=%d",_u8PrevKeyCode,*pu8Key,_ePrevKeyProperty);
        if (_u8PrevKeyCode != *pu8Key)
        {
            _ePrevKeyProperty = E_IR_KEY_PROPERTY_INIT;
        }
        if ((_u8PrevKeyCode == (*pu8Key + 61))||(_u8PrevKeyCode == (*pu8Key + 211)))
        {
            _ePrevKeyProperty = E_IR_KEY_PROPERTY_1st;
        }

        i = _MDrv_IR_GetSystemTime();
        if( _ePrevKeyProperty == E_IR_KEY_PROPERTY_INIT)
        {
            _u8PrevKeyCode     = *pu8Key;
            _ulPrevKeyTime    = i;
            _ePrevKeyProperty  = E_IR_KEY_PROPERTY_1st;
        }
        else if(_ePrevKeyProperty == E_IR_KEY_PROPERTY_1st)
        {
        //printk("i=%ld,_ulPrevKeyTime=%ld",i,_ulPrevKeyTime);
            if( (i - _ulPrevKeyTime) > 150)
            {
                _ulPrevKeyTime = i;
                _ePrevKeyProperty  = E_IR_KEY_PROPERTY_FOLLOWING;
            }
            else
            {
                bRet = FALSE;
            }
        }
        else //E_IR_KEY_PROPERTY_FOLLOWING
        {
            if( (i - _ulPrevKeyTime) > 150)
            {
                _ulPrevKeyTime = i;
            }
            else
            {
                bRet = FALSE;
            }
        }
    }

    return bRet;

#elif(IR_TYPE_SEL == IR_TYPE_SKYWORTH)
    if(_bskyworth_normal_code || _bskyworth_shuttle_code)
    {
        if(_brepeat_flag)
        {
            *pu8Key = _u8PrevKeyCode;
            *pu8Flag = 1;
            _u32IRCount = 0;
            _brepeat_flag=FALSE;
            _bskyworth_normal_code = FALSE;
            _bskyworth_shuttle_code = FALSE;
            DEBUG_IR(printk("Repeat input key is 0x%x\n", *pu8Key));
            return TRUE;
        }

        if(_bskyworth_normal_code)
        {
            DEBUG_IR(printk("Use skyworth normal code\n"));
            if (_u32IRCount < 34)  //multiple 8
            {
                DEBUG_IR(printk("Get normal IRCount error, _u32IRCount=%d\n", _u32IRCount));
                return FALSE; //not complete yet
            }
            if(_u32IRData[1]< (IR_HDC_LOB)||_u32IRData[1] > (IR_HDC_UPB))
            {
                _u32IRCount = 0;
                _bskyworth_normal_code = FALSE;
                _bIRStart = FALSE;
                DEBUG_IR(printk("No normal IR header\n"));
                return FALSE;
            }
            for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
            {
                u8IRSwModeBuf[u8Byte] = 0;
            }
            pu32IRData = &_u32IRData[2];
        }
        else if(_bskyworth_shuttle_code)
        {
            DEBUG_IR(printk("Use skyworth shuttle code\n"));
            if(_u32IRCount < (10))
            {
                DEBUG_IR(printk("Get shuttle IRCount error, _u32IRCount=%d\n", _u32IRCount));
                return FALSE; //not complete yet
            }
            if(_u32IRData[1]< (IR_OFC_LOB2)||_u32IRData[1] > (IR_OFC_UPB2))
            {
                _u32IRCount = 0;
                _bskyworth_shuttle_code = FALSE;
                _bIRStart = FALSE;
                DEBUG_IR(printk("No shuttle IR header\n"));
                return FALSE;
            }
            u8IRSwModeByte = 0;
            pu32IRData = &_u32IRData[2];
            //printk("StartSignal = %d\n", _u32IRData[0]);
            //printk("OFCSignal = %d\n",_u32IRData[1]);
        }

        if(_bskyworth_normal_code)
        {
            for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
            {
                for( u8Bit=0; u8Bit<8; u8Bit++)
                {
                    u32 u32BitLen = pu32IRData[u8Byte*8+u8Bit];
                    u8IRSwModeBuf[u8Byte] >>= 1;

                    if( u32BitLen > IR_LG0_LOB && u32BitLen < IR_LG0_UPB ) //0
                    {
                        u8IRSwModeBuf[u8Byte] |= 0x00UL;
                    }
                    else if (u32BitLen > IR_LG1_LOB && u32BitLen < IR_LG1_UPB) //1
                    {
                        u8IRSwModeBuf[u8Byte] |= 0x80UL;
                    }
                    else
                    {
                        _u32IRCount = 0;
                        _bskyworth_normal_code = FALSE;
                        _bIRStart = FALSE;
                        DEBUG_IR(printk("Invalid waveform,0x%x\n", u32BitLen));
                        return FALSE;
                    }
                }
                if((u8IRSwModeBuf[0]!=_u8IRHeaderCode0)||(u8Byte==1&&u8IRSwModeBuf[1]!=_u8IRHeaderCode1))
                {
                    _u32IRCount = 0;
                    _bskyworth_normal_code = FALSE;
                    _bIRStart = FALSE;
                    DEBUG_IR(printk("[IR] Invalid header,0x%x,0x%x\n", u8IRSwModeBuf[0], u8IRSwModeBuf[1]));
                    return FALSE;
                }
                else if(u8Byte==3)
                {
                    if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
                    {
                        *pu8Key = u8IRSwModeBuf[2];
                        *pu8Flag = 0;
                        _u8PrevKeyCode=*pu8Key;
                        _u32IRCount = 0;
                        _bskyworth_normal_code = FALSE;
                        _bIRStart = FALSE;
                        DEBUG_IR(printk("Input key is 0x%x\n", *pu8Key));
                        return TRUE;
                    }
                }
            }
        }
        else if(_bskyworth_shuttle_code) //we have to handle shuttle key
        {
            for( u8Bit=0; u8Bit<8; u8Bit++)
            {
                u32 u32BitLen = pu32IRData[u8Bit];
                u8IRSwModeByte >>= 1;

                if( u32BitLen > IR_LG0_LOB2 && u32BitLen < IR_LG0_UPB2 ) //0
                {
                    u8IRSwModeByte |= 0x00UL;
                }
                else if (u32BitLen > IR_LG1_LOB2 && u32BitLen < IR_LG1_UPB2) //1
                {
                    u8IRSwModeByte |= 0x80UL;
                }
                else
                {
                    _u32IRCount = 0;
                    _bskyworth_shuttle_code = FALSE;
                    _bIRStart = FALSE;
                    DEBUG_IR(printk("Invalid waveform,0x%x,u32BitLen[%d]=ERROR\n", u32BitLen, u8Bit));
                    return FALSE;
                }
            }

            //printk("u8IRSwModeByte = %d\n",u8IRSwModeByte);
            if((u8IRSwModeByte&0x03UL) != 0x01UL ) // Customer_Code(01)
            {
                _u32IRCount = 0;
                _bskyworth_shuttle_code = FALSE;
                _bIRStart = FALSE;
                DEBUG_IR(printk("Invalid header\n"));
                return FALSE;
            }

            if(((u8IRSwModeByte>>2)&0x07UL)+((u8IRSwModeByte>>5)&0x07UL) == 0x07UL)
            {
                if(0x01UL == ((u8IRSwModeByte>>2)&0x07UL))
                {
                    *pu8Key = 0x15UL;
                }
                else
                {
                    *pu8Key = 0x14UL;
                }
                *pu8Flag = 0;
                _u8PrevKeyCode=*pu8Key;
                _u32IRCount = 0;
                _bskyworth_shuttle_code = FALSE;
                _bIRStart = FALSE;
                DEBUG_IR(printk("Input key is 0x%x\n", *pu8Key));
                return TRUE;
            }
        }
        _bskyworth_shuttle_code = FALSE;
        _bskyworth_normal_code = FALSE;
        _bIRStart = FALSE;
    }
    return bRet;
#elif (defined(IR_TYPE_SWRC6) && IR_TYPE_SEL == IR_TYPE_SWRC6)
//for get key code
if(done_flag)
{
    //printk("[Lwc Debug Msg] KeyCode[0]=%x\n",rc6_data[0]);
    //printk("[Lwc Debug Msg] KeyCode[1]=%x\n",rc6_data[1]);
    //printk("[Lwc Debug Msg] KeyCode[2]=%x\n",rc6_data[2]);
    //printk("[Lwc Debug Msg] KeyCode[3]=%x\n",rc6_data[3]);
    //printk("[Lwc Debug Msg] RC6_Mode=%d\n",rc6_mode);
    done_flag = 0;
    if(rc6_mode==0)
    {
        *pu8Key = rc6_data[1];
    }
    else if(rc6_mode==1)
    {
        *pu8Key = rc6_data[3];
    }
    *pu8Flag = 0;
    status = STATE_INACTIVE;
    bRet = TRUE;;
}
return bRet;
#elif(IR_TYPE_SEL == IR_TYPE_TCL)
    unsigned long i;
    u32 u32IRDataTmp = 0;

    if (_u32IRCount < 2+3*8) {
        return FALSE; //not complete yet
    }
    for (u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++) {
        u8IRSwModeBuf[u8Byte] = 0;
    }

    if (_u32IRData[1] > IR_HDC_LOB && _u32IRData[1] < IR_HDC_UPB) {
        pu32IRData = &_u32IRData[2];
    } else {
        printk(KERN_EMERG "Invalid leader code\n");
        bRet = FALSE;
        goto done;
    }
    for (u8Bit=0; u8Bit<24; u8Bit++) {
        u32 u32BitLen = pu32IRData[u8Bit];
        u32IRDataTmp <<= 1;
        if (u32BitLen > IR_LG0_LOB && u32BitLen < IR_LG0_UPB ) {//0
            u32IRDataTmp |= 0x00UL;
        } else if (u32BitLen > IR_LG1_LOB && u32BitLen < IR_LG1_UPB) {//1
            u32IRDataTmp |= 0x01UL;
        } else {
            DEBUG_IR(printk("Invalid waveform,0x%x\n",u32BitLen));
            bRet = FALSE;
            goto done;
        }
    }

    u8IRSwModeBuf[0] = ((u32IRDataTmp & 0x00F00000UL) >> 20);
    u8IRSwModeBuf[1] = ((u32IRDataTmp & 0x000FF000UL) >> 12);
    u8IRSwModeBuf[2] = ((u32IRDataTmp & 0x00000F00UL) >> 8);
    u8IRSwModeBuf[3] = (u32IRDataTmp & 0x000000FFUL);
    if (u8IRSwModeBuf[0] == (((u8)~u8IRSwModeBuf[2]) & 0x0FUL)) {
        if (u8IRSwModeBuf[1] == (u8)~u8IRSwModeBuf[3]) {
            if ((_brepeat_flag == TRUE) && (u8IRSwModeBuf[1] == _u8PrevKeyCode)) {
                *pu8Key = _u8PrevKeyCode;
                *pu8Flag = 0x01UL;//(((u8IRSwModeBuf[0]<<4) & 0xF0)|0x01);
                *pu8System = (u8IRSwModeBuf[2]<<4)|u8IRSwModeBuf[0];
                _brepeat_flag = FALSE;
                bRet = TRUE;
                goto done;
            } else {
                *pu8Key = u8IRSwModeBuf[1];
                *pu8Flag = 0;//((u8IRSwModeBuf[0]<<4) & 0xF0);
                _u8PrevKeyCode=*pu8Key;
                *pu8System = (u8IRSwModeBuf[2]<<4)|u8IRSwModeBuf[0];
                bRet = TRUE;
                goto done;
            }
        }
    }
    DEBUG_IR(printk("Invalid data\n"));
    bRet = FALSE;

    done:
#ifndef CONFIG_MSTAR_IR_INPUT_DEVICE
    if (bRet) {
        if ((_u8PrevKeyCode != *pu8Key) || (!*pu8Flag)) {
            _ePrevKeyProperty = E_IR_KEY_PROPERTY_INIT;
        }

        i = _MDrv_IR_GetSystemTime();
        if (_ePrevKeyProperty == E_IR_KEY_PROPERTY_INIT) {
            _u8PrevKeyCode = *pu8Key;
            _ulPrevKeyTime    = i;
            _ulPrevKeyTimeFirstPress = i;
            _ePrevKeyProperty = E_IR_KEY_PROPERTY_1st;
            *pu8Flag = 0;
        } else if (_ePrevKeyProperty == E_IR_KEY_PROPERTY_1st) {
            if ((i - _ulPrevKeyTime) > 500) {//2*_u32_1stDelayTimeMs
                _ulPrevKeyTime = i;
                _ePrevKeyProperty = E_IR_KEY_PROPERTY_FOLLOWING;
                *pu8Flag = 0x01UL;
            } else {
                bRet = FALSE;
            }
        } else {
            if ((i - _ulPrevKeyTime) > 80) {//2*_u32_2ndDelayTimeMs
                _ulPrevKeyTime = i;
                *pu8Flag = 0x01UL;
            } else {
                bRet = FALSE;
            }

            if (_u8PrevKeyCode == 0x0BUL) {//OK key press
                if ((i-_ulPrevKeyTimeFirstPress) > 1500) {
                    _ulPrevKeyTimeFirstPress = i;
                    *pu8Key = 0x9AUL;
                    *pu8Flag = 0x00UL; //clear repeat flag
                    bRet = TRUE; //send long press key
                }
            }
        }
    }
#endif

    _u32IRCount = 0;
    return bRet;

 #elif(IR_TYPE_SEL == IR_TYPE_HAIER)
    //U8 u8tmp;

    if(_btoshiba_code)
    {
        if(_brepeat_flag)
        {

                if(_bBeginRepeat_flag)
                {
                    _Repeat_Num++;
                    if(_Repeat_Num == 1)
                    {
                        return FALSE;//drop first repeate for ir too smart
                    }
                    *pu8Key = _u8PrevKeyCode;
                    *pu8Flag = 1;
                    _u32IRCount = 0;
                    _btoshiba_code=FALSE;
                    _brepeat_flag=FALSE;
                    DEBUG_IR(printk("Input key is 0x%x\n", *pu8Key));
                    return TRUE;
                }
                else
                {
                    _u32IRCount = 0;
                    _btoshiba_code=FALSE;
                    _brepeat_flag=FALSE;
                    printk("[IR] Bad Repeat Receive\r\n");
                    return FALSE;
                }
        }

        if (_u32IRCount< (34))  //multiple 8
            return FALSE; //not complete yet
        else
        {
            for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
            {
               u8IRSwModeBuf[u8Byte] = 0;
            }
            pu32IRData = &_u32IRData[2];

            for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
            {
                for( u8Bit=0; u8Bit<8; u8Bit++)
                {
                    u32 u32BitLen = pu32IRData[u8Byte*8+u8Bit];
                    u8IRSwModeBuf[u8Byte] >>= 1;
                    if( u32BitLen > IR_LG0_LOB && u32BitLen < IR_LG0_UPB ) //0
                    {
                        u8IRSwModeBuf[u8Byte] |= 0x00UL;
                    }
                    else // if (u32BitLen > IR_LG1_LOB && u32BitLen < IR_LG1_UPB) //1
                    {
                        u8IRSwModeBuf[u8Byte] |= 0x80UL;
                    }
                    /*else
                    {
                        DEBUG_IR(printk(" invalid waveform,0x%x\n",u32BitLen));
                        for(u8tmp=0;u8tmp<_u32IRCount;u8tmp++)
                            printk("%d data=%d\n",u8tmp,_u32IRData[u8tmp]);
                        _u32IRCount = 0;
                        btoshiba_code=FALSE;
                        return FALSE;
                    }*/
                }

                if((u8IRSwModeBuf[0]!=_u8IRHeaderCode0)||(u8Byte==1&&u8IRSwModeBuf[1]!=_u8IRHeaderCode1))
                {
                    U8 u8tmp;
                    for(u8tmp=0;u8tmp<_u32IRCount;u8tmp++)
                            printk("[IR] %d data=%d\n",u8tmp,_u32IRData[u8tmp]);
                    _u32IRCount = 0;
                    _btoshiba_code=FALSE;
                    DEBUG_IR(printk("[IR] invalid header\n"));
                    return FALSE;
                }
                else if(u8Byte==3)
                {
                    if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
                    {
                        *pu8Key = u8IRSwModeBuf[2];
                        *pu8Flag = 0;
                        _u8PrevKeyCode=*pu8Key;
                        _u32IRCount = 0;
                        _btoshiba_code=FALSE;
                        _bBeginRepeat_flag = 1;
                        _Repeat_Num = 0;
                        DEBUG_IR(printk("[IR] input key is 0x%x\n",*pu8Key));
                        return TRUE;
                    }
                }
            }//for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
        }//else
    }//toshiba code
    else
    {
        if(_u32IRCount< (18))  //multiple 4
            return FALSE; //not complete yet
        else
        {
            for( u8Byte=0; u8Byte<2; u8Byte++)
            {
               u8IRSwModeBuf[u8Byte] = 0;
            }
            pu32IRData = &_u32IRData[2];
            for( u8Byte=0; u8Byte<2; u8Byte++)
            {
                for( u8Bit=0; u8Bit<8; u8Bit++)
                {
                    u32 u32BitLen = pu32IRData[u8Byte*8+u8Bit];
                    u8IRSwModeBuf[u8Byte] >>= 1;

                    if( u32BitLen > IR_LG0_LOB && u32BitLen < IR_LG0_UPB ) //0
                    {
                        u8IRSwModeBuf[u8Byte] |= 0x00UL;
                    }
                    else // if (u32BitLen > IR_LG1_LOB && u32BitLen < IR_LG1_UPB) //1
                    {
                        u8IRSwModeBuf[u8Byte] |= 0x80UL;
                    }
                    /*else
                    {
                        DEBUG_IR(printk(" invalid waveform,0x%x\n",u32BitLen));
                        bRet = FALSE;
                    }*/
                }
            }//for( u8Byte=0; u8Byte<2; u8Byte++)

            if(((u8IRSwModeBuf[0]&0xf0UL)>>4)==_u8FactoryIRHeaderCode0)
            {
                u8IRSwModeBuf[2]=u8IRSwModeBuf[0]&0x0fUL;
                u8IRSwModeBuf[3]=u8IRSwModeBuf[1]&0x0fUL;
                if((u8IRSwModeBuf[2]+u8IRSwModeBuf[3])==0x0fUL)
                {
                    switch(u8IRSwModeBuf[2])
                    {
                        case IRKEY_DESIL1:
                        case IRKEY_DESIL2:
                        case IRKEY_DESIL3:
                        case IRKEY_DESIL4:
                        case IRKEY_DESIL5:
                        case IRKEY_DESIL6:
                            *pu8Key=IRKEY_SLIDE_L1+(u8IRSwModeBuf[2]-IRKEY_DESIL1);
                             break;
                        case IRKEY_REVERSE1:
                        case IRKEY_REVERSE2:
                        case IRKEY_REVERSE3:
                        case IRKEY_REVERSE4:
                        case IRKEY_REVERSE5:
                        case IRKEY_REVERSE6:
                            *pu8Key=IRKEY_SLIDE_R1+(u8IRSwModeBuf[2]-IRKEY_REVERSE1);
                            break;
                    }
                    _u8PrevKeyCode=*pu8Key;
                    DEBUG_IR(printk("[IR] slide key 0x%x\n",*pu8Key));
                    printk("[IR] slide key 0x%x\n",*pu8Key);
                    *pu8Flag = 0;
                    bRet = TRUE;
                }
            }
            _u32IRCount = 0;
            _btoshiba_code=FALSE;
            return bRet;
        }
    }
    return bRet;

#elif (IR_TYPE_SEL == IR_TYPE_CUS08_RC5)

        if(_u8IrRc5Bits > IR_RC5_DATA_BITS)
        {
            if ((_u16IrRc5Data&0xF000UL)==0x3000UL)
            {
                if(_u16PreIrRc5Data==_u16IrRc5Data)
                    *pu8Flag = TRUE;

                *pu8Key = _u16IrRc5Data&0x3FUL;
                *pu8System = (_u16IrRc5Data&0x3C0UL)>>6;
                _u16PreIrRc5Data=_u16IrRc5Data;
                _u8IrRc5Bits=0;
                bRet = TRUE;
            }
        }

        if(bRet)
        {
            if ( (_u8PrevKeyCode != *pu8Key) || (!*pu8Flag) )
            {
                _ePrevKeyProperty = E_IR_KEY_PROPERTY_INIT;
            }

            i = _MDrv_IR_GetSystemTime();
            if( _ePrevKeyProperty == E_IR_KEY_PROPERTY_INIT)
            {
                _u8PrevKeyCode     = *pu8Key;
                _ulPrevKeyTime    = i;
                _ePrevKeyProperty  = E_IR_KEY_PROPERTY_1st;
            }
            else if(_ePrevKeyProperty == E_IR_KEY_PROPERTY_1st)
            {
                if( (i - _ulPrevKeyTime) > _u32_1stDelayTimeMs)
                {
                    _ulPrevKeyTime = i;
                    _ePrevKeyProperty  = E_IR_KEY_PROPERTY_FOLLOWING;
                }
                else
                {
                    bRet = FALSE;
                }
            }
            else //E_IR_KEY_PROPERTY_FOLLOWING
            {
                if( (i - _ulPrevKeyTime) > _u32_2ndDelayTimeMs)
                {
                    _ulPrevKeyTime = i;
                }
                else
                {
                    bRet = FALSE;
                }
            }
        }

        return bRet;

#elif (IR_TYPE_SEL == IR_TYPE_CUS21SH)

    #if (1)
    bRet = FALSE;

    if(_u32IRCount == 15)
    {
        if(_MDrv_SH_IR_GetKey(pu8Key, pu8System, pu8Flag))
        {
            bRet = TRUE;
        }
        _u32IRCount = 0;
        _u16IRKeyMap = 0;
    }
    #else
    //printk("Linux: %s:%s:%d \n",__FILE__,__FUNCTION__,__LINE__);
    bRet = FALSE;

    for( u8Byte=0; u8Byte<16/*IR_RAW_DATA_NUM*/; u8Byte++)
    {
       u8IRSwModeBuf[u8Byte] = 0;
    }

    if (_u32IRCount< 32)//3+IR_RAW_DATA_NUM*8)
        return FALSE; //not complete yet

    DEBUG_IR(printk("[IR] _u32IRCount=%d", _u32IRCount));
    for( u8Byte=0; u8Byte<_u32IRCount; u8Byte++)
    {
       DEBUG_IR(printk(" 0x%x", _u32IRData[u8Byte]));
    }
    printk("\n");
    printk("_u16IRKeyMap:0x%x \n",_u16IRKeyMap);

    if(_bKeyValueHit == FALSE)
    {
        *pu8Key = _MDrv_IR_CUS21SH_ParseKey(_u16IRKeyMap);    // translate for SH

        printk("_u16IRKeyMap:0x%x ==> *pu8Key:%x\n",_u16IRKeyMap,*pu8Key);
        //TODO: Implement repeat code later.
        *pu8Flag = 0;
        //_bKeyValueHit = 1;
        bRet = TRUE;
    }
    #endif

    return bRet;

#elif (IR_TYPE_SEL == IR_TYPE_TOSHIBA)

    bRet = FALSE;

    if(_MDrv_IR_CUS22T_GetKey(pu8Key, pu8System, pu8Flag))
    {
        bRet = TRUE;
    }

    return bRet;

#else
    for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
    {
       u8IRSwModeBuf[u8Byte] = 0;
    }
    #if (IR_TYPE_SEL == IR_TYPE_HISENSE || IR_TYPE_SEL == IR_TYPE_MSTAR_DTV || IR_TYPE_SEL == IR_TYPE_CHANGHONG)
    if(_u8IRRepeated)//repeate key
    {
        _u8IRRepeatedNum++;
        if (_u8IRRepeatedNum < 5)
        {
            return FALSE;
        }
        _u8IRRepeated = 0;
        _u8IRHeadReceived = 0;//clear head receive flag
        *pu8Key = _u8PrevKeyCode;
        *pu8Flag = 1;
        bRet = TRUE;
        #if (IR_TYPE_SEL == IR_TYPE_HISENSE)
        *pu8System = _u8PrevSystemCode;//timl
    if(_u8IRType == FCTORY_HEADER_RECEIVE)
            *pu8Flag |= FCTORY_HEADER_RECEIVE;
        else if(_u8IRType == PCCOMMAND_HEADER_RECEIVE)
            *pu8Flag |= PCCOMMAND_HEADER_RECEIVE;
        #endif

        goto done;
    }
    if (_u32IRCount<(2+IR_RAW_DATA_NUM*8))
        return FALSE; //not complete yet
    //#elif( IR_TYPE_SEL == IR_TYPE_CHANGHONG)
    //if (_u32IRCount<(2+IR_RAW_DATA_NUM*8))
        //return FALSE; //not complete yet
    #else
    if (_u32IRCount< 3+IR_RAW_DATA_NUM*8)
        return FALSE; //not complete yet
    #endif
    DEBUG_IR(printk("_u32IRCount=%d", _u32IRCount));
    for( u8Byte=0; u8Byte<_u32IRCount; u8Byte++)
    {
       DEBUG_IR(printk(" %d", _u32IRData[u8Byte]));
    }

    if( _u32IRData[0] > IR_HDC_LOB && _u32IRData[1] > IR_OFC_LOB+IR_LG01H_LOB && _u32IRData[1] < REG_IR_OFC_UPB+IR_LG01H_UPB )
    {
        pu32IRData = &_u32IRData[2];
        DEBUG_IR(printk(" H1 "));
    }
    else if( _u32IRData[1] > IR_HDC_LOB && _u32IRData[2] > IR_OFC_LOB+IR_LG01H_LOB && _u32IRData[2] < REG_IR_OFC_UPB+IR_LG01H_UPB )
    {
        pu32IRData = &_u32IRData[3];
        DEBUG_IR(printk(" H2 "));
    }
    else
    {
        DEBUG_IR(printk(" invalid leader code\n"));
        bRet = FALSE;
        goto done;
    }

    for( u8Byte=0; u8Byte<IR_RAW_DATA_NUM; u8Byte++)
    {
        for( u8Bit=0; u8Bit<8; u8Bit++)
        {
            u32 u32BitLen = pu32IRData[u8Byte*8+u8Bit];
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
            Shotcnt[u8Byte*8+u8Bit] = u32BitLen;
#endif
            u8IRSwModeBuf[u8Byte] >>= 1;

            if( u32BitLen > IR_LG0_LOB && u32BitLen < IR_LG0_UPB ) //0
            {
                u8IRSwModeBuf[u8Byte] |= 0x00UL;
            }
            else if (u32BitLen > IR_LG1_LOB && u32BitLen < IR_LG1_UPB) //1
            {
                u8IRSwModeBuf[u8Byte] |= 0x80UL;
            }
            else
            {
                DEBUG_IR(printk(" invalid waveform,0x%x\n",u32BitLen));
                bRet = FALSE;
                goto done;
            }
        }
    }

    #if (IR_TYPE_SEL == IR_TYPE_HISENSE)
    *pu8System = u8IRSwModeBuf[1];//timl
    #endif

#ifdef CONFIG_MSTAR_DYNAMIC_IR

    do{
        int headcodeindex;
        for (headcodeindex = 0; headcodeindex < g_headcodecount; headcodeindex ++)
        {
            if (u8IRSwModeBuf[0] != headcode0array[headcodeindex])
                continue;
            if (u8IRSwModeBuf[1] != headcode1array[headcodeindex])
                continue;
            break;
        }
        if (headcodeindex != g_headcodecount)
        {
            /*
            Find the specific head code.
            */
            if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
            {
                U16 scancode = (U16)((headcodeindex << 8) + u8IRSwModeBuf[2]);
                if(debug_flag) printk(KERN_DEBUG "[IR] scancode = 0x%x\n", scancode);
                *pu8Key = translate_scancode(scancode);
                if(debug_flag) printk(KERN_DEBUG "[IR] keycode = 0x%x\n", *pu8Key);
                *pu8Flag = 0;
                bRet = TRUE;
            #if (IR_TYPE_SEL == IR_TYPE_HISENSE || IR_TYPE_SEL == IR_TYPE_MSTAR_DTV || IR_TYPE_SEL == IR_TYPE_CHANGHONG)
                _u8PrevKeyCode = *pu8Key;
                _u8PrevSystemCode = *pu8System;
                _u8IRRepeateDetect = 1;
                _u8IRHeadReceived = 0;
                _u8IRType = 0;
            #endif

                goto done;
            }
        }
    }while(0);
#else
if(u8IRSwModeBuf[0] == _u8IRHeaderCode0)
    {
        if(u8IRSwModeBuf[1] == _u8IRHeaderCode1)
        {
            if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
            {
                *pu8Key = u8IRSwModeBuf[2];
                *pu8Flag = 0;
                bRet = TRUE;
                #if (IR_TYPE_SEL == IR_TYPE_HISENSE || IR_TYPE_SEL == IR_TYPE_MSTAR_DTV || IR_TYPE_SEL == IR_TYPE_CHANGHONG)
                _u8PrevKeyCode = *pu8Key;
                _u8PrevSystemCode = *pu8System;
                _u8IRRepeateDetect = 1;
                _u8IRHeadReceived = 0;
                _u8IRType = 0;
                #endif

                goto done;
            }
        }
    }


    if(u8IRSwModeBuf[0] == _u8IR2HeaderCode0)
    {
        if(u8IRSwModeBuf[1] == _u8IR2HeaderCode1)
        {
            if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
            {
                *pu8Key = u8IRSwModeBuf[2];
                *pu8Flag = 0;
                bRet = TRUE;
                #if (IR_TYPE_SEL == IR_TYPE_HISENSE || IR_TYPE_SEL == IR_TYPE_MSTAR_DTV || IR_TYPE_SEL == IR_TYPE_CHANGHONG)
                _u8PrevKeyCode = *pu8Key;
                _u8PrevSystemCode = *pu8System;
                _u8IRRepeateDetect = 1;
                _u8IRHeadReceived = 0;
                _u8IRType = 0;
                #endif

                goto done;
            }
        }
    }
#endif
    #if (IR_TYPE_SEL == IR_TYPE_HISENSE)
    if(u8IRSwModeBuf[0] == _u8FactoryIRHeaderCode0)
    {
        if(u8IRSwModeBuf[1] == _u8FactoryIRHeaderCode1)
        {
            if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
            {
                *pu8Key = u8IRSwModeBuf[2];
                *pu8Flag = 0;
                *pu8Flag |= FCTORY_HEADER_RECEIVE;
                bRet = TRUE;
                _u8PrevKeyCode = *pu8Key;
                _u8PrevSystemCode = *pu8System;
                _u8IRRepeateDetect = 1;
                _u8IRHeadReceived = 0;
                _u8IRType = FCTORY_HEADER_RECEIVE;

                goto done;
            }
        }
    }

    if(u8IRSwModeBuf[0] == _u8PCCommandHeaderCode0)
    {
        if(u8IRSwModeBuf[1] == _u8PCCommandHeaderCode1)
        {
            if(u8IRSwModeBuf[2] == (u8)~u8IRSwModeBuf[3])
            {
                *pu8Key = u8IRSwModeBuf[2];
                *pu8Flag = 0;
                *pu8Flag |= PCCOMMAND_HEADER_RECEIVE;
                bRet = TRUE;
                _u8PrevKeyCode = *pu8Key;
                _u8PrevSystemCode = *pu8System;
                _u8IRRepeateDetect = 1;
                _u8IRHeadReceived = 0;
                _u8IRType = PCCOMMAND_HEADER_RECEIVE;

                goto done;
            }
        }
    }
    #endif

    DEBUG_IR(printk("[IR] invalid data\n"));
    bRet = FALSE;

done:
    _u32IRCount = 0;
    return bRet;
#endif
}

#elif (IR_MODE_SEL == IR_TYPE_SWDECODE_KON_MODE)
//-------------------------------------------------------------------------------------------------
/// Get IR key.
/// @param pu8Key  \b IN: Return IR key value.
/// @param pu8Flag \b IN: Return IR repeat code.
///
/// @return TRUE:  Success
/// @return FALSE: Failure
//-------------------------------------------------------------------------------------------------
static BOOL _MDrv_IR_GetKey(U8 *pu8Key, U8 *pu8System, U8 *pu8Flag)
{
    U8 u8Temp;

    *pu8System = 0;

    if(g_u8IrKonFlag & IR_KON_DETECT_END)
    {
        printk("\n[IR] Key:%x",g_u16IrKonDecode);
        if (((g_u16IrKonDecode >> 8) & 0xFFUL) != _u8IRHeaderCode0)
        {
            g_u8IrKonFlag &= (~IR_KON_DETECT_END);
            printk("\n[IR] Invalid HC!");
            return FALSE;
        }

        u8Temp = (U8)(g_u16IrKonDecode & 0x00FFUL);
        *pu8Key = u8Temp;

        if ( (g_u8IrKonFlag & IR_KON_REPEATE_TIMEOUT) || (g_u8IrKonPreKey != u8Temp)
            || (*pu8Key==70) || (*pu8Key==69))
        {
            *pu8Flag = 0;
            g_u8IrKonPreKey = u8Temp;
        }
        else
        {
            *pu8Flag = 1;
        }
//        printk("\n0x%x(%d)\n", (U16)(*pu8Key), (U16)(*pu8Flag));
        g_u8IrKonFlag &= (~IR_KON_DETECT_END);

        if (*pu8Flag == 0)
        {
            g_u8IrKonRepeatCount = 0;
            return TRUE;
        }
        else
        {
            g_u8IrKonRepeatCount++;
//#if 0    //modified by caomeng 20090713 for power key sensitivity
            if (g_u8IrKonRepeatCount >= 10)//IR_FILTER_REPEAT_NUM)
                return TRUE;
//#else
//            if ((u8Temp == IRKEY_POWER ||u8Temp == IRKEY_POWER_KEYPAD)&&
//            (g_u8IrKonRepeatCount >= 20))    //IR_FILTER_REPEAT_NUM
//                return MSRET_OK;
//         else if(g_u8IrKonRepeatCount >= 10)
//                return MSRET_OK;
//#endif
        }
    }
    return FALSE;
}
#endif

void MDrv_IR_Disable_SWFIFO(void)
{
#if (IR_SWFIFO_DECODE_MODE==ENABLE)
     REG(REG_IR_SEPR_BIT_FIFO_CTRL) &= ~(0x1UL <<14);
     REG(REG_IR_SEPR_BIT_FIFO_CTRL) &= ~(0x1UL <<6);
#endif
}

static unsigned int bIRDriverInit = 0;
//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/// Initialize IR timing and enable IR.
/// @return None
//-------------------------------------------------------------------------------------------------
void MDrv_IR_Init(int bResumeInit)
{
    int result;

    if( (bIRDriverInit) && ( !bResumeInit ) )
    {
        pr_info("we had aready init driver, return\n");
        return;
    }

    bIRDriverInit = 1;
    ir_irq_depth = 1;
    _u8PrevSystemCode = 0;

   //Prevent residual value exist in sw fifo when system wakeup from standby status
   REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= 0x8000UL; //clear IR FIFO(Trigger reg_ir_fifo_clr)

#ifdef CONFIG_MSTAR_DYNAMIC_IR
    dynamic_ir_mode_init();
#endif

#ifdef SUPPORT_MULTI_PROTOCOL
    _MDrv_MultiProtocol_Init();
#endif

#if (IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
    _u8IRRawModeCount = 0;
#endif
    #if(IR_MODE_SEL == IR_TYPE_SWDECODE_KON_MODE)
    g_u8IrKonBits = 0;
    g_u8IrKonFlag  = 0;
    g_u8IrKonRepeatCount = 10;
    g_u8IrKonRepeatTimeout = 0;
    g_u8IrKonPreKey = 0xFFUL;
    #endif
    if(!bResumeInit)
    {
    sema_init(&fantasy_protocol.sem, 1);
    init_waitqueue_head(&fantasy_protocol.inq);
    }
    fantasy_protocol.data=0;
#if defined(CONFIG_MSTAR_GPIO) && defined(CONFIG_MSTAR_IR_GPIO_TOGGLE)
    if (!bResumeInit) {
        init_completion(&ir_gpio_completion);
        gpio_toggle_workqueue = create_workqueue("keygpio_wq");
        queue_work(gpio_toggle_workqueue, &gpio_toggle_thread);
    }
#endif

#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
#if(IR_TYPE_SEL != IR_TYPE_RCMM)
    _u32IRCount = 0;
#endif
#if (IR_TYPE_SEL == IR_TYPE_HAIER)
    _bBeginRepeat_flag = 0;
    _Repeat_Num = 0;

#endif

#endif


    if(IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
    {
        REG(REG_IR_CTRL) = IR_TIMEOUT_CHK_EN |
                                   IR_INV            |
                                   IR_RPCODE_EN      |
                                   IR_LG01H_CHK_EN   |
                                   IR_LDCCHK_EN      |
                                   IR_EN;
    }
    else if(IR_MODE_SEL == IR_TYPE_HWRC_MODE)
    {
        REG(REG_IR_CTRL) = IR_INV;
#if (defined(IR_TYPE_HWRC5) && IR_TYPE_SEL==IR_TYPE_HWRC5)
      REG(REG_IR_RC_CTRL) = IR_RC_EN;
#endif
    }
    else
    {
        REG(REG_IR_CTRL) = IR_TIMEOUT_CHK_EN |
                                   IR_INV            |
                                   IR_RPCODE_EN      |
                                   IR_LG01H_CHK_EN   |
                                   IR_DCODE_PCHK_EN  |
                                   IR_CCODE_CHK_EN   |
                                   IR_LDCCHK_EN      |
                                   IR_EN;
    }

    _MDrv_IR_Timing();
    if(IR_MODE_SEL != IR_TYPE_HWRC_MODE)
    {
      REG(REG_IR_CCODE) = ((u16)_u8IRHeaderCode1<<8) | _u8IRHeaderCode0;
      REG(REG_IR_GLHRM_NUM) = 0x804UL;
    }

    REG(REG_IR_SEPR_BIT_FIFO_CTRL) = 0xF00UL;//[10:8]: FIFO depth, [11]:Enable FIFO full
#if (IR_MODE_SEL==IR_TYPE_FULLDECODE_MODE)
    REG(REG_IR_GLHRM_NUM) |= (0x3UL <<12);
    REG(REG_IR_FIFO_RD_PULSE) |= 0x0020UL; //wakeup key sel
#elif (IR_MODE_SEL==IR_TYPE_RAWDATA_MODE)
    REG(REG_IR_GLHRM_NUM) |= (0x2UL <<12);
    REG(REG_IR_FIFO_RD_PULSE) |= 0x0020UL; //wakeup key sel
#elif (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
    //wakeup key sel
    REG(REG_IR_RC_COMP_KEY1_KEY2) = 0xffffUL;
    REG(REG_IR_RC_CMP_RCKEY) = IR_RC_POWER_WAKEUP_EN + IR_RC_POWER_WAKEUP_KEY;
#else
    REG(REG_IR_GLHRM_NUM) |= (0x1UL <<12);
#if(IR_TYPE_SEL == IR_TYPE_RCMM)

    REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= 0x1 <<12;

#else
    #ifdef IR_INT_NP_EDGE_TRIG    //for N/P edge trigger
        REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= 0x3UL <<12;
    #else
        REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= 0x2UL <<12;
    #endif
#endif
#endif


#if((IR_MODE_SEL==IR_TYPE_RAWDATA_MODE) || (IR_MODE_SEL==IR_TYPE_FULLDECODE_MODE) || (IR_MODE_SEL == IR_TYPE_HWRC_MODE))
    // Empty the FIFO
    _MDrv_IR_ClearFIFO();
#endif

#if (IR_SWFIFO_DECODE_MODE==ENABLE)
     REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= (0x1UL <<14);
     REG(REG_IR_CKDIV_NUM_KEY_DATA) = 0x00CFUL ;
     REG(REG_IR_SEPR_BIT_FIFO_CTRL) |= (0x1UL <<6);

    _MDrv_IR_ClearFIFO();
#endif




    _u32_1stDelayTimeMs = 0;
    _u32_2ndDelayTimeMs = 0;
    _ePrevKeyProperty = E_IR_KEY_PROPERTY_INIT;

    if (!bResumeInit) {
#ifdef CONFIG_MSTAR_PM_SWIR
    //result = request_irq(E_FIQ_IR, MDrv_SWIR_PM51_ISR, SA_INTERRUPT, "IR", NULL); //Doyle change ISR
#elif (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
    result = request_irq(E_FIQEXPL_IR_INT_RC, _MDrv_IR_RC_ISR, SA_INTERRUPT, "IR_RC", NULL);
#else
    result = request_irq(INT_NUM_IR_ALL, _MDrv_IR_ISR, SA_INTERRUPT, "IR", NULL);
#endif

    ir_irq_depth = 0;
    if (result) {
        printk(KERN_EMERG "IR IRQ registartion ERROR\n");
    } else{
        printk("IR IRQ registartion OK\n");
    }
}

#ifdef SUPPORT_MULTI_PROTOCOL
    memset(_eDetectList, 0, sizeof(_eDetectList));
    _eDetectList[0] = E_IR_PROTOCOL_NEC; //default used protocol
#endif

    #if (defined(CONFIG_MSTAR_TITANIA)||defined(CONFIG_MSTAR_TITANIA2))

    #else
    // unmask IR IRQ on PM
    #ifdef CONFIG_MSTAR_PM_SWIR //to -do
    REG(REG_IRQ_MASK_IR) |= 0x800UL;
    #else
    //REG(REG_IRQ_MASK_IR) &= IRQ_UNMASK_IR;
    #endif
    #endif
    //enable_irq(E_FIQ_IR);

    #ifdef CONFIG_MSTAR_PM_SWIR
    MBX_Result MbxResult=E_MBX_UNKNOW_ERROR;

extern int MDrv_MBX_NotifyMsgRecCbFunc(void *);

    if( E_MBX_SUCCESS != MDrv_MBX_Init_Async(SWIR_FILEOPS, E_MBX_CPU_MIPS, E_MBX_ROLE_HK, IR_MBX_TIMEOUT))
    {
        swir_pm_status = -1;
    }
    else
    {
        MDrv_MBX_Enable_Async(SWIR_FILEOPS, TRUE);
        swir_pm_status = 1;

        MbxResult = MDrv_MBX_RegisterMSG_Async(SWIR_FILEOPS, E_MBX_CLASS_IRKEY_NOWAIT, IR_MBX_QUEUESIZE);

        MDrv_MBX_NotifyMsgRecCbFunc(MDrv_PMSWIR_ReceiveMsg);



        //send msg to PM to unlock interrupt mask
        MBX_Msg MB_Command;
        MS_U8 password[] = {'M', 'S', 'T', 'A', 'R'};
        int i;

        memset((void*)&MB_Command, 0, sizeof(MBX_Msg));
        MB_Command.eRoleID = E_MBX_ROLE_PM;
        MB_Command.eMsgType = E_MBX_MSG_TYPE_INSTANT;
        MB_Command.u8Ctrl = 0;
        MB_Command.u8MsgClass = E_MBX_CLASS_IRKEY_NOWAIT;
        MB_Command.u8Index = E_IR_CPUTo51_CMD_SWIR_INT;
        MB_Command.u8ParameterCount = sizeof(password);
        for(i = 0; i < sizeof(password); i++)
            MB_Command.u8Parameters[i] = password[i];
        MbxResult = MDrv_MBX_SendMsg_Async(SWIR_FILEOPS, &MB_Command, 1);
    }
    #endif
}
EXPORT_SYMBOL(MDrv_IR_Init);

//-------------------------------------------------------------------------------------------------
/// Set the initialized control parameters for IrDa at BOOT stage.
/// @param  pIRInitCfg \b IN: carry with initialized control parameters.
/// @return TRUE/FALSE
//-------------------------------------------------------------------------------------------------
BOOL MDrv_IR_InitCfg(MS_IR_InitCfg* pIRInitCfg)
{
    if (pIRInitCfg->u8DecMode != IR_DECMODE_EXT)
    {
        return FALSE;
    }

    _u8IRHeaderCode0 = pIRInitCfg->u8HdrCode0;
    _u8IRHeaderCode1 = pIRInitCfg->u8HdrCode1;

    if (pIRInitCfg->u8ExtFormat == IR_XFM_DUALRC)
    {
        _u8FactoryIRHeaderCode0 = pIRInitCfg->u8Ctrl0;
        _u8FactoryIRHeaderCode1 = pIRInitCfg->u8Ctrl1;
    }
    REG(REG_IR_CCODE) = ((u16)_u8IRHeaderCode1<<8) | _u8IRHeaderCode0;

    return TRUE;

}
EXPORT_SYMBOL(MDrv_IR_InitCfg);

//-------------------------------------------------------------------------------------------------
/// Set which protocol will be used in multiple protocol mode.
/// @param  pstProtocolCfg \b IN: carry with used protocol parameter.
/// @return TRUE/FALSE
//-------------------------------------------------------------------------------------------------
BOOL MDrv_IR_SetProtocol(MS_MultiProtocolCfg *pstProtocolCfg)
{
#ifdef SUPPORT_MULTI_PROTOCOL
    U8 u8Count=0;

    if(pstProtocolCfg==NULL)
    {
        return FALSE;
    }

    memset(_eDetectList, 0, sizeof(_eDetectList));
    _eDetectList[u8Count]=pstProtocolCfg->eProtocol;
    u8Count++;
    while(pstProtocolCfg->pNextProtCfg!=NULL && u8Count<PROTOCOL_SUPPORT_MAX)
    {
        pstProtocolCfg = (MS_MultiProtocolCfg *)pstProtocolCfg->pNextProtCfg;
        _eDetectList[u8Count]=pstProtocolCfg->eProtocol;
        u8Count++;
    }

    return TRUE;
#else
    printk("[IR] Multiple protocol not support");
    return FALSE;
#endif
}
EXPORT_SYMBOL(MDrv_IR_SetProtocol);

//-------------------------------------------------------------------------------------------------
/// Set the initialized time parameters for IrDa at BOOT stage.
/// @param  pIRTimeCfg \b IN: carry with initialized time parameter.
/// @return TRUE/FALSE
//-------------------------------------------------------------------------------------------------
BOOL MDrv_IR_TimeCfg(MS_IR_TimeCfg* pIRTimeCfg)
{
    pIRTimeCfg = pIRTimeCfg;
    return TRUE;

}
EXPORT_SYMBOL(MDrv_IR_TimeCfg);

//-------------------------------------------------------------------------------------------------
/// Get the shot count information from ping-pong/ring buffer
/// @param  pstShotInfo \b OUT: Get the read index,length and shot count sequence.
/// @return TRUE/FALSE
//-------------------------------------------------------------------------------------------------
BOOL MDrv_IR_ReadShotBuffer(MS_IR_ShotInfo* pstShotInfo)//--@@@--IR Pulse Shot Mode
{
    pstShotInfo = pstShotInfo;
    return TRUE;
}
EXPORT_SYMBOL(MDrv_IR_ReadShotBuffer);

//-------------------------------------------------------------------------------------------------
/// Set the IR delay time to recognize a valid key.
/// @param  u32_1stDelayTimeMs \b IN: Set the delay time to get the 1st key.
/// @param  u32_2ndDelayTimeMs \b IN: Set the delay time to get the following keys.
/// @return None
//-------------------------------------------------------------------------------------------------
void MDrv_IR_SetDelayTime(u32 u32_1stDelayTimeMs, u32 u32_2ndDelayTimeMs)
{
//TBD
    //u32 u32OldIntr;

    //u32OldIntr = MsOS_DisableAllInterrupts();

    _u32_1stDelayTimeMs = u32_1stDelayTimeMs;
    _u32_2ndDelayTimeMs = u32_2ndDelayTimeMs;

    //MsOS_RestoreAllInterrupts(u32OldIntr);
}
EXPORT_SYMBOL(MDrv_IR_SetDelayTime);

//-----------------------------------------------------------------------------------------------
///set Multi ir header code
//-----------------------------------------------------------------------------------------------
BOOL MDrv_IR_SetHeaderCode(MS_MultiIR_HeaderInfo* pMIrHeaderInfo) {

    _u8IRHeaderCode0 = pMIrHeaderInfo->_u8IRHeaderCode0;
    _u8IRHeaderCode1 = pMIrHeaderInfo->_u8IRHeaderCode1;
#if (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
    _u8IR2HeaderCode0 = pMIrHeaderInfo->_u8IR2HeaderCode0;
    _u8IR2HeaderCode1 = pMIrHeaderInfo->_u8IR2HeaderCode1;
    printk("\r[IR][%x][%x][%x][%x]\n",_u8IRHeaderCode0,_u8IRHeaderCode1,_u8IR2HeaderCode0,_u8IR2HeaderCode1);
#endif
    return TRUE;
}
EXPORT_SYMBOL(MDrv_IR_SetHeaderCode);

void MDrv_IR_EnableIR(U8 bEnable) {
    bIRPass = !bEnable;
    if (bEnable) {
        if (ir_irq_depth > 0) {
            enable_irq(INT_NUM_IR_ALL);
            #if defined(INT_NUM_IR_INT_RC) //some chip's ir_int_rc are not covered by INT_NUM_IR_INT
            {
              enable_irq(INT_NUM_IR_INT_RC);
            }
            #endif
            ir_irq_depth--;
        }
    } else {
        disable_irq(INT_NUM_IR_ALL);
        #if defined(INT_NUM_IR_INT_RC) //some chip's ir_int_rc are not covered by INT_NUM_IR_INT
        {
          disable_irq(INT_NUM_IR_INT_RC);
        }
        #endif
        ir_irq_depth++;
        //free_irq(E_FIQ_IR, NULL);
    }
}

ssize_t MDrv_IR_Read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
#ifdef SUPPORT_MULTI_PROTOCOL
    if (count == sizeof(long long))
#else
    if (count == sizeof(U32))
#endif
    {
        if (down_interruptible(&fantasy_protocol.sem)) {
            return -ERESTARTSYS;
        }

        while (fantasy_protocol.data == 0) {
            up(&fantasy_protocol.sem); /* release the lock */
            if (filp->f_flags & O_NONBLOCK) {
                return -EAGAIN;
            }

            if (wait_event_interruptible(fantasy_protocol.inq, (fantasy_protocol.data!=0))) {
                return -ERESTARTSYS;
            }

            if (down_interruptible(&fantasy_protocol.sem)) {
                return -ERESTARTSYS;
            }
        }

#ifdef CONFIG_MSTAR_SOFTWARE_IR_MODULE
        if (software_ir_enable()) {
            take_over_by_software_ir_dfb(u8Key_for_mdrv_software_ir, u8RepeatFlag_for_mdrv_software_ir);
            set_software_ir_processing_undone();
            while (software_ir_processing_undone()) {
                ;
            }
            set_software_ir_processing_undone();
        }
#endif

#ifdef SUPPORT_MULTI_PROTOCOL
        if (copy_to_user(buf, &fantasy_protocol.data, sizeof(long long)))
#else
        if (copy_to_user(buf, &fantasy_protocol.data, sizeof(U32)))
#endif
        {
            up(&fantasy_protocol.sem);
            return -EFAULT;
        }

        //success
        fantasy_protocol.data=0;
        up(&fantasy_protocol.sem);

        return count;
    } else {
        return 0;
    }
}
EXPORT_SYMBOL(MDrv_IR_Read);

unsigned int MDrv_IR_Poll(struct file *filp, poll_table *wait) {
    unsigned int mask=0;

    down(&fantasy_protocol.sem);
    poll_wait(filp, &fantasy_protocol.inq, wait);
    if (fantasy_protocol.data != 0) {
        mask |= POLLIN|POLLRDNORM;
    }
    up(&fantasy_protocol.sem);

    return mask;
}
EXPORT_SYMBOL(MDrv_IR_Poll);

void MDrv_IR_SendKey(U8 u8Key, U8 u8RepeatFlag) {
#if defined(IR_TYPE_HISENSE) && (IR_TYPE_SEL == IR_TYPE_HISENSE)//for hisense need suport 2 ir head code ;
    U32 hisense_factory_ir = 0;
    if (u8RepeatFlag&FCTORY_HEADER_RECEIVE) {
        u8RepeatFlag &= (~FCTORY_HEADER_RECEIVE);
        hisense_factory_ir = (0x04UL << 28);
    } else if(u8RepeatFlag&PCCOMMAND_HEADER_RECEIVE) {
        u8RepeatFlag &= (~PCCOMMAND_HEADER_RECEIVE);
        hisense_factory_ir = (0x08UL << 28);
    } else {
        hisense_factory_ir = (0x01UL << 28);
    }
    fantasy_protocol.data = (u8Key<<8 | u8RepeatFlag);
    fantasy_protocol.data |= hisense_factory_ir;
#else
    fantasy_protocol.data = (u8Key<<8 | u8RepeatFlag);
    fantasy_protocol.data |= (0x01 << 28); //set fantasy class to IR
#endif
    wake_up_interruptible(&fantasy_protocol.inq);
}
EXPORT_SYMBOL(MDrv_IR_SendKey);

void MDrv_IR_SetMasterPid(pid_t pid)
{
    MasterPid = pid;
}

pid_t MDrv_IR_GetMasterPid(void)
{
    return MasterPid;
}

#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
static DECLARE_WORK(key_dispatch_thread,key_dispatch);
static struct workqueue_struct *key_dispatch_workqueue;
int MDrv_IR_Input_Init(void) {
#if (IR_TYPE_SEL == IR_TYPE_CHANGHONG)
    char *map_name = RC_MAP_CHANGHONG_TV;
    char *input_name = "ChangHong Smart TV IR Receiver";
    __u16 vendor_id = 0x0037UL;
#elif (IR_TYPE_SEL == IR_TYPE_HAIER)
    char *map_name = RC_MAP_HAIER_TV;
    char *input_name = "Haier Smart TV IR Receiver";
    __u16 vendor_id = 0x201eUL;
#elif (IR_TYPE_SEL == IR_TYPE_HISENSE)
    char *map_name = RC_MAP_HISENSE_TV;
    char *input_name = "Hisense Smart TV IR Receiver";
    __u16 vendor_id = 0x0018UL;
#elif (IR_TYPE_SEL == IR_TYPE_KONKA)
    char *map_name = RC_MAP_KONKA_TV;
    char *input_name = "Konka Smart TV IR Receiver";
    __u16 vendor_id = 0x0416UL;
#elif (IR_TYPE_SEL == IR_TYPE_SKYWORTH)
    char *map_name = RC_MAP_SKYWORTH_TV;
    char *input_name = "Skyworth Smart TV IR Receiver";
    __u16 vendor_id = 0x1918UL;
#elif (IR_TYPE_SEL == IR_TYPE_TCL)
    char *map_name = RC_MAP_TCL_TV;
    char *input_name = "Tcl Smart TV IR Receiver";
    __u16 vendor_id = 0x0019UL;
#elif (IR_TYPE_SEL == IR_TYPE_SWRC6)
    char *map_name = RC_MAP_RC6_6AMODE;
    char *input_name = "MStar Smart TV IR Receiver";
    __u16 vendor_id = 0x3697UL;
#else
    char *map_name = RC_MAP_MSTAR_TV;
    char *input_name = "MStar Smart TV IR Receiver";
    __u16 vendor_id = 0x3697UL;
#endif

    int err = 0;
    struct rc_dev *dev;

    ir = kzalloc(sizeof(struct mstar_ir), GFP_KERNEL);
    dev = rc_allocate_device();
    if (!ir || !dev)
        return -ENOMEM;

    // init input device
    ir->dev = dev;

    dev->driver_name = MDRV_NAME_IR;
    dev->map_name = map_name;
    dev->driver_type = RC_DRIVER_IR_RAW;
    dev->input_name = input_name;
    dev->input_phys = "/dev/ir";
    dev->input_id.bustype = BUS_I2C;
    dev->input_id.vendor = vendor_id;
    dev->input_id.product = 0x0001UL;
    dev->input_id.version = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
    dev->allowed_protocols = RC_TYPE_ALL;
#else
    dev->allowed_protos = RC_TYPE_ALL;
#endif
    err = rc_register_device(dev);
    if (err) {
        rc_free_device(dev);
        kfree(ir);
        return err;
    }

    // No auto-repeat.
    clear_bit(EV_REP, ir->dev->input_dev->evbit);

    init_completion(&key_completion);
    key_dispatch_workqueue = create_workqueue("keydispatch_wq");
    queue_work(key_dispatch_workqueue, &key_dispatch_thread);
    return 0;
}

void MDrv_IR_Input_Exit(void) {
    destroy_workqueue(key_dispatch_workqueue);
    rc_register_device(ir->dev);
    rc_free_device(ir->dev);
    kfree(ir);
}
#endif // CONFIG_MSTAR_IR_INPUT_DEVICE


#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)

static ssize_t mdb_node_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    printk("\n*************** echo help ***************\n");
    printk("mdebug IR only support read command now \n");
    printk("*****************************************\n");

    return count;
}

static ssize_t mdb_node_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    printk("---------MStar HW IR Info---------\n");
#if(IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
    printk("IR_protocol             : Hardware Decode nec\n");
#elif (IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
    printk("IR_protocol             : Hardware Decode raw\n");
#elif (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
    printk("IR_protocol             : Hardware Decode rc\n");
#elif (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE)
    printk("IR_protocol             : Software Decode\n");
#elif (IR_MODE_SEL == IR_TYPE_SWDECODE_KON_MODE)
    printk("IR_protocol             : Software Decode KON Mode\n");
#endif

#if(IR_SWFIFO_DECODE_MODE == ENABLE)
    printk("IR_protocol             : Software FIFO Enable\n");
#endif

    printk("IR_bRepKey              : %d\n",IR_bRepKey);
    printk("IR_KeyValue             : %d\n",IR_KeyValue);
    printk("IR_HeaderCode1          : 0x%x\n",IR_HeaderCode1);
    printk("IR_HeaderCode2          : 0x%x\n",IR_HeaderCode2);
    printk("IR_TimeoutCycle         : %d\n",IR_TIMEOUT_CYC);
#ifdef CONFIG_MSTAR_IR_INPUT_DEVICE
    printk("IR_EventTimeout         : %d\n\n\n",IR_EVENT_TIMEOUT);
#endif
    printk("---------MStar SW IR Info---------\n");
#if(IR_MODE_SEL == IR_TYPE_FULLDECODE_MODE)
    printk("IR_protocol             : nec\n");
    printk("Logic 0  => 560us H + 560us  L\n");
    printk("Logic 1  => 560us H + 1680us L\n");
#elif (IR_MODE_SEL == IR_TYPE_RAWDATA_MODE)
    printk("IR_protocol             : raw\n");
    printk("Logic 0  => 560us H + 560us  L\n");
    printk("Logic 1  => 560us H + 1680us L\n");
#elif (IR_MODE_SEL == IR_TYPE_HWRC_MODE)
    printk("IR_protocol             : rc5\n");
    printk("Logic 0  => 888us H + 888us L\n");
    printk("Logic 1  => 888us L + 888us H\n");
    printk("IR_protocol             : rc6\n");
    printk("Logic 0  => 444us L + 444us H\n");
    printk("Logic 1  => 444us H + 444us L\n");
#elif (IR_MODE_SEL == IR_TYPE_SWDECODE_MODE) || (IR_MODE_SEL == IR_TYPE_SWDECODE_KON_MODE)
    int i ;

    for(i = 0 ; i<32 ; i++)
    {
        printk("Shotcnt[%d]             : %d\n",i,Shotcnt[i]);
    }
#endif

    return 0;
}

struct file_operations proc_mdb_node_operations = {
    .write      = mdb_node_proc_write,
    .read       = mdb_node_proc_read,
};

#endif
