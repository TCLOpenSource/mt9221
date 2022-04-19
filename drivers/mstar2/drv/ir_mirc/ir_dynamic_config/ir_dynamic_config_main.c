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
/// file    ir_dynamic_config_main.c
/// @brief  Load IR config from INI config file
/// @author Vick.Sun@MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////



#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/reboot.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/init.h>

#include "ir_dynamic_config.h"
#include "iniparser.h"

static int debug = 1;
#define STRINIT(g,h) (g h)
#if defined(CONFIG_DATA_SEPARATION)
#define PROJECT_ID_LEN 5
#define PROJECT_ID_BUF_SIZE  50 //project_id.ini length
#define DATA_INDEX_NAME_BUF_SIZE  100//dataIndex.ini path length
#define IR_CONFIG_FILE_NAME_BUF_SIZE  100//ir_config.ini path length
#endif
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug:\n\t0: error only\n\t1:debug info\n\t2:all\n");

#if defined(CONFIG_DATA_SEPARATION)
static char *config_file = NULL;
static char *purelinux_config_file = NULL;
#else
#ifdef CONFIG_AMAZON_RECOVERY_MODE_IR
static char ir_config_path_recovery[IR_CONFIG_PATH_LEN] = "/tvconfig_recovery/config/ir_config.ini";
static char *config_file;
#else
static char ir_config_path[IR_CONFIG_PATH_LEN] = CONFIG_IR_CONFIG_PATH;
static char *config_file = ir_config_path;
#endif
static char *purelinux_config_file = "/config/ir_config.ini";
#endif
module_param(purelinux_config_file,charp, 0644);
MODULE_PARM_DESC(purelinux_config_file, "config file path");

module_param(config_file, charp, 0644);
MODULE_PARM_DESC(config_file, "config file path");

#define DBG_ALL(__fmt,args...)              do{if (2 <= debug){printk("[IR config]"__fmt,##args);}}while(0)
#define DBG_INFO(__fmt,args...)             do{if (1 <= debug){printk("[IR config]"__fmt,##args);}}while(0)
#define DBG_ERR(__fmt,args...)              do{if (0 <= debug){printk("[IR config]"__fmt,##args);}}while(0)

#define SIZE(array)  (sizeof(array)/sizeof(array[0]))
#define PM51_PARSE_KEY_LEN   3

static IR_Profile_t *ir_profiles = NULL;
static int ir_profile_num = 0;
static int ir_profile_parse_num = 0;
struct key_map_list *key_map_lists = NULL;

static int by_pass_kernel_keymap_num = 0;
static int by_pass_kernel_keymap_parse_num = 0;
static by_pass_kernel_keymap_Profile_t *by_pass_kernel_keymap_profiles = NULL;

#if defined(CONFIG_MSTAR_PM)
static IR_PM51_Profile_t *ir_PM51_profiles = NULL;
static int ir_PM51_profile_num = 0;
static int ir_PM51_profile_parse_num = 0;
static unsigned char ir_PM51_cfg_data[32] = {0};
static unsigned char *ir_p16 = &ir_PM51_cfg_data[IR_HEAD16_KEY_PM_CFG_OFFSET];
static unsigned char *ir_p32 = &ir_PM51_cfg_data[IR_HEAD32_KEY_PM_CFG_OFFSET];
#endif

key_value_t IR_KeysList[] = {
#include "input_keys_available.h"
};

static void load_ir_config_thread(struct work_struct *work);
static int load_ir_config_thread_need_stop = 0;
static DECLARE_DELAYED_WORK(load_ir_config_work, load_ir_config_thread);

#ifdef CONFIG_AMAZON_RECOVERY_MODE_IR
static inline int isRecoveryBoot(void)
{
	return strstr(saved_command_line, "recovery_mode=true") != NULL;
}
#endif

/** * @return value * 0 : NormalMode * 1 : RecoveryMode */
static int isRecoveryMode(void)
{
	struct file *filp;
	filp = filp_open("/system/bin/recovery", O_RDONLY, 0644);
	if (IS_ERR(filp))
	{
		printk("Normal Boot!!!\n");
		return 0;
	}
	filp_close(filp, NULL);
	printk("Recovery Boot!!!\n");
	return 1;
}

unsigned int get_ir_keycode(unsigned int scancode)
{
	int i = 0;
	unsigned int ir_header_code = 0;
	unsigned int keycode = KEY_RESERVED;

	if (ir_PM51_profiles == NULL) {
		DBG_ERR("ir_PM51_profiles is NULL\n");
		return keycode;
	}

	DBG_ALL("ir_PM51_profile_parse_num:%d\n", ir_PM51_profile_parse_num);
	//Search IR Header from PM51 profiles first
	for (i = 0; i < ir_PM51_profile_parse_num; i++) {
		if (scancode == ir_PM51_profiles[i].u32PowerKey) {
			ir_header_code = ir_PM51_profiles[i].u32Header;
			DBG_ALL("Match ir_PM51_profiles success, scancode:%u, ir_header_code:%u\n", scancode, ir_header_code);
			break;
		}
	}
	if (i == ir_PM51_profile_parse_num) {
		DBG_ERR("Match ir_PM51_profiles failed\n");
		return keycode;
	}

	//Get keycode from ir_header_code and scancode
	keycode = MIRC_Get_Keycode(ir_header_code, scancode);
	return keycode;
}
EXPORT_SYMBOL(get_ir_keycode);

static void fileread(char *filename, char **data)
{
    struct file *filp;
    mm_segment_t fs;
    loff_t size;
    ssize_t ret;
    filp = filp_open(filename, O_RDONLY, 0644);
    if (IS_ERR(filp))
    {
        DBG_ALL("open %s error...\n", filename);
        return;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    size = filp->f_op->llseek(filp, 0, SEEK_END);
    *data = (char *)kmalloc(size + 1, GFP_KERNEL);
    filp->f_op->llseek(filp, 0, SEEK_SET);
    ret = vfs_read(filp, *data, size, &filp->f_pos);
    if (ret != size)
    {
        DBG_ERR(KERN_WARNING"read %s error!\n", filename);
        kfree(*data);
        *data = NULL;
        goto out;
    }
    (*data)[size] = 0;
out:
    set_fs(fs);
    filp_close(filp, NULL);
}
static int str2ul(const char *str, u32 *val)
{
    if (sscanf(str, "%i", val) == -1)
    {
        return -1;
    }
    return 0;
}

static void parse_all_keymaps(IniSectionNode *root, const char *name, const char *value)
{
    IniKeyNode *k;
    const char *enable;
    const char *protocol;
    const char *header;
    const char *speed;
    const char *keymap_name;
    IniSectionNode *keymap_sec;
    int keymap_size;
    struct key_map_table *table;
    int i;
    IR_Profile_t *curr_ir_profile = &ir_profiles[ir_profile_parse_num];
    IniSectionNode *ir_config = get_section(root, value);
    if(ir_profile_parse_num > 15)return;
    if (ir_config == NULL)
    {
        DBG_ERR("no section named %s\n", value);
        return;
    }
    /**
     * Keymap is Enabled?
     */
    enable = get_key_value(ir_config,"Enable");
    if(enable && (*enable == 'f' || *enable == 'F' || *enable == '0' || *enable == 'n' || *enable == 'N'))
    {
        return;
    }
    curr_ir_profile->u8Enable = true;
    /**
     * Parse Protocol
     */
    protocol = get_key_value(ir_config, "Protocol");
    if (protocol == NULL)
    {
        DBG_ERR("no Protocol for %s\n", ir_config->name);
        return;
    }
    if(str2ul(protocol,&curr_ir_profile->eIRType))
    {
        DBG_ERR("Protocol for %s format error.\n", ir_config->name);
        return;
    }
    DBG_INFO("Protocol value:0x%02x\n",curr_ir_profile->eIRType);
    curr_ir_profile->eIRType--;
    /**
     * Parse Header code
     */
    header = get_key_value(ir_config, "Header");
    if (header == NULL)
    {
        DBG_ERR("no Header for %s\n", ir_config->name);
        return;
    }
    if (str2ul(header, &curr_ir_profile->u32HeadCode))
    {
        DBG_ERR("header code format err for %s\n", ir_config->name);
        return;
    }
    DBG_INFO("Header code:0x%x\n", curr_ir_profile->u32HeadCode);
    /**
     * Parse IR Speed
     */
    speed = get_key_value(ir_config, "Speed");
    if (speed == NULL)
    {
        DBG_ERR("Speed for %s is empty. use default.\n", ir_config->name);
    }
    else
    {
        if (str2ul(speed, &curr_ir_profile->u32IRSpeed))
        {
            DBG_ERR("IR Speed format err for %s\n", ir_config->name);
            return;
        }
        DBG_INFO("Speed:0x%x\n", curr_ir_profile->u32IRSpeed);
    }
    /**
     * Parse Keymap
     */
    keymap_name = get_key_value(ir_config, "Keymap");
    if (keymap_name == NULL)
    {
        DBG_ERR("no Keymap key for %s\n", ir_config->name);
        return;
    }
    keymap_sec = get_section(root, keymap_name);
    if (keymap_sec == NULL)
    {
        DBG_ERR("no Keymap section for %s\n", keymap_name);
        return;
    }
    keymap_size = get_key_num(keymap_sec);
    if (keymap_size == 0)
    {
        DBG_ERR("no keys in section %s\n", keymap_name);
        return;
    }
    table = kmalloc(sizeof(struct key_map_table) * keymap_size, GFP_KERNEL);
    if (table == NULL)
    {
        DBG_ERR(KERN_ERR"OOM!\n");
        return;
    }
    memset(table, 0, sizeof(struct key_map_table) * keymap_size);
    key_map_lists[ir_profile_parse_num].map.scan = table;
    for (k = keymap_sec->keys;k != NULL; k = k->next)
    {
        for ( i = 0; i < SIZE(IR_KeysList); i++ )
        {
            if (!strcmp(k->name, IR_KeysList[i].key))
            {
                table->keycode = IR_KeysList[i].value;

                if (str2ul(k->value, &table->scancode))
                {
                    DBG_ERR("scan code format err for %s\n", keymap_sec->name);
                    return;
                }
                DBG_ALL("KEY name:%s,KEY val:0x%x,scan code:0x%x\n", k->name, table->keycode, table->scancode);
                table++;
                break;
            }
        }
        if(i == SIZE(IR_KeysList))
        {
            if(str2ul(k->name,&table->keycode))
            {
                DBG_ERR("non-standard key format err for %s\n", k->name);
                return;
            }
            if(str2ul(k->value,&table->scancode))
            {
                DBG_ERR("non-standard key's value format err for %s=%s\n", k->name,k->value);
                return;
            }
            DBG_ALL("KEY name:%s,KEY val:0x%x,scan code:0x%x\n", k->name, table->keycode, table->scancode);
            table++;
        }
    }
    key_map_lists[ir_profile_parse_num].map.headcode = curr_ir_profile->u32HeadCode;
    key_map_lists[ir_profile_parse_num].map.size = keymap_size;
    key_map_lists[ir_profile_parse_num].map.name = kstrdup(keymap_name, GFP_KERNEL);
    DBG_INFO("register config:%s with Header code 0x%x\n", ir_config->name, curr_ir_profile->u32HeadCode);
    DBG_INFO("register keymap:%s\n", keymap_name);
    MIRC_IRCustomer_Add(curr_ir_profile);
    MIRC_Map_Register(&key_map_lists[ir_profile_parse_num]);
    ir_profile_parse_num++;
}

extern void MIRC_Map_free(void);
extern void MIRC_Decoder_free(void);
extern void MIRC_Dis_support_ir(void);

static void parse_by_pass_kernel_keymap_config(IniSectionNode *root, const char *name, const char *value)
{
    const char *hw_dec;
    by_pass_kernel_keymap_Profile_t *curr_pass_kernel_keymap_profile = &by_pass_kernel_keymap_profiles[by_pass_kernel_keymap_parse_num];

    IniSectionNode *by_pass_kernel_keymap_config = get_section(root, value);

    if(by_pass_kernel_keymap_parse_num > 1)return;
    if (by_pass_kernel_keymap_config == NULL)
    {
        DBG_ERR("no section named %s\n", value);
        return;
    }

    if(by_pass_kernel_keymap_parse_num > 1)return;
    hw_dec = get_key_value(by_pass_kernel_keymap_config, "Hwdec_enable");

    if (hw_dec == NULL)
    {
        DBG_ERR("no Hwdec_enable for %s\n", by_pass_kernel_keymap_config->name);
        return;
    }

    if(str2ul(hw_dec,&curr_pass_kernel_keymap_profile->eDECType))
    {
        DBG_ERR("Hwdec_enable for %s format error.\n", by_pass_kernel_keymap_config->name);
        return;
    }
    DBG_INFO("Hwdec_enable value:0x%02x\n",curr_pass_kernel_keymap_profile->eDECType);
    if(curr_pass_kernel_keymap_profile->eDECType)
    {
        MIRC_Map_free();
        MIRC_Decoder_free();
        MIRC_Dis_support_ir();
    }
    by_pass_kernel_keymap_parse_num++;
}

#if defined(CONFIG_MSTAR_PM)
#if defined(CONFIG_SUSPEND)

struct hot_key {
    char name[20];
    u8 index;
    int key_cnt;
    unsigned int keycode;
};

#define POWERKEYSIZE 5
extern u32 powerkey[POWERKEYSIZE];
extern int ir_PM51_PWR_profile_parse_num;
#define PIRKEY "PIR_IR"
#define NFXKEYSIZE 5
extern u32 netflixkey[NFXKEYSIZE];
extern int ir_PM51_NFX_profile_parse_num;
#define NETFLIX "PIR_NETFLIX"
#define ADDHOTKEYSIZE 32
extern struct hot_key mtkrc_hotkey[ADDHOTKEYSIZE];
extern int ir_PM51_ADDHOTKEY_profile_parse_num;
#define ADDHOTKEY "PIR_ADDHOTKEY"
#endif
bool hotkeyparser(char * str, char hotkey[])
{
    char *p;
    int i=0;
    p=strstr(str,ADDHOTKEY);
    p+=strlen(ADDHOTKEY);
    if((*p)=='_'){
        p++;
        while((*p)!='_' && (*p) !='\0')
        {
           hotkey[i]=(*p);
           i++;
           p++;
           if((*p)=='\0')
               return false;
        }
    }
    return true;
}
static void parse_PM51_all_keymaps(IniSectionNode *root, const char *name, const char *value)
{
    const char *protocol;
    const char *header;
    const char *power_key;
    IR_PM51_Profile_t *curr_ir_PM51_profile = &ir_PM51_profiles[ir_PM51_profile_parse_num];
    IniSectionNode *ir_PM51_config = get_section(root, value);
    if(ir_PM51_profile_parse_num > (IR_SUPPORT_PM_NUM_MAX - 1))return;
    if (ir_PM51_config == NULL)
    {
        DBG_ERR("no section named %s\n", value);
        return;
    }
    /**
     * Parse Protocol
     */
    protocol = get_key_value(ir_PM51_config, "Protocol");
    if (protocol == NULL)
    {
        DBG_ERR("no Protocol for %s\n", ir_PM51_config->name);
        return;
    }
    if(str2ul(protocol,&curr_ir_PM51_profile->eIRType))
    {
        DBG_ERR("Protocol for %s format error.\n", ir_PM51_config->name);
        return;
    }
    DBG_INFO("Protocol value:0x%02x\n",curr_ir_PM51_profile->eIRType);
    /**
     * Parse Header code
     */
    header = get_key_value(ir_PM51_config, "Header");
    if (header == NULL)
    {
        DBG_ERR("no Header for %s\n", ir_PM51_config->name);
        return;
    }
    if (str2ul(header, &curr_ir_PM51_profile->u32Header))
    {
        DBG_ERR("header code format err for %s\n", ir_PM51_config->name);
        return;
    }
    DBG_INFO("Header code:0x%x\n", curr_ir_PM51_profile->u32Header);
    /**
     * Parse PowerKey
     */
    power_key = get_key_value_by_len(ir_PM51_config, "Key", PM51_PARSE_KEY_LEN);
    if (power_key == NULL)
    {
        DBG_ERR("no PowerKey for %s\n", ir_PM51_config->name);
        return;
    }
    if (str2ul(power_key, &curr_ir_PM51_profile->u32PowerKey))
    {
        DBG_ERR("PowerKey code format err for %s\n", ir_PM51_config->name);
        return;
    }
    DBG_INFO("PowerKey code:0x%x\n", curr_ir_PM51_profile->u32PowerKey);
    if((curr_ir_PM51_profile->u32Header > 0x0000ffff) || (ir_PM51_profile_parse_num == (IR_SUPPORT_PM_NUM_MAX - 1)))
    {
        *ir_p32++ = (u8)(curr_ir_PM51_profile->eIRType & 0xff);
        *ir_p32++ = (u8)((curr_ir_PM51_profile->u32Header >> 24) & 0xff);
        *ir_p32++ = (u8)((curr_ir_PM51_profile->u32Header >> 16) & 0xff);
        *ir_p32++ = (u8)((curr_ir_PM51_profile->u32Header >> 8) & 0xff);
        *ir_p32++ = (u8)(curr_ir_PM51_profile->u32Header & 0xff);
        *ir_p32++ = (u8)((curr_ir_PM51_profile->u32PowerKey >> 8) & 0xff);
        *ir_p32++ = (u8)(curr_ir_PM51_profile->u32PowerKey & 0xff);
    }
    else
    {
        *ir_p16++ = (u8)(curr_ir_PM51_profile->eIRType & 0xff);
        *ir_p16++ = (u8)((curr_ir_PM51_profile->u32Header >> 8) & 0xff);
        *ir_p16++ = (u8)(curr_ir_PM51_profile->u32Header & 0xff);
        *ir_p16++ = (u8)((curr_ir_PM51_profile->u32PowerKey >> 8) & 0xff);
        *ir_p16++ = (u8)(curr_ir_PM51_profile->u32PowerKey & 0xff);
    }
#if defined(CONFIG_SUSPEND)
    if(!strncmp(PIRKEY,value,strlen(PIRKEY)))
    {
        powerkey[ir_PM51_PWR_profile_parse_num]=curr_ir_PM51_profile->u32PowerKey;
        ir_PM51_PWR_profile_parse_num++;
    }
    if(!strncmp(NETFLIX,value,strlen(NETFLIX)))
    {
        netflixkey[ir_PM51_NFX_profile_parse_num]=curr_ir_PM51_profile->u32PowerKey;
        ir_PM51_NFX_profile_parse_num++;
    }
    if(!strncmp(ADDHOTKEY,value,strlen(ADDHOTKEY)))
    {
        if(hotkeyparser((char *)value,mtkrc_hotkey[ir_PM51_ADDHOTKEY_profile_parse_num].name))
        {
            mtkrc_hotkey[ir_PM51_ADDHOTKEY_profile_parse_num].index=ir_PM51_ADDHOTKEY_profile_parse_num;
            mtkrc_hotkey[ir_PM51_ADDHOTKEY_profile_parse_num].keycode=curr_ir_PM51_profile->u32PowerKey;
        }
        ir_PM51_ADDHOTKEY_profile_parse_num++;
    }
#endif
    ir_PM51_profile_parse_num++;
}
#endif

#if defined(CONFIG_DATA_SEPARATION)
static int getDataIndexPath(char* buf, size_t buf_size)
{
    char *str = NULL;
    extern char *saved_command_line;
    char *tmp_command_line = NULL;
    size_t saved_command_line_len = 0;
    const char acPattern[] = "cusdata_dataIndex_path=";
    size_t lenOfDataIndexPath = 0;
    char *pc_dataindex_path = NULL;
    char *pc_dataindex_path_start = NULL;
    char *found = NULL;

    tmp_command_line = (char *)kmalloc(strlen(saved_command_line) + 1, GFP_KERNEL);
    saved_command_line_len = strlen(saved_command_line) + 1;

    memset(tmp_command_line, 0, saved_command_line_len);
    memcpy(tmp_command_line, saved_command_line, saved_command_line_len);

    pc_dataindex_path = strstr(tmp_command_line, acPattern);
    if (pc_dataindex_path == NULL)
    {
        goto err;
    }

    pc_dataindex_path_start = pc_dataindex_path + sizeof(acPattern) - 1;
    found = strchr(pc_dataindex_path_start, ' ');

    if (found == NULL)
    {
        snprintf(buf, buf_size, "%s", pc_dataindex_path_start);
    }
    else
    {
        lenOfDataIndexPath = found - pc_dataindex_path_start;
        if (lenOfDataIndexPath == 0)
        {
            goto err;
        }
        else
        {
            *found = '\0';
            snprintf(buf, buf_size, "%s", pc_dataindex_path_start);
        }
    }
    kfree(tmp_command_line);
    tmp_command_line = NULL;

    return 0;

err:
    if (tmp_command_line != NULL)
    {
        kfree(tmp_command_line);
        tmp_command_line = NULL;
    }
    return 1;
}


static void getDataIndexIniFile(char **data,char **data2)
{
    int res = 0;

    *data2 = NULL;
    //When accessing user memory, we need to make sure the entire area really is in user-level space.
    //KERNEL_DS addr user-level space need less than TASK_SIZE
    mm_segment_t fs=get_fs();
    set_fs(KERNEL_DS);
    *data = (char *)kzalloc(DATA_INDEX_NAME_BUF_SIZE+1, GFP_KERNEL);
    if(!(*data))
    {
        set_fs(fs);
        return;
    }

    memset(*data , 0, DATA_INDEX_NAME_BUF_SIZE);
    res = getDataIndexPath(*data, DATA_INDEX_NAME_BUF_SIZE);
    if (res != 0)
    {
        kfree(*data);
        *data = NULL;
        return;
    }

    set_fs(fs);
    return;
}

static void getIrConfigIniFile(char **data)
{
    char *data_index_buf = NULL;
    char *data_index_name = NULL;
	char *data_index_name2 = NULL;
    int u16BufIdx=0;
    char *str=NULL;
    int u8CntSwitch=0;
    //When accessing user memory, we need to make sure the entire area really is in user-level space.
    //KERNEL_DS addr user-level space need less than TASK_SIZE
    mm_segment_t fs=get_fs();
    set_fs(KERNEL_DS);

    //get dataIndex.ini path
    getDataIndexIniFile(&data_index_name,&data_index_name2);
    //read /vendor/cusdata dataIndex.ini   
    if (data_index_name)    
	{        
	    printk("dataIndex path = %s\n", data_index_name);        
		fileread(data_index_name, &data_index_buf);
		kfree(data_index_name);
	}    
	else    
	{        
	    printk("dataIndex path is null \n");
	}
	
    if (data_index_buf == NULL)
    {
        //read /cusdata dataIndex.ini
        if (data_index_name2)        
		{
		    printk("dataIndex2 path = %s\n", data_index_name2);            
			fileread(data_index_name2, &data_index_buf);            
			kfree(data_index_name2);
        }
		else        
		{            
		    printk("dataIndex2 path is null \n"); 
		}
        if (data_index_buf == NULL)
		{            
		    set_fs(fs);            
			return;        
		}
    }
	else
	{        
        if(data_index_name2)        
		{            
		    kfree(data_index_name2);        
		}    
    }

    //parser dataIndex.ini
    str = strstr(data_index_buf,"m_pIrConfig_File");
    if (str)
    {
        *data = (char*)kmalloc(IR_CONFIG_FILE_NAME_BUF_SIZE+1, GFP_KERNEL);
        memset(*data, 0, IR_CONFIG_FILE_NAME_BUF_SIZE + 1);
        char* pTypeStart = strchr(str, '"');
        if(pTypeStart)
        {
            char* pTypeEnd = strchr(pTypeStart+1, '"');
            if(pTypeEnd)
            {
                *pTypeEnd = '\0';
                if(strlen(pTypeStart) > IR_CONFIG_FILE_NAME_BUF_SIZE)
                {
                    printk("m_pIrConfig_File = %s is too long, ir path used default path.",pTypeStart);
                    memcpy(*data, CONFIG_IR_CONFIG_PATH, strlen(CONFIG_IR_CONFIG_PATH));
                }
                else
                {
                    if(isRecoveryMode()==0)
                    {
                        memcpy(*data, pTypeStart+1, strlen(pTypeStart));
                    }
                    else
                    {
                        snprintf(*data, IR_CONFIG_FILE_NAME_BUF_SIZE, "/mnt%s", pTypeStart+1);
                    }
                }
                printk("getIrConfigIniFile = %s \n", *data);
                set_fs(fs);
                kfree(data_index_buf);
                return;
            }
        }
        kfree(*data);
        *data = NULL;
    }
    set_fs(fs);
    kfree(data_index_buf);
}
#endif

extern int MIRC_support_ir_max_timeout(void);

static void load_ir_config_thread(struct work_struct *work)
{
    while (!load_ir_config_thread_need_stop)
    {
        char *contents = NULL;
        IniSectionNode *root;
        IniSectionNode *kernel_sec;
        IniSectionNode *by_pass_kernel_keymap_sec;//HW decode
#if defined(CONFIG_MSTAR_PM)
        IniSectionNode *PM51_sec;
#endif
#if defined(CONFIG_DATA_SEPARATION)
        getIrConfigIniFile(&config_file);
        printk("ir_config.ini path = %s\n", config_file);
#elif defined(CONFIG_AMAZON_RECOVERY_MODE_IR)
	config_file = isRecoveryBoot() ? ir_config_path_recovery : ir_config_path;
#endif
        if(config_file != NULL)
            fileread(config_file, &contents);
        if (contents == NULL)
            fileread(STRINIT("/mnt",CONFIG_IR_CONFIG_PATH), &contents);
        if (contents == NULL)
        {
            if(purelinux_config_file != NULL)
            {
                fileread(purelinux_config_file, &contents);
            }
            if(contents == NULL)
            {
                msleep(400);
                continue;
            }
        }
        if (alloc_and_init_ini_tree(&root, contents))
        {
            DBG_ERR(KERN_ERR"OOM!\n");
            goto out;
        }
        if (debug > 1)
        {
            dump_ini(root);
        }

        by_pass_kernel_keymap_sec=get_section(root, "Hwdec");

        if (by_pass_kernel_keymap_sec == NULL)
        {
            DBG_ERR("no section named Hwdec\n");
            //goto out;
        }
        by_pass_kernel_keymap_num = get_key_num(by_pass_kernel_keymap_sec);
        if (by_pass_kernel_keymap_num == 0)
        {
            DBG_ERR("no no keys in section Hwdec\n");
            //goto out;
        }
        by_pass_kernel_keymap_profiles = kzalloc(sizeof(by_pass_kernel_keymap_Profile_t) * by_pass_kernel_keymap_num, GFP_KERNEL);
        if (by_pass_kernel_keymap_profiles == NULL)
        {
            DBG_ERR(KERN_ERR"OOM!\n");
        }
        foreach_key(root, by_pass_kernel_keymap_sec, parse_by_pass_kernel_keymap_config);
        if (by_pass_kernel_keymap_profiles)
        {
            kfree(by_pass_kernel_keymap_profiles);
            by_pass_kernel_keymap_profiles = NULL;
        }
        kernel_sec = get_section(root, "Kernel");
        if (kernel_sec == NULL)
        {
            DBG_ERR("no section named Kernel\n");
            goto out;
        }
        ir_profile_num = get_section_num(kernel_sec);
        if (ir_profile_num == 0)
        {
            DBG_ERR("no no keys in section Kernel\n");
            goto out;
        }
        ir_profiles = kzalloc(sizeof(IR_Profile_t) * ir_profile_num, GFP_KERNEL);
        key_map_lists = kzalloc(sizeof(struct key_map_list) * ir_profile_num, GFP_KERNEL);
        if ((key_map_lists == NULL) || (ir_profiles == NULL))
        {
            DBG_ERR(KERN_ERR"OOM!\n");
        }
        foreach_key(root, kernel_sec, parse_all_keymaps);
        MIRC_IRCustomer_Init();
        mstar_ir_reinit();
        MIRC_support_ir_max_timeout();
#if defined(CONFIG_MSTAR_PM)
        PM51_sec = get_section(root, "PM51");
        if (PM51_sec == NULL)
        {
            DBG_ERR("no section named PM51\n");
            goto out;
        }
        ir_PM51_profile_num = get_key_num(PM51_sec);
        if (ir_PM51_profile_num == 0)
        {
            DBG_ERR("no no keys in section PM51\n");
            goto out;
        }
        ir_PM51_profiles = kzalloc(sizeof(IR_PM51_Profile_t) * ir_PM51_profile_num, GFP_KERNEL);
        if (ir_PM51_profiles == NULL)
        {
            DBG_ERR(KERN_ERR"OOM!\n");
        }
        foreach_key(root, PM51_sec, parse_PM51_all_keymaps);
        MDrv_PM_Set_IRCfg(ir_PM51_cfg_data, sizeof(ir_PM51_cfg_data));
#endif
    out:
        release_ini_tree(root);
        kfree(contents);

        return;
    }
}

static int __init ir_dynamic_config_init(void)
{
    schedule_delayed_work(&load_ir_config_work,10);
    return 0;
}

static void __exit ir_dynamic_config_exit(void)
{
    int i;
    load_ir_config_thread_need_stop = 1;
    cancel_delayed_work_sync(&load_ir_config_work);
    for ( i = 0; i < ir_profile_parse_num; i++ )
    {
        if (key_map_lists != NULL)
        {
            if (key_map_lists[i].map.scan != NULL)
            {
                MIRC_Map_UnRegister(&key_map_lists[i]);
                kfree(key_map_lists[i].map.scan);
                key_map_lists[i].map.scan = NULL;
            }
            if (key_map_lists[i].map.name != NULL)
            {
                DBG_INFO("unregister keymap:%s\n", key_map_lists[i].map.name);
                kfree(key_map_lists[i].map.name);
                key_map_lists[i].map.name = NULL;
            }
        }
    }

    if (key_map_lists)
    {
        kfree(key_map_lists);
        key_map_lists = NULL;
    }
    if (ir_profiles)
    {
        for ( i = 0; i < ir_profile_parse_num; i++ )
        {
            DBG_INFO("register config with Header code:%x\n", ir_profiles[i].u32HeadCode);
            MIRC_IRCustomer_Remove(&ir_profiles[i]);
        }
        kfree(ir_profiles);
        ir_profiles = NULL;
    }
}

module_init(ir_dynamic_config_init);
module_exit(ir_dynamic_config_exit);

MODULE_DESCRIPTION("IR dynamic config module for MSTAR");
MODULE_AUTHOR("Vick Sun <vick.sun@mstarsemi.com>");
MODULE_LICENSE("GPL v2");
