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
/// file    iniparser.c
/// @brief  Parse Ini config file
/// @author Vick.Sun@MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////



#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/slab.h>

#include "iniparser.h"

#define MALLOC _malloc
#define FREE kfree
#define PRINT printk

static struct kmem_cache *_malloc(size_t size)
{
    return kmalloc(size,GFP_KERNEL);
}

static const char* strtrim(char* str)
{
    char* p = str + strlen(str) - 1;
    while(p != str && (isspace(*p) || *p == '\"'))
    {
        *p = '\0';
        p--;
    }

    p = str;
    while(*p != '\0' && (isspace(*p) || *p == '\"'))
    {
        p++;
    }

    if (p != str)
    {
        char* d = str;
        char* s = p;
        while(*s != '\0')
        {
            *d = *s;
            s++;
            d++;
        }
        *d = '\0';
    }

    return str;
}

int alloc_and_init_ini_tree(IniSectionNode **root,char*p)
{
    enum _State
    {
        STAT_NONE = 0,
        STAT_GROUP,
        STAT_KEY,
        STAT_VAL,
        STAT_COMMENT
    }state = STAT_NONE;
    char *group_start = p;
    char *key_start = p;
    char *val_start = p;
    IniSectionNode *current_section = NULL;
    IniKeyNode *current_key = NULL;
    while (*p != '\0')
    {
        if (*p == '#' || *p == ';')
        {
            /*skip comment*/
            while (*p != '\0' &&* p != '\n')
            {
                *p = 0;
                p++;
            }
            if (*p == '\0')
            {
                break;
            }
        }
        switch(state)
        {
            case STAT_NONE:
            {
                if (*p == '[')
                {
                    state = STAT_GROUP;
                    group_start = p + 1;
                    if (current_section == NULL)
                    {
                        current_section = (IniSectionNode *)MALLOC(sizeof(IniSectionNode));
                        if (!current_section)
                        {
                            PRINT("return on OOM!\n");
                            return -1;
                        }
                        memset(current_section, 0, sizeof(IniSectionNode));
                        *root = current_section;
                    }
                    else
                    {
                        current_section->next = (IniSectionNode*)MALLOC(sizeof(IniSectionNode));
                        if (!current_section->next)
                        {
                            PRINT("return on OOM!\n");
                            return -1;
                        }
                        memset(current_section->next, 0, sizeof(IniSectionNode));
                        current_section = current_section->next;
                    }
                }
                else if (!isspace(*p))
                {
                    key_start = p;
                    if (current_key == NULL)
                    {
                        current_key = (IniKeyNode*)MALLOC(sizeof(IniKeyNode));
                        if (!current_key)
                        {
                            PRINT("return on OOM!\n");
                            return -1;
                        }
                        memset(current_key, 0, sizeof(IniKeyNode));
                        if (current_section == NULL)
                        {
                            current_section = (IniSectionNode *)MALLOC(sizeof(IniSectionNode));
                            if (!current_section)
                            {
                                PRINT("return on OOM!\n");
                                return -1;
                            }
                            memset(current_section, 0, sizeof(IniSectionNode));
                            *root = current_section;
                        }
                        current_section->keys = current_key;
                    }
                    else
                    {
                        current_key->next = (IniKeyNode*)MALLOC(sizeof(IniKeyNode));
                        if (!current_key->next)
                        {
                            PRINT("return on OOM!\n");
                            return -1;
                        }
                        memset(current_key->next, 0, sizeof(IniKeyNode));
                        current_key = current_key->next;
                    }
                    state = STAT_KEY;
                }
                break;
            }
            case STAT_GROUP:
            {
                if (*p == ']')
                {
                    *p = '\0';
                    current_section->name = strtrim(group_start);
                    current_key = NULL;
                    state = STAT_NONE;
                }
                break;
            }
            case STAT_KEY:
            {
                if (*p == '=')
                {
                    *p = '\0';
                    val_start = p + 1;
                    current_key->name = strtrim(key_start);
                    state = STAT_VAL;
                }
                break;
            }
            case STAT_VAL:
            {
                if (*p == '\n' || *p == '#' || *p == ';')
                {
                    if (*p != '\n')
                    {
                        /*skip comment*/
                        while (*p != '\0' && *p != '\n')
                        {
                            *p = '\0';
                            p++;
                        }
                    }
                    *p = '\0';
                    current_key->value = strtrim(val_start);
                    state = STAT_NONE;

                    break;
                }
            }
            default:
                break;
        }
        p++;
    }
    if (STAT_VAL == state)
    {
        current_key->value = strtrim(val_start);
    }
    return 0;
}
EXPORT_SYMBOL_GPL(alloc_and_init_ini_tree);
void release_ini_tree(IniSectionNode *root)
{
    IniSectionNode *current_section = root;
    IniKeyNode *current_key;
    IniSectionNode *last_section = NULL;
    IniKeyNode *last_key = NULL;
    while (current_section != NULL)
    {
        current_key = current_section->keys;
        while (current_key != NULL)
        {
            last_key = current_key;
            current_key = current_key->next;
            FREE(last_key);
            last_key = NULL;
        }
        last_section = current_section;
        current_section = current_section->next;
        FREE(last_section);
        last_section = NULL;
    }
}
EXPORT_SYMBOL_GPL(release_ini_tree);
IniSectionNode* get_section(IniSectionNode *root, const char *name)
{
    IniSectionNode *current_section;
    current_section = root;
    while (current_section != NULL)
    {
        if (!strcmp(name, current_section->name))
        {
            return current_section;
        }
        current_section = current_section->next;
    }
    return NULL;
}
EXPORT_SYMBOL_GPL(get_section);
size_t get_section_num(IniSectionNode *root)
{
    size_t i = 0;
    IniSectionNode *current_section = root;
    while (current_section != NULL)
    {
        i++;
        current_section = current_section->next;
    }

    return i;
}
EXPORT_SYMBOL_GPL(get_section_num);
void foreach_section(IniSectionNode *root, void (*fun)(IniSectionNode *, IniSectionNode *, const char *))
{
    IniSectionNode *current_section;
    current_section = root;
    while (current_section != NULL)
    {
        fun(root, current_section, current_section->name);
        current_section = current_section->next;
    }
}
EXPORT_SYMBOL_GPL(foreach_section);
void foreach_key(IniSectionNode *root, IniSectionNode *node, void (*fun)(IniSectionNode *, const char *, const char *))
{
    IniKeyNode *current_key;
    if (node && node->keys)
    {
        current_key = node->keys;
        while (current_key != NULL)
        {
            fun(root,current_key->name, current_key->value);
            current_key = current_key->next;
        }
    }
}
EXPORT_SYMBOL_GPL(foreach_key);
const char* get_key_value(IniSectionNode *node, const char *name)
{
    IniKeyNode *current_key;
    if (node && node->keys)
    {
        current_key = node->keys;
        while (current_key != NULL)
        {
            if (!strcmp(current_key->name,name))
            {
                return current_key->value;
            }
            current_key = current_key->next;
        }
    }
    return NULL;
}
EXPORT_SYMBOL_GPL(get_key_value);
const char* get_key_value_by_len(IniSectionNode *node, const char *name, char len)
{
    IniKeyNode *current_key;
    if (node && node->keys)
    {
        current_key = node->keys;
        while (current_key != NULL)
        {
            if (!strncmp(current_key->name,name, len))
            {
                return current_key->value;
            }
            current_key = current_key->next;
        }
    }
    return NULL;
}
EXPORT_SYMBOL_GPL(get_key_value_by_len);
size_t get_key_num(IniSectionNode *node)
{
    size_t i = 0;
    IniKeyNode *current_key;
    if (node && node->keys)
    {
        current_key = node->keys;
        while (current_key != NULL)
        {
            i++;
            current_key = current_key->next;
        }
    }
    return i;
}
EXPORT_SYMBOL_GPL(get_key_num);
static void print_key_value(IniSectionNode *root,const char *name, const char *value)
{
    PRINT("%s=%s\n", name, value);
}
static void print_section(IniSectionNode *root, IniSectionNode *node,const char *name)
{
    PRINT("[%s]\n", name);
    foreach_key(root, node, print_key_value);
}

void dump_ini(IniSectionNode *root)
{
    foreach_section(root, print_section);
}
EXPORT_SYMBOL_GPL(dump_ini);
