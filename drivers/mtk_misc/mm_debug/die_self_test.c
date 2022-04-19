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
#ifdef CONFIG_SYSFS 

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/slub_def.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/kasan.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/kdebug.h>
#include <linux/workqueue.h> //system_highpri_wq
#include <linux/rmap.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <asm/sections.h>
#include <linux/memblock.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,15,0)
#include <linux/sched/signal.h>
#endif

static ssize_t selftest_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count);
static struct kobj_attribute selftest_attr=__ATTR_WO(selftest);
static struct kobject *die_st_kobj=NULL;

typedef void (*pfn_cb)(void);
struct die_st_info
{
    char *ptr_slub;
    char *ptr_vmalloc;
    pfn_cb cb;
};
static ssize_t selftest_store(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t count)
{
    struct die_st_info *die_info=NULL;
    unsigned long opt;
    
    if(kstrtoul(buf,0,&opt))
       return -EINVAL;
    do
    {
        if(opt==0)die_info=(struct die_st_info *)vmalloc(sizeof(struct die_st_info));
        else if(opt==1)die_info=(struct die_st_info *)kmalloc(sizeof(struct die_st_info),GFP_KERNEL);
        else if(opt==2)die_info=(struct die_st_info *)alloc_pages(GFP_HIGHUSER_MOVABLE,0);
        if(!die_info)return count;
        else memset(die_info,0x5a,sizeof(struct die_st_info));
    }while(0);
    if(opt==0)*die_info->ptr_vmalloc=0x2b;
    else if(opt==1)
    {
        pr_info("die_info in slub=%px\n",die_info);
        *die_info->ptr_slub=0x3b;
    }
    else if(opt==2)die_info->cb();
    pr_info("selftest done\n");
    return count;    
}
static const struct attribute *die_st_attrs[] = {
    &selftest_attr.attr,
    NULL,
};
static __init int die_selftest_init(void)
{
    int error=0;
    die_st_kobj=kobject_create_and_add("die_selftest", kernel_kobj);
    if(!die_st_kobj)
    {
        return -ENOMEM;
    }
    error = sysfs_create_files(die_st_kobj, die_st_attrs);
    if (error)
    {
        kobject_put(die_st_kobj);
    }
    return error;
}
static void __exit die_selftest_exit(void)
{
    
}

module_init(die_selftest_init);
module_exit(die_selftest_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mediatek");
#endif