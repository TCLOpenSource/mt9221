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

#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
#include <linux/uaccess.h>
#endif

#include "mma_common.h"
#include "mma_core.h"
#include "mma_api.h"
#include "mma_tee_inf.h"
#include "mma_debugfs.h"

extern struct mma_device mma_dev;

/*
1. list all memory info allocated by mma api
echo 1 > debug/mma/record_time   //record alloc time
cat debug/mma/meminfo //list mem info
format:
serial    buf_tag    addr    size    pid    secure  alloc_time(us)

*/

/*
2. open call trace
echo 1> /debug/mma/calltrace_enable
*/

/*
3. dump memory content allocated by mma api
cat /debug/mma/buffer/xxx, xxx 代表要显示calltrace的buffer handle serial。

*/

static unsigned int meminfo_flag = 0;
static ssize_t meminfo_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	struct mma_buf_handle* handle;
	ssize_t count = 0, linemax = 45;
	u64 total = 0;
	int flag = 0;
	char *data = NULL;

	mutex_lock(&mma_dev.buf_lock);

	if(meminfo_flag == 3){
		data = kmalloc(PAGE_SIZE, GFP_KERNEL);
		if(!data)
			goto out;
		count = sprintf(data, "%10.s %16.s %10.s %9.s %5.s %6.s %4.s %6.s %10.s\n",
			  "serial", "buf_tag", "addr", "size", "pid", "secure", "iova", "2XIOVA", "a_time(us)");
		pr_emerg("mma:%s", data);
		list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
			count = sprintf(data, "%10d %16.s %10llx %9x %5d %6d %4d %6d %10d\n",
				handle->serial, handle->buf_tag, handle->addr, (u32)handle->length,
				handle->tpid, handle->auth_count>0?1:0, handle->is_iova,
				(handle->flag & MMA_FLAG_2XIOVA) >> 3, handle->alloc_time);
			total += handle->length;
			pr_emerg("mma:%s", data);
		}
		pr_emerg("mma:total size 0x%llx\n", total);
		kfree(data);
		count = 0;
	}else if(meminfo_flag == 2){
		count += sprintf(buf, "%10.s %16.s %10.s %9.s %5.s %6.s %4.s %6.s %10.s\n",
			  "serial", "buf_tag", "addr", "size", "pid", "secure", "iova", "2XIOVA", "a_time(us)");
		linemax += count;
		list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
			if(count > PAGE_SIZE - linemax){
				flag = 1;
				goto out;
			}
			count += sprintf(buf + count, "%10d %16.s %10llx %9x %5d %6d %4d %6d %10d\n",
				handle->serial, handle->buf_tag, handle->addr, (u32)handle->length,
				handle->tpid, handle->auth_count>0?1:0, handle->is_iova,
				(handle->flag & MMA_FLAG_2XIOVA) >> 3, handle->alloc_time);
			total += handle->length;
		}

	}else if(meminfo_flag == 1){
		count += sprintf(buf, "%16.s %10.s %9.s %5.s %6.s\n",
			  "buf_tag", "addr", "size", "pid", "secure");
		linemax += count;
		list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
			if(count > PAGE_SIZE - linemax){
				flag = 1;
				goto out;
			}
			count += sprintf(buf + count, "%16.s %10llx %9x %5d %6d\n",
				handle->buf_tag, handle->addr, (u32)handle->length,
				handle->tpid, handle->auth_count>0?1:0);
			total += handle->length;
		}

	}else{
		count += sprintf(buf, "%16.s %10.s %9.s\n",
			  "buf_tag", "addr", "size");
		linemax += count;
		list_for_each_entry(handle, &mma_dev.buf_list_head, buf_list_node) {
			if(count > PAGE_SIZE - linemax){
				flag = 1;
				goto out;
			}
			count += sprintf(buf + count, "%16.s %10llx %9x\n",
				handle->buf_tag, handle->addr, (u32)handle->length);
			total += handle->length;
		}

	}
out:
	count += sprintf(buf + count, "Total size:0x%llx\n", total);
	mutex_unlock(&mma_dev.buf_lock);
	if(flag == 1)
		count += sprintf(buf + count, "more buffer not show\n");
	return count;
}


static ssize_t meminfo_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int enable = 0;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret)
		return ret;

	meminfo_flag = enable;

	printk("meminfo_flag = %u\n", enable);
	return count;
}

static ssize_t interrupt_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int enable = 0;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret)
		return ret;

	if (enable > 255)
		return -EINVAL;

	if(enable == 1) {
		ret = _MDrv_IOMMU_RegisterInterrupt();
		if(!ret)
			printk("enable interrupt\n");
		else
			printk("enable interrupt fail\n");
	} else {
		_MDrv_IOMMU_DeRegisterInterrupt();
		printk("disable interrupt\n");
	}
	printk("interrupt = %u\n", enable);
	return count;
}

static ssize_t calltrace_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int enable = 0;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret)
		return ret;

	if (enable > 255)
		return -EINVAL;

	mma_dev.calltrace_enable = enable;
	printk("calltrace = %u\n", enable);
	return count;
}

static ssize_t recordtime_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int val;
	int err;

	err = kstrtouint(buf, 10, &val);
	if (err)
		return err;

	if (val > 255)
		return -EINVAL;

	mma_dev.record_alloc_time = val;
	printk("recordtime = %u\n", val);
	return count;
}

static ssize_t lock_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int err;
	tee_map_lock lock;
	int ret;
	char *tmp;
	char *start, *end;
	size_t size, i;

	tmp = vmalloc(count);
	if (!tmp)
		return count;
	strncpy(tmp, buf, count);
	tmp[count - 1] = '\0';

	memset((void *)&lock, 0, sizeof(lock));

	start = tmp;
	end = strchr(start, ',');
	if (end) {
		size = (size_t)(end - start);
		if (size > (MAX_NAME_SIZE - 1)) {
			pr_err("buf tag error\n");
			goto out;
		}
		strncpy(lock.buffer_tag, tmp, size);
		lock.buffer_tag[size] = '\0';
	} else {
		pr_err("input buf tag error\n");
		goto out;
	}

	start = end + 1;
	end = strchr(start, ',');
	if (end)
		*end = '\0';
	else
		goto out;
	ret = kstrtou32(start, 10, &lock.aid_num);
	if (ret) {
		pr_err("kstrtoul aid_num error,%s\n", start);
		goto out;
	}
	if (lock.aid_num > 8) {
		pr_err("aid_num =%d error\n", lock.aid_num);
		goto out;
	}
	if (lock.aid_num > 0) {
		for (i = 0; i < lock.aid_num; i++) {
			start = end + 1;
			end = strchr(start, ',');
			if (end) {
				*end = '\0';
				ret = kstrtou32(start, 10, &lock.aid[i]);
				if (ret) {
					pr_err("kstrtoul aid error\n");
					goto out;
				}
			} else {
				pr_err("aid error\n");
				goto out;
			}
		}
	}
	ret = mma_tee_lockdebug(&lock);
	if (ret)
		pr_err("lockdebug fail ret =0x%x\n", ret);
out:
	vfree(tmp);
	return count;
}

int mma_creat_buffer_file(struct mma_buf_handle* handle, struct dentry* parent)
{
    return 0;
}

static struct kobj_attribute meminfo_attr =
	__ATTR_RW(meminfo);

static struct kobj_attribute interrupt_attr =
	__ATTR_WO(interrupt);

static struct kobj_attribute lock_attr =
	__ATTR_WO(lock);

static struct kobj_attribute calltrace_attr =
	__ATTR_WO(calltrace);

static struct kobj_attribute recordtime_attr =
	__ATTR_WO(recordtime);

static struct attribute *mma_device_attrs[] = {
	&interrupt_attr.attr,
	&meminfo_attr.attr,
	&lock_attr.attr,
	&calltrace_attr.attr,
	&recordtime_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(mma_device);

int mma_debugfs_init(struct mma_device* mma_dev)
{
	struct kobject *mma_kobj;
	int ret;

	mma_kobj = kobject_create_and_add("mma", kernel_kobj);
	if (!mma_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(mma_kobj, mma_device_groups);
	if (ret) {
		kobject_put(mma_kobj);
		return ret;
	}
	mma_dev->mma_kobj = mma_kobj;
	return 0;

}

int mma_debugfs_destroy(struct mma_device* mma_dev)
{
    if (mma_dev->mma_kobj) {
        sysfs_remove_groups(mma_dev->mma_kobj, mma_device_groups);
		kobject_put(mma_dev->mma_kobj);
    }
    return 0;
}
