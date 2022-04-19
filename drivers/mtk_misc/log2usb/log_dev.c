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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>


#define LOG_DEV_MAJOR        241
#define LOG_DEV_MINOR        32
#define LOG_DEV_NAME         "log2usb"
#define LOG_DEV_BUFF_SIZE    0x40000

/*
 * If this mcro be defined, writing log device will use driver write api
 * log_dev_write_by_driver. writing function of device in this module only is
 * for test. Normally, log_dev_write_by_driver should be associated with of
 * uart driver.
 */
#define LOG_FROM_DRIVER


struct log_device {
    struct kfifo buff;
    unsigned char *buff_addr;
    unsigned int buff_size;

    unsigned int major;
    unsigned int minor;
    char *name;

    bool inited;
    bool opened;
};

struct log_device log_device = {
    .buff_size = LOG_DEV_BUFF_SIZE,
    .major = LOG_DEV_MAJOR,
    .minor = LOG_DEV_MINOR,
    .name = LOG_DEV_NAME,
    .inited = false,
    .opened = false,
};

#ifdef LOG_FROM_DRIVER
/**
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these w/r api.
 */
ssize_t log_dev_write_by_driver(char in[], unsigned int cnt)
{
    struct log_device *log_dev = &log_device;
    if (!log_dev->opened)
        return -EINVAL;

    return kfifo_in(&log_dev->buff, in, cnt);
}
EXPORT_SYMBOL(log_dev_write_by_driver);

/**
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these w/r api.
 */
static ssize_t log_dev_write(struct file *filp, const char __user *from,
                 size_t cnt, loff_t *f_pos)
{
    /* anyhow, return cnt value from param */
    return cnt;
}

#else

/**
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these w/r api.
 */
static ssize_t log_dev_write(struct file *filp, const char __user *from,
                 size_t cnt, loff_t *f_pos)
{
    struct log_device *log_dev = filp->private_data;
    unsigned int from_cnt;
    ssize_t ret;

    if (!log_dev || !log_dev->inited)
        return -EINVAL;

    ret = kfifo_from_user(&log_dev->buff, from, (unsigned int)cnt,
                  &from_cnt);
    if (ret != 0) {
        printk(KERN_ERR "error: log_dev: copy data from user failed\n");
    }
    else {
        *f_pos += from_cnt;
        ret = from_cnt;
    }

    return ret;
}
#endif

/**
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these w/r api.
 */
static ssize_t log_dev_read(struct file *filp, char __user *to, size_t cnt,
            loff_t *f_pos)
{
    struct log_device *log_dev = filp->private_data;
    unsigned int to_cnt;
    ssize_t ret;

    if (!log_dev || !log_dev->inited)
        return -EINVAL;

    ret = kfifo_to_user(&log_dev->buff, to, (unsigned int)cnt, &to_cnt);
    if (ret != 0) {
        printk(KERN_ERR "error: log_dev: copy data to user failed\n");
    } else {
        *f_pos += to_cnt;
        ret = to_cnt;
    }

    return ret;
}

static int log_dev_open(struct inode *inode, struct file *filp)
{
    struct log_device *log_dev = &log_device;
    printk(KERN_DEBUG "log_dev: open\n");

    filp->private_data = &log_device;
    log_dev->opened = true;
    return 0;
}

static int log_dev_release(struct inode *inode, struct file *filp)
{
    struct log_device *log_dev = filp->private_data;
    printk(KERN_DEBUG "log_dev: release\n");

    if (!log_dev || !log_dev->inited)
        return -EINVAL;
    log_dev->opened = false;
    return 0;
}

static struct file_operations log_dev_fops = {
    .owner = THIS_MODULE,
    .open = log_dev_open,
    .release = log_dev_release,
    .read = log_dev_read,
    .write = log_dev_write,
};

static struct cdev log_dev_cdev;
static int log_dev_init(void)
{
    struct log_device *log_dev = &log_device;
    struct cdev *cdev = &log_dev_cdev;
    dev_t devno;
    int ret;
    printk(KERN_DEBUG "log_dev: init start\n");

    log_dev->buff_size = roundup_pow_of_two(log_dev->buff_size);
    log_dev->buff_addr = (unsigned char *)kmalloc(log_dev->buff_size,
                     GFP_KERNEL);
    if (log_dev->buff_addr == NULL) {
        printk(KERN_ERR "error: log_dev: malloc failed\n");
        ret = -ENOMEM;
        goto return_fail;
    }

    ret = kfifo_init(&log_dev->buff, log_dev->buff_addr,
                 log_dev->buff_size);
    if (ret)
        goto free_mem;

    devno= MKDEV(log_dev->major, log_dev->minor);
    ret = register_chrdev_region(devno, 1, log_dev->name);
    if (ret) {
        printk(KERN_ERR "error: log_dev: register chrdev failed\n");
        goto free_mem;
    }

    cdev->owner = THIS_MODULE;
    cdev_init(cdev, &log_dev_fops);
    ret = cdev_add(cdev, devno, 1);
    if (ret) {
        printk(KERN_ERR "error: log_dev: cdev_add failed\n");
        goto unregister_chrdev;
    }

    log_dev->inited = true;
    printk(KERN_DEBUG "log_dev: init done\n");
    return 0;

unregister_chrdev:
    unregister_chrdev_region(devno, 1);
free_mem:
    kfree(log_dev->buff_addr);
    log_dev->buff_addr = NULL;
return_fail:
    return ret;
}

static void log_dev_destory(void)
{
    struct log_device *log_dev = &log_device;

    if (!log_dev->inited)
        return;

    kfree(log_dev->buff_addr);
    cdev_del(&log_dev_cdev);
    unregister_chrdev_region(MKDEV(log_dev->major, log_dev->minor), 1);
    log_dev->inited = false;
}

static int __init log_dev_mod_init(void)
{
    printk(KERN_DEBUG "log_dev: hello world\n");
    return log_dev_init();
}

static void __exit log_dev_mod_exit(void)
{
    printk(KERN_DEBUG "log_dev: bye world\n");
    log_dev_destory();
}

module_init(log_dev_mod_init);
module_exit(log_dev_mod_exit);
MODULE_DESCRIPTION("mtk log device test");
