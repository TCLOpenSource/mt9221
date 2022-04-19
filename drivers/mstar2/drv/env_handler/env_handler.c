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
#include <linux/time.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <../block/partitions/check.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/page-flags.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <generated/asm-offsets.h>
#include "env_handler.h"

#ifdef CONFIG_COMPAT

#include <linux/compat.h>

typedef struct {

     compat_int_t                 block_id;
     compat_int_t               offset;
} compat_read_write_pos;

typedef struct {

     compat_read_write_pos                 pos;
     compat_uptr_t               data;
} compat_read_write_struct;

#define COMPAT_ENV_HANDLER_GET_DEVICE_INFO _IOR(ENV_HANDLER_MAGIC,1,device_info)
#define COMPAT_ENV_HANDLER_READ _IOR(ENV_HANDLER_MAGIC,2,compat_read_write_struct)
#define COMPAT_ENV_HANDLER_WRITE _IOW(ENV_HANDLER_MAGIC,3,compat_read_write_struct)
#define COMPAT_ENV_HANDLER_ERASE _IOW(ENV_HANDLER_MAGIC,4,int)

#endif

#define MOD_ENV_HANDLER_DEVICE_COUNT     2
#define MOD_ENV_HANDLER_NAME            "env_handler"
#define MOD_ENV_HANDLER_NAME_1             "env_handler_1"
#define MOD_ENV_HANDLER_NAME_2             "env_handler_2"

#define MOD_ENV_HANDLER_PATH 		"/dev/block/platform/mstar_mci.0/by-name/%s"
#define MOD_ENV_HANDLER_SIZE 		(100)

typedef struct
{
    int                         env_handler_major;
    int                         env_handler_minor;
    struct cdev                 env_handler_device;
    struct file_operations      env_handler_file_operations;
    device_info          info;
} env_handler_mod;

static long env_handler_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int env_handler_open (struct inode *inode, struct file *filp);

#ifdef CONFIG_COMPAT
static long compat_env_handler_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int compat_env_handler_read_allocation_data(compat_read_write_struct __user *data32, read_write_struct  __user *data);
#endif


static struct class *env_handler_class;

static env_handler_mod env_handler_dev=
{
    .env_handler_major=               199,
    .env_handler_minor=               1,
    .env_handler_device=
    {
        .kobj=                  {.name= MOD_ENV_HANDLER_NAME, },
        .owner  =               THIS_MODULE,
    },
    .env_handler_file_operations=
    {
    	 .open             =     env_handler_open,
        .unlocked_ioctl=          env_handler_ioctl,
        #ifdef CONFIG_COMPAT
        .compat_ioctl =         compat_env_handler_ioctl,
        #endif
    },
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
static void partion_end_bio(struct bio *bio)
#else
static void partion_end_bio(struct bio *bio, int err)
#endif
{
	struct page *page = bio->bi_io_vec[0].bv_page;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
	unsigned long long bi_sector = (unsigned long long)bio->bi_iter.bi_sector;
#else
	unsigned long long bi_sector = (unsigned long long)bio->bi_sector;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
	if (bio->bi_status) {
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
	if (bio->bi_error) {
#else
	if (!test_bit(BIO_UPTODATE, &bio->bi_flags)) {
#endif
		SetPageError(page);
		ClearPageUptodate(page);
		printk(KERN_ALERT "Read-error on swap-device (%u:%u:%Lu)\n",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
				MAJOR(bio_dev(bio)), MINOR(bio_dev(bio)),
#else
				imajor(bio->bi_bdev->bd_inode),
				iminor(bio->bi_bdev->bd_inode),
#endif
				bi_sector);
	} else {
		SetPageUptodate(page);
	}
	unlock_page(page);
	bio_put(bio);
}

static int submit_page(struct block_device* bd, int rw, pgoff_t page_off,
                                            struct page *page)
{
      struct bio *bio;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
	bio = bio_alloc(__GFP_DIRECT_RECLAIM | __GFP_HIGH, 1);
#else
	bio = bio_alloc(__GFP_WAIT | __GFP_HIGH, 1);
#endif
	if (!bio)
		return -ENOMEM;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,40)
	bio->bi_iter.bi_sector = page_off * (PAGE_SIZE >> 9);
#else
	bio->bi_sector = page_off * (PAGE_SIZE >> 9);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
	bio_set_dev(bio, bd);
#else
	bio->bi_bdev = bd;
#endif
	bio->bi_end_io = partion_end_bio;

	if (bio_add_page(bio, page, PAGE_SIZE, 0) < PAGE_SIZE) {
		printk(KERN_ERR "ENV_HANDLER: Adding page to bio failed at %ld\n",page_off);
		bio_put(bio);
		return -EFAULT;
	}

	lock_page(page);
	bio_get(bio);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
	bio_set_op_attrs(bio, rw, REQ_SYNC);
	submit_bio(bio);
#else
	submit_bio(rw | REQ_SYNC, bio);
#endif
	wait_on_page_locked(page);
	if (rw == READ)
		bio_set_pages_dirty(bio);
	bio_put(bio);
	return 0;
}

unsigned char *read_page(struct block_device *bdev, sector_t n, Sector *p)
{
      struct address_space *mapping = bdev->bd_inode->i_mapping;
      struct page *page;
      page = read_mapping_page(mapping, (pgoff_t)n, NULL);
      p->v = page;
      return (unsigned char *)page_address(page) ;
}


#ifdef CONFIG_COMPAT

static int compat_env_handler_read_allocation_data(
			compat_read_write_struct __user *data32,
			read_write_struct  __user *data)
{
      int err;
      compat_int_t  s32;
      compat_uptr_t compat_ptr;
	err = get_user(s32, &data32->pos.block_id);
	err |= put_user(s32, &data->pos.block_id);
	err |= get_user(s32, &data32->pos.offset);
	err |= put_user(s32, &data->pos.offset);
	err |= get_user(compat_ptr, &data32->data);
	err |= put_user(compat_ptr, &data->data);

	return err;
}

static long compat_env_handler_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err=0;
    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

      switch (cmd)
	{
	    case COMPAT_ENV_HANDLER_GET_DEVICE_INFO:
	    {
			      return filp->f_op->unlocked_ioctl(filp, ENV_HANDLER_GET_DEVICE_INFO, (unsigned long)compat_ptr(arg));
	    }
	    case COMPAT_ENV_HANDLER_READ:
	    {
                         compat_read_write_struct __user *data32;
		            read_write_struct __user *data;
		            data32 = compat_ptr(arg);
		            data = compat_alloc_user_space(sizeof(*data));

		            if (data == NULL)
		                return -EFAULT;

		            err = compat_env_handler_read_allocation_data(data32, data);
		            if (err)
		                return err;
		    		return  filp->f_op->unlocked_ioctl(filp, ENV_HANDLER_READ,  (unsigned long)data);
	      }
		case COMPAT_ENV_HANDLER_WRITE:
	      {

                         compat_read_write_struct __user *data32;
		            read_write_struct __user *data;
		            data32 = compat_ptr(arg);
		            data = compat_alloc_user_space(sizeof(*data));

		            if (data == NULL)
		                return -EFAULT;

		            err = compat_env_handler_read_allocation_data(data32, data);
		            if (err)
		                return err;
		    		return  filp->f_op->unlocked_ioctl(filp, ENV_HANDLER_WRITE,  (unsigned long)data);
	      }
		case COMPAT_ENV_HANDLER_ERASE:
		{
				return filp->f_op->unlocked_ioctl(filp, ENV_HANDLER_ERASE, (unsigned long)compat_ptr(arg));
		}
	}
	  return 	 -ENOSYS;
}

#endif

static unsigned short env_handler_logical_block_size(struct request_queue *q)
{
	int retval = 512;

	if (q && q->limits.logical_block_size)
		retval = q->limits.logical_block_size;

	return retval;
}

static int env_handler_open (struct inode *inode, struct file *filp)
{
	 struct block_device *bd;
	 char device_name[MOD_ENV_HANDLER_SIZE];

	 memset(&device_name, '\0', MOD_ENV_HANDLER_SIZE);
	 if(MINOR(inode->i_rdev)==1)
	 {
		snprintf(device_name, (MOD_ENV_HANDLER_SIZE - 1), MOD_ENV_HANDLER_PATH, CONFIG_MSTAR_ENV_HANDLER_1);
	 }
	 else if(MINOR(inode->i_rdev)==2)
	 {
		snprintf(device_name, (MOD_ENV_HANDLER_SIZE - 1), MOD_ENV_HANDLER_PATH, CONFIG_MSTAR_ENV_HANDLER_2);
	 }

	 bd = blkdev_get_by_path(device_name, FMODE_READ|FMODE_WRITE, NULL);

       if(PAGE_SZ != bd->bd_block_size)
	  {
	         set_blocksize(bd,PAGE_SZ);
        }

	  env_handler_dev.info.rw_block_size=bd->bd_block_size;
	  env_handler_dev.info.erase_block_size=bd->bd_block_size;
	  env_handler_dev.info.erase_block_count=i_size_read(bd->bd_inode)/env_handler_dev.info.erase_block_size;
	  filp->private_data = bd;

	 return 0;
}

static long env_handler_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    Sector sector;
    char *data;
    read_write_struct m_read_write_struct;
    struct block_device *bd = filp->private_data;

    switch(cmd)
    {
        case ENV_HANDLER_GET_DEVICE_INFO:
	 {
            if (copy_to_user(( struct device_info *) arg, &env_handler_dev.info, sizeof(env_handler_dev.info)))
			 return -EFAULT;
            break;
	 }
        case ENV_HANDLER_READ:
	 {
          if (copy_from_user (&m_read_write_struct, (struct read_write_struct *) arg,sizeof(m_read_write_struct)))
			return -EFAULT;

	     data=read_page(bd, m_read_write_struct.pos.block_id, &sector);

           if (copy_to_user((char *)m_read_write_struct.data, data,env_handler_dev.info.rw_block_size))
	              return -EFAULT;

	      put_dev_sector(sector);
            break;
	 }
        case ENV_HANDLER_WRITE:
	 {
            if (copy_from_user (&m_read_write_struct, (struct read_write_struct *) arg,sizeof(m_read_write_struct)))
			return -EFAULT;

	     data= read_page(bd, m_read_write_struct.pos.block_id, &sector);

         if (m_read_write_struct.data)
         {
	        if (copy_from_user (data, (char *) m_read_write_struct.data, env_handler_dev.info.rw_block_size))
		    return -EFAULT;
         }

	     submit_page(bd,1,m_read_write_struct.pos.block_id,sector.v);
	     put_dev_sector(sector);
            break;
	 }
        case ENV_HANDLER_ERASE:
            return 0;

        default:
            printk("\nUnknown ioctl command %x\n", cmd);

        return -ENOTTY;
    }

    return 0;
}

 int __init mod_env_handler_init(void)
{
    int s32Ret;
    dev_t dev;

    env_handler_class = class_create(THIS_MODULE, "env_handler");
    if (IS_ERR(env_handler_class))
    {
        return PTR_ERR(env_handler_class);
    }

    if (env_handler_dev.env_handler_major)
    {
        dev = MKDEV(env_handler_dev.env_handler_major, env_handler_dev.env_handler_minor);
        s32Ret = register_chrdev_region(dev, MOD_ENV_HANDLER_DEVICE_COUNT, MOD_ENV_HANDLER_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, env_handler_dev.env_handler_minor, MOD_ENV_HANDLER_DEVICE_COUNT, MOD_ENV_HANDLER_NAME);
        env_handler_dev.env_handler_major = MAJOR(dev);
    }

    if ( 0 > s32Ret)
    {
        class_destroy(env_handler_class);
        return s32Ret;
    }

    cdev_init(&env_handler_dev.env_handler_device, &env_handler_dev.env_handler_file_operations);

    if (0!= (s32Ret= cdev_add(&env_handler_dev.env_handler_device, dev, MOD_ENV_HANDLER_DEVICE_COUNT)))
    {
        unregister_chrdev_region(dev, MOD_ENV_HANDLER_DEVICE_COUNT);
        class_destroy(env_handler_class);
        return s32Ret;
    }

    device_create(env_handler_class, NULL, MKDEV(env_handler_dev.env_handler_major, 1), NULL, CONFIG_MSTAR_ENV_HANDLER_1);
    device_create(env_handler_class, NULL, MKDEV(env_handler_dev.env_handler_major, 2), NULL, CONFIG_MSTAR_ENV_HANDLER_2);

    return 0;
}


 void __exit mod_env_handler_exit(void)
{
    cdev_del(&env_handler_dev.env_handler_device);
    unregister_chrdev_region(MKDEV(env_handler_dev.env_handler_major, env_handler_dev.env_handler_minor), MOD_ENV_HANDLER_DEVICE_COUNT);

    device_destroy(env_handler_class, MKDEV(env_handler_dev.env_handler_major, env_handler_dev.env_handler_minor));
    class_destroy(env_handler_class);
}

module_init(mod_env_handler_init);
module_exit(mod_env_handler_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("Env handler driver");
MODULE_LICENSE("GPL");

