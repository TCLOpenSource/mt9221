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
/// file    mdrv_teeshm.c
/// @brief  TEE SHM Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
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
#include <linux/device.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/compat.h>
#include <linux/syscalls.h>
#include <linux/proc_fs.h>



#if defined(CONFIG_MIPS)
#include <asm/mips-boards/prom.h>
#else
#include <asm/outercache.h>
#endif

#include "mdrv_teeshm.h"
#include "mst_devid.h"
#include "mdrv_types.h"
#include "mst_platform.h"
#include "mdrv_system.h"

#if !(defined(CONFIG_MSTAR_TITANIA3) || defined(CONFIG_MSTAR_TITANIA10))
#include "chip_setup.h"
#endif

#include <linux/dma-contiguous.h>	// for tee_shm doing flush
//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global variables
//-------------------------------------------------------------------------------------------------
static DEFINE_MUTEX(teeshm_mutex);
static int MAP_CACHED = 0;
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define MOD_TEESHM_DEVICE_COUNT		1
#define MOD_TEESHM_NAME				"mstar_share_resource"

#define SHAREDMEMORY_DEFAULT_CNT 20
#define SHM_NAME_LEN 32
#define SHM_CNT 20
#define SHM_INIT_ID 9887

#define TEESHM_SHOW KERN_ERR			// this is error message

#ifdef TEESHM_DEBUG						// this is debug message
#define TEESHM_DPRINTK(fmt, args...)      printk("[TEESHM (Driver)][%05d] " fmt, __LINE__, ## args)
#else
#define TEESHM_DPRINTK(fmt, args...)
#endif

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------;
static struct class *teeshm_class;

typedef struct {
	int                         s32TeeShmMajor;
	int                         s32TeeShmMinor;
	struct cdev                 cDevice;
	struct file_operations      TeeShmFop;
} TeeShmModHandle;

typedef struct {
	char name[SHM_NAME_LEN];
	u16 m_shm_id;					// we will initialize this as "9887"
	u32 create_pid;
	u32 create_tgid;
	struct list_head pid_va_list;
	u32 size;
	struct page *alloc_page;
	atomic_t vma_ref_count;

	struct mutex obtain_mutex;		// once this shm is obtained, lock this mutex
} TEE_SHM_INFO;

typedef struct {
	u16 mmap_shm_id;
	u16 un_mmap_shm_id;
	struct vm_area_struct *mmaped_vma[SHM_CNT];
} TeeShmPrivate;

//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int  _MDrv_TEESHM_Open(struct inode *inode, struct file *filp);
static int  _MDrv_TEESHM_Release(struct inode *inode, struct file *filp);
static int  _MDrv_TEESHM_MMap(struct file *filp, struct vm_area_struct *vma);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long _MDrv_TEESHM_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int  _MDrv_TEESHM_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif

#if defined(CONFIG_COMPAT)
static long Compat_MDrv_TEESHM_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

//-------------------------------------------------------------------------------------------------
// Local Variables
//-------------------------------------------------------------------------------------------------
static TeeShmModHandle TeeShmDev = {

	.s32TeeShmMajor =               MDRV_MAJOR_TEESHM,
	.s32TeeShmMinor =               MDRV_MINOR_TEESHM,
	.cDevice = {

		.kobj = {.name = MOD_TEESHM_NAME, },
		.owner  =               THIS_MODULE,
	},
	.TeeShmFop = {

		.open =                   _MDrv_TEESHM_Open,
		.release =                _MDrv_TEESHM_Release,	// when process is terminated
		.mmap =                   _MDrv_TEESHM_MMap,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
		.unlocked_ioctl = _MDrv_TEESHM_Ioctl,
#else
		.ioctl =                  _MDrv_TEESHM_Ioctl,
#endif

#if defined(CONFIG_COMPAT)
		.compat_ioctl =          Compat_MDrv_TEESHM_Ioctl,
#endif
	},
};

static TEE_SHM_INFO tee_shm_info[SHM_CNT];

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static vm_fault_t teeshm_vma_fault(struct vm_fault *vmf)
#else
static int teeshm_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#endif
{
	TEESHM_DPRINTK("semutex_vma_fault!!!\n");
	BUG();
	return -ENOMEM;
}

static void teeshm_vma_close(struct vm_area_struct *vma)
{
	TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, close vma\033[m\n", __PRETTY_FUNCTION__, __LINE__);
	return;
}

static struct vm_operations_struct teeshm_vm_ops = {
	.close = teeshm_vma_close,
	.fault = teeshm_vma_fault,
};

// Debug_API
void show_tee_shm_info(void)
{
	int i = 0;

	for (i = 0; i < SHM_CNT; i++)
		printk(KERN_ERR "\033[35mFunction = %s, Line = %d, [%d] m_shm_id: %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, i, tee_shm_info[i].m_shm_id);

	return;
}

// this is copy from __dma_clear_buffer2, for some chip config doesnot enable CMA
static void teeshm_clear_buffer(struct page *page, size_t size)
{
	void *ptr;

	if (!page)
		return;
	ptr = page_address(page);
	if (ptr) {
		memset(ptr, 0, size);
		dmac_flush_range(ptr, ptr + size);
		outer_flush_range(__pa(ptr), __pa(ptr) + size);
	}
}

static int _MDrv_TEESHM_Open(struct inode *inode, struct file *filp)
{
	TeeShmPrivate *pdev = kzalloc(sizeof(TeeShmPrivate), GFP_KERNEL);

	if (pdev == NULL)
		return -ENOMEM;

	TEESHM_DPRINTK("\033[31mFunction = %s, open /dev/mstar_share_resource!!\033[m\n", __PRETTY_FUNCTION__);

	pdev->mmap_shm_id = SHM_INIT_ID;
	pdev->un_mmap_shm_id = SHM_INIT_ID;

	filp->private_data = pdev;

#if 0
	filp_private *pdev = kzalloc(sizeof(filp_private), GFP_KERNEL);

	if (pdev == NULL)
		return -ENOMEM;

	pdev->pid = task_pid_nr(current->group_leader);
	INIT_LIST_HEAD(&pdev->list_heap_proc_info);
	mutex_init(&pdev->lock);
	filp->private_data = pdev;
#endif

	return 0;
}

static int _MDrv_TEESHM_Release(struct inode *inode, struct file *filp)
{
	TeeShmPrivate *pdev = (TeeShmPrivate *)filp->private_data;

	kfree(pdev);
	TEESHM_DPRINTK("\033[31mFunction = %s, close /dev/mstar_share_resource !!\033[m\n", __PRETTY_FUNCTION__);
	return 0;
}

static int _MDrv_TEESHM_MMap(struct file *filp, struct vm_area_struct *vma)
{
	TeeShmPrivate *pdev = (TeeShmPrivate *)filp->private_data;

	/* we will alloc 2^order pages, but we will not map 2^order pages */
	/* ex: alloc_size is 0x2001, we will ask 2^2 = 4 pages, but we only mmap 3 pages here */
	if (vma == NULL)
		return -EINVAL;

	/* set page to cache/non-cache */
	if (MAP_CACHED)
	{
		TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, map as cache\033[m\n", __PRETTY_FUNCTION__, __LINE__);
#if defined(CONFIG_MIPS)
		pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
		pgprot_val(vma->vm_page_prot) |= _page_cachable_default;
#elif defined(CONFIG_ARM)
		vma->vm_page_prot = __pgprot_modify(vma->vm_page_prot, L_PTE_MT_MASK, L_PTE_MT_DEV_CACHED);
#elif defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 32)
		pgprot_val(vma->vm_page_prot) = pgprot_val(pgprot_cached(vma->vm_page_prot));
#else
		pgprot_val(vma->vm_page_prot) = pgprot_cached(vma->vm_page_prot);
#endif
#endif
	} else
	{
		TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, map as non-cache\033[m\n", __PRETTY_FUNCTION__, __LINE__);
#if defined(CONFIG_MIPS)
		pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
		pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 32)
		pgprot_val(vma->vm_page_prot) = pgprot_val(pgprot_dmacoherent(vma->vm_page_prot));
#else
		pgprot_val(vma->vm_page_prot) = pgprot_dmacoherent(vma->vm_page_prot);
#endif
#endif
	}

	vma->vm_ops = &teeshm_vm_ops;

	// keep_vma(multi-process env should make sure only one can do mmap_flow at the same time(including specify m_shm_id)
	TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, start: 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, vma->vm_start);
	TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, end: 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, vma->vm_end);
	if (pdev->mmap_shm_id == SHM_INIT_ID)
	{
		printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, does not specify mmap_shm_id, mmap_shm_id is %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, pdev->mmap_shm_id);
		return -EINVAL;
	} else
	{
		remap_pfn_range(vma, vma->vm_start, page_to_pfn(tee_shm_info[pdev->mmap_shm_id].alloc_page), (vma->vm_end - vma->vm_start), vma->vm_page_prot);
		atomic_inc(&tee_shm_info[pdev->mmap_shm_id].vma_ref_count);
		pdev->mmaped_vma[pdev->mmap_shm_id] = vma;
		pdev->mmap_shm_id = SHM_INIT_ID;
		return 0;
	}
}



#if defined(CONFIG_COMPAT)
static long Compat_MDrv_TEESHM_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	/* no need to do any compat */
	case TEESHM_IOC_RESOURCE_DESTROY:
	case TEESHM_IOC_RESOURCE_OBTAIN:
	case TEESHM_IOC_RESOURCE_RELEASE:
		{
			return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
		}

		/* compat handle */
	case COMPAT_TEESHM_IOC_RESOURCE_CREATE:
		{
			compat_uptr_t uptr;
			compat_ulong_t ulong;
			__compat_uid16_t u16id;

			DrvTeeShmResourceCreate_32 __user *data32;

			DrvTeeShmResourceCreate __user *data;

			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			data32 = compat_ptr(arg);

			/* copy 32_bit data to 64_bit data */
			err = get_user(uptr, &data32->name);
			err |= put_user(uptr, &data->name);

			err |= get_user(ulong, &data32->size);
			err |= put_user(ulong, &data->size);

			err |= get_user(u16id, &data32->m_shm_id);
			err |= put_user(u16id, &data->m_shm_id);

			if (err)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, COMPAT_TEESHM_IOC_RESOURCE_CREATE handle compat data(32bit --> 64bit) error!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				return err;
			}

			err = filp->f_op->unlocked_ioctl(filp, TEESHM_IOC_RESOURCE_CREATE, (unsigned long)data);

			if (err)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE error!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				return err;
			}

			/* copy 32_bit data to 64_bit data */
			err = get_user(uptr, &data->name);
			err |= put_user(uptr, &data32->name);

			err |= get_user(ulong, &data->size);
			err |= put_user(ulong, &data32->size);

			err |= get_user(u16id, &data->m_shm_id);
			err |= put_user(u16id, &data32->m_shm_id);

			if (err)
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, COMPAT_TEESHM_IOC_RESOURCE_CREATE handle compat data(64_bit --> 32bit) error!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);

			return err;
		}
	case COMPAT_TEESHM_IOC_RESOURCE_MAP:
		{
			__compat_uid16_t u16id;
			compat_uptr_t uptr;

			DrvTeeShmResourceMap_32 __user *data32;

			DrvTeeShmResourceMap __user *data;

			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			data32 = compat_ptr(arg);

			err = get_user(u16id, &data32->m_shm_id);
			err |= put_user(u16id, &data->m_shm_id);

			err |= get_user(uptr, &data32->va);
			err |= put_user(uptr, &data->va);

			if (err)
				return err;

			err = filp->f_op->unlocked_ioctl(filp, TEESHM_IOC_RESOURCE_MAP, (unsigned long)data);

			if (err)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, COMPAT_TEESHM_IOC_RESOURCE_MAP error!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				return err;
			}

			/* copy 32_bit data to 64_bit data */
			err |= get_user(u16id, &data->m_shm_id);
			err |= put_user(u16id, &data32->m_shm_id);

			err |= get_user(uptr, &data->va);
			err |= put_user(uptr, &data32->va);

			if (err)
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, COMPAT_TEESHM_IOC_RESOURCE_CREATE handle compat data(64_bit --> 32bit) error!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);

			return err;
		}
	case COMPAT_TEESHM_IOC_RESOURCE_UNMAP:
		{
			__compat_uid16_t u16id;
			compat_uptr_t uptr;

			DrvTeeShmResourceUnmap_32 __user *data32;

			DrvTeeShmResourceUnmap __user *data;

			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			data32 = compat_ptr(arg);

			err = get_user(u16id, &data32->m_shm_id);
			err |= put_user(u16id, &data->m_shm_id);

			err |= get_user(uptr, &data32->va);
			err |= put_user(uptr, &data->va);

			if (err)
				return err;

			err = filp->f_op->unlocked_ioctl(filp, TEESHM_IOC_RESOURCE_UNMAP, (unsigned long)data);

			if (err)
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, COMPAT_TEESHM_IOC_RESOURCE_UNMAP error!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);

			return err;
		}
	default:
		printk(TEESHM_SHOW "Function = %s, Line = %d, Unknown ioctl command %u\n",  __PRETTY_FUNCTION__, __LINE__, cmd);

		return -ENOTTY;
	}
}
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long _MDrv_TEESHM_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_TEESHM_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	int err	= 0;
	int search_cnt = 0;
	char  __user *pu8UserParam = NULL;
	char tmp[SHM_NAME_LEN];

	TeeShmPrivate *pdev = (TeeShmPrivate *)filp->private_data;

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != TEESHM_IOC_MAGIC)
		return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)		// _IOC_DIR is this ioctl_cma type (r/w, r, w, none), _IOC_READ means read_data from device, so we must make sure the arg can be writen
		err |= !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		err |= !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	mutex_lock(&teeshm_mutex);
	switch (cmd) {
	case TEESHM_IOC_RESOURCE_CREATE:
		{
			DrvTeeShmResourceCreate to;
			int alloc_page_order;
			struct page *obtain_page = NULL;

			if (copy_from_user(&to, (void __user *)arg, _IOC_SIZE(cmd))) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE copy_from_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			/*Fix Kasan user-memory-access error*/
			pu8UserParam = to.name;

			if (pu8UserParam == NULL) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d \033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -EFAULT;
			}

			to.name = tmp;

			if (to.name == NULL) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d \033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOMEM;
			}


			if (copy_from_user(to.name, pu8UserParam, SHM_NAME_LEN - 1)) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE copy_from_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}


			for (search_cnt = 0; search_cnt < SHM_CNT; search_cnt++) {
				if (strncmp(tee_shm_info[search_cnt].name, to.name, SHM_NAME_LEN-1) == 0) {
					printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE ReCreate %s \033[m\n", __PRETTY_FUNCTION__, __LINE__, to.name);
					tee_shm_info[search_cnt].create_pid = current->pid;
					tee_shm_info[search_cnt].create_tgid = current->tgid;
					printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE ReCreate %s Change owner pid %x \033[m\n", __PRETTY_FUNCTION__, __LINE__, to.name, (unsigned int)current->pid );
					to.m_shm_id = search_cnt;	// this will return to user_mode
					if (copy_to_user((void __user *)arg, &to, _IOC_SIZE(cmd)))
					{
						/* we need to reset tee_shm_info */
						INIT_LIST_HEAD(&tee_shm_info[search_cnt].pid_va_list);
						mutex_destroy(&tee_shm_info[search_cnt].obtain_mutex);

						memset(&tee_shm_info[search_cnt], 0, sizeof(TEE_SHM_INFO));
						tee_shm_info[search_cnt].m_shm_id = SHM_INIT_ID;
						printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE copy_to_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
						mutex_unlock(&teeshm_mutex);
						return -ENOTTY;
					}
					mutex_unlock(&teeshm_mutex);
					return 0;
				}
			}

			for (search_cnt = 0; search_cnt < SHM_CNT; search_cnt++)
			{
				if (tee_shm_info[search_cnt].m_shm_id != SHM_INIT_ID)	// not SHM_INIT_ID, this one is used
					continue;

				to.m_shm_id = search_cnt;	// this will return to user_mode
				/* init this shm_info */
				strncpy(tee_shm_info[search_cnt].name, to.name, SHM_NAME_LEN-1);
				tee_shm_info[search_cnt].name[SHM_NAME_LEN-1] = '\0';
				tee_shm_info[search_cnt].m_shm_id = search_cnt;	// index = m_shm_id
				tee_shm_info[search_cnt].create_pid = current->pid;
				tee_shm_info[search_cnt].create_tgid = current->tgid;
				INIT_LIST_HEAD(&tee_shm_info[search_cnt].pid_va_list);
				tee_shm_info[search_cnt].size = to.size;
				tee_shm_info[search_cnt].alloc_page = NULL;
				mutex_init(&tee_shm_info[search_cnt].obtain_mutex);

				TEESHM_DPRINTK("\033[34mFunction = %s, Line = %d, [%d] create a new TEE_SHM: %s\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, tee_shm_info[search_cnt].m_shm_id, tee_shm_info[search_cnt].name);

				TEESHM_DPRINTK("\033[34mcreate_pid is %u, create_tgid is %u, size is %u\033[m\n",
						tee_shm_info[search_cnt].create_pid, tee_shm_info[search_cnt].create_tgid, tee_shm_info[search_cnt].size);

				break;
			}

			if (search_cnt == SHM_CNT)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE cannot find an un-used tee_shm_info!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (copy_to_user((void __user *)arg, &to, _IOC_SIZE(cmd)))
			{
				/* we need to reset tee_shm_info */
				INIT_LIST_HEAD(&tee_shm_info[search_cnt].pid_va_list);
				mutex_destroy(&tee_shm_info[search_cnt].obtain_mutex);

				//show_tee_shm_info();
				memset(&tee_shm_info[search_cnt], 0, sizeof(TEE_SHM_INFO));
				tee_shm_info[search_cnt].m_shm_id = SHM_INIT_ID;
				//show_tee_shm_info();
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_CREATE copy_to_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			alloc_page_order = get_order(tee_shm_info[search_cnt].size);
			TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, alloc %u bytes(%d pages) for this shm\033[m\n", __PRETTY_FUNCTION__, __LINE__, tee_shm_info[search_cnt].size, 1 << alloc_page_order);
			obtain_page = alloc_pages(GFP_KERNEL, alloc_page_order);

			if (!obtain_page)
			{
				/* we need to reset tee_shm_info */
				INIT_LIST_HEAD(&tee_shm_info[search_cnt].pid_va_list);
				mutex_destroy(&tee_shm_info[search_cnt].obtain_mutex);

				//show_tee_shm_info();
				memset(&tee_shm_info[search_cnt], 0, sizeof(TEE_SHM_INFO));
				tee_shm_info[search_cnt].m_shm_id = SHM_INIT_ID;
				//show_tee_shm_info();
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, get %u bytes for this shm fail!!\033[m\n", __PRETTY_FUNCTION__, __LINE__, tee_shm_info[search_cnt].size);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}
			TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, get va is 0x%lX, ba is 0x%lX, go flush with size 0x%lX\033[m\n",
					__PRETTY_FUNCTION__, __LINE__, (unsigned long)page_address(obtain_page), __pa(page_address(obtain_page)), PAGE_SIZE << alloc_page_order);
			tee_shm_info[search_cnt].alloc_page = obtain_page;
			teeshm_clear_buffer(obtain_page, PAGE_SIZE << alloc_page_order);

			break;
		}
	case TEESHM_IOC_RESOURCE_DESTROY:
		{
			DrvTeeShmResourceDestroy to;
			int free_page_order;
			//struct vm_area_struct *unmap_vma = NULL;

			if (copy_from_user(&to, (void __user *)arg, _IOC_SIZE(cmd)))
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_DESTROY copy_from_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (to.m_shm_id >= SHM_CNT)
				return -ENOTTY;

			search_cnt = to.m_shm_id;
			if (tee_shm_info[search_cnt].m_shm_id == SHM_INIT_ID)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, tee_shm_info[%d].m_shm_id is %d, not created...\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt, tee_shm_info[search_cnt].m_shm_id);
				show_tee_shm_info();
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			/* use trylock, if pass, then nobody is obtaining this shm, we can destroy it. If not, then this shm is occupied now, can not be destroyed */
			if (mutex_trylock(&tee_shm_info[search_cnt].obtain_mutex))
			{
				TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, get %d resourse mutex pass!!, we can destroy this shm\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt);
			} else
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, get %d resourse mutex fail!!, we cannot destroy this shm\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

#if 0
			unmap_vma = pdev->mmaped_vma[search_cnt];
			if (unmap_vma)
			{
				TEESHM_DPRINTK("\033[31mFunction = %s, Line = %d, unmap while destroy!! user should do unmap itself\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				sys_munmap((unsigned long)unmap_vma->vm_start, (size_t)(unmap_vma->vm_end - unmap_vma->vm_start));
				atomic_dec(&tee_shm_info[search_cnt].vma_ref_count);
				pdev->mmaped_vma[search_cnt] = NULL;
			}
#endif	// this only unmap one vma...!! user should unmap all vmas by itself

			/* [DONE BY USER] need to unmap all mappings before TEESHM_IOC_RESOURCE_DESTROY, vma_ref_count means how many mmaping are on this shm_id, and pdev->mmaped_vma[search_cnt] only specify one vma on this shm_id(current_fd) */
			if (atomic_read(&tee_shm_info[search_cnt].vma_ref_count) != 0)
			{
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, the vma_ref_count of shm_id(%d) is %d, ERROR!!, user should unmap all mapping before destroy\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, search_cnt, atomic_read(&tee_shm_info[search_cnt].vma_ref_count));
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, the vma_ref_count of shm_id(%d) is %d, ERROR!!, user should unmap all mapping before destroy\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, search_cnt, atomic_read(&tee_shm_info[search_cnt].vma_ref_count));
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, the vma_ref_count of shm_id(%d) is %d, ERROR!!, user should unmap all mapping before destroy\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, search_cnt, atomic_read(&tee_shm_info[search_cnt].vma_ref_count));
			}

			/* free_memory(if alloc) */
			if (tee_shm_info[search_cnt].alloc_page != NULL)
			{
				free_page_order = get_order(tee_shm_info[search_cnt].size);
				TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, free %u bytes(%d pages) for this shm\033[m\n", __PRETTY_FUNCTION__, __LINE__, tee_shm_info[search_cnt].size, 1 << free_page_order);
				__free_pages(tee_shm_info[search_cnt].alloc_page, free_page_order);
				tee_shm_info[search_cnt].alloc_page = NULL;
			} else
			{
				printk(TEESHM_SHOW "\033[34mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_DESTROY alloc_page is NULL\033[m\n", __PRETTY_FUNCTION__, __LINE__);
			}

			INIT_LIST_HEAD(&tee_shm_info[search_cnt].pid_va_list);
			mutex_unlock(&tee_shm_info[search_cnt].obtain_mutex);
			mutex_destroy(&tee_shm_info[search_cnt].obtain_mutex);

			//show_tee_shm_info();
			memset(&tee_shm_info[search_cnt], 0, sizeof(TEE_SHM_INFO));
			tee_shm_info[search_cnt].m_shm_id = SHM_INIT_ID;
			memset(tee_shm_info[search_cnt].name, 0, SHM_NAME_LEN);
			atomic_set(&tee_shm_info[search_cnt].vma_ref_count, 0);
			//show_tee_shm_info();

			break;
		}
	case TEESHM_IOC_RESOURCE_OBTAIN:		// lock_mutex
		{
			DrvTeeShmResourceObtain to;

			if (copy_from_user(&to, (void __user *)arg, _IOC_SIZE(cmd)))
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_OBTAIN copy_from_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (to.m_shm_id >= SHM_CNT)
				return -ENOTTY;

			search_cnt = to.m_shm_id;
			if (tee_shm_info[search_cnt].m_shm_id == SHM_INIT_ID)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, tee_shm_info[%d].m_shm_id is %d, not created...\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt, tee_shm_info[search_cnt].m_shm_id);
				show_tee_shm_info();
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			mutex_unlock(&teeshm_mutex);
			mutex_lock(&tee_shm_info[search_cnt].obtain_mutex);
			return 0;

			break;
		}
	case TEESHM_IOC_RESOURCE_RELEASE:	// unlock_mutex
		{
			DrvTeeShmResourceRelease to;

			if (copy_from_user(&to, (void __user *)arg, _IOC_SIZE(cmd)))
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TEESHM_IOC_RESOURCE_RELEASE copy_from_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (to.m_shm_id >= SHM_CNT)
				return -ENOTTY;

			search_cnt = to.m_shm_id;
			if (tee_shm_info[search_cnt].m_shm_id == SHM_INIT_ID)
			{
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, tee_shm_info[%d].m_shm_id is %d, not created...\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt, tee_shm_info[search_cnt].m_shm_id);
				show_tee_shm_info();
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			mutex_unlock(&teeshm_mutex);
			mutex_unlock(&tee_shm_info[search_cnt].obtain_mutex);
			return 0;

			break;
		}
	case TEESHM_IOC_RESOURCE_MAP:
		{
			DrvTeeShmResourceMap to;

			if (copy_from_user(&to, (void __user *)arg, _IOC_SIZE(cmd)))
			{
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, TTEESHM_IOC_RESOURCE_MAP copy_from_user failed!!\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (to.m_shm_id >= SHM_CNT)
				return -ENOTTY;

			search_cnt = to.m_shm_id;
			if (tee_shm_info[search_cnt].m_shm_id == SHM_INIT_ID)
			{
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, tee_shm_info[%d].m_shm_id is %d, not created...\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt, tee_shm_info[search_cnt].m_shm_id);
				show_tee_shm_info();
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (pdev->mmap_shm_id != SHM_INIT_ID)
			{
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, mmap_shm_id is %d, mmap can be done once at the same time\033[m\n", __PRETTY_FUNCTION__, __LINE__, pdev->mmap_shm_id);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (pdev->mmaped_vma[to.m_shm_id] != NULL)
			{
				printk(TEESHM_SHOW "\033[31mFunction = %s, Line = %d, this fd is already mapping va for shm_id: %d, from 0x%lX to 0x%lX, we wont map again\033[m\n",
						__PRETTY_FUNCTION__, __LINE__, to.m_shm_id, pdev->mmaped_vma[to.m_shm_id]->vm_start, pdev->mmaped_vma[to.m_shm_id]->vm_end);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			pdev->mmap_shm_id = to.m_shm_id;
			TEESHM_DPRINTK("\033[34mFunction = %s, Line = %d, mmap_shm_id is set to %d\033[m\n", __PRETTY_FUNCTION__, __LINE__, pdev->mmap_shm_id);

			break;
		}
	case TEESHM_IOC_RESOURCE_UNMAP:
		{
			DrvTeeShmResourceUnmap to;
			void *unmap_start_va = NULL;
			struct vm_area_struct *unmap_vma = NULL;

			if (copy_from_user(&to, (void __user *)arg, _IOC_SIZE(cmd))) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, TTEESHM_IOC_RESOURCE_MAP copy_from_user failed!!\033[m\n", __func__, __LINE__);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			if (to.m_shm_id >= SHM_CNT)
				return -ENOTTY;

			search_cnt = (int)to.m_shm_id;
			unmap_start_va = to.va;

			if (tee_shm_info[search_cnt].m_shm_id == SHM_INIT_ID) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, tee_shm_info[%d].m_shm_id is %d, not created...\033[m\n", __PRETTY_FUNCTION__, __LINE__, search_cnt, tee_shm_info[search_cnt].m_shm_id);
				show_tee_shm_info();
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}

			unmap_vma = pdev->mmaped_vma[search_cnt];
			TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, (copy from user) unmap_start_va is 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, (unsigned long)unmap_start_va);
			TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, (from kernel) unmap_vma->vm_start is 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, unmap_vma->vm_start);
			TEESHM_DPRINTK("\033[35mFunction = %s, Line = %d, unmap from 0x%lX to 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, unmap_vma->vm_start, unmap_vma->vm_end);

			if ((unsigned long)unmap_start_va != unmap_vma->vm_start) {
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, unmap_start_va is 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, (unsigned long)unmap_start_va);
				printk(TEESHM_SHOW "\033[35mFunction = %s, Line = %d, unmap_vma->vm_start is 0x%lX\033[m\n", __PRETTY_FUNCTION__, __LINE__, unmap_vma->vm_start);
				mutex_unlock(&teeshm_mutex);
				return -ENOTTY;
			}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
			vm_munmap((unsigned long)unmap_start_va, (size_t)(unmap_vma->vm_end - unmap_vma->vm_start));
#else
			sys_munmap((unsigned long)unmap_start_va, (size_t)(unmap_vma->vm_end - unmap_vma->vm_start));
#endif
			atomic_dec(&tee_shm_info[search_cnt].vma_ref_count);
			pdev->mmaped_vma[search_cnt] = NULL;

			break;
		}
	default:
		printk(TEESHM_SHOW "Unknown ioctl command %d\n", cmd);

		mutex_unlock(&teeshm_mutex);
		return -ENOTTY;
	}

	mutex_unlock(&teeshm_mutex);
	return 0;
}

MSYSTEM_STATIC int __init mod_teeshm_init(void)
{
	int s32Ret, search_cnt;
	dev_t dev;

	teeshm_class = class_create(THIS_MODULE, "teeshm");
	if (IS_ERR(teeshm_class))
	{
		return PTR_ERR(teeshm_class);
	}

	if (TeeShmDev.s32TeeShmMajor)
	{
		dev = MKDEV(TeeShmDev.s32TeeShmMajor, TeeShmDev.s32TeeShmMinor);
		s32Ret = register_chrdev_region(dev, MOD_TEESHM_DEVICE_COUNT, MOD_TEESHM_NAME);
	} else
	{
		s32Ret = alloc_chrdev_region(&dev, TeeShmDev.s32TeeShmMinor, MOD_TEESHM_DEVICE_COUNT, MOD_TEESHM_NAME);
		TeeShmDev.s32TeeShmMajor = MAJOR(dev);
	}

	if (s32Ret < 0)
	{
		printk(TEESHM_SHOW "Unable to get major %d\n", TeeShmDev.s32TeeShmMajor);
		class_destroy(teeshm_class);
		return s32Ret;
	}

	cdev_init(&TeeShmDev.cDevice, &TeeShmDev.TeeShmFop);
	if (0 != (s32Ret = cdev_add(&TeeShmDev.cDevice, dev, MOD_TEESHM_DEVICE_COUNT)))
	{
		printk(TEESHM_SHOW "Unable add a character device\n");
		unregister_chrdev_region(dev, MOD_TEESHM_DEVICE_COUNT);
		class_destroy(teeshm_class);
		return s32Ret;
	}
	device_create(teeshm_class, NULL, dev, NULL, MOD_TEESHM_NAME);

	memset(tee_shm_info, 0, sizeof(TEE_SHM_INFO) * SHM_CNT);
	for (search_cnt = 0; search_cnt < SHM_CNT; search_cnt++) {
		tee_shm_info[search_cnt].m_shm_id = SHM_INIT_ID;	// initialize the id with 9887
		memset(tee_shm_info[search_cnt].name, 0, SHM_NAME_LEN);
		atomic_set(&tee_shm_info[search_cnt].vma_ref_count, 0);
	}

	proc_create(MOD_TEESHM_NAME, S_IRUSR | S_IWUSR, NULL, &TeeShmDev.TeeShmFop);

	return 0;
}

MSYSTEM_STATIC void __exit mod_teeshm_exit(void)
{
	remove_proc_entry(MOD_TEESHM_NAME, NULL);

	cdev_del(&TeeShmDev.cDevice);
	unregister_chrdev_region(MKDEV(TeeShmDev.s32TeeShmMajor, TeeShmDev.s32TeeShmMinor), MOD_TEESHM_DEVICE_COUNT);

	device_destroy(teeshm_class, MKDEV(TeeShmDev.s32TeeShmMajor, TeeShmDev.s32TeeShmMinor));
	class_destroy(teeshm_class);
}

module_init(mod_teeshm_init);
module_exit(mod_teeshm_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("TEESHM driver");
MODULE_LICENSE("GPL");
