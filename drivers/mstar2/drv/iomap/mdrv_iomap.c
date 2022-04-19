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
/// file    mdrv_iomap.c
/// @brief  Memory IO remap Control Interface
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
#include <linux/time.h>  //added
#include <linux/timer.h> //added
#include <linux/device.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>

#include "mst_devid.h"
#include "mdrv_iomap.h"
#include "mdrv_types.h"
#include "mst_platform.h"
#include "mdrv_system.h"
//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
//#define MIOMAP_DPRINTK(fmt, args...) printk(KERN_WARNING"%s:%d " fmt,__FUNCTION__,__LINE__,## args)
#define MIOMAP_DPRINTK(fmt, args...)

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

#define MOD_MIOMAP_DEVICE_COUNT     1
#define MOD_MIOMAP_NAME             "miomap"

// Define MIOMAP Device

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------;

typedef struct
{
    int                         s32MIOMapMajor;
    int                         s32MIOMapMinor;
    void*                       dmaBuf;
    struct cdev                 cDevice;
    struct file_operations      MIOMapFop;
} MIOMapModHandle;

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
typedef  struct
{
   U32 miomap_base;
   U32 miomap_size;
}MMIO_FileData;
#elif defined(CONFIG_ARM64)
typedef  struct
{
   u64 miomap_base;
   u64 miomap_size;
}MMIO_FileData;
#endif
//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int                      _MDrv_MIOMAP_Open (struct inode *inode, struct file *filp);
static int                      _MDrv_MIOMAP_Release(struct inode *inode, struct file *filp);
static int                      _MDrv_MIOMAP_MMap(struct file *filp, struct vm_area_struct *vma);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long                     _MDrv_MIOMAP_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int                      _MDrv_MIOMAP_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif

#if defined(CONFIG_COMPAT)
static long Compat_MDrv_MIOMAP_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

//-------------------------------------------------------------------------------------------------
// Local Variables
//-------------------------------------------------------------------------------------------------
static struct class *miomap_class;

static MIOMapModHandle MIOMapDev=
{
    .s32MIOMapMajor=               MDRV_MAJOR_MIOMAP,
    .s32MIOMapMinor=               MDRV_MINOR_MIOMAP,
    .cDevice=
    {
        .kobj=                  {.name= MOD_MIOMAP_NAME, },
        .owner  =               THIS_MODULE,
    },
    .MIOMapFop=
    {
        .open=                  _MDrv_MIOMAP_Open,
        .release=               _MDrv_MIOMAP_Release,
        .mmap=                  _MDrv_MIOMAP_MMap,
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
        .unlocked_ioctl=        _MDrv_MIOMAP_Ioctl,
	#else
        .ioctl =                _MDrv_MIOMAP_Ioctl,
	#endif
	#if defined(CONFIG_COMPAT)
		.compat_ioctl =          Compat_MDrv_MIOMAP_Ioctl,
	#endif
    },
};

extern struct mutex mpool_iomap_mutex;

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------

static int _MDrv_MIOMAP_Open (struct inode *inode, struct file *filp)
{

    MMIO_FileData *mmioData;

    MIOMAP_DPRINTK("LennyD\n");


    mmioData = kzalloc(sizeof(*mmioData), GFP_KERNEL);
    if (mmioData == NULL)
         return -ENOMEM;

    filp->private_data = mmioData;

    return 0;
}

static int _MDrv_MIOMAP_Release(struct inode *inode, struct file *filp)
{
     MMIO_FileData *mmioData = filp->private_data ;
     kfree(mmioData);

    MIOMAP_DPRINTK("LennyD\n");
    // iounmap(dev->dmaBuf) ;
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static vm_fault_t _miomap_cpu_page_fault_handler(struct vm_fault *vmf)
#else
int _miomap_cpu_page_fault_handler(struct vm_area_struct *vma, struct vm_fault *vmf)
#endif
{
	// 1 Bank -> 512Bytes
	// 1 Page -> 8 Banks
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	unsigned long bank_base = 0;
	unsigned long phys_offset, virt_offset = 0;
	struct vm_area_struct *vma = vmf->vma;

	printk(KERN_EMERG "======================================== @@START@@ ========================================\n");
	printk(KERN_EMERG "\033[31mFunction = %s, Line = %d, pid is %d, tgid is %d, comm is %s\033[m\n", __PRETTY_FUNCTION__, __LINE__, current->pid, current->tgid, current->comm);
	printk(KERN_EMERG "\033[35mvmf->pgoff   (fault_phys_address) = 0x%lX\033[m\n", vmf->pgoff << PAGE_SHIFT);
	printk(KERN_EMERG "\033[31mvmf->address (fault_virt_address) = 0x%lX\n\033[m\n", vmf->address);
	printk(KERN_EMERG "\033[31mvmf->fault_real_address (fault_virt_address) = 0x%lX\n\033[m\n", vmf->fault_real_address);

	if ((unsigned long)vma->vm_private_data == 0x1F000000)
	{
		printk(KERN_EMERG "\033[31mPM Bank, size 0x%lX\033[m\n", vma->vm_end - vma->vm_start);
		bank_base = 0;
	}
	else if ((unsigned long)vma->vm_private_data == 0x1F200000)
	{
		printk(KERN_EMERG "\033[31mNon_PM Bank, size 0x%lX\033[m\n", vma->vm_end - vma->vm_start);
		bank_base = 0x1000;
	}
	else if ((unsigned long)vma->vm_private_data == 0x1F600000)
	{
		printk(KERN_EMERG "\033[31mEXT_RIU Bank, size 0x%lX\033[m\n", vma->vm_end - vma->vm_start);
		bank_base = 0x3000;
	}

	phys_offset = (vmf->pgoff << PAGE_SHIFT) - (unsigned long)vma->vm_private_data;	// fault phys_offset
	virt_offset = vmf->fault_real_address - vma->vm_start;							// fault virt_offset
	printk(KERN_EMERG "\033[31mphys_offset is 0x%lX\033[m\n", phys_offset);
	printk(KERN_EMERG "\033[31mvirt_offset is 0x%lX\033[m\n", virt_offset);
	printk(KERN_EMERG "\033[31mbank_base is 0x%lX\033[m\n", bank_base);
	printk(KERN_EMERG "\033[31mbank is 0x%lX\033[m\n", (virt_offset/2) >> 8);
	printk(KERN_EMERG "\033[31mreal bank is 0x%lX\033[m\n", bank_base + ((virt_offset/2) >> 8));
	printk(KERN_EMERG "\033[31m8_bit reg is 0x%lX\033[m\n", virt_offset/2 & 0xFF);
	printk(KERN_EMERG "======================================== @@END@@ ========================================\n");
#else
	void __user *address = vmf->virtual_address;
	printk(KERN_EMERG "\033[31mFunction = %s, Line = %d, vmf->virtual_address = %lX\n\033[m\n", __PRETTY_FUNCTION__, __LINE__, (unsigned long)address);
#endif
	return VM_FAULT_SIGBUS;
}

static struct vm_operations_struct miomap_vm_ops =
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	.fault = _miomap_cpu_page_fault_handler
#else
	.nopfn = _miomap_cpu_page_fault_handler
#endif
};

static int _MDrv_MIOMAP_MMap(struct file *filp, struct vm_area_struct *vma)
{
	MMIO_FileData *mmioData = filp->private_data ;

	mutex_lock(&mpool_iomap_mutex);
	vma->vm_pgoff = mmioData->miomap_base >> PAGE_SHIFT;
	vma->vm_private_data = (void *)mmioData->miomap_base;
	//printk(KERN_EMERG "\033[31mvma->vm_pgoff is 0x%lX\033[m\n", vma->vm_pgoff << PAGE_SHIFT);
	//printk(KERN_EMERG "\033[35mvma->vm_start is 0x%lX\033[m\n", vma->vm_start);
	//printk(KERN_EMERG "\033[35mvma->vm_end is 0x%lX\033[m\n", vma->vm_end);
	//dump_stack();

#if 0
	/* we allow only PM & Non-PM (IO_PHYS ~ IO_PHYS + IO_SIZE) / EXT_RIU (IO_XC_EXT_PHYS ~ IO_XC_EXT_PHYS + IO_XC_EXT_SIZE) */
	if (((mmioData->miomap_base < IO_PHYS) || (mmioData->miomap_base + mmioData->miomap_size > IO_PHYS + IO_SIZE))
#ifdef CONFIG_MP_PLATFORM_XC_EXT_MAPPING
		&& ((mmioData->miomap_base < IO_XC_EXT_PHYS) || (mmioData->miomap_base + mmioData->miomap_size > IO_XC_EXT_PHYS + IO_XC_EXT_SIZE))
#endif
		) {
		printk(KERN_EMERG "\033[31mwe allow only PM & Non-PM (IO_PHYS ~ IO_PHYS + IO_SIZE) / EXT_RIU (IO_XC_EXT_PHYS ~ IO_XC_EXT_PHYS + IO_XC_EXT_SIZE)\033[m\n");
		mutex_unlock(&mpool_iomap_mutex);
		return -EFAULT;
	}
#endif

	if (mmioData->miomap_base >= ARM_MIU0_BUS_BASE) {
		printk(KERN_EMERG "\033[31mmiomap_base 0x%lX over 0x%lX\033[m\n", mmioData->miomap_base, ARM_MIU0_BUS_BASE);
		mutex_unlock(&mpool_iomap_mutex);
		return -EFAULT;
	}

	if(vma->vm_end-vma->vm_start != PAGE_ALIGN(mmioData->miomap_size))
	{
		printk(KERN_EMERG "_MDrv_MIOMAP_MMap invalid argument, request io length is 0x%08lx, current io length is 0x%08tx\n--pgoff 0x%08lx, tid%d, pid%d\n",
			vma->vm_end-vma->vm_start , (size_t)mmioData->miomap_size, vma->vm_pgoff, current->tgid, current->pid);
		panic("It's not really a kernel bug. I panic here for purpose to notify you that user mode app has bug!");
		mutex_unlock(&mpool_iomap_mutex);
		return -EINVAL;
	}

	vma->vm_flags &= (~(VM_EXEC|VM_MAYEXEC));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
	pgprot_val(vma->vm_page_prot) = pgprot_val(vm_get_page_prot(vma->vm_flags));
#else
	pgprot_val(vma->vm_page_prot) = vm_get_page_prot(vma->vm_flags);
#endif
	/* set page to no cache */
#if defined(CONFIG_MIPS)
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
	pgprot_val(vma->vm_page_prot) = pgprot_val(pgprot_noncached(vma->vm_page_prot));
#else
	pgprot_val(vma->vm_page_prot) = pgprot_noncached(vma->vm_page_prot);
#endif
#endif

	vma->vm_ops = &miomap_vm_ops;

#if 0
	if(io_remap_pfn_range(vma, vma->vm_start,
		mmioData->miomap_base >> PAGE_SHIFT, mmioData->miomap_size,
		vma->vm_page_prot))
	{
		mutex_unlock(&mpool_iomap_mutex);
		return -EAGAIN;
	}
#else
#if 1
	unsigned long vaddr;
	unsigned long pfn;

	if(mmioData->miomap_base == 0x1F000000)
	{
		vaddr = vma->vm_start + (0x1800 << 1);
		pfn = (mmioData->miomap_base + (0x1800 << 1)) >> PAGE_SHIFT;
		//printk(KERN_EMERG "\033[35mPatch_1: Only PM Bank (pm_top): 0x18 ~ 0x20\033[m\n");
		//printk(KERN_EMERG "\033[35mremap from 0x%lX to 0x%lX\033[m\n", vaddr, vaddr + PAGE_SIZE);
		remap_pfn_range(vma, vaddr, pfn, PAGE_SIZE, vma->vm_page_prot);

		vaddr = vma->vm_start + (0x2000 << 1);
		pfn = (mmioData->miomap_base + (0x2000 << 1)) >> PAGE_SHIFT;
		//printk(KERN_EMERG "\033[35mPatch_1: Only PM Bank (efuse): 0x20 ~ 0x28\033[m\n");
		//printk(KERN_EMERG "\033[35mremap from 0x%lX to 0x%lX\033[m\n", vaddr, vaddr + PAGE_SIZE);
		remap_pfn_range(vma, vaddr, pfn, PAGE_SIZE, vma->vm_page_prot);
	}
#endif
#endif

	mutex_unlock(&mpool_iomap_mutex);

	return 0;
}

#if defined(CONFIG_COMPAT)
static long Compat_MDrv_MIOMAP_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
		case MIOMAP_IOC_CHIPINFO:
		case MIOMAP_IOC_SET_MAP:
		case MIOMAP_IOC_FLUSHDCACHE:
		case MIOMAP_IOC_GET_BLOCK_OFFSET:
		{
			return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
		}
		default:
			printk("Unknown ioctl command %d\n", cmd);
			return -ENOTTY;
    }
    return 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long _MDrv_MIOMAP_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_MIOMAP_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    int         err= 0;
	int         ret= 0;
   MMIO_FileData *mmioData = filp->private_data ;

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if (MIOMAP_IOC_MAGIC!= _IOC_TYPE(cmd))
    {
        return -ENOTTY;
    }

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (err)
    {
        return -EFAULT;
    }

    // @FIXME: Use a array of function pointer for program readable and code size later
    switch(cmd)
    {
    //------------------------------------------------------------------------------
    // Signal
    //------------------------------------------------------------------------------
    case MIOMAP_IOC_CHIPINFO:
        {
            DrvMIOMap_ChipInfo_t i = { 0, 0 };
            // Chip Type
            // Uranus 0x0101
            // Oberon 0x0102
            // Euclid 0x0103
            // T1     0x0201
            // T2     0x0202
            // T3     0x0203
            // Chip version
            // FPGA   0x0000
            // U01    0x0001
            // U02    0x0002
            // U03    0x0003

            #if defined(CONFIG_MSTAR_OBERON)
                i.u16chiptype    = 0x0102 ;
                i.u16chipversion = 0x0001 ;
                //printk("It's oberon now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_TITANIA2)
                i.u16chiptype    = 0x0202 ;
                i.u16chipversion = 0x0001 ;
                //printk("It's oberon now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_EUCLID)
                i.u16chiptype    = 0x0103 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's oberon now\n"); // test string
            #endif

            #if (defined(CONFIG_MSTAR_TITANIA3)||defined(CONFIG_MSTAR_TITANIA10))
                i.u16chiptype    = 0x0203 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's oberon now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_TITANIA4)
                i.u16chiptype    = 0x0204 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's titania4 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_TITANIA8) || defined(CONFIG_MSTAR_TITANIA12)
                i.u16chiptype    = 0x0208 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's titania8 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_TITANIA9)
                i.u16chiptype    = 0x0209 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's titania9 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_TITANIA11)
                i.u16chiptype    = 0x020B ;
                i.u16chipversion = 0x0000 ;
                //printk("It's titania11 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_TITANIA13)
                i.u16chiptype    = 0x020D ;
                i.u16chipversion = 0x0000 ;
                //printk("It's titania13 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_URANUS4)
                i.u16chiptype    = 0x0104 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's uranus4 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_JANUS2)
                i.u16chiptype    = 0x0302 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's janus2 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_KRONUS)
                i.u16chiptype    = 0x0104 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's kronus now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_KAISERIN)
                i.u16chiptype    = 0x0104 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's kaiserin now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_KENYA)
                i.u16chiptype    = 0x0104 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's kenya now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_AMBER1)
                i.u16chiptype    = 0x0301 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's amber1 now\n"); // test string
            #endif

	    #if defined(CONFIG_MSTAR_AMBER2)
                i.u16chiptype    = 0x0303 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's amber2 now\n"); // test string
            #endif

	        #if defined(CONFIG_MSTAR_AMBER5)
                i.u16chiptype    = 0x0303 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's amber5 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_AMBER6)
                i.u16chiptype    = 0x0400 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's amber6 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_AMBER7)
                i.u16chiptype    = 0x0402 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's amber7 now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_AMETHYST)
                i.u16chiptype    = 0x040D ;
                i.u16chipversion = 0x0000 ;
                //printk("It's amethyst now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_EAGLE)
                i.u16chiptype    = 0x0501 ;
                i.u16chipversion = 0x0000 ;
                //printk("It's eagle now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_EIFFEL)
                i.u16chiptype    = 0x0607;
                i.u16chipversion = 0x0000;
                //printk("It's eiffel now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_NIKE)
                i.u16chiptype    = 0x0704;
                i.u16chipversion = 0x0000;
                //printk("It's nike now\n"); // test string
            #endif

            #if defined(CONFIG_MSTAR_MADISON)
                i.u16chiptype    = 0x050C;
                i.u16chipversion = 0x0000;
                //printk("It's madison now\n"); // test string
            #endif

            ret= copy_to_user( (void *)arg, &i, sizeof(i) ) ;
        }
        break;

    case MIOMAP_IOC_SET_MAP:
        {
            DrvMIOMap_Info_t i ;

            ret= copy_from_user( &i, (DrvMIOMap_Info_t*)arg, sizeof(i) ) ;
            mmioData->miomap_base = i.u32Addr ;
            mmioData->miomap_size = i.u32Size ;
            //printk("MIOMAP_BASE=%x MIOMAP_SIZE=%x\n", MIOMAP_BASE, MIOMAP_SIZE);
        }
        break;

    case MIOMAP_IOC_FLUSHDCACHE:
        {
            dsb(sy);
        }
        break;

    case MIOMAP_IOC_GET_BLOCK_OFFSET:
        {
            DrvMIOMap_Info_t i ;
            ret= copy_from_user( &i, (DrvMIOMap_Info_t*)arg, sizeof(i) ) ;
            // mark temporarily, restore later
            //MDrv_SYS_GetMMAP(i.u32Addr, &(i.u32Addr), &(i.u32Size)) ;
            ret= copy_to_user( (void *)arg, &i, sizeof(i) ) ;
        }
        break ;
    default:
        printk("Unknown ioctl command %d\n", cmd);
        return -ENOTTY;
    }
    return 0;
}

MSYSTEM_STATIC int __init mod_miomap_init(void)
{
    int s32Ret;
    dev_t dev;

    miomap_class = class_create(THIS_MODULE, "miomap");
    if (IS_ERR(miomap_class))
    {
        return PTR_ERR(miomap_class);
    }

    if (MIOMapDev.s32MIOMapMajor)
    {
        dev = MKDEV(MIOMapDev.s32MIOMapMajor, MIOMapDev.s32MIOMapMinor);
        s32Ret = register_chrdev_region(dev, MOD_MIOMAP_DEVICE_COUNT, MOD_MIOMAP_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, MIOMapDev.s32MIOMapMinor, MOD_MIOMAP_DEVICE_COUNT, MOD_MIOMAP_NAME);
        MIOMapDev.s32MIOMapMajor = MAJOR(dev);
    }

    if ( 0 > s32Ret)
    {
        MIOMAP_DPRINTK("Unable to get major %d\n", MIOMapDev.s32MIOMapMajor);
        class_destroy(miomap_class);
        return s32Ret;
    }

    cdev_init(&MIOMapDev.cDevice, &MIOMapDev.MIOMapFop);
    if (0!= (s32Ret= cdev_add(&MIOMapDev.cDevice, dev, MOD_MIOMAP_DEVICE_COUNT)))
    {
        MIOMAP_DPRINTK("Unable add a character device\n");
        unregister_chrdev_region(dev, MOD_MIOMAP_DEVICE_COUNT);
        class_destroy(miomap_class);
        return s32Ret;
    }

    device_create(miomap_class, NULL, dev, NULL, MOD_MIOMAP_NAME);

    return 0;
}

MSYSTEM_STATIC void __exit mod_miomap_exit(void)
{
    cdev_del(&MIOMapDev.cDevice);
    unregister_chrdev_region(MKDEV(MIOMapDev.s32MIOMapMajor, MIOMapDev.s32MIOMapMinor), MOD_MIOMAP_DEVICE_COUNT);
    device_destroy(miomap_class, MKDEV(MIOMapDev.s32MIOMapMajor, MIOMapDev.s32MIOMapMinor));
    class_destroy(miomap_class);
}

#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
#else//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
module_init(mod_miomap_init);
module_exit(mod_miomap_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("MIOMAP driver");
MODULE_LICENSE("GPL");
#endif//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
