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
/// file    mdrv_semutex.c
/// @brief  Memory Pool Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <generated/autoconf.h>
#include <linux/version.h>
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
#include <linux/device.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>
#include <linux/vmalloc.h>
#include <linux/compat.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/sched/mm.h>
#endif



#if defined(CONFIG_MIPS)
#include <asm/mips-boards/prom.h>
#elif defined(CONFIG_ARM)
#endif
#include "mdrv_msos_io.h"
#include "mdrv_semutex_io.h"
#include "mdrv_msos.h"
#include "mdrv_semutex.h"
#include "mst_devid.h"
#include "mdrv_types.h"
#include "mst_platform.h"
#include "mdrv_system.h"
#if !(defined(CONFIG_MSTAR_TITANIA3) || defined(CONFIG_MSTAR_TITANIA10) )
#include "chip_setup.h"
#endif

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
#ifdef SEMUTEX_DEBUG
#define SEMUTEX_DPRINTK(fmt, args...)      printk("[Semutex (Driver)][%05d] " fmt, __LINE__, ## args)
#else
#define SEMUTEX_DPRINTK(fmt, args...)
#endif


//-------------------------------------------------------------------------------------------------
//  Global variables
//-------------------------------------------------------------------------------------------------
static DEFINE_MUTEX(semutex_mutex);
static DEFINE_MUTEX(semutex_semaphore);
static DEFINE_MUTEX(semutex_sharememory);
static DEFINE_MUTEX(semutex_rwlock);
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define MOD_SEMUTEX_DEVICE_COUNT     1
#define MOD_SEMUTEX_NAME             "semutex"

//for sharememory
#define SHAREDMEMORY_DEFAULT_LEN       3*1024*1024

//for mutex
#define MUTEX_NAME_LEN  64
#define MUTEX_SUPPORTED 320

//for read-write lock
#define RWLOCK_NAME_LEN  64
#define RWLOCK_SUPPORTED 320

/* Fix SOPHIA-3810 */
#define MDRV_SEMUTEX_MAX            (32)

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------;

typedef struct
{
    char name[MUTEX_NAME_LEN];
    bool     locked_status;                   //mutex reference count
    struct semaphore *sem;
    pid_t           locked_pid;                 //the pid this mutex locked by
    pid_t           locked_tgid;                 //the tgid this mutex locked by
    struct file *locked_file;
    char procname[MUTEX_NAME_LEN];
    CROSS_THREAD_UNLOCK_FLAG cross_thread_unlock;
}mutex_info;

typedef struct
{
    char                sema_name[SEMA_NAME_LEN];
    int                 sema_number;
    int                 locked_count;
    struct semaphore    *sem;
    pid_t               *locked_pid_list;             //the pid this mutex locked by
    //pid_t               locked_tgid;            //the tgid this mutex locked by
    //struct file         *locked_file;
    //char                procname[64];
}semaphore_info;

typedef struct
{
    char                name[RWLOCK_NAME_LEN];
    struct              rw_semaphore *rw_sem;
    int                 reader;
    int                 writer;
    pid_t               write_lock_pid;
    pid_t               write_lock_tgid;
    struct file         *locked_file;
    char                procname[MUTEX_NAME_LEN];
}rwlock_info;

typedef struct
{
    struct page *               pg;
    struct list_head            list;
} share_page_node;

typedef struct
{
    int                         s32SemutexMajor;
    int                         s32SemutexMinor;

    int                         refcnt_sharemem;            //sharedmemory reference count
    unsigned int                shared_memorysize;
    //struct page **              pages;
    struct list_head            share_page_list;
    int                         length;
    int                         pageCount;
    bool                        isfirst_sharememory;

    mutex_info                  mutex_info[MUTEX_SUPPORTED];
    semaphore_info              sema_info[MUTEX_SUPPORTED];
    rwlock_info                 rwlock_info[RWLOCK_SUPPORTED];

    struct cdev                 cDevice;
    struct file_operations      SemutexFileop;
} SemutexModHandle;


typedef struct
{
    //for sharememory
    struct vm_area_struct   *vma;
    struct task_struct *tsk;
    struct file *createsharememory_filp;

    //for mutex
    //
    SemutexModHandle        *semutex;
} procinfo;

//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int  _MDrv_SEMUTEX_Open (struct inode *inode, struct file *filp);
static int  _MDrv_SEMUTEX_Release(struct inode *inode, struct file *filp);
static int  _MDrv_SEMUTEX_MMap(struct file *filp, struct vm_area_struct *vma);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long _MDrv_SEMUTEX_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int  _MDrv_SEMUTEX_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static int  _MDrv_SEMUTEX_Flush(struct file *filp, fl_owner_t id);

#if defined(CONFIG_COMPAT)
static long Compat_MDrv_SEMUTEX_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

//-------------------------------------------------------------------------------------------------
// Local Variables
//-------------------------------------------------------------------------------------------------

static struct class *semutex_class;

static SemutexModHandle SemutexDev=
{
    .s32SemutexMajor=               MDRV_MAJOR_SEMUTEX,
    .s32SemutexMinor=               MDRV_MINOR_SEMUTEX,
    .refcnt_sharemem               =                 0,
    .shared_memorysize=             SHAREDMEMORY_DEFAULT_LEN,
    //.pages              =                   NULL,
    .cDevice=
    {
        .kobj=                  {.name= MOD_SEMUTEX_NAME, },
        .owner  =               THIS_MODULE,
    },
    .SemutexFileop=
    {
        .open =                   _MDrv_SEMUTEX_Open,
        .release =                _MDrv_SEMUTEX_Release,
        .mmap =                   _MDrv_SEMUTEX_MMap,
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
        .unlocked_ioctl=          _MDrv_SEMUTEX_Ioctl,
        #else
        .ioctl =                  _MDrv_SEMUTEX_Ioctl,
        #endif
      #if defined(CONFIG_COMPAT)
      .compat_ioctl =          Compat_MDrv_SEMUTEX_Ioctl,
      #endif
        .flush =                  _MDrv_SEMUTEX_Flush,
    },
};

static int __init SHM_SIZE_setup(char *str)
{
    //printk("SHM_SIZE = %s\n", str);
    if( str != NULL )
    {
        SemutexDev.shared_memorysize = simple_strtol(str, NULL, 16);

        if(SemutexDev.shared_memorysize < SHAREDMEMORY_DEFAULT_LEN)
        {
            printk(KERN_ERR "SHM_SIZE too small, set as default 2M bytes\n");
            SemutexDev.shared_memorysize = SHAREDMEMORY_DEFAULT_LEN;
        }
    }

    return 0;
}

early_param("SHM_SIZE", SHM_SIZE_setup);
//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static int alloc_sharepage(SemutexModHandle *dev,int page_num)
{
    share_page_node* page_node = NULL;
    struct page * pg = NULL;
    struct list_head *p, *n;
    int i;
    SEMUTEX_DPRINTK("share page:allocated page \n");
    for(i = 0; i < page_num; i++)
    {
        page_node = kzalloc(sizeof(share_page_node), GFP_KERNEL);
        pg = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
        if(pg == NULL || page_node == NULL)
        {
            if(page_node)
            {
                kfree(page_node);
            }
            printk(KERN_ERR "share page:allocated page fail!!! \n");
            goto allocated_fail;
        }
        page_node->pg = pg;
        SEMUTEX_DPRINTK("share page:allocated page pg=%X\n",(unsigned int)pg);
        SEMUTEX_DPRINTK("share page:add node to list i=%d\n",i);

        list_add_tail(&(page_node->list), &(dev->share_page_list));
    }

    return 0;

allocated_fail:

    list_for_each_safe(p, n,&(dev->share_page_list)) {
        page_node = list_entry(p, share_page_node, list);
        pg = page_node->pg;
        if(pg != NULL)
            __free_page(pg);
        list_del(p);
        kfree(page_node);
    }
    return -ENOMEM;
}

static int free_sharemem(SemutexModHandle *dev)
{
    share_page_node* page_node = NULL;
    struct page * pg = NULL;
    struct list_head *p, *n;
    list_for_each_safe(p, n,&(dev->share_page_list)) {
        page_node = list_entry(p, share_page_node, list);
        pg = page_node->pg;
        if(pg != NULL)
            __free_page(pg);
        list_del(p);
        kfree(page_node);
    }
    return 0;
}


struct page * append_onePage(SemutexModHandle *dev)
{
    share_page_node* page_node = NULL;
    struct page * pg = NULL;
    page_node = kzalloc(sizeof(share_page_node), GFP_KERNEL);
    pg = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
    if(pg == NULL || page_node == NULL)
    {
        if(page_node)
        kfree(page_node);
        printk("share page:allocated page fail!!! \n");
        return NULL;
    }
    page_node->pg = pg;
    list_add_tail(&(page_node->list), &(dev->share_page_list));
    return pg;
}

static int expend_sharemem(procinfo * pprocinfo, SemutexModHandle *dev,int page_num)
{
    int i;
    int ret;
    unsigned long user_page_addr,start,end;
    struct page * pg = NULL;
    struct vm_area_struct *vma = NULL;
    struct mm_struct *mm = NULL;

    SEMUTEX_DPRINTK("share memory expend_sharemem!!! \n");
    if( pprocinfo->vma == NULL)
    {
        printk("share memory No MMAPPING!!! \n");
        return -1;
    }
    else
    {
        mm = get_task_mm(pprocinfo->tsk);
        if (mm) {
            down_write(&mm->mmap_sem);
        }
        vma = pprocinfo->vma;
    }

    start = vma->vm_start;
    end   = vma->vm_end;
    if(end < start + ((dev->pageCount + page_num) << PAGE_SHIFT))
    {
        printk("not enough virtual address space!!! \n");
        return -1;
    }

    user_page_addr = start + (dev->pageCount << PAGE_SHIFT);
    for(i = 0; i < page_num; i++)
    {
        pg = append_onePage(dev);
        if(pg == NULL)
            return -ENOMEM;
        SEMUTEX_DPRINTK( "share page:expend_sharemem page=%X, i=%d, addr = %X \n",(unsigned int)pg, i, (unsigned int)user_page_addr);

        ret = vm_insert_page(vma, user_page_addr, pg);
        if (ret) {
            printk(KERN_ERR "vm_insert_page fail. ret1 = %d \n",ret);
            goto err_no_vma;
        }
        else
        {
            user_page_addr+= PAGE_SIZE;
        }

    }

    if (mm) {
        up_write(&mm->mmap_sem);
        mmput(mm);
    }
    return 0;

err_no_vma:
    if (mm) {
        up_write(&mm->mmap_sem);
        mmput(mm);
    }
    return -ENOMEM;
}

#if 0
static int alloc_sharemem(SemutexModHandle *dev,int len)
{
    struct page **page;
    int i,j;
    for(i = 0 ; i < len / PAGE_SIZE; i ++)
    {
        page = &dev->pages[i];
        *page = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
        if(*page == NULL)
        {
            printk("allocated page fail!!! \n");
            goto allocated_fail;
        }
    }

    return 0;

allocated_fail:
    for(j = 0 ; j < i ; j ++)
    {
        page = &dev->pages[i];
        __free_page(*page);
    }
    return -ENOMEM;
}
#endif

void dumpMutexStatus(SemutexModHandle *dev)
{
    int i;
    for(i = 0 ; i < MUTEX_SUPPORTED ; i ++)
    {
        if(i < 40)
            SEMUTEX_DPRINTK(" i=%d, mutex name = %s ,status = %d,lockedpid=%d,locked_tgid=%d,pid=%d,tgid = %d\n",
            i,dev->mutex_info[i].name,dev->mutex_info[i].locked_status,dev->mutex_info[i].locked_pid,dev->mutex_info[i].locked_tgid,current->pid,current->tgid);

        if(strcmp(dev->mutex_info[i].name,"SkiaDecodeMutex") == 0)
            printk(KERN_ERR "~!~  dump!!! i=%d, mutex name = %s ,status = %d,lockedpid=%d,locked_tgid=%d,pid=%d,tgid = %d\n",
            i,dev->mutex_info[i].name,dev->mutex_info[i].locked_status,dev->mutex_info[i].locked_pid,dev->mutex_info[i].locked_tgid,current->pid,current->tgid);

    }
}

void check_pid_alive(SemutexModHandle *dev,int index)
{
    int i = 0;
    struct file *filep = NULL;

    for(i = 0; i < dev->sema_info[index].sema_number ; i++)
    {
        pid_t p = dev->sema_info[index].locked_pid_list[i];
        SEMUTEX_DRV_DEBUG("[semaphore] pid = %d\n",p);
    }

    for(i = 0; i < dev->sema_info[index].sema_number ; i++)
    {
        pid_t p = dev->sema_info[index].locked_pid_list[i];
        if (p != -1)
        {
            char path[20];
            snprintf(path,sizeof(path),"/proc/%d",p);
            filep = filp_open(path,O_RDONLY,0);
            if(IS_ERR(filep))
            {
                dev->sema_info[index].locked_pid_list[i] = -1;
                up(dev->sema_info[index].sem);
            }
            else
            {
                filp_close(filep,0);
            }
        }
    }
}

void set_pid(SemutexModHandle *dev,int index)
{
    int i = 0;
    for(i = 0; i < dev->sema_info[index].sema_number ; i++)
    {
        pid_t p = dev->sema_info[index].locked_pid_list[i];
        if (p == -1)
        {
            dev->sema_info[index].locked_pid_list[i] = current->pid;
            break;
        }
    }

    for(i = 0; i < dev->sema_info[index].sema_number ; i++)
    {
        pid_t p = dev->sema_info[index].locked_pid_list[i];
        SEMUTEX_DRV_DEBUG("[semaphore] pid = %d\n",p);
    }

}

void clear_pid(SemutexModHandle *dev,int index)
{
    int i = 0;
    for(i = 0; i < dev->sema_info[index].sema_number ; i++)
    {
        pid_t p = dev->sema_info[index].locked_pid_list[i];
        if (p == current->pid )
        {
            dev->sema_info[index].locked_pid_list[i] = -1;
            break;
        }
    }

    for(i = 0; i < dev->sema_info[index].sema_number ; i++)
    {
        pid_t p = dev->sema_info[index].locked_pid_list[i];
        SEMUTEX_DRV_DEBUG("[semaphore] pid = %d\n",p);
    }

}

static int _Cleanup_Mutex(struct file *filp,SemutexModHandle *dev)
{
    int i, flag = false;

    for(i = 0 ; i < MUTEX_SUPPORTED ; i ++)
    {
        mutex_lock(&semutex_mutex);
        if(dev->mutex_info[i].locked_tgid == current->tgid)
        {
            dev->mutex_info[i].locked_status = false;
            dev->mutex_info[i].locked_pid = 0;
            dev->mutex_info[i].locked_tgid = 0;
            dev->mutex_info[i].locked_file = NULL;
            memset(dev->mutex_info[i].procname, 0, MUTEX_NAME_LEN);
            dev->mutex_info[i].cross_thread_unlock = E_CROSS_THREAD_UNLOCK_DISABLE;

            flag = true;
        }
        mutex_unlock(&semutex_mutex);

        if (flag == true)
        {
            up(dev->mutex_info[i].sem);
            flag = false;
        }
    }

    return 0;
}

static int  _MDrv_SEMUTEX_Flush(struct file *filp, fl_owner_t id)
{
    procinfo *pprocinfo = filp->private_data;
    SemutexModHandle *dev =  pprocinfo->semutex;

    if(!(current->flags & PF_EXITING))      //do not the exit case,
    {
        //printk(KERN_ERR"~!~do not the exit case, flags = %x, pid=%d,tgid=%d,proc=%s \n",
            //current->flags, current->pid,current->tgid,current->comm);
        return 0;
    }

    //printk(KERN_ERR"~!~current->flags = %x , pid = %d , tgid = %d , proc=%s \n",
        //current->flags, current->pid,current->tgid,current->comm);
    return _Cleanup_Mutex(filp,dev);
}

static int _MDrv_SEMUTEX_Open (struct inode *inode, struct file *filp)
{
    SemutexModHandle *dev;

//!!! must asume that one process only open the device once!
    procinfo *pprocinfo = (procinfo *)kzalloc(sizeof(procinfo),GFP_KERNEL);
    if (!pprocinfo)
    {
        printk(KERN_ERR "~!~[%s][%d] kzalloc fail\n",__FUNCTION__,__LINE__);
        return -1;
    }

    dev = container_of(inode->i_cdev, SemutexModHandle, cDevice);

    pprocinfo->semutex = dev;
    pprocinfo->tsk = current;
    pprocinfo->createsharememory_filp  = NULL;

    filp->private_data = pprocinfo;

    return 0;
}

static int _MDrv_SEMUTEX_Release(struct inode *inode, struct file *filp)
{
    procinfo *pprocinfo = filp->private_data;
    SemutexModHandle *dev =  pprocinfo->semutex;
    int i = 0, flag = false;

    BUG_ON(dev->refcnt_sharemem < 0);
    //if open, don't map , then close, dev->refcnt_sharemem will be 0, it is allowed!
    //But in this case , pprocinfo->createsharememory_filp should be NULL.
    //We bug on the case where dev->refcnt_sharemem == 0 and pprocinfo->createsharememory_filp is not NULL
    //(which means that it has been mapped )
    BUG_ON((dev->refcnt_sharemem == 0) && (pprocinfo->createsharememory_filp));

    if(pprocinfo->createsharememory_filp)   //this means it has create the sharememory before.
    {
        mutex_lock(&semutex_sharememory);
        dev->refcnt_sharemem --;
        SEMUTEX_DPRINTK("[SEMUTEX] in release dev->refcnt_sharemem = %d \n",dev->refcnt_sharemem);

        if(0 == dev->refcnt_sharemem)
        {
            //struct page **page1;
            SEMUTEX_DPRINTK("[SEMUTEX] semutex refcnt_sharemem drops to 0.\n");
            free_sharemem(dev);
#if 0
            for(i = 0 ; i < dev->length / PAGE_SIZE ; i ++)
            {
                page1 = &dev->pages[i];
                __free_page(*page1);
            }

            kfree(dev->pages);
            dev->pages = NULL;
#endif
        }
        pprocinfo->createsharememory_filp = NULL;
        mutex_unlock(&semutex_sharememory);
    }
    //mutex clean up??
    for(i = 0 ; i < MUTEX_SUPPORTED ; i ++)
    {
        mutex_lock(&semutex_mutex);
        if(dev->mutex_info[i].locked_file == filp)
        {
            dev->mutex_info[i].locked_status = false;
            dev->mutex_info[i].locked_file = NULL;
            dev->mutex_info[i].cross_thread_unlock = E_CROSS_THREAD_UNLOCK_DISABLE;
            printk(KERN_EMERG" i=%d, mutex name = %s \n",i,dev->mutex_info[i].name);

            flag = true;
        }
        mutex_unlock(&semutex_mutex);

        if (flag == TRUE)
        {
            up(dev->mutex_info[i].sem);
            flag = false;
        }
    }

    //rwlock clean up
    for(i = 0 ; i < RWLOCK_SUPPORTED ; i ++)
    {
        if(dev->rwlock_info[i].locked_file == filp)
        {
            int current_reader = dev->rwlock_info[i].reader;
            int current_writer = dev->rwlock_info[i].writer;
            while (current_reader--)
            {
                up_read(dev->rwlock_info[i].rw_sem);
            }
            if (current_writer == 1)
            {
                up_write(dev->rwlock_info[i].rw_sem);
            }
            mutex_lock(&semutex_rwlock);
            dev->rwlock_info[i].reader = 0;
            dev->rwlock_info[i].writer = 0;
            dev->rwlock_info[i].locked_file = NULL;
            mutex_unlock(&semutex_rwlock);
            printk(KERN_EMERG" i=%d, rwlock name = %s \n",i,dev->rwlock_info[i].name);
            //break;
        }
    }
    kfree(pprocinfo);

    return 0;
}



static int semutex_update_page_range(procinfo * pprocinfo, int allocate,
                    unsigned long start, unsigned long end,
                    struct vm_area_struct *vma)
{
    unsigned long user_page_addr;
    SemutexModHandle *dev =  pprocinfo->semutex;
    struct page *page;
    struct mm_struct *mm;
    share_page_node *page_node = NULL;
    struct list_head *p;
    //int index = 0;

    if (end <= start)
        return 0;

    if (vma)
        mm = NULL;
    else
        mm = get_task_mm(pprocinfo->tsk);

    if (mm) {
        down_write(&mm->mmap_sem);
        vma = pprocinfo->vma;
    }

    if (allocate == 0)
        goto free_range;

    if (vma == NULL) {
        printk(KERN_ERR "no vma\n");
        goto err_no_vma;
    }

#if 1
    user_page_addr = start;
    list_for_each(p,&(dev->share_page_list)) {
        int ret;

        if(user_page_addr >= end)
        {
            break;
        }
        page_node = list_entry(p, share_page_node, list);
        page = page_node->pg;
        BUG_ON(NULL == page);
        SEMUTEX_DPRINTK("share page:vm_insert_page addr=%lX, %lX, page:%X\n",user_page_addr,end,(unsigned int)page);

        ret = vm_insert_page(vma, user_page_addr, page);
        if (ret) {
            printk(KERN_ERR "vm_insert_page fail. ret1 = %d \n",ret);
            goto err_no_vma;
        }
        else
        {
            user_page_addr += PAGE_SIZE;
        }
    }
#else

    for (user_page_addr = start; user_page_addr < end; user_page_addr += PAGE_SIZE) {
        int ret;
        page = &dev->pages[index++];

        BUG_ON(NULL == *page);

        ret = vm_insert_page(vma, user_page_addr, page[0]);
        if (ret) {
            printk(KERN_ERR "vm_insert_page fail. ret1 = %d \n",ret);
            goto err_no_vma;
        }
        /* vm_insert_page does not seem to increment the refcount */
    }
#endif
    if (mm) {
        up_write(&mm->mmap_sem);
        mmput(mm);
    }
    return 0;

free_range:
    for (user_page_addr = end - PAGE_SIZE; user_page_addr >= start;user_page_addr -= PAGE_SIZE)
    {
        if (vma)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
		zap_page_range(vma, user_page_addr, PAGE_SIZE);
#else
		zap_page_range(vma, user_page_addr, PAGE_SIZE, NULL);
#endif
    }
err_no_vma:
    if (mm) {
        up_write(&mm->mmap_sem);
        mmput(mm);
    }
    return -ENOMEM;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static vm_fault_t semutex_vma_fault(struct vm_fault *vmf)
#else
static int semutex_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#endif
{
    SEMUTEX_DPRINTK("semutex_vma_fault !!! \n");
    BUG();
    return -ENOMEM;
}

static void semutex_vma_close(struct vm_area_struct *vma)
{
    unsigned int len;
    procinfo *pprocinfo = vma->vm_private_data;
    //SemutexModHandle *dev = pprocinfo->semutex;
    len = vma->vm_end - vma->vm_start;

    //unmap it
    semutex_update_page_range(pprocinfo, 0, vma->vm_start, vma->vm_end, vma);
}


static struct vm_operations_struct semutex_vm_ops = {
    //.open = semutex_vma_open,
    .close = semutex_vma_close,
    .fault = semutex_vma_fault,
};

static int  _MDrv_SEMUTEX_MMap(struct file *filp, struct vm_area_struct *vma)
{
    int ret = 0;
    unsigned int len;
    procinfo *pprocinfo = filp->private_data;
    SemutexModHandle *dev = pprocinfo->semutex;

    if ((vma->vm_end - vma->vm_start) > dev->shared_memorysize)
        vma->vm_end = vma->vm_start + dev->shared_memorysize;

    len = vma->vm_end - vma->vm_start;

    vma->vm_ops = &semutex_vm_ops;
    vma->vm_private_data = pprocinfo;

    if (semutex_update_page_range(pprocinfo, 1, vma->vm_start, vma->vm_end,vma))
    {
        ret = -ENOMEM;
        goto err_map_failed;
    }

    pprocinfo->vma = vma;

    //
    //strcpy(pprocinfo->sharedmemroy,"hello world from linux kernel!!!!!!");
    //printk("~!~mmaped pprocinfo->sharedmemroy = %p \n",pprocinfo->sharedmemroy);

err_map_failed:
    return ret;
}

#if defined(CONFIG_COMPAT)

static long Compat_MDrv_SEMUTEX_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    S32 s32Ret = 0;
    compat_uptr_t pPtr;
    /* Event. */
    MDRV_EVENT_PARM stEventParm = {0};                           /* Tmp. */
    MDRV_EVENT_PARM        __user *pstEventParm       = NULL;    /* New. */
    MDRV_EVENT_PARM_COMPAT __user *pstEventParmCompat = NULL;    /* Org. */
    /* Queue. */
    MDRV_QUEUE_PARM stQueueParm = {0};                           /* Tmp. */
    MDRV_QUEUE_PARM        __user *pstQueueParm       = NULL;    /* New. */
    MDRV_QUEUE_PARM_COMPAT __user *pstQueueParmCompat = NULL;    /* Org. */

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    /*
     * Copy from original data.
     * Copy to alloc compat data.
     */
    switch (cmd)
    {
        case MDRV_EVENT_IOC_COMPAT_INIT:
        case MDRV_EVENT_IOC_COMPAT_CREATE:
        case MDRV_EVENT_IOC_COMPAT_DELETE:
        case MDRV_EVENT_IOC_COMPAT_SET:
        case MDRV_EVENT_IOC_COMPAT_CLEAR:
        case MDRV_EVENT_IOC_COMPAT_WAIT:
            pstEventParmCompat = compat_ptr(arg);
            pstEventParm = (MDRV_EVENT_PARM *)compat_alloc_user_space(sizeof(MDRV_EVENT_PARM));
            if (pstEventParm == NULL)
                return -EFAULT;

            get_user(stEventParm.Num, &pstEventParmCompat->Num);
            put_user(stEventParm.Num, &pstEventParm->Num);

            if ((copy_from_user(&stEventParm.Name, &pstEventParmCompat->Name, MDRV_EVENT_BUF_SIZE) != 0) ||
                (copy_to_user(&pstEventParm->Name, &stEventParm.Name, MDRV_EVENT_BUF_SIZE) != 0))
                return -EFAULT;

            get_user(stEventParm.Id, &pstEventParmCompat->Id);
            put_user(stEventParm.Id, &pstEventParm->Id);

            get_user(stEventParm.Flag, &pstEventParmCompat->Flag);
            put_user(stEventParm.Flag, &pstEventParm->Flag);

            get_user(pPtr, &pstEventParmCompat->CmpFlag);
            put_user(compat_ptr(pPtr), &pstEventParm->CmpFlag);

            get_user(stEventParm.Mode, &pstEventParmCompat->Mode);
            put_user(stEventParm.Mode, &pstEventParm->Mode);

            get_user(stEventParm.WaitMs, &pstEventParmCompat->WaitMs);
            put_user(stEventParm.WaitMs, &pstEventParm->WaitMs);
            break;

        case MDRV_QUEUE_IOC_COMPAT_INIT:
        case MDRV_QUEUE_IOC_COMPAT_CREATE:
        case MDRV_QUEUE_IOC_COMPAT_DELETE:
        case MDRV_QUEUE_IOC_COMPAT_SEND:
        case MDRV_QUEUE_IOC_COMPAT_RECV:
        case MDRV_QUEUE_IOC_COMPAT_PEEK:
            pstQueueParmCompat = compat_ptr(arg);
            pstQueueParm = (MDRV_QUEUE_PARM *)compat_alloc_user_space(sizeof(MDRV_QUEUE_PARM));
            if (pstQueueParm == NULL)
                return -EFAULT;

            get_user(stQueueParm.Num, &pstQueueParmCompat->Num);
            put_user(stQueueParm.Num, &pstQueueParm->Num);

            get_user(stQueueParm.Id, &pstQueueParmCompat->Id);
            put_user(stQueueParm.Id, &pstQueueParm->Id);

            get_user(stQueueParm.QueueSize, &pstQueueParmCompat->QueueSize);
            put_user(stQueueParm.QueueSize, &pstQueueParm->QueueSize);

            get_user(stQueueParm.MsgType, &pstQueueParmCompat->MsgType);
            put_user(stQueueParm.MsgType, &pstQueueParm->MsgType);

            get_user(stQueueParm.MsgAlignSize, &pstQueueParmCompat->MsgAlignSize);
            put_user(stQueueParm.MsgAlignSize, &pstQueueParm->MsgAlignSize);

            get_user(stQueueParm.Attr, &pstQueueParmCompat->Attr);
            put_user(stQueueParm.Attr, &pstQueueParm->Attr);

            if ((copy_from_user(&stQueueParm.Name, &pstQueueParmCompat->Name, MDRV_QUEUE_BUF_SIZE) != 0) ||
                (copy_to_user(&pstQueueParm->Name, &stQueueParm.Name, MDRV_QUEUE_BUF_SIZE) != 0))
                return -EFAULT;

            get_user(pPtr, &pstQueueParmCompat->MsgData);
            put_user(compat_ptr(pPtr), &pstQueueParm->MsgData);

            get_user(stQueueParm.MsgSendSize, &pstQueueParmCompat->MsgSendSize);
            put_user(stQueueParm.MsgSendSize, &pstQueueParm->MsgSendSize);

            get_user(pPtr, &pstQueueParmCompat->MsgRecvSize);
            put_user(compat_ptr(pPtr), &pstQueueParm->MsgRecvSize);

            get_user(stQueueParm.WaitMs, &pstQueueParmCompat->WaitMs);
            put_user(stQueueParm.WaitMs, &pstQueueParm->WaitMs);

            break;
    }

    /* Command. */
    switch (cmd)
    {
        /* Event. */
        case MDRV_EVENT_IOC_COMPAT_INIT:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_EVENT_IOC_INIT, (unsigned long)pstEventParm);
            break;

        case MDRV_EVENT_IOC_COMPAT_CREATE:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_EVENT_IOC_CREATE, (unsigned long)pstEventParm);
            break;

        case MDRV_EVENT_IOC_COMPAT_DELETE:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_EVENT_IOC_DELETE, (unsigned long)pstEventParm);
            break;

        case MDRV_EVENT_IOC_COMPAT_SET:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_EVENT_IOC_SET, (unsigned long)pstEventParm);
            break;

        case MDRV_EVENT_IOC_COMPAT_CLEAR:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_EVENT_IOC_CLEAR, (unsigned long)pstEventParm);
            break;

        case MDRV_EVENT_IOC_COMPAT_WAIT:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_EVENT_IOC_WAIT, (unsigned long)pstEventParm);
            break;

        /* Queue. */
        case MDRV_QUEUE_IOC_COMPAT_INIT:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_QUEUE_IOC_INIT, (unsigned long)pstQueueParm);
            break;

        case MDRV_QUEUE_IOC_COMPAT_CREATE:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_QUEUE_IOC_CREATE, (unsigned long)pstQueueParm);
            break;

        case MDRV_QUEUE_IOC_COMPAT_DELETE:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_QUEUE_IOC_DELETE, (unsigned long)pstQueueParm);
            break;

        case MDRV_QUEUE_IOC_COMPAT_SEND:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_QUEUE_IOC_SEND, (unsigned long)pstQueueParm);
            break;

        case MDRV_QUEUE_IOC_COMPAT_RECV:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_QUEUE_IOC_RECV, (unsigned long)pstQueueParm);
            break;

        case MDRV_QUEUE_IOC_COMPAT_PEEK:
            s32Ret = filp->f_op->unlocked_ioctl(filp, MDRV_QUEUE_IOC_PEEK, (unsigned long)pstQueueParm);
            break;

        /* Other. */
        default:
            return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
    }

    return s32Ret;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long _MDrv_SEMUTEX_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_SEMUTEX_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    S32 s32Ret = 0;
    int err= 0,i;
    procinfo *pprocinfo = filp->private_data;
    SemutexModHandle *dev = pprocinfo->semutex;

    /* Event */
    MDRV_EVENT_PARM stEventParm = {0};
    U32 u32CmpFlag = 0;

    /* Queue. */
    MDRV_QUEUE_PARM stQueueParm = {0};
    U8* pu8Data = NULL;
    u32 u32Size = 0;

    /*
    * extract the type and number bitfields, and don't decode
    * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
    */
    if ((SEMUTEX_IOC_MAGIC  != _IOC_TYPE(cmd)) &&
        (MDRV_EVENT_IOC_MAGIC != _IOC_TYPE(cmd)) &&
        (MDRV_QUEUE_IOC_MAGIC != _IOC_TYPE(cmd)))
        return -ENOTTY;

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
        return -EFAULT;

    // @FIXME: Use a array of function pointer for program readable and code size later
    switch(cmd)
    {
        case MDRV_SEMUTEX_CREATE_MUTEX:
            {
                int first_unused = -1,ret_index;
                char namefromuser[MUTEX_NAME_LEN] = {0};

                if(copy_from_user(namefromuser, (void __user *)arg, MUTEX_NAME_LEN-1))
                {
                    printk(KERN_EMERG"create mutex: copy from user error!\n");
                    ret_index = -ENOENT;
                    goto return_index0;
                }

                /* Fix SOPHIA-3823 */
                namefromuser[MUTEX_NAME_LEN - 1] = '\0';
                if (strlen(namefromuser) == 0)
                {
                    printk(KERN_EMERG "string name is empty.\n");
                    ret_index = -ENOENT;
                    goto return_index0;
                }

                mutex_lock(&semutex_mutex);
                for(i = 0 ; i < MUTEX_SUPPORTED; i ++)
                {
                    //find by name

                    if(strncmp(namefromuser,dev->mutex_info[i].name, MUTEX_NAME_LEN-1) == 0)
                    {
                        ret_index = MUTEX_INDEX_BEGIN + i;
                        goto return_index;
                    }

                    //if the slot not used (we assue that no mutex named with ZERO length)
                    if(first_unused == -1 && strlen(dev->mutex_info[i].name) == 0)
                        first_unused = i;
                }

                if(first_unused == -1)
                {
                    printk("[SEMUTEX] not enough mutex slot... \n");
                    ret_index = -ENOENT;
                    goto return_index;
                }
                memset(dev->mutex_info[first_unused].name, '\0', MUTEX_NAME_LEN);
                strncpy(dev->mutex_info[first_unused].name,namefromuser,MUTEX_NAME_LEN-1);
                if(dev->mutex_info[first_unused].sem == NULL)
                {
                    dev->mutex_info[first_unused].sem = kmalloc(sizeof(struct semaphore),GFP_KERNEL);
                    if(dev->mutex_info[first_unused].sem == NULL)
                    {
                        printk("[SEMUTEX] not enough memory for MDRV_SEMUTEX_CREATE_MUTEX!\n");
                        ret_index = -ENOMEM;
                        goto return_index;
                    }
                    sema_init(dev->mutex_info[first_unused].sem,1);
                }
                ret_index = MUTEX_INDEX_BEGIN + first_unused;
return_index:
                mutex_unlock(&semutex_mutex);
return_index0:
                if(ret_index > 0)
                    SEMUTEX_DPRINTK("[SEMUTEX] i = %d, create mutex name =%s, ret index = %d \n",i,dev->mutex_info[ret_index - MUTEX_INDEX_BEGIN].name,ret_index);
                return ret_index;
            }
            break;
        case MDRV_SEMUTEX_LOCK_WITHTIMEOUT:
            {
                LOCKARG lockarg;
                int res;
                //printk("@@@@@@@@@@ semutex ioctl cmd MDRV_SEMUTEX_LOCK_WITHTIMEOUT = %d \n",MDRV_SEMUTEX_LOCK_WITHTIMEOUT);
                if (copy_from_user(&lockarg, (void __user *)arg, sizeof(LOCKARG)))
                {
                    printk("[SEMUTEX] get  lock timeout  parameter failed! \n");
                    return -1;
                }

                lockarg.index -= (int)MUTEX_INDEX_BEGIN;
                /* Fix SOPHIA-3819 */
                if ((lockarg.index < 0) || (lockarg.index >= MUTEX_SUPPORTED))
                    return -1;

                SEMUTEX_DPRINTK("[SEMUTEX] index %d name %s  lock with timeout \n",
                                lockarg.index, dev->mutex_info[lockarg.index].name);

                /* Fix SOPHIA-3830 */
                if (dev->mutex_info[lockarg.index].sem == NULL)
                {
                    printk("[SEMUTEX] sem is NULL.\n");
                    return -1;
                }

                res = down_timeout(dev->mutex_info[lockarg.index].sem, msecs_to_jiffies(lockarg.time));
                if(res == -ETIME)
                {
                    printk("[SEMUTEX] mutex %s timeout with %d ms \n",dev->mutex_info[lockarg.index].name,lockarg.time);
                }
                else if(res == 0)
                {
                    mutex_lock(&semutex_mutex);
                    dev->mutex_info[lockarg.index].locked_status = true;
                    dev->mutex_info[lockarg.index].locked_pid = current->pid;
                    dev->mutex_info[lockarg.index].locked_tgid = current->tgid;
                    dev->mutex_info[lockarg.index].locked_file = filp;
                    memset(dev->mutex_info[lockarg.index].procname, 0, MUTEX_NAME_LEN);
                    strncpy(dev->mutex_info[lockarg.index].procname, current->comm, MUTEX_NAME_LEN-1);

                    mutex_unlock(&semutex_mutex);
                }
                else
                    printk("unknown error %d \n",res);
                SEMUTEX_DPRINTK("[SEMUTEX] lock with timeout  return with res %d \n",res);
                return res;
            }
            break;
        case MDRV_SEMUTEX_TRY_LOCK:
            {
                int index,flag;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)MUTEX_INDEX_BEGIN;

                /* Fix SOPHIA-3830 */
                if ((index < 0) || (index >= MUTEX_SUPPORTED) ||
                    (dev->mutex_info[index].sem == NULL))
                    return -1;

                flag = down_trylock(dev->mutex_info[index].sem);
                //~!~ may have problem here!!!!

                SEMUTEX_DPRINTK("[SEMUTEX] trylock with index =%d, name = %s \n",index,dev->mutex_info[index].name);
                if(flag == 0)
                {//already locked!!!
                    mutex_lock(&semutex_mutex);
                    dev->mutex_info[index].locked_status = true;
                    dev->mutex_info[index].locked_pid = current->pid;
                    dev->mutex_info[index].locked_tgid = current->tgid;
                    dev->mutex_info[index].locked_file = filp;
                    memset(dev->mutex_info[index].procname, 0, MUTEX_NAME_LEN);
                    strncpy(dev->mutex_info[index].procname, current->comm, MUTEX_NAME_LEN-1);

                    mutex_unlock(&semutex_mutex);
                }
                SEMUTEX_DPRINTK("[SEMUTEX] trylock return with res %d \n",flag);
                return flag;
            }
            break;
        case MDRV_SEMUTEX_LOCK:
            {
                int index,ret;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)MUTEX_INDEX_BEGIN;

                /* Fix SOPHIA-3830 */
                if ((index < 0) || (index >= MUTEX_SUPPORTED) ||
                    (dev->mutex_info[index].sem == NULL))
                    return -1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,10)
                //ret = -ETIME;
                //ret = down_timeout_interruptible(dev->mutex_info[index].sem, msecs_to_jiffies(1000 * 30));
                while(( ret = down_timeout_interruptible(dev->mutex_info[index].sem, msecs_to_jiffies(1000 * 10))) == -ETIME)
                {
                    printk(KERN_EMERG"~!~ mutex %s timeout ,index = %d,locked pid=%d,tgid=%d,procname =%s, current->pid=%d,tgid=%d,procname=%s \n",
                            dev->mutex_info[index].name,index,
                            dev->mutex_info[index].locked_pid, dev->mutex_info[index].locked_tgid, dev->mutex_info[index].procname,
                            current->pid, current->tgid, current->comm);

                    //dumpMutexStatus(dev);

                //    msleep(30*1000);
                };
                if(ret)
                    return ret;
#else
                ret = down_interruptible(dev->mutex_info[index].sem);
                if(ret == -EINTR)
                    return -EINTR;
#endif
                mutex_lock(&semutex_mutex);
                dev->mutex_info[index].locked_status = true;
                dev->mutex_info[index].locked_pid = current->pid;
                dev->mutex_info[index].locked_tgid = current->tgid;
                dev->mutex_info[index].locked_file = filp;
                memset(dev->mutex_info[index].procname, 0, MUTEX_NAME_LEN);
                strncpy(dev->mutex_info[index].procname, current->comm, MUTEX_NAME_LEN-1);
                mutex_unlock(&semutex_mutex);

                return 0;
            }
            break;
        case MDRV_SEMUTEX_UNLOCK:
            {
                int index;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;
                //printk(KERN_EMERG"~!~! unlock index %d \n",index);

                index -= (int)MUTEX_INDEX_BEGIN;

                /* Fix SOPHIA-3830 */
                if ((index < 0) || (index >= MUTEX_SUPPORTED) ||
                    (dev->mutex_info[index].sem == NULL))
                    return -1;

                mutex_lock(&semutex_mutex);
                if(dev->mutex_info[index].locked_status == false)
                {
                    printk(KERN_WARNING "WARNING1!Try to unlock mutex index %d name %s, but it is not in lock status!!! \n",index,dev->mutex_info[index].name);
                    mutex_unlock(&semutex_mutex);
                    return -1;
                }
                if((E_CROSS_THREAD_UNLOCK_DISABLE == dev->mutex_info[index].cross_thread_unlock)&&(dev->mutex_info[index].locked_pid != current->pid))
                {
                    printk(KERN_WARNING "WARNING2!Try to unlock mutex index %d name %s with pid %d->%s , but not in the lock pid %d->%s\n",
                    index,dev->mutex_info[index].name,current->pid,current->comm,dev->mutex_info[index].locked_pid,dev->mutex_info[index].procname);
            printk(KERN_WARNING "WARNING2!Try to unlock mutex index %d name %s with pid %d->%s , but not in the lock pid %d->%s\n",
                    index,dev->mutex_info[index].name,current->pid,current->comm,dev->mutex_info[index].locked_pid,dev->mutex_info[index].procname);
            printk(KERN_WARNING "WARNING2!Try to unlock mutex index %d name %s with pid %d->%s , but not in the lock pid %d->%s\n",
                    index,dev->mutex_info[index].name,current->pid,current->comm,dev->mutex_info[index].locked_pid,dev->mutex_info[index].procname);
            printk(KERN_WARNING "WARNING2!Try to unlock mutex index %d name %s with pid %d->%s , but not in the lock pid %d->%s\n",
                    index,dev->mutex_info[index].name,current->pid,current->comm,dev->mutex_info[index].locked_pid,dev->mutex_info[index].procname);
            printk(KERN_WARNING "WARNING2!Try to unlock mutex index %d name %s with pid %d->%s , but not in the lock pid %d->%s\n",
                    index,dev->mutex_info[index].name,current->pid,current->comm,dev->mutex_info[index].locked_pid,dev->mutex_info[index].procname);
                    mutex_unlock(&semutex_mutex);
                    /*
                     * SOPHIA-3837:
                     * This feature avoid deadlock when thread dead.
                     */
                    up(dev->mutex_info[index].sem);
                    return 0;
                }
                if(dev->mutex_info[index].locked_file != filp)
                {
                    printk(KERN_WARNING "WARNING!Try to unlock mutex index %d name %s, but use wrong filp!!! \n",index,dev->mutex_info[index].name);
                    mutex_unlock(&semutex_mutex);
                    return -1;
                }

                dev->mutex_info[index].locked_file = NULL;
                dev->mutex_info[index].locked_status = false;
                dev->mutex_info[index].locked_pid = 0;
                dev->mutex_info[index].locked_tgid = 0;
                memset(dev->mutex_info[index].procname, 0, MUTEX_NAME_LEN);

                mutex_unlock(&semutex_mutex);

                up(dev->mutex_info[index].sem);

                return 0;
            }
            break;
        case MDRV_SEMUTEX_SET_CROSS_THREAD_UNLOCK:
            {
                CROSS_THREAD_UNLOCK_INFO unlock_info = {0};

                if (copy_from_user(&unlock_info, (void __user *) arg, sizeof(CROSS_THREAD_UNLOCK_INFO)))
                {
                    return -EFAULT;
                }

                unlock_info.index -= (int)MUTEX_INDEX_BEGIN;

                /* Fix SOPHIA-3830 */
                if ((unlock_info.index < 0) || (unlock_info.index >= MUTEX_SUPPORTED) ||
                    (dev->mutex_info[unlock_info.index].sem == NULL))
                    return -1;

                mutex_lock(&semutex_mutex);
                dev->mutex_info[unlock_info.index].cross_thread_unlock = unlock_info.flag;
                mutex_unlock(&semutex_mutex);
            }
            break;
        case MDRV_SEMUTEX_DEL_MUTEX:
            {
                int index;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)MUTEX_INDEX_BEGIN;

                /* Fix SOPHIA-3830 */
                if ((index < 0) || (index >= MUTEX_SUPPORTED) ||
                    (dev->mutex_info[index].sem == NULL))
                    return -1;

                memset(dev->mutex_info[index].name,0,strlen(dev->mutex_info[index].name));
                /* Fix SOPHIA-3835 */
                sema_init(dev->mutex_info[index].sem, 1);
            }
            break;
        case MDRV_SEMUTEX_CREATE_SEMAPHORE:
            {
                CREATE_SEMA_ARG semaarg;
                int first_unused = -1, ret_index;

                if (copy_from_user(&semaarg, (void __user *)arg, sizeof(CREATE_SEMA_ARG)))
                {
                    printk("[SEMUTEX] ERROR, get create semaphore parameter failed!\n");
                    return -EINVAL;
                }

                semaarg.semaname[SEMA_NAME_LEN-1] = '\0';
                //check parameter
                if (strlen(semaarg.semaname) == 0)
                {
                    printk("[SEMUTEX] ERROR, create semaphore name is empty!\n");
                    return -EINVAL;
                }

                /* Fix SOPHIA-3810 */
                if ((semaarg.semanum <= 0) || (semaarg.semanum > MDRV_SEMUTEX_MAX))
                {
                    printk("[SEMUTEX] ERROR, create semaphore number error :%d!\n", semaarg.semanum);
                    return -EINVAL;
                }

                mutex_lock(&semutex_semaphore);

                for(i = 0 ; i < MUTEX_SUPPORTED; i ++)
                {
                    //find by name
                    if(strncmp(semaarg.semaname, dev->sema_info[i].sema_name, SEMA_NAME_LEN-1) == 0)
                    {
                        ret_index = i;
                        printk("[SEMUTEX] WARNING, create semaphore name had already exist!\n");
                        goto create_semaphore_end;
                    }
                    //if the slot not used (we assume that no mutex named with ZERO length)
                    if(first_unused == -1 && strlen(dev->sema_info[i].sema_name) == 0)
                    {
                        first_unused = i;
                        break;
                    }
                }

                if(first_unused == -1)
                {
                    printk("[SEMUTEX] ERROR, not enough semaphore slot...\n");
                    ret_index = -ENOENT;
                    goto create_semaphore_end;
                }

                strncpy(dev->sema_info[first_unused].sema_name, semaarg.semaname, SEMA_NAME_LEN-1);

                if(dev->sema_info[first_unused].sem == NULL)
                {
                    dev->sema_info[first_unused].sem = kmalloc(sizeof(struct semaphore),GFP_KERNEL);
                    if(dev->sema_info[first_unused].sem == NULL)
                    {
                        printk("[SEMUTEX] ERROR, not enough memory for MDRV_SEMUTEX_CREATE_SEMAPHORE!\n");
                        ret_index = -ENOMEM;
                        goto create_semaphore_end;
                    }
                    sema_init(dev->sema_info[first_unused].sem, semaarg.semanum);
                    dev->sema_info[first_unused].sema_number = semaarg.semanum;
                    dev->sema_info[first_unused].locked_count = 0;
                    dev->sema_info[first_unused].locked_pid_list = kmalloc(sizeof(pid_t)*semaarg.semanum,GFP_KERNEL);
                    /* Fix SOPHIA-3810. */
                    if (dev->sema_info[first_unused].locked_pid_list == NULL)
                    {
                        printk("[SEMUTEX] ERROR, not enough memory for MDRV_SEMUTEX_CREATE_SEMAPHORE!\n");
                        ret_index = -ENOMEM;
                        goto create_semaphore_end;
                    }

                    for(i = 0; i < semaarg.semanum ; i++)
                    {
                        dev->sema_info[first_unused].locked_pid_list[i] = -1;
                    }
                    ret_index = first_unused;
                }
                else
                {
                    ret_index = -EEXIST;
                    printk("[SEMUTEX] ERROR, semaphore create fail: Non-used slot sema is not NULL!\n");
                }

create_semaphore_end:
                mutex_unlock(&semutex_semaphore);
                if(ret_index >= 0)
                {
                    SEMUTEX_DPRINTK("[SEMUTEX] i = %d, create mutex name = %s, total sema = %d, ret index = %d.\n",
                        first_unused, dev->sema_info[ret_index].sema_name, dev->sema_info[ret_index].sema_number, ret_index+SEMAPHORE_INDEX_BEGIN);
                    ret_index += SEMAPHORE_INDEX_BEGIN;
                }
                //printk("[SEMUTEX] create id %d, name %s, num:%d.\n", ret_index, semaarg.semaname, semaarg.semanum);
                return ret_index;
            }
            break;
        case MDRV_SEMUTEX_SEMA_LOCK:
            {
                int index, ret;
                err = get_user(index, (int __user *)arg);
                //printk("[SEMUTEX] LOCK index %d.\n", index);
                if (err)
                    return -EINVAL;
                index -= (int)SEMAPHORE_INDEX_BEGIN;
                if(index < 0){printk("[SEMUTEX] LOCK semaphore index error:%d.\n", index);return -EINVAL;}
                if(index >= MUTEX_SUPPORTED){printk("[SEMUTEX] LOCK semaphore index error:%d.\n", index);return -EINVAL;}
                if(dev->sema_info[index].sem == NULL){printk("[SEMUTEX] LOCK semaphore [%d] is NULL.\n", index);return -EINVAL;}

                SEMUTEX_DPRINTK("[SEMUTEX] Semaphore lock index %d, name = %s, lock count %d, total sema %d.\n",
                    index, dev->sema_info[index].sema_name, dev->sema_info[index].locked_count, dev->sema_info[index].sema_number);

                mutex_lock(&semutex_semaphore);
                check_pid_alive(dev,index);
                mutex_unlock(&semutex_semaphore);

#if 0
                mutex_lock(&semutex_semaphore);
                dev->sema_info[index].locked_count++;
                mutex_unlock(&semutex_semaphore);
#endif
                while(( ret = down_timeout_interruptible(dev->sema_info[index].sem, msecs_to_jiffies(1000 * 10))) == -ETIME)
                {
                    mutex_lock(&semutex_semaphore);
                    check_pid_alive(dev,index);
                    mutex_unlock(&semutex_semaphore);

                    printk(KERN_EMERG"~!~ semaphore %s timeout ,index = %d ~!~\n", dev->sema_info[index].sema_name, index);
                };

                if(ret) //fali case
                {
#if 0
                    mutex_lock(&semutex_semaphore);
                    dev->sema_info[index].locked_count--;
                    mutex_unlock(&semutex_semaphore);
#endif
                    return ret;
                }

                mutex_lock(&semutex_semaphore);
                set_pid(dev,index);
                mutex_unlock(&semutex_semaphore);

                return 0; //get lock successful
            }
            break;
        case MDRV_SEMUTEX_SEMA_TRY_LOCK:
            {
                int index, ret;
                err = get_user(index, (int __user *)arg);
                //printk("[SEMUTEX] TRY LOCK index %d.\n", index);
                if (err)
                    return -EINVAL;
                index -= (int)SEMAPHORE_INDEX_BEGIN;
                if(index < 0){printk("[SEMUTEX] TRY_LOCK semaphore index error:%d.\n", index);return -EINVAL;}
                if(index >= MUTEX_SUPPORTED){printk("[SEMUTEX] TRY_LOCK semaphore index error:%d.\n", index);return -EINVAL;}
                if(dev->sema_info[index].sem == NULL){printk("[SEMUTEX] TRY_LOCK semaphore [%d] is NULL.\n", index);return -EINVAL;}

                SEMUTEX_DPRINTK("[SEMUTEX] Semaphore trylock index %d, name = %s, lock count %d, total sema %d.\n",
                    index, dev->sema_info[index].sema_name, dev->sema_info[index].locked_count, dev->sema_info[index].sema_number);

                mutex_lock(&semutex_semaphore);
                check_pid_alive(dev,index);
                mutex_unlock(&semutex_semaphore);

                ret = down_trylock(dev->sema_info[index].sem);

                /* Fix SOPHIA-3825 */
                if (ret == 0)
                {
                    mutex_lock(&semutex_semaphore);
                    set_pid(dev,index);
                    mutex_unlock(&semutex_semaphore);
                }

                SEMUTEX_DPRINTK("[SEMUTEX] Semaphore trylock return with ret %d \n", ret);
                return ret;
            }
            break;
        case MDRV_SEMUTEX_SEMA_UNLOCK:
            {
                int index;
                err = get_user(index, (int __user *)arg);
                //printk("[SEMUTEX] UNLOCK index %d.\n", index);
                if (err)
                    return -EINVAL;
                index -= (int)SEMAPHORE_INDEX_BEGIN;
                if(index < 0){printk("[SEMUTEX] UNLOCK semaphore index error:%d.\n", index);return -EINVAL;}
                if(index >= MUTEX_SUPPORTED){printk("[SEMUTEX] UNLOCK semaphore index error:%d.\n", index);return -EINVAL;}
                if(dev->sema_info[index].sem == NULL){printk("[SEMUTEX] UNLOCK semaphore [%d] is NULL.\n", index);return -EINVAL;}

#if 0
                if(dev->sema_info[index].locked_count <= 0)
                {
                    printk(KERN_EMERG"WARNING1!Try to unlock semaphore index %d name %s, but it is not in lock status!!! \n",
                        index, dev->sema_info[index].sema_name);
                    return -1;
                }

                mutex_lock(&semutex_semaphore);
                dev->sema_info[index].locked_count--;
                mutex_unlock(&semutex_semaphore);
#endif
                up(dev->sema_info[index].sem);

                mutex_lock(&semutex_semaphore);
                clear_pid(dev,index);
                mutex_unlock(&semutex_semaphore);

                return 0;
            }
            break;
        case MDRV_SEMUTEX_SEMA_RESET:
            {
                int index;
                err = get_user(index, (int __user *)arg);
                //printk("[SEMUTEX] RESET index %d.\n", index);
                if (err)
                    return -EINVAL;
                index -= (int)SEMAPHORE_INDEX_BEGIN;
                if(index < 0){printk("[SEMUTEX] RESET semaphore index error:%d.\n", index);return -EINVAL;}
                if(index >= MUTEX_SUPPORTED){printk("[SEMUTEX] RESET semaphore index error:%d.\n", index);return -EINVAL;}
                if(dev->sema_info[index].sem == NULL){printk("[SEMUTEX] RESET semaphore [%d] is NULL.\n", index);return -EINVAL;}

                printk("[SEMUTEX] It does not support reset semaphore yet!\n");
#if 0
                mutex_lock(&semutex_semaphore);

                if(dev->sema_info[index].locked_count > 0)
                {
                    printk("[SEMUTEX] WARNING WARNING! %s semaphore was still used!! Reset will clean them all!!!\n", dev->sema_info[index].sema_name);

                    //release all semaphore.
                    for(;dev->sema_info[index].locked_count > 0; dev->sema_info[index].locked_count--)
                        up(dev->sema_info[index].sem);
                }

                for(i = 0;i < dev->sema_info[index].sema_number;i++)
                {
                    dev->sema_info[index].locked_pid_list[i] = -1;
                }

                mutex_unlock(&semutex_semaphore);
#endif
            }
            break;
        case MDRV_SEMUTEX_QUERY_ISFIRST_SHAREMEMORY:
            {
                return dev->isfirst_sharememory;
            }
            break;
        case MDRV_SEMUTEX_CREATE_SHAREMEMORY:
            {
                int len,err;
                int page_num;
                SEMUTEX_DPRINTK( " Try to create sharememory !!!! \n");
                if(filp == pprocinfo->createsharememory_filp)
                {
                    printk("WARNING! Try to create sharememory again! \n");
                    return -1;
                }
                err = get_user(len, (int __user *)arg);
                if (err)
                    return -1;

                SEMUTEX_DPRINTK("create sharememory with len %d \n",len);
                if(len > dev->shared_memorysize)
                {
                    printk("the required length %d exceed the avaiable len %u\n", len, dev->shared_memorysize);
                    return -1;
                }

                /* Fix SOPHIA-3839 */
                if (len <= 0)
                {
                    printk("the required length %d not belong natural number.\n", len);
                    return -1;
                }

                SEMUTEX_DPRINTK( " Try to create sharememory len=%d !!!! \n",len);
                len += PAGE_SIZE - 1;
                page_num = (len>> PAGE_SHIFT);
                len = (len>> PAGE_SHIFT)<< PAGE_SHIFT; // make it 4KBytes alignment
                //
                mutex_lock(&semutex_sharememory);

           //     printk("[SEMUTEX] in create dev->refcnt_sharemem = %d \n",dev->refcnt_sharemem);

                if(dev->refcnt_sharemem == 0)
                {
                    SEMUTEX_DPRINTK("~!~!~ re-allocate memory!!! \n");
#if 0
                    dev->pages = kzalloc(sizeof(dev->pages[0]) * (len / PAGE_SIZE), GFP_KERNEL);
                    if(dev->pages == NULL)
                    {
                        printk("not enough memory for pages[] \n");
                        mutex_unlock(&semutex_sharememory);
                        return -1;
                    }
                    err = alloc_sharemem(dev,len);
#endif
                    SEMUTEX_DPRINTK("DEBUG! Try to create sharememory pn=%d !!!! \n",page_num);
                    err = alloc_sharepage(dev, page_num);
                    if(err)
                    {
                        mutex_unlock(&semutex_sharememory);
                    //    printk("[SEMUTEX] alloc fail!!!!! dev->refcnt_sharemem = %d \n",dev->refcnt_sharemem);
                        return -1;
                    }
                    dev->length = len;
                    dev->pageCount = page_num;
                    dev->isfirst_sharememory = true;
                }
                else
                    dev->isfirst_sharememory = false;
                dev->refcnt_sharemem ++;
               // printk("[SEMUTEX] dev->refcnt_sharemem = %d \n",dev->refcnt_sharemem);

                pprocinfo->createsharememory_filp = filp;

                mutex_unlock(&semutex_sharememory);

                return 0;
            }
            break ;
        case MDRV_SEMUTEX_EXPAND_SHAREMEMORY:
            {
                int len,err;
                int page_num;
                int expand_num;
                err = get_user(len, (int __user *)arg);
                if (err)
                    return -1;

                SEMUTEX_DPRINTK("expand sharememory with len %d \n",len);
                if(len > dev->shared_memorysize)
                {
                    printk("the required length %d exceed the avaiable len %u\n", len, dev->shared_memorysize);
                    return -1;
                }

                if( len <= dev->length)
                {
                    printk("the required length %d shorter the current len   %d\n",len,dev->length);
                    return -1;
                }

                /* Fix SOPHIA-3839 */
                if (len <= 0)
                {
                    printk("the required length %d not belong natural number.\n", len);
                    return -1;
                }

                SEMUTEX_DPRINTK( "DEBUG! Try to expand sharememory len=%d !!!! \n",len);
                len += PAGE_SIZE - 1;
                page_num = (len>> PAGE_SHIFT);
                len = (len>> PAGE_SHIFT)<< PAGE_SHIFT; // make it 4KBytes alignment
                //
                mutex_lock(&semutex_sharememory);

                //printk("~!~!in create dev->refcnt_sharemem = %d \n",dev->refcnt_sharemem);

                if(dev->refcnt_sharemem == 0)
                {
                    SEMUTEX_DPRINTK("~!~!~ uninitialized share memory!!! \n");
                    mutex_unlock(&semutex_sharememory);
                    return -1;
                }
                expand_num = page_num - dev->pageCount;
                SEMUTEX_DPRINTK( "DEBUG! Try to expand sharememory pn=%d !!!! \n",expand_num);
                err = expend_sharemem(pprocinfo, dev, expand_num);
                if(err)
                {
                    mutex_unlock(&semutex_sharememory);
                    return -1;
                }
                 dev->length = len;
                 dev->pageCount = page_num;
                //printk("~!~! dev->refcnt_sharemem = %d \n",dev->refcnt_sharemem);

                //pprocinfo->createsharememory_filp = filp;

                mutex_unlock(&semutex_sharememory);

                return 0;
            }
            break;
        case MDRV_SEMUTEX_GET_SHMSIZE:
            {
                if(copy_to_user( (void __user *)arg, &(dev->shared_memorysize), _IOC_SIZE(cmd)))
                    return -EFAULT;
            }
            break;
        case MDRV_SEMUTEX_CREATE_RWLOCK:
            {
                int first_unused = -1,ret_index;
                char namefromuser[RWLOCK_NAME_LEN] = {0};

                if(copy_from_user(namefromuser, (void __user *)arg, RWLOCK_NAME_LEN-1))
                {
                    printk(KERN_EMERG"create rwlock: copy from user error!\n");
                    ret_index = -ENOENT;
                    goto return_rwlock_index0;
                }

                /* Fix SOPHIA-3823 */
                namefromuser[RWLOCK_NAME_LEN - 1] = '\0';
                if (strlen(namefromuser) == 0)
                {
                    printk(KERN_EMERG "string name is empty.\n");
                    ret_index = -ENOENT;
                    goto return_rwlock_index0;
                }

                mutex_lock(&semutex_rwlock);
                for(i = 0 ; i < RWLOCK_SUPPORTED; i ++)
                {
                    if(strncmp(namefromuser,dev->rwlock_info[i].name, RWLOCK_NAME_LEN-1) == 0) //already create
                    {
                        ret_index = RWLOCK_INDEX_BEGIN + i;
                        goto return_rwlock_index;
                    }

                    if(first_unused == -1 && strlen(dev->rwlock_info[i].name) == 0) //find first non-used
                        first_unused = i;
                }


                if(first_unused == -1)
                {
                    printk("[SEMUTEX] not enough rwlock slot... \n");
                    ret_index = -ENOENT;
                    goto return_rwlock_index;
                }
                memset(dev->rwlock_info[first_unused].name, '\0', RWLOCK_NAME_LEN);
                strncpy(dev->rwlock_info[first_unused].name,namefromuser,RWLOCK_NAME_LEN-1);
                if(dev->rwlock_info[first_unused].rw_sem == NULL)
                {
                    dev->rwlock_info[first_unused].rw_sem = kmalloc(sizeof(struct rw_semaphore),GFP_KERNEL);
                    if(dev->rwlock_info[first_unused].rw_sem == NULL)
                    {
                        printk("[SEMUTEX] not enough memory for MDRV_SEMUTEX_CREATE_RWLOCK!\n");
                        ret_index = -ENOMEM;
                        goto return_rwlock_index;
                    }
                    init_rwsem(dev->rwlock_info[first_unused].rw_sem); //init semaphore
                    dev->rwlock_info[first_unused].reader = 0;
                    dev->rwlock_info[first_unused].writer = 0;
                }
                ret_index = RWLOCK_INDEX_BEGIN + first_unused;
return_rwlock_index:
                mutex_unlock(&semutex_rwlock);
return_rwlock_index0:
                if(ret_index > 0)
                    SEMUTEX_DPRINTK("[SEMUTEX] i = %d, create rwlock name =%s, ret index = %d \n",i,dev->rwlock_info[ret_index - MUTEX_INDEX_BEGIN].name,ret_index);
                return ret_index;
            }
            break;
        case MDRV_SEMUTEX_DEL_RWLOCK:
            {
                // 0:succ
                int index;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                kfree(dev->rwlock_info[index].rw_sem);
                dev->rwlock_info[index].rw_sem = NULL;
                memset(dev->rwlock_info[index].name,0,strlen(dev->rwlock_info[index].name));
                return 0;
            }
            break;
        case MDRV_SEMUTEX_RDLOCK_RWLOCK:  // get read lock (block)
            {
                // 1: succ
                int index;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                down_read(dev->rwlock_info[index].rw_sem);

                mutex_lock(&semutex_rwlock);  // criticla start
                dev->rwlock_info[index].reader += 1;
                dev->rwlock_info[index].locked_file = filp;
                memset(dev->rwlock_info[index].procname, 0, MUTEX_NAME_LEN);
                strncpy(dev->rwlock_info[index].procname, current->comm, MUTEX_NAME_LEN-1);
                mutex_unlock(&semutex_rwlock);  // criticla end

                return 1;
            }
            break;
        case MDRV_SEMUTEX_TIMEOUT_RDLOCK_RWLOCK: // get read lock (timeout)
            {
                // 1: succ
                LOCKARG lockarg;
                int res = 0;
                unsigned long interval;
                struct timeval begin;
                struct timeval end;
                if (copy_from_user(&lockarg, (void __user *)arg, sizeof(LOCKARG)))
                {
                    printk("[SEMUTEX] get read lock timeout parameter failed! \n");
                    return -1;
                }
                // lockarg.time: ms
                lockarg.index -= (int)MUTEX_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((lockarg.index < 0) || (lockarg.index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[lockarg.index].rw_sem == NULL))
                    return -1;

                do_gettimeofday(&begin);
                do
                {
                    usleep_range(10000,10000); // sleep 10ms
                    res = down_read_trylock(dev->rwlock_info[lockarg.index].rw_sem);
                    if (res == 1) // get lock
                    {
                        mutex_lock(&semutex_rwlock);  // criticla start
                        dev->rwlock_info[lockarg.index].reader += 1;
                        dev->rwlock_info[lockarg.index].locked_file = filp;
                        memset(dev->rwlock_info[lockarg.index].procname, 0, MUTEX_NAME_LEN);
                        strncpy(dev->rwlock_info[lockarg.index].procname, current->comm, MUTEX_NAME_LEN-1);
                        mutex_unlock(&semutex_rwlock);  // criticla end
                        break;
                    }
                    else
                    {
                        do_gettimeofday(&end);
                        interval = (end.tv_sec - begin.tv_sec) * 1000;
                        interval += ((end.tv_usec - begin.tv_usec) / 1000);
                    }
                }while(interval < lockarg.time);
                return res;
            }
            break;
        case MDRV_SEMUTEX_TRY_RDLOCK_RWLOCK:  // get read lock (non-block)
            {
                // 1: succ
                int index,flag;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                flag = down_read_trylock(dev->rwlock_info[index].rw_sem);

                if(flag == 1)
                {
                    mutex_lock(&semutex_rwlock);  // criticla start
                    dev->rwlock_info[index].reader += 1;
                    dev->rwlock_info[index].locked_file = filp;
                    memset(dev->rwlock_info[index].procname, 0, MUTEX_NAME_LEN);
                    strncpy(dev->rwlock_info[index].procname, current->comm, MUTEX_NAME_LEN-1);
                    mutex_unlock(&semutex_rwlock);  // criticla end
                }
                return flag;
            }
            break;
        case MDRV_SEMUTEX_WRLOCK_RWLOCK: // get write lock (block)
            {
                // 1: succ
                int index;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                down_write(dev->rwlock_info[index].rw_sem);

                mutex_lock(&semutex_rwlock); // criticla start
                dev->rwlock_info[index].writer = 1;
                dev->rwlock_info[index].write_lock_pid = current->pid;
                dev->rwlock_info[index].write_lock_tgid = current->tgid;
                dev->rwlock_info[index].locked_file = filp;
                memset(dev->rwlock_info[index].procname, 0, MUTEX_NAME_LEN);
                strncpy(dev->rwlock_info[index].procname, current->comm, MUTEX_NAME_LEN-1);
                mutex_unlock(&semutex_rwlock); // criticla end

                return 1;
            }
            break;
        case MDRV_SEMUTEX_TIMEOUT_WRLOCK_RWLOCK: // get write lock (timeout)
            {
                // 1: succ
                LOCKARG lockarg;
                int res = 0;
                struct timeval begin;
                struct timeval end;
                unsigned long interval;
                if (copy_from_user(&lockarg, (void __user *)arg, sizeof(LOCKARG)))
                {
                    printk("[SEMUTEX] get write lock timeout parameter failed! \n");
                    return -1;
                }
                // lockarg.time: ms
                lockarg.index -= (int)MUTEX_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((lockarg.index < 0) || (lockarg.index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[lockarg.index].rw_sem == NULL))
                    return -1;

                do_gettimeofday(&begin);
                do
                {
                    usleep_range(lockarg.time*1000,lockarg.time*1000);
                    res = down_write_trylock(dev->rwlock_info[lockarg.index].rw_sem);
                    if (res == 1)
                    {
                        mutex_lock(&semutex_rwlock);  // criticla start
                        dev->rwlock_info[lockarg.index].writer = 1;
                        dev->rwlock_info[lockarg.index].write_lock_pid = current->pid;
                        dev->rwlock_info[lockarg.index].write_lock_tgid = current->tgid;
                        dev->rwlock_info[lockarg.index].locked_file = filp;
                        memset(dev->rwlock_info[lockarg.index].procname, 0, MUTEX_NAME_LEN);
                        strncpy(dev->rwlock_info[lockarg.index].procname, current->comm, MUTEX_NAME_LEN-1);
                        mutex_unlock(&semutex_rwlock);   // criticla end
                        break;
                    }
                    else
                    {
                        do_gettimeofday(&end);
                        interval = (end.tv_sec - begin.tv_sec) * 1000;
                        interval += ((end.tv_usec - begin.tv_usec) / 1000);
                    }
                }while(interval < lockarg.time);
                return res;
            }
            break;
        case MDRV_SEMUTEX_TRY_WRLOCK_RWLOCK: // get write lock (non-block)
            {
                // 1: succ
                int index;
                int flag;
                err = get_user(index, (int __user *)arg);
                if (err)
                    return -1;

                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                flag = down_write_trylock(dev->rwlock_info[index].rw_sem);

                if(flag == 1)
                {
                    mutex_lock(&semutex_rwlock);  // criticla start
                    dev->rwlock_info[index].writer = 1;
                    dev->rwlock_info[index].write_lock_pid = current->pid;
                    dev->rwlock_info[index].write_lock_tgid = current->tgid;
                    dev->rwlock_info[index].locked_file = filp;
                    memset(dev->rwlock_info[index].procname, 0, MUTEX_NAME_LEN);
                    strncpy(dev->rwlock_info[index].procname, current->comm, MUTEX_NAME_LEN-1);
                    mutex_unlock(&semutex_rwlock);   // criticla end
                }
                return flag;
            }
            break;
        case MDRV_SEMUTEX_RDUNLOCK: // read unlock
            {
                // 0:succ
                int index;
                err = get_user(index, (int __user *)arg);

                if (err)
                    return -1;
                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                mutex_lock(&semutex_rwlock); // critical start
                if(dev->rwlock_info[index].locked_file != filp)
                {
                    printk(KERN_WARNING "WARNING!Try to rdunlock rwlock index %d name %s, but use wrong filp!!! \n",index,dev->rwlock_info[index].name);
                    mutex_unlock(&semutex_rwlock);
                    return -1;
                }

                dev->rwlock_info[index].reader -=1;
                if (dev->rwlock_info[index].reader == 0)
                {
                    dev->rwlock_info[index].locked_file = NULL;
                    memset(dev->rwlock_info[index].procname, 0, MUTEX_NAME_LEN);
                }

                mutex_unlock(&semutex_rwlock);   // critical end
                up_read(dev->rwlock_info[index].rw_sem);
                return 0;
            }
            break;
        case MDRV_SEMUTEX_WRUNLOCK: // write unlock
            {
                // 0:succ
                int index;
                err = get_user(index, (int __user *)arg);

                if (err)
                    return -1;

                index -= (int)RWLOCK_INDEX_BEGIN;
                /* Fix SOPHIA-3815 again. */
                if ((index < 0) || (index >= RWLOCK_SUPPORTED) ||
                    (dev->rwlock_info[index].rw_sem == NULL))
                    return -1;

                mutex_lock(&semutex_rwlock); // critical start

                if(dev->rwlock_info[index].locked_file != filp)
                {
                    printk(KERN_WARNING "WARNING!Try to unlock rwlock index %d name %s, but use wrong filp!!! \n",index,dev->rwlock_info[index].name);
                    mutex_unlock(&semutex_rwlock);
                    return -1;
                }

                dev->rwlock_info[index].writer = 0;
                dev->rwlock_info[index].locked_file = NULL;
                dev->rwlock_info[index].write_lock_pid = 0;
                dev->rwlock_info[index].write_lock_tgid = 0;
                memset(dev->rwlock_info[index].procname, 0, MUTEX_NAME_LEN);

                mutex_unlock(&semutex_rwlock); // critical end
                up_write(dev->rwlock_info[index].rw_sem);
                return 0;
            }
            break;

        /* Event. */
        case MDRV_EVENT_IOC_INIT:
            memset(&stEventParm, 0, sizeof(MDRV_EVENT_PARM));
            if (copy_from_user(&stEventParm, (void __user *)arg, sizeof(MDRV_EVENT_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Event_Init(stEventParm.Num);
            break;

        case MDRV_EVENT_IOC_CREATE:
            memset(&stEventParm, 0, sizeof(MDRV_EVENT_PARM));
            if (copy_from_user(&stEventParm, (void __user *)arg, sizeof(MDRV_EVENT_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Event_Create(stEventParm.Name);
            break;

        case MDRV_EVENT_IOC_DELETE:
            memset(&stEventParm, 0, sizeof(MDRV_EVENT_PARM));
            if (copy_from_user(&stEventParm, (void __user *)arg, sizeof(MDRV_EVENT_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Event_Delete(stEventParm.Id);
            break;

        case MDRV_EVENT_IOC_SET:
            memset(&stEventParm, 0, sizeof(MDRV_EVENT_PARM));
            if (copy_from_user(&stEventParm, (void __user *)arg, sizeof(MDRV_EVENT_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Event_Set(stEventParm.Id, stEventParm.Flag);
            break;

        case MDRV_EVENT_IOC_CLEAR:
            memset(&stEventParm, 0, sizeof(MDRV_EVENT_PARM));
            if (copy_from_user(&stEventParm, (void __user *)arg, sizeof(MDRV_EVENT_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Event_Clear(stEventParm.Id, stEventParm.Flag);
            break;

        case MDRV_EVENT_IOC_WAIT:
            memset(&stEventParm, 0, sizeof(MDRV_EVENT_PARM));
            if (copy_from_user(&stEventParm, (void __user *)arg, sizeof(MDRV_EVENT_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Event_Wait(stEventParm.Id, stEventParm.Flag,
                                     &u32CmpFlag,
                                     stEventParm.Mode,
                                     stEventParm.WaitMs);
            if(copy_to_user((void __user *)(stEventParm.CmpFlag), &u32CmpFlag, sizeof(u32CmpFlag)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_to_user() fail.\n");
                break;
            }
            break;

        /* Queue. */
        case MDRV_QUEUE_IOC_INIT:
            memset(&stQueueParm, 0, sizeof(MDRV_QUEUE_PARM));
            if (copy_from_user(&stQueueParm, (void __user *)arg, sizeof(MDRV_QUEUE_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Queue_Init(stQueueParm.Num);
            break;

        case MDRV_QUEUE_IOC_CREATE:
            memset(&stQueueParm, 0, sizeof(MDRV_QUEUE_PARM));
            if (copy_from_user(&stQueueParm, (void __user *)arg, sizeof(MDRV_QUEUE_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Queue_Create(NULL,
                                       stQueueParm.QueueSize,
                                       stQueueParm.MsgType,
                                       stQueueParm.MsgAlignSize,
                                       stQueueParm.Attr,
                                       stQueueParm.Name);
            break;

        case MDRV_QUEUE_IOC_DELETE:
            memset(&stQueueParm, 0, sizeof(MDRV_QUEUE_PARM));
            if (copy_from_user(&stQueueParm, (void __user *)arg, sizeof(MDRV_QUEUE_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            s32Ret = MDrv_Queue_Delete(stQueueParm.Id);
            break;

        case MDRV_QUEUE_IOC_SEND:
            memset(&stQueueParm, 0, sizeof(MDRV_QUEUE_PARM));
            if (copy_from_user(&stQueueParm, (void __user *)arg, sizeof(MDRV_QUEUE_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            if ((pu8Data = kzalloc(stQueueParm.MsgSendSize, GFP_KERNEL)) == NULL)
            {
                MDRV_MSOS_DEBUG("kzalloc() fail.\n");
                break;
            }
            if (copy_from_user(pu8Data, (void __user *)stQueueParm.MsgData, stQueueParm.MsgSendSize) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                kfree(pu8Data);
                break;
            }
            s32Ret = MDrv_Queue_Send(stQueueParm.Id, pu8Data, stQueueParm.MsgSendSize, stQueueParm.WaitMs);
            kfree(pu8Data);
            pu8Data = NULL;
            break;

        case MDRV_QUEUE_IOC_RECV:
            memset(&stQueueParm, 0, sizeof(MDRV_QUEUE_PARM));
            if (copy_from_user(&stQueueParm, (void __user *)arg, sizeof(MDRV_QUEUE_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            if ((pu8Data = kzalloc(stQueueParm.MsgAlignSize, GFP_KERNEL)) == NULL)
            {
                MDRV_MSOS_DEBUG("kzalloc() fail.\n");
                break;
            }
            s32Ret = MDrv_Queue_Recv(stQueueParm.Id, pu8Data, stQueueParm.MsgAlignSize, &u32Size, stQueueParm.WaitMs);
            if(copy_to_user((void __user *)(stQueueParm.MsgData), pu8Data, stQueueParm.MsgAlignSize) != 0)
            {
                MDRV_MSOS_DEBUG("copy_to_user() fail.\n");
                kfree(pu8Data);
                break;
            }
            kfree(pu8Data);
            pu8Data = NULL;
            if(copy_to_user((void __user *)(stQueueParm.MsgRecvSize), &u32Size, sizeof(u32Size)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_to_user() fail.\n");
                break;
            }
            break;

        case MDRV_QUEUE_IOC_PEEK:
            memset(&stQueueParm, 0, sizeof(MDRV_QUEUE_PARM));
            if (copy_from_user(&stQueueParm, (void __user *)arg, sizeof(MDRV_QUEUE_PARM)) != 0)
            {
                MDRV_MSOS_DEBUG("copy_from_user() fail.\n");
                break;
            }
            if ((pu8Data = kzalloc(stQueueParm.MsgAlignSize, GFP_KERNEL)) == NULL)
            {
                MDRV_MSOS_DEBUG("kzalloc() fail.\n");
                break;
            }
            s32Ret = MDrv_Queue_Peek(stQueueParm.Id, pu8Data, stQueueParm.MsgAlignSize, &u32Size);
            if (s32Ret == TRUE)
            {
                if(copy_to_user((void __user *)(stQueueParm.MsgData), pu8Data, stQueueParm.MsgAlignSize) != 0)
                {
                    MDRV_MSOS_DEBUG("copy_to_user() fail.\n");
                    kfree(pu8Data);
                    break;
                }
                if(copy_to_user((void __user *)(stQueueParm.MsgRecvSize), &u32Size, sizeof(u32Size)) != 0)
                {
                    MDRV_MSOS_DEBUG("copy_to_user() fail.\n");
                    kfree(pu8Data);
                    break;
                }
            }
            kfree(pu8Data);
            pu8Data = NULL;
            break;


        default:
            printk("Unknown ioctl command %d\n", cmd);
            return -ENOTTY;
    }

    return s32Ret;
}

MSYSTEM_STATIC int __init mod_semutex_init(void)
{
    int s32Ret,i;
    dev_t dev;

    semutex_class = class_create(THIS_MODULE, "semutex");
    if (IS_ERR(semutex_class))
    {
        return PTR_ERR(semutex_class);
    }
    for(i = 0 ; i < MUTEX_SUPPORTED ; i ++)
    {
        memset(&SemutexDev.mutex_info[i],0,sizeof(SemutexDev.mutex_info[i]));
        SemutexDev.mutex_info[i].cross_thread_unlock = E_CROSS_THREAD_UNLOCK_DISABLE;
        memset(&SemutexDev.sema_info[i],0,sizeof(SemutexDev.sema_info[i]));
    }
    // for read-write lock
    for(i = 0 ; i < RWLOCK_SUPPORTED ; i ++)
    {
        memset(&SemutexDev.rwlock_info[i],0,sizeof(SemutexDev.rwlock_info[i]));
    }
    if (SemutexDev.s32SemutexMajor)
    {
        dev = MKDEV(SemutexDev.s32SemutexMajor, SemutexDev.s32SemutexMinor);
        s32Ret = register_chrdev_region(dev, MOD_SEMUTEX_DEVICE_COUNT, MOD_SEMUTEX_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, SemutexDev.s32SemutexMinor, MOD_SEMUTEX_DEVICE_COUNT, MOD_SEMUTEX_NAME);
        SemutexDev.s32SemutexMajor = MAJOR(dev);
    }

    if ( 0 > s32Ret)
    {
        class_destroy(semutex_class);
        return s32Ret;
    }

    cdev_init(&SemutexDev.cDevice, &SemutexDev.SemutexFileop);
    if (0!= (s32Ret= cdev_add(&SemutexDev.cDevice, dev, MOD_SEMUTEX_DEVICE_COUNT)))
    {
        unregister_chrdev_region(dev, MOD_SEMUTEX_DEVICE_COUNT);
        class_destroy(semutex_class);
        return s32Ret;
    }
    //init share page list, add by austin
    INIT_LIST_HEAD(&(SemutexDev.share_page_list));
    device_create(semutex_class, NULL, dev, NULL, MOD_SEMUTEX_NAME);


/*
    init_timer(&gtimer);
    gtimer.data = 0;
    gtimer.function = enum_timer_fn;
    gtimer.expires = jiffies + HZ*30;
    mod_timer(&gtimer,jiffies + HZ*30);
*/

    return 0;
}


MSYSTEM_STATIC void __exit mod_semutex_exit(void)
{
    cdev_del(&SemutexDev.cDevice);
    unregister_chrdev_region(MKDEV(SemutexDev.s32SemutexMajor, SemutexDev.s32SemutexMinor), MOD_SEMUTEX_DEVICE_COUNT);

    device_destroy(semutex_class, MKDEV(SemutexDev.s32SemutexMajor, SemutexDev.s32SemutexMinor));
    class_destroy(semutex_class);
}

module_param(mstar_semutex_debug, uint, 0644);
MODULE_PARM_DESC(mstar_debug, "debug for semutex");

module_init(mod_semutex_init);
module_exit(mod_semutex_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("Semutex driver");
MODULE_LICENSE("GPL");
