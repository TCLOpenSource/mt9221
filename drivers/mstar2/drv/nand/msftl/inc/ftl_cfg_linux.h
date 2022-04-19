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


#ifndef _FTL_CFG_LINUX_H_
#define _FTL_CFG_LINUX_H_

// =========================================
// the code connected to system
// =========================================



#include <linux/list.h>
#include <linux/string.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h> /* printk() */
#include <linux/fs.h>     /* everything... */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>

#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <asm/errno.h>
#include <asm/uaccess.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/version.h>
#include <linux/kthread.h>

#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/wait.h>

#include "ftl_dbg.h"

/*
Each partition has a device for 
*/
struct msftl_queue{
	struct task_struct	*thread;
	struct semaphore	thread_sem;
	struct request_queue	*queue;
};

struct msftl_sg
{
    char* buf[128];
    uint32_t u32_512SectorCnt[128];
};

typedef struct _msftl_dev
{
	uint32_t u32_StartSector;
	uint32_t u32_SectorCount;
	struct gendisk *gd;
    
	struct list_head list;
//	struct request_queue *queue;
	struct scatterlist *sg;
    struct scatterlist *bounce_sg;
    
    struct page *bounce_page;
    char* bounce_buf;
	struct msftl_queue uq;
	int sg_len;
	struct device *dev;
	spinlock_t lock;
    
    struct msftl_sg sg_buf;
}msftl_dev_t;

struct msftl_sg_info
{
    char* buf;
    uint32_t u32_512SectorCnt;
    uint32_t u32_512SectorIdx;
    uint32_t u32_512BlkAddr;
    uint32_t u32_PhyRowIdx;
    uint32_t u32_Cmd;
	union {
		struct rb_node rb;
		struct list_head list;
	} u;
};


extern struct mstar_nand_info *info;
extern struct msftl_sg_info* gp_sg_info;
extern struct semaphore FtlFlagSem;
// =========================================
// platform config
// =========================================
#define FTL_HW_TIMER_HZ             (12*1000*1000)

#define FTL_SET_FLAG(x)   {down(&FtlFlagSem); gu32_FtlFlag |= (x); up(&FtlFlagSem);}
#define FTL_CLR_FLAG(x)   {down(&FtlFlagSem); gu32_FtlFlag &= ~(x); up(&FtlFlagSem);}
#define FTL_CHK_FLAG(x)   FTL_CHK_FLAG_KL(x)/*(x == (gu32_FtlFlag & (x)))*/

extern void *FMALLOC(unsigned int);
extern void *FVMALLOC(unsigned int);

extern void *FDMAMALLOC(unsigned int);

extern void FFREE(void*);

extern bool FTL_CHK_FLAG_KL(int x);


#define LOCK_FTL()                  
#define UNLOCK_FTL()                

// 2 background threads:
#define BKG_HK_THREAD               1 // 0: no background hk_thread

#define BKG_W_THREAD                1
    #define bgw_no_test    0
    #define wt_gap         1
    #define bgw_sleepon    2
    #define bgw_waitevt    3
    #define bgw_tasklet    4
    #define TEST_INDEX     bgw_no_test

// SW features
#define PAGE_SIZE_RAM_BUFFER    0
#define HOT_DATA_EN             0
#define BKG_ERASE_DB            0
#define STATIC_WL               1
#define READ_DISTURB            1
	
#define READ_DISTURB_CNT        4

extern void NFTL_Sleep(U32 u32_us);
// =========================================
// debug
// =========================================

#define FTL_DBG_LV_LOG             (0) /* just print */
#define FTL_DBG_LV_ERR             (1) /* Error condition debug messages. */
#define FTL_DBG_LV_WARN            (2) /* Warning condition debug messages. */
#define FTL_DBG_LV_HIGH            (3) /* Debug messages (high debugging). */
#define FTL_DBG_LV_MEDIUM          (4) /* Debug messages. */
#define FTL_DBG_LV_LOW             (5) /* Debug messages (low debugging). */

#define FTL_DEBUG_LEVEL            FTL_DBG_LV_LOW
#define MSFTL_DEBUG_MSG            1

#if defined(MSFTL_DEBUG_MSG) && MSFTL_DEBUG_MSG
//#define ftl_printf(fmt, arg...)   printk(KERN_ALERT fmt, ##arg)
#define ftl_printf                      printk // FIXME

#define ftl_dbg(dbg_lv, tag, str, ...) \
    do { \
        if (dbg_lv > FTL_DEBUG_LEVEL || (FTL_Disable_ErrLog==1 && dbg_lv!=FTL_DBG_LV_ERR) ) \
            break; \
        else { \
	        if (tag) \
            { \
                ftl_printf("[%s %u] ", __func__, __LINE__); \
            } \
            if (dbg_lv == FTL_DBG_LV_ERR) \
		        ftl_printf("FTL Err: "); \
		    else if (dbg_lv == FTL_DBG_LV_WARN) \
		        ftl_printf("FTL Warn: "); \
            ftl_printf(str, ##__VA_ARGS__); \
        } \
    } while(0)
#else /* _FTL_CFG_UBOOT_H_ */

#define ftl_dbg(enable, tag, str, ...) {}
#endif /* MSFTL_DEBUG_MSG */


// --------------------------
#if TEMP_CHECK
#define ftl_die()   {   \
    FTL_Disable_ErrLog=0;  dump_stack(); ftl_dbg(0,0,"\n\n");\
    ftl_dbg(FTL_DBG_LV_ERR,1,"Fatal Error, Stop.\n\n"); NFTL_DumpInfo(); \
    while(1);}
#else
#define ftl_die()   {   \
    FTL_Disable_ErrLog=0;  dump_stack(); ftl_dbg(0,0,"\n\n");\
    ftl_dbg(FTL_DBG_LV_ERR,1,"Fatal Error, Stop.\n\n"); NFTL_DumpInfo(); \
    return FTL_ERR_FATAL;}
#endif

#define ftl_stop() \
	{ftl_dbg(FTL_DBG_LV_LOG,1,"Stop."); while(1)  nand_reset_WatchDog();}


#endif // _FTL_CFG_UBOOT_H_
