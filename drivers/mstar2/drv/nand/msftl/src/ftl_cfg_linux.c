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

/*
 * Copyright 2008, Freescale Semiconductor, Inc
 * Andy Fleming
 *
 * Based vaguely on the Linux code
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include "ftl_api.h"
#include "../ftl.h"

#define DEVICE_NAME     "msftl"

#define BGTHREAD_HANDLE_REQUEST 1

#define SCATTERLIST_SUPPORT     1

#if defined(SCATTERLIST_SUPPORT) && SCATTERLIST_SUPPORT
#define MSFTL_MAX_SG_SUPPORT     64
#else
#define MSFTL_MAX_SG_SUPPORT     1
#endif

#define WRITE_BOUNCE            1

extern struct semaphore                 PfModeSem;

extern void nand_lock_fcie(void);
extern void nand_unlock_fcie(void);

static LIST_HEAD(msftl_devices);
static LIST_HEAD(msftl_ctrl_devices);
static DEFINE_MUTEX(msftl_devices_mutex);
struct class *msftl_class;
struct timeval t_sg_time1, t_sg_time2;
struct timeval t_orig_time1, t_orig_time2;

struct msftl_sg_info* gp_sg_info;

static int msftl_getgeo(struct block_device *dev, struct hd_geometry *geo)
{
    geo->cylinders = get_capacity(dev->bd_disk) / (4 * 16);
    geo->heads = 4;
    geo->sectors = 16;
    return 0;
}

static int msftl_ioctl(struct block_device *dev, fmode_t file,unsigned cmd, unsigned long arg)
{
    return 0;
}

static struct block_device_operations msftl_fops ={
    .owner      =   THIS_MODULE,
    .ioctl      = msftl_ioctl,
    .getgeo     = msftl_getgeo,
};
static int msftl_transfer(msftl_dev_t *dev, struct request *req)
{
    uint8_t *pu8_Buffer;
    uint32_t u32_SectIdx, u32_SectCnt;
    uint32_t u32_TransferedSectCnt = 0;

    #if defined(SCATTERLIST_SUPPORT) && SCATTERLIST_SUPPORT
    uint32_t u32_SgCnt;
    struct scatterlist *pSg;
    #endif
    int ret = 0, i;
	#if defined(WRITE_BOUNCE) && WRITE_BOUNCE
    size_t buflen=0;
	#endif
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
    u32_SectIdx = blk_rq_pos(req);
    u32_SectCnt = blk_rq_cur_sectors(req);
    #else
    u32_SectIdx = req->sector;
    u32_SectCnt = req->current_nr_sectors;
    #endif
    pu8_Buffer = req->buffer;
    u32_TransferedSectCnt = 0;

    #if 0
    if (req->cmd_flags & REQ_FLUSH){
        printk("[%s] %s flush cmd\n", __func__, rq_data_dir(req) == READ? "READ" :"WRITE");
    }else if (req->cmd_flags & REQ_FUA){
        printk("[%s] %s FUA cmd\n", __func__, rq_data_dir(req) == READ? "READ" :"WRITE");
    }else if (req->cmd_flags & REQ_SYNC){
        printk("[%s] %s SYNC cmd\n", __func__, rq_data_dir(req) == READ? "READ" :"WRITE");
    }
    #endif

    
    if (req->cmd_flags & REQ_DISCARD){
        u32_TransferedSectCnt = req->bio->bi_size >> 9;
        //mod_debug(MOD_TRACE, "\n");
        //discard
    }
    else
    {
        #if defined(SCATTERLIST_SUPPORT) && SCATTERLIST_SUPPORT
        #if defined(WRITE_BOUNCE) && WRITE_BOUNCE
        if(rq_data_dir(req) == READ)
        {
            u32_SgCnt = dma_map_sg(dev->dev, dev->sg, dev->sg_len, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

            if(1)//(u32_SgCnt >= 3)
            {
                for_each_sg(dev->sg, pSg, dev->sg_len, i)
                {
                    dev->sg_buf.buf[i] = sg_virt(pSg);
                    dev->sg_buf.u32_512SectorCnt[i] = sg_dma_len(pSg) >> 9;
                    buflen+=sg_dma_len(pSg);
                    //printk("Read Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, dev->sg_buf.u32_SectorCnt[i], dev->sg_buf.buf[i]);
                }
                u32_SectCnt = buflen>>9;
                
				#if (defined(BKG_W_THREAD) && BKG_W_THREAD) && (defined(TEST_INDEX) && TEST_INDEX==bgw_no_test)
                ret = NFTL_ReadData_sg(&dev->sg_buf, u32_SectCnt, dev->u32_StartSector + u32_SectIdx, dev->sg_len);
				#else 
                nand_lock_fcie();
                ret = NFTL_ReadData_sg(&dev->sg_buf, u32_SectCnt, dev->u32_StartSector + u32_SectIdx, dev->sg_len);
                nand_unlock_fcie();
				#endif
                dma_unmap_sg(dev->dev, dev->sg, dev->sg_len, DMA_BIDIRECTIONAL);//(rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);			
                #if 0
                do_gettimeofday(&t_sg_time2);

                if(t_sg_time1.tv_usec > t_sg_time2.tv_usec)
                {
                    t_sg_time2.tv_sec--;
                    t_sg_time2.tv_usec += 1000000;
                }
                #endif
                //printk("Sg Elapsed time: %lu sec %lu usec, sgcnt %d\n",                
                //t_sg_time2.tv_sec-t_sg_time1.tv_sec,
                //(t_sg_time2.tv_usec-t_sg_time1.tv_usec), dev->sg_len);
                
                if(!ret)
                    u32_TransferedSectCnt = u32_SectCnt;
            }
            else
            {
                pSg = &(dev->sg[0]);
                for(i = 0; i < u32_SgCnt; i ++)
                {
                    pu8_Buffer = sg_virt(pSg);
                    u32_SectCnt = sg_dma_len(pSg) >> 9;      
                    //printk("Read Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, u32_SectCnt, pu8_Buffer);
					#if (defined(BKG_W_THREAD) && BKG_W_THREAD) && (defined(TEST_INDEX) && TEST_INDEX==bgw_no_test)
                    ret = NFTL_ReadData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                    #else
                    nand_lock_fcie();
                    ret = NFTL_ReadData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);                
                    nand_unlock_fcie();//<-
					#endif
                    if(ret)
                    {
                        printk("[%s]transfer error 0x%X\n", __func__, ret);
                        goto ERRUNMAPSG;
                    }
                    u32_TransferedSectCnt += u32_SectCnt;
                    u32_SectIdx += u32_SectCnt;
                    pSg = sg_next(pSg);
                }
                dma_unmap_sg(dev->dev, dev->sg, dev->sg_len, DMA_BIDIRECTIONAL);//(rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);			
            }

            #if 0
            u32_TransferedSectCnt = 0;
            do_gettimeofday(&t_orig_time1);
//            u32_SgCnt = dma_map_sg(dev->dev, dev->sg, dev->sg_len, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

            pSg = &(dev->sg[0]);
            for(i = 0; i < u32_SgCnt; i ++)
            {
                pu8_Buffer = sg_virt(pSg);
                u32_SectCnt = sg_dma_len(pSg) >> 9;      
                printk("Read Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, u32_SectCnt, pu8_Buffer);

                #if defined(BKG_THREAD_WRITEBUF) && BKG_THREAD_WRITEBUF
                ret = NFTL_CopyFromWriteBuf(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                if(ret)
                {
                    printk("[%s]transfer error 0x%X\n", __func__, ret);
                    goto ERRUNMAPSG;             
                }
                #else
                //printk("Read Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, u32_SectCnt, pu8_Buffer);
                //nand_lock_fcie();
                ret = NFTL_ReadData(dev->bounce_buf, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                //nand_unlock_fcie();
                if(ret)
                {
                    printk("[%s]transfer error 0x%X\n", __func__, ret);
                    goto ERRUNMAPSG;
                }
                #endif
                //compare
                #if 1
                if(memcmp(dev->bounce_buf, pu8_Buffer, sg_dma_len(pSg)))
                {
                    printk("read compare fails\n");
                    printk("good\n");
                    dump_mem(dev->bounce_buf, sg_dma_len(pSg));
                    printk("bad\n");
                    dump_mem(pu8_Buffer, sg_dma_len(pSg));
                    ftl_die();
                }
                #endif
                
                u32_TransferedSectCnt += u32_SectCnt;
                u32_SectIdx += u32_SectCnt;
                pSg = sg_next(pSg);
            }            
  //          dma_unmap_sg(dev->dev, dev->sg, dev->sg_len, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
            do_gettimeofday(&t_orig_time2);
            
            if(t_orig_time1.tv_usec > t_orig_time2.tv_usec)
            {
                t_orig_time2.tv_sec--;
                t_orig_time2.tv_usec += 1000000;
            }
            //printk("orig Elapsed time: %lu.%lu sec, sgcnt %d\n",                
            //t_orig_time2.tv_sec-t_orig_time1.tv_sec,
            //(t_orig_time2.tv_usec-t_orig_time1.tv_usec), dev->sg_len);
            #endif

            #if 0
            if((t_orig_time2.tv_usec-t_orig_time1.tv_usec) > (t_sg_time2.tv_usec-t_sg_time1.tv_usec))
            {
                printk("fast sg Elapsed time: orig %lu sg %lu usec, sgcnt %d\n",(t_orig_time2.tv_usec-t_orig_time1.tv_usec), (t_sg_time2.tv_usec-t_sg_time1.tv_usec), dev->sg_len);
            }
            else
                printk("fast orig Elapsed time: orig %lu sg %lu usec, sgcnt %d\n",(t_orig_time2.tv_usec-t_orig_time1.tv_usec), (t_sg_time2.tv_usec-t_sg_time1.tv_usec), dev->sg_len);
            #endif
            
        }
        else if(rq_data_dir(req) == WRITE)
        {
            buflen = 0;
            for_each_sg(dev->sg, pSg, dev->sg_len, i)
                buflen+=sg_dma_len(pSg);
            
            sg_init_one(dev->bounce_sg, dev->bounce_buf, buflen);
            
            sg_copy_to_buffer(dev->sg, dev->sg_len,
                dev->bounce_buf, dev->bounce_sg[0].length);

            u32_SgCnt = dma_map_sg(dev->dev, dev->bounce_sg, 1, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
            //flush_dcache_page(dev->bounce_page);
            for_each_sg(dev->bounce_sg, pSg, u32_SgCnt, i)
            {
                pu8_Buffer = sg_virt(pSg);
                u32_SectCnt = sg_dma_len(pSg) >> 9;
                //printk("Write Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, u32_SectCnt, pu8_Buffer);                
                #if (defined(BKG_W_THREAD) && BKG_W_THREAD) && (defined(TEST_INDEX) && TEST_INDEX==bgw_no_test)
                ret = NFTL_WriteData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                #else
                nand_lock_fcie();
                ret = NFTL_WriteData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                nand_unlock_fcie(); // <-
                #endif
                if(ret)
                {
                    printk("[%s]transfer error 0x%X\n", __func__, ret);
                    goto ERRUNMAPSG;
                }

                u32_TransferedSectCnt += u32_SectCnt;
                u32_SectIdx += u32_SectCnt;
            }            
            dma_unmap_sg(dev->dev, dev->bounce_sg, 1, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);        
        }
        #else // else of WRITE_BOUNCE
        u32_SgCnt = dma_map_sg(dev->dev, dev->sg, dev->sg_len, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

        pSg = &(dev->sg[0]);
        for(i = 0; i < u32_SgCnt; i ++)
        {
            pu8_Buffer = sg_virt(pSg);
            u32_SectCnt = sg_dma_len(pSg) >> 9;      

            switch(rq_data_dir(req))
            {
                case READ:
                  //printk("Read Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, u32_SectCnt, pu8_Buffer);
					#if (defined(BKG_W_THREAD) && BKG_W_THREAD) && (defined(TEST_INDEX) && TEST_INDEX==bgw_no_test)
                    ret = NFTL_ReadData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);					
					#else
                    nand_lock_fcie();
                    ret = NFTL_ReadData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                    nand_unlock_fcie();
					#endif
    //                ret = nand_ReadFlow(dev->u32_StartSector + u32_SectIdx, u32_SectCnt, (uintptr_t)pu8_Buffer);
                    break;
                case WRITE:
                  //printk("Write Sectors from %lX length %lX, buffer 0x%X ++\n", dev->u32_StartSector + u32_SectIdx, u32_SectCnt, pu8_Buffer);
					#if (defined(BKG_W_THREAD) && BKG_W_THREAD) && (defined(TEST_INDEX) && TEST_INDEX==bgw_no_test)
                    ret = NFTL_WriteData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);					
                  	#else
                    nand_lock_fcie();
                    ret = NFTL_WriteData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);
                    nand_unlock_fcie();
					#endif
    //                ret = nand_WriteFlow(dev->u32_StartSector + u32_SectIdx, u32_SectCnt, (uintptr_t)pu8_Buffer);            
                    break;
                default:
                    printk("[%s] Unknown request 0x%X\n", __func__, rq_data_dir(req));
                    goto ERRUNMAPSG;
            }

            if(ret)
            {
                printk("[%s]transfer error 0x%X\n", __func__, ret);
                goto ERRUNMAPSG;
            }

            u32_TransferedSectCnt += u32_SectCnt;
            u32_SectIdx += u32_SectCnt;
            pSg = sg_next(pSg);
        }
        /*
        * Reason for Using BIDIRECTIONAL is that Using DMA_FROM_DEVICE would lead dma data fail when sector count  == 1
        */
        dma_unmap_sg(dev->dev, dev->sg, dev->sg_len, (rq_data_dir(req) == READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
        #endif      //WRITE_BOUNCE
        
        #else // else of SCATTERLIST_SUPPORT
        switch(rq_data_dir(req))
        {
            case READ:
                
                nand_lock_fcie();
                ret = NFTL_ReadData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);                
                nand_unlock_fcie();
    //                ret = nand_ReadFlow(dev->u32_StartSector + u32_SectIdx, u32_SectCnt, (uintptr_t)pu8_Buffer);
                break;
            case WRITE:
                
                nand_lock_fcie();
                ret = NFTL_WriteData(pu8_Buffer, u32_SectCnt, dev->u32_StartSector + u32_SectIdx);                
                nand_unlock_fcie();
    //                ret = nand_WriteFlow(dev->u32_StartSector + u32_SectIdx, u32_SectCnt, (uintptr_t)pu8_Buffer);            
                break;
            default:
                printk("[%s] Unknown request 0x%X\n", __func__, rq_data_dir(req));
                goto ERRINVAL;
        }
        if(ret)
        {
            printk("[%s]transfer error 0x%X\n", __func__, ret);
            goto ERRINVAL;
        }

        u32_TransferedSectCnt += u32_SectCnt;
        #endif // SCATTERLIST_SUPPORT
    }
    //printk("transfer sector1 %lX\n", u32_TransferedSectCnt);

    
    spin_lock_irq(&dev->lock);      //must call queue lock before issue blk_end_request
    __blk_end_request(req, (ret!= 0), u32_TransferedSectCnt << 9);
    spin_unlock_irq(&dev->lock);

    //printk("transfer sector2 %lX\n", u32_TransferedSectCnt);

    return 0;
#if defined(SCATTERLIST_SUPPORT) && SCATTERLIST_SUPPORT
ERRUNMAPSG:
    dma_unmap_sg(dev->dev, dev->sg, dev->sg_len, DMA_BIDIRECTIONAL);
#else
ERRINVAL:
#endif
    //nand_unlock_fcie();gijgerjogjerogjo
    spin_lock_irq(&dev->lock);
    __blk_end_request(req, (ret!= 0), u32_TransferedSectCnt << 9);
    spin_unlock_irq(&dev->lock);

    return -EINVAL;
}

static void msftl_request(struct request_queue *rq)
{
    msftl_dev_t *dev = rq->queuedata;
    #if defined(BGTHREAD_HANDLE_REQUEST) && BGTHREAD_HANDLE_REQUEST
    wake_up_process(dev->uq.thread);
    #else
    struct request *req;
    while((req = elv_next_request(rq)) != NULL)
    {
        if(!blk_fs_request(req))
        {
            blk_end_request(req, 1, req->nr_sectors << 9);
            continue;
        }

        #if defined(SCATTERLIST_SUPPORT) && SCATTERLIST_SUPPORT
        dev->sg_len = blk_rq_map_sg(rq, req, dev->sg);
        #endif

        msftl_transfer(dev, req);
    }
    #endif
}

#if defined(BGTHREAD_HANDLE_REQUEST) && BGTHREAD_HANDLE_REQUEST

static int msftl_queue_thread(void *d)
{
    msftl_dev_t *dev = d;
    struct request_queue *q = dev->uq.queue;

    current->flags |= PF_MEMALLOC;

    down(&dev->uq.thread_sem);
    do{
        struct request *req = NULL;
    
        spin_lock_irq(q->queue_lock);
        set_current_state(TASK_INTERRUPTIBLE);
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
        req = blk_fetch_request(q);
        #else
        req = elv_next_request(q);
        #endif
        spin_unlock_irq(q->queue_lock);
        
        if (req != NULL) {

            set_current_state(TASK_RUNNING);
            #if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,36)
            if(!blk_fs_request(req))
            {
                spin_lock_irq(&dev->lock);
                __blk_end_request(req, 1, req->nr_sectors << 9);
                spin_unlock_irq(&dev->lock);
            }
            else
            #endif
            {
                #if defined(SCATTERLIST_SUPPORT) && SCATTERLIST_SUPPORT
                dev->sg_len = blk_rq_map_sg(q, req, dev->sg);
                #endif
                msftl_transfer(dev, req);
            }
        } else {
            if (kthread_should_stop()) {
                set_current_state(TASK_RUNNING);
                break;
            }
            up(&dev->uq.thread_sem);
            schedule();
            down(&dev->uq.thread_sem);
        }
    }while(1);
    up(&dev->uq.thread_sem);
    return 0;
}
#endif

static void msftl_delete_disk(msftl_dev_t *dev)
{
    if(dev->gd)
    {
        del_gendisk(dev->gd);
        put_disk(dev->gd);
    }

    if(dev->uq.queue)
        blk_cleanup_queue(dev->uq.queue);

    kfree(dev);
}

static int msftl_create_disk(int part_num, int major_number)
{
    msftl_dev_t *dev;
    uint32_t u32_TotalSectCnt;

    dev = kzalloc(sizeof(msftl_dev_t), GFP_KERNEL);
    if(!dev)
    {
        printk(KERN_CRIT"[%s] msftl_dev malloc fail\n", __func__);
        return -ENOMEM;
    }
    spin_lock_init(&dev->lock);
    INIT_LIST_HEAD(&dev->list);

    /*
        get partition info here
    */
    dev->u32_StartSector = 0;  
    dev->u32_SectorCount = u32_TotalSectCnt = NFTL_GetDataCapacity();
    
    list_add_tail(&dev->list, &msftl_devices);
    //initial request queue
    dev->uq.queue = blk_init_queue(msftl_request, &dev->lock);
    dev->uq.queue->queuedata = dev;

    blk_queue_bounce_limit(dev->uq.queue, BLK_BOUNCE_HIGH);
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
//  blk_queue_logical_block_size(dev->uq.queue, 2048);      //if block size is set to 2048, parameter "-b" of mkfs.ext2 should be set to 2048 
//  blk_queue_physical_block_size(dev->uq.queue, 2048);     //better write performance but poorer read performance
//  blk_queue_io_min(dev->uq.queue, 2048);
    blk_queue_max_hw_sectors(dev->uq.queue, 512);
    blk_queue_max_segments(dev->uq.queue, MSFTL_MAX_SG_SUPPORT);
    blk_queue_max_segment_size(dev->uq.queue, MSFTL_MAX_SG_SUPPORT << 9);
//    dev->uq.queue->flush_flags |= (REQ_FUA | REQ_FLUSH| REQ_SYNC);
//    dev->uq.queue->queue_flags |= (1 << QUEUE_FLAG_DISCARD);
//    blk_queue_max_discard_sectors(dev->uq.queue, 131072*16);            //max discard sectors ???    
    #else
    //hardsect size:    set for minimum size that upper layer passby; can be set as 512, 1024, 2048
    blk_queue_hardsect_size(dev->uq.queue, 512);
    //max_sectors <-- unit is 512 Byte
    blk_queue_max_sectors(dev->uq.queue, 128);
    blk_queue_max_phys_segments(dev->uq.queue, MSFTL_MAX_SG_SUPPORT);
    blk_queue_max_hw_segments(dev->uq.queue, MSFTL_MAX_SG_SUPPORT);
    #endif

    /*allocate scatterlist*/

    dev->sg = kmalloc(sizeof(struct scatterlist) * MSFTL_MAX_SG_SUPPORT, GFP_KERNEL);
    if(!dev->sg)
    {
        list_del(&dev->list);
        blk_cleanup_queue(dev->uq.queue);
        kfree(dev);
        printk(KERN_CRIT"[%s] scatterlist malloc fail\n",__func__);
        return -ENOMEM;
    }
    sg_init_table(dev->sg, MSFTL_MAX_SG_SUPPORT);

    #if defined(WRITE_BOUNCE) && WRITE_BOUNCE
    dev->bounce_sg = kmalloc(sizeof(struct scatterlist) * 1, GFP_KERNEL);
    if(!dev->bounce_sg)
    {
        kfree(dev->sg);
        list_del(&dev->list);
        blk_cleanup_queue(dev->uq.queue);
        kfree(dev);
        printk(KERN_CRIT"[%s] scatterlist malloc fail\n",__func__);
        return -ENOMEM;        
    }
    sg_init_table(dev->bounce_sg, 1);

    dev->bounce_page = alloc_pages(__GFP_COMP,6);
    if(!dev->bounce_page)
    {
        kfree(dev->bounce_sg);
        kfree(dev->sg);
        list_del(&dev->list);
        blk_cleanup_queue(dev->uq.queue);
        kfree(dev);
        printk(KERN_CRIT"[%s] pages malloc fail\n",__func__);
        return -ENOMEM;
    }
    dev->bounce_buf = kmap(dev->bounce_page);
    #endif

    #if defined(BGTHREAD_HANDLE_REQUEST) && BGTHREAD_HANDLE_REQUEST
    sema_init(&dev->uq.thread_sem, 1);

    dev->uq.thread = kthread_run(msftl_queue_thread, dev, "msftlqd/%d",part_num);
    if (IS_ERR(dev->uq.thread)) {
        kfree(dev->sg);
        #if defined(WRITE_BOUNCE) && WRITE_BOUNCE
        kfree(dev->bounce_sg);        
        free_pages((unsigned long)dev->bounce_page, 6);
        #endif
        list_del(&dev->list);
        blk_cleanup_queue(dev->uq.queue);
        kfree(dev);
        return PTR_ERR(dev->uq.thread);
    }
    #endif

    //sg_structure
    gp_sg_info = vmalloc(384*sizeof(struct msftl_sg_info));
    if(gp_sg_info == NULL)
    {
        kfree(dev->sg);
        #if defined(WRITE_BOUNCE) && WRITE_BOUNCE
        kfree(dev->bounce_sg);        
        free_pages((unsigned long)dev->bounce_page, 6);
        #endif        
        list_del(&dev->list);
        blk_cleanup_queue(dev->uq.queue);
        kfree(dev);        
        printk(KERN_CRIT"[%s] sg struct malloc fail\n",__func__);
        return -ENOMEM;
    }

    //initial gendisk
    dev->gd = alloc_disk(1);
    if(!dev->gd)
    {
        kfree(dev->sg);
        #if defined(WRITE_BOUNCE) && WRITE_BOUNCE
        kfree(dev->bounce_sg);        
        free_pages((unsigned long)dev->bounce_page, 6);
        #endif        
        list_del(&dev->list);
        blk_cleanup_queue(dev->uq.queue);
        kfree(dev);
        printk(KERN_CRIT"[%s] gendisk malloc fail\n",__func__);
        return -ENOMEM;
    }

    dev->gd->major = major_number;
    dev->gd->first_minor = part_num * 128;
    dev->gd->fops = &msftl_fops;
    dev->gd->queue = dev->uq.queue;
    dev->gd->private_data = dev;
    dev->gd->minors = 128;                  //max partition is 128
    dev->gd->flags |= GENHD_FL_EXT_DEVT;     //allow partitions

    sprintf(dev->gd->disk_name,"msftl%d", part_num);

    set_capacity(dev->gd, u32_TotalSectCnt);
    
    add_disk(dev->gd);

    //device_create(msftl_class, NULL, MKDEV(major_number, part_num), NULL, dev->gd->disk_name);

    return 0;
}


int FTL_STARTBLK = 0;
int FTL_BLKCNT = 0;

int msftl_mtd_part_setup(void)
{
    uint64_t offset, size, size_tmp, offset_tmp;
	struct mtd_info *mtd;

    mtd = get_mtd_device_nm("MSFTL");
    if (IS_ERR(mtd) && PTR_ERR(mtd) == -ENODEV)
        return 1;

    offset = mtd_get_partition_offset(mtd);

    size = mtd->size;
    offset_tmp = offset;
    do_div(offset_tmp, mtd->erasesize);

    size_tmp = mtd->size;
    do_div(size_tmp, mtd->erasesize);

    FTL_STARTBLK = offset_tmp;
    FTL_BLKCNT= size_tmp;
    if(offset + size == mtd_get_device_size(mtd))
    {
        size_tmp = mtd->size;
        do_div(size_tmp, mtd->erasesize * 128);
        FTL_BLKCNT -= size_tmp;
    }
    printk("[%s]\tstartblk %d, blkcnt %d\n", __func__, FTL_STARTBLK, FTL_BLKCNT);

    return 0;
}


static int msftl_blkdev_create(int major_number)
{
    int iRet = 0;
    printk("Mstar FTL:");
    msftl_mtd_part_setup();

    if(NFTL_Init(FTL_STARTBLK, FTL_BLKCNT))
        return 1;
    iRet = msftl_create_disk(0, major_number);
    printk("\n");
    return iRet;
}

static void msftl_blkdev_free(void)
{
    msftl_dev_t *dev;
    struct list_head *this, *next;

    list_for_each_safe(this, next, &msftl_devices) {
        dev = list_entry(this, msftl_dev_t, list);
        msftl_delete_disk(dev);
        list_del(&dev->list);
    }
}


//===========================================
void NFTL_Sleep(U32 u32_us)
{
    if(u32_us <= MAX_UDELAY_MS * 1000)
        udelay(u32_us);
    else
        mdelay(u32_us/1000);
}

void *FMALLOC(unsigned int ByteCnt)
{
    void *p;
    p = kzalloc(ByteCnt+UNFD_CACHE_LINE-1, GFP_KERNEL);
    if(NULL==p)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"size: %Xh \n", ByteCnt);
        panic("%s\n FTL Assert(%d)\n", __func__, __LINE__);
    }
    p = (void*)(((U32)p+(UNFD_CACHE_LINE-1)) & ~(UNFD_CACHE_LINE-1));
    return p;
}

void *FDMAMALLOC(unsigned int ByteCnt)
{
    void *p;
	dma_addr_t handle;
	
    p = dma_alloc_coherent(NULL, ByteCnt+UNFD_CACHE_LINE-1, &handle, GFP_KERNEL);
    if(NULL==p)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"size: %Xh \n", ByteCnt);
        panic("%s\n FTL Assert(%d)\n", __func__, __LINE__);
    }
    p = (void*)(((U32)p+(UNFD_CACHE_LINE-1)) & ~(UNFD_CACHE_LINE-1));	
    return p;
}


void *FVMALLOC(unsigned int ByteCnt)
{
    void *p;
    p = vmalloc(ByteCnt);
    if(NULL==p)
    {
        ftl_dbg(FTL_DBG_LV_ERR,1,"size: %Xh \n", ByteCnt);
        ftl_die();
    }
    return p;
}

bool FTL_CHK_FLAG_KL(int x)
{
	bool b;
	down(&FtlFlagSem);
	b = (x == (gu32_FtlFlag & (x)));
	up(&FtlFlagSem);
	return b;
}


void FFREE(void *p)
{
}


int msftl_resume(void)
{

//  down(&FtlFlagSem);
//  NFTL_ResetReadyFlag();
//  up(&FtlFlagSem);
    //printk("msftl resume +\n");
    if(NFTL_Resume(FTL_STARTBLK, FTL_BLKCNT))
        return 1;
//  gu32_FtlFlag |= FTL_INIT_READY;
//  FTL_SET_FLAG(FTL_INIT_READY);
//  up(&FtlFlagSem);

    printk("msftl resume done\n");
    return 0;
}

EXPORT_SYMBOL(msftl_resume);


int major_number;
static int __init mstar_msftl_init(void)
{
    major_number = register_blkdev(0, DEVICE_NAME);
    if(major_number <= 0)
    {
        printk(KERN_CRIT"[%s] unable to get major\n", __func__);
        return -EAGAIN;
    }

    msftl_class = class_create(THIS_MODULE, "msftl");
    if(IS_ERR(msftl_class))
    {
        printk(KERN_CRIT"[%s] create msftl class fail\n", __func__);
        unregister_blkdev(major_number, DEVICE_NAME);        
        return -EAGAIN;
    }

    if(msftl_blkdev_create(major_number))
    {
        printk(KERN_CRIT"[%s] create disk fail\n", __func__);
        unregister_blkdev(major_number, DEVICE_NAME);
        return -EINVAL;
    }
    return 0;
}

static void __exit mstar_msftl_exit(void)
{
    msftl_blkdev_free();
    unregister_blkdev(major_number, DEVICE_NAME);
}

//module_init(mstar_unfd_block_init);
late_initcall(mstar_msftl_init);
module_exit(mstar_msftl_exit);

MODULE_LICENSE("MSTAR");
MODULE_AUTHOR("mstarsemi");
MODULE_DESCRIPTION("The block device interface for mstar ftl");






