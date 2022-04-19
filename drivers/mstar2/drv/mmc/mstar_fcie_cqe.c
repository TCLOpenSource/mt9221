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

#include "mstar_mci.h"
#include "mstar_fcie_cqe.h"
#include "linux/mmc/mmc.h"
#include <linux/of.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MMC_MSTAR_CMDQ

static s32 mstar_fcie_cqe_init(struct mstar_cqe_host *cqe_host, struct mmc_host *mmc);
static int mstar_fcie_cqe_enable(struct mmc_host *mmc, struct mmc_card *card);
static int mstar_fcie_cqe_host_alloc_tdl(struct mstar_cqe_host *cqe_host);
static void _mstar_fcie_cqe_enable(struct mstar_cqe_host *cqe_host);
static void mstar_fcie_cqe_off(struct mmc_host *mmc);
static void _mstar_fcie_cqe_disable(struct mstar_cqe_host *cqe_host);
static void mstar_fcie_cqe_disable(struct mmc_host *mmc);
static void mstar_fcie_dma_map(struct mmc_host *host, struct mmc_request *mrq);
static void mstar_fcie_cqe_post_req(struct mmc_host *host, struct mmc_request *mrq);
static bool mstar_fcie_cqe_can_execute_tasks(struct mstar_cqe_host *cqe_host);
static int mstar_fcie_cqe_execute_tasks(void *Data);
static int mstar_fcie_cqe_prepare_slots_mrgs(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot);
static int mstar_fcie_cqe_request(struct mmc_host *mmc, struct mmc_request *mrq);
static void mstar_fcie_cqe_prep_adma_desc(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot, int tag);
static void mstar_fcie_cqe_prep_link_desc(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot, int tag);
static void mstar_fcie_cqe_prep_tran_desc(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot, int tag);
static void mstar_fcie_cqe_prep_task_desc(struct mstar_cqe_slot *slot, int tag);
static void mstar_fcie_cqe_task(struct mstar_cqe_host *cqe_host, int task_cnt);
static void mstar_fcie_cqe_recovery_needed(struct mstar_cqe_host *cqe_host, struct mmc_request *mrq, bool notify);
static void mstar_fcie_cqe_finish_mrq(struct mstar_cqe_host *cqe_host);
static bool mstar_fcie_cqe_is_idle(struct mstar_cqe_host *cqe_host, int *ret);
static int mstar_fcie_cqe_wait_for_idle(struct mmc_host *mmc);
static bool mstar_fcie_cqe_chk_req_timeout(struct mstar_cqe_host *cqe_host, struct mmc_request *mrq);
static bool mstar_fcie_cqe_timeout(struct mmc_host *mmc, struct mmc_request *mrq, bool *recovery_needed);
static void mstar_fcie_cqe_recovery_start(struct mmc_host *mmc);
static void mstar_fcie_cqe_recovery_finish(struct mmc_host *mmc);
static void mstar_fcie_cqe_error_irq(struct mstar_cqe_host *cqe_host, bool r1_error);
static void mstar_fcie_cqe_recover_mrqs(struct mstar_cqe_host *cqe_host);
static int mstar_fcie_cqe_error_from_flags(int flags);
static void mstar_fcie_cqe_recover_mrq(struct mstar_cqe_host *cqe_host);
#if defined(eMMC_EMULATE_CQE_FAIL) &&eMMC_EMULATE_CQE_FAIL
static void mstar_fcie_cqe_error_retry(struct mstar_cqe_host *cqe_host);
#endif

/******************************************************************************
 * Defines
 ******************************************************************************/

static struct task_struct *mstar_fcie_cqe_thread_st = NULL;

const struct mmc_cqe_ops fcie_cqe_ops = {
    .cqe_enable = mstar_fcie_cqe_enable,
    .cqe_disable = mstar_fcie_cqe_disable,
    .cqe_request = mstar_fcie_cqe_request,
    .cqe_post_req = mstar_fcie_cqe_post_req,
    .cqe_off = mstar_fcie_cqe_off,
    .cqe_wait_for_idle = mstar_fcie_cqe_wait_for_idle,
    .cqe_timeout = mstar_fcie_cqe_timeout,
    .cqe_recovery_start = mstar_fcie_cqe_recovery_start,
    .cqe_recovery_finish = mstar_fcie_cqe_recovery_finish,
};

static bool mstar_fcie_cqe_is_idle(struct mstar_cqe_host *cqe_host, int *ret)
{
    bool is_idle;
    unsigned long flags;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(cqe_host->mmc);

    spin_lock_irqsave(pMStarHost_st->lock, flags);
    is_idle = !cqe_host->qcnt || cqe_host->recovery_halt;
    *ret = cqe_host->recovery_halt ? -EBUSY : 0;
    cqe_host->waiting_for_idle = !is_idle;
    spin_unlock_irqrestore(pMStarHost_st->lock, flags);

    return is_idle;
}

static int mstar_fcie_cqe_wait_for_idle(struct mmc_host *mmc)
{
    struct mstar_cqe_host *cqe_host = mmc->cqe_private;
    int ret;

    wait_event(cqe_host->wait_queue, mstar_fcie_cqe_is_idle(cqe_host, &ret));

    return ret;
}

static void mstar_fcie_cqe_task(struct mstar_cqe_host *cqe_host, int task_cnt)
{
    int i;
    u32 task_start;

    task_start = 0;
    for (i=0; i< task_cnt; i++)
        task_start |= 1<<i;

    REG_FCIE_W1C(FCIE_SD_STATUS, BIT_SD_FCIE_ERR_FLAGS); // W1C
    REG_FCIE_W1C(FCIE_CMDQ_STS, BIT_RSP_R1_ERR|BIT_RSP_IDX_ERR);

    REG_FCIE_SETBIT(GET_REG_ADDR(FCIE4_BASE, (task_cnt-1)*4), BIT0);
    // Set task request          
    REG_FCIE_SETBIT(FCIE_CMDQ_FIFO_CTL, BIT_CMD_FIFO_FULL_TRIG);

    REG_FCIE_W(FCIE_TSK_REQ0, task_start & 0xFFFF);
    REG_FCIE_W(FCIE_TSK_REQ1, (task_start>>16) & 0xFFFF);

    REG_FCIE_SETBIT(FCIE_CMDQ_CTL, BIT_CMDQ_EN);
}

static int mstar_fcie_cqe_host_alloc_tdl(struct mstar_cqe_host *cqe_host)
{
    cqe_host->trans_desc_len = sizeof(struct _Transfer_Descriptor);
    cqe_host->trans_desc_size = cqe_host->trans_desc_len*cqe_host->num_slots;

    cqe_host->trans_desc_base = (struct _Transfer_Descriptor*)dmam_alloc_coherent(mmc_dev(cqe_host->mmc),
        cqe_host->trans_desc_size,
        &cqe_host->trans_desc_dma_base,
        GFP_NOIO);
    if (!cqe_host->trans_desc_base) {
        eMMC_debug(eMMC_DEBUG_LEVEL_ERROR, 1, "memory allocate trans_desc_base fail!\n");
        return -ENOMEM;
    }
    memset(cqe_host->trans_desc_base, 0, cqe_host->trans_desc_size);

    return 0;
}

static void _mstar_fcie_cqe_enable(struct mstar_cqe_host *cqe_host)
{
    struct mmc_host *mmc = cqe_host->mmc;
    #if defined(CONFIG_ARM64)
    uintptr_t uint_dma_addr = 0;
    #else
    u32 dma_addr            = 0;
    #endif

    REG_FCIE_SETBIT(FCIE_SD_MODE, BIT_JOB_START_CLR_SEL|BIT_CLK_EN);
    REG_FCIE_W(FCIE_SD_CTRL, BIT_SYNC_REG_FOR_CMDQ_EN|BIT_ERR_DET_ON|BIT_ADMA_EN);

    REG_FCIE_W(FCIE_JOB_BL_CNT, 1);
    REG_FCIE_W(FCIE_BLK_SIZE, eMMC_SECTOR_512BYTE);

    #if  defined(CONFIG_ARM64)
    uint_dma_addr = eMMC_translate_DMA_address_Ex(cqe_host->trans_desc_dma_base, 0);
    REG_FCIE_W(FCIE_MIU_DMA_ADDR_15_0, uint_dma_addr & 0xFFFF);
    REG_FCIE_W(FCIE_MIU_DMA_ADDR_31_16, (uint_dma_addr >> 16) & 0xFFFF);
    REG_FCIE_W(FCIE_MIU_DMA_ADDR_47_32, uint_dma_addr >> 32);
    #else
    dma_addr = eMMC_translate_DMA_address_Ex(cqe_host->trans_desc_dma_base, 0);
    REG_FCIE_W(FCIE_MIU_DMA_ADDR_15_0, dma_addr & 0xFFFF);
    REG_FCIE_W(FCIE_MIU_DMA_ADDR_31_16, dma_addr >> 16);
    #endif
    REG_FCIE_W(FCIE_MIU_DMA_LEN_15_0, 0x0010);
    REG_FCIE_W(FCIE_MIU_DMA_LEN_31_16, 0x0000);
    REG_FCIE_SETBIT(FCIE_MIE_INT_EN, BIT_ERR_STS);
    REG_FCIE_SETBIT(FCIE_CMDQ_INT_EN, BIT_CMDQ_END_INT);

    mmc->cqe_on = true;
}

static void _mstar_fcie_cqe_disable(struct mstar_cqe_host *cqe_host)
{
    dmam_free_coherent(mmc_dev(cqe_host->mmc), cqe_host->trans_desc_size,
               cqe_host->trans_desc_base,
               cqe_host->trans_desc_dma_base);

    cqe_host->trans_desc_base = NULL;
}

static int mstar_fcie_cqe_enable(struct mmc_host *mmc, struct mmc_card *card)
{
    struct mstar_cqe_host *cqe_host = mmc->cqe_private;
    int err;

    if (cqe_host->enabled)
        return 0;

    cqe_host->rca = card->rca;
    err = mstar_fcie_cqe_init(cqe_host, mmc);
    if (err)
        return err;

    err = mstar_fcie_cqe_host_alloc_tdl(cqe_host);
    if (err)
        return err;

    cqe_host->enabled = true;

    return 0;
}

static void mstar_fcie_cqe_off(struct mmc_host *mmc)
{
    if (!mmc->cqe_on)
        return;

    REG_FCIE_CLRBIT(FCIE_SD_MODE, BIT_JOB_START_CLR_SEL|BIT_CLK_EN);
    REG_FCIE_CLRBIT(FCIE_SD_CTRL, BIT_SYNC_REG_FOR_CMDQ_EN|BIT_ERR_DET_ON|BIT_ADMA_EN);
    REG_FCIE_CLRBIT(FCIE_MIE_INT_EN, BIT_ERR_STS);
    REG_FCIE_CLRBIT(FCIE_CMDQ_INT_EN, BIT_CMDQ_END_INT);

    mmc->cqe_on = false;
}

static void mstar_fcie_cqe_disable(struct mmc_host *mmc)
{
    struct mstar_cqe_host *cqe_host = mmc->cqe_private;

    if (!cqe_host->enabled)
        return;

    mstar_fcie_cqe_off(mmc);

    _mstar_fcie_cqe_disable(cqe_host);

    cqe_host->enabled = false;
}

static void mstar_fcie_cqe_prep_adma_desc(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot, int tag)
{
    struct mmc_request *mrq = slot[tag].mrq;
    struct mmc_data *pData_st   = mrq->data;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(cqe_host->mmc);
    struct scatterlist  *pSG_st = 0;
    dma_addr_t dmaaddr  = 0;
    u32 dmalen          = 0;
    int i, slot_adma_index;

    if (pData_st == NULL)
        eMMC_die("eMMC Err: pData_st is null\n");

    pSG_st = pData_st->sg;

    if( pData_st->sg_len > cqe_host->mmc->max_segs)
        eMMC_die("eMMC Err: sglist has more than max_segs items.\n");

    slot_adma_index = tag*cqe_host->mmc->max_segs;
    for(i=0; i<pData_st->sg_len; i++)
    {
        pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_End = 0;
        dmaaddr = sg_dma_address(pSG_st);
        dmalen = sg_dma_len(pSG_st);
 
        #ifdef CONFIG_MSTAR_M7821
 
        if( dmaaddr >= MSTAR_MIU1_BUS_BASE) // MIU1
        {
            dmaaddr -= MSTAR_MIU1_BUS_BASE;
            dmaaddr += 0x100000000;
        }
        else // MIU0
        {
            dmaaddr -= MSTAR_MIU0_BUS_BASE;
        }
        pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_MiuSel = 0;
  
        #else // Murphy

        #if defined(MSTAR_MIU1_BUS_BASE_H) && defined(ARM_MIU1_BASE_ADDR)       // for 64 bit address beyond 0x1_0000_0000
        if(dmaaddr > MSTAR_MIU1_BUS_BASE_H)
        {
            dmaaddr -= MSTAR_MIU1_BUS_BASE_H;
            dmaaddr += ARM_MIU1_BASE_ADDR;
        }
        else
            dmaaddr -= MSTAR_MIU0_BUS_BASE;
        #else
            dmaaddr -= MSTAR_MIU0_BUS_BASE;
        #endif
            pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_MiuSel = 0;

        #endif

        #if defined(CONFIG_ARM64)
        pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_Address = (u32)dmaaddr;
        pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_Address2 = (u32)(dmaaddr >> 32);
        #else
        pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_Address = (u32)dmaaddr;
        #endif
        pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_DmaLen = dmalen;

        if(dmalen >= 0x200)
            pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_JobCnt = (dmalen >> 9);
        else
            pMStarHost_st->adma_desc_base[slot_adma_index+i].u32_JobCnt = 1;

        pSG_st = sg_next(pSG_st);
    }

    pMStarHost_st->adma_desc_base[slot_adma_index+pData_st->sg_len-1].u32_End = 1;
}

static void mstar_fcie_cqe_prep_link_desc(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot, int tag)
{
    #if defined(CONFIG_ARM64)
    uintptr_t uint_dma_addr     = 0;
    #else
    u32 u32_dma_addr            = 0;
    #endif
    int slot_adma_desc_len, slot_adma_offset;
    struct mmc_request *mrq = slot[tag].mrq;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(cqe_host->mmc);

    if (mrq->data == NULL)
        eMMC_die("eMMC Err: mrq data is null\n");

    slot_adma_desc_len = sizeof(struct _AdmaDescriptor)*cqe_host->mmc->max_segs; 
    slot_adma_offset = tag*slot_adma_desc_len;


    #if defined(CONFIG_ARM64)
    uint_dma_addr = eMMC_translate_DMA_address_Ex(pMStarHost_st->adma_desc_dma_base+slot_adma_offset, 0);
    #else
    u32_dma_addr = eMMC_translate_DMA_address_Ex(pMStarHost_st->adma_desc_dma_base+slot_adma_offset, 0);
    #endif

    #if defined(CONFIG_ARM64)
    cqe_host->trans_desc_base[tag].u32_Address = (u32)uint_dma_addr;
    cqe_host->trans_desc_base[tag].u32_Address2 = (u16)(uint_dma_addr>>32);
    #else
    cqe_host->trans_desc_base[tag].u32_Address = u32_dma_addr;
    #endif

    cqe_host->trans_desc_base[tag].u32_End = 0;
    cqe_host->trans_desc_base[tag].u32_MiuSel = 0;
    cqe_host->trans_desc_base[tag].u32_Link = 1;
    cqe_host->trans_desc_base[tag].u32_Dir = (mrq->data->flags & MMC_DATA_WRITE)?1:0;
    cqe_host->trans_desc_base[tag].u32_JobCnt = (u16)mrq->data->blocks;
    cqe_host->trans_desc_base[tag].u32_Length = mrq->data->blocks * mrq->data->blksz;
}

static void mstar_fcie_cqe_prep_tran_desc(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot, int tag)
{
    mstar_fcie_cqe_prep_adma_desc(cqe_host, slot, tag);
    mstar_fcie_cqe_prep_link_desc(cqe_host, slot, tag);
}

static void mstar_fcie_cqe_prep_task_desc(struct mstar_cqe_slot *slot, int tag)
{
    struct mmc_request *mrq = slot[tag].mrq;
    u16 u16_task_desc_0x00;
    u16 u16_task_desc_0x01;
    u16 u16_task_desc_0x02;
    u16 u16_task_desc_0x03;   
    struct _TaskDescriptor task_desc;

    if (mrq == NULL)
        eMMC_debug(0, 1, " mrq is null\n");

    task_desc.u32_End = 0;
    task_desc.u32_TaskID = tag;
    task_desc.u32_Priority = (mrq->data->flags & MMC_DATA_PRIO)?1:0;
    task_desc.u32_Forced_Prog = (mrq->data->flags & MMC_DATA_FORCED_PRG)?1:0;
    task_desc.u32_Context = 0;
    task_desc.u32_Tag = (mrq->data->flags & MMC_DATA_DAT_TAG)?1:0;
    task_desc.u32_Dir = (mrq->data->flags & MMC_DATA_READ)?1:0;
    task_desc.u32_Reliable = (mrq->data->flags & MMC_DATA_REL_WR)?1:0;

    task_desc.u32_Blk_cnt = mrq->data->blocks;	
    task_desc.u32_Blk_addr = mrq->data->blk_addr;

    u16_task_desc_0x00 = (u16)(MSTAR_CQE_END(task_desc.u32_End)|
        MSTAR_CQE_TSKID(task_desc.u32_TaskID)|
        MSTAR_CQE_PRIORITY(task_desc.u32_Priority)|
        MSTAR_CQE_FORCED_PROG(task_desc.u32_Forced_Prog)|
        MSTAR_CQE_CONTEXT(task_desc.u32_Context)|
        MSTAR_CQE_DATA_TAG(task_desc.u32_Tag)|
        MSTAR_CQE_DATA_DIR(task_desc.u32_Dir)|
        MSTAR_CQE_REL_WRITE(task_desc.u32_Reliable)); 
    
    u16_task_desc_0x01 = task_desc.u32_Blk_cnt & 0xFFFF;
    u16_task_desc_0x02 = mrq->data->blk_addr & 0xFFFF;
    u16_task_desc_0x03 = (mrq->data->blk_addr>>16) & 0xFFFF;
    REG_FCIE_W(GET_REG_ADDR(FCIE4_BASE, tag*4),   u16_task_desc_0x00);
    REG_FCIE_W(GET_REG_ADDR(FCIE4_BASE, tag*4+1), u16_task_desc_0x01);
    REG_FCIE_W(GET_REG_ADDR(FCIE4_BASE, tag*4+2), u16_task_desc_0x02);
    REG_FCIE_W(GET_REG_ADDR(FCIE4_BASE, tag*4+3), u16_task_desc_0x03);
}

static void mstar_fcie_dma_map(struct mmc_host *host, struct mmc_request *mrq)
{
    struct mmc_data *data = mrq->data;    
    dma_map_sg(mmc_dev(host), data->sg, data->sg_len, mstar_mci_get_dma_dir(data));
}

static void mstar_fcie_cqe_post_req(struct mmc_host *host, struct mmc_request *mrq)
{
    struct mmc_data *data = mrq->data;

    if (data) {
        dma_unmap_sg(mmc_dev(host), data->sg, data->sg_len,
                 (data->flags & MMC_DATA_READ) ?
                 DMA_FROM_DEVICE : DMA_TO_DEVICE);
    }
}

static bool mstar_fcie_cqe_chk_req_timeout(struct mstar_cqe_host *cqe_host, struct mmc_request *mrq)
{
    int tag, task_cnt;
    struct mmc_request *unexecuted_mrq = NULL;
    bool timed_out = false;

    task_cnt = cqe_host->qcnt;

    if (task_cnt > 0) {
        tag = cqe_host->cqe_start;

        while(task_cnt > 0)
        {
            unexecuted_mrq = cqe_host->slot[tag].mrq;

            if (unexecuted_mrq == mrq) {
                timed_out = true;
                break;
            }

            tag++;
            if (tag >= cqe_host->num_slots)
                tag =0;

            task_cnt--;
        }
    }
    else
        timed_out = false;

    return timed_out;
}

static bool mstar_fcie_cqe_timeout(struct mmc_host *mmc, struct mmc_request *mrq,
              bool *recovery_needed)
{
    struct mstar_mci_host *pMStarHost_st = mmc_priv(mmc);
    struct mstar_cqe_host *cqe_host = mmc->cqe_private;
    unsigned long flags;
    bool timed_out;
    int tag, task_cnt;

    spin_lock_irqsave(pMStarHost_st->lock, flags);
    timed_out = mstar_fcie_cqe_chk_req_timeout(cqe_host, mrq);
    if (timed_out) {
        tag = cqe_host->cqe_start;
        task_cnt = 0;
        while(task_cnt < cqe_host->recovery_cnt)
        {
            cqe_host->slot[tag].flags = MSTAR_CQE_EXTERNAL_TIMEOUT;

            tag++;
            if (tag >= cqe_host->num_slots)
                tag =0;
            task_cnt++;
        }

        mstar_fcie_cqe_recovery_needed(cqe_host, mrq, false);
        *recovery_needed = cqe_host->recovery_halt;
    }
    spin_unlock_irqrestore(pMStarHost_st->lock, flags);
     
    if (timed_out) {
        eMMC_DumpDriverStatus();
        eMMC_DumpPadClk();
        eMMC_FCIE_DumpRegisters();
        eMMC_FCIE_DumpDebugBus();
        eMMC_Dump_eMMCStatus();
    }

    return timed_out;
}

static void mstar_fcie_cqe_recovery_needed(struct mstar_cqe_host *cqe_host, struct mmc_request *mrq, bool notify)
{
     if (!cqe_host->recovery_halt) {
         cqe_host->recovery_halt = true;
         wake_up(&cqe_host->wait_queue);
         if (notify && mrq->recovery_notifier)
            mrq->recovery_notifier(mrq);         
     }
}

static void mstar_fcie_cqe_finish_mrq(struct mstar_cqe_host *cqe_host)
{
    struct mmc_data *data = 0;
    struct mmc_request *mrq = 0;
    int tag = cqe_host->cqe_start;
    mrq = cqe_host->slot[tag].mrq;

    cqe_host->cqe_start++;
    if (cqe_host->cqe_start >= cqe_host->num_slots)
        cqe_host->cqe_start =0;

    cqe_host->qcnt--;

    data = mrq->data;
    if (data) {
        if (data->error)
            data->bytes_xfered = 0;
        else
            data->bytes_xfered = data->blksz * data->blocks;
    }
    mmc_cqe_request_done(cqe_host->mmc, mrq);
}


static void mstar_fcie_cqe_recovery_start(struct mmc_host *mmc)
{
    mstar_fcie_cqe_off(mmc);

    mmc->cqe_on = false;
}

static int mstar_fcie_cqe_error_from_flags(int flags)
{
    if (!flags)
        return 0;

    /* CRC errors might indicate re-tuning so prefer to report that */
    if (flags & MSTAR_CQE_HOST_CRC)
        return -EILSEQ;

    if (flags & (MSTAR_CQE_EXTERNAL_TIMEOUT | MSTAR_CQE_HOST_TIMEOUT))
        return -ETIMEDOUT;

    return -EIO;
}

static void mstar_fcie_cqe_recover_mrq(struct mstar_cqe_host *cqe_host)
{
    int tag = cqe_host->cqe_start;
    struct mstar_cqe_slot *slot = &cqe_host->slot[tag];
    struct mmc_request *mrq = slot->mrq;
    struct mmc_data *data;

    if (!mrq)
        return;

    cqe_host->cqe_start++;
    if (cqe_host->cqe_start >= cqe_host->num_slots)
        cqe_host->cqe_start =0;

    cqe_host->qcnt -= 1;

    data = mrq->data;
    if (data) {
        data->bytes_xfered = 0;
        data->error = mstar_fcie_cqe_error_from_flags(slot->flags);
    } else 
    mrq->cmd->error = mstar_fcie_cqe_error_from_flags(slot->flags);


    mmc_cqe_request_done(cqe_host->mmc, mrq);

}

static void mstar_fcie_cqe_recover_mrqs(struct mstar_cqe_host *cqe_host)
{
    int i;

    for (i = 0; i < cqe_host->recovery_cnt; i++)
        mstar_fcie_cqe_recover_mrq(cqe_host);
}

static void mstar_fcie_cqe_recovery_finish(struct mmc_host *mmc)
{
    struct mstar_cqe_host *cqe_host = mmc->cqe_private;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(mmc);
    unsigned long flags;

    mstar_fcie_cqe_recover_mrqs(cqe_host);

    spin_lock_irqsave(pMStarHost_st->lock, flags);
    cqe_host->recovery_halt = false;
    cqe_host->cqe_is_busy = false;
    mmc->cqe_on = false;
    wake_up_process(mstar_fcie_cqe_thread_st);  
    spin_unlock_irqrestore(pMStarHost_st->lock, flags);
}


static int mstar_fcie_cqe_error_flags(u16 u16_status)
{
    int error = 0;

    if(u16_status & (BIT_DAT_RD_CERR|BIT_DAT_WR_CERR|BIT_CMD_RSP_CERR))
        error = MSTAR_CQE_HOST_CRC;
    else
        error = MSTAR_CQE_HOST_TIMEOUT;

    return error;
}


static void mstar_fcie_cqe_error_irq(struct mstar_cqe_host *cqe_host, bool r1_error)
{
    int tag , task_cnt;
    volatile u16 u16_status;
    struct mmc_request *mrq = 0;
    if (r1_error == false)
        u16_status = REG_FCIE(FCIE_SD_STATUS);

    tag = cqe_host->cqe_start;
    task_cnt = 0;
    mrq = cqe_host->slot[tag].mrq;
    while(task_cnt < cqe_host->recovery_cnt)
    {
        if (r1_error == true)
            cqe_host->slot[tag].flags = MSTAR_CQE_HOST_OTHER;
        else
            cqe_host->slot[tag].flags = mstar_fcie_cqe_error_flags(u16_status);
    
        tag++;
        if (tag >= cqe_host->num_slots)
            tag =0;
        task_cnt++;
    }
    mstar_fcie_cqe_recovery_needed(cqe_host, mrq, true);
}
#if defined(eMMC_EMULATE_CQE_FAIL) && eMMC_EMULATE_CQE_FAIL

static void mstar_fcie_cqe_error_retry(struct mstar_cqe_host *cqe_host)
{
    int tag , task_cnt;
    struct mmc_request *mrq = 0;

    tag = cqe_host->cqe_start;
    task_cnt = 0;
    mrq = cqe_host->slot[tag].mrq;
    while(task_cnt < cqe_host->recovery_cnt)
    {
        cqe_host->slot[tag].flags = MSTAR_CQE_HOST_CRC;

        tag++;
        if (tag >= cqe_host->num_slots)
            tag =0;
        task_cnt++;
    }
    mstar_fcie_cqe_recovery_needed(cqe_host, mrq, true);
}

#endif
irqreturn_t mstar_fcie_cqe_irq(struct mstar_mci_host *pMStarHost_st)
{
    struct mstar_cqe_host *cqe_host = pMStarHost_st->mmc->cqe_private;
    volatile u16 u16_CQE_Events, u16_Events, u16_CQE_st;
    int tag, task_cnt;
    u32 u32_task_done = 0;

	REG_FCIE_R(FCIE_MIE_EVENT, u16_Events);
    u16_CQE_Events = REG_FCIE(FCIE_CMDQ_EVENT) & REG_FCIE(FCIE_CMDQ_INT_EN);
    REG_FCIE_W1C(FCIE_CMDQ_EVENT, u16_CQE_Events);

    if (u16_Events & BIT_ERR_STS)   
    {
        REG_FCIE_W1C(FCIE_MIE_EVENT, u16_Events);
        spin_lock(pMStarHost_st->lock);
        mstar_fcie_cqe_error_irq(cqe_host, false);    
        spin_unlock(pMStarHost_st->lock); 
        return IRQ_HANDLED;
    }
    #if defined(eMMC_EMULATE_CQE_FAIL) && eMMC_EMULATE_CQE_FAIL
    if(!(prandom_u32() % 1000)) {
        spin_lock(pMStarHost_st->lock);
        mstar_fcie_cqe_error_retry(cqe_host);
        spin_unlock(pMStarHost_st->lock);
		return IRQ_HANDLED;
    }
    #endif

    if(u16_CQE_Events & BIT_CMDQ_END_INT)
    {
        REG_FCIE_R(FCIE_CMDQ_STS,  u16_CQE_st);
        if (u16_CQE_st & (BIT_RSP_IDX_ERR|BIT_RSP_R1_ERR)) {
            spin_lock(pMStarHost_st->lock);
            mstar_fcie_cqe_error_irq(cqe_host, true);
            spin_unlock(pMStarHost_st->lock); 
            return IRQ_HANDLED;
        }
        u32_task_done = (REG_FCIE(FCIE_TSK_DONE1)<<16)|REG_FCIE(FCIE_TSK_DONE0);
        task_cnt = 0;
        for (tag=0;tag < cqe_host->num_slots;tag++) {
            if(u32_task_done & (1<<tag))
                task_cnt++;
        }
        spin_lock(pMStarHost_st->lock);

        for (tag=0;tag < task_cnt;tag++)
            mstar_fcie_cqe_finish_mrq(cqe_host);

        if (cqe_host->waiting_for_idle && !cqe_host->qcnt)
            wake_up(&cqe_host->wait_queue);

        cqe_host->cqe_is_busy = false;
        wake_up_process(mstar_fcie_cqe_thread_st);      
        spin_unlock(pMStarHost_st->lock);
        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static int mstar_fcie_cqe_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct mstar_cqe_host *cqe_host = mmc->cqe_private;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(mmc);
    unsigned long flags;
    int err = 0;

    if (!cqe_host->enabled)
        return -EINVAL;

    /* First request after resume has to re-enable */
    if (!mmc->cqe_on)
        _mstar_fcie_cqe_enable(cqe_host);

    if (mrq->data)
        mstar_fcie_dma_map(mrq->host, mrq);

    spin_lock_irqsave(pMStarHost_st->lock, flags);
    if (cqe_host->recovery_halt) {
        err = -EBUSY;
        goto out_unlock;
    }
    cqe_host->slot[cqe_host->cqe_end].mrq = mrq;
    cqe_host->slot[cqe_host->cqe_end].flags = 0;

    if (cqe_host->qcnt == 0)
         wake_up_process(mstar_fcie_cqe_thread_st);  

    cqe_host->qcnt++;
    cqe_host->cqe_end++;
    if (cqe_host->cqe_end >= cqe_host->num_slots)
        cqe_host->cqe_end = 0;

    out_unlock:
    spin_unlock_irqrestore(pMStarHost_st->lock, flags);

    if (err && mrq->data)
        mstar_fcie_cqe_post_req(mmc, mrq);

    return err;
}

static int mstar_fcie_cqe_prepare_slots_mrgs(struct mstar_cqe_host *cqe_host, struct mstar_cqe_slot *slot)
{
    int task_cnt, tag;

    tag = cqe_host->cqe_start;
    task_cnt = 0;
    while(task_cnt < cqe_host->qcnt)
    {
        slot[task_cnt].mrq = cqe_host->slot[tag].mrq;
        if (slot[task_cnt].mrq->data->flags & MMC_DATA_QBR)
            break;

        tag++;
        if (tag >= cqe_host->num_slots)
            tag =0;
        task_cnt++;
    }
    if (task_cnt ==0)
        task_cnt++;

    cqe_host->recovery_cnt = task_cnt;

    return task_cnt;
}


static bool mstar_fcie_cqe_can_execute_tasks(struct mstar_cqe_host *cqe_host)
{
    bool can_execute_tasks;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(cqe_host->mmc);
    unsigned long flags;

    spin_lock_irqsave(pMStarHost_st->lock, flags);
    can_execute_tasks = (cqe_host->cqe_is_busy == false &&
        cqe_host->recovery_halt == false &&
        cqe_host->qcnt > 0)?true:false;
    spin_unlock_irqrestore(pMStarHost_st->lock, flags);
    return can_execute_tasks;
}

static int mstar_fcie_cqe_execute_tasks(void *Data)
{
    struct mstar_cqe_host *cqe_host = (struct mstar_cqe_host*)Data;
    struct mstar_mci_host *pMStarHost_st = mmc_priv(cqe_host->mmc); 
    int tag, task_cnt =0;
    unsigned long flags;

    do
    {
        if (mstar_fcie_cqe_can_execute_tasks(cqe_host)) {
            spin_lock_irqsave(pMStarHost_st->lock, flags);
            cqe_host->cqe_is_busy = true;
            task_cnt = mstar_fcie_cqe_prepare_slots_mrgs(cqe_host, cqe_host->temp_slot);
            spin_unlock_irqrestore(pMStarHost_st->lock, flags);
            if (task_cnt >0) {
                for (tag =0; tag < task_cnt; tag++) {
                    mstar_fcie_cqe_prep_task_desc(cqe_host->temp_slot, tag);
                    mstar_fcie_cqe_prep_tran_desc(cqe_host, cqe_host->temp_slot, tag);
                }
                wmb();
                mstar_fcie_cqe_task(cqe_host, task_cnt);
            }
        }
        else {
            if (kthread_should_stop()) {
                set_current_state(TASK_RUNNING);
                break;
            }

            spin_lock_irqsave(pMStarHost_st->lock, flags);
            if (cqe_host->cqe_is_busy == true || cqe_host->recovery_halt == true || cqe_host->qcnt == 0)
                set_current_state(TASK_INTERRUPTIBLE);
            spin_unlock_irqrestore(pMStarHost_st->lock, flags);

            schedule();
        }
    }while(1);

    return 0;
}

static s32 mstar_fcie_cqe_init(struct mstar_cqe_host *cqe_host, struct mmc_host *mmc)
{
    s32 err =0;

    if (cqe_host->init_done)
        return 0;

    cqe_host->mmc = mmc;
    cqe_host->mmc->cqe_private = cqe_host;

    cqe_host->num_slots = CQE_NUM_SLOTS;
    cqe_host->slot = kzalloc(sizeof(struct mstar_cqe_slot)*cqe_host->num_slots, GFP_NOIO);
    if (!cqe_host->slot) {
        err = -ENOMEM;
        return err;
    }

    cqe_host->temp_slot = kzalloc(sizeof(struct mstar_cqe_slot)*cqe_host->num_slots, GFP_NOIO);
    if (!cqe_host->temp_slot) {
        err = -ENOMEM;
        return err;
    }

    mmc->cqe_qdepth = CQE_NUM_SLOTS;
    if (mmc->caps2 & MMC_CAP2_CQE_DCMD)
        mmc->cqe_qdepth -= 1;

    REG_FCIE_W(FCIE_RCA, cqe_host->rca);
    REG_FCIE_W(FCIE_CHK_CNT, 0x8100);

    init_waitqueue_head(&cqe_host->wait_queue);

    mstar_fcie_cqe_thread_st = kthread_run(mstar_fcie_cqe_execute_tasks, cqe_host, "mstar_fcie_cqe_thread_st");
    if(IS_ERR(mstar_fcie_cqe_thread_st))
    {
        eMMC_debug(eMMC_DEBUG_LEVEL_ERROR, 1, "eMMC Err: create thread fail \n");
        return PTR_ERR(mstar_fcie_cqe_thread_st);
    }

    cqe_host->init_done = true;

    return err;
}

#endif

