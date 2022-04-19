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


#ifndef MSTAR_MCI_H
#define MSTAR_MCI_H

#include "./inc/common/eMMC.h"

/******************************************************************************
* Function define for this driver
******************************************************************************/

/******************************************************************************
* Register Address Base
******************************************************************************/
#define CLK_400KHz       		400*1000
#define CLK_200MHz       		200*1000*1000

#define eMMC_GENERIC_WAIT_TIME  (HW_TIMER_DELAY_1s*10)
#define eMMC_READ_WAIT_TIME     (HW_TIMER_DELAY_500ms)

/******************************************************************************
* Low level type for this driver
******************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,20)

struct mstar_mci_host_next
{
    unsigned int                dma_len;
    s32                         cookie;
};

#endif

struct mstar_mci_host
{
    struct mmc_host *mmc;
    struct mmc_command *cmd;
    struct mmc_request *request;

    void __iomem *baseaddr;
    int irq;

    u16 sd_clk;
    u16 sd_mod;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,20)

    struct mstar_mci_host_next next_data;
    #ifdef CONFIG_MMC_MSTAR_NO_WORK_QUEUE
    struct delayed_work wait_dmaend_timeout_work;
    struct delayed_work card_busy_timeout_work;
    #else
    struct work_struct async_work;
    #endif

#endif

    struct _AdmaDescriptor *adma_desc_base;
    u32 adma_desc_len;
	#if defined (ENABLE_UMA) && ENABLE_UMA
    dma_addr_t adma_desc_dma_base;
    #endif
    spinlock_t  *lock;
};  /* struct mstar_mci_host*/

extern  void eMMC_dump_WR_Count(void);
extern  void eMMC_record_WR_time(U8 u8_CmdIdx, U32 u32_DataByteCnt);

struct mstar_rw_speed
{
    s64 s64_total_read_bytes;
    s64 s64_total_read_time_usec;//usec
    s64 s64_total_write_bytes;
    s64 s64_total_write_time_usec;//usec

    s64 s64_day_write_bytes;
    int int_tm_yday; /* day of year (0 -365) */
};



extern spinlock_t fcie_lock;
extern ktime_t starttime;
extern struct mstar_rw_speed emmc_rw_speed;
extern U32 u32_miu_chksum_nbytes;
#define eMMC_DAY_WRITE_WARN  (1024*1024*1024) // 1 GB
extern struct attribute *mstar_mci_attr[];
extern U32 gu32_eMMC_read_log_enable;
extern U32 gu32_eMMC_write_log_enable;
extern U32 gu32_eMMC_monitor_enable;
extern u8 u8_enable_sar5;
extern void eMMC_Check_Life(U32 u32_DataByteCnt);
extern int mstar_mci_get_dma_dir(struct mmc_data *data);
#if defined(ENABLE_FCIE_MIU_CHECKSUM) && ENABLE_FCIE_MIU_CHECKSUM
extern void eMMC_enable_miu_chksum(void);
extern void eMMC_clear_miu_chksum(void);
extern U32 mstar_mci_miu_chksum(struct mmc_data *data);
#endif



#endif
