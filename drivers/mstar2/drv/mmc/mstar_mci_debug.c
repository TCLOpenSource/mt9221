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
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#if defined(ENABLE_FCIE_MIU_CHECKSUM) && ENABLE_FCIE_MIU_CHECKSUM
void eMMC_enable_miu_chksum(void)
{
    // Enable MIU checksum
    REG_FCIE_CLRBIT(GET_REG_ADDR(FCIE1_BASE, 0x50), BIT2);

    REG_FCIE_CLRBIT(FCIE_EMMC_DEBUG_BUS1, BIT11|BIT10|BIT9|BIT8);
    REG_FCIE_SETBIT(FCIE_EMMC_DEBUG_BUS1, 9<<8);

    REG_FCIE_CLRBIT(GET_REG_ADDR(FCIE1_BASE, 0x50), BIT3);

    REG_FCIE_CLRBIT(GET_REG_ADDR(FCIE1_BASE, 0x51), BIT15|BIT14|BIT13);
}

void eMMC_clear_miu_chksum(void)
{
    int i;

    // Clear checksum
    REG_FCIE_SETBIT(GET_REG_ADDR(FCIE1_BASE, 0x50), BIT2);
    //udelay(1);

    // Make sure checksum is cleared
    for (i = 0x10; i<=0x1F; i++) {
        REG_FCIE_CLRBIT(GET_REG_ADDR(FCIE1_BASE, 0x51), BIT12|BIT11|BIT10|BIT9|BIT8);
        REG_FCIE_SETBIT(GET_REG_ADDR(FCIE1_BASE, 0x51), i<<8);
    
        if (REG_FCIE(FCIE_EMMC_DEBUG_BUS0) != 0) {
            eMMC_debug(eMMC_DEBUG_LEVEL, 0, "Debug bus is not cleared\n");
            break;
        }
    }

    REG_FCIE_CLRBIT(GET_REG_ADDR(FCIE1_BASE, 0x50), BIT2);
}

U32 mstar_mci_miu_chksum(struct mmc_data *data)
{
    struct scatterlist  *pSG_st = data->sg;
    int i;
    u32 dmalen                  = 0;
    u16 checksum[8];
    u16 old_checksum[8];
    u16 data_in[8];
    u16 *u16_pBuf;
    struct sg_mapping_iter miter;
    int nents;

    memset(checksum, 0, 8*sizeof(u16));
    memset(old_checksum, 0, 8*sizeof(u16));
    memset(data_in, 0, 8*sizeof(u16));


    nents = sg_nents_for_len(pSG_st, u32_miu_chksum_nbytes);
    if (nents < 0)
        return -EINVAL;

    sg_miter_start(&miter, pSG_st, nents, SG_MITER_ATOMIC | SG_MITER_FROM_SG);
    sg_miter_next(&miter);

    while(u32_miu_chksum_nbytes > 0) {
        dmalen = (miter.length > u32_miu_chksum_nbytes) ? u32_miu_chksum_nbytes:miter.length;
        u16_pBuf = (u16*)miter.addr;
        if (u16_pBuf == NULL)
        {
            eMMC_debug(eMMC_DEBUG_LEVEL_ERROR, 1,"miter addr=NULL\n");
        }

        for (i = 0; i < dmalen / sizeof(u16); i++) {
    
            data_in[i%8] = u16_pBuf[i];
    
            old_checksum[i%8] = checksum[i%8];
    
            if (i % 8 != 0)
                checksum[i%8] = data_in[i%8] ^ (((old_checksum[i%8] << 1) & 0xFFFF) | (old_checksum[(i-1)%8] >> 15));
            else
                checksum[i%8] = data_in[i%8] ^ (((old_checksum[i%8] << 1) & 0xFFFF) | (checksum[7] >> 15));
        }

        u32_miu_chksum_nbytes -= dmalen;
        sg_miter_next(&miter);
    }
    sg_miter_stop(&miter);

    for (i = 0; i < 8; i++) {
        REG_FCIE_CLRBIT(GET_REG_ADDR(FCIE1_BASE, 0x51), BIT12|BIT11|BIT10|BIT9|BIT8);
        REG_FCIE_SETBIT(GET_REG_ADDR(FCIE1_BASE, 0x51), (0x10+i)<<8);

        if (checksum[i] != REG_FCIE(FCIE_EMMC_DEBUG_BUS0))
        {
            eMMC_debug(eMMC_DEBUG_LEVEL_ERROR, 1, "\n\nFCIE's MIU checksum is not the same with CPU!\n\n");
            eMMC_debug(eMMC_DEBUG_LEVEL_ERROR, 1, "\n\nFCIE's MIU checksum:0x%x, CPU checksum:0x%x, I:%d\n\n",
                 REG_FCIE(FCIE_EMMC_DEBUG_BUS0) , checksum[i], i);
        }
    }

    return eMMC_ST_SUCCESS;
}


#endif

void eMMC_record_WR_time(U8 u8_CmdIdx, U32 u32_DataByteCnt)
{
    U32 u32_per_read_time = 0, u32_per_write_time =0;

    if(gu32_eMMC_monitor_enable)
    {
       if((u8_CmdIdx==17)||(u8_CmdIdx==18)) {
           u32_per_read_time = (U32)(ktime_to_us(ktime_sub(ktime_get(), starttime)));
           emmc_rw_speed.s64_total_read_bytes += u32_DataByteCnt;
           emmc_rw_speed.s64_total_read_time_usec += u32_per_read_time;

       }
       else if((u8_CmdIdx==24)||(u8_CmdIdx==25)) {
           u32_per_write_time = (U32)(ktime_to_us(ktime_sub(ktime_get(), starttime)));
           emmc_rw_speed.s64_total_write_bytes += u32_DataByteCnt;
           emmc_rw_speed.s64_total_write_time_usec += u32_per_write_time;
       }
    }
}

void eMMC_Check_Life(U32 u32_DataByteCnt)
{
    struct timeval time_st;
    struct tm time_tmp ={0};//fix compile warring

    if ((emmc_rw_speed.s64_day_write_bytes + u32_DataByteCnt) >0)
        emmc_rw_speed.s64_day_write_bytes += u32_DataByteCnt;

    do_gettimeofday(&time_st);
    time_to_tm(time_st.tv_sec, 0, &time_tmp);

    if (emmc_rw_speed.int_tm_yday != time_tmp.tm_yday) {
        if (emmc_rw_speed.s64_day_write_bytes >= eMMC_DAY_WRITE_WARN)
            eMMC_debug(eMMC_DEBUG_LEVEL_ERROR, 0, "eMMC Err: Warn, w %lld MB\n",
                emmc_rw_speed.s64_day_write_bytes>>20);

        emmc_rw_speed.s64_day_write_bytes = 0;
    }

    emmc_rw_speed.int_tm_yday = time_tmp.tm_yday;

}

#if 0
static ssize_t fcie_pwrsvr_gpio_trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gu32_pwrsvr_gpio_trigger);
}

static ssize_t fcie_pwrsvr_gpio_trigger_store(struct device *dev, struct device_attribute *attr,
                                              const char *buf, size_t count)
{
    unsigned long u32_pwrsvr_gpio_trigger = 0;

    if(kstrtoul(buf, 0, &u32_pwrsvr_gpio_trigger))
        return -EINVAL;

    if(u32_pwrsvr_gpio_trigger > 1)
        return -EINVAL;

    gu32_pwrsvr_gpio_trigger = u32_pwrsvr_gpio_trigger;

    return count;
}

DEVICE_ATTR(fcie_pwrsvr_gpio_trigger,
            S_IRUSR | S_IWUSR,
            fcie_pwrsvr_gpio_trigger_show,
            fcie_pwrsvr_gpio_trigger_store);

static ssize_t fcie_pwrsvr_gpio_bit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gu32_pwrsvr_gpio_bit);
}

static ssize_t fcie_pwrsvr_gpio_bit_store(struct device *dev, struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    unsigned long u32_pwrsvr_gpio_bit = 0;

    if(kstrtoul(buf, 0, &u32_pwrsvr_gpio_bit))
        return -EINVAL;

    if(u32_pwrsvr_gpio_bit)
    {
        if(u32_pwrsvr_gpio_bit > 0xF)
            return -EINVAL;
        gu32_pwrsvr_gpio_bit = u32_pwrsvr_gpio_bit;
    }

    return count;
}

DEVICE_ATTR(fcie_pwrsvr_gpio_bit,
            S_IRUSR | S_IWUSR,
            fcie_pwrsvr_gpio_bit_show,
            fcie_pwrsvr_gpio_bit_store);

static ssize_t fcie_pwrsvr_gpio_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0x%X\n", gu32_pwrsvr_gpio_addr);
}

static ssize_t fcie_pwrsvr_gpio_addr_store(struct device *dev, struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    unsigned long u32_pwrsvr_gpio_addr = 0;

    if(kstrtoul(buf, 0, &u32_pwrsvr_gpio_addr))
        return -EINVAL;

    if(u32_pwrsvr_gpio_addr)
        gu32_pwrsvr_gpio_addr = u32_pwrsvr_gpio_addr;

    return count;
}

DEVICE_ATTR(fcie_pwrsvr_gpio_addr,
            S_IRUSR | S_IWUSR,
            fcie_pwrsvr_gpio_addr_show,
            fcie_pwrsvr_gpio_addr_store);

static ssize_t fcie_pwrsvr_gpio_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gu32_pwrsvr_gpio_enable);
}

static ssize_t fcie_pwrsvr_gpio_enable_store(struct device *dev, struct device_attribute *attr,
                                             const char *buf, size_t count)
{
    unsigned long u32_pwrsvr_gpio_enable = 0;

    #if 0
    if(u8_enable_sar5)
        return count;
    #endif

    if(kstrtoul(buf, 0, &u32_pwrsvr_gpio_enable))
        return -EINVAL;

    if(u32_pwrsvr_gpio_enable)
        gu32_pwrsvr_gpio_enable = u32_pwrsvr_gpio_enable;

    return count;
}

DEVICE_ATTR(fcie_pwrsvr_gpio_enable,
            S_IRUSR | S_IWUSR,
            fcie_pwrsvr_gpio_enable_show,
            fcie_pwrsvr_gpio_enable_store);


static ssize_t emmc_sanitize_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gu32_emmc_sanitize);
}

static ssize_t emmc_sanitize_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    unsigned long u32_temp = 0;

    if(kstrtoul(buf, 0, &u32_temp))
        return -EINVAL;

    gu32_emmc_sanitize = u32_temp;
    //printk("%Xh\n", gu32_emmc_sanitize);

    if(gu32_emmc_sanitize)
    {
        eMMC_LockFCIE((U8*)__FUNCTION__);
        eMMC_debug(eMMC_DEBUG_LEVEL,0,"eMMC: santizing ...\n");
        eMMC_Sanitize(0xAA);
        eMMC_debug(eMMC_DEBUG_LEVEL,0,"eMMC: done\n");
        eMMC_UnlockFCIE((U8*)__FUNCTION__);
    }

    return count;
}

DEVICE_ATTR(emmc_sanitize,
            S_IRUSR | S_IWUSR,
            emmc_sanitize_show,
            emmc_sanitize_store);
#endif

static ssize_t eMMC_monitor_count_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    U32 u32_speed_read=0, u32_speed_write=0;
    U32 u32_multi, u32_total_read_GB, u32_total_write_GB, u32_total_read_MB, u32_total_write_MB;
    if (emmc_rw_speed.s64_total_read_time_usec > 1000) {
        u32_multi = emmc_rw_speed.s64_total_read_bytes/(emmc_rw_speed.s64_total_read_time_usec/1000);
        u32_speed_read = u32_multi*1000/(1024*1024);//MB/sec
    }
    if (emmc_rw_speed.s64_total_write_time_usec > 1000) {
        u32_multi = emmc_rw_speed.s64_total_write_bytes/(emmc_rw_speed.s64_total_write_time_usec/1000);
        u32_speed_write = u32_multi*1000/(1024*1024);//MB/sec
    }
    u32_total_read_GB = emmc_rw_speed.s64_total_read_bytes/(1024*1024*1024);
    u32_total_read_MB = (emmc_rw_speed.s64_total_read_bytes%(1024*1024*1024))/(1024*1024);

    u32_total_write_GB = emmc_rw_speed.s64_total_write_bytes/(1024*1024*1024);
    u32_total_write_MB = (emmc_rw_speed.s64_total_write_bytes%(1024*1024*1024))/(1024*1024);

    return scnprintf(buf, PAGE_SIZE, "eMMC: read total %dGB %dMB, %uMB/sec\n"
        "eMMC: write total %dGB %dMB, %uMB/sec\n""%xh\n",
        u32_total_read_GB, u32_total_read_MB, u32_speed_read,
        u32_total_write_GB, u32_total_write_MB, u32_speed_write, gu32_eMMC_monitor_enable);
}



static ssize_t eMMC_monitor_count_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
    unsigned long u32_temp = 0;


    if(kstrtoul(buf, 0, &u32_temp))
        return -EINVAL;

    gu32_eMMC_monitor_enable = u32_temp;

    if (gu32_eMMC_monitor_enable == 0) {
        emmc_rw_speed.s64_total_read_bytes = 0;
        emmc_rw_speed.s64_total_read_time_usec = 0;
        emmc_rw_speed.s64_total_write_bytes = 0;
        emmc_rw_speed.s64_total_write_time_usec =0;  
    }
    return count;
}

DEVICE_ATTR(eMMC_monitor_count_enable,
            S_IRUSR | S_IWUSR,
            eMMC_monitor_count_enable_show,
            eMMC_monitor_count_enable_store);

static ssize_t emmc_write_log_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", gu32_eMMC_write_log_enable);
}

static ssize_t eMMC_write_log_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
    unsigned long u32_temp = 0;

    if(kstrtoul(buf, 0, &u32_temp))
        return -EINVAL;

    gu32_eMMC_write_log_enable = u32_temp;

    if(gu32_eMMC_write_log_enable)
    {
        eMMC_debug(eMMC_DEBUG_LEVEL,0,"eMMC: write log enable\n");
    }

    return count;
}

DEVICE_ATTR(eMMC_write_log_enable,
            S_IRUSR | S_IWUSR,
            emmc_write_log_show,
            eMMC_write_log_store);

static ssize_t emmc_read_log_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", gu32_eMMC_read_log_enable);
}

static ssize_t eMMC_read_log_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
    unsigned long u32_temp = 0;

    if(kstrtoul(buf, 0, &u32_temp))
        return -EINVAL;

    gu32_eMMC_read_log_enable = u32_temp;

    if(gu32_eMMC_read_log_enable)
    {
        eMMC_debug(eMMC_DEBUG_LEVEL,0,"eMMC: read log enable\n");
    }

    return count;
}

DEVICE_ATTR(eMMC_read_log_enable,
            S_IRUSR | S_IWUSR,
            emmc_read_log_show,
            eMMC_read_log_store);

static ssize_t emmc_sar5_status_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "SAR5: %u\n", u8_enable_sar5);
}

DEVICE_ATTR(eMMC_sar5_status,
            S_IRUSR,
            emmc_sar5_status_show,
            NULL);


static ssize_t emmc_write_count_day_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int total_MB = (int)(emmc_rw_speed.s64_day_write_bytes >> 20);


    return scnprintf(buf, PAGE_SIZE, "%uMB\n", total_MB);
}

DEVICE_ATTR(eMMC_write_count_day,
            S_IRUSR,
            emmc_write_count_day_show,
            NULL);


struct attribute *mstar_mci_attr[] =
{
    #if 0
    &dev_attr_fcie_pwrsvr_gpio_enable.attr,
    &dev_attr_fcie_pwrsvr_gpio_addr.attr,
    &dev_attr_fcie_pwrsvr_gpio_bit.attr,
    &dev_attr_fcie_pwrsvr_gpio_trigger.attr,
    &dev_attr_emmc_sanitize.attr,
    #endif
    &dev_attr_eMMC_read_log_enable.attr,
    &dev_attr_eMMC_write_log_enable.attr,
    &dev_attr_eMMC_monitor_count_enable.attr,
    &dev_attr_eMMC_sar5_status.attr,
    &dev_attr_eMMC_write_count_day.attr,
    NULL,
};


