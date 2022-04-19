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


#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/version.h>
#include "mdrv_mspi.h"
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/dma-mapping.h>
/*
 * CONFIG_DEBUG_MSPI:
 *      1:map MAXIM pins of mspi to  reg_ld_spi3_config
 *      0:require pins mapping at sboot/mboot stage.
 */
#define CONFIG_DEBUG_MSPI 0

/* SPI register offsets */
#define MSPI_WD0_1                      0x40
#define MSPI_WD2_3                      0x41
#define MSPI_WD4_5                      0x42
#define MSPI_WD6_7                      0x43
#define MSPI_RD0_1                      0x78
#define MSPI_RD2_3                      0x79
#define MSPI_RD4_5                      0x7a
#define MSPI_RD6_7                      0x7b
#define MSPI_WBF_RBF_SIZE               0x48
#define MSPI_WBF_RBF_SIZE_MAX           8
#define MSPI_CTRL_CLOCK_RATE            0x49
#define MSPI_ENABLE_BIT                 BIT(0)
#define MSPI_RESET_BIT                  BIT(1)
#define MSPI_ENABLE_INT_BIT             BIT(2)
#define MSPI_3WARE_MODE_BIT             BIT(4)
#define MSPI_CPHA_BIT                   BIT(6)
#define MSPI_CPOL_BIT                   BIT(7)
#define MSPI_TR_START_END_TIME          0x4a
#define MSPI_TBYTE_INTERVAL_AROUND_TIME 0x4b
#define MSPI_WD0_3_BIT_SEL              0x4c
#define MSPI_WD4_7_BIT_SEL              0x4d
#define MSPI_RD0_3_BIT_SEL              0x4e
#define MSPI_RD4_7_BIT_SEL              0x4f
#define MSPI_LSB_FIRST                  0x50
#define MSPI_LSB_FIRST_BIT              BIT(0)
#define MSPI_TRIGGER                    0x5a
#define MSPI_TRIGGER_BIT                BIT(0)
#define MSPI_DONE_FLAG                  0x5b
#define MSPI_CLEAR_DONE_FLAG            0x5c
#define MSPI_CLEAR_DONE_FLAG_BIT        BIT(0)
#define MSPI_CHIP_SELECT                0x5f
#define MSPI_CHIP_SELECT_BIT            BIT(0)


#define MSTAR_SPI_TIMEOUT_MS    10000
#define MSTAR_SPI_MODE_BITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_NO_CS | SPI_LSB_FIRST)

#define DMA_INT_EN_REG  0x3

#define DRV_NAME    "mspi-mtk"

#define INT_ENABLE_REG  0x13
#define INT_EN_BIT      BIT(1)
#define INT_CLR_REG     0x11
#define INT_CLR_FLG     BIT(2)

#define SPI_SW_MODE_REG 0xa
#define SPI_SW_CS_BIT   BIT(1)
#define SPI_SW_CS_EN    BIT(0)

#define FSP_HW_CTRL     0x6c
#define FSP_HW_TRIG     0x6d

#define FSP_WD0_1       0x60
#define FSP_WD2_3       0x61
#define FSP_WD4_5       0x62
#define FSP_WD6_7       0x63
#define FSP_RD0_1       0x65
#define FSP_RD2_3       0x66

#define FSP_ENABLE_BIT  BIT(0)
#define FSP_RESET_BIT   BIT(1)

#define DMA_TRIG_REG    0x10
#define DMA_TRIG_FLAG   BIT(0)
#define DMA_DONE_STS    BIT(3)

#define DMA_STS_REG     0x11
#define DMA_PATH_REG    0x12
#define DMA_SRC_ADDR_L  0x14
#define DMA_SRC_ADDR_H  0x15
#define DMA_DST_ADDR_L  0x16
#define DMA_DST_ADDR_H  0x17
#define DMA_SIZE_L      0x18
#define DMA_SIZE_H      0x19

#define FSP_WBSZ_REG    0x6a
#define FSP_RBSZ_REG    0x6b
#define FSP_TRIG_REG    0x6d
#define FSP_STS_REG     0x6e
#define FSP_CLR_FLAG_REG    0x6f
#define FSP_OUTBUF_REG  0x78
#define FSP_WBF_CTRL_REG    0x79
#define CMD_RD   0x03
#define CMD_WR   0x02
#define PAGE_READ_SIZE 256
#define CMD_SIZE  4

static volatile unsigned int xfer_flag;

struct mtk_spi {
    u64 regs;
    u32 mspi_channel;
    struct clk *clk;
    int irq;
    struct completion done;
    const u8 *tx_buf;
    u8 *rx_buf;
    int len;
    int current_trans_len;
    struct device *dev;
    wait_queue_head_t wait;
    unsigned int flash_addr;
    u8 device_cmd_flag;
    u8 device_busy;
    u8 tx_xfer_flag;
    u8 rx_done;
    u8 tx_done;
};

struct mtk_spi_data {
    u32 regs;
    u32 irq;
    u32 mspi_channel;
};

static struct task_struct *t_xfer_tsk;
static int t_spixfer_thread(void* arg);
static u8 *xferbuf;
#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define REG_ADDR(addr)  (*((volatile u16 *)((mstar_pm_base + (addr )))))
#define BASEREG_ADDR(addr)  ((mstar_pm_base + (addr )))
#else
extern ptrdiff_t mstar_pm_base;
#define REG_ADDR(addr)  (*((volatile u16 *)((mstar_pm_base + (addr )))))
#define BASEREG_ADDR(addr)  (mstar_pm_base + (addr ))
#endif
// read 2 byte
#define MSPI_READ(_reg_)          (REG_ADDR(bs->regs + ((_reg_)<<2)))

// write 2 byte
#define MSPI_WRITE(_reg_, _val_)    \
        do{ REG_ADDR(bs->regs + ((_reg_)<<2)) =(_val_) ; }while(0)

//0x111Dh FSP Demura
#define FSP_READ(_reg_)          (REG_ADDR(0x223A00 + ((_reg_)<<2)))

#define FSP_WRITE(_reg_, _val_)    \
        do{ REG_ADDR(0x223A00 + ((_reg_)<<2)) =(_val_) ; }while(0)

//0x153Fh QSPI Demura
#define QSPI_READ(_reg_)          (REG_ADDR(0x2A7E00 + ((_reg_)<<2)))

#define QSPI_WRITE(_reg_, _val_)    \
        do{ REG_ADDR(0x2A7E00 + ((_reg_)<<2)) =(_val_) ; }while(0)

static inline u16 mtk_rd(struct mtk_spi *bs, u32 reg)
{
    return MSPI_READ(reg);
}
static inline u8 mtk_rdh(struct mtk_spi *bs, u32 reg)
{
    return mtk_rd(bs,reg)>>8;
}
static inline u8 mtk_rdl(struct mtk_spi *bs, u16 reg)
{
    return mtk_rd(bs,reg)&0xff;
}
static inline void mtk_wr(struct mtk_spi *bs, u16 reg, u32 val)
{
    MSPI_WRITE(reg,val);
}
static inline void mtk_wrh(struct mtk_spi *bs, u16 reg, u8 val)
{
    u16 val16 = mtk_rd(bs,reg)&0xff;
    val16 |= ((u16)val)<<8;
    mtk_wr(bs,reg,val16);
}
static inline void mtk_wrl(struct mtk_spi *bs, u16 reg, u8 val)
{
    u16 val16 = mtk_rd(bs,reg)&0xff00;
    val16 |= val;
    mtk_wr(bs,reg,val16);
}

static inline u16 mtk_fsp_rd(struct mtk_spi *bs, u32 reg)
{
    return FSP_READ(reg);
}

static inline u8 mtk_fsp_rdh(struct mtk_spi *bs, u32 reg)
{
    return mtk_fsp_rd(bs,reg)>>8;
}
static inline u8 mtk_fsp_rdl(struct mtk_spi *bs, u16 reg)
{
    return mtk_fsp_rd(bs,reg)&0xff;
}
static inline void mtk_fsp_wr(struct mtk_spi *bs, u16 reg, u32 val)
{
    FSP_WRITE(reg,val);
}
static inline void mtk_fsp_wrh(struct mtk_spi *bs, u16 reg, u8 val)
{
    u16 val16 = mtk_fsp_rd(bs,reg)&0xff;
    val16 |= ((u16)val)<<8;
    mtk_fsp_wr(bs,reg,val16);
}
static inline void mtk_fsp_wrl(struct mtk_spi *bs, u16 reg, u8 val)
{
    u16 val16 = mtk_fsp_rd(bs,reg)&0xff00;
    val16 |= val;
    mtk_fsp_wr(bs,reg,val16);
}

static const u16 mspi_txfifoaddr[] = {
    MSPI_WD0_1,
    MSPI_WD2_3,
    MSPI_WD4_5,
    MSPI_WD6_7,
};

static void mtk_hw_set_clock(struct mtk_spi *bs,struct spi_device *spi,struct spi_transfer *tfr)
{

}
static void mtk_hw_enable_interrupt(struct mtk_spi *bs,bool enable)
{
    u8 val = mtk_rdl(bs,INT_ENABLE_REG);
    if (enable){
        val |= INT_EN_BIT;
    }
    else{
        val &= ~INT_EN_BIT;
    }
    mtk_wrl(bs,INT_ENABLE_REG,val);
}

static inline void mtk_hw_chip_select(struct mtk_spi *bs,struct spi_device *spi,bool enable)
{
    u8 val;
    if (spi->mode&SPI_NO_CS){
        return ;
    }
    val = mtk_rdl(bs, SPI_SW_MODE_REG);
    if (enable != (!!(spi->mode&SPI_CS_HIGH))){
        val &= ~SPI_SW_CS_BIT;
    }else{
        val |= SPI_SW_CS_BIT;
    }

    val |= SPI_SW_CS_EN;
    mtk_wrl(bs,SPI_SW_MODE_REG,val);
}
static inline void mtk_hw_clear_done(struct mtk_spi *bs)
{
    mtk_wrl(bs,INT_CLR_REG,INT_CLR_FLG);
    mtk_wrl(bs,INT_CLR_REG,DMA_DONE_STS);
}
static inline void mtk_hw_enable(struct mtk_spi *bs,bool enable)
{
    u8 val;
    val = mtk_fsp_rdl(bs,FSP_HW_CTRL);
    if (enable){
        val |= FSP_ENABLE_BIT;
        val |= FSP_RESET_BIT;
    }else{
        val &= ~FSP_ENABLE_BIT;
        val &= ~FSP_RESET_BIT;
    }
    mtk_wrl(bs,FSP_HW_CTRL,val);

}
static inline void mtk_hw_transfer_trigger(struct mtk_spi *bs)
{
    mtk_wr(bs,DMA_TRIG_REG,DMA_TRIG_FLAG);
}
static void mtk_hw_txdummy(struct mtk_spi *bs,u8 len)
{
    int cnt;
    for (cnt = 0; cnt < len>>1;cnt++)
    {
        mtk_wr(bs,mspi_txfifoaddr[cnt],0xffff);
    }
    if (len&1)
    {
        mtk_wrl(bs,mspi_txfifoaddr[cnt],0xff);
    }
    mtk_wrl(bs,MSPI_WBF_RBF_SIZE,len);
}
static void mtk_hw_txcmdfill(struct mtk_spi *bs,const u8*buffer,u8 len)
{
    u8 sts;
    u8 tx_dat;

    tx_dat = (bs->tx_buf)[0];

    if(tx_dat != 0x03)
    {
       mtk_fsp_wr(bs,FSP_WBSZ_REG,0);
       mtk_fsp_wr(bs,FSP_RBSZ_REG,0);

       //Flash Write Enable
       mtk_fsp_wrl(bs,FSP_WD0_1,0x06);


       //Write Enable command size is one byte
       mtk_fsp_wrl(bs,FSP_WBSZ_REG,1);
       mtk_fsp_wrl(bs,FSP_HW_CTRL,3);
       mtk_fsp_wrh(bs,FSP_HW_CTRL,0);
       //FSP trig
       mtk_fsp_wrl(bs,FSP_TRIG_REG,1);
       udelay(5);
       sts = mtk_fsp_rdl(bs,FSP_STS_REG);
       mtk_fsp_wrl(bs,FSP_CLR_FLAG_REG,1);



        mtk_fsp_wrl(bs,FSP_OUTBUF_REG,(len));

        /*1 : wbf1 = reg_fsp_wbf_size_outside + reg_fsp_wbf_size1 wbf0 for Write Enable*/
        /*4 :  outside wdata cmd+addr=4*/
        mtk_fsp_wr(bs,FSP_WBF_CTRL_REG,0x1004);
        mtk_fsp_wrl(bs,FSP_WD0_1,(bs->tx_buf)[0]);
        mtk_fsp_wrh(bs,FSP_WD0_1,(bs->tx_buf)[1]);
        mtk_fsp_wrl(bs,FSP_WD2_3,(bs->tx_buf)[2]);
        mtk_fsp_wrh(bs,FSP_WD2_3,(bs->tx_buf)[3]);
        //Write 3Byter address +1 command+dummy byte
        mtk_fsp_wrl(bs,FSP_WBSZ_REG,5);
        mtk_fsp_wrl(bs,FSP_TRIG_REG,1);
    }
    bs->tx_xfer_flag = 1 ;
    bs->flash_addr = (((bs->tx_buf)[1])<<16)|(((bs->tx_buf)[2])<<8)|(((bs->tx_buf)[3]));
    bs->device_cmd_flag = 1;
}
static void mtk_hw_deviceRd(struct mtk_spi *bs,const u8*buffer,u8 len)
{
    u8 sts;
    u8 tx_dat;
    u8 rx_dat[4];


    tx_dat = (bs->tx_buf)[0];
    mtk_fsp_wr(bs,FSP_WBSZ_REG,0);
    mtk_fsp_wr(bs,FSP_RBSZ_REG,0);

    mtk_fsp_wrl(bs,FSP_WD0_1,tx_dat);
    //Write Enable command size is one byte
    mtk_fsp_wrl(bs,FSP_WBSZ_REG,1);
    mtk_fsp_wrl(bs,FSP_HW_CTRL,3);
    mtk_fsp_wrh(bs,FSP_HW_CTRL,0);
    mtk_fsp_wr(bs,FSP_RBSZ_REG,(len));

    mtk_fsp_wrl(bs,FSP_TRIG_REG,1);

    udelay(10);
    sts = mtk_fsp_rdl(bs,FSP_STS_REG);
    mtk_fsp_wrl(bs,FSP_CLR_FLAG_REG,1);
    rx_dat[0] = mtk_fsp_rdl(bs,FSP_RD0_1);
    rx_dat[1] = mtk_fsp_rdh(bs,FSP_RD0_1);
    rx_dat[2] = (u8)(mtk_fsp_rd(bs,FSP_RD2_3));
    rx_dat[3] = (u8)(mtk_fsp_rd(bs,FSP_RD2_3)>>8);

    memcpy(buffer,rx_dat,(len));
}

static void mtk_hw_txdatdma(struct mtk_spi *bs,const u8*buffer,u32 len)
{
    unsigned int paddr;

    memcpy(xferbuf,buffer,(len));
    paddr = virt_to_phys((unsigned int)xferbuf);

    paddr -=0x20000000;
    //Set source and destination path
    mtk_wr(bs,DMA_PATH_REG, 0x2C40);

    // Set start address
    mtk_wr(bs,DMA_SRC_ADDR_L, (paddr&0x0000FFFF));
    mtk_wr(bs,DMA_SRC_ADDR_H, (paddr >> 16));

    // Set end address
    mtk_wr(bs,DMA_DST_ADDR_L, ((bs->flash_addr)&0x0000FFFF));
    mtk_wr(bs,DMA_DST_ADDR_H, ((bs->flash_addr) >> 16));

    // Set Size
    mtk_wr(bs,DMA_SIZE_L, (len&0x0000FFFF));
    mtk_wr(bs,DMA_SIZE_H, (len >> 16));

    mtk_hw_transfer_trigger(bs);
}

static void mtk_hw_rxdatdma(struct mtk_spi *bs,const u8*buffer,u32 len)
{
    unsigned int paddr;


    paddr = virt_to_phys((unsigned int)xferbuf);

    paddr -=0x20000000;
    //Set source and destination path
    mtk_wr(bs,DMA_PATH_REG, 0x404B);

    // Set start address
    mtk_wr(bs,DMA_SRC_ADDR_L, ((bs->flash_addr)&0x0000FFFF));
    mtk_wr(bs,DMA_SRC_ADDR_H, ((bs->flash_addr) >> 16));

    // Set end address
    mtk_wr(bs,DMA_DST_ADDR_L, (paddr&0x0000FFFF));
    mtk_wr(bs,DMA_DST_ADDR_H, (paddr >> 16));

    // Set Size
    mtk_wr(bs,DMA_SIZE_L, (len &0x0000FFFF));
    mtk_wr(bs,DMA_SIZE_H, (len >> 16));
    mtk_hw_transfer_trigger(bs);
}

static mtk_hw_sts_check(struct mtk_spi *bs)
{
    u8 sts;

    if(bs->tx_xfer_flag ==1)
    {
       udelay(5);
       sts = mtk_fsp_rdl(bs,FSP_STS_REG);

       if(sts)
       {
            mtk_fsp_wr(bs,FSP_WBSZ_REG,0);
            mtk_fsp_wr(bs,FSP_RBSZ_REG,0);
            mtk_fsp_wr(bs,FSP_WBF_CTRL_REG,0x0);
            mtk_fsp_wrl(bs,FSP_CLR_FLAG_REG,1);
       }
       mtk_fsp_wr(bs,FSP_WBSZ_REG,0);
       mtk_fsp_wrl(bs,FSP_HW_CTRL,3);
       mtk_fsp_wr(bs,FSP_RBSZ_REG,1);
       mtk_fsp_wrl(bs,FSP_WD0_1,0x05);
       //Write Enable command size is one byte
       mtk_fsp_wrl(bs,FSP_WBSZ_REG,1);
       mtk_fsp_wrh(bs,FSP_HW_CTRL,0);

       mtk_fsp_wrl(bs,FSP_TRIG_REG,1);

       udelay(5);
       sts = mtk_fsp_rdl(bs,FSP_STS_REG);
       mtk_fsp_wrl(bs,FSP_CLR_FLAG_REG,1);
       bs->device_busy = mtk_fsp_rdl(bs,FSP_RD0_1)&0x1;
    }

    if(bs->device_busy)
    {
        wake_up(&bs->wait);
    }
    else
    {
        //bs->flash_addr = 0;
        xfer_flag = 0;
        bs->tx_xfer_flag = 0;
        bs->rx_buf += bs->current_trans_len;
        bs->flash_addr += bs->current_trans_len;
        bs->tx_done = 0;
    }
}

static void mtk_spi_hw_receive(struct mtk_spi *bs)
{
    if (bs->rx_buf != NULL)
    {
        xfer_flag = 1 ;
        bs->rx_done = 1;
        wake_up(&bs->wait);
    }
}
static void mtk_spi_hw_transfer(struct mtk_spi *bs)
{

    if(bs->rx_done == 0){

    if (bs->tx_buf != NULL){
        xfer_flag = 1 ;
        bs->tx_done = 1;
        wake_up(&bs->wait);
    }else{
        mtk_hw_txdummy(bs,0);
    }
    }
}

static irqreturn_t mtk_spi_interrupt(int irq, void *dev_id)
{
    struct spi_master *master = dev_id;
    struct mtk_spi *bs = spi_master_get_devdata(master);

    if((mtk_rdl(bs,DMA_STS_REG)&INT_CLR_FLG)){
        mtk_hw_clear_done(bs);

        if (bs->current_trans_len != 0){
            mtk_spi_hw_receive(bs);
        }else{
            return IRQ_NONE;
        }
        if (bs->len != 0){
           mtk_spi_hw_transfer(bs);
        }
        else{
            bs->current_trans_len = 0;
            bs->device_cmd_flag = 0;
            complete(&bs->done);
        }
        return IRQ_HANDLED;
    }
    dev_err(&master->dev, "Error:incorrect irq num!\n");
    mtk_hw_clear_done(bs);
    return IRQ_NONE;
}

static int mtk_spi_start_transfer(struct spi_device *spi,
        struct spi_transfer *tfr)
{
    struct mtk_spi *bs = spi_master_get_devdata(spi->master);

    /*
    *   Setup mspi clock for this transfer.
    */
    mtk_hw_set_clock(bs,spi,tfr);

    /*
    * Enable SPI master controller&&Interrupt.
    */

    mtk_hw_enable(bs,true);
    mtk_hw_clear_done(bs);
    mtk_hw_enable_interrupt(bs,true);

    /*
    *   Setup mspi chip select for this transfer.
    */
    mtk_hw_chip_select(bs,spi,true);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 1)
    INIT_COMPLETION(bs->done);
#else
    reinit_completion(&bs->done);
#endif
    bs->tx_buf = tfr->tx_buf;
    bs->rx_buf = tfr->rx_buf;
    bs->len = tfr->len;


    if((bs->len)>(64*1024))
    {
        printk("!!!xfer over size 64K\n",bs->len);
        return -EINVAL;
    }
    /*
    *   Start transfer loop.
    */

    //mtk_spi_hw_transfer(bs);
    xfer_flag = 1 ;

    /*patch for add_timer issue*/
    t_xfer_tsk = kthread_create(t_spixfer_thread, bs, "SPI_Xfer");
    kthread_bind(t_xfer_tsk, 0);
    if (IS_ERR(t_xfer_tsk)) {
            printk("create kthread for spi xfer observation fail\n");
                t_xfer_tsk = NULL;
    }else
    wake_up_process(t_xfer_tsk);
    /*patch for add_timer issue*/


    return 0;
}

static int mtk_spi_finish_transfer(struct spi_device *spi,
        struct spi_transfer *tfr, bool cs_change)
{
    struct mtk_spi *bs = spi_master_get_devdata(spi->master);

    if (tfr->delay_usecs)
        udelay(tfr->delay_usecs);

    if (cs_change){
        /*
        * Cancel chip select.
        */
        mtk_hw_chip_select(bs,spi,false);
    }

    return 0;
}

static int mtk_spi_transfer_one(struct spi_master *master,
        struct spi_message *mesg)
{
    struct mtk_spi *bs = spi_master_get_devdata(master);
    struct spi_transfer *tfr;
    struct spi_device *spi = mesg->spi;
    int err = 0;
    unsigned int timeout;
    bool cs_change;

    list_for_each_entry(tfr, &mesg->transfers, transfer_list) {

        err = mtk_spi_start_transfer(spi, tfr);

        if (err)
            goto out;

        timeout = wait_for_completion_timeout(&bs->done,
                msecs_to_jiffies(MSTAR_SPI_TIMEOUT_MS));
        if (!timeout) {
            printk("mtk_spi_transfer_one !!! -ETIMEDOUT\n");
            err = -ETIMEDOUT;
            goto out;
        }

        cs_change = tfr->cs_change ||
            list_is_last(&tfr->transfer_list, &mesg->transfers);
        err = mtk_spi_finish_transfer(spi, tfr, cs_change);
        if (err)
            goto out;

        mesg->actual_length += (tfr->len - bs->len);
    }

out:

    mesg->status = err;
    spi_finalize_current_message(master);

    return 0;
}
static const struct of_device_id mtk_mspi_match[] = {
    { .compatible = "mtk,mtk-qspi", },
    {}
};
MODULE_DEVICE_TABLE(of, mtk_mspi_match);

static int t_spixfer_thread(void* arg)
{
   struct mtk_spi *spixfer = arg;
   volatile unsigned int len = spixfer->len;

   while(1)
   {
       len = spixfer->len;
       wait_event_freezable(spixfer->wait, (xfer_flag) == 1);/*Reduce CPU Loading*/

       if(spixfer->rx_buf == NULL)
       {
           /*Flash ID read*/
           if(spixfer->tx_done)
           {
               mtk_hw_sts_check(spixfer);
               if(len > 0)
               {
                     spixfer->len -= spixfer->current_trans_len;
                     len = spixfer->len;
               }
           }
           if(len == 0)
           {
               mtk_hw_enable_interrupt(spixfer,0);
               spixfer->device_cmd_flag = 0;
               xfer_flag = 0;
               complete(&spixfer->done);
               do_exit(0);
           }
           else
           {
               if((spixfer->tx_buf)[0] != CMD_WR)
               {
                  ;
               }
               else
               {
                  if (len >= PAGE_READ_SIZE){
                      len = PAGE_READ_SIZE;
                  }

                  spixfer->current_trans_len = len;
                  if(spixfer->device_cmd_flag == 0)
                  {
                     mtk_hw_txcmdfill(spixfer, spixfer->tx_buf, len);
                  }
                  mtk_hw_txdatdma(spixfer, spixfer->tx_buf, len);
                  xfer_flag = 0;
              }
           }
           /*DMA Trigger*/
           //mtk_hw_transfer_trigger(spixfer);
       }
       else
       {
           if(spixfer->rx_done)
           {
                memcpy(spixfer->rx_buf,xferbuf,(spixfer->current_trans_len));
                spixfer->rx_buf += spixfer->current_trans_len;
                spixfer->flash_addr += spixfer->current_trans_len;
                xfer_flag = 0;
                spixfer->rx_done = 0;
                if(len > 0)
                {
                     spixfer->len -= spixfer->current_trans_len;
                     len = spixfer->len;
                }
           }
           if(len == 0)
           {
               mtk_hw_enable_interrupt(spixfer,0);
               spixfer->device_cmd_flag = 0;
               spixfer->flash_addr = 0;
               spixfer->current_trans_len =0;
               xfer_flag = 0;
               complete(&spixfer->done);
               do_exit(0);
           }
           else
           {
               if((spixfer->tx_buf)[0] != CMD_RD)
               {
                   mtk_hw_deviceRd(spixfer,spixfer->rx_buf,len);
                   xfer_flag = 0;
                   complete(&spixfer->done);
                   do_exit(0);
               }
               else
              {
                  if (len >= 1*1024)
                  {
                      len = 1*1024;
                  }
                  else
                  {
                      if (spixfer->device_cmd_flag == 0)
                      {
                          spixfer->current_trans_len = len - CMD_SIZE ;
                      }
                  }
                  spixfer->current_trans_len = len;

                  if (spixfer->device_cmd_flag == 0)
                  {
                      mtk_hw_txcmdfill(spixfer, spixfer->tx_buf, len);
                      spixfer->rx_buf[0] = CMD_RD;
                      spixfer->rx_buf[1] = (u8)((spixfer->flash_addr & 0xFF0000) >> 16);
                      spixfer->rx_buf[2] = (u8)((spixfer->flash_addr & 0xFF00) >> 8);
                      spixfer->rx_buf[3] = (u8)(spixfer->flash_addr & 0xFF);
                      spixfer->rx_buf += 4;
                  }

                  xfer_flag = 0;
                  mtk_hw_rxdatdma(spixfer, spixfer->rx_buf , len);
              }
           }
       }
   }
}

static int mtk_spi_probe(struct platform_device *pdev)
{
    struct spi_master *master;
    struct mtk_spi *bs;
    int err = -ENODEV;

    master = spi_alloc_master(&pdev->dev, sizeof(*bs));
    if (!master) {
        dev_err(&pdev->dev, "spi_alloc_master() failed\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, master);

    master->mode_bits = MSTAR_SPI_MODE_BITS;
    master->bits_per_word_mask = BIT(8 - 1)|BIT(7 - 1)
                                |BIT(6 - 1)|BIT(5 - 1)
                                |BIT(4 - 1)|BIT(3 - 1)
                                |BIT(2 - 1)|BIT(1 - 1);
    master->bus_num = -1;
    master->num_chipselect = 1;
    master->transfer_one_message = mtk_spi_transfer_one;
    master->dev.of_node = pdev->dev.of_node;

    bs = spi_master_get_devdata(master);
    bs->dev = &pdev->dev;

    init_completion(&bs->done);

    xferbuf = kmalloc(64*1024,GFP_KERNEL);

    if(of_match_device(mtk_mspi_match,&pdev->dev)){
        u32 reg = 0;

        err = of_property_read_u32_index(pdev->dev.of_node, "reg",2,&reg);
        if (err){
            dev_err(&pdev->dev, "could not get resource reg\n");
            return -EINVAL;
        }
        bs->regs = reg;

        dev_err(&pdev->dev, "SPI master reg:%lx!!!!!!!!:\n",reg);

        err = of_property_read_u32(pdev->dev.of_node, "interrupts",&bs->irq);

        dev_err(&pdev->dev, "SPI master irq:%lx!!!!!!!!:\n",bs->irq);
        if (err){
            dev_err(&pdev->dev, "could not get resource intr\n");
            return -EINVAL;
        }
        err = of_property_read_u32(pdev->dev.of_node, "mspi_channel",&bs->mspi_channel);
        if (err){
            dev_err(&pdev->dev, "could not get resource channel\n");
            return -EINVAL;
        }
    }else{
        struct mtk_spi_data *data = dev_get_platdata(&pdev->dev);
        if (!data){
            dev_err(&pdev->dev, "could not get priv resource\n");
            return -EINVAL;
        }
        bs->regs = data->regs;
        bs->irq = data->irq;
        bs->mspi_channel = data->mspi_channel;
    }
    if (!bs->regs || !bs->irq) {
        dev_err(&pdev->dev, "could not get both resource\n");
        return -EINVAL;
    }

    err = devm_request_irq(&pdev->dev,bs->irq, mtk_spi_interrupt, 0,
            dev_name(&pdev->dev), master);
    if (err) {
        dev_err(&pdev->dev, "could not request IRQ: %d:%d\n", bs->irq,err);
        return err;
    }


   /*
    * FIXME:
    *   Setup mspi clock from clock tree.
    */
    {

    }


    err = spi_register_master(master);

    if (err) {
        dev_err(&pdev->dev, "could not register SPI master: %d\n", err);
        return err;
    }
    bs->device_cmd_flag = 0;
    xfer_flag = 0;
    init_waitqueue_head(&bs->wait);

    return 0;
}

static int mtk_spi_remove(struct platform_device *pdev)
{
    struct spi_master *master = platform_get_drvdata(pdev);
    struct mtk_spi *bs = spi_master_get_devdata(master);

    devm_free_irq(&pdev->dev,bs->irq, master);
    spi_unregister_master(master);

    spi_master_put(master);

    return 0;
}


static struct platform_driver mtk_spi_driver = {
    .driver     = {
        .name       = DRV_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = mtk_mspi_match,
    },
    .probe      = mtk_spi_probe,
    .remove     = mtk_spi_remove,
};

module_platform_driver(mtk_spi_driver);

MODULE_DESCRIPTION("QSPI controller driver for Demura");
MODULE_AUTHOR("Owen.Tseng <owen.tseng@mtksemi.com>");
MODULE_LICENSE("GPL v2");
