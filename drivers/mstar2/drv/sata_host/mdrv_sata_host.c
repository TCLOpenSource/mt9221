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
/// file    mdrv_sata_host.c
/// @brief  SATA Host Driver
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <linux/libata.h>
#include <asm/io.h>
#include <mdrv_sata_host.h>
#include <mhal_sata_host.h>
#include "chip_int.h"
#include "chip_setup.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif

extern ptrdiff_t mstar_pm_base;

#define SATA_DEBUG
#ifdef SATA_DEBUG
#define sata_debug(fmt, args...) printk("[%s] " fmt, __FUNCTION__, ##args)
#else
#define sata_debug(fmt, args...) do {} while(0)
#endif

struct prdte {
    u32 dba;
    u32 dbau;
    u32 rev1;

    u32 dbc:22;
    u32 rev2:9;
    u32 i:1;
};

#if 0
static void print_fis(hal_cmd_h2dfis *pfis)
{
    hal_cmd_h2dfis * p = pfis;

    printk("fis @ 0x%8.8x, fis size = 0x%x\n", (unsigned int)pfis, sizeof(hal_cmd_h2dfis));
    printk("fis phys @ 0x%8.8x\n", virt_to_phys(pfis));
    printk("fis type = 0x%x\n", p->u8fis_type);
    printk("PMP:4\t = 0x%x\n", p->u8MPM);
    printk("rev:3\t = 0x%x\n", p->reserved_0);
    printk("iscmd:1\t = 0x%x\n", p->isclear);
    printk("ata cmd\t = 0x%x\n", p->ata_cmd);
    printk("feat.\t = 0x%x\n", p->fearure);
    printk("lba_l\t = 0x%x\n", p->lba_l);
    printk("lba_m\t = 0x%x\n", p->lba_m);
    printk("lba_h\t = 0x%x\n", p->lba_h);
    printk("dev\t = 0x%x\n", p->device);
    printk("lba_l_e\t = 0x%x\n", p->lba_l_exp);
    printk("lba_m_e\t = 0x%x\n", p->lba_m_exp);
    printk("lba_h_e\t = 0x%x\n", p->lba_h_exp);
    printk("feat. ex = 0x%x\n", p->fearure_exp);
    printk("sector\t = 0x%x (u16)\n", p->u16sector_cnt);
    printk("rev\t = 0x%x\n", p->reserved_1);
    printk("ctrl\t = 0x%x\n", p->control);
    printk("rev \t = 0x%x (u32)\n", p->reserved_2);
}

static void print_prdt(struct prdte * pt, u32 num)
{
    struct prdte * p = pt;
    int i;
    printk("prdt @ 0x%8.8x, prdte size = 0x%x, total = 0x%x\n", (unsigned int)pt, sizeof(struct prdte), sizeof(struct prdte) * num);

    printk("prdt @ phys 0x%8.8x\n", virt_to_phys(pt));
    for (i = 0; i < num; i++, p++)
    {
        printk("dba\t = 0x%x (u32)\n", p->dba);
        printk("dbau\t = 0x%x (u32)\n", p->dbau);
        printk("rev1\t = 0x%x (u32)\n", p->rev1);
        printk("dbc\t = 0x%x (:22)\n", p->dbc);
        printk("rev2\t = 0x%x (:9)\n", p->rev2);
        printk("i\t = 0x%x (:1)\n", p->i);
    }
}
static void p_prdt(struct prdte * pt)
{
    struct prdte * p = pt;
    int i;
    printk("prdt @ 0x%8.8x, prdte size = 0x%x\n", (unsigned int)pt, sizeof(struct prdte));

        printk("dba\t = 0x%x (u32)\n", p->dba);
        printk("dbau\t = 0x%x (u32)\n", p->dbau);
        printk("rev1\t = 0x%x (u32)\n", p->rev1);
        printk("dbc\t = 0x%x (:22)\n", p->dbc);
        printk("rev2\t = 0x%x (:9)\n", p->rev2);
        printk("i\t = 0x%x (:1)\n", p->i);
}
#endif

static void print_cmd_header(hal_cmd_header *p)
{
    printk(KERN_DEBUG "command header @ 0x%8.8x, cmd header size = 0x%x\n", (unsigned int)p, sizeof(hal_cmd_header));

//    printk(KERN_DEBUG "cmd fis len\t = 0x%x (:5)\n", p->cmd_fis_len);
//    printk(KERN_DEBUG "isATAPI\t = 0x%x (:1)\n", p->isATA_PI);
//    printk(KERN_DEBUG "isWrite\t = 0x%x (:1)\n", p->iswrite);
//    printk(KERN_DEBUG "isprefetch\t = 0x%x (:1)\n", p->isprefetch);
//    printk(KERN_DEBUG "isswrst\t = 0x%x (:1)\n", p->issoftreset);
//    printk(KERN_DEBUG "isbist\t = 0x%x (:1)\n", p->isbist);
//    printk(KERN_DEBUG "isclrok\t = 0x%x (:1)\n", p->isclearok);
//    printk(KERN_DEBUG "rev1\t = 0x%x (:1)\n", p->reserverd);
//    printk(KERN_DEBUG "PMPid\t = 0x%x (:4)\n", p->PMPid);
//    printk(KERN_DEBUG "PRDlen\t = 0x%x (16)\n", p->PRDTlength);
//    printk(KERN_DEBUG "PRDbyte\t = 0x%x (32)\n", p->PRDBytes);
//    printk(KERN_DEBUG "ctbal\t = 0x%x (32)\n", p->ctba_lbase);
//    printk(KERN_DEBUG "ctbah\t = 0x%x (32)\n", p->ctba_hbase);
//    printk("resv1\t = 0x%x (32)\n", p->resv[0]);
//    printk("resv2\t = 0x%x (32)\n", p->resv[1]);
//    printk("resv3\t = 0x%x (32)\n", p->resv[2]);
//    printk("resv4\t = 0x%x (32)\n", p->resv[3]);
}


static void mstar_sata_reg_write(u32 data, phys_addr_t reg_addr)
{
#if defined(CONFIG_ARM64)
    writew(data & 0xFFFF, reg_addr);
    writew((data >> 16) & 0xFFFF, (reg_addr + 0x04));

#else
    iowrite16(data&0xFFFF, reg_addr);
    iowrite16((data >> 16)&0xFFFF,(reg_addr + 0x04));
#endif
}

static u32 mstar_sata_reg_read(phys_addr_t reg_addr)
{
    u32 data;
#if defined(CONFIG_ARM64)
    data = (readw(reg_addr + 0x04) << 16) + readw(reg_addr);
#else
    data = (ioread16(reg_addr + 0x04)<<16) + ioread16(reg_addr);
#endif
    return data;
}

static void read_cmd_fis(void *rx_fis, u32 u32cmd_offset ,hal_cmd_h2dfis *pfis, phys_addr_t misc_base)
{
#if (SATA_CMD_TYPE != TYPE_RIU)
    void *cmd_address = rx_fis + u32cmd_offset;

#if (SATA_CMD_TYPE == TYPE_XIU)
    unsigned long u32MiscAddr = misc_base;

    writew(0x00,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif

    memcpy(pfis, cmd_address, sizeof(hal_cmd_h2dfis));

#if (SATA_CMD_TYPE == TYPE_XIU)
    writew(0x01,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif
#else
    u32 address = (u32)rx_fis + u32cmd_offset;
    u32 rdata;
    u32 *p = (u32 *)pfis;
    u32 offset;

    // loop through data
    for (offset = 0; offset < sizeof(hal_cmd_h2dfis); offset += 4)
    {
        // which address to read?
        mstar_sata_reg_write(address, misc_base + SATA_MISC_CFIFO_ADDRL);
        //printf("write data 0x%8.8x to addr 0x%x\n", address, misc_base + SATA_MISC_CFIFO_ADDRL);

        // what data to write?
        //mstar_sata_reg_write(*dptr, misc_base + SATA_MISC_CFIFO_WDATAL);
        //printf("write data 0x%8.8x to addr 0x%x\n", *dptr, misc_base + SATA_MISC_CFIFO_WDATAL);

        // read(0) or write(1)? normally write
        writew(0x00, misc_base + SATA_MISC_CFIFO_RORW);
        //sata_reg_write16(0x01, misc_base + SATA_MISC_CFIFO_RORW);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_RORW);

        // trigger
        writew(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        //sata_reg_write16(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_ACCESS);

        rdata = mstar_sata_reg_read(misc_base + SATA_MISC_CFIFO_RDATAL);
        //printk("rdata = 0x%x\n", rdata);

        *p = rdata;
        p += 4;

        address += 4;
        //dptr++;
    }
#endif
}

static void build_cmd_fis(void *cmd_tbl ,hal_cmd_h2dfis *pfis, phys_addr_t misc_base)
{
#if (SATA_CMD_TYPE != TYPE_RIU)
#if (SATA_CMD_TYPE == TYPE_XIU)
    unsigned long u32MiscAddr = misc_base;

    writew(0x00,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif

    memcpy(cmd_tbl, pfis,sizeof(hal_cmd_h2dfis));
    //print_fis((hal_cmd_h2dfis *)cmd_tbl);

#if (SATA_CMD_TYPE == TYPE_XIU)
    writew(0x01,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif
#else // TYPE_RIU mode
    u32 address = (u32)cmd_tbl;
    u32 * dptr = (u32 *)pfis;
    u32 offset;

    address = (u32)cmd_tbl;

    // loop through data
    for (offset = 0; offset < sizeof(hal_cmd_h2dfis); offset += 4)
    {
        // which address to write?
        mstar_sata_reg_write(address, misc_base + SATA_MISC_CFIFO_ADDRL);
        //printf("write data 0x%8.8x to addr 0x%x\n", address, misc_base + SATA_MISC_CFIFO_ADDRL);

        // what data to write?
        mstar_sata_reg_write(*dptr, misc_base + SATA_MISC_CFIFO_WDATAL);
        //printf("write data 0x%8.8x to addr 0x%x\n", *dptr, misc_base + SATA_MISC_CFIFO_WDATAL);

        // read(0) or write(1)? normally write
        writew(0x01, misc_base + SATA_MISC_CFIFO_RORW);
        //sata_reg_write16(0x01, misc_base + SATA_MISC_CFIFO_RORW);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_RORW);

        // trigger
        writew(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        //sata_reg_write16(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_ACCESS);

        address += 4;
        dptr++;
    }

#endif
}

static void build_cmd_prdt(void *base_address ,u32 *pprdt, phys_addr_t misc_base, u32 prdt_num)
{
#if (SATA_CMD_TYPE != TYPE_RIU)
    void *cmd_address = base_address + SATA_KA9_CMD_DESC_OFFSET_TO_PRDT;

#if (SATA_CMD_TYPE == TYPE_XIU)
    unsigned long u32MiscAddr = misc_base;


    writew(0x00,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif

    memcpy(cmd_address,pprdt,sizeof(u32)*prdt_num*4);
    //print_prdt(cmd_address, prdt_num);

#if (SATA_CMD_TYPE == TYPE_XIU)
    writew(0x01,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif
#else // TYPE_RIU
    u32 address = (u32)base_address + SATA_KA9_CMD_DESC_OFFSET_TO_PRDT;
    u32 * dptr = pprdt;
    u32 offset;

    p_prdt((struct prdte *)pprdt);

    for (offset = 0; offset < sizeof(u32) * prdt_num * 4; offset += 4) {
        // which address to write?
        mstar_sata_reg_write(address, misc_base + SATA_MISC_CFIFO_ADDRL);
        //printf("write data 0x%8.8x to addr 0x%x\n", address, misc_base + SATA_MISC_CFIFO_ADDRL);

        // what data to write?
        mstar_sata_reg_write(*dptr, misc_base + SATA_MISC_CFIFO_WDATAL);
        //printf("write data 0x%8.8x to addr 0x%x\n", *dptr, misc_base + SATA_MISC_CFIFO_WDATAL);

        // read(0) or write(1)? normally write
        writew(0x01, misc_base + SATA_MISC_CFIFO_RORW);
        //printf("write data 0x0001 to addr 0x%8.8x\n", misc_base + SATA_MISC_CFIFO_RORW);

        // trigger
        writew(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_ACCESS);

        address += 4;
        dptr++;
    }
#endif
}

static void build_cmd_header(void *cmd_slot, u32 u32offset_address,u32 *pcmdheader, phys_addr_t misc_base)
{
#if (SATA_CMD_TYPE != TYPE_RIU)
    void *cmd_address = cmd_slot + u32offset_address;

#if (SATA_CMD_TYPE == TYPE_XIU)
    unsigned long u32MiscAddr = misc_base;

    writew(0x00,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif

    memcpy(cmd_address, pcmdheader, SATA_KA9_CMD_HDR_SIZE);
    //print_cmd_header(cmd_address);
    Chip_Flush_Memory();

#if (SATA_CMD_TYPE == TYPE_XIU)
    writew(0x01,u32MiscAddr+SATA_MISC_ACCESS_MODE);
#endif
#else
    u32 address = (u32)cmd_slot + u32offset_address;
    u32 * dptr = pcmdheader;
    u32 offset;

    for (offset = 0; offset < SATA_KA9_CMD_HDR_SIZE; offset += 4) {
        // which address to write?
        mstar_sata_reg_write(address, misc_base + SATA_MISC_CFIFO_ADDRL);
        //printf("write data 0x%8.8x to addr 0x%x\n", address, misc_base + SATA_MISC_CFIFO_ADDRL);

        // what data to write?
        mstar_sata_reg_write(*dptr, misc_base + SATA_MISC_CFIFO_WDATAL);
        //printf("write data 0x%8.8x to addr 0x%x\n", *dptr, misc_base + SATA_MISC_CFIFO_WDATAL);

        // read(0) or write(1)? normally write
        //sata_reg_write16(0x01, misc_base + SATA_MISC_CFIFO_RORW);
        writew(0x01, misc_base + SATA_MISC_CFIFO_RORW);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_RORW);

        // trigger
        //sata_reg_write16(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        writew(0x01, misc_base + SATA_MISC_CFIFO_ACCESS);
        //printf("write data 0x0001 to addr 0x%x\n", misc_base + SATA_MISC_CFIFO_ACCESS);

        address += 4;
        dptr++;
    }
#endif
}

static inline unsigned int sata_mstar_tag(unsigned int tag)
{
    /* all non NCQ/queued commands should have tag#0 */
    if (ata_tag_internal(tag)) {
        return 0;
    }

    if (unlikely(tag >= SATA_KA9_QUEUE_DEPTH)) {
        DPRINTK("tag %d invalid : out of range\n", tag);
        //printk("[%s][%d]\n",__FUNCTION__,__LINE__);
        return 0;
    }
    return tag;
}

static void sata_mstar_setup_cmd_hdr_entry(struct sata_mstar_port_priv *pp,
                     unsigned int tag, u32 data_xfer_len, u8 num_prde,
                     u8 fis_len, phys_addr_t misc_base)
{
    dma_addr_t cmd_descriptor_address;
    hal_cmd_header cmd_header = {0};
    void *cmd_slot = pp->cmd_slot;

    cmd_descriptor_address = pp->cmd_tbl_dma + tag * SATA_KA9_CMD_DESC_SIZE;

    cmd_header.cmd_fis_len = fis_len;
    cmd_header.PRDTlength = num_prde;
    cmd_header.isclearok = 0;
    cmd_header.PRDBytes = data_xfer_len;
    cmd_header.ctba_hbase = 0;
    cmd_header.ctba_lbase = cmd_descriptor_address;

    build_cmd_header(cmd_slot, tag * SATA_KA9_CMD_HDR_SIZE,(u32 *)&cmd_header, misc_base);
}

static unsigned int sata_mstar_fill_sg(struct ata_queued_cmd *qc,
                     u32 *ttl, void *cmd_tbl, phys_addr_t misc_base)
{
    struct scatterlist *sg;
    u32 ttl_dwords = 0;
    u32 prdt[SATA_KA9_USED_PRD * 4] = {0};
    unsigned int si;

    for_each_sg(qc->sg, sg, qc->n_elem, si) {
        dma_addr_t sg_addr = sg_dma_address(sg);
        u32 sg_len = sg_dma_len(sg);

        if (si == (SATA_KA9_USED_PRD - 1) && ((sg_next(sg)) != NULL))
        {
            printk(KERN_ERR "setting indirect prde , out of prdt\n");
        }
        ttl_dwords += sg_len;

        prdt[si*4 + 0] =  (u32)(MHal_SATA_bus_address(cpu_to_le32(sg_addr)));
        prdt[si*4 + 1] =  (u32)(MHal_SATA_bus_address(cpu_to_le32(sg_addr)) >> 16);
        prdt[si*4 + 2] = 0xFFFFFFFF;
        prdt[si*4 + 3] = (cpu_to_le32(sg_len) - 1);
    }
    build_cmd_prdt(cmd_tbl,&prdt[0], misc_base, si);
    *ttl = ttl_dwords;
    return si;
}

static u32 mstar_sata_wait_reg(phys_addr_t reg_addr, u32 mask, u32 val, unsigned long interval, unsigned long timeout)
{
    u32 temp;
    unsigned long timeout_vale = 0;

    temp = mstar_sata_reg_read(reg_addr);

    while((temp & mask) == val)
    {
        msleep(interval);
        timeout_vale += interval;
        if (timeout_vale > timeout)
            break;
        temp = mstar_sata_reg_read(reg_addr);
    }
    return temp;
}

static int mstar_ahci_stop_engine(phys_addr_t port_base)
{
    u32 temp;

    temp = mstar_sata_reg_read(PORT_CMD + port_base);

    /* check if the HBA is idle */
    if ((temp & (PORT_CMD_START | PORT_CMD_LIST_ON)) == 0)
        return 0;

    /* setting HBA to idle */
    temp &= ~PORT_CMD_START;
    mstar_sata_reg_write(temp, PORT_CMD + port_base);

    temp = mstar_sata_wait_reg(PORT_CMD + port_base, PORT_CMD_LIST_ON, PORT_CMD_LIST_ON, 1, 500);

    if (temp & PORT_CMD_LIST_ON)
        return -EIO;

    return 0;
}

static void mstar_ahci_start_engine(phys_addr_t port_base)
{
    u32 temp;

    /* Start Port DMA */
    temp = mstar_sata_reg_read(PORT_CMD + port_base);
    temp |= PORT_CMD_START;
    mstar_sata_reg_write(temp, PORT_CMD + port_base);
    mstar_sata_reg_read(PORT_CMD + port_base); /* Flush */
}

static void mstar_ahci_start_fis_rx(struct ata_port *ap)
{
    struct sata_mstar_port_priv *pp = ap->private_data;
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;
    u32 tmp;

    // set FIS registers
    tmp = pp->cmd_slot_dma;
    mstar_sata_reg_write(tmp, PORT_LST_ADDR + port_base);

    tmp = pp->rx_fis_dma;
    mstar_sata_reg_write(tmp , PORT_FIS_ADDR + port_base);

    // enable FIS reception
    tmp = mstar_sata_reg_read(PORT_CMD + port_base);
    tmp |= PORT_CMD_FIS_RX;
    mstar_sata_reg_write(tmp, PORT_CMD + port_base);

    // flush
    tmp= mstar_sata_reg_read(PORT_CMD + port_base);
}

static int mstar_ahci_stop_fis_rx(struct ata_port *ap)
{
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;
    u32 tmp;

    // Disable FIS reception
    tmp = mstar_sata_reg_read(PORT_CMD + port_base);
    tmp &= ~PORT_CMD_FIS_RX;
    mstar_sata_reg_write(tmp, PORT_CMD + port_base);

    // Wait FIS reception Stop for 1000ms
    tmp = mstar_sata_wait_reg(PORT_CMD + port_base, PORT_CMD_FIS_ON, PORT_CMD_FIS_ON, 10, 1000);

    if (tmp & PORT_CMD_FIS_ON)
        return -EBUSY;

    return 0;
}

static void sata_mstar_qc_prep(struct ata_queued_cmd *qc)
{
    struct ata_port *ap = qc->ap;
    struct sata_mstar_port_priv *pp = ap->private_data;
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    u32 misc_base = host_priv->misc_base;
    unsigned int tag = sata_mstar_tag(qc->tag);
    hal_cmd_h2dfis h2dfis;
    u32 num_prde = 0;
    u32 ttl_dwords = 0;
    void *cmd_tbl;

    //printk("%s\n", __func__);
    cmd_tbl = pp->cmd_tbl + (tag * SATA_KA9_CMD_DESC_SIZE);

    ata_tf_to_fis(&qc->tf, qc->dev->link->pmp, 1, (u8 *)&h2dfis);

    build_cmd_fis(cmd_tbl ,&h2dfis, misc_base);

    if (qc->flags & ATA_QCFLAG_DMAMAP)
    {
        num_prde = sata_mstar_fill_sg(qc, &ttl_dwords, cmd_tbl, misc_base);
    }

    sata_mstar_setup_cmd_hdr_entry(pp, tag, ttl_dwords,
                     num_prde, 5, misc_base);

}

static unsigned int sata_mstar_qc_issue(struct ata_queued_cmd *qc)
{
    struct sata_mstar_host_priv *host_priv = qc->ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;
    unsigned int tag = sata_mstar_tag(qc->tag);

    if (qc->tf.protocol == ATA_PROT_NCQ)
    {
        //printk("NCQ device- %d\n", qc->tag);
        mstar_sata_reg_write(1 << qc->tag, PORT_SCR_ACT + port_base);
    }
    mstar_sata_reg_write(1 << tag, PORT_CMD_ISSUE + port_base);
    return 0;
}

static bool sata_mstar_qc_fill_rtf(struct ata_queued_cmd *qc)
{
    struct sata_mstar_port_priv *pp = qc->ap->private_data;
    struct sata_mstar_host_priv *host_priv = qc->ap->host->private_data;
    unsigned int tag = sata_mstar_tag(qc->tag);
    hal_cmd_h2dfis cd;
    phys_addr_t misc_base = host_priv->misc_base;
    void *rx_fis;

    rx_fis = pp->rx_fis;

    read_cmd_fis(rx_fis,(tag * SATA_KA9_CMD_DESC_SIZE),&cd, misc_base);

    ata_tf_from_fis((const u8 *)&cd, &qc->result_tf);
    return true;
}

static int sata_mstar_scr_offset(struct ata_port *ap, unsigned int sc_reg)
{
    static const int offset[] = {
        [SCR_STATUS]        = PORT_SCR_STAT,
        [SCR_CONTROL]       = PORT_SCR_CTL,
        [SCR_ERROR]         = PORT_SCR_ERR,
        [SCR_ACTIVE]        = PORT_SCR_ACT,
        [SCR_NOTIFICATION]  = PORT_SCR_NTF,
    };

    if (sc_reg < ARRAY_SIZE(offset) && (sc_reg != SCR_NOTIFICATION))
        return offset[sc_reg];

    return 0;
}

static int sata_mstar_scr_read(struct ata_link *link,
                 unsigned int sc_reg, u32 *val)
{
    int offset = sata_mstar_scr_offset(link->ap, sc_reg);
    struct sata_mstar_host_priv *host_priv = link->ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;

    if (offset) {
        *val = mstar_sata_reg_read(offset + port_base);
        return 0;
    }
    return -1;
}

static int sata_mstar_scr_write(struct ata_link *link,
                  unsigned int sc_reg_in, u32 val)
{
    int offset = sata_mstar_scr_offset(link->ap, sc_reg_in);
    struct sata_mstar_host_priv *host_priv = link->ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;

    if (offset) {
        mstar_sata_reg_write(val, offset + port_base);
        return 0;
    }
    return -1;
}

static void sata_mstar_freeze(struct ata_port *ap)
{
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;

    mstar_sata_reg_write(0, PORT_IRQ_MASK + port_base);
}

static void sata_mstar_thaw(struct ata_port *ap)
{
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    phys_addr_t hba_base = host_priv->hba_base;
    phys_addr_t port_base = host_priv->port_base;
    u32 u32Temp = 0;

    // clear IRQ
    u32Temp = mstar_sata_reg_read(PORT_IRQ_STAT + port_base);
    mstar_sata_reg_write(u32Temp, PORT_IRQ_STAT + port_base);

    // Clear Port 0 IRQ on HBA
    u32Temp = mstar_sata_reg_read(HOST_IRQ_STAT + hba_base);
    mstar_sata_reg_write(u32Temp, HOST_IRQ_STAT + hba_base);

    // Enable Host Interrupt
    u32Temp = mstar_sata_reg_read(HOST_CTL + hba_base);
    u32Temp |= HOST_IRQ_EN;
    mstar_sata_reg_write(u32Temp, HOST_CTL + hba_base);

    // Enable Port Interrupt
    mstar_sata_reg_write(DEF_PORT_IRQ, PORT_IRQ_MASK + port_base);
    mstar_sata_reg_read(PORT_IRQ_MASK + port_base);
}

static unsigned int sata_mstar_dev_classify(struct ata_port *ap)
{
    struct ata_taskfile tf;
    u32 temp = 0;
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;

    temp = mstar_sata_reg_read(PORT_SIG + host_priv->port_base);

    tf.lbah = (temp >> 24) & 0xff;
    tf.lbam = (temp >> 16) & 0xff;
    tf.lbal = (temp >> 8) & 0xff;
    tf.nsect = temp & 0xff;
    return ata_dev_classify(&tf);
}

int mstar_ahci_check_ready(struct ata_link *link)
{
    struct ata_port *ap = link->ap;
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    u8 status;

    phys_addr_t port_base = host_priv->port_base;
    status = mstar_sata_reg_read(PORT_TFDATA + port_base) & 0xFF;

    return ata_check_ready(status);
}

static int sata_mstar_hardreset(struct ata_link *link, unsigned int *class,
                    unsigned long deadline)
{
    struct ata_port *ap = link->ap;
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    const unsigned long *timing = sata_ehc_deb_timing(&link->eh_context);
    phys_addr_t port_base = host_priv->port_base;
    bool online;
    int rc;

    mstar_ahci_stop_engine(port_base);

    rc = sata_link_hardreset(link, timing, deadline, &online,
                 mstar_ahci_check_ready);

    mstar_ahci_start_engine(port_base);

    if (online)
        *class = sata_mstar_dev_classify(ap);

    printk(KERN_INFO "[%s] done\n", __func__);
    return rc;
}

static int sata_mstar_softreset(struct ata_link *link, unsigned int *class,
                    unsigned long deadline)
{
    // Unused Function
    return 0;
}

static void sata_mstar_error_handler(struct ata_port *ap)
{
    sata_pmp_error_handler(ap);
}

static void sata_mstar_post_internal_cmd(struct ata_queued_cmd *qc)
{
    if (qc->flags & ATA_QCFLAG_FAILED)
        qc->err_mask |= AC_ERR_OTHER;

    if (qc->err_mask) {
        /* make DMA engine forget about the failed command */
    }
}

static int sata_mstar_port_start(struct ata_port *ap)
{
    struct sata_mstar_port_priv *pp;
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    u32 temp;
    phys_addr_t port_base = host_priv->port_base;
#if (SATA_CMD_TYPE == TYPE_DRAM)
    struct device *dev = ap->host->dev;
    void *mem;
    dma_addr_t mem_dma;
    size_t dma_sz;
#endif

    // Allocate SATA Port Private Data
    pp = kzalloc(sizeof(*pp), GFP_KERNEL);
    if (!pp) {
        printk("[%s][Error] SATA Allocate Port Private Data Fail\n", __func__);
        return -ENOMEM;
    }

#if (SATA_CMD_TYPE == TYPE_XIU)
    printk(KERN_INFO "SATA cmd type: XIU\n");
    pp->cmd_slot = (void *)(SATA_SDMAP_RIU_BASE + (AHCI_P0CLB & 0xfff));
    pp->rx_fis = (void *)(SATA_SDMAP_RIU_BASE + (AHCI_P0FB & 0xfff));
    pp->cmd_tbl = (void *)(SATA_SDMAP_RIU_BASE + (AHCI_CTBA0 & 0xfff));

    pp->cmd_slot_dma = AHCI_P0CLB;
    pp->rx_fis_dma = AHCI_P0FB;
    pp->cmd_tbl_dma = AHCI_CTBA0;

#elif (SATA_CMD_TYPE == TYPE_DRAM)
    printk(KERN_INFO "SATA cmd type: DRAM\n");

    dma_sz = 0x10000;
    memset(host_priv->mem, 0, dma_sz);
    mem = host_priv->mem;
    mem_dma = host_priv->mem_dma;
    memset(mem, 0, dma_sz);

    pp->cmd_slot = mem;
    // Translate physical address to bus address since SATA engine uses bus address.
    mem_dma = (dma_addr_t)MHal_SATA_bus_address(mem_dma);
    pp->cmd_slot_dma = mem_dma;

    mem += SATA_CMD_HEADER_SIZE;
    mem_dma += SATA_CMD_HEADER_SIZE;

    pp->rx_fis = mem;
    pp->rx_fis_dma = mem_dma;

    mem += SATA_FIS_SIZE;
    mem_dma += SATA_FIS_SIZE;

    pp->cmd_tbl = mem;
    pp->cmd_tbl_dma = mem_dma;
#elif (SATA_CMD_TYPE == TYPE_RIU)
    printk(KERN_INFO "SATA cmd type: RIU\n");
    pp->cmd_slot = (void *)(AHCI_P0CLB);
    pp->rx_fis = (void *)(AHCI_P0FB);
    pp->cmd_tbl = (void *)(AHCI_CTBA0);

    pp->cmd_slot_dma = AHCI_P0CLB;
    pp->rx_fis_dma = AHCI_P0FB;
    pp->cmd_tbl_dma = AHCI_CTBA0;
#else
#error "SATA_CMD_TYPE Unknown"
#endif

    sata_debug("cmd_slot = 0x%llx ; cmd_slot_dma = 0x%llx\n", (u32)pp->cmd_slot, (u32)pp->cmd_slot_dma);
    sata_debug("rx_fis = 0x%llx ; rx_fis_dma = 0x%llx\n", (u32)pp->rx_fis, (u32)pp->rx_fis_dma);
    sata_debug("cmd_tbl = 0x%llx ; cmd_tbl_dma = 0x%llx\n", (u32)pp->cmd_tbl, (u32)pp->cmd_tbl_dma);

    ap->private_data = pp;

    temp = mstar_sata_reg_read(PORT_CMD + port_base) & ~PORT_CMD_ICC_MASK;

    // spin up device
    temp |= PORT_CMD_SPIN_UP;
    mstar_sata_reg_write(temp, PORT_CMD + port_base);

    // wake up link
    mstar_sata_reg_write((temp | PORT_CMD_ICC_ACTIVE), PORT_CMD + port_base);

    // start FIS RX
    mstar_ahci_start_fis_rx(ap);

    // Clear IS , Interrupt Status
    mstar_sata_reg_write(0xFFFFFFFF, PORT_IRQ_STAT + port_base);
    mstar_sata_reg_write(0xFFFFFFFF, PORT_SCR_ERR + port_base);

    // set to speed limit with gen 1, gen 2 or auto
    temp = mstar_sata_reg_read(PORT_SCR_CTL + port_base);
    temp = temp & (~E_PORT_SPEED_MASK);

    temp = temp | MHal_SATA_get_max_speed();
    mstar_sata_reg_write(temp, PORT_SCR_CTL + port_base);

    // Start DMA Engine
    mstar_ahci_start_engine(port_base);

    return 0;
}

static void sata_mstar_port_stop(struct ata_port *ap)
{
    struct sata_mstar_host_priv *host_priv = ap->host->private_data;
    phys_addr_t port_base = host_priv->port_base;
    int ret;

    printk("%s\n", __func__);
    // Stop DMA Engine
    ret = mstar_ahci_stop_engine(port_base);
    if (ret)
    {
        printk(KERN_ERR "[%s][Error] Fail to Stop SATA Port\n", __func__);
    }

    // Disable FIS reception
    ret = mstar_ahci_stop_fis_rx(ap);
    if (ret) {
        printk(KERN_ERR "[%s][Error] Fail to Stop FIS RX\n", __func__);
    }
}

irqreturn_t sata_mstar_interrupt(int irq, void *dev_instance)
{
    struct ata_host *host = dev_instance;
    struct ata_port *ap = host->ports[0];
    struct sata_mstar_host_priv *host_priv = host->private_data;
    struct ata_link *link = NULL;
    struct ata_eh_info *ehi;

    phys_addr_t port_base = host_priv->port_base;
    phys_addr_t hba_base = host_priv->hba_base;
    u32 host_status,port_status;
    u32 qc_active;
    u32 port_ie;
    u32 port_err;
    u32 clr_port_err = 0;

    link = &ap->link;
    ehi = &link->eh_info;
    ata_ehi_clear_desc(ehi);

    host_status = mstar_sata_reg_read(HOST_IRQ_STAT + hba_base);

    spin_lock(&host->lock);
    port_status = mstar_sata_reg_read(PORT_IRQ_STAT + port_base);
    if (port_status & (1 << 6)) {
        printk(KERN_WARNING "HDD inserted (Device presence change, clear diag.X)\n");
        mstar_sata_reg_write((1 << 26), PORT_SCR_ERR + port_base);
    }
    if (port_status & (1 << 22)) {
        printk(KERN_WARNING "HDD removed (PhyRdy change, clear diag.N)\n");
        mstar_sata_reg_write((1 << 16), PORT_SCR_ERR + port_base);
        //ata_ehi_hotplugged(ehi);
        //ata_ehi_push_desc(ehi, "%s", "PHY RDY changed");
    }

    port_err = mstar_sata_reg_read(PORT_SCR_ERR + port_base);
    if (port_err & (1 << 1)) {
        printk(KERN_WARNING "SATA recovered from communication error\n");
        clr_port_err |= (1 << 1);
    }
    if (port_err & (1 << 9)) {
        printk(KERN_WARNING "SATA Persistent comm error/data integrity error\n");
        clr_port_err |= (1 << 9);
    }
    if (clr_port_err)
        mstar_sata_reg_write(clr_port_err, PORT_SCR_ERR + port_base);

#if 0
    if (port_status) {
        printk("port status = 0x%x\n", port_status);
    }
#endif

    // clear interrupt
    mstar_sata_reg_write(port_status, PORT_IRQ_STAT + port_base);

    qc_active = mstar_sata_reg_read(PORT_SCR_ACT + port_base);
    qc_active |= mstar_sata_reg_read(PORT_CMD_ISSUE + port_base);

    ata_qc_complete_multiple(ap, qc_active);
    mstar_sata_reg_write(host_status, HOST_IRQ_STAT + hba_base);
    spin_unlock(&host->lock);

    return IRQ_RETVAL(1);
}

static int mstar_sata_hardware_init(struct sata_mstar_host_priv *hpriv)
{
    u32 i;
    u32 u32Temp = 0;
    phys_addr_t misc_base = hpriv->misc_base;
    phys_addr_t port_base = hpriv->port_base;
    phys_addr_t hba_base = hpriv->hba_base;

    MHal_SATA_HW_Inital(misc_base, port_base, hba_base);

    writew(HOST_RESET, hba_base + (HOST_CTL));

    u32Temp = mstar_sata_wait_reg(HOST_CTL + hba_base, HOST_RESET, HOST_RESET, 1, 500);

    if (u32Temp & HOST_RESET)
        return -1;

    // Turn on AHCI_EN
    u32Temp = mstar_sata_reg_read(HOST_CTL + hba_base);
    if (u32Temp & HOST_AHCI_EN)
    {
        MHal_SATA_Setup_Port_Implement(misc_base, port_base, hba_base);
        return 0;
    }

    // Try AHCI_EN Trurn on for a few time
    for (i = 0; i < 5; i++) {
        u32Temp |= HOST_AHCI_EN;
        mstar_sata_reg_write(u32Temp, HOST_CTL + hba_base);
        u32Temp = mstar_sata_reg_read(HOST_CTL + hba_base);
        if (u32Temp & HOST_AHCI_EN)
            break;
        msleep(10);
    }

    MHal_SATA_Setup_Port_Implement(misc_base, port_base, hba_base);

    return 0;
}

static struct scsi_host_template mstar_sata_sht = {
#if defined(USE_NCQ)
    ATA_NCQ_SHT("mstar_sata"),
#else
    ATA_BASE_SHT("mstar_sata"),
#endif
    .can_queue = SATA_KA9_QUEUE_DEPTH,
    .sg_tablesize = SATA_KA9_USED_PRD,
    .dma_boundary = ATA_DMA_BOUNDARY,
};

static struct ata_port_operations mstar_sata_ops = {
    .inherits       = &sata_pmp_port_ops,

    .qc_defer = ata_std_qc_defer,
    .qc_prep = sata_mstar_qc_prep,
    .qc_issue = sata_mstar_qc_issue,
    .qc_fill_rtf = sata_mstar_qc_fill_rtf,

    .scr_read = sata_mstar_scr_read,
    .scr_write = sata_mstar_scr_write,

    .freeze = sata_mstar_freeze,
    .thaw = sata_mstar_thaw,

    .softreset = sata_mstar_softreset,
    .hardreset = sata_mstar_hardreset,

    .pmp_softreset = sata_mstar_softreset,
    .error_handler = sata_mstar_error_handler,
    .post_internal_cmd = sata_mstar_post_internal_cmd,

    .port_start = sata_mstar_port_start,
    .port_stop = sata_mstar_port_stop,

};

static const struct ata_port_info mstar_sata_port_info[] = {
    {
     .flags = SATA_KA9_HOST_FLAGS,
     .pio_mask = ATA_PIO6,
     .udma_mask = ATA_UDMA6,
     .port_ops = &mstar_sata_ops,
     },
};

static int mstar_sata_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct ata_port_info pi = mstar_sata_port_info[0];
    const struct ata_port_info *ppi[] = { &pi, NULL };
    struct sata_mstar_host_priv *hpriv;
    struct ata_host *host;
    int irq = 0;
    int ret = 0;
    size_t dma_sz;
    dma_addr_t mem_dma;
    dma_sz = 0x10000;

#if 0 // defined(CONFIG_OF)
    struct device_node *np = dev->of_node;
    int hba_base;
    int port_base;
    int misc_base;
#else
    struct resource *port_mem;
    struct resource *misc_mem;
    struct resource *hba_mem;
#endif

    printk("MStar SATA Host Controller Probing...\n");

#if 0 // defined(CONFIG_OF)
    ret = of_property_read_u32(np, "mstar,hba-base", &hba_base);
    if (ret) {
        printk("[%s][Error] SATA Get HBA Resource Fail\n", __func__);
        return -EINVAL;
    }
    printk("dtb hba_base = %x\n", hba_base);

    ret = of_property_read_u32(np, "mstar,port-base", &port_base);
    if (ret) {
        printk("[%s][Error] SATA Get Port Resource Fail\n", __func__);
        return -EINVAL;
    }
    printk("dtb port_base= %x\n", port_base);

    ret = of_property_read_u32(np, "mstar,misc-base", &misc_base);
    if (ret) {
        printk("[%s][Error] SATA Get MISC Resource Fail\n", __func__);
        return -EINVAL;
    }
    printk("dtb misc_base= %x\n", misc_base);

    ret = of_property_read_u32(np, "interrupts", &irq);
    if ((ret) && (irq <= 0)) {
        printk("[%s][Error] SATA Get IRQ Fail\n", __func__);
        return -EINVAL;
    }
#else
    hba_mem  = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    port_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    misc_mem = platform_get_resource(pdev, IORESOURCE_MEM, 2);

    if (!hba_mem) {
        printk(KERN_ERR "[%s][Error] SATA Get HBA Resource Fail\n", __func__);
        return -EINVAL;
    }
    if (!port_mem) {
        printk(KERN_ERR "[%s][Error] SATA Get Port Resource Fail\n", __func__);
        return -EINVAL;
    }
    if (!misc_mem) {
        printk(KERN_ERR "[%s][Error] SATA Get MISC Resource Fail\n", __func__);
        return -EINVAL;
    }

    irq = platform_get_irq(pdev, 0);
    if (irq <= 0) {
        printk(KERN_ERR "[%s][Error] SATA Get IRQ Fail\n", __func__);
        return -EINVAL;
    }
#endif
    // Allocate Host Private Data
    hpriv = devm_kzalloc(dev, sizeof(*hpriv), GFP_KERNEL);
    if (!hpriv)
    {
        printk(KERN_ERR "[%s][Error] SATA Allocate Host Private Data Fail\n", __func__);
        return -ENOMEM;
    }

// FIXME: need a way to set DMA mask in DTB, otherwise we will get dma allocation fail.
#if 0 //defined(CONFIG_OF)
    printk("CONFIG_OF\n");
    printk("MSTAR_RIU_BASE = 0x%llx..\n", mstar_pm_base);
    hpriv->hba_base  = (mstar_pm_base + (hba_base << 1));
    hpriv->port_base = (mstar_pm_base + (port_base << 1));
    hpriv->misc_base = (mstar_pm_base + (misc_base << 1));
#else
    hpriv->hba_base  = mstar_pm_base + (hba_mem->start & 0x00FFFFFF);
    hpriv->port_base = mstar_pm_base + (port_mem->start & 0x00FFFFFF);
    hpriv->misc_base = mstar_pm_base + (misc_mem->start & 0x00FFFFFF);
#endif

    sata_debug("hba_base = 0x%llx ; port_base = 0x%llx ; misc_base = 0x%llx, IRQ = %d\n",
        hpriv->hba_base, hpriv->port_base, hpriv->misc_base, irq);
#if defined(USE_NCQ)
    sata_debug("USE_NCQ\n");
#endif

    // Initial SATA Hardware
    if (mstar_sata_hardware_init(hpriv))
    {
        printk(KERN_ERR "[%s][Error] SATA Hardware Initial Failed\n", __func__);
        return -EINVAL;
    }

    host = ata_host_alloc_pinfo(dev, ppi, SATA_PORT_NUM);
    if (!host) {
        ret = -ENOMEM;
        printk(KERN_ERR "[%s][Error] SATA Allocate ATA Host Fail\n", __func__);
        goto out_devm_kzalloc_hpriv;
    }

    host->private_data = hpriv;

    hpriv->mem = dmam_alloc_coherent(dev, dma_sz, &mem_dma, GFP_KERNEL);
    if (!hpriv->mem)
        return -ENOMEM;
    hpriv->mem_dma = mem_dma;
    memset(hpriv->mem, 0, dma_sz);
    return ata_host_activate(host, irq, sata_mstar_interrupt, IRQF_SHARED, &mstar_sata_sht);
    return 0;

out_devm_kzalloc_hpriv:
    devm_kfree(dev, hpriv);

    return ret;
}

static int mstar_sata_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct ata_host *host = dev_get_drvdata(dev);
    struct sata_mstar_host_priv *host_priv = host->private_data;

    ata_host_detach(host);

    devm_kfree(dev, host_priv);

    return 0;
}

static int mstar_sata_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct ata_host *host = dev_get_drvdata(&pdev->dev);
    struct ata_port *ap = host->ports[0];
    struct sata_mstar_port_priv *pp = ap->private_data;

    printk(KERN_INFO "[%s]\n", __func__);
    kfree(pp);
    return ata_host_suspend(host, state);
}

static int mstar_sata_resume(struct platform_device *pdev)
{
    struct ata_host *host = dev_get_drvdata(&pdev->dev);
    struct sata_mstar_host_priv *hpriv = host->private_data;

    printk(KERN_INFO "[%s]\n", __func__);
    if (mstar_sata_hardware_init(hpriv))
    {
        printk(KERN_ERR "[%s][Error] SATA Hardware Initial Failed, SATA resume failed!\n", __func__);
        return -EINVAL;
    }
    //sata_mstar_port_start(struct ata_port *ap)
    sata_mstar_port_start(host->ports[0]);

    ata_host_resume(host);
    printk(KERN_INFO "[%s] done\n", __func__);
    return 0;
}
#if defined(CONFIG_ARM64)
#if 0 // defined(CONFIG_OF)
static const struct of_device_id mstar_satahost_dt_match[] = {
    { .compatible = "mstar-sata", },
    {}
};
MODULE_DEVICE_TABLE(of, mstar_satahost_dt_match);
#endif
#endif

static struct platform_driver mstar_sata_driver = {
    .probe          = mstar_sata_probe,
    .remove         = mstar_sata_remove,
#ifdef CONFIG_PM
    .suspend        = mstar_sata_suspend,
    .resume         = mstar_sata_resume,
#endif
    .driver         = {
        .name       = "Mstar-sata",
#if defined(CONFIG_ARM64)
#if 0 // defined(CONFIG_OF)
       .of_match_table = mstar_satahost_dt_match,
#endif
#endif
       .owner       = THIS_MODULE,
    }
};

#if defined(CONFIG_ARM64)
static u64 sata_dmamask = DMA_BIT_MASK(64);
#else
static u64 sata_dmamask = DMA_BIT_MASK(32);
#endif

static struct resource satahost_resources[] = {
   [0] = {
      .start = SATA_GHC_0_ADDRESS_START,
      .end   = SATA_GHC_0_ADDRESS_END,
      .flags = IORESOURCE_MEM,
   },
   [1] = {
      .start = SATA_GHC_1_ADDRESS_START,
      .end   = SATA_GHC_1_ADDRESS_END,
      .flags = IORESOURCE_MEM,
   },
   [2] = {
      .start = SATA_MISC_ADDRESS_START,
      .end   = SATA_MISC_ADDRESS_END,
      .flags = IORESOURCE_MEM,
   },
   [3] = {
      .start = E_IRQ_SATA_INT,
      .end   = E_IRQ_SATA_INT,
      .flags = IORESOURCE_IRQ,
   },
};

struct platform_device Mstar_satahost_device = {
    .name = "Mstar-sata",
    .id = 0,
    .dev = {
        .dma_mask = &sata_dmamask,
#if defined(CONFIG_ARM64)
        .coherent_dma_mask = DMA_BIT_MASK(64),
#else
        .coherent_dma_mask = DMA_BIT_MASK(32),
#endif
    },
    .num_resources = ARRAY_SIZE(satahost_resources),
    .resource = satahost_resources,
};

static int __init mstar_sata_drv_init(void)
{
    int ret = 0;

    printk(KERN_INFO "Mstar sata host register platform driver\n");
    ret= platform_driver_register(&mstar_sata_driver);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register SATA platform driver, %d\n", ret);
        return ret;
    }

    printk(KERN_INFO "Mstar sata host register platform device\n");
    ret = platform_device_register(&Mstar_satahost_device);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register SATA platform device, %d\n", ret);
        platform_driver_unregister(&mstar_sata_driver);
        return -ret;
    }

    return ret;
}

static void __exit mstar_sata_drv_exit(void)
{
    platform_driver_unregister(&mstar_sata_driver);
}

MODULE_AUTHOR("Mstar Semiconductor");
MODULE_DESCRIPTION("Mstar 3.0Gbps SATA controller low level driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.00");

module_init(mstar_sata_drv_init);
module_exit(mstar_sata_drv_exit);
