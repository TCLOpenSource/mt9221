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

//*********************************************************************
//
//   MODULE NAME:
//       mtd_serflash.c
//
//   DESCRIPTION:
//      This file is an agent between mtd layer and spi flash driver
//
//  PUBLIC PROCEDURES:
//       Name                    Title
//       ----------------------- --------------------------------------
//       int xxx_proc           declare in its corresponding header file
//
//   LOCAL PROCEDURES:
//       Name                    Title
//       ----------------------- --------------------------------------
//       get_prnt_cnvs           the local procedure in the file
//
//  Written by Tao.Zhou@MSTAR Inc.
//---------------------------------------------------------------------
//
//********************************************************************
//--------------------------------------------------------------------
//                             GENERAL INCLUDE
//--------------------------------------------------------------------

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
#include <linux/slab.h>
#endif
#include "drvSERFLASH.h"

/* Flash opcodes. */
#define OPCODE_BE_4K        0x20UL  /* Erase 4KiB block */
#define OPCODE_BE_32K       0x52UL  /* Erase 32KiB block */
//#define   OPCODE_CHIP_ERASE   0xc7    /* Erase whole flash chip */
#define OPCODE_SE       0xd8UL  /* Sector erase (usually 64KiB) */
#define OPCODE_RDID     0x9fUL  /* Read JEDEC ID */

/* Define max times to check status register before we give up. */
#define MAX_READY_WAIT_COUNT    100000
#define CMD_SIZE        4

#ifdef CONFIG_MTD_PARTITIONS
#define mtd_has_partitions()    (1)
#else
#define mtd_has_partitions()    (0)
#endif

/****************************************************************************/
static unsigned g_spiclk;
struct mst_serflash_dev {
    struct platform_device* pdev;
    u8     flash_freq;
    u8     flash_readmod;
};

struct serflash {
    struct mutex        lock;
    struct mtd_info     mtd;
    unsigned            partitioned:1;
    u8                  erase_opcode;
};

static inline struct serflash *mtd_to_serflash(struct mtd_info *mtd)
{
    return container_of(mtd, struct serflash, mtd);
}

/* Erase flash fully or part of it */
static int serflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    struct serflash *flash = mtd_to_serflash(mtd);
    //ulong addr,len,start_sec,end_sec;
    u32 addr,len;

    printk(KERN_DEBUG"%s: addr 0x%08x, len 0x%08x\n",
    __func__, (u32)instr->addr, (u32)instr->len);
    addr = (u32)instr->addr;
    len = (u32)instr->len;

    /* range and alignment check */
    if (instr->addr + instr->len > mtd->size)
        return -EINVAL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
    if ((do_div(instr->addr , mtd->erasesize)) != 0 ||(do_div(instr->len, mtd->erasesize) != 0))
#else
    if ((instr->addr % mtd->erasesize) != 0 || (instr->len % mtd->erasesize) != 0)
#endif
        return -EINVAL;

    instr->addr = addr;
    instr->len = len;

    mutex_lock(&flash->lock);

#ifdef  CONFIG_ADD_WRITE_PROTECT
    /*write protect false before erase*/
    if (!MDrv_SERFLASH_WriteProtect(0))
    {
        mutex_unlock(&flash->lock);
        return -EIO;
    }
#endif
    /* erase the whole chip */
    if (len == mtd->size && !MDrv_SERFLASH_EraseChip())
    {
        instr->state = MTD_ERASE_FAILED;
#ifdef  CONFIG_ADD_WRITE_PROTECT
        MDrv_SERFLASH_WriteProtect(1);
#endif
        mutex_unlock(&flash->lock);
        return -EIO;
    }
    else
    {
        if (len)
        {
            if (!MDrv_SERFLASH_AddressErase(addr, len, 1))
            {
                instr->state = MTD_ERASE_FAILED;
#ifdef  CONFIG_ADD_WRITE_PROTECT
                MDrv_SERFLASH_WriteProtect(1);
#endif
                mutex_unlock(&flash->lock);
                return -EIO;
            }
        }
    }
#ifdef  CONFIG_ADD_WRITE_PROTECT
    MDrv_SERFLASH_WriteProtect(1);
#endif
    mutex_unlock(&flash->lock);

    instr->state = MTD_ERASE_DONE;

    mtd_erase_callback(instr);

    return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int serflash_read(struct mtd_info *mtd, loff_t from, size_t len,
    size_t *retlen, u_char *buf)
{
    struct serflash *flash = mtd_to_serflash(mtd);

    printk(KERN_DEBUG"%s %s 0x%08x, len %zd\n", __func__, "from", (u32)from, len);

    /* sanity checks */
    if (!len)
        return 0;

    if (from + len > flash->mtd.size)
        return -EINVAL;

    mutex_lock(&flash->lock);

    #if 0
    /* Wait till previous write/erase is done. */
    if (wait_till_ready(flash)) {
        /* REVISIT status return?? */
        mutex_unlock(&flash->lock);
        return 1;
    }
    #endif

       if (MDrv_SERFLASH_Read(from, len, (unsigned char *)buf))
       {
           *retlen = len;
       }
       else
       {
           *retlen = 0;
           mutex_unlock(&flash->lock);
           return -EIO;
       }

    mutex_unlock(&flash->lock);

    return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int serflash_write(struct mtd_info *mtd, loff_t to, size_t len,
    size_t *retlen, const u_char *buf)
{
    struct serflash *flash = mtd_to_serflash(mtd);
       u32 erase_start, erase_end;

    printk(KERN_DEBUG"%s %s 0x%08x, len %zd\n", __func__, "to", (u32)to, len);

    if (retlen)
        *retlen = 0;

    /* sanity checks */
    if (!len)
        return(0);

    if (to + len > flash->mtd.size)
        return -EINVAL;

       // calculate erase sectors
       erase_start = (u32)to / mtd->erasesize;
       erase_end = (u32)(to+len) / mtd->erasesize - 1;

    mutex_lock(&flash->lock);

    #if 0
    /* Wait until finished previous write command. */
    if (wait_till_ready(flash)) {
        mutex_unlock(&flash->lock);
        return 1;
    }
    #endif
#ifdef  CONFIG_ADD_WRITE_PROTECT
    if (!MDrv_SERFLASH_WriteProtect(0))
    {
        mutex_unlock(&flash->lock);
        return -EIO;
    }
#endif
//modified by daniel.lee 2010/0514
/*
       if (!MDrv_SERFLASH_BlockErase(erase_start, erase_end, TRUE))
       {
           mutex_unlock(&flash->lock);
        return -EIO;
       }
*/
       if (MDrv_SERFLASH_Write(to, len, (unsigned char *)buf))
       {
           if (retlen)
               *retlen = len;
       }
       else
       {
           if (retlen)
               *retlen = 0;
#ifdef  CONFIG_ADD_WRITE_PROTECT
           MDrv_SERFLASH_WriteProtect(1);
#endif
           mutex_unlock(&flash->lock);
           return -EIO;
       }
#ifdef  CONFIG_ADD_WRITE_PROTECT
       MDrv_SERFLASH_WriteProtect(1);
#endif
    mutex_unlock(&flash->lock);

    return 0;
}

/* SPI device information structure */
struct flash_info {
    char        *name;

    u32     jedec_id; //no ID or manufacturer id & two byte device id
    u16            ext_id;

    unsigned    sector_size;
    u16     n_sectors;

    u16     flags;
  #define   SECT_4K     0x01UL
};

/* chip list */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,13)
static struct flash_info serflash_data [] = {
#else
static struct flash_info __devinitdata serflash_data [] = {
#endif
    { "MX25L3205D",     0xc22016,   0,  64 * 1024,  64, },
    { "MX25L6405D",     0xc22017,   0,  64 * 1024,  128,},
    { "MX25L12805D",    0xc22018,   0,  64 * 1024,  256,},
    { "MX25L12855E",    0xc22618,   0,  64 * 1024,  256,},
    { "MX25L6456E",     0xc22617,   0,  64 * 1024,  128,},
    { "W25X32",         0xef3016,   0,  64 * 1024,  64, },
    { "W25Q32",         0xef4016,   0,  64 * 1024,  64, },
    { "W25X64",         0xef3017,   0,  64 * 1024,  128,},
    { "W25Q64",         0xef4017,   0,  64 * 1024,  128,},
    { "W25Q128",        0xef4018,   0,  64 * 1024,  256,},
    { "AT26DF321",      0x1f4700,   0,  64 * 1024,  64, },
    { "STM25P32",       0x202016,   0,  64 * 1024,  64, },
    { "EN25B32B",       0x1c2016,   0,  64 * 1024,  68, },
    { "EN25B64B",       0x1c2017,   0,  64 * 1024,  132,},
    { "S25FL128P",      0x012018,   0,  64 * 1024,  256,},
    { "GD25Q128",       0xC84018,   0,  64 * 1024,  256,},
};

#define MSTAR_SERFLASH_SIZE                 (8 * 1024 * 1024)

#define SERFLASH_PART_PARTITION_TBL_OFFSET      0
#define SERFLASH_PART_PARTITION_TBL_SIZE        512 * 1024

#define SERFLASH_PART_LINUX_BOOT_PARAM_OFFSET   (SERFLASH_PART_PARTITION_TBL_OFFSET + SERFLASH_PART_PARTITION_TBL_SIZE)
#define SERFLASH_PART_LINUX_BOOT_PARAM_SIZE     512 * 1024

#define SERFLASH_PART_KERNEL_OFFSET             (SERFLASH_PART_LINUX_BOOT_PARAM_OFFSET + SERFLASH_PART_LINUX_BOOT_PARAM_SIZE)
#define SERFLASH_PART_KERNEL_SIZE               1536 * 1024

#define SERFLASH_PART_ROOTFS_OFFSET             (SERFLASH_PART_KERNEL_OFFSET + SERFLASH_PART_KERNEL_SIZE)
#define SERFLASH_PART_ROOTFS_SIZE               2560 * 1024

#define SERFLASH_PART_CONF_OFFSET               (SERFLASH_PART_ROOTFS_OFFSET + SERFLASH_PART_ROOTFS_SIZE)
#define SERFLASH_PART_CONF_SIZE                 64 * 1024

#if 0
#define SERFLASH_PART_CHAKRA_BOOT_PARAM_OFFSET  (NAND_PART_KERNEL_OFFSET + NAND_PART_KERNEL_SIZE)
#define SERFLASH_PART_CHAKRA_BOOT_PARAM_SIZE    SZ_512KB

#define SERFLASH_PART_CHAKRA_BIN_OFFSET         (NAND_PART_CHAKRA_BOOT_PARAM_OFFSET + NAND_PART_CHAKRA_BOOT_PARAM_SIZE)
#define SERFLASH_PART_CHAKRA_BIN_PARAM_SIZE     SZ_8MB

#define SERFLASH_PART_SUBSYSTEM_OFFSET          (NAND_PART_CONF_OFFSET + NAND_PART_CONF_SIZE)
#define SERFLASH_PART_SUBSYSTEM_SIZE            SZ_2MB

#define SERFLASH_PART_FONT_OFFSET               (NAND_PART_SUBSYSTEM_OFFSET + NAND_PART_SUBSYSTEM_SIZE)
#define SERFLASH_PART_FONT_SIZE                 SZ_4MB

#define SERFLASH_PART_OPT_OFFSET                (NAND_PART_FONT_OFFSET + NAND_PART_FONT_SIZE)
#define SERFLASH_PART_OPT_SIZE                  SZ_8MB

#define SERFLASH_PART_APPLICATION_OFFSET        (NAND_PART_OPT_OFFSET + NAND_PART_OPT_SIZE)
#define SERFLASH_PART_APPLICATION_SIZE          (MSTAR_NAND_SIZE - NAND_PART_APPLICATION_OFFSET)
#endif

#if ( (NAND_PART_APPLICATION_OFFSET) >= MSTAR_SERFLASH_SIZE)
    #error "Error: NAND partition is not correct!!!"
#endif

static const struct mtd_partition serflash_partition_info[] =
{
    {
        .name   = "nand_partition_tbl",
        .offset = SERFLASH_PART_PARTITION_TBL_OFFSET,
        .size   = SERFLASH_PART_PARTITION_TBL_SIZE
    },
    {
        .name   = "linux_boot_param",
        .offset = SERFLASH_PART_LINUX_BOOT_PARAM_OFFSET,
        .size   = SERFLASH_PART_LINUX_BOOT_PARAM_SIZE
    },
    {
        .name   = "kernel",
        .offset = SERFLASH_PART_KERNEL_OFFSET,
        .size   = SERFLASH_PART_KERNEL_SIZE
    },
    {
        .name   = "rootfs",
        .offset = SERFLASH_PART_ROOTFS_OFFSET,
        .size   = SERFLASH_PART_ROOTFS_SIZE
    },
    {
    .name   = "conf",
    .offset = SERFLASH_PART_CONF_OFFSET,
    .size   = SERFLASH_PART_CONF_SIZE
    },
#if 0
    {
        .name   = "chakra_boot_param",
        .offset = NAND_PART_CHAKRA_BOOT_PARAM_OFFSET,
        .size   = NAND_PART_CHAKRA_BOOT_PARAM_SIZE
    },
    {
        .name   = "charkra_bin",
        .offset = NAND_PART_CHAKRA_BIN_OFFSET,
        .size   = NAND_PART_CHAKRA_BIN_PARAM_SIZE
    },
    {
        .name   = "subsystem",
        .offset = NAND_PART_SUBSYSTEM_OFFSET,
        .size   = NAND_PART_SUBSYSTEM_SIZE
    },
    {
        .name   = "font",
        .offset = NAND_PART_FONT_OFFSET,
        .size   = NAND_PART_FONT_SIZE
    },
    {
        .name   = "opt",
        .offset = NAND_PART_OPT_OFFSET,
        .size   = NAND_PART_OPT_SIZE
    },
    {
        .name   = "application",
        .offset = NAND_PART_APPLICATION_OFFSET,
        .size   = NAND_PART_APPLICATION_SIZE
    },
#endif
};

#define SERFLASH_NUM_PARTITIONS ARRAY_SIZE(serflash_partition_info)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,13)
static struct flash_info *jedec_probe(void)
#else
static struct flash_info *__devinit jedec_probe(void)
#endif
{
    int         tmp;
    u8          id[5];
    u32         jedec;
    u16                     ext_jedec;
    struct flash_info   *info;

    /* JEDEC also defines an optional "extended device information"
     * string for after vendor-specific data, after the three bytes
     * we use here.  Supporting some chips might require using it.
     */
       tmp = MDrv_SERFLASH_ReadID(id, 5);
    if (!tmp)
    {
        printk(KERN_ERR"error %d reading JEDEC ID\n", tmp);
        return NULL;
    }
    jedec = id[0];
    jedec = jedec << 8;
    jedec |= id[1];
    jedec = jedec << 8;
    jedec |= id[2];

    ext_jedec = id[3] << 8 | id[4];

    for (tmp = 0, info = serflash_data; tmp < ARRAY_SIZE(serflash_data); tmp++, info++)
    {
        if (info->jedec_id == jedec)
        {
            if (info->ext_id != 0 && info->ext_id != ext_jedec)
                continue;
            return info;
        }
    }
    //dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
    return NULL;
}

static int mstserflash_parse_dt(struct device_node *dn, struct mst_serflash_dev *serflash_dev)
{
    u32 speed_Mhz = 0;
    u32 readmode = 0;
    /*
    (1) Parse Device Tree
    */
    if (0 != of_property_read_u32(dn, "speed-Mhz", &speed_Mhz))
    {
          printk("[mst][kserflash]Parse freq dts error\n");
      return -ENXIO;
    }
    if (0 != of_property_read_u32(dn, "read-mode", &readmode))
    {
          printk("[mst][kserflash]Parse readmod dts error\n");
      return -ENXIO;
    }
    /*
    (3) Assign value
    */
    serflash_dev->flash_freq = (u8)speed_Mhz;
    serflash_dev->flash_readmod = (u8)readmode;
    return 0;
}

static void mst_serflash_clk(u32 spiclk)
{
   /*
    * FIXME:
    *   Setup spi clock from clock tree.
    */
    extern MS_BOOL HAL_SERFLASH_CLK_Cfg(u32 u32spiClk);
    HAL_SERFLASH_CLK_Cfg(spiclk);
}

static unsigned mst_serflash_clk_get(void)
{
   unsigned clktmp;
   extern MS_BOOL HAL_SERFLASH_CLK_Get(MS_U8 *u8MspiClk);
   HAL_SERFLASH_CLK_Get(&clktmp);
   return clktmp;
}

static bool mst_serflash_readmode_set(u8 flashmode)
{
   bool rtn;
   extern MS_BOOL HAL_SERFLASH_ReadModeSet(MS_U8 u8ModeSet);
   rtn = HAL_SERFLASH_ReadModeSet(flashmode);
   return rtn;
}

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __init serflash_probe(struct platform_device *pdev)
{
    struct serflash         *flash;
    struct flash_info       *info;
    unsigned            i;
    struct device_node *dn;
    struct mst_serflash_dev *serspi_dev;
    int retval = 0;
       // jedec_probe() will read id, so initialize hardware first
       MDrv_SERFLASH_Init();

    /* Platform data helps sort out which chip type we have, as
     * well as how this board partitions it.  If we don't have
     * a chip ID, try the JEDEC id commands; they'll work for most
     * newer chips, even if we don't recognize the particular chip.
     */
    info = jedec_probe();

    if (!info)
        return -ENODEV;

    flash = kzalloc(sizeof *flash, GFP_KERNEL);
    if (!flash)
        return -ENOMEM;

    mutex_init(&flash->lock);

       flash->mtd.priv = flash;

    /* parse OF serflash info */
    #if defined (CONFIG_OF)
        /* init serflash device struct */
    serspi_dev = devm_kzalloc(&pdev->dev, sizeof(struct mst_serflash_dev), GFP_KERNEL);
    if (!serspi_dev)
    {
        dev_err(&pdev->dev, "Unable to allocate memory for mst serflash\n");
        return -ENOMEM;
    }
    dn = pdev->dev.of_node;
    if (dn)
    {
        retval = mstserflash_parse_dt(dn, serspi_dev);
        if (retval < 0)
        {
            printk("[kserflash] unable to parse device tree\n");
            return -ENODEV;
        }
    }
    if(!mst_serflash_readmode_set(serspi_dev->flash_readmod))
    {
        printk("[kserflash read mode not support]\n");
        return -ENODEV;
    }
    if(((serspi_dev->flash_freq)>0)&&((serspi_dev->flash_freq)<=54))
    {
        printk("serspi_dev freq:%x\n",serspi_dev->flash_freq);
        mst_serflash_clk((serspi_dev->flash_freq)*1000000);
    }
    else
    {
        printk(KERN_DEBUG"Not Support SPI CLK:0x%llx",(u32)serspi_dev->flash_freq);
    }


       #endif
#if 0
    /*
     * Atmel serial flash tend to power up
     * with the software protection bits set
     */

    if (info->jedec_id >> 16 == 0x1fUL)
    {
        write_enable(flash);
        write_sr(flash, 0);
    }
#endif

    flash->mtd.type = MTD_NORFLASH;
    flash->mtd.writesize = 1;
    flash->mtd.flags = MTD_CAP_NORFLASH;
    //flash->mtd.size = info->sector_size * info->n_sectors;
    MDrv_SERFLASH_DetectSize((MS_U32*)&flash->mtd.size);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,13)
    flash->mtd._erase = serflash_erase;
    flash->mtd._read = serflash_read;
    flash->mtd._write = serflash_write;
#else
    flash->mtd.erase = serflash_erase;
    flash->mtd.read = serflash_read;
    flash->mtd.write = serflash_write;
#endif

    /* prefer "small sector" erase if possible */
    if (info->flags & SECT_4K)
    {
        flash->erase_opcode = OPCODE_BE_4K;
        flash->mtd.erasesize = 4096;
    }
    else
    {
        flash->erase_opcode = OPCODE_SE;
        flash->mtd.erasesize = info->sector_size;
    }

    //dev_info(&spi->dev, "%s (%d Kbytes)\n", info->name,
            //flash->mtd.size / 1024);

    printk(KERN_DEBUG
        "mtd .name = %s, .size = 0x%.8x (%uMiB) "
            ".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
        flash->mtd.name,
        flash->mtd.size, flash->mtd.size / (1024*1024),
        flash->mtd.erasesize, flash->mtd.erasesize / 1024,
        flash->mtd.numeraseregions);

    if (flash->mtd.numeraseregions)
        for (i = 0; i < flash->mtd.numeraseregions; i++)
            printk(KERN_DEBUG
                "mtd.eraseregions[%d] = { .offset = 0x%.8x, "
                ".erasesize = 0x%.8x (%uKiB), "
                ".numblocks = %d }\n",
                i, flash->mtd.eraseregions[i].offset,
                flash->mtd.eraseregions[i].erasesize,
                flash->mtd.eraseregions[i].erasesize / 1024,
                flash->mtd.eraseregions[i].numblocks);

    /* partitions should match sector boundaries; and it may be good to
     * use readonly partitions for writeprotected sectors (BP2..BP0).
     */
    if (mtd_has_partitions())
    {
        struct mtd_partition    *parts = NULL;
        int    nr_parts = 0;
#if 0
#ifdef CONFIG_MTD_CMDLINE_PARTS
        static const char *part_probes[] = { "cmdlinepart", NULL, };

        nr_parts = parse_mtd_partitions(&flash->mtd,
            part_probes, &parts, 0);
#endif
#endif
        if (nr_parts > 0)
        {
            for (i = 0; i < nr_parts; i++)
            {
                printk(KERN_DEBUG"partitions[%d] = "
                        "{.name = %s, .offset = 0x%.8x, "
                        ".size = 0x%.8x (%uKiB) }\n",
                    i, parts[i].name,
                    parts[i].offset,
                    parts[i].size,
                    parts[i].size / 1024);
            }
            flash->partitioned = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
            return mtd_device_register(&flash->mtd, parts, nr_parts);
#else
            return add_mtd_partitions(&flash->mtd, parts, nr_parts);
#endif
        }
        else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
            return mtd_device_register(&flash->mtd, serflash_partition_info, SERFLASH_NUM_PARTITIONS);
#else
            return add_mtd_partitions(&flash->mtd, serflash_partition_info, SERFLASH_NUM_PARTITIONS);
#endif
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
    return mtd_device_register(&flash->mtd, NULL, NULL);
#else
    return add_mtd_device(&flash->mtd) == 1 ? -ENODEV : 0;
#endif
}


static int mst_serflash_remove(struct platform_device *pdev)
{
    if( !(pdev->name) || strcmp(pdev->name,"mst,serflash")
        || pdev->id!=0)
    {
        return -1;
    }
    pdev->dev.platform_data = NULL;
    return 0;
}

static int mst_serflash_suspend(struct platform_device *pdev)
{
     struct mst_serflash_dev *serspi_dev;
     int ret = 0;

     serspi_dev = (struct mst_serflash_dev *)platform_get_drvdata(pdev);
     BUG_ON(serspi_dev->flash_freq == 0);
     g_spiclk = mst_serflash_clk_get();
     if(g_spiclk==0)
     {
         printk("[kserflash]clk gating");
         return -1;
     }
     return ret;
}

static int mst_serflash_resume(struct platform_device *pdev)
{
    struct mst_serflash_dev *serspi_dev;
    int ret = 0;

    serspi_dev = (struct mst_serflash_dev *)platform_get_drvdata(pdev);
    BUG_ON(serspi_dev->flash_freq == 0);
    if(g_spiclk==0)
    {
       printk("[kserflash]clk gating");
       return -1;
    }
    mst_serflash_clk(g_spiclk*1000000);
    return ret;
}


#if defined (CONFIG_OF)
static struct of_device_id mstserflash_of_device_ids[] = {
        {.compatible = "mst,serflash"},
        {},
};
#endif

static struct platform_driver Mst_serflash_driver = {
    .probe      = serflash_probe,
    .remove     = mst_serflash_remove,
    .suspend    = mst_serflash_suspend,
    .resume     = mst_serflash_resume,

    .driver = {
#if defined(CONFIG_OF)
        .of_match_table = mstserflash_of_device_ids,
#endif
        .name   = "Mst-serflash",
        .owner  = THIS_MODULE,
    }
};


static int __init mst_serlash_init_module(void)
{
    int retval=0;
    printk("[kserflash] %s\n",__FUNCTION__);
    retval = platform_driver_register(&Mst_serflash_driver);
    return retval;
}

static void __exit mst_serlash_exit_module(void)
{
    platform_driver_unregister(&Mst_serflash_driver);
}

module_init(mst_serlash_init_module);
module_exit(mst_serlash_exit_module);

MODULE_AUTHOR("MST");
MODULE_DESCRIPTION("MTD Mstar driver for spi flash chips");
MODULE_LICENSE("GPL");


