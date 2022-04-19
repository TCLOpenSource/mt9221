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
/// @file   mdrv_sata_host.h
/// @author MStar Semiconductor Inc.
/// @brief  SATA Host Driver
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _MDRV_SATA_HOST_H_
#define _MDRV_SATA_HOST_H_

#define TYPE_XIU 0
#define TYPE_DRAM 1
#define TYPE_RIU 2

#define SATA_CMD_TYPE TYPE_DRAM
//#define SATA_CMD_TYPE TYPE_RIU
//#define SATA_CMD_TYPE TYPE_XIU

#define USE_NCQ //Only Support at TYPE DRAM Mode

enum {
#if defined(USE_NCQ)
    SATA_KA9_HOST_FLAGS = ATA_FLAG_SATA | ATA_FLAG_PIO_DMA |
                      ATA_FLAG_ACPI_SATA | ATA_FLAG_AN | ATA_FLAG_NCQ,
    SATA_KA9_QUEUE_DEPTH    = 31,
#else
    SATA_KA9_HOST_FLAGS = ATA_FLAG_SATA | ATA_FLAG_PIO_DMA |
                      ATA_FLAG_ACPI_SATA | ATA_FLAG_AN,
    SATA_KA9_QUEUE_DEPTH    = 1,
#endif
    SATA_KA9_USED_PRD    = 24,
    SATA_KA9_MAX_PRD    = 24,
    SATA_KA9_CMD_HDR_SIZE   = 0x20,

    SATA_KA9_CMD_DESC_CFIS_SZ   = 64,
    SATA_KA9_CMD_DESC_ACMD_SZ   = 16,
    SATA_KA9_CMD_DESC_RSRVD     = 48,

    SATA_KA9_CMD_DESC_SIZE  = (SATA_KA9_CMD_DESC_CFIS_SZ +
                 SATA_KA9_CMD_DESC_ACMD_SZ +
                 SATA_KA9_CMD_DESC_RSRVD +
                 SATA_KA9_MAX_PRD * 16),

    SATA_KA9_CMD_DESC_OFFSET_TO_PRDT    =
                (SATA_KA9_CMD_DESC_CFIS_SZ +
                 SATA_KA9_CMD_DESC_ACMD_SZ +
                 SATA_KA9_CMD_DESC_RSRVD),
};

typedef struct sata_cmd_header
{
    u8     cmd_fis_len : 5;
    u8     isATA_PI    : 1;
    u8     iswrite     : 1;
    u8     isprefetch  : 1; // enable only PRDT not zero
    u8     issoftreset : 1;
    u8     isbist      : 1;
    u8     isclearok   : 1;
    u8     reserverd   : 1;
    u8     PMPid       : 4;
    u16    PRDTlength  ;
    u32    PRDBytes    ;
    u32    ctba_lbase  ; // 0~6 is reserved
    u32    ctba_hbase  ;

}hal_cmd_header;


typedef struct sata_cmd_h2dfis
{
    u8 u8fis_type       ;
    u8 u8MPM           : 4;
    u8 reserved_0      : 3;
    u8 isclear         : 1;
    u8 ata_cmd          ;
    u8 fearure          ;
    u8 lba_l            ;
    u8 lba_m            ;
    u8 lba_h            ;
    u8 device           ;
    u8 lba_l_exp        ;
    u8 lba_m_exp        ;
    u8 lba_h_exp        ;
    u8 fearure_exp      ;
    u16 u16sector_cnt   ;
    u8 reserved_1       ;
    u8 control          ;
    u32 reserved_2      ;
}hal_cmd_h2dfis;

struct sata_mstar_port_priv {
    void	*cmd_slot;
    void	*cmd_tbl;
    void	*rx_fis;
    dma_addr_t	cmd_slot_dma;
    dma_addr_t	cmd_tbl_dma;
    dma_addr_t	rx_fis_dma;
};

struct sata_mstar_host_priv {
    phys_addr_t hba_base;
    phys_addr_t port_base;
    phys_addr_t misc_base;
    void *mem;
    dma_addr_t mem_dma;
};

#endif
