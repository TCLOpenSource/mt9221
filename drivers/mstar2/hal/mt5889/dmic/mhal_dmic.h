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


#ifndef _MHAL_ALSA_DRIVER_HEADER
#define _MHAL_ALSA_DRIVER_HEADER

/*
 * ============================================================================
 * Include Headers
 * ============================================================================
 */
#include "mdrv_public.h"
#include "mhal_version.h"


/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
#define DMIC_ctrl    0x2402 //[5:4] 0:1ch, 1:2ch, 2:4ch, 3:8ch
#define VDMA_ctrl    0x2460
#define VDMA_WR_RESET    (1<<15)
#define VDMA_WR_BIT_MODE    (1<<9)
#define VDMA_WR_LEVEL_MASK    (1<<5)
#define VDMA_WR_TRIG    (1<<4)
#define VDMA_WR_LR_SWAP    (1<<3)
#define VDMA_WR_FREE_RUN    (1<<2)
#define VDMA_WR_ENABLE    (1<<1)
#define VDMA_WR_MIU_ENABLE    (1<<0)
#define VDMA_BaseAddress_Lo    0x2462
#define VDMA_BaseAddress_Hi    0x2464
#define VDMA_DRAM_size    0x2466
#define VDMA_DRAM_readsize    0x2468
#define VDMA_DRAM_overrun_threshold    0x246A
#define VDMA_DRAM_levelcnt    0x2474
#define VDMA_Channel_mode    0x2480 //mch, 0:2ch, 1:4ch, 2:6ch, 3:8ch

#define VDMA_LOCALBUF_SIZE    (64) //bytes
#define VDMA_vrec_ctrl_4	0x2408

#define AECDMA_ctrl    0x25C0
#define AECDMA_WR_RESET    (1<<15)
#define AECDMA_WR_BIT_MODE    (1<<9)
#define AECDMA_WR_LOCAL_FULL_FLAG_CLR    (1<<8)
#define AECDMA_WR_FULL_FLAG_CLR    (1<<7)
#define AECDMA_WR_MIU_HIGH_PRIORITY    (1<<6)
#define AECDMA_WR_LEVEL_MASK    (1<<5)
#define AECDMA_WR_TRIG    (1<<4)
#define AECDMA_WR_LR_SWAP    (1<<3)
#define AECDMA_WR_FREE_RUN    (1<<2)
#define AECDMA_WR_ENABLE    (1<<1)
#define AECDMA_WR_MIU_ENABLE    (1<<0)
#define AECDMA_BaseAddress_Lo    0x25C2
#define AECDMA_BaseAddress_Hi    0x25C4
#define AECDMA_DRAM_size    0x25C6
#define AECDMA_DRAM_readsize    0x25C8
#define AECDMA_DRAM_overrun_threshold    0x25CA
#define AECDMA_DRAM_levelcnt    0x25D4
#define AECDMA_Channel_mode    0x2240 //mch, 0:2ch, 1:4ch, 2:6ch, 3:8ch

#define AECDMA_LOCALBUF_SIZE    (64) //bytes
#define AECDMA_src_ctrl_1    0x2202
#define AECDMA_src_ctrl_7    0x220E
#define AECDMA_MCH_ENABLE    (1<<0)
#define AECDMA_Enable    (1<<15)

#define DMIC_CLK_ctrl	0x2920
#define DMIC_HFP_ctrl	0x24A6


/* D-MIC channel mode */
typedef enum
{
	DMIC_CHANNEL_1 = 0,
	DMIC_CHANNEL_2,
	DMIC_CHANNEL_4,
	DMIC_CHANNEL_6,
	DMIC_CHANNEL_8,
}DmicChannel_e;

/* VDMA channel mode */
typedef enum
{
	VDMA_CHANNEL_2 = 0,
	VDMA_CHANNEL_4,
	VDMA_CHANNEL_6,
	VDMA_CHANNEL_8,
}VdmaChannel_e;

/* REF channel mode */
typedef enum
{
	REF_CHANNEL_2 = 0,
	REF_CHANNEL_4,
	REF_CHANNEL_6,
	REF_CHANNEL_8,
}RefChannel_e;

/* Define a Ring Buffer data structure for MStar Audio DSP */
struct MStar_Ring_Buffer_Struct {
	unsigned char *addr;
	unsigned int size;
	unsigned char *w_ptr;
	unsigned char *r_ptr;
};

/* Define a STR (Suspend To Ram) data structure for MStar Audio DSP */
 struct MStar_STR_MODE_Struct {
	ptrdiff_t physical_addr;
	ptrdiff_t bus_addr;
	ptrdiff_t virtual_addr;
};


/* Define a DMA Reader data structure for MStar Audio DSP */
struct MStar_PCM_DMIC_Capture_Struct {
	struct MStar_Ring_Buffer_Struct buffer;
	struct MStar_Ring_Buffer_Struct aec_buffer;
	struct MStar_STR_MODE_Struct str_mode_info;
	struct MStar_STR_MODE_Struct aec_str_mode_info;
	unsigned int initialized_status;
	unsigned int channel_mode;
	unsigned int sample_rate;
	unsigned int status;
	unsigned int bit_width;
	int vdma_channel;
	int aec_channel;
    unsigned char dmic_enable;
    unsigned char hpf_switch;
    unsigned char hpf_config;
    unsigned char sine_gen;
    unsigned int gain;
};

/*
 * ============================================================================
 * Forward Declarations
 * ============================================================================
 */
extern void Chip_Flush_Memory(void);

#endif /* _MHAL_ALSA_DRIVER_HEADER */

