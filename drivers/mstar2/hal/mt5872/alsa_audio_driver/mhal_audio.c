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
 * ============================================================================
 * Include Headers
 * ============================================================================
 */
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,1))
#include <linux/module.h>
#endif
#include <asm/io.h>

#include <linux/fs.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#include "mstar/mstar_chip.h"
#include "mhal_audio.h"


/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define _MAD_PM_MODE_REG_BASE    mstar_pm_base
#else
#define _MAD_PM_MODE_REG_BASE    0xfd000000
#endif
#define _MAD_NON_PM_MODE_REG_BASE    0xfd200000
#define _MAD_PHYSICAL_MEM_BASE    _MAD_PM_MODE_REG_BASE

#define _MAD_BYTES_IN_LINE    16
#define _MAD_ADDR_CONVERTOR    0x1000
#define _MAD_MAILBOX_OFFSET    0x110000
#define _MAD_MAILBOX_OFFSET2    0x160000

#define _MAD_MAX_CHANNEL    2

#define _MAD_DSP2_DRAM_SIZE    0x280000

#define _MAD_DMA_READER_BASE_OFFSET    0x00000000
#define _MAD_DMA_READER_BUF_UNIT    4096
#define _MAD_DMA_READER_TOTAL_BUF_SIZE    0x10000 /* 64 KByte */
#define _MAD_DMA_READER_MIN_AVAIL    (_MAD_DMA_READER_TOTAL_BUF_SIZE >> 3) /* The minimal available size should be reserved */
#define _MAD_DMA_READER_HIGH_THRESHOLD    (_MAD_DMA_READER_TOTAL_BUF_SIZE - _MAD_DMA_READER_MIN_AVAIL)
#define _MAD_DMA_READER_BUF_SIZE    _MAD_DMA_READER_TOTAL_BUF_SIZE
#define _MAD_DMA_READER_PERIOD_SIZE    (_MAD_DMA_READER_BUF_SIZE >> 2)
#define _MAD_DMA_READER_QUEUE_SIZE    100 /* ms */

#define _MAD_DMA_READER2_BASE_OFFSET    0x147000
#define _MAD_DMA_READER2_BUF_UNIT    4096
#define _MAD_DMA_READER2_TOTAL_BUF_SIZE    0x10000 /* 64 KByte */
#define _MAD_DMA_READER2_MIN_AVAIL    (_MAD_DMA_READER2_TOTAL_BUF_SIZE >> 3) /* The minimal available size should be reserved */
#define _MAD_DMA_READER2_HIGH_THRESHOLD    (_MAD_DMA_READER2_TOTAL_BUF_SIZE - _MAD_DMA_READER2_MIN_AVAIL)
#define _MAD_DMA_READER2_BUF_SIZE    _MAD_DMA_READER2_TOTAL_BUF_SIZE
#define _MAD_DMA_READER2_PERIOD_SIZE    (_MAD_DMA_READER2_BUF_SIZE >> 2)
#define _MAD_DMA_READER2_QUEUE_SIZE    100 /* ms */

#define _MAD_PCM_PLAYBACK3_BASE_OFFSET    0x20000
#define _MAD_PCM_PLAYBACK3_BUF_UNIT    4096
#define _MAD_PCM_PLAYBACK3_TOTAL_BUF_SIZE    0x20000 /* 128 KByte */
#define _MAD_PCM_PLAYBACK3_MIN_AVAIL    (_MAD_PCM_PLAYBACK3_TOTAL_BUF_SIZE >> 3) /* The minimal available size should be reserved */
#define _MAD_PCM_PLAYBACK3_HIGH_THRESHOLD    (_MAD_PCM_PLAYBACK3_TOTAL_BUF_SIZE - _MAD_PCM_PLAYBACK3_MIN_AVAIL)
#define _MAD_PCM_PLAYBACK3_BUF_SIZE    (_MAD_PCM_PLAYBACK3_TOTAL_BUF_SIZE >> 1) /* Only half size can be used */
#define _MAD_PCM_PLAYBACK3_PERIOD_SIZE    (_MAD_PCM_PLAYBACK3_BUF_SIZE >> 2)
#define _MAD_PCM_PLAYBACK3_QUEUE_SIZE    100 /* ms */

#define _MAD_PCM_PLAYBACK4_BASE_OFFSET    0x788000
#define _MAD_PCM_PLAYBACK4_BUF_UNIT    4096
#define _MAD_PCM_PLAYBACK4_TOTAL_BUF_SIZE    0x20000 /* 128 KByte */
#define _MAD_PCM_PLAYBACK4_MIN_AVAIL    (_MAD_PCM_PLAYBACK4_TOTAL_BUF_SIZE >> 3) /* The minimal available size should be reserved */
#define _MAD_PCM_PLAYBACK4_HIGH_THRESHOLD    (_MAD_PCM_PLAYBACK4_TOTAL_BUF_SIZE - _MAD_PCM_PLAYBACK4_MIN_AVAIL)
#define _MAD_PCM_PLAYBACK4_BUF_SIZE    (_MAD_PCM_PLAYBACK4_TOTAL_BUF_SIZE >> 1) /* Only half size can be used */
#define _MAD_PCM_PLAYBACK4_PERIOD_SIZE    (_MAD_PCM_PLAYBACK4_BUF_SIZE >> 2)
#define _MAD_PCM_PLAYBACK4_QUEUE_SIZE    100 /* ms */

#define _MAD_PCM_PLAYBACK7_BASE_OFFSET    0x778000
#define _MAD_PCM_PLAYBACK7_BUF_UNIT    4096
#define _MAD_PCM_PLAYBACK7_TOTAL_BUF_SIZE    0x20000 /* 128 KByte */
#define _MAD_PCM_PLAYBACK7_MIN_AVAIL    (_MAD_PCM_PLAYBACK7_TOTAL_BUF_SIZE >> 3) /* The minimal available size should be reserved */
#define _MAD_PCM_PLAYBACK7_HIGH_THRESHOLD    (_MAD_PCM_PLAYBACK7_TOTAL_BUF_SIZE - _MAD_PCM_PLAYBACK7_MIN_AVAIL)
#define _MAD_PCM_PLAYBACK7_BUF_SIZE    (_MAD_PCM_PLAYBACK7_TOTAL_BUF_SIZE >> 1) /* Only half size can be used */
#define _MAD_PCM_PLAYBACK7_PERIOD_SIZE    (_MAD_PCM_PLAYBACK7_BUF_SIZE >> 2)
#define _MAD_PCM_PLAYBACK7_QUEUE_SIZE    100 /* ms */

#define _MAD_PCM_CAPTURE1_BASE_OFFSET   0x30000
#define _MAD_PCM_CAPTURE2_BASE_OFFSET   0x3C000
#define _MAD_PCM_CAPTURE3_BASE_OFFSET   0x810000
#define _MAD_PCM_CAPTURE_BUF_UNIT    128
#define _MAD_PCM_CAPTURE_BUF_SIZE    0xC000 /* 48 KByte */

#define _MAD_READ_BYTE(_reg)    (*(volatile unsigned char*)(_reg))
#define _MAD_READ_WORD(_reg)    (*(volatile unsigned short*)(_reg))
#define _MAD_WRITE_BYTE(_reg, _val)    { (*((volatile unsigned char*)(_reg))) = (unsigned char)(_val); }
#define _MAD_WRITE_WORD(_reg, _val)    { (*((volatile unsigned short*)(_reg))) = (unsigned short)(_val); }
#define _MAD_R1BYTE(u32Addr, u8mask)    (_MAD_READ_BYTE (_MAD_PHYSICAL_MEM_BASE + ((u32Addr) << 1) - ((u32Addr) & 1)) & (u8mask))
#define _MAD_AU_AbsReadByte(u32Reg)    (_MAD_READ_BYTE (_MAD_PHYSICAL_MEM_BASE + ((u32Reg) << 1) - ((u32Reg) & 1)))
#define _MAD_AU_AbsRead2Byte(u32Reg)    (_MAD_READ_WORD (_MAD_PHYSICAL_MEM_BASE + ((u32Reg) << 1)) )
#define _MAD_AU_AbsWriteByte(u32Reg, u8Val) \
	do { \
		(_MAD_WRITE_BYTE(_MAD_PHYSICAL_MEM_BASE + ((u32Reg) << 1) - ((u32Reg) & 1), u8Val)); \
	} while(0)
#define _MAD_AU_AbsWriteMaskByte(u32Reg, u8Mask, u8Val) \
	do { \
		(_MAD_WRITE_BYTE(_MAD_PHYSICAL_MEM_BASE + ((u32Reg) << 1) - ((u32Reg) & 1), (_MAD_R1BYTE((u32Reg), 0xFF) & ~(u8Mask)) | ((u8Val) & (u8Mask)))); \
	} while(0)
#define _MAD_AU_AbsWrite2Byte(u32Reg, u16Val) \
	do { \
		(_MAD_WRITE_WORD(_MAD_PHYSICAL_MEM_BASE + ((u32Reg) << 1), u16Val)); \
	} while(0)


/*
 * ============================================================================
 * Forward Declarations
 * ============================================================================
 */
/* Read/Write Register */
#if 0 /* It's not in used for the moment, might be TODO */
static unsigned char _mhal_alsa_read_byte(unsigned int u32RegAddr);
#endif
static unsigned short _mhal_alsa_read_reg(unsigned int u32RegAddr);
#if 0 /* It's not in used for the moment, might be TODO */
static void _mhal_alsa_write_byte(unsigned int u32RegAddr, unsigned char u8Val);
#endif
static void _mhal_alsa_write_mask_byte(unsigned int u32RegAddr, unsigned char u8Mask, unsigned char u8Val);
static void _mhal_alsa_write_reg(unsigned int u32RegAddr, unsigned short u16Val);
#if 0
static void _mhal_alsa_write_mask_reg(unsigned int u32RegAddr, unsigned short u16Mask, unsigned short u16Val);
#endif
static unsigned int _mhal_alsa_get_device_status(void);

/* DMA Reader */
static int _mhal_alsa_dma_reader_init(void);
static int _mhal_alsa_dma_reader_exit(void);
static int _mhal_alsa_dma_reader_start(void);
static int _mhal_alsa_dma_reader_stop(void);
static int _mhal_alsa_dma_reader_resume(void);
static int _mhal_alsa_dma_reader_suspend(void);
static unsigned int _mhal_alsa_dma_reader_write(void* buffer, unsigned int bytes);
static int _mhal_alsa_dma_reader_get(int cmd, unsigned int *param);
static int _mhal_alsa_dma_reader_set(int cmd, unsigned int *param);
static int _mhal_alsa_dma_reader_set_sample_rate(unsigned int sample_rate);
static int _mhal_alsa_dma_reader_set_channel_mode(unsigned int channel_mode);
static int _mhal_alsa_dma_reader_get_inused_lines(void);
static int _mhal_alsa_dma_reader_get_avail_lines(void);

/* DMA Reader2 */
static int _mhal_alsa_dma_reader2_init(void);
static int _mhal_alsa_dma_reader2_exit(void);
static int _mhal_alsa_dma_reader2_start(void);
static int _mhal_alsa_dma_reader2_stop(void);
static int _mhal_alsa_dma_reader2_resume(void);
static int _mhal_alsa_dma_reader2_suspend(void);
static unsigned int _mhal_alsa_dma_reader2_write(void* buffer, unsigned int bytes);
static int _mhal_alsa_dma_reader2_get(int cmd, unsigned int *param);
static int _mhal_alsa_dma_reader2_set(int cmd, unsigned int *param);
static int _mhal_alsa_dma_reader2_set_sample_rate(unsigned int sample_rate);
static int _mhal_alsa_dma_reader2_set_channel_mode(unsigned int channel_mode);
static int _mhal_alsa_dma_reader2_get_inused_lines(void);
static int _mhal_alsa_dma_reader2_get_avail_lines(void);

/* PCM Playback3 */
static int _mhal_alsa_pcm_playback3_init(void);
static int _mhal_alsa_pcm_playback3_exit(void);
static int _mhal_alsa_pcm_playback3_start(void);
static int _mhal_alsa_pcm_playback3_stop(void);
static int _mhal_alsa_pcm_playback3_resume(void);
static int _mhal_alsa_pcm_playback3_suspend(void);
static unsigned int _mhal_alsa_pcm_playback3_write(void* buffer, unsigned int bytes);
static int _mhal_alsa_pcm_playback3_get(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_playback3_set(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_playback3_set_sample_rate(unsigned int sample_rate);
static int _mhal_alsa_pcm_playback3_set_channel_mode(unsigned int channel_mode);
static int _mhal_alsa_pcm_playback3_get_inused_lines(void);
static int _mhal_alsa_pcm_playback3_get_avail_lines(void);

/* PCM Playback4 */
static int _mhal_alsa_pcm_playback4_init(void);
static int _mhal_alsa_pcm_playback4_exit(void);
static int _mhal_alsa_pcm_playback4_start(void);
static int _mhal_alsa_pcm_playback4_stop(void);
static int _mhal_alsa_pcm_playback4_resume(void);
static int _mhal_alsa_pcm_playback4_suspend(void);
static unsigned int _mhal_alsa_pcm_playback4_write(void* buffer, unsigned int bytes);
static int _mhal_alsa_pcm_playback4_get(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_playback4_set(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_playback4_set_sample_rate(unsigned int sample_rate);
static int _mhal_alsa_pcm_playback4_set_channel_mode(unsigned int channel_mode);
static int _mhal_alsa_pcm_playback4_get_inused_lines(void);
static int _mhal_alsa_pcm_playback4_get_avail_lines(void);

/* PCM Playback7 */
static int _mhal_alsa_pcm_playback7_init(void);
static int _mhal_alsa_pcm_playback7_exit(void);
static int _mhal_alsa_pcm_playback7_start(void);
static int _mhal_alsa_pcm_playback7_stop(void);
static int _mhal_alsa_pcm_playback7_resume(void);
static int _mhal_alsa_pcm_playback7_suspend(void);
static unsigned int _mhal_alsa_pcm_playback7_write(void* buffer, unsigned int bytes);
static int _mhal_alsa_pcm_playback7_get(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_playback7_set(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_playback7_set_sample_rate(unsigned int sample_rate);
static int _mhal_alsa_pcm_playback7_set_channel_mode(unsigned int channel_mode);
static int _mhal_alsa_pcm_playback7_get_inused_lines(void);
static int _mhal_alsa_pcm_playback7_get_avail_lines(void);
static int _mhal_alsa_pcm_playback7_mmap(struct vm_area_struct *vma);

/* PCM Capture1 */
static int _mhal_alsa_pcm_capture1_init(void);
static int _mhal_alsa_pcm_capture1_exit(void);
static int _mhal_alsa_pcm_capture1_start(void);
static int _mhal_alsa_pcm_capture1_stop(void);
static int _mhal_alsa_pcm_capture1_resume(void);
static int _mhal_alsa_pcm_capture1_suspend(void);
static unsigned int _mhal_alsa_pcm_capture1_read(void *buffer, unsigned int bytes);
static int _mhal_alsa_pcm_capture1_get(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_capture1_set(int cmd, unsigned int *param);
static unsigned int _mhal_alsa_pcm_capture1_get_new_avail_bytes(void);
static unsigned int _mhal_alsa_pcm_capture1_get_total_avail_bytes(void);
static int _mhal_alsa_pcm_capture1_set_buffer_size(unsigned int buffer_size);

/* PCM Capture2 */
static int _mhal_alsa_pcm_capture2_init(void);
static int _mhal_alsa_pcm_capture2_exit(void);
static int _mhal_alsa_pcm_capture2_start(void);
static int _mhal_alsa_pcm_capture2_stop(void);
static int _mhal_alsa_pcm_capture2_resume(void);
static int _mhal_alsa_pcm_capture2_suspend(void);
static unsigned int _mhal_alsa_pcm_capture2_read(void *buffer, unsigned int bytes);
static int _mhal_alsa_pcm_capture2_get(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_capture2_set(int cmd, unsigned int *param);
static unsigned int _mhal_alsa_pcm_capture2_get_new_avail_bytes(void);
static unsigned int _mhal_alsa_pcm_capture2_get_total_avail_bytes(void);
static int _mhal_alsa_pcm_capture2_set_buffer_size(unsigned int buffer_size);

/* PCM Capture3 */
static int _mhal_alsa_pcm_capture3_init(void);
static int _mhal_alsa_pcm_capture3_exit(void);
static int _mhal_alsa_pcm_capture3_start(void);
static int _mhal_alsa_pcm_capture3_stop(void);
static int _mhal_alsa_pcm_capture3_resume(void);
static int _mhal_alsa_pcm_capture3_suspend(void);
static unsigned int _mhal_alsa_pcm_capture3_read(void *buffer, unsigned int bytes);
static int _mhal_alsa_pcm_capture3_get(int cmd, unsigned int *param);
static int _mhal_alsa_pcm_capture3_set(int cmd, unsigned int *param);
static unsigned int _mhal_alsa_pcm_capture3_get_new_avail_bytes(void);
static unsigned int _mhal_alsa_pcm_capture3_get_total_avail_bytes(void);
static int _mhal_alsa_pcm_capture3_set_buffer_size(unsigned int buffer_size);


/*
 * ============================================================================
 * Local Variables
 * ============================================================================
 */
/* MStar Audio DSP */
static struct MStar_MAD_Info MStar_MAD;

/* Supported Audio Rates by MStar Audio DSP */
static unsigned int mad_rates[] = {
	8000,
	11025,
	12000,
	16000,
	22050,
	24000,
	32000,
	44100,
	48000,
};

/* MStar Audio DSP - DMA Reader */
static struct MStar_DMA_Reader_Struct g_dma_reader = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_DMA_READER_TOTAL_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 0,
	.sample_rate = 0,
	.period_size = _MAD_DMA_READER_PERIOD_SIZE,
	.high_threshold = _MAD_DMA_READER_HIGH_THRESHOLD,
	.remain_size = 0,
	.written_size = 0,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_DMA_Reader_Ops = {
	.open = _mhal_alsa_dma_reader_init,
	.close = _mhal_alsa_dma_reader_exit,
	.start = _mhal_alsa_dma_reader_start,
	.stop = _mhal_alsa_dma_reader_stop,
	.resume = _mhal_alsa_dma_reader_resume,
	.suspend = _mhal_alsa_dma_reader_suspend,
	.read = NULL,
	.write = _mhal_alsa_dma_reader_write,
	.get = _mhal_alsa_dma_reader_get,
	.set = _mhal_alsa_dma_reader_set,
};

/* MStar Audio DSP - DMA Reader2 */
static struct MStar_DMA_Reader_Struct g_dma_reader2 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_DMA_READER2_TOTAL_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 0,
	.sample_rate = 0,
	.period_size = _MAD_DMA_READER2_PERIOD_SIZE,
	.high_threshold = _MAD_DMA_READER2_HIGH_THRESHOLD,
	.remain_size = 0,
	.written_size = 0,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_DMA_Reader2_Ops = {
	.open = _mhal_alsa_dma_reader2_init,
	.close = _mhal_alsa_dma_reader2_exit,
	.start = _mhal_alsa_dma_reader2_start,
	.stop = _mhal_alsa_dma_reader2_stop,
	.resume = _mhal_alsa_dma_reader2_resume,
	.suspend = _mhal_alsa_dma_reader2_suspend,
	.read = NULL,
	.write = _mhal_alsa_dma_reader2_write,
	.get = _mhal_alsa_dma_reader2_get,
	.set = _mhal_alsa_dma_reader2_set,
};

/* MStar Audio DSP - PCM Playback3 */
static struct MStar_DMA_Reader_Struct g_pcm_playback3 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_PCM_PLAYBACK3_TOTAL_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 0,
	.sample_rate = 0,
	.period_size = _MAD_PCM_PLAYBACK3_PERIOD_SIZE,
	.high_threshold = _MAD_PCM_PLAYBACK3_HIGH_THRESHOLD,
	.remain_size = 0,
	.written_size = 0,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_PCM_Playback3_Ops = {
	.open = _mhal_alsa_pcm_playback3_init,
	.close = _mhal_alsa_pcm_playback3_exit,
	.start = _mhal_alsa_pcm_playback3_start,
	.stop = _mhal_alsa_pcm_playback3_stop,
	.resume = _mhal_alsa_pcm_playback3_resume,
	.suspend = _mhal_alsa_pcm_playback3_suspend,
	.read = NULL,
	.write = _mhal_alsa_pcm_playback3_write,
	.get = _mhal_alsa_pcm_playback3_get,
	.set = _mhal_alsa_pcm_playback3_set,
};

/* MStar Audio DSP - PCM Playback4 */
static struct MStar_DMA_Reader_Struct g_pcm_playback4 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_PCM_PLAYBACK4_TOTAL_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 0,
	.sample_rate = 0,
	.period_size = _MAD_PCM_PLAYBACK4_PERIOD_SIZE,
	.high_threshold = _MAD_PCM_PLAYBACK4_HIGH_THRESHOLD,
	.remain_size = 0,
	.written_size = 0,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_PCM_Playback4_Ops = {
	.open = _mhal_alsa_pcm_playback4_init,
	.close = _mhal_alsa_pcm_playback4_exit,
	.start = _mhal_alsa_pcm_playback4_start,
	.stop = _mhal_alsa_pcm_playback4_stop,
	.resume = _mhal_alsa_pcm_playback4_resume,
	.suspend = _mhal_alsa_pcm_playback4_suspend,
	.read = NULL,
	.write = _mhal_alsa_pcm_playback4_write,
	.get = _mhal_alsa_pcm_playback4_get,
	.set = _mhal_alsa_pcm_playback4_set,
};

/* MStar Audio DSP - PCM Playback7 */
static struct MStar_DMA_Reader_Struct g_pcm_playback7 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_PCM_PLAYBACK7_TOTAL_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 0,
	.sample_rate = 0,
	.period_size = _MAD_PCM_PLAYBACK7_PERIOD_SIZE,
	.high_threshold = _MAD_PCM_PLAYBACK7_HIGH_THRESHOLD,
	.remain_size = 0,
	.written_size = 0,
	.status = E_STOP,
	.mmap_flag = 0,
};

static struct MStar_MAD_Ops MStar_PCM_Playback7_Ops = {
	.open = _mhal_alsa_pcm_playback7_init,
	.close = _mhal_alsa_pcm_playback7_exit,
	.start = _mhal_alsa_pcm_playback7_start,
	.stop = _mhal_alsa_pcm_playback7_stop,
	.resume = _mhal_alsa_pcm_playback7_resume,
	.suspend = _mhal_alsa_pcm_playback7_suspend,
	.read = NULL,
	.write = _mhal_alsa_pcm_playback7_write,
	.get = _mhal_alsa_pcm_playback7_get,
	.set = _mhal_alsa_pcm_playback7_set,
	.mmap = _mhal_alsa_pcm_playback7_mmap,
};

/* MStar Audio DSP - PCM Capture1 */
static struct MStar_PCM_Capture_Struct g_pcm_capture1 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_PCM_CAPTURE_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 2,
	.sample_rate = 48000,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_PCM_Capture1_Ops = {
	.open = _mhal_alsa_pcm_capture1_init,
	.close = _mhal_alsa_pcm_capture1_exit,
	.start = _mhal_alsa_pcm_capture1_start,
	.stop = _mhal_alsa_pcm_capture1_stop,
	.resume = _mhal_alsa_pcm_capture1_resume,
	.suspend = _mhal_alsa_pcm_capture1_suspend,
	.read = _mhal_alsa_pcm_capture1_read,
	.write = NULL,
	.get = _mhal_alsa_pcm_capture1_get,
	.set = _mhal_alsa_pcm_capture1_set,
};

/* MStar Audio DSP - PCM Capture2 */
static struct MStar_PCM_Capture_Struct g_pcm_capture2 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_PCM_CAPTURE_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 2,
	.sample_rate = 48000,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_PCM_Capture2_Ops = {
	.open = _mhal_alsa_pcm_capture2_init,
	.close = _mhal_alsa_pcm_capture2_exit,
	.start = _mhal_alsa_pcm_capture2_start,
	.stop = _mhal_alsa_pcm_capture2_stop,
	.resume = _mhal_alsa_pcm_capture2_resume,
	.suspend = _mhal_alsa_pcm_capture2_suspend,
	.read = _mhal_alsa_pcm_capture2_read,
	.write = NULL,
	.get = _mhal_alsa_pcm_capture2_get,
	.set = _mhal_alsa_pcm_capture2_set,
};

/* MStar Audio DSP - PCM Capture3 */
static struct MStar_PCM_Capture_Struct g_pcm_capture3 = {
	.buffer = {
		.addr = NULL,
		.size = _MAD_PCM_CAPTURE_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = MAD_FALSE,
	.channel_mode = 2,
	.sample_rate = 48000,
	.status = E_STOP,
};

static struct MStar_MAD_Ops MStar_PCM_Capture3_Ops = {
	.open = _mhal_alsa_pcm_capture3_init,
	.close = _mhal_alsa_pcm_capture3_exit,
	.start = _mhal_alsa_pcm_capture3_start,
	.stop = _mhal_alsa_pcm_capture3_stop,
	.resume = _mhal_alsa_pcm_capture3_resume,
	.suspend = _mhal_alsa_pcm_capture3_suspend,
	.read = _mhal_alsa_pcm_capture3_read,
	.write = NULL,
	.get = _mhal_alsa_pcm_capture3_get,
	.set = _mhal_alsa_pcm_capture3_set,
};

static ptrdiff_t g_pcm_dmaRdr1_base_va = 0;
static ptrdiff_t g_pcm_dmaRdr2_base_va = 0;
static ptrdiff_t g_pcm_dmaRdr3_base_va = 0;
static ptrdiff_t g_pcm_dmaRdr4_base_va = 0;
static ptrdiff_t g_pcm_dmaRdr7_base_va = 0;
static ptrdiff_t g_pcm_capture1_base_va = 0;
static ptrdiff_t g_pcm_capture2_base_va = 0;
static ptrdiff_t g_pcm_capture3_base_va = 0;

struct MStar_COMMON_PCM_SHM_Struct *g_shm_info = NULL;
struct MStar_COMMON_PCM_SHM_Struct PCM_SHM_INFO;

/*
 * ============================================================================
 * Function Implementations
 * ============================================================================
 */
#if 0 /* It's not in used for the moment, might be TODO */
static unsigned char _mhal_alsa_read_byte(unsigned int u32RegAddr)
{
	return (_MAD_AU_AbsReadByte(u32RegAddr+_MAD_MAILBOX_OFFSET));
}
#endif

static unsigned short _mhal_alsa_read_reg(unsigned int u32RegAddr)
{
	return (_MAD_AU_AbsRead2Byte(u32RegAddr+_MAD_MAILBOX_OFFSET));
}

#if 0 /* It's not in used for the moment, might be TODO */
static void _mhal_alsa_write_byte(unsigned int u32RegAddr, unsigned char u8Val)
{
	_MAD_AU_AbsWriteByte((u32RegAddr+_MAD_MAILBOX_OFFSET), u8Val);
}
#endif

static void _mhal_alsa_write_mask_byte(unsigned int u32RegAddr, unsigned char u8Mask, unsigned char u8Val)
{
	_MAD_AU_AbsWriteMaskByte((u32RegAddr+_MAD_MAILBOX_OFFSET), u8Mask, u8Val);
}

static void _mhal_alsa_write_reg(unsigned int u32RegAddr, unsigned short u16Val)
{
	_MAD_AU_AbsWrite2Byte((u32RegAddr+_MAD_MAILBOX_OFFSET), u16Val);
}

static void _mhal_alsa_write_mask_reg(unsigned int u32RegAddr, unsigned short u16Mask, unsigned short u16Val)
{
	unsigned short u16RegVal;

	u16RegVal = _mhal_alsa_read_reg(u32RegAddr);
	u16RegVal = ((u16RegVal & (~(u16Mask))) | (u16Val & u16Mask));
	_mhal_alsa_write_reg(u32RegAddr, u16RegVal);
}

#if 0 /* It's not in used for the moment, might be TODO */
static unsigned char _mhal_alsa_read2_byte(unsigned int u32RegAddr)
{
	return (_MAD_AU_AbsReadByte(u32RegAddr+_MAD_MAILBOX_OFFSET));
}
#endif

static unsigned short _mhal_alsa_read2_reg(unsigned int u32RegAddr)
{
	return (_MAD_AU_AbsRead2Byte(u32RegAddr+_MAD_MAILBOX_OFFSET2));
}

#if 0 /* It's not in used for the moment, might be TODO */
static void _mhal_alsa_write2_byte(unsigned int u32RegAddr, unsigned char u8Val)
{
	_MAD_AU_AbsWriteByte((u32RegAddr+_MAD_MAILBOX_OFFSET), u8Val);
}

static void _mhal_alsa_write2_mask_byte(unsigned int u32RegAddr, unsigned char u8Mask, unsigned char u8Val)
{
	_MAD_AU_AbsWriteMaskByte((u32RegAddr+_MAD_MAILBOX_OFFSET2), u8Mask, u8Val);
}
#endif

static void _mhal_alsa_write2_reg(unsigned int u32RegAddr, unsigned short u16Val)
{
	_MAD_AU_AbsWrite2Byte((u32RegAddr+_MAD_MAILBOX_OFFSET2), u16Val);
}

static void _mhal_alsa_write2_mask_reg(unsigned int u32RegAddr, unsigned short u16Mask, unsigned short u16Val)
{
	unsigned short u16RegVal;

	u16RegVal = _mhal_alsa_read2_reg(u32RegAddr);
	u16RegVal = ((u16RegVal & (~(u16Mask))) | (u16Val & u16Mask));
	_mhal_alsa_write2_reg(u32RegAddr, u16RegVal);
}


static unsigned int _mhal_alsa_get_device_status(void)
{
	if (((_mhal_alsa_read_reg(0x2D30) & 0x0200) == 0x0200) || (_mhal_alsa_read_reg(0x2A90) == 0x0000)) {
		return MAD_FALSE;
	}
	else {
		return MAD_TRUE;
	}
}

static unsigned int _mhal_alsa_get_common_buffer_pa(void)
{
	unsigned int dsp2_pa = 0;
	unsigned int dsp2_size = 0;

	dsp2_pa = (_mhal_alsa_read_reg(0x2A90) + ((_mhal_alsa_read_reg(0x2AC0) & 0x0F) << 16)) * _MAD_ADDR_CONVERTOR;

	if (g_shm_info == NULL) {
		dsp2_size = _mhal_alsa_read_reg(0x2D82) + (_mhal_alsa_read_reg(0x2D84) << 16);
		if (dsp2_size == 0) {
			MAD_PRINT(KERN_ERR "Error! Can't get DSP2 size from mailbox!\n");
			dsp2_size = _MAD_DSP2_DRAM_SIZE;
		}
	}

	return (dsp2_pa + dsp2_size);
}

static ptrdiff_t _mhal_alsa_convert_pa_to_ba(ptrdiff_t ptrPhysicalAddr)
{
	ptrdiff_t ptrPa = ptrPhysicalAddr;
	ptrdiff_t ptrBa = 0;
	ptrdiff_t ptrPaToBaOffset = 0;

	if (ptrPhysicalAddr >= ARM_MIU2_BASE_ADDR)
	{
		/* MIU2 */
		ptrPa -= ARM_MIU2_BASE_ADDR;
		ptrPaToBaOffset = ARM_MIU2_BUS_BASE;
	}
	else if (ptrPhysicalAddr >= ARM_MIU1_BASE_ADDR)
	{
		/* MIU1 */
		ptrPa -= ARM_MIU1_BASE_ADDR;
		ptrPaToBaOffset = ARM_MIU1_BUS_BASE;
	}
	else
	{
		/* MIU0 */
		ptrPaToBaOffset = ARM_MIU0_BUS_BASE;
	}

	ptrBa = ptrPa + ptrPaToBaOffset;
	//MAD_PRINT(KERN_ERR "ptrPhysicalAddr 0x%llX, ptrPa 0x%llX, ptrBa 0x%llX, ptrPaToBaOffset 0x%llX\n", ptrPhysicalAddr, ptrPa, ptrBa, ptrPaToBaOffset);

	return ptrBa;
}

static int _mhal_alsa_get_share_mem(void)
{
	ptrdiff_t audio_share_mem_base_pa = 0;
	ptrdiff_t audio_share_mem_base_ba = 0;
	ptrdiff_t audio_share_mem_base_va = 0;
	unsigned int dsp2_pa = 0;

	if (g_shm_info)
		return 0;

	dsp2_pa = (_mhal_alsa_read_reg(0x2A90) + ((_mhal_alsa_read_reg(0x2AC0) & 0x0F) << 16)) * _MAD_ADDR_CONVERTOR;

	audio_share_mem_base_pa = (ptrdiff_t)dsp2_pa;
	audio_share_mem_base_ba = _mhal_alsa_convert_pa_to_ba(audio_share_mem_base_pa);
	audio_share_mem_base_va = (ptrdiff_t)ioremap_wc(audio_share_mem_base_ba, COMMON_PCM_SHM_DRAM_SIZE);

	g_shm_info = (struct MStar_COMMON_PCM_SHM_Struct *)audio_share_mem_base_va;
	if (g_shm_info == NULL) {
		return -EINVAL;
	}

	/* check share memory version */
	if (g_shm_info->version != COMMON_PCM_SHM_VERSION) {
		MAD_PRINT(KERN_ERR "WARNING! COMMON PCM SHARE MEMORY Version not match %x !!\n", g_shm_info->version);
		g_shm_info = NULL;
		return -EINVAL;
	}
	/* check share memory init or not */
	if (g_shm_info->init_flag != MAD_TRUE) {
		MAD_PRINT(KERN_ERR "WARNING! COMMON PCM SHARE MEMORY not init !!\n");
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check hw_dma_reader1_offset value */
	if (g_shm_info->hw_dma_reader1_offset == 0 || (g_shm_info->hw_dma_reader1_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! hw_dma_reader1_offset is not right %x !!\n", g_shm_info->hw_dma_reader1_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check hw_dma_reader2_offset value */
	if (g_shm_info->hw_dma_reader2_offset == 0 || (g_shm_info->hw_dma_reader2_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! hw_dma_reader2_offset is not right %x !!\n", g_shm_info->hw_dma_reader2_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check sw_dma_reader1_offset value */
	if (g_shm_info->sw_dma_reader1_offset == 0 || (g_shm_info->sw_dma_reader1_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! sw_dma_reader1_offset is not right %x !!\n", g_shm_info->sw_dma_reader1_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check sw_dma_reader2_offset value */
	if (g_shm_info->sw_dma_reader2_offset == 0 || (g_shm_info->sw_dma_reader2_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! sw_dma_reader2_offset is not right %x !!\n", g_shm_info->sw_dma_reader2_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check capture1_offset value */
	if (g_shm_info->capture1_offset == 0 || (g_shm_info->capture1_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! capture1_offset is not right %x !!\n", g_shm_info->capture1_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check capture2_offset value */
	if (g_shm_info->capture2_offset == 0 || (g_shm_info->capture2_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! capture2_offset is not right %x !!\n", g_shm_info->capture2_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	/* check capture3_offset value */
	if (g_shm_info->capture3_offset == 0 || (g_shm_info->capture3_offset & 0xFF) != 0) {
		MAD_PRINT(KERN_ERR "WARNING! capture3_offset is not right %x !!\n", g_shm_info->capture3_offset);
		g_shm_info = NULL;
		return -EINVAL;
	}

	PCM_SHM_INFO.version = g_shm_info->version;
	PCM_SHM_INFO.init_flag = g_shm_info->init_flag;
	PCM_SHM_INFO.hw_dma_reader1_offset = g_shm_info->hw_dma_reader1_offset;
	PCM_SHM_INFO.hw_dma_reader1_size = g_shm_info->hw_dma_reader1_size;
	PCM_SHM_INFO.hw_dma_reader2_offset = g_shm_info->hw_dma_reader2_offset;
	PCM_SHM_INFO.hw_dma_reader2_size = g_shm_info->hw_dma_reader2_size;
	PCM_SHM_INFO.sw_dma_reader1_offset = g_shm_info->sw_dma_reader1_offset;
	PCM_SHM_INFO.sw_dma_reader1_size = g_shm_info->sw_dma_reader1_size;
	PCM_SHM_INFO.sw_dma_reader2_offset = g_shm_info->sw_dma_reader2_offset;
	PCM_SHM_INFO.sw_dma_reader2_size = g_shm_info->sw_dma_reader2_size;
	PCM_SHM_INFO.capture1_offset = g_shm_info->capture1_offset;
	PCM_SHM_INFO.capture1_size = g_shm_info->capture1_size;
	PCM_SHM_INFO.capture2_offset = g_shm_info->capture2_offset;
	PCM_SHM_INFO.capture2_size = g_shm_info->capture2_size;
	PCM_SHM_INFO.capture3_offset = g_shm_info->capture3_offset;
	PCM_SHM_INFO.capture3_size = g_shm_info->capture3_size;
	PCM_SHM_INFO.dmic_capture_offset = g_shm_info->dmic_capture_offset;
	PCM_SHM_INFO.dmic_capture_size = g_shm_info->dmic_capture_size;
	PCM_SHM_INFO.aec_capture_offset = g_shm_info->aec_capture_offset;
	PCM_SHM_INFO.aec_capture_size = g_shm_info->aec_capture_size;

	return 0;
}

static char _mhal_alsa_get_ch_num(char dma)
{
	char ch_num = 0;

	switch (dma) {
		case E_HW_DMA_Reader1:
			ch_num = g_shm_info->dma_channel_mapping & 0xF;
			break;
		case E_HW_DMA_Reader2:
			ch_num = (g_shm_info->dma_channel_mapping >> 4) & 0xF;
			break;
		case E_SW_DMA_Reader1:
			ch_num = g_shm_info->sw_dma_channel_mapping & 0xF;
			break;
		case E_SW_DMA_Reader2:
			ch_num = (g_shm_info->sw_dma_channel_mapping >> 4) & 0xF;
			break;
		case E_SW_DMA_Reader3:
			ch_num = (g_shm_info->sw_dma_channel_mapping >> 8) & 0xF;
			break;
		default:
			break;
	}

	return ch_num;
}

static unsigned int _mhal_alsa_get_input_mux_reg(char dma)
{
	unsigned int input_mux_reg = 0;
	char ch_num;

	ch_num = _mhal_alsa_get_ch_num(dma);

	switch(ch_num) {
		case 6:
			input_mux_reg = 0x2C67;
			break;
		case 7:
			input_mux_reg = 0x2C69;
			break;
		case 8:
			input_mux_reg = 0x2C6B;
			break;
		default:
			break;
	}

	return input_mux_reg;
}

/* Initiate DMA Reader */
static int _mhal_alsa_dma_reader_init(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	unsigned int audio_pcm_dmaRdr_bufSz = 0;  /* must be multiple of DMA_RDR_PCM_BUF_UNIT*2 (= 0x2000) */
	unsigned int audio_pcm_dmaRdr_offset = _MAD_DMA_READER_BASE_OFFSET;
	ptrdiff_t audio_pcm_dmaRdr_base_pa = 0; /* DMA Reader v2-1 Input buffer (DM_Prefetch) */
	ptrdiff_t audio_pcm_dmaRdr_base_ba = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_va = 0;
	unsigned char tmp_value = 0;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Playback1 engine\n");

	if (PCM_SHM_INFO.hw_dma_reader1_offset != 0) {
		audio_pcm_dmaRdr_offset = PCM_SHM_INFO.hw_dma_reader1_offset;
	}

	if ((dma_reader->initialized_status != MAD_TRUE) || (dma_reader->status != E_RESUME)) {
		audio_pcm_dmaRdr_bufSz = dma_reader->buffer.size;
		audio_pcm_dmaRdr_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_dmaRdr_offset;
		audio_pcm_dmaRdr_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_dmaRdr_base_pa);

		if ((audio_pcm_dmaRdr_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM reader bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		if (g_pcm_dmaRdr1_base_va == 0)	{
			g_pcm_dmaRdr1_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_dmaRdr_base_ba, audio_pcm_dmaRdr_bufSz);
			if (g_pcm_dmaRdr1_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Playback1 Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_dmaRdr_base_va = g_pcm_dmaRdr1_base_va;

		dma_reader->str_mode_info.physical_addr = audio_pcm_dmaRdr_base_pa;
		dma_reader->str_mode_info.bus_addr = audio_pcm_dmaRdr_base_ba;
		dma_reader->str_mode_info.virtual_addr = audio_pcm_dmaRdr_base_va;

		dma_reader->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_dmaRdr_bufSz = dma_reader->buffer.size;
		audio_pcm_dmaRdr_base_pa = dma_reader->str_mode_info.physical_addr;
		audio_pcm_dmaRdr_base_ba = dma_reader->str_mode_info.bus_addr;
		audio_pcm_dmaRdr_base_va = dma_reader->str_mode_info.virtual_addr;
	}

	/* init DMA writer address */
	dma_reader->buffer.addr = (unsigned char *)audio_pcm_dmaRdr_base_va;
	dma_reader->buffer.w_ptr = dma_reader->buffer.addr;
	//MAD_PRINT(KERN_INFO "PCM Playback1 buffer start address = 0x%08X\n", dma_reader->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Playback1 buffer end address = 0x%08X\n", (dma_reader->buffer.addr + dma_reader->buffer.size));

	/* initial DMA Reader v2-1 */
	_mhal_alsa_write2_reg(0x3E40, 0x0);
	_mhal_alsa_write2_reg(0x3E42, ((audio_pcm_dmaRdr_base_pa /_MAD_BYTES_IN_LINE) & 0xFFFF));
	_mhal_alsa_write2_reg(0x3E44, (((audio_pcm_dmaRdr_base_pa /_MAD_BYTES_IN_LINE) >> 16) & 0xFFFF));
	_mhal_alsa_write2_reg(0x3E46, (audio_pcm_dmaRdr_bufSz /_MAD_BYTES_IN_LINE)); /* setting : DMA Reader v2-1 Size */
	_mhal_alsa_write2_mask_reg(0x3E4A, 0xFFFF, 0x00012); /* setting : DMA Reader v2-1 Underrun Thr */

	/* reset and start DMA Reader v2-1 */
	_mhal_alsa_write2_mask_reg(0x3E40, 0xFFFF, 0x800B);
	_mhal_alsa_write2_mask_reg(0x3E40, 0xFFFF, 0x000B);

	/* reset remain size */
	dma_reader->remain_size = 0;

	/* reset written size */
	dma_reader->written_size = 0;

	/* clear DMA reader buffer */
	memset((void *)dma_reader->buffer.addr, 0x00, dma_reader->buffer.size);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *(dma_reader->buffer.addr);

	Chip_Flush_Memory();

	return 0;
}

/* Exit DMA Reader */
static int _mhal_alsa_dma_reader_exit(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Playback1 engine\n");

	if (g_pcm_dmaRdr1_base_va != 0) {
		if (dma_reader->buffer.addr) {
			iounmap((void *)dma_reader->buffer.addr);
			dma_reader->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Playback1 buffer address should not be 0 !\n");
		}

		g_pcm_dmaRdr1_base_va = 0;
	}

	dma_reader->status = E_STOP;
	dma_reader->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_HW_DMA_Reader1);
	}

	return 0;
}

/* Start DMA Reader */
static int _mhal_alsa_dma_reader_start(void)
{
	unsigned int input_mux_reg = 0x2C6B;
	unsigned int ret = 0;
	//MAD_PRINT(KERN_INFO "Start MStar PCM Playback1 engine\n");

	if (g_shm_info) {
		ret = _mhal_alsa_get_input_mux_reg(E_HW_DMA_Reader1);
		if (ret) {
			input_mux_reg = ret;
		}
	}

	_mhal_alsa_write_mask_byte(input_mux_reg, 0x9F, 0x90); /* CH8 sel to HW DMA Reader v2-1 */

	return 0;
}

/* Stop DMA Reader */
static int _mhal_alsa_dma_reader_stop(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Playback1 engine\n");

	/* clear wr cntrs */
	_mhal_alsa_write2_mask_reg(0x3E48, 0xFFFF, 0x0000);

	/* reset and start DMA Reader */
	_mhal_alsa_write2_mask_reg(0x3E40, 0xFFFF, 0x800B);
	_mhal_alsa_write2_mask_reg(0x3E40, 0xFFFF, 0x000B);

	/* reset Write Pointer */
	dma_reader->buffer.w_ptr = dma_reader->buffer.addr;

	/* reset remain size */
	dma_reader->remain_size = 0;

	/* reset written size */
	dma_reader->written_size = 0;

	dma_reader->status = E_STOP;

	return 0;
}

/* Resume DMA Reader */
static int _mhal_alsa_dma_reader_resume(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Playback1 engine\n");

	dma_reader->status = E_RESUME;

	return 0;
}

/* Suspend DMA Reader */
static int _mhal_alsa_dma_reader_suspend(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Playback1 engine\n");

	dma_reader->status = E_SUSPEND;

	return 0;
}

/* Write PCM to DMA Reader */
static unsigned int _mhal_alsa_dma_reader_write(void *buffer, unsigned int bytes)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	unsigned char *bufptr = (unsigned char *)buffer;
    unsigned char *w_ptr = NULL;
	unsigned char tmp_bufptr1 = 0;
	unsigned char tmp_bufptr2 = 0;
	unsigned char tmp_value = 0;
	int loop = 0;
	int inused_lines = 0;
	unsigned int copy_lr_sample = 0;
	unsigned int copy_size = 0;

	inused_lines = _mhal_alsa_dma_reader_get_inused_lines();
	if (inused_lines == 0) {
		//MAD_PRINT(KERN_INFO "***** PCM Playback1 Buffer empty !! ***** \n");
	}

	copy_lr_sample = bytes / 2; /* L + R samples */
	copy_size = (dma_reader->channel_mode == E_MONO) ? (bytes * 2) : bytes;

	/* copy data to DMA Reader v2-1 buffer */
	if ( ((inused_lines * _MAD_BYTES_IN_LINE) + copy_size) < dma_reader->high_threshold) {
		if (dma_reader->channel_mode == E_MONO) {
			for (loop = 0; loop < copy_lr_sample; loop++) {
				tmp_bufptr1 = *bufptr++;
				tmp_bufptr2 = *bufptr++;

				*(dma_reader->buffer.w_ptr++) = tmp_bufptr1;
				*(dma_reader->buffer.w_ptr++) = tmp_bufptr2;
				*(dma_reader->buffer.w_ptr++) = tmp_bufptr1;
				*(dma_reader->buffer.w_ptr++) = tmp_bufptr2;

				if (dma_reader->buffer.w_ptr >= (dma_reader->buffer.addr + dma_reader->buffer.size))
					dma_reader->buffer.w_ptr = dma_reader->buffer.addr;
			}
		}
		else {
			for (loop = 0; loop < copy_lr_sample; loop++) {
				*(dma_reader->buffer.w_ptr++) = *bufptr++;
				*(dma_reader->buffer.w_ptr++) = *bufptr++;

				if (dma_reader->buffer.w_ptr >= (dma_reader->buffer.addr + dma_reader->buffer.size))
					dma_reader->buffer.w_ptr = dma_reader->buffer.addr;
			}
		}

		/* a patch to prevent abnormal data in this HW module, so force to clear next 96 bytes of PCM buffer */
		w_ptr = dma_reader->buffer.w_ptr;
		for (loop = 0; loop < 96; loop++) {
			*(w_ptr++) = 0x00;

			if (w_ptr >= (dma_reader->buffer.addr + dma_reader->buffer.size))
				w_ptr = dma_reader->buffer.addr;
		}

		/*
		 * it's a patch here!
		 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
		 * in order to ensure MIU data can be updated since there is some queue in CPU side,
		 * we just read any byte of them then all MIU data can be updated.
		 */
		tmp_value = *(dma_reader->buffer.w_ptr);

		/* flush MIU */
		Chip_Flush_Memory();

		/* update copied size to DMA Reader v2-1 */
		copy_size += dma_reader->remain_size;
		_mhal_alsa_write2_mask_reg(0x3E40, 0x0010, 0x0000);
		_mhal_alsa_write2_mask_reg(0x3E48, 0xFFFF, (copy_size / _MAD_BYTES_IN_LINE));
		_mhal_alsa_write2_mask_reg(0x3E40, 0x0010, 0x0010);
		dma_reader->remain_size = copy_size % _MAD_BYTES_IN_LINE;
		dma_reader->written_size += (copy_size - dma_reader->remain_size);
		dma_reader->status = E_START;

		return bytes;
	}

	//MAD_PRINT(KERN_INFO "***** PCM Playback1 Buffer busy !! ***** \n");

	return 0;
}

/* Get information from DMA Reader */
static int _mhal_alsa_dma_reader_get(int cmd, unsigned int *param)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Playback1\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_GET_BUFFER_SIZE:
		{
			*param = dma_reader->buffer.size;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_PERIOD_SIZE:
		{
			*param = dma_reader->period_size;
			break;
		}

		case E_PCM_PLAYBACK_GET_SAMPLE_RATE:
		{
			*param = dma_reader->sample_rate;
			break;
		}

		case E_PCM_PLAYBACK_GET_CHANNEL_MODE:
		{
			*param = dma_reader->channel_mode;
			break;
		}

		case E_PCM_PLAYBACK_GET_MAX_CHANNEL:
		{
			*param = _MAD_MAX_CHANNEL;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_COUNT:
		{
			*param = sizeof(mad_rates) / sizeof(mad_rates[0]);
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_LIST:
		{
			*param = (ptrdiff_t)&mad_rates;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_MASK:
		{
			*param = 0;
			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_INUSED_BYTES:
		{
			*param = _mhal_alsa_dma_reader_get_inused_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_AVAIL_BYTES:
		{
			*param = _mhal_alsa_dma_reader_get_avail_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_CONSUMED_BYTES:
		{
			int inused_bytes = 0;
			int consumed_bytes = 0;

			inused_bytes = _mhal_alsa_dma_reader_get_inused_lines() * _MAD_BYTES_IN_LINE;
			consumed_bytes = dma_reader->written_size - inused_bytes;
			dma_reader->written_size = inused_bytes;
			*param = consumed_bytes;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_STR_STATUS:
		{
			*param = dma_reader->status;
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_HW_DMA_Reader1))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to DMA Reader */
static int _mhal_alsa_dma_reader_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Playback1\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_SET_SAMPLE_RATE:
		{
			_mhal_alsa_dma_reader_set_sample_rate(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_CHANNEL_MODE:
		{
			_mhal_alsa_dma_reader_set_channel_mode(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_HW_DMA_Reader1);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get DMA Reader PCM buffer level */
static int _mhal_alsa_dma_reader_get_inused_lines(void)
{
	int inused_lines = 0;

	/* Mask LEVEL_CNT_MASK before read */
	_mhal_alsa_write2_mask_reg(0x3E40, 0x0020, 0x0020);
	inused_lines = _mhal_alsa_read2_reg(0x3E54);
	_mhal_alsa_write2_mask_reg(0x3E40, 0x0020, 0x0000);

	return inused_lines;
}

/* Get DMA Reader PCM avail bytes */
static int _mhal_alsa_dma_reader_get_avail_lines(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	int inused_lines = 0;
	int avail_lines = 0;

	inused_lines = _mhal_alsa_dma_reader_get_inused_lines();
	avail_lines = (dma_reader->high_threshold / _MAD_BYTES_IN_LINE) - inused_lines;
	if (avail_lines < 0) {
		MAD_PRINT(KERN_ERR "Error! Incorrect inused lines %d!\n", inused_lines);
		avail_lines = 0;
	}

	return avail_lines;
}

/* Set smaple rate to DMA Reader */
static int _mhal_alsa_dma_reader_set_sample_rate(unsigned int sample_rate)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	unsigned short synthrate, divisor;
	MAD_PRINT(KERN_INFO "Target sample rate is %u\n", sample_rate);

	dma_reader->sample_rate = sample_rate;

	/* New DMA Reader v2-1 setting
	 * Formula is :
	 * Synthesizer Rate = 216MHz / Divisor(1, 2, 4 or 8) * 1024 / 256 / Sampling Rate(32Khz, 44.1KHz or 48KHz)
	 */
	switch(sample_rate) {
		case 8000:
		{
			divisor = 2;
			synthrate = 0x6978;
			break;
		}

		case 11025:
		{
			divisor = 2;
			synthrate = 0x4C87;
			break;
		}

		case 12000:
		{
			divisor = 2;
			synthrate = 0x4650;
			break;
		}

		case 16000:
		{
			divisor = 1;
			synthrate = 0x6978;
			break;
		}

		case 22050:
		{
			divisor = 1;
			synthrate = 0x4C87;
			break;
		}

		case 24000:
		{
			divisor = 1;
			synthrate = 0x4650;
			break;
		}

		case 32000:
		{
			divisor = 0;
			synthrate = 0x6978;
			break;
		}

		case 44100:
		{
			divisor = 0;
			synthrate = 0x4C87;
			break;
		}

		case 48000:
		{
			divisor = 0;
			synthrate = 0x4650;
			break;
		}

		default:
		{
			MAD_PRINT(KERN_ERR "Error! un-supported sample rate %u !!!\n", sample_rate);
			divisor = 0;
			synthrate = 0x4650;
			dma_reader->sample_rate = 48000;
			break;
		}
	}

	/* synthersizer setting update */
	_mhal_alsa_write2_mask_reg(0x3E4C, 0x0030, (divisor << 4)); /* set divisor */
	_mhal_alsa_write2_reg(0x3E4E, synthrate); /* DMA synthesizer N.F. */
	_mhal_alsa_write2_mask_reg(0x3E4C, 0x0145, 0x0145); /* enable DMA synthesizer */

	return 0;
}

/* Set channel mode to DMA Reader */
static int _mhal_alsa_dma_reader_set_channel_mode(unsigned int channel_mode)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader;
	unsigned int buffer_size = 0;
	MAD_PRINT(KERN_INFO "Target channel mode is %u\n", channel_mode);

	dma_reader->channel_mode = channel_mode;
	buffer_size = ((dma_reader->sample_rate << dma_reader->channel_mode) * _MAD_DMA_READER_QUEUE_SIZE) / 1000;
	if ((buffer_size % _MAD_BYTES_IN_LINE))
		buffer_size += (_MAD_BYTES_IN_LINE - (buffer_size % _MAD_BYTES_IN_LINE));

	dma_reader->buffer.size = (dma_reader->channel_mode == E_MONO) ? (buffer_size * 2) : buffer_size;
	dma_reader->period_size = dma_reader->buffer.size >> 2;
	dma_reader->high_threshold = dma_reader->buffer.size - (dma_reader->buffer.size >> 3);

	return 0;
}

/* Initiate DMA Reader2 */
static int _mhal_alsa_dma_reader2_init(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	unsigned int audio_pcm_dmaRdr_bufSz = 0;  /* must be multiple of DMA_RDR_PCM_BUF_UNIT*2 (= 0x2000) */
	unsigned int audio_pcm_dmaRdr_offset = _MAD_DMA_READER2_BASE_OFFSET;
	ptrdiff_t audio_pcm_dmaRdr_base_pa = 0; /* DMA Reader v2-2 Input buffer (DM_Prefetch) */
	ptrdiff_t audio_pcm_dmaRdr_base_ba = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_va = 0;
	unsigned char tmp_value = 0;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Playback2 engine\n");

	if (PCM_SHM_INFO.hw_dma_reader2_offset != 0) {
		audio_pcm_dmaRdr_offset = PCM_SHM_INFO.hw_dma_reader2_offset;
	}

	if ((dma_reader->initialized_status != MAD_TRUE) || (dma_reader->status != E_RESUME)) {
		audio_pcm_dmaRdr_bufSz = dma_reader->buffer.size;
		audio_pcm_dmaRdr_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_dmaRdr_offset;
		audio_pcm_dmaRdr_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_dmaRdr_base_pa);

		if ((audio_pcm_dmaRdr_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM reader bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		if (g_pcm_dmaRdr2_base_va == 0)	{
			g_pcm_dmaRdr2_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_dmaRdr_base_ba, audio_pcm_dmaRdr_bufSz);
			if (g_pcm_dmaRdr2_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Playback2 Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_dmaRdr_base_va = g_pcm_dmaRdr2_base_va;

		dma_reader->str_mode_info.physical_addr = audio_pcm_dmaRdr_base_pa;
		dma_reader->str_mode_info.bus_addr = audio_pcm_dmaRdr_base_ba;
		dma_reader->str_mode_info.virtual_addr = audio_pcm_dmaRdr_base_va;

		dma_reader->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_dmaRdr_bufSz = dma_reader->buffer.size;
		audio_pcm_dmaRdr_base_pa = dma_reader->str_mode_info.physical_addr;
		audio_pcm_dmaRdr_base_ba = dma_reader->str_mode_info.bus_addr;
		audio_pcm_dmaRdr_base_va = dma_reader->str_mode_info.virtual_addr;
	}

	/* init DMA writer address */
	dma_reader->buffer.addr = (unsigned char *)audio_pcm_dmaRdr_base_va;
	dma_reader->buffer.w_ptr = dma_reader->buffer.addr;
	//MAD_PRINT(KERN_INFO "PCM Playback2 buffer start address = 0x%08X\n", dma_reader->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Playback2 buffer end address = 0x%08X\n", (dma_reader->buffer.addr + dma_reader->buffer.size));

	/* initial DMA Reader v2-2 */
	_mhal_alsa_write2_reg(0x3E60, 0x0);
	_mhal_alsa_write2_reg(0x3E62, ((audio_pcm_dmaRdr_base_pa /_MAD_BYTES_IN_LINE) & 0xFFFF));
	_mhal_alsa_write2_reg(0x3E64, (((audio_pcm_dmaRdr_base_pa /_MAD_BYTES_IN_LINE) >> 16) & 0xFFFF));
	_mhal_alsa_write2_reg(0x3E66, (audio_pcm_dmaRdr_bufSz /_MAD_BYTES_IN_LINE)); /* setting : DMA Reader v2-2 Size */
	_mhal_alsa_write2_mask_reg(0x3E6A, 0xFFFF, 0x00012); /* setting : DMA Reader v2-2 Underrun Thr */

	/* reset and start DMA Reader v2-2 */
	_mhal_alsa_write2_mask_reg(0x3E60, 0xFFFF, 0x800B);
	_mhal_alsa_write2_mask_reg(0x3E60, 0xFFFF, 0x000B);

	/* reset remain size */
	dma_reader->remain_size = 0;

	/* reset written size */
	dma_reader->written_size = 0;

	/* clear DMA reader buffer */
	memset((void *)dma_reader->buffer.addr, 0x00, dma_reader->buffer.size);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *(dma_reader->buffer.addr);

	Chip_Flush_Memory();

	return 0;
}

/* Exit DMA Reader2 */
static int _mhal_alsa_dma_reader2_exit(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Playback2 engine\n");

	if (g_pcm_dmaRdr2_base_va != 0) {
		if (dma_reader->buffer.addr) {
			iounmap((void *)dma_reader->buffer.addr);
			dma_reader->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Playback2 buffer address should not be 0 !\n");
		}

		g_pcm_dmaRdr2_base_va = 0;
	}

	dma_reader->status = E_STOP;
	dma_reader->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_HW_DMA_Reader2);
	}

	return 0;
}

/* Start DMA Reader2 */
static int _mhal_alsa_dma_reader2_start(void)
{
	unsigned int input_mux_reg = 0x2C67;
	unsigned int ret = 0;
	//MAD_PRINT(KERN_INFO "Start MStar PCM Playback2 engine\n");

	if (g_shm_info) {
		ret = _mhal_alsa_get_input_mux_reg(E_HW_DMA_Reader2);
		if (ret) {
			input_mux_reg = ret;
		}
	}

	_mhal_alsa_write_mask_byte(input_mux_reg, 0x9F, 0x91); /* CH6 sel to HW DMA Reader v2-2 */

	return 0;
}

/* Stop DMA Reader2 */
static int _mhal_alsa_dma_reader2_stop(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Playback2 engine\n");

	/* clear wr cntrs */
	_mhal_alsa_write2_mask_reg(0x3E68, 0xFFFF, 0x0000);

	/* reset and start DMA Reader v2-2 */
	_mhal_alsa_write2_mask_reg(0x3E60, 0xFFFF, 0x800B);
	_mhal_alsa_write2_mask_reg(0x3E60, 0xFFFF, 0x000B);

	/* reset Write Pointer */
	dma_reader->buffer.w_ptr = dma_reader->buffer.addr;

	/* reset remain size */
	dma_reader->remain_size = 0;

	/* reset written size */
	dma_reader->written_size = 0;

	dma_reader->status = E_STOP;

	return 0;
}

/* Resume DMA Reader2 */
static int _mhal_alsa_dma_reader2_resume(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Playback2 engine\n");

	dma_reader->status = E_RESUME;

	return 0;
}

/* Suspend DMA Reader2 */
static int _mhal_alsa_dma_reader2_suspend(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Playback2 engine\n");

	dma_reader->status = E_SUSPEND;

	return 0;
}

/* Write PCM to DMA Reader2 */
static unsigned int _mhal_alsa_dma_reader2_write(void *buffer, unsigned int bytes)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	unsigned char *bufptr = (unsigned char *)buffer;
    unsigned char *w_ptr = NULL;
	unsigned char tmp_bufptr1 = 0;
	unsigned char tmp_bufptr2 = 0;
	unsigned char tmp_value = 0;
	int loop = 0;
	int inused_lines = 0;
	unsigned int copy_lr_sample = 0;
	unsigned int copy_size = 0;

	inused_lines = _mhal_alsa_dma_reader2_get_inused_lines();
	if (inused_lines == 0) {
		//MAD_PRINT(KERN_INFO "***** PCM Playback2 Buffer empty !! ***** \n");
	}

	copy_lr_sample = bytes / 2; /* L + R samples */
	copy_size = (dma_reader->channel_mode == E_MONO) ? (bytes * 2) : bytes;

	/* copy data to DMA Reader v2-2 buffer */
	if ( ((inused_lines * _MAD_BYTES_IN_LINE) + copy_size) < dma_reader->high_threshold) {
		if (dma_reader->channel_mode == E_MONO) {
			for (loop = 0; loop < copy_lr_sample; loop++) {
				tmp_bufptr1 = *bufptr++;
				tmp_bufptr2 = *bufptr++;

				*(dma_reader->buffer.w_ptr++) = tmp_bufptr1;
				*(dma_reader->buffer.w_ptr++) = tmp_bufptr2;
				*(dma_reader->buffer.w_ptr++) = tmp_bufptr1;
				*(dma_reader->buffer.w_ptr++) = tmp_bufptr2;

				if (dma_reader->buffer.w_ptr >= (dma_reader->buffer.addr + dma_reader->buffer.size))
					dma_reader->buffer.w_ptr = dma_reader->buffer.addr;
			}
		}
		else {
			for (loop = 0; loop < copy_lr_sample; loop++) {
				*(dma_reader->buffer.w_ptr++) = *bufptr++;
				*(dma_reader->buffer.w_ptr++) = *bufptr++;

				if (dma_reader->buffer.w_ptr >= (dma_reader->buffer.addr + dma_reader->buffer.size))
					dma_reader->buffer.w_ptr = dma_reader->buffer.addr;
			}
		}

 		/* a patch to prevent abnormal data in this HW module, so force to clear next 96 bytes of PCM buffer */
		w_ptr = dma_reader->buffer.w_ptr;
		for (loop = 0; loop < 96; loop++) {
			*(w_ptr++) = 0x00;

			if (w_ptr >= (dma_reader->buffer.addr + dma_reader->buffer.size))
				w_ptr = dma_reader->buffer.addr;
		}

		/*
		 * it's a patch here!
		 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
		 * in order to ensure MIU data can be updated since there is some queue in CPU side,
		 * we just read any byte of them then all MIU data can be updated.
		 */
		tmp_value = *(dma_reader->buffer.w_ptr);

		/* flush MIU */
		Chip_Flush_Memory();

		/* update copied size to DMA Reader v2-2 */
		copy_size += dma_reader->remain_size;
		_mhal_alsa_write2_mask_reg(0x3E60, 0x0010, 0x0000);
		_mhal_alsa_write2_mask_reg(0x3E68, 0xFFFF, (copy_size / _MAD_BYTES_IN_LINE));
		_mhal_alsa_write2_mask_reg(0x3E60, 0x0010, 0x0010);
		dma_reader->remain_size = copy_size % _MAD_BYTES_IN_LINE;
		dma_reader->written_size += (copy_size - dma_reader->remain_size);
		dma_reader->status = E_START;

		return bytes;
	}

	//MAD_PRINT(KERN_INFO "***** PCM Playback2 Buffer busy !! ***** \n");

	return 0;
}

/* Get information from DMA Reader2 */
static int _mhal_alsa_dma_reader2_get(int cmd, unsigned int *param)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Playback2\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_GET_BUFFER_SIZE:
		{
			*param = dma_reader->buffer.size;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_PERIOD_SIZE:
		{
			*param = dma_reader->period_size;
			break;
		}

		case E_PCM_PLAYBACK_GET_SAMPLE_RATE:
		{
			*param = dma_reader->sample_rate;
			break;
		}

		case E_PCM_PLAYBACK_GET_CHANNEL_MODE:
		{
			*param = dma_reader->channel_mode;
			break;
		}

		case E_PCM_PLAYBACK_GET_MAX_CHANNEL:
		{
			*param = _MAD_MAX_CHANNEL;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_COUNT:
		{
			*param = sizeof(mad_rates) / sizeof(mad_rates[0]);
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_LIST:
		{
			*param = (ptrdiff_t)&mad_rates;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_MASK:
		{
			*param = 0;
			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_INUSED_BYTES:
		{
			*param = _mhal_alsa_dma_reader2_get_inused_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_AVAIL_BYTES:
		{
			*param = _mhal_alsa_dma_reader2_get_avail_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_CONSUMED_BYTES:
		{
			int inused_bytes = 0;
			int consumed_bytes = 0;

			inused_bytes = _mhal_alsa_dma_reader2_get_inused_lines() * _MAD_BYTES_IN_LINE;
			consumed_bytes = dma_reader->written_size - inused_bytes;
			dma_reader->written_size = inused_bytes;
			*param = consumed_bytes;

			/* report actual level to driver layer */
			if (dma_reader->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_STR_STATUS:
		{
			*param = dma_reader->status;
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_HW_DMA_Reader2))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to DMA Reader2 */
static int _mhal_alsa_dma_reader2_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Playback2\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_SET_SAMPLE_RATE:
		{
			_mhal_alsa_dma_reader2_set_sample_rate(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_CHANNEL_MODE:
		{
			_mhal_alsa_dma_reader2_set_channel_mode(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_HW_DMA_Reader2);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get DMA Reader2 PCM buffer level */
static int _mhal_alsa_dma_reader2_get_inused_lines(void)
{
	int inused_lines = 0;

	/* Mask LEVEL_CNT_MASK before read */
	_mhal_alsa_write2_mask_reg(0x3E60, 0x0020, 0x0020);
	inused_lines = _mhal_alsa_read2_reg(0x3E74);
	_mhal_alsa_write2_mask_reg(0x3E60, 0x0020, 0x0000);

	return inused_lines;
}

/* Get DMA Reader2 PCM avail bytes */
static int _mhal_alsa_dma_reader2_get_avail_lines(void)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	int inused_lines = 0;
	int avail_lines = 0;

	inused_lines = _mhal_alsa_dma_reader2_get_inused_lines();
	avail_lines = (dma_reader->high_threshold / _MAD_BYTES_IN_LINE) - inused_lines;
	if (avail_lines < 0) {
		MAD_PRINT(KERN_ERR "Error! Incorrect inused lines %d!\n", inused_lines);
		avail_lines = 0;
	}

	return avail_lines;
}

/* Set smaple rate to DMA Reader2 */
static int _mhal_alsa_dma_reader2_set_sample_rate(unsigned int sample_rate)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	unsigned short synthrate, divisor;
	MAD_PRINT(KERN_INFO "Target sample rate is %u\n", sample_rate);

	dma_reader->sample_rate = sample_rate;

	/* New DMA Reader v2-2 setting
	 * Formula is :
	 * Synthesizer Rate = 216MHz / Divisor(1, 2, 4 or 8) * 1024 / 256 / Sampling Rate(32Khz, 44.1KHz or 48KHz)
	 */
	switch(sample_rate) {
		case 8000:
		{
			divisor = 2;
			synthrate = 0x6978;
			break;
		}

		case 11025:
		{
			divisor = 2;
			synthrate = 0x4C87;
			break;
		}

		case 12000:
		{
			divisor = 2;
			synthrate = 0x4650;
			break;
		}

		case 16000:
		{
			divisor = 1;
			synthrate = 0x6978;
			break;
		}

		case 22050:
		{
			divisor = 1;
			synthrate = 0x4C87;
			break;
		}

		case 24000:
		{
			divisor = 1;
			synthrate = 0x4650;
			break;
		}

		case 32000:
		{
			divisor = 0;
			synthrate = 0x6978;
			break;
		}

		case 44100:
		{
			divisor = 0;
			synthrate = 0x4C87;
			break;
		}

		case 48000:
		{
			divisor = 0;
			synthrate = 0x4650;
			break;
		}

		default:
		{
			MAD_PRINT(KERN_ERR "Error! un-supported sample rate %u !!!\n", sample_rate);
			divisor = 0;
			synthrate = 0x4650;
			dma_reader->sample_rate = 48000;
			break;
		}
	}

	/* synthersizer setting update */
	_mhal_alsa_write2_mask_reg(0x3E6C, 0x0030, (divisor << 4)); /* set divisor */
	_mhal_alsa_write2_reg(0x3E6E, synthrate); /* DMA synthesizer N.F. */
	_mhal_alsa_write2_mask_reg(0x3E6C, 0x0145, 0x0145); /* enable DMA synthesizer */

	return 0;
}

/* Set channel mode to DMA Reader2 */
static int _mhal_alsa_dma_reader2_set_channel_mode(unsigned int channel_mode)
{
	struct MStar_DMA_Reader_Struct *dma_reader = &g_dma_reader2;
	unsigned int buffer_size = 0;
	MAD_PRINT(KERN_INFO "Target channel mode is %u\n", channel_mode);

	dma_reader->channel_mode = channel_mode;
	buffer_size = ((dma_reader->sample_rate << dma_reader->channel_mode) * _MAD_DMA_READER2_QUEUE_SIZE) / 1000;
	if ((buffer_size % _MAD_BYTES_IN_LINE))
		buffer_size += (_MAD_BYTES_IN_LINE - (buffer_size % _MAD_BYTES_IN_LINE));

	dma_reader->buffer.size = (dma_reader->channel_mode == E_MONO) ? (buffer_size * 2) : buffer_size;
	dma_reader->period_size = dma_reader->buffer.size >> 2;
	dma_reader->high_threshold = dma_reader->buffer.size - (dma_reader->buffer.size >> 3);

	return 0;
}

/* Initiate PCM Playback3 */
static int _mhal_alsa_pcm_playback3_init(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	ptrdiff_t audio_pcm_dmaRdr_base_pa = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_ba = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_va = 0;
	unsigned int audio_pcm_dmaRdr_offset = _MAD_PCM_PLAYBACK3_BASE_OFFSET;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Playback3 engine\n");

	if (PCM_SHM_INFO.sw_dma_reader1_offset != 0) {
		audio_pcm_dmaRdr_offset = PCM_SHM_INFO.sw_dma_reader1_offset;
	}

	if ((pcm_playback->initialized_status != MAD_TRUE) || (pcm_playback->status != E_RESUME)) {
		audio_pcm_dmaRdr_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_dmaRdr_offset;
		audio_pcm_dmaRdr_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_dmaRdr_base_pa);

		if ((audio_pcm_dmaRdr_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM reader bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		if (g_pcm_dmaRdr3_base_va == 0) {
			g_pcm_dmaRdr3_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_dmaRdr_base_ba, pcm_playback->buffer.size);
			if (g_pcm_dmaRdr3_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Playback Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_dmaRdr_base_va = g_pcm_dmaRdr3_base_va;

		pcm_playback->str_mode_info.physical_addr = audio_pcm_dmaRdr_base_pa;
		pcm_playback->str_mode_info.bus_addr = audio_pcm_dmaRdr_base_ba;
		pcm_playback->str_mode_info.virtual_addr = audio_pcm_dmaRdr_base_va;

		pcm_playback->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_dmaRdr_base_pa = pcm_playback->str_mode_info.physical_addr;
		audio_pcm_dmaRdr_base_ba = pcm_playback->str_mode_info.bus_addr;
		audio_pcm_dmaRdr_base_va = pcm_playback->str_mode_info.virtual_addr;
	}

	/* init PCM playback3 buffer address */
	pcm_playback->buffer.addr = (unsigned char *)audio_pcm_dmaRdr_base_va;
	pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
	//MAD_PRINT(KERN_INFO "PCM Playback3 buffer start address = 0x%08X\n", pcm_playback->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Playback3 buffer end address = 0x%08X\n", (pcm_playback->buffer.addr + pcm_playback->buffer.size));

	/* enable PCM playback3 engine */
	_mhal_alsa_write_mask_reg(0x2D36, 0x0010, 0x0010);
	/* reset PCM playback3 engine */
	_mhal_alsa_write_mask_reg(0x2D36, 0x0001, 0x0001);

	/* clear PCM playback3 write pointer */
	_mhal_alsa_write_mask_reg(0x2D34, 0xFFFF, 0x0000);

	/* reset remain size */
	pcm_playback->remain_size = 0;

	/* reset written size */
	pcm_playback->written_size = 0;

	/* clear PCM playback3 pcm buffer */
	memset((void *)pcm_playback->buffer.addr, 0x00, pcm_playback->buffer.size);
	Chip_Flush_Memory();

	return 0;
}

/* Exit PCM Playback3 */
static int _mhal_alsa_pcm_playback3_exit(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Playback3 engine\n");

	/* reset PCM playback3 engine */
	_mhal_alsa_write_mask_reg(0x2D36, 0x0001, 0x0001);
	_mhal_alsa_write_mask_reg(0x2D36, 0x0010, 0x0000);

	/* clear PCM playback3 write pointer */
	_mhal_alsa_write_mask_reg(0x2D34, 0xFFFF, 0x0000);

	if (g_pcm_dmaRdr3_base_va != 0) {
		if (pcm_playback->buffer.addr) {
			iounmap((void *)pcm_playback->buffer.addr);
			pcm_playback->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Playback3 buffer address should not be 0 !\n");
		}

		g_pcm_dmaRdr3_base_va = 0;
	}

	pcm_playback->status = E_STOP;
	pcm_playback->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_SW_DMA_Reader1);
	}

	return 0;
}

/* Start PCM Playback3 */
static int _mhal_alsa_pcm_playback3_start(void)
{
	unsigned int input_mux_reg = 0x2C69;
	unsigned int ret = 0;
	//MAD_PRINT(KERN_INFO "Start MStar PCM Playback3 engine\n");

	if (g_shm_info) {
		ret = _mhal_alsa_get_input_mux_reg(E_SW_DMA_Reader1);
		if (ret) {
			input_mux_reg = ret;
		}
	}

	/* start PCM playback3 engine */
	_mhal_alsa_write_mask_reg(0x2D36, 0x0003, 0x0002);

	_mhal_alsa_write_mask_byte(input_mux_reg, 0x9F, 0x8C); /* CH7 sel to SW DMA Reader */

	return 0;
}

/* Stop PCM Playback3 */
static int _mhal_alsa_pcm_playback3_stop(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Playback3 engine\n");

	/* reset PCM playback3 engine */
	_mhal_alsa_write_mask_reg(0x2D36, 0x0001, 0x0001);

	/* clear PCM playback3 write pointer */
	_mhal_alsa_write_mask_reg(0x2D34, 0xFFFF, 0x0000);

	/* reset Write Pointer */
	pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;

	/* reset remain size */
	pcm_playback->remain_size = 0;

	/* reset written size */
	pcm_playback->written_size = 0;

	pcm_playback->status = E_STOP;

	return 0;
}

/* Resume PCM Playback3 */
static int _mhal_alsa_pcm_playback3_resume(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Playback3 engine\n");

	pcm_playback->status = E_RESUME;

	return 0;
}

/* Suspend PCM Playback3 */
static int _mhal_alsa_pcm_playback3_suspend(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Playback3 engine\n");

	pcm_playback->status = E_SUSPEND;

	return 0;
}

/* Write PCM to PCM Playback3 */
static unsigned int _mhal_alsa_pcm_playback3_write(void *buffer, unsigned int bytes)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	unsigned char *buffer_tmp = (unsigned char *)buffer;
	unsigned int w_ptr_offset = 0;
	unsigned int copy_lr_sample = 0;
	unsigned int copy_size = 0;
	int inused_bytes = 0;
	int loop = 0;

	copy_lr_sample = bytes / 2; /* L + R samples */
	copy_size = (pcm_playback->channel_mode == E_MONO) ? (bytes * 2) : bytes;
	inused_bytes = _mhal_alsa_pcm_playback3_get_inused_lines() * _MAD_BYTES_IN_LINE;

	if (inused_bytes == 0) {
		MAD_PRINT(KERN_INFO "***** PCM Playback3 Buffer empty !! ***** \n");
	}
	else if ((inused_bytes + copy_size) > pcm_playback->high_threshold) {
		//MAD_PRINT(KERN_INFO "***** PCM Playback3 Buffer full !! *****\n");
		return 0;
	}

	if (pcm_playback->channel_mode == E_MONO) {
		for (loop = 0; loop < copy_lr_sample; loop++) {
		unsigned char sample_lo = *(buffer_tmp++);
		unsigned char sample_hi = *(buffer_tmp++);

		*(pcm_playback->buffer.w_ptr++) = sample_lo;
		*(pcm_playback->buffer.w_ptr++) = sample_hi;
		*(pcm_playback->buffer.w_ptr++) = sample_lo;
		*(pcm_playback->buffer.w_ptr++) = sample_hi;

		if (pcm_playback->buffer.w_ptr >= (pcm_playback->buffer.addr + pcm_playback->buffer.size))
			pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
		}
	}
	else {
		for (loop = 0; loop < copy_lr_sample; loop++) {
		*(pcm_playback->buffer.w_ptr++) = *(buffer_tmp++);
		*(pcm_playback->buffer.w_ptr++) = *(buffer_tmp++);

		if (pcm_playback->buffer.w_ptr >= (pcm_playback->buffer.addr + pcm_playback->buffer.size))
			pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
		}
	}

	/* flush MIU */
	Chip_Flush_Memory();

	/* update PCM playback3 write pointer */
	w_ptr_offset = pcm_playback->buffer.w_ptr - pcm_playback->buffer.addr;
	_mhal_alsa_write_mask_reg(0x2D34, 0xFFFF, (w_ptr_offset / _MAD_BYTES_IN_LINE));
	pcm_playback->written_size += (copy_size - pcm_playback->remain_size);
	pcm_playback->status = E_START;
	/* ensure write pointer can be applied */
	mdelay(1);

	return bytes;
}

/* Get information from PCM Playback3 */
static int _mhal_alsa_pcm_playback3_get(int cmd, unsigned int *param)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Playback3\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_GET_BUFFER_SIZE:
		{
			*param = pcm_playback->buffer.size;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_PERIOD_SIZE:
		{
			*param = pcm_playback->period_size;
			break;
		}

		case E_PCM_PLAYBACK_GET_SAMPLE_RATE:
		{
			*param = pcm_playback->sample_rate;
			break;
		}

		case E_PCM_PLAYBACK_GET_CHANNEL_MODE:
		{
			*param = pcm_playback->channel_mode;
			break;
		}

		case E_PCM_PLAYBACK_GET_MAX_CHANNEL:
		{
			*param = _MAD_MAX_CHANNEL;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_COUNT:
		{
			*param = sizeof(mad_rates) / sizeof(mad_rates[0]);
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_LIST:
		{
			*param = (unsigned int)&mad_rates;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_MASK:
		{
			*param = 0;
			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_INUSED_BYTES:
		{
			*param = _mhal_alsa_pcm_playback3_get_inused_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_playback3_get_avail_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_CONSUMED_BYTES:
		{
			int inused_bytes = 0;
			int consumed_bytes = 0;

			inused_bytes = _mhal_alsa_pcm_playback3_get_inused_lines() * _MAD_BYTES_IN_LINE;
			consumed_bytes = pcm_playback->written_size - inused_bytes;
			pcm_playback->written_size = inused_bytes;
			*param = consumed_bytes;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_STR_STATUS:
		{
			*param = pcm_playback->status;
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_SW_DMA_Reader1))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to PCM Playback3 */
static int _mhal_alsa_pcm_playback3_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Playback3\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_SET_SAMPLE_RATE:
		{
			_mhal_alsa_pcm_playback3_set_sample_rate(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_CHANNEL_MODE:
		{
			_mhal_alsa_pcm_playback3_set_channel_mode(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_SW_DMA_Reader1);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get PCM Playback3 buffer level */
static int _mhal_alsa_pcm_playback3_get_inused_lines(void)
{
	int inused_lines = 0;

	_mhal_alsa_write_mask_reg(0x2D36, 0x0008, 0x0008);
	inused_lines = _mhal_alsa_read_reg(0x2DE2);
	_mhal_alsa_write_mask_reg(0x2D36, 0x0008, 0x0000);

	return inused_lines;
}

/* Get PCM Playback3 avail bytes */
static int _mhal_alsa_pcm_playback3_get_avail_lines(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	int inused_lines = 0;
	int avail_lines = 0;

	inused_lines = _mhal_alsa_pcm_playback3_get_inused_lines();
	avail_lines = (pcm_playback->buffer.size / _MAD_BYTES_IN_LINE) - inused_lines;
	if (avail_lines < 0) {
		MAD_PRINT(KERN_ERR "Error! Incorrect inused lines %d!\n", inused_lines);
		avail_lines = 0;
	}

	return avail_lines;
}

/* Set smaple rate to PCM Playback3 */
static int _mhal_alsa_pcm_playback3_set_sample_rate(unsigned int sample_rate)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	unsigned short synthrate_h;
	unsigned short synthrate_l;
	MAD_PRINT(KERN_INFO "Target sample rate is %u\n", sample_rate);

	pcm_playback->sample_rate = sample_rate;

	switch(sample_rate) {
		case 8000:
		{
			synthrate_h = 0x6978;
			synthrate_l = 0x0000;
			break;
		}

		case 11025:
		{
			synthrate_h = 0x4C87;
			synthrate_l = 0xd634;
			break;
		}

		case 12000:
		{
			synthrate_h = 0x4650;
			synthrate_l = 0x0000;
			break;
		}

		case 16000:
		{
			synthrate_h = 0x34BC;
			synthrate_l = 0x0000;
			break;
		}

		case 22050:
		{
			synthrate_h = 0x2643;
			synthrate_l = 0xeb1a;
			break;
		}

		case 24000:
		{
			synthrate_h = 0x2328;
			synthrate_l = 0x0000;
			break;
		}

		case 32000:
		{
			synthrate_h = 0x1A5E;
			synthrate_l = 0x0000;
			break;
		}

		case 44100:
		{
			synthrate_h = 0x1321;
			synthrate_l = 0xf58d;
			break;
		}

		case 48000:
		{
			synthrate_h = 0x1194;
			synthrate_l = 0x0000;
			break;
		}

		default:
		{
			MAD_PRINT(KERN_ERR "Error! un-supported sample rate %u !!!\n", sample_rate);
			synthrate_h = 0x1194;
			synthrate_l = 0x0000;
			pcm_playback->sample_rate = 48000;
			break;
		}
	}

	_mhal_alsa_write_mask_reg(0x2C24, 0xa080, 0xa080);
	_mhal_alsa_write_mask_reg(0x2C26, 0xFFFF, synthrate_h);
	_mhal_alsa_write_mask_reg(0x2C28, 0xFFFF, synthrate_l);

	_mhal_alsa_write_mask_reg(0x2C24, 0x1000, 0x1000);
	mdelay(1);
	_mhal_alsa_write_mask_reg(0x2C24, 0x1000, 0x0000);

	return 0;
}

/* Set channel mode to PCM Playback3 */
static int _mhal_alsa_pcm_playback3_set_channel_mode(unsigned int channel_mode)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback3;
	unsigned int buffer_size = 0;
	MAD_PRINT(KERN_INFO "Target channel mode is %u %d\n", channel_mode, pcm_playback->sample_rate);

	pcm_playback->channel_mode = channel_mode;
	buffer_size = _MAD_PCM_PLAYBACK3_TOTAL_BUF_SIZE >> 1;
	pcm_playback->period_size = buffer_size >> 2;
	pcm_playback->buffer.size = buffer_size;
	pcm_playback->high_threshold = pcm_playback->buffer.size - (pcm_playback->buffer.size >> 3);

	return 0;
}

/* Initiate PCM Playback4 */
static int _mhal_alsa_pcm_playback4_init(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	ptrdiff_t audio_pcm_dmaRdr_base_pa = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_ba = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_va = 0;
    unsigned int audio_pcm_dmaRdr_offset = _MAD_PCM_PLAYBACK4_BASE_OFFSET;
    MAD_PRINT(KERN_INFO "Initiate MStar PCM Playback4 engine\n");

    if (g_shm_info && (g_shm_info->sw_dma_reader3_offset != 0)) {
        audio_pcm_dmaRdr_offset = g_shm_info->sw_dma_reader3_offset;
    }

    if ((pcm_playback->initialized_status != MAD_TRUE) || (pcm_playback->status != E_RESUME)) {
        audio_pcm_dmaRdr_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_dmaRdr_offset;
        audio_pcm_dmaRdr_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_dmaRdr_base_pa);

		if ((audio_pcm_dmaRdr_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM reader bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		if (g_pcm_dmaRdr4_base_va == 0)	{
			g_pcm_dmaRdr4_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_dmaRdr_base_ba, pcm_playback->buffer.size);
			if (g_pcm_dmaRdr4_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Playback Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_dmaRdr_base_va = g_pcm_dmaRdr4_base_va;

		pcm_playback->str_mode_info.physical_addr = audio_pcm_dmaRdr_base_pa;
		pcm_playback->str_mode_info.bus_addr = audio_pcm_dmaRdr_base_ba;
		pcm_playback->str_mode_info.virtual_addr = audio_pcm_dmaRdr_base_va;

		pcm_playback->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_dmaRdr_base_pa = pcm_playback->str_mode_info.physical_addr;
		audio_pcm_dmaRdr_base_ba = pcm_playback->str_mode_info.bus_addr;
		audio_pcm_dmaRdr_base_va = pcm_playback->str_mode_info.virtual_addr;
	}

	/* init PCM playback4 buffer address */
	pcm_playback->buffer.addr = (unsigned char *)audio_pcm_dmaRdr_base_va;
	pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
	//MAD_PRINT(KERN_INFO "PCM Playback4 buffer start address = 0x%08X\n", pcm_playback->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Playback4 buffer end address = 0x%08X\n", (pcm_playback->buffer.addr + pcm_playback->buffer.size));

	/* enable PCM playback4 engine */
	_mhal_alsa_write_mask_reg(0x2E26, 0x0010, 0x0010);
	/* reset PCM playback4 engine */
	_mhal_alsa_write_mask_reg(0x2E26, 0x0001, 0x0001);

	/* clear PCM playback4 write pointer */
	_mhal_alsa_write_mask_reg(0x2E24, 0xFFFF, 0x0000);

	/* reset remain size */
	pcm_playback->remain_size = 0;

	/* reset written size */
	pcm_playback->written_size = 0;

	/* clear PCM playback4 pcm buffer */
	memset((void *)pcm_playback->buffer.addr, 0x00, pcm_playback->buffer.size);
	Chip_Flush_Memory();

	return 0;
}

/* Exit PCM Playback4 */
static int _mhal_alsa_pcm_playback4_exit(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Playback4 engine\n");

	/* reset PCM playback4 engine */
	_mhal_alsa_write_mask_reg(0x2E26, 0x0001, 0x0001);
	_mhal_alsa_write_mask_reg(0x2E26, 0x0010, 0x0000);

	/* clear PCM playback4 write pointer */
	_mhal_alsa_write_mask_reg(0x2E24, 0xFFFF, 0x0000);

	if (g_pcm_dmaRdr4_base_va != 0) {
		if (pcm_playback->buffer.addr) {
			iounmap((void *)pcm_playback->buffer.addr);
			pcm_playback->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Playback4 buffer address should not be 0 !\n");
		}

		g_pcm_dmaRdr4_base_va = 0;
	}

	pcm_playback->status = E_STOP;
	pcm_playback->initialized_status = MAD_FALSE;

    if (g_shm_info) {
        g_shm_info->dma_use_status &= ~(0x1 << E_SW_DMA_Reader3);
    }

	return 0;
}

/* Start PCM Playback4 */
static int _mhal_alsa_pcm_playback4_start(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	MAD_PRINT(KERN_INFO "Start MStar PCM Playback4 engine\n");

	/* start PCM playback4 engine */
	_mhal_alsa_write_mask_reg(0x2E26, 0x0003, 0x0002);

	return 0;
}

/* Stop PCM Playback4 */
static int _mhal_alsa_pcm_playback4_stop(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Playback4 engine\n");

	/* reset PCM playback4 engine */
	_mhal_alsa_write_mask_reg(0x2E26, 0x0001, 0x0001);

	/* clear PCM playback4 write pointer */
	_mhal_alsa_write_mask_reg(0x2E24, 0xFFFF, 0x0000);

	/* reset Write Pointer */
	pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;

	/* reset remain size */
	pcm_playback->remain_size = 0;

	/* reset written size */
	pcm_playback->written_size = 0;

	pcm_playback->status = E_STOP;

	return 0;
}

/* Resume PCM Playback4 */
static int _mhal_alsa_pcm_playback4_resume(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Playback4 engine\n");

	pcm_playback->status = E_RESUME;

	return 0;
}

/* Suspend PCM Playback4 */
static int _mhal_alsa_pcm_playback4_suspend(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Playback4 engine\n");

	pcm_playback->status = E_SUSPEND;

	return 0;
}

/* Write PCM to PCM Playback4 */
static unsigned int _mhal_alsa_pcm_playback4_write(void *buffer, unsigned int bytes)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	unsigned char *buffer_tmp = (unsigned char *)buffer;
	unsigned int w_ptr_offset = 0;
	unsigned int copy_lr_sample = 0;
	unsigned int copy_size = 0;
	int inused_bytes = 0;
	int loop = 0;

	copy_lr_sample = bytes / 2; /* L + R samples */
	copy_size = (pcm_playback->channel_mode == E_MONO) ? (bytes * 2) : bytes;
	inused_bytes = _mhal_alsa_pcm_playback4_get_inused_lines() * _MAD_BYTES_IN_LINE;

	if (inused_bytes == 0) {
		MAD_PRINT(KERN_INFO "***** PCM Playback4 Buffer empty !! ***** \n");
	}
	else if ((inused_bytes + copy_size) > pcm_playback->high_threshold) {
		//MAD_PRINT(KERN_INFO "***** PCM Playback4 Buffer full !! *****\n");
		return 0;
	}

	if (pcm_playback->channel_mode == E_MONO) {
		for (loop = 0; loop < copy_lr_sample; loop++) {
			unsigned char sample_lo = *(buffer_tmp++);
			unsigned char sample_hi = *(buffer_tmp++);

			*(pcm_playback->buffer.w_ptr++) = sample_lo;
			*(pcm_playback->buffer.w_ptr++) = sample_hi;
			*(pcm_playback->buffer.w_ptr++) = sample_lo;
			*(pcm_playback->buffer.w_ptr++) = sample_hi;

			if (pcm_playback->buffer.w_ptr >= (pcm_playback->buffer.addr + pcm_playback->buffer.size))
				pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
		}
	}
	else {
		for (loop = 0; loop < copy_lr_sample; loop++) {
			*(pcm_playback->buffer.w_ptr++) = *(buffer_tmp++);
			*(pcm_playback->buffer.w_ptr++) = *(buffer_tmp++);

			if (pcm_playback->buffer.w_ptr >= (pcm_playback->buffer.addr + pcm_playback->buffer.size))
				pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
		}
	}

	/* flush MIU */
	Chip_Flush_Memory();

	/* update PCM playback4 write pointer */
	w_ptr_offset = pcm_playback->buffer.w_ptr - pcm_playback->buffer.addr;
	_mhal_alsa_write_mask_reg(0x2E24, 0xFFFF, (w_ptr_offset / _MAD_BYTES_IN_LINE));
	pcm_playback->written_size += (copy_size - pcm_playback->remain_size);
	pcm_playback->status = E_START;
	/* ensure write pointer can be applied */
	mdelay(1);

	return bytes;
}

/* Get information from PCM Playback4 */
static int _mhal_alsa_pcm_playback4_get(int cmd, unsigned int *param)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Playback4\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_GET_BUFFER_SIZE:
		{
			*param = pcm_playback->buffer.size;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_PERIOD_SIZE:
		{
			*param = pcm_playback->period_size;
			break;
		}

		case E_PCM_PLAYBACK_GET_SAMPLE_RATE:
		{
			*param = pcm_playback->sample_rate;
			break;
		}

		case E_PCM_PLAYBACK_GET_CHANNEL_MODE:
		{
			*param = pcm_playback->channel_mode;
			break;
		}

		case E_PCM_PLAYBACK_GET_MAX_CHANNEL:
		{
			*param = _MAD_MAX_CHANNEL;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_COUNT:
		{
			*param = sizeof(mad_rates) / sizeof(mad_rates[0]);
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_LIST:
		{
			*param = (unsigned int)&mad_rates;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_MASK:
		{
			*param = 0;
			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_INUSED_BYTES:
		{
			*param = _mhal_alsa_pcm_playback4_get_inused_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_playback4_get_avail_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
        }

		case E_PCM_PLAYBACK_GET_BUFFER_CONSUMED_BYTES:
		{
			int inused_bytes = 0;
			int consumed_bytes = 0;

			inused_bytes = _mhal_alsa_pcm_playback4_get_inused_lines() * _MAD_BYTES_IN_LINE;
			consumed_bytes = pcm_playback->written_size - inused_bytes;
			pcm_playback->written_size = inused_bytes;
			*param = consumed_bytes;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}
			break;
		}

		case E_PCM_PLAYBACK_GET_STR_STATUS:
		{
			*param = pcm_playback->status;
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_SW_DMA_Reader3))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to PCM Playback4 */
static int _mhal_alsa_pcm_playback4_set(int cmd, unsigned int *param)
{
	int err = 0;
	MAD_PRINT(KERN_INFO "Set parameter to PCM Playback4\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_SET_SAMPLE_RATE:
		{
			_mhal_alsa_pcm_playback4_set_sample_rate(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_CHANNEL_MODE:
		{
			_mhal_alsa_pcm_playback4_set_channel_mode(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_SW_DMA_Reader3);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get PCM Playback4 buffer level */
static int _mhal_alsa_pcm_playback4_get_inused_lines(void)
{
	int inused_lines = 0;

	_mhal_alsa_write_mask_reg(0x2E26, 0x0008, 0x0008);
	inused_lines = _mhal_alsa_read_reg(0x2E12);
	_mhal_alsa_write_mask_reg(0x2E26, 0x0008, 0x0000);

	return inused_lines;
}

/* Get PCM Playback4 avail bytes */
static int _mhal_alsa_pcm_playback4_get_avail_lines(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	int inused_lines = 0;
	int avail_lines = 0;

	inused_lines = _mhal_alsa_pcm_playback4_get_inused_lines();
	avail_lines = (pcm_playback->buffer.size / _MAD_BYTES_IN_LINE) - inused_lines;
	if (avail_lines < 0) {
		MAD_PRINT(KERN_ERR "Error! Incorrect inused lines %d!\n", inused_lines);
		avail_lines = 0;
	}

	return avail_lines;
}

/* Set smaple rate to PCM Playback4 */
static int _mhal_alsa_pcm_playback4_set_sample_rate(unsigned int sample_rate)
{
	if (sample_rate != 48000) {
		MAD_PRINT(KERN_ERR "Warning, SW DMA Reader 3 sample rate only support 48000, sound may abnormal\n");
	}

	return 0;
}

/* Set channel mode to PCM Playback4 */
static int _mhal_alsa_pcm_playback4_set_channel_mode(unsigned int channel_mode)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback4;
	unsigned int buffer_size = 0;
	MAD_PRINT(KERN_INFO "Target channel mode is %u %d\n", channel_mode, pcm_playback->sample_rate);

	pcm_playback->channel_mode = channel_mode;
	buffer_size = _MAD_PCM_PLAYBACK4_TOTAL_BUF_SIZE >> 1;
	pcm_playback->period_size = buffer_size >> 2;
	pcm_playback->buffer.size = buffer_size;
	pcm_playback->high_threshold = pcm_playback->buffer.size - (pcm_playback->buffer.size >> 3);

	return 0;
}

/* Initiate PCM Playback7 */
static int _mhal_alsa_pcm_playback7_init(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	ptrdiff_t audio_pcm_dmaRdr_base_pa = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_ba = 0;
	ptrdiff_t audio_pcm_dmaRdr_base_va = 0;
	unsigned int audio_pcm_dmaRdr_offset = _MAD_PCM_PLAYBACK7_BASE_OFFSET;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Playback7 engine\n");

	if(PCM_SHM_INFO.sw_dma_reader2_offset != 0) {
		audio_pcm_dmaRdr_offset = PCM_SHM_INFO.sw_dma_reader2_offset;
	}

	if ((pcm_playback->initialized_status != MAD_TRUE) || (pcm_playback->status != E_RESUME)) {
		audio_pcm_dmaRdr_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_dmaRdr_offset;
		audio_pcm_dmaRdr_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_dmaRdr_base_pa);

		if ((audio_pcm_dmaRdr_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM reader bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		if (g_pcm_dmaRdr7_base_va == 0)	{
			g_pcm_dmaRdr7_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_dmaRdr_base_ba, pcm_playback->buffer.size);
			if (g_pcm_dmaRdr7_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Playback Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_dmaRdr_base_va = g_pcm_dmaRdr7_base_va;

		pcm_playback->str_mode_info.physical_addr = audio_pcm_dmaRdr_base_pa;
		pcm_playback->str_mode_info.bus_addr = audio_pcm_dmaRdr_base_ba;
		pcm_playback->str_mode_info.virtual_addr = audio_pcm_dmaRdr_base_va;

		pcm_playback->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_dmaRdr_base_pa = pcm_playback->str_mode_info.physical_addr;
		audio_pcm_dmaRdr_base_ba = pcm_playback->str_mode_info.bus_addr;
		audio_pcm_dmaRdr_base_va = pcm_playback->str_mode_info.virtual_addr;
	}

	/* init PCM playback7 buffer address */
	pcm_playback->buffer.addr = (unsigned char *)audio_pcm_dmaRdr_base_va;
	pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
	//MAD_PRINT(KERN_INFO "PCM Playback7 buffer start address = 0x%08X\n", pcm_playback->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Playback7 buffer end address = 0x%08X\n", (pcm_playback->buffer.addr + pcm_playback->buffer.size));

	/* enable PCM playback7 engine */
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0010, 0x0010);
	/* reset PCM playback7 engine */
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0001, 0x0001);

	/* clear PCM playback7 write pointer */
	_mhal_alsa_write_mask_reg(0x2D3C, 0xFFFF, 0x0000);

	/* reset remain size */
	pcm_playback->remain_size = 0;

	/* reset written size */
	pcm_playback->written_size = 0;

	/* clear PCM playback7 pcm buffer */
	memset((void *)pcm_playback->buffer.addr, 0x00, pcm_playback->buffer.size);
	Chip_Flush_Memory();

	return 0;
}

/* Exit PCM Playback7 */
static int _mhal_alsa_pcm_playback7_exit(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Playback7 engine\n");

	/* reset PCM playback7 engine */
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0001, 0x0001);
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0010, 0x0000);

	if (pcm_playback->mmap_flag) {
		_mhal_alsa_write_mask_reg(0x2D3E, 0x0004, 0x0000);
		pcm_playback->mmap_flag = 0;
	}

	/* clear PCM playback7 write pointer */
	_mhal_alsa_write_mask_reg(0x2D3C, 0xFFFF, 0x0000);

	if (g_pcm_dmaRdr7_base_va != 0) {
		if (pcm_playback->buffer.addr) {
			iounmap((void *)pcm_playback->buffer.addr);
			pcm_playback->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Playback7 buffer address should not be 0 !\n");
		}

		g_pcm_dmaRdr7_base_va = 0;
	}

	pcm_playback->status = E_STOP;
	pcm_playback->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_SW_DMA_Reader2);
	}

	return 0;
}

/* Start PCM Playback7 */
static int _mhal_alsa_pcm_playback7_start(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	//MAD_PRINT(KERN_INFO "Start MStar PCM Playback7 engine\n");

	// set free run mode
	if (pcm_playback->mmap_flag) {
		_mhal_alsa_write_mask_reg(0x2D3E, 0x0004, 0x0004);
	}

	/* start PCM playback7 engine */
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0003, 0x0002);

	return 0;
}

/* Stop PCM Playback7 */
static int _mhal_alsa_pcm_playback7_stop(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Playback7 engine\n");

	/* reset PCM playback7 engine */
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0001, 0x0001);

	/* clear PCM playback7 write pointer */
	_mhal_alsa_write_mask_reg(0x2D3C, 0xFFFF, 0x0000);

	/* reset Write Pointer */
	pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;

	/* reset remain size */
	pcm_playback->remain_size = 0;

	/* reset written size */
	pcm_playback->written_size = 0;

	pcm_playback->status = E_STOP;

	return 0;
}

/* Resume PCM Playback7 */
static int _mhal_alsa_pcm_playback7_resume(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Playback7 engine\n");

	pcm_playback->status = E_RESUME;

	return 0;
}

/* Suspend PCM Playback7 */
static int _mhal_alsa_pcm_playback7_suspend(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Playback7 engine\n");

	pcm_playback->status = E_SUSPEND;

	return 0;
}

/* Write PCM to PCM Playback7 */
static unsigned int _mhal_alsa_pcm_playback7_write(void *buffer, unsigned int bytes)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	unsigned char *buffer_tmp = (unsigned char *)buffer;
	unsigned int w_ptr_offset = 0;
	unsigned int copy_lr_sample = 0;
	unsigned int copy_size = 0;
	int inused_bytes = 0;
	int loop = 0;

	copy_lr_sample = bytes / 2; /* L + R samples */
	copy_size = (pcm_playback->channel_mode == E_MONO) ? (bytes * 2) : bytes;
	inused_bytes = _mhal_alsa_pcm_playback7_get_inused_lines() * _MAD_BYTES_IN_LINE;

	if (inused_bytes == 0) {
		MAD_PRINT(KERN_INFO "***** PCM Playback7 Buffer empty !! ***** \n");
	}
	else if ((inused_bytes + copy_size) > pcm_playback->high_threshold) {
		//MAD_PRINT(KERN_INFO "***** PCM Playback7 Buffer full !! *****\n");
		return 0;
	}

	if (pcm_playback->channel_mode == E_MONO) {
		for (loop = 0; loop < copy_lr_sample; loop++) {
			unsigned char sample_lo = *(buffer_tmp++);
			unsigned char sample_hi = *(buffer_tmp++);

			*(pcm_playback->buffer.w_ptr++) = sample_lo;
			*(pcm_playback->buffer.w_ptr++) = sample_hi;
			*(pcm_playback->buffer.w_ptr++) = sample_lo;
			*(pcm_playback->buffer.w_ptr++) = sample_hi;

			if (pcm_playback->buffer.w_ptr >= (pcm_playback->buffer.addr + pcm_playback->buffer.size))
				pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
		}
	}
	else {
		for (loop = 0; loop < copy_lr_sample; loop++) {
			*(pcm_playback->buffer.w_ptr++) = *(buffer_tmp++);
			*(pcm_playback->buffer.w_ptr++) = *(buffer_tmp++);

			if (pcm_playback->buffer.w_ptr >= (pcm_playback->buffer.addr + pcm_playback->buffer.size))
				pcm_playback->buffer.w_ptr = pcm_playback->buffer.addr;
		}
	}

	/* flush MIU */
	Chip_Flush_Memory();

	/* update PCM playback7 write pointer */
	w_ptr_offset = pcm_playback->buffer.w_ptr - pcm_playback->buffer.addr;
	_mhal_alsa_write_mask_reg(0x2D3C, 0xFFFF, (w_ptr_offset / _MAD_BYTES_IN_LINE));
	pcm_playback->written_size += (copy_size - pcm_playback->remain_size);
	pcm_playback->status = E_START;
	/* ensure write pointer can be applied */
	mdelay(1);

	return bytes;
}

/* Get information from PCM Playback7 */
static int _mhal_alsa_pcm_playback7_get(int cmd, unsigned int *param)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Playback7\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_GET_BUFFER_SIZE:
		{
			*param = pcm_playback->buffer.size;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_PERIOD_SIZE:
		{
			*param = pcm_playback->period_size;
			break;
		}

		case E_PCM_PLAYBACK_GET_SAMPLE_RATE:
		{
			*param = pcm_playback->sample_rate;
			break;
		}

		case E_PCM_PLAYBACK_GET_CHANNEL_MODE:
		{
			*param = pcm_playback->channel_mode;
			break;
		}

		case E_PCM_PLAYBACK_GET_MAX_CHANNEL:
		{
			*param = _MAD_MAX_CHANNEL;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_COUNT:
		{
			*param = sizeof(mad_rates) / sizeof(mad_rates[0]);
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_LIST:
		{
			*param = (unsigned int)&mad_rates;
			break;
		}

		case E_PCM_PLAYBACK_GET_CONSTRAINTS_MASK:
		{
			*param = 0;
			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_INUSED_BYTES:
		{
			*param = _mhal_alsa_pcm_playback7_get_inused_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_BUFFER_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_playback7_get_avail_lines() * _MAD_BYTES_IN_LINE;

			/* report actual level to driver layer */
			if (pcm_playback->channel_mode == E_MONO) {
				*param >>= 1;
			}

			break;
	}

		case E_PCM_PLAYBACK_GET_BUFFER_CONSUMED_BYTES:
		{
			int inused_bytes = 0;
			int consumed_bytes = 0;

			if (pcm_playback->mmap_flag == 0) {
				inused_bytes = _mhal_alsa_pcm_playback7_get_inused_lines() * _MAD_BYTES_IN_LINE;
				consumed_bytes = pcm_playback->written_size - inused_bytes;
				pcm_playback->written_size = inused_bytes;
				*param = consumed_bytes;

				/* report actual level to driver layer */
				if (pcm_playback->channel_mode == E_MONO) {
					*param >>= 1;
				}
			}
			else {
				unsigned int curr_written_samples = 0;

				_mhal_alsa_write_mask_reg(0x2D3E, 0x0008, 0x0008);
				curr_written_samples = _mhal_alsa_read_reg(0x2E08) + (_mhal_alsa_read_reg(0x2E06) << 16);
				_mhal_alsa_write_mask_reg(0x2D3E, 0x0008, 0x0000);

				/* SW DMA report sample counts base on channel number */
				if (curr_written_samples >= pcm_playback->written_size) {
					consumed_bytes = ((curr_written_samples - pcm_playback->written_size) << 1) << 1;
				}
				else {
					consumed_bytes = ((0xFFFFFFFF - pcm_playback->written_size + curr_written_samples) << 1) << 1;
				}

				pcm_playback->written_size = curr_written_samples;

				*param = consumed_bytes;
			}

			break;
		}

		case E_PCM_PLAYBACK_GET_STR_STATUS:
		{
			*param = pcm_playback->status;
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_PLAYBACK_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_SW_DMA_Reader2))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to PCM Playback7 */
static int _mhal_alsa_pcm_playback7_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Playback7\n");

	switch(cmd) {
		case E_PCM_PLAYBACK_SET_SAMPLE_RATE:
		{
			_mhal_alsa_pcm_playback7_set_sample_rate(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_CHANNEL_MODE:
		{
			_mhal_alsa_pcm_playback7_set_channel_mode(*param);
			break;
		}

		case E_PCM_PLAYBACK_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_SW_DMA_Reader2);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get PCM Playback7 buffer level */
static int _mhal_alsa_pcm_playback7_get_inused_lines(void)
{
	int inused_lines = 0;

	_mhal_alsa_write_mask_reg(0x2D3E, 0x0008, 0x0008);
	inused_lines = _mhal_alsa_read_reg(0x2DE8);
	_mhal_alsa_write_mask_reg(0x2D3E, 0x0008, 0x0000);

	return inused_lines;
}

/* Get PCM Playback7 avail bytes */
static int _mhal_alsa_pcm_playback7_get_avail_lines(void)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	int inused_lines = 0;
	int avail_lines = 0;

	inused_lines = _mhal_alsa_pcm_playback7_get_inused_lines();
	avail_lines = (pcm_playback->buffer.size / _MAD_BYTES_IN_LINE) - inused_lines;
	if (avail_lines < 0) {
		MAD_PRINT(KERN_ERR "Error! Incorrect inused lines %d!\n", inused_lines);
		avail_lines = 0;
	}

	return avail_lines;
}

/* Set smaple rate to PCM Playback7 */
static int _mhal_alsa_pcm_playback7_set_sample_rate(unsigned int sample_rate)
{
	if (sample_rate != 48000) {
		MAD_PRINT(KERN_ERR "Warning, SW DMA Reader 2 sample rate only support 48000, sound may abnormal\n");
	}

	return 0;
}

/* Set channel mode to PCM Playback7 */
static int _mhal_alsa_pcm_playback7_set_channel_mode(unsigned int channel_mode)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	unsigned int buffer_size = 0;
	MAD_PRINT(KERN_INFO "Target channel mode is %u %d\n", channel_mode, pcm_playback->sample_rate);

	pcm_playback->channel_mode = channel_mode;
	buffer_size = _MAD_PCM_PLAYBACK7_TOTAL_BUF_SIZE >> 1;
	pcm_playback->period_size = buffer_size >> 2;
	pcm_playback->buffer.size = buffer_size;
	pcm_playback->high_threshold = pcm_playback->buffer.size - (pcm_playback->buffer.size >> 3);

	return 0;
}

static int _mhal_alsa_pcm_playback7_mmap(struct vm_area_struct *vma)
{
	struct MStar_DMA_Reader_Struct *pcm_playback = &g_pcm_playback7;
	int ret = 0;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start,
             pcm_playback->str_mode_info.bus_addr >> PAGE_SHIFT,
               vma->vm_end - vma->vm_start, vma->vm_page_prot);

	pcm_playback->mmap_flag = 1;

    return ret;
}

/* Initiate PCM Capture1 */
static int _mhal_alsa_pcm_capture1_init(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	ptrdiff_t audio_pcm_capture_base_pa = 0;
	ptrdiff_t audio_pcm_capture_base_ba = 0;
	ptrdiff_t audio_pcm_capture_base_va = 0;
	unsigned int w_ptr_offset = 0;
	unsigned int audio_pcm_capture_offset = _MAD_PCM_CAPTURE1_BASE_OFFSET;
	unsigned char tmp_value = 0;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Capture1 engine\n");

	if (PCM_SHM_INFO.capture1_offset != 0) {
		audio_pcm_capture_offset = PCM_SHM_INFO.capture1_offset;
	}

	if ((pcm_capture->initialized_status != MAD_TRUE) || (pcm_capture->status != E_RESUME)) {
		audio_pcm_capture_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_capture_offset;
		audio_pcm_capture_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_capture_base_pa);

		if ((audio_pcm_capture_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM capture bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		pcm_capture->buffer.size = _MAD_PCM_CAPTURE_BUF_SIZE;
		if (g_pcm_capture1_base_va == 0) {
			g_pcm_capture1_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_capture_base_ba, pcm_capture->buffer.size);
			if (g_pcm_capture1_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Capture1 Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_capture_base_va = g_pcm_capture1_base_va;

		pcm_capture->str_mode_info.physical_addr = audio_pcm_capture_base_pa;
		pcm_capture->str_mode_info.bus_addr = audio_pcm_capture_base_ba;
		pcm_capture->str_mode_info.virtual_addr = audio_pcm_capture_base_va;

		pcm_capture->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_capture_base_pa = pcm_capture->str_mode_info.physical_addr;
		audio_pcm_capture_base_ba = pcm_capture->str_mode_info.bus_addr;
		audio_pcm_capture_base_va = pcm_capture->str_mode_info.virtual_addr;
	}

	/* init PCM capture1 buffer address */
	pcm_capture->buffer.addr = (unsigned char *)audio_pcm_capture_base_va;
	//MAD_PRINT(KERN_INFO "PCM Capture1 buffer start address = 0x%08X\n", pcm_capture->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Capture1 buffer end address = 0x%08X\n", (pcm_capture->buffer.addr + pcm_capture->buffer.size));

	/* clear all PCM capture1 buffer */
	memset((void *)pcm_capture->buffer.addr, 0x00, _MAD_PCM_CAPTURE_BUF_SIZE);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *(pcm_capture->buffer.addr);

	Chip_Flush_Memory();

	/* reset PCM capture1 write pointer */
	w_ptr_offset = _mhal_alsa_read_reg(0x2DF0) * _MAD_BYTES_IN_LINE;
	pcm_capture->buffer.w_ptr = pcm_capture->buffer.addr + w_ptr_offset;

	/* reset PCM capture1 read pointer */
	pcm_capture->buffer.r_ptr = pcm_capture->buffer.w_ptr;
	_mhal_alsa_write_reg(0x2DD4, (unsigned short)(w_ptr_offset / _MAD_BYTES_IN_LINE));

	/* reset PCM capture1 buffer size */
	_mhal_alsa_write_reg(0x2DD6, (unsigned short)(pcm_capture->buffer.size / _MAD_BYTES_IN_LINE));

	return 0;
}

/* Exit PCM Capture1 */
static int _mhal_alsa_pcm_capture1_exit(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Capture1 engine\n");

	/* clear PCM capture1 buffer size */
	_mhal_alsa_write_reg(0x2DD6, 0x0000);

	/* clear PCM capture1 read pointer */
	_mhal_alsa_write_reg(0x2DD4, 0x0000);

	if (g_pcm_capture1_base_va != 0) {
		if (pcm_capture->buffer.addr) {
			iounmap((void *)pcm_capture->buffer.addr);
			pcm_capture->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Capture1 buffer address should not be 0 !\n");
		}

		g_pcm_capture1_base_va = 0;
	}

	pcm_capture->status = E_STOP;
	pcm_capture->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_PCM_CAPTURE1);
	}

	return 0;
}

/* Start PCM Capture1 */
static int _mhal_alsa_pcm_capture1_start(void)
{
	//MAD_PRINT(KERN_INFO "Start MStar PCM Capture1 engine\n");

	return 0;
}

/* Stop PCM Capture1 */
static int _mhal_alsa_pcm_capture1_stop(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Capture1 engine\n");

	pcm_capture->status = E_STOP;

	return 0;
}

/* Resume PCM Capture1 */
static int _mhal_alsa_pcm_capture1_resume(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Capture1 engine\n");

	pcm_capture->status = E_RESUME;

	return 0;
}

/* Suspend PCM Capture1 */
static int _mhal_alsa_pcm_capture1_suspend(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Capture1 engine\n");

	pcm_capture->status = E_SUSPEND;

	return 0;
}

/* Read PCM from PCM Capture1 */
static unsigned int _mhal_alsa_pcm_capture1_read(void *buffer, unsigned int bytes)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	unsigned int rest_size_to_buffer_end = (pcm_capture->buffer.addr + _MAD_PCM_CAPTURE_BUF_SIZE) - pcm_capture->buffer.r_ptr;
	unsigned int r_ptr_offset = 0;
	unsigned int read_size = 0;
	unsigned char tmp_value = 0;
	//MAD_PRINT(KERN_INFO "Read PCM from PCM Capture1 engine\n");

	read_size = (rest_size_to_buffer_end > bytes) ? bytes : rest_size_to_buffer_end;

	memcpy(buffer, pcm_capture->buffer.r_ptr, read_size);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *((unsigned char *)buffer);

	Chip_Flush_Memory();

	pcm_capture->buffer.r_ptr += read_size;
	if (pcm_capture->buffer.r_ptr == (pcm_capture->buffer.addr + _MAD_PCM_CAPTURE_BUF_SIZE))
		pcm_capture->buffer.r_ptr = pcm_capture->buffer.addr;

	r_ptr_offset = pcm_capture->buffer.r_ptr - pcm_capture->buffer.addr;
	_mhal_alsa_write_reg(0x2DD4, (unsigned short)(r_ptr_offset / _MAD_BYTES_IN_LINE));

	pcm_capture->status = E_START;

	return read_size;
}

/* Get infromation from PCM Capture1 */
static int _mhal_alsa_pcm_capture1_get(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Capture1 engine\n");

	switch(cmd) {
		case E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_capture1_get_new_avail_bytes();
			break;
		}

		case E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_capture1_get_total_avail_bytes();
			break;
		}

		case E_PCM_CAPTURE_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_CAPTURE_GET_IS_DMIC:
		{
			*param = 0;
			break;
		}

		case E_PCM_CAPTURE_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_PCM_CAPTURE1))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to PCM Capture1 */
static int _mhal_alsa_pcm_capture1_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Capture1 engine\n");

	switch(cmd) {
		case E_PCM_CAPTURE_SET_BUFFER_SIZE:
		{
			if (*param > _MAD_PCM_CAPTURE_BUF_SIZE) {
				*param = _MAD_PCM_CAPTURE_BUF_SIZE;
				MAD_PRINT(KERN_INFO "Target buffer is too large, reset to %u\n", *param);
			}

			if ((*param % _MAD_BYTES_IN_LINE)) {
				*param = (*param / _MAD_BYTES_IN_LINE) * _MAD_BYTES_IN_LINE;
				MAD_PRINT(KERN_INFO "Target buffer is not aligned, reset to %u\n", *param);
			}

			_mhal_alsa_pcm_capture1_set_buffer_size(*param);
			break;
		}

		case E_PCM_CAPTURE_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_PCM_CAPTURE1);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get PCM Capture1's new PCM available bytes */
static unsigned int _mhal_alsa_pcm_capture1_get_new_avail_bytes(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	unsigned char *new_w_ptr = NULL;
	unsigned int new_w_ptr_offset = 0;
	int new_avail_bytes = 0;

	new_w_ptr_offset = _mhal_alsa_read_reg(0x2DF0) * _MAD_BYTES_IN_LINE;
	new_w_ptr = pcm_capture->buffer.addr + new_w_ptr_offset;

	new_avail_bytes = new_w_ptr - pcm_capture->buffer.w_ptr;
	if (new_avail_bytes < 0)
		new_avail_bytes += _MAD_PCM_CAPTURE_BUF_SIZE;

	pcm_capture->buffer.w_ptr = new_w_ptr;

	return new_avail_bytes;
}

/* Get PCM Capture1's total PCM available bytes */
static unsigned int _mhal_alsa_pcm_capture1_get_total_avail_bytes(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	unsigned int r_ptr_offset = 0;
	unsigned int w_ptr_offset = 0;
	int avail_bytes = 0;
	int loop = 0;
	int loop_timeout = 100;

	r_ptr_offset = pcm_capture->buffer.r_ptr - pcm_capture->buffer.addr;
	w_ptr_offset = _mhal_alsa_read_reg(0x2DF0) * _MAD_BYTES_IN_LINE;

	avail_bytes = w_ptr_offset - r_ptr_offset;
	if (avail_bytes < 0)
		avail_bytes += _MAD_PCM_CAPTURE_BUF_SIZE;

	if (avail_bytes >= (pcm_capture->buffer.size - _MAD_PCM_CAPTURE_BUF_UNIT)) {
		MAD_PRINT(KERN_INFO "***** Audio PCM Capture1 Buffer is overrun !! ***** \n");

		/* clear PCM capture buffer size */
		_mhal_alsa_write_reg(0x2DD6, 0x0000);

		/* clear PCM capture read pointer */
		_mhal_alsa_write_reg(0x2DD4, 0x0000);

		/* check if PCM capture receives reset command */
		while(_mhal_alsa_read_reg(0x2DF0) != 0) {
			mdelay(1);

			if ((++loop) >= loop_timeout)
				break;
		}

		/* reset PCM capture write pointer */
		w_ptr_offset = _mhal_alsa_read_reg(0x2DF0) * _MAD_BYTES_IN_LINE;
		pcm_capture->buffer.w_ptr = pcm_capture->buffer.addr + w_ptr_offset;

		/* reset PCM capture read pointer */
		pcm_capture->buffer.r_ptr = pcm_capture->buffer.w_ptr;
		_mhal_alsa_write_reg(0x2DD4, (unsigned short)(w_ptr_offset / _MAD_BYTES_IN_LINE));

		/* reset PCM capture buffer size */
		_mhal_alsa_write_reg(0x2DD6, (unsigned short)(pcm_capture->buffer.size / _MAD_BYTES_IN_LINE));

		return 0;
	}

	return avail_bytes;
}

/* Set PCM Capture1's PCM buffer size */
static int _mhal_alsa_pcm_capture1_set_buffer_size(unsigned int buffer_size)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture1;
	MAD_PRINT(KERN_INFO "Target buffer size is %u\n", buffer_size);

	pcm_capture->buffer.size = buffer_size;
	_mhal_alsa_write_reg(0x2DD6, (unsigned short)(buffer_size / _MAD_BYTES_IN_LINE));

	return 0;
}

/* Initiate PCM Capture2 */
static int _mhal_alsa_pcm_capture2_init(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	ptrdiff_t audio_pcm_capture_base_pa = 0;
	ptrdiff_t audio_pcm_capture_base_ba = 0;
	ptrdiff_t audio_pcm_capture_base_va = 0;
	unsigned int audio_pcm_capture_offset = _MAD_PCM_CAPTURE2_BASE_OFFSET;
	unsigned int w_ptr_offset = 0;
	unsigned char tmp_value = 0;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Capture2 engine\n");

	if (PCM_SHM_INFO.capture2_offset != 0) {
		audio_pcm_capture_offset = PCM_SHM_INFO.capture2_offset;
	}

	if ((pcm_capture->initialized_status != MAD_TRUE) || (pcm_capture->status != E_RESUME)) {
		audio_pcm_capture_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_capture_offset;
		audio_pcm_capture_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_capture_base_pa);

		if ((audio_pcm_capture_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM capture bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		pcm_capture->buffer.size = _MAD_PCM_CAPTURE_BUF_SIZE;
		if (g_pcm_capture2_base_va == 0) {
			g_pcm_capture2_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_capture_base_ba, pcm_capture->buffer.size);
			if (g_pcm_capture2_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Capture2 Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_capture_base_va = g_pcm_capture2_base_va;

		pcm_capture->str_mode_info.physical_addr = audio_pcm_capture_base_pa;
		pcm_capture->str_mode_info.bus_addr = audio_pcm_capture_base_ba;
		pcm_capture->str_mode_info.virtual_addr = audio_pcm_capture_base_va;

		pcm_capture->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_capture_base_pa = pcm_capture->str_mode_info.physical_addr;
		audio_pcm_capture_base_ba = pcm_capture->str_mode_info.bus_addr;
		audio_pcm_capture_base_va = pcm_capture->str_mode_info.virtual_addr;
	}

	/* init PCM capture2 buffer address */
	pcm_capture->buffer.addr = (unsigned char *)audio_pcm_capture_base_va;
	//MAD_PRINT(KERN_INFO "PCM Capture2 buffer start address = 0x%08X\n", pcm_capture->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Capture2 buffer end address = 0x%08X\n", (pcm_capture->buffer.addr + pcm_capture->buffer.size));

	/* clear all PCM capture2 buffer */
	memset((void *)pcm_capture->buffer.addr, 0x00, _MAD_PCM_CAPTURE_BUF_SIZE);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *(pcm_capture->buffer.addr);

	Chip_Flush_Memory();

	/* reset PCM capture2 write pointer */
	w_ptr_offset = _mhal_alsa_read_reg(0x2DF4) * _MAD_BYTES_IN_LINE;
	pcm_capture->buffer.w_ptr = pcm_capture->buffer.addr + w_ptr_offset;

	/* reset PCM capture2 read pointer */
	pcm_capture->buffer.r_ptr = pcm_capture->buffer.w_ptr;
	_mhal_alsa_write_reg(0x2D38, (unsigned short)(w_ptr_offset / _MAD_BYTES_IN_LINE));

	/* reset PCM capture2 buffer size */
	_mhal_alsa_write_reg(0x2D3A, (unsigned short)(pcm_capture->buffer.size / _MAD_BYTES_IN_LINE));

	return 0;
}

/* Exit PCM Capture2 */
static int _mhal_alsa_pcm_capture2_exit(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Capture2 engine\n");

	/* clear PCM capture2 buffer size */
	_mhal_alsa_write_reg(0x2D3A, 0x0000);

	/* clear PCM capture2 read pointer */
	_mhal_alsa_write_reg(0x2D38, 0x0000);

	if (g_pcm_capture2_base_va != 0) {
		if (pcm_capture->buffer.addr) {
			iounmap((void *)pcm_capture->buffer.addr);
			pcm_capture->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Capture2 buffer address should not be 0 !\n");
		}

		g_pcm_capture2_base_va = 0;
	}

	pcm_capture->status = E_STOP;
	pcm_capture->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_PCM_CAPTURE2);
	}

	return 0;
}

/* Start PCM Capture2 */
static int _mhal_alsa_pcm_capture2_start(void)
{
	//MAD_PRINT(KERN_INFO "Start MStar PCM Capture2 engine\n");

	return 0;
}

/* Stop PCM Capture2 */
static int _mhal_alsa_pcm_capture2_stop(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Capture2 engine\n");

	pcm_capture->status = E_STOP;

	return 0;
}

/* Resume PCM Capture2 */
static int _mhal_alsa_pcm_capture2_resume(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Capture2 engine\n");

	pcm_capture->status = E_RESUME;

	return 0;
}

/* Suspend PCM Capture2 */
static int _mhal_alsa_pcm_capture2_suspend(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Capture2 engine\n");

	pcm_capture->status = E_SUSPEND;

	return 0;
}

/* Read PCM from PCM Capture2 */
static unsigned int _mhal_alsa_pcm_capture2_read(void *buffer, unsigned int bytes)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	unsigned int rest_size_to_buffer_end = (pcm_capture->buffer.addr + _MAD_PCM_CAPTURE_BUF_SIZE) - pcm_capture->buffer.r_ptr;
	unsigned int r_ptr_offset = 0;
	unsigned int read_size = 0;
	unsigned char tmp_value = 0;
	//MAD_PRINT(KERN_INFO "Read PCM from PCM Capture2 engine\n");

	read_size = (rest_size_to_buffer_end > bytes) ? bytes : rest_size_to_buffer_end;

	memcpy(buffer, pcm_capture->buffer.r_ptr, read_size);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *((unsigned char *)buffer);

	Chip_Flush_Memory();

	pcm_capture->buffer.r_ptr += read_size;
	if (pcm_capture->buffer.r_ptr == (pcm_capture->buffer.addr + _MAD_PCM_CAPTURE_BUF_SIZE))
		pcm_capture->buffer.r_ptr = pcm_capture->buffer.addr;

	r_ptr_offset = pcm_capture->buffer.r_ptr - pcm_capture->buffer.addr;
	_mhal_alsa_write_reg(0x2D38, (unsigned short)(r_ptr_offset / _MAD_BYTES_IN_LINE));

	pcm_capture->status = E_START;

	return read_size;
}

/* Get infromation from PCM Capture2 */
static int _mhal_alsa_pcm_capture2_get(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Capture2 engine\n");

	switch(cmd) {
		case E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_capture2_get_new_avail_bytes();
			break;
		}

		case E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_capture2_get_total_avail_bytes();
			break;
		}

		case E_PCM_CAPTURE_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_CAPTURE_GET_IS_DMIC:
		{
			*param = 0;
			break;
		}

		case E_PCM_CAPTURE_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_PCM_CAPTURE2))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to PCM Capture2 */
static int _mhal_alsa_pcm_capture2_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Capture2 engine\n");

	switch(cmd) {
		case E_PCM_CAPTURE_SET_BUFFER_SIZE:
		{
			if (*param > _MAD_PCM_CAPTURE_BUF_SIZE) {
				*param = _MAD_PCM_CAPTURE_BUF_SIZE;
				MAD_PRINT(KERN_INFO "Target buffer is too large, reset to %u\n", *param);
			}

			if ((*param % _MAD_BYTES_IN_LINE)) {
				*param = (*param / _MAD_BYTES_IN_LINE) * _MAD_BYTES_IN_LINE;
				MAD_PRINT(KERN_INFO "Target buffer is not aligned, reset to %u\n", *param);
			}

			_mhal_alsa_pcm_capture2_set_buffer_size(*param);
			break;
		}

		case E_PCM_CAPTURE_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_PCM_CAPTURE2);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get PCM Capture2's new PCM available bytes */
static unsigned int _mhal_alsa_pcm_capture2_get_new_avail_bytes(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	unsigned char *new_w_ptr = NULL;
	unsigned int new_w_ptr_offset = 0;
	int new_avail_bytes = 0;

	new_w_ptr_offset = _mhal_alsa_read_reg(0x2DF4) * _MAD_BYTES_IN_LINE;
	new_w_ptr = pcm_capture->buffer.addr + new_w_ptr_offset;

	new_avail_bytes = new_w_ptr - pcm_capture->buffer.w_ptr;
	if (new_avail_bytes < 0)
		new_avail_bytes += _MAD_PCM_CAPTURE_BUF_SIZE;

	pcm_capture->buffer.w_ptr = new_w_ptr;

	return new_avail_bytes;
}

/* Get PCM Capture2's total PCM available bytes */
static unsigned int _mhal_alsa_pcm_capture2_get_total_avail_bytes(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	unsigned int r_ptr_offset = 0;
	unsigned int w_ptr_offset = 0;
	int avail_bytes = 0;
	int loop = 0;
	int loop_timeout = 100;

	r_ptr_offset = pcm_capture->buffer.r_ptr - pcm_capture->buffer.addr;
	w_ptr_offset = _mhal_alsa_read_reg(0x2DF4) * _MAD_BYTES_IN_LINE;

	avail_bytes = w_ptr_offset - r_ptr_offset;
	if (avail_bytes < 0)
		avail_bytes += _MAD_PCM_CAPTURE_BUF_SIZE;

	if (avail_bytes >= (pcm_capture->buffer.size - _MAD_PCM_CAPTURE_BUF_UNIT)) {
		MAD_PRINT(KERN_INFO "***** Audio PCM Capture2 Buffer is overrun !! ***** \n");

		/* clear PCM capture buffer size */
		_mhal_alsa_write_reg(0x2D3A, 0x0000);

		/* clear PCM capture read pointer */
		_mhal_alsa_write_reg(0x2D38, 0x0000);

		/* check if PCM capture receives reset command */
		while(_mhal_alsa_read_reg(0x2DF4) != 0) {
			mdelay(1);

			if ((++loop) >= loop_timeout)
				break;
		}

		/* reset PCM capture write pointer */
		w_ptr_offset = _mhal_alsa_read_reg(0x2DF4) * _MAD_BYTES_IN_LINE;
		pcm_capture->buffer.w_ptr = pcm_capture->buffer.addr + w_ptr_offset;

		/* reset PCM capture read pointer */
		pcm_capture->buffer.r_ptr = pcm_capture->buffer.w_ptr;
		_mhal_alsa_write_reg(0x2D38, (unsigned short)(w_ptr_offset / _MAD_BYTES_IN_LINE));

		/* reset PCM capture buffer size */
		_mhal_alsa_write_reg(0x2D3A, (unsigned short)(pcm_capture->buffer.size / _MAD_BYTES_IN_LINE));

		return 0;
	}

	return avail_bytes;
}

/* Set PCM Capture2's PCM buffer size */
static int _mhal_alsa_pcm_capture2_set_buffer_size(unsigned int buffer_size)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture2;
	MAD_PRINT(KERN_INFO "Target buffer size is %u\n", buffer_size);

	pcm_capture->buffer.size = buffer_size;
	_mhal_alsa_write_reg(0x2D3A, (unsigned short)(buffer_size / _MAD_BYTES_IN_LINE));

	return 0;
}

/* Initiate PCM Capture3 */
static int _mhal_alsa_pcm_capture3_init(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	ptrdiff_t audio_pcm_capture_base_pa = 0;
	ptrdiff_t audio_pcm_capture_base_ba = 0;
	ptrdiff_t audio_pcm_capture_base_va = 0;
	unsigned int audio_pcm_capture_offset = _MAD_PCM_CAPTURE3_BASE_OFFSET;
	unsigned int w_ptr_offset = 0;
	unsigned char tmp_value = 0;
	MAD_PRINT(KERN_INFO "Initiate MStar PCM Capture3 engine\n");

	if (PCM_SHM_INFO.capture3_offset != 0) {
		audio_pcm_capture_offset = PCM_SHM_INFO.capture3_offset;
	}

	if ((pcm_capture->initialized_status != MAD_TRUE) || (pcm_capture->status != E_RESUME)) {
		audio_pcm_capture_base_pa = (ptrdiff_t)_mhal_alsa_get_common_buffer_pa() + audio_pcm_capture_offset;
		audio_pcm_capture_base_ba = _mhal_alsa_convert_pa_to_ba(audio_pcm_capture_base_pa);

		if ((audio_pcm_capture_base_ba % 0x1000)) {
			MAD_PRINT(KERN_ERR "Error! Invalid MStar PCM capture bus address, it should be aligned by 4 KB!\n");
			return -EFAULT;
		}

		/* convert Bus Address to Virtual Address */
		pcm_capture->buffer.size = _MAD_PCM_CAPTURE_BUF_SIZE;
		if (g_pcm_capture3_base_va == 0) {
			g_pcm_capture3_base_va = (ptrdiff_t)ioremap_wc(audio_pcm_capture_base_ba, pcm_capture->buffer.size);
			if (g_pcm_capture3_base_va == 0) {
				MAD_PRINT(KERN_ERR "Error! fail to convert PCM Capture3 Buffer Bus Address to Virtual Address\n");
				return -ENOMEM;
			}
		}
		audio_pcm_capture_base_va = g_pcm_capture3_base_va;

		pcm_capture->str_mode_info.physical_addr = audio_pcm_capture_base_pa;
		pcm_capture->str_mode_info.bus_addr = audio_pcm_capture_base_ba;
		pcm_capture->str_mode_info.virtual_addr = audio_pcm_capture_base_va;

		pcm_capture->initialized_status = MAD_TRUE;
	}
	else {
		audio_pcm_capture_base_pa = pcm_capture->str_mode_info.physical_addr;
		audio_pcm_capture_base_ba = pcm_capture->str_mode_info.bus_addr;
		audio_pcm_capture_base_va = pcm_capture->str_mode_info.virtual_addr;
	}

	/* init PCM capture3 buffer address */
	pcm_capture->buffer.addr = (unsigned char *)audio_pcm_capture_base_va;
	//MAD_PRINT(KERN_INFO "PCM Capture3 buffer start address = 0x%08X\n", pcm_capture->buffer.addr);
	//MAD_PRINT(KERN_INFO "PCM Capture3 buffer end address = 0x%08X\n", (pcm_capture->buffer.addr + pcm_capture->buffer.size));

	/* clear all PCM capture3 buffer */
	memset((void *)pcm_capture->buffer.addr, 0x00, _MAD_PCM_CAPTURE_BUF_SIZE);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *(pcm_capture->buffer.addr);

	Chip_Flush_Memory();

	/* reset PCM capture3 write pointer */
	w_ptr_offset = _mhal_alsa_read_reg(0x2D7E) * _MAD_BYTES_IN_LINE;
	pcm_capture->buffer.w_ptr = pcm_capture->buffer.addr + w_ptr_offset;

	/* reset PCM capture3 read pointer */
	pcm_capture->buffer.r_ptr = pcm_capture->buffer.w_ptr;
	_mhal_alsa_write_reg(0x2D94, (unsigned short)(w_ptr_offset / _MAD_BYTES_IN_LINE));

	/* reset PCM capture3 buffer size */
	_mhal_alsa_write_reg(0x2D96, (unsigned short)(pcm_capture->buffer.size / _MAD_BYTES_IN_LINE));

	return 0;
}

/* Exit PCM Capture3 */
static int _mhal_alsa_pcm_capture3_exit(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	MAD_PRINT(KERN_INFO "Exit MStar PCM Capture3 engine\n");

	/* clear PCM capture3 buffer size */
	_mhal_alsa_write_reg(0x2D96, 0x0000);

	/* clear PCM capture3 read pointer */
	_mhal_alsa_write_reg(0x2D94, 0x0000);

	if (g_pcm_capture3_base_va != 0) {
		if (pcm_capture->buffer.addr) {
			iounmap((void *)pcm_capture->buffer.addr);
			pcm_capture->buffer.addr = 0;
		}
		else {
			MAD_PRINT(KERN_ERR "Error! MStar PCM Capture3 buffer address should not be 0 !\n");
		}

		g_pcm_capture3_base_va = 0;
	}

	pcm_capture->status = E_STOP;
	pcm_capture->initialized_status = MAD_FALSE;

	if (g_shm_info) {
		g_shm_info->dma_use_status &= ~(0x1 << E_PCM_CAPTURE3);
	}

	return 0;
}

/* Start PCM Capture3 */
static int _mhal_alsa_pcm_capture3_start(void)
{
	//MAD_PRINT(KERN_INFO "Start MStar PCM Capture3 engine\n");

	return 0;
}

/* Stop PCM Capture3 */
static int _mhal_alsa_pcm_capture3_stop(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	//MAD_PRINT(KERN_INFO "Stop MStar PCM Capture3 engine\n");

	pcm_capture->status = E_STOP;

	return 0;
}

/* Resume PCM Capture3 */
static int _mhal_alsa_pcm_capture3_resume(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	//MAD_PRINT(KERN_INFO "Resume MStar PCM Capture3 engine\n");

	pcm_capture->status = E_RESUME;

	return 0;
}

/* Suspend PCM Capture3 */
static int _mhal_alsa_pcm_capture3_suspend(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	//MAD_PRINT(KERN_INFO "Suspend MStar PCM Capture3 engine\n");

	pcm_capture->status = E_SUSPEND;

	return 0;
}

/* Read PCM from PCM Capture3 */
static unsigned int _mhal_alsa_pcm_capture3_read(void *buffer, unsigned int bytes)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	unsigned int rest_size_to_buffer_end = (pcm_capture->buffer.addr + _MAD_PCM_CAPTURE_BUF_SIZE) - pcm_capture->buffer.r_ptr;
	unsigned int r_ptr_offset = 0;
	unsigned int read_size = 0;
	unsigned char tmp_value = 0;
	//MAD_PRINT(KERN_INFO "Read PCM from PCM Capture3 engine\n");

	read_size = (rest_size_to_buffer_end > bytes) ? bytes : rest_size_to_buffer_end;

	memcpy(buffer, pcm_capture->buffer.r_ptr, read_size);

	/*
	 * it's a patch here!
	 * We have a SW patch by using ioremap_wc() instead of ioremap_noncache() to fix CPU hang issue,
	 * in order to ensure MIU data can be updated since there is some queue in CPU side,
	 * we just read any byte of them then all MIU data can be updated.
	 */
	tmp_value = *((unsigned char *)buffer);

	Chip_Flush_Memory();

	pcm_capture->buffer.r_ptr += read_size;
	if (pcm_capture->buffer.r_ptr == (pcm_capture->buffer.addr + _MAD_PCM_CAPTURE_BUF_SIZE))
		pcm_capture->buffer.r_ptr = pcm_capture->buffer.addr;

	r_ptr_offset = pcm_capture->buffer.r_ptr - pcm_capture->buffer.addr;
	_mhal_alsa_write_reg(0x2D94, (unsigned short)(r_ptr_offset / _MAD_BYTES_IN_LINE));

	pcm_capture->status = E_START;

	return read_size;
}

/* Get infromation from PCM Capture3 */
static int _mhal_alsa_pcm_capture3_get(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Get parameter from PCM Capture3 engine\n");

	switch(cmd) {
		case E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_capture3_get_new_avail_bytes();
			break;
		}

		case E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES:
		{
			*param = _mhal_alsa_pcm_capture3_get_total_avail_bytes();
			break;
		}

		case E_PCM_CAPTURE_GET_DEVICE_STATUS:
		{
			*param = _mhal_alsa_get_device_status();
			break;
		}

		case E_PCM_CAPTURE_GET_IS_DMIC:
		{
			*param = 0;
			break;
		}

		case E_PCM_CAPTURE_GET_DEVICE_USED:
		{
			if (g_shm_info == NULL) {
				err = _mhal_alsa_get_share_mem();
				if (err < 0)
					break;
			}

			if ((g_shm_info != NULL) && (g_shm_info->dma_use_status & (0x1 << E_PCM_CAPTURE3))) {
				*param = 1;
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid GET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Set information to PCM Capture3 */
static int _mhal_alsa_pcm_capture3_set(int cmd, unsigned int *param)
{
	int err = 0;
	//MAD_PRINT(KERN_INFO "Set parameter to PCM Capture3 engine\n");

	switch(cmd) {
		case E_PCM_CAPTURE_SET_BUFFER_SIZE:
		{
			if (*param > _MAD_PCM_CAPTURE_BUF_SIZE) {
				*param = _MAD_PCM_CAPTURE_BUF_SIZE;
				MAD_PRINT(KERN_INFO "Target buffer is too large, reset to %u\n", *param);
			}

			if ((*param % _MAD_BYTES_IN_LINE)) {
				*param = (*param / _MAD_BYTES_IN_LINE) * _MAD_BYTES_IN_LINE;
				MAD_PRINT(KERN_INFO "Target buffer is not aligned, reset to %u\n", *param);
			}

			_mhal_alsa_pcm_capture3_set_buffer_size(*param);
			break;
		}

		case E_PCM_CAPTURE_SET_DEVICE_USED:
		{
			if (g_shm_info) {
				g_shm_info->dma_use_status |= (0x1 << E_PCM_CAPTURE3);
			}
			break;
		}

		default:
		{
			MAD_PRINT(KERN_INFO "Invalid SET command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

/* Get PCM Capture3's new PCM available bytes */
static unsigned int _mhal_alsa_pcm_capture3_get_new_avail_bytes(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	unsigned char *new_w_ptr = NULL;
	unsigned int new_w_ptr_offset = 0;
	int new_avail_bytes = 0;

	new_w_ptr_offset = _mhal_alsa_read_reg(0x2D7E) * _MAD_BYTES_IN_LINE;
	new_w_ptr = pcm_capture->buffer.addr + new_w_ptr_offset;

	new_avail_bytes = new_w_ptr - pcm_capture->buffer.w_ptr;
	if (new_avail_bytes < 0)
		new_avail_bytes += _MAD_PCM_CAPTURE_BUF_SIZE;

	pcm_capture->buffer.w_ptr = new_w_ptr;

	return new_avail_bytes;
}

/* Get PCM Capture3's total PCM available bytes */
static unsigned int _mhal_alsa_pcm_capture3_get_total_avail_bytes(void)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	unsigned int r_ptr_offset = 0;
	unsigned int w_ptr_offset = 0;
	int avail_bytes = 0;
	int loop = 0;
	int loop_timeout = 100;

	r_ptr_offset = pcm_capture->buffer.r_ptr - pcm_capture->buffer.addr;
	w_ptr_offset = _mhal_alsa_read_reg(0x2D7E) * _MAD_BYTES_IN_LINE;

	avail_bytes = w_ptr_offset - r_ptr_offset;
	if (avail_bytes < 0)
		avail_bytes += _MAD_PCM_CAPTURE_BUF_SIZE;

	if (avail_bytes >= (pcm_capture->buffer.size - _MAD_PCM_CAPTURE_BUF_UNIT)) {
		MAD_PRINT(KERN_INFO "***** Audio PCM Capture3 Buffer is overrun !! ***** \n");

		/* clear PCM capture buffer size */
		_mhal_alsa_write_reg(0x2D96, 0x0000);

		/* clear PCM capture read pointer */
		_mhal_alsa_write_reg(0x2D94, 0x0000);

		/* check if PCM capture receives reset command */
		while(_mhal_alsa_read_reg(0x2D7E) != 0) {
			mdelay(1);

			if ((++loop) >= loop_timeout)
				break;
		}

		/* reset PCM capture write pointer */
		w_ptr_offset = _mhal_alsa_read_reg(0x2D7E) * _MAD_BYTES_IN_LINE;
		pcm_capture->buffer.w_ptr = pcm_capture->buffer.addr + w_ptr_offset;

		/* reset PCM capture read pointer */
		pcm_capture->buffer.r_ptr = pcm_capture->buffer.w_ptr;
		_mhal_alsa_write_reg(0x2D94, (unsigned short)(w_ptr_offset / _MAD_BYTES_IN_LINE));

		/* reset PCM capture buffer size */
		_mhal_alsa_write_reg(0x2D96, (unsigned short)(pcm_capture->buffer.size / _MAD_BYTES_IN_LINE));

		return 0;
	}

	return avail_bytes;
}

/* Set PCM Capture3's PCM buffer size */
static int _mhal_alsa_pcm_capture3_set_buffer_size(unsigned int buffer_size)
{
	struct MStar_PCM_Capture_Struct *pcm_capture = &g_pcm_capture3;
	MAD_PRINT(KERN_INFO "Target buffer size is %u\n", buffer_size);

	pcm_capture->buffer.size = buffer_size;
	_mhal_alsa_write_reg(0x2D96, (unsigned short)(buffer_size / _MAD_BYTES_IN_LINE));

	return 0;
}

static int __init _mhal_alsa_init(void)
{
	int err = 0;

	MAD_PRINT(KERN_INFO "Initiate MStar ALSA core driver\n");

	memset(&MStar_MAD, 0x00, sizeof(struct MStar_MAD_Info));
	memset(&PCM_SHM_INFO, 0x00, sizeof(struct MStar_COMMON_PCM_SHM_Struct));

	snprintf(MStar_MAD.version, sizeof(MStar_MAD.version), "%d.%d.%d", _MAD_ALSA_HAL_VERSION_MAJOR, _MAD_ALSA_HAL_VERSION_MINOR, _MAD_ALSA_HAL_VERSION_REVISION);

	/* Hook Playback Operators */
	MStar_MAD.playback_pcm_ops[0] = &MStar_DMA_Reader_Ops;
	MStar_MAD.playback_pcm_ops[1] = &MStar_DMA_Reader2_Ops;
	MStar_MAD.playback_pcm_ops[2] = &MStar_PCM_Playback3_Ops;//sw1
	MStar_MAD.playback_pcm_ops[3] = &MStar_PCM_Playback4_Ops;//sw3
	MStar_MAD.playback_pcm_ops[7] = &MStar_PCM_Playback7_Ops;//sw2

	/* Hook Capture Operators */
	MStar_MAD.capture_pcm_ops[0] = &MStar_PCM_Capture1_Ops;
	MStar_MAD.capture_pcm_ops[1] = &MStar_PCM_Capture2_Ops;
	MStar_MAD.capture_pcm_ops[2] = &MStar_PCM_Capture3_Ops;

	err = _mdrv_alsa_hook_device(&MStar_MAD);
	if (err < 0) {
		MAD_PRINT(KERN_ERR "Error(%d)! fail to hook PCM operators\n", err);
		return err;
	}

	return 0;
}

static void __exit _mhal_alsa_exit(void)
{
	int err = 0;

	MAD_PRINT(KERN_INFO "Exit MStar ALSA core driver\n");

	err = _mdrv_alsa_unhook_device();
	if (err < 0) {
		MAD_PRINT(KERN_ERR "Error(%d)! fail to unhook PCM operators\n", err);
		return;
	}

	if (g_shm_info)
		g_shm_info = NULL;

	return;
}

/*
 * ============================================================================
 * Module Information
 * ============================================================================
 */
module_init(_mhal_alsa_init);
module_exit(_mhal_alsa_exit);

MODULE_AUTHOR("MStar Semiconductor, Inc.");
MODULE_DESCRIPTION("MStar ALSA Driver - HAL Layer");
MODULE_SUPPORTED_DEVICE("MAD DEVICE");
MODULE_LICENSE("Proprietary");
