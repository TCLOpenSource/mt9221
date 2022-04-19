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
#include <linux/platform_device.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 1))
#include <linux/module.h>
#endif
#include <asm/io.h>

#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/mm.h>

#include "mhal_dmic.h"
#include "mstar/mstar_chip.h"

/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
#if defined(CONFIG_ARM64)
extern ptrdiff_t mstar_pm_base;
#define _DMIC_PM_MODE_REG_BASE mstar_pm_base
#else
#define _DMIC_PM_MODE_REG_BASE 0xfd000000
#endif
#define _DMIC_NON_PM_MODE_REG_BASE 0xfd200000
#define _DMIC_PHYSICAL_MEM_BASE _DMIC_PM_MODE_REG_BASE

#define _MAD_ADDR_CONVERTOR    0x1000
#define _MAD_MAILBOX_OFFSET 0x110000
#define _MAD_DSP2_DRAM_SIZE    0x280000

#define _DMIC_BYTES_IN_LINE 16
#define _DMIC_MAILBOX_OFFSET 0x320000

#define _DMIC_SEAMLESS_BASE_OFFSET 0x0
#define _DMIC_SEAMLESS_BUF_UNIT 0 /* bytes */
#define _DMIC_SEAMLESS_BUF_SIZE 0x0

#define _DMIC_CAPTURE_BASE_OFFSET 0x81C000
#define _DMIC_CAPTURE_BUF_UNIT 32      /* bytes */
#define _DMIC_CAPTURE_BUF_SIZE 0x12000 /* 48*3 KByte */

#define _DMIC_AEC_CAPTURE_BASE_OFFSET 0x82E000
#define _DMIC_AEC_CAPTURE_BUF_UNIT 32     /* bytes */
#define _DMIC_AEC_CAPTURE_BUF_SIZE 0x9000 /* 48*3 KByte */

#define _DMIC_READ_BYTE(_reg) (*( volatile unsigned char * ) (_reg))
#define _DMIC_READ_WORD(_reg) (*( volatile unsigned short * ) (_reg))
#define _DMIC_WRITE_BYTE(_reg, _val)                                         \
    {                                                                        \
        (*(( volatile unsigned char * ) (_reg))) = ( unsigned char ) (_val); \
    }
#define _DMIC_WRITE_WORD(_reg, _val)                                           \
    {                                                                          \
        (*(( volatile unsigned short * ) (_reg))) = ( unsigned short ) (_val); \
    }
#define _DMIC_R1BYTE(u32Addr, u8mask) \
    (_DMIC_READ_BYTE(_DMIC_PHYSICAL_MEM_BASE + ((u32Addr) << 1) - (( u32Addr ) &1)) & (u8mask))
#define _DMIC_AU_AbsReadByte(u32Reg) (_DMIC_READ_BYTE(_DMIC_PHYSICAL_MEM_BASE + ((u32Reg) << 1) - (( u32Reg ) &1)))
#define _DMIC_AU_AbsRead2Byte(u32Reg) (_DMIC_READ_WORD(_DMIC_PHYSICAL_MEM_BASE + ((u32Reg) << 1)))
#define _DMIC_AU_AbsWriteByte(u32Reg, u8Val)                                                    \
    do                                                                                          \
    {                                                                                           \
        (_DMIC_WRITE_BYTE(_DMIC_PHYSICAL_MEM_BASE + ((u32Reg) << 1) - (( u32Reg ) &1), u8Val)); \
    } while (0)
#define _DMIC_AU_AbsWriteMaskByte(u32Reg, u8Mask, u8Val)                                       \
    do                                                                                         \
    {                                                                                          \
        (_DMIC_WRITE_BYTE(_DMIC_PHYSICAL_MEM_BASE + ((u32Reg) << 1) - (( u32Reg ) &1),         \
                          (_DMIC_R1BYTE((u32Reg), 0xFF) & ~(u8Mask)) | ((u8Val) & (u8Mask)))); \
    } while (0)
#define _DMIC_AU_AbsWrite2Byte(u32Reg, u16Val)                                 \
    do                                                                         \
    {                                                                          \
        (_DMIC_WRITE_WORD(_DMIC_PHYSICAL_MEM_BASE + ((u32Reg) << 1), u16Val)); \
    } while (0)

#define _DMIC_AU_DO_ALIGNMENT(value, alignment_size)         \
    do                                                       \
    {                                                        \
        (value = (value / alignment_size) * alignment_size); \
    } while (0)

/*
 * ============================================================================
 * Forward Declarations
 * ============================================================================
 */
/* Read/Write Register */

static unsigned int _mhal_get_device_status(void);
static unsigned int _mhal_get_common_buffer_pa(void);


static int _mhal_dmic_capture_init(void);
static int _mhal_dmic_capture_exit(void);
static int _mhal_dmic_capture_start(void);
static int _mhal_dmic_capture_stop(void);
static int _mhal_dmic_capture_resume(void);
static int _mhal_dmic_capture_suspend(void);
static unsigned int _mhal_dmic_capture_read(void *buffer, unsigned int bytes);
static int _mhal_dmic_capture_get(int cmd, unsigned int *param);
static int _mhal_dmic_capture_set(int cmd, unsigned int *param);
static unsigned int _mhal_dmic_capture_get_new_avail_bytes(void);
static unsigned int _mhal_dmic_capture_get_total_avail_bytes(void);
static int _mhal_dmic_capture_set_buffer_size(unsigned int buffer_size);
static int _mhal_dmic_capture_set_channel_mode(unsigned int channel_mode);
static int _mhal_dmic_set_dmic_channel_mode(unsigned int nChannel);
static int _mhal_dmic_set_vdma_channel_mode(unsigned int nChannel);
static int _mhal_dmic_set_ref_channel_mode(unsigned int nChannel);
static int _mhal_dmic_capture_set_sample_rate(unsigned int sample_rate);
static int _mhal_dmic_capture_set_bit_width(unsigned int bit_width);

/*
 * ============================================================================
 * Local Variables
 * ============================================================================
 */

typedef struct
{
    unsigned int u32Addr;  ///< Reg address
    unsigned short u8Mask; ///< Reg Mask
    unsigned char u8Value; ///< Reg Value
} DMIC_REG_TYPE;

const DMIC_REG_TYPE DmicInitTbl_0[] = {
    // Paganini Init

    // XTAIL ON
    {0x3225f0, 0x01, 0x01}, // Enable

    // REG_PAGA_PLL_DIV_ENABLE
    {0x3225e0, 0x01, 0x01}, // REG_PAGA_PLL_DIV_ENABLE
    {0x3225e1, 0x80, 0x80}, // REG_PAGA_PLL_DIV_AUTO_MODE_EN

    // CM4 CLK ON
    {0x322503, 0x01, 0x00}, // [8]reg_dsppll_pd_dft
    {0x3225f8, 0x08, 0x08}, // DFS ICG Enable
    {0x3225f9, 0x04, 0x04}, // SYS Enable
    {0x3225f9, 0x01, 0x01}, // TSV Enable
    {0x3225f8, 0x80, 0x80}, // CM4 Enable
    {0x3225f8, 0x10, 0x10}, // Glitch Free Sel
    {0x3225f7, 0x02, 0x02}, // DFS Setting Enable
    {0x3225f6, 0x3f, 0x1f}, // DFS Setting
    {0x3225f7, 0x01, 0x01}, // DFS Setting  update
    {0x3225f7, 0x01, 0x00}, // DFS Setting  update

    // IMI CLK ON
    {0x3225f4, 0x80, 0x80}, // IMI Enable
    {0x3225f4, 0x08, 0x08}, // IMI DFS Enable
    {0x3225f4, 0x10, 0x10}, // Glitch Free Sel
    {0x3225f4, 0x03, 0x01}, // IMI Clk sel
    {0x3225f3, 0x02, 0x02}, // DFS Setting Enable
    {0x3225f2, 0x3f, 0x1f}, // DFS Setting
    {0x3225f3, 0x01, 0x01}, // DFS Setting  update
    {0x3225f3, 0x01, 0x00}, // DFS Setting  update

    // MAC CLK ON
    {0x32253c, 0x0f, 0x01}, // REG_SEL_CLK_VREC_MAC
    {0x32253a, 0x03, 0x03}, // REG_ENABLE_CLK_VREC_MAC

    //VREC
    {0x322402, 0x09, 0x09}, // REG_DMA_PAUSE
    {0x322314, 0x01, 0x01}, // REG_IIR_IN_SEL
    {0x32253b, 0x0c, 0x04}, // REG_TO_VAD_SEL
    {0x3224a6, 0x01, 0x01}, // REG_BYP_HPF
    {0x3224a1, 0x0f, 0x01}, // REG_MTX_SEL1
    {0x322303, 0xc0, 0xc0}, // REG_DMIC_BCK_SEL_TO_PAD
    {0x322323, 0x80, 0x80}, // REG_DMIC_BCK_SEL_TO_PAD
    {0x32253a, 0x02, 0x02}, // REG_ENABLE_CLK_VREC_MAC
    {0x32253a, 0x01, 0x01}, // REG_ENABLE_CLK_DIG_MIC
    {0x32253c, 0x01, 0x01}, // REG_SEL_CLK_24MHZ
    {0x32253c, 0x0c, 0x00}, // REG_SEL_CLK_VREC_MAC
    {0x32240c, 0xff, 0x1f}, // [8] Timer Enable
    {0x32240d, 0xff, 0x81}, // [8] Timer Enable
    {0x322408, 0x11, 0x01}, // [5:0]  REG_MODE_ALL
    {0x322402, 0xff, 0x29}, // [1]    REG_DIG_MIC_EN
    {0x322403, 0xff, 0x00}, // [1]    REG_DIG_MIC_EN
    {0x322420, 0xff, 0x00}, // [7:4]  REG_DIG_MIC_PD
    {0x322421, 0xff, 0x00}, // [7:4]  REG_DIG_MIC_PD
    {0x322440, 0xff, 0x09}, // [11:4]  REG_CIC_PD ; [0] REG_CIC_EN   2CH
    {0x322441, 0xff, 0x0f}, // [11:4]  REG_CIC_PD ; [0] REG_CIC_EN   2CH
    {0x32240c, 0xff, 0x13}, // [3:2]  REG_EN_DEC
    {0x32240d, 0xff, 0x05}, // [10]   REG_48MHZ_MD ; [8] REG_TG_EN
    {0x322422, 0xff, 0x03}, // [0]    REG_DIG_MIC_IN_LR_SYNC_EN_CH1 ; [1]  REG_DIG_MIC_IN_DG_EN_CH1
    {0x322423, 0xff, 0x00}, // [0]    REG_DIG_MIC_IN_LR_SYNC_EN_CH1 ; [1]  REG_DIG_MIC_IN_DG_EN_CH1
    {0x322421, 0x01, 0x01}, //        REG_AUTO_CNT_CH1
    {0x322426, 0xff, 0x03}, // [0]    REG_DIG_MIC_IN_LR_SYNC_EN_CH2 ; [1]  REG_DIG_MIC_IN_DG_EN_CH2
    {0x322427, 0xff, 0x00}, // [0]    REG_DIG_MIC_IN_LR_SYNC_EN_CH2 ; [1]  REG_DIG_MIC_IN_DG_EN_CH2
    {0x322421, 0x02, 0x02}, //        REG_AUTO_CNT_CH2
    {0x322442, 0xff, 0x01}, // [0]  REG_CIC_STAGE_1LR
    {0x322444, 0xff, 0x01}, // [0]  REG_CIC_STAGE_2LR
    {0x322446, 0xff, 0x01}, // [0]  REG_CIC_STAGE_3LR
    {0x322448, 0xff, 0x01}, // [0]  REG_CIC_STAGE_4LR
    {0x322408, 0xff, 0x18}, // [5:0] REG_MODE_ALL
    {0x322440, 0x09, 0x00}, //  REG_CIC_EN
    {0x322440, 0x09, 0x01}, //  REG_CIC_EN

    // PPCS REG setting
    {0x3224a0, 0xff, 0x00}, //[ 7: 0] REG_DBG_PPCS //[    8] REG_BYP_PPCS
    {0x3224a1, 0xff, 0x00}, //[ 7: 0] REG_DBG_PPCS //[    8] REG_BYP_PPCS
    {0x3224a2, 0xff, 0x21}, //[ 3: 0] REG_MTX_SEL*
    {0x3224a3, 0xff, 0x43}, //[ 3: 0] REG_MTX_SEL*
    {0x3224a4, 0xff, 0x65}, //[ 3: 0] REG_MTX_SEL*
    {0x3224a5, 0xff, 0x87}, //[ 3: 0] REG_MTX_SEL*
    {0x3224a6, 0xff, 0x0a}, //[ 3] REG_DITHER_PP_EN   [1] REG_EN_HPF_2STAGE  [0] REG_BYP_HPF
    {0x3224a7, 0xff, 0x73}, //[15:12] REG_NUM_CH, HPF 8channel [11: 8] REG_HPF_N (1~12)
    {0x3224a8, 0xff, 0x00}, //[15: 0] REG_DPGA_1
    {0x3224a9, 0xff, 0x00}, //[15: 0] REG_DPGA_1
    {0x3224aa, 0xff, 0x00}, //[15: 0] REG_DPGA_2
    {0x3224ab, 0xff, 0x00}, //[15: 0] REG_DPGA_2
    {0x3224ac, 0xff, 0x00}, //[15: 0] REG_DPGA_3
    {0x3224ad, 0xff, 0x00}, //[15: 0] REG_DPGA_3
    {0x3224ae, 0xff, 0x06}, //[15: 0] REG_DPGA_LR_CTRL0 [0]:DPGA_EN
    {0x3224af, 0xff, 0x07}, //[15: 0] REG_DPGA_LR_CTRL0 [0]:DPGA_EN
    {0x3224b0, 0xff, 0x00}, //[15: 0] REG_DPGA_LR_CTRL1
    {0x3224b1, 0xff, 0x00}, //[15: 0] REG_DPGA_LR_CTRL1
    {0x3224b2, 0xff, 0x00}, //[15: 0] REG_DPGA_LR_CTRL2
    {0x3224b3, 0xff, 0x00}, //[15: 0] REG_DPGA_LR_CTRL2

    //dma setting
    {0x32246a, 0xff, 0x00}, //overrun -th
    {0x32246b, 0xff, 0x01},
    {0x322460, 0xff, 0x00}, //reset
    {0x322461, 0xff, 0x80},
    {0x322460, 0xff, 0x09}, //enable, [9]=1 for 24bit
    {0x322461, 0xff, 0x00},

    //aec_dma setting
    {0x32256a, 0xff, 0x00}, //overrun -th
    {0x32256b, 0xff, 0x01},
    {0x322560, 0xff, 0x00}, //reset
    {0x322561, 0xff, 0x80},
    {0x322560, 0xff, 0x09}, //enable, [9]=1 for 24bit
    {0x322561, 0xff, 0x00},

    //i2s tdm rx setting
    {0x322530, 0x01, 0x01}, //REG_ENABLE_CLK_SRC_A1_256FSi
    {0x322530, 0x10, 0x10}, //REG_ENABLE_CLK_CODEC_I2S_RX_BCK
    {0x322530, 0x40, 0x40}, //REG_ENABLE_CLK_NF_SYNTH_REF
    {0x3225ec, 0x02, 0x02}, //REG_CODEC_RX_EN_TIME_GEN
    {0x322561, 0x80, 0x80}, //REG_ENABLE_CODEC_I2S_RX_BCK_GEN
    //PDM On setting

    {0xffffff, 0x00, 0x00}, // end of table


};

static struct MStar_DMIC_Info hal_dmic_info;

/* MStar Voice VDMA with AEC  */
static struct MStar_PCM_DMIC_Capture_Struct g_dmic_aec_capture = {
	.buffer = {
		.addr = NULL,
		.size = _DMIC_CAPTURE_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.aec_buffer = {
		.addr = NULL,
		.size = _DMIC_AEC_CAPTURE_BUF_SIZE,
		.w_ptr = NULL,
		.r_ptr = NULL,
	},
	.str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.aec_str_mode_info = {
		.physical_addr = 0,
		.bus_addr = 0,
		.virtual_addr = 0,
	},
	.initialized_status = DMIC_FALSE,
	.channel_mode = 0,
	.sample_rate = 0,
	.bit_width = 0,
	.vdma_channel = -1,
	.aec_channel = -1,
};

static struct MStar_DMIC_Ops MStar_DMIC_AEC_Capture_Ops = {
    .open = _mhal_dmic_capture_init,
    .close = _mhal_dmic_capture_exit,
    .start = _mhal_dmic_capture_start,
    .stop = _mhal_dmic_capture_stop,
    .resume = _mhal_dmic_capture_resume,
    .suspend = _mhal_dmic_capture_suspend,
    .read = _mhal_dmic_capture_read,
    .write = NULL,
    .get = _mhal_dmic_capture_get,
    .set = _mhal_dmic_capture_set,
};

// static ptrdiff_t g_dmic_seamless_capture_base_va = 0;
static ptrdiff_t g_dmic_vdma_capture_base_va = 0;
static ptrdiff_t g_dmic_aec_capture_base_va = 0;

/*
 * ============================================================================
 * Function Implementations
 * ============================================================================
 */
static unsigned short _mhal_read_reg(unsigned int u32RegAddr)
{
    return (_DMIC_AU_AbsRead2Byte(u32RegAddr + _DMIC_MAILBOX_OFFSET));
}

static void _mhal_write_reg(unsigned int u32RegAddr, unsigned short u16Val)
{
    _DMIC_AU_AbsWrite2Byte((u32RegAddr + _DMIC_MAILBOX_OFFSET), u16Val);
}

static void _mhal_write_mask_reg(unsigned int u32RegAddr, unsigned short u16Mask, unsigned short u16Val)
{
    unsigned short u16RegVal;

    u16RegVal = _mhal_read_reg(u32RegAddr);
    u16RegVal = ((u16RegVal & (~(u16Mask))) | (u16Val & u16Mask));
    _mhal_write_reg(u32RegAddr, u16RegVal);
}

static unsigned int _mhal_get_device_status(void)
{
    if (((_DMIC_AU_AbsRead2Byte(0x2D30 + _MAD_MAILBOX_OFFSET) & 0x0200) == 0x0200)
        || (_DMIC_AU_AbsRead2Byte(0x2A90 + _MAD_MAILBOX_OFFSET) == 0x0000))
    {
        return DMIC_FALSE;
    }
    else
    {
        return DMIC_TRUE;
    }
}

static ptrdiff_t _mhal_convert_pa_to_ba(ptrdiff_t ptrPhysicalAddr)
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
    // DMIC_ERR("ptrPhysicalAddr 0x%llX, ptrPa 0x%llX, ptrBa 0x%llX, ptrPaToBaOffset 0x%llX\n", ptrPhysicalAddr, ptrPa, ptrBa, ptrPaToBaOffset);

    return ptrBa;
}

static unsigned int _mhal_get_common_buffer_pa(void)
{
	unsigned int dsp2_pa = 0;
	unsigned int dsp2_size = 0;

	dsp2_pa = (_DMIC_AU_AbsRead2Byte(0x2A90+_MAD_MAILBOX_OFFSET) + ((_DMIC_AU_AbsRead2Byte(0x2AC0+_MAD_MAILBOX_OFFSET) & 0x0F) << 16)) * _MAD_ADDR_CONVERTOR;
	dsp2_size = _DMIC_AU_AbsRead2Byte(0x2D82+_MAD_MAILBOX_OFFSET) + (_DMIC_AU_AbsRead2Byte(0x2D84+_MAD_MAILBOX_OFFSET) << 16);
	if (dsp2_size == 0) {
		DMIC_ERR("Error! Can't get DSP2 size from mailbox!\n");
		dsp2_size = _MAD_DSP2_DRAM_SIZE;
	}

	return (dsp2_pa + dsp2_size);
}


/* Initiate D-MIC wtih AEC Capture */
static int _mhal_dmic_capture_init(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    //    ptrdiff_t audio_dmic_seamless_capture_base_pa = 0;
    //    ptrdiff_t audio_dmic_seamless_capture_base_ba = 0;

    ptrdiff_t audio_dmic_capture_base_pa = 0;
    ptrdiff_t audio_dmic_capture_base_ba = 0;
    ptrdiff_t audio_dmic_capture_base_va = 0;
    ptrdiff_t audio_dmic_aec_capture_base_pa = 0;
    ptrdiff_t audio_dmic_aec_capture_base_ba = 0;
    ptrdiff_t audio_dmic_aec_capture_base_va = 0;

    //    unsigned int audio_dmic_seamless_capture_offset = _DMIC_SEAMLESS_BASE_OFFSET;

    unsigned int audio_dmic_capture_offset = _DMIC_CAPTURE_BASE_OFFSET;
    unsigned int audio_dmic_aec_capture_offset = _DMIC_AEC_CAPTURE_BASE_OFFSET;
    DMIC_INFO("Initiate MStar DMIC with AEC Capture engine\n");

    //    audio_dmic_seamless_capture_base_pa = _mhal_get_common_buffer_pa() + audio_dmic_seamless_capture_offset;
    //    audio_dmic_seamless_capture_base_ba = _mhal_convert_pa_to_ba(audio_dmic_seamless_capture_base_pa);
    //    g_dmic_seamless_capture_base_va = ( ptrdiff_t ) ioremap_wc(audio_dmic_capture_base_ba, _DMIC_SEAMLESS_BUF_SIZE);

    if ((dmic_aec_capture->initialized_status != DMIC_TRUE) || (dmic_aec_capture->status != E_RESUME))
    {
        audio_dmic_capture_base_pa = _mhal_get_common_buffer_pa() + audio_dmic_capture_offset;
        audio_dmic_capture_base_ba = _mhal_convert_pa_to_ba(audio_dmic_capture_base_pa);

        if ((audio_dmic_capture_base_ba % 0x1000))
        {
            DMIC_ERR("Error! Invalid MStar DMIC capture bus address, it should be aligned by 4 KB!\n");
            return -EFAULT;
        }

        /* convert Bus Address to Virtual Address */
        dmic_aec_capture->buffer.size = _DMIC_CAPTURE_BUF_SIZE;
        if (g_dmic_vdma_capture_base_va == 0)
        {
            g_dmic_vdma_capture_base_va = ( ptrdiff_t ) ioremap_wc(audio_dmic_capture_base_ba, dmic_aec_capture->buffer.size);
            if (g_dmic_vdma_capture_base_va == 0)
            {
                DMIC_ERR("Error! fail to convert DMIC Capture Buffer Bus Address to Virtual Address\n");
                return -ENOMEM;
            }
        }

        audio_dmic_aec_capture_base_pa = _mhal_get_common_buffer_pa() + audio_dmic_aec_capture_offset;
        audio_dmic_aec_capture_base_ba = _mhal_convert_pa_to_ba(audio_dmic_aec_capture_base_pa);

        if ((audio_dmic_aec_capture_base_ba % 0x1000))
        {
            DMIC_ERR("Error! Invalid MStar DMIC AEC capture bus address, it should be aligned by 4 KB!\n");
            return -EFAULT;
        }

        /* convert Bus Address to Virtual Address */
        dmic_aec_capture->aec_buffer.size = _DMIC_AEC_CAPTURE_BUF_SIZE;
        if (g_dmic_aec_capture_base_va == 0)
        {
            g_dmic_aec_capture_base_va
                = ( ptrdiff_t ) ioremap_wc(audio_dmic_aec_capture_base_ba, dmic_aec_capture->aec_buffer.size);
            if (g_dmic_aec_capture_base_va == 0)
            {
                DMIC_ERR("Error! fail to convert DMIC AEC Capture Buffer Bus Address to Virtual Address\n");
                return -ENOMEM;
            }
        }

        audio_dmic_capture_base_va = g_dmic_vdma_capture_base_va;
        audio_dmic_aec_capture_base_va = g_dmic_aec_capture_base_va;

        dmic_aec_capture->str_mode_info.physical_addr = audio_dmic_capture_base_pa;
        dmic_aec_capture->str_mode_info.bus_addr = audio_dmic_capture_base_ba;
        dmic_aec_capture->str_mode_info.virtual_addr = audio_dmic_capture_base_va;

        dmic_aec_capture->aec_str_mode_info.physical_addr = audio_dmic_aec_capture_base_pa;
        dmic_aec_capture->aec_str_mode_info.bus_addr = audio_dmic_aec_capture_base_ba;
        dmic_aec_capture->aec_str_mode_info.virtual_addr = audio_dmic_aec_capture_base_va;

        dmic_aec_capture->initialized_status = DMIC_TRUE;
    }
    else
    {
        audio_dmic_capture_base_pa = dmic_aec_capture->str_mode_info.physical_addr;
        audio_dmic_capture_base_ba = dmic_aec_capture->str_mode_info.bus_addr;
        audio_dmic_capture_base_va = dmic_aec_capture->str_mode_info.virtual_addr;

        audio_dmic_aec_capture_base_pa = dmic_aec_capture->aec_str_mode_info.physical_addr;
        audio_dmic_aec_capture_base_ba = dmic_aec_capture->aec_str_mode_info.bus_addr;
        audio_dmic_aec_capture_base_va = dmic_aec_capture->aec_str_mode_info.virtual_addr;
    }

    _mhal_write_mask_reg(DMIC_ctrl, 0x08, 0x8); // VDMA Pause

    /* init VDMA capture buffer address */
    dmic_aec_capture->buffer.addr = ( unsigned char * ) audio_dmic_capture_base_va;
    _mhal_write_reg(VDMA_BaseAddress_Lo, ((audio_dmic_capture_base_pa / _DMIC_BYTES_IN_LINE) & 0xFFFF));
    _mhal_write_reg(VDMA_BaseAddress_Hi, (((audio_dmic_capture_base_pa / _DMIC_BYTES_IN_LINE) >> 16) & 0xFFFF));

    /* init AEC_DMA capture buffer address */
    dmic_aec_capture->aec_buffer.addr = ( unsigned char * ) audio_dmic_aec_capture_base_va;
    _mhal_write_reg(AECDMA_BaseAddress_Lo, ((audio_dmic_aec_capture_base_pa / _DMIC_BYTES_IN_LINE) & 0xFFFF));
    _mhal_write_reg(AECDMA_BaseAddress_Hi, (((audio_dmic_aec_capture_base_pa / _DMIC_BYTES_IN_LINE) >> 16) & 0xFFFF));

    /* reset VDMA buffer size */
    _mhal_write_reg(VDMA_DRAM_size, ( unsigned short ) (dmic_aec_capture->buffer.size / _DMIC_BYTES_IN_LINE));

    /* reset AECDMA buffer size */
    _mhal_write_reg(AECDMA_DRAM_size, ( unsigned short ) (dmic_aec_capture->aec_buffer.size / _DMIC_BYTES_IN_LINE));

    /* clear all VDMA buffer */
    memset(( void * ) dmic_aec_capture->buffer.addr, 0x00, _DMIC_CAPTURE_BUF_SIZE);
    /* clear all AECDMA buffer */
    memset(( void * ) dmic_aec_capture->aec_buffer.addr, 0x00, _DMIC_AEC_CAPTURE_BUF_SIZE);
    Chip_Flush_Memory();

    /* reset VDMA write pointer */
    dmic_aec_capture->buffer.w_ptr = dmic_aec_capture->buffer.addr;

    /* reset AECDMA write pointer */
    dmic_aec_capture->aec_buffer.w_ptr = dmic_aec_capture->aec_buffer.addr;

    /* reset VDMA read pointer */
    dmic_aec_capture->buffer.r_ptr = dmic_aec_capture->buffer.w_ptr;
    _mhal_write_reg(VDMA_DRAM_readsize, 0);

    /* reset AECDMA read pointer */
    dmic_aec_capture->aec_buffer.r_ptr = dmic_aec_capture->aec_buffer.w_ptr;
    _mhal_write_reg(AECDMA_DRAM_readsize, 0);

    /* reset VDMA */
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_RESET, VDMA_WR_RESET);
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_RESET, 0);

    /* reset AECDMA */
    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_RESET, AECDMA_WR_RESET);
    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_RESET, 0);

    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_LR_SWAP, VDMA_WR_LR_SWAP);

    if (dmic_aec_capture->aec_channel)
    {
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_LR_SWAP, AECDMA_WR_LR_SWAP);
    }

    /* start VDMA */
    _mhal_write_mask_reg(0x2524, 0x1, 0x1);

    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_MIU_ENABLE, VDMA_WR_MIU_ENABLE);
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_ENABLE, VDMA_WR_ENABLE);

    /* start AECDMA */
    if (dmic_aec_capture->aec_channel)
    {
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_MIU_ENABLE, AECDMA_WR_MIU_ENABLE);
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_ENABLE, AECDMA_WR_ENABLE);
    }

    _mhal_write_mask_reg(DMIC_ctrl, 0x08, 0x0); // VDMA un-Pause

    if (dmic_aec_capture->aec_channel)
    {
        _mhal_write_mask_reg(AECDMA_src_ctrl_1, AECDMA_Enable, AECDMA_Enable); // AEC En
    }

    return 0;
}

/* Exit D-MIC AEC capture */
static int _mhal_dmic_capture_exit(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;

    DMIC_INFO("Exit MStar DMIC AEC Capture engine OK\n");

    /* stop VDMA */
    _mhal_write_mask_reg(0x2524, 0x1, 0x0);

    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_LR_SWAP, 0);
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_MIU_ENABLE, 0);
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_ENABLE, 0);

    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_LR_SWAP, 0);
    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_MIU_ENABLE, 0);
    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_ENABLE, 0);

    _mhal_write_mask_reg(DMIC_ctrl, 0x08, 0x8);                // VDMA Pause
    _mhal_write_mask_reg(AECDMA_src_ctrl_1, AECDMA_Enable, 0); // AEC disable

    /* clear D-MIC capture buffer size */
    _mhal_write_reg(VDMA_DRAM_size, 0);

    /* clear D-MIC capture read pointer */
    _mhal_write_reg(VDMA_DRAM_readsize, 0);

    /* clear AECDMA capture buffer size */
    _mhal_write_reg(AECDMA_DRAM_size, 0);

    /* clear AECDMA capture read pointer */
    _mhal_write_reg(AECDMA_DRAM_readsize, 0);

    if (g_dmic_vdma_capture_base_va != 0)
    {
        if (dmic_aec_capture->buffer.addr)
        {
            iounmap(( void * ) dmic_aec_capture->buffer.addr);
            dmic_aec_capture->buffer.addr = 0;
        }
        else
        {
            DMIC_ERR("Error! MStar DMIC AEC Capture buffer address should not be 0 !\n");
        }

        g_dmic_vdma_capture_base_va = 0;
    }

    if (g_dmic_aec_capture_base_va != 0)
    {
        if (dmic_aec_capture->aec_buffer.addr)
        {
            iounmap(( void * ) dmic_aec_capture->aec_buffer.addr);
            dmic_aec_capture->aec_buffer.addr = 0;
        }
        else
        {
            DMIC_ERR("Error! MStar DMIC AEC Capture buffer address should not be 0 !\n");
        }

        g_dmic_aec_capture_base_va = 0;
    }

    /* reset VDMA */
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_RESET, VDMA_WR_RESET);
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_RESET, 0);

    /* reset AECDMA */
    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_RESET, AECDMA_WR_RESET);
    _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_RESET, 0);

    dmic_aec_capture->status = E_STOP;
    dmic_aec_capture->initialized_status = DMIC_FALSE;

    dmic_aec_capture->vdma_channel = -1;
    dmic_aec_capture->aec_channel = -1;

    return 0;
}

/* Start D-MIC AEC capture */
static int _mhal_dmic_capture_start(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    // DMIC_INFO("Start MStar D-MIC capture engine\n");

    dmic_aec_capture->status = E_START;

    return 0;
}

/* Stop D-MIC AEC capture */
static int _mhal_dmic_capture_stop(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    // DMIC_INFO("Stop MStar D-MIC capture engine\n");

    dmic_aec_capture->status = E_STOP;

    return 0;
}

/* Resume D-MIC AEC capture */
static int _mhal_dmic_capture_resume(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    // DMIC_INFO("Resume MStar D-MIC capture engine\n");

    dmic_aec_capture->status = E_RESUME;

    return 0;
}

/* Suspend D-MIC AEC capture */
static int _mhal_dmic_capture_suspend(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    // DMIC_INFO("Suspend MStar D-MIC capture engine\n");

    dmic_aec_capture->status = E_SUSPEND;

    return 0;
}

/* Read PCM from D-MIC AEC capture */
static unsigned int _mhal_dmic_capture_read(void *buffer, unsigned int bytes)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    unsigned int rest_size_to_buffer_end
        = (dmic_aec_capture->buffer.addr + _DMIC_CAPTURE_BUF_SIZE) - dmic_aec_capture->buffer.r_ptr;
    unsigned int read_size = 0;
    unsigned int vdma_bytes = 0;
    unsigned int vdma_read_size = 0;
    unsigned int aec_read_size = 0;
    int loop = 0;
    unsigned int copy_sample = 0;
    unsigned char *bufptr = ( unsigned char * ) buffer;
    int target_alignment_size = 0;

    target_alignment_size = ((dmic_aec_capture->channel_mode >> 1) * _DMIC_BYTES_IN_LINE);
    _DMIC_AU_DO_ALIGNMENT(bytes, target_alignment_size);

    if (dmic_aec_capture->aec_channel)
    {
        vdma_bytes = (bytes / dmic_aec_capture->channel_mode) * dmic_aec_capture->vdma_channel;
        vdma_read_size = (rest_size_to_buffer_end > vdma_bytes) ? vdma_bytes : rest_size_to_buffer_end;
        aec_read_size = (vdma_read_size / dmic_aec_capture->vdma_channel) * dmic_aec_capture->aec_channel;
    }
    else
    {
        vdma_read_size = (rest_size_to_buffer_end > bytes) ? bytes : rest_size_to_buffer_end;
    }

    if (dmic_aec_capture->buffer.r_ptr > (dmic_aec_capture->buffer.addr + _DMIC_CAPTURE_BUF_SIZE))
    {
        DMIC_ERR(" VDMA read size is too large\n");
    }

    if (dmic_aec_capture->aec_buffer.r_ptr > (dmic_aec_capture->aec_buffer.addr + _DMIC_AEC_CAPTURE_BUF_SIZE))
    {
        DMIC_ERR(" AEC DMA read size is too large\n");
    }

    copy_sample = ((vdma_read_size + aec_read_size) / dmic_aec_capture->channel_mode / (dmic_aec_capture->bit_width / 8));

    if (dmic_aec_capture->aec_channel)
    {
        for (loop = 0; loop < copy_sample; loop++)
        {
            /* VDMA */
            *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
            *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
            *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
            *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
            if (dmic_aec_capture->bit_width == 32)
            {
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
            }

            if (dmic_aec_capture->vdma_channel > 2)
            {
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                if (dmic_aec_capture->bit_width == 32)
                {
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                }
            }

            if (dmic_aec_capture->vdma_channel > 4)
            {
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                if (dmic_aec_capture->bit_width == 32)
                {
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                    *bufptr++ = *(dmic_aec_capture->buffer.r_ptr++);
                }
            }

            if (dmic_aec_capture->buffer.r_ptr >= (dmic_aec_capture->buffer.addr + _DMIC_CAPTURE_BUF_SIZE))
                dmic_aec_capture->buffer.r_ptr = dmic_aec_capture->buffer.addr;

            /* AEC DMA */
            *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
            *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
            *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
            *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
            if (dmic_aec_capture->bit_width == 32)
            {
                *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
                *bufptr++ = *(dmic_aec_capture->aec_buffer.r_ptr++);
            }

            if (dmic_aec_capture->aec_buffer.r_ptr >= (dmic_aec_capture->aec_buffer.addr + _DMIC_AEC_CAPTURE_BUF_SIZE))
                dmic_aec_capture->aec_buffer.r_ptr = dmic_aec_capture->aec_buffer.addr;
        }
    }
    else
    {
        memcpy(buffer, dmic_aec_capture->buffer.r_ptr, vdma_read_size);

        dmic_aec_capture->buffer.r_ptr += vdma_read_size;
        if (dmic_aec_capture->buffer.r_ptr == (dmic_aec_capture->buffer.addr + _DMIC_CAPTURE_BUF_SIZE))
            dmic_aec_capture->buffer.r_ptr = dmic_aec_capture->buffer.addr;
    }
    Chip_Flush_Memory();

    _mhal_write_reg(VDMA_DRAM_readsize, ( unsigned short ) (vdma_read_size / _DMIC_BYTES_IN_LINE));
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_TRIG, VDMA_WR_TRIG);
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_TRIG, 0);

    if (dmic_aec_capture->aec_channel)
    {
        _mhal_write_reg(AECDMA_DRAM_readsize, ( unsigned short ) (aec_read_size / _DMIC_BYTES_IN_LINE));
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_TRIG, AECDMA_WR_TRIG);
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_TRIG, 0);
    }

    read_size = vdma_read_size + aec_read_size;

    return read_size;
}

/* Get infromation from D-MIC AEC Capture */
static int _mhal_dmic_capture_get(int cmd, unsigned int *param)
{
    int err = 0;
    // DMIC_INFO("Get parameter from DMIC Capture engine\n");

    switch (cmd)
    {
        case E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES:
        {
            *param = _mhal_dmic_capture_get_new_avail_bytes();
            break;
        }

        case E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES:
        {
            *param = _mhal_dmic_capture_get_total_avail_bytes();
            break;
        }

        case E_PCM_CAPTURE_GET_DEVICE_STATUS:
        {
            *param = _mhal_get_device_status();
            break;
        }

        default:
        {
            DMIC_INFO("Invalid GET command %d\n", cmd);
            err = -EINVAL;
            break;
        }
    }

    return err;
}

/* Set information to D-MIC AEC Capture */
static int _mhal_dmic_capture_set(int cmd, unsigned int *param)
{
    int err = 0;
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    // DMIC_INFO("Set parameter to DMIC Capture engine\n");

    switch (cmd)
    {
        case E_PCM_CAPTURE_SET_BUFFER_SIZE:
        {
            if (*param > _DMIC_CAPTURE_BUF_SIZE)
            {
                *param = _DMIC_CAPTURE_BUF_SIZE;
                DMIC_INFO("Target buffer is too large, reset to %u\n", *param);
            }

            if ((*param % _DMIC_BYTES_IN_LINE))
            {
                *param = (*param / _DMIC_BYTES_IN_LINE) * _DMIC_BYTES_IN_LINE;
                DMIC_INFO("Target buffer is not aligned, reset to %u\n", *param);
            }

            _mhal_dmic_capture_set_buffer_size(*param);
            break;
        }

        case E_PCM_CAPTURE_SET_CHANNEL_MODE:
        {
            dmic_aec_capture->channel_mode = ( unsigned int ) *param;
            _mhal_dmic_capture_set_channel_mode(dmic_aec_capture->channel_mode);
            break;
        }

        case E_PCM_CAPTURE_SET_SAMPLE_RATE:
        {
            _mhal_dmic_capture_set_sample_rate(*param);
            break;
        }

        case E_PCM_CAPTURE_SET_BIT_WIDTH:
        {
            _mhal_dmic_capture_set_bit_width(*param);
            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_ENABLE:
        {
            /* DMIC CLOCK OFF */
            if (*param == 0)
            {
                _mhal_write_mask_reg(0x2920, 0x1, 0x0);
            }
            /* DMIC CLOCK ON */
            else
            {
                _mhal_write_mask_reg(0x2920, 0x1, 0x1);
            }

            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_NUM:
        {
            dmic_aec_capture->vdma_channel = *param;
            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_REF_NUM:
        {
            dmic_aec_capture->aec_channel = *param;
            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_HPF_SWITCH:
        {
            if (*param == 0)
            {
                _mhal_write_mask_reg(0x24a6, 0x1, 0x1); // bypass HPF
            }
            else
            {
                _mhal_write_mask_reg(0x24a6, 0x1, 0x0); // enable HPF

                if (*param == 1)
                {
                    _mhal_write_mask_reg(0x24a6, 0x2, 0x0); // HPF 1-stage
                }
                else
                {
                    _mhal_write_mask_reg(0x24a6, 0x2, 0x2); // HPF 2-stage
                }
            }
            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_HPF_CONFIG:
        {
            if (*param >= 1 && *param <= 12)
            {
                _mhal_write_mask_reg(0x24a6, 0xf00, (*param << 8)); // HPF range 1~12
            }
            else
            {
                DMIC_INFO("Invalid HPF value %d, range should be 1~12\n", *param);
            }

            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_SINE_GEN:
        {
            if (*param == 0)
            {
                /* DMIC SINE GEN OFF */
                _mhal_write_mask_reg(0x2406, 0x1, 0x0);
                _mhal_write_mask_reg(0x2406, 0x30, 0x00);
                /* REF SINE GEN OFF */
                _mhal_write_mask_reg(0x25da, 0x400, 0x0);
            }
            else
            {
                /* DMIC SINE GEN ON */
                if (dmic_aec_capture->vdma_channel)
                {
                    _mhal_write_mask_reg(0x2406, 0x30, 0x20);
                    _mhal_write_mask_reg(0x2406, 0x1, 0x1);
                    _mhal_write_mask_reg(0x2406, 0xf00, 0x0);
                    _mhal_write_mask_reg(0x2406, 0xf000, 0x2000);
                }

                /* REF SINE GEN ON */
                if (dmic_aec_capture->aec_channel)
                {
                    if (dmic_aec_capture->channel_mode > 4)
                    {
                        /* sine gen */
                        _mhal_write_mask_reg(0x25da, 0x400, 0x400);
                        /* pattern gen */
                        //_mhal_write_mask_reg(0x2206, 0x1000, 0x1000);
                        //_mhal_write_mask_reg(0x2206, 0x2000, 0x2000);
                    }
                }
            }

            break;
        }

        case E_PCM_CAPTURE_SET_DMIC_GAIN:
        {
            unsigned int gain = 0;

            if (*param)
            {
                gain = ~(*param) + 1;
            }

            _mhal_write_mask_reg(0x24ae, 0x1, 0x1);
            _mhal_write_mask_reg(0x24b2, 0x0fff, gain);   // db
            _mhal_write_mask_reg(0x24b2, 0x8000, 0x0000); // trigger : 0 -> 1 -> 0
            _mhal_write_mask_reg(0x24b2, 0x8000, 0x8000);
            _mhal_write_mask_reg(0x24b2, 0x8000, 0x0000);

            break;
        }

        default:
        {
            DMIC_INFO("Invalid SET command %d\n", cmd);
            err = -EINVAL;
            break;
        }
    }

    return err;
}

/* Get D-MIC AEC Capture's new PCM available bytes */
static unsigned int _mhal_dmic_capture_get_new_avail_bytes(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    unsigned char *new_w_ptr = NULL;
    unsigned char *aec_new_w_ptr = NULL;
    int new_avail_bytes = 0;
    int vdma_level_bytes = 0;
    int aecdma_level_bytes = 0;
    int avail_bytes = 0;
    int target_alignment_size = 0;
    int aec_new_avail_bytes = 0;
    int aec_avail_bytes = 0;
    int aec_actual_avail_byte = 0;

    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_LEVEL_MASK, VDMA_WR_LEVEL_MASK);
    vdma_level_bytes = _mhal_read_reg(VDMA_DRAM_levelcnt) * _DMIC_BYTES_IN_LINE;
    _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_LEVEL_MASK, 0);

    if (dmic_aec_capture->aec_channel)
    {
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_LEVEL_MASK, AECDMA_WR_LEVEL_MASK);
        aecdma_level_bytes = _mhal_read_reg(AECDMA_DRAM_levelcnt) * _DMIC_BYTES_IN_LINE;
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_LEVEL_MASK, 0);
    }

    // vdma
    avail_bytes = ((vdma_level_bytes > VDMA_LOCALBUF_SIZE) ? (vdma_level_bytes - VDMA_LOCALBUF_SIZE)
                                                           : 0); // level count contains the local buffer data size
    // aec dma
    if (dmic_aec_capture->aec_channel)
    {
        aec_avail_bytes = ((aecdma_level_bytes > AECDMA_LOCALBUF_SIZE) ? (aecdma_level_bytes - AECDMA_LOCALBUF_SIZE) : 0); // level count contains the local buffer data size
    }

    if ((avail_bytes >= (dmic_aec_capture->buffer.size - VDMA_LOCALBUF_SIZE - _DMIC_CAPTURE_BUF_UNIT))
        || (aec_avail_bytes >= (dmic_aec_capture->aec_buffer.size - AECDMA_LOCALBUF_SIZE - _DMIC_CAPTURE_BUF_UNIT)))
    {
        DMIC_INFO("***** Audio DMIC AEC Capture Buffer is overrun !! ***** \n");

        /* reset D-MIC capture buffer size */
        _mhal_write_reg(VDMA_DRAM_size, ( unsigned short ) (dmic_aec_capture->buffer.size / _DMIC_BYTES_IN_LINE));

        /* reset D-MIC capture write pointer */
        dmic_aec_capture->buffer.w_ptr = dmic_aec_capture->buffer.addr;

        /* reset D-MIC capture read pointer */
        dmic_aec_capture->buffer.r_ptr = dmic_aec_capture->buffer.w_ptr;
        _mhal_write_reg(VDMA_DRAM_readsize, 0);

        /* reset D-MIC capture */
        _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_RESET, VDMA_WR_RESET);
        _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_RESET, 0);

        /* reset AECDMA capture buffer size */
        _mhal_write_reg(AECDMA_DRAM_size, ( unsigned short ) (dmic_aec_capture->aec_buffer.size / _DMIC_BYTES_IN_LINE));

        /* reset AECDMA capture write pointer */
        dmic_aec_capture->aec_buffer.w_ptr = dmic_aec_capture->aec_buffer.addr;

        /* reset AECDMA capture read pointer */
        dmic_aec_capture->aec_buffer.r_ptr = dmic_aec_capture->aec_buffer.w_ptr;
        _mhal_write_reg(AECDMA_DRAM_readsize, 0);

        /* reset AECDMA capture */
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_RESET, AECDMA_WR_RESET);
        _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_RESET, 0);

        return 0;
    }

    if (dmic_aec_capture->aec_channel)
    {
        if (aec_avail_bytes < (avail_bytes / (dmic_aec_capture->vdma_channel / dmic_aec_capture->aec_channel)))
        {
            avail_bytes = aec_avail_bytes * (dmic_aec_capture->vdma_channel / dmic_aec_capture->aec_channel);
        }
    }

    target_alignment_size = (_DMIC_BYTES_IN_LINE * 2);
    _DMIC_AU_DO_ALIGNMENT(avail_bytes, target_alignment_size);

    new_w_ptr = dmic_aec_capture->buffer.r_ptr + avail_bytes;

    if (new_w_ptr >= (dmic_aec_capture->buffer.addr + _DMIC_CAPTURE_BUF_SIZE))
    {
        new_w_ptr -= _DMIC_CAPTURE_BUF_SIZE;
    }

    new_avail_bytes = new_w_ptr - dmic_aec_capture->buffer.w_ptr;
    if (new_avail_bytes < 0)
    {
        new_avail_bytes += _DMIC_CAPTURE_BUF_SIZE;
    }

    dmic_aec_capture->buffer.w_ptr = new_w_ptr;

    if (dmic_aec_capture->aec_channel)
    {
        aec_actual_avail_byte = (((avail_bytes / dmic_aec_capture->vdma_channel) * dmic_aec_capture->aec_channel) > aec_avail_bytes)
                                    ? aec_avail_bytes
                                    : ((avail_bytes / dmic_aec_capture->vdma_channel) * dmic_aec_capture->aec_channel);

        target_alignment_size = _DMIC_BYTES_IN_LINE;
        _DMIC_AU_DO_ALIGNMENT(aec_actual_avail_byte, target_alignment_size);

        aec_new_w_ptr = dmic_aec_capture->aec_buffer.r_ptr + aec_actual_avail_byte;

        if (aec_new_w_ptr >= (dmic_aec_capture->aec_buffer.addr + _DMIC_AEC_CAPTURE_BUF_SIZE))
        {
            aec_new_w_ptr -= _DMIC_AEC_CAPTURE_BUF_SIZE;
        }

        aec_new_avail_bytes = aec_new_w_ptr - dmic_aec_capture->aec_buffer.w_ptr;
        if (aec_new_avail_bytes < 0)
        {
            aec_new_avail_bytes += _DMIC_AEC_CAPTURE_BUF_SIZE;
        }

        dmic_aec_capture->aec_buffer.w_ptr = aec_new_w_ptr;
    }

    return (new_avail_bytes + aec_new_avail_bytes);
}

/* Get D-MIC AEC Capture's total PCM available bytes */
static unsigned int _mhal_dmic_capture_get_total_avail_bytes(void)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    int avail_bytes = 0;
    int aec_avail_bytes = 0;
    int acutal_size = 0;
    int target_alignment_size = 0;

    avail_bytes = dmic_aec_capture->buffer.w_ptr - dmic_aec_capture->buffer.r_ptr;
    if (avail_bytes < 0)
    {
        avail_bytes += _DMIC_CAPTURE_BUF_SIZE;
    }

    if (dmic_aec_capture->aec_channel)
    {
        aec_avail_bytes = dmic_aec_capture->aec_buffer.w_ptr - dmic_aec_capture->aec_buffer.r_ptr;
        if (aec_avail_bytes < 0)
        {
            aec_avail_bytes += _DMIC_AEC_CAPTURE_BUF_SIZE;
        }

        if (aec_avail_bytes >= (avail_bytes / (dmic_aec_capture->vdma_channel / dmic_aec_capture->aec_channel)))
        {
            acutal_size = avail_bytes + (avail_bytes / (dmic_aec_capture->vdma_channel / dmic_aec_capture->aec_channel));
        }
        else
        {
            acutal_size = aec_avail_bytes + (aec_avail_bytes * (dmic_aec_capture->vdma_channel / dmic_aec_capture->aec_channel));
        }
    }
    else
    {
        acutal_size = avail_bytes;
    }

    target_alignment_size = ((dmic_aec_capture->channel_mode / 2) * _DMIC_BYTES_IN_LINE);
    _DMIC_AU_DO_ALIGNMENT(acutal_size, target_alignment_size);

    if (acutal_size < ((dmic_aec_capture->channel_mode >> 1) * _DMIC_BYTES_IN_LINE))
    {
        return 0;
    }

    return acutal_size;
}

/* Set D-MIC AEC Capture's PCM buffer size */
static int _mhal_dmic_capture_set_buffer_size(unsigned int buffer_size)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    DMIC_INFO("Target buffer size is %u\n", buffer_size);

    dmic_aec_capture->buffer.size = buffer_size;
    _mhal_write_reg(VDMA_DRAM_size, ( unsigned short ) (buffer_size / _DMIC_BYTES_IN_LINE));

    return 0;
}

/* Set D-MIC AEC Capture's channel mode */
static int _mhal_dmic_capture_set_channel_mode(unsigned int channel_mode)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    DMIC_INFO("Target channel is %d with AEC\n", channel_mode);

    switch (channel_mode)
    {
        case 2:
        {
            if (dmic_aec_capture->aec_channel < 0)
            {
                dmic_aec_capture->aec_channel = 0;
            }

            if (dmic_aec_capture->vdma_channel < 0)
            {
                dmic_aec_capture->vdma_channel = dmic_aec_capture->channel_mode - dmic_aec_capture->aec_channel;
            }

            if (dmic_aec_capture->channel_mode != (dmic_aec_capture->vdma_channel + dmic_aec_capture->aec_channel))
            {
                DMIC_ERR(" Channel mode not right !! channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
                dmic_aec_capture->channel_mode = 2;
                dmic_aec_capture->vdma_channel = 2;
                dmic_aec_capture->aec_channel = 0;
                DMIC_ERR(" Set Channel mode to channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
            }

            _mhal_dmic_set_dmic_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_vdma_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_ref_channel_mode(dmic_aec_capture->aec_channel);

            break;
        }

        case 4:
        {
            if (dmic_aec_capture->aec_channel < 0)
            {
                dmic_aec_capture->aec_channel = 0;
            }

            if (dmic_aec_capture->vdma_channel < 0)
            {
                dmic_aec_capture->vdma_channel = dmic_aec_capture->channel_mode - dmic_aec_capture->aec_channel;
            }

            if (dmic_aec_capture->channel_mode != (dmic_aec_capture->vdma_channel + dmic_aec_capture->aec_channel))
            {
                DMIC_ERR(" Channel mode not right !! channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
                dmic_aec_capture->channel_mode = 4;
                dmic_aec_capture->vdma_channel = 4;
                dmic_aec_capture->aec_channel = 0;
                DMIC_ERR(" Set Channel mode to channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
            }

            _mhal_dmic_set_dmic_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_vdma_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_ref_channel_mode(dmic_aec_capture->aec_channel);

            break;
        }

        case 6:
        {
            if (dmic_aec_capture->aec_channel < 0)
            {
                dmic_aec_capture->aec_channel = 2;
            }

            if (dmic_aec_capture->vdma_channel < 0)
            {
                dmic_aec_capture->vdma_channel = dmic_aec_capture->channel_mode - dmic_aec_capture->aec_channel;
            }

            if (dmic_aec_capture->channel_mode != (dmic_aec_capture->vdma_channel + dmic_aec_capture->aec_channel))
            {
                DMIC_ERR(" Channel mode not right !! channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
                dmic_aec_capture->channel_mode = 6;
                dmic_aec_capture->vdma_channel = 4;
                dmic_aec_capture->aec_channel = 2;
                DMIC_ERR(" Set Channel mode to channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
            }

            _mhal_dmic_set_dmic_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_vdma_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_ref_channel_mode(dmic_aec_capture->aec_channel);
            break;
        }

        case 8:
        {
            if (dmic_aec_capture->aec_channel < 0)
            {
                dmic_aec_capture->aec_channel = 2;
            }

            if (dmic_aec_capture->vdma_channel < 0)
            {
                dmic_aec_capture->vdma_channel = dmic_aec_capture->channel_mode - dmic_aec_capture->aec_channel;
            }

            if (dmic_aec_capture->channel_mode != (dmic_aec_capture->vdma_channel + dmic_aec_capture->aec_channel))
            {
                DMIC_ERR(" Channel mode not right !! channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
                dmic_aec_capture->channel_mode = 8;
                dmic_aec_capture->vdma_channel = 6;
                dmic_aec_capture->aec_channel = 2;
                DMIC_ERR(" Set Channel mode to channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
            }

            _mhal_dmic_set_dmic_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_vdma_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_ref_channel_mode(dmic_aec_capture->aec_channel);

            break;
        }

        default:
        {
            dmic_aec_capture->vdma_channel = 4;
            dmic_aec_capture->aec_channel = 0;

            if (dmic_aec_capture->channel_mode != (dmic_aec_capture->vdma_channel + dmic_aec_capture->aec_channel))
            {
                DMIC_ERR(" Channel mode not right !! channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
                dmic_aec_capture->channel_mode = 4;
                dmic_aec_capture->vdma_channel = 4;
                dmic_aec_capture->aec_channel = 0;
                DMIC_ERR(" Set Channel mode to channel num [%d] vdma [%d] ref [%d]\n",
                           dmic_aec_capture->channel_mode, dmic_aec_capture->vdma_channel, dmic_aec_capture->aec_channel);
            }

            _mhal_dmic_set_dmic_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_vdma_channel_mode(dmic_aec_capture->vdma_channel);
            _mhal_dmic_set_ref_channel_mode(dmic_aec_capture->aec_channel);

            DMIC_ERR("unsupported channel number(%d), set to default 4 ch\n", channel_mode);
            break;
        }
    }

    return 0;
}

/* Set smaple rate to  D-MIC AEC */
static int _mhal_dmic_capture_set_sample_rate(unsigned int sample_rate)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    DMIC_INFO("Target sample rate is %u\n", sample_rate);

    switch (sample_rate)
    {
        case 16000:
        {
            dmic_aec_capture->sample_rate = sample_rate;
            /* PDM */
            _mhal_write_mask_reg(VDMA_vrec_ctrl_4, 0x3F, 0x18);
            /* AEC */
            _mhal_write_mask_reg(AECDMA_src_ctrl_1, 0x0F, 0);
            break;
        }

        case 32000:
        {
            dmic_aec_capture->sample_rate = sample_rate;
            /* PDM */
            _mhal_write_mask_reg(VDMA_vrec_ctrl_4, 0x3F, 0x28);
            /* AEC */
            _mhal_write_mask_reg(AECDMA_src_ctrl_1, 0x0F, 1);
            break;
        }

        case 48000:
        {
            dmic_aec_capture->sample_rate = sample_rate;
            /* PDM */
            _mhal_write_mask_reg(VDMA_vrec_ctrl_4, 0x3F, 0x38);
            /* AEC */
            _mhal_write_mask_reg(AECDMA_src_ctrl_1, 0x0F, 2);
            break;
        }

        default:
        {
            DMIC_ERR("Error! un-supported sample rate %u, set to 48KHz !!!\n", sample_rate);
            dmic_aec_capture->sample_rate = 48000;
            break;
        }
    }

    return 0;
}

/* set D-MIC channel mode*/
static int _mhal_dmic_set_dmic_channel_mode(unsigned int nChannel)
{
    unsigned int channel_mode;

    switch (nChannel)
    {
        case 1:
        {
            channel_mode = DMIC_CHANNEL_1;
            break;
        }

        case 2:
        {
            channel_mode = DMIC_CHANNEL_2;
            break;
        }

        case 4:
        {
            channel_mode = DMIC_CHANNEL_4;
            break;
        }

        case 6:
        {
            channel_mode = DMIC_CHANNEL_6;
            break;
        }

        case 8:
        {
            channel_mode = DMIC_CHANNEL_8;
            break;
        }

        default:
        {
            channel_mode = DMIC_CHANNEL_2;
            break;
        }
    }

    _mhal_write_mask_reg(DMIC_ctrl, 0x30, channel_mode << 4);

    return 0;
}

/* set VDMA channel mode*/
static int _mhal_dmic_set_vdma_channel_mode(unsigned int nChannel)
{
    unsigned int channel_mode;

    switch (nChannel)
    {
        case 2:
        {
            channel_mode = VDMA_CHANNEL_2;
            break;
        }

        case 4:
        {
            channel_mode = VDMA_CHANNEL_4;
            break;
        }

        case 6:
        {
            channel_mode = VDMA_CHANNEL_6;
            break;
        }

        case 8:
        {
            channel_mode = VDMA_CHANNEL_8;
            break;
        }

        default:
        {
            channel_mode = VDMA_CHANNEL_2;
            break;
        }
    }

    _mhal_write_mask_reg(VDMA_Channel_mode, 0x07, channel_mode);

    return 0;
}

/* set REF channel mode*/
static int _mhal_dmic_set_ref_channel_mode(unsigned int nChannel)
{
    unsigned int channel_mode;

    switch (nChannel)
    {
        case 2:
        {
            channel_mode = REF_CHANNEL_2;
            break;
        }

        case 4:
        {
            channel_mode = REF_CHANNEL_4;
            break;
        }

        case 6:
        {
            channel_mode = REF_CHANNEL_6;
            break;
        }

        case 8:
        {
            channel_mode = REF_CHANNEL_8;
            break;
        }

        default:
        {
            channel_mode = REF_CHANNEL_2;
            break;
        }
    }

    _mhal_write_mask_reg(AECDMA_src_ctrl_7, AECDMA_MCH_ENABLE, AECDMA_MCH_ENABLE);
    _mhal_write_mask_reg(AECDMA_Channel_mode, 0x07, channel_mode);

    return 0;
}

/* Set bit width to D-MIC AEC */
static int _mhal_dmic_capture_set_bit_width(unsigned int bit_width)
{
    struct MStar_PCM_DMIC_Capture_Struct *dmic_aec_capture = &g_dmic_aec_capture;
    DMIC_INFO("Target bit width is %u\n", bit_width);

    switch (bit_width)
    {
        case 16:
        {
            dmic_aec_capture->bit_width = 16;
            _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_BIT_MODE, 0);
            _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_BIT_MODE, 0);
            break;
        }

        case 32:
        {
            dmic_aec_capture->bit_width = 32;
            _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_BIT_MODE, VDMA_WR_BIT_MODE);
            _mhal_write_mask_reg(AECDMA_ctrl, AECDMA_WR_BIT_MODE, AECDMA_WR_BIT_MODE);
            break;
        }

        default:
        {
            dmic_aec_capture->bit_width = 16;
            _mhal_write_mask_reg(VDMA_ctrl, VDMA_WR_BIT_MODE, 0);
            _mhal_write_mask_reg(AECDMA_ctrl, VDMA_WR_BIT_MODE, 0);
            DMIC_ERR("Error! un-supported bit width %u, set to 16bit !!!\n", bit_width);
            break;
        }
    }

    return 0;
}

static int __init _mhal_dmic_init(void)
{
    unsigned int i = 0;
    while (!((DmicInitTbl_0[i].u32Addr == 0xFFFFFF) && (DmicInitTbl_0[i].u8Mask == 0)))
    {
        _DMIC_AU_AbsWriteMaskByte(DmicInitTbl_0[i].u32Addr, DmicInitTbl_0[i].u8Mask, DmicInitTbl_0[i].u8Value);
        i++;
    };

    int err = 0;
    DMIC_INFO("Initiate MStar DMIC core driver\n");

    memset(&hal_dmic_info, 0x00, sizeof(struct MStar_DMIC_Info));

    snprintf(hal_dmic_info.version, sizeof(hal_dmic_info.version), "%d.%d.%d", _DMIC_HAL_VERSION_MAJOR,
             _DMIC_HAL_VERSION_MINOR, _DMIC_HAL_VERSION_REVISION);

    hal_dmic_info.capture_pcm_ops = &MStar_DMIC_AEC_Capture_Ops;
    hal_dmic_info.max_pcm_buffer_size = _DMIC_CAPTURE_BUF_SIZE;

    err = _mdrv_dmic_hook_device(&hal_dmic_info);
    if (err < 0)
    {
        DMIC_ERR("Error(%d)! fail to hook PCM operators\n", err);
        return err;
    }

    return 0;
}

static void __exit _mhal_dmic_exit(void)
{
    int err = 0;

    DMIC_INFO("Exit DMIC core driver\n");

    err = _mdrv_dmic_unhook_device();
    if (err < 0)
    {
        DMIC_ERR("Error(%d)! fail to unhook PCM operators\n", err);
        return;
    }

    return;
}

/*
 * ============================================================================
 * Module Information
 * ============================================================================
 */
module_init(_mhal_dmic_init);
module_exit(_mhal_dmic_exit);

MODULE_AUTHOR("MStar Semiconductor, Inc.");
MODULE_DESCRIPTION("MStar DMIC Driver - HAL Layer");
MODULE_SUPPORTED_DEVICE("DMIC DEVICE");
MODULE_LICENSE("Proprietary");
