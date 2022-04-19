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

#ifndef __eMMC_M7332_LINUX__
#define __eMMC_M7332_LINUX__

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/scatterlist.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <mstar/mstar_chip.h>
#include <mach/io.h>
#include <linux/random.h>
#include "chip_int.h"
#include "chip_setup.h"
#include "linux/kern_levels.h"

extern ptrdiff_t mstar_pm_base;

//=====================================================
// tool-chain attributes
//=====================================================
//[FIXME] -->
#define eMMC_CACHE_LINE                 0x40 // [FIXME]

#define eMMC_PACK0
#define eMMC_PACK1                      __attribute__((__packed__))
#define eMMC_ALIGN0
#define eMMC_ALIGN1                     __attribute__((aligned(eMMC_CACHE_LINE)))
// <-- [FIXME]

//=====================================================
// HW registers
//=====================================================
#define REG_OFFSET_SHIFT_BITS           2

#define REG_FCIE_U16(Reg_Addr)          (*(volatile U16*)((uintptr_t)Reg_Addr))
#define GET_REG_ADDR(x, y)              ((x)+((y) << REG_OFFSET_SHIFT_BITS))

#define REG_FCIE(reg_addr)              REG_FCIE_U16(reg_addr)
#define REG_FCIE_W(reg_addr, val)       REG_FCIE(reg_addr) = (val)
#define REG_FCIE_R(reg_addr, val)       val = REG_FCIE(reg_addr)
#define REG_FCIE_SETBIT(reg_addr, val)  REG_FCIE(reg_addr) |= (val)
#define REG_FCIE_CLRBIT(reg_addr, val)  REG_FCIE(reg_addr) &= ~(val)
#define REG_FCIE_W1C(reg_addr, val)     REG_FCIE_W(reg_addr, REG_FCIE(reg_addr)&(val))

//------------------------------
#if defined(CONFIG_ARM)
#define RIU_PM_BASE                     (IO_ADDRESS(0x1F000000UL))
#define RIU_BASE                        (IO_ADDRESS(0x1F200000UL))
#define GPIO_RIU_BASE_ADDRESS           (IO_ADDRESS(0x1F600000UL))
#elif defined(CONFIG_ARM64)
#define RIU_PM_BASE                     ((uintptr_t)(mstar_pm_base))
#define RIU_BASE                        ((uintptr_t)(mstar_pm_base+0x200000))
#define GPIO_RIU_BASE_ADDRESS           ((uintptr_t)(mstar_pm_base+0x600000))
#endif

#define REG_BANK_FCIE0                  0x8980
#define REG_BANK_FCIE1                  0x8A00
#define REG_BANK_FCIE2                  0x8A80

#define FCIE0_BASE                      GET_REG_ADDR(RIU_BASE, REG_BANK_FCIE0)
#define FCIE1_BASE                      GET_REG_ADDR(RIU_BASE, REG_BANK_FCIE1)
#define FCIE2_BASE                      GET_REG_ADDR(RIU_BASE, REG_BANK_FCIE2)

#define FCIE_REG_BASE_ADDR              GET_REG_ADDR(FCIE0_BASE, 0x00)
#define FCIE_CMDFIFO_BASE_ADDR          GET_REG_ADDR(FCIE0_BASE, 0x20)
#define FCIE_CIFD_BASE_ADDR             GET_REG_ADDR(FCIE1_BASE, 0x00)
#define FCIE_POWEER_SAVE_MODE_BASE      GET_REG_ADDR(FCIE2_BASE, 0x00)

#define FCIE_NC_WBUF_CIFD_BASE          GET_REG_ADDR(FCIE1_BASE, 0x00)
#define FCIE_NC_RBUF_CIFD_BASE          GET_REG_ADDR(FCIE1_BASE, 0x20)
//------------------------------------------------------------------
#define FCIE1_REG_BASE_ADDR             GET_REG_ADDR(FCIE1_BASE, 0x00)
#define FCIE1_0X51 			            GET_REG_ADDR(FCIE1_REG_BASE_ADDR, 0x51)
#define ENABLE_FCIE_MIU_DEBUG           1
//------------------------------------------------------------------
#include "eMMC_reg_v5.h"
//--------------------------------MIU------------------------------------
#define REG_BANK_MIU                        0x900
#define MIU_BASE_ADDR                       GET_REG_ADDR(RIU_BASE, REG_BANK_MIU)

//--------------------------------clock gen2------------------------------------
#define REG_BANK_CLKGEN2                0x0500	// (0x100A - 0x1000) x 80h
#define CLKGEN2_BASE                    GET_REG_ADDR(RIU_BASE, REG_BANK_CLKGEN2)
#define reg_ckg_fcie_syn                GET_REG_ADDR(CLKGEN2_BASE, 0x0C)

//--------------------------------clock gen------------------------------------
#define REG_BANK_CLKGEN0                0x0580	// (0x100B - 0x1000) x 80h
#define CLKGEN0_BASE                    GET_REG_ADDR(RIU_BASE, REG_BANK_CLKGEN0)

#define reg_ckg_fcie                    GET_REG_ADDR(CLKGEN0_BASE, 0x64)
#define BIT_FCIE_CLK_GATING             BIT0
#define BIT_FCIE_CLK_INVERSE            BIT1
#define BIT_CLKGEN_FCIE_MASK            (BIT5|BIT4|BIT3|BIT2)
#define BIT_FCIE_CLK_SRC_SEL            BIT6

#define reg_ckg_ecc                     GET_REG_ADDR(CLKGEN0_BASE, 0x67)
#define BIT_ECC_CLK_GATING              BIT0

//--------------------------------gpio function mux--------------------------------------
#define REG_BANK_GPIO_MUX               0x11480  // (0x3229 - 0x3000) x 80h
#define PAD_GPIO_MUX_BASE               GET_REG_ADDR(GPIO_RIU_BASE_ADDRESS, REG_BANK_GPIO_MUX)
#define PAD_CHIPTOP_BASE                GET_REG_ADDR(GPIO_RIU_BASE_ADDRESS, REG_BANK_GPIO_MUX)

#define reg_gpio_mux_0x30               GET_REG_ADDR(PAD_GPIO_MUX_BASE, 0x30)
#define BIT_EMMC_CONFIG_EMMC_MODE_1     BIT0
#define BIT_reg_emmc_rstz_en            BIT4

#define reg_nand_mode                   GET_REG_ADDR(PAD_GPIO_MUX_BASE, 0x32)
#define BIT_NAND_MODE                   (BIT1|BIT0)

#define reg_gpio_mux_0x70               GET_REG_ADDR(PAD_GPIO_MUX_BASE, 0x70)
#define BIT_ALL_PAD_IN                  BIT0
//--------------------------------FCIE4-------------------------------------
#define REG_BANK_FCIE4                  0x100  // (0x3002 - 0x3000) x 80h
#define FCIE4_BASE                      GET_REG_ADDR(GPIO_RIU_BASE_ADDRESS, REG_BANK_FCIE4)

//--------------------------------emmc pll--------------------------------------
#define REG_BANK_EMMC_PLL               0x11F80	// (0x123F - 0x1000) x 80h
#define EMMC_PLL_BASE                   GET_REG_ADDR(RIU_BASE, REG_BANK_EMMC_PLL)
#define REG_EMMC_PLL_RX01               GET_REG_ADDR(EMMC_PLL_BASE, 0x01)
#define reg_emmcpll_0x02                GET_REG_ADDR(EMMC_PLL_BASE, 0x02)

#define reg_emmcpll_0x03                GET_REG_ADDR(EMMC_PLL_BASE, 0x03)
#define BIT_SKEW1_MASK                  (BIT3|BIT2|BIT1|BIT0)
#define BIT_SKEW2_MASK                  (BIT7|BIT6|BIT5|BIT4)
#define BIT_SKEW3_MASK                  (BIT11|BIT10|BIT9|BIT8)
#define BIT_SKEW4_MASK                  (BIT15|BIT14|BIT13|BIT12)
#define BIT_DEFAULT_SKEW2               (BIT6|BIT4)         //5

#define reg_emmcpll_fbdiv               GET_REG_ADDR(EMMC_PLL_BASE, 0x04)
#define reg_emmcpll_pdiv                GET_REG_ADDR(EMMC_PLL_BASE, 0x05)
#define reg_emmc_pll_reset              GET_REG_ADDR(EMMC_PLL_BASE, 0x06)
#define reg_emmc_pll_test               GET_REG_ADDR(EMMC_PLL_BASE, 0x07)
#define reg_emmcpll_0x09                GET_REG_ADDR(EMMC_PLL_BASE, 0x09)
#define reg_emmcpll_0x14                GET_REG_ADDR(EMMC_PLL_BASE, 0x14)
#define reg_emmcpll_0x15                GET_REG_ADDR(EMMC_PLL_BASE, 0x15)
#define reg_ddfset_15_00                GET_REG_ADDR(EMMC_PLL_BASE, 0x18)
#define reg_ddfset_23_16                GET_REG_ADDR(EMMC_PLL_BASE, 0x19)
#define reg_emmc_test                   GET_REG_ADDR(EMMC_PLL_BASE, 0x1a)

#define reg_emmcpll_0x1a                GET_REG_ADDR(EMMC_PLL_BASE, 0x1a)
#define reg_emmcpll_0x1c                GET_REG_ADDR(EMMC_PLL_BASE, 0x1c)
#define reg_emmcpll_0x1d                GET_REG_ADDR(EMMC_PLL_BASE, 0x1d)
#define reg_emmcpll_0x1e                GET_REG_ADDR(EMMC_PLL_BASE, 0x1e)
#define reg_emmcpll_0x1f                GET_REG_ADDR(EMMC_PLL_BASE, 0x1f)
#define reg_emmcpll_0x20                GET_REG_ADDR(EMMC_PLL_BASE, 0x20)
#define REG_EMMC_PLL_RX30               GET_REG_ADDR(EMMC_PLL_BASE, 0x30)
#define REG_EMMC_PLL_RX32               GET_REG_ADDR(EMMC_PLL_BASE, 0x32)
#define REG_EMMC_PLL_RX33               GET_REG_ADDR(EMMC_PLL_BASE, 0x33)
#define REG_EMMC_PLL_RX34               GET_REG_ADDR(EMMC_PLL_BASE, 0x34)
#define reg_emmcpll_0x45                GET_REG_ADDR(EMMC_PLL_BASE, 0x45)
#define reg_emmcpll_0x63                GET_REG_ADDR(EMMC_PLL_BASE, 0x63)
#define reg_emmcpll_0x68                GET_REG_ADDR(EMMC_PLL_BASE, 0x68)
#define reg_emmcpll_0x69                GET_REG_ADDR(EMMC_PLL_BASE, 0x69)
#define reg_emmcpll_0x6a                GET_REG_ADDR(EMMC_PLL_BASE, 0x6a)
#define reg_emmcpll_0x6b                GET_REG_ADDR(EMMC_PLL_BASE, 0x6b)

#define reg_emmcpll_0x6c                GET_REG_ADDR(EMMC_PLL_BASE, 0x6c)
#define BIT_DQS_DELAY_CELL_MASK         (BIT4|BIT5|BIT6|BIT7)
#define BIT_DQS_DELAY_CELL_SHIFT        4
#define BIT_DQS_MODE_MASK               (BIT0|BIT1|BIT2)
#define BIT_DQS_MODE_SHIFT              0
#define BIT_DQS_MODE_0T                 (0 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_0_5T               (1 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_1T                 (2 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_1_5T               (3 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_2T                 (4 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_2_5T               (5 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_3T                 (6 << BIT_DQS_MODE_SHIFT)
#define BIT_DQS_MODE_3_5T               (7 << BIT_DQS_MODE_SHIFT)


#define reg_emmcpll_0x6d                GET_REG_ADDR(EMMC_PLL_BASE, 0x6d)
#define reg_emmcpll_0x6f                GET_REG_ADDR(EMMC_PLL_BASE, 0x6f)
#define reg_emmcpll_0x70                GET_REG_ADDR(EMMC_PLL_BASE, 0x70)
#define reg_emmcpll_0x71                GET_REG_ADDR(EMMC_PLL_BASE, 0x71)
#define reg_emmcpll_0x73                GET_REG_ADDR(EMMC_PLL_BASE, 0x73)
#define reg_emmcpll_0x74                GET_REG_ADDR(EMMC_PLL_BASE, 0x74)
#define reg_emmcpll_0x7f                GET_REG_ADDR(EMMC_PLL_BASE, 0x7f)


#define BIT_TUNE_SHOT_OFFSET_MASK       (BIT4|BIT5|BIT6|BIT7)
#define BIT_TUNE_SHOT_OFFSET_SHIFT      4

//--------------------------------clock gen------------------------------------
#define BIT_FCIE_CLK_12M                0x0
#define BIT_FCIE_CLK_20M                0x1
#define BIT_FCIE_CLK_32M                0x2
#define BIT_FCIE_CLK_36M                0x3
#define BIT_FCIE_CLK_40M                0x4
#define BIT_FCIE_CLK_43_2M              0x5
#define BIT_FCIE_CLK_54M                0x6
#define BIT_FCIE_CLK_62M                0x7
#define BIT_FCIE_CLK_72M                0x8
#define BIT_FCIE_CLK_86M                0x9
//                                      0xA
#define BIT_FCIE_CLK_EMMC_PLL_1X        0xB // 8 bits macro & 32 bit macro HS200
#define BIT_FCIE_CLK_EMMC_PLL_2X        0xC // 32 bit macroDDR & HS400
#define BIT_FCIE_CLK_300K               0xD
#define BIT_FCIE_CLK_24M                0xE
#define BIT_FCIE_CLK_48M                0xF

#define eMMC_PLL_FLAG                   0x80
#define eMMC_PLL_CLK__20M               (0x01|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__27M               (0x02|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__32M               (0x03|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__36M               (0x04|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__40M               (0x05|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__48M               (0x06|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__52M               (0x07|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__62M               (0x08|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__72M               (0x09|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__80M               (0x0A|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK__86M               (0x0B|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK_100M               (0x0C|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK_120M               (0x0D|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK_140M               (0x0E|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK_160M               (0x0F|eMMC_PLL_FLAG)
#define eMMC_PLL_CLK_200M               (0x10|eMMC_PLL_FLAG)

#define CLK_SRC_CLKGEN                  0
#define CLK_SRC_EMMC_PLL_4X_CLK         4
#define CLK_SRC_EMMC_PLL_2X_CLK         2
#define CLK_SRC_EMMC_PLL_1X_CLK         1

#define eMMC_FCIE_VALID_CLK_CNT         3// FIXME

#define PLL_SKEW4_CNT                   9
#define MIN_OK_SKEW_CNT                 7

extern  U8 gau8_FCIEClkSel[];
extern  U8 gau8_eMMCPLLSel_52[];
extern  U8 gau8_eMMCPLLSel_200[]; // for DDR52 or HS200

typedef eMMC_PACK0 struct _eMMC_FCIE_ATOP_SET {
    U32 u32_ScanResult;
    U8  u8_Clk;
    U8  u8_Reg2Ch, u8_Skew4;
    U8  u8_Cell;
    U8  u8_Skew2, u8_CellCnt;
} eMMC_PACK1 eMMC_FCIE_ATOP_SET_t;

typedef eMMC_PACK0 struct _eMMC_FCIE_ATOP_SET_EXT {
    U32 au32_RXDLLResult[5];
    U8 u8_Skew4Idx;
    U8 au8_Reg2Ch[5], au8_Skew4[5];
    U8 au8_Cell[5], au8_CellCnt[5];
} eMMC_PACK1 eMMC_FCIE_ATOP_SET_EXT_t;


// ----------------------------------------------

#define eMMC_RST_L()                    {REG_FCIE_SETBIT(reg_gpio_mux_0x30, BIT_reg_emmc_rstz_en);\
	                                     REG_FCIE_SETBIT(FCIE_BOOT_CONFIG, BIT_EMMC_RSTZ_EN);\
	                                     REG_FCIE_CLRBIT(FCIE_BOOT_CONFIG, BIT_EMMC_RSTZ);}
#define eMMC_RST_H()                    {REG_FCIE_SETBIT(reg_gpio_mux_0x30, BIT_reg_emmc_rstz_en);\
                                         REG_FCIE_SETBIT(FCIE_BOOT_CONFIG, BIT_EMMC_RSTZ_EN);\
                                         REG_FCIE_SETBIT(FCIE_BOOT_CONFIG, BIT_EMMC_RSTZ);}

//--------------------------------power saving mode----------------------------
#define REG_BANK_PM_SLEEP               (0x700)
#define PM_SLEEP_REG_BASE_ADDR          GET_REG_ADDR(RIU_PM_BASE, REG_BANK_PM_SLEEP)
#define reg_pwrgd_int_glirm             GET_REG_ADDR(PM_SLEEP_REG_BASE_ADDR, 0x61)
#define BIT_PWRGD_INT_GLIRM_EN          BIT9
#define BIT_PWEGD_INT_GLIRM_MASK        (BIT15|BIT14|BIT13|BIT12|BIT11|BIT10)


//--------------------------------INV----------------------------
#define REG_ANL_SKEW4_INV                reg_emmcpll_0x6c

#define BIT_ANL_SKEW4_INV               BIT7
//=====================================================
// API declarations
//=====================================================
extern  U32 eMMC_hw_timer_delay(U32 u32us);
extern  U32 eMMC_hw_timer_sleep(U32 u32ms);

#define eMMC_HW_TIMER_HZ                12000000
#define FCIE_eMMC_DISABLE               0
#define FCIE_eMMC_BYPASS                1
#define FCIE_eMMC_SDR                   2
#define FCIE_eMMC_DDR_8BIT_MACRO        3
#define FCIE_eMMC_DDR                   FCIE_eMMC_DDR_8BIT_MACRO
#define FCIE_eMMC_HS200                 5
#define FCIE_eMMC_HS400_DS              6
#define FCIE_eMMC_HS400                 FCIE_eMMC_HS400_DS
#define FCIE_eMMC_HS400_5_1             7
#define FCIE_eMMC_HS400_AIFO_5_1        FCIE_eMMC_HS400_5_1


extern  U32 eMMC_pads_switch(U32 u32_FCIE_IF_Type);
extern  U32 eMMC_clock_setting(U16 u16_ClkParam);
extern  U32 eMMC_clock_gating(void);
extern void eMMC_set_WatchDog(U8 u8_IfEnable);
extern void eMMC_reset_WatchDog(void);
extern  uintptr_t eMMC_translate_DMA_address_Ex(dma_addr_t dma_DMAAddr, U32 u32_ByteCnt);
extern  dma_addr_t eMMC_DMA_MAP_address(uintptr_t ulongBuffer, U32 u32_ByteCnt, int mode);
extern void eMMC_DMA_UNMAP_address(dma_addr_t dma_DMAAddr, U32 u32_ByteCnt, int mode);
extern void eMMC_flush_data_cache_buffer(uintptr_t ulongDMAAddr, U32 u32_ByteCnt);
extern void eMMC_Invalidate_data_cache_buffer(uintptr_t ulongDMAAddr, U32 u32_ByteCnt);
extern void eMMC_flush_miu_pipe(void);
extern  U32 eMMC_PlatformResetPre(void);
extern  U32 eMMC_PlatformResetPost(void);
extern  U32 eMMC_PlatformInit(void);
extern  U32 eMMC_PlatformDeinit(void);
extern  U32 eMMC_CheckIfMemCorrupt(void);
extern void eMMC_DumpPadClk(void);

#define eMMC_BOOT_PART_W                BIT0
#define eMMC_BOOT_PART_R                BIT1

extern U32 eMMC_BootPartitionHandler_WR(U8 *pDataBuf, U16 u16_PartType, U32 u32_StartSector, U32 u32_SectorCnt, U8 u8_OP);
extern U32 eMMC_BootPartitionHandler_E(U16 u16_PartType);
extern U32 eMMC_hw_timer_start(void);
extern U32 eMMC_hw_timer_tick(void);
extern irqreturn_t eMMC_FCIE_IRQ(int irq, void *dummy); // [FIXME]
extern U32 eMMC_WaitCompleteIntr(uintptr_t ulongRegAddr, U16 u16_WaitEvent, U32 u32_MicroSec);
extern struct mutex FCIE3_mutex;
extern void eMMC_LockFCIE(U8 *pu8_str);
extern void eMMC_UnlockFCIE(U8 *pu8_str);
extern int  mstar_mci_Housekeep(void *pData);
extern U32  mstar_SD_CardChange(void);

//=====================================================
// partitions config
//=====================================================
// every blk is 512 bytes (reserve 2MB-64KB for internal use)
#define eMMC_DRV_RESERVED_BLK_CNT       ((0x200000-0x10000)/0x200)

#define eMMC_CIS_NNI_BLK_CNT            2
#define eMMC_CIS_PNI_BLK_CNT            2
#define eMMC_TEST_BLK_CNT               (0x100000/0x200) // 1MB

#define eMMC_CIS_BLK_0                  (64*1024/512) // from 64KB
#define eMMC_NNI_BLK_0                  (eMMC_CIS_BLK_0+0)
#define eMMC_NNI_BLK_1                  (eMMC_CIS_BLK_0+1)
#define eMMC_PNI_BLK_0                  (eMMC_CIS_BLK_0+2)
#define eMMC_PNI_BLK_1                  (eMMC_CIS_BLK_0+3)
#define eMMC_DDRTABLE_BLK_0             (eMMC_CIS_BLK_0+4)
#define eMMC_DDRTABLE_BLK_1             (eMMC_CIS_BLK_0+5)
#define eMMC_HS200TABLE_BLK_0           (eMMC_CIS_BLK_0+6)
#define eMMC_HS200TABLE_BLK_1           (eMMC_CIS_BLK_0+7)
#define eMMC_HS400TABLE_BLK_0           (eMMC_CIS_BLK_0+8)
#define eMMC_HS400TABLE_BLK_1           (eMMC_CIS_BLK_0+9)
#define eMMC_HS400EXTTABLE_BLK_0        (eMMC_CIS_BLK_0+10)
#define eMMC_HS400EXTTABLE_BLK_1        (eMMC_CIS_BLK_0+11)
#define eMMC_ALLRSP_BLK_0               (eMMC_CIS_BLK_0+12)
#define eMMC_ALLRSP_BLK_1               (eMMC_CIS_BLK_0+13)
#define eMMC_BURST_LEN_BLK_0            (eMMC_CIS_BLK_0+14)

#define eMMC_CIS_BLK_END                eMMC_BURST_LEN_BLK_0
// last 1MB in reserved area, use for eMMC test
#define eMMC_TEST_BLK_0                 (eMMC_CIS_BLK_END+1)


//=====================================================
// Driver configs
//=====================================================
#define DRIVER_NAME                     "mstar_mci"
#define eMMC_UPDATE_FIRMWARE            0

#define eMMC_ST_PLAT                    0x80000000
// [CAUTION]: to verify IP and HAL code, defaut 0
#define IF_IP_VERIFY                    0 // [FIXME] -->

// need to eMMC_pads_switch
#define IF_FCIE_SHARE_PINS              0

// need to eMMC_clock_setting
#define IF_FCIE_SHARE_CLK               0
#if defined(CONFIG_MSTAR_FCIE_HOST) && CONFIG_MSTAR_FCIE_HOST
#define IF_FCIE_SHARE_IP                1
#else
#define IF_FCIE_SHARE_IP                0
#endif

//------------------------------
#define FICE_BYTE_MODE_ENABLE           1 // always 1
#define ENABLE_eMMC_INTERRUPT_MODE      1
#define ENABLE_eMMC_RIU_MODE            0 // for debug cache issue
#if ENABLE_eMMC_RIU_MODE
#undef ENABLE_eMMC_INTERRUPT_MODE
#define ENABLE_eMMC_INTERRUPT_MODE      0
#endif

#define ENABLE_EMMC_POWER_SAVING_MODE   1
#define ENABLE_FCIE_HW_BUSY_CHECK       1

#define ENABLE_EMMC_ASYNC_IO            1
#define ENABLE_EMMC_PRE_DEFINED_BLK     1
#define ENABLE_FCIE_ADMA                1

#define ENABLE_UMA						1

                                                                                                                                    
#define eMMC_EMULATE_WR_FAIL            0
//------------------------------
// DDR48, DDR52, HS200, HS400
#define IF_DETECT_eMMC_DDR_TIMING       0 // DDR48 (digital macro)

#if defined(CONFIG_ARM64)
#define ENABLE_eMMC_ATOP                1 // DDR52 (ATOP)
#define ENABLE_eMMC_HS200               1 // HS200
#define ENABLE_eMMC_HS400               1 // HS400
#define ENABLE_eMMC_HS400_5_1           1
#elif defined(CONFIG_ARM)
#define ENABLE_eMMC_ATOP                1 // DDR52 (ATOP)
#define ENABLE_eMMC_HS200               1 // HS200
#define ENABLE_eMMC_HS400               1// HS400
#define ENABLE_eMMC_HS400_5_1           1
#endif

#if defined(CONFIG_MMC_MSTAR_EMMC_FORCE_HS200) && CONFIG_MMC_MSTAR_EMMC_FORCE_HS200
#undef  ENABLE_eMMC_HS200
#undef  ENABLE_eMMC_HS400
#undef  ENABLE_eMMC_HS400_5_1
#define ENABLE_eMMC_HS200               1
#define ENABLE_eMMC_HS400               0
#define ENABLE_eMMC_HS400_5_1           0
#endif

#if defined(CONFIG_MMC_MSTAR_EMMC_FORCE_DDR52) && CONFIG_MMC_MSTAR_EMMC_FORCE_DDR52
#undef  ENABLE_eMMC_HS200
#undef  ENABLE_eMMC_HS400
#undef  ENABLE_eMMC_HS400_5_1
#define ENABLE_eMMC_HS200               0
#define ENABLE_eMMC_HS400               0
#define ENABLE_eMMC_HS400_5_1           0
#endif

#if defined(CONFIG_MSTAR_ARM_BD_FPGA) && CONFIG_MSTAR_ARM_BD_FPGA
#undef  ENABLE_eMMC_ATOP
#undef  ENABLE_eMMC_HS200
#undef  ENABLE_eMMC_HS400
#undef  ENABLE_eMMC_HS400_5_1
#define ENABLE_eMMC_ATOP                0
#define ENABLE_eMMC_HS200               0
#define ENABLE_eMMC_HS400               0
#define ENABLE_eMMC_HS400_5_1           0
#endif


#if ENABLE_eMMC_RIU_MODE || ENABLE_eMMC_ATOP
#undef IF_DETECT_eMMC_DDR_TIMING
#define IF_DETECT_eMMC_DDR_TIMING       0
#endif

#if (defined(ENABLE_eMMC_HS200) && ENABLE_eMMC_HS200) || \
    (defined(ENABLE_eMMC_HS400) && ENABLE_eMMC_HS400)
#define ENABLE_eMMC_AFIFO               1
#endif

#define eMMC_IF_TUNING_TTABLE()         (g_eMMCDrv.u32_DrvFlag&DRV_FLAG_TUNING_TTABLE)

//------------------------------
#define eMMC_FEATURE_RELIABLE_WRITE     1
#if eMMC_UPDATE_FIRMWARE
#undef  eMMC_FEATURE_RELIABLE_WRITE
#define eMMC_FEATURE_RELIABLE_WRITE     0
#endif

//------------------------------
#define eMMC_RSP_FROM_RAM               0
#define eMMC_BURST_LEN_AUTOCFG          0

#define eMMC_HOUSEKEEP_THREAD           0
#define eMMC_PROFILE_WR                 0
#define eMMC_SCAN_HS200                 0 // for Irwin

//------------------------------
#define eMMC_SECTOR_BUF_BYTECTN         eMMC_SECTOR_BUF_16KB
extern U8 *gau8_eMMC_SectorBuf;


//=====================================================
// debug option
//=====================================================
#define eMMC_TEST_IN_DESIGN             0 // [FIXME]: set 1 to verify HW timer

#ifndef eMMC_DEBUG_MSG
#define eMMC_DEBUG_MSG                  1
#endif

/* Define trace levels. */
#define eMMC_DEBUG_LEVEL_ERROR          (1)    /* Error condition debug messages. */
#define eMMC_DEBUG_LEVEL_WARNING        (2)    /* Warning condition debug messages. */
#define eMMC_DEBUG_LEVEL_HIGH           (3)    /* Debug messages (high debugging). */
#define eMMC_DEBUG_LEVEL_MEDIUM         (4)    /* Debug messages. */
#define eMMC_DEBUG_LEVEL_LOW            (5)    /* Debug messages (low debugging). */

/* Higer debug level means more verbose */
#ifndef eMMC_DEBUG_LEVEL
#define eMMC_DEBUG_LEVEL                eMMC_DEBUG_LEVEL_WARNING
#endif

#if defined(eMMC_DEBUG_MSG) && eMMC_DEBUG_MSG
  #if eMMC_HOUSEKEEP_THREAD && eMMC_SCAN_HS200
#define eMMC_printf(fmt, arg...)        printk(fmt, ##arg)
  #else
#define eMMC_printf(fmt, arg...)        printk(KERN_ERR fmt, ##arg)
#define eMMC_printf_err(fmt, arg...)    printk(KERN_EMERG fmt, ##arg)
  #endif

#define eMMC_debug(dbg_lv, tag, str, ...)						\
    do {                                        \
        if (dbg_lv > eMMC_DEBUG_LEVEL)                      \
            break;                      \
        else {                                  \
            if (dbg_lv > eMMC_DEBUG_LEVEL_ERROR) {\
                if (tag)\
                    eMMC_printf("[ %s() Ln.%u ] ", __FUNCTION__, __LINE__); \
                eMMC_printf(str, ##__VA_ARGS__);                \
            }\
            else {\
                if (tag)                            \
                    eMMC_printf_err("[ %s() Ln.%u ] ", __FUNCTION__, __LINE__); \
                eMMC_printf_err(str, ##__VA_ARGS__);                \
            }\
        }                                   \
    } while(0)
#else
#define eMMC_printf(...)
#define eMMC_debug(enable, tag, str, ...)	do{}while(0)
#endif

#define eMMC_die(str) {eMMC_printf("eMMC Die: %s() Ln.%u, %s \n", __FUNCTION__, __LINE__, str); \
	                   panic("\n");}

#define eMMC_stop() \
	while(1)  eMMC_reset_WatchDog();

//=====================================================
// unit for HW Timer delay (unit of us)
//=====================================================
#define HW_TIMER_DELAY_1us              1
#define HW_TIMER_DELAY_5us              5
#define HW_TIMER_DELAY_10us             10
#define HW_TIMER_DELAY_100us            100
#define HW_TIMER_DELAY_500us            500
#define HW_TIMER_DELAY_1ms              (1000 * HW_TIMER_DELAY_1us)
#define HW_TIMER_DELAY_5ms              (5    * HW_TIMER_DELAY_1ms)
#define HW_TIMER_DELAY_10ms             (10   * HW_TIMER_DELAY_1ms)
#define HW_TIMER_DELAY_100ms            (100  * HW_TIMER_DELAY_1ms)
#define HW_TIMER_DELAY_500ms            (500  * HW_TIMER_DELAY_1ms)
#define HW_TIMER_DELAY_1s               (1000 * HW_TIMER_DELAY_1ms)

//=====================================================
// set FCIE clock
//=====================================================
// [FIXME] -->
#define FCIE_SLOWEST_CLK                BIT_FCIE_CLK_300K
#define FCIE_SLOW_CLK                   BIT_FCIE_CLK_12M
#define FCIE_DEFAULT_CLK                eMMC_PLL_CLK_200M // BIT_FCIE_CLK_36M

// used for test
#define eMMC_PLL_CLK_SLOW               eMMC_PLL_CLK__20M
#define eMMC_PLL_CLK_FAST               eMMC_PLL_CLK_200M


// <-- [FIXME]
//=====================================================
// transfer DMA Address
//=====================================================
#define MIU_BUS_WIDTH_BITS              4 // Need to confirm
/*
 * Important:
 * The following buffers should be large enough for a whole eMMC block
 */
// FIXME, this is only for verifing IP
#define DMA_W_ADDR                      0x40C00000
#define DMA_R_ADDR                      0x40D00000
#define DMA_W_SPARE_ADDR                0x40E00000
#define DMA_R_SPARE_ADDR                0x40E80000
#define DMA_BAD_BLK_BUF                 0x40F00000

//=====================================================
// misc
//=====================================================

extern U32 eMMC_FCIE_ResetToHS400(U8 u8_ClkParam);
extern  U32 eMMC_FCIE_DetectHS400Timing(void);
extern void eMMC_FCIE_SetDelayLatch(U32 u32Value);
extern U32 eMMC_U02_ECO_Skew4_Adjust_patch(void);

#if defined(ENABLE_eMMC_HS400_5_1) && ENABLE_eMMC_HS400_5_1
extern U32 eMMC_FCIE_DetectHS400_5_1_Timing(void);
#endif

#endif /* __eMMC_M7332_LINUX__ */
