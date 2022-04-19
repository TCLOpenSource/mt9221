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
#ifndef __HAL_SDIO_PLATFORM_H__
#define __HAL_SDIO_PLATFORM_H__

// this file is use only for mt5867 project 2020.02.20

//#include <config.h> // mboot need, kernel don't

#ifdef CONFIG_SD

	// uboot

	#include <common.h>

	#define printk printf

	#define TUNE_NOW		0x01
	#define TUNE_SILENCE	0x02
	#define TUNE_FINISH		0x00


	#define uintptr_t U32

	// use config value to define, for compatible with kernel
	#ifdef CONFIG_MIU0_BUSADDR
	#define MSTAR_MIU0_BUS_BASE CONFIG_MIU0_BUSADDR
	#endif

	#ifdef CONFIG_MIU1_BUSADDR
	#define MSTAR_MIU1_BUS_BASE CONFIG_MIU1_BUSADDR
	#endif

	//#ifdef CONFIG_MIU2_BUSADDR
	//#define MSTAR_MIU2_BUS_BASE CONFIG_MIU2_BUSADDR
	//#endif


	// uboot riu
	//#define RIU_PM_BASE			0x1F000000
	#define RIU_BASE				0x1F000000

#else

	// kernel

	#include <linux/printk.h>
	#include <linux/string.h> // memcpy
	#include <linux/delay.h> // udelay mdelay
	#include <linux/sched.h> // DECLARE_WAIT_QUEUE_HEAD & wait_event_timeout() & TASK_NORMAL

	#include <mstar/mstar_chip.h> // MSTAR_MIU0_BUS_BASE
	#include <linux/irqreturn.h> // irqreturn_t
	#include <linux/mmc/core.h> // struct mmc_command, struct mmc_data, struct mmc_request
	#include <linux/scatterlist.h> // struct scatterlist
	#include "chip_setup.h" // Chip_Clean_Cache_Range_VA_PA()
	#include <chip_int.h> // interrupt vector

	// kernel riu
	//#define RIU_PM_BASE			0xFD000000
	//#define RIU_BASE				0xFD200000

	#if defined(CONFIG_ARM)
		#define RIU_BASE                     (IO_ADDRESS(0x1F000000UL))
	#elif defined(CONFIG_ARM64)
		extern ptrdiff_t   mstar_pm_base;
		#define RIU_BASE                     ((uintptr_t)(mstar_pm_base))
	#endif

	extern unsigned long ulSDIO_CONFIG_NUM;

#endif




#ifdef CONFIG_SD

	// uboot need only

#else
	// kernel need only
	irqreturn_t HalSdio_KernelIrq(int irq, void *devid);

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//                               PLATFORM FUNCTION DEFINITION
//
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//                               DATATYPE DEFINITION
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef NULL
#define NULL    ((void*)0)
#endif

#define U64  unsigned long long
#define U32  unsigned int
#define U16  unsigned short
#define U8   unsigned char
#define S32  signed int


#define BIT00 0x0001
#define BIT01 0x0002
#define BIT02 0x0004
#define BIT03 0x0008
#define BIT04 0x0010
#define BIT05 0x0020
#define BIT06 0x0040
#define BIT07 0x0080
#define BIT08 0x0100
#define BIT09 0x0200
#define BIT10 0x0400
#define BIT11 0x0800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000


#define U32BEND2LEND(X) ( ((X&0x000000FF)<<24) + ((X&0x0000FF00)<<8) + ((X&0x00FF0000)>>8) + ((X&0xFF000000)>>24) )
#define U16BEND2LEND(X) ( ((X&0x00FF)<<8) + ((X&0xFF00)>>8) )


///////////////////////////////////////////////////////////////////////////////////////////////////
// RIU MACRO
////////////////////////////////////////////////////////////////////////////////////////////////////
#define REG_OFFSET_SHIFT_BITS           2
#define GET_REG_ADDR(x, y)              ((x)+((y) << REG_OFFSET_SHIFT_BITS))

#define FCIE_RIU_W16(addr,value)	*((volatile U16*)(uintptr_t)(addr)) = (value)
#define FCIE_RIU_R16(addr)		*((volatile U16*)(uintptr_t)(addr))
#define FCIE_RIU_W8(addr, value)	*((volatile  U8*)(uintptr_t)(addr)) = (value)
#define FCIE_RIU_R8(addr)		*((volatile  U8*)(uintptr_t)(addr))

// read modify write 16 bits register macro
#define FCIE_RIU_16_ON(addr,value)	FCIE_RIU_W16(addr, FCIE_RIU_R16(addr)|(value))
#define FCIE_RIU_16_OF(addr,value)	FCIE_RIU_W16(addr, FCIE_RIU_R16(addr)&(~(value)))
#define FCIE_RIU_W1C(addr, val)		FCIE_RIU_W16(addr, FCIE_RIU_R16(addr)&(val))

#define FCIE_RIU_8_ON(addr,value)	FCIE_RIU_W8(addr, FCIE_RIU_R8(addr)|(value))
#define FCIE_RIU_8_OF(addr,value)	FCIE_RIU_W8(addr, FCIE_RIU_R8(addr)&(~(value)))


////////////////////////////////////////////////////////////////////////////////////////////////////
// RIU BASE ADDRESS
////////////////////////////////////////////////////////////////////////////////////////////////////

#define RIU_BANK_2_BASE(BANK)		(RIU_BASE+(BANK<<9))
// register num per bank --> 0x80 = 128
// register addr offset  --> 4

////////////////////////////////////////////////////////////////////////////////////////////////////
// RIU BANK DEFINITION
////////////////////////////////////////////////////////////////////////////////////////////////////
#define RIU_BANK_CLKGEN0		0x100BUL
#define RIU_BANK_SDIO0			0x1119UL
#define RIU_BANK_SDIO1			0x111AUL
#define RIU_BANK_SDIO2			0x111BUL
#define	RIU_BANK_CHIPTOP		0x101EUL
#define	RIU_BANK_CHIP_GPIO		0x1210UL

#define RIU_BASE_CLKGEN0		RIU_BANK_2_BASE(RIU_BANK_CLKGEN0)
#define RIU_BASE_SDIO0			RIU_BANK_2_BASE(RIU_BANK_SDIO0) // main bank
#define RIU_BASE_SDIO1			RIU_BANK_2_BASE(RIU_BANK_SDIO1) // CIFD + CRC
#define RIU_BASE_SDIO2			RIU_BANK_2_BASE(RIU_BANK_SDIO2) // power save bank
#define RIU_BASE_CHIPTOP		RIU_BANK_2_BASE(RIU_BANK_CHIPTOP)
#define RIU_BASE_CHIP_GPIO		RIU_BANK_2_BASE(RIU_BANK_CHIP_GPIO)

////////////////////////////////////////////////////////////////////////////////////////////////////

#define FCIE_REG_BASE_ADDR              (RIU_BASE_SDIO0)
#define FCIE_CMDFIFO_BASE_ADDR          (RIU_BASE_SDIO0 + (0x20<<2)) // CIFC command FIFO
#define FCIE_CIFD_FIFO_W		(RIU_BASE_SDIO1)
#define FCIE_CIFD_FIFO_R		(RIU_BASE_SDIO1 + (0x20<<2))
#define FCIE_CRC_BUF			(RIU_BASE_SDIO1 + (0x40<<2))
#define FCIE_POWER_SAVE_MODE_BASE	(RIU_BASE_SDIO2)

#define SD_USE_FCIE5		1
#define SDIO_D1_INTR_VER	2

#include "hal_reg_sdio.h"

#define RIU_UNIT_SHIFT           2


////////////////////////////////////////////////////////////////////////////////////////////////////
// RIU_BANK_CLKGEN0 0x100B
////////////////////////////////////////////////////////////////////////////////////////////////////
#define REG_CLK_SDIO			(RIU_BASE_CLKGEN0+(0x69<<RIU_UNIT_SHIFT))

#define BIT_SDIO_CLK_GATING		(BIT00)
#define BIT_SDIO_CLK_INVERSE		(BIT01)
#define BIT_CLKGEN_SDIO_MASK		(BIT05|BIT04|BIT03|BIT02)
#define BIT_SDIO_CLK_SRC_SEL		(BIT06) // 0: clk_xtal 12M, 1: clk_sdio_p1


////////////////////////////////////////////////////////////////////////////////////////////////////
// RIU_BANK_CHIPTOP 0x101E
////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct _RIU_CHIPTOP { // [101Eh] PAD_CHIPTOP_BASE

	//h0000~h000B
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32; U32:32; U32:32; U32:32;
	//h000C
	U32			:8;
	U32 reg_pcm_pe		:4;
	U32			:20;
	//h000D~h000F
	U32:32; U32:32; U32:32;
	//h0010
	U32:32;
	//h0011
	U32			:6;
	U32 reg_sdio_config	:2;
	U32			:24;
	//h0012
	U32 reg_test_in_mode		:3;
	U32				:1; // 3
	U32 reg_test_out_mode		:3;
	U32				:25;
	//h0013
	U32				:8;
	U32 reg_ej_config_x		:3;
	U32				:21;
	//h0014~h001A
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	//h001B
	U32 reg_od2nduart		:3;
	U32				:9; // 3~11
	U32 reg_odfastuart		:4;
	U32				:16;
	//h001C
	U32:32;
	//h001D
	U32				:9; // 0~8
	U32 reg_sd_config_eco		:1;
	U32				:22;
	//h001E~h002D
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	//h002E~h002F
	U32:32; U32:32;
	//h0030
	U32 reg_seconduartmode		:3;
	U32				:9; // 3~11
	U32 reg_fastuartmode		:4;
	U32				:16;
	//h0031~h0032
	U32:32; U32:32;
	//h0033
	U32 reg_nand_cs1_en		:2;
	U32				:30;
	//h0034~h0043
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	//h0044~h004D
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32; U32:32;
	//h004E
	U32				:12;
	U32 reg_pcmadconfig		:2;
	U32				:18;
	//h004F
	U32				:2;
	U32 reg_emmc_rstz_en		:1;
	U32				:29;
	//h0050
	U32				:7;
	U32 reg_nand_mode		:3; // 7~9
	U32				:5; // 10~14
	U32 reg_all_pad_in		:1;
	U32				:16;
	//h0051~h005A
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32; U32:32;
	//h005B
	U32				:8;
	U32 reg_ld_spi3_config		:3;
	U32				:21;
	//h005C~h0060
	U32:32; U32:32; U32:32; U32:32; U32:32;
	//h0061~h006D
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32; U32:32; U32:32; U32:32; U32:32;
	//h006E
	U32				:6;
	U32 reg_emmc_config		:1;
	U32				:25;
	//h006F~h0077
	U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32; U32:32;
	U32:32;
	//h0078
	U32 reg_spi_nand		:1;
	U32 reg_spi_nor			:1;
	U32				:30;

} RIU_CHIPTOP;

////////////////////////////////////////////////////////////////////////////////////////////////////
// RIU_BANK_CHIP_GPIO 0x1210
////////////////////////////////////////////////////////////////////////////////////////////////////
#define REG_CHIP_GPIO_50h		(RIU_BASE_CHIP_GPIO+(0x50<<RIU_UNIT_SHIFT))
#define REG_CHIP_GPIO_52h		(RIU_BASE_CHIP_GPIO+(0x52<<RIU_UNIT_SHIFT))
#define REG_MIIC_MODE			(RIU_BASE_CHIP_GPIO+(0x71<<RIU_UNIT_SHIFT))

//0x50
#define SDIO_MODE_1_DATA_DRV_MSK	(0x00FF)
#define SDIO_MODE_1_DATA_DRV_01		(BIT06|BIT04|BIT02|BIT00)
#define SDIO_MODE_1_DATA_DRV_10		(BIT07|BIT05|BIT03|BIT01)
#define SDIO_MODE_1_DATA_DRV_11		(0x00FF)

//0x52
#define SDIO_CLK_CMD_DRV_MSK		(BIT07|BIT06)
#define SDIO_CLK_CMD_DRV_01		(BIT06)
#define SDIO_CLK_CMD_DRV_10		(BIT07)
#define SDIO_CLK_CMD_DRV_11		(BIT07|BIT06)

//0x71
#define REG_MIIC_MODE_MSK		(BIT10|BIT09|BIT08)
#define REG_MIIC_MODE_2			(      BIT09      )
#define REG_MIIC_MODE_3			(      BIT09|BIT08)


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//                               EXTERN GLOBAL FUNCTION
//
///////////////////////////////////////////////////////////////////////////////////////////////////


#define SDIO_MODE_GPIO_PAD_BPS			1
#define SDIO_MODE_GPIO_PAD_SDR			2
#define SDIO_MODE_UNKNOWN			7

//#define SDIO_PAD_SDR104			SDIO_MODE_32BITS_MACRO_SDR104
//#define SDIO_PAD_SDR50			SDIO_MODE_32BITS_MACRO_SDR104
//#define SDIO_PAD_DDR50			SDIO_MODE_8BITS_MACRO_DDR
#define SDIO_PAD_SDR25				SDIO_MODE_GPIO_PAD_SDR
#define SDIO_PAD_SDR12				SDIO_MODE_GPIO_PAD_SDR

///////////////////////////////////////////////////////////////////////////////////////////////////
// interrutp vector number
// check mstar2/hal/[project]/cpu/chip_int.h
//       mstar2/hal/[project]/cpu/arm64/chip_int.h
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SDIO_INT_VECTOR E_IRQ_SDIO


///////////////////////////////////////////////////////////////////////////////////////////////////
// new feature or patch different with other project
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SDIO_NEW_R2N_PATCH // R2N mode new patch from manhattan 2015.06.09
#define SDIO_R3_CRC_PATCH // response CRC false alarm patch 2015.06.09, SDIO has patch only

#define SDIO_ERRDET 			1

#if (defined SDIO_ERRDET) && (SDIO_ERRDET==1)
#define SDIO_ERRDET_RSP_TO		1
#define SDIO_ERRDET_RSP_CRC_ERR	1
#define SDIO_ERRDET_R_CRC_ERR	1
#define SDIO_ERRDET_W_CRC_STS	1
#define SDIO_ERRDET_R_SBIT_TO	1 // Manhattan fix counter 2 issue
#define SDIO_ERRDET_W_MIU_TO	1 //
#endif

#ifdef CONFIG_SD

	// uboot
	//#define ENABLE_SDIO_INTERRUPT_MODE	1
	//#define ENABLE_SDIO_ADMA			1

#else

	// kernel
	#define ENABLE_SDIO_INTERRUPT_MODE	1
	#define ENABLE_SDIO_ADMA			1

#endif


typedef struct _SKEWER {

	unsigned int u32LatchOKStart;
	unsigned int u32LatchOKEnd;
	unsigned int u32LatchBest;
	unsigned char u8_edge;

} SKEWER;

typedef struct _GpioReg
{
	unsigned int option;
	unsigned int bank;
	unsigned int offset;
	unsigned int bits;

} GpioReg;

#define CHKPAD(MODE, CONFLICT) do { if(CONFLICT) printk(LIGHT_RED"SdErr: sd_mode=%ld conflict w/ "#CONFLICT"!\n"NONE, MODE); } while(0)

//#define CONFIG_MIU0_BUSADDR 0x20000000
//#define CONFIG_MIU1_BUSADDR 0xA0000000

U32	HalSdio_SlectBestSkew4(U32 u32_Candidate, SKEWER * pSkewer);
void	HalSdio_SetTuning(U8 u8Type, U8 u8Count);
void	HalSdio_SetTriggerLevel(U8 u8STrigLevel);
void	HalSdio_ResetIP(void);
void	HalSdio_Platform_InitChiptop(void);
U8	HalSdio_GetPadType(void);
void	HalSdio_SwitchPad(unsigned char);
void	HalSdioDelayMs(U32 u32Ms);
void	HalSdioDelayUs(U32 u32Us);
void	HalSdio_DelaySecond(U32 u32Sec);
U32	HalSdio_SetClock(U32 u32Clock);
void	HalSdio_DumpDebugBus(void);
U8	HalSdio_GetSBootGPIOConfig(void);
S32	HalSdio_GetCardDetect(void);
S32	HalSdio_GetWriteProtect(void);
void	HalSdio_SetCardPower(U8 u8OnOff);
void	HalSdio_SetDrivingStrength(U16 u16DrivingLevel);
int	HalSdio_ParseDTS(void);

#endif // #ifndef __HAL_SDIO_PLATFORM_H__

