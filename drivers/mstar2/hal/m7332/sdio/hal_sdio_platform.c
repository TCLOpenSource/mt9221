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

//#include <config.h>
//#include "drvFCIE_config.h"
#include <linux/of.h> // of_property_read_u32()



//#include <stdio.h>
//#include <mmc.h>
//#include "drvFCIE5.h"

struct mmc_host;
#include "hal_sdio.h"
extern unsigned char sd_ddr_mode;
extern void HalSdio_DumpDebugBus(void);
extern unsigned char gu8SdioSilenceTuning;

/**************************************
* Config definition
**************************************/
#define DBG_CR_CLK(MSG)             //MSG
#define DBG_CR_PAD(MSG)             //MSG

/**************************************
* Local function definition
**************************************/
void HalSdioDelayUs(U32 u32Us);

static U16 gu16Signature = 0;

static int CDZ_option = 0;
static int CdzGpioBank = 0;
static int CdzGpioOffset = 0;
static int CdzGpioBits = 0;

static int pwr_option = 0;
static int pwrGpioBank = 0;
static int pwrGpioOffset = 0;
static int pwrGpioBits = 0;

static int wp_option = 0;
static int wpGpioBank = 0;
static int wpGpioOffset = 0;
static int wpGpioBits = 0;

//static int sdio_io_voltage_type = 0;

static int sdio_fast_connect = 0;

#define	SBOOT_SIGNATURE		0x38
#define	SBOOT_SIGNATURE_V2	0x24
#define	SBOOT_SIGNATURE_V3	0x61
#define	SBOOT_SIGNATURE_DTS	0x18

#define CD_VIRTUAL_HOTPLUG	0
#define CD_INSERT_L			1
#define CD_INSERT_H			2
#define CD_ALWAYS_INSERT	3

#define PC_ALWAYS_ON		0
#define PC_POWERON_L		1
#define PC_POWERON_H		2

#define WP_PROTECT_L		1
#define WP_PROTECT_H		2

#define BITNUM(X)  (X&0x00000001? 0:X&0x00000002? 1:X&0x00000004? 2:X&0x00000008? 3: \
					X&0x00000010? 4:X&0x00000020? 5:X&0x00000040? 6:X&0x00000080? 7: \
					X&0x00000100? 8:X&0x00000200? 9:X&0x00000400?10:X&0x00000800?11: \
					X&0x00001000?12:X&0x00002000?13:X&0x00004000?14:X&0x00008000?15: \
					X&0x00010000?16:X&0x00020000?17:X&0x00040000?18:X&0x00080000?19: \
					X&0x00100000?20:X&0x00200000?21:X&0x00400000?22:X&0x00800000?23: \
					X&0x01000000?24:X&0x02000000?25:X&0x04000000?26:X&0x08000000?27: \
					X&0x10000000?28:X&0x20000000?29:X&0x40000000?30:X&0x80000000?31:3697)


/**************************************
* Global function definition
**************************************/
void HalSdio_ResetIP(void)
{
	U16 u16Reg, u16Cnt;

	FCIE_RIU_W16(FCIE_SD_CTRL, 0); // clear job start for safety

	//printk(LIGHT_CYAN"sdio reset\n"NONE);

	FCIE_RIU_W16(FCIE_MIE_FUNC_CTL, BIT_SDIO_EN);

	FCIE_RIU_16_OF(FCIE_RST, BIT_FCIE_SOFT_RST_n); /* active low */

	// SDIO reset - wait
	u16Cnt=0;

	do
	{
		u16Reg = FCIE_RIU_R16(FCIE_RST);

		HalSdioDelayUs(1);

		if(0x1000 == u16Cnt++)
		{
			printk("SD Err: SDIO Reset fail!! FCIE_RST = %04Xh\n", u16Reg);
			while(1);
		}

	} while (BIT_RST_STS_MASK  != (u16Reg  & BIT_RST_STS_MASK));

	FCIE_RIU_16_ON(FCIE_RST, BIT_FCIE_SOFT_RST_n);

	u16Cnt=0;

	do
	{
		u16Reg = FCIE_RIU_R16(FCIE_RST);
		//printk("FCIE_RST = %04Xh\n", u16Reg);

		if(0x1000 == u16Cnt++)
		{
			printk("SD Err: SDIO Reset fail2:h \n");
			return ;
		}

		HalSdioDelayUs(1);


	} while (0  != (u16Reg  & BIT_RST_STS_MASK));

	//printk("ok\n");

}


void HalSdio_Platform_InitChiptop(void)
{
	// actually it is GPIO_FUNC_MUX bank
	FCIE_RIU_16_OF(GPIO_FUNC_MUX_70h, REG_ALLPAD_IN);

	FCIE_RIU_16_OF(GPIO_FUNC_MUX_34h, REG_SDIO_CONFIG);

		FCIE_RIU_16_ON(GPIO_FUNC_MUX_34h, REG_SDIO_MODE_1);
		// 0x00
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_00h)&REG_TS2CONFIG)==REG_TS2CONFIG_1)
			printk(LIGHT_RED"SdioErr, reg_ts2config = 1 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_00h)&REG_TS2CONFIG)==REG_TS2CONFIG_2)
			printk(LIGHT_RED"SdioErr, reg_ts2config = 2 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_00h)&REG_TS2CONFIG)==REG_TS2CONFIG_3)
			printk(LIGHT_RED"SdioErr, reg_ts2config = 3 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_00h)&REG_TS2CONFIG)==REG_TS2CONFIG_4)
			printk(LIGHT_RED"SdioErr, reg_ts2config = 4 conflict with sdio_mode\n"NONE);
		// 0x08
		if(FCIE_RIU_R16(GPIO_FUNC_MUX_08h)&REG_PCM2CTRLCONFIG)
			printk(LIGHT_RED"SdioErr, reg_pcm2ctrlconfig conflict with sdio_mode\n"NONE);
		if(FCIE_RIU_R16(GPIO_FUNC_MUX_08h)&REG_PCM2_CDN_CONFIG)
			printk(LIGHT_RED"SdioErr, reg_pcm2_cdn_config conflict with sdio_mode\n"NONE);
		// 0x16
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_16h)&REG_I2S_BT_MD)==REG_I2S_BT_MD_1)
			printk(LIGHT_RED"SdioErr, reg_i2s_bt_md = 1 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_16h)&REG_I2S_BT_MD)==REG_I2S_BT_MD_2)
			printk(LIGHT_RED"SdioErr, reg_i2s_bt_md = 2 conflict with sdio_mode\n"NONE);
		// 0x21
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_21h)&REG_SECOND_UART_MODE)==REG_SECOND_UART_MODE_2)
			printk(LIGHT_RED"SdioErr, reg_sencond_uart_mode = 2 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_21h)&REG_OD2NDUART)==REG_OD2NDUART_2)
			printk(LIGHT_RED"SdioErr, reg_od2nuart = 2 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_21h)&REG_THIRDUART_MD)==REG_THIRDUART_MD_2)
			printk(LIGHT_RED"SdioErr, reg_thirduart = 2 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_21h)&REG_OD3NDUART)==REG_OD3NDUART_2)
			printk(LIGHT_RED"SdioErr, reg_od3nduart = 2 conflict with sdio_mode\n"NONE);
		// 0x28
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_28h)&REG_MIIC_MD)==REG_MIIC_MD_3)
			printk(LIGHT_RED"SdioErr, reg_miic_md = 3 conflict with sdio_mode\n"NONE);
		// 0x2C
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_2Ch)&REG_MSPI3_CONFIG)==REG_MSPI3_CONFIG_1)
			printk(LIGHT_RED"SdioErr, reg_mspi3 = 1 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_2Ch)&REG_MSPI3_CONFIG)==REG_MSPI3_CONFIG_2)
			printk(LIGHT_RED"SdioErr, reg_mspi3 = 2 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_2Ch)&REG_MSPI3_CONFIG)==REG_MSPI3_CONFIG_3)
			printk(LIGHT_RED"SdioErr, reg_mspi3 = 3 conflict with sdio_mode\n"NONE);
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_2Ch)&REG_MSPI3_CONFIG)==REG_MSPI3_CONFIG_4)
			printk(LIGHT_RED"SdioErr, reg_mspi3 = 4 conflict with sdio_mode\n"NONE);
		// 0x41
		if((FCIE_RIU_R16(GPIO_FUNC_MUX_41h)&REG_TCON_CONFIG5)==REG_TCON_CONFIG5_1)
			printk(LIGHT_RED"SdioErr, reg_tcon_config5 = 1 conflict with sdio_mode\n"NONE);
		// 0x42
		if(FCIE_RIU_R16(GPIO_FUNC_MUX_42h)&REG_TCON_CONFIG8)
			printk(LIGHT_RED"SdioErr, reg_tcon_congig8 conflict with sdio_mode\n"NONE);
		if(FCIE_RIU_R16(GPIO_FUNC_MUX_42h)&REG_TCON_CONFIG9)
			printk(LIGHT_RED"SdioErr, reg_tcon_congig9 conflict with sdio_mode\n"NONE);
		if(FCIE_RIU_R16(GPIO_FUNC_MUX_42h)&REG_TCON_CONFIG10)
			printk(LIGHT_RED"SdioErr, reg_tcon_congig10 conflict with sdio_mode\n"NONE);


	FCIE_RIU_16_ON(PCM_CS_34h,BIT04);
	FCIE_RIU_16_OF(PCM_CS_34h,BIT05);

	FCIE_RIU_16_ON(PCM_CS_33h,BIT04);
	FCIE_RIU_16_OF(PCM_CS_33h,BIT05);
	FCIE_RIU_16_ON(PCM_CS_33h,BIT10);

	FCIE_RIU_16_OF(TS_CS_28h,BIT04);
	FCIE_RIU_16_ON(TS_CS_28h,BIT05);
	FCIE_RIU_16_OF(TS_CS_29h,BIT04);
	FCIE_RIU_16_ON(TS_CS_29h,BIT05);
	FCIE_RIU_16_OF(TS_CS_2Ah,BIT04);
	FCIE_RIU_16_ON(TS_CS_2Ah,BIT05);
	FCIE_RIU_16_OF(TS_CS_20h,BIT04);
	FCIE_RIU_16_ON(TS_CS_20h,BIT05);
}


#if !defined FPGA_BOARD || FPGA_BOARD==0

const U8 SdioClockIdxNum = 7;

const U32 SDIO_CLOCK[] =
{
    48000, //  0
    43200, //  1
    40000, //  2
    36000, //  3
    32000, //  4
    20000, //  5
    12000, //  6
      300, //  7
};

// find clock close to target but not over

U32 HalSdio_SetClock(U32 u32Clock)
{
	U8 u8ClockSlector;
	/*static U32 u32_OldClock=0xFFFFFFFF;

	//printk("HalSdio_SetClock(%ld)\n", u32Clock);

	if(u32_OldClock == u32Clock)
		return 0;
	else
		u32_OldClock = u32Clock;*/

	FCIE_RIU_16_ON(REG_CLK_SDIO, BIT00); // turn on clock gating

	if(u32Clock>1000) {
		DBG_CR_CLK(printk("Set SDIO clock as %d.%d MHz, ", u32Clock/1000, (u32Clock%1000)/100 ) );
	} else {
		DBG_CR_CLK(printk("Set SDIO clock as %d KHz, ", u32Clock));
	}

	for(u8ClockSlector=0; u8ClockSlector<=SdioClockIdxNum; u8ClockSlector++)
	{
		if (SDIO_CLOCK[u8ClockSlector] <= u32Clock) {
			break;
		}
	}

	if (u8ClockSlector>SdioClockIdxNum) {
		printk("Error!!! Can not find proper clock!\r\n");
		while(1);
		return 0x01;
	}

	if (u32Clock>1000) {
		DBG_CR_CLK(printk("select SDIO clock as %d.%d MHz\r\n", SDIO_CLOCK[u8ClockSlector]/1000, (SDIO_CLOCK[u8ClockSlector]%1000)/100));
	} else {
		DBG_CR_CLK(printk("select SDIO clock as %d KHz\r\n", SDIO_CLOCK[u8ClockSlector]));
	}

	FCIE_RIU_16_ON(REG_CLK_SDIO, BIT06);
	FCIE_RIU_16_OF(REG_CLK_SDIO, BIT05+BIT04+BIT03+BIT02); // mask all clock select

	//printk("switch to clock: %d\n", u8ClockSlector);

	switch(u8ClockSlector)
	{
	case 0: // 48M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0xF<<2);
		break;

	case 1: // 43.2M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0x5<<2);
		break;

	case 2: // 40M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0x4<<2);
		break;

	case 3: // 36M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0x3<<2);
		break;

	case 4: // 32M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0x2<<2);
		break;

	case 5: // 20M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0x1<<2);
		break;

	case 6: // 12M
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0xE<<2);
		break;

	case 7: // 300K
		FCIE_RIU_16_ON(REG_CLK_SDIO, 0xD<<2);
		break;

	default:
		printk("SDIO Err: wrong clock selector!\n");
		while(1);
		break;
	}

	DBG_CR_CLK(printk("REG_CLK_SDIO = 0x%04X\r\n", FCIE_RIU_R16(REG_CLK_SDIO)));

	FCIE_RIU_16_OF(REG_CLK_SDIO, BIT00); // turn off clock gating

	return SDIO_CLOCK[u8ClockSlector];

}

//static U32 gu32SdClock = 0;

#define BUS_SPEED_SDR12		1
#define BUS_SPEED_SDR25		2
#define BUS_SPEED_DDR50		3
#define BUS_SPEED_SDR50		4
#define BUS_SPEED_SDR104	5

//static U32 gu8BusSpeed = BUS_SPEED_SDR12;

#else

U8 HalSdio_SetSdioClock(U32 u32Clock)
{
	if (u32Clock < 400)
	{
		printk("SDIO FPGA clock 187.5KHz\n");
		FCIE_RIU_16_ON(R_SDIO_PLL_0x1D, BIT00);
	}
	else
	{
		printk("SDIO FPGA clock 1.5MHz\n");
		FCIE_RIU_16_OF(R_SDIO_PLL_0x1D, BIT00);
	}
	FCIE_RIU_16_ON(FCIE_SD_MODE, BIT_CLK_EN); // enable clk

	return 0;
}

#endif

static U8 u8CurrentPadType = SDIO_MODE_UNKNOWN;

U8 HalSdio_GetPadType(void)
{
	return u8CurrentPadType;
}

void HalSdio_SwitchPad(unsigned char u32Mode)
{
	//printk("switch pad %d, current pad type = %d\n", u32Mode, u8CurrentPadType);

	// chiptop
	//HalSdio_Platform_InitChiptop(); // eMMC & nand driver should NOT touch sdio related chiptop regs.

	// sdio
	FCIE_RIU_16_OF(FCIE_DDR_MODE, BIT_FALL_LATCH|BIT_PAD_IN_SEL_SD|BIT_32BIT_MACRO_EN|BIT_DDR_EN|BIT_8BIT_MACRO_EN|BIT_CIF_PAD_IN_RDY_SEL|BIT_CIF_PRE_FULL_SEL_MSK);

	switch(u32Mode)
	{
	///////////////////////////////////////////////////////////////////////////////////////////
	case SDIO_MODE_GPIO_PAD_BPS:
		DBG_CR_PAD(printk(LIGHT_CYAN"SDIO_MODE_GPIO_PAD_BPS\n"NONE));
		u8CurrentPadType = SDIO_MODE_GPIO_PAD_BPS;

		break;

	///////////////////////////////////////////////////////////////////////////////////////////
	case SDIO_MODE_GPIO_PAD_SDR:
		DBG_CR_PAD(printk(LIGHT_CYAN"SDIO_MODE_GPIO_PAD_SDR\n"NONE));
		u8CurrentPadType = SDIO_MODE_GPIO_PAD_SDR;
		FCIE_RIU_16_ON(FCIE_DDR_MODE, BIT_FALL_LATCH|BIT_PAD_IN_SEL_SD);
		// New patch for feedback clock reflex
		FCIE_RIU_16_ON(FCIE_DDR_MODE, BIT_FEEDBACK_CLK2);
		break;

	default:
		u8CurrentPadType = SDIO_MODE_UNKNOWN;
		printk(LIGHT_CYAN"SdErr: wrong parameter for switch pad func\n"NONE);
		break;
	}

}


U8 HalSdio_Platform_ClearEvent(U16 nReq)
{
	U16  u16Tmp;

	u16Tmp = 0x0080;
	while((FCIE_RIU_R16(FCIE_MIE_EVENT)& nReq) != 0x00)
	{
		FCIE_RIU_W16(FCIE_MIE_EVENT, nReq); // write 1 clear register
		if (u16Tmp==0)
		{
			printk("Error!!! Can not clear MIE event.\r\n");
			return(1);
		}
		else
		{
			u16Tmp--;
		}
	}

	return 0;
}

#if defined(ENABLE_SDIO_INTERRUPT_MODE)&&ENABLE_SDIO_INTERRUPT_MODE

///////////////////////////////////////////////////////////////////////////////////////////////////

static DECLARE_WAIT_QUEUE_HEAD(sdio_mie_wait);

static volatile U16 sdio_event;

typedef enum
{
	IRQ_TYPE_NONE	= 0,
	IRQ_TYPE_EVENT	= 1,
	IRQ_TYPE_D1INT	= 2,

} E_IRQ_TYPE;


void HalSdio_ClearWaitQueue(void)
{
	sdio_event = 0;
}

E_IRQ_TYPE HalSdio_SaveMieEvent(struct mmc_host *host)
{
#if defined ENABLE_SDIO_D1_INTERRUPT && ENABLE_SDIO_D1_INTERRUPT

	U16 u16Reg = FCIE_RIU_R16(FCIE_MIE_EVENT);

	if(u16Reg & BIT_SDIO_INT)
	{
		mmc_signal_sdio_irq(host);
		FCIE_RIU_W16(FCIE_MIE_EVENT, BIT_SDIO_INT); // W1C event
		return IRQ_TYPE_D1INT;
	}
	else

#endif

	{
		//sdio_event |= u16Reg; // summary all mie event

		#if defined ENABLE_SDIO_D1_INTERRUPT && ENABLE_SDIO_D1_INTERRUPT
			sdio_event = FCIE_RIU_R16(FCIE_MIE_EVENT) & FCIE_RIU_R16(FCIE_MIE_INT_EN) & (~BIT_SDIO_INT);
		#else
			sdio_event = FCIE_RIU_R16(FCIE_MIE_EVENT) & FCIE_RIU_R16(FCIE_MIE_INT_EN);
		#endif

		if(sdio_event & BIT_DMA_END)
		{
			FCIE_RIU_16_OF(FCIE_MIE_INT_EN, BIT_DMA_END);
		}

		if(sdio_event & BIT_SD_CMD_END)
		{
			FCIE_RIU_16_OF(FCIE_MIE_INT_EN, BIT_SD_CMD_END);
		}

		if(sdio_event & BIT_BUSY_END_INT)
		{
			FCIE_RIU_16_OF(FCIE_MIE_INT_EN, BIT_BUSY_END_INT);
		}

		if(sdio_event & BIT_ERR_STS)
		{
			FCIE_RIU_16_OF(FCIE_MIE_INT_EN, BIT_ERR_STS);
		}

		return IRQ_TYPE_EVENT;
	}

	return IRQ_TYPE_NONE;

}


irqreturn_t HalSdio_KernelIrq(int irq, void *devid)
{
	irqreturn_t irq_t = IRQ_NONE;
	struct mmc_host *host = devid;
	E_IRQ_TYPE eIrqType;

	//printk("SDIO IRQ EV_%04Xh, IE_%04Xh\n", FCIE_RIU_R16(FCIE_MIE_EVENT), FCIE_RIU_R16(FCIE_MIE_INT_EN));

	eIrqType = HalSdio_SaveMieEvent(host);

	if(eIrqType ==IRQ_TYPE_EVENT)
	{
		wake_up(&sdio_mie_wait);
		irq_t = IRQ_HANDLED;
	}
	else if(eIrqType ==IRQ_TYPE_D1INT) // no need to wake up wait queue head
	{
		irq_t = IRQ_HANDLED;
	}

	return irq_t;
}


E_IO_STS HalSdio_WaitMieEvent(U16 u16ReqVal, U32 u32WaitMs)
{
	unsigned long timeout;

	timeout = msecs_to_jiffies(u32WaitMs+10);

	if(wait_event_timeout(sdio_mie_wait, (sdio_event==u16ReqVal) || ( sdio_event&BIT_ERR_STS), timeout))
	{
		if(sdio_event&BIT_ERR_STS)
		{
			return IO_ERROR_DETECT;
		}
		return IO_SUCCESS;
	}
	else
	{
		printk("wait sdio mie evnet to req %04Xh, event = %04Xh\n", u16ReqVal, sdio_event);
		return IO_TIME_OUT;
	}

	return IO_SUCCESS;
}

void HalSdio_SetupD1IntrEnable(U8 u8Enable)
{
	if(u8Enable)
	{
		FCIE_RIU_16_ON(FCIE_MIE_INT_EN, BIT_SDIO_INT);
	}
	else
	{
		FCIE_RIU_16_OF(FCIE_MIE_INT_EN, BIT_SDIO_INT);
	}
}

void HalSdio_EnableD1Interrupt(U8 u8Enable)
{
	FCIE_RIU_16_ON(FCIE_SDIO_MOD, BIT_SDIO_DET_INT_SRC); // level detect

	if(u8Enable)
	{
		FCIE_RIU_16_ON(SDIO_INTR_DET, BIT_SDIO_DET_ON); // enable SDIO interrupt
	}
	else
	{
		FCIE_RIU_16_OF(SDIO_INTR_DET, BIT_SDIO_DET_ON); // hardware auto clear detect_on
														// sdio wifi driver might disable intr while suspend
	}
}

#else

E_IO_STS HalSdio_WaitMieEvent(U16 u16ReqVal, U32 u32WaitMs)
{
	U32 u32Count = 0;
	//u32WaitMs *= 100;
	U16 u16Event;

	while(1)
	{
		u16Event = FCIE_RIU_R16(FCIE_MIE_EVENT);

		if(u16Event&BIT_ERR_STS)
		{
			/*if(!gu8SdioSilenceTuning)
			{
				printk("\33[1;31mSDIO ErrDet, SD_STS = %04Xh\33[m\n", FCIE_RIU_R16(FCIE_SD_STATUS));
			}*/
			HalSdio_Platform_ClearEvent(BIT_ERR_STS);
			return IO_ERROR_DETECT;
		}
		else if((u16Event&u16ReqVal)==u16ReqVal)
		{
			//printk("Got event %04X\n", u16Event);
			break;
		}

		HalSdioDelayUs(1000);

		u32Count++;

		if(u32Count>u32WaitMs)
		{
			printk("u32Counter=%d\r\n", u32Count);
			printk("\r\n");
			printk("------------------------------------------\r\n");
			printk("ERROR!!! MIE EVENT TIME OUT!!!\n");
			printk("request event = %04Xh, event = %04Xh\n", u16ReqVal, u16Event);
			printk("------------------------------------------\r\n");

			if(!gu8SdioSilenceTuning)
			{
				HalSdio_DumpRegister();
				HalSdio_DumpDebugBus();
				//while(1);
			}

			return(IO_TIME_OUT);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// Clear mie event
	// Only can clear request value,
	// because read operation might enable command and data transfer at the same time

	if (HalSdio_Platform_ClearEvent(u16ReqVal))
	{
		return (IO_TIME_OUT);
	}

	return(IO_SUCCESS);
} // wait MIE Event End

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

void HalSdioDelayUs(U32 u32Us)
{
	U32 u32_i, u32Ms;

	if(u32Us<1000)
	{
		udelay(u32Us);
	}
	else
	{
		u32Ms = u32Us/1000;
		for(u32_i=0; u32_i<u32Ms; u32_i++)
			udelay(1000);
	}
}

void HalSdioDelayMs(U32 u32Ms)
{
	U32 u32_i;

	for(u32_i=0; u32_i<u32Ms; u32_i++)
		udelay(1000);
}

void HalSdio_DelaySecond(U32 u32Sec)
{
	U32 u32_i;

	for(u32_i=0; u32_i<u32Sec; u32_i++)
		HalSdioDelayMs(1000);
}

#if 1 // uboot need

void HalSdio_TimerEnable(void)
{
	// reset HW timer
	FCIE_RIU_W16(TIMER1_MAX_LOW, 0xFFFF);
	FCIE_RIU_W16(TIMER1_MAX_HIGH, 0xFFFF);
	FCIE_RIU_W16(TIMER1_ENABLE, 0);

	// start HW timer
	FCIE_RIU_16_ON(TIMER1_ENABLE, 0x0001);

	// 0xFFFFFFFF = 4,294,967,295 tick
	// divide 12 --> 357,913,941 us --> 357 sec --> 6 min
}


U32 HalSdio_TimerGetTick(void)
{
	U32 u32HWTimer = 0;
	U16 u16TimerLow = 0;
	U16 u16TimerHigh = 0;

	// Get HW timer
	u16TimerLow = FCIE_RIU_R16(TIMER1_CAP_LOW);
	u16TimerHigh = FCIE_RIU_R16(TIMER1_CAP_HIGH);

	u32HWTimer = (u16TimerHigh<<16) | u16TimerLow;

	return u32HWTimer;
}

// max: 357,913,941 = 0x15555555
U32 HalSdio_TimerGetMicroSec(void)
{
	return (HalSdio_TimerGetTick()/12);
}

void HalSdio_TimerTest(void)
{
	unsigned int sec = 0;

	HalSdio_TimerEnable();
	printk("count to 3 then start test: ");
	while(1)
	{
		if (HalSdio_TimerGetMicroSec() >= (1000000+sec*1000000))
		{
			printk("%d ", ++sec);
		}
		if(sec==3)
		{
			printk("Go!\n");
			break;
		}
	}
}

static unsigned int tick_start;
static unsigned int tick_stop;

void HalSdio_TimerStart(void)
{
	HalSdio_TimerEnable();
	tick_start = HalSdio_TimerGetTick();
}

U32 HalSdio_TimerStop(void)
{
	tick_stop = HalSdio_TimerGetTick();
	return ((tick_stop - tick_start) / 12);
}

#endif


static S32 s32VirtualCardDetect = 1;

void HalSdio_SetCardDetect(S32 s32CardDetect)
{
	if(CDZ_option==CD_VIRTUAL_HOTPLUG) // export symbol virtual hotplug
	{
		s32VirtualCardDetect = s32CardDetect;
	}
}
EXPORT_SYMBOL(HalSdio_SetCardDetect);

S32 HalSdio_GetCardDetect(void)
{

	if (!gu16Signature) // do not get configuration form sboot customer setting
		return 0;

	if (CDZ_option==CD_VIRTUAL_HOTPLUG) { // export symbol virtual hotplug
		return s32VirtualCardDetect;
	} else if (CDZ_option==CD_INSERT_L) { // insert low case
		if( FCIE_RIU_R8( RIU_BANK_2_BASE(CdzGpioBank) + ((CdzGpioOffset&0xFE)<<1) + (CdzGpioOffset&0x01) ) & CdzGpioBits )
			return 0;
		else
			return 1;
	} else if (CDZ_option==CD_INSERT_H) { // insert high case
		if( FCIE_RIU_R8( RIU_BANK_2_BASE(CdzGpioBank) + ((CdzGpioOffset&0xFE)<<1) + (CdzGpioOffset&0x01) ) & CdzGpioBits )
			return 1;
		else
			return 0;
	} else { // always insert case, ex: wifi module
		return 1;
	}
}

// 0: not write protect
// 1: write protect
S32 HalSdio_GetWriteProtect(void)
{
	if(wp_option==WP_PROTECT_L) // protect low case
	{
		if( FCIE_RIU_R8( RIU_BANK_2_BASE(wpGpioBank) + ((wpGpioOffset&0xFE)<<1) + (wpGpioOffset&0x01) ) & wpGpioBits )
		{
			return 0;
		}
		else
		{
			printk("write protected\n");
			return 1;
		}
	}
	else if(wp_option==WP_PROTECT_H) // protect high case
	{
		if( FCIE_RIU_R8( RIU_BANK_2_BASE(wpGpioBank) + ((wpGpioOffset&0xFE)<<1) + (wpGpioOffset&0x01) ) & wpGpioBits )
		{
			printk("write protected\n");
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

void HalSdio_SetCardPower(U8 u8OnOff)
{
	if (pwr_option==PC_POWERON_L) { // output low to turn on power case
		if(u8OnOff)
		{
			FCIE_RIU_8_OF( RIU_BANK_2_BASE(pwrGpioBank) + ((pwrGpioOffset&0xFE)<<1) + (pwrGpioOffset&0x01), pwrGpioBits);
		}
		else
		{
			FCIE_RIU_8_ON( RIU_BANK_2_BASE(pwrGpioBank) + ((pwrGpioOffset&0xFE)<<1) + (pwrGpioOffset&0x01), pwrGpioBits);
		}
	} else if (pwr_option==PC_POWERON_H) { // output high to turn on power case
		if (u8OnOff)
			FCIE_RIU_8_ON( RIU_BANK_2_BASE(pwrGpioBank) + ((pwrGpioOffset&0xFE)<<1) + (pwrGpioOffset&0x01), pwrGpioBits);
		else
			FCIE_RIU_8_OF( RIU_BANK_2_BASE(pwrGpioBank) + ((pwrGpioOffset&0xFE)<<1) + (pwrGpioOffset&0x01), pwrGpioBits);
	} else {
		// always power on case
	}
}

U8 HalSdio_GetFactConnectSetting(void)
{
	if (gu16Signature==SBOOT_SIGNATURE_V3) {
		return sdio_fast_connect;
	}
	return 0;
}


int HalSdio_ParseDTS(void)
{
	struct device_node *dn;
	GpioReg SdCdz,SdWp,SdPC;

	printk("sdio parse device tree\n");

	for_each_compatible_node(dn, NULL, "mstar, sdio")
	{

		printk("sdio read mstar device tree\n");


		if(!of_property_read_u32_array(dn, "cd-option", (u32*)&SdCdz, 4))
		{
			printk("cd-option =%x, %x, %x, %x\n", SdCdz.option, SdCdz.bank, SdCdz.offset, SdCdz.bits);

		}

		if(!of_property_read_u32_array(dn, "wp-option",(u32*) &SdWp,4))
		{
			printk("wp-option = %x,%x, %x, %x\n", SdWp.option, SdWp.bank, SdWp.offset, SdWp.bits);
		}
		if(!of_property_read_u32_array(dn, "pc-option",(u32*) &SdPC, 4))
		{
			printk("pc-option =%x, %x, %x, %x\n",SdPC.option, SdPC.bank, SdPC.offset, SdPC.bits);

		}

		//for old project need to be compatibile with sboot GPIO settings
		CDZ_option = SdCdz.option;
		CdzGpioBank = SdCdz.bank;
		CdzGpioBits = SdCdz.bits;
		CdzGpioOffset = SdCdz.offset;
		pwr_option = SdPC.option;
		pwrGpioBank = SdPC.bank;
		pwrGpioOffset = SdPC.offset;
		pwrGpioBits = SdPC.bits;
		wp_option = SdWp.option;
		wpGpioBank = SdWp.bank;
		wpGpioBits = SdWp.bits;
		wpGpioOffset = SdWp.offset;
		gu16Signature = SBOOT_SIGNATURE_DTS;
		return 1;
	}

	return 0;

}
