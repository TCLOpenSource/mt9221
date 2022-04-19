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

//#include "hal_card_common.h"
#include "hal_card_regs5.h"
#include "sd_platform.h"
#include "hal_card_common.h"
#include <linux/delay.h>
#include "hal_sdmmc.h"

#define MST220B_LIKE	1
#define MST085B_LIKE	2
#define SD_PAD			MST085B_LIKE

#define A_SD_REG_BANK(IP)			GET_CARD_BANK(IP, EV_REG_BANK)
#define A_MIE_FUNC_CTL(IP)			GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x07)
#define A_DDR_MOD_REG(IP)			GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x0F)
#define A_NC_FUNC_CTL(IP)			GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x63)

#define SDIO_MODE_GPIO_PAD_BPS			1
#define SDIO_MODE_GPIO_PAD_SDR			2
#define SDIO_MODE_8BITS_MACRO_SDR		3
#define SDIO_MODE_8BITS_MACRO_DDR		4
#define SDIO_MODE_32BITS_MACRO_DDR		5
#define SDIO_MODE_32BITS_MACRO_SDR104	6
#define SDIO_MODE_UNKNOWN				7

void HalFcie_SwitchPad(IPEmType eIP, unsigned char u32Mode)
{
	// chiptop

	// sdio
	FCIE_RIU_16_OF(A_DDR_MOD_REG(eIP), BIT_FALL_LATCH|BIT_PAD_IN_SEL_SD|BIT_32BIT_MACRO_EN|BIT_DDR_EN|BIT_8BIT_MACRO_EN);

	FCIE_RIU_16_OF(A_NC_FUNC_CTL(eIP), BIT_NC_EN);

	//REG_FCIE_W(FCIE_DDR_TOGGLE_CNT, 0x0000);

	switch(u32Mode)
	{
		case SDIO_MODE_GPIO_PAD_BPS:
			//printk("SDIO_MODE_GPIO_PAD_BPS\n");
			break;

		case SDIO_MODE_GPIO_PAD_SDR:
			//printk("SDIO_MODE_GPIO_PAD_SDR\n");
			FCIE_RIU_16_ON(A_DDR_MOD_REG(eIP), BIT_FALL_LATCH|BIT_PAD_IN_SEL_SD);
			break;

		default:
			printk("SDIO Err: wrong parameter for switch pad func\n");
			break;
	}

}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_InitSetting
 *     @author jeremy.wang (2011/12/1)
 * Desc: IP Begin Setting before every operation, it's about platform setting.
 *
 * @param eIP : FCIE1/FCIE2/...
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_IPBeginSetting(IPEmType eIP, PADEmType ePAD)
{
	HalFcieSdio_CheckStateMachine(eIP);

	FCIE_RIU_16_OF(CHIPTOP_50, BIT15); // all_pad_in = 0

#if 0
	FCIE_RIU_16_OF(CHIPTOP_0B, WBIT01|WBIT02|WBIT03|WBIT04);
	FCIE_RIU_16_ON(CHIPTOP_0B, WBIT03); // reg_pcm_pe[7:4] = 4

	FCIE_RIU_16_OF(CHIPTOP_0C, WBIT00|WBIT01|WBIT02|WBIT03);
	FCIE_RIU_16_ON(CHIPTOP_0C, WBIT02); // reg_pcm_pe[19:16] = 4
#else
	FCIE_RIU_16_ON(CHIPTOP_0B, BIT01|BIT02|BIT03|BIT04);
	FCIE_RIU_16_ON(CHIPTOP_0C, BIT00|BIT01|BIT02|BIT03);
#endif

	FCIE_RIU_16_OF(CHIPTOP_50, BIT07|BIT08|BIT09); // reg_nand_mode = 0

	FCIE_RIU_16_OF(CHIPTOP_6E, BIT07|BIT06); // emmc_config

	FCIE_RIU_16_OF(CHIPTOP_4F, BIT13|BIT14|BIT15); // pcm xxx

	FCIE_RIU_16_OF(CHIPTOP_64, BIT00); // ciadconfig

	FCIE_RIU_16_OF(CHIPTOP_5B, BIT08|BIT09); // spi??

#if (defined SD_PAD) && (SD_PAD == MST220B_LIKE)
	// reg_sd_config = 2 --> MST220B
	FCIE_RIU_16_OF(CHIPTOP_57, BIT08|BIT10|BIT11);
	FCIE_RIU_16_ON(CHIPTOP_57, BIT09);
#else
	// MST085B
	// reg_sd_config = 3
	FCIE_RIU_16_ON(CHIPTOP_57, BIT08|BIT09);
	// reg_sd_config2 = 0
	FCIE_RIU_16_OF(CHIPTOP_57, BIT10|BIT11);
	// reg_sd_config_eco = 0
	FCIE_RIU_16_OF(CHIPTOP_1D, BIT09);
	//printk("A CHIPTOP_57 = %04Xh\n", FCIE_RIU_R16(CHIPTOP_57));
	//printk("A CHIPTOP_58 = %04Xh\n", FCIE_RIU_R16(CHIPTOP_58));
#endif

	HalFcie_SwitchPad(eIP, SDIO_MODE_GPIO_PAD_SDR);

	if(eIP==0) // FCIE
	{
		printk(LIGHT_RED"0 there is no SDIO in munich\n"NONE);
	}
	else if(eIP==1)
	{
		if( !(FCIE_RIU_R16(MIU2_7B) & MIU_SELECT_BY_FCIE))
		{
			printk(LIGHT_RED"SdWarn: MIU select not enable for FCIE IP\n"NONE);
			FCIE_RIU_16_ON(MIU2_7B, MIU_SELECT_BY_FCIE);	// miu select by IP
		}

		FCIE_RIU_W16(A_MIE_FUNC_CTL(eIP), BIT_SD_EN);
	}

}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_IPEndSetting
 *     @author jeremy.wang (2012/5/3)
 * Desc: IP End Setting after every operation, it's about platform setting.
 *
 * @param eIP :
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_IPEndSetting(IPEmType eIP, PADEmType ePAD)
{
	if(eIP==1) // FCIE, the share IP
	{
		//FCIE_RIU_16_OF(CHIPTOP_5A, BIT8|BIT9); // diable SD pad to prevent eMMC driver send command to SD card
	}
}

void Hal_CARD_InitPADPin(PADEmType ePAD, BOOL_T bTwoCard)
{
	//printk("switch pad hal %d\n", ePAD);

	if(ePAD==1) // SDIO coneect to PAD_SDIO_IO0 ~ PAD_SDIO_IO5
	{
		//printk(LIGHT_RED"switch pad hal for SDIO connect to PAD_SDIO...\n"NONE);
	}
	else if(ePAD==0) // FCIE connect to PAD_SD_CLK, PAD_SD_CMD, PAD_SD_DX
	{
		//printk(YELLOW"switch pad hal for FCIE connect to PAD_SD...\n"NONE);
	}
}

void Hal_CARD_InitGPIO(GPIOEmType eGPIO, BOOL_T bEnable)
{
	if( eGPIO==EV_GPIO1 ) // SDIO
	{
		#if (defined SD_PAD) && (SD_PAD == MST220B_LIKE)
			if (bEnable)
			{
				FCIE_RIU_16_ON(PMGPIO_REG_0F, PMGPIO15_OEN); // set oen to high for input
			}
			else
			{

			}
		#else // MST085B
			if (bEnable)
			{
				FCIE_RIU_16_ON(PMGPIO_REG_07, PMGPIO7_OEN); // config as input
			}
			else
			{
			}
		#endif
	}
	else if( eGPIO==EV_GPIO2 )
	{
	}
}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_GetGPIOState
 *     @author jeremy.wang (2012/5/22)
 * Desc:
 *
 * @param eGPIO :
 *
 * @return BOOL_T  :
 ----------------------------------------------------------------------------------------------------------*/
BOOL_T Hal_CARD_GetGPIOState(GPIOEmType eGPIO)
{
	if( eGPIO==EV_GPIO1 ) // SDIO
	{
		#if (defined SD_PAD) && (SD_PAD == MST220B_LIKE)
			if (FCIE_RIU_R16(PMGPIO_REG_0F) & PMGPIO15_IN)
			{
				return FALSE;
			}
			else
			{
				return TRUE;
			}
		#else // MST085B
			if (FCIE_RIU_R16(PMGPIO_REG_07) & PMGPIO7_FIQ_RAW_STS)
			{
				//printk("FALSE\n");
				return FALSE;
			}
			else
			{
				//printk("TRUE\n");
				return TRUE;
			}
		#endif
	}
	else if( eGPIO==EV_GPIO2 )
	{
	}

	return TRUE;
}
/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_SetGPIOIntAttr
 *     @author jeremy.wang (2012/5/22)
 * Desc:
 *
 * @param eGPIO :
 * @param eGPIOOPT :
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_SetGPIOIntAttr(GPIOEmType eGPIO, GPIOOptEmType eGPIOOPT)
{
	//printk(CYAN"Set GPIO int attr %d\n"NONE, eGPIO);
}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_GPIOIntFilter
 *     @author jeremy.wang (2012/5/9)
 * Desc:
 *
 * @param eGPIO :
 *
 * @return BOOL_T  :
 ----------------------------------------------------------------------------------------------------------*/

BOOL_T Hal_CARD_GPIOIntFilter(GPIOEmType eGPIO)
{
    BOOL_T bRet=FALSE;
    return bRet;
}


void ClockBeginSetting(void)
{
}


void HalCard_SetCardPower(U8_T u8OnOff)
{
	FCIE_RIU_16_OF(PM_SAR_REG_11, PM_SAR1_OEN); // active low

	if(u8OnOff)
	{
		FCIE_RIU_16_OF(PM_SAR_REG_12, PM_SAR1_OUT); // set GPIO low to power on
	}
	else
	{
		FCIE_RIU_16_ON(PM_SAR_REG_12, PM_SAR1_OUT); // power off
	}
}


