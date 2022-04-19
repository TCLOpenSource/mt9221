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

/***************************************************************************************************************
 *
 * FileName hal_card_platform.c
 *     @author benson.hsiao (2012/7/14)
 * Desc:
 *     The platform Setting of all cards will run here.
 *     Every Project will have XX project name for different hal_card_platform_XX.c files
 *     The goal is that we don't need to change "other" HAL Level code.
 *
 *     The limitations were listed as below:
 *     (1) This c file belongs to HAL level.
 *     (2) Its h file is included by driver API level, not driver flow process.
 *     (3) IP Init, PADPath, PADInit, Clock and Power function belong to here.
 *     (4) Timer Setting doesn't belong to here, because it will be included by other HAL level.
 *     (5) FCIE/SDIO IP Reg Setting doesn't belong to here.
 *     (6) If we could, we don't need to change any code of hal_card_platform.h
 *
 ***************************************************************************************************************/
#include "hal_card_regs5.h"

#include "hal_card_common.h"
#include "hal_card_timer.h"
#include "hal_sdmmc.h"

#include "chip_int.h"
#include <mstar/mstar_chip.h>
#include <linux/sched.h>
#include "sd_platform.h"
#include "ms_sdmmc_lnx.h"

#if defined(SHARE_DATA_BUS_WITH_EMMC) && (SHARE_DATA_BUS_WITH_EMMC)
extern bool mstar_mci_exit_checkdone_ForSD(void);
#endif
int gScan=0;

//***********************************************************************************************************
// Config Setting (Internel)
//***********************************************************************************************************
#define A_SD_REG_BANK(IP)           GET_CARD_BANK(IP, EV_REG_BANK)

#define A_MIE_EVENT_REG(IP)         GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x00)
#define A_MIE_INT_EN_REG(IP)        GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x01)
#define A_MMA_PRI_REG(IP)			GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x02)
#define A_MIU_DMA1_REG(IP)          GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x03)
#define A_MIE_FUNC_CTL(IP)			GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x07)
#define A_SD_MODE_REG(IP)           GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x0B)
#define A_SD_STS_REG(IP)            GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x0D)
#define A_NC_REORDER_REG(IP)        GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x2D)
#define A_EMMC_BOOTCFG_REG(IP)      GET_CARD_REG_ADDR(A_SD_REG_BANK(IP), 0x2F)

// Reg Dynamic Variable
//-----------------------------------------------------------------------------------------------------------
static volatile BOOL_T	    gb_SD_Mode_HighSpeed[2] = {0};
// Timer Definition
//-----------------------------------------------------------------------------------------------------------

#define SD_DELAY_1us					1
#define SD_DELAY_10us					10
#define SD_DELAY_100us				    100
#define SD_DELAY_1ms					(1000 * SD_DELAY_1us)
#define SD_DELAY_10ms					(10   * SD_DELAY_1ms)
#define SD_DELAY_100ms				    (100  * SD_DELAY_1ms)
#define SD_DELAY_1s					    (1000 * SD_DELAY_1ms)

//***********************************************************************************************************
// IP Setting for Card Platform
//***********************************************************************************************************

#define WAIT_EMMC_D0_TIME      3000

bool ms_sdmmc_wait_d0_for_emmc(void) // currently wait D0 by sd driver itself
{

    return TRUE;
}


bool ms_sdmmc_card_chg(unsigned int slotNo)
{
        return FALSE; // edison don't have share pin issue
}
/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_InitIPOnce
 *     @author jeremy.wang (2012/6/7)
 * Desc:  IP Once Setting , it's about platform setting.
 *
 * @param eIP :
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_IPOnceSetting(IPEmType eIP)
{

}
int Hal_Check_Card_Pins(void)
{
	printk("card sts:%x\n", FCIE_RIU_R16(A_SD_STS_REG(0)));
	return !(FCIE_RIU_R16(A_SD_STS_REG(0)) & 0xf00);
}
/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_Wait_Emmc_D0
 *     @author benson.hsiao (2012/6/21)
 * Desc:  Wait EMMC D0 to High.
 *
 * @param eIP :
 ----------------------------------------------------------------------------------------------------------*/
BOOL_T Hal_CARD_Wait_Emmc_D0(void)
{
	#if defined(SHARE_DATA_BUS_WITH_EMMC) && (SHARE_DATA_BUS_WITH_EMMC)

		printk("Hal_CARD_Wait_Emmc_D0()\n");
		return (mstar_mci_exit_checkdone_ForSD());

    #else

    	return TRUE;

	#endif
}

// Switch PAD
//------------------------------------------------------------------------------------------------


/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_SetPADToPortPath
 *     @author jeremy.wang (2011/12/1)
 * Desc: Set PAD to connect IP Port
 *
 * @param eIP : FCIE1/FCIE2/...
 * @param ePort : IP Port
 * @param ePAD : PAD (Could Set NOPAD for 1-bit two cards)
 * @param bTwoCard : 1-bit two cards or not
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_SetPADToPortPath(IPEmType eIP, PortEmType ePort, PADEmType ePAD, BOOL_T bTwoCard)
{
    SET_CARD_PORT(eIP, ePort);
#if 0
    if(eIP == EV_IP_FCIE1)
    {

    }
#endif
}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_PullPADPin
 *     @author jeremy.wang (2011/12/1)
 * Desc: Pull PAD Pin for Special Purpose (Avoid Power loss.., Save Power)
 *
 * @param ePAD :  PAD
 * @param ePin :  Group Pin
 * @param ePinPull : Pull up/Pull down
 * @param bTwoCard :  two card(1 bit) or not
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_PullPADPin(PADEmType ePAD, PinEmType ePin, PinPullEmType ePinPull, BOOL_T bTwoCard)
{
}

//***********************************************************************************************************
// Clock Setting for Card Platform
//***********************************************************************************************************
extern void GoSDR50(int);
extern void GoDDR50(int );

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_SetClock
 *     @author jeremy.wang (2011/12/14)
 * Desc: Set Clock Level by Real Clock from IP
 *
 * @param eIP : FCIE1/FCIE2/...
 * @param u32KClkFromIP : Clock Value From IP Source Set
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_SetClock(IPEmType eIP, U32_T u32ClkFromIPSet)
{
    if(eIP == EV_IP_FCIE1) // SDIO
    {
		//printk("SDIO set clock %d\n", u32ClkFromIPSet);

        ClockBeginSetting(); // clear clkgen setting

		#if 1
        switch(u32ClkFromIPSet)
        {
			case CLK_F:      //48000KHz
				//	FCIE_RIU_16_ON(GET_CARD_REG_ADDR(CLKGEN_BANK, CLOCK_GEN_REG), _48M);
				//printk("48M\n");
				//mdelay(3000);
				#if defined SDBUS && (SDBUS==DDR50)
					FCIE_RIU_16_ON(CLKGEN_SDIO, (0x0c<<2));
					GoDDR50(gScan);
				#elif defined SDBUS && ((SDBUS==SDR50) ||(SDBUS==SDR104))
					FCIE_RIU_16_ON(CLKGEN_SDIO, (0x0b<<2));
					GoSDR50(gScan);
				#else
			 		FCIE_RIU_16_ON(CLKGEN_SDIO, _48M);
				#endif
				break;

			case CLK_E:      //43200KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _43M);
				break;

			case CLK_D:      //40000KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _36M);	//36M
				break;

			case CLK_C:      //36000KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _36M);	//36M
				break;

			case CLK_B:      //32000KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _32M);	//32M
				break;

			case CLK_A:      //27000KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _27M);	// 27M
				break;

			case CLK_9:      //20000KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _18M);	// 18M
				break;

			case CLK_8:      //12000KHz
				break;

			case CLK_7:      //300KHz
				FCIE_RIU_16_ON(CLKGEN_SDIO, _300K);	// 300K
				break;
			case 0:
				printk("clock gating\n");
				FCIE_RIU_16_ON(CLKGEN_SDIO, BIT00);	// clock gating
				break;

        }
		#else
			FCIE_RIU_16_ON(GET_CARD_REG_ADDR(CLKGEN_BANK, CLOCK_GEN_REG), _300K);	// 300K
		#endif

    }
	else // FCIE
	{
		//printk("FCIE set clock %d\n", u32ClkFromIPSet);

		FCIE_RIU_16_ON(CLKGEN_FCIE, FCIE_CLK_GATING); // gate clock

		FCIE_RIU_16_OF(CLKGEN_FCIE, FCIE_CLK_INVERSE|FCIE_CLK_SOURCE_MSK); // clear inverse & clock source
		FCIE_RIU_16_ON(CLKGEN_FCIE, FCIE_CLK_SOURCE_SEL); // select source from clk_fcie_p1

		switch(u32ClkFromIPSet)
		{
			case FCIE_CLK_F: // 48000000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0xF<<2);
				break;
			case FCIE_CLK_5: // 43200000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0x5<<2);
				break;
			case FCIE_CLK_4: // 40000000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0x4<<2);
				break;
			case FCIE_CLK_3: // 36000000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0x3<<2);
				break;
			case FCIE_CLK_2: // 32000000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0x2<<2);
				break;
			case FCIE_CLK_1: // 20000000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0x1<<2);
				break;
			case FCIE_CLK_D: //   300000
				FCIE_RIU_16_ON(CLKGEN_FCIE, 0xD<<2);
				break;
			default:
				printk("Select wrong FCIE clock\n");
				break;
		}

		FCIE_RIU_16_OF(CLKGEN_FCIE, FCIE_CLK_GATING); // open clock

	}
}



/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_FindClockSetting
 *     @author jeremy.wang (2012/5/22)
 * Desc: Find Real Clock Level Setting by Reference Clock
 *
 * @param eIP : FCIE1/FCIE2/...
 * @param u32ReffClk : Reference Clock Value
 * @param u8PassLevel : Pass Level to Clock Speed
 * @param u8DownLevel : Down Level to Decrease Clock Speed
 *
 * @return U32_T  : Real Clock
 ----------------------------------------------------------------------------------------------------------*/
U32_T Hal_CARD_FindClockSetting(IPEmType eIP, U32_T u32ReffClk, U8_T u8PassLevel, U8_T u8DownLevel)
{

	if(eIP==0) // SDIO
	{
	    U8_T  u8LV = u8PassLevel;
	    U32_T u32RealClk = 0;
	    U32_T u32ClkArr[16] = {CLK_F, CLK_E, CLK_D, CLK_C, CLK_B, CLK_A, \
	        CLK_9, CLK_8, CLK_7, CLK_6, CLK_5, CLK_4, CLK_3, CLK_2, CLK_1, CLK_0};

		//printk(LIGHT_RED"find clock setting %d\n"NONE, eIP);

		//printk("Reffclk:%d\n",u32ReffClk);
	    for(; u8LV<16; u8LV++)
	    {
	        if( (u32ReffClk >= u32ClkArr[u8LV]) || (u8LV==15)  )
	        {
	            u32RealClk = u32ClkArr[u8LV];
	            break;
	        }
	    }
		//printk("realclk:%d\n",u32RealClk);
	    /****** For decrease clock speed******/
	    if( (u8DownLevel) && (u32RealClk) && ((u8LV+u8DownLevel)<=15) )
	    {
	        if(u32ClkArr[u8LV+u8DownLevel]>0) //Have Level for setting
	            u32RealClk = u32ClkArr[u8LV+u8DownLevel];
	    }
		//printk(LIGHT_RED"real clock select: %u\n"NONE,u32RealClk);

	    return u32RealClk;
	}
	else
	{
	    U8_T  u8LV = u8PassLevel;
	    U32_T u32RealClk = 0;
	    U32_T u32ClkArr[7] = {FCIE_CLK_F, FCIE_CLK_5, FCIE_CLK_4, FCIE_CLK_3, FCIE_CLK_2, FCIE_CLK_1, FCIE_CLK_D};

		//printk(YELLOW"find clock setting %d\n"NONE, eIP);

	    for(; u8LV<16; u8LV++)
	    {
	        if( (u32ReffClk >= u32ClkArr[u8LV]) || (u8LV==6)  )
	        {
	            u32RealClk = u32ClkArr[u8LV];
	            break;
	        }
	    }

		//printk(YELLOW"real clock select: %u\n"NONE, u32RealClk);

	    return u32RealClk;
	}
}


//***********************************************************************************************************
// Power Setting for Card Platform
//***********************************************************************************************************

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_SetPADPower
 *     @author jeremy.wang (2012/1/4)
 * Desc: Set PAD Power to Different Voltage
 *
 * @param ePAD : PAD
 * @param ePADVdd : LOW/MIDDLE/HIGH Voltage Level
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_SetPADPower(PADEmType ePAD, PADVddEmType ePADVdd)
{


}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_PowerOn
 *     @author jeremy.wang (2011/12/13)
 * Desc: Power on Card Power
 *
 * @param ePAD : PAD
 * @param u16DelayMs : Delay ms for Stable Power
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_PowerOn(PADEmType ePAD, U16_T u16DelayMs)
{
	HalCard_SetCardPower(ePAD, 1);
}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_PowerOff
 *     @author jeremy.wang (2011/12/13)
 * Desc: Power off Card Power
 *
 * @param ePAD : PAD
 * @param u16DelayMs :  Delay ms to Confirm No Power
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_PowerOff(PADEmType ePAD, U16_T u16DelayMs)
{
	HalCard_SetCardPower(ePAD, 0);
    Hal_Timer_uDelay(u16DelayMs);
}

/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_SDMMC_SetHighSpeed
 *     @author jeremy.wang (2012/3/29)
 * Desc: Set High Speed registers in other bank for SDR/DDR
 *
 * @param eIP : FCIE1/FCIE2/...
 * @param bEnable : Enable or Disable
 ----------------------------------------------------------------------------------------------------------*/
void Hal_CARD_SetHighSpeed(IPEmType eIP, BOOL_T bEnable)
{
    gb_SD_Mode_HighSpeed[eIP] = bEnable;
}


/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_GetGPIONum
 *     @author jeremy.wang (2012/5/22)
 * Desc:
 *
 * @param eGPIO :
 *
 * @return U32_T  :
 ----------------------------------------------------------------------------------------------------------*/
U32_T Hal_CARD_GetGPIONum(GPIOEmType eGPIO)
{
    S32_T s32GPIO = -1;

    if( eGPIO==EV_GPIO1 ) //EV_GPIO1 for Slot 0
    {
    }
    else if( eGPIO==EV_GPIO2)
    {
    }

    if(s32GPIO>0)
        return (U32_T)s32GPIO;
    else
        return 0;
}


/*----------------------------------------------------------------------------------------------------------
 *
 * Function: Hal_CARD_TransMIUAddr
 *     @author jeremy.wang (2012/6/7)
 * Desc:
 *
 * @param eIP
 * @param u32Addr :
 * @return U32_T  :
 ----------------------------------------------------------------------------------------------------------*/

#define SDIO_DBG_MIU_ADDR(MSG) //MSG

U32_T Hal_CARD_TransMIUAddr(IPEmType eIP, U64_T u64BusAddr)
{

    FCIE_RIU_16_OF(A_MMA_PRI_REG(eIP), BIT02|BIT03);

    if( u64BusAddr >= MSTAR_MIU1_BUS_BASE)
    {
		SDIO_DBG_MIU_ADDR(printk("miu 1 addr: %08llX_%08llXh - %08llX_%08llXh = ", u64BusAddr>>32, u64BusAddr&0xFFFFFFFF, ((U64_T)MSTAR_MIU1_BUS_BASE)>>32, ((U64_T)MSTAR_MIU1_BUS_BASE)&0xFFFFFFFF));
        u64BusAddr -= MSTAR_MIU1_BUS_BASE;
		SDIO_DBG_MIU_ADDR(printk("%08llX_%08llXh\n", u64BusAddr>>32, u64BusAddr&0xFFFFFFFF));

        FCIE_RIU_16_ON(A_MMA_PRI_REG(eIP), BIT02);
    }
    else // miu 0
    {
		SDIO_DBG_MIU_ADDR(printk("miu 0 addr: %08llX_%08llXh - %08llX_%08llXh = ", u64BusAddr>>32, u64BusAddr&0xFFFFFFFF, ((U64_T)MSTAR_MIU0_BUS_BASE)>>32, ((U64_T)MSTAR_MIU0_BUS_BASE)&0xFFFFFFFF));
        u64BusAddr -= MSTAR_MIU0_BUS_BASE;
		SDIO_DBG_MIU_ADDR(printk("%08llX_%08llXh\n", u64BusAddr>>32, u64BusAddr&0xFFFFFFFF));
    }

    return (U32_T)u64BusAddr; // FCIE/SDIO IP DMA address is 32 bits
}




















