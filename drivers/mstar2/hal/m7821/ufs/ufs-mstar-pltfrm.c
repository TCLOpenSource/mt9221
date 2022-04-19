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

#include "ufs-mstar-pltfrm.h"
#include "unipro.h"

int efuse_fails;

int ufs_mstar_pltfrm_init(void)
{
	#define RESET_TIMEOUT	0x1000
	int i;
	volatile u16 u16_reg;

	//set clock
	REG_CLR_BITS_UINT16(REG_CKG_UFSHCI, BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);

	//reset ufs pad
	REG_SET_BITS_UINT16(REG_BANK3021_0x00, BIT14);
	REG_SET_BITS_UINT16(REG_BANK3021_0x01, BIT1);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x05, BIT14);

	REG_CLR_BITS_UINT16(REG_BANK3021_0x06, BIT1);
	udelay(100);
	REG_SET_BITS_UINT16(REG_BANK3021_0x06, BIT1);
	udelay(100);

	//set value from efuse
	REG_WRITE_UINT16(REG_BANKeFuse_0x28, 0x213C);		//0x002050 write 0x213C
	i = 0;
	efuse_fails = 0;
	while((REG(REG_BANKeFuse_0x28)& BIT13) == BIT13)
	{
		if(i == 100)
		{
			efuse_fails = 1;		//efuse timeout
			break;
		}
		udelay(1);
		i ++;
	}
	if(efuse_fails == 0 && ((REG(REG_BANKeFuse_0x2D) & BIT14) == BIT14))
	{
		REG_READ_UINT16(REG_BANKeFuse_0x2D, u16_reg);
		REG_CLR_BITS_UINT16(REG_BANK3021_0x19, BIT4|BIT3|BIT2|BIT1|BIT0);
		REG_SET_BITS_UINT16(REG_BANK3021_0x19, (u16_reg>>5) & 0x1F);	//efuse value bit [9:5] to 0x19 bit [4:0]
		REG_CLR_BITS_UINT16(REG_BANK3021_0x1C, BIT12|BIT11|BIT10|BIT9|BIT8);
		REG_SET_BITS_UINT16(REG_BANK3021_0x1C, (u16_reg &0x1F) <<8);			//efuse value bit[4:0] to 0x1C bit[12:8]
		REG_CLR_BITS_UINT16(REG_BANK3021_0x22, BIT7|BIT6|BIT5|BIT4);
		REG_SET_BITS_UINT16(REG_BANK3021_0x22, ((u16_reg>>10) & 0xF) << 4);	//efuse value bit [13:10] to 0x22 bit[7:4]
		REG_SET_BITS_UINT16(REG_BANK3021_0x03, BIT1);
	}
	else		//efuse timeout or efuse is empty
	{
		efuse_fails = 1;
	}
	if(efuse_fails == 0)
	{
		REG_WRITE_UINT16(REG_BANKeFuse_0x28, 0x2140);		//0x002050 write 0x2140
		i = 0;
		efuse_fails = 0;
		while((REG(REG_BANKeFuse_0x28)& BIT13) == BIT13)
		{
			if(i == 100)
			{
				efuse_fails = 1;		//efuse timeout
				break;
			}
			udelay(1);
			i ++;
		}
	}
	if(efuse_fails == 0)
	{
		REG_READ_UINT16(REG_BANKeFuse_0x2D, u16_reg);
		REG_CLR_BITS_UINT16(REG_BANK3021_0x1F, BIT12|BIT11|BIT10|BIT9|BIT8);
		REG_SET_BITS_UINT16(REG_BANK3021_0x1F, ((u16_reg>>6) & 0x1F)<<8);	//efuse value bit [10:6] to 0x1f bit [12:8]
		REG_SET_BITS_UINT16(REG_BANK3021_0x01, BIT7);
	}
	//M-PHY initialization
	REG_SET_BITS_UINT16(REG_BANK3021_0x00, BIT2);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x05, BIT2);
	REG_SET_BITS_UINT16(REG_BANK3021_0x00, BIT3);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x05, BIT3);
	REG_SET_BITS_UINT16(REG_BANK3021_0x00, BIT6);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x05, BIT6);
	REG_SET_BITS_UINT16(REG_BANK3021_0x00, BIT11);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x05, BIT11);
	REG_SET_BITS_UINT16(REG_BANK3021_0x00, BIT10);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x05, BIT10);
	REG_SET_BITS_UINT16(REG_BANK3021_0x02, BIT13);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x14, BIT5);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x14, BIT0);
	udelay(10);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x00, BIT2);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x00, BIT3);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x00, BIT6);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x00, BIT11);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x00, BIT10);
	REG_CLR_BITS_UINT16(REG_BANK3021_0x02, BIT13);

	//reset ip
	REG_SET_BITS_UINT16(REG_BANK3021_0x48, (3 << 14));

	REG_CLR_BITS_UINT16(REG_UFSHCI_RESET_N, 0xFF);
	for(i = 0; i < RESET_TIMEOUT; i ++)
	{
		udelay(1);
		if((REG(REG_UFSHCI_RESET_N) & 0xFF00) == 0x0)
			break;
	}
	if(i == RESET_TIMEOUT)
	{
		return -EIO;
	}

	REG_SET_BITS_UINT16(REG_UFSHCI_RESET_N, 0xFF);

	for(i = 0; i < RESET_TIMEOUT; i ++)
	{
		udelay(1);
		if((REG(REG_UFSHCI_RESET_N) & 0xFF00) == 0xFF00)
			break;
	}
	if(i == RESET_TIMEOUT)
	{
		return -EIO;
	}

	REG_CLR_BITS_UINT16(REG_BANK3021_0x48, (3 << 14));

	// by system design, hardcode now
	REG_WRITE_UINT16(REG_UFSHCI_MI0_START_L, 0x0200);
	REG_WRITE_UINT16(REG_UFSHCI_MI0_START_H, 0x0000);
	REG_WRITE_UINT16(REG_UFSHCI_MI0_END_L, 0x0800);
	REG_WRITE_UINT16(REG_UFSHCI_MI0_END_H, 0x0000);

	return 0;
}

int ufs_mstar_pltfrm_clock(u16 u16ClkParam, u32 u32_gear, u32 u32_hs_rate)
{
	u16 u16_reg;

	REG_READ_UINT16(REG_CKG_UFSHCI, u16_reg);

	u16_reg &= (~UFSHCI_CLK_MASK);
	u16_reg |= (BIT_UFSHCI_CLK_SELECT|u16ClkParam);

	REG_WRITE_UINT16(REG_CKG_UFSHCI, u16_reg);

	if(u32_gear == 1)
	{
		REG_CLR_BITS_UINT16(REG_BANK3021_0x3D, (BIT14-1));//[0:13] = 0x1D4
		REG_CLR_BITS_UINT16(REG_BANK3021_0x3E, (BIT8-1));//[0:7] = d3
		REG_CLR_BITS_UINT16(REG_BANK3021_0x4A, (BIT8-1) << 8);//[15:8] = d5
		if(u32_hs_rate == PA_HS_MODE_A)
		{
			REG_SET_BITS_UINT16(REG_BANK3021_0x3D, 0x1D4);
			REG_SET_BITS_UINT16(REG_BANK3021_0x3E, 3);
			REG_SET_BITS_UINT16(REG_BANK3021_0x4A, (5 << 8));
		}
		else
		{
			REG_SET_BITS_UINT16(REG_BANK3021_0x3D, 0x222);
			REG_SET_BITS_UINT16(REG_BANK3021_0x3E, 3);
			REG_SET_BITS_UINT16(REG_BANK3021_0x4A, (6 << 8));
		}
	}
	else if(u32_gear == 2)
	{
		REG_CLR_BITS_UINT16(REG_BANK3021_0x3D, (BIT14-1));//[0:13] = 0x3A8
		REG_CLR_BITS_UINT16(REG_BANK3021_0x3E, (BIT8-1));//[0:7] = d5
		REG_CLR_BITS_UINT16(REG_BANK3021_0x4A, (BIT8-1) << 8);//[15:8] = d10
		if(u32_hs_rate == PA_HS_MODE_A)
		{
			REG_SET_BITS_UINT16(REG_BANK3021_0x3D, 0x3A8);
			REG_SET_BITS_UINT16(REG_BANK3021_0x3E, 5);
			REG_SET_BITS_UINT16(REG_BANK3021_0x4A, (10 << 8));
		}
		else
		{
			REG_SET_BITS_UINT16(REG_BANK3021_0x3D, 0x444);
			REG_SET_BITS_UINT16(REG_BANK3021_0x3E, 6);
			REG_SET_BITS_UINT16(REG_BANK3021_0x4A, (11 << 8));
		}
	}
	else if(u32_gear == 3)
	{
		REG_CLR_BITS_UINT16(REG_BANK3021_0x3D, (BIT14-1));//[0:13] = 0x750
		REG_CLR_BITS_UINT16(REG_BANK3021_0x3E, (BIT8-1));//[0:7] = d10
		REG_CLR_BITS_UINT16(REG_BANK3021_0x4A, (BIT8-1) << 8);//[15:8] = d19
		if(u32_hs_rate == PA_HS_MODE_A)
		{
			REG_SET_BITS_UINT16(REG_BANK3021_0x3D, 0x750);
			REG_SET_BITS_UINT16(REG_BANK3021_0x3E, 10);
			REG_SET_BITS_UINT16(REG_BANK3021_0x4A, (19 << 8));
		}
		else
		{
			REG_SET_BITS_UINT16(REG_BANK3021_0x3D, 0x888);
			REG_SET_BITS_UINT16(REG_BANK3021_0x3E, 11);
			REG_SET_BITS_UINT16(REG_BANK3021_0x4A, (22 << 8));
		}
	}
	REG_SET_BITS_UINT16(REG_BANK3021_0x69, BIT3);

	return 0;
}
