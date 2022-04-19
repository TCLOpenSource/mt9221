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
 * FileName hal_card_regs.c
 *     @author jeremy.wang (2011/08/30)
 * Desc:
 * 	   For Dynamic IP Address Reading and Port Setting.
 * 	   We could get current REG/CIFC/CIFD Bank Address anytime!
 * 	   We use global varible to record current port setting of IP, then we could use it to decide reg postions.
 * 	   The goal is that we don't need to change HAL Level code. (But its h file code)
 *
 * 	   The limitations were listed as below:
 * 	   (1) This c file could not use project/cpu/icver/specific define option here, but its h file could.
 *
 ***************************************************************************************************************/

#include "hal_card_regs.h"

static volatile PortEmType gePort[2];


void Hal_CREG_SET_PORT(IPEmType eIP, PortEmType ePort)
{
	gePort[eIP] = ePort;
}

volatile PortEmType Hal_CREG_GET_PORT(IPEmType eIP)
{
	return gePort[eIP];
}

volatile U32_T Hal_CREG_GET_REG_BANK(IPEmType eIP, IPBankEmType eBANK)
{
	if(eIP == EV_IP_FCIE1)
	{
		switch((U8_T)eBANK)
		{
			case (U8_T)EV_REG_BANK:
				return A_FCIE1_0_BANK;
			case (U8_T)EV_CIFC_BANK:
				return A_FCIE1_1_BANK;
			case (U8_T)EV_CIFD_BANK:
				return A_FCIE1_2_BANK;
			default:
				return A_FCIE1_0_BANK;
		}
	}
	else if(eIP == EV_IP_FCIE2)
	{
		switch((U8_T)eBANK)
		{
			case (U8_T)EV_REG_BANK:
				return A_FCIE2_0_BANK;
			case (U8_T)EV_CIFC_BANK:
				return A_FCIE2_1_BANK;
			case (U8_T)EV_CIFD_BANK:
				return A_FCIE2_2_BANK;
			default:
				return A_FCIE2_0_BANK;
		}
	}

	return 0;

}



