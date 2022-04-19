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
 * FileName hal_card_intr.h
 *     @author jeremy.wang (2012/01/10)
 * Desc:
 * 	   This file is the header file of hal_card_intr.c.
 *
 ***************************************************************************************************************/

#ifndef __HAL_CARD_INTR_H
#define __HAL_CARD_INTR_H


//***********************************************************************************************************
// Config Setting (Externel)
//***********************************************************************************************************

//###########################################################################################################
//#if (D_PROJECT == D_PROJECT__CB2)    //For Columbus2
//###########################################################################################################

//###########################################################################################################
//#elif (D_PROJECT == D_PROJECT__G2)   //For G2
//###########################################################################################################

#define WT_INT_RISKTIME     10		//(ms) Add Risk Time for wait_event_timer

//###########################################################################################################
//#endif
//============================================
//CARD_EVENT:offset 0x05
//============================================
#define R_SD_STS_CHG            BIT00
#define R_MS_STS_CHG            BIT01
#define R_CF_STS_CHG            BIT02
#define	R_SM_STS_CHG            BIT03
#define	R_XD_STS_CHG            BIT04
#define R_SD_PWR_OC_CHG         BIT05
#define R_CF_PWR_OC_CHG         BIT06
#define R_SDIO_STS_CHG          BIT07
#define R_SDIO2_STS_CHG         BIT08


//***********************************************************************************************************
//***********************************************************************************************************
typedef enum
{
	EV_INT_SD		= R_SD_STS_CHG,
	EV_INT_MS   	= R_MS_STS_CHG,
	EV_INT_CF		= R_CF_STS_CHG,
	EV_INT_SM   	= R_SM_STS_CHG,
	EV_INT_XD		= R_XD_STS_CHG,

} IntCardEmType;

typedef struct
{
	U32_T slotNo;
	IPEmType eIP;
	IntCardEmType eCardInt;
	void * p_data;

} IntSourceStruct;


void Hal_CARD_INT_MIEModeCtrl(IPEmType eIP, IntCardEmType eCardInt, BOOL_T bOpen);
BOOL_T Hal_CARD_INT_MIEModeRunning(IPEmType eIP, IntCardEmType eCardInt);

void Hal_CARD_INT_SetMIEIntEn(IPEmType eIP, IntCardEmType eCardInt, U16_T u16RegMIEIntEN);
void Hal_CARD_INT_ClearMIEEvent(IPEmType eIP);
U16_T Hal_CARD_INT_GetMIEEvent(IPEmType eIP);

BOOL_T Hal_CARD_INT_WaitMIEEvent(IPEmType eIP, U16_T u16ReqEvent, U32_T u32WaitMs);
void Hal_CARD_INT_WaitMIEEventCtrl(IPEmType eIP, BOOL_T bStop);

//###########################################################################################################
#if(D_OS == D_OS__LINUX)
//###########################################################################################################
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>

irqreturn_t Hal_CARD_INT_MIE(int irq, void *p_dev_id);

#endif
//###########################################################################################################




#endif //End of __HAL_CARD_INTR_H
