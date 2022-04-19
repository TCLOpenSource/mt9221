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

#ifndef MS_SDMMC_H
#define MS_SDMMC_H

#include <linux/cdev.h>

//***********************************************************************************************************
// Config Setting (Externel)
//***********************************************************************************************************

#if defined(CONFIG_MSTAR_SDMMC_TWOCARDS)
#define EN_SDMMC_TWO_CARDS     (TRUE)
#else
#define EN_SDMMC_TWO_CARDS     (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC_REVCDZ)
#define EN_SDMMC_CDZREV        (TRUE)
#else
#define EN_SDMMC_CDZREV        (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC_REVWP)
#define EN_SDMMC_WPREV         (TRUE)
#else
#define EN_SDMMC_WPREV         (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC1_HOTP)
#define EN_SDMMC1_HOTP         (TRUE)
#else
#define EN_SDMMC1_HOTP         (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC2_HOTP)
#define EN_SDMMC2_HOTP         (TRUE)
#else
#define EN_SDMMC2_HOTP         (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC1_FAKECDZ)
#define EN_SDMMC1_FAKECDZ      (TRUE)
#else
#define EN_SDMMC1_FAKECDZ      (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC2_FAKECDZ)
#define EN_SDMMC2_FAKECDZ      (TRUE)
#else
#define EN_SDMMC2_FAKECDZ      (FALSE)
#endif

#if defined(CONFIG_MSTAR_SDMMC_HIGHCLK)
#define MAX_CLK_SPEED1         24600000
#define MAX_CLK_SPEED2         24600000
#else
#define MAX_CLK_SPEED1         19200000
#define MAX_CLK_SPEED2         19200000
#endif


//***********************************************************************************************************
typedef enum
{
	EV_SDMMC1 = 0,
	EV_SDMMC2 = 1,

} SlotEmType;

struct ms_sdmmc_host
{
	struct platform_device	*pdev;
	struct ms_sdmmc_slot *sdmmc_slot[2];
};

struct ms_sdmmc_slot
{
	struct mmc_host		*mmc;

	unsigned int	slotNo;   	//Slot No.
	unsigned int	mieIRQNo;	//MIE IRQ No.
	unsigned int	cdzIRQNo;	//CDZ IRQ No.
	unsigned int	irqIP;		//IP for INT
	unsigned int	currClk;	//Current Clock

	int ro;		  //WP
	int card_det;		//Card Detect

	/****** DMA buffer used for transmitting *******/
	u32 *dma_buffer;
	dma_addr_t dma_phy_addr;

	/***** Tasklet for hotplug ******/
	struct tasklet_struct   hotplug_tasklet;

};  /* struct ms_sdmmc_hot*/


#endif

