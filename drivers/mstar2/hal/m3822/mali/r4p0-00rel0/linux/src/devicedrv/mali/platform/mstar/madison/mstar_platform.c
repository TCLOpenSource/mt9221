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


/**
 * @file mstar_platform.c
 * MStar platform specific driver functions for Madison
 */
#include "mstar/mstar_platform.h"
#include <linux/delay.h>

#define REG_CLKGEN1_BASE        0x103300
#define   REG_CKG_GPU           0x20
#define     DISABLE_CLK_GPU     0x0001

#define REG_G3D_BASE            0x110800
#define   REG_SOFT_RESET        0x00
#define     BITMASK_SOFT_RESET  0x0001
#define   REG_GPUPLL_CTRL0_LOW  0x44
#define   REG_GPUPLL_CTRL0_HIGH 0x45
#define   REG_GPUPLL_CTRL1_LOW  0x46
#define   REG_GPUPLL_CTRL1_HIGH 0x47
#define   REG_SPLIT_2CH_MD      0x60
#define     READ_OUTSTANDING    0x0002
#define   REG_G3D_RREQ          0x62
#define     REG_G3D_RREQ_THRD   0x000f
#define   REG_G3D_WREQ          0x63
#define     REG_G3D_WREQ_THRD   0x000f
#define   REG_RIU_APB_EN        0x6a
#define     BITMASK_RIU_APB_EN  0x0001
#define   REG_MIU1_BASE_LOW     0x77
#define   REG_MIU1_BASE_HIGH    0x78

/* RIU */
#define RIU_MAP               0xfd000000
#define RIU                   ((volatile unsigned short*)RIU_MAP)
#define CLKGEN1_REG(addr)     RIU[REG_CLKGEN1_BASE + ((addr) << 1)]
#define G3D_REG(addr)         RIU[REG_G3D_BASE + ((addr) << 1)]

/* GPU clock */
extern int mali_gpu_clock;

/* platform functions */
void mstar_platform_init(void)
{
    mali_gpu_clock = GPU_CLOCK/6*6;
    mstar_platform_power_on();
}

void mstar_platform_deinit(void)
{
    mstar_platform_power_off();
}

void mstar_platform_power_on(void)
{
    /* set GPU clock: must before power on */
    G3D_REG(REG_GPUPLL_CTRL1_LOW)  = GPU_CLOCK/6;
    G3D_REG(REG_GPUPLL_CTRL1_HIGH) = 0x0000;
    udelay(100);

    /* enable read by outstanding order*/
    G3D_REG(REG_SPLIT_2CH_MD) |= READ_OUTSTANDING;

    /* reg_g3d_rreq_thrd = 0x0 */
    G3D_REG(REG_G3D_RREQ) &= ~REG_G3D_RREQ_THRD;

    /* reg_g3d_wreq_thrd = 0x0 */
    G3D_REG(REG_G3D_WREQ) &= ~REG_G3D_WREQ_THRD;

    /* Set MIU1 base address */
    G3D_REG(REG_MIU1_BASE_LOW)  = MIU1_PHY_BASE_ADDR_LOW;
    G3D_REG(REG_MIU1_BASE_HIGH) = MIU1_PHY_BASE_ADDR_HIGH;
    udelay(100);

    /* enable RIU access */
#ifdef MSTAR_RIU_ENABLED
    G3D_REG(REG_RIU_APB_EN) |= BITMASK_RIU_APB_EN;
    udelay(100);
#endif

    /* power on mali */
    G3D_REG(REG_GPUPLL_CTRL0_LOW)  = 0x0098;
    G3D_REG(REG_GPUPLL_CTRL0_HIGH) = 0x00C0;
    udelay(100);

    /* disable GPU clock gating */
    CLKGEN1_REG(REG_CKG_GPU) &= ~DISABLE_CLK_GPU;
    udelay(100);

    /* reset mali */
    G3D_REG(REG_SOFT_RESET) &= ~BITMASK_SOFT_RESET;
    G3D_REG(REG_SOFT_RESET) |= BITMASK_SOFT_RESET;
    udelay(100); /*delay for run-time suspend*/
    G3D_REG(REG_SOFT_RESET) &= ~BITMASK_SOFT_RESET;
    udelay(100);
}

void mstar_platform_power_off(void)
{
    /* enable GPU clock gating */
    CLKGEN1_REG(REG_CKG_GPU) |= DISABLE_CLK_GPU;
    udelay(100);
}

void mstar_platform_suspend(void)
{
    mstar_platform_power_off(); /* just power off */
}

void mstar_platform_resume(void)
{
    mstar_platform_power_on();  /* just power on */
}
