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

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/semaphore.h>
#include <linux/suspend.h>
#include <linux/param.h>
#include <linux/sysrq.h>

#include "cb2_low.h"
#include "mt53xx_cb2.h"

#ifdef CONFIG_NATIVE_CB2
#define WAIT_CB2_TIMEOUT_SEC (60)
struct semaphore pdwnc_mutex = __SEMAPHORE_INITIALIZER(pdwnc_mutex, 0);
void init_pdwnc_queue(void)
{
}

void wait_pdwnc_queue(void)
{
    int iRet = 0;
    printk("(yjdbg) wait_pdwnc_queue down -1.\n");
    iRet = down_timeout(&pdwnc_mutex,(HZ*WAIT_CB2_TIMEOUT_SEC));
    if(iRet == -ETIME )
    {
        printk("(yjdbg) wait_pdwnc_queue_new down -2 [(%d)s timeout occur].\n", (UINT32)WAIT_CB2_TIMEOUT_SEC);
        printk("(yjdbg) do sysrq(w).\n");
        handle_sysrq('w');
    }
	printk("(yjdbg) wait_pdwnc_queue down -2.\n");
}

void wakeup_pdwnc_queue(void)
{
	printk("(yjdbg) wakeup_pdwnc_queue up.\n");
	up(&pdwnc_mutex);
}

EXPORT_SYMBOL(wakeup_pdwnc_queue);

//new add
struct semaphore pdwnc_mutex_new = __SEMAPHORE_INITIALIZER(pdwnc_mutex_new, 0);
void init_pdwnc_queue_new(void)
{
}

void wait_pdwnc_queue_new(void)
{
    int iRet = 0;
    printk("(yjdbg) wait_pdwnc_queue_new down -1.\n");
    iRet = down_timeout(&pdwnc_mutex_new,(HZ*WAIT_CB2_TIMEOUT_SEC));
    if(iRet == -ETIME )
    {
        printk("(yjdbg) wait_pdwnc_queue_new down -2 [(%d)s timeout occur].\n", (UINT32)WAIT_CB2_TIMEOUT_SEC);
        printk("(yjdbg) do sysrq(w).\n");
        handle_sysrq('w');
    }
	printk("(yjdbg) wait_pdwnc_queue_new down -2.\n");
}

void wakeup_pdwnc_queue_new(void)
{
	printk("(yjdbg) wakeup_pdwnc_queue_new up.\n");
	up(&pdwnc_mutex_new);
}
EXPORT_SYMBOL(wakeup_pdwnc_queue_new);

typedef struct
{
    UINT32 u4PowerStateId;
    UINT32 u4Arg1;
    UINT32 u4Arg2;
    UINT32 u4Arg3;
    UINT32 u4Arg4;
} MTPDWNC_POWER_STATE_T;
extern unsigned int isPowerStateRegistered;
extern unsigned int is2ndPowerStateRegistered;
void PDWNC_PowerStateNotify(UINT32 u4PowerState, UINT32 u4Arg1, UINT32 u4Arg2, UINT32 u4Arg3, UINT32 u4Arg4)
{
    MTPDWNC_POWER_STATE_T t_cb_info;

    if(isPowerStateRegistered == 0)
    {
        printk(KERN_DEBUG "%s: CB2_DRV_PDWNC_POWER_STATE isn't registered\n", __FUNCTION__);
        return;
    }

    printk("(yjdbg)  PDWNC_PowerStateNotify (%d,%d,%d,%d,%d)\n", u4PowerState, u4Arg1, u4Arg2, u4Arg3, u4Arg4);

    t_cb_info.u4PowerStateId = u4PowerState;
    t_cb_info.u4Arg1         = u4Arg1;
    t_cb_info.u4Arg2         = u4Arg2;
    t_cb_info.u4Arg3         = u4Arg3;
    t_cb_info.u4Arg4         = u4Arg4;

    _CB_PutEvent(CB2_DRV_PDWNC_POWER_STATE, sizeof(MTPDWNC_POWER_STATE_T), &t_cb_info);
    wait_pdwnc_queue();
    if(is2ndPowerStateRegistered == 1)
    {
	  _CB_PutEvent(CB2_DRV_PDWNC_POWER_STATE2,
				 sizeof(MTPDWNC_POWER_STATE_T), &t_cb_info);
	  wait_pdwnc_queue_new();
    }
}

static int mtkcore_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
    if (event == PM_SUSPEND_PREPARE) {
        //PDWNC_SuspendPrepare();
        PDWNC_PowerStateNotify(100,1,2,3,4);
    } else if (event == PM_POST_SUSPEND) {
        //PDWNC_PostSuspend();
        PDWNC_PowerStateNotify(200,5,6,7,8);
    }
    return NOTIFY_OK;
}

static struct notifier_block mtkcore_pm_notifier = {
	.notifier_call = mtkcore_pm_notify,
};

static int __init mt53xx_init_machine(void)
{
	init_pdwnc_queue();
	_CB_Init();
	register_pm_notifier(&mtkcore_pm_notifier);
    return 0;
}
arch_initcall(mt53xx_init_machine);
#endif
