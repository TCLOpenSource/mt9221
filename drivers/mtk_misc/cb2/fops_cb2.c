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

/*-----------------------------------------------------------------------------
 *
 * $Author: dtvbm11 $
 * $Date: 2015/04/15 $
 * $RCSfile: fops_cb2.c,v $
 * $Revision: #1 $
 *
 *---------------------------------------------------------------------------*/

/** @file cb_mod.c
 *  Callback interface of MT53XX.
 */


//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include <linux/poll.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include "cb2_low.h"
#include "mt53xx_cb2.h"
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
//-----------------------------------------------------------------------------
// Constant definitions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define DECLARE_MUTEX_LOCKED(name) \
       struct semaphore name = __SEMAPHORE_INITIALIZER(name, 0)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#define DECLARE_MUTEX_LOCKED(name) __DECLARE_SEMAPHORE_GENERIC(name,0)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
#define USE_UNLOCK_IOCTL
#endif

#ifdef CB_TIEMR_DEBUG
typedef struct _private_data {
    HANDLE_T timer;
}private_data;
#endif

//#define CB_TIEMR_DEBUG
#define CB_TIMER_TIMEROUT 1000

//-----------------------------------------------------------------------------
// Static functions
//-----------------------------------------------------------------------------
#ifdef CB_TIEMR_DEBUG
static VOID _cb_timer_timeout(HANDLE_T  pt_tm_handle, VOID *pv_tag)
{
	CB_CALLBACK_EVENT_T *prCbEv = (CB_CALLBACK_EVENT_T *) pv_tag;

	if (prCbEv != NULL) {
		printk("_cb_timer_timeout Id(%d) Timer Out\n",
		       (int) prCbEv->rGetCb.eFctId);
	} else {
		printk("_cb_timer_timeout Timer Out\n");
	}
}
#endif

static DEFINE_MUTEX(twoworlds_power_lock);

// Public functions
//-----------------------------------------------------------------------------
#ifndef USE_UNLOCK_IOCTL
int cb_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	     unsigned long arg)
#else
long cb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int retval = 0;
	CB_SET_CALLBACK_T rSetCb;
	CB_CALLBACK_EVENT_T *prCbEv = NULL;
	int sig_return;
	CB_GET_PID_T rGetPidCb;
	CB_GET_CBQUEUE_COUNT_T rGetCbCount;
#if CB_MULTIPROCESS
	UINT32 u4DoneMask;
#endif

#ifdef CB_TIEMR_DEBUG
	private_data *pdata = (private_data *) file->private_data;
#endif

	switch (cmd) {
	case CB_SET_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}
#if CB_MULTIPROCESS_REPLACE
		retval =
		    _CB_RegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				      rSetCb.rCbPid);
#else
		retval =
		    _CB_RegCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				 rSetCb.rCbPid);
#endif
		break;
#if CB_MULTIPROCESS
	case CB_SET_MULTI_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		retval =
		    _CB_RegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				      rSetCb.rCbPid);

		break;
#endif
	case CB_GET_CALLBACK:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			//ASSERT(0);
			return -EFAULT;
		}
#ifdef CB_TIEMR_DEBUG
		x_timer_stop(pdata->timer);
#endif

#if CB_MULTIPROCESS
		sig_return =
		    _CB_PutThdIntoWQ((prCbEv =
				      _CB_GetEvent(&u4DoneMask)) != NULL);
#else
		sig_return =
		    _CB_PutThdIntoWQ((prCbEv = _CB_GetEvent()) != NULL);
#endif
		if (sig_return) {
			return -EINTR;
		}

		if (prCbEv != NULL) {
#ifdef CB_TIEMR_DEBUG
			x_timer_start(pdata->timer, CB_TIMER_TIMEROUT,
				      X_TIMER_FLAG_ONCE, _cb_timer_timeout,
				      (VOID *) prCbEv);
#endif
			//ASSERT((sizeof(CB_GET_CALLBACK_T) + prCbEv->rGetCb.i4TagSize) < CB_MAX_STRUCT_SIZE);

			if (copy_to_user
			    ((void __user *) arg, &(prCbEv->rGetCb),
			     sizeof(CB_GET_CALLBACK_T) +
			     prCbEv->rGetCb.i4TagSize)) {
				printk
				    ("copy_to_user error, user addr=0x%x\n",
				     (unsigned int) (void __user *) arg);
				retval = -EFAULT;
			}
#if CB_MULTIPROCESS
			_CB_FreeEvent(prCbEv, u4DoneMask);
#else
			kfree(prCbEv);
#endif
		}
		else {
			retval = -EFAULT;
		}
		break;

	case CB_UNSET_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}
#if CB_MULTIPROCESS_REPLACE
		retval =
		    _CB_UnRegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
					rSetCb.rCbPid);
#else
		retval =
		    _CB_UnRegCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				   rSetCb.rCbPid);
#endif
		break;
#if CB_MULTIPROCESS
	case CB_UNSET_MULTI_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		retval =
		    _CB_UnRegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
					rSetCb.rCbPid);

		break;
#endif
	case CB_GET_PID:

		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_GET_PID_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rGetPidCb, (void __user *) arg,
				   sizeof(CB_GET_PID_T))) {
			return -EFAULT;
		}

		rGetPidCb.rCbPid = _CB_GetPid(rGetPidCb.eFctId);
		//printk("GetPID %d\n",rGetPidCb.rCbPid);
		if (copy_to_user((void __user *) arg, &rGetPidCb,
				 sizeof(CB_GET_PID_T))) {
			printk("copy_to_user error, user addr=0x%x\n",
			       (unsigned int) (void __user *) arg);
			retval = -EFAULT;
		}
		break;

	case CB_GET_CBQUEUE_COUNT:

		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_GET_CBQUEUE_COUNT_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rGetCbCount, (void __user *) arg,
				   sizeof(CB_GET_CBQUEUE_COUNT_T))) {
			return -EFAULT;
		}

		rGetCbCount.iCount = _CB_GetCbCount();
		printk("GetCbCount = %d\n", rGetCbCount.iCount);
		if (copy_to_user((void __user *) arg, &rGetCbCount,
				 sizeof(CB_GET_CBQUEUE_COUNT_T))) {
			printk("copy_to_user error, user addr=0x%x\n",
			       (unsigned int) (void __user *) arg);
			retval = -EFAULT;
		}
		break;
	case CB_SEND_CALLBACK:
		{
			int i4Dummy = 0;
			if (!access_ok(VERIFY_READ, (void __user *) arg,
				 sizeof(CB_SET_CALLBACK_T))) {
				return -EFAULT;
			}
		
			if (copy_from_user(&rSetCb, (void __user *) arg,
				  sizeof(CB_SET_CALLBACK_T))) {
				return -EFAULT;
			}
			_CB_PutEvent(rSetCb.eFctId,
					 (INT32) sizeof(int),
					 (void *) &i4Dummy);
		}
		break;


#if 0
	case CB_SET_TEMINATE:
		{
			int i4Dummy = 0;
			_CB_PutEvent(CB_MTAL_TEMINATE_TRIGGER,
				     (INT32) sizeof(int),
				     (void *) &i4Dummy);
		}
		break;
#endif

	default:
		//ASSERT(0);
		printk(KERN_ALERT "cb_ioctl: Error: Unknown cmd 0x%x\n", cmd);
		retval = -EFAULT;
		break;
	}

	return retval;
}

#define CB2_CMD_SUSPEND  "mem"
#define CB2_CMD_RESUME   "off"

extern void PDWNC_PowerStateNotify(UINT32 u4PowerState, UINT32 u4Arg1,
				   UINT32 u4Arg2, UINT32 u4Arg3, UINT32 u4Arg4);

static int cb_open(struct inode *inode, struct file *file)
{
#ifdef CB_TIEMR_DEBUG
	private_data *pdata;
#endif

#ifdef CB_TIEMR_DEBUG
	pdata = kmalloc(sizeof(private_data), GFP_KERNEL);
	file->private_data = (void *) pdata;
	if (file->private_data == NULL) {
		return -1;
	}
	x_timer_create(&(pdata->timer));
#endif
	return 0;
}

extern void _CB_ClearPendingEventEntries(void);
extern void _CB_ClearCbIdArray(void);

static int cb_release(struct inode *inode, struct file *file)
{
	_CB_ClearPendingEventEntries();
	_CB_ClearCbIdArray();
#ifdef CB_TIEMR_DEBUG
	kfree(file->private_data);
#endif
	return 0;
}

static struct workqueue_struct *twoworlds_power_wq;
static struct wakeup_source *twoworlds_power_ws;

static int twoworlds_power_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	twoworlds_power_ws = wakeup_source_register(NULL, "twoworlds_power");
#else
	twoworlds_power_ws = wakeup_source_register("twoworlds_power");
#endif
	if (!twoworlds_power_ws)
		return -ENOMEM;

	twoworlds_power_wq = create_singlethread_workqueue("twoworlds_power");
	if (!twoworlds_power_wq) {
		wakeup_source_unregister(twoworlds_power_ws);
		return -ENOMEM;
	}

	return 0;
}
core_initcall(twoworlds_power_init);

static void early_suspend(struct work_struct *work)
{
	PDWNC_PowerStateNotify(0, 0, 0, 0, 0);
	__pm_relax(twoworlds_power_ws);
}
static DECLARE_WORK(early_suspend_work, early_suspend);

static void late_resume(struct work_struct *work)
{
	PDWNC_PowerStateNotify(1, 0, 0, 0, 0);
}
static DECLARE_WORK(late_resume_work, late_resume);

static suspend_state_t twoworlds_power_state;

extern void msleep(unsigned int msecs);
static void twoworlds_power_set_state(suspend_state_t state)
{
	int ret;

	mutex_lock(&twoworlds_power_lock);
	if (twoworlds_power_state == PM_SUSPEND_MEM && state == PM_SUSPEND_ON) {
		pr_info("twoworld power: queue late_resume_work start\n");
		while (!queue_work(twoworlds_power_wq, &late_resume_work))
		{
		    msleep(10);
		}
		pr_info("twoworld power: queue late_resume_work end\n");
	} else if (twoworlds_power_state == PM_SUSPEND_ON &&
		   state == PM_SUSPEND_MEM) {
		/**
		 * 1. acquire a wakesource
		 * 2. queue up early_suspend_work
		 * 3. the wakesource will be relaxed by early_suspend_work
		 */
		__pm_stay_awake(twoworlds_power_ws);
		pr_info("twoworld power: queue early_suspend_work start\n");
		while (!queue_work(twoworlds_power_wq, &early_suspend_work))
		{
		    msleep(10);
		}
		pr_info("twoworld power: queue early_suspend_work end\n");
	}
	twoworlds_power_state = state;
	mutex_unlock(&twoworlds_power_lock);
}

static ssize_t cb_write(struct file *file, const char __user *from,
			size_t cnt, loff_t *ppos)
{
#define MAX_CNT 16
	char str[MAX_CNT + 1];
	suspend_state_t state;

	if (cnt > MAX_CNT)
		return -EINVAL;

	if (copy_from_user(str, from, cnt))
		return -EFAULT;

	str[cnt] = 0;

	pr_info("twoworld power: write [%s] to cb2\n", str);

	if (!strcmp(str, CB2_CMD_SUSPEND))
		state = PM_SUSPEND_MEM;
	else if (!strcmp(str, CB2_CMD_RESUME) && twoworlds_power_state == PM_SUSPEND_MEM)
		state = PM_SUSPEND_ON;
	else
		return -EINVAL;

	twoworlds_power_set_state(state);
	return cnt;
}

static ssize_t cb_read(struct file *filp, char __user *to,
		       size_t size, loff_t *ppos)
{
	char *str;

	if (twoworlds_power_state == PM_SUSPEND_ON)
		str = "on\n";
	else if (twoworlds_power_state == PM_SUSPEND_MEM)
		str = "mem\n";
	else
		str = "unknown\n";

	return simple_read_from_buffer(to, size, ppos, str, strlen(str));
}

suspend_state_t twoworlds_power_get_state(void)
{
	return twoworlds_power_state;
}
EXPORT_SYMBOL(twoworlds_power_get_state);

struct file_operations cb_native_fops = {
#ifndef USE_UNLOCK_IOCTL
	.ioctl = cb_ioctl,
#else
	.unlocked_ioctl = cb_ioctl,
#endif
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = cb_ioctl,
#endif
	.write = cb_write,
	.read = cb_read,
	.open = cb_open,
	.release = cb_release
};
EXPORT_SYMBOL(cb_native_fops);
