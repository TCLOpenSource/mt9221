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
#include <linux/module.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/version.h>

#include "chip_int.h"
#include "mdrv_types.h"
#include "mdrv_rtc.h"
#include "mdrv_pm.h"
#if defined(CONFIG_OF)
#include <linux/of.h>
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include "mdrv_system.h"
#include <linux/seq_file.h>
#endif /* CONFIG_MSTAR_UTOPIA2K_MDEBUG */

struct mstar_rtc_plat_data {
    u32 index;
    u32 xtal;
    u32 freq;
    u32 default_cnt;
};

#if !defined(CONFIG_OF)
static struct mstar_rtc_plat_data mstar_rtc_pdata = {
        .index = 0,
        .xtal = 12000000,  // external clock reference @ 12MHz
        .freq = 1,         // RTC update frequency in second
        .default_cnt = 946684800,
};

struct platform_device mstar_rtc_pdev = {
        .name = "mstar-rtc",
        .dev = {
            .platform_data = &mstar_rtc_pdata,
        }
};
MODULE_DEVICE_TABLE(platform, mstar_rtc_pdev);
#endif

struct mstar_rtc_dev {
	struct platform_device *pdev;
	struct rtc_device *rtc;
	PM_RtcParam mstar_rtc_param;
	spinlock_t mstar_rtc_lock;
	struct mutex mutex_lock;
};

#if defined (CONFIG_ANDROID)
static u8 rtc_int_is_enabled = 0;
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
static struct proc_dir_entry *mdb_rtc_proc_entry = NULL;
static bool mdb_rtc_is_wakeup = TRUE;
static int mdb_rtc_node_open(struct inode *inode, struct file *file);
static ssize_t mdb_rtc_node_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
const struct file_operations mdb_rtc_node_operations = {
    .owner      = THIS_MODULE,
    .open       = mdb_rtc_node_open,
    .read       = seq_read,
    .write      = mdb_rtc_node_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static ssize_t mdb_rtc_node_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char tmpString[20] = {};

    if (!count) {
        printk("count is 0\n");
        return 0;
    }

    if (count > (sizeof(tmpString) - 1)) {
        printk("input is too big\n");
        return -EINVAL;
    }

    if (copy_from_user(tmpString, buf, count)) {
        printk("copy_from_user failed\n");
        return -EFAULT;
    }

    if (!strncmp(tmpString, "help", 4)) {
        printk("\n--------- MStar RTC Help ---------\n");
        printk("  Get RTC Information:\n");
        printk("    cat /proc/utopia_mdb/rtc\n");
        printk("  Disable RTC wakeup:\n");
        printk("    echo wakeup_disable > /proc/utopia_mdb/rtc\n");
        printk("  Enable RTC wakeup:\n");
        printk("    echo wakeup_enable > /proc/utopia_mdb/rtc\n");
    } else if (!strncmp(tmpString, "wakeup_disable", 14)) {
        mdb_rtc_is_wakeup = FALSE;
        printk("Disable RTC wakeup\n");
    } else if (!strncmp(tmpString, "wakeup_enable", 13)) {
        mdb_rtc_is_wakeup = TRUE;
        printk("Enable RTC wakeup\n");
    }

    return count;
}

static int mdb_rtc_node_show(struct seq_file *m, void *v)
{
    seq_printf(m, "\n--------- MStar MBX Info ---------\n");

    seq_printf(m, " MStar RTC Debug Usage:\n");
    seq_printf(m, "  echo help > /proc/utopia_mdb/rtc\n\n");

    seq_printf(m, " RTC wakeup state: %s\n", (mdb_rtc_is_wakeup ? "TRUE" : "FALSE"));

    return 0;
}

static int mdb_rtc_node_open(struct inode *inode, struct file *file)
{
    return single_open(file, mdb_rtc_node_show, NULL);
}
#endif /* CONFIG_MSTAR_UTOPIA2K_MDEBUG */

static int mstar_rtc_get_time(struct device *dev, struct rtc_time *tm)
{
    struct timeval time;
    PM_RtcParam pmRtcParam;

    pmRtcParam.u8PmRtcIndex = 0;
    time.tv_sec = (__kernel_time_t)MDrv_RTC_GetCount(&pmRtcParam);

    rtc_time_to_tm(time.tv_sec, tm);
    return rtc_valid_tm(tm);
}

static int mstar_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
    unsigned long time;
    PM_RtcParam pmRtcParam;

    rtc_tm_to_time(tm, &time);
    pmRtcParam.u8PmRtcIndex = 0;
    pmRtcParam.u32RtcSetCounter = (u32)time;
    MDrv_RTC_SetCount(&pmRtcParam);
    return 0;
}

static irqreturn_t mstar_rtc_irq_handler(int irq, void *data)
{
	struct device *dev = data;
	struct mstar_rtc_dev *mstar_rtc = dev_get_drvdata(dev);
	unsigned long events = 0;

	events |= RTC_IRQF | RTC_AF;
	rtc_update_irq(mstar_rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

static int mstar_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct mstar_rtc_dev *mstar_rtc = dev_get_drvdata(dev);
	unsigned long irq_flags;

#if defined (CONFIG_ANDROID)
	mutex_lock(&mstar_rtc->mutex_lock);
	if (enable)
	{
		spin_lock_irqsave(&mstar_rtc->mstar_rtc_lock, irq_flags);
		if (!rtc_int_is_enabled)
		{
			//enable_irq(E_IRQ_PM_SLEEP);
			MDrv_RTC_Enable_Interrupt();
			rtc_int_is_enabled = 1;
		}
		spin_unlock_irqrestore(&mstar_rtc->mstar_rtc_lock, irq_flags);
	}
	else
	{
		if (rtc_int_is_enabled)
		{
			//disable_irq(E_IRQ_PM_SLEEP);
			MDrv_RTC_Disable_Interrupt();
			rtc_int_is_enabled = 0;
		}
	}
	mutex_unlock(&mstar_rtc->mutex_lock);
#endif

	return 0;
}

static int mstar_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	PM_RtcParam pmRtcParam;
	unsigned long sec;

	printk(KERN_ERR "mstar-rtc %s\n", __func__);

	if (alarm->enabled)
		rtc_tm_to_time(&alarm->time, &sec);
	else
		sec = 0;

	pmRtcParam.u8PmRtcIndex = 0;
	pmRtcParam.u32RtcSetMatchCounter = (u32)sec;
	MDrv_RTC_SetMatchCount((PM_RtcParam*)&pmRtcParam);

	if (sec) {
		mstar_rtc_alarm_irq_enable(dev, 1);
		dev_vdbg(dev, "alarm set as %lu. %d/%d/%d %d:%02u:%02u\n",
			sec,
			alarm->time.tm_mon+1,
			alarm->time.tm_mday,
			alarm->time.tm_year+1900,
			alarm->time.tm_hour,
			alarm->time.tm_min,
			alarm->time.tm_sec);
	} else {
		/* disable alarm */
		dev_vdbg(dev, "alarm disabled\n");
		mstar_rtc_alarm_irq_enable(dev, 0);
	}

	return 0;
}

static int mstar_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	unsigned long sec;
	PM_RtcParam pmRtcParam;
	pmRtcParam.u8PmRtcIndex = 0;
	sec = MDrv_RTC_GetMatchCount(&pmRtcParam);

	if (sec == 0xFFFFFFFF) {
		alarm->enabled = 0;
	} else {
		alarm->enabled = 1;
		rtc_time_to_tm(sec, &alarm->time);
	}

    return 0;
}

static int mstar_rtc_proc(struct device *dev, struct seq_file *seq)
{
    printk(KERN_INFO "mstar-rtc %s: currently not supported\n", __func__);

    return 0;
}

static const struct rtc_class_ops mstar_rtc_ops = {
	.proc = mstar_rtc_proc,
	.read_time = mstar_rtc_get_time,
	.set_time = mstar_rtc_set_time,
	.read_alarm = mstar_rtc_read_alarm,
	.set_alarm = mstar_rtc_set_alarm,
	.alarm_irq_enable = mstar_rtc_alarm_irq_enable,
};

#if defined(CONFIG_OF)
static const struct of_device_id mstar_rtc_dt_match[] = {
	{ .compatible = "mstar-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, mstar_rtc_dt_match);
#endif


#if defined(CONFIG_OF)
static int mstar_rtc_parse_dt(struct device_node *dn, struct mstar_rtc_dev *mstar_rtc)
{
    u32 rtc_index = 0;
    u32 freq = 0;
    u32 xtal = 0;
    u32 rtc_default_cnt = 0;

    if ((of_property_read_u32(dn, "index", &rtc_index) != 0) ||
        (of_property_read_u32(dn, "xtal", &xtal) != 0)       ||
        (of_property_read_u32(dn, "freq", &freq) != 0)) {
        printk(KERN_ERR "Mstar-rtc parse DT error\n");
        return -1;
    }

    if (freq < 0)
        return -1;

    mstar_rtc->mstar_rtc_param.u8PmRtcIndex = rtc_index;
    mstar_rtc->mstar_rtc_param.u32RtcCtrlWord = xtal / freq;

    if (of_property_read_u32(dn, "default_cnt", &rtc_default_cnt) != 0) {
        printk(KERN_NOTICE "mstar-rtc parse dt default_cnt fail, set default to 946684800\n");
        rtc_default_cnt = 946684800;
    }

    mstar_rtc->mstar_rtc_param.u32RtcDefaultAcCounter = rtc_default_cnt;
#if 0
    printk("rtc index = %d\n", rtc_index);
    printk("rtc xtal = %d\n", xtal);
    printk("rtc update freq = %d\n", freq);
    printk("rtc cw = %d\n", mstar_rtc->mstar_rtc_param.u32RtcCtrlWord);
#endif
    return 0;
}
#endif

static int __init mstar_rtc_probe(struct platform_device *pdev)
{
    struct mstar_rtc_dev *mstar_rtc;
    PM_RtcParam pmRtcParam;
    int ret = 0;
#if defined(CONFIG_OF)
    struct device_node * dn;
#else
    struct mstar_rtc_plat_data *mstar_pdata;
#endif
    u32 rtc_count;
    u8 rtc_index;

    printk(KERN_INFO "mstar-rtc probe\n");

    if (!device_can_wakeup(&pdev->dev))
        device_init_wakeup(&pdev->dev, 1);

    mstar_rtc = devm_kzalloc(&pdev->dev, sizeof(struct mstar_rtc_dev), GFP_KERNEL);
    if (!mstar_rtc) {
        dev_err(&pdev->dev, "unable to allocate memory for mstar-rtc\n");
        return -ENOMEM;
    }

#if defined(CONFIG_OF)
    printk(KERN_INFO "mstar-rtc: DTS\n");
    dn = pdev->dev.of_node;
    if (dn) {
        ret = mstar_rtc_parse_dt(dn, mstar_rtc);
        if (ret < 0) {
            dev_err(&pdev->dev, "failed to parse RTC device tree\n");
            return -1;
        }
    }
#else // platform device
    printk(KERN_INFO "mstar-rtc: platform_device\n");
    mstar_pdata = (struct mstar_rtc_plat_data *)pdev->dev.platform_data;
    mstar_rtc->mstar_rtc_param.u8PmRtcIndex = mstar_pdata->index;
    // clock freq = xtal / cw
    mstar_rtc->mstar_rtc_param.u32RtcCtrlWord = mstar_pdata->xtal / mstar_pdata->freq;
    mstar_rtc->mstar_rtc_param.u32RtcDefaultAcCounter = mstar_pdata->default_cnt;
#endif

    spin_lock_init(&mstar_rtc->mstar_rtc_lock);
    mutex_init(&mstar_rtc->mutex_lock);

    platform_set_drvdata(pdev, mstar_rtc);

    MDrv_RTC_Init(&mstar_rtc->mstar_rtc_param);

    rtc_count = MDrv_RTC_GetCount(&mstar_rtc->mstar_rtc_param);

    if (rtc_count <= mstar_rtc->mstar_rtc_param.u32RtcDefaultAcCounter) {
        // provide an initial counter value for RTC to avoid hctosys return error.
        mstar_rtc->mstar_rtc_param.u32RtcSetCounter = mstar_rtc->mstar_rtc_param.u32RtcDefaultAcCounter;
        MDrv_RTC_SetCount(&mstar_rtc->mstar_rtc_param);
        printk(KERN_INFO "[%s %d] set rtc count to %d\n", __FUNCTION__, __LINE__, mstar_rtc->mstar_rtc_param.u32RtcDefaultAcCounter);
#if !defined(CONFIG_MSTAR_M7322) && !defined(CONFIG_MSTAR_M5621)
        /* switch dummy reg usage from rtc0 to rtc1 due to brick terminator */
        rtc_index = mstar_rtc->mstar_rtc_param.u8PmRtcIndex;
        mstar_rtc->mstar_rtc_param.u8PmRtcIndex = 1;
        mstar_rtc->mstar_rtc_param.u8PmRtcDummyIndex = 0;
        MDrv_RTC_SetDummy(&mstar_rtc->mstar_rtc_param, 0);
        mstar_rtc->mstar_rtc_param.u8PmRtcDummyIndex = 1;
        MDrv_RTC_SetDummy(&mstar_rtc->mstar_rtc_param, 0);
        /* restore rtc back */
        mstar_rtc->mstar_rtc_param.u8PmRtcIndex = rtc_index;
#endif /* !CONFIG_MSTAR_M7322 && !CONFIG_MSTAR_M5621 */
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,108)
    mstar_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
    if (IS_ERR(mstar_rtc->rtc))
    {
        printk(KERN_ERR "mstar-rtc: fail to allocate RTC device: %ld\n", PTR_ERR(mstar_rtc->rtc));
        return PTR_ERR(mstar_rtc->rtc);
    }

    mstar_rtc->rtc->ops = &mstar_rtc_ops;

    ret = rtc_register_device(mstar_rtc->rtc);
    if (ret) {
        dev_err(&pdev->dev, "unable to register device\n");
        return ret;
    }
#else
     mstar_rtc->rtc = rtc_device_register("mstar-rtc", &pdev->dev, &mstar_rtc_ops, THIS_MODULE);

     if (IS_ERR(mstar_rtc->rtc))
     {
         printk(KERN_ERR "mstar-rtc: fail to register RTC device: %ld\n", PTR_ERR(mstar_rtc->rtc));
         return PTR_ERR(mstar_rtc->rtc);
     }
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,108) */

#if defined (CONFIG_ANDROID)
    ret = Request_RTC_IRQ(mstar_rtc_irq_handler, &pdev->dev);
    if (ret)
    {
        printk(KERN_ERR "mstar-rtc: fail to request irq\n");
        return ret;
    }

    //disable_irq(E_IRQ_PM_SLEEP);
#endif

    printk(KERN_INFO "mstar-rtc: rtc%d registered\n", mstar_rtc->rtc->id);
    printk(KERN_DEBUG "mstar-rtc: control word = %d\n", mstar_rtc->mstar_rtc_param.u32RtcCtrlWord);

    return 0;
}

static int __exit mstar_rtc_remove(struct platform_device *pdev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,108)
#else
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	rtc_device_unregister(rtc);
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,108) */

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mstar_rtc_suspend(struct device *pdev)
{
    struct mstar_rtc_dev *mstar_rtc = dev_get_drvdata(pdev);
#ifdef CONFIG_MSTAR_PM
    PM_WakeCfg_t buf;

    memset(&buf, 0, sizeof(PM_WakeCfg_t));
    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (char *)&buf);
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    if (mdb_rtc_is_wakeup == FALSE)
        buf.bPmWakeEnableRTC0 = 0x0;
    else
        buf.bPmWakeEnableRTC0 = 0x1;
#else
    buf.bPmWakeEnableRTC0 = 0x1;
#endif /* CONFIG_MSTAR_UTOPIA2K_MDEBUG */

    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (char *)&buf, sizeof(PM_WakeCfg_t));
#endif

    printk("%s\n", __func__);
    return 0;
}

static int mstar_rtc_resume(struct device *dev)
{
    printk("%s\n", __func__);
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mstar_rtc_pm_ops, mstar_rtc_suspend, mstar_rtc_resume);

static struct platform_driver mstar_rtc_driver = {
    .probe = mstar_rtc_probe,
	.driver = {
		.name = "mstar-rtc",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = mstar_rtc_dt_match,
#endif
        .pm	= &mstar_rtc_pm_ops,
	},
	.remove = __exit_p(mstar_rtc_remove),
};

static int __init mstar_rtc_init(void)
{
    int ret;

    ret = platform_driver_register(&mstar_rtc_driver);
    if (ret < 0) {
        printk(KERN_ERR "mstar-rtc: fail to register RTC platform driver\n");
        return ret;
    }

#if !defined(CONFIG_OF)
    printk(KERN_DEBUG "mstar-rtc: RTC platform driver\n");
    ret = platform_device_register(&mstar_rtc_pdev);
    if (ret < 0) {
        printk(KERN_ERR "mstar-rtc: fail to register RTC platform device\n");
        platform_driver_unregister(&mstar_rtc_driver);
        return ret;
    }
#endif

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    MDrv_SYS_UtopiaMdbMkdir();
    mdb_rtc_proc_entry = proc_create("utopia_mdb/rtc", (S_IRUGO|S_IWUGO), NULL, &mdb_rtc_node_operations);
    if (mdb_rtc_proc_entry == NULL)
        printk(KERN_ERR "mstar-rtc: unable to create proc node\n");
#endif /* CONFIG_MSTAR_UTOPIA2K_MDEBUG */

    printk(KERN_DEBUG "mstar-rtc: init done\n");

	return 0;
}

static void __exit mstar_rtc_fini(void)
{
	platform_driver_unregister(&mstar_rtc_driver);
#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
    if (mdb_rtc_proc_entry != NULL)
        proc_remove(mdb_rtc_proc_entry);
#endif /* CONFIG_MSTAR_UTOPIA2K_MDEBUG */
}

module_init(mstar_rtc_init);
module_exit(mstar_rtc_fini);

MODULE_AUTHOR("Mstarsemi");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mstar RTC driver");
MODULE_ALIAS("platform:mstar-rtc");
