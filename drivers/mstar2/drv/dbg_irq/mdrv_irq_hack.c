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

#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include "chip_int.h"
#include <trace/events/irq.h>
#include <linux/sysfs.h>


//#include "internals.h"
extern void irq_enable(struct irq_desc *desc);
extern void irq_disable(struct irq_desc *desc);
extern ptrdiff_t mstar_pm_base;

struct irq_stat
{
    unsigned int count[NR_IRQS];
    ktime_t interval_stamp[NR_IRQS];
    ktime_t entry_stamp;
};

#ifdef CONFIG_MP_POTENTIAL_BUG
static bool __read_mostly dbg_irq_enabled = true;
#else
static bool __read_mostly dbg_irq_enabled = false;
#endif
static unsigned int __read_mostly threshold_cnt = 30000;
static unsigned int __read_mostly threshold_interval = 1000;
static unsigned int __read_mostly threshold_latency = 1000;

static struct irq_stat *pirq_stat;

static spinlock_t irq_hacking_lock;

static struct kobject *dbg_irq_kobj=NULL;
static unsigned int helpinfo_size=(PAGE_SIZE);//default 1 page in case of showing help info

static int __init dbg_irq_setup(char *str)
{
    char *temp;
    char *p;
    int index = 0;

    if (*str++ != '=' || !*str)
        return -EINVAL;

    p = str;
    while((temp = strstr(p, ",")) != NULL)
    {
        p = temp + 1;
        index++;
    }

    if(index == 2)
        sscanf(str, "%u,%u,%u", &threshold_cnt, &threshold_interval, &threshold_latency);

    dbg_irq_enabled = true;

    return 1;
}
__setup("dbg_irq", dbg_irq_setup);

static irqreturn_t irq_hacking(int irq, void *dev_id)
{
    unsigned long flags;
    static int i = 3000;

    spin_lock_irqsave(&irq_hacking_lock,flags);
    while (i > 0)
    {
        mdelay(1);
        i--;
    }
    spin_unlock_irqrestore(&irq_hacking_lock, flags);

    return IRQ_HANDLED;
}

static ssize_t irq_hack_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{

    char *helpinfo = NULL;
    int j;
    helpinfo=vmalloc(helpinfo_size);
    if(!helpinfo)
    {
        pr_emerg("please echo test > /sys/kernel/dbg_irq/irq_hack for test\n");
        return 0;
    }

    j = sprintf( helpinfo, "####### Abnormal Interrupt Detector #######\n");
    j += sprintf( helpinfo + j, "----------------- Setting -----------------\n");
    j += sprintf( helpinfo + j, "1. Using default threshold values\n");
    j += sprintf( helpinfo + j, "ac dbg_irq 1 @ mboot \n");
    j += sprintf( helpinfo + j, "2. Using self-defining threshold values\n");
    j += sprintf( helpinfo + j, "ac dbg_irq threshold_cnt,threshold_interval,threshold_latency, Ex. ac dbg_irq 30000,500,1000\n");
    j += sprintf( helpinfo + j, "- Panic if interrupt frequency is over threshold_cnt (Times)/threshold_interval (milliseconds)\n");
    j += sprintf( helpinfo + j, "- Dump warning message if interrupt latency is over threshold_latency (milliseconds)\n");
    j += sprintf( helpinfo + j, "echo test > /sys/kernel/dbg_irq/irq_hack for test\n");
    pr_emerg("%s",helpinfo);
    vfree(helpinfo);
    return 0;

}


static ssize_t irq_hack_store(struct kobject *kobj, struct kobj_attribute *attr,    const char *buf, size_t count)
{
    char buffer[32];
    volatile void __iomem *reg = (volatile void __iomem *) (REG_IRQ_MASK_L - (4 << 2));

    strncpy(buffer, buf, count);
    buffer[count] = '\0';
    pr_emerg("you want to %s irq_hack.\n",buffer);
    if (!strncmp(buffer,"test",4)) {
        pr_err("Abnormal Interrupt detector is testing irq\n");

        if(irq_to_desc(34)->action)
            free_irq(34, NULL);

        spin_lock_init(&irq_hacking_lock);
        request_irq(34, irq_hacking, SA_INTERRUPT, "irq_hack_test", NULL);

        __raw_writeb(0, reg);
        irq_enable(irq_to_desc(34));
        __raw_writeb(4, reg);
    }

    return count;
}

static ssize_t threshold_cnt_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
    unsigned int temp_threshold_cnt = 0;
    int ret;
    ret = kstrtouint(buf, 10, &temp_threshold_cnt);
    if (ret)
        return ret;
    threshold_cnt = temp_threshold_cnt;
    printk("threshold_cnt = %d\n", threshold_cnt);
    return count;
}

static ssize_t threshold_interval_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
    unsigned int temp_threshold_interval = 0;
    int ret;
    ret = kstrtouint(buf, 10, &temp_threshold_interval);
    if (ret)
        return ret;
    threshold_interval = temp_threshold_interval;
    printk("threshold_interval = %d\n", temp_threshold_interval);
    return count;
}

static ssize_t threshold_latency_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
    unsigned int temp_threshold_latency = 0;
    int ret;
    ret = kstrtouint(buf, 10, &temp_threshold_latency);
    if (ret)
        return ret;
    threshold_latency = temp_threshold_latency;
    printk("threshold_latency = %d\n", temp_threshold_latency);
    return count;
}

static inline void irq_handler_entry_probe(void *data, int irq, struct irqaction *action)
{
    struct irq_stat *irq_stat;
    unsigned long delta_ms;
    int count = 0;

    if (action && (action->flags & IRQF_PERCPU))//do not check ppi
        return;

    if (irq >= NR_IRQS) {
        WARN_ONCE(irq >= NR_IRQS, "IRQ%d is enlarged NR_IRQS (%d), can not trace\n", irq, NR_IRQS);
        return;
    }

    irq_stat = this_cpu_ptr(data);
    irq_stat->entry_stamp = ktime_get();

    if (unlikely(irq_stat->count[irq] > threshold_cnt)) {
        delta_ms =  ktime_to_ms(ktime_sub(ktime_get(), irq_stat->interval_stamp[irq]));

        if (delta_ms < threshold_interval) {
            pr_emerg("Abnormal interrupt detected!!! over %u interrupts within %lu ms. IRQ%d, %s @ CPU%d\n",
                irq_stat->count[irq], delta_ms, irq, irq_to_desc(irq)->action->name, smp_processor_id());
            pr_emerg("Try to disable IRQ%d to ease the system\n", irq);

            irq_disable(irq_to_desc(irq));
            // You can dump related registers befor panic.
            if (irq == 84)
            {
                for(count=0; count<4; count++)
                {
                    switch(count)
                    {
                        case 0:
                            pr_emerg("@@ Bank:0x1710, [8bit] 0x3B[0x%x],0x3C[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171000+0x3B)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171000+0x3C)*2));

                            pr_emerg("@@ Bank:0x1712, [8bit] 0x66[0x%x],0x67[0x%x],0x01[0x%x],0x15[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171200+0x66)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171200+0x67)*2),
                                *(volatile unsigned short *)(mstar_pm_base + (0x171200+0x01)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171200+0x15)*2)); 

                            break;

                        case 1:
                            pr_emerg("@@ Bank:0x1714, [8bit] 0x3B[0x%x],0x3C[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171400+0x3B)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171400+0x3C)*2));

                            pr_emerg("@@ Bank:0x1716, [8bit] 0x66[0x%x],0x67[0x%x],0x01[0x%x],0x15[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171600+0x66)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171600+0x67)*2),
                                *(volatile unsigned short *)(mstar_pm_base + (0x171600+0x01)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171600+0x15)*2)); 

                            break;

                        case 2:
                            pr_emerg("@@ Bank:0x1718, [8bit] 0x3B[0x%x],0x3C[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171800+0x3B)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171800+0x3C)*2));

                            pr_emerg("@@ Bank:0x171A, [8bit] 0x66[0x%x],0x67[0x%x],0x01[0x%x],0x15[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171A00+0x66)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171A00+0x67)*2),
                                *(volatile unsigned short *)(mstar_pm_base + (0x171A00+0x01)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171A00+0x15)*2)); 

                            break;

                        case 3:
                            pr_emerg("@@ Bank:0x171C, [8bit] 0x3B[0x%x],0x3C[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171C00+0x3B)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171C00+0x3C)*2));

                            pr_emerg("@@ Bank:0x171E, [8bit] 0x66[0x%x],0x67[0x%x],0x01[0x%x],0x15[0x%x]\n",
                                *(volatile unsigned short *)(mstar_pm_base + (0x171E00+0x66)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171E00+0x67)*2),
                                *(volatile unsigned short *)(mstar_pm_base + (0x171E00+0x01)*2),*(volatile unsigned short *)(mstar_pm_base + (0x171E00+0x15)*2)); 

                            break;

                        default:
                            break;
                    };
                }
            } 
            panic("MTK panic, Interrupt Overload @ IRQ%d, %s", irq, irq_to_desc(irq)->action->name);
        }

        irq_stat->count[irq] = 0;
        irq_stat->interval_stamp[irq] = ktime_get();
    }
    irq_stat->count[irq]++;
}

static void irq_handler_exit_probe(void *data, int irq, struct irqaction *action, int res)
{
    struct irq_stat *irq_stat;
    unsigned long latency_ms;

    if (action && (action->flags & IRQF_PERCPU))//do not check ppi
        return;

    if (irq >= NR_IRQS) {
        WARN_ONCE(irq >= NR_IRQS, "IRQ%d is enlarged NR_IRQS (%d), can not trace\n", irq, NR_IRQS);
        return;
    }

    irq_stat = this_cpu_ptr(data);
    latency_ms = ktime_to_ms(ktime_sub(ktime_get(), irq_stat->entry_stamp));

    if (unlikely(latency_ms >= threshold_latency)) {
        pr_emerg("\nAbnormal interrupt behavior detected!!! IRQ%d, %s @ CPU%d took %lu ms\n\n",
            irq, irq_to_desc(irq)->action->name, smp_processor_id(), latency_ms);
#ifdef CONFIG_MP_POTENTIAL_BUG
        panic("MTK panic, Interrupt took too long @ IRQ%d, %s", irq, irq_to_desc(irq)->action->name);
#endif
    }
}

static struct kobj_attribute irq_hack_attr =__ATTR_RW(irq_hack);

static struct kobj_attribute threshold_cnt_attr =__ATTR_WO(threshold_cnt);

static struct kobj_attribute threshold_interval_attr =__ATTR_WO(threshold_interval);

static struct kobj_attribute threshold_latency_attr =__ATTR_WO(threshold_latency);

//static const struct attribute *dbg_irq_device_attrs[] = {
static struct attribute *dbg_irq_device_attrs[] = {
    &irq_hack_attr.attr,
    &threshold_cnt_attr.attr,
    &threshold_interval_attr.attr,
    &threshold_latency_attr.attr,
    NULL,
};

ATTRIBUTE_GROUPS(dbg_irq_device);

static int __init mdrv_dbg_irq_init(void)
{
    int ret = 0;

    if (!dbg_irq_enabled)
        return ret;

    pirq_stat = alloc_percpu(struct irq_stat);

    if (!pirq_stat) {
        ret = -ENOMEM;
        goto fail;
    }

    ret = register_trace_irq_handler_entry(irq_handler_entry_probe, pirq_stat);
    if(ret) {
        free_percpu(pirq_stat);
        goto fail;
    }

    ret = register_trace_irq_handler_exit(irq_handler_exit_probe,pirq_stat);
    if(ret) {
        free_percpu(pirq_stat);
        goto fail;
    }

    dbg_irq_kobj = kobject_create_and_add("dbg_irq", kernel_kobj);

    if(!dbg_irq_kobj)
    {
        ret = -ENOMEM;
        goto fail;
    }

    //ret = sysfs_create_files(dbg_irq_kobj, dbg_irq_device_attrs);
    ret = sysfs_create_groups(dbg_irq_kobj, dbg_irq_device_groups);
    if (ret) {
        kobject_put(dbg_irq_kobj);
        goto fail;
    }


    pr_info("Abnomal interrupt detector initialized, frequency threshold is %u/%u (times/ms) and latency threshold is %u (ms)\n",
        threshold_cnt, threshold_interval, threshold_latency);
    return ret;

fail:
    pr_err("[%s] init fail, error code %d\n", __func__, ret);
    return ret;

}

static void __exit mdrv_dbg_irq_exit(void)
{
    if (!dbg_irq_enabled)
        return;

    sysfs_remove_groups(dbg_irq_kobj, dbg_irq_device_groups);
        kobject_put(dbg_irq_kobj);

    unregister_trace_irq_handler_entry(irq_handler_entry_probe,pirq_stat);
    unregister_trace_irq_handler_exit(irq_handler_exit_probe,pirq_stat);
    free_percpu(pirq_stat);
}

module_init(mdrv_dbg_irq_init);
module_exit(mdrv_dbg_irq_exit);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("Abnormal Interrupt Detector");
MODULE_LICENSE("GPL");
