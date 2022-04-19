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
    Include Files
------------------------------------------------------------------------------*/
#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
//#include <asm/mach-types.h>
#include <asm/smp.h>
#include <mach/pm.h>
#include <mach/io.h>
#include <asm/cputype.h>
#include "chip_int.h"
#include "chip_setup.h"

#include <mach/system.h>
#include <asm/suspend.h>

#include "mdrv_types.h"
#include "mdrv_tee_general.h"
#include "uapi/linux/psci.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,10,86)
	extern int psci_cpu_suspend_enter(unsigned long index);
#else
	extern int cpu_psci_cpu_suspend(unsigned long index);
#endif


#if defined(CONFIG_MSTAR_PM)
#include "mdrv_pm.h"
#endif
#ifdef CONFIG_MP_MSTAR_STR_BASE
#include "mdrv_mpm.h"
#endif

extern void sleep_save_cpu_registers(void);
extern void sleep_set_wakeup_addr_phy(unsigned long phy_addr, void *virt_addr);
extern void sleep_prepare_last(void);
extern void save_performance_monitors(void *pointer);
extern void restore_performance_monitors(void *pointer);
extern void  MDrv_MBX_NotifyPMtoSetPowerOff(void);
extern void  MDrv_MBX_NotifyPMPassword(unsigned char passwd[16]);
extern int is_mstar_str(void);
#define PMREG(a) (*(volatile unsigned short*)(((unsigned long)mstar_pm_base)+(a)))
#define PMREG_BYTEMASK(s) (((unsigned int)0xFF)<<(s))
#define PMREG_BYTECLEAR(v,s) (((unsigned int)(v))&~(PMREG_BYTEMASK(s)))
#define PMREG_BYTE(v,s1,s2) (((((unsigned int)(v))>>(s1))&(0xFF))<<(s2))
#define PMREG_MAKVAL(v1,s1,v2,s2) (PMREG_BYTECLEAR(v1,s1)|PMREG_BYTE(v2,s2,s1))
#define PMON_SAVE_LONG_CNT 36
static unsigned long pmon_save_buf[PMON_SAVE_LONG_CNT];
DEFINE_SPINLOCK(ser_printf_lock);
static unsigned char pass_wd[16]={0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,
                                   0x11,0x00,0xFF,0xEE,0xDD,0xCC,0xBB,0xAA};

static u32 MStar_IntMaskSave[8];
static u32 MStar_HypIntMaskSave[8];

void SerPrintChar(char ch)
{
	unsigned long uart_base = 0x100980;
	while(!(*((volatile unsigned long*)(mstar_pm_base + uart_base* 2 + 0xA * 4)) & 0x20))
		;
	*(volatile unsigned long*)(mstar_pm_base + uart_base * 2) = ch;
}
void SerPrintStr(char *p)
{
    int nLen=strlen(p);
    int i;
    for(i=0;i<nLen;i++)
    {
        if(p[i]=='\n')SerPrintChar('\r');
        SerPrintChar(p[i]);
    }
}
void SerPrintStrAtomic(char *p)
{
    u_long flag;
    spin_lock_irqsave(&ser_printf_lock,flag);
    SerPrintStr(p);
    spin_unlock_irqrestore(&ser_printf_lock,flag);
}
void SerPrintf(char *fmt,...)
{
    char tmpbuf[500];
    int nLen;
    va_list args;
    va_start(args, fmt);
    nLen=vscnprintf(tmpbuf, 500, fmt, args);
    va_end(args);
    if(nLen<=0)
    {
        nLen=0;
    }
    else if(nLen>=500)
    {
        nLen=500-1;
    }
    tmpbuf[nLen]=0;
    SerPrintStr(tmpbuf);
}
void SerPrintfAtomic(char *fmt,...)
{
    char tmpbuf[500];
    int nLen;
    va_list args;
    va_start(args, fmt);
    nLen=vscnprintf(tmpbuf, 500, fmt, args);
    va_end(args);
    if(nLen<=0)
    {
        nLen=0;
    }
    else if(nLen>=500)
    {
        nLen=500-1;
    }
    tmpbuf[nLen]=0;
    SerPrintStrAtomic(tmpbuf);
}
int vSerPrintf(const char *fmt, va_list args)
{
    char tmpbuf[500];
    int nLen;
    nLen=vscnprintf(tmpbuf, 500, fmt, args);
    if(nLen<=0)
    {
        nLen=0;
    }
    else if(nLen>=500)
    {
        nLen=500-1;
    }
    tmpbuf[nLen]=0;
    SerPrintStr(tmpbuf);
    return nLen;
}
int vSerPrintfAtomic(const char *fmt, va_list args)
{
    char tmpbuf[500];
    int nLen;
    nLen=vscnprintf(tmpbuf, 500, fmt, args);
    if(nLen<=0)
    {
        nLen=0;
    }
    else if(nLen>=500)
    {
        nLen=500-1;
    }
    tmpbuf[nLen]=0;
    SerPrintStrAtomic(tmpbuf);
    return nLen;
}
phys_addr_t mstar_virt_to_phy(void* virtaddr)
{
    phys_addr_t rest=0;
    rest=virt_to_phys(virtaddr);
    return rest;
}

void* mstar_phy_to_virt(phys_addr_t phyaddr )
{
    void *rest=0;
    rest=phys_to_virt(phyaddr);
    return rest;
}

void mstar_sleep_cur_cpu_flush(void)
{
    Chip_Flush_Cache_All_Single();
}
static void mstar_str_notifypmmaxcnt_off(void)
{
    pass_wd[0x0A]=0xFD;
    MDrv_MBX_NotifyPMPassword(pass_wd);
    while(1);
}

#if defined(CONFIG_MSTAR_STR_ACOFF_ON_ERR)
void mstar_str_notifypmerror_off(void)
{
    pass_wd[0x0A]=0xFE;
    MDrv_MBX_NotifyPMPassword(pass_wd);
    while(1);
}
#endif

void mstar_pm_regw(unsigned short val)
{
    unsigned short tmp;
    tmp=PMREG(PMU_WAKEUP_ADDR_REGL);
    tmp=PMREG_MAKVAL(tmp,PMU_WAKEUP_ADDR_LSHIFT,val, 0);
    PMREG(PMU_WAKEUP_ADDR_REGL)=tmp;
    tmp=PMREG(PMU_WAKEUP_ADDR_REGH);
    tmp=PMREG_MAKVAL(tmp,PMU_WAKEUP_ADDR_HSHIFT,val, 8);
    PMREG(PMU_WAKEUP_ADDR_REGH)=tmp;
}
unsigned short mstar_pm_regr(void)
{
    unsigned short tmp,val=0;
    tmp=PMREG(PMU_WAKEUP_ADDR_REGL);
    val |= PMREG_BYTE(tmp,PMU_WAKEUP_ADDR_LSHIFT, 0);
    tmp=PMREG(PMU_WAKEUP_ADDR_REGH);
    val |= PMREG_BYTE(tmp,PMU_WAKEUP_ADDR_HSHIFT, 8);
    return val;
}

void mstar_prepare_secondary(void)
{
    extern int mstar_smp_spin_table_prepare_cpu(int cpu);

    mstar_smp_spin_table_prepare_cpu(1);
    mstar_smp_spin_table_prepare_cpu(2);
    mstar_smp_spin_table_prepare_cpu(3);
}
void mstar_save_int_mask(void)
{
    volatile unsigned long *int_mask_base=(volatile unsigned long *)REG_INT_BASE;
    volatile unsigned long *hypint_mask_base=(volatile unsigned long *)REG_INT_HYP_BASE;
    MStar_IntMaskSave[0]=int_mask_base[0x24];
    MStar_IntMaskSave[1]=int_mask_base[0x25];
    MStar_IntMaskSave[2]=int_mask_base[0x26];
    MStar_IntMaskSave[3]=int_mask_base[0x27];
    MStar_IntMaskSave[4]=int_mask_base[0x34];
    MStar_IntMaskSave[5]=int_mask_base[0x35];
    MStar_IntMaskSave[6]=int_mask_base[0x36];
    MStar_IntMaskSave[7]=int_mask_base[0x37];

    MStar_HypIntMaskSave[0]=hypint_mask_base[0x24];
    MStar_HypIntMaskSave[1]=hypint_mask_base[0x25];
    MStar_HypIntMaskSave[2]=hypint_mask_base[0x26];
    MStar_HypIntMaskSave[3]=hypint_mask_base[0x27];
    MStar_HypIntMaskSave[4]=hypint_mask_base[0x34];
    MStar_HypIntMaskSave[5]=hypint_mask_base[0x35];
    MStar_HypIntMaskSave[6]=hypint_mask_base[0x36];
    MStar_HypIntMaskSave[7]=hypint_mask_base[0x37];
}

//mstar_restore_int_mask use str_gic_dist_base to mask irq 31 in GIC SPI mode
#if defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
extern void __iomem* str_gic_dist_base;
#endif

void mstar_restore_int_mask(void)
{
#if defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
    u32 val;
#endif

    volatile unsigned long *int_mask_base=(volatile unsigned long *)REG_INT_BASE;
    volatile unsigned long *hypint_mask_base=(volatile unsigned long *)REG_INT_HYP_BASE;
    int_mask_base[0x24]=MStar_IntMaskSave[0];
    int_mask_base[0x25]=MStar_IntMaskSave[1];
    int_mask_base[0x26]=MStar_IntMaskSave[2];
    int_mask_base[0x27]=MStar_IntMaskSave[3];
    int_mask_base[0x34]=MStar_IntMaskSave[4];
    int_mask_base[0x35]=MStar_IntMaskSave[5];
    int_mask_base[0x36]=MStar_IntMaskSave[6];
    int_mask_base[0x37]=MStar_IntMaskSave[7];

    hypint_mask_base[0x24]=MStar_HypIntMaskSave[0];
    hypint_mask_base[0x25]=MStar_HypIntMaskSave[1];
    hypint_mask_base[0x26]=MStar_HypIntMaskSave[2];
    hypint_mask_base[0x27]=MStar_HypIntMaskSave[3];
    hypint_mask_base[0x34]=MStar_HypIntMaskSave[4];
    hypint_mask_base[0x35]=MStar_HypIntMaskSave[5];
    hypint_mask_base[0x36]=MStar_HypIntMaskSave[6];
    hypint_mask_base[0x37]=MStar_HypIntMaskSave[7];

#if defined(CONFIG_MP_PLATFORM_INT_1_to_1_SPI)
    val = readl_relaxed(str_gic_dist_base + 0x180 + (INT_PPI_IRQ / 32) * 4);
    val= val | (0x01 << INT_PPI_IRQ );
    writel_relaxed(val, str_gic_dist_base + 0x180 + (INT_PPI_IRQ / 32) * 4);
#endif


}

void mstar_suspend_save(void)
{
    extern ptrdiff_t mstar_pm_base;
#ifdef CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING
	unsigned int BIG_CLUSTER_CPUFREQ = *((volatile unsigned int *)(mstar_pm_base + (0x10050a << 1)));
#endif
	unsigned int LITTLE_CLUSTER_CPUFREQ = *((volatile unsigned int *)(mstar_pm_base + (0x100502 << 1)));
	SerPrintf("\nMStar STR Suspending...\n");

	LITTLE_CLUSTER_CPUFREQ &= 0xFFF;
	SerPrintf("\nSTR Suspending LITTLE_CLUSTER CPU FREQ %d\n", LITTLE_CLUSTER_CPUFREQ);

#ifdef CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING
	BIG_CLUSTER_CPUFREQ &= 0xFFF;
	SerPrintf("\nSTR Suspending BIG_CLUSTER CPU FREQ %d\n", BIG_CLUSTER_CPUFREQ);
#endif

    mstar_save_int_mask();
    save_performance_monitors(pmon_save_buf);
}
void mstar_resume_restore(void)
{
	extern ptrdiff_t mstar_pm_base;
	unsigned int PiuTick;
	unsigned int PiuTime;
#ifdef CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING
	unsigned int BIG_CLUSTER_CPUFREQ = *((volatile unsigned int *)(mstar_pm_base + (0x10050a << 1)));
#endif
	unsigned int LITTLE_CLUSTER_CPUFREQ = *((volatile unsigned int *)(mstar_pm_base + (0x100502 << 1)));
	SerPrintf("\nMStar STR Resuming...\n");

    MDrv_MPM_Set_Cnt(MDRV_MPM_KEY_STR_CNT, NULL);

	LITTLE_CLUSTER_CPUFREQ &= 0xFFF;
	SerPrintf("\nSTR Resuming LITTLE_CLUSTER CPU FREQ %d\n", LITTLE_CLUSTER_CPUFREQ);

#ifdef CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING
	BIG_CLUSTER_CPUFREQ &= 0xFFF;
	SerPrintf("\nSTR Resuming BIG_CLUSTER CPU FREQ %d\n", BIG_CLUSTER_CPUFREQ);
#endif

#if defined(CONFIG_MP_CHECKPT_BOOT)
#ifdef CONFIG_MP_PLATFORM_ARM
    PiuTick = reg_readw(0x1f006090UL);
    PiuTick += (reg_readw(0x1f006094UL) << 16);
#else
    PiuTick = *(volatile unsigned short *)(0xbf006090);
    PiuTick += (*(volatile unsigned short *)(0xbf006094)) << 16;
#endif
    PiuTime = PiuTick / 12000;
    SerPrintf("\n[AT][KR][STR Resuming][%u]\n", PiuTime);
#endif

    restore_performance_monitors(pmon_save_buf);

#if 0
#if defined(CONFIG_MP_PLATFORM_CPU_SETTING)
#if defined CONFIG_MSTAR_CPU_calibrating
    //Init Test Bus for CPU Clock Counter
    if(*(volatile u32 *)(mstar_pm_base + (0x100500 << 1)) == 0x3697)
   	{
		SerPrintf("Do Init Test Bus for CPU Clock Counter\n");
		*(volatile u32 *)(mstar_pm_base + 0x101896 *2) =0x01;
		*(volatile u32 *)(mstar_pm_base + 0x101eea *2) =0x00;
		*(volatile u32 *)(mstar_pm_base + 0x101eea *2) =0x04;
		*(volatile u32 *)(mstar_pm_base + 0x101eea *2) =0x4004;
		*(volatile u32 *)(mstar_pm_base + 0x101eee *2) =0x001f;
		*(volatile u32 *)(mstar_pm_base + 0x101e62 *2) = 0;
		*(volatile u32 *)(mstar_pm_base + 0x101e62 *2) =0x01;
   	}
#endif // defined CONFIG_MSTAR_CPU_calibrating
#endif /*MP_PLATFORM_CPU_SETTING*/
#endif
    mstar_restore_int_mask();
    mstar_prepare_secondary();
}

static void _mstar_str_enter_tee(void *pWakeup)
{
    extern uint32_t isPSCI;

    if ((TEEINFO_TYPTE==SECURITY_TEEINFO_OSTYPE_OPTEE) &&
        (isPSCI == PSCI_RET_SUCCESS))
    {
        printk("\033[0;32;31m [PSCI] %s %d %d %d %s %lx\033[m\n",__func__,__LINE__,current->pid,current->tgid,current->comm,KSTK_ESP(current));
        #if LINUX_VERSION_CODE > KERNEL_VERSION(3,10,86)
        psci_cpu_suspend_enter(1);
        #else
        cpu_psci_cpu_suspend(0);
        #endif
        while(1);
    }

}

void mstar_str_power_off(void)
{
    SerPrintf("\nMStar STR waiting power off...[%d]\n", MDrv_MPM_Get_Cnt(MDRV_MPM_KEY_STR_CNT));

    if (MDrv_MPM_Check_DC())
    {
		SerPrintf("Max Cnt Ac off...\n");
		if (is_mstar_str())
			mstar_str_notifypmmaxcnt_off();
	} else {
		SerPrintf("\ndo MDrv_MBX_NotifyPMtoSetPowerOff\n");
		if (is_mstar_str()) {
			SerPrintf("\nUser Mode STR, send password to 51...\n");
			MDrv_MBX_NotifyPMtoSetPowerOff();
		} else {
#if defined(CONFIG_MSTAR_PM)
			SerPrintf("\nKernel Mode STR, run 51...\n");
#endif
		}
	}
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,10,86)
unsigned long strsavearea[STRSAVE_AREA_COUNT];
#endif

/*------------------------------------------------------------------------------
    Function: mstar_pm_enter

    Description:
        Actually enter sleep state
    Input: (The arguments were used by caller to input data.)
        state - suspend state (not used)
    Output: (The arguments were used by caller to receive data.)
        None.
    Return:
        0
    Remark:
        None.
-------------------------------------------------------------------------------*/
static int mstar_pm_enter(suspend_state_t state)
{
    void *pWakeup=0;

    __asm__ volatile (
        "LDR X1, =MSTAR_WAKEUP_ENTRY\n"
        "STR X1, %0\n"
        :"=m"(pWakeup)::"r1");
    mstar_suspend_save();
    sleep_set_wakeup_addr_phy(mstar_virt_to_phy((void*)pWakeup),(void*)pWakeup);
    sleep_save_cpu_registers();
    sleep_prepare_last();
    mstar_sleep_cur_cpu_flush();

    mstar_str_power_off();

    _mstar_str_enter_tee(pWakeup);

    __asm__ volatile(
        "WAITHERE: B WAITHERE\n"
        "MSTAR_WAKEUP_ENTRY:\n"
        :::"r0","r1","r2","r3","r4","r5","r6","r7","r8","r9","r10","r11","r12","r13","r14","r15","memory","cc"
    );
    mstar_resume_restore();
    mstar_sleep_cur_cpu_flush();
    return 0;
}

#if defined(CONFIG_MSTAR_VOICE_DRIVER_HAL) && (CONFIG_MSTAR_VOICE_DRIVER_HAL == 1)
extern int voc_soc_shutdown(void);
#endif

static void mstar_pm_power_off_prepare(void)
{
#if defined(CONFIG_MSTAR_VOICE_DRIVER_HAL) && (CONFIG_MSTAR_VOICE_DRIVER_HAL == 1)
    voc_soc_shutdown();
#endif
    SerPrintf("mstar set pm power off param\n");
    MDrv_PM_Reboot(E_PM_STATE_POWER_OFF_PRE);
    mstar_sleep_cur_cpu_flush();
}

static void mstar_pm_power_off(void)
{
    SerPrintf("mstar pm power down\n");
    MDrv_PM_Reboot(E_PM_STATE_POWER_OFF);
    _mstar_str_enter_tee(NULL);
}

static struct platform_suspend_ops mstar_pm_ops =
{
    .enter      = mstar_pm_enter,
    .valid      = suspend_valid_only_mem,
};


/*------------------------------------------------------------------------------
    Function: mstar_pm_init

    Description:
        init function of power management
    Input: (The arguments were used by caller to input data.)
        None.
    Output: (The arguments were used by caller to receive data.)
        None.
    Return:
        0
    Remark:
        None.
-------------------------------------------------------------------------------*/
static int __init mstar_pm_init(void)
{
    pm_power_off_prepare = mstar_pm_power_off_prepare;
    pm_power_off = mstar_pm_power_off;
    /* set operation function of suspend */
    suspend_set_ops(&mstar_pm_ops);
    return 0;
}

__initcall(mstar_pm_init);

