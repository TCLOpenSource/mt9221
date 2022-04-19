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
#include <asm/mach-types.h>
#include <asm/smp.h>
#include <mach/pm.h>
#include <mach/io.h>
#include <asm/cputype.h>
#include <asm/tlbflush.h>
#include "chip_int.h"
#include "chip_setup.h"
#include "sleep_helper.h"

#include <asm/psci.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/psci.h>
#include <uapi/linux/psci.h>
#endif
#ifdef CONFIG_MP_MSTAR_STR_BASE
#include "mdrv_mpm.h"
#endif
#include "mdrv_pm.h"
#include "mdrv_types.h"
#include "mdrv_tee_general.h"

#ifdef CONFIG_MSTAR_STR_CRC
#include <asm/arm-boards/prom.h>
#include <linux/delay.h>
#define CRC_PATTERN_REG1        0xC0
#define CRC_PATTERN_REG2        0xC2
#define CRC_LX1_REG_ST          0xC4
#define CRC_LX1_REG_SZ          0xC6
#define CRC_LX2_REG_ST          0xC8
#define CRC_LX2_REG_SZ          0xCA
#define CRC_LX3_REG_ST          0xCC
#define CRC_LX3_REG_SZ          0xCE
#define IS_MIU0                 (0x0 << 10)
#define IS_MIU1                 (0x1 << 10)
#define MBX_BLANK               0x103300
#define STR_CRC_REG(n)       *(volatile int*)(0xFD000000+((MBX_BLANK+(n))<<1))
#endif

#define WAKEUP_SAVE_ADDR PAGE_OFFSET
#define INT_MASK_REG_BASE IO_ADDRESS(REG_INT_BASE_PA)

extern void sleep_save_cpu_registers(void * pbuffer);
extern void sleep_restore_cpu_registers(void * pbuffer);
extern void sleep_set_wakeup_save_addr_phy(unsigned long phy_addr, void *virt_addr);
extern void sleep_clear_wakeup_save_addr_phy(unsigned long phy_addr, void *virt_addr);
extern void sleep_prepare_last(unsigned long wakeup_addr_phy);
extern void sleep_wakeup_first(unsigned long boot_start,void *exit_addr_virt);
extern void sleep_save_neon_regs(void * pbuffer);
extern void sleep_restore_neon_regs(void * pbuffer);
extern void MDrv_MBX_NotifyPMtoSetPowerOff(void);
extern void MDrv_MBX_NotifyPMPassword(unsigned char passwd[16]);
extern int is_mstar_str(void);
extern void SerPrintf(char *fmt,...);
extern void SerPrintfAtomic(char *fmt,...);
extern int platform_smp_boot_secondary_init(unsigned int cpu);
extern int platform_smp_boot_secondary_reset(unsigned int cpu);

extern void __iomem *_gic_cpu_base_addr;
extern void __iomem *_gic_dist_base_addr;

static u32 MStar_Suspend_Buffer[SLEEPDATA_SIZE];
static u32 MStar_IntMaskSave[8];
DEFINE_SPINLOCK(ser_printf_lock);
static unsigned char pass_wd[16] = {
	0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22,
	0x11, 0x00, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

unsigned long strsavearea[STRSAVE_AREA_COUNT];

#if defined(CONFIG_MP_PLATFORM_FIQ_IRQ_HYP)
static u32 MStar_HypIntMaskSave[8];
#endif

void SerPrintChar(char ch)
{
	unsigned long uart_base = 0x100980;
	while(!(*((volatile unsigned long*)(mstar_pm_base + uart_base * 2 + 0xA * 4)) && 0x20))
		;
	*(volatile unsigned long*)(mstar_pm_base + 0x100980 * 2) = ch;
}

void SerPrintStr(char *p)
{
	int i;
	int nLen = strlen(p);

	for (i = 0; i < nLen;i ++) {
		if(p[i]=='\n')
			SerPrintChar('\r');
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
	if(nLen <= 0)
		nLen = 0;
	else if(nLen >= 500)
		nLen = 500 - 1;
	tmpbuf[nLen] = 0;
	SerPrintStr(tmpbuf);
}

#ifdef CONFIG_MP_PLATFORM_UTOPIA2K_EXPORT_SYMBOL
EXPORT_SYMBOL(SerPrintf);
#endif

void SerPrintfAtomic(char *fmt,...)
{
	char tmpbuf[500];
	int nLen;
	va_list args;

	va_start(args, fmt);
	nLen = vscnprintf(tmpbuf, 500, fmt, args);
	va_end(args);
	if(nLen <= 0)
		nLen = 0;
	else if(nLen >= 500)
		nLen = 500 - 1;
	tmpbuf[nLen] = 0;
	SerPrintStrAtomic(tmpbuf);
}

int vSerPrintf(const char *fmt, va_list args)
{
	char tmpbuf[500];
	int nLen;

	nLen = vscnprintf(tmpbuf, 500, fmt, args);
	if (nLen <= 0)
		nLen = 0;
	else if(nLen >= 500)
		nLen = 500-1;
	tmpbuf[nLen] = 0;
	SerPrintStr(tmpbuf);

	return nLen;
}

int vSerPrintfAtomic(const char *fmt, va_list args)
{
	char tmpbuf[500];
	int nLen;

	nLen = vscnprintf(tmpbuf, 500, fmt, args);
	if(nLen <= 0)
		nLen = 0;
	else if(nLen >= 500)
		nLen = 500-1;
	tmpbuf[nLen] = 0;
	SerPrintStrAtomic(tmpbuf);

	return nLen;
}

void mstar_save_int_mask(void)
{
	volatile unsigned long *int_mask_base = (volatile unsigned long *)INT_MASK_REG_BASE;
#if defined(CONFIG_MP_PLATFORM_FIQ_IRQ_HYP)
	volatile unsigned long *hypint_mask_base = (volatile unsigned long *)REG_INT_HYP_BASE;
#endif
	MStar_IntMaskSave[0] = int_mask_base[0x24];
	MStar_IntMaskSave[1] = int_mask_base[0x25];
	MStar_IntMaskSave[2] = int_mask_base[0x26];
	MStar_IntMaskSave[3] = int_mask_base[0x27];
	MStar_IntMaskSave[4] = int_mask_base[0x34];
	MStar_IntMaskSave[5] = int_mask_base[0x35];
	MStar_IntMaskSave[6] = int_mask_base[0x36];
	MStar_IntMaskSave[7]  =int_mask_base[0x37];
#if defined(CONFIG_MP_PLATFORM_FIQ_IRQ_HYP)
	MStar_HypIntMaskSave[0] = hypint_mask_base[0x24];
	MStar_HypIntMaskSave[1] = hypint_mask_base[0x25];
	MStar_HypIntMaskSave[2] = hypint_mask_base[0x26];
	MStar_HypIntMaskSave[3] = hypint_mask_base[0x27];
	MStar_HypIntMaskSave[4] = hypint_mask_base[0x34];
	MStar_HypIntMaskSave[5] = hypint_mask_base[0x35];
	MStar_HypIntMaskSave[6] = hypint_mask_base[0x36];
	MStar_HypIntMaskSave[7] = hypint_mask_base[0x37];
#endif
}

void mstar_restore_int_mask(void)
{
	volatile unsigned long *int_mask_base=(volatile unsigned long *)INT_MASK_REG_BASE;
#if defined(CONFIG_MP_PLATFORM_FIQ_IRQ_HYP)
	volatile unsigned long *hypint_mask_base=(volatile unsigned long *)REG_INT_HYP_BASE;
#endif
	int_mask_base[0x24] = MStar_IntMaskSave[0];
	int_mask_base[0x25] = MStar_IntMaskSave[1];
	int_mask_base[0x26] = MStar_IntMaskSave[2];
	int_mask_base[0x27] = MStar_IntMaskSave[3];
	int_mask_base[0x34] = MStar_IntMaskSave[4];
	int_mask_base[0x35] = MStar_IntMaskSave[5];
	int_mask_base[0x36] = MStar_IntMaskSave[6];
	int_mask_base[0x37] = MStar_IntMaskSave[7];
#if defined(CONFIG_MP_PLATFORM_FIQ_IRQ_HYP)
	hypint_mask_base[0x24] = MStar_HypIntMaskSave[0];
	hypint_mask_base[0x25] = MStar_HypIntMaskSave[1];
	hypint_mask_base[0x26] = MStar_HypIntMaskSave[2];
	hypint_mask_base[0x27] = MStar_HypIntMaskSave[3];
	hypint_mask_base[0x34] = MStar_HypIntMaskSave[4];
	hypint_mask_base[0x35] = MStar_HypIntMaskSave[5];
	hypint_mask_base[0x36] = MStar_HypIntMaskSave[6];
	hypint_mask_base[0x37] = MStar_HypIntMaskSave[7];
#endif
}

void mstar_clear_int(void)
{
    volatile unsigned long *int_mask_base=(volatile unsigned long *)INT_MASK_REG_BASE;
    int_mask_base[0x4c]=0xFFFF;
    int_mask_base[0x4d]=0xFFFF;
    int_mask_base[0x4e]=0xFFFF;
    int_mask_base[0x4f]=0xFFFF;
    int_mask_base[0x5c]=0xFFFF;
    int_mask_base[0x5d]=0xFFFF;
    int_mask_base[0x5e]=0xFFFF;
    int_mask_base[0x5f]=0xFFFF;
}

unsigned long mstar_virt_to_phy(void* virtaddr)
{
	unsigned long rest = 0;

	rest = virt_to_phys(virtaddr);
	return rest;
}

void *mstar_phy_to_virt(unsigned long phyaddr)
{
	void *rest=0;
	rest = phys_to_virt(phyaddr);
	return rest;
}

void mstar_sleep_cur_cpu_flush(void)
{
	Chip_Flush_Cache_All_Single();
}

static void mstar_str_notifypmmaxcnt_off(void)
{
	sleep_clear_wakeup_save_addr_phy(mstar_virt_to_phy((void*)WAKEUP_SAVE_ADDR),(void*)WAKEUP_SAVE_ADDR);
	pass_wd[0x0A] = 0xFD;
	MDrv_MBX_NotifyPMPassword(pass_wd);
	while(1);
}

#if defined(CONFIG_MSTAR_STR_ACOFF_ON_ERR)
void mstar_str_notifypmerror_off(void)
{
	sleep_clear_wakeup_save_addr_phy(mstar_virt_to_phy((void*)WAKEUP_SAVE_ADDR),(void*)WAKEUP_SAVE_ADDR);
	pass_wd[0x0A] = 0xFE;
	MDrv_MBX_NotifyPMPassword(pass_wd);
	while(1);
}
#endif

void WriteRegWord(unsigned long addr, unsigned long val)
{
	volatile unsigned long *regaddr = (unsigned long *)addr;
	(*regaddr) = val;
}

unsigned long ReadRegWord(unsigned long addr)
{
	volatile unsigned long *regaddr = (unsigned long *)addr;
	return (*regaddr);
}

#ifdef CONFIG_MSTAR_STR_CRC
extern int get_str_crc(void);
extern void get_boot_mem_info(BOOT_MEM_INFO type, unsigned int *addr, unsigned int *len);
void MDrv_MBX_NotifyPMtoCrcCheck(int finish)
{
	if (!finish)
		STR_CRC_REG(CRC_PATTERN_REG1) = 0x9987;
	else
		STR_CRC_REG(CRC_PATTERN_REG2) = 0x8887;
}

void MDrv_MBX_write_kernel_info(void)
{
	unsigned int linux_memory_address = 0, linux_memory_length = 0;
	unsigned int linux_memory2_address = 0, linux_memory2_length = 0;
	unsigned int linux_memory3_address = 0, linux_memory3_length = 0;

	STR_CRC_REG(CRC_LX1_REG_ST) = 0x0000; STR_CRC_REG(CRC_LX1_REG_SZ) = 0x0000;
	STR_CRC_REG(CRC_LX2_REG_ST) = 0x0000; STR_CRC_REG(CRC_LX2_REG_SZ) = 0x0000;
	STR_CRC_REG(CRC_LX3_REG_ST) = 0x0000; STR_CRC_REG(CRC_LX3_REG_SZ) = 0x0000;

	get_boot_mem_info(LINUX_MEM, &linux_memory_address, &linux_memory_length);
	get_boot_mem_info(LINUX_MEM2, &linux_memory2_address, &linux_memory2_length);
	get_boot_mem_info(LINUX_MEM3, &linux_memory3_address, &linux_memory3_length);

	// setup the kernel Lx1 start address and length
	// transform to phyical address and change address unit(8kb)
	STR_CRC_REG(CRC_LX1_REG_ST) = (linux_memory_address & (0x40000000 - 1))/0x2000;
	STR_CRC_REG(CRC_LX1_REG_SZ) = linux_memory_length / 0x100000; //change unit(1M)

	// setup the kernel Lx2&Lx3 start address and length
	if (linux_memory2_length != 0 && linux_memory3_length == 0) {
		STR_CRC_REG(CRC_LX2_REG_ST) = (linux_memory2_address & (0x40000000 - 1))/0x2000;
		STR_CRC_REG(CRC_LX2_REG_SZ) = linux_memory2_length / 0x100000;
		STR_CRC_REG(CRC_LX2_REG_SZ) |= IS_MIU1; // MIU1
	} else if (linux_memory2_length == 0 && linux_memory3_length != 0) {
		STR_CRC_REG(CRC_LX2_REG_ST) = (linux_memory3_address & 0xFFFFFFF)/0x2000;
		STR_CRC_REG(CRC_LX2_REG_SZ) = linux_memory3_length / 0x100000;
		STR_CRC_REG(CRC_LX2_REG_SZ) |= IS_MIU1; // MIU1
	} else if (linux_memory2_length != 0 && linux_memory3_length != 0) {
		STR_CRC_REG(CRC_LX2_REG_ST) = (linux_memory2_address & (0x40000000 - 1))/0x2000;
		STR_CRC_REG(CRC_LX2_REG_SZ) = linux_memory2_length / 0x100000;
		STR_CRC_REG(CRC_LX3_REG_ST) = (linux_memory3_address & 0xFFFFFFF)/0x2000;
		STR_CRC_REG(CRC_LX3_REG_SZ) = linux_memory3_length / 0x100000;
		STR_CRC_REG(CRC_LX3_REG_SZ) |= IS_MIU1; // MIU1
	}

	//SerPrintf( "CRC_LX1_REG_ST = 0x%x\n",STR_CRC_REG(CRC_LX1_REG_ST));
	//SerPrintf( "CRC_LX1_REG_SZ = 0x%x\n",STR_CRC_REG(CRC_LX1_REG_SZ));
	//SerPrintf( "CRC_LX2_REG_ST = 0x%x\n",STR_CRC_REG(CRC_LX2_REG_ST));
	//SerPrintf( "CRC_LX2_REG_SZ = 0x%x\n",STR_CRC_REG(CRC_LX2_REG_SZ));
	//SerPrintf( "CRC_LX3_REG_ST = 0x%x\n",STR_CRC_REG(CRC_LX3_REG_ST));
	//SerPrintf( "CRC_LX3_REG_SZ = 0x%x\n",STR_CRC_REG(CRC_LX3_REG_SZ));
}
bool MDrv_MBX_recviceAck(void)
{
	if((STR_CRC_REG(CRC_PATTERN_REG1) == 0x1234) &&
			(STR_CRC_REG(CRC_PATTERN_REG2) == 0x5678))
		return true;

	udelay(1000); // delay 1ms
	return false;
}
#endif

void mstar_save_context(void)
{
	platform_smp_boot_secondary_reset(1);
	mstar_save_int_mask();
	save_performance_monitors((appf_u32 *)performance_monitor_save);
	//save_a12_generic_timer((appf_u32 *)a12_generic_timer_save);

	save_gic_interface((appf_u32 *)gic_interface_save,(unsigned)_gic_cpu_base_addr,1);
	save_gic_distributor_private((appf_u32 *)gic_distributor_private_save,(unsigned)_gic_dist_base_addr,1);

	save_cp15((appf_u32 *)cp15_save);// CSSELR

	save_gic_distributor_shared((appf_u32 *)gic_distributor_shared_save,(unsigned)_gic_dist_base_addr,1);
	//save_control_registers(control_data, 1);
	save_mmu(mmu_data);
	//save_a12_scu((appf_u32 *)a12_scu_save);

	sleep_save_neon_regs(&MStar_Suspend_Buffer[SLEEPSTATE_NEONREG/WORD_SIZE]);
	sleep_save_cpu_registers(MStar_Suspend_Buffer);
	write_actlr(read_actlr() & ~A9_SMP_BIT);//add
}
void mstar_restore_context(void)
{
	sleep_restore_neon_regs(&MStar_Suspend_Buffer[SLEEPSTATE_NEONREG/WORD_SIZE]);
	//restore_a12_scu((appf_u32 *)a12_scu_save);
	restore_mmu(mmu_data);
	//restore_control_registers(control_data, 1);
	restore_gic_distributor_shared((appf_u32 *)gic_distributor_shared_save,(unsigned)_gic_dist_base_addr,1);
	gic_distributor_set_enabled(TRUE, (unsigned)_gic_dist_base_addr);//add
	restore_gic_distributor_private((appf_u32 *)gic_distributor_private_save,(unsigned)_gic_dist_base_addr,1);
	restore_gic_interface((appf_u32 *)gic_interface_save,(unsigned)_gic_cpu_base_addr,1);

	restore_cp15((appf_u32 *)cp15_save);

	//restore_a12_generic_timer((appf_u32 *)a12_generic_timer_save);
	restore_performance_monitors((appf_u32 *)performance_monitor_save);

	mstar_restore_int_mask();

	sleep_clear_wakeup_save_addr_phy(mstar_virt_to_phy((void*)WAKEUP_SAVE_ADDR),(void*)WAKEUP_SAVE_ADDR);
	platform_smp_boot_secondary_init(1);
	mstar_sleep_cur_cpu_flush();
	flush_pmd_entry((void*)strsavearea[saved_pmd_entry]);
	local_flush_bp_all();
	local_flush_tlb_all();
}

#ifdef CONFIG_MP_PLATFORM_UTOPIA2K_EXPORT_SYMBOL
typedef void (*UtopiaSTREntry)(void);
static UtopiaSTREntry save_utopia2k_pfn = NULL;
static UtopiaSTREntry restore_utopia2k_pfn = NULL;

void mstar_set_utopia2k_cbf_pm(UtopiaSTREntry pfUtopiaSave, UtopiaSTREntry pfUtopiaRestore)
{
	save_utopia2k_pfn = pfUtopiaSave;
	restore_utopia2k_pfn = pfUtopiaRestore;
}
EXPORT_SYMBOL(mstar_set_utopia2k_cbf_pm);
#endif

unsigned int mstar_tee_suspend(void)
{
#if 0
	__asm__ volatile(
			".arch_extension sec\n\t"
			"ldr r0, =0x8400ff1e\n\t"
			"smc #0\n\t"
			:
			:
			);
#else
	struct arm_smccc_res_mstar input_param = {0};
	input_param.a0 = 0x8400ff1e;
	tee_fast_call_cmd(&input_param);
#endif

	return 0;
}

static void _mstar_str_enter_tee(void *pWakeup)
{
    extern uint32_t isPSCI;

    if ((TEEINFO_TYPTE==SECURITY_TEEINFO_OSTYPE_OPTEE) &&
        (isPSCI == PSCI_RET_SUCCESS))
    {
        psci_smp_ops.cpu_suspend(0,pWakeup);
        __asm__ volatile(
            "WAIT_SLEEP_TEE:\n"
            "nop\n"
            "nop\n"
            "b WAIT_SLEEP_TEE\n"
        );
    }
}

void mstar_str_power_off(void)
{
    SerPrintf("\nMStar STR waiting power off...[%d]\n", MDrv_MPM_Get_Cnt(MDRV_MPM_KEY_STR_CNT));

    if (MDrv_MPM_Check_DC())
	{
		SerPrintf("Max Cnt Ac off...\n");
		memset(strsavearea, 0, sizeof(strsavearea));

		if (is_mstar_str())
		{
		    mstar_str_notifypmmaxcnt_off();
		}
	}
	else
	{
#ifdef CONFIG_MSTAR_STR_CRC
		if(get_str_crc())
		{
			MDrv_MBX_NotifyPMtoCrcCheck(false);
			MDrv_MBX_write_kernel_info();
			MDrv_MBX_NotifyPMtoCrcCheck(true);
			while(!MDrv_MBX_recviceAck());
		}
#endif
		SerPrintf("\ndo MDrv_MBX_NotifyPMtoSetPowerOff\n");
		if (is_mstar_str())
		{
			SerPrintf("\nUser Mode STR, send password to 51...\n");
			MDrv_MBX_NotifyPMtoSetPowerOff();
		}
		else
		{
#if defined(CONFIG_MSTAR_PM)
			SerPrintf("\nKernel Mode STR, run 51...\n");
#endif
		}
	}
}

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

	if (TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_OPTEE)
		mstar_tee_suspend();

	__asm__ volatile (
		"ldr r1, =MSTAR_WAKEUP_ENTRY\n"
		"str r1, %0"
		:"=m"(pWakeup)::"r1"
	);

	platform_smp_boot_secondary_reset(1);
	mstar_save_int_mask();
	save_performance_monitors((appf_u32 *)performance_monitor_save);
	//save_a12_generic_timer((appf_u32 *)a12_generic_timer_save);

#ifdef CONFIG_MP_PLATFORM_UTOPIA2K_EXPORT_SYMBOL
	if(save_utopia2k_pfn) save_utopia2k_pfn();
#endif

	save_gic_interface((appf_u32 *)gic_interface_save,(unsigned)_gic_cpu_base_addr,1);
	save_gic_distributor_private((appf_u32 *)gic_distributor_private_save,(unsigned)_gic_dist_base_addr,1);

	save_cp15((appf_u32 *)cp15_save);// CSSELR

	save_gic_distributor_shared((appf_u32 *)gic_distributor_shared_save,(unsigned)_gic_dist_base_addr,1);
	//save_control_registers(control_data, 1);
	save_mmu(mmu_data);
	//save_a12_scu((appf_u32 *)a12_scu_save);

	sleep_save_neon_regs(&MStar_Suspend_Buffer[SLEEPSTATE_NEONREG/WORD_SIZE]);
	sleep_save_cpu_registers(MStar_Suspend_Buffer);
	sleep_set_wakeup_save_addr_phy(mstar_virt_to_phy((void*)WAKEUP_SAVE_ADDR),(void*)WAKEUP_SAVE_ADDR);

	sleep_prepare_last(mstar_virt_to_phy(pWakeup));
	write_actlr(read_actlr() & ~A9_SMP_BIT);//add

	mstar_str_power_off();

    _mstar_str_enter_tee(pWakeup);

	__asm__ volatile(
		"WAIT_SLEEP:\n"
		"nop\n"
		"nop\n"
		"b WAIT_SLEEP\n"
	);

    //////////////////////////////////////////////////////////////
    __asm__ volatile(
        "MSTAR_WAKEUP_ENTRY:\n" //"nop\nb MSTAR_WAKEUP_ENTRY\n"
        "bl ensure_environment\n"
        "bl use_tmp_stack\n"
        "mov r0, #'K'\n"
        "bl __PUTCHAR\n"
        "ldr r1, =exit_addr\n"
        "sub r0, pc,#4 \n"
        "b   sleep_wakeup_first\n"          //sleep_wakeup_first();
        "exit_addr: \n"
        "mov r0, #'L'\n"
        "bl PUTCHAR_VIRT\n"
        "ldr r0,=MStar_Suspend_Buffer\n"
        "bl sleep_restore_cpu_registers\n"  //sleep_restore_cpu_registers(MStar_Suspend_Buffer)
        :::"r0","r1","r2","r3","r4","r5","r6","r7","r8","r9","r10","r12"
    );
    SerPrintf("\nMStar STR Resuming...\n");

    MDrv_MPM_Set_Cnt(MDRV_MPM_KEY_STR_CNT, NULL);

#if defined(CONFIG_MP_PLATFORM_CPU_SETTING)
#if defined CONFIG_MSTAR_CPU_calibrating
	//Init Test Bus for CPU Clock Counter
	if (*(volatile u32 *)(0xfd200a00) == 0x3697) {
		*(volatile u32 *)(0xfd000000 + 0x101896 *2) = 0x01;
		*(volatile u32 *)(0xfd000000 + 0x101eea *2) = 0x00;
		*(volatile u32 *)(0xfd000000 + 0x101eea *2) = 0x04;
		*(volatile u32 *)(0xfd000000 + 0x101eea *2) = 0x4004;
		*(volatile u32 *)(0xfd000000 + 0x101eee *2) = 0x001f;
		*(volatile u32 *)(0xfd000000 + 0x101e62 *2) = 0;
		*(volatile u32 *)(0xfd000000 + 0x101e62 *2) = 0x01;
	}
#endif // defined CONFIG_MSTAR_CPU_calibrating
#endif /*MP_PLATFORM_CPU_SETTING*/
	sleep_restore_neon_regs(&MStar_Suspend_Buffer[SLEEPSTATE_NEONREG/WORD_SIZE]);
	//restore_a12_scu((appf_u32 *)a12_scu_save);
	restore_mmu(mmu_data);
	//restore_control_registers(control_data, 1);
	restore_gic_distributor_shared((appf_u32 *)gic_distributor_shared_save,(unsigned)_gic_dist_base_addr,1);
	gic_distributor_set_enabled(TRUE, (unsigned)_gic_dist_base_addr);//add
	restore_gic_distributor_private((appf_u32 *)gic_distributor_private_save,(unsigned)_gic_dist_base_addr,1);
	restore_gic_interface((appf_u32 *)gic_interface_save,(unsigned)_gic_cpu_base_addr,1);

	restore_cp15((appf_u32 *)cp15_save);

#ifdef CONFIG_MP_PLATFORM_UTOPIA2K_EXPORT_SYMBOL
	if(restore_utopia2k_pfn)
		restore_utopia2k_pfn();
#endif
	//restore_a12_generic_timer((appf_u32 *)a12_generic_timer_save);
	restore_performance_monitors((appf_u32 *)performance_monitor_save);

	mstar_restore_int_mask();

	sleep_clear_wakeup_save_addr_phy(mstar_virt_to_phy((void*)WAKEUP_SAVE_ADDR),(void*)WAKEUP_SAVE_ADDR);
	platform_smp_boot_secondary_init(1);
	mstar_sleep_cur_cpu_flush();
	flush_pmd_entry((void*)strsavearea[saved_pmd_entry]);
	local_flush_bp_all();
	local_flush_tlb_all();
	return 0;
}

static void mstar_pm_power_off_prepare(void)
{
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
	.enter	= mstar_pm_enter,
	.valid	= suspend_valid_only_mem,
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

