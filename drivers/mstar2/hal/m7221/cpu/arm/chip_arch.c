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

/*------------------------------------------------------------------------------
    PROJECT: chip

	FILE NAME: arch/arm/mach-chip/chip_arch.c

    DESCRIPTION:
          Power Management Driver

    HISTORY:
         <Date>     <Author>    <Modification Description>
        2008/07/18  Fred Cheng  Add IO tables for ITCM and DTCM
        2008/07/22  Evan Chang  Add SD card init

------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    Include Files
------------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/memory.h>
#include <mach/io.h>
#include <asm/mach/map.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <asm/irq.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/smp_twd.h>
#include <asm/cacheflush.h>
#include "chip_int.h"
#include <linux/version.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <include/prom.h>
#include <mstar/mstar_chip.h>
#ifdef CONFIG_WRITE_TO_ANDROID_MISC
#include <linux/reboot.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,49)
#include <linux/perf/arm_pmu.h>
#include <../mm/dma.h>  // for using dmac_map_area @ arch/arm/mm/dma.h
#else
#include <asm/pmu.h>
#endif

#ifdef CONFIG_WRITE_TO_ANDROID_MISC_NAND
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/stringify.h>
#include <linux/namei.h>
#include <linux/stat.h>
#include <linux/miscdevice.h>
#include <linux/log2.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#endif

#if defined (CONFIG_MSTAR_IIC) || defined(CONFIG_MSTAR_GPIO)
#include "../../gpio/mhal_gpio_reg.h"
#endif

#include <linux/of_platform.h>
/*------------------------------------------------------------------------------
    External Function Prototypes
-------------------------------------------------------------------------------*/
/*add for 3.10.23, this is used for matching the new machine_desc in arch.h*/
extern void chip_init_ptimer(void);
extern void chip_init_timer(void );
/* initialize chip's IRQ */
extern void chip_init_irq(void);
/* Add SD Card driver */
extern void __init chip_add_device_mmc(short mmc_id);
extern void __init chip_add_device_memstick(s16 memstick_id);
extern int Mstar_ehc_platform_init(void);
#ifdef CONFIG_WRITE_TO_ANDROID_MISC_NAND
int Write_Mtd0_init(void);
int Mtd_erase_ctrl_blocks(void);
int Write_Mtd0_recovery(__u8* DataBuf,unsigned long u32Size);
#endif

/*------------------------------------------------------------------------------
    External Global Variable
-------------------------------------------------------------------------------*/
/* chip's system timer */
extern struct sys_timer chip_timer;
extern struct sys_timer chip_ptimer;
extern struct smp_operations chip_smp_ops;
#ifdef CONFIG_MP_MSTAR_STR_BASE
extern struct platform_device Mstar_xc_str_device;
#endif
/*------------------------------------------------------------------------------
    Global Variable
-------------------------------------------------------------------------------*/
/* platform device for chip_board_init() */
#if defined(CONFIG_MP_PLATFORM_ARM_PMU)

/* resource for pmu irq */
static struct resource pmu_resources[] =
{
    [0] =
    {
		.start	= CHIP_IRQ_PMU0,
		.end	= CHIP_IRQ_PMU0,
		.flags	= IORESOURCE_IRQ,
	},

    [1] =
    {
		.start	= CHIP_IRQ_PMU1,
		.end	= CHIP_IRQ_PMU1,
		.flags	= IORESOURCE_IRQ,
	},

	[2] = {
		.start	= CHIP_IRQ_PMU2,
		.end	= CHIP_IRQ_PMU2,
		.flags	= IORESOURCE_IRQ,
	},

	[3] = {
		.start	= CHIP_IRQ_PMU3,
		.end	= CHIP_IRQ_PMU3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device chip_device_pmu =
{
    .name      = "arm-pmu",
    .id	       = -1,
    .num_resources = ARRAY_SIZE(pmu_resources),
    .resource = pmu_resources,
};
#endif

static struct platform_device Mstar_emac_device = {
    .name       = "Mstar-emac",
    .id     = 0,
};
static struct platform_device Mstar_ir_device = {
	.name		= "Mstar-ir",
	.id		= 0,
};

static struct platform_device Mstar_gmac_device = {
    .name       = "Mstar-gmac",
    .id     = 0,
};

static struct platform_device Mstar_mbx_device = {
	.name		= "Mstar-mbx",
	.id		= 0,
};

static struct platform_device Mstar_gflip_device = {
	.name		= "Mstar-gflip",
	.id		= 0,
};

static struct platform_device Mstar_alsa_device = {
	.name		= "Mstar-alsa",
	.id		= 0,
};
#ifdef CONFIG_MSTAR_IIC
static struct mstar_i2c_attr
{
    u16 bus_index;
    u16 sda_pad;
    u16 scl_pad;
    u16 retries;
    u16 def_delay;
    u16 speed_khz;
};
struct mstar_i2c_attr iic0 ={0, PAD_DDCR_DA, PAD_DDCR_CK, 5, 100, 100};
struct mstar_i2c_attr iic1 ={1, PAD_TGPIO1, PAD_TGPIO0, 5, 100, 100};

static struct platform_device Mstar_iic0_device = {
	.name		= "Mstar-iic",
	.id		= 0,
        .dev = {
            .platform_data = &iic0,
        },
};
static struct platform_device Mstar_iic1_device = {
	.name		= "Mstar-iic",
	.id		= 1,
        .dev = {
            .platform_data = &iic1,
        },
};

#endif
#ifdef CONFIG_MSTAR_GPIO
#define GPIO_MAX_RESERVED_PIN_NUMBER 20 /* reserved pins that should not be exposed to sysfs. E.g. swi2c pads */

int reserved_pads[GPIO_MAX_RESERVED_PIN_NUMBER]={PAD_DDCR_DA,PAD_DDCR_CK,PAD_TGPIO1,PAD_TGPIO0,0};

static struct platform_device Mstar_gpio_device = {
	.name		= "Mstar-gpio",
	.id		= 0,
	.dev = {
            .platform_data = &reserved_pads,
        },
};
#endif
#if defined(CONFIG_MSTAR_RTC) || defined(CONFIG_MP_Android_DUMMY_MSTAR_RTC)
static struct platform_device Mstar_rtc_device = {
	.name		= "rtc-mstar",
	.id		= 0,
};
#endif

#ifdef CONFIG_MSTAR_XC_HDR_SUPPORT
static struct platform_device Mstar_scaler_device = {
    .name       = "Mstar-scaler",
    .id         = 0,
};
#endif

/* platform device array for chip_board_init() */
static struct platform_device *chip_devices[] __initdata =
{
#if defined(CONFIG_MP_PLATFORM_ARM_PMU)
    &chip_device_pmu,
#endif
    &Mstar_emac_device,
    &Mstar_ir_device,
    &Mstar_gmac_device,
    &Mstar_mbx_device,
#ifdef CONFIG_MP_MSTAR_STR_BASE
    &Mstar_xc_str_device,
#endif
    &Mstar_gflip_device,
    &Mstar_alsa_device,
#ifdef CONFIG_MSTAR_IIC
    &Mstar_iic0_device,
    &Mstar_iic1_device,
#endif
#ifdef CONFIG_MSTAR_GPIO
    &Mstar_gpio_device,
#endif
#if defined(CONFIG_MSTAR_RTC) || defined(CONFIG_MP_Android_DUMMY_MSTAR_RTC)
    &Mstar_rtc_device,
#endif
#ifdef CONFIG_MSTAR_XC_HDR_SUPPORT
    &Mstar_scaler_device,
#endif
};

/* IO tables for Registers, ITCM and DTCM */
static struct map_desc chip_io_desc[] __initdata =
{
    /* Define Registers' physcial and virtual addresses */
    {
	    .virtual        = IO_VIRT,
	    .pfn            = __phys_to_pfn(IO_PHYS),
	    .length         = IO_SIZE,
	    .type           = MT_DEVICE
    },

    /* Define periphral physcial and virtual addresses */
    {
	    .virtual        = PERI_VIRT,
	    .pfn            = __phys_to_pfn(PERI_PHYS),
	    .length         = PERI_SIZE,
	    .type           = MT_DEVICE
    },

    {
	    .virtual	    = L2_CACHE_VIRT,
	    .pfn	    = __phys_to_pfn(L2_CACHE_PHYS),
	    .length	    = L2_CACHE_SIZE,
	    .type	    = MT_DEVICE,
    },



    #if 0
    /* Define ITCM' physcial and virtual addresses */
    {
		.virtual	= chip_ITCM_VIRT,
		.pfn		= __phys_to_pfn(chip_ITCM_PHYS),
		.length		= chip_ITCM_SIZE,
		.type		= MT_DEVICE
    },
	/* Define DTCM' physcial and virtual addresses */
	{
		.virtual	= chip_DTCM_VIRT,
		.pfn		= __phys_to_pfn(chip_DTCM_PHYS),
		.length		= chip_DTCM_SIZE,
		.type		= MT_DEVICE
	},
    /* Define MEM51' physcial and virtual addresses */
	{
		.virtual	= chip_MEM51_VIRT,
		.pfn		= __phys_to_pfn(chip_MEM51_PHY),
		.length		= chip_MEM51_SIZE,
		.type		= MT_MEMORY_NONCACHED
	}
	#endif
};

/*------------------------------------------------------------------------------
    Local Function
-------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
    Function: chip_map_io

    Description:
        init chip IO tables
    Input: (The arguments were used by caller to input data.)
        None.
    Output: (The arguments were used by caller to receive data.)
        None.
    Return:
        None.
    Remark:
        None.
-------------------------------------------------------------------------------*/
void __init chip_map_io(void)
{
printk("@@ [%s](%d): HERE\n",__func__,__LINE__);
    iotable_init(chip_io_desc, ARRAY_SIZE(chip_io_desc) );
}

void __init create_mapping(struct map_desc *md, bool force_pages);
void __init chip_early_map_io(void)
{
	create_mapping(chip_io_desc, false); /* create riu mapping */
}

void __init serial_init(void)
{

#ifdef CONFIG_SERIAL_8250
    struct uart_port s, u, k;

    memset(&s, 0, sizeof(s));
    s.type = PORT_16550;
    s.iobase = 0xFD201300; //virtual RIU_BASE = 0xFD000000
    s.irq = E_IRQ_UART0;

#ifdef CONFIG_MSTAR_ARM_BD_FPGA
    s.uartclk = 12000000; //FPGA
#elif defined(CONFIG_MSTAR_ARM_BD_GENERIC)
    s.uartclk = 123000000; //real chip
#endif
    s.iotype = 0;
    s.regshift = 0;
    s.fifosize = 16 ; // use the 8 byte depth FIFO well
    s.line = 0;

    if (early_serial_setup(&s) != 0) {
        printk(KERN_ERR "Serial(0) setup failed!\n");
    }
	memset(&u, 0, sizeof(u));
	u.type = PORT_16550;
	u.iobase = 0xFD220C00;
	u.irq = E_IRQEXPL_UART1;
	u.uartclk = 108000000;
	u.iotype = 0;
	u.regshift = 0;
	u.fifosize = 16 ; // use the 8 byte depth FIFO well
	u.line = 1;

	if (early_serial_setup(&u) != 0) {
		printk(KERN_ERR "Serial piu_uart1 setup failed!\n");
	}

	memset(&k, 0, sizeof(k));
	k.type = PORT_16550;
	k.iobase = 0xFD220D00;
	k.irq = E_IRQEXPH_UART2MCU;
	k.uartclk = 123000000;
	k.iotype = 0;
	k.regshift = 0;
	k.fifosize = 16 ; // use the 8 byte depth FIFO well
	k.line = 2;

	if (early_serial_setup(&k) != 0) {
		printk(KERN_ERR "Serial fuart setup failed!\n");
	}
#endif
}

/*------------------------------------------------------------------------------
    Function: chip_board_init

    Description:
        chip board init function
    Input: (The arguments were used by caller to input data.)
        None.
    Output: (The arguments were used by caller to receive data.)
        None.
    Return:
        None.
    Remark:
        None.
-------------------------------------------------------------------------------*/
#ifdef CONFIG_WRITE_TO_ANDROID_MISC

#define REBOOT_RECOVERY_STR  "boot-recovery"
#define REBOOT_UPGRADEBOOTLOADER_STR  "update-radio/hboot"

static int recovery_sys_notify_reboot(struct notifier_block *this,
            unsigned long code, void *x)
{
    if(x && strlen((char*)x))
    {
        mm_segment_t old_fs;
        loff_t off;

        old_fs = get_fs();
        set_fs(KERNEL_DS);
        //printk(KERN_ERR "reboot cmd %s\n", (char*)x);
#if defined(CONFIG_WRITE_TO_ANDROID_MISC_EMMC) /* EMMC */
		{
			struct file *fd = filp_open("/dev/block/platform/mstar_mci.0/by-name/misc", O_RDWR, 0);
			size_t t;
			if(fd)
			{
				off = 0;
				t = vfs_write(fd, REBOOT_RECOVERY_STR, strlen(REBOOT_RECOVERY_STR)+1, &off);
				printk(KERN_ERR "write to \"/dev/block/platform/mstar_mci.0/by-name/misc\" %ld bytes\n", (long int)t);
				vfs_fsync(fd, 0);
				filp_close(fd, NULL);
			}
			else
				printk(KERN_ERR "open \"/dev/block/platform/mstar_mci.0/by-name/misc\" failed\n");
		}
#elif defined(CONFIG_WRITE_TO_ANDROID_MISC_NAND) /* NAND */
# if 1
        if(!Write_Mtd0_init())
        {
            Mtd_erase_ctrl_blocks();
            Write_Mtd0_recovery(REBOOT_RECOVERY_STR,2048);
            //Read_Mtd0_recovery();
        }
        else
        {
            printk(  "Write_Mtd0_init failed\n");
        }
# else
        struct file *fd = filp_open("/dev/mtd/mtd0", O_RDWR, 0);
        char *buff = kzalloc(2048, GFP_USER);
        strncpy(buff, REBOOT_RECOVERY_STR, strlen(REBOOT_RECOVERY_STR)+1);
        if(fd)
        {
            off = 0;
            size_t t = vfs_write(fd, buff, 2048, &off);
            printk(KERN_ERR "write to /dev/mtd/mtd0 %ld bytes\n", t);
            vfs_fsync(fd, 0);
            filp_close(fd, NULL);
        }
        else
            printk(KERN_ERR "open /dev/mtd/mtd0 failed\n");
        kfree(buff);
# endif
#endif
        set_fs(old_fs);
    }
    return NOTIFY_DONE;
}

static struct notifier_block recovery_sys_notifier = {
        .notifier_call    = recovery_sys_notify_reboot,
        .next   = NULL,
        .priority = 0,
};
#endif

//for flush store buffer
static unsigned int * uncached_buf = NULL;
static dma_addr_t   buf_phy_addr;
static int          dummy_read;

//-----------------------------------------------------------------------------
#ifdef CONFIG_MP_PLATFORM_PIPE_FLUSH_DOUBLE_CHECK
BOOT_MEM_INFO list[3] = { LINUX_MEM2, LINUX_MEM3, LINUX_MEM4 };
volatile phys_addr_t alloLXRAM0_PA_BASE, alloLXRAM1_PA_BASE, alloLXRAM2_PA_BASE;
volatile phys_addr_t alloLXRAM0_SIZE, alloLXRAM1_SIZE, alloLXRAM2_SIZE;
volatile phys_addr_t * alloRAM0_VA_BASE, * alloRAM1_VA_BASE, * alloRAM2_VA_BASE;
bool gbInit = false;

#ifdef CONFIG_CMA
extern struct CMA_BootArgs_Config cma_config[MAX_CMA_AREAS];
#endif

#endif
//-----------------------------------------------------------------------------

static void __init chip_board_init(void)
{
#ifdef CONFIG_MP_PLATFORM_PIPE_FLUSH_DOUBLE_CHECK
	//-----------------------------------------------------------------------------
	int i, j, cma_end;
	phys_addr_t tmp_pa, tmp2_pa, tmp_size;

	//! LINUX_MEM
	get_boot_mem_info(LINUX_MEM,  &alloLXRAM0_PA_BASE, &alloLXRAM0_SIZE);
	alloLXRAM1_PA_BASE = alloLXRAM0_PA_BASE;
	alloLXRAM2_PA_BASE = alloLXRAM0_PA_BASE;

	for(i = 0; i < 3; i++)
	{
		get_boot_mem_info(list[i], &tmp_pa, &tmp_size);    	//! check the range of LINUX_MEM2

		if (tmp_pa >= ARM_MIU1_BUS_BASE && tmp_pa < ARM_MIU2_BUS_BASE)
		{
			tmp2_pa = tmp_pa;
		#ifdef CONFIG_CMA
			 for (j = 0; j < MAX_CMA_AREAS; j++)
			 {
			    cma_end = cma_config[j].start + cma_config[j].size;
			    if (cma_config[j].start <= tmp2_pa && cma_end > tmp2_pa)
			    {
				    if (cma_end + 0x1000 <= tmp_pa + tmp_size)
				    {
					    tmp2_pa = cma_end;
						j = 0;
					}
				}
			 }
 		#endif
		     tmp_pa = tmp2_pa;
		     if(alloLXRAM1_PA_BASE == alloLXRAM0_PA_BASE)
			 {
			 	alloLXRAM1_PA_BASE = tmp_pa;
				alloLXRAM1_SIZE    = tmp_size;
		     }
	    }
		#ifdef  ARM_MIU2_BUS_BASE
		else if (tmp_pa >= ARM_MIU2_BUS_BASE)
		{
			tmp2_pa = tmp_pa;
		#ifdef CONFIG_CMA
			for (j = 0; j < MAX_CMA_AREAS; j++)
			{
			    cma_end = cma_config[j].start + cma_config[j].size;
			    if (cma_config[j].start <= tmp2_pa && cma_end > tmp2_pa)
			    {
				    if (cma_end + 0x1000 <= tmp_pa + tmp_size)
				    {
					    tmp2_pa = cma_end;
						j = 0;
					}
				}
			}
		#endif
			tmp_pa = tmp2_pa;
		    if(alloLXRAM2_PA_BASE == alloLXRAM0_PA_BASE)
			{
				alloLXRAM2_PA_BASE = tmp_pa;
				alloLXRAM2_SIZE    = tmp_size;
		    }
		}
		#endif
	}

	alloRAM0_VA_BASE = (phys_addr_t *)ioremap(alloLXRAM0_PA_BASE, 0x1000); //m <== miu0
	if (alloRAM0_VA_BASE == NULL)
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] remap miu0 address fails. alloLXRAM0_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloLXRAM0_PA_BASE);
	else
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] alloRAM0_VA_BASE: 0x%lx, alloLXRAM0_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloRAM0_VA_BASE, alloLXRAM0_PA_BASE);

	alloRAM1_VA_BASE = (phys_addr_t *)ioremap(alloLXRAM1_PA_BASE, 0x1000);
	if (alloLXRAM1_PA_BASE == alloLXRAM0_PA_BASE)
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] Cannot find miu1 address. alloLXRAM1_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloLXRAM1_PA_BASE);
	else if (alloRAM1_VA_BASE == NULL)
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] remap miu1 address fails. alloLXRAM1_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloLXRAM1_PA_BASE);
	else
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] alloRAM1_VA_BASE: 0x%lx, alloLXRAM1_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloRAM1_VA_BASE, alloLXRAM1_PA_BASE);

#ifdef ARM_MIU2_BUS_BASE
	alloRAM2_VA_BASE = (phys_addr_t *)ioremap(alloLXRAM2_PA_BASE, 0x1000);
	if (alloLXRAM2_PA_BASE == alloLXRAM0_PA_BASE)
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] Cannot find miu2 address. alloLXRAM2_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloLXRAM2_PA_BASE);
	else if (alloRAM2_VA_BASE == NULL)
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] remap miu2 address fails. alloLXRAM2_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloLXRAM2_PA_BASE);
	else
		printk("\033[0;32;31m Function = %s Line = %d, [SAM-CC] alloRAM2_VA_BASE: 0x%lx, alloLXRAM2_PA_BASE: 0x%lx\033[m\n", __func__, __LINE__, alloRAM2_VA_BASE, alloLXRAM2_PA_BASE);
#endif

	gbInit = true;

#ifdef ARM_MIU2_BUS_BASE
	BUG_ON(!alloRAM0_VA_BASE || !alloRAM1_VA_BASE || !alloRAM2_VA_BASE);
#else
	BUG_ON(!alloRAM0_VA_BASE || !alloRAM1_VA_BASE);
#endif
	//-----------------------------------------------------------------------------
#endif

#ifdef CONFIG_CACHE_L2X0
	unsigned int val = 0;
#endif

    //allocate non-cached memory for flush store buffer
    uncached_buf=arm_dma_alloc(NULL,4 ,&buf_phy_addr, GFP_KERNEL,NULL);

#ifdef CONFIG_OF
	of_platform_populate(NULL, of_default_bus_match_table,
					NULL, NULL);
#else
	platform_add_devices(chip_devices, ARRAY_SIZE(chip_devices));

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	Mstar_ehc_platform_init();
#endif
#endif

#if defined(CONFIG_MEMSTICK_CHIP_MS)
	chip_add_device_memstick(0);
#endif

#ifdef CONFIG_CACHE_L2X0
	{
		void __iomem * l2x0_base
			= (void __iomem *)(L2_CACHE_ADDRESS(L2_CACHE_PHYS));

# if L2_LINEFILL
		val = L2_CACHE_read(L2_CACHE_PHYS + PREFETCH_CTL_REG);
		L2_CACHE_write(val | DOUBLE_LINEFILL_ENABLE | LINEFILL_WRAP_DISABLE
				, L2_CACHE_PHYS + PREFETCH_CTL_REG);
# endif

# if L2_PREFETCH
		val = L2_CACHE_read(L2_CACHE_PHYS + PREFETCH_CTL_REG);
		L2_CACHE_write(val | I_PREFETCH_ENABLE | D_PREFETCH_ENABLE
				| PREFETCH_OFFSET, L2_CACHE_PHYS + PREFETCH_CTL_REG );
# endif

		/* latency settings are given by hardware designer */
		writel(0x121, l2x0_base + L2X0_TAG_LATENCY_CTRL);
		writel(0x121, l2x0_base + L2X0_DATA_LATENCY_CTRL);

# ifdef CONFIG_MP_ACP_L2
		l2x0_init(l2x0_base, 0x00000000, 0xfe0fffff);
# else
		l2x0_init(l2x0_base, 0x00400000, 0xfe0fffff);
# endif
	}
#endif

	*(volatile unsigned short *)(0xfd200200) = 0x1 ;
	*(volatile unsigned short *)(0xfd200250) = 0xFFFF ;
	*(volatile unsigned short *)(0xfd200254) = 0xFFFF ;

#ifdef CONFIG_WRITE_TO_ANDROID_MISC
	register_reboot_notifier(&recovery_sys_notifier);
#endif
}

/*************************************
*		Mstar chip flush function
*************************************/
#define _BIT(x)                      (1<<(x))
static DEFINE_SPINLOCK(l2prefetch_lock);

inline void _chip_flush_miu_pipe(void)
{
/*
	unsigned long   dwLockFlag = 0;
	unsigned short dwReadData = 0;
	unsigned long ulReadMIU0Data, ulReadMIU1Data;
#ifdef ARM_MIU2_BUS_BASE
	unsigned long ulReadMIU2Data;
#endif

	spin_lock_irqsave(&l2prefetch_lock, dwLockFlag);

    //toggle the flush miu pipe fire bit, 0x10188a[0
    *(volatile unsigned short *)(0xfd203114) &= ~(0x0001);	// reset to 0 first, to ensure check bits is reset
    *(volatile unsigned short *)(0xfd203114) |= 0x0001;

	// check bits are 0x1018a1[4]~ 0x1018a1[6]
	do
	{
		dwReadData = *(volatile unsigned short *)(0xfd203140);
        dwReadData &= (_BIT(12));  //Check Status of Flush Pipe Finish

    } while(dwReadData != (_BIT(12)));	// for U01, we need to check 3 bits (MIU0/1/2)
#ifdef CONFIG_MP_PLATFORM_PIPE_FLUSH_DOUBLE_CHECK
	//! dummy read
	if (gbInit == true)
	{
		ulReadMIU0Data = (unsigned long)(* alloRAM0_VA_BASE);
		ulReadMIU1Data = (unsigned long)(* alloRAM1_VA_BASE);
#ifdef ARM_MIU2_BUS_BASE
		ulReadMIU2Data = (unsigned long)(* alloRAM2_VA_BASE);
#endif
	}
#endif
	spin_unlock_irqrestore(&l2prefetch_lock, dwLockFlag);
*/
}
void Chip_Flush_Miu_Pipe(void)
{
    _chip_flush_miu_pipe();
}

#ifndef CONFIG_OUTER_CACHE
struct outer_cache_fns outer_cache;
#endif

void Chip_Flush_Cache_Range(unsigned long u32Addr, unsigned long u32Size)
{
	if(  u32Addr == (unsigned long) NULL )
        {
                printk("u32Addr is invalid\n");
                return;
        }
	//Clean L1 & Inv L1
	dmac_flush_range((const void *)u32Addr,(const void *)(u32Addr + u32Size));

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{
		if(!virt_addr_valid(u32Addr) || !virt_addr_valid(u32Addr+ u32Size - 1))
			//Clean&Inv L2 by Way
			outer_cache.flush_all();
		else
			//Clean&Inv L2 by Range
			outer_cache.flush_range(__pa(u32Addr) ,__pa(u32Addr)+ u32Size);
	}
#endif

#ifndef CONFIG_OUTER_CACHE //flush miu pipe for L2 disabled case
	_chip_flush_miu_pipe();
#endif

}

void Chip_Clean_Cache_Range(unsigned long u32Addr, unsigned long u32Size)
{
        if(  u32Addr == (unsigned long) NULL )
        {
                printk("u32Addr is invalid\n");
                return;
        }
	//Clean L1
	dmac_map_area((const void *)u32Addr,(size_t)u32Size,1);

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{
		if(!virt_addr_valid(u32Addr) || !virt_addr_valid(u32Addr+ u32Size - 1))
			//Clean L2 by Way
			outer_cache.clean_all();
		else
			//Clean L2 by Range
			outer_cache.clean_range( __pa(u32Addr),__pa(u32Addr) + u32Size);
	}
#endif

#ifndef CONFIG_OUTER_CACHE //flush miu pipe for L2 disabled case
	_chip_flush_miu_pipe();
#endif
}


void Chip_Inv_Cache_Range(unsigned long u32Addr, unsigned long u32Size)
{
        if(  u32Addr == (unsigned long) NULL )
        {
                printk("u32Addr is invalid\n");
                return;
        }
#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{

		if(!virt_addr_valid(u32Addr) || !virt_addr_valid(u32Addr+ u32Size - 1))
			printk(KERN_DEBUG "Input VA can't be converted to PA\n");
		else
			//Inv L2 by range
			outer_cache.inv_range(__pa(u32Addr) , __pa(u32Addr) + u32Size);
	}
#endif
	//Inv L1
	dmac_map_area((const void *)u32Addr,(size_t)u32Size,2);
}

void Chip_Flush_Cache_Range_VA_PA(unsigned long u32VAddr,unsigned long u32PAddr,unsigned long u32Size)
{
        if(  u32VAddr == (unsigned long) NULL )
        {
                printk("u32VAddr is invalid\n");
                return;
        }
	//Clean & Invalid L1
	dmac_flush_range((const void *)u32VAddr,(const void *)(u32VAddr + u32Size));

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{
		//Clean&Inv L2 by range
		outer_cache.flush_range(u32PAddr,u32PAddr + u32Size);
	}
#endif

#ifndef CONFIG_OUTER_CACHE
	_chip_flush_miu_pipe();
#endif
}

void Chip_Clean_Cache_Range_VA_PA(unsigned long u32VAddr,unsigned long u32PAddr,unsigned long u32Size)
{
        if(  u32VAddr == (unsigned long) NULL )
        {
                printk("u32VAddr is invalid\n");
                return;
        }
	//Clean L1
	dmac_map_area((const void *)u32VAddr,(size_t)u32Size,1);

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{
		//Clean L2 by Way
		outer_cache.clean_range(u32PAddr,u32PAddr + u32Size);
	}
#endif

#ifndef CONFIG_OUTER_CACHE
	_chip_flush_miu_pipe();
#endif
}

void Chip_Inv_Cache_Range_VA_PA(unsigned long u32VAddr,unsigned long u32PAddr,unsigned long u32Size)
{
        if(  u32VAddr == (unsigned long) NULL )
        {
                printk("u32VAddr is invalid\n");
                return;
        }

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{
		//Inv L2 by range
		outer_cache.inv_range( u32PAddr ,u32PAddr + u32Size );
	}
#endif
	//Inv L1
	dmac_map_area((const void *)u32VAddr,(size_t)u32Size,2);
}

//add for fixing compile warning
void CPU_Cache_Flush_All(void *p)
{
	__cpuc_flush_kern_all();
}

void Chip_Flush_Cache_All(void)
{
	unsigned long flags;

	preempt_disable();
	local_irq_save(flags);

	//Clean & Inv All L1
	__cpuc_flush_kern_all();

	local_irq_restore(flags);

	smp_call_function(CPU_Cache_Flush_All, NULL, 1);

	local_irq_save(flags);

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()) //check if L2 is enabled
	{
		//Clean&Inv L2 by Way
		outer_cache.flush_all();
	}
#endif
	//Clean L1  & Inv L1
	//dmac_flush_range(u32Addr,u32Addr + u32Size );

#ifndef CONFIG_OUTER_CACHE
	_chip_flush_miu_pipe();
#endif

	local_irq_restore(flags);
	preempt_enable();
}

void Chip_Flush_Cache_All_Single(void)
{
	unsigned long flags;

	local_irq_save(flags);

	//Clean & Inv All L1
	__cpuc_flush_kern_all();

#ifdef CONFIG_OUTER_CACHE
	if (outer_cache.is_enable()&&outer_cache.flush_all) //check if L2 is enabled
	{
		//Clean&Inv L2 by Way
		outer_cache.flush_all();
	}
#endif
	//Clean L1  & Inv L1
	//dmac_flush_range(u32Addr,u32Addr + u32Size );

#ifndef CONFIG_OUTER_CACHE
	_chip_flush_miu_pipe();
#endif

	local_irq_restore(flags);
}

//need to be modified
void Chip_L2_cache_wback_inv( unsigned long addr, unsigned long size)
{
	outer_cache.flush_all();
}

//need to be modified
void Chip_L2_cache_wback( unsigned long addr, unsigned long size)
{
	//Flush L2 by Way, change to by Addr later
	outer_cache.clean_all();
}

//need to be modified
void Chip_L2_cache_inv( unsigned long addr, unsigned long size)
{
	//Inv L2 by Way, change to by Addr later
	outer_cache.inv_all();
}

void Chip_Flush_Memory_Range(unsigned long pAddress , unsigned long  size)
{
    #ifdef CONFIG_OUTER_CACHE
	if(outer_cache.sync)
		outer_cache.sync();
	else
    #endif
    {
        //flush store buffer
        *uncached_buf= *(volatile unsigned short *)(0xfd000000 + (0x101274 << 1));
        dmb();
        dummy_read = *uncached_buf;
		_chip_flush_miu_pipe();
}
}

void Chip_Flush_Memory(void)
{
    #ifdef CONFIG_OUTER_CACHE
	if(outer_cache.sync)
		outer_cache.sync();
	else
	#endif
    {
        //flush store buffer
        *uncached_buf= *(volatile unsigned short *)(0xfd000000 + (0x101274 << 1));
        dmb();
        dummy_read = *uncached_buf;
		_chip_flush_miu_pipe();
}
}

void Chip_Read_Memory_Range(unsigned long pAddress , unsigned long  size)
{
    #ifdef CONFIG_OUTER_CACHE
	if(outer_cache.sync)
		outer_cache.sync();
	else
	#endif
    {
        //flush store buffer
        *uncached_buf= *(volatile unsigned short *)(0xfd000000 + (0x101274 << 1));
        dmb();
        dummy_read = *uncached_buf;
		_chip_flush_miu_pipe();
}
}

void Chip_Read_Memory(void)
{
    #ifdef CONFIG_OUTER_CACHE
	if(outer_cache.sync)
		outer_cache.sync();
	else
	#endif
    {
        //flush store buffer
        *uncached_buf= *(volatile unsigned short *)(0xfd000000 + (0x101274 << 1));
        dmb();
        dummy_read = *uncached_buf;
		_chip_flush_miu_pipe();
}
}

void Chip_Query_L2_Config(void)
{
}

unsigned int Chip_Query_Rev(void)
{
	unsigned int    dwRevisionId = 0;
	dwRevisionId = *((volatile unsigned int*)(0xFD003D9C));
	dwRevisionId >>= 8;

	return dwRevisionId;
}
#ifdef CONFIG_WRITE_TO_ANDROID_MISC_NAND
static struct mtd_info * ctrl_mtd;
static __u8 u8_Partfound;

//extern struct mtd_info * __init open_mtd_device(const char *mtd_dev);

extern struct mtd_info *get_mtd_device_nm(const char *name);
extern struct mtd_info *get_mtd_device(struct mtd_info *mtd, int num);
/**
 * open_mtd_by_chdev - open an MTD device by its character device node path.
 * @mtd_dev: MTD character device node path
 *
 * This helper function opens an MTD device by its character node device path.
 * Returns MTD device description object in case of success and a negative
 * error code in case of failure.
 */
static struct mtd_info * __init open_mtd_by_chdev(const char *mtd_dev)
{
	int err, major, minor, mode;
	struct path path;

	/* Probably this is an MTD character device node path */
	err = kern_path(mtd_dev, LOOKUP_FOLLOW, &path);
	if (err)
		return ERR_PTR(err);

	/* MTD device number is defined by the major / minor numbers */
	major = imajor(path.dentry->d_inode);
	minor = iminor(path.dentry->d_inode);
	mode = path.dentry->d_inode->i_mode;
	path_put(&path);
	if (major != MTD_CHAR_MAJOR || !S_ISCHR(mode))
		return ERR_PTR(-EINVAL);

	if (minor & 1)
		/*
		 * Just do not think the "/dev/mtdrX" devices support is need,
		 * so do not support them to avoid doing extra work.
		 */
		return ERR_PTR(-EINVAL);

	return get_mtd_device(NULL, minor / 2);
}



static struct mtd_info * __init open_mtd_device(const char *mtd_dev)
{
	struct mtd_info *mtd;
	int mtd_num;
	char *endp;

	mtd_num = simple_strtoul(mtd_dev, &endp, 0);
	if (*endp != '\0' || mtd_dev == endp) {
		/*
		 * This does not look like an ASCII integer, probably this is
		 * MTD device name.
		 */
		mtd = get_mtd_device_nm(mtd_dev);
		if (IS_ERR(mtd) && PTR_ERR(mtd) == -ENODEV)
			/* Probably this is an MTD character device node path */
			mtd = open_mtd_by_chdev(mtd_dev);
	} else
		mtd = get_mtd_device(NULL, mtd_num);

	return mtd;
}


int Write_Mtd0_init(void)
{
	int err = 0;
	u8_Partfound = 0;
	ctrl_mtd = open_mtd_device("misc");
	if(IS_ERR(ctrl_mtd))
	{
		printk("Can't find \"misc\" partition\n");
		err = PTR_ERR(ctrl_mtd);
		return err;
	}
	u8_Partfound = 1;

	return err;
}



#define ENINIT_MAX_SQNUM			0x7FFFFFFF
//control error code
#define ERR_CTRL_RECORD_FOUND		(0x600|1)
#define ERR_CTRL_RECORD_INVALID		(0x600|2)
#define ERR_CTRL_RECORD_FAIL		(0x600|3)
#define ERR_CTRL_RECORD_NOFOUND		(0x600|4)
#define ERR_CTRL_RECORD_NOAVLBLK 	(0x600|5)
#define ERR_CTRL_RECORD_WRITEFAIL 	(0x600|6)
#define ERR_TBL_RECORD_POWERCUT		(0x600|7)
#define ERR_TBL_RECORD_NOAVLBLK 	(0x600|8)


int Mtd_erase_ctrl_blocks(void)
{
	int blk_page_count, err;
	__u32 u32_i, cb_count;
	struct erase_info ei;
	blk_page_count = (ctrl_mtd->erasesize/ctrl_mtd->writesize);
	cb_count = ctrl_mtd->size >> ctrl_mtd->writesize_shift;

	for(u32_i = 0; u32_i < cb_count; u32_i += blk_page_count)
	{
		err = ctrl_mtd->block_isbad(ctrl_mtd, u32_i << ctrl_mtd->writesize_shift);
		if(err == 1)
		{
			printk("Skip bad block %X", u32_i);
			continue;
		}
		else if (err < 0)
		{
			return ERR_CTRL_RECORD_FAIL;
		}

		memset(&ei, 0, sizeof(struct erase_info));
		ei.mtd = ctrl_mtd;
		ei.addr = (u32_i / blk_page_count) << ctrl_mtd->erasesize_shift;
		ei.len = ctrl_mtd->erasesize;
		ei.callback = NULL;

		err = ctrl_mtd->erase(ctrl_mtd, &ei);
		if(err == -EIO)
		{
			err = ctrl_mtd->block_markbad(ctrl_mtd,
					(u32_i / blk_page_count)<< ctrl_mtd->erasesize_shift);
			if(err < 0)
				return err;
		}
		else if(err < 0 && err != -EIO)
		{
			return err;
		}

	}
	return 0;
}


int Write_Mtd0_recovery(__u8* DataBuf,unsigned long u32Size)
{
    __u8* pu8_DataBuf;
    __u32 err = 0;
	size_t written;
    pu8_DataBuf = kzalloc(ctrl_mtd->writesize, GFP_KERNEL);
	memcpy(pu8_DataBuf,DataBuf,u32Size);
	if(!pu8_DataBuf)
		return err;

    err = ctrl_mtd->write(ctrl_mtd, 0, ctrl_mtd->writesize, &written, pu8_DataBuf);

    kfree(pu8_DataBuf);
	return err;
}

int Read_Mtd0_recovery(void)
{
    __u8* pu8_DataBuf;
    __u32 err = 0;
    size_t read;
	pu8_DataBuf = kzalloc(ctrl_mtd->writesize, GFP_KERNEL);
	memset(pu8_DataBuf, 0xFF, ctrl_mtd->writesize);
    err = ctrl_mtd->read(ctrl_mtd,0, 10, &read, pu8_DataBuf);

	printk("%d %d %d \n",pu8_DataBuf[0],pu8_DataBuf[1],pu8_DataBuf[2]);
	return err;
}

#endif

#if 0 //modify later
unsigned int Chip_Query_CLK(void)
{
    return (12*reg_readw(0x1F22184c)); // 1 = 12(Hz)

}
#endif

/*
#ifndef CONFIG_CPU_FREQ
unsigned int cpufreq_quick_get_max(unsigned int cpu)
{
	return 1000000;
}
EXPORT_SYMBOL(cpufreq_quick_get_max);
#endif
*/

EXPORT_SYMBOL(Chip_Flush_Cache_Range);
EXPORT_SYMBOL(Chip_Flush_Cache_All);
EXPORT_SYMBOL(Chip_Flush_Cache_All_Single);
EXPORT_SYMBOL(Chip_Clean_Cache_Range);
EXPORT_SYMBOL(Chip_Inv_Cache_Range);
EXPORT_SYMBOL(Chip_L2_cache_wback_inv);
EXPORT_SYMBOL(Chip_L2_cache_wback);
EXPORT_SYMBOL(Chip_L2_cache_inv);
EXPORT_SYMBOL(Chip_Flush_Memory);
EXPORT_SYMBOL(Chip_Flush_Memory_Range);
EXPORT_SYMBOL(Chip_Read_Memory);
EXPORT_SYMBOL(Chip_Read_Memory_Range);
EXPORT_SYMBOL(Chip_Flush_Cache_Range_VA_PA);
EXPORT_SYMBOL(Chip_Clean_Cache_Range_VA_PA);
EXPORT_SYMBOL(Chip_Inv_Cache_Range_VA_PA);
EXPORT_SYMBOL(Chip_Flush_Miu_Pipe);

/**
 ** please refer to include/asm-arm/mach/arch.h for further details
 **
 ** #define MACHINE_START(_type,_name)				\
 ** static const struct machine_desc __mach_desc_##_type	\
 **  __used							\
 **   __attribute__((__section__(".arch.info.init"))) = {	\
 **   	.nr		= MACH_TYPE_##_type,			\
 **   	.name		= _name,
 **
 **
 ** #define MACHINE_END			\
 ** };
 **
 **
 **/
 static const char * const mstar_dt_board_compat[] = {
	"arm,foundation-arm",
	"arm,armv8",
	"arm,flounder64",
	"arm,flounder64_lte",
	NULL
};

#ifdef CONFIG_OF
DT_MACHINE_START(MSTAR, CONFIG_MSTAR_CHIP_NAME)
#else
MACHINE_START(MSTAR, CONFIG_MSTAR_CHIP_NAME)
#endif
    .atag_offset   = 0x00000100,
    .map_io = chip_map_io,
    .init_irq = chip_init_irq,
    .init_machine = chip_board_init,
    .smp = smp_ops(chip_smp_ops),
#ifdef CONFIG_GENERIC_CLOCKEVENTS
    .init_time = chip_init_timer,
#else
    .init_time = chip_init_ptimer,
#endif
    .dt_compat	= mstar_dt_board_compat,
MACHINE_END
