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

/*
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 1999,2000 MIPS Technologies, Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * PROM library functions for acquiring/using memory descriptors given to
 * us from the YAMON.
 */
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/pfn.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/binfmts.h>
#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/sections.h>

#include <asm/mips-boards/prom.h>

#if defined(CONFIG_MSTAR_TITANIA3) // @FIXME:
#include "../titania3/board/Board.h"
#elif defined(CONFIG_MSTAR_TITANIA10) // @FIXME:
#include "../titania10/board/Board.h"
#else
#include "Board.h" // #include "../titania2/board/Board.h"
#endif
/*#define DEBUG*/
#include <linux/version.h>

#include <linux/math64.h>
//__aeabi_uldivmod
unsigned long long __aeabi_uldivmod(unsigned long long n, unsigned long long d)
{
    return div64_u64(n, d);
}

EXPORT_SYMBOL(__aeabi_uldivmod);


//__aeabi_ldivmod
long long __aeabi_ldivmod(long long n, long long d)
{
    return div64_s64(n, d);
}

EXPORT_SYMBOL(__aeabi_ldivmod);

enum yamon_memtypes {
    yamon_dontuse,
    yamon_prom,
    yamon_free,
};
//static struct prom_pmemblock mdesc[PROM_MAX_PMEMBLOCKS];

#ifdef DEBUG
static char *mtypes[3] = {
    "Dont use memory",
    "YAMON PROM memory",
    "Free memmory",
};
#endif

/* determined physical memory size, not overridden by command line args  */
unsigned long physical_memsize = 0L;
#if 0
static struct prom_pmemblock * __init prom_getmdesc(void)
{
    char *memsize_str;
    unsigned int memsize;
    char cmdline[CL_SIZE], *ptr;

    /* otherwise look in the environment */
    memsize_str = prom_getenv("memsize");
    if (!memsize_str) {
        printk(KERN_WARNING
               "memsize not set in boot prom, set to default (32Mb)\n");
        physical_memsize = 0x02000000;
    } else {
#ifdef DEBUG
        pr_debug("prom_memsize = %s\n", memsize_str);
#endif
        physical_memsize = simple_strtol(memsize_str, NULL, 0);
    }

#ifdef CONFIG_CPU_BIG_ENDIAN
    /* SOC-it swaps, or perhaps doesn't swap, when DMA'ing the last
       word of physical memory */
    physical_memsize -= PAGE_SIZE;
#endif

    /* Check the command line for a memsize directive that overrides
       the physical/default amount */
    strcpy(cmdline, arcs_cmdline);
    ptr = strstr(cmdline, "memsize=");
    if (ptr && (ptr != cmdline) && (*(ptr - 1) != ' '))
        ptr = strstr(ptr, " memsize=");

    if (ptr)
        memsize = memparse(ptr + 8, &ptr);
    else
        memsize = physical_memsize;

    memset(mdesc, 0, sizeof(mdesc));

    mdesc[0].type = yamon_dontuse;
    mdesc[0].base = 0x00000000;
    mdesc[0].size = 0x00001000;

    mdesc[1].type = yamon_prom;
    mdesc[1].base = 0x00001000;
    mdesc[1].size = 0x000ef000;

#ifdef CONFIG_MIPS_MALTA
    /*
     * The area 0x000f0000-0x000fffff is allocated for BIOS memory by the
     * south bridge and PCI access always forwarded to the ISA Bus and
     * BIOSCS# is always generated.
     * This mean that this area can't be used as DMA memory for PCI
     * devices.
     */
    mdesc[2].type = yamon_dontuse;
    mdesc[2].base = 0x000f0000;
    mdesc[2].size = 0x00010000;
#else
    mdesc[2].type = yamon_prom;
    mdesc[2].base = 0x000f0000;
    mdesc[2].size = 0x00010000;
#endif

    mdesc[3].type = yamon_dontuse;
    mdesc[3].base = 0x00100000;
    mdesc[3].size = CPHYSADDR(PFN_ALIGN((unsigned long)&_end)) - mdesc[3].base;

    mdesc[4].type = yamon_free;
    mdesc[4].base = CPHYSADDR(PFN_ALIGN(&_end));
    mdesc[4].size = memsize - mdesc[4].base;

    return &mdesc[0];
}

static int __init prom_memtype_classify(unsigned int type)
{
    switch (type) {
    case yamon_free:
        return BOOT_MEM_RAM;
    case yamon_prom:
        return BOOT_MEM_ROM_DATA;
    default:
        return BOOT_MEM_RESERVED;
    }
}
#endif

//DRAMlen is the length of DRAM mapping area,not the actual length of DRAM
static unsigned long LXmem2Addr=0,LXmem2Size=0,LXmem=0, EMACmem=0, DRAMlen=0, BBAddr=0;
static unsigned long G3Dmem0Addr=0, G3Dmem0Len=0, G3Dmem1Addr=0, G3Dmem1Len=0, G3DCmdQAddr=0, G3DCmdQLen=0;
#if defined(CONFIG_LX3_SUPPORT)
static unsigned long LXmem3Addr=0,LXmem3Size=0;
#endif
static unsigned long GMACmemAddr=0,GMACmemLen=0;
static char coredump_path[CORENAME_MAX_SIZE]={0};

static int __init LX_MEM_setup(char *str)
{
    //printk("LX_MEM= %s\n", str);
    if( str != NULL )
    {
        LXmem = simple_strtol(str, NULL, 16);
    }
    else
    {
        printk("\nLX_MEM not set\n");
    }
    return 0;
}

static int __init EMAC_MEM_setup(char *str)
{
   //printk("EMAC_MEM= %s\n", str);
    if( str != NULL )
    {
        EMACmem = simple_strtol(str, NULL, 16);
    }
    else
    {
        printk("\nEMAC_MEM not set\n");
    }
    return 0;
}

static int __init DRAM_LEN_setup(char *str)
{
    //printk("DRAM_LEN= %s\n", str);
    if( str != NULL )
    {
        DRAMlen = simple_strtol(str, NULL, 16);
    }
    else
    {
        printk("\nDRAM_LEN not set\n");
    }
    return 0;
}

static int __init LX_MEM2_setup(char *str)
{
    //printk("LX_MEM2= %s\n", str);
    if( str != NULL )
    {
        sscanf(str,"%lx,%lx",&LXmem2Addr,&LXmem2Size);
    }
    else
    {
        printk("\nLX_MEM2 not set\n");
    }
    return 0;
}

#if defined(CONFIG_LX3_SUPPORT)
static int __init LX_MEM3_setup(char *str)
{
    printk("LX_MEM3= %s", str);
    if( str != NULL )
    {
        sscanf(str,"%lx,%lx",&LXmem3Addr,&LXmem3Size);
    }
    else
    {
        printk("\nLX_MEM3 not set\n");
    }
    return 0;
}
#endif

static int __init G3D_MEM_setup(char *str)
{
    if( str != NULL )
    {
        sscanf(str,"%lx,%lx,%lx,%lx,%lx,%lx",
        &G3Dmem0Addr,&G3Dmem0Len,&G3Dmem1Addr,&G3Dmem1Len,
        &G3DCmdQAddr,&G3DCmdQLen);
    }
    else
    {
        printk("\nG3D_MEM not set\n");
    }
    return 0;
}


static int __init GMAC_MEM_setup(char *str)
{
    printk("GMAC_MEM= %s\n", str);
    if( str != NULL )
    {
        sscanf(str,"%lx,%lx",&GMACmemAddr,&GMACmemLen);
    }
    else
    {
        printk("\nGMAC_MEM not set\n");
    }
    return 0;
}
static int __init BBAddr_setup(char *str)
{
    //printk("LX_MEM2= %s\n", str);
    if( str != NULL )
    {
        sscanf(str,"%lx",&BBAddr);
    }

    return 0;
}

early_param("LX_MEM", LX_MEM_setup);
early_param("EMAC_MEM", EMAC_MEM_setup);
early_param("GMAC_MEM", GMAC_MEM_setup);
early_param("DRAM_LEN", DRAM_LEN_setup);
early_param("LX_MEM2", LX_MEM2_setup);
#if defined(CONFIG_LX3_SUPPORT)
early_param("LX_MEM3", LX_MEM3_setup);
#endif
early_param("BB_ADDR", BBAddr_setup);
early_param("G3D_MEM", G3D_MEM_setup);

static char bUseDefMMAP=0;
static void check_boot_mem_info(void)
{
    if( LXmem==0 || DRAMlen==0 )
    {
        bUseDefMMAP = 1;
    }
}
#if defined(CONFIG_MSTAR_OFFSET_FOR_SBOOT)
#define SBOOT_LINUX_MEM_START 0x00400000    //4M
#define SBOOT_LINUX_MEM_LEN   0x01400000    //20M
#define SBOOT_EMAC_MEM_LEN    0x100000      //1M
#define SBOOT_GMAC_MEM_LEN    0x100000      //1M
void get_boot_mem_info_sboot(BOOT_MEM_INFO type, unsigned int *addr, unsigned int *len)
{
    switch (type)
    {
    case LINUX_MEM:
        *addr = SBOOT_LINUX_MEM_START;
        *len = SBOOT_LINUX_MEM_LEN;
        break;

    case EMAC_MEM:
        *addr = SBOOT_LINUX_MEM_START+SBOOT_LINUX_MEM_LEN;
        *len = SBOOT_EMAC_MEM_LEN;
        break;

    case GMAC_MEM:
        *addr = SBOOT_LINUX_MEM_START + SBOOT_LINUX_MEM_LEN + SBOOT_EMAC_MEM_LEN;
        *len = SBOOT_GMAC_MEM_LEN;
        break;

    case MPOOL_MEM:
        *addr = SBOOT_LINUX_MEM_START + SBOOT_LINUX_MEM_LEN + SBOOT_EMAC_MEM_LEN;
        *len = 256*1024*1024;
        break;

    case LINUX_MEM2:
        *addr = 0;
        *len = 0;
        break;

    default:
        *addr = 0;
        *len = 0;
        break;
    }
}
#endif//CONFIG_MSTAR_OFFSET_FOR_SBOOT


void get_boot_mem_info(BOOT_MEM_INFO type, unsigned int *addr, unsigned int *len)
{
#if defined(CONFIG_MSTAR_OFFSET_FOR_SBOOT)
    get_boot_mem_info_sboot(type, addr, len);
    printk("!!!!!!!!!!SBOOT memory type=%x addr=%x, len=%x!!!!!!!!!!\n",type, *addr, *len);
    return;
#endif//CONFIG_MSTAR_OFFSET_FOR_SBOOT

    if (bUseDefMMAP == 0)
    {
        switch (type)
        {
        case LINUX_MEM:
            *addr = LINUX_MEM_BASE_ADR;
            *len = LXmem;
            break;

        case EMAC_MEM:
            *addr = LINUX_MEM_BASE_ADR + LXmem;
            *len = EMACmem;
            break;

        case MPOOL_MEM:
            *addr = LINUX_MEM_BASE_ADR + LXmem + EMACmem;
            *len = DRAMlen - *addr;
            break;

        case LINUX_MEM2:
            if (LXmem2Addr!=0 && LXmem2Size!=0)
            {
                *addr = LXmem2Addr;
                *len = LXmem2Size;
            }
            else
            {
                *addr = 0;
                *len = 0;
            }
            break;
        case G3D_MEM0:
            *addr = G3Dmem0Addr;
            *len = G3Dmem0Len;
            break;
        case G3D_MEM1:
            *addr = G3Dmem1Addr;
            *len = G3Dmem1Len;
            break;
        case G3D_CMDQ:
            *addr = G3DCmdQAddr;
            *len = G3DCmdQLen;
            break;
        case DRAM:
            *addr = 0;
            *len = DRAMlen;
            break;
        case BB:
            *addr = BBAddr;
            *len = 0;
            break;
        case GMAC_MEM:
            *addr = GMACmemAddr;
            *len = GMACmemLen;
            break;
#if defined(CONFIG_LX3_SUPPORT)
        case LINUX_MEM3:
            if (LXmem3Addr!=0 && LXmem3Size!=0)
            {
                *addr = LXmem3Addr;
                *len = LXmem3Size;
            }
            else
            {
                *addr = 0;
                *len = 0;
            }
            break;
#endif
        default:
            *addr = 0;
            *len = 0;
            break;
        }
    }
    else
    {
        switch (type)
        {
        case LINUX_MEM:
            *addr = LINUX_MEM_BASE_ADR;
            *len = LINUX_MEM_LEN;
            break;

        case EMAC_MEM:
            *addr = EMAC_MEM_ADR;
            *len = EMAC_MEM_LEN;
            break;

        case MPOOL_MEM:
            *addr = MPOOL_ADR;
            *len = MPOOL_LEN;
            break;

        case LINUX_MEM2:
            if (LXmem2Addr!=0 && LXmem2Size!=0)
            {
                *addr = LXmem2Addr;
                *len  = LXmem2Size;
            }
            else
            {
                #ifdef LINUX_MEM2_BASE_ADR    // reserved miu1 memory for linux
                *addr = LINUX_MEM2_BASE_ADR;
                *len = LINUX_MEM2_LEN;
                printk("LINUX_MEM2 addr %08x, len %08x\n", *addr, *len);
                #else
                *addr = 0;
                *len = 0;
                #endif
            }
            break;
#if defined(CONFIG_LX3_SUPPORT)
        case LINUX_MEM3:
            if (LXmem3Addr!=0 && LXmem3Size!=0)
            {
                *addr = LXmem3Addr;
                *len = LXmem3Size;
            }
            else
            {
                *addr = 0;
                *len = 0;
            }
            break;
#endif
        case G3D_MEM0:
            *addr = G3Dmem0Addr;
            *len = G3Dmem0Len;
            break;
        case G3D_MEM1:
            *addr = G3Dmem1Addr;
            *len = G3Dmem1Len;
            break;
        case G3D_CMDQ:
            *addr = G3DCmdQAddr;
            *len = G3DCmdQLen;
            break;
        case DRAM:
            *addr = 0;
            *len = DRAMlen;
            break;
        case BB:
            *addr = BBAddr;
            *len = 0;
            break;
        case GMAC_MEM:
            *addr = GMACmemAddr;
            *len = GMACmemLen;
            break;
        default:
            *addr = 0;
            *len = 0;
            break;
        }
    }
}
EXPORT_SYMBOL(get_boot_mem_info);

void __init prom_meminit(void)
{

    unsigned int linux_memory_address = 0, linux_memory_length = 0;
    unsigned int linux_memory2_address = 0, linux_memory2_length = 0;
#if defined(CONFIG_LX3_SUPPORT)
    unsigned int linux_memory3_address = 0, linux_memory3_length = 0;
#endif
    check_boot_mem_info();
    get_boot_mem_info(LINUX_MEM, &linux_memory_address, &linux_memory_length);
    get_boot_mem_info(LINUX_MEM2, &linux_memory2_address, &linux_memory2_length);
#if defined(CONFIG_LX3_SUPPORT)
    get_boot_mem_info(LINUX_MEM3, &linux_memory3_address, &linux_memory3_length);
#endif
    if (linux_memory_length != 0)
        add_memory_region(linux_memory_address, linux_memory_length, BOOT_MEM_RAM);
    if (linux_memory2_length != 0)
        add_memory_region(linux_memory2_address, linux_memory2_length, BOOT_MEM_RAM);
#if defined(CONFIG_LX3_SUPPORT)
    if (linux_memory3_length != 0)
        add_memory_region(linux_memory3_address, linux_memory3_length, BOOT_MEM_RAM);
#endif
    printk("\n");
    printk("LX_MEM  = 0x%X, 0x%X\n", linux_memory_address,linux_memory_length);
    printk("LX_MEM2 = 0x%X, 0x%X\n",linux_memory2_address, linux_memory2_length);
#if defined(CONFIG_LX3_SUPPORT)
    printk("LX_MEM3 = 0x%X, 0x%X\n",linux_memory3_address, linux_memory3_length);
#endif
    printk("CPHYSADDR(PFN_ALIGN(&_end))= 0x%X\n", CPHYSADDR(PFN_ALIGN(&_end)));
    printk("EMAC_LEN= 0x%lX\n", EMACmem);
    printk("DRAM_LEN= 0x%lX\n", DRAMlen);

    #if defined(CONFIG_MSTAR_TITANIA8) 	|| \
		defined(CONFIG_MSTAR_JANUS2) 	|| \
		defined(CONFIG_MSTAR_MIPS)		|| \
		defined(CONFIG_MSTAR_TITANIA11) || \
		defined(CONFIG_MSTAR_TITANIA12) || \
		defined(CONFIG_MSTAR_KRONUS) 	|| \
		defined(CONFIG_MSTAR_KAISERIN) 	|| \
        defined(CONFIG_MSTAR_AMBER5) 	|| \
		defined(CONFIG_MSTAR_KENYA) || \
		defined(CONFIG_MSTAR_KERES)
        if ( DRAMlen > 0x20000000 )
            printk("ENV VARIABLE DRAM_LEN SETTING ERROR!!! LARGER THAN MAXIMUM MAPPING SIZE!!!\n");
    #endif
}

inline unsigned long get_BBAddr(void)
{
    return BBAddr;
}

void __init prom_free_prom_memory(void)
{
    unsigned long addr;
    int i;

    for (i = 0; i < boot_mem_map.nr_map; i++) {
        if (boot_mem_map.map[i].type != BOOT_MEM_ROM_DATA)
            continue;

        addr = boot_mem_map.map[i].addr;
        free_init_pages("prom memory",
                addr, addr + boot_mem_map.map[i].size);
    }
}

char* get_coredump_path(void)
{
    return coredump_path;
}
EXPORT_SYMBOL(get_coredump_path);

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,20)
#if  defined(CONFIG_MP_CHECKPT_BOOT)
#define piu_timer1_cap_low    0xbf006090
#define piu_timer1_cap_high   0xbf006094
int Mstar_Timer1_GetMs(void)
{
     int timer_value = 0;
	     timer_value = *(volatile unsigned short *)(piu_timer1_cap_low);
	     timer_value += ((*(volatile unsigned short *)(piu_timer1_cap_high)) << 16);
	     timer_value = timer_value / 12000;
		 return timer_value;
}
#endif // MP_CHECKPT_BOOT

#endif // LINUX_VERSION_CODE > KERNEL_VERSION(3,0,20)

#ifdef CONFIG_MP_PLATFORM_VERIFY_LX_MEM_ALIGN
void mstar_lx_mem_alignment_check(void)
{
}
#endif //#ifdef CONFIG_MP_PLATFORM_VERIFY_LX_MEM_ALIGN

