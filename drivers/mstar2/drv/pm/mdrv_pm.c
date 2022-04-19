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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    mdrv_mpool.c
/// @brief  Memory Pool Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>  //added
#include <linux/timer.h> //added
#include <linux/device.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/binfmts.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>
#include <linux/backlight.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "power.h"

#include "mdrv_mstypes.h"
#include "mdrv_mpm.h"
#include "mdrv_pm.h"
#include "mhal_pm.h"
#include "mdrv_system.h"
#include "mdrv_gpio.h"

//--------------------------------------------------------------------------------------------------
//  Forward declaration
//--------------------------------------------------------------------------------------------------
/* MMAP lay out. */
#define PM_OFFSET_DEFAULT       (0x0000)
#define PM_OFFSET_BT            (0x0070)
#define PM_OFFSET_USB           (0x0073)
#define PM_OFFSET_EWBS          (0x0075)
#define PM_OFFSET_BTW           (0x0078)
#define PM_OFFSET_VAD           (0x007A)
#define PM_OFFSET_POWER_DOWN    (0x00A0)
#define PM_OFFSET_SAR0          (0x00C0)
#define PM_OFFSET_SAR1          (0x00E0)
#define PM_OFFSET_LED           (0x0140)
#define PM_OFFSET_IR_VER        (0x0150)
#define PM_OFFSET_IR_NORMAL     (0x0160) //data_size + data(4* irkey_number)
#define PM_OFFSET_IR_EXTEND     (0x0200) //data_size + data(7* irkey_number)

/* MMAP Align. */
#define MMAP_DATA_ALIGN         (0x1000UL)
#define MMAP_DATA_SIZE          (0x1000UL)
#define MMAP_DRAM_ALIGN         (0x10000UL)
#define MMAP_DRAM_SIZE          (0x10000UL)

#define PM_SUPPORT_SAR_KEYS_MAX     5

/* Debug. */
#define DBG_CMD_SIZE            (15)
#define DBG_CMD_CAE             (0xF1)
#define DBG_CMD_PW              (0xF2)
#define DBG_CMD_STR             (0xF3)
#define DBG_CMD_DVI013          (PM_WAKEUPSRC_DVI)
#define DBG_CMD_DVI2            (PM_WAKEUPSRC_DVI2)
#define DBG_CMD_LAN             (PM_WAKEUPSRC_WOL)
#define DBG_CMD_VOC             (PM_WAKEUPSRC_VOICE)
#define DBG_CMD_USB             (PM_WAKEUPSRC_USB)
#define DBG_CMD_VAD             (PM_WAKEUPSRC_VAD)
#define DBG_CMD_CEC             (PM_WAKEUPSRC_CEC)
#define DBG_CMD_BTW             (0xF4)
#define DBG_CMD_WIFI            (PM_WAKEUPSRC_GPIO_WOWLAN)
#define DBG_CMD_BT              (PM_WAKEUPSRC_GPIO_WOBT)
#define DBG_CMD_EWBS            (PM_WAKEUPSRC_GPIO_WOEWBS)

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_FROM_PART_TO_LOCAL,
    E_FROM_LOCAL_TO_DRAM,
} MDRV_COPY_TYPE;

typedef struct
{
    void *  pvLocal;
    void *  pvDram;
    size_t  tSize;
    bool    bMagic;
} MDRV_BUFF_INFO;

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Golbal variable
//-------------------------------------------------------------------------------------------------
/* PM paramter data. */
static PM_WakeCfg_t         gstCfgDef = {0};
static PM_PowerDownCfg_t    gstCfgPowerDown = {0};
static PM_IrInfoCfg_t       gstCfgIr = {0};
static SAR_RegCfg           gstCfgSar0 = {0};
static SAR_RegCfg           gstCfgSar1 = {0};
static u8                   gu8CfgLed[10] = {0};
static PM_WoBT_WakeCfg      gstCfgBt = {0};
static u8                   gu8CfgBtw = PM_SOURCE_DISABLE;
static PM_WoEWBS_WakeCfg    gstCfgEwbs = {0};
static u8                   gu8CfgUsb = PM_SOURCE_DISABLE;
static u8                   gu8CfgVad = PM_SOURCE_DISABLE;

/* PM paramter buffer. */
static phys_addr_t *        gPM_DataVa = NULL;
static unsigned long long   gPM_DataAddr = 0;
static unsigned long long   gPM_DataSize = MMAP_DATA_SIZE;
/* PM firmware buffer. */
static char                 gPM_Path[CORENAME_MAX_SIZE] = {0};
static phys_addr_t *        gPM_DramTemp = NULL;
static phys_addr_t *        gPM_DramVa = NULL;
static unsigned long long   gPM_DramAddr = MMAP_DRAM_ALIGN;
static unsigned long long   gPM_DramSize = MMAP_DRAM_SIZE;

/* RTPM control. */
static unsigned int         gRTPM_Enable = 0;
/* RTPM firmware buffer. */
static char                 gRTPM_Path[CORENAME_MAX_SIZE] = {0};
static phys_addr_t *        gRTPM_DramTemp = NULL;
static phys_addr_t *        gRTPM_DramVa = NULL;
static unsigned long long   gRTPM_DramAddr = 0;
static unsigned long long   gRTPM_DramSize = MMAP_DRAM_SIZE;

/* Panic firmare buffer. */
static uint8_t              gPanic[] = {
#ifdef CONFIG_MTK_PSTORE
                                            #include "panic.dat"
#endif
                                        };

/* Other: Remove...? */
static u8                   gu8PmIr[32] = {0};
static u8                   gu8PmLed = PM_SOURCE_DISABLE;
static char                 gSlot_Suffix[3] = {0};

#ifdef CONFIG_AMAZON_WOL
static atomic_t PM_WOL_EN = ATOMIC_INIT(0);
#endif

/* Debug. */
static bool                 gbDebug = FALSE;

//-------------------------------------------------------------------------------------------------
//  Local function
//-------------------------------------------------------------------------------------------------
static unsigned long long _MDrv_PM_Ba2Pa(unsigned long long ba)
{
    unsigned long long pa = 0;

    if ((ba >= ARM_MIU0_BUS_BASE) && (ba <= ARM_MIU1_BUS_BASE))
        pa = ba - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
    else if ((ba > ARM_MIU1_BUS_BASE) && (ba <= ARM_MIU2_BUS_BASE))
        pa = ba - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;
    else
        MDRV_PM_ERR("ba=0x%llX, pa=0x%llX.\n", ba, pa);

    return pa;
}

static unsigned long long _MDrv_PM_Pa2Ba(unsigned long long pa)
{
    unsigned long long ba = 0;

    if ((pa >= ARM_MIU0_BASE_ADDR) && (pa <= ARM_MIU1_BASE_ADDR))
        ba = pa + ARM_MIU0_BUS_BASE - ARM_MIU0_BASE_ADDR;
    else if ((pa > ARM_MIU1_BASE_ADDR) && (pa <= ARM_MIU2_BASE_ADDR))
        ba = pa + ARM_MIU1_BUS_BASE - ARM_MIU1_BASE_ADDR;
    else
        MDRV_PM_ERR("pa=0x%llX, ba=0x%llX.\n", pa, ba);

    return ba;
}

static void _MDrv_RTPM_Mapping(void)
{
    /* Check parameter. */
    if ((gRTPM_DramAddr % MMAP_DRAM_ALIGN) || (gRTPM_DramSize % MMAP_DATA_SIZE))
    {
        MDRV_PM_ERR("Paramter error: Addr=0x%tX Size=0x%tX\n",
                    (size_t)gRTPM_DramAddr, (size_t)gRTPM_DramSize);
        WARN_ON(1);
    }

    /* Mapping fw buffer. */
    if (gRTPM_DramVa == NULL)
    {
        gRTPM_DramVa = ioremap_wc(gRTPM_DramAddr, gRTPM_DramSize);
        MDRV_PM_ERR("[DRAM] 0x%tX = ioremap_wc(0x%tX, 0x%tX).\n",
                    (size_t)gRTPM_DramVa,
                    (size_t)gRTPM_DramAddr, (size_t)gRTPM_DramSize);
    }
}

static void _MDrv_RTPM_Unmapping(void)
{
    if (gRTPM_DramVa != NULL)
    {
        iounmap(gRTPM_DramVa);
        gRTPM_DramVa = NULL;
    }
}

static bool _MDrv_PM_Check_Path(char *output, char *input)
{
    struct file *fp = NULL;

    if ((strlen(input) == 0) || (!output))
        return FALSE;

    fp = filp_open(input, O_RDONLY, 0);
    if (IS_ERR(fp))
        return FALSE;
    filp_close(fp, NULL);
    strncpy(output, input, CORENAME_MAX_SIZE);
    return TRUE;
}

static PM_Result _MDrv_PM_Get_Path(char *path)
{
    if ((_MDrv_PM_Check_Path(path, gPM_Path)) ||    /* Default from bootarg. */
        (_MDrv_PM_Check_Path(path, "/tvconfig/config/PM.bin")) ||
        (_MDrv_PM_Check_Path(path, "/config/PM.bin")) ||
        (_MDrv_PM_Check_Path(path, "/mnt/vendor/tvservice/glibc/bin/PM.bin")) ||
        (_MDrv_PM_Check_Path(path, "/mnt/vendor/tvconfig/config/PM.bin")))
        return E_PM_OK;

    MDRV_PM_ERR("Can't get PM.bin path.\n");
    return E_PM_FAIL;
}

static PM_Result _MDrv_RTPM_Get_Path(char *path)
{
    int ret = E_PM_OK;
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;
    int blk = 1;
    char buff[CORENAME_MAX_SIZE] = {0};

    /* Already init. */
    if (strlen(path) != 0)
    {
        fp = filp_open(path, O_RDONLY, 0);
        if (!IS_ERR(fp))
        {
            filp_close(fp, NULL);
            return E_PM_OK;
        }
    }

    /* Find /config/RT_PM.bin. */
    fp = filp_open("/config/RT_PM.bin", O_RDONLY, 0);
    if (!IS_ERR(fp))
    {
        strncpy(path, "/config/RT_PM.bin", CORENAME_MAX_SIZE);
        filp_close(fp, NULL);
        return E_PM_OK;
    }

    /* Find /dev/block/platform/mstar_mci.0/by-name/RTPM exist. */
    MDRV_PM_ERR("Ray Open slot %s.\n", gSlot_Suffix);
    snprintf(buff, CORENAME_MAX_SIZE, "/dev/block/platform/mstar_mci.0/by-name/RTPM%s", gSlot_Suffix);
    fp = filp_open(buff, O_RDONLY, 0);
    if ((!IS_ERR(fp)) || (PTR_ERR(fp) == -EACCES))
    {
        MDRV_PM_ERR("Try non-A path.\n");
        snprintf(buff, CORENAME_MAX_SIZE, "/dev/block/platform/mstar_mci.0/by-name/RTPM%s", gSlot_Suffix);
        fp = filp_open(buff, O_RDONLY, 0);
        if ((!IS_ERR(fp)) || (PTR_ERR(fp) == -EACCES))
        {
            strncpy(path, buff, CORENAME_MAX_SIZE);
            if (!IS_ERR(fp))
                filp_close(fp, NULL);
            return E_PM_OK;
	    }
    }

    /* Store fs. */
    fs = get_fs();
    set_fs(KERNEL_DS);

    /* Find /dev/mmcblk0p#. */
    do
    {
        memset(buff, '\0', CORENAME_MAX_SIZE);
        snprintf(buff, CORENAME_MAX_SIZE, "/sys/block/mmcblk0/mmcblk0p%d/uevent", blk);
        fp = filp_open(buff, O_RDONLY, 0);
        if (IS_ERR(fp))
        {
            MDRV_PM_ERR("RTPM path not found.\n");
            ret = E_PM_FAIL;
            goto _MDrv_RTPM_Get_Path_End;
        }
        memset(buff, '\0', CORENAME_MAX_SIZE);
        vfs_read(fp, buff, CORENAME_MAX_SIZE, &pos);
        filp_close(fp, NULL);

        /* Check found. */
        if (strnstr(buff, "PARTNAME=RTPM", CORENAME_MAX_SIZE))
        {
            memset(buff, '\0', CORENAME_MAX_SIZE);
            snprintf(buff, CORENAME_MAX_SIZE, "/dev/mmcblk0p%d", blk);
            fp = filp_open(buff, O_RDONLY, 0);
            if (!IS_ERR(fp))
            {
                snprintf(path, CORENAME_MAX_SIZE, "/dev/mmcblk0p%d", blk);
                filp_close(fp, NULL);
            }
            else
            {
                snprintf(path, CORENAME_MAX_SIZE, "/dev/mmcblk%d", blk);
            }
            break;
        }
        else
        {
            pos = 0;
            blk++;
        }
    } while (1);

_MDrv_RTPM_Get_Path_End:
    /* Restore fs. */
    set_fs(fs);

    return ret;
}

static PM_Result _MDrv_PM_Copy_Bin(MDRV_COPY_TYPE eType, int size, void *src, void *dst)
{
    PM_Result eRet = E_PM_OK;
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;

    switch (eType)
    {
        case E_FROM_PART_TO_LOCAL:
            if ((strlen((char *)src) == 0) || (dst == NULL))
            {
                MDRV_PM_ERR("Data error: src=%s, dst=0x%tX.\n", (char *)src, (size_t)dst);
                return E_PM_FAIL;
            }
            fp = filp_open((char *)src, O_RDONLY, 0);
            if (IS_ERR(fp))
            {
                MDRV_PM_ERR("filp_open(%s) fail = %td.\n", (char *)src, (size_t)PTR_ERR(fp));
                BUG_ON(1);
                return E_PM_FAIL;
            }
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, (char *)dst, size, &pos);
            set_fs(fs);
            filp_close(fp, NULL);
            break;

        case E_FROM_LOCAL_TO_DRAM:
            if ((src == NULL) || (dst == NULL))
            {
                MDRV_PM_ERR("Data error: src=%tX, dst=0x%tX.\n", (size_t)src, (size_t)dst);
                return E_PM_FAIL;
            }
            memcpy(dst, src, size);
            break;

        default:
            eRet = E_PM_FAIL;
            break;
    }

    return eRet;
}

static PM_Result _MDrv_PM_Store_Fw(void)
{
    if (gPM_DramTemp == NULL)
    {
        if ((gPM_DramTemp = vzalloc(gPM_DramSize)) == NULL)
        {
            MDRV_PM_ERR("Buffer vzalloc(0x%tX) fail.\n", (size_t)gPM_DramSize);
            return E_PM_FAIL;
        }

        if ((_MDrv_PM_Get_Path(gPM_Path) != E_PM_OK) ||
            (_MDrv_PM_Copy_Bin(E_FROM_PART_TO_LOCAL, (int)gPM_DramSize,
                               (void *)gPM_Path, (void *)gPM_DramTemp) != E_PM_OK))
        {
            return E_PM_FAIL;
        }
    }
    return E_PM_OK;
}

static PM_Result _MDrv_RTPM_Store_Fw(void)
{
    if (gRTPM_DramTemp == NULL)
    {
        if ((gRTPM_DramTemp = vzalloc(gRTPM_DramSize)) == NULL)
        {
            MDRV_PM_ERR("Buffer vzalloc(0x%tX) fail.\n", (size_t)gRTPM_DramSize);
            return E_PM_FAIL;
        }

        if ((_MDrv_RTPM_Get_Path(gRTPM_Path) != E_PM_OK) ||
            (_MDrv_PM_Copy_Bin(E_FROM_PART_TO_LOCAL, (int)gRTPM_DramSize,
                               (void *)gRTPM_Path, (void *)gRTPM_DramTemp) != E_PM_OK))
        {
            return E_PM_FAIL;
        }
    }
    return E_PM_OK;
}

static void _MDrv_PM_Freed_Fw(void)
{
    if (gPM_DramTemp != NULL)
    {
        vfree(gPM_DramTemp);
        gPM_DramTemp = NULL;
    }
}

static void _MDrv_RTPM_Freed_Fw(void)
{
    if (gRTPM_DramTemp != NULL)
    {
        vfree(gRTPM_DramTemp);
        gRTPM_DramTemp = NULL;
    }
}

static void _MDrv_PM_Get_Buff(u16 u16Key, MDRV_BUFF_INFO *pstInfo)
{
    if (pstInfo == NULL)
    {
        MDRV_PM_ERR("pstInfo=NULL.\n");
        return;
    }

    switch (u16Key)
    {
        case PM_KEY_DEFAULT:
            pstInfo->pvLocal = &gstCfgDef;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_DEFAULT + 0x00);
            pstInfo->tSize = sizeof(PM_WakeCfg_t);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_POWER_DOWN:
            pstInfo->pvLocal = &gstCfgPowerDown;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_POWER_DOWN + 0x00);
            pstInfo->tSize = sizeof(PM_PowerDownCfg_t);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_IR:
            pstInfo->pvLocal = &gstCfgIr;
            pstInfo->pvDram = NULL;
            pstInfo->tSize = sizeof(PM_IrInfoCfg_t);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_IR_VERSION:
            pstInfo->pvLocal = &gstCfgIr.u8IrVersion;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_IR_VER + 0x00);
            pstInfo->tSize = sizeof(gstCfgIr.u8IrVersion);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_IR_NORMAL:
            pstInfo->pvLocal = &gstCfgIr.u8NormalKeyNumber;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_IR_NORMAL + 0x00);
            pstInfo->tSize = (IR_POWER_KEY_NORMAL_SIZE_MAX+1);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_IR_EXT:
            pstInfo->pvLocal = &gstCfgIr.u8ExtendKeyNumber;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_IR_EXTEND + 0x00);
            pstInfo->tSize = (IR_POWER_KEY_EXTEND_SIZE_MAX+1);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_SAR0:
            pstInfo->pvLocal = &gstCfgSar0;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_SAR0 + 0x00);
            pstInfo->tSize = sizeof(SAR_RegCfg);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_SAR1:
            pstInfo->pvLocal = &gstCfgSar1;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_SAR1 + 0x00);
            pstInfo->tSize = sizeof(SAR_RegCfg);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_LED:
            pstInfo->pvLocal = &gu8CfgLed;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_LED + 0x00);
            pstInfo->tSize = sizeof(gu8CfgLed);
            pstInfo->bMagic = FALSE;
            break;

        case PM_KEY_BT:
            pstInfo->pvLocal = &gstCfgBt;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_BT + 0x01);
            pstInfo->tSize = sizeof(PM_WoBT_WakeCfg);
            pstInfo->bMagic = TRUE;
            break;

        case PM_KEY_BTW:
            pstInfo->pvLocal = &gu8CfgBtw;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_BTW + 0x01);
            pstInfo->tSize = sizeof(gu8CfgBtw);
            pstInfo->bMagic = TRUE;
            break;

        case PM_KEY_EWBS:
            pstInfo->pvLocal = &gstCfgEwbs;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_EWBS + 0x01);
            pstInfo->tSize = sizeof(PM_WoEWBS_WakeCfg);
            pstInfo->bMagic = TRUE;
            break;

        case PM_KEY_USB:
            pstInfo->pvLocal = &gu8CfgUsb;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_USB + 0x01);
            pstInfo->tSize = sizeof(gu8CfgUsb);
            pstInfo->bMagic = TRUE;
            break;

        case PM_KEY_VAD:
            pstInfo->pvLocal = &gu8CfgVad;
            pstInfo->pvDram = (void *)((u8 *)gPM_DataVa + PM_OFFSET_VAD + 0x01);
            pstInfo->tSize = sizeof(gu8CfgVad);
            pstInfo->bMagic = TRUE;
            break;

        default:
            break;
    }

    /* This stage can't copy to DRAM. */
    if (gPM_DataVa == NULL)
    {
        pstInfo->pvDram = NULL;
    }
}

static void _MDrv_PM_Set_Mode(u8 u8Mode)
{
    PM_WakeCfg_t    stCfgDef = {0};

    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
    stCfgDef.u8PmStrMode = u8Mode;
    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (void *)&stCfgDef, sizeof(stCfgDef));
}

static void _MDrv_PM_Set_To_Dram(u16 u16Key, void *pData)
{
    MDRV_BUFF_INFO stInfo = {0};

    _MDrv_PM_Get_Buff(u16Key, &stInfo);

    /* Check result. */
    if (stInfo.tSize == 0)
        MDRV_PM_ERR("Data(0x%X) command fail.\n", u16Key);
    else if (stInfo.pvDram == NULL)
        MDRV_PM_ERR("Data(0x%X) mapping fail.\n", u16Key);
    else
        memcpy((void *)stInfo.pvDram, (const void *)pData, stInfo.tSize);

    /* Write magic number for PM51 check. */
    if (TRUE == stInfo.bMagic)
    {
        *((u8 *)stInfo.pvDram - 0x01) = (u8)u16Key;
    }
}

static void _MDrv_PM_Flush_Config(void)
{
    /* Flush all of local config to dram. */
    _MDrv_PM_Set_To_Dram(PM_KEY_DEFAULT,    (void *)&gstCfgDef);
    _MDrv_PM_Set_To_Dram(PM_KEY_POWER_DOWN, (void *)&gstCfgPowerDown);
    _MDrv_PM_Set_To_Dram(PM_KEY_IR_VERSION, (void *)&gstCfgIr.u8IrVersion);
    _MDrv_PM_Set_To_Dram(PM_KEY_IR_NORMAL,  (void *)&gstCfgIr.u8NormalKeyNumber);
    _MDrv_PM_Set_To_Dram(PM_KEY_IR_EXT,     (void *)&gstCfgIr.u8ExtendKeyNumber);
    _MDrv_PM_Set_To_Dram(PM_KEY_SAR0,       (void *)&gstCfgSar0);
    _MDrv_PM_Set_To_Dram(PM_KEY_SAR1,       (void *)&gstCfgSar1);
    _MDrv_PM_Set_To_Dram(PM_KEY_LED,        (void *)&gu8CfgLed);
    _MDrv_PM_Set_To_Dram(PM_KEY_BT,         (void *)&gstCfgBt);
    _MDrv_PM_Set_To_Dram(PM_KEY_BTW,        (void *)&gu8CfgBtw);
    _MDrv_PM_Set_To_Dram(PM_KEY_EWBS,       (void *)&gstCfgEwbs);
    _MDrv_PM_Set_To_Dram(PM_KEY_USB,        (void *)&gu8CfgUsb);
    _MDrv_PM_Set_To_Dram(PM_KEY_VAD,        (void *)&gu8CfgVad);
    // Add debug: MDrv_PM_Show_Config(XXX);
}

static PM_Result _MDrv_PM_BringUp(void *fw_buf)
{
    /* Disable RTPM. */
    MHal_PM_RunTimePM_Disable_PassWord();

    /* Copy FW Backup -> DRAM. */
    if (_MDrv_PM_Copy_Bin(E_FROM_LOCAL_TO_DRAM, (int)gPM_DramSize,
                          fw_buf, (void *)gPM_DramVa) != E_PM_OK)
    {
        MDRV_PM_ERR("Copy FW from Backup to DRAM fail.\n");
        BUG_ON(1);
    }

    /* Check data. */
    if (strncmp((char *)gPM_DramVa, (char *)fw_buf, gPM_DramSize) != 0)
    {
        MDRV_PM_ERR("Copy FW to DRAM fail, maybe PA=0x%tX SZ=0x%tX can't access.\n",
                    (size_t)gPM_DramAddr, (size_t)gPM_DramSize);
        BUG_ON(1);
    }

    /* Copy FW DRAM -> SRAM. */
    if (MHal_PM_CopyBin2Sram(gPM_DramAddr) != E_PM_OK)
    {
        MDRV_PM_ERR("Copy FW from DRAM to SRAM fail.\n");
        BUG_ON(1);
    }

    /* Write config address to 8051. */
    MHal_PM_SetDram2Register(gPM_DataAddr);

    /* Setting 8051 REG. */
    return MHal_PM_SetSRAMOffsetForMCU();
}

static PM_Result _MDrv_RTPM_BringUp(void)
{
    /* Disable 8051. */
    MHal_PM_Disable_8051();

    /* Copy FW Backup -> DRAM. */
    if (_MDrv_PM_Copy_Bin(E_FROM_LOCAL_TO_DRAM, (int)gRTPM_DramSize,
                          (void *)gRTPM_DramTemp, (void *)gRTPM_DramVa) != E_PM_OK)
    {
        MDRV_PM_ERR("Copy FW from Backup to DRAM fail.\n");
        return E_PM_FAIL;
    }

    /* Check data. */
    if (strncmp((char *)gRTPM_DramVa, (char *)gRTPM_DramTemp, gRTPM_DramSize) != 0)
    {
        MDRV_PM_ERR("Copy FW to DRAM fail, maybe PA=0x%tX SZ=0x%tX can't access.\n",
                    (size_t)gRTPM_DramAddr, (size_t)gRTPM_DramSize);
        BUG_ON(1);
    }

    /* Setting 8051 REG. */
    MHal_PM_SetDRAMOffsetForMCU(_MDrv_PM_Ba2Pa(gRTPM_DramAddr));

    return E_PM_OK;
}

//TODO: Remove.
static void _MDrv_PM_Set_Ir(void)
{
    PM_WakeCfg_t    stCfgDef = {0};
    PM_IrInfoCfg_t  stCfgIr = {0};

    /* Change local config. */
    MDrv_PM_Read_Key(PM_KEY_IR, (void *)&stCfgIr);
    if (stCfgIr.u8IrVersion == IR_INI_VERSION_NUM)
    {
        MDRV_PM_INFO("IRVersion = 0x20,copy gu8PmIr to u8PmWakeIR!\n");
        MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
        memcpy((void *)&stCfgDef.u8PmWakeIR[PM_SUPPORT_SAR_KEYS_MAX],
               (void *)&gu8PmIr[PM_SUPPORT_SAR_KEYS_MAX],
               (sizeof(gu8PmIr) - PM_SUPPORT_SAR_KEYS_MAX));
        MDrv_PM_Write_Key(PM_KEY_DEFAULT, (void *)&stCfgDef, sizeof(stCfgDef));
    }
    else
    {
        MDRV_PM_INFO("IRVersion = 0x10,use origin IRCfg!\n");
    }
}

/* TODO: Remove. */
#ifdef CONFIG_AMAZON_WOL
static void _MDrv_PM_Set_Lan(u8 u8Mode)
{
    PM_WakeCfg_t    stCfgDef = {0};

    MDRV_PM_INFO("Azamon change WOL flag = %d.\n", stCfgDef.bPmWakeEnableWOL);
    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
    stCfgDef.bPmWakeEnableWOL = (u8)atomic_read(&PM_WOL_EN);
    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (void *)&stCfgDef, sizeof(stCfgDef));
}
#endif

/* TODO: Remove. */
#if (CONFIG_MSTAR_GPIO)
static void _MDrv_PM_Set_Led(void)
{
    u8 u8LedPad = 0;

    u8LedPad = MDrv_PM_Get_PowerOffLed();
    MDRV_PM_INFO("Get LedPadNum = %d.\n", u8LedPad);
    if (u8LedPad != PM_SOURCE_DISABLE)
    {
        MDrv_GPIO_Set_Low(u8LedPad);
        MDRV_PM_INFO("======= set red led on ========\n");
    }
}
#endif

static void _MDrv_PM_Patch_Mapping(void)
{
    /* Check patch. */
    if ((gPM_DataVa != NULL) && (gPM_DramVa != NULL))
        return;

    MDRV_PM_ERR("Patch for buffer can't mapping with mmap.\n");

    /* Write data address to kernel. */
    MDrv_PM_Mapping(PM_MMAP_DATA, gPM_DataAddr, gPM_DataSize);

    /* Write fw address to kernel. */
    MDrv_PM_Mapping(PM_MMAP_DRAM, gPM_DramAddr, gPM_DramSize);
}

//-------------------------------------------------------------------------------------------------
//  Golbal function
//-------------------------------------------------------------------------------------------------
void MDrv_PM_Mapping(u16 u16Buf, unsigned long long ullAddr, unsigned long long ullSize)
{
    switch (u16Buf)
    {
        case PM_MMAP_DATA:
            /* Check parameter. */
            if ((ullAddr % MMAP_DATA_ALIGN) || (ullSize != MMAP_DATA_SIZE))
            {
                MDRV_PM_ERR("Paramter error: Addr=0x%tX Size=0x%tX\n",
                            (size_t)ullAddr, (size_t)ullSize);
                WARN_ON(1);
            }
            else
            {
                gPM_DataAddr = ullAddr;
                gPM_DataSize = ullSize;
            }

            /* Mapping data buffer. */
            if (gPM_DataVa == NULL)
            {
                gPM_DataVa = ioremap_wc(_MDrv_PM_Pa2Ba(gPM_DataAddr), gPM_DataSize);
                MDRV_PM_ERR("[DATA] 0x%tX = ioremap_wc(0x%tX, 0x%tX).\n",
                            (size_t)gPM_DataVa,
                            (size_t)_MDrv_PM_Pa2Ba(gPM_DataAddr), (size_t)gPM_DataSize);
            }
            break;

        case PM_MMAP_DRAM:
            /* Check parameter. */
            if ((ullAddr % MMAP_DRAM_ALIGN) || (ullSize != MMAP_DRAM_SIZE))
            {
                MDRV_PM_ERR("Paramter error: Addr=0x%tX Size=0x%tX\n",
                            (size_t)ullAddr, (size_t)ullSize);
                WARN_ON(1);
            }
            else
            {
                gPM_DramAddr = ullAddr;
                gPM_DramSize = ullSize;
            }

            /* Mapping fw buffer. */
            if (gPM_DramVa == NULL)
            {
                gPM_DramVa = ioremap_wc(_MDrv_PM_Pa2Ba(gPM_DramAddr), gPM_DramSize);
                MDRV_PM_ERR("[DRAM] 0x%tX = ioremap_wc(0x%tX, 0x%tX).\n",
                            (size_t)gPM_DramVa,
                            (size_t)_MDrv_PM_Pa2Ba(gPM_DramAddr), (size_t)gPM_DramSize);
            }
            break;

        default:
            break;
    }
}
EXPORT_SYMBOL(MDrv_PM_Mapping);

void MDrv_PM_Unmapping(u16 u16Buf)
{
    switch (u16Buf)
    {
        case PM_MMAP_DATA:
            if (gPM_DataVa != NULL)
            {
                iounmap(gPM_DataVa);
                gPM_DataVa = NULL;
            }
            break;

        case PM_MMAP_DRAM:
            if (gPM_DramVa != NULL)
            {
                iounmap(gPM_DramVa);
                gPM_DramVa = NULL;
            }
            break;

        default:
            break;
    }
}
EXPORT_SYMBOL(MDrv_PM_Unmapping);

void MDrv_PM_Reset_Key(u16 u16Key)
{
    u8 idx = 0;

    switch (u16Key)
    {
        case PM_KEY_DEFAULT:
            memset(&gstCfgDef, 0, sizeof(PM_WakeCfg_t));
            gstCfgDef.bPmWakeEnableIR = 1;
            gstCfgDef.bPmWakeEnableSAR = 1;
            gstCfgDef.bPmWakeEnableGPIO0 = 1;
            gstCfgDef.bPmWakeEnableRTC0 = 1;
            gstCfgDef.bPmWakeEnableRTC1 = 1;
            gstCfgDef.bPmWakeEnableCEC = 1;

            gstCfgDef.u8PmWakeIR[0] = 0x46;
            for (idx = 1; idx < MAX_BUF_WAKE_IR; idx++)
                gstCfgDef.u8PmWakeIR[idx] = PM_SOURCE_DISABLE;
            gstCfgDef.u8PmWakeIR2[0] = 0x46;
            for (idx = 1; idx < MAX_BUF_WAKE_IR2; idx++)
                gstCfgDef.u8PmWakeIR2[idx] = PM_SOURCE_DISABLE;

            gstCfgDef.u8PmStrMode = PM_DC_MODE;

            gstCfgDef.u8PmWakeEnableWOWLAN = PM_SOURCE_DISABLE;

            break;

        case PM_KEY_POWER_DOWN:
            memset(&gstCfgPowerDown, 0, sizeof(PM_PowerDownCfg_t));
            gstCfgPowerDown.u8PowerDownMode = E_PM_STANDBY;
            gstCfgPowerDown.u8WakeAddress = E_PM_LAST_TWOSTAGE_POWERDOWN;
            break;

        case PM_KEY_SAR0:
            memset(&gstCfgSar0, 0, sizeof(SAR_RegCfg));
            gstCfgSar0.u8SARChID = 0x00;
            gstCfgSar0.tSARChBnd.u8UpBnd = 0xFF;
            gstCfgSar0.tSARChBnd.u8LoBnd = 0xF0;
            gstCfgSar0.u8KeyLevelNum = 0x08;

            gstCfgSar0.u8KeyThreshold[0] = 0x10;
            gstCfgSar0.u8KeyThreshold[1] = 0x2F;
            gstCfgSar0.u8KeyThreshold[2] = 0x4D;
            gstCfgSar0.u8KeyThreshold[3] = 0x71;
            gstCfgSar0.u8KeyThreshold[4] = 0x92;
            gstCfgSar0.u8KeyThreshold[5] = 0xAB;
            gstCfgSar0.u8KeyThreshold[6] = 0xC3;
            gstCfgSar0.u8KeyThreshold[7] = 0xE7;

            gstCfgSar0.u8KeyCode[0] = 0x46;
            gstCfgSar0.u8KeyCode[1] = 0xA8;
            gstCfgSar0.u8KeyCode[2] = 0xA4;
            gstCfgSar0.u8KeyCode[3] = 0xA2;
            break;

        case PM_KEY_SAR1:
            memset(&gstCfgSar1, 0, sizeof(SAR_RegCfg));
            break;

        case PM_KEY_LED:
            memset(&gu8CfgLed, 0, sizeof(gu8CfgLed));
            break;

        case PM_KEY_BT:
            memset(&gstCfgBt, 0, sizeof(PM_WoBT_WakeCfg));
            gstCfgBt.u8GpioNum = PM_SOURCE_DISABLE;
            break;

        case PM_KEY_EWBS:
            memset(&gstCfgEwbs, 0, sizeof(PM_WoEWBS_WakeCfg));
            gstCfgEwbs.u8GpioNum = PM_SOURCE_DISABLE;
            break;

        case PM_KEY_USB:
            gu8CfgUsb = PM_SOURCE_DISABLE;
            break;

        case PM_KEY_VAD:
            gu8CfgVad = PM_SOURCE_DISABLE;
            break;

        case PM_KEY_BTW:
            gu8CfgBtw = PM_SOURCE_DISABLE;
            break;

        default:
            break;
    }
}

ssize_t MDrv_PM_Read_Key(u16 u16Key, const char *buf)
{
    MDRV_BUFF_INFO stInfo = {0};

    _MDrv_PM_Get_Buff(u16Key, &stInfo);

    /* Check result. */
    if ((stInfo.pvLocal == NULL) || (stInfo.tSize == 0))
        MDRV_PM_ERR("Data(0x%X) command fail.\n", u16Key);
    else
        memcpy((void *)buf, (const void *)stInfo.pvLocal, stInfo.tSize);

    return stInfo.tSize;
}
EXPORT_SYMBOL(MDrv_PM_Read_Key);

ssize_t MDrv_PM_Write_Key(u16 u16Key, const char *buf, size_t size)
{
    MDRV_BUFF_INFO stInfo = {0};

    /* Only for debug. */
    if (gbDebug == TRUE)
        return size;

    _MDrv_PM_Get_Buff(u16Key, &stInfo);

    /* Check result and align size. */
    if ((stInfo.pvLocal == NULL) || (stInfo.tSize == 0))
        MDRV_PM_ERR("Data(0x%X) command fail.\n", u16Key);
    else if (size != stInfo.tSize)
        MDRV_PM_ERR("Data(0x%X) un-align: input_size=0x%zX, internal_size=0x%zX\n", u16Key, size, stInfo.tSize);
    else
        memcpy((void *)stInfo.pvLocal, (const void *)buf, stInfo.tSize);

    return stInfo.tSize;
}
EXPORT_SYMBOL(MDrv_PM_Write_Key);

void MDrv_PM_Show_Config(u16 u16Key)
{
    u8 i = 0;

    pr_info("==========================\n");
    switch (u16Key)
    {
        case PM_KEY_DEFAULT:
            pr_info("EnableIR      = %X.\n",       gstCfgDef.bPmWakeEnableIR);
            pr_info("EnableSAR     = %X.\n",       gstCfgDef.bPmWakeEnableSAR);
            pr_info("EnableGPIO0   = %X.\n",       gstCfgDef.bPmWakeEnableGPIO0);
            pr_info("EnableGPIO1   = %X.\n",       gstCfgDef.bPmWakeEnableGPIO1);
            pr_info("EnableUART1   = %X.\n",       gstCfgDef.bPmWakeEnableUART1);
            pr_info("EnableSYNC    = %X.\n",       gstCfgDef.bPmWakeEnableSYNC);
            pr_info("EnableRTC0    = %X.\n",       gstCfgDef.bPmWakeEnableRTC0);
            pr_info("EnableRTC1    = %X.\n",       gstCfgDef.bPmWakeEnableRTC1);
            pr_info("EnableDVI0    = %X.\n",       gstCfgDef.bPmWakeEnableDVI0);
            pr_info("EnableDVI2    = %X.\n",       gstCfgDef.bPmWakeEnableDVI2);
            pr_info("EnableCEC     = %X.\n",       gstCfgDef.bPmWakeEnableCEC);
            pr_info("EnableAVLINK  = %X.\n",       gstCfgDef.bPmWakeEnableAVLINK);
            pr_info("EnableMHL     = %X.\n",       gstCfgDef.bPmWakeEnableMHL);
            pr_info("EnableWOL     = %X.\n",       gstCfgDef.bPmWakeEnableWOL);
            pr_info("EnableCM4     = %X.\n",       gstCfgDef.bPmWakeEnableCM4);
            pr_info("==========================\n");
            pr_info("WIFI GPIO NUM = 0x%02X.\n",   gstCfgDef.u8PmWakeEnableWOWLAN);
            pr_info("WIFI GPIO POL = 0x%02X.\n",   gstCfgDef.u8PmWakeWOWLANPol);
            pr_info("==========================\n");
            pr_info("PmStrMode     = 0x%X.\n",     gstCfgDef.u8PmStrMode);
            pr_info("==========================\n");
            for (i = 0; i < MAX_BUF_WAKE_MAC_ADDRESS; i++)
                pr_info("MAC           = 0x%02X.\n",   gstCfgDef.u8PmWakeMACAddress[i]);
            break;

        case PM_KEY_POWER_DOWN:
            pr_info("PowerDownMode = 0x%X.\n",     gstCfgPowerDown.u8PowerDownMode);
            pr_info("WakeAddress   = 0x%X.\n",     gstCfgPowerDown.u8WakeAddress);
            break;

        case PM_KEY_IR:
            for (i = 0; i < MAX_BUF_WAKE_IR; i++)
                pr_info("IR List       = 0x%02X.\n",   gstCfgDef.u8PmWakeIR[i]);
            for (i = 0; i < MAX_BUF_WAKE_IR2; i++)
                pr_info("IR List2      = 0x%02X.\n",   gstCfgDef.u8PmWakeIR2[i]);
            break;

        case PM_KEY_IR_VERSION:
            pr_info("IR version    = 0x%X.\n",     gstCfgIr.u8IrVersion);
            break;

        case PM_KEY_IR_NORMAL:
            pr_info("IR Normal Num = %d.\n",       gstCfgIr.u8NormalKeyNumber);
            for (i = 0; i <= gstCfgIr.u8NormalKeyNumber; i++)
                pr_info("NormalKey[%d]  = 0x%02X.\n",i , gstCfgIr.au8IrNormalKey[i]);
            break;

        case PM_KEY_IR_EXT:
            pr_info("IR Extend Num = %d.\n",       gstCfgIr.u8ExtendKeyNumber);
            for (i = 0; i <= gstCfgIr.u8ExtendKeyNumber; i++)
                pr_info("ExtendKey[%d]  = 0x%02X.\n",i , gstCfgIr.au8IrExtendKey[i]);
            break;

        case PM_KEY_SAR0:
            pr_info("SAR0 ChID     = 0x%02X.\n",   gstCfgSar0.u8SARChID);
            pr_info("SAR0 UpBnd    = 0x%02X.\n",   gstCfgSar0.tSARChBnd.u8UpBnd);
            pr_info("SAR0 LoBnd    = 0x%02X.\n",   gstCfgSar0.tSARChBnd.u8LoBnd);
            pr_info("SAR0 KeyLVNum = 0x%02X.\n",   gstCfgSar0.u8KeyLevelNum);
            for (i = 0; i < gstCfgSar0.u8KeyLevelNum; i++)
                pr_info("Threshold[%d]  = 0x%02X.\n", i, gstCfgSar0.u8KeyThreshold[i]);
            for (i = 0; i < gstCfgSar0.u8KeyLevelNum; i++)
                pr_info("KeyCode[%d]    = 0x%02X.\n", i, gstCfgSar0.u8KeyCode[i]);
            break;

        case PM_KEY_SAR1:
            pr_info("SAR1 ChID     = 0x%02X.\n",   gstCfgSar1.u8SARChID);
            pr_info("SAR1 UpBnd    = 0x%02X.\n",   gstCfgSar1.tSARChBnd.u8UpBnd);
            pr_info("SAR1 LoBnd    = 0x%02X.\n",   gstCfgSar1.tSARChBnd.u8LoBnd);
            pr_info("SAR1 KeyLVNum = 0x%02X.\n",   gstCfgSar1.u8KeyLevelNum);
            for (i = 0; i < gstCfgSar1.u8KeyLevelNum; i++)
                pr_info("Threshold[%d]  = 0x%02X.\n", i, gstCfgSar1.u8KeyThreshold[i]);
            for (i = 0; i < gstCfgSar1.u8KeyLevelNum; i++)
                pr_info("KeyCode[%d]    = 0x%02X.\n", i, gstCfgSar1.u8KeyCode[i]);
            break;

        case PM_KEY_LED:
           for (i = 0; i < 10; i++)
                pr_info("gu8CfgLed[%d] = 0x%02X.\n",i, gu8CfgLed[i]);
            break;

        case PM_KEY_BT:
            pr_info("BT   GPIO NUM = 0x%02X.\n",   gstCfgBt.u8GpioNum);
            pr_info("BT   GPIO POL = 0x%02X.\n",   gstCfgBt.u8Polarity);
            break;

        case PM_KEY_BTW:
            pr_info("BT   Wave     = 0x%02X.\n",   gu8CfgBtw);
            break;

        case PM_KEY_EWBS:
            pr_info("EWBS GPIO NUM = 0x%02X.\n",   gstCfgEwbs.u8GpioNum);
            pr_info("EWBS GPIO POL = 0x%02X.\n",   gstCfgEwbs.u8Polarity);
            break;

        case PM_KEY_USB:
            pr_info("EnableUSB     = 0x%02X.\n",   gu8CfgUsb);
            break;

        case PM_KEY_VAD:
            pr_info("EnableVAD     = 0x%02X.\n",   gu8CfgVad);
            break;

        default:
            break;
    }
}

PM_Result MDrv_PM_Suspend(PM_STATE eState)
{
    PM_Result eRet = E_PM_OK;
    PM_WakeCfg_t stCfgDef = {0};

    switch (eState)
    {
        case E_PM_STATE_SUSPEND_PRE:
            if (!is_mstar_str())
            {
                // MDrv_PM_Mapping() via mi notify callback.
                eRet &= _MDrv_PM_Store_Fw();
            }
            if (gRTPM_Enable == 1)
            {
                _MDrv_RTPM_Mapping();
                // Need to check (eRet &= XXX), but AN, MI not sync.
                _MDrv_RTPM_Store_Fw();
            }
            break;

        case E_PM_STATE_SUSPEND_NOIRQ:
            if (!is_mstar_str())
            {
                /* TODO: Remove. */
                _MDrv_PM_Set_Ir();
                #ifdef CONFIG_AMAZON_WOL
                _MDrv_PM_Set_Lan();
                #endif

                #if (CONFIG_MSTAR_GPIO)
                _MDrv_PM_Set_Led();
                #endif

                /* Check /sys/power/str_max_cnt trigger. */
                _MDrv_PM_Set_Mode(MDrv_MPM_Check_DC() ? PM_DC_MODE : PM_STR_MODE);
                /* TODO: Support quie boot. */
                if (MDrv_MPM_Check_DC() && disable_background_wakeup)
                {
                    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (void *)&stCfgDef);
                    stCfgDef.u8PmWakeEnableWOWLAN = PM_SOURCE_DISABLE;
                    stCfgDef.bPmWakeEnableRTC0 = 0;
                    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (void *)&stCfgDef, sizeof(stCfgDef));
                }
                _MDrv_PM_Flush_Config();
                eRet &= _MDrv_PM_BringUp((void *)gPM_DramTemp);
            }
            break;

        default:
            eRet = E_PM_FAIL;
            break;
    }

    return eRet;
}

PM_Result MDrv_PM_Resume(PM_STATE eState)
{
    PM_Result eRet = E_PM_OK;

    switch (eState)
    {
        case E_PM_STATE_RESUME_NOIRQ:
            if (gRTPM_Enable == 1)
            {
                eRet &= _MDrv_RTPM_BringUp();
            }
            break;

        case E_PM_STATE_RESUME_COMPLETE:
            if (!is_mstar_str())
            {
                // MDrv_PM_Unmapping() via mi notify callback.
                _MDrv_PM_Freed_Fw();
            }
            if (gRTPM_Enable == 1)
            {
                _MDrv_RTPM_Unmapping();
                _MDrv_RTPM_Freed_Fw();
            }
            break;

        default:
            eRet = E_PM_FAIL;
            break;
    }

    return eRet;
}

PM_Result MDrv_PM_Reboot(PM_STATE eState)
{
    PM_Result eRet = E_PM_OK;

    switch (eState)
    {
        case E_PM_STATE_REBOOT_NOTIFY:
            if (!is_mstar_str())
            {
                // MDrv_PM_Mapping() via mi notify callback.
                eRet &= _MDrv_PM_Store_Fw();
            }
            break;

        case E_PM_STATE_POWER_OFF_PRE:
            if (!is_mstar_str())
            {
                /* TODO: Remove. */
                _MDrv_PM_Set_Ir();
                #ifdef CONFIG_MSTAR_SYSFS_BACKLIGHT
                MDrv_PM_TurnoffBacklight();
                #endif

                #if (CONFIG_MSTAR_GPIO)
                _MDrv_PM_Set_Led();
                #endif

                // TODO: Remove patch for reboot recovery case.
                _MDrv_PM_Patch_Mapping();

                _MDrv_PM_Set_Mode(PM_DC_MODE);
                _MDrv_PM_Flush_Config();
                eRet &= _MDrv_PM_BringUp((void *)gPM_DramTemp);
            }
            break;

        default:
            eRet = E_PM_FAIL;
            break;
    }

    return eRet;
}

PM_Result MDrv_PM_Panic(void)
{
    _MDrv_PM_Patch_Mapping();
    _MDrv_PM_Set_Mode(PM_STR_MODE);
    _MDrv_PM_Flush_Config();
    return _MDrv_PM_BringUp((void *)gPanic);
}

//--------------------------------------------------------------------------------------------------
// Other
//--------------------------------------------------------------------------------------------------
u8 MDrv_PM_GetWakeupSource(void)
{
    return MHal_PM_GetWakeupSource();
}

u8 MDrv_PM_GetPowerOnKey(void)
{
    return MHal_PM_GetPowerOnKey();
}

void MDrv_PM_CleanPowerOnKey(void)
{
    MHal_PM_CleanPowerOnKey();
}

/* TODO: Remove to ir driver. */
ssize_t MDrv_PM_Set_IRCfg(const u8 *buf, size_t size)
{
    memcpy((void *)gu8PmIr, buf, sizeof(gu8PmIr));

    return sizeof(gu8PmIr);
}
EXPORT_SYMBOL(MDrv_PM_Set_IRCfg);

void MDrv_PM_WakeIrqMask(u8 mask)
{
    #ifdef CONFIG_KEYBOARD_MTK
    MHal_PM_WakeIrqMask(mask);
    #endif
}
EXPORT_SYMBOL(MDrv_PM_WakeIrqMask);

void MDrv_PM_WDT_RST(void)
{
    #ifdef CONFIG_KEYBOARD_MTK
    MHAL_PM_WdtRst();
    #endif
}
EXPORT_SYMBOL(MDrv_PM_WDT_RST);

/* TODO: Remove to led framework. */
u8 MDrv_PM_Get_PowerOffLed(void)
{
    return gu8PmLed;
}

void MDrv_PM_Set_PowerOffLed(u8 u8LedPad)
{
    MDRV_PM_INFO("LedPadNum = %d.\n", u8LedPad);
    gu8PmLed = u8LedPad;
}

#ifdef CONFIG_AMAZON_WOL
void MDrv_PM_Show_PM_WOL_EN(void)
{
    MDRV_PM_INFO("Current Parameter of WOL_EN=%d \n", atomic_read(&PM_WOL_EN));
}

void MDrv_PM_SetPM_WOL_EN(const char *buf)
{
    unsigned int WOL_EN = 0;
    int readCount = 0;

    readCount = sscanf(buf, "%d", &WOL_EN);
    if (readCount != 1) {
        MDRV_PM_ERR("ERROR cannot read WOL_EN from [%s] \n", buf);
        return;
    }
    if (WOL_EN > 0x01) {
        MDRV_PM_ERR("ERROR Parameter WOL_EN=%d \n", WOL_EN);
        return;
    }

    MDRV_PM_INFO("Set Parameter WOL_EN=%d success\n", WOL_EN);
    atomic_set(&PM_WOL_EN, WOL_EN);
}
#endif

#ifdef CONFIG_MSTAR_SYSFS_BACKLIGHT
PM_Result MDrv_PM_TurnoffBacklight(void)
{
    struct backlight_device *bd;

    bd = backlight_device_get_by_type(BACKLIGHT_RAW);
    if (!bd) {
        MDRV_PM_ERR("MDrv_PM_TurnoffBacklight: get BACKLIGHT_RAW dev failed\n");
        return -ENODEV;
    }

    backlight_device_set_brightness(bd, 0);

    return E_PM_OK;
}
#endif

//--------------------------------------------------------------------------------------------------
// DEBUG
//--------------------------------------------------------------------------------------------------
void MDrv_PM_Read_Debug(void)
{
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // CAE Power Test   , Only IR wakeup.\n",       DBG_CMD_CAE);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x00 0xFF' // Power Down Mode  , Standby.\n",              DBG_CMD_PW);
    pr_info("echo '0x%02X 0x01 0xFF' // Power Down Mode  , Sleep.\n",                DBG_CMD_PW);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x01 0xFF' // STR Mode         , DC.\n",                   DBG_CMD_STR);
    pr_info("echo '0x%02X 0x02 0xFF' // STR Mode         , STR.\n",                  DBG_CMD_STR);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x00 0xFF' // DVI0/1/3 WK      , Disable.\n",              DBG_CMD_DVI013);
    pr_info("echo '0x%02X 0x01 0xFF' // DVI0/1/3 WK      , Enable.\n",               DBG_CMD_DVI013);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x00 0xFF' // DVI2 WK          , Disable.\n",              DBG_CMD_DVI2);
    pr_info("echo '0x%02X 0x01 0xFF' // DVI2 WK          , Enable.\n",               DBG_CMD_DVI2);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x00 0xFF' // WOL WK           , Disable.\n",              DBG_CMD_LAN);
    pr_info("echo '0x%02X 0x01 0xFF' // WOL WK           , Enable.\n",               DBG_CMD_LAN);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x00 0xFF' // CM4 WK           , Disable.\n",              DBG_CMD_VOC);
    pr_info("echo '0x%02X 0x01 0xFF' // CM4 WK           , Enable.\n",               DBG_CMD_VOC);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // USB WK           , Disable.\n",              DBG_CMD_USB);
    pr_info("echo '0x%02X 0x01 0xFF' // USB WK           , Enable.\n",               DBG_CMD_USB);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // VAD WK           , Disable.\n",              DBG_CMD_VAD);
    pr_info("echo '0x%02X 0x01 0xFF' // VAD WK           , Enable.\n",               DBG_CMD_VAD);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0x00 0xFF' // CEC WK           , Disable.\n",              DBG_CMD_CEC);
    pr_info("echo '0x%02X 0x01 0xFF' // CEC WK           , Enable.\n",               DBG_CMD_CEC);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // BTW WK           , Disable.\n",              DBG_CMD_BTW);
    pr_info("echo '0x%02X 0x01 0xFF' // BTW WK           , Enable.\n",               DBG_CMD_BTW);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // WIFI WK          , Disable.\n",              DBG_CMD_WIFI);
    pr_info("echo '0x%02X 0xNN 0x00' // WIFI WK          , Enable Num, Rising.\n",   DBG_CMD_WIFI);
    pr_info("echo '0x%02X 0xNN 0x01' // WIFI WK          , Enable Num, Falling.\n",  DBG_CMD_WIFI);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // BT WK            , Disable.\n",              DBG_CMD_BT);
    pr_info("echo '0x%02X 0xNN 0x00' // BT WK            , Enable Num, Rising.\n",   DBG_CMD_BT);
    pr_info("echo '0x%02X 0xNN 0x01' // BT WK            , Enable Num, Falling.\n",  DBG_CMD_BT);
    pr_info("===================================================================\n");
    pr_info("echo '0x%02X 0xFF 0xFF' // EWBS WK          , Disable.\n",              DBG_CMD_EWBS);
    pr_info("echo '0x%02X 0xNN 0x00' // EWBS WK          , Enable Num, Rising.\n",   DBG_CMD_EWBS);
    pr_info("echo '0x%02X 0xNN 0x01' // EWBS WK          , Enable Num, Falling.\n",  DBG_CMD_EWBS);
    pr_info("===================================================================\n");
}

void MDrv_PM_Write_Debug(const char *buf, size_t size)
{
    bool bDebug = TRUE;
    u8 u8Cmd = 0, u8Param0 = 0, u8Param1 = 0;
    PM_WakeCfg_t         stCfgDef = {0};
    PM_PowerDownCfg_t    stCfgPowerDown = {0};
    PM_WoBT_WakeCfg      stCfgBt = {0};
    PM_WoEWBS_WakeCfg    stCfgEwbs = {0};
    u8                   u8CfgBtw = PM_SOURCE_DISABLE;
    u8                   u8CfgUsb = PM_SOURCE_DISABLE;
    u8                   u8CfgVad = PM_SOURCE_DISABLE;

    if (size != DBG_CMD_SIZE)
    {
        MDRV_PM_ERR("Please reference command list:\n");
        MDrv_PM_Read_Debug();
        return;
    }

    sscanf(buf, "%hhX %hhX %hhX\n",  &u8Cmd, &u8Param0, &u8Param1);
    MDRV_PM_INFO("Cmd=0x%02tX Param0=0x%02tX Param1=0x%02tX.\n",
                 (size_t)u8Cmd, (size_t)u8Param0, (size_t)u8Param1);

    gbDebug = FALSE;
    MDrv_PM_Read_Key(PM_KEY_DEFAULT,    (void *)&stCfgDef);
    MDrv_PM_Read_Key(PM_KEY_POWER_DOWN, (void *)&stCfgPowerDown);
    MDrv_PM_Read_Key(PM_KEY_BT,         (void *)&stCfgBt);
    MDrv_PM_Read_Key(PM_KEY_BTW,        (void *)&u8CfgBtw);
    MDrv_PM_Read_Key(PM_KEY_EWBS,       (void *)&stCfgEwbs);
    MDrv_PM_Read_Key(PM_KEY_USB,        (void *)&u8CfgUsb);
    MDrv_PM_Read_Key(PM_KEY_VAD,        (void *)&u8CfgVad);
    switch (u8Cmd)
    {
        case DBG_CMD_CAE:
            memset(&stCfgDef, 0, 2);
            stCfgDef.bPmWakeEnableIR = 1;
            stCfgDef.u8PmWakeEnableWOWLAN = PM_SOURCE_DISABLE;
            stCfgBt.u8GpioNum = PM_SOURCE_DISABLE;
            u8CfgBtw = PM_SOURCE_DISABLE;
            stCfgEwbs.u8GpioNum = PM_SOURCE_DISABLE;
            u8CfgUsb = PM_SOURCE_DISABLE;
            break;
        case DBG_CMD_PW:
            stCfgPowerDown.u8PowerDownMode = u8Param0;
            break;
        case DBG_CMD_STR:
            stCfgDef.u8PmStrMode = u8Param0;
            break;
        case DBG_CMD_DVI013:
            stCfgDef.bPmWakeEnableDVI0 = u8Param0;
            break;
        case DBG_CMD_DVI2:
            stCfgDef.bPmWakeEnableDVI2 = u8Param0;
            break;
        case DBG_CMD_LAN:
            stCfgDef.bPmWakeEnableWOL = u8Param0;
            break;
        case DBG_CMD_VOC:
            stCfgDef.bPmWakeEnableCM4 = u8Param0;
            break;
        case DBG_CMD_USB:
            u8CfgUsb = u8Param0;
            break;
        case DBG_CMD_VAD:
            u8CfgVad = u8Param0;
            break;
        case DBG_CMD_CEC:
            stCfgDef.bPmWakeEnableCEC = u8Param0;
            break;
        case DBG_CMD_BTW:
            u8CfgBtw = u8Param0;
            break;
        case DBG_CMD_WIFI:
            stCfgDef.u8PmWakeEnableWOWLAN = u8Param0;
            stCfgDef.u8PmWakeWOWLANPol = u8Param1;
            break;
        case DBG_CMD_BT:
            stCfgBt.u8GpioNum = u8Param0;
            stCfgBt.u8Polarity = u8Param1;
            break;
        case DBG_CMD_EWBS:
            stCfgEwbs.u8GpioNum = u8Param0;
            stCfgEwbs.u8Polarity = u8Param1;
            break;
        default:
            MDRV_PM_INFO("Command not support, exsit debug mode.\n");
            bDebug = FALSE;
            break;
    }
    MDrv_PM_Write_Key(PM_KEY_DEFAULT,       (void *)&stCfgDef,          sizeof(stCfgDef));
    MDrv_PM_Write_Key(PM_KEY_POWER_DOWN,    (void *)&stCfgPowerDown,    sizeof(stCfgPowerDown));
    MDrv_PM_Write_Key(PM_KEY_BT,            (void *)&stCfgBt,           sizeof(stCfgBt));
    MDrv_PM_Write_Key(PM_KEY_BTW,           (void *)&u8CfgBtw,          sizeof(u8CfgBtw));
    MDrv_PM_Write_Key(PM_KEY_EWBS,          (void *)&stCfgEwbs,         sizeof(stCfgEwbs));
    MDrv_PM_Write_Key(PM_KEY_USB,           (void *)&u8CfgUsb,          sizeof(u8CfgUsb));
    MDrv_PM_Write_Key(PM_KEY_VAD,           (void *)&u8CfgVad,          sizeof(u8CfgVad));
    gbDebug = bDebug;
}

//--------------------------------------------------------------------------------------------------
// EARLY_PARAM
//--------------------------------------------------------------------------------------------------
static int __init MDrv_PM_Set_Info(char *str)
{
    if( str != NULL)
    {
        strncpy(gPM_Path, str, (CORENAME_MAX_SIZE - 1));
        gPM_Path[sizeof(gPM_Path) - 1] = '\0';
    }
    return 0;
}

static int __init MDrv_RTPM_Set_Info(char *str)
{
    /* TODO: Need to parsing mmap. */
    if (str != NULL)
    {
        sscanf(str, "%u, %llx", &gRTPM_Enable, &gRTPM_DramAddr);
    }
    return 0;
}

static int __init MDrv_RTPM_Set_Slot_Info(char *str)
{
    if( str != NULL)
    {
        strncpy(gSlot_Suffix, str, 2);
        gSlot_Suffix[sizeof(gSlot_Suffix) - 1] = '\0';
    }
    return 0;
}
early_param("pm_path", MDrv_PM_Set_Info);
early_param("RTPM_INFO", MDrv_RTPM_Set_Info);
early_param("androidboot.slot_suffix", MDrv_RTPM_Set_Slot_Info);

//--------------------------------------------------------------------------------------------------
//TODO: Remove API as follow.
//--------------------------------------------------------------------------------------------------
PM_Result MDrv_PM_CopyBin2Dram(void)
{
    return E_PM_OK;
}
EXPORT_SYMBOL(MDrv_PM_CopyBin2Dram);
