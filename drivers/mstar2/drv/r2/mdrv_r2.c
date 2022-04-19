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
/// file    mdrv_r2.c
/// @brief  Coprocessor related functions
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
/* From linux. */
#include <asm/io.h>
#include <asm/uaccess.h>
#include <crypto/internal/hash.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>

/* From mstar. */
#include "mdrv_r2.h"
#include "mhal_r2.h"
#include "mdrv_system.h"
#include "mdrv_types.h"

//--------------------------------------------------------------------------------------------------
//  Debug
//--------------------------------------------------------------------------------------------------
#define MDRV_R2_DEBUG(fmt, args...)  printk(KERN_ERR "[%s][%d] " fmt, __func__, __LINE__, ## args)

//--------------------------------------------------------------------------------------------------
//  Forward declaration
//--------------------------------------------------------------------------------------------------
#define MDRV_R2_CODE_SZ     (0x100000)
#define MDRV_R2_BUFF_SZ     (128)

#if defined(CONFIG_ARM)
#define MDRV_R2_MAP_PROT    pgprot_noncached(L_PTE_MT_BUFFERABLE | L_PTE_XN)
#elif defined(CONFIG_ARM64)
#define MDRV_R2_MAP_PROT    __pgprot(PROT_NORMAL_NC)
#else
#error "MDRV_R2_MAP_PROT undeclared"
#endif

//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------
typedef struct _MDRV_R2_INFO
{
    U8                      u8Enable;
    U64                     u64Address;
    U8                      pu8Path[MDRV_R2_BUFF_SZ];
} MDRV_R2_INFO;

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------
#if defined(CONFIG_MSTAR_FRC_R2)
static MDRV_R2_INFO         _stFRC_R2 = {0};
static unsigned char        _pFRC_R2_BIN[MDRV_R2_CODE_SZ] __attribute__((__aligned__(16))) = {0};
static unsigned int         _FRC_R2_Live = 0;
static unsigned int         _FRC_R2_Debug = 0;
static unsigned char        _FRC_Part[10] = "frc";
#endif

//-------------------------------------------------------------------------------------------------
//  Golbal variable
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local function
//-------------------------------------------------------------------------------------------------
static U64 _MDrv_R2_Ba2Pa(U64 ba)
{
    U64 pa = 0;

    if ((ba > ARM_MIU0_BUS_BASE) && (ba <= ARM_MIU1_BUS_BASE))
        pa = ba - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
    else if ((ba > ARM_MIU1_BUS_BASE) && (ba <= ARM_MIU2_BUS_BASE))
        pa = ba - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;
    else
        MDRV_R2_DEBUG("ba=0x%llX, pa=0x%llX.\n", ba, pa);

    return pa;
}

static int _MDrv_R2_Get_Path(char *part, char *path)
{
    int ret = 0;
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;
    int blk = 1;
    char buff[MDRV_R2_BUFF_SZ] = {0};
    char key[MDRV_R2_BUFF_SZ] = {0};

    /* Find /dev/block/platform/mstar_mci.0/by-name/? exist. */
    snprintf(buff, MDRV_R2_BUFF_SZ, "/dev/block/platform/mstar_mci.0/by-name/%s", part);
    fp = filp_open(buff, O_RDONLY, 0);
    if ((!IS_ERR(fp)) || (PTR_ERR(fp) == -EACCES))
    {
        strncpy(path, buff, MDRV_R2_BUFF_SZ);
        if (!IS_ERR(fp))
            filp_close(fp, NULL);
        return 0;
    }

    /* Store fs. */
    fs = get_fs();
    set_fs(KERNEL_DS);

    /* Find /dev/mmcblk0p#. */
    memset(key, '\0', MDRV_R2_BUFF_SZ);
    snprintf(key, MDRV_R2_BUFF_SZ, "PARTNAME=%s", part);
    do
    {
        memset(buff, '\0', MDRV_R2_BUFF_SZ);
        snprintf(buff, MDRV_R2_BUFF_SZ, "/sys/block/mmcblk0/mmcblk0p%d/uevent", blk);
        fp = filp_open(buff, O_RDONLY, 0);
        if (IS_ERR(fp))
        {
            MDRV_R2_DEBUG("%s R2 path not found.\n", part);
            ret = -ENODATA;
            goto _MDrv_R2_Get_Path_End;
        }
        memset(buff, '\0', MDRV_R2_BUFF_SZ);
        vfs_read(fp, buff, MDRV_R2_BUFF_SZ, &pos);
        filp_close(fp, NULL);

        /* Check found. */
        if (strnstr(buff, key, MDRV_R2_BUFF_SZ))
        {
            memset(buff, '\0', MDRV_R2_BUFF_SZ);
            snprintf(buff, MDRV_R2_BUFF_SZ, "/dev/mmcblk0p%d", blk);
            fp = filp_open(buff, O_RDONLY, 0);
            if (!IS_ERR(fp))
            {
                snprintf(path, MDRV_R2_BUFF_SZ, "/dev/mmcblk0p%d", blk);
                filp_close(fp, NULL);
            }
            else
            {
                snprintf(path, MDRV_R2_BUFF_SZ, "/dev/mmcblk%d", blk);
            }
            break;
        }
        else
        {
            pos = 0;
            blk++;
        }
    } while (1);

_MDrv_R2_Get_Path_End:
    /* Restore fs. */
    set_fs(fs);

    return ret;
}

static void _MDrv_R2_Cal_MD5(char *name, unsigned char *src, loff_t size)
{
    char dst[16] = {0};
    struct crypto_shash *md5;
    struct shash_desc *shash = NULL;

    md5 = crypto_alloc_shash("md5", 0, 0);
    if (IS_ERR(md5))
    {
        MDRV_R2_DEBUG("crypto_alloc_shash failed(%td)!\n", (size_t)PTR_ERR(md5));
        goto _MDrv_R2_Cal_MD5_End;
    }

    shash = kmalloc(sizeof(struct shash_desc) + crypto_shash_descsize(md5), GFP_KERNEL);
    if (!shash)
    {
        MDRV_R2_DEBUG("kmalloc failed!\n");
        goto _MDrv_R2_Cal_MD5_End;
    }

    shash->tfm = md5;
    shash->flags = 0;

    if (crypto_shash_init(shash))
    {
        MDRV_R2_DEBUG("crypto_shash_init\n");
        goto _MDrv_R2_Cal_MD5_End;
    }

    if (crypto_shash_update(shash, src, size))
    {
        MDRV_R2_DEBUG("crypto_shash_update failed!\n");
        goto _MDrv_R2_Cal_MD5_End;
    }

    if (crypto_shash_final(shash, dst))
    {
        MDRV_R2_DEBUG("crypto_shash_final failed!\n");
        goto _MDrv_R2_Cal_MD5_End;
    }

    MDRV_R2_DEBUG("%s MD5: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", name,
                (unsigned char)dst[0], (unsigned char)dst[1], (unsigned char)dst[2], (unsigned char)dst[3],
                (unsigned char)dst[4], (unsigned char)dst[5], (unsigned char)dst[6], (unsigned char)dst[7],
                (unsigned char)dst[8], (unsigned char)dst[9], (unsigned char)dst[10], (unsigned char)dst[11],
                (unsigned char)dst[12], (unsigned char)dst[13], (unsigned char)dst[14], (unsigned char)dst[15]);
_MDrv_R2_Cal_MD5_End:
    if (!IS_ERR(md5))
        crypto_free_shash(md5);
    if (shash)
        kfree(shash);

    return;
}

static int _MDrv_R2_Copy_Part_To_Local(char *src, char *dst, int size)
{
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;

    /* Check para. */
    if (strlen(src) == 0)
    {
        MDRV_R2_DEBUG("Invalid path: src=%s.\n", src);
        return -EINVAL;
    }
    /* Get source. */
    fp = filp_open(src, O_RDONLY, 0);
    if (IS_ERR(fp))
    {
        MDRV_R2_DEBUG("filp_open(%s) fail = %td.\n", src, (size_t)PTR_ERR(fp));
        BUG_ON(1);
        return -EBADF;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    /* Copy data. */
    vfs_read(fp, dst, size, &pos);
    /* Close resource. */
    set_fs(fs);
    filp_close(fp, NULL);

    return 0;
}

static int _MDrv_R2_Copy_Local_To_Dram(char *src, U64 dst, int size)
{
    int align = 0x1000;
    phys_addr_t *va = NULL;

    /* Check para. */
    if (dst % align)
    {
        MDRV_R2_DEBUG("Invalid align=0x%tX : dst=0x%tX.\n", (size_t)align, (size_t)dst);
        return -EINVAL;
    }
    /* Get source. */
    if (pfn_valid(__phys_to_pfn(dst)))
        va = dma_common_contiguous_remap(pfn_to_page(__phys_to_pfn(dst)),
                                        size, VM_USERMAP, MDRV_R2_MAP_PROT, __func__);
    else
        va = ioremap_nocache(dst, size);
    if (va == NULL)
    {
        MDRV_R2_DEBUG("Mapping(%td) fail: ba=0x%llX sz=0x%tX.\n",
                    (size_t)(pfn_valid(__phys_to_pfn(dst))), dst, (size_t)size);
        return -EIO;
    }

    /* Copy data. */
    memcpy(va, (void *)src, size);

    /* Close resource. */
    if (pfn_valid(__phys_to_pfn(dst)))
        dma_common_free_remap(va, size, VM_USERMAP);
    else
        iounmap(va);

    return 0;
}

static int _MDrv_R2_Debug(int lv)
{
    phys_addr_t *va = 0;
    int ret = 0;

    if (_FRC_R2_Debug == 0)
        return 0;

    /* Get DRAM VA.*/
    if (pfn_valid(__phys_to_pfn(_stFRC_R2.u64Address)))
        va = dma_common_contiguous_remap(pfn_to_page(__phys_to_pfn(_stFRC_R2.u64Address)),
                                        MDRV_R2_CODE_SZ, VM_USERMAP, MDRV_R2_MAP_PROT, __func__);
    else
        va = ioremap_nocache(_stFRC_R2.u64Address, MDRV_R2_CODE_SZ);
    if (va == NULL)
    {
        MDRV_R2_DEBUG("Mapping(%td) fail: ba=0x%llX sz=0x%tX.\n",
                    (size_t)(pfn_valid(__phys_to_pfn(_stFRC_R2.u64Address))),
                    _stFRC_R2.u64Address, (size_t)MDRV_R2_CODE_SZ);
        return 0;
    }

    /* Debug case. */
    switch (lv)
    {
        case 1:
            if (_FRC_R2_Debug >= 1)
            {
                MDRV_R2_DEBUG("memcmp STATIC with DRAM = %d.\n", memcmp(_pFRC_R2_BIN, va,MDRV_R2_CODE_SZ));
                _MDrv_R2_Cal_MD5("STATIC",  (unsigned char *)_pFRC_R2_BIN,    MDRV_R2_CODE_SZ);
                _MDrv_R2_Cal_MD5("DRAM  ",  (unsigned char *)va,              MDRV_R2_CODE_SZ);
            }
            break;
        case 2:
            if (_FRC_R2_Debug >= 2)
            {
                MDRV_R2_DEBUG("overwrite DRAM from STATIC.\n");
                memcpy(va, _pFRC_R2_BIN, MDRV_R2_CODE_SZ);
            }
            break;
        case 3:
            if (_FRC_R2_Debug >= 3)
            {
                MDRV_R2_DEBUG("remove one-times copy from emmc.\n");
                memset(_stFRC_R2.pu8Path, '\0', MDRV_R2_BUFF_SZ);
            }
            break;
        case 4:
            if (_FRC_R2_Debug >= 4)
            {
                ret = 1;
            }
            break;
    }

    /* Put DRAM VA.*/
    if (pfn_valid(__phys_to_pfn(_stFRC_R2.u64Address)))
        dma_common_free_remap(va, MDRV_R2_CODE_SZ, VM_USERMAP);
    else
        iounmap(va);

    return ret;
}

//-------------------------------------------------------------------------------------------------
//  Golbal function
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_MSTAR_FRC_R2)
S32 MDrv_FRC_R2_Suspend(MDRV_R2_STATE eState)
{
    S32 s32Ret = 0;

    switch (eState)
    {
        case E_R2_STATE_SUSPEND_PRE:
            if (_stFRC_R2.u8Enable == 1)
            {
                _MDrv_R2_Debug(3);
                /* One time. */
                if (strlen(_stFRC_R2.pu8Path) != 0)
                    break;

                if (((s32Ret = _MDrv_R2_Get_Path(_FRC_Part, _stFRC_R2.pu8Path)) != 0) ||
                    ((s32Ret = _MDrv_R2_Copy_Part_To_Local(_stFRC_R2.pu8Path, _pFRC_R2_BIN, MDRV_R2_CODE_SZ)) != 0))
                    MDRV_R2_DEBUG("Copy FRC-R2.bin to local buffer fail = %d.\n", s32Ret);
                _MDrv_R2_Debug(1);
            }
            break;

        case E_R2_STATE_SUSPEND_NOIRQ:
            if (_stFRC_R2.u8Enable == 1)
            {
                MHal_FRC_R2_Disable();
                s32Ret = _MDrv_R2_Copy_Local_To_Dram(_pFRC_R2_BIN, _stFRC_R2.u64Address, MDRV_R2_CODE_SZ);
                _FRC_R2_Live = 0;
                _MDrv_R2_Debug(1);
                _MDrv_R2_Debug(2);
                _MDrv_R2_Debug(1);
            }
            break;

        default:
            MDRV_R2_DEBUG("Invalid argument.\n");
            s32Ret = -EINVAL;
            break;
    }

    return s32Ret;
}

S32 MDrv_FRC_R2_Resume(MDRV_R2_STATE eState)
{
    S32 s32Ret = 0;

    switch (eState)
    {
        case E_R2_STATE_RESUME_NOIRQ:
            if (_stFRC_R2.u8Enable == 1)
            {
                if (_FRC_R2_Live == 1)
                    break;

                _MDrv_R2_Debug(1);
                _MDrv_R2_Debug(2);
                _MDrv_R2_Debug(1);
                while (_MDrv_R2_Debug(4))
                    _MDrv_R2_Debug(1);
                MHal_FRC_R2_Enable(_MDrv_R2_Ba2Pa(_stFRC_R2.u64Address));
                MDRV_R2_DEBUG("Enable at PA=0x%llX done.\n", _MDrv_R2_Ba2Pa(_stFRC_R2.u64Address));
                _FRC_R2_Live = 1;
            }
            break;

        default:
            MDRV_R2_DEBUG("Invalid argument.\n");
            s32Ret = -EINVAL;
            break;
    }

    return s32Ret;
}
#endif

//-------------------------------------------------------------------------------------------------
//  Early
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_MSTAR_FRC_R2)
static int __init _MDrv_R2_Set_FRC_Info(char *str)
{
    if (str != NULL)
    {
        sscanf(str, "%hhu, %llx", &(_stFRC_R2.u8Enable), &(_stFRC_R2.u64Address));
    }
    return 0;
}
static int __init _MDrv_R2_Set_FRC_Debug(char *str)
{
    if (str != NULL)
    {
        sscanf(str, "%u", &(_FRC_R2_Debug));
    }
    return 0;
}
static int __init _MDrv_R2_Set_Slot_Info(char *str)
{
    if ( str != NULL)
    {
        snprintf(_FRC_Part, 10, "frc%s", str);
    }
    return 0;
}
early_param("FRC_R2_INFO", _MDrv_R2_Set_FRC_Info);
early_param("FRC_R2_DEBUG", _MDrv_R2_Set_FRC_Debug);
early_param("androidboot.slot_suffix", _MDrv_R2_Set_Slot_Info);
#endif
