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
/// file    mdrv_pnl_cust.c
/// @brief  Panel Customerized Driver Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/printk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <generated/autoconf.h>
#include "MsTypes.h"
#include "mdrv_iic_io.h"

#ifdef CONFIG_MP_PNL_STR_CUST
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/err.h>

//drver header files
#include "mdrv_gpio.h"
#include "mhal_gpio_reg.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define HIBYTE(value)  ((MS_U8)((value) / 0x100))
#define LOBYTE(value)  ((MS_U8)(value))

#define PNL_CUST_STR_PRINT(fmt, args...)    printk("[PNL_CUST_STR][%05d] " fmt, __LINE__, ## args)

#define PNL_CUST_I2C_BUSID_DTS_DEFAULT  (2) // SWI2C Bus scl-gpio/sda-gpio should be configed in dts.

#define PNL_CUST_BIN_SIZE_MAX   (1024)   // Bin size max. forllow Mboot BUFFER_SIZE define.


//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------

typedef enum
{
    E_PNL_CUST_IC_NONE = 0,                     ///< Default
    E_PNL_CUST_IC_PMIC,                         ///< PMIC
    E_PNL_CUST_IC_PGAMMAIC,                     ///< Pgamma IC
    E_PNL_CUST_IC_GENERAL,                      ///< General external IC
    E_PNL_CUST_IC_MAX,
}EN_PNL_CUST_IC_TYPE;

/// Define Panel customized IC info
// PNL_VCC on -> VCC delay(ms) -> Pre GPIO Operation -> Pre delay(ms) -> IC write (from bin) -> Post GPIO Operation -> Post delay(ms)
#define PNL_CUST_BIN_FILE_PATH_LENGTH (128)  // Bin file path length.
typedef struct __attribute__((__packed__))
{
    ///< General
    MS_U32 u32PnlCustICInfo_Version;        ///< Structure version
    MS_U32 u32PnlCustICInfo_Length;         ///< Structure length
    EN_PNL_CUST_IC_TYPE  ePnlCustIcType;    ///< External customized IC type.
    char    pm_PNL_CUST_IC_FILE_PATH[PNL_CUST_BIN_FILE_PATH_LENGTH];    ///< bin file path
    MS_U8   u8BinFormatType;                ///< bin file format
    MS_BOOL bForceInit;                     ///< Force Power On int.
    ///< I2C Info
    MS_U16  u16I2C_BUS;                     ///< I2C Bus ID
    MS_U16  u16I2C_MODE;                    ///< I2C Bus Mode: 0:swi2c; 1:hwi2c
    MS_U16  u16I2C_DEV_ADDR;
    MS_U16  u16I2C_REG_OFFSET;
    ///< Pre GPIO Operation
    MS_U16  u16GPIO_PRE_NUM;
    MS_U8   u8GPIO_PRE_OPERATION;
    ///< Post GPIO Operation
    MS_U16  u16GPIO_POST_NUM;
    MS_U8   u8GPIO_POST_OPERATION;
    ///< VCC delay
    MS_U16  u16DELAY_VCC_TIME;
    ///< Pre delay
    MS_U16  u16DELAY_PRE_TIME;
    ///< Post delay
    MS_U16  u16DELAY_POST_TIME;
} ST_PNL_CUST_IC_INFO;

typedef struct
{
    MS_U8*  pu8BinData;
    MS_U16  u16Size;
} ST_PNL_CUST_BIN_DATA;

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static ST_PNL_CUST_IC_INFO _stPnlCustPMICInfo      = {0};
static ST_PNL_CUST_IC_INFO _stPnlCustPgammaICInfo  = {0};

static ST_PNL_CUST_BIN_DATA _stPnlCustPMICBin       = {NULL, 0};
static ST_PNL_CUST_BIN_DATA _stPnlCustPgammaICBin   = {NULL, 0};

#define AUTOPAGMMA_OFFSET_HEADER    0x2FF000
#define AUTOPAGMMA_OFFSET_DATA      0x2FF020
#define AUTOPAGMMA_RD_DATA_LEN      256
#define DEVINFO_PATH_LEN            128

//#define DEVINFO_PATH                ("/dev/mmcblk0p26")    //demura raw data partition
static char                 gAutoPgamma_Path[DEVINFO_PATH_LEN] = {0};

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static _PNL_CUST_Get_AutoPgamma_Path(char *path)
{
    int ret = 0;
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;
    int blk = 1;
    char buff[DEVINFO_PATH_LEN] = {0};

    /* Already init. */
    if (strlen(path) != 0)
    {
        fp = filp_open(path, O_RDONLY, 0);
        if (!IS_ERR(fp))
        {
            filp_close(fp, NULL);
            return 1;
        }
    }

    /* Find /dev/block/platform/mstar_mci.0/by-name/demura exist. */
    snprintf(buff, DEVINFO_PATH_LEN, "/dev/block/platform/mstar_mci.0/by-name/demura");
    fp = filp_open(buff, O_RDONLY, 0);
    if ((!IS_ERR(fp)) || (PTR_ERR(fp) == -EACCES))
    {
        strncpy(path, buff, DEVINFO_PATH_LEN);
        if (!IS_ERR(fp))
            filp_close(fp, NULL);
        return 1;
    }

    /* Store fs. */
    fs = get_fs();
    set_fs(KERNEL_DS);

    /* Find /dev/mmcblk0p#. */
    do
    {
        memset(buff, '\0', DEVINFO_PATH_LEN);
        snprintf(buff, DEVINFO_PATH_LEN, "/sys/block/mmcblk0/mmcblk0p%d/uevent", blk);
        fp = filp_open(buff, O_RDONLY, 0);
        if (IS_ERR(fp))
        {
            PNL_CUST_STR_PRINT("demura path not found.\n");
            ret = 0;
            goto _PNL_CUST_Get_AutoPgamma_Path;
        }
        memset(buff, '\0', DEVINFO_PATH_LEN);
        vfs_read(fp, buff, DEVINFO_PATH_LEN, &pos);
        filp_close(fp, NULL);

        /* Check found. */
        if (strnstr(buff, "PARTNAME=demura", DEVINFO_PATH_LEN))
        {
            memset(buff, '\0', DEVINFO_PATH_LEN);
            snprintf(buff, DEVINFO_PATH_LEN, "/dev/mmcblk0p%d", blk);
            fp = filp_open(buff, O_RDONLY, 0);
            if (!IS_ERR(fp))
            {
                snprintf(path, DEVINFO_PATH_LEN, "/dev/mmcblk0p%d", blk);
                filp_close(fp, NULL);
            }
            else
            {
                snprintf(path, DEVINFO_PATH_LEN, "/dev/mmcblk%d", blk);
            }
            break;
        }
        else
        {
            pos = 0;
            blk++;
        }
    } while (1);

_PNL_CUST_Get_AutoPgamma_Path:
    /* Restore fs. */
    set_fs(fs);

    return ret;
}


static int _PNL_CUST_Get_AutoPgamma_data(ST_PNL_CUST_BIN_DATA *stPnlCustAutoPgamma)
{
    size_t size;
    loff_t offset;
    char buff_header[AUTOPAGMMA_RD_DATA_LEN] = {0};
    //char buff_data[AUTOPAGMMA_RD_DATA_LEN] = {0};
    int i = 0, x = 0, y = 0;
    mm_segment_t cur_mm_seg;

    MS_U32  u32TempSum1 = 0, u32TempSum2 = 0;
    MS_U8  u8IsDone = 0;
    MS_U8  u8DataLen = 0;

    _PNL_CUST_Get_AutoPgamma_Path(gAutoPgamma_Path);
    PNL_CUST_STR_PRINT(" gAutoPgamma_Path=%s \n", gAutoPgamma_Path);
    struct file *fd = filp_open(gAutoPgamma_Path, O_RDONLY, 0);
    if(!IS_ERR(fd))
    {
        offset = AUTOPAGMMA_OFFSET_HEADER;
        cur_mm_seg = get_fs();
        set_fs(KERNEL_DS);
        size = vfs_read(fd, buff_header, (AUTOPAGMMA_OFFSET_DATA - AUTOPAGMMA_OFFSET_HEADER), &offset);
        set_fs(cur_mm_seg);
        u8IsDone = buff_header[0];
        u8DataLen = buff_header[1];
        PNL_CUST_STR_PRINT(" u8IsDone=0x%x u8DataLen=%d \n", u8IsDone, u8DataLen);
        if((u8IsDone == 0x55) && (u8DataLen > 0))
        {
            offset = AUTOPAGMMA_OFFSET_DATA;
            // Release the previous resource first.
            if(stPnlCustAutoPgamma->pu8BinData != NULL)
            {
                vfree(stPnlCustAutoPgamma->pu8BinData);
                stPnlCustAutoPgamma->pu8BinData = NULL;
                stPnlCustAutoPgamma->u16Size = 0;
            }
            stPnlCustAutoPgamma->pu8BinData = (MS_U8*)vmalloc(u8DataLen);
            if(stPnlCustAutoPgamma->pu8BinData == NULL)
            {
                PNL_CUST_STR_PRINT("out of memory \n");
                stPnlCustAutoPgamma->u16Size = 0;
                filp_close(fd, NULL);
                return -1;
            }
            cur_mm_seg = get_fs();
            set_fs(KERNEL_DS);
            size = vfs_read(fd, stPnlCustAutoPgamma->pu8BinData, u8DataLen, &offset);
            set_fs(cur_mm_seg);
            stPnlCustAutoPgamma->u16Size = u8DataLen;

            PNL_CUST_STR_PRINT(" read part:%s sieze:%ld bytes\n", gAutoPgamma_Path, stPnlCustAutoPgamma->u16Size);
            vfs_fsync(fd, 0);
            filp_close(fd, NULL);
            return 1;
        }
        else
            PNL_CUST_STR_PRINT("Not support AutoPgamma!!!\n");
        return 0;
    }
    else
    {
        PNL_CUST_STR_PRINT("open deviceinfo failed!!!\n");
        return 0;
    }
    return 0;
}

static int _PNL_CUST_PMIC_ReadFile(void *data)
{
    MS_U32  u32Rst = 0;
    long lFileSize = 0;
    struct file *pFile = NULL;
    mm_segment_t cur_mm_seg;
    loff_t pos;

    if(_stPnlCustPMICInfo.u8BinFormatType >= 2)
    {
        _PNL_CUST_Get_AutoPgamma_data(&_stPnlCustPMICBin);
        return 0;
    }

    if(_stPnlCustPMICInfo.pm_PNL_CUST_IC_FILE_PATH==NULL)
    {
        PNL_CUST_STR_PRINT("PMIC Read file fail:parameter error!\n");
        return -1;
    }

    pFile = filp_open(_stPnlCustPMICInfo.pm_PNL_CUST_IC_FILE_PATH, O_RDONLY, 0);
    if(IS_ERR(pFile))
    {
        PNL_CUST_STR_PRINT("%s open fail\n",_stPnlCustPMICInfo.pm_PNL_CUST_IC_FILE_PATH);
        return -1;
    }
    vfs_llseek(pFile, 0, SEEK_END);
    lFileSize = pFile->f_pos;
    PNL_CUST_STR_PRINT("PMIC bin size = %ld\n", lFileSize);

    if((lFileSize <= 0) || (lFileSize > PNL_CUST_BIN_SIZE_MAX))
    {
        PNL_CUST_STR_PRINT("PMIC bin size invalid\n");
        filp_close(pFile, NULL);
        return -1;
    }
    pos = vfs_llseek(pFile, 0, SEEK_SET);

    // Release the previous resource first.
    if(_stPnlCustPMICBin.pu8BinData != NULL)
    {
        vfree(_stPnlCustPMICBin.pu8BinData);
        _stPnlCustPMICBin.pu8BinData = NULL;
        _stPnlCustPMICBin.u16Size = 0;
    }

    _stPnlCustPMICBin.pu8BinData = (MS_U8*)vmalloc(lFileSize);
    if(_stPnlCustPMICBin.pu8BinData == NULL)
    {
        PNL_CUST_STR_PRINT("PMIC bin out of memory \n");
        _stPnlCustPMICBin.u16Size= 0;
        filp_close(pFile, NULL);
        return -1;
    }
    cur_mm_seg = get_fs();
    set_fs(KERNEL_DS);
    u32Rst = vfs_read(pFile, _stPnlCustPMICBin.pu8BinData, lFileSize, &pos);
    set_fs(cur_mm_seg);

    if(u32Rst != (MS_U32)lFileSize)
    {
        PNL_CUST_STR_PRINT("PMIC bin Wrong %td %ld \n", (ptrdiff_t)u32Rst, lFileSize);
        if(_stPnlCustPMICBin.pu8BinData != NULL)
        {
            vfree(_stPnlCustPMICBin.pu8BinData);
            _stPnlCustPMICBin.pu8BinData = NULL;
            _stPnlCustPMICBin.u16Size = 0;
        }
        filp_close(pFile, NULL);
        return -1;
    }

    _stPnlCustPMICBin.u16Size = (MS_U16)lFileSize;
    filp_close(pFile, NULL);

    return 0;
}

static int _PNL_CUST_PGAMMAIC_ReadFile(void *data)
{
    MS_U32  u32Rst = 0;
    long lFileSize = 0;
    struct file *pFile = NULL;
    mm_segment_t cur_mm_seg;
    loff_t pos;

    if(_stPnlCustPgammaICInfo.u8BinFormatType >= 2)
    {
        _PNL_CUST_Get_AutoPgamma_data(&_stPnlCustPgammaICBin);
        return 0;
    }

    if(_stPnlCustPgammaICInfo.pm_PNL_CUST_IC_FILE_PATH==NULL)
    {
        PNL_CUST_STR_PRINT("PGammaIC Read file fail:parameter error!\n");
        return -1;
    }

    pFile = filp_open(_stPnlCustPgammaICInfo.pm_PNL_CUST_IC_FILE_PATH, O_RDONLY, 0);
    if(IS_ERR(pFile))
    {
        PNL_CUST_STR_PRINT("%s open fail\n",_stPnlCustPgammaICInfo.pm_PNL_CUST_IC_FILE_PATH);
        return -1;
    }
    vfs_llseek(pFile, 0, SEEK_END);
    lFileSize = pFile->f_pos;
    PNL_CUST_STR_PRINT("PGammaIC bin size = %ld\n", lFileSize);

    if((lFileSize <= 0) || (lFileSize > PNL_CUST_BIN_SIZE_MAX))
    {
        PNL_CUST_STR_PRINT("PGammaIC bin size invalid\n");
        filp_close(pFile, NULL);
        return -1;
    }
    pos = vfs_llseek(pFile, 0, SEEK_SET);

    // Release the previous resource first.
    if(_stPnlCustPgammaICBin.pu8BinData != NULL)
    {
        vfree(_stPnlCustPgammaICBin.pu8BinData);
        _stPnlCustPgammaICBin.pu8BinData = NULL;
        _stPnlCustPgammaICBin.u16Size = 0;
    }

    _stPnlCustPgammaICBin.pu8BinData = (MS_U8*)vmalloc(lFileSize);
    if(_stPnlCustPgammaICBin.pu8BinData == NULL)
    {
        PNL_CUST_STR_PRINT("PGammaIC bin out of memory \n");
        _stPnlCustPgammaICBin.u16Size= 0;
        filp_close(pFile, NULL);
        return -1;
    }
    cur_mm_seg = get_fs();
    set_fs(KERNEL_DS);
    u32Rst = vfs_read(pFile, _stPnlCustPgammaICBin.pu8BinData, lFileSize, &pos);
    set_fs(cur_mm_seg);

    if(u32Rst != (MS_U32)lFileSize)
    {
        PNL_CUST_STR_PRINT("PGammaIC bin Wrong %td %ld \n", (ptrdiff_t)u32Rst, lFileSize);
        if(_stPnlCustPgammaICBin.pu8BinData != NULL)
        {
            vfree(_stPnlCustPgammaICBin.pu8BinData);
            _stPnlCustPgammaICBin.pu8BinData = NULL;
            _stPnlCustPgammaICBin.u16Size = 0;
        }
        filp_close(pFile, NULL);
        return -1;
    }

    _stPnlCustPgammaICBin.u16Size = (MS_U16)lFileSize;
    filp_close(pFile, NULL);

    return 0;
}

static MS_BOOL _PNL_Cust_Initial_Setting_PMIC_Init(ST_PNL_CUST_IC_INFO *pstPnlCustIC_Info)
{
    MS_BOOL bRet = FALSE;
    struct task_struct *pstTask = NULL;
    ST_PNL_CUST_IC_INFO *pstPMIC_Info = pstPnlCustIC_Info;

    if(pstPMIC_Info == NULL)
    {
        PNL_CUST_STR_PRINT("PMIC_BIN: Invalid parameters!!\n");
        return bRet;
    }

    memset(&_stPnlCustPMICInfo, 0, (sizeof(ST_PNL_CUST_IC_INFO)));
    memcpy(&_stPnlCustPMICInfo, pstPMIC_Info, (sizeof(ST_PNL_CUST_IC_INFO)));

    if(strlen(_stPnlCustPMICInfo.pm_PNL_CUST_IC_FILE_PATH) == 0)
    {
        PNL_CUST_STR_PRINT("PMIC_BIN file lens is 0\n");
        return bRet;
    }

    PNL_CUST_STR_PRINT("PMIC_BIN bin path=%s\n", _stPnlCustPMICInfo.pm_PNL_CUST_IC_FILE_PATH);
    PNL_CUST_STR_PRINT("PMIC_BIN u8BinFormatType=%d\n",_stPnlCustPMICInfo.u8BinFormatType);
    PNL_CUST_STR_PRINT("PMIC_BIN bForceInit=%d\n",_stPnlCustPMICInfo.bForceInit);
    PNL_CUST_STR_PRINT("PMIC_BIN I2C bus=%d\n",_stPnlCustPMICInfo.u16I2C_BUS);
    PNL_CUST_STR_PRINT("PMIC_BIN I2C mode=%d\n",_stPnlCustPMICInfo.u16I2C_MODE);
    PNL_CUST_STR_PRINT("PMIC_BIN I2C dev=0x%x\n",_stPnlCustPMICInfo.u16I2C_DEV_ADDR);
    PNL_CUST_STR_PRINT("PMIC_BIN I2C offset=%d\n",_stPnlCustPMICInfo.u16I2C_REG_OFFSET);
    PNL_CUST_STR_PRINT("PMIC_BIN Delay_VCC_TIME=%d\n",_stPnlCustPMICInfo.u16DELAY_VCC_TIME);
    PNL_CUST_STR_PRINT("PMIC_BIN GPIO_PRE_NUM=%d\n",_stPnlCustPMICInfo.u16GPIO_PRE_NUM);
    PNL_CUST_STR_PRINT("PMIC_BIN Delay_PRE_TIME=%d\n",_stPnlCustPMICInfo.u16DELAY_PRE_TIME);
    PNL_CUST_STR_PRINT("PMIC_BIN GPIO_POST_NUM=%d\n",_stPnlCustPMICInfo.u16GPIO_POST_NUM);
    PNL_CUST_STR_PRINT("PMIC_BIN Delay_POST_TIME=%d\n",_stPnlCustPMICInfo.u16DELAY_POST_TIME);

    if(_stPnlCustPMICInfo.bForceInit == FALSE)
    {
        PNL_CUST_STR_PRINT("PMIC_BIN is not necessary\n");
        return bRet;
    }


    pstTask = kthread_create(_PNL_CUST_PMIC_ReadFile, NULL, "_PNL_CUST_PMIC_ReadFile");
    if(IS_ERR(pstTask))
    {
        PNL_CUST_STR_PRINT("PMIC_BIN thread create fail\n");
        return bRet;
    }
    wake_up_process(pstTask);

    bRet = TRUE;
    return bRet;
}

static MS_BOOL _PNL_Cust_Initial_Setting_PGAMMAIC_Init(ST_PNL_CUST_IC_INFO *pstPnlCustIC_Info)
{
    MS_BOOL bRet = FALSE;
    struct task_struct *pstTask = NULL;
    ST_PNL_CUST_IC_INFO *pstPgammaIC_Info = pstPnlCustIC_Info;

    if(pstPgammaIC_Info == NULL)
    {
        PNL_CUST_STR_PRINT("PGAMMA_IC_BIN: Invalid parameters!!\n");
        return bRet;
    }

    memset(&_stPnlCustPgammaICInfo, 0, (sizeof(ST_PNL_CUST_IC_INFO)));
    memcpy(&_stPnlCustPgammaICInfo, pstPgammaIC_Info, (sizeof(ST_PNL_CUST_IC_INFO)));

    if(strlen(_stPnlCustPgammaICInfo.pm_PNL_CUST_IC_FILE_PATH) == 0)
    {
        PNL_CUST_STR_PRINT("PGAMMA_BIN file lens is 0\n");
        return bRet;
    }

    PNL_CUST_STR_PRINT("PGAMMA_BIN bin path=%s\n", _stPnlCustPgammaICInfo.pm_PNL_CUST_IC_FILE_PATH);
    PNL_CUST_STR_PRINT("PGAMMA_BIN u8BinFormatType=%d\n",_stPnlCustPgammaICInfo.u8BinFormatType);
    PNL_CUST_STR_PRINT("PGAMMA_BIN bForceInit=%d\n",_stPnlCustPgammaICInfo.bForceInit);
    PNL_CUST_STR_PRINT("PGAMMA_BIN I2C bus=%d\n",_stPnlCustPgammaICInfo.u16I2C_BUS);
    PNL_CUST_STR_PRINT("PGAMMA_BIN I2C mode=%d\n",_stPnlCustPgammaICInfo.u16I2C_MODE);
    PNL_CUST_STR_PRINT("PGAMMA_BIN I2C dev=0x%x\n",_stPnlCustPgammaICInfo.u16I2C_DEV_ADDR);
    PNL_CUST_STR_PRINT("PGAMMA_BIN I2C offset=%d\n",_stPnlCustPgammaICInfo.u16I2C_REG_OFFSET);
    PNL_CUST_STR_PRINT("PGAMMA_BIN Delay_VCC_TIME=%d\n",_stPnlCustPgammaICInfo.u16DELAY_VCC_TIME);
    PNL_CUST_STR_PRINT("PGAMMA_BIN GPIO_PRE_NUM=%d\n",_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM);
    PNL_CUST_STR_PRINT("PGAMMA_BIN Delay_PRE_TIME=%d\n",_stPnlCustPgammaICInfo.u16DELAY_PRE_TIME);
    PNL_CUST_STR_PRINT("PGAMMA_BIN GPIO_POST_NUM=%d\n",_stPnlCustPgammaICInfo.u16GPIO_POST_NUM);
    PNL_CUST_STR_PRINT("PGAMMA_BIN Delay_POST_TIME=%d\n",_stPnlCustPgammaICInfo.u16DELAY_POST_TIME);

    if(_stPnlCustPgammaICInfo.bForceInit == FALSE)
    {
        PNL_CUST_STR_PRINT("PGAMMA_IC_BIN is not necessary\n");
        return bRet;
    }

    pstTask = kthread_create(_PNL_CUST_PGAMMAIC_ReadFile, NULL, "_PNL_CUST_PGAMMAIC_ReadFile");
    if(IS_ERR(pstTask))
    {
        PNL_CUST_STR_PRINT("PGAMMA_IC_BIN thread create fail\n");
        return bRet;
    }
    wake_up_process(pstTask);

    bRet = TRUE;
    return bRet;
}

static MS_BOOL _PNL_Cust_Resume_Setting_PMIC_PowerOn(void)
{
    MS_BOOL bRet = FALSE;
    MS_S32  s32Ret = -1;
    MS_U8*  pu8PNLCustData = NULL;
    MS_U8   u8BusID = PNL_CUST_I2C_BUSID_DTS_DEFAULT;
    MS_U8   u8SlaveID = 0;
    MS_U8   u8Offset = 0;
    MS_U16  u16Size = 0;

    if((_stPnlCustPMICBin.pu8BinData == NULL) || (_stPnlCustPMICBin.u16Size == 0))
    {
        if(_stPnlCustPMICInfo.u8BinFormatType < 2)
        {
            PNL_CUST_STR_PRINT("PMIC bin invalid\n");
            return bRet;
        }
    }

    /******************************************************************************************************/
    // PNL_VCC on -> VCC delay(ms) -> Pre GPIO Operation -> Pre delay(ms) -> IC write (from bin) -> Post GPIO Operation -> Post delay(ms)
    /******************************************************************************************************/

    /*** VCC delay(ms) ***/
    if(_stPnlCustPMICInfo.u16DELAY_VCC_TIME > 0)
    {
        msleep(_stPnlCustPMICInfo.u16DELAY_VCC_TIME);
    }

    /*** Pre GPIO Operation ***/
    if(_stPnlCustPMICInfo.u16GPIO_PRE_NUM > 0)
    {
        switch(_stPnlCustPMICInfo.u8GPIO_PRE_OPERATION)
        {
            case 0:
            {
                MDrv_GPIO_Set_Low(_stPnlCustPMICInfo.u16GPIO_PRE_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput Low\n",_stPnlCustPMICInfo.u16GPIO_PRE_NUM);
                break;
            }
            case 1:
            {
                MDrv_GPIO_Set_High(_stPnlCustPMICInfo.u16GPIO_PRE_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput High\n",_stPnlCustPMICInfo.u16GPIO_PRE_NUM);
                break;
            }
            case 0xFF:
            default:
            {
                MDrv_GPIO_Set_Input(_stPnlCustPMICInfo.u16GPIO_PRE_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Input Hi-Z\n",_stPnlCustPMICInfo.u16GPIO_PRE_NUM);
                break;
            }
        }
    }

    /*** Pre delay(ms) ***/
    if(_stPnlCustPMICInfo.u16DELAY_PRE_TIME > 0)
    {
        msleep(_stPnlCustPMICInfo.u16DELAY_PRE_TIME);
    } 
    /*** IC write (from bin) ***/
    if(_stPnlCustPMICInfo.u8BinFormatType >= 2)
    {
        PNL_CUST_STR_PRINT("PMIC write (from SPI flash data)\n ");
    }
    else
    {
        PNL_CUST_STR_PRINT("PMIC write (from bin)\n ");
    }
    pu8PNLCustData  = _stPnlCustPMICBin.pu8BinData;
    u16Size         = _stPnlCustPMICBin.u16Size;
    u8BusID     = (MS_U8)_stPnlCustPMICInfo.u16I2C_BUS;
    u8SlaveID   = (MS_U8)_stPnlCustPMICInfo.u16I2C_DEV_ADDR;
    u8Offset    = (MS_U8)_stPnlCustPMICInfo.u16I2C_REG_OFFSET;

    PNL_CUST_STR_PRINT("PMIC bin I2C write MultiByte BusID %d, SlaveID 0x%02x, Offset=%d, Size=%d\n ",u8BusID,u8SlaveID,u8Offset,u16Size);

    if(_stPnlCustPMICInfo.u16I2C_MODE == 1)  // hw i2c
    {
        s32Ret = MDrv_HW_IIC_WriteBytes(u8BusID, u8SlaveID, 1, &u8Offset, (MS_U32)u16Size, pu8PNLCustData);
        if(s32Ret < 0)
        {
            PNL_CUST_STR_PRINT("PMIC bin HWI2C write fail. s32Ret=%d\n",s32Ret);
        }
    }
    else    // default sw i2c
    {
        s32Ret = MDrv_SW_IIC_WriteBytes(u8BusID, u8SlaveID, 1, &u8Offset, u16Size, pu8PNLCustData);
        if(s32Ret < 0)
        {
            PNL_CUST_STR_PRINT("PMIC bin SWI2C write fail. s32Ret=%d\n",s32Ret);
        }
    }

    /*** Post GPIO Operation ***/
    if(_stPnlCustPMICInfo.u16GPIO_POST_NUM > 0)
    {
        switch(_stPnlCustPMICInfo.u8GPIO_POST_OPERATION)
        {
            case 0:
            {
                MDrv_GPIO_Set_Low(_stPnlCustPMICInfo.u16GPIO_POST_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput Low\n",_stPnlCustPMICInfo.u16GPIO_POST_NUM);
                break;
            }
            case 1:
            {
                MDrv_GPIO_Set_High(_stPnlCustPMICInfo.u16GPIO_POST_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput High\n",_stPnlCustPMICInfo.u16GPIO_POST_NUM);
                break;
            }
            case 0xFF:
            default:
            {
                MDrv_GPIO_Set_Input(_stPnlCustPMICInfo.u16GPIO_POST_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Input Hi-Z\n",_stPnlCustPMICInfo.u16GPIO_POST_NUM);
                break;
            }
        }
    }

    /*** Post delay(ms) ***/
    if(_stPnlCustPMICInfo.u16DELAY_POST_TIME > 0)
    {
        msleep(_stPnlCustPMICInfo.u16DELAY_POST_TIME);
    }

    bRet = TRUE;
    return bRet;
}

static MS_BOOL _PNL_Cust_Resume_Setting_PGAMMAIC_PowerOn(void)
{
    MS_BOOL bRet = FALSE;
    MS_S32  s32Ret = -1;
    MS_U8*  pu8PNLCustData = NULL;
    MS_U8   u8BusID = PNL_CUST_I2C_BUSID_DTS_DEFAULT;
    MS_U8   u8SlaveID = 0;
    MS_U8   u8Offset = 0;
    MS_U16  u16Size = 0;

    if((_stPnlCustPgammaICBin.pu8BinData == NULL) || (_stPnlCustPgammaICBin.u16Size == 0))
    {
        if(_stPnlCustPgammaICInfo.u8BinFormatType < 2)
        {
            PNL_CUST_STR_PRINT("PgammaIC bin invalid\n");
            return bRet;
        }
    }

    /******************************************************************************************************/
    // PNL_VCC on -> VCC delay(ms) -> Pre GPIO Operation -> Pre delay(ms) -> IC write (from bin) -> Post GPIO Operation -> Post delay(ms)
    /******************************************************************************************************/

    /*** VCC delay(ms) ***/
    if(_stPnlCustPgammaICInfo.u16DELAY_VCC_TIME > 0)
    {
        msleep(_stPnlCustPgammaICInfo.u16DELAY_VCC_TIME);
    }

    /*** Pre GPIO Operation ***/
    if(_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM > 0)
    {
        switch(_stPnlCustPgammaICInfo.u8GPIO_PRE_OPERATION)
        {
            case 0:
            {
                MDrv_GPIO_Set_Low(_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput Low\n",_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM);
                break;
            }
            case 1:
            {
                MDrv_GPIO_Set_High(_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput High\n",_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM);
                break;
            }
            case 0xFF:
            default:
            {
                MDrv_GPIO_Set_Input(_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Input Hi-Z\n",_stPnlCustPgammaICInfo.u16GPIO_PRE_NUM);
                break;
            }
        }
    }

    /*** Pre delay(ms) ***/
    if(_stPnlCustPgammaICInfo.u16DELAY_PRE_TIME > 0)
    {
        msleep(_stPnlCustPgammaICInfo.u16DELAY_PRE_TIME);
    }

    if(_stPnlCustPgammaICInfo.u8BinFormatType >= 2)
    {
        PNL_CUST_STR_PRINT("pgamma write (from SPI flash data)\n ");
    }
    else
    {
        PNL_CUST_STR_PRINT("pgamma write (from bin)\n ");
    }

    pu8PNLCustData  = _stPnlCustPgammaICBin.pu8BinData;
    u16Size         = _stPnlCustPgammaICBin.u16Size;
    u8BusID     = (MS_U8)_stPnlCustPgammaICInfo.u16I2C_BUS;
    u8SlaveID   = (MS_U8)_stPnlCustPgammaICInfo.u16I2C_DEV_ADDR;
    u8Offset    = (MS_U8)_stPnlCustPgammaICInfo.u16I2C_REG_OFFSET;

    PNL_CUST_STR_PRINT("PgammaIC bin I2C write MultiByte BusID %d, SlaveID 0x%02x, Offset=%d, Size=%d\n ",u8BusID,u8SlaveID,u8Offset,u16Size);

    if(_stPnlCustPgammaICInfo.u16I2C_MODE == 1)  // hw i2c
    {
        s32Ret = MDrv_HW_IIC_WriteBytes(u8BusID, u8SlaveID, 1, &u8Offset, (MS_U32)u16Size, pu8PNLCustData);
        if(s32Ret < 0)
        {
            PNL_CUST_STR_PRINT("PgammaIC bin HWI2C write fail. s32Ret=%d\n",s32Ret);
        }
    }
    else    // default sw i2c
    {
        s32Ret = MDrv_SW_IIC_WriteBytes(u8BusID, u8SlaveID, 1, &u8Offset, u16Size, pu8PNLCustData);
        if(s32Ret < 0)
        {
            PNL_CUST_STR_PRINT("PgammaIC bin SWI2C write fail. s32Ret=%d\n",s32Ret);
        }
    }

    /*** Post GPIO Operation ***/
    if(_stPnlCustPgammaICInfo.u16GPIO_POST_NUM > 0)
    {
        switch(_stPnlCustPgammaICInfo.u8GPIO_POST_OPERATION)
        {
            case 0:
            {
                MDrv_GPIO_Set_Low(_stPnlCustPgammaICInfo.u16GPIO_POST_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput Low\n",_stPnlCustPgammaICInfo.u16GPIO_POST_NUM);
                break;
            }
            case 1:
            {
                MDrv_GPIO_Set_High(_stPnlCustPgammaICInfo.u16GPIO_POST_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Ouput High\n",_stPnlCustPgammaICInfo.u16GPIO_POST_NUM);
                break;
            }
            case 0xFF:
            default:
            {
                MDrv_GPIO_Set_Input(_stPnlCustPgammaICInfo.u16GPIO_POST_NUM - 1);
                PNL_CUST_STR_PRINT("GPIO[%d - 1] Input Hi-Z\n",_stPnlCustPgammaICInfo.u16GPIO_POST_NUM);
                break;
            }
        }
    }

    /*** Post delay(ms) ***/
    if(_stPnlCustPgammaICInfo.u16DELAY_POST_TIME > 0)
    {
        msleep(_stPnlCustPgammaICInfo.u16DELAY_POST_TIME);
    }

    bRet = TRUE;
    return bRet;
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

/*
  1.What is pnl_cust:

  KApi_PNL_Cust_xxx is used for adding customization code at panel str flow.

  we name it by the related position.

  for exampe: KApi_PNL_Cust_Resume_Setting_VCC_onTiming1
  It means the customization function is active between VCC and onTiming1.

  2.others notes:

      2.1 need define CONFIG_MP_PNL_STR_CUST at config (only can define at customer branch now)
      2.2 the caller is at utopia Ex:UTPA2-700.0.x\modules\xc\api\pnl\apiPNL.c
*/


/*
  Normal Panel Suspend flow:

  -----------------------------
  Backlight
  -----------------------------
  offTiming1 delay
  -----------------------------
  Data
  -----------------------------
  offTiming2 delay
  -----------------------------
  VCC
  -----------------------------

  if all customization insert at str suspend like bellow:

  ------
  KApi_PNL_Cust_Suspend_Setting_Before_Backlight
  ------
  -----------------------------
  Backlight
  -----------------------------
  KApi_PNL_Cust_Suspend_Backlight_PWM
  -----------------------------
  PWM
  -----------------------------
  ------
  KApi_PNL_Cust_Suspend_Setting_Backlight_OffTiming1
  ------
  -----------------------------
  offTiming1 delay
  -----------------------------
  ------
  KApi_PNL_Cust_Suspend_Setting_OffTiming1_Data
  ------
  -----------------------------
  Data
  -----------------------------
  ------
  KApi_PNL_Cust_Suspend_Setting_Data_OffTiming2
  ------
  -----------------------------
  offTiming2 delay
  -----------------------------
  ------
  KApi_PNL_Cust_Suspend_Setting_OffTiming2_VCC
  ------
  -----------------------------
  VCC
  -----------------------------
  ------
  KApi_PNL_Cust_Suspend_Setting_After_VCC
  ------

*/

MS_BOOL KApi_PNL_Cust_Suspend_Setting_Before_Backlight(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Backlight_PWM(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_Backlight_OffTiming1(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_OffTiming1_Data(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_Data_OffTiming2(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_OffTiming2_VCC(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_ExtIC_PowerOff(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_After_VCC(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}


/*

  Normal Panel Resume flow:

  -----------------------------
  VCC
  -----------------------------
  onTiming1 delay
  -----------------------------
  Data
  -----------------------------
  onTiming2 delay
  -----------------------------
  Backlight
  -----------------------------

  if all customization insert at str resume like bellow:

  ------
  KApi_PNL_Cust_Resume_Setting_Before_VCC
  ------
  -----------------------------
  VCC
  -----------------------------
  ------
  KApi_PNL_Cust_Resume_Setting_VCC_onTiming1
  ------
  -----------------------------
  onTiming1 delay
  -----------------------------
  ------
  KApi_PNL_Cust_Resume_Setting_OnTiming1_Data
  ------
  -----------------------------
  Data
  -----------------------------
  ------
  KApi_PNL_Cust_Resume_Setting_Data_OnTiming2
  ------
  -----------------------------
  onTiming2 delay
  -----------------------------
  ------
  KApi_PNL_Cust_Resume_Setting_OnTiming2_Backlight
  ------
  -----------------------------
  PWM
  -----------------------------
  KApi_PNL_Cust_Resume_Setting_PWM_Backlight
  -----------------------------
  Backlight
  -----------------------------
  ------
  KApi_PNL_Cust_Resume_Setting_After_Backlight
  ------
*/
MS_BOOL KApi_PNL_Cust_Resume_Setting_Before_VCC(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_ExtIC_PowerOn(void)
{
    MS_BOOL bRet = FALSE;

    //PNL_CUST_STR_PRINT("PMIC bForceInit=%d\n",_stPnlCustPMICInfo.bForceInit);
    if(_stPnlCustPMICInfo.bForceInit)
    {
        if(_PNL_Cust_Resume_Setting_PMIC_PowerOn() == FALSE)
        {
            PNL_CUST_STR_PRINT("PMIC PowerOn fail\n");
        }
    }

    //PNL_CUST_STR_PRINT("PgammaIC bForceInit=%d\n",_stPnlCustPgammaICInfo.bForceInit);
    if(_stPnlCustPgammaICInfo.bForceInit)
    {
        if(_PNL_Cust_Resume_Setting_PGAMMAIC_PowerOn() == FALSE)
        {
            PNL_CUST_STR_PRINT("PgammaIC PowerOn fail\n");
        }
    }

    bRet = TRUE;    // PowerOn always return TRUE
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_VCC_OnTiming1(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_OnTiming1_Data(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_Data_OnTiming2(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_OnTiming2_Backlight(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_PWM_Backlight(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_After_Backlight(void)
{
    MS_BOOL bRet = FALSE;

//Here is a example for using IIC cmd to externel device
/*
   MS_U16 u16BusNumSlaveID = 0x266;
   MS_U8 u8BusNum = HIBYTE(u16BusNumSlaveID);
   MS_U8 u8SlaveID = LOBYTE(u16BusNumSlaveID);

   MS_U8 u8Offset=0x00;
   MS_U8 u8Reg_Value[42]={0x48,0x19,0xA4,0x00,0x00,0x23,0xF8,0x66,0x7B,0x2B,
                          0x28,0x00,0x10,0x10,0x07,0x07,0x3E,0xD3,0xD5,0x35,
                          0x13,0x06,0x2D,0xB2,0x73,0x24,0x81,0xEA,0x1A,0xE1,
                          0x4C,0x11,0xC0,0xC7,0x02,0xB0,0x0D,0x1C,0x51,0xB2,
                          0x4D,0x1D};
 
    pr_info("*****CONFIG_MP_PNL_STR_CUST define*****\n");
    int ret = MDrv_SW_IIC_WriteBytes(u8BusNum, u8SlaveID, 1, &u8Offset, sizeof(u8Reg_Value), u8Reg_Value);
    if (ret >= 0)
    {
        bRet = TRUE;
    }
    else
    {
        pr_err("*****\SWI2C error*****\n",__FILE__, __func__);
        bRet = FALSE;
    }
*/
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Initial_Setting_ExtIC_Init(void *pInfoArgs,MS_U32 u32InfoArgsSize)
{
    MS_BOOL bRet = FALSE;

    if(pInfoArgs == NULL)
    {
        PNL_CUST_STR_PRINT("Invalid parameters!!\n");
        return bRet;
    }

    if(u32InfoArgsSize == sizeof(ST_PNL_CUST_IC_INFO))
    {
        ST_PNL_CUST_IC_INFO *pstPnlCustIC_Info = (ST_PNL_CUST_IC_INFO *)pInfoArgs;
        PNL_CUST_STR_PRINT("IC Info size=%d. Match ST_PNL_CUST_IC_INFO structure size, length = %d\n",u32InfoArgsSize, pstPnlCustIC_Info->u32PnlCustICInfo_Length);

        if(pstPnlCustIC_Info->ePnlCustIcType == E_PNL_CUST_IC_PMIC)
        {
            bRet = _PNL_Cust_Initial_Setting_PMIC_Init(pstPnlCustIC_Info);
        }
        else if(pstPnlCustIC_Info->ePnlCustIcType == E_PNL_CUST_IC_PGAMMAIC)
        {
            bRet = _PNL_Cust_Initial_Setting_PGAMMAIC_Init(pstPnlCustIC_Info);
        }
        else
        {
            PNL_CUST_STR_PRINT("Invalid EN_PNL_CUST_IC_TYPE!!\n");
            bRet = FALSE;
        }
    }
    else
    {
        PNL_CUST_STR_PRINT("IC Info size=%d. Not match ST_PNL_CUST_IC_INFO structure size %d !!\n",u32InfoArgsSize, (MS_U32)sizeof(ST_PNL_CUST_IC_INFO));
        bRet = FALSE;
    }

    return bRet;
}

#else
MS_BOOL KApi_PNL_Cust_Suspend_Setting_Before_Backlight(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Suspend_Backlight_PWM(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_Backlight_OffTiming1(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_OffTiming1_Data(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_Data_OffTiming2(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_OffTiming2_VCC(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_ExtIC_PowerOff(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Suspend_Setting_After_VCC(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_Before_VCC(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_ExtIC_PowerOn(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_VCC_OnTiming1(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_OnTiming1_Data(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_Data_OnTiming2(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_OnTiming2_Backlight(void)
{
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_PWM_Backlight(void)
{
    MS_BOOL bRet = FALSE;
    return bRet;
}

MS_BOOL KApi_PNL_Cust_Resume_Setting_After_Backlight(void)
{
    pr_info("*****CONFIG_MP_PNL_STR_CUST not define*****\n");
    return FALSE;
}

MS_BOOL KApi_PNL_Cust_Initial_Setting_ExtIC_Init(void *pInfoArgs,MS_U32 u32InfoArgsSize)
{
    pr_info("*****CONFIG_MP_PNL_STR_CUST not define*****\n");
    return FALSE;
}

#endif
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_Before_Backlight);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Backlight_PWM);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_Backlight_OffTiming1);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_OffTiming1_Data);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_Data_OffTiming2);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_OffTiming2_VCC);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_ExtIC_PowerOff);
EXPORT_SYMBOL(KApi_PNL_Cust_Suspend_Setting_After_VCC);

EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_Before_VCC);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_ExtIC_PowerOn);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_VCC_OnTiming1);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_OnTiming1_Data);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_Data_OnTiming2);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_OnTiming2_Backlight);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_PWM_Backlight);
EXPORT_SYMBOL(KApi_PNL_Cust_Resume_Setting_After_Backlight);

EXPORT_SYMBOL(KApi_PNL_Cust_Initial_Setting_ExtIC_Init);


