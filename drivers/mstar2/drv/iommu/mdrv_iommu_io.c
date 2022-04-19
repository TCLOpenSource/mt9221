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
/// file    mdrv_temp_io.c
/// @brief  TEMP Driver Interface for Export
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif
#include <linux/device.h>

#if defined(CONFIG_MSTAR_UTOPIA2K_MDEBUG)
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include "mdrv_types.h"
#include "mdrv_system.h"
#endif

//drver header files
#include "mst_devid.h"
#include "mdrv_mstypes.h"
#include "chip_int.h"
#include "mhal_iommu.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define  IOMMUDBG_WARN(x)   x // open when debug time for serious issue.

#define ADDRESS_BIT33                                        (0x200000000ULL)

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
typedef struct
{
    int     id;
    bool        id_auto;
} platform_iommu_device;

static platform_iommu_device _deviommu =
{
    .id = 6,
    .id_auto = 0,
};
//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------

static irqreturn_t _irq_top(int eIntNum, void* dev_id)
{
    return IRQ_WAKE_THREAD;
}

irqreturn_t MDrv_IOMMU_SecurityInterrupt(int irq,void *devid)
{
    ST_IOMMU_MIU_ERROR_LOG stMIUErrorLog;
    ST_IOMMU_IOMMU_ERROR_LOG stIOMMUErrorLog;

    memset(&stMIUErrorLog, 0, sizeof(ST_IOMMU_MIU_ERROR_LOG));
    memset(&stIOMMUErrorLog, 0, sizeof(ST_IOMMU_IOMMU_ERROR_LOG));
    MHal_IOMMU_GetMIUErrorLog(&stMIUErrorLog);

    switch (stMIUErrorLog.enErrorCode)
    {
        case E_IOMMU_ERROR_LEGACY_FAILED:         // legacy mode check failed
            IOMMUDBG_WARN(printk("[IOMMU]error case0:legacy mode check failed.\n"));
            printk("[IOMMU] error info: PA=0x%llx,AID=%d,readpermission=%d,writepermission=%d,writeops=%d,SecRgeSrt=0x%llx,SecRgeEnd=0x%llx\n",
                    stMIUErrorLog.u64PAAddr, stMIUErrorLog.u16AID, stMIUErrorLog.bReadPermission, stMIUErrorLog.bWritePermission,
                    stMIUErrorLog.bWriteOperation, stMIUErrorLog.u64SecRangeStart, stMIUErrorLog.u64SecRangeEnd);
            break;
        case E_IOMMU_ERROR_SEC_RANGE_FAILED:      // IOMMU/MPU secure range check failed
            IOMMUDBG_WARN(printk("[IOMMU]error case1:IOMMU/MPU(SR_info) secure range check failed.\n"));
            printk("[IOMMU] error info: PA=0x%llx,AID=%d,SR_info=%d,readpermission=%d,writepermission=%d,writeops=%d,SecRgeSrt=0x%llx,SecRgeEnd=0x%llx\n",
                    stMIUErrorLog.u64PAAddr, stMIUErrorLog.u16AID,stMIUErrorLog.u8SR_info, stMIUErrorLog.bReadPermission, stMIUErrorLog.bWritePermission,
                    stMIUErrorLog.bWriteOperation, stMIUErrorLog.u64SecRangeStart, stMIUErrorLog.u64SecRangeEnd);
            break;
        case E_IOMMU_ERROR_LEGACY_SCE_OVERLAP:    // legacy mode secure range overlap
            IOMMUDBG_WARN(printk("[IOMMU]error case2:legacy mode secure range overlapped.\n"));
            printk("[IOMMU] error info: PA=0x%llx,u8FirstSRI=%d,u8SecondSRI=%d,SecRgeSrt=0x%llx,SecRgeEnd=0x%llx\n",
                    stMIUErrorLog.u64PAAddr, stMIUErrorLog.u8FirstSRI, stMIUErrorLog.u8SecondSRI, stMIUErrorLog.u64SecRangeStart, stMIUErrorLog.u64SecRangeEnd);
            break;
        case E_IOMMU_ERROR_MISMATCH_SEC_RANGE:     // secure range mismatch between legacy mode and IOMMU/MPU mode
            IOMMUDBG_WARN(printk("[IOMMU] error case3:secure range mismatch between legacy mode and IOMMU/MPU mode.\n"));
            printk("[IOMMU] error info: PA=0x%llx,u8FirstSRI=%d,u8SR_info=%d,SecRgeSrt=0x%llx,SecRgeEnd=0x%llx\n",
                    stMIUErrorLog.u64PAAddr, stMIUErrorLog.u8FirstSRI, stMIUErrorLog.u8SR_info, stMIUErrorLog.u64SecRangeStart, stMIUErrorLog.u64SecRangeEnd);
            break;
        case E_IOMMU_ERROR_CASE3_IN_MPU_AREA:      // case3 IP access MPU area
            IOMMUDBG_WARN(printk("[IOMMU]error case4: case3 IP access MPU area.\n"));
            MHal_IOMMU_GetIOMMUErrorLog(stMIUErrorLog.u8ClientID, &stIOMMUErrorLog);
            printk("[IOMMU] error info: PA=0x%llx,u8SR_info=%d, miuclient=0x%x,SecRgeSrt=0x%llx,SecRgeEnd=0x%llx,mpuEnable=%d\n",
                    stMIUErrorLog.u64PAAddr, stMIUErrorLog.u8SR_info, stMIUErrorLog.u8ClientID, stMIUErrorLog.u64SecRangeStart, stMIUErrorLog.u64SecRangeEnd, stIOMMUErrorLog.bMPUEnable);
            break;
        case E_IOMMU_ERROR_ACCESS_DENIED:          // IOMMU/MPU IP access denied
            IOMMUDBG_WARN(printk("[IOMMU]error case5: IOMMU/MPU IP access denied.\n"));
            MHal_IOMMU_GetIOMMUErrorLog(stMIUErrorLog.u8ClientID, &stIOMMUErrorLog);
            printk("[IOMMU] error info: PA=0x%lx,u8SR_info=%d,\n", stMIUErrorLog.u64PAAddr, stMIUErrorLog.u8SR_info);
            // addr[33] == 0?  PA or IOVA
            if ((stIOMMUErrorLog.u64PAAddr & ADDRESS_BIT33) == 0)
            {
                printk("[IOMMU] This error is caused by Unsupported VSR for case2 IP.\n");
            }
            else
            {
                printk("[IOMMU] error info:IOMMU_info=%d,IOMMUEnable=%d.\n", stIOMMUErrorLog.u8IOMMU_info, stIOMMUErrorLog.bIOMMUEnable);
            }
            break;
        default:
            IOMMUDBG_WARN(printk("[IOMMU]unkonw error case.\n"));
    }

    return IRQ_HANDLED;
}

static int _MDrv_IOMMU_RegisterInterrupt(void)
{
    int error;

    if(0 > (error=request_threaded_irq(E_IRQ_DISP, _irq_top, MDrv_IOMMU_SecurityInterrupt, IRQF_SHARED | IRQF_ONESHOT, "iommu", &_deviommu)))
    {
        printk("\n[XC] Fail to request IRQ:%d\n", E_IRQ_DISP);
        return -EFAULT;
    }

    return 0;
}

static int _MDrv_IOMMU_DeRegisterInterrupt(void)
{
    free_irq(E_IRQ_DISP,NULL);

    return 0;
}

MSYSTEM_STATIC int __init mod_iommu_init(void)
{
   // _MDrv_IOMMU_RegisterInterrupt();

    return 0;
}

MSYSTEM_STATIC void __exit mod_iommu_exit(void)
{
    //_MDrv_IOMMU_DeRegisterInterrupt();
}

module_init(mod_iommu_init);
module_exit(mod_iommu_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("Smart card driver");
MODULE_LICENSE("GPL");
