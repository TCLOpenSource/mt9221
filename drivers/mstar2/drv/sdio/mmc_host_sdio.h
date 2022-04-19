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


#include <linux/mmc/host.h> // struct mmc_host_ops, mmc_host
#include <linux/mmc/core.h> // struct mmc_command, struct mmc_data, struct mmc_request
#include <linux/platform_device.h> // struct platform_device
#include <linux/module.h> //
#include <linux/dma-mapping.h> // dma_map_sg, dma_unmap_sg
#include <linux/errno.h> // error code
#include <linux/semaphore.h> // semaphore, down(), up()
#include <linux/kthread.h> // kthread_run()
#include <linux/delay.h> // msleep()
#include <linux/mmc/card.h> // struct mmc_card
#include <linux/debugfs.h> // single_open()
#include <linux/version.h> // KERNEL_VERSION
#include <linux/fs.h> // file_open()
#include <asm/uaccess.h> // get_fs()
#include <linux/syscalls.h> // sys_open()
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/interrupt.h>
#include <card.h>
#endif
#include "hal_sdio.h"

#define PATCH_BCM_USING_SLOW_CLK_IN_SDR104	0
#define KEEP_SDIO_BUS_CLOCK	0
#define PATCH_MTK_7668S_CMD11_ISSUE	1

void mmc_sdio_request(struct mmc_host *host, struct mmc_request *req);
void mmc_sdio_pre_req(struct mmc_data *data);
void mmc_sdio_post_req(struct mmc_data *data);
void mmc_sdio_set_ios(struct mmc_host *host, struct mmc_ios *ios);
int mmc_sdio_get_cd(struct mmc_host *host);
int mmc_sdio_get_ro(struct mmc_host *host);

int mmc_sdio_hotplug(void *data);

