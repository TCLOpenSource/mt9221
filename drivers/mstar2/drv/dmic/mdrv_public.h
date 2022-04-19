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


#ifndef _MDRV_ALSA_PUBLIC_HEADER
#define _MDRV_ALSA_PUBLIC_HEADER

/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
#define DMIC_TRUE    1
#define DMIC_FALSE    0
#define DMIC_MAX_DEVICES    1
#define DMIC_MAX_SUBSTREAMS    8

#define DMIC_ERR(fmt,args...)       pr_err("[%010u][DMIC_ALSA][%06d]     " fmt, jiffies_to_msecs(jiffies), __LINE__, ## args)
#define DMIC_INFO(fmt,args...)      pr_info("[%010u][DMIC_ALSA][%06d]     " fmt, jiffies_to_msecs(jiffies), __LINE__, ## args)
#define DMIC_DEBUG(fmt,args...)      pr_debug("[%010u][DMIC_ALSA][%06d]     " fmt, jiffies_to_msecs(jiffies), __LINE__, ## args)

enum MStar_GET_CMD {
	E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES,
	E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES,
	E_PCM_CAPTURE_GET_DEVICE_STATUS,
	E_PCM_CAPTURE_GET_DEVICE_USED,
};

enum MStar_SET_CMD {
	/* Capture */
	E_PCM_CAPTURE_SET_CHANNEL_MODE,
	E_PCM_CAPTURE_SET_SAMPLE_RATE,
	E_PCM_CAPTURE_SET_BUFFER_SIZE,
	E_PCM_CAPTURE_SET_BIT_WIDTH,

	E_PCM_CAPTURE_SET_DMIC_ENABLE,
	E_PCM_CAPTURE_SET_DMIC_NUM,
	E_PCM_CAPTURE_SET_DMIC_REF_NUM,
	E_PCM_CAPTURE_SET_DMIC_HPF_SWITCH,
	E_PCM_CAPTURE_SET_DMIC_HPF_CONFIG,
	E_PCM_CAPTURE_SET_DMIC_SINE_GEN,
	E_PCM_CAPTURE_SET_DMIC_GAIN,
	E_PCM_CAPTURE_SET_DMIC_DSP_SUPPORT,
};

enum {
	E_STOP = 0,
	E_START,
	E_PAUSE,
	E_PAUSE_RELEASE,
	E_PREPARE,
	E_SUSPEND,
	E_RESUME,
};

struct MStar_DMIC_Ops {
	int (*open)(void);
	int (*close)(void);
	int (*start)(void);
	int (*stop)(void);
	int (*resume)(void);
	int (*suspend)(void);
	unsigned int (*read)(void *buffer, unsigned int bytes);
	unsigned int (*write)(void *buffer, unsigned int bytes);
	int (*get)(int cmd, unsigned int *param);
	int (*set)(int cmd, unsigned int *param);
	int (*mmap)(struct vm_area_struct *vma);
    int (*init)(void);
};

struct MStar_DMIC_Info {
	char name[32];
	char version[32];
    unsigned int max_pcm_buffer_size;
	struct MStar_DMIC_Ops *capture_pcm_ops;
};


/*
 * ============================================================================
 * Forward Declarations
 * ============================================================================
 */
extern int _mdrv_dmic_hook_device(struct MStar_DMIC_Info *mad_info);
extern int _mdrv_dmic_unhook_device(void);

#endif /* _MDRV_ALSA_PUBLIC_HEADER */

