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


#ifndef _MDRV_ALSA_DRIVER_HEADER
#define _MDRV_ALSA_DRIVER_HEADER

/*
 * ============================================================================
 * Include Headers
 * ============================================================================
 */
#include "mdrv_public.h"
#include "mdrv_version.h"


/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
/* Define a Ring Buffer data structure for MStar Audio DSP */
struct MStar_Device_Buffer_Struct {
	unsigned char *addr;
	unsigned int size;
	unsigned int avail_size;
	unsigned int inused_size;
	unsigned int consumed_size;
};

/* Define a Substream data structure for MStar Audio DSP */
struct MStar_Substream_Struct {
	struct snd_pcm_substream *substream;
	unsigned int substream_status;
};

/* Define a Monitor data structure for MStar Audio DSP */
struct MStar_Monitor_Struct {
	unsigned int monitor_status;
	unsigned int expiration_counter;
	snd_pcm_uframes_t last_appl_ptr;
	snd_pcm_uframes_t last_hw_ptr;
};

/* Define a Runtime data structure for MStar Audio DSP */
struct MStar_Runtime_Struct {
	struct snd_pcm_hw_constraint_list constraints_rates;
	struct MStar_MAD_Ops ops;
	struct MStar_Substream_Struct substreams[MAD_MAX_SUBSTREAMS];
	struct MStar_Device_Buffer_Struct buffer[MAD_MAX_SUBSTREAMS];
	struct MStar_Device_Buffer_Struct convert_buffer[MAD_MAX_SUBSTREAMS];
	struct MStar_Monitor_Struct monitor;
	struct timer_list timer;
	struct mutex mutex_lock;
	spinlock_t spin_lock;
	unsigned int active_substreams;
	unsigned int max_substreams;
	unsigned int channel_mode;
	unsigned int sample_rate;
	unsigned int runtime_status;
	unsigned int device_status;
};

/* Define a Device data structure for MStar Audio DSP */
struct MStar_Device_Struct {
	struct snd_card *card;
	struct snd_pcm *pcm[MAD_MAX_DEVICES];
	struct MStar_Runtime_Struct pcm_playback[MAD_MAX_DEVICES];
	struct MStar_Runtime_Struct pcm_capture[MAD_MAX_DEVICES];
	unsigned int active_playback_devices;
	unsigned int active_capture_devices;
};

#endif /* _MDRV_ALSA_DRIVER_HEADER */

