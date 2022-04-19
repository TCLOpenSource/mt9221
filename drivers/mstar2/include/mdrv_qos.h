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

#ifndef _QOS_DRV_H
#define _QOS_DRV_H

#define QOS_RET_SUCCESS   0
#define QOS_RET_FAIL     -1

/* types of QoS, for both input events & output callbacks */
enum qos_type {
	QOS_TYPE_CPU = 0,
	QOS_TYPE_BW,

	__NR_QOS_TYPE
};

#define QOS_BIT_CPU  BIT(QOS_TYPE_CPU)
#define QOS_BIT_BW   BIT(QOS_TYPE_BW)
#define QOS_BIT_ALL  (QOS_BIT_BW | QOS_BIT_CPU)

#define QOS_CALLBACK_NAME_LEN   32

enum qos_event_in {
	/* DTS setting */
	QOS_EVENT_KEY = 0,
	QOS_EVENT_VIDEO,
	QOS_EVENT_VOICE,

	__NR_QOS_DTS_EVENT,

	/* ctrl status indication */
	QOS_EVENT_AIPQ = __NR_QOS_DTS_EVENT,
	QOS_EVENT_UCD,

	__NR_QOS_IN_EVENTS
};

/* Notify events to QoS */
extern void qos_event_detect(u32 qos_type, enum qos_event_in event,
		void *info);

/* Exit video playing or HDMI disconnect to exit scenario */
extern void qos_src_in_stop(void);

struct qos_key_info {
	int key_action;
	unsigned int key_button; 
};

struct qos_voice_info {
	bool is_enabled;
};

struct qos_in_bin_info {
	bool is_enabled;
	u64 event_on_bit; /* the event id, such as QOS_AIPQ_ON & QOS_UCD_ON */
};

/* 
 * mapping to EN_QOS_SOURCE,
 * possible desc. file options:
 * IGNORE, MM, DTV, ATV, HDMI
 */
enum qos_src_type {
	QOS_VIDEO_START_BIT = 0,
	QOS_SRC_IGNORE = QOS_VIDEO_START_BIT, // 0
	QOS_SRC_NONE,
	QOS_SRC_MM,
	QOS_SRC_DTV,
	QOS_SRC_ATV,
	QOS_SRC_HDMI,                         // 5

	__END_POS_QOS_SRC
};

/* 
 * mapping to EN_QOS_RESOLUTION
 * possible desc. file options:
 * IGNORE, SD, HD, FHD, 4KUHD, 8KUHD
 * The real resol. >= $RESOL in scenario is treated as matched.
 * e.g. if scenario is 4KUHD, 4KUHD & 8KUHD are both matched.
 */
enum qos_resolution_type {
    QOS_RESOL_IGNORE = __END_POS_QOS_SRC, // 6
    QOS_RESOL_NONE,
    QOS_RESOL_SD,
    QOS_RESOL_HD,
    QOS_RESOL_FHD,
    QOS_RESOL_4KUHD,
    QOS_RESOL_8KUHD,                      // 12

	__END_POS_QOS_RESOLUTION
};

/* 
 * mapping to EN_QOS_FPS
 * possible desc. file options:
 * 60, 50, 30, 25, 24
 * The real fps >= $FPS in scenario is treated as matched
 * e.g. if scenario is 30, 30 50 60 are all matched.
 */
enum qos_fps_type {
    QOS_FPS_IGNORE = __END_POS_QOS_RESOLUTION, // 13
    QOS_FPS_NONE,
    QOS_FPS_24P,
    QOS_FPS_25P,
    QOS_FPS_30P,
    QOS_FPS_50P,
    QOS_FPS_60P,                               // 19

	__END_POS_QOS_FPS
};

/* 
 * mapping to EN_QOS_CODEC_TYPE 
 * possible desc. file options:
 * H264, H265, VP9, AV1
 */
enum qos_codec_type {
    QOS_CODEC_IGNORE = __END_POS_QOS_FPS,  // 20
    QOS_CODEC_NONE,
    QOS_CODEC_H264,
    QOS_CODEC_H265,
    QOS_CODEC_VP9,
    QOS_CODEC_AV1,                         // 25

	__END_POS_QOS_CODEC, /* also means the number of bits are used */
	__END_POS_QOS_VIDEO_USED = __END_POS_QOS_CODEC
};

struct qos_video_info {
	enum qos_src_type src;
	enum qos_resolution_type resol;
	enum qos_fps_type fps;
	enum qos_codec_type codec;
};

enum qos_video_info_idx {
	QOS_VIDEO_INFO_SRC = 0,
	QOS_VIDEO_INFO_RESOL,
	QOS_VIDEO_INFO_FPS,
	QOS_VIDEO_INFO_CODEC,

	__NR_QOS_VIDEO_MEMBER
};

enum qos_event_bits {
	/* QOS_IR_KEY_SET & QOS_IR_KEY_NONE should be mutually exclusive */
	QOS_IR_KEY_SET = __END_POS_QOS_VIDEO_USED, // 26
	QOS_IR_KEY_NONE,                           // 27
	__END_POS_IR_KEY_USED,

	QOS_VOICE_ON = __END_POS_IR_KEY_USED,      // 28
	QOS_VOICE_OFF,                             // 29
	__END_POS_VOICE_USED,
	QOS_AIPQ_ON = __END_POS_VOICE_USED,        // 30
	__END_POS_AIPQ_USED,
	QOS_UCD_ON = __END_POS_AIPQ_USED,          // 31
	__END_POS_UCD_USED,
	__NR_TOTAL_EVENTS_USED_BITS = __END_POS_UCD_USED
};

/* Modify this if the # of events changes */
#define __NR_QOS_EVENT_DEF   6

#define QOS_CTRL_OUT_DISABLE   QOS_CTRL_OUT_LEVEL_0
#define QOS_CTRL_OUT_ENABLE    QOS_CTRL_OUT_LEVEL_7

enum qos_ctrl_level {
	QOS_CTRL_OUT_LEVEL_0 = 0, /* for the worst scenario (disable the feature) */
	QOS_CTRL_OUT_LEVEL_1,
	QOS_CTRL_OUT_LEVEL_2,
	QOS_CTRL_OUT_LEVEL_3,
	QOS_CTRL_OUT_LEVEL_4,
	QOS_CTRL_OUT_LEVEL_5,
	QOS_CTRL_OUT_LEVEL_6,
	QOS_CTRL_OUT_LEVEL_7, /* Do what ever you can (fully enable) */

	__NR_QOS_CTRL_OUT_LEVEL
};

/*
 * return value:
 * QOS_RET_SUCCESS or QOS_RET_FAIL to indicate status.
 * Once qos_ctrl_cb_register() return QOS_RET_SUCCESS, do not call it again!
 *
 * args:
 * cb_owner: name of the module, it must match the name used in the DTS & callback
 * ctrl_callback: the callback function in the module that can control the
 *     resource usage w.r.t. ctrl_callback().
 *     ctrl_callback() must return QOS_RET_SUCCESS or QOS_RET_FAIL to indicate
 *     status.
 *
 * This will create a kthread for registry.
 */
extern int qos_ctrl_cb_register(char *cb_owner,
        int (*ctrl_callback)(enum qos_ctrl_level level));

/* 
 * This is non-blocking API to find the callback (struct qos_cb_unit).
 * return NULL if not found.
 */
extern void *qos_find_cb(char *owner_name);

/* 
 * cb_unit is what callabck you are going to call. (qos_find_cb can help you)
 * qos_type & level are bound together.
 * qos_ctrl_call_cb will make sure the "worst output" are used.
 * If a worse level is passed, this will create a kwork to call the cb
 * asynchronically.
 */
extern void qos_ctrl_call_cb(u32 qos_type, void *cb_unit, u32 level);
#endif
