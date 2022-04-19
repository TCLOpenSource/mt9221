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

#ifndef _CPUQOS_DRV_H
#define _CPUQOS_DRV_H
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>

#include "mdrv_qos.h"
#include "mdrv_qos_drv.h"

#define CPUQOS_PRINT_LOW    1
#define CPUQOS_PRINT_HIGH   2
#define cpuqos_print(level, x...)           \
do {                        \
    if (likely(!qos_debug_msg)) {\
        switch (level) {    \
            case CPUQOS_PRINT_HIGH: \
                pr_err(LOG_TAG" " x);         \
                break; \
            case CPUQOS_PRINT_LOW: \
            default: \
                break; \
        } \
	} else if (qos_debug_msg == 1){\
        switch (level) {    \
            case CPUQOS_PRINT_HIGH: \
                pr_err(LOG_TAG" " x);         \
                break; \
            case CPUQOS_PRINT_LOW: \
            default: \
                pr_info(LOG_TAG" " x);         \
                break; \
        } \
	} else if (qos_debug_msg == 2){\
		pr_emerg(LOG_TAG" " x);\
	}\
} while (0)

#define CPUQOS_FILE_LINE_LEN     128
#define CPUQOS_FILE_CONTENT_LEN  PAGE_SIZE
#define CPUQOS_PATH_LEN          128
#define CPUQOS_VAL_LEN            32
#define CPUQOS_PID_LEN            16
#define CPUQOS_OWNER_LEN          32
#define CPUQOS_RET_FAIL               QOS_RET_FAIL
#define CPUQOS_RET_SUCCESS            QOS_RET_SUCCESS
/* Android's definition for long press */
#define CPUQOS_ANDROID_LONG_PRESS_MS  500

enum cpuqos_key_detect {
	CPUQOS_KEY_IS_NOT_DETECTED = 0,
	CPUQOS_KEY_IS_DETECTED,
};

struct cpuqos_tids {
	pid_t tid;
	struct list_head tid_head;
	struct cpuqos_pids *pid; /* point to the parent */
};

struct cpuqos_pids {
	pid_t pid;
	struct list_head tid_heads;
	struct list_head pid_head;
	struct cpuqos_registry *reg; /* point to the parent */
};

enum trigger_map_index {
    QOS_CPU_LOAD_INDEX = 0,

	__NR_TRIGGER_MAP_TYPE
};

struct cpuqos_scenario {
	char node_name[CPUQOS_OWNER_LEN];
	struct qos_video_info video;
	u64 mask_id; /* used for fast matching */
	u32 output_map[__NR_OUTPUT_MAP_TYPE]; /* parsed from DTS */
	u32 delay_ctrl[__NR_OUTPUT_MAP_TYPE]; /* parsed from DTS */
	u32 init_ctrl; /* parsed from DTS */
	struct cpuqos_output_map *outputs[__NR_OUTPUT_MAP_TYPE]; /* related output node */
	u32 trigger_map[__NR_TRIGGER_MAP_TYPE]; /* the thresholds to trigger QoS */
	struct list_head head;  /* list_head to next scenario */
	bool has_delayed_ctrl;
};

enum cpuqos_reg_type {
	CPUQOS_REG_UNINIT = 0,
	CPUQOS_REG_PID,
	CPUQOS_REG_TID,
	CPUQOS_REG_OTHER,

	__NR_CPUQOS_REG_TYPE
};

struct cpuqos_registry {
    u32 reg_type;
	char node_name[CPUQOS_OWNER_LEN];
    char write_path[CPUQOS_PATH_LEN];
    char val_string[CPUQOS_VAL_LEN];
    char proc_comm[TASK_COMM_LEN];
    char thread_comm[TASK_COMM_LEN];
	u32 pid_num;
	u32 tid_num;
	pid_t last_searched_pid;
    struct list_head pid_heads;
	struct list_head head; /* list_head to next registry */
};

enum cpuqos_output_map_type {
	CPUQOS_OUTPUT_MAP_UNINIT = 0,
	CPUQOS_OUTPUT_MAP_WRITER,
	CPUQOS_OUTPUT_MAP_EPOLL,
	CPUQOS_OUTPUT_MAP_CALLBACK,

	__NR_CPUQOS_OUTPUT_MAP_TYPE
};

struct cpuqos_output_writer {
    char write_path[CPUQOS_PATH_LEN];
	char writer_str[__NR_QOS_CTRL_OUT_LEVEL][CPUQOS_VAL_LEN];
	struct list_head head;  /* list_head to next writer */
};

struct cpuqos_output_map {
	u32 output_type;
	u32 id;
	char owner[CPUQOS_OWNER_LEN];
	u32 output_set[__NR_QOS_CTRL_OUT_LEVEL];
	struct list_head writer_heads;
	struct qos_cb_unit *cb;
	struct list_head head;  /* list_head to next output_map */
};

extern atomic_t cpuqos_cpu_load;
extern int cpu_load_period_ms;
extern int cpu_load_release_diff;
extern int register_scan_period_ms;
extern int cpu_ctrl_flip_ms;
extern int key_detect_lasting_ms;
extern struct qos_ctrl qos_brain;

int cpuqos_init(void);
int cpuqos_reboot_handler (struct notifier_block *nb, unsigned long event,
		void *dummy);
int cpuqos_pm_handler (struct notifier_block *nb, unsigned long event,
		void *dummy);
bool cpuqos_is_on(void);
void cpuqos_set_off(void);
void cpuqos_set_on(void);
void cpuqos_key_detect_entry(int key_value, unsigned int key_code,
		u64 *new_scen_id);
void cpuqos_get_cpu_load_once(u64 *usage, u64 *total);
void cpuqos_output_adjust(struct cpuqos_scenario *main_scen,
        u32 output_level, enum qos_output_mode mode,
		struct cpuqos_scenario *opt_scen);
u32 cpuqos_update_cpu_load(u64 *used_1, u64 *total_1);
int cpuqos_proc_init(struct proc_dir_entry *parent_dir);
int cpuqos_parse_dts(void);
void cpuqos_print_parsed_dts(void);
int cpuqos_build_pids(struct cpuqos_registry *reg);
int cpuqos_build_tids(struct cpuqos_registry *reg);
void cpuqos_print_reg(struct cpuqos_registry *reg);
#endif
