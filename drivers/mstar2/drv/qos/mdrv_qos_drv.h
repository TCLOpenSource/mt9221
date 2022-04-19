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

#ifndef _QOS_PROC_H
#define _QOS_PROC_H

#include <linux/fs.h>
#include <linux/list.h>

#include "mdrv_qos.h"

#define QOS_PRINT_LOW    1
#define QOS_PRINT_HIGH   2

#define tmp_buffer_size   32

#define LOG_TAG  "[QoS][CPU]"

#define qos_print(level, x...)           \
do {                        \
    if (likely(!qos_debug_msg)) {\
        switch (level) {    \
            case QOS_PRINT_HIGH: \
                pr_err(LOG_TAG" " x);         \
                break; \
            case QOS_PRINT_LOW: \
            default: \
                break; \
        } \
	} else if (qos_debug_msg == 1){ \
        switch (level) {    \
            case QOS_PRINT_HIGH: \
                pr_err(LOG_TAG" " x);         \
                break; \
            case QOS_PRINT_LOW: \
            default: \
                pr_info(LOG_TAG" " x);         \
                break; \
        } \
    } else if (qos_debug_msg == 2) {\
        pr_emerg(LOG_TAG" " x);\
    }\
} while (0)

#define qos_print_cont(x...)           \
do {                        \
        printk(KERN_CONT x);         \
} while (0)

/* these macro are used to save the efforts for qos' proc implementation */
#define __MACRO_PASTER(x, y) x ## y
#define MACRO_PASTER(x, y)   __MACRO_PASTER(x, y)
#define STRINGIZE(x) #x
#define CPUQOS_PROC_OPS_POSTFIX    _fileops
#define CPUQOS_PROC_OPEN_POSTFIX   _open
#define CPUQOS_PROC_WRITE_POSTFIX  _write
#define CPUQOS_PROC_SHOW_POSTFIX   _seq_show


/* helper macro for QOS_ATTR_INIT_RW */
#define QOS_ATTR_IMPL_RW(var_name, default_value) \
int var_name = default_value; \
static ssize_t MACRO_PASTER(var_name, CPUQOS_PROC_WRITE_POSTFIX) \
(struct file *file, const char __user *buf, size_t count, loff_t *ppos); \
\
static int MACRO_PASTER(var_name, CPUQOS_PROC_OPEN_POSTFIX) \
(struct inode *inode, struct file *file);\
\
static const struct file_operations MACRO_PASTER(var_name, _fileops) = { \
    .owner      = THIS_MODULE, \
    .open       = MACRO_PASTER(var_name, CPUQOS_PROC_OPEN_POSTFIX), \
    .write      = MACRO_PASTER(var_name, CPUQOS_PROC_WRITE_POSTFIX), \
    .read       = seq_read, \
    .llseek     = seq_lseek, \
}; \
\
static int MACRO_PASTER(var_name, CPUQOS_PROC_SHOW_POSTFIX) \
(struct seq_file *seq, void *v) \
{ \
    seq_printf(seq, "%d\n", var_name); \
    return 0; \
} \
\
static int MACRO_PASTER(var_name, CPUQOS_PROC_OPEN_POSTFIX) \
(struct inode *inode, struct file *file) \
{ \
    single_open(file, MACRO_PASTER(var_name, CPUQOS_PROC_SHOW_POSTFIX), NULL); \
    return 0; \
} \
\
static ssize_t  MACRO_PASTER(var_name, CPUQOS_PROC_WRITE_POSTFIX) \
(struct file *file, const char __user *buf, size_t count, loff_t *ppos) \
{ \
    char buffer[tmp_buffer_size] = {}; \
    size_t ret = count; \
    if (count >= tmp_buffer_size) \
        count = tmp_buffer_size - 1; \
    if (copy_from_user(buffer, buf, count)) { \
        return -EFAULT; \
    } \
    if (sscanf(buffer, "%d", &var_name) != 1) \
    { \
        qos_print(QOS_PRINT_HIGH, "Error parsing\n"); \
        ret = -EINVAL; \
    } else { \
        qos_print(QOS_PRINT_HIGH, STRINGIZE(var_name)"=%d\n", var_name); \
    } \
    return ret; \
}

/* create proc file that need to implement fops independently */
#define QOS_ATTR_CREATE(entry, var_name, parent_dir, fops) \
do { \
    entry = proc_create(STRINGIZE(var_name), S_IRUSR | S_IWUSR, parent_dir, \
            &fops); \
    if(entry == NULL) { \
        qos_print(QOS_PRINT_HIGH, \
                "proc_create("STRINGIZE(var_name)") failed.\n"); \
        return -ENOMEM; \
    } \
} while (0)


/* create the "int" proc file for as magic number or threshold */
#define QOS_ATTR_INIT_RW(entry, var_name, parent_dir) \
do { \
    QOS_ATTR_CREATE(entry, var_name, parent_dir,\
            MACRO_PASTER(var_name, CPUQOS_PROC_OPS_POSTFIX));\
} while (0)

/* helper to print in binary format*/
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define U64_TO_BINARY_PATTERN \
    BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN \
    BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN \
    BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN

#define U64_TO_BINARY_ARGS(val) \
    BYTE_TO_BINARY(val >> 56), BYTE_TO_BINARY(val >> 48), \
    BYTE_TO_BINARY(val >> 40), BYTE_TO_BINARY(val >> 32), \
    BYTE_TO_BINARY(val >> 24), BYTE_TO_BINARY(val >> 16), \
    BYTE_TO_BINARY(val >> 8), BYTE_TO_BINARY(val)

struct file *qos_fopen(const char *path, int flags, int rights);
void qos_fclose(struct file *file);
int qos_fread(struct file* file, unsigned long long *offset, char *data,
		unsigned int size);
int qos_fwrite(struct file *file, unsigned long long *offset, char *data,
		unsigned int size);
int qos_write_path(char *path, char buf[]);
int qos_multi_write_path(char *path, char *buf, int write_count, int buf_stride);

/* parsing DTS related helper, make sure a local var "ret" exist. */
#define QOS_GET_NODE(full_path, node_ptr) \
do { \
    node_ptr = of_find_node_opts_by_path(full_path, NULL); \
    if (!node_ptr) { \
        qos_print(QOS_PRINT_HIGH, "Cannot find %s node at %s: %d\n",\
                full_path, __FILE__, __LINE__); \
        ret = -ENODEV;\
    } \
} while (0)
void qos_dts_to_video(struct qos_video_info *result,
		u32 video_dts[__NR_QOS_VIDEO_MEMBER]);
u64 qos_scenario_to_mask(struct qos_video_info *video, u32 key_bit, u32 voice_bit);
void qos_dump_scenario_mask(u64 mask_id, int log_level);

struct qos_masks {
	/* general masks */
	u64 mask_src;
	u64 mask_resol;
	u64 mask_fps;
	u64 mask_codec;
	u64 mask_key;
	u64 mask_voc;
	/* special masks */
	u64 mask_video;
	u64 video_stop;
};

extern struct qos_masks matching_masks;

/* each callback has one "struct qos_cb_unit" */
struct qos_cb_unit {
    int (*ctrl_fn)(enum qos_ctrl_level level); /* callback function */
    char owner[QOS_CALLBACK_NAME_LEN]; /* must match the config. file */
	struct list_head cb_head; /* list node of "struct qos_cb_unit" */
	struct list_head cb_work_arg_heads; /* list heads of "struct qos_cb_args" */
	struct mutex cb_args_lock; /* ensure the calling order w/ args */
    struct work_struct qos_cb_work;
	/* qos uses "the worst" policy for a cb shared w/ more than one QoS */
	u32 current_levels[__NR_QOS_TYPE];
	u32 real_level;
};

/* Pass callback's args to kworker */
struct qos_cb_args {
	enum qos_ctrl_level level;
	struct list_head queue_head;
};

enum qos_scenario_record_index {
	QOS_LAST_SCENARIO_INDEX = 0,
	QOS_PREV_LAST_SCENARIO_INDEX,

	__NR_QOS_SCENARIO_RECORD
};

struct cpuqos_output_status {
	struct cpuqos_scenario *scen;
	u32 output_level;
};

/* indices for "outputs" in "struct cpuqos_scenario" */
enum output_map_index {
    __OUTPUT_MAP_START_INDEX = 0,
    /* writer */
    QOS_WRITER_OUTPUT_INDEX = __OUTPUT_MAP_START_INDEX,
    /* callbacks */
    __QOS_CALLBACK_START_INDEX,
    QOS_AIPQ_OUTPUT_INDEX = __QOS_CALLBACK_START_INDEX,
    QOS_UCD_OUTPUT_INDEX,
    __QOS_CALLBACK_END_INDEX,
	/* epoll */
	QOS_EPOLL_OUTPUT_INDEX = __QOS_CALLBACK_END_INDEX,
    /* total number of outputs */
    __NR_OUTPUT_MAP_TYPE
};

enum qos_ver_index {
	QOS_SRC_MAJOR_VER_INDEX = 0,
	QOS_SRC_MINOR_VER_INDEX,
	QOS_DTS_MAJOR_VER_INDEX,
	QOS_DTS_MINOR_VER_INDEX,

	__NR_QOS_VERSION_NUM,
};

enum qos_src_status {
	QOS_SRC_STOP = 0,
	QOS_SRC_IN, /* HDMI, MM or other input comes in */

	__NR_QOS_SRC_STATUS
};

enum qos_output_mode {
	QOS_OUTPUT_NORMAL = 0,
	QOS_OUTPUT_DELAY_INIT,
	QOS_OUTPUT_RESTORE,

	__NR_QOS_OUTPUT_MODE
};

enum qos_proc_op {
	QOS_OP_SRC_IN_STOP = 0,

	__NR_QOS_OP
};

struct qos_poll {
    char read_buf[32];
    size_t read_len;
    wait_queue_head_t wq;
    bool is_registered;
};

struct cpuqos_dts_lists {
    struct list_head scenario_head;
    struct list_head registry_head;
    struct list_head output_map_head;
};

struct cpuqos_meta {
    spinlock_t key_status_lock;
    struct timer_list monitor_key_timer;
    int key_detect_switch; /* key is pressed? */
    bool cpuqos_pm_ready;
    struct qos_poll poll;
	struct cpuqos_dts_lists *dts_heads;
};

/* shared by all QoS, including CPU & BW */
struct qos_ctrl {
	/* shared by all QoS */
    struct list_head qos_cb_heads;
	struct mutex cb_list_lock;
	spinlock_t scen_lock;
	u32 versions[__NR_QOS_VERSION_NUM];
	u32 src_status; /* enum qos_src_status, which is used for delay_ctrl */
	struct delayed_work delay_ctrl_timer;
	bool delay_timeout;
	bool delay_first_ctrl;
	u32 ctrl_reg_opt[__NR_OUTPUT_MAP_TYPE];
	/* the video info has specific protocol for playing video, "video_in_cnt" is
	 * used for keep in correct state machine */
	u32 video_in_cnt;

	/* Used by CPUQoS only */
	struct cpuqos_meta cpu_meta;
	u64 current_scen_id; /* current scenario in bitfields */
	struct cpuqos_scenario *matched_scen;
	struct cpuqos_scenario *default_scen;
	struct cpuqos_output_status prev_scen_rec[__NR_QOS_SCENARIO_RECORD];
	u32 cpuqos_prev_mapped_output[__NR_OUTPUT_MAP_TYPE];
};

void qos_scenario_set(u64 *scen_id, u64 num);
void qos_scenario_clear(u64 *scen_id, u64 num);
bool qos_scenario_test(u64 *scen_id, u64 num);
void qos_dump_masks(int log_level);
void qos_enter_scenario(void);
int qos_get_ver(void);
void qos_enter_scenario_helper(u32 clear_bits, u32 set_bits, bool need_lock);

extern struct qos_ctrl qos_brain;
extern bool qos_init_done;
extern int qos_debug_msg;
extern int qos_debug_scenario;
extern int qos_delay_ctrl_ms;

#endif
