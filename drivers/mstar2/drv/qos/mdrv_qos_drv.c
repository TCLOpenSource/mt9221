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
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/ioctl.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/compiler.h>

#include "cpu/cpuqos_drv.h"
#include "mdrv_qos_drv.h"
#include "mdrv_qos.h"

struct qos_ctrl qos_brain = {
    /* version number of code base */
    .versions[QOS_SRC_MAJOR_VER_INDEX] = 1,
    .versions[QOS_SRC_MINOR_VER_INDEX] = 0
};

bool qos_init_done = false;
struct qos_masks matching_masks;

struct file *qos_fopen(const char *path, int flags, int rights)
{
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(filp)) {
        err = PTR_ERR(filp);
        qos_print(QOS_PRINT_HIGH, "filp_open() ret=%d.\n", err);
        return NULL;
    }
    return filp;
}

void qos_fclose(struct file *file)
{
    filp_close(file, NULL);
}

int qos_fwrite(struct file *file, unsigned long long *offset,
        char *data, unsigned int size)
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());
    ret = vfs_write(file, data, size, offset);
    set_fs(oldfs);
    return ret;
}

int qos_fread(struct file* file, unsigned long long *offset,
        char *data, unsigned int size)
{
    /* return value is the number of read chars */
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());
    ret = vfs_read(file, data, size, offset);
    set_fs(oldfs);
    return ret;
}

int qos_write_path(char *path, char buf[])
{
    struct file *f = qos_fopen(path, O_WRONLY | O_CREAT, S_IWUSR);
    int ret = 0;
    if (f) {
        ret = qos_fwrite(f, &f->f_pos, buf, strlen(buf)+1);
        qos_print(QOS_PRINT_LOW, "write %s to %s;\n", buf, path);
        if (ret < 0) {
            qos_print(QOS_PRINT_HIGH, "Failed to write %s at %s w/ ret=%d\n", buf, path, ret);
            qos_fclose(f);
            return QOS_RET_FAIL;
        }
        qos_fclose(f);
    } else {
        qos_print(QOS_PRINT_HIGH, "Open %s failed\n", path);
        return QOS_RET_FAIL;
    }
    return QOS_RET_SUCCESS;
}

int qos_multi_write_path(char *path, char *buf, int write_count, int buf_stride)
{
    struct file *f = qos_fopen(path, O_WRONLY, S_IWUSR);
    int fail_count = 0;
    int i;
    int ret;
    char *tmp_buf;
    if (likely(f)) {
		tmp_buf = buf;
        for (i = 0; i < write_count; i++) {
            ret = qos_fwrite(f, &f->f_pos, tmp_buf, strlen(tmp_buf)+1);
            qos_print(QOS_PRINT_LOW, "write_mul %s to %s;\n", tmp_buf, path);
            if (ret < 0) {
                qos_print(QOS_PRINT_HIGH,
                    "Failed to write %s at %s w/ ret=%d\n", tmp_buf, path, ret);
                fail_count++;
            }
			tmp_buf += buf_stride;
        }
        qos_fclose(f);
    } else {
        qos_print(QOS_PRINT_HIGH, "Open %s failed\n", path);
        return QOS_RET_FAIL;
    }
    if (unlikely(fail_count)) {
        return QOS_RET_FAIL;
    }
    return QOS_RET_SUCCESS;
}

void qos_dts_to_video(struct qos_video_info *result,
        u32 video_dts[__NR_QOS_VIDEO_MEMBER])
{
	/* prepare for u64 qos_scenario_to_mask() */
    result->src = video_dts[QOS_VIDEO_INFO_SRC] + QOS_SRC_IGNORE;
    result->resol = video_dts[QOS_VIDEO_INFO_RESOL] + QOS_RESOL_IGNORE;
    result->fps = video_dts[QOS_VIDEO_INFO_FPS] + QOS_FPS_IGNORE;
    result->codec = video_dts[QOS_VIDEO_INFO_CODEC] + QOS_CODEC_IGNORE;
}

/* This is used for matching, not for generating scenario */
u64 qos_scenario_to_mask(struct qos_video_info *video, u32 key_bit,
		u32 voice_bit)
{
	u64 ret = 0;
	u64 bit_LHS, bit_RHS;
	/* SRC, unique matching */
	if (video->src == QOS_SRC_IGNORE) {
		bit_RHS = QOS_SRC_IGNORE;
		bit_LHS = __END_POS_QOS_SRC;
		ret |= GENMASK_ULL(bit_LHS - 1, bit_RHS);
	} else {
		ret |= BIT(video->src);
	}
	/* 
	 * Resolution, above are all matched.
	 * But NONE is unique. 
	 */
	if (video->resol == QOS_RESOL_NONE) {
		ret |= BIT(video->resol);
	} else {
		bit_RHS = video->resol;
		bit_LHS = __END_POS_QOS_RESOLUTION;
		ret |= GENMASK_ULL(bit_LHS - 1, bit_RHS);
	}
	/*
	 * FPS, above are all matched.
	 * But NONE is unique. 
	 */
	if (video->fps == QOS_FPS_NONE) {
		ret |= BIT(video->fps);
	} else {
		bit_RHS = video->fps;
		bit_LHS = __END_POS_QOS_FPS;
		ret |= GENMASK_ULL(bit_LHS - 1, bit_RHS);
	}
	/* Codec, unique matching */
	if (video->codec == QOS_CODEC_IGNORE) {
		bit_RHS = QOS_CODEC_IGNORE;
		bit_LHS = __END_POS_QOS_CODEC;
		ret |= GENMASK_ULL(bit_LHS - 1, bit_RHS);
	} else {
		ret |= BIT(video->codec);
	}
	/* key event, try to match QOS_IR_KEY_SET or QOS_IR_KEY_NONE */
	ret |= BIT(key_bit);
	/* voice event, try to match QOS_VOICE_ON or QOS_VOICE_OFF */
	ret |= BIT(voice_bit);
	return ret;
}

void qos_dump_masks(int log_level)
{
	qos_print(log_level, "\n%10s -> "U64_TO_BINARY_PATTERN"\n"
			"%10s -> "U64_TO_BINARY_PATTERN"\n"
			"%10s -> "U64_TO_BINARY_PATTERN"\n"
			"%10s -> "U64_TO_BINARY_PATTERN"\n"
			"%10s -> "U64_TO_BINARY_PATTERN"\n"
			"%10s -> "U64_TO_BINARY_PATTERN"\n",
			"src_mask", U64_TO_BINARY_ARGS(
						GENMASK_ULL(__END_POS_QOS_SRC-1, QOS_SRC_IGNORE)),
			"resol_mask", U64_TO_BINARY_ARGS(
						GENMASK_ULL(__END_POS_QOS_RESOLUTION-1, QOS_RESOL_IGNORE)),
			"fps_mask", U64_TO_BINARY_ARGS(
						GENMASK_ULL(__END_POS_QOS_FPS-1, QOS_FPS_IGNORE)),
			"codec_mask", U64_TO_BINARY_ARGS(
						GENMASK_ULL(__END_POS_QOS_CODEC-1, QOS_CODEC_IGNORE)),
			"key_mask", U64_TO_BINARY_ARGS(
						GENMASK_ULL(__END_POS_IR_KEY_USED-1, QOS_IR_KEY_SET)),
			"voice_mask", U64_TO_BINARY_ARGS(
						GENMASK_ULL(__END_POS_VOICE_USED-1, QOS_VOICE_ON))
			);
}

/*
 * Print the results of qos_dts_to_video() -> qos_scenario_to_mask()
 */
void qos_dump_scenario_mask(u64 mask_id, int log_level)
{
	qos_print(log_level, "\n%10s -> "U64_TO_BINARY_PATTERN"\n",
			"mask_id", U64_TO_BINARY_ARGS(mask_id));
	qos_dump_masks(log_level);
	qos_print_cont("===================================\n");
}


static void qos_dump_compare_id(u64 scen_id, struct cpuqos_scenario *scen,
		int log_level)
{
	qos_print(log_level, "\n%10s -> "U64_TO_BINARY_PATTERN"\n"
			"%10s -> "U64_TO_BINARY_PATTERN"\n",
			"scen_id", U64_TO_BINARY_ARGS(scen_id),
			scen->node_name, U64_TO_BINARY_ARGS(scen->mask_id));
	qos_dump_masks(log_level);
	qos_print_cont("===================================\n");
}

static void qos_cb_engine(struct work_struct *w)
{
	int ret;
    struct qos_cb_unit *cb;
	struct qos_cb_args *args;
    cb = container_of(w, struct qos_cb_unit, qos_cb_work);
	mutex_lock(&cb->cb_args_lock);
	args = list_first_entry(&cb->cb_work_arg_heads, struct qos_cb_args, queue_head);
	qos_print(QOS_PRINT_LOW, "Ctrl-> cb-start: %s(%u);\n", cb->owner, args->level);
	ret = cb->ctrl_fn(args->level);
	qos_print(QOS_PRINT_LOW, "Ctrl-> cb-end: %s(%u), ret=%d;\n", cb->owner, 
			args->level, ret);
	list_del(&args->queue_head);
	kfree(args);
	mutex_unlock(&cb->cb_args_lock);
}

/* 
 * owner_name: the name that who registered. e.g. AIPQ
 * return NULL if not found.
 */
void *qos_find_cb(char *owner_name)
{
	struct qos_cb_unit *cb_curr;
	void *ret = NULL;
	if (!qos_init_done) {
		return NULL;
	}
	mutex_lock(&qos_brain.cb_list_lock);
    list_for_each_entry(cb_curr, &qos_brain.qos_cb_heads, cb_head) {
        if (!strncmp(cb_curr->owner, owner_name, QOS_CALLBACK_NAME_LEN)) {
            ret = (void *)cb_curr;
            break;
        }
    }
	mutex_unlock(&qos_brain.cb_list_lock);
	return ret;
}
EXPORT_SYMBOL(qos_find_cb); /* make ko can use it */

/*
 * args:
 * qos_type: the desired callback type to call.
 * cb: the registered callback's info.
 * level: the level of corresponding QoS is calling
 */
void qos_ctrl_call_cb(u32 qos_type, void *cb_unit, u32 level)
{
	struct qos_cb_args *arg;
	struct qos_cb_unit *cb = (struct qos_cb_unit *)cb_unit;
	u32 new_min_level;
	char _cpu[] = "CPU", _bw[] = "BW", _others[] = "others";
	char *who;

	if (unlikely(!cb_unit)) {
		qos_print(QOS_PRINT_HIGH, "CB is NULL for qos_type=%u.\n",
				qos_type);
		return;
	}
	mutex_lock(&cb->cb_args_lock);
	/* calc. the worst level after change */
	cb->current_levels[qos_type] = level; /* update level */
	new_min_level = min(cb->current_levels[QOS_TYPE_CPU],
			cb->current_levels[QOS_TYPE_BW]);
	if (cb->real_level == new_min_level) {
		switch (qos_type) {
			case QOS_TYPE_CPU:
				who = _cpu;
				break;
			case QOS_TYPE_BW:
				who = _bw;
				break;
			default:
				who = _others;
				break;
		}
		qos_print(QOS_PRINT_LOW, "[%s] set [%s] from [%u] to [%u] is cancelled.\n",
				who, cb->owner, cb->real_level, level);
		mutex_unlock(&cb->cb_args_lock);
		return;
	}
	cb->real_level = level;

	arg = kmalloc(sizeof(*arg), GFP_KERNEL);
	if (unlikely(!arg)) {
		qos_print(QOS_PRINT_HIGH, "qos_ctrl_call_cb() kmalloc() failed."
				"owner=%s, level=%u\n", cb->owner, level);
		mutex_unlock(&cb->cb_args_lock);
		return;
	}
	arg->level = level;
	/* use list as queue to pass args to kworker */
	list_add_tail(&arg->queue_head, &cb->cb_work_arg_heads);
	mutex_unlock(&cb->cb_args_lock);
	/* use kworker to prevent callback() blocking QoS */
	schedule_work(&cb->qos_cb_work);
}
EXPORT_SYMBOL(qos_ctrl_call_cb); /* make ko can use it */

static int __qos_ctrl_cb_register_helper(struct qos_cb_unit *cb_arg)
{
	if (qos_init_done) {
		mutex_lock(&qos_brain.cb_list_lock);
		list_add(&cb_arg->cb_head, &qos_brain.qos_cb_heads);
		mutex_unlock(&qos_brain.cb_list_lock);
		return QOS_RET_SUCCESS;
	}
	return QOS_RET_FAIL;
}

static int qos_ctrl_cb_register_helper(void *cb_arg)
{
    int qos_ret = QOS_RET_FAIL;
	struct qos_cb_unit *cb = (struct qos_cb_unit *)cb_arg;
    while (qos_ret != QOS_RET_SUCCESS) {
        qos_ret = __qos_ctrl_cb_register_helper(cb);
        if (qos_ret == QOS_RET_SUCCESS) {
            qos_print(QOS_PRINT_HIGH, "Register callback(owner=%s) "
                    "successfully.\n", cb->owner);
        } else {
            qos_print(QOS_PRINT_HIGH, "Register callback(owner=%s) "
                    "failed, retry...\n", cb->owner);
            msleep(2000);
        }
    }
	return 0;
}

int qos_ctrl_cb_register(char *cb_owner,
		int (*ctrl_callback)(enum qos_ctrl_level level))
{
	struct qos_cb_unit *cb_obj;
	struct task_struct *thread;
	int i;
	u32 fake_level = QOS_CTRL_OUT_ENABLE + 1;
    cb_obj = kzalloc(sizeof(*cb_obj), GFP_KERNEL);
    if (!cb_obj) {
		qos_print(QOS_PRINT_HIGH, "qos_ctrl_cb_register(%s) failed\n", cb_owner);
        return QOS_RET_FAIL;
    }
    cb_obj->ctrl_fn = ctrl_callback;
	for (i = 0; i < __NR_QOS_TYPE; i++) {
		/* avoid the first control to be ignored */
		cb_obj->current_levels[i] = fake_level;
	}
	cb_obj->real_level = fake_level;
    /* strlcpy always copy valid string */
    strlcpy((char *)cb_obj->owner, cb_owner, QOS_CALLBACK_NAME_LEN);
	mutex_init(&cb_obj->cb_args_lock);
	INIT_WORK(&cb_obj->qos_cb_work, qos_cb_engine);
	INIT_LIST_HEAD(&cb_obj->cb_work_arg_heads);
	thread = kthread_run(qos_ctrl_cb_register_helper, (void*)cb_obj,
			"qos_ctrl_cb_register_thread");
	if (IS_ERR(thread)) {
		qos_print(QOS_PRINT_HIGH, "qos_ctrl_cb_register(%s) create thread "
				"failed\n", cb_owner);
        return QOS_RET_FAIL;
	}
	return QOS_RET_SUCCESS;
}
EXPORT_SYMBOL(qos_ctrl_cb_register); /* make ko can use it */

void qos_scenario_set(u64 *scen_id, u64 num)
{
	*scen_id |= num;
}

void qos_scenario_clear(u64 *scen_id, u64 num)
{
	*scen_id &= ~num;
}

bool qos_scenario_test(u64 *scen_id, u64 num)
{
	if (*scen_id & num) {
		return true;
	}
	return false;
}

static bool qos_match_scen(u64 scen_result)
{
	int ret;
	ret = ((scen_result & matching_masks.mask_src) > 0) +
		((scen_result & matching_masks.mask_resol) > 0) +
		((scen_result & matching_masks.mask_fps) > 0) +
		((scen_result & matching_masks.mask_codec) > 0) +
		((scen_result & matching_masks.mask_key) > 0) +
		((scen_result & matching_masks.mask_voc) > 0);
	if (unlikely(qos_debug_scenario)) {
		qos_print(QOS_PRINT_LOW, "src=%d; resol=%d; fps=%d; codec=%d; key=%d;"
				" voc=%d\n", ((scen_result & matching_masks.mask_src) > 0),
				((scen_result & matching_masks.mask_resol) > 0),
				((scen_result & matching_masks.mask_fps) > 0),
				((scen_result & matching_masks.mask_codec) > 0),
				((scen_result & matching_masks.mask_key) > 0),
				((scen_result & matching_masks.mask_voc) > 0)
			);
	}
	if (ret == __NR_QOS_EVENT_DEF) {
		return true;
	}
	return false;
}

void qos_enter_scenario(void)
{
	struct cpuqos_scenario *scen;
	bool found = false;
	u64 cached_scen_id = READ_ONCE(qos_brain.current_scen_id);
	u64 tmp_video_id;
	int output_level;
	if (unlikely(!cpuqos_is_on())) {
		return;
	}
    list_for_each_entry(scen, &qos_brain.cpu_meta.dts_heads->scenario_head,
			head) {
		/* 
		 * the order of iteration is decided by the dts, and QoS uses this
		 * feature to be priority-aware of scenarios.
		 */
		if (unlikely(qos_debug_scenario)) {
			qos_dump_compare_id(cached_scen_id, scen, QOS_PRINT_LOW);
		}
		if (qos_match_scen(scen->mask_id & cached_scen_id)) {
			tmp_video_id = cached_scen_id & matching_masks.mask_video;
			if (tmp_video_id == matching_masks.video_stop &&
					qos_brain.src_status != QOS_SRC_STOP) {
				qos_brain.src_status = QOS_SRC_STOP;
				cancel_delayed_work(&qos_brain.delay_ctrl_timer);
				qos_brain.delay_timeout = false;
				qos_brain.delay_first_ctrl = false;
				qos_brain.video_in_cnt = 0;
				qos_print(QOS_PRINT_LOW, "Src stopped.\n");
			} else if (qos_brain.src_status == QOS_SRC_STOP
					&& (tmp_video_id != matching_masks.video_stop)) {
				if (!qos_brain.video_in_cnt) {
					qos_brain.video_in_cnt = 1;
				} else if (qos_brain.video_in_cnt == 1) {
					qos_brain.video_in_cnt++;
					qos_brain.src_status = QOS_SRC_IN;
					schedule_delayed_work(&qos_brain.delay_ctrl_timer,
							msecs_to_jiffies(qos_delay_ctrl_ms));
					output_level = QOS_CTRL_OUT_DISABLE;
					qos_print(QOS_PRINT_LOW, "New src, set level for "
							"the delayed to [%u] for [%s]\n", output_level,
							scen->node_name);
					qos_brain.delay_first_ctrl = true;
					WRITE_ONCE(qos_brain.matched_scen, scen);
					cpuqos_output_adjust(scen, output_level,
							QOS_OUTPUT_DELAY_INIT, NULL);
				}
			}
			qos_print(QOS_PRINT_LOW, "Enter scenario -> [%s]\n",
					scen->node_name);
			/* update current scenario */
			WRITE_ONCE(qos_brain.matched_scen, scen);
			found = true;
			break;
		}
    }
	if (unlikely(!found)) {
		qos_print(QOS_PRINT_HIGH, "No new scen found, current = %s.\n",
				qos_brain.matched_scen->node_name);
	}
}

void qos_event_detect(u32 qos_type, enum qos_event_in event, void *info)
{
	/* add more QoS type by using more "if ( qos_type == TYPE )" */
	struct qos_key_info *key;
	struct qos_video_info *v_info;
	struct qos_voice_info *voc_info;
	struct qos_in_bin_info *bin_info; /* for those only has on/off status */
	u64 new_scen_id;
	spin_lock(&qos_brain.scen_lock);
	new_scen_id = READ_ONCE(qos_brain.current_scen_id);
	if (qos_type & QOS_BIT_CPU) {
		if (unlikely((!cpuqos_is_on()) && (event < __NR_QOS_DTS_EVENT))) {
			goto CPU_END;
			/* 
			 * events from __NR_QOS_DTS_EVENT to __NR_QOS_IN_EVENTS like
			 * QOS_EVENT_AIPQ or QOS_EVENT_UCD can be modified when cpuqos
			 * is NOT on.
			 */
		}
		switch (event) {
			case QOS_EVENT_KEY:
				key = (struct qos_key_info *)info;
				cpuqos_key_detect_entry(key->key_action, key->key_button,
						&new_scen_id);
				break;
			case QOS_EVENT_VIDEO:
				v_info = (struct qos_video_info *)info;
				qos_scenario_clear(&new_scen_id, 
						GENMASK_ULL(__END_POS_QOS_VIDEO_USED - 1,
							QOS_VIDEO_START_BIT));
				qos_scenario_set(&new_scen_id, BIT(v_info->src) |
						BIT(v_info->resol) | BIT(v_info->fps) | BIT(v_info->codec));
				break;
			case QOS_EVENT_VOICE:
				voc_info = (struct qos_voice_info *)info;
				if (voc_info->is_enabled) {
					qos_scenario_set(&new_scen_id, BIT(QOS_VOICE_ON));
					qos_scenario_clear(&new_scen_id, BIT(QOS_VOICE_OFF));
				} else {
					qos_scenario_clear(&new_scen_id, BIT(QOS_VOICE_ON));
					qos_scenario_set(&new_scen_id, BIT(QOS_VOICE_OFF));
				}
				break;
			case QOS_EVENT_AIPQ:
			case QOS_EVENT_UCD:
				/*
				 * AIPQ & UCD only uses ONE bit to represent on/off 
				 * because it is not used for fast matching w/ matching_masks
				 * from DTS.
				 *
				 * That is, these bits are used to decide whether the features
				 * are enabled by the UI. If they are NOT enabled, QoS should
				 * not be able to control them.
				 */
				bin_info = (struct qos_in_bin_info *)info;
				if (bin_info->is_enabled) {
					qos_scenario_set(&new_scen_id, BIT(bin_info->event_on_bit));
				} else {
					qos_scenario_clear(&new_scen_id, BIT(bin_info->event_on_bit));
				}
				WRITE_ONCE(qos_brain.current_scen_id, new_scen_id);
				break;
			default:
				qos_print(QOS_PRINT_HIGH, "Not handled event for CPUQoS\n");
				break;
		}
	}
CPU_END:
	if (cpuqos_is_on() && (event != QOS_EVENT_KEY)) {
		/* QOS_EVENT_KEY handles scenarios by itself */
		WRITE_ONCE(qos_brain.current_scen_id, new_scen_id);
		qos_enter_scenario();
	}
	spin_unlock(&qos_brain.scen_lock);
}
EXPORT_SYMBOL(qos_event_detect);

static int qos_pm_handler (struct notifier_block *nb, unsigned long event,
    void *dummy) {
	return cpuqos_pm_handler(nb, event, dummy);
}

static struct notifier_block qos_pm_notifier = {
    .notifier_call = qos_pm_handler,
};

static int qos_reboot_handler (struct notifier_block *nb, unsigned long event,
    void *dummy) {
	return cpuqos_reboot_handler(nb, event, dummy);
}


static struct notifier_block qos_reboot_notifier = {
    .notifier_call = qos_reboot_handler,
};

void qos_src_in_stop(void)
{
	/* video stop or HDMI disconnect, etc for exiting the scenario */
	qos_enter_scenario_helper(matching_masks.mask_video,
			matching_masks.video_stop, true);
}
EXPORT_SYMBOL(qos_src_in_stop); /* make ko can use it */

int qos_get_ver(void)
{
	struct device_node *qos_dev;
	int ret;
	QOS_GET_NODE("/qos", qos_dev);
	ret = of_property_read_u32_array(qos_dev, "dts_version",
				&qos_brain.versions[QOS_DTS_MAJOR_VER_INDEX], 2);
	if (ret) {
		qos_print(QOS_PRINT_HIGH, "Parsing DTS version error.\n");
        ret = QOS_RET_FAIL;
	} else {
        ret = QOS_RET_SUCCESS;
    }
	of_node_put(qos_dev);
	return ret;
}

static int proc_video_seq_show(struct seq_file *seq, void *v)
{
    seq_printf(seq, "Used for notify video stop.\n");
    return 0;
}

static int proc_video_open(struct inode *inode, struct file *file)
{
	single_open(file, proc_video_seq_show, NULL);
    return 0;
}

static ssize_t proc_video_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
    char buffer[tmp_buffer_size] = {};
    size_t ret = count;
	int input;
    if (count >= tmp_buffer_size)
        count = tmp_buffer_size - 1;
    if (copy_from_user(buffer, buf, count)) {
        return -EFAULT;
    }
    if (sscanf(buffer, "%d", &input) != 1)
    {
        qos_print(QOS_PRINT_HIGH, "Invalid input count\n");
        return -EINVAL;
    } else {
        if (input == QOS_OP_SRC_IN_STOP) {
			qos_src_in_stop();
        } else {
			qos_print(QOS_PRINT_HIGH, "Not handled video status.\n");
        }
    }
    return ret;
}

static const struct file_operations proc_video_fops = {
    .owner      = THIS_MODULE,
	.write      = proc_video_write,
	.open       = proc_video_open,
    .release    = single_release,
    .read       = seq_read,
    .llseek     = seq_lseek,
};

QOS_ATTR_IMPL_RW(qos_debug_msg, 0)
QOS_ATTR_IMPL_RW(qos_debug_scenario, 0)
QOS_ATTR_IMPL_RW(qos_delay_ctrl_ms, 5000)

static int qos_proc_init(void)
{
	int ret = QOS_RET_SUCCESS;
	struct proc_dir_entry *proc_qos_dir, *entry;
	proc_qos_dir = proc_mkdir("qos", NULL);
	if (!proc_qos_dir) {
		qos_print(QOS_PRINT_HIGH, "Create /proc/qos failed.\n");
		return -ENOMEM;
	}
	/* proc w/ int type */
	QOS_ATTR_INIT_RW(entry, qos_debug_msg, proc_qos_dir);
	QOS_ATTR_INIT_RW(entry, qos_debug_scenario, proc_qos_dir);
	QOS_ATTR_INIT_RW(entry, qos_delay_ctrl_ms, proc_qos_dir);

	/* proc need to customize the fops */
	QOS_ATTR_CREATE(entry, video_op, proc_qos_dir, proc_video_fops);
	ret = cpuqos_proc_init(proc_qos_dir);

	return ret;
}

static void qos_genmask_match(void)
{
	matching_masks.mask_src =
		GENMASK_ULL(__END_POS_QOS_SRC - 1, QOS_SRC_IGNORE);
	matching_masks.mask_resol =
		GENMASK_ULL(__END_POS_QOS_RESOLUTION - 1, QOS_RESOL_IGNORE);
	matching_masks.mask_fps =
		GENMASK_ULL(__END_POS_QOS_FPS - 1, QOS_FPS_IGNORE);
	matching_masks.mask_codec =
		GENMASK_ULL(__END_POS_QOS_CODEC - 1, QOS_CODEC_IGNORE);
	matching_masks.mask_key =
		GENMASK_ULL(__END_POS_IR_KEY_USED - 1, QOS_IR_KEY_SET);
	matching_masks.mask_voc =
		GENMASK_ULL(__END_POS_VOICE_USED - 1, QOS_VOICE_ON);

	/* special masks */
	matching_masks.mask_video = GENMASK_ULL(__END_POS_QOS_VIDEO_USED - 1,
				QOS_VIDEO_START_BIT);
	matching_masks.video_stop = BIT(QOS_SRC_NONE) | BIT(QOS_RESOL_NONE)
		| BIT(QOS_FPS_NONE)	| BIT(QOS_CODEC_NONE);
}

void qos_enter_scenario_helper(u32 clear_bits, u32 set_bits, bool need_lock)
{
    u64 new_scen_id;
    if (need_lock) {
        spin_lock(&qos_brain.scen_lock);
    }
    new_scen_id = READ_ONCE(qos_brain.current_scen_id);
    qos_scenario_clear(&new_scen_id, clear_bits);
    qos_scenario_set(&new_scen_id, set_bits);
    WRITE_ONCE(qos_brain.current_scen_id, new_scen_id);
    qos_enter_scenario();
    if (need_lock) {
        spin_unlock(&qos_brain.scen_lock);
    }
}

static int qos_init_cb_helper(void)
{
	int ret, i;
	struct device_node *qos_dev;
    QOS_GET_NODE("/qos", qos_dev);
    ret = of_property_read_u32_array(qos_dev, "ctrl_reg_opt",
                qos_brain.ctrl_reg_opt, __NR_OUTPUT_MAP_TYPE);
    if (ret) {
        qos_print(QOS_PRINT_HIGH, "Parsing DTS ctrl_reg_opt error, all set to 0.\n");
        ret = QOS_RET_FAIL;
		for (i = 0; i < __NR_OUTPUT_MAP_TYPE; i++) {
			qos_brain.ctrl_reg_opt[i] = 0;
        }
    }
    ret = QOS_RET_SUCCESS; /* always return success, reg is not mandatary */
	return ret;
}

static void qos_delay_ctrl_handler(struct work_struct *w)
{
	qos_brain.delay_timeout = true;
}

static int __init qos_init(void)
{
	int ret = 0;
	INIT_LIST_HEAD(&qos_brain.qos_cb_heads);
	qos_brain.current_scen_id = 0;
	qos_brain.src_status = QOS_SRC_STOP;
	qos_brain.video_in_cnt = 0; /* init for the first play */
	qos_brain.delay_timeout = false;
	qos_brain.delay_first_ctrl = false;
	INIT_DELAYED_WORK(&qos_brain.delay_ctrl_timer, qos_delay_ctrl_handler);
	mutex_init(&qos_brain.cb_list_lock);
	spin_lock_init(&qos_brain.scen_lock);
    /* register suspension & reboot handler */
    register_pm_notifier(&qos_pm_notifier);
    register_reboot_notifier(&qos_reboot_notifier);

	qos_genmask_match();
	ret = qos_get_ver();
    if (ret) {
        qos_print(QOS_PRINT_HIGH, "Parsing version error at %s:%d.\n",
                __FILE__, __LINE__);
    }

	ret = qos_init_cb_helper();
    if (ret) {
        qos_print(QOS_PRINT_HIGH, "Parsing necessities of cb registery error at %s: %d.\n",
                __FILE__, __LINE__);
    }

	ret = qos_proc_init();
    if (ret) {
        qos_print(QOS_PRINT_HIGH, "Initializing qos in sysfs error at %s: %d.\n",
                __FILE__, __LINE__);
    }

	ret = cpuqos_init();
	if (ret) {
		qos_print(QOS_PRINT_HIGH, "cpuqos_init() failed at %s: %d.\n",
                __FILE__, __LINE__);
	}
    qos_init_done = true;
	return ret;
}

module_init(qos_init);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("MP_QOS");
MODULE_LICENSE("GPL");
