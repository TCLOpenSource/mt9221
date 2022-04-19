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
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kthread.h>
#include "mdrv_qos_drv.h"
#include "cpuqos_drv.h"

static int proc_cpu_init_seq_show(struct seq_file *seq, void *v)
{
    seq_printf(seq, "cpuqos_on=%d; echo 2 > /proc/qos/qos_debug_msg for more.\n",
			cpuqos_is_on());
	cpuqos_print_parsed_dts();
    return 0;
}

static int proc_cpu_init_open(struct inode *inode, struct file *file)
{
    single_open(file, proc_cpu_init_seq_show, NULL);
    return 0;
}

static int proc_cpu_init_release(struct inode *inode, struct file * file)
{
	single_release(inode, file);
    return 0;
}

static int cpuqos_runtime_init(void *arg)
{
	if (cpuqos_parse_dts() == CPUQOS_RET_SUCCESS) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "[notice] some config. "
                "parameters in /proc/qos/cpu/* should not be changed "
                "after initialized.\ne.g. %s, %s\n",
                "register_scan_period_ms", "cpu_load_period_ms");
    } else {
        cpuqos_print(CPUQOS_PRINT_HIGH, "Some dts config is missing\n");
    }
	return CPUQOS_RET_SUCCESS;
}

static ssize_t proc_cpu_init_write(struct file *file, const char __user *buf,
        size_t count, loff_t *ppos)
{
	const int buf_len = 8;
    char buffer[buf_len];
	int input;
    size_t ret = count;
	struct task_struct *thread;

    if (count >= buf_len) {
		cpuqos_print(CPUQOS_PRINT_HIGH, "Invalid input length, max=%u\n",
				buf_len - 1);
        return -EINVAL;
	}

    if (copy_from_user(buffer, buf, count)) {
        return -EFAULT;
    }

    if (sscanf(buffer, "%d", &input) != 1)
    {
		cpuqos_print(CPUQOS_PRINT_HIGH, "Invalid input count\n");
        return -EINVAL;
    } else {
		if (input == 1) {
			if (cpuqos_is_on()) {
				cpuqos_print(CPUQOS_PRINT_HIGH, "CPUQoS is already inited.\n");
			} else {
				thread = kthread_run(cpuqos_runtime_init, NULL, "M_QualityInit");
				if (IS_ERR(thread)) {
					cpuqos_print(CPUQOS_PRINT_HIGH,
							"Create M_QualityInit thread failed.\n");
				}
			}
		} else {
			cpuqos_set_off();
		}
    }
    return ret;
}

static const struct file_operations cpu_init_fops = {
    .owner      = THIS_MODULE,
    .open       = proc_cpu_init_open,
    .write      = proc_cpu_init_write,
    .release    = proc_cpu_init_release,
    .read       = seq_read,
    .llseek     = seq_lseek,
};

QOS_ATTR_IMPL_RW(cpu_load_period_ms, 250)
QOS_ATTR_IMPL_RW(cpu_load_release_diff, 10)
QOS_ATTR_IMPL_RW(register_scan_period_ms, 30000)
QOS_ATTR_IMPL_RW(key_detect_lasting_ms, CPUQOS_ANDROID_LONG_PRESS_MS)
QOS_ATTR_IMPL_RW(cpu_ctrl_flip_ms, 500)

static int proc_cpuqos_load_seq_show(struct seq_file *seq, void *v)
{
    /* show the current cpu loading, maximum is 100 */
    seq_printf(seq, "%d\n", atomic_read(&cpuqos_cpu_load));
    return 0;
}

static int proc_cpuqos_load_open(struct inode *inode, struct file *file)
{
    single_open(file, proc_cpuqos_load_seq_show, NULL);
    return 0;
}

static const struct file_operations cpu_cpuload_fops = {
    .owner      = THIS_MODULE,
    .open       = proc_cpuqos_load_open,
	.release    = single_release,
    .read       = seq_read,
    .llseek     = seq_lseek,
};

static int proc_cpuqos_off_seq_show(struct seq_file *seq, void *v)
{
    seq_printf(seq, "cpuqos_on=%d\n", cpuqos_is_on());
    return 0;
}

static ssize_t proc_cpuqos_off_write(struct file *file, const char __user *buf,
        size_t count, loff_t *ppos)
{
    char buffer[tmp_buffer_size] = {};
    size_t ret = count;
    if (count >= tmp_buffer_size)
        count = tmp_buffer_size - 1;
    if (copy_from_user(buffer, buf, count)) {
        return -EFAULT;
    }
	cpuqos_set_off();
    return ret;
}

static int proc_cpuqos_off_open(struct inode *inode, struct file *file)
{
    single_open(file, proc_cpuqos_off_seq_show, NULL);
    return 0;
}

static const struct file_operations cpu_off_fops = {
    .owner      = THIS_MODULE,
    .write      = proc_cpuqos_off_write,
	.open       = proc_cpuqos_off_open,
	.release    = single_release,
    .read       = seq_read,
    .llseek     = seq_lseek,
};


static int proc_cpuqos_ver_seq_show(struct seq_file *seq, void *v)
{
    seq_printf(seq, "ver. %d.%d for CPUQoS code base.\n"
			"ver. %d.%d for CPUQoS DTS setting.\n",
			qos_brain.versions[QOS_SRC_MAJOR_VER_INDEX],
			qos_brain.versions[QOS_SRC_MINOR_VER_INDEX],
			qos_brain.versions[QOS_DTS_MAJOR_VER_INDEX],
			qos_brain.versions[QOS_DTS_MINOR_VER_INDEX]);
    return 0;
}

static int proc_cpuqos_ver_open(struct inode *inode, struct file *file)
{
    single_open(file, proc_cpuqos_ver_seq_show, NULL);
    return 0;
}

static const struct file_operations cpu_version_fops = {
    .owner      = THIS_MODULE,
	.open       = proc_cpuqos_ver_open,
	.release    = single_release,
    .read       = seq_read,
    .llseek     = seq_lseek,
};

static int proc_cpuqos_curr_scen_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "\n%10s -> "U64_TO_BINARY_PATTERN"\n", "curr_id",
			U64_TO_BINARY_ARGS(qos_brain.current_scen_id));
	qos_dump_masks(QOS_PRINT_HIGH);
    return 0;
}

static int proc_cpuqos_curr_scen_open(struct inode *inode, struct file *file)
{
    single_open(file, proc_cpuqos_curr_scen_show, NULL);
    return 0;
}

static const struct file_operations cpu_curr_scen_fops = {
    .owner      = THIS_MODULE,
	.open       = proc_cpuqos_curr_scen_open,
	.release    = single_release,
    .read       = seq_read,
    .llseek     = seq_lseek,
};

static ssize_t event_read(struct file *filp, char __user *buf, size_t len,
		loff_t *off)
{
    ssize_t ret;
	struct qos_poll *poll;
	poll = &qos_brain.cpu_meta.poll;
    if (copy_to_user(buf, poll->read_buf, poll->read_len)) {
        ret = -EFAULT;
    } else {
        ret = poll->read_len;
    }
    /* data gets drained once a reader reads from it. */
    poll->read_len = 0;
    return ret;
}

unsigned int event_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct qos_poll *poll;
	poll = &qos_brain.cpu_meta.poll;
    poll_wait(filp, &poll->wq, wait);
    if (poll->read_len) {
        return POLLIN;
	}
	return POLLERR;
}

static ssize_t proc_poll_write(struct file *file, const char __user *buf,
        size_t count, loff_t *ppos)
{
    char buffer[tmp_buffer_size] = {};
	int input;
    size_t ret = count;
    if (count >= tmp_buffer_size)
        count = tmp_buffer_size - 1;
    if (copy_from_user(buffer, buf, count)) {
        return -EFAULT;
    }
    if (sscanf(buffer, "%d", &input) != 1)
    {
		cpuqos_print(CPUQOS_PRINT_HIGH, "Invalid input count\n");
        return -EINVAL;
    } else {
		if (input == 1) {
			qos_brain.cpu_meta.poll.is_registered = true;
			cpuqos_print(CPUQOS_PRINT_LOW, "poll.is_registered=%d\n",
					qos_brain.cpu_meta.poll.is_registered);
		} else {
			qos_brain.cpu_meta.poll.is_registered = false;
		}
	}
    return ret;
}

static const struct file_operations cpu_event_fops = {
    .owner = THIS_MODULE,
    .read = event_read,
    .poll = event_poll,
    .write = proc_poll_write,
};

int cpuqos_proc_init(struct proc_dir_entry *parent_dir)
{
    struct proc_dir_entry *proc_cpu_dir, *entry;
	proc_cpu_dir = proc_mkdir("cpu", parent_dir);
    if (!proc_cpu_dir) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "proc_mkdir(cpu) failed\n");
        return -ENOMEM;
    }

	/* proc need to customize the fops */
	QOS_ATTR_CREATE(entry, init, proc_cpu_dir, cpu_init_fops);
	QOS_ATTR_CREATE(entry, cpu_load, proc_cpu_dir, cpu_cpuload_fops);
	QOS_ATTR_CREATE(entry, off, proc_cpu_dir, cpu_off_fops);
	QOS_ATTR_CREATE(entry, version, proc_cpu_dir, cpu_version_fops);
	QOS_ATTR_CREATE(entry, current_scenario, proc_cpu_dir, cpu_curr_scen_fops);
	QOS_ATTR_CREATE(entry, event_notifier, proc_cpu_dir, cpu_event_fops);

	/* proc w/ int type */
	QOS_ATTR_INIT_RW(entry, cpu_load_period_ms, proc_cpu_dir);
	QOS_ATTR_INIT_RW(entry, cpu_load_release_diff, proc_cpu_dir);
	QOS_ATTR_INIT_RW(entry, register_scan_period_ms, proc_cpu_dir);
	QOS_ATTR_INIT_RW(entry, key_detect_lasting_ms, proc_cpu_dir);
	QOS_ATTR_INIT_RW(entry, cpu_ctrl_flip_ms, proc_cpu_dir);

	return 0;
}

