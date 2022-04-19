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
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>
#include <linux/input-event-codes.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
#include <linux/sched/signal.h>
#endif

#include "cpuqos_drv.h"

static bool cpuqos_on = 0;

bool cpuqos_is_on(void)
{
	return cpuqos_on;
}

static inline void cpuqos_update_key_expire_timer(int ms)
{
	if (ms == INT_MAX) {
		/**
		 * MAX_JIFFY_OFFSET means 'infinite timeout'
		 * check msecs_to_jiffies's comment for more.
		 */
		mod_timer(&qos_brain.cpu_meta.monitor_key_timer, MAX_JIFFY_OFFSET);
	} else {
		mod_timer(&qos_brain.cpu_meta.monitor_key_timer,
				jiffies + msecs_to_jiffies(ms));
	}
}

static void cpuqos_send_poll_message(struct cpuqos_meta *ctrl, u32 level)
{
	ctrl->poll.read_len = snprintf(ctrl->poll.read_buf,
			sizeof(ctrl->poll.read_buf), "%u\n", level);
	wake_up(&ctrl->poll.wq);
    cpuqos_print(CPUQOS_PRINT_LOW, "EPOLL -> %u\n", level);
}

static void cpuqos_force_exit_detect(void);
void cpuqos_set_off(void)
{
	cpuqos_force_exit_detect();
	cpuqos_on = false;
	cpuqos_print(CPUQOS_PRINT_HIGH, "cpuqos_on=%d, exit "
			"(cannot be re-init again)\n", cpuqos_on);
}

static void cpuqos_remove_pid_threads(struct cpuqos_pids *pid)
{
	/* remove all threads */
	struct cpuqos_tids *tid;
	struct cpuqos_tids *tmp_tid;
	if (pid) {
		if (!list_empty(&pid->tid_heads)) {
			list_for_each_entry_safe(tid, tmp_tid, &pid->tid_heads,
					tid_head) {
				list_del(&tid->tid_head);
				kfree(tid);
			}
		}
	}
}

static void __cpuqos_add_pid(struct cpuqos_registry *reg, pid_t pid)
{
    struct cpuqos_pids *pids = kzalloc(sizeof(*pids), GFP_ATOMIC);
	pids->pid = pid;
	pids->reg = reg;
	INIT_LIST_HEAD(&pids->tid_heads);
	list_add_tail(&pids->pid_head, &reg->pid_heads);
}

#define prev_task(p) \
    list_entry_rcu((p)->tasks.prev, struct task_struct, tasks)

int cpuqos_build_pids(struct cpuqos_registry *reg)
{
    struct task_struct *proc;
	char *proc_comm = reg->proc_comm;
	u32 count = 0;
    /* try to find the process */
    rcu_read_lock(); /* protects the comm & thread iteration */
    for_each_process (proc) {
        if (proc) {
			get_task_struct(proc);
			task_lock(proc);
			reg->last_searched_pid = proc->pid;
			if (!strncmp(proc->comm, proc_comm, TASK_COMM_LEN)) {
				__cpuqos_add_pid(reg, proc->pid);
				count++;
				if (count >= reg->pid_num) {
					task_unlock(proc);
					put_task_struct(proc);
					break;
				}
			}
			task_unlock(proc);
			put_task_struct(proc);
        }
    }
    rcu_read_unlock();
	if (list_empty(&reg->pid_heads)) {
		cpuqos_print(CPUQOS_PRINT_HIGH, "Unable to find comm=%s\n", proc_comm);
		/* if we cannot find it now, we can search it later in daemon. */
	}
	return CPUQOS_RET_SUCCESS;
}

static void __cpuqos_add_tid(struct cpuqos_pids *pid, pid_t tid)
{
    struct cpuqos_tids *tids = kzalloc(sizeof(*tids), GFP_ATOMIC);
	tids->tid = tid;
	tids->pid = pid;
	list_add_tail(&tids->tid_head, &pid->tid_heads);
}

int cpuqos_build_tids(struct cpuqos_registry *reg)
{
    /* thread_comm is the sub thread of proc_comm */
    struct task_struct *proc, *thread;
	struct cpuqos_pids *pids;
	u32 count = 0;
	struct pid *pid;
	rcu_read_lock();
	list_for_each_entry(pids, &reg->pid_heads, pid_head) {
		pid = find_get_pid(pids->pid);
		proc = get_pid_task(pid, PIDTYPE_PID);
		/* try to find all the threads in the process */
		if (proc) {
			task_lock(proc);
			for_each_thread(proc, thread) {
				if (thread) {
					get_task_struct(thread);
					if (!strncmp(thread->comm, reg->thread_comm,
								TASK_COMM_LEN)) {
						__cpuqos_add_tid(pids, thread->pid);
						count++;
						if (count >= reg->tid_num) {
							break;
						}
					}
					put_task_struct(thread);
				}
			}
			task_unlock(proc);
			put_task_struct(proc);
		}
		put_pid(pid);
		if (list_empty(&pids->tid_heads)) {
			cpuqos_print(CPUQOS_PRINT_LOW, "Unable to find thread_comm=%s\n",
					reg->thread_comm);
			/* if we cannot find it now, we can search it later in daemon. */
		}
	}
	rcu_read_unlock();
	return CPUQOS_RET_SUCCESS;
}

extern int pid_max;
static void cpuqos_update_processes(struct cpuqos_registry *reg,
		bool *write_pid)
{
	struct cpuqos_pids *pids, *tmp_pid;
    struct task_struct *proc = &init_task, *prev_proc;
	bool is_different = false, is_added, first_iter = true, found = false;
	u32 alive_cnt = 0, wrap_around = 0, prev_alive;
	pid_t begin_pid, sec_pid;
	struct pid *pid;
	bool type_1 = true, type_2 = false;
	/* check whether the # of proc matches */
    rcu_read_lock();
	list_for_each_entry_safe(pids, tmp_pid, &reg->pid_heads, pid_head) {
		pid = find_get_pid(pids->pid);
		proc = get_pid_task(pid, PIDTYPE_PID);
		if (proc) {
			task_lock(proc);
			if (!strncmp(proc->comm, reg->proc_comm, TASK_COMM_LEN)) {
				alive_cnt++;
			}
			task_unlock(proc);
			put_task_struct(proc);
		} else {
			is_different = true;
			/* remove the pid */
			cpuqos_remove_pid_threads(pids);
			list_del(&pids->pid_head);
			kfree(pids);
		}
		put_pid(pid);
	}
	rcu_read_unlock();
	if ((alive_cnt >= reg->pid_num) && (!is_different)) {
		cpuqos_print(CPUQOS_PRINT_LOW, "No update for proc=%s\n",
				reg->proc_comm);
		return; /* save time ^^ */
	}
	/* 
	 * pid increases until it meet pid_max, and then it will wrap around.
	 * Searching from the next (last known pid) to remove redundant searching.
	 */
	begin_pid = reg->last_searched_pid;
	sec_pid = next_task(&init_task)->pid; /* sec_pid = 1 */
	prev_alive = alive_cnt;
    rcu_read_lock();
	pid = find_get_pid(begin_pid);
	proc = get_pid_task(pid, PIDTYPE_PID);
	if (proc) {
		/* found it */
		task_lock(proc);
		put_pid(pid);
	} else {
		put_pid(pid);
		begin_pid = sec_pid;
		/* sec_pid will never be killed, no need to check & get_pid */
		proc = get_pid_task(find_vpid(begin_pid), PIDTYPE_PID);
        if (likely(proc)) {
            task_lock(proc);
        } else {
            cpuqos_print(CPUQOS_PRINT_HIGH, "[Error] Failed to find the begin pid.\n");
            rcu_read_unlock();
            *write_pid = false;
            return;
        }
	}
	cpuqos_print(CPUQOS_PRINT_LOW, "Start search pid from %d "
			"for proc_comm=%s, search from comm=%s, before=%u/%u\n",
			begin_pid, reg->proc_comm, proc->comm, alive_cnt, reg->pid_num);

	while (proc && (type_1 || type_2)) {
		/* find the missing # of (reg->proc_num - alive_cnt) pids. */
		if (likely(!first_iter)) {
			get_task_struct(proc);
			task_lock(proc);
		} else {
			first_iter = false;
		}
		if (!strncmp(proc->comm, reg->proc_comm, TASK_COMM_LEN)) {
			is_added = false;
			list_for_each_entry_safe(pids, tmp_pid, &reg->pid_heads,
					pid_head) {
				if (pids->pid == proc->pid) {
					is_added = true;
					break;
				}
			}
			/* do not add the already found pid */
			if (!is_added) {
				__cpuqos_add_pid(reg, proc->pid);
				cpuqos_print(CPUQOS_PRINT_LOW, "Found pid=%d\n",
						proc->pid);
				alive_cnt++;
				reg->last_searched_pid = proc->pid;
				found = true;
			}
			if (alive_cnt >= reg->pid_num) {
				task_unlock(proc);
				put_task_struct(proc);
				break;
			}
		}
		if (unlikely(proc->pid == init_task.pid)) {
			/* meet the "init" process, which pid=0 */
			wrap_around = 1;
			cpuqos_print(CPUQOS_PRINT_LOW, "Wrap. init=%d, begin_pid=%d\n",
					init_task.pid, begin_pid);
		}
		if (begin_pid == sec_pid) {
			/* type_1 */
			type_1 = (wrap_around == 0);
			type_2 = false;
		} else {
			/* type_2 */
			type_1 = false;
			type_2 = ((proc->pid >= begin_pid) && (wrap_around == 0)) ||
				((proc->pid < begin_pid) && (wrap_around != 0));
		}
		prev_proc = proc;
		proc = next_task(proc);
		task_unlock(prev_proc);
		put_task_struct(prev_proc);
    }
	/* update the search record */
	if (!found) {
        *write_pid = false;
		reg->last_searched_pid = prev_task(&init_task)->pid;
	} else {
        *write_pid = true;
    }
    rcu_read_unlock();
	if (prev_alive == alive_cnt) {
		cpuqos_print(CPUQOS_PRINT_LOW, "PID found nothing new for %s.\n",
				reg->proc_comm);
	}
}

static void cpuqos_update_threads(struct cpuqos_registry *reg,
		bool *write_tid)
{
	struct cpuqos_pids *pids, *tmp_pid;
	struct cpuqos_tids *tids, *tmp_tid;
    struct task_struct *proc, *thread;
	bool is_different = false;
	u32 alive_cnt = 0, prev_alive;
	struct pid *pid, *tid;
	/* check whether the # of thread matches */
    rcu_read_lock();
	list_for_each_entry_safe(pids, tmp_pid, &reg->pid_heads, pid_head) {
		pid = find_get_pid(pids->pid);
		proc = get_pid_task(pid, PIDTYPE_PID);
		/* comm is already checked in cpuqos_update_processes() */
		if (proc) {
			task_lock(proc); /* prevent the thread being killed */
			list_for_each_entry_safe(tids, tmp_tid, &pids->tid_heads,
					tid_head) {
				tid = find_get_pid(tids->tid);
				thread = get_pid_task(tid, PIDTYPE_PID);
				if (thread) {
					if (!strncmp(thread->comm, reg->thread_comm,
								TASK_COMM_LEN)) {
						alive_cnt++;
					}
					put_task_struct(thread);
				} else {
					is_different = true;
					/* remove the tid */
					list_del(&tids->tid_head);
					kfree(tids);
				}
				put_pid(tid);
			}
			task_unlock(proc);
			put_task_struct(proc);
		} else {
			is_different = true;
			/* 
			 * remove the pid
			 * if there is a new pid for the thread, wait to next round
			 */
			cpuqos_remove_pid_threads(pids);
			list_del(&pids->pid_head);
			kfree(pids);
		}
		put_pid(pid);
	}
	rcu_read_unlock();
	if ((alive_cnt >= reg->tid_num) && (!is_different)) {
		cpuqos_print(CPUQOS_PRINT_LOW, "No update for thread=%s\n",
				reg->thread_comm);
		return; /* save time ^^ */
	}
	prev_alive = alive_cnt;
	list_for_each_entry(pids, &reg->pid_heads, pid_head) {
		/* add */
		rcu_read_lock();
		pid = find_get_pid(pids->pid);
		proc = get_pid_task(pid, PIDTYPE_PID);
		/* try to find all the threads in the process */
		if (proc) {
			task_lock(proc); /* protect threads */
			cpuqos_print(CPUQOS_PRINT_LOW, "Search for thread_comm=%s in "
					"proc comm=%s, pid=%d\n", reg->thread_comm,
					reg->proc_comm, pids->pid);
			for_each_thread(proc, thread) {
				if (thread) {
					get_task_struct(thread);
					if (!strncmp(thread->comm, reg->thread_comm,
								TASK_COMM_LEN)) {
						is_different = true;
						list_for_each_entry(tids, &pids->tid_heads,
								tid_head) {
							if (tids->tid == thread->pid) {
								is_different = false;
								put_task_struct(thread);
								break;
							}
						}
						if (is_different) {
							/* add */
							__cpuqos_add_tid(pids, thread->pid);
							cpuqos_print(CPUQOS_PRINT_LOW, "Found tid=%d\n",
									thread->pid);
							alive_cnt++;
						}
						if (alive_cnt >= reg->tid_num) {
							put_task_struct(thread);
							break;
						}
					}
					put_task_struct(thread);
				}
			}
			task_unlock(proc);
			put_task_struct(proc);
		}
		put_pid(pid);
		rcu_read_unlock();
	}
	if (prev_alive == alive_cnt) {
		cpuqos_print(CPUQOS_PRINT_LOW, "TID found nothing new for %s\n",
				reg->thread_comm);
	}
	*write_tid = true;
}

static int __cpuqos_write_multi_pids(struct cpuqos_registry *reg)
{
	struct cpuqos_pids *pid;
	int count = 0;
	char *buf, *tmp_buf;
	int ret;
	if (list_empty(&reg->pid_heads)) {
		cpuqos_print(CPUQOS_PRINT_HIGH, "Unable to find proc_comm=%s\n",
				reg->proc_comm);
		return CPUQOS_RET_SUCCESS; /* it may be a normal brhavior */
	} else {
		list_for_each_entry(pid, &reg->pid_heads, pid_head) {
			count++;
		}
		buf = tmp_buf = kzalloc(CPUQOS_PID_LEN*count, GFP_KERNEL);
		if (!buf) {
			cpuqos_print(CPUQOS_PRINT_HIGH, "kzalloc fail in "
					"__cpuqos_write_multi_pids()\n");
			return CPUQOS_RET_FAIL;
		}
		list_for_each_entry(pid, &reg->pid_heads, pid_head) {
			snprintf(buf, CPUQOS_PID_LEN, "%d", pid->pid);
			buf += CPUQOS_PID_LEN;
		}
		ret = qos_multi_write_path(reg->write_path, tmp_buf, count,
				CPUQOS_PID_LEN);
		kfree(tmp_buf);
	}
	return ret;
}

static int __cpuqos_write_multi_tids(struct cpuqos_registry *reg)
{
	int ret;
	struct cpuqos_pids *pid;
	struct cpuqos_tids *tid;
	int count = 0;
	char *buf, *tmp_buf;
	if (list_empty(&reg->pid_heads)) {
		cpuqos_print(CPUQOS_PRINT_HIGH,
				"Unable to find proc_comm=%s for thread\n", reg->proc_comm);
		return CPUQOS_RET_SUCCESS; /* it may be a normal brhavior */
	}
	list_for_each_entry(pid, &reg->pid_heads, pid_head) {
		if (list_empty(&reg->pid_heads)) {
			cpuqos_print(CPUQOS_PRINT_HIGH, "Unable to find thread_comm=%s, "
					"but pid=%d\n",
					reg->thread_comm, pid->pid);
		}
		list_for_each_entry(tid, &pid->tid_heads, tid_head) {
			count++;
		}
	}
	buf = tmp_buf = kzalloc(CPUQOS_PID_LEN*count, GFP_KERNEL);
	if (!buf) {
		cpuqos_print(CPUQOS_PRINT_HIGH, "kzalloc fail in "
				"__cpuqos_write_multi_tids()\n");
		return CPUQOS_RET_FAIL;
	}
	list_for_each_entry(pid, &reg->pid_heads, pid_head) {
		list_for_each_entry(tid, &pid->tid_heads, tid_head) {
			snprintf(buf, CPUQOS_PID_LEN, "%d", tid->tid);
			buf += CPUQOS_PID_LEN;
		}
	}
	ret = qos_multi_write_path(reg->write_path, tmp_buf, count,
			CPUQOS_PID_LEN);
	kfree(tmp_buf);
	return ret;
}

static int cpuqos_write_multi_pid_tid(struct cpuqos_registry *reg,
		bool write_pid, bool write_tid)
{
	int ret = 0;
	if (reg->reg_type & BIT(CPUQOS_REG_PID)) {
		if ((reg->reg_type & BIT(CPUQOS_REG_TID)) && write_tid) {
			/* tid */
			ret = __cpuqos_write_multi_tids(reg);
		} else if ((reg->reg_type & BIT(CPUQOS_REG_PID)) && write_pid){
			/* pid */
			ret = __cpuqos_write_multi_pids(reg);
		}
	}
	return ret;
}

static int __cpuqos_update_registry(struct cpuqos_registry *reg)
{
	bool write_pid = false, write_tid = false;
	if (reg->reg_type & BIT(CPUQOS_REG_OTHER)) {
		return qos_write_path(reg->write_path, reg->val_string);
	}
	if (reg->reg_type & BIT(CPUQOS_REG_PID)) {
		cpuqos_update_processes(reg, &write_pid);
	}
	if (reg->reg_type & BIT(CPUQOS_REG_TID)) {
		cpuqos_update_threads(reg, &write_tid);
	}
	return cpuqos_write_multi_pid_tid(reg, write_pid, write_tid);
}

static void cpuqos_update_registry(struct cpuqos_meta *ctrl)
{
    struct cpuqos_registry *curr_reg;
    int ret = CPUQOS_RET_SUCCESS;
	/* keep pid or tid correct */
	if (likely(qos_brain.cpu_meta.cpuqos_pm_ready)) {
		list_for_each_entry(curr_reg,
				&qos_brain.cpu_meta.dts_heads->registry_head, head) {
			ret = __cpuqos_update_registry(curr_reg);
			if (ret != CPUQOS_RET_SUCCESS) {
				cpuqos_print(CPUQOS_PRINT_HIGH, "Update registry failed.\n");
				cpuqos_print_reg(curr_reg);
				cpuqos_print(CPUQOS_PRINT_LOW, "===========================\n");
			}
		}
	}
}

static int cpuqos_maintainer(void *arg)
{
	u64 prev_used = -1;
	u64 prev_total = -1;
	u64 register_elapsed_us;
	int max_cnt, current_cnt = 0;
	struct timeval tv_1, tv_2;
	/* prevent the first cpu_load is incorrect */
	cpuqos_get_cpu_load_once(&prev_used, &prev_total);
	/* changes for register_scan_period_ms after initialized will never work */
	max_cnt = register_scan_period_ms / cpu_load_period_ms;
	while (cpuqos_is_on()) {
		cpuqos_update_cpu_load(&prev_used, &prev_total);
		if (current_cnt >= max_cnt) {
			do_gettimeofday(&tv_1);
			cpuqos_update_registry(&qos_brain.cpu_meta);
			do_gettimeofday(&tv_2);
			register_elapsed_us = (tv_2.tv_sec * 1000000 + tv_2.tv_usec) -
					(tv_1.tv_sec * 1000000 + tv_1.tv_usec);
			cpuqos_print(CPUQOS_PRINT_LOW, "Update registries takes "
					"%llu us.\n", register_elapsed_us);
			if (likely(cpu_load_period_ms > (register_elapsed_us >> 10))) {
				msleep(cpu_load_period_ms - (register_elapsed_us >> 10));
			}
			current_cnt = 1;
			continue;
		}
		msleep(cpu_load_period_ms);
		current_cnt++;
	}
	return CPUQOS_RET_SUCCESS;
}

static void cpuqos_key_schedule_timer(struct cpuqos_meta *ctrl)
{
	if (likely(ctrl->key_detect_switch == CPUQOS_KEY_IS_NOT_DETECTED)) {
		ctrl->key_detect_switch = CPUQOS_KEY_IS_DETECTED;
		if (unlikely(timer_pending(&ctrl->monitor_key_timer))) {
			del_timer(&ctrl->monitor_key_timer);
			cpuqos_print(CPUQOS_PRINT_HIGH, "First key-detect after resume.\n");
		}
		ctrl->monitor_key_timer.expires = MAX_JIFFY_OFFSET;
		add_timer(&ctrl->monitor_key_timer);
		cpuqos_print(CPUQOS_PRINT_LOW, "Start key detection.\n");
	} else {
		cpuqos_print(CPUQOS_PRINT_HIGH, "ctrl->key_detect_switch=%d is wrong.\n",
				ctrl->key_detect_switch);
	}
}

static void __cpuqos_exit_key_helper(void)
{
    unsigned long flags;
	cpuqos_print(CPUQOS_PRINT_LOW, "Exit key detection.\n");
    spin_lock_irqsave(&qos_brain.cpu_meta.key_status_lock, flags);
	qos_brain.cpu_meta.key_detect_switch = CPUQOS_KEY_IS_NOT_DETECTED;
	qos_enter_scenario_helper(BIT(QOS_IR_KEY_SET), BIT(QOS_IR_KEY_NONE), true);
    spin_unlock_irqrestore(&qos_brain.cpu_meta.key_status_lock, flags);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
static void cpuqos_key_exit_handler(struct timer_list *timer)
#else
static void cpuqos_key_exit_handler(unsigned long val)
#endif
{
	__cpuqos_exit_key_helper();
}

static inline u32 cpuqos_output_level_calc(bool is_same_scen, u32 output_level,
		int cpu_load, int cpu_load_threshold, struct cpuqos_scenario *scen)
{
	u32 minus;
	if (is_same_scen) {
		/* the same scenario as the last */
		if (cpu_load > cpu_load_threshold) {
			/* CPU loading is too heavy */
			if (output_level > QOS_CTRL_OUT_DISABLE) {
				/* Drop quality faster than restore quality */
				minus = output_level >= QOS_CTRL_OUT_LEVEL_2 ? 2 : 1;
				output_level -= minus;
				cpuqos_print(CPUQOS_PRINT_HIGH, "Limit ctrl_level=%u for"
						" scen = %s, load %d > %d.\n", output_level,
						scen->node_name, cpu_load, cpu_load_threshold);
			}
		} else if (cpu_load < (cpu_load_threshold - cpu_load_release_diff)) {
			/* CPU loading is relaxed enough */
			if (output_level < QOS_CTRL_OUT_ENABLE) {
				output_level += 1;
				cpuqos_print(CPUQOS_PRINT_HIGH, "Release ctrl_level=%u for"
						" scen = %s, load %d < (%d-%d).\n", output_level,
						scen->node_name, cpu_load, cpu_load_threshold,
                        cpu_load_release_diff);
			}
		}
	} else {
		/* 
		 * As the prev_last scenario or enter a new scenario.
		 * Already restore or reset the output_level.
		 * ex. pressed IR-key and release it.
		 * ex. enter a different scenario that is not in history.
		 * DO NOTHING.
		 */
	}
	return output_level;
}

static inline void __cpuqos_do_cb(struct cpuqos_scenario *matched_scen,
		u32 output_level, int i, bool always_do)
{
	if (matched_scen->outputs[i]->cb &&
			((qos_brain.cpuqos_prev_mapped_output[i] !=
			 matched_scen->outputs[i]->output_set[output_level]) || always_do)) {
		/* 
		 * Make sure the cb is registered during runtime, and
		 * do not do redundant cb's jobs
		 */
		qos_ctrl_call_cb(QOS_TYPE_CPU,
			(void *)matched_scen->outputs[i]->cb,
			matched_scen->outputs[i]->output_set[output_level]);
		qos_brain.cpuqos_prev_mapped_output[i] =
			matched_scen->outputs[i]->output_set[output_level];
	}
}

static inline void __cpuqos_do_poll(struct cpuqos_scenario *matched_scen,
		u32 output_level, int i, bool always_do)
{
	if (qos_brain.cpu_meta.poll.is_registered &&
			((qos_brain.cpuqos_prev_mapped_output[i] !=
			 matched_scen->outputs[i]->output_set[output_level]) || always_do)) {
		/* 
		 * Make sure the cb is registered during runtime, and
		 * do not do redundant cb's jobs
		 */
		cpuqos_send_poll_message(&qos_brain.cpu_meta,
                matched_scen->outputs[i]->output_set[output_level]);
		qos_brain.cpuqos_prev_mapped_output[i] =
			matched_scen->outputs[i]->output_set[output_level];
	}
}

static inline void __cpuqos_do_write(struct cpuqos_scenario *matched_scen,
		u32 output_level, int i)
{
	/* 
	 * Writer cannot use cpuqos_prev_mapped_output as cb or poll for the fact
	 * that:
	 * cb or poll: different scenarios after mapping might have the same outputs.
	 * Writer: we cannot identify whether different mapped writer have the same
	 * outputs.
     * Although we can reduce some, it cost too many "if" for it.
     * Not worth to do it.
	 */
	struct cpuqos_output_writer *writer;
    list_for_each_entry(writer, &matched_scen->outputs[i]->writer_heads,
            head) {
        qos_write_path(writer->write_path,
                writer->writer_str[output_level]);
    }
}

static void cpuqos_function_adjust(struct cpuqos_scenario *matched_scen,
		u32 output_level, int type_index, bool always_do)
{
	/* writer & callbacks & epoll */
	if (matched_scen->output_map[type_index] && matched_scen->outputs[
			type_index]) {
		/* only the linked outputs are able to come here */
		switch (type_index) {
			case QOS_WRITER_OUTPUT_INDEX:
				__cpuqos_do_write(matched_scen, output_level, type_index);
				break;
			case QOS_EPOLL_OUTPUT_INDEX:
				/* notify the registered poll */
				__cpuqos_do_poll(matched_scen, output_level, type_index,
						always_do);
				break;
			case QOS_AIPQ_OUTPUT_INDEX:
				/* only call aipq's cb when aipq is enabled */
				if (READ_ONCE(qos_brain.current_scen_id) & BIT(QOS_AIPQ_ON)) {
					__cpuqos_do_cb(matched_scen, output_level, type_index,
							always_do);
				}
				break;
			case QOS_UCD_OUTPUT_INDEX:
				/* only call ucd's cb when ucd is enabled */
				if (READ_ONCE(qos_brain.current_scen_id) & BIT(QOS_UCD_ON)) {
					__cpuqos_do_cb(matched_scen, output_level, type_index,
							always_do);
				}
				break;
			default:
				/* other cb */
				__cpuqos_do_cb(matched_scen, output_level, type_index,
						always_do);
				break;
		}
	}
}

void cpuqos_output_adjust(struct cpuqos_scenario *main_scen,
		u32 output_level, enum qos_output_mode mode,
		struct cpuqos_scenario *prev_scen)
{
	int i;
	for (i = __OUTPUT_MAP_START_INDEX; i < __NR_OUTPUT_MAP_TYPE; i++) {
		switch (mode) {
			case QOS_OUTPUT_NORMAL:
				cpuqos_function_adjust(main_scen, output_level, i, false);
				break;
			case QOS_OUTPUT_DELAY_INIT:
				/* 
				 * Used when the video start to play for those "delay_ctrl"
				 * to pause the feature at begining.
				 */
				if (main_scen->delay_ctrl[i]) {
					cpuqos_print(CPUQOS_PRINT_LOW, "First ctrl. for delay.\n");
					cpuqos_function_adjust(main_scen, output_level, i, true);
				}
				break;
			case QOS_OUTPUT_RESTORE:
				if (prev_scen && prev_scen->output_map[i] &&
						!(main_scen->output_map[i])) {
					/* 
					 * Only restore those not controlled in the new scen,
					 * pass prev_scen as matched_scen.
					 */
					if (qos_brain.delay_timeout && main_scen->delay_ctrl[i]) {
						cpuqos_function_adjust(prev_scen, output_level, i,
								false);
					} else if (!main_scen->delay_ctrl[i]) {
						cpuqos_function_adjust(prev_scen, output_level, i,
								false);
					}
				}
				break;
			default:
				cpuqos_print(CPUQOS_PRINT_HIGH,
						"Not handled cpuqos_function_adjust() event.\n");
				break;
		}
	}
}

static int cpuqos_dispatcher(void *arg)
{
	/* main key-resource control entry for CPU */
	int cpu_load, i, higher_polling_cnt = 0;
	const int higher_polling_max = 4;
	bool new_scen = false, scen_popped = false, is_same_scen;
	bool higher_polling = false, do_ctrl;
	u32 output_level = QOS_CTRL_OUT_ENABLE;
	struct cpuqos_scenario *matched_scen;
	for (i = 0; i < __NR_QOS_SCENARIO_RECORD; i++) {
		qos_brain.prev_scen_rec[i].scen = qos_brain.default_scen;
		qos_brain.prev_scen_rec[i].output_level = QOS_CTRL_OUT_ENABLE;
	}
	/* 
	 * qos_brain.matched_scen is initialized during parsing dts as "default"
	 * scenario 
	 */
	while (cpuqos_is_on()) {
		if (unlikely(!qos_brain.cpu_meta.cpuqos_pm_ready)) {
			cpuqos_print(CPUQOS_PRINT_LOW, "CPU_Dispatcher paused...\n");
			msleep(cpu_ctrl_flip_ms << 2);
			continue;
		}
		/* detect the change of scenarios */
		matched_scen = READ_ONCE(qos_brain.matched_scen);
		if (matched_scen != qos_brain.prev_scen_rec[
				QOS_LAST_SCENARIO_INDEX].scen) {
			/* 
			 * scenario changed.
			 * Since QoS are not able to be aware of "the end of a scenario",
			 * stack is not suitable for here. We only maintain the "last 2
			 * secnarios" to detect the changes 
			 */
			if (matched_scen == qos_brain.prev_scen_rec[
					QOS_PREV_LAST_SCENARIO_INDEX].scen) {
				scen_popped = true;
				/* restore */
				output_level = qos_brain.prev_scen_rec[
					QOS_PREV_LAST_SCENARIO_INDEX].output_level;
			} else {
				new_scen = true;
				/* reset */
				output_level = matched_scen->init_ctrl;
			}
			is_same_scen = !new_scen && !scen_popped;
			if (!is_same_scen) {
				/* restore the proper output of prev scen as not controlled */
				cpuqos_print(CPUQOS_PRINT_LOW,
						"Scen changed, restore level=%d if not ctrl in this scne.\n",
						QOS_CTRL_OUT_ENABLE);
				cpuqos_output_adjust(matched_scen, QOS_CTRL_OUT_ENABLE,
						QOS_OUTPUT_RESTORE,
						qos_brain.prev_scen_rec[QOS_LAST_SCENARIO_INDEX].scen);
				cpuqos_print(CPUQOS_PRINT_LOW, "End restore.\n");
			}
			qos_brain.prev_scen_rec[QOS_PREV_LAST_SCENARIO_INDEX] =
				qos_brain.prev_scen_rec[QOS_LAST_SCENARIO_INDEX];
		} else {
			is_same_scen = true;
		}
		cpu_load = atomic_read(&cpuqos_cpu_load);
		output_level = cpuqos_output_level_calc(is_same_scen, output_level,
				cpu_load, matched_scen->trigger_map[QOS_CPU_LOAD_INDEX],
				matched_scen);
		do_ctrl = !(is_same_scen && (qos_brain.prev_scen_rec[
					QOS_LAST_SCENARIO_INDEX].output_level == output_level));
		if (unlikely(qos_brain.delay_first_ctrl && qos_brain.delay_timeout &&
					matched_scen->has_delayed_ctrl)) {
			qos_brain.delay_first_ctrl = false;
			for (i = __OUTPUT_MAP_START_INDEX; i < __NR_OUTPUT_MAP_TYPE; i++) {
				/* 
				 * Only control the delayed features after played for once
				 * to avoid the output_level keeps the same, and the delayed
				 * features are not enable before changing the output_level.
				 */
				if ((qos_brain.src_status == QOS_SRC_IN) &&
					matched_scen->delay_ctrl[i]) {
					cpuqos_print(CPUQOS_PRINT_LOW,
							"First ctrl. after delay for [%s].\n",
							matched_scen->outputs[i]->owner);
					cpuqos_function_adjust(matched_scen, output_level, i, true);
				}
			}
		} else if (do_ctrl) {
			higher_polling = true;
			for (i = __OUTPUT_MAP_START_INDEX; i < __NR_OUTPUT_MAP_TYPE; i++) {
				/* try to reduce the redundant control */
				if ((qos_brain.src_status == QOS_SRC_STOP) ||
					((qos_brain.src_status == QOS_SRC_IN) &&
					((matched_scen->delay_ctrl[i] && qos_brain.delay_timeout) ||
					!matched_scen->delay_ctrl[i]))) {
					cpuqos_function_adjust(matched_scen, output_level, i, false);
				}
			}
		}
		/* update prev. status */
		qos_brain.prev_scen_rec[QOS_LAST_SCENARIO_INDEX].scen = matched_scen;
		qos_brain.prev_scen_rec[QOS_LAST_SCENARIO_INDEX].output_level = 
			output_level;
		new_scen = false;
		scen_popped = false;
		if (higher_polling) {
			higher_polling_cnt += 1;
			if (higher_polling_cnt >= higher_polling_max) {
				higher_polling_cnt = 0;
				higher_polling = false;
			}
			msleep(cpu_load_period_ms);
		} else {
			msleep(cpu_ctrl_flip_ms);
		}
	}
	return CPUQOS_RET_FAIL;
}

void cpuqos_set_on(void)
{
	struct task_struct *manager, *crew;
	qos_brain.cpu_meta.cpuqos_pm_ready = true;
	qos_brain.cpu_meta.key_detect_switch = CPUQOS_KEY_IS_NOT_DETECTED;
	/* init scen_id */
    qos_scenario_set(&qos_brain.current_scen_id, matching_masks.video_stop
			| BIT(QOS_IR_KEY_NONE) | BIT(QOS_VOICE_OFF));

	cpuqos_on = true;
    crew = kthread_run(cpuqos_maintainer, NULL, "CPU_Maintainer");
    manager = kthread_run(cpuqos_dispatcher, NULL, "CPU_Dispatcher");
    if (IS_ERR(manager) || IS_ERR(crew)) {
		if (IS_ERR(crew)) {
			qos_print(QOS_PRINT_HIGH, "CPU_Maintainer create failed\n");
		}
		if (IS_ERR(manager)) {
			qos_print(QOS_PRINT_HIGH, "CPU_Dispatcher create failed\n");
		}
    } else {
		cpuqos_print(CPUQOS_PRINT_HIGH, "=======CPUQoS is initialized=======\n");
	}
}

void cpuqos_key_detect_entry(int key_value, unsigned int key_code,
		u64 *new_scen_id)
{
    unsigned long flags;
	/* main entry for key control */
	if (unlikely((key_code == KEY_POWER) ||
				(!qos_brain.cpu_meta.cpuqos_pm_ready))) {
		/* KEY_POWER may make CPUQoS internal status incorrect,
		 * skip control when KEY_POWER is in */
		return;
	}
    spin_lock_irqsave(&qos_brain.cpu_meta.key_status_lock, flags);
	if (key_value == 1) {
		/* press key */
		if (qos_brain.cpu_meta.key_detect_switch == CPUQOS_KEY_IS_NOT_DETECTED) {
			/* QOS_IR_KEY_SET & QOS_IR_KEY_NONE are mutually exclusive */
			qos_enter_scenario_helper(BIT(QOS_IR_KEY_NONE), BIT(QOS_IR_KEY_SET),
					false);
			/* start timer for exiting key related scenario */
			cpuqos_key_schedule_timer(&qos_brain.cpu_meta);
		} else {
			cpuqos_update_key_expire_timer(INT_MAX);
		}
	} else if (key_value == 0) {
		/* release key */
		/* stop detecting after a while */
		cpuqos_update_key_expire_timer(key_detect_lasting_ms);
	}
    spin_unlock_irqrestore(&qos_brain.cpu_meta.key_status_lock, flags);
}

static void cpuqos_force_exit_detect(void)
{
	qos_brain.cpu_meta.cpuqos_pm_ready = false;
	del_timer(&qos_brain.cpu_meta.monitor_key_timer);
	__cpuqos_exit_key_helper();
	cpuqos_output_adjust(qos_brain.matched_scen, QOS_CTRL_OUT_ENABLE,
			QOS_OUTPUT_NORMAL, NULL);
}

int cpuqos_pm_handler (struct notifier_block *nb, unsigned long event,
	void *dummy) {
	if (cpuqos_is_on()) {
		if (event == PM_SUSPEND_PREPARE) {
			cpuqos_print(CPUQOS_PRINT_HIGH, "Pre-Suspend handler start...\n");
			cpuqos_force_exit_detect();
			cpuqos_print(CPUQOS_PRINT_HIGH, "Pre-Suspend handler done, "
					"release all.\n");
			return NOTIFY_OK;
		} else if (event == PM_POST_SUSPEND ) {
			qos_brain.cpu_meta.cpuqos_pm_ready = true;
			cpuqos_print(CPUQOS_PRINT_HIGH, "Post-Resume handler done.\n");
			return NOTIFY_OK;
		}
	}
	return NOTIFY_DONE;
}

int cpuqos_reboot_handler (struct notifier_block *nb, unsigned long event,
		void *dummy) {
	if (cpuqos_is_on()) {
		if (event == SYS_POWER_OFF) {
			cpuqos_print(CPUQOS_PRINT_HIGH, "Reboot handler start...\n");
			cpuqos_set_off();
			cpuqos_print(CPUQOS_PRINT_HIGH, "Reboot handler end\n");
			return NOTIFY_OK;
		} else {
			cpuqos_print(CPUQOS_PRINT_HIGH, "Unknow reboot event=%lu\n", event);
		}
	}
	return NOTIFY_DONE;
}

int cpuqos_init(void)
{
	init_waitqueue_head(&qos_brain.cpu_meta.poll.wq);
    spin_lock_init(&qos_brain.cpu_meta.key_status_lock);
	qos_brain.cpu_meta.poll.is_registered = false;
	qos_brain.cpu_meta.dts_heads = NULL;
	qos_brain.src_status = QOS_SRC_STOP;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
	timer_setup(&qos_brain.cpu_meta.monitor_key_timer, cpuqos_key_exit_handler,
			0);
#else
	init_timer(&qos_brain.cpu_meta.monitor_key_timer);
	qos_brain.cpu_meta.monitor_key_timer.function = cpuqos_key_exit_handler;
#endif
	return CPUQOS_RET_SUCCESS;
}
