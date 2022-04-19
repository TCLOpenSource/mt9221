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

#ifndef _MDRV_CPU_DVFS_SCALING_LIST_H
#define _MDRV_CPU_DVFS_SCALING_LIST_H
#define LIST_MAX_LENGTH 512
#define sensor_polling_time 1500
typedef enum{
    e_Bigcore=0,           //Big Core
    e_excl_Littlecore,     //Little core
    e_AllCpuMask,          //all mask
    e_excl_one_Littlecore  //one Little core
}Get_CPU_MASK;
unsigned long get_CPU_mask(int mask_type);
void bench_boost_check(void);
void app_boost_check(void);
bool foundSpecialTask(char* searchSource, char* pattern,int len);
struct cpu_scaling_list* accel87_list_add(struct cpu_scaling_list *head,struct task_struct *delete_node);
struct cpu_scaling_list* accel87_list_delete(struct cpu_scaling_list *head,struct task_struct *delete_node);
void accel87_check(void);
struct cpu_scaling_list* cpu_scaling_list_add(struct cpu_scaling_list *head,struct task_struct *delete_node);
struct cpu_scaling_list* cpu_scaling_list_delete(struct cpu_scaling_list *head,struct task_struct *delete_node);
struct cpu_scaling_list {
       struct task_struct *tsk;
       struct cpu_scaling_list *next;
       int target_id;
       };
extern char bench_boost_name_list[LIST_MAX_LENGTH];
extern char app_boost_name_list[LIST_MAX_LENGTH];
#endif
