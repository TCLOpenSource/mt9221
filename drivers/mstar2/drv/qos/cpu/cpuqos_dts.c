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
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/delay.h>

#include "mdrv_qos.h"
#include "mdrv_qos_drv.h"
#include "cpuqos_drv.h"

static int cpuqos_alloc_lists(void)
{
    if (qos_brain.cpu_meta.dts_heads) {
        cpuqos_print(CPUQOS_PRINT_HIGH,
                "CPUQoS does not support re-initialization.\n");
        return CPUQOS_RET_FAIL;
    }
    qos_brain.cpu_meta.dts_heads =
		kzalloc(sizeof(*qos_brain.cpu_meta.dts_heads), GFP_KERNEL);
    if (!qos_brain.cpu_meta.dts_heads) {
        cpuqos_print(CPUQOS_PRINT_HIGH,
                "CPUQoS failed to allocaed qos_brain.cpu_meta.dts_heads\n");
        return CPUQOS_RET_FAIL;
    } else {
        INIT_LIST_HEAD(&qos_brain.cpu_meta.dts_heads->scenario_head);
        INIT_LIST_HEAD(&qos_brain.cpu_meta.dts_heads->registry_head);
        INIT_LIST_HEAD(&qos_brain.cpu_meta.dts_heads->output_map_head);
    }
    return CPUQOS_RET_SUCCESS;
}

void cpuqos_print_reg(struct cpuqos_registry *reg)
{
    cpuqos_print(CPUQOS_PRINT_LOW, "REG-> node=%s; "
            "type="BYTE_TO_BINARY_PATTERN"; path=%s; value=%s; "
            "proc_comm=%s; thread_comm=%s; pid_num=%u; tid_num=%u\n",
            reg->node_name, BYTE_TO_BINARY(reg->reg_type),
            reg->write_path, reg->val_string, reg->proc_comm,
            reg->thread_comm, reg->pid_num, reg->tid_num);
}

static void cpuqos_parse_reg_type(struct cpuqos_registry *reg,
        const char *str)
{
    if (!strcmp(str, "PID")) {
        reg->reg_type = BIT(CPUQOS_REG_PID);
    } else if (!strcmp(str, "TID")) {
        reg->reg_type = BIT(CPUQOS_REG_TID) | BIT(CPUQOS_REG_PID);
    } else {
        /* treat as "OTHER" */
        reg->reg_type = BIT(CPUQOS_REG_OTHER);
    }
}

static int cpuqos_parse_registry(struct device_node *registry)
{
    struct device_node *child;
    struct cpuqos_registry *reg;
    const char *str_ptr;
    int ret;
    int func_ret = CPUQOS_RET_SUCCESS;
    for_each_available_child_of_node(registry, child) {
        reg = kzalloc(sizeof(*reg), GFP_KERNEL);
        INIT_LIST_HEAD(&reg->pid_heads);
        if (!reg) {
            cpuqos_print(CPUQOS_PRINT_HIGH, "kmalloc() fail for reg\n");
            return CPUQOS_RET_FAIL;
        }
        strlcpy(reg->node_name, child->name, sizeof(reg->node_name));
        /* target */
        ret = of_property_read_string(child, "target", &str_ptr);
        if (ret) {
            reg->reg_type = BIT(CPUQOS_REG_OTHER);
        } else {
            /* set type */
            cpuqos_parse_reg_type(reg, str_ptr);
        }
        /* path */
        ret = of_property_read_string(child, "path", &str_ptr);
        if (ret) {
            strlcpy(reg->write_path, "/dev/null", sizeof(reg->write_path));
            func_ret = CPUQOS_RET_FAIL;
        }
        strlcpy(reg->write_path, str_ptr, sizeof(reg->write_path));
        /* comm_pid */
        if (reg->reg_type & BIT(CPUQOS_REG_PID)) {
            ret = of_property_read_string(child, "comm_pid", &str_ptr);
            if (ret) {
                cpuqos_print(CPUQOS_PRINT_HIGH, "comm_pid is missing in "
                        "node=%s.\n", child->full_name);
                strlcpy(reg->proc_comm, "ERROR", sizeof(reg->proc_comm));
                func_ret = CPUQOS_RET_FAIL;
            }
            strlcpy(reg->proc_comm, str_ptr, sizeof(reg->proc_comm));
            /* get pid_num */
            ret = of_property_read_u32(child, "pid_num", &reg->pid_num);
            if (ret) {
                reg->pid_num = 1;
                cpuqos_print(CPUQOS_PRINT_HIGH, "pid_num set to default=%u"
                        " in node=%s.\n", reg->pid_num, child->full_name);
            }
        }
        /* comm_tid */
        if (reg->reg_type & BIT(CPUQOS_REG_TID)) {
            ret = of_property_read_string(child, "comm_tid", &str_ptr);
            if (ret) {
                cpuqos_print(CPUQOS_PRINT_HIGH, "comm_tid is missing in "
                        "node=%s.\n", child->full_name);
                strlcpy(reg->thread_comm, "ERROR", sizeof(reg->thread_comm));
                func_ret = CPUQOS_RET_FAIL;
            }
            strlcpy(reg->thread_comm, str_ptr, sizeof(reg->thread_comm));
            /* get tid_num */
            ret = of_property_read_u32(child, "tid_num", &reg->tid_num);
            if (ret) {
                reg->tid_num = 1;
                cpuqos_print(CPUQOS_PRINT_HIGH, "tid_num set to default=%u"
                        " in node=%s.\n", reg->tid_num, child->full_name);
            }
        }
        /* value */
        if (reg->reg_type & BIT(CPUQOS_REG_OTHER)) {
            ret = of_property_read_string(child, "value", &str_ptr);
            if (ret) {
                cpuqos_print(CPUQOS_PRINT_HIGH, "value is missing in "
                        "node=%s.\n", child->full_name);
                strlcpy(reg->val_string, "ERROR", sizeof(reg->val_string));
                func_ret = CPUQOS_RET_FAIL;
            }
            strlcpy(reg->val_string, str_ptr, sizeof(reg->val_string));
        }
        list_add_tail(&reg->head, &qos_brain.cpu_meta.dts_heads->registry_head);
    }
    return func_ret;
}


static void cpuqos_parse_output_map_type(struct cpuqos_output_map *map)
{
    if (!strcmp(map->owner, "writer")) {
        map->output_type = BIT(CPUQOS_OUTPUT_MAP_WRITER);
	} else if (!strcmp(map->owner, "epoll")) {
        map->output_type = BIT(CPUQOS_OUTPUT_MAP_EPOLL);
    } else if (!strcmp(map->owner, "ERROR")){
        map->output_type = BIT(CPUQOS_OUTPUT_MAP_UNINIT);
    } else {
        /* treat as "callback" */
        map->output_type = BIT(CPUQOS_OUTPUT_MAP_CALLBACK);
    }
}

static void cpuqos_print_output_map(struct cpuqos_output_map *map)
{
#define QOS_TMP_LEN_1  128
#define QOS_TMP_LEN_2   16
    char buf_1[QOS_TMP_LEN_1] = {'\0'}, tmp_buf[QOS_TMP_LEN_2];
    struct cpuqos_output_writer *writer;
    int i;
    for (i = 0; i < __NR_QOS_CTRL_OUT_LEVEL; i++) {
        snprintf(tmp_buf, QOS_TMP_LEN_2, "%d, ", map->output_set[i]);
        strlcat(buf_1, tmp_buf, QOS_TMP_LEN_1);
    }
    cpuqos_print(CPUQOS_PRINT_LOW, "OUTPUT_MAP -> type="BYTE_TO_BINARY_PATTERN
            "; owner=%s; id=%u;\noutput_set(max. print = %d)=%s;\n",
            BYTE_TO_BINARY(map->output_type), map->owner, map->id,
            QOS_TMP_LEN_1, buf_1);
    if (map->output_type & BIT(CPUQOS_OUTPUT_MAP_WRITER)) {
        list_for_each_entry(writer, &map->writer_heads, head) {
            qos_print_cont("write_path=%s;\nwriter_str=", writer->write_path);
            for (i = 0; i < __NR_QOS_CTRL_OUT_LEVEL; i++) {
                qos_print_cont("%s, ", writer->writer_str[i]);
            }
            qos_print_cont("\n");
        }
    }
    qos_print_cont("-----------------------------------\n");
#undef QOS_TMP_LEN_1
#undef QOS_TMP_LEN_2
}

static int cpuqos_parse_output_map(struct device_node *map_node)
{
    struct device_node *child, *gchild;
    struct cpuqos_output_map *map;
    struct cpuqos_output_writer *writer;
    const char *str_ptr;
    const char *writer_str_ptr[__NR_QOS_CTRL_OUT_LEVEL];
    int ret, i;
    int func_ret = CPUQOS_RET_SUCCESS;

    for_each_available_child_of_node(map_node, child) {
        map = kzalloc(sizeof(*map), GFP_KERNEL);
        if (!map) {
            cpuqos_print(CPUQOS_PRINT_HIGH, "kmalloc() fail for map\n");
            return CPUQOS_RET_FAIL;
        }
        /* owner */
        ret = of_property_read_string(child, "owner", &str_ptr);
        if (ret) {
            cpuqos_print(CPUQOS_PRINT_HIGH, "owner is missing in "
                    "node=%s.\n", child->full_name);
            strlcpy(map->owner, "ERROR", sizeof(map->owner));
            func_ret = CPUQOS_RET_FAIL;
        }
        strlcpy(map->owner, str_ptr, sizeof(map->owner));

        cpuqos_parse_output_map_type(map);

        /* id */
        ret = of_property_read_u32(child, "id", &map->id);
        if (ret) {
			/* id = 0 means to be disabled */
            map->id = 0;
            cpuqos_print(CPUQOS_PRINT_HIGH, "id is missing in node=%s.\n",
                    child->full_name);
        }

        if (map->output_type & BIT(CPUQOS_OUTPUT_MAP_CALLBACK) ||
				map->output_type & BIT(CPUQOS_OUTPUT_MAP_EPOLL)) {
            /* callback or epoll */
            ret = of_property_read_u32_array(child, "output_set",
                    map->output_set, __NR_QOS_CTRL_OUT_LEVEL);
            if (ret) {
                cpuqos_print(CPUQOS_PRINT_HIGH, "output_set is error in "
                        "node=%s.\n", child->full_name);
                memset(map->output_set, 0,
                        __NR_QOS_CTRL_OUT_LEVEL * sizeof(map->output_set[0]));
                func_ret = CPUQOS_RET_FAIL;
            }
        } else if (map->output_type & BIT(CPUQOS_OUTPUT_MAP_WRITER)) {
            /* writer */
            INIT_LIST_HEAD(&map->writer_heads);
            for_each_available_child_of_node(child, gchild) {
                writer = kzalloc(sizeof(*writer), GFP_KERNEL);
                if (!writer) {
                    cpuqos_print(CPUQOS_PRINT_HIGH,
							"kmalloc() fail for writer\n");
                    return CPUQOS_RET_FAIL;
                }
                ret = of_property_read_string(gchild, "path", &str_ptr);
                if (ret) {
                    cpuqos_print(CPUQOS_PRINT_HIGH, "path is missing in "
                            "node=%s.\n", gchild->full_name);
                    strlcpy(writer->write_path, "/dev/null",
                            sizeof(writer->write_path));
                    func_ret = CPUQOS_RET_FAIL;
                }
                strlcpy(writer->write_path, str_ptr, sizeof(writer->write_path));

                ret = of_property_read_string_array(gchild, "output_set",
                        writer_str_ptr, __NR_QOS_CTRL_OUT_LEVEL);
                if (ret != __NR_QOS_CTRL_OUT_LEVEL) {
                    cpuqos_print(CPUQOS_PRINT_HIGH, "output_set is missing in "
                            "node=%s; ret=%d\n", gchild->full_name, ret);
                    for (i = 0; i < __NR_QOS_CTRL_OUT_LEVEL; i++) {
                        strlcpy(writer->writer_str[i], "ERROR",
                                CPUQOS_VAL_LEN);
                    }
                    func_ret = CPUQOS_RET_FAIL;
                } else {
                    for (i = 0; i < __NR_QOS_CTRL_OUT_LEVEL; i++) {
                        strlcpy(writer->writer_str[i], writer_str_ptr[i],
                                CPUQOS_VAL_LEN);
                    }
                }
                list_add_tail(&writer->head, &map->writer_heads);
            }
        }
        list_add_tail(&map->head, &qos_brain.cpu_meta.dts_heads->output_map_head);
    }
    return func_ret;
}

static void cpuqos_print_scenario(struct cpuqos_scenario *scen)
{
	/* 
	 * output_map: refer to "enum output_map_index"
	 * trigger_map: refer to "enum trigger_map_index"
	 */
	char output_map_msg[32] = {'\0'};
	char delay_ctrl_msg[32] = {'\0'};
	int i, rel_addr = 0;
	for (i = __OUTPUT_MAP_START_INDEX; i < __NR_OUTPUT_MAP_TYPE; i++) {
		rel_addr += snprintf(output_map_msg + rel_addr, sizeof(output_map_msg),
				"%d ", scen->output_map[i]);
	}
	rel_addr = 0;
	for (i = __OUTPUT_MAP_START_INDEX; i < __NR_OUTPUT_MAP_TYPE; i++) {
		rel_addr += snprintf(delay_ctrl_msg + rel_addr, sizeof(delay_ctrl_msg),
				"%d ", scen->delay_ctrl[i]);
	}
    cpuqos_print(CPUQOS_PRINT_LOW, "Parsed scenario [%s] --> "
            "output_map=%s; delay_ctrl=%s; init_ctrl=%u; trigger_map=%d;\n",
            scen->node_name, output_map_msg, delay_ctrl_msg, scen->init_ctrl,
			scen->trigger_map[0]);
    qos_dump_scenario_mask(scen->mask_id, CPUQOS_PRINT_LOW);
}

static int cpuqos_parse_scenario(struct device_node *scenarios)
{
    struct device_node *child, *events;
    int ret, i, delay = 0;
    struct cpuqos_scenario *scen;
	u32 key_press = 0, voice = 0;
    u32 video_arr[__NR_QOS_VIDEO_MEMBER] = {0}; /* default to "ignore" */
    for_each_available_child_of_node(scenarios, child) {
        /* for each scenario, from top to down of DTS */
        cpuqos_print(CPUQOS_PRINT_LOW, "DTS: get node %s\n", child->name);
        events = of_get_child_by_name(child, "input-events");
        if (!events) {
            cpuqos_print(CPUQOS_PRINT_HIGH,
					"DTS: parsing error at input-events\n");
            return CPUQOS_RET_FAIL;
        }
        scen = kzalloc(sizeof(*scen), GFP_KERNEL);
        if (!scen) {
            cpuqos_print(CPUQOS_PRINT_HIGH, "kmalloc() fail for scen\n");
            return CPUQOS_RET_FAIL;
        }
        if (!strcmp(child->name, "default")) {
			/* if here does not go through once, this will cause kernel panic */
            cpuqos_print(CPUQOS_PRINT_LOW, "Get default CPUQoS config set: %s\n",
                    child->name);
			WRITE_ONCE(qos_brain.default_scen, scen);
			WRITE_ONCE(qos_brain.matched_scen, scen);
        }
        strlcpy(scen->node_name, child->name, sizeof(scen->node_name));
        /* key_press */
        ret = of_property_read_u32(events, "key_press", &key_press);
        if (ret) {
            key_press = 0;
        }
        /* voice */
        ret = of_property_read_u32(events, "voice", &voice);
        if (ret) {
            voice = 0;
        }
        /* video */
        ret = of_property_read_u32_array(events, "video", video_arr,
                __NR_QOS_VIDEO_MEMBER);
        if (ret) {
            memset(video_arr, 0, sizeof(video_arr));
        }
        qos_dts_to_video(&scen->video, video_arr);
		if (key_press) {
			key_press = QOS_IR_KEY_SET;
		} else {
			key_press = QOS_IR_KEY_NONE;
		}
		if (voice) {
			voice = QOS_VOICE_ON;
		} else {
			voice = QOS_VOICE_OFF;
		}
        scen->mask_id = qos_scenario_to_mask(&scen->video, key_press,
				voice);
        /* output-level */
        ret = of_property_read_u32_array(child, "output_map",
                scen->output_map, __NR_OUTPUT_MAP_TYPE);
        if (ret) {
            memset(scen->output_map, 0, sizeof(scen->output_map));
        }
        /* delayed ctrl for those features are default disabled */
        ret = of_property_read_u32_array(child, "delay_ctrl",
                scen->delay_ctrl, __NR_OUTPUT_MAP_TYPE);
        if (ret) {
            memset(scen->delay_ctrl, 0, sizeof(scen->delay_ctrl));
        }
		scen->has_delayed_ctrl = false;
		for (i =0; i < __NR_OUTPUT_MAP_TYPE; i++) {
			delay |= scen->delay_ctrl[i];
		}
		if (delay) {
			scen->has_delayed_ctrl = true;
		}
        /* init  */
        ret = of_property_read_u32_array(child, "init_ctrl",
                &scen->init_ctrl, 1);
        if (ret) {
			scen->init_ctrl = QOS_CTRL_OUT_ENABLE;
        }
        /* trigger-threshold */
        ret = of_property_read_u32_array(child, "trigger_map",
                scen->trigger_map, __NR_TRIGGER_MAP_TYPE);
        if (ret) {
			for (i = 0; i < __NR_TRIGGER_MAP_TYPE; i++) {
                /* timing latency may cause cpu load > 100 sometimes.
                 * Therefore, set to a number > 101 or 102 is fine to
                 * indicate DISABLE.
                 */
				scen->trigger_map[i] = 999;
			}
        }
        /* use list as queue to make the DTS search from top to down */
        list_add_tail(&scen->head, &qos_brain.cpu_meta.dts_heads->scenario_head);
    }
    return CPUQOS_RET_SUCCESS;
}

static int qos_link_callbacks(struct cpuqos_output_map *map,
		char target_cb[])
{
	struct qos_cb_unit *cb_curr;
	bool link_done = false;
	int ret;
	while (!qos_init_done) {
		/* This does not guarantee that callbacks are done, but a hint */
		cpuqos_print(CPUQOS_PRINT_HIGH, "Try to link callbacks, but init_done="
				"%d.\n", qos_init_done);
		msleep(1000);
	}
	mutex_lock(&qos_brain.cb_list_lock);
	list_for_each_entry(cb_curr, &qos_brain.qos_cb_heads, cb_head) {
		if (!strncmp(map->owner, cb_curr->owner, QOS_CALLBACK_NAME_LEN)) {
			map->cb = cb_curr;
			cpuqos_print(CPUQOS_PRINT_LOW, "Link cb(%s) successfully.\n",
					target_cb);
			link_done = true;
			ret = CPUQOS_RET_SUCCESS;
			break;
		}
	}
	mutex_unlock(&qos_brain.cb_list_lock);
	if (unlikely(!link_done)) {
		ret = CPUQOS_RET_FAIL;
		cpuqos_print(CPUQOS_PRINT_HIGH, "Link cb(%s) failed. "
			"(\"the owner did not register?\" or "
			"\"the owner is not initialized yet?\")\n",
			target_cb);
	}
	return ret;
}

static void qos_scenario_link_outputs(struct cpuqos_scenario *scen)
{
    int count = 0, i, ret;
    struct cpuqos_output_map *map;
	/* 
	 * if add new callback in future, this should be expaned in order
	 * as "enum output_map_index"
	 */
	char register_table[__NR_OUTPUT_MAP_TYPE][CPUQOS_OWNER_LEN];
	memset(register_table, '0', sizeof(register_table));
	strlcpy(register_table[QOS_AIPQ_OUTPUT_INDEX], "AIPQ",
			CPUQOS_OWNER_LEN);
	strlcpy(register_table[QOS_UCD_OUTPUT_INDEX], "UCD",
			CPUQOS_OWNER_LEN);
	strlcpy(register_table[QOS_EPOLL_OUTPUT_INDEX], "epoll",
			CPUQOS_OWNER_LEN);
    list_for_each_entry(map, &qos_brain.cpu_meta.dts_heads->output_map_head,
			head) {
		if (map->id == 0) {
			qos_print(QOS_PRINT_HIGH,
					"Error: id = 0 is reserved for \"DISABLE\"!!!\n");
		}
        if (map->output_type & BIT(CPUQOS_OUTPUT_MAP_WRITER)) {
			if (scen->output_map[QOS_WRITER_OUTPUT_INDEX] ==
					map->id) {
				scen->outputs[QOS_WRITER_OUTPUT_INDEX] = map;
				count++;
				qos_print(QOS_PRINT_LOW, "%s found %s -> id=%u\n",
						scen->node_name, map->owner, map->id);
			}
        } else if (map->output_type & BIT(CPUQOS_OUTPUT_MAP_CALLBACK)) {
			/* link all callbacks to scenarios */
			for (i = __QOS_CALLBACK_START_INDEX; i < __QOS_CALLBACK_END_INDEX;
					i++) {
				if ((!strncmp(map->owner, register_table[i],
						CPUQOS_OWNER_LEN))) {
					if (scen->output_map[i] &&
							(scen->output_map[i] == map->id)) {
						qos_print(QOS_PRINT_LOW, "%s found %s -> id=%u\n",
							scen->node_name, map->owner, map->id);
						scen->outputs[i] = map;
						do {
							ret = qos_link_callbacks(map,
									register_table[i]);
							if ((ret != CPUQOS_RET_SUCCESS) &&
									qos_brain.ctrl_reg_opt[i]) {
								qos_print(QOS_PRINT_HIGH,
										"Try to link [%s] again.\n",
										map->owner);
								msleep(1000);
							} else {
								count++;
								break;
							}
						} while (ret);
					}
				}
			}
		} else if (map->output_type & BIT(CPUQOS_OUTPUT_MAP_EPOLL)) {
			/* link epoll's output_map */
			if ((!strncmp(map->owner, register_table[QOS_EPOLL_OUTPUT_INDEX],
					CPUQOS_OWNER_LEN))) {
				if (scen->output_map[QOS_EPOLL_OUTPUT_INDEX] ==
						map->id) {
					scen->outputs[QOS_EPOLL_OUTPUT_INDEX] = map;
					count++;
					qos_print(QOS_PRINT_LOW, "%s found %s -> id=%u\n",
							scen->node_name, map->owner, map->id);
				}
			}
		}
        if (unlikely(count == __NR_OUTPUT_MAP_TYPE)) {
            break;
        }
    }
}

int cpuqos_parse_dts(void)
{
    int ret = CPUQOS_RET_SUCCESS, search_ret = CPUQOS_RET_SUCCESS;
	int i;
    struct device_node *scenarios, *registry, *map;
    struct cpuqos_registry *reg;
	struct cpuqos_scenario *scen;
    char name_reg[] = "/qos/cpu/registry";
    char name_scen[] = "/qos/cpu/scenarios";
    char name_map[] = "/qos/cpu/output-map";
    ret = cpuqos_alloc_lists();
    if (ret) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "Allocate lists for DTS failed.\n");
        return CPUQOS_RET_FAIL;
    }
    QOS_GET_NODE(name_reg, registry);
    ret |= cpuqos_parse_registry(registry);
    if (ret) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "Parsing node \"%s\" error, "
                "but this may only affect WRITER at %s:%d.\n",
                name_reg, __FILE__, __LINE__);
    }

    QOS_GET_NODE(name_scen, scenarios);
    if (ret) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "Parsing node \"%s\" error, "
                "but it is mandatory at %s:%d.\n",
                name_scen, __FILE__, __LINE__);
        return ret;
    }
    QOS_GET_NODE(name_map, map);
    if (ret) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "Parsing node \"%s\" error, "
                "but it is mandatory at %s:%d.\n",
                name_map, __FILE__, __LINE__);
        return ret;
    }

	/* 
	 * Warning: qos_scenario_link_outputs() should be called after
	 * scenario & ouptut is parsed.
	 */
    ret |= cpuqos_parse_scenario(scenarios);
    ret |= cpuqos_parse_output_map(map);
	if (ret != CPUQOS_RET_SUCCESS) {
		cpuqos_print(CPUQOS_PRINT_HIGH,
				"[Error] Parsing DTS error, please check dmesg(7).\n");
	}
    list_for_each_entry(scen, &qos_brain.cpu_meta.dts_heads->scenario_head,
			head) {
		/* make sure the callbacks are registered.
		 * e.g. initialize CPUQoS after boot complete.
		 */
		for (i = __OUTPUT_MAP_START_INDEX; i < __NR_OUTPUT_MAP_TYPE; i++) {
			/* make sure those "0" should be unliked */
			scen->outputs[i] = NULL;
		}
		qos_scenario_link_outputs(scen);
	}
    /* create pid/tid metadata */
    list_for_each_entry(reg, &qos_brain.cpu_meta.dts_heads->registry_head,
			head) {
        INIT_LIST_HEAD(&reg->pid_heads);
        if (reg->reg_type & BIT(CPUQOS_REG_PID)) {
            search_ret |= cpuqos_build_pids(reg);
        }
        if (reg->reg_type & BIT(CPUQOS_REG_TID)) {
            search_ret |= cpuqos_build_tids(reg);
        }
    }
    if (search_ret != CPUQOS_RET_SUCCESS) {
        cpuqos_print(CPUQOS_PRINT_HIGH, "Not all the PIDs/TIDs are found.\n");
    }
    /* start all service */
    cpuqos_set_on();

    of_node_put(scenarios);
    of_node_put(registry);
    of_node_put(map);
    return CPUQOS_RET_SUCCESS;
}

void cpuqos_print_parsed_dts(void)
{
    struct cpuqos_scenario *scen;
    struct cpuqos_registry *reg;
    struct cpuqos_output_map *map;
	char reg_cb[32] = {'\0'}, dts_ver[8] = {'\0'};
	int i, rel_addr = 0;
	for (i = QOS_DTS_MAJOR_VER_INDEX; i <= QOS_DTS_MINOR_VER_INDEX; i++) {
		rel_addr += snprintf(dts_ver + rel_addr, sizeof(dts_ver),
				"%u ", qos_brain.versions[i]);
	}
	rel_addr = 0;
	for (i = 0; i < __NR_OUTPUT_MAP_TYPE; i++) {
		rel_addr += snprintf(reg_cb + rel_addr, sizeof(reg_cb),
				"%u ", qos_brain.ctrl_reg_opt[i]);
	}
	qos_print(QOS_PRINT_LOW, "dts_version = <%s>\n", dts_ver);
	qos_print(QOS_PRINT_LOW, "ctrl_reg_opt = <%s>\n", reg_cb);
	if (!cpuqos_is_on()) {
		cpuqos_print(CPUQOS_PRINT_HIGH, "CPUQoS is not on in "
				"cpuqos_print_parsed_dts().\n");
		return;
	}
    cpuqos_print(CPUQOS_PRINT_LOW, "\n===============Begin=================\n");
	if (!list_empty(&qos_brain.cpu_meta.dts_heads->scenario_head)) {
		list_for_each_entry(scen,
				&qos_brain.cpu_meta.dts_heads->scenario_head, head) {
			cpuqos_print_scenario(scen);
		}
	}
    cpuqos_print(CPUQOS_PRINT_LOW, "=======================================\n");
	if (!list_empty(&qos_brain.cpu_meta.dts_heads->registry_head)) {
		list_for_each_entry(reg,
				&qos_brain.cpu_meta.dts_heads->registry_head, head) {
			cpuqos_print_reg(reg);
		}
	}
    cpuqos_print(CPUQOS_PRINT_LOW, "=======================================\n");
	if (!list_empty(&qos_brain.cpu_meta.dts_heads->output_map_head)) {
		list_for_each_entry(map,
				&qos_brain.cpu_meta.dts_heads->output_map_head, head) {
			cpuqos_print_output_map(map);
		}
	}
    cpuqos_print(CPUQOS_PRINT_LOW, "==================End==================\n");
}
