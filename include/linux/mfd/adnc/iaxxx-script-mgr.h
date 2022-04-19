/*
 * iaxxx-script-mgr.h -- IAxxx Script Manager header
 *
 * Copyright 2019 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IAXXX_SCRPT_MGR_H__
#define __IAXXX_SCRPT_MGR_H__

struct iaxxx_script_data {
	uint32_t script_id;
	struct list_head script_node;
};

bool iaxxx_core_is_valid_script_id(uint32_t script_id);
struct iaxxx_script_data *iaxxx_core_script_id_exist(struct iaxxx_priv *priv,
							uint32_t scrpt_id);
bool iaxxx_core_script_list_empty(struct iaxxx_priv *priv);
int iaxxx_clr_script_list(struct iaxxx_priv *priv);

int iaxxx_core_script_load(struct device *dev, const char *script_name,
							uint32_t script_id);
int iaxxx_core_script_unload(struct device *dev, uint32_t script_id);
int iaxxx_core_script_trigger(struct device *dev, uint32_t script_id);

#endif /* __IAXXX_SCRPT_MGR_H__ */
