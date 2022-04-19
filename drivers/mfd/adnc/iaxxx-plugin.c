/*
 * iaxxx-plugin.c -- IAxxx plugin interface for Plugins
 *
 * Copyright 2017 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-regmap.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include "iaxxx.h"

#define IAXXX_BITS_SWAP	32
#define IAXXX_BLK_HEADER_SIZE 4
#define IAXXX_BIN_INFO_SEC_ADDR	0xF1F00000
#define IAXXX_INVALID_FILE ('\0')

/*
 * Generate package id with 'i' package id and 'p' processor id
 */
#define GEN_PKG_ID(i, p) \
	((i & IAXXX_PKG_MGMT_PKG_PROC_ID_PACKAGE_ID_MASK)| \
	((p << IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_POS) & \
	IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_MASK))

#define GET_PROC_ID_FROM_PKG_ID(pkg_id) \
	((pkg_id & IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_MASK) \
	>> IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_POS)

struct pkg_bin_info {
	uint32_t    version;
	uint32_t    entry_point;
	uint32_t    core_id;
	uint32_t    vendor_id;
	uint32_t    text_start_addr;
	uint32_t    text_end_addr;
	uint32_t    ro_data_start_addr;
	uint32_t    ro_data_end_addr;
	uint32_t    data_start_addr;
	uint32_t    data_end_addr;
	uint32_t    bss_start_addr;
	uint32_t    bss_end_addr;
};

struct pkg_mgmt_info {
	uint32_t req;
	uint32_t proc_id;
	uint32_t info;
	uint32_t p_text_addr;
	uint32_t v_text_addr;
	uint32_t text_size;
	uint32_t p_data_addr;
	uint32_t v_data_addr;
	uint32_t data_size;
	uint32_t entry_pt;
	uint32_t error;
};

/*****************************************************************************
 * iaxxx_core_plg_is_valid_pkg_id()
 * @brief validate the plugin package id
 *
 * @id              Plugin package Id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_pkg_id(uint32_t pkg_id)
{
	bool ret = true;

	if (pkg_id > IAXXX_PKG_ID_MASK) {
		pr_err("Invalid pkg id %d", pkg_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_pkg_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_priority()
 * @brief validate the plugin priority
 *
 * @id              Plugin priority
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_priority(uint32_t priority)
{
	bool ret = true;

	if (priority > (IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_MASK
				>> IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS)) {
		pr_err("Invalid priority %d", priority);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_priority);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_block_id()
 * @brief validate the plugin block id
 *
 * @id              Plugin block id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_block_id(uint32_t block_id)
{
	bool ret = true;

	if (block_id != IAXXX_BLOCK_0 && block_id != IAXXX_BLOCK_1) {
		pr_err("Invalid block id %d", block_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_block_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_plg_idx()
 * @brief validate the plugin idx
 *
 * @id              Plugin index
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_plg_idx(uint32_t plg_idx)
{
	bool ret = true;

	if (plg_idx > (IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_MASK
			>> IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS)) {
		pr_err("Invalid plugin idx %d", plg_idx);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_plg_idx);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_id()
 * @brief validate the plugin param id
 *
 * @id              Plugin param id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_id(uint32_t param_id)
{
	bool ret = true;

	if (param_id > (IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_MASK
			>> IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_POS)) {
		pr_err("Invalid param id %d", param_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_val()
 * @brief validate the plugin idx
 *
 * @id              Plugin param value
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_val(uint32_t param_val)
{
	bool ret = true;

	if (param_val > (IAXXX_PLUGIN_INS_GRP_PARAM_VAL_MASK
				>> IAXXX_PLUGIN_INS_GRP_PARAM_VAL_POS)) {
		pr_err("Invalid param val %d", param_val);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_val);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_blk_id()
 * @brief validate the plugin param blk id
 *
 * @id              Plugin param blk id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_blk_id(uint32_t param_id)
{
	bool ret = true;

	if (param_id > (IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_MASK
			>> IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_POS)) {
		pr_err("Invalid param block id %d", param_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_blk_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_blk_size()
 * @brief validate the plugin param blk size
 *
 * @id              Plugin param blk size
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_blk_size(uint32_t param_size)
{
	bool ret = true;

	if (param_size > (IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK
		>> IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS)) {
		pr_err("Invalid param blk size %d", param_size);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_blk_size);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_cfg_size()
 * @brief validate the plugin create cfg size
 *
 * @id              Plugin cfg size
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_cfg_size(uint32_t cfg_size)
{
	bool ret = true;

	if (cfg_size > (IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK
		>> IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS)) {
		pr_err("Invalid config size %d", cfg_size);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_cfg_size);

/*****************************************************************************
 * iaxxx_core_plugin_exist()
 * @brief check if plugin exists or not
 *
 * @priv  Pointer to iaxxx privata data structure
 * @inst_id  Instance id of a plugin
 * @ret pointer to plugin if exists, NULL otherwise
 ****************************************************************************/
struct iaxxx_plugin_data *iaxxx_core_plugin_exist(struct iaxxx_priv *priv,
							uint32_t inst_id)
{
	struct iaxxx_plugin_data *plugin_data = NULL;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	if (!list_empty_careful(&priv->iaxxx_state->plugin_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->plugin_head_list) {
			plugin_data = list_entry(node,
					struct iaxxx_plugin_data,
					plugin_node);
			if (plugin_data->inst_id == inst_id)
				goto exit;
			else
				plugin_data = NULL;
		}
	}
exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return plugin_data;
}
EXPORT_SYMBOL(iaxxx_core_plugin_exist);

/*****************************************************************************
 * iaxxx_core_pkg_exist()
 * @brief check if pkg exists or not
 *
 * @priv  Pointer to iaxxx privata data structure
 * @pkg_id  Package id of package
 * @ret pointer to package if exists, NULL otherwise
 ****************************************************************************/
struct iaxxx_pkg_data *iaxxx_core_pkg_exist(struct iaxxx_priv *priv,
						uint32_t pkg_id)
{
	struct iaxxx_pkg_data *pkg_data = NULL;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	if (!list_empty_careful(&priv->iaxxx_state->pkg_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->pkg_head_list) {
			pkg_data = list_entry(node,
				struct iaxxx_pkg_data, pkg_node);
			if (pkg_data->pkg_id == pkg_id)
				goto exit;
			else
				pkg_data = NULL;
		}
	}

exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return pkg_data;
}
EXPORT_SYMBOL(iaxxx_core_pkg_exist);

/*****************************************************************************
 * iaxxx_check_proc_pkg_plg_list_empty()
 * @brief check if package and plugin list is empty for this proc
 *
 * @priv Pointer to iaxxx privata data structure
 * @proc_id Processor id of package
 * @status true if package and plugin list is empty false otherwise.
 *
 * Return 0 on success & -ve value on failure
 ****************************************************************************/
int iaxxx_check_proc_pkg_plg_list_empty(struct iaxxx_priv *priv,
			uint32_t proc_id, bool *status)
{
	int rc = 0;
	struct iaxxx_pkg_data *pkg_data = NULL;
	struct iaxxx_plugin_data *plg_data = NULL;
	struct list_head *node, *tmp;
	uint32_t config_data = 0, block_id;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	/* Check if any plugin for the processor exist */
	if (!list_empty_careful(&priv->iaxxx_state->plugin_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->plugin_head_list) {
			plg_data = list_entry(node,
					struct iaxxx_plugin_data, plugin_node);
			if (plg_data->proc_id == proc_id) {
				pr_info("dev id-%d: Plugin exist",
								priv->dev_id);
				*status = false;
				goto exit;
			}
		}
	}

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);

	/* If any config is created for the plugin block */
	rc = regmap_read(priv->regmap_no_pm,
			 IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
			 &config_data);
	if (rc) {
		dev_err(priv->dev, "%s: read failed : %d\n", __func__, rc);
		goto exit;
	}

	if (config_data &
	    IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_IS_CREATE_CFG_MASK) {
		pr_info("dev id-%d: Plugin config exist", priv->dev_id);
		*status = false;
		goto exit;
	}

	/* Check if any package for the processor exist */
	if (!list_empty_careful(&priv->iaxxx_state->pkg_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->pkg_head_list) {
			pkg_data = list_entry(node,
					struct iaxxx_pkg_data, pkg_node);
			if (((pkg_data->proc_id &
				IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_MASK) >>
				IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_POS)
				== proc_id) {
				pr_info("dev id-%d: Package exist",
								priv->dev_id);
				*status = false;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return rc;
}
EXPORT_SYMBOL(iaxxx_check_proc_pkg_plg_list_empty);

/*****************************************************************************
 * iaxxx_core_pkg_plg_list_empty()
 * @brief check if package plugin list is empty
 *
 * @priv  Pointer to iaxxx privata data structure
 * @ret true if no plugin/package, false otherwise.
 ****************************************************************************/
bool iaxxx_core_pkg_plg_list_empty(struct iaxxx_priv *priv)
{
	bool status;
	uint32_t config_data, block_id;
	int rc;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	status = list_empty_careful(&priv->iaxxx_state->plugin_head_list);
	if (!status)
		goto exit;

	status = list_empty_careful(&priv->iaxxx_state->pkg_head_list);
	if (!status)
		goto exit;

	for (block_id = 0; block_id < MAX_CORES; block_id++) {
		/* If any config is created for the plugin block */
		rc = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
			&config_data);
		if (rc) {
			dev_err(priv->dev, "%s: read failed : %d\n", __func__,
				rc);
			status = false;
			goto exit;
		}

		if (config_data &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_IS_CREATE_CFG_MASK) {
			pr_info("dev id-%d: Plugin config exist",
								priv->dev_id);
			status = false;
			goto exit;
		}
	}

exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);

	return status;

}
EXPORT_SYMBOL(iaxxx_core_pkg_plg_list_empty);

/*****************************************************************************
 * iaxxx_clr_pkg_plg_list()
 * @brief del, destroy and unload all plugins and packages from list.
 *
 * @priv  Pointer to iaxxx privata data structure
 *
 * @ret  SUCCESS or FAIL
 ****************************************************************************/
int iaxxx_clr_pkg_plg_list(struct iaxxx_priv *priv)
{
	struct iaxxx_pkg_data *pkg_data;
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
							plugin_node);
		list_del(&plugin_data->plugin_node);
		kvfree(plugin_data);
	}

	list_for_each_safe(node, tmp, &priv->iaxxx_state->pkg_head_list) {
		pkg_data = list_entry(node, struct iaxxx_pkg_data, pkg_node);
		list_del(&pkg_data->pkg_node);
		kvfree(pkg_data);
	}
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);

	return 0;
}
EXPORT_SYMBOL(iaxxx_clr_pkg_plg_list);

static void iaxxx_add_plugin_to_list(struct iaxxx_priv *priv,
		struct iaxxx_plugin_data *plugin_data)
{
	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	list_add_tail(&plugin_data->plugin_node,
		&priv->iaxxx_state->plugin_head_list);
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
}

static void iaxxx_del_plugin_from_list(struct iaxxx_priv *priv,
		uint32_t inst_id)
{
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	/* Search and delete the node with mutex protection
	 * to avoid the case where plugin could be cleared
	 * simultaneously (ex: by crash recovery)
	 */
	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	if (!list_empty_careful(&priv->iaxxx_state->plugin_head_list)) {
		list_for_each_safe(node, tmp,
		    &priv->iaxxx_state->plugin_head_list) {
			plugin_data = list_entry(node,
					struct iaxxx_plugin_data,
					plugin_node);
			if (plugin_data->inst_id == inst_id) {
				list_del(&plugin_data->plugin_node);
				kvfree(plugin_data);
				goto exit;
			}
		}
	}
exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
}

/*****************************************************************************
 * iaxxx_core_create_plg()
 * @brief Create plugin instance
 *
 * @id		Plugin Instance Id
 * @param_id	Param Id
 * @param_val	Param value
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_create_plg(struct device *dev, uint32_t inst_id,
			uint32_t priority, uint32_t pkg_id,
			uint32_t plg_idx, uint8_t block_id,
			uint32_t config_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t status;
	uint32_t package;
	uint8_t proc_id;
	struct iaxxx_plugin_data *plugin_data;

	if (!priv)
		return ret;

	dev_info(dev,
		"%s: inst_id=%u prio=%u pkg_id=%u plg_idx=%u blk_id=%u cfg_id=%u\n",
		__func__, inst_id, priority, pkg_id, plg_idx,
		block_id, config_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check Package is loaded */
	package = pkg_id & IAXXX_PKG_ID_MASK;
	inst_id &= IAXXX_PLGIN_ID_MASK;
	if (!iaxxx_core_pkg_exist(priv, package))
		dev_warn(dev, "%s: Package 0x%x is not loaded by Host\n",
				__func__, pkg_id);

	/* Check if Plugin exist */
	if (iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x exist\n",
				__func__, inst_id);
		ret = -EEXIST;
		goto core_create_plugin_err;
	}

	if (!IAXXX_IS_VALID_GRP_ID(priv, IAXXX_BLOCK_PLUGIN, inst_id)) {
		ret = -EINVAL;
		goto core_create_plugin_err;
	}

	proc_id = IAXXX_BLOCK_ID_TO_PROC_ID(block_id);

	ret = iaxxx_check_and_powerup_proc(priv, proc_id);
	if (ret) {
		dev_err(dev,
			"%s: core %d bring up failed. rc %d\n", __func__,
			proc_id, ret);
		goto core_create_plugin_err;
	}

	/* Create pkg_id of Package ID using Package Index
	 * and Proc ID
	 */
	pkg_id = GEN_PKG_ID(package, proc_id);

	/* Update Package ID of plugin to be created */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_ORIGIN_ADDR,
				   inst_id),
		IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_MASK,
		pkg_id << IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_POS);

	if (ret) {
		dev_err(dev, "%s: Pkg ID write failed : %d\n", __func__, ret);
		goto core_create_plugin_err;
	}
	/* Update Plugin priority */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_CTRL_ADDR,
				   inst_id),
		IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_MASK,
		priority << IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS);
	if (ret) {
		dev_err(dev, "%s: Plg Priority write failed : %d\n",
								__func__, ret);
		goto core_create_plugin_err;
	}

	/* Update Plugin index */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_ORIGIN_ADDR,
				   inst_id),
		IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_MASK,
		plg_idx << IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS);
	if (ret) {
		dev_err(dev, "%s: plg indx write failed : %d\n", __func__, ret);
		goto core_create_plugin_err;
	}

	/* Update Plugin instance id in plg inst header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_CREATE_BLOCK_ADDR(block_id),
		1 << inst_id,
		1 << inst_id);
	if (ret) {
		dev_err(dev, "%s: plg inst id write failed : %d\n",
								__func__, ret);
		goto core_create_plugin_err;
	}

	/* Update Config_id of plugin to be created */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_ORIGIN_ADDR,
				   inst_id),
		IAXXX_PLUGIN_INS_GRP_ORIGIN_CONFIG_ID_MASK,
		config_id << IAXXX_PLUGIN_INS_GRP_ORIGIN_CONFIG_ID_POS);

	if (ret) {
		dev_err(dev, "%s: plg config id write failed : %d\n",
								__func__, ret);
		goto core_create_plugin_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN INS GRP ORIGIN ADDR Update blk failed : %d\n",
		__func__, ret);
		goto core_create_plugin_err;
	}

	/* Insert plugin node to the list */
	plugin_data = iaxxx_kvmalloc(sizeof(*plugin_data), __GFP_ZERO);
	if (!plugin_data) {
		ret = -ENOMEM;
		goto core_create_plugin_err;
	}

	plugin_data->plugin_state = IAXXX_PLUGIN_LOADED;
	plugin_data->inst_id = inst_id;
	plugin_data->proc_id = proc_id;
	plugin_data->block_id = block_id;

	iaxxx_add_plugin_to_list(priv, plugin_data);

core_create_plugin_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_create_plg);

/*****************************************************************************
 * iaxxx_core_change_plg_state()
 * @brief Change plugin state to enable/disable
 *
 * @inst_id	Plugin Instance Id
 * @is_enable	0 - Disable, 1 - Enable
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_change_plg_state(struct device *dev, uint32_t inst_id,
			uint8_t is_enable, uint8_t block_id)
{
	int ret = -EINVAL;
	uint32_t status;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_info(dev, "%s: inst_id:%u block_id:%u is_enable:%u\n",
				__func__, inst_id, block_id, is_enable);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check plugin instance is created */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created\n",
				__func__, inst_id);
		ret = -EEXIST;
		goto core_change_plg_state_err;
	}

	/* Set enable bit in plugin inst enable header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_ENABLE_BLOCK_ADDR(block_id),
		1 << inst_id,
		is_enable << inst_id);
	if (ret) {
		dev_err(dev, "%s: enb plg inst write failed : %d\n",
								__func__, ret);
		goto core_change_plg_state_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret)
		dev_err(dev,
		"%s: PLUGIN HDR ENABLE BLOCK ADDR Update blk failed : %d\n",
		__func__, ret);

core_change_plg_state_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_change_plg_state);

/*****************************************************************************
 * iaxxx_core_destroy_plg()
 * @brief Destroy plugin instance
 *
 * @inst_id	Plugin Instance Id
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_destroy_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_plugin_data *plugin_data;
	int ret = -EINVAL;
	uint32_t status = 0;

	if (!priv)
		return ret;

	dev_info(dev, "%s: inst_id:%u block_id:%u\n", __func__,
			inst_id, block_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check plugin instance is created */
	plugin_data = iaxxx_core_plugin_exist(priv, inst_id);
	if (!plugin_data) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created()\n",
			__func__, inst_id);
		ret = -ENOENT;
		goto core_destroy_plg_err;
	}

	/* Clear bit in plugin instance header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_CREATE_BLOCK_ADDR(block_id),
		1 << inst_id, 0);
	if (ret) {
		dev_err(dev, "%s: clr plg inst write failed : %d\n",
								__func__, ret);
		goto core_destroy_plg_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR CREATE BLOCK ADDR Update blk failed : %d\n",
		__func__, ret);
		goto core_destroy_plg_err;
	}

	/* Remove plugin node from list */
	iaxxx_del_plugin_from_list(priv, inst_id);

core_destroy_plg_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_destroy_plg);

/*****************************************************************************
 * iaxxx_core_reset_plg()
 * @brief Reset plugin instance
 *
 * @inst_id	Plugin Instance Id
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_reset_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id)
{
	int ret = -EINVAL;
	int rc;
	int status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;
	dev_info(dev, "%s: inst_id:%u block_id:%u\n", __func__,
			inst_id, block_id);
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check plugin instance is created */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created\n",
				__func__, inst_id);
		goto core_reset_plg_err;
	}

	/* Clear bit in plugin instance header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_RESET_BLOCK_ADDR(block_id),
		1 << inst_id,
		1 << inst_id);
	if (ret) {
		dev_err(dev, "%s: clr plg inst write failed : %d\n",
								__func__, ret);
		goto core_reset_plg_err;
	}
	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR RESET BLOCK ADDR Update blk failed : %d\n",
		__func__, ret);
		if (status) {
			/* Clear bit in plugin instance header */
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_RESET_BLOCK_ADDR(block_id),
				1 << inst_id, 0);
			if (rc) {
				dev_err(dev, "%s: clear failed : %d\n",
						__func__, rc);
				goto core_reset_plg_err;
			}
		}
		goto core_reset_plg_err;
	}

core_reset_plg_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_reset_plg);

/*****************************************************************************
 * iaxxx_core_plg_set_param_by_inst()
 * @brief Set a param in a plugin instance
 *
 * @id		Plugin Instance Id
 * @param_id	Param Id
 * @param_val	Param value
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc;

	if (!priv)
		return ret;

	dev_info(dev, "%s: inst_id=%u param_id=%u blk_id=%u param_val=%u\n",
		__func__, inst_id, param_id, block_id, param_val);

	inst_id &= IAXXX_PLGIN_ID_MASK;
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created\n",
				__func__, inst_id);
		goto plg_set_param_inst_err;
	}
	ret = regmap_write(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_PARAM_ID_ADDR,
		inst_id), param_id);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto plg_set_param_inst_err;
	}

	ret = regmap_write(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_PARAM_ADDR,
					   inst_id), param_val);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto plg_set_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_SET_PARAM_REQ_BLOCK_ADDR(block_id),
		1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN INS GRP PARAM ADDR update bit failed : %d\n",
		__func__, ret);
		goto plg_set_param_inst_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR SET PARAM REQ BLOCK ADDR Update blk failed : %d\n",
		__func__, ret);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_SET_PARAM_REQ_BLOCK_ADDR
				(block_id),
				1 << inst_id, 0);
			if (rc)
				dev_err(dev, "%s: clear bit failed : %d\n",
						__func__, rc);
		}
		goto plg_set_param_inst_err;
	}

plg_set_param_inst_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_set_param_by_inst);


/*****************************************************************************
 * iaxxx_core_plg_get_param_by_inst()
 * @brief get a param in a plugin instance
 *
 * @id		Plugin Instance Id
 * @param_id	Param Id
 * @param_val	return param value
 * @block_id	Update block id
 *
 * @ret 0 in case of success, -EINVAL in case of error.
 ****************************************************************************/
int iaxxx_core_plg_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	int rc;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;

	dev_info(dev, "%s: inst_id=%u param_id=%u blk_id=%u\n",
		__func__, inst_id, param_id, block_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created\n",
				__func__, inst_id);
		goto plg_get_param_inst_err;
	}
	ret = regmap_write(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_PARAM_ID_ADDR,
		inst_id), param_id);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto plg_get_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
			IAXXX_PLUGIN_HDR_GET_PARAM_REQ_BLOCK_ADDR(block_id),
			1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto plg_get_param_inst_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR GET PARAM REQ BLOCK ADDR Update blk failed : %d\n",
		__func__, ret);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_GET_PARAM_REQ_BLOCK_ADDR
				(block_id),
				1 << inst_id, 0);
			if (rc)
				dev_err(dev, "%s: clear bit failed : %d\n",
						__func__, rc);
		}
		goto plg_get_param_inst_err;
	}

	ret = regmap_read(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_PARAM_ADDR,
					   inst_id), param_val);
	if (ret) {
		dev_err(dev, "%s: read failed : %d\n", __func__, ret);
		goto plg_get_param_inst_err;
	}

plg_get_param_inst_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_get_param_by_inst);

/*****************************************************************************
 * iaxxx_core_set_create_cfg()
 * @brief Set a param in a plugin instance
 *
 * @inst_id	Plugin Instance Id
 * @cfg_size	Create cfg size
 * @cfg_val	Config val
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_set_create_cfg(struct device *dev, uint32_t inst_id,
			uint32_t cfg_size, uint64_t cfg_val, uint32_t block_id,
			char *file)
{
	int ret = -EINVAL;
	int status;
	uint32_t reg_addr;
	uint32_t val;
	uint32_t reg_val;
	uint32_t proc_id = IAXXX_BLOCK_ID_TO_PROC_ID(block_id);
	const struct firmware *fw = NULL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint8_t *data = NULL;

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;

	dev_info(dev, "%s: inst_id=%u cfg_size=%u blk_id=%u\n",
		__func__, inst_id, cfg_size, block_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* If plugin instance already exist */
	if (iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x already exist\n",
				__func__, inst_id);
		ret = -EEXIST;
		goto set_create_cfg_err;
	}

	if (!IAXXX_IS_VALID_GRP_ID(priv, IAXXX_BLOCK_PLUGIN, inst_id)) {
		ret = -EINVAL;
		goto set_create_cfg_err;
	}

	if (file[0] != IAXXX_INVALID_FILE) {
		dev_info(dev, "%s: Download config : %s\n", __func__, file);
		ret = request_firmware(&fw, file, priv->dev);
		if (ret) {
			dev_err(dev, "Firmware file not found = %d\n", ret);
			ret = -EINVAL;
			goto set_create_cfg_err;
		}
		cfg_size = fw->size;
		dev_dbg(dev, "%s: cfg_size %d\n", __func__, cfg_size);
	}
	if (cfg_size > sizeof(uint32_t)) {
		if (file[0] == IAXXX_INVALID_FILE) {
			dev_dbg(dev, "%s: invalid cfg_val %llx\n",
				__func__, cfg_val);
			/* MSB word should be the first word to be written */
			cfg_val = (cfg_val >> IAXXX_BITS_SWAP) |
				(cfg_val << IAXXX_BITS_SWAP);
			dev_dbg(dev, "%s: After correction, cfg_val 0x%llx\n",
					__func__, cfg_val);
			iaxxx_copy_le32_to_cpu(&cfg_val, &cfg_val,
					       sizeof(cfg_val));
		} else {
			data = iaxxx_kvmalloc(cfg_size, 0);
			if (!data) {
				ret = -ENOMEM;
				goto set_create_cfg_err;
			}
			iaxxx_copy_le32_to_cpu(data, fw->data, cfg_size);
		}

		/* Write to the ParamBlkCtrl register */
		val = (((cfg_size >> 2) <<
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS) &
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK) |
			((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK) |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_REQ_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_IS_CREATE_CFG_MASK;

		ret = iaxxx_check_and_powerup_proc(priv, proc_id);
		if (ret) {
			dev_err(dev,
				"%s: core %d bring up failed. rc %d\n",
				__func__, proc_id, ret);
			goto set_create_cfg_err;
		}

		ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
			val);
		if (ret) {
			dev_err(dev, "%s: write failed : %d\n", __func__, ret);
			goto set_create_cfg_err;
		}

		ret = iaxxx_send_update_block_request(dev, &status, block_id);
		if (ret) {
			dev_err(dev,
			"%s: PLUGIN HDR PARAM BLK CTRL BLOCK ADDR Update blk failed : %d\n",
								__func__, ret);
			goto set_create_cfg_err;
		}

		ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_ADDR_BLOCK_ADDR(block_id),
			&reg_addr);
		if (ret || !reg_addr) {
			dev_err(dev, "%s: read failed : %d addr = 0x%x\n",
				__func__, ret, reg_addr);
			goto set_create_cfg_err;
		}
		pr_debug("dev id-%d: Configuration addr 0x%x",
							priv->dev_id, reg_addr);

		if (priv->bus_write) {
			if (file[0] == IAXXX_INVALID_FILE)
				ret = priv->bus_write(dev, reg_addr, &cfg_val,
					sizeof(cfg_val) / sizeof(uint32_t));
			else {
				ret = priv->bus_write(dev, reg_addr, data,
						cfg_size / sizeof(uint32_t));
			}
			if (ret) {
				dev_err(dev, "%s: Blk write failed : %d\n",
								__func__, ret);
				goto set_create_cfg_err;
			}
		} else {
			dev_err(dev, "%s: Raw blk write() missing\n", __func__);
			goto set_create_cfg_err;
		}
	} else {
		if (file[0] == IAXXX_INVALID_FILE)
			reg_val = (uint32_t)cfg_val;
		else
			iaxxx_copy_le32_to_cpu(&reg_val, fw->data, cfg_size);
		pr_debug("dev id-%d: reg_val 0x%x", priv->dev_id, reg_val);

		ret = regmap_write(priv->regmap,
			IAXXX_GET_GRP_ADDR(priv,
			IAXXX_PLUGIN_INS_GRP_CREATION_CFG_ADDR, inst_id),
			reg_val);
		if (ret) {
			dev_err(dev, "%s: write failed : %d\n", __func__, ret);
			goto set_create_cfg_err;
		}
	}

set_create_cfg_err:
	if (fw)
		release_firmware(fw);
	kvfree(data);
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_set_create_cfg);

/**************************************************************************
 * iaxxx_core_set_param_blk
 * @brief Set parameter block on a plugin from given buffer
 *
 * @dev			device structure
 * @inst_id		Instance ID
 * @block_id		Block ID
 * @param_blk_id	Parameter block id
 * @blk_size		Size of buffer data in bytes
 * @ptr_blk		Pointer to buffer data
 * @ret - 0 on success, on failure < 0
 **************************************************************************/
int iaxxx_core_set_param_blk(
			struct device *dev,
			uint32_t inst_id, uint32_t blk_size,
			const void *ptr_blk, uint32_t block_id,
			uint32_t param_blk_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	int rc;
	uint32_t reg_addr;
	uint32_t reg_val;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;
	dev_info(dev, "%s: inst_id=%u blk_size=%u blk_id=%u id=%u\n",
		__func__, inst_id, blk_size, block_id, param_blk_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created n",
				__func__, inst_id);
		ret = -EEXIST;
		goto set_param_blk_err;
	}

	/* Write the PluginHdrParamBlkCtrl register */
	/* The block size is divided by 4 here because this function gets it
	 * as block size in bytes but firmware expects in 32bit words.
	 */
	reg_val = (((blk_size >> 2) <<
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS) &
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK);
	reg_val |= ((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK);
	reg_val |= IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_REQ_MASK;
	ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
			reg_val);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto set_param_blk_err;
	}

	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_HDR_BLOCK_ADDR(block_id),
		param_blk_id);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto set_param_blk_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR PARAM BLK ID BLOCK ADDR Update blk failed : %d after slot-id config\n",
			__func__, ret);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_REQ_MASK,
			0);
			if (rc) {
				dev_err(dev, "%s: clear failed : %d\n",
					__func__, rc);
				goto set_param_blk_err;
			}
		}
		goto set_param_blk_err;
	}
	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_ADDR_BLOCK_ADDR(block_id),
			&reg_addr);
	if (ret) {
		dev_err(dev, "%s: read failed : %d\n", __func__, ret);
		goto set_param_blk_err;
	}

	if (priv->bus_write) {
		ret = priv->bus_write(dev, reg_addr, ptr_blk,
				      blk_size / sizeof(uint32_t));
		if (ret) {
			dev_err(dev, "%s: Raw blk write failed : %d\n",
								__func__, ret);
			goto set_param_blk_err;
		}
	} else {
		dev_err(dev, "%s: Raw blk write not supported\n", __func__);
		goto set_param_blk_err;
	}

	/* The block size is divided by 4 here because this function gets it
	 * as block size in bytes but firmware expects in 32bit words.
	 */
	reg_val = (((blk_size >> 2)
		<< IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK);
	reg_val |= ((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK);
	reg_val |= IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_DONE_MASK;
	ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
			reg_val);

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR PARAM BLK CTRL BLOCK ADDR Update blk failed : %d after plugin ctrl block config\n",
		__func__, ret);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id),
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_DONE_MASK,
			0);
			if (rc)
				dev_err(dev, "%s: clear failed : %d\n",
					__func__, rc);
			}
	}

set_param_blk_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_set_param_blk);

int iaxxx_core_set_param_blk_from_file(
			struct device *dev,
			uint32_t inst_id,
			uint32_t block_id,
			uint32_t param_blk_id,
			const char *file)
{
	int ret = -EINVAL;
	uint8_t *data = NULL;
	const struct firmware *fw = NULL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	if (file && IAXXX_INVALID_FILE != file[0]) {
		dev_info(dev, "%s: Download param blk from file : %s\n",
								__func__, file);
		ret = request_firmware(&fw, file, priv->dev);
		if (ret) {
			dev_err(dev, "Firmware file %s not found = %d\n", file,
				ret);
			goto iaxxx_core_set_param_blk_from_file_err;
		}

		if (!fw->size || !fw->data) {
			dev_err(dev, "invalid fw data or size\n");
			ret = -EINVAL;
			goto iaxxx_core_set_param_blk_from_file_err;
		}

		data = iaxxx_kvmalloc(fw->size, 0);
		if (!data) {
			ret = -ENOMEM;
			goto iaxxx_core_set_param_blk_from_file_err;
		}

		iaxxx_copy_le32_to_cpu(data, fw->data, fw->size);
		ret = iaxxx_core_set_param_blk(
				dev, inst_id, fw->size, fw->data,
				block_id, param_blk_id);
	}

iaxxx_core_set_param_blk_from_file_err:
	kvfree(data);
	if (fw)
		release_firmware(fw);

	return ret;
}
EXPORT_SYMBOL(iaxxx_core_set_param_blk_from_file);

/*****************************************************************************
 * iaxxx_core_set_event()
 * @brief Write the event enable mask to a plugin instance
 *
 * @inst_id		Plugin Instance Id
 * @event_enable_mask	Event enable mask
 * @block_id		Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_set_event(struct device *dev, uint8_t inst_id,
			uint32_t event_enable_mask, uint32_t block_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t status = 0;
	int rc;

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;
	dev_dbg(dev, "%s: inst_id:%u block_id:%u event_en_mask:%x\n",
			__func__, inst_id, block_id, event_enable_mask);
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "%s: Plugin instance 0x%x is not created\n",
				__func__, inst_id);
		goto set_event_err;
	}

	ret = regmap_write(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PLUGIN_INS_GRP_EVT_EN_ADDR,
			inst_id), event_enable_mask);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto set_event_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_EVT_UPDATE_BLOCK_ADDR(block_id),
		1 << inst_id, 1 << inst_id);
	if (ret)
		dev_err(dev,
		"%s: PLUGIN INS GRP EVT EN ADDR update bit failed : %d\n",
		__func__, ret);

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev,
		"%s: PLUGIN HDR EVT UPDATE BLOCK ADDR Update blk failed : %d\n",
		__func__, ret);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_EVT_UPDATE_BLOCK_ADDR
				(block_id),
				1 << inst_id, 0);
			if (rc)
				dev_err(dev, "%s: clear failed : %d\n",
						__func__, rc);
		}
	}

set_event_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;

}
EXPORT_SYMBOL(iaxxx_core_set_event);

static int write_pkg_info(bool update, struct iaxxx_priv *priv, uint32_t pkg_id,
		struct pkg_bin_info bin_info, struct pkg_mgmt_info *pkg)
{
	int rc;
	struct device *dev = priv->dev;
	uint32_t status;
	uint32_t block_id;

	pkg_id &= IAXXX_PKG_ID_MASK;
	if (update) {
		dev_info(dev,
		"Text:start:0x%x end:0x%x\nRO data:start 0x%x end:0x%x\n",
		bin_info.text_start_addr, bin_info.text_end_addr,
		bin_info.ro_data_start_addr, bin_info.ro_data_end_addr);
		dev_info(dev,
		"Data:start 0x%x end 0x%x\nBSS:start 0x%x end 0x%x\n",
		bin_info.data_start_addr, bin_info.data_end_addr,
		bin_info.bss_start_addr, bin_info.bss_end_addr);

		pkg->req = 1 << IAXXX_PKG_MGMT_PKG_REQ_LOAD_POS;
		pkg->proc_id = GEN_PKG_ID(pkg_id, bin_info.core_id);
		pkg->info = bin_info.core_id |
			(bin_info.vendor_id <<
			 IAXXX_PKG_MGMT_PKG_INFO_VENDOR_ID_POS);
		pkg->v_text_addr = bin_info.text_start_addr;
		pkg->text_size = bin_info.text_end_addr -
			bin_info.text_start_addr;
		pkg->v_data_addr = bin_info.ro_data_start_addr;
		pkg->data_size = bin_info.bss_end_addr -
			bin_info.ro_data_start_addr;
		pkg->entry_pt = bin_info.entry_point;
	} else
		pkg->req = 1;
	/* Write Package Binary information */
	rc = regmap_bulk_write(priv->regmap, IAXXX_PKG_MGMT_PKG_REQ_ADDR, pkg,
					sizeof(struct pkg_mgmt_info) >> 2);
	if (rc) {
		dev_err(dev, "%s: Pkg info write failed : %d\n", __func__, rc);
		return rc;
	}
	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(bin_info.core_id);
	rc = iaxxx_send_update_block_request(dev, &status, block_id);
	if (rc) {
		dev_err(dev,
		"%s: PKG MGMT PKG REQ ADDR Update blk failed : %d\n",
		__func__, rc);
		return rc;
	}
	return rc;
}

static uint32_t get_physical_address(uint32_t addr,
		uint32_t text, uint32_t data, struct pkg_bin_info bin_info)
{
	/* Calculate the physical address to write to */
	if ((addr >= bin_info.text_start_addr)
			&& (addr <= bin_info.text_end_addr))
		return (text + (addr - bin_info.text_start_addr));
	else
		return (data + (addr - bin_info.ro_data_start_addr));
}

static int iaxxx_download_pkg(struct iaxxx_priv *priv,
		const struct firmware *fw, uint32_t pkg_id, uint32_t *proc_id)
{
	const uint8_t *data;
	struct device *dev = priv->dev;
	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };
	/* Checksum variable */
	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;
	struct pkg_bin_info bin_info = {0};
	uint32_t *word_data;
	int i, j;
	int rc = 0;
	uint32_t text_phy_addr = 0;
	uint32_t data_phy_addr = 0;
	uint8_t *buf_data;
	struct pkg_mgmt_info pkg = {0};

	dev_dbg(dev, "%s: enter\n", __func__);
	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad package binary file (too small)\n");
		return -EINVAL;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad Package binary file, rc %d\n", rc);
		return rc;
	}
	/* Include file header fields as part of the checksum */
	CALC_FLETCHER16(header.number_of_sections, sum1, sum2);
	CALC_FLETCHER16(header.entry_point, sum1, sum2);

	/* Find the Binary info section */
	for (i = 0; i < header.number_of_sections; i++) {
		/* Load the next data section */
		if (((data - fw->data) + sizeof(file_section)) > fw->size)
			return -EINVAL;
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);
		/* Check for the magic number for the start of info section */
		if (file_section.start_address == IAXXX_BIN_INFO_SEC_ADDR) {
			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);
			if (((data - fw->data) + sizeof(bin_info))
				> fw->size)
				return -EINVAL;
			iaxxx_copy_le32_to_cpu
				(&bin_info, data, sizeof(bin_info));
			word_data = (uint32_t *)&bin_info;
			for (j = 0 ; j < file_section.length; j++)
				CALC_FLETCHER16(word_data[j], sum1, sum2);
			data += sizeof(bin_info);

			rc = iaxxx_check_and_powerup_proc(priv,
							bin_info.core_id);
			if (rc) {
				dev_err(dev,
					"%s: core %d bring up failed. rc %d\n",
					__func__, bin_info.core_id, rc);
				return rc;
			}

			rc = write_pkg_info(true, priv, pkg_id, bin_info, &pkg);
			if (rc) {
				dev_err(dev, "%s: Pkg info failed : %d\n",
								__func__, rc);
				return rc;
			}
			break;
		} else if (file_section.length > 0)
			data += file_section.length * sizeof(uint32_t);
	}
	/* Read text and data physical address */
	rc = regmap_read(priv->regmap,
			IAXXX_PKG_MGMT_PKG_IADDR_P_ADDR, &text_phy_addr);
	if (rc) {
		dev_err(dev, "%s: Text physical addr read failed : %d\n",
								__func__, rc);
		return rc;
	}
	rc = regmap_read(priv->regmap,
			IAXXX_PKG_MGMT_PKG_DADDR_P_ADDR, &data_phy_addr);
	if (rc) {
		dev_err(dev, "%s: Data physical addr read failed : %d\n",
								__func__, rc);
		return rc;
	}
	dev_dbg(dev, "%s: Text physical addr:0x%x Data physical addr 0x%x\n",
					__func__, text_phy_addr, data_phy_addr);

	data = fw->data + sizeof(header);
	/* Download sections except binary info and checksum */
	for (i = 0; i < header.number_of_sections; i++) {
		if (((data - fw->data) + sizeof(file_section)) > fw->size)
			return -EINVAL;
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);
		dev_dbg(dev, "%s: Section%d addr 0x%x length 0x%x\n", __func__,
			i, file_section.start_address, file_section.length);
		if (file_section.start_address == IAXXX_BIN_INFO_SEC_ADDR)
			data += sizeof(bin_info);
		else if (file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);
			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);
			file_section.start_address =
				get_physical_address(file_section.start_address,
					text_phy_addr, data_phy_addr, bin_info);
			dev_dbg(dev, "%s: Physical address 0x%x\n",
					__func__, file_section.start_address);
			buf_data = iaxxx_kvmalloc(file_section.length *
					   sizeof(uint32_t), __GFP_ZERO);
			if (!buf_data)
				return -ENOMEM;

			if (((data - fw->data) + (file_section.length
				* sizeof(uint32_t))) > fw->size) {
				kvfree(buf_data);
				return -EINVAL;
			}
			iaxxx_copy_le32_to_cpu(buf_data, data,
					file_section.length * sizeof(uint32_t));
			word_data = (uint32_t *)buf_data;
			for (j = 0 ; j < file_section.length; j++)
				CALC_FLETCHER16(word_data[j], sum1, sum2);
			rc = iaxxx_download_section(priv, data, &file_section,
								priv->regmap);
			if (rc) {
				dev_err(dev, "%s() download section fail %d\n",
								__func__, rc);
				kvfree(buf_data);
				return rc;
			}
			data += file_section_bytes;
			kvfree(buf_data);
		}
	}
	/* If the last section length is 0, then verify the checksum */
	if (file_section.length == 0) {
		uint32_t checksum = (sum2 << 16) | sum1;

		dev_info(dev, "final checksum from chip = 0x%.08X\n", checksum);
		if (checksum != file_section.start_address) {
			rc = -EINVAL;
			dev_err(dev, "%s: mismatch 0x%.08X != 0x%.08X\n",
				__func__, checksum, file_section.start_address);
			return rc;
		}
	}
	/* Write zeros to BSS region */
	if (bin_info.bss_start_addr != bin_info.bss_end_addr) {
		file_section.start_address = data_phy_addr +
			(bin_info.bss_start_addr - bin_info.ro_data_start_addr);
		file_section.length = (bin_info.bss_end_addr
				- bin_info.bss_start_addr) >> 2;
		buf_data = iaxxx_kvmalloc(file_section.length *
					  sizeof(uint32_t), __GFP_ZERO);
		if (!buf_data)
			return -ENOMEM;

		rc = iaxxx_download_section(priv, buf_data, &file_section,
								priv->regmap);
		kvfree(buf_data);
		if (rc) {
			dev_err(dev, "%s() download bss section fail %d\n",
							__func__, rc);
			return rc;
		}
	}
	/* Write to Package Management ARB */
	rc = write_pkg_info(false, priv, pkg_id, bin_info, &pkg);
	if (rc) {
		dev_err(dev, "%s() Pkg info write fail %d\n", __func__, rc);
		return rc;
	}
	*proc_id = pkg.proc_id;
	return 0;
}

static int iaxxx_unload_pkg(struct iaxxx_priv *priv, uint32_t pkg_id,
			    uint32_t proc_id)
{
	struct device *dev = priv->dev;
	uint32_t status;
	uint32_t block_id;
	int rc = 0;
	uint32_t proc_pkg_id = GEN_PKG_ID(pkg_id, proc_id);

	/* Write the package id and proc id */
	rc = regmap_write(priv->regmap, IAXXX_PKG_MGMT_PKG_PROC_ID_ADDR,
				proc_pkg_id);
	if (rc) {
		dev_err(dev,
			"%s: Write to package pkg id (%d) register failed : %d\n",
			__func__, pkg_id, rc);
		return rc;
	}

	/* Write the request to unload */
	rc = regmap_write(priv->regmap, IAXXX_PKG_MGMT_PKG_REQ_ADDR,
				IAXXX_PKG_MGMT_PKG_REQ_UNLOAD_MASK);
	if (rc) {
		dev_err(dev,
			"%s: Write to package (%d) request register failed : %d\n",
			__func__, pkg_id, rc);
		return rc;
	}

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);
	rc = iaxxx_send_update_block_request(dev, &status, block_id);
	if (rc) {
		dev_err(dev,
		"%s: PKG MGMT PKG REQ ADDR Update blk failed : %d\n",
		__func__, rc);
		return rc;
	}
	return 0;
}

/*****************************************************************************
 * iaxxx_package_load()
 * @brief Load the package
 *
 * @pkg_name	Package binary name
 * @pkg_id	Package Id
 * @proc_id	Package Id and Core Id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_package_load(struct device *dev, const char *pkg_name,
					uint32_t pkg_id, uint32_t *proc_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_pkg_data *pkg_data;
	const struct firmware *fw = NULL;
	int rc = 0;

	dev_info(dev, "%s: enter\n", __func__);

	if (!pkg_name) {
		dev_err(dev, "%s: Package name is NULL\n", __func__);
		return -EINVAL;
	}
	dev_info(dev, "Download Package %s, pkg id %d\n", pkg_name, pkg_id);

	pkg_id &= IAXXX_PKG_ID_MASK;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* If package already exist */
	pkg_data = iaxxx_core_pkg_exist(priv, pkg_id);
	if (pkg_data) {
		dev_err(dev, "%s: Package 0x%x already exist\n",
				__func__, pkg_id);
		*proc_id = pkg_data->proc_id;
		rc = -EEXIST;
		goto pkg_exist_err;
	}

	rc = request_firmware(&fw, pkg_name, priv->dev);
	if (rc) {
		dev_err(dev, "Firmware file %s not found : %d\n",
							pkg_name, rc);
		goto request_fw_err;
	}
	rc = iaxxx_download_pkg(priv, fw, pkg_id, proc_id);
	if (rc) {
		dev_err(dev, "%s: pkg load failed : %d\n", __func__, rc);
		rc = -EINVAL;
		goto out;
	}

	/* Insert package node to the list */
	pkg_data = iaxxx_kvmalloc(sizeof(*pkg_data), __GFP_ZERO);
	if (!pkg_data) {
		rc = -ENOMEM;
		goto out;
	}
	pkg_data->proc_id = *proc_id;
	pkg_data->pkg_id = pkg_id;
	pkg_data->pkg_state = IAXXX_PKG_LOADED;
	list_add_tail(&pkg_data->pkg_node, &priv->iaxxx_state->pkg_head_list);

out:
	if (fw)
		release_firmware(fw);
request_fw_err:
pkg_exist_err:
	mutex_unlock(&priv->plugin_lock);
	return rc;
}
EXPORT_SYMBOL(iaxxx_package_load);

/*****************************************************************************
 * iaxxx_package_unload()
 * @brief Load the package
 *
 * @pkg_id	Package id
 * @proc_id	Core Id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_package_unload(struct device *dev, uint32_t pkg_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_pkg_data *pkg_data;
	uint32_t proc_id;
	int rc = 0;

	dev_info(dev, "Unloading Package id %d ...\n", pkg_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	pkg_id &= IAXXX_PKG_ID_MASK;
	pkg_data = iaxxx_core_pkg_exist(priv, pkg_id);
	if (!pkg_data) {
		dev_err(dev, "%s: pkg not loaded already %d\n",
			__func__, pkg_id);
		rc = -ENOENT;
		goto out;
	}
	proc_id = GET_PROC_ID_FROM_PKG_ID(pkg_data->proc_id);

	rc = iaxxx_unload_pkg(priv, pkg_id, proc_id);
	if (rc) {
		dev_err(dev, "%s: pkg unload failed : %d\n", __func__,
			rc);
		goto out;
	}

	dev_info(dev, "%s: pkg_id:0x%x proc_id:%u\n", __func__,
							pkg_id, proc_id);
	/* Remove package node from the list */
	list_del(&pkg_data->pkg_node);
	kvfree(pkg_data);

out:
	mutex_unlock(&priv->plugin_lock);
	return rc;
}
EXPORT_SYMBOL(iaxxx_package_unload);

unsigned int iaxxx_get_num_packages(struct iaxxx_priv *priv)
{
	/* num_of_grp represents num of packages here */
	return priv->rbdt_info[IAXXX_BLOCK_PACKAGE].num_of_grp;
}

int iaxxx_core_plg_get_package_version(struct device *dev,
		uint8_t inst_id, char *ver, uint32_t len)
{
	int rc = 0, i, found = 0;
	u32 pkg_id, pkg_info;
	u32 num_pkgs;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	rc = regmap_read(priv->regmap, IAXXX_GET_GRP_ADDR(priv,
		IAXXX_PLUGIN_INS_GRP_ORIGIN_ADDR, inst_id), &pkg_id);
	if (rc) {
		dev_err(dev, "%s: Error reading PLGIN Origin Reg. rc : %d\n",
							__func__, rc);
		return rc;
	}

	pkg_id &= IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_MASK;

	pr_debug("dev id-%d: inst id %d, pkg id 0x%x",
					priv->dev_id, inst_id, pkg_id);

	num_pkgs = iaxxx_get_num_packages(priv);

	for (i = 0; i < num_pkgs; i++) {
		rc = regmap_read(priv->regmap, IAXXX_GET_GRP_ADDR(priv,
			IAXXX_PKG_GRP_PKG_INFO_ADDR, i), &pkg_info);
		if (rc) {
			dev_err(dev, "%s: pkg info Reg Fail. rc : %d\n",
								__func__, rc);
			return rc;
		}

		pr_debug("dev id-%d: pkg info 0x%08x", priv->dev_id, pkg_info);

		/* Extract the package ID from the info */
		pkg_info &= (IAXXX_PKG_GRP_PKG_INFO_PACKAGE_ID_MASK |
					IAXXX_PKG_GRP_PKG_INFO_PROC_ID_MASK);

		if (pkg_id == pkg_info) {
			pr_debug("dev id-%d: pkg info match found at index %d",
							priv->dev_id, i);
			found = 1;
			break;
		}
	}

	if (found == 0) {
		pr_info("dev id-%d: pkg info not found", priv->dev_id);
		return -ENOENT;
	}

	return get_version_str(priv,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_PKG_GRP_PKG_VER_STR_ADDR, i),
		ver, len);
}
EXPORT_SYMBOL(iaxxx_core_plg_get_package_version);

int iaxxx_core_plg_get_plugin_version(struct device *dev,
		uint8_t inst_id, char *ver, uint32_t len)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	return get_version_str(priv, IAXXX_GET_GRP_ADDR(priv,
		IAXXX_PLUGIN_INS_GRP_PLUGIN_VER_STR_ADDR, inst_id), ver, len);
}
EXPORT_SYMBOL(iaxxx_core_plg_get_plugin_version);

/*****************************************************************************
 * iaxxx_set_channel_gain()
 * @brief Change the channel gain
 *
 * @id		Channel id
 * @target_gain	Target gain (in dB) (-128 dB to 127 dB)
 * @gain_ramp	Gain ramp (in dB per second)
 * @block_id	Block id
 *
 * @ret 0 on success, negative in case of error
 ****************************************************************************/
int iaxxx_set_channel_gain(struct device *dev, uint8_t id,
			int8_t target_gain, uint16_t gain_ramp,
			uint8_t block_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	u32 status;
	int ret = 0;

	dev_info(dev, "%s: enter\n", __func__);

	if (!IAXXX_IS_VALID_GRP_ID(priv, IAXXX_BLOCK_CHANNEL, id)) {
		ret = -EINVAL;
		goto set_chan_gain_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, id),
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK,
		target_gain << IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS);
	if (ret) {
		dev_err(dev, "%s: target gain set failed : %d\n",
								__func__, ret);
		goto set_chan_gain_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, id),
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK,
		gain_ramp << IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS);
	if (ret) {
		dev_err(dev, "%s: target gain set failed : %d\n",
								__func__, ret);
		goto set_chan_gain_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_CH_HDR_CH_GAIN_ADDR, 1 << id, 1 << id);
	if (ret) {
		dev_err(dev, "%s: Channel update failed : %d\n",
								__func__, ret);
		goto set_chan_gain_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret)
		dev_err(dev, "%s: CH HDR CH GAIN ADDR Update blk failed : %d\n",
		__func__, ret);

set_chan_gain_err:
	return ret;
}
EXPORT_SYMBOL(iaxxx_set_channel_gain);

/*****************************************************************************
 * iaxxx_get_package_type()
 * @brief Get the package type
 *
 * @pkg_id	package id
 * @pkg_type	Static, dynamic or invalid(if pkg doesn't exist)
 *
 * @ret 0 on success, negative in case of error
 ****************************************************************************/
int iaxxx_get_package_type(struct device *dev, uint32_t pkg_id,
				uint8_t *pkg_type)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	u32 max_pkg_num, pkg_info;
	int ret, i;

	*pkg_type = INVALID_PKG;

	/* Power Up the Non-Control processors */
	ret = iaxxx_check_and_powerup_proc(priv, IAXXX_HMD_ID);
	if (ret) {
		dev_err(dev,
			"%s: core %d bring up failed. rc %d\n", __func__,
			IAXXX_HMD_ID, ret);
		return ret;
	}

	/* Update Package ID of plugin to be created */
	max_pkg_num = iaxxx_get_num_packages(priv);

	for (i = 0; i < max_pkg_num; i++) {
		ret = regmap_read(priv->regmap, IAXXX_GET_GRP_ADDR(priv,
			IAXXX_PKG_GRP_PKG_INFO_ADDR, i), &pkg_info);
		if (ret) {
			dev_err(dev, "%s: Error reading PLGIN Origin Reg. rc : %d\n",
				__func__, ret);
			return ret;
		}

		if ((pkg_info & IAXXX_PKG_GRP_PKG_INFO_PACKAGE_ID_MASK) ==
								pkg_id) {
			if (pkg_info &
			    IAXXX_PKG_GRP_PKG_INFO_PACKAGE_TYPE_MASK) {
				pr_info("dev id-%d: Static package",
								priv->dev_id);
				*pkg_type = IAXXX_STATIC_PKG;
			} else {
				pr_info("dev id-%d: Dynamic package",
								priv->dev_id);
				*pkg_type = IAXXX_DYNAMIC_PKG;
			}

			break;
		}

	}

	return 0;
}
EXPORT_SYMBOL(iaxxx_get_package_type);
