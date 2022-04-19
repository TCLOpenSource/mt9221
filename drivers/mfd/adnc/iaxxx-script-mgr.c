/*
 * iaxxx-script-mgr.c -- IAxxx Script Manager Service
 *
 * Copyright 2016 Knowles Corporation
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

#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include <linux/mfd/adnc/iaxxx-script-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-script-mgmt.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include "iaxxx.h"

static int iaxxx_download_script(struct iaxxx_priv *priv,
			const struct firmware *fw, uint16_t script_id)
{
	const uint8_t *data;
	uint8_t *buf_data;

	/* Checksum variable */
	uint32_t sum1 = 0xffff, sum2 = 0xffff;
	uint32_t finalsum1 = 0xffff, finalsum2 = 0xffff;

	uint32_t *word_data;
	uint32_t value = 0;
	uint32_t status, checksum, script_addr;
	size_t script_size = 0, scrpt_sz_in_words;
	int i, ret, rc = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s:\n", __func__);

	/* fw->size = script_size + CRC_value (sizeof(uint32_t)).
	 * so fw size should be > 4 bytes
	 */
	if (fw->size <= 4) {
		dev_err(dev,
			"%s: bad script binary file size: %zu\n",
			__func__, fw->size);
		return -EINVAL;
	}
	script_size = fw->size - sizeof(uint32_t);
	scrpt_sz_in_words = script_size >> 2;

	buf_data = iaxxx_kvmalloc(fw->size, __GFP_ZERO);
	if (!buf_data)
		return -ENOMEM;

	data = fw->data;

	iaxxx_copy_le32_to_cpu(buf_data, data, fw->size);
	word_data = (uint32_t *)buf_data;

	/* Include file header fields as part of the checksum */
	for (i = 0 ; i < scrpt_sz_in_words; i++)
		CALC_FLETCHER16(word_data[i], sum1, sum2);

	checksum = (sum2 << 16) | sum1;

	dev_dbg(dev, "%s: calculated binary checksum = 0x%.08X\n",
							__func__, checksum);
	if (checksum != word_data[scrpt_sz_in_words]) {
		dev_err(dev,
			"%s:: script bin checksum mismatch. 0x%08X != 0x%08X\n",
			__func__, checksum, word_data[scrpt_sz_in_words]);
		goto out;
	}

	value = IAXXX_SCRIPT_MGMT_SCRIPT_REQ_LOAD_MASK |
		(IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK
		& (script_id
		<< IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS));
	rc = regmap_write(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_REQ_ADDR, value);
	if (rc) {
		dev_err(dev,
			"%s: write to script req (%u) register failed. %d\n",
			__func__, script_id, rc);
		goto out;
	}

	rc = regmap_write(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_SIZE_ADDR, script_size);
	if (rc) {
		dev_err(dev,
			"%s: write to script size (%zu) register failed. %d\n",
			__func__, script_size, rc);
		goto out;
	}

	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		/* Read script error */
		ret = regmap_read(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_ERROR_ADDR, &status);
		if (ret) {
			dev_err(dev, "%s: script error reg read failed. %d\n",
					__func__, ret);
			rc = ret;
			goto out;
		}
		dev_err(dev, "%s: script error reg status 0x%08x. rc %d\n",
							__func__, status, rc);
		goto out;
	}

	rc = regmap_read(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_ADDR_ADDR, &script_addr);
	if (rc) {
		dev_err(dev, "%s: script addr req read failed. %d\n",
						__func__, rc);
		goto out;
	}
	dev_dbg(dev, "%s: script write physical addr:0x%08x\n",
						__func__, script_addr);

	/* Write Package Binary information. Arg "size" should be in words */
	rc = priv->bus_write(dev, script_addr, buf_data, scrpt_sz_in_words);
	if (rc) {
		dev_err(dev,
			"%s: script download at addr: 0x%08x, failed. rc %d\n",
			__func__, script_addr, rc);
		goto out;
	}

	/* Include checksum for this section */
	rc = iaxxx_checksum_request(priv,
			script_addr, scrpt_sz_in_words,
			&finalsum1, &finalsum2, priv->regmap);
	if (rc) {
		dev_err(dev,
			"%s: script Checksum request error: %d\n",
			__func__, rc);
		goto out;
	}

	dev_dbg(dev, "final calculated binary checksum = 0x%04X-0x%04X\n",
		finalsum1, finalsum2);

	if (finalsum1 != sum1 || finalsum2 != sum2) {
		dev_err(dev, "script after download Checksum failed\n");
		dev_err(dev,
			"checksum missmatch 0x%04X-0x%04X != 0x%04X-0x%04X\n",
			sum2, sum1, finalsum2, finalsum1);
		rc = -EINVAL;
		goto out;
	}

	dev_info(dev, "%s: script (%d) download success\n",
							__func__, script_id);
out:
	kvfree(buf_data);
	return rc;
}

/*****************************************************************************
 * iaxxx_core_is_valid_script_id()
 * @brief validate the script id
 *
 * @id  script id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_is_valid_script_id(uint32_t script_id)
{
	bool ret = true;

	if (script_id > (IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK
			>> IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS))
		ret = false;

	return ret;
}
EXPORT_SYMBOL(iaxxx_core_is_valid_script_id);

/*****************************************************************************
 * iaxxx_core_script_id_exist()
 * @brief check if script id exists or not
 *
 * @priv  Pointer to iaxxx private data structure
 * @scrpt_id  script id
 *
 * @ret True if a script id exists (and is loaded) else false
 ****************************************************************************/
struct iaxxx_script_data *iaxxx_core_script_id_exist(struct iaxxx_priv *priv,
							uint32_t scrpt_id)
{
	struct iaxxx_script_data *script_data = NULL;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->script_list_lock);

	if (!list_empty_careful(&priv->iaxxx_state->script_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->script_head_list) {
			script_data = list_entry(node,
					struct iaxxx_script_data,
					script_node);
			if (script_data->script_id == scrpt_id)
				goto exit;
			else
				script_data = NULL;
		}
	}
exit:
	mutex_unlock(&priv->iaxxx_state->script_list_lock);
	return script_data;
}
EXPORT_SYMBOL(iaxxx_core_script_id_exist);

/*****************************************************************************
 * iaxxx_core_script_list_empty()
 * @brief check if script list is empty
 *
 * @priv  Pointer to iaxxx private data structure
 * @ret true if script list is empty false otherwise.
 ****************************************************************************/
bool iaxxx_core_script_list_empty(struct iaxxx_priv *priv)
{
	bool list_empty;

	mutex_lock(&priv->iaxxx_state->script_list_lock);
	list_empty = list_empty_careful(&priv->iaxxx_state->script_head_list);
	mutex_unlock(&priv->iaxxx_state->script_list_lock);

	return list_empty;
}
EXPORT_SYMBOL(iaxxx_core_script_list_empty);

/*****************************************************************************
 * iaxxx_clr_script_list()
 * @brief del, destroy and unload all scripts from list.
 *
 * @priv  Pointer to iaxxx private data structure
 *
 * @ret  void
 ****************************************************************************/
int iaxxx_clr_script_list(struct iaxxx_priv *priv)
{
	struct iaxxx_script_data *script_data;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->script_list_lock);

	list_for_each_safe(node, tmp, &priv->iaxxx_state->script_head_list) {
		script_data = list_entry(node, struct iaxxx_script_data,
							script_node);
		list_del(&script_data->script_node);
		kvfree(script_data);
	}

	mutex_unlock(&priv->iaxxx_state->script_list_lock);

	return 0;
}
EXPORT_SYMBOL(iaxxx_clr_script_list);

/*****************************************************************************
 * iaxxx_add_script_to_list()
 * @brief add scripts to list.
 *
 * @priv  Pointer to iaxxx private data structure
 * @script_data  Pointer to sript data
 *
 * @ret  void
 ****************************************************************************/
static void iaxxx_add_script_to_list(struct iaxxx_priv *priv,
		struct iaxxx_script_data *script_data)
{
	mutex_lock(&priv->iaxxx_state->script_list_lock);
	list_add_tail(&script_data->script_node,
		&priv->iaxxx_state->script_head_list);
	mutex_unlock(&priv->iaxxx_state->script_list_lock);
}

/*****************************************************************************
 * iaxxx_del_script_from_list()
 * @brief delete script from list.
 *
 * @priv  Pointer to iaxxx private data structure
 * @scrpt_id  script id
 *
 * @ret  void
 ****************************************************************************/
static void iaxxx_del_script_from_list(struct iaxxx_priv *priv,
		uint32_t scrpt_id)
{
	struct iaxxx_script_data *script_data;
	struct list_head *node, *tmp;

	/* Search and delete the node with mutex protection
	 * to avoid the case where script could be cleared
	 * simultaneously (ex: by crash recovery)
	 */
	mutex_lock(&priv->iaxxx_state->script_list_lock);
	if (!list_empty_careful(&priv->iaxxx_state->script_head_list)) {
		list_for_each_safe(node, tmp,
		    &priv->iaxxx_state->script_head_list) {
			script_data = list_entry(node,
					struct iaxxx_script_data,
					script_node);
			if (script_data->script_id == scrpt_id) {
				list_del(&script_data->script_node);
				kvfree(script_data);
				goto exit;
			}
		}
	}
exit:
	mutex_unlock(&priv->iaxxx_state->script_list_lock);
}

/*****************************************************************************
 * iaxxx_core_script_load()
 * @brief Load the script
 *
 * @script_name	Script binary name
 * @script_id	Script Id
 *
 * @ret 0 on success, error in case of failure
 ****************************************************************************/
int iaxxx_core_script_load(struct device *dev, const char *script_name,
					uint32_t script_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	const struct firmware *fw = NULL;
	struct iaxxx_script_data *script_data;
	int rc = -EINVAL;

	dev_dbg(dev, "%s:\n", __func__);

	if (!script_name) {
		dev_err(dev, "%s: Err. Script name is NULL\n", __func__);
		return -EINVAL;
	}
	dev_info(dev, "%s: Download Script %s with id %d\n",
					__func__, script_name, script_id);

	mutex_lock(&priv->iaxxx_state->script_lock);

	/* Check if script exist */
	if (iaxxx_core_script_id_exist(priv, script_id)) {
		dev_err(dev, "%s: Script ID %d (0x%x) already exists\n",
				__func__, script_id, script_id);
		rc = -EEXIST;
		goto out;
	}

	rc = request_firmware(&fw, script_name, priv->dev);
	if (rc || !fw) {
		dev_err(dev,
			"%s: request firmware %s image failed. rc = %d\n",
			__func__, script_name, rc);
		goto out;
	}

	rc = iaxxx_download_script(priv, fw, script_id);
	if (rc) {
		dev_err(dev, "%s: script (%d)  download fail. %d\n",
						__func__, script_id, rc);
		goto out;
	}

	/* Insert script node to the list */
	script_data = iaxxx_kvmalloc(sizeof(*script_data), __GFP_ZERO);
	if (!script_data) {
		rc = -ENOMEM;
		goto out;
	}

	script_data->script_id = script_id;

	iaxxx_add_script_to_list(priv, script_data);

	dev_info(dev, "%s: Script ID %d (0x%x) load success\n",
					__func__, script_id, script_id);
out:
	if (fw)
		release_firmware(fw);

	mutex_unlock(&priv->iaxxx_state->script_lock);

	return rc;
}
EXPORT_SYMBOL(iaxxx_core_script_load);

/*****************************************************************************
 * iaxxx_core_script_unload()
 * @brief UnLoad the script
 *
 * @script_id	Script Id
 *
 * @ret 0 on success, error in case of failure
 ****************************************************************************/
int iaxxx_core_script_unload(struct device *dev, uint32_t script_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t value = 0;
	uint32_t status = 0;
	int rc = -EINVAL, ret;

	dev_info(dev, "%s: script (%d) unload\n", __func__, script_id);

	mutex_lock(&priv->iaxxx_state->script_lock);

	/* Check script id is created */
	if (!iaxxx_core_script_id_exist(priv, script_id)) {
		dev_err(dev, "%s: Script ID %d (0x%x) is not created\n",
			__func__, script_id, script_id);
		rc = -ENOENT;
		goto out;
	}

	value = IAXXX_SCRIPT_MGMT_SCRIPT_REQ_UNLOAD_MASK |
		(IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK
		& (script_id
		<< IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS));
	rc = regmap_write(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_REQ_ADDR, value);
	if (rc) {
		dev_err(dev,
			"%s: Write to script (%d) req register failed. %d\n",
			__func__, script_id, rc);
		goto out;
	}

	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		/* Read script error */
		ret = regmap_read(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_ERROR_ADDR, &status);
		if (ret) {
			dev_err(dev, "%s: script error read failed: %d\n",
					__func__, ret);
			rc = ret;
			goto out;
		}
		dev_err(dev, "%s: script error reg status: 0x%08x rc: %d\n",
				__func__, status, rc);
		goto out;
	}

	/* Remove script node from list */
	iaxxx_del_script_from_list(priv, script_id);

	dev_info(dev, "%s: Script ID %d (0x%x) unload success\n",
						__func__, script_id, script_id);
out:
	mutex_unlock(&priv->iaxxx_state->script_lock);
	return rc;
}
EXPORT_SYMBOL(iaxxx_core_script_unload);

/*****************************************************************************
 * iaxxx_core_script_trigger()
 * @brief trigger the script
 *
 * @script_id	Script Id
 *
 * @ret 0 on success, error in case of failure
 ****************************************************************************/
int iaxxx_core_script_trigger(struct device *dev, uint32_t script_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t value = 0;
	uint32_t status = 0;
	int ret = -EINVAL;
	int rc;

	dev_dbg(dev, "%s:\n", __func__);

	mutex_lock(&priv->iaxxx_state->script_lock);

	/* Check script id is created */
	if (!iaxxx_core_script_id_exist(priv, script_id)) {
		dev_warn(dev,
		"%s: Script ID %d (0x%x) not in list. May be static.\n",
			__func__, script_id, script_id);
	}

	value = IAXXX_SCRIPT_MGMT_SCRIPT_REQ_EXECUTE_MASK |
		(IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK
		& (script_id
		<< IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS));

	ret = regmap_write(priv->regmap, IAXXX_SCRIPT_MGMT_SCRIPT_REQ_ADDR,
				 value);
	if (ret) {
		dev_err(dev,
			"%s: Write to script req (%d) register failed: %d\n",
			__func__, script_id, ret);
		goto out;
	}

	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret) {
		/* Read script error */
		rc = regmap_read(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_ERROR_ADDR, &status);
		if (rc) {
			dev_err(dev, "%s: script error read failed: %d\n",
					__func__, rc);
			ret = rc;
			goto out;
		}
		dev_err(dev, "%s: script error reg status: 0x%x ret: %d\n",
				__func__, status, ret);
		goto out;
	}
	dev_info(dev, "%s: Script ID %d (0x%x) trigger success\n",
					__func__, script_id, script_id);
out:
	mutex_unlock(&priv->iaxxx_state->script_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_script_trigger);
