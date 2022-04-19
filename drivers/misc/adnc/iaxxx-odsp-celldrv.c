/*
 * iaxxx-odsp-celldrv.c -- IAxxx OpenDSP cell driver
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

#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-script-mgr.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/module.h>

#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include "iaxxx-odsp-celldrv.h"

static struct odsp_cell_params odsp_cell_priv;



static long odsp_dev_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct odsp_device_priv *odsp_dev_priv = file->private_data;
	struct iaxxx_priv *priv = to_iaxxx_priv(odsp_dev_priv->parent);
	struct iaxxx_plugin_info plg_info;
	struct iaxxx_plugin_param param_info;
	struct iaxxx_plugin_param_blk param_blk_info;
	struct iaxxx_plugin_create_cfg cfg_info;
	struct iaxxx_set_event set_event;
	struct iaxxx_get_event get_event;
	struct iaxxx_evt_info event_info;
	struct iaxxx_pkg_mgmt_info pkg_info;
	struct iaxxx_pkg_type pkg;
	struct iaxxx_set_chan_gain ch_gain_info;
	struct iaxxx_script_info script_info;
	struct iaxxx_evt_trigger et;
	void __user *blk_buff = NULL;
	u16 script_id;
	int ret = -EINVAL;
	u32 mode, speed, fw_status, dev_id;

	pr_debug("cmd 0x%x", cmd);

	if (_IOC_TYPE(cmd) != IAXXX_ODSP_IOCTL_MAGIC)
		return -ENOTTY;

	if (!priv)
		return -EINVAL;

	if (!test_bit(IAXXX_FLG_FW_READY, &priv->flags)) {
		dev_err(priv->dev, "%s: FW is not in App mode\n", __func__);
		return -EINVAL;
	}

	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "iaxxx state NULL\n");
		return -EINVAL;
	}

	switch (cmd) {
	case ODSP_PLG_CREATE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_pkg_id(plg_info.pkg_id) &&
			iaxxx_core_plg_is_valid_priority(plg_info.priority) &&
			iaxxx_core_plg_is_valid_plg_idx(plg_info.plg_idx) &&
			iaxxx_core_plg_is_valid_block_id(plg_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_CREATE");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_create_plg(odsp_dev_priv->parent,
					plg_info.inst_id,
					plg_info.priority, plg_info.pkg_id,
					plg_info.plg_idx, plg_info.block_id,
					plg_info.config_id);
		if (ret) {
			pr_err("dev id-%d: Plugin create failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_DESTROY:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_block_id(plg_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_DESTROY");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_destroy_plg(odsp_dev_priv->parent,
						plg_info.inst_id,
						plg_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Plugin destroy failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_ENABLE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info))) {
			ret = -EFAULT;
			pr_err("dev id-%d: Plugin copy from user failed : %d",
							priv->dev_id, ret);
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_block_id(plg_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_ENABLE");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_change_plg_state(odsp_dev_priv->parent,
				plg_info.inst_id, 1, plg_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Plugin enable failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_DISABLE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_block_id(plg_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_DISABLE");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_change_plg_state(odsp_dev_priv->parent,
				plg_info.inst_id, 0, plg_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Plugin disable failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_RESET:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_block_id(plg_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_RESET");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_reset_plg(odsp_dev_priv->parent,
						plg_info.inst_id,
						plg_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Plugin reset failed : %d",
						priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_SET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
						sizeof(param_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_param_id(param_info.param_id)
			&& iaxxx_core_plg_is_valid_param_val(
						param_info.param_val)
			&& iaxxx_core_plg_is_valid_block_id(
						param_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_SET_PARAM");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_plg_set_param_by_inst(odsp_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				param_info.param_val, param_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Plugin set param failed : %d",
							priv->dev_id, ret);
			pr_err("dev id-%d: Param info inst 0x%x p_id 0x%x p_val 0x%x",
					priv->dev_id,
					param_info.inst_id, param_info.param_id,
					param_info.param_val);
			goto exit;
		}
		break;
	case ODSP_PLG_GET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
						sizeof(param_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_param_id(
						param_info.param_id)
			&& iaxxx_core_plg_is_valid_block_id(
						param_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_GET_PARAM");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_plg_get_param_by_inst(odsp_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				&param_info.param_val, param_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Plugin get param failed : %d",
							priv->dev_id, ret);
			pr_err("dev id-%d: Param info inst 0x%x p_id 0x%x",
				priv->dev_id,
				param_info.inst_id, param_info.param_id);
			goto exit;
		}
		/* After read, copy back the data to user space */
		if (copy_to_user((void __user *)arg, &param_info,
						sizeof(param_info))) {
			ret = -EFAULT;
			goto exit;
		}

		break;
	case ODSP_PLG_SET_PARAM_BLK:
		if (copy_from_user(&param_blk_info, (void __user *)arg,
				sizeof(struct iaxxx_plugin_param_blk))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_block_id(
							param_blk_info.block_id)
			&& iaxxx_core_plg_is_valid_param_blk_size(
						param_blk_info.param_size)
			&& iaxxx_core_plg_is_valid_param_blk_id(
							param_blk_info.id))) {

			dev_err(priv->dev, "invalid plugin parameter received %s\n",
				"ODSP_PLG_SET_PARAM_BLK");
			ret = -EINVAL;
			goto exit;
		}

		param_blk_info.file_name[sizeof(param_blk_info.file_name) - 1]
			 = '\0';

		if (param_blk_info.param_size > 0) {
			blk_buff = (void __user *)
					(uintptr_t)param_blk_info.param_blk;
			blk_buff = memdup_user(blk_buff,
					param_blk_info.param_size);
			param_blk_info.param_blk = (uintptr_t)blk_buff;
			if (IS_ERR(blk_buff)) {
				ret = PTR_ERR(blk_buff);
				pr_err("dev id-%d: memdup failed : %d",
							priv->dev_id, ret);
				goto exit;
			}
			blk_buff = (void *)(uintptr_t)
					(param_blk_info.param_blk);

			ret = iaxxx_core_set_param_blk(odsp_dev_priv->parent,
				param_blk_info.inst_id,
				param_blk_info.param_size,
				blk_buff, param_blk_info.block_id,
				param_blk_info.id);

			kfree(blk_buff);
		} else {
			ret = iaxxx_core_set_param_blk_from_file(
				odsp_dev_priv->parent,
				param_blk_info.inst_id,
				param_blk_info.block_id,
				param_blk_info.id,
				param_blk_info.file_name);
		}

		if (ret) {
			pr_err("dev id-%d: Set param blk failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_SET_CREATE_CFG:
		if (copy_from_user(&cfg_info, (void __user *)arg,
						sizeof(cfg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_cfg_size(cfg_info.cfg_size) &&
			iaxxx_core_plg_is_valid_block_id(cfg_info.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_SET_CREATE_CFG");
			ret = -EINVAL;
			goto exit;
		}

		cfg_info.file_name[sizeof(cfg_info.file_name) - 1] = '\0';
		ret = iaxxx_core_set_create_cfg(odsp_dev_priv->parent,
				cfg_info.inst_id, cfg_info.cfg_size,
				cfg_info.cfg_val, cfg_info.block_id,
				cfg_info.file_name);
		if (ret) {
			pr_err("dev id-%d: Plugin create cfg failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_PLG_SET_EVENT:
		if (copy_from_user(&set_event, (void __user *)arg,
						sizeof(set_event))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_block_id(set_event.block_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_PLG_SET_EVENT");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_set_event(odsp_dev_priv->parent,
					set_event.inst_id,
					set_event.event_enable_mask,
					set_event.block_id);
		if (ret) {
			pr_err("dev id-%d: Set event failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_EVENT_SUBSCRIBE:
		if (copy_from_user(&event_info, (void __user *)arg,
						sizeof(event_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_evt_is_valid_src_id(event_info.src_id)
			&& iaxxx_core_evt_is_valid_dst_id(event_info.dst_id)
			&& iaxxx_core_evt_is_valid_event_id(
						event_info.event_id)
			&& iaxxx_core_evt_is_valid_dst_opaque(
						event_info.dst_opaque))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_EVENT_SUBSCRIBE");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_evt_subscribe(odsp_dev_priv->parent,
					event_info.src_id,
					event_info.event_id,
					event_info.dst_id,
					event_info.dst_opaque);
		if (ret) {
			pr_err("dev id-%d: Event subscribe failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_GET_EVENT:
		if (copy_from_user(&get_event, (void __user *)arg,
						sizeof(get_event))) {
			ret = -EFAULT;
			goto exit;
		}
		ret = iaxxx_core_retrieve_event(odsp_dev_priv->parent,
				&get_event.event_id,
				&get_event.data,
				&get_event.event_src);
		if (ret) {
			pr_err("dev id-%d: Get event failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		/*After read, copy back the data to user space */
		if (copy_to_user((void __user *)arg, &get_event,
						sizeof(get_event))) {
			ret = -EFAULT;
			goto exit;
		}
		break;
	case ODSP_EVENT_UNSUBSCRIBE:
		if (copy_from_user(&event_info, (void __user *)arg,
						sizeof(event_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!(iaxxx_core_evt_is_valid_src_id(event_info.src_id)
			&& iaxxx_core_evt_is_valid_dst_id(event_info.dst_id)
			&& iaxxx_core_evt_is_valid_event_id(
						event_info.event_id))) {

			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_EVENT_UNSUBSCRIBE");
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_evt_unsubscribe(odsp_dev_priv->parent,
					event_info.src_id,
					event_info.event_id,
					event_info.dst_id);
		if (ret) {
			pr_err("dev id-%d: Event unsubscribe failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	case ODSP_LOAD_PACKAGE:
		if (copy_from_user(&pkg_info, (void __user *)arg,
						sizeof(pkg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!iaxxx_core_plg_is_valid_pkg_id(pkg_info.pkg_id)) {
			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_LOAD_PACKAGE");
			ret = -EINVAL;
			goto exit;
		}

		pr_info("dev id-%d: Pkg name %s, pkg id %d",
				priv->dev_id,
				pkg_info.pkg_name, pkg_info.pkg_id);
		ret = iaxxx_package_load(odsp_dev_priv->parent,
			pkg_info.pkg_name, pkg_info.pkg_id,
			&pkg_info.proc_id);
		if (ret) {
			pr_err("dev id-%d: Load package failed : %d",
							priv->dev_id, ret);
			if (ret == -EEXIST) {
				if (copy_to_user((void __user *)arg, &pkg_info,
							sizeof(pkg_info))) {
					ret = -EFAULT;
					goto exit;
				}
			}
			goto exit;
		}
		/*After read, copy back the data to user space */
		if (copy_to_user((void __user *)arg, &pkg_info,
						sizeof(pkg_info))) {
			ret = -EFAULT;
			goto exit;
		}
		break;

	case ODSP_UNLOAD_PACKAGE:
		if (copy_from_user(&pkg_info, (void __user *)arg,
						sizeof(pkg_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the plugin parameters */
		if (!iaxxx_core_plg_is_valid_pkg_id(pkg_info.pkg_id)) {
			dev_err(priv->dev, "invalid plugin parameter received for %s\n",
				"ODSP_UNLOAD_PACKAGE");
			ret = -EINVAL;
			goto exit;
		}

		pr_info("dev id-%d: Pkg name : %s, id : %d",
					priv->dev_id,
					pkg_info.pkg_name, pkg_info.pkg_id);
		ret = iaxxx_package_unload(odsp_dev_priv->parent,
			pkg_info.pkg_id);
		if (ret) {
			pr_err("dev id-%d: Unload package failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;

	case ODSP_GET_PACKAGE_TYPE:
		if (copy_from_user(&pkg, (void __user *)arg, sizeof(pkg))) {
			ret = -EFAULT;
			goto exit;
		}

		ret = iaxxx_get_package_type(odsp_dev_priv->parent, pkg.pkg_id,
					     &pkg.pkg_type);
		if (ret) {
			pr_err("dev id-%d: Get package type failed : %d",
							priv->dev_id, ret);
			goto exit;
		}

		if (copy_to_user((void __user *)arg, &pkg, sizeof(pkg))) {
			pr_err("dev id-%d: copy_to_user package type failed",
								priv->dev_id);
			ret = -EFAULT;
			goto exit;
		}
		break;
	case ODSP_GET_FW_STATE:
		dev_info(priv->dev, "%s: get fw_state id\n", __func__);
		ret = regmap_read(priv->regmap,
					IAXXX_SRB_SYS_STATUS_ADDR, &mode);
		if (ret) {
			dev_err(priv->dev,
			"read SYSTEM_STATUS failed : %d\n", ret);
			goto exit;
		}

		dev_info(priv->dev, "%s: fw_state : 0x%x\n", __func__, mode);

		mode = mode & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		if (copy_to_user((void __user *)arg,
				 &mode, sizeof(u32))) {
			pr_err("dev id-%d: copy_to_user fw_state failed",
							priv->dev_id);
			ret = -EFAULT;
			goto exit;
		}
		break;
	case ODSP_SET_CHAN_GAIN:
		if (copy_from_user(&ch_gain_info, (void __user *)arg,
						sizeof(ch_gain_info))) {
			ret = -EFAULT;
			goto exit;
		}
		ret = iaxxx_set_channel_gain(odsp_dev_priv->parent,
					ch_gain_info.id,
					ch_gain_info.target_gain,
					ch_gain_info.gain_ramp,
					ch_gain_info.block_id);
		if (ret) {
			pr_err("dev id-%d: Set channel gain failed : %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;

	case ODSP_SCRIPT_TRIGGER:
		if (copy_from_user(&script_id, (void __user *)arg,
				   sizeof(script_id))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the script parameters */
		if (!iaxxx_core_is_valid_script_id(script_id)) {
			pr_err("dev id-%d: invalid script parameter received %d",
						priv->dev_id, script_id);
			ret = -EINVAL;
			goto exit;
		}
		ret = iaxxx_core_script_trigger(
				odsp_dev_priv->parent, script_id);
		if (ret) {
			pr_err("dev id-%d: Script trigger fail %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;

	case ODSP_SCRIPT_LOAD:
		if (copy_from_user(&script_info, (void __user *)arg,
					sizeof(script_info))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the script load parameters */
		if (!iaxxx_core_is_valid_script_id(script_info.script_id)) {
			pr_err("dev id-%d: invalid script parameter received %d",
						priv->dev_id, script_id);
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_script_load(odsp_dev_priv->parent,
				script_info.script_name, script_info.script_id);
		if (ret) {
			pr_err("dev id-%d: Script (%d) load fail. ret %d",
				priv->dev_id, script_info.script_id, ret);
			goto exit;
		}
		break;

	case ODSP_SCRIPT_UNLOAD:
		if (copy_from_user(&script_id, (void __user *)arg,
					sizeof(uint16_t))) {
			ret = -EFAULT;
			goto exit;
		}

		/* validate the script parameters */
		if (!iaxxx_core_is_valid_script_id(script_id))  {
			pr_err("dev id-%d: invalid script parameter received %d",
						priv->dev_id, script_id);
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_core_script_unload(
				odsp_dev_priv->parent, script_id);
		if (ret) {
			pr_err("dev id-%d: Script (%d) unload fail. ret %d",
				priv->dev_id, script_info.script_id, ret);
			goto exit;
		}
		break;

	case ODSP_EVENT_TRIGGER: {
		if (copy_from_user(&et, (void __user *)arg, sizeof(et))) {
			ret = -EFAULT;
			goto exit;
		}

		ret = iaxxx_core_evt_trigger(odsp_dev_priv->parent,
				et.src_id, et.evt_id, et.src_opaque);
		if (ret) {
			pr_err("dev id-%d: Event trigger fail %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	}

	case ODSP_UPDATE_INTF_SPEED: {
		if (copy_from_user(&speed, (void __user *)arg, sizeof(speed))) {
			ret = -EFAULT;
			goto exit;
		}

		if (!speed) {
			pr_err("dev id-%d: 0Hz speed requested", priv->dev_id);
			ret = -EINVAL;
			goto exit;
		}

		ret = iaxxx_update_intf_speed(odsp_dev_priv->parent, speed);
		if (ret) {
			pr_err("dev id-%d: interface speed update fail %d",
							priv->dev_id, ret);
			goto exit;
		}
		break;
	}

	case ODSP_DISABLE_OPTIMAL_TO_NORMAL_PWR_TRANSITION: {
		if (copy_from_user(&priv->disable_optimal_to_normal,
			(void __user *)arg,
			sizeof(priv->disable_optimal_to_normal))) {
			ret = -EFAULT;
			goto exit;
		}

		if (!priv->disable_optimal_to_normal) {
			ret =
				iaxxx_enable_optimal_to_normal_transition(priv);
			if (ret)
				pr_err(
				"dev id-%d: Enable optimal to normal transition fail %d",
				priv->dev_id, ret);
		} else {
			ret = 0;
		}

		pr_info("dev id-%d: Optimal to Normal mode transition %s",
			priv->dev_id,
			priv->disable_optimal_to_normal ? "Disabled" :
			"Enabled");

		break;
	}

	case ODSP_GET_CHIP_STATUS: {
		if (!iaxxx_core_pkg_plg_list_empty(priv) ||
			 iaxxx_core_get_route_status(priv))
			fw_status = IAXXX_CHIP_ACTIVE;
		else
			fw_status = IAXXX_CHIP_IDLE;

		if (copy_to_user((void __user *)arg, &fw_status,
				 sizeof(fw_status))) {
			ret = -EFAULT;
			goto exit;
		}

		ret = 0;

		break;
	}

	case ODSP_SET_ROUTE_STATUS: {
		u32 route_status = 0;

		if (copy_from_user(&route_status,
			(void __user *)arg,
			sizeof(route_status))) {
			ret = -EFAULT;
			goto exit;
		}
		iaxxx_core_set_route_status(priv,
					route_status ? true : false);
		ret = 0;
		break;
	}

	case ODSP_GET_DEV_ID:
		dev_id = priv->dev_id;

		if (copy_to_user((void __user *)arg, &dev_id,
				 sizeof(dev_id))) {
			ret = -EFAULT;
			goto exit;
		}

		ret = 0;
		break;

	case ODSP_RESET_FW: {
		ret = iaxxx_fw_reset(priv);
		if (ret) {
			pr_err("dev id-%d: FW reset fail, %d",
							priv->dev_id, ret);
			goto exit;
		}

		break;
	}
	default:
		break;
	}

exit:
	return ret;
}

#ifdef CONFIG_COMPAT
static long odsp_dev_compat_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	return odsp_dev_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int odsp_dev_open(struct inode *inode, struct file *file)
{
	struct odsp_device_priv *odsp_dev_priv = container_of(inode->i_cdev,
						struct odsp_device_priv, cdev);
	file->private_data = odsp_dev_priv;

	return 0;
}

static int odsp_dev_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations odsp_dev_fops = {
	.open = odsp_dev_open,
	.release = odsp_dev_release,
	.unlocked_ioctl	= odsp_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= odsp_dev_compat_ioctl,
#endif
};


static int iaxxx_odsp_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct odsp_device_priv *odsp_dev_priv;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	int ret;

	dev_info(dev, "%s: enter, dev->id %d, pdev->id %d\n", __func__,
							dev->id, pdev->id);

	if (odsp_cell_priv.cdev_minor == ODSP_CDEV_MAX_DEVICES) {
		dev_err(dev, "Minor nos exhausted. Increase CVQ_CDEV_MAX_DEVICES\n");
		return -ENOBUFS;
	}

	odsp_dev_priv = iaxxx_kvmalloc(sizeof(*odsp_dev_priv), __GFP_ZERO);
	if (!odsp_dev_priv) {
		ret = -ENOMEM;
		goto out_err;
	}

	odsp_dev_priv->regmap = priv->regmap;
	odsp_dev_priv->parent = dev->parent;
	odsp_dev_priv->cdev.owner = THIS_MODULE;
	odsp_dev_priv->dev_num =
		MKDEV(MAJOR(odsp_cell_priv.dev_num), odsp_cell_priv.cdev_minor);
	cdev_init(&odsp_dev_priv->cdev, &odsp_dev_fops);
	ret = cdev_add(&odsp_dev_priv->cdev, odsp_dev_priv->dev_num, 1);
	if (ret) {
		dev_err(dev, "add cdev error failed : %d\n", ret);
		goto free_odsp;
	}

	odsp_dev_priv->dev = device_create(odsp_cell_priv.cdev_class, dev,
		odsp_dev_priv->dev_num, odsp_dev_priv, "%s%d",
		"iaxxx-odsp-celldrv", priv->dev_id);
	if (IS_ERR(odsp_dev_priv->dev)) {
		ret = PTR_ERR(odsp_dev_priv->dev);
		dev_err(dev, "create cdev device failed : %d\n", ret);
		goto err_device_create;
	}

	odsp_cell_priv.cdev_minor++;
	dev_set_drvdata(dev, odsp_dev_priv);
	dev_info(dev, "ODSP device cdev initialized.\n");

	odsp_dev_priv->dev = dev;

	return 0;

err_device_create:
	cdev_del(&odsp_dev_priv->cdev);
free_odsp:
	kvfree(odsp_dev_priv);
out_err:
	dev_err(dev, "cdev setup failed : %d. no cdevs available!\n", ret);

	return ret;

}
static int iaxxx_odsp_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct odsp_device_priv *odsp_dev_priv = dev_get_drvdata(dev);

	odsp_cell_priv.cdev_minor--;
	device_destroy(odsp_cell_priv.cdev_class, odsp_dev_priv->dev_num);
	cdev_del(&odsp_dev_priv->cdev);

	kvfree(odsp_dev_priv);

	return 0;
}

static int iaxxx_odsp_rt_suspend(struct device *dev)
{
	return 0;
}

static int iaxxx_odsp_rt_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops iaxxx_odsp_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_odsp_rt_suspend,
		iaxxx_odsp_rt_resume, NULL)

};

static const struct of_device_id iaxxx_odsp_dt_match[] = {
	{.compatible = "knowles,iaxxx-odsp-celldrv"},
	{}
};

static struct platform_driver iaxxx_odsp_driver = {
	.probe  = iaxxx_odsp_dev_probe,
	.remove = iaxxx_odsp_dev_remove,
	.driver = {
		.name = "iaxxx-odsp-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_odsp_dt_match,
		.pm = &iaxxx_odsp_pm_ops,
	},
};

static int __init iaxxx_odsp_init(void)
{
	int ret;

	odsp_cell_priv.cdev_minor = 0;
	odsp_cell_priv.cdev_class = class_create(THIS_MODULE,
						"iaxxx-odsp-celldrv");
	if (IS_ERR(odsp_cell_priv.cdev_class)) {
		ret = PTR_ERR(odsp_cell_priv.cdev_class);
		pr_err("creating iaxxx-odsp-celldrv class failed : %d", ret);
		goto out_err;
	}

	ret = alloc_chrdev_region(&odsp_cell_priv.dev_num, 0,
				ODSP_CDEV_MAX_DEVICES, "iaxxx-odsp-celldrv");
	if (ret) {
		pr_err("alloc chardev region failed : %d", ret);
		goto err_alloc_cdev;
	}

	pr_info("Allocated Major device number %d",
						MAJOR(odsp_cell_priv.dev_num));

	ret = platform_driver_register(&iaxxx_odsp_driver);
	if (ret)
		goto err_plat_register;

	return 0;

err_plat_register:
	unregister_chrdev_region(odsp_cell_priv.dev_num, ODSP_CDEV_MAX_DEVICES);
err_alloc_cdev:
	class_destroy(odsp_cell_priv.cdev_class);
out_err:
	pr_err("ODSP Cell init failed : %d. No CVQ cdevs available!", ret);
	return ret;
}


static void __exit iaxxx_odsp_exit(void)
{
	platform_driver_unregister(&iaxxx_odsp_driver);

	unregister_chrdev_region(odsp_cell_priv.dev_num,
						ODSP_CDEV_MAX_DEVICES);
	class_destroy(odsp_cell_priv.cdev_class);
}
module_init(iaxxx_odsp_init);
module_exit(iaxxx_odsp_exit);

MODULE_DESCRIPTION("ODSP functions support for Knowles IAxxx");
MODULE_LICENSE("GPL");
