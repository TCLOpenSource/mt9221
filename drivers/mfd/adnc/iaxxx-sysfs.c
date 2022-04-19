/*
 * iaxxx-sysfs.c -- IAxxx Sysfs attributes
 *
 * Copyright 2018 Knowles Corporation
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
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include "iaxxx.h"
#include "iaxxx-build-info.h"

#define IAXXX_FW_DOWNLOAD_TIMEOUT	10000		/* 10 secs */
#define IAXXX_VER_STR_SIZE		60

/**
 * Sysfs save isr_disable option
 */
static ssize_t iaxxx_isr_disable_save(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		dev_info(dev, "%s: ISR Disabled\n", __func__);
		priv->debug_isr_disable = true;
	} else {
		priv->debug_isr_disable = false;
		dev_info(dev, "%s: ISR Enabled\n", __func__);
	}
	return count;
}

/**
 * sysfs show isr disable option
 */
static ssize_t iaxxx_isr_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	return scnprintf(buf, PAGE_SIZE, "ISR Event Handling %s\n",
		priv->debug_isr_disable ? "Disabled" : "Enabled");
}

static DEVICE_ATTR(isr_disable, 0600,
		iaxxx_isr_disable_show, iaxxx_isr_disable_save);

/**
 * iaxxx_firmware_version_show - sys node show function for firmware version
 */
static ssize_t iaxxx_firmware_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	char appbuf[IAXXX_VER_STR_SIZE], rombuf[IAXXX_VER_STR_SIZE];
	uint32_t rom_num = 0, app_num = 0;
	int32_t len = IAXXX_VER_STR_SIZE;
	int rc;

	if (!priv || !priv->iaxxx_state) {
		dev_err(dev, "%s: Invalid Parameter\n", __func__);
		return -EINVAL;
	}

	rombuf[0] = '\0';
	appbuf[0] = '\0';

	/* Rome version is available when chip is in SBL/APP mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR,
				       &rom_num);
	if (rc) {
		dev_err(dev, "regmap_read failed : %d\n", rc);
		return rc;
	}

	rc = get_version_str(priv, IAXXX_SRB_SYS_ROM_VER_STR_ADDR,
			     rombuf, len);
	if (rc)
		dev_err(dev, "%s: FW Rome version read failed : %d\n",
			__func__, rc);

	/* App version is available when chip is in APP mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_APP_VER_NUM_ADDR,
			       &app_num);
	if (rc)
		dev_err(dev, "regmap_read failed : %d\n", rc);

	rc = get_version_str(priv, IAXXX_SRB_SYS_APP_VER_STR_ADDR,
			     appbuf, len);
	if (rc)
		dev_err(dev, "%s: FW APP version read failed : %d\n",
				__func__, rc);

	return scnprintf(buf, PAGE_SIZE,
			 "ROM_version = %s:%x\nAPP_version = %s:%x\n",
			 rombuf[0] ? rombuf : "NULL", rom_num,
			 appbuf[0] ? appbuf : "NULL", app_num);
}
static DEVICE_ATTR(fw_version, 0400, iaxxx_firmware_version_show, NULL);

/**
 * iaxxx_host_version_show - sys node show function hsw veriso
 * and fw version built with
 */
static ssize_t iaxxx_host_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"HSW ver %s built with Firmware ver %s\n",
			HOST_SOFTWARE_VERSION_STR, FW_VERSION_IN_HOST_STR);
}
static DEVICE_ATTR(host_version, 0400, iaxxx_host_version_show, NULL);

/*
 * Sysfs - firmware update
 */
static ssize_t iaxxx_fw_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	/* Kick work queue for firmware loading */
	iaxxx_work(priv, fw_update_work);
	return count;
}
static DEVICE_ATTR(fw_update, 0200, NULL, iaxxx_fw_update);

/**
 * iaxxx_plugin_version_show - sys node show function for plugin version
 */
static ssize_t iaxxx_plugin_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	uint32_t buf_len = 0;
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	if (!priv) {
		dev_err(dev, "%s: Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
							plugin_node);
		rc = iaxxx_core_plg_get_plugin_version(dev,
					plugin_data->inst_id, verbuf, len);
		if (rc) {
			dev_err(dev, "%s: Plugin version read failed : %d\n",
								__func__, rc);
			buf_len = 0;
			goto exit;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE,
			"plugin-%d:proc-id-%d\t%s\n", plugin_data->inst_id,
						plugin_data->proc_id, verbuf);
	}

exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return buf_len;
}
static DEVICE_ATTR(plugin_version, 0400, iaxxx_plugin_version_show, NULL);

/**
 * iaxxx_package_version_show - sys node show function for package version
 */
static ssize_t iaxxx_package_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	uint32_t buf_len = 0;
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	if (!priv) {
		dev_err(dev, "%s: Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
								plugin_node);
		rc = iaxxx_core_plg_get_package_version(dev,
					plugin_data->inst_id, verbuf, len);
		if (rc) {
			dev_err(dev, "%s: Package version read failed : %d\n",
								__func__, rc);
			buf_len = 0;
			goto exit;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE,
			"package-%d:\t%s\n", plugin_data->inst_id, verbuf);
	}
exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return buf_len;
}
static DEVICE_ATTR(package_version, 0400, iaxxx_package_version_show, NULL);

/**
 * iaxxx_firmware_timestamp_show - sys node show function for firmware timestamp
 */
static ssize_t iaxxx_firmware_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t fw_clk_rd[2];
	uint64_t timestamp;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "Error, invalid device\n");

	/* Read Firmware wall clock timestamp */
	rc = regmap_bulk_read(priv->regmap,
			IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR,
			fw_clk_rd, ARRAY_SIZE(fw_clk_rd));

	if (rc) {
		dev_err(dev,
			"IAXXX_AF_WCPT_WALL_CLOCK_RD failed : %d\n", rc);
		return rc;
	}
	timestamp = (((long)((fw_clk_rd[1] & 0xFFFF)) << 32) | fw_clk_rd[0]);

	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", timestamp);
}
static DEVICE_ATTR(fw_timestamp, 0400, iaxxx_firmware_timestamp_show, NULL);

/**
 * iaxxx_firmware_update_test_show - sys node show function for firmware test
 *
 * Trigger firmware update test work and return results
 *
 */
static ssize_t iaxxx_firmware_update_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	long rc;
	ssize_t count;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	mutex_lock(&priv->test_mutex);

	cancel_work_sync(&priv->dev_fw_update_test_work);
	init_completion(&priv->bootup_done);
	schedule_work(&priv->dev_fw_update_test_work);

	rc = wait_for_completion_interruptible_timeout(&priv->bootup_done,
			msecs_to_jiffies(IAXXX_FW_DOWNLOAD_TIMEOUT));

	if (rc > 0 && priv->test_result)
		count = scnprintf(buf, PAGE_SIZE, "SUCCESS\n");
	else
		count = scnprintf(buf, PAGE_SIZE, "FAIL\n");

	mutex_unlock(&priv->test_mutex);

	return count;
}
static DEVICE_ATTR(fw_update_test, 0400, iaxxx_firmware_update_test_show,
			NULL);

/**
 * iaxxx_cmem_test_show - sys node show function for memory test
 *
 * Trigger firmware update test work and return results
 *
 */
static ssize_t iaxxx_cmem_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	long rc;
	ssize_t count;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	mutex_lock(&priv->test_mutex);

	cancel_work_sync(&priv->dev_cmem_test_work);
	init_completion(&priv->cmem_done);
	schedule_work(&priv->dev_cmem_test_work);

	rc = wait_for_completion_interruptible_timeout(&priv->cmem_done,
			msecs_to_jiffies(IAXXX_FW_DOWNLOAD_TIMEOUT));
	if (rc > 0)
		count = scnprintf(buf, PAGE_SIZE, "SUCCESS\n");
	else
		count = scnprintf(buf, PAGE_SIZE, "FAIL\n");

	mutex_unlock(&priv->test_mutex);

	return count;
}
static DEVICE_ATTR(cmem_test, 0400, iaxxx_cmem_test_show, NULL);

/*
 * iaxxx_pm_wakeup_chip
 */
static ssize_t iaxxx_sysfs_pm_wakeup_chip(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, reg_val, rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	/* SPI transaction should wake up the chip
	 * reading SYS_STATUS reg
	 */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);
	msleep(50);
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);

	if (rc)
		dev_err(dev, "%s: failed : %d\n", __func__, rc);
	else
		dev_info(dev, "%s: Success\n", __func__);
	return count;
}
static DEVICE_ATTR(pm_wakeup_chip, 0200, NULL, iaxxx_sysfs_pm_wakeup_chip);

/* sysfs chip_reset function
 */
static ssize_t iaxxx_chip_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!priv)
		return -EINVAL;
	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	iaxxx_reset_to_sbl(priv);

	return count;
}
static DEVICE_ATTR(chip_reset, 0200, NULL, iaxxx_chip_reset_store);

/* sysfs disable runtime pm
 */
static ssize_t iaxxx_runtime_pm_disable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		/* Wake up and disable runtime pm */
		if (!priv->debug_runtime_pm_disable) {

			/* wake up chip */
			iaxxx_wakeup_chip(priv);
			pm_runtime_forbid(dev);
			dev_info(dev, "%s: Runtime PM Disabled\n", __func__);
			priv->debug_runtime_pm_disable = true;
		} else
			dev_info(dev, "%s: Runtime PM Already Disabled\n",
					__func__);

	} else {
		if (priv->debug_runtime_pm_disable) {
			pm_runtime_allow(dev);
			dev_info(dev, "%s: Runtime PM Enabled\n", __func__);
			priv->debug_runtime_pm_disable = false;
		} else
			dev_info(dev, "%s: Runtime PM Already Enabled\n",
					__func__);
	}

	return count;
}
static DEVICE_ATTR(runtime_pm_disable, 0200, NULL,
		iaxxx_runtime_pm_disable);

/* sysfs disable fw crash handling
 */
static ssize_t iaxxx_fw_crash_handling_disable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		priv->debug_fw_crash_handling_disable = true;
		dev_info(dev, "%s: FW Crash Handling Disabled\n", __func__);

	} else {
		priv->debug_fw_crash_handling_disable = false;
		dev_info(dev, "%s: FW Crash Handling Enabled\n", __func__);
	}

	return count;
}
static DEVICE_ATTR(fwcrash_handling_disable, 0200, NULL,
		iaxxx_fw_crash_handling_disable);

/* sysfs debug simulate fw_crash
 */
static ssize_t iaxxx_simulate_fw_crash(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	rc = iaxxx_pm_get_sync(priv->dev);
	if (rc < 0)
		return rc;

	iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT);

	iaxxx_pm_put_autosuspend(priv->dev);

	return count;
}
static DEVICE_ATTR(simulate_fw_crash, 0200, NULL,
		iaxxx_simulate_fw_crash);

static ssize_t iaxxx_get_current_power_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	uint32_t power_state;

	if (!priv || !priv->iaxxx_state)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	power_state = atomic_read(&priv->iaxxx_state->power_state);

	switch (power_state) {
	case IAXXX_OPTIMAL_MODE:
		return scnprintf(buf, PAGE_SIZE, "OPTIMAL\n");
	case IAXXX_SLEEP_MODE:
		return scnprintf(buf, PAGE_SIZE, "SLEEP\n");
	case IAXXX_NORMAL_MODE:
		return scnprintf(buf, PAGE_SIZE, "NORMAL\n");
	default:
		return scnprintf(buf, PAGE_SIZE, "INVALID %d\n", power_state);
	}
}
static DEVICE_ATTR(get_current_power_mode, 0400,
		   iaxxx_get_current_power_mode_show, NULL);

static ssize_t iaxxx_tunnel_threshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &priv->tunnel_threshold))
		return -EINVAL;

	return count;
}

static ssize_t iaxxx_tunnel_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	return scnprintf(buf, PAGE_SIZE,
			 "Tunnel threshold = 0x%x\n",
			 priv->tunnel_threshold);
}
static DEVICE_ATTR(tnl_threshold, 0600, iaxxx_tunnel_threshold_show,
		   iaxxx_tunnel_threshold_store);

/**
 * Sysfs show/check & save interface max speed
 */
static ssize_t iaxxx_intf_max_speed_save(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	u32 val;
	int rc;

	if (!buf || !priv)
		return -EINVAL;

	rc = kstrtoint(buf, 0, &val);
	if (rc) {
		dev_err(priv->dev,
		"%s: Invalid format. Expected hex or decimal integer. rc %d\n",
								__func__, rc);
		return rc;
	}

	if (val <= priv->intf_max_speed) {
		dev_err(priv->dev, "%s: invalid val: %d. val should be > %d\n",
					__func__, val, priv->intf_max_speed);
		return -EINVAL;
	}

	priv->update_intf_max_speed  = val;

	return count;
}

static ssize_t iaxxx_intf_max_speed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf || !priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "update Interface Max Speed : %d\n",
		priv->update_intf_max_speed);
}
static DEVICE_ATTR(intf_max_speed, 0600,
		   iaxxx_intf_max_speed_show, iaxxx_intf_max_speed_save);

static ssize_t iaxxx_kw_detect_count_clear(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int i;

	if (!buf || !priv)
		return -EINVAL;

	pr_info("dev id-%d: Kw detect count clear request", priv->dev_id);
	for (i = 0; i < IAXXX_MAX_KW_EVENTS; i++) {
		pr_info("dev id-%d: Event id %d, count %d",
				priv->dev_id, i, priv->kw_detect_count[i]);
		priv->kw_detect_count[i] = 0;
	}

	return count;
}

static ssize_t iaxxx_kw_detect_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	return scnprintf(buf, PAGE_SIZE,
			 "event 0: %d, event 1: %d, event2: %d, event3: %d, event4: %d\n",
			 priv->kw_detect_count[0], priv->kw_detect_count[1],
			 priv->kw_detect_count[2], priv->kw_detect_count[3],
			 priv->kw_detect_count[4]);
}
static DEVICE_ATTR(kw_detect_count, 0600, iaxxx_kw_detect_count_show,
		   iaxxx_kw_detect_count_clear);

/*
 * sysfs memsweep test
 */

static const char *mem_sweep_test_bin = "audience/ia8x01/Mem_sweep_test.bin";
static int iaxxx_mem_sweep_test_cmds(struct device *dev)
{
	struct iaxxx_priv *iaxxx = dev ? to_iaxxx_priv(dev) : NULL;
	int retry = IAXXX_SYNC_RETRY;
	int rc = 0, ret = 0;
	uint32_t response;
	int i;

	if (!iaxxx)
		return -EINVAL;

	if (!iaxxx->raw_ops->cmd)
		return -EINVAL;

	do {
		retry--;

		/* Send a SYNC command */
		rc = iaxxx->raw_ops->cmd(iaxxx, SBL_SYNC_CMD, &response);
		if (rc || response != SBL_SYNC_CMD) {
			dev_err(dev,
				"%s: SYNC cmd failed : %d, resp:0x%.08X. retry\n",
				__func__, rc, response);

			/* if rc is 0, update it with proper error value */
			if (rc == 0)
				rc = -EINVAL;
			continue;
		}

		dev_dbg(dev, "SYNC response: 0x%.08X\n", response);
	} while (rc && retry);

	if (rc)
		goto out;

	ret = 0;
	for (i = 0; i < IAXXX_MEMSWEEP_CMD_COUNT; i++) {
		msleep(500);
		if (!mem_test_cmds_val[i][1]) {
			rc = iaxxx->raw_ops->cmd(iaxxx,
				mem_test_cmds_val[i][0], NULL);
			response = 0;
		} else {
			rc = iaxxx->raw_ops->cmd(iaxxx,
				mem_test_cmds_val[i][0], &response);
		}

		if (rc || response != mem_test_cmds_val[i][1]) {
			dev_err(dev,
				"%s: Improper response for cmd: 0x%x, resp:0x%x, rc = %d\n",
				__func__, mem_test_cmds_val[i][0],
				response, rc);

			ret |= -EINVAL;
			continue;
		}

		dev_info(dev,
			"%s: cmd: 0x%x, resp:0x%x\n", __func__,
			mem_test_cmds_val[i][0], response);
	}

	if (!ret && !rc)
		dev_info(dev,
			"%s: memsweep test is successful\n", __func__);

out:
	return ret ? ret : rc;
}

static int iaxxx_mem_sweep_test_bin_download(struct iaxxx_priv *priv)
{
	const struct firmware *fw;
	struct device *dev = priv->dev;
	const struct firmware_file_header *fw_header;
	u32 status;
	int rc;

	/* Request the firmware image */
	rc = request_firmware(&fw, mem_sweep_test_bin, dev);
	if (rc) {
		dev_err(dev, "Firmware file %s not found rc = %d\n",
						mem_sweep_test_bin, rc);
		return rc;
	}

	/* Call SPI speed setup callback if exists */
	if (priv->intf_speed_setup)
		priv->intf_speed_setup(dev, IAXXX_SPI_SPEED_FOR_MEM_SWEEP_TEST);

	/* Download the firmware to device memory */
	rc = iaxxx_download_firmware(priv, fw);
	if (rc) {
		dev_err(dev, "Memory Sweep binary download failed, rc = %d\n",
			rc);
		goto out;
	} else {
		dev_info(dev, "Memory Sweep binary download success\n");
	}

	fw_header = (const struct firmware_file_header *)fw->data;

	usleep_range(1000, 1010);

	dev_info(dev, "Requesting Jump to App Mode (Addr : 0x%08x)\n",
						fw_header->entry_point);

	/* Program BOOTLOADER_JUMP_ADDRESS with the start address */
	rc = regmap_write(priv->regmap_no_pm, IAXXX_SRB_BOOT_JUMP_ADDR_ADDR,
				le32_to_cpu(fw_header->entry_point));
	if (rc) {
		dev_err(dev, "Failed to set JUMP_ADDRESS, rc = %d\n", rc);
		goto out;
	}

	/* Set the request bit in BOOTLOADER_REQUEST register */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_BOOT_REQ_ADDR,
				IAXXX_SRB_BOOT_REQ_JUMP_REQ_MASK,
				0x1 << IAXXX_SRB_BOOT_REQ_JUMP_REQ_POS);
	if (rc) {
		dev_err(dev, "Failed to set BOOTLOADER_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		dev_err(dev, "BOOTLOADER_REQUEST failed, rc = %d\n", rc);
		goto out;
	}

	msleep(50);

	/*
	 * Rather than checking for the update block bit to be cleared, added
	 * check for SYNC command response. If its 0x80000000, that means chip
	 * moved to legacy mode after memory sweep binary download. Can't wait
	 * for bit clear because as soon as bit will be cleared it will move to
	 * legacy mode and reading the register after that can lead the response
	 * to move in next word.
	 */
	rc = iaxxx_mem_sweep_test_cmds(dev);
	if (rc)
		dev_err(dev, "Mem Sweep commands response failed, rc = %d\n",
			rc);

out:
	if (fw)
		release_firmware(fw);
	return rc;
}

static ssize_t iaxxx_memsweep_test_start(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	int rc;
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf)
		return -EINVAL;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		/* Bypass regmap cache */
		regcache_cache_bypass(priv->regmap, true);

		if (priv->reset_cb) {
			rc = priv->reset_cb(dev);
			if (rc) {
				pr_err("%s() reset fail rc %d\n", __func__, rc);
				return rc;
			}
		}

		rc = iaxxx_mem_sweep_test_bin_download(priv);
	}

	return count;
}
static DEVICE_ATTR(memsweep_test_start, 0200, NULL,
			iaxxx_memsweep_test_start);

static ssize_t iaxxx_spi_speed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 spi_speed;
	int rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &spi_speed))
		return -EINVAL;

	rc = iaxxx_update_intf_speed(dev, spi_speed);
	if (rc)
		return rc;

	return count;
}

static ssize_t iaxxx_spi_speed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	u32 spi_speed = 0;

	if (!priv->get_intf_speed) {
		dev_err(dev, "%s() Get Speed callback is NULL\n", __func__);
		return -EINVAL;
	}

	spi_speed = priv->get_intf_speed(dev);

	return scnprintf(buf, PAGE_SIZE,
			 "SPI speed = %dHz\n",
			 spi_speed);
}
static DEVICE_ATTR(spi_speed, 0600, iaxxx_spi_speed_show,
		   iaxxx_spi_speed_store);

static ssize_t iaxxx_bypass_regmap_cache(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!priv) {
		dev_err(dev, "%s priv is NULL\n", __func__);
		return -EINVAL;
	}

	dev_info(dev, "%s  Bypass Regmap Cache\n", __func__);

	/* Bypass regmap cache */
	regcache_cache_bypass(priv->regmap, true);

	return count;
}
static DEVICE_ATTR(bypass_regmap_cache, 0200, NULL,
		   iaxxx_bypass_regmap_cache);

static ssize_t iaxxx_reload_fw(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;

	if (!priv) {
		dev_err(dev, "%s priv is NULL\n", __func__);
		return -EINVAL;
	}

	dev_info(dev, "%s Reload FW\n", __func__);

	rc = iaxxx_fw_reload(priv);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(reload_fw, 0200, NULL,
		   iaxxx_reload_fw);

/**
 * Sysfs to change wakeup pin to SPI CS
 */
static ssize_t iaxxx_wakeup_from_cs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (!gpio_is_valid(priv->wakeup_gpio)) {
		dev_info(dev, "%s: wakeup pin is already SPI CS\n",
			 __func__);
		return count;
	}

	if (val) {
		dev_info(dev, "%s: wakeup pin SPI CS\n", __func__);
		priv->wakeup_from_cs = true;
	} else {
		dev_info(dev, "%s: wakeup pin wakeup GPIO\n", __func__);
		priv->wakeup_from_cs = false;
	}
	return count;
}

static ssize_t iaxxx_wakeup_from_cs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!gpio_is_valid(priv->wakeup_gpio))
		return scnprintf(buf, PAGE_SIZE, "Wakeup pin is SPI CS\n");

	return scnprintf(buf, PAGE_SIZE, "Wakeup pin is %s\n",
		priv->wakeup_from_cs ? "SPI CS" : "Wakeup GPIO");
}

static DEVICE_ATTR(wakeup_from_cs, 0600,
		iaxxx_wakeup_from_cs_show, iaxxx_wakeup_from_cs_store);

static struct attribute *iaxxx_attrs[] = {
	&dev_attr_fw_update_test.attr,
	&dev_attr_cmem_test.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_host_version.attr,
	&dev_attr_fw_timestamp.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_plugin_version.attr,
	&dev_attr_package_version.attr,
	&dev_attr_pm_wakeup_chip.attr,
	&dev_attr_chip_reset.attr,
	&dev_attr_isr_disable.attr,
	&dev_attr_simulate_fw_crash.attr,
	&dev_attr_runtime_pm_disable.attr,
	&dev_attr_fwcrash_handling_disable.attr,
	&dev_attr_get_current_power_mode.attr,
	&dev_attr_tnl_threshold.attr,
	&dev_attr_intf_max_speed.attr,
	&dev_attr_kw_detect_count.attr,
	&dev_attr_memsweep_test_start.attr,
	&dev_attr_spi_speed.attr,
	&dev_attr_bypass_regmap_cache.attr,
	&dev_attr_reload_fw.attr,
	&dev_attr_wakeup_from_cs.attr,
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
	.name	= "iaxxx"
};

int iaxxx_init_sysfs(struct iaxxx_priv *priv)
{
	int ret;
	/* Create sysfs */
	ret = sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group);
	if (ret) {
		dev_err(priv->dev,
			"%s: sysfs_create_group failed : %d\n", __func__, ret);
	}
	return ret;
}

void iaxxx_remove_sysfs(struct iaxxx_priv *priv)
{
	sysfs_remove_group(&priv->dev->kobj, &iaxxx_attr_group);
}
