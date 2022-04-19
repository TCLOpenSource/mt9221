/*
 * iaxxx-core.c -- IAxxx Multi-Function Device driver
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
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-script-mgr.h>
#include "iaxxx.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-tunnel.h"
#include "iaxxx-debug.h"
#include "iaxxx-sysfs.h"
#include "iaxxx-cdev.h"
#include "iaxxx-build-info.h"
#include <linux/circ_buf.h>
#include "mdrv_gpio.h"  /* Adapt for MT9615 */
#include "mdrv_pm.h"

#define IAXXX_RESET_RETRIES		5		/* retry attempts */
#define IAXXX_RESET_HOLD_TIME		(20*1000)	/* 20 ms */
#define IAXXX_RESET_READY_DELAY		(20*1000)	/* 20 ms */
#define IAXXX_RESET_RANGE_INTERVAL	100		/* 100 us */

#define WAKEUP_TIMEOUT			5000
#define IRQ_WAIT_TIMEOUT		3000

#define IAXXX_FW_RETRY_COUNT		2		/* retry attempts */
#define IAXXX_BYTES_IN_A_WORD		4

#define IAXXX_INVALID_EVT_CNT		0xFFFFFFFF

#define IAXXX_ISR_WAKE_RETRY_CNT	5

#define iaxxx_ptr2priv(ptr, item) container_of(ptr, struct iaxxx_priv, item)

#define FW_NAME "audience/ia8x01/RomeApp.bin"

enum {
	E_IAXXX_REGMAP_ERROR = -1,
	E_IAXXX_BOOTUP_ERROR = -2,
	E_IAXXX_RESET_SYNC_ERROR = -3
};

/* mutex for locking reset logic */
static DEFINE_MUTEX(iaxxx_reset_intrf_detect_mutex);

/*===========================================================================
 * MFD Driver
 *===========================================================================
 */

static struct mfd_cell iaxxx_devices[] = {
	{
		.name = "iaxxx-codec",
	},
	{
		.name = "iaxxx-alsa-tnl",
	},
	{
		.name = "iaxxx-odsp-celldrv",
	},
	{
		.name = "iaxxx-tunnel-celldrv",
	},
};

static struct iaxxx_skip_regdump iaxxx_regs_to_skip_regdump[] = {
	{ IAXXX_TNL_HDR_TNL_OUT_BUF_HEAD_ADDR, 0x0, IAXXX_READ },
	{ IAXXX_TNL_HDR_TNL_OUT_BUF_TAIL_ADDR, 0x0, IAXXX_READ }
};

static const char *iaxxx_crash_err2str(int error)
{
	switch (error) {
	case IAXXX_FW_CRASH_EVENT:
		return "crash event";
	case IAXXX_FW_CRASH_UPDATE_BLOCK_REQ:
		return "crash during update block req";
	case IAXXX_FW_CRASH_TUNNEL_WRONG_BUFF:
		return "forced recovery after wrong tunnel buff params";
	case IAXXX_FW_CRASH_RESUME:
		return "crash during resume";
	case IAXXX_FW_CRASH_SUSPEND:
		return "crash during suspend";
	case IAXXX_FW_CRASH_SIMULATED:
		return "simulated crash for testing";
	default:
		return "unknown error";
	}
}

/**
 * iaxxx_send_uevent - Send uevent KOBJ_CHANGE
 *
 * @priv	: iaxxx private data
 * @type	  Type of event
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_send_uevent(struct iaxxx_priv *priv, char *type)
{
	char *event[2] = {type, NULL};

	/* Send event to HAL */
	return kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, event);
}

static int iaxxx_fw_bootup_regmap_init(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	bool status = 0;
	int rc;

	/*
	 * If the initial bootup try fails, priv->regmap are cleared. In such
	 * cases, on the next trial, ->regmap usage will lead to crash.
	 * Make sure regmap is present before using it.
	 */
	if (!priv->regmap) {
		dev_err(dev, "%s: regmap is not populated. err out\n",
							__func__);
		rc = -EINVAL;
		return rc;
	}

	rc = iaxxx_check_bootloader_status(priv,
			0x1 << IAXXX_SRB_BOOT_REQ_JUMP_REQ_POS, &status);
	if (rc) {
		dev_err(dev, "bootloader status check failed : %d\n", rc);
		goto err_regmap;
	} else if (!status) {
		dev_err(dev, "bootup request is not complete\n");
		rc = -EINVAL;
		goto err_regmap;
	}

	/* Initialize "application" regmap */
	rc = iaxxx_application_regmap_init(priv);
	if (rc)
		goto err_regmap;

	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_switch_regmap(dev, priv->regmap, priv->dfs_node);
	if (rc)
		dev_err(dev, "create debugfs entry failed : %d\n", rc);
	else
		dev_info(dev, "%s: debugfs entry created\n", __func__);

	return rc;

err_regmap:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}

	return rc;
}

/* Clear system state */
static void clear_system_state(struct iaxxx_priv *priv)
{
	iaxxx_clr_pkg_plg_list(priv);
	iaxxx_clr_script_list(priv);
	atomic_set(&priv->iaxxx_state->power_state, IAXXX_NOCHANGE);
	memset(&priv->iaxxx_state->err, 0, sizeof(struct iaxxx_block_err));
}

static int iaxxx_fw_recovery_regmap_init(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int rc = 0;
	bool status = 0;

	rc = iaxxx_check_bootloader_status(priv,
			0x1 << IAXXX_SRB_BOOT_REQ_JUMP_REQ_POS, &status);
	if (rc) {
		dev_err(dev, "bootloader status check failed : %d\n",
			rc);
		goto regmap_init_failed;
	} else if (!status) {
		rc = -EINVAL;
		dev_err(dev, "%s: bootup request is not complete\n", __func__);
		goto regmap_init_failed;
	}

	if (test_bit(IAXXX_FLG_FW_RELOAD, &priv->flags)) {
		rc = iaxxx_application_regmap_init(priv);
		if (rc)
			goto regmap_init_failed;

		rc = iaxxx_dfs_switch_regmap(priv->dev, priv->regmap,
					     priv->dfs_node);
		if (rc)
			dev_err(priv->dev, "create debugfs entry failed : %d\n",
				rc);
		else
			dev_info(priv->dev, "%s: debugfs entry created\n",
				 __func__);

	} else {
		rc = iaxxx_regmap_drop_regions(priv);
		if (rc) {
			dev_err(priv->dev,
				"%s: drop regmap regions recovery failed : %d\n",
				__func__, rc);
			goto regmap_recovery_failed;
		}
		regcache_cache_bypass(priv->regmap, false);
	}

	/* Clear system state */
	clear_system_state(priv);

	priv->is_application_mode = true;

	dev_info(priv->dev, "%s: FW Reload done\n", __func__);
regmap_recovery_failed:
regmap_init_failed:
	return rc;
}

static int iaxxx_do_suspend(struct iaxxx_priv *priv)
{
	if (test_bit(IAXXX_FLG_FW_RELOAD, &priv->flags)) {
		pr_info("dev id-%d: Reload request\n", priv->dev_id);
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RELOAD_START, NULL);
		iaxxx_work(priv, fw_update_work);
	} else if (test_bit(IAXXX_FLG_FW_RESET, &priv->flags)) {
		pr_info("dev id-%d: FW Reset request\n", priv->dev_id);
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_FW_RESET, NULL);
		iaxxx_work(priv, fw_update_work);
	} else {
		pr_info("dev id-%d: FW crash", priv->dev_id);
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_CRASH, NULL);
		iaxxx_work(priv, fw_crash_work);
	}

	return 0;
}

static int iaxxx_do_resume(struct iaxxx_priv *priv)
{
	unsigned long *flags = &priv->flags;
	char action[IAXXX_MAX_UEVENT_STRING_LENGTH];

	if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_STARTUP, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_STARTUP, NULL);

		/* Send firmware startup event to HAL */
		snprintf(action, sizeof(action),
			"ACTION=IAXXX_FW_STARTUP_EVENT-%d", priv->dev_id);
		iaxxx_send_uevent(priv, (char *)action);
		snprintf(action, sizeof(action),
			"ACTION=IAXXX_FW_DWNLD_SUCCESS-%d", priv->dev_id);
		iaxxx_send_uevent(priv, (char *)action);
	} else if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_RECOVERY, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RECOVERY, NULL);

		/* Send recovery event to HAL */
		snprintf(action, sizeof(action),
			"ACTION=IAXXX_RECOVERY_EVENT-%d", priv->dev_id);
		iaxxx_send_uevent(priv, (char *)action);
	} else if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_FW_RELOAD, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RELOAD_DONE, NULL);

		/* Send reload event to HAL */
		snprintf(action, sizeof(action),
			"ACTION=IAXXX_RELOAD_EVENT-%d", priv->dev_id);
		iaxxx_send_uevent(priv, (char *)action);
	} else if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_FW_RESET, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RECOVERY, NULL);

		/* Send reset event to HAL */
		snprintf(action, sizeof(action),
			"ACTION=IAXXX_RESET_EVENT-%d", priv->dev_id);
		iaxxx_send_uevent(priv, (char *)action);
	}

	return 0;
}

#if defined(CONFIG_ARCH_MSM8996)
static int iaxxx_get_pinctrl(struct iaxxx_priv *priv)
{
	struct pinctrl_info *pnctrl_info;
	struct pinctrl *pinctrl;

	pr_debug("dev id-%d: Checking for pinctrl Support", priv->dev_id);

	pnctrl_info = &priv->pnctrl_info;
	pnctrl_info->pinctrl = NULL;
	pnctrl_info->has_pinctrl = false;

	pinctrl = devm_pinctrl_get(priv->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(priv->dev, "%s: Unable to get pinctrl handle\n",
			__func__);
		return -EINVAL;
	}

	pnctrl_info->pinctrl = pinctrl;
	pnctrl_info->has_pinctrl = true;

	return 0;
}

static int iaxxx_lookup_and_select_pinctrl(struct iaxxx_priv *priv,
			struct pinctrl_state **state_pin, const char *state_str)
{
	struct pinctrl_info *pnctrl_info;
	struct pinctrl *pinctrl;
	int ret;

	pr_debug("dev id-%d: setting up %s pinctrl", priv->dev_id, state_str);

	pnctrl_info = &priv->pnctrl_info;

	if (pnctrl_info->has_pinctrl == false) {
		dev_warn(priv->dev, "%s: Pinctrl not present\n", __func__);
		return -EINVAL;
	}

	pinctrl = pnctrl_info->pinctrl;

	*state_pin = pinctrl_lookup_state(pinctrl, state_str);
	if (IS_ERR(*state_pin)) {
		ret = PTR_ERR(*state_pin);
		dev_warn(priv->dev, "%s: lookup for %s state failed : %d\n",
			 __func__, state_str, ret);
		goto err;
	}

	/* Select the pins to state requested */
	ret = pinctrl_select_state(pnctrl_info->pinctrl, *state_pin);
	if (ret) {
		dev_warn(priv->dev, "%s: select state %s failed : %d\n",
			__func__, state_str, ret);
		goto err;
	}

	return 0;

err:
	devm_pinctrl_put(pnctrl_info->pinctrl);
	return ret;
}
#endif

/**
 * iaxxx_gpio_free - free a gpio for a managed device
 * @dev: device to free the gpio for
 * @gpio: GPIO to free
 */
static inline void iaxxx_gpio_free(struct device *dev, unsigned int gpio)
{
#if 0 /* Adapt for MT9615 who doesn't need this */
	devm_gpio_free(dev, gpio);
	dev_dbg(dev, "%s: %d\n", __func__, gpio);
#endif
}

/**
 * get_named_gpio - Gets a GPIO property from a device tree node
 */
static inline int get_named_gpio(struct device *dev, const char *name)
{
	int rc;

	rc = of_get_named_gpio(dev->of_node, name, 0);
	if (rc < 0) {
		dev_err(dev, "Looking up %s property in node %s failed : %d\n",
			name, dev->of_node->full_name, rc);
		return rc;
	}
	dev_dbg(dev, "%s: %s %d\n", __func__, name, rc);
	return rc;
}

/**
 * iaxxx_populate_dt_gpios - populates GPIO data from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_gpios(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;
	struct device_node *node = dev->of_node;

	dev_dbg(dev, "%s: enter\n", __func__);

#if 0 /* Knowles original */
	rc = get_named_gpio(dev, "adnc,reset-gpio");
	if (rc < 0) {
		priv->reset_gpio = -EINVAL;
		dev_err(dev, "read reset-gpio failed : %d\n", rc);
		return rc;
	}
	priv->reset_gpio = rc;

	rc = get_named_gpio(dev, "adnc,event-gpio");
	if (rc < 0) {
		priv->event_gpio = -EINVAL;
		dev_err(dev, "read event-gpio gpio failed : %d\n", rc);
		return rc;
	}
	priv->event_gpio = rc;
#else  /* Adapt for MT9615 */
	rc = of_property_read_u32(node, "adnc,reset-gpio", &priv->reset_gpio);
	if (rc < 0) {
		priv->reset_gpio = -EINVAL;
		dev_err(dev, "read reset-gpio failed : %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "adnc,event-gpio", &priv->event_gpio);
	if (rc < 0) {
		priv->event_gpio = -EINVAL;
		dev_err(dev, "read event-gpio gpio failed : %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "adnc,tvonoff-gpio", &priv->tvonoff_gpio);
	if (rc < 0) {
		priv->tvonoff_gpio = -EINVAL;
		dev_err(dev, "read tvonoff-gpio failed : %d\n", rc);
		return rc;
	}
#endif

#ifdef CONFIG_IA8X01_WAKEUP_GPIO
	rc = get_named_gpio(dev, "adnc,wakeup-gpio");
	if (rc < 0) {
		dev_warn(dev, "wakeup-gpio dts entry is missing : %d\n", rc);
		priv->wakeup_gpio = -EINVAL;
		return rc;
	}

	priv->wakeup_gpio = rc;
#else
	priv->wakeup_gpio = -1;
#endif

	return 0;
}

/**
 * iaxxx_populate_dt_pdata - populate platform data from device tree
 */
static int iaxxx_populate_dt_pdata(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;
	const char *fw_name = NULL;
	size_t len;

	dev_dbg(dev, "%s: enter\n", __func__);

	if (!dev->of_node) {
		dev_err(dev, "Missing device tree information\n");
		return -ENODEV;
	}

	rc = iaxxx_populate_dt_gpios(priv);
	if (rc) {
		dev_err(dev, "read GPIO data failed : %d\n", rc);
		return rc;
	}

#if defined(CONFIG_ARCH_MSM8996)
	rc = iaxxx_get_pinctrl(priv);
	if (rc < 0)
		dev_err(priv->dev, "%s: unable to get pinctrl. rc %d\n",
								__func__, rc);
	rc = iaxxx_lookup_and_select_pinctrl(priv,
			&priv->pnctrl_info.active_gpios, "active-gpios");
	if (rc < 0)
		dev_err(priv->dev, "%s: set reset gpio pinctrl failed : %d\n",
								__func__, rc);
#endif

	rc = of_property_read_string(dev->of_node, "adnc,fw-name", &fw_name);
	/* error out only if fw name is found but unreadable */
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw image name %d\n", rc);
		return rc;
	}

	if (fw_name) {
		len = strlen(fw_name);
		if (len > MAX_FILENAME_BUFFER_SIZE - 1) {
			dev_err(dev, "%s: Invalid fw-name (%s). ignoring\n",
							__func__, fw_name);
			fw_name = NULL;
		}
	}

	snprintf(priv->fw_name, MAX_FILENAME_BUFFER_SIZE, "%s",
		fw_name ? fw_name : FW_NAME);

	return 0;
}

/**
 * iaxxx_gpio_init(): Requests the GPIO for the device
 */
static int iaxxx_gpio_init(struct iaxxx_priv *priv)
{
	int rc = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s: enter\n", __func__);

#if 0  /* Adapt for MT9615 who doesn't need this */
	/* GPIO: event (input used for interrupts) */
	/* TODD: interrupt is active HIGH so the GPIO needs pull-down */
	rc = devm_gpio_request_one(dev, priv->event_gpio,
						GPIOF_DIR_IN, "EVENT");
	if (rc < 0)
		goto err_missing_event_gpio;

	/* GPIO: reset (output used for chip reset) */
	rc = devm_gpio_request_one(dev, priv->reset_gpio,
						GPIOF_OUT_INIT_LOW, "RESET");
	if (rc < 0)
		goto err_missing_reset_gpio;

	/* GPIO: wakeup (output used for chip wakeup) */
	if (gpio_is_valid(priv->wakeup_gpio)) {
		rc = devm_gpio_request_one(dev, priv->wakeup_gpio,
						GPIOF_OUT_INIT_HIGH, "WAKEUP");
		if (rc < 0)
			goto err_missing_wakeup_gpio;
	}

	return 0;

err_missing_wakeup_gpio:
	iaxxx_gpio_free(dev, priv->reset_gpio);
err_missing_reset_gpio:
	iaxxx_gpio_free(dev, priv->event_gpio);
err_missing_event_gpio:
#endif
	return rc;
}

/*
 * iaxxx_kvmalloc - do allocation preferring kmalloc if size is less than
 * PAGE_SIZE but fall back to vmalloc if kmalloc fails or size requested is
 * greater than a PAGE_SIZE
 *
 * @size: how many bytes of memory are required
 * @flags: the type of memory to allocate (see kmalloc).
 *
 * Return: allocated buffer or NULL if failed
 */
void *iaxxx_kvmalloc(size_t size, gfp_t flags)
{
	void *buffer = NULL;

	if (size == 0)
		return NULL;

	/* Use kmalloc first for less than PAGE_SIZE with no warning */
	if (size <= PAGE_SIZE)
		buffer = kmalloc(size, flags | GFP_KERNEL | __GFP_NOWARN);

	/* If size is more than PAGE_SIZE or prev allocation fails */
	if (!buffer) {
		if (flags & __GFP_ZERO)
			buffer = vzalloc(size);
		else
			buffer = vmalloc(size);
	}

	return buffer;
}
EXPORT_SYMBOL(iaxxx_kvmalloc);

static void dump_to_log(struct device *dev,
		struct iaxxx_reg_dump_priv *reg_dump,
		struct iaxxx_register_log *log)
{
	spin_lock(&reg_dump->ring_lock);
	/* Check if Log buffer has space, if not increment the tail index to
	 * get buffer to overwrite oldest data
	 */
	if (!(CIRC_SPACE(reg_dump->head, reg_dump->tail, IAXXX_BUF_MAX_LEN))) {
		dev_dbg(dev, "%s: Register log buffer is full\n",
				__func__);
		reg_dump->tail++;
		reg_dump->tail %= IAXXX_BUF_MAX_LEN;
	}
	reg_dump->log[reg_dump->head] = *log;
	reg_dump->head++;
	/* Align with the Ring Buffer boundary */
	reg_dump->head %= IAXXX_BUF_MAX_LEN;
	spin_unlock(&reg_dump->ring_lock);
}

static bool is_reg_in_regdump_skip_list(struct iaxxx_priv *priv,
		uint32_t reg_phy_addr, u8 op)
{
	int i;

	/* If the firmware is not ready, the skip list
	 * cannot be checked because virtual to physical
	 * address map not available.
	 */
	if (!iaxxx_is_firmware_ready(priv))
		return false;

	for (i = 0; i < ARRAY_SIZE(iaxxx_regs_to_skip_regdump); i++) {
		/* Since register address passed here is physical
		 * address, convert the virtual address in the
		 * skip list to physical address for comparison.
		 */
		if (!iaxxx_regs_to_skip_regdump[i].reg_phy_addr)
			iaxxx_regs_to_skip_regdump[i].reg_phy_addr =
				iaxxx_conv_virtual_to_physical_register_address
				(priv, iaxxx_regs_to_skip_regdump[i].reg);

		if ((iaxxx_regs_to_skip_regdump[i].reg_phy_addr == reg_phy_addr)
			&& (iaxxx_regs_to_skip_regdump[i].op == op))
			return true;

	}
	return false;
}

void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
		uint8_t op)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_reg_dump_priv *reg_dump = priv->reg_dump;
	struct iaxxx_register_log log;

	if (!priv || !priv->reg_dump || !priv->reg_dump->log) {
		pr_err("struct is NULL");
		return;
	}

	/* If the register and operation done is in the skip list
	 * then skip the regdump
	 */
	if (is_reg_in_regdump_skip_list(priv, reg, op))
		return;

	log.val = val;
	log.addr = reg;
	log.op = op;
	get_monotonic_boottime(&log.timestamp);
	/* Add the log into circular buffer */
	dump_to_log(dev, reg_dump, &log);
}


int iaxxx_update_intf_speed(struct device *dev, uint32_t intf_speed)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc;

	dev_info(dev, "%s Set speed %d\n", __func__, intf_speed);

	if (!priv->intf_speed_setup) {
		dev_err(dev, "%s() Speed setup callback is NULL\n", __func__);
		return -EINVAL;
	}

	rc = priv->intf_speed_setup(dev, intf_speed);
	if (rc) {
		dev_err(dev, "%s() Speed setup fail, rc = %d\n", __func__, rc);
		return rc;
	}

	/* Update app speed to retain value after recovery */
	priv->app_speed  = intf_speed;

	/*
	 * Update interface max speed so that before requesting OPTIMAL
	 * mode, current SPI speed will be used
	 */
	priv->intf_max_speed  = intf_speed;

	return 0;
}
EXPORT_SYMBOL(iaxxx_update_intf_speed);

/**
 * iaxxx_reset(): Reset IAxxx device
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
static int iaxxx_reset(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s: reset iaxxx\n", __func__);

	if (!gpio_is_valid(priv->reset_gpio)) {
		dev_err(dev, "%s: reset_gpio(%d) is an invalid gpio.\n",
						__func__, priv->reset_gpio);
		return -EIO;
	}

	gpio_set_value(priv->reset_gpio, 0);
	usleep_range(IAXXX_RESET_HOLD_TIME,
		IAXXX_RESET_HOLD_TIME + IAXXX_RESET_RANGE_INTERVAL);

	gpio_set_value(priv->reset_gpio, 1);
	usleep_range(IAXXX_RESET_READY_DELAY, IAXXX_RESET_READY_DELAY +
		IAXXX_RESET_RANGE_INTERVAL);

	return 0;
}

/**
 * iaxxx_reset_to_sbl - Boots the chip hardware
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
int iaxxx_reset_to_sbl(struct iaxxx_priv *priv)
{
	int rc = 0;
	struct device *dev = priv->dev;

	dev_info(dev, "%s: enter\n", __func__);

	/* Reset the chip */
	rc = iaxxx_reset(priv);
	if (rc)
		dev_err(dev, "%s: device reset failed : %d\n", __func__, rc);

	return rc;
}

#if 0 /* Knowles original */
/**
 * iaxxx_event_isr - Interrupt / Event handler
 *
 * @irq	 : interrupt number
 * @data : iaxxx private data
 */
static irqreturn_t iaxxx_event_isr(int irq, void *data)
{
	int rc;
	uint32_t count;
	int retry_cnt = IAXXX_ISR_WAKE_RETRY_CNT;
	bool handled = false;
	bool is_startup;
	uint32_t status = 0;
	struct iaxxx_priv *priv = (struct iaxxx_priv *)data;
	struct regmap *regmap;

	/* If ISR is disabled, return as handled */
	if (priv->debug_isr_disable || !priv->jump_to_app_mode) {
		pr_info("dev id-%d: Invalid Interrupt", priv->dev_id);
		return IRQ_HANDLED;
	}

	dev_info(priv->dev, "%s: IRQ %d\n", __func__, irq);

	pm_wakeup_event(priv->dev, WAKEUP_TIMEOUT);

	if (!atomic_read(&priv->pm_resume)) {
		rc = wait_event_timeout(priv->irq_wake,
			atomic_read(&priv->pm_resume),
			msecs_to_jiffies(IRQ_WAIT_TIMEOUT));
		if (rc == 0 && !atomic_read(&priv->pm_resume)) {
			dev_err(priv->dev,
				"IRQ wait resume timeout! failed : %d\n", rc);
			goto out;
		}
	}

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto out;
	}

	if (!test_and_set_bit(IAXXX_FLG_CHIP_WAKEUP,
					&priv->flags)) {
		/* On any event always assume chip is awake */
		wake_up(&priv->wakeup_wq);
		dev_info(priv->dev,
		"%s: FW is expected to be in wakeup state\n", __func__);
	}

retry_reading_count_reg:
	/*
	 * Since this ISR can happen anytime,
	 * choose which regmap to use to read the event
	 * based on boot state.
	 */
	regmap = !iaxxx_is_firmware_ready(priv) ? priv->regmap_no_pm :
			priv->regmap;
	/* Any events in the event queue? */
	rc = regmap_read(regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(priv->dev,
			"read EVENT_COUNT failed : %d\n", rc);
		count = 0;
	}

	if ((count > 0 && count < IAXXX_INVALID_EVT_CNT) && priv->event_workq) {
		dev_info(priv->dev, "%s: %d event(s) avail\n", __func__, count);
		queue_work(priv->event_workq, &priv->event_work_struct);
		handled = true;
	} else {
		/* Read SYSTEM_STATUS to ensure that device is in App Mode */
		rc = regmap_read(regmap,
					IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (rc)
			dev_err(priv->dev,
				"%s: read SYSTEM_STATUS failed : %d\n",
				__func__, rc);

		dev_info(priv->dev, "%s: fw status : 0x%x\n", __func__, status);

		if (status != SYSTEM_STATUS_MODE_APPS) {
			dev_err(priv->dev,
				"%s: Not in app mode, status = %d\n", __func__,
				status);
			priv->core_crashed = true;
			queue_work(priv->event_workq, &priv->event_work_struct);
			handled = true;
			goto out;
		} else if (status == SYSTEM_STATUS_MODE_APPS && retry_cnt--) {
			goto retry_reading_count_reg;
		}
	}

	complete_all(&priv->cmem_done);
out:
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

#else  /* Adapt for MT9615 who doesn't support request_threaded_irq() */

static void iaxxx_threaded_isr_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, threaded_isr_work);
	int rc;
	uint32_t count;
	int retry_cnt = IAXXX_ISR_WAKE_RETRY_CNT;
	bool handled = false;
	bool is_startup;
	uint32_t status = 0;
	struct regmap *regmap;

	dev_info(priv->dev, "%s\n", __func__);

	if (!atomic_read(&priv->pm_resume)) {
		rc = wait_event_timeout(priv->irq_wake,
			atomic_read(&priv->pm_resume),
			msecs_to_jiffies(IRQ_WAIT_TIMEOUT));
		if (rc == 0 && !atomic_read(&priv->pm_resume)) {
			dev_err(priv->dev,
				"IRQ wait resume timeout! failed : %d\n", rc);
			goto out;
		}
	}

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto out;
	}

	if (!test_and_set_bit(IAXXX_FLG_CHIP_WAKEUP,
					&priv->flags)) {
		/* On any event always assume chip is awake */
		wake_up(&priv->wakeup_wq);
		dev_info(priv->dev,
		"%s: FW is expected to be in wakeup state\n", __func__);
	}

retry_reading_count_reg:
	/*
	 * Since this ISR can happen anytime,
	 * choose which regmap to use to read the event
	 * based on boot state.
	 */
	regmap = !iaxxx_is_firmware_ready(priv) ? priv->regmap_no_pm :
			priv->regmap;
	/* Any events in the event queue? */
	rc = regmap_read(regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(priv->dev,
			"read EVENT_COUNT failed : %d\n", rc);
		count = 0;
	}

	if ((count > 0 && count < IAXXX_INVALID_EVT_CNT) && priv->event_workq) {
		dev_info(priv->dev, "%s: %d event(s) avail\n", __func__, count);
		queue_work(priv->event_workq, &priv->event_work_struct);
		handled = true;
	} else {
		/* Read SYSTEM_STATUS to ensure that device is in App Mode */
		rc = regmap_read(regmap,
					IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (rc)
			dev_err(priv->dev,
				"%s: read SYSTEM_STATUS failed : %d\n",
				__func__, rc);

		dev_info(priv->dev, "%s: fw status : 0x%x\n", __func__, status);

		if (status != SYSTEM_STATUS_MODE_APPS) {
			dev_err(priv->dev,
				"%s: Not in app mode, status = %d\n", __func__,
				status);
			priv->core_crashed = true;
			queue_work(priv->event_workq, &priv->event_work_struct);
			handled = true;
			goto out;
		} else if (status == SYSTEM_STATUS_MODE_APPS && retry_cnt--) {
			goto retry_reading_count_reg;
		}
	}

	complete_all(&priv->cmem_done);
out:
	// return handled ? IRQ_HANDLED : IRQ_NONE;
	return;
}

/**
 * iaxxx_event_isr - Interrupt / Event handler
 *
 * @irq	 : interrupt number
 * @data : iaxxx private data
 */
static irqreturn_t iaxxx_event_isr(int irq, void *data)
{
	struct iaxxx_priv *priv = (struct iaxxx_priv *)data;

	/* If ISR is disabled, return as handled */
	if (priv->debug_isr_disable || !priv->jump_to_app_mode) {
		pr_info("dev id-%d: Invalid Interrupt", priv->dev_id);
		return IRQ_HANDLED;
	}

	dev_info(priv->dev, "%s: IRQ %d\n", __func__, irq);

	// pm_wakeup_event(priv->dev, WAKEUP_TIMEOUT);
	iaxxx_work(priv, threaded_isr_work);

	return IRQ_HANDLED;
}
#endif

/**
 * iaxxx_irq_init - Initialize interrupt handling
 *
 * @priv : iaxxx private data
 */
static int iaxxx_irq_init(struct iaxxx_priv *priv)
{
	int rc;

	if (!gpio_is_valid(priv->event_gpio)) {
		pr_err("dev id-%d: invalid event gpio", priv->dev_id);
		return -ENXIO;
	}

	irq_set_status_flags(gpio_to_irq(priv->event_gpio), IRQ_DISABLE_UNLAZY);
#if 0  /* Knowles original */
	rc = request_threaded_irq(gpio_to_irq(priv->event_gpio), NULL,
		iaxxx_event_isr,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT |
		IRQF_EARLY_RESUME | IRQF_NO_SUSPEND,
		"iaxxx-event-irq", priv);
	if (rc) {
		dev_err(priv->dev,
			"%s(): request_threaded_irq() fail %d\n",
			__func__, rc);
		goto err_out;
	}

	rc = enable_irq_wake(gpio_to_irq(priv->event_gpio));
	if (rc < 0) {
		pr_err("dev id-%d: enable irq wake failed : %d",
							priv->dev_id, rc);
		disable_irq(gpio_to_irq(priv->event_gpio));
		free_irq(gpio_to_irq(priv->event_gpio), priv);
		goto err_out;
	}
#else  /* Adapt for MT9615 */
	rc = request_gpio_irq(priv->event_gpio, iaxxx_event_isr,
					IRQF_TRIGGER_RISING, priv);
	if (rc) {
		dev_err(priv->dev,
			"%s(): request_gpio_irq() fail %d\n",
			__func__, rc);
		goto err_out;
	}
#endif
	/* disable the irq until fw is loaded */
	disable_irq(gpio_to_irq(priv->event_gpio));
	priv->is_irq_enabled = false;

err_out:
	return rc;
}

/**
 * iaxxx_irq_exit - Frees the event IRQ
 *
 * @priv	: iaxxx private data
 */
static void iaxxx_irq_exit(struct iaxxx_priv *priv)
{
	if (gpio_is_valid(priv->event_gpio))
		free_irq(gpio_to_irq(priv->event_gpio), priv);
}

static int iaxxx_regdump_init(struct iaxxx_priv *priv)
{
	dev_dbg(priv->dev, "%s: enter\n", __func__);

	priv->reg_dump = iaxxx_kvmalloc(sizeof(struct iaxxx_reg_dump_priv),
			__GFP_ZERO);
	if (!priv->reg_dump)
		return -ENOMEM;

	priv->reg_dump->log = iaxxx_kvmalloc(sizeof(struct iaxxx_register_log)
			* IAXXX_BUF_MAX_LEN, __GFP_ZERO);
	if (!priv->reg_dump->log) {
		kvfree(priv->reg_dump);
		priv->reg_dump = NULL;
		return -ENOMEM;
	}

	spin_lock_init(&priv->reg_dump->ring_lock);
	return 0;
}

void iaxxx_regdump_exit(struct iaxxx_priv *priv)
{
	kvfree(priv->reg_dump->log);
	priv->reg_dump->log = NULL;
	kvfree(priv->reg_dump);
	priv->reg_dump = NULL;
}


/**
 * iaxxx_do_fw_update - reset and sync target and do firmware update
 *
 * @priv	: iaxxx private data
 * Return
 *       0                    Success
 *       E_IAXXX_REGMAP_ERROR error accessing regmap
 *       E_IAXXX_BOOTUP_ERROR target bootup failure
 *
 */
static int iaxxx_do_fw_update(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t reg, status, mem_elec_ctrl_val;
	uint32_t mode = SYSTEM_STATUS_MODE_RESET;
	struct device *dev = priv->dev;
	int mode_retry = 5;

	gpio_set_value(priv->tvonoff_gpio, 0);

	mutex_lock(&iaxxx_reset_intrf_detect_mutex);
	if (priv->reset_cb) {
		rc = priv->reset_cb(dev);
		if (rc) {
			dev_err(dev, "reset/sync failed : %d\n", rc);
			mutex_unlock(&iaxxx_reset_intrf_detect_mutex);
			return E_IAXXX_RESET_SYNC_ERROR;
		}
	}

	mutex_unlock(&iaxxx_reset_intrf_detect_mutex);

	/* Read Efuse Boot address, to get cores and interface details */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_AO_EFUSE_BOOT_ADDR,
			 &status);
	if (rc) {
		dev_err(dev, "regmap_read failed : %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_info(dev, "Efuse Boot register val 0x%08X\n", status);

	do {
		/* Verify that the device is in bootloader mode */
		rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_STATUS_ADDR,
				&status);
		if (rc) {
			dev_err(dev, "regmap_read failed : %d\n", rc);
			return E_IAXXX_REGMAP_ERROR;
		}

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_info(dev, "System Status: 0x%08X mode: 0x%08X\n", status,
				mode);
	} while (!mode && mode_retry--);

	priv->boot_completed      = false;
	priv->is_application_mode = false;

	if (!mode && !mode_retry) {
		dev_err(dev, "SBL SYS MODE retry expired\n");
		return -ETIMEDOUT;
	}

	if (mode != SYSTEM_STATUS_MODE_SBL) {
		dev_err(dev, "Chip is in %d mode\n", mode);
		return -EINVAL;
	}

	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap_no_pm,
			 IAXXX_SRB_SYS_DEVICE_ID_ADDR, &reg);
	if (rc) {
		dev_err(dev, "regmap_read failed : %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_info(dev, "Device ID: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap_no_pm,
			 IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, &reg);
	if (rc) {
		dev_err(dev, "regmap_read failed : %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_info(dev, "ROM Version: 0x%.08X\n", reg);


	/* Electrical control for Memory
	 * Value 2(Normal-Read 2) for read and write margins for ROMs.
	 * Value 0(Slowest Read, Slowest Write) for read and write margins
	 * for SRAMs
	 * Value 8(Medium Leakage) for Retention Voltage for PD8 memories.
	 *
	 *  This fix is required for FW download on FF part or else we get
	 *  checksum error after downloading it. It fixes chip
	 *  stability issues due to clock changes and heating issues.
	 *
	 */
	mem_elec_ctrl_val =
		((2 << IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_ROM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_ROM_MASK) |
		((0 << IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_RAM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_RAM_MASK) |
		((8 << IAXXX_AO_MEM_ELEC_CTRL_PD8_BTRIM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_PD8_BTRIM_MASK);

	rc = regmap_write(priv->regmap_no_pm, IAXXX_AO_MEM_ELEC_CTRL_ADDR,
			mem_elec_ctrl_val);
	if (rc) {
		dev_err(dev, "Electrical control register failed : %d\n",
				rc);
		return E_IAXXX_REGMAP_ERROR;
	}

	/* Boot the device into application mode */
	rc = iaxxx_bootup(priv);
	if (rc) {
		dev_err(dev, "bootup failed : %d\n", rc);
		return E_IAXXX_BOOTUP_ERROR;
	}

	return 0;
}

/**
 * iaxxx_fw_update_work - worker thread to download firmware.
 *
 * @work : used to retrieve private structure
 *
 * Firmware download and switch to application mode of firmware.
 *
 */
static void iaxxx_fw_update_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_update_work);
	struct device *dev = priv->dev;
	char action[IAXXX_MAX_UEVENT_STRING_LENGTH];
	bool is_startup;
	int rc;

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);

	if (!priv->regmap_no_pm) {
		dev_err(dev, "%s: regmap_no_pm is not populated. err out\n",
							__func__);
		goto exit_fw_fail;
	}

	rc = iaxxx_do_fw_update(priv);

	if (rc == E_IAXXX_BOOTUP_ERROR) {
		/* If there's device reset cb, retry */
		if (!priv->reset_cb) {
			dev_err(dev, "%s: Reset callback is missing\n",
				__func__);
			goto exit_fw_fail;
		}
		if (++priv->try_count < IAXXX_FW_RETRY_COUNT) {
			dev_err(dev, "%s: Bootup error. retrying... %d\n",
						__func__, priv->try_count);
			iaxxx_work(priv, fw_update_work);
			return;
		}
		dev_err(dev, "%s: %d retry failed! EXIT\n",
				__func__, IAXXX_FW_RETRY_COUNT);
		goto exit_fw_fail;
	} else if (rc < 0) {
		dev_err(dev, "%s: fail, %d\n", __func__, rc);
		goto exit_fw_fail;
	}

	priv->try_count = 0;

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto exit_fw_fail;

		iaxxx_flush_event_list(priv);
		priv->boot_completed = true;
	}

	/* Send firmware ready event to HAL */
	snprintf(action, sizeof(action),
			"ACTION=IAXXX_FW_READY_EVENT-%d", priv->dev_id);
	iaxxx_send_uevent(priv, (char *)action);

	set_bit(IAXXX_FLG_FW_READY, &priv->flags);
	/* Reset route status */
	iaxxx_core_set_route_status(priv, false);

	atomic_set(&priv->iaxxx_state->power_state, IAXXX_NORMAL_MODE);

	rc = iaxxx_get_system_clk_src(priv);
	if (rc) {
		dev_err(priv->dev, "%s: Read system clock src failed : %d\n",
			__func__, rc);
		goto exit_fw_fail;
	}

	iaxxx_fw_notifier_call(dev, IAXXX_EV_APP_MODE, NULL);

	if (test_and_clear_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		set_bit(IAXXX_FLG_RESUME_BY_RECOVERY, &priv->flags);
	else if (test_and_clear_bit(IAXXX_FLG_FW_RELOAD, &priv->flags))
		set_bit(IAXXX_FLG_RESUME_BY_FW_RELOAD, &priv->flags);
	else if (test_and_clear_bit(IAXXX_FLG_FW_RESET, &priv->flags))
		set_bit(IAXXX_FLG_RESUME_BY_FW_RESET, &priv->flags);
	else {
		set_bit(IAXXX_FLG_RESUME_BY_STARTUP, &priv->flags);

		dev_info(dev, "adding mfd devices for device id : %d",
								priv->dev_id);
		rc = mfd_add_devices(priv->dev, priv->dev_id, iaxxx_devices,
			ARRAY_SIZE(iaxxx_devices), NULL, 0, NULL);
		if (rc) {
			dev_err(dev, "add cell devices failed : %d\n",
									rc);
			goto exit_fw_fail;
		}
	}

	iaxxx_work(priv, runtime_work);

	/* Subscribing for FW wakeup event */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CORE_CTRL_MGR_SRC_ID,
			IAXXX_WAKEUP_EVENT_ID, IAXXX_SYSID_HOST, 0);
	if (rc) {
		dev_err(dev,
			"%s: subscribe for wakeup event failed : %d\n",
			__func__, rc);
		goto exit_fw_fail;
	}

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	iaxxx_pm_enable(priv);

	rc = device_init_wakeup(priv->dev, true);
	if (rc < 0) {
		pr_err("dev id-%d: device_init_wakeup fail %d",
							priv->dev_id, rc);
		goto exit_fw_fail;
	}

	if (priv->debug_runtime_pm_disable) {
		/* wake up chip */
		rc = iaxxx_pm_get_sync(dev);
		if (rc < 0)
			pr_err("dev id-%d: pm get sync fail %d",
							priv->dev_id, rc);

		pm_runtime_put_sync(dev);

		pm_runtime_forbid(dev);
		dev_info(dev, "%s: Runtime PM forbid\n", __func__);
	}
#endif

	rc = iaxxx_get_crashlog_header(priv);
	if (rc) {
		dev_err(priv->dev,
			 "crash header read failed : %d\n", rc);
		goto exit_fw_fail;
	}

	return;

exit_fw_fail:

	/* Send firmware fail uevent to HAL */
	snprintf(action, sizeof(action),
			"ACTION=IAXXX_FW_FAIL_EVENT-%d", priv->dev_id);
	iaxxx_send_uevent(priv, (char *)action);

	/* Clear try counter */
	priv->try_count = 0;
}

static void iaxxx_pre_fw_load_settings(struct iaxxx_priv *priv)
{
	int rc;

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);

	/* Clear event queue */
	if (gpio_is_valid(priv->event_gpio) && priv->is_irq_enabled) {
		disable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = false;
	}

	rc = device_init_wakeup(priv->dev, false);
	if (rc < 0)
		pr_err("dev id-%d: device_init_wakeup disable fail %d",
							priv->dev_id, rc);

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	/* Disable runtime pm*/
	if (pm_runtime_enabled(priv->dev)) {
		if (priv->debug_runtime_pm_disable) {
			pm_runtime_allow(priv->dev);
			dev_info(priv->dev,
				 "%s: Runtime PM Allowed\n", __func__);
		}
		/* Make sure PM usage counter is 0 */
		pm_runtime_put_noidle(priv->dev);
		pm_runtime_disable(priv->dev);
	}
#endif

	mutex_lock(&priv->event_queue_lock);
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;
	mutex_unlock(&priv->event_queue_lock);

}

int iaxxx_fw_reset(struct iaxxx_priv *priv)
{
	/*
	 * If FW crash/ reload/ reset is in progress,
	 * return
	 */
	if (test_bit(IAXXX_FLG_FW_CRASH, &priv->flags) ||
	    test_bit(IAXXX_FLG_FW_RELOAD, &priv->flags) ||
	    test_bit(IAXXX_FLG_FW_RESET, &priv->flags)) {
		pr_err("dev id-%d: can't reset the FW", priv->dev_id);
		return -EBUSY;
	}

	set_bit(IAXXX_FLG_FW_RESET, &priv->flags);

	iaxxx_pre_fw_load_settings(priv);

	regcache_cache_bypass(priv->regmap, true);

	iaxxx_work(priv, runtime_work);

	return 0;
}
EXPORT_SYMBOL(iaxxx_fw_reset);

int iaxxx_fw_reload(struct iaxxx_priv *priv)
{
	if (test_bit(IAXXX_FLG_FW_CRASH, &priv->flags) ||
	    test_bit(IAXXX_FLG_FW_RELOAD, &priv->flags) ||
	    test_bit(IAXXX_FLG_FW_RESET, &priv->flags)) {
		pr_err("dev id-%d: can't reload the FW", priv->dev_id);
		return -EBUSY;
	}

	set_bit(IAXXX_FLG_FW_RELOAD, &priv->flags);

	iaxxx_pre_fw_load_settings(priv);

	regcache_cache_bypass(priv->regmap, true);

	iaxxx_work(priv, runtime_work);

	return 0;
}

static void iaxxx_fw_crash_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_crash_work);
	char action[IAXXX_MAX_UEVENT_STRING_LENGTH];
	int rc;
	uint32_t core_crashed = 0;
	uint32_t mode, status;

	priv->crash_count++;
	dev_info(priv->dev, "%s: iaxxx %d time crashed\n",
			__func__, priv->crash_count);

	dev_info(priv->dev, "%s: Route Status %d\n",
		 __func__, atomic_read(&priv->route_status));

	iaxxx_pre_fw_load_settings(priv);

	/* Read SYSTEM_STATUS to ensure that device is in SRB Mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &status);
	if (rc) {
		dev_err(priv->dev,
			"read SYSTEM_STATUS failed : %d\n", rc);
		goto recover;
	}

	dev_dbg(priv->dev, "iaxxx status reg : 0x%.08X\n", status);

	mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
	if (mode != SYSTEM_STATUS_MODE_SBL && mode != SYSTEM_STATUS_MODE_APPS) {
		dev_err(priv->dev,
			"Not in app mode FW crashed, mode = %d\n", mode);
		if (priv->reset_cb) {
			rc = priv->reset_cb(priv->dev);
			if (rc)
				dev_err(priv->dev, "reset/sync failed : %d\n",
					rc);
		}
		goto recover;
	}

	rc = regmap_read(priv->regmap,
				IAXXX_SRB_PROCESSOR_CRASH_STATUS_ADDR,
				&core_crashed);
	/* Crash status read fails, means DMX core crashed */
	if (rc) {
		dev_err(priv->dev,
			 "%s: Crash status read failed : %d\n",
			 __func__, rc);
	} else {
		if (core_crashed == 1 << IAXXX_HMD_ID)
			dev_err(priv->dev, "ia8x01 HMD core crashed\n");
		else if (core_crashed == 1 << IAXXX_DMX_ID)
			dev_err(priv->dev, "ia8x01 DMX Core Crashed\n");
		else
			dev_err(priv->dev,
				 "ia8x01 Update block failed recovery\n");
	}

recover:
	/* Collect the crash logs */
	mutex_lock(&priv->crashdump_lock);

	iaxxx_dump_crashlogs(priv);

	mutex_unlock(&priv->crashdump_lock);

	/* Notify the user about crash and read crash dump log*/
	snprintf(action, sizeof(action),
			"ACTION=IAXXX_CRASH_EVENT-%d", priv->dev_id);
	iaxxx_send_uevent(priv, (char *)action);

	/* Bypass regmap cache */
	regcache_cache_bypass(priv->regmap, true);
	iaxxx_work(priv, fw_update_work);
}

static void iaxxx_runtime_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, runtime_work);
	unsigned long *flags = &priv->flags;
	bool is_reload_or_crash;

	/*
	 * Suspend due FW crash pending or aborted recovery
	 * or manual FW reload request
	 */
	is_reload_or_crash = test_bit(IAXXX_FLG_FW_CRASH, flags) ||
		test_bit(IAXXX_FLG_FW_RELOAD, flags) ||
		test_bit(IAXXX_FLG_FW_RESET, flags);

	if (is_reload_or_crash)
		iaxxx_do_suspend(priv);
	else
		iaxxx_do_resume(priv);
}

/**
 * iaxxx_fw_update_test_work - work thread for running firmware test
 *
 * @work : used to retrieve Transport Layer private structure
 * Set the regmap to sbl mode and firmware update
 * If firmware update successful, then set bootup_done
 *
 */
static void iaxxx_fw_update_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_fw_update_test_work);
	int rc;

	priv->test_result = false;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);

	if (!rc)
		priv->test_result = true;

	complete_all(&priv->bootup_done);
}

int iaxxx_fw_crash(struct device *dev, enum iaxxx_fw_crash_reasons reasons)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	dev_err(priv->dev, "FW Crash occurred, reasons %d (%s)\n",
		reasons, iaxxx_crash_err2str(reasons));

	if (priv->debug_fw_crash_handling_disable) {
		mutex_lock(&priv->crashdump_lock);
		iaxxx_dump_crashlogs(priv);
		mutex_unlock(&priv->crashdump_lock);

		dev_info(priv->dev, "FW Crash Handling Skipped!\n");
		return 0;
	}

	/* Avoid second times if currently is handled */
	if (test_and_set_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EBUSY;

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);

	priv->fw_crash_reasons = reasons;
	iaxxx_work(priv, runtime_work);
	return 0;
}
EXPORT_SYMBOL(iaxxx_fw_crash);

/*
 * iaxxx_abort_fw_recovery - abort current FW loading works
 *
 * @priv - context structure of driver
 *
 * FW Recovery procedure take more then 4sec.  When entering privacy mode,
 * we block the notifier call chain for the entire suspend time. Current
 * FW load procedure must be aborted. And started again when privacy complete
 *
 */
int iaxxx_abort_fw_recovery(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	/* Not need abort if device is in active state */
	if (pm_runtime_enabled(dev) && pm_runtime_active(dev))
		return -EPERM;

	if (!test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EPERM;

	dev_info(dev, "Aborting FW recovery...\n");

	iaxxx_work_flush(priv, runtime_work);
	iaxxx_work_flush(priv, fw_crash_work);
	iaxxx_work_flush(priv, fw_update_work);
	/* Adapt for MT9615 who doesn't support request_threaded_irq() */
	iaxxx_work_flush(priv, threaded_isr_work);

	dev_info(dev, "FW recovery aborted!\n");

	return 0;
}

/**
 * iaxxx_cmem_test_work - work thread for running memory test
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * Wait for an event interrupt happening with timeout
 *
 */
static void iaxxx_cmem_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_cmem_test_work);
	int rc;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);
}

/**
 * iaxxx_test_init - initialize work items and mutex for test environment
 *
 * @priv	: iaxxx private data
 *
 */
static void iaxxx_test_init(struct iaxxx_priv *priv)
{
	INIT_WORK(&priv->dev_fw_update_test_work, iaxxx_fw_update_test_work);
	INIT_WORK(&priv->dev_cmem_test_work, iaxxx_cmem_test_work);
	mutex_init(&priv->test_mutex);
	init_completion(&priv->bootup_done);
	init_completion(&priv->cmem_done);
}

int get_version_str(struct iaxxx_priv *priv, uint32_t reg, char *verbuf,
							uint32_t len)
{
	struct device *dev = priv->dev;
	int rc;
	uint32_t addr;
	uint32_t i = 0;
	const uint32_t size = len;

	/* Read the version string address */
	rc = regmap_read(priv->regmap, reg, &addr);
	if (rc) {
		dev_err(dev, "%s: String address read failed : %d\n",
							__func__, rc);
		return rc;
	}

	if (!addr) {
		dev_err(dev, "%s: invalid address\n", __func__);
		return -EINVAL;
	}

	dev_dbg(dev, "%s: String address 0x%x\n", __func__, addr);

	/* Read the FW string from address read above */
	while (len > 0) {
		rc = priv->bus_read(dev, addr + i, &verbuf[i], 1);
		if (rc != 1) {
			dev_err(dev, "String Read addr 0x%x failed : %d\n",
							addr + i, rc);
			return -EIO;
		}
		/* Reached NULL character, FW version string ends here */
		if ((!verbuf[i]) || (!verbuf[i + 1]) ||
				(!verbuf[i + 2]) || (!verbuf[i + 3])) {
			dev_dbg(dev, "%s: String ends here\n", __func__);
			i += IAXXX_BYTES_IN_A_WORD;
			break;
		}
		/* 4 characters read, go for next 4 bytes to read */
		len -= IAXXX_BYTES_IN_A_WORD;
		i += IAXXX_BYTES_IN_A_WORD;
	}

	verbuf[size - 1] = '\0';
	print_hex_dump(KERN_INFO, "Version: ", DUMP_PREFIX_OFFSET, 32, 4,
			(void *)verbuf, i, true);
	/*
	 * If not reached end of buffer and buffer is not empty,
	 * then print Firmware version.
	 */
	if (len > 0 && verbuf[0] != '\0')
		return 0;
	return -EIO;
}

int iaxxx_core_suspend_rt(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;

	if (!iaxxx_is_firmware_ready(priv))
		/* return -EBUSY; */
		return 0;

	if (!pm_runtime_enabled(dev) && !(test_bit(IAXXX_FLG_FW_CRASH,
		&priv->flags) || test_bit(IAXXX_FLG_FW_RELOAD, &priv->flags)
		|| test_bit(IAXXX_FLG_FW_RESET, &priv->flags))) {
		dev_warn(dev, "RT suspend requested while PM is not enabled\n");
		return -EINVAL;
	}

	/* Chip suspend/optimal power switch happens here
	 * and they shouldn't be done in case of FW crash/reload/reset
	 */
	if (!(test_bit(IAXXX_FLG_FW_CRASH, &priv->flags) ||
	      test_bit(IAXXX_FLG_FW_RELOAD, &priv->flags)
	      || test_bit(IAXXX_FLG_FW_RESET, &priv->flags)))
		rc = iaxxx_suspend_chip(priv);

	if (rc) {
		dev_err(dev, "%s: Chip suspend failed : %d\n",
			__func__, rc);
		priv->in_suspend = 0;
		return rc;
	}

	return 0;
}

int iaxxx_core_resume_rt(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc;

	if (!iaxxx_is_firmware_ready(priv))
		/*return -EBUSY;*/
		return 0;

	/* Regmap access must be enabled before enable event interrupt */

	rc = iaxxx_wakeup_chip(priv);
	if (rc) {
		dev_err(dev, "%s: Chip wakeup failed : %d\n", __func__, rc);
		priv->in_resume = 0;
		return rc;
	}

	if (!pm_runtime_enabled(dev)) {
		/* this can happen during crash recovery, its not an error
		 * condition
		 */
		dev_warn(dev, "Resume requested while PM is not enabled\n");
	}

	priv->in_resume = 0;

	return 0;
}

int iaxxx_core_dev_suspend(struct device *dev)
{
	int rc;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	dev_info(dev, "%s priv->boot_completed:%d \n", __func__, priv->boot_completed);
	iaxxx_flush_kthread_worker(&priv->worker);

	if (priv->boot_completed) {
		rc = iaxxx_suspend_chip(priv);
		if (rc) {
			dev_err(dev, "%s: Chip suspend failed : %d\n",
				__func__, rc);
			priv->in_suspend = 0;
			return rc;
		}
	}
	atomic_set(&priv->pm_resume, 0);
	/* Will be done in PM.bin */
	/*
	if (priv->boot_completed) {
		gpio_set_value(priv->tvonoff_gpio, 0);
	}
	*/
	return 0;
}

int iaxxx_core_dev_resume(struct device *dev)
{
	int rc;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	unsigned int wakeup_source = MDrv_PM_GetWakeupSource();

	dev_info(dev, "%s wakeup_source=0x%x (%svoice)\n", __func__,
		wakeup_source,
		wakeup_source == PM_WAKEUPSRC_VOICE ? "" : "not ");

	dev_info(dev, "%s priv->boot_completed:%d \n", __func__, priv->boot_completed);

	if (priv->boot_completed) {
		/* Adapt for MT9615 who loses the ISR registration after system resume */
		rc = free_gpio_irq(priv->event_gpio, priv);
		if (rc) {
			dev_err(priv->dev,
				"%s(): free_gpio_irq() fail %d\n",
				__func__, rc);
		}
		rc = request_gpio_irq(priv->event_gpio, iaxxx_event_isr,
						IRQF_TRIGGER_RISING, priv);
		if (rc) {
			dev_err(priv->dev,
				"%s(): request_gpio_irq() fail %d\n",
				__func__, rc);
			return rc;
		}

		priv->is_wifi_wakeup = wakeup_source == PM_WAKEUPSRC_WOC ? true : false;
		if (priv->is_wifi_wakeup == false) {
			if (iaxxx_core_get_route_status(priv))
				gpio_set_value(priv->tvonoff_gpio, 1);
		}
	}
	atomic_set(&priv->pm_resume, 1);

	if (priv->boot_completed) {
		/* Adapt for MT9615 who loses the ISR registration after system resume */
		priv->isr_from_resume = true;
		rc = iaxxx_wakeup_chip(priv);
		if (rc) {
			dev_err(dev, "%s: Chip wakeup failed : %d\n", __func__, rc);
			priv->in_resume = 0;
		} else {
			wake_up(&priv->irq_wake);
			iaxxx_threaded_isr_work(&priv->threaded_isr_work);
		}
	}

	return rc;
}

void iaxxx_pm_complete(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
}

/**
 * iaxxx_device_power_init - init power
 *
 * @priv: iaxxx private data
 */
static int iaxxx_device_power_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	/* Initialize the platform data */
	rc = iaxxx_populate_dt_pdata(priv);
	if (rc) {
		dev_err(dev,
			"initialize platform data failed : %d\n", rc);
		goto err_populate_pdata;
	}

	/* Initialize the GPIOs */
	rc = iaxxx_gpio_init(priv);
	if (rc) {
		dev_err(dev, "initialize GPIOs failed : %d\n", rc);
		goto err_gpio_init;
	}

	/* Initialize interrupts */
	rc = iaxxx_irq_init(priv);
	if (rc) {
		dev_err(dev,
			"initialize interrupts failed : %d\n", rc);
		goto err_irq_init;
	}

	return 0;

err_irq_init:
err_gpio_init:
err_populate_pdata:
	return rc;
}

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_device_reset - called from probe to perform chip reset
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_reset(struct iaxxx_priv *priv)
{
	/* Pull the device out of reset. */
	return iaxxx_reset_to_sbl(priv);
}

int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_register(&priv->core_notifier_list, nb);
	return ret;
}
EXPORT_SYMBOL(iaxxx_fw_notifier_register);

int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_unregister(&priv->core_notifier_list, nb);
	return ret;
}
EXPORT_SYMBOL(iaxxx_fw_notifier_unregister);

int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	return srcu_notifier_call_chain(&priv->core_notifier_list, val, v);
}
EXPORT_SYMBOL(iaxxx_fw_notifier_call);

/**
 * iaxxx_device_init - called from probe to perform device initialization
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_init(struct iaxxx_priv *priv)
{
	int rc;

	priv->iaxxx_state = devm_kzalloc(priv->dev,
			sizeof(struct iaxxx_system_state), GFP_KERNEL);
	if (!priv->iaxxx_state) {
		rc = -ENOMEM;
		return rc;
	}

	priv->crashlog = devm_kzalloc(priv->dev,
			sizeof(struct iaxxx_crashlog), GFP_KERNEL);
	if (!priv->crashlog) {
		rc = -ENOMEM;
		goto err_crashlog_alloc;
	}

	priv->debug_log = devm_kzalloc(priv->dev,
			sizeof(struct iaxxx_debug_log), GFP_KERNEL);
	if (!priv->debug_log) {
		rc = -ENOMEM;
		goto err_debug_log_alloc;
	}

	/* Init mutexes */
	mutex_init(&priv->update_block_lock);
	mutex_init(&priv->event_work_lock);
	mutex_init(&priv->event_queue_lock);
	mutex_init(&priv->plugin_lock);
	mutex_init(&priv->crashdump_lock);
	mutex_init(&priv->pm_mutex);
	mutex_init(&priv->event_lock);

	init_waitqueue_head(&priv->boot_wq);
	init_waitqueue_head(&priv->wakeup_wq);
	init_waitqueue_head(&priv->irq_wake);
	atomic_set(&priv->pm_resume, 1);

	iaxxx_init_kthread_worker(&priv->worker);
	priv->thread = kthread_run(kthread_worker_fn, &priv->worker,
				   "iaxxx-core");
	if (IS_ERR(priv->thread)) {
		dev_err(priv->dev, "Can't create kthread worker: %ld\n",
			PTR_ERR(priv->thread));
		rc = PTR_ERR(priv->thread);
		goto err_kthread_run;
	}

	iaxxx_init_kthread_work(&priv->fw_update_work, iaxxx_fw_update_work);
	iaxxx_init_kthread_work(&priv->fw_crash_work, iaxxx_fw_crash_work);
	iaxxx_init_kthread_work(&priv->runtime_work, iaxxx_runtime_work);
	/* Adapt for MT9615 who doesn't support request_threaded_irq() */
	iaxxx_init_kthread_work(&priv->threaded_isr_work, iaxxx_threaded_isr_work);

	INIT_LIST_HEAD(&priv->iaxxx_state->plugin_head_list);
	INIT_LIST_HEAD(&priv->iaxxx_state->pkg_head_list);
	mutex_init(&priv->iaxxx_state->plg_pkg_list_lock);

	INIT_LIST_HEAD(&priv->iaxxx_state->script_head_list);
	mutex_init(&priv->iaxxx_state->script_lock);
	mutex_init(&priv->iaxxx_state->script_list_lock);

	/* Initialize regmap for SBL */
	rc = iaxxx_regmap_init(priv);
	if (rc)
		goto err_regmap_init;

	/* Initialize the register dump */
	rc = iaxxx_regdump_init(priv);
	if (rc)
		goto err_regdump_init;


	/* Create sysfs */
	rc = iaxxx_init_sysfs(priv);
	if (rc) {
		dev_err(priv->dev,
			"%s: sysfs_create_group failed : %d\n", __func__, rc);
	}

	/* TODO: SYSTEM_ROM_VER_STR */

	/* Initialize test environment */
	iaxxx_test_init(priv);

	/* Initialize the cdev interface */
	rc = iaxxx_cdev_init();
	if (rc) {
		dev_err(priv->dev,
			"initialize cdev interface failed : %d\n", rc);
		goto err_debug_init;
	}

	/* Initialize the debug interface */
	rc = iaxxx_debug_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"initialize debug interface failed : %d\n", rc);
		goto err_debug_init;
	}

	srcu_init_notifier_head(&priv->core_notifier_list);

	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_add_regmap(priv->dev, priv->regmap, &priv->dfs_node);
	if (rc)
		dev_err(priv->dev,
			"%s: create debugfs entry failed : %d\n", __func__, rc);

	rc = iaxxx_event_init(priv);
	if (rc) {
		dev_err(priv->dev, "initialize the event failed : %d\n", rc);
		goto err_event_init;
	}

#ifdef CONFIG_ARCH_HISI
	priv->need_extra_delay_for_rd_wr = true;
#endif

	/*
	 * Make the device power up the chip first so that
	 * the knowles chip won't cause any i2c communication error
	 * for other devices on the same bus
	 */
	rc = iaxxx_device_power_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"power up device failed : %d\n", rc);
		goto err_power_init;
	}

	iaxxx_work(priv, fw_update_work);

	return 0;

err_power_init:
	iaxxx_event_exit(priv);
err_event_init:
err_debug_init:
	iaxxx_regdump_exit(priv);
	mutex_destroy(&priv->test_mutex);
	iaxxx_remove_sysfs(priv);
err_regdump_init:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
		regmap_exit(priv->regmap);
	}
	if (priv->regmap_no_pm)
		regmap_exit(priv->regmap_no_pm);

	mutex_destroy(&priv->update_block_lock);
	mutex_destroy(&priv->event_work_lock);
	mutex_destroy(&priv->event_queue_lock);
	mutex_destroy(&priv->plugin_lock);
	mutex_destroy(&priv->crashdump_lock);
	mutex_destroy(&priv->event_lock);
err_regmap_init:
	kthread_stop(priv->thread);
err_kthread_run:
	devm_kfree(priv->dev, priv->debug_log);
err_debug_log_alloc:
	devm_kfree(priv->dev, priv->crashlog);
err_crashlog_alloc:
	devm_kfree(priv->dev, priv->iaxxx_state);
	return rc;
}

void iaxxx_device_exit(struct iaxxx_priv *priv)
{
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	iaxxx_pm_disable(priv);
#endif
	if (test_and_clear_bit(IAXXX_FLG_STARTUP, &priv->flags))
		mfd_remove_devices(priv->dev);

	/* Delete the work queue */
	flush_work(&priv->event_work_struct);

	iaxxx_flush_kthread_worker(&priv->worker);
	kthread_stop(priv->thread);

	mutex_destroy(&priv->update_block_lock);
	mutex_destroy(&priv->event_work_lock);
	mutex_destroy(&priv->event_queue_lock);
	mutex_destroy(&priv->plugin_lock);
	mutex_destroy(&priv->crashdump_lock);
	mutex_destroy(&priv->pm_mutex);
	mutex_destroy(&priv->iaxxx_state->plg_pkg_list_lock);
	mutex_destroy(&priv->event_lock);
	mutex_destroy(&priv->iaxxx_state->script_lock);
	mutex_destroy(&priv->iaxxx_state->script_list_lock);
	mutex_destroy(&priv->test_mutex);

	iaxxx_remove_sysfs(priv);
	iaxxx_regdump_exit(priv);
	iaxxx_irq_exit(priv);
	iaxxx_debug_exit(priv);
	iaxxx_cdev_exit();
	iaxxx_event_exit(priv);
	iaxxx_dfs_destroy();

	if (priv->regmap_config->ranges) {
		devm_kfree(priv->dev, (void *)priv->regmap_config->ranges);
		priv->regmap_config->ranges = NULL;
	}

	if (priv->regmap_no_pm_config->ranges) {
		devm_kfree(priv->dev,
				(void *)priv->regmap_no_pm_config->ranges);
		priv->regmap_no_pm_config->ranges = NULL;
	}

	if (priv->regmap) {
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}

	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}
	devm_kfree(priv->dev, priv->iaxxx_state);
	kfree(priv->crashlog->log_buffer);
	devm_kfree(priv->dev, priv->crashlog);
	devm_kfree(priv->dev, priv->debug_log);

	if (priv->fw)
		release_firmware(priv->fw);
}
