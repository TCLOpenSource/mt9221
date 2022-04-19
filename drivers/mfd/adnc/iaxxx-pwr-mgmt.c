/*
 * iaxxx-pwr-mgmt.c -- IAxxx Power Management
 *
 * Copyright 2019 Knowles Corporation
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

#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include "iaxxx.h"

#define IAXXX_PROC_STATUS_MASK \
	IAXXX_SRB_PROC_ACTIVE_STATUS_PWR_STATUS_PROC_4_MASK

#define IAXXX_WAKEUP_HOLD_TIME			10000
#define IAXXX_WAKEUP_TIME_RANGE			5

#define IAXXX_POWER_MODE_CHANGE_WAIT_TIME_IN_MS	30

#define IAXXX_PORT_PWR_REG_COUNT	4

#define IAXXX_PM_AUTOSUSPEND_DELAY 3000

#define IAXXX_WAKEUP_MAX_WAIT_TIME_IN_MS	100
#define IAXXX_WAKEUP_WAIT_TIME_IN_MS		20

#define IAXXX_PROC_STATUS_WAIT_MIN		(1000)
#define IAXXX_PROC_STATUS_WAIT_MAX		(2000)

#define IAXXX_PWR_STATE_RETRY			5

#define IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id) \
	(IAXXX_SRB_PROC_PWR_CTRL_PWR_ON_PROC_0_MASK << proc_id)

#define IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id) \
	(IAXXX_SRB_PROC_PWR_CTRL_STALL_PROC_0_MASK << proc_id)

#define IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_CTRL_MEM_PWR_ON_PROC_0_MASK << proc_id)

#define IAXXX_MEM_RETN_ON_OFF_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_CTRL_MEM_RETN_PROC_0_MASK << proc_id)

#define IAXXX_GET_PROC_PWR_STATUS_MASK(proc_id) \
	(IAXXX_SRB_PROC_ACTIVE_STATUS_PWR_STATUS_PROC_0_MASK << proc_id)

#define IAXXX_GET_PER_PROC_MEM_PWR_STATUS_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_STATUS_PROC_0_MEM_PWR_MASK << proc_id)

void iaxxx_pm_enable(struct iaxxx_priv *priv)
{
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	int ret = 0;

	dev_info(priv->dev, "%s: enter\n", __func__);
	priv->in_suspend = 0;
	priv->in_resume = 0;
	ret = pm_runtime_set_active(priv->dev);
	if (ret < 0)
		dev_err(priv->dev, "pm_runtime_set_active failed : %d\n", ret);

	pm_runtime_set_autosuspend_delay(priv->dev, IAXXX_PM_AUTOSUSPEND_DELAY);
	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_use_autosuspend(priv->dev);
	pm_runtime_enable(priv->dev);
#endif
}

void iaxxx_pm_disable(struct iaxxx_priv *priv)
{
	int rc = 0;

	dev_info(priv->dev, "%s: enter\n", __func__);

	pm_runtime_disable(priv->dev);
	pm_runtime_set_suspended(priv->dev);

	rc = device_init_wakeup(priv->dev, false);
	if (rc < 0)
		pr_err("dev id-%d: device_init_wakeup fail %d",
							priv->dev_id, rc);
}

int iaxxx_pm_get_sync(struct device *dev)
{
	int ret = 0;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (priv == NULL) {
		dev_err(dev, "%s: dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev)) {
		dev_err(dev, "%s: Run time PM not enabled\n", __func__);
		return 0;
	}

	mutex_lock(&priv->pm_mutex);
	ret = pm_runtime_get_sync(dev);
	mutex_unlock(&priv->pm_mutex);
	if (ret < 0)
		dev_err(dev, "%s: failed : %d\n", __func__, ret);

#endif
	return ret;
}

int iaxxx_pm_put_autosuspend(struct device *dev)
{
	int ret = 0;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (priv == NULL) {
		dev_err(dev, "%s: dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev)) {
		dev_dbg(dev, "%s: Run time PM not enabled\n", __func__);
		return 0;
	}

	mutex_lock(&priv->pm_mutex);
	pm_runtime_mark_last_busy(dev);
	ret = pm_runtime_put_sync_autosuspend(dev);
	if (ret && ret != -EBUSY)
		dev_err(dev, "%s: failed : %d\n", __func__, ret);
	mutex_unlock(&priv->pm_mutex);

	if (ret == -EBUSY)
		ret = 0;
#endif
	return ret;
}

int iaxxx_config_apll(struct iaxxx_priv *priv,
		       u32 apll_src, u32 apll_out_freq, u32 apll_in_freq)
{
	int ret;
	u32 status;

	if (!apll_src || !apll_in_freq || !apll_out_freq) {
		dev_err(priv->dev, "%s: Invalid Apll Src %d IN freq %d Out freq %d\n",
			__func__, apll_src, apll_in_freq, apll_out_freq);
		return -EINVAL;
	}

	dev_info(priv->dev, "%s: Apll Src %d IN freq %d Out freq %d\n",
		__func__, apll_src, apll_in_freq, apll_out_freq);

	ret = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_SRC_MASK,
		(apll_src - 1) <<
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_SRC_POS);
	if (ret) {
		dev_err(priv->dev,
			"%s: APLL src update failed : %d\n", __func__, ret);
		goto exit;
	}

	ret = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_IN_FREQ_MASK,
		(apll_in_freq - 1) <<
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_IN_FREQ_POS);
	if (ret) {
		dev_err(priv->dev,
			"%s: APLL freq in update failed : %d\n", __func__, ret);
		goto exit;
	}

	ret = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_MASK,
		(apll_out_freq - 1) <<
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_POS);
	if (ret) {
		dev_err(priv->dev,
			"%s: APLL out freq update failed : %d\n",
								__func__, ret);
		goto exit;
	}

	ret = regmap_update_bits(priv->regmap, IAXXX_SRB_SYS_POWER_CTRL_ADDR,
		IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK,
		IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK);
	if (ret) {
		dev_err(priv->dev,
			"%s: Update Power Ctrl update %d\n", __func__, ret);
		goto exit;
	}

	ret = iaxxx_send_update_block_request(priv->dev,
				&status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(priv->dev,
			"%s: power ctrl addr update block failed : %d\n",
								__func__, ret);

exit:
	return ret;
}
EXPORT_SYMBOL(iaxxx_config_apll);

int iaxxx_get_system_clk_src(struct iaxxx_priv *priv)
{
	u32 clk_src;
	u32 clk_data;
	int rc;

	/* Read clock source */
	rc = regmap_read(priv->regmap,
		IAXXX_CNR_SYS_CLK_SRC_SEL_ADDR, &clk_src);
	if (rc) {
		dev_err(priv->dev, "%s: reg 0x%x read failed : %d\n", __func__,
			IAXXX_CNR_SYS_CLK_SRC_SEL_ADDR, rc);
		return rc;
	}

	priv->sys_clk_src = ((clk_src &
		IAXXX_CNR_SYS_CLK_SRC_SEL_MASK_VAL) >> 1);

	if (priv->sys_clk_src) {
		priv->sys_clk_src = PLL_SRC_EXT_CLK;
		/*
		 * TBD: Change it to external clock value as per
		 * the platform is generating
		 */
		priv->sys_clk_in_freq = APLL_SRC_FREQ_19200;
	} else {
		priv->sys_clk_src = PLL_SRC_OSC_CLK;
		priv->sys_clk_in_freq = APLL_SRC_FREQ_6144;
	}

	/* Read system output clock frequency */
	rc = regmap_read(priv->regmap,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR, &clk_data);
	if (rc) {
		dev_err(priv->dev, "%s: reg 0x%x read failed : %d\n", __func__,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR, rc);
		return rc;
	}

	/*
	 * Increment the value by 1 as output frequency index 0 is being
	 * used for None
	 */
	priv->sys_clk_out_freq = ((clk_data &
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_MASK) >>
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_POS) + 1;

	dev_info(priv->dev, "%s: System clock Src %d In freq %d Out freq %d\n",
		 __func__, priv->sys_clk_src, priv->sys_clk_in_freq,
		 priv->sys_clk_out_freq);

	dev_info(priv->dev, "%s: Reg 0x%x value:0x%x Reg 0x%x value:0x%x\n",
		 __func__, IAXXX_CNR_SYS_CLK_SRC_SEL_ADDR, clk_src,
		 IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR, clk_data);

	return rc;
}

static int iaxxx_set_proc_pwr_ctrl(struct iaxxx_priv *priv,
		uint32_t proc_id, uint32_t proc_state, struct regmap *regmap)
{
	uint32_t proc_pwr_ctrl_val = 0;
	uint32_t proc_pwr_ctrl_mask = 0;
	int rc;

	dev_info(priv->dev, "%s: proc_id:%u proc_state:%u\n", __func__,
			proc_id, proc_state);

	switch (proc_state) {
	case PROC_PWR_DOWN:
		proc_pwr_ctrl_mask = IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id);
		proc_pwr_ctrl_val = 0;
	break;
	case PROC_PWR_UP:
		proc_pwr_ctrl_mask = IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id);
		proc_pwr_ctrl_val = IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id);
	break;
	case PROC_STALL_ENABLE:
		proc_pwr_ctrl_mask =
			IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id);
		proc_pwr_ctrl_val =
			IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id);
	break;
	case PROC_STALL_DISABLE:
		proc_pwr_ctrl_mask =
			IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id);
		proc_pwr_ctrl_val = 0;
	break;
	default:
		dev_err(priv->dev, "%s: wrong proc state requested (%d)\n",
					__func__, proc_state);
		rc = -EINVAL;
		goto exit;
	}

	/* set the processor bits */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_PROC_PWR_CTRL_ADDR,
				proc_pwr_ctrl_mask, proc_pwr_ctrl_val);
	if (rc) {
		dev_err(priv->dev, "%s: SRB_PROC_PWR_CTRL_ADDR write err = 0x%0x\n",
					__func__, rc);
		goto exit;
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK);
	if (rc) {
		dev_err(priv->dev, "%s: SRB_SYS_POWER_CTRL_ADDR write err. rc = %d\n",
					__func__, rc);
		goto exit;
	}

	dev_info(priv->dev, "%s: proc_id:%u proc_state:%u transition done\n",
			__func__, proc_id, proc_state);

exit:
	return rc;
}

static int iaxxx_set_mem_pwr_ctrl(struct iaxxx_priv *priv,
		uint32_t proc_id, uint32_t mem_state, struct regmap *regmap)
{
	uint32_t mem_pwr_ctrl_val = 0;
	uint32_t mem_pwr_ctrl_mask = 0;
	int rc;

	dev_info(priv->dev, "%s: proc_id:%u mem_state:%u\n", __func__,
		proc_id, mem_state);

	switch (mem_state) {
	case MEM_PWR_DOWN:
		mem_pwr_ctrl_mask = IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id);
		mem_pwr_ctrl_val = 0;
	break;
	case MEM_PWR_UP:
		mem_pwr_ctrl_mask = IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id);
		mem_pwr_ctrl_val = IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id);
	break;
	case MEM_RETN_ON:
		mem_pwr_ctrl_mask = IAXXX_MEM_RETN_ON_OFF_MASK(proc_id);
		mem_pwr_ctrl_val = IAXXX_MEM_RETN_ON_OFF_MASK(proc_id);
	break;
	case MEM_RETN_OFF:
		mem_pwr_ctrl_mask = IAXXX_MEM_RETN_ON_OFF_MASK(proc_id);
		mem_pwr_ctrl_val = 0;
	break;
	default:
		dev_err(priv->dev, "%s: wrong mem_state state requested (%d)\n",
					__func__, mem_state);
		rc = -EINVAL;
		goto exit;
	}

	/* set the processor bits */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_DED_MEM_PWR_CTRL_ADDR,
				mem_pwr_ctrl_mask, mem_pwr_ctrl_val);
	if (rc) {
		dev_err(priv->dev, "%s: SRB_DED_MEM_PWR_CTRL write err. rc %d\n",
					__func__, rc);
		goto exit;
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK);
	if (rc)
		dev_err(priv->dev, "%s: SRB_SYS_POWER_CTRL_ADDR write err. rc = %d\n",
					__func__, rc);

exit:
	return rc;
}

static int iaxxx_get_proc_pwr_status(struct iaxxx_priv *priv,
	uint32_t proc_id, uint32_t *pwr_status, uint32_t *mem_status)
{
	uint32_t proc_pwr_status = 0;
	uint32_t mem_pwr_status = 0;
	int rc;

	dev_info(priv->dev, "%s: proc id %d\n", __func__, proc_id);

	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR,
							&proc_pwr_status);
	if (rc) {
		dev_err(priv->dev,
			"%s: read proc status fail. rc %d\n", __func__, rc);
		goto exit;
	}

	*pwr_status = proc_pwr_status & IAXXX_GET_PROC_PWR_STATUS_MASK(proc_id);
	dev_info(priv->dev,
		"%s: SRB_PROC_ACTIVE_STATUS_ADDR: 0x%08x, pwr_status: 0x%08x\n",
		__func__, proc_pwr_status, *pwr_status);

	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_DED_MEM_PWR_STATUS_ADDR,
							&mem_pwr_status);
	if (rc) {
		dev_err(priv->dev,
			"%s: SRB_DED_MEM_PWR_STATUS_ADDR read err = 0x%0x\n",
			__func__, rc);
		goto exit;
	}

	*mem_status = mem_pwr_status &
			(IAXXX_GET_PER_PROC_MEM_PWR_STATUS_MASK(proc_id) |
			IAXXX_MEM_RETN_ON_OFF_MASK(proc_id));

	dev_info(priv->dev,
		"%s: SRB_DED_MEM_PWR_STATUS_ADDR: 0x%08x, mem_status: 0x%08x\n",
		__func__, mem_pwr_status, *mem_status);

exit:
	return rc;
}

static int check_proc_power_status(struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t pwr_status = 0;
	uint32_t mem_status = 0;
	int retry_count = IAXXX_PWR_STATE_RETRY;
	int rc = 0;

	dev_info(priv->dev, "%s: proc id %d\n", __func__, proc_id);
	do {
		rc = iaxxx_get_proc_pwr_status(priv, proc_id,
				&pwr_status, &mem_status);
		if (!rc && pwr_status == 0)
			break;
		if (rc || retry_count <= 0) {
			dev_err(priv->dev, "%s: timedout in processor power down, rc %d\n",
				__func__, rc ? rc : -ETIMEDOUT);
			return -ETIMEDOUT;
		}
		usleep_range(IAXXX_PROC_STATUS_WAIT_MIN,
				IAXXX_PROC_STATUS_WAIT_MAX);
	} while (retry_count--);

	return 0;
}

static int iaxxx_proc_power_up_mem_retn_off(
	struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t status = 0;
	int rc = 0;

	rc = iaxxx_set_mem_pwr_ctrl(priv, proc_id, MEM_RETN_OFF, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s: MEM_RETN_OFF fail. rc = %d\n",
			__func__, rc);
		goto exit;
	}

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_PWR_UP, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s: PROC_PWR_UP fail. rc = %d\n", __func__,
			rc);
		goto exit;
	}

	rc = iaxxx_send_update_block_request(priv->dev, &status, IAXXX_BLOCK_0);
	if (rc)
		dev_err(priv->dev,
		"%s: Update blk after proc pwr up fail. Status 0x%x. rc %d\n",
		__func__, status, rc);

exit:
	return rc;
}

static int iaxxx_proc_power_up(
	struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t status = 0, block_id;
	uint32_t mem_status = 0;
	int rc = 0;

	dev_dbg(priv->dev, "%s: proc_id:%u\n", __func__, proc_id);

	/* Check if memory retention is ON for the processor */
	rc = regmap_read(priv->regmap, IAXXX_SRB_DED_MEM_PWR_STATUS_ADDR,
							&mem_status);
	if (rc) {
		dev_err(priv->dev,
			"%s: SRB_DED_MEM_PWR_STATUS_ADDR read err = 0x%0x\n",
			__func__, rc);
		return rc;
	}

	if (mem_status & IAXXX_MEM_RETN_ON_OFF_MASK(proc_id)) {
		rc = iaxxx_proc_power_up_mem_retn_off(priv, proc_id);
		if (rc) {
			dev_err(priv->dev, "%s: proc on and memory retention off fail. rc = %d\n",
				__func__, rc);
			return rc;
		}

		dev_info(priv->dev, "%s: proc(%u) Power up Memory retention off success\n",
			 __func__, proc_id);

		block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);
		priv->retain_mem[block_id] = true;

		return rc;
	}

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_STALL_ENABLE,
			priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s: PROC_STALL_ENABLE fail. rc = %d\n",
					__func__, rc);
		return rc;
	}
	dev_info(priv->dev,
		"%s: proc(%u) stall enable Requested\n", __func__, proc_id);

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_PWR_UP, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s: PROC_PWR_UP fail. rc = %d\n",
					__func__, rc);
		return rc;
	}
	dev_info(priv->dev,
		"%s: proc(%u) power up Requested\n", __func__, proc_id);

	rc = iaxxx_set_mem_pwr_ctrl(priv, proc_id, MEM_PWR_UP, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s MEM_PWR_UP fail. rc = %d\n",
					__func__, rc);
		return rc;
	}
	dev_info(priv->dev,
		"%s: proc(%u) memory power up Requested\n", __func__, proc_id);

	rc = iaxxx_send_update_block_request(priv->dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		dev_err(priv->dev, "%s(): Update blk before download fail. Status 0x%x. rc %d\n",
				__func__, status, rc);
		return rc;
	}

	rc = iaxxx_boot_proc(priv, proc_id);
	if (rc) {
		dev_err(priv->dev, "bootup processor (%d) fail :%d\n", proc_id,
			rc);
		return rc;
	}

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id,
				PROC_STALL_DISABLE, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s PROC_STALL_DIS fail. rc = %d\n",
			__func__, rc);
		return rc;
	}
	dev_info(priv->dev, "%s: proc(%u) stall disable Requested\n",
							__func__, proc_id);

	rc = iaxxx_send_update_block_request(priv->dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		dev_err(priv->dev,
		"%s: Update blk after download failed. Status 0x%x. rc %d\n",
		__func__, status, rc);
		return rc;
	}

	dev_info(priv->dev, "%s: proc(%u) power up success\n", __func__,
		 proc_id);

	return rc;
}

static int iaxxx_proc_power_down(struct iaxxx_priv *priv,
					 uint32_t proc_id)
{
	uint32_t status = 0;
	uint32_t block_id;
	int rc = 0;

	dev_info(priv->dev, "%s: proc_id:%u\n", __func__, proc_id);

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_PWR_DOWN,
							priv->regmap_no_pm);
	if (rc) {
		dev_err(priv->dev, "%s: PROC_PWR_DOWN fail. rc = %d\n",
			__func__, rc);
		goto exit;
	}

	dev_info(priv->dev,
		"%s: proc(%u) power down Requested\n", __func__, proc_id);

	rc = iaxxx_send_update_block_request_with_options(
		priv->dev, IAXXX_BLOCK_0,
		priv->regmap_no_pm,
		0,
		UPDATE_BLOCK_NO_LOCK, &status);
	if (rc) {
		dev_err(priv->dev,
		"%s: Update blk after proc pwr down fail. Status 0x%x. rc %d\n",
		__func__, status, rc);
		goto exit;
	}

	rc = check_proc_power_status(priv, proc_id);
	if (rc) {
		dev_err(priv->dev, "%s: check_proc_power_status fail. rc = %d\n",
					__func__, rc);
		goto exit;
	}
	dev_info(priv->dev, "%s: proc(%u) power down Success\n",
					__func__, proc_id);

	if (priv->retain_mem[block_id]) {
		rc = iaxxx_set_mem_pwr_ctrl(priv, proc_id, MEM_RETN_ON,
					    priv->regmap_no_pm);
		if (rc) {
			dev_err(priv->dev, "%s: MEM_RETN_ON fail. rc = %d\n",
				__func__, rc);
			goto exit;
		}
		dev_info(priv->dev, "%s: proc(%u) memory Retention ON Requested\n",
			 __func__, proc_id);
	} else {
		rc = iaxxx_set_mem_pwr_ctrl(priv, proc_id, MEM_PWR_DOWN,
					    priv->regmap_no_pm);
		if (rc) {
			dev_err(priv->dev, "%s: MEM_PWR_DOWN fail. rc = %d\n",
				__func__, rc);
			goto exit;
		}
		dev_info(priv->dev, "%s: proc(%u) memory power down Requested\n",
			 __func__, proc_id);
	}

	rc = iaxxx_send_update_block_request_with_options(priv->dev,
		IAXXX_BLOCK_0, priv->regmap_no_pm, 0, UPDATE_BLOCK_NO_LOCK,
		&status);
	if (rc) {
		dev_err(priv->dev,
		"%s: Update blk after mem pwr down fail. Status 0x%x. rc %d\n",
		__func__, status, rc);
		goto exit;
	}

	dev_info(priv->dev, "%s: proc(%u) power down success\n", __func__,
		 proc_id);
exit:
	return rc;
}

int iaxxx_check_and_powerup_proc(struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t proc_status;
	int rc = 0;

	dev_info(priv->dev, "%s: proc id %d\n", __func__, proc_id);

	if ((proc_id != IAXXX_HMD_ID) && (proc_id != IAXXX_DMX_ID)) {
		dev_err(priv->dev, "%s: Invalid proc id %d\n",
						__func__, proc_id);
		return -EINVAL;
	}

	if (!iaxxx_is_firmware_ready(priv)) {
		dev_err(priv->dev, "%s: FW not ready\n", __func__);
		return -EIO;
	}

	/* Read core processor status */
	rc = regmap_read(priv->regmap,
			IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR, &proc_status);
	if (rc) {
		dev_err(priv->dev,
			"%s: read proc status fail. rc %d\n", __func__, rc);
		goto exit;
	}

	proc_status &= IAXXX_GET_PROC_PWR_STATUS_MASK(proc_id);

	if (!proc_status)
		rc = iaxxx_proc_power_up(priv, proc_id);

	if (rc)
		dev_err(priv->dev, "%s: Failed to power up processor %d\n",
				__func__, proc_id);

exit:
	return rc;
}

static int iaxxx_check_and_powerdown_procs(struct iaxxx_priv *priv,
		uint32_t *proc_status)
{
	int rc;
	uint32_t proc_active;
	bool status= true;

	/* Read core processor status */
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR, &proc_active);
	if (rc) {
		dev_err(priv->dev, "%s: read proc status Fail. rc %d\n",
			__func__, rc);
		return rc;
	}

	/* Check if HMD processor is active */
	proc_active = proc_active & IAXXX_PROC_STATUS_MASK;
	if (proc_active) {
		dev_info(priv->dev, "%s: Proc status 0x%x\n",
							__func__, proc_active);
	} else {
		*proc_status = 0;
		return 0;
	}

	rc = iaxxx_check_proc_pkg_plg_list_empty(priv, IAXXX_HMD_ID, &status);
	if (rc) {
		dev_err(priv->dev, "package plugin load check fail\n");
		return rc;
	}

	if (!status) {
		pr_info("dev id-%d: proc(%u) package/plugin is active, can't be powered down",
			priv->dev_id, IAXXX_HMD_ID);
		*proc_status = proc_active;
		return 0;
	}

	rc = iaxxx_proc_power_down(priv, IAXXX_HMD_ID);
	if (rc && (rc != -EAGAIN)) {
		dev_err(priv->dev, "%s: power down HMD Fail. rc %d\n",
			__func__, rc);
		return rc;
	}

	if (rc == -EAGAIN) {
		*proc_status = proc_active;
		return 0;
	}

	proc_active &= ~(IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_HMD_ID));

	*proc_status = proc_active;
	return rc;
}

int iaxxx_suspend_chip(struct iaxxx_priv *priv)
{
	int rc;
	u32 status;
	u32 intf_mx_speed;
	u32 proc_status = -1;

	pr_info("dev id-%d: called", priv->dev_id);

	rc = iaxxx_check_and_powerdown_procs(priv, &proc_status);
	if (rc) {
		dev_err(priv->dev, "%s: power check on processors fail. rc %d\n",
			__func__, rc);
		goto recovery;
	}

	dev_info(priv->dev, "%s: Proc status 0x%x\n", __func__, proc_status);

	/*
	 * SLEEP MODE: If route is inactive and Processor status for HMD
	 * is inactive
	 */
	if (!iaxxx_core_get_route_status(priv) && !proc_status) {
		/* Issue sleep power mode command */
		rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK,
				(IAXXX_SLEEP_MODE <<
				IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_POS));
		if (rc) {
			dev_err(priv->dev, "%s: Sleep mode update failed : %d\n",
				__func__, rc);
			goto recovery;
		}

		/* Update block lock is not taken for no_pm calls
		 * because those can trigger PM wakeup which will
		 * try to do some fw-setup which will need the
		 * update block.
		 */
		rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				priv->regmap_no_pm,
				IAXXX_POWER_MODE_CHANGE_WAIT_TIME_IN_MS,
				UPDATE_BLOCK_FIXED_WAIT |
				UPDATE_BLOCK_NO_LOCK,
				&status);
		if (rc) {
			dev_err(priv->dev,
				"%s: Sleep pwr mode update blk failed : %d\n",
				__func__, rc);
			goto recovery;
		}

		atomic_set(&priv->iaxxx_state->power_state, IAXXX_SLEEP_MODE);
		dev_info(priv->dev, "%s: Power mode: SLEEP\n", __func__);
	} else {
		if (priv->update_intf_max_speed != 0)
			intf_mx_speed = priv->update_intf_max_speed;
		else
			intf_mx_speed = priv->intf_max_speed;

		/* set up the MAX interface speed thats expected when the
		 * system is wakeup. Set the Interface Speed to maximum so
		 * system will be awake with max clock speed.
		 */
		rc = regmap_write(priv->regmap_no_pm, priv->intf_speed_addr,
					intf_mx_speed);
		if (rc) {
			dev_err(priv->dev,
				"%s: set Max Interface speed failed : %d\n",
				__func__, rc);
			goto recovery;
		}

		/* Issue Optimal power mode command */
		rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK,
			(IAXXX_OPTIMAL_MODE <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS));
		if (rc) {
			dev_err(priv->dev, "%s: Power mode update failed : %d\n",
				__func__, rc);
			goto recovery;
		}

		dev_info(priv->dev,
			"%s: disable control interface\n", __func__);

		rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				priv->regmap_no_pm,
				IAXXX_POWER_MODE_CHANGE_WAIT_TIME_IN_MS,
				UPDATE_BLOCK_FIXED_WAIT |
				UPDATE_BLOCK_NO_LOCK,
				&status);
		if (rc) {
			dev_err(priv->dev,
				"%s: Opt pwr mode update block failed : %d\n",
				__func__, rc);
			goto recovery;
		}

		atomic_set(&priv->iaxxx_state->power_state,
			   IAXXX_OPTIMAL_MODE);
		atomic_set(&priv->iaxxx_state->disable_ctrl_intf, true);

		dev_info(priv->dev, "%s: Power mode: OPTIMAL, Disable Ctrl Interface\n",
			 __func__);
	}

	test_and_clear_bit(IAXXX_FLG_CHIP_WAKEUP, &priv->flags);
	dev_info(priv->dev, "%s: Success\n", __func__);

	return rc;

recovery:
	iaxxx_fw_crash(priv->dev, IAXXX_FW_CRASH_SUSPEND);
	dev_err(priv->dev, "%s: failed : %d\n", __func__, rc);
	return rc;
}
EXPORT_SYMBOL(iaxxx_suspend_chip);

static int iaxxx_switch_optimal_to_normal(struct iaxxx_priv *priv)
{
	u32 status;
	u32 reg_val;
	int rc = 0;

	/*
	 * If power mode is Optimal then control interface should be enabled.
	 * If control interface is disabled then mode switching shouldn't
	 * happen. Optimal with disable control interface mode during this call
	 * is possible if Runtime PM is disabled and kcontrol is used to put
	 * the chip into Optimal with disable control interface mode and
	 * didn't wakeup the chip.
	 */
	if ((atomic_read(&priv->iaxxx_state->power_state) ==
	     IAXXX_OPTIMAL_MODE) &&
	    !(atomic_read(&priv->iaxxx_state->disable_ctrl_intf))) {
		rc = regmap_read(priv->regmap_no_pm,
			       IAXXX_SRB_SYS_POWER_CTRL_ADDR, &reg_val);
		if (rc) {
			dev_err(priv->dev, "%s: Pwr Ctrl reg Read fail %d\n",
								__func__, rc);
			goto exit;
		}

		dev_dbg(priv->dev, "%s: Pwr Ctrl reg Read : 0x%08x\n",
				__func__, reg_val);

		if (reg_val &
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK)
			dev_err(priv->dev,
				"%s: Optimal bit set (0x%08x) after wakeup\n",
				__func__, reg_val);

		rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			(IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK |
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK),
			(IAXXX_NORMAL_MODE <<
			IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_POS));
		if (rc) {
			dev_err(priv->dev, "%s: Normal mode update failed : %d\n",
				__func__, rc);
			goto exit;
		}

		/*
		 * No Update block lock and status check. Lock should not be
		 * there if wakeup is called. Because if due to update block
		 * request, wakeup sequence is started then wakeup should not
		 * be on hold due to lock is acquired
		 */
		rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				priv->regmap_no_pm,
				IAXXX_POWER_MODE_CHANGE_WAIT_TIME_IN_MS,
				UPDATE_BLOCK_FIXED_WAIT |
				UPDATE_BLOCK_NO_LOCK,
				&status);
		if (rc) {
			dev_err(priv->dev,
				"%s: Normal pwr mode update blk failed : %d\n",
				__func__, rc);
			goto exit;
		}

		atomic_set(&priv->iaxxx_state->power_state, IAXXX_NORMAL_MODE);
		dev_info(priv->dev, "%s: Power mode: NORMAL\n", __func__);
	}

exit:
	return rc;
}

int iaxxx_wakeup_chip(struct iaxxx_priv *priv)
{
	int rc = 0;
	u32 reg_val;
	long wake_timeout = HZ;

	pr_info("dev id-%d: called", priv->dev_id);

	/* If the chip is already woken up, skip wakeup*/
	if (!test_bit(IAXXX_FLG_CHIP_WAKEUP, &priv->flags)) {
		/* Toggle wakeup pin if defined or do the read to
		 * Toggle SPI CS line to wakeup
		 * the chip
		 */
		if (gpio_is_valid(priv->wakeup_gpio) && !priv->wakeup_from_cs) {
			pr_info("dev id-%d: Wakeup GPIO toggle", priv->dev_id);
			gpio_set_value(priv->wakeup_gpio, 0);
			usleep_range(IAXXX_WAKEUP_HOLD_TIME,
				     IAXXX_WAKEUP_HOLD_TIME +
				     IAXXX_WAKEUP_TIME_RANGE);
			gpio_set_value(priv->wakeup_gpio, 1);
		} else {

			pr_info("dev id-%d: do dummy read to wake chip",
								priv->dev_id);

			rc = regmap_read(priv->regmap_no_pm,
					 IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);
			if (rc) {
				dev_err(priv->dev, "FW status read failed : %d\n",
					rc);
				return rc;
			}

		       pr_debug("dev id-%d: FW status : 0x%08x",
							priv->dev_id, reg_val);

			if (reg_val == SYSTEM_STATUS_MODE_APPS)
				goto chip_woken_up;
		}

		if (atomic_read(&priv->iaxxx_state->power_state)
		    == IAXXX_SLEEP_MODE)
			wake_timeout = HZ / 10;

	       pr_info("dev id-%d: waiting for wakeup event", priv->dev_id);

		rc = wait_event_timeout(priv->wakeup_wq,
					test_bit(IAXXX_FLG_CHIP_WAKEUP,
						 &priv->flags), wake_timeout);

		if (!test_bit(IAXXX_FLG_CHIP_WAKEUP,
			      &priv->flags) && rc == 0) {
			dev_err(priv->dev,
				"Timeout for wakeup event rc :%d wake flag :%d\n",
				rc, test_bit(IAXXX_FLG_CHIP_WAKEUP,
					     &priv->flags));
			/*
			 * If timeout happen, read the FW status, if it is in
			 * APP mode chip is woken up
			 */
			rc = regmap_read(priv->regmap_no_pm,
					 IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);
			if (rc) {
				dev_err(priv->dev, "FW status read failed : %d\n",
					rc);
				return rc;
			}

			if (reg_val == SYSTEM_STATUS_MODE_APPS)
				goto chip_woken_up;

			rc = -ETIMEDOUT;
		} else {
			rc = 0;
			goto chip_woken_up;
		}

		dev_err(priv->dev,
			"%s: chip wake up failed : %d, reg_val 0x%x\n",
			__func__, rc, reg_val);
		goto recovery;
	}

chip_woken_up:
	if (atomic_read(&priv->iaxxx_state->power_state)
					== IAXXX_OPTIMAL_MODE) {
		atomic_set(&priv->iaxxx_state->disable_ctrl_intf, false);
		dev_info(priv->dev, "%s: Power mode: OPTIMAL, Enable Control Interface\n",
			 __func__);
	} else {
		atomic_set(&priv->iaxxx_state->power_state, IAXXX_NORMAL_MODE);
		dev_info(priv->dev, "%s: Power mode: NORMAL\n", __func__);
	}

	/* If optimal to normal mode transition is not disabled */
	if (!priv->disable_optimal_to_normal) {
		rc = iaxxx_switch_optimal_to_normal(priv);
		if (rc)
			goto recovery;
	}

	return rc;

recovery:
	iaxxx_fw_crash(priv->dev, IAXXX_FW_CRASH_RESUME);
	dev_err(priv->dev, "%s: failed : %d\n", __func__, rc);
	return rc;
}
EXPORT_SYMBOL(iaxxx_wakeup_chip);

int iaxxx_enable_optimal_to_normal_transition(struct iaxxx_priv *priv)
{
	int rc;

	/*
	 * Need to call pm sync so that during optimal to normal mode
	 * transition, suspend won't be called
	 */
	rc = iaxxx_pm_get_sync(priv->dev);
	if (rc < 0)
		return rc;

	/*
	 * If chip is in Optimal mode after wakeup before modifying the route
	 * then its required to change the power mode to Normal
	 */
	rc = iaxxx_switch_optimal_to_normal(priv);

	iaxxx_pm_put_autosuspend(priv->dev);

	if (rc) {
		pr_err("dev id-%d: Optimal to normal mode change fail %d",
							priv->dev_id, rc);
		goto recovery;
	}

	return rc;

recovery:
	iaxxx_fw_crash(priv->dev, IAXXX_FW_CRASH_RESUME);
	dev_err(priv->dev, "%s: failed : %d\n", __func__, rc);
	return rc;
}
EXPORT_SYMBOL(iaxxx_enable_optimal_to_normal_transition);
