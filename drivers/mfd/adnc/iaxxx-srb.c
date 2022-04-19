/*
 * iaxxx-srb.c -- IAxxx System Register Block Requests
 *
 * Copyright 2016 Knowles Corporation
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
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include "iaxxx.h"

#define IAXXX_READ_DELAY        10	/* 10 us delay before read */
#define IAXXX_READ_DELAY_RANGE  10	/* 10 us range */

/* No of times to retry in case of update block register
 * does not return expected value.
 */
#define UPDATE_BLOCK_FAIL_RETRIES 15
#define UPDATE_BLOCK_WRITE_CHECK_RETRIES 5

#define IAXXX_UPDATE_BLOCK_WITH_FIXED_WAIT	(true)
#define IAXXX_UPDATE_BLOCK_WITH_NORMAL_WAIT	(false)

#define IAXXX_UPDATE_BLOCK_NO_PM      (true)
#define IAXXX_UPDATE_BLOCK_WITH_PM    (false)

/* 1ms delay before read */
#define IAXXX_UPDATE_BLK_READ_DELAY		3000
#define IAXXX_UPDATE_BLK_READ_DELAY_RANGE	5
/* 2ms delay before read retry */
#define IAXXX_UPDATE_BLK_READ_RETRY_DELAY	2000

#define IAXXX_WAIT_FOR_UPDATE_BLOCK_FINISH ((UPDATE_BLOCK_FAIL_RETRIES * \
				IAXXX_UPDATE_BLK_READ_RETRY_DELAY) / 1000)

/**
 * iaxxx_regmap_wait_match - waits for register read value
 * to match the given value. This is a dual test to check
 * if necessary bits are cleared and necessary bits bit
 * still set. This is to fix cases when invalid 0 value
 * read from spi would be treated as update block
 * success.
 *
 * @priv: iaxxx private data
 * @regmap: regmap to use
 * @reg : the register to be monitored
 * @match: the value to be matched
 * @status: status of update block
 *
 * Returns 0 on success or -ETIMEDOUT if the bits didn't clear.
 */
static int
iaxxx_regmap_wait_match(struct iaxxx_priv *priv, struct regmap *regmap,
		uint32_t reg, uint32_t err_reg_addr, uint32_t *status)
{
	int rc, retries;
	const int max_retries = UPDATE_BLOCK_FAIL_RETRIES;
	struct device *dev = priv->dev;

	/* 1ms of delay after write is required as per FW */
	usleep_range(IAXXX_UPDATE_BLK_READ_DELAY, IAXXX_UPDATE_BLK_READ_DELAY +
		     IAXXX_UPDATE_BLK_READ_DELAY_RANGE);

	for (retries = 0; retries < max_retries; ++retries) {
		rc = regmap_read(regmap, reg, status);
		if (rc) {
			/* We should not return here until we are done
			 * with our retries
			 */
			dev_err(dev, "regmap_read failed : %d\n", rc);

			/*
			 * if read error is EIO that means data transfer over
			 * Host interface is not working, return err
			 */
			if (rc == -EIO)
				return rc;
		}

		/* Read value matches */
		if (*status == 0) {
			rc = regmap_read(regmap, err_reg_addr,
					 status);
			if (rc) {
				dev_err(dev, "reg 0x%x read failed : %d\n",
					err_reg_addr, rc);
				return rc;
			}

			if (*status & IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE_MASK) {
				/*
				 * If the BLOCK_UPDATE_RESULT is non-zero, then
				 * some update failed. The caller(s) will need
				 * to determine how to handle the failure.
				 */
				dev_err(dev, "%s: status 0x%x\n", __func__,
					*status);
				return -ENXIO;
			}

			return 0;
		}

		/*
		 * Ideally 1ms of delay before read is enough, but if bit is not
		 * cleared yet wait for 2ms before retry
		 */
		usleep_range(IAXXX_UPDATE_BLK_READ_RETRY_DELAY,
			IAXXX_UPDATE_BLK_READ_RETRY_DELAY +
			IAXXX_UPDATE_BLK_READ_DELAY_RANGE);
	}

	dev_err(dev, "%s: Update block timed out\n", __func__);
	return -ETIMEDOUT;
}

static inline int
iaxxx_regmap_clear_bits(struct iaxxx_priv *priv, uint32_t reg, uint32_t mask)
{
	return regmap_update_bits(priv->regmap, reg, mask, 0);
}

static int iaxxx_read_error_register(struct iaxxx_priv *priv,
		struct regmap *regmap)
{
	int rc;
	int i;

	for (i = 0; i < MAX_CORES; i++) {
		rc = regmap_read(regmap,
			IAXXX_PLUGIN_HDR_ERROR_BLOCK_ADDR(i),
			&priv->iaxxx_state->err.error[i]);
		if (rc) {
			dev_err(priv->dev,
				"Read Error Block %d addr failed : %d\n",
									i, rc);
			goto out;
		}
		rc = regmap_read(regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_ADDR(i),
			&priv->iaxxx_state->err.plg_inst_id[i]);
		if (rc) {
			dev_err(priv->dev,
			"Read Inst id Block %d addr failed : %d\n", i, rc);
			goto out;
		}
	}

	for (i = 0 ; i < MAX_CORES; i++)
		dev_info(priv->dev, "Block id %d : Err 0x%x Inst id 0x%x\n", i,
		priv->iaxxx_state->err.error[i],
		priv->iaxxx_state->err.plg_inst_id[i] == PLUGIN_INST_NONE ? 0
			: priv->iaxxx_state->err.plg_inst_id[i]);
out:
	return rc;
}

/**
 * iaxxx_update_block_request - sends Update Block Request & waits for response
 *
 * @priv     : iaxxx private data
 * @block_id : block id of the processor
 * @regmap    : if main regmap or no_pm regmap
 * @wait_time_in_ms: wait time to use if fixed_wait is true.
 * @options  : options
 * @status   : pointer for the returned system status
 *
 * Core lock must be held by the caller.
 *
 * All requests are expected to finish within a few milliseconds and therefore
 * polling is used rather than using event interrupts to signal completion.
 */

static int iaxxx_update_block_request(struct iaxxx_priv *priv,
		int  block_id,
		struct regmap *regmap,
		uint32_t wait_time_in_ms,
		enum update_block_options_t options,
		uint32_t *status)
{
	int rc;
	int ret;
	struct device *dev = priv->dev;
	uint32_t sys_blk_update_addr;
	uint32_t err_reg_addr;
	int retry_cnt = UPDATE_BLOCK_WRITE_CHECK_RETRIES;
	bool retry_status_check = true;

	/* Check status is not NULL, its an address */
	if (!status)
		return -EINVAL;

	dev_dbg(dev, "%s: block_id:%d\n", __func__, block_id);

	/* Choose update block register address based on block id */
	switch (block_id) {
	case IAXXX_BLOCK_0:
		sys_blk_update_addr = IAXXX_PCTRL_REG_TRIG_DMX_INTR_ADDR;
		err_reg_addr = IAXXX_SRB_SYS_BLK_UPDATE_ADDR;
		break;

	case IAXXX_BLOCK_1:
		sys_blk_update_addr = IAXXX_PCTRL_REG_TRIG_HMD_INTR_ADDR;
		err_reg_addr = IAXXX_SRB_SYS_BLK_UPDATE_1_ADDR;
		break;

	default:
		dev_err(dev, "%s: Invalid block_id:%d\n", __func__, block_id);
		return -EINVAL;
	}

	/* This check ensure that pm_resume is executed in normal use case
	 * scenarios except from the suspend path as regmap_no_pm is used in
	 * those functions.  This ensure that update_block_lock is not contended
	 * by both resume and suspend paths simultaneously
	 */
	if (regmap ==  priv->regmap) {
		rc = iaxxx_pm_get_sync(priv->dev);
		if (rc < 0) {
			dev_err(priv->dev, "%s: pm get sync failed, rc 0x%x\n",
					__func__, rc);
			return rc;
		}
	}

	/* To protect concurrent update blocks requests.
	 *
	 * If the option to not use update block locking
	 * don't lock the mutex.
	 */
	if (!(options & UPDATE_BLOCK_NO_LOCK))
		mutex_lock(&priv->update_block_lock);

status_read:
	/* Check if previous update block is in progress */
	rc = regmap_read(regmap, sys_blk_update_addr, status);
	if (rc) {
		dev_err(dev, "%s: update block register read failed : %d\n",
			__func__, rc);
		goto out;
	}

	/* If update block is in progress, wait for sometime and re-check */
	if (*status != 0) {
		dev_info(dev, "%s: Update block is in progress\n", __func__);
		msleep(IAXXX_WAIT_FOR_UPDATE_BLOCK_FINISH);

		if (iaxxx_is_firmware_ready(priv) && retry_status_check) {
			retry_status_check = false;
			goto status_read;
		}

		dev_err(dev, "%s: Update block fail\n", __func__);
		rc = -EBUSY;
		goto out;
	}

retry_update_block_reg_write:
	/*
	 * Set the UPDATE_BLOCK_REQUEST bit in the SRB_UPDATE_CTRL register
	 */
	rc = regmap_write(regmap, sys_blk_update_addr,
			       IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "set UPDATE_BLOCK_REQUEST bit failed : %d\n", rc);
		goto out;
	}

	/* If fixed wait is needed after update block wait and return */
	if (options & UPDATE_BLOCK_FIXED_WAIT) {
		if (wait_time_in_ms)
			msleep(wait_time_in_ms);
		goto out;
	}

	/*
	 * Note that in application mode, events can be used instead of polling.
	 * Make a wait for event like task that either polls or waits for an
	 * incoming event and wakes us up when the block update has completed.
	 */

	/* Poll waiting for the UPDATE_BLOCK_REQUEST bit to clear */
	rc = iaxxx_regmap_wait_match(priv, regmap, sys_blk_update_addr,
			err_reg_addr, status);
	if (rc) {
		/* retry if update bit is still not clear (-ETIMEDOUT)*/
		if (rc == -ETIMEDOUT && retry_cnt--)
			goto retry_update_block_reg_write;

		dev_err(dev, "Update Block req poll failed : %d\n", rc);

		goto out;
	}

out:
	/* We should not acquire lock during pm operations
	 * otherwise it leads to dead lock
	 */
	if (!(options & UPDATE_BLOCK_NO_LOCK))
		mutex_unlock(&priv->update_block_lock);
	if (rc) {
		ret = iaxxx_read_error_register(priv, regmap);
		if (ret)
			dev_err(dev, "Read error register failed : %d\n", ret);
	}

	if (regmap ==  priv->regmap)
		iaxxx_pm_put_autosuspend(priv->dev);

	return rc;
}

static int iaxxx_check_update_block_err_and_recover(struct device *dev,
		int rc, uint32_t status)
{
	uint32_t err = 0;

	if (rc == -ETIMEDOUT) {
		dev_err(dev, "%s:Update block timed out\n", __func__);
	} else if (rc == -ENXIO) {
		err = status & IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE_MASK;
		if (err == SYSRC_FAIL) {
			dev_err(dev, "FW SYSRC_FAIL:0x%x\n", status);
		} else if (err == SYSRC_ERR_MEM) {
			dev_err(dev, "FW NOMEM err:0x%x\n", status);
		} else if (err) {
			dev_err(dev, "FW general err:0x%x\n", status);
			return -EINVAL;
		}
	} else {
		return rc;
	}

	iaxxx_fw_crash(dev, IAXXX_FW_CRASH_UPDATE_BLOCK_REQ);
	return rc;
}

/*===========================================================================
 * Bootloader
 *===========================================================================
 */

static int iaxxx_bootloader_request(struct iaxxx_priv *priv, u32 mask, u32 val)
{
	int rc;
	struct device *dev = priv->dev;
	uint32_t reg_addr = IAXXX_PCTRL_REG_TRIG_DMX_INTR_ADDR;

	/* Set the request bit in BOOTLOADER_REQUEST register */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_BOOT_REQ_ADDR, mask, val);
	if (rc) {
		dev_err(dev, "set BOOTLOADER_REQUEST failed : %d\n", rc);
		goto out;
	}


	priv->jump_to_app_mode = true;

	/*
	 * Set the UPDATE_BLOCK_REQUEST bit in the SRB_UPDATE_CTRL register
	 * to trigger BOOTLOADER request. Don't check update block
	 * status to avoid spi-reads that could return invalid data.
	 *
	 */
	rc = regmap_write(priv->regmap_no_pm,
				reg_addr,
				IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc)
		dev_err(dev, "set UPDATE_BLOCK_REQUEST bit failed : %d\n", rc);

out:
	return rc;
}

int iaxxx_check_bootloader_status(struct iaxxx_priv *priv, u32 mask,
				    bool *status)
{
	int rc;
	u32 val;
	struct device *dev = priv->dev;

	if (!priv->regmap_no_pm) {
		dev_err(dev, "%s: regmap_no_pm is not populated. err out\n",
							__func__);
		rc = -EINVAL;
		return rc;
	}

	/* The bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_SRB_BOOT_REQ_ADDR, &val);
	if (rc) {
		dev_err(dev, "read BOOTLOADR_REQUEST failed : %d\n", rc);
		return rc;
	}

	if (val & mask) {
		/* Clear the request bit */
		regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_BOOT_REQ_ADDR, mask, 0);
		*status = false;
	} else {
		*status = true;
	}

	return rc;

}

int iaxxx_jump_to_request(struct iaxxx_priv *priv, uint32_t address)
{
	int rc;
	struct device *dev = priv->dev;

	/* Program BOOTLOADER_JUMP_ADDRESS with the start address */
	rc = regmap_write(priv->regmap_no_pm,
			  IAXXX_SRB_BOOT_JUMP_ADDR_ADDR, address);
	if (rc) {
		dev_err(dev, "set JUMP_ADDRESS failed : %d\n", rc);
		goto out;
	}

	/* Send the request */
	rc = iaxxx_bootloader_request(priv,
				IAXXX_SRB_BOOT_REQ_JUMP_REQ_MASK,
				0x1 << IAXXX_SRB_BOOT_REQ_JUMP_REQ_POS);
	if (rc) {
		dev_err(dev, "JUMP_REQUEST failed : %d\n", rc);
		goto out;
	}

out:
	return rc;
}

int iaxxx_calibrate_oscillator_request(struct iaxxx_priv *priv, uint32_t delay)
{
	return -ENOTSUPP;
}

/*===========================================================================
 * Checksum
 *===========================================================================
 */
int iaxxx_checksum_request(struct iaxxx_priv *priv, uint32_t address,
			uint32_t length, uint32_t *sum1, uint32_t *sum2,
			struct regmap *regmap)
{
	int rc, ret;
	uint32_t status;
	uint32_t request = 0;
	struct device *dev = priv->dev;

	if (!sum1 || !sum2)
		return -EINVAL;

	/* Set the CHECKSUM_VALUE */
	rc = regmap_write(regmap, IAXXX_SRB_CHKSUM_VAL1_ADDR, *sum1);
	if (rc) {
		dev_err(dev, "clear CHECKSUM_VALUE1 failed : %d\n", rc);
		goto out;
	}

	rc = regmap_write(regmap, IAXXX_SRB_CHKSUM_VAL2_ADDR, *sum2);
	if (rc) {
		dev_err(dev, "clear CHECKSUM_VALUE2 failed : %d\n", rc);
		goto out;
	}

	/* Program the CHECKSUM_ADDRESS with first memory address */
	rc = regmap_write(regmap,
			  IAXXX_SRB_CHKSUM_BUFFER_ADDR_ADDR, address);
	if (rc) {
		dev_err(dev, "set CHECKSUM_ADDRESS failed : %d\n", rc);
		goto out;
	}

	/* Program the CHECKSUM_LENGTH with the buffer length (in bytes) */
	rc = regmap_write(regmap,
			  IAXXX_SRB_CHKSUM_LENGTH_ADDR, length);
	if (rc) {
		dev_err(dev, "set CHECKSUM_LENGTH failed : %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Set the CHECKSUM_REQUEST bit in CHECKSUM_REQUEST */
	rc = regmap_update_bits(regmap, IAXXX_SRB_CHKSUM_ADDR,
			       IAXXX_SRB_CHKSUM_SECTION_REQ_MASK,
			       0x1 << IAXXX_SRB_CHKSUM_SECTION_REQ_POS);
	if (rc) {
		dev_err(dev, "set CHECKSUM_REQUEST failed : %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, IAXXX_BLOCK_0,
			regmap, 0, UPDATE_BLOCK_NO_OPTIONS, &status);
	if (rc) {
		dev_err(dev, "CHECKSUM_REQUEST failed : %d\n", rc);

		/* Do the recovery for update block failure except if this
		 * function is called for loading Firmware during booting
		 */
		if (regmap == priv->regmap) {
			ret = iaxxx_check_update_block_err_and_recover(
					dev, rc, status);
			dev_info(dev, "Error in Update block err and recover: %d\n",
				 ret);
		}

		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* The CHECKSUM_REQUEST bit should have been cleared by firmware */
	rc = regmap_read(regmap, IAXXX_SRB_CHKSUM_ADDR,
			       &request);
	if (rc) {
		dev_err(dev, "read CHECKSUM_REQUEST failed : %d\n", rc);
		goto out;
	}

	if (request & IAXXX_SRB_CHKSUM_SECTION_REQ_MASK)
		goto out;

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Read the checksum from CHECKSUM_VALUE */
	rc = regmap_read(regmap, IAXXX_SRB_CHKSUM_VAL1_ADDR, sum1);
	if (rc) {
		dev_err(dev, "read CHECKSUM_VALUE1 failed : %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	rc = regmap_read(regmap, IAXXX_SRB_CHKSUM_VAL2_ADDR, sum2);
	if (rc) {
		dev_err(dev, "read CHECKSUM_VALUE2 failed : %d\n", rc);
		goto out;
	}

out:
	return rc;
}

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_send_update_block_request - sends Update Block Request to the device
 *
 * @dev    : MFD core device pointer
 * @status : pointer for the returned system status
 * @id     : Block ID/ Proc ID
 *
 * This call will be blocked until either the device responds to the request
 * or a timeout occurs while waiting for the response.
 */
int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
						int id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;

	if (!priv)
		return -EINVAL;

	rc = iaxxx_update_block_request(priv, id,
			priv->regmap, 0, UPDATE_BLOCK_NO_OPTIONS,
			status);

	rc = iaxxx_check_update_block_err_and_recover(dev, rc, *status);

	return rc;
}
EXPORT_SYMBOL(iaxxx_send_update_block_request);

/**
 * iaxxx_send_update_block_request_with_options -
 * sends update Block Request to the device based on options.
 *
 * @dev    : MFD core device pointer
 * @block_id : Block ID/ Proc ID
 * @regmap  : regmap to use
 * @wait_time_in_ms: wait time to use if fixed_wait flag is set
 * @options : options:
 *            UPDATE_BLOCK_FIXED_WAIT - This option enables fixed wait
 *            time specified by wait_time_in_ms parameter. This delay would
 *            execute instead of polling for status after sending update block
 *            to firmware.
 *
 *            UPDATE_BLOCK_NO_LOCK - This option disables mutex lock
 *            during whole update block operation. This option is used
 *            from power management context which can trigger another
 *	      update block.
 *
 * @status: status returned on error.
 *
 */
int iaxxx_send_update_block_request_with_options(struct device *dev,
					int block_id,
					struct regmap *regmap,
					uint32_t wait_time_in_ms,
					enum update_block_options_t options,
					uint32_t *status)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;

	if (!priv)
		return -EINVAL;

	rc = iaxxx_update_block_request(priv, block_id,
				regmap, wait_time_in_ms, options, status);

	rc = iaxxx_check_update_block_err_and_recover(dev, rc, *status);

	return rc;
}
EXPORT_SYMBOL(iaxxx_send_update_block_request_with_options);
