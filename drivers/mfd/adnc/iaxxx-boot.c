/*
 * iaxxx-boot.c -- IAxxx boot firmware to application mode
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

#include <linux/delay.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"
#include <linux/mfd/adnc/iaxxx-core.h>

#define CHUNK_SIZE (32768/8)

#define IAXXX_DWNLD_SECTION_RETRY	5

static bool iaxxx_sect_valid_proc_mask(
			uint32_t sect_addr, uint32_t proc_mask)
{
	if (((sect_addr >= GLOBAL_DRAM_START) &&
		(sect_addr <= GLOBAL_DRAM_END) &&
		(proc_mask & GLBL_MEM_ID_MASK)) ||
		((sect_addr >= HMD_DMEM_SYS_START) &&
		(sect_addr <= HMD_IMEM_SYS_END) &&
		(proc_mask & HMD_ID_MASK)) ||
		((sect_addr >= DMX_DMEM_SYS_START) &&
		(sect_addr <= DMX_IMEM_SYS_END) &&
		(proc_mask & DMX_ID_MASK)))
		return true;
	else
		return false;
}

void iaxxx_copy_le32_to_cpu(void *dst, const void *src, size_t nbytes)
{
	int i;
	int num_words = nbytes / sizeof(uint32_t);
	uint32_t *p_dst = dst;
	const uint32_t *p_src = src;

	if (WARN_ON(!src) || WARN_ON(!dst) || WARN_ON(nbytes & 0x3))
		return;

	for (i = 0; i < num_words; ++i, ++p_dst, ++p_src)
		*p_dst = le32_to_cpu(*p_src);
}

/**
 * iaxxx_download_section - downloads a firmware text or data section
 *
 * @priv    : iaxxx private data
 * @data    : the section data to be downloaded
 * @section : pointer to the section data (section address, length, etc).
 */
int iaxxx_download_section(struct iaxxx_priv *priv, const uint8_t *data,
				const struct firmware_section_header *section,
				struct regmap *regmap)
{
	int rc, i = 0;
	struct device *dev = priv->dev;
	uint32_t iaxxx_chunk_size = (priv->bus == IAXXX_I2C)
				? CHUNK_SIZE/64 : CHUNK_SIZE;
	int rem_bytes = section->length % (iaxxx_chunk_size);
	int temp_len = section->length / (iaxxx_chunk_size);
	int chunk_word_size = iaxxx_chunk_size * 4;


	dev_info(dev, "Writing section at 0x%.08X, %d words(s)\n",
				section->start_address, section->length);

	/* Write the section data directly to the device memory */
	for (i = 0; i < temp_len; i++) {

		rc = regmap_bulk_write(regmap,
			(section->start_address + ((i * chunk_word_size))),
			data + (i * chunk_word_size), iaxxx_chunk_size);
		if (rc) {
			dev_err(dev, "%s: regmap_write failed : %d\n",
						__func__, rc);
			return rc;
		}
	}
	if (rem_bytes) {
		rc = regmap_bulk_write(regmap,
			(section->start_address + ((i * chunk_word_size))),
			data + (i * chunk_word_size), rem_bytes);
		if (rc) {
			dev_err(dev, "%s: regmap_write failed : %d\n",
						__func__, rc);
			return rc;
		}
	}

	return 0;
}

int iaxxx_verify_fw_header(struct device *dev,
			struct firmware_file_header *header)
{
	const uint32_t SIG1 = 0x49445541;	/* "IDUA */
	const uint32_t SIG2 = 0x45434E45;	/* "ECNE */

	/* Verify the file signature */
	if (header->signature[0] != SIG1 || header->signature[1] != SIG2) {
		dev_err(dev,
		"Bad fw file signature sig0 0x%x, sig1 0x%x\n",
					header->signature[0],
					header->signature[1]);
		return -EINVAL;
	}

	/* The file must contain at least 1 section */
	if (!header->number_of_sections) {
		dev_err(dev, "Missing sections in firmware file\n");
		return -EINVAL;
	}

	return 0;
}

int
iaxxx_download_firmware(struct iaxxx_priv *priv, const struct firmware *fw)
{
	int i, rc = 0;
	int retries = 0;
	const int max_retries = IAXXX_DWNLD_SECTION_RETRY;
	const uint8_t *data;
	struct device *dev = priv->dev;

	/* Checksum variable */
	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;

	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };

	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad Firmware image file (too small)\n");
		goto out;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad firmware image file %d\n", rc);
		goto out;
	}

	/* Include file header fields as part of the checksum */
	CALC_FLETCHER16(header.number_of_sections, sum1, sum2);
	CALC_FLETCHER16(header.entry_point, sum1, sum2);

	if (priv->bus_open) {
		rc = priv->bus_open(priv);
		if (rc) {
			pr_err("dev id-%d: block open failed : %d",
							priv->dev_id, rc);
			goto out;
		}
	}

	/*
	 * The last section has the binary checksum. So a valid binary
	 * should have atleast one more section
	 */
	if (header.number_of_sections <= 1) {
		dev_err(dev, "No sections available to download\n");
		rc = -EINVAL;
		goto out;
	}

	/* Download each memory section */
	for (i = 0; i < header.number_of_sections; ++i) {
		/* Load the next data section */
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);

		if (file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);

			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);

			do {
				/* Use no_pm regmap since this is during booting
				 * and access to default regmap is disallowed
				 */
				rc = iaxxx_download_section(
						priv, data, &file_section,
						priv->regmap_no_pm);

			} while (rc && ++retries < max_retries);

			if (rc) {
				dev_err(dev, "load fw section failed : %d\n",
									rc);
				goto bus_close;
			}

			/* Include checksum for this section
			 * Use no_pm regmap for this because this is called
			 * during booting and there is no access to default
			 * regmap.
			 */
			rc = iaxxx_checksum_request(priv,
					file_section.start_address,
					file_section.length, &sum1, &sum2,
					priv->regmap_no_pm);
			if (rc) {
				dev_err(dev, "Checksum request failed : %d\n",
									rc);
				goto bus_close;
			}

			dev_info(dev, "%s: section(%d) Checksum 0x%04X%04x\n",
				__func__, i, sum2, sum1);

			/* Next section */
			data += file_section_bytes;
			WARN_ON((data - fw->data) > fw->size);
		}
	}


	/* If the last section length is 0, then verify the checksum */
	if (file_section.length == 0) {
		uint32_t checksum = (sum2 << 16) | sum1;

		dev_info(dev, "final checksum from chip = 0x%.08X\n", checksum);

		if (checksum != file_section.start_address) {
			rc = -EINVAL;
			dev_err(dev, "%s: Checksum mismatch 0x%.08X != 0x%.08X\n",
				__func__, checksum, file_section.start_address);
		}
	}

bus_close:
	if (priv->bus_close)
		priv->bus_close(priv);
out:
	return rc;
}

static int iaxxx_download_per_core_fw(struct iaxxx_priv *priv,
		const struct firmware *fw, u32 proc_id)
{
	int i, rc = 0;
	int retries = 0;
	const int max_retries = IAXXX_DWNLD_SECTION_RETRY;
	const uint8_t *data;
	struct device *dev = priv->dev;
	u32 proc_id_mask = 1 << proc_id;

	/* Checksum variable */
	uint32_t devicesum1 = 0xffff;
	uint32_t devicesum2 = 0xffff;

	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };

	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad Firmware image file (too small)\n");
		goto out;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad firmware image file\n");
		goto out;
	}

	if (header.number_of_sections <= 1) {
		dev_err(dev, "No sections available to download\n");
		rc = -EINVAL;
		goto out;
	}

	/* Download each memory section */
	for (i = 0; i < header.number_of_sections; ++i) {
		/* Load the next data section */
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);

		dev_dbg(dev, "file_section.start_address: 0x%.08X\n",
				file_section.start_address);
		if (iaxxx_sect_valid_proc_mask(file_section.start_address,
			proc_id_mask) && file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);

			do {
				rc = iaxxx_download_section(priv, data,
					&file_section, priv->regmap);
			} while (rc && ++retries < max_retries);

			if (rc) {
				dev_err(dev, "Failed to load firmware section\n");
				goto out;
			}

			/* Include checksum for this section */
			rc = iaxxx_checksum_request(priv,
				file_section.start_address, file_section.length,
				&devicesum1, &devicesum2, priv->regmap);
			if (rc) {
				dev_err(dev, "Checksum request error\n");
				goto out;
			}
			dev_dbg(dev, "%s: section(%d) Checksum 0x%04X%04x\n",
				__func__, i, devicesum2, devicesum1);
		}
		/* Next section */
		data += file_section.length * sizeof(u32);
		WARN_ON((data - fw->data) > fw->size);
	}

	dev_info(dev, "Proc (%d) download success\n", proc_id);
out:
	return rc;
}

int iaxxx_boot_proc(struct iaxxx_priv *priv, u32 proc_id)
{
	struct device *dev = priv->dev;
	int rc;

	if (!priv->fw) {
		/* Request the firmware image */
		rc = request_firmware(&priv->fw, priv->fw_name, dev);
		if (rc) {
			dev_err(dev, "%s: Firmware file %s not found rc = %d\n",
				__func__, priv->fw_name, rc);
			return rc;
		}
	}

	/* Download the firmware to device memory */
	rc = iaxxx_download_per_core_fw(priv, priv->fw, proc_id);
	if (rc) {
		dev_err(dev, "core(%d) firmware download failed, rc = %d\n",
			proc_id, rc);
		goto out;
	}

	/* core is up and running */
	dev_info(dev, "Proc (%d) Firmware is loaded\n", proc_id);

out:
	return rc;
}

/**
 * iaxxx_wait_apps_ready - Wait for firmware application mode to be ready
 *
 * @priv: iaxxx private data
 *
 * Returns 0 on success or -ETIMEDOUT if the device wasn't ready within the
 * expected time.
 */
static int iaxxx_wait_apps_ready(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int i, rc = 0;
	unsigned int count;
	uint32_t status;
	uint8_t mode;

	const unsigned int IAXXX_APPS_MODE_POLL_MSEC = 20;	/* 20 ms */
	const unsigned int IAXXX_APPS_MODE_WAIT_MSEC = 200;	/* 200 ms */

	count = IAXXX_APPS_MODE_WAIT_MSEC / IAXXX_APPS_MODE_POLL_MSEC;

	/* wait for fw boot to app mode success event */
	rc = wait_event_timeout(priv->boot_wq, priv->boot_completed, HZ);
	if (!priv->boot_completed && rc == 0)
		dev_err(priv->dev,
			"Timedout in wait for boot completion event, try polling\n");
	else
		return 0;

	/* Apps mode is expected to be ready within 50 msecs */
	for (i = 0; i < count; ++i) {
		/*
		 * There are chances that while expecting the boot complete
		 * event, chip may crash and hence regmap_init will fail, there
		 * by clearing the regmap params. In such case, ->regmap usage
		 * will lead to crash.
		 * Make sure regmap is present before using it.
		 *
		 * regmap check is done here, because the chip might provide a
		 * delayed event after the above wait event timeout which again
		 * might clear the regmap params.
		 */
		if (!priv->regmap) {
			dev_err(dev, "%s: regmap is not populated. err out\n",
								__func__);
			rc = -EINVAL;
			return rc;
		}

		/* In absence of events, poll SYSTEM_STATUS for App Mode */
		rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_STATUS_ADDR,
				&status);
		if (rc) {
			dev_err(dev,
				"%s: read SYSTEM_STATUS failed : %d\n",
								__func__, rc);
			return rc;
		}

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(dev, "System status 0x%.08x, mode = %d\n",
						status, mode);

		if (mode == SYSTEM_STATUS_MODE_APPS)
			return 0;

		msleep(IAXXX_APPS_MODE_POLL_MSEC);
	}

	return -ETIMEDOUT;
}

/**
 * iaxxx_bootup - Boots the chip hardware
 *
 * @priv: iaxxx private data
 *
 * Downloads any necessary hardware configuration and RAM patches.
 */
int iaxxx_bootup(struct iaxxx_priv *priv)
{
	const struct firmware *fw;
	int rc;
	struct device *dev = priv->dev;
	const struct firmware_file_header *fw_header;
#ifdef DEBUG
	uint32_t reg;
#endif

	/* Request the firmware image */
	rc = request_firmware(&fw, priv->fw_name, dev);
	if (rc) {
		dev_err(dev, "%s: Firmware file %s not found : %d\n",
					__func__, priv->fw_name, rc);
		return rc;
	}
	/* Download the firmware to device memory */
	rc = iaxxx_download_firmware(priv, fw);
	if (rc) {
		dev_err(dev, "%s: Firmware download failed : %d\n",
								__func__, rc);
		goto out;
	} else {
		dev_info(dev, "%s: Firmware download success\n", __func__);
	}

	fw_header = (const struct firmware_file_header *)fw->data;

	usleep_range(1000, 1010);

#ifdef DEBUG
	/* Get and log the Device ID */
	rc = regmap_read(priv->regmapi_no_pm,
			 IAXXX_SRB_SYS_DEVICE_ID_ADDR, &reg);
	if (rc)
		dev_err(dev, "%s: regmap_read failed : %d\n", __func__, rc);

	dev_dbg(dev, "Device ID before jump to ram: 0x%.08X\n", reg);

	usleep_range(1000, 1010);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap_no_pm,
			 IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, &reg);
	if (rc)
		dev_err(dev, "regmap_read failed : %d\n", rc);

	dev_dbg(dev, "ROM Version before jump to ram: 0x%.08X\n", reg);
#endif

	dev_info(dev, "Requesting Jump to App Mode (Addr : 0x%08x)\n",
						fw_header->entry_point);

	priv->jump_to_app_mode = false;

	/* Enable the irq to receive boot complete event */
	if (gpio_is_valid(priv->event_gpio) && !priv->is_irq_enabled) {
		enable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = true;
	}

	/* Boot device into application mode */
	rc = iaxxx_jump_to_request(priv, le32_to_cpu(fw_header->entry_point));
	if (rc) {
		dev_err(dev, "boot firmware failed : %d\n", rc);
		goto out;
	}

	dev_info(dev, "Wait for the application mode to be up and running\n");
	/* Wait for the application mode to be up and running */
	rc = iaxxx_wait_apps_ready(priv);
	if (rc) {
		dev_err(dev, "Timed out waiting for apps mode : %d\n", rc);
		goto out;
	}

	dev_info(dev, "Confirm switch to App mode\n");

	/* Call SPI speed setup callback if exists */
	if (priv->intf_speed_setup)
		priv->intf_speed_setup(dev, priv->app_speed);

	/*
	 * device running in application mode
	 */
	dev_info(dev, "Firmware running in application mode\n");

out:
	release_firmware(fw);
	return rc;
}
