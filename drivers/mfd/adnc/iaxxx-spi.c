/*
 * iaxxx-spi.c -- SPI driver for Knowles IAxxx device
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include "iaxxx.h"
#include <linux/of.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include "mdrv_gpio.h"  /* Adapt for MT9615 */

/* spi large reads are failing and currently
 * our code request max read as 8k bytes
 * which translates to 4096 words
 */
#define IAXXX_SPI_PACKET_LEN	4096
#define IAXXX_REG_LEN		4
/* Padding is required to give time FW ready for data */
#define IAXXX_REG_PADDING	12
#define IAXXX_REG_LEN_WITH_PADDING (IAXXX_REG_LEN + IAXXX_REG_PADDING)

/* Burst size used when sending raw data to firmware */
#define IAXXX_SPI_BURST_SIZE  (32*1024)

#define SET_SBL_SPI_SPEED_CMD	0x801D0003

static DECLARE_BITMAP(dev_id_map, MAX_IAXXX_DEVICES);
static DEFINE_MUTEX(device_reg_order_lock);

/**
 * Description of driver private data
 *
 * @priv: IAxxx private data
 * @spi:  spi client pointer
 */
struct iaxxx_spi_priv {
	struct iaxxx_priv priv;	/* private data */
	struct spi_device *spi;
};

static u32 iaxxx_get_spi_speed(struct device *dev)
{
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;

	if (!spi) {
		pr_info("SPI pointer is NULL");
		return 0;
	}

	return spi->max_speed_hz;
}

static inline struct iaxxx_spi_priv *to_spi_priv(struct iaxxx_priv *priv)
{
	return priv ? container_of(priv, struct iaxxx_spi_priv, priv) : NULL;
}

/* Register address passed to this function is in BE order.
 * The array of data in valaddr can be in BE or CPU order based
 * on the flag passed.
 */
static void register_dump_log(struct device *dev,
			const uint32_t regaddr, const uint32_t *valaddr,
			const bool data_in_be32_fmt,
			const uint32_t no_words, u8 op)
{
	int i;
	uint32_t regaddr_cpuorder = be32_to_cpu(regaddr);

	for (i = 0; i < no_words; i++)
		register_transac_log(dev,
			regaddr_cpuorder + (sizeof(uint32_t) * i),
			data_in_be32_fmt ? be32_to_cpu(valaddr[i]) :
			valaddr[i], op);
}

/* send command to firmware and read response */
static int iaxxx_spi_cmd(struct spi_device *spi, u32 cmd, u32 *resp)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(&spi->dev);
	int ret;

	if (!spi) {
		pr_err("NULL input pointer(s)");
		return -EINVAL;
	}

	ret = iaxxx_pm_get_sync(&spi->dev);
	if (ret < 0)
		return ret;

	dev_dbg(&spi->dev, "iaxxx: cmd = 0x%08x\n", cmd);
	cmd = cpu_to_be32(cmd);

	ret = spi_write(spi, &cmd, sizeof(cmd));
	if (ret) {
		dev_err(&spi->dev, "send cmd 0x%.08X failed : %d\n", cmd, ret);
		return ret;
	}

	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

	if (resp) {
		usleep_range(4000, 4500);
		ret = spi_read(spi, resp, sizeof(*resp));
		if (ret) {
			dev_err(&spi->dev, "read cmd response failed : %d\n",
									ret);
			return ret;
		}
		*resp = be32_to_cpu(*resp);
	}

	iaxxx_pm_put_autosuspend(&spi->dev);

	return 0;
}

static int iaxxx_copy_cpu_to_be32(struct device *dev,
				void *data_buff, uint32_t len)
{
	int i;
	uint32_t *buff = data_buff;
	uint32_t align_len = len;

	if (len % 4) {
		align_len = len + (4 - (len % 4));
		dev_warn(dev, "Not aligned 32bit boundary (%d). aligned len to %d\n",
				len, align_len);
		len -= (len % 4);
	}

	for (i = 0; i < len/sizeof(uint32_t); i++) {
		u32 data = buff[i];

		buff[i] = cpu_to_be32(data);
	}

	return align_len;
}

/* This is a common function used by all public spi write functions.
 *
 * This function sends data over SPI to firmware with the option to
 * convert the data and addr to be32.
 * If the option says the data and addr are NOT in be32 Fmt then
 * data and addr would be converted before writing.
 *
 */
static int iaxxx_spi_write_common(void *context,
				const void *reg, const size_t reg_len,
				const void *val, const size_t val_len,
				const bool data_in_be32_fmt,
				bool pm_needed)
{
	int rc;
	size_t reg_len_new;
	size_t val_len_new = val_len;
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct spi_message m;
	struct spi_transfer t[1] = { };
	uint8_t *padding;

	uint32_t reg_addr = (*(uint32_t *)reg);

	pr_debug("dev id-%d: Register address 0x%x", priv->dev_id, reg_addr);

	padding = iaxxx_kvmalloc((val_len + IAXXX_REG_LEN_WITH_PADDING),
				 __GFP_ZERO);
	if (!padding)
		return -ENOMEM;

	if (data_in_be32_fmt)
		reg_addr = be32_to_cpu(reg_addr);

	/* Device protocol requires address to be shifted by one bit */
	reg_addr = cpu_to_be32(reg_addr >> 1);

	spi_message_init(&m);

	reg_len_new = (reg_len > IAXXX_REG_LEN) ? IAXXX_REG_LEN_WITH_PADDING :
			IAXXX_REG_LEN;

	memcpy(padding, &reg_addr, IAXXX_REG_LEN);
	memcpy(padding + reg_len_new, val, val_len);
	if (!data_in_be32_fmt)
		val_len_new = iaxxx_copy_cpu_to_be32(dev, padding +
				reg_len_new, val_len);
	/* Register address */
	t[0].len = reg_len_new + val_len_new;
	t[0].tx_buf = padding;
	spi_message_add_tail(&t[0], &m);

	if (pm_needed) {
		rc = iaxxx_pm_get_sync(&spi->dev);
		if (rc < 0)
			goto pm_sync_err;
	}

	rc = spi_sync(spi, &m);
	if (rc)
		pr_err("%s spi_sync() failed %d\n", __func__, rc);

	if (pm_needed)
		iaxxx_pm_put_autosuspend(&spi->dev);

pm_sync_err:
	if (padding != NULL)
		kvfree(padding);

	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

	return rc;

}

/* This spi-write function is regmap interface function. It is called
 * from regmap which is configured to send data in Big-Endian.
 * Since SPI communication with firmware is Big-Endian, there is no
 * conversion needed before sending data.
 */
static int iaxxx_regmap_spi_gather_write(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	/* flag set to indicate data and reg-address are in be32 format */
	return iaxxx_spi_write_common(context,
				 reg, reg_len,
				 val, val_len,
				 true, true);
}

/* This version of spi-write is called by regmap. Since regmap is configured
 * for Big-endian, the data received here will be in Big-endian format.
 * Since SPI communication with Firmware is Big-endian, no byte-order
 * conversion is needed before sending data.
 */
static int iaxxx_regmap_spi_write(void *context, const void *data, size_t count)
{
	const void *val = data + sizeof(uint32_t);
	size_t reg_len = sizeof(uint32_t);
	size_t val_len = count - reg_len;
	struct device *dev = context;
	uint32_t *writebuf = (uint32_t *)((uint8_t *)val + IAXXX_REG_PADDING);
	uint32_t reg_addr = *(uint32_t *)data;
	uint32_t words = (val_len - IAXXX_REG_PADDING) / sizeof(uint32_t);
	int rc;

	if (WARN_ON(count <= sizeof(uint32_t))
		|| WARN_ON(val_len <= IAXXX_REG_PADDING)) {
		pr_err("Error Input param put of range");
		return -EINVAL;
	}

	/* flag set to indicate data and reg-address are in be32 format */
	rc = iaxxx_spi_write_common(context, data, reg_len,
			val, val_len, true, true);

	register_dump_log(dev, reg_addr, writebuf, true, words, IAXXX_WRITE);
	return rc;
}

/* This version of spi-write is used privately across the driver to write
 * directly to firmware's memory. Since data received here will be in
 * CPU byte order, the data would need to be converted to Big-endian
 * before sending to firmware over SPI.
 *
 * Also this function splits big buffers into burst size defined by
 * IAXXX_SPI_BURST_SIZE and sends to firmware due to limitations
 * in some SPI master drivers handling big buffers size.
 *
 */
static int iaxxx_spi_bus_write(struct device *dev,
				uint32_t reg,
				const void *val, size_t words)
{
	size_t reg_len = IAXXX_REG_LEN_WITH_PADDING;
	size_t val_index;
	size_t val_len = words * 4;
	size_t burst_size = val_len > IAXXX_SPI_BURST_SIZE ?
			IAXXX_SPI_BURST_SIZE : val_len;
	uint8_t *val_addr = (uint8_t *) val;
	int rc = 0;

	for (val_index = 0; val_index < val_len; val_index += burst_size) {
		/* Flag is false to indicate data and reg-address are
		 * not in be32 (in CPU byte order)
		 */
		rc = iaxxx_spi_write_common(dev,
			&reg, reg_len,
			val_addr + val_index,
			(val_len - val_index) < burst_size ?
			(val_len - val_index) :
			burst_size, false, true);
		if (rc)
			break;
		reg += burst_size;
	}

	return rc;
}

/* This is the common function used by all public spi read functions.
 * This function assumes that the register address in be32 format.
 * The data read in the buffer will be in be32 format. It will be up to
 * the caller to convert the data if needed to CPU byte order.
 */
#if 0  /* Knowles original */
static int iaxxx_spi_read_common(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len, bool pm_needed)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct spi_message m;
	struct spi_transfer t[1] = { };
	uint32_t reg2[4] = {0};
	uint8_t *rx_buf;
	uint8_t *tx_buf;
	int rc = 0;

	uint32_t reg_addr = be32_to_cpu(*(uint32_t *)reg);
	size_t msg_len = max(sizeof(reg2),
			     val_len + IAXXX_REG_LEN_WITH_PADDING);

	tx_buf = kzalloc(msg_len, GFP_DMA | GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	rx_buf = kzalloc(msg_len, GFP_DMA | GFP_KERNEL);
	if (!rx_buf) {
		rc = -ENOMEM;
		goto mem_alloc_fail;
	}

	/* For reads, most significant bit is set after address shifted */
	reg_addr = cpu_to_be32(reg_addr >> 1) | 0x80;
	reg2[0] = reg_addr;
	memcpy(tx_buf, reg2, sizeof(reg2));

	spi_message_init(&m);

	if (pm_needed) {
		rc = iaxxx_pm_get_sync(&spi->dev);
		if (rc < 0)
			goto pm_sync_err;
	}

	/* Register address */
	t[0].len = msg_len;
	t[0].tx_buf = tx_buf;
	t[0].rx_buf = rx_buf;
	spi_message_add_tail(&t[0], &m);

	rc = spi_sync(spi, &m);
	if (rc)
		goto spi_sync_err;

	memcpy(val, rx_buf + IAXXX_REG_LEN_WITH_PADDING, val_len);

	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

spi_sync_err:
	if (pm_needed)
		iaxxx_pm_put_autosuspend(&spi->dev);
pm_sync_err:
	kfree(rx_buf);
mem_alloc_fail:
	kfree(tx_buf);

	return rc;
}
#else  /* Adapt for MT9615 who doesn't support full-duplex communication in one spi_transfer. */
static int iaxxx_spi_read_common(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len, bool pm_needed)
{
	int i;
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct spi_message m;
	struct spi_transfer t[2] = { };
	uint32_t reg2[4] = {0};
	uint8_t *buf = NULL;
	int rc = 0;

	uint32_t reg_addr = be32_to_cpu(*(uint32_t *)reg);
	size_t msg_len = max(sizeof(reg2),
			     val_len + IAXXX_REG_LEN_WITH_PADDING);

	buf = kzalloc(msg_len, GFP_DMA | GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* For reads, most significant bit is set after address shifted */
	reg_addr = cpu_to_be32(reg_addr >> 1) | 0x80;
	reg2[0] = reg_addr;
	memcpy(buf, reg2, sizeof(reg2));

	spi_message_init(&m);

	if (pm_needed) {
		rc = iaxxx_pm_get_sync(&spi->dev);
		if (rc < 0)
			goto pm_sync_err;
	}

	memset(t, 0, sizeof(t));
	t[0].len = IAXXX_REG_LEN_WITH_PADDING;
	t[0].tx_buf = buf;
	t[1].len = val_len;
	t[1].rx_buf = buf + IAXXX_REG_LEN_WITH_PADDING;
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	rc = spi_sync(spi, &m);
	if (rc)
		goto spi_sync_err;

	memcpy(val, buf + IAXXX_REG_LEN_WITH_PADDING, val_len);

	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

spi_sync_err:
	if (pm_needed)
		iaxxx_pm_put_autosuspend(&spi->dev);
pm_sync_err:
	kfree(buf);

	return rc;
}
#endif

/* This version of spi-read is called by regmap. Since SPI communication
 * with firmware is in Big-endian and regmap is also configured for
 * Big-endian, the data read here does not need to be altered.
 * Also the register address received here is also in Big-endian format.
 */
static int iaxxx_regmap_spi_read(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len)
{
	struct device *dev = context;
	uint32_t words = val_len / sizeof(uint32_t);
	int rc = 0;
	uint32_t reg_addr = *(uint32_t *) reg;

	rc = iaxxx_spi_read_common(context, reg, reg_len, val, val_len, true);
	if (rc)
		goto reg_read_err;

	register_dump_log(dev, reg_addr, val, true, words, IAXXX_READ);

reg_read_err:
	return rc;
}

/* This spi-cmd function is raw data version. This function sends cmd
 * to firmware using SPI functions and pass the response to caller as
 * it is without modifying any data.
 */
static int iaxxx_spi_raw_cmd(struct iaxxx_priv *priv, u32 cmd, u32 *resp)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);

	return iaxxx_spi_cmd(spi, cmd, resp);
}

/* This spi-read function is raw data version. This function reads data
 * from firmware using SPI functions and pass it to caller as it is
 * without modifying any data.
 */
#if 0  /* Knowles original */
static int iaxxx_spi_bus_raw_read(struct iaxxx_priv *priv, void *buf, int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_transfer t[1] = {};
	struct spi_message m;
	uint8_t *cbuf = (uint8_t *)buf;
	int rc;
	uint32_t val_len;
	uint8_t *rx_buf;
	uint8_t *tx_buf;

	if (len <= IAXXX_REG_LEN_WITH_PADDING) {
		pr_err("dev id-%d: read command, len %d", priv->dev_id, len);
		return spi_read(spi, buf, len);
	}

	spi_message_init(&m);
	/* Create buffer to store read data */
	val_len = len - IAXXX_REG_LEN_WITH_PADDING;

	tx_buf = kzalloc(len, GFP_DMA | GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	rx_buf = kzalloc(len, GFP_DMA | GFP_KERNEL);
	if (!rx_buf) {
		rc = -ENOMEM;
		goto mem_alloc_fail;
	}

	memcpy(tx_buf, cbuf, IAXXX_REG_LEN_WITH_PADDING);

	rc = iaxxx_pm_get_sync(&spi->dev);
	if (rc < 0)
		goto pm_sync_err;

	/* Add Register address write message */
	t[0].len = len;
	t[0].tx_buf = tx_buf;
	t[0].rx_buf = rx_buf;
	spi_message_add_tail(&t[0], &m);

	/* Transfer the message */
	rc = spi_sync(spi, &m);
	if (rc)
		goto reg_read_err;

	/* Copy the read data into buffer after reagister address */
	memcpy((uint8_t *)(buf + IAXXX_REG_LEN_WITH_PADDING),
			rx_buf + IAXXX_REG_LEN_WITH_PADDING, val_len);
	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

reg_read_err:
	iaxxx_pm_put_autosuspend(&spi->dev);
pm_sync_err:
	kfree(rx_buf);
mem_alloc_fail:
	kfree(tx_buf);
	return rc;
}
#else  /* Adapt for MT9615 who doesn't support full-duplex communication in one spi_transfer. */
static int iaxxx_spi_bus_raw_read(struct iaxxx_priv *priv, void *buf, int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_transfer t[2] = {};
	struct spi_message m;
	uint8_t *cbuf = (uint8_t *)buf;
	int rc;
	uint32_t val_len;
	uint8_t *rx_buf;
	uint8_t *tx_buf;

	if (len <= IAXXX_REG_LEN_WITH_PADDING) {
		pr_err("dev id-%d: read command, len %d", priv->dev_id, len);
		return spi_read(spi, buf, len);
	}

	spi_message_init(&m);
	/* Create buffer to store read data */
	val_len = len - IAXXX_REG_LEN_WITH_PADDING;

	tx_buf = kzalloc(len, GFP_DMA | GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	rx_buf = kzalloc(len, GFP_DMA | GFP_KERNEL);
	if (!rx_buf) {
		rc = -ENOMEM;
		goto mem_alloc_fail;
	}

	memcpy(tx_buf, cbuf, IAXXX_REG_LEN_WITH_PADDING);

	rc = iaxxx_pm_get_sync(&spi->dev);
	if (rc < 0)
		goto pm_sync_err;

	memset(t, 0, sizeof(t));
	/* Add Register address write message */
	t[0].len = IAXXX_REG_LEN_WITH_PADDING;
	t[0].tx_buf = tx_buf;
	t[1].len = val_len;
	t[1].rx_buf = rx_buf + IAXXX_REG_LEN_WITH_PADDING;
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	/* Transfer the message */
	rc = spi_sync(spi, &m);
	if (rc)
		goto reg_read_err;

	/* Copy the read data into buffer after reagister address */
	memcpy((uint8_t *)(buf + IAXXX_REG_LEN_WITH_PADDING),
			rx_buf + IAXXX_REG_LEN_WITH_PADDING, val_len);
	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

reg_read_err:
	iaxxx_pm_put_autosuspend(&spi->dev);
pm_sync_err:
	kfree(rx_buf);
mem_alloc_fail:
	kfree(tx_buf);
	return rc;
}
#endif

/* This spi-write function is raw data version. The caller of this function
 * generates the whole data and this driver does not alter anything and just
 * uses SPI functions to pass it to firmware.
 */
static int iaxxx_spi_bus_raw_write(struct iaxxx_priv *priv, const void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);
	int rc;

	rc = iaxxx_pm_get_sync(dev);
	if (rc < 0)
		goto pm_sync_err;

	rc = spi_write(spi, buf, len);
	if (rc) {
		dev_err(priv->dev, "%s: failed : %d\n", __func__, rc);
		goto reg_write_err;
	}

	if (priv && priv->need_extra_delay_for_rd_wr == true)
		usleep_range(50, 100);

reg_write_err:
	iaxxx_pm_put_autosuspend(dev);
pm_sync_err:
	return rc;
}

static int iaxxx_copy_be32_to_cpu(struct device *dev,
		void *data_buff, uint32_t len)
{
	int i;
	uint32_t *buff = data_buff;

	if (len % 4) {
		dev_warn(dev, "Not aligned 32bit boundary (%d).\n",
				len);
		len -= (len % 4);
	}

	for (i = 0; i < len/sizeof(uint32_t); i++) {
		u32 data = buff[i];

		buff[i] = be32_to_cpu(data);
	}
	return len;
}

/* This spi-read function is used across driver code for reading directly
 * from firmware's memory. Since SPI communication with firmware is in
 * Big-Endian format, the data read here needs to be converted to CPU
 * byte-order before passing it back. Also register address received here
 * is in CPU byte-order and should be converted to Big-endian before
 * sending for spi read.
 *
 * This function also reads from firmware in chunks defined by
 * IAXXX_SPI_PACKET_LEN since some SPI master drivers have limitation on max
 * amount of data that can be read.
 */
static int iaxxx_spi_bulk_read(struct device *dev,
			uint32_t address, void *read_buf, size_t words)
{
	uint32_t reg_addr;
	size_t reg_len = sizeof(uint32_t) + 12;
	uint32_t words_to_read;
	int rc;
	int count = 0;

	while (words > 0) {
		words_to_read = (words < IAXXX_SPI_PACKET_LEN) ?
				words : IAXXX_SPI_PACKET_LEN;

		dev_dbg(dev, "%s: words_to_read %d\n",
					__func__, words_to_read);

		/*
		 * The register address passed to iaxxx_spi_read_common
		 * should be in Big-endian format.
		 */
		reg_addr = cpu_to_be32(address);
		rc = iaxxx_spi_read_common(dev, &reg_addr, reg_len, read_buf,
				(words_to_read * 4), true);
		if (rc < 0) {
			dev_err(dev,
				"%s: bulk read failed : %d\n",
				__func__, rc);
			return rc;
		}

		/* Data from iaxxx_spi_read_common function is in be32 format
		 * so it should be converted back to CPU byte order.
		 */
		iaxxx_copy_be32_to_cpu(dev,
					read_buf, (words_to_read << 2));
		address += (words_to_read * sizeof(uint32_t));
		read_buf = ((char *)read_buf) +
				(words_to_read * sizeof(uint32_t));
		words -= words_to_read;
		count += words_to_read;
	}

	return count;
}

/* This version of spi-write is called by second regmap for no_pm calls.
 * Since regmap is configured for Big-endian, the data received here will be
 * in Big-endian format. Since SPI communication with Firmware is Big-endian,
 * no byte-order conversion is needed before sending data.
 *
 * This version does not trigger any power-management related calls.
 */
static int iaxxx_regmap_spi_write_no_pm(void *context,
		const void *data, size_t count)
{
	const void *val = data + sizeof(uint32_t);
	size_t reg_len = sizeof(uint32_t);
	size_t val_len = count - reg_len;
	struct device *dev = context;
	uint32_t *writebuf = (uint32_t *)((uint8_t *)val + IAXXX_REG_PADDING);
	uint32_t reg_addr = *(uint32_t *)data;
	uint32_t words = (val_len - IAXXX_REG_PADDING) / sizeof(uint32_t);
	int rc;

	if (WARN_ON(count <= sizeof(uint32_t))
		|| WARN_ON(val_len <= IAXXX_REG_PADDING)) {
		pr_err("Error Input param put of range");
		return -EINVAL;
	}

	/* flag set to indicate data and reg-address are in BE32 format */
	rc = iaxxx_spi_write_common(context, data, reg_len,
			val, val_len, true, false);

	register_dump_log(dev, reg_addr, writebuf, true, words, IAXXX_WRITE);
	return rc;
}

/* This version of spi-read is called by second regmap for no-pm calls.
 * Since SPI communication with firmware is in Big-endian and regmap
 * is also configured for Big-endian, the data read here does not
 * need to be altered.
 * Also the register address received here is also in Big-endian format.
 * This function would execute without calling any power-management
 * functions.
 */
static int iaxxx_regmap_spi_read_no_pm(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len)
{
	struct device *dev = context;
	uint32_t words = val_len / sizeof(uint32_t);
	int rc = 0;
	uint32_t reg_addr = *(uint32_t *) reg;

	rc = iaxxx_spi_read_common(context, reg, reg_len, val, val_len, false);
	if (rc)
		goto reg_read_err;

	register_dump_log(dev, reg_addr, val, true, words, IAXXX_READ);

reg_read_err:
	return rc;
}

/* This spi-write function is regmap interface function for second
 * no_pm regmap. It is called from regmap which is configured to send
 * data in Big-Endian.
 * Since SPI communication with firmware is Big-Endian, there is no
 * conversion needed before sending data.
 *
 * This function does not trigger any power-management related calls.
 */
static int iaxxx_regmap_spi_gather_write_no_pm(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	/* flag set to indicate data and reg-address are in BE32 format
	 * and no pm.
	 */
	return iaxxx_spi_write_common(context,
				 reg, reg_len,
				 val, val_len,
				 true, false);
}

static int iaxxx_spi_populate_dt_data(struct device *dev,
						struct iaxxx_priv *priv)
{
	struct device_node *node = dev->of_node;
	int rc = -EINVAL;
	u32 dt_val32;

	/* No node means no dts provision */
	if (node == NULL) {
		pr_err("Invalid of_node");
		return 0;
	}

	rc = of_property_read_u32(node, "adnc,spi-app-speed", &dt_val32);
	if (rc < 0) {
		pr_err("read spi_app_speed failed : %d", rc);
		return rc;
	}
	priv->app_speed = dt_val32;
	dev_dbg(dev, "%s: spi_app_speed %d\n", __func__, priv->app_speed);

	rc = of_property_read_u32(node, "adnc,dev-id", &dt_val32);
	/* if "dev-id" property is not found, assume and set as 0 */
	if (rc && rc == -EINVAL) {
		pr_err("\"adnc,dev-id\" property not found. assuming 0");
		dt_val32 = 0;
	} else if (rc < 0) {
		pr_err("read dev-id failed : %d", rc);
		return rc;
	}
	dev_dbg(dev, "%s: dev_id %d\n", __func__, dt_val32);

	/* validate the dev-id received */
	if (dt_val32 >= MAX_IAXXX_DEVICES) {
		pr_err("dev-id(%d) more than allowed (%d)",
						dt_val32, MAX_IAXXX_DEVICES);
		return -EINVAL;
	}

	mutex_lock(&device_reg_order_lock);

	/* check if dev-id is already assigned */
	if (test_bit(dt_val32, dev_id_map)) {
		pr_err("dev-id (%d) already assigned. check log", dt_val32);
		mutex_unlock(&device_reg_order_lock);
		return -EEXIST;
	}

	priv->dev_id = dt_val32;
	set_bit(dt_val32, dev_id_map);
	mutex_unlock(&device_reg_order_lock);

	dev_info(dev, "SPI Device Id %d", priv->dev_id);

	rc = of_property_read_u32(node, "adnc,spi-sbl-speed", &dt_val32);
	if (rc < 0) {
		pr_err("read spi-sbl-speed failed : %d. defaulting to %d",
						rc, IAXXX_SPI_SBL_SPEED);
		priv->spi_sbl_speed = IAXXX_SPI_SBL_SPEED;
		rc = 0;
	} else {
		priv->spi_sbl_speed = dt_val32;
	}
	dev_dbg(dev, "%s: spi_sbl_speed %d\n", __func__, priv->spi_sbl_speed);

	return rc;

}

static const struct regmap_bus regmap_spi = {
	.write = iaxxx_regmap_spi_write,
	.gather_write = iaxxx_regmap_spi_gather_write,
	.read = iaxxx_regmap_spi_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_bus regmap_no_pm_spi = {
	.write = iaxxx_regmap_spi_write_no_pm,
	.gather_write = iaxxx_regmap_spi_gather_write_no_pm,
	.read = iaxxx_regmap_spi_read_no_pm,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};


/* Register map initialization */
static int iaxxx_spi_regmap_init(struct iaxxx_priv *priv)
{
	int ret;
	struct device *dev;
	struct regmap *regmap;
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);

	if (!spi_priv || !priv->regmap_config || !priv->regmap_no_pm_config) {
		pr_err("NULL input pointer(s)");
		return -EINVAL;
	}

	dev = &spi_priv->spi->dev;

	/* spi controller requires 12 bytes of zero padding between
	 * address and data
	 */
	priv->regmap_config->pad_bits       = 96;
	priv->regmap_no_pm_config->pad_bits = 96;

	regmap = regmap_init(dev, &regmap_spi,
					spi_priv->spi, priv->regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "allocate register map failed : %d\n", ret);
		return ret;
	}
	priv->regmap = regmap;

	regmap = regmap_init(dev, &regmap_no_pm_spi,
			spi_priv->spi, priv->regmap_no_pm_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "register map alloc for no pm fail : %d\n", ret);
		return ret;
	}
	priv->regmap_no_pm = regmap;
	return 0;
}

static int iaxxx_spi_sync(struct iaxxx_spi_priv *spi_priv)
{
	int retry = IAXXX_SYNC_RETRY;
	int rc = 0;
	uint32_t sync_response;
	struct device *dev = spi_priv->priv.dev;

	do {
		retry--;
		/* Populate device tree data and Reset chip to SBL */
		rc = iaxxx_device_reset(&spi_priv->priv);
		if (rc) {
			dev_err(dev, "%s: device reset failed : %d\n",
				__func__, rc);
			break;
		}

		/*
		 * If SPI SBL Speed is > 4MHz, special command needs to be sent
		 * to the chip
		 */
		if (spi_priv->priv.spi_sbl_speed > IAXXX_SPI_MAX_SBL_SPEED) {
			rc = iaxxx_spi_cmd(spi_priv->spi, SET_SBL_SPI_SPEED_CMD,
								&sync_response);
			if (rc || sync_response != SET_SBL_SPI_SPEED_CMD) {
				dev_err(dev,
				"%s: high speed setup failed : %d resp 0x%08x\n",
						__func__, rc, sync_response);

				/* if rc is 0, set proper error value */
				if (rc == 0)
					rc = -EINVAL;

				continue;
			}
		}

		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD, &sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_RESPONSE) {
			dev_err(dev,
				"%s: SYNC cmd failed : %d, resp:0x%.08X. retry\n",
				__func__, rc, sync_response);

			/* if rc is 0, update it with proper error value */
			if (rc == 0)
				rc = -EINVAL;

			continue;
		}

		dev_info(dev, "SYNC response: 0x%.08X\n", sync_response);

		sync_response = 0;
		dev_info(dev, "Putting device in regmap mode\n");

		/* Switch the device into regmap mode */
		rc = iaxxx_spi_cmd(spi_priv->spi, CMD_REGMAP_MODE, NULL);
		if (rc) {
			dev_err(dev,
				"%s: switch to regmap mode failed : %d\n",
				__func__, rc);
			continue;
		}
	} while (rc && retry);

	return rc;
}

static int iaxxx_spi_reset_cb(struct device *dev)
{
	struct iaxxx_spi_priv *spi_priv = dev ? dev_get_drvdata(dev) : NULL;
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;
	int rc = -EINVAL;

	if (!spi_priv || !spi)
		return rc;

	/* set SPI speed to 4.8Mhz */
	spi->max_speed_hz = spi_priv->priv.spi_sbl_speed;
	rc = spi_setup(spi);
	if (rc)
		dev_err(dev, "%s: spi %dhz setup failed : %d\n",
				__func__, spi->max_speed_hz, rc);
	else
		dev_info(dev, "success spi->max_speed_hz = %d\n",
							spi->max_speed_hz);

	rc = iaxxx_spi_sync(spi_priv);
	if (rc)
		return rc;

	usleep_range(1000, 2500);

	return rc;
}

static int iaxxx_spi_speed_setup(struct device *dev, u32 iaxxx_spi_speed)
{
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;
	int rc = -EINVAL;

	if (!spi)
		return rc;

	dev_dbg(dev, "%s: requested spi speed %d\n", __func__, iaxxx_spi_speed);

	/*
	 * If SPI speed is already set to what is requested,
	 * ignore the request
	 */
	if (spi->max_speed_hz == iaxxx_spi_speed) {
		dev_info(dev, "%s SPI speed is already %dHz\n",
			 __func__, spi->max_speed_hz);
		return 0;
	}

	/*
	 * Before jumping to disable control interface mode, host updates the
	 * current speed. If SPI speed is increased and chip is in disable
	 * control interface mode then wakeup will not work. Thats why its
	 * required to wakeup the chip before modifying the speed.
	 */
	rc = iaxxx_pm_get_sync(&spi->dev);
	if (rc < 0)
		return rc;

	spi->max_speed_hz = iaxxx_spi_speed;

	rc = spi_setup(spi);
	if (rc)
		dev_err(dev, "%s: spi speed:%d setup failed : %d\n",
				__func__, spi->max_speed_hz, rc);
	else
		dev_info(dev, "success spi->max_speed_hz = %d\n",
							spi->max_speed_hz);

	iaxxx_pm_put_autosuspend(&spi->dev);

	return rc;
}

static int iaxxx_spi_probe(struct spi_device *spi)
{
	int rc = 0;
	struct iaxxx_spi_priv *spi_priv;
	struct device *dev = &spi->dev;

	dev_info(dev, "%s:\n", __func__);

	/* Create driver private-data struct */
	spi_priv = devm_kzalloc(dev, sizeof(*spi_priv), GFP_KERNEL);
	if (!spi_priv)
		return -ENOMEM;

	spi_priv->spi = spi;
	spi_priv->priv.dev = dev;
	spi_priv->priv.regmap_init_bus = iaxxx_spi_regmap_init;
	spi_priv->priv.bus_read = iaxxx_spi_bulk_read;
	spi_priv->priv.bus_write = iaxxx_spi_bus_write;
	spi_priv->priv.bus = IAXXX_SPI;
	/*
	 * Buffer plugin EP tunneling using event based method is not working,
	 * thats why as a workaround changed the tunnel method to polling
	 * spi_priv->priv.tunnel_method = IAXXX_EVENT;
	 */
	spi_priv->priv.tunnel_method = IAXXX_POLL;

	spi_set_drvdata(spi, spi_priv);

	spi_priv->priv.intf_speed_setup = iaxxx_spi_speed_setup;
	spi_priv->priv.get_intf_speed = iaxxx_get_spi_speed;
	spi_priv->priv.reset_cb = iaxxx_spi_reset_cb;

	/* populate SPI params from DT*/
	rc = iaxxx_spi_populate_dt_data(dev, &spi_priv->priv);
	if (rc < 0) {
		dev_err(dev, "populate spi params from dt failed : %d\n",
			rc);
		goto probe_failed;
	}

	spi_priv->priv.intf_speed_addr = IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR;
	spi_priv->priv.intf_max_speed = spi_priv->priv.app_speed;

	rc = iaxxx_device_init(&spi_priv->priv);
	if (rc) {
		dev_err(dev, "%s: device init failed : %d\n", __func__, rc);
		goto probe_failed;
	}
	spi_priv->priv.raw_ops->write = iaxxx_spi_bus_raw_write;
	spi_priv->priv.raw_ops->read = iaxxx_spi_bus_raw_read;
	spi_priv->priv.raw_ops->cmd = iaxxx_spi_raw_cmd;
	spi_priv->priv.crash_count = 0;

	return rc;

probe_failed:
	devm_kfree(dev, spi_priv);
	spi_set_drvdata(spi, NULL);
	return rc;
}

static int iaxxx_spi_remove(struct spi_device *spi)
{
	struct iaxxx_spi_priv *spi_priv = spi_get_drvdata(spi);
	struct iaxxx_priv *priv = to_iaxxx_priv(&spi->dev);
	int dev_id = priv->dev_id;

	mutex_lock(&device_reg_order_lock);
	if (!test_bit(dev_id, dev_id_map)) {
		pr_err("dev-id (%d) not assigned. check log", dev_id);
		mutex_unlock(&device_reg_order_lock);
		return -ENODEV;
	}

	clear_bit(dev_id, dev_id_map);
	mutex_unlock(&device_reg_order_lock);

	if (spi_priv) {
		iaxxx_device_exit(&spi_priv->priv);
		devm_kfree(&spi->dev, spi_priv);
	}
	spi_set_drvdata(spi, NULL);

	return 0;
}

static void iaxxx_spi_shutdown(struct spi_device *spi)
{
	struct iaxxx_spi_priv *spi_priv = spi_get_drvdata(spi);
	struct iaxxx_priv *priv = to_iaxxx_priv(&spi->dev);

	if (priv->boot_completed) {
		dev_info(priv->dev, "%s: stop dsp \n", __func__);
		gpio_set_value(priv->reset_gpio, 0);
	}
}

static const struct dev_pm_ops iaxxx_spi_pm_ops = {
	.complete = iaxxx_pm_complete,
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	SET_RUNTIME_PM_OPS(iaxxx_core_suspend_rt, iaxxx_core_resume_rt, NULL)
#endif
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_core_dev_suspend, iaxxx_core_dev_resume)
};

static const struct spi_device_id iaxxx_spi_id[] = {
	{ "iaxxx-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, iaxxx_spi_id);

static const struct of_device_id iaxxx_spi_dt_match[] = {
	{
		.compatible = "knowles,iaxxx-spi",
	},
	{}
};

static struct spi_driver iaxxx_spi_driver = {
	.driver = {
		.name = "iaxxx-spi",
		.owner = THIS_MODULE,
		.pm = &iaxxx_spi_pm_ops,
		.of_match_table = iaxxx_spi_dt_match,
	},
	.probe = iaxxx_spi_probe,
	.remove = iaxxx_spi_remove,
	.shutdown = iaxxx_spi_shutdown,
	.id_table = iaxxx_spi_id,
/*	.device_up = iaxxx_spi_up, */
/*	.device_down = iaxxx_spi_down, */
/*	.reset_device = iaxxx_spi_reset, */
};

module_spi_driver(iaxxx_spi_driver);

MODULE_DESCRIPTION("SPI support for Knowles IAxxx");
MODULE_LICENSE("GPL");
