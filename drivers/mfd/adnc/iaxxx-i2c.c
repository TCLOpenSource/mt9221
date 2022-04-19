/*
 * iaxxx-i2c.c -- Generic i2c driver for Knowles IAxxx device
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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include "iaxxx.h"

#if defined(CONFIG_ARCH_MSM)
#define IAXXX_I2C_LOAD_UA	30000
#define IAXXX_I2C_VTG_MIN_UV	1800000
#define IAXXX_I2C_VTG_MAX_UV	1800000
#endif

#define IAXXX_I2C_NUMBER_OF_TRANSFER_MSG	2
#define IAXXX_RETRY_COUNT			5
#define IAXXX_MAX_RECV_LEN			0x100
#define IAXXX_MAX_TX_LEN			0x800
#define IAXXX_MAX_I2C_SPEED			1000000

/**
 * Description of driver private data
 *
 * @priv: IAxxx private data
 * @i2c:  i2c client pointer
 */
struct iaxxx_i2c_priv {
	struct iaxxx_priv priv;	/* private data */
	struct i2c_client *i2c;

#if defined(CONFIG_ARCH_MSM)
	bool i2c_pull_up;
	struct regulator *vcc_i2c;
#endif
};

static inline struct iaxxx_i2c_priv *to_i2c_priv(struct iaxxx_priv *priv)
{
	return priv ? container_of(priv, struct iaxxx_i2c_priv, priv) : NULL;
}

static int iaxxx_i2c_read(struct i2c_client *i2c, void *buf, int len)
{
	int ret = -1;
	int max_recv_len = IAXXX_MAX_RECV_LEN;
	int count_remain = len;
	int bytes_read = 0;
	int retry = IAXXX_RETRY_COUNT;
	struct i2c_msg msg;

	msg.addr = i2c->addr;
	msg.flags = I2C_M_RD;

	while (count_remain && retry) {
		msg.len = min(count_remain, max_recv_len);
		msg.buf = buf + bytes_read;
		ret = i2c_transfer(i2c->adapter, &msg, 1);
		if (ret != 1) {
			retry--;
			continue;
		}
		retry = IAXXX_RETRY_COUNT;
		bytes_read += msg.len;
		count_remain -= msg.len;
	}

	if (ret != 1) {
		dev_err(&i2c->dev, "i2c_transfer() failed : %d\n", ret);
		return ret > 0 ? -EIO : ret;
	}

	return 0;
}

static int iaxxx_i2c_write(struct i2c_client *i2c, const void *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = len,
			.buf = (void *)buf,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&i2c->dev, "i2c_transfer() failed : %d\n", ret);
		return ret > 0 ? -EIO : ret;
	}

	return 0;
}

static int iaxxx_i2c_bus_raw_read(struct iaxxx_priv *priv, void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct i2c_client *i2c = to_i2c_client(dev);
	int rc;

	rc = iaxxx_i2c_read(i2c, buf, len);
	return rc;
}

static int iaxxx_i2c_bus_raw_write(struct iaxxx_priv *priv, const void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct i2c_client *i2c = to_i2c_client(dev);
	int rc;

	rc = iaxxx_i2c_write(i2c, buf, len);
	return rc;
}

static int iaxxx_i2c_cmd(struct i2c_client *i2c, u32 cmd, u32 *resp)
{
	int ret;

	dev_dbg(&i2c->dev, "iaxxx: cmd = 0x%08x\n", cmd);
	cmd = cpu_to_be32(cmd);
	dev_dbg(&i2c->dev, "iaxxx: cmd = 0x%08x\n", cmd);

	ret = iaxxx_i2c_write(i2c, &cmd, sizeof(cmd));
	if (ret) {
		dev_err(&i2c->dev, "send command 0x%.08X failed : %d\n",
								cmd, ret);
		return ret;
	}

	if (resp) {
		usleep_range(4000, 4500);
		ret = iaxxx_i2c_read(i2c, resp, sizeof(*resp));
		if (ret) {
			dev_err(&i2c->dev, "read cmd response failed : %d\n",
									ret);
			return ret;
		}
		*resp = be32_to_cpu(*resp);
	}

	return 0;
}

static void iaxxx_i2c_32bit_plat_endian(void *data_buf, uint32_t len)
{
	int i;
	uint32_t *buf = data_buf;
	uint32_t data;

	for (i = 0; i < len; i++) {
		data = buf[i];
		buf[i] = cpu_to_be32(data);
	}
}

static int iaxxx_i2c_bus_read(struct device *dev,
			uint32_t reg, void *read_buf, size_t words)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	if (!read_buf || words <= 0) {
		pr_err("Invalid parameters");
		return -EINVAL;
	}

	reg = cpu_to_be32(reg);
	ret = iaxxx_i2c_write(i2c, &reg, 4);
	if (ret) {
		dev_err(dev, "i2c write failed : %d\n", ret);
		return ret;
	}

	ret = iaxxx_i2c_read(i2c, read_buf, words * 4);

	if (!ret) {
		iaxxx_i2c_32bit_plat_endian(read_buf, words);
		return words;
	}

	return ret;
}

static int iaxxx_i2c_chunk_write(struct device *dev,
			uint32_t reg, const void *buf, uint32_t bytes)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct i2c_msg xfer[IAXXX_I2C_NUMBER_OF_TRANSFER_MSG];
	int ret;

	reg = cpu_to_be32(reg);
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(uint32_t);
	xfer[0].buf = (void *)&reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = 0;
	/* Length should be in bytes */
	xfer[1].len = bytes;
	xfer[1].buf = (void *)buf;

	ret = i2c_transfer(i2c->adapter, xfer,
			   IAXXX_I2C_NUMBER_OF_TRANSFER_MSG);

	if (ret == IAXXX_I2C_NUMBER_OF_TRANSFER_MSG)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;

	return ret;
}

static int iaxxx_i2c_bus_write(struct device *dev,
			uint32_t reg, const void *write_buf, size_t words)
{
	int rc = -1;
	void *buf = (void *)write_buf;
	size_t len_in_bytes = words * 4;
	size_t written = 0;
	uint32_t bytes_to_write;

	if (!write_buf || words <= 0) {
		pr_err("Invalid parameters");
		return -EINVAL;
	}

	iaxxx_i2c_32bit_plat_endian(buf, words);

	while (len_in_bytes != 0) {
		bytes_to_write = (uint32_t)min_t(size_t, len_in_bytes,
					       IAXXX_MAX_TX_LEN);
		rc = iaxxx_i2c_chunk_write(dev, reg + written,
			(uint8_t *)buf + written, bytes_to_write);
		if (rc) {
			pr_err("Write failed : %d", rc);
			break;
		}
		written += bytes_to_write;
		len_in_bytes -= bytes_to_write;
	}
	return rc;
}

/* Register map initialization */
static int iaxxx_i2c_regmap_init(struct iaxxx_priv *priv)
{
	int ret;
	struct device *dev;
	struct regmap *regmap;
	struct iaxxx_i2c_priv *i2c_priv = to_i2c_priv(priv);

	if (!i2c_priv || !priv->regmap_config) {
		pr_err("NULL input pointer(s)");
		return -EINVAL;
	}

	regmap = regmap_init_i2c(i2c_priv->i2c, priv->regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev = &i2c_priv->i2c->dev;
		dev_err(dev, "allocate register map failed : %d\n", ret);
		return ret;
	}

	priv->regmap = regmap;

	regmap = regmap_init_i2c(i2c_priv->i2c, priv->regmap_no_pm_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev = &i2c_priv->i2c->dev;
		dev_err(dev, "allocate register map failed : %d\n", ret);
		return ret;
	}

	priv->regmap_no_pm = regmap;

	return 0;
}

#if defined(CONFIG_ARCH_MSM)
static int reg_set_optimum_mode_check(struct regulator *reg, int load_ua)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_ua) : 0;
}

static int iaxxx_i2c_power_on(struct iaxxx_i2c_priv *i2c_priv)
{
	int rc;
	struct device *dev = &i2c_priv->i2c->dev;

	if (i2c_priv->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(i2c_priv->vcc_i2c,
				IAXXX_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(dev,
				"Regulator vcc_i2c set_opt failed : %d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(i2c_priv->vcc_i2c);
		if (rc) {
			dev_err(dev,
				"Regulator vcc_i2c enable failed : %d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}
	msleep(130);
	return 0;

error_reg_en_vcc_i2c:
	if (i2c_priv->i2c_pull_up)
		reg_set_optimum_mode_check(i2c_priv->vcc_i2c, 0);
error_reg_opt_i2c:
	return rc;

}

static int iaxxx_i2c_regulator_configure(struct iaxxx_i2c_priv *i2c_priv)
{
	int rc;
	struct device *dev = &i2c_priv->i2c->dev;

	pr_debug("Configuring i2c_pull_up");

	if (i2c_priv->i2c_pull_up) {
		i2c_priv->vcc_i2c = regulator_get(dev, "vcc_i2c");
		if (IS_ERR(i2c_priv->vcc_i2c)) {
			rc = PTR_ERR(i2c_priv->vcc_i2c);
			dev_err(dev, "Regulator get failed : %d\n", rc);
			goto error_get_vtg_i2c;
		}
		pr_debug("regulator_count_voltages %d",
				regulator_count_voltages(i2c_priv->vcc_i2c));

		if (regulator_count_voltages(i2c_priv->vcc_i2c) > 0) {
			rc = regulator_set_voltage(i2c_priv->vcc_i2c,
				IAXXX_I2C_VTG_MIN_UV, IAXXX_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(dev,
					"regulator set_vtg failed : %d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}
	return 0;

error_set_vtg_i2c:
	regulator_put(i2c_priv->vcc_i2c);
error_get_vtg_i2c:
	return rc;
}

static void iaxxx_i2c_regulator_disable(struct iaxxx_i2c_priv *i2c_priv)
{
	if (i2c_priv->i2c_pull_up) {
		reg_set_optimum_mode_check(i2c_priv->vcc_i2c, 0);
		regulator_disable(i2c_priv->vcc_i2c);
		msleep(50);
	}
}

static int iaxxx_i2c_power_off(struct iaxxx_i2c_priv *i2c_priv)
{
	if (i2c_priv->i2c_pull_up) {
		if (regulator_count_voltages(i2c_priv->vcc_i2c) > 0)
			regulator_set_voltage(i2c_priv->vcc_i2c, 0,
						IAXXX_I2C_VTG_MAX_UV);
		regulator_put(i2c_priv->vcc_i2c);
	}
	return 0;
}
#endif

static int iaxxx_i2c_reset_cb(struct device *dev)
{
	struct iaxxx_i2c_priv *i2c_priv = dev ? dev_get_drvdata(dev) : NULL;
	int retry = IAXXX_SYNC_RETRY;
	int rc = 0;
	uint32_t sync_response;

	if (!i2c_priv)
		return -EINVAL;

	do {
		retry--;
		/* Populate device tree data and Reset chip to SBL */
		rc = iaxxx_device_reset(&i2c_priv->priv);
		if (rc) {
			dev_err(dev, "%s: device reset failed : %d\n",
				__func__, rc);
			break;
		}

		/* Send a SYNC command */
		rc = iaxxx_i2c_cmd(i2c_priv->i2c, SBL_SYNC_CMD, &sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_RESPONSE) {
			dev_err(dev,
				"%s: SYNC cmd failed : %d, resp:0x%.08x. retry\n",
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
		rc = iaxxx_i2c_cmd(i2c_priv->i2c, CMD_REGMAP_MODE, NULL);
		if (rc) {
			dev_err(dev,
				"%s: switch to regmap mode failed : %d\n",
				__func__, rc);
			continue;
		}
	} while (rc && retry);

	usleep_range(1000, 2500);
	return rc;
}

static int iaxxx_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	int rc = 0;
	struct iaxxx_i2c_priv *i2c_priv;
	struct device *dev = &i2c->dev;

#if defined(CONFIG_ARCH_MSM)
	struct device_node *np = NULL;
#endif

	dev_dbg(dev, "%s:\n", __func__);

	/* Create driver private-data struct */
	i2c_priv = devm_kzalloc(dev, sizeof(*i2c_priv), GFP_KERNEL);
	if (!i2c_priv)
		return -ENOMEM;

	i2c_priv->i2c = i2c;
	i2c_priv->priv.dev = dev;
	i2c_priv->priv.regmap_init_bus = iaxxx_i2c_regmap_init;
	i2c_priv->priv.bus = IAXXX_I2C;
	i2c_priv->priv.tunnel_method = IAXXX_POLL;

	i2c_set_clientdata(i2c, i2c_priv);

#if defined(CONFIG_ARCH_MSM)
	np = dev->of_node;

	if (np) {
		/* regulator info */
		i2c_priv->i2c_pull_up =
			of_property_read_bool(np, "adnc,i2c-pull-up");
		dev_dbg(dev, "i2c_pull_up %d\n", i2c_priv->i2c_pull_up);

		rc = iaxxx_i2c_regulator_configure(i2c_priv);
		if (rc) {
			dev_err(dev, "%s: config hw failed : %d\n",
								__func__, rc);
			goto continue_init;
		}
		rc = iaxxx_i2c_power_on(i2c_priv);
		if (rc)
			dev_err(dev, "%s: Power On hardware failed : %d\n",
				__func__, rc);
	}

continue_init:
#endif

	i2c_priv->priv.reset_cb = iaxxx_i2c_reset_cb;

	i2c_priv->priv.intf_speed_addr = IAXXX_PWR_MGMT_MAX_I2C_SPEED_REQ_ADDR;
	i2c_priv->priv.intf_max_speed = IAXXX_MAX_I2C_SPEED;

	rc = iaxxx_device_init(&i2c_priv->priv);
	if (rc) {
		dev_err(dev, "%s: device init failed : %d\n", __func__, rc);
		goto probe_failed;
	}

	/* Raw read write callbacks */
	i2c_priv->priv.raw_ops->read = iaxxx_i2c_bus_raw_read;
	i2c_priv->priv.raw_ops->write = iaxxx_i2c_bus_raw_write;
	i2c_priv->priv.bus_read = iaxxx_i2c_bus_read;
	i2c_priv->priv.bus_write = iaxxx_i2c_bus_write;

	return rc;

probe_failed:
	devm_kfree(dev, i2c_priv);
	i2c_set_clientdata(i2c, NULL);
	return rc;
}

static int iaxxx_i2c_remove(struct i2c_client *i2c)
{
	struct iaxxx_i2c_priv *i2c_priv = i2c_get_clientdata(i2c);

	if (i2c_priv) {
		iaxxx_device_exit(&i2c_priv->priv);
#if defined(CONFIG_ARCH_MSM)
		iaxxx_i2c_power_off(i2c_priv);
		iaxxx_i2c_regulator_disable(i2c_priv);
#endif
		devm_kfree(&i2c->dev, i2c_priv);
	}
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id iaxxx_i2c_id[] = {
	{ "iaxxx-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, iaxxx_i2c_id);

static const struct of_device_id iaxxx_i2c_dt_match[] = {
	{.compatible = "knowles,iaxxx-i2c"},
	{}
};

static struct i2c_driver iaxxx_i2c_driver = {
	.driver = {
		.name = "iaxxx-i2c",
		/* .pm = &iaxxx_pm_ops, */
		.of_match_table = iaxxx_i2c_dt_match,
	},
	.probe = iaxxx_i2c_probe,
	.remove = iaxxx_i2c_remove,
	.id_table = iaxxx_i2c_id,
};

module_i2c_driver(iaxxx_i2c_driver);

MODULE_DESCRIPTION("I2C support for Knowles IAxxx");
MODULE_LICENSE("GPL");
