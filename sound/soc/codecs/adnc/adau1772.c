/*
 * Driver for ADAU1772 codec
 *
 * Copyright 2019 Knowles Corporation
 *
 * Licensed under the GPL-2 or later.
 */

#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "ia8x01-dev-board.h"

struct adau_priv {
	struct device *dev;
	struct regmap *regmap;

	int reset_gpio;
	int sw_i2s_sel_gpio;
};

static struct adau_priv *adau_local;

#define ADAU1772_CLK_CONTROL		0x00
#define ADAU1772_PLL_CTRL0		0x01
#define ADAU1772_PLL_CTRL1		0x02
#define ADAU1772_PLL_CTRL2		0x03
#define ADAU1772_PLL_CTRL3		0x04
#define ADAU1772_PLL_CTRL4		0x05
#define ADAU1772_PLL_CTRL5		0x06
#define ADAU1772_CLKOUT_SEL		0x07
#define ADAU1772_REGULATOR		0x08
#define ADAU1772_CORE_CONTROL		0x09
#define ADAU1772_CORE_ENABLE		0x0b
#define ADAU1772_DBREG0			0x0c
#define ADAU1772_DBREG1			0x0d
#define ADAU1772_DBREG2			0x0e
#define ADAU1772_CORE_IN_MUX_0_1	0x0f
#define ADAU1772_CORE_IN_MUX_2_3	0x10
#define ADAU1772_DAC_SOURCE_0_1		0x11
#define ADAU1772_PDM_SOURCE_0_1		0x12
#define ADAU1772_SOUT_SOURCE_0_1	0x13
#define ADAU1772_SOUT_SOURCE_2_3	0x14
#define ADAU1772_SOUT_SOURCE_4_5	0x15
#define ADAU1772_SOUT_SOURCE_6_7	0x16
#define ADAU1772_ADC_SDATA_CH		0x17
#define ADAU1772_ASRCO_SOURCE_0_1	0x18
#define ADAU1772_ASRCO_SOURCE_2_3	0x19
#define ADAU1772_ASRC_MODE		0x1a
#define ADAU1772_ADC_CONTROL0		0x1b
#define ADAU1772_ADC_CONTROL1		0x1c
#define ADAU1772_ADC_CONTROL2		0x1d
#define ADAU1772_ADC_CONTROL3		0x1e
#define ADAU1772_ADC0_VOLUME		0x1f
#define ADAU1772_ADC1_VOLUME		0x20
#define ADAU1772_ADC2_VOLUME		0x21
#define ADAU1772_ADC3_VOLUME		0x22
#define ADAU1772_PGA_CONTROL_0		0x23
#define ADAU1772_PGA_CONTROL_1		0x24
#define ADAU1772_PGA_CONTROL_2		0x25
#define ADAU1772_PGA_CONTROL_3		0x26
#define ADAU1772_PGA_STEP_CONTROL	0x27
#define ADAU1772_PGA_10DB_BOOST		0x28
#define ADAU1772_POP_SUPPRESS		0x29
#define ADAU1772_TALKTHRU		0x2a
#define ADAU1772_TALKTHRU_GAIN0		0x2b
#define ADAU1772_TALKTHRU_GAIN1		0x2c
#define ADAU1772_MIC_BIAS		0x2d
#define ADAU1772_DAC_CONTROL1		0x2e
#define ADAU1772_DAC0_VOLUME		0x2f
#define ADAU1772_DAC1_VOLUME		0x30
#define ADAU1772_OP_STAGE_MUTES		0x31
#define ADAU1772_SAI_0			0x32
#define ADAU1772_SAI_1			0x33
#define ADAU1772_SOUT_CONTROL0		0x34
#define ADAU1772_PDM_OUT		0x36
#define ADAU1772_PDM_PATTERN		0x37
#define ADAU1772_MODE_MP0		0x38
#define ADAU1772_MODE_MP1		0x39
#define ADAU1772_MODE_MP2		0x3a
#define ADAU1772_MODE_MP3		0x3b
#define ADAU1772_MODE_MP4		0x3c
#define ADAU1772_MODE_MP5		0x3d
#define ADAU1772_MODE_MP6		0x3e
#define ADAU1772_PB_VOL_SET		0x3f
#define ADAU1772_PB_VOL_CONV		0x40
#define ADAU1772_DEBOUNCE_MODE		0x41
#define ADAU1772_OP_STAGE_CTRL		0x43
#define ADAU1772_DECIM_PWR_MODES	0x44
#define ADAU1772_INTERP_PWR_MODES	0x45
#define ADAU1772_BIAS_CONTROL0		0x46
#define ADAU1772_BIAS_CONTROL1		0x47
#define ADAU1772_PAD_CONTROL0		0x48
#define ADAU1772_PAD_CONTROL1		0x49
#define ADAU1772_PAD_CONTROL2		0x4a
#define ADAU1772_PAD_CONTROL3		0x4b
#define ADAU1772_PAD_CONTROL4		0x4c
#define ADAU1772_PAD_CONTROL5		0x4d

static struct reg_default adau1772_reg_required_1[] = {
	{ ADAU1772_CLKOUT_SEL,		0x07 },
	{ ADAU1772_CORE_ENABLE,		0x00 },
	{ ADAU1772_DAC_SOURCE_0_1,	0xDC },
	{ ADAU1772_ASRCO_SOURCE_0_1,	0x54 },
	{ ADAU1772_SOUT_SOURCE_0_1,	0x54 },
	{ ADAU1772_ASRC_MODE,		0x01 },
	{ ADAU1772_ADC_CONTROL0,	0x00 },
	{ ADAU1772_ADC_CONTROL1,	0x00 },
	{ ADAU1772_ADC_CONTROL2,	0x03 },
	{ ADAU1772_ADC_CONTROL3,	0x03 },
	{ ADAU1772_PGA_CONTROL_0,	0x90 },
	{ ADAU1772_PGA_CONTROL_1,	0x90 },
	{ ADAU1772_PGA_CONTROL_2,	0x90 },
	{ ADAU1772_PGA_CONTROL_3,	0x90 },
	{ ADAU1772_DAC_CONTROL1,	0x03 },
	{ ADAU1772_OP_STAGE_MUTES,	0x00 },
	{ ADAU1772_MODE_MP1,		0x10 },
	{ ADAU1772_OP_STAGE_CTRL,	0x00 },
	{ ADAU1772_DECIM_PWR_MODES,	0xff },
	{ ADAU1772_INTERP_PWR_MODES,	0x0f },
	{ ADAU1772_SAI_0,		0x00 },
	{ ADAU1772_SAI_1,		0x00 },
};

static struct reg_default adau1772_reg_required_2[] = {
	{ ADAU1772_CLKOUT_SEL,		0x07 },
	{ ADAU1772_CORE_ENABLE,		0x00 },
	{ ADAU1772_DAC_SOURCE_0_1,	0xDC },
	{ ADAU1772_ASRCO_SOURCE_0_1,	0x54 },
	{ ADAU1772_SOUT_SOURCE_0_1,	0x54 },
	{ ADAU1772_ASRC_MODE,		0x01 },
	{ ADAU1772_ADC_CONTROL0,	0x00 },
	{ ADAU1772_ADC_CONTROL1,	0x00 },
	{ ADAU1772_ADC_CONTROL2,	0x03 },
	{ ADAU1772_ADC_CONTROL3,	0x03 },
	{ ADAU1772_PGA_CONTROL_0,	0x90 },
	{ ADAU1772_PGA_CONTROL_1,	0x90 },
	{ ADAU1772_PGA_CONTROL_2,	0x90 },
	{ ADAU1772_PGA_CONTROL_3,	0x90 },
	{ ADAU1772_DAC_CONTROL1,	0x03 },
	{ ADAU1772_OP_STAGE_MUTES,	0x00 },
	{ ADAU1772_MODE_MP1,		0x10 },
	{ ADAU1772_OP_STAGE_CTRL,	0x00 },
	{ ADAU1772_DECIM_PWR_MODES,	0xff },
	{ ADAU1772_INTERP_PWR_MODES,	0x0f },
	{ ADAU1772_SAI_1,		0x44 },
	{ ADAU1772_SOUT_SOURCE_0_1,	0x98 },
};

int adau1772_reset(struct adau_priv *adau)
{
	int ret = 0;

	gpio_set_value_cansleep(adau->reset_gpio, 0);
	usleep_range(500, 1000);

	gpio_set_value_cansleep(adau->reset_gpio, 1);
	usleep_range(500, 1000);

	return ret;
}

int adau1772_swap_i2s_do_di_pins(bool swap_on)
{
	int ret = 0;

	pr_debug("Enter, swap_on %d", swap_on);
	if (!adau_local)
		return -EINVAL;

	gpio_set_value_cansleep(adau_local->sw_i2s_sel_gpio, (swap_on ? 1 : 0));
	usleep_range(500, 1000);

	return ret;
}
EXPORT_SYMBOL(adau1772_swap_i2s_do_di_pins);

int _setup_adau1772_pll(struct adau_priv *adau)
{
	int ret = 0;
	int retries = 0;
	unsigned int val;

	pr_debug("Enter");

	ret = regmap_write(adau->regmap, ADAU1772_CLK_CONTROL, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(adau->regmap, ADAU1772_PLL_CTRL4, 0x10);
	if (ret)
		return ret;

	ret = regmap_write(adau->regmap, ADAU1772_CLK_CONTROL, 0x80);
	if (ret)
		return ret;

	do {
		usleep_range(1000, 1500);

		ret = regmap_read(adau->regmap, ADAU1772_PLL_CTRL5, &val);
		if (ret)
			return ret;

		retries++;

		if (retries >= 8)
			return -EIO;

	} while (!(val & 1));

	ret = regmap_write(adau->regmap, ADAU1772_CLK_CONTROL, 0x88);
	if (ret)
		return ret;

	ret = regmap_update_bits(adau->regmap, ADAU1772_CLK_CONTROL, 0x01, 0x1);

	return ret;
}

int _setup_adau1772_required_reg(struct adau_priv *adau)
{
	int ret = 0;
	int i;

	pr_debug("Enter");

	ret = _setup_adau1772_pll(adau);
	if (ret) {
		pr_err("setting pll failed : %d", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(adau1772_reg_required_1); i++) {
		ret = regmap_write(adau->regmap, adau1772_reg_required_1[i].reg,
					adau1772_reg_required_1[i].def);
		if (ret) {
			pr_err("writing defaults to reg 0x%08x failed : %d",
					adau1772_reg_required_1[i].reg, ret);
			return ret;
		}
	}

	ret = _setup_adau1772_pll(adau);
	if (ret) {
		pr_err("setting pll failed : %d", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(adau1772_reg_required_2); i++) {
		ret = regmap_write(adau->regmap, adau1772_reg_required_2[i].reg,
					adau1772_reg_required_2[i].def);
		if (ret) {
			pr_err("writing defaults to reg 0x%08x failed : %d",
					adau1772_reg_required_2[i].reg, ret);
			return ret;
		}
	}

	return ret;
}

int setup_adau1772_required_reg(void)
{
	int ret = 0;

	pr_debug("Enter");

	if (!adau_local)
		return -EINVAL;

	ret = _setup_adau1772_required_reg(adau_local);

	return ret;
}

static const DECLARE_TLV_DB_SCALE(adau1772_out_tlv, -9562, 37, 0);

static const struct snd_kcontrol_new adau1772_snd_controls[] = {
	SOC_DOUBLE_R_TLV("DAC Volume", ADAU1772_DAC0_VOLUME,
		ADAU1772_DAC1_VOLUME, 0, 255, 1, adau1772_out_tlv),
};

static int adau1772_codec_probe(struct snd_soc_codec *codec)
{
	struct adau_priv *adau = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(adau->dev, "%s: Enter\n", __func__);

	ret = _setup_adau1772_required_reg(adau);
	if (ret)
		return ret;

	snd_soc_add_codec_controls(codec, adau1772_snd_controls,
		ARRAY_SIZE(adau1772_snd_controls));

	return ret;
}

#define ADAU1772_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver adau1772_dai_driver = {
	.name = "adau-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ADAU1772_FORMATS,
	},
};

static struct snd_soc_codec_driver adau1772_codec_driver = {
	.probe			= adau1772_codec_probe,
};

bool adau1772_volatile_register(struct device *dev, unsigned int reg)
{
	return true;
}

static bool adau1772_readable_register(struct device *dev, unsigned int reg)
{
	if (reg == 0x0A || reg == 0x35 || reg == 0x42)
		return false;

	if (reg <= 0x4d)
		return true;

	return false;
}

static const struct regmap_config adau1772_i2c_regmap_config = {
	.name = "adau1772_regmap",
	.val_bits = 8,
	.reg_bits = 16,
	.max_register = 0x4d,
	.readable_reg = adau1772_readable_register,
	.volatile_reg = adau1772_volatile_register,
	.cache_type = REGCACHE_NONE,
};


static int adau1772_populate_dt_gpios(struct adau_priv *adau)
{
	int ret;

	ret = of_get_named_gpio(adau->dev->of_node, "knowles,reset-gpio", 0);
	if (ret < 0) {
		adau->reset_gpio = -EINVAL;
		dev_err(adau->dev, "read reset-gpio failed : %d\n", ret);
		goto err_reset_exit;
	}
	adau->reset_gpio = ret;

	if (!gpio_is_valid(adau->reset_gpio)) {
		ret = -ENXIO;
		goto err_reset_exit;
	}

	/* GPIO: reset (output used for chip reset) */
	ret = devm_gpio_request_one(adau->dev, adau->reset_gpio,
						GPIOF_OUT_INIT_HIGH, "RESET");
	if (ret < 0)
		goto err_reset_exit;

	ret = of_get_named_gpio(adau->dev->of_node,
						"knowles,sw-i2s-sel-gpio", 0);
	if (ret < 0) {
		adau->sw_i2s_sel_gpio = -EINVAL;
		dev_err(adau->dev, "read sw-i2s-sel-gpio failed : %d\n",
								ret);
		goto err_missing_sw_i2s_sel_gpio;
	}
	adau->sw_i2s_sel_gpio = ret;

	if (!gpio_is_valid(adau->sw_i2s_sel_gpio)) {
		ret = -ENXIO;
		goto err_missing_sw_i2s_sel_gpio;
	}

	ret = devm_gpio_request_one(adau->dev, adau->sw_i2s_sel_gpio,
					GPIOF_OUT_INIT_LOW, "SW-I2S-SEL");
	if (ret < 0)
		goto err_missing_sw_i2s_sel_gpio;

	return ret;

err_missing_sw_i2s_sel_gpio:
	devm_gpio_free(adau->dev, adau->reset_gpio);
err_reset_exit:
	return ret;
}

int adau1772_bus_probe(struct device *dev, struct regmap *regmap)
{
	struct adau_priv *adau;
	int ret;

	dev_dbg(adau->dev, "%s: Enter\n", __func__);

	adau = devm_kzalloc(dev, sizeof(*adau), GFP_KERNEL);
	if (!adau) {
		ret = -ENOMEM;
		goto err_regmap_exit;
	}

	adau->regmap = regmap;
	adau->dev = dev;

	adau_local = adau;

	dev_set_drvdata(dev, adau);

	ret = adau1772_populate_dt_gpios(adau);
	if (ret) {
		dev_err(dev, "read GPIO data failed : %d\n", ret);
		goto err_exit;
	}

	ret = adau1772_reset(adau);
	if (ret < 0)
		goto err_exit;

	return 0;

err_exit:
	dev_set_drvdata(dev, NULL);
err_regmap_exit:
	adau->regmap = NULL;
	return ret;
}

static int adau1772_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id) {
	struct regmap *regmap;
	int ret;

	dev_dbg(&client->dev, "%s: Enter\n", __func__);

	regmap = devm_regmap_init_i2c(client, &adau1772_i2c_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = adau1772_bus_probe(&client->dev, regmap);
	if (ret)
		return ret;

	ret = snd_soc_register_codec(&client->dev, &adau1772_codec_driver,
						&adau1772_dai_driver, 1);
	if (ret)
		goto err_remove;

	return 0;

err_remove:
	dev_set_drvdata(&client->dev, NULL);
	return ret;
}

static int adau1772_i2c_remove(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s: Enter\n", __func__);

	snd_soc_unregister_codec(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static const struct i2c_device_id adau1772_i2c_id[] = {
	{ "adi,adau1772", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1772_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id adau1772_i2c_dt_ids[] = {
	{ .compatible = "adi,adau1772", },
	{ },
};
MODULE_DEVICE_TABLE(of, adau1772_i2c_dt_ids);
#endif

static struct i2c_driver adau1772_i2c_driver = {
	.driver = {
		.name = "adau1772",
		.of_match_table = of_match_ptr(adau1772_i2c_dt_ids),
	},
	.probe = adau1772_i2c_probe,
	.remove = adau1772_i2c_remove,
	.id_table = adau1772_i2c_id,
};

module_i2c_driver(adau1772_i2c_driver);

MODULE_DESCRIPTION("ASoC ADAU1772 CODEC driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
