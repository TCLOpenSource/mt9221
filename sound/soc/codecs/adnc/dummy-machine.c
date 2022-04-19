/*
 * dummy-machine.c  --  Dummy ALSA SoC Machine Driver for IAXXX
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

#include <linux/module.h>
#include <sound/soc.h>


static struct snd_soc_dai_link dummy_dai_links[] = {
	{
		.name = "be-link-with-iaxxx-porta",
		.no_pcm = 1,
		.platform_name = "snd-soc-dummy",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_dai_name = "iaxxx-pcm0",
		.codec_name = "iaxxx-codec.0",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS,
	},
};

static struct snd_soc_card dummy_soc_card = {
	.name = "dummy-card-iaxxx",
	.owner = THIS_MODULE,
	.dai_link = dummy_dai_links,
	.num_links = ARRAY_SIZE(dummy_dai_links)
};

static int dummy_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &dummy_soc_card;
	int ret;
	struct device *dev = &pdev->dev;

	dev_info(dev, "%s enter\n", __func__);

	card->dev = dev;

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		dev_err(dev, "%s devm_snd_soc_register_card fail %d\n",
							__func__, ret);
	return ret;
}

static const struct of_device_id dummy_machine_dt_match[] = {
	{.compatible = "adnc,dummy-machine"},
	{}
};

static struct platform_driver dummy_machine = {
	.driver = {
		.name = "dummy-machine-iaxxx",
		.of_match_table = dummy_machine_dt_match,
	},
	.probe = dummy_machine_probe
};

module_platform_driver(dummy_machine);

/* Module information */
MODULE_DESCRIPTION("Dummy ALSA SoC Machine Driver for IAXXX");
MODULE_AUTHOR("Kui Wang <kui.wang@knowles.com>");
MODULE_LICENSE("GPL v2");
