/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*
 * ============================================================================
 * Include Headers
 * ============================================================================
 */
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/soundcard.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 1))
#include <linux/module.h>
#endif

#include <sound/asound.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 1))
#else
#include <sound/version.h>
#endif
#include <sound/control.h>
#include <sound/initval.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
#include <linux/of.h>
#endif
#include "mdrv_dmic.h"

/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
#define _DMIC_SND_PLATFORM "Mstar-dmic"
#define _DMIC_SND_DRIVER "MStar-DMIC-Drv"
#define _DMIC_SND_SHORTNAME "MStar-dmic"
#define _DMIC_SND_LONGNAME "MStar-dmic"

#define _DMIC_PCM_INSTANCE "dmic_pcm"
#define _DMIC_PCM_NAME "MStar DMIC PCM"
#define _DMIC_CONTROL_NAME "MStar DMIC Control"

/*
 * ============================================================================
 * Forward Declarations
 * ============================================================================
 */

static int _mdrv_set_default_hw(struct snd_pcm_hardware *target_hw);
static int _mdrv_set_default_constraint_list(struct snd_pcm_hw_constraint_list *target_constraint_list);
static int _mdrv_set_hw_by_constrain_list(struct snd_pcm_hardware *target_hw, struct snd_pcm_hw_constraint_list *target_constraint_list);

static int _mdrv_capture_open(struct snd_pcm_substream *substream);
static int _mdrv_capture_close(struct snd_pcm_substream *substream);
static int _mdrv_capture_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);
static int _mdrv_capture_hw_free(struct snd_pcm_substream *substream);
static int _mdrv_capture_prepare(struct snd_pcm_substream *substream);
static int _mdrv_capture_trigger(struct snd_pcm_substream *substream, int cmd);
static snd_pcm_uframes_t _mdrv_capture_pointer(struct snd_pcm_substream *substream);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 53)
static int _mdrv_capture_copy(struct snd_pcm_substream *substream, int channel, unsigned long pos, void __user *buf, unsigned long count);
#else
static int _mdrv_capture_copy(struct snd_pcm_substream *substream, int channel, snd_pcm_uframes_t pos, void __user *buf, snd_pcm_uframes_t count);
#endif
static int _mdrv_capture_suspend(void);
static int _mdrv_capture_resume(void);

static unsigned int _mdrv_convert_format_to_bitwidth(struct snd_pcm_substream *substream);

static int _mdrv_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int _mdrv_control_mic_gain_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);

static int _mdrv_kcontrol_get_enum(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int _mdrv_kcontrol_put_enum(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

static int _mdrv_system_probe(struct platform_device *dev);
static int _mdrv_system_remove(struct platform_device *dev);
static int _mdrv_system_suspend(struct platform_device *dev, pm_message_t state);
static int _mdrv_system_resume(struct platform_device *dev);

static int _mdrv_pcm_new(struct MStar_Device_Struct *dmic_chip, int capture_substreams);
static int _mdrv_probe(void);
static int _mdrv_remove(void);

/*
 * ============================================================================
 * Local Variables
 * ============================================================================
 */

/* MStar Audio DSP */
static bool driver_has_installed = DMIC_FALSE;
static struct MStar_Device_Struct *DMIC_Chip = NULL;
static struct snd_card *DMIC_SND_Card = NULL;
static unsigned char DMIC_Capability = 0;
static unsigned int DMIC_Max_Pcm_Buffer_Size;

/* Default sample rate of ALSA PCM capture */
static unsigned int DMIC_Capture_default_rates[] = {
	16000,
	32000,
	48000,
};

/* Default hardware settings of ALSA PCM capture */
static struct snd_pcm_hardware DMIC_Capture_default_HW = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
	.rate_min = 16000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 8,
	.period_bytes_min = 64,
	.periods_min = 1,
	.periods_max = 1024,
	.fifo_size = 0,
};

/* Operators for ALSA PCM capture */
static struct snd_pcm_ops DMIC_Capture_Ops = {
	.open = _mdrv_capture_open,
	.close = _mdrv_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = _mdrv_capture_hw_params,
	.hw_free = _mdrv_capture_hw_free,
	.prepare = _mdrv_capture_prepare,
	.trigger = _mdrv_capture_trigger, /* Atomic */
	.pointer = _mdrv_capture_pointer, /* Atomic */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 53)
	.copy_user = _mdrv_capture_copy,
#else
	.copy = _mdrv_capture_copy,
#endif
};


#if defined(CONFIG_OF)
static struct of_device_id mstar_dmic_of_device_ids[] = {
	{.compatible = _DMIC_SND_PLATFORM},
	{},
};
#endif

/* Platform driver definition */
static struct platform_driver mstar_dmic_system_driver = {
	.probe = _mdrv_system_probe,
	.remove = _mdrv_system_remove,
	.suspend = _mdrv_system_suspend,
	.resume = _mdrv_system_resume,
	.driver = {
#if defined(CONFIG_OF)
	.of_match_table = mstar_dmic_of_device_ids,
#endif
	.name = _DMIC_SND_PLATFORM,
	.owner = THIS_MODULE,
	}
};

#define SOC_ENUM_DMIC(xname, xenum)																			   \
	{																											 \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .info = _mdrv_control_info,						   \
		.get = _mdrv_kcontrol_get_enum, .put = _mdrv_kcontrol_put_enum, .private_value = ( unsigned long ) &xenum \
	}

#define SOC_INTEGER_DMIC(xname, xenum)																			\
	{																											 \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .info = _mdrv_control_mic_gain_info,				  \
		.get = _mdrv_kcontrol_get_enum, .put = _mdrv_kcontrol_put_enum, .private_value = ( unsigned long ) &xenum \
	}

enum
{
	DMIC_REG_DMIC_ONOFF = 0,
	DMIC_REG_VQ_ONOFF,
	DMIC_REG_SEAMLESS_ONOFF,
	DMIC_REG_DA_ONOFF,
	DMIC_REG_AEC_MODE,
	DMIC_REG_I2S_ONOFF,
	DMIC_REG_HW_AEC_ONOFF,
	DMIC_REG_DMIC_NUMBER,
	DMIC_REG_REF_CH_NUMBER,
	DMIC_REG_MIC_BITWIDTH,
	DMIC_REG_HPF_ONOFF,
	DMIC_REG_HPF_CONFIG,
	DMIC_REG_UART_ONOFF,
	DMIC_REG_SIGEN_ONOFF,
	DMIC_REG_DMIC_GAIN,
	DMIC_REG_LEN,
};

static unsigned int card_reg[DMIC_REG_LEN] = {
	0x0, // DMIC_REG_DMIC_ONOFF
	0x0, // DMIC_REG_VQ_ONOFF
	0x0, // DMIC_REG_SEAMLESS_ONOFF
	0x0, // DMIC_REG_DA_ONOFF
	0x0, // DMIC_REG_AEC_MODE,
	0x0, // DMIC_REG_I2S_ONOFF,
	0x0, // DMIC_REG_HW_AEC_ONOFF,
	0x0, // DMIC_REG_DMIC_NUMBER
	0x0, // DMIC_REG_REF_CH_NUMBER
	0x0, // DMIC_REG_MIC_BITWIDTH
	0x2, // DMIC_REG_HPF_ONOFF
	0x3, // DMIC_REG_HPF_CONFIG
	0x0, // DMIC_REG_UART_ONOFF
	0x0, // DMIC_REG_SIGEN_ONOFF
	0x0, // DMIC_REG_DMIC_GAIN
};

static const char *dmic_onoff_text[] = {"Off", "On"};
static const char *vq_onoff_text[] = {"Off", "On"};
static const char *seamless_onoff_text[] = {"Off", "On"};
static const char *da_onoff_text[] = {"Off", "On"};
static const char *aec_mode_text[] = {"None", "48K Mono", "48k Stereo", "16K Mono", "16K Stereo"};
static const char *i2s_onoff_text[] = {"Off", "On"};
static const char *hw_aec_onoff_text[] = {"Off", "On"};
static const char *dmic_number_text[] = {"2", "4", "6", "8"};
static const char *ref_ch_number_text[] = {"0", "2", "4"};
static const char *mic_bitwidth_text[] = {"16", "24", "32"};
static const char *hpf_onoff_text[] = {"Off", "1-stage", "2-stage"};
static const char *hpf_config_text[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
static const char *uart_onoff_text[] = {"Off", "On"};
static const char *sigen_onoff_text[] = {"Off", "On"};

static const struct soc_enum dmic_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_DMIC_ONOFF, 0, 2, dmic_onoff_text);

static const struct soc_enum vq_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_VQ_ONOFF, 0, 2, vq_onoff_text);

static const struct soc_enum seamless_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_SEAMLESS_ONOFF, 0, 2, seamless_onoff_text);

static const struct soc_enum da_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_DA_ONOFF, 0, 2, da_onoff_text);

static const struct soc_enum aec_mode_enum = SOC_ENUM_SINGLE(DMIC_REG_AEC_MODE, 0, 5, aec_mode_text);

static const struct soc_enum i2s_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_I2S_ONOFF, 0, 2, i2s_onoff_text);

static const struct soc_enum hw_aec_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_HW_AEC_ONOFF, 0, 2, hw_aec_onoff_text);

static const struct soc_enum dmic_number_enum = SOC_ENUM_SINGLE(DMIC_REG_DMIC_NUMBER, 0, 4, dmic_number_text);

static const struct soc_enum ref_ch_number_enum = SOC_ENUM_SINGLE(DMIC_REG_REF_CH_NUMBER, 0, 3, ref_ch_number_text);

static const struct soc_enum mic_bitwidth_enum = SOC_ENUM_SINGLE(DMIC_REG_MIC_BITWIDTH, 0, 3, mic_bitwidth_text);

static const struct soc_enum hpf_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_HPF_ONOFF, 0, 3, hpf_onoff_text);

static const struct soc_enum hpf_config_enum = SOC_ENUM_SINGLE(DMIC_REG_HPF_CONFIG, 0, 12, hpf_config_text);

static const struct soc_enum uart_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_UART_ONOFF, 0, 2, uart_onoff_text);

static const struct soc_enum sigen_onoff_enum = SOC_ENUM_SINGLE(DMIC_REG_SIGEN_ONOFF, 0, 2, sigen_onoff_text);

static const struct soc_enum dmic_gain_enum = SOC_ENUM_SINGLE(DMIC_REG_DMIC_GAIN, 0, 480, 0);

bool bIsNeedReInit = DMIC_TRUE;

static const struct snd_kcontrol_new DMIC_Control_Ops[] = {
	SOC_ENUM_DMIC("DMIC switch", dmic_onoff_enum),
	SOC_ENUM_DMIC("Voice WakeUp Switch", vq_onoff_enum),
	SOC_ENUM_DMIC("Seamless Mode", seamless_onoff_enum),
	SOC_ENUM_DMIC("Mic data arrange Switch", da_onoff_enum),
	SOC_ENUM_DMIC("AEC Mode", aec_mode_enum),
	SOC_ENUM_DMIC("I2S Switch", i2s_onoff_enum),
	SOC_ENUM_DMIC("HW AEC Switch", hw_aec_onoff_enum),
	SOC_ENUM_DMIC("Mic Number", dmic_number_enum),
	SOC_ENUM_DMIC("Ref channel Number", ref_ch_number_enum),
	SOC_ENUM_DMIC("Mic Bitwidth", mic_bitwidth_enum),
	SOC_ENUM_DMIC("HPF Switch", hpf_onoff_enum),
	SOC_ENUM_DMIC("HPF Coef", hpf_config_enum),
	SOC_ENUM_DMIC("Uart enable Switch", uart_onoff_enum),
	SOC_ENUM_DMIC("Sigen Switch", sigen_onoff_enum),
	SOC_INTEGER_DMIC("Mic Gain Step", dmic_gain_enum),
};

/*
 * ============================================================================
 * Function Implementations
 * ============================================================================
 */
int _mdrv_dmic_hook_device(struct MStar_DMIC_Info *dmic_info)
{
	struct MStar_Device_Struct *dmic_chip = DMIC_Chip;
	struct snd_card *card = DMIC_SND_Card;

	int err = 0;
	int i = 0;

	if (dmic_info == NULL)
	{
		DMIC_ERR("Error! dmic_info should not be NULL!\n");
		return -EINVAL;
	}

	if ((dmic_chip == NULL) || (card == NULL))
	{
		DMIC_ERR("Error! invalid sound card!\n");
		return -EINVAL;
	}

	if (dmic_chip->pcm_capture.active_substreams > 0)
		return -EBUSY;

	// DMIC_INFO("Hal Driver '%s' is hooked.\n", dmic_info->name);
	DMIC_INFO("MStar Core Version : %s\n", dmic_info->version);

	DMIC_INFO("MStar DMIC Card number : %d\n", card->number);

	DMIC_Max_Pcm_Buffer_Size = dmic_info->max_pcm_buffer_size;

	if (dmic_info->capture_pcm_ops != NULL)
	{
		memcpy(&dmic_chip->pcm_capture.ops, dmic_info->capture_pcm_ops, sizeof(struct MStar_DMIC_Ops));
		DMIC_Capability = 1;
	}
	else
	{
		memset(&dmic_chip->pcm_capture.ops, 0x00, sizeof(struct MStar_DMIC_Ops));
	}

	/* Create a MStar PCM instance */

	if (DMIC_Capability != 0)
	{
		err = _mdrv_pcm_new(dmic_chip, DMIC_Capability);
		if (err < 0)
		{
			DMIC_ERR("Error(%d)! fail to create PCM instance No.%02d!\n", err, 0);
			_mdrv_remove();
			return err;
		}
		dmic_chip->pcm_capture.max_substreams = DMIC_Capability;
	}

	for (i = 0; i < ARRAY_SIZE(DMIC_Control_Ops); i++)
	{
		err = snd_ctl_add(card, snd_ctl_new1(&DMIC_Control_Ops[i], card));
		if (err < 0)
		{
			DMIC_ERR("Error(%d)! fail to create a control element!\n", err);
			return err;
		}
	}

	/* Register sound card to OS */
	if ((err = snd_card_register(card)) < 0)
	{
		DMIC_ERR("Error(%d)! fail to register sound card to OS!\n", err);
		_mdrv_remove();
		return err;
	}

	DMIC_INFO("MStar ALSA driver is registered to slot %d\n", card->number);

	return 0;
}

int _mdrv_dmic_unhook_device(void)
{
	if (DMIC_Chip == NULL)
		return 0;

	if (DMIC_Chip->pcm_capture.active_substreams > 0)
		return -EBUSY;

	DMIC_INFO("Unhook Hal Driver.\n");

	memset(&DMIC_Chip->pcm_capture.ops, 0x00, sizeof(struct MStar_DMIC_Ops));
	DMIC_Capability = 0;

	return 0;
}

static int _mdrv_set_default_hw(struct snd_pcm_hardware *target_hw)
{
	if (target_hw == NULL)
		return -EINVAL;

	DMIC_Capture_default_HW.buffer_bytes_max = DMIC_Max_Pcm_Buffer_Size;
	DMIC_Capture_default_HW.period_bytes_max = (DMIC_Max_Pcm_Buffer_Size >> 2);

	memset(target_hw, 0x00, sizeof(struct snd_pcm_hardware));
	memcpy(target_hw, &DMIC_Capture_default_HW, sizeof(struct snd_pcm_hardware));

	return 0;
}

static int _mdrv_set_default_constraint_list(struct snd_pcm_hw_constraint_list *target_constraint_list)
{
	if (target_constraint_list == NULL)
		return -EINVAL;

	memset(target_constraint_list, 0x00, sizeof(struct snd_pcm_hw_constraint_list));

	target_constraint_list->count = ARRAY_SIZE(DMIC_Capture_default_rates);
	target_constraint_list->list = DMIC_Capture_default_rates;

	target_constraint_list->mask = 0;

	return 0;
}

static int _mdrv_set_hw_by_constrain_list(struct snd_pcm_hardware *target_hw, struct snd_pcm_hw_constraint_list *target_constraint_list)
{
	int idx = 0;

	target_hw->rates = 0;
	for (idx = 0; idx < target_constraint_list->count; idx++)
	{
		if (target_constraint_list->list[idx] == 5512)
			target_hw->rates |= SNDRV_PCM_RATE_5512;
		else if (target_constraint_list->list[idx] == 8000)
			target_hw->rates |= SNDRV_PCM_RATE_8000;
		else if (target_constraint_list->list[idx] == 11025)
			target_hw->rates |= SNDRV_PCM_RATE_11025;
		else if (target_constraint_list->list[idx] == 12000)
			target_hw->rates |= SNDRV_PCM_RATE_12000;
		else if (target_constraint_list->list[idx] == 16000)
			target_hw->rates |= SNDRV_PCM_RATE_16000;
		else if (target_constraint_list->list[idx] == 22050)
			target_hw->rates |= SNDRV_PCM_RATE_22050;
		else if (target_constraint_list->list[idx] == 24000)
			target_hw->rates |= SNDRV_PCM_RATE_24000;
		else if (target_constraint_list->list[idx] == 32000)
			target_hw->rates |= SNDRV_PCM_RATE_32000;
		else if (target_constraint_list->list[idx] == 44100)
			target_hw->rates |= SNDRV_PCM_RATE_44100;
		else if (target_constraint_list->list[idx] == 48000)
			target_hw->rates |= SNDRV_PCM_RATE_48000;
		else if (target_constraint_list->list[idx] == 64000)
			target_hw->rates |= SNDRV_PCM_RATE_64000;
		else if (target_constraint_list->list[idx] == 88200)
			target_hw->rates |= SNDRV_PCM_RATE_88200;
		else if (target_constraint_list->list[idx] == 96000)
			target_hw->rates |= SNDRV_PCM_RATE_96000;
		else if (target_constraint_list->list[idx] == 176400)
			target_hw->rates |= SNDRV_PCM_RATE_176400;
		else if (target_constraint_list->list[idx] == 192000)
			target_hw->rates |= SNDRV_PCM_RATE_192000;
	}
	target_hw->rate_min = target_constraint_list->list[0];
	target_hw->rate_max = target_constraint_list->list[target_constraint_list->count - 1];

	return 0;
}

static int _mdrv_capture_open(struct snd_pcm_substream *substream)
{
	struct MStar_Device_Struct *dmic_chip = snd_pcm_substream_chip(substream);
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;
	struct snd_pcm_hw_constraint_list *madchip_constraints_list = &madchip_runtime->constraints_rates;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hardware madchip_hardware;
	int substream_no = substream->number;
	int err = 0;
	DMIC_INFO("Open '%s'(%d) of MStar ALSA capture device\n", substream->name, substream->number);

	mutex_lock(&madchip_runtime->mutex_lock);

	if (dmic_chip->active_capture_devices == DMIC_MAX_DEVICES)
	{
		DMIC_ERR("Error! no more availabe device, try again later\n");
		mutex_unlock(&madchip_runtime->mutex_lock);
		return -EBUSY;
	}
	else if (madchip_runtime->active_substreams == madchip_runtime->max_substreams)
	{
		DMIC_ERR("Error! no more availabe substream, try again later\n");
		mutex_unlock(&madchip_runtime->mutex_lock);
		return -EBUSY;
	}

	/* Set specified information */
	madchip_runtime->substreams[substream_no].substream = substream;
	runtime->private_data = madchip_runtime;
	runtime->private_free = NULL;

	/* Set default configurations */
	if ((err = _mdrv_set_default_hw(&madchip_hardware)) < 0)
	{
		DMIC_ERR("Error(%d)! set default hw fail!\n", err);
		mutex_unlock(&madchip_runtime->mutex_lock);
		return err;
	}

	if ((err = _mdrv_set_default_constraint_list(madchip_constraints_list)) < 0)
	{
		DMIC_ERR("Error(%d)! set default constraint list fail!\n", err);
		mutex_unlock(&madchip_runtime->mutex_lock);
		return err;
	}

	substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_CONTINUOUS;

	/* Allocate system memory for Specific Copy */
	madchip_runtime->buffer[substream_no].addr = ( unsigned char * ) vmalloc(DMIC_Max_Pcm_Buffer_Size);
	if (madchip_runtime->buffer[substream_no].addr == NULL)
	{
		DMIC_ERR("Error! fail to allocate %u bytes memory for PCM buffer!\n", DMIC_Max_Pcm_Buffer_Size);
		mutex_unlock(&madchip_runtime->mutex_lock);
		return -ENOMEM;
	}
	memset(( void * ) madchip_runtime->buffer[substream_no].addr, 0x00, DMIC_Max_Pcm_Buffer_Size);
	madchip_runtime->buffer[substream_no].size = DMIC_Max_Pcm_Buffer_Size;

	/* Fill in some settings */
	memcpy(&runtime->hw, &madchip_hardware, sizeof(struct snd_pcm_hardware));
	if ((err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, madchip_constraints_list)) < 0)
	{
		DMIC_ERR("Error(%d)! fail to set hw constraint list!\n", err);
		mutex_unlock(&madchip_runtime->mutex_lock);
		return err;
	}

	if (madchip_runtime->active_substreams == 0)
	{
		dmic_chip->active_capture_devices++;
	}
	madchip_runtime->active_substreams++;

	mutex_unlock(&madchip_runtime->mutex_lock);

	return 0;
}

static int _mdrv_capture_close(struct snd_pcm_substream *substream)
{
	struct MStar_Device_Struct *dmic_chip = snd_pcm_substream_chip(substream);
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;
	int substream_no = substream->number;
	DMIC_INFO("Close '%s'(%d) of MStar ALSA capture device\n", substream->name, substream->number);

	mutex_lock(&madchip_runtime->mutex_lock);

	if (dmic_chip->active_capture_devices == 0)
	{
		DMIC_ERR("Error! no device is in active\n");
		mutex_unlock(&madchip_runtime->mutex_lock);
		return -ENODEV;
	}
	else if (madchip_runtime->active_substreams == 0)
	{
		DMIC_ERR("Error! no substream is in active\n");
		mutex_unlock(&madchip_runtime->mutex_lock);
		return -ENODEV;
	}

	madchip_runtime->active_substreams--;

	/* Reset some variables */
	madchip_runtime->substreams[substream_no].substream_status = E_STOP;
	madchip_runtime->substreams[substream_no].substream = NULL;

	/* Free allocated memory */
	if (madchip_runtime->buffer[substream_no].addr != NULL)
	{
		vfree(madchip_runtime->buffer[substream_no].addr);
		madchip_runtime->buffer[substream_no].addr = NULL;
	}

	if (madchip_runtime->active_substreams == 0)
	{
		dmic_chip->active_capture_devices--;

		memset(&madchip_runtime->monitor, 0x00, sizeof(struct MStar_Monitor_Struct));

		/* Reset PCM configurations */
		madchip_runtime->sample_rate = 0;
		madchip_runtime->channel_mode = 0;
		madchip_runtime->runtime_status = E_STOP;
		madchip_runtime->device_status = DMIC_FALSE;

		/* Stop MStar Audio DSP */
		if (madchip_runtime->ops.stop)
		{
			madchip_runtime->ops.stop();
			mdelay(2);
		}

		/* Close MStar Audio DSP */
		if (madchip_runtime->ops.close)
		{
			madchip_runtime->ops.close();
			mdelay(2);
		}
	}

	mutex_unlock(&madchip_runtime->mutex_lock);

	return 0;
}

static int _mdrv_capture_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	int err = 0;
	// DMIC_INFO("Configure PCM HW resource\n");

	err = snd_pcm_lib_malloc_pages(substream, ( size_t ) DMIC_Max_Pcm_Buffer_Size);
	if (err < 0)
	{
		DMIC_ERR("Error(%d)! fail to allocate %u bytes for PCM HW resource !\n", err, DMIC_Max_Pcm_Buffer_Size);
	}

	return err;
}

static int _mdrv_capture_hw_free(struct snd_pcm_substream *substream)
{
	// DMIC_INFO("Free PCM HW resource\n");
	return snd_pcm_lib_free_pages(substream);
}

static int _mdrv_capture_prepare(struct snd_pcm_substream *substream)
{
	struct MStar_Device_Struct *dmic_chip = snd_pcm_substream_chip(substream);
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;
	struct snd_pcm_runtime *runtime = substream->runtime;
	static unsigned int warning_counter = 0;
	static unsigned int bit_width = 0;
	DMIC_INFO("Reset and prepare PCM resource\n");

	mutex_lock(&madchip_runtime->mutex_lock);

	if (madchip_runtime->ops.get)
	{
		madchip_runtime->ops.get(E_PCM_CAPTURE_GET_DEVICE_STATUS, &madchip_runtime->device_status);
		if (madchip_runtime->device_status != DMIC_TRUE)
		{
			if (((++warning_counter) % 10) == 1)
				DMIC_ERR("Warning! Capture device is not ready yet, try again later!\n");

			mutex_unlock(&madchip_runtime->mutex_lock);

			return 0;
		}
		warning_counter = 0;
	}
	else
	{
		madchip_runtime->device_status = DMIC_TRUE;
	}

	/* Configure MStar Audio DSP channel mode */
	madchip_runtime->channel_mode = runtime->channels;
	if (madchip_runtime->ops.set)
	{
		madchip_runtime->ops.set(E_PCM_CAPTURE_SET_CHANNEL_MODE, &runtime->channels);
	}

	/* Configure MStar Audio DSP sample rate */
	madchip_runtime->sample_rate = runtime->rate;
	if (madchip_runtime->ops.set)
	{
		madchip_runtime->ops.set(E_PCM_CAPTURE_SET_SAMPLE_RATE, &runtime->rate);
	}

	/* Configure MStar Audio DSP bit width */
	madchip_runtime->sample_rate = runtime->rate;
	if (madchip_runtime->ops.set)
	{
		bit_width = _mdrv_convert_format_to_bitwidth(substream);
		madchip_runtime->ops.set(E_PCM_CAPTURE_SET_BIT_WIDTH, &bit_width);
	}

	if (madchip_runtime->runtime_status != E_START)
	{
		/* Open MStar Audio DSP */
		if (madchip_runtime->ops.open)
		{
			madchip_runtime->ops.open();
		}

		/* Start MStar Audio DSP */
		if (madchip_runtime->ops.start)
		{
			madchip_runtime->ops.start();
		}

		/* Configure MStar Audio DSP device status */
		madchip_runtime->runtime_status = E_START;
	}
	else
	{
		/* Start MStar Audio DSP */
		if (madchip_runtime->ops.start)
		{
			madchip_runtime->ops.start();
		}
	}

	mutex_unlock(&madchip_runtime->mutex_lock);

	return 0;
}

static int _mdrv_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct MStar_Device_Struct *dmic_chip = snd_pcm_substream_chip(substream);
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;
	int substream_no = substream->number;
	int err = 0;

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_STOP:
		{
			// DMIC_INFO("SNDRV_PCM_TRIGGER_STOP command is received\n");
			madchip_runtime->substreams[substream_no].substream_status = E_STOP;
			break;
		}

		case SNDRV_PCM_TRIGGER_START:
		{
			// DMIC_INFO("SNDRV_PCM_TRIGGER_START command is received\n");
			madchip_runtime->substreams[substream_no].substream_status = E_START;
			break;
		}

		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		{
			// DMIC_INFO("SNDRV_PCM_TRIGGER_PAUSE_PUSH command is received\n");
			madchip_runtime->substreams[substream_no].substream_status = E_PAUSE;
			break;
		}

		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		{
			// DMIC_INFO("SNDRV_PCM_TRIGGER_PAUSE_RELEASE command is received\n");
			madchip_runtime->substreams[substream_no].substream_status = E_PAUSE_RELEASE;
			break;
		}

		case SNDRV_PCM_TRIGGER_SUSPEND:
		{
			// DMIC_INFO("SNDRV_PCM_TRIGGER_SUSPEND command is received\n");
			madchip_runtime->substreams[substream_no].substream_status = E_SUSPEND;
			break;
		}

		case SNDRV_PCM_TRIGGER_RESUME:
		{
			// DMIC_INFO("SNDRV_PCM_TRIGGER_RESUME command is received\n");
			madchip_runtime->substreams[substream_no].substream_status = E_RESUME;
			break;
		}

		default:
		{
			DMIC_INFO("Invalid capture's trigger command %d\n", cmd);
			err = -EINVAL;
			break;
		}
	}

	return err;
}

static snd_pcm_uframes_t _mdrv_capture_pointer(struct snd_pcm_substream *substream)
{
	struct MStar_Device_Struct *dmic_chip = snd_pcm_substream_chip(substream);
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int new_avail_bytes = 0;
	snd_pcm_uframes_t pcm_buffer_level = 0;
	snd_pcm_uframes_t new_pointer = 0;
	//DMIC_INFO("%s() is invoked\n", __FUNCTION__);



	if (madchip_runtime->ops.get)
	{
		madchip_runtime->ops.get(E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES, &new_avail_bytes);
	}
	else
	{
		new_avail_bytes += frames_to_bytes(runtime, runtime->periods);
	}

	pcm_buffer_level = bytes_to_frames(runtime, new_avail_bytes);
	new_pointer = runtime->status->hw_ptr + pcm_buffer_level;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)) /* Since ALSA Kernel Module v1.0.20 */
	new_pointer = new_pointer % runtime->buffer_size;
#endif

	//DMIC_INFO("%s() hw_ptr=0x%08x, apl_ptr=0x%08x , new_ptr=0x%08x\n", __FUNCTION__, runtime->status->hw_ptr, runtime->status->hw_ptr, new_pointer);
	return new_pointer;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 53)
static int _mdrv_capture_copy(struct snd_pcm_substream *substream, int channel, unsigned long pos, void __user *buf, unsigned long count)
#else
static int _mdrv_capture_copy(struct snd_pcm_substream *substream, int channel, snd_pcm_uframes_t pos, void __user *buf, snd_pcm_uframes_t count)
#endif
{
	struct MStar_Device_Struct *dmic_chip = snd_pcm_substream_chip(substream);
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned char *buffer_tmp = NULL;
	unsigned int request_size = 0;
	unsigned int copied_size = 0;
	unsigned int size = 0;
	int substream_no = substream->number;
	unsigned long ret = 0;
	static unsigned int warning_counter = 0;
	//DMIC_INFO("%s() is invoked\n", __FUNCTION__);

	if (buf == NULL)
	{
		DMIC_ERR("Error! invalid memory address!\n");
		return -EINVAL;
	}

	if (count == 0)
	{
		DMIC_ERR("Error! request bytes is zero!\n");
		return -EINVAL;
	}

	/* Ensure device status is up to date */
	if (madchip_runtime->ops.get)
	{
		madchip_runtime->ops.get(E_PCM_CAPTURE_GET_DEVICE_STATUS, &madchip_runtime->device_status);
	}

	if ((madchip_runtime->device_status != DMIC_TRUE)
		|| ((madchip_runtime->runtime_status == E_STOP) || (madchip_runtime->runtime_status == E_SUSPEND)))
	{
		if (((++warning_counter) % 10) == 1)
			DMIC_ERR("Warning! Capture device is not ready yet, try again later!\n");
		return -EAGAIN;
	}

	if (madchip_runtime->runtime_status == E_RESUME)
	{
		_mdrv_capture_resume();
		if (((++warning_counter) % 10) == 1)
			DMIC_ERR("Warning! Capture device is still in resume state, try again later!\n");
		return -EAGAIN;
	}

	warning_counter = 0;

	buffer_tmp = madchip_runtime->buffer[substream_no].addr;
	if (buffer_tmp == NULL)
	{
		DMIC_ERR("Error! need to allocate system memory for PCM buffer!\n");
		return -ENXIO;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 53)
	request_size = count;
#else
	request_size = frames_to_bytes(runtime, count);
#endif

	if (madchip_runtime->ops.get)
	{
		madchip_runtime->ops.get(E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES, &madchip_runtime->buffer[substream_no].avail_size);
	}
	else
	{
		madchip_runtime->buffer[substream_no].avail_size = request_size;
	}

	if (((substream->f_flags & O_NONBLOCK) > 0) && (request_size > madchip_runtime->buffer[substream_no].avail_size))
	{
		DMIC_ERR("Error! available PCM is %u bytes, but request %u bytes\n",
				   madchip_runtime->buffer[substream_no].avail_size, request_size);
		return -EAGAIN;
	}

	do
	{
		if (madchip_runtime->ops.read)
		{
			/* Receive PCM data */
			size = madchip_runtime->ops.read(buffer_tmp, (request_size - copied_size));
			ret = copy_to_user((buf + copied_size), buffer_tmp, size);
			copied_size += size;
		}
		else
		{
			copied_size = request_size;
		}
	} while (copied_size < request_size);

	return 0;
}

static int _mdrv_capture_suspend(void)
{
	DMIC_INFO("%s() is invoked\n", __FUNCTION__);
	struct MStar_Runtime_Struct *madchip_runtime = NULL;
	madchip_runtime = &DMIC_Chip->pcm_capture;

	if (madchip_runtime == NULL)
		return -EINVAL;

	if (madchip_runtime->runtime_status == E_STOP)
	{
		return -EINVAL;
	}

	mutex_lock(&madchip_runtime->mutex_lock);

	if (madchip_runtime->ops.stop)
	{
		madchip_runtime->ops.stop();
	}

	madchip_runtime->runtime_status = E_SUSPEND;
	madchip_runtime->device_status = DMIC_FALSE;

	if (madchip_runtime->ops.suspend)
	{
		madchip_runtime->ops.suspend();
	}

	mutex_unlock(&madchip_runtime->mutex_lock);

	return 0;
}

static int _mdrv_capture_resume(void)
{
	DMIC_INFO("%s() is invoked\n", __FUNCTION__);

	struct MStar_Runtime_Struct *madchip_runtime = NULL;
	madchip_runtime = &DMIC_Chip->pcm_capture;

	//DMIC_INFO("substream runtime status state = %d\n", madchip_runtime->substreams[0].substream->runtime->status->state);

	if (madchip_runtime == NULL)
		return -EINVAL;

    if (madchip_runtime->ops.init)
	{
		madchip_runtime->ops.init();
	}

	if (madchip_runtime->runtime_status == E_STOP)
	{
		return -EINVAL;
	}

	mutex_lock(&madchip_runtime->mutex_lock);
	madchip_runtime->device_status = DMIC_TRUE;
	madchip_runtime->runtime_status = E_RESUME;

	if (madchip_runtime->ops.resume)
	{
		madchip_runtime->ops.resume();
	}

	if (madchip_runtime->ops.open)
	{
		madchip_runtime->ops.open();
	}

	if (madchip_runtime->ops.start)
	{
		madchip_runtime->ops.start();
	}

	madchip_runtime->runtime_status = E_START;

	mutex_unlock(&madchip_runtime->mutex_lock);

	return 0;
}

static unsigned int _mdrv_convert_format_to_bitwidth(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int bitwidth = 0;

	switch (runtime->format)
	{
		case SNDRV_PCM_FORMAT_S16_LE:
		case SNDRV_PCM_FORMAT_S16_BE:
		case SNDRV_PCM_FORMAT_U16_LE:
		case SNDRV_PCM_FORMAT_U16_BE:
		{
			bitwidth = 16;
			break;
		}

		case SNDRV_PCM_FORMAT_S32_LE:
		case SNDRV_PCM_FORMAT_S32_BE:
		case SNDRV_PCM_FORMAT_U32_LE:
		case SNDRV_PCM_FORMAT_U32_BE:
		{
			bitwidth = 32;
			break;
		}

		default:
		{
			DMIC_ERR("un-supported runtime format[%d], reset bit width to 16\n", runtime->format);
			bitwidth = 16;
			break;
		}
	}

	return bitwidth;
}

static int _mdrv_control_mic_gain_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x1e0;

	return 0;
}

static int _mdrv_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = ( struct soc_enum * ) kcontrol->private_value;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
	snd_ctl_enum_info(uinfo, 1, e->items, e->texts);
#else
	snd_ctl_enum_info(uinfo, 1, e->max, e->texts);
#endif

	return 0;
}

static int _mdrv_pcm_new(struct MStar_Device_Struct *dmic_chip, int capture_substreams)
{
	struct snd_card *card = dmic_chip->card;
	struct snd_pcm *pcm = NULL;
	int err = 0;

	/* Create a PCM instance */
	if ((err = snd_pcm_new(card, _DMIC_PCM_INSTANCE, 0, 0, capture_substreams, &pcm)) < 0)
	{
		DMIC_ERR("Error(%d)! fail to create a PCM instance!\n", err);
		return err;
	}

	/* Set operations to the PCM instance */
	if (capture_substreams > 0)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &DMIC_Capture_Ops);

	/* Fill in MStar device data structure */
	pcm->info_flags = 0;
	pcm->private_data = dmic_chip;
	dmic_chip->pcm = pcm;

	/* Fill in MStar ALSA information (PCM) */
	snprintf(pcm->name, sizeof(pcm->name), "%s CARD%d No.%02d", _DMIC_PCM_NAME, card->number, 0); /* 80 bytes */

	/* Pre-allocate DMA memory to all substreams of the given pcm */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data(GFP_KERNEL),
										  DMIC_Max_Pcm_Buffer_Size, DMIC_Max_Pcm_Buffer_Size);

	return 0;
}

unsigned int _mdrv_get_mic_num(void)
{
	switch (card_reg[DMIC_REG_DMIC_NUMBER])
	{
		case 2: // DMIC_CHANNEL_2
			return 0;
		case 4: // DMIC_CHANNEL_4
			return 1;
		case 6: // DMIC_CHANNEL_6
			return 2;
		case 8: // DMIC_CHANNEL_8
			return 3;
		default:
			return 2;
	}
}

unsigned int _mdrv_get_ref_num(void)
{
	switch (card_reg[DMIC_REG_REF_CH_NUMBER])
	{
		case 0:
			return 0;
		case 2: // REF_CHANNEL_2
			return 1;
		case 4: // REF_CHANNEL_4
			return 2;
		default:
			return 0;
	}
}

unsigned int _mdrv_kcontrol_read(struct snd_card *card, unsigned int reg)
{
	int ret = 0;

	if (reg >= DMIC_REG_LEN)
		return -1;

	switch (reg)
	{
		case DMIC_REG_DMIC_NUMBER:
			ret = _mdrv_get_mic_num();
			break;
		case DMIC_REG_REF_CH_NUMBER:
			ret = _mdrv_get_ref_num();
			break;

		case DMIC_REG_DMIC_ONOFF:
		case DMIC_REG_VQ_ONOFF:
		case DMIC_REG_SEAMLESS_ONOFF:
		case DMIC_REG_DA_ONOFF:
		case DMIC_REG_SIGEN_ONOFF:
		case DMIC_REG_DMIC_GAIN:
		case DMIC_REG_MIC_BITWIDTH:
		case DMIC_REG_AEC_MODE:
		case DMIC_REG_I2S_ONOFF:
		case DMIC_REG_HW_AEC_ONOFF:
		case DMIC_REG_HPF_ONOFF:
		case DMIC_REG_HPF_CONFIG:
		case DMIC_REG_UART_ONOFF:
			ret = card_reg[reg];
			break;
		default:
			ret = card_reg[reg];
			break;
	}

	return ret;
}

int _mdrv_kcontrol_write(struct snd_kcontrol *kcontrol, unsigned int reg, unsigned int value)
{
	struct snd_card *card = snd_kcontrol_chip(kcontrol);
	struct MStar_Device_Struct *dmic_chip = card->private_data;
	struct MStar_Runtime_Struct *madchip_runtime = &dmic_chip->pcm_capture;

	if ((reg != DMIC_REG_DMIC_ONOFF) && !card_reg[DMIC_REG_DMIC_ONOFF])
	{
		DMIC_ERR("%s: dmic not power on, reg = 0x%x, val = 0x%x\n", __FUNCTION__, reg, value);
		return -1;
	}

	DMIC_DEBUG("%s: reg = 0x%x, val = 0x%x\n", __FUNCTION__, reg, value);
	switch (reg)
	{
		case DMIC_REG_DMIC_ONOFF:
			if (value > 1 || value < 0)
			{
				DMIC_ERR("Invalid value %d, should be 0/1 \n", value);
				return -1;
			}

			if (value)
			{
				DMIC_ERR("dmic enable \n");
			}
			else
			{
				DMIC_ERR("dmic disable \n");
			}

			break;

		case DMIC_REG_VQ_ONOFF:
			DMIC_ERR("vg is not support\n");
			break;

		case DMIC_REG_SEAMLESS_ONOFF:
            DMIC_ERR("Seamless is %d !!!\n", value);

            struct MStar_Runtime_Struct *madchip_runtime = NULL;
            madchip_runtime = &DMIC_Chip->pcm_capture;

            if (madchip_runtime == NULL)
                return -1;

            if (madchip_runtime->ops.set)
            {
                madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_DSP_SUPPORT, &value);
            }

            if(bIsNeedReInit)
            {
                DMIC_ERR(" Reinit dmic driver !!!\n");
                if (madchip_runtime->ops.init)
                {
                    madchip_runtime->ops.init();
                }
                bIsNeedReInit = DMIC_FALSE;
            }

			break;

		case DMIC_REG_DA_ONOFF:
			DMIC_ERR("data arrange is not support\n");
			break;

		case DMIC_REG_SIGEN_ONOFF:
			if (value)
			{
				DMIC_ERR("ouput sine tone \n");
			}
			else
			{
				DMIC_ERR("don't ouput sine tone \n");
			}

			if (value > 1 || value < 0)
			{
				DMIC_ERR("Invalid value %d, should be 0/1 \n", value);
				return -1;
			}

			if (madchip_runtime->ops.set)
			{
				madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_SINE_GEN, &value);
			}

			break;

		case DMIC_REG_DMIC_NUMBER:
			DMIC_ERR("set dmic number: %d \n", value);

			if (value > 8 || value < 0 || (value % 2))
			{
				DMIC_ERR("Invalid value %d, should be 2/4/6/8 \n", value);
				return -1;
			}

			if (madchip_runtime->ops.set)
			{
				madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_NUM, &value);
			}

			break;

		case DMIC_REG_REF_CH_NUMBER:
			DMIC_ERR("set ref channel number: %d \n", value);

			if (value > 4 || value < 0 || (value % 2))
			{
				DMIC_ERR("Invalid value %d, should be 0/2/4 \n", value);
				return -1;
			}

			if (madchip_runtime->ops.set)
			{
				madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_REF_NUM, &value);
			}

			break;

		case DMIC_REG_DMIC_GAIN:
			DMIC_ERR("set dmic gain %d \n", value);

			if (value > 480 || value < 0)
			{
				DMIC_ERR("Invalid value %d, should be 0~480 \n", value);
				return -1;
			}

			if (madchip_runtime->ops.set)
			{
				madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_GAIN, &value);
			}

			break;

		case DMIC_REG_MIC_BITWIDTH:
			DMIC_ERR("set bit width is not support \n");
			break;

		case DMIC_REG_AEC_MODE:
			DMIC_ERR("set AEC mode is not support\n");
			break;

		case DMIC_REG_I2S_ONOFF:
			DMIC_ERR("set I2S is not support\n");
			break;

		case DMIC_REG_HW_AEC_ONOFF:
			DMIC_ERR("set HW AEC is not support\n");
			break;

		case DMIC_REG_HPF_ONOFF:
			DMIC_ERR("enable/disable High pass filter : %d \n", value);

			if (value > 2 || value < 0)
			{
				DMIC_ERR("Invalid value %d, should be 0~2 \n", value);
				return -1;
			}

			if (madchip_runtime->ops.set)
			{
				madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_HPF_SWITCH, &value);
			}

			break;

		case DMIC_REG_HPF_CONFIG:
		{
			DMIC_ERR("set HPF param: %d \n", value);

			if (value > 12 || value < 0)
			{
				DMIC_ERR("Invalid value %d, should be 0~12 \n", value);
				return -1;
			}

			if (madchip_runtime->ops.set)
			{
				madchip_runtime->ops.set(E_PCM_CAPTURE_SET_DMIC_HPF_CONFIG, &value);
			}

			break;
		}

		case DMIC_REG_UART_ONOFF:
			DMIC_ERR("enable/disable Uart is not support\n");
			break;

		default:
			DMIC_ERR("%s: error parameter, reg = 0x%x, val = 0x%x\n", __FUNCTION__, reg, value);
			return -1;
	}

	card_reg[reg] = value;
	return 1;
}

static int _mdrv_kcontrol_get_enum(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = ( struct soc_enum * ) kcontrol->private_value;

	ucontrol->value.integer.value[0] = _mdrv_kcontrol_read(card, e->reg);

	return 0;
}

static int _mdrv_kcontrol_put_enum(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = ( struct soc_enum * ) kcontrol->private_value;
	return _mdrv_kcontrol_write(kcontrol, e->reg, ucontrol->value.integer.value[0]);
}

static int _mdrv_system_probe(struct platform_device *dev)
{
	DMIC_INFO("%s() is invoked\n", __FUNCTION__);
	return 0;
}

static int _mdrv_system_remove(struct platform_device *dev)
{
	DMIC_INFO("%s() is invoked\n", __FUNCTION__);
	return 0;
}

static int _mdrv_system_suspend(struct platform_device *dev, pm_message_t state)
{
	DMIC_INFO("%s() is invoked\n", __FUNCTION__);
	if (DMIC_Chip == NULL)
		return -EINVAL;

	_mdrv_capture_suspend();

	return 0;
}

static int _mdrv_system_resume(struct platform_device *dev)
{
	DMIC_INFO("%s() is invoked\n", __FUNCTION__);

	if (DMIC_Chip == NULL)
		return -EINVAL;

	_mdrv_capture_resume();

	return 0;
}

static int _mdrv_probe(void)
{
	int err = 0;

	/* Create a sound card */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)) /* Since ALSA Kernel Module v3.18.0 */
	if ((err = snd_card_new(&platform_bus, -1, NULL, THIS_MODULE, sizeof(struct MStar_Device_Struct), &DMIC_SND_Card)) != 0)
	{
		DMIC_ERR("Error(%d)! fail to create a sound card!\n", err);
		return err;
	}
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)) /* Since ALSA Kernel Module v1.0.19 */
	if ((err = snd_card_create(-1, NULL, THIS_MODULE, sizeof(struct MStar_Device_Struct), &DMIC_SND_Card)) != 0)
	{
		DMIC_ERR("Error(%d)! fail to create a sound card!\n", err);
		return err;
	}
#else
	DMIC_SND_Card = snd_card_new(-1, NULL, THIS_MODULE, sizeof(struct MStar_Device_Struct));
	if (DMIC_SND_Card == NULL)
	{
		DMIC_ERR("Error! fail to create a sound card!\n");
		return -EPERM;
	}
#endif

	snprintf(DMIC_SND_Card->driver, sizeof(DMIC_SND_Card->driver), "%s", _DMIC_SND_DRIVER); /* 16 bytes */
	snprintf(DMIC_SND_Card->shortname, sizeof(DMIC_SND_Card->shortname), "%s-No%02d", _DMIC_SND_SHORTNAME, DMIC_SND_Card->number); /* 32 bytes */
	snprintf(DMIC_SND_Card->longname, sizeof(DMIC_SND_Card->longname), "%s-No%02d", _DMIC_SND_LONGNAME, DMIC_SND_Card->number); /* 80 bytes */

	DMIC_INFO("Go to register MStar ALSA driver %d ID:%s\n", DMIC_SND_Card->number, DMIC_SND_Card->id);
	DMIC_Chip = DMIC_SND_Card->private_data;
	memset(DMIC_Chip, 0x00, sizeof(struct MStar_Device_Struct));

	DMIC_Chip->card = DMIC_SND_Card;

	/* Initiate Mutex */
	mutex_init(&DMIC_Chip->pcm_capture.mutex_lock);

	return 0;
}

static int _mdrv_remove(void)
{
	int substream_no = 0;
	int err = 0;
	struct snd_card *card = DMIC_SND_Card;
	DMIC_INFO("Remove MStar ALSA driver from slot %d\n", DMIC_SND_Card->number);

	/* Unhook device operatos */
	_mdrv_dmic_unhook_device();

	/* Free a created MStar device and allocated memory */
	if (DMIC_Chip != NULL)
	{
		mutex_destroy(&DMIC_Chip->pcm_capture.mutex_lock);
		if (DMIC_Chip->pcm_capture.buffer[substream_no].addr != NULL)
		{
			vfree(DMIC_Chip->pcm_capture.buffer[substream_no].addr);
			DMIC_Chip->pcm_capture.buffer[substream_no].addr = NULL;
		}

		DMIC_Chip->card = NULL;
		DMIC_Chip->pcm = NULL;
		DMIC_Chip = NULL;
		kfree(DMIC_Chip);

		DMIC_SND_Card->private_data = NULL;
		DMIC_SND_Card = NULL;
	}

	/* Free a created sound card */
	if ((err = snd_card_free(card)) < 0)
		DMIC_ERR("Error(%d)! fail to free allocated sound card!\n", err);

	return err;
}

static int __init _mdrv_dmic_init(void)
{
	int err = 1;

	if (driver_has_installed == DMIC_TRUE)
	{
		DMIC_ERR("MStar ALSA driver is already installed!\n");
	}
	else
	{
		DMIC_INFO("Linux Kernel Version : %d.%d.%d\n", ((LINUX_VERSION_CODE >> 16) & 0xFF),
				   ((LINUX_VERSION_CODE >> 8) & 0xFF), (LINUX_VERSION_CODE & 0xFF));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 1))
		DMIC_INFO("ALSA Kernel Version : %s\n", "XX.XX.XX");
#else
		DMIC_INFO("ALSA Kernel Version : %s\n", CONFIG_SND_VERSION);
#endif
		DMIC_INFO("MStar ALSA Driver Version : %d.%d.%d\n", _DMIC_DRIVER_VERSION_MAJOR,
				   _DMIC_DRIVER_VERSION_MINOR, _DMIC_DRIVER_VERSION_REVISION);
		DMIC_INFO("Initiate MStar ALSA driver engine\n");

		err = _mdrv_probe();
		if (err == 0)
		{
			/* Only need to register one device to ALSA Kernel Moduel */
			driver_has_installed = DMIC_TRUE;
		}
		else
		{
			DMIC_ERR("Error(%d)! fail to _mdrv_probe!\n", err);
		}

		err = platform_driver_register(&mstar_dmic_system_driver);

		if (err < 0)
		{
			DMIC_ERR("Error(%d)! fail to register to platform driver system!\n", err);
		}
	}
	return err;
}

static void __exit _mdrv_dmic_exit(void)
{
	int err = 0;

	if (driver_has_installed == DMIC_TRUE)
	{
		DMIC_INFO("Exit MStar ALSA driver engine\n");

		err = _mdrv_remove();
		driver_has_installed = DMIC_FALSE;

		platform_driver_unregister(&mstar_dmic_system_driver);
	}

	return;
}

/*
 * ============================================================================
 * Module Information
 * ============================================================================
 */
module_init(_mdrv_dmic_init);
module_exit(_mdrv_dmic_exit);

MODULE_AUTHOR("MStar Semiconductor, Inc.");
MODULE_DESCRIPTION("MStar DMIC Driver - DRV Layer");
MODULE_SUPPORTED_DEVICE("DMIC DEVICE");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(_mdrv_dmic_hook_device);
EXPORT_SYMBOL(_mdrv_dmic_unhook_device);
