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

//------------------------------------------------------------------------------
//  Include Files
//------------------------------------------------------------------------------

#include <linux/module.h>
#include <linux/device.h>
#include <sound/soc.h>

#include "drvVoc.h"
#include "voc_pcm.h"
#include "voc_debug.h"

//------------------------------------------------------------------------------
//  Macros
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//  Variables
//------------------------------------------------------------------------------
static struct voc_pcm_dma_data voc_pcm_dma_wr =
{
    .name		= "DMA writer",
    .channel	= 0,
};

//------------------------------------------------------------------------------
//  Function
//------------------------------------------------------------------------------
#if 0
static int voc_soc_dai_ops_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static void voc_soc_dai_ops_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
}

static int voc_soc_dai_ops_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static int voc_soc_dai_ops_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static int voc_soc_dai_ops_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static int voc_soc_dai_ops_hw_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static int voc_soc_dai_ops_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static int voc_soc_dai_ops_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static int voc_soc_dai_ops_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}

static struct snd_soc_dai_ops voc_soc_cpu_dai_ops =
{
    .set_sysclk = voc_soc_dai_ops_set_sysclk,
    .set_pll    = NULL,
    .set_clkdiv = voc_soc_dai_ops_set_clkdiv,

    .set_fmt    = voc_soc_dai_ops_set_fmt,

    .startup		= voc_soc_dai_ops_startup,
    .shutdown		= voc_soc_dai_ops_shutdown,
    .trigger		= voc_soc_dai_ops_trigger,
    .prepare      = voc_soc_dai_ops_prepare,
    .hw_params	= voc_soc_dai_ops_hw_params,
    .hw_free      = voc_soc_dai_ops_hw_free,
};
#endif

static int voc_soc_dai_probe(struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    dai->capture_dma_data = (void *)&voc_pcm_dma_wr;
    return 0;
}

static int voc_soc_dai_remove(struct snd_soc_dai *dai)
{
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    return 0;
}


static int voc_soc_dai_suspend(struct snd_soc_dai *dai)
{
 struct snd_pcm_substream *substream;
    struct snd_pcm_runtime *runtime;
    struct snd_soc_pcm_runtime *rtd;
    struct snd_soc_card *card = dai->component->card;
    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);

    list_for_each_entry(rtd, &card->rtd_list, list) {

		if (rtd->dai_link->ignore_suspend)
			continue;

        substream = rtd->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
        if(substream)
        {
            /* FIXME: the open/close code should lock this as well */
            if (substream->runtime == NULL)
                continue;

            runtime = substream->runtime;

            VOC_PRINTF(TRACE_LEVEL, "%s: VocDmaInitChannel\n", __FUNCTION__);
            VOC_PRINTF(TRACE_LEVEL, "params_channels     = %d\n", runtime->channels);
            VOC_PRINTF(TRACE_LEVEL, "params_rate         = %d\n", runtime->rate);
            VOC_PRINTF(TRACE_LEVEL, "params_sample_width = %d\n", snd_pcm_format_physical_width(runtime->format));

            // reconfigure CM4 after updating image
            VocDmaInitChannel((U32)(runtime->dma_addr - MIU_BASE),
                              runtime->dma_bytes,
                              runtime->channels,
                              snd_pcm_format_physical_width(runtime->format),
                              runtime->rate);
        }
    }
    return 0;
}

extern bool voc_soc_is_seamless_enabled(void);

static int voc_soc_dai_resume(struct snd_soc_dai *dai)
{
    struct snd_pcm_substream *substream;
    struct snd_pcm_runtime *runtime;
    struct voc_pcm_runtime_data *prtd;
    //struct voc_pcm_dma_data *dma_data;
    struct snd_soc_pcm_runtime *rtd;
    //struct snd_soc_codec *codec;
    struct snd_soc_card *card = dai->component->card;

    VOC_PRINTF(TRACE_LEVEL, "%s: dai = %s\n", __FUNCTION__, dai->name);
    VocDmaReset();
    VOC_PRINTF(DEBUG_LEVEL, "Dma reset\n");


    list_for_each_entry(rtd, &card->rtd_list, list) {

		if (rtd->dai_link->ignore_suspend)
			continue;

        for (substream = rtd->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
                substream; substream = substream->next)
        {
            if (substream->runtime == NULL)
            {
                //when seamless mode is enabled, we have to reset CM4 audio if recording isn't required.
                if(voc_soc_is_seamless_enabled())
                    VocDmaResetAudio();
                continue;
            }

            runtime = substream->runtime;
            prtd = runtime->private_data;

            if (prtd && prtd->dma_data)
            {
                //memset(runtime->dma_area, 0, runtime->dma_bytes);
                prtd->dma_level_count = 0;
                prtd->state = DMA_EMPTY; //EMPTY
            }
            if(VocIsCm4ShutDown())
            {
                VOC_PRINTF(TRACE_LEVEL, "%s: VocDmaInitChannel\n", __FUNCTION__);
                // reconfigure CM4 after updating image
                VocDmaInitChannel((U32)(runtime->dma_addr - MIU_BASE),
                                runtime->dma_bytes,
                                runtime->channels,
                                snd_pcm_format_physical_width(runtime->format),
                                runtime->rate);
            }
        }
    }

    return 0;
}

struct snd_soc_dai_driver voc_soc_cpu_dai_drv =
{
    .probe				= voc_soc_dai_probe,
    .remove             = voc_soc_dai_remove,
    .suspend			= voc_soc_dai_suspend,
    .resume				= voc_soc_dai_resume,

    .capture			=
    {
        .channels_min	= 2,
        .channels_max	= 8,
        .rates			= SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
        .formats		= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
    },
//  .ops				= &voc_soc_cpu_dai_ops,
};

static const struct snd_soc_component_driver voc_soc_component =
{
    .name		= "voc-pcm",
};

static int voc_cpu_dai_probe(struct platform_device *pdev)
{
    int ret;

    VOC_PRINTF(TRACE_LEVEL, "%s enter\r\n", __FUNCTION__);

    ret = snd_soc_register_component(&pdev->dev, &voc_soc_component, &voc_soc_cpu_dai_drv, 1);
    if (ret)
    {
        return ret;
    }

    return 0;
}

static int voc_cpu_dai_remove(struct platform_device *pdev)
{
    VOC_PRINTF(TRACE_LEVEL, "%s enter\r\n", __FUNCTION__);
    snd_soc_unregister_component(&pdev->dev);
    return 0;
}


static struct platform_driver voc_cpu_dai_driver =
{
    .probe = voc_cpu_dai_probe,
    .remove = (voc_cpu_dai_remove),
    .driver = {
        .name = "voc-cpu-dai",
        .owner = THIS_MODULE,
    },
};


static struct platform_device *voc_cpu_dai_device = NULL;
static int __init voc_cpu_dai_init(void)
{

    int ret = 0;

    struct device_node *np;

    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__)

    voc_cpu_dai_device = platform_device_alloc("voc-cpu-dai", -1);
    if (!voc_cpu_dai_device)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: platform_device_alloc voc-cpu-dai error\r\n", __FUNCTION__);
        return -ENOMEM;
    }
#if CONFIG_OF
    np = of_find_compatible_node(NULL, NULL, "mstar,voc-audio");
    if (np)
    {
        voc_cpu_dai_device->dev.of_node = of_node_get(np);
        of_node_put(np);
    }
#endif
    ret = platform_device_add(voc_cpu_dai_device);
    if (ret)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: platform_device_add voc_cpu_dai_device error\r\n", __FUNCTION__);
        platform_device_put(voc_cpu_dai_device);
    }

    ret = platform_driver_register(&voc_cpu_dai_driver);
    if (ret)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: platform_driver_register voc_cpu_dai_driver error\r\n", __FUNCTION__);
        platform_device_unregister(voc_cpu_dai_device);
    }

    return ret;
}

static void __exit voc_cpu_dai_exit(void)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    platform_device_unregister(voc_cpu_dai_device);
    platform_driver_unregister(&voc_cpu_dai_driver);
}

module_init(voc_cpu_dai_init);
module_exit(voc_cpu_dai_exit);


/* Module information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Trevor Wu, trevor.wu@mstarsemi.com");
MODULE_DESCRIPTION("Voc Audio ALSA SoC Dai");
