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
//  Include files
//------------------------------------------------------------------------------
#include <linux/module.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <sound/soc.h>
#ifdef __HWDEP__
#include <sound/asound.h>
#include <sound/hwdep.h>
#include <sound/soc-dapm.h>
#include "mdrv_voc_io.h"
#include "mdrv_voc_io_st.h"
#endif
#include "drvVoc.h"
//#include "drvAec.h"
#include "voc_debug.h"

#define LOW_POWER_FOR_DISABLE_WAKEUP   1
//Be careful!! Some STR logic is based on LOW_POWER_FOR_DISABLE_WAKEUP==1.
//If you set this flag = 0, please review the whole str flow.

#define SOC_ENUM_VOC(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = voc_soc_card_get_enum, .put = voc_soc_card_put_enum, \
	.private_value = (unsigned long)&xenum }

#ifdef CONFIG_MSTAR_VOICE_MMAP_BUF

typedef struct
{
    U32 phy_addr;
    U32 size;
}MMAP_INFO_s;

MMAP_INFO_s voc_buf={0};
#endif
unsigned int u32VocDbgLogLevel = DEFAULT_MESSAGES;

enum
{
    VOC_REG_CM4_ONOFF = 0,
    VOC_REG_VQ_ONOFF,
    VOC_REG_SEAMLESS_ONOFF,
    VOC_REG_SIGEN_ONOFF,
    VOC_REG_DMIC_NUMBER,
    VOC_REG_MIC_GAIN,
    VOC_REG_MIC_BITWIDTH,
    VOC_REG_AEC_MODE,
    VOC_REG_HW_AEC_ONOFF,
    VOC_REG_I2S_ONOFF,
    VOC_REG_DA_ONOFF,
    VOC_REG_HPF_ONOFF,
    VOC_REG_HPF_CONFIG,
    VOC_REG_UART_ONOFF,
    VOC_REG_WAKEUP_DMIC_NUMBER,
    VOC_REG_WAKEUP_DMIC_GAIN,
    VOC_REG_WAKEUP_DMIC_BITWIDTH,
    VOC_REG_WAKEUP_HPF_ONOFF,
    VOC_REG_WAKEUP_HPF_CONFIG,
    VOC_REG_SW_GAIN,
    VOC_REG_WAKEUP_SW_GAIN,
    VOC_REG_REF_CHN_NUMBER,
    VOC_REG_LEN,
};

static unsigned int card_reg[VOC_REG_LEN] =
{
    0x0,
    0x0,    //VOC_REG_VQ_ONOFF
    0x0,
    0x0,
#ifdef CONFIG_MSTAR_M7221
    0x0,  //VOC_REG_DMIC_NUMBER
#else
    0x2,
#endif
    0x0,
    0x0,
    0x0,   //VOC_REG_AEC_MODE
    0x0,   //VOC_REG_HW_AEC_ONOFF
    0x0,   //VOC_REG_I2S_ONOFF
    0x0,   //VOC_REG_DA_ONOFF
    0x2,   //VOC_REG_HPF_ONOFF
    0x7,   //VOC_REG_HPF_CONFIG
    0x0,   //VOC_REG_UART_ONOFF
#ifdef CONFIG_MSTAR_M7221
    0x0,   //VOC_REG_WAKEUP_DMIC_NUMBER
#else
    0x2,
#endif
    0x0,   //VOC_REG_WAKEUP_DMIC_GAIN
    0x0,   //VOC_REG_WAKEUP_DMIC_BITWIDTH
    0x2,   //VOC_REG_WAKEUP_HPF_ONOFF
    0x7,   //VOC_REG_WAKEUP_HPF_CONFIG
    0x0,   //VOC_REG_SW_GAIN
    0x0,   //VOC_REG_WAKEUP_SW_GAIN
    0x0,   //VOC_REG_REF_CHN_NUMBER
    0x0
};

static unsigned int card_reg_backup[VOC_REG_LEN] =
{
    0x0,
    0x0,    //VOC_REG_VQ_ONOFF
    0x0,
    0x0,
#ifdef CONFIG_MSTAR_M7221
    0x0,  //VOC_REG_DMIC_NUMBER
#else
    0x2,
#endif
    0x0,
    0x0,
    0x0,   //VOC_REG_AEC_MODE
    0x0,   //VOC_REG_HW_AEC_ONOFF
    0x0,   //VOC_REG_I2S_ONOFF
    0x0,   //VOC_REG_DA_ONOFF
    0x2,   //VOC_REG_HPF_ONOFF
    0x7,   //VOC_REG_HPF_CONFIG
    0x0,   //VOC_REG_UART_ONOFF
#ifdef CONFIG_MSTAR_M7221
    0x0,   //VOC_REG_WAKEUP_DMIC_NUMBER
#else
    0x2,
#endif
    0x0,   //VOC_REG_WAKEUP_DMIC_GAIN
    0x0,   //VOC_REG_WAKEUP_DMIC_BITWIDTH
    0x2,   //VOC_REG_WAKEUP_HPF_ONOFF
    0x7,   //VOC_REG_WAKEUP_HPF_CONFIG
    0x0,   //VOC_REG_SW_GAIN
    0x0,   //VOC_REG_WAKEUP_SW_GAIN
    0x0,   //VOC_REG_REF_CHN_NUMBER
    0x0
};

static const char *vq_config_text[] = {"Off", "PM", "VAD", "TEST"};
static const char *seamless_onoff_text[] = {"Off", "On"};
static const char *da_onoff_text[] = {"Off", "On"};
static const char *sigen_onoff_text[] = {"Off", "On"};
static const char *cm4_onoff_text[] = {"Off", "On"};
#ifdef CONFIG_MSTAR_M7221
static const char *dmic_number_text[] = {"2", "4", "6", "8"};
static const char *hw_aec_onoff_text[] = {"Off", "On"};
#endif
static const char *mic_bitwidth_text[] = {"16", "24", "32"};
//static const char *aec_mode_text[] = {"None", "48K Mono", "48k Stereo", "16K Mono", "16K Stereo"};

static const char *i2s_onoff_text[] = {"Off", "On"};
static const char *hpf_onoff_text[] = {"Off", "1-stage", "2-stage"};
static const char *hpf_config_text[] = {"-2", "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
static const char *uart_onoff_text[] = {"Off", "On"};

static const struct soc_enum cm4_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_CM4_ONOFF, 0, 2, cm4_onoff_text);

static const struct soc_enum vq_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_VQ_ONOFF, 0, 4, vq_config_text);

static const struct soc_enum seamless_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_SEAMLESS_ONOFF, 0, 2, seamless_onoff_text);

static const struct soc_enum da_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_DA_ONOFF, 0, 2, da_onoff_text);

static const struct soc_enum hpf_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_HPF_ONOFF, 0, 3, hpf_onoff_text);

static const struct soc_enum hpf_config_enum =
    SOC_ENUM_SINGLE(VOC_REG_HPF_CONFIG, 0, 12, hpf_config_text);

static const struct soc_enum uart_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_UART_ONOFF, 0, 2, uart_onoff_text);

static const struct soc_enum sigen_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_SIGEN_ONOFF, 0, 2, sigen_onoff_text);

#ifdef CONFIG_MSTAR_M7221
static const struct soc_enum dmic_number_enum =
    SOC_ENUM_SINGLE(VOC_REG_DMIC_NUMBER, 0, 4, dmic_number_text);

static const struct soc_enum hw_aec_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_HW_AEC_ONOFF, 0, 2, hw_aec_onoff_text);
#endif
static const struct soc_enum mic_bitwidth_enum =
    SOC_ENUM_SINGLE(VOC_REG_MIC_BITWIDTH, 0, 3, mic_bitwidth_text);


static const struct soc_enum i2s_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_I2S_ONOFF, 0, 2, i2s_onoff_text);

#ifdef CONFIG_MSTAR_M7221
static const struct soc_enum wakeup_dmic_number_enum =
    SOC_ENUM_SINGLE(VOC_REG_WAKEUP_DMIC_NUMBER, 0, 4, dmic_number_text);
#endif
static const struct soc_enum wakeup_mic_bitwidth_enum =
    SOC_ENUM_SINGLE(VOC_REG_WAKEUP_DMIC_BITWIDTH, 0, 3, mic_bitwidth_text);

static const struct soc_enum wakeup_hpf_onoff_enum =
    SOC_ENUM_SINGLE(VOC_REG_WAKEUP_HPF_ONOFF, 0, 3, hpf_onoff_text);

static const struct soc_enum wakeup_hpf_config_enum =
    SOC_ENUM_SINGLE(VOC_REG_WAKEUP_HPF_CONFIG, 0, 12, hpf_config_text);

int tmp_reg[VOC_REG_LEN] = {0};
unsigned int voc_soc_card_read(struct snd_soc_card *card, unsigned int reg)
{
    //VOC_PRINTF(DEBUG_LEVEL, "%s: reg = 0x%x, val = 0x%x\n", __FUNCTION__, reg, card_reg[reg]);
    return card_reg[reg];
}
EXPORT_SYMBOL_GPL(voc_soc_card_read);

/* TO DO : AMIC */
unsigned int voc_soc_get_mic_num(void)
{
#ifdef CONFIG_MSTAR_M7221
    switch(card_reg[VOC_REG_DMIC_NUMBER])
    {
        case 0://E_MBX_AUD_DMA_CHN2
            return 2;
        case 1://E_MBX_AUD_DMA_CHN4
            return 4;
        case 2://E_MBX_AUD_DMA_CHN6
            return 6;
        case 3://E_MBX_AUD_DMA_CHN8
            return 8;
        default:
            return 2;
    }
#else
    return card_reg[VOC_REG_DMIC_NUMBER];
#endif

}
EXPORT_SYMBOL_GPL(voc_soc_get_mic_num);

unsigned int voc_soc_get_ref_num(void)
{

    return card_reg[VOC_REG_REF_CHN_NUMBER];

}
EXPORT_SYMBOL_GPL(voc_soc_get_ref_num);


bool voc_soc_is_seamless_enabled(void)
{
    if(card_reg[VOC_REG_CM4_ONOFF] &&
        card_reg[VOC_REG_SEAMLESS_ONOFF] &&
        card_reg[VOC_REG_VQ_ONOFF])
        return true;
    return false;
}
EXPORT_SYMBOL_GPL(voc_soc_is_seamless_enabled);


unsigned int voc_soc_get_mic_bitwidth(void)
{
    switch(card_reg[VOC_REG_MIC_BITWIDTH])
    {
        case 0://E_MBX_AUD_BITWIDTH_16
            return 16;
        case 1://E_MBX_AUD_BITWIDTH_24
            return 24;
        case 2://E_MBX_AUD_BITWIDTH_32
            return 32;
        default:
            return 16;
    }

}
EXPORT_SYMBOL_GPL(voc_soc_get_mic_bitwidth);

int voc_soc_card_write(struct snd_soc_card *card, unsigned int reg, unsigned int value)
{
    S32 nRet = 0;
    int i = 0;
    VQ_CONFIG_S tVqConf;

    VOC_PRINTF(DEBUG_LEVEL, "%s: reg = 0x%x, val = 0x%x\n", __FUNCTION__, reg, value);

    if (!(card_reg[reg] ^ value)) //no change
        return 0;

    if ((reg != VOC_REG_CM4_ONOFF) && !card_reg[VOC_REG_CM4_ONOFF])
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: CM4 not power on, reg = 0x%x, val = 0x%x\n",
                   __FUNCTION__, reg, value);
        return -1;
    }

    switch(reg)
    {
        case VOC_REG_CM4_ONOFF:
            if (value == 1)
            {
                int tmp_voc_reg;
                if(!VocCopyBin2Dram())
                    return -1;
                while(i++<5)
                {
                    nRet = VocUpdateCm4Fw();
                    if(nRet == -1)
                    {
                        VOC_PRINTF(ERROR_LEVEL, "%s: %d load CM4 failed, retry again \r\n", __FUNCTION__,__LINE__);
                        VocDcOn();
                    }else{
                        break;
                    }
                }
                if(i>5)
                {
                    VOC_PRINTF(ERROR_LEVEL, "%s: %d load CM4 failed \r\n", __FUNCTION__,__LINE__);
                    VocInitOff();
                    return -1;
                }
                card_reg[VOC_REG_CM4_ONOFF] = value;
                for(i=1;i<VOC_REG_LEN;i++)
                {
                    if(card_reg[i]!=card_reg_backup[i])
                    {
                        VOC_PRINTF(DEBUG_LEVEL, "%s set %d, value:%d\r\n", __FUNCTION__,i,card_reg[i]);
                        tmp_voc_reg = card_reg[i];
                        card_reg[i] = card_reg_backup[i];
                        voc_soc_card_write(card,i,tmp_voc_reg);
                    }
                }
            }
            else
            {
                if(value!=0)
                {
                    VOC_PRINTF(ERROR_LEVEL, "%s: error parameter, reg = 0x%x, val = 0x%x\n",
                       __FUNCTION__, reg, value);
                    return -1;
                }
                VocDmaResetAudio();
                VocInitOff();
            }
            break;
        case VOC_REG_VQ_ONOFF:
          /*
            VocEnableVp(value);
            tVqConf.nMode = 1; //loop
            VocConfigVq(tVqConf);
            VocEnableVq(value);
          */
            if(value==0)
            {
//                nRet = VocEnableVp(value);
//                tVqConf.nMode = value; //loop
//                nRet |= VocConfigVq(tVqConf);
//                nRet |= VocEnableVq(value);
            }
            if(value==1)
            {
                tVqConf.nMode = 2;//E_MBX_VQ_MODE_PM;
                nRet = VocConfigVq(tVqConf);
            }
            if(value==2)
            {
                tVqConf.nMode = 3;//E_MBX_VQ_MODE_VAD;
                nRet = VocConfigVq(tVqConf);
            }
            if(value==3)
            {
                nRet = VocEnableVp(value);
                tVqConf.nMode = 1;//E_MBX_VQ_MODE_LOOP; //test loop
                nRet |= VocConfigVq(tVqConf);
                nRet |= VocEnableVq(value);
            }
            break;
        case VOC_REG_SEAMLESS_ONOFF:
#ifdef CONFIG_MSTAR_M7221
            /*Because some STR flow has been modified.
              If M7221 have to support the seamless feature,
              please review the STR flow including sboot again.*/
            VOC_PRINTF(ERROR_LEVEL, "%s:  Seamless is not support\n",__FUNCTION__);
            return -1;
#else
            VocEnableSeamless(value);
#endif
            break;
        case VOC_REG_DA_ONOFF:
            VocEnableDa(value);
            break;
        case VOC_REG_SIGEN_ONOFF:
            nRet = VocDmaEnableSinegen(value);
            break;
        case VOC_REG_DMIC_NUMBER:
            nRet = VocDmicNumber(value);
            //VocAecSetCapMode(value);
            break;
        case VOC_REG_MIC_GAIN:
            nRet = VocDmicGain(value);
            break;
        case VOC_REG_MIC_BITWIDTH:
            nRet = VocDmicBitwidth(value);
            break;
        case VOC_REG_I2S_ONOFF:
            nRet = VocI2sEnable(value);
            break;
        case VOC_REG_HW_AEC_ONOFF:
        case VOC_REG_REF_CHN_NUMBER:
            nRet = VocHwAecEnable(value);
            break;
        case VOC_REG_HPF_ONOFF:
            nRet = VocEnableHpf(value);
            break;
        case VOC_REG_HPF_CONFIG:
        {
            long tmp;
            if(!kstrtol(hpf_config_text[value],10,&tmp))
            {
                nRet = VocConfigHpf((int)tmp);
              //VOC_PRINTF(ERROR_LEVEL, "%s:  val = %ld\n",__FUNCTION__,tmp );
            }
            break;
        }
        case VOC_REG_UART_ONOFF:
            VocEnableUart(value);
            break;
        case VOC_REG_WAKEUP_DMIC_NUMBER:
        case VOC_REG_WAKEUP_DMIC_GAIN:
        case VOC_REG_WAKEUP_DMIC_BITWIDTH:
        case VOC_REG_WAKEUP_HPF_ONOFF:
        case VOC_REG_WAKEUP_HPF_CONFIG:
        case VOC_REG_WAKEUP_SW_GAIN:
            VOC_PRINTF(DEBUG_LEVEL, "%s:  store Wakeup setting reg: x%x, val = 0x%x\n",__FUNCTION__,reg,value );
            break;
        case VOC_REG_SW_GAIN:
            VocSwGain(value);
            break;
        default:
            VOC_PRINTF(ERROR_LEVEL, "%s: error parameter, reg = 0x%x, val = 0x%x\n",
                       __FUNCTION__, reg, value);
            return -1;
    }

    if(nRet)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: update error , reg = 0x%x, val = 0x%x\n",
                       __FUNCTION__, reg, value);
        return -1;
    }
    else
    {
        card_reg[reg] = value;
        return 0;
    }

}
EXPORT_SYMBOL_GPL(voc_soc_card_write);

static int voc_soc_card_get_enum(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
    struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
    ucontrol->value.integer.value[0] = voc_soc_card_read(card, e->reg);
    return 0;
}

static int voc_soc_card_put_enum(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
    struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;

	if (item[0] >= e->items)
		return -EINVAL;

    return voc_soc_card_write(card, e->reg, ucontrol->value.integer.value[0]);
}

static int voc_soc_card_get_volsw(struct snd_kcontrol *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
    struct soc_mixer_control *mc =
        (struct soc_mixer_control *)kcontrol->private_value;
    ucontrol->value.integer.value[0] = voc_soc_card_read(card, mc->reg);
    return 0;
}

static int voc_soc_card_put_volsw(struct snd_kcontrol *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
    struct soc_mixer_control *mc =
        (struct soc_mixer_control *)kcontrol->private_value;
    if((ucontrol->value.integer.value[0] < 0) || (ucontrol->value.integer.value[0] > mc->max))
    {
       VOC_PRINTF(ERROR_LEVEL, "reg:%d range MUST is (0->%d), %ld is overflow\r\n", mc->reg, mc->max, ucontrol->value.integer.value[0]);
       return -1;
    }
    return voc_soc_card_write(card, mc->reg, ucontrol->value.integer.value[0]);
}


#ifdef __HWDEP__
static int voc_hwdep_open(struct snd_hwdep * hw, struct file *file)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    return 0;
}

static int voc_hwdep_ioctl(struct snd_hwdep * hw, struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)(arg);
    int nArg;
    char n8Arg;
    int err = 0;
    VQ_CONFIG_S sVqConfig;
    VP_CONFIG_S sVpConfig;

    if (_IOC_TYPE(cmd) != VOICE_IOCTL_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > VOICE_IOCTL_MAXNR)
        return -ENOTTY;

    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (err)
    {
        return -EFAULT;
    }

    switch (cmd)
    {
        case MDRV_VOICE_IOCTL_LOAD_IMAGE:
            VocUpdateCm4Fw();
            break;

        case MDRV_VOICE_IOCTL_VQ_ENABLE:
            if (get_user(nArg, (int __user *)argp))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_VQ_ENABLE %d\r\n",nArg);
            VocEnableVq(nArg);
            break;

        case MDRV_VOICE_IOCTL_VQ_CONFIG:
            if (copy_from_user(&sVqConfig, argp, sizeof(VQ_CONFIG_S)))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_VQ_CONFIG mode:%d\r\n",sVqConfig.nMode);
            VocConfigVq(sVqConfig);
            break;

        case MDRV_VOICE_IOCTL_VP_ENABLE:
            if (get_user(nArg, (int __user *)argp))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_VP_ENABLE %d\r\n",nArg);
            VocEnableVp(nArg);
            break;

        case MDRV_VOICE_IOCTL_VP_CONFIG:
            if (copy_from_user(&sVpConfig, argp, sizeof(VP_CONFIG_S)))
            //if (get_user(sVpConfig, (VP_CONFIG_S __user *)argp))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_VP_CONFIG scale:%d\r\n",sVpConfig.nScale);
            VocConfigVp(sVpConfig);
            break;

        case MDRV_VOICE_IOCTL_SLEEP_CM4:
            if (get_user(nArg, (int __user *)argp))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_SLEEP_CM4 %d\r\n",nArg);
            VocSleepCm4(nArg);
            break;

        case MDRV_VOICE_IOCTL_KEYWORD_MATCH:
            /*nArg = VocDmaGetFlag();
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_KEYWORD_MATCH %d\r\n",nArg);
            if (put_user(nArg, (int __user *)argp))
                return -EFAULT;
*/
            break;
        case MDRV_VOICE_IOCTL_SINEGEN_ENABLE:
            if (get_user(nArg, (int __user *)argp))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_SINEGEN_ENABLE %d\r\n",nArg);
            VocDmaEnableSinegen(nArg);
            break;
        case MDRV_VOICE_IOCTL_DMIC_GAIN:
            if (get_user(n8Arg, (char __user *)argp))
                return -EFAULT;
            VOC_PRINTF(TRACE_LEVEL, "MDRV_VOICE_IOCTL_DMIC_GAIN %d\r\n",n8Arg);
            VocMicGain(n8Arg);
            break;
        default:
            VOC_PRINTF(ERROR_LEVEL, "Not supported ioctl for VOC-HWDEP\r\n");
            return -ENOIOCTLCMD;

            //......
    }
    return 0;

}
#endif

///sys/devices/platform/voc-audio/log_level
static ssize_t log_level_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "Voice driver log level = %u\n", u32VocDbgLogLevel);
}


static ssize_t log_level_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long level=0;
	int ret;

	ret = kstrtol(buf, 10, &level);
	if (ret != 0)
		return ret;

	u32VocDbgLogLevel = level;

	return count;

}

static DEVICE_ATTR(log_level, 0644, log_level_show, log_level_store);

///sys/devices/platform/voc-audio/self_Diagnostic
static ssize_t self_Diagnostic_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
    int ret = VocTestEnableSLT(0);  // 0: pass, otherwise: fail
    return sprintf(buf, "%s\n",(ret)?"FAIL":"PASS");
}

static ssize_t self_Diagnostic_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
    long type=0;
    int ret;

    ret = kstrtol(buf, 10, &type);
    if (ret != 0)
        return ret;
    if(type == 1)
    {
        VocTestEnableSLT(1);
    }else{
        VOC_PRINTF(ERROR_LEVEL, "type is not support %ld\n",type);
    }
    VOC_PRINTF(TRACE_LEVEL, "type: %ld\n",type);
    return count;
}
static DEVICE_ATTR(self_Diagnostic, 0644, self_Diagnostic_show, self_Diagnostic_store);
static int voc_soc_card_probe(struct snd_soc_card *card)
{
    int ret;
#ifdef __HWDEP__
    struct snd_hwdep *hwdep;
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    if (snd_hwdep_new(card->snd_card, "VOC-HWDEP", 0, &hwdep) < 0)
    {
        VOC_PRINTF(ERROR_LEVEL, "create VOC-HWDEP fail\n");
        return 0;
    }

    sprintf(hwdep->name, "VOC-HWDEP %d", 0);

    // hwdep->iface = SNDRV_HWDEP_IFACE_MS_VOC;
    hwdep->ops.open = voc_hwdep_open;
    hwdep->ops.ioctl = voc_hwdep_ioctl;
#else
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
#endif

    ret = device_create_file(card->dev, &dev_attr_log_level);
    if (ret != 0)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s Create log level attr err = %d\r\n", __FUNCTION__,ret);
    }
    ret = device_create_file(card->dev, &dev_attr_self_Diagnostic);
    if (ret != 0)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s Create self diagnostic attr err = %d\r\n", __FUNCTION__,ret);
    }
    return 0;
}

int voc_soc_card_suspend_pre(struct snd_soc_card *card)
{
   // VQ_CONFIG_S tVqConf;
    int i;
    S32 nRet;
    int tmp_voc_reg;

    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    if(voc_soc_card_read(card, VOC_REG_CM4_ONOFF) == 0)
    {
        VOC_PRINTF(DEBUG_LEVEL, "CM4 is disable\n");
        return 0;
    }

    VOC_PRINTF(TRACE_LEVEL, "%s record CM4 setting\r\n", __FUNCTION__);
    for(i=1;i<VOC_REG_LEN;i++)
    {
        tmp_reg[i] = voc_soc_card_read(card,i);
    }

    if(voc_soc_card_read(card, VOC_REG_VQ_ONOFF) == 0)
    {
        VOC_PRINTF(DEBUG_LEVEL, "CM4 voice wakeup is disable\n");
        return 0;
    }
  // load cm4
    nRet = VocUpdateCm4Fw();
    if(nRet)//load code error
    {
      memcpy(card_reg, card_reg_backup, sizeof(card_reg));
      VOC_PRINTF(ERROR_LEVEL, "%s reload CM4 failed\r\n", __FUNCTION__);
    }
    else
    {
      for(i=1;i<VOC_REG_LEN;i++)
      {
        if(card_reg[i]!=card_reg_backup[i])
        {
          VOC_PRINTF(DEBUG_LEVEL, "%s set %d\r\n", __FUNCTION__,card_reg[i]);
          tmp_voc_reg = card_reg[i];
          card_reg[i] = card_reg_backup[i];
          voc_soc_card_write(card,i,tmp_voc_reg);
        }
      }
    }

    if(voc_soc_card_read(card, VOC_REG_SEAMLESS_ONOFF))
    {
        VOC_PRINTF(TRACE_LEVEL, "%s don't use wakeup config\r\n", __FUNCTION__);
    }
    else
    {
        VOC_PRINTF(TRACE_LEVEL, "%s use wakeup config\r\n", __FUNCTION__);
        //swtich aec mode off(Trevor)
        voc_soc_card_write(card,VOC_REG_REF_CHN_NUMBER,0);
        //swtich mic number
        voc_soc_card_write(card,VOC_REG_DMIC_NUMBER,voc_soc_card_read(card,VOC_REG_WAKEUP_DMIC_NUMBER));
        //swtich mic bitwidth
        voc_soc_card_write(card,VOC_REG_MIC_BITWIDTH,voc_soc_card_read(card,VOC_REG_WAKEUP_DMIC_BITWIDTH));
        //swtich HPF
        voc_soc_card_write(card,VOC_REG_HPF_ONOFF,voc_soc_card_read(card,VOC_REG_WAKEUP_HPF_ONOFF));
        //HPF Coef
        voc_soc_card_write(card,VOC_REG_HPF_CONFIG,voc_soc_card_read(card,VOC_REG_WAKEUP_HPF_CONFIG));
        //switch DMIC_GAIN
        voc_soc_card_write(card,VOC_REG_MIC_GAIN,voc_soc_card_read(card,VOC_REG_WAKEUP_DMIC_GAIN));
        voc_soc_card_write(card,VOC_REG_SW_GAIN,voc_soc_card_read(card,VOC_REG_WAKEUP_SW_GAIN));
    }

    VocEnableVp(1); // enable pre-processing
    VocEnableVq(1); // enable wakup

    return 0;
}
int voc_soc_card_suspend_post(struct snd_soc_card *card)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    if(voc_soc_card_read(card, VOC_REG_CM4_ONOFF) == 0)
    {
        VOC_PRINTF(DEBUG_LEVEL, "CM4 is disable\n");
        return 0;
    }
    VocDcOff();
#if LOW_POWER_FOR_DISABLE_WAKEUP
    if (voc_soc_card_read(card, VOC_REG_VQ_ONOFF) == 0)
    {
        VocDmaResetAudio();
        VocInitOff();
    }
#endif
    return 0;
}
int voc_soc_shutdown(void)
{
//    VQ_CONFIG_S tVqConf;

    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    if(card_reg[VOC_REG_CM4_ONOFF] == 0)
    {
        VOC_PRINTF(DEBUG_LEVEL, "CM4 is disable\n");
        return 0;
    }
    if (card_reg[VOC_REG_VQ_ONOFF] != 0)
    {
        VocDmaResetAudio();
        VocInitOff();
        int i=0;
        // load cm4
        S32 nRet;
        int tmp_voc_reg;
        while(i++<5)
        {
            nRet = VocUpdateCm4Fw();
            if(nRet == -1)
            {
                VOC_PRINTF(ERROR_LEVEL, "%s: %d load CM4 failed, retry again \r\n", __FUNCTION__,__LINE__);
                VocDcOn();
            }else{
                break;
            }
        }
        if(i>5)
        {
            VOC_PRINTF(ERROR_LEVEL, "%s: %d load CM4 failed \r\n", __FUNCTION__,__LINE__);
            return -1;
        }
        for(i=1;i<VOC_REG_LEN;i++)
        {
            if(card_reg[i]!=card_reg_backup[i])
            {
              VOC_PRINTF(DEBUG_LEVEL, "%s set %d\r\n", __FUNCTION__,card_reg[i]);
              tmp_voc_reg = card_reg[i];
              card_reg[i] = card_reg_backup[i];
              voc_soc_card_write(NULL,i,tmp_voc_reg);
            }
        }

        if(card_reg[VOC_REG_SEAMLESS_ONOFF])
        {
            VOC_PRINTF(TRACE_LEVEL, "%s don't use wakeup config\r\n", __FUNCTION__);
        }
        else
        {
            VOC_PRINTF(TRACE_LEVEL, "%s use wakeup config\r\n", __FUNCTION__);
            //swtich aec mode off(Trevor)
            voc_soc_card_write(NULL,VOC_REG_REF_CHN_NUMBER,0);
            //swtich mic number
            voc_soc_card_write(NULL,VOC_REG_DMIC_NUMBER,card_reg[VOC_REG_WAKEUP_DMIC_NUMBER]);
            //swtich mic bitwidth
            voc_soc_card_write(NULL,VOC_REG_MIC_BITWIDTH,card_reg[VOC_REG_WAKEUP_DMIC_BITWIDTH]);
            //swtich HPF
            voc_soc_card_write(NULL,VOC_REG_HPF_ONOFF,card_reg[VOC_REG_WAKEUP_HPF_ONOFF]);
            //HPF Coef
            voc_soc_card_write(NULL,VOC_REG_HPF_CONFIG,card_reg[VOC_REG_WAKEUP_HPF_CONFIG]);
            //switch DMIC_GAIN
            voc_soc_card_write(NULL,VOC_REG_MIC_GAIN,card_reg[VOC_REG_WAKEUP_DMIC_GAIN]);
            voc_soc_card_write(NULL,VOC_REG_SW_GAIN,card_reg[VOC_REG_WAKEUP_SW_GAIN]);
        }
        VocEnableVp(1); // enable pre-processing
        VocEnableVq(1); // enable wakup
    }
    VocDcOff();
#if LOW_POWER_FOR_DISABLE_WAKEUP
    if (card_reg[VOC_REG_VQ_ONOFF] == 0)
    {
        VocDmaResetAudio();
        VocInitOff();
    }
#endif
    return 0;
}

//move DC on flow to sboot when CM4 is on(exclude M7221).
int voc_soc_card_resume_pre(struct snd_soc_card *card)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    if(voc_soc_card_read(card, VOC_REG_CM4_ONOFF) == 0)
    {
        VOC_PRINTF(DEBUG_LEVEL, "CM4 is disable\n");
        return 0;
    }
#ifdef CONFIG_MSTAR_M7221
#if LOW_POWER_FOR_DISABLE_WAKEUP
    if (voc_soc_card_read(card, VOC_REG_VQ_ONOFF))
#endif
        VocDcOn();
#endif

    //Be careful!! We have to disable VQ before reloading settings. It's possible that VQ still keeps enabled
    //because wake up source is not CM4.
    if (voc_soc_card_read(card, VOC_REG_VQ_ONOFF))
    {
        VocEnableVp(0);
        VocEnableVq(0);

        // reload setting
        VOC_PRINTF(TRACE_LEVEL, "%s reload CM4 setting\r\n", __FUNCTION__);
        if(voc_soc_card_read(card, VOC_REG_SEAMLESS_ONOFF))
        {
            VOC_PRINTF(TRACE_LEVEL, "%s don't reload CM4 setting\r\n", __FUNCTION__);
        }
        else
        {
            int i;
            VocDmaResetAudio();
            for(i=1;i<VOC_REG_LEN;i++)
            {
                VOC_PRINTF(DEBUG_LEVEL, "%s reload CM4 setting[%d][%d]\r\n", __FUNCTION__,i,tmp_reg[i]);
                voc_soc_card_write(card,i,tmp_reg[i]);
            }
        }
    }
#if LOW_POWER_FOR_DISABLE_WAKEUP
    else//if(voc_soc_card_read(card, VOC_REG_VQ_ONOFF) == 0)
    {
        voc_soc_card_write(card,VOC_REG_CM4_ONOFF,0);
        voc_soc_card_write(card,VOC_REG_CM4_ONOFF,1);
    }
#endif

    return 0;
}

int voc_soc_card_resume_post(struct snd_soc_card *card)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
  /*  if(voc_soc_card_read(card, VOC_REG_CM4_ONOFF)==1)
    {
        if (voc_soc_card_read(card, VOC_REG_VQ_ONOFF))
        {
            VocEnableVp(0);
            VocEnableVq(0);
        }
    }*/
    return 0;
}

static struct snd_soc_dai_link voc_soc_dais[] =
{
    {
        .name           = "Voc Soc Dai Link",
        .codec_name     = "snd-soc-dummy",
        .codec_dai_name = "snd-soc-dummy-dai",
        .cpu_dai_name   = "voc-cpu-dai",
        .platform_name  = "voc-platform",
    },
};


/* sound card controls */
static const struct snd_kcontrol_new voc_snd_controls[] =
{
    SOC_ENUM_VOC("CM4 switch", cm4_onoff_enum),
    SOC_ENUM_VOC("Voice WakeUp Switch", vq_onoff_enum),
    SOC_ENUM_VOC("Seamless Mode", seamless_onoff_enum),
#if defined(__VOICE_DA__)
    SOC_ENUM_VOC("Mic data arrange Switch", da_onoff_enum),
#endif

#if defined(__VOICE_I2S__)
    SOC_ENUM_VOC("I2S Switch", i2s_onoff_enum),
#endif
#ifdef CONFIG_MSTAR_M7221
#if defined(__VOICE_HW_AEC__)
    SOC_ENUM_VOC("HW AEC Switch", hw_aec_onoff_enum),
#endif
    SOC_ENUM_VOC("Mic Number", dmic_number_enum),
    SOC_SINGLE_EXT("Mic Gain Step (+6db)", VOC_REG_MIC_GAIN, 0, 7, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#else
    SOC_SINGLE_EXT("Mic Number", VOC_REG_DMIC_NUMBER, 0, 8, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#if defined(__VOICE_HW_AEC__)
    SOC_SINGLE_EXT("Ref channel Number", VOC_REG_REF_CHN_NUMBER, 0, 8, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#endif
    SOC_SINGLE_EXT("Mic Gain Step", VOC_REG_MIC_GAIN, 0, 480, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#endif
    SOC_ENUM_VOC("Mic Bitwidth", mic_bitwidth_enum),
    SOC_ENUM_VOC("HPF Switch", hpf_onoff_enum),
    SOC_ENUM_VOC("HPF Coef", hpf_config_enum),
    SOC_ENUM_VOC("Uart enable Switch", uart_onoff_enum),
    SOC_ENUM_VOC("Sigen Switch", sigen_onoff_enum),
    SOC_SINGLE_EXT("SW Gain Level (+1db)", VOC_REG_SW_GAIN, 0, 5, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#ifdef CONFIG_MSTAR_M7221
    SOC_ENUM_VOC("Wakeup Mic Number", wakeup_dmic_number_enum),
    SOC_SINGLE_EXT("Wakeup Mic Gain Step (+6db)", VOC_REG_WAKEUP_DMIC_GAIN, 0, 7, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#else
    SOC_SINGLE_EXT("Wakeup Mic Number", VOC_REG_WAKEUP_DMIC_NUMBER, 0, 8, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
    SOC_SINGLE_EXT("Wakeup Mic Gain Step", VOC_REG_WAKEUP_DMIC_GAIN, 0, 480, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
#endif
    SOC_ENUM_VOC("Wakeup Mic Bitwidth", wakeup_mic_bitwidth_enum),
    SOC_ENUM_VOC("Wakeup HPF Switch", wakeup_hpf_onoff_enum),
    SOC_ENUM_VOC("Wakeup HPF Coef", wakeup_hpf_config_enum),
    SOC_SINGLE_EXT("Wakeup SW Gain Level (+1db)", VOC_REG_WAKEUP_SW_GAIN, 0, 5, 0, voc_soc_card_get_volsw, voc_soc_card_put_volsw),
};

struct snd_soc_card voc_soc_card =
{
  .name       = "voc_snd_card",
  .owner      = THIS_MODULE,
  .dai_link	  = voc_soc_dais,
  .num_links  = 1,
  .probe      = voc_soc_card_probe,

  .controls = voc_snd_controls,
  .num_controls = ARRAY_SIZE(voc_snd_controls),

  .suspend_pre = voc_soc_card_suspend_pre,
  .suspend_post = voc_soc_card_suspend_post,
  .resume_pre = voc_soc_card_resume_pre,
  .resume_post = voc_soc_card_resume_post,
};
//EXPORT_SYMBOL_GPL(voc_soc_card);

#if 1//#ifndef CONFIG_OF

static struct platform_device *voc_snd_device = NULL;


static int voc_audio_probe(struct platform_device *pdev)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\n", __FUNCTION__);
    voc_soc_card.dev = &pdev->dev;
 #ifdef CONFIG_MSTAR_VOICE_MMAP_BUF
    if(voc_buf.size!=0)
        voc_mmap_buf_init(voc_buf.phy_addr, voc_buf.size);
    else
    {
        VOC_PRINTF(ERROR_LEVEL, "%s mmap memory error!\n", __FUNCTION__);
        return -EINVAL;
    }
 #endif
    return snd_soc_register_card(&voc_soc_card);
}

int voc_audio_remove(struct platform_device *pdev)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    return snd_soc_unregister_card(&voc_soc_card);
}

static struct platform_driver voc_audio_driver =
{
    .driver = {
        .name	= "voc-audio",
        .owner = THIS_MODULE,
        .pm     = &snd_soc_pm_ops,
    },
    .probe		= voc_audio_probe,
    .remove   = voc_audio_remove,
};

static int __init voc_audio_init(void)
{
    int ret = 0;
#ifdef CONFIG_MSTAR_VOICE_MMAP_BUF
    VOC_PRINTF(TRACE_LEVEL, "%s, addr %x, size %u\n",__FUNCTION__,voc_buf.phy_addr,voc_buf.size);
#endif
    //----------------------------------------------------------------


    //----------------------------------------------------------------
    voc_snd_device = platform_device_alloc("voc-audio", -1);
    if (!voc_snd_device)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: platform_device_alloc soc-audio error\r\n", __FUNCTION__);
        return -ENOMEM;
    }
    platform_set_drvdata(voc_snd_device, &voc_soc_card);
    ret = platform_device_add(voc_snd_device);
    if (ret)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: platform_device_add infinity_snd_device error\r\n", __FUNCTION__);
        platform_device_put(voc_snd_device);
    }

    ret = platform_driver_register(&voc_audio_driver);
    if (ret)
    {
        VOC_PRINTF(ERROR_LEVEL, "%s: platform_driver_register voc_dma_driver error\r\n", __FUNCTION__);
        platform_device_unregister(voc_snd_device);
    }


    return ret;
}

static void __exit voc_audio_exit(void)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\n", __FUNCTION__);
    platform_device_unregister(voc_snd_device);


}


#ifdef CONFIG_MSTAR_VOICE_MMAP_BUF
static int __init voc_set_mmap(char *str)
{
    if (str != NULL)
    {
        sscanf(str, "%x, %x", &voc_buf.phy_addr, &voc_buf.size);
    }
    return 0;
}

early_param("VOC_MEM", voc_set_mmap);
#endif


module_init(voc_audio_init);
module_exit(voc_audio_exit);

#else
static int voc_audio_probe(struct platform_device *pdev)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    voc_soc_card.dev = &pdev->dev;
    return snd_soc_register_card(&voc_soc_card);
}

int voc_audio_remove(struct platform_device *pdev)
{
    VOC_PRINTF(TRACE_LEVEL, "%s\r\n", __FUNCTION__);
    return snd_soc_unregister_card(&voc_soc_card);
}

static const struct of_device_id voc_audio_of_match[] =
{
    { .compatible = "mstar,voc-audio", },
    {},
};
MODULE_DEVICE_TABLE(of, voc_audio_of_match);

static struct platform_driver voc_audio_driver =
{
    .driver = {
        .name	= "voc-audio",
        .owner = THIS_MODULE,
        .pm     = &snd_soc_pm_ops,
        .of_match_table = voc_audio_of_match,
    },
    .probe		= voc_audio_probe,
    .remove   = voc_audio_remove,
};

module_platform_driver(voc_audio_driver);
#endif

/* Module information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Trevor Wu, trevor.wu@mstarsemi.com");
MODULE_DESCRIPTION("Voice Audio ASLA SoC Machine");

