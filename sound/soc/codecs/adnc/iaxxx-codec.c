/*
 * iaxxx-codec.c -- IAxxx CODEC driver
 *
 * Copyright 2017 Knowles Corporation
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
#include <linux/device.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/version.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-regmap.h>
#include "ia8x01-dev-board.h"

/* Use a bitmap to maintain the order of codecs registration */
static DECLARE_BITMAP(codec_reg_order, MAX_IAXXX_DEVICES);
static DEFINE_MUTEX(codec_reg_order_lock);

#define IAXXX_MAX_RETRY 5
#define IAXXX_MAX_PROC 2	/* Number of procs on ia8x01 */
#define IAXXX_MAX_VAL 0xFFFFFFFF

#define PCM_PORT_I2S 1
#define IAXXX_MAX_PORTS		3
#define IAXXX_MAX_PDM_PORTS	8
#define IAXXX_RX_DMIC_ENABLE_MASK 255
#define IAXXX_DELAY_2MS 2000

#define IAXXX_APLL_INPUT_FREQ_MAX	15
#define IAXXX_PORT_NUM_SEL	7

#define IAXXX_SOC_READ_INVALID_VAL 0xFFFFFFFF

#define IAXXX_DEFAULT_I2S_RATE_16KHZ	16000
#define IAXXX_PLAYBACK_ID		0
#define IAXXX_CAPTURE_ID		1

#define IAXXX_SND_SOC_UPDATE_BITS(codec, addr, mask, val) \
do { \
	ret = snd_soc_component_update_bits(codec, addr, mask, val); \
	if (ret < 0) { \
		dev_err(codec->dev, \
			"%s: Update reg 0x%x val 0x%x mask 0x%x failed,%d\n",\
			__func__, addr, val, mask, ret); \
	} \
} while (0)

/* Check for valid APLL port clock source */
#define IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apllsource) \
	((PLL_SRC_PORTA_CLK == apllsource) || \
	 (PLL_SRC_PORTB_CLK == apllsource) || \
	 (PLL_SRC_PORTC_CLK == apllsource) || \
	 (PLL_SRC_PORTA_DO == apllsource) || \
	 (PLL_SRC_PORTB_DO == apllsource) || \
	 (PLL_SRC_PORTC_DO == apllsource))

#define PLAT_IO_CTRL_AND_SEL_CDC_CLK_MASK \
	(1 << IAXXX_IO_CTRL_AND_SEL_CDC_CLK_POS)

static const u32 cdc0_io_ctrl_reg_list[] = {
	IAXXX_IO_CTRL_COMMA_0_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
	IAXXX_IO_CTRL_GPIO_0_ADDR,
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
};

static const u32 cdc1_io_ctrl_reg_list[] = {
	IAXXX_IO_CTRL_COMMA_3_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
	IAXXX_IO_CTRL_IRQ_ADDR,
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
};

enum {
	I2S_SRATE_NA = 0,
	I2S_SRATE_8K = 8000,
	I2S_SRATE_11K = 11025,
	I2S_SRATE_12K = 12000,
	I2S_SRATE_16K = 16000,
	I2S_SRATE_22K = 22050,
	I2S_SRATE_24K = 24000,
	I2S_SRATE_32K = 32000,
	I2S_SRATE_44K = 44100,
	I2S_SRATE_48K = 48000,
	I2S_SRATE_88K = 88200,
	I2S_SRATE_96K = 96000,
	I2S_SRATE_176K = 176400,
	I2S_SRATE_192K = 192000,
};

static const u32 iaxxx_sample_rate[] = {
	I2S_SRATE_NA,
	I2S_SRATE_8K,
	I2S_SRATE_11K,
	I2S_SRATE_12K,
	I2S_SRATE_16K,
	I2S_SRATE_22K,
	I2S_SRATE_24K,
	I2S_SRATE_32K,
	I2S_SRATE_44K,
	I2S_SRATE_48K,
	I2S_SRATE_88K,
	I2S_SRATE_96K,
	I2S_SRATE_176K,
	I2S_SRATE_192K,
};

enum {
	I2S_WORD_LEN_NA = 0, /* Invalid word length*/
	I2S_WORD_LEN_16 = 15, /*Word length of 16 bits per frame*/
	I2S_WORD_LEN_20 = 19, /*Word length of 20 bits per frame*/
	I2S_WORD_LEN_24 = 23, /*Word length of 24 bits per frame*/
	I2S_WORD_LEN_32 = 31, /*Word length of 32 bits per frame*/
};

static const u32 iaxxx_word_len[] = {
	I2S_WORD_LEN_NA,
	I2S_WORD_LEN_16,
	I2S_WORD_LEN_20,
	I2S_WORD_LEN_24,
	I2S_WORD_LEN_32,
};

enum {
	I2S_WORD_PER_FRAME_1,  /*1  word  per FS*/
	I2S_WORD_PER_FRAME_2,  /*2  words per FS*/
	I2S_WORD_PER_FRAME_3,  /*3  words per FS*/
	I2S_WORD_PER_FRAME_4,  /*4  words per FS*/
	I2S_WORD_PER_FRAME_5,  /*5  words per FS*/
	I2S_WORD_PER_FRAME_6,  /*6  words per FS*/
	I2S_WORD_PER_FRAME_7,  /*7  words per FS*/
	I2S_WORD_PER_FRAME_8,  /*8  words per FS*/
	I2S_WORD_PER_FRAME_9,  /*9  words per FS*/
	I2S_WORD_PER_FRAME_10, /*10 words per FS*/
	I2S_WORD_PER_FRAME_11, /*11 words per FS*/
	I2S_WORD_PER_FRAME_12, /*12 words per FS*/
	I2S_WORD_PER_FRAME_13, /*13 words per FS*/
	I2S_WORD_PER_FRAME_14, /*14 words per FS*/
	I2S_WORD_PER_FRAME_15, /*15 words per FS*/
	I2S_WORD_PER_FRAME_16, /*16 words per FS*/
	I2S_WORD_PER_FRAME_LIMIT  = 32,  /*Hardware limit*/
	I2S_WORD_PER_FRAME_NA  = 255,
};

static const u32 iaxxx_words_per_frame[] = {
	I2S_WORD_PER_FRAME_1,  /* 1  word  per FS   */
	I2S_WORD_PER_FRAME_2,  /* 2  words per FS   */
	I2S_WORD_PER_FRAME_3,  /* 3  words per FS   */
	I2S_WORD_PER_FRAME_4,  /* 4  words per FS   */
	I2S_WORD_PER_FRAME_5,  /* 5  words per FS   */
	I2S_WORD_PER_FRAME_6,  /* 6  words per FS   */
	I2S_WORD_PER_FRAME_7,  /* 7  words per FS   */
	I2S_WORD_PER_FRAME_8,  /* 8  words per FS   */
	I2S_WORD_PER_FRAME_9,  /* 9  words per FS   */
	I2S_WORD_PER_FRAME_10, /* 10 words per FS   */
	I2S_WORD_PER_FRAME_11, /* 11 words per FS   */
	I2S_WORD_PER_FRAME_12, /* 12 words per FS   */
	I2S_WORD_PER_FRAME_13, /* 13 words per FS   */
	I2S_WORD_PER_FRAME_14, /* 14 words per FS   */
	I2S_WORD_PER_FRAME_15, /* 15 words per FS   */
	I2S_WORD_PER_FRAME_16, /* 16 words per FS   */
};

enum {
	I2S_FSYNC_NA,
	I2S_FSYNC_1CLK  = 1,    /* 1 clk pulse sync */
	I2S_FSYNC_16CLK = 16,   /* 16 clk pulse sync */
	I2S_FSYNC_20CLK = 20,   /* 20 clk pulse sync */
	I2S_FSYNC_24CLK = 24,   /* 24 clk pulse sync */
	I2S_FSYNC_32CLK = 32,   /* 32 clk pulse sync */
};

static const u32 iaxxx_fs_duration[] = {
	I2S_FSYNC_NA,
	I2S_FSYNC_1CLK,    /* 1 clk pulse sync */
	I2S_FSYNC_16CLK,   /* 16 clk pulse sync */
	I2S_FSYNC_20CLK,   /* 20 clk pulse sync */
	I2S_FSYNC_24CLK,   /* 24 clk pulse sync */
	I2S_FSYNC_32CLK,   /* 32 clk pulse sync */
};

enum {
	NA,
	MONO = 1,
	STEREO = 3,
};

static const u32 iaxxx_data_format[] = {
	NA,
	MONO,
	STEREO
};

static int iaxxx_calc_i2s_div(struct snd_soc_component *codec,
			u32 bits_per_frame, u32 sampling_rate,
			u32 *period, u32 *div_val, u32 *nr_val);

static int iaxxx_set_i2s_cfg(struct snd_soc_dai *dai, bool is_pseudo, int id);

static int iaxxx_set_i2s_controller(struct snd_soc_component *codec, bool is_pseudo,
				    int id);
static int iaxxx_snd_soc_info_multi_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);

struct iaxxx_soc_multi_mixer_control {
	int min, max, platform_max, count;
	unsigned int reg, rreg, shift, rshift, invert;
};

#define IAXXX_SOC_SINGLE_MULTI_EXT(xname, xreg, xshift, xmax, xinvert, xcount,\
	xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = iaxxx_snd_soc_info_multi_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct iaxxx_soc_multi_mixer_control)\
		{.reg = xreg, .shift = xshift, .rshift = xshift, .max = xmax, \
		.count = xcount, .platform_max = xmax, .invert = xinvert} }

/* Plugin struct to store param id and val
 * Param ID and Param VAL registers are set as pair
 */
struct plg_param {
	u32 param_id;
	u32 param_val;
	u32 param_id_reg;
	u32 param_val_reg;
};

enum {
	IAXXX_DMIC0_CLK_SRC,	/*DMIC0_CLK */
	IAXXX_CDC0_CLK_SRC,	/*CDC0_CLK */
	IAXXX_DMIC1_CLK_SRC,	/*DMIC1_CLK */
	IAXXX_CDC1_CLK_SRC,	/*CDC1_CLK */

	IAXXX_NUM_PDM_CLK_SRC,
};

enum {
	I2S_GEN0, /* PortA Clk */
	I2S_GEN1, /* PortB DO  */
	I2S_GEN2, /* PortC Clk */
	I2S_GEN3, /* PortA DO or COMMB_0 */
	I2S_GEN4, /* PortB Clk */
	I2S_GEN5, /* PortC DO */
	I2S_GEN6, /* COMMB_3 */

	NUM_I2S_GEN,
};

enum {
    /* Digital PDM Mic In*/
	PDM_DMIC_IN0 = 0,  /* PDM DMIC Input0 */
	PDM_DMIC_IN1,      /* PDM DMIC Input1 */
	PDM_DMIC_IN2,      /* PDM DMIC Input2 */
	PDM_DMIC_IN3,      /* PDM DMIC Input3 */
	PDM_DMIC_IN4,      /* PDM DMIC Input4 */
	PDM_DMIC_IN5,      /* PDM DMIC Input5 */
	PDM_DMIC_IN6,      /* PDM DMIC Input6 */
	PDM_DMIC_IN7,      /* PDM DMIC Input7 */

	/* Digital PDM Spk out */
	PDM_DMIC_OUT0,     /* PDM DMIC Out0 */
	PDM_DMIC_OUT1,     /* PDM DMIC Out1 */

	/* ADCs PDM In */
	PDM_CDC0_IN0,      /* PDM ADC In0 */
	PDM_CDC1_IN1,      /* PDM ADC In1 */
	PDM_CDC2_IN2,      /* PDM ADC In2 */
	PDM_CDC3_IN3,      /* PDM ADC In3 */

	PDM_CDC0_IN4,      /* PDM ADC In0 */
	PDM_CDC1_IN5,      /* PDM ADC In1 */
	PDM_CDC2_IN6,      /* PDM ADC In2 */
	PDM_CDC3_IN7,      /* PDM ADC In3 */

	/* DACs PDM Out */
	PDM_CDC_DAC_OUT0,  /* PDM DAC Out0 */
	PDM_CDC_DAC_OUT1,  /* PDM DAC Out1 */

	PDM_DMIC_MONO_IN0,      /* PDM DMIC MONO Input0 */
	PDM_DMIC_MONO_IN1,      /* PDM DMIC MONO Input1 */
	PDM_DMIC_MONO_IN2,      /* PDM DMIC MONO Input2 */
	PDM_DMIC_MONO_IN3,      /* PDM DMIC MONO Input3 */
	PDM_NUM_IO_MICS
};

struct i2s_clk_params {
	u32 sample_rate;
	u32 word_len;
	u32 words_per_frame;
	u32 fs_duration;
	bool clk_pol;
	bool fs_pol;
};

struct pcm_cfg {
	u32 word_len;
	u32 no_of_channels;
	bool tristate;
};

struct iaxxx_codec_priv {
	int is_codec_master[IAXXX_MAX_PORTS];
	int pcm_dai_fmt[IAXXX_MAX_PORTS];
	struct regmap *regmap;
	struct snd_soc_component *codec;
	struct device *dev;
	struct device *dev_parent;
	struct notifier_block nb_core;	/* Core notifier */
	/* Add entry for plg_param struct for each proc
	 * param id and param val registers are set as pair
	 */
	struct plg_param plugin_param[IAXXX_MAX_PROC];
	bool is_ip_port_master[IAXXX_MAX_PORTS];
	struct i2s_clk_params clk_param_info[IAXXX_MAX_PORTS];
	struct pcm_cfg pcm_info[IAXXX_MAX_PORTS];
	u32 pcm_port_fmt[IAXXX_MAX_PORTS];
	/* pcm port word length configuration
	 *  0: auto (according to application)
	 * 16: 16 bits per sample
	 * 20: 20 bits per sample
	 * 24: 24 bits per sample
	 * 32: 32 bits per sample
	 */
	u32 pcm_port_word_len[IAXXX_MAX_PORTS];

	/* PCM Port Data-out (Dout) line tri-state on playback
	 * 0 : no-tristate
	 * 1 : tri-state / HiZ
	 */
	u32 pcm_port_dout_hiz_on_playback[IAXXX_MAX_PORTS];
	u32 master_src[NUM_I2S_GEN];

	u32 port_start_en[IAXXX_NUM_PDM_CLK_SRC];
	bool port_filter[NUM_I2S_GEN];
	bool port_pcm_start[IAXXX_MAX_PORTS];
	bool port_pcm_setup[IAXXX_MAX_PORTS];
	u32 pdm_clk_drive_strength[IAXXX_MAX_PDM_PORTS];
	bool plugin_blk_en[32];
	bool stream_en[32];
	u32 op_channels_active;
	u8 i2s_master_clk;

	/* pdm mic enable flags*/
	u32 port_mic_en[PDM_NUM_IO_MICS];

	/* pdm bclk and aclk flags for port configuration*/
	u32 pdm_bclk;
	u32 apll_clk;
	u32 apll_src;
	u32 apll_input_freq;
	u32 pdm_aud_port_clk;

	u32 head_of_strm_rx_all;
	u32 head_of_strm_tx_all;
	u32 cdc_dmic_enable;
	int is_stream_in_use[IAXXX_MAX_PORTS][2];

	/* For PDM port selection */
	u32 pdm_mic_port[PDM_NUM_IO_MICS];
};

static const u32 cic_rx_addr[] = {
	IAXXX_CNR0_CIC_RX_0_1_ADDR,
	IAXXX_CNR0_CIC_RX_0_1_ADDR,
	IAXXX_CNR0_CIC_RX_2_3_ADDR,
	IAXXX_CNR0_CIC_RX_2_3_ADDR,
};

static const u32 cic_rx_m_pos[] = {
	IAXXX_CNR0_CIC_RX_0_1_M_0_POS,
	IAXXX_CNR0_CIC_RX_0_1_M_1_POS,
	IAXXX_CNR0_CIC_RX_2_3_M_2_POS,
	IAXXX_CNR0_CIC_RX_2_3_M_3_POS,
};

static const u32 dmic_enable_addr[] = {
	IAXXX_CNR0_DMIC0_ENABLE_ADDR,
	IAXXX_CNR0_CDC0_ENABLE_ADDR,
	IAXXX_CNR0_DMIC1_ENABLE_ADDR,
	IAXXX_CNR0_CDC1_ENABLE_ADDR,
};

static const u32 dmic_busy_addr[] = {
	IAXXX_CNR0_DMIC0_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_CDC0_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_DMIC1_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_CDC1_ENABLE_BUSY_ADDR,
};

enum {
	RX_0 = 0,
	RX_1,
	RX_2,
	RX_3,
	RX_4,
	RX_5,
	RX_6,
	RX_7,
	RX_8,
	RX_9,
	RX_10,
	RX_11,
	RX_12,
	RX_13,
	RX_14,
	RX_15,
	MAX_RX,
};

enum {
	TX_0 = 0,
	TX_1,
	TX_2,
	TX_3,
	TX_4,
	TX_5,
	TX_6,
	TX_7,
	TX_8,
	TX_9,
	TX_10,
	TX_11,
	TX_12,
	TX_13,
	TX_14,
	TX_15,
	MAX_TX,
};

enum iaxxx_pdm_port {
	NONE,
	IAXXX_PORT_A,
	IAXXX_PORT_B,
	IAXXX_PORT_C,
	IAXXX_COMM_A,
	IAXXX_COMM_B,
};

enum {
	PCM_PORTA = 0,
	PCM_PORTB = 1,
	PCM_PORTC = 2,
};

enum {
	PCM_SLAVE = 0,
	PCM_MASTER = 1,

};

enum iaxxx_pdm_clk {
	PORTA_CLK,
	PORTB_DO,
	PORTC_CLK,
	COMMB_0,
	PORTA_DO,
	PORTB_CLK,
	PORTC_DO,
	COMMB_3,
};

enum {
	PLUGIN0 = 0,
	PLUGIN1,
	PLUGIN2,
	PLUGIN3,
	PLUGIN4,
	PLUGIN5,
	PLUGIN6,
	PLUGIN7,
	PLUGIN8,
	PLUGIN9,
	PLUGIN10,
	PLUGIN11,
	PLUGIN12,
	PLUGIN13,
	PLUGIN14,
	MAX_PLUGIN,
};

enum {
	CIC0,
	CIC1,
	CIC2,
	CIC3,
	CIC4,
	CIC5,
	CIC6,
	CIC7,
	CIC_NONE
};

enum {
	DMIC0,
	DMIC1,
	DMIC2,
	DMIC3,
	DMIC4,
	DMIC5,
	DMIC6,
	DMIC7,
};

enum {
	IAXXX_PDM_CLK_NONE,
	IAXXX_PDM_CLK_0P_256MHZ,
	IAXXX_PDM_CLK_0P_512MHZ,
	IAXXX_PDM_CLK_0P_768MHZ,
	IAXXX_PDM_CLK_1P_024MHZ,
	IAXXX_PDM_CLK_1P_536MHZ,
	IAXXX_PDM_CLK_2P_048MHZ,
	IAXXX_PDM_CLK_2P_560MHZ,
	IAXXX_PDM_CLK_2P_822MHZ,
	IAXXX_PDM_CLK_3P_072MHZ,
	IAXXX_PDM_CLK_3P_840MHZ,
	IAXXX_PDM_CLK_4P_096MHZ,
	IAXXX_PDM_CLK_5P_644MHZ,
	IAXXX_PDM_CLK_6P_144MHZ,
	IAXXX_PDM_CLK_7P_680MHZ,
	IAXXX_PDM_CLK_8P_192MHZ,
	IAXXX_PDM_CLK_11P_520MHZ,
	IAXXX_PDM_CLK_12P_288MHZ,
	IAXXX_PDM_CLK_24P_576MHZ,
	IAXXX_PDM_CLK_MAX,
};

enum {
	IAXXX_ACLK_FREQ_NONE,
	IAXXX_ACLK_FREQ_3072,
	IAXXX_ACLK_FREQ_6144,
	IAXXX_ACLK_FREQ_12288,
	IAXXX_ACLK_FREQ_24576,
	IAXXX_ACLK_FREQ_49152,
	IAXXX_ACLK_FREQ_98304,
	IAXXX_ACLK_FREQ_368640,
	IAXXX_ACLK_FREQ_MAX,
};

enum {
	IAXXX_AUD_PORT_NONE,
	IAXXX_AUD_PORT_8K,
	IAXXX_AUD_PORT_11_025K,
	IAXXX_AUD_PORT_12K,
	IAXXX_AUD_PORT_16K,
	IAXXX_AUD_PORT_22_05K,
	IAXXX_AUD_PORT_24K,
	IAXXX_AUD_PORT_32K,
	IAXXX_AUD_PORT_44_1K,
	IAXXX_AUD_PORT_48K,
	IAXXX_AUD_PORT_64K,
	IAXXX_AUD_PORT_88_2K,
	IAXXX_AUD_PORT_96K,
	IAXXX_AUD_PORT_176_4K,
	IAXXX_AUD_PORT_192K,
	IAXXX_AUD_PORT_384K,
	IAXXX_AUD_PORT_768K,
	IAXXX_AUD_PORT_1536K,
	IAXXX_AUD_PORT_128K,
	IAXXX_AUD_PORT_256K,
	IAXXX_AUD_PORT_MAX,
};

static const uint32_t iaxxx_apllClk_Val[IAXXX_ACLK_FREQ_MAX] = {
	0,
	3072000,        /*I2S_ACLK_FREQ_3072*/
	6144000,        /*I2S_ACLK_FREQ_6144*/
	12288000,       /*I2S_ACLK_FREQ_12288*/
	24576000,       /*I2S_ACLK_FREQ_24576*/
	49152000,       /*I2S_ACLK_FREQ_49152*/
	98304000,       /*I2S_ACLK_FREQ_98304*/
	368640000,      /*I2S_ACLK_FREQ_368640*/
};

static const uint32_t iaxxx_apll_in_freq_val[IAXXX_APLL_INPUT_FREQ_MAX] = {
	0,
	512000,		/* 512KHz */
	768000,		/* 768KHz */
	1024000,	/* 1.024MHz */
	1536000,	/* 1.536MHz */
	2048000,	/* 2.048MHz */
	2400000,        /* 2.400MHz */
	3072000,        /* 3.072MHz */
	4608000,        /* 4.608MHz */
	4800000,        /* 4.800MHz */
	6144000,        /* 6.144MHz */
	9600000,        /* 9.600MHz */
	12288000,       /* 12.288MHz */
	19200000,       /* 19.200MHz */
	24576000,       /* 24.576MHz */
};

static const uint32_t iaxxx_bitClock_Val[IAXXX_PDM_CLK_MAX] = {
	0,            /* I2S_BIT_CLK_FREQ_NONE */
	256000,       /* I2S_BIT_CLK_FREQ_0_256M */
	512000,       /* I2S_BIT_CLK_FREQ_0_512M */
	768000,       /* I2S_BIT_CLK_FREQ_0_768M */
	1024000,      /* I2S_BIT_CLK_FREQ_1_024M */
	1536000,      /* I2S_BIT_CLK_FREQ_1_536M */
	2048000,      /* I2S_BIT_CLK_FREQ_2_048M */
	2560000,      /* I2S_BIT_CLK_FREQ_2_560M */
	2822400,      /* I2S_BIT_CLK_FREQ_2_8224M */
	3072000,      /* I2S_BIT_CLK_FREQ_3_072M */
	3840000,      /* I2S_BIT_CLK_FREQ_3_840M */
	4096000,      /* I2S_BIT_CLK_FREQ_4_096M */
	5644800,      /* I2S_BIT_CLK_FREQ_5_6448M */
	6144000,      /* I2S_BIT_CLK_FREQ_6_144M */
	7680000,      /* I2S_BIT_CLK_FREQ_7_680M */
	8192000,      /* I2S_BIT_CLK_FREQ_8_192M */
	11520000,     /* I2S_BIT_CLK_FREQ_11_52M */
	12288000,     /* I2S_BIT_CLK_FREQ_12_288M */
	24576000,     /* I2S_BIT_CLK_FREQ_24_576M */
};

enum {
	PDM_PORT_DMIC,
	PDM_PORT_PDMO,
	PDM_PORT_ADC,
	PDM_PORT_DAC,
	PDM_PORT_MONO,
};

enum {
	FILTER_CIC_DECIMATION_4  = (4 - 1), /*Decimate by 4*/
	FILTER_CIC_DECIMATION_6 = (6 - 1),  /*Decimate by 6*/
	FILTER_CIC_DECIMATION_8 = (8 - 1),  /*Decimate by 8*/
	FILTER_CIC_DECIMATION_12 = (12 - 1),/*Decimate by 12*/
	FILTER_CIC_DECIMATION_16 = (16 - 1),/*Decimate by 16*/
	FILTER_CIC_DECIMATION_24 = (24 - 1),/*Decimate by 24*/
	FILTER_CIC_DECIMATION_32 = (32 - 1),/*Decimate by 32*/
	FILTER_CIC_DECIMATION_NA = (0xFF),  /*Invalid Decimation*/
};

enum {
	FILTER_CIC_HB_PT,           /*PassThru, no HB decimation*/
	FILTER_CIC_HB_BY_2,         /*Decimate by 2*/
	FILTER_CIC_HB_BY_4,         /*Decimate by 4*/
	FILTER_CIC_HB_NA,
};

struct iaxxx_cic_deci_table {
	u32 cic_dec;
	u32 hb_dec;
};

struct iaxxx_pdm_bit_cfg {
	u32 sample_rate;
	u32 words_per_frame;
	u32 word_length;
};

/* PDM port number to IO_CTRL register mapping */
static const uint32_t iaxxx_io_ctrl_data[24][2] = {
	/* PDM_DMIC_IN0 */
	{PLAT_DMIC_DATA_IN0_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN0_AND_SEL_MASK},
	/* PDM_DMIC_IN1 */
	{PLAT_DMIC_DATA_IN1_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN1_AND_SEL_MASK},
	/* PDM_DMIC_IN2 */
	{PLAT_DMIC_DATA_IN2_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN2_AND_SEL_MASK},
	/* PDM_DMIC_IN3 */
	{PLAT_DMIC_DATA_IN3_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN3_AND_SEL_MASK},
	/* PDM_DMIC_IN4 */
	{PLAT_DMIC_DATA_IN4_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN4_AND_SEL_MASK},
	/* PDM_DMIC_IN5 */
	{PLAT_DMIC_DATA_IN5_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN5_AND_SEL_MASK},
	/* PDM_DMIC_IN6 */
	{PLAT_DMIC_DATA_IN6_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN6_AND_SEL_MASK},
	/* PDM_DMIC_IN7 */
	{PLAT_DMIC_DATA_IN7_IO_CTRL_ADDR, PLAT_DMIC_DATA_IN7_AND_SEL_MASK},
	/* PDM_DMIC_OUT0 */
	{PLAT_DMIC_DATA_OUT0_IO_CTRL_ADDR, PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
	/* PDM_DMIC_OUT1 */
	{PLAT_DMIC_DATA_OUT1_IO_CTRL_ADDR, PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
	/* PDM_CDC_IN0 */
	{PLAT_ADC_DATA_IN0_IO_CTRL_ADDR, PLAT_ADC_DATA_IN0_AND_SEL_MASK},
	/* PDM_CDC_IN1 */
	{PLAT_ADC_DATA_IN1_IO_CTRL_ADDR, PLAT_ADC_DATA_IN1_AND_SEL_MASK},
	/* PDM_CDC_IN2 */
	{PLAT_ADC_DATA_IN2_IO_CTRL_ADDR, PLAT_ADC_DATA_IN2_AND_SEL_MASK},
	/* PDM_CDC_IN3 */
	{PLAT_ADC_DATA_IN3_IO_CTRL_ADDR, PLAT_ADC_DATA_IN3_AND_SEL_MASK},
	/* PDM_CDC_IN4 */
	{PLAT_ADC_DATA_IN4_IO_CTRL_ADDR, PLAT_ADC_DATA_IN4_AND_SEL_MASK},
	/* PDM_CDC_IN5 */
	{PLAT_ADC_DATA_IN5_IO_CTRL_ADDR, PLAT_ADC_DATA_IN5_AND_SEL_MASK},
	/* PDM_CDC_IN6 */
	{PLAT_ADC_DATA_IN6_IO_CTRL_ADDR, PLAT_ADC_DATA_IN6_AND_SEL_MASK},
	/* PDM_CDC_IN7 */
	{PLAT_ADC_DATA_IN7_IO_CTRL_ADDR, PLAT_ADC_DATA_IN7_AND_SEL_MASK},
	/* PDM_DAC_OUT0 */
	{PLAT_DAC_DATA_OUT0_IO_CTRL_ADDR, PLAT_DAC_DATA_OUT0_AND_SEL_MASK},
	/* PDM_DAC_OUT1 */
	{PLAT_DAC_DATA_OUT1_IO_CTRL_ADDR, PLAT_DAC_DATA_OUT1_AND_SEL_MASK},
	/* PDM_DMIC_MONO_IN0 */
	{PLAT_DMIC_MONO_DATA_IN0_IO_CTRL_ADDR,
		PLAT_DMIC_MONO_DATA_IN0_AND_SEL_MASK},
	/* PDM_DMIC_MONO_IN1 */
	{PLAT_DMIC_MONO_DATA_IN1_IO_CTRL_ADDR,
		PLAT_DMIC_MONO_DATA_IN1_AND_SEL_MASK},
	/* PDM_DMIC_MONO_IN2 */
	{PLAT_DMIC_MONO_DATA_IN2_IO_CTRL_ADDR,
		PLAT_DMIC_MONO_DATA_IN2_AND_SEL_MASK},
	/* PDM_DMIC_MONO_IN3 */
	{PLAT_DMIC_MONO_DATA_IN3_IO_CTRL_ADDR,
		PLAT_DMIC_MONO_DATA_IN3_AND_SEL_MASK}
};

/* PDM clock to IO_CTRL register mapping */
static const uint32_t iaxxx_io_ctrl_clk_in_port[PDM_NUM_IO_MICS][3] = {
	{PLAT_DMIC_CLK_IN0_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN1_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN2_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN3_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN4_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN5_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN6_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_IN7_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_OUT0_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_DMIC_CLK_OUT1_IO_CTRL_ADDR, PDM_CLK_SLAVE, PDM_CLK_MASTER},
	{PLAT_ADC_CLK_IN0_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN1_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN2_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN3_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN4_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN5_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN6_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_ADC_CLK_IN7_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_DAC_CLK_OUT0_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_DAC_CLK_OUT1_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_DMIC_MONO_CLK_IN0_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_DMIC_MONO_CLK_IN1_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_DMIC_MONO_CLK_IN2_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
	{PLAT_DMIC_MONO_CLK_IN3_IO_CTRL_ADDR, CDC_CLK_SLAVE, CDC_CLK_MASTER},
};

static const uint32_t iaxxx_io_ctrl_clk_out_port[4][3] = {
		{PLAT_DMIC0_OUT_CLK_IO_CTRL_ADDR, PDM0_OUT_CLK_SLAVE,
			PDM0_OUT_CLK_MASTER},
		{PLAT_CDC0_OUT_CLK_IO_CTRL_ADDR, CDC0_OUT_CLK_SLAVE,
			CDC0_OUT_CLK_MASTER},
		{PLAT_DMIC1_OUT_CLK_IO_CTRL_ADDR, PDM1_OUT_CLK_SLAVE,
			PDM1_OUT_CLK_MASTER},
		{PLAT_CDC1_OUT_CLK_IO_CTRL_ADDR, CDC1_OUT_CLK_SLAVE,
			CDC1_OUT_CLK_MASTER},
};

static const uint32_t iaxxx_io_ctrl_pdm_clk_addr[IAXXX_MAX_PDM_PORTS] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,

};

static const uint32_t iaxxx_pad_ctrl_pdm_clk_addr[IAXXX_MAX_PDM_PORTS] = {
	IAXXX_PAD_CTRL_PORTA_CLK_ADDR,
	IAXXX_PAD_CTRL_PORTB_DO_ADDR,
	IAXXX_PAD_CTRL_PORTC_CLK_ADDR,
	IAXXX_PAD_CTRL_COMMB_0_ADDR,
	IAXXX_PAD_CTRL_PORTA_DO_ADDR,
	IAXXX_PAD_CTRL_PORTB_CLK_ADDR,
	IAXXX_PAD_CTRL_PORTC_DO_ADDR,
	IAXXX_PAD_CTRL_COMMB_3_ADDR,
};

static const uint32_t
	iaxxx_dynamic_io_ctrl_clk_in_reset_val[][PDM_NUM_IO_MICS] = {
	/* PORT_A */
	{
		PLAT_DMIC_CLK_IN0_PORTA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN1_PORTA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN2_PORTA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN3_PORTA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN3 */
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_PORTA_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_PORTA_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_PORTA_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_PORTA_IO_CTRL_RESET_VAL,
	},

	/* PORT_B */
	{
		PLAT_DMIC_CLK_IN0_PORTB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN1_PORTB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN2_PORTB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN3_PORTB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN3 */
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_PORTB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_PORTB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_PORTB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_PORTB_IO_CTRL_RESET_VAL,
	},

	/* PORT_C */
	{
		PLAT_DMIC_CLK_IN0_PORTC_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN1_PORTC_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN2_PORTC_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN3_PORTC_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN3 */
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_PORTC_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_PORTC_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_PORTC_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_PORTC_IO_CTRL_RESET_VAL,
	},

	/* COMM_A */
	{
		PLAT_DMIC_CLK_IN0_COMMA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN1_COMMA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN2_COMMA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN3_COMMA_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN3 */
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_COMMA_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_COMMA_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_COMMA_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_COMMA_IO_CTRL_RESET_VAL,
	},

	/* COMM_B */
	{
		PLAT_DMIC_CLK_IN0_COMMB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN1_COMMB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN2_COMMB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN3_COMMB_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN3 */
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_COMMB_IO_CTRL_RESET_VAL,
	},

	/* GPIO_OTHR */
	{
		/* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN0_GPIO_OTHR_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN1_GPIO_OTHR_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN2_GPIO_OTHR_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_IN3 */
		PLAT_DMIC_CLK_IN3_GPIO_OTHR_IO_CTRL_RESET_VAL,
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_GPIO_OTHR_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_GPIO_OTHR_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_GPIO_OTHR_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_GPIO_OTHR_IO_CTRL_RESET_VAL,
	},

	/* GPIO_COMMB */
	{
		/* PDM_DMIC_IN0 */
		PLAT_DMIC_CLK_IN0_GPIO_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_IN1 */
		PLAT_DMIC_CLK_IN1_GPIO_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_IN2 */
		PLAT_DMIC_CLK_IN2_GPIO_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_IN3 */
		PLAT_DMIC_CLK_IN3_GPIO_COMMB_IO_CTRL_RESET_VAL,
		0, /* PDM_DMIC_IN4 */
		0, /* PDM_DMIC_IN5 */
		0, /* PDM_DMIC_IN6 */
		0, /* PDM_DMIC_IN7 */
		0, /* PDM_DMIC_OUT0 */
		0, /* PDM_DMIC_OUT1 */
		0, /* PDM_CDC_IN0 */
		0, /* PDM_CDC_IN1 */
		0, /* PDM_CDC_IN2 */
		0, /* PDM_CDC_IN3 */
		0, /* PDM_CDC_IN4 */
		0, /* PDM_CDC_IN5 */
		0, /* PDM_CDC_IN6 */
		0, /* PDM_CDC_IN7 */
		0, /* PDM_DAC_OUT0 */
		0,  /* PDM_DAC_OUT1 */
		/* PDM_DMIC_MONO_IN0 */
		PLAT_DMIC_MONO_CLK_IN0_GPIO_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN1 */
		PLAT_DMIC_MONO_CLK_IN1_GPIO_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN2 */
		PLAT_DMIC_MONO_CLK_IN2_GPIO_COMMB_IO_CTRL_RESET_VAL,
		/* PDM_DMIC_MONO_IN3 */
		PLAT_DMIC_MONO_CLK_IN3_GPIO_COMMB_IO_CTRL_RESET_VAL,
	},
};

/* PDM port data line mapping */
static const uint32_t iaxxx_dynamic_ioctrl_set_data
	[IAXXX_PORT_NUM_SEL][PDM_NUM_IO_MICS][2] = {
	{
		/* PDM_DMIC_IN0 */
		{PLAT_DMIC_DATA_IN0_IO_CTRL_PORTA_ADDR,
			PLAT_DMIC_DATA_IN0_PORTA_AND_SEL_MASK},
		/* PDM_DMIC_IN1 */
		{PLAT_DMIC_DATA_IN1_IO_CTRL_PORTA_ADDR,
			PLAT_DMIC_DATA_IN1_PORTA_AND_SEL_MASK},
		/* PDM_DMIC_IN2 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_PORTA_ADDR,
			PLAT_DMIC_DATA_IN2_PORTA_AND_SEL_MASK},
		/* PDM_DMIC_IN3 */
		{PLAT_DMIC_DATA_IN3_IO_CTRL_PORTA_ADDR,
			PLAT_DMIC_DATA_IN3_PORTA_AND_SEL_MASK},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{PLAT_DMIC_DATA_OUT0_IO_CTRL_PORTA_ADDR,
			PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
		/* PDM_DMIC_OUT1 */
		{PLAT_DMIC_DATA_OUT1_IO_CTRL_PORTA_ADDR,
			PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{0, 0},
		/* PDM_DMIC_MONO_IN1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN2 */
		{0, 0},
		/* PDM_DMIC_MONO_IN3 */
		{0, 0},
	},
	{
		/* PDM_DMIC_IN0 */
		{PLAT_DMIC_DATA_IN0_IO_CTRL_PORTB_ADDR,
			PLAT_DMIC_DATA_IN0_PORTB_AND_SEL_MASK},
		/* PDM_DMIC_IN1 */
		{PLAT_DMIC_DATA_IN1_IO_CTRL_PORTB_ADDR,
			PLAT_DMIC_DATA_IN1_PORTB_AND_SEL_MASK},
		/* PDM_DMIC_IN2 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_PORTB_ADDR,
			PLAT_DMIC_DATA_IN2_PORTB_AND_SEL_MASK},
		/* PDM_DMIC_IN3 */
		{PLAT_DMIC_DATA_IN3_IO_CTRL_PORTB_ADDR,
			PLAT_DMIC_DATA_IN3_PORTB_AND_SEL_MASK},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{PLAT_DMIC_DATA_OUT0_IO_CTRL_PORTB_ADDR,
			PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
		/* PDM_DMIC_OUT1 */
		{PLAT_DMIC_DATA_OUT1_IO_CTRL_PORTB_ADDR,
			PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{0, 0},
		/* PDM_DMIC_MONO_IN1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN2 */
		{0, 0},
		/* PDM_DMIC_MONO_IN3 */
		{0, 0},
	},
	{
		/* PDM_DMIC_IN0 */
		{PLAT_DMIC_DATA_IN0_IO_CTRL_PORTC_ADDR,
			PLAT_DMIC_DATA_IN0_PORTC_AND_SEL_MASK},
		/* PDM_DMIC_IN1 */
		{PLAT_DMIC_DATA_IN1_IO_CTRL_PORTC_ADDR,
			PLAT_DMIC_DATA_IN1_PORTC_AND_SEL_MASK},
		/* PDM_DMIC_IN2 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_PORTC_ADDR,
			PLAT_DMIC_DATA_IN2_PORTC_AND_SEL_MASK},
		/* PDM_DMIC_IN3 */
		{PLAT_DMIC_DATA_IN3_IO_CTRL_PORTC_ADDR,
			PLAT_DMIC_DATA_IN3_PORTC_AND_SEL_MASK},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{PLAT_DMIC_DATA_OUT0_IO_CTRL_PORTC_ADDR,
			PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
		/* PDM_DMIC_OUT1 */
		{PLAT_DMIC_DATA_OUT1_IO_CTRL_PORTC_ADDR,
			PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_DATA_IN0_IO_CTRL_ADDR,
			PLAT_DMIC_MONO_DATA_IN0_AND_SEL_MASK},
		/* PDM_DMIC_MONO_IN1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN2 */
		{0, 0},
		/* PDM_DMIC_MONO_IN3 */
		{PLAT_DMIC_MONO_DATA_IN3_IO_CTRL_ADDR,
			PLAT_DMIC_MONO_DATA_IN3_AND_SEL_MASK},
	},
	{
		/* PDM_DMIC_IN0 */
		{PLAT_DMIC_DATA_IN0_IO_CTRL_COMMA_ADDR,
			PLAT_DMIC_DATA_IN0_COMMA_AND_SEL_MASK},
		/* PDM_DMIC_IN1 */
		{PLAT_DMIC_DATA_IN1_IO_CTRL_COMMA_ADDR,
			PLAT_DMIC_DATA_IN1_COMMA_AND_SEL_MASK},
		/* PDM_DMIC_IN2 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_COMMA_ADDR,
			PLAT_DMIC_DATA_IN2_COMMA_AND_SEL_MASK},
		/* PDM_DMIC_IN3 */
		{PLAT_DMIC_DATA_IN3_IO_CTRL_COMMA_ADDR,
			PLAT_DMIC_DATA_IN3_COMMA_AND_SEL_MASK},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{PLAT_DMIC_DATA_OUT0_IO_CTRL_COMMA_ADDR,
			PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
		/* PDM_DMIC_OUT1 */
		{PLAT_DMIC_DATA_OUT1_IO_CTRL_COMMA_ADDR,
			PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{0, 0},
		/* PDM_DMIC_MONO_IN1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN2 */
		{0, 0},
		/* PDM_DMIC_MONO_IN3 */
		{0, 0},
	},
	{
		/* PDM_DMIC_IN0 */
		{PLAT_DMIC_DATA_IN0_IO_CTRL_COMMB_ADDR,
			PLAT_DMIC_DATA_IN0_COMMB_AND_SEL_MASK},
		/* PDM_DMIC_IN1 */
		{PLAT_DMIC_DATA_IN1_IO_CTRL_COMMB_ADDR,
		PLAT_DMIC_DATA_IN1_COMMB_AND_SEL_MASK},
		/* PDM_DMIC_IN2 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_COMMB_ADDR,
			PLAT_DMIC_DATA_IN2_COMMB_AND_SEL_MASK},
		/* PDM_DMIC_IN3 */
		{PLAT_DMIC_DATA_IN3_IO_CTRL_COMMB_ADDR,
			PLAT_DMIC_DATA_IN3_COMMB_AND_SEL_MASK},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{PLAT_DMIC_DATA_OUT0_IO_CTRL_COMMB_ADDR,
			PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
		/* PDM_DMIC_OUT1 */
		{PLAT_DMIC_DATA_OUT1_IO_CTRL_COMMB_ADDR,
			PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{0, 0},
		/* PDM_DMIC_MONO_IN1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_DATA_IN2_IO_CTRL_COMMB_ADDR,
			PLAT_DMIC_MONO_DATA_IN2_AND_SEL_MASK},
		/* PDM_DMIC_MONO_IN3 */
		{0, 0},
	},
	{
		/* PDM_DMIC_IN0 */
		{PLAT_DMIC_DATA_IN0_IO_CTRL_GPIO_OTHR_ADDR,
			PLAT_DMIC_DATA_IN0_GPIO_OTHR_AND_SEL_MASK},
		/* PDM_DMIC_IN1 */
		{PLAT_DMIC_DATA_IN1_IO_CTRL_GPIO_OTHR_ADDR,
			PLAT_DMIC_DATA_IN1_GPIO_OTHR_AND_SEL_MASK},
		/* PDM_DMIC_IN2 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_GPIO_OTHR_ADDR,
			PLAT_DMIC_DATA_IN2_GPIO_OTHR_AND_SEL_MASK},
		/* PDM_DMIC_IN3 */
		{PLAT_DMIC_DATA_IN3_IO_CTRL_GPIO_OTHR_ADDR,
			PLAT_DMIC_DATA_IN3_GPIO_OTHR_AND_SEL_MASK},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{PLAT_DMIC_DATA_OUT0_IO_CTRL_GPIO_OTHR_ADDR,
			PLAT_DMIC_DATA_OUT0_MUX_SEL_VAL},
		/* PDM_DMIC_OUT1 */
		{PLAT_DMIC_DATA_OUT1_IO_CTRL_GPIO_OTHR_ADDR,
			PLAT_DMIC_DATA_OUT1_MUX_SEL_VAL},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{0, 0},
		/* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_DATA_IN2_IO_CTRL_GPIO_OTHR_ADDR,
			IAXXX_IO_CTRL_GPIO_1_CDC_ADC_IN1_AND_SEL_MASK},
		/* PDM_DMIC_MONO_IN2 */
		{0, 0},
		/* PDM_DMIC_MONO_IN3 */
		{0, 0},
	},
	{
		/* PDM_DMIC_IN0 */
		{0, 0},
		/* PDM_DMIC_IN1 */
		{0, 0},
		/* PDM_DMIC_IN2 */
		{0, 0},
		/* PDM_DMIC_IN3 */
		{0, 0},
		/* PDM_DMIC_IN4 */
		{0, 0},
		/* PDM_DMIC_IN5 */
		{0, 0},
		/* PDM_DMIC_IN6 */
		{0, 0},
		/* PDM_DMIC_IN7 */
		{0, 0},
		/* PDM_DMIC_OUT0 */
		{0, 0},
		/* PDM_DMIC_OUT1 */
		{0, 0},
		/* PDM_CDC_IN0 */
		{0, 0},
		/* PDM_CDC_IN1 */
		{0, 0},
		/* PDM_CDC_IN2 */
		{0, 0},
		/* PDM_CDC_IN3 */
		{0, 0},
		/* PDM_CDC_IN4 */
		{0, 0},
		/* PDM_CDC_IN5 */
		{0, 0},
		/* PDM_CDC_IN6 */
		{0, 0},
		/* PDM_CDC_IN7 */
		{0, 0},
		/* PDM_DAC_OUT0 */
		{0, 0},
		/* PDM_DAC_OUT1 */
		{0, 0},
		/* PDM_DMIC_MONO_IN0 */
		{0, 0},
		/* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_DATA_IN1_IO_CTRL_GPIO_COMMB_ADDR,
			PLAT_DMIC_MONO_DATA_IN1_GPIO_COMMB_AND_SEL_MASK},
		/* PDM_DMIC_MONO_IN2 */
		{0, 0},
		/* PDM_DMIC_MONO_IN3 */
		{0, 0},
	}
};

static const uint32_t iaxxx_ioctrl_clk_reset_val[PDM_NUM_IO_MICS] = {
	PLAT_DMIC_CLK_IN0_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN0 */
	PLAT_DMIC_CLK_IN1_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN1 */
	PLAT_DMIC_CLK_IN2_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN2 */
	PLAT_DMIC_CLK_IN3_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN3 */
	PLAT_DMIC_CLK_IN4_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN4 */
	PLAT_DMIC_CLK_IN5_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN5 */
	PLAT_DMIC_CLK_IN6_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN6 */
	PLAT_DMIC_CLK_IN7_IO_CTRL_RESET_VAL, /* PDM_DMIC_IN7 */
	PLAT_DMIC_CLK_OUT0_IO_CTRL_RESET_VAL, /* PDM_DMIC_OUT0 */
	PLAT_DMIC_CLK_OUT1_IO_CTRL_RESET_VAL, /* PDM_DMIC_OUT1 */
	PLAT_ADC_CLK_IN0_IO_CTRL_RESET_VAL, /* PDM_CDC_IN0 */
	PLAT_ADC_CLK_IN1_IO_CTRL_RESET_VAL, /* PDM_CDC_IN1 */
	PLAT_ADC_CLK_IN2_IO_CTRL_RESET_VAL, /* PDM_CDC_IN2 */
	PLAT_ADC_CLK_IN3_IO_CTRL_RESET_VAL, /* PDM_CDC_IN3 */
	PLAT_ADC_CLK_IN4_IO_CTRL_RESET_VAL, /* PDM_CDC_IN4 */
	PLAT_ADC_CLK_IN5_IO_CTRL_RESET_VAL, /* PDM_CDC_IN5 */
	PLAT_ADC_CLK_IN6_IO_CTRL_RESET_VAL, /* PDM_CDC_IN6 */
	PLAT_ADC_CLK_IN7_IO_CTRL_RESET_VAL, /* PDM_CDC_IN7 */
	PLAT_DAC_CLK_OUT0_IO_CTRL_RESET_VAL, /* PDM_DAC_OUT0 */
	PLAT_DAC_CLK_OUT1_IO_CTRL_RESET_VAL, /* PDM_DAC_OUT1 */
	PLAT_DMIC_MONO_CLK_IN0_IO_CTRL_RESET_VAL, /* PDM_DMIC_MONO_IN0 */
	PLAT_DMIC_MONO_CLK_IN1_IO_CTRL_RESET_VAL, /* PDM_DMIC_MONO_IN1 */
	PLAT_DMIC_MONO_CLK_IN2_IO_CTRL_RESET_VAL, /* PDM_DMIC_MONO_IN2 */
	PLAT_DMIC_MONO_CLK_IN3_IO_CTRL_RESET_VAL, /* PDM_DMIC_MONO_IN3 */
};

/* PDM port clock line mapping */
static const uint32_t iaxxx_dynamic_ioctrl_clkin_port
	[IAXXX_PORT_NUM_SEL][PDM_NUM_IO_MICS][3] = {
	/* PORT_A */
	{
		{PLAT_DMIC_CLK_IN0_PORTA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_PORTA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_PORTA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_PORTA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_PORTA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_PORTA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_PORTA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_PORTA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},

	/* PORT_B */
	{
		{PLAT_DMIC_CLK_IN0_PORTB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_PORTB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_PORTB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_PORTB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_PORTB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_PORTB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_PORTB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_PORTB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},

	/* PORT_C */
	{
		{PLAT_DMIC_CLK_IN0_PORTC_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_PORTC_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_PORTC_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_PORTC_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_PORTC_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_PORTC_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_PORTC_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_PORTC_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},

	/* COMM_A */
	{
		{PLAT_DMIC_CLK_IN0_COMMA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_COMMA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_COMMA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_COMMA_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_COMMA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_COMMA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_COMMA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_COMMA_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},

	/* COMM_B */
	{
		{PLAT_DMIC_CLK_IN0_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},

	/* GPIO_OTHR */
	{
		{PLAT_DMIC_CLK_IN0_GPIO_OTHR_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_GPIO_OTHR_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_GPIO_OTHR_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_GPIO_OTHR_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_GPIO_OTHR_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_GPIO_OTHR_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_GPIO_OTHR_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_GPIO_OTHR_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},

	/* GPIO_COMMB */
	{
		{PLAT_DMIC_CLK_IN0_GPIO_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN0 */
		{PLAT_DMIC_CLK_IN1_GPIO_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN1 */
		{PLAT_DMIC_CLK_IN2_GPIO_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN2 */
		{PLAT_DMIC_CLK_IN3_GPIO_COMMB_IO_CTRL_ADDR, PDM_CLK_SLAVE,
			PDM_CLK_MASTER}, /* PDM_DMIC_IN3 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN4 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN5 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN6 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_IN7 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT0 */
		{0, PDM_CLK_SLAVE, PDM_CLK_MASTER}, /* PDM_DMIC_OUT1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN1 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN2 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN3 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN4 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN5 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN6 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_CDC_IN7 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT0 */
		{0, CDC_CLK_SLAVE, CDC_CLK_MASTER}, /* PDM_DAC_OUT1 */
		{PLAT_DMIC_MONO_CLK_IN0_GPIO_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN0 */
		{PLAT_DMIC_MONO_CLK_IN1_GPIO_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN1 */
		{PLAT_DMIC_MONO_CLK_IN2_GPIO_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN2 */
		{PLAT_DMIC_MONO_CLK_IN3_GPIO_COMMB_IO_CTRL_ADDR, CDC_CLK_SLAVE,
			CDC_CLK_MASTER}, /* PDM_DMIC_MONO_IN3 */
	},
};

/* Table taken from FW PDM Driver */
static struct iaxxx_pdm_bit_cfg pdm_cfg[IAXXX_PDM_CLK_MAX] = {
	{ I2S_SRATE_NA, I2S_WORD_PER_FRAME_NA,
		I2S_WORD_LEN_NA  }, /* BIT_CLK_FREQ_NONE    */
	{ I2S_SRATE_8K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_16  }, /* BIT_CLK_FREQ_0_256M  */
	{ I2S_SRATE_8K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_0_512M  */
	{ I2S_SRATE_16K,   I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_24  }, /* BIT_CLK_FREQ_0_768M  */
	{ I2S_SRATE_16K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_1_024M  */
	{ I2S_SRATE_32K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_24  }, /* BIT_CLK_FREQ_1_536M  */
	{ I2S_SRATE_32K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_2_048M  */
	{ I2S_SRATE_32K, I2S_WORD_PER_FRAME_5,
		I2S_WORD_LEN_16  }, /* BIT_CLK_FREQ_2_560M  */
	{ I2S_SRATE_44K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_2_8224M */
	{ I2S_SRATE_48K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_3_072M  */
	{ I2S_SRATE_32K, I2S_WORD_PER_FRAME_5,
		I2S_WORD_LEN_24  }, /* BIT_CLK_FREQ_3_840M  */
	{ I2S_SRATE_32K, I2S_WORD_PER_FRAME_4,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_4_096M  */
	{ I2S_SRATE_88K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_5_6448M */
	{ I2S_SRATE_96K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_6_144M  */
	{ I2S_SRATE_96K, I2S_WORD_PER_FRAME_5,
		I2S_WORD_LEN_16  }, /* BIT_CLK_FREQ_7_680M  */
	{ I2S_SRATE_32K, I2S_WORD_PER_FRAME_8,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_8_192M  */
	{ I2S_SRATE_96K, I2S_WORD_PER_FRAME_5,
		I2S_WORD_LEN_24  }, /* BIT_CLK_FREQ_11_52M  */
	{ I2S_SRATE_192K, I2S_WORD_PER_FRAME_2,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_12_288M */
	{ I2S_SRATE_192K, I2S_WORD_PER_FRAME_4,
		I2S_WORD_LEN_32  }, /* BIT_CLK_FREQ_24_576M */
};

/* This table is two dimension array of CIC decimation and Green box(Half band)
 *  values with PDM_BCLK(rows) * AUD_PORT_CLK(columns)
 */
static struct iaxxx_cic_deci_table
	deci_rb_enable[IAXXX_PDM_CLK_MAX][IAXXX_AUD_PORT_MAX] = {
	{/* PDM_PORT_BIT_CLK_FREQ_NONE */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_0_256M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_8,  FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_0_512M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_0_768M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_24, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_12, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_12, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_6,  FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_6, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_1_024M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_1_536M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_24, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_12, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_6, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_6, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_2_048M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_2_560M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_2_8224M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FRQ_22_05K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/*PDM_PORT_BIT_CLK_FREQ_3_072M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_24, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_12, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_6, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_6, FILTER_CIC_HB_BY_2}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_3_840M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_4_096M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_2}  /*PDM_PORT_FREQ_256K*/
	},

	{/*PDM_PORT_BIT_CLK_FREQ_5_6448M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_6_144M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_24, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_12, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_6, FILTER_CIC_HB_BY_4}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_7_680M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_8_192M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_PT}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_12_288M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_24, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_12, FILTER_CIC_HB_BY_4}  /*PDM_PORT_FREQ_256K*/
	},

	{/* PDM_PORT_BIT_CLK_FREQ_24_576M */
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_NONE*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_8K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_11_025K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_12K*/
	{FILTER_CIC_DECIMATION_32, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_16K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_22_050K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_24K*/
	{FILTER_CIC_DECIMATION_16, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_32K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_44_1K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_48K*/
	{FILTER_CIC_DECIMATION_8, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_64*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_88_2K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_96K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_192K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_384K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_768K*/
	{FILTER_CIC_DECIMATION_NA, FILTER_CIC_HB_NA}, /*PDM_PORT_FREQ_1536K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_4}, /*PDM_PORT_FREQ_128K*/
	{FILTER_CIC_DECIMATION_4, FILTER_CIC_HB_BY_2}  /*PDM_PORT_FREQ_256K*/
	},
};

struct iaxxx_i2s_div_config {
	u32 N;
	u32 R;
	u32 HL;
	u32 period;
};

static struct iaxxx_i2s_div_config
	i2s_div_config[IAXXX_ACLK_FREQ_MAX][IAXXX_PDM_CLK_MAX] = {
	/* I2S_ACLK_FREQ_NONE */
	{
	/*	  N  R  HL  P		*/
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_NONE */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_0_256M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_0_512M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_0_768M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_1_024M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_1_536M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_2_048M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_2_560M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_2_8224M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_3_072M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_3_840M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_4_096M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_5_6448M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_6_144M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_7_680M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_8_192M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_11_52M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_12_288M */
		{ 0, 0, 0,  0 },          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_3072 */
	{
	/*	  N  R     HL  P	*/
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_NONE */
		{ 1, 4096, 3,  4 },          /* I2S_BIT_CLK_FREQ_0_256M */
		{ 1, 4096, 1,  6 },          /* I2S_BIT_CLK_FREQ_0_512M */
		{ 1, 4096, 1,  4 },          /* I2S_BIT_CLK_FREQ_0_768M */
		{ 1, 4096, 1,  3 },          /* I2S_BIT_CLK_FREQ_1_024M */
		{ 1, 4096, 1,  2 },          /* I2S_BIT_CLK_FREQ_1_536M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_048M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_560M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_8224M */
		{ 1, 4096, 1,  1 },          /* I2S_BIT_CLK_FREQ_3_072M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_3_840M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_4_096M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_5_6448M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_6_144M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_7_680M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_8_192M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_11_52M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_12_288M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_6144 */
	{
	/*	  N  R     HL  P	*/
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_NONE */
		{ 1, 4096, 6,  4 },          /* I2S_BIT_CLK_FREQ_0_256M */
		{ 1, 4096, 3,  4 },          /* I2S_BIT_CLK_FREQ_0_512M */
		{ 1, 4096, 2,  4 },          /* I2S_BIT_CLK_FREQ_0_768M */
		{ 1, 4096, 1,  6 },          /* I2S_BIT_CLK_FREQ_1_024M */
		{ 1, 4096, 1,  4 },          /* I2S_BIT_CLK_FREQ_1_536M */
		{ 1, 4096, 1,  3 },          /* I2S_BIT_CLK_FREQ_2_048M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_560M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_8224M */
		{ 1, 4096, 1,  2 },          /* I2S_BIT_CLK_FREQ_3_072M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_3_840M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_4_096M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_5_6448M */
		{ 1, 4096, 1,  1 },          /* I2S_BIT_CLK_FREQ_6_144M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_7_680M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_8_192M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_11_52M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_12_288M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_12288 */
	{
	/*	  N  R     HL  P	*/
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_NONE */
		{ 1, 4096, 12, 4 },          /* I2S_BIT_CLK_FREQ_0_256M */
		{ 1, 4096, 6,  4 },          /* I2S_BIT_CLK_FREQ_0_512M */
		{ 1, 4096, 4,  4 },          /* I2S_BIT_CLK_FREQ_0_768M */
		{ 1, 4096, 3,  4 },          /* I2S_BIT_CLK_FREQ_1_024M */
		{ 1, 4096, 2,  4 },          /* I2S_BIT_CLK_FREQ_1_536M */
		{ 1, 4096, 1,  6 },          /* I2S_BIT_CLK_FREQ_2_048M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_560M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_2_8224M */
		{ 1, 4096, 1,  4 },          /* I2S_BIT_CLK_FREQ_3_072M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_3_840M */
		{ 1, 4096, 3,  1 },          /* I2S_BIT_CLK_FREQ_4_096M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_5_6448M */
		{ 1, 4096, 2,  1 },          /* I2S_BIT_CLK_FREQ_6_144M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_7_680M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_8_192M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_11_52M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_12_288M */
		{ 0, 0,    0,  0 },          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_24576 */
	{
	/*	 N  R     HL  P		*/
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_NONE */
		{1, 4095, 12, 4},          /* I2S_BIT_CLK_FREQ_0_256M */
		{1, 4096, 12, 4},          /* I2S_BIT_CLK_FREQ_0_512M */
		{1, 4096, 8,  4},          /* I2S_BIT_CLK_FREQ_0_768M */
		{1, 4096, 6,  4},          /* I2S_BIT_CLK_FREQ_1_024M */
		{1, 4096, 4,  4},          /* I2S_BIT_CLK_FREQ_1_536M */
		{1, 4096, 3,  4},          /* I2S_BIT_CLK_FREQ_2_048M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_560M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_8224M */
		{1, 4096, 2,  4},          /* I2S_BIT_CLK_FREQ_3_072M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_3_840M */
		{1, 4096, 1,  6},          /* I2S_BIT_CLK_FREQ_4_096M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_5_6448M */
		{1, 4096, 1,  4},          /* I2S_BIT_CLK_FREQ_6_144M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_7_680M */
		{1, 4096, 3,  1},          /* I2S_BIT_CLK_FREQ_8_192M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_11_52M */
		{1, 4096, 2,  1},          /* I2S_BIT_CLK_FREQ_12_288M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_49152 */
	{
	/*	 N  R     HL  P		*/
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_NONE */
		{1, 4093, 12, 4},          /* I2S_BIT_CLK_FREQ_0_256M */
		{1, 4094, 8,  4},          /* I2S_BIT_CLK_FREQ_0_512M */
		{1, 4095, 8,  4},          /* I2S_BIT_CLK_FREQ_0_768M */
		{1, 4096, 12, 4},          /* I2S_BIT_CLK_FREQ_1_024M */
		{1, 4096, 8,  4},          /* I2S_BIT_CLK_FREQ_1_536M */
		{1, 4096, 6,  4},          /* I2S_BIT_CLK_FREQ_2_048M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_560M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_8224M */
		{1, 4096, 4,  4},          /* I2S_BIT_CLK_FREQ_3_072M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_3_840M */
		{1, 4096, 3,  4},          /* I2S_BIT_CLK_FREQ_4_096M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_5_6448M */
		{1, 4096, 2,  4},          /* I2S_BIT_CLK_FREQ_6_144M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_7_680M */
		{1, 4096, 1,  6},          /* I2S_BIT_CLK_FREQ_8_192M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_11_52M */
		{1, 4096, 1,  4},          /* I2S_BIT_CLK_FREQ_12_288M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_98304 */
	{
	/*	 N  R     HL  P		*/
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_NONE */
		{1, 4089, 12, 4},          /* I2S_BIT_CLK_FREQ_0_256M */
		{1, 4091, 8,  4},          /* I2S_BIT_CLK_FREQ_0_512M */
		{1, 4093, 8,  4},          /* I2S_BIT_CLK_FREQ_0_768M */
		{1, 4094, 8,  4},          /* I2S_BIT_CLK_FREQ_1_024M */
		{1, 4095, 8,  4},          /* I2S_BIT_CLK_FREQ_1_536M */
		{1, 4096, 12, 4},          /* I2S_BIT_CLK_FREQ_2_048M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_560M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_8224M */
		{1, 4096, 8,  4},          /* I2S_BIT_CLK_FREQ_3_072M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_3_840M */
		{1, 4096, 6,  4},          /* I2S_BIT_CLK_FREQ_4_096M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_5_6448M */
		{1, 4096, 4,  4},          /* I2S_BIT_CLK_FREQ_6_144M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_7_680M */
		{1, 4096, 3,  4},          /* I2S_BIT_CLK_FREQ_8_192M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_11_52M */
		{1, 4096, 2,  4},          /* I2S_BIT_CLK_FREQ_12_288M */
		{1, 4096, 1,  4},          /* I2S_BIT_CLK_FREQ_24_576M */
	},

	/* I2S_ACLK_FREQ_368640 */
	{
	/*	 N  R     HL  P		*/
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_NONE */
		{1, 4067, 12, 4},          /* I2S_BIT_CLK_FREQ_0_256M */
		{1, 4082, 12, 4},          /* I2S_BIT_CLK_FREQ_0_512M */
		{1, 4082, 8,  4},          /* I2S_BIT_CLK_FREQ_0_768M */
		{1, 4082, 6,  4},          /* I2S_BIT_CLK_FREQ_1_024M */
		{1, 4092, 12, 4},          /* I2S_BIT_CLK_FREQ_1_536M */
		{1, 4094, 15, 4},          /* I2S_BIT_CLK_FREQ_2_048M */
		{1, 4085, 3,  4},          /* I2S_BIT_CLK_FREQ_2_560M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_2_8224M */
		{1, 4092, 6,  4},          /* I2S_BIT_CLK_FREQ_3_072M */
		{1, 4093, 6,  4},          /* I2S_BIT_CLK_FREQ_3_840M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_4_096M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_5_6448M */
		{1, 4096, 15, 4},          /* I2S_BIT_CLK_FREQ_6_144M */
		{1, 4096, 12, 4},          /* I2S_BIT_CLK_FREQ_7_680M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_8_192M */
		{1, 4096, 8,  4},          /* I2S_BIT_CLK_FREQ_11_52M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_12_288M */
		{0, 0,    0,  0},          /* I2S_BIT_CLK_FREQ_24_576M */
	},
};

int get_decimator_val(u32 port_bclk, u32 aud_port_clk,
			u32 *cic_dec, u32 *hb_deci)
{
	int ret = -EINVAL;

	if (port_bclk >= IAXXX_PDM_CLK_MAX)
		return ret;

	if (aud_port_clk >= IAXXX_AUD_PORT_MAX)
		return ret;

	if ((cic_dec == NULL) || (hb_deci == NULL))
		return ret;

	*cic_dec = deci_rb_enable[port_bclk][aud_port_clk].cic_dec;

	/* Not supported value */
	if (*cic_dec == FILTER_CIC_DECIMATION_NA)
		return ret;

	*hb_deci = deci_rb_enable[port_bclk][aud_port_clk].hb_dec;

	return 0;
}

static const u32 port_clk_addr[] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_COMMA_0_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
};

static const u32 port_fs_addr[] = {
	IAXXX_IO_CTRL_PORTA_FS_ADDR,
	IAXXX_IO_CTRL_PORTB_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_COMMA_1_ADDR,
	IAXXX_IO_CTRL_COMMB_1_ADDR,
};

static const u32 port_di_addr[] = {
	IAXXX_IO_CTRL_PORTA_DI_ADDR,
	IAXXX_IO_CTRL_PORTB_DI_ADDR,
	IAXXX_IO_CTRL_PORTC_DI_ADDR,
	IAXXX_IO_CTRL_COMMA_2_ADDR,
	IAXXX_IO_CTRL_COMMB_2_ADDR,
};

static const u32 port_do_addr[] = {
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_COMMA_3_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
};

enum {
	IAXXX_AIF0 = 0,
	IAXXX_AIF1,
	IAXXX_AIF2,
	IAXXX_NUM_CODEC_DAIS,
};

enum {
	STREAM0 = 0,
	STREAM1,
	STREAM2,
	STREAM3,
	STREAM4,
	STREAM5,
	STREAM6,
	STREAM7,
	STREAM_NONE,
	MAX_STREAM = STREAM_NONE,
};

enum Encoding_s {
	ENCODING_OPAQUE		= 0x00,
	ENCODING_AFLOAT		= 0x01,
	ENCODING_G711A		= 0x02,
	ENCODING_G711U		= 0x03,
	ENCODING_FLOAT		= 0x04,
	ENCODING_Q15		= 0x0F,
	ENCODING_Q16		= 0x10,
	ENCODING_Q17		= 0x11,
	ENCODING_Q18		= 0x12,
	ENCODING_Q19		= 0x13,
	ENCODING_Q20		= 0x14,
	ENCODING_Q21		= 0x15,
	ENCODING_Q22		= 0x16,
	ENCODING_Q23		= 0x17,
	ENCODING_Q24		= 0x18,
	ENCODING_Q25		= 0x19,
	ENCODING_Q26		= 0x1A,
	ENCODING_Q27		= 0x1B,
	ENCODING_Q28		= 0x1C,
	ENCODING_Q29		= 0x1D,
	ENCODING_Q30		= 0x1E,
	ENCODING_Q31		= 0x1F,
	ENCODING_ERROR		= 0xFFFF,
};

enum Rate_s {
	RATE_8K = 0x0,
	RATE_11P025K = 0x1,
	RATE_12K = 0x2,
	RATE_16K = 0x3,
	RATE_22P050K = 0x4,
	RATE_24K = 0x5,
	RATE_32K = 0x6,
	RATE_44P1K = 0x7,
	RATE_48K = 0x8,
	RATE_64K = 0x9,
	RATE_88P2K = 0xA,
	RATE_96K = 0xB,
	RATE_176P4K = 0xC,
	RATE_192K = 0xD,
	RATE_384K = 0xE,
	RATE_768K = 0xF,
	RATE_INVALID = 0xFF,
};

enum Gain_Ramp_Step {
	STEP_0 = 0x0,
	STEP_300 = 0x12C,
	STEP_600 = 0x258,
	STEP_900 = 0x384,
	STEP_1200 = 0x4B0,
	STEP_1600 = 0x640,
	STEP_2000 = 0x7D0,
	STEP_INST = 0xFFFF,
};

static const char * const master_src_texts[] = {
	"I2S_GEN0",
	"I2S_GEN1",
	"I2S_GEN2",
	"I2S_GEN3",
	"I2S_GEN4",
	"I2S_GEN5",
	"I2S_GEN6",
};

static const struct soc_enum iaxxx_master_src_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(master_src_texts),
			master_src_texts);

static const char * const pdm_bclk_texts[] = {
	"IAXXX_PDM_CLK_NONE",
	"IAXXX_PDM_CLK_0P_256MHZ",
	"IAXXX_PDM_CLK_0P_512MHZ",
	"IAXXX_PDM_CLK_0P_768MHZ",
	"IAXXX_PDM_CLK_1P_024MHZ",
	"IAXXX_PDM_CLK_1P_536MHZ",
	"IAXXX_PDM_CLK_2P_048MHZ",
	"IAXXX_PDM_CLK_2P_560MHZ",
	"IAXXX_PDM_CLK_2P_822MHZ",
	"IAXXX_PDM_CLK_3P_072MHZ",
	"IAXXX_PDM_CLK_3P_840MHZ",
	"IAXXX_PDM_CLK_4P_096MHZ",
	"IAXXX_PDM_CLK_5P_644MHZ",
	"IAXXX_PDM_CLK_6P_144MHZ",
	"IAXXX_PDM_CLK_7P_680MHZ",
	"IAXXX_PDM_CLK_8P_192MHZ",
	"IAXXX_PDM_CLK_11P_520MHZ",
	"IAXXX_PDM_CLK_12P_288MHZ",
	"IAXXX_PDM_CLK_24P_576MHZ",
};

static const struct soc_enum iaxxx_pdm_bclk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_bclk_texts),
		pdm_bclk_texts);

static const char * const pdm_aud_port_clk_texts[] = {
	"IAXXX_AUD_PORT_NONE",
	"IAXXX_AUD_PORT_8K",
	"IAXXX_AUD_PORT_11_025K",
	"IAXXX_AUD_PORT_12K",
	"IAXXX_AUD_PORT_16K",
	"IAXXX_AUD_PORT_22_05K",
	"IAXXX_AUD_PORT_24K",
	"IAXXX_AUD_PORT_32K",
	"IAXXX_AUD_PORT_44_1K",
	"IAXXX_AUD_PORT_48K",
	"IAXXX_AUD_PORT_64K",
	"IAXXX_AUD_PORT_88_2K",
	"IAXXX_AUD_PORT_96K",
	"IAXXX_AUD_PORT_176_4K",
	"IAXXX_AUD_PORT_192K",
	"IAXXX_AUD_PORT_384K",
	"IAXXX_AUD_PORT_768K",
	"IAXXX_AUD_PORT_1536K",
	"IAXXX_AUD_PORT_128K",
	"IAXXX_AUD_PORT_256K",
};

static const struct soc_enum iaxxx_pdm_aud_port_clk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_aud_port_clk_texts),
			pdm_aud_port_clk_texts);

static const char * const apll_clk_texts[] = {
	"IAXXX_ACLK_FREQ_NONE",
	"IAXXX_ACLK_FREQ_3072",
	"IAXXX_ACLK_FREQ_6144",
	"IAXXX_ACLK_FREQ_12288",
	"IAXXX_ACLK_FREQ_24576",
	"IAXXX_ACLK_FREQ_49152",
	"IAXXX_ACLK_FREQ_98304",
	"IAXXX_ACLK_FREQ_368640",
};

static const char * const apll_input_freq_texts[] = {
	"APLL_SRC_FREQ_NONE",
	"APLL_SRC_FREQ_512",          /*!< 512     KHz */
	"APLL_SRC_FREQ_768",          /*!< 768     KHz */
	"APLL_SRC_FREQ_1024",         /*!< 1.024   MHz */
	"APLL_SRC_FREQ_1536",         /*!< 1.536   MHz */
	"APLL_SRC_FREQ_2048",         /*!< 2.048   MHz */
	"APLL_SRC_FREQ_2400",         /*!< 2.400   MHz */
	"APLL_SRC_FREQ_3072",         /*!< 3.072   MHz */
	"APLL_SRC_FREQ_4608",         /*!< 4.608   MHz */
	"APLL_SRC_FREQ_4800",         /*!< 4.800   MHz */
	"APLL_SRC_FREQ_6144",         /*!< 6.144   MHz */
	"APLL_SRC_FREQ_9600",         /*!< 9.600   MHz */
	"APLL_SRC_FREQ_12288",        /*!< 12.288  MHz */
	"APLL_SRC_FREQ_19200",        /*!< 19.200  MHz */
	"APLL_SRC_FREQ_24576",        /*!< 24.576  MHz */
};

static const char * const apll_src_texts[] = {
	"PLL_SRC_NONE",
	"PLL_SRC_SYS_CLK",       /*!< System Clock        - supported      */
	"PLL_SRC_PORTA_CLK",     /*!< io_ref0_clk2cnr_i   - supported      */
	"PLL_SRC_PORTA_DO",      /*!< io_ref1_clk2cnr_i   - Not supported  */
	"PLL_SRC_PORTB_CLK",     /*!< io_ref2_clk2cnr_i   - supported      */
	"PLL_SRC_PORTB_DO",      /*!< io_ref3_clk2cnr_i   - Not supported  */
	"PLL_SRC_PORTC_CLK",     /*!< io_ref4_clk2cnr_i   - supported      */
	"PLL_SRC_PORTC_DO",      /*!< io_ref5_clk2cnr_i   - Not supported  */
	"PLL_SRC_COMMB_3",       /*!< io_ref6_clk2cnr_i   - Not supported  */
	"PLL_SRC_OSC_CLK",       /*!< Internal Oscillator - supported      */
	"PLL_SRC_EXT_CLK",       /*!< External clock      - supported      */
};

static const struct soc_enum iaxxx_apll_clk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(apll_clk_texts),
								apll_clk_texts);
static const struct soc_enum iaxxx_apll_input_freq_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(apll_input_freq_texts),
						apll_input_freq_texts);

static const struct soc_enum iaxxx_apll_src_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(apll_src_texts),
						apll_src_texts);

static const char * const pdm_clr_texts[] = {
	"NONE",
	"PDM_DMIC_IN0",
	"PDM_DMIC_IN1",
	"PDM_DMIC_IN2",
	"PDM_DMIC_IN3",
	"PDM_DMIC_IN4",
	"PDM_DMIC_IN5",
	"PDM_DMIC_IN6",
	"PDM_DMIC_IN7",
	"PDM_DMIC_OUT0",
	"PDM_DMIC_OUT1",
	"PDM_CDC0_IN0",
	"PDM_CDC1_IN1",
	"PDM_CDC2_IN2",
	"PDM_CDC3_IN3",
	"PDM_CDC0_IN4",
	"PDM_CDC1_IN5",
	"PDM_CDC2_IN6",
	"PDM_CDC3_IN7",
	"PDM_CDC_DAC_OUT0",
	"PDM_CDC_DAC_OUT1",
};

static const struct soc_enum iaxxx_pdm_clr_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_clr_texts),
					pdm_clr_texts);

static const char * const pdm_port_sel_texts[] = {
	"NONE",
	"IAXXX_PORT_A",
	"IAXXX_PORT_B",
	"IAXXX_PORT_C",
	"IAXXX_COMM_A",
	"IAXXX_COMM_B",
	"IAXXX_GPIO_OTHR",
	"IAXXX_GPIO_COMMB",
};

static const struct soc_enum iaxxx_pdm_port_sel_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_port_sel_texts),
			pdm_port_sel_texts);

static const char * const port_clk_stop_texts[] = {
	"PORTA", "PORTB", "PORTC",
};
static const struct soc_enum iaxxx_port_clk_stop_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(port_clk_stop_texts),
						port_clk_stop_texts);

#define ENUM_NAME(NAME) (#NAME)

/* Port values are 16 bit systemID's of Ports
 * 15-12 bits are Pheripheral type which is 1
 * 11-7 bits are which pheripheral type like PCM, PDM
 * 6-0 bits are instance index of port.
 */
static const unsigned int io_port_value[] = {
	 0x0,
	IAXXX_SYSID_PCM0, IAXXX_SYSID_PCM1, IAXXX_SYSID_PCM2,
	IAXXX_SYSID_PDMI0, IAXXX_SYSID_PDMI1, IAXXX_SYSID_PDMI2,
	IAXXX_SYSID_PDMI3, IAXXX_SYSID_PDMI4, IAXXX_SYSID_PDMI5,
	IAXXX_SYSID_PDMI6, IAXXX_SYSID_PDMI7, IAXXX_SYSID_PDMO0,
};

static const char * const io_port_texts[] = {
	"NONE",
	"PCM0", "PCM1", "PCM2",
	"PDMI0", "PDMI1", "PDMI2", "PDMI3",
	"PDMI4", "PDMI5", "PDMI6", "PDMI7",
	"PDMO0",
};

static const struct soc_enum iaxxx_strm_port_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(io_port_texts),
					io_port_texts);

static const unsigned int strm_ch_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, };

static const char * const strm_ch_idx_texts[] = {
	ENUM_NAME(STRM_CH0), ENUM_NAME(STRM_CH1), ENUM_NAME(STRM_CH2),
	ENUM_NAME(STRM_CH3), ENUM_NAME(STRM_CH4), ENUM_NAME(STRM_CH5),
	ENUM_NAME(STRM_CH6), ENUM_NAME(STRM_CH7),
	ENUM_NAME(STRM_CH_NONE), };

static const unsigned int port_ch_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, };

static const char * const port_ch_idx_texts[] = {
	ENUM_NAME(PORT_CH0), ENUM_NAME(PORT_CH1), ENUM_NAME(PORT_CH2),
	ENUM_NAME(PORT_CH3), ENUM_NAME(PORT_CH4), ENUM_NAME(PORT_CH5),
	ENUM_NAME(PORT_CH6), ENUM_NAME(PORT_CH7),};

static const unsigned int gain_ramp_value[] = {
	STEP_0,
	STEP_300, STEP_600, STEP_900, STEP_1200,
	STEP_1600, STEP_2000, STEP_INST, };

static const char * const gain_ramp_texts[] = {
	ENUM_NAME(STEP_0),
	ENUM_NAME(STEP_300), ENUM_NAME(STEP_600), ENUM_NAME(STEP_900),
	ENUM_NAME(STEP_1200), ENUM_NAME(STEP_1600), ENUM_NAME(STEP_2000),
	ENUM_NAME(STEP_INST), };

static const struct soc_enum iaxxx_ch_gain_ramp_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(gain_ramp_texts),
			gain_ramp_texts);

static const unsigned int str_frm_len_values[] = {
	0x0,
	0x20, 0x40, 0xA0,
	0x140, 0x200, 0x280,
	0x640, 0xC80, 0x1900,

	0x40, 0x80, 0x140,
	0x280, 0x320, 0x400,
	0x500, 0xC80, 0x1900,
	0x3200,

	0x60, 0xC0, 0x1E0,
	0x3C0, 0x600, 0x780,
	0x12C0, 0x2580, 0x4B00,

	0x80, 0x100, 0x280,
	0x500, 0x800, 0xA00,
	0x1900, 0x3200, 0x6400,

	0xB0, 0x160, 0x372,
	0x6E0, 0xB00, 0xDC0,
	0x2260, 0x44C0, 0x8980,

	0xC0, 0x180, 0x3C0,
	0x780, 0xC00, 0xF00,
	0x2580, 0x4B00, 0x9600,

	0x180, 0x300, 0x780,
	0xF00, 0x1800, 0x1E00,
	0x4B00, 0x9600,

	0x300, 0x600, 0xF00,
	0x1E00, 0x3000, 0x3C00,
	0x9600,
};

static const char * const str_frm_len_text[] = {
	ENUM_NAME(NONE),
	ENUM_NAME(FRAME_8K1MS), ENUM_NAME(FRAME_8K2MS),
	ENUM_NAME(FRAME_8K5MS),
	ENUM_NAME(FRAME_8K10MS), ENUM_NAME(FRAME_8K16MS),
	ENUM_NAME(FRAME_8K20MS),
	ENUM_NAME(FRAME_8K50MS), ENUM_NAME(FRAME_8K100MS),
	ENUM_NAME(FRAME_8K200MS),

	ENUM_NAME(FRAME_16K1MS), ENUM_NAME(FRAME_16K2MS),
	ENUM_NAME(FRAME_16K5MS),
	ENUM_NAME(FRAME_16K10MS), ENUM_NAME(FRAME_16K12.5MS),
	ENUM_NAME(FRAME_16K16MS), ENUM_NAME(FRAME_16K20MS),
	ENUM_NAME(FRAME_16K50MS), ENUM_NAME(FRAME_16K100MS),
	ENUM_NAME(FRAME_16K200MS),

	ENUM_NAME(FRAME_24K1MS), ENUM_NAME(FRAME_24K2MS),
	ENUM_NAME(FRAME_24K5MS),
	ENUM_NAME(FRAME_24K10MS), ENUM_NAME(FRAME_24K16MS),
	ENUM_NAME(FRAME_24K20MS),
	ENUM_NAME(FRAME_24K50MS), ENUM_NAME(FRAME_24K100MS),
	ENUM_NAME(FRAME_24K200MS),

	ENUM_NAME(FRAME_32K1MS), ENUM_NAME(FRAME_32K2MS),
	ENUM_NAME(FRAME_32K5MS),
	ENUM_NAME(FRAME_32K10MS), ENUM_NAME(FRAME_32K16MS),
	ENUM_NAME(FRAME_32K20MS),
	ENUM_NAME(FRAME_32K50MS), ENUM_NAME(FRAME_32K100MS),
	ENUM_NAME(FRAME_32K200MS),

	ENUM_NAME(FRAME_44.1K1MS), ENUM_NAME(FRAME_44.1K2MS),
	ENUM_NAME(FRAME_44.1K5MS),
	ENUM_NAME(FRAME_44.1K10MS), ENUM_NAME(FRAME_44.1K16MS),
	ENUM_NAME(FRAME_44.1K20MS),
	ENUM_NAME(FRAME_44.1K50MS), ENUM_NAME(FRAME_44.1K100MS),
	ENUM_NAME(FRAME_44.1K200MS),

	ENUM_NAME(FRAME_48K1MS), ENUM_NAME(FRAME_48K2MS),
	ENUM_NAME(FRAME_48K5MS),
	ENUM_NAME(FRAME_48K10MS), ENUM_NAME(FRAME_48K16MS),
	ENUM_NAME(FRAME_48K20MS),
	ENUM_NAME(FRAME_48K50MS), ENUM_NAME(FRAME_48K100MS),
	ENUM_NAME(FRAME_48K200MS),

	ENUM_NAME(FRAME_96K1MS), ENUM_NAME(FRAME_96K2MS),
	ENUM_NAME(FRAME_96K5MS),
	ENUM_NAME(FRAME_96K10MS), ENUM_NAME(FRAME_96K16MS),
	ENUM_NAME(FRAME_96K20MS),
	ENUM_NAME(FRAME_96K50MS), ENUM_NAME(FRAME_96K100MS),

	ENUM_NAME(FRAME_192K1MS), ENUM_NAME(FRAME_192K2MS),
	ENUM_NAME(FRAME_192K5MS),
	ENUM_NAME(FRAME_192K10MS), ENUM_NAME(FRAME_192K16MS),
	ENUM_NAME(FRAME_192K20MS),
	ENUM_NAME(FRAME_192K50MS),
};

static const struct soc_enum iaxxx_frm_len_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(str_frm_len_text),
						str_frm_len_text);

static const char * const str_rate_text[] = {
	ENUM_NAME(RATE_8K),
	ENUM_NAME(RATE_11P025K),
	ENUM_NAME(RATE_12K),
	ENUM_NAME(RATE_16K),
	ENUM_NAME(RATE_22P050K),
	ENUM_NAME(RATE_24K),
	ENUM_NAME(RATE_32K),
	ENUM_NAME(RATE_44P1K),
	ENUM_NAME(RATE_48K),
	ENUM_NAME(RATE_64K),
	ENUM_NAME(RATE_88P2K),
	ENUM_NAME(RATE_96K),
	ENUM_NAME(RATE_176P4K),
	ENUM_NAME(RATE_192K),
	ENUM_NAME(RATE_384K),
	ENUM_NAME(RATE_768K),
	ENUM_NAME(RATE_INVALID),
};

static const unsigned int str_rate_values[] = {
	RATE_8K,
	RATE_11P025K,
	RATE_12K,
	RATE_16K,
	RATE_22P050K,
	RATE_24K,
	RATE_32K,
	RATE_44P1K,
	RATE_48K,
	RATE_64K,
	RATE_88P2K,
	RATE_96K,
	RATE_176P4K,
	RATE_192K,
	RATE_384K,
	RATE_768K,
	RATE_INVALID,
};

static const struct soc_enum iaxxx_strm_sr_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(str_rate_text),
						str_rate_text);

/* supported stream encodings */
static const unsigned int str_enc_values[] = {
	ENCODING_OPAQUE,
	ENCODING_AFLOAT,
	ENCODING_G711A,
	ENCODING_G711U,
	ENCODING_FLOAT,
	ENCODING_Q15,
	ENCODING_Q16,
	ENCODING_Q17,
	ENCODING_Q18,
	ENCODING_Q19,
	ENCODING_Q20,
	ENCODING_Q21,
	ENCODING_Q22,
	ENCODING_Q23,
	ENCODING_Q24,
	ENCODING_Q25,
	ENCODING_Q26,
	ENCODING_Q27,
	ENCODING_Q28,
	ENCODING_Q29,
	ENCODING_Q30,
	ENCODING_Q31,
	ENCODING_ERROR,
};

static const char * const str_enc_text[] = {
	ENUM_NAME(ENCODING_OPAQUE),
	ENUM_NAME(ENCODING_AFLOAT),
	ENUM_NAME(ENCODING_G711A),
	ENUM_NAME(ENCODING_G711U),
	ENUM_NAME(ENCODING_FLOAT),
	ENUM_NAME(ENCODING_Q15),
	ENUM_NAME(ENCODING_Q16),
	ENUM_NAME(ENCODING_Q17),
	ENUM_NAME(ENCODING_Q18),
	ENUM_NAME(ENCODING_Q19),
	ENUM_NAME(ENCODING_Q20),
	ENUM_NAME(ENCODING_Q21),
	ENUM_NAME(ENCODING_Q22),
	ENUM_NAME(ENCODING_Q23),
	ENUM_NAME(ENCODING_Q24),
	ENUM_NAME(ENCODING_Q25),
	ENUM_NAME(ENCODING_Q26),
	ENUM_NAME(ENCODING_Q27),
	ENUM_NAME(ENCODING_Q28),
	ENUM_NAME(ENCODING_Q29),
	ENUM_NAME(ENCODING_Q30),
	ENUM_NAME(ENCODING_Q31),
	ENUM_NAME(ENCODING_ERROR),
};

static const struct soc_enum iaxxx_str_enc_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(str_enc_text),
			str_enc_text);

static const char * const pdm_mic_en_texts[] = {
	"DMIC0_CLK_SRC",
	"CDC0_CLK_SRC",
	"DMIC1_CLK_SRC",
	"CDC1_CLK_SRC",
	"DISABLE",
};

static const struct soc_enum iaxxx_pdm_mic_en_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_mic_en_texts),
			pdm_mic_en_texts);

#define IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(channel, channel_name) \
static const char * const channel##_rx_off_on_texts[] = { \
	"Off", \
	"Rx0"channel_name"On", "Rx1"channel_name"On", \
	"Rx2"channel_name"On", "Rx3"channel_name"On", \
	"Rx4"channel_name"On", "Rx5"channel_name"On", \
	"Rx6"channel_name"On", "Rx7"channel_name"On", \
	"Rx8"channel_name"On", "Rx9"channel_name"On", \
	"Rx10"channel_name"On", "Rx11"channel_name"On", \
	"Rx12"channel_name"On", "Rx13"channel_name"On", \
	"Rx14"channel_name"On", "Rx15"channel_name"On", \
	"Plgin0"channel_name"On", "Plgin1"channel_name"On", \
	"Plgin2"channel_name"On", "Plgin3"channel_name"On", \
	"Plgin4"channel_name"On", "Plgin5"channel_name"On", \
	"Plgin6"channel_name"On", "Plgin7"channel_name"On", \
	"Plgin8"channel_name"On", "Plgin9"channel_name"On", \
	"Plgin10"channel_name"On", "Plgin11"channel_name"On", \
	"Plgin12"channel_name"On", "Plgin13"channel_name"On", \
	"Plgin14"channel_name"On" \
}

IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_0, "Tx0");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_1, "Tx1");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_2, "Tx2");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_3, "Tx3");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_4, "Tx4");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_5, "Tx5");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_6, "Tx6");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_7, "Tx7");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_8, "Tx8");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_9, "Tx9");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_10, "Tx10");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_11, "Tx11");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_12, "Tx12");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_13, "Tx13");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_14, "Tx14");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_15, "Tx15");

static const char * const iaxxx_route_status_texts[] = {
	"InActive", "Active"
};

static const struct soc_enum iaxxx_route_status_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(iaxxx_route_status_texts),
			iaxxx_route_status_texts);

static const char * const iaxxx_power_mode_texts[] = {
	"LOW_POWER", "NORMAL"
};

static const struct soc_enum iaxxx_power_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(iaxxx_power_mode_texts),
			iaxxx_power_mode_texts);

static const char * const pdm_clk_drive_strength_texts[] = {
	"2MA",
	"4MA",
	"8MA",
	"12MA",
};

static const struct soc_enum iaxxx_pdm_clk_drive_strength_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
		ARRAY_SIZE(pdm_clk_drive_strength_texts),
		pdm_clk_drive_strength_texts);

/**
 * iaxxx_snd_soc_info_multi_ext - external single mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a single external mixer control.
 * that accepts multiple input.
 *
 * Returns 0 for success.
 */
static int iaxxx_snd_soc_info_multi_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct iaxxx_soc_multi_mixer_control *mc =
		(struct iaxxx_soc_multi_mixer_control *)kcontrol->private_value;
	int platform_max;

	if (!mc->platform_max)
		mc->platform_max = mc->max;
	platform_max = mc->platform_max;

	if (platform_max == 1 && !strnstr(kcontrol->id.name, " Volume", 30))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = mc->count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = platform_max;
	return 0;
}

static int iaxxx_get_pdm_bclk(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->pdm_bclk;
	return 0;
}

static int iaxxx_put_pdm_bclk(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	iaxxx->pdm_bclk = ucontrol->value.enumerated.item[0];
	return 0;
}

static int iaxxx_get_apll_clk(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->apll_clk;
	return 0;
}

static int iaxxx_put_apll_clk(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	if (iaxxx->apll_clk == ucontrol->value.enumerated.item[0] ||
				iaxxx->i2s_master_clk)
		return 0;

	if (!ucontrol->value.enumerated.item[0])
		iaxxx->apll_clk = priv->sys_clk_out_freq;
	else
		iaxxx->apll_clk = ucontrol->value.enumerated.item[0];

	return 0;
}

static int iaxxx_get_apll_src(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->apll_src;
	return 0;
}

static int iaxxx_put_apll_src(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	int ret = 0;
	int val = ucontrol->value.enumerated.item[0];

	iaxxx->apll_src = val;

	/* If APLL clock is bypassed, return */
	if (iaxxx->i2s_master_clk)
		return 0;

	/*
	 * If Apll source is None then update the parameters
	 * as per system clock and frequency.
	 */
	if (iaxxx->apll_src == PLL_SRC_NONE) {
		iaxxx->apll_input_freq = priv->sys_clk_in_freq;
		iaxxx->apll_clk = priv->sys_clk_out_freq;
		iaxxx->apll_src = priv->sys_clk_src;
	}

	ret = iaxxx_config_apll(priv, iaxxx->apll_src, iaxxx->apll_clk,
			  iaxxx->apll_input_freq);
	if (ret) {
		iaxxx->apll_input_freq = 0;
		iaxxx->apll_clk = 0;
		iaxxx->apll_src = 0;
	}

	return ret;
}

static int iaxxx_get_apll_input_freq(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->apll_input_freq;
	return 0;
}

static int iaxxx_put_apll_input_freq(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	if (iaxxx->apll_input_freq == ucontrol->value.enumerated.item[0] ||
		iaxxx->i2s_master_clk)
		return 0;

	if (!ucontrol->value.enumerated.item[0])
		iaxxx->apll_input_freq = priv->sys_clk_in_freq;
	else
		iaxxx->apll_input_freq = ucontrol->value.enumerated.item[0];
	return 0;
}

static int iaxxx_get_system_clk_input_freq(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	dev_info(priv->dev, "%s(): input clock value %d\n",
		 __func__, priv->sys_clk_in_freq);

	ucontrol->value.enumerated.item[0] = priv->sys_clk_in_freq;
	return 0;
}

static int iaxxx_put_system_clk_input_freq(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	/*
	 * Default APLL input frequency can be modified only if default clock
	 * source is external clock
	 */
	if (priv->sys_clk_src == PLL_SRC_EXT_CLK) {
		priv->sys_clk_in_freq = ucontrol->value.enumerated.item[0];

		dev_info(priv->dev, "%s(): input clock value set to %d\n",
			 __func__, priv->sys_clk_in_freq);
	} else {
		dev_info(priv->dev, "%s(): input clock frequency can't be modified\n",
			 __func__);
	}

	return 0;
}

static int iaxxx_get_bypass_apll(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->i2s_master_clk;
	return 0;
}

static int iaxxx_put_bypass_apll(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	u32 aclk_freq;
	u32 in_freq;
	u32 out_freq;
	u32 clk_src;
	u32 i2s_master_freq;
	u32 i2s_master_clk;
	u32 status;
	int i;
	int ret;

	if (iaxxx->i2s_master_clk == ucontrol->value.enumerated.item[0])
		return 0;

	/*
	 * If there is no APLL clock change, use values read and stored
	 * after bootup, otherwise use the updated values
	 */
	if (!iaxxx->apll_src) {
		in_freq = priv->sys_clk_in_freq;
		clk_src = priv->sys_clk_src;
		out_freq = priv->sys_clk_out_freq;
	} else {
		in_freq = iaxxx->apll_input_freq;
		clk_src = iaxxx->apll_src;
		out_freq = iaxxx->apll_clk;
	}

	/*
	 * During bypass APLL reset, the I2S clock will be
	 * same as output frequency
	 */
	if (!ucontrol->value.enumerated.item[0]) {
		i2s_master_clk = out_freq;
	} else {
		i2s_master_clk = ucontrol->value.enumerated.item[0];
		/*
		 * Find Output frequency index for input frequency,
		 * to set output and input frequency same
		 */
		for (i = 0; i < IAXXX_ACLK_FREQ_MAX; i++) {
			if (iaxxx_apll_in_freq_val[in_freq] ==
					iaxxx_apllClk_Val[i]) {
				out_freq = i;
				break;
			}
		}

		if (i == IAXXX_ACLK_FREQ_MAX || !out_freq) {
			pr_err(
			"dev id-%d: Input freq %d match is not available in out freq",
			       priv->dev_id, iaxxx_apll_in_freq_val[in_freq]);
			return -EINVAL;
		}
	}

	ret = iaxxx_config_apll(priv, clk_src, out_freq, in_freq);
	if (ret) {
		pr_err("dev id-%d: config APLL failed : %d", priv->dev_id, ret);
		return ret;
	}

	aclk_freq = iaxxx_apllClk_Val[out_freq];
	i2s_master_freq = iaxxx_apllClk_Val[i2s_master_clk];
	dev_info(codec->dev, "%s: Aclk freq %d and I2S freq %d\n",
		 __func__, aclk_freq, i2s_master_freq);
	/*
	 * Set the I2S Master skip count if i2sMasterFreq can be derived from
	 * the A_CLK frequency (if A_CLK can be divided exactly). For proper
	 * audio quality, it is essential that the I2S Master clock frequency
	 * is exactly what it is expected to be.
	 */
	if (aclk_freq % i2s_master_freq == 0) {
		/* Set the I2S Master clock frequency */
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PWR_MGMT_A_CLK_CTRL_ADDR,
				IAXXX_PWR_MGMT_A_CLK_CTRL_I2S_MASTER_CLK_MASK,
				i2s_master_clk - 1);
		if (ret < 0)
			return ret;

		/* Set SRB to configure the A_CLK tree */
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_SET_I2SM_CLK_MASK,
				IAXXX_SRB_SYS_POWER_CTRL_SET_I2SM_CLK_MASK);
		if (ret < 0)
			return ret;

		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
						&status, IAXXX_BLOCK_0);
		if (ret) {
			dev_err(codec->dev,
				"%s: Sys pwr ctrl addr Update blk failed : %d\n",
				__func__, ret);
			return ret;
		}
	} else {
		dev_err(codec->dev, "%s: Cannot divide %d to %d\n",
			__func__, aclk_freq, i2s_master_freq);
		return -EINVAL;
	}

	iaxxx->i2s_master_clk = ucontrol->value.enumerated.item[0];

	return 0;
}

static int iaxxx_get_pdm_aud_port_clk(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->pdm_aud_port_clk;
	return 0;
}

static int iaxxx_put_pdm_aud_port_clk(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	iaxxx->pdm_aud_port_clk = ucontrol->value.enumerated.item[0];
	return 0;
}

#define IAXXX_UPDATE_BLOCK_SET_GET(blk_name, block) \
static int iaxxx_put_update_##blk_name(struct snd_kcontrol *kcontrol, \
				  struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	u32 status = 0; \
	int ret = 0; \
	dev_dbg(codec->dev, "%s: enter\n", __func__); \
	if (ucontrol->value.enumerated.item[0]) { \
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
						&status, block); \
		if (ret) \
			pr_err("Update blk failed : %d", ret); \
	} \
	return ret; \
} \
static int iaxxx_get_update_##blk_name(struct snd_kcontrol *kcontrol, \
				 struct snd_ctl_elem_value *ucontrol) \
{ \
	return 0; \
}
IAXXX_UPDATE_BLOCK_SET_GET(block0, IAXXX_BLOCK_0)
IAXXX_UPDATE_BLOCK_SET_GET(block1, IAXXX_BLOCK_1)
IAXXX_UPDATE_BLOCK_SET_GET(plgblock0, IAXXX_BLOCK_0)
IAXXX_UPDATE_BLOCK_SET_GET(plgblock1, IAXXX_BLOCK_1)

static int iaxxx_put_route_status(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct device *dev = iaxxx->dev_parent;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret = 0;

	pr_info("dev id-%d: enter, val requested %d",
			priv->dev_id, ucontrol->value.enumerated.item[0]);

	if (ucontrol->value.enumerated.item[0]) {
		dev_info(dev, "Route active request received\n");
		ret = iaxxx_fw_notifier_call(priv->dev,
				IAXXX_EV_ROUTE_ACTIVE, NULL);
		if (ret)
			dev_err(dev, "%s: Propagate route status failed : %d\n",
								__func__, ret);
	}

	iaxxx_core_set_route_status(priv, ucontrol->value.enumerated.item[0]);
	return ret;
}

static int iaxxx_get_route_status(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	ucontrol->value.enumerated.item[0] = iaxxx_core_get_route_status(priv);
	return 0;

}

static int iaxxx_put_power_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct device *dev = iaxxx->dev_parent;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret = 0;
	int val = ucontrol->value.enumerated.item[0];
	u32 power_mode = atomic_read(&priv->iaxxx_state->power_state);
	u32 dis_ctrl_intf = atomic_read(&priv->iaxxx_state->disable_ctrl_intf);

	pr_info("dev id-%d: val requested %d", priv->dev_id, val);

	/* Check if Chip is already in the same state as requested */
	if ((((power_mode == IAXXX_NORMAL_MODE) ||
	      ((power_mode == IAXXX_OPTIMAL_MODE) && !dis_ctrl_intf)) && val) ||
	    (((power_mode == IAXXX_SLEEP_MODE) ||
	      ((power_mode == IAXXX_OPTIMAL_MODE) && dis_ctrl_intf)) && !val)) {
		pr_info("dev id-%d: Already in %s mode",
				priv->dev_id, val ? "Normal" : "Low Power");
		return 0;
	}

	if (!val)
		ret = iaxxx_suspend_chip(priv);
	else
		ret  = iaxxx_wakeup_chip(priv);

	if (ret)
		dev_err(dev, "not able change the power mode\n");

	return ret;
}

static int iaxxx_get_power_mode(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	if (atomic_read(&priv->iaxxx_state->power_state) == IAXXX_NORMAL_MODE)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;
	return 0;
}

static const DECLARE_TLV_DB_SCALE(gn_ch_ep_tlv, -1200, 100, 0);

/* Dummy CH Port for DAPM */
#define IAXXX_CH_MGR_DAPM_CTLS(channel, channel_name) \
static const struct soc_enum channel##_port_enum = \
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,\
			ARRAY_SIZE(io_port_texts), io_port_texts); \
static const struct snd_kcontrol_new channel##_port_mux =      \
	SOC_DAPM_ENUM(channel_name "Port", channel##_port_enum)

#define IAXXX_CH_MGR_DAPM_MUX(channel, channel_name) \
	SND_SOC_DAPM_MUX(channel_name " Port", SND_SOC_NOPM, 0, 0, \
			 &(channel##_port_mux))

#define IAXXX_CH_RX_TO_TX_DAPM_CTLS(channel, channel_name) \
static const SOC_ENUM_SINGLE_DECL(channel##_rx_en_enum, \
			SND_SOC_NOPM, 0, channel##_rx_off_on_texts); \
static const struct snd_kcontrol_new channel##_rx_mux =        \
	SOC_DAPM_ENUM(channel_name "PortTxEn", channel##_rx_en_enum)

#define IAXXX_CH_RX_TO_TX_DAPM_MUX(channel, channel_name) \
	SND_SOC_DAPM_MUX(channel_name " PortMux En", SND_SOC_NOPM, 0, 0, \
						&(channel##_rx_mux))

#define IAXXXCORE_RX_CHMGR(channel) \
static int iaxxx_set_##channel##_gain_ramp(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(gain_ramp_value)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = gain_ramp_value[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, \
		val << \
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_gain_ramp(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr; \
	u32 val; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK) >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(gain_ramp_value); i++) \
		if (val == gain_ramp_value[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(gain_ramp_value)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_ep_gain(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, \
		val << \
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_ep_gain(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 addr; \
	u32 val; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK) >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_gain_evt(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr; \
	uint32_t val = ucontrol->value.integer.value[0]; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK, \
		val << \
		IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_gain_evt(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 addr; \
	u32 val; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK) >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_gain_en(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	int ret; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CH_HDR_CH_GAIN_ADDR, \
		1 << channel, val << channel); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_gain_en(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	u32 val; \
	\
	val = snd_soc_component_read32(codec, IAXXX_CH_HDR_CH_GAIN_ADDR); \
	ucontrol->value.integer.value[0] = (val & (1 << channel)) >> channel; \
	return 0; \
}

IAXXXCORE_RX_CHMGR(RX_0);
IAXXXCORE_RX_CHMGR(RX_1);
IAXXXCORE_RX_CHMGR(RX_2);
IAXXXCORE_RX_CHMGR(RX_3);
IAXXXCORE_RX_CHMGR(RX_4);
IAXXXCORE_RX_CHMGR(RX_5);
IAXXXCORE_RX_CHMGR(RX_6);
IAXXXCORE_RX_CHMGR(RX_7);
IAXXXCORE_RX_CHMGR(RX_8);
IAXXXCORE_RX_CHMGR(RX_9);
IAXXXCORE_RX_CHMGR(RX_10);
IAXXXCORE_RX_CHMGR(RX_11);
IAXXXCORE_RX_CHMGR(RX_12);
IAXXXCORE_RX_CHMGR(RX_13);
IAXXXCORE_RX_CHMGR(RX_14);
IAXXXCORE_RX_CHMGR(RX_15);

#define IAXXXCORE_RX_CHMGR_KCTRL(channel, channel_name) \
	SOC_ENUM_EXT(channel_name "Chan GnRmp", iaxxx_ch_gain_ramp_enum, \
		iaxxx_get_##channel##_gain_ramp, \
		iaxxx_set_##channel##_gain_ramp), \
	SOC_SINGLE_EXT(channel_name "Ch EpGain", SND_SOC_NOPM,\
			0, 0xFF, 0, iaxxx_get_##channel##_ep_gain, \
			iaxxx_set_##channel##_ep_gain), \
	SOC_SINGLE_BOOL_EXT(channel_name "Chan GnReEvt", 0, \
		iaxxx_get_##channel##_gain_evt, \
		iaxxx_set_##channel##_gain_evt), \
	SOC_SINGLE_BOOL_EXT(channel_name "Chan Gain En", \
		0, iaxxx_get_##channel##_gain_en, \
		iaxxx_set_##channel##_gain_en)

IAXXX_CH_MGR_DAPM_CTLS(RX_0, "Rx0 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_1, "Rx1 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_2, "Rx2 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_3, "Rx3 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_4, "Rx4 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_5, "Rx5 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_6, "Rx6 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_7, "Rx7 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_8, "Rx8 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_9, "Rx9 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_10, "Rx10 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_11, "Rx11 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_12, "Rx12 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_13, "Rx13 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_14, "Rx14 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_15, "Rx15 Mux");

/* Stream configuration */
static const unsigned int str_mstr_id_values[] = {0x0, 0x1,
	0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0xFFFF, };

static const char * const str_mstr_id_texts[] = {
	ENUM_NAME(STREAMID_0), ENUM_NAME(STREAMID_1), ENUM_NAME(STREAMID_2),
	ENUM_NAME(STREAMID_3), ENUM_NAME(STREAMID_4), ENUM_NAME(STREAMID_5),
	ENUM_NAME(STREAMID_6), ENUM_NAME(STREAMID_7), ENUM_NAME(STREAMID_NONE),
};

static const struct soc_enum iaxxx_strm_mstr_id_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(str_mstr_id_texts),
						str_mstr_id_texts);

static const unsigned int strm_pwr_mode_value[] = {
	0x0, 0x1, 0x2, };

static const char * const strm_pwr_mode_texts[] = {
	ENUM_NAME(STANDARD), ENUM_NAME(LOW_POWER), ENUM_NAME(LOW_POWER_VQ), };

static const char * const strm_src_mode_texts[] = {
	ENUM_NAME(DOWN_SAMPLE_BY_2), ENUM_NAME(DOWN_SAMPLE_BY_3),
	ENUM_NAME(DOWN_SAMPLE_BY_4), ENUM_NAME(DOWN_SAMPLE_BY_6),
	ENUM_NAME(UP_SAMPLE_BY_2), ENUM_NAME(UP_SAMPLE_BY_3),
	ENUM_NAME(UP_SAMPLE_BY_4), ENUM_NAME(UP_SAMPLE_BY_6)};

static const struct soc_enum iaxxx_strm_src_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(strm_src_mode_texts),
			strm_src_mode_texts);

static const char * const strm_asrc_mode_texts[] = {
	ENUM_NAME(ASRC_ENABLE), ENUM_NAME(ASRC_DISABLE), ENUM_NAME(REDBOX_2-1),
	ENUM_NAME(REDBOX_1-2), ENUM_NAME(SRC_ENABLED)};

static const struct soc_enum iaxxx_strm_asrc_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(strm_asrc_mode_texts),
			strm_asrc_mode_texts);

static const unsigned int strm_xfer_mode_value[] = {
	0x0, 0x1, 0x2, 0x3, };

static const char * const strm_xfer_mode_texts[] = {
	ENUM_NAME(SSP), ENUM_NAME(DMA), ENUM_NAME(CPU),
	ENUM_NAME(Unused), };

static const struct soc_enum iaxxx_strm_xfer_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(strm_xfer_mode_texts),
			strm_xfer_mode_texts);

/* stream direction */
#define IAXXXCORE_STREAM(stream) \
static int iaxxx_set_strm##stream##_asrc_mode(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr; \
	uint32_t val = ucontrol->value.integer.value[0]; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_MASK, \
		val << IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_asrc_mode(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr, val; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_MASK) >> \
			IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_src_mode(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_CTRL_SRC_MODE_MASK, \
		val << IAXXX_STR_GRP_STR_CTRL_SRC_MODE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_src_mode(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr, val; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_CTRL_SRC_MODE_MASK) >> \
			IAXXX_STR_GRP_STR_CTRL_SRC_MODE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_xfer_mode(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_CTRL_XFER_MODE_MASK, \
		val << IAXXX_STR_GRP_STR_CTRL_XFER_MODE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_xfer_mode(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr, val; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_CTRL_XFER_MODE_MASK) >> \
			IAXXX_STR_GRP_STR_CTRL_XFER_MODE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_port_enc(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(str_enc_values)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_PORT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = str_enc_values[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_MASK, \
		val << IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_port_enc(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_PORT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_MASK) >> \
			IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(str_enc_values); i++) \
		if (val == str_enc_values[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(str_enc_values)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_port(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(io_port_value)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_PORT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = io_port_value[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_PORT_PORT_MASK, \
		val << IAXXX_STR_GRP_STR_PORT_PORT_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_port(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_PORT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_STR_GRP_STR_PORT_PORT_MASK) >> \
			IAXXX_STR_GRP_STR_PORT_PORT_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(io_port_value); i++) \
		if (val == io_port_value[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(io_port_value)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_master_id(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_SYNC_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_SYNC_MASTER_STR_MASK, \
		val << IAXXX_STR_GRP_STR_SYNC_MASTER_STR_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_master_id(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_SYNC_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_SYNC_MASTER_STR_MASK) >> \
			IAXXX_STR_GRP_STR_SYNC_MASTER_STR_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_format_enc(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(str_enc_values)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_FORMAT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = str_enc_values[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_FORMAT_ENCODING_MASK, \
		val << IAXXX_STR_GRP_STR_FORMAT_ENCODING_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_format_enc(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_FORMAT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_STR_GRP_STR_FORMAT_ENCODING_MASK) >> \
			IAXXX_STR_GRP_STR_FORMAT_ENCODING_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(str_enc_values); i++) \
		if (val == str_enc_values[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(str_enc_values)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_sample_rate(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_FORMAT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_MASK, \
		val << IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_sample_rate(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_FORMAT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_MASK) >> \
			IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_frame_length( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= \
			ARRAY_SIZE(str_frm_len_values)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_FORMAT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = str_frm_len_values[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_FORMAT_LENGTH_MASK, \
		val << IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_frame_length( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_FORMAT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_STR_GRP_STR_FORMAT_LENGTH_MASK) >> \
			IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(str_frm_len_values); i++) \
		if (val == str_frm_len_values[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(str_frm_len_values)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_dc_block_en( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	bool val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_CTRL_DC_BLOCK_ENABLE_MASK, \
		val << IAXXX_STR_GRP_STR_CTRL_DC_BLOCK_ENABLE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_dc_block_en(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_CTRL_DC_BLOCK_ENABLE_MASK) >> \
			IAXXX_STR_GRP_STR_CTRL_DC_BLOCK_ENABLE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_droop_comp_en( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	bool val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_MASK, \
		val << IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_droop_comp_en( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_MASK) >> \
			IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_tone_gen_en( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	bool val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_CTRL_TONE_GEN_ENABLE_MASK, \
		val << IAXXX_STR_GRP_STR_CTRL_TONE_GEN_ENABLE_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_tone_gen_en(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CTRL_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_CTRL_TONE_GEN_ENABLE_MASK) >> \
			IAXXX_STR_GRP_STR_CTRL_TONE_GEN_ENABLE_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_dir( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_PORT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_PORT_DIR_MASK, \
		val << IAXXX_STR_GRP_STR_PORT_DIR_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_dir(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_PORT_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_PORT_DIR_MASK) >> \
			IAXXX_STR_GRP_STR_PORT_DIR_POS; \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_ch_mask( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CHN_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	ret = snd_soc_component_write(codec, addr, val); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_ch_mask(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_CHN_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = snd_soc_component_read32(codec, addr); \
	return 0; \
} \
\
static int iaxxx_set_strm##stream##_inter_stream_delay( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_SYNC_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_STR_GRP_STR_SYNC_INTER_STR_DELAY_MASK, \
		val << IAXXX_STR_GRP_STR_SYNC_INTER_STR_DELAY_POS); \
	return ret; \
} \
\
static int iaxxx_get_strm##stream##_inter_stream_delay( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_STR_GRP_STR_SYNC_ADDR, \
				  stream); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_STR_GRP_STR_SYNC_INTER_STR_DELAY_MASK) >> \
			IAXXX_STR_GRP_STR_SYNC_INTER_STR_DELAY_POS; \
	return 0; \
} \
\
static int iaxxxcore_set_strm##stream##_en( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	int ret = 0; \
	u32 status; \
	dev_dbg(codec->dev, "%s: enter\n", __func__); \
	if (ucontrol->value.integer.value[0] == iaxxx->stream_en[stream]) \
		return 0; \
	if (ucontrol->value.integer.value[0]) { \
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_STR_HDR_STR_EN_ADDR, \
					1 << stream, 1 << stream); \
		if (ret < 0) \
			return ret; \
		iaxxx->stream_en[stream] = 1; \
	} else { \
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_STR_HDR_STR_EN_ADDR, \
					1 << stream, 0 << stream); \
		if (ret < 0) \
			return ret; \
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
						&status, IAXXX_BLOCK_0); \
		if (ret) { \
			pr_err("STR EN Addr Update blk failed : %d", ret); \
			return ret; \
		} \
		iaxxx->stream_en[stream] = 0; \
	} \
	return ret; \
} \
\
static int iaxxxcore_get_strm##stream##_en( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->stream_en[stream]; \
	return 0; \
}

IAXXXCORE_STREAM(STREAM0);
IAXXXCORE_STREAM(STREAM1);
IAXXXCORE_STREAM(STREAM2);
IAXXXCORE_STREAM(STREAM3);
IAXXXCORE_STREAM(STREAM4);
IAXXXCORE_STREAM(STREAM5);
IAXXXCORE_STREAM(STREAM6);
IAXXXCORE_STREAM(STREAM7);

#define IAXXXCORE_STREAM_KCTRL(stream, strm_name) \
	SOC_SINGLE_BOOL_EXT(strm_name" En", 0, \
		       iaxxxcore_get_strm##stream##_en, \
		       iaxxxcore_set_strm##stream##_en), \
	SOC_ENUM_EXT(strm_name" Format Enc", iaxxx_str_enc_enum, \
		     iaxxx_get_strm##stream##_format_enc, \
		     iaxxx_set_strm##stream##_format_enc), \
	SOC_ENUM_EXT(strm_name" Format Sr", iaxxx_strm_sr_enum, \
		     iaxxx_get_strm##stream##_sample_rate, \
		     iaxxx_set_strm##stream##_sample_rate), \
	SOC_ENUM_EXT(strm_name" Format FrLn", iaxxx_frm_len_enum, \
		     iaxxx_get_strm##stream##_frame_length, \
		     iaxxx_set_strm##stream##_frame_length), \
	SOC_ENUM_EXT(strm_name" ASRC Mode", iaxxx_strm_asrc_mode_enum, \
		     iaxxx_get_strm##stream##_asrc_mode, \
		     iaxxx_set_strm##stream##_asrc_mode), \
	SOC_ENUM_EXT(strm_name" SRC Mode", iaxxx_strm_src_mode_enum, \
		     iaxxx_get_strm##stream##_src_mode, \
		     iaxxx_set_strm##stream##_src_mode), \
	SOC_ENUM_EXT(strm_name" Xfer Mode", iaxxx_strm_xfer_mode_enum, \
		     iaxxx_get_strm##stream##_xfer_mode, \
		     iaxxx_set_strm##stream##_xfer_mode), \
	SOC_SINGLE_BOOL_EXT(strm_name" droop comp en", 0, \
		       iaxxx_get_strm##stream##_droop_comp_en, \
		       iaxxx_set_strm##stream##_droop_comp_en), \
	SOC_SINGLE_BOOL_EXT(strm_name" DC block en", 0, \
		       iaxxx_get_strm##stream##_dc_block_en, \
		       iaxxx_set_strm##stream##_dc_block_en), \
	SOC_SINGLE_BOOL_EXT(strm_name" tone gen en", 0, \
		       iaxxx_get_strm##stream##_tone_gen_en, \
		       iaxxx_set_strm##stream##_tone_gen_en), \
	SOC_ENUM_EXT(strm_name" Port", iaxxx_strm_port_enum, \
		     iaxxx_get_strm##stream##_port, \
		     iaxxx_set_strm##stream##_port), \
	SOC_ENUM_EXT(strm_name" Port Enc", iaxxx_str_enc_enum, \
		     iaxxx_get_strm##stream##_port_enc, \
		     iaxxx_set_strm##stream##_port_enc), \
	SOC_SINGLE_BOOL_EXT(strm_name" Dir", 0, \
		       iaxxx_get_strm##stream##_dir, \
		       iaxxx_set_strm##stream##_dir), \
	SOC_SINGLE_EXT(strm_name" CH Mask En", SND_SOC_NOPM,\
			0, 0xFFFF, 0, \
			iaxxx_get_strm##stream##_ch_mask, \
			iaxxx_set_strm##stream##_ch_mask), \
	SOC_ENUM_EXT(strm_name" Master Strm Id", iaxxx_strm_mstr_id_enum, \
		     iaxxx_get_strm##stream##_master_id, \
		     iaxxx_set_strm##stream##_master_id), \
	SOC_SINGLE_EXT(strm_name" inter strm delay", SND_SOC_NOPM,\
			0, 0xFFFF, 0, \
			iaxxx_get_strm##stream##_inter_stream_delay, \
			iaxxx_set_strm##stream##_inter_stream_delay)

static const unsigned int ip_ep_values[] = {
	IAXXX_SYSID_INVALID,
	/* Output Channels 0-7 to EndPoint-0 */
	IAXXX_SYSID_CHANNEL_RX_0_EP_0,
	IAXXX_SYSID_CHANNEL_RX_1_EP_0,
	IAXXX_SYSID_CHANNEL_RX_2_EP_0,
	IAXXX_SYSID_CHANNEL_RX_3_EP_0,
	IAXXX_SYSID_CHANNEL_RX_4_EP_0,
	IAXXX_SYSID_CHANNEL_RX_5_EP_0,
	IAXXX_SYSID_CHANNEL_RX_6_EP_0,
	IAXXX_SYSID_CHANNEL_RX_7_EP_0,
	IAXXX_SYSID_CHANNEL_RX_8_EP_0,
	IAXXX_SYSID_CHANNEL_RX_9_EP_0,
	IAXXX_SYSID_CHANNEL_RX_10_EP_0,
	IAXXX_SYSID_CHANNEL_RX_11_EP_0,
	IAXXX_SYSID_CHANNEL_RX_12_EP_0,
	IAXXX_SYSID_CHANNEL_RX_13_EP_0,
	IAXXX_SYSID_CHANNEL_RX_14_EP_0,
	IAXXX_SYSID_CHANNEL_RX_15_EP_0,

	/* Plugin 0 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_0_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_7,

	/* Plugin 1 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_1_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_7,

	/* Plugin 2 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_2_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_7,

	/* Plugin 3 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_3_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_7,

	/* Plugin 4 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_4_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_7,

	/* Plugin 5 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_5_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_7,

	/* Plugin 6 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_6_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_7,

	/* Plugin 7 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_7_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_7,

	/* Plugin 8 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_8_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_7,

	/* Plugin 9 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_9_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_7,

	/* Plugin 10 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_10_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_7,

	/* Plugin 11 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_11_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_7,

	/* Plugin 12 EndPoint 0 to 7 */
	IAXXX_SYSID_PLUGIN_12_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_7,
};

static const char * const ip_ep_texts[] = {
	ENUM_NAME(UNKNOWN),
	ENUM_NAME(RX0_ChanMgr), ENUM_NAME(RX1_ChanMgr),
	ENUM_NAME(RX2_ChanMgr), ENUM_NAME(RX3_ChanMgr),
	ENUM_NAME(RX4_ChanMgr), ENUM_NAME(RX5_ChanMgr),
	ENUM_NAME(RX6_ChanMgr), ENUM_NAME(RX7_ChanMgr),
	ENUM_NAME(RX8_ChanMgr), ENUM_NAME(RX9_ChanMgr),
	ENUM_NAME(RX10_ChanMgr), ENUM_NAME(RX11_ChanMgr),
	ENUM_NAME(RX12_ChanMgr), ENUM_NAME(RX13_ChanMgr),
	ENUM_NAME(RX14_ChanMgr), ENUM_NAME(RX15_ChanMgr),
	ENUM_NAME(plugin0Out0), ENUM_NAME(plugin0Out1),
	ENUM_NAME(plugin0Out2), ENUM_NAME(plugin0Out3),
	ENUM_NAME(plugin0Out4), ENUM_NAME(plugin0Out5),
	ENUM_NAME(plugin0Out6), ENUM_NAME(plugin0Out7),
	ENUM_NAME(plugin1Out0), ENUM_NAME(plugin1Out1),
	ENUM_NAME(plugin1Out2), ENUM_NAME(plugin1Out3),
	ENUM_NAME(plugin1Out4), ENUM_NAME(plugin1Out5),
	ENUM_NAME(plugin1Out6), ENUM_NAME(plugin1Out7),
	ENUM_NAME(plugin2Out0), ENUM_NAME(plugin2Out1),
	ENUM_NAME(plugin2Out2), ENUM_NAME(plugin2Out3),
	ENUM_NAME(plugin2Out4), ENUM_NAME(plugin2Out5),
	ENUM_NAME(plugin2Out6), ENUM_NAME(plugin2Out7),
	ENUM_NAME(plugin3Out0), ENUM_NAME(plugin3Out1),
	ENUM_NAME(plugin3Out2), ENUM_NAME(plugin3Out3),
	ENUM_NAME(plugin3Out4), ENUM_NAME(plugin3Out5),
	ENUM_NAME(plugin3Out6), ENUM_NAME(plugin3Out7),
	ENUM_NAME(plugin4Out0), ENUM_NAME(plugin4Out1),
	ENUM_NAME(plugin4Out2), ENUM_NAME(plugin4Out3),
	ENUM_NAME(plugin4Out4), ENUM_NAME(plugin4Out5),
	ENUM_NAME(plugin4Out6), ENUM_NAME(plugin4Out7),
	ENUM_NAME(plugin5Out0), ENUM_NAME(plugin5Out1),
	ENUM_NAME(plugin5Out2), ENUM_NAME(plugin5Out3),
	ENUM_NAME(plugin5Out4), ENUM_NAME(plugin5Out5),
	ENUM_NAME(plugin5Out6), ENUM_NAME(plugin5Out7),
	ENUM_NAME(plugin6Out0), ENUM_NAME(plugin6Out1),
	ENUM_NAME(plugin6Out2), ENUM_NAME(plugin6Out3),
	ENUM_NAME(plugin6Out4), ENUM_NAME(plugin6Out5),
	ENUM_NAME(plugin6Out6), ENUM_NAME(plugin6Out7),
	ENUM_NAME(plugin7Out0), ENUM_NAME(plugin7Out1),
	ENUM_NAME(plugin7Out2), ENUM_NAME(plugin7Out3),
	ENUM_NAME(plugin7Out4), ENUM_NAME(plugin7Out5),
	ENUM_NAME(plugin7Out6), ENUM_NAME(plugin7Out7),
	ENUM_NAME(plugin8Out0), ENUM_NAME(plugin8Out1),
	ENUM_NAME(plugin8Out2), ENUM_NAME(plugin8Out3),
	ENUM_NAME(plugin8Out4), ENUM_NAME(plugin8Out5),
	ENUM_NAME(plugin8Out6), ENUM_NAME(plugin8Out7),
	ENUM_NAME(plugin9Out0), ENUM_NAME(plugin9Out1),
	ENUM_NAME(plugin9Out2), ENUM_NAME(plugin9Out3),
	ENUM_NAME(plugin9Out4), ENUM_NAME(plugin9Out5),
	ENUM_NAME(plugin9Out6), ENUM_NAME(plugin9Out7),
	ENUM_NAME(plugin10Out0), ENUM_NAME(plugin10Out1),
	ENUM_NAME(plugin10Out2), ENUM_NAME(plugin10Out3),
	ENUM_NAME(plugin10Out4), ENUM_NAME(plugin10Out5),
	ENUM_NAME(plugin10Out6), ENUM_NAME(plugin10Out7),
	ENUM_NAME(plugin11Out0), ENUM_NAME(plugin11Out1),
	ENUM_NAME(plugin11Out2), ENUM_NAME(plugin11Out3),
	ENUM_NAME(plugin11Out4), ENUM_NAME(plugin11Out5),
	ENUM_NAME(plugin11Out6), ENUM_NAME(plugin11Out7),
	ENUM_NAME(plugin12Out0), ENUM_NAME(plugin12Out1),
	ENUM_NAME(plugin12Out2), ENUM_NAME(plugin12Out3),
	ENUM_NAME(plugin12Out4), ENUM_NAME(plugin12Out5),
	ENUM_NAME(plugin12Out6), ENUM_NAME(plugin12Out7),
	ENUM_NAME(plugin13Out0), ENUM_NAME(plugin13Out1),
	ENUM_NAME(plugin13Out2), ENUM_NAME(plugin13Out3),
	ENUM_NAME(plugin13Out4), ENUM_NAME(plugin13Out5),
	ENUM_NAME(plugin13Out6), ENUM_NAME(plugin13Out7),
	ENUM_NAME(plugin14Out0), ENUM_NAME(plugin14Out1),
	ENUM_NAME(plugin14Out2), ENUM_NAME(plugin14Out3),
	ENUM_NAME(plugin14Out4), ENUM_NAME(plugin14Out5),
	ENUM_NAME(plugin14Out6), ENUM_NAME(plugin14Out7),
};

static const struct soc_enum iaxxx_ip_ep_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(ip_ep_texts),
			ip_ep_texts);


#define IAXXXCORE_TX_CHMGR(channel) \
static int iaxxx_set_##channel##_gain_ramp(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(gain_ramp_value)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = gain_ramp_value[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, \
		val << \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_gain_ramp(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK) >> \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(gain_ramp_value); i++) \
		if (val == gain_ramp_value[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(gain_ramp_value)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_ep_gain(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, \
		val << \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_ep_gain(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK) >> \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_gain_evt(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	uint32_t addr; \
	int ret; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK, \
		val << \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_gain_evt(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	ucontrol->value.integer.value[0] = (val & \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK) >> \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_gain_en(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	uint32_t val = ucontrol->value.integer.value[0]; \
	int ret; \
	\
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CH_HDR_CH_GAIN_ADDR, \
		1 << channel, val << channel); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_gain_en(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	uint32_t val; \
	\
	val = snd_soc_component_read32(codec, IAXXX_CH_HDR_CH_GAIN_ADDR); \
	ucontrol->value.integer.value[0] = (val & (1 << channel)) >> channel; \
	return 0; \
} \
\
static int iaxxx_set_##channel##_in_connect(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(ip_ep_values)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_IN_CONNECT_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = ip_ep_values[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_MASK, \
		val << IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_POS); \
	return ret; \
} \
\
static int iaxxx_get_##channel##_in_connect(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_CH_GRP_IN_CONNECT_ADDR, \
				  channel); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_MASK) >> \
			IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(ip_ep_values); i++) \
		if (val == ip_ep_values[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(ip_ep_values)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
}

IAXXXCORE_TX_CHMGR(TX_0);
IAXXXCORE_TX_CHMGR(TX_1);
IAXXXCORE_TX_CHMGR(TX_2);
IAXXXCORE_TX_CHMGR(TX_3);
IAXXXCORE_TX_CHMGR(TX_4);
IAXXXCORE_TX_CHMGR(TX_5);
IAXXXCORE_TX_CHMGR(TX_6);
IAXXXCORE_TX_CHMGR(TX_7);
IAXXXCORE_TX_CHMGR(TX_8);
IAXXXCORE_TX_CHMGR(TX_9);
IAXXXCORE_TX_CHMGR(TX_10);
IAXXXCORE_TX_CHMGR(TX_11);
IAXXXCORE_TX_CHMGR(TX_12);
IAXXXCORE_TX_CHMGR(TX_13);
IAXXXCORE_TX_CHMGR(TX_14);
IAXXXCORE_TX_CHMGR(TX_15);

#define IAXXXCORE_TX_CHMGR_KCTRL(channel, channel_name) \
	SOC_ENUM_EXT(channel_name "Chan GnRmp", iaxxx_ch_gain_ramp_enum, \
		iaxxx_get_##channel##_gain_ramp, \
		iaxxx_set_##channel##_gain_ramp), \
	SOC_SINGLE_EXT(channel_name "Ch EpGain", SND_SOC_NOPM,\
			0, 0xFF, 0, iaxxx_get_##channel##_ep_gain, \
			iaxxx_set_##channel##_ep_gain), \
	SOC_SINGLE_BOOL_EXT(channel_name "Chan GnReEvt", 0, \
		iaxxx_get_##channel##_gain_evt, \
		iaxxx_set_##channel##_gain_evt), \
	SOC_SINGLE_BOOL_EXT(channel_name "Chan Gain En", \
		0, iaxxx_get_##channel##_gain_en, \
		iaxxx_set_##channel##_gain_en), \
	SOC_ENUM_EXT(channel_name "Chan IpSrcId", iaxxx_ip_ep_enum, \
		iaxxx_get_##channel##_in_connect, \
		iaxxx_set_##channel##_in_connect)

IAXXX_CH_MGR_DAPM_CTLS(TX_0, "Tx0 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_1, "Tx1 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_2, "Tx2 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_3, "Tx3 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_4, "Tx4 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_5, "Tx5 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_6, "Tx6 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_7, "Tx7 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_8, "Tx8 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_9, "Tx9 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_10, "Tx10 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_11, "Tx11 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_12, "Tx12 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_13, "Tx13 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_14, "Tx14 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_15, "Tx15 Mux");

IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_0, "Tx0");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_1, "Tx1");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_2, "Tx2");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_3, "Tx3");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_4, "Tx4");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_5, "Tx5");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_6, "Tx6");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_7, "Tx7");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_8, "Tx8");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_9, "Tx9");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_10, "Tx10");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_11, "Tx11");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_12, "Tx12");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_13, "Tx13");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_14, "Tx14");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_15, "Tx15");

#define IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, ep_num) \
static int iaxxx_set_##plugin##_ep##ep_num##_connect( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int ret; \
	\
	if (ucontrol->value.integer.value[0] >= ARRAY_SIZE(ip_ep_values)) \
		return -EINVAL; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, \
			IAXXX_PLUGIN_INS_GRP_IN_##ep_num##_CONNECT_ADDR, \
			plugin); \
	if (!addr) \
		return -EINVAL; \
	\
	val = ip_ep_values[ucontrol->value.integer.value[0]]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, addr, \
		IAXXX_PLUGIN_INS_GRP_IN_##ep_num##_CONNECT_SOURCEID_MASK, \
		val << \
		IAXXX_PLUGIN_INS_GRP_IN_##ep_num##_CONNECT_SOURCEID_POS); \
	return ret; \
} \
\
static int iaxxx_get_##plugin##_ep##ep_num##_connect( \
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	uint32_t val, addr; \
	int i; \
	\
	addr = IAXXX_GET_GRP_ADDR(priv, \
			IAXXX_PLUGIN_INS_GRP_IN_##ep_num##_CONNECT_ADDR, \
			plugin); \
	if (!addr) \
		return -EINVAL; \
	\
	val = snd_soc_component_read32(codec, addr); \
	val = (val & \
		IAXXX_PLUGIN_INS_GRP_IN_##ep_num##_CONNECT_SOURCEID_MASK) >> \
		IAXXX_PLUGIN_INS_GRP_IN_##ep_num##_CONNECT_SOURCEID_POS; \
	\
	for (i = 0; i < ARRAY_SIZE(ip_ep_values); i++) \
		if (val == ip_ep_values[i]) \
			break; \
	\
	if (i == ARRAY_SIZE(ip_ep_values)) \
		return -EINVAL; \
	\
	ucontrol->value.integer.value[0] = i; \
	return 0; \
}

#define IAXXXCORE_PLUGIN(plugin) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 0) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 1) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 2) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 3) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 4) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 5) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 6) \
	IAXXX_PLUGIN_EP_CONNECT_GET_SET(plugin, 7) \

IAXXXCORE_PLUGIN(PLUGIN0)
IAXXXCORE_PLUGIN(PLUGIN1)
IAXXXCORE_PLUGIN(PLUGIN2)
IAXXXCORE_PLUGIN(PLUGIN3)
IAXXXCORE_PLUGIN(PLUGIN4)
IAXXXCORE_PLUGIN(PLUGIN5)
IAXXXCORE_PLUGIN(PLUGIN6)
IAXXXCORE_PLUGIN(PLUGIN7)
IAXXXCORE_PLUGIN(PLUGIN8)
IAXXXCORE_PLUGIN(PLUGIN9)
IAXXXCORE_PLUGIN(PLUGIN10)
IAXXXCORE_PLUGIN(PLUGIN11)
IAXXXCORE_PLUGIN(PLUGIN12)
IAXXXCORE_PLUGIN(PLUGIN13)
IAXXXCORE_PLUGIN(PLUGIN14)

#define IAXXXCORE_PLUGIN_KCTRL(plugin, plugin_name) \
	SOC_ENUM_EXT(plugin_name "Ip Ep0 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep0_connect, \
		iaxxx_set_##plugin##_ep0_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep1 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep1_connect, \
		iaxxx_set_##plugin##_ep1_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep2 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep2_connect, \
		iaxxx_set_##plugin##_ep2_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep3 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep3_connect, \
		iaxxx_set_##plugin##_ep3_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep4 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep4_connect, \
		iaxxx_set_##plugin##_ep4_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep5 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep5_connect, \
		iaxxx_set_##plugin##_ep5_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep6 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep6_connect, \
		iaxxx_set_##plugin##_ep6_connect), \
	SOC_ENUM_EXT(plugin_name "Ip Ep7 Conf", iaxxx_ip_ep_enum, \
		iaxxx_get_##plugin##_ep7_connect, \
		iaxxx_set_##plugin##_ep7_connect)

#define IAXXXCORE_PLUGIN_ON_OFF_TEXTS(plugin, plugin_name) \
static const char * const plugin##_off_on_texts[] = { \
	"Off", \
	"Rx0"plugin_name"On", "Rx1"plugin_name"On", "Rx2"plugin_name"On", \
	"Rx3"plugin_name"On", "Rx4"plugin_name"On", "Rx5"plugin_name"On", \
	"Rx6"plugin_name"On", "Rx7"plugin_name"On", \
	"Rx8"plugin_name"On", "Rx9"plugin_name"On", \
	"Rx10"plugin_name"On", "Rx11"plugin_name"On", \
	"Rx12"plugin_name"On", "Rx13"plugin_name"On", \
	"Rx14"plugin_name"On", "Rx15"plugin_name"On", \
}

IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN0, "Plgin0");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN1, "Plgin1");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN2, "Plgin2");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN3, "Plgin3");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN4, "Plgin4");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN5, "Plgin5");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN6, "Plgin6");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN7, "Plgin7");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN8, "Plgin8");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN9, "Plgin9");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN10, "Plgin10");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN11, "Plgin11");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN12, "Plgin12");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN13, "Plgin13");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN14, "Plgin14");

#define IAXXX_PLUGIN_DAPM_MUX(plugin, plugin_name) \
	SND_SOC_DAPM_MUX(plugin_name "En", SND_SOC_NOPM, 0, 0, &plugin##_mux)

#define IAXXX_PLUGIN_BLK_SET_GET(plugin) \
static int iaxxxcore_set_plgin##plugin##_Blk0En( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 status = 0; \
	int ret = 0; \
	\
	dev_info(codec->dev, "%s: enter, plugin %d, enb %d\n", \
		__func__, plugin, ucontrol->value.enumerated.item[0]); \
	if (ucontrol->value.enumerated.item[0]) { \
		IAXXX_SND_SOC_UPDATE_BITS(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
					1 << plugin, 1 << plugin); \
		if (ret < 0) \
			return ret; \
		iaxxx->plugin_blk_en[plugin] = 1; \
	} else { \
		IAXXX_SND_SOC_UPDATE_BITS(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
					1 << plugin, 0 << plugin); \
		if (ret < 0) \
			return ret; \
		iaxxx->plugin_blk_en[plugin] = 0; \
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
						      &status, IAXXX_BLOCK_0); \
		if (ret) \
			dev_err(priv->dev,	\
			"%s: Plg HDR En Update blk0 failed %d, status %u\n", \
			__func__, ret, status); \
		\
	} \
	\
	return ret;\
} \
\
static int iaxxxcore_get_plgin##plugin##_Blk0En( \
				struct snd_kcontrol *kcontrol, \
				struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->plugin_blk_en[plugin]; \
	return 0; \
} \
\
static int iaxxxcore_set_plgin##plugin##_Blk1En( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 status = 0; \
	int ret = 0; \
	\
	dev_info(codec->dev, "%s: enter, plugin %d, enb %d\n", \
		__func__, plugin, ucontrol->value.enumerated.item[0]); \
	if (ucontrol->value.enumerated.item[0]) { \
		IAXXX_SND_SOC_UPDATE_BITS(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
					1 << plugin, 1 << plugin); \
		if (ret < 0) \
			return ret; \
		iaxxx->plugin_blk_en[plugin] = 1; \
	} else { \
		IAXXX_SND_SOC_UPDATE_BITS(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
					1 << plugin, 0 << plugin); \
		if (ret < 0) \
			return ret; \
		iaxxx->plugin_blk_en[plugin] = 0; \
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
						      &status, IAXXX_BLOCK_1); \
		if (ret) \
			dev_err(priv->dev, \
			"%s: Plg HDR En Update blk1 failed %d, status %u\n", \
			__func__, ret, status); \
		\
	} \
	\
	return ret;\
} \
\
static int iaxxxcore_get_plgin##plugin##_Blk1En( \
				struct snd_kcontrol *kcontrol, \
				struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->plugin_blk_en[plugin]; \
	return 0; \
}

IAXXX_PLUGIN_BLK_SET_GET(PLUGIN0)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN1)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN2)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN3)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN4)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN5)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN6)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN7)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN8)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN9)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN10)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN11)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN12)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN13)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN14)

#define IAXXX_PLUGIN_EN_CTLS(plugin, plugin_name) \
	SOC_SINGLE_BOOL_EXT(plugin_name "Blk0En", 0, \
		       iaxxxcore_get_plgin##plugin##_Blk0En, \
		       iaxxxcore_set_plgin##plugin##_Blk0En), \
	SOC_SINGLE_BOOL_EXT(plugin_name "Blk1En", 0, \
		       iaxxxcore_get_plgin##plugin##_Blk1En, \
		       iaxxxcore_set_plgin##plugin##_Blk1En)

#define IAXXX_PLUGIN_DAPM_CTLS(plugin, plugin_name) \
static const SOC_ENUM_SINGLE_DECL(plugin##_en_enum, \
						SND_SOC_NOPM, plugin, \
						plugin##_off_on_texts); \
static const struct snd_kcontrol_new plugin##_mux =	\
	SOC_DAPM_ENUM(plugin_name "Enable", plugin##_en_enum)

IAXXX_PLUGIN_DAPM_CTLS(PLUGIN0, "Plgin0");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN1, "Plgin1");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN2, "Plgin2");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN3, "Plgin3");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN4, "Plgin4");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN5, "Plgin5");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN6, "Plgin6");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN7, "Plgin7");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN8, "Plgin8");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN9, "Plgin9");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN10, "Plgin10");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN11, "Plgin11");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN12, "Plgin12");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN13, "Plgin13");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN14, "Plgin14");

static int iaxxx_put_start_pdm(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	int try = IAXXX_MAX_RETRY, ret = -EINVAL;
	uint32_t port_mics_en = ucontrol->value.integer.value[0];

	dev_dbg(codec->dev, "enter %s: mics to en:%u on port:%d\n",
				__func__, port_mics_en, port);

	if (iaxxx->port_start_en[port] == port_mics_en)
		return 0;

	if (port_mics_en > IAXXX_RX_DMIC_ENABLE_MASK)
		return ret;

	/* Read DMIC enable busy reg, check for DMIC is not busy
	 * set DMIC enable only if DMIC enable busy reg is not set
	 * Check again DMIC enable busy reg is cleared by HW.
	 * DMIC enable busy will get cleared once port clock is available.
	 */
	if (!IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(iaxxx->apll_src)) {
		while (snd_soc_component_read32(codec, dmic_busy_addr[port]) &&
		       (try-- != 0))
			usleep_range(IAXXX_DELAY_2MS, IAXXX_DELAY_2MS + 5);

		if (!try) {
			dev_err(codec->dev,
				"%s: port: %d DMIC enable bit busy before\n",
				__func__, port);
			return ret;
		}
	}


	/* DMIC enable */
	IAXXX_SND_SOC_UPDATE_BITS(codec, dmic_enable_addr[port], port_mics_en,
				  port_mics_en);
	if (ret < 0)
		return ret;

	if (!IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(iaxxx->apll_src)) {
		try = IAXXX_MAX_RETRY;
		/*  Read DMIC ENABLE busy  */
		while (snd_soc_component_read32(codec, dmic_busy_addr[port]) &&
		       (try-- != 0))
			usleep_range(2000, 2005);

		if (!try) {
			pr_err("port: %d DMIC enable bit busy after",
			       port);
			return ret;
		}
	}

	iaxxx->port_start_en[port] = ucontrol->value.integer.value[0];
	return 0;
}

static int iaxxx_put_port_clk_stop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	u32 apll_src;
	u32 status = 0;
	int ret = 0;

	dev_dbg(codec->dev, "%s: enter\n", __func__);

	if (port > NUM_I2S_GEN) {
		dev_err(codec->dev,
			"%s: PDM port setup failed Invalid port number %d\n",
			__func__, port);
		return -EINVAL;
	}

	pr_info("dev id-%d: stop clock src %d", priv->dev_id, port);

	if (iaxxx->port_filter[port] == ucontrol->value.integer.value[0])
		return 0;

	apll_src = iaxxx->apll_src;

	if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src)) {
		/*
		 * In case of IA8201, HW has a limitation that leaf nodes can't
		 * be configured if root clock is not present. When source of
		 * A_CLK is selected as any of the port clock, can't guarantee
		 * that clock will be present during configuration. Hence,
		 * change APLL clock source to known clock source (CLK_IN)
		 * for the configuration and later switch it back to port clock.
		 */
		ret = iaxxx_config_apll(priv, priv->sys_clk_src,
				iaxxx->apll_clk,
				priv->sys_clk_in_freq);
		if (ret) {
			pr_err("dev id-%d: Apll clk src set failed : %d",
							priv->dev_id, ret);
			return ret;
		}
	}

	/* DISABLE I2S PORT */
	/* CNR0_I2S_Enable  - Disable I2S  */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(port),
			IAXXX_CNR0_I2S_ENABLE_LOW);
	if (ret < 0)
		return ret;
	/* I2S Trigger - Enable */
	ret = snd_soc_component_write(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, ret);
		return ret;
	}

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
			1 << port, 0x0);
	if (ret < 0)
		return ret;

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
							IAXXX_BLOCK_0);
	if (ret) {
		dev_err(codec->dev,
			"%s: I2S Port PWR En Addr Update block failed : %d\n",
							__func__, ret);
		return ret;
	}

	iaxxx->port_filter[port] = 0;

	if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src)) {
		/* Switch back APLL clock source to port clock */
		ret = iaxxx_config_apll(priv, iaxxx->apll_src,
				  iaxxx->apll_clk,
				  iaxxx->apll_input_freq);
		if (ret) {
			pr_err("dev id-%d: Apll clk src set failed : %d",
							priv->dev_id, ret);
			return ret;
		}
	}

	return ret;
}

#define IAXXX_PDM_PORT_START_SET_GET(port_name, port) \
static int iaxxx_put_start_##port_name(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_put_start_pdm(kcontrol, ucontrol, port); \
	else { \
		iaxxx->port_start_en[port] = 0; \
		return 0; \
	} \
} \
static int iaxxx_get_start_##port_name(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_start_en[port]; \
	return 0; \
}

IAXXX_PDM_PORT_START_SET_GET(dmic0, IAXXX_DMIC0_CLK_SRC)
IAXXX_PDM_PORT_START_SET_GET(dmic1, IAXXX_DMIC1_CLK_SRC)
IAXXX_PDM_PORT_START_SET_GET(cdc0, IAXXX_CDC0_CLK_SRC)
IAXXX_PDM_PORT_START_SET_GET(cdc1, IAXXX_CDC1_CLK_SRC)

static int iaxxx_calc_i2s_div(struct snd_soc_component *codec,
			u32 bits_per_frame, u32 sampling_rate,
			u32 *period, u32 *div_val, u32 *nr_val)
{
	u32 bit_clk;
	u32 divider;
	u32 r_val;
	u32 n_val;
	u32 arr_len;
	int i = 0;
	u32 apll_clk = 0;
	u32 aclk_id;
	u32 i2s_master_freq;

	apll_clk = snd_soc_component_read32(codec, IAXXX_SRB_SYS_AUDIO_CLOCK_ADDR);
	if (apll_clk == 0 || apll_clk == IAXXX_SOC_READ_INVALID_VAL) {
		pr_err("apll clk read failed : %d", apll_clk);
		if (!apll_clk)
			apll_clk = -EINVAL;
		return apll_clk;
	}
	pr_info("apll clk %d", apll_clk);

	i2s_master_freq = snd_soc_component_read32(codec, IAXXX_SRB_SYS_I2SM_CLOCK_ADDR);
	if (i2s_master_freq == IAXXX_SOC_READ_INVALID_VAL) {
		pr_err("i2s master freq read failed : %d", i2s_master_freq);
		return -EINVAL;
	}
	pr_info("I2S master freq %d", i2s_master_freq);

	if (i2s_master_freq == 0)
		i2s_master_freq = apll_clk;

	for (i = 0; i < IAXXX_ACLK_FREQ_MAX; i++) {
		if (i2s_master_freq == iaxxx_apllClk_Val[i]) {
			aclk_id = i;
			break;
		}
	}

	if (i == IAXXX_ACLK_FREQ_MAX) {
		pr_err("No valid apll clk entry");
		return -EINVAL;
	}

	*div_val = 0;
	*nr_val = 0;
	/* get
	 * bit_clk
	 * freq
	 * */
	bit_clk = (sampling_rate * (bits_per_frame + 1));
	arr_len = ARRAY_SIZE(iaxxx_bitClock_Val);
	for (i = 0; i < arr_len; i++) {
		if (bit_clk == iaxxx_bitClock_Val[i]) {
			bit_clk = i;
			break;
		}
	}

	if (i == arr_len) {
		pr_err("BitClk is invalid sample rate %d, bits per frame %d",
		       sampling_rate, bits_per_frame);
		return -EINVAL;
	}

	n_val = i2s_div_config[aclk_id][bit_clk].N;
	r_val = i2s_div_config[aclk_id][bit_clk].R;
	*period = i2s_div_config[aclk_id][bit_clk].period;
	divider = i2s_div_config[aclk_id][bit_clk].HL;

	*div_val = *div_val |
		((divider << IAXXX_I2S_I2S0_HL_P_POS) &
		 IAXXX_I2S_I2S0_HL_P_MASK);
	*nr_val = *nr_val |
		(((n_val << IAXXX_I2S_I2S0_NR_N_POS) &
		  IAXXX_I2S_I2S0_NR_N_MASK) |
		 ((r_val << IAXXX_I2S_I2S0_NR_R_POS) &
		  IAXXX_I2S_I2S0_NR_R_MASK));

	return 0;
}

static int iaxxx_set_i2s_controller(struct snd_soc_component *codec, bool is_pseudo,
				    int id)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	u32 bits_per_frame = 0;
	u32 period = 0, div_val = 0, nr_val = 0, status = 0;
	u32 clk_ctrl_val = 0;
	u32 cfg_val;
	u32 apll_src = iaxxx->apll_src;
	u32 cnr0_i2s_enable_val;
	int ret;

	if (is_pseudo) {
		dev_err(codec->dev,
			"%s: Pseudo mode not supported\n", __func__);
		return -EINVAL;
	}

	cnr0_i2s_enable_val = snd_soc_component_read32(codec, IAXXX_CNR0_I2S_ENABLE_ADDR);
	if (cnr0_i2s_enable_val == IAXXX_SOC_READ_INVALID_VAL) {
		dev_err(codec->dev, "%s: reg 0x%x read failed\n",
			__func__, IAXXX_CNR0_I2S_ENABLE_ADDR);
		return -EINVAL;
	}
	pr_info("dev id-%d: cnr0 i2s enable register val 0x%x",
					priv->dev_id, cnr0_i2s_enable_val);

	/*
	 * No need of temp A_CLK switching, if any of the I2S chan is enabled
	 */
	if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src) &&
			(cnr0_i2s_enable_val == 0)) {
		/*
		 * In case of IA8201, HW has a limitation that leaf nodes can't
		 * be configured if root clock is not present. When source of
		 * A_CLK is selected as any of the port clock, can't guarantee
		 * that clock will be present during configuration. Hence,
		 * change APLL clock source to known clock source (CLK_IN)
		 * for the configuration and later switch it back to port clock.
		 */
		ret = iaxxx_config_apll(priv, priv->sys_clk_src,
				iaxxx->apll_clk,
				priv->sys_clk_in_freq);
		if (ret) {
			pr_err("dev id-%d: APLL CLK_IN clk src set failed : %d",
							priv->dev_id, ret);
			return ret;
		}
	}

	/* TODO need to move to pm ops functions in future */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << id), (0x1 << id));
	if (ret < 0)
		return ret;

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			    IAXXX_BLOCK_0);
	if (ret) {
		dev_err(codec->dev,
			"%s: I2S Port PWR En Addr Update block failed : %d\n",
			__func__, ret);
		return ret;
	}

	/* CNR0_I2S_Enable  - Disable I2S */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(id),
		IAXXX_CNR0_I2S_ENABLE_LOW);
	if (ret < 0)
		return ret;

	/* I2S Trigger - Disable I2S */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, ret);
		return ret;
	}

	bits_per_frame = (((iaxxx->clk_param_info[id].word_len + 1) *
			(iaxxx->clk_param_info[id].words_per_frame + 1)) - 1);

	dev_dbg(codec->dev, "bits_per_frame :%d\n", bits_per_frame);

	if (iaxxx_calc_i2s_div(codec, bits_per_frame,
			       iaxxx->clk_param_info[id].sample_rate,
			       &period, &div_val, &nr_val)) {
		dev_err(codec->dev, "%s: Invalid arguments requested\n",
								__func__);
		return -EINVAL;
	}
	/* Disable hl divider */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_HL_ADDR(id),
		      IAXXX_I2S_I2S0_HL_DISABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_HL_ADDR(id),
			IAXXX_I2S_I2S0_HL_DISABLE, ret);
		return ret;
	}
	/* Set HL value */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_HL_ADDR(id),
		div_val | IAXXX_I2S_I2S0_HL_ENABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_HL_ADDR(id),
			div_val | IAXXX_I2S_I2S0_HL_ENABLE, ret);
		return ret;
	}

	/* Disable NR divider */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_NR_ADDR(id),
		IAXXX_I2S_I2S0_NR_DISABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_NR_ADDR(id),
			IAXXX_I2S_I2S0_NR_DISABLE, ret);
		return ret;
	}
	/* Set NR value */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_NR_ADDR(id),
		nr_val | IAXXX_I2S_I2S0_NR_ENABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_NR_ADDR(id),
			nr_val | IAXXX_I2S_I2S0_NR_ENABLE, ret);
		return ret;
	}

	cfg_val = ((iaxxx->clk_param_info[id].fs_duration <<
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS) &
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK) |
		(iaxxx->clk_param_info[id].fs_pol ?
		 IAXXX_I2S_GEN_CFG_FS_POL_HIGH : IAXXX_I2S_GEN_CFG_FS_POL_LOW) |
		(iaxxx->clk_param_info[id].clk_pol ?
		 IAXXX_I2S_GEN_CFG_CLK_POL_HIGH :
		 IAXXX_I2S_GEN_CFG_CLK_POL_LOW) |
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE |
		((bits_per_frame <<
		 IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS) &
		 IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK) |
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE;

	/* Configure fsPol, clkPol, etc in config register */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
		cfg_val);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
			cfg_val, ret);
		return ret;
	}

	/* Configure frameSync_align register */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_FS_ALIGN_ADDR(id), 0);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_FS_ALIGN_ADDR(id),
			0, ret);
		return ret;
	}

	/* Clk control */
	clk_ctrl_val = (((period >> 1) - 1) <<
		IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS) |
		(((period - 1) << IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS)
		& IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_MASK);

	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_CLK_CTRL_ADDR(id),
			    clk_ctrl_val);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_CLK_CTRL_ADDR(id),
			clk_ctrl_val, ret);
		return ret;
	}

	if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src) &&
		(cnr0_i2s_enable_val == 0)) {
		/* Switch back APLL clock source to port clock */
		ret = iaxxx_config_apll(priv, iaxxx->apll_src,
				  iaxxx->apll_clk,
				  iaxxx->apll_input_freq);
		if (ret) {
			pr_err("dev id-%d: APLL Clk src Back to port clk failed : %d",
			       priv->dev_id, ret);
			return ret;
		}
		/*
		 * FW will take care of I2S Clock Start, because if i2s clock is
		 * started now then for CommB PDM port configuration PDM mic
		 * will be in unexpected state when clock is disabled for
		 * sometime by FW during stream enable
		 */
	} else {
		/*
		 * As per FW design for non Single clock source route,
		 * I2S clock has to be configured and enabled by host.
		 *
		 * If any I2S is enabled that means its an incremental route
		 * and FW will not go for APLL restart cycle, where FW
		 * updates this register.
		 */

		/* Start the I2S clock */
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
					  IAXXX_CNR0_I2S_ENABLE_MASK(id),
					  IAXXX_CNR0_I2S_ENABLE_HIGH << id);
		if (ret < 0)
			return ret;

		/* I2S Trigger - Disable I2S */
		ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				    IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL);
		if (ret < 0) {
			dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
				__func__, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, ret);
			return ret;
		}

	}

	return 0;
}

enum {
	PDM_IN,
	PDM_OUT,
};

static int iaxxx_pdm_mic_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol,
			int dmic, u32 port_sel)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct device *dev = codec->dev;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(dev);
	u32 pdm_bclk = 0;
	u8 aud_port_clk = 0;
	u32 cic_hb = 0, hb_dec = 0;
	u32 io_ctrl_reg, io_ctrl_mask, io_ctrl_val;
	u32 io_ctrl_clk_reg, io_ctrl_clk_val;
	u32 cic_ctrl = 0;
	u32 cic_rx_rt_ctrl = 0, cic_dec = 0, cic_tx_rt_ctrl;
	int ret = -EINVAL;
	int io_port_mic = dmic;
	u32 status = 0;
	u32 clk_src = 0;
	int pdm_mstr = 0;
	u32 port_type = 0;
	u32 port_dir = 0;
	int i, j;
	u32 reg_val;
	u32 mono_mask = PLAT_IO_CTRL_AND_SEL_CDC_CLK_MASK;

	pdm_bclk = iaxxx->pdm_bclk;
	aud_port_clk = iaxxx->pdm_aud_port_clk;

	/* Value contains clock source info in the 1st and 2nd bit */
	clk_src = ucontrol->value.integer.value[0];

	dev_info(dev, "%s: Port sel %d dmic %d clk_src:%d\n", __func__,
			port_sel, dmic, clk_src);

	if ((dmic >= PDM_DMIC_IN0) && (dmic <= PDM_DMIC_IN7)) {
		/* PDM port is of DMIC type */
		port_type = PDM_PORT_DMIC;
		port_dir = 1;
		pdm_mstr = 1;
	} else if ((dmic >= PDM_CDC0_IN0) && (dmic <= PDM_CDC3_IN7)) {
		/* PDM port is of ADC type */
		port_type = PDM_PORT_ADC;
		dmic  = dmic - PDM_CDC0_IN0;
		port_dir = 1;
		pdm_mstr = 1;
	} else if ((dmic >= PDM_DMIC_OUT0) &&
				(dmic <= PDM_DMIC_OUT1)) {
		/* PDM port is of Output type */
		port_type = PDM_PORT_PDMO;
		port_dir = 0;
		pdm_mstr = 0;
	} else if ((dmic >= PDM_DMIC_MONO_IN0) &&
				(dmic <= PDM_DMIC_MONO_IN3)) {
		/* PDM port is of Mono input type
		 * In mono mode, PDM inputs are mapped to CICs 4-7
		 */
		port_type = PDM_PORT_MONO;
		dmic = dmic - PDM_DMIC_MONO_IN0;
		port_dir = 1;
		pdm_mstr = 1;
	}

	dev_info(dev, "port sel %d clk_src %d pdm_mstr %d\n", port_sel,
		clk_src, pdm_mstr);

	if (port_dir) {
		if (port_sel) {
			io_ctrl_clk_reg =
				iaxxx_dynamic_ioctrl_clkin_port
				[port_sel - 1][io_port_mic][0];
			io_ctrl_clk_val =
				iaxxx_dynamic_ioctrl_clkin_port
				[port_sel - 1][io_port_mic]
				[pdm_mstr + 1];
		} else {
			io_ctrl_clk_reg =
				iaxxx_io_ctrl_clk_in_port[io_port_mic][0];
			io_ctrl_clk_val =
				iaxxx_io_ctrl_clk_in_port[io_port_mic]
				[pdm_mstr + 1];
		}
	} else {
		io_ctrl_clk_reg =
			iaxxx_io_ctrl_clk_out_port[clk_src][0];
		io_ctrl_clk_val =
			iaxxx_io_ctrl_clk_out_port[clk_src]
			[pdm_mstr + 1];
	}

	pr_info("IO Ctrl Clk addr 0x%x val 0x%x", io_ctrl_clk_reg,
		io_ctrl_clk_val);

	if (port_type == PDM_PORT_MONO && !port_sel) {
		for (i = 0; i < ARRAY_SIZE(cdc0_io_ctrl_reg_list); i++) {
			if (io_ctrl_clk_reg == cdc0_io_ctrl_reg_list[i]) {
				for (j = 0;
				     j < ARRAY_SIZE(cdc0_io_ctrl_reg_list);
				     j++) {
					reg_val = snd_soc_component_read32(codec,
						cdc0_io_ctrl_reg_list[j]);
					if (reg_val & mono_mask) {
						io_ctrl_clk_val &= (~mono_mask);
						break;
					}
				}
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(cdc1_io_ctrl_reg_list); i++) {
			if (io_ctrl_clk_reg == cdc1_io_ctrl_reg_list[i]) {
				for (j = 0;
				     j < ARRAY_SIZE(cdc1_io_ctrl_reg_list);
				     j++) {
					reg_val = snd_soc_component_read32(codec,
						cdc1_io_ctrl_reg_list[j]);
					if (reg_val & mono_mask) {
						io_ctrl_clk_val &= (~mono_mask);
						break;
					}
				}
				break;
			}
		}
	}

	pr_info("IO Ctrl Clk addr 0x%x val 0x%x", io_ctrl_clk_reg,
		io_ctrl_clk_val);

	ret = snd_soc_component_write(codec, io_ctrl_clk_reg, io_ctrl_clk_val);
	if (ret < 0) {
		dev_err(dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, io_ctrl_clk_reg,
			io_ctrl_clk_val, ret);
		return ret;
	}

	if ((port_type == PDM_PORT_DMIC) || (port_type == PDM_PORT_ADC)
					|| (port_type == PDM_PORT_MONO)) {
		IAXXX_SND_SOC_UPDATE_BITS(
				codec, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
				(1 << dmic), (1 << dmic));
		if (ret < 0)
			return ret;
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
					&status, IAXXX_BLOCK_0);
		if (ret) {
			dev_err(codec->dev,
			"%s: PDMI PORT PWR EN ADDR Update blk failed : %d\n",
				__func__, ret);
			return ret;
		}

		IAXXX_SND_SOC_UPDATE_BITS(codec,
				IAXXX_CNR0_CIC_RX_ADTL_CTRL_ADDR,
				IAXXX_CIC_ADTL_RX_MASK(dmic), 0);
		if (ret < 0)
			return ret;

		/* Get CIC decimation value and half band decimation value */
		ret = get_decimator_val(pdm_bclk, aud_port_clk,
					&cic_dec, &hb_dec);
		if (ret) {
			dev_err(dev, "%s: get decimation value failed : %d\n",
			       __func__, ret);
			return ret;
		}

		/* Set state bit, Disable the Filter */
		if ((dmic & 0x1) != 0) {
			cic_dec = cic_dec << IAXXX_CNR0_CIC_RX_0_1_M_1_POS;
			/* we are in the 'odd' channel : 1/3/5/7 */
			IAXXX_SND_SOC_UPDATE_BITS(codec, cic_rx_addr[dmic],
				IAXXX_CNR0_CIC_RX_0_1_M_1_MASK, cic_dec);
			if (ret < 0)
				return ret;
		} else {
			cic_dec = cic_dec << IAXXX_CNR0_CIC_RX_0_1_M_0_POS;
			/* we are in even channels : 0/2/4/6 */
			IAXXX_SND_SOC_UPDATE_BITS(codec, cic_rx_addr[dmic],
				IAXXX_CNR0_CIC_RX_0_1_M_0_MASK, cic_dec);
			if (ret < 0)
				return ret;
		}

		cic_ctrl = IAXXX_PDM_POLARITY <<
			IAXXX_CNR0_CIC_CTRL_RX_POL_POS(dmic);
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_POL_CTRL_ADDR,
			IAXXX_CNR0_CIC_CTRL_RX_POL_MASK(dmic), cic_ctrl);
		if (ret < 0)
			return ret;

		/* setup input clk source (base/alternative) & Bit Polarity*/
		if (clk_src == IAXXX_DMIC0_CLK_SRC ||
		    clk_src == IAXXX_CDC0_CLK_SRC)
			cic_ctrl = IAXXX_DMIC0_CLK << dmic;
		else
			cic_ctrl = IAXXX_DMIC1_CLK << dmic;

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_CLOCK_CTRL_ADDR,
			IAXXX_CNR0_CIC_CTRL_RX_MASK(dmic), cic_ctrl);
		if (ret < 0)
			return ret;

		/* Green box /Half band decimation filter */
		cic_hb = hb_dec << IAXXX_CNR0_CIC_HB_CIC_RX_POS(dmic);
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_RX_HB_ADDR,
			IAXXX_CNR0_CIC_HB_CIC_RX_MASK(dmic), cic_hb);
		if (ret < 0)
			return ret;

		if (port_type == PDM_PORT_DMIC) {
			cic_rx_rt_ctrl = IAXXX_CIC_MIC_ENABLE <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_POS(dmic);
			cic_rx_rt_ctrl = cic_rx_rt_ctrl |
				(IAXXX_CIC_S_DMIC_ENABLE <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_S_POS(dmic));
		}
		cic_rx_rt_ctrl = cic_rx_rt_ctrl | (IAXXX_CLK_ENABLE <<
			IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_POS(dmic));

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
			IAXXX_CNR0_CIC_RX_RT_MASK(dmic), cic_rx_rt_ctrl);
		if (ret < 0)
			return ret;

		if (port_sel) {
			io_ctrl_reg = iaxxx_dynamic_ioctrl_set_data
				[port_sel - 1][io_port_mic][0];
			io_ctrl_mask = iaxxx_dynamic_ioctrl_set_data
				[port_sel - 1][io_port_mic][1];
			io_ctrl_val = iaxxx_dynamic_ioctrl_set_data
				[port_sel - 1][io_port_mic][1];
		} else {
			io_ctrl_reg = iaxxx_io_ctrl_data[io_port_mic][0];
			io_ctrl_mask = iaxxx_io_ctrl_data[io_port_mic][1];
			io_ctrl_val = iaxxx_io_ctrl_data[io_port_mic][1];
		}

		ret = snd_soc_component_write(codec, io_ctrl_reg, io_ctrl_val);
		if (ret < 0) {
			dev_err(dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
				__func__, io_ctrl_reg,
				io_ctrl_val, ret);
			return ret;
		}

		/* Set state bit, Disable the Filter */
		if ((dmic & 0x1) != 0) {
			/* we are in the 'odd' channel : 1/3/5/7 */
			IAXXX_SND_SOC_UPDATE_BITS(codec, cic_rx_addr[dmic],
				IAXXX_CNR0_CIC_RX_0_1_CLR_1_MASK,
				(0 << IAXXX_CNR0_CIC_RX_0_1_CLR_1_POS));
			if (ret < 0)
				return ret;
		} else {
			/* we are in even channels : 0/2/4/6 */
			IAXXX_SND_SOC_UPDATE_BITS(codec, cic_rx_addr[dmic],
				IAXXX_CNR0_CIC_RX_0_1_CLR_0_MASK,
				(0 << IAXXX_CNR0_CIC_RX_0_1_CLR_0_POS));
			if (ret < 0)
				return ret;
		}

	} else if (port_type == PDM_PORT_PDMO) {
		/* get the output Port Number */
		dmic = dmic - PDM_DMIC_OUT0;

		IAXXX_SND_SOC_UPDATE_BITS(codec,
					IAXXX_SRB_PDMO_PORT_PWR_EN_ADDR,
					(1 << dmic), (1 << dmic));
		if (ret < 0)
			return ret;

		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
					&status, IAXXX_BLOCK_0);
		if (ret) {
			dev_err(codec->dev,
			"%s: PDMO PORT PWR EN ADDR Update blk failed : %d\n",
				__func__, ret);
			return ret;
		}

		IAXXX_SND_SOC_UPDATE_BITS(codec,
				IAXXX_CNR0_CIC_TX_ADTL_CTRL_ADDR,
				IAXXX_CIC_ADTL_TX_MASK(dmic), 0);
		if (ret < 0)
			return ret;

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_0_1_ADDR,
				IAXXX_CIC_TX_DSEL_MASK(dmic) |
				IAXXX_CIC_TX_PHASE_MASK(dmic),
				IAXXX_CIC_TX_DSEL_MASK(dmic) |
				(IAXXX_CIC_TX_PHASE_OP_ENABLE <<
				 IAXXX_CIC_TX_PHASE_MASK(dmic)));
		if (ret < 0)
			return ret;

		/* Get CIC decimation value and half band decimation value */
		ret = get_decimator_val(pdm_bclk, aud_port_clk,
					&cic_dec, &hb_dec);
		if (ret) {
			dev_err(dev, "%s: get decimation value failed : %d\n",
			       __func__, ret);
			return ret;
		}

		/* Write to PHASE */
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_0_1_ADDR,
				IAXXX_CIC_TX_L_MASK(dmic), cic_dec);
		if (ret < 0)
			return ret;

		cic_ctrl = (IAXXX_PDM_POLARITY <<
				IAXXX_CIC_POL_CTRL_TX_POL_POS(dmic));
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_POL_CTRL_ADDR,
			IAXXX_CIC_POL_CTRL_TX_POL_MASK(dmic), cic_ctrl);
		if (ret < 0)
			return ret;

		/* setup input clk source (base/alternative) & Bit Polarity*/
		if (clk_src == IAXXX_DMIC0_CLK_SRC ||
		    clk_src == IAXXX_CDC0_CLK_SRC)
			cic_ctrl = IAXXX_DMIC0_CLK <<
				IAXXX_CIC_CLOCK_CTRL_TX_AC_POS(dmic);
		else
			cic_ctrl = IAXXX_DMIC1_CLK <<
				IAXXX_CIC_CLOCK_CTRL_TX_AC_POS(dmic);

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_CLOCK_CTRL_ADDR,
			IAXXX_CIC_CLOCK_CTRL_TX_AC_MASK(dmic),
			cic_ctrl);
		if (ret < 0)
			return ret;

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_HB_ADDR,
			IAXXX_CIC_HB_CIC_TX_MASK(dmic), cic_hb);
		if (ret < 0)
			return ret;

		cic_tx_rt_ctrl = (dmic == 0) ?
			IAXXX_CNR0_CIC_TX_RT_CTRL_CLK_EN_0_MASK :
			IAXXX_CNR0_CIC_TX_RT_CTRL_CLK_EN_1_MASK;

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_RT_CTRL_ADDR,
			cic_tx_rt_ctrl, cic_tx_rt_ctrl);
		if (ret < 0)
			return ret;

		io_ctrl_reg = iaxxx_io_ctrl_data[io_port_mic][0];
		io_ctrl_mask = iaxxx_io_ctrl_data[io_port_mic][1];
		io_ctrl_val = iaxxx_io_ctrl_data[io_port_mic][1];

		/* Set up the IO_CTRL appropriately */
		ret = snd_soc_component_write(codec, io_ctrl_reg, io_ctrl_val);
		if (ret < 0) {
			dev_err(dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
				__func__, io_ctrl_reg,
				io_ctrl_val, ret);
			return ret;
		}

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_0_1_ADDR,
			IAXXX_CIC_TX_CLR_MASK(dmic), 0);
		if (ret < 0)
			return ret;

		if (clk_src == IAXXX_DMIC0_CLK_SRC ||
		    clk_src == IAXXX_CDC0_CLK_SRC) {
			IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_AO_CLK_CFG_ADDR,
				IAXXX_AO_CLK_CFG_PCM_PORT2_DO_OE_MASK,
				IAXXX_AO_CLK_CFG_PCM_PORT2_DO_OE_MASK);
			if (ret < 0)
				return ret;
		} else {
			IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_AO_CLK_CFG_ADDR,
				IAXXX_AO_CLK_CFG_PCM_PORT1_DO_OE_MASK,
				IAXXX_AO_CLK_CFG_PCM_PORT1_DO_OE_MASK);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

static int iaxxx_pdm_port_clr(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol,
			      int port, int port_mic, u32 port_sel)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 io_ctrl_reg, io_ctrl_mask;
	u32 io_ctrl_clk_reg;
	u32 io_ctrl_clk_reset;
	u32 port_type = 0;
	u32 rx_rt_clr = 0, tx_rt_clr = 0;
	u32 op_port_mic = 0;
	u32 status = 0;
	int ret = 0;
	int port_dir;

	dev_info(codec->dev, "%s: clk src %d for mic %d\n",
		 __func__, port, port_mic);

	if (port_sel) {
		io_ctrl_reg = iaxxx_dynamic_ioctrl_set_data
			[port_sel - 1][port_mic][0];
		io_ctrl_mask = iaxxx_dynamic_ioctrl_set_data
			[port_sel - 1][port_mic][1];
		io_ctrl_clk_reg = iaxxx_dynamic_ioctrl_clkin_port
			[port_sel - 1][port_mic][0];
		io_ctrl_clk_reset =
			iaxxx_dynamic_io_ctrl_clk_in_reset_val
			[port_sel - 1][port_mic];
	} else {
		io_ctrl_reg = iaxxx_io_ctrl_data[port_mic][0];
		io_ctrl_mask = iaxxx_io_ctrl_data[port_mic][1];
		io_ctrl_clk_reg = iaxxx_io_ctrl_clk_in_port[port_mic][0];
		io_ctrl_clk_reset = iaxxx_ioctrl_clk_reset_val[port_mic];
	}

	if (port_mic > PDM_DMIC_MONO_IN3) {
		dev_err(codec->dev,
			"%s: PDM port stop failed Invalid port number %d\n",
			__func__, port_mic);
		return -EINVAL;
	}

	if ((port_mic >= PDM_DMIC_IN0) && (port_mic <= PDM_DMIC_IN7)) {
		/* PDM port is of DMIC type */
		port_type = PDM_PORT_DMIC;
		port_dir = 1;
	} else if ((port_mic >= PDM_CDC0_IN0) && (port_mic <= PDM_CDC3_IN7)) {
		/* PDM port is of ADC type */
		port_type = PDM_PORT_ADC;
		port_mic = port_mic - PDM_CDC0_IN0;
		port_dir = 1;
	} else if ((port_mic >= PDM_DMIC_OUT0) && (port_mic <= PDM_DMIC_OUT1)) {
		/* PDM port is of Output type */
		port_type = PDM_PORT_PDMO;
		port_dir = 0;
	} else if ((port_mic >= PDM_DMIC_MONO_IN0) &&
		   (port_mic <= PDM_DMIC_MONO_IN3)) {
		port_type = PDM_PORT_MONO;
		port_mic = port_mic - PDM_DMIC_MONO_IN0;
		port_dir = 1;
	}

	if ((port_type == PDM_PORT_DMIC) || (port_type == PDM_PORT_ADC)
		 || (port_type == PDM_PORT_MONO)) {
		/* Set state bit, Disable the Filter */
		if ((port_mic & 0x1) != 0) {
			/* we are in the 'odd' channel : 1/3/5/7 */
			IAXXX_SND_SOC_UPDATE_BITS(codec, cic_rx_addr[port_mic],
				IAXXX_CNR0_CIC_RX_0_1_CLR_1_MASK,
				(1 << IAXXX_CNR0_CIC_RX_0_1_CLR_1_POS));
			if (ret < 0)
				return ret;
		} else {
			/* we are in even channels : 0/2/4/6 */
			IAXXX_SND_SOC_UPDATE_BITS(codec, cic_rx_addr[port_mic],
				IAXXX_CNR0_CIC_RX_0_1_CLR_0_MASK,
				(1 << IAXXX_CNR0_CIC_RX_0_1_CLR_0_POS));
			if (ret < 0)
				return ret;
		}

		/* Set up the IO_CTRL appropriately */
		IAXXX_SND_SOC_UPDATE_BITS(codec, io_ctrl_reg, io_ctrl_mask, 0);
		if (ret < 0)
			return ret;

		/* Reset and update routing and disable the Clock to the CIC*/
		rx_rt_clr = (IAXXX_CIC_RX_RT_CTRL_OPT_MASK <<
			(IAXXX_CIC_RX_RT_CTRL_OPT_SIZE * port_mic));
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
							rx_rt_clr, 0);
		if (ret < 0)
			return ret;
	} else if (port_type == PDM_PORT_PDMO) {
		/* get the output port mic number */
		op_port_mic = port_mic - PDM_DMIC_OUT0;
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_0_1_ADDR,
			(IAXXX_CNR0_CIC_TX_0_1_CLR_0_MASK << op_port_mic), 0);
		if (ret < 0)
			return ret;

		/* Set up the IO_CTRL appropriately */
		IAXXX_SND_SOC_UPDATE_BITS(codec, io_ctrl_reg, io_ctrl_mask, 0);
		if (ret < 0)
			return ret;

		/* Reset and Disable Clock to the CIC */
		tx_rt_clr = ((IAXXX_CNR0_CIC_TX_RT_CTRL_CLK_EN_0_MASK) |
				(IAXXX_CNR0_CIC_TX_RT_CTRL_CLK_EN_1_MASK));
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_RT_CTRL_ADDR,
						tx_rt_clr, 0);
		if (ret < 0)
			return ret;
	}

	/* Disable DMIC */
	IAXXX_SND_SOC_UPDATE_BITS(codec, dmic_enable_addr[port],
			(IAXXX_DMIC_ENABLE_MASK << port_mic), 0);
	if (ret < 0)
		return ret;

	if ((port_type == PDM_PORT_DMIC) || (port_type == PDM_PORT_ADC) ||
	    (port_type == PDM_PORT_MONO)) {
		IAXXX_SND_SOC_UPDATE_BITS(codec,
			IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
						(1 << port_mic), 0);
		if (ret < 0)
			return ret;
	} else if (port_type == PDM_PORT_PDMO) {
		IAXXX_SND_SOC_UPDATE_BITS(codec,
			IAXXX_SRB_PDMO_PORT_PWR_EN_ADDR,
			(1 << op_port_mic), 0);
		if (ret < 0)
			return ret;
	}

	/* tristate the pdm clock lines */
	ret = snd_soc_component_write(codec, io_ctrl_clk_reg, io_ctrl_clk_reset);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__,
			io_ctrl_clk_reg, io_ctrl_clk_reset, ret);
		return ret;
	}

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
					IAXXX_BLOCK_0);
	if (ret) {
		pr_err("io_ctrl_clk_reg Update block failed : %d", ret);
		return ret;
	}

	return 0;
}

/* DMIC MIC START */
#define IAXXX_PDM_DMIC_MIC_PUT_GET(mic, mic_in) \
static int iaxxx_port_##mic##_put(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec); \
	u32 clk_src = iaxxx->port_mic_en[mic_in]; \
	if (iaxxx->port_mic_en[mic_in] == ucontrol->value.integer.value[0]) \
		return 0; \
	iaxxx->port_mic_en[mic_in] = ucontrol->value.integer.value[0]; \
	if (iaxxx->port_mic_en[mic_in] < IAXXX_NUM_PDM_CLK_SRC) \
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, \
					mic_in, iaxxx->pdm_mic_port[mic_in]); \
	else \
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, \
				clk_src, mic_in, iaxxx->pdm_mic_port[mic_in]); \
} \
static int iaxxx_port_##mic##_get(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_mic_en[mic_in]; \
	return 0; \
} \
static int iaxxx_pdm_port_##mic##_put( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec); \
	\
	iaxxx->pdm_mic_port[mic_in] = \
		ucontrol->value.integer.value[0]; \
	return 0; \
} \
static int iaxxx_pdm_port_##mic##_get( \
	struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = \
		iaxxx->pdm_mic_port[mic_in]; \
	return 0; \
}

IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_in0, PDM_DMIC_IN0)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_in1, PDM_DMIC_IN1)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_in2, PDM_DMIC_IN2)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_in3, PDM_DMIC_IN3)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_mono_in0, PDM_DMIC_MONO_IN0)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_mono_in1, PDM_DMIC_MONO_IN1)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_mono_in2, PDM_DMIC_MONO_IN2)
IAXXX_PDM_DMIC_MIC_PUT_GET(dmic_mono_in3, PDM_DMIC_MONO_IN3)
/* DMIC MIC END */

static int iaxxx_pdm_port_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	u32 period = 0, div_val = 0, nr_val = 0, fs_sync_active = 0;
	u32 pdm_bclk = 0, port_sample_rate = 0, port_bits_per_frame = 0;
	u32 word_len = 0, words_per_frm = 0;
	u32 clk_ctrl_val = 0;
	u32 status = 0;
	u32 apll_src;
	u32 cnr0_i2s_enable_val;
	int ret;

	if (port > NUM_I2S_GEN) {
		dev_err(codec->dev,
			"%s: PDM port stop failed Invalid port number %d\n",
			__func__, port);
		return -EINVAL;
	}

	if (iaxxx->port_filter[port] == ucontrol->value.integer.value[0])
		return 0;

	cnr0_i2s_enable_val = snd_soc_component_read32(codec, IAXXX_CNR0_I2S_ENABLE_ADDR);
	if (cnr0_i2s_enable_val == IAXXX_SOC_READ_INVALID_VAL) {
		dev_err(codec->dev, "%s: reg 0x%x read failed\n",
			__func__, IAXXX_CNR0_I2S_ENABLE_ADDR);
		return -EINVAL;
	}
	pr_info("dev id-%d: cnr0 i2s enable register val 0x%x",
					priv->dev_id, cnr0_i2s_enable_val);

	apll_src = iaxxx->apll_src;

	/*
	 * No need of temp A_CLK switching, if any of the I2S chan is enabled
	 */
	if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src) &&
			(cnr0_i2s_enable_val == 0)) {
		/*
		 * In case of IA8201, HW has a limitation that leaf nodes can't
		 * be configured if root clock is not present. When source of
		 * A_CLK is selected as any of the port clock, can't guarantee
		 * that clock will be present during configuration. Hence,
		 * change APLL clock source to known clock source (CLK_IN)
		 * for the configuration and later switch it back to port clock.
		 */
		ret = iaxxx_config_apll(priv, priv->sys_clk_src,
				iaxxx->apll_clk,
				priv->sys_clk_in_freq);
		if (ret) {
			dev_err(codec->dev,
				"%s: Apll clk src set failed : %d\n",
							__func__, ret);
			return ret;
		}
	}

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << port), (0x1 << port));
	if (ret < 0)
		return ret;

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		dev_err(codec->dev,
			"%s: I2S PORT PWR EN ADDR Update block failed : %d\n",
							__func__, ret);
		return ret;
	}
	/* Configure I2S clock */
	pdm_bclk = iaxxx->pdm_bclk;
	port_sample_rate = pdm_cfg[pdm_bclk].sample_rate;
	words_per_frm = pdm_cfg[pdm_bclk].words_per_frame;
	word_len = pdm_cfg[pdm_bclk].word_length;
	port_bits_per_frame = ((word_len + 1) * (words_per_frm + 1) - 1);

	if (iaxxx_calc_i2s_div(codec, port_bits_per_frame, port_sample_rate,
			       &period, &div_val, &nr_val)) {
		dev_err(codec->dev,
			"%s: Invalid arguments requested\n", __func__);
		return -EINVAL;
	}

	/* Disable I2S generator for programming dividers */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port),
		IAXXX_CNR0_I2S_ENABLE_LOW);
	if (ret < 0)
		return ret;

	/* I2S Trigger - Disable I2S */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, ret);
		return ret;
	}

	/* Disable hl divider */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		      IAXXX_I2S_I2S0_HL_DISABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_HL_ADDR(port),
			IAXXX_I2S_I2S0_HL_DISABLE, ret);
		return ret;
	}
	/* Set HL value */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		div_val | IAXXX_I2S_I2S0_HL_ENABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_HL_ADDR(port),
			div_val | IAXXX_I2S_I2S0_HL_ENABLE, ret);
		return ret;
	}

	/* Disable NR divider */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_DISABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_NR_ADDR(port),
			IAXXX_I2S_I2S0_NR_DISABLE, ret);
		return ret;
	}
	/* Set NR value */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		nr_val | IAXXX_I2S_I2S0_NR_ENABLE);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_NR_ADDR(port),
			nr_val | IAXXX_I2S_I2S0_NR_ENABLE, ret);
		return ret;
	}

	/* For PDM FS is assumed is fixed to 16 */
	fs_sync_active = ((16 << IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS) &
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK) |
		(IAXXX_I2S_GEN_CFG_FS_POL_LOW |
		 IAXXX_I2S_GEN_CFG_CLK_POL_LOW |
		 IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE |
		 IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE |
		 (port_bits_per_frame <<
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));
	/* Configure fsPol, clkPol, etc in config register */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		fs_sync_active);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
			fs_sync_active, ret);
		return ret;
	}

	/*
	 * Configure clock_ctrl register Always set CLK_CTRL_LOW field to half
	 * of the CLK_CTRL_PERIOD to generate 50% duty cycle based I2S Clock.
	 */
	clk_ctrl_val = (((period >> 1) - 1) <<
		IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS) |
		(((period - 1) << IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS)
		 & IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_MASK);

	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_CLK_CTRL_ADDR(port),
			    clk_ctrl_val);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_CLK_CTRL_ADDR(port),
			clk_ctrl_val, ret);
		return ret;
	}

	/* Configure frameSync_align register */
	ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_FS_ALIGN_ADDR(port), 0);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_I2S_I2S_FS_ALIGN_ADDR(port),
			0, ret);
		return ret;
	}


	if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src) &&
			(cnr0_i2s_enable_val == 0)) {
		/* Switch back APLL clock source to port clock */
		ret = iaxxx_config_apll(priv, iaxxx->apll_src,
				  iaxxx->apll_clk,
				  iaxxx->apll_input_freq);
		if (ret) {
			dev_err(codec->dev,
				"%s: Apll clk src set failed : %d\n",
							__func__, ret);
			return ret;
		}
	} else {
		/* Start the I2S clock */
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
					  IAXXX_CNR0_I2S_ENABLE_MASK(port),
					  IAXXX_CNR0_I2S_ENABLE_HIGH << port);
		if (ret < 0)
			return ret;

		/* I2S Trigger - Disable I2S */
		ret = snd_soc_component_write(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				    IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL);
		if (ret < 0) {
			dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
				__func__, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, ret);
			return ret;
		}
	}

	iaxxx->port_filter[port] = 1;

	return 0;
}

#define IAXXX_PDM_PORT_SETUP_SET_GET(port_name, port) \
static int iaxxx_pdm_##port_name##_put(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_pdm_port_setup(kcontrol, ucontrol, port); \
	else \
		return iaxxx_put_port_clk_stop(kcontrol, ucontrol, port); \
} \
static int iaxxx_pdm_##port_name##_get(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_filter[port]; \
	return 0; \
}

IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen0, I2S_GEN0);
IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen1, I2S_GEN1);
IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen2, I2S_GEN2);
IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen3, I2S_GEN3);
IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen4, I2S_GEN4);
IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen5, I2S_GEN5);
IAXXX_PDM_PORT_SETUP_SET_GET(i2s_gen6, I2S_GEN6);

static int iaxxx_i2s_master_src_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol,
			int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 clk_src = ucontrol->value.integer.value[0];
	int ret;

	if (iaxxx->master_src[port] == clk_src)
		return 0;

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_PCM_MX3_ADDR,
		IAXXX_CNR0_PCM_MX3_I2S_PORT0_SEL_MASK <<
		(IAXXX_CNR0_PCM_MX3_I2S_PORT1_SEL_POS * port),
		clk_src << (IAXXX_CNR0_PCM_MX3_I2S_PORT1_SEL_POS * port));
	if (ret < 0)
		return ret;

	iaxxx->master_src[port] = clk_src;
	return 0;
}

#define IAXXX_I2S_PORT_MASTER_SRC_SET_GET(port_name, port) \
static int iaxxx_##port_name##_mstr_src_put(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	return iaxxx_i2s_master_src_setup( \
				kcontrol, ucontrol, port); \
} \
static int iaxxx_##port_name##_mstr_src_get(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->master_src[port]; \
	return 0; \
}

IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen0, I2S_GEN0);
IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen1, I2S_GEN1);
IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen2, I2S_GEN2);
IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen3, I2S_GEN3);
IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen4, I2S_GEN4);
IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen5, I2S_GEN5);
IAXXX_I2S_PORT_MASTER_SRC_SET_GET(i2s_gen6, I2S_GEN6);

#define IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(port_name, port) \
static int iaxxx_pdm_##port_name##_put_drive_strength( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec); \
	uint32_t val; \
	int ret; \
	\
	val = ucontrol->value.integer.value[0]; \
	IAXXX_SND_SOC_UPDATE_BITS(codec, \
		iaxxx_pad_ctrl_pdm_clk_addr[port], \
		IAXXX_PAD_CTRL_PORTA_CLK_STRENGTH_MASK, \
		val << IAXXX_PAD_CTRL_PORTA_CLK_STRENGTH_POS); \
	if (ret < 0) \
		return ret; \
	\
	iaxxx->pdm_clk_drive_strength[port] = \
		ucontrol->value.integer.value[0]; \
	return 0; \
} \
static int iaxxx_pdm_##port_name##_get_drive_strength( \
	struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = \
		iaxxx->pdm_clk_drive_strength[port]; \
	return 0; \
}

IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(porta_clk, PORTA_CLK);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(portb_do, PORTB_DO);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(portc_clk, PORTC_CLK);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(commb_0, COMMB_0);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(porta_do, PORTA_DO);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(portb_clk, PORTB_CLK);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(portc_do, PORTC_DO);
IAXXX_PDM_PORT_CLK_DRIVE_STRENGTH(commb_3, COMMB_3);


static int iaxxx_pdm_head_strm_rx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec);
	u32 value = 0;
	int ret;

	iaxxx->head_of_strm_rx_all = ucontrol->value.integer.value[0];
	if (iaxxx->head_of_strm_rx_all)
		value = iaxxx->head_of_strm_rx_all;

	/* CIC filter config */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_RX_HOS_ADDR,
		IAXXX_CNR0_CIC_RX_HOS_MASK_VAL, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int iaxxx_pdm_head_strm_rx_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->head_of_strm_rx_all;
	return 0;
}

static int iaxxx_pdm_head_strm_tx_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec);
	u32 value = 0;
	int ret;

	iaxxx->head_of_strm_tx_all = ucontrol->value.integer.value[0];
	if (iaxxx->head_of_strm_tx_all)
		value = iaxxx->head_of_strm_tx_all;

	/* CIC filter config */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_CIC_TX_HOS_ADDR,
		IAXXX_CNR0_CIC_TX_HOS_MASK_VAL, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int iaxxx_pdm_head_strm_tx_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->head_of_strm_tx_all;
	return 0;
}

static int iaxxx_configure_pcm_port(struct snd_soc_component *codec, int port,
			bool is_master, u32 frame_len, u32 ch_count,
			u32 word_len, u32 mode, u32 srdd_val)
{
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 status;
	u32 ao_mask;
	u32 ao_val;
	u32 ch_val;
	int ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
			(1 << port), (1 << port));
	if (ret < 0)
		return ret;

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
					&status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(codec->dev,
			"%s: PCM PORT PWR EN ADDR Update block failed : %d\n",
			__func__, ret);
		return ret;
	}

	if (is_master) {
		ao_mask = IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(port) |
			IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(port);
		ao_val = (1 << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(port)) |
			   (1 << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(port));
	} else {
		ao_mask = IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(port);
		ao_val = 1 << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(port);
	}

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PCM_SWLR_ADDR(port),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PCM_SRSA_ADDR(port),
		IAXXX_PCM0_SRSA_WMASK_VAL, ch_count);
	if (ret < 0)
		return ret;

	/* Allow extra cycles for HW to set the value
	 * by doing a read of the same register.
	 */
	ch_val = snd_soc_component_read32(codec, IAXXX_PCM_SRSA_ADDR(port));
	if (ch_val == IAXXX_SOC_READ_INVALID_VAL) {
		dev_err(codec->dev, "%s: reg 0x%x read failed\n",
			__func__, IAXXX_PCM_SRSA_ADDR(port));
		return -EINVAL;
	}

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PCM_STSA_ADDR(port),
		IAXXX_PCM0_STSA_WMASK_VAL, ch_count);
	if (ret < 0)
		return ret;

	/* Allow extra cycles for HW to set the value
	 * by doing a read of the same register.
	 */
	ch_val = snd_soc_component_read32(codec, IAXXX_PCM_STSA_ADDR(port));
	if (ch_val == IAXXX_SOC_READ_INVALID_VAL) {
		dev_err(codec->dev, "%s: reg 0x%x read failed\n",
			__func__, IAXXX_PCM_STSA_ADDR(port));
		return -EINVAL;
	}

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PCM_SFLR_ADDR(port),
		IAXXX_PCM0_SFLR_WMASK_VAL, frame_len);
	if (ret < 0)
		return ret;


	/* Set Port FS, DO reg */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_AO_CLK_CFG_ADDR,
		ao_mask, ao_val);
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PCM_SRDD_ADDR(port),
		IAXXX_PCM0_SRDD_WMASK_VAL, srdd_val);
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_PCM_MC_ADDR(port),
		IAXXX_PCM0_MC_WMASK_VAL, mode);
	if (ret < 0)
		return ret;

	/*
	 * IA8x01 PCM has new interrupt, PCM_ON_ENABLE which will trigger when
	 * very first FSYNC is detected on BCLK. PCM_ON_ENABLE internally used
	 * for PDM-PCM synchronization done at h/w level. Now, all PCM
	 * interrupts (error and PCM_ON) are generated to PCTRL through
	 * AF_PHY_ERR interrupt. Firmware AFIsr() clears entire PCM_RIS register
	 * upon any PHY_ERR which also clears PCM_ON. As it is cleared before
	 * even PDM/PCM sync configuration, sync never work. To avoid this,
	 * RomeControl API should always mask PCM_ON interrupt as by default, it
	 * is enabled. Also, AFIsr() should never clear this bit in PCM_RIS.
	 * PCM_ON will be reset in PCMx_RIS once PCM port is put back in reset.
	 */
	ret = snd_soc_component_write(codec, IAXXX_PCM_ISEN_ADDR(port),
			    IAXXX_PCM_ISEN_WMASK_VAL);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_PCM_ISEN_ADDR(port),
			IAXXX_PCM_ISEN_WMASK_VAL, ret);
		return ret;
	}

	/* Set cnr0 pcm active reg */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(port),
		IAXXX_CNR0_PCM_ENABLE <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(port));
	if (ret < 0)
		return ret;

	return 0;
}

static int iaxxx_setup_pcm_port(struct snd_soc_component *codec, int port,
				    bool is_master)
{
	u32 port_clk_val;
	u32 port_fs_val;
	u32 port_di_val;
	u32 port_do_val;
	int ret;

	if (is_master) {
		/* Format for port master configuration */
		port_clk_val = IAXXX_IO_CTRL_CLK_MASTER;
		port_fs_val = IAXXX_IO_CTRL_FS_MASTER;
	} else {
		/* Format for port slave configuration */
		port_clk_val = IAXXX_IO_CTRL_CLK_SLAVE;
		port_fs_val = IAXXX_IO_CTRL_FS_SLAVE;
	}
	port_di_val = IAXXX_IO_CTRL_DI;
	port_do_val = IAXXX_IO_CTRL_DO;

	IAXXX_SND_SOC_UPDATE_BITS(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK, port_clk_val);
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, port_fs_addr[port],
		 IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK, port_fs_val);
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, port_di_addr[port],
		 IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, port_do_addr[port],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);
	if (ret < 0)
		return ret;

	return 0;
}

static int iaxxx_reset_pcm_port(struct snd_soc_component *codec, int port,
			  bool is_master)
{
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	uint32_t status;
	uint32_t apll_src = iaxxx->apll_src;
	u32 ao_mask;
	int ret;

	ret = snd_soc_component_write(codec, IAXXX_PCM_SRSA_ADDR(port), 0);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_PCM_SRSA_ADDR(port), 0, ret);
		return ret;
	}

	ret = snd_soc_component_write(codec, IAXXX_PCM_STSA_ADDR(port), 0);
	if (ret < 0) {
		dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
			__func__, IAXXX_PCM_STSA_ADDR(port), 0, ret);
		return ret;
	}

	ao_mask = (IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(port) |
		    IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(port) |
		    IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(port));
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_AO_CLK_CFG_ADDR,
						ao_mask, 0);
	if (ret < 0)
		return ret;

	if (is_master) {
		if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src)) {
			/*
			 * In case of IA8201, HW has a limitation that leaf
			 * nodes can't be configured if root clock is not
			 * present. When source of A_CLK is selected as any
			 * of the port clock, can't guarantee that clock will
			 * be present during configuration. Hence, change APLL
			 * clock source to known clock source (CLK_IN) for the
			 * configuration and later switch it back to port clock
			 */
			ret = iaxxx_config_apll(priv, priv->sys_clk_src,
						iaxxx->apll_clk,
						priv->sys_clk_in_freq);
			if (ret) {
				pr_err("dev id-%d: Apll CLK_IN clk src set failed : %d",
				       priv->dev_id, ret);
				return ret;
			}
		}

		/* CNR0_I2S_Enable - Disable I2S */
		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(port),
			IAXXX_CNR0_I2S_ENABLE_LOW << port);
		if (ret < 0)
			return ret;

		/* I2S Trigger - Enable */
		ret = snd_soc_component_write(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL);
		if (ret < 0) {
			dev_err(codec->dev, "%s: reg write 0x%x with val 0x%x failed : %d\n",
				__func__,
				IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, ret);
			return ret;
		}

		IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
			(0x1 << port), (0 << port));
		if (ret < 0)
			return ret;

		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
				&status, IAXXX_BLOCK_0);
		if (ret) {
			dev_err(codec->dev,
			"%s: I2S Port PWR En Addr Update block failed : %d\n",
							__func__, ret);
			return ret;
		}
		if (IAXXX_CHECK_VALID_APLL_PORT_CLK_SRC(apll_src)) {
			/* Switch back APLL clock source to port clock */
			ret = iaxxx_config_apll(priv, iaxxx->apll_src,
						iaxxx->apll_clk,
						iaxxx->apll_input_freq);
			if (ret) {
				pr_err("dev id-%d: APLL Clk src Back to port clk failed : %d",
				       priv->dev_id, ret);
				return ret;
			}
		}
	}
	/* Set cn0 pcm active reg */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(port),
		IAXXX_CNR0_PCM_DISABLE <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(port));
	if (ret < 0)
		return ret;

	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
		(0x1 << port), (0 << port));
	if (ret < 0)
		return ret;

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret)
		dev_err(codec->dev,
			"%s: PCM PORT PWR EN ADDR Update block failed : %d\n",
			__func__, ret);

	return ret;
}

static int iaxxx_pcm_port_stop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	int ret;

	dev_info(codec->dev, "%s: port:%d mstrclk:%d\n",
			__func__, port, iaxxx->is_ip_port_master[port]);

	if (iaxxx->port_pcm_start[port] == ucontrol->value.integer.value[0])
		return 0;

	ret = iaxxx_reset_pcm_port(codec, port, iaxxx->is_ip_port_master[port]);
	if (ret < 0) {
		dev_err(codec->dev, "%s: Reset PCM port failed: %d\n",
				__func__, ret);
		return ret;
	}

	iaxxx->port_pcm_start[port] = 0;
	return 0;
}

static int iaxxx_pcm_port_start(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 word_len = 0;
	u32 frame_len = 0;
	u32 channel_val = 0;
	u32 reg_srdd_val;
	u32 mode;
	bool tristate;
	int count;
	int ret;

	dev_info(codec->dev, "%s: port:%d mstrclk:%d\n",
			__func__, port, iaxxx->is_ip_port_master[port]);

	/*
	 * If port is configured/started before, return.
	 * Non-zero value means port is already configured.
	 */
	if (iaxxx->port_pcm_start[port])
		return 0;

	word_len = iaxxx->pcm_info[port].word_len;
	channel_val = iaxxx->pcm_info[port].no_of_channels;
	tristate = iaxxx->pcm_info[port].tristate;

	frame_len = channel_val;

	for (count = 0; frame_len != 0; count++)
		frame_len &= frame_len - 1;

	frame_len = count - 1;
	dev_info(codec->dev, "%s: word_len:%u channel_val:%u frame_len: %u tristate: %d\n",
			__func__, word_len, channel_val, frame_len, tristate);

	if (iaxxx->pcm_port_fmt[port] == 0) {
		reg_srdd_val = 1;
		if (!tristate)
			mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT;
		else
			mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT |
				IAXXX_PCM_CTRL_TRISTATE_ENABLE;
	} else if (iaxxx->pcm_port_fmt[port] == 1) {
		reg_srdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_TDMFMT;
	} else if (iaxxx->pcm_port_fmt[port] == 2) {
		reg_srdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_DSPFMT;
	} else {
		dev_err(codec->dev, "%s: unsupported format\n", __func__);
		return -EINVAL;
	}

	ret = iaxxx_configure_pcm_port(codec, port,
		iaxxx->is_ip_port_master[port], frame_len, channel_val,
		word_len, mode, reg_srdd_val);
	if (ret < 0) {
		dev_err(codec->dev, "%s: configure PCM port %d failed, %d\n",
			__func__, port, ret);
		return ret;
	}

	if (iaxxx->is_ip_port_master[port]) {
		/* Start I2S clock, if Chip I2S is master */
		ret = iaxxx_set_i2s_controller(codec, false, port);
		if (ret < 0) {
			dev_err(codec->dev, "%s: iaxxx_set_i2s_controller fail, %d\n",
				__func__, ret);
			return ret;
		}
	}

	/* port mode I2S, DSP or TDM mode */
	iaxxx->port_pcm_start[port] = ucontrol->value.integer.value[0];

	return 0;
}

static int iaxxx_pcm_port_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	int ret;

	dev_info(codec->dev, "%s: port:%d mstrclk:%d\n",
			__func__, port, iaxxx->is_ip_port_master[port]);
	if (iaxxx->port_pcm_setup[port] == ucontrol->value.integer.value[0])
		return 0;

	ret = iaxxx_setup_pcm_port(codec, port,
				    iaxxx->is_ip_port_master[port]);
	if (ret < 0) {
		dev_err(codec->dev, "%s: setup PCM port failed, %d\n",
			__func__, ret);
		return ret;
	}

	iaxxx->port_pcm_setup[port] = ucontrol->value.integer.value[0];
	return 0;
}

static bool is_valid_cfg_params(uint32_t word_len, uint32_t data_fmt)
{
	int i;
	bool found = false;

	for (i = 0; i < ARRAY_SIZE(iaxxx_word_len); i++) {
		if (iaxxx_word_len[i] == word_len) {
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("Invalid word length %d", word_len);
		return found;
	}

	found = false;
	for (i = 0; i < ARRAY_SIZE(iaxxx_data_format); i++) {
		if (iaxxx_data_format[i] == data_fmt) {
			found = true;
			break;
		}
	}
	if (!found)
		pr_err("Invalid data format %d", data_fmt);

	return found;
}

static bool is_valid_clk_params(uint32_t sample_rate, uint32_t word_len,
			uint32_t words_per_frame, uint32_t fs_duration)
{
	int i;
	bool found = false;

	for (i = 0; i < ARRAY_SIZE(iaxxx_sample_rate); i++) {
		if (iaxxx_sample_rate[i] == sample_rate) {
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("Invalid sample rate %d", sample_rate);
		return found;
	}

	found = false;
	for (i = 0; i < ARRAY_SIZE(iaxxx_word_len); i++) {
		if (iaxxx_word_len[i] == word_len) {
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("Invalid word length %d", word_len);
		return found;
	}

	found = false;
	for (i = 0; i < ARRAY_SIZE(iaxxx_words_per_frame); i++) {
		if (iaxxx_words_per_frame[i] == words_per_frame) {
			found = true;
			break;
		}
	}
	if (!found) {
		pr_err("Invalid words per frame %d", words_per_frame);
		return found;
	}

	found = false;
	for (i = 0; i < ARRAY_SIZE(iaxxx_fs_duration); i++) {
		if (iaxxx_fs_duration[i] == fs_duration) {
			found = true;
			break;
		}
	}
	if (!found)
		pr_err("Invalid fs duration %d", fs_duration);

	return found;
}

#define IAXXX_PCM_PORT_SET_GET(port_name, port) \
static int iaxxx_pcm_##port_name##_start_put( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_pcm_port_start(kcontrol, ucontrol, port); \
	else \
		return iaxxx_pcm_port_stop(kcontrol, ucontrol, port); \
} \
static int iaxxx_pcm_##port_name##_start_get( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_pcm_start[port]; \
	return 0; \
} \
static int iaxxx_pcm_##port_name##_cfg_put( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	\
	if (!is_valid_cfg_params(ucontrol->value.integer.value[0], \
				 ucontrol->value.integer.value[1])) \
		return -EINVAL; \
	\
	iaxxx->pcm_info[port].word_len = ucontrol->value.integer.value[0]; \
	iaxxx->pcm_info[port].no_of_channels = \
		ucontrol->value.integer.value[1]; \
	iaxxx->pcm_info[port].tristate = ucontrol->value.integer.value[2]; \
	\
	pr_info("Word len %d, no. of channel %d, tristate %d", \
		iaxxx->pcm_info[port].word_len, \
		iaxxx->pcm_info[port].no_of_channels, \
		iaxxx->pcm_info[port].tristate); \
	return 0; \
} \
static int iaxxx_pcm_##port_name##_cfg_get( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->pcm_info[port].word_len; \
	ucontrol->value.integer.value[1] = \
		iaxxx->pcm_info[port].no_of_channels; \
	ucontrol->value.integer.value[2] = iaxxx->pcm_info[port].tristate; \
	return 0; \
} \
static int iaxxx_pcm_##port_name##_setup_put( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec); \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_pcm_port_setup(kcontrol, ucontrol, port); \
	else { \
		iaxxx->port_pcm_setup[port] = \
			ucontrol->value.integer.value[0]; \
		return 0; \
	} \
} \
static int iaxxx_pcm_##port_name##_setup_get( \
		struct snd_kcontrol *kcontrol, \
		struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_pcm_setup[port]; \
	return 0; \
} \
static int iaxxx_pcm_##port_name##_set_master( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	\
	if (!is_valid_clk_params(ucontrol->value.integer.value[0], \
				 ucontrol->value.integer.value[1], \
				 ucontrol->value.integer.value[2], \
				 ucontrol->value.integer.value[3])) \
		return -EINVAL; \
	\
	iaxxx->clk_param_info[port].sample_rate = \
		ucontrol->value.integer.value[0]; \
	iaxxx->clk_param_info[port].word_len = \
		ucontrol->value.integer.value[1]; \
	iaxxx->clk_param_info[port].words_per_frame = \
		ucontrol->value.integer.value[2]; \
	iaxxx->clk_param_info[port].fs_duration = \
		ucontrol->value.integer.value[3]; \
	iaxxx->clk_param_info[port].clk_pol = \
		ucontrol->value.integer.value[4]; \
	iaxxx->clk_param_info[port].fs_pol = \
		ucontrol->value.integer.value[5]; \
	pr_info("Sample Rate %d, word len %d, word per frame %d",\
		iaxxx->clk_param_info[port].sample_rate,\
		iaxxx->clk_param_info[port].word_len,\
		iaxxx->clk_param_info[port].words_per_frame);\
	pr_info("fs_duration %d, clk pol %d fs pol %d",\
		iaxxx->clk_param_info[port].fs_duration,\
		iaxxx->clk_param_info[port].clk_pol,\
		iaxxx->clk_param_info[port].fs_pol);\
	if (iaxxx->clk_param_info[port].sample_rate) \
		iaxxx->is_ip_port_master[port] = 1; \
	else \
		iaxxx->is_ip_port_master[port] = 0; \
	return 0; \
} \
static int iaxxx_pcm_##port_name##_get_master( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	\
	ucontrol->value.integer.value[0] = \
		iaxxx->clk_param_info[port].sample_rate; \
	ucontrol->value.integer.value[1] = \
		iaxxx->clk_param_info[port].word_len; \
	ucontrol->value.integer.value[2] = \
		iaxxx->clk_param_info[port].words_per_frame; \
	ucontrol->value.integer.value[3] = \
		iaxxx->clk_param_info[port].fs_duration; \
	ucontrol->value.integer.value[4] = \
		iaxxx->clk_param_info[port].clk_pol; \
	ucontrol->value.integer.value[5] = \
		iaxxx->clk_param_info[port].fs_pol; \
	return 0; \
} \

IAXXX_PCM_PORT_SET_GET(porta, PCM_PORTA)
IAXXX_PCM_PORT_SET_GET(portb, PCM_PORTB)
IAXXX_PCM_PORT_SET_GET(portc, PCM_PORTC)

static const struct snd_kcontrol_new iaxxx_rx_snd_controls[] = {

	IAXXXCORE_RX_CHMGR_KCTRL(RX_0, "Rx0"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_1, "Rx1"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_2, "Rx2"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_3, "Rx3"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_4, "Rx4"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_5, "Rx5"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_6, "Rx6"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_7, "Rx7"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_8, "Rx8"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_9, "Rx9"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_10, "Rx10"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_11, "Rx11"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_12, "Rx12"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_13, "Rx13"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_14, "Rx14"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_15, "Rx15"),
};

static const struct snd_kcontrol_new iaxxx_tx_snd_controls[] = {
	IAXXXCORE_TX_CHMGR_KCTRL(TX_0, "Tx0"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_1, "Tx1"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_2, "Tx2"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_3, "Tx3"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_4, "Tx4"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_5, "Tx5"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_6, "Tx6"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_7, "Tx7"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_8, "Tx8"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_9, "Tx9"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_10, "Tx10"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_11, "Tx11"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_12, "Tx12"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_13, "Tx13"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_14, "Tx14"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_15, "Tx15"),
};

static const struct snd_kcontrol_new iaxxx_stream_snd_controls[] = {
	IAXXXCORE_STREAM_KCTRL(STREAM0, "strm0"),
	IAXXXCORE_STREAM_KCTRL(STREAM1, "strm1"),
	IAXXXCORE_STREAM_KCTRL(STREAM2, "strm2"),
	IAXXXCORE_STREAM_KCTRL(STREAM3, "strm3"),
	IAXXXCORE_STREAM_KCTRL(STREAM4, "strm4"),
	IAXXXCORE_STREAM_KCTRL(STREAM5, "strm5"),
	IAXXXCORE_STREAM_KCTRL(STREAM6, "strm6"),
	IAXXXCORE_STREAM_KCTRL(STREAM7, "strm7"),
};

static const struct snd_kcontrol_new iaxxx_plugin_ep_snd_controls[] = {
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN0, "Plgin0"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN1, "Plgin1"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN2, "Plgin2"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN3, "Plgin3"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN4, "Plgin4"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN5, "Plgin5"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN6, "Plgin6"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN7, "Plgin7"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN8, "Plgin8"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN9, "Plgin9"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN10, "Plgin10"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN11, "Plgin11"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN12, "Plgin12"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN13, "Plgin13"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN14, "Plgin14"),
};

static const struct snd_kcontrol_new iaxxx_plugin_en_snd_controls[] = {
	IAXXX_PLUGIN_EN_CTLS(PLUGIN0, "Plgin0"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN1, "Plgin1"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN2, "Plgin2"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN3, "Plgin3"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN4, "Plgin4"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN5, "Plgin5"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN6, "Plgin6"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN7, "Plgin7"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN8, "Plgin8"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN9, "Plgin9"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN10, "Plgin10"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN11, "Plgin11"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN12, "Plgin12"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN13, "Plgin13"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN14, "Plgin14"),
};

static const struct snd_kcontrol_new iaxxx_snd_controls[] = {

	SOC_SINGLE_BOOL_EXT("Update Block0 Req", 0,
		iaxxx_get_update_block0, iaxxx_put_update_block0),
	SOC_SINGLE_BOOL_EXT("Update Block1 Req", 0,
		iaxxx_get_update_block1, iaxxx_put_update_block1),

	SOC_SINGLE_BOOL_EXT("Update Plugin Block0 Req", 0,
		iaxxx_get_update_plgblock0, iaxxx_put_update_plgblock0),
	SOC_SINGLE_BOOL_EXT("Update Plugin Block1 Req", 0,
		iaxxx_get_update_plgblock1, iaxxx_put_update_plgblock1),

	SOC_SINGLE_BOOL_EXT("PCM PortA Setup", 0,
		iaxxx_pcm_porta_setup_get, iaxxx_pcm_porta_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortB Setup", 0,
		iaxxx_pcm_portb_setup_get, iaxxx_pcm_portb_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortC Setup", 0,
		iaxxx_pcm_portc_setup_get, iaxxx_pcm_portc_setup_put),

	IAXXX_SOC_SINGLE_MULTI_EXT("PCM PortA Cfg", SND_SOC_NOPM, 0, 0xFF, 0, 3,
		iaxxx_pcm_porta_cfg_get, iaxxx_pcm_porta_cfg_put),
	IAXXX_SOC_SINGLE_MULTI_EXT("PCM PortB Cfg", SND_SOC_NOPM, 0, 0xFF, 0, 3,
		iaxxx_pcm_portb_cfg_get, iaxxx_pcm_portb_cfg_put),
	IAXXX_SOC_SINGLE_MULTI_EXT("PCM PortC Cfg", SND_SOC_NOPM, 0, 0xFF, 0, 3,
		iaxxx_pcm_portc_cfg_get, iaxxx_pcm_portc_cfg_put),

	SOC_SINGLE_BOOL_EXT("PCM PortA Start", 0,
		iaxxx_pcm_porta_start_get, iaxxx_pcm_porta_start_put),
	SOC_SINGLE_BOOL_EXT("PCM PortB Start", 0,
		iaxxx_pcm_portb_start_get, iaxxx_pcm_portb_start_put),
	SOC_SINGLE_BOOL_EXT("PCM PortC Start", 0,
		iaxxx_pcm_portc_start_get, iaxxx_pcm_portc_start_put),

	IAXXX_SOC_SINGLE_MULTI_EXT("PCM PortA Master", SND_SOC_NOPM, 0, 0xFFFFF,
		0, 6, iaxxx_pcm_porta_get_master, iaxxx_pcm_porta_set_master),
	IAXXX_SOC_SINGLE_MULTI_EXT("PCM PortB Master", SND_SOC_NOPM, 0, 0xFFFFF,
		0, 6, iaxxx_pcm_portb_get_master, iaxxx_pcm_portb_set_master),
	IAXXX_SOC_SINGLE_MULTI_EXT("PCM PortC Master", SND_SOC_NOPM, 0, 0xFFFFF,
		0, 6, iaxxx_pcm_portc_get_master, iaxxx_pcm_portc_set_master),

	SOC_ENUM_EXT("PDM BCLK", iaxxx_pdm_bclk_enum,
		       iaxxx_get_pdm_bclk,
		       iaxxx_put_pdm_bclk),

	SOC_ENUM_EXT("PDM Port AudCLK", iaxxx_pdm_aud_port_clk_enum,
		     iaxxx_get_pdm_aud_port_clk,
		     iaxxx_put_pdm_aud_port_clk),

	SOC_ENUM_EXT("Port ApllCLK", iaxxx_apll_clk_enum,
		     iaxxx_get_apll_clk,
		     iaxxx_put_apll_clk),

	SOC_ENUM_EXT("Port Apll Input Freq", iaxxx_apll_input_freq_enum,
		     iaxxx_get_apll_input_freq,
		     iaxxx_put_apll_input_freq),

	SOC_ENUM_EXT("Port Apll Src", iaxxx_apll_src_enum,
		     iaxxx_get_apll_src,
		     iaxxx_put_apll_src),

	SOC_ENUM_EXT("System Input Clock Freq", iaxxx_apll_input_freq_enum,
		     iaxxx_get_system_clk_input_freq,
		     iaxxx_put_system_clk_input_freq),

	SOC_ENUM_EXT("PDM DMIC In0 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_in0_get, iaxxx_port_dmic_in0_put),
	SOC_ENUM_EXT("PDM DMIC In1 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_in1_get, iaxxx_port_dmic_in1_put),
	SOC_ENUM_EXT("PDM DMIC In2 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_in2_get, iaxxx_port_dmic_in2_put),
	SOC_ENUM_EXT("PDM DMIC In3 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_in3_get, iaxxx_port_dmic_in3_put),

	SOC_ENUM_EXT("PDM DMIC MONO In0 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_mono_in0_get, iaxxx_port_dmic_mono_in0_put),
	SOC_ENUM_EXT("PDM DMIC MONO In1 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_mono_in1_get, iaxxx_port_dmic_mono_in1_put),
	SOC_ENUM_EXT("PDM DMIC MONO In2 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_mono_in2_get, iaxxx_port_dmic_mono_in2_put),
	SOC_ENUM_EXT("PDM DMIC MONO In3 En", iaxxx_pdm_mic_en_enum,
		iaxxx_port_dmic_mono_in3_get, iaxxx_port_dmic_mono_in3_put),

	SOC_SINGLE_BOOL_EXT("PDM I2S Gen0 Setup", 0,
		iaxxx_pdm_i2s_gen0_get, iaxxx_pdm_i2s_gen0_put),
	SOC_SINGLE_BOOL_EXT("PDM I2S Gen1 Setup", 0,
		iaxxx_pdm_i2s_gen1_get, iaxxx_pdm_i2s_gen1_put),
	SOC_SINGLE_BOOL_EXT("PDM I2S Gen2 Setup", 0,
		iaxxx_pdm_i2s_gen2_get, iaxxx_pdm_i2s_gen2_put),
	SOC_SINGLE_BOOL_EXT("PDM I2S Gen3 Setup", 0,
		iaxxx_pdm_i2s_gen3_get, iaxxx_pdm_i2s_gen3_put),
	SOC_SINGLE_BOOL_EXT("PDM I2S Gen4 Setup", 0,
		iaxxx_pdm_i2s_gen4_get, iaxxx_pdm_i2s_gen4_put),
	SOC_SINGLE_BOOL_EXT("PDM I2S Gen5 Setup", 0,
		iaxxx_pdm_i2s_gen5_get, iaxxx_pdm_i2s_gen5_put),
	SOC_SINGLE_BOOL_EXT("PDM I2S Gen6 Setup", 0,
		iaxxx_pdm_i2s_gen6_get, iaxxx_pdm_i2s_gen6_put),

	/* Enable DMIC0_CLK paths. For TX ports [9:8] are 0R/0L,
	 * for RX ports [7/6/5/4/3/2/1/0] are 3R/3L/2R/2L/1R/1L/0R/0L
	 * -> L is left channel, R is right channel.
	 */
	SOC_SINGLE_EXT("PDM DMIC0 Clk Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_dmic0, iaxxx_put_start_dmic0),
	SOC_SINGLE_EXT("PDM DMIC1 Clk Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_dmic1, iaxxx_put_start_dmic1),
	SOC_SINGLE_EXT("PDM CDC0 Clk Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_cdc0, iaxxx_put_start_cdc0),
	SOC_SINGLE_EXT("PDM CDC1 Clk Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_cdc1, iaxxx_put_start_cdc1),

	SOC_SINGLE_EXT("PDM Hos Rx", SND_SOC_NOPM, 0, 0xFF, 0,
			iaxxx_pdm_head_strm_rx_get, iaxxx_pdm_head_strm_rx_put),
	SOC_SINGLE_EXT("PDM Hos Tx", SND_SOC_NOPM, 0, 0xFF, 0,
			iaxxx_pdm_head_strm_tx_get, iaxxx_pdm_head_strm_tx_put),

	SOC_ENUM_EXT("Route Status", iaxxx_route_status_enum,
			iaxxx_get_route_status,
			iaxxx_put_route_status),

	SOC_ENUM_EXT("PDM PortA Clk Drive Strength",
			iaxxx_pdm_clk_drive_strength_enum,
			iaxxx_pdm_porta_clk_get_drive_strength,
			iaxxx_pdm_porta_clk_put_drive_strength),
	SOC_ENUM_EXT("PDM PortB Clk Drive Strength",
			iaxxx_pdm_clk_drive_strength_enum,
			iaxxx_pdm_portb_clk_get_drive_strength,
			iaxxx_pdm_portb_clk_put_drive_strength),
	SOC_ENUM_EXT("PDM PortC Clk Drive Strength",
			iaxxx_pdm_clk_drive_strength_enum,
			iaxxx_pdm_portc_clk_get_drive_strength,
			iaxxx_pdm_portc_clk_put_drive_strength),
	SOC_ENUM_EXT("PDM PortA DO Drive Strength",
		     iaxxx_pdm_clk_drive_strength_enum,
		     iaxxx_pdm_porta_do_get_drive_strength,
		     iaxxx_pdm_porta_do_put_drive_strength),
	SOC_ENUM_EXT("PDM PortB DO Drive Strength",
		     iaxxx_pdm_clk_drive_strength_enum,
		     iaxxx_pdm_portb_do_get_drive_strength,
		     iaxxx_pdm_portb_do_put_drive_strength),
	SOC_ENUM_EXT("PDM PortC DO Drive Strength",
		     iaxxx_pdm_clk_drive_strength_enum,
		     iaxxx_pdm_portc_do_get_drive_strength,
		     iaxxx_pdm_portc_do_put_drive_strength),
	SOC_ENUM_EXT("PDM COMMB_0 Drive Strength",
		     iaxxx_pdm_clk_drive_strength_enum,
		     iaxxx_pdm_commb_0_get_drive_strength,
		     iaxxx_pdm_commb_0_put_drive_strength),
	SOC_ENUM_EXT("PDM COMMB_3 Drive Strength",
		     iaxxx_pdm_clk_drive_strength_enum,
		     iaxxx_pdm_commb_3_get_drive_strength,
		     iaxxx_pdm_commb_3_put_drive_strength),

	SOC_ENUM_EXT("PDM I2S Gen0 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen0_mstr_src_get,
		iaxxx_i2s_gen0_mstr_src_put),
	SOC_ENUM_EXT("PDM I2S Gen1 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen1_mstr_src_get,
		iaxxx_i2s_gen1_mstr_src_put),
	SOC_ENUM_EXT("PDM I2S Gen2 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen2_mstr_src_get,
		iaxxx_i2s_gen2_mstr_src_put),
	SOC_ENUM_EXT("PDM I2S Gen3 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen3_mstr_src_get,
		iaxxx_i2s_gen3_mstr_src_put),
	SOC_ENUM_EXT("PDM I2S Gen4 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen4_mstr_src_get,
		iaxxx_i2s_gen4_mstr_src_put),
	SOC_ENUM_EXT("PDM I2S Gen5 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen5_mstr_src_get,
		iaxxx_i2s_gen5_mstr_src_put),
	SOC_ENUM_EXT("PDM I2S Gen6 Master Src", iaxxx_master_src_enum,
		iaxxx_i2s_gen6_mstr_src_get,
		iaxxx_i2s_gen6_mstr_src_put),

	/* Kcontrols for PDM port selection */
	SOC_ENUM_EXT("PDM DMIC In0 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_in0_get, iaxxx_pdm_port_dmic_in0_put),
	SOC_ENUM_EXT("PDM DMIC In1 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_in1_get, iaxxx_pdm_port_dmic_in1_put),
	SOC_ENUM_EXT("PDM DMIC In2 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_in2_get, iaxxx_pdm_port_dmic_in2_put),
	SOC_ENUM_EXT("PDM DMIC In3 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_in3_get, iaxxx_pdm_port_dmic_in3_put),
	SOC_ENUM_EXT("PDM DMIC MONO In0 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_mono_in0_get,
		iaxxx_pdm_port_dmic_mono_in0_put),
	SOC_ENUM_EXT("PDM DMIC MONO In1 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_mono_in1_get,
		iaxxx_pdm_port_dmic_mono_in1_put),
	SOC_ENUM_EXT("PDM DMIC MONO In2 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_mono_in2_get,
		iaxxx_pdm_port_dmic_mono_in2_put),
	SOC_ENUM_EXT("PDM DMIC MONO In3 Port", iaxxx_pdm_port_sel_enum,
		iaxxx_pdm_port_dmic_mono_in3_get,
		iaxxx_pdm_port_dmic_mono_in3_put),

	SOC_ENUM_EXT("Bypass APLL", iaxxx_apll_clk_enum,
		     iaxxx_get_bypass_apll,
		     iaxxx_put_bypass_apll),

	SOC_ENUM_EXT("Power Mode", iaxxx_power_mode_enum,
			iaxxx_get_power_mode,
			iaxxx_put_power_mode),
};

static int iaxxxcore_enable_i2srx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	pr_debug("event 0x%x, port id: 0x%x", event, w->id);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		pr_debug("SND_SOC_DAPM_POST_PMU event");
		break;

	case SND_SOC_DAPM_POST_PMD:
		pr_debug("SND_SOC_DAPM_POST_PMD event");
		break;

	default:
		pr_err("Unknown event 0x%x", event);
		break;
	}

	return ret;
}

static int iaxxxcore_enable_i2stx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		pr_debug("SND_SOC_DAPM_POST_PMU event");
		break;

	case SND_SOC_DAPM_POST_PMD:
		/* Disable PCM ports post streaming */
		pr_debug("SND_SOC_DAPM_POST_PMD event");
		break;

	default:
		pr_debug("Unknown event 0x%x", event);
		break;
	}

	return ret;
}

static int iaxxx_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_component *codec = dai->component;
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(dai->component);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	int gain;
	u32 status = 0;
	int active = 0, ch_mask = 0xFFFF;
	u32 pcm_op_gn_mask = iaxxx->op_channels_active << 16;
	u32 pcm_op_gn_en_val = iaxxx->op_channels_active << 16;
	u32 addr;
	int ret;

	dev_info(codec->dev, "%s: (dai id:%d): mute: %d op_gn_en:%u\n",
				__func__, mute, dai->id, pcm_op_gn_en_val);

	/* set gain to -60db when mute is called
	 * set gain to 0db when unmute is called
	 */
	if (mute)
		gain = -60; /*0xBC*/
	else
		gain = 0;

	/* Update TX CHANNEL GAIN EN HDR REG */
	IAXXX_SND_SOC_UPDATE_BITS(codec, IAXXX_CH_HDR_CH_GAIN_ADDR,
				  pcm_op_gn_mask, pcm_op_gn_en_val);
	if (ret < 0)
		return ret;

	while (ch_mask & 1 << active) {
		if (iaxxx->op_channels_active & 1 << active) {
			/* Update the Gain ramp rate for TX Channel REG */
			addr = IAXXX_GET_GRP_ADDR(priv,
					IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR,
					(active + TX_0));
			if (!addr)
				return -EINVAL;

			ret = snd_soc_component_write(codec, addr, (STEP_INST <<
				 IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS) |
				gain);
			if (ret < 0) {
				dev_err(codec->dev, "%s: reg write 0x%x with gain 0x%x failed : %d\n",
					__func__, addr, gain, ret);
				return ret;
			}
		}
		active++;
	}

	/* Update Block to set gain settings */
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		dev_err(codec->dev,
		"%s: OUT CH GRP CH GAIN CTRL ADDR Update block failed : %d\n",
							__func__, ret);
		return ret;
	}

	return 0;
}

static int iaxxx_pcm_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(dai->component);
	struct snd_soc_component *codec = dai->component;
	int id = dai->id;
	int ret;

	dev_info(codec->dev, "%s: enter with dai id:%d, fmt 0x%x\n",
				__func__, id, fmt);

	if (id >= IAXXX_NUM_CODEC_DAIS) {
		dev_err(codec->dev, "%s: Unsupported dai id:%d\n",
							__func__, id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is Master , chip is slave */
		pr_info("CPU is Master , chip is slave");
		iaxxx->is_codec_master[id] = 0;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CPU is slave , chip is Master */
		pr_info("CPU is slave , chip is Master");
		iaxxx->is_codec_master[id] = 1;
		break;
	default:
		pr_err("Unsupported fmt 0x%x", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iaxxx->pcm_dai_fmt[id] = SND_SOC_DAIFMT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iaxxx->pcm_dai_fmt[id] = SND_SOC_DAIFMT_DSP_A;
		break;
	default:
		iaxxx->pcm_dai_fmt[id] = 0;
		pr_err("Unsupported fmt 0x%x, using Default settings", fmt);
	}

	ret = iaxxx_setup_pcm_port(codec, id, iaxxx->is_codec_master[id]);
	if (ret < 0) {
		dev_err(codec->dev, "%s: setup PCM port failed, %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}


static int iaxxx_set_i2s_cfg(struct snd_soc_dai *dai, bool is_pseudo, int id)
{
	struct snd_soc_component *codec = dai->component;
	int ret;

	if (is_pseudo) {
		dev_err(codec->dev, "%s: Pseudo mode not supported\n",
								__func__);
		return -EINVAL;
	}

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		dev_err(codec->dev, "%s: Unsupported dai id:%d\n",
								__func__, id);
		return -EINVAL;
	}

	ret = iaxxx_set_i2s_controller(codec, is_pseudo, id);
	if (ret < 0) {
		dev_err(codec->dev, "%s: setup i2s controller fail, %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int iaxxx_pcm_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec);
	u32 bit_width, word_len = 0;
	u32 channel_port_rx = 0;
	u32 channel_port_tx = 0;
	u32 frame_length;
	u32 reg_srdd_val;
	u32 mode;
	int id = dai->id;
	int ret;

	if (id >= IAXXX_NUM_CODEC_DAIS) {
		dev_err(codec->dev, "%s: Unsupported dai id:%d\n",
							__func__, id);
		return -EINVAL;
	}

	dev_info(dai->dev, "%s: (dai id:%d): format = 0x%x, rate = %d param width %d\n",
			__func__, id,
			params_format(params),
			params_rate(params),
			params_width(params));

	if (iaxxx->is_stream_in_use[id][IAXXX_PLAYBACK_ID] ||
		    iaxxx->is_stream_in_use[id][IAXXX_CAPTURE_ID]) {
		dev_info(codec->dev, "PCM Port %d is already configured\n", id);
		return 0;
	}

	bit_width = iaxxx->pcm_port_word_len[id] ?
		iaxxx->pcm_port_word_len[id] : params_width(params);

	switch (bit_width) {
	case 16:
		word_len = 15;
		break;
	case 20:
		word_len = 19;
		break;
	case 24:
		word_len = 23;
		break;
	case 32:
		word_len = 31;
		break;
	default:
		dev_err(codec->dev, "%s: Unsupported word length %d\n",
							__func__, bit_width);
		return -EINVAL;
	}

#ifdef CONFIG_IAXXX_SND_SOC_ADAU1772
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
				iaxxx->pcm_port_dout_hiz_on_playback[id]) {
		if (adau1772_swap_i2s_do_di_pins(1) < 0)
			return -EINVAL;
	}
#endif

	dev_info(codec->dev, "%s: selected bit_width :%d, word_len: %d\n",
		__func__, bit_width, word_len);

	switch (params_channels(params)) {
	case 8:
		channel_port_rx = 0xFF;
		channel_port_tx = 0xFF; /* 8 channels */
		frame_length = 7; /*words per frame - 1 */
		break;
	case 6:
		channel_port_rx = 0x3F;
		channel_port_tx = 0x3F;
		frame_length = 5; /*words per frame - 1 */
		break;
	case 4:
		channel_port_rx = 0x0F;
		channel_port_tx = 0x0F;
		frame_length = 3; /*words per frame - 1 */
		break;
	case 3:
		channel_port_rx = 0x07;
		channel_port_tx = 0x07;
		frame_length = 2; /*words per frame - 1 */
		break;
	case 2:
		channel_port_rx = 0x03;
		channel_port_tx = 0x03;
		frame_length = 1; /*words per frame - 1 */
		break;
	case 1:
		channel_port_rx = 0x01;
		channel_port_tx = 0x01;
		frame_length = 0; /*words per frame - 1 */
		break;
	default:
		dev_err(codec->dev, "%s: Unsupported channels :%d\n",
				__func__, params_channels(params));
		return -EINVAL;
	}
	dev_info(codec->dev, "supported channels :%d rx: %d tx: %d\n",
			params_channels(params), channel_port_rx,
			channel_port_tx);

	switch (iaxxx->pcm_dai_fmt[id]) {
	case SND_SOC_DAIFMT_I2S:
		reg_srdd_val = 1;
		mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		reg_srdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_DSPFMT;
		break;
	default:
		reg_srdd_val = 0;
		mode = 0;
		dev_info(codec->dev, "default settings\n");
	}

#ifdef CONFIG_IAXXX_SND_SOC_ADAU1772
	/*
	 * Below change is required to make PortX DO line in
	 * High-Z mode. It is required by IA8x01 AEC usecase
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (iaxxx->pcm_port_dout_hiz_on_playback[id]) {
			switch (id) {
			case IAXXX_AIF0:
				IAXXX_SND_SOC_UPDATE_BITS(codec,
					IAXXX_PAD_CTRL_PORTA_DO_ADDR,
					IAXXX_PAD_CTRL_PORTA_DO_DDST_MASK, 0);
				if (ret < 0)
					return ret;

				IAXXX_SND_SOC_UPDATE_BITS(codec,
				port_do_addr[id],
				IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK,
				IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_RESET_VAL);
				if (ret < 0)
					return ret;
				break;
			case IAXXX_AIF1:
				IAXXX_SND_SOC_UPDATE_BITS(codec,
					IAXXX_PAD_CTRL_PORTB_DO_ADDR,
					IAXXX_PAD_CTRL_PORTB_DO_DDST_MASK, 0);
				if (ret < 0)
					return ret;

				IAXXX_SND_SOC_UPDATE_BITS(codec,
				port_do_addr[id],
				IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_MASK,
				IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_RESET_VAL);
				if (ret < 0)
					return ret;
				break;
			case IAXXX_AIF2:
				IAXXX_SND_SOC_UPDATE_BITS(codec,
					IAXXX_PAD_CTRL_PORTC_DO_ADDR,
					IAXXX_PAD_CTRL_PORTC_DO_DDST_MASK, 0);
				if (ret < 0)
					return ret;

				IAXXX_SND_SOC_UPDATE_BITS(codec,
				port_do_addr[id],
				IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_MASK,
				IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_RESET_VAL);
				if (ret < 0)
					return ret;
				break;
			default:
				dev_err(codec->dev, "%s: Unsupported id :%d\n",
						__func__, id);
				return -EINVAL;
			}
		}
	}
#endif

	ret = iaxxx_configure_pcm_port(codec, id, iaxxx->is_codec_master[id],
		frame_length, channel_port_tx, word_len, mode, reg_srdd_val);
	if (ret < 0) {
		dev_err(codec->dev, "%s: configure PCM port %d failed, %d\n",
			__func__, id, ret);
		return ret;
	}

	if (iaxxx->is_codec_master[id]) {
		iaxxx->clk_param_info[id].sample_rate = params_rate(params);
		iaxxx->clk_param_info[id].word_len = word_len;
		iaxxx->clk_param_info[id].words_per_frame = frame_length;
		iaxxx->clk_param_info[id].fs_duration = word_len + 1;
		ret = iaxxx_set_i2s_cfg(dai, false, id);
		if (ret < 0) {
			dev_err(codec->dev, "%s: setup i2s configuration failed : %d\n",
			__func__, ret);
			return ret;
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		iaxxx->is_stream_in_use[id][IAXXX_PLAYBACK_ID] = true;
	else
		iaxxx->is_stream_in_use[id][IAXXX_CAPTURE_ID] = true;

	return 0;
}

static int iaxxx_pcm_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(codec);
	unsigned int dai_fmt = 0;
	unsigned int mstr_fmt = iaxxx->is_codec_master[dai->id];
	int ret;
	int id = dai->id;

	dev_info(codec->dev, "%s: enter for dai id %d\n", __func__, id);
	if (id >= IAXXX_NUM_CODEC_DAIS) {
		dev_err(codec->dev, "%s: Unsupported dai id:%d\n",
							__func__, id);
		return -EINVAL;
	}

	if (iaxxx->is_stream_in_use[id][IAXXX_PLAYBACK_ID] ||
		    iaxxx->is_stream_in_use[id][IAXXX_CAPTURE_ID]) {
		dev_info(codec->dev, "%s: PCM Port %d is already configured\n",
								__func__, id);
		return 0;
	}

	mstr_fmt = mstr_fmt ? SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS;
	if (iaxxx->pcm_dai_fmt[id] == SND_SOC_DAIFMT_I2S) {
		dai_fmt = SND_SOC_DAIFMT_I2S | mstr_fmt;
	} else if (iaxxx->pcm_dai_fmt[id] == SND_SOC_DAIFMT_DSP_A) {
		dai_fmt = SND_SOC_DAIFMT_DSP_A | mstr_fmt;
	} else if (iaxxx->pcm_port_fmt[id]) {
		dai_fmt = mstr_fmt;
	} else {
		dev_err(codec->dev,
			"%s: DAI FMT configuration is missing\n", __func__);
		return -EINVAL;
	}

	ret = iaxxx_pcm_set_fmt(dai, dai_fmt);
	if (ret < 0) {
		dev_err(codec->dev,
			"%s: PCM dai set fmt failed : %d\n", __func__, ret);
		return ret;
	}

	ret = iaxxx_digital_mute(dai, false);
	if (ret < 0)
		dev_err(codec->dev,
			"%s: tx channel unmute failed : %d\n", __func__, ret);

	return 0;
}

static void iaxxx_port_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret;
	struct snd_soc_component *codec = dai->component;

	dev_info(codec->dev, "%s: enter, dai id:%d\n", __func__, dai->id);

	ret = iaxxx_digital_mute(dai, true);
	if (ret < 0)
		pr_err("tx channel mute failed : %d", ret);
}

static int iaxxx_pcm_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;
	struct iaxxx_codec_priv *iaxxx = snd_soc_component_get_drvdata(dai->component);
	int id = dai->id;
	int ret;

	dev_info(codec->dev, "%s: enter for dai id:%d\n", __func__, id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#ifdef CONFIG_IAXXX_SND_SOC_ADAU1772
		if (iaxxx->pcm_port_dout_hiz_on_playback[id])
			if (adau1772_swap_i2s_do_di_pins(0) < 0)
				return -EINVAL;
#endif
		iaxxx->is_stream_in_use[id][IAXXX_PLAYBACK_ID] = false;
	} else {
		iaxxx->is_stream_in_use[id][IAXXX_CAPTURE_ID] = false;
	}

	if (iaxxx->is_stream_in_use[id][IAXXX_PLAYBACK_ID] ||
	    iaxxx->is_stream_in_use[id][IAXXX_CAPTURE_ID]) {
		dev_info(codec->dev,
			"%s: one of the stream is still active:%d\n",
				__func__, substream->stream);
		return 0;
	}

	ret = iaxxx_reset_pcm_port(codec, id, iaxxx->is_codec_master[id]);
	if (ret < 0) {
		dev_err(codec->dev, "%s: Reset PCM port %d failed: %d\n",
				__func__, id, ret);
		return ret;
	}

	return 0;
}

#define IAXXX_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S24_3LE|\
			SNDRV_PCM_FMTBIT_S32_LE)

#define IAXXX_PCM_RATES SNDRV_PCM_RATE_8000_48000

static const struct snd_soc_dai_ops iaxxx_pcm_ops = {
	.set_fmt = iaxxx_pcm_set_fmt,
	.hw_params = iaxxx_pcm_hw_params,
	.startup = iaxxx_pcm_startup,
	.shutdown = iaxxx_port_shutdown,
	.hw_free = iaxxx_pcm_hw_free,
};

static struct snd_soc_dai_driver iaxxx_dai[] = {
	{
		.name = "iaxxx-pcm0",
		.id = IAXXX_AIF0,
		.playback = {
			.stream_name = "I2S PCM0 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM0 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm1",
		.id = IAXXX_AIF1,
		.playback = {
			.stream_name = "I2S PCM1 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM1 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm2",
		.id = IAXXX_AIF2,
		.playback = {
			.stream_name = "I2S PCM2 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM2 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
};

static int iaxxx_tristate_pdm_pins(struct iaxxx_priv *priv)
{
	int i = 0, rc;

	/* Tristate the PDM clock lines */
	for (i = 0; i < IAXXX_MAX_PDM_PORTS; i++) {

		rc = regmap_update_bits(priv->regmap,
			iaxxx_pad_ctrl_pdm_clk_addr[i],
			IAXXX_PAD_CTRL_PORTC_CLK_DDST_MASK,
			0x00);
		if (rc) {
			pr_err("dev id-%d: write reg 0x%08x failed : %d",
			priv->dev_id, iaxxx_pad_ctrl_pdm_clk_addr[i], rc);
			goto err_out;
		}

		rc = regmap_update_bits(priv->regmap,
			iaxxx_io_ctrl_pdm_clk_addr[i],
			IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_MASK,
			IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_RESET_VAL);
		if (rc) {
			pr_err("dev id-%d: write reg 0x%08x failed : %d",
			priv->dev_id, iaxxx_io_ctrl_pdm_clk_addr[i], rc);
			goto err_out;
		}
	}

err_out:
	return rc;
}

static void iaxxx_reset_codec_params(struct iaxxx_codec_priv *iaxxx)
{
	int i = 0;

	dev_info(iaxxx->dev, "%s: enter\n", __func__);

	for (i = 0; i < IAXXX_MAX_PDM_PORTS; i++)
		iaxxx->pdm_clk_drive_strength[i] = 1;

	for (i = 0; i < NUM_I2S_GEN; i++) {
		iaxxx->port_filter[i] = 0;
		iaxxx->master_src[i] = i;
	}

	for (i = 0; i < IAXXX_NUM_PDM_CLK_SRC; i++)
		iaxxx->port_start_en[i] = 0;

	for (i = 0; i < IAXXX_MAX_PORTS; i++) {
		memset(&iaxxx->pcm_info[i], 0, sizeof(struct pcm_cfg));
		memset(&iaxxx->clk_param_info[i], 0,
		       sizeof(struct i2s_clk_params));
		iaxxx->port_pcm_start[i] = 0;
		iaxxx->port_pcm_setup[i] = 0;
	}

	for (i = 0; i < 32; i++) {
		iaxxx->plugin_blk_en[i] = 0;
		iaxxx->stream_en[i] = 0;
	}
	for (i = 0; i < PDM_NUM_IO_MICS; i++) {
		iaxxx->port_mic_en[i] = IAXXX_NUM_PDM_CLK_SRC;
		iaxxx->pdm_mic_port[i] = 0;
	}

	iaxxx->pdm_bclk = 0;
	iaxxx->apll_clk = 0;
	iaxxx->apll_src = 0;
	iaxxx->apll_input_freq = 0;
	iaxxx->pdm_aud_port_clk = 0;
	iaxxx->head_of_strm_rx_all = 0;
	iaxxx->head_of_strm_tx_all = 0;
	iaxxx->cdc_dmic_enable = 0;

	iaxxx->i2s_master_clk = 0;

	return;

}

static int iaxxx_add_widgets(struct snd_soc_component *codec)
{
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);
	uint32_t num_of_ch;
	uint32_t num_of_plg;
	uint32_t num_of_strm;

	if (!priv)
		return -EINVAL;

	/* Collect the Max instances supported info from RBDT info */
	num_of_ch = priv->rbdt_info[IAXXX_BLOCK_CHANNEL].num_of_grp;
	num_of_plg = priv->rbdt_info[IAXXX_BLOCK_PLUGIN].num_of_grp;
	num_of_strm = priv->rbdt_info[IAXXX_BLOCK_STREAM].num_of_grp;

	/*
	 * Check if no. of instances supported is greater than the
	 * inst defined in arrays. If it is greater, then update
	 * the max inst supported as per the instances defined in arrays.
	 * So that array out of bound issue will be taken care.
	 */
	if (num_of_ch > MAX_RX) {
		pr_warn("No. of channel instances supported (%d) differ from max instances from array (%d)",
			num_of_ch, MAX_RX);
		num_of_ch = MAX_RX;
	}

	if (num_of_plg > MAX_PLUGIN) {
		pr_warn("No. of plugin instances supported (%d) differ from max instances from array (%d)",
			num_of_plg, MAX_PLUGIN);
		num_of_plg = MAX_PLUGIN;
	}

	if (num_of_strm > MAX_STREAM) {
		pr_warn("No. of stream instances supported (%d) differ from max instances from array (%d)",
			num_of_strm, MAX_STREAM);
		num_of_strm = MAX_STREAM;
	}

	/* Add codec controls */

	/*
	 * ARRAY_SIZE / MAX_ARB_instance gives the number of controls defined
	 * for each instance
	 */
	snd_soc_add_component_controls(codec, iaxxx_rx_snd_controls,
		num_of_ch * (ARRAY_SIZE(iaxxx_rx_snd_controls) / MAX_RX));

	snd_soc_add_component_controls(codec, iaxxx_tx_snd_controls,
		num_of_ch * (ARRAY_SIZE(iaxxx_tx_snd_controls) / MAX_TX));

	snd_soc_add_component_controls(codec, iaxxx_plugin_ep_snd_controls,
		num_of_plg * (ARRAY_SIZE(iaxxx_plugin_ep_snd_controls) /
		MAX_PLUGIN));

	snd_soc_add_component_controls(codec, iaxxx_plugin_en_snd_controls,
		num_of_plg * (ARRAY_SIZE(iaxxx_plugin_en_snd_controls) /
		MAX_PLUGIN));

	snd_soc_add_component_controls(codec, iaxxx_snd_controls,
		ARRAY_SIZE(iaxxx_snd_controls));

	snd_soc_add_component_controls(codec, iaxxx_stream_snd_controls,
		num_of_strm * (ARRAY_SIZE(iaxxx_stream_snd_controls) /
		MAX_STREAM));

	return 0;
}

static void iaxxx_init_codec_regmap(struct snd_soc_component *codec)
{
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	pr_info("dev id-%d: called", priv->dev_id);
	iaxxx->regmap = priv->regmap;
	snd_soc_component_init_regmap(codec, iaxxx->regmap);
	iaxxx_add_widgets(codec);
}

static void iaxxx_exit_codec_regmap(struct snd_soc_component *codec)
{
	pr_info("called");
	codec->regmap = NULL;
}

static int iaxxx_codec_probe(struct snd_soc_component *codec)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0))
	int ret;

	codec->control_data = iaxxx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);
	if (ret) {
		dev_err(codec->dev, "%s: unable to set the cache io %d\n",
								__func__, ret);
		return ret;
	}
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	iaxxx->codec = codec;

	dev_info(codec->dev, "%s: enter\n", __func__);

	snd_soc_component_init_regmap(codec, iaxxx->regmap);

	iaxxx_add_widgets(codec);

	dev_info(codec->dev, "%s: exit\n", __func__);
	return 0;
}

static void iaxxx_codec_remove(struct snd_soc_component *codec)
{
	dev_dbg(codec->dev, "%s: enter\n", __func__);
	snd_soc_component_exit_regmap(codec);
}

static struct snd_soc_component_driver soc_codec_iaxxx = {
	.probe = iaxxx_codec_probe,
	.remove = iaxxx_codec_remove,
/*	.suspend = iaxxx_codec_suspend,	*/
/*	.resume = iaxxx_codec_resume,	*/
/*	.set_bias_level = iaxxx_codec_set_bias_level,	*/
};

static const struct of_device_id iaxxx_platform_dt_match[] = {
	{.compatible = "adnc,iaxxx-codec"},
	{}
};

static int iaxxx_codec_notify(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct iaxxx_codec_priv *iaxxx =
		container_of(nb, struct iaxxx_codec_priv, nb_core);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	switch (action) {
	case IAXXX_EV_RELOAD_DONE:
		iaxxx_init_codec_regmap(iaxxx->codec);
	case IAXXX_EV_STARTUP:
	case IAXXX_EV_RECOVERY:
		iaxxx_tristate_pdm_pins(priv);
		iaxxx_reset_codec_params(iaxxx);
		break;
	case IAXXX_EV_RELOAD_START:
		iaxxx_exit_codec_regmap(iaxxx->codec);
	}

	return 0;
}

static int iaxxx_codec_driver_probe(struct platform_device *pdev)
{
	struct iaxxx_codec_priv *iaxxx;
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv;
	int ret = 0;
	int count = 0;
	struct device_node *np = NULL;
	unsigned long get_codec_reg_order;

	dev_info(dev, "%s: enter, dev->id %d, pdev->id %d\n", __func__,
							dev->id, pdev->id);

	if (pdev->dev.of_node) {
		dev_err(dev, "%s: DT Node exists for iaxxx-codec %s\n",
				__func__, pdev->dev.of_node->name);
	} else {
		/* In MFD probe, the devices do not have DT node, so scan the
		 * DT to see if our device node is present and if so use it.
		 */
		for_each_child_of_node(dev->parent->of_node, np) {
			dev_dbg(dev, "%s: Found child-node %s\n",
							__func__, np->name);
			if (of_device_is_compatible(np,
				iaxxx_platform_dt_match[0].compatible)) {
				dev_dbg(dev, "%s: Child-node is compatible %s\n",
							__func__, np->name);
				pdev->dev.of_node = np;
			}
		}
	}

	/* MFD core will provide the regmap instance */
	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		dev_err(dev,
			"%s: MFD parent device data not found yet. Deferred.\n",
			__func__);
		return -EPROBE_DEFER;
	}

	mutex_lock(&codec_reg_order_lock);

	/*
	 * get the current codec registration bitmap status.
	 * ffzb() returns bit position of the first zero bit from LSB to MSB
	 */
	get_codec_reg_order = find_first_zero_bit(codec_reg_order,
							MAX_IAXXX_DEVICES);

	dev_dbg(dev, "%s: probe order bitmap %ld\n",
						__func__, get_codec_reg_order);

	/* make sure the current order is less than the max allowed */
	if (get_codec_reg_order < MAX_IAXXX_DEVICES) {
		if (priv->dev_id) {
			/*
			 * requested dev-id should match next codec order.
			 * if not in order, differ the probe
			 */
			if (priv->dev_id != get_codec_reg_order) {
				pr_err(
				"dev (%d) probe not in order. probe deffered",
				priv->dev_id);
				mutex_unlock(&codec_reg_order_lock);
				return -EPROBE_DEFER;
			}
		}
	} else {
		/* should never reach here  but to be extra cautious */
		pr_err("Err. probe order value (%ld) more than allowed (%d)",
					get_codec_reg_order, MAX_IAXXX_DEVICES);
		mutex_unlock(&codec_reg_order_lock);
		return -ENODEV;
	}

	/* update the bitmap to indicate new codec registration */
	set_bit(get_codec_reg_order, codec_reg_order);
	mutex_unlock(&codec_reg_order_lock);

	iaxxx = devm_kzalloc(&pdev->dev, sizeof(*iaxxx), GFP_KERNEL);
	if (!iaxxx)
		return -ENOMEM;

	iaxxx->regmap = priv->regmap;
	iaxxx->dev = dev;
	iaxxx->dev_parent = dev->parent;
	iaxxx->plugin_param[0].param_id = IAXXX_MAX_VAL;
	iaxxx->plugin_param[1].param_id = IAXXX_MAX_VAL;
	iaxxx->cdc_dmic_enable = 0;
	platform_set_drvdata(pdev, iaxxx);

	ret = of_property_read_u32_array(priv->dev->of_node,
			"adnc,pcm-port-word-len",
			iaxxx->pcm_port_word_len, IAXXX_MAX_PORTS);
	if (ret) {
		dev_warn(dev,
			"%s: pcm-port-word-len dt entry missing. Assuming defaults\n",
			__func__);

		for (count = 0; count < IAXXX_MAX_PORTS; count++)
			iaxxx->pcm_port_word_len[count] = 0;
	}

	ret = of_property_read_u32_array(priv->dev->of_node,
			"adnc,pcm-port-dout-hiz-on-playback",
			iaxxx->pcm_port_dout_hiz_on_playback, IAXXX_MAX_PORTS);
	if (ret) {
		dev_warn(dev, "%s: pcm-port-dout-hiz state dt entry missing.",
								__func__);
		dev_warn(dev, "Assuming defaults\n");

		for (count = 0; count < IAXXX_MAX_PORTS; count++)
			iaxxx->pcm_port_dout_hiz_on_playback[count] = 0;
	}

	iaxxx->nb_core.notifier_call = iaxxx_codec_notify;
	iaxxx_fw_notifier_register(priv->dev, &iaxxx->nb_core);
	ret = snd_soc_register_component(iaxxx->dev, &soc_codec_iaxxx,
				     iaxxx_dai, ARRAY_SIZE(iaxxx_dai));
	if (ret)
		dev_err(iaxxx->dev,
			"%s: codec registration failed : %d\n", __func__, ret);

	return ret;
}

static int iaxxx_codec_driver_remove(struct platform_device *pdev)
{
	struct iaxxx_codec_priv *iaxxx = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	int dev_id;

	snd_soc_unregister_component(&pdev->dev);

	if (!priv || !iaxxx) {
		dev_err(dev, "%s() either iaxxx private or codec private pointer is NULL\n",
			__func__);
		return -EINVAL;
	}

	dev_id = priv->dev_id;

	mutex_lock(&codec_reg_order_lock);
	/* make sure the device id is set in the codec order bitmap */
	if (!test_bit(dev_id, codec_reg_order)) {
		pr_err("dev-id (%d) not assigned. check log", dev_id);
		mutex_unlock(&codec_reg_order_lock);
		return -ENODEV;
	}

	/* update the bitmap to indicate codec removal */
	clear_bit(dev_id, codec_reg_order);
	mutex_unlock(&codec_reg_order_lock);

	iaxxx_fw_notifier_unregister(priv->dev, &iaxxx->nb_core);

	return 0;
}

static struct platform_driver iaxxx_codec_driver = {
	.probe  = iaxxx_codec_driver_probe,
	.remove = iaxxx_codec_driver_remove,
	.driver = {
		.name = "iaxxx-codec",
		.of_match_table = iaxxx_platform_dt_match,
	},
};

module_platform_driver(iaxxx_codec_driver);

/* Module information */
MODULE_DESCRIPTION("Knowles IAXXX ALSA SoC CODEC Driver");
MODULE_LICENSE("GPL v2");
