/*
 * iaxxx-alsa-tnl.h  --  IAXXX IAxxx ALSA Dai/Device node based Tunneling header
 *
 * Copyright 2020 Knowles, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

struct kt_pcm_priv {
	struct ia_tunneling_hal *tnl_hdl;

	void *pcm_buf;
	unsigned int pcm_buf_size;
	unsigned int pcm_avail_size;
	unsigned int pcm_read_offset;

	void *unparsed_buf;
	unsigned int unparsed_buf_size;
	unsigned int unparsed_avail_size;

	unsigned int lastseqno;
	unsigned int framerepeat;
	unsigned int framedrop;
};

struct alsa_tnl_cfg_priv {
	uint32_t tunlSrc;
	uint32_t tunlMode;
	uint32_t tunlEncode;

	uint32_t tunlSetup;
	uint32_t tunnel_buff_size;
	void *client_data;
} __attribute__((__packed__));

struct iaxxx_alsa_tnl_priv {
	struct device *dev;
	struct device *dev_parent;

	struct delayed_work copy_work;
	/* mutex to lock the alsa data copy/init*/
	struct mutex data_lock;
	struct snd_pcm_substream *substream;

	struct alsa_tnl_cfg_priv alsa_tnl_cfg;

	struct notifier_block crash_notifier;
	unsigned long tunnel_state;

	struct kt_pcm_priv *kt_pcm;

	size_t dma_offset;
};
