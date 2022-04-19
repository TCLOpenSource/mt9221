/*
 * iaxxx-alsa-tnl.c -- IAxxx ALSA Dai/Device node based Tunneling driver
 *
 * Copyright 2020 Knowles Corporation
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
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include "iaxxx-alsa-tnl.h"

/* Use a bitmap to maintain the order of codecs registration */
static DECLARE_BITMAP(codec_reg_order, MAX_IAXXX_DEVICES);
static DEFINE_MUTEX(codec_reg_order_lock);

/***********************************************
Tunnel Read & Parser code
****************** *****************************/
#define MAX_TUNNELS	(32)
#define BUF_SIZE	(8192)

#define IAXXX_TFLG_FW_CRASH		0

struct raf_format_type {
	uint16_t frameSizeInBytes;	/* Frame length in bytes */
	uint8_t encoding;		/* Encoding */
	uint8_t sampleRate;		/* Sample rate */
};

struct raf_frame_type {
	uint64_t timeStamp;	/* Timestamp of the frame */
	uint32_t seqNo;		/* Optional sequence number of the frame */
	struct raf_format_type format;	/* Format information for the frame */
	uint32_t data[0];	/* Start of the variable size payload.
				 It must start at 128 bit aligned
				 address for all the frames */
};

static int parse_tunnel_buf(struct kt_pcm_priv *kt_pcm_hdl)
{
	/* The magic number is ROME in ASCII reversed.
	   So we are looking for EMOR in the byte stream */
	const unsigned char magic_num[4] = {0x45, 0x4D, 0x4F, 0x52};
	unsigned short int tunnel_id;
	bool valid_frame = true;
	unsigned char *buf_itr = kt_pcm_hdl->unparsed_buf;
	/* Minimum bytes required is magic number + tunnel id + reserved
	   and crc + raf struct */
	int min_bytes_req = 4 + 2 + 6 + sizeof(struct raf_frame_type);
	int bytes_avail = kt_pcm_hdl->unparsed_avail_size;
	unsigned char *pcm_buf_itr = NULL;
	int curr_pcm_frame_size;
	bool is_q15_conversion_required = false;
	struct raf_frame_type rft;

	if (!buf_itr) {
		pr_err("Invalid input sent to parse_tunnel_buf");
		return 0;
	}

	do {
		/* Check for MagicNumber 0x454D4F52 */
		while (buf_itr[0] != magic_num[0] ||
				buf_itr[1] != magic_num[1] ||
				buf_itr[2] != magic_num[2] ||
				buf_itr[3] != magic_num[3]) {
			buf_itr++;
			bytes_avail--;
			if (bytes_avail <= 0) {
				pr_err("Could not find the magic no, retry");
				pr_err("buf_itr[0] %x buf_itr[1] %x buf_itr[2] %x buf_itr[3] %x",
				buf_itr[0], buf_itr[1], buf_itr[2], buf_itr[3]);
				return 0;
			}
		}

		/* Skip the magic number */
		buf_itr += 4;
		bytes_avail -= 4;

		/* Read the tunnelID */
		tunnel_id = ((unsigned char)(buf_itr[0]) |
					(unsigned char)(buf_itr[1]) << 8);

		/* Skip tunnelID */
		buf_itr += 2;
		bytes_avail -= 2;

		/* Skip Reserved field and CRC - 6 bytes in total */
		buf_itr += 6;
		bytes_avail -= 6;

		valid_frame = true;

		/* There is only one tunnel data we are looking */
		if (tunnel_id > MAX_TUNNELS) {
			pr_err("Invalid tunnel id %d", tunnel_id);
			valid_frame = false;
		}

		memcpy(&rft, buf_itr, sizeof(struct raf_frame_type));

		/* 1 indicates that it is afloat encoding and */
		/* F indicates it is in q15 encoding */
		if (rft.format.encoding == 1) {
			is_q15_conversion_required = true;
			curr_pcm_frame_size = rft.format.frameSizeInBytes / 2;
		} else {
			is_q15_conversion_required = false;
			curr_pcm_frame_size = rft.format.frameSizeInBytes;
		}

		/* Skip the raf_frame_type */
		buf_itr += sizeof(struct raf_frame_type);
		bytes_avail -= sizeof(struct raf_frame_type);

		if (bytes_avail < rft.format.frameSizeInBytes) {
			pr_debug("Incomplete frame received bytes_avail %d framesize %d",
				bytes_avail, rft.format.frameSizeInBytes);
			bytes_avail += min_bytes_req;
			break;
		}

		if (valid_frame == true) {
			if ((kt_pcm_hdl->pcm_avail_size + curr_pcm_frame_size) <
						kt_pcm_hdl->pcm_buf_size) {
				pcm_buf_itr =
					(unsigned char *)kt_pcm_hdl->pcm_buf +
						kt_pcm_hdl->pcm_avail_size;
				memcpy(pcm_buf_itr, buf_itr,
						rft.format.frameSizeInBytes);
			kt_pcm_hdl->pcm_avail_size += curr_pcm_frame_size;
			} else {
				pr_debug("Not enough PCM buff available. exit");
				bytes_avail += min_bytes_req;
				break;
			}
		}

		if (kt_pcm_hdl->lastseqno == 0) {
			kt_pcm_hdl->lastseqno = rft.seqNo;
		} else {
			int diff = 0;

			if (rft.seqNo > (kt_pcm_hdl->lastseqno + 1)) {
				/*  current frame no is bigger than 1 frame */
				diff = (rft.seqNo - kt_pcm_hdl->lastseqno - 1);
				pr_err("Frame drop. prev seq # %u cur seq # %u",
					kt_pcm_hdl->lastseqno, rft.seqNo);
				kt_pcm_hdl->framedrop += diff;
			} else if (rft.seqNo <= kt_pcm_hdl->lastseqno) {
				diff = (int)((int)kt_pcm_hdl->lastseqno -
							(int)rft.seqNo) + 1;
				pr_err("Frame repeat at seq # %u for %d frames",
							rft.seqNo, diff);
				kt_pcm_hdl->framerepeat += diff;
			}
				kt_pcm_hdl->lastseqno = rft.seqNo;
		}

		/* Skip the data */
		buf_itr += rft.format.frameSizeInBytes;
		bytes_avail -= rft.format.frameSizeInBytes;
	} while (bytes_avail > min_bytes_req);

	return bytes_avail;
}

int kt_pcm_read(struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl, void *client_data,
						void *buffer, u32 bytes)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);
	struct kt_pcm_priv *kt_pcm_hdl = iaxxx_alsa_tnl->kt_pcm;
	int err;
	int bytes_read, bytes_rem;

	if (!kt_pcm_hdl) {
		pr_err("dev id-%d: Invalid handle", priv->dev_id);
		err = 0;
		goto exit;
	}

	if (bytes > kt_pcm_hdl->pcm_avail_size) {
		/* We don't have enough PCM data, read more from the device.
		   First copy the remainder of the PCM buffer to the front
		 of the PCM buffer */
		if (0 != kt_pcm_hdl->pcm_avail_size) {
			pr_debug("dev id-%d: Copying to front of buff pcm_avail_size %u pcm_read_offset %u",
					priv->dev_id,
					kt_pcm_hdl->pcm_avail_size,
					kt_pcm_hdl->pcm_read_offset);
			memcpy(kt_pcm_hdl->pcm_buf,
				((unsigned char *)kt_pcm_hdl->pcm_buf +
						kt_pcm_hdl->pcm_read_offset),
					kt_pcm_hdl->pcm_avail_size);
		}

		/* Always read from start of PCM buffer at this point of time */
		kt_pcm_hdl->pcm_read_offset = 0;

read_again:
		/* Read data from the kernel, account for the leftover
		   data from previous run */
		bytes_read = _tunneling_read_to_alsa_tnl_bfr(priv, client_data,
				(void *)((unsigned char *)
				kt_pcm_hdl->unparsed_buf +
				kt_pcm_hdl->unparsed_avail_size),
				BUF_SIZE);
		if (bytes_read <= 0) {
			pr_err("dev id-%d: Failed to read data from tunnel %d",
						priv->dev_id, bytes_read);
			/* TODO should we try to read a couple of times? */
			err = 0;
			goto exit;
		}

		/* Parse the data to get PCM data */
		kt_pcm_hdl->unparsed_avail_size += bytes_read;
		bytes_rem = parse_tunnel_buf(kt_pcm_hdl);

		/*  Copy left over unparsed data to the front of the buffer */
		if (bytes_rem != 0) {
			int offset = kt_pcm_hdl->unparsed_avail_size -
								bytes_rem;

			memcpy(kt_pcm_hdl->unparsed_buf,
			((unsigned char *)kt_pcm_hdl->unparsed_buf + offset),
			bytes_rem);
		}
		kt_pcm_hdl->unparsed_avail_size = bytes_rem;

		/* If stripping is enabled then we didn't read anything to the
		    pcm buffer. so read again or if we still don't have enough
		    bytes then read data again. */
		if (kt_pcm_hdl->pcm_avail_size == 0 ||
			kt_pcm_hdl->pcm_avail_size < bytes) {
			goto read_again;
		}
	}

	/*  Copy the PCM data to output buffer and return */
	memcpy(buffer, ((unsigned char *)kt_pcm_hdl->pcm_buf +
				kt_pcm_hdl->pcm_read_offset), bytes);

	kt_pcm_hdl->pcm_avail_size -= bytes;
	kt_pcm_hdl->pcm_read_offset += bytes;

	err = bytes;
exit:
	return err;
}

int kt_pcm_open(struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl)
{
	struct kt_pcm_priv *kt_pcm_hdl = NULL;
	int err = 0;

	if (!iaxxx_alsa_tnl) {
		pr_err("Invalid handle");
		err = -EINVAL;
		goto exit;
	}

	kt_pcm_hdl = iaxxx_kvmalloc(
			sizeof(*kt_pcm_hdl), __GFP_ZERO);
	if (NULL == kt_pcm_hdl) {
		err = -ENOMEM;
		goto exit_on_error;
	}

	iaxxx_alsa_tnl->kt_pcm = kt_pcm_hdl;

	kt_pcm_hdl->unparsed_buf_size = BUF_SIZE * 2;
	kt_pcm_hdl->unparsed_avail_size = 0;
	kt_pcm_hdl->unparsed_buf = iaxxx_kvmalloc(
				kt_pcm_hdl->unparsed_buf_size, __GFP_ZERO);
	if (!kt_pcm_hdl->unparsed_buf) {
		err = -ENOMEM;
		goto exit_on_error;
	}

	kt_pcm_hdl->pcm_buf_size = BUF_SIZE * 2;
	kt_pcm_hdl->pcm_avail_size = 0;
	kt_pcm_hdl->pcm_read_offset = 0;
	kt_pcm_hdl->pcm_buf = iaxxx_kvmalloc(
					kt_pcm_hdl->pcm_buf_size, __GFP_ZERO);
	if (!kt_pcm_hdl->pcm_buf) {
		err = -ENOMEM;
		goto exit_on_error;
	}

	kt_pcm_hdl->lastseqno = 0;
	kt_pcm_hdl->framerepeat = 0;
	kt_pcm_hdl->framedrop = 0;

	return 0;

exit_on_error:
	if (kt_pcm_hdl->pcm_buf)
		kvfree(kt_pcm_hdl->pcm_buf);

	if (kt_pcm_hdl->unparsed_buf)
		kvfree(kt_pcm_hdl->unparsed_buf);

	if (kt_pcm_hdl)
		kvfree(kt_pcm_hdl);

exit:
	return err;
}

int kt_pcm_close(struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl)
{
	int err = 0;
	struct kt_pcm_priv *kt_pcm_hdl = iaxxx_alsa_tnl->kt_pcm;

	if (!kt_pcm_hdl) {
		pr_err("Invalid handle");
		err = -1;
		goto exit;
	}

	if (kt_pcm_hdl->pcm_buf)
		kvfree(kt_pcm_hdl->pcm_buf);

	if (kt_pcm_hdl->unparsed_buf)
		kvfree(kt_pcm_hdl->unparsed_buf);

	pr_info("Total Frames repeated %u", kt_pcm_hdl->framerepeat);
	pr_info("Total Frames dropped %u", kt_pcm_hdl->framedrop);

	if (kt_pcm_hdl)
		kvfree(kt_pcm_hdl);

	iaxxx_alsa_tnl->kt_pcm = NULL;

exit:
	return err;
}

/*
 * Capture PCM hardware definition
 */
static const struct snd_pcm_hardware iaxxx_alsa_tnl_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_MMAP_VALID |
					SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= 256,
	.period_bytes_max	= PAGE_SIZE * 4,
	.periods_min		= 4,
	.periods_max		= (PAGE_SIZE * 4) / 16,
	.buffer_bytes_max	= 128 * 1024,
	.channels_min		= 1,
	.channels_max		= 8,
};

#define READ_BUFF_SIZE	(8192)	/* 8k */
#define MAX_READ_RETRY	5

static void iaxxx_alsa_tnl_copy_work(struct work_struct *work)
{
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
		container_of(work, struct iaxxx_alsa_tnl_priv, copy_work.work);
	struct snd_pcm_runtime *runtime;
	size_t period_bytes;
	void *client;
	int read_bytes = 0, retries = 0;
	u32 bytes_to_read;

	mutex_lock(&iaxxx_alsa_tnl->data_lock);
	if (!iaxxx_alsa_tnl->substream) {
		dev_err(iaxxx_alsa_tnl->dev, "No pcm substream\n");
		goto done;
	}

	runtime = iaxxx_alsa_tnl->substream->runtime;
	period_bytes = snd_pcm_lib_period_bytes(iaxxx_alsa_tnl->substream);
	if (!period_bytes) {
		schedule_delayed_work(&iaxxx_alsa_tnl->copy_work,
							msecs_to_jiffies(10));
		goto done;
	}

	client = iaxxx_alsa_tnl->alsa_tnl_cfg.client_data;

	while (period_bytes > 0) {

		if (test_bit(IAXXX_TFLG_FW_CRASH,
			&iaxxx_alsa_tnl->tunnel_state))
			goto done;

		bytes_to_read = period_bytes < (READ_BUFF_SIZE / 2) ?
					period_bytes : (READ_BUFF_SIZE / 2);

		read_bytes = kt_pcm_read(iaxxx_alsa_tnl, client,
					(void *)(runtime->dma_area +
					iaxxx_alsa_tnl->dma_offset),
					bytes_to_read);

		if (read_bytes < 0) {
			pr_err("bulk read failed : %d", read_bytes);
			goto done;
		}

		if (read_bytes > 0) {
			period_bytes -= read_bytes;
			iaxxx_alsa_tnl->dma_offset += read_bytes;

			if (iaxxx_alsa_tnl->dma_offset >= runtime->dma_bytes)
				iaxxx_alsa_tnl->dma_offset = 0;
		} else {
			/*
			 * if we receive 0 bytes, retry with delay +
			 * retry count check
			 */
			if (++retries < MAX_READ_RETRY) {
				pr_err("received 0 data. retry count %d",
								retries);
				usleep_range(15000, 20000);
				continue;
			} else {
				pr_err("bulk read failed with retry also");
				goto done;
			}
		}
	}

	snd_pcm_period_elapsed(iaxxx_alsa_tnl->substream);
	schedule_delayed_work(&iaxxx_alsa_tnl->copy_work, msecs_to_jiffies(10));

done:
	mutex_unlock(&iaxxx_alsa_tnl->data_lock);
}

static int iaxxx_alsa_tnl_setup(struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl,
				uint32_t src, uint32_t mode, uint32_t encode)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);
	void *client;
	int rc = 0;

	pr_info("dev id-%d: ALSA Tunnel Tunneling Setup called", priv->dev_id);

	rc = _tunneling_open_stub(priv, (void **)&client);
	if (rc)
		return rc;

	iaxxx_alsa_tnl->alsa_tnl_cfg.client_data = (void *)client;

	rc = tunnel_setup_an_endpoint_stub(client, src, mode, encode);

	return rc;
}

static int iaxxx_alsa_tnl_terminate(struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl,
				uint32_t src, uint32_t mode, uint32_t encode)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);
	void *client;
	int rc = 0;

	client = iaxxx_alsa_tnl->alsa_tnl_cfg.client_data;

	rc = tunnel_term_an_endpoint_stub(client, src, mode, encode);
	if (rc)
		return rc;

	rc = _tunnel_release_stub(priv, client);
	if (rc)
		return rc;

	iaxxx_alsa_tnl->alsa_tnl_cfg.client_data = NULL;
	return rc;
}

static int iaxxx_alsa_tnl_notifier_cb(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl = container_of(nb,
			struct iaxxx_alsa_tnl_priv, crash_notifier);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);
	uint32_t tunlSetup;
	int ret = 0;
	void *client;

	pr_info("dev id-%d: called\n", priv->dev_id);

	switch (action) {

	case IAXXX_EV_RECOVERY:
		clear_bit(IAXXX_TFLG_FW_CRASH,
				&iaxxx_alsa_tnl->tunnel_state);
		break;

	case IAXXX_EV_CRASH:
		set_bit(IAXXX_TFLG_FW_CRASH, &iaxxx_alsa_tnl->tunnel_state);
		cancel_delayed_work_sync(&iaxxx_alsa_tnl->copy_work);
		tunlSetup = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSetup;

		if (tunlSetup) {
			client = iaxxx_alsa_tnl->alsa_tnl_cfg.client_data;
			ret = _tunnel_release_stub(priv, client);
			if (ret)
				return ret;
			iaxxx_alsa_tnl->alsa_tnl_cfg.client_data = NULL;

			iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSetup = 0;
			iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size = 0;
			ret = kt_pcm_close(iaxxx_alsa_tnl);
			if (ret) {
				dev_err(iaxxx_alsa_tnl->dev,
					"%s: pcm close fail %d\n",
					__func__, ret);
			}
		}
		break;
	}
	return 0;
}

/* PCM for streaming audio from the ALSA Tunnel buffer */
static int iaxxx_alsa_tnl_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
					dev_get_drvdata(rtd->dev);
	struct iaxxx_priv *priv;
	uint32_t src, mode, encode;
	uint32_t tunlSetup, tunl_buf_siz;
	int ret = 0;

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);

	snd_soc_set_runtime_hwparams(substream, &iaxxx_alsa_tnl_pcm_hardware);

	src = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSrc;
	mode = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlMode;
	encode = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlEncode;
	tunlSetup = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSetup;
	tunl_buf_siz = iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size;

	if (!tunlSetup) {
		ret = iaxxx_alsa_tnl_setup(iaxxx_alsa_tnl, src, mode, encode);
		if (ret) {
			dev_err(iaxxx_alsa_tnl->dev,
			"%s: alsa tunnel setup fail %d\n", __func__, ret);
		} else {
			iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSetup = 1;
			if (tunl_buf_siz) {
				ret = iaxxx_set_tunnel_output_buffer_size(priv,
								tunl_buf_siz);
				if (ret) {
					pr_err("dev id-%d: Set tnl buffer size fail %d",
							priv->dev_id, ret);
					return ret;
				}
			}
			ret = kt_pcm_open(iaxxx_alsa_tnl);
			if (ret) {
				dev_err(iaxxx_alsa_tnl->dev,
				"%s: pcm open fail %d\n", __func__, ret);
			}
		}
	}

	return ret;
}

static int iaxxx_alsa_tnl_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
			dev_get_drvdata(rtd->dev);
	uint32_t src, mode, encode;
	uint32_t tunlSetup;
	int ret = 0;

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	src = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSrc;
	mode = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlMode;
	encode = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlEncode;
	tunlSetup = iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSetup;

	if (tunlSetup) {
		ret = iaxxx_alsa_tnl_terminate(iaxxx_alsa_tnl, src, mode,
									encode);
		if (ret) {
			dev_err(iaxxx_alsa_tnl->dev,
			"%s: alsa tunnel terminate fail %d\n", __func__, ret);
		} else {
			iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSetup = 0;
			iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size = 0;
			ret = kt_pcm_close(iaxxx_alsa_tnl);
			if (ret) {
				dev_err(iaxxx_alsa_tnl->dev,
				"%s: pcm close fail %d\n", __func__, ret);
			}
		}
	}

	return ret;
}

static int iaxxx_alsa_tnl_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
			dev_get_drvdata(rtd->dev);
	int ret;

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	dev_info(iaxxx_alsa_tnl->dev,
		"%s: format = 0x%x, rate = %d param width %d, channel %d\n",
		__func__,
		params_format(hw_params),
		params_rate(hw_params),
		params_width(hw_params),
		params_channels(hw_params));

	mutex_lock(&iaxxx_alsa_tnl->data_lock);
	ret = snd_pcm_lib_alloc_vmalloc_buffer(substream,
			params_buffer_bytes(hw_params));
	iaxxx_alsa_tnl->substream = substream;
	mutex_unlock(&iaxxx_alsa_tnl->data_lock);

	return ret;
}

static int iaxxx_alsa_tnl_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
			dev_get_drvdata(rtd->dev);

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	mutex_lock(&iaxxx_alsa_tnl->data_lock);
	iaxxx_alsa_tnl->substream = NULL;
	mutex_unlock(&iaxxx_alsa_tnl->data_lock);

	cancel_delayed_work_sync(&iaxxx_alsa_tnl->copy_work);

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int iaxxx_alsa_tnl_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
			dev_get_drvdata(rtd->dev);

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	iaxxx_alsa_tnl->dma_offset = 0;

	return 0;
}

static int iaxxx_alsa_tnl_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
			dev_get_drvdata(rtd->dev);

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	if (cmd == SNDRV_PCM_TRIGGER_START)
		schedule_delayed_work(&iaxxx_alsa_tnl->copy_work, 0);

	return 0;
}

static snd_pcm_uframes_t iaxxx_alsa_tnl_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
		dev_get_drvdata(rtd->dev);
	snd_pcm_uframes_t frames;

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}

	frames = bytes_to_frames(runtime, iaxxx_alsa_tnl->dma_offset);
	if (frames >= runtime->buffer_size) {
		frames -= runtime->buffer_size;
		dev_warn(iaxxx_alsa_tnl->dev, "buffer capture limited!\n");
	}

	return frames;
}

static const struct snd_pcm_ops iaxxx_alsa_tnl_pcm_ops = {
	.open		= iaxxx_alsa_tnl_pcm_open,
	.close		= iaxxx_alsa_tnl_pcm_close,
	.hw_params	= iaxxx_alsa_tnl_hw_params,
	.hw_free	= iaxxx_alsa_tnl_hw_free,
	.trigger	= iaxxx_alsa_tnl_trigger,
	.prepare	= iaxxx_alsa_tnl_prepare,
	.pointer	= iaxxx_alsa_tnl_pcm_pointer,
	// .mmap		= snd_pcm_lib_mmap_vmalloc,
	// .page		= snd_pcm_lib_get_vmalloc_page,
};

static int iaxxx_alsa_tnl_pcm_probe(struct snd_soc_component *platform)
{
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
					snd_soc_component_get_drvdata(platform);
	struct iaxxx_priv *priv = NULL;
	int ret;

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return -ENODEV;
	}
	priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);

	mutex_init(&iaxxx_alsa_tnl->data_lock);
	INIT_DELAYED_WORK(&iaxxx_alsa_tnl->copy_work, iaxxx_alsa_tnl_copy_work);

	/* register the fw crash handler */
	iaxxx_alsa_tnl->crash_notifier.notifier_call =
		iaxxx_alsa_tnl_notifier_cb;
	ret = iaxxx_fw_notifier_register
		(priv->dev, &iaxxx_alsa_tnl->crash_notifier);
	if (ret) {
		dev_err(iaxxx_alsa_tnl->dev,
			"%s: register for fw notifier failed : %d\n",
			__func__, ret);
		goto error_crash_notifier;
	}

error_crash_notifier:
	return ret;

}

static void iaxxx_alsa_tnl_pcm_remove(struct snd_soc_component *platform)
{
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
					snd_soc_component_get_drvdata(platform);
	struct iaxxx_priv *priv = NULL;

	if (!iaxxx_alsa_tnl) {
		pr_err("Unable to fetch alsa tunnel private data");
		return;
	}
	priv = to_iaxxx_priv(iaxxx_alsa_tnl->dev_parent);

	/* unregister the fw crash notifier */
	iaxxx_fw_notifier_unregister(priv->dev,
				&iaxxx_alsa_tnl->crash_notifier);
}

static const struct snd_soc_component_driver iaxxx_alsa_tnl_platform = {
	.probe = iaxxx_alsa_tnl_pcm_probe,
	.remove = iaxxx_alsa_tnl_pcm_remove,
	.ops = &iaxxx_alsa_tnl_pcm_ops,
};

static struct snd_soc_dai_driver iaxxx_alsa_tnl_dai = {
	.name = "iaxxx-tunl",
	.id = 0,
	.capture = {
		.stream_name = "ALSA Tunnel Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

/***********************************************************************
IAxxx ALSA Tunnel Kcontrols (macros, supporting functions)
************************************************************************/

static int iaxxx_alsa_tnl_snd_soc_info_multi_ext(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo);

// struct soc_multi_mixer_control {
// 	int min, max, platform_max, count;
// 	unsigned int reg, rreg, shift, rshift, invert;
// };

#define IAXXX_SOC_SINGLE_MULTI_EXT(xname, xreg, xshift, xmax, xinvert, xcount,\
	xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = iaxxx_alsa_tnl_snd_soc_info_multi_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)\
		{.reg = xreg, .shift = xshift, .rshift = xshift, .max = xmax, \
		.count = xcount, .platform_max = xmax, .invert = xinvert} \
}

/**
 * iaxxx_alsa_tnl_snd_soc_info_multi_ext - external single mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a single external mixer control.
 * that accepts multiple input.
 *
 * Returns 0 for success.
 */
static int iaxxx_alsa_tnl_snd_soc_info_multi_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_multi_mixer_control *mc =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
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

static int iaxxx_alsa_tnl_cfg_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
						dev_get_drvdata(codec->dev);

	if (!is_valid_tnl_cfg_params(ucontrol->value.integer.value[0],
					ucontrol->value.integer.value[2],
					ucontrol->value.integer.value[1]))
		return -EINVAL;

	iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSrc =
					ucontrol->value.integer.value[0];
	iaxxx_alsa_tnl->alsa_tnl_cfg.tunlMode =
					ucontrol->value.integer.value[1];
	iaxxx_alsa_tnl->alsa_tnl_cfg.tunlEncode =
					ucontrol->value.integer.value[2];

	pr_info("tunl Src 0x%0x, Mode %d, Encode %d",
		iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSrc,
		iaxxx_alsa_tnl->alsa_tnl_cfg.tunlMode,
		iaxxx_alsa_tnl->alsa_tnl_cfg.tunlEncode);

	return 0;
}

static int iaxxx_alsa_tnl_cfg_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
						dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] =
					iaxxx_alsa_tnl->alsa_tnl_cfg.tunlSrc;
	ucontrol->value.integer.value[1] =
					iaxxx_alsa_tnl->alsa_tnl_cfg.tunlMode;
	ucontrol->value.integer.value[2] =
					iaxxx_alsa_tnl->alsa_tnl_cfg.tunlEncode;
	return 0;
}

static int iaxxx_alsa_tnl_buf_siz_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
						dev_get_drvdata(codec->dev);

	iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size =
					ucontrol->value.integer.value[0];
	pr_info("tunnel_buff_size = %d (0x%08x)",
				iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size,
				iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size);
	return 0;
}
static int iaxxx_alsa_tnl_buf_siz_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl =
						dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] =
				iaxxx_alsa_tnl->alsa_tnl_cfg.tunnel_buff_size;
	return 0;
}

static const struct snd_kcontrol_new iaxxx_alsa_tnl_snd_controls[] = {
	IAXXX_SOC_SINGLE_MULTI_EXT("ALSA Tunnel Cfg", SND_SOC_NOPM, 0, 0xFFFF,
			0, 3,
			iaxxx_alsa_tnl_cfg_get, iaxxx_alsa_tnl_cfg_put),
	SOC_SINGLE_EXT("ALSA Tunnel Buffer Size", SND_SOC_NOPM, 0, 0xFFFF, 0,
			iaxxx_alsa_tnl_buf_siz_get, iaxxx_alsa_tnl_buf_siz_put),
};

static int iaxxx_alsa_tnl_codec_probe(struct snd_soc_component *codec)
{
	return snd_soc_add_component_controls(codec, iaxxx_alsa_tnl_snd_controls,
				ARRAY_SIZE(iaxxx_alsa_tnl_snd_controls));
}

static struct snd_soc_component_driver iaxxx_alsa_tnl_dai_codec = {
	.probe = iaxxx_alsa_tnl_codec_probe,
};

static const struct of_device_id iaxxx_alsa_tnl_of_match[];

static int iaxxx_alsa_tnl_probe(struct platform_device *pdev)
{
	struct iaxxx_alsa_tnl_priv *iaxxx_alsa_tnl;
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv;
	int ret;
	struct device_node *np = NULL;
	unsigned long get_codec_reg_order;

	if (pdev->dev.of_node) {
		dev_info(dev, "%s: DT Node exists for iaxxx-alsa-tnl %s\n",
				__func__, pdev->dev.of_node->name);
	} else {
		/* In MFD probe, the devices do not have DT node, so scan the
		 * DT to see if our device node is present and if so use it.
		 */
		for_each_child_of_node(dev->parent->of_node, np) {
			dev_dbg(dev, "Found child-node %s\n", np->name);
			if (of_device_is_compatible(np,
				iaxxx_alsa_tnl_of_match[0].compatible)) {
				dev_dbg(dev, "Child-node is compatible %s\n",
					np->name);
				pdev->dev.of_node = np;
			}
		}
	}

	/* MFD core will provide the regmap instance */
	priv = to_iaxxx_priv(dev->parent);
	if (!priv) {
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

	iaxxx_alsa_tnl = devm_kzalloc(&pdev->dev, sizeof(*iaxxx_alsa_tnl),
								GFP_KERNEL);
	if (!iaxxx_alsa_tnl)
		return -ENOMEM;

	iaxxx_alsa_tnl->dev = dev;
	iaxxx_alsa_tnl->dev_parent = dev->parent;

	platform_set_drvdata(pdev, iaxxx_alsa_tnl);

	ret = devm_snd_soc_register_component(iaxxx_alsa_tnl->dev,
						&iaxxx_alsa_tnl_platform, NULL, 0);
	if (ret < 0) {
		dev_err(iaxxx_alsa_tnl->dev, "Failed to register platform.\n");
		return ret;
	}

	ret = devm_snd_soc_register_component(iaxxx_alsa_tnl->dev,
			&iaxxx_alsa_tnl_dai_codec, &iaxxx_alsa_tnl_dai, 1);
	if (ret < 0)
		dev_err(iaxxx_alsa_tnl->dev, "Failed to register codec.\n");

	return ret;
}

static int iaxxx_alsa_tnl_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	int dev_id = priv->dev_id;

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

	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id iaxxx_alsa_tnl_of_match[] = {
	{.compatible = "adnc,iaxxx-alsa-tnl"},
	{},
};
MODULE_DEVICE_TABLE(of, iaxxx_alsa_tnl_of_match);

static struct platform_driver iaxxx_alsa_tnl_driver = {
	.driver = {
		.name = "iaxxx-alsa-tnl",
		.of_match_table = of_match_ptr(iaxxx_alsa_tnl_of_match),
	},
	.probe = iaxxx_alsa_tnl_probe,
	.remove = iaxxx_alsa_tnl_remove,
};

module_platform_driver(iaxxx_alsa_tnl_driver);

/* Module information */
MODULE_DESCRIPTION("Knowles IAXXX ALSA based Tunneling Driver");
MODULE_LICENSE("GPL v2");
