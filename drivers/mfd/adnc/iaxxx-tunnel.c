/*
 * iaxxx-tunnel.c -- iaxxx tunneling Service
 *
 * Copyright 2017 Knowles Corporation
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
#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/mfd/adnc/iaxxx-tunnel-intf.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-map.h>
#include <linux/mfd/adnc/iaxxx-regmap.h>

#include "iaxxx.h"
#include "iaxxx-tunnel-priv.h"
#define ia_profiling(x) getnstimeofday(x)

#define TUNNEL_REG_READ_SIZE	4
/*#define TUNNEL_PROFILE 1*/
/**
 * Internal interfaces of tunneling service
 */
/**
 * dump_buffer() - Debug function to print buffer
 * @buff:		pointer to the buffer
 * @len:		length of the buffer
 * @str:		string that will be print before printing the buffer
 */
static inline void dump_buffer(const char *buff, int len, const char *str)
{
#ifdef DEBUG
	int i;
	u32 *ptr;

	pr_debug("%s: ", str);
	for (i = 0, ptr = (u32 *)buff; i < (len/sizeof(u32)); i++)
		pr_cont("0x%08x ", ptr[i]);
#endif
}

/**
 * iaxxx_get_tunnel_output_buffer_size() - get the tunnel output buffer size
 * @priv: pointer to the MFD Private struct.
 * @tunnel_buff_thrshld: buffer threshold.
 *
 * Returns success or failure.
 */
int iaxxx_get_tunnel_output_buffer_size(struct iaxxx_priv *priv,
					uint32_t *tunnel_buff_size)
{
	int rc = 0;
	struct device *dev = priv->dev;

	rc = regmap_read(priv->regmap,
			IAXXX_TNL_HDR_TNL_OUT_BUF_SIZE_CONFIG_ADDR,
			tunnel_buff_size);
	if (rc) {
		dev_err_ratelimited(dev,
		"Failed TNL_HDR_TNL_OUT_BUF_SIZE_CONFIG_ADDR, rc:%d\n", rc);
		return rc;
	}

	dev_dbg(priv->dev,
		"tunnel_buff_size %u\n", *tunnel_buff_size);

	return rc;
}

/**
 * iaxxx_set_tunnel_output_buffer_size() - update the tunnel output buffer size
 * @priv: pointer to the MFD Private struct.
 * @tunnel_buff_thrshld: buffer threshold.
 *
 * Returns success or failure.
 */
int iaxxx_set_tunnel_output_buffer_size(struct iaxxx_priv *priv,
					uint32_t tunnel_buff_size)
{
	int rc = 0;
	struct device *dev = priv->dev;

	rc = regmap_write(priv->regmap,
			IAXXX_TNL_HDR_TNL_OUT_BUF_SIZE_CONFIG_ADDR,
			tunnel_buff_size);
	if (rc)
		dev_err_ratelimited(dev,
		"Failed TNL_HDR_TNL_OUT_BUF_SIZE_CONFIG_ADDR, rc:%d\n", rc);

	return rc;
}

/**
 * iaxxx_set_tunnel_event_threshold() - write tunneling threshold.
 * @priv: pointer to the MFD Private struct.
 * @tunnel_buff_thrshld: buffer threshold.
 *
 * Returns success or failure.
 */
int iaxxx_set_tunnel_event_threshold(struct iaxxx_priv *priv,
					uint32_t tunnel_buff_thrshld)
{
	int rc = 0;
	struct device *dev = priv->dev;

	rc = regmap_write(priv->regmap,
			IAXXX_TNL_HDR_TNL_OUT_BUF_THRESHOLD_ADDR,
			tunnel_buff_thrshld);
	if (rc) {
		dev_err_ratelimited(dev,
			"TNL_HDR_TNL_OUT_BUF_THRESHOLD failed : %d\n", rc);
		return rc;
	}

	dev_dbg(priv->dev,
		"tunnel_buff_thrshld: %u\n", tunnel_buff_thrshld);

	return rc;
}

/**
 * iaxxx_get_tunnel_buff_params() - read tunneling buf size.
 * @priv: pointer to the MFD Private struct.
 * @buff_param: buffer params.
 *
 * Returns the number buff params.
 */
int iaxxx_get_tunnel_buff_params(struct iaxxx_priv *priv,
			struct iaxxx_tunnel_buff_params *buff_param)
{
	int rc = 0;
	struct device *dev = priv->dev;
	uint32_t  reg_read[TUNNEL_REG_READ_SIZE];
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;
#ifdef TUNNEL_PROFILE
	struct timespec read_start;
	struct timespec read_end;
	struct timespec read_time;
#endif

#ifdef TUNNEL_PROFILE
	ia_profiling(&read_start);
#endif

	if (!t_intf_priv->tunnel_first_attach) {
		rc = regmap_bulk_read(priv->regmap,
				IAXXX_TNL_HDR_TNL_OUT_BUF_SIZE_ADDR,
				reg_read, ARRAY_SIZE(reg_read));
		if (rc) {
			dev_err_ratelimited(dev,
			"TNL_HDR_TNL_OUT_BUF_SIZE_ADDR failed : %d\n", rc);
			return rc;
		}

#ifdef TUNNEL_PROFILE
		ia_profiling(&read_end);
		read_time = (timespec_sub(read_end, read_start));
		dev_info(priv->dev, "Total read time = %lu.%03lu sec\n",
				read_time.tv_sec, (read_time.tv_nsec)/1000000);
#endif

		buff_param->buff_size = reg_read[0];
		buff_param->buff_addr = reg_read[1];
		buff_param->buff_head = reg_read[2];
		buff_param->buff_tail = reg_read[3];

		dev_dbg(priv->dev,
			"bufSize: %u bufAddr: %x bufHead: %u bufTail: %u\n",
			buff_param->buff_size, buff_param->buff_addr,
			buff_param->buff_head, buff_param->buff_tail);

		t_intf_priv->tunnel_first_attach_buff_params = *buff_param;
		t_intf_priv->tunnel_first_attach = true;
	} else {

		*buff_param = t_intf_priv->tunnel_first_attach_buff_params;
		/* Reading only two registers - Tunnel Head and Tail pointers */
		rc = regmap_read(priv->regmap,
				IAXXX_TNL_HDR_TNL_OUT_BUF_TAIL_ADDR,
				&buff_param->buff_tail);
		if (rc) {
			dev_err_ratelimited(dev,
			"TNL_HDR_TNL_OUT_BUF_HEAD_ADDR failed : %d\n", rc);
			return rc;
		}

#ifdef TUNNEL_PROFILE
		ia_profiling(&read_end);
		read_time = (timespec_sub(read_end, read_start));
		dev_info(priv->dev, "Total read time = %lu.%03lu sec\n",
				read_time.tv_sec, (read_time.tv_nsec)/1000000);
#endif

		dev_dbg(priv->dev, "bufHead: %u bufTail: %u\n",
			buff_param->buff_head, buff_param->buff_tail);
	}

	if (buff_param->buff_head > buff_param->buff_size ||
		buff_param->buff_tail > buff_param->buff_size ||
		buff_param->buff_size == 0 || buff_param->buff_addr == 0) {
		dev_err_ratelimited(priv->dev,
			"wrong buf values head(%d) tail(%d) addr(0x%x) size(%d)\n",
			buff_param->buff_head, buff_param->buff_tail,
			buff_param->buff_addr, buff_param->buff_size);
		iaxxx_fw_crash(dev, IAXXX_FW_CRASH_TUNNEL_WRONG_BUFF);
		return -EINVAL;
	}

	return rc;
}

/**
 * iaxxx_update_tunnel_buff_params() - read tunneling buf size.
 * @priv: pointer to the MFD Private struct.
 * @buff_head: update the read buffer.
 * @mode: 0 - async, other - sync
 *
 * Returns the number buff params.
 */
int iaxxx_update_tunnel_buff_params(struct iaxxx_priv *priv,
				uint32_t buff_head, int mode)
{
	struct device *dev = priv->dev;
	uint32_t status = 0;
	int rc = 0;
#ifdef TUNNEL_PROFILE
	struct timespec read_start;
	struct timespec read_end;
	struct timespec read_time;
#endif

#ifdef TUNNEL_PROFILE
	ia_profiling(&read_start);
#endif
	/* Update the buffer head in the device */
	rc = regmap_write(priv->regmap,
				IAXXX_TNL_HDR_TNL_OUT_BUF_HEAD_ADDR, buff_head);
	if (rc) {
		dev_err_ratelimited(dev,
			"clearing the out buff head addr failed : %d\n", rc);
		goto update_error;
	}

	rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				priv->regmap, 0,
				UPDATE_BLOCK_FIXED_WAIT,
				&status);
	if (rc) {
		dev_err_ratelimited(dev,
			"tnl buf param update block req failed : %d\n", rc);
		goto update_error;
	}

#ifdef TUNNEL_PROFILE
		ia_profiling(&read_end);
		read_time = (timespec_sub(read_end, read_start));
		dev_info(dev, "Total reg update time = %lu.%03lu sec\n",
				read_time.tv_sec, (read_time.tv_nsec)/1000000);
#endif

update_error:
	return rc;
}

/**
 * iaxxx_tunnel_read() - read tunneling data.
 * @dev: pointer to the MFD device.
 * @readbuff: buffer pointer to which data has to be copied.
 * @count: length of the data to be copied.
 * @mode: 0: async other: sync
 * @bytes_remainning: amount of data left in the FW buffer.
 *
 * Returns the number of bytes copied to the buffer.
 */
int iaxxx_tunnel_read(struct iaxxx_priv *priv, void *readbuff,
			    int words_to_read, int mode, int *bytes_remaining)
{
	struct device *dev = priv->dev;
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;
	struct iaxxx_tunnel_buff_params buff_param;
	uint32_t buff_head_prev;
	uint32_t words_read = 0;
	int read_len = 0;
	uint32_t words_left_to_read = words_to_read;
	int rc = -EINVAL;
#ifdef TUNNEL_PROFILE
	struct timespec read_start;
	struct timespec read_end;
	struct timespec read_time;
#endif

	dev_dbg(dev,
		"buff pointer: %p read count req in words: %u\n",
		readbuff, words_to_read);

	*bytes_remaining = 0;
	/* get fw tunnel buff head and tail pointers to read frames */
	rc = iaxxx_get_tunnel_buff_params(priv, &buff_param);
	if (rc < 0) {
		dev_err_ratelimited(dev,
			"get tunnel buff params failed : %d\n", rc);
		return rc;
	}

#ifdef TUNNEL_PROFILE
	ia_profiling(&read_start);
#endif
	/* keep current fw head offset for next FW head offset update*/
	buff_head_prev = buff_param.buff_head;

	while (buff_param.buff_head != buff_param.buff_tail
				&& read_len < words_to_read) {

		if (buff_param.buff_head < buff_param.buff_tail)
			words_read =
			buff_param.buff_tail - buff_param.buff_head;
		else
			words_read =
			buff_param.buff_size - buff_param.buff_head;

		if (words_read > words_left_to_read) {
			/* Bytes remaining in the tunnel. Need to issue
			 * read without again, waiting for interrupt.
			 */
			*bytes_remaining = words_read - words_left_to_read;
			words_read = words_left_to_read;
		}

		dev_dbg(dev, "words_read: %u\n", words_read);
		/* Perform a bulk read from chip over control interface */
		rc = priv->bus_read(priv->dev,
		(buff_param.buff_addr + (buff_param.buff_head * sizeof(u32))),
			readbuff, words_read);
		if (rc < 0) {
			dev_err_ratelimited(dev, "reading the frame data failed : %d\n",
									rc);
			return rc;
		}

		readbuff += (words_read * sizeof(uint32_t));
		read_len += words_read;
		words_left_to_read -= words_read;

		dev_dbg(dev, "words_left_to_read: %u\n", words_left_to_read);

		buff_param.buff_head += words_read;
		if (buff_param.buff_head >= buff_param.buff_size)
			buff_param.buff_head = 0;

		dev_dbg(dev, "buff_param.buff_head local: %u\n",
						buff_param.buff_head);
	}

#ifdef TUNNEL_PROFILE
	ia_profiling(&read_end);
	read_time = (timespec_sub(read_end, read_start));
	dev_info(dev, "Total bulk read time = %lu.%03lu sec\n",
				read_time.tv_sec, (read_time.tv_nsec)/1000000);
#endif

	if (buff_param.buff_head != buff_head_prev) {
		dev_dbg(dev, "update fw buff_head: %u\n", buff_param.buff_head);
		rc = iaxxx_update_tunnel_buff_params(priv,
			buff_param.buff_head, mode);
		if (rc) {
			dev_err_ratelimited(dev,
			"failed to update buff head rc = %d\n", rc);
			return rc;
		}
		/* Store Head ptr so that no need to read it from register */
		t_intf_priv->tunnel_first_attach_buff_params.buff_head =
			buff_param.buff_head;
	} else {
		dev_dbg(dev, "FW buff is empty at this movement\n");
		read_len = 0;
	}

	if (priv->tunnel_threshold > 0) {
		/* Re-read buff_param to get the exact data size in queue */
		rc = iaxxx_get_tunnel_buff_params(priv, &buff_param);
		if (!rc && (buff_param.buff_head != buff_param.buff_tail)) {
			/* means some data is pending in the buffer */
			if (buff_param.buff_head < buff_param.buff_tail)
				*bytes_remaining =
					buff_param.buff_tail
					- buff_param.buff_head;
			else
				*bytes_remaining = buff_param.buff_size
						- buff_param.buff_head
						+ buff_param.buff_tail;
		}
	}

	dev_dbg(dev, "read return length: %d\n", read_len);
	return read_len;
}

/**
 * iaxxx_tunnel_setup() - Tunnel setup for the endpoint data.
 * @dev: pointer to the MFD device.
 * @tunlEP: endpoint to setup from.
 * @tunlSrc: source for this endpoint
 * @tunlMode: Tunnel mode sync or async
 * @tunlEncode: Tunnel encoding format
 *
 * Returns 0 in success < 0 if any error caused in unsubscribe.
 */
int iaxxx_tunnel_setup(struct iaxxx_priv *priv,
			uint32_t tunlEP, uint32_t tunlSrc,
			uint32_t tunlMode, uint32_t tunlEncode)
{
	struct device *dev = priv->dev;
	uint32_t tunnel_ctrl_val;
	uint32_t tunnel_hdr_en;
	uint32_t status = 0, addr;
	int rc = -EINVAL;

	if (dev == NULL) {
		pr_err("Invalid device pointer");
		goto error;
	}

	dev_dbg(dev, "Tunnel endpoint setup: 0x%x\n", tunlEP);
	if (tunlEP < 0 || tunlEP > 31) {
		dev_err(dev, "Invalid Tunnel EP %d provided to setup\n",
			tunlEP);
		goto error;
	}

	/* setup tunnel EP ctrl reg values */
	tunnel_ctrl_val =
	TNL_DEVICE_HOST_TX << IAXXX_OUT_TNL_GRP_TNL_CTRL_DIR_POS |
	tunlEncode << IAXXX_OUT_TNL_GRP_TNL_CTRL_OUTPUT_ENCODING_POS |
	tunlMode << IAXXX_OUT_TNL_GRP_TNL_CTRL_MODE_POS;

	/* Program TNL0_CTRL REG */
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_TNL_GRP_TNL_CTRL_ADDR,
				  tunlEP);
	if (!addr) {
		rc = -EINVAL;
		goto error;
	}

	rc = regmap_write(priv->regmap, addr, tunnel_ctrl_val);
	if (rc) {
		dev_err(dev,
			"set TunlEP_CTRL reg failed : %d\n", rc);
		goto error;
	}

	/* Program TNLEP_CONNECT SRC ID */
	addr = IAXXX_GET_GRP_ADDR(priv, IAXXX_OUT_TNL_GRP_CONNECT_ADDR,
				  tunlEP);
	if (!addr) {
		rc = -EINVAL;
		goto error;
	}

	rc = regmap_write(priv->regmap, addr, tunlSrc);
	if (rc) {
		dev_err(dev,
			"set TunlEP src connect reg failed : %d\n", rc);
		goto error;
	}

	rc = regmap_read(priv->regmap,
			IAXXX_TNL_HDR_TNL_ENABLE_ADDR, &tunnel_hdr_en);
	if (rc) {
		dev_err(dev,
			"read tunnel enable hdr failed : %d\n", rc);
		goto error;
	}

	tunnel_hdr_en = tunnel_hdr_en | (1 << tunlEP);
	/* Enable TNL HDR REG */
	rc = regmap_write(priv->regmap, IAXXX_TNL_HDR_TNL_ENABLE_ADDR,
								tunnel_hdr_en);
	if (rc) {
		dev_err(dev,
			"set TNL HDR EN reg failed : %d\n", rc);
		goto error;
	}

	/* Wait for the request to complete */
	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		dev_err(dev,
			"tnl enable addr update block failed : %d\n", rc);
		goto error;
	}

error:
	return rc;
}

/**
 * iaxxx_tunnel_terminate() - terminate the tunneling endpoint.
 * @dev: pointer to the MFD device.
 * @tunlEP: endpoint to terminate from.
 *
 * Returns 0 in success < 0 if any error caused in unsubscribe.
 */
int iaxxx_tunnel_terminate(struct iaxxx_priv *priv, uint32_t tunlEP)
{
	struct device *dev = priv->dev;
	uint32_t tunnel_hdr_dis = 0;
	uint32_t status = 0;
	int rc = -EINVAL;

	if (dev == NULL) {
		pr_err("Invalid device pointer");
		goto error;
	}

	dev_dbg(dev, "Terminate for tunnel endpoint: 0x%x\n", tunlEP);
	if (tunlEP < 0 || tunlEP > 31) {
		dev_err(dev, "Invalid Tunnel EP %d provided to setup\n",
			tunlEP);
		goto error;
	}

	rc = regmap_read(priv->regmap,
			IAXXX_TNL_HDR_TNL_ENABLE_ADDR, &tunnel_hdr_dis);
	if (rc) {
		dev_err(dev,
			"read tunnel enable hdr failed : %d\n", rc);
		goto error;
	}

	tunnel_hdr_dis = tunnel_hdr_dis & ~(1 << tunlEP);
	/* Disable TNL HDR REG */
	rc = regmap_write(priv->regmap,
			IAXXX_TNL_HDR_TNL_ENABLE_ADDR, tunnel_hdr_dis);
	if (rc) {
		dev_err(dev,
			"disable tunnel endpoint failed : %d\n", rc);
		goto error;
	}

	/* Wait for the request to complete */
	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		dev_err(dev,
			"tunnel update_block_request failed : %d\n", rc);
		goto error;
	}
error:
	return rc;
}

MODULE_DESCRIPTION("Tunneling Service Layer module");
