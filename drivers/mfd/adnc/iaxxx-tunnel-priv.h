/*
 * iaxxx-tunnel-priv.h -- iaxxx tunneling Service private data
 *
 * Copyright (c) 2016 Audience, inc.
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

#ifndef _IAXXX_TUNNEL_H
#define _IAXXX_TUNNEL_H

#include <linux/mfd/adnc/iaxxx-register-defs-out-tunnel-group.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/cdev.h>
#include <linux/timer.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"

enum iaxxx_tunnel_thread_status {
	TNL_THRD_NONE = 0,
	TNL_THRD_STARTED,
	TNL_THRD_RUNNING,
	TNL_THRD_WAIT1,
	TNL_THRD_WAIT2,	/* different wait events */
	TNL_THRD_PARK,
	TNL_THRD_PARKED,
	TNL_THRD_UNPARK,
	TNL_THRD_SLEEP,
	TNL_THRD_STOP,
	TNL_THRD_STOPPED,
	TNL_THRD_EXIT
};

enum iaxxx_out_tunnel_type {
	TNL0 = 0,
	TNL1,
	TNL2,
	TNL3,
	TNL4,
	TNL5,
	TNL6,
	TNL7,
	TNL8,
	TNL9,
	TNL10,
	TNL11,
	TNL12,
	TNL13,
	TNL14,
	TNL15,
	TNL16,
	TNL17,
	TNL18,
	TNL19,
	TNL20,
	TNLMAX
};

struct iaxxx_circ_buf {
	char *buf;
	int head;
	int tail;
	int size;
};

struct iaxxx_tunnel_buff_params {
	uint32_t buff_tail;
	uint32_t buff_head;
	uint32_t buff_addr;
	uint32_t buff_size;
};

struct iaxxx_tunnel_data;
struct iaxxx_tunnel_client {
	struct iaxxx_tunnel_data *tunnel_data;
	struct iaxxx_circ_buf user_circ;
	struct list_head node;
	wait_queue_head_t wq;
	unsigned long tid_flag;
};

struct iaxxx_tunnel_data {
	struct device *dev; /* mfd device */
	struct iaxxx_priv *priv;

	/* Tunnel device locks */
	struct mutex tunnel_dev_lock;

	struct iaxxx_cdev tunnel_cdev;

	struct iaxxx_circ_buf stream_circ;

	struct list_head list;
	struct list_head src_list;
	spinlock_t src_list_lock;
	spinlock_t lock;

	unsigned long flags;	/* Bit 0: Tunnel 0, ... Bit31: Tunnel 31 */
	/* Bit 0: Tunnel 0, ... Bit31: Tunnel 31
	 * Corresponding bits are enabled to indicate which
	 * Tunnels are setup for Sync Mode
	 */
	unsigned long flags_sync_mode;

	int client_no;
	atomic_t src_enable_id[TNLMAX];
	atomic_t event_occurred;
	atomic_t tunnel_ref_cnt[TNLMAX];
	atomic_t tunnel_evt_cnt;
	uint32_t tunnel_seq_no[TNLMAX];
	int tunnels_active_count;
	unsigned long tunnel_packet_no[TNLMAX];
	unsigned long tunnel_total_packet_no;
	uint32_t tunnel_seq_err[TNLMAX];
	unsigned long tunnel_magic_errcnt;
	struct task_struct *producer_thread;
	uint32_t producer_thread_sts;
	struct task_struct *consumer_thread;
	uint32_t consumer_thread_sts;
	wait_queue_head_t consumer_wq;
	wait_queue_head_t producer_wq;
	struct notifier_block crash_notifier;
	bool event_registered;
	unsigned long tunnel_state;
	bool tunnel_first_attach;
	struct iaxxx_tunnel_buff_params tunnel_first_attach_buff_params;
	unsigned long tunnels_enabled;

	struct iaxxx_tunnel_client dummy_client;
	struct timer_list timer_to_stop_kw_buffering;
};

int iaxxx_tunnel_setup(struct iaxxx_priv *priv,
		uint32_t tunlEP, uint32_t tunlSrc,
		uint32_t tunlMode, uint32_t tunlEncode);
int iaxxx_tunnel_terminate(struct iaxxx_priv *priv, uint32_t tunlEP);
int iaxxx_get_tunnel_buff_params(struct iaxxx_priv *priv,
			struct iaxxx_tunnel_buff_params *buff_param);
int iaxxx_update_tunnel_buff_params(struct iaxxx_priv *priv,
				uint32_t buff_head, int mode);
int iaxxx_tunnel_read(struct iaxxx_priv *priv, void *readbuff, int count,
			int mode, int *bytes_remaining);
int iaxxx_tunnel_dev_init(struct iaxxx_priv *priv);
int iaxxx_tunnel_dev_destroy(struct iaxxx_priv *priv);
int iaxxx_set_tunnel_event_threshold(struct iaxxx_priv *priv,
					uint32_t tunnel_buff_thrshld);
int iaxxx_get_tunnel_output_buffer_size(struct iaxxx_priv *priv,
					uint32_t *tunnel_buff_size);
int iaxxx_start_keyword_buffering(struct iaxxx_priv *priv);
#endif
