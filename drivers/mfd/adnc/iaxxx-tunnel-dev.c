/*
 * iaxxx-tunnel-dev.c -- iaxxx Tunneling Service device node
 *
 * Copyright 2016 Knowles Corporation
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

#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mfd/adnc/iaxxx-tunnel-intf.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>

#include "iaxxx.h"
#include "iaxxx-cdev.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-tunnel-priv.h"

#define PBUFF_SIZE			(1024*1024)	/* Producer Buffer */
#define UBUFF_SIZE			(256*1024)	/* Client Fifo */

#define TUNNEL_MAGIC1 0x45
#define TUNNEL_MAGIC2 0x4D
#define TUNNEL_MAGIC3 0x4F
#define TUNNEL_MAGIC4 0x52

#define TNL_SYNC_MODE 0
#define TNL_ASYNC_MODE 1
#define MAX_PACKET_SIZE (1024 * 4)

#define IAXXX_TUNNEL_THRESHOLD 0x800

#define CHECK_SEQUENCE

#define PRODUCER_PRIO 15	/* Less than mdss_dsi_event */
#define PRODUCER_WAIT_TIME_US 5000 /* 5ms interval */
#define PRODUCER_MAX_WAIT_TIME_US 100000 /* 100ms interval */
#define TNL_SRC_Q 0x00010000	/* For defining new tunnel id with Q format */

#define IAXXX_DEBUG_LAUNCH_DELAY 60000 /* 60 seconds */

#define IAXXX_TFLG_FW_CRASH		0

#define IAXXX_TNL_TERM_CHECK_RETRY_COUNT	50
#define IAXXX_TNL_TERM_DELAY		5000
#define IAXXX_TNL_TERM_DELAY_RANGE	1000

struct iaxxx_tunnel_ep {
	struct tunlMsg tnl_ep;
	struct list_head src_head_list;
};

static phys_addr_t iaxxx_prod_buf;
static size_t iaxxx_prod_buf_size;

#define is_valid_tid(__id__) ((__id__) < TNLMAX)
#define is_valid_size(__size__) ((__size__) > 0 && \
			(__size__) <= MAX_PACKET_SIZE && \
			!((__size__) & 3))

#define circ_cnt(__c__) \
	(CIRC_CNT((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_space(__c__) \
	(CIRC_SPACE((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_cnt_to_end(__c__) \
	(CIRC_CNT_TO_END((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_space_to_end(__c__) \
	(CIRC_SPACE_TO_END((__c__)->head, (__c__)->tail, (__c__)->size))

/*
 * Check the buffer falls into kernel address space or in userspace.
 * If the pointer is >= PAGE_OFFSET belong to kernel address.
 */
static inline bool is_user_buf(unsigned long ptr)
{
	bool ret = false;

	/*  Easy way to check the buffer is in user spacer or kernel */
	if (ptr < PAGE_OFFSET)
		ret = true;

	return ret;
}

/*
 * Init head / tail
 */
static inline void circ_init(struct iaxxx_circ_buf *circ)
{
	circ->head = circ->tail = 0;
}

/*
 * Update tail
 */
static inline void circ_peek_finish(struct iaxxx_circ_buf *circ, int size)
{
	smp_wmb(); /* Make sure data is copied before updating indexes */
	circ->tail = (circ->tail + (size >> 2)) & (circ->size - 1);
}

/*
 * Update head pointer
 */
static inline void circ_fill_finish(struct iaxxx_circ_buf *circ, int size)
{
	smp_wmb(); /* Make sure data is copied before updating indexes */
	circ->head = (circ->head + (size >> 2)) & (circ->size - 1);
}

/*
 * Return maximum contiguous free buffer
 */
static inline int circ_get_free_buffer(struct iaxxx_circ_buf *circ, void **buf)
{
	int size = circ_space_to_end(circ) << 2;

	if (size > 0)
		*buf = &circ->buf[circ->head << 2];

	return size;
}

/*
 * Fill buf with the data in circular queue but no tail moving
 */
static inline int circ_peek_data(struct iaxxx_circ_buf *circ,
						void *buf, int size)
{
	u8 *p = buf;

	int len = min(size, circ_cnt(circ) << 2);
	int first_len =
		min(len, circ_cnt_to_end(circ) << 2);

	smp_rmb(); /* Make sure fetching indexes before data copy */
	memcpy(p, circ->buf + (circ->tail << 2), first_len);
	if (first_len < len)
		memcpy(p + first_len, circ->buf, len - first_len);

	return len;
}

/*
 * Push data to circular queue
 */
static inline void circ_in(struct iaxxx_circ_buf *circ,
					const void *buf, int size)
{
	void *user_buf = NULL;
	int len;

	len = circ_get_free_buffer(circ, &user_buf);
	if (len < size) {
		memcpy(user_buf, buf, len);
		memcpy(circ->buf, buf + len, size - len);
	} else {
		memcpy(user_buf, buf, size);
	}

	circ_fill_finish(circ, size);
}

/*
 * Copy data from circular buffer to user buffer
 */
static int circ_copy_to_user(struct iaxxx_circ_buf *circ,
		void *buf, int size)
{
	int len;
	int ret;

	len = min(size, circ_cnt_to_end(circ) << 2);

	if (is_user_buf((unsigned long) buf) == true) {
		ret = copy_to_user((char __user *)buf,
					circ->buf + (circ->tail << 2), len);
		if (ret)
			return -EFAULT;

		if (size > len) {
			ret = copy_to_user((char __user *)buf + len,
					circ->buf, size - len);
			if (ret)
				return -EFAULT;
		}
	} else {
		memcpy(buf, circ->buf + (circ->tail << 2), len);

		if (size > len)
			memcpy(buf + len, circ->buf, size - len);
	}
	circ_peek_finish(circ, size);

	return 0;
}

/*
 * Copy whatever the size of tunnel buffer that
 * user has requested regardless of whether
 * it is frame-aligned or not. If the amount
 * of data request is not available return
 * the available data.
 *
 */
static int circ_to_user(struct iaxxx_circ_buf *circ, void *buf,
		size_t count, unsigned int *copied)
{
	int cnt = min_t(int, circ_cnt(circ) << 2, count);

	if (circ_copy_to_user(circ, buf, cnt))
		return -EFAULT;

	*copied = cnt;
	return 0;
}


/* Tunnel end-point source list functions */

/* Add end-point node to source list */
static void iaxxx_tunnel_src_list_add_endpoint(
		struct iaxxx_tunnel_data *t_intf_priv,
		struct iaxxx_tunnel_ep *tnl_src_node)
{
	spin_lock(&t_intf_priv->src_list_lock);
	list_add_tail(&tnl_src_node->src_head_list,
			&t_intf_priv->src_list);
	spin_unlock(&t_intf_priv->src_list_lock);
}

/* Delete given end-point from source list */
static void iaxxx_tunnel_src_list_del_endpoint(
		struct iaxxx_tunnel_data *t_intf_priv,
		int ep_id)
{
	struct list_head *position, *tmp;
	struct iaxxx_tunnel_ep *tnl_src_node;

	spin_lock(&t_intf_priv->src_list_lock);
	list_for_each_safe(position,
			tmp, &t_intf_priv->src_list) {
		tnl_src_node = list_entry(position,
				struct iaxxx_tunnel_ep, src_head_list);
		/* map and remove the src node from list */
		if (tnl_src_node->tnl_ep.tunlEP == ep_id) {
			list_del(position);
			kvfree(tnl_src_node);
			goto exit;
		}
	}
exit:
	spin_unlock(&t_intf_priv->src_list_lock);
}

/* Find a end-point id in source list based on parameters given
 * If not return the (greatest ep-id in list + 1)
 */
static int iaxxx_tunnel_src_list_find_endpoint(
		struct iaxxx_tunnel_data *t_intf_priv,
		int src, uint32_t mode, uint32_t encode,
		bool *ep_exists)
{
	int id = 0;
	struct iaxxx_tunnel_ep *tnl_src_node;
	struct list_head *pos, *tmp;

	*ep_exists = false;

	spin_lock(&t_intf_priv->src_list_lock);
	if (!list_empty_careful(&t_intf_priv->src_list)) {
		list_for_each_safe(pos, tmp, &t_intf_priv->src_list) {
			tnl_src_node = list_entry(pos,
				struct iaxxx_tunnel_ep, src_head_list);
			if (tnl_src_node->tnl_ep.tunlSrc == src &&
				tnl_src_node->tnl_ep.tunlEncode == encode &&
				tnl_src_node->tnl_ep.tunlMode == mode) {
				*ep_exists = true;
				id = tnl_src_node->tnl_ep.tunlEP;
				goto exit;
			}
			/* update id if the current end point id is
			 * greter than previous id.
			 */
			if (tnl_src_node->tnl_ep.tunlEP > id)
				id = tnl_src_node->tnl_ep.tunlEP;
		}
		/* Add one to id.
		 * This is will be the next max best id to assign
		 */
		id += 1;
	} else
		id = -EINVAL;
exit:
	spin_unlock(&t_intf_priv->src_list_lock);
	return id;
}

/* Find a end-point node in source list for end-point id given */
static bool iaxxx_tunnel_src_list_find_endpoint_node(
		struct iaxxx_tunnel_data *t_intf_priv,
		int ep_id,
		struct iaxxx_tunnel_ep *ret_tnl_src_node)
{
	struct iaxxx_tunnel_ep *tnl_src_node;
	struct list_head *pos, *tmp;
	bool result = false;

	spin_lock(&t_intf_priv->src_list_lock);
	list_for_each_safe(pos, tmp, &t_intf_priv->src_list) {
		tnl_src_node = list_entry(pos, struct iaxxx_tunnel_ep,
				src_head_list);

		if (tnl_src_node->tnl_ep.tunlEP == ep_id) {
			*ret_tnl_src_node = *tnl_src_node;
			result = true;
			goto exit;
		}
	}
exit:
	spin_unlock(&t_intf_priv->src_list_lock);
	return result;

}

/* Return all endpoint nodes as array */
static int iaxxx_tunnel_src_list_get_all_endpoint_nodes(
		struct iaxxx_tunnel_data *t_intf_priv,
		struct iaxxx_tunnel_ep ret_tnl_src_node[],
		int max_ret_entries)
{
	struct iaxxx_tunnel_ep *tnl_src_node;
	struct list_head *pos, *tmp;
	int index = 0;

	spin_lock(&t_intf_priv->src_list_lock);
	list_for_each_safe(pos, tmp, &t_intf_priv->src_list) {
		tnl_src_node = list_entry(pos, struct iaxxx_tunnel_ep,
				src_head_list);
		ret_tnl_src_node[index++] = *tnl_src_node;
		if (max_ret_entries == index)
			goto exit;
	}
exit:
	spin_unlock(&t_intf_priv->src_list_lock);
	return index;

}

/*
 * Parse packet header and return tunnel ID and data len
 */
static inline int parse_header(struct iaxxx_tunnel_header *header, u16 *tid)
{
	if (header->magic[0] != TUNNEL_MAGIC1 ||
		header->magic[1] != TUNNEL_MAGIC2 ||
		header->magic[2] != TUNNEL_MAGIC3 ||
		header->magic[3] != TUNNEL_MAGIC4 ||
		!is_valid_tid(header->tunnel_id) ||
		((header->encoding != TNL_ENC_OPAQUE) &&
		!is_valid_size(header->size))) {
		pr_err_ratelimited(
			"Fault pkt: magic: 0x%x 0x%x 0x%x 0x%x, tid=%d, sz=%d",
			header->magic[0], header->magic[1],
			header->magic[2], header->magic[3],
			header->tunnel_id, header->size);
		return -EINVAL;
	}

	*tid = header->tunnel_id;
	return header->size;
}

/*
 * Request to subscribe the tunnel events with a threshold
 */
static int tunnel_event_subscribe(struct iaxxx_tunnel_data *t_intf_priv,
					uint32_t src_id, uint32_t evt_id,
					uint32_t dst_id, uint32_t destOpaque,
					uint32_t event_thrshld)
{
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct device *dev = (struct device *)priv->dev;
	int ret = 0;

	pr_debug("dev id-%d: tunnel_evt_cnt: %d, event_thrshld %d",
			priv->dev_id, atomic_read(&t_intf_priv->tunnel_evt_cnt),
			event_thrshld);

	/* Only in non-polling mode events are subscribed
	 * and for first opened tunnel.
	 */
	if (event_thrshld &&
		atomic_inc_return(&t_intf_priv->tunnel_evt_cnt) == 1) {
		ret = iaxxx_set_tunnel_event_threshold(priv, event_thrshld);
		if (ret) {
			pr_err("dev id-%d: set tnl evt threshold failed : %d",
							priv->dev_id, ret);
			return ret;
		}

		ret = iaxxx_core_evt_subscribe(dev, src_id, evt_id,
						dst_id, destOpaque);
		if (ret) {
			pr_err("dev id-%d: evt subscribe failed : %d",
							priv->dev_id, ret);
			return ret;
		}
	}
	return ret;
}

/*
 * Request to unsubscribe the tunnel event
 */
static int tunnel_event_unsubscribe(struct iaxxx_tunnel_data *t_intf_priv,
					uint32_t src_id, uint32_t evt_id,
					uint32_t dst_id, uint32_t destOpaque,
					uint32_t event_thrshld)
{
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct device *dev = (struct device *)priv->dev;
	int ret = 0;

	pr_debug("dev id-%d: tunnel_evt_cnt: %d, event_thrshld %d",
			priv->dev_id, atomic_read(&t_intf_priv->tunnel_evt_cnt),
			event_thrshld);

	/* Only in non-polling mode events are un-susbcribed
	 * and for last closed tunnel.
	 */
	if (priv->tunnel_threshold &&
		atomic_dec_return(&t_intf_priv->tunnel_evt_cnt) == 0) {
		ret = iaxxx_set_tunnel_event_threshold(priv,
						priv->tunnel_threshold);
		if (ret) {
			pr_err("dev id-%d: set tnl evt threshold failed : %d",
							priv->dev_id, ret);
			return ret;
		}
		ret = iaxxx_core_evt_unsubscribe(dev, src_id, evt_id,
						dst_id);
		if (ret) {
			pr_err("dev id-%d: evt subscribe failed : %d",
							priv->dev_id, ret);
			return ret;
		}
	}

	return ret;
}

/*
 * Adjust tunnel settings by comparing new & old flags
 */
static void adjust_tunnels(struct iaxxx_tunnel_data *t_intf_priv,
				unsigned long new)
{
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	int changes = (t_intf_priv->tunnels_enabled ^ new) &
							((1 << TNLMAX) - 1);
	int id;
	int pos;
	int tnl_count = 0;

	while (changes) {
		id = ffs(changes) - 1;
		pos = (1 << id);

		if (new & pos) {
			/*
			 * The chip level tunnel setup is moved to SETUP
			 * ioctl to handle the endpoint errors and return
			 * failure immediately.
			 */

			if (!t_intf_priv->event_registered) {
				if (!tunnel_event_subscribe(t_intf_priv,
					IAXXX_SYSID_TUNNEL_EVENT, 0,
					IAXXX_SYSID_HOST, 0,
					priv->tunnel_threshold)) {
					t_intf_priv->event_registered = true;
				} else {
					pr_err_ratelimited(
					"dev id-%d: tnl events subscribe failed",
					priv->dev_id);
					return;
				}
			}

		} else {
			/*
			 * The chip level tunnel term is moved to TERMINATE
			 * ioctl to handle the endpoint errors and return
			 * failure immediately.
			 */

			while (tnl_count < TNLMAX) {
				if (atomic_read(
				&t_intf_priv->src_enable_id[tnl_count]) == 1)
					break;
				tnl_count++;
			}
			if (tnl_count == TNLMAX &&
				t_intf_priv->event_registered) {
				if (!tunnel_event_unsubscribe(t_intf_priv,
					IAXXX_SYSID_TUNNEL_EVENT, 0,
					IAXXX_SYSID_HOST, 0, 0)) {
					t_intf_priv->event_registered = false;
				} else {
					pr_err_ratelimited(
					"dev id-%d: subscribe tunnel events failed",
					priv->dev_id);
					return;
				}
			}
		}
		changes &= ~pos;

		t_intf_priv->tunnels_enabled ^= pos;
	}
}

/*
 * Get tunnel data from codec and fill * into circular buffer
 */
static int producer_thread(void *arg)
{
	struct iaxxx_tunnel_data *t_intf_priv = arg;
	struct iaxxx_priv *priv = t_intf_priv->priv;
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int size;
	int bytes, bytes_remaining;
	void *buf;
	unsigned long flags;
	int wait_time_us = PRODUCER_WAIT_TIME_US;
	static bool print_bfr_unavailable_flag = true;

	while (1) {
		t_intf_priv->producer_thread_sts = TNL_THRD_RUNNING;

		/* For MT9615 SPI driver timeout (interrupt missing) workaround */
		if (!atomic_read(&priv->pm_resume)) {
			pr_info("%s: System is supending/resuming, just wait for a while\n", __func__);
			usleep_range(5000, 5005);
			continue;
		}

		/* Get a free contiguous buffer */
		if (kthread_should_stop()) {
			t_intf_priv->producer_thread_sts = TNL_THRD_STOPPED;
			pr_debug("dev id-%d: thread should stop", priv->dev_id);
			break;
		}

		if (kthread_should_park()) {
			pr_info_ratelimited(
			"dev id-%d: parking producer thread", priv->dev_id);
			t_intf_priv->producer_thread_sts = TNL_THRD_PARKED;
			kthread_parkme();
			t_intf_priv->producer_thread_sts = TNL_THRD_RUNNING;
			continue;
		}

		flags = t_intf_priv->flags;
		if (flags != t_intf_priv->tunnels_enabled)
			adjust_tunnels(t_intf_priv, flags);

		size = circ_get_free_buffer(circ, &buf);

		if (size < 4) {
			if (print_bfr_unavailable_flag == true) {
				print_bfr_unavailable_flag = false;
				pr_err_ratelimited(
					"dev id-%d: No more buffer (size %d)",
							priv->dev_id, size);
			}

			/* if consumer thread is sleeping. wake it up */
			if (waitqueue_active(&t_intf_priv->consumer_wq))
				wake_up(&t_intf_priv->consumer_wq);
		}

		if (size >= 4 && t_intf_priv->tunnels_active_count > 0) {
			/* Get the data max size words */
			/* Sync mode reading is enabled if at least
			 * one tunnel is configured in sync mode
			 */
			bytes = iaxxx_tunnel_read(priv,
					buf, size >> 2,
					t_intf_priv->flags_sync_mode,
					&bytes_remaining) << 2;

			pr_debug("dev id-%d: buf = %p sz = %d, bytes = %d, cnt = %d",
					priv->dev_id, buf, size,
					bytes, circ_cnt(circ) << 2);

			if (bytes > 0) {
				circ_fill_finish(circ, bytes);

#if 0
				/*
				 * Wakeup consumer only when client
				 * is registered.
				 * If only debug client (client_no==0)
				 * just keep matching tail with head
				 */
				if (t_intf_priv->client_no > 0)
					wake_up(&t_intf_priv->consumer_wq);
				else
					circ->tail = circ->head;
#else
				/*
				 * Wakeup consumer only when client
				 * is registered.
				 * If no client (client_no==0), it means
				 * the keyword buffering is ongoing,
				 * so keep the tail not changed.
				 */
				if (t_intf_priv->client_no > 0)
					wake_up(&t_intf_priv->consumer_wq);
#endif
				/* Restore timeout value */
				wait_time_us = PRODUCER_WAIT_TIME_US;

				/* This means not in polling mode */
				if (priv->tunnel_threshold > 0) {
					/*
					 * Interrupt is not triggerred if the
					 * remaining data is bigger than
					 * threshold, so keep consuming data
					 */
					if ((bytes_remaining << 2) >=
							priv->tunnel_threshold)
						continue;

					if (atomic_xchg(
						&t_intf_priv->event_occurred, 0)
						> 1)
						continue;

					pr_debug("dev id-%d: tnl producer wait for event",
							priv->dev_id);
					t_intf_priv->producer_thread_sts =
								TNL_THRD_WAIT1;
					wait_event(t_intf_priv->producer_wq,
						atomic_read(
						&t_intf_priv->event_occurred)
						|| kthread_should_stop() ||
						kthread_should_park() ||
						t_intf_priv->flags !=
						t_intf_priv->tunnels_enabled);
					t_intf_priv->producer_thread_sts =
							TNL_THRD_RUNNING;
					continue;
				} else
					/* Read was successful and there could
					 * be data pending in tunnel buffer so
					 * continuing without delay
					 */
					continue;
			} else if (bytes == 0) {
				usleep_range(wait_time_us, wait_time_us + 5);
				continue;
			}

		}

		if (t_intf_priv->tunnels_enabled) {
			usleep_range(wait_time_us, wait_time_us + 5);

			/*
			 * If failed, keep increasing wait time to
			 * 100us until MAX
			 */
			wait_time_us += 100;
			wait_time_us = min(wait_time_us,
					PRODUCER_MAX_WAIT_TIME_US);
			continue;
		}
		pr_info_ratelimited("dev id-%d: producer thread wait for start",
								priv->dev_id);
		t_intf_priv->producer_thread_sts = TNL_THRD_WAIT2;
		wait_event(t_intf_priv->producer_wq,
			t_intf_priv->flags ||
			kthread_should_stop() ||
			kthread_should_park() ||
			t_intf_priv->flags != t_intf_priv->tunnels_enabled);
		t_intf_priv->producer_thread_sts = TNL_THRD_RUNNING;
		pr_info_ratelimited("dev id-%d: producer thread woken-up",
								priv->dev_id);
		print_bfr_unavailable_flag = true;
	}

	flags = t_intf_priv->flags;
	pr_debug("dev id-%d: producer thread flags: %lu tunnel_flags : %lu",
			priv->dev_id, flags, t_intf_priv->tunnels_enabled);
	if (flags != t_intf_priv->tunnels_enabled)
		adjust_tunnels(t_intf_priv, flags);

	t_intf_priv->producer_thread_sts = TNL_THRD_EXIT;
	print_bfr_unavailable_flag = true;
	return 0;
}

/*
 * Attach a client to tunnel stream
 */
static int tunneling_attach_client(struct iaxxx_tunnel_data *tunnel_data,
			struct iaxxx_tunnel_client *client)
{
	client->user_circ.buf = iaxxx_kvmalloc(UBUFF_SIZE, __GFP_ZERO);
	if (!client->user_circ.buf)
		return -ENOMEM;

	client->user_circ.size = UBUFF_SIZE >> 2;
	circ_init(&client->user_circ);

	client->tunnel_data = tunnel_data;

	spin_lock(&tunnel_data->lock);
	list_add_tail_rcu(&client->node, &tunnel_data->list);
	tunnel_data->client_no++;
	spin_unlock(&tunnel_data->lock);

	return 0;
}

/*
 * Detach a client from tunnel stream
 */
static int tunneling_detach_client(struct iaxxx_tunnel_data *tunnel_data,
			struct iaxxx_tunnel_client *client)
{
	spin_lock(&tunnel_data->lock);
	list_del_rcu(&client->node);
	tunnel_data->client_no--;
	spin_unlock(&tunnel_data->lock);
	synchronize_rcu();

	kvfree(client->user_circ.buf);

	return 0;
}

/*
 * Return tunned id for given src end
 */
static int tunnel_find_id(struct iaxxx_tunnel_data *t_intf_priv,
			int src, uint32_t mode, uint32_t encode, int set)
{
	int id = 0;
	int unused_id = 0;
	bool ep_exists;

	id = iaxxx_tunnel_src_list_find_endpoint(t_intf_priv, src, mode,
			encode, &ep_exists);

	if (id < 0)
		return 0;

	if (ep_exists)
		return id;

	if (!set)
		return -ENOENT;

	pr_debug("new id : %d", id);
	/* see if any unused id can be found between the next best id */
	while (unused_id < id) {
		if (atomic_read(&t_intf_priv->src_enable_id[unused_id]) == 0)
			return unused_id;
		unused_id++;
	}

	/* If no unused id is found then check
	 * if next best ID is greater then or equal to max
	 * tunnel id then it should return error
	 * else return the next best id as tunnel id to assign.
	 */
	if (id >= TNLMAX) {
		pr_err("Invalid Tunnel id : %d", id);
		return -EINVAL;
	}

	return id;
}

/*
 * Copy buffer to a client's buffer
 */
static void tunnel_copy_to_client(struct iaxxx_tunnel_client *client,
				u16 tid, int count)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int size;

	if (!is_valid_tid(tid) || !test_bit(tid, &client->tid_flag))
		return;

	if ((circ_space(&client->user_circ) << 2) < count) {
		/* drop the current packet */
		pr_err_ratelimited("%d bytes of data dropped", count);
		return;
	}

	size = min(count, circ_cnt_to_end(circ) << 2);
	circ_in(&client->user_circ, circ->buf + (circ->tail << 2), size);
	if (size < count)
		circ_in(&client->user_circ, circ->buf, count - size);

	wake_up(&client->wq);
}

#ifdef CHECK_SEQUENCE
static void check_packet(struct iaxxx_tunnel_data *t_intf_priv,
			u16 tid, uint32_t seq_no)
{
	if (t_intf_priv->tunnel_seq_no[tid] == 0) {
		t_intf_priv->tunnel_seq_no[tid] = seq_no;
		return;
	}

	if ((seq_no - t_intf_priv->tunnel_seq_no[tid]) != 1) {
		pr_debug("Sequence number err id = %d old = 0x%x, new = 0x%x",
			tid,
			t_intf_priv->tunnel_seq_no[tid],
			seq_no);
		t_intf_priv->tunnel_seq_err[tid] +=
			seq_no - t_intf_priv->tunnel_seq_no[tid];
	}

	t_intf_priv->tunnel_seq_no[tid] = seq_no;
	t_intf_priv->tunnel_packet_no[tid]++;
	t_intf_priv->tunnel_total_packet_no++;
}
#endif

/*
 * Consume circular buffer and feed data to each client kfifo
 */
static int consumer_thread(void *arg)
{
	struct iaxxx_tunnel_data *t_intf_priv = arg;
	struct iaxxx_tunnel_client *client;
	u16 tid;
	int rc;
	int size;
	int sz_used;
	struct iaxxx_tunnel_header hdr;
	int hdr_size = sizeof(hdr);
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;

	while (1) {
		t_intf_priv->consumer_thread_sts = TNL_THRD_RUNNING;

		if (kthread_should_stop()) {
			t_intf_priv->consumer_thread_sts = TNL_THRD_STOPPED;
			break;
		}

		size = 0;
		sz_used = circ_cnt(circ) << 2;
		if (sz_used >= hdr_size) {
			circ_peek_data(circ, &hdr, hdr_size);

			size = parse_header(&hdr, &tid);
			if (size < 0) {
				/* Increase magic error */
				t_intf_priv->tunnel_magic_errcnt++;

				/* Packet header error */
				circ_peek_finish(circ, 4);
				continue;
			}
			if (size > 0)
				size += hdr_size;
		}

		if (size > 0 && sz_used >= size) {
			rcu_read_lock();
			/* Fill to each client's fifo */
			list_for_each_entry_rcu(client,
						&t_intf_priv->list, node)
				tunnel_copy_to_client(client, tid, size);
			rcu_read_unlock();
			circ_peek_finish(circ, size);

#ifdef CHECK_SEQUENCE
			if (t_intf_priv->client_no)
				check_packet(t_intf_priv, tid, hdr.seq_no);
#endif

			continue;
		}

		if (sz_used == 0) {
			t_intf_priv->consumer_thread_sts = TNL_THRD_WAIT1;

			wait_event(t_intf_priv->consumer_wq,
				(circ_cnt(circ) > 0) ||
				kthread_should_stop());
			t_intf_priv->consumer_thread_sts = TNL_THRD_RUNNING;
			continue;
		}

		/* We got some data but not enough */
		t_intf_priv->consumer_thread_sts = TNL_THRD_WAIT2;
		rc = wait_event_timeout(t_intf_priv->consumer_wq,
			((circ_cnt(circ) << 2) > sz_used) ||
			kthread_should_stop(),
			HZ);

		t_intf_priv->consumer_thread_sts = TNL_THRD_RUNNING;
		sz_used = circ_cnt(circ) << 2;
		if (!rc && sz_used < size) {
			pr_err_ratelimited(
				"Consume invalid packet in circ queue");
			circ_peek_finish(circ, sz_used);
		}
	}

	t_intf_priv->consumer_thread_sts = TNL_THRD_EXIT;

	return 0;
}

/*
 * Request to setup a tunnel to producer
 */
static int tunnel_setup_an_endpoint(struct iaxxx_tunnel_client *client,
				uint32_t src,
				uint32_t mode, uint32_t encode)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct iaxxx_tunnel_ep *tnl_src_node;
	int id;
	int rc = 0;

	pr_debug("dev id-%d: src 0x%x", priv->dev_id, src);

	mutex_lock(&t_intf_priv->tunnel_dev_lock);
	id = tunnel_find_id(t_intf_priv, src, mode, encode, true);
	if (id < 0) {
		rc = -EINVAL;
		goto exit;
	}

	/* Allocate tunnel endpoint list for the tunneling */
	tnl_src_node =
	iaxxx_kvmalloc(sizeof(struct iaxxx_tunnel_ep), __GFP_ZERO);
	if (!tnl_src_node) {
		rc = -ENOMEM;
		goto exit;
	}

	pr_debug("dev id-%d: id found %d already there :%d",
		priv->dev_id, id, atomic_read(&t_intf_priv->src_enable_id[id]));

	/* check if this is a new tunnel src endpoint */
	if (atomic_read(&t_intf_priv->src_enable_id[id]) == 0) {
		tnl_src_node->tnl_ep.tunlEP = id;
		tnl_src_node->tnl_ep.tunlSrc = src;
		tnl_src_node->tnl_ep.tunlMode = mode;
		tnl_src_node->tnl_ep.tunlEncode = encode;

		/* Add tunnel endpoint to tunnel src list */
		iaxxx_tunnel_src_list_add_endpoint(t_intf_priv, tnl_src_node);

		atomic_set(&t_intf_priv->src_enable_id[id], 1);

	} else if (iaxxx_tunnel_src_list_find_endpoint_node(
			t_intf_priv, id, tnl_src_node)) {
		pr_info("dev id-%d: tunnel src_id(0x%x) already exist!",
			priv->dev_id, tnl_src_node->tnl_ep.tunlSrc);

	} else { /* we should not be here, but a safety check */
		pr_err("dev id-%d: failed to find tunnel enpoint id: %d",
						priv->dev_id, id);
		rc = -EIO;
		goto exit;
	}

	pr_info("dev id-%d: setup tnl%d, src%x, mode%s, enc%s",
			priv->dev_id, id,
			tnl_src_node->tnl_ep.tunlSrc & 0xffff,
			(tnl_src_node->tnl_ep.tunlMode
			== TNL_SYNC_MODE) ? "SYNC" : "ASYNC",
			(tnl_src_node->tnl_ep.tunlEncode
			== TNL_ENC_AFLOAT) ? "AFLOAT" :
			(tnl_src_node->tnl_ep.tunlEncode
			== TNL_ENC_Q15) ? "Q15" : "Other");

	set_bit(id, &client->tid_flag);
	if (atomic_inc_return(&t_intf_priv->tunnel_ref_cnt[id]) == 1) {
		pr_info("dev id-%d: id found %d ref count :%d",
			priv->dev_id, id,
			atomic_read(&t_intf_priv->tunnel_ref_cnt[id]));

		set_bit(id, &t_intf_priv->flags);
		/* Set flag if this tunnel id is in sync mode */
		if (mode == TNL_SYNC_MODE)
			set_bit(id, &t_intf_priv->flags_sync_mode);

		/* call the chiplevel tunnel setup */
		rc = iaxxx_tunnel_setup(priv,
			tnl_src_node->tnl_ep.tunlEP,
			tnl_src_node->tnl_ep.tunlSrc & 0xffff,
			tnl_src_node->tnl_ep.tunlMode,
			tnl_src_node->tnl_ep.tunlEncode);
		if (rc < 0) {
			pr_err("dev id-%d: iaxxx_tunnel_setup failed",
							priv->dev_id);
			/*
			 * tunnel setup has failed but enable bit needs to be
			 * cleared with other ep config registers. if not this
			 * will resutl in failure on future update blocks.
			 */
			iaxxx_tunnel_terminate(priv, id);
			goto setup_err;
		}

		wake_up(&t_intf_priv->producer_wq);

setup_err:
		/* Init sequence number and update active cnt*/
		t_intf_priv->tunnel_seq_no[id] = 0;
		t_intf_priv->tunnels_active_count++;
	}

	pr_debug("dev id-%d: tid: %x src: %x client flag: %lx global flag: %lx",
		priv->dev_id, id, src, client->tid_flag, t_intf_priv->flags);

exit:
	mutex_unlock(&t_intf_priv->tunnel_dev_lock);
	return rc;
}

int tunnel_setup_an_endpoint_stub(void *client_data,
				uint32_t src, uint32_t mode, uint32_t encode)
{
	int ret;
	struct iaxxx_tunnel_client *client =
				(struct iaxxx_tunnel_client *)client_data;

	ret = tunnel_setup_an_endpoint(client, src, mode, encode);
	if (ret)
		pr_err("Unable to setup tunnel %d", ret);

	return ret;
}
EXPORT_SYMBOL(tunnel_setup_an_endpoint_stub);

static int tunnel_term_core(struct iaxxx_tunnel_client *client, uint32_t mode,
			    int id)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
					(struct iaxxx_priv *)t_intf_priv->priv;

	int rc = 0;

	mutex_lock(&t_intf_priv->tunnel_dev_lock);

	if (!test_and_clear_bit(id, &client->tid_flag) ||
		!atomic_read(&t_intf_priv->tunnel_ref_cnt[id])) {
		rc = -EINVAL;
		goto exit;
	}

	if (atomic_dec_return(&t_intf_priv->tunnel_ref_cnt[id]) == 0) {
		clear_bit(id, &t_intf_priv->flags);
		if (mode == TNL_SYNC_MODE)
			clear_bit(id, &t_intf_priv->flags_sync_mode);

		/* park the producer thread  before tunnel stop */
		pr_debug("dev id-%d: park producer_thread tid[%d]",
							priv->dev_id, id);
		kthread_park(t_intf_priv->producer_thread);

		/* terminate the tunnel */
		pr_info("dev id-%d: terminate tunnel %d", priv->dev_id, id);
		if (iaxxx_tunnel_terminate(priv, id))
			pr_err("dev id-%d: iaxxx_tunnel_terminate failed",
								priv->dev_id);

		if (t_intf_priv->tunnels_active_count > 0)
			t_intf_priv->tunnels_active_count--;
		else
			pr_err("dev id-%d: tunnel active count is going -ve",
					priv->dev_id);

		/* unpark the producer thread */
		kthread_unpark(t_intf_priv->producer_thread);
		pr_debug("dev id-%d: unpark producer_thread tid[%d]",
							priv->dev_id, id);

		if (!t_intf_priv->tunnels_active_count)
			t_intf_priv->tunnel_first_attach = false;

		iaxxx_tunnel_src_list_del_endpoint(t_intf_priv, id);

		atomic_set(&t_intf_priv->src_enable_id[id], 0);
	}

	pr_info("dev id-%d: id=%d, client->tid_flag=0x%lx, t_intf_priv->flags=0x%lx",
			priv->dev_id, id, client->tid_flag, t_intf_priv->flags);

	if (atomic_read(&t_intf_priv->src_enable_id[id])) {
		pr_err("dev id-%d: Source enable id[%d] not cleared (%d)\n",
			priv->dev_id, id,
			atomic_read(&t_intf_priv->tunnel_ref_cnt[id]));
		rc = -EIO;
	}

	pr_info("dev id-%d: tunnel id %d state %d", id,
		priv->dev_id, atomic_read(&t_intf_priv->src_enable_id[id]));

exit:
	mutex_unlock(&t_intf_priv->tunnel_dev_lock);
	return rc;
}

/*
 * Terminate a tunnel for a client (Producer will terminate the tunnel)
 */
static int tunnel_term_an_endpoint(struct iaxxx_tunnel_client *client,
				uint32_t src, uint32_t mode, uint32_t encode)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	int id;

	pr_info("tunnel src = 0x%x", src);
	id = tunnel_find_id(t_intf_priv, src, mode, encode, false);
	if (id < 0)
		return -EINVAL;

	return tunnel_term_core(client, mode, id);
}

int tunnel_term_an_endpoint_stub(void *client_data,
				uint32_t src, uint32_t mode, uint32_t encode)
{
	struct iaxxx_tunnel_client *client =
				(struct iaxxx_tunnel_client *)client_data;
	int ret;

	ret = tunnel_term_an_endpoint(client, src, mode, encode);
	if (ret)
		pr_err("Unable to terminate tunnel %d", ret);

	return ret;
}
EXPORT_SYMBOL(tunnel_term_an_endpoint_stub);

/*
 * Terminate all the tunnels for a client (Producer will terminate the tunnel)
 */
static int tunnel_term_all_endpoints(struct iaxxx_tunnel_client *client)
{
	int i;

	pr_info("called");
	for (i = TNL0; i < TNLMAX; ++i)
		tunnel_term_core(client, TNL_SYNC_MODE, i);

	return 0;
}

ssize_t _tunneling_read_to_alsa_tnl_bfr(struct iaxxx_priv *priv,
				void *client_data, void *buf, size_t count)
{
	struct iaxxx_tunnel_client *client =
				(struct iaxxx_tunnel_client *)client_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_circ_buf *user_circ = &client->user_circ;
	unsigned int copied;
	int ret;

	if (!circ_cnt(user_circ)) {
		if (!(wait_event_interruptible_timeout(client->wq,
						circ_cnt(user_circ), HZ))) {
			pr_err("dev id-%d: Timed out (tnl magic err %ld)",
					priv->dev_id,
					t_intf_priv->tunnel_magic_errcnt);
			pr_err("dev id-%d: Producer thrd Sts %d, consumer thrd Sts %d",
					priv->dev_id,
					t_intf_priv->producer_thread_sts,
					t_intf_priv->consumer_thread_sts);
			return 0;
		}
	}
	ret = circ_to_user(user_circ, buf, count, &copied);

	return ret ? ret : copied;
}
EXPORT_SYMBOL(_tunneling_read_to_alsa_tnl_bfr);

static ssize_t tunneling_read(struct file *filp, char __user *buf,
			      size_t count, loff_t *f_pos)
{
	struct iaxxx_tunnel_client *client = filp->private_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_circ_buf *user_circ = &client->user_circ;
	struct iaxxx_priv * const priv =
					(struct iaxxx_priv *)t_intf_priv->priv;
	unsigned int copied;
	int ret;

	if (!circ_cnt(user_circ)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (!(wait_event_interruptible_timeout(client->wq,
						circ_cnt(user_circ), HZ))) {
			pr_err_ratelimited(
				"dev id-%d: Timed out in tunne read (tnl magic err %ld)",
				priv->dev_id, t_intf_priv->tunnel_magic_errcnt);
			pr_err_ratelimited(
				"dev id-%d: Producer thrd Sts %d, consumer thrd Sts %d",
				priv->dev_id, t_intf_priv->producer_thread_sts,
				t_intf_priv->consumer_thread_sts);
			return 0;
		}
	}
	ret = circ_to_user(user_circ, (void *)buf, count, &copied);

	return ret ? ret : copied;
}

int _tunneling_open_stub(struct iaxxx_priv *priv, void **client_ptr)
{
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;
	struct iaxxx_tunnel_client **client_data =
				(struct iaxxx_tunnel_client **)client_ptr;
	struct iaxxx_tunnel_client *client;
	int rc;

	priv = (struct iaxxx_priv *)t_intf_priv->priv;
	if (!iaxxx_is_firmware_ready(priv))
		return -EIO;

	if (!t_intf_priv) {
		pr_err("dev id-%d: Unable to fetch tunnel private data",
								priv->dev_id);
		return -ENODEV;
	}

	*client_data = iaxxx_kvmalloc(sizeof(struct iaxxx_tunnel_client),
						__GFP_ZERO | __GFP_NOWARN);
	client = *client_data;
	if (!client)
		return -ENOMEM;

	rc = tunneling_attach_client(t_intf_priv, client);
	if (rc) {
		kvfree(*client_data);
		return rc;
	}

	init_waitqueue_head(&client->wq);

	return rc;
}
EXPORT_SYMBOL(_tunneling_open_stub);

static int tunneling_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_client *client;
	struct iaxxx_priv *priv;
	int rc;

	pr_info("tunneling device open called");

	if ((inode)->i_cdev == NULL) {
		pr_err("retrieve cdev from inode failed");
		return -ENODEV;
	}

	t_intf_priv = container_of((inode)->i_cdev,
				struct iaxxx_tunnel_data, tunnel_cdev.cdev);
	if (t_intf_priv == NULL) {
		pr_err("Unable to fetch tunnel private data");
		return -ENODEV;
	}

	priv = (struct iaxxx_priv *)t_intf_priv->priv;
	rc = _tunneling_open_stub(priv, (void **)&client);
	if (rc)
		return rc;

	filp->private_data = client;

	priv->is_wifi_wakeup = false;
	gpio_set_value(priv->tvonoff_gpio, 1);

	return 0;
}

/* confirm tunnel parameters received from the caller */
bool is_valid_tnl_cfg_params(uint32_t tunlSrc, uint32_t tunlEncode,
							uint32_t tunlMode)
{
	/* validate the tunnel parameters */
	if (tunlSrc > (IAXXX_OUT_TNL_GRP_CONNECT_SOURCE_ID_MASK >>
			IAXXX_OUT_TNL_GRP_CONNECT_SOURCE_ID_POS) ||
		tunlEncode >
			(IAXXX_OUT_TNL_GRP_TNL_CTRL_OUTPUT_ENCODING_MASK >>
			IAXXX_OUT_TNL_GRP_TNL_CTRL_OUTPUT_ENCODING_POS) ||
		tunlMode > (IAXXX_OUT_TNL_GRP_TNL_CTRL_MODE_MASK >>
				IAXXX_OUT_TNL_GRP_TNL_CTRL_MODE_POS)) {
		pr_err("invalid tunnel parameter received: "
			"tunlSrc: 0x%x, tunlMode: 0x%x, tunlEncode: 0x%x",
					tunlSrc, tunlMode, tunlEncode);
		return false;
	}

	return true;
}
EXPORT_SYMBOL(is_valid_tnl_cfg_params);

static long tunnel_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct iaxxx_tunnel_client * const client =
			(struct iaxxx_tunnel_client *)filp->private_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct tunlMsg msg;
	uint32_t tnl_buf_size;

	int ret = 0;

	if (_IOC_TYPE(cmd) != IAXXX_TNL_IOCTL_MAGIC)
		return -ENOTTY;

	if (!priv) {
		pr_err("Unable to fetch tunnel private data");
		return -EINVAL;
	}
	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL");
		return -EINVAL;
	}

	if (!iaxxx_is_firmware_ready(priv)) {
		dev_err(priv->dev, "%s: FW state not valid", __func__);
		return -EIO;
	}

	if (arg != 0 && _IOC_DIR(cmd) == (_IOC_READ | _IOC_WRITE)
		&& _IOC_SIZE(cmd) == sizeof(struct tunlMsg)) {
		if (copy_from_user(&msg, (void __user *)arg,
					sizeof(struct tunlMsg))) {
			pr_err("dev id-%d: parameter copy from user failed",
								priv->dev_id);
			return -EFAULT;
		}

		if (!is_valid_tnl_cfg_params(msg.tunlSrc, msg.tunlEncode,
							msg.tunlMode))
			return -EINVAL;
	}

	switch (cmd) {
	case TUNNEL_SETUP:
		ret = tunnel_setup_an_endpoint(client, msg.tunlSrc,
					       msg.tunlMode, msg.tunlEncode);
		if (ret) {
			pr_err("dev id-%d: Unable to setup tunnel %d",
							priv->dev_id, ret);
			return	-EINVAL;
		}
		break;
	case TUNNEL_TERMINATE:
		ret = tunnel_term_an_endpoint(client, msg.tunlSrc, msg.tunlMode,
						msg.tunlEncode);
		if (ret) {
			pr_err("dev id-%d: Unable to terminate tunnel %d",
							priv->dev_id, ret);
			ret = -EINVAL;
		}
		break;
	case TUNNEL_PAUSE:
		pr_info("dev id-%d: Tunnel Pause request", priv->dev_id);
		ret = kthread_park(t_intf_priv->producer_thread);
		t_intf_priv->producer_thread_sts = TNL_THRD_PARK;
		if (ret)
			pr_err("dev id-%d: Kthread park fail %d",
							priv->dev_id, ret);
		break;
	case TUNNEL_RESUME:
		pr_info("dev id-%d: Tunnel Resume request", priv->dev_id);
		t_intf_priv->producer_thread_sts = TNL_THRD_UNPARK;
		kthread_unpark(t_intf_priv->producer_thread);
		break;
	case TUNNEL_GET_OUTPUT_BUF_SIZE:
		if (copy_from_user(&tnl_buf_size, (void __user *)arg,
						sizeof(tnl_buf_size))) {
			return -EFAULT;
		}

		ret = iaxxx_get_tunnel_output_buffer_size(priv, &tnl_buf_size);
		if (ret) {
			pr_err("dev id-%d: Get tunnel output buffer size fail",
								priv->dev_id);
			return ret;
		}

		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &tnl_buf_size,
						sizeof(tnl_buf_size))) {
			ret = -EFAULT;
		}
		break;
	case TUNNEL_SET_OUTPUT_BUF_SIZE:
		if (copy_from_user(&tnl_buf_size, (void __user *)arg,
						sizeof(tnl_buf_size))) {
			return -EFAULT;
		}

		ret = iaxxx_set_tunnel_output_buffer_size(priv, tnl_buf_size);
		if (ret)
			pr_err("dev id-%d: Set tunnel output buffer size fail",
								priv->dev_id);

		break;
	default:
		pr_err("dev id-%d: Invalid ioctl command received 0x%x",
							priv->dev_id, cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long tunnel_compat_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	return tunnel_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

int _tunnel_release_stub(struct iaxxx_priv *priv, void *client_data)
{
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;
	struct iaxxx_tunnel_ep arr_tnl_src_node[TNLMAX];
	struct iaxxx_tunnel_ep *tnl_src_node;
	int no_tunnel_ep, i;
	uint32_t tunlEP;
	uint64_t sum = 0;
	int timeout = 50;

	if (!t_intf_priv) {
		pr_err("dev id-%d: Unable to fetch tunnel private data",
							priv->dev_id);
		return -ENODEV;
	}

	if (client_data) {
		tunnel_term_all_endpoints(client_data);
		tunneling_detach_client(t_intf_priv, client_data);

		kvfree(client_data);
	}

	no_tunnel_ep = iaxxx_tunnel_src_list_get_all_endpoint_nodes(
			t_intf_priv, arr_tnl_src_node, TNLMAX);

	if (no_tunnel_ep)
		pr_info("dev id-%d: Tunnel ID\tTunnel Src ID\tSeqno Error Count\n",
								priv->dev_id);

	for (i = 0; i < no_tunnel_ep ; i++) {
		tnl_src_node = &arr_tnl_src_node[i];
		tunlEP = tnl_src_node->tnl_ep.tunlEP;

		pr_info("    %02d    \t0x%04x\t\t\t%d\n",
			tunlEP, tnl_src_node->tnl_ep.tunlSrc,
			t_intf_priv->tunnel_seq_err[tunlEP]);
		sum +=
			t_intf_priv->tunnel_seq_err[tunlEP];
	}

	if (sum)
		pr_info("dev id-%d: Total Seq error: %llu\n",
							priv->dev_id, sum);

	if (t_intf_priv->tunnel_magic_errcnt)
		pr_info("dev id-%d: tunnel magic_errcnt %ld",
				priv->dev_id, t_intf_priv->tunnel_magic_errcnt);

	while (t_intf_priv->producer_thread_sts != TNL_THRD_WAIT2 && timeout--)
		usleep_range(20 * 1000, 21 * 1000);
	if (t_intf_priv->producer_thread_sts != TNL_THRD_WAIT2 && timeout == -1)
		pr_err("dev id-%d: Timeout to wait for producer_thread_sts(%u)",
				priv->dev_id, t_intf_priv->producer_thread_sts);
	return 0;
}
EXPORT_SYMBOL(_tunnel_release_stub);

static int tunnel_release(struct inode *inode, struct file *filp)
{
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_priv *priv;

	pr_info("tunneling device release called");

	if (inode == NULL) {
		pr_err("invalid inode pointer");
		goto error;
	}

	if ((inode)->i_cdev == NULL) {
		pr_err("invalid cdev pointer in inode");
		goto error;
	}

	t_intf_priv = container_of((inode)->i_cdev, struct iaxxx_tunnel_data,
					tunnel_cdev.cdev);
	if (t_intf_priv == NULL) {
		pr_err("unable to fetch tunnel private data");
		goto error;
	}

	priv = (struct iaxxx_priv *)t_intf_priv->priv;

	_tunnel_release_stub(priv, (void *)filp->private_data);

	filp->private_data = NULL;

error:
	return 0;
}

static unsigned int tunnel_poll(struct file *filep,
	struct poll_table_struct *wait)
{
	struct iaxxx_tunnel_client *const client = filep->private_data;
	unsigned int mask = 0;

	poll_wait(filep, &client->wq, wait);
	if (circ_cnt(&client->user_circ))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations tunneling_fops = {
	.owner = THIS_MODULE,
	.open = tunneling_open,
	.read = tunneling_read,
	.unlocked_ioctl	= tunnel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= tunnel_compat_ioctl,
#endif
	.release = tunnel_release,
	.poll = tunnel_poll,
};

/*
 * Show tunnel status, enable, num of clients etc.
 */
static ssize_t iaxxx_tunnel_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_ep arr_tnl_src_node[TNLMAX];
	struct iaxxx_tunnel_ep *tnl_src_node;
	int index;
	int no_tunnel_ep, i;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	no_tunnel_ep = iaxxx_tunnel_src_list_get_all_endpoint_nodes(t_intf_priv,
			arr_tnl_src_node, TNLMAX);

	scnprintf(buf, PAGE_SIZE,
	"Tunnel ID\tSRC\tEnable\tMode\tEncode\tNum of Clients\tNo Packets\tError Packets\n");

	for (i = 0; i < no_tunnel_ep ; i++) {
		int enable = true;

		tnl_src_node = &arr_tnl_src_node[i];
		index = strnlen(buf, PAGE_SIZE);
		scnprintf(&buf[index], PAGE_SIZE - index,
		"    %02d    \t0x%04x\t%d\t%s\t%s\t\t%d\t%lu\t\t%d\n",
		tnl_src_node->tnl_ep.tunlEP,
		tnl_src_node->tnl_ep.tunlSrc & 0xffff,
		enable,
		(tnl_src_node->tnl_ep.tunlMode == TNL_SYNC_MODE) ?
		"SYNC" : "ASYNC",
		((tnl_src_node->tnl_ep.tunlEncode == TNL_ENC_AFLOAT) ?
		"AFLOAT" : (tnl_src_node->tnl_ep.tunlEncode == TNL_ENC_Q15) ?
		"Q15" : "Other"),
		atomic_read(
		&t_intf_priv->tunnel_ref_cnt[tnl_src_node->tnl_ep.tunlEP]),
		t_intf_priv->tunnel_packet_no[tnl_src_node->tnl_ep.tunlEP],
		t_intf_priv->tunnel_seq_err[tnl_src_node->tnl_ep.tunlEP]);
	}

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
		"\nTotal Packets: %lu\n",
		t_intf_priv->tunnel_total_packet_no);
	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
		"\nproducer_thread_sts: %d\n",
		t_intf_priv->producer_thread_sts);
	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
		"\nconsumer_thread_sts: %d\n",
		t_intf_priv->consumer_thread_sts);

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t iaxxx_tunnel_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	if (t_intf_priv->tunnels_enabled)
		return -EPERM;

	/* Clear packet history */
	memset(t_intf_priv->tunnel_packet_no, 0,
		sizeof(t_intf_priv->tunnel_packet_no));
	memset(t_intf_priv->tunnel_seq_err, 0,
		sizeof(t_intf_priv->tunnel_seq_err));
	t_intf_priv->tunnel_total_packet_no = 0;

	return count;
}

static DEVICE_ATTR(tunnel_status, 0600,
		iaxxx_tunnel_status_show, iaxxx_tunnel_status_store);

/*
 * Get the information on producer circular queue
 */
static ssize_t iaxxx_tunnel_circ_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_circ_buf *circ;
	int index;
	int cnt;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	circ = &t_intf_priv->stream_circ;

	cnt = circ_cnt_to_end(circ) << 2;

	scnprintf(buf, PAGE_SIZE,
		"Circular: head %d, tail %d, cnt %d, space %d\n",
		circ->head, circ->tail, cnt, circ_space(circ) << 2);

	index = strnlen(buf, PAGE_SIZE);

	hex_dump_to_buffer(circ->buf + (circ->tail << 2), cnt, 16, 1,
			&buf[index], PAGE_SIZE - index, false);
	return index;
}
static DEVICE_ATTR(tunnel_circ, 0400, iaxxx_tunnel_circ_show, NULL);

/*
 * Show packet header magic error
 */
static ssize_t iaxxx_tunnel_header_errcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE,
		"Packet Header Error Count = %ld\n",
		t_intf_priv->tunnel_magic_errcnt);
}
static DEVICE_ATTR(tunnel_header_errcnt, 0400,
			iaxxx_tunnel_header_errcnt_show, NULL);

/*
 * Show sequence number error
 */
static ssize_t iaxxx_tunnel_seqno_errcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_ep arr_tnl_src_node[TNLMAX];
	struct iaxxx_tunnel_ep *tnl_src_node;
	int index;
	int no_tunnel_ep, i;
	uint64_t sum = 0;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	no_tunnel_ep = iaxxx_tunnel_src_list_get_all_endpoint_nodes(
			t_intf_priv, arr_tnl_src_node, TNLMAX);

	scnprintf(buf, PAGE_SIZE,
		"Tunnel ID\tTunnel Src ID\tSeqno Error Count\n");
	for (i = 0; i < no_tunnel_ep ; i++) {
		index = strnlen(buf, PAGE_SIZE);
		tnl_src_node = &arr_tnl_src_node[i];
		scnprintf(&buf[index], PAGE_SIZE - index,
		"    %02d    \t0x%04x\t\t\t%d\n",
		tnl_src_node->tnl_ep.tunlEP,
		tnl_src_node->tnl_ep.tunlSrc,
		t_intf_priv->tunnel_seq_err[tnl_src_node->tnl_ep.tunlEP]);
		sum +=
		t_intf_priv->tunnel_seq_err[tnl_src_node->tnl_ep.tunlEP];
	}

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
			"Total error: %llu\n", sum);

	return strnlen(buf, PAGE_SIZE);
}
static DEVICE_ATTR(tunnel_seqno_errcnt, 0400,
			iaxxx_tunnel_seqno_errcnt_show, NULL);

/*
 * sysfs attr info
 */
static struct attribute *iaxxx_attrs[] = {
	&dev_attr_tunnel_status.attr,
	&dev_attr_tunnel_circ.attr,
	&dev_attr_tunnel_header_errcnt.attr,
	&dev_attr_tunnel_seqno_errcnt.attr,
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
};

/*
 * Init early stage before firmware download
 */
static int iaxxx_tunnel_dev_init_early(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv = NULL;

	t_intf_priv = devm_kzalloc(priv->dev, sizeof(*t_intf_priv), GFP_KERNEL);
	if (!t_intf_priv)
		return -ENOMEM;

	priv->tunnel_data = t_intf_priv;
	t_intf_priv->priv = priv;

	return 0;
}

static void iaxxx_tunnel_stop(struct iaxxx_tunnel_data *t_intf_priv)
{
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int i;

	t_intf_priv->tunnels_enabled = 0;
	atomic_set(&t_intf_priv->event_occurred, 0);
	atomic_set(&t_intf_priv->tunnel_evt_cnt, 0);
	for (i = 0; i < TNLMAX; i++) {
		t_intf_priv->tunnel_seq_err[i] = 0;
		t_intf_priv->tunnel_seq_no[i] = 0;
		t_intf_priv->tunnel_packet_no[i] = 0;
		atomic_set(&t_intf_priv->tunnel_ref_cnt[i], 0);
		test_and_clear_bit(i, &t_intf_priv->flags_sync_mode);
		test_and_clear_bit(i, &t_intf_priv->flags);
		iaxxx_tunnel_src_list_del_endpoint(t_intf_priv, i);
		atomic_set(&t_intf_priv->src_enable_id[i], 0);
	}
	t_intf_priv->tunnel_total_packet_no = 0;
	t_intf_priv->tunnel_magic_errcnt = 0;
	t_intf_priv->tunnels_active_count = 0;
	t_intf_priv->event_registered = false;

	if (t_intf_priv->producer_thread) {
		kthread_stop(t_intf_priv->producer_thread);
		t_intf_priv->producer_thread_sts = TNL_THRD_STOP;
		t_intf_priv->producer_thread = NULL;
	}
	if (t_intf_priv->consumer_thread) {
		kthread_stop(t_intf_priv->consumer_thread);
		t_intf_priv->consumer_thread_sts = TNL_THRD_STOP;
		t_intf_priv->consumer_thread = NULL;
	}

	circ_init(circ);
}

static int iaxxx_tunnel_recovery(struct iaxxx_tunnel_data *t_intf_priv)
{
	int err = 0;

	/* Recovery is to be done only if recovery event is preceded by
	 * a crash event. This would prevent multiple calls to recovery
	 */

	if (!test_bit(IAXXX_TFLG_FW_CRASH, &t_intf_priv->tunnel_state)) {
		pr_err("Spurious recovery event, returning");
		return 0;
	}

	if (t_intf_priv->producer_thread != NULL) {
		kthread_stop(t_intf_priv->producer_thread);
		t_intf_priv->producer_thread_sts = TNL_THRD_STOP;
		pr_err("t_intf_priv->producer_thread is not NULL");
		t_intf_priv->producer_thread = NULL;
	}

	t_intf_priv->producer_thread = kthread_run(producer_thread,
			t_intf_priv, "iaxxx tunnel producer thread");

	if (IS_ERR(t_intf_priv->producer_thread)) {
		pr_err("Cannot create producer thread");
		err = PTR_ERR(t_intf_priv->producer_thread);
		return err;
	}
	t_intf_priv->producer_thread_sts = TNL_THRD_STARTED;

	if (t_intf_priv->consumer_thread != NULL) {
		pr_err("t_intf_priv->consumer_thread is not NULL");
		kthread_stop(t_intf_priv->consumer_thread);
		t_intf_priv->consumer_thread_sts = TNL_THRD_STOP;
		t_intf_priv->consumer_thread = NULL;
	}

	t_intf_priv->consumer_thread = kthread_run(consumer_thread,
			t_intf_priv, "iaxxx tunnel consumer thread");

	if (IS_ERR(t_intf_priv->consumer_thread)) {
		pr_err("Cannot create consumer thread");
		err = PTR_ERR(t_intf_priv->consumer_thread);
		return err;
	}
	t_intf_priv->consumer_thread_sts = TNL_THRD_STARTED;

	return err;
}

static int iaxxx_tunnel_signal_event(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;

	atomic_inc(&t_intf_priv->event_occurred);
	wake_up(&t_intf_priv->producer_wq);
	return 0;
}

static int iaxxx_notifier_cb(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct iaxxx_tunnel_data *priv = container_of(nb,
			struct iaxxx_tunnel_data, crash_notifier);
	int ret = 0;

	if (priv == NULL) {
		pr_err("No tunnel priv data found");
		return -EINVAL;
	}

	switch (val) {

	case IAXXX_EV_RECOVERY:
	case IAXXX_EV_RELOAD_DONE:
		ret = iaxxx_tunnel_recovery(priv);
		if (!ret)
			clear_bit(IAXXX_TFLG_FW_CRASH,
				&priv->tunnel_state);
		priv->tunnel_first_attach = 0;
		break;

	case IAXXX_EV_RELOAD_START:
	case IAXXX_EV_CRASH:
	case IAXXX_EV_FW_RESET:
		set_bit(IAXXX_TFLG_FW_CRASH, &priv->tunnel_state);
		iaxxx_tunnel_stop(priv);
		break;
	case IAXXX_EV_TUNNEL:
		iaxxx_tunnel_signal_event(priv->priv);
		break;
	}

	return ret;
}

#define FINAL_BUFFER_PLUGIN_OUT_EP  IAXXX_SYSID_PLUGIN_5_OUT_EP_0

static void iaxxx_stop_kw_buffering_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = container_of(work, struct iaxxx_priv,
			stop_kw_buffering_work);
	struct iaxxx_tunnel_data *tunnel_data = priv->tunnel_data;

	tunnel_term_an_endpoint(&tunnel_data->dummy_client,
		FINAL_BUFFER_PLUGIN_OUT_EP,
		TNL_SYNC_MODE, TNL_ENC_Q15);
}

static void iaxxx_stop_keyword_buffering(struct timer_list *t)
{
	struct iaxxx_tunnel_data *tunnel_data =
		from_timer(tunnel_data, t, timer_to_stop_kw_buffering);
	struct iaxxx_priv *priv = tunnel_data->priv;

	iaxxx_work(priv, stop_kw_buffering_work);
}

int iaxxx_start_keyword_buffering(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *tunnel_data = priv->tunnel_data;
	struct iaxxx_tunnel_client *dummy_client = &tunnel_data->dummy_client;
	int ret = 0;
	int id;

	id = tunnel_find_id(tunnel_data,
		FINAL_BUFFER_PLUGIN_OUT_EP, TNL_SYNC_MODE, TNL_ENC_Q15, true);
	if (id < 0)
		return -EINVAL;

	if (atomic_read(&tunnel_data->src_enable_id[id])) {
		pr_info("Tunneling endpoint exists, don't start keyword buffering");
		return 0;
	}

	dummy_client->tunnel_data = tunnel_data;
	ret = tunnel_setup_an_endpoint(dummy_client,
		FINAL_BUFFER_PLUGIN_OUT_EP,
		TNL_SYNC_MODE, TNL_ENC_Q15);
	if (!ret) {
		iaxxx_init_kthread_work(&priv->stop_kw_buffering_work, iaxxx_stop_kw_buffering_work);
		mod_timer(&tunnel_data->timer_to_stop_kw_buffering,
					jiffies + msecs_to_jiffies(1000));
	} else {
		pr_err("Failed to setup the endpoint for buffer plugin (%d)", ret);
	}

	return ret;
}

/*
 * Init remaining stuffs
 */
static int iaxxx_tunnel_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
	struct iaxxx_priv *priv = NULL;
	int err;

	dev_info(dev, "%s: enter, dev->id %d, pdev->id %d\n", __func__,
							dev->id, pdev->id);

	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("No device data found");
		return -EINVAL;
	}

	pr_debug("dev id-%d: initializing tunneling", priv->dev_id);

	dev_set_drvdata(dev, priv);

	/* Init early stage for tunneling */
	err = iaxxx_tunnel_dev_init_early(priv);
	if (err) {
		dev_err(priv->dev,
			"%s: tunnel dev early init failed : %d\n",
							__func__, err);
		goto err_out;
	}

	t_intf_priv = priv->tunnel_data;

	if (priv->tunnel_method == IAXXX_EVENT)
		priv->tunnel_threshold = IAXXX_TUNNEL_THRESHOLD;
	else
		priv->tunnel_threshold = 0;

	t_intf_priv->event_registered = false;
	t_intf_priv->dev = dev;

	mutex_init(&t_intf_priv->tunnel_dev_lock);

	if (iaxxx_prod_buf) {
		/* If reserved memory exists, use it */
		t_intf_priv->stream_circ.buf = phys_to_virt(iaxxx_prod_buf);
		t_intf_priv->stream_circ.size =
			iaxxx_prod_buf_size >> 2;
		pr_debug("dev id-%d: carvout %p, %zd",
			priv->dev_id, t_intf_priv->stream_circ.buf,
			iaxxx_prod_buf_size);
	} else {
		/* If no reserved, allocate default memory */
		t_intf_priv->stream_circ.buf = devm_kmalloc(priv->dev,
						PBUFF_SIZE, GFP_KERNEL);
		t_intf_priv->stream_circ.size = PBUFF_SIZE >> 2;
		iaxxx_prod_buf_size = PBUFF_SIZE;
	}

	if (!t_intf_priv->stream_circ.buf) {
		err = -ENOMEM;
		goto error_circ_buf;
	}

	/* Initialize client structure */
	INIT_LIST_HEAD(&t_intf_priv->list);
	spin_lock_init(&t_intf_priv->lock);
	spin_lock_init(&t_intf_priv->src_list_lock);

	err = iaxxx_cdev_create(&t_intf_priv->tunnel_cdev, dev,
		&tunneling_fops, t_intf_priv, priv->dev_id, IAXXX_CDEV_TUNNEL);
	if (err) {
		pr_err("dev id-%d: creating the char device failed : %d",
							priv->dev_id, err);
		err = -EIO;
		goto error_cdev;
	}


	INIT_LIST_HEAD(&t_intf_priv->src_list);

	/* Create producer thread */
	init_waitqueue_head(&t_intf_priv->producer_wq);
	t_intf_priv->producer_thread = kthread_run(producer_thread,
			t_intf_priv, "iaxxx tunnel producer thread");
	if (IS_ERR(t_intf_priv->producer_thread)) {
		pr_err("dev id-%d: Cannot create producer thread",
							priv->dev_id);
		err = PTR_ERR(t_intf_priv->producer_thread);
		goto error_producer_thread;
	}
	t_intf_priv->producer_thread_sts = TNL_THRD_STARTED;

	/* Create consumer thread */
	init_waitqueue_head(&t_intf_priv->consumer_wq);
	t_intf_priv->consumer_thread = kthread_run(consumer_thread,
			t_intf_priv, "iaxxx tunnel consumer thread");
	if (IS_ERR(t_intf_priv->consumer_thread)) {
		pr_err("dev id-%d: Cannot create consumer thread",
						priv->dev_id);
		err = PTR_ERR(t_intf_priv->consumer_thread);
		goto error_consumer_thread;
	}
	t_intf_priv->consumer_thread_sts = TNL_THRD_STARTED;

	if (sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group))
		pr_err("dev id-%d: Cannot create tunnel sysfs", priv->dev_id);

	pr_debug("dev id-%d: streaming cdev initialized.", priv->dev_id);

	t_intf_priv->crash_notifier.notifier_call = iaxxx_notifier_cb;
	err = iaxxx_fw_notifier_register
		(priv->dev, &t_intf_priv->crash_notifier);
	if (err) {
		dev_err(dev, "%s: register for fw notifier failed : %d\n",
				__func__, err);
		goto error_crash_notifier;
	}

	timer_setup(&t_intf_priv->timer_to_stop_kw_buffering,
		iaxxx_stop_keyword_buffering, 0);

	return 0;

error_crash_notifier:
	kthread_stop(t_intf_priv->consumer_thread);
	t_intf_priv->consumer_thread_sts = TNL_THRD_STOP;
error_consumer_thread:
	kthread_stop(t_intf_priv->producer_thread);
	t_intf_priv->producer_thread_sts = TNL_THRD_STOP;
error_producer_thread:
error_cdev:
error_circ_buf:
	if (err && t_intf_priv)
		kvfree(t_intf_priv);
err_out:
	return err;
}

static int iaxxx_tunnel_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv = NULL;
	struct iaxxx_tunnel_data *t_intf_priv = NULL;

	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;

	iaxxx_fw_notifier_unregister(priv->dev, &t_intf_priv->crash_notifier);

	iaxxx_cdev_destroy(&t_intf_priv->tunnel_cdev, IAXXX_CDEV_TUNNEL);

	sysfs_remove_group(&priv->dev->kobj, &iaxxx_attr_group);
	kthread_stop(t_intf_priv->producer_thread);
	t_intf_priv->producer_thread_sts = TNL_THRD_STOP;
	kthread_stop(t_intf_priv->consumer_thread);
	t_intf_priv->consumer_thread_sts = TNL_THRD_STOP;
	mutex_destroy(&t_intf_priv->tunnel_dev_lock);
	del_timer(&t_intf_priv->timer_to_stop_kw_buffering);
	kvfree(t_intf_priv);
	return 0;
}

#if IS_BUILTIN(CONFIG_MFD_IAXXX)
static int __init iaxxx_reserve_audio_buffer(char *p)
{
	char *old_p = p;
	unsigned long start, size;

	if (!p)
		return 0;

	size = memparse(p, &p);

	/* Check if value and power of 2 */
	if (p == old_p || !is_power_of_2(size))
		return 0;

	if (*p != '$')
		return 0;

	start = memparse(p + 1, &p);

	if (!iaxxx_prod_buf &&
		!memblock_reserve(start, size)) {
		iaxxx_prod_buf = start;
		iaxxx_prod_buf_size = size;
	}

	return 0;
}
#endif

static const struct of_device_id iaxxx_tunnel_dt_match[] = {
	{.compatible = "knowles,iaxxx-tunnel-celldrv"},
	{}
};

static struct platform_driver iaxxx_tunnel_driver = {
	.probe  = iaxxx_tunnel_dev_probe,
	.remove = iaxxx_tunnel_dev_remove,
	.driver = {
		.name = "iaxxx-tunnel-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_tunnel_dt_match,
	},
};

static int __init iaxxx_tunnel_init(void)
{
	int ret;

	ret = platform_driver_register(&iaxxx_tunnel_driver);
	if (ret)
		pr_err("register tunnel platform driver failed : %d", ret);
	return ret;
}

static void __exit iaxxx_tunnel_exit(void)
{
	platform_driver_unregister(&iaxxx_tunnel_driver);
}

#if IS_BUILTIN(CONFIG_MFD_IAXXX)
early_param("audio_buffer", iaxxx_reserve_audio_buffer);
#endif

module_init(iaxxx_tunnel_init);
module_exit(iaxxx_tunnel_exit);
MODULE_LICENSE("GPL");
