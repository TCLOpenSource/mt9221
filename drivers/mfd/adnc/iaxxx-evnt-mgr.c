/*
 * iaxxx-evnt-mgr.c -- IAxxx Event Manager Service
 *
 * Copyright 2016 Knowles Corporation
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

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include "iaxxx.h"
#include "iaxxx-tunnel-priv.h"

#define IAXXX_EVT_SRC_INFO_SIZE	20

struct iaxxx_event_subscription {
	u16 event_dst;
	void *userdata;
	iaxxx_evnt_mgr_callback event_handler;
};

static void iaxxx_add_event(struct iaxxx_priv *priv,
		const struct iaxxx_event *evt)
{
	struct device *dev = priv->dev;
	struct iaxxx_evt_queue *eq = priv->event_queue;
	int w_index;

	eq->w_index++;
	w_index = eq->w_index;
	if (w_index == IAXXX_MAX_EVENTS) {
		dev_info(dev, "%s: Event Index reach last\n", __func__);
		w_index = 0;
		eq->w_index = w_index;
	}

	if (eq->w_index == eq->r_index) {
		dev_info(dev, "%s: Event buffer is full\n", __func__);
		eq->r_index++;
	}

	eq->event_info[w_index].event_id = evt->event_id;
	eq->event_info[w_index].data = evt->src_opaque;
	eq->event_info[w_index].event_src = evt->event_src;
	if (eq->r_index == IAXXX_MAX_EVENTS)
		eq->r_index = 0;
	dev_dbg(dev, "%s: written index %d\n", __func__, eq->w_index);
}

/*
 * uevent format:
 *   "ACTION=IAXXX_VQ_EVENT\0
 *    EVENT_ID=1\0
 *    EVENT_DATA=12345678\0"
 */
int iaxxx_event_handler(struct iaxxx_priv *priv, struct iaxxx_event *evt)
{
	struct device *dev = priv->dev;
	char action[IAXXX_MAX_UEVENT_STRING_LENGTH];
	char eid[14];
	char esrc[IAXXX_EVT_SRC_INFO_SIZE];
	char edata[20];
	char *event[5] = {action, eid, edata, esrc, NULL};
	int ret = 0;

	if (WARN_ON(!evt))
		return -EINVAL;

	dev_info(dev, "%s:src:0x%04x id:0x%04x src_opq=0x%08x dst_opq=0x%08x\n",
			__func__, evt->event_src, evt->event_id,
			evt->src_opaque, evt->dst_opaque);

	/* Check for tunnel event and notify producer thread*/
	if (evt->event_src == IAXXX_SYSID_TUNNEL_EVENT) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_TUNNEL, NULL);
		return 0;
	}

	if (evt->event_src == IAXXX_CORE_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_BOOT_COMPLETE_EVENT_ID) {
		dev_info(dev, "FW boot complete event %s: src:0x%.04x\n",
						__func__, evt->event_src);
		priv->boot_completed = true;
		wake_up(&priv->boot_wq);
		return ret;
	}

	if (evt->event_src == IAXXX_CORE_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_WAKEUP_EVENT_ID) {
		dev_info(dev, "%s: FW chip wakeup event: src:0x%.04x\n",
						__func__, evt->event_src);
		return ret;
	}

	if (evt->event_src == IAXXX_CORE_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_CRASH_EVENT_ID) {
		dev_err(dev, "FW crash %s: src:0x%.04x, proc id:0x%.04X\n",
			__func__, evt->event_src, evt->src_opaque);
		iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT);
		return ret;
	}

	dev_info(dev,
		"%s:EVENT src:0x%04x id:0x%04x src_opq=0x%08x dst_opq=0x%08x\n",
		__func__, evt->event_src, evt->event_id,
		evt->src_opaque, evt->dst_opaque);

	if ((evt->event_id == EVENT_ID_KW_ID)
			&& (priv->isr_from_resume || priv->is_wifi_wakeup)) {
		snprintf(eid, sizeof(eid), "EVENT_ID=%hx", evt->event_id);
		snprintf(edata, sizeof(edata), "EVENT_DATA=0x%x", evt->src_opaque);
		snprintf(esrc, sizeof(edata), "EVENT_SOURCE=0x%x", evt->event_src);

		mutex_lock(&priv->event_queue_lock);
		iaxxx_add_event(priv, evt);
		mutex_unlock(&priv->event_queue_lock);

		snprintf(action, sizeof(action), "ACTION=IAXXX_VQ_EVENT-%d",
									priv->dev_id);
		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, event);

		iaxxx_start_keyword_buffering(priv);
	}
	if (evt->event_id < IAXXX_MAX_KW_EVENTS)
		priv->kw_detect_count[evt->event_id]++;

	return 0;
}

int iaxxx_evnt_mgr_unsubscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst)
{
	return -ENOTSUPP;
}
EXPORT_SYMBOL(iaxxx_evnt_mgr_unsubscribe);

int iaxxx_evnt_mgr_subscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst,
					u32 dst_opaque, void *userdata,
					iaxxx_evnt_mgr_callback cb_func_ptr)
{
	return -ENOTSUPP;
}
EXPORT_SYMBOL(iaxxx_evnt_mgr_subscribe);
