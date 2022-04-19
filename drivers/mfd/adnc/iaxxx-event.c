/*
 * iaxxx-event.c -- IAxxx events
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

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"

#define BIT_FNAME(N, T)  N ## _ ## T
#define GET_BITS(R, N)	(((R) & BIT_FNAME(N, MASK)) >> BIT_FNAME(N, POS))

/*****************************************************************************
 * iaxxx_core_evt_is_valid_src_id()
 * @brief validate the plugin event scr id
 *
 * @id event scr id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_src_id(uint32_t src_id)
{
	bool ret = true;

	if (src_id > (IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_MASK
					>> IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_src_id);

/*****************************************************************************
 * iaxxx_core_evt_is_valid_dst_id()
 * @brief validate the plugin event dest id
 *
 * @id Plugin event dst id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_dst_id(uint32_t dst_id)
{
	bool ret = true;

	if (dst_id > (IAXXX_EVT_MGMT_EVT_SUB_DST_ID_MASK
					>> IAXXX_EVT_MGMT_EVT_SUB_DST_ID_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_dst_id);

/*****************************************************************************
 * iaxxx_core_evt_is_valid_event_id()
 * @brief validate the plugin event id
 *
 * @id Plugin event event id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_event_id(uint32_t event_id)
{
	bool ret = true;

	if (event_id > (IAXXX_EVT_MGMT_EVT_ID_REG_MASK
					>> IAXXX_EVT_MGMT_EVT_ID_REG_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_event_id);

/*****************************************************************************
 * iaxxx_core_evt_is_valid_dst_opaque()
 * @brief validate the plugin dst opaque
 *
 * @id Plugin event dst opaque
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_evt_is_valid_dst_opaque(uint32_t dst_opaque)
{
	bool ret = true;

	if (dst_opaque > (IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_REG_MASK
				>> IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_REG_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_is_valid_dst_opaque);

/*****************************************************************************
 * iaxxx_core_evt_subscribe()
 * @brief Subscribe to an event
 *
 * @src_id     -   System Id of event source
 * @event_id   -   Event Id
 * @dst_id     -   System Id of event destination
 * @ds_opaque  -   Information sought by destination task when even occurs.
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque)
{
	int ret = -EINVAL;
	int status;
	uint32_t sys_id;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_info(dev,
		"%s: src_id 0x%x, event_id 0x%x, dst_id 0x%x, dst_opq 0x%x",
		__func__, src_id, event_id, dst_id, dst_opaque);

	if (src_id == IAXXX_SYSID_INVALID || dst_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "%s: Invalid System Ids\n", __func__);
		return ret;
	}

	/*
	 * Update all event subscription registers
	 * Event ID, IDS of source and destination, destination opaque
	 */
	mutex_lock(&priv->event_lock);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR, event_id);
	if (ret) {
		dev_err(dev, "%s: evet id addr write failed : %d\n",
								__func__, ret);
		goto evt_err;
	}
	sys_id = ((dst_id << 16) | src_id);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR, sys_id);
	if (ret) {
		dev_err(dev, "%s: evt sub addr write failed : %d\n",
								__func__, ret);
		goto evt_err;
	}
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_ADDR,
			dst_opaque);
	if (ret) {
		dev_err(dev, "%s: evt mgmt dst opq write failed : %d\n",
								__func__, ret);
		goto evt_err;
	}
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
		(1 << IAXXX_EVT_MGMT_EVT_SUB_REQ_POS),
		IAXXX_EVT_MGMT_EVT_SUB_REQ_MASK);
	if (ret) {
		dev_err(dev, "%s: evt mgmt addr write Update bit failed : %d\n",
								__func__, ret);
		goto evt_err;
	}
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "%s: evt mgmt addr Update blk failed : %d\n",
								__func__, ret);

evt_err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_subscribe);

/*****************************************************************************
 * iaxxx_core_evt_unsubscribe()
 * @brief UnSubscribe to an event
 *
 * @src_id     -   System Id of event source
 * @event_id   -   Event Id
 * @dst_id     -   System Id of event destination
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id)
{
	int ret = -EINVAL;
	uint32_t status;
	uint32_t sys_id;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	pr_info("dev id-%d: src_id 0x%x, event_id 0x%x, dst_id 0x%x",
			priv->dev_id, src_id, event_id, dst_id);

	if (src_id == IAXXX_SYSID_INVALID || dst_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "%s: Invalid System Ids\n", __func__);
		return ret;
	}
	/*
	 * Update all event subscription registers
	 * Event ID, Subsystem IDS of source and destination, destination
	 *  opaque
	 */
	mutex_lock(&priv->event_lock);

	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR, event_id);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto err;
	}
	sys_id = ((dst_id << 16) | src_id);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR, sys_id);
	if (ret) {
		dev_err(dev, "%s: write failed : %d\n", __func__, ret);
		goto err;
	}

	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
		(1 << IAXXX_EVT_MGMT_EVT_UNSUB_REQ_POS),
		IAXXX_EVT_MGMT_EVT_UNSUB_REQ_MASK);
	if (ret) {
		dev_err(dev, "%s: evt mgmt addr write Update bit failed : %d\n",
								__func__, ret);
		goto err;
	}
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(dev, "%s: evt mgmt addr Update blk failed : %d\n",
								__func__, ret);
		goto err;
	}

err:
	mutex_unlock(&priv->event_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_unsubscribe);

/*****************************************************************************
 * iaxxx_core_evt_trigger()
 *  @brief Trigger an event. This may be most useful when debugging the system,
 *        but can also be used to trigger simultaneous behavior in entities
 *        which have subscribed, or to simply provide notifications regarding
 *        host status:
 *
 *  @param[in] src_id        SystemId of event source
 *  @param[in] evt_id        Id of event
 *  @param[in] src_opaque    Source opaque to pass with event notification
 *
 *  @return 0 if successful, error number in case of error
 ****************************************************************************/
int iaxxx_core_evt_trigger(struct device *dev,
			uint16_t src_id, uint16_t evt_id, uint32_t src_opaque)
{
	int ret = -EINVAL;
	int status;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s() src_id=%u, evt_id=%u, src_opaque=%u\n",
				__func__, src_id, evt_id, src_opaque);

	if (src_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "%s() Invalid System Ids\n", __func__);
		return ret;
	}

	/* Write System ID(src Id and evt Id) */
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SRC_INFO_ADDR,
			(src_id << IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID_POS)
			| (evt_id << IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID_POS));
	if (ret) {
		dev_err(dev, "%s() Writing source information failed\n",
								__func__);
		return ret;
	}

	/* Write source opaque data */
	ret = regmap_write(priv->regmap,
			IAXXX_EVT_MGMT_EVT_SRC_OPAQUE_ADDR, src_opaque);
	if (ret) {
		dev_err(dev, "%s() Writing source opaque failed\n", __func__);
		return ret;
	}

	/* Set the Trigger Request bit */
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
				IAXXX_EVT_MGMT_EVT_TRIG_REQ_MASK,
				(1 << IAXXX_EVT_MGMT_EVT_TRIG_REQ_POS));
	if (ret) {
		dev_err(dev,
		    "%s() Setting the TRIG_REQ bit in EVT register failed\n",
		    __func__);
		return ret;
	}

	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "%s() Update blk failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(iaxxx_core_evt_trigger);

/*****************************************************************************
 * iaxxx_core_retrieve_event()
 * @brief Retrieve an event notification
 *
 * @event_id	-	Event Id
 * @data	-	Event data
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
		uint32_t *data, uint16_t *event_src)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int r_index = priv->event_queue->r_index;

	if (!priv)
		return ret;

	dev_dbg(dev, "%s: enter\n", __func__);

	mutex_lock(&priv->event_queue_lock);
	r_index++;
	/* Check if there are no events */
	if (r_index == (priv->event_queue->w_index + 1)) {
		dev_err(dev, "%s: Buffer underflow\n", __func__);
		mutex_unlock(&priv->event_queue_lock);
		return ret;
	}
	if (r_index == IAXXX_MAX_EVENTS)
		r_index = 0;

	priv->event_queue->r_index = r_index;
	*event_id = priv->event_queue->event_info[r_index].event_id;
	*data = priv->event_queue->event_info[r_index].data;
	*event_src = priv->event_queue->event_info[r_index].event_src;
	pr_info("dev id-%d: event Src 0x%x event Id %d, data %d read index %d",
			priv->dev_id, *event_src, *event_id, *data, r_index);
	mutex_unlock(&priv->event_queue_lock);
	return 0;
}
EXPORT_SYMBOL(iaxxx_core_retrieve_event);

/*===========================================================================
 * Event Notification
 *===========================================================================
 */

static int iaxxx_next_event_request(struct device *dev,
		struct regmap *regmap,
		struct iaxxx_event *ev)
{
	int rc;
	uint32_t status, data[3];
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	mutex_lock(&priv->event_lock);
	/* Set the next event notification request */
	rc = regmap_update_bits(regmap,
			IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
			IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK,
			0x1 << IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_POS);
	if (rc) {
		dev_err(dev, "set EVT_NEXT_REQ failed : %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_send_update_block_request_with_options(
			dev, IAXXX_BLOCK_0, regmap, 0,
			UPDATE_BLOCK_NO_OPTIONS,
			&status);
	if (rc) {
		dev_err(dev, "EVT_NEXT_REQ failed failed : %d\n", rc);

		/* Clear the request bit */
		regmap_update_bits(regmap,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK, 0);

		goto out;
	}

	/* The EVT_NEXT_REQ bit should have been cleared by firmware */
	rc = regmap_read(regmap, IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
			&status);
	if (rc) {
		dev_err(dev, "read EVT_NEXT_REQ failed : %d\n", rc);
		goto out;
	}

	WARN_ON(status & IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK);
	rc = regmap_bulk_read(regmap,
			IAXXX_EVT_MGMT_EVT_SRC_INFO_ADDR, data,
			ARRAY_SIZE(data));
	if (rc) {
		dev_err(dev, "read EVENT_INFO failed : %d\n", rc);
		goto out;
	}

	ev->event_src = GET_BITS(data[0],
			IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID);
	ev->event_id = GET_BITS(data[0],
			IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID);
	ev->src_opaque = data[1];
	ev->dst_opaque = data[2];

out:
	mutex_unlock(&priv->event_lock);
	return rc;
}

/**
 * iaxxx_get_event_work - Work function to read events from the event queue
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * This work function is scheduled by the ISR when any data is found in the
 * event queue.
 *
 * This function reads the available events from the queue and passes them
 * along to the event manager.
 */
static void iaxxx_get_event_work(struct work_struct *work)
{
	int rc;
	uint32_t count;
	struct iaxxx_event event;
	struct iaxxx_priv *priv = container_of(work, struct iaxxx_priv,
							event_work_struct);
	struct device *dev = priv->dev;

	/*
	 * Since event would have to read anytime,
	 * choose which regmap to use to read the event
	 * based on boot state.
	 */
	struct regmap *regmap = !iaxxx_is_firmware_ready(priv) ?
			priv->regmap_no_pm : priv->regmap;

	dev_info(dev, "%s: enter\n", __func__);

	mutex_lock(&priv->event_work_lock);

	if (priv->core_crashed) {
		dev_dbg(priv->dev, "Core crash event handler called:%d\n",
							priv->core_crashed);
		iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT);
		priv->core_crashed = false;
		goto out;
	}

	/* Read the count of available events */
	rc = regmap_read(regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(dev, "read EVENT_COUNT ADDR failed : %d\n", rc);
		goto out;
	}

	dev_info(priv->dev, "%s: %d event(s) avail\n", __func__, count);

	while (count) {
		rc = iaxxx_next_event_request(dev, regmap, &event);
		if (rc) {
			dev_err(dev, "read event failed : %d\n", rc);
			goto out;
		}

		rc = iaxxx_event_handler(priv, &event);
		if (rc) {
			dev_err(dev, "Event 0x%.04X:0x%.04X not delivered %d\n",
					event.event_src, event.event_id, rc);
			goto out;
		}

		count = 0;

		/* Read the count of available events */
		rc = regmap_read(regmap,
				IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
				&count);
		if (rc) {
			dev_err(dev, "EVENT_COUNT Addr read failed : %d\n",
					rc);
			goto out;
		}

	       dev_info(priv->dev, "%s: %d event(s) avail\n", __func__, count);
	}

out:
	priv->isr_from_resume = false;
	dev_info(dev, "%s: end\n", __func__);
	mutex_unlock(&priv->event_work_lock);
}

/**
 * iaxxx_flush_event_list - Flush function to read events from chip & flush
 *
 * This function reads the available events from the queue and flushes them
 */
int iaxxx_flush_event_list(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t count;
	struct iaxxx_event event;
	struct device *dev = priv->dev;

	dev_dbg(priv->dev, "%s: enter\n", __func__);

	mutex_lock(&priv->event_work_lock);

	/* Read the count of available events */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(dev, "read EVENT_COUNT ADDR failed : %d\n", rc);
		goto out;
	}

	dev_info(priv->dev, "%s: %d event(s) avail\n", __func__, count);

	while (count) {
		dev_info(priv->dev, "%s: %d event(s) available\n",
							__func__, count);
		memset(&event, 0, sizeof(event));
		rc = iaxxx_next_event_request(dev, priv->regmap_no_pm, &event);
		if (rc) {
			dev_err(dev, "read event failed : %d\n", rc);
			goto out;
		}

		dev_dbg(dev,
			"%s: src:0x%x id:0x%x src_opq=0x%08x dst_opq=0x%08x\n",
			__func__, event.event_src, event.event_id,
			event.src_opaque, event.dst_opaque);

		count = 0;

		/* Read the count of available events */
		rc = regmap_read(priv->regmap_no_pm,
				IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
				&count);
		if (rc) {
			dev_err(dev, "EVENT_COUNT Addr read failed : %d\n",
					rc);
			goto out;
		}

	       dev_info(priv->dev, "%s: %d event(s) avail\n", __func__, count);
	}

out:
	mutex_unlock(&priv->event_work_lock);
	return rc;
}

/**
 * iaxxx_event_init - Initialize Event Queue
 *
 * @priv : iaxxx private data
 */
int iaxxx_event_init(struct iaxxx_priv *priv)
{
	int rc;

	priv->event_queue = iaxxx_kvmalloc(sizeof(struct iaxxx_evt_queue), 0);
	if (!priv->event_queue)
		return -ENOMEM;
	priv->event_queue->r_index = -1;
	priv->event_queue->w_index = -1;
	priv->event_workq =
		alloc_workqueue("iaxxx-evnt-wq", WQ_MEM_RECLAIM, 0);
	if (!priv->event_workq) {
		pr_err("dev id-%d: alloc event workq failed", priv->dev_id);
		rc = -ENOMEM;
		kvfree(priv->event_queue);
		return rc;
	}
	/* Set the work queue function as iaxxx_get_event_work() */
	INIT_WORK(&priv->event_work_struct, iaxxx_get_event_work);
	return 0;
}

/**
 * iaxxx_event_exit - Free Event Queue
 *
 * @priv : iaxxx private data
 */
void iaxxx_event_exit(struct iaxxx_priv *priv)
{
	kvfree(priv->event_queue);
	destroy_workqueue(priv->event_workq);
	priv->event_workq = NULL;
}
