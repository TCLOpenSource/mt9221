/*
 * iaxxx-debug.c  --  iaxxx debug support
 *
 * Copyright 2017 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/mfd/adnc/iaxxx-debug-intf.h>
#include <linux/circ_buf.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-debuglog.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-plugin-instance-group.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-regmap.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"

#define GET_INTF_PRIV(iaxxx) \
	((!iaxxx || !iaxxx->intf_priv) ? NULL : iaxxx->intf_priv)
#define IAXXX_MAX_LOG_SIZE	50

#define MAX_ARBS	32
#define NUM_SRB_ARBS	(MAX_ARBS + 1)

#define INVALID_LOG_INFO	0xFFFFFFFF
#define INVALID_BLOCK_ID	0xFFFF

#define DEBUG_LOG_SRB_ARB_DUMP_START 0
#define DEBUG_LOG_DEBUG_LOG_START (DEBUG_LOG_SRB_ARB_DUMP_START + NUM_SRB_ARBS)
#define DEBUG_LOG_TRACE_LOG_START (DEBUG_LOG_DEBUG_LOG_START + MAX_CORES)
#define DEBUG_LOG_REG_ACCESS_LOG_START (DEBUG_LOG_TRACE_LOG_START + MAX_CORES)
#define DEBUG_LOG_MEM_LOG_START (DEBUG_LOG_REG_ACCESS_LOG_START + 1)
#define DEBUG_LOG_MIPS_LOG_START (DEBUG_LOG_MEM_LOG_START + 1)
#define DEBUG_LOG_PLUGIN_LOG_START (DEBUG_LOG_MIPS_LOG_START + 1)

#define CRASH_LOG_CRASH_LOG_START 0
#define CRASH_LOG_DEBUG_LOG_START (CRASH_LOG_CRASH_LOG_START + MAX_CORES)
#define CRASH_LOG_TRACE_LOG_START (CRASH_LOG_DEBUG_LOG_START + MAX_CORES)
#define CRASH_LOG_REG_ACCESS_LOG_START (CRASH_LOG_TRACE_LOG_START + MAX_CORES)
#define CRASH_LOG_MEM_LOG_START (CRASH_LOG_REG_ACCESS_LOG_START + 1)
#define CRASH_LOG_MIPS_LOG_START (CRASH_LOG_MEM_LOG_START + 1)

#define CRASH_LOG_COUNT (CRASH_LOG_MIPS_LOG_START + 1)

enum log_type {
	CRASH_LOG,
	DEBUG_LOG,
	TRACE_LOG,
	FW_REG_HISTORY,
	SRB_ARB_DUMP,
	PLUGIN_DBGLOG,
	MEM_STATS,
	MIPS_STATS
};

struct iaxxx_debug_data {
	struct iaxxx_priv *priv;
	struct iaxxx_cdev raw_cdev;
	struct iaxxx_cdev regdump_cdev;
	struct iaxxx_cdev crashdump_cdev;
	struct iaxxx_cdev debug_log_cdev;
};

static ssize_t raw_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	int rc = 0;
	void *kbuf = NULL;

	pr_debug("called");

	if (!iaxxx || !buf) {
		pr_err("Invalid pointer");
		rc = -EINVAL;
		goto raw_read_err;
	}

	kbuf = memdup_user(buf, count);
	if (!kbuf) {
		pr_err("dev id-%d: copy user data of len: %d failed",
						iaxxx->dev_id, (u32)count);
		rc = -ENOMEM;
		goto raw_read_err;
	}

	rc = iaxxx->raw_ops->read(iaxxx, kbuf, count);
	if (rc < 0) {
		pr_err("dev id-%d: read data failed : %d", iaxxx->dev_id, rc);
		rc = -EIO;
	} else {
		rc = copy_to_user(buf, kbuf, count);
		if (rc) {
			pr_err("dev id-%d: copy response to userspace fail: %d",
							iaxxx->dev_id, rc);
			rc = -EIO;
			goto raw_read_err;
		}

		rc = count;
	}

raw_read_err:
	kfree(kbuf);
	return rc;
}

static ssize_t raw_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	int rc = 0;
	void *kbuf;

	pr_debug("called");

	if (!iaxxx || !buf) {
		pr_err("Invalid pointer");
		rc = -EINVAL;
		goto raw_write_err;
	}

	kbuf = memdup_user(buf, count);
	if (!kbuf) {
		pr_err("dev id-%d: copy user data of len: %d failed",
						iaxxx->dev_id, (u32)count);
		rc = -ENOMEM;
		goto raw_write_err;
	}

	rc = iaxxx->raw_ops->write(iaxxx, kbuf, count);
	if (rc < 0) {
		pr_err("dev id-%d: write data failed : %d", iaxxx->dev_id, rc);
		rc = -EIO;
	} else {
		rc = count;
	}

	kfree(kbuf);

raw_write_err:
	return rc;
}

static long raw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct iaxxx_priv * const iaxxx
			= (struct iaxxx_priv *)file->private_data;
	int ret = 0;
	u8 bus_config;

	pr_debug("called");

	if (_IOC_TYPE(cmd) != IAXXX_RAW_IOCTL_MAGIC)
		return -ENOTTY;

	if (iaxxx == NULL) {
		pr_err("Invalid private pointer");
		return -EINVAL;
	}

	switch (cmd) {
	case IAXXX_BUS_CONFIG:
		bus_config = iaxxx->bus;

		ret = copy_to_user((void __user *)arg, &bus_config,
								sizeof(u8));
		if (ret) {
			pr_err("dev id-%d: copy response to userspace fail: %d",
							iaxxx->dev_id, ret);
			ret = -EFAULT;
		}
		break;
	case IAXXX_RESET:
		if (gpio_is_valid(iaxxx->event_gpio)) {
			iaxxx->is_irq_enabled = false;
			disable_irq(gpio_to_irq(iaxxx->event_gpio));
		}
		ret = iaxxx_device_reset(iaxxx);
		break;
	default:
		pr_err("dev id-%d: Invalid ioctl command received 0x%x",
							iaxxx->dev_id, cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long raw_compat_ioctl(struct file *filp, unsigned int cmd,
						unsigned long arg)
{
	return raw_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int raw_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("raw device open called");

	if ((inode)->i_cdev == NULL) {
		pr_err("retrieve cdev from inode failed");
		return -ENODEV;
	}

	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, raw_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch tunnel private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;

	return 0;
}

static int raw_release(struct inode *inode, struct file *filp)
{
	pr_debug("raw device release called");

	filp->private_data = NULL;
	return 0;
}

static ssize_t regdump_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	char *kbuf;
	int err;

	if (!iaxxx)
		return -EINVAL;
	dev_dbg(iaxxx->dev, "%s: called\n", __func__);

	kbuf = iaxxx_kvmalloc(count, __GFP_ZERO);
	if (!kbuf)
		return -ENOMEM;
	err = copy_from_user(kbuf, buf, count);
	if (err) {
		dev_err(iaxxx->dev, "%s: copy from user failed : %d\n",
								__func__, err);
		kvfree(kbuf);
		return -EINVAL;
	}
	if (!strncmp(kbuf, "clear", (count - 1))) {
		spin_lock(&iaxxx->reg_dump->ring_lock);
		iaxxx->reg_dump->head = 0;
		iaxxx->reg_dump->tail = 0;
		spin_unlock(&iaxxx->reg_dump->ring_lock);
	} else
		dev_err(iaxxx->dev, "%s: Invalid command\n", __func__);

	kvfree(kbuf);
	return count;
}

static ssize_t regdump_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	struct iaxxx_reg_dump_priv *reg_dump;
	struct iaxxx_register_log log;
	char *kbuf;
	ssize_t bytes_written = 0;
	static uint32_t logs_to_read;
	static bool done;
	static uint32_t r_index;

	/* Return if no priv structure */
	if (!iaxxx)
		return -EINVAL;

	dev_dbg(iaxxx->dev, "%s: called\n", __func__);

	if (!iaxxx->reg_dump || !iaxxx->reg_dump->log)
		return -EINVAL;

	/* Register dump read is complete */
	if (done) {
		done = false;
		return 0;
	}
	/* Allocate kernel buffer to read register dump */
	kbuf = iaxxx_kvmalloc(count, __GFP_ZERO);
	if (!kbuf)
		return -ENOMEM;
	reg_dump = iaxxx->reg_dump;
	spin_lock(&reg_dump->ring_lock);
	/* reading first time or last time read is complete */
	if (!logs_to_read) {
		logs_to_read = CIRC_CNT(reg_dump->head, reg_dump->tail,
				IAXXX_BUF_MAX_LEN);
		r_index = reg_dump->tail;
		bytes_written += scnprintf(kbuf + bytes_written, PAGE_SIZE,
				"R/W:\t[TimeStamp]\t0xAddress\t0xValue\n");
	}
	/*
	 * Until kernel buf full or all the logs read,
	 * (count - IAXXX_MAX_LOG_SIZE) check is to avoid
	 * buffer overflow
	 */
	while ((bytes_written < (count - IAXXX_MAX_LOG_SIZE))
					&& logs_to_read > 0) {
		log = reg_dump->log[r_index];

		bytes_written += scnprintf(kbuf + bytes_written,
				PAGE_SIZE, "%c:\t[%lu.%03lu]\t0x%08x\t0x%08x\n",
				log.op == IAXXX_READ ? 'R' : 'W',
				log.timestamp.tv_sec,
				log.timestamp.tv_nsec / (1000 * 1000),
				iaxxx_conv_physical_to_virtual_register_address
				(iaxxx, log.addr),
				log.val);

		/* Increment read index and align with ring buffer boundary */
		r_index++;
		r_index %= IAXXX_BUF_MAX_LEN;
		/* update to remaining logs to read */
		logs_to_read--;
	}
	spin_unlock(&reg_dump->ring_lock);
	/* Copy data to user buffer */
	if (copy_to_user(buf, kbuf, bytes_written)) {
		kvfree(kbuf);
		return -EFAULT;
	}
	/* If no more logs to read, mark read complete */
	if (!logs_to_read)
		done = true;
	kvfree(kbuf);
	return bytes_written;
}

static int regdump_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("enter");

	if ((inode)->i_cdev == NULL) {
		pr_err("retrieve cdev from inode failed");
		return -ENODEV;
	}

	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, regdump_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch register dump private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int regdump_release(struct inode *inode, struct file *filp)
{
	pr_debug("called");

	filp->private_data = NULL;
	return 0;
}

static int crashdump_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("enter");

	if ((inode)->i_cdev == NULL) {
		pr_err("retrieve cdev from inode failed");
		return -ENODEV;
	}
	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, crashdump_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch crash dump private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int crashdump_release(struct inode *inode, struct file *filp)
{

	filp->private_data = NULL;
	return 0;
}

static ssize_t crashdump_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *priv = (struct iaxxx_priv *)filp->private_data;
	ssize_t size;

	if (!priv)
		return -EINVAL;

	if (!priv->crashlog)
		return -EINVAL;

	dev_info(priv->dev, "%s: called\n", __func__);

	if (!priv->crashlog->log_buffer) {
		dev_err(priv->dev, "%s: crash log buffer is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&priv->crashdump_lock);

	/* Read done */
	if (priv->crashlog->logs_read == priv->crashlog->log_buffer_size) {
		priv->crashlog->logs_read = 0;
		mutex_unlock(&priv->crashdump_lock);
		dev_info(priv->dev, "%s: Read done\n", __func__);
		return 0;
	}

	if (priv->crashlog->logs_read + count > priv->crashlog->log_buffer_size)
		size = priv->crashlog->log_buffer_size
			- priv->crashlog->logs_read;
	else
		size = count;

	/* Copy data to user buffer */
	if (copy_to_user(buf, priv->crashlog->log_buffer
				+ priv->crashlog->logs_read, size)) {
		mutex_unlock(&priv->crashdump_lock);
		dev_err(priv->dev, "%s: copy to user fail\n", __func__);
		return -EFAULT;
	}
	priv->crashlog->logs_read += size;
	mutex_unlock(&priv->crashdump_lock);

	return size;
}

static int get_mem_stats_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	if (!header)
		return -EINVAL;

	/* Get memory statistics log header */
	header[log_id].block_id = INVALID_BLOCK_ID;
	header[log_id].log_info = INVALID_LOG_INFO;
	header[log_id].log_type = MEM_STATS;

	header[log_id].start_addr =
		priv->rbdt_info[IAXXX_BLOCK_MEM_STAT].hdr_base_addr;
	header[log_id].size = (priv->rbdt_info[IAXXX_BLOCK_MEM_STAT].hdr_size *
		priv->rbdt_info[IAXXX_BLOCK_MEM_STAT].num_of_hdr) +
		(priv->rbdt_info[IAXXX_BLOCK_MEM_STAT].grp_size *
		priv->rbdt_info[IAXXX_BLOCK_MEM_STAT].num_of_grp);

	return 0;
}

static int get_mips_stats_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	if (!header)
		return -EINVAL;

	/* Get mips statistics log header */
	header[log_id].block_id = INVALID_BLOCK_ID;
	header[log_id].log_info = INVALID_LOG_INFO;
	header[log_id].log_type = MIPS_STATS;

	header[log_id].start_addr =
		priv->rbdt_info[IAXXX_BLOCK_MIPS].hdr_base_addr;
	header[log_id].size = (priv->rbdt_info[IAXXX_BLOCK_MIPS].hdr_size *
		priv->rbdt_info[IAXXX_BLOCK_MIPS].num_of_hdr) +
		(priv->rbdt_info[IAXXX_BLOCK_MIPS].grp_size *
		priv->rbdt_info[IAXXX_BLOCK_MIPS].num_of_grp);

	return 0;
}

static int get_crash_log_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	int rc;
	int i;

	if (!header)
		return -EINVAL;

	/* Get crash log header for each core */
	for (i = 0; i < MAX_CORES; i++) {
		header[log_id].block_id = i;
		header[log_id].log_info = INVALID_LOG_INFO;
		header[log_id].log_type = CRASH_LOG;

		/*
		 * Read 2 words to get crash log memory address and size,
		 * 8 represents number of bytes
		 */
		rc = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUG_BLOCK_0_CRASHLOG_ADDR_ADDR + i * 8,
					&header[log_id].start_addr, 2);
		if (rc) {
			dev_err(priv->dev,
				"%s: Block %d addr size read failed : %d\n",
				__func__, i, rc);
			return rc;
		}

		log_id++;
	}
	return rc;
}

static int get_debug_log_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	int rc;
	int i;

	if (!header)
		return -EINVAL;

	/* Get debug log header for each core */
	for (i = 0; i < MAX_CORES; i++) {
		header[log_id].block_id = i;
		header[log_id].log_info = INVALID_LOG_INFO;
		header[log_id].log_type = DEBUG_LOG;

		/*
		 * Read 2 words to get debug log memory address and size,
		 * 8 represents number of bytes
		 */
		rc = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUG_BLOCK_0_DEBUGLOG_ADDR_ADDR + i * 8,
					&header[log_id].start_addr, 2);
		if (rc) {
			dev_err(priv->dev,
				"%s: Block %d addr size read failed : %d\n",
				__func__, i, rc);
			return rc;
		}

		log_id++;
	}
	return rc;
}

static int get_fw_register_access_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	int rc;

	if (!header)
		return -EINVAL;

	/* Get header of register access history stored by FW */
	header[log_id].block_id = INVALID_BLOCK_ID;
	header[log_id].log_info = INVALID_LOG_INFO;
	header[log_id].log_type = FW_REG_HISTORY;

	/* Read 2 words to get register access history address and size */
	rc = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUG_DEBUGREG_HISTLOG_ADDR_ADDR,
			&header[log_id].start_addr, 2);
	if (rc) {
		dev_err(priv->dev,
			"%s: addr size read failed : %d\n", __func__, rc);
		return rc;
	}

	return rc;
}

static int get_trace_log_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	int rc;
	int i;

	if (!header)
		return -EINVAL;

	/* Get trace log header of each core */
	for (i = 0; i < MAX_CORES; i++) {
		header[log_id].block_id = i;
		header[log_id].log_info = INVALID_LOG_INFO;
		header[log_id].log_type = TRACE_LOG;

		/*
		 * Read 2 words to get trace log memory address and size,
		 * 8 represents number of bytes
		 */
		rc = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUG_BLOCK_0_TRACE_BUFFER_SIZE_ADDR + i * 8,
					&header[log_id].start_addr, 2);
		if (rc) {
			dev_err(priv->dev,
				"%s: Block %d addr size read failed : %d\n",
				__func__, i , rc);
			return rc;
		}

		log_id++;
	}
	return rc;
}

static int get_srb_arb_log_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)
{
	int i;

	if (!header)
		return -EINVAL;

	/* SRB header info */
	header[log_id].block_id = INVALID_BLOCK_ID;
	header[log_id].log_info = 0;
	header[log_id].log_type = SRB_ARB_DUMP;
	header[log_id].start_addr = IAXXX_SRB_REGS_ADDR;
	header[log_id].size = IAXXX_SRB_ARB_31_INFO_ADDR - IAXXX_SRB_REGS_ADDR;
	log_id++;

	/* ARB header info */
	for (i = 0; i < MAX_ARBS; i++) {
		header[log_id].block_id = INVALID_BLOCK_ID;
		/* 0xA represents its ARB log header */
		header[log_id].log_info = (0xA << 8) + i;
		header[log_id].log_type = SRB_ARB_DUMP;

		/*
		 * Copy ARB addr and size from rbdt struct
		 */
		header[log_id].start_addr = priv->rbdt_info[i].hdr_base_addr;
		header[log_id].size = (priv->rbdt_info[i].hdr_size *
			priv->rbdt_info[i].num_of_hdr) +
			(priv->rbdt_info[i].grp_size *
			 priv->rbdt_info[i].num_of_grp);

		log_id++;
	}

	return 0;
}

static int get_plugin_log_header(struct iaxxx_priv *priv,
					struct log_header *header, u32 log_id)

{
	int rc;
	uint32_t inst_id, addr;
	struct list_head *node, *tmp;
	struct iaxxx_plugin_data *plugin_data;

	if (!header)
		return -EINVAL;

	/* Get plugin log header */
	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
							plugin_node);
		if (!plugin_data->log_enabled)
			continue;

		inst_id = plugin_data->inst_id;
		header[log_id].block_id = plugin_data->block_id;
		header[log_id].log_info = inst_id;
		header[log_id].log_type = PLUGIN_DBGLOG;

		/*
		 * Read 2 words to get plugin log memory address and size,
		 * 8 represents number of bytes
		 */
		addr = IAXXX_GET_GRP_ADDR(priv,
			IAXXX_PLUGIN_INS_GRP_PLUGINLOG_ADDR_ADDR,
			inst_id);
		if (!addr)
			return -EINVAL;
		rc = regmap_bulk_read(priv->regmap, addr,
				      &header[log_id].start_addr, 2);
		if (rc) {
			dev_err(priv->dev, "Log %d address read failed : %d\n",
				inst_id, rc);
			return -EINVAL;
		}

		log_id++;
	}

	return rc;
}

static u32 get_plugin_log_en_count(struct iaxxx_priv *priv)
{
	u32 log_enabled = 0;
	struct list_head *node, *tmp;
	struct iaxxx_plugin_data *plugin_data;

	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
							plugin_node);
		if (plugin_data->log_enabled) {
			dev_info(priv->dev, "Plugin %d logs are enabled\n",
							plugin_data->inst_id);
			log_enabled++;
		}
	}

	dev_dbg(priv->dev, "Number of plugin logs enabled %d\n", log_enabled);
	return log_enabled;
}

int plugin_log_enable(struct iaxxx_priv *priv, u32 plg_inst, u8 block_id)
{
	int rc;
	u32 status;

	dev_info(priv->dev, "Enable log for inst %d block %d\n",
		 plg_inst, block_id);

	rc = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_ENABLE_BLOCK_ADDR(block_id),
		1 << plg_inst,
		1 << plg_inst);
	if (rc) {
		dev_err(priv->dev, "%s: update failed : %d\n", __func__, rc);
		return rc;
	}

	rc = iaxxx_send_update_block_request(priv->dev, &status, block_id);
	if (rc) {
		dev_err(priv->dev,
		"%s: PLUGIN HDR ENABLE BLOCK ADDR Update blk failed : %d\n",
			__func__, rc);
		return rc;
	}

	return rc;
}

static ssize_t log_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *priv = (struct iaxxx_priv *)filp->private_data;
	size_t size;
	int rc;
	char *kbuf;
	u32 data_written;
	u32 read_data;
	uint32_t buf_size;
	uint32_t log_count;
	int i;
	bool read_status;
	struct log_header *header;

	kbuf = priv->debug_log->kbuf;
	data_written = priv->debug_log->data_written;
	read_data = priv->debug_log->read_data;
	read_status = priv->debug_log->read_status;
	header = priv->debug_log->header;

	/* If data read is complete, clear the static variables */
	if (read_status) {
		kvfree(kbuf);
		priv->debug_log->kbuf = NULL;
		kvfree(header);
		priv->debug_log->header = NULL;
		priv->debug_log->data_written = 0;
		priv->debug_log->read_data = 0;
		priv->debug_log->read_status = false;
		return 0;
	}

	/* If buffer is not alloacted, then only read the log data */
	if (!kbuf) {
		/* Read the number of logs available */
		log_count = DEBUG_LOG_PLUGIN_LOG_START;
		log_count += get_plugin_log_en_count(priv);

		header = iaxxx_kvmalloc(log_count * sizeof(struct log_header),
				 __GFP_ZERO);
		if (!header)
			return -ENOMEM;

		priv->debug_log->header = header;

		/* Collect all the logs header info */
		rc = get_srb_arb_log_header(priv, header,
						DEBUG_LOG_SRB_ARB_DUMP_START);
		if (rc)
			goto err;
		rc = get_debug_log_header(priv, header,
						DEBUG_LOG_DEBUG_LOG_START);
		if (rc)
			goto err;
		rc = get_trace_log_header(priv, header,
						DEBUG_LOG_TRACE_LOG_START);
		if (rc)
			goto err;
		rc = get_fw_register_access_header(priv, header,
						DEBUG_LOG_REG_ACCESS_LOG_START);
		if (rc)
			goto err;
		rc = get_mem_stats_header(priv, header,
						DEBUG_LOG_MEM_LOG_START);
		if (rc)
			goto err;
		rc = get_mips_stats_header(priv, header,
						DEBUG_LOG_MIPS_LOG_START);
		if (rc)
			goto err;

		/* Read plugin log header only if any plugin log enabled */
		if (get_plugin_log_en_count(priv)) {
			rc = get_plugin_log_header(priv, header,
					DEBUG_LOG_PLUGIN_LOG_START);
			if (rc)
				goto err;
		}

		/* Get the log buffer size */
		buf_size = log_count * sizeof(struct log_header);
		for (i = 0; i < log_count; i++)
			buf_size += header[i].size;

		kbuf = iaxxx_kvmalloc(buf_size, __GFP_ZERO);
		if (!kbuf) {
			rc = -ENOMEM;
			goto err;
		}

		priv->debug_log->kbuf = kbuf;

		/* Read the log data */
		for (i = 0; i < log_count; i++) {
			/* Copy the log header */
			memcpy(kbuf + data_written, &header[i],
			       sizeof(struct log_header));
			data_written += sizeof(struct log_header);

			/* If log size is 0, skip log read */
			if (!header[i].size)
				continue;

			/* Read the logs */
			rc = priv->bus_read(priv->dev,
				header[i].start_addr,
				kbuf + data_written,
				header[i].size / sizeof(uint32_t));
			if (rc != header[i].size / sizeof(uint32_t)) {
				dev_err(priv->dev,
					"Not able to read log data %d\n", rc);
				rc = -EIO;
				goto data_read_err;
			}

			data_written += header[i].size;
		}
	}

	if (count > (data_written - read_data))
		size = data_written - read_data;
	else
		size = count;

	if (copy_to_user(buf, kbuf + read_data, size)) {
		rc = -EFAULT;
		goto copy_user_err;
	}

	read_data += size;

	if (read_data == data_written)
		read_status = true;

	priv->debug_log->data_written = data_written;
	priv->debug_log->read_data = read_data;
	priv->debug_log->read_status = read_status;

	dev_dbg(priv->dev, "Data written 0x%x\n", data_written);

	return size;

copy_user_err:
data_read_err:
	kvfree(kbuf);
	priv->debug_log->kbuf = NULL;
err:
	kvfree(header);
	priv->debug_log->header = NULL;
	priv->debug_log->data_written = data_written;
	priv->debug_log->read_data = read_data;
	priv->debug_log->read_status = read_status;
	dev_err(priv->dev, "%s: failed : %d\n", __func__, rc);
	return rc;
}

/*
 * log_write is to enable the plugin logs at runtime
 * To enable the logs run #echo plglogen<inst_id> > /dev/debug_logX
 * Example: #echo plglogen4 > /dev/debug_logX will enable the logs for
 * plugin instance 4
 */
static ssize_t log_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *priv = (struct iaxxx_priv *)filp->private_data;
	struct iaxxx_plugin_data *plugin_data;
	char *kbuf;
	char *plugin_str = "plglogen";
	int err;
	u32 plg_inst;

	if (!priv)
		return -EINVAL;

	dev_dbg(priv->dev, "%s: called\n", __func__);

	kbuf = iaxxx_kvmalloc(count, __GFP_ZERO);
	if (!kbuf)
		return -ENOMEM;

	err = copy_from_user(kbuf, buf, count);
	if (err) {
		dev_err(priv->dev, "%s: copy from user failed : %d\n",
								__func__, err);
		kvfree(kbuf);
		return -EINVAL;
	}

	if (strncmp(kbuf, plugin_str, strlen(plugin_str))) {
		dev_err(priv->dev, "%s: string should start with %s\n",
			__func__, plugin_str);
		kvfree(kbuf);
		return -EINVAL;
	}

	err = kstrtou32(kbuf + strlen(plugin_str), 0, &plg_inst);
	if (err) {
		kvfree(kbuf);
		return err;
	}

	dev_info(priv->dev, "Enable logs for plugin Inst %d\n", plg_inst);

	/* Check plugin instance is created */
	plugin_data = iaxxx_core_plugin_exist(priv, plg_inst);
	if (!plugin_data) {
		dev_err(priv->dev,
			"%s: Plugin instance 0x%x is not created\n",
			__func__, plg_inst);
		kvfree(kbuf);
		return -EINVAL;
	}

	err = plugin_log_enable(priv, plg_inst,
				plugin_data->block_id);
	if (err)
		dev_err(priv->dev, "%s: log enable failed : %d\n",
			__func__, err);
	else
		plugin_data->log_enabled = true;


	kvfree(kbuf);
	return count;
}

static int log_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("enter");

	if ((inode)->i_cdev == NULL) {
		pr_err("retrieve cdev from inode failed");
		return -ENODEV;
	}
	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, debug_log_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch log private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int log_release(struct inode *inode, struct file *filp)
{

	filp->private_data = NULL;
	return 0;
}

int iaxxx_get_crashlog_header(struct iaxxx_priv *priv)
{
	int rc;

	struct log_header *header;

	/* Free the header memory if allocated before */
	kvfree(priv->crashlog->header);
	priv->crashlog->header = NULL;

	priv->crashlog->header = iaxxx_kvmalloc(
		CRASH_LOG_COUNT * sizeof(struct log_header),
		__GFP_ZERO);
	if (!priv->crashlog->header)
		return -ENOMEM;

	header = priv->crashlog->header;

	/* Get all the logs header */
	rc = get_debug_log_header(priv, header, CRASH_LOG_DEBUG_LOG_START);
	if (rc)
		goto err;
	rc = get_trace_log_header(priv, header, CRASH_LOG_TRACE_LOG_START);
	if (rc)
		goto err;
	rc = get_fw_register_access_header(priv, header,
			CRASH_LOG_REG_ACCESS_LOG_START);
	if (rc)
		goto err;
	rc = get_crash_log_header(priv, header, CRASH_LOG_CRASH_LOG_START);
	if (rc)
		goto err;
	rc = get_mem_stats_header(priv, header, CRASH_LOG_MEM_LOG_START);
	if (rc)
		goto err;
	rc = get_mips_stats_header(priv, header, CRASH_LOG_MIPS_LOG_START);
	if (rc)
		goto err;

	return rc;
err:
	kvfree(priv->crashlog->header);
	priv->crashlog->header = NULL;
	dev_err(priv->dev, "%s: failed : %d\n", __func__, rc);
	return rc;

}

int iaxxx_dump_crashlogs(struct iaxxx_priv *priv)
{
	uint32_t buf_size;
	uint32_t data_written = 0;
	struct log_header header;
	int i;
	int rc;

	pr_info("dev id-%d: called", priv->dev_id);

	if (!priv->crashlog->header)
		return -EINVAL;

	/* Get log buffer size */
	buf_size = CRASH_LOG_COUNT * sizeof(struct log_header);
	for (i = 0; i < CRASH_LOG_COUNT; i++)
		buf_size += priv->crashlog->header[i].size;

	/* Free the buffer memory if allocated before */
	kvfree(priv->crashlog->log_buffer);
	priv->crashlog->log_buffer = NULL;

	priv->crashlog->log_buffer = iaxxx_kvmalloc(buf_size, __GFP_ZERO);
	if (!priv->crashlog->log_buffer)
		return -ENOMEM;

	priv->crashlog->log_buffer_size = buf_size;

	/* Read the log data */
	for (i = 0; i < CRASH_LOG_COUNT; i++) {
		/* Copy the log header info */
		header = priv->crashlog->header[i];
		memcpy(priv->crashlog->log_buffer + data_written, &header,
		       sizeof(struct log_header));
		data_written += sizeof(struct log_header);

		/* If log size is 0, skip log read */
		if (!header.size)
			continue;

		/* Read the logs */
		rc = priv->bus_read(priv->dev,
				header.start_addr,
				priv->crashlog->log_buffer + data_written,
				header.size / sizeof(uint32_t));
		if (rc != header.size / sizeof(uint32_t)) {
			dev_err(priv->dev,
				"Not able to read crash log data %d\n", rc);
			rc = -EIO;
			return rc;
		}

		data_written += header.size;
	}

	pr_info("dev id-%d: Crash Log size %d", priv->dev_id, data_written);

	return 0;
}

static const struct file_operations raw_fops = {
	.owner = THIS_MODULE,
	.open = raw_open,
	.read = raw_read,
	.write = raw_write,
	.unlocked_ioctl = raw_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = raw_compat_ioctl,
#endif
	.release = raw_release,
};

static const struct file_operations regdump_fops = {
	.owner = THIS_MODULE,
	.open = regdump_open,
	.read = regdump_read,
	.write = regdump_write,
	.release = regdump_release,
};

static const struct file_operations crashdump_fops = {
	.owner = THIS_MODULE,
	.open = crashdump_open,
	.read = crashdump_read,
	.release = crashdump_release,
};

static const struct file_operations log_fops = {
	.owner = THIS_MODULE,
	.open = log_open,
	.read = log_read,
	.write = log_write,
	.release = log_release,
};

int iaxxx_debug_init(struct iaxxx_priv *priv)
{
	struct iaxxx_debug_data *intf_priv = NULL;
	int err;

	pr_debug("initializing debug interface");

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	intf_priv = devm_kzalloc(priv->dev, sizeof(*intf_priv), GFP_KERNEL);
	if (!intf_priv)
		return -ENOMEM;

	priv->intf_priv = intf_priv;
	intf_priv->priv = priv;

	err = iaxxx_cdev_create(&intf_priv->raw_cdev, priv->dev,
		&raw_fops, intf_priv, priv->dev_id, IAXXX_CDEV_RAWDEV);
	if (err) {
		pr_err("creating rawdev%d device failed : %d",
							priv->dev_id, err);
		err = -EIO;
		goto raw_cdev_err;
	}

	err = iaxxx_cdev_create(&intf_priv->regdump_cdev, priv->dev,
		&regdump_fops, intf_priv, priv->dev_id, IAXXX_CDEV_REGDUMP);
	if (err) {
		pr_err("creating regdump%d device failed : %d",
							priv->dev_id, err);
		err = -EIO;
		goto regdump_cdev_err;
	}

	err = iaxxx_cdev_create(&intf_priv->crashdump_cdev, priv->dev,
		&crashdump_fops, intf_priv, priv->dev_id, IAXXX_CDEV_CRASHDUMP);
	if (err) {
		pr_err("creating crashdump%d device failed : %d",
							priv->dev_id, err);
		err = -EIO;
		goto crashdump_cdev_err;
	}

	err = iaxxx_cdev_create(&intf_priv->debug_log_cdev, priv->dev,
		&log_fops, intf_priv, priv->dev_id, IAXXX_CDEV_DBGLOG);
	if (err) {
		pr_err("creating debug_log%d device failed : %d",
							priv->dev_id, err);
		goto debug_log_cdev_err;
	}

	priv->raw_ops = iaxxx_kvmalloc(sizeof(struct iaxxx_raw_bus_ops), 0);
	if (!priv->raw_ops) {
		err = -ENOMEM;
		goto raw_mem_alloc_err;
	}
	return err;
raw_mem_alloc_err:
	iaxxx_cdev_destroy(&intf_priv->debug_log_cdev, IAXXX_CDEV_DBGLOG);
debug_log_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->crashdump_cdev, IAXXX_CDEV_CRASHDUMP);
crashdump_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->regdump_cdev, IAXXX_CDEV_REGDUMP);
regdump_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->raw_cdev, IAXXX_CDEV_RAWDEV);
raw_cdev_err:

	devm_kfree(priv->dev, intf_priv);
	priv->intf_priv = NULL;
	return err;
}

int iaxxx_debug_exit(struct iaxxx_priv *priv)
{
	struct iaxxx_debug_data *intf_priv = NULL;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}
	intf_priv = (struct iaxxx_debug_data *) priv->intf_priv;
	iaxxx_cdev_destroy(&intf_priv->raw_cdev, IAXXX_CDEV_RAWDEV);
	iaxxx_cdev_destroy(&intf_priv->regdump_cdev, IAXXX_CDEV_REGDUMP);
	iaxxx_cdev_destroy(&intf_priv->crashdump_cdev, IAXXX_CDEV_CRASHDUMP);
	iaxxx_cdev_destroy(&intf_priv->debug_log_cdev, IAXXX_CDEV_DBGLOG);
	devm_kfree(priv->dev, intf_priv);

	kvfree(priv->raw_ops);

	return 0;
}
