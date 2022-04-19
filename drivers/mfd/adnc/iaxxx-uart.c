/*
 * iaxxx-uart.c  --  Audience ia6xx UART interface
 *
 * Copyright 2013 Audience, Inc.
 *
 * Author: Matt Lupfer <mlupfer@cardinalpeak.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/module.h>
#include <linux/tty.h>
#include <asm/ioctls.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/delay.h>

#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include "iaxxx.h"

#define IA_DELAY_1MS                    1000    /* 1 ms*/
#define IA_DELAY_2MS                    2000    /* 2 ms*/
#define IA_DELAY_5MS                    5000    /* 5 ms*/
#define IA_DELAY_12MS                   12000   /* 12 ms*/

#define IA_RESP_TOUT                    IA_DELAY_12MS /* 20ms */
#define IA_RESP_POLL_TOUT               4000  /*  4ms */
#define IA_MAX_RETRIES			\
					(IA_RESP_TOUT / IA_RESP_POLL_TOUT)

#define UART_OPEN_TIMEOUT		60000
#define UART_MORE_TIMEOUT		(UART_OPEN_TIMEOUT + 1000)

#ifdef CONFIG_ARCH_HISI
#define UART_TTY_DEVICE_NODE            "/dev/ttyAMA3"
#else
#define UART_TTY_DEVICE_NODE            "/dev/ttyHS1"
#endif

#define UART_TTY_STOP_BITS              2
#define UART_TTY_WRITE_SZ               512
#define UART_TTY_READ_SZ		512
#define MAX_EAGAIN_RETRY                20
#define MAX_READ_FAILURE_RETRY          5
#define EAGAIN_RETRY_DELAY              IA_DELAY_1MS
#define IA_TTY_BUF_AVAIL_WAIT_DELAY     IA_DELAY_2MS
#define IA_TTY_WAIT_TIMEOUT             500

/* Below Ioctls are taken from serial driver msm_serial_hs.c */
#define UART_OP_CLK_ON			_IO('T', 0x41)
#define UART_OP_CLK_OFF			_IO('T', 0x42)
#define UART_OP_CLK_STATE		_IO('T', 0x43)

#define	IAXXX_REG_LEN			4 /* Register length */
#define IAXXX_MIN_DATA_LEN		4 /* Minimum data size */
/*
 * To update last 2 bits of address. Last 2 bits has the info of read write
 * operation.
 */
#define IAXXX_UART_OP_UPDATE(OP)	(OP << 24)

#define IAXXX_UART_FLOW_EN_CMD		0x80190001

struct iaxxx_uart_priv {
	struct iaxxx_priv priv;	/* private data */
	struct iaxxx_uart_device *uart;
};

struct iaxxx_uart_device {
	struct tty_struct *tty;
	struct file *file;
	struct task_struct *uart_thread;
	struct mutex uart_thread_mutex;
	struct mutex uart_lock;
	wait_queue_head_t uart_wait_q;
	wait_queue_head_t uart_compl_q;
	atomic_t uart_thread_cond;
	atomic_t uart_users;
	unsigned int baudrate;
	int vote_counter;
	u8 uart_ready;
};

enum{
	UART_OPEN = 1,
	UART_CLOSE = 2,
};

enum iaxxx_uart_op {
	IAXXX_SINGLE_READ = 0,
	IAXXX_SINGLE_WRITE,
	IAXXX_MULTI_READ,
	IAXXX_MULTI_WRITE,
};

static inline struct iaxxx_uart_priv *to_uart_priv(struct iaxxx_priv *priv)
{
	return priv ? container_of(priv, struct iaxxx_uart_priv, priv) : NULL;
}

#if (defined(CONFIG_ARCH_MSM8996) || defined(CONFIG_ARCH_MSM8953))
static long iaxxx_uart_tty_ioctl(struct file *f, unsigned int op)
{
	unsigned long param = 0;

	if (f->f_op->unlocked_ioctl)
		return f->f_op->unlocked_ioctl(f, op, param);

	pr_err("no unlocked_ioctl defined");
	return -EINVAL;
}

static int iaxxx_uart_clock_control(struct iaxxx_uart_device *iaxxx_uart,
				    int cmd)
{
	int err = 0;

	pr_debug("called 0x%x", cmd);

	switch (cmd) {
	case UART_OP_CLK_ON:
		pr_debug("Clock On");
		iaxxx_uart->vote_counter++;
		if (iaxxx_uart->vote_counter == 1)
			err = iaxxx_uart_tty_ioctl(iaxxx_uart->file, cmd);
		break;
	case UART_OP_CLK_OFF:
		pr_debug("Clock Off");
		iaxxx_uart->vote_counter--;
		if (iaxxx_uart->vote_counter < 0)
			iaxxx_uart->vote_counter = 0;
		else if (iaxxx_uart->vote_counter == 0)
			err = iaxxx_uart_tty_ioctl(iaxxx_uart->file, cmd);
		break;
	case UART_OP_CLK_STATE:
		pr_debug("Get clock state");
		err = iaxxx_uart_tty_ioctl(iaxxx_uart->file, cmd);
		break;
	}

	pr_debug("cmd 0x%x, ret %d", cmd, err);

	return err;
}
#endif

static int iaxxx_uart_open_raw(struct iaxxx_uart_device *iaxxx_uart)
{
	long err = 0;
	int ret = 0;
	struct file *fp = NULL;

	/* Timeout increased to 60 Seconds to avoid UART open failure in KK */
	unsigned long timeout = jiffies + msecs_to_jiffies(UART_OPEN_TIMEOUT);
	int attempt = 0;

	mutex_lock(&iaxxx_uart->uart_lock);

	pr_debug("UART users: %d enter",
		atomic_read(&iaxxx_uart->uart_users));

	if (atomic_inc_return(&iaxxx_uart->uart_users) > 1) {
		pr_debug("UART is already opened, users: %d",
			 atomic_read(&iaxxx_uart->uart_users));
		goto unlock_exit;
	}

	pr_debug("start open tty");
	/* try to probe tty node every 100 ms for 6 sec */
	do {
		if (attempt)
			msleep(50);
		pr_debug("probing for tty on %s (attempt %d)",
				UART_TTY_DEVICE_NODE, attempt);

		fp = filp_open(UART_TTY_DEVICE_NODE,
			       O_RDWR | O_NONBLOCK | O_NOCTTY, 0);
		err = PTR_ERR(fp);
		if (IS_ERR(fp))
			pr_debug("UART failed : %ld, retry", err);

		attempt = (-EACCES == err || -ENOENT == err);
	} while (time_before(jiffies, timeout) && attempt);

	if (IS_ERR_OR_NULL(fp)) {
		pr_err("UART device node open failed : %ld", err);
		atomic_dec(&iaxxx_uart->uart_users);
		ret = -ENODEV;
		goto unlock_exit;
	}

	/* set uart_dev members */
	iaxxx_uart->file = fp;
#if (defined(CONFIG_ARCH_MSM8996) || defined(CONFIG_ARCH_MSM8953))
	iaxxx_uart_clock_control(iaxxx_uart, UART_OP_CLK_ON);
#endif
	iaxxx_uart->tty =
		((struct tty_file_private *)fp->private_data)->tty;

	iaxxx_uart->uart_ready = 1;
	pr_debug("UART open successfully!");

unlock_exit:
	mutex_unlock(&iaxxx_uart->uart_lock);
	return ret;
}

static int iaxxx_uart_close_raw(struct iaxxx_uart_device *iaxxx_uart)
{
	mutex_lock(&iaxxx_uart->uart_lock);

	pr_debug("UART users count: %d enter",
			atomic_read(&iaxxx_uart->uart_users));

	if (IS_ERR_OR_NULL(iaxxx_uart->file)) {
		pr_err("UART file error");
		goto unlock_exit;
	}

	if (atomic_read(&iaxxx_uart->uart_users) < 1) {
		pr_err("UART is already closed");
		atomic_set(&iaxxx_uart->uart_users, 0);
		goto unlock_exit;
	}

	if (atomic_dec_return(&iaxxx_uart->uart_users)) {
		pr_debug("UART is busy to close, user count: %d",
			 atomic_read(&iaxxx_uart->uart_users));
		goto unlock_exit;
	}

#if (defined(CONFIG_ARCH_MSM8996) || defined(CONFIG_ARCH_MSM8953))
	iaxxx_uart_clock_control(iaxxx_uart, UART_OP_CLK_OFF);
#endif
	iaxxx_uart->tty = NULL;
	filp_close(iaxxx_uart->file, NULL);
	iaxxx_uart->file = NULL;
	iaxxx_uart->uart_ready = 0;

	pr_debug("UART close successfully!");
unlock_exit:
	mutex_unlock(&iaxxx_uart->uart_lock);
	return 0;
}

static int iaxxx_uart_thread_enable(struct iaxxx_uart_device *iaxxx_uart,
				    int enable)
{
	int ret = 0;
	int cmd = 0;

	if (!iaxxx_uart || !iaxxx_uart->uart_thread) {
		pr_err("Invalid null pointer");
		return -EINVAL;
	}

	mutex_lock(&iaxxx_uart->uart_thread_mutex);

	if (enable)
		cmd = UART_OPEN;
	else
		cmd = UART_CLOSE;

	pr_debug("cmd %d", cmd);
	atomic_set(&iaxxx_uart->uart_thread_cond, cmd);

	/* Commit data */
	smp_wmb();

	wake_up(&iaxxx_uart->uart_wait_q);

	pr_debug("wake up uart_wait_q");
	ret = wait_event_timeout(iaxxx_uart->uart_compl_q,
		(atomic_read(&iaxxx_uart->uart_thread_cond) <= 0),
		msecs_to_jiffies(UART_MORE_TIMEOUT));
	pr_debug("wait_event_timeout for uart_compl_q");
	if (ret > 0)
		ret = atomic_read(&iaxxx_uart->uart_thread_cond);
	else if (ret == 0)
		pr_err("wait timed out");

	mutex_unlock(&iaxxx_uart->uart_thread_mutex);
	return ret;
}

int iaxxx_uart_open(struct iaxxx_priv *iaxxx)
{
	int ret = 0;
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);
	struct iaxxx_uart_device *iaxxx_uart;

	if (!uart_priv) {
		pr_err("UART Private node is NULL");
		return -EINVAL;
	}

	iaxxx_uart = uart_priv->uart;

	if (!iaxxx_uart) {
		pr_err("UART Device node is NULL");
		return -EINVAL;
	}

	if (!current->fs)
		ret = iaxxx_uart_thread_enable(iaxxx_uart, true);
	else
		ret = iaxxx_uart_open_raw(iaxxx_uart);

	return ret;
}

int iaxxx_uart_close(struct iaxxx_priv *iaxxx)
{
	int ret = 0;
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);

	if (!uart_priv || !uart_priv->uart) {
		pr_err("UART Private node is NULL");
		return -EINVAL;
	}

	if (!current->fs)
		ret = iaxxx_uart_thread_enable(uart_priv->uart, false);
	else
		ret = iaxxx_uart_close_raw(uart_priv->uart);

	return ret;
}

static int iaxxx_uart_read_internal(struct iaxxx_uart_device *iaxxx_uart,
				    void *buf, int len)
{
	int rc;
	mm_segment_t oldfs;
	loff_t pos = 0;
	int retry = MAX_EAGAIN_RETRY;

	pr_debug("size %d", len);

	if (unlikely(!iaxxx_uart->uart_ready)) {
		pr_err("Error: UART is not ready");
		return -EIO;
	}

	if (atomic_read(&iaxxx_uart->uart_users) < 1) {
		pr_err("Error: UART is not open");
		return -EIO;
	}

	if (IS_ERR_OR_NULL(iaxxx_uart->file)) {
		pr_err("Error: invalid uart fd");
		return -EBADF;
	}

	/*
	 * we may call from user context via char dev, so allow
	 * read buffer in kernel address space
	 */
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	do {
		rc = vfs_read(iaxxx_uart->file, (char __user *)buf, len, &pos);
		if (rc == -EAGAIN) {
			usleep_range(EAGAIN_RETRY_DELAY,
				EAGAIN_RETRY_DELAY + 50);
			retry--;
		}
	} while (rc == -EAGAIN && retry);

	/* restore old fs context */
	set_fs(oldfs);

	pr_debug("read bytes %d", rc);

	return rc;
}

/* read all extra bytes on the UART line */
void iaxxx_uart_discard_rxbytes(struct iaxxx_priv *iaxxx)
{
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);
	int rc;
	u32 word;

	if (!uart_priv || !uart_priv->uart) {
		pr_err("UART Private node is NULL");
		return;
	}

	do {
		rc = iaxxx_uart_read_internal(uart_priv->uart, &word,
					      sizeof(word));
		if (!rc)
			pr_debug("reading extra bytes on UART 0x%x", word);
	} while (rc && rc > 0);
}

int iaxxx_uart_read(struct iaxxx_priv *iaxxx, void *buf, int len)
{
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);
	int rc = 0;
	int retry = MAX_READ_FAILURE_RETRY;
	int count_remain = len;
	int read_bytes = 0;

	if (!uart_priv || !uart_priv->uart) {
		pr_err("UART Private node is NULL");
		return -EINVAL;
	}

	if (unlikely(!uart_priv->uart->uart_ready)) {
		pr_err("Error UART is not open");
		return -EIO;
	}

	/*
	 * Wait until expected no. of bytes received, Data is requested in 512
	 * bytes chunks
	 */
	while (count_remain && retry) {
		pr_debug("count remain %d, read bytes %d", count_remain,
			 read_bytes);
		rc = iaxxx_uart_read_internal(uart_priv->uart,
					(char *)buf + read_bytes,
					min(UART_TTY_READ_SZ, count_remain));
		if (rc <= 0) {
			pr_debug("no bytes received from the chip");
			/*
			 * sometimes hosts tries to read for response earlier
			 * than firmware sends responds
			 */
			usleep_range(IA_DELAY_5MS, IA_DELAY_5MS + 5);
			retry--;
			continue;
		}
		retry = MAX_READ_FAILURE_RETRY;
		read_bytes += rc;
		count_remain -= rc;
	}

	if (count_remain) {
		pr_err("Read Failed for len: %d, bytes remain = %d read = %d",
		       len, count_remain, read_bytes);
		return -EIO;
	}

	return 0;
}

int iaxxx_uart_write(struct iaxxx_priv *iaxxx, const void *buf, int len)
{
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);
	struct iaxxx_uart_device *iaxxx_uart;
	int rc = 0;
	int count_remain = len;
	int bytes_wr = 0;
	mm_segment_t oldfs;
	loff_t pos = 0;
	void *data_buf = (void *) buf;
	int retry = IA_MAX_RETRIES;

	if (!uart_priv || !uart_priv->uart) {
		pr_err("UART Private node is NULL");
		return -EINVAL;
	}

	iaxxx_uart = uart_priv->uart;

	if (unlikely(!iaxxx_uart->uart_ready)) {
		pr_err("Error UART is not ready");
		return -EIO;
	}

	if (atomic_read(&iaxxx_uart->uart_users) < 1) {
		pr_err("Error: UART is not open");
		return -EIO;
	}

	if (IS_ERR_OR_NULL(iaxxx_uart->file)) {
		pr_err("Error: invalid uart fd");
		return -EBADF;
	}

	if (IS_ERR_OR_NULL(iaxxx_uart->tty)) {
		pr_err("tty is not available");
		return -EINVAL;
	}
	dev_dbg(iaxxx->dev, "%s: size %d\n", __func__, len);

	/*
	 * we may call from user context via char dev, so allow
	 * read buffer in kernel address space
	 */
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	while (count_remain > 0) {
		/* block until tx buffer space is available */
		while (tty_write_room(iaxxx_uart->tty) < UART_TTY_WRITE_SZ)
			usleep_range(IA_TTY_BUF_AVAIL_WAIT_DELAY,
					IA_TTY_BUF_AVAIL_WAIT_DELAY + 50);

		rc = vfs_write(iaxxx_uart->file,
				(__force const char __user *)
				data_buf + bytes_wr,
				min(UART_TTY_WRITE_SZ, count_remain),
				&pos);
		if (rc == -ERESTARTSYS) {
			pr_err("was interrupted by a signal by %s(%d)",
					current->comm, task_pid_nr(current));

			clear_thread_flag(TIF_SIGPENDING);
			if (retry--)
				continue;
			else
				goto err_out;
		}
		if (rc < 0) {
			pr_err("uart write failed for len %d, rc %d",
						count_remain, rc);
			goto err_out;
		}

		bytes_wr += rc;
		count_remain -= rc;
	}

err_out:
	/* restore old fs context */
	set_fs(oldfs);
	if (rc == -ERESTARTSYS) {
		rc = -EINTR;
	} else if (count_remain) {
		pr_err("uart write failed, count_remain %d", count_remain);
		rc = -EIO;
	} else {
		rc = 0;
	}
	pr_debug("returning %d", rc);

	return rc;
}

int iaxxx_uart_cmd(struct iaxxx_priv *iaxxx, u32 cmd_to_send, u32 *resp)
{
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);
	struct iaxxx_uart_device *iaxxx_uart;
	int err = 0;
	int send_cmd_iter_cnt = IA_MAX_RETRIES;
	int retry = IA_MAX_RETRIES;
	u32 rv = 0;
	u32 cmd;

	dev_dbg(iaxxx->dev, "%s: cmd = 0x%08x\n", __func__,
							cmd_to_send);
	if (!uart_priv || !uart_priv->uart) {
		pr_err("UART Private node is NULL");
		return -EINVAL;
	}

	iaxxx_uart = uart_priv->uart;

	err = iaxxx_uart_open(iaxxx);
	if (err) {
		dev_err(iaxxx->dev,
		"%s: uart_open() failed : %d", __func__, err);
		return -EIO;
	}

retry_from_1st:
	retry = IA_MAX_RETRIES + 1;
	err = 0;

	cmd = cpu_to_be32(cmd_to_send);

	err = iaxxx_uart_write(iaxxx, &cmd, sizeof(cmd));
	if (err || !resp) {
		if (err)
			dev_err(iaxxx->dev,
				"%s: uart_write() failed : %d\n",
								__func__, err);
		else
			/* Wait till cmd is completely sent to chip */
			tty_wait_until_sent(iaxxx_uart->tty,
				msecs_to_jiffies(IA_TTY_WAIT_TIMEOUT));

		goto cmd_exit;
	}

	*resp = 0;
	do {
		usleep_range(IA_RESP_POLL_TOUT,
				IA_RESP_POLL_TOUT + 500);
		rv = 0;
		err = iaxxx_uart_read(iaxxx, &rv, sizeof(rv));
		pr_debug("err = %d", err);
		*resp = be32_to_cpu(rv);
		pr_info("*resp = 0x%08x", *resp);
		if (err) {
			dev_err(iaxxx->dev,
				"%s: uart_read() failed : %d\n", __func__, err);
		} else {
			goto cmd_exit;
		}

		--retry;
	} while (retry != 0);

cmd_exit:
	if ((err || !retry) && send_cmd_iter_cnt--) {
		usleep_range(IA_DELAY_2MS, IA_DELAY_2MS + 5);
		goto retry_from_1st;
	}

	if (resp)
		pr_info("cmd 0x%08x, resp 0x%08x exit", cmd_to_send, *resp);

	iaxxx_uart_close(iaxxx);

	return err;
}

int iaxxx_configure_tty(struct tty_struct *tty, u32 bps, int stop)
{
	int rc = 0;
	struct ktermios termios;

	if (IS_ERR_OR_NULL(tty)) {
		pr_err("tty is not available");
		return -EINVAL;
	}

	termios = tty->termios;

	pr_info("Requesting baud %u", bps);

	termios.c_cflag &= ~(CBAUD | CSIZE | PARENB);   /* clear csize, baud */

#ifdef CONFIG_IAXXX_UART_HW_FLOW_CONTROL
	termios.c_cflag |= CRTSCTS;
#else
	termios.c_cflag &= ~CRTSCTS;
#endif

	termios.c_cflag |= BOTHER;	      /* allow arbitrary baud */
	termios.c_cflag |= CS8;

	if (stop == 2)
		termios.c_cflag |= CSTOPB;

	/* set uart port to raw mode (see termios man page for flags) */
	termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
		| INLCR | IGNCR | ICRNL | IXON);
	termios.c_oflag &= ~(OPOST);
	termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	/* set baud rate */
	termios.c_ospeed = bps;
	termios.c_ispeed = bps;

	/* Added to set baudrate dynamically */
	tty_wait_until_sent(tty, msecs_to_jiffies(IA_TTY_WAIT_TIMEOUT));
	rc = tty_set_termios(tty, &termios);

	pr_debug("New baud %u", tty->termios.c_ospeed);

	return rc;
}
EXPORT_SYMBOL_GPL(iaxxx_configure_tty);

static int ia_uart_thread_handler(void *ptr)
{
	struct iaxxx_uart_device *iaxxx_uart = (struct iaxxx_uart_device *) ptr;
	int cmd, ret = 0;

	if (!iaxxx_uart) {
		pr_err("Invalid null pointer");
		return -EINVAL;
	}

	set_freezable();
	do {
		wait_event_freezable(iaxxx_uart->uart_wait_q,
			(cmd = atomic_read(&iaxxx_uart->uart_thread_cond)) > 0);
		pr_debug("Handled thread resumed");
		switch (cmd) {
		case UART_OPEN:
			ret = iaxxx_uart_open_raw(iaxxx_uart);
			break;
		case UART_CLOSE:
			ret = iaxxx_uart_close_raw(iaxxx_uart);
			break;
		default:
			break;
		}
		atomic_set(&iaxxx_uart->uart_thread_cond, ret);

		/* Commit data */
		smp_wmb();

		pr_debug("Handled thread cmd %d, ret %d", cmd, ret);
		wake_up(&iaxxx_uart->uart_compl_q);
		pr_debug("Handled thread wake_up uart_compl_q");
	} while (!kthread_should_stop());

	return 0;
}

int iaxxx_uart_interface_detect(struct iaxxx_priv *iaxxx)
{
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(iaxxx);
	struct iaxxx_uart_device *iaxxx_uart = uart_priv->uart;
	int rc;
	uint32_t zeroes = 0x00000000;

	pr_debug("Enter");

	rc = iaxxx_uart_open(iaxxx);
	if (rc) {
		pr_err("iaxxx_uart_open() failed : %d", rc);
		return -EIO;
	}

	if (IS_ERR_OR_NULL(iaxxx_uart->tty)) {
		pr_err("tty is not available");
		goto uart_interface_det_fail;
	}

	/* set Host UART speed to bootloader baud */
	rc = iaxxx_configure_tty(iaxxx_uart->tty,
			iaxxx_uart->baudrate, UART_TTY_STOP_BITS);
	if (rc) {
		pr_err("TTY BAUD (%d) configuration failed : %d",
					iaxxx_uart->baudrate, rc);
		goto uart_interface_det_fail;
	}

	iaxxx_uart_discard_rxbytes(iaxxx);

	rc = iaxxx_uart_write(iaxxx, &zeroes, sizeof(zeroes));
	if (rc < 0) {
		dev_err(iaxxx->dev, "%s: UART zero byte send failed : %d\n",
				__func__, rc);
		goto uart_interface_det_fail;
	}

	/* 5ms delay required after 4 bytes of 0s written */
	usleep_range(IA_DELAY_5MS, IA_DELAY_5MS + 10);

	/* Read 0 byte sent by FW */
	zeroes = ~0; /* Assign it to some invalid value */
	rc = iaxxx_uart_read(iaxxx, &zeroes, sizeof(uint32_t));
	if (rc) {
		pr_err("Read failed : %d", rc);
		goto uart_interface_det_fail;
	}

	if (zeroes) {
		pr_err("didn't receive expected result (0x00000000) from FW");
		rc = -EINVAL;
		goto uart_interface_det_fail;
	}

	pr_debug("done");

uart_interface_det_fail:
	iaxxx_uart_close(iaxxx);
	return rc;
}

static int ia_uart_kthread_init(struct iaxxx_uart_device *iaxxx_uart)
{
	int ret = 0;

	if (!iaxxx_uart) {
		pr_err("Invalid null pointer");
		return -EINVAL;
	}

	mutex_init(&iaxxx_uart->uart_thread_mutex);
	atomic_set(&iaxxx_uart->uart_thread_cond, 0);
	init_waitqueue_head(&iaxxx_uart->uart_wait_q);
	init_waitqueue_head(&iaxxx_uart->uart_compl_q);

	iaxxx_uart->uart_thread = kthread_run(ia_uart_thread_handler,
			(void *)iaxxx_uart, "iaxxx uart thread");
	if (IS_ERR_OR_NULL(iaxxx_uart->uart_thread)) {
		pr_err("can't create iaxxx uart thread");
		iaxxx_uart->uart_thread = NULL;
		ret = -ENOMEM;
	}
	pr_info("Created iaxxx uart thread");
	return ret;
}

static void ia_uart_kthread_exit(struct iaxxx_uart_device *iaxxx_uart)
{
	if (!iaxxx_uart) {
		pr_err("Invalid null pointer");
		return;
	}

	if (iaxxx_uart->uart_thread) {
		kthread_stop(iaxxx_uart->uart_thread);
		pr_info("stopping stream kthread");
		iaxxx_uart->uart_thread = NULL;
	}
}

static struct of_device_id iaxxx_uart_id[] = {
	{
		.compatible = "knowles,iaxxx-uart",
		.data = NULL
	},
	{}
};
MODULE_DEVICE_TABLE(of, iaxxx_uart_id);

static int iaxxx_regmap_uart_read(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len)
{
	struct iaxxx_uart_priv *uart_priv = context;
	struct iaxxx_priv *priv = &uart_priv->priv;
	size_t data_len = cpu_to_be32(val_len / 4);
	int rc = 0;

	rc = iaxxx_uart_open(priv);
	if (rc < 0) {
		pr_err("UART open Failed : %d", rc);
		goto open_err;
	}

	/* If more than one register read, set MULTI and READ bit in addr */
	if (val_len > IAXXX_MIN_DATA_LEN)
		*(uint32_t *)reg = (*(uint32_t *)reg) |
			(IAXXX_MULTI_READ << 24);

	/* Write register address */
	rc = iaxxx_uart_write(priv, reg, reg_len);
	if (rc)
		goto err;

	/* Write the length to be read, if more than one register read */
	if (val_len > IAXXX_MIN_DATA_LEN) {
		rc = iaxxx_uart_write(priv, &data_len, sizeof(uint32_t));
		if (rc) {
			pr_err("Length write failed : %d", rc);
			goto err;
		}
	}

	if (!rc)
		/* Wait till cmd is completely sent to chip */
		tty_wait_until_sent(uart_priv->uart->tty,
				    msecs_to_jiffies(IA_TTY_WAIT_TIMEOUT));

	/* Read data from the register written above */
	rc = iaxxx_uart_read(priv, val, val_len);

err:
	iaxxx_uart_close(priv);
open_err:
	return rc;
}

static int iaxxx_regmap_uart_write(void *context, const void *data,
				   size_t count)
{
	struct iaxxx_uart_priv *uart_priv = context;
	struct iaxxx_priv *priv = &uart_priv->priv;
	int rc = 0;

	if (count != (IAXXX_REG_LEN + IAXXX_MIN_DATA_LEN))
		return -EINVAL;

	rc = iaxxx_uart_open(priv);
	if (rc) {
		pr_err("UART open Failed : %d", rc);
		goto open_err;
	}

	*(uint32_t *)data = (*(uint32_t *)data) |
		IAXXX_UART_OP_UPDATE(IAXXX_SINGLE_WRITE);
	rc = iaxxx_uart_write(priv, data, count);
	if (rc)
		pr_err("uart write failed : %d", rc);

	iaxxx_uart_close(priv);
open_err:
	return rc;
}

static int iaxxx_regmap_uart_gather_write(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	struct iaxxx_uart_priv *uart_priv = context;
	struct iaxxx_priv *priv = &uart_priv->priv;
	int rc = 0;
	size_t data_len = cpu_to_be32(val_len / 4);

	pr_debug("Enter, data length :%lu", val_len);

	rc = iaxxx_uart_open(priv);
	if (rc) {
		pr_err("UART open Failed : %d", rc);
		goto open_err;
	}

	*(uint32_t *)reg = (*(uint32_t *)reg) |
		IAXXX_UART_OP_UPDATE(IAXXX_MULTI_WRITE);
	rc = iaxxx_uart_write(priv, reg, reg_len);
	if (rc) {
		pr_err("Register write failed : %d", rc);
		goto err;
	}

	rc = iaxxx_uart_write(priv, &data_len, sizeof(uint32_t));
	if (rc) {
		pr_err("Length write failed : %d", rc);
		goto err;
	}

	rc = iaxxx_uart_write(priv, val, val_len);
	if (rc) {
		pr_err("Data write failed : %d", rc);
		goto err;
	}

	/* Wait till cmd is completely sent to chip */
	tty_wait_until_sent(uart_priv->uart->tty,
		    msecs_to_jiffies(IA_TTY_WAIT_TIMEOUT));
err:
	rc = iaxxx_uart_close(priv);
open_err:
	return rc;
}

static struct regmap_bus regmap_uart = {
	.write = iaxxx_regmap_uart_write,
	.read = iaxxx_regmap_uart_read,
	.gather_write = iaxxx_regmap_uart_gather_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

/* Register map initialization */
static int iaxxx_uart_regmap_init(struct iaxxx_priv *priv)
{
	int ret;
	struct device *dev = priv->dev;
	struct regmap *regmap;
	struct iaxxx_uart_priv *uart_priv = to_uart_priv(priv);

	if (!uart_priv || !priv->regmap_config) {
		pr_err("NULL input pointer(s)");
		return -EINVAL;
	}

	regmap = regmap_init(dev, &regmap_uart,
					uart_priv, priv->regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "initialize register map failed : %d\n", ret);
		return ret;
	}

	priv->regmap = regmap;
	return 0;
}

static int iaxxx_uart_reset_cb(struct device *dev)
{
	int retry = IAXXX_SYNC_RETRY;
	int rc = 0;
	uint32_t response;
	struct iaxxx_uart_priv *uart_priv = dev_get_drvdata(dev);

	if (!uart_priv) {
		pr_info("UART Private node is NULL");
		return -EINVAL;
	}

	do {
		retry--;
		/* Populate device tree data and Reset chip to SBL */
		rc = iaxxx_device_reset(&uart_priv->priv);
		if (rc) {
			dev_err(dev, "%s: device reset failed : %d\n",
				__func__, rc);
			break;
		}

		rc = iaxxx_uart_interface_detect(&uart_priv->priv);
		if (rc)
			continue;

		response = 0;
		/* Send a SYNC command */
		rc = iaxxx_uart_cmd(&uart_priv->priv, SBL_SYNC_CMD,
				    &response);
		if (rc || response != SBL_SYNC_CMD_RESPONSE) {
			dev_err(dev,
				"%s: SYNC cmd failed : %d response:0x%.08X\n",
				__func__, rc, response);

			/* if rc is 0, update it with proper error value */
			if (rc == 0)
				rc = -EINVAL;

			continue;
		}

		dev_info(dev, "SYNC response: 0x%08X\n", response);

#ifdef CONFIG_IAXXX_UART_HW_FLOW_CONTROL
		response = 0;
		/* Enable UART flow control */
		rc = iaxxx_uart_cmd(&uart_priv->priv, IAXXX_UART_FLOW_EN_CMD,
				    &response);
		if (rc || response != IAXXX_UART_FLOW_EN_CMD) {
			dev_err(dev,
				"%s: SYNC failed : %d response:0x%.08X\n",
				__func__, rc, response);

			/* if rc is 0, update it with proper error value */
			if (rc == 0)
				rc = -EINVAL;

			continue;
		}

		dev_info(dev, "UART_FLOW_EN_CMD response: 0x%08X\n", response);
#endif

		/* Switch the device into regmap mode */
		rc = iaxxx_uart_cmd(&uart_priv->priv, CMD_REGMAP_MODE, NULL);
		if (rc) {
			dev_err(dev,
				"%s: REGMAP MODE CMD failed : %d\n",
				__func__, rc);
			continue;
		}

	} while (rc && retry);

	return rc;
}

static void iaxxx_uart_32bit_plat_endian(void *data_buf, uint32_t len)
{
	int i;
	uint32_t *buf = data_buf;
	uint32_t data;

	for (i = 0; i < len; i++) {
		data = buf[i];
		buf[i] = cpu_to_be32(data);
	}
}

static int iaxxx_uart_bus_read(struct device *dev,
			uint32_t reg, void *read_buf, size_t words)
{
	struct iaxxx_uart_priv *uart_priv = dev_get_drvdata(dev);
	int rc;

	if (!read_buf || words <= 0) {
		pr_err("Invalid parameters");
		return -EINVAL;
	}

	reg = cpu_to_be32(reg);

	rc = iaxxx_regmap_uart_read(uart_priv, &reg, sizeof(reg),
				    read_buf, words * sizeof(uint32_t));

	if (!rc)
		iaxxx_uart_32bit_plat_endian(read_buf, words);

	return words;
}

static int iaxxx_uart_bus_write(struct device *dev,
		uint32_t reg, const void *write_buf, size_t words)
{
	struct iaxxx_uart_priv *uart_priv = dev_get_drvdata(dev);
	void *buf = (void *)write_buf;

	if (!write_buf || words <= 0) {
		pr_err("Invalid parameters");
		return -EINVAL;
	}

	reg = cpu_to_be32(reg);
	iaxxx_uart_32bit_plat_endian(buf, words);

	return iaxxx_regmap_uart_gather_write(uart_priv,
			&reg, sizeof(reg),
			buf, words * sizeof(uint32_t));
}

static int iaxxx_uart_bus_raw_read(struct iaxxx_priv *iaxxx, void *buf,
								int len)
{
	int rc;
	uint32_t header_len;

	/*
	 * If it is multi read then buffer has register address and data
	 * length else buffer has register address. Register endian is
	 * changed to big endian, so 24th and 25th bit has the operation
	 * information.
	 */
	if ((*((uint32_t *)buf) & IAXXX_UART_OP_UPDATE(IAXXX_MULTI_READ)))
		header_len = 8;
	else
		header_len = 4;

	rc = iaxxx_uart_open(iaxxx);
	if (rc) {
		dev_err(iaxxx->dev, "Open() failed : %d", rc);
		return -EIO;
	}

	rc = iaxxx_uart_write(iaxxx, buf, header_len);
	if (rc)
		goto err;

	rc = iaxxx_uart_read(iaxxx, (uint8_t *)buf + header_len,
			len - header_len);

err:
	iaxxx_uart_close(iaxxx);
	return rc;
}

static int iaxxx_uart_bus_raw_write(struct iaxxx_priv *iaxxx, const void *buf,
								int len)
{
	int rc;

	rc = iaxxx_uart_open(iaxxx);
	if (rc) {
		dev_err(iaxxx->dev,
		"%s: uart_open() failed : %d", __func__, rc);
		return -EIO;
	}

	rc = iaxxx_uart_write(iaxxx, buf, len);
	iaxxx_uart_close(iaxxx);
	return rc;
}

static int iaxxx_uart_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;
	struct iaxxx_uart_priv *uart_priv;
	struct iaxxx_uart_device *iaxxx_uart;

	/* Create driver private-data struct */
	uart_priv = devm_kzalloc(dev, sizeof(struct iaxxx_uart_priv),
						GFP_KERNEL);
	if (!uart_priv)
		return -ENOMEM;

	/* Create driver private-data struct */
	iaxxx_uart = devm_kzalloc(dev, sizeof(struct iaxxx_uart_device),
								GFP_KERNEL);
	if (!iaxxx_uart) {
		rc = -ENOMEM;
		goto uart_dev_mem_alloc_fail;
	}

	uart_priv->priv.dev = dev;
	uart_priv->priv.regmap_init_bus = iaxxx_uart_regmap_init;
	uart_priv->priv.bus_read = iaxxx_uart_bus_read;
	uart_priv->priv.bus_write = iaxxx_uart_bus_write;
	uart_priv->priv.bus_open = iaxxx_uart_open;
	uart_priv->priv.bus_close = iaxxx_uart_close;

	uart_priv->priv.bus = IAXXX_UART;
	uart_priv->priv.tunnel_method = IAXXX_POLL;
	uart_priv->priv.reset_cb = iaxxx_uart_reset_cb;
	dev_set_drvdata(dev, uart_priv);

	uart_priv->uart = iaxxx_uart;
	mutex_init(&uart_priv->uart->uart_lock);

#ifdef CONFIG_IAXXX_UART_HS_BAUD
	iaxxx_uart->baudrate = CONFIG_IAXXX_UART_HS_BAUD;
#else
	iaxxx_uart->baudrate = 2000000;
#endif

	ia_uart_kthread_init(uart_priv->uart);

	uart_priv->priv.intf_speed_addr =
		IAXXX_PWR_MGMT_MAX_UART_BAUDRATE_ADDR;
	uart_priv->priv.intf_max_speed = iaxxx_uart->baudrate;

	rc = iaxxx_device_init(&uart_priv->priv);
	if (rc) {
		dev_err(dev, "%s: device init failed : %d\n", __func__, rc);
		goto probe_failed;
	}

	/* Raw read write callbacks */
	uart_priv->priv.raw_ops->read = iaxxx_uart_bus_raw_read;
	uart_priv->priv.raw_ops->write = iaxxx_uart_bus_raw_write;

	return rc;

probe_failed:
	devm_kfree(dev, iaxxx_uart);
uart_dev_mem_alloc_fail:
	devm_kfree(dev, uart_priv);
	dev_set_drvdata(dev, NULL);
	pr_err("exit with error %d", rc);
	return rc;
}

static int iaxxx_uart_remove(struct platform_device *pdev)
{
	struct iaxxx_uart_priv *uart_priv = dev_get_drvdata(&pdev->dev);

	if (uart_priv) {
		iaxxx_device_exit(&uart_priv->priv);
		ia_uart_kthread_exit(uart_priv->uart);
		uart_priv->uart->file = NULL;
		devm_kfree(&pdev->dev, uart_priv->uart);
		devm_kfree(&pdev->dev, uart_priv);
	}

	return 0;
}

#ifdef ENABLE_LATER
static const struct dev_pm_ops iaxxx_uart_pm_ops = {
	.complete = iaxxx_pm_complete,
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_pm_suspend, iaxxx_pm_resume)
	SET_RUNTIME_PM_OPS(iaxxx_pm_runtime_suspend,
			iaxxx_pm_runtime_resume, NULL)
};
#endif

static struct platform_driver iaxxx_uart_driver = {
	.driver = {
		.name = "iaxxx-uart",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_uart_id,
		/* .pm = &iaxxx_uart_pm_ops, */
	},
	.probe = iaxxx_uart_probe,
	.remove = iaxxx_uart_remove,
};

static __init int iaxxx_uart_bus_init(void)
{
	int rc;

	rc = platform_driver_register(&iaxxx_uart_driver);
	if (rc)
		return rc;

	pr_debug("Registered iaxxx platform driver");
	return rc;
}

static __exit void iaxxx_uart_bus_exit(void)
{
	platform_driver_unregister(&iaxxx_uart_driver);
}

module_init(iaxxx_uart_bus_init);
module_exit(iaxxx_uart_bus_exit);

MODULE_DESCRIPTION("ASoC iaxxx driver");
MODULE_AUTHOR("Greg Clemson <gclemson@audience.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:iaxxx-codec");
