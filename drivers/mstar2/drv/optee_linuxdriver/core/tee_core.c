/*
 * Copyright (c) 2014, STMicroelectronics International N.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
/* #define DEBUG */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/idr.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <asm-generic/ioctl.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include "linux/tee_core.h"
#include "linux/tee_ioc.h"

#include <linux/of.h>
#include <linux/time.h>
#include <linux/version.h>
#include "tee_core_priv.h"

#include "tee_sysfs.h"
#include "tee_debugfs.h"
#include "tee_shm.h"
#include "tee_supp_com.h"
#include "mdrv_types.h"
#include <linux/cpumask.h>
#include <linux/threads.h>

#define _TEE_CORE_FW_VER "1:0.1"

static char *_tee_supp_app_name = "tee-supplicant";

/* Store the class misc reference */
static struct class *misc_class;

static int device_match(struct device *device, const void *devname)
{
	struct tee *tee = dev_get_drvdata(device);
	int ret = strncmp(devname, tee->name, sizeof(tee->name));

	BUG_ON(!tee);
	if (ret == 0)
		return 1;
	else
		return 0;
}

/*
 * For the kernel api.
 * Get a reference on a device tee from the device needed
 */
struct tee *tee_get_tee(const char *devname)
{
	struct device *device;

	if (!devname)
		return NULL;
	device = class_find_device(misc_class, NULL, devname, device_match);
	if (!device) {
		pr_err("%s:%d - can't find device [%s]\n", __func__, __LINE__,
		       devname);
		return NULL;
	}

	return dev_get_drvdata(device);
}

void tee_inc_stats(struct tee_stats_entry *entry)
{
	entry->count++;
	if (entry->count > entry->max)
		entry->max = entry->count;
}

void tee_dec_stats(struct tee_stats_entry *entry)
{
	entry->count--;
}

#ifdef CONFIG_MSTAR_CHIP
atomic_t tee_sup_count = ATOMIC_INIT(0);

int tee_get_supplicant_count(void)
{
#ifndef CONFIG_TEE_2_4
	return (int)atomic_read(&tee_sup_count);
#else	//2.4 & 3.2
//	int tee_ver = get_optee_version();
	extern int optee_version;
	extern int _tee_get_supplicant_count(void);
	if (optee_version == 1) {
		//optee 1.0.1
		return (int)atomic_read(&tee_sup_count);
	}
	else {
		pr_info("%s optee_version:%d\n", __func__,optee_version);
		return _tee_get_supplicant_count();
	}
#endif
}
EXPORT_SYMBOL(tee_get_supplicant_count);

#ifndef CONFIG_TEE_2_4
void tee_client_close_context(struct tee_context *ctx __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return ;
}
void tee_client_get_version(struct tee_context *ctx __attribute__((unused)), struct tee_ioctl_version_data *vers __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return ;
}
int tee_client_open_session(struct tee_context *ctx __attribute__((unused)), struct tee_ioctl_open_session_arg *arg __attribute__((unused)), struct tee_param *param __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return 0;
}
int tee_client_close_session(struct tee_context *ctx __attribute__((unused)), u32 session __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return 0;
}
int tee_client_invoke_func(struct tee_context *ctx __attribute__((unused)), struct tee_ioctl_invoke_arg *arg __attribute__((unused)), struct tee_param *param __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return 0;
}
struct tee_context *tee_client_open_context(struct tee_context *start __attribute__((unused)),
			int (*match)(struct tee_ioctl_version_data *,
				const void *),
			const void *data __attribute__((unused)), struct tee_ioctl_version_data *vers __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return NULL;
}
void *tee_client_shm_get_va(struct tee_shm *shm __attribute__((unused)), size_t offs __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return 0;
}
struct tee_shm *tee_client_shm_alloc(struct tee_context *ctx __attribute__((unused)), size_t size __attribute__((unused)), u32 flags __attribute__((unused)))
{
	pr_err("%s This is for weak symbol\n", __func__);
	return 0;
}

EXPORT_SYMBOL(tee_client_close_context);
EXPORT_SYMBOL(tee_client_open_context);
EXPORT_SYMBOL(tee_client_get_version);
EXPORT_SYMBOL(tee_client_open_session);
EXPORT_SYMBOL(tee_client_close_session);
EXPORT_SYMBOL(tee_client_invoke_func);
EXPORT_SYMBOL(tee_client_shm_alloc);
EXPORT_SYMBOL(tee_client_shm_get_va);
#endif //CONFIG_TEE_2_4
#endif

/**
 * tee_get - increases refcount of the tee
 * @tee:	[in]	tee to increase refcount of
 *
 * @note: If tee.ops.start() callback function is available,
 * it is called when refcount is equal at 1.
 */
int tee_get(struct tee *tee)
{
	int ret = 0;

	BUG_ON(!tee);

	if (atomic_inc_return(&tee->refcount) == 1) {
		BUG_ON(!try_module_get(tee->ops->owner));
		dev_dbg(_DEV(tee), "%s: refcount=1 call %s::start()...\n",
			__func__, tee->name);
		get_device(tee->dev);
		if (tee->ops->start)
			ret = tee->ops->start(tee);
	}
	if (ret) {
		put_device(tee->dev);
		module_put(tee->ops->owner);
		dev_err(_DEV(tee), "%s: %s::start() failed, err=%d\n",
			__func__, tee->name, ret);
		atomic_dec(&tee->refcount);
	} else {
		int count = (int)atomic_read(&tee->refcount);

		dev_dbg(_DEV(tee), "%s: refcount=%d\n", __func__, count);
		if (count > tee->max_refcount)
			tee->max_refcount = count;
	}
	return ret;
}

/**
 * tee_put - decreases refcount of the tee
 * @tee:	[in]	tee to reduce refcount of
 *
 * @note: If tee.ops.stop() callback function is available,
 * it is called when refcount is equal at 0.
 */
int tee_put(struct tee *tee)
{
	int ret = 0;
	int count;

	BUG_ON(!tee);

	if (atomic_dec_and_test(&tee->refcount)) {
		dev_dbg(_DEV(tee), "%s: refcount=0 call %s::stop()...\n",
			__func__, tee->name);
		if (tee->ops->stop)
			ret = tee->ops->stop(tee);
		module_put(tee->ops->owner);
		put_device(tee->dev);
	}
	if (ret) {
		dev_err(_DEV(tee), "%s: %s::stop() has failed, ret=%d\n",
			__func__, tee->name, ret);
	}

	count = (int)atomic_read(&tee->refcount);
	dev_dbg(_DEV(tee), "%s: refcount=%d\n", __func__, count);
	return ret;
}

#ifdef CONFIG_MSTAR_CHIP
extern wait_queue_head_t wait_tee_supp;
#endif
static int tee_supp_open(struct tee *tee)
{
	int ret = 0;
#ifdef CONFIG_MSTAR_CHIP
	int cnt;
#endif

	dev_dbg(_DEV(tee), "%s: appclient=\"%s\" pid=%d\n", __func__,
		current->comm, current->pid);

	BUG_ON(!tee->rpc);

	if (strncmp(_tee_supp_app_name, current->comm,
			strlen(_tee_supp_app_name)) == 0) {
		if (atomic_add_return(1, &tee->rpc->used) > 1) {
			ret = -EBUSY;
			dev_err(tee->dev, "%s: ERROR Only one Supplicant is allowed\n",
					__func__);
			atomic_sub(1, &tee->rpc->used);
		}
#ifdef CONFIG_MSTAR_CHIP
		else {
			wake_up_interruptible (&wait_tee_supp);
		}
		cnt = atomic_read(&tee->rpc->used);
		atomic_set(&tee_sup_count, cnt);
	} else {
		if (atomic_read(&tee->rpc->used) == 0)
			dev_err(tee->dev, "\033[1;33m%s: Warning, Supplicant not ready\033[m\n",
				__func__);
#endif
	}

	return ret;
}

static void tee_supp_release(struct tee *tee)
{
#ifdef CONFIG_MSTAR_CHIP
	int cnt;
#endif

	dev_dbg(_DEV(tee), "%s: appclient=\"%s\" pid=%d\n", __func__,
		current->comm, current->pid);

	BUG_ON(!tee->rpc);

	if ((atomic_read(&tee->rpc->used) == 1) &&
			(strncmp(_tee_supp_app_name, current->comm,
					strlen(_tee_supp_app_name)) == 0)) {
		atomic_sub(1, &tee->rpc->used);
#ifdef CONFIG_MSTAR_CHIP
		cnt = atomic_read(&tee->rpc->used);
		atomic_set(&tee_sup_count, cnt);
#endif
	}
}

static int tee_ctx_open(struct inode *inode, struct file *filp)
{
	struct tee_context *ctx;
	struct tee *tee;
	int ret;

	tee = container_of(filp->private_data, struct tee, miscdev);

	BUG_ON(!tee);
	BUG_ON(tee->miscdev.minor != iminor(inode));

	dev_dbg(_DEV(tee), "%s: > name=\"%s\"\n", __func__, tee->name);

	ret = tee_supp_open(tee);
	if (ret)
		return ret;

	ctx = tee_context_create(tee);
	if (IS_ERR_OR_NULL(ctx))
		return PTR_ERR(ctx);

	ctx->usr_client = 1;
	filp->private_data = ctx;

	dev_dbg(_DEV(tee), "%s: < ctx=%p is created\n", __func__, (void *)ctx);

	return 0;
}

static int tee_ctx_release(struct inode *inode, struct file *filp)
{
	struct tee_context *ctx = filp->private_data;
	struct tee *tee;

	if (!ctx)
		return -EINVAL;

	BUG_ON(!ctx->tee);
	tee = ctx->tee;
	BUG_ON(tee->miscdev.minor != iminor(inode));

	dev_dbg(_DEV(tee), "%s: > ctx=%p\n", __func__, ctx);

	tee_context_destroy(ctx);
	tee_supp_release(tee);

	dev_dbg(_DEV(tee), "%s: < ctx=%p is destroyed\n", __func__, ctx);
	return 0;
}

static int tee_do_create_session(struct tee_context *ctx,
				 struct tee_cmd_io __user *u_cmd)
{
	int ret = -EINVAL;
	struct tee_cmd_io k_cmd;
	struct tee *tee;

	tee = ctx->tee;
	BUG_ON(!ctx->usr_client);

	dev_dbg(_DEV(tee), "%s: >\n", __func__);

	if (copy_from_user(&k_cmd, (void *)u_cmd, sizeof(struct tee_cmd_io))) {
		dev_err(_DEV(tee), "%s: copy_from_user failed\n", __func__);
		goto exit;
	}

	if (k_cmd.fd_sess > 0) {
		dev_err(_DEV(tee), "%s: invalid fd_sess %d\n", __func__,
			k_cmd.fd_sess);
		goto exit;
	}

	if ((k_cmd.op == NULL) || (k_cmd.uuid == NULL) ||
	    ((k_cmd.data != NULL) && (k_cmd.data_size == 0)) ||
	    ((k_cmd.data == NULL) && (k_cmd.data_size != 0))) {
		dev_err(_DEV(tee),
			"%s: op or/and data parameters are not valid\n",
			__func__);
		goto exit;
	}

	ret = tee_session_create_fd(ctx, &k_cmd);
	put_user(k_cmd.err, &u_cmd->err);
	put_user(k_cmd.origin, &u_cmd->origin);
	if (ret)
		goto exit;

	put_user(k_cmd.fd_sess, &u_cmd->fd_sess);

exit:
	dev_dbg(_DEV(tee), "%s: < ret=%d, sessfd=%d\n", __func__, ret,
		k_cmd.fd_sess);
	return ret;
}

static int tee_do_shm_alloc(struct tee_context *ctx,
			    struct tee_shm_io __user *u_shm)
{
	int ret = -EINVAL;
	struct tee_shm_io k_shm;
	struct tee *tee = ctx->tee;

	BUG_ON(!ctx->usr_client);

	dev_dbg(_DEV(tee), "%s: >\n", __func__);

	if (copy_from_user(&k_shm, (void *)u_shm, sizeof(struct tee_shm_io))) {
		dev_err(_DEV(tee), "%s: copy_from_user failed\n", __func__);
		goto exit;
	}

	if ((k_shm.buffer != NULL) || (k_shm.fd_shm != 0) ||
	    /*(k_shm.flags & ~(tee->shm_flags)) ||*/
	    ((k_shm.flags & tee->shm_flags) == 0) || (k_shm.registered != 0)) {
		dev_err(_DEV(tee),
			"%s: shm parameters are not valid %p %d %08x %08x %d\n",
			__func__, (void *)k_shm.buffer, k_shm.fd_shm,
			(unsigned int)k_shm.flags, (unsigned int)tee->shm_flags,
			k_shm.registered);
		goto exit;
	}

	ret = tee_shm_alloc_io(ctx, &k_shm);
	if (ret)
		goto exit;

	put_user(k_shm.fd_shm, &u_shm->fd_shm);
	put_user(k_shm.flags, &u_shm->flags);

exit:
	dev_dbg(_DEV(tee), "%s: < ret=%d, shmfd=%d\n", __func__, ret,
		k_shm.fd_shm);
	return ret;
}

static int tee_do_get_fd_for_rpc_shm(struct tee_context *ctx,
				     struct tee_shm_io __user *u_shm)
{
	int ret = -EINVAL;
	struct tee_shm_io k_shm;
	struct tee *tee = ctx->tee;

	dev_dbg(_DEV(tee), "%s: >\n", __func__);
	BUG_ON(!ctx->usr_client);

	if (copy_from_user(&k_shm, (void *)u_shm, sizeof(struct tee_shm_io))) {
		dev_err(_DEV(tee), "%s: copy_from_user failed\n", __func__);
		goto exit;
	}

	if ((k_shm.buffer == NULL) || (k_shm.size == 0) || (k_shm.fd_shm != 0)
	    || (k_shm.flags & ~(tee->shm_flags))
	    || ((k_shm.flags & tee->shm_flags) == 0)
	    || (k_shm.registered != 0)) {
		dev_err(_DEV(tee), "%s: shm parameters are not valid\n",
			__func__);
		goto exit;
	}

	ret = tee_shm_fd_for_rpc(ctx, &k_shm);
	if (ret)
		goto exit;

	put_user(k_shm.fd_shm, &u_shm->fd_shm);

exit:
	dev_dbg(_DEV(tee), "%s: < ret=%d, shmfd=%d\n", __func__, ret,
		k_shm.fd_shm);
	return ret;
}

#define CHAR_SIZE 	256
#define LOG_ARRAY_SIZE	8192

#define LOG_OUT 	0x80000000
#define LOG_SAVE 	0x40000000

char parse_name[CHAR_SIZE];
char file_name[CHAR_SIZE];
int name_count=0;
int file_name_count=0;
char *log_buffer[LOG_ARRAY_SIZE] = {NULL};
int log_buffer_init = 0;
int log_count = 0;
struct mutex tee_ramlog_lock2;
unsigned int log_out = 0;
unsigned int log_save = 0;
static struct task_struct *tee_ramlog_tsk2 = NULL;

void _TIME_PRINTK(char *buf,struct timespec now,long diff,uint32_t index)
{
	unsigned long long time_us,diff_us;

	time_us = now.tv_nsec;
	diff_us = diff;

	do_div(time_us,NSEC_PER_USEC);
	do_div(diff_us,NSEC_PER_USEC);
	if(log_save != 0 && log_buffer_init == 1){
		if( name_count != 0 && strncmp(current->comm,parse_name,name_count-1) != 0)
			return;

		if(log_out != 0)
			printk("\033[0;32;31m %64s p0 %4x tv_sec %4ld tv_us %6lld diff (us) %6lld pid %4d tgid %4d name %s\033[m\n",buf , index,now.tv_sec, time_us, diff_us, current->pid, current->tgid, current->comm);

		mutex_lock(&tee_ramlog_lock2);
		snprintf(log_buffer[log_count++], CHAR_SIZE,"%64s p0 %4x tv_sec %4ld tv_us %6lld diff (us) %6lld pid %4d tgid %4d name %s\n",buf , index,now.tv_sec, time_us, diff_us, current->pid, current->tgid, current->comm);
		log_count %= LOG_ARRAY_SIZE;
		mutex_unlock(&tee_ramlog_lock2);
	}

}
EXPORT_SYMBOL(_TIME_PRINTK);

static inline unsigned int _is_user_addr(unsigned long addr)
{
	if ((addr & 0xffffffc000000000) == 0xffffffc000000000)
		return 0; // kernel address
	else
		return 1;
}

static long tee_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	struct tee_context *ctx = filp->private_data;
	void __user *u_arg;
#ifdef MSTAR_MEASURE_TIME
	struct timespec start,end;
#endif
	BUG_ON(!ctx);
	BUG_ON(!ctx->tee);

	dev_dbg(_DEV(ctx->tee), "%s: > cmd nr=%d\n", __func__, _IOC_NR(cmd));

#ifdef CONFIG_COMPAT
	if (is_compat_task() && _is_user_addr(arg))
		u_arg = compat_ptr(arg);
	else
		u_arg = (void __user *)arg;
#else
	u_arg = (void __user *)arg;
#endif

	switch (cmd) {
	case TEE_OPEN_SESSION_IOC:
		TIME_PRINTK_START("TEE_OPEN_SESSION_IOC ENTER",start,0);
		ret = tee_do_create_session(ctx,
				(struct tee_cmd_io __user *)u_arg);
		TIME_PRINTK_END("TEE_OPEN_SESSION_IOC EXIT",start,end,0);
		break;
	case TEE_ALLOC_SHM_IOC:
		TIME_PRINTK_START("TEE_ALLOC_SHM_IOC_IOC ENTER",start,0);
		ret = tee_do_shm_alloc(ctx, (struct tee_shm_io __user *)u_arg);
		TIME_PRINTK_END("TEE_ALLOC_SHM_IOC_IOC EXIT",start,end,0);
		break;
	case TEE_GET_FD_FOR_RPC_SHM_IOC:
		TIME_PRINTK_START("TEE_GET_FD_FOR_RPC_SHM_IOC ENTER",start,0);
		ret = tee_do_get_fd_for_rpc_shm(ctx,
				 (struct tee_shm_io __user *)u_arg);
		TIME_PRINTK_END("TEE_GET_FD_FOR_RPC_SHM_IOC EXIT",start,end,0);
		break;
#ifdef CONFIG_MSTAR_CHIP
	case TEE_SET_LOG_LEVEL:
		{
			uint32_t k_level = 0;
			struct tee *tee;
			if (copy_from_user(&k_level, (void *)u_arg, sizeof(uint32_t))) {
				printk("\033[0;32;31m [SECARM] %s %d Error copy_from_user\033[m\n",__func__,__LINE__);
				break;
			}
			tee = ctx->tee;
			tee->ops->set_log_level(k_level, tee);
			break;
		}
#endif
	case TEE_PM51CTL_IOC:
		{
			uint32_t cmd = -1;
			struct tee *tee;
			if (copy_from_user(&cmd, (void *)u_arg, sizeof(uint32_t))) {
				printk("\033[0;32;31m %s %d Error copy_from_user\033[m\n",__func__,__LINE__);
				break;
			}
			tee = ctx->tee;
			ret = tee->ops->pm51ctl(cmd, tee);
			break;
		}
	default:
		ret = -ENOSYS;
		break;
	}

	dev_dbg(_DEV(ctx->tee), "%s: < ret=%d\n", __func__, ret);

	return ret;
}

static const struct file_operations tee_fops = {
	.owner = THIS_MODULE,
	.read = tee_supp_read,
	.write = tee_supp_write,
	.open = tee_ctx_open,
	.release = tee_ctx_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_ioctl,
#endif
	.unlocked_ioctl = tee_ioctl
};

static void tee_plt_device_release(struct device *dev)
{
	pr_debug("%s: (dev=%p)....\n", __func__, dev);
}

struct tee *tee_core_alloc(struct device *dev, char *name, int id,
			   const struct tee_ops *ops, size_t len)
{
	struct tee *tee;

	if (!dev || !name || !ops ||
	    !ops->open || !ops->close || !ops->alloc || !ops->free)
		return NULL;

	tee = devm_kzalloc(dev, sizeof(struct tee) + len, GFP_KERNEL);
	if (!tee) {
		dev_err(dev, "%s: kzalloc failed\n", __func__);
		return NULL;
	}

	if (!dev->release)
		dev->release = tee_plt_device_release;

	tee->dev = dev;
	tee->id = id;
	tee->ops = ops;
	tee->priv = &tee[1];

	snprintf(tee->name, sizeof(tee->name), "optee%s%02d", name, tee->id);
	pr_info("TEE core: Alloc the misc device \"%s\" (id=%d)\n", tee->name,
		tee->id);

	tee->miscdev.parent = dev;
	tee->miscdev.minor = MISC_DYNAMIC_MINOR;
	tee->miscdev.name = tee->name;
	tee->miscdev.fops = &tee_fops;

	mutex_init(&tee->lock);
	atomic_set(&tee->refcount, 0);
	INIT_LIST_HEAD(&tee->list_ctx);
	INIT_LIST_HEAD(&tee->list_rpc_shm);

	tee->state = TEE_OFFLINE;
	tee->shm_flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT;
	tee->test = 0;

	tee_supp_init(tee);

	return tee;
}
EXPORT_SYMBOL(tee_core_alloc);

int tee_core_free(struct tee *tee)
{
	if (tee) {
		tee_supp_deinit(tee);
		devm_kfree(tee->dev, tee);
	}
	return 0;
}
EXPORT_SYMBOL(tee_core_free);

int tee_core_add(struct tee *tee)
{
	int rc = 0;

	if (!tee)
		return -EINVAL;

	rc = misc_register(&tee->miscdev);
	if (rc != 0) {
		pr_err("TEE Core: misc_register() failed name=\"%s\"\n",
		       tee->name);
		return rc;
	}

	dev_set_drvdata(tee->miscdev.this_device, tee);

	tee_init_sysfs(tee);
	tee_create_debug_dir(tee);

	/* Register a static reference on the class misc
	 * to allow finding device by class */
	BUG_ON(!tee->miscdev.this_device->class);
	if (misc_class)
		BUG_ON(misc_class != tee->miscdev.this_device->class);
	else
		misc_class = tee->miscdev.this_device->class;

	pr_info("TEE Core: Register the misc device \"%s\" (id=%d,minor=%d)\n",
		dev_name(tee->miscdev.this_device), tee->id,
		tee->miscdev.minor);
	return rc;
}
EXPORT_SYMBOL(tee_core_add);

int tee_core_del(struct tee *tee)
{
	if (tee) {
		pr_info("TEE Core: Destroy the misc device \"%s\" (id=%d)\n",
			dev_name(tee->miscdev.this_device), tee->id);

		tee_cleanup_sysfs(tee);
		tee_delete_debug_dir(tee);

		if (tee->miscdev.minor != MISC_DYNAMIC_MINOR) {
			pr_info("TEE Core: Deregister the misc device \"%s\" (id=%d)\n",
			     dev_name(tee->miscdev.this_device), tee->id);
			misc_deregister(&tee->miscdev);
		}
	}

	tee_core_free(tee);

	return 0;
}
EXPORT_SYMBOL(tee_core_del);

#ifdef CONFIG_MSTAR_CHIP
#include <linux/proc_fs.h>

static ssize_t tz_write_name(struct file *filp, const char __user *buffer,
                                    size_t count, loff_t *ppos)
{
	char local_buf[256];
	if(count>=256)
		return -EINVAL;

	if (copy_from_user(local_buf, buffer, count))
		return -EFAULT;

	memcpy(parse_name, local_buf, sizeof(parse_name));

	printk("\033[0;32;31m %s %d %s %zu\033[m\n",__func__,__LINE__,parse_name,count);
	name_count = count;

	return count;
}

static ssize_t tz_write_file_name(struct file *filp, const char __user *buffer,
                                    size_t count, loff_t *ppos)
{
	char local_buf[256];
	if(count>=256)
		return -EINVAL;

	if (copy_from_user(local_buf, buffer, count))
		return -EFAULT;

	memcpy(file_name, local_buf, sizeof(file_name));

	printk("\033[0;32;31m %s %d %s %zu\033[m\n",__func__,__LINE__,file_name,count);
	file_name_count = count;

	return count;
}

static int ram_count = 0;
static int tee_ramlog_loop2(void *p)
{
	struct file *fp = NULL;
	mm_segment_t oldfs;

	if(file_name_count == 0)
		fp=filp_open("/data/tee_log",O_RDWR | O_CREAT,0644);
	else
		fp=filp_open(file_name,O_RDWR | O_CREAT,0644);

	printk("\033[0;32;31m %s %d \033[m\n",__func__,__LINE__);

	while(1){
		oldfs = get_fs();
		set_fs(KERNEL_DS);

		if (fp!=NULL)
		{
			if (!IS_ERR(fp)) {
				mutex_lock(&tee_ramlog_lock2);
				while(ram_count != log_count){
					fp->f_op->write(fp, log_buffer[ram_count++], CHAR_SIZE, &fp->f_pos);
					ram_count %= LOG_ARRAY_SIZE;
				}
				mutex_unlock(&tee_ramlog_lock2);
			}
		}
		set_fs(oldfs);
		msleep(50);
	}
	return 0;
}
static ssize_t tz_write(struct file *filp, const char __user *buffer,
                                    size_t count, loff_t *ppos)
{
	char local_buf[256];
	uint32_t loglevel;
	struct tee *tee;
	int i;

	if(count>=256)
		return -EINVAL;

	if (copy_from_user(local_buf, buffer, count))
		return -EFAULT;

	local_buf[count] = 0;

	loglevel = simple_strtol(local_buf,NULL,10);

	printk("\033[0;32;31m [SECARM] %s %d set_log_level %d\033[m\n",__func__,__LINE__,loglevel);

	tee = tee_get_tee("opteearmtz00");
	if (!tee) {
		pr_err("%s - can't get device [%s]\n", __func__, "opteearmtz00");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	if(loglevel & LOG_OUT)
		log_out = 1;
	else
		log_out = 0;

	if(loglevel & LOG_SAVE)
		log_save = 1;
	else
		log_save = 0;

	if(tee->ops->set_log_level && ((loglevel & (LOG_OUT|LOG_SAVE)) == 0))
	{
		tee->ops->set_log_level(loglevel,tee);
	}else
	{
		if(log_buffer_init == 0){

			log_buffer[0] = kzalloc(sizeof(char)*LOG_ARRAY_SIZE*CHAR_SIZE, GFP_KERNEL);
			if(log_buffer[0] == NULL){
				printk("\033[0;32;31m  %s %d alloc fail\033[m\n",__func__,__LINE__);
				return count;
			}

			for(i = 1;i < LOG_ARRAY_SIZE; i++){
				log_buffer[i] = log_buffer[0] + (sizeof(char)*i*CHAR_SIZE);
				if(log_buffer[i] == NULL){
					printk("\033[0;32;31m  %s %d alloc fail\033[m\n",__func__,__LINE__);
					return count;
				}
			}
			log_buffer_init = 1;
		}
		if(tee_ramlog_tsk2 == NULL)
			tee_ramlog_tsk2 = kthread_run(tee_ramlog_loop2, NULL, "tee_ramlog_loop2");
		printk("\033[0;32;31m  %s %d log_out = %d log_save %d\033[m\n",__func__,__LINE__,log_out,log_save);
	}

	return count;
}

static ssize_t tz_write_ramlog_addr(struct file *filp, const char __user *buffer,
                                    size_t count, loff_t *ppos)
{
	char local_buf[256];
	char* const delim = " ";
  	char *token, *cur;
	int i;
	uint32_t param_value[2];
	struct tee *tee;
	long val;

	if(count >= 256)
		return -EINVAL;

	if (copy_from_user(local_buf, buffer, count))
		return -EFAULT;

	local_buf[count] = 0;
	cur = local_buf;

	for (i = 0 ; i < (sizeof(param_value)/sizeof(param_value[0])) ; i++) {
		token = strsep(&cur, delim);

		if (!token)
			return -EINVAL;

		if (kstrtol(token, 0, &val))
			return -EINVAL;

		param_value[i] = (uint32_t)val;
	}

	printk("\033[0;32;31m [RAMLOG] %s addr = 0x%x , length = %d\033[m\n",
		__func__,param_value[0],param_value[1]);

	tee = tee_get_tee("opteearmtz00");
	if (!tee) {
		pr_err("%s - can't get device [%s]\n", __func__, "opteearmtz00");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	if(tee->ops->set_log_addr){
		tee->ops->set_log_addr(param_value[0],param_value[1]);
	}else
		pr_err("%s - can't configure ramlog [%s]\n", __func__, ".set_log_addr");

	return count;
}

static ssize_t crc_write(struct file *filp, const char __user *buffer,
                                    size_t count, loff_t *ppos)
{
	char local_buf[256];
	char* const delim = " ";
  	char *token, *cur;
	int i;
	uint32_t param_value[3];
	struct tee *tee;
	long val;

	if(count >= 256)
		return -EINVAL;

	if (copy_from_user(local_buf, buffer, count))
		return -EFAULT;

	local_buf[count] = 0;
	cur = local_buf;

	for (i = 0 ; i < 3 ; i++) {
		token = strsep(&cur, delim);

		if (!token)
			return -EINVAL;

		if (kstrtol(token, 0, &val))
			return -EINVAL;

		param_value[i] = (uint32_t)val;
	}

	printk("\033[0;32;31m [SECARM] %s miu = %d ,offset = 0x%x ,size = 0x%x\033[m\n",
		__func__,param_value[0],param_value[1],param_value[2]);

	tee = tee_get_tee("opteearmtz00");
	if (!tee) {
		pr_err("%s - can't get device [%s]\n", __func__, "opteearmtz00");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	if(tee->ops->configure_crc_args){
		tee->ops->configure_crc_args(param_value[0],param_value[1],param_value[2]);
	}else
		pr_err("%s - can't configure crc args [%s]\n", __func__, "configure_crc_args");

	return count;
}

static ssize_t tz_write_nagra_warmboot(struct file *filp, const char __user *buffer,
                                    size_t count, loff_t *ppos)
{
	char local_buf[256];
	char* const delim = " ";
  	char *token, *cur;
	uint32_t param_value;
	struct tee *tee;
	long val;

	if (count >= 256)
		return -EINVAL;

	if (copy_from_user(local_buf, buffer, count))
		return -EFAULT;

	local_buf[count] = 0;
	cur = local_buf;
	token = strsep(&cur, delim);

	if (kstrtol(token, 0, &val))
		return -EINVAL;

	param_value = (uint32_t)val;

	tee = tee_get_tee("opteearmtz00");
	if (!tee) {
		pr_err("%s - can't get device [%s]\n", __func__, "opteearmtz00");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	if (tee->ops->configure_crc_args) {
		tee->ops->set_nagra_warmboot(param_value, tee);
	} else
		pr_err("%s - can't configure nagra_warmboot args\n", __func__);

	return count;
}

static long tee_xtest_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	void __user *u_arg;
	uint32_t result = 0;
	extern unsigned int setup_max_cpus;
#ifdef CONFIG_COMPAT
	if (is_compat_task() && _is_user_addr(arg))
		u_arg = compat_ptr(arg);
	else
		u_arg = (void __user *)arg;
#else
	u_arg = (void __user *)arg;
#endif

	switch (cmd) {
		case TEE_XTEST_SUPPLICANT_CNT_IOC:
			result = tee_get_supplicant_count();
			pr_info("%s %d supplicant cnt:%d\n", __func__, __LINE__, result);
			if (result) {
				put_user(TEEC_SUCCESS,(unsigned int __user *) u_arg);
				ret = 0;
			}
			else {
				put_user(TEEC_ERROR_GENERIC, (unsigned int __user *)u_arg);
				ret = -ENOSYS;
			}
			break;
		case TEE_XTEST_CHECK_SMP_IOC:
			result = num_online_cpus();
			pr_info("%s %d The number of online core:%d,\
and expect setup_max_cpus:%d\n", __func__, __LINE__, result, setup_max_cpus);
			if (result == setup_max_cpus) {
				put_user(TEEC_SUCCESS, (unsigned int __user *)u_arg);
				ret = 0;
			}
			else {
				put_user(TEEC_ERROR_GENERIC, (unsigned int __user *)u_arg);
				ret = -ENOSYS;
			}
			break;
		default:
			ret = -ENOSYS;
		break;
	}
	return ret;
}

static struct file_operations crc_fops = {
	.owner   = THIS_MODULE, // system
	.write   = crc_write,
};

static struct file_operations tz_fops = {
	.owner   = THIS_MODULE, // system
	.write   = tz_write,
};

static struct file_operations tz_fops2 = {
	.owner   = THIS_MODULE, // system
	.write   = tz_write_name,
};

static struct file_operations tz_fops3 = {
	.owner   = THIS_MODULE, // system
	.write   = tz_write_file_name,
};

static struct file_operations tz_fops4 = {
	.owner   = THIS_MODULE, // system
	.write   = tz_write_nagra_warmboot,
};

static struct file_operations ramlog_fops = {
	.owner   = THIS_MODULE, // system
	.write   = tz_write_ramlog_addr,
};

static struct file_operations tz_fops_misc = {
	.owner   = THIS_MODULE,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_xtest_ioctl,
#endif
	.unlocked_ioctl = tee_xtest_ioctl,
};

#define USER_ROOT_DIR "tz_mstar"
static struct proc_dir_entry *tz_root;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
static const struct of_device_id optee_match[] = {
	{ .compatible = "linaro,optee-tz-1.0.1" },
	{},
};
#endif

#define XTEST_ROOT_DIR "mtk_xtest"
static struct proc_dir_entry *xtest_root;
static int __init tee_create_xtest(void)
{
	struct proc_dir_entry *timeout;
	xtest_root = proc_mkdir(XTEST_ROOT_DIR, NULL);
	if (NULL == xtest_root)
	{
		printk(KERN_ALERT "Create dir /proc/%s error!\n",XTEST_ROOT_DIR);
		return -1;
	}

	timeout = proc_create("optee_xtest", 0644, xtest_root, &tz_fops_misc);
	if (!timeout){
		printk(KERN_ALERT "Create dir /proc/%s/optee_xtest error!\n",XTEST_ROOT_DIR);
		return -ENOMEM;
	}
	return 0;
}

static int __init tee_core_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	struct device_node *fw_np;
	struct device_node *np;

	if (tee_create_xtest())
		pr_err("%s - tee_create_xtest init fail!\n", __func__);

	/* Node is supposed to be below /firmware */
	fw_np = of_find_node_by_name(NULL, "firmware");
	if (!fw_np)
		return -ENODEV;

	np = of_find_matching_node(fw_np, optee_match);
	if (!np)
		return -ENODEV;
#else
	if (tee_create_xtest())
		pr_err("%s - tee_create_xtest init fail!\n", __func__);
#endif
	if(TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_SECARM) {
		printk(KERN_ALERT "This kernel verison is not support OSTYPE_SECARM!\n");
		return -EFAULT;
	}

	if(TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_OPTEE) {
#ifdef CONFIG_MSTAR_CHIP
		struct proc_dir_entry *timeout;

		tz_root = proc_mkdir(USER_ROOT_DIR, NULL);
		if (NULL==tz_root)
		{
			printk(KERN_ALERT "Create dir /proc/%s error!\n",USER_ROOT_DIR);
			return -1;
		}

		timeout = proc_create("log_level", 0644, tz_root, &tz_fops);
		if (!timeout){
			printk(KERN_ALERT "Create dir /proc/%s/log_level error!\n",USER_ROOT_DIR);
			return -ENOMEM;
		}

		timeout = proc_create("config_crc", 0644, tz_root, &crc_fops);
		if (!timeout){
			printk(KERN_ALERT "Create dir /proc/%s/config_crc error!\n",USER_ROOT_DIR);
			return -ENOMEM;
		}

		timeout = proc_create("name", 0644, tz_root, &tz_fops2);
		if (!timeout){
			printk(KERN_ALERT "Create dir /proc/%s/name error!\n",USER_ROOT_DIR);
			return -ENOMEM;
		}

		timeout = proc_create("file_name", 0644, tz_root, &tz_fops3);
		if (!timeout){
			printk(KERN_ALERT "Create dir /proc/%s/file_name error!\n",USER_ROOT_DIR);
			return -ENOMEM;
		}

		timeout = proc_create("nagra_warmboot", 0644, tz_root, &tz_fops4);
		if (!timeout){
			printk(KERN_ALERT "Create dir /proc/%s/nagra_warmboot error!\n",USER_ROOT_DIR);
			return -ENOMEM;
		}

		timeout = proc_create("ramlog_setup", 0644, tz_root, &ramlog_fops);
		if (!timeout){
			printk(KERN_ALERT "Create dir /proc/%s/ramlog_setup error!\n",USER_ROOT_DIR);
			return -ENOMEM;
		}

		mutex_init(&tee_ramlog_lock2);
		memset(parse_name,0,sizeof(parse_name));
#endif

		pr_info("\nTEE Core Framework initialization (ver %s)\n",
			_TEE_CORE_FW_VER);
		tee_init_debugfs();
	}
	return 0;

}
static void __exit tee_core_exit(void)
{
	tee_exit_debugfs();
	pr_info("TEE Core Framework unregistered\n");

#ifdef CONFIG_MSTAR_CHIP
	remove_proc_entry(USER_ROOT_DIR, tz_root);
	remove_proc_entry(XTEST_ROOT_DIR, xtest_root);
#endif
}
module_init(tee_core_init);
module_exit(tee_core_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("STM Secure TEE Framework/Core TEEC v1.0");
MODULE_SUPPORTED_DEVICE("");
MODULE_VERSION(_TEE_CORE_FW_VER);
MODULE_LICENSE("GPL");
