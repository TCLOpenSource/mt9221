/*
 * iaxxx-cdev.c  --  iaxxx character device support
 *
 * Copyright 2017 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "iaxxx: %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h>

#include "iaxxx.h"
#include "iaxxx-cdev.h"

#define IAXXX_CDEV_MINOR	0
#define IAXXX_CDEV_COUNT	IAXXX_CDEV_LAST

static struct class *iaxxx_cdev_class;
static dev_t iaxxx_cdev_major;
static dev_t iaxxx_cdev_minor;
static dev_t iaxxx_cdev_indexes[IAXXX_CDEV_LAST] = { 0 };

static char *iaxxx_cdev_get_name(enum iaxxx_cdev_types type)
{
	switch (type) {
	case IAXXX_CDEV_TUNNEL:
		return "tunnel%d";
	case IAXXX_CDEV_REGDUMP:
		return "regdump%d";
	case IAXXX_CDEV_RAWDEV:
		return "raw_dev%d";
	case IAXXX_CDEV_CRASHDUMP:
		return "crashdump%d";
	case IAXXX_CDEV_DBGLOG:
		return "debug_log%d";
	default:
		break;
	}

	return NULL;
}

int iaxxx_cdev_create(struct iaxxx_cdev *cdev,
	struct device *parent,
	const struct file_operations *fops,
	void *drvdata, u32 dev_id, enum iaxxx_cdev_types type)
{
	dev_t devno;
	char *name;
	int idx, err = -EINVAL;

	name = iaxxx_cdev_get_name(type);
	if (!name)
		return -EINVAL;

	cdev_init(&cdev->cdev, fops);
	cdev->cdev.owner = THIS_MODULE;

	devno = MKDEV(iaxxx_cdev_major, iaxxx_cdev_minor);
	err = cdev_add(&cdev->cdev, devno, 1);
	if (err) {
		pr_err("add cdev=0x%04x failed : %d", devno, err);
		goto exit_cdev_add;
	}

	idx = iaxxx_cdev_indexes[type];
	cdev->dev = device_create(iaxxx_cdev_class, parent, devno,
				drvdata, name, dev_id);
	if (IS_ERR(cdev->dev)) {
		err = PTR_ERR(cdev->dev);
		pr_err("device_create cdev=0x%04x failed : %d", devno, err);
		goto exit_dev_create;
	}

	iaxxx_cdev_minor++;
	iaxxx_cdev_indexes[type]++;

	return 0;

exit_dev_create:
	cdev_del(&cdev->cdev);
exit_cdev_add:
	return err;
}
EXPORT_SYMBOL(iaxxx_cdev_create);

void iaxxx_cdev_destroy(struct iaxxx_cdev *cdev, enum iaxxx_cdev_types type)
{
	if (iaxxx_cdev_minor > 0) {
		iaxxx_cdev_minor--;
		iaxxx_cdev_indexes[type]--;
	}
	device_destroy(iaxxx_cdev_class, cdev->cdev.dev);
	cdev_del(&cdev->cdev);
}
EXPORT_SYMBOL(iaxxx_cdev_destroy);

int iaxxx_cdev_init(void)
{
	static const char *cdev_name = "iaxxx-dev";
	dev_t devno;
	int err;

	if (iaxxx_cdev_class)
		return 0;

	err = alloc_chrdev_region(&devno, 0, IAXXX_CDEV_COUNT, cdev_name);
	if (err) {
		pr_err("allocate char dev failed : %d", err);
		return err;
	}
	iaxxx_cdev_major = MAJOR(devno);

	/* register device class */
	iaxxx_cdev_class = class_create(THIS_MODULE, cdev_name);
	if (IS_ERR(iaxxx_cdev_class)) {
		err = PTR_ERR(iaxxx_cdev_class);
		iaxxx_cdev_class = NULL;
		pr_err("create %s class failed : %d", cdev_name, err);
		return err;
	}

	return 0;
}

void iaxxx_cdev_exit(void)
{
	class_destroy(iaxxx_cdev_class);
	unregister_chrdev_region(MKDEV(iaxxx_cdev_major, 0), IAXXX_CDEV_COUNT);
}
