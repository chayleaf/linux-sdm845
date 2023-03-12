// SPDX-License-Identifier: GPL-2.0
/*
 * Fingerprint class core
 *
 * Copyright (C) 2021, Yassine Oudjana <y.oudjana@protonmail.com>
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fingerprint.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>

static struct class *fingerprint_class;

static dev_t fingerprint_base_dev;
/* TODO: pick a proper maximum */
#define FINGERPRINT_MAX_CHAR_DEVICES 1024

static DEFINE_IDA(fingerprint_ida);

static ssize_t name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fingerprint_device *fpdev = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", fpdev->name);
}
static DEVICE_ATTR_RO(name);

static ssize_t width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fingerprint_device *fpdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", fpdev->width);
}
static DEVICE_ATTR_RO(width);

static ssize_t height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fingerprint_device *fpdev = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", fpdev->height);
}
static DEVICE_ATTR_RO(height);

static struct attribute *fingerprint_dev_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_width.attr,
	&dev_attr_height.attr,
	NULL,
};

static const struct attribute_group fingerprint_dev_attr_group = {
	.attrs = fingerprint_dev_attrs
};

static const struct attribute_group *fingerprint_dev_attr_groups[] = {
	&fingerprint_dev_attr_group,
	NULL
};

static const struct device_type fingerprint_dev_type = {
	.groups = fingerprint_dev_attr_groups
};

static int fingerprint_open(struct inode *inode, struct file *file)
{
	struct fingerprint_device *fpdev = container_of(inode->i_cdev,
						struct fingerprint_device,
						cdev);

	init_waitqueue_head(&fpdev->wait);

	file->private_data = fpdev;

	return 0;
}

static __poll_t fingerprint_poll(struct file *file, struct poll_table_struct *wait)
{
	struct fingerprint_device *fpdev = file->private_data;

	poll_wait(file, &fpdev->wait, wait);

	if (fpdev->finger_changed) {
		fpdev->finger_changed = false;
		return fpdev->finger_down ? EPOLLIN | EPOLLRDNORM : EPOLLHUP;
	}

	return 0;
}

static ssize_t fingerprint_read(struct file *file, char __user *buffer,
			size_t length, loff_t *ppos)
{
	struct fingerprint_device *fpdev = file->private_data;
	size_t capture_size = fpdev->width * fpdev->height * fpdev->bytes_per_pixel;
	u8 *buf;
	size_t captured_length;
	ssize_t len = min((ssize_t)capture_size - (ssize_t)*ppos, (ssize_t)length);
	ssize_t ret;

	if (len <= 0)
		return 0;

	if (file->f_flags & O_NONBLOCK && !fpdev->finger_down)
		return -EWOULDBLOCK;

	ret = wait_event_interruptible(fpdev->wait, fpdev->finger_down);
	if (ret)
		return ret;

	buf = kmalloc(sizeof(u8) * capture_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = fpdev->ops->capture(fpdev, buf);
	if (ret < 0)
		goto err;

	captured_length = min((size_t)ret, length);

	ret = copy_to_user(buffer + *ppos, buf, captured_length);
	if (ret < 0)
		goto err;

	*ppos += captured_length;
err:
	kfree(buf);

	if (ret < 0)
		return ret;

	return captured_length;
}

static const struct file_operations fingerprint_fops = {
	.owner = THIS_MODULE,
	.open = fingerprint_open,
	.poll = fingerprint_poll,
	.read = fingerprint_read
};

void fingerprint_report_finger(struct fingerprint_device *fpdev, bool down)
{
	fpdev->finger_changed = true;
	fpdev->finger_down = down;

	wake_up_interruptible(&fpdev->wait);
	dev_dbg(&fpdev->dev, "Woke up poll\n");
}
EXPORT_SYMBOL_GPL(fingerprint_report_finger);

/**
 * fingerprint_register_device() - register a fingerprint device
 * @parent: parent device struct
 * @fpdev: fingerprint device struct to register
 * @drvdata: pointer to be stored in fpdev for later use
 *
 * This function registers a new fingerprint device.
 * Returns 0 on success, -errno on failure.
 */
int fingerprint_register_device(struct device *parent,
				struct fingerprint_device *fpdev,
				void *drvdata)
{
	int minor, ret;

	if (!(fpdev->name &&
	      fpdev->width &&
	      fpdev->height &&
	      fpdev->bytes_per_pixel))
		return -EINVAL;

	minor = ida_alloc(&fingerprint_ida, GFP_KERNEL);

	device_initialize(&fpdev->dev);
	fpdev->dev.class = fingerprint_class;
	fpdev->dev.parent = parent;
	fpdev->dev.type = &fingerprint_dev_type;
	fpdev->dev.devt = MKDEV(MAJOR(fingerprint_base_dev), minor);
	dev_set_drvdata(&fpdev->dev, fpdev);
	ret = dev_set_name(&fpdev->dev, "fp%d", minor);
	if (ret)
		goto err;

	cdev_init(&fpdev->cdev, &fingerprint_fops);
	ret = cdev_device_add(&fpdev->cdev, &fpdev->dev);
	if (ret)
		goto err;

	fpdev->drvdata = drvdata;

	init_waitqueue_head(&fpdev->wait);

	return 0;
err:
	ida_free(&fingerprint_ida, minor);
	return ret;
}
EXPORT_SYMBOL_GPL(fingerprint_register_device);

/**
 * fingerprint_unregister_device() - unregister a fingerprint device
 * @fpdev: fingerprint device to unregister
 *
 * This function unregisters a fingerprint device.
 * Returns 0 on success, -errno on failure.
 */
void fingerprint_unregister_device(struct fingerprint_device *fpdev)
{
	cdev_device_del(&fpdev->cdev, &fpdev->dev);
}
EXPORT_SYMBOL_GPL(fingerprint_unregister_device);

static int __init fingerprint_init(void)
{
	int ret;

	fingerprint_class = class_create(THIS_MODULE, "fingerprint");
	if(IS_ERR(fingerprint_class))
		return PTR_ERR(fingerprint_class);

	ret = alloc_chrdev_region(&fingerprint_base_dev, 0,
				FINGERPRINT_MAX_CHAR_DEVICES, "fp");
	if (ret) {
		pr_err("Failed to register chardev region: %d\n", ret);
		goto err;
	}

	pr_info("Fingerprint class registered at major %d\n",
		MAJOR(fingerprint_base_dev));

	return 0;
err:
	class_destroy(fingerprint_class);
	return ret;
}

static void __exit fingerprint_exit(void)
{
	unregister_chrdev_region(fingerprint_base_dev,
				FINGERPRINT_MAX_CHAR_DEVICES);

	class_destroy(fingerprint_class);
}

subsys_initcall(fingerprint_init);
module_exit(fingerprint_exit);

MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com");
MODULE_DESCRIPTION("Fingerprint sensor class");
MODULE_LICENSE("GPL");
