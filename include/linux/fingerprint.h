/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Fingerprint class header
 *
 * Copyright (C) 2021, Yassine Oudjana <Y.oudjana@protonmail.com>
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>

struct fingerprint_device;

struct fingerprint_ops {
	ssize_t (*capture)(struct fingerprint_device *fpdev, u8 *buf);
};

struct fingerprint_device {
	struct device dev;
	struct cdev cdev;
	void *drvdata;
	const char *name;
	size_t width;
	size_t height;
	size_t bytes_per_pixel;

	struct fingerprint_ops *ops;
	wait_queue_head_t wait;
	bool finger_changed;
	bool finger_down;
};

int fingerprint_register_device(struct device *parent,
				struct fingerprint_device *fpdev,
				void *drvdata);
void fingerprint_unregister_device(struct fingerprint_device *fpdev);
void fingerprint_report_finger(struct fingerprint_device *fpdev, bool down);
