/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 * Copyright (C) 2018 XiaoMi, Inc.
 * Copyright (c) 2021 Caleb Connolly <caleb@connolly.tech>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include <linux/vmalloc.h>
#include <linux/notifier.h>

#include "focaltech_fts.h"

#define FTS_DRIVER_NAME "fts_ts"
#define INTERVAL_READ_REG 100 /* unit:ms */
#define TIMEOUT_READ_REG 2000 /* unit:ms */
#define FTS_VDD_MIN_UV 2600000
#define FTS_VDD_MAX_UV 3300000
#define FTS_I2C_VCC_MIN_UV 1800000
#define FTS_I2C_VCC_MAX_UV 1800000

#define INPUT_EVENT_START 0
#define INPUT_EVENT_SENSITIVE_MODE_OFF 0
#define INPUT_EVENT_SENSITIVE_MODE_ON 1
#define INPUT_EVENT_STYLUS_MODE_OFF 2
#define INPUT_EVENT_STYLUS_MODE_ON 3
#define INPUT_EVENT_WAKUP_MODE_OFF 4
#define INPUT_EVENT_WAKUP_MODE_ON 5
#define INPUT_EVENT_COVER_MODE_OFF 6
#define INPUT_EVENT_COVER_MODE_ON 7
#define INPUT_EVENT_SLIDE_FOR_VOLUME 8
#define INPUT_EVENT_DOUBLE_TAP_FOR_VOLUME 9
#define INPUT_EVENT_SINGLE_TAP_FOR_VOLUME 10
#define INPUT_EVENT_LONG_SINGLE_TAP_FOR_VOLUME 11
#define INPUT_EVENT_PALM_OFF 12
#define INPUT_EVENT_PALM_ON 13
#define INPUT_EVENT_END 13
#define I2C_RETRY_NUMBER 3

static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

static DEFINE_MUTEX(i2c_rw_access);

/// TODO: rewrite the i2c xfer functions
int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen,
		 char *readbuf, int readlen)
{
	int ret = 0;
	int msg_count = !!writelen + 1;
	struct i2c_msg msgs[2];

	if (readlen < 0 || writelen < 0) {
		return -EINVAL;
	}

	// If writelen is zero then only populate msgs[0].
	// otherwise we read into msgs[1]
	msgs[msg_count-1].len = readlen;
	msgs[msg_count-1].buf = readbuf;
	msgs[msg_count-1].addr = client->addr;
	msgs[msg_count-1].flags = I2C_M_RD;

	if (writelen > 0) {
		msgs[0].len = writelen;
		msgs[0].buf = writebuf;
		msgs[0].addr = client->addr;
		msgs[0].flags = 0;
	}

	mutex_lock(&i2c_rw_access);

	ret = i2c_transfer(client->adapter, msgs, msg_count);

	mutex_unlock(&i2c_rw_access);
	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;
	struct i2c_msg msg;

	if (writelen <= 0)
		return -EINVAL;

	msg.addr = client->addr,
	msg.flags = 0,
	msg.len = writelen,
	msg.buf = writebuf,

	mutex_lock(&i2c_rw_access);
	//for (i = 0; i < I2C_RETRY_NUMBER; i++) {
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		FTS_ERROR(
			"%s: fts_i2c_write failed, ret=%d",
			__func__, ret);
	}
	//}
	mutex_unlock(&i2c_rw_access);

	return ret;
}

int fts_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	u8 buf[2] = { 0 };

	buf[0] = regaddr;
	buf[1] = regvalue;
	return fts_i2c_write(client, buf, sizeof(buf));
}

int fts_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

int fts_wait_ready(struct fts_ts_data *data)
{
	int ret = 0;
	int cnt = 0;
	u8 reg_value = 0;
	struct i2c_client *client = data->client;
	u8 chip_id = data->chip_type->chip_idh;

	do {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);
		if ((ret < 0) || (reg_value != chip_id)) {
		} else if (reg_value == chip_id) {
			FTS_DEBUG("TP Ready, Device ID = 0x%x, count = %d", reg_value, cnt);
			return 0;
		}
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	return -EIO;
}

static int fts_get_chip_types(struct fts_ts_data *ts_data, u8 id_h, u8 id_l,
			      bool fw_valid)
{
	if ((0x0 == id_h) || (0x0 == id_l)) {
		FTS_ERROR("id_h or id_l is 0");
		return -EINVAL;
	}

	FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
	if (((id_h != ts_data->chip_type->rom_idh) ||
		(id_l != ts_data->chip_type->rom_idl)) &&
		((id_h != ts_data->chip_type->pb_idh) ||
		(id_l != ts_data->chip_type->pb_idl)) &&
		((id_h != ts_data->chip_type->bl_idh) ||
		(id_l != ts_data->chip_type->bl_idl)))
		return -EINVAL;

	return 0;
}

static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 chip_id[2] = { 0 };
	struct i2c_client *client = ts_data->client;

	fts_wait_ready(ts_data);

	ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &chip_id[0]);
	if (ret < 0) {
		FTS_ERROR("Failed to read chip ID(0x%02x),ret=%d",
			  chip_id[0], ret);
		return ret;
	}
	ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID2, &chip_id[1]);
	if (ret < 0 || chip_id[0] == 0 || chip_id[1] == 0) {
		FTS_ERROR("Failed to read chip ID(0x%02x),ret=%d",
			  chip_id[0], ret);
		return ret;
	}
	ret = fts_get_chip_types(ts_data, chip_id[0],
					chip_id[1], VALID);
	if (ret < 0) {
		FTS_ERROR("TP STILL not ready, read:0x%02x%02x",
				chip_id[0], chip_id[1]);
		return -EINVAL;
	}

	ts_data->chipid = (short)(chip_id[0] << 8 | chip_id[1]);
	FTS_INFO("get ic information, chip id = 0x%02x%02x",
		 ts_data->chip_type->chip_idh, ts_data->chip_type->chip_idl);

	return 0;
}

static int fts_power_source_init(struct fts_ts_data *data)
{
	int ret = 0;

	data->vdd = devm_regulator_get(&data->client->dev, "vdd");
	if (IS_ERR_OR_NULL(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		FTS_ERROR("get vdd regulator failed,ret=%d", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd, FTS_VDD_MIN_UV,
					    FTS_VDD_MAX_UV);
		if (ret) {
			FTS_ERROR("vdd regulator set_VDD failed ret=%d", ret);
			goto exit;
		}
	}

	data->vcc_i2c = devm_regulator_get(&data->client->dev, "vcc-i2c");
	if (IS_ERR(data->vcc_i2c)) {
		ret = PTR_ERR(data->vcc_i2c);
		FTS_ERROR("get vcc_i2c regulator failed,ret=%d", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		ret = regulator_set_voltage(data->vcc_i2c, FTS_I2C_VCC_MIN_UV,
					    FTS_I2C_VCC_MAX_UV);
		if (ret) {
			FTS_ERROR("vcc_i2c regulator set_vcc_i2c failed ret=%d", ret);
			goto exit;
		}
	}

exit:
	return ret;
}

static int fts_power_source_release(struct fts_ts_data *data)
{
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FTS_I2C_VCC_MAX_UV);
	devm_regulator_put(data->vdd);

	devm_regulator_put(data->vcc_i2c);

	return 0;
}

static int fts_power_source_ctrl(struct fts_ts_data *data, int enable)
{
	int ret = 0;

	if (enable) {
		if (data->power_disabled) {
			ret = regulator_enable(data->vdd);
			if (ret) {
				FTS_ERROR(
					"enable vdd regulator failed,ret=%d",
					ret);
			}

			ret = regulator_enable(data->vcc_i2c);
			if (ret) {
				FTS_ERROR("enable vcc_i2c regulator failed,ret=%d",
					  ret);
			}
			data->power_disabled = false;
		}
	} else {
		if (!data->power_disabled) {
			ret = regulator_disable(data->vdd);
			if (ret) {
				FTS_ERROR(
					"disable vdd regulator failed,ret=%d",
					ret);
			}

			ret = regulator_disable(data->vcc_i2c);
			if (ret) {
				FTS_ERROR("disable vcc_i2c regulator failed,ret=%d",
					  ret);
			}

			data->power_disabled = true;
		}
	}

	return ret;
}

static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;
	struct i2c_client *client = ts->client;

	ts->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		FTS_ERROR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		FTS_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto err_pinctrl_lookup;
	}

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		FTS_ERROR("Pin state[release] not found");
		ret = PTR_ERR(ts->pins_release);
	}

	return 0;
err_pinctrl_lookup:
	if (ts->pinctrl) {
		devm_pinctrl_put(ts->pinctrl);
	}
err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_release = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0) {
			FTS_ERROR("Set normal pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0) {
			FTS_ERROR("Set suspend pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl) {
		if (IS_ERR_OR_NULL(ts->pins_release)) {
			devm_pinctrl_put(ts->pinctrl);
			ts->pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->pinctrl,
						   ts->pins_release);
			if (ret < 0)
				FTS_ERROR("Set gesture pin state error:%d",
					  ret);
		}
	}

	return ret;
}

static void fts_release_all_finger(struct fts_ts_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	u32 finger_count = 0;

	mutex_lock(&data->report_mutex);

	for (finger_count = 0; finger_count < data->max_touch_number;
	     finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}

	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	mutex_unlock(&data->report_mutex);
}

static int fts_input_report_b(struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;
	bool va_reported = false;
	struct ts_event *events = data->events;

	for (i = 0; i < data->touch_point; i++) {
		if (events[i].id >= data->max_touch_number)
			break;

		va_reported = true;
		input_mt_slot(data->input_dev, events[i].id);

		if (EVENT_DOWN(events[i].flag)) {
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, true);

			if (events[i].p <= 0) {
				events[i].p = 0x3f;
			}
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					 events[i].p);

			if (events[i].area <= 0) {
				events[i].area = 0x09;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 events[i].area);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 events[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 events[i].y);

			touchs |= BIT(events[i].id);
			data->touchs |= BIT(events[i].id);
		} else {
			uppoint++;

			input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);

			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(events[i].id);
		}
	}

	if (data->touchs ^ touchs) {
		for (i = 0; i < data->max_touch_number; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				va_reported = true;
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(
					data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;

	if (va_reported) {
		if (EVENT_NO_DOWN(data) || (!touchs)) {
			input_report_key(data->input_dev, BTN_TOUCH, 0);
		} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	} else {
		FTS_ERROR("va not reported, but touchs=%d", touchs);
	}

	input_sync(data->input_dev);
	return 0;
}

static int fts_read_touchdata(struct fts_ts_data *data)
{
	int ret = 0;
	int i = 0;
	u8 pointid;
	int base;
	struct ts_event *events = data->events;
	int max_touch_num = data->max_touch_number;
	u8 *buf = data->point_buf;

	data->point_num = 0;
	data->touch_point = 0;

	memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x00;

	ret = fts_i2c_read(data->client, buf, 1, buf, data->pnt_buf_size);
	if (ret < 0) {
		FTS_ERROR("read touchdata failed, ret:%d", ret);
		return ret;
	}
	data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;

	if (data->point_num > max_touch_num) {
		return -EINVAL;
	}

	for (i = 0; i < max_touch_num; i++) {
		base = FTS_ONE_TCH_LEN * i;

		pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else if (pointid >= max_touch_num) {
			return -EINVAL;
		}

		data->touch_point++;

		events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
			      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
		events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
			      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
		events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
		events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
		events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
		events[i].p = buf[FTS_TOUCH_PRE_POS + base];

		if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
			FTS_INFO("abnormal touch data from fw");
			return -EIO;
		}
	}
	if (data->touch_point == 0) {
		FTS_INFO("no touch point information");
		return -EIO;
	}

	return 0;
}

static void fts_report_event(struct fts_ts_data *data)
{
	fts_input_report_b(data);
}

static irqreturn_t fts_ts_interrupt(int irq, void *data)
{
	int ret = 0;
	struct fts_ts_data *ts_data = (struct fts_ts_data *)data;

	if (!ts_data) {
		FTS_ERROR("[INTR]: Invalid fts_ts_data");
		return IRQ_HANDLED;
	}

	if (ts_data->dev_pm_suspend) {
		ret = wait_for_completion_timeout(
			&ts_data->dev_pm_suspend_completion,
			msecs_to_jiffies(700));
		if (!ret) {
			FTS_ERROR(
				"system(i2c) can't finished resuming procedure, skip it");
			return IRQ_HANDLED;
		}
	}

	ret = fts_read_touchdata(ts_data);
	if (ret == 0) {
		mutex_lock(&ts_data->report_mutex);
		fts_report_event(ts_data);
		mutex_unlock(&ts_data->report_mutex);
	}

	return IRQ_HANDLED;
}

static int fts_input_init(struct fts_ts_data *data)
{
	int ret = 0;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("Failed to allocate memory for input device");
		return -ENOMEM;
	}

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;

	input_set_drvdata(input_dev, data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	//__set_bit(EV_KEY, input_dev->evbit); ?
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, data->max_touch_number,
			    INPUT_MT_DIRECT);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     data->width - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     data->height - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);

	data->pnt_buf_size = data->max_touch_number * FTS_ONE_TCH_LEN + 3;
	data->point_buf = (u8 *)kzalloc(data->pnt_buf_size, GFP_KERNEL);
	if (!data->point_buf) {
		FTS_ERROR("failed to alloc memory for point buf!");
		ret = -ENOMEM;
		goto err_point_buf;
	}

	data->events = (struct ts_event *)kzalloc(
		data->max_touch_number * sizeof(struct ts_event), GFP_KERNEL);
	if (!data->events) {
		FTS_ERROR("failed to alloc memory for point events!");
		ret = -ENOMEM;
		goto err_event_buf;
	}
	ret = input_register_device(input_dev);
	if (ret) {
		FTS_ERROR("Input device registration failed");
		goto err_input_reg;
	}

	data->input_dev = input_dev;

	return 0;

err_input_reg:
	kfree_safe(data->events);

err_event_buf:
	kfree_safe(data->point_buf);

err_point_buf:
	input_set_drvdata(input_dev, NULL);
	input_free_device(input_dev);
	input_dev = NULL;

	return ret;
}

static int fts_gpio_configure(struct fts_ts_data *data)
{
	int ret = 0;

	/* request irq gpio */
	if (gpio_is_valid(data->irq_gpio)) {
		ret = gpio_request(data->irq_gpio, "fts_irq_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->irq_gpio);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->reset_gpio)) {
		ret = gpio_request(data->reset_gpio, "fts_reset_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->reset_gpio, 0);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}

		msleep(20);

		ret = gpio_direction_output(data->reset_gpio, 1);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}

	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->reset_gpio))
		gpio_free(data->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->irq_gpio))
		gpio_free(data->irq_gpio);
err_irq_gpio_req:
	return ret;
}

static int fts_parse_dt(struct fts_ts_data *data)
{
	int ret = 0;
	struct device *dev = &data->client->dev;
	struct device_node *np = dev->of_node;
	u32 size[2], val;

	ret = of_property_read_u32_array(np, "focaltech,display-size", size, 2);
	if (ret && (ret != -EINVAL)) {
		FTS_ERROR("Unable to read property 'focaltech,display-size'");
		return -ENODATA;
	}
	data->width = size[0];
	data->height = size[1];

	FTS_DEBUG("display-size:%dx%d", data->width, data->height);

	ret = of_property_read_u32(np, "focaltech,max-touch-number", &val);
	if (ret < 0) {
		FTS_ERROR("Unable to read property 'focaltech,max-touch-number'");
		return -ENODATA;
	}
	if (val < 2)
		data->max_touch_number = 2;
	else if (val > FTS_MAX_POINTS_SUPPORT)
		data->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	else
		data->max_touch_number = val;

	/* reset, irq gpio info */
	data->reset_gpio = of_get_named_gpio_flags(
		np, "focaltech,reset-gpio", 0, &data->reset_gpio_flags);
	if (data->reset_gpio < 0)
		FTS_ERROR("Unable to get reset_gpio");

	data->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio", 0,
						  &data->irq_gpio_flags);
	if (data->irq_gpio < 0)
		FTS_ERROR("Unable to get irq_gpio");

	return 0;
}

static int fts_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct fts_ts_data *data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_ERROR("I2C not supported");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		FTS_ERROR(
			"Failed to allocate memory for driver data");
		return -ENOMEM;
	}

	data->client = client;
	ret = fts_parse_dt(data);
	if (ret)
		FTS_ERROR("[DTS]DT parsing failed");

	data->chip_type = (struct fts_chip_type*) of_device_get_match_data(&client->dev);

	i2c_set_clientdata(client, data);

	data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (NULL == data->ts_workqueue) {
		FTS_ERROR("failed to create fts workqueue");
	}

	spin_lock_init(&data->irq_lock);
	mutex_init(&data->report_mutex);

	ret = fts_input_init(data);
	if (ret) {
		FTS_ERROR("fts input initialize fail");
		goto err_input_init;
	}

	ret = fts_power_source_init(data);
	if (ret) {
		FTS_ERROR("fail to get vdd/vcc_i2c regulator");
		goto err_power_init;
	}

	data->power_disabled = true;
	ret = fts_power_source_ctrl(data, ENABLE);
	if (ret) {
		FTS_ERROR("fail to enable vdd/vcc_i2c regulator");
		goto err_power_ctrl;
	}
	ret = fts_pinctrl_init(data);
	if (0 == ret) {
		fts_pinctrl_select_normal(data);
	}

	ret = fts_gpio_configure(data);
	if (ret) {
		FTS_ERROR("[GPIO]Failed to configure the gpios");
		goto err_gpio_config;
	}

	ret = fts_get_ic_information(data);
	if (ret) {
		FTS_ERROR("can't get ic information");
		goto err_irq_req;
	}

	data->event_wq =
		alloc_workqueue("fts-event-queue",
				WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!data->event_wq) {
		FTS_ERROR("ERROR: Cannot create work thread\n");
		goto err_irq_req;
	}

	data->irq = gpio_to_irq(data->irq_gpio);
	if (data->irq != data->client->irq)
		FTS_ERROR(
			"IRQs are inconsistent, please check <interrupts> & <focaltech,irq-gpio> in DTS");

	if (0 == data->irq_gpio_flags)
		data->irq_gpio_flags = IRQF_TRIGGER_FALLING;
	ret = request_threaded_irq(data->irq, NULL, fts_ts_interrupt,
				   data->irq_gpio_flags | IRQF_ONESHOT,
				   data->client->name, data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_event_wq;
	}

	device_init_wakeup(&client->dev, 1);
	data->dev_pm_suspend = false;
	init_completion(&data->dev_pm_suspend_completion);

	return 0;

err_event_wq:
	if (data->event_wq)
		destroy_workqueue(data->event_wq);
err_irq_req:
	if (gpio_is_valid(data->reset_gpio))
		gpio_free(data->reset_gpio);
	if (gpio_is_valid(data->irq_gpio))
		gpio_free(data->irq_gpio);
err_gpio_config:

	fts_pinctrl_select_release(data);

	fts_power_source_ctrl(data, DISABLE);
err_power_ctrl:
	fts_power_source_release(data);
err_power_init:
	kfree_safe(data->point_buf);
	kfree_safe(data->events);
	input_unregister_device(data->input_dev);
err_input_init:
	if (data->ts_workqueue)
		destroy_workqueue(data->ts_workqueue);
	devm_kfree(&client->dev, data);

	return ret;
}

static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *ts_data = i2c_get_clientdata(client);


	destroy_workqueue(ts_data->event_wq);

	free_irq(client->irq, ts_data);
	input_unregister_device(ts_data->input_dev);

	if (gpio_is_valid(ts_data->reset_gpio))
		gpio_free(ts_data->reset_gpio);

	if (gpio_is_valid(ts_data->irq_gpio))
		gpio_free(ts_data->irq_gpio);

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

	fts_pinctrl_select_release(ts_data);
	fts_power_source_ctrl(ts_data, DISABLE);
	fts_power_source_release(ts_data);

	kfree_safe(ts_data->point_buf);
	kfree_safe(ts_data->events);

	devm_kfree(&client->dev, ts_data);

	return 0;
}


static int fts_ts_suspend(struct device *dev)
{
	int ret = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	disable_irq(ts_data->irq);

	ret = fts_power_source_ctrl(ts_data, DISABLE);
	if (ret < 0) {
		FTS_ERROR("power off fail, ret=%d", ret);
	}
	fts_pinctrl_select_suspend(ts_data);

	return 0;
}

static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	unsigned long irqflags = 0;

	fts_release_all_finger(ts_data);
	fts_power_source_ctrl(ts_data, ENABLE);
	fts_pinctrl_select_normal(ts_data);
	fts_wait_ready(ts_data);

	spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	enable_irq(ts_data->irq);
	spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);

	return 0;
}

static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	ts_data->dev_pm_suspend = true;

	if (ts_data->lpwg_mode) {
		ret = enable_irq_wake(ts_data->irq);
		if (ret) {
			FTS_ERROR("enable_irq_wake(irq:%d) failed",
				 ts_data->irq);
		}
	}

	reinit_completion(&ts_data->dev_pm_suspend_completion);

	return ret;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	ts_data->dev_pm_suspend = false;

	if (ts_data->lpwg_mode) {
		ret = disable_irq_wake(ts_data->irq);
		if (ret) {
			FTS_ERROR("disable_irq_wake(irq:%d) failed",
				 ts_data->irq);
		}
	}

	complete(&ts_data->dev_pm_suspend_completion);

	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};

static const struct fts_chip_type chip_type_fts8719 = {
	.type = 0x0D,
	.chip_idh = 0x87,
	.chip_idl = 0x19,
	.rom_idh = 0x87,
	.rom_idl = 0x19,
	.pb_idh = 0x87,
	.pb_idl = 0xA9,
	.bl_idh = 0x87,
	.bl_idl = 0xB9,
};

static const struct fts_chip_type chip_type_fts3518 = {
	.type = 0x81,
	.chip_idh = 0x54,
	.chip_idl = 0x52,
	.rom_idh = 0x54,
	.rom_idl = 0x52,
	.pb_idh = 0x00,
	.pb_idl = 0x00,
	.bl_idh = 0x54,
	.bl_idl = 0x5C,
};

static struct of_device_id fts_match_table[] = {
	{
		.compatible = "focaltech,fts8719",
		.data = &chip_type_fts8719,
	},
	{
		.compatible = "focaltech,fts3518",
		.data = &chip_type_fts3518,
	},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, fts_match_table);

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.pm = &fts_dev_pm_ops,
		.of_match_table = fts_match_table,
	},
};
module_i2c_driver(fts_ts_driver);

MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
