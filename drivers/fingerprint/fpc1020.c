// SPDX-License-Identifier: GPL-2.0
/*
 * Fingerprint Cards FPC1020 Fingerprint Sensor Driver
 *
 * Copyright (c) 2021, Yassine Oudjana <y.oudjana@protonmail.com>
 */

#define DEBUG

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fingerprint.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#define FPC102X_TIRQVALID			1300
#define FPC102X_SPI_CLK_FREQ			8000000U
#define FPC102X_CAPTURE_WAIT_FINGER_DELAY_MS 	20

#define FPC102X_ROWS				56
#define FPC102X_COLUMNS				192
#define FPC102X_IMG_DATA_LEN			10752

#define FPC102X_CMD_FINGER_PRESENT_QUERY	0x20
#define FPC102X_CMD_MODE_WAIT_FOR_FINGER	0x24
#define FPC102X_CMD_MODE_SLEEP			0x28
#define FPC102X_CMD_MODE_DEEP_SLEEP		0x2c
#define FPC102X_CMD_CAPTURE_IMG			0xc0
#define FPC102X_CMD_READ_IMG_DATA		0xc4

#define FPC102X_REG_INTR			0x18
#define FPC102X_REG_INTR_CLEAR			0x1c
#define FPC102X_REG_FNGRDRIVECONF		0x8c
#define FPC102X_REG_ADCSHIFTGAIN		0xa0
#define FPC102X_REG_FNGRPRESENTSTATUS		0xd4
#define FPC102X_REG_FNGRDETTHRESH		0xd8
#define FPC102X_REG_HWID			0xfc

#define FPC102X_REG_INTR_SIZE			1
#define FPC102X_REG_HWID_SIZE			2

#define FPC102X_INTR_BIT_FINGER_DOWN		BIT(0)
#define FPC102X_INTR_BIT_ERROR			BIT(2)
#define FPC102X_INTR_BIT_IMG_DATA_AVAIL		BIT(5)
#define FPC102X_INTR_BIT_CMD_DONE		BIT(7)

static const char * const fpc1020_supply_name[] = {
	"vcc",
	"vdda",
	"vddio",
};

#define FPC1020_NUM_SUPPLIES ARRAY_SIZE(fpc1020_supply_name)

struct fpc1020_chip {
	struct device *dev;
	struct spi_device *spi;
	struct fingerprint_device *fpdev;

	struct regulator_bulk_data supplies[FPC1020_NUM_SUPPLIES];
	struct gpio_desc *reset;
	int irq;

	struct completion ready;
	struct completion cmd_done;
	struct completion img_avail;
	struct work_struct finger_query_work;
	bool finger_down;
	u8 last_cmd;
};

static int fpc1020_send_cmd(struct fpc1020_chip *chip, u8 cmd)
{
	int ret;

	ret = spi_write(chip->spi, &cmd, 1);
	if (ret)
		return ret;

	chip->last_cmd = cmd;

	return 0;
}

static int fpc1020_access_reg(struct fpc1020_chip *chip, u8 addr, size_t len,
				void *val, bool write)
{
	struct spi_message msg;
	struct spi_transfer *reg;
	struct spi_transfer *data;

	reg = devm_kzalloc(chip->dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	data = devm_kzalloc(chip->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	reg->speed_hz = FPC102X_SPI_CLK_FREQ;
	reg->len = 1;
	reg->tx_buf = &addr;

	data->speed_hz = FPC102X_SPI_CLK_FREQ;
	data->len = len;
	data->bits_per_word = 8 * (!(len % 2) && len <= 4 ? len : 1);
	if (write)
		data->tx_buf = val;
	else
		data->rx_buf = val;

	spi_message_init(&msg);
	spi_message_add_tail(reg, &msg);
	spi_message_add_tail(data, &msg);

	return spi_sync(chip->spi, &msg);
}

static void fpc1020_finger_query_worker(struct work_struct *work)
{
	struct fpc1020_chip *chip = container_of(work, struct fpc1020_chip,
							finger_query_work);
	int ret;

	msleep(FPC102X_CAPTURE_WAIT_FINGER_DELAY_MS);
	ret = fpc1020_send_cmd(chip, FPC102X_CMD_FINGER_PRESENT_QUERY);

	if (ret)
		dev_err(chip->dev, "Failed to query finger presence: %d", ret);
}

static irqreturn_t fpc1020_irq_handler(int irq, void *ptr)
{
	struct fpc1020_chip *chip = ptr;
	u8 intr_type;
	u16 finger_present_mask;
	int ret;

	ret = fpc1020_access_reg(chip, FPC102X_REG_INTR_CLEAR,
				FPC102X_REG_INTR_SIZE, &intr_type, false);
	if (ret) {
		dev_err(chip->dev, "Failed to read interrupt type: %d\n", ret);
		return IRQ_NONE;
	}

	dev_vdbg(chip->dev, "Interrupt type 0x%02x", intr_type);

	if(intr_type == 0xff) {
		/* Chip came out of reset */
		complete_all(&chip->ready);
		return IRQ_HANDLED;
	}

	if (intr_type & FPC102X_INTR_BIT_ERROR)
		dev_err(chip->dev, "Received error interrupt\n");
		/* TODO: Handle error */

	if (intr_type & FPC102X_INTR_BIT_FINGER_DOWN && !chip->finger_down) {
		dev_dbg(chip->dev, "Finger down\n");

		chip->finger_down = true;
		fingerprint_report_finger(chip->fpdev, true);

		/* Start checking for finger removal */
		schedule_work(&chip->finger_query_work);
	}

	if (intr_type & FPC102X_INTR_BIT_IMG_DATA_AVAIL) {
		dev_dbg(chip->dev, "Image data available\n");
		complete_all(&chip->img_avail);
	}

	if (intr_type & FPC102X_INTR_BIT_CMD_DONE) {
		dev_vdbg(chip->dev, "Command 0x%x done\n", chip->last_cmd);

		if (chip->last_cmd == FPC102X_CMD_FINGER_PRESENT_QUERY) {
			ret = fpc1020_access_reg(chip,
						 FPC102X_REG_FNGRPRESENTSTATUS,
						 2, &finger_present_mask, false);
			dev_vdbg(chip->dev, "Finger present status 0x%04x\n",
				finger_present_mask);

			if (!finger_present_mask) {
				dev_dbg(chip->dev, "Finger up\n");

				chip->finger_down = false;
				fingerprint_report_finger(chip->fpdev, false);

				/* Go back to wait for finger mode */
				ret = fpc1020_send_cmd(chip,
					FPC102X_CMD_MODE_WAIT_FOR_FINGER);
				if (ret)
					dev_err(chip->dev,
					"Failed to set wait for finger mode: %d\n",
					ret);
			} else
				/* Start checking for finger removal again */
				schedule_work(&chip->finger_query_work);
		}

		complete_all(&chip->cmd_done);
	}

	return IRQ_HANDLED;
}

static int fpc1020_set_adc_shift(struct fpc1020_chip *chip, u8 shift)
{
	u16 temp;
	int ret;

	if (shift > 0x1f)
		return -EINVAL;

	ret = fpc1020_access_reg(chip, FPC102X_REG_ADCSHIFTGAIN, 2, &temp, false);
	if (ret)
		return ret;

	temp = (temp & GENMASK(7, 0)) | shift << 8;
	ret = fpc1020_access_reg(chip, FPC102X_REG_ADCSHIFTGAIN, 2, &temp, true);

	return ret;
}

static int fpc1020_set_adc_gain(struct fpc1020_chip *chip, u8 gain)
{
	u16 temp;
	int ret;

	if (gain > 0xf)
		return -EINVAL;

	ret = fpc1020_access_reg(chip, FPC102X_REG_ADCSHIFTGAIN, 2, &temp, false);
	if (ret)
		return ret;

	temp = (temp & GENMASK(15, 7)) | gain;
	ret = fpc1020_access_reg(chip, FPC102X_REG_ADCSHIFTGAIN, 2, &temp, true);

	return ret;
}

static int fpc1020_set_det_threshold(struct fpc1020_chip *chip, u8 threshold)
{
	return fpc1020_access_reg(chip, FPC102X_REG_FNGRDETTHRESH, 1,
				  &threshold, true);
}

static ssize_t fpc1020_capture(struct fingerprint_device *fpdev,
				u8 *buf)
{
	struct fpc1020_chip *chip = fpdev->drvdata;
	struct spi_message msg;
	struct spi_transfer *reg;
	struct spi_transfer *dummy;
	struct spi_transfer *data;
	u8 cmd, temp;
	int ret;

	reg = devm_kzalloc(chip->dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	dummy = devm_kzalloc(chip->dev, sizeof(*dummy), GFP_KERNEL);
	if (!dummy)
		return -ENOMEM;

	data = devm_kzalloc(chip->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Capture image */
	ret = fpc1020_send_cmd(chip, FPC102X_CMD_CAPTURE_IMG);
	if (ret) {
		dev_err(chip->dev, "Failed to send capture command: %d\n", ret);
		return ret;
	}
	ret = wait_for_completion_killable(&chip->img_avail);
	if (ret)
		goto out;
	reinit_completion(&chip->img_avail);

	/* Read captured image data */
	spi_message_init(&msg);

	cmd = FPC102X_CMD_READ_IMG_DATA;
	reg->speed_hz = FPC102X_SPI_CLK_FREQ;
	reg->len = 1;
	reg->tx_buf = &cmd;
	spi_message_add_tail(reg, &msg);

	dummy->speed_hz = FPC102X_SPI_CLK_FREQ;
	dummy->len = 1;
	dummy->rx_buf = &temp;
	spi_message_add_tail(dummy, &msg);

	data->speed_hz = FPC102X_SPI_CLK_FREQ;
	data->len = FPC102X_IMG_DATA_LEN;
	data->rx_buf = buf;
	spi_message_add_tail(data, &msg);

	ret = spi_sync(chip->spi, &msg);
	if (ret)
		dev_err(chip->dev, "Failed to read image data: %d\n", ret);

out:
	/* Start detecting finger presence again */
	schedule_work(&chip->finger_query_work);

	return ret ? ret : FPC102X_IMG_DATA_LEN;
}

static struct fingerprint_ops fpc1020_ops = {
	.capture = fpc1020_capture,
};

static void fpc1020_disable_regulators(void *arg)
{
	struct fpc1020_chip *chip = arg;

	regulator_bulk_disable(FPC1020_NUM_SUPPLIES, chip->supplies);
}

static int fpc1020_probe(struct spi_device *spi)
{
	struct fingerprint_device *fpdev;
        struct fpc1020_chip *chip;
	u16 hw_id;
	u32 temp;
	int i, irq, ret = 0;

	fpdev = devm_kzalloc(&spi->dev, sizeof(*fpdev), GFP_KERNEL);
        if (!fpdev)
                return -ENOMEM;

        chip = devm_kzalloc(&spi->dev, sizeof(*chip), GFP_KERNEL);
        if (!chip)
                return -ENOMEM;

	chip->dev = &spi->dev;
	chip->spi = spi;
	spi_set_drvdata(spi, chip);

	chip->reset = devm_gpiod_get(chip->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(chip->reset)) {
		ret = PTR_ERR(chip->reset);
		dev_err(chip->dev, "Failed to get reset pin: %d\n", ret);
		return ret;
	}

	for (i = 0; i < FPC1020_NUM_SUPPLIES; i++)
		chip->supplies[i].supply = fpc1020_supply_name[i];

	ret = devm_regulator_bulk_get(chip->dev, FPC1020_NUM_SUPPLIES,
					chip->supplies);
	if (ret) {
		dev_err(chip->dev, "Failed to get regulators: %d\n", ret);
		return ret;
	}

	INIT_WORK(&chip->finger_query_work, fpc1020_finger_query_worker);
	init_completion(&chip->ready);
	init_completion(&chip->cmd_done);
	init_completion(&chip->img_avail);

	irq = of_irq_get(chip->dev->of_node, 0);
	if (irq < 0) {
		dev_err(chip->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}
	ret = devm_request_threaded_irq(chip->dev, irq,
					NULL, fpc1020_irq_handler,
					IRQF_ONESHOT,
					"fpc1020", chip);
	if (ret) {
		dev_err(chip->dev,
			"Failed to request IRQ: %d", ret);
		return ret;
	}

	/* Power-on chip */
	ret = regulator_bulk_enable(FPC1020_NUM_SUPPLIES, chip->supplies);
	if (ret) {
		dev_err(chip->dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(chip->dev,
				       fpc1020_disable_regulators,
				       chip);
	if (ret)
		return ret;

	/* Reset chip */
	gpiod_set_value_cansleep(chip->reset, 1);
	usleep_range(1000, 1010);
	gpiod_set_value_cansleep(chip->reset, 0);

	/* Wait for chip to become ready */
	ret = wait_for_completion_timeout(&chip->ready,
				msecs_to_jiffies(FPC102X_TIRQVALID));
	if (ret <= 0) {
		ret = -ETIMEDOUT;
		dev_err(chip->dev, "Failed to power-on chip: %d\n", ret);
		return ret;
	}

	/* Read hardware ID */
	ret = fpc1020_access_reg(chip, FPC102X_REG_HWID, FPC102X_REG_HWID_SIZE,
				&hw_id, false);
	if (ret) {
		dev_err(chip->dev, "Failed to read HW ID: %d\n", ret);
		return ret;
	}
	dev_info(chip->dev, "Chip type: FPC1%x%X\n", hw_id >> 4, hw_id & 0xf);

	/* Set ADC shift */
	ret = device_property_read_u32_array(chip->dev, "fpc,adc-shift", &temp, 1);
	/* This property is optional, continue if not found */
	if (ret < 0 && ret != -ENODATA && ret != -ENXIO) {
		dev_err(chip->dev, "Failed to get ADC shift: %d\n", ret);
		return ret;
	}
	else if (!ret) {
		ret = fpc1020_set_adc_shift(chip, (u8) temp);
		if (ret) {
			dev_err(chip->dev, "Failed to set ADC shift: %d\n", ret);
			return ret;
		}
		dev_dbg(chip->dev, "ADC shift set to %d\n", temp);
	}

	/* Set ADC gain */
	ret = device_property_read_u32_array(chip->dev, "fpc,adc-gain", &temp, 1);
	/* This property is optional, continue if not found */
	if (ret < 0 && ret != -ENODATA && ret != -ENXIO) {
		dev_err(chip->dev, "Failed to get ADC gain: %d\n", ret);
		return ret;
	}
	else if (!ret) {
		ret = fpc1020_set_adc_gain(chip, (u8) temp);
		if (ret) {
			dev_err(chip->dev, "Failed to set ADC gain: %d\n", ret);
			return ret;
		}
		dev_dbg(chip->dev, "ADC gain set to %d\n", temp);
	}

	/* Set detection threshold */
	ret = device_property_read_u32_array(chip->dev, "fpc,det-threshold",
					     &temp, 1);
	/* This property is optional, continue if not found */
	if (ret < 0 && ret != -ENODATA && ret != -ENXIO) {
		dev_err(chip->dev,
			"Failed to get detection threshold: %d\n", ret);
		return ret;
	}
	else if (!ret) {
		ret = fpc1020_set_det_threshold(chip, (u8) temp);
		if (ret) {
			dev_err(chip->dev,
				"Failed to set detection threshold: %d\n", ret);
			return ret;
		}
		dev_dbg(chip->dev, "detection threshold set to %d\n", temp);
	}

	fpdev->name = "fpc1020";
	fpdev->width = FPC102X_COLUMNS;
	fpdev->height = FPC102X_ROWS;
	fpdev->bytes_per_pixel = 1;
	fpdev->ops = &fpc1020_ops;

	ret = fingerprint_register_device(chip->dev, fpdev, chip);
	if (ret) {
		dev_err(chip->dev,
			"Failed to register fingerprint device: %d\n", ret);

		return ret;
	}
	chip->fpdev = fpdev;

	/* Start detecting finger presence */
	ret = fpc1020_send_cmd(chip, FPC102X_CMD_MODE_WAIT_FOR_FINGER);
	if (ret) {
		dev_err(chip->dev,
			"Failed to start waiting for finger: %d\n", ret);
		goto err;
	}

        return 0;
err:
	fingerprint_unregister_device(chip->fpdev);
	return ret;
}

static void fpc1020_remove(struct spi_device *spi)
{
	struct fpc1020_chip *chip = spi_get_drvdata(spi);

	fingerprint_unregister_device(chip->fpdev);
}

static int __maybe_unused fpc1020_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fpc1020_chip *chip = spi_get_drvdata(spi);
	int ret;

	/* Enter deep sleep mode. This will disable finger detection */
	ret = fpc1020_send_cmd(chip, FPC102X_CMD_MODE_DEEP_SLEEP);
	if (ret) {
		dev_err(chip->dev, "Failed to enter deep sleep: %d\n", ret);
		return ret;
	}

	disable_irq(chip->irq);

	return 0;
}

static int __maybe_unused fpc1020_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fpc1020_chip *chip = spi_get_drvdata(spi);
	int ret;

	enable_irq(chip->irq);

	/* Start detecting finger presence */
	ret = fpc1020_send_cmd(chip, FPC102X_CMD_MODE_WAIT_FOR_FINGER);
	if (ret) {
		dev_err(chip->dev,
			"Failed to start waiting for finger: %d\n", ret);
		return ret;
	}

	return 0;
}

static int __maybe_unused fpc1020_idle(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fpc1020_chip *chip = spi_get_drvdata(spi);
	int ret;

	/* Enter sleep mode. This will reduce finger query frequency */
	ret = fpc1020_send_cmd(chip, FPC102X_CMD_MODE_SLEEP);
	if (ret) {
		dev_err(chip->dev, "Failed to enter sleep: %d\n", ret);
		return ret;
	}

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(fpc1020_pm_ops, fpc1020_suspend,
				fpc1020_resume, fpc1020_idle);

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020" },
	{},
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct spi_driver fpc1020_driver = {
	.driver = {
		.name = "fpc1020",
		.pm = &fpc1020_pm_ops,
		.of_match_table = fpc1020_of_match
        },
	.probe = fpc1020_probe,
	.remove = fpc1020_remove
};
module_spi_driver(fpc1020_driver);

MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com");
MODULE_DESCRIPTION("Fingerprint Cards FPC1020 Fingerprint Sensor Driver");
MODULE_LICENSE("GPL v2");
