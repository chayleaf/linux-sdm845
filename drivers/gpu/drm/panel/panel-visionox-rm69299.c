// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct visionox_rm69299 {
	struct drm_panel panel;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	struct mipi_dsi_device *dsi;
	const struct drm_display_mode *mode;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode visionox_rm69299_1080x2248_60hz = {
	.name = "1080x2248",
	.clock = 158695,
	.hdisplay = 1080,
	.hsync_start = 1080 + 26,
	.hsync_end = 1080 + 26 + 2,
	.htotal = 1080 + 26 + 2 + 36,
	.vdisplay = 2248,
	.vsync_start = 2248 + 56,
	.vsync_end = 2248 + 56 + 4,
	.vtotal = 2248 + 56 + 4 + 4,
	.flags = 0,
};

static const struct drm_display_mode visionox_rm69299_1080x2160_60hz = {
	.name = "Visionox 1080x2160@60Hz",
	.clock = 158695,
	.hdisplay = 1080,
	.hsync_start = 1080 + 26,
	.hsync_end = 1080 + 26 + 2,
	.htotal = 1080 + 26 + 2 + 36,
	.vdisplay = 2160,
	.vsync_start = 2160 + 8,
	.vsync_end = 2160 + 8 + 4,
	.vtotal = 2160 + 8 + 4 + 4,
	.flags = 0,
	.width_mm = 74,
	.height_mm = 131,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static inline struct visionox_rm69299 *panel_to_ctx(struct drm_panel *panel)
{
	return container_of(panel, struct visionox_rm69299, panel);
}

static int visionox_rm69299_power_on(struct visionox_rm69299 *ctx)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	/*
	 * Reset sequence of visionox panel requires the panel to be
	 * out of reset for 10ms, followed by being held in reset
	 * for 10ms and then out again
	 */
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 20000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10000, 20000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 20000);

	return 0;
}

static int visionox_rm69299_power_off(struct visionox_rm69299 *ctx)
{
	gpiod_set_value(ctx->reset_gpio, 0);

	return regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int visionox_rm69299_unprepare(struct drm_panel *panel)
{
	struct visionox_rm69299 *ctx = panel_to_ctx(panel);
	int ret;

	ctx->dsi->mode_flags = 0;

	ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	if (ret < 0)
		dev_err(ctx->panel.dev, "set_display_off cmd failed ret = %d\n",
			ret);

	/* 120ms delay required here as per DCS spec */
	msleep(120);

	ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_ENTER_SLEEP_MODE, NULL, 0);
	if (ret < 0) {
		dev_err(ctx->panel.dev, "enter_sleep cmd failed ret = %d\n",
			ret);
	}

	ret = visionox_rm69299_power_off(ctx);

	ctx->prepared = false;
	return ret;
}

#define VISIONOX_RM69299_SHIFT_INIT_SEQ_LEN 432

static const u8 visionox_rm69299_1080x2248_60hz_init_seq[VISIONOX_RM69299_SHIFT_INIT_SEQ_LEN][2] = {
#include "shift6mq_out.h"
};

static int visionox_rm69299_prepare(struct drm_panel *panel)
{
	struct visionox_rm69299 *ctx = panel_to_ctx(panel);
	int ret, i;

	if (ctx->prepared)
		return 0;

	ret = visionox_rm69299_power_on(ctx);
	if (ret < 0)
		return ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (ctx->mode == &visionox_rm69299_1080x2160_60hz) {
		for (i = 0; i < VISIONOX_RM69299_SHIFT_INIT_SEQ_LEN; i++) {
			ret = mipi_dsi_dcs_write_buffer(ctx->dsi,
				visionox_rm69299_1080x2248_60hz_init_seq[i], 2);
			if (ret < 0) {
				dev_err(ctx->panel.dev,
					"cmd set tx 0 failed, ret = %d\n", ret);
				return ret;
			}
		}
	} else {
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi, (u8[]){ 0xfe, 0x00 },
						2);
		if (ret < 0) {
			dev_err(ctx->panel.dev,
				"cmd set tx 0 failed, ret = %d\n", ret);
			return ret;
		}

		ret = mipi_dsi_dcs_write_buffer(ctx->dsi, (u8[]){ 0xc2, 0x08 },
						2);
		if (ret < 0) {
			dev_err(ctx->panel.dev,
				"cmd set tx 1 failed, ret = %d\n", ret);
			return ret;
		}

		ret = mipi_dsi_dcs_write_buffer(ctx->dsi, (u8[]){ 0x35, 0x00 },
						2);
		if (ret < 0) {
			dev_err(ctx->panel.dev,
				"cmd set tx 2 failed, ret = %d\n", ret);
			return ret;
		}

		ret = mipi_dsi_dcs_write_buffer(ctx->dsi, (u8[]){ 0x51, 0xff },
						2);
		if (ret < 0) {
			dev_err(ctx->panel.dev,
				"cmd set tx 3 failed, ret = %d\n", ret);
			return ret;
		}
	}

	ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret < 0) {
		dev_err(ctx->panel.dev, "exit_sleep_mode cmd failed ret = %d\n",
			ret);
		return ret;
	}

	/* Per DSI spec wait 120ms after sending exit sleep DCS command */
	msleep(120);

	ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	if (ret < 0) {
		dev_err(ctx->panel.dev, "set_display_on cmd failed ret = %d\n",
			ret);
		return ret;
	}

	/* Per DSI spec wait 120ms after sending set_display_on DCS command */
	msleep(120);

	ctx->prepared = true;

	return 0;
}

static int visionox_rm69299_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	struct visionox_rm69299 *ctx = panel_to_ctx(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ctx->mode);
	if (!mode)
		return -ENOMEM;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs visionox_rm69299_drm_funcs = {
	.unprepare = visionox_rm69299_unprepare,
	.prepare = visionox_rm69299_prepare,
	.get_modes = visionox_rm69299_get_modes,
};

static int visionox_rm69299_panel_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret;

	ret = mipi_dsi_dcs_set_display_brightness(dsi,
						backlight_get_brightness(bl));
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops visionox_rm69299_panel_bl_ops = {
	.update_status = visionox_rm69299_panel_bl_update_status,
};

static struct backlight_device *
visionox_rm69299_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_PLATFORM,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					&visionox_rm69299_panel_bl_ops, &props);
}

static int visionox_rm69299_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct visionox_rm69299 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->mode = of_device_get_match_data(dev);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->panel.dev = dev;
	ctx->dsi = dsi;

	ctx->supplies[0].supply = "vdda";
	ctx->supplies[1].supply = "vdd3p3";

	ret = devm_regulator_bulk_get(ctx->panel.dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->panel.dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset gpio %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	drm_panel_init(&ctx->panel, dev, &visionox_rm69299_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &visionox_rm69299_drm_funcs;

	if (ctx->mode == &visionox_rm69299_1080x2160_60hz) {
		ctx->panel.backlight = visionox_rm69299_create_backlight(dsi);
		if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");
	}

	drm_panel_add(&ctx->panel);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = /*MIPI_DSI_MODE_VIDEO |*/ MIPI_DSI_MODE_LPM |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;
	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "dsi attach failed ret = %d\n", ret);
		goto err_dsi_attach;
	}

	ret = regulator_set_load(ctx->supplies[0].consumer, 32000);
	if (ret) {
		dev_err(dev,
			"regulator set load failed for vdda supply ret = %d\n",
			ret);
		goto err_set_load;
	}

	ret = regulator_set_load(ctx->supplies[1].consumer, 13200);
	if (ret) {
		dev_err(dev,
			"regulator set load failed for vdd3p3 supply ret = %d\n",
			ret);
		goto err_set_load;
	}

	return 0;

err_set_load:
	mipi_dsi_detach(dsi);
err_dsi_attach:
	drm_panel_remove(&ctx->panel);
	return ret;
}

static int visionox_rm69299_remove(struct mipi_dsi_device *dsi)
{
	struct visionox_rm69299 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(ctx->dsi);
	mipi_dsi_device_unregister(ctx->dsi);

	drm_panel_remove(&ctx->panel);
	return 0;
}

static const struct of_device_id visionox_rm69299_of_match[] = {
	{ .compatible = "visionox,rm69299-1080p-display",
	  .data = &visionox_rm69299_1080x2248_60hz },
	{ .compatible = "visionox,rm69299-shift",
	  .data = &visionox_rm69299_1080x2160_60hz },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, visionox_rm69299_of_match);

static struct mipi_dsi_driver visionox_rm69299_driver = {
	.driver = {
		.name = "panel-visionox-rm69299",
		.of_match_table = visionox_rm69299_of_match,
	},
	.probe = visionox_rm69299_probe,
	.remove = visionox_rm69299_remove,
};
module_mipi_dsi_driver(visionox_rm69299_driver);

MODULE_DESCRIPTION("Visionox RM69299 DSI Panel Driver");
MODULE_LICENSE("GPL v2");
