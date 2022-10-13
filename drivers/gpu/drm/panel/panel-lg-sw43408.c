// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2022 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct sw43408_dsc_fhd {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct sw43408_dsc_fhd *to_sw43408_dsc_fhd(struct drm_panel *panel)
{
	return container_of(panel, struct sw43408_dsc_fhd, panel);
}

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void sw43408_dsc_fhd_reset(struct sw43408_dsc_fhd *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(9000, 10000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int sw43408_dsc_fhd_on(struct sw43408_dsc_fhd *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_GAMMA_CURVE, 0x02);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0x53, 0x0c, 0x30);
	dsi_dcs_write_seq(dsi, 0x55, 0x00, 0x70, 0xdf, 0x00, 0x70, 0xdf);
	dsi_dcs_write_seq(dsi, 0xf7, 0x01, 0x49, 0x0c);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(50);

	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0) {
		dev_err(dev, "Failed to dcs compression mode: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0xb0, 0xac);
	dsi_dcs_write_seq(dsi, 0xe5, 0x00, 0x3a, 0x00, 0x3a, 0x00, 0x0e, 0x10);
	dsi_dcs_write_seq(dsi, 0xb5,
			  0x75, 0x60, 0x2d, 0x5d, 0x80, 0x00, 0x0a, 0x0b, 0x00,
			  0x05, 0x0b, 0x00, 0x80, 0x0d, 0x0e, 0x40, 0x00, 0x0c,
			  0x00, 0x16, 0x00, 0xb8, 0x00, 0x80, 0x0d, 0x0e, 0x40,
			  0x00, 0x0c, 0x00, 0x16, 0x00, 0xb8, 0x00, 0x81, 0x00,
			  0x03, 0x03, 0x03, 0x01, 0x01);
	msleep(85);
	dsi_dcs_write_seq(dsi, 0xcd,
			  0x00, 0x00, 0x00, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
			  0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x16, 0x16);
	dsi_dcs_write_seq(dsi, 0xcb, 0x80, 0x5c, 0x07, 0x03, 0x28);
	dsi_dcs_write_seq(dsi, 0xc0, 0x02, 0x02, 0x0f);
	dsi_dcs_write_seq(dsi, 0x55, 0x04, 0x61, 0xdb, 0x04, 0x70, 0xdb);
	dsi_dcs_write_seq(dsi, 0xb0, 0xca);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	msleep(50);

	return 0;
}

static int sw43408_dsc_fhd_off(struct sw43408_dsc_fhd *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(100);

	return 0;
}

static int sw43408_dsc_fhd_prepare(struct drm_panel *panel)
{
	struct sw43408_dsc_fhd *ctx = to_sw43408_dsc_fhd(panel);
	struct device *dev = &ctx->dsi->dev;

	struct drm_dsc_picture_parameter_set pps;

	int ret;

	if (ctx->prepared)
		return 0;

	sw43408_dsc_fhd_reset(ctx);

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = sw43408_dsc_fhd_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	if (ctx->dsi->dsc) {
		/* this panel uses DSC so send the pps */
		drm_dsc_pps_payload_pack(&pps, ctx->dsi->dsc);
		print_hex_dump(KERN_DEBUG, "DSC params:", DUMP_PREFIX_NONE,
			       16, 1, &pps, sizeof(pps), false);

		ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
		if (ret < 0) {
			dev_err(panel->dev, "failed to set pps: %d\n", ret);
			return ret;
		}
		msleep(28); /* TODO: Is this panel-dependent? */
	}

	ctx->prepared = true;

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return 0;
}

static int sw43408_dsc_fhd_unprepare(struct drm_panel *panel)
{
	struct sw43408_dsc_fhd *ctx = to_sw43408_dsc_fhd(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = sw43408_dsc_fhd_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode sw43408_dsc_fhd_mode = {
	.clock = (1080 + 20 + 32 + 20) * (2160 + 20 + 4 + 20) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 20,
	.hsync_end = 1080 + 20 + 32,
	.htotal = 1080 + 20 + 32 + 20,
	.vdisplay = 2160,
	.vsync_start = 2160 + 20,
	.vsync_end = 2160 + 20 + 4,
	.vtotal = 2160 + 20 + 4 + 20,
	.width_mm = 62,
	.height_mm = 124,
};

static int sw43408_dsc_fhd_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &sw43408_dsc_fhd_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs sw43408_dsc_fhd_panel_funcs = {
	.prepare = sw43408_dsc_fhd_prepare,
	.unprepare = sw43408_dsc_fhd_unprepare,
	.get_modes = sw43408_dsc_fhd_get_modes,
};

static int sw43408_dsc_fhd_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret = 0;
	uint16_t brightness;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	brightness = (uint16_t)backlight_get_brightness(bl);
	/* Brightness is sent in big-endian */
	brightness = cpu_to_be16(brightness);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	return ret;
}

static int sw43408_dsc_fhd_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret = 0;
	uint16_t brightness = 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}


static const struct backlight_ops sw43408_dsc_fhd_bl_ops = {
	.update_status = sw43408_dsc_fhd_bl_update_status,
	.get_brightness = sw43408_dsc_fhd_bl_get_brightness,
};

static struct backlight_device *
sw43408_dsc_fhd_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 900,
		.max_brightness = 900,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &sw43408_dsc_fhd_bl_ops, &props);
}

static int sw43408_dsc_fhd_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct sw43408_dsc_fhd *ctx;
	struct drm_dsc_config *dsc;

	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &sw43408_dsc_fhd_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->panel.backlight = sw43408_dsc_fhd_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	dsc = devm_kzalloc(&dsi->dev, sizeof(*dsc), GFP_KERNEL);
	if (!dsc)
		return -ENOMEM;

	dsc->dsc_version_major = 1;
	dsc->dsc_version_minor = 1;

	dsc->slice_height = 16;
	dsc->slice_width = 540;
	dsc->slice_count = 1;
	dsc->bits_per_component = 8;
	dsc->bits_per_pixel = 8 << 4; /* 4 fractional bits */
	dsc->block_pred_enable = true;

	dsi->dsc = dsc;

	return 0;
}

static void sw43408_dsc_fhd_remove(struct mipi_dsi_device *dsi)
{
	struct sw43408_dsc_fhd *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	return;
}

static const struct of_device_id sw43408_dsc_fhd_of_match[] = {
	{ .compatible = "lg,sw43408" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sw43408_dsc_fhd_of_match);

static struct mipi_dsi_driver sw43408_dsc_fhd_driver = {
	.probe = sw43408_dsc_fhd_probe,
	.remove = sw43408_dsc_fhd_remove,
	.driver = {
		.name = "panel-sw43408-dsc-fhd",
		.of_match_table = sw43408_dsc_fhd_of_match,
	},
};
module_mipi_dsi_driver(sw43408_dsc_fhd_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for sw43408 cmd mode dsc dsi panel");
MODULE_LICENSE("GPL v2");
