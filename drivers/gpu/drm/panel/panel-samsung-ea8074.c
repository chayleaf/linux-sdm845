// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung EA8074 DriverIC panels driver
 *
 * Copyright (c) 2024 Arseniy Velikanov <me@adomerle.xyz>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

struct ea8074 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct ea8074_desc *desc;

	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
	struct regulator *vddpos;
	struct regulator *vddneg;
};

struct ea8074_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct ea8074 *ctx);

	bool has_dcs_backlight;
};

static inline
struct ea8074 *to_ea8074(struct drm_panel *panel)
{
	return container_of(panel, struct ea8074, panel);
}

static int dipper_ea8074_init_sequence(struct ea8074 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_set_page_address(dsi, 0x0000, 0x08c7);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0xef, 0xf0, 0x31, 0x00, 0x33, 0x31, 0x14, 0x35);
	dsi_dcs_write_seq(dsi, 0xb0, 0x01);
	dsi_dcs_write_seq(dsi, 0xbb, 0x03);
	dsi_dcs_write_seq(dsi, 0xb0, 0x4f);
	dsi_dcs_write_seq(dsi, 0xcb, 0x00);
	dsi_dcs_write_seq(dsi, 0xb0, 0x6b);
	dsi_dcs_write_seq(dsi, 0xcb, 0x00);
	dsi_dcs_write_seq(dsi, 0xf7, 0x03);
	dsi_dcs_write_seq(dsi, 0xb0, 0x05);
	dsi_dcs_write_seq(dsi, 0xb1, 0x10);
	dsi_dcs_write_seq(dsi, 0xb0, 0x02);
	dsi_dcs_write_seq(dsi, 0xd5, 0x02, 0x17, 0x54, 0x14);
	dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0xfc, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0xd2, 0x9f, 0xf0);
	dsi_dcs_write_seq(dsi, 0xb0, 0x0e);
	dsi_dcs_write_seq(dsi, 0xd2, 0x70);
	dsi_dcs_write_seq(dsi, 0xb0, 0x04);
	dsi_dcs_write_seq(dsi, 0xd2, 0x20);
	dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	dsi_dcs_write_seq(dsi, 0xfc, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x0000);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	msleep(110);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	msleep(30);

	return 0;
}

static const struct drm_display_mode dipper_ea8074_modes[] = {
	{
		.clock = (1080 + 56 + 18 + 56) * (2248 + 26 + 12 + 24) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 56,
		.hsync_end = 1080 + 56 + 18,
		.htotal = 1080 + 56 + 18 + 56,
		.vdisplay = 2248,
		.vsync_start = 2248 + 26,
		.vsync_end = 2248 + 26 + 12,
		.vtotal = 2248 + 26 + 12 + 24,
	},
};

static const struct ea8074_desc dipper_ea8074_desc = {
	.modes = dipper_ea8074_modes,
	.num_modes = ARRAY_SIZE(dipper_ea8074_modes),
	.dsi_info = {
		.type = "EA8074-dipper",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 68,
	.height_mm = 142,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = dipper_ea8074_init_sequence,
};

static void ea8074_reset(struct ea8074 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int ea8074_prepare(struct drm_panel *panel)
{
	struct ea8074 *ctx = to_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_enable(ctx->vddio);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(ctx->vddpos);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddpos regulator: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(ctx->vddneg);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddneg regulator: %d\n", ret);
		return ret;
	}

	ea8074_reset(ctx);

	ret = ctx->desc->init_sequence(ctx);
	if (ret < 0) {
		regulator_disable(ctx->vddio);
		regulator_disable(ctx->vddpos);
		regulator_disable(ctx->vddneg);
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int ea8074_disable(struct drm_panel *panel)
{
	struct ea8074 *ctx = to_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	usleep_range(17000, 18000);

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int ea8074_unprepare(struct drm_panel *panel)
{
	struct ea8074 *ctx = to_ea8074(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	regulator_disable(ctx->vddio);
	regulator_disable(ctx->vddpos);
	regulator_disable(ctx->vddneg);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static void ea8074_remove(struct mipi_dsi_device *dsi)
{
	struct ea8074 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	return;
}

static int ea8074_get_modes(struct drm_panel *panel,
					 struct drm_connector *connector)
{
	struct ea8074 *ctx = to_ea8074(panel);
	int i;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct drm_display_mode *m = &ctx->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = ctx->desc->width_mm;
	connector->display_info.height_mm = ctx->desc->height_mm;

	return ctx->desc->num_modes;
}

static const struct drm_panel_funcs ea8074_panel_funcs = {
	.prepare = ea8074_prepare,
	.disable = ea8074_disable,
	.unprepare = ea8074_unprepare,
	.get_modes = ea8074_get_modes,
};

static int ea8074_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ea8074 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio), "failed to get vddio regulator\n");

	ctx->vddpos = devm_regulator_get(dev, "vddpos");
	if (IS_ERR(ctx->vddpos))
		return dev_err_probe(dev, PTR_ERR(ctx->vddpos), "failed to get vddpos regulator\n");

	ctx->vddneg = devm_regulator_get(dev, "vddneg");
	if (IS_ERR(ctx->vddneg))
		return dev_err_probe(dev, PTR_ERR(ctx->vddneg), "failed to get vddneg regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);
	drm_panel_init(&ctx->panel, dev, &ea8074_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	ctx->dsi->lanes = ctx->desc->lanes;
	ctx->dsi->format = ctx->desc->format;
	ctx->dsi->mode_flags = ctx->desc->mode_flags;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static const struct of_device_id ea8074_of_match[] = {
	{
		.compatible = "xiaomi,ea8074-dipper",
		.data = &dipper_ea8074_desc,
	},
	{}
};
MODULE_DEVICE_TABLE(of, ea8074_of_match);

static struct mipi_dsi_driver ea8074_driver = {
	.probe = ea8074_probe,
	.remove = ea8074_remove,
	.driver = {
		.name = "panel-samsung-ea8074",
		.of_match_table = ea8074_of_match,
	},
};
module_mipi_dsi_driver(ea8074_driver);

MODULE_AUTHOR("Arseniy Velikanov <me@adomerle.xyz>");
MODULE_DESCRIPTION("DRM driver for the Samsung EA8074 based panels");
MODULE_LICENSE("GPL");
