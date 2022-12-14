// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Teguh Sobirin
 * Author: Teguh Sobirin <teguh@sobir.in>
 *
 * This driver is for the DSI interface to Innolux panel
 * Using the TD4328 display driver IC from Synaptics.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct td4328 {
	struct device *dev;
	struct drm_panel panel;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	enum drm_panel_orientation orientation;
	bool prepared;
};

static inline struct td4328 *panel_to_td4328(struct drm_panel *panel)
{
	return container_of(panel, struct td4328, panel);
}

static int td4328_init_sequence(struct td4328 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct device *dev = ctx->dev;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(70);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, 1080 - 1);
	if (ret < 0) {
		dev_err(dev, "Failed to set sleep column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0, 1920 - 1);
	if (ret < 0) {
		dev_err(dev, "Failed to set sleep page address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "Panel init sequence done\n");
	return 0;
}

static int td4328_unprepare(struct drm_panel *panel)
{
	struct td4328 *ctx = panel_to_td4328(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		dev_err(ctx->dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	regulator_disable(ctx->supply);

	ctx->prepared = false;

	return 0;
}

static int td4328_prepare(struct drm_panel *panel)
{
	struct td4328 *ctx = panel_to_td4328(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->prepared)
		return 0;

	dev_dbg(ctx->dev, "Resetting the panel\n");
	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to enable supply: %d\n", ret);
		return ret;
	}

	msleep(20);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10, 20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);

	msleep(20);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to exit sleep mode: %d\n", ret);
		goto disable_supply;
	}

	msleep(250);

	ret = td4328_init_sequence(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "Panel init sequence failed: %d\n", ret);
		goto disable_supply;
	}

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set display on: %d\n", ret);
		goto disable_supply;
	}

	msleep(50);

	ctx->prepared = true;

	return 0;

disable_supply:
	regulator_disable(ctx->supply);
	return ret;
}

static const struct drm_display_mode default_mode = {
	.clock       = (1080 + 60 + 10 + 60) * (1920 + 20 + 8 + 20) * 60 / 1000,
	.hdisplay    = 1080,
	.hsync_start = 1080 + 60,
	.hsync_end   = 1080 + 60 + 10,
	.htotal      = 1080 + 60 + 10 + 60,
	.vdisplay    = 1920,
	.vsync_start = 1920 + 20,
	.vsync_end   = 1920 + 20 + 8,
	.vtotal      = 1920 + 20 + 8 + 20,
	.width_mm    = 75,
	.height_mm   = 132,
};

static int td4328_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct td4328 *ctx = panel_to_td4328(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(ctx->dev, "Failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);
	drm_connector_set_panel_orientation(connector, ctx->orientation);

	return 1;
}

static const struct drm_panel_funcs td4328_funcs = {
	.unprepare	= td4328_unprepare,
	.prepare	= td4328_prepare,
	.get_modes	= td4328_get_modes,
};

static int td4328_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct td4328 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request vdd regulator: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request vddio regulator: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "vddpos");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request vddpos regulator: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "vddneg");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request vddneg regulator: %d\n", ret);
		return ret;
	}

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0) {
		dev_err(dev, "Failed to get orientation %d\n", ret);
		return ret;
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM
		| MIPI_DSI_MODE_VIDEO_HSE
		| MIPI_DSI_CLOCK_NON_CONTINUOUS
		| MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_NO_EOT_PACKET;

	drm_panel_init(&ctx->panel, &dsi->dev, &td4328_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void td4328_shutdown(struct mipi_dsi_device *dsi)
{
	struct td4328 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = drm_panel_unprepare(&ctx->panel);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to unprepare panel: %d\n", ret);

	ret = drm_panel_disable(&ctx->panel);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to disable panel: %d\n", ret);
}

static void td4328_remove(struct mipi_dsi_device *dsi)
{
	struct td4328 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	td4328_shutdown(dsi);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id td4328_of_match[] = {
	{ .compatible = "innolux,td4328" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, td4328_of_match);

static struct mipi_dsi_driver td4328_driver = {
	.driver = {
		.name = "panel-innolux-td4328",
		.of_match_table = td4328_of_match,
	},
	.probe	= td4328_probe,
	.remove = td4328_remove,
	.shutdown = td4328_shutdown,
};
module_mipi_dsi_driver(td4328_driver);

MODULE_AUTHOR("Teguh Sobirin <teguh@sobir.in>");
MODULE_DESCRIPTION("DRM driver for TD4328 cmd mode DSI Innolux panel");
MODULE_LICENSE("GPL v2");
