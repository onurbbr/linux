// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI driver for the Xiaomi "m7 38 0c 0a" FHD+ OLED video mode panel
 * found in the Xiaomi Redmi Note 12 4G (SM6225, "tapas"/"topaz").
 *
 * The panel is a 1080x2400 AMOLED controller that is closely related to the
 * Samsung S6E3FC family; the initialization sequence was taken from the
 * downstream device tree panel description.
 *
 * Power sequencing: vddio -> enable GPIO -> reset pulse -> panel init, and the
 * reverse order on power-down.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct xiaomi_m7_38_0c_0a {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator *supply;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
};

static inline
struct xiaomi_m7_38_0c_0a *to_xiaomi_m7_38_0c_0a(struct drm_panel *panel)
{
	return container_of_const(panel, struct xiaomi_m7_38_0c_0a, panel);
}

/*
 * The reset line idles low; the downstream reset sequence is
 * <0 10 1 10>, i.e. assert low, wait, release high, wait.
 */
static void xiaomi_m7_38_0c_0a_reset(struct xiaomi_m7_38_0c_0a *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
}

static int xiaomi_m7_38_0c_0a_on(struct xiaomi_m7_38_0c_0a *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd0, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x25, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x50);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x2f, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0xa5, 0xa5);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 32);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x01, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf,
				     0x09, 0x30, 0x95, 0x43, 0xa9, 0x43, 0xa9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9d, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9e,
				     0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09,
				     0x60, 0x04, 0x38, 0x00, 0x28, 0x02, 0x1c,
				     0x02, 0x1c, 0x02, 0x00, 0x02, 0x0e, 0x00,
				     0x20, 0x03, 0xdd, 0x00, 0x07, 0x00, 0x0c,
				     0x02, 0x77, 0x02, 0x8b, 0x18, 0x00, 0x10,
				     0xf0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b,
				     0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46,
				     0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
				     0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09,
				     0x40, 0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa,
				     0x19, 0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a,
				     0xb6, 0x2a, 0xf6, 0x2b, 0x34, 0x2b, 0x74,
				     0x3b, 0x74, 0x6b, 0xf4, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x40, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x11, 0xfe);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x0d, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x0c, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x01, 0xcd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x06, 0xf4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x28);
	/* Program the initial brightness before enabling the output. */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x00, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

/*
 * Downstream has no explicit off command for this panel, but its
 * "pre-off" sequence is display off, wait 20ms, enter sleep mode, wait 150ms.
 */
static int xiaomi_m7_38_0c_0a_off(struct xiaomi_m7_38_0c_0a *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 20000, 21000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);

	return dsi_ctx.accum_err;
}

static int xiaomi_m7_38_0c_0a_prepare(struct drm_panel *panel)
{
	struct xiaomi_m7_38_0c_0a *ctx = to_xiaomi_m7_38_0c_0a(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(ctx->enable_gpio, 1);
	usleep_range(10000, 11000);

	xiaomi_m7_38_0c_0a_reset(ctx);

	ret = xiaomi_m7_38_0c_0a_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		gpiod_set_value_cansleep(ctx->enable_gpio, 0);
		regulator_disable(ctx->supply);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	/* Wait for the panel to apply the compressed stream configuration. */
	msleep(28);

	return 0;
}

static int xiaomi_m7_38_0c_0a_unprepare(struct drm_panel *panel)
{
	struct xiaomi_m7_38_0c_0a *ctx = to_xiaomi_m7_38_0c_0a(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = xiaomi_m7_38_0c_0a_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	gpiod_set_value_cansleep(ctx->enable_gpio, 0);
	regulator_disable(ctx->supply);

	return 0;
}

static const struct drm_display_mode xiaomi_m7_38_0c_0a_mode = {
	.clock = (1080 + 120 + 28 + 120) * (2400 + 20 + 2 + 10) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 120,
	.hsync_end = 1080 + 120 + 28,
	.htotal = 1080 + 120 + 28 + 120,
	.vdisplay = 2400,
	.vsync_start = 2400 + 20,
	.vsync_end = 2400 + 20 + 2,
	.vtotal = 2400 + 20 + 2 + 10,
	.width_mm = 69,
	.height_mm = 154,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int xiaomi_m7_38_0c_0a_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &xiaomi_m7_38_0c_0a_mode);
}

static const struct drm_panel_funcs xiaomi_m7_38_0c_0a_panel_funcs = {
	.prepare = xiaomi_m7_38_0c_0a_prepare,
	.unprepare = xiaomi_m7_38_0c_0a_unprepare,
	.get_modes = xiaomi_m7_38_0c_0a_get_modes,
};

/* The panel uses a raw 11-bit DCS brightness value. */
static int xiaomi_m7_38_0c_0a_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int xiaomi_m7_38_0c_0a_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops xiaomi_m7_38_0c_0a_bl_ops = {
	.update_status = xiaomi_m7_38_0c_0a_bl_update_status,
	.get_brightness = xiaomi_m7_38_0c_0a_bl_get_brightness,
};

static struct backlight_device *
xiaomi_m7_38_0c_0a_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 307,
		.max_brightness = 2047,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &xiaomi_m7_38_0c_0a_bl_ops, &props);
}

static int xiaomi_m7_38_0c_0a_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct xiaomi_m7_38_0c_0a *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct xiaomi_m7_38_0c_0a, panel,
				   &xiaomi_m7_38_0c_0a_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supply = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "Failed to get vddio regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio),
				     "Failed to get enable-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = xiaomi_m7_38_0c_0a_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports a compressed stream. */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 20;
	ctx->dsc.slice_width = xiaomi_m7_38_0c_0a_mode.hdisplay / 2;
	ctx->dsc.slice_count = 2;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void xiaomi_m7_38_0c_0a_remove(struct mipi_dsi_device *dsi)
{
	struct xiaomi_m7_38_0c_0a *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id xiaomi_m7_38_0c_0a_of_match[] = {
	{ .compatible = "xiaomi,m7-38-0c-0a" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, xiaomi_m7_38_0c_0a_of_match);

static struct mipi_dsi_driver xiaomi_m7_38_0c_0a_driver = {
	.probe = xiaomi_m7_38_0c_0a_probe,
	.remove = xiaomi_m7_38_0c_0a_remove,
	.driver = {
		.name = "panel-xiaomi-m7-38-0c-0a",
		.of_match_table = xiaomi_m7_38_0c_0a_of_match,
	},
};
module_mipi_dsi_driver(xiaomi_m7_38_0c_0a_driver);

MODULE_AUTHOR("Onur BÜBER <onurbuber@engineer.com>");
MODULE_DESCRIPTION("DRM driver for Xiaomi m7 38 0c 0a video mode FHD+ DSI panel");
MODULE_LICENSE("GPL");
