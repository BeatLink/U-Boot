// SPDX-License-Identifier: GPL-2.0+
/*
 * Xingbangda XBD599 MIPI-DSI panel, a 720x1440 module built around a
 * Sitronix ST7703 controller, as fitted to the PinePhone.
 *
 * Init sequence and timings taken from Linux
 * drivers/gpu/drm/panel/panel-sitronix-st7703.c.
 */

#include <backlight.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <log.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dsi_host.h>
#include <linux/delay.h>
#include <power/regulator.h>
#include "sunxi/sunxi_mipi_dsi.h"

#define ST7703_CMD_SETDISP	 0xB2
#define ST7703_CMD_SETRGBIF	 0xB3
#define ST7703_CMD_SETCYC	 0xB4
#define ST7703_CMD_SETBGP	 0xB5
#define ST7703_CMD_SETVCOM	 0xB6
#define ST7703_CMD_SETPOWER_EXT	 0xB8
#define ST7703_CMD_SETEXTC	 0xB9
#define ST7703_CMD_SETMIPI	 0xBA
#define ST7703_CMD_SETVDC	 0xBC
#define ST7703_CMD_UNKNOWN_BF	 0xBF
#define ST7703_CMD_SETSCR	 0xC0
#define ST7703_CMD_SETPOWER	 0xC1
#define ST7703_CMD_SETECO	 0xC6
#define ST7703_CMD_SETPANEL	 0xCC
#define ST7703_CMD_SETGAMMA	 0xE0
#define ST7703_CMD_SETEQ	 0xE3
#define ST7703_CMD_SETGIP1	 0xE9
#define ST7703_CMD_SETGIP2	 0xEA

struct xbd599_priv {
	struct udevice *backlight;
	struct udevice *host;
	struct udevice *vcc;
	struct udevice *iovcc;
	struct gpio_desc reset;
	struct mipi_dsi_device device;
};

static const struct display_timing xbd599_timing = {
	.pixelclock = { 69000000, 69000000, 69000000 },
	.hactive = { 720, 720, 720 },
	.hfront_porch = { 40, 40, 40 },
	.hsync_len = { 40, 40, 40 },
	.hback_porch = { 40, 40, 40 },
	.vactive = { 1440, 1440, 1440 },
	.vfront_porch = { 18, 18, 18 },
	.vsync_len = { 10, 10, 10 },
	.vback_porch = { 17, 17, 17 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW,
};

/* Each entry is a length byte, the DCS command, then that many payload bytes. */
static const u8 xbd599_init_seq[] = {
	4, ST7703_CMD_SETEXTC,
		0xF1, 0x12, 0x83,
	28, ST7703_CMD_SETMIPI,
		0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25,
		0x00, 0x91, 0x0A, 0x00, 0x00, 0x02, 0x4F, 0x11,
		0x00, 0x00, 0x37,
	5, ST7703_CMD_SETPOWER_EXT,
		0x25, 0x22, 0x20, 0x03,
	11, ST7703_CMD_SETRGBIF,
		0x10, 0x10, 0x05, 0x05, 0x03, 0xFF, 0x00, 0x00,
		0x00, 0x00,
	10, ST7703_CMD_SETSCR,
		0x73, 0x73, 0x50, 0x50, 0x00, 0xC0, 0x08, 0x70,
		0x00,
	2, ST7703_CMD_SETVDC,
		0x4E,
	2, ST7703_CMD_SETPANEL,
		0x0B,
	2, ST7703_CMD_SETCYC,
		0x80,
	4, ST7703_CMD_SETDISP,
		0xF0, 0x12, 0xF0,
	15, ST7703_CMD_SETEQ,
		0x00, 0x00, 0x0B, 0x0B, 0x10, 0x10, 0x00, 0x00,
		0x00, 0x00, 0xFF, 0x00, 0xC0, 0x10,
	6, ST7703_CMD_SETECO,
		0x01, 0x00, 0xFF, 0xFF, 0x00,
	13, ST7703_CMD_SETPOWER,
		0x74, 0x00, 0x32, 0x32, 0x77, 0xF1, 0xFF, 0xFF,
		0xCC, 0xCC, 0x77, 0x77,
	3, ST7703_CMD_SETBGP,
		0x07, 0x07,
	3, ST7703_CMD_SETVCOM,
		0x2C, 0x2C,
	4, ST7703_CMD_UNKNOWN_BF,
		0x02, 0x11, 0x00,
	64, ST7703_CMD_SETGIP1,
		0x82, 0x10, 0x06, 0x05, 0xA2, 0x0A, 0xA5, 0x12,
		0x31, 0x23, 0x37, 0x83, 0x04, 0xBC, 0x27, 0x38,
		0x0C, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x00,
		0x03, 0x00, 0x00, 0x00, 0x75, 0x75, 0x31, 0x88,
		0x88, 0x88, 0x88, 0x88, 0x88, 0x13, 0x88, 0x64,
		0x64, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
		0x02, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	62, ST7703_CMD_SETGIP2,
		0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x02, 0x46, 0x02, 0x88,
		0x88, 0x88, 0x88, 0x88, 0x88, 0x64, 0x88, 0x13,
		0x57, 0x13, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
		0x75, 0x88, 0x23, 0x14, 0x00, 0x00, 0x02, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0A,
		0xA5, 0x00, 0x00, 0x00, 0x00,
	35, ST7703_CMD_SETGAMMA,
		0x00, 0x09, 0x0D, 0x23, 0x27, 0x3C, 0x41, 0x35,
		0x07, 0x0D, 0x0E, 0x12, 0x13, 0x10, 0x12, 0x12,
		0x18, 0x00, 0x09, 0x0D, 0x23, 0x27, 0x3C, 0x41,
		0x35, 0x07, 0x0D, 0x0E, 0x12, 0x13, 0x10, 0x12,
		0x12, 0x18,
};


static int xbd599_send_init_seq(struct mipi_dsi_device *dsi)
{
	const u8 *p = xbd599_init_seq;
	const u8 *end = xbd599_init_seq + ARRAY_SIZE(xbd599_init_seq);
	int ret;

	while (p < end) {
		u8 len = *p++;

		ret = mipi_dsi_dcs_write_buffer(dsi, p, len);
		if (ret < 0) {
			log_err("xbd599: init command 0x%02x failed: %d\n",
				*p, ret);
			return ret;
		}
		p += len;
	}

	return 0;
}

static int xbd599_get_display_timing(struct udevice *dev,
				     struct display_timing *timing)
{
	memcpy(timing, &xbd599_timing, sizeof(*timing));

	return 0;
}

static int xbd599_enable(struct udevice *dev)
{
	struct xbd599_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *dsi = &priv->device;
	struct display_timing timing;
	int ret;

	ret = uclass_get_device(UCLASS_DSI_HOST, 0, &priv->host);
	if (ret) {
		dev_err(dev, "No DSI host found: %d\n", ret);
		return ret;
	}

	dsi->dev = dev;
	dsi->host = sunxi_mipi_dsi_host(priv->host);
	if (!dsi->host)
		return -ENODEV;

	strlcpy(dsi->name, "xbd599", sizeof(dsi->name));
	dsi->channel = 0;
	dsi->lanes = plat->lanes;
	dsi->format = plat->format;
	dsi->mode_flags = plat->mode_flags;
	plat->device = dsi;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		return ret;

	xbd599_get_display_timing(dev, &timing);

	ret = dsi_host_init(priv->host, dsi, &timing, plat->lanes, NULL);
	if (ret < 0) {
		dev_err(dev, "DSI host init failed: %d\n", ret);
		return ret;
	}

	/* Hold the panel in reset until both of its supplies are up. */
	dm_gpio_set_value(&priv->reset, 1);

	if (priv->iovcc) {
		ret = regulator_set_enable(priv->iovcc, true);
		if (ret)
			dev_err(dev, "Cannot enable iovcc: %d\n", ret);
	}

	if (priv->vcc) {
		ret = regulator_set_enable(priv->vcc, true);
		if (ret)
			dev_err(dev, "Cannot enable vcc: %d\n", ret);
	}

	mdelay(20);

	/* This first call brings up clocks and the D-PHY, leaving command mode active. */
	ret = dsi_host_enable(priv->host);
	if (ret < 0)
		return ret;

	dm_gpio_set_value(&priv->reset, 0);
	mdelay(20);

	ret = xbd599_send_init_seq(dsi);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	/* The controller needs 120ms to wake before it will accept display-on. */
	mdelay(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		return ret;

	/* With the panel initialised, the second call starts high-speed video. */
	ret = dsi_host_enable(priv->host);
	if (ret < 0)
		return ret;

	if (priv->backlight) {
		ret = backlight_enable(priv->backlight);
		if (ret && ret != -ENOSYS && ret != -ENOENT)
			return ret;
	}

	return 0;
}

static int xbd599_of_to_plat(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct xbd599_priv *priv = dev_get_priv(dev);
	int ret;

	plat->lanes = 4;
	plat->format = MIPI_DSI_FMT_RGB888;
	/* Linux runs this panel in burst mode, which this host does not do. */
	plat->mode_flags = MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset,
				   GPIOD_IS_OUT | GPIOD_ACTIVE_LOW);
	if (ret) {
		dev_err(dev, "Cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	return 0;
}

static int xbd599_probe(struct udevice *dev)
{
	struct xbd599_priv *priv = dev_get_priv(dev);
	int ret;

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret) {
		dev_warn(dev, "Cannot get backlight: %d\n", ret);
		priv->backlight = NULL;
	}

	/* Missing rails leave the panel dark but silent, so say so. */
	ret = device_get_supply_regulator(dev, "vcc-supply", &priv->vcc);
	if (ret) {
		dev_err(dev, "Cannot get vcc supply: %d\n", ret);
		priv->vcc = NULL;
	}

	ret = device_get_supply_regulator(dev, "iovcc-supply", &priv->iovcc);
	if (ret) {
		dev_err(dev, "Cannot get iovcc supply: %d\n", ret);
		priv->iovcc = NULL;
	}

	return 0;
}

static const struct panel_ops xbd599_ops = {
	.enable_backlight	= xbd599_enable,
	.get_display_timing	= xbd599_get_display_timing,
};

static const struct udevice_id xbd599_ids[] = {
	{ .compatible = "xingbangda,xbd599" },
	{ }
};

U_BOOT_DRIVER(panel_xingbangda_xbd599) = {
	.name		= "panel-xingbangda-xbd599",
	.id		= UCLASS_PANEL,
	.of_match	= xbd599_ids,
	.ops		= &xbd599_ops,
	.of_to_plat	= xbd599_of_to_plat,
	.probe		= xbd599_probe,
	.priv_auto	= sizeof(struct xbd599_priv),
	.plat_auto	= sizeof(struct mipi_dsi_panel_plat),
};
