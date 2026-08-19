// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner LCD driver
 *
 * (C) Copyright 2017 Vasily Khoruzhick <anarsoul@gmail.com>
 */

#include <display.h>
#include <log.h>
#include <panel.h>
#include <video.h>
#include <video_bridge.h>
#include <backlight.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <dm/ofnode.h>
#include <dm/root.h>
#include <dsi_host.h>
#include <edid.h>
#include <mipi_dsi.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/lcdc.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <sunxi_gpio.h>

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
#include "sunxi_tcon.h"
#endif

struct sunxi_lcd_priv {
	struct display_timing timing;
	int panel_bpp;
#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
	bool has_dsi;
	unsigned int lanes;
	struct udevice *dsi_host;
	struct udevice *panel;
#endif
};

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
static const char * const sunxi_dsi_compatibles[] = {
	"allwinner,sun50i-a64-mipi-dsi",
	"allwinner,sun6i-a31-mipi-dsi",
};

static ofnode sunxi_lcd_find_dsi_node(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sunxi_dsi_compatibles); i++) {
		ofnode node = ofnode_by_compatible(ofnode_null(),
						   sunxi_dsi_compatibles[i]);

		if (ofnode_valid(node) && ofnode_is_enabled(node))
			return node;
	}

	return ofnode_null();
}

/* The panel is the DSI host's only child that carries a compatible string. */
static ofnode sunxi_lcd_find_panel_node(ofnode dsi_node)
{
	ofnode node;

	ofnode_for_each_subnode(node, dsi_node) {
		if (ofnode_read_prop(node, "compatible", NULL))
			return node;
	}

	return ofnode_null();
}

static int sunxi_lcd_setup_dsi(struct udevice *dev)
{
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat;
	ofnode dsi_node, panel_node;
	int ret;

	dsi_node = sunxi_lcd_find_dsi_node();
	if (!ofnode_valid(dsi_node))
		return -ENODEV;

	panel_node = sunxi_lcd_find_panel_node(dsi_node);
	if (!ofnode_valid(panel_node))
		return -ENODEV;

	ret = uclass_get_device_by_ofnode(UCLASS_DSI_HOST, dsi_node,
					  &priv->dsi_host);
	if (ret == -ENODEV) {
		ret = device_bind_driver_to_node(dm_root(), "sunxi-mipi-dsi",
						 ofnode_get_name(dsi_node),
						 dsi_node, NULL);
		if (ret && ret != -EEXIST)
			return ret;

		ret = uclass_get_device_by_ofnode(UCLASS_DSI_HOST, dsi_node,
						  &priv->dsi_host);
	}
	if (ret)
		return ret;

	ret = uclass_get_device_by_ofnode(UCLASS_PANEL, panel_node,
					  &priv->panel);
	if (ret == -ENODEV) {
		ret = lists_bind_fdt(priv->dsi_host, panel_node, &priv->panel,
				     NULL, false);
		if (ret && ret != -EEXIST)
			return ret;

		ret = uclass_get_device_by_ofnode(UCLASS_PANEL, panel_node,
						  &priv->panel);
	}
	if (ret)
		return ret;

	ret = panel_get_display_timing(priv->panel, &priv->timing);
	if (ret)
		return ret;

	plat = dev_get_plat(priv->panel);
	priv->lanes = plat->lanes ? plat->lanes : 4;
	priv->panel_bpp = mipi_dsi_pixel_format_to_bpp(plat->format);
	priv->has_dsi = true;

	return 0;
}

/*
 * PLL_MIPI multiplies PLL_VIDEO0 by n*k/m and feeds TCON0 in DSI mode. Its own
 * setter is A31-only and allows a k of one, which the A64 does not.
 */
static void sunxi_lcd_set_mipi_pll(unsigned int rate)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	unsigned int best_k = 2, best_m = 1, best_n = 1, best_diff = ~0U;
	unsigned int src = clock_get_pll3() / 1000;
	unsigned int k, m, n, value, diff;

	rate /= 1000;

	for (k = 2; k <= 4; k++) {
		for (m = 1; m <= 16; m++) {
			for (n = 1; n <= 16; n++) {
				value = src * n * k / m;
				if (value > rate)
					continue;

				diff = rate - value;
				if (diff < best_diff) {
					best_diff = diff;
					best_k = k;
					best_m = m;
					best_n = n;
				}
			}
		}
	}

	writel(CCM_MIPI_PLL_CTRL_EN | CCM_MIPI_PLL_CTRL_LDO_EN |
	       CCM_MIPI_PLL_CTRL_N(best_n) | CCM_MIPI_PLL_CTRL_K(best_k) |
	       CCM_MIPI_PLL_CTRL_M(best_m), &ccm->mipi_pll_cfg);

	debug("pll_mipi: %ukHz requested, %ukHz set from %ukHz\n",
	      rate, src * best_n * best_k / best_m, src);

	udelay(100);
}

/* TCON0 drives the DSI host over its 8080 CPU interface rather than an RGB bus. */
static void sunxi_lcd_tcon0_dsi_mode_set(struct sunxi_lcdc_reg * const lcdc,
					 const struct display_timing *mode,
					 unsigned int bpp, unsigned int lanes)
{
	void __iomem * const base = (void __iomem *)lcdc;
	u32 block_space, htotal, start_delay;

	clrbits_le32(&lcdc->tcon0_ctrl, SUN4I_TCON0_CTL_IF_MASK);
	setbits_le32(&lcdc->tcon0_ctrl, SUN4I_TCON0_CTL_IF_8080);
	setbits_le32(base + SUN4I_TCON_ECC_FIFO_REG, SUN4I_TCON_ECC_FIFO_EN);

	writel(SUN4I_TCON0_CPU_IF_MODE_DSI |
	       SUN4I_TCON0_CPU_IF_TRI_FIFO_FLUSH |
	       SUN4I_TCON0_CPU_IF_TRI_FIFO_EN |
	       SUN4I_TCON0_CPU_IF_TRI_EN,
	       &lcdc->tcon0_cpu_intf);

	htotal = mode->hactive.typ + mode->hback_porch.typ +
		 mode->hfront_porch.typ + mode->hsync_len.typ;

	block_space = htotal * bpp / (SUN6I_DSI_TCON_DIV * lanes);
	if (block_space > mode->hactive.typ + 40)
		block_space -= mode->hactive.typ + 40;
	else
		block_space = 1;
	if (block_space > 4095)
		block_space = 4095;

	writel(SUN4I_TCON0_CPU_TRI0_BLOCK_SPACE(block_space) |
	       SUN4I_TCON0_CPU_TRI0_BLOCK_SIZE(mode->hactive.typ),
	       base + SUN4I_TCON0_CPU_TRI0_REG);

	writel(SUN4I_TCON0_CPU_TRI1_BLOCK_NUM(mode->vactive.typ),
	       base + SUN4I_TCON0_CPU_TRI1_REG);

	start_delay = mode->vback_porch.typ + mode->vfront_porch.typ +
		      mode->vsync_len.typ - 10 - 1;
	start_delay = start_delay * htotal * 149;
	start_delay = start_delay / (mode->pixelclock.typ / 1000) / 8;
	if (start_delay > 65535)
		start_delay = 65535;

	writel(SUN4I_TCON0_CPU_TRI2_START_DELAY(start_delay) |
	       SUN4I_TCON0_CPU_TRI2_TRANS_START_SET(10),
	       base + SUN4I_TCON0_CPU_TRI2_REG);

	writel(SUN4I_TCON_SAFE_PERIOD_NUM(3000) |
	       SUN4I_TCON_SAFE_PERIOD_MODE(3),
	       base + SUN4I_TCON_SAFE_PERIOD_REG);

	/* Linux leaves these tristate bits set for DSI; the RGB path clears them. */
	writel(0xe0000000, &lcdc->tcon0_io_tristate);
}
#endif /* CONFIG_VIDEO_SUNXI_MIPI_DSI */

static void sunxi_lcdc_config_pinmux(void)
{
#ifdef CONFIG_MACH_SUN50I
	int pin;

	for (pin = SUNXI_GPD(0); pin <= SUNXI_GPD(21); pin++) {
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPD_LCD0);
		sunxi_gpio_set_drv(pin, 3);
	}
#endif
}

static int sunxi_lcd_enable(struct udevice *dev, int bpp,
			    const struct display_timing *edid)
{
	struct sunxi_ccm_reg * const ccm =
	       (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_lcdc_reg * const lcdc =
	       (struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);
	struct udevice *backlight;
	int clk_div, clk_double, ret;

	/* Reset off */
	setbits_le32(&ccm->ahb_reset1_cfg, 1 << AHB_RESET_OFFSET_LCD0);
	/* Clock on */
	setbits_le32(&ccm->ahb_gate1, 1 << AHB_GATE_OFFSET_LCD0);

	lcdc_init(lcdc);

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
	if (priv->has_dsi) {
		/* PLL_VIDEO0 clocks the D-PHY at the per-lane bit rate. */
		clock_set_pll3(edid->pixelclock.typ * priv->panel_bpp /
			       priv->lanes);

		/* TCON0 runs at four times the pixel clock in DSI mode. */
		sunxi_lcd_set_mipi_pll(edid->pixelclock.typ *
				       SUN6I_DSI_TCON_DIV);
		clk_div = SUN6I_DSI_TCON_DIV;

		writel(CCM_LCD_CH0_CTRL_GATE | CCM_LCD_CH0_CTRL_RST |
		       CCM_LCD_CH0_CTRL_A64_MIPI_PLL, &ccm->lcd0_clk_cfg);
	} else
#endif
	{
		sunxi_lcdc_config_pinmux();
		lcdc_pll_set(ccm, 0, edid->pixelclock.typ / 1000,
			     &clk_div, &clk_double, false);
	}

	lcdc_tcon0_mode_set(lcdc, edid, clk_div, false,
			    priv->panel_bpp, CONFIG_VIDEO_LCD_DCLK_PHASE);

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
	if (priv->has_dsi) {
		sunxi_lcd_tcon0_dsi_mode_set(lcdc, edid, priv->panel_bpp,
					     priv->lanes);

		/*
		 * The panel sends its init sequence in command mode and then
		 * puts the host into video mode, so it runs before the TCON.
		 */
		ret = panel_enable_backlight(priv->panel);
		if (ret) {
			dev_err(dev, "Cannot enable panel: %d\n", ret);
			return ret;
		}
	}
#endif

	lcdc_enable(lcdc, priv->panel_bpp);

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
	if (priv->has_dsi)
		return 0;
#endif

	ret = uclass_get_device(UCLASS_PANEL_BACKLIGHT, 0, &backlight);
	if (!ret)
		backlight_enable(backlight);

	return 0;
}

static int sunxi_lcd_read_timing(struct udevice *dev,
				 struct display_timing *timing)
{
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);

	memcpy(timing, &priv->timing, sizeof(struct display_timing));

	return 0;
}

static int sunxi_lcd_probe(struct udevice *dev)
{
	struct udevice *cdev;
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);
	int ret;
	int node, timing_node, val;

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
	ret = sunxi_lcd_setup_dsi(dev);
	if (!ret)
		return 0;

	debug("%s: no DSI panel (ret=%d)\n", __func__, ret);
#endif

#ifdef CONFIG_VIDEO_BRIDGE
	/* Try to get timings from bridge first */
	ret = uclass_get_device(UCLASS_VIDEO_BRIDGE, 0, &cdev);
	if (!ret) {
		u8 edid[EDID_SIZE];
		int channel_bpp;

		ret = video_bridge_attach(cdev);
		if (ret) {
			debug("video bridge attach failed: %d\n", ret);
			return ret;
		}
		ret = video_bridge_read_edid(cdev, edid, EDID_SIZE);
		if (ret > 0) {
			ret = edid_get_timing(edid, ret,
					      &priv->timing, &channel_bpp);
			priv->panel_bpp = channel_bpp * 3;
			if (!ret)
				return ret;
		}
	}
#endif

	/* Fallback to timings from DT if there's no bridge or
	 * if reading EDID failed
	 */
	ret = uclass_get_device(UCLASS_PANEL, 0, &cdev);
	if (ret) {
		debug("video panel not found: %d\n", ret);
		return ret;
	}

	if (fdtdec_decode_display_timing(gd->fdt_blob, dev_of_offset(cdev),
					 0, &priv->timing)) {
		debug("%s: Failed to decode display timing\n", __func__);
		return -EINVAL;
	}
	timing_node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(cdev),
					 "display-timings");
	node = fdt_first_subnode(gd->fdt_blob, timing_node);
	val = fdtdec_get_int(gd->fdt_blob, node, "bits-per-pixel", -1);
	if (val != -1)
		priv->panel_bpp = val;
	else
		priv->panel_bpp = 18;

	return 0;
}

/*
 * The kernel's own driver expects to find the pipeline idle, and wedges on a
 * DSI block left streaming, so give it back the way it was handed over.
 */
static int sunxi_lcd_remove(struct udevice *dev)
{
	struct sunxi_lcdc_reg * const lcdc =
	       (struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);
	struct udevice *backlight;

	/* A board handing the OS a live simple framebuffer keeps it lit. */
	if (!priv->has_dsi)
		return 0;

	if (!uclass_get_device(UCLASS_PANEL_BACKLIGHT, 0, &backlight))
		backlight_set_brightness(backlight, BACKLIGHT_OFF);

#ifdef CONFIG_VIDEO_SUNXI_MIPI_DSI
	if (priv->dsi_host) {
		struct dsi_host_ops *ops = dsi_host_get_ops(priv->dsi_host);

		if (ops && ops->disable)
			ops->disable(priv->dsi_host);
	}
#endif

	clrbits_le32(&lcdc->tcon0_dclk, SUNXI_LCDC_TCON0_DCLK_ENABLE);
	clrbits_le32(&lcdc->tcon0_ctrl, SUNXI_LCDC_TCON0_CTRL_ENABLE);
	clrbits_le32(&lcdc->ctrl, SUNXI_LCDC_CTRL_TCON_ENABLE);

	return 0;
}

static const struct dm_display_ops sunxi_lcd_ops = {
	.read_timing = sunxi_lcd_read_timing,
	.enable = sunxi_lcd_enable,
};

U_BOOT_DRIVER(sunxi_lcd) = {
	.name   = "sunxi_lcd",
	.id     = UCLASS_DISPLAY,
	.ops    = &sunxi_lcd_ops,
	.probe  = sunxi_lcd_probe,
	.remove = sunxi_lcd_remove,
	.priv_auto	= sizeof(struct sunxi_lcd_priv),
	.flags  = DM_FLAG_OS_PREPARE,
};

#ifdef CONFIG_MACH_SUN50I
U_BOOT_DRVINFO(sunxi_lcd) = {
	.name = "sunxi_lcd"
};
#endif
