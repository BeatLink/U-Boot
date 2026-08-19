// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samuel Dionne-Riel
 *
 * Based on the following patch:
 *
 * > Copyright (C) 2020 Ondrej Jirman <megous@megous.com>
 * > https://megous.com/git/u-boot/commit/?h=opi-v2020.01&id=0ab6225154c3d8b74f06fb3b181b52a9a64b4602
 *
 * Loosely based on button-adc.c:
 *
 * > Copyright (C) 2021 Samsung Electronics Co., Ltd.
 * >	http://www.samsung.com
 * > Author: Marek Szyprowski <m.szyprowski@samsung.com>
 */

#include <asm/io.h>
#include <button.h>
#include <log.h>
#include <dm.h>
#include <dm/lists.h>
#include <dm/of_access.h>
#include <dm/uclass-internal.h>
#include <power/regulator.h>

/* Half the 200 mV spacing the buttons' voltage ladder uses. */
#define LRADC_WINDOW_UV		100000

#define LRADC_CTRL		0x00
#define LRADC_INTC		0x04
#define LRADC_INTS		0x08
#define LRADC_DATA0		0x0c
#define LRADC_DATA1		0x10

/* LRADC_CTRL bits */
#define FIRST_CONVERT_DLY(x)	((x) << 24) /* 8 bits */
#define CHAN_SELECT(x)		((x) << 22) /* 2 bits */
#define CONTINUE_TIME_SEL(x)	((x) << 16) /* 4 bits */
#define KEY_MODE_SEL(x)		((x) << 12) /* 2 bits */
#define LEVELA_B_CNT(x)		((x) << 8)  /* 4 bits */
#define HOLD_KEY_EN(x)		((x) << 7)
#define HOLD_EN(x)		((x) << 6)
#define LEVELB_VOL(x)		((x) << 4)  /* 2 bits */
#define SAMPLE_RATE(x)		((x) << 2)  /* 2 bits */
#define ENABLE(x)		((x) << 0)

struct button_sun4i_lradc_priv {
	void __iomem *base;
	int vref;
	int voltage;
	u32 code;
};

static void lradc_enable(void __iomem *base)
{
	writel(0xffffffff, base + LRADC_INTS);
	writel(0, base + LRADC_INTC);

	/* Sample continuously at 250 Hz; the caller's polling debounces. */
	writel(FIRST_CONVERT_DLY(0) | LEVELA_B_CNT(0) | HOLD_EN(0) |
		SAMPLE_RATE(0) | ENABLE(1), base + LRADC_CTRL);
}

static void lradc_disable(void __iomem *base)
{
	writel(0xffffffff, base + LRADC_INTS);
	writel(0, base + LRADC_INTC);
	writel(0, base + LRADC_CTRL);
}

static enum button_state_t button_sun4i_lradc_get_state(struct udevice *dev)
{
	struct button_sun4i_lradc_priv *priv = dev_get_priv(dev);
	int uV;

	uV = (readl(priv->base + LRADC_DATA0) & 0x3f) * priv->vref / 63;

	return abs(uV - priv->voltage) < LRADC_WINDOW_UV ?
		BUTTON_ON : BUTTON_OFF;
}

static int button_sun4i_lradc_of_to_plat(struct udevice *dev)
{
	struct button_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct button_sun4i_lradc_priv *priv = dev_get_priv(dev);
	u32 voltage;
	int ret;

	/* Ignore the top-level button node */
	if (!uc_plat->label)
		return 0;

	ret = ofnode_read_u32(dev_ofnode(dev), "voltage",
			      &voltage);
	if (ret)
		return ret;

	priv->voltage = voltage;

	if (ofnode_read_u32(dev_ofnode(dev), "u-boot,code", &priv->code))
		ofnode_read_u32(dev_ofnode(dev), "linux,code", &priv->code);

	return ret;
}

static int button_sun4i_lradc_probe(struct udevice *dev)
{
	struct button_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct button_sun4i_lradc_priv *priv = dev_get_priv(dev);
	struct udevice *vref;

	if (!uc_plat->label)
		return 0;

	priv->base = dev_read_addr_ptr(dev->parent);
	if (!priv->base)
		return -EINVAL;

	/* The ADC measures against two thirds of the reference rail. */
	priv->vref = 3000000 * 2 / 3;
	if (CONFIG_IS_ENABLED(DM_REGULATOR) &&
	    !device_get_supply_regulator(dev->parent, "vref-supply", &vref)) {
		int uV = regulator_get_value(vref);

		if (uV > 0)
			priv->vref = uV * 2 / 3;
	}

	/* Enabling is idempotent, so each button may do it on its own. */
	lradc_enable(priv->base);

	return 0;
}

static int button_sun4i_lradc_remove(struct udevice *dev)
{
	struct button_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct button_sun4i_lradc_priv *priv = dev_get_priv(dev);

	if (uc_plat->label)
		lradc_disable(priv->base);

	return 0;
}

static int button_sun4i_lradc_bind(struct udevice *parent)
{
	struct udevice *dev;
	ofnode node;
	int ret;

	dev_for_each_subnode(node, parent) {
		struct button_uc_plat *uc_plat;
		const char *label;

		label = ofnode_read_string(node, "label");
		if (!label) {
			debug("%s: node %s has no label\n", __func__,
			      ofnode_get_name(node));
			return -EINVAL;
		}
		ret = device_bind_driver_to_node(parent, "button_sun4i_lradc",
						 ofnode_get_name(node),
						 node, &dev);
		if (ret)
			return ret;
		uc_plat = dev_get_uclass_plat(dev);
		uc_plat->label = label;
	}

	return 0;
}

/* U-Boot's own code wins where it is set, so Linux keeps the volume keys. */
static int button_sun4i_lradc_get_code(struct udevice *dev)
{
	struct button_sun4i_lradc_priv *priv = dev_get_priv(dev);

	if (!priv->code)
		return -ENODATA;

	return priv->code;
}

static const struct button_ops button_sun4i_lradc_ops = {
	.get_state	= button_sun4i_lradc_get_state,
	.get_code	= button_sun4i_lradc_get_code,
};

static const struct udevice_id button_sun4i_lradc_ids[] = {
	{ .compatible = "allwinner,sun8i-a83t-r-lradc" },
	{ }
};

U_BOOT_DRIVER(button_sun4i_lradc) = {
	.name		= "button_sun4i_lradc",
	.id		= UCLASS_BUTTON,
	.of_match	= button_sun4i_lradc_ids,
	.ops		= &button_sun4i_lradc_ops,
	.priv_auto	= sizeof(struct button_sun4i_lradc_priv),
	.bind		= button_sun4i_lradc_bind,
	.of_to_plat	= button_sun4i_lradc_of_to_plat,
	.probe		= button_sun4i_lradc_probe,
	.remove		= button_sun4i_lradc_remove,
	.flags		= DM_FLAG_OS_PREPARE,
};
