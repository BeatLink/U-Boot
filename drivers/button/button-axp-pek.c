// SPDX-License-Identifier: GPL-2.0+
/*
 * Power key of an X-Powers AXP PMIC, exposed as a button.
 *
 * The PMIC latches a short press in its interrupt status register rather than
 * reporting the pin level, so a press is reported once and then cleared.
 *
 * Register layout follows Linux drivers/mfd/axp20x.c and is specific to the
 * AXP803/AXP813 family, which the compatible reflects.
 */

#include <button.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <log.h>
#include <power/pmic.h>

#define AXP_IRQ5_ENABLE		0x44
#define AXP_IRQ5_STATUS		0x4c
#define AXP_IRQ5_PEK_SHORT	BIT(4)
#define AXP_IRQ5_PEK_FALL	BIT(5)

struct button_axp_pek_priv {
	u32 code;
};

static enum button_state_t button_axp_pek_get_state(struct udevice *dev)
{
	int status;

	status = pmic_reg_read(dev->parent, AXP_IRQ5_STATUS);
	if (status < 0)
		return BUTTON_OFF;

	if (!(status & (AXP_IRQ5_PEK_SHORT | AXP_IRQ5_PEK_FALL)))
		return BUTTON_OFF;

	/* Write-one-to-clear; a press latched between read and write is lost. */
	pmic_reg_write(dev->parent, AXP_IRQ5_STATUS,
		       status & (AXP_IRQ5_PEK_SHORT | AXP_IRQ5_PEK_FALL));

	return BUTTON_ON;
}

static int button_axp_pek_get_code(struct udevice *dev)
{
	struct button_axp_pek_priv *priv = dev_get_priv(dev);

	if (!priv->code)
		return -ENODATA;

	return priv->code;
}

static int button_axp_pek_of_to_plat(struct udevice *dev)
{
	struct button_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct button_axp_pek_priv *priv = dev_get_priv(dev);

	uc_plat->label = ofnode_read_string(dev_ofnode(dev), "label");

	if (ofnode_read_u32(dev_ofnode(dev), "u-boot,code", &priv->code))
		ofnode_read_u32(dev_ofnode(dev), "linux,code", &priv->code);

	return 0;
}

static int button_axp_pek_probe(struct udevice *dev)
{
	int ret;

	/* The status bits only latch while the matching interrupt is enabled. */
	ret = pmic_clrsetbits(dev->parent, AXP_IRQ5_ENABLE, 0,
			      AXP_IRQ5_PEK_SHORT | AXP_IRQ5_PEK_FALL);
	if (ret < 0)
		return ret;

	return pmic_reg_write(dev->parent, AXP_IRQ5_STATUS,
			      AXP_IRQ5_PEK_SHORT | AXP_IRQ5_PEK_FALL);
}

static const struct button_ops button_axp_pek_ops = {
	.get_state	= button_axp_pek_get_state,
	.get_code	= button_axp_pek_get_code,
};

static const struct udevice_id button_axp_pek_ids[] = {
	{ .compatible = "x-powers,axp803-pek" },
	{ }
};

U_BOOT_DRIVER(button_axp_pek) = {
	.name		= "button_axp_pek",
	.id		= UCLASS_BUTTON,
	.of_match	= button_axp_pek_ids,
	.ops		= &button_axp_pek_ops,
	.probe		= button_axp_pek_probe,
	.of_to_plat	= button_axp_pek_of_to_plat,
	.priv_auto	= sizeof(struct button_axp_pek_priv),
};
