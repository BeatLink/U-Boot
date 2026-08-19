// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun6i/sun50i MIPI-DSI host driver for U-Boot
 *
 * Ported from Linux drivers/gpu/drm/sun4i/sun6i_mipi_dsi.c and
 * drivers/phy/allwinner/phy-sun6i-mipi-dphy.c.
 */

#include <clk.h>
#include <dm.h>
#include <dm/device.h>
#include <dsi_host.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <mipi_dsi.h>
#include <reset.h>
#include <asm/io.h>
#include <power/regulator.h>
#include <asm/arch/clock.h>
#include <dm/device_compat.h>
#include <u-boot/crc.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#include "sunxi_mipi_dsi.h"
#include "sunxi_tcon.h"

#define SUN6I_DPHY_GCTL_REG		0x00
#define SUN6I_DPHY_GCTL_LANE_NUM(n)	((((n) - 1) & 3) << 4)
#define SUN6I_DPHY_GCTL_EN		BIT(0)

#define SUN6I_DPHY_TX_CTL_REG		0x04
#define SUN6I_DPHY_TX_CTL_HS_TX_CLK_CONT BIT(28)

#define SUN6I_DPHY_TX_TIME0_REG	0x10
#define SUN6I_DPHY_TX_TIME0_HS_TRAIL(n)	(((n) & 0xff) << 24)
#define SUN6I_DPHY_TX_TIME0_HS_PREPARE(n)	(((n) & 0xff) << 16)
#define SUN6I_DPHY_TX_TIME0_LP_CLK_DIV(n)	((n) & 0xff)

#define SUN6I_DPHY_TX_TIME1_REG		0x14
#define SUN6I_DPHY_TX_TIME1_CLK_POST(n)	(((n) & 0xff) << 24)
#define SUN6I_DPHY_TX_TIME1_CLK_PRE(n)		(((n) & 0xff) << 16)
#define SUN6I_DPHY_TX_TIME1_CLK_ZERO(n)		(((n) & 0xff) << 8)
#define SUN6I_DPHY_TX_TIME1_CLK_PREPARE(n)	((n) & 0xff)

#define SUN6I_DPHY_TX_TIME2_REG		0x18
#define SUN6I_DPHY_TX_TIME2_CLK_TRAIL(n)	((n) & 0xff)

#define SUN6I_DPHY_TX_TIME3_REG		0x1c

#define SUN6I_DPHY_TX_TIME4_REG		0x20
#define SUN6I_DPHY_TX_TIME4_HS_TX_ANA1(n)	(((n) & 0xff) << 8)
#define SUN6I_DPHY_TX_TIME4_HS_TX_ANA0(n)	((n) & 0xff)

#define SUN6I_DPHY_ANA0_REG		0x4c
#define SUN6I_DPHY_ANA0_REG_PWS		BIT(31)
#define SUN6I_DPHY_ANA0_REG_DMPC		BIT(28)
#define SUN6I_DPHY_ANA0_REG_DMPD(n)		(((n) & 0xf) << 24)
#define SUN6I_DPHY_ANA0_REG_SLV(n)		(((n) & 7) << 12)
#define SUN6I_DPHY_ANA0_REG_DEN(n)		(((n) & 0xf) << 8)
#define SUN6I_DPHY_ANA0_REG_PLR(n)		(((n) & 0xf) << 4)
#define SUN6I_DPHY_ANA0_REG_SFB(n)		(((n) & 3) << 2)

#define SUN6I_DPHY_ANA1_REG		0x50
#define SUN6I_DPHY_ANA1_REG_VTTMODE		BIT(31)
#define SUN6I_DPHY_ANA1_REG_CSMPS(n)		(((n) & 3) << 28)
#define SUN6I_DPHY_ANA1_REG_SVTT(n)		(((n) & 0xf) << 24)

#define SUN6I_DPHY_ANA2_REG		0x54
#define SUN6I_DPHY_ANA2_EN_P2S_CPU(n)		(((n) & 0xf) << 24)
#define SUN6I_DPHY_ANA2_EN_P2S_CPU_MASK		GENMASK(27, 24)
#define SUN6I_DPHY_ANA2_EN_CK_CPU		BIT(4)
#define SUN6I_DPHY_ANA2_REG_ENIB		BIT(1)

#define SUN6I_DPHY_ANA3_REG		0x58
#define SUN6I_DPHY_ANA3_EN_VTTD(n)		(((n) & 0xf) << 28)
#define SUN6I_DPHY_ANA3_EN_VTTD_MASK		GENMASK(31, 28)
#define SUN6I_DPHY_ANA3_EN_VTTC			BIT(27)
#define SUN6I_DPHY_ANA3_EN_DIV			BIT(26)
#define SUN6I_DPHY_ANA3_EN_LDOC			BIT(25)
#define SUN6I_DPHY_ANA3_EN_LDOD			BIT(24)
#define SUN6I_DPHY_ANA3_EN_LDOR			BIT(18)

#define SUN6I_DPHY_ANA4_REG		0x5c
#define SUN6I_DPHY_ANA4_REG_EN_MIPI		BIT(31)
#define SUN6I_DPHY_ANA4_REG_IB(n)		(((n) & 3) << 25)
#define SUN6I_DPHY_ANA4_REG_DMPLVC		BIT(24)
#define SUN6I_DPHY_ANA4_REG_DMPLVD(n)		(((n) & 0xf) << 20)
#define SUN6I_DPHY_ANA4_REG_VTT_SET(n)		(((n) & 0x7) << 17)
#define SUN6I_DPHY_ANA4_REG_CKDV(n)		(((n) & 0x1f) << 12)
#define SUN6I_DPHY_ANA4_REG_TMSC(n)		(((n) & 3) << 10)
#define SUN6I_DPHY_ANA4_REG_TMSD(n)		(((n) & 3) << 8)
#define SUN6I_DPHY_ANA4_REG_TXDNSC(n)		(((n) & 3) << 6)
#define SUN6I_DPHY_ANA4_REG_TXDNSD(n)		(((n) & 3) << 4)
#define SUN6I_DPHY_ANA4_REG_TXPUSC(n)		(((n) & 3) << 2)
#define SUN6I_DPHY_ANA4_REG_TXPUSD(n)		((n) & 3)

#define SUN50I_DPHY_PLL_REG0		0x104
#define SUN50I_DPHY_PLL_REG0_LOCK		BIT(0)
#define SUN50I_DPHY_PLL_REG0_CP36_EN		BIT(23)
#define SUN50I_DPHY_PLL_REG0_LDO_EN		BIT(22)
#define SUN50I_DPHY_PLL_REG0_EN_LVS		BIT(21)
#define SUN50I_DPHY_PLL_REG0_PLL_EN		BIT(20)
#define SUN50I_DPHY_PLL_REG0_P(n)		(((n) & 0xf) << 16)
#define SUN50I_DPHY_PLL_REG0_N(n)		(((n) & 0xff) << 8)
#define SUN50I_DPHY_PLL_REG0_NDET		BIT(7)
#define SUN50I_DPHY_PLL_REG0_TDIV		BIT(6)
#define SUN50I_DPHY_PLL_REG0_M0(n)		(((n) & 3) << 4)
#define SUN50I_DPHY_PLL_REG0_M1(n)		((n) & 0xf)

#define SUN50I_DPHY_PLL_REG2		0x10c
#define SUN50I_DPHY_PLL_REG2_SDM_EN		BIT(31)

#define SUN50I_COMBO_PHY_REG0		0x110
#define SUN50I_COMBO_PHY_REG0_EN_CP		BIT(0)
#define SUN50I_COMBO_PHY_REG0_EN_COMBOLDO	BIT(1)
#define SUN50I_COMBO_PHY_REG0_EN_MIPI		BIT(3)

#define SUN50I_COMBO_PHY_REG2		0x118
#define SUN50I_COMBO_PHY_REG2_HS_STOP_DLY(n)	((n) & 0xff)

struct sunxi_dsi;

struct sunxi_dphy_variant {
	void (*tx_power_on)(struct sunxi_dsi *dsi);
};

struct sunxi_dsi {
	struct udevice *dev;
	void __iomem *regs;
	void __iomem *dphy_regs;
	struct mipi_dsi_host host;
	struct mipi_dsi_device *device;
	struct display_timing timings;
	struct clk mod_clk;
	struct clk bus_clk;
	struct reset_ctl reset;
	struct clk dphy_mod_clk;
	struct clk dphy_bus_clk;
	struct reset_ctl dphy_reset;
	bool mod_clk_valid;
	bool reset_valid;
	bool dphy_reset_valid;
	const struct sunxi_dphy_variant *dphy_variant;
	struct udevice *vcc_dsi;
	bool video_configured;
	bool powered;
	bool video_started;
	unsigned long dphy_rate;
};

static inline struct sunxi_dsi *host_to_sunxi_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct sunxi_dsi, host);
}

static void dsi_write(struct sunxi_dsi *dsi, u32 reg, u32 val)
{
	writel(val, dsi->regs + reg);
}

static u32 dsi_read(struct sunxi_dsi *dsi, u32 reg)
{
	return readl(dsi->regs + reg);
}

static void dsi_update_bits(struct sunxi_dsi *dsi, u32 reg, u32 mask, u32 val)
{
	u32 tmp = dsi_read(dsi, reg);

	tmp = (tmp & ~mask) | (val & mask);
	dsi_write(dsi, reg, tmp);
}

static void dphy_write(struct sunxi_dsi *dsi, u32 reg, u32 val)
{
	writel(val, dsi->dphy_regs + reg);
}

static void dphy_update_bits(struct sunxi_dsi *dsi, u32 reg, u32 mask, u32 val)
{
	u32 tmp = readl(dsi->dphy_regs + reg);

	tmp = (tmp & ~mask) | (val & mask);
	writel(tmp, dsi->dphy_regs + reg);
}

static const u32 sun6i_dsi_ecc_array[] = {
	[0] = (BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(7) | BIT(10) |
	       BIT(11) | BIT(13) | BIT(16) | BIT(20) | BIT(21) | BIT(22) |
	       BIT(23)),
	[1] = (BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(6) | BIT(8) | BIT(10) |
	       BIT(12) | BIT(14) | BIT(17) | BIT(20) | BIT(21) | BIT(22) |
	       BIT(23)),
	[2] = (BIT(0) | BIT(2) | BIT(3) | BIT(5) | BIT(6) | BIT(9) | BIT(11) |
	       BIT(12) | BIT(15) | BIT(18) | BIT(20) | BIT(21) | BIT(22)),
	[3] = (BIT(1) | BIT(2) | BIT(3) | BIT(7) | BIT(8) | BIT(9) | BIT(13) |
	       BIT(14) | BIT(15) | BIT(19) | BIT(20) | BIT(21) | BIT(23)),
	[4] = (BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(9) | BIT(16) |
	       BIT(17) | BIT(18) | BIT(19) | BIT(20) | BIT(22) | BIT(23)),
	[5] = (BIT(10) | BIT(11) | BIT(12) | BIT(13) | BIT(14) | BIT(15) |
	       BIT(16) | BIT(17) | BIT(18) | BIT(19) | BIT(21) | BIT(22) |
	       BIT(23)),
};

static u32 sun6i_dsi_ecc_compute(u32 data)
{
	u8 ecc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(sun6i_dsi_ecc_array); i++) {
		u32 field = sun6i_dsi_ecc_array[i];
		bool init = false;
		u8 val = 0;
		int j;

		for (j = 0; j < 24; j++) {
			if (!(BIT(j) & field))
				continue;
			if (!init) {
				val = (BIT(j) & data) ? 1 : 0;
				init = true;
			} else {
				val ^= (BIT(j) & data) ? 1 : 0;
			}
		}
		ecc |= val << i;
	}
	return ecc;
}

static u16 sun6i_dsi_crc_compute(const u8 *buffer, size_t len)
{
	return crc16_ccitt(0xffff, buffer, len);
}

static u16 sun6i_dsi_crc_repeat(u8 pd, size_t len)
{
	u16 crc = 0xffff;

	while (len--)
		crc = crc16_ccitt(crc, &pd, 1);

	return crc;
}

static u32 sun6i_dsi_build_sync_pkt(u8 dt, u8 vc, u8 d0, u8 d1)
{
	u32 val = dt & 0x3f;

	val |= (vc & 3) << 6;
	val |= (d0 & 0xff) << 8;
	val |= (d1 & 0xff) << 16;
	val |= sun6i_dsi_ecc_compute(val) << 24;

	return val;
}

static u32 sun6i_dsi_build_blk0_pkt(u8 vc, u16 wc)
{
	return sun6i_dsi_build_sync_pkt(MIPI_DSI_BLANKING_PACKET, vc,
					wc & 0xff, wc >> 8);
}

static u32 sun6i_dsi_build_blk1_pkt(u8 pd, size_t len)
{
	u32 val = SUN6I_DSI_BLK_PD(pd);

	return val | SUN6I_DSI_BLK_PF(sun6i_dsi_crc_repeat(pd, len));
}

static void sun6i_dsi_inst_abort(struct sunxi_dsi *dsi)
{
	dsi_update_bits(dsi, SUN6I_DSI_BASIC_CTL0_REG,
			SUN6I_DSI_BASIC_CTL0_INST_ST, 0);
}

static void sun6i_dsi_inst_commit(struct sunxi_dsi *dsi)
{
	dsi_update_bits(dsi, SUN6I_DSI_BASIC_CTL0_REG,
			SUN6I_DSI_BASIC_CTL0_INST_ST,
			SUN6I_DSI_BASIC_CTL0_INST_ST);
}

static int sun6i_dsi_inst_wait_for_completion(struct sunxi_dsi *dsi)
{
	u32 val;

	return readl_poll_timeout(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG, val,
				 !(val & SUN6I_DSI_BASIC_CTL0_INST_ST), 5000);
}

static int sun6i_dsi_start(struct sunxi_dsi *dsi,
			   enum sun6i_dsi_start_inst func);

static void sun6i_dsi_inst_setup(struct sunxi_dsi *dsi,
				 enum sun6i_dsi_inst_id id,
				 enum sun6i_dsi_inst_mode mode,
				 bool clock, u8 data,
				 enum sun6i_dsi_inst_packet packet,
				 enum sun6i_dsi_inst_escape escape)
{
	dsi_write(dsi, SUN6I_DSI_INST_FUNC_REG(id),
		  SUN6I_DSI_INST_FUNC_INST_MODE(mode) |
		  SUN6I_DSI_INST_FUNC_ESCAPE_ENTRY(escape) |
		  SUN6I_DSI_INST_FUNC_TRANS_PACKET(packet) |
		  (clock ? SUN6I_DSI_INST_FUNC_LANE_CEN : 0) |
		  SUN6I_DSI_INST_FUNC_LANE_DEN(data));
}

static void sun6i_dsi_inst_init(struct sunxi_dsi *dsi,
				struct mipi_dsi_device *device)
{
	u8 lanes_mask = GENMASK(device->lanes - 1, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_LP11, DSI_INST_MODE_STOP,
			     true, lanes_mask, 0, 0);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_TBA, DSI_INST_MODE_TBA,
			     false, 1, 0, 0);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSC, DSI_INST_MODE_HS,
			     true, 0, DSI_INST_PACK_PIXEL, 0);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSD, DSI_INST_MODE_HS,
			     false, lanes_mask, DSI_INST_PACK_PIXEL, 0);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_LPDT, DSI_INST_MODE_ESCAPE,
			     false, 1, DSI_INST_PACK_COMMAND,
			     DSI_INST_ESCA_LPDT);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSCEXIT, DSI_INST_MODE_HSCEXIT,
			     true, 0, 0, 0);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_NOP, DSI_INST_MODE_STOP,
			     false, lanes_mask, 0, 0);
	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_DLY, DSI_INST_MODE_NOP,
			     true, lanes_mask, 0, 0);

	dsi_write(dsi, SUN6I_DSI_INST_JUMP_CFG_REG(0),
		  SUN6I_DSI_INST_JUMP_CFG_POINT(DSI_INST_ID_NOP) |
		  SUN6I_DSI_INST_JUMP_CFG_TO(DSI_INST_ID_HSCEXIT) |
		  SUN6I_DSI_INST_JUMP_CFG_NUM(1));
}

static int sun6i_dsi_dcs_build_pkt_hdr(struct sunxi_dsi *dsi,
				       const struct mipi_dsi_msg *msg)
{
	u32 pkt = msg->type;

	if (msg->type == MIPI_DSI_DCS_LONG_WRITE) {
		pkt |= ((msg->tx_len) & 0xffff) << 8;
		pkt |= (((msg->tx_len) >> 8) & 0xffff) << 16;
	} else {
		pkt |= (((u8 *)msg->tx_buf)[0] << 8);
		if (msg->tx_len > 1)
			pkt |= (((u8 *)msg->tx_buf)[1] << 16);
	}

	pkt |= sun6i_dsi_ecc_compute(pkt) << 24;
	return pkt;
}

static int sun6i_dsi_dcs_write_short(struct sunxi_dsi *dsi,
				     const struct mipi_dsi_msg *msg)
{
	int ret;

	dsi_write(dsi, SUN6I_DSI_CMD_TX_REG(0),
		  sun6i_dsi_dcs_build_pkt_hdr(dsi, msg));
	dsi_update_bits(dsi, SUN6I_DSI_CMD_CTL_REG, 0xff, (4 - 1));

	sun6i_dsi_start(dsi, DSI_START_LPTX);

	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		sun6i_dsi_inst_abort(dsi);
		return ret;
	}

	return msg->tx_len;
}

static int sun6i_dsi_dcs_write_long(struct sunxi_dsi *dsi,
				    const struct mipi_dsi_msg *msg)
{
	/* Zeroed like the kernel's allocation, so the tail of the last word is not stack noise. */
	u32 bounce_words[ALIGN(255 + 2, 4) / 4] = { };
	u8 *bounce = (u8 *)bounce_words;
	u16 crc;
	int len = 0;
	int ret;

	/* The four header bytes go to their own register, not this buffer. */
	if (msg->tx_len + sizeof(crc) > sizeof(bounce_words))
		return -EINVAL;

	dsi_write(dsi, SUN6I_DSI_CMD_TX_REG(0),
		  sun6i_dsi_dcs_build_pkt_hdr(dsi, msg));

	memcpy(bounce, msg->tx_buf, msg->tx_len);
	len += msg->tx_len;

	crc = sun6i_dsi_crc_compute(bounce, msg->tx_len);
	memcpy(bounce + msg->tx_len, &crc, sizeof(crc));
	len += sizeof(crc);

	{
		int i;

		for (i = 0; i < DIV_ROUND_UP(len, 4); i++)
			dsi_write(dsi, SUN6I_DSI_CMD_TX_REG(1 + i),
				  bounce_words[i]);
	}

	dsi_write(dsi, SUN6I_DSI_CMD_CTL_REG, len + 4 - 1);

	sun6i_dsi_start(dsi, DSI_START_LPTX);

	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		sun6i_dsi_inst_abort(dsi);
		return ret;
	}

	return msg->tx_len;
}

static ssize_t sun6i_dsi_transfer(struct mipi_dsi_host *host,
				  const struct mipi_dsi_msg *msg)
{
	struct sunxi_dsi *dsi = host_to_sunxi_dsi(host);
	int ret;

	/* Wait for any previous instruction to finish, then clear flags. */
	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		sun6i_dsi_inst_abort(dsi);
		return ret;
	}

	dsi_write(dsi, SUN6I_DSI_CMD_CTL_REG,
		  SUN6I_DSI_CMD_CTL_RX_OVERFLOW |
		  SUN6I_DSI_CMD_CTL_RX_FLAG |
		  SUN6I_DSI_CMD_CTL_TX_FLAG);

	switch (msg->type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		return sun6i_dsi_dcs_write_short(dsi, msg);
	case MIPI_DSI_DCS_LONG_WRITE:
		return sun6i_dsi_dcs_write_long(dsi, msg);
	default:
		return -EOPNOTSUPP;
	}
}

static int sun6i_dsi_attach(struct mipi_dsi_host *host,
			    struct mipi_dsi_device *device)
{
	struct sunxi_dsi *dsi = host_to_sunxi_dsi(host);

	dsi->device = device;
	dev_info(dsi->dev, "Attached DSI device %s\n", device->name);

	return 0;
}

static int sun6i_dsi_detach(struct mipi_dsi_host *host,
			    struct mipi_dsi_device *device)
{
	struct sunxi_dsi *dsi = host_to_sunxi_dsi(host);

	dsi->device = NULL;
	return 0;
}

static const struct mipi_dsi_host_ops sunxi_mipi_dsi_host_ops = {
	.attach = sun6i_dsi_attach,
	.detach = sun6i_dsi_detach,
	.transfer = sun6i_dsi_transfer,
};

static u16 sun6i_dsi_get_video_start_delay(struct sunxi_dsi *dsi,
					  struct display_timing *timings)
{
	u16 vtotal = timings->vactive.typ + timings->vback_porch.typ +
		     timings->vfront_porch.typ + timings->vsync_len.typ;
	u16 delay = vtotal - timings->vfront_porch.typ + 1;

	if (delay > vtotal)
		delay = delay % vtotal;

	return (delay >= 1) ? delay : 1;
}

static u16 sun6i_dsi_get_line_num(struct sunxi_dsi *dsi,
				  struct display_timing *timings)
{
	struct mipi_dsi_device *device = dsi->device;
	unsigned int Bpp = mipi_dsi_pixel_format_to_bpp(device->format) / 8;
	u16 htotal = timings->hactive.typ + timings->hfront_porch.typ +
		     timings->hsync_len.typ + timings->hback_porch.typ;

	return htotal * Bpp / device->lanes;
}

static u16 sun6i_dsi_get_drq_edge1(struct sunxi_dsi *dsi,
				   struct display_timing *timings,
				   u16 line_num)
{
	struct mipi_dsi_device *device = dsi->device;
	unsigned int Bpp = mipi_dsi_pixel_format_to_bpp(device->format) / 8;
	unsigned int hbp = timings->hback_porch.typ;
	u16 edge1;

	edge1 = SUN6I_DSI_SYNC_POINT;
	edge1 += (timings->hactive.typ + hbp + 20) * Bpp / device->lanes;

	if (edge1 > line_num)
		return line_num;

	return edge1;
}

static u16 sun6i_dsi_get_drq_edge0(struct sunxi_dsi *dsi,
				   struct display_timing *timings,
				   u16 line_num, u16 edge1)
{
	u16 edge0 = edge1;

	edge0 += (timings->hactive.typ + 40) * SUN6I_DSI_TCON_DIV / 8;

	if (edge0 > line_num)
		return edge0 - line_num;

	return 1;
}

static void sun6i_dsi_setup_burst(struct sunxi_dsi *dsi,
				  struct display_timing *timings)
{
	struct mipi_dsi_device *device = dsi->device;
	u32 val = 0;

	if (device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		u16 line_num = sun6i_dsi_get_line_num(dsi, timings);
		u16 edge0, edge1;

		edge1 = sun6i_dsi_get_drq_edge1(dsi, timings, line_num);
		edge0 = sun6i_dsi_get_drq_edge0(dsi, timings, line_num, edge1);
		dsi_write(dsi, SUN6I_DSI_BURST_DRQ_REG,
			  SUN6I_DSI_BURST_DRQ_EDGE0(edge0) |
			  SUN6I_DSI_BURST_DRQ_EDGE1(edge1));
		dsi_write(dsi, SUN6I_DSI_BURST_LINE_REG,
			  SUN6I_DSI_BURST_LINE_NUM(line_num) |
			  SUN6I_DSI_BURST_LINE_SYNC_POINT(SUN6I_DSI_SYNC_POINT));
		val = SUN6I_DSI_TCON_DRQ_ENABLE_MODE;
	} else if (timings->hfront_porch.typ > 20) {
		u16 drq = (timings->hfront_porch.typ - 20);
		drq *= mipi_dsi_pixel_format_to_bpp(device->format);
		drq /= 32;
		val = SUN6I_DSI_TCON_DRQ_ENABLE_MODE |
		      SUN6I_DSI_TCON_DRQ_SET(drq);
	}
	dsi_write(dsi, SUN6I_DSI_TCON_DRQ_REG, val);
}

static void sun6i_dsi_setup_inst_loop(struct sunxi_dsi *dsi,
				      struct display_timing *timings)
{
	u16 delay = 50 - 1;

	dsi_write(dsi, SUN6I_DSI_INST_LOOP_SEL_REG,
		  2 << (4 * DSI_INST_ID_LP11) |
		  3 << (4 * DSI_INST_ID_DLY));
	dsi_write(dsi, SUN6I_DSI_INST_LOOP_NUM_REG(0),
		  SUN6I_DSI_INST_LOOP_NUM_N0(50 - 1) |
		  SUN6I_DSI_INST_LOOP_NUM_N1(delay));
	dsi_write(dsi, SUN6I_DSI_INST_LOOP_NUM_REG(1),
		  SUN6I_DSI_INST_LOOP_NUM_N0(50 - 1) |
		  SUN6I_DSI_INST_LOOP_NUM_N1(delay));
}

static void sun6i_dsi_setup_format(struct sunxi_dsi *dsi,
				   struct display_timing *timings)
{
	struct mipi_dsi_device *device = dsi->device;
	u32 val = SUN6I_DSI_PIXEL_PH_VC(device->channel);
	u8 dt, fmt;
	u16 wc;

	switch (device->format) {
	case MIPI_DSI_FMT_RGB888:
		dt = MIPI_DSI_PACKED_PIXEL_STREAM_24;
		fmt = 8;
		break;
	default:
		dev_err(dsi->dev, "Unsupported pixel format %d\n", device->format);
		return;
	}
	val |= SUN6I_DSI_PIXEL_PH_DT(dt);

	wc = timings->hactive.typ * mipi_dsi_pixel_format_to_bpp(device->format) / 8;
	val |= SUN6I_DSI_PIXEL_PH_WC(wc);
	val |= SUN6I_DSI_PIXEL_PH_ECC(sun6i_dsi_ecc_compute(val));

	dsi_write(dsi, SUN6I_DSI_PIXEL_PH_REG, val);
	dsi_write(dsi, SUN6I_DSI_PIXEL_PF0_REG,
		  SUN6I_DSI_PIXEL_PF0_CRC_FORCE(0xffff));
	dsi_write(dsi, SUN6I_DSI_PIXEL_PF1_REG,
		  SUN6I_DSI_PIXEL_PF1_CRC_INIT_LINE0(0xffff) |
		  SUN6I_DSI_PIXEL_PF1_CRC_INIT_LINEN(0xffff));
	dsi_write(dsi, SUN6I_DSI_PIXEL_CTL0_REG,
		  SUN6I_DSI_PIXEL_CTL0_PD_PLUG_DISABLE |
		  SUN6I_DSI_PIXEL_CTL0_FORMAT(fmt));
}

static void sun6i_dsi_setup_timings(struct sunxi_dsi *dsi,
				    struct display_timing *timings)
{
	struct mipi_dsi_device *device = dsi->device;
	int Bpp = mipi_dsi_pixel_format_to_bpp(device->format) / 8;
	u16 hbp = 0, hfp = 0, hsa = 0, hblk = 0, vblk = 0;
	u32 basic_ctl = 0;

	if (device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		hblk = timings->hactive.typ * Bpp;
		basic_ctl = SUN6I_DSI_BASIC_CTL_VIDEO_BURST |
			    SUN6I_DSI_BASIC_CTL_HSA_HSE_DIS |
			    SUN6I_DSI_BASIC_CTL_HBP_DIS;
		if (device->lanes == 4)
			basic_ctl |= SUN6I_DSI_BASIC_CTL_TRAIL_FILL |
				     SUN6I_DSI_BASIC_CTL_TRAIL_INV(0xc);
	} else {
		hsa = max(10, (int)(timings->hsync_len.typ * Bpp - 10));
		hbp = max(6, (int)(timings->hback_porch.typ * Bpp - 6));
		hfp = max(16, (int)(timings->hfront_porch.typ * Bpp - 16));
		hblk = max(10, (int)((timings->hactive.typ + timings->hback_porch.typ +
				    timings->hfront_porch.typ) * Bpp - 10));
		vblk = 0;
	}

	dsi_write(dsi, SUN6I_DSI_BASIC_CTL_REG, basic_ctl);

	dsi_write(dsi, SUN6I_DSI_SYNC_HSS_REG,
		  sun6i_dsi_build_sync_pkt(MIPI_DSI_H_SYNC_START,
					   device->channel, 0, 0));
	dsi_write(dsi, SUN6I_DSI_SYNC_HSE_REG,
		  sun6i_dsi_build_sync_pkt(MIPI_DSI_H_SYNC_END,
					   device->channel, 0, 0));
	dsi_write(dsi, SUN6I_DSI_SYNC_VSS_REG,
		  sun6i_dsi_build_sync_pkt(MIPI_DSI_V_SYNC_START,
					   device->channel, 0, 0));
	dsi_write(dsi, SUN6I_DSI_SYNC_VSE_REG,
		  sun6i_dsi_build_sync_pkt(MIPI_DSI_V_SYNC_END,
					   device->channel, 0, 0));

	dsi_write(dsi, SUN6I_DSI_BASIC_SIZE0_REG,
		  SUN6I_DSI_BASIC_SIZE0_VSA(timings->vsync_len.typ) |
		  SUN6I_DSI_BASIC_SIZE0_VBP(timings->vback_porch.typ));
	dsi_write(dsi, SUN6I_DSI_BASIC_SIZE1_REG,
		  SUN6I_DSI_BASIC_SIZE1_VACT(timings->vactive.typ) |
		  SUN6I_DSI_BASIC_SIZE1_VT(timings->vactive.typ +
					   timings->vback_porch.typ +
					   timings->vfront_porch.typ +
					   timings->vsync_len.typ));

	dsi_write(dsi, SUN6I_DSI_BLK_HSA0_REG,
		  sun6i_dsi_build_blk0_pkt(device->channel, hsa));
	dsi_write(dsi, SUN6I_DSI_BLK_HSA1_REG,
		  sun6i_dsi_build_blk1_pkt(0, hsa));
	dsi_write(dsi, SUN6I_DSI_BLK_HBP0_REG,
		  sun6i_dsi_build_blk0_pkt(device->channel, hbp));
	dsi_write(dsi, SUN6I_DSI_BLK_HBP1_REG,
		  sun6i_dsi_build_blk1_pkt(0, hbp));
	dsi_write(dsi, SUN6I_DSI_BLK_HFP0_REG,
		  sun6i_dsi_build_blk0_pkt(device->channel, hfp));
	dsi_write(dsi, SUN6I_DSI_BLK_HFP1_REG,
		  sun6i_dsi_build_blk1_pkt(0, hfp));
	dsi_write(dsi, SUN6I_DSI_BLK_HBLK0_REG,
		  sun6i_dsi_build_blk0_pkt(device->channel, hblk));
	dsi_write(dsi, SUN6I_DSI_BLK_HBLK1_REG,
		  sun6i_dsi_build_blk1_pkt(0, hblk));
	dsi_write(dsi, SUN6I_DSI_BLK_VBLK0_REG,
		  sun6i_dsi_build_blk0_pkt(device->channel, vblk));
	dsi_write(dsi, SUN6I_DSI_BLK_VBLK1_REG,
		  sun6i_dsi_build_blk1_pkt(0, vblk));
}

static int sun6i_dsi_start(struct sunxi_dsi *dsi,
			   enum sun6i_dsi_start_inst func)
{
	switch (func) {
	case DSI_START_LPTX:
		dsi_write(dsi, SUN6I_DSI_INST_JUMP_SEL_REG,
			  DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11) |
			  DSI_INST_ID_END  << (4 * DSI_INST_ID_LPDT));
		break;
	case DSI_START_LPRX:
		dsi_write(dsi, SUN6I_DSI_INST_JUMP_SEL_REG,
			  DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11) |
			  DSI_INST_ID_DLY  << (4 * DSI_INST_ID_LPDT) |
			  DSI_INST_ID_TBA  << (4 * DSI_INST_ID_DLY) |
			  DSI_INST_ID_END  << (4 * DSI_INST_ID_TBA));
		break;
	case DSI_START_HSC:
		dsi_write(dsi, SUN6I_DSI_INST_JUMP_SEL_REG,
			  DSI_INST_ID_HSC  << (4 * DSI_INST_ID_LP11) |
			  DSI_INST_ID_END  << (4 * DSI_INST_ID_HSC));
		break;
	case DSI_START_HSD:
		dsi_write(dsi, SUN6I_DSI_INST_JUMP_SEL_REG,
			  DSI_INST_ID_NOP  << (4 * DSI_INST_ID_LP11) |
			  DSI_INST_ID_HSD  << (4 * DSI_INST_ID_NOP) |
			  DSI_INST_ID_DLY  << (4 * DSI_INST_ID_HSD) |
			  DSI_INST_ID_NOP  << (4 * DSI_INST_ID_DLY) |
			  DSI_INST_ID_END  << (4 * DSI_INST_ID_HSCEXIT));
		break;
	default:
		dsi_write(dsi, SUN6I_DSI_INST_JUMP_SEL_REG,
			  DSI_INST_ID_END  << (4 * DSI_INST_ID_LP11));
		break;
	}

	sun6i_dsi_inst_abort(dsi);
	sun6i_dsi_inst_commit(dsi);

	if (func == DSI_START_HSC)
		dsi_update_bits(dsi, SUN6I_DSI_INST_FUNC_REG(DSI_INST_ID_LP11),
				SUN6I_DSI_INST_FUNC_LANE_CEN, 0);

	return 0;
}

static void sun6i_a31_dphy_power_on(struct sunxi_dsi *dsi)
{
	u8 lanes = dsi->device ? dsi->device->lanes : 1;
	u8 lanes_mask = GENMASK(lanes - 1, 0);

	dphy_write(dsi, SUN6I_DPHY_ANA0_REG,
		   SUN6I_DPHY_ANA0_REG_PWS |
		   SUN6I_DPHY_ANA0_REG_DMPC |
		   SUN6I_DPHY_ANA0_REG_SLV(7) |
		   SUN6I_DPHY_ANA0_REG_DMPD(lanes_mask) |
		   SUN6I_DPHY_ANA0_REG_DEN(lanes_mask));
	dphy_write(dsi, SUN6I_DPHY_ANA1_REG,
		   SUN6I_DPHY_ANA1_REG_CSMPS(1) |
		   SUN6I_DPHY_ANA1_REG_SVTT(7));
	dphy_write(dsi, SUN6I_DPHY_ANA4_REG,
		   SUN6I_DPHY_ANA4_REG_CKDV(1) |
		   SUN6I_DPHY_ANA4_REG_TMSC(1) |
		   SUN6I_DPHY_ANA4_REG_TMSD(1) |
		   SUN6I_DPHY_ANA4_REG_TXDNSC(1) |
		   SUN6I_DPHY_ANA4_REG_TXDNSD(1) |
		   SUN6I_DPHY_ANA4_REG_TXPUSC(1) |
		   SUN6I_DPHY_ANA4_REG_TXPUSD(1) |
		   SUN6I_DPHY_ANA4_REG_DMPLVC |
		   SUN6I_DPHY_ANA4_REG_DMPLVD(lanes_mask));
	dphy_write(dsi, SUN6I_DPHY_ANA2_REG, SUN6I_DPHY_ANA2_REG_ENIB);
	udelay(5);
	dphy_write(dsi, SUN6I_DPHY_ANA3_REG,
		   SUN6I_DPHY_ANA3_EN_LDOR |
		   SUN6I_DPHY_ANA3_EN_LDOC |
		   SUN6I_DPHY_ANA3_EN_LDOD);
}

static unsigned int sunxi_order_base_2(unsigned int x)
{
	unsigned int r = 0;
	unsigned int y = 1;

	while (y < x) {
		y <<= 1;
		r++;
	}
	return r;
}

static void sun50i_a100_dphy_power_on(struct sunxi_dsi *dsi)
{
	unsigned long mipi_symbol_rate = dsi->dphy_rate;
	unsigned int div, n;
	unsigned int val;
	int ret;

	dphy_write(dsi, SUN6I_DPHY_ANA4_REG,
		   SUN6I_DPHY_ANA4_REG_IB(2) |
		   SUN6I_DPHY_ANA4_REG_DMPLVD(4) |
		   SUN6I_DPHY_ANA4_REG_VTT_SET(3) |
		   SUN6I_DPHY_ANA4_REG_CKDV(3) |
		   SUN6I_DPHY_ANA4_REG_TMSD(1) |
		   SUN6I_DPHY_ANA4_REG_TMSC(1) |
		   SUN6I_DPHY_ANA4_REG_TXPUSD(2) |
		   SUN6I_DPHY_ANA4_REG_TXPUSC(3) |
		   SUN6I_DPHY_ANA4_REG_TXDNSD(2) |
		   SUN6I_DPHY_ANA4_REG_TXDNSC(3));

	dphy_update_bits(dsi, SUN6I_DPHY_ANA2_REG,
			 SUN6I_DPHY_ANA2_EN_CK_CPU,
			 SUN6I_DPHY_ANA2_EN_CK_CPU);
	dphy_update_bits(dsi, SUN6I_DPHY_ANA2_REG,
			 SUN6I_DPHY_ANA2_REG_ENIB,
			 SUN6I_DPHY_ANA2_REG_ENIB);
	dphy_write(dsi, SUN6I_DPHY_ANA3_REG,
		   SUN6I_DPHY_ANA3_EN_LDOR |
		   SUN6I_DPHY_ANA3_EN_LDOC |
		   SUN6I_DPHY_ANA3_EN_LDOD);
	dphy_write(dsi, SUN6I_DPHY_ANA0_REG,
		   SUN6I_DPHY_ANA0_REG_PLR(4) |
		   SUN6I_DPHY_ANA0_REG_SFB(1));
	dphy_write(dsi, SUN50I_COMBO_PHY_REG0, SUN50I_COMBO_PHY_REG0_EN_CP);

	div = 16 >> sunxi_order_base_2(DIV_ROUND_UP(mipi_symbol_rate, 264000000));
	n = mipi_symbol_rate * div / 24000000;

	dphy_write(dsi, SUN50I_DPHY_PLL_REG0,
		   SUN50I_DPHY_PLL_REG0_CP36_EN |
		   SUN50I_DPHY_PLL_REG0_LDO_EN |
		   SUN50I_DPHY_PLL_REG0_EN_LVS |
		   SUN50I_DPHY_PLL_REG0_PLL_EN |
		   SUN50I_DPHY_PLL_REG0_NDET |
		   SUN50I_DPHY_PLL_REG0_P((div - 1) % 8) |
		   SUN50I_DPHY_PLL_REG0_N(n) |
		   SUN50I_DPHY_PLL_REG0_M0((div - 1) / 8) |
		   SUN50I_DPHY_PLL_REG0_M1(2));
	dphy_write(dsi, SUN50I_DPHY_PLL_REG2, 0);

	ret = readl_poll_timeout(dsi->dphy_regs + SUN50I_DPHY_PLL_REG0,
				  val, val & SUN50I_DPHY_PLL_REG0_LOCK,
				  10000);
	if (ret)
		dev_warn(dsi->dev, "D-PHY PLL lock timeout: %d\n", ret);

	dphy_update_bits(dsi, SUN6I_DPHY_ANA4_REG,
			 SUN6I_DPHY_ANA4_REG_EN_MIPI,
			 SUN6I_DPHY_ANA4_REG_EN_MIPI);
	dphy_update_bits(dsi, SUN50I_COMBO_PHY_REG0,
			 SUN50I_COMBO_PHY_REG0_EN_MIPI |
			 SUN50I_COMBO_PHY_REG0_EN_COMBOLDO,
			 SUN50I_COMBO_PHY_REG0_EN_MIPI |
			 SUN50I_COMBO_PHY_REG0_EN_COMBOLDO);
	dphy_write(dsi, SUN50I_COMBO_PHY_REG2,
		   SUN50I_COMBO_PHY_REG2_HS_STOP_DLY(20));
	udelay(1);
}

static const struct sunxi_dphy_variant sunxi_a31_dphy = {
	.tx_power_on = sun6i_a31_dphy_power_on,
};

static const struct sunxi_dphy_variant sunxi_a100_dphy = {
	.tx_power_on = sun50i_a100_dphy_power_on,
};

static void sunxi_dphy_tx_power_on(struct sunxi_dsi *dsi)
{
	u8 lanes = dsi->device ? dsi->device->lanes : 1;
	u8 lanes_mask = GENMASK(lanes - 1, 0);

	dphy_write(dsi, SUN6I_DPHY_TX_CTL_REG,
		   SUN6I_DPHY_TX_CTL_HS_TX_CLK_CONT);
	dphy_write(dsi, SUN6I_DPHY_TX_TIME0_REG,
		   SUN6I_DPHY_TX_TIME0_LP_CLK_DIV(14) |
		   SUN6I_DPHY_TX_TIME0_HS_PREPARE(6) |
		   SUN6I_DPHY_TX_TIME0_HS_TRAIL(10));
	dphy_write(dsi, SUN6I_DPHY_TX_TIME1_REG,
		   SUN6I_DPHY_TX_TIME1_CLK_PREPARE(7) |
		   SUN6I_DPHY_TX_TIME1_CLK_ZERO(50) |
		   SUN6I_DPHY_TX_TIME1_CLK_PRE(3) |
		   SUN6I_DPHY_TX_TIME1_CLK_POST(10));
	dphy_write(dsi, SUN6I_DPHY_TX_TIME2_REG,
		   SUN6I_DPHY_TX_TIME2_CLK_TRAIL(30));
	dphy_write(dsi, SUN6I_DPHY_TX_TIME3_REG, 0);
	dphy_write(dsi, SUN6I_DPHY_TX_TIME4_REG,
		   SUN6I_DPHY_TX_TIME4_HS_TX_ANA0(3) |
		   SUN6I_DPHY_TX_TIME4_HS_TX_ANA1(3));

	dsi->dphy_variant->tx_power_on(dsi);

	dphy_update_bits(dsi, SUN6I_DPHY_ANA3_REG,
			 SUN6I_DPHY_ANA3_EN_VTTC |
			 SUN6I_DPHY_ANA3_EN_VTTD_MASK,
			 SUN6I_DPHY_ANA3_EN_VTTC |
			 SUN6I_DPHY_ANA3_EN_VTTD(lanes_mask));
	udelay(1);
	dphy_update_bits(dsi, SUN6I_DPHY_ANA3_REG,
			 SUN6I_DPHY_ANA3_EN_DIV, SUN6I_DPHY_ANA3_EN_DIV);
	udelay(1);
	dphy_update_bits(dsi, SUN6I_DPHY_ANA2_REG,
			 SUN6I_DPHY_ANA2_EN_CK_CPU,
			 SUN6I_DPHY_ANA2_EN_CK_CPU);
	udelay(1);
	dphy_update_bits(dsi, SUN6I_DPHY_ANA1_REG,
			 SUN6I_DPHY_ANA1_REG_VTTMODE,
			 SUN6I_DPHY_ANA1_REG_VTTMODE);
	dphy_update_bits(dsi, SUN6I_DPHY_ANA2_REG,
			 SUN6I_DPHY_ANA2_EN_P2S_CPU_MASK,
			 SUN6I_DPHY_ANA2_EN_P2S_CPU(lanes_mask));
	dphy_write(dsi, SUN6I_DPHY_GCTL_REG,
		   SUN6I_DPHY_GCTL_LANE_NUM(lanes) |
		   SUN6I_DPHY_GCTL_EN);
}

/* Point the D-PHY module clock at PLL_VIDEO0 and divide it down to the lane bit rate. */
static void sunxi_dsi_set_dphy_rate(struct sunxi_dsi *dsi, unsigned long rate)
{
#ifdef CONFIG_MACH_SUN50I
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	unsigned int parent = clock_get_pll3();
	unsigned int div = rate ? parent / rate : 1;

	/* The CCU driver only gates this clock, so its mux and divider are set here. */
	if (div < 1)
		div = 1;
	if (div > 16)
		div = 16;

	clrsetbits_le32(&ccm->mipi_dsi_clk_cfg,
			CCM_DSI_DPHY_SRC_MASK | CCM_DSI_DPHY_DIV_MASK,
			CCM_DSI_DPHY_SRC_PLL_VIDEO0 | CCM_DSI_DPHY_DIV(div));

	if (rate && parent % rate)
		dev_dbg(dsi->dev, "dphy rate %lu is not an exact divisor of %u\n",
			rate, parent);
#endif
}

static int sunxi_dsi_video_setup(struct sunxi_dsi *dsi)
{
	struct display_timing *timings = &dsi->timings;
	u16 delay;
	int ret;
	u8 lanes = dsi->device ? dsi->device->lanes : 1;
	u8 format = dsi->device ?
		mipi_dsi_pixel_format_to_bpp(dsi->device->format) : 24;
	unsigned long dphy_rate;

	if (!dsi->device)
		return -ENODEV;

	/* D-PHY rate: bits per second per lane */
	dphy_rate = timings->pixelclock.typ * format / lanes;
	dsi->dphy_rate = dphy_rate;

	if (dsi->mod_clk_valid) {
		ret = clk_set_rate(&dsi->mod_clk, dphy_rate);
		if (ret < 0)
			dev_err(dsi->dev, "failed to set mod_clk rate: %d\n", ret);
	}

	sunxi_dsi_set_dphy_rate(dsi, dphy_rate);

	/* Reset and enable the DSI block; D-PHY reset is already released in host init. */
	if (dsi->reset_valid) {
		reset_assert(&dsi->reset);
		udelay(10);
		reset_deassert(&dsi->reset);
	}
	if (dsi->dphy_reset_valid)
		reset_deassert(&dsi->dphy_reset);

	dsi_write(dsi, SUN6I_DSI_CTL_REG, SUN6I_DSI_CTL_EN);

	dphy_write(dsi, SUN6I_DPHY_GCTL_REG,
		   SUN6I_DPHY_GCTL_LANE_NUM(lanes) | SUN6I_DPHY_GCTL_EN);

	dsi_write(dsi, SUN6I_DSI_BASIC_CTL0_REG,
		  SUN6I_DSI_BASIC_CTL0_ECC_EN | SUN6I_DSI_BASIC_CTL0_CRC_EN);
	dsi_write(dsi, SUN6I_DSI_TRANS_START_REG, 10);
	dsi_write(dsi, SUN6I_DSI_TRANS_ZERO_REG, 0);

	sun6i_dsi_inst_init(dsi, dsi->device);
	dsi_write(dsi, SUN6I_DSI_DEBUG_DATA_REG, 0xff);

	delay = sun6i_dsi_get_video_start_delay(dsi, timings);
	dsi_write(dsi, SUN6I_DSI_BASIC_CTL1_REG,
		  SUN6I_DSI_BASIC_CTL1_VIDEO_ST_DELAY(delay) |
		  SUN6I_DSI_BASIC_CTL1_VIDEO_FILL |
		  SUN6I_DSI_BASIC_CTL1_VIDEO_PRECISION |
		  SUN6I_DSI_BASIC_CTL1_VIDEO_MODE);

	sun6i_dsi_setup_burst(dsi, timings);
	sun6i_dsi_setup_inst_loop(dsi, timings);
	sun6i_dsi_setup_format(dsi, timings);
	sun6i_dsi_setup_timings(dsi, timings);

	sunxi_dphy_tx_power_on(dsi);

	dsi->video_configured = true;
	return 0;
}

static int sunxi_dsi_video_start(struct sunxi_dsi *dsi)
{
	if (dsi->video_started)
		return 0;

	if (!dsi->video_configured) {
		int ret = sunxi_dsi_video_setup(dsi);
		if (ret)
			return ret;
	}

	sun6i_dsi_start(dsi, DSI_START_HSC);
	udelay(1000);
	sun6i_dsi_start(dsi, DSI_START_HSD);

	dsi->video_started = true;
	return 0;
}

static int sunxi_dsi_video_stop(struct sunxi_dsi *dsi)
{
	if (!dsi->video_started)
		return 0;

	sun6i_dsi_inst_abort(dsi);
	dsi_write(dsi, SUN6I_DSI_CTL_REG, 0);
	dsi->video_started = false;
	dsi->video_configured = false;
	return 0;
}

static int sunxi_dsi_host_init(struct udevice *dev,
			       struct mipi_dsi_device *device,
			       struct display_timing *timings,
			       unsigned int max_data_lanes,
			       const struct mipi_dsi_phy_ops *phy_ops)
{
	struct sunxi_dsi *dsi = dev_get_priv(dev);
	int ret;

	if (device->lanes > max_data_lanes)
		return -EINVAL;

	ret = clk_enable(&dsi->bus_clk);
	if (ret)
		return ret;

	if (dsi->mod_clk_valid) {
		ret = clk_enable(&dsi->mod_clk);
		if (ret)
			goto err_bus_clk;
	}

	ret = clk_enable(&dsi->dphy_bus_clk);
	if (ret)
		goto err_mod_clk;

	ret = clk_enable(&dsi->dphy_mod_clk);
	if (ret)
		goto err_dphy_bus_clk;

	/*
	 * The DPHY module clock on this CCU path is gate-only. Treat it as an
	 * enable-only clock and leave its parent rate alone.
	 */

	if (dsi->reset_valid) {
		ret = reset_deassert(&dsi->reset);
		if (ret)
			goto err_dphy_mod_clk;
	}

	if (dsi->dphy_reset_valid) {
		ret = reset_deassert(&dsi->dphy_reset);
		if (ret)
			goto err_reset;
	}

	/* Allow clocks to stabilize before the DSI block exits reset. */
	udelay(10);

	dsi->device = device;
	memcpy(&dsi->timings, timings, sizeof(*timings));
	device->host = &dsi->host;
	dsi->powered = true;

	return 0;

err_reset:
	if (dsi->reset_valid)
		reset_assert(&dsi->reset);
err_dphy_mod_clk:
	clk_disable(&dsi->dphy_mod_clk);
err_dphy_bus_clk:
	clk_disable(&dsi->dphy_bus_clk);
err_mod_clk:
	if (dsi->mod_clk_valid)
		clk_disable(&dsi->mod_clk);
err_bus_clk:
	clk_disable(&dsi->bus_clk);
	return ret;
}

/* Shared by disable and remove, so being called by both stays harmless. */
static void sunxi_dsi_power_off(struct sunxi_dsi *dsi)
{
	if (!dsi->powered)
		return;
	dsi->powered = false;

	if (dsi->mod_clk_valid)
		clk_disable(&dsi->mod_clk);
	clk_disable(&dsi->bus_clk);
	clk_disable(&dsi->dphy_mod_clk);
	clk_disable(&dsi->dphy_bus_clk);
	if (dsi->reset_valid)
		reset_assert(&dsi->reset);
	if (dsi->dphy_reset_valid)
		reset_assert(&dsi->dphy_reset);
}

static int sunxi_dsi_host_enable(struct udevice *dev)
{
	struct sunxi_dsi *dsi = dev_get_priv(dev);

	/*
	 * First call (from video uclass before panel init) sets up the DSI
	 * engine and D-PHY in command mode so the panel can send DCS commands.
	 * The panel driver will call this again after init to start HS video.
	 */
	if (!dsi->video_configured)
		return sunxi_dsi_video_setup(dsi);

	return sunxi_dsi_video_start(dsi);
}

static int sunxi_dsi_host_disable(struct udevice *dev)
{
	struct sunxi_dsi *dsi = dev_get_priv(dev);

	sunxi_dsi_video_stop(dsi);
	sunxi_dsi_power_off(dsi);

	return 0;
}

static int sunxi_dsi_probe(struct udevice *dev)
{
	struct sunxi_dsi *dsi = dev_get_priv(dev);
	struct ofnode_phandle_args phandle;
	fdt_addr_t addr;
	int ret;

	dsi->dev = dev;
	dsi->host.dev = (struct device *)dev;
	dsi->host.ops = &sunxi_mipi_dsi_host_ops;

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE) {
		dev_err(dev, "dsi no addr\n");
		return -EINVAL;
	}
	dsi->regs = (void __iomem *)addr;

	ret = dev_read_phandle_with_args(dev, "phys", "#phy-cells", 0, 0,
					 &phandle);
	if (ret) {
		return ret;
	}

	/* Not ofnode_get_addr_size(), which assumes as many cells as the address type is wide. */
	addr = ofnode_get_addr(phandle.node);
	if (addr == FDT_ADDR_T_NONE) {
		dev_err(dev, "dphy node no reg\n");
		return -EINVAL;
	}
	dsi->dphy_regs = (void __iomem *)addr;

	/* The A64 binding names no clocks and carries the bus clock alone. */
	ret = clk_get_by_name(dev, "bus", &dsi->bus_clk);
	if (ret)
		ret = clk_get_by_index(dev, 0, &dsi->bus_clk);
	if (ret) {
		dev_err(dev, "Cannot get bus clock: %d\n", ret);
		return ret;
	}

	ret = clk_get_by_name(dev, "mod", &dsi->mod_clk);
	if (ret) {
		dsi->mod_clk_valid = false;
	} else {
		dsi->mod_clk_valid = true;
	}

	ret = reset_get_by_index(dev, 0, &dsi->reset);
	if (ret) {
		dev_dbg(dev, "dsi host reset idx0 not found: %d, continuing\n", ret);
		dsi->reset_valid = false;
	} else {
		dsi->reset_valid = true;
		ret = reset_deassert(&dsi->reset);
		if (ret)
			dev_dbg(dev, "dsi host reset deassert failed: %d, continuing\n", ret);
	}

	ret = clk_get_by_name_nodev(phandle.node, "bus", &dsi->dphy_bus_clk);
	if (ret) {
		dev_err(dev, "Cannot get dphy bus clock: %d\n", ret);
		return ret;
	}

	ret = clk_get_by_name_nodev(phandle.node, "mod", &dsi->dphy_mod_clk);
	if (ret) {
		dev_err(dev, "Cannot get dphy mod clock: %d\n", ret);
		return ret;
	}

	ret = reset_get_by_index_nodev(phandle.node, 0, &dsi->dphy_reset);
	if (ret) {
		dev_dbg(dev, "dsi dphy reset idx0 not found: %d, continuing\n", ret);
		dsi->dphy_reset_valid = false;
	} else {
		dsi->dphy_reset_valid = true;
	}

	/* The panel sees nothing at all without this rail, and says nothing about it. */
	ret = device_get_supply_regulator(dev, "vcc-dsi-supply", &dsi->vcc_dsi);
	if (ret) {
		dev_dbg(dev, "no vcc-dsi supply: %d\n", ret);
		dsi->vcc_dsi = NULL;
	} else {
		ret = regulator_set_enable(dsi->vcc_dsi, true);
		if (ret)
			dev_err(dev, "Cannot enable vcc-dsi: %d\n", ret);
	}

	dsi->dphy_variant = (const struct sunxi_dphy_variant *)dev_get_driver_data(dev);
	if (!dsi->dphy_variant) {
		dev_err(dev, "unknown compatible, defaulting to a31 dphy\n");
		dsi->dphy_variant = &sunxi_a31_dphy;
	}

	return 0;
}

struct mipi_dsi_host *sunxi_mipi_dsi_host(struct udevice *dev)
{
	struct sunxi_dsi *dsi = dev_get_priv(dev);

	return &dsi->host;
}

static int sunxi_dsi_remove(struct udevice *dev)
{
	struct sunxi_dsi *dsi = dev_get_priv(dev);

	sunxi_dsi_power_off(dsi);

	return 0;
}

static const struct dsi_host_ops sunxi_dsi_host_dev_ops = {
	.init = sunxi_dsi_host_init,
	.enable = sunxi_dsi_host_enable,
	.disable = sunxi_dsi_host_disable,
};

static const struct udevice_id sunxi_dsi_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-mipi-dsi",
	  .data = (ulong)&sunxi_a31_dphy },
	{ .compatible = "allwinner,sun50i-a64-mipi-dsi",
	  .data = (ulong)&sunxi_a31_dphy },
	{ .compatible = "allwinner,sun50i-a100-mipi-dsi",
	  .data = (ulong)&sunxi_a100_dphy },
	{ .compatible = "allwinner,sun20i-d1-mipi-dsi",
	  .data = (ulong)&sunxi_a100_dphy },
	{ .compatible = "allwinner,sun8i-r528-mipi-dsi",
	  .data = (ulong)&sunxi_a31_dphy },
	{ }
};

U_BOOT_DRIVER(sunxi_mipi_dsi) = {
	.name		= "sunxi-mipi-dsi",
	.id		= UCLASS_DSI_HOST,
	.of_match	= sunxi_dsi_ids,
	.ops		= &sunxi_dsi_host_dev_ops,
	.probe		= sunxi_dsi_probe,
	.remove		= sunxi_dsi_remove,
	.priv_auto	= sizeof(struct sunxi_dsi),
};
