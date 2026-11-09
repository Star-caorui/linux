// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6833 MIPI CSI-2 receiver D-PHY
 *
 * One instance drives one CSI port pair: an analog half for each of its A
 * and B ports, and the D-PHY lane controller they share. The port is used
 * whole (4 data lanes and a clock lane over both halves) or as two
 * independent ports of up to 2 data lanes each.
 *
 * The register values and their order are those of MediaTek's camera HAL
 * for this SoC (libcam.halsensor.so, SeninfDrvImp::hal_csirx_*), which
 * match the CSI PHY 2.0 code in MediaTek's MT8188 camera series.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "phy-mtk-io.h"

/* analog half, at 0x0000 (A) and 0x1000 (B) */
#define CSI_ANA_HALF(h)			((h) * 0x1000)
#define CSI_ANA_0			0x00
#define  ANA_0_BG_CORE_EN		BIT(0)
#define  ANA_0_BG_LPF_EN		BIT(1)
#define  ANA_0_CKSEL			GENMASK(10, 8)
#define  ANA_0_CKMODE			GENMASK(14, 12)
#define  ANA_0_CPHY_EN			BIT(20)
#define  ANA_0_BIT21_22			GENMASK(22, 21)
#define CSI_ANA_1			0x04
#define CSI_ANA_2			0x08
#define CSI_ANA_3			0x0c
#define CSI_ANA_4			0x10
#define  ANA_IMP_LO			GENMASK(4, 0)
#define  ANA_IMP_HI			GENMASK(12, 8)
#define CSI_ANA_5			0x14
#define  ANA_5_BW			GENMASK(1, 0)
#define  ANA_5_IS			GENMASK(3, 2)
#define  ANA_5_DG0			BIT(4)
#define  ANA_5_DG1			BIT(5)
#define  ANA_5_LATCH			BIT(6)
#define  ANA_5_SR0			GENMASK(11, 8)
#define  ANA_5_SR1			GENMASK(15, 12)
#define CSI_ANA_6			0x18
#define CSI_ANA_7			0x1c
#define  ANA_67_W0			GENMASK(5, 0)
#define  ANA_67_W1			GENMASK(13, 8)
#define  ANA_67_W2			GENMASK(21, 16)
#define  ANA_67_CK_DELAY		GENMASK(29, 24)
#define CSI_ANA_8			0x20
#define  ANA_8_EN			GENMASK(21, 16)
#define CSI_ANA_9			0x24
#define  ANA_9_RSV			GENMASK(31, 16)
#define CSI_ANA_SETTING_0		0xf0
#define  ANA_SETTING_0_RST_MODE		GENMASK(17, 16)

/* D-PHY lane controller, shared by both halves */
#define CSI_DPHY			0x2000
#define DPHY_LANE_EN			0x00
#define DPHY_LANE_SELECT		0x04
#define DPHY_CLOCK_LANE(n)		(0x10 + 4 * (n))
#define DPHY_DATA_LANE_HS(n)		(0x20 + 4 * (n))
#define  DPHY_HS_PREPARE		GENMASK(7, 0)
#define  DPHY_HS_TRAIL			GENMASK(15, 8)
#define  DPHY_SETTLE			GENMASK(23, 16)
#define  DPHY_HS_TRAIL_EN		BIT(29)

/* the HAL's lane maps: 4D1C across both halves, 2D1C in each half */
#define DPHY_4D1C_LANE_SELECT		0x80403102
#define DPHY_4D1C_LANE_EN		0x0f01
#define DPHY_2D1C_LANE_SELECT		0x80532041
#define DPHY_2D1C_LANE_EN		0x0f03

/* the HAL's D-PHY timing, in receiver clock cycles */
#define DPHY_HS_PREPARE_VAL		2
#define DPHY_HS_TRAIL_VAL		0x17
#define DPHY_SETTLE_VAL			0x20
#define DPHY_TRAIL_EN_MAX_RATE		799999999UL
#define DPHY_EQ_LOW_MAX_RATE		2900000000UL

enum mt6833_csi_port_id {
	MT6833_CSI_PORT_4D1C,	/* both halves */
	MT6833_CSI_PORT_A,	/* A half, up to 2 data lanes */
	MT6833_CSI_PORT_B,	/* B half, up to 2 data lanes */
	MT6833_CSI_NUM_PORTS,
};

struct mt6833_csi_rx;

struct mt6833_csi_port {
	struct mt6833_csi_rx *rx;
	struct phy *phy;
	enum mt6833_csi_port_id id;
	unsigned int lanes;
	unsigned long rate;	/* bits per second and lane */
};

struct mt6833_csi_rx {
	struct device *dev;
	void __iomem *base;
	u32 calib;
	struct mutex lock;	/* halves_on */
	unsigned int halves_on;	/* BIT(0) A, BIT(1) B */
	struct mt6833_csi_port ports[MT6833_CSI_NUM_PORTS];
};

static unsigned int mt6833_csi_port_halves(enum mt6833_csi_port_id id)
{
	switch (id) {
	case MT6833_CSI_PORT_A:
		return BIT(0);
	case MT6833_CSI_PORT_B:
		return BIT(1);
	default:
		return BIT(0) | BIT(1);
	}
}

static void mt6833_csi_half_init(struct mt6833_csi_rx *rx, unsigned int h)
{
	void __iomem *ana = rx->base + CSI_ANA_HALF(h);
	u32 imp[3] = { 0x10, 0x10, 0x10 };
	static const u32 imp_regs[] = { CSI_ANA_2, CSI_ANA_3, CSI_ANA_4 };
	unsigned int i;

	mtk_phy_update_bits(ana + CSI_ANA_1, GENMASK(18, 16) | GENMASK(11, 8) |
			    GENMASK(6, 4) | GENMASK(2, 0),
			    FIELD_PREP(GENMASK(2, 0), 4) | FIELD_PREP(GENMASK(6, 4), 4) |
			    FIELD_PREP(GENMASK(11, 8), 8) | FIELD_PREP(GENMASK(18, 16), 2));
	mtk_phy_update_bits(ana + CSI_ANA_2, GENMASK(30, 28) | GENMASK(26, 24) |
			    BIT(17) | BIT(16),
			    FIELD_PREP(GENMASK(30, 28), 4) | FIELD_PREP(GENMASK(26, 24), 4));

	/*
	 * Termination impedance trim from the efuse: three 5-bit values for
	 * the A half from the top of the word, three for the B half below.
	 */
	if (rx->calib) {
		unsigned int shift = h ? 12 : 27;

		for (i = 0; i < 3; i++)
			imp[i] = (rx->calib >> (shift - 5 * i)) & 0x1f;
	}
	for (i = 0; i < 3; i++)
		mtk_phy_update_bits(ana + imp_regs[i], ANA_IMP_LO | ANA_IMP_HI,
				    FIELD_PREP(ANA_IMP_LO, imp[i]) |
				    FIELD_PREP(ANA_IMP_HI, imp[i]));

	mtk_phy_update_bits(ana + CSI_ANA_5, ANA_5_BW | ANA_5_IS | ANA_5_DG0 |
			    ANA_5_DG1 | ANA_5_LATCH | ANA_5_SR0 | ANA_5_SR1,
			    FIELD_PREP(ANA_5_BW, 3) | FIELD_PREP(ANA_5_IS, 1) |
			    ANA_5_DG0 | ANA_5_DG1 | ANA_5_LATCH);
	mtk_phy_update_field(ana + CSI_ANA_9, ANA_9_RSV, 0x3003);
	mtk_phy_update_field(ana + CSI_ANA_SETTING_0, ANA_SETTING_0_RST_MODE, 2);
	mtk_phy_clear_bits(ana + CSI_ANA_0, ANA_0_BIT21_22);
	for (i = 0; i < 2; i++)
		mtk_phy_update_bits(ana + CSI_ANA_6 + 4 * i, ANA_67_CK_DELAY |
				    ANA_67_W0 | ANA_67_W1 | ANA_67_W2,
				    FIELD_PREP(ANA_67_W0, 9) | FIELD_PREP(ANA_67_W1, 9) |
				    FIELD_PREP(ANA_67_W2, 9));
}

static void mt6833_csi_half_setting(struct mt6833_csi_rx *rx, unsigned int h,
				    bool full, unsigned long rate)
{
	void __iomem *ana = rx->base + CSI_ANA_HALF(h);
	u32 ckmode;

	/* the clock lane: lane 2 of the A half in 4D1C, lane 1 of each half in 2D1C */
	if (full)
		ckmode = h ? 0 : BIT(2);
	else
		ckmode = BIT(1);

	mtk_phy_clear_bits(ana + CSI_ANA_0, ANA_0_CPHY_EN);
	mtk_phy_update_bits(ana + CSI_ANA_0, ANA_0_CKSEL | ANA_0_CKMODE,
			    FIELD_PREP(ANA_0_CKSEL, full ? 0x7 : 0) |
			    FIELD_PREP(ANA_0_CKMODE, ckmode));

	if (rate <= DPHY_EQ_LOW_MAX_RATE)
		mtk_phy_update_bits(ana + CSI_ANA_5, ANA_5_BW | ANA_5_IS | ANA_5_DG0 |
				    ANA_5_DG1 | ANA_5_LATCH | ANA_5_SR0 | ANA_5_SR1,
				    FIELD_PREP(ANA_5_BW, 1) | FIELD_PREP(ANA_5_IS, 1) |
				    ANA_5_LATCH);
	else
		mtk_phy_update_bits(ana + CSI_ANA_5, ANA_5_BW | ANA_5_IS | ANA_5_DG0 |
				    ANA_5_DG1 | ANA_5_LATCH | ANA_5_SR0 | ANA_5_SR1,
				    FIELD_PREP(ANA_5_BW, 3) | FIELD_PREP(ANA_5_IS, 1) |
				    ANA_5_DG0 | ANA_5_DG1 | ANA_5_LATCH |
				    FIELD_PREP(ANA_5_SR0, 1));
}

static void mt6833_csi_half_power_on(struct mt6833_csi_rx *rx, unsigned int h)
{
	void __iomem *ana = rx->base + CSI_ANA_HALF(h);

	mtk_phy_clear_bits(ana + CSI_ANA_8, ANA_8_EN);
	mtk_phy_clear_bits(ana + CSI_ANA_0, ANA_0_BG_LPF_EN);
	mtk_phy_clear_bits(ana + CSI_ANA_0, ANA_0_BG_CORE_EN);
	fsleep(200);
	mtk_phy_set_bits(ana + CSI_ANA_0, ANA_0_BG_CORE_EN);
	fsleep(30);
	mtk_phy_set_bits(ana + CSI_ANA_0, ANA_0_BG_LPF_EN);
	udelay(1);
	mtk_phy_set_bits(ana + CSI_ANA_8, ANA_8_EN);
	udelay(1);
}

static void mt6833_csi_half_power_off(struct mt6833_csi_rx *rx, unsigned int h)
{
	void __iomem *ana = rx->base + CSI_ANA_HALF(h);

	mtk_phy_clear_bits(ana + CSI_ANA_8, ANA_8_EN);
	mtk_phy_clear_bits(ana + CSI_ANA_0, ANA_0_BG_LPF_EN | ANA_0_BG_CORE_EN);
}

static void mt6833_csi_dphy_setting(struct mt6833_csi_rx *rx, bool full,
				    unsigned long rate)
{
	void __iomem *dphy = rx->base + CSI_DPHY;
	u32 hs;
	unsigned int i;

	hs = FIELD_PREP(DPHY_HS_PREPARE, DPHY_HS_PREPARE_VAL) |
	     FIELD_PREP(DPHY_HS_TRAIL, DPHY_HS_TRAIL_VAL) |
	     FIELD_PREP(DPHY_SETTLE, DPHY_SETTLE_VAL);
	if (rate <= DPHY_TRAIL_EN_MAX_RATE)
		hs |= DPHY_HS_TRAIL_EN;

	for (i = 0; i < 4; i++)
		mtk_phy_update_bits(dphy + DPHY_DATA_LANE_HS(i),
				    DPHY_HS_PREPARE | DPHY_HS_TRAIL | DPHY_SETTLE |
				    DPHY_HS_TRAIL_EN, hs);
	for (i = 0; i < 2; i++)
		mtk_phy_clear_bits(dphy + DPHY_CLOCK_LANE(i), DPHY_SETTLE);

	writel(full ? DPHY_4D1C_LANE_SELECT : DPHY_2D1C_LANE_SELECT,
	       dphy + DPHY_LANE_SELECT);
	writel(full ? DPHY_4D1C_LANE_EN : DPHY_2D1C_LANE_EN,
	       dphy + DPHY_LANE_EN);
}

static int mt6833_csi_power_on(struct phy *phy)
{
	struct mt6833_csi_port *port = phy_get_drvdata(phy);
	struct mt6833_csi_rx *rx = port->rx;
	unsigned int halves = mt6833_csi_port_halves(port->id);
	bool full = port->id == MT6833_CSI_PORT_4D1C;
	unsigned int h;
	int ret = 0;

	if (!port->rate) {
		dev_err(rx->dev, "port %u: no lane rate configured\n", port->id);
		return -EINVAL;
	}

	mutex_lock(&rx->lock);
	if (rx->halves_on & halves) {
		ret = -EBUSY;
		goto out;
	}

	for (h = 0; h < 2; h++)
		if (halves & BIT(h))
			mt6833_csi_half_init(rx, h);
	mt6833_csi_dphy_setting(rx, full, port->rate);
	for (h = 0; h < 2; h++) {
		if (!(halves & BIT(h)))
			continue;
		mt6833_csi_half_setting(rx, h, full, port->rate);
		mt6833_csi_half_power_on(rx, h);
	}
	rx->halves_on |= halves;

	dev_dbg(rx->dev, "port %u on: %u lanes at %lu bps\n", port->id,
		port->lanes, port->rate);
out:
	mutex_unlock(&rx->lock);
	return ret;
}

static int mt6833_csi_power_off(struct phy *phy)
{
	struct mt6833_csi_port *port = phy_get_drvdata(phy);
	struct mt6833_csi_rx *rx = port->rx;
	unsigned int halves = mt6833_csi_port_halves(port->id);
	unsigned int h;

	mutex_lock(&rx->lock);
	for (h = 0; h < 2; h++)
		if (halves & rx->halves_on & BIT(h))
			mt6833_csi_half_power_off(rx, h);
	rx->halves_on &= ~halves;
	mutex_unlock(&rx->lock);

	return 0;
}

static int mt6833_csi_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct mt6833_csi_port *port = phy_get_drvdata(phy);
	unsigned int max_lanes = port->id == MT6833_CSI_PORT_4D1C ? 4 : 2;
	const struct phy_configure_opts_mipi_dphy *cfg = &opts->mipi_dphy;

	if (!cfg->lanes || cfg->lanes > max_lanes || !cfg->hs_clk_rate)
		return -EINVAL;

	port->lanes = cfg->lanes;
	port->rate = cfg->hs_clk_rate;

	return 0;
}

static const struct phy_ops mt6833_csi_phy_ops = {
	.power_on = mt6833_csi_power_on,
	.power_off = mt6833_csi_power_off,
	.configure = mt6833_csi_configure,
	.owner = THIS_MODULE,
};

static struct phy *mt6833_csi_xlate(struct device *dev,
				    const struct of_phandle_args *args)
{
	struct mt6833_csi_rx *rx = dev_get_drvdata(dev);

	if (args->args_count != 1 || args->args[0] >= MT6833_CSI_NUM_PORTS)
		return ERR_PTR(-EINVAL);

	return rx->ports[args->args[0]].phy;
}

static int mt6833_csi_read_calib(struct mt6833_csi_rx *rx)
{
	struct nvmem_cell *cell;
	void *buf;
	size_t len;

	cell = devm_nvmem_cell_get(rx->dev, "calibration");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -ENOENT)
			return 0;
		return dev_err_probe(rx->dev, PTR_ERR(cell),
				     "failed to get the calibration cell\n");
	}

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf))
		return dev_err_probe(rx->dev, PTR_ERR(buf),
				     "failed to read the calibration cell\n");

	if (len == sizeof(u32))
		rx->calib = get_unaligned_le32(buf);
	kfree(buf);

	dev_dbg(rx->dev, "impedance calibration 0x%08x\n", rx->calib);

	return 0;
}

static int mt6833_csi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct mt6833_csi_rx *rx;
	unsigned int i;
	int ret;

	rx = devm_kzalloc(dev, sizeof(*rx), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	rx->dev = dev;
	dev_set_drvdata(dev, rx);

	ret = devm_mutex_init(dev, &rx->lock);
	if (ret)
		return ret;

	rx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rx->base))
		return PTR_ERR(rx->base);

	ret = mt6833_csi_read_calib(rx);
	if (ret)
		return ret;

	for (i = 0; i < MT6833_CSI_NUM_PORTS; i++) {
		struct mt6833_csi_port *port = &rx->ports[i];

		port->rx = rx;
		port->id = i;
		port->phy = devm_phy_create(dev, NULL, &mt6833_csi_phy_ops);
		if (IS_ERR(port->phy))
			return dev_err_probe(dev, PTR_ERR(port->phy),
					     "failed to create PHY %u\n", i);
		phy_set_drvdata(port->phy, port);
	}

	provider = devm_of_phy_provider_register(dev, mt6833_csi_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(dev, PTR_ERR(provider),
				     "failed to register the PHY provider\n");

	return 0;
}

static const struct of_device_id mt6833_csi_of_match[] = {
	{ .compatible = "mediatek,mt6833-csi-rx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6833_csi_of_match);

static struct platform_driver mt6833_csi_driver = {
	.probe = mt6833_csi_probe,
	.driver = {
		.name = "mtk-mipi-csi-mt6833",
		.of_match_table = mt6833_csi_of_match,
	},
};
module_platform_driver(mt6833_csi_driver);

MODULE_DESCRIPTION("MediaTek MT6833 MIPI CSI-2 receiver D-PHY driver");
MODULE_LICENSE("GPL");
