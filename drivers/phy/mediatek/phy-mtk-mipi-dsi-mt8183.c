// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: jitao.shi <jitao.shi@mediatek.com>
 */

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_LANE_CON		0x000c
#define RG_DSI_CPHY_T1DRV_EN		BIT(0)
#define RG_DSI_ANA_CK_SEL		BIT(1)
#define RG_DSI_PHY_CK_SEL		BIT(2)
#define RG_DSI_CPHY_EN			BIT(3)
#define RG_DSI_PHYCK_INV_EN		BIT(4)
#define RG_DSI_PWR04_EN			BIT(5)
#define RG_DSI_BG_LPF_EN		BIT(6)
#define RG_DSI_BG_CORE_EN		BIT(7)
#define RG_DSI_PAD_TIEL_SEL		BIT(8)

#define MIPITX_VOLTAGE_SEL	0x0010
#define RG_DSI_HSTX_LDO_REF_SEL		GENMASK(9, 6)

#define MIPITX_PLL_PWR		0x0028
#define MIPITX_PLL_CON0		0x002c
#define MIPITX_PLL_CON1		0x0030
#define MIPITX_PLL_CON2		0x0034
#define MIPITX_PLL_CON3		0x0038
#define MIPITX_PLL_CON4		0x003c
#define RG_DSI_PLL_IBIAS		GENMASK(11, 10)

#define MIPITX_D2P_RTCODE	0x0100
#define MIPITX_D2_SW_CTL_EN	0x0144
#define MIPITX_D0_SW_CTL_EN	0x0244
#define MIPITX_CK_CKMODE_EN	0x0328
#define DSI_CK_CKMODE_EN		BIT(0)
#define MIPITX_CK_SW_CTL_EN	0x0344
#define MIPITX_D1_SW_CTL_EN	0x0444
#define MIPITX_D3_SW_CTL_EN	0x0544
#define DSI_SW_CTL_EN			BIT(0)
#define AD_DSI_PLL_SDM_PWR_ON		BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN		BIT(1)

#define RG_DSI_PLL_EN			BIT(4)
#define RG_DSI_PLL_POSDIV		GENMASK(10, 8)

static int mtk_mipi_tx_pll_enable(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	unsigned int txdiv, txdiv0;
	u64 pcw;

	dev_dbg(mipi_tx->dev, "enable: %u bps\n", mipi_tx->data_rate);

	if (mipi_tx->data_rate >= 2000000000) {
		txdiv = 1;
		txdiv0 = 0;
	} else if (mipi_tx->data_rate >= 1000000000) {
		txdiv = 2;
		txdiv0 = 1;
	} else if (mipi_tx->data_rate >= 500000000) {
		txdiv = 4;
		txdiv0 = 2;
	} else if (mipi_tx->data_rate > 250000000) {
		txdiv = 8;
		txdiv0 = 3;
	} else if (mipi_tx->data_rate >= 125000000) {
		txdiv = 16;
		txdiv0 = 4;
	} else {
		return -EINVAL;
	}

	mtk_phy_clear_bits(base + MIPITX_PLL_CON4, RG_DSI_PLL_IBIAS);

	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	udelay(1);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	pcw = div_u64(((u64)mipi_tx->data_rate * txdiv) << 24, 26000000);
	writel(pcw, base + MIPITX_PLL_CON0);
	mtk_phy_update_field(base + MIPITX_PLL_CON1, RG_DSI_PLL_POSDIV, txdiv0);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);

	return 0;
}

static void mtk_mipi_tx_pll_disable(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);

	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
}

static int mtk_mipi_tx_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);

	req->rate = clamp_val(req->rate, 125000000,
			      mipi_tx->driver_data->max_data_rate);

	return 0;
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.enable = mtk_mipi_tx_pll_enable,
	.disable = mtk_mipi_tx_pll_disable,
	.determine_rate = mtk_mipi_tx_pll_determine_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

static void mtk_mipi_tx_config_calibration_data(struct mtk_mipi_tx *mipi_tx)
{
	int i, j;

	for (i = 0; i < 5; i++) {
		if ((mipi_tx->rt_code[i] & 0x1f) == 0)
			mipi_tx->rt_code[i] |= 0x10;

		if ((mipi_tx->rt_code[i] >> 5 & 0x1f) == 0)
			mipi_tx->rt_code[i] |= 0x10 << 5;

		for (j = 0; j < 10; j++)
			mtk_phy_update_bits(mipi_tx->regs +
				MIPITX_D2P_RTCODE * (i + 1) + j * 4,
				1, mipi_tx->rt_code[i] >> j & 1);
	}
}

/*
 * The impedance codes the bootloader left in the RT code registers, in the
 * layout mtk_mipi_tx_config_calibration_data() writes: MediaTek's kernel
 * keeps them this way on SoCs without the codes in an nvmem cell
 * (backup_mipitx_impedance(), refill_mipitx_impedance()).
 */
static void mtk_mipi_tx_read_boot_calibration(struct mtk_mipi_tx *mipi_tx)
{
	int i, j;

	for (i = 0; i < 5; i++)
		for (j = 0; j < 10; j++)
			mipi_tx->rt_code[i] |= (readl(mipi_tx->regs +
				MIPITX_D2P_RTCODE * (i + 1) + j * 4) & 1) << j;

	dev_dbg(mipi_tx->dev, "impedance from the bootloader: %*ph\n",
		(int)sizeof(mipi_tx->rt_code), mipi_tx->rt_code);
}

static void mtk_mipi_tx_power_on_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);
	void __iomem *base = mipi_tx->regs;
	u32 lpf = mipi_tx->driver_data->bg_lpf_off ? 0 : RG_DSI_BG_LPF_EN;

	/* BG_LPF_EN / BG_CORE_EN */
	writel(RG_DSI_PAD_TIEL_SEL | RG_DSI_BG_CORE_EN, base + MIPITX_LANE_CON);
	usleep_range(30, 100);
	writel(RG_DSI_BG_CORE_EN | lpf, base + MIPITX_LANE_CON);

	/* Switch OFF each Lane */
	mtk_phy_clear_bits(base + MIPITX_D0_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_clear_bits(base + MIPITX_D1_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_clear_bits(base + MIPITX_D2_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_clear_bits(base + MIPITX_D3_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_clear_bits(base + MIPITX_CK_SW_CTL_EN, DSI_SW_CTL_EN);

	mtk_phy_update_field(base + MIPITX_VOLTAGE_SEL, RG_DSI_HSTX_LDO_REF_SEL,
			     (mipi_tx->mipitx_drive - 3000) / 200);

	mtk_mipi_tx_config_calibration_data(mipi_tx);

	mtk_phy_set_bits(base + MIPITX_CK_CKMODE_EN, DSI_CK_CKMODE_EN);
}

static void mtk_mipi_tx_power_off_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);
	void __iomem *base = mipi_tx->regs;

	/* Switch ON each Lane */
	mtk_phy_set_bits(base + MIPITX_D0_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_D1_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_D2_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_D3_SW_CTL_EN, DSI_SW_CTL_EN);
	mtk_phy_set_bits(base + MIPITX_CK_SW_CTL_EN, DSI_SW_CTL_EN);

	writel(RG_DSI_PAD_TIEL_SEL | RG_DSI_BG_CORE_EN, base + MIPITX_LANE_CON);
	writel(RG_DSI_PAD_TIEL_SEL, base + MIPITX_LANE_CON);
}

const struct mtk_mipitx_data mt8183_mipitx_data = {
	.max_data_rate = 1600000000,
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};

/*
 * MT6833's PLL is MT8183's (MediaTek's mtk_mipi_tx_pll_prepare_mt6833()
 * programs it the same way as for MT6873, the SoC MT8192 is based on), but
 * its D-PHY runs at up to 2.5 Gbps per lane: its PLL takes rates up to 3 GHz
 * (mtk_mipi_tx_pll_round_rate()), and its bootloader drives the lanes at
 * 2326 Mbps on the Redmi Note 13 5G.
 */
const struct mtk_mipitx_data mt6833_mipitx_data = {
	.max_data_rate = 2500000000U,
	/* mtk_mipi_tx_pll_prepare_mt6833(): LANE_CON 0x3fff0180, then 0x3fff0080 */
	.bg_lpf_off = true,
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
	.read_boot_calibration = mtk_mipi_tx_read_boot_calibration,
};
