// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-fhctl.h"
#include "clk-mtk.h"
#include "clk-pll.h"
#include "clk-pllfh.h"

#define MT6833_PLL_FMAX		(3800UL * MHZ)
#define MT6833_PLL_FMIN		(1500UL * MHZ)
#define MT6833_INTEGER_BITS	8

#define PLL(_id, _name, _reg, _en_reg, _en_mask, _pll_en_bit,		\
			_pwr_reg, _flags, _rst_bar_mask,		\
			_pd_reg, _pd_shift, _tuner_reg,			\
			_tuner_en_reg, _tuner_en_bit,			\
			_pcw_reg, _pcw_shift, _pcwbits) {		\
		.id = _id,						\
		.name = _name,						\
		.reg = _reg,						\
		.en_reg = _en_reg,					\
		.en_mask = _en_mask,					\
		.pll_en_bit = _pll_en_bit,				\
		.pwr_reg = _pwr_reg,					\
		.flags = _flags,				\
		.rst_bar_mask = _rst_bar_mask,				\
		.fmax = MT6833_PLL_FMAX,				\
		.fmin = MT6833_PLL_FMIN,				\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.tuner_reg = _tuner_reg,				\
		.tuner_en_reg = _tuner_en_reg,			\
		.tuner_en_bit = _tuner_en_bit,				\
		.pcw_reg = _pcw_reg,					\
		.pcw_shift = _pcw_shift,				\
		.pcwbits = _pcwbits,					\
		.pcwibits = MT6833_INTEGER_BITS,			\
	}

#define PLL_B(_id, _name, _reg, _en_reg, _en_mask, _pll_en_bit,		\
			_pwr_reg, _flags, _rst_bar_mask,		\
			_pd_reg, _pd_shift, _tuner_reg,			\
			_tuner_en_reg, _tuner_en_bit,			\
			_pcw_reg, _pcw_shift, _pcwbits, _div_table) {					\
		.id = _id,						\
		.name = _name,						\
		.reg = _reg,						\
		.en_reg = _en_reg,					\
		.en_mask = _en_mask,					\
		.pll_en_bit = _pll_en_bit,				\
		.pwr_reg = _pwr_reg,					\
		.flags = _flags,				\
		.rst_bar_mask = _rst_bar_mask,				\
		.fmax = MT6833_PLL_FMAX,				\
		.fmin = MT6833_PLL_FMIN,				\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.tuner_reg = _tuner_reg,				\
		.tuner_en_reg = _tuner_en_reg,			\
		.tuner_en_bit = _tuner_en_bit,				\
		.pcw_reg = _pcw_reg,					\
		.pcw_shift = _pcw_shift,				\
		.pcwbits = _pcwbits,	                \
		.div_table = _div_table,				\
		.pcwibits = MT6833_INTEGER_BITS,        \
	}

static const struct mtk_pll_div_table mfgpll_div_table[] = {
	{ .div = 0, .freq = 1900000000},
	{ .div = 1, .freq = 1069000000 },
	{ .div = 2, .freq = 954000000 },
	{ .div = 3, .freq = 389000000 },
	{ .div = 4, .freq = 1 },
	{ /* sentinel */ }
};

static const struct mtk_pll_data plls[] = {
	PLL(CLK_APMIXED_ARMPLL_LL, "armpll_ll", 0x208, 0x208, 0, 0, 0x214, PLL_AO, BIT(0), 0x20c,
	    24, 0, 0, 0, 0x20c, 0, 22),
	PLL(CLK_APMIXED_ARMPLL_BL0, "armpll_bl0", 0x218, 0x218, 0, 0, 0x224, PLL_AO, BIT(0), 0x21c,
	    24, 0, 0, 0, 0x21c, 0, 22),
	PLL(CLK_APMIXED_CCIPLL, "ccipll", 0x258, 0x258, 0, 0, 0x264, PLL_AO, BIT(0), 0x25c, 24, 0,
	    0, 0, 0x25c, 0, 22),
	PLL(CLK_APMIXED_MPLL, "mpll", 0x390, 0x390, 0, 0, 0x39c, PLL_AO, BIT(0), 0x394, 24, 0, 0, 0,
	    0x394, 0, 22),
	PLL(CLK_APMIXED_MAINPLL, "mainpll", 0x340, 0x340, 0xff000000, 0, 0x34c,
	    HAVE_RST_BAR | PLL_AO, BIT(23), 0x344, 24, 0, 0, 0, 0x344, 0, 22),
	PLL(CLK_APMIXED_UNIVPLL, "univpll", 0x308, 0x308, 0xff000000, 0, 0x314, HAVE_RST_BAR,
	    BIT(23), 0x30c, 24, 0, 0, 0, 0x30c, 0, 22),
	PLL(CLK_APMIXED_MSDCPLL, "msdcpll", 0x350, 0x350, 0, 0, 0x35c, 0, BIT(0), 0x354, 24, 0, 0,
	    0, 0x354, 0, 22),
	PLL(CLK_APMIXED_MMPLL, "mmpll", 0x360, 0x360, 0xff000000, 0, 0x36c, HAVE_RST_BAR, BIT(23),
	    0x364, 24, 0, 0, 0, 0x364, 0, 22),
	PLL(CLK_APMIXED_ADSPPLL, "adsppll", 0x370, 0x370, 0xff000000, 0, 0x37c, 0, BIT(0), 0x374,
	    24, 0, 0, 0, 0x374, 0, 22),
	PLL_B(CLK_APMIXED_MFGPLL, "mfgpll", 0x268, 0x268, 0, 0, 0x274, 0, BIT(0), 0x26c, 24, 0, 0,
	      0, 0x26c, 0, 22, mfgpll_div_table),
	PLL(CLK_APMIXED_TVDPLL, "tvdpll", 0x380, 0x380, 0, 0, 0x38c, 0, BIT(0), 0x384, 24, 0, 0, 0,
	    0x384, 0, 22),
	PLL(CLK_APMIXED_APLL1, "apll1", 0x318, 0x318, 0, 0, 0x328, 0, BIT(0), 0x31c, 24, 0x040,
	    0x00c, 0, 0x320, 0, 32),
	PLL(CLK_APMIXED_APLL2, "apll2", 0x32c, 0x32c, 0, 0, 0x33c, 0, BIT(0), 0x330, 24, 0x044,
	    0x00c, 5, 0x334, 0, 32),
	PLL(CLK_APMIXED_NPUPLL, "npupll", 0x3b4, 0x3b4, 0xff000000, 0, 0x3c0, 0, BIT(0), 0x3b8, 24,
	    0, 0, 0, 0x3b8, 0, 22),
	PLL(CLK_APMIXED_USBPLL, "usbpll", 0x3c4, 0x3cc, 0, 2, 0x3cc, 0, BIT(0), 0x3c4, 24, 0, 0, 0,
	    0x3c4, 0, 22),
};

enum fh_pll_id {
	FH_ARMPLL_LL,
	FH_ARMPLL_BL0,
	FH_ARMPLL_BL1,
	FH_ARMPLL_BL2,
	FH_NPUPLL,
	FH_CCIPLL,
	FH_MFGPLL,
	FH_MEMPLL,
	FH_MPLL,
	FH_MMPLL,
	FH_MAINPLL,
	FH_MSDCPLL,
	FH_ADSPPLL,
	FH_APUPLL,
	FH_TVDPLL,
	FH_NR_FH,
};

#define FH(_pllid, _fhid, _offset) {					\
		.data = {						\
			.pll_id = _pllid,				\
			.fh_id = _fhid,					\
			.fh_ver = FHCTL_PLLFH_V2,			\
			.fhx_offset = _offset,				\
			.dds_mask = GENMASK(21, 0),			\
			.slope0_value = 0x6003c97,			\
			.slope1_value = 0x6003c97,			\
			.sfstrx_en = BIT(2),				\
			.frddsx_en = BIT(1),				\
			.fhctlx_en = BIT(0),				\
			.tgl_org = BIT(31),				\
			.dvfs_tri = BIT(31),				\
			.pcwchg = BIT(31),				\
			.dt_val = 0x0,					\
			.df_val = 0x9,					\
			.updnlmt_shft = 16,				\
			.msk_frddsx_dys = GENMASK(23, 20),		\
			.msk_frddsx_dts = GENMASK(19, 16),		\
		},							\
	}

/*
 * The PLLs the kernel may hop. The CPU PLLs and CCIPLL belong to the
 * MCUPM's CPU DVFS, so they are not listed.
 */
static struct mtk_pllfh_data pllfhs[] = {
	FH(CLK_APMIXED_NPUPLL, FH_NPUPLL, 0x8c),
	FH(CLK_APMIXED_MFGPLL, FH_MFGPLL, 0xb4),
	FH(CLK_APMIXED_MAINPLL, FH_MAINPLL, 0x104),
	FH(CLK_APMIXED_MSDCPLL, FH_MSDCPLL, 0x118),
	FH(CLK_APMIXED_ADSPPLL, FH_ADSPPLL, 0x12c),
	FH(CLK_APMIXED_TVDPLL, FH_TVDPLL, 0x154),
};

static const struct of_device_id of_match_clk_mt6833_apmixed[] = {
	{ .compatible = "mediatek,mt6833-apmixedsys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_apmixed);

static int clk_mt6833_apmixed_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;
	const u8 *fhctl_node = "mediatek,mt6833-fhctl";
	int r;

	clk_data = mtk_alloc_clk_data(ARRAY_SIZE(plls));
	if (!clk_data)
		return -ENOMEM;

	fhctl_parse_dt(fhctl_node, pllfhs, ARRAY_SIZE(pllfhs));

	r = mtk_clk_register_pllfhs(&pdev->dev, plls, ARRAY_SIZE(plls),
				    pllfhs, ARRAY_SIZE(pllfhs), clk_data);
	if (r)
		goto free_clk_data;

	r = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	if (r)
		goto unregister_plls;

	platform_set_drvdata(pdev, clk_data);

	return 0;

unregister_plls:
	mtk_clk_unregister_pllfhs(plls, ARRAY_SIZE(plls), pllfhs,
				  ARRAY_SIZE(pllfhs), clk_data);
free_clk_data:
	mtk_free_clk_data(clk_data);
	return r;
}

static void clk_mt6833_apmixed_remove(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_hw_onecell_data *clk_data = platform_get_drvdata(pdev);

	of_clk_del_provider(node);
	mtk_clk_unregister_pllfhs(plls, ARRAY_SIZE(plls), pllfhs,
				  ARRAY_SIZE(pllfhs), clk_data);
	mtk_free_clk_data(clk_data);
}

static struct platform_driver clk_mt6833_apmixed_drv = {
	.driver = {
		.name = "clk-mt6833-apmixed",
		.of_match_table = of_match_clk_mt6833_apmixed,
	},
	.probe = clk_mt6833_apmixed_probe,
	.remove = clk_mt6833_apmixed_remove,
};
module_platform_driver(clk_mt6833_apmixed_drv);

MODULE_DESCRIPTION("MediaTek MT6833 apmixed clocks driver");
MODULE_LICENSE("GPL");
