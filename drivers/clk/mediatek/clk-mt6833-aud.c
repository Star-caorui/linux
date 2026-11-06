// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs afe0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs afe1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x4,
	.sta_ofs = 0x4,
};

static const struct mtk_gate_regs afe2_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0x8,
	.sta_ofs = 0x8,
};

#define GATE_AFE0(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &afe0_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr)

#define GATE_AFE1(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &afe1_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr)

#define GATE_AFE2(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &afe2_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr)

static const struct mtk_gate afe_clks[] = {
	/* AFE0 */
	GATE_AFE0(CLK_AFE_AFE, "afe_afe", "audio_ck", 2),
	GATE_AFE0(CLK_AFE_22M, "afe_22m", "aud_engen1_ck", 8),
	GATE_AFE0(CLK_AFE_24M, "afe_24m", "aud_engen2_ck", 9),
	GATE_AFE0(CLK_AFE_APLL2_TUNER, "afe_apll2_tuner", "aud_engen2_ck", 18),
	GATE_AFE0(CLK_AFE_APLL_TUNER, "afe_apll_tuner", "aud_engen1_ck", 19),
	GATE_AFE0(CLK_AFE_TDM, "afe_tdm_ck", "aud_1_ck", 20),
	GATE_AFE0(CLK_AFE_ADC, "afe_adc", "audio_ck", 24),
	GATE_AFE0(CLK_AFE_DAC, "afe_dac", "audio_ck", 25),
	GATE_AFE0(CLK_AFE_DAC_PREDIS, "afe_dac_predis", "audio_ck", 26),
	GATE_AFE0(CLK_AFE_TML, "afe_tml", "audio_ck", 27),
	GATE_AFE0(CLK_AFE_NLE, "afe_nle", "audio_ck", 28),
	/* AFE1 */
	GATE_AFE1(CLK_AFE_I2S1_BCLK, "afe_i2s1_bclk", "audio_ck", 4),
	GATE_AFE1(CLK_AFE_I2S2_BCLK, "afe_i2s2_bclk", "audio_ck", 5),
	GATE_AFE1(CLK_AFE_I2S3_BCLK, "afe_i2s3_bclk", "audio_ck", 6),
	GATE_AFE1(CLK_AFE_I2S4_BCLK, "afe_i2s4_bclk", "audio_ck", 7),
	GATE_AFE1(CLK_AFE_CONNSYS_I2S_ASRC, "afe_connsys_i2s_asrc", "audio_ck", 12),
	GATE_AFE1(CLK_AFE_GENERAL1_ASRC, "afe_general1_asrc", "audio_ck", 13),
	GATE_AFE1(CLK_AFE_GENERAL2_ASRC, "afe_general2_asrc", "audio_ck", 14),
	GATE_AFE1(CLK_AFE_DAC_HIRES, "afe_dac_hires", "audio_h_ck", 15),
	GATE_AFE1(CLK_AFE_ADC_HIRES, "afe_adc_hires", "audio_h_ck", 16),
	GATE_AFE1(CLK_AFE_ADC_HIRES_TML, "afe_adc_hires_tml", "audio_h_ck", 17),
	GATE_AFE1(CLK_AFE_ADDA6_ADC, "afe_adda6_adc", "audio_ck", 20),
	GATE_AFE1(CLK_AFE_ADDA6_ADC_HIRES, "afe_adda6_adc_hires", "audio_h_ck", 21),
	GATE_AFE1(CLK_AFE_3RD_DAC, "afe_3rd_dac", "audio_ck", 28),
	GATE_AFE1(CLK_AFE_3RD_DAC_PREDIS, "afe_3rd_dac_predis", "audio_ck", 29),
	GATE_AFE1(CLK_AFE_3RD_DAC_TML, "afe_3rd_dac_tml", "audio_ck", 30),
	GATE_AFE1(CLK_AFE_3RD_DAC_HIRES, "afe_3rd_dac_hires", "audio_h_ck", 31),
	/* AFE2 */
	GATE_AFE2(CLK_AFE_I2S5_BCLK, "afe_i2s5_bclk", "audio_ck", 0),
};

static const struct mtk_clk_desc afe_desc = {
	.clks = afe_clks,
	.num_clks = ARRAY_SIZE(afe_clks),
	.need_runtime_pm = true,
};

static int clk_mt6833_aud_probe(struct platform_device *pdev)
{
	int r;

	r = mtk_clk_simple_probe(pdev);
	if (r)
		return r;

	r = devm_of_platform_populate(&pdev->dev);
	if (r)
		mtk_clk_simple_remove(pdev);

	return r;
}

static void clk_mt6833_aud_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	mtk_clk_simple_remove(pdev);
}

static const struct of_device_id of_match_clk_mt6833_aud[] = {
	{ .compatible = "mediatek,mt6833-audsys", .data = &afe_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_aud);

static struct platform_driver clk_mt6833_aud_drv = {
	.driver = {
		.name = "clk-mt6833-aud",
		.of_match_table = of_match_clk_mt6833_aud,
	},
	.probe = clk_mt6833_aud_probe,
	.remove = clk_mt6833_aud_remove,
};
module_platform_driver(clk_mt6833_aud_drv);

MODULE_DESCRIPTION("MediaTek MT6833 audio clocks driver");
MODULE_LICENSE("GPL");
