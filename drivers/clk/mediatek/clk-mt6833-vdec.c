// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs vde20_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x4,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs vde21_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0xc,
	.sta_ofs = 0x8,
};

#define GATE_VDE20(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &vde20_cg_regs, _shift, &mtk_clk_gate_ops_setclr_inv)

#define GATE_VDE21(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &vde21_cg_regs, _shift, &mtk_clk_gate_ops_setclr_inv)

static const struct mtk_gate vde2_clks[] = {
	/* VDE20 */
	GATE_VDE20(CLK_VDE2_VDEC_CKEN, "vde2_vdec_cken", "vdec_ck", 0),
	GATE_VDE20(CLK_VDE2_VDEC_ACTIVE, "vde2_vdec_active", "vdec_ck", 4),
	/* VDE21 */
	GATE_VDE21(CLK_VDE2_LARB1_CKEN, "vde2_larb1_cken", "vdec_ck", 0),
};

static const struct mtk_clk_desc vde2_desc = {
	.clks = vde2_clks,
	.num_clks = ARRAY_SIZE(vde2_clks),
};

static const struct of_device_id of_match_clk_mt6833_vdec[] = {
	{ .compatible = "mediatek,mt6833-vdecsys", .data = &vde2_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_vdec);

static struct platform_driver clk_mt6833_vdec_drv = {
	.driver = {
		.name = "clk-mt6833-vdec",
		.of_match_table = of_match_clk_mt6833_vdec,
	},
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
};
module_platform_driver(clk_mt6833_vdec_drv);

MODULE_DESCRIPTION("MediaTek MT6833 video decoder clocks driver");
MODULE_LICENSE("GPL");
