// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs impc_cg_regs = {
	.set_ofs = 0xE08,
	.clr_ofs = 0xE04,
	.sta_ofs = 0xE00,
};
static const struct mtk_gate_regs impe_cg_regs = {
	.set_ofs = 0xE08,
	.clr_ofs = 0xE04,
	.sta_ofs = 0xE00,
};
static const struct mtk_gate_regs impn_cg_regs = {
	.set_ofs = 0xE08,
	.clr_ofs = 0xE04,
	.sta_ofs = 0xE00,
};
static const struct mtk_gate_regs imps_cg_regs = {
	.set_ofs = 0xE08,
	.clr_ofs = 0xE04,
	.sta_ofs = 0xE00,
};
static const struct mtk_gate_regs impw_cg_regs = {
	.set_ofs = 0xE08,
	.clr_ofs = 0xE04,
	.sta_ofs = 0xE00,
};
static const struct mtk_gate_regs impws_cg_regs = {
	.set_ofs = 0xE08,
	.clr_ofs = 0xE04,
	.sta_ofs = 0xE00,
};

#define GATE_IMPC(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &impc_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IMPE(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &impe_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IMPN(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &impn_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IMPS(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &imps_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IMPW(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &impw_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IMPWS(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &impws_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate impc_clks[] = {
	GATE_IMPC(CLK_IMPC_I2C10_AP_CLOCK, "impc_i2c10_ap_clock", "i2c_pseudo_ck", 0),
	GATE_IMPC(CLK_IMPC_I2C11_AP_CLOCK, "impc_i2c11_ap_clock", "i2c_pseudo_ck", 1),
};

static const struct mtk_gate impe_clks[] = {
	GATE_IMPE(CLK_IMPE_I2C3_AP_CLOCK, "impe_i2c3_ap_clock", "i2c_pseudo_ck", 0),
};

static const struct mtk_gate impn_clks[] = {
	GATE_IMPN(CLK_IMPN_I2C6_AP_CLOCK, "impn_i2c6_ap_clock", "i2c_pseudo_ck", 0),
};

static const struct mtk_gate imps_clks[] = {
	GATE_IMPS(CLK_IMPS_I3C9_AP_CLOCK, "imps_i3c9_ap_clock", "i2c_pseudo_ck", 1),
	GATE_IMPS(CLK_IMPS_I3C8_AP_CLOCK, "imps_i3c8_ap_clock", "i2c_pseudo_ck", 0),
};

static const struct mtk_gate impw_clks[] = {
	GATE_IMPW(CLK_IMPW_I2C0_AP_CLOCK, "impw_i2c0_ap_clock", "i2c_pseudo_ck", 0),
	GATE_IMPW(CLK_IMPW_I2C5_AP_CLOCK, "impw_i2c5_ap_clock", "i2c_pseudo_ck", 1),
	GATE_IMPW(CLK_IMPW_I3C7_AP_CLOCK, "impw_i3c7_ap_clock", "i2c_pseudo_ck", 2),
};

static const struct mtk_gate impws_clks[] = {
	GATE_IMPWS(CLK_IMPWS_I2C1_AP_CLOCK, "impws_i2c1_ap_clock", "i2c_pseudo_ck", 0),
	GATE_IMPWS(CLK_IMPWS_I3C2_AP_CLOCK, "impws_i3c2_ap_clock", "i2c_pseudo_ck", 1),
	GATE_IMPWS(CLK_IMPWS_I3C4_AP_CLOCK, "impws_i3c4_ap_clock", "i2c_pseudo_ck", 2),
};

static const struct mtk_clk_desc impc_desc = {
	.clks = impc_clks,
	.num_clks = ARRAY_SIZE(impc_clks),
};

static const struct mtk_clk_desc impe_desc = {
	.clks = impe_clks,
	.num_clks = ARRAY_SIZE(impe_clks),
};

static const struct mtk_clk_desc impn_desc = {
	.clks = impn_clks,
	.num_clks = ARRAY_SIZE(impn_clks),
};

static const struct mtk_clk_desc imps_desc = {
	.clks = imps_clks,
	.num_clks = ARRAY_SIZE(imps_clks),
};

static const struct mtk_clk_desc impw_desc = {
	.clks = impw_clks,
	.num_clks = ARRAY_SIZE(impw_clks),
};

static const struct mtk_clk_desc impws_desc = {
	.clks = impws_clks,
	.num_clks = ARRAY_SIZE(impws_clks),
};

static const struct of_device_id of_match_clk_mt6833_imp_iic_wrap[] = {
	{ .compatible = "mediatek,mt6833-imp-iic-wrap-c", .data = &impc_desc },
	{ .compatible = "mediatek,mt6833-imp-iic-wrap-e", .data = &impe_desc },
	{ .compatible = "mediatek,mt6833-imp-iic-wrap-n", .data = &impn_desc },
	{ .compatible = "mediatek,mt6833-imp-iic-wrap-s", .data = &imps_desc },
	{ .compatible = "mediatek,mt6833-imp-iic-wrap-w", .data = &impw_desc },
	{ .compatible = "mediatek,mt6833-imp-iic-wrap-ws", .data = &impws_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_imp_iic_wrap);

static struct platform_driver clk_mt6833_imp_iic_wrap_drv = {
	.driver = {
		.name = "clk-mt6833-imp_iic_wrap",
		.of_match_table = of_match_clk_mt6833_imp_iic_wrap,
	},
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
};
module_platform_driver(clk_mt6833_imp_iic_wrap_drv);

MODULE_DESCRIPTION("MediaTek MT6833 I2C clocks driver");
MODULE_LICENSE("GPL");
