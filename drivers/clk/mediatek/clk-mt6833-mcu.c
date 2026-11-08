// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6833 mcusys clocks: the source selectors of the two CPU
 * clusters and of the CCI, which mediatek-cpufreq and mtk-cci-devfreq move
 * to the crystal while they retune the cluster PLLs.
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"

/*
 * Inputs 2 and 3 are MAINPLL and UNIVPLL in MediaTek's kernel, which never
 * selects them; they are left out until something needs them.
 */
static const char * const mcu_armpll_ll_parents[] = {
	"clk26m",
	"armpll_ll",
};

static const char * const mcu_armpll_bl_parents[] = {
	"clk26m",
	"armpll_bl0",
};

static const char * const mcu_armpll_bus_parents[] = {
	"clk26m",
	"ccipll",
};

/* Only the selectors are Linux's; the dividers stay as the bootloader set them. */
static struct mtk_composite mcu_muxes[] = {
	/* CPU_PLLDIV_CFG0 */
	MUX(CLK_MCU_ARMPLL_LL_SEL, "mcu_armpll_ll_sel", mcu_armpll_ll_parents, 0x2a0, 9, 2),
	/* CPU_PLLDIV_CFG1 */
	MUX(CLK_MCU_ARMPLL_BL_SEL, "mcu_armpll_bl_sel", mcu_armpll_bl_parents, 0x2a4, 9, 2),
	/* BUS_PLLDIV_CFG */
	MUX(CLK_MCU_ARMPLL_BUS_SEL, "mcu_armpll_bus_sel", mcu_armpll_bus_parents, 0x2e0, 9, 2),
};

static const struct mtk_clk_desc mcu_desc = {
	.composite_clks = mcu_muxes,
	.num_composite_clks = ARRAY_SIZE(mcu_muxes),
};

static const struct of_device_id of_match_clk_mt6833_mcu[] = {
	{ .compatible = "mediatek,mt6833-mcusys", .data = &mcu_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_mcu);

static struct platform_driver clk_mt6833_mcu_drv = {
	.driver = {
		.name = "clk-mt6833-mcu",
		.of_match_table = of_match_clk_mt6833_mcu,
	},
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
};
module_platform_driver(clk_mt6833_mcu_drv);

MODULE_DESCRIPTION("MediaTek MT6833 mcusys clocks driver");
MODULE_LICENSE("GPL");
