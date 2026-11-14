/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Jitao Shi <jitao.shi@mediatek.com>
 */

#ifndef _MTK_MIPI_TX_H
#define _MTK_MIPI_TX_H

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/slab.h>

struct mtk_mipi_tx;

struct mtk_mipitx_data {
	const u32 mppll_preserve;
	const u32 max_data_rate;
	/* leave the bandgap low pass filter off once the core is up */
	const bool bg_lpf_off;
	const struct clk_ops *mipi_tx_clk_ops;
	void (*mipi_tx_enable_signal)(struct phy *phy);
	void (*mipi_tx_disable_signal)(struct phy *phy);
	/* without an nvmem cell: take the impedance the bootloader set */
	void (*read_boot_calibration)(struct mtk_mipi_tx *mipi_tx);
};

struct mtk_mipi_tx {
	struct device *dev;
	void __iomem *regs;
	u32 data_rate;
	u32 mipitx_drive;
	u32 rt_code[5];
	const struct mtk_mipitx_data *driver_data;
	struct clk_hw pll_hw;
};

struct mtk_mipi_tx *mtk_mipi_tx_from_clk_hw(struct clk_hw *hw);
int mtk_mipi_tx_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate);
unsigned long mtk_mipi_tx_pll_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate);

extern const struct mtk_mipitx_data mt2701_mipitx_data;
extern const struct mtk_mipitx_data mt6833_mipitx_data;
extern const struct mtk_mipitx_data mt8173_mipitx_data;
extern const struct mtk_mipitx_data mt8183_mipitx_data;
extern const struct mtk_mipitx_data mt8196_mipitx_data;
#endif
