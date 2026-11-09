// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 * Copyright (c) 2026 Rui Cao <i@udp0.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-mux.h"

static DEFINE_SPINLOCK(mt6833_clk_lock);

static const struct mtk_fixed_clk top_fixed_clks[] = {
	FIXED_CLK(CLK_TOP_ULPOSC, "ulposc", NULL, 260000000),
};

/*
 * Nearly every bus and peripheral clock of the SoC runs from MAINPLL or
 * UNIVPLL, so rate requests stop at their dividers instead of retuning them.
 */
static const struct mtk_fixed_factor top_divs[] = {
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D4, "mainpll_d4", "mainpll", 1, 4, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D4_D2, "mainpll_d4_d2", "mainpll", 1, 8, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D4_D4, "mainpll_d4_d4", "mainpll", 1, 16, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D4_D8, "mainpll_d4_d8", "mainpll", 1, 32, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D4_D16, "mainpll_d4_d16", "mainpll", 1, 64, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D5, "mainpll_d5", "mainpll", 1, 5, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D5_D2, "mainpll_d5_d2", "mainpll", 1, 10, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D5_D4, "mainpll_d5_d4", "mainpll", 1, 20, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D5_D8, "mainpll_d5_d8", "mainpll", 1, 40, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D6, "mainpll_d6", "mainpll", 1, 6, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D6_D2, "mainpll_d6_d2", "mainpll", 1, 12, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D6_D4, "mainpll_d6_d4", "mainpll", 1, 24, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D7, "mainpll_d7", "mainpll", 1, 7, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D7_D2, "mainpll_d7_d2", "mainpll", 1, 14, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D7_D4, "mainpll_d7_d4", "mainpll", 1, 28, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D7_D8, "mainpll_d7_d8", "mainpll", 1, 56, 0),
	FACTOR_FLAGS(CLK_TOP_MAINPLL_D9, "mainpll_d9", "mainpll", 1, 9, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D4, "univpll_d4", "univpll", 1, 4, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D4_D2, "univpll_d4_d2", "univpll", 1, 8, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D4_D4, "univpll_d4_d4", "univpll", 1, 16, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D4_D8, "univpll_d4_d8", "univpll", 1, 32, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D5, "univpll_d5", "univpll", 1, 5, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D5_D2, "univpll_d5_d2", "univpll", 1, 10, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D5_D4, "univpll_d5_d4", "univpll", 1, 20, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D5_D8, "univpll_d5_d8", "univpll", 1, 40, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D6, "univpll_d6", "univpll", 1, 6, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D6_D2, "univpll_d6_d2", "univpll", 1, 12, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D6_D4, "univpll_d6_d4", "univpll", 1, 24, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D6_D8, "univpll_d6_d8", "univpll", 1, 48, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D6_D16, "univpll_d6_d16", "univpll", 1, 96, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_D7, "univpll_d7", "univpll", 1, 7, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_192M_D2, "univpll_192m_d2", "univpll", 1, 26, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_192M_D4, "univpll_192m_d4", "univpll", 1, 52, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_192M_D8, "univpll_192m_d8", "univpll", 1, 104, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_192M_D16, "univpll_192m_d16", "univpll", 1, 208, 0),
	FACTOR_FLAGS(CLK_TOP_UNIVPLL_192M_D32, "univpll_192m_d32", "univpll", 1, 416, 0),
	FACTOR(CLK_TOP_APLL1_D2, "apll1_d2", "apll1", 1, 2),
	FACTOR(CLK_TOP_APLL1_D4, "apll1_d4", "apll1", 1, 4),
	FACTOR(CLK_TOP_APLL1_D8, "apll1_d8", "apll1", 1, 8),
	FACTOR(CLK_TOP_APLL2_D2, "apll2_d2", "apll2", 1, 2),
	FACTOR(CLK_TOP_APLL2_D4, "apll2_d4", "apll2", 1, 4),
	FACTOR(CLK_TOP_APLL2_D8, "apll2_d8", "apll2", 1, 8),
	FACTOR(CLK_TOP_MMPLL_D4_D2, "mmpll_d4_d2", "mmpll", 1, 8),
	FACTOR(CLK_TOP_MMPLL_D5_D2, "mmpll_d5_d2", "mmpll", 1, 10),
	FACTOR(CLK_TOP_MMPLL_D6, "mmpll_d6", "mmpll", 1, 6),
	FACTOR(CLK_TOP_MMPLL_D6_D2, "mmpll_d6_d2", "mmpll", 1, 12),
	FACTOR(CLK_TOP_MMPLL_D7, "mmpll_d7", "mmpll", 1, 7),
	FACTOR(CLK_TOP_MMPLL_D9, "mmpll_d9", "mmpll", 1, 9),
	FACTOR(CLK_TOP_MSDCPLL_D2, "msdcpll_d2", "msdcpll", 1, 2),
	FACTOR(CLK_TOP_MSDCPLL_D4, "msdcpll_d4", "msdcpll", 1, 4),
	FACTOR(CLK_TOP_F26M_CK_D2, "f26m_d2", "clk26m", 1, 2),
	FACTOR(CLK_TOP_OSC_D2, "osc_d2", "ulposc", 1, 2),
	FACTOR(CLK_TOP_OSC_D4, "osc_d4", "ulposc", 1, 4),
	FACTOR(CLK_TOP_OSC_D8, "osc_d8", "ulposc", 1, 8),
	FACTOR(CLK_TOP_OSC_D16, "osc_d16", "ulposc", 1, 16),
	FACTOR(CLK_TOP_OSC_D10, "osc_d10", "ulposc", 1, 10),
	FACTOR(CLK_TOP_OSC_D20, "osc_d20", "ulposc", 1, 20),
};

static const char * const axi_parents[] = {
	"clk26m",
	"mainpll_d4_d4",
	"mainpll_d7_d2",
	"mainpll_d4_d2",
	"mainpll_d5_d2",
	"mainpll_d6_d2",
	"osc_d4",
};

static const char * const spm_parents[] = {
	"clk26m",
	"osc_d10",
	"mainpll_d7_d4",
	"clk32k",
};

static const char * const scp_parents[] = {
	"clk26m",
	"univpll_d4",
	"npupll",
	"mainpll_d6",
	"univpll_d6",
	"mainpll_d4_d2",
	"mainpll_d4",
	"mainpll_d7",
};

static const char * const bus_aximem_parents[] = {
	"clk26m",
	"mainpll_d7_d2",
	"mainpll_d4_d2",
	"mainpll_d5_d2",
	"mainpll_d6",
};

static const char * const disp_parents[] = {
	"clk26m",
	"univpll_d6_d2",
	"mainpll_d5_d2",
	"mmpll_d6_d2",
	"univpll_d5_d2",
	"univpll_d4_d2",
	"mmpll_d7",
	"univpll_d6",
	"mainpll_d4",
	"mmpll_d5_d2",
};

static const char * const mdp_parents[] = {
	"clk26m",
	"mainpll_d5_d2",
	"mmpll_d6_d2",
	"mainpll_d4_d2",
	"mmpll_d4_d2",
	"mainpll_d6",
	"mainpll_d5",
	"mainpll_d4",
	"tvdpll",
	"univpll_d4",
	"mmpll_d5_d2",
};

static const char * const img_parents[] = {
	"clk26m",
	"univpll_d4",
	"tvdpll",
	"mainpll_d4",
	"univpll_d5",
	"mmpll_d6",
	"univpll_d6",
	"mainpll_d6",
	"mmpll_d4_d2",
	"mainpll_d4_d2",
	"mmpll_d6_d2",
	"mmpll_d5_d2",
};

static const char * const ipe_parents[] = {
	"clk26m",
	"mainpll_d4",
	"mmpll_d6",
	"univpll_d6",
	"mainpll_d6",
	"univpll_d4_d2",
	"mainpll_d4_d2",
	"mmpll_d6_d2",
	"mmpll_d5_d2",
};

static const char * const dpe_parents[] = {
	"clk26m",
	"mainpll_d4",
	"mmpll_d6",
	"univpll_d6",
	"mainpll_d6",
	"univpll_d4_d2",
	"univpll_d5_d2",
	"mmpll_d6_d2",
};

static const char * const cam_parents[] = {
	"clk26m",
	"mainpll_d4",
	"mmpll_d6",
	"univpll_d4",
	"univpll_d5",
	"univpll_d6",
	"mmpll_d7",
	"univpll_d4_d2",
	"mainpll_d4_d2",
	"npupll",
};

static const char * const ccu_parents[] = {
	"clk26m",
	"mainpll_d4",
	"mmpll_d6",
	"mainpll_d6",
	"mmpll_d7",
	"univpll_d4_d2",
	"mmpll_d6_d2",
	"mmpll_d5_d2",
	"univpll_d5",
	"univpll_d6_d2",
};

static const char * const mfg_ref_parents[] = {
	"clk26m",
	"mainpll_d6_d2",
	"mainpll_d6",
	"mainpll_d5_d2",
};

static const char * const mfg_pll_parents[] = {
	"mfg_ref_sel",
	"mfgpll",
};

static const char * const camtg_parents[] = {
	"clk26m",
	"univpll_192m_d8",
	"univpll_d6_d8",
	"univpll_192m_d4",
	"univpll_d6_d16",
	"f26m_d2",
	"univpll_192m_d16",
	"univpll_192m_d32",
};

static const char * const uart_parents[] = {
	"clk26m",
	"univpll_d6_d8",
};

static const char * const spi_parents[] = {
	"clk26m",
	"mainpll_d5_d4",
	"mainpll_d6_d4",
	"msdcpll_d4",
	"msdcpll_d2",
	"mainpll_d6_d2",
	"mainpll_d4_d4",
	"univpll_d5_d4",
};

static const char * const msdc5hclk_parents[] = {
	"clk26m",
	"mainpll_d4_d2",
	"mainpll_d6_d2",
};

static const char * const msdc50_0_parents[] = {
	"clk26m",
	"msdcpll",
	"msdcpll_d2",
	"univpll_d4_d4",
	"mainpll_d6_d2",
	"univpll_d4_d2",
};

static const char * const msdc30_1_parents[] = {
	"clk26m",
	"univpll_d6_d2",
	"mainpll_d6_d2",
	"mainpll_d7_d2",
	"msdcpll_d2",
};

static const char * const audio_parents[] = {
	"clk26m",
	"mainpll_d5_d8",
	"mainpll_d7_d8",
	"mainpll_d4_d16",
};

static const char * const aud_intbus_parents[] = {
	"clk26m",
	"mainpll_d4_d4",
	"mainpll_d7_d4",
};

static const char * const pwrap_ulposc_parents[] = {
	"osc_d10",
	"clk26m",
	"osc_d4",
	"osc_d8",
	"osc_d16",
};

static const char * const atb_parents[] = {
	"clk26m",
	"mainpll_d4_d2",
	"mainpll_d5_d2",
};

static const char * const sspm_parents[] = {
	"clk26m",
	"mainpll_d7_d2",
	"mainpll_d6_d2",
	"mainpll_d5_d2",
	"mainpll_d9",
	"mainpll_d4_d2",
};

static const char * const scam_parents[] = {
	"clk26m",
	"mainpll_d5_d4",
};

static const char * const disp_pwm_parents[] = {
	"clk26m",
	"univpll_d6_d4",
	"osc_d2",
	"osc_d4",
	"osc_d16",
};

static const char * const usb_parents[] = {
	"clk26m",
	"univpll_d5_d4",
	"univpll_d6_d4",
	"univpll_d5_d2",
};

static const char * const i2c_parents[] = {
	"clk26m",
	"mainpll_d4_d8",
	"univpll_d5_d4",
};

static const char * const seninf_parents[] = {
	"clk26m",
	"univpll_d4_d4",
	"univpll_d6_d2",
	"univpll_d4_d2",
	"npupll",
	"mmpll_d7",
	"mmpll_d6",
	"univpll_d5",
};

static const char * const dxcc_parents[] = {
	"clk26m",
	"mainpll_d4_d2",
	"mainpll_d4_d4",
	"mainpll_d4_d8",
};

static const char * const aud_engen1_parents[] = {
	"clk26m",
	"apll1_d2",
	"apll1_d4",
	"apll1_d8",
};

static const char * const aud_engen2_parents[] = {
	"clk26m",
	"apll2_d2",
	"apll2_d4",
	"apll2_d8",
};

static const char * const aes_ufsfde_parents[] = {
	"clk26m",
	"mainpll_d4",
	"mainpll_d4_d2",
	"mainpll_d6",
	"mainpll_d4_d4",
	"univpll_d4_d2",
	"univpll_d6",
};

static const char * const ufs_parents[] = {
	"clk26m",
	"mainpll_d4_d4",
	"mainpll_d4_d8",
	"univpll_d4_d4",
	"mainpll_d6_d2",
	"mainpll_d5_d2",
	"msdcpll_d2",
};

static const char * const aud_1_parents[] = {
	"clk26m",
	"apll1",
};

static const char * const aud_2_parents[] = {
	"clk26m",
	"apll2",
};

static const char * const adsp_parents[] = {
	"clk26m",
	"mainpll_d6",
	"mainpll_d5_d2",
	"univpll_d4_d4",
	"univpll_d4",
	"univpll_d6",
	"ulposc",
	"npupll",
};

static const char * const dpmaif_main_parents[] = {
	"clk26m",
	"univpll_d4_d4",
	"mainpll_d6",
	"mainpll_d4_d2",
	"univpll_d4_d2",
};

static const char * const venc_parents[] = {
	"clk26m",
	"mmpll_d7",
	"mainpll_d6",
	"univpll_d4_d2",
	"mainpll_d4_d2",
	"univpll_d6",
	"mmpll_d6",
	"mainpll_d5_d2",
	"mainpll_d6_d2",
	"mmpll_d9",
	"univpll_d4_d4",
	"mainpll_d4",
	"univpll_d4",
	"univpll_d5",
	"univpll_d5_d2",
	"mainpll_d5",
};

static const char * const vdec_parents[] = {
	"clk26m",
	"univpll_192m_d2",
	"univpll_d5_d4",
	"mainpll_d5",
	"mainpll_d5_d2",
	"mmpll_d6_d2",
	"univpll_d5_d2",
	"mainpll_d4_d2",
	"univpll_d4_d2",
	"univpll_d7",
	"mmpll_d7",
	"mmpll_d6",
	"univpll_d5",
	"mainpll_d4",
	"univpll_d4",
	"univpll_d6",
};

static const char * const camtm_parents[] = {
	"clk26m",
	"univpll_d7",
	"univpll_d6_d2",
	"univpll_d4_d2",
};

static const char * const pwm_parents[] = {
	"clk26m",
	"univpll_d4_d8",
};

static const char * const audio_h_parents[] = {
	"clk26m",
	"univpll_d7",
	"apll1",
	"apll2",
};

static const char * const spmi_mst_parents[] = {
	"clk26m",
	"f26m_d2",
	"osc_d8",
	"osc_d10",
	"osc_d16",
	"osc_d20",
	"clk32k",
};

static const char * const dvfsrc_parents[] = {
	"clk26m",
	"osc_d10",
};

static const char * const aes_msdcfde_parents[] = {
	"clk26m",
	"mainpll_d4_d2",
	"mainpll_d6",
	"mainpll_d4_d4",
	"univpll_d4_d2",
	"univpll_d6",
};

static const char * const mcupm_parents[] = {
	"clk26m",
	"mainpll_d6_d4",
	"mainpll_d6_d2",
};

static const char * const sflash_parents[] = {
	"clk26m",
	"mainpll_d7_d8",
	"univpll_d6_d8",
	"univpll_d5_d8",
};

static const char * const dsi_occ_parents[] = {
	"clk26m",
	"mainpll_d6_d2",
	"univpll_d5_d2",
	"univpll_d4_d2",
};

static const char * const apll_i2s_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

/*
 * MUX_CLR_SET_UPD() with flags, for the critical muxes that have no gate.
 */
#define MUX_CLR_SET_UPD_FLAGS(_id, _name, _parents, _mux_ofs,		\
			      _mux_set_ofs, _mux_clr_ofs, _shift, _width,	\
			      _upd_ofs, _upd, _flags)				\
	GATE_CLR_SET_UPD_FLAGS(_id, _name, _parents, _mux_ofs,		\
			       _mux_set_ofs, _mux_clr_ofs, _shift, _width,	\
			       0, _upd_ofs, _upd, _flags,			\
			       mtk_mux_clr_set_upd_ops)

/*
 * CRITICAL CLOCK:
 * axi_sel is the main bus clock of the SoC and bus_aximem_sel the clock of
 * the bus to the EMI. spm_sel, sspm_sel and mcupm_sel clock the always-on
 * coprocessors, dvfsrc_sel the DVFS resource collector and pwrap_ulposc_sel
 * the PMIC wrapper, through which every voltage change goes.
 */
static const struct mtk_mux top_mtk_muxes[] = {
	/* CLK_CFG_0 */
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_AXI_SEL, "axi_sel", axi_parents, 0x010, 0x014, 0x018, 0, 3,
			      0x004, 0, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_SPM_SEL, "spm_sel", spm_parents, 0x010, 0x014, 0x018, 8, 2,
			      0x004, 1, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SCP_SEL, "scp_sel", scp_parents, 0x010, 0x014, 0x018, 16, 3,
			     23, 0x004, 2),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_BUS_AXIMEM_SEL, "bus_aximem_sel", bus_aximem_parents, 0x010,
			      0x014, 0x018, 24, 3, 0x004, 3, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	/* CLK_CFG_1 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DISP_SEL, "disp_sel", disp_parents, 0x020, 0x024, 0x028, 0, 4,
			     7, 0x004, 4),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MDP_SEL, "mdp_sel", mdp_parents, 0x020, 0x024, 0x028, 8, 4, 15,
			     0x004, 5),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IMG1_SEL, "img1_sel", img_parents, 0x020, 0x024, 0x028, 16, 4,
			     23, 0x004, 6),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IMG2_SEL, "img2_sel", img_parents, 0x020, 0x024, 0x028, 24, 4,
			     31, 0x004, 7),
	/* CLK_CFG_2 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IPE_SEL, "ipe_sel", ipe_parents, 0x030, 0x034, 0x038, 0, 4, 7,
			     0x004, 8),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DPE_SEL, "dpe_sel", dpe_parents, 0x030, 0x034, 0x038, 8, 3, 15,
			     0x004, 9),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAM_SEL, "cam_sel", cam_parents, 0x030, 0x034, 0x038, 16, 4,
			     23, 0x004, 10),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CCU_SEL, "ccu_sel", ccu_parents, 0x030, 0x034, 0x038, 24, 4,
			     31, 0x004, 11),
	/* CLK_CFG_4 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MFG_REF_SEL, "mfg_ref_sel", mfg_ref_parents, 0x050, 0x054,
			     0x058, 16, 2, 23, 0x004, 18),
	MUX_CLR_SET(CLK_TOP_MFG_PLL_SEL, "mfg_pll_sel", mfg_pll_parents, 0x050, 0x054, 0x058, 18,
		    1),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG_SEL, "camtg_sel", camtg_parents, 0x050, 0x054, 0x058, 24,
			     3, 31, 0x004, 19),
	/* CLK_CFG_5 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG2_SEL, "camtg2_sel", camtg_parents, 0x060, 0x064, 0x068,
			     0, 3, 7, 0x004, 20),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG3_SEL, "camtg3_sel", camtg_parents, 0x060, 0x064, 0x068,
			     8, 3, 15, 0x004, 21),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG4_SEL, "camtg4_sel", camtg_parents, 0x060, 0x064, 0x068,
			     16, 3, 23, 0x004, 22),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG5_SEL, "camtg5_sel", camtg_parents, 0x060, 0x064, 0x068,
			     24, 3, 31, 0x004, 23),
	/* CLK_CFG_6 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_UART_SEL, "uart_sel", uart_parents, 0x070, 0x074, 0x078, 8, 1,
			     15, 0x004, 25),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SPI_SEL, "spi_sel", spi_parents, 0x070, 0x074, 0x078, 16, 3,
			     23, 0x004, 26),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC50_0_HCLK_SEL, "msdc5hclk_sel", msdc5hclk_parents, 0x070,
			     0x074, 0x078, 24, 2, 31, 0x004, 27),
	/* CLK_CFG_7 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC50_0_SEL, "msdc50_0_sel", msdc50_0_parents, 0x080, 0x084,
			     0x088, 0, 3, 7, 0x004, 28),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC30_1_SEL, "msdc30_1_sel", msdc30_1_parents, 0x080, 0x084,
			     0x088, 8, 3, 15, 0x004, 29),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUDIO_SEL, "audio_sel", audio_parents, 0x080, 0x084, 0x088, 24,
			     2, 31, 0x008, 0),
	/* CLK_CFG_8 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_INTBUS_SEL, "aud_intbus_sel", aud_intbus_parents, 0x090,
			     0x094, 0x098, 0, 2, 7, 0x008, 1),
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_PWRAP_ULPOSC_SEL, "pwrap_ulposc_sel",
				   pwrap_ulposc_parents, 0x090, 0x094, 0x098, 8, 3, 15, 0x008, 2,
				   CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ATB_SEL, "atb_sel", atb_parents, 0x090, 0x094, 0x098, 16, 2,
			     23, 0x008, 3),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_SSPM_SEL, "sspm_sel", sspm_parents, 0x090, 0x094, 0x098, 24,
			      3, 0x008, 4, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	/* CLK_CFG_9 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SCAM_SEL, "scam_sel", scam_parents, 0x0a0, 0x0a4, 0x0a8, 8, 1,
			     15, 0x008, 6),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DISP_PWM_SEL, "disp_pwm_sel", disp_pwm_parents, 0x0a0, 0x0a4,
			     0x0a8, 16, 3, 23, 0x008, 7),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_USB_TOP_SEL, "usb_sel", usb_parents, 0x0a0, 0x0a4, 0x0a8, 24,
			     2, 31, 0x008, 8),
	/* CLK_CFG_10 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_USB_XHCI_SEL, "ssusb_xhci_sel", usb_parents, 0x0b0, 0x0b4,
			     0x0b8, 0, 2, 7, 0x008, 9),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_I2C_SEL, "i2c_sel", i2c_parents, 0x0b0, 0x0b4, 0x0b8, 8, 2, 15,
			     0x008, 10),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SENINF_SEL, "seninf_sel", seninf_parents, 0x0b0, 0x0b4, 0x0b8,
			     16, 3, 23, 0x008, 11),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SENINF1_SEL, "seninf1_sel", seninf_parents, 0x0b0, 0x0b4,
			     0x0b8, 24, 3, 31, 0x008, 12),
	/* CLK_CFG_11 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SENINF2_SEL, "seninf2_sel", seninf_parents, 0x0c0, 0x0c4,
			     0x0c8, 0, 3, 7, 0x008, 13),
	MUX_CLR_SET_UPD(CLK_TOP_DXCC_SEL, "dxcc_sel", dxcc_parents, 0x0c0, 0x0c4, 0x0c8, 24, 2,
			0x008, 16),
	/* CLK_CFG_12 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_ENGEN1_SEL, "aud_engen1_sel", aud_engen1_parents, 0x0d0,
			     0x0d4, 0x0d8, 0, 2, 7, 0x008, 17),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_ENGEN2_SEL, "aud_engen2_sel", aud_engen2_parents, 0x0d0,
			     0x0d4, 0x0d8, 8, 2, 15, 0x008, 18),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AES_UFSFDE_SEL, "aes_ufsfde_sel", aes_ufsfde_parents, 0x0d0,
			     0x0d4, 0x0d8, 16, 3, 23, 0x008, 19),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_UFS_SEL, "ufs_sel", ufs_parents, 0x0d0, 0x0d4, 0x0d8, 24, 3,
			     31, 0x008, 20),
	/* CLK_CFG_13 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_1_SEL, "aud_1_sel", aud_1_parents, 0x0e0, 0x0e4, 0x0e8, 0,
			     1, 7, 0x008, 21),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_2_SEL, "aud_2_sel", aud_2_parents, 0x0e0, 0x0e4, 0x0e8, 8,
			     1, 15, 0x008, 22),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ADSP_SEL, "adsp_sel", adsp_parents, 0x0e0, 0x0e4, 0x0e8, 16, 3,
			     23, 0x008, 23),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DPMAIF_MAIN_SEL, "dpmaif_main_sel", dpmaif_main_parents, 0x0e0,
			     0x0e4, 0x0e8, 24, 3, 31, 0x008, 24),
	/* CLK_CFG_14 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_VENC_SEL, "venc_sel", venc_parents, 0x0f0, 0x0f4, 0x0f8, 0, 4,
			     7, 0x008, 25),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_VDEC_SEL, "vdec_sel", vdec_parents, 0x0f0, 0x0f4, 0x0f8, 8, 4,
			     15, 0x008, 26),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTM_SEL, "camtm_sel", camtm_parents, 0x0f0, 0x0f4, 0x0f8, 16,
			     2, 23, 0x008, 27),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_PWM_SEL, "pwm_sel", pwm_parents, 0x0f0, 0x0f4, 0x0f8, 24, 1,
			     31, 0x008, 28),
	/* CLK_CFG_15 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUDIO_H_SEL, "audio_h_sel", audio_h_parents, 0x100, 0x104,
			     0x108, 0, 2, 7, 0x008, 29),
	MUX_CLR_SET_UPD(CLK_TOP_SPMI_MST_SEL, "spmi_mst_sel", spmi_mst_parents, 0x100, 0x104, 0x108,
			8, 3, 0x008, 30),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_DVFSRC_SEL, "dvfsrc_sel", dvfsrc_parents, 0x100, 0x104, 0x108,
			      16, 1, 0x00c, 0, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AES_MSDCFDE_SEL, "aes_msdcfde_sel", aes_msdcfde_parents, 0x100,
			     0x104, 0x108, 24, 3, 31, 0x00c, 1),
	/* CLK_CFG_16 */
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_MCUPM_SEL, "mcupm_sel", mcupm_parents, 0x110, 0x114, 0x118, 0,
			      2, 0x00c, 2, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SFLASH_SEL, "sflash_sel", sflash_parents, 0x110, 0x114, 0x118,
			     8, 2, 15, 0x00c, 3),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DSI_OCC_SEL, "dsi_occ_sel", dsi_occ_parents, 0x110, 0x114,
			     0x118, 16, 2, 23, 0x00c, 4),
};

static const struct mtk_composite top_muxes[] = {
	MUX(CLK_TOP_APLL_I2S0_MCK_SEL, "apll_i2s0_mck_sel", apll_i2s_mck_parents, 0x320, 16, 1),
	MUX(CLK_TOP_APLL_I2S1_MCK_SEL, "apll_i2s1_mck_sel", apll_i2s_mck_parents, 0x320, 17, 1),
	MUX(CLK_TOP_APLL_I2S2_MCK_SEL, "apll_i2s2_mck_sel", apll_i2s_mck_parents, 0x320, 18, 1),
	MUX(CLK_TOP_APLL_I2S3_MCK_SEL, "apll_i2s3_mck_sel", apll_i2s_mck_parents, 0x320, 19, 1),
	MUX(CLK_TOP_APLL_I2S4_MCK_SEL, "apll_i2s4_mck_sel", apll_i2s_mck_parents, 0x320, 20, 1),
	MUX(CLK_TOP_APLL_I2S5_MCK_SEL, "apll_i2s5_mck_sel", apll_i2s_mck_parents, 0x320, 21, 1),
	MUX(CLK_TOP_APLL_I2S6_MCK_SEL, "apll_i2s6_mck_sel", apll_i2s_mck_parents, 0x320, 22, 1),
	MUX(CLK_TOP_APLL_I2S7_MCK_SEL, "apll_i2s7_mck_sel", apll_i2s_mck_parents, 0x320, 23, 1),
	MUX(CLK_TOP_APLL_I2S8_MCK_SEL, "apll_i2s8_mck_sel", apll_i2s_mck_parents, 0x320, 24, 1),
	MUX(CLK_TOP_APLL_I2S9_MCK_SEL, "apll_i2s9_mck_sel", apll_i2s_mck_parents, 0x320, 25, 1),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV0, "apll12_div0", "apll_i2s0_mck_sel", 0x320, 0, 0x328, 8, 0),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV1, "apll12_div1", "apll_i2s1_mck_sel", 0x320, 1, 0x328, 8, 8),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV2, "apll12_div2", "apll_i2s2_mck_sel", 0x320, 2, 0x328, 8,
		 16),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV3, "apll12_div3", "apll_i2s3_mck_sel", 0x320, 3, 0x328, 8,
		 24),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV4, "apll12_div4", "apll_i2s4_mck_sel", 0x320, 4, 0x334, 8, 0),
	DIV_GATE(CLK_TOP_APLL12_CK_DIVB, "apll12_divb", "apll12_div4", 0x320, 5, 0x334, 8, 8),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV5, "apll12_div5", "apll_i2s5_mck_sel", 0x320, 6, 0x334, 8,
		 16),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV6, "apll12_div6", "apll_i2s6_mck_sel", 0x320, 7, 0x334, 8,
		 24),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV7, "apll12_div7", "apll_i2s7_mck_sel", 0x320, 8, 0x338, 8, 0),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV8, "apll12_div8", "apll_i2s8_mck_sel", 0x320, 9, 0x338, 8, 8),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV9, "apll12_div9", "apll_i2s9_mck_sel", 0x320, 10, 0x338, 8,
		 16),
};

static const struct mtk_clk_desc topck_desc = {
	.fixed_clks = top_fixed_clks,
	.num_fixed_clks = ARRAY_SIZE(top_fixed_clks),
	.factor_clks = top_divs,
	.num_factor_clks = ARRAY_SIZE(top_divs),
	.mux_clks = top_mtk_muxes,
	.num_mux_clks = ARRAY_SIZE(top_mtk_muxes),
	.composite_clks = top_muxes,
	.num_composite_clks = ARRAY_SIZE(top_muxes),
	.clk_lock = &mt6833_clk_lock,
};

static const struct of_device_id of_match_clk_mt6833_topck[] = {
	{ .compatible = "mediatek,mt6833-topckgen", .data = &topck_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_topck);

static struct platform_driver clk_mt6833_topck_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6833-topck",
		.of_match_table = of_match_clk_mt6833_topck,
	},
};
module_platform_driver(clk_mt6833_topck_drv);

MODULE_DESCRIPTION("MediaTek MT6833 topckgen clocks driver");
MODULE_LICENSE("GPL");
