// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"
#include "clk-mux.h"

static DEFINE_SPINLOCK(mt6833_clk_lock);

static const struct mtk_fixed_factor top_divs[] = {
	FACTOR(CLK_TOP_MFGPLL, "mfgpll_ck", "mfgpll", 1, 1),
	FACTOR(CLK_TOP_MAINPLL, "mainpll_ck", "mainpll", 1, 1),
	FACTOR(CLK_TOP_MAINPLL_D3, "mainpll_d3", "mainpll", 1, 3),
	FACTOR(CLK_TOP_MAINPLL_D4, "mainpll_d4", "mainpll", 1, 4),
	FACTOR(CLK_TOP_MAINPLL_D4_D2, "mainpll_d4_d2", "mainpll", 1, 8),
	FACTOR(CLK_TOP_MAINPLL_D4_D4, "mainpll_d4_d4", "mainpll", 1, 16),
	FACTOR(CLK_TOP_MAINPLL_D4_D8, "mainpll_d4_d8", "mainpll", 43, 1375),
	FACTOR(CLK_TOP_MAINPLL_D4_D16, "mainpll_d4_d16", "mainpll", 64, 4099),
	FACTOR(CLK_TOP_MAINPLL_D5, "mainpll_d5", "mainpll", 1, 5),
	FACTOR(CLK_TOP_MAINPLL_D5_D2, "mainpll_d5_d2", "mainpll", 1, 10),
	FACTOR(CLK_TOP_MAINPLL_D5_D4, "mainpll_d5_d4", "mainpll", 1, 20),
	FACTOR(CLK_TOP_MAINPLL_D5_D8, "mainpll_d5_d8", "mainpll", 1, 40),
	FACTOR(CLK_TOP_MAINPLL_D6, "mainpll_d6", "mainpll", 1, 6),
	FACTOR(CLK_TOP_MAINPLL_D6_D2, "mainpll_d6_d2", "mainpll", 1, 12),
	FACTOR(CLK_TOP_MAINPLL_D6_D4, "mainpll_d6_d4", "mainpll", 1, 24),
	FACTOR(CLK_TOP_MAINPLL_D6_D8, "mainpll_d6_d8", "mainpll", 1, 48),
	FACTOR(CLK_TOP_MAINPLL_D7, "mainpll_d7", "mainpll", 1, 7),
	FACTOR(CLK_TOP_MAINPLL_D7_D2, "mainpll_d7_d2", "mainpll", 1, 14),
	FACTOR(CLK_TOP_MAINPLL_D7_D4, "mainpll_d7_d4", "mainpll", 1, 28),
	FACTOR(CLK_TOP_MAINPLL_D7_D8, "mainpll_d7_d8", "mainpll", 1, 56),
	FACTOR(CLK_TOP_MAINPLL_D9, "mainpll_d9", "mainpll", 1, 9),
	FACTOR(CLK_TOP_UNIVPLL, "univpll_ck", "univpll", 1, 1),
	FACTOR(CLK_TOP_UNIVPLL_D2, "univpll_d2", "univpll", 1, 2),
	FACTOR(CLK_TOP_UNIVPLL_D3, "univpll_d3", "univpll", 1, 3),
	FACTOR(CLK_TOP_UNIVPLL_D4, "univpll_d4", "univpll", 1, 4),
	FACTOR(CLK_TOP_UNIVPLL_D4_D2, "univpll_d4_d2", "univpll", 1, 8),
	FACTOR(CLK_TOP_UNIVPLL_D4_D4, "univpll_d4_d4", "univpll", 1, 16),
	FACTOR(CLK_TOP_UNIVPLL_D4_D8, "univpll_d4_d8", "univpll", 1, 32),
	FACTOR(CLK_TOP_UNIVPLL_D5, "univpll_d5", "univpll", 1, 5),
	FACTOR(CLK_TOP_UNIVPLL_D5_D2, "univpll_d5_d2", "univpll", 1, 10),
	FACTOR(CLK_TOP_UNIVPLL_D5_D4, "univpll_d5_d4", "univpll", 1, 20),
	FACTOR(CLK_TOP_UNIVPLL_D5_D8, "univpll_d5_d8", "univpll", 1, 40),
	FACTOR(CLK_TOP_UNIVPLL_D5_D16, "univpll_d5_d16", "univpll", 1, 80),
	FACTOR(CLK_TOP_UNIVPLL_D6, "univpll_d6", "univpll", 1, 6),
	FACTOR(CLK_TOP_UNIVPLL_D6_D2, "univpll_d6_d2", "univpll", 1, 12),
	FACTOR(CLK_TOP_UNIVPLL_D6_D4, "univpll_d6_d4", "univpll", 1, 24),
	FACTOR(CLK_TOP_UNIVPLL_D6_D8, "univpll_d6_d8", "univpll", 1, 48),
	FACTOR(CLK_TOP_UNIVPLL_D6_D16, "univpll_d6_d16", "univpll", 1, 96),
	FACTOR(CLK_TOP_UNIVPLL_D7, "univpll_d7", "univpll", 1, 7),
	FACTOR(CLK_TOP_UNIVPLL_D7_D2, "univpll_d7_d2", "univpll", 1, 14),
	FACTOR(CLK_TOP_UNIVPLL_192M_D2, "univpll_192m_d2", "univpll", 1, 26),
	FACTOR(CLK_TOP_UNIVPLL_192M_D4, "univpll_192m_d4", "univpll", 1, 52),
	FACTOR(CLK_TOP_UNIVPLL_192M_D8, "univpll_192m_d8", "univpll", 1, 104),
	FACTOR(CLK_TOP_UNIVPLL_192M_D16, "univpll_192m_d16", "univpll", 1, 208),
	FACTOR(CLK_TOP_UNIVPLL_192M_D32, "univpll_192m_d32", "univpll", 1, 416),
	FACTOR(CLK_TOP_USB20_192M, "usb20_192m_ck", "univpll", 1, 13),
	FACTOR(CLK_TOP_USB20_PLL_D2, "usb20_pll_d2", "univpll", 1, 26),
	FACTOR(CLK_TOP_USB20_PLL_D4, "usb20_pll_d4", "univpll", 1, 52),
	FACTOR(CLK_TOP_MPLL_208M, "mpll_208m_ck", "mpll", 1, 1),
	FACTOR(CLK_TOP_MPLL_D2, "mpll_d2", "mpll", 1, 2),
	FACTOR(CLK_TOP_MPLL_D4, "mpll_d4", "mpll", 1, 4),
	FACTOR(CLK_TOP_APLL1, "apll1_ck", "apll1", 1, 1),
	FACTOR(CLK_TOP_APLL1_D2, "apll1_d2", "apll1", 1, 2),
	FACTOR(CLK_TOP_APLL1_D4, "apll1_d4", "apll1", 1, 4),
	FACTOR(CLK_TOP_APLL1_D8, "apll1_d8", "apll1", 1, 8),
	FACTOR(CLK_TOP_APLL2, "apll2_ck", "apll2", 1, 1),
	FACTOR(CLK_TOP_APLL2_D2, "apll2_d2", "apll2", 1, 2),
	FACTOR(CLK_TOP_APLL2_D4, "apll2_d4", "apll2", 1, 4),
	FACTOR(CLK_TOP_APLL2_D8, "apll2_d8", "apll2", 1, 8),
	FACTOR(CLK_TOP_MMPLL_D3, "mmpll_d3", "mmpll", 1, 3),
	FACTOR(CLK_TOP_MMPLL_D4, "mmpll_d4", "mmpll", 1, 4),
	FACTOR(CLK_TOP_MMPLL_D4_D2, "mmpll_d4_d2", "mmpll", 1, 8),
	FACTOR(CLK_TOP_MMPLL_D4_D4, "mmpll_d4_d4", "mmpll", 1, 16),
	FACTOR(CLK_TOP_MMPLL_D5, "mmpll_d5", "mmpll", 1, 5),
	FACTOR(CLK_TOP_MMPLL_D5_D2, "mmpll_d5_d2", "mmpll", 1, 10),
	FACTOR(CLK_TOP_MMPLL_D5_D4, "mmpll_d5_d4", "mmpll", 1, 20),
	FACTOR(CLK_TOP_MMPLL_D6, "mmpll_d6", "mmpll", 1, 6),
	FACTOR(CLK_TOP_MMPLL_D6_D2, "mmpll_d6_d2", "mmpll", 1, 12),
	FACTOR(CLK_TOP_MMPLL_D7, "mmpll_d7", "mmpll", 1, 7),
	FACTOR(CLK_TOP_MMPLL_D9, "mmpll_d9", "mmpll", 1, 9),
	FACTOR(CLK_TOP_NPUPLL, "npupll_ck", "npupll", 1, 1),
	FACTOR(CLK_TOP_TVDPLL, "tvdpll_ck", "tvdpll", 1, 1),
	FACTOR(CLK_TOP_TVDPLL_D2, "tvdpll_d2", "tvdpll", 1, 2),
	FACTOR(CLK_TOP_TVDPLL_D4, "tvdpll_d4", "tvdpll", 1, 4),
	FACTOR(CLK_TOP_TVDPLL_D8, "tvdpll_d8", "tvdpll", 1, 8),
	FACTOR(CLK_TOP_TVDPLL_D16, "tvdpll_d16", "tvdpll", 92, 1473),
	FACTOR(CLK_TOP_MSDCPLL, "msdcpll_ck", "msdcpll", 1, 1),
	FACTOR(CLK_TOP_MSDCPLL_D2, "msdcpll_d2", "msdcpll", 1, 2),
	FACTOR(CLK_TOP_MSDCPLL_D4, "msdcpll_d4", "msdcpll", 1, 4),
	FACTOR(CLK_TOP_MSDCPLL_D8, "msdcpll_d8", "msdcpll", 1, 8),
	FACTOR(CLK_TOP_MSDCPLL_D16, "msdcpll_d16", "msdcpll", 1, 16),
	FACTOR(CLK_TOP_CLKRTC, "clkrtc", "clk32k", 1, 1),
	FACTOR(CLK_TOP_TCK_26M_MX8, "tck_26m_mx8_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_TCK_26M_MX9, "tck_26m_mx9_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_TCK_26M_MX10, "tck_26m_mx10_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_TCK_26M_MX11, "tck_26m_mx11_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_TCK_26M_MX12, "tck_26m_mx12_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_CSW_FAXI, "csw_faxi_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_F26M_CK_D52, "f26m_d52", "clk26m", 1, 1),
	FACTOR(CLK_TOP_F26M_CK_D2, "f26m_d2", "clk26m", 1, 2),
	FACTOR(CLK_TOP_OSC, "osc_ck", "ulposc", 1, 1),
	FACTOR(CLK_TOP_OSC_D2, "osc_d2", "ulposc", 1, 2),
	FACTOR(CLK_TOP_OSC_D4, "osc_d4", "ulposc", 1, 4),
	FACTOR(CLK_TOP_OSC_D8, "osc_d8", "ulposc", 1, 8),
	FACTOR(CLK_TOP_OSC_D16, "osc_d16", "ulposc", 61, 973),
	FACTOR(CLK_TOP_OSC_D10, "osc_d10", "ulposc", 1, 10),
	FACTOR(CLK_TOP_OSC_D20, "osc_d20", "ulposc", 1, 20),
	FACTOR(CLK_TOP_ULPOSC, "ulposc_ck", "ulposc", 1, 1),
	FACTOR(CLK_TOP_F26M, "f26m_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_RTC, "rtc_ck", "clk32k", 1, 1),
	FACTOR(CLK_TOP_AXI, "axi_ck", "axi_sel", 1, 1),
	FACTOR(CLK_TOP_SPM, "spm_ck", "spm_sel", 1, 1),
	FACTOR(CLK_TOP_SCP, "scp_ck", "scp_sel", 1, 1),
	FACTOR(CLK_TOP_BUS, "bus_ck", "bus_aximem_sel", 1, 1),
	FACTOR(CLK_TOP_DISP, "disp_ck", "disp_sel", 1, 1),
	FACTOR(CLK_TOP_MDP, "mdp_ck", "mdp_sel", 1, 1),
	FACTOR(CLK_TOP_IMG1, "img1_ck", "img1_sel", 1, 1),
	FACTOR(CLK_TOP_IMG2, "img2_ck", "img2_sel", 1, 1),
	FACTOR(CLK_TOP_IPE, "ipe_ck", "ipe_sel", 1, 1),
	FACTOR(CLK_TOP_DPE, "dpe_ck", "dpe_sel", 1, 1),
	FACTOR(CLK_TOP_CAM, "cam_ck", "cam_sel", 1, 1),
	FACTOR(CLK_TOP_CCU, "ccu_ck", "ccu_sel", 1, 1),
	FACTOR(CLK_TOP_MFG_REF, "mfg_ref_ck", "mfg_pll_sel", 1, 1),
	FACTOR(CLK_TOP_MFG_REF_SEL_CK, "mfg_ref_sel_ck", "mfg_ref_sel", 1, 1),
	FACTOR(CLK_TOP_CAMTG, "camtg_ck", "camtg_sel", 1, 1),
	FACTOR(CLK_TOP_CAMTG2, "camtg2_ck", "camtg2_sel", 1, 1),
	FACTOR(CLK_TOP_CAMTG3, "camtg3_ck", "camtg3_sel", 1, 1),
	FACTOR(CLK_TOP_CAMTG4, "camtg4_ck", "camtg4_sel", 1, 1),
	FACTOR(CLK_TOP_CAMTG5, "camtg5_ck", "camtg5_sel", 1, 1),
	FACTOR(CLK_TOP_UART, "uart_ck", "uart_sel", 1, 1),
	FACTOR(CLK_TOP_SPI, "spi_ck", "spi_sel", 1, 1),
	FACTOR(CLK_TOP_MSDC50_0_HCLK, "msdc5hclk_ck", "msdc5hclk_sel", 1, 1),
	FACTOR(CLK_TOP_MSDC50_0, "msdc50_0_ck", "msdc50_0_sel", 1, 1),
	FACTOR(CLK_TOP_MSDC30_1, "msdc30_1_ck", "msdc30_1_sel", 1, 1),
	FACTOR(CLK_TOP_AUDIO, "audio_ck", "audio_sel", 1, 1),
	FACTOR(CLK_TOP_AUD_INTBUS, "aud_intbus_ck", "aud_intbus_sel", 1, 1),
	FACTOR(CLK_TOP_PWRAP_ULPOSC, "pwrap_ulposc_ck", "pwrap_ulposc_sel", 1, 1),
	FACTOR(CLK_TOP_ATB, "atb_ck", "atb_sel", 1, 1),
	FACTOR(CLK_TOP_SSPM, "sspm_ck", "sspm_sel", 1, 1),
	FACTOR(CLK_TOP_SCAM, "scam_ck", "scam_sel", 1, 1),
	FACTOR(CLK_TOP_DISP_PWM, "disp_pwm_ck", "disp_pwm_sel", 1, 1),
	FACTOR(CLK_TOP_USB_TOP, "usb_ck", "usb_sel", 1, 1),
	FACTOR(CLK_TOP_USB_XHCI, "ssusb_xhci_ck", "ssusb_xhci_sel", 1, 1),
	FACTOR(CLK_TOP_I2C, "i2c_ck", "i2c_sel", 1, 1),
	FACTOR(CLK_TOP_SENINF, "seninf_ck", "seninf_sel", 1, 1),
	FACTOR(CLK_TOP_SENINF1, "seninf1_ck", "seninf1_sel", 1, 1),
	FACTOR(CLK_TOP_SENINF2, "seninf2_ck", "seninf2_sel", 1, 1),
	FACTOR(CLK_TOP_DXCC, "dxcc_ck", "dxcc_sel", 1, 1),
	FACTOR(CLK_TOP_AUD_ENGEN1, "aud_engen1_ck", "aud_engen1_sel", 1, 1),
	FACTOR(CLK_TOP_AUD_ENGEN2, "aud_engen2_ck", "aud_engen2_sel", 1, 1),
	FACTOR(CLK_TOP_AES_UFSFDE, "aes_ufsfde_ck", "aes_ufsfde_sel", 1, 1),
	FACTOR(CLK_TOP_UFS, "ufs_ck", "ufs_sel", 1, 1),
	FACTOR(CLK_TOP_AUD_1, "aud_1_ck", "aud_1_sel", 1, 1),
	FACTOR(CLK_TOP_AUD_2, "aud_2_ck", "aud_2_sel", 1, 1),
	FACTOR(CLK_TOP_ADSP, "adsp_ck", "adsp_sel", 1, 1),
	FACTOR(CLK_TOP_DPMAIF_MAIN, "dpmaif_main_ck", "dpmaif_main_sel", 1, 1),
	FACTOR(CLK_TOP_VENC, "venc_ck", "venc_sel", 1, 1),
	FACTOR(CLK_TOP_VDEC, "vdec_ck", "vdec_sel", 1, 1),
	FACTOR(CLK_TOP_CAMTM, "camtm_ck", "camtm_sel", 1, 1),
	FACTOR(CLK_TOP_PWM, "pwm_ck", "pwm_sel", 1, 1),
	FACTOR(CLK_TOP_AUDIO_H, "audio_h_ck", "audio_h_sel", 1, 1),
	FACTOR(CLK_TOP_SPMI_MST, "spmi_mst_ck", "spmi_mst_sel", 1, 1),
	FACTOR(CLK_TOP_DVFSRC, "dvfsrc_ck", "dvfsrc_sel", 1, 1),
	FACTOR(CLK_TOP_AES_MSDCFDE, "aes_msdcfde_ck", "aes_msdcfde_sel", 1, 1),
	FACTOR(CLK_TOP_MCUPM, "mcupm_ck", "mcupm_sel", 1, 1),
	FACTOR(CLK_TOP_SFLASH, "sflash_ck", "sflash_sel", 1, 1),
	FACTOR(CLK_TOP_DSI_OCC, "dsi_occ_ck", "dsi_occ_sel", 1, 1),
	FACTOR(CLK_TOP_SPMI_MST_32K, "spmi_mst_32k_ck", "clkrtc", 1, 1),
	FACTOR(CLK_TOP_SYS_26M, "sys_26m_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_UFS_SAP_CFG, "ufs_cfg_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_UFS_TICK1US, "f_u_tick1us_ck", "clk26m", 1, 1),
	FACTOR(CLK_TOP_I2C_PSEUDO, "i2c_pseudo_ck", "ifrao_i2c_pseudo", 1, 1),
	FACTOR(CLK_TOP_APDMA_PSEUDO, "apdma_pseudo", "ifrao_i2c1_pseudo", 1, 1),
};

static const char * const axi_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d4",
	"mainpll_d7_d2",
	"mainpll_d4_d2",
	"mainpll_d5_d2",
	"mainpll_d6_d2",
	"osc_d4",
};

static const char * const spm_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10",
	"mainpll_d7_d4",
	"clkrtc",
};

static const char * const scp_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4",
	"npupll_ck",
	"mainpll_d6",
	"univpll_d6",
	"mainpll_d4_d2",
	"mainpll_d4",
	"mainpll_d7",
};

static const char * const bus_aximem_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d7_d2",
	"mainpll_d4_d2",
	"mainpll_d5_d2",
	"mainpll_d6",
};

static const char * const disp_parents[] = {
	"tck_26m_mx9_ck",
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
	"tck_26m_mx9_ck",
	"mainpll_d5_d2",
	"mmpll_d6_d2",
	"mainpll_d4_d2",
	"mmpll_d4_d2",
	"mainpll_d6",
	"mainpll_d5",
	"mainpll_d4",
	"tvdpll_ck",
	"univpll_d4",
	"mmpll_d5_d2",
};

static const char * const img1_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4",
	"tvdpll_ck",
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

static const char * const img2_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4",
	"tvdpll_ck",
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
	"tck_26m_mx9_ck",
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
	"tck_26m_mx9_ck",
	"mainpll_d4",
	"mmpll_d6",
	"univpll_d6",
	"mainpll_d6",
	"univpll_d4_d2",
	"univpll_d5_d2",
	"mmpll_d6_d2",
};

static const char * const cam_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4",
	"mmpll_d6",
	"univpll_d4",
	"univpll_d5",
	"univpll_d6",
	"mmpll_d7",
	"univpll_d4_d2",
	"mainpll_d4_d2",
	"npupll_ck",
};

static const char * const ccu_parents[] = {
	"tck_26m_mx9_ck",
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
	"tck_26m_mx9_ck",
	"mainpll_d6_d2",
	"mainpll_d6",
	"mainpll_d5_d2",
};

static const char * const mfg_pll_parents[] = {
	"mfg_ref_sel_ck",
	"mfgpll_ck",
};

static const char * const camtg_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_192m_d8",
	"univpll_d6_d8",
	"univpll_192m_d4",
	"univpll_d6_d16",
	"f26m_d2",
	"univpll_192m_d16",
	"univpll_192m_d32",
};

static const char * const camtg2_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_192m_d8",
	"univpll_d6_d8",
	"univpll_192m_d4",
	"univpll_d6_d16",
	"f26m_d2",
	"univpll_192m_d16",
	"univpll_192m_d32",
};

static const char * const camtg3_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_192m_d8",
	"univpll_d6_d8",
	"univpll_192m_d4",
	"univpll_d6_d16",
	"f26m_d2",
	"univpll_192m_d16",
	"univpll_192m_d32",
};

static const char * const camtg4_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_192m_d8",
	"univpll_d6_d8",
	"univpll_192m_d4",
	"univpll_d6_d16",
	"f26m_d2",
	"univpll_192m_d16",
	"univpll_192m_d32",
};

static const char * const camtg5_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_192m_d8",
	"univpll_d6_d8",
	"univpll_192m_d4",
	"univpll_d6_d16",
	"f26m_d2",
	"univpll_192m_d16",
	"univpll_192m_d32",
};

static const char * const uart_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d6_d8",
};

static const char * const spi_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d4",
	"mainpll_d6_d4",
	"msdcpll_d4",
	"msdcpll_d2",
	"mainpll_d6_d2",
	"mainpll_d4_d4",
	"univpll_d5_d4",
};

static const char * const msdc5hclk_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d2",
	"mainpll_d6_d2",
};

static const char * const msdc50_0_parents[] = {
	"tck_26m_mx9_ck",
	"msdcpll_ck",
	"msdcpll_d2",
	"univpll_d4_d4",
	"mainpll_d6_d2",
	"univpll_d4_d2",
};

static const char * const msdc30_1_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d6_d2",
	"mainpll_d6_d2",
	"mainpll_d7_d2",
	"msdcpll_d2",
};

static const char * const audio_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d8",
	"mainpll_d7_d8",
	"mainpll_d4_d16",
};

static const char * const aud_intbus_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d4",
	"mainpll_d7_d4",
};

static const char * const pwrap_ulposc_parents[] = {
	"osc_d10",
	"tck_26m_mx9_ck",
	"osc_d4",
	"osc_d8",
	"osc_d16",
};

static const char * const atb_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d2",
	"mainpll_d5_d2",
};

static const char * const sspm_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d7_d2",
	"mainpll_d6_d2",
	"mainpll_d5_d2",
	"mainpll_d9",
	"mainpll_d4_d2",
};

static const char * const scam_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d5_d4",
};

static const char * const disp_pwm_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d6_d4",
	"osc_d2",
	"osc_d4",
	"osc_d16",
};

static const char * const usb_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d5_d4",
	"univpll_d6_d4",
	"univpll_d5_d2",
};

static const char * const ssusb_xhci_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d5_d4",
	"univpll_d6_d4",
	"univpll_d5_d2",
};

static const char * const i2c_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d8",
	"univpll_d5_d4",
};

static const char * const seninf_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4_d4",
	"univpll_d6_d2",
	"univpll_d4_d2",
	"npupll_ck",
	"mmpll_d7",
	"mmpll_d6",
	"univpll_d5",
};

static const char * const seninf1_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4_d4",
	"univpll_d6_d2",
	"univpll_d4_d2",
	"npupll_ck",
	"mmpll_d7",
	"mmpll_d6",
	"univpll_d5",
};

static const char * const seninf2_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4_d4",
	"univpll_d6_d2",
	"univpll_d4_d2",
	"npupll_ck",
	"mmpll_d7",
	"mmpll_d6",
	"univpll_d5",
};

static const char * const dxcc_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d2",
	"mainpll_d4_d4",
	"mainpll_d4_d8",
};

static const char * const aud_engen1_parents[] = {
	"tck_26m_mx9_ck",
	"apll1_d2",
	"apll1_d4",
	"apll1_d8",
};

static const char * const aud_engen2_parents[] = {
	"tck_26m_mx9_ck",
	"apll2_d2",
	"apll2_d4",
	"apll2_d8",
};

static const char * const aes_ufsfde_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4",
	"mainpll_d4_d2",
	"mainpll_d6",
	"mainpll_d4_d4",
	"univpll_d4_d2",
	"univpll_d6",
};

static const char * const ufs_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d4",
	"mainpll_d4_d8",
	"univpll_d4_d4",
	"mainpll_d6_d2",
	"mainpll_d5_d2",
	"msdcpll_d2",
};

static const char * const aud_1_parents[] = {
	"tck_26m_mx9_ck",
	"apll1_ck",
};

static const char * const aud_2_parents[] = {
	"tck_26m_mx9_ck",
	"apll2_ck",
};

static const char * const adsp_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d6",
	"mainpll_d5_d2",
	"univpll_d4_d4",
	"univpll_d4",
	"univpll_d6",
	"ulposc_ck",
	"npupll_ck",
};

static const char * const dpmaif_main_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4_d4",
	"mainpll_d6",
	"mainpll_d4_d2",
	"univpll_d4_d2",
};

static const char * const venc_parents[] = {
	"tck_26m_mx9_ck",
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
	"tck_26m_mx9_ck",
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
	"tck_26m_mx9_ck",
	"univpll_d7",
	"univpll_d6_d2",
	"univpll_d4_d2",
};

static const char * const pwm_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d4_d8",
};

static const char * const audio_h_parents[] = {
	"tck_26m_mx9_ck",
	"univpll_d7",
	"apll1_ck",
	"apll2_ck",
};

static const char * const spmi_mst_parents[] = {
	"tck_26m_mx9_ck",
	"f26m_d2",
	"osc_d8",
	"osc_d10",
	"osc_d16",
	"osc_d20",
	"clkrtc",
};

static const char * const dvfsrc_parents[] = {
	"tck_26m_mx9_ck",
	"osc_d10",
};

static const char * const aes_msdcfde_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d4_d2",
	"mainpll_d6",
	"mainpll_d4_d4",
	"univpll_d4_d2",
	"univpll_d6",
};

static const char * const mcupm_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d6_d4",
	"mainpll_d6_d2",
};

static const char * const sflash_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d7_d8",
	"univpll_d6_d8",
	"univpll_d5_d8",
};

static const char * const dsi_occ_parents[] = {
	"tck_26m_mx9_ck",
	"mainpll_d6_d2",
	"univpll_d5_d2",
	"univpll_d4_d2",
};

static const char * const apll_i2s0_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s1_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s2_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s3_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s4_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s5_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s6_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s7_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s8_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

static const char * const apll_i2s9_mck_parents[] = {
	"aud_1_sel",
	"aud_2_sel",
};

/*
 * CRITICAL CLOCK:
 * axi_sel and bus_aximem_sel clock the SoC's main bus and the bus to the EMI,
 * spm_sel, sspm_sel, mcupm_sel, dvfsrc_sel and pwrap_ulposc_sel the always-on
 * coprocessors, the DVFS resource collector and the PMIC wrapper.
 */
static const struct mtk_mux top_mtk_muxes[] = {
	GATE_CLR_SET_UPD_FLAGS(CLK_TOP_AXI_SEL, "axi_sel", axi_parents, 0x010, 0x014, 0x018, 0, 3,
			       0, 0x004, 0, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
			       mtk_mux_clr_set_upd_ops),
	GATE_CLR_SET_UPD_FLAGS(CLK_TOP_SPM_SEL, "spm_sel", spm_parents, 0x010, 0x014, 0x018, 8, 2,
			       0, 0x004, 1, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
			       mtk_mux_clr_set_upd_ops),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SCP_SEL, "scp_sel", scp_parents, 0x010, 0x014, 0x018, 16, 3,
			     23, 0x004, 2),
	GATE_CLR_SET_UPD_FLAGS(CLK_TOP_BUS_AXIMEM_SEL, "bus_aximem_sel", bus_aximem_parents, 0x010,
			       0x014, 0x018, 24, 3, 0, 0x004, 3,
			       CLK_IS_CRITICAL | CLK_SET_RATE_PARENT, mtk_mux_clr_set_upd_ops),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DISP_SEL, "disp_sel", disp_parents, 0x020, 0x024, 0x028, 0, 4,
			     7, 0x004, 4),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MDP_SEL, "mdp_sel", mdp_parents, 0x020, 0x024, 0x028, 8, 4, 15,
			     0x004, 5),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IMG1_SEL, "img1_sel", img1_parents, 0x020, 0x024, 0x028, 16, 4,
			     23, 0x004, 6),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IMG2_SEL, "img2_sel", img2_parents, 0x020, 0x024, 0x028, 24, 4,
			     31, 0x004, 7),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IPE_SEL, "ipe_sel", ipe_parents, 0x030, 0x034, 0x038, 0, 4, 7,
			     0x004, 8),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DPE_SEL, "dpe_sel", dpe_parents, 0x030, 0x034, 0x038, 8, 3, 15,
			     0x004, 9),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAM_SEL, "cam_sel", cam_parents, 0x030, 0x034, 0x038, 16, 4,
			     23, 0x004, 10),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CCU_SEL, "ccu_sel", ccu_parents, 0x030, 0x034, 0x038, 24, 4,
			     31, 0x004, 11),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MFG_REF_SEL, "mfg_ref_sel", mfg_ref_parents, 0x050, 0x054,
			     0x058, 16, 2, 23, 0x004, 18),
	MUX_CLR_SET(CLK_TOP_MFG_PLL_SEL, "mfg_pll_sel", mfg_pll_parents, 0x050, 0x054, 0x058, 18,
		    1),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG_SEL, "camtg_sel", camtg_parents, 0x050, 0x054, 0x058, 24,
			     3, 31, 0x004, 19),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG2_SEL, "camtg2_sel", camtg2_parents, 0x060, 0x064, 0x068,
			     0, 3, 7, 0x004, 20),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG3_SEL, "camtg3_sel", camtg3_parents, 0x060, 0x064, 0x068,
			     8, 3, 15, 0x004, 21),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG4_SEL, "camtg4_sel", camtg4_parents, 0x060, 0x064, 0x068,
			     16, 3, 23, 0x004, 22),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTG5_SEL, "camtg5_sel", camtg5_parents, 0x060, 0x064, 0x068,
			     24, 3, 31, 0x004, 23),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_UART_SEL, "uart_sel", uart_parents, 0x070, 0x074, 0x078, 8, 1,
			     15, 0x004, 25),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SPI_SEL, "spi_sel", spi_parents, 0x070, 0x074, 0x078, 16, 3,
			     23, 0x004, 26),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC50_0_HCLK_SEL, "msdc5hclk_sel", msdc5hclk_parents, 0x070,
			     0x074, 0x078, 24, 2, 31, 0x004, 27),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC50_0_SEL, "msdc50_0_sel", msdc50_0_parents, 0x080, 0x084,
			     0x088, 0, 3, 7, 0x004, 28),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC30_1_SEL, "msdc30_1_sel", msdc30_1_parents, 0x080, 0x084,
			     0x088, 8, 3, 15, 0x004, 29),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUDIO_SEL, "audio_sel", audio_parents, 0x080, 0x084, 0x088, 24,
			     2, 31, 0x008, 0),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_INTBUS_SEL, "aud_intbus_sel", aud_intbus_parents, 0x090,
			     0x094, 0x098, 0, 2, 7, 0x008, 1),
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_PWRAP_ULPOSC_SEL, "pwrap_ulposc_sel",
				   pwrap_ulposc_parents, 0x090, 0x094, 0x098, 8, 3, 15, 0x008, 2,
				   CLK_IS_CRITICAL | CLK_SET_RATE_PARENT),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ATB_SEL, "atb_sel", atb_parents, 0x090, 0x094, 0x098, 16, 2,
			     23, 0x008, 3),
	GATE_CLR_SET_UPD_FLAGS(CLK_TOP_SSPM_SEL, "sspm_sel", sspm_parents, 0x090, 0x094, 0x098, 24,
			       3, 0, 0x008, 4, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
			       mtk_mux_clr_set_upd_ops),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SCAM_SEL, "scam_sel", scam_parents, 0x0a0, 0x0a4, 0x0a8, 8, 1,
			     15, 0x008, 6),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DISP_PWM_SEL, "disp_pwm_sel", disp_pwm_parents, 0x0a0, 0x0a4,
			     0x0a8, 16, 3, 23, 0x008, 7),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_USB_TOP_SEL, "usb_sel", usb_parents, 0x0a0, 0x0a4, 0x0a8, 24,
			     2, 31, 0x008, 8),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_USB_XHCI_SEL, "ssusb_xhci_sel", ssusb_xhci_parents, 0x0b0,
			     0x0b4, 0x0b8, 0, 2, 7, 0x008, 9),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_I2C_SEL, "i2c_sel", i2c_parents, 0x0b0, 0x0b4, 0x0b8, 8, 2, 15,
			     0x008, 10),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SENINF_SEL, "seninf_sel", seninf_parents, 0x0b0, 0x0b4, 0x0b8,
			     16, 3, 23, 0x008, 11),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SENINF1_SEL, "seninf1_sel", seninf1_parents, 0x0b0, 0x0b4,
			     0x0b8, 24, 3, 31, 0x008, 12),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SENINF2_SEL, "seninf2_sel", seninf2_parents, 0x0c0, 0x0c4,
			     0x0c8, 0, 3, 7, 0x008, 13),
	MUX_CLR_SET_UPD(CLK_TOP_DXCC_SEL, "dxcc_sel", dxcc_parents, 0x0c0, 0x0c4, 0x0c8, 24, 2,
			0x008, 16),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_ENGEN1_SEL, "aud_engen1_sel", aud_engen1_parents, 0x0d0,
			     0x0d4, 0x0d8, 0, 2, 7, 0x008, 17),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_ENGEN2_SEL, "aud_engen2_sel", aud_engen2_parents, 0x0d0,
			     0x0d4, 0x0d8, 8, 2, 15, 0x008, 18),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AES_UFSFDE_SEL, "aes_ufsfde_sel", aes_ufsfde_parents, 0x0d0,
			     0x0d4, 0x0d8, 16, 3, 23, 0x008, 19),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_UFS_SEL, "ufs_sel", ufs_parents, 0x0d0, 0x0d4, 0x0d8, 24, 3,
			     31, 0x008, 20),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_1_SEL, "aud_1_sel", aud_1_parents, 0x0e0, 0x0e4, 0x0e8, 0,
			     1, 7, 0x008, 21),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_2_SEL, "aud_2_sel", aud_2_parents, 0x0e0, 0x0e4, 0x0e8, 8,
			     1, 15, 0x008, 22),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ADSP_SEL, "adsp_sel", adsp_parents, 0x0e0, 0x0e4, 0x0e8, 16, 3,
			     23, 0x008, 23),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DPMAIF_MAIN_SEL, "dpmaif_main_sel", dpmaif_main_parents, 0x0e0,
			     0x0e4, 0x0e8, 24, 3, 31, 0x008, 24),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_VENC_SEL, "venc_sel", venc_parents, 0x0f0, 0x0f4, 0x0f8, 0, 4,
			     7, 0x008, 25),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_VDEC_SEL, "vdec_sel", vdec_parents, 0x0f0, 0x0f4, 0x0f8, 8, 4,
			     15, 0x008, 26),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_CAMTM_SEL, "camtm_sel", camtm_parents, 0x0f0, 0x0f4, 0x0f8, 16,
			     2, 23, 0x008, 27),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_PWM_SEL, "pwm_sel", pwm_parents, 0x0f0, 0x0f4, 0x0f8, 24, 1,
			     31, 0x008, 28),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUDIO_H_SEL, "audio_h_sel", audio_h_parents, 0x100, 0x104,
			     0x108, 0, 2, 7, 0x008, 29),
	MUX_CLR_SET_UPD(CLK_TOP_SPMI_MST_SEL, "spmi_mst_sel", spmi_mst_parents, 0x100, 0x104, 0x108,
			8, 3, 0x008, 30),
	GATE_CLR_SET_UPD_FLAGS(CLK_TOP_DVFSRC_SEL, "dvfsrc_sel", dvfsrc_parents, 0x100, 0x104,
			       0x108, 16, 1, 0, 0x00c, 0, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
			       mtk_mux_clr_set_upd_ops),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AES_MSDCFDE_SEL, "aes_msdcfde_sel", aes_msdcfde_parents, 0x100,
			     0x104, 0x108, 24, 3, 31, 0x00c, 1),
	GATE_CLR_SET_UPD_FLAGS(CLK_TOP_MCUPM_SEL, "mcupm_sel", mcupm_parents, 0x110, 0x114, 0x118,
			       0, 2, 0, 0x00c, 2, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
			       mtk_mux_clr_set_upd_ops),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SFLASH_SEL, "sflash_sel", sflash_parents, 0x110, 0x114, 0x118,
			     8, 2, 15, 0x00c, 3),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DSI_OCC_SEL, "dsi_occ_sel", dsi_occ_parents, 0x110, 0x114,
			     0x118, 16, 2, 23, 0x00c, 4),
};

static const struct mtk_composite top_muxes[] = {
	MUX(CLK_TOP_APLL_I2S0_MCK_SEL, "apll_i2s0_mck_sel", apll_i2s0_mck_parents, 0x0320, 16, 1),
	MUX(CLK_TOP_APLL_I2S1_MCK_SEL, "apll_i2s1_mck_sel", apll_i2s1_mck_parents, 0x0320, 17, 1),
	MUX(CLK_TOP_APLL_I2S2_MCK_SEL, "apll_i2s2_mck_sel", apll_i2s2_mck_parents, 0x0320, 18, 1),
	MUX(CLK_TOP_APLL_I2S3_MCK_SEL, "apll_i2s3_mck_sel", apll_i2s3_mck_parents, 0x0320, 19, 1),
	MUX(CLK_TOP_APLL_I2S4_MCK_SEL, "apll_i2s4_mck_sel", apll_i2s4_mck_parents, 0x0320, 20, 1),
	MUX(CLK_TOP_APLL_I2S5_MCK_SEL, "apll_i2s5_mck_sel", apll_i2s5_mck_parents, 0x0320, 21, 1),
	MUX(CLK_TOP_APLL_I2S6_MCK_SEL, "apll_i2s6_mck_sel", apll_i2s6_mck_parents, 0x0320, 22, 1),
	MUX(CLK_TOP_APLL_I2S7_MCK_SEL, "apll_i2s7_mck_sel", apll_i2s7_mck_parents, 0x0320, 23, 1),
	MUX(CLK_TOP_APLL_I2S8_MCK_SEL, "apll_i2s8_mck_sel", apll_i2s8_mck_parents, 0x0320, 24, 1),
	MUX(CLK_TOP_APLL_I2S9_MCK_SEL, "apll_i2s9_mck_sel", apll_i2s9_mck_parents, 0x0320, 25, 1),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV0, "apll12_div0", "apll_i2s0_mck_sel", 0x0320, 0, 0x328, 8,
		 0),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV1, "apll12_div1", "apll_i2s1_mck_sel", 0x0320, 1, 0x328, 8,
		 8),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV2, "apll12_div2", "apll_i2s2_mck_sel", 0x0320, 2, 0x328, 8,
		 16),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV3, "apll12_div3", "apll_i2s3_mck_sel", 0x0320, 3, 0x328, 8,
		 24),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV4, "apll12_div4", "apll_i2s4_mck_sel", 0x0320, 4, 0x334, 8,
		 0),
	DIV_GATE(CLK_TOP_APLL12_CK_DIVB, "apll12_divb", "apll12_div4", 0x0320, 5, 0x334, 8, 8),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV5, "apll12_div5", "apll_i2s5_mck_sel", 0x0320, 6, 0x334, 8,
		 16),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV6, "apll12_div6", "apll_i2s6_mck_sel", 0x0320, 7, 0x334, 8,
		 24),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV7, "apll12_div7", "apll_i2s7_mck_sel", 0x0320, 8, 0x338, 8,
		 0),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV8, "apll12_div8", "apll_i2s8_mck_sel", 0x0320, 9, 0x338, 8,
		 8),
	DIV_GATE(CLK_TOP_APLL12_CK_DIV9, "apll12_div9", "apll_i2s9_mck_sel", 0x0320, 10, 0x338, 8,
		 16),
};

static const struct mtk_gate_regs ifrao0_cg_regs = {
	.set_ofs = 0x80,
	.clr_ofs = 0x84,
	.sta_ofs = 0x90,
};
static const struct mtk_gate_regs ifrao1_cg_regs = {
	.set_ofs = 0x88,
	.clr_ofs = 0x8C,
	.sta_ofs = 0x94,
};
static const struct mtk_gate_regs ifrao2_cg_regs = {
	.set_ofs = 0xA4,
	.clr_ofs = 0xA8,
	.sta_ofs = 0xAC,
};
static const struct mtk_gate_regs ifrao3_cg_regs = {
	.set_ofs = 0xC0,
	.clr_ofs = 0xC4,
	.sta_ofs = 0xC8,
};
static const struct mtk_gate_regs ifrao4_cg_regs = {
	.set_ofs = 0xE0,
	.clr_ofs = 0xE4,
	.sta_ofs = 0xE8,
};
static const struct mtk_gate_regs ifrao5_cg_regs = {
	.set_ofs = 0xd0,
	.clr_ofs = 0xd4,
	.sta_ofs = 0xd8,
};

#define GATE_IFRAO0(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ifrao0_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IFRAO1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ifrao1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IFRAO2(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ifrao2_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IFRAO3(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ifrao3_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IFRAO4(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ifrao4_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}
#define GATE_IFRAO5(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ifrao5_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate ifrao_clks[] = {
	GATE_IFRAO0(CLK_IFRAO_PMIC_TMR, "ifrao_pmic_tmr", "pwrap_ulposc_ck", 0),
	GATE_IFRAO0(CLK_IFRAO_PMIC_AP, "ifrao_pmic_ap", "pwrap_ulposc_ck", 1),
	GATE_IFRAO0(CLK_IFRAO_GCE, "ifrao_gce", "axi_ck", 8),
	GATE_IFRAO0(CLK_IFRAO_GCE2, "ifrao_gce2", "axi_ck", 9),
	GATE_IFRAO0(CLK_IFRAO_THERM, "ifrao_therm", "axi_ck", 10),
	GATE_IFRAO0(CLK_IFRAO_I2C_PSEUDO, "ifrao_i2c_pseudo", "i2c_ck", 11),
	GATE_IFRAO0(CLK_IFRAO_I2C1_PSEUDO, "ifrao_i2c1_pseudo", "axi_ck", 12),
	GATE_IFRAO0(CLK_IFRAO_PWM_HCLK, "ifrao_pwm_hclk", "axi_ck", 15),
	GATE_IFRAO0(CLK_IFRAO_PWM1, "ifrao_pwm1", "pwm_ck", 16),
	GATE_IFRAO0(CLK_IFRAO_PWM2, "ifrao_pwm2", "pwm_ck", 17),
	GATE_IFRAO0(CLK_IFRAO_PWM3, "ifrao_pwm3", "pwm_ck", 18),
	GATE_IFRAO0(CLK_IFRAO_PWM4, "ifrao_pwm4", "pwm_ck", 19),
	GATE_IFRAO0(CLK_IFRAO_PWM, "ifrao_pwm", "pwm_ck", 21),
	GATE_IFRAO0(CLK_IFRAO_UART0, "ifrao_uart0", "uart_ck", 22),
	GATE_IFRAO0(CLK_IFRAO_UART1, "ifrao_uart1", "uart_ck", 23),
	GATE_IFRAO0(CLK_IFRAO_UART2, "ifrao_uart2", "uart_ck", 24),
	GATE_IFRAO0(CLK_IFRAO_UART3, "ifrao_uart3", "uart_ck", 25),
	GATE_IFRAO0(CLK_IFRAO_GCE_26M, "ifrao_gce_26m", "axi_ck", 27),
	GATE_IFRAO0(CLK_IFRAO_BTIF, "ifrao_btif", "axi_ck", 31),
	GATE_IFRAO1(CLK_IFRAO_SPI0, "ifrao_spi0", "spi_ck", 1),
	GATE_IFRAO1(CLK_IFRAO_MSDC0, "ifrao_msdc0", "axi_ck", 2),
	GATE_IFRAO1(CLK_IFRAO_MSDC1, "ifrao_msdc1", "axi_ck", 4),
	GATE_IFRAO1(CLK_IFRAO_MSDC0_SRC, "ifrao_msdc0_clk", "msdc50_0_ck", 6),
	GATE_IFRAO1(CLK_IFRAO_AUXADC, "ifrao_auxadc", "f26m_ck", 10),
	GATE_IFRAO1(CLK_IFRAO_CPUM, "ifrao_cpum", "axi_ck", 11),
	GATE_IFRAO1(CLK_IFRAO_CCIF1_AP, "ifrao_ccif1_ap", "axi_ck", 12),
	GATE_IFRAO1(CLK_IFRAO_CCIF1_MD, "ifrao_ccif1_md", "axi_ck", 13),
	GATE_IFRAO1(CLK_IFRAO_AUXADC_MD, "ifrao_auxadc_md", "f26m_ck", 14),
	GATE_IFRAO1(CLK_IFRAO_MSDC1_SRC, "ifrao_msdc1_clk", "msdc30_1_ck", 16),
	GATE_IFRAO1(CLK_IFRAO_MSDC0_AES, "ifrao_msdc0_aes_clk", "msdc50_0_ck", 17),
	GATE_IFRAO1(CLK_IFRAO_CCIF_AP, "ifrao_ccif_ap", "axi_ck", 23),
	GATE_IFRAO1(CLK_IFRAO_DEBUGSYS, "ifrao_debugsys", "axi_ck", 24),
	GATE_IFRAO1(CLK_IFRAO_AUDIO, "ifrao_audio", "axi_ck", 25),
	GATE_IFRAO1(CLK_IFRAO_CCIF_MD, "ifrao_ccif_md", "axi_ck", 26),
	GATE_IFRAO2(CLK_IFRAO_SSUSB, "ifrao_ssusb", "usb_ck", 1),
	GATE_IFRAO2(CLK_IFRAO_DISP_PWM, "ifrao_disp_pwm", "axi_ck", 2),
	GATE_IFRAO2(CLK_IFRAO_CLDMA_BCLK, "ifrao_cldmabclk", "axi_ck", 3),
	GATE_IFRAO2(CLK_IFRAO_AUDIO_26M_BCLK, "ifrao_audio26m", "f26m_ck", 4),
	GATE_IFRAO2(CLK_IFRAO_SPI1, "ifrao_spi1", "spi_ck", 6),
	GATE_IFRAO2(CLK_IFRAO_SPI2, "ifrao_spi2", "spi_ck", 9),
	GATE_IFRAO2(CLK_IFRAO_SPI3, "ifrao_spi3", "spi_ck", 10),
	GATE_IFRAO2(CLK_IFRAO_UNIPRO_SYSCLK, "ifrao_unipro_sysclk", "ufs_ck", 11),
	GATE_IFRAO2(CLK_IFRAO_UFS_SAP_BCLK, "ifrao_u_bclk", "f26m_ck", 13),
	GATE_IFRAO2(CLK_IFRAO_SPI4, "ifrao_spi4", "spi_ck", 25),
	GATE_IFRAO2(CLK_IFRAO_SPI5, "ifrao_spi5", "spi_ck", 26),
	GATE_IFRAO2(CLK_IFRAO_CQ_DMA, "ifrao_cq_dma", "axi_ck", 27),
	GATE_IFRAO2(CLK_IFRAO_UFS, "ifrao_ufs", "ufs_ck", 28),
	GATE_IFRAO2(CLK_IFRAO_UFS_AES, "ifrao_u_aes", "aes_ufsfde_ck", 29),
	GATE_IFRAO2(CLK_IFRAO_USB_XHCI, "ifrao_usb_xhci", "ssusb_xhci_ck", 31),
	GATE_IFRAO3(CLK_IFRAO_AP_MSDC0, "ifrao_ap_msdc0", "msdc50_0_ck", 7),
	GATE_IFRAO3(CLK_IFRAO_MD_MSDC0, "ifrao_md_msdc0", "msdc50_0_ck", 8),
	GATE_IFRAO3(CLK_IFRAO_CCIF5_MD, "ifrao_ccif5_md", "axi_ck", 10),
	GATE_IFRAO3(CLK_IFRAO_CCIF2_AP, "ifrao_ccif2_ap", "axi_ck", 16),
	GATE_IFRAO3(CLK_IFRAO_CCIF2_MD, "ifrao_ccif2_md", "axi_ck", 17),
	GATE_IFRAO3(CLK_IFRAO_FBIST2FPC, "ifrao_fbist2fpc", "msdc50_0_ck", 24),
	GATE_IFRAO3(CLK_IFRAO_DPMAIF_MAIN, "ifrao_dpmaif_main", "dpmaif_main_ck", 26),
	GATE_IFRAO3(CLK_IFRAO_CCIF4_AP, "ifrao_ccif4_ap", "axi_ck", 28),
	GATE_IFRAO3(CLK_IFRAO_CCIF4_MD, "ifrao_ccif4_md", "axi_ck", 29),
	GATE_IFRAO3(CLK_IFRAO_SPI6_CK, "ifrao_spi6_ck", "spi_ck", 30),
	GATE_IFRAO3(CLK_IFRAO_SPI7_CK, "ifrao_spi7_ck", "spi_ck", 31),
	GATE_IFRAO4(CLK_IFRAO_66MP_BUS_MCLK_CK_P, "ifrao_66mp_mclk_p", "axi_ck", 2),
	GATE_IFRAO4(CLK_IFRAO_INFRA_FREE_DCM_133M, "ifrao_infra_133m", "axi_ck", 3),
	GATE_IFRAO4(CLK_IFRAO_INFRA_FREE_DCM_66M, "ifrao_infra_66m", "axi_ck", 4),
	GATE_IFRAO4(CLK_IFRAO_PERU_BUS_DCM_133M, "ifrao_peru_bus_133m", "axi_ck", 5),
	GATE_IFRAO4(CLK_IFRAO_PERU_BUS_DCM_66M, "ifrao_peru_bus_66m", "axi_ck", 6),
	GATE_IFRAO4(CLK_IFRAO_RG_FLASHIFP_26M_CK, "ifrao_flash_26m", "axi_ck", 30),
	GATE_IFRAO4(CLK_IFRAO_RG_FLASHIF_SFLASH_CK, "ifrao_sflash_ck", "axi_ck", 31),
	GATE_IFRAO5(CLK_IFRAO_AP_DMA, "ifrao_ap_dma", "apdma_pseudo", 31),
};

static const struct mtk_clk_desc ifrao_desc = {
	.clks = ifrao_clks,
	.num_clks = ARRAY_SIZE(ifrao_clks),
};

static const struct mtk_clk_desc topck_desc = {
	.factor_clks = top_divs,
	.num_factor_clks = ARRAY_SIZE(top_divs),
	.mux_clks = top_mtk_muxes,
	.num_mux_clks = ARRAY_SIZE(top_mtk_muxes),
	.composite_clks = top_muxes,
	.num_composite_clks = ARRAY_SIZE(top_muxes),
	.clk_lock = &mt6833_clk_lock,
};

static const struct of_device_id of_match_clk_mt6833[] = {
	{ .compatible = "mediatek,mt6833-infracfg-ao", .data = &ifrao_desc },
	{ .compatible = "mediatek,mt6833-topckgen", .data = &topck_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833);

static struct platform_driver clk_mt6833_drv = {
	.driver = {
		.name = "clk-mt6833",
		.of_match_table = of_match_clk_mt6833,
	},
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
};
module_platform_driver(clk_mt6833_drv);

MODULE_DESCRIPTION("MediaTek MT6833 main clocks driver");
MODULE_LICENSE("GPL");
