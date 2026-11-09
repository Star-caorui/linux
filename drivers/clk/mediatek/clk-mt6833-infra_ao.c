// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 * Copyright (c) 2026 Rui Cao <i@udp0.com>
 */

#include <dt-bindings/clock/mediatek,mt6833-clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs infra_ao0_cg_regs = {
	.set_ofs = 0x80,
	.clr_ofs = 0x84,
	.sta_ofs = 0x90,
};

static const struct mtk_gate_regs infra_ao1_cg_regs = {
	.set_ofs = 0x88,
	.clr_ofs = 0x8c,
	.sta_ofs = 0x94,
};

static const struct mtk_gate_regs infra_ao2_cg_regs = {
	.set_ofs = 0xa4,
	.clr_ofs = 0xa8,
	.sta_ofs = 0xac,
};

static const struct mtk_gate_regs infra_ao3_cg_regs = {
	.set_ofs = 0xc0,
	.clr_ofs = 0xc4,
	.sta_ofs = 0xc8,
};

static const struct mtk_gate_regs infra_ao4_cg_regs = {
	.set_ofs = 0xd0,
	.clr_ofs = 0xd4,
	.sta_ofs = 0xd8,
};

static const struct mtk_gate_regs infra_ao5_cg_regs = {
	.set_ofs = 0xe0,
	.clr_ofs = 0xe4,
	.sta_ofs = 0xe8,
};

#define GATE_INFRA_AO0(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao0_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO1(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao1_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO2(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao2_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO3(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao3_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO4(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao4_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO5(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao5_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr)

static const struct mtk_gate infra_ao_clks[] = {
	/* INFRA_AO0 */
	GATE_INFRA_AO0(CLK_IFRAO_PMIC_TMR, "ifrao_pmic_tmr", "pwrap_ulposc_sel", 0),
	GATE_INFRA_AO0(CLK_IFRAO_PMIC_AP, "ifrao_pmic_ap", "pwrap_ulposc_sel", 1),
	GATE_INFRA_AO0(CLK_IFRAO_GCE, "ifrao_gce", "axi_sel", 8),
	GATE_INFRA_AO0(CLK_IFRAO_GCE2, "ifrao_gce2", "axi_sel", 9),
	GATE_INFRA_AO0(CLK_IFRAO_THERM, "ifrao_therm", "axi_sel", 10),
	GATE_INFRA_AO0(CLK_IFRAO_I2C_PSEUDO, "ifrao_i2c_pseudo", "i2c_sel", 11),
	GATE_INFRA_AO0(CLK_IFRAO_I2C1_PSEUDO, "ifrao_i2c1_pseudo", "axi_sel", 12),
	GATE_INFRA_AO0(CLK_IFRAO_PWM_HCLK, "ifrao_pwm_hclk", "axi_sel", 15),
	GATE_INFRA_AO0(CLK_IFRAO_PWM1, "ifrao_pwm1", "pwm_sel", 16),
	GATE_INFRA_AO0(CLK_IFRAO_PWM2, "ifrao_pwm2", "pwm_sel", 17),
	GATE_INFRA_AO0(CLK_IFRAO_PWM3, "ifrao_pwm3", "pwm_sel", 18),
	GATE_INFRA_AO0(CLK_IFRAO_PWM4, "ifrao_pwm4", "pwm_sel", 19),
	GATE_INFRA_AO0(CLK_IFRAO_PWM, "ifrao_pwm", "pwm_sel", 21),
	GATE_INFRA_AO0(CLK_IFRAO_UART0, "ifrao_uart0", "uart_sel", 22),
	GATE_INFRA_AO0(CLK_IFRAO_UART1, "ifrao_uart1", "uart_sel", 23),
	GATE_INFRA_AO0(CLK_IFRAO_UART2, "ifrao_uart2", "uart_sel", 24),
	GATE_INFRA_AO0(CLK_IFRAO_UART3, "ifrao_uart3", "uart_sel", 25),
	GATE_INFRA_AO0(CLK_IFRAO_GCE_26M, "ifrao_gce_26m", "axi_sel", 27),
	GATE_INFRA_AO0(CLK_IFRAO_BTIF, "ifrao_btif", "axi_sel", 31),
	/* INFRA_AO1 */
	GATE_INFRA_AO1(CLK_IFRAO_SPI0, "ifrao_spi0", "spi_sel", 1),
	GATE_INFRA_AO1(CLK_IFRAO_MSDC0, "ifrao_msdc0", "axi_sel", 2),
	GATE_INFRA_AO1(CLK_IFRAO_MSDC1, "ifrao_msdc1", "axi_sel", 4),
	GATE_INFRA_AO1(CLK_IFRAO_MSDC0_SRC, "ifrao_msdc0_clk", "msdc50_0_sel", 6),
	GATE_INFRA_AO1(CLK_IFRAO_AUXADC, "ifrao_auxadc", "clk26m", 10),
	GATE_INFRA_AO1(CLK_IFRAO_CPUM, "ifrao_cpum", "axi_sel", 11),
	GATE_INFRA_AO1(CLK_IFRAO_CCIF1_AP, "ifrao_ccif1_ap", "axi_sel", 12),
	GATE_INFRA_AO1(CLK_IFRAO_CCIF1_MD, "ifrao_ccif1_md", "axi_sel", 13),
	GATE_INFRA_AO1(CLK_IFRAO_AUXADC_MD, "ifrao_auxadc_md", "clk26m", 14),
	GATE_INFRA_AO1(CLK_IFRAO_MSDC1_SRC, "ifrao_msdc1_clk", "msdc30_1_sel", 16),
	GATE_INFRA_AO1(CLK_IFRAO_MSDC0_AES, "ifrao_msdc0_aes_clk", "msdc50_0_sel", 17),
	GATE_INFRA_AO1(CLK_IFRAO_CCIF_AP, "ifrao_ccif_ap", "axi_sel", 23),
	GATE_INFRA_AO1(CLK_IFRAO_DEBUGSYS, "ifrao_debugsys", "axi_sel", 24),
	GATE_INFRA_AO1(CLK_IFRAO_AUDIO, "ifrao_audio", "axi_sel", 25),
	GATE_INFRA_AO1(CLK_IFRAO_CCIF_MD, "ifrao_ccif_md", "axi_sel", 26),
	/* INFRA_AO2 */
	GATE_INFRA_AO2(CLK_IFRAO_SSUSB, "ifrao_ssusb", "usb_sel", 1),
	GATE_INFRA_AO2(CLK_IFRAO_DISP_PWM, "ifrao_disp_pwm", "axi_sel", 2),
	GATE_INFRA_AO2(CLK_IFRAO_CLDMA_BCLK, "ifrao_cldmabclk", "axi_sel", 3),
	GATE_INFRA_AO2(CLK_IFRAO_AUDIO_26M_BCLK, "ifrao_audio26m", "clk26m", 4),
	GATE_INFRA_AO2(CLK_IFRAO_SPI1, "ifrao_spi1", "spi_sel", 6),
	GATE_INFRA_AO2(CLK_IFRAO_SPI2, "ifrao_spi2", "spi_sel", 9),
	GATE_INFRA_AO2(CLK_IFRAO_SPI3, "ifrao_spi3", "spi_sel", 10),
	GATE_INFRA_AO2(CLK_IFRAO_UNIPRO_SYSCLK, "ifrao_unipro_sysclk", "ufs_sel", 11),
	GATE_INFRA_AO2(CLK_IFRAO_UFS_SAP_BCLK, "ifrao_u_bclk", "clk26m", 13),
	GATE_INFRA_AO2(CLK_IFRAO_SPI4, "ifrao_spi4", "spi_sel", 25),
	GATE_INFRA_AO2(CLK_IFRAO_SPI5, "ifrao_spi5", "spi_sel", 26),
	GATE_INFRA_AO2(CLK_IFRAO_CQ_DMA, "ifrao_cq_dma", "axi_sel", 27),
	GATE_INFRA_AO2(CLK_IFRAO_UFS, "ifrao_ufs", "ufs_sel", 28),
	GATE_INFRA_AO2(CLK_IFRAO_UFS_AES, "ifrao_u_aes", "aes_ufsfde_sel", 29),
	GATE_INFRA_AO2(CLK_IFRAO_USB_XHCI, "ifrao_usb_xhci", "ssusb_xhci_sel", 31),
	/* INFRA_AO3 */
	GATE_INFRA_AO3(CLK_IFRAO_AP_MSDC0, "ifrao_ap_msdc0", "msdc50_0_sel", 7),
	GATE_INFRA_AO3(CLK_IFRAO_MD_MSDC0, "ifrao_md_msdc0", "msdc50_0_sel", 8),
	GATE_INFRA_AO3(CLK_IFRAO_CCIF5_MD, "ifrao_ccif5_md", "axi_sel", 10),
	GATE_INFRA_AO3(CLK_IFRAO_CCIF2_AP, "ifrao_ccif2_ap", "axi_sel", 16),
	GATE_INFRA_AO3(CLK_IFRAO_CCIF2_MD, "ifrao_ccif2_md", "axi_sel", 17),
	GATE_INFRA_AO3(CLK_IFRAO_FBIST2FPC, "ifrao_fbist2fpc", "msdc50_0_sel", 24),
	GATE_INFRA_AO3(CLK_IFRAO_DPMAIF_MAIN, "ifrao_dpmaif_main", "dpmaif_main_sel", 26),
	GATE_INFRA_AO3(CLK_IFRAO_CCIF4_AP, "ifrao_ccif4_ap", "axi_sel", 28),
	GATE_INFRA_AO3(CLK_IFRAO_CCIF4_MD, "ifrao_ccif4_md", "axi_sel", 29),
	GATE_INFRA_AO3(CLK_IFRAO_SPI6_CK, "ifrao_spi6_ck", "spi_sel", 30),
	GATE_INFRA_AO3(CLK_IFRAO_SPI7_CK, "ifrao_spi7_ck", "spi_sel", 31),
	/* INFRA_AO5 */
	GATE_INFRA_AO5(CLK_IFRAO_66MP_BUS_MCLK_CK_P, "ifrao_66mp_mclk_p", "axi_sel", 2),
	GATE_INFRA_AO5(CLK_IFRAO_INFRA_FREE_DCM_133M, "ifrao_infra_133m", "axi_sel", 3),
	GATE_INFRA_AO5(CLK_IFRAO_INFRA_FREE_DCM_66M, "ifrao_infra_66m", "axi_sel", 4),
	GATE_INFRA_AO5(CLK_IFRAO_PERU_BUS_DCM_133M, "ifrao_peru_bus_133m", "axi_sel", 5),
	GATE_INFRA_AO5(CLK_IFRAO_PERU_BUS_DCM_66M, "ifrao_peru_bus_66m", "axi_sel", 6),
	GATE_INFRA_AO5(CLK_IFRAO_RG_FLASHIFP_26M_CK, "ifrao_flash_26m", "axi_sel", 30),
	GATE_INFRA_AO5(CLK_IFRAO_RG_FLASHIF_SFLASH_CK, "ifrao_sflash_ck", "axi_sel", 31),
	/* INFRA_AO4 */
	GATE_INFRA_AO4(CLK_IFRAO_AP_DMA, "ifrao_ap_dma", "ifrao_i2c1_pseudo", 31),
};

static const struct mtk_clk_desc infra_ao_desc = {
	.clks = infra_ao_clks,
	.num_clks = ARRAY_SIZE(infra_ao_clks),
};

static const struct of_device_id of_match_clk_mt6833_infra_ao[] = {
	{ .compatible = "mediatek,mt6833-infracfg-ao", .data = &infra_ao_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6833_infra_ao);

static struct platform_driver clk_mt6833_infra_ao_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6833-infra_ao",
		.of_match_table = of_match_clk_mt6833_infra_ao,
	},
};
module_platform_driver(clk_mt6833_infra_ao_drv);

MODULE_DESCRIPTION("MediaTek MT6833 infracfg_ao clocks driver");
MODULE_LICENSE("GPL");
