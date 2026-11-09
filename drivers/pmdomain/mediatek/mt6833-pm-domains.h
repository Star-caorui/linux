/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#ifndef __PMDOMAIN_MEDIATEK_MT6833_PM_DOMAINS_H
#define __PMDOMAIN_MEDIATEK_MT6833_PM_DOMAINS_H

#include <linux/soc/mediatek/infracfg.h>
#include <dt-bindings/power/mediatek,mt6833-power.h>
#include "mtk-pm-domains.h"

/* MT6833 uses the same INFRACFG bus protection registers as MT8192 */
#define MT6833_TOP_AXI_PROT_EN_MD			BIT(7)
#define MT6833_TOP_AXI_PROT_EN_VDNR_MD			(BIT(0) | BIT(10) | BIT(16))
#define MT6833_TOP_AXI_PROT_EN_CONN			(BIT(13) | BIT(18))
#define MT6833_TOP_AXI_PROT_EN_CONN_2ND			BIT(14)
#define MT6833_TOP_AXI_PROT_EN_1_CONN			BIT(10)
#define MT6833_TOP_AXI_PROT_EN_1_MFG1			BIT(21)
#define MT6833_TOP_AXI_PROT_EN_2_MFG1			GENMASK(6, 5)
#define MT6833_TOP_AXI_PROT_EN_MFG1			GENMASK(22, 21)
#define MT6833_TOP_AXI_PROT_EN_2_MFG1_2ND		BIT(7)
#define MT6833_TOP_AXI_PROT_EN_MM_2_ISP			BIT(8)
#define MT6833_TOP_AXI_PROT_EN_MM_2_ISP_2ND		BIT(9)
#define MT6833_TOP_AXI_PROT_EN_MM_IPE			BIT(16)
#define MT6833_TOP_AXI_PROT_EN_MM_IPE_2ND		BIT(17)
#define MT6833_TOP_AXI_PROT_EN_MM_VDEC			BIT(24)
#define MT6833_TOP_AXI_PROT_EN_MM_VDEC_2ND		BIT(25)
#define MT6833_TOP_AXI_PROT_EN_MM_VENC			BIT(26)
#define MT6833_TOP_AXI_PROT_EN_MM_VENC_2ND		BIT(27)
#define MT6833_TOP_AXI_PROT_EN_MM_DISP			(BIT(0) | BIT(2) | BIT(10) | \
							 BIT(12) | BIT(16) | BIT(24) | \
							 BIT(26))
#define MT6833_TOP_AXI_PROT_EN_MM_2_DISP		BIT(8)
#define MT6833_TOP_AXI_PROT_EN_DISP			(BIT(6) | BIT(23))
#define MT6833_TOP_AXI_PROT_EN_MM_DISP_2ND		(BIT(1) | BIT(3) | BIT(17) | \
							 BIT(25) | BIT(27))
#define MT6833_TOP_AXI_PROT_EN_MM_2_DISP_2ND		BIT(9)
#define MT6833_TOP_AXI_PROT_EN_2_AUDIO			BIT(4)
#define MT6833_TOP_AXI_PROT_EN_2_CAM			BIT(0)
#define MT6833_TOP_AXI_PROT_EN_MM_CAM			(BIT(0) | BIT(2))
#define MT6833_TOP_AXI_PROT_EN_1_CAM			BIT(22)
#define MT6833_TOP_AXI_PROT_EN_MM_CAM_2ND		(BIT(1) | BIT(3))
#define MT6833_TOP_AXI_PROT_EN_VDNR_CAM			BIT(15)

static enum scpsys_bus_prot_block scpsys_bus_prot_blocks_mt6833[] = {
	BUS_PROT_BLOCK_INFRA
};

static const struct scpsys_domain_data scpsys_domain_data_mt6833[] = {
	[MT6833_POWER_DOMAIN_MD] = {
		.name = "md",
		.sta_mask = BIT(0),
		.ctl_offs = 0x300,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.ext_buck_iso_offs = 0x398,
		.ext_buck_iso_mask = GENMASK(1, 0),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MD,
					MT8192_TOP_AXI_PROT_EN_SET,
					MT8192_TOP_AXI_PROT_EN_CLR,
					MT8192_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_VDNR_MD,
					MT8192_TOP_AXI_PROT_EN_VDNR_SET,
					MT8192_TOP_AXI_PROT_EN_VDNR_CLR,
					MT8192_TOP_AXI_PROT_EN_VDNR_STA1),
		},
		.caps = MTK_SCPD_MODEM_PWRSEQ | MTK_SCPD_EXT_BUCK_ISO |
			MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6833_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = BIT(1),
		.ctl_offs = 0x304,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_CONN,
					MT8192_TOP_AXI_PROT_EN_SET,
					MT8192_TOP_AXI_PROT_EN_CLR,
					MT8192_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_CONN_2ND,
					MT8192_TOP_AXI_PROT_EN_SET,
					MT8192_TOP_AXI_PROT_EN_CLR,
					MT8192_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_1_CONN,
					MT8192_TOP_AXI_PROT_EN_1_SET,
					MT8192_TOP_AXI_PROT_EN_1_CLR,
					MT8192_TOP_AXI_PROT_EN_1_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6833_POWER_DOMAIN_MFG0] = {
		.name = "mfg0",
		.sta_mask = BIT(2),
		.ctl_offs = 0x308,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6833_POWER_DOMAIN_MFG1] = {
		.name = "mfg1",
		.sta_mask = BIT(3),
		.ctl_offs = 0x30c,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_1_MFG1,
					MT8192_TOP_AXI_PROT_EN_1_SET,
					MT8192_TOP_AXI_PROT_EN_1_CLR,
					MT8192_TOP_AXI_PROT_EN_1_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_2_MFG1,
					MT8192_TOP_AXI_PROT_EN_2_SET,
					MT8192_TOP_AXI_PROT_EN_2_CLR,
					MT8192_TOP_AXI_PROT_EN_2_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MFG1,
					MT8192_TOP_AXI_PROT_EN_SET,
					MT8192_TOP_AXI_PROT_EN_CLR,
					MT8192_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_2_MFG1_2ND,
					MT8192_TOP_AXI_PROT_EN_2_SET,
					MT8192_TOP_AXI_PROT_EN_2_CLR,
					MT8192_TOP_AXI_PROT_EN_2_STA1),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6833_POWER_DOMAIN_MFG2] = {
		.name = "mfg2",
		.sta_mask = BIT(4),
		.ctl_offs = 0x310,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6833_POWER_DOMAIN_MFG3] = {
		.name = "mfg3",
		.sta_mask = BIT(5),
		.ctl_offs = 0x314,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
	[MT6833_POWER_DOMAIN_ISP] = {
		.name = "isp",
		.sta_mask = BIT(13),
		.ctl_offs = 0x334,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_2_ISP,
					MT8192_TOP_AXI_PROT_EN_MM_2_SET,
					MT8192_TOP_AXI_PROT_EN_MM_2_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_2_ISP_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_2_SET,
					MT8192_TOP_AXI_PROT_EN_MM_2_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_2_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_ISP2] = {
		.name = "isp2",
		.sta_mask = BIT(14),
		.ctl_offs = 0x338,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
	},
	[MT6833_POWER_DOMAIN_IPE] = {
		.name = "ipe",
		.sta_mask = BIT(15),
		.ctl_offs = 0x33c,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_IPE,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_IPE_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = BIT(16),
		.ctl_offs = 0x340,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_VDEC,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_VDEC_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = BIT(18),
		.ctl_offs = 0x348,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_VENC,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_VENC_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_DISP] = {
		.name = "disp",
		.sta_mask = BIT(21),
		.ctl_offs = 0x354,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_DISP,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_2_DISP,
					MT8192_TOP_AXI_PROT_EN_MM_2_SET,
					MT8192_TOP_AXI_PROT_EN_MM_2_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_2_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_DISP,
					MT8192_TOP_AXI_PROT_EN_SET,
					MT8192_TOP_AXI_PROT_EN_CLR,
					MT8192_TOP_AXI_PROT_EN_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_DISP_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_2_DISP_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_2_SET,
					MT8192_TOP_AXI_PROT_EN_MM_2_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_2_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = BIT(22),
		.ctl_offs = 0x358,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_2_AUDIO,
					MT8192_TOP_AXI_PROT_EN_2_SET,
					MT8192_TOP_AXI_PROT_EN_2_CLR,
					MT8192_TOP_AXI_PROT_EN_2_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_CAM] = {
		.name = "cam",
		.sta_mask = BIT(23),
		.ctl_offs = 0x35c,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_2_CAM,
					MT8192_TOP_AXI_PROT_EN_2_SET,
					MT8192_TOP_AXI_PROT_EN_2_CLR,
					MT8192_TOP_AXI_PROT_EN_2_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_CAM,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_1_CAM,
					MT8192_TOP_AXI_PROT_EN_1_SET,
					MT8192_TOP_AXI_PROT_EN_1_CLR,
					MT8192_TOP_AXI_PROT_EN_1_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_MM_CAM_2ND,
					MT8192_TOP_AXI_PROT_EN_MM_SET,
					MT8192_TOP_AXI_PROT_EN_MM_CLR,
					MT8192_TOP_AXI_PROT_EN_MM_STA1),
			BUS_PROT_WR_IGN(INFRA,
					MT6833_TOP_AXI_PROT_EN_VDNR_CAM,
					MT8192_TOP_AXI_PROT_EN_VDNR_SET,
					MT8192_TOP_AXI_PROT_EN_VDNR_CLR,
					MT8192_TOP_AXI_PROT_EN_VDNR_STA1),
		},
	},
	[MT6833_POWER_DOMAIN_CAM_RAWA] = {
		.name = "cam_rawa",
		.sta_mask = BIT(24),
		.ctl_offs = 0x360,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
	},
	[MT6833_POWER_DOMAIN_CAM_RAWB] = {
		.name = "cam_rawb",
		.sta_mask = BIT(25),
		.ctl_offs = 0x364,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
	},
};

static const struct scpsys_soc_data mt6833_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6833,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6833),
	.bus_prot_blocks = scpsys_bus_prot_blocks_mt6833,
	.num_bus_prot_blocks = ARRAY_SIZE(scpsys_bus_prot_blocks_mt6833),
};

#endif /* __PMDOMAIN_MEDIATEK_MT6833_PM_DOMAINS_H */
