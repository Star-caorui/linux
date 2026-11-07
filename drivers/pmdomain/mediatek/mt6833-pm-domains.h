/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Based on MediaTek's mtk-scpsys-mt6833.c:
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#ifndef __PMDOMAIN_MEDIATEK_MT6833_PM_DOMAINS_H
#define __PMDOMAIN_MEDIATEK_MT6833_PM_DOMAINS_H

#include <linux/soc/mediatek/infracfg.h>
#include <dt-bindings/power/mediatek,mt6833-power.h>
#include "mtk-pm-domains.h"

/*
 * The bus protection registers are MT8192's. The masks are spelled out for
 * MT6833 even where they match MT8192's, so that each can be checked against
 * MediaTek's table.
 */
/*
 * MFG1's way to memory, in INFRA_BCRM (which the driver reaches as its
 * INFRA_NAO block): MediaTek's gpufreq closes it before the bus protection
 * on power-off and opens it on power-on, waiting for it to read back open.
 * Its wait on closing looks only at bit 12, which it keeps set and which is
 * left alone here.
 */
#define MT6833_INFRA_BCRM_MFG1_WAY_EN			0x04c
#define MT6833_INFRA_BCRM_MFG1_WAY_EN_MASK		BIT(11)

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

static const struct scpsys_domain_data scpsys_domain_data_mt6833[] = {
	[MT6833_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = BIT(1),
		.ctl_offs = 0x304,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = 0,
		.sram_pdn_ack_bits = 0,
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
		.caps = MTK_SCPD_DOMAIN_SUPPLY,
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
			_BUS_PROT_STA(INFRA_NAO, INFRA_NAO,
				      MT6833_INFRA_BCRM_MFG1_WAY_EN_MASK,
				      MT6833_INFRA_BCRM_MFG1_WAY_EN,
				      MT6833_INFRA_BCRM_MFG1_WAY_EN,
				      MT6833_INFRA_BCRM_MFG1_WAY_EN_MASK,
				      MT6833_INFRA_BCRM_MFG1_WAY_EN,
				      BUS_PROT_INVERTED | BUS_PROT_REG_UPDATE |
				      BUS_PROT_IGNORE_CLR_ACK),
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
		.caps = MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6833_POWER_DOMAIN_MFG2] = {
		.name = "mfg2",
		.sta_mask = BIT(4),
		.ctl_offs = 0x310,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
	},
	[MT6833_POWER_DOMAIN_MFG3] = {
		.name = "mfg3",
		.sta_mask = BIT(5),
		.ctl_offs = 0x314,
		.pwr_sta_offs = 0x16c,
		.pwr_sta2nd_offs = 0x170,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
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
};

#endif /* __PMDOMAIN_MEDIATEK_MT6833_PM_DOMAINS_H */
