// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6382 MIPI DSI to MIPI DSI bridge with a DSC encoder
 *
 * The MT6382 takes uncompressed RGB888 video from the SoC's DSI host into a
 * Synopsys DesignWare MIPI DSI-2 device controller and its D-PHY, buffers it
 * in an RDMA engine, compresses it with a DSC 1.1/1.2 encoder and sends it on
 * to the panel through a MediaTek DSI transmitter of its own. Its registers
 * are reached over SPI with MediaTek's SPI slave protocol.
 *
 * The register programming is ported from MediaTek's driver as shipped in the
 * kernel of the Xiaomi Redmi Note 13 5G ("gold", MT6833), in its
 * drivers/gpu/drm/mediatek/mediatek_v2/: mtk_disp_bdg.c (the bridge),
 * mtk_reg_disp_bdg.h and mt6382.h (register layout), mtk_dsi.c (the parts the
 * SoC's DSI host does for the bridge), and drivers/misc/mediatek/spi_slave_drv/
 * spi_slave.c (the SPI protocol). Line numbers in comments refer to those
 * files. Where MediaTek writes values it does not explain, they are kept as
 * tables.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/* TEST: the SoC's DSI host, for its vertical state (DSI_STATE_DBG7) */
#define AP_DSI_BASE	0x14013000
#define AP_DBG7		0x164

/* SPI slave protocol, spi_slave.c:29-55 */
#define SPISLV_CMD_RS			0x06	/* read status */
#define SPISLV_CMD_WS			0x08	/* write status */
#define SPISLV_CMD_CR			0x0a	/* config read */
#define SPISLV_CMD_CW			0x0c	/* config write */
#define SPISLV_CMD_RD			0x81	/* read data */
#define SPISLV_CMD_WD			0x0e	/* write data */
#define SPISLV_CMD_CT			0x10	/* config type */
#define SPISLV_CT_4BYTE_SINGLE		0x04

#define SPISLV_SR_CFG_SUCCESS		BIT(1)
#define SPISLV_SR_TXRX_FIFO_RDY		BIT(2)
#define SPISLV_SR_RD_ERR		BIT(3)
#define SPISLV_SR_WR_ERR		BIT(4)
#define SPISLV_SR_RDWR_FINISH		BIT(5)
#define SPISLV_SR_TIMEOUT_ERR		BIT(6)
#define SPISLV_SR_CONFIG_READY		(SPISLV_SR_CFG_SUCCESS | SPISLV_SR_TXRX_FIFO_RDY)

/* spi_slave.h: the slave's clocks only allow the high rates once it is set up */
#define SPISLV_LOW_SPEED_HZ		6500000
#define SPISLV_TX_MAX_SPEED_HZ		55000000
#define SPISLV_RX_MAX_SPEED_HZ		28000000

/*
 * Bridge register map. Offsets come from the structs in mtk_reg_disp_bdg.h
 * placed at the bases of mtk_disp_bdg.h:69-82.
 */
#define SYS_PWR_CTRL			0x00008
#define PWR_ON				BIT(8)
#define PWR_ON_2ND			BIT(10)
#define PISO_EN				BIT(0)
#define DISP_PWR_CLK_DIS		BIT(4)
#define SYSBUF_SRAM_SLEEP_B		BIT(21)
#define GCE_SRAM_SLEEP_B		BIT(17)
#define SYS_RST_CTRL			0x00010
#define SW_RST_EN_DISP_PWR_WRAP		BIT(15)
#define SYS_RST_DG_CTRL			0x00064
#define SYS_DISP_MISC0			0x00080
#define SYS_DISP_MISC1			0x00084
#define SYS_CKBUF_CTRL			0x000a0
#define CKBUF_NFC_TERM			GENMASK(3, 0)
#define CKBUF_EXT_CLK_REQ_EN		BIT(4)
#define CKBUF_NFC_CK_OUT_EN		BIT(5)	/* on regardless of the request */
#define SYS_LDO_CTRL0			0x000a4
#define PHYLDO_MASKB			BIT(31)
#define LDO_TRIM_BY_EFUSE		BIT(30)
#define SYS_LDO_CTRL1			0x000a8
#define PHYLDO1_LP_EN			BIT(29)
#define PHYLDO2_EN			BIT(0)
#define SYS_LDO_STATUS			0x000ac
#define SPISLV_CTRL			0x02000	/* spi_slave.c:67 */
#define SPISLV_EARLY_TRANS		BIT(16)
#define CK_CLK_CFG_UPDATE		0x03004
#define CK_CLK_CFG_0_SET		0x03014
#define CK_CLK_CFG_0_CLR		0x03018
#define AP_PLL_CON3			0x0400c
#define AP_PLL_CON5			0x04014
#define AP_PLLON_CON0			0x04050
#define AP_PLLON_CON1			0x04054
#define AP_MAINPLL_CON0			0x04218
#define AP_MAINPLL_CON1			0x0421c
#define AP_MAINPLL_CON3			0x04224
#define IOCFG_DRV_CFG0			0x05000	/* spi_slave.c:63-65 */
#define SPIS_SLVO			GENMASK(23, 21)
#define GPIO_MODE1			0x07310
#define GPIO_MODE1_GPIO12		GENMASK(18, 16)
#define GPIO_MODE1_TE			GENMASK(31, 24)
#define EFUSE_STATUS			0x09128
#define EFUSE_TRIM1			0x098e0
#define EFUSE_TRIM2			0x098e4
#define EFUSE_TRIM3			0x098e8
#define EFUSE_DCM_ON			0x09480
#define DSI2_BASE			0x0d000	/* the DSI-2 device (RX MAC) */
#define DSI2_PHY_SHUTDOWNZ		0x0d100
#define DSI2_PHY_RSTZ			0x0d104
#define DSI2_PHY_CLK_STATUS		0x0d120
/*
 * Read 6 while the bootloader's picture streams through and 2 when the AP's
 * clock lane was already in HS as the RX came out of reset: taken to be the
 * D-PHY's RxClkActiveHS.
 */
#define DSI2_PHY_CLK_ACTIVE_HS		BIT(2)
#define DSI2_PHY_DATA_STATUS		0x0d124
#define GCE_CTL_INT0			0x100f0
#define OCLA_PHY_READY			0x14010
#define OCLA_LANE_SWAP			0x14020
#define OCLA_LANE0_CON			0x14100
#define OCLA_LANE0_STOPSTATE		0x1412c
#define OCLA_LANE1_CON			0x14200
#define OCLA_LANE2_CON			0x14300
#define OCLA_LANE3_CON			0x14400
#define OCLA_LANEC_CON			0x14500
#define OCLA_FORCE_RX_MODE		BIT(2)
#define DSC_CON				0x20000
#define DSC_CON_EN			BIT(0)
#define DSC_CON_PT_MEM_EN		BIT(7)
#define DSC_CON_EMPTY_FLAG_SEL		GENMASK(15, 14)
#define DSC_CON_UFOE_SEL		BIT(16)
#define DSC_PIC_W			0x20018
#define DSC_PIC_H			0x2001c
#define DSC_SLICE_W			0x20020
#define DSC_SLICE_H			0x20024
#define DSC_CHUNK_SIZE			0x20028
#define DSC_BUF_SIZE			0x2002c
#define DSC_MODE			0x20030
#define DSC_CFG				0x20034
#define DSC_PAD				0x20038
#define DSC_OBUF			0x20070
#define DSC_PPS(n)			(0x20080 + 4 * (n))
#define DSC_SHADOW			0x20200
#define DSC_VERSION_MINOR		GENMASK(8, 5)
#define TX_START			0x21000
#define TX_START_START			BIT(0)
#define TX_INTEN			0x21008
#define TX_INT_VM_VFP_STR		BIT(10)
#define TX_INT_BUFFER_UNDERRUN		BIT(12)
#define TX_INTSTA			0x2100c
#define TX_INT_SLEEPIN_ULPS		BIT(15)
#define TX_INT_BUSY			BIT(31)
#define TX_COM_CON			0x21010
#define TX_DSI_RESET			BIT(0)
#define TX_MODE_CON			0x21014
#define TX_MODE				GENMASK(1, 0)
#define TX_MODE_CMD			0
#define TX_MODE_SYNC_PULSE		1
#define TX_MODE_SYNC_EVENT		2
#define TX_MODE_BURST			3
#define TX_TXRX_CON			0x21018
#define TX_LANE_NUM			GENMASK(5, 2)
#define TX_DIS_EOT			BIT(6)
#define TX_EXT_TE_EN			BIT(9)
#define TX_HSTX_CKLP_EN			BIT(16)
#define TX_PSCON			0x2101c
#define TX_PS_WC			GENMASK(14, 0)
#define TX_PS_SEL			GENMASK(19, 16)
#define TX_PS_SEL_COMPRESSION		5
#define TX_CUSTOM_HEADER		GENMASK(31, 26)
#define TX_VSA_NL			0x21020
#define TX_VBP_NL			0x21024
#define TX_VFP_NL			0x21028
#define TX_VACT_NL			0x2102c
#define TX_VACT_NL_MASK			GENMASK(14, 0)
#define TX_SIZE_CON			0x21038
#define TX_WIDTH			GENMASK(14, 0)
#define TX_HEIGHT			GENMASK(30, 16)
#define TX_HSA_WC			0x21050
#define TX_HBP_WC			0x21054
#define TX_HFP_WC			0x21058
#define TX_BLLP_WC			0x2105c
#define TX_CMDQ_CON			0x21060
#define TX_CMDQ_SIZE_SEL		BIT(15)
#define TX_PHY_LCCON			0x21104
#define TX_LC_HS_TX_EN			BIT(0)
#define TX_LC_ULPM_EN			BIT(1)
#define TX_PHY_LD0CON			0x21108
#define TX_L0_ULPM_EN			BIT(1)
#define TX_LX_ULPM_AS_L0		BIT(3)
#define TX_PHY_TIMECON0			0x21110
#define TX_LPX				GENMASK(7, 0)
#define TX_HS_PRPR			GENMASK(15, 8)
#define TX_HS_ZERO			GENMASK(23, 16)
#define TX_HS_TRAIL			GENMASK(31, 24)
#define TX_PHY_TIMECON1			0x21114
#define TX_TA_GO			GENMASK(7, 0)
#define TX_TA_SURE			GENMASK(15, 8)
#define TX_TA_GET			GENMASK(23, 16)
#define TX_DA_HS_EXIT			GENMASK(31, 24)
#define TX_PHY_TIMECON2			0x21118
#define TX_CLK_ZERO			GENMASK(23, 16)
#define TX_CLK_TRAIL			GENMASK(31, 24)
#define TX_PHY_TIMECON3			0x2111c
#define TX_CLK_HS_PRPR			GENMASK(7, 0)
#define TX_CLK_HS_POST			GENMASK(15, 8)
#define TX_CLK_HS_EXIT			GENMASK(23, 16)
#define TX_RESYNC_CON			0x211f0
#define TX_BUF_CON0			0x21400
#define TX_ANTI_LATENCY_BUF_EN		BIT(0)
#define TX_BUF_RW_TIMES			0x21410
#define TX_CMDQ(n)			(0x21d00 + 4 * (n))
#define TX_CMDQ_WORDS			32	/* how much of the window we use */
#define TX_CMDQ_TYPE_LONG		2	/* DSI_TX_CMDQ_REG, mtk_reg_disp_bdg.h:484 */
#define TX_CMDQ_HS			BIT(3)
#define MIPITX_LANE_CON			0x2200c
#define MIPITX_BG_LPF_EN		BIT(6)
#define MIPITX_BG_CORE_EN		BIT(7)
#define MIPITX_PAD_TIEL_SEL		BIT(8)
#define MIPITX_PLL_PWR			0x22028
#define MIPITX_PLL_SDM_PWR_ON		BIT(0)
#define MIPITX_PLL_SDM_ISO_EN		BIT(1)
#define MIPITX_PLL_CON0			0x2202c
#define MIPITX_PLL_CON1			0x22030
#define MIPITX_PLL_EN			BIT(4)
#define MIPITX_PLL_POSDIV		GENMASK(10, 8)
#define MIPITX_PLL_CON2			0x22034
#define MIPITX_PLL_SDM_SSC_EN		BIT(1)
#define MIPITX_PLL_CON4			0x2203c
#define MIPITX_SW_CTRL_CON4		0x22060
#define MIPITX_SW_ANA_CK_EN		BIT(8)
#define MIPITX_D2_SW_CTL_EN		0x22144
#define MIPITX_D0_SW_CTL_EN		0x22244
#define MIPITX_CK_CKMODE_EN		0x22328
#define MIPITX_CK_SW_CTL_EN		0x22344
#define MIPITX_D1_SW_CTL_EN		0x22444
#define MIPITX_D3_SW_CTL_EN		0x22544
#define MIPITX_SW_CTL_EN		BIT(0)
/* after each lane's SW_CTL_EN: SW_LPTX_PRE_OE, _OE, _DP, _DN, bit 0 each */
#define MIPITX_SW_LPTX(lane_ctl, n)	((lane_ctl) + 4 * (n))
#define MIPITX_CKMODE_EN		BIT(0)
#define MM_CG_CON0			0x23100
#define MM_HW_DCM_1ST_DIS0		0x23120
#define MM_HW_DCM_2ND_DIS0		0x23140
#define MM_MIPI_RX_POST_CTRL		0x23170
#define MIPI_RX_MODE_SEL		BIT(0)	/* 0: DDI (commands), 1: IPI (video) */
#define MM_DDI_POST_CTRL		0x23174
#define RDMA_INTSTA			0x24004
#define RDMA_GLOBAL_CON			0x24010
#define RDMA_ENGINE_EN			BIT(0)
#define RDMA_SOFT_RESET			BIT(4)
#define RDMA_SIZE_CON_0			0x24014
#define RDMA_SIZE_CON_1			0x24018
#define RDMA_FIFO_CON			0x24040
#define RDMA_SRAM_SEL			0x240b0
#define RDMA_SRAM_SEL_SEL		GENMASK(2, 0)
#define RDMA_STALL_CG_CON		0x240b4
#define MUTEX_INTEN			0x25000
#define MUTEX0_EN			0x25020
#define MUTEX0_EN_EN			BIT(0)
#define MUTEX0_CTL			0x2502c
#define MUTEX0_MOD0			0x25030
#define MIPI_RX_PHY(n)			(0x40000 + 4 * (n))	/* mt6382.h:12 */
#define MT6382_MAX_REGISTER		0x4fffc

/* The bridge's MM clock, as LK and the stock kernel set it (bdg_mm_clk 405) */
#define MT6382_MM_CLK_MHZ		405
/* RX (AP side) to TX (panel side) lane rate ratio with DSC, x100 (bdg_rxtx_ratio) */
#define MT6382_RXTX_RATIO		229
/* The stock panel's TX rate, for a panel that does not give dsi->hs_rate */
#define MT6382_DEFAULT_TX_RATE_MBPS	1016

/* Bridge side of the link to the panel, and the AP side derived from it */
struct mt6382_timing {
	/* panel mode */
	u32 hactive, hsa, hbp, hfp;
	u32 vactive, vsa, vbp, vfp;
	u32 lanes;
	unsigned long mode_flags;
	u32 tx_rate;			/* Mbps per lane */

	/* bdg_tx_phy_config(), mtk_disp_bdg.c:1325-1477 */
	u8 lpx, hs_prpr, hs_zero, hs_trail;
	u8 ta_go, ta_sure, ta_get, da_hs_exit;
	u8 clk_zero, clk_trail, clk_hs_prpr, clk_hs_post, clk_hs_exit;
	u32 data_phy_cycle;

	/* bdg_tx_vdo_timing_set(), bdg_dsi_line_timing_dphy_setting() */
	u32 hsa_byte, hbp_byte, hfp_byte, bllp_byte, ps_wc;
	u32 line_cycle;

	/* the AP's DSI host, sending into the bridge's RX */
	u32 rx_rate;			/* Mbps per lane */
	u32 ap_htotal;			/* pixels per line at rx_rate */
};

enum mt6382_state {
	MT6382_OFF,		/* powered down, in reset or not set up by us */
	MT6382_CMD,		/* set up, TX in command mode: CMDQ path */
	MT6382_VIDEO,		/* TX streaming video */
};

struct mt6382 {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct clk *refclk;

	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	bool next_is_panel;
	struct mipi_dsi_host host;
	struct mipi_dsi_device *panel_dsi;
	struct mipi_dsi_device *ap_dsi;	/* us, on the SoC's DSI host */
	/* our registers through the SoC's DSI host, on virtual channel 1 */
	struct mipi_dsi_device *cfg_dsi;
	/* the host streams for us: between our enable and disable */
	bool host_streaming;

	/* serialises bring-up, power-down and panel commands */
	struct mutex lock;
	enum mt6382_state state;
	bool spi_fast;
	struct mt6382_timing tm;
	struct drm_dsc_config dsc;

	/* SPI buffers, DMA safe */
	u8 *txbuf, *rxbuf, *rs_tx, *rs_rx;
	struct mutex spi_lock;		/* the buffers above */

};

static inline struct mt6382 *bridge_to_mt6382(struct drm_bridge *bridge)
{
	return container_of(bridge, struct mt6382, bridge);
}

static inline struct mt6382 *host_to_mt6382(struct mipi_dsi_host *host)
{
	return container_of(host, struct mt6382, host);
}

/* --------------------------------------------------------------------------
 * SPI slave protocol (spi_slave.c)
 *
 * Every access is a config phase (CR/CW, address, length - 1), a data phase
 * (RD/WD) and status reads (RS), with chip select dropped between commands.
 * The slave samples LSB first in SPI mode 0 (spi_slave.c:430).
 */

static void mt6382_spi_xfer_init(struct spi_transfer *x, const void *tx, void *rx,
				 unsigned int len, u32 speed)
{
	x->tx_buf = tx;
	x->rx_buf = rx;
	x->len = len;
	x->speed_hz = speed;
	x->bits_per_word = 8;
}

static u32 mt6382_tx_speed(struct mt6382 *ctx)
{
	return ctx->spi_fast ? SPISLV_TX_MAX_SPEED_HZ : SPISLV_LOW_SPEED_HZ;
}

static u32 mt6382_rx_speed(struct mt6382 *ctx)
{
	return ctx->spi_fast ? SPISLV_RX_MAX_SPEED_HZ : SPISLV_LOW_SPEED_HZ;
}

/* One command, then RS in the same message (spi_slave.c:128-153) */
static int mt6382_spi_cmd(struct mt6382 *ctx, unsigned int len, bool read, u8 *status)
{
	struct spi_transfer x[2] = { };
	struct spi_message msg;
	int ret;

	mt6382_spi_xfer_init(&x[0], ctx->txbuf, read ? ctx->rxbuf : NULL, len,
			     read ? mt6382_rx_speed(ctx) : mt6382_tx_speed(ctx));
	x[0].cs_change = 1;
	ctx->rs_tx[0] = SPISLV_CMD_RS;
	ctx->rs_tx[1] = 0;
	mt6382_spi_xfer_init(&x[1], ctx->rs_tx, ctx->rs_rx, 2, mt6382_rx_speed(ctx));
	spi_message_init_with_transfers(&msg, x, ARRAY_SIZE(x));

	ret = spi_sync(ctx->spi, &msg);
	if (ret)
		return ret;

	*status = ctx->rs_rx[1];
	return 0;
}

static int mt6382_spi_status(struct mt6382 *ctx, u8 *status)
{
	struct spi_transfer x = { };
	int ret;

	ctx->rs_tx[0] = SPISLV_CMD_RS;
	ctx->rs_tx[1] = 0;
	mt6382_spi_xfer_init(&x, ctx->rs_tx, ctx->rs_rx, 2, mt6382_rx_speed(ctx));
	ret = spi_sync_transfer(ctx->spi, &x, 1);
	if (ret)
		return ret;

	*status = ctx->rs_rx[1];
	return 0;
}

static int mt6382_spi_short(struct mt6382 *ctx, u8 cmd, u8 arg)
{
	struct spi_transfer x = { };

	ctx->txbuf[0] = cmd;
	ctx->txbuf[1] = arg;
	mt6382_spi_xfer_init(&x, ctx->txbuf, NULL, 2, mt6382_tx_speed(ctx));
	return spi_sync_transfer(ctx->spi, &x, 1);
}

/* spislv_sync_sub(), spi_slave.c:107-238, for one 32-bit register */
static int mt6382_spi_access(struct mt6382 *ctx, u32 addr, u32 *val, bool read)
{
	unsigned int tries;
	u8 status;
	int ret;

	/* CR/CW, address, length - 1 */
	ctx->txbuf[0] = read ? SPISLV_CMD_CR : SPISLV_CMD_CW;
	put_unaligned_le32(addr, &ctx->txbuf[1]);
	put_unaligned_le32(4 - 1, &ctx->txbuf[5]);
	ret = mt6382_spi_cmd(ctx, 9, false, &status);
	if (ret)
		return ret;
	/* writing SPISLV_CTRL changes the sampling; its status is not valid */
	if (addr == SPISLV_CTRL && !read)
		status = SPISLV_SR_CONFIG_READY;
	if ((status & SPISLV_SR_CONFIG_READY) != SPISLV_SR_CONFIG_READY)
		return -EAGAIN;

	/* RD/WD and the data */
	memset(ctx->txbuf, 0, 5);
	ctx->txbuf[0] = read ? SPISLV_CMD_RD : SPISLV_CMD_WD;
	if (!read)
		put_unaligned_le32(*val, &ctx->txbuf[1]);
	ret = mt6382_spi_cmd(ctx, 5, read, &status);
	if (ret)
		return ret;
	if (addr == SPISLV_CTRL && !read)
		status = SPISLV_SR_CONFIG_READY | SPISLV_SR_RDWR_FINISH;

	if (status & (SPISLV_SR_RD_ERR | SPISLV_SR_WR_ERR | SPISLV_SR_TIMEOUT_ERR)) {
		/* clear the error (WS 0xff), spi_slave.c:196-215 */
		mt6382_spi_short(ctx, SPISLV_CMD_WS, 0xff);
		return -EAGAIN;
	}

	for (tries = 0; !(status & SPISLV_SR_RDWR_FINISH); tries++) {
		if (tries >= 10)
			return -ETIMEDOUT;
		usleep_range(1000, 1500);
		ret = mt6382_spi_status(ctx, &status);
		if (ret)
			return ret;
	}

	if (read)
		*val = get_unaligned_le32(&ctx->rxbuf[1]);
	return 0;
}

static int mt6382_spi_retry(struct mt6382 *ctx, u32 addr, u32 *val, bool read)
{
	int ret, tries;

	mutex_lock(&ctx->spi_lock);
	for (tries = 0; tries < 3; tries++) {
		ret = mt6382_spi_access(ctx, addr, val, read);
		if (ret != -EAGAIN && ret != -ETIMEDOUT)
			break;
	}
	mutex_unlock(&ctx->spi_lock);

	if (ret)
		dev_err_ratelimited(ctx->dev, "SPI %s 0x%05x failed: %d\n",
				    read ? "read" : "write", addr, ret);
	return ret == -EAGAIN ? -EIO : ret;
}

static int mt6382_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	return mt6382_spi_retry(context, reg, val, true);
}

static int mt6382_reg_write(void *context, unsigned int reg, unsigned int val)
{
	return mt6382_spi_retry(context, reg, &val, false);
}

static const struct regmap_range mt6382_ranges[] = {
	regmap_reg_range(0x00000, 0x000ff),	/* SYSREG */
	regmap_reg_range(0x02000, 0x02000),	/* SPI slave */
	regmap_reg_range(0x03000, 0x030ff),	/* TOPCKGEN */
	regmap_reg_range(0x04000, 0x042ff),	/* APMIXEDSYS */
	regmap_reg_range(0x05000, 0x05000),	/* IOCFG */
	regmap_reg_range(0x07000, 0x073ff),	/* GPIO */
	regmap_reg_range(0x09000, 0x09fff),	/* EFUSE */
	regmap_reg_range(0x0d000, 0x0d4ff),	/* DSI-2 device */
	regmap_reg_range(0x10000, 0x100ff),	/* GCE */
	regmap_reg_range(0x14000, 0x145ff),	/* RX D-PHY wrapper (OCLA) */
	regmap_reg_range(0x20000, 0x202ff),	/* DSC */
	regmap_reg_range(0x21000, 0x21dff),	/* DSI TX */
	regmap_reg_range(0x22000, 0x225ff),	/* MIPI TX */
	regmap_reg_range(0x23000, 0x233ff),	/* MMSYS */
	regmap_reg_range(0x24000, 0x240ff),	/* RDMA */
	regmap_reg_range(0x25000, 0x250ff),	/* MUTEX */
	regmap_reg_range(0x40000, MT6382_MAX_REGISTER),	/* RX D-PHY */
};

static const struct regmap_access_table mt6382_access_table = {
	.yes_ranges = mt6382_ranges,
	.n_yes_ranges = ARRAY_SIZE(mt6382_ranges),
};

static const struct regmap_config mt6382_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = MT6382_MAX_REGISTER,
	.reg_read = mt6382_reg_read,
	.reg_write = mt6382_reg_write,
	.rd_table = &mt6382_access_table,
	.wr_table = &mt6382_access_table,
	.cache_type = REGCACHE_NONE,
};

/*
 * After a reset: line mode, sampling and output drive of the slave
 * (spislv_init(), spi_slave.c:284-301). Gold's stock device tree gives
 * early_trans 0 at both speeds and slave-drive-strength 3.
 */
static int mt6382_spi_setup_slave(struct mt6382 *ctx)
{
	int ret;

	ctx->spi_fast = false;
	mutex_lock(&ctx->spi_lock);
	ret = mt6382_spi_short(ctx, SPISLV_CMD_CT, SPISLV_CT_4BYTE_SINGLE);
	mutex_unlock(&ctx->spi_lock);
	if (ret)
		return ret;
	ret = regmap_write(ctx->regmap, SPISLV_CTRL, 0x40);
	if (ret)
		return ret;
	return regmap_update_bits(ctx->regmap, IOCFG_DRV_CFG0, SPIS_SLVO,
				  FIELD_PREP(SPIS_SLVO, 3));
}

/* --------------------------------------------------------------------------
 * Bring-up sequence (bdg_common_init(), mtk_disp_bdg.c:5407-5567, then
 * mipi_dsi_rx_mac_init() and check_stopstate() as mtk_dsi.c calls them)
 */

/* calculate_datarate_cfgs_rx(), mtk_disp_bdg.c:3667-3794, the values in use */
enum mt6382_rx_var {
	RX_DDL_CNTR_REF,
	RX_MAX_PHASE,
	RX_DLL_FBK_COARSE,
	RX_SEL_FAST,
	RX_HSRX_CLK_DIV,
	RX_HS_THSSETTLE,
	RX_FJUMP_DESKEW,
	RX_EYE_OPEN_DESKEW,
	RX_NUM_VARS,
};

enum mt6382_rx_cond {
	RX_ANY,
	RX_GT_1500,	/* data rate above bdg_rx_v12 (1500 Mbps) */
	RX_LE_1500,
	RX_GT_2500,
	RX_LE_2500,
};

#define RXV(v)		(BIT(31) | (v))

struct mt6382_rx_write {
	u16 reg;	/* in 32-bit words from MIPI_RX_PHY_BASE */
	u32 mask;
	u32 val;	/* field value, or RXV() of a computed one */
	u8 cond;
};

/*
 * startup_seq_common() (mtk_disp_bdg.c:3834-3907, after the OCLA writes)
 * and startup_seq_dphy_specific() (mtk_disp_bdg.c:4266-5341) for a D-PHY
 * link, in source order, with the stock options (bdg_config_init(),
 * mtk_disp_bdg.c:149-208): hs-dco-enable, lp-tx-l023-enable and
 * g-mode-enable all 1. Register names are mt6382.h's.
 */
static const struct mt6382_rx_write mt6382_rx_startup[] = {
	/* startup_seq_common() */
	{ 0x1cf0, 0x000000fc, 63, RX_ANY },	/* CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0 */
	{ 0x0c11, 0x00000fff, 563, RX_ANY },	/* PPI_STARTUP_RW_COMMON_STARTUP_1_1 */
	{ 0x0c06, 0x000000ff, 39, RX_ANY },	/* PPI_STARTUP_RW_COMMON_DPHY_6 */
	{ 0x0c26, 0x000001ff, 500, RX_ANY },	/* PPI_CALIBCTRL_RW_COMMON_BG_0 */
	{ 0x0e40, 0x0000007f, 25, RX_ANY },	/* PPI_RW_TERMCAL_CFG_0 */
	{ 0x0e50, 0x0000001f, 5, RX_ANY },	/* PPI_RW_OFFSETCAL_CFG_0 */
	{ 0x0e01, 0x000003ff, 103, RX_ANY },	/* PPI_RW_LPDCOCAL_TIMEBASE */
	{ 0x0e02, 0x000007ff, 800, RX_ANY },	/* PPI_RW_LPDCOCAL_NREF */
	{ 0x0e03, 0x0000001f, 15, RX_ANY },	/* PPI_RW_LPDCOCAL_NREF_RANGE */
	{ 0x0e05, 0x0000fe00, 127, RX_ANY },	/* PPI_RW_LPDCOCAL_TWAIT_CONFIG */
	{ 0x0e05, 0x000001ff, 32, RX_ANY },	/* PPI_RW_LPDCOCAL_TWAIT_CONFIG */
	{ 0x0e06, 0x0000ff80, 32, RX_ANY },	/* PPI_RW_LPDCOCAL_VT_CONFIG */
	{ 0x0e06, 0x0000007c, 15, RX_ANY },	/* PPI_RW_LPDCOCAL_VT_CONFIG */
	{ 0x0e06, 0x00000002, 0, RX_ANY },	/* PPI_RW_LPDCOCAL_VT_CONFIG */
	{ 0x0e06, 0x00000001, 0, RX_ANY },	/* PPI_RW_LPDCOCAL_VT_CONFIG */
	{ 0x0e08, 0x00000003, 1, RX_ANY },	/* PPI_RW_LPDCOCAL_COARSE_CFG */
	{ 0x1c26, 0x00001000, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6 */
	{ 0x0e36, 0x00000003, 3, RX_ANY },	/* PPI_RW_COMMON_CFG */
	/* startup_seq_dphy_specific() */
	{ 0x1c47, 0x000003ff, 0, RX_ANY },	/* CORE_DIG_RW_COMMON_7 */
	{ 0x0c07, 0x000000ff, 40, RX_GT_1500 },	/* PPI_STARTUP_RW_COMMON_DPHY_7 */
	{ 0x0c07, 0x000000ff, 104, RX_LE_1500 },	/* PPI_STARTUP_RW_COMMON_DPHY_7 */
	{ 0x0c08, 0x000000ff, 80, RX_ANY },	/* PPI_STARTUP_RW_COMMON_DPHY_8 */
	{ 0x0e20, 0x000003ff, 125, RX_ANY },	/* PPI_RW_DDLCAL_CFG_0 */
	{ 0x0e27, 0x00001f80, 34, RX_ANY },	/* PPI_RW_DDLCAL_CFG_7 */
	{ 0x0e21, 0x0000ff00, 25, RX_ANY },	/* PPI_RW_DDLCAL_CFG_1 */
	{ 0x0e22, 0xffffffff, 0x4b19, RX_ANY },	/* PPI_RW_DDLCAL_CFG_2 */
	{ 0x0e24, 0x000003ff, 10, RX_ANY },	/* PPI_RW_DDLCAL_CFG_4 */
	{ 0x0e26, 0x000003ff, 10, RX_ANY },	/* PPI_RW_DDLCAL_CFG_6 */
	{ 0x0e27, 0x0000007f, 12, RX_ANY },	/* PPI_RW_DDLCAL_CFG_7 */
	{ 0x0e23, 0x000003ff, RXV(RX_DDL_CNTR_REF), RX_GT_1500 },	/* PPI_RW_DDLCAL_CFG_3 */
	{ 0x0e21, 0x000000ff, RXV(RX_MAX_PHASE), RX_GT_1500 },	/* PPI_RW_DDLCAL_CFG_1 */
	{ 0x0e25, 0x000003ff, RXV(RX_DLL_FBK_COARSE), RX_GT_1500 },	/* PPI_RW_DDLCAL_CFG_5 */
	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_8 */
	{ 0x1028, 0x00001000, RXV(RX_SEL_FAST), RX_ANY },
	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_8 */
	{ 0x1228, 0x00001000, RXV(RX_SEL_FAST), RX_ANY },
	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_8 */
	{ 0x1428, 0x00001000, RXV(RX_SEL_FAST), RX_ANY },
	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_8 */
	{ 0x1628, 0x00001000, RXV(RX_SEL_FAST), RX_ANY },
	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_8 */
	{ 0x1828, 0x00001000, RXV(RX_SEL_FAST), RX_ANY },
	{ 0x3040, 0x00000f00, 6, RX_ANY },	/* CORE_DIG_DLANE_0_RW_LP_0 */
	{ 0x3240, 0x00000f00, 6, RX_ANY },	/* CORE_DIG_DLANE_1_RW_LP_0 */
	{ 0x3440, 0x00000f00, 6, RX_ANY },	/* CORE_DIG_DLANE_2_RW_LP_0 */
	{ 0x3640, 0x00000f00, 6, RX_ANY },	/* CORE_DIG_DLANE_3_RW_LP_0 */
	{ 0x1022, 0x00000001, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2 */
	{ 0x1222, 0x00000001, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2 */
	{ 0x1422, 0x00000001, 1, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2 */
	{ 0x1622, 0x00000001, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2 */
	{ 0x1822, 0x00000001, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2 */
	{ 0x1c46, 0x0000003f, 9, RX_ANY },	/* CORE_DIG_RW_COMMON_6 */
	{ 0x102c, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12 */
	{ 0x122c, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12 */
	{ 0x142c, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12 */
	{ 0x162c, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12 */
	{ 0x182c, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12 */
	{ 0x102d, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13 */
	{ 0x122d, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13 */
	{ 0x142d, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13 */
	{ 0x162d, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13 */
	{ 0x182d, 0x00000002, 0, RX_GT_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13 */
	{ 0x102c, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12 */
	{ 0x122c, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12 */
	{ 0x142c, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12 */
	{ 0x162c, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12 */
	{ 0x182c, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12 */
	{ 0x102d, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13 */
	{ 0x122d, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13 */
	{ 0x142d, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13 */
	{ 0x162d, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13 */
	{ 0x182d, 0x00000002, 1, RX_LE_1500 },	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13 */
	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9 */
	{ 0x1429, 0x000000e0, RXV(RX_HSRX_CLK_DIV), RX_ANY },
	{ 0x3880, 0x000000ff, 28, RX_ANY },	/* CORE_DIG_DLANE_CLK_RW_HS_RX_0 */
	{ 0x3887, 0x000000ff, 6, RX_ANY },	/* CORE_DIG_DLANE_CLK_RW_HS_RX_7 */
	{ 0x3080, 0x0000ff00, RXV(RX_HS_THSSETTLE), RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_0 */
	{ 0x3280, 0x0000ff00, RXV(RX_HS_THSSETTLE), RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_0 */
	{ 0x3480, 0x0000ff00, RXV(RX_HS_THSSETTLE), RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_0 */
	{ 0x3680, 0x0000ff00, RXV(RX_HS_THSSETTLE), RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_0 */
	{ 0x3001, 0x00000004, 1, RX_GT_1500 },	/* CORE_DIG_DLANE_0_RW_CFG_1 */
	{ 0x3201, 0x00000004, 1, RX_GT_1500 },	/* CORE_DIG_DLANE_1_RW_CFG_1 */
	{ 0x3401, 0x00000004, 1, RX_GT_1500 },	/* CORE_DIG_DLANE_2_RW_CFG_1 */
	{ 0x3601, 0x00000004, 1, RX_GT_1500 },	/* CORE_DIG_DLANE_3_RW_CFG_1 */
	{ 0x3001, 0x00000008, 0, RX_GT_1500 },	/* CORE_DIG_DLANE_0_RW_CFG_1 */
	{ 0x3201, 0x00000008, 0, RX_GT_1500 },	/* CORE_DIG_DLANE_1_RW_CFG_1 */
	{ 0x3401, 0x00000008, 0, RX_GT_1500 },	/* CORE_DIG_DLANE_2_RW_CFG_1 */
	{ 0x3601, 0x00000008, 0, RX_GT_1500 },	/* CORE_DIG_DLANE_3_RW_CFG_1 */
	{ 0x3001, 0x00000004, 0, RX_LE_1500 },	/* CORE_DIG_DLANE_0_RW_CFG_1 */
	{ 0x3201, 0x00000004, 0, RX_LE_1500 },	/* CORE_DIG_DLANE_1_RW_CFG_1 */
	{ 0x3401, 0x00000004, 0, RX_LE_1500 },	/* CORE_DIG_DLANE_2_RW_CFG_1 */
	{ 0x3601, 0x00000004, 0, RX_LE_1500 },	/* CORE_DIG_DLANE_3_RW_CFG_1 */
	{ 0x3001, 0x00000008, 1, RX_LE_1500 },	/* CORE_DIG_DLANE_0_RW_CFG_1 */
	{ 0x3201, 0x00000008, 1, RX_LE_1500 },	/* CORE_DIG_DLANE_1_RW_CFG_1 */
	{ 0x3401, 0x00000008, 1, RX_LE_1500 },	/* CORE_DIG_DLANE_2_RW_CFG_1 */
	{ 0x3601, 0x00000008, 1, RX_LE_1500 },	/* CORE_DIG_DLANE_3_RW_CFG_1 */
	{ 0x3082, 0x00008000, 0, RX_GT_2500 },	/* CORE_DIG_DLANE_0_RW_HS_RX_2 */
	{ 0x3282, 0x00008000, 0, RX_GT_2500 },	/* CORE_DIG_DLANE_1_RW_HS_RX_2 */
	{ 0x3482, 0x00008000, 0, RX_GT_2500 },	/* CORE_DIG_DLANE_2_RW_HS_RX_2 */
	{ 0x3682, 0x00008000, 0, RX_GT_2500 },	/* CORE_DIG_DLANE_3_RW_HS_RX_2 */
	{ 0x3882, 0x00008000, 0, RX_GT_2500 },	/* CORE_DIG_DLANE_CLK_RW_HS_RX_2 */
	{ 0x3082, 0x00008000, 1, RX_LE_2500 },	/* CORE_DIG_DLANE_0_RW_HS_RX_2 */
	{ 0x3282, 0x00008000, 1, RX_LE_2500 },	/* CORE_DIG_DLANE_1_RW_HS_RX_2 */
	{ 0x3482, 0x00008000, 1, RX_LE_2500 },	/* CORE_DIG_DLANE_2_RW_HS_RX_2 */
	{ 0x3682, 0x00008000, 1, RX_LE_2500 },	/* CORE_DIG_DLANE_3_RW_HS_RX_2 */
	{ 0x3882, 0x00008000, 1, RX_LE_2500 },	/* CORE_DIG_DLANE_CLK_RW_HS_RX_2 */
	{ 0x3082, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_2 */
	{ 0x3282, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_2 */
	{ 0x3482, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_2 */
	{ 0x3682, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_2 */
	{ 0x3882, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_CLK_RW_HS_RX_2 */
	{ 0x3081, 0x000000ff, 16, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_1 */
	{ 0x3281, 0x000000ff, 16, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_1 */
	{ 0x3481, 0x000000ff, 16, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_1 */
	{ 0x3681, 0x000000ff, 16, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_1 */
	{ 0x3082, 0x00001e00, 3, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_2 */
	{ 0x3282, 0x00001e00, 3, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_2 */
	{ 0x3482, 0x00001e00, 3, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_2 */
	{ 0x3682, 0x00001e00, 3, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_2 */
	{ 0x3882, 0x00001e00, 3, RX_ANY },	/* CORE_DIG_DLANE_CLK_RW_HS_RX_2 */
	{ 0x3083, 0x00000007, 1, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_3 */
	{ 0x3283, 0x00000007, 1, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_3 */
	{ 0x3483, 0x00000007, 1, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_3 */
	{ 0x3683, 0x00000007, 1, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_3 */
	{ 0x3084, 0x0000ffff, 150, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_4 */
	{ 0x3284, 0x0000ffff, 150, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_4 */
	{ 0x3484, 0x0000ffff, 150, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_4 */
	{ 0x3684, 0x0000ffff, 150, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_4 */
	{ 0x3085, 0x000000ff, 0, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_5 */
	{ 0x3285, 0x000000ff, 0, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_5 */
	{ 0x3485, 0x000000ff, 0, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_5 */
	{ 0x3685, 0x000000ff, 0, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_5 */
	{ 0x3085, 0x0000ff00, 1, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_5 */
	{ 0x3285, 0x0000ff00, 1, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_5 */
	{ 0x3485, 0x0000ff00, 1, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_5 */
	{ 0x3685, 0x0000ff00, 1, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_5 */
	{ 0x3086, 0x000000ff, 2, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_6 */
	{ 0x3286, 0x000000ff, 2, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_6 */
	{ 0x3486, 0x000000ff, 2, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_6 */
	{ 0x3686, 0x000000ff, 2, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_6 */
	{ 0x3087, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_7 */
	{ 0x3287, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_7 */
	{ 0x3487, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_7 */
	{ 0x3687, 0x00002000, 1, RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_7 */
	{ 0x3083, 0x000001f8, RXV(RX_FJUMP_DESKEW), RX_ANY },	/* CORE_DIG_DLANE_0_RW_HS_RX_3 */
	{ 0x3283, 0x000001f8, RXV(RX_FJUMP_DESKEW), RX_ANY },	/* CORE_DIG_DLANE_1_RW_HS_RX_3 */
	{ 0x3483, 0x000001f8, RXV(RX_FJUMP_DESKEW), RX_ANY },	/* CORE_DIG_DLANE_2_RW_HS_RX_3 */
	{ 0x3683, 0x000001f8, RXV(RX_FJUMP_DESKEW), RX_ANY },	/* CORE_DIG_DLANE_3_RW_HS_RX_3 */
	/* CORE_DIG_DLANE_0_RW_HS_RX_6 */
	{ 0x3086, 0x0000ff00, RXV(RX_EYE_OPEN_DESKEW), RX_ANY },
	/* CORE_DIG_DLANE_1_RW_HS_RX_6 */
	{ 0x3286, 0x0000ff00, RXV(RX_EYE_OPEN_DESKEW), RX_ANY },
	/* CORE_DIG_DLANE_2_RW_HS_RX_6 */
	{ 0x3486, 0x0000ff00, RXV(RX_EYE_OPEN_DESKEW), RX_ANY },
	/* CORE_DIG_DLANE_3_RW_HS_RX_6 */
	{ 0x3686, 0x0000ff00, RXV(RX_EYE_OPEN_DESKEW), RX_ANY },
	{ 0x1ff0, 0xffffffff, 0x0404, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x040c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0414, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x041c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0423, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0429, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0430, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x043a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0445, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x044a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0450, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x045a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0465, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0469, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0472, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x047a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0485, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0489, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0490, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x049a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04a4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04ac, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04b4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04bc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04c4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04cc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04d4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04dc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04e4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04ec, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04f4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x04fc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0504, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x050c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0514, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x051c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0523, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0529, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0530, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x053a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0545, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x054a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0550, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x055a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0565, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0569, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0572, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x057a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0585, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0589, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0590, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x059a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05a4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05ac, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05b4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05bc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05c4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05cc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05d4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05dc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05e4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05ec, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05f4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x05fc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0604, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x060c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0614, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x061c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0623, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0629, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0632, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x063a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0645, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x064a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0650, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x065a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0665, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0669, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0672, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x067a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0685, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0689, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0690, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x069a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06a4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06ac, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06b4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06bc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06c4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06cc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06d4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06dc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06e4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06ec, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06f4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x06fc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0704, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x070c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0714, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x071c, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0723, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x072a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0730, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x073a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0745, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x074a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0750, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x075a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0765, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0769, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0772, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x077a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0785, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0789, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x0790, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x079a, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07a4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07ac, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07b4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07bc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07c4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07cc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07d4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07dc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07e4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07ec, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07f4, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1ff0, 0xffffffff, 0x07fc, RX_ANY },	/* CORE_DIG_COMMON_RW_DESKEW_FINE_MEM */
	{ 0x1029, 0x00000018, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9 */
	{ 0x1229, 0x00000018, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9 */
	{ 0x1429, 0x00000018, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9 */
	{ 0x1629, 0x00000018, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9 */
	{ 0x1829, 0x00000018, 0, RX_ANY },	/* CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9 */
};

static int mt6382_rx_values(u32 rate, u32 *v)
{
	/* calculate_datarate_cfgs_rx(), mtk_disp_bdg.c:3667-3794 */
	static const struct {
		u32 min_rate;
		u8 sel_fast, max_phase, dll_fbk, coarse_bank;
	} ddl[] = {
		{ 4500, 1, 63, 7, 0 },
		{ 4000, 1, 71, 8, 1 },
		{ 3600, 1, 79, 9, 1 },
		{ 3230, 1, 87, 10, 1 },
		{ 3000, 0, 71, 8, 0 },
		{ 2700, 0, 79, 9, 1 },
		{ 2455, 0, 87, 10, 1 },
		{ 2250, 0, 95, 11, 1 },
		{ 2077, 0, 103, 12, 1 },
		{ 1929, 0, 111, 13, 2 },
		{ 1800, 0, 119, 14, 2 },
		{ 1688, 0, 127, 15, 2 },
		{ 1588, 0, 135, 16, 2 },
		{ 1500, 0, 143, 17, 3 },
		{ 0, 0, 0, 0, 0 },
	};
	const u32 t_dco = 5, itminrx = 4;	/* ns, T_DCO and its 200 MHz */
	u32 hs_clk_freq = rate * 1000 / 2;	/* kHz */
	unsigned int i;
	int thssettle;

	for (i = 0; rate < ddl[i].min_rate; i++)
		;

	v[RX_DDL_CNTR_REF] = 5 * hs_clk_freq / 1000 / 2 / 16;
	v[RX_MAX_PHASE] = ddl[i].max_phase;
	v[RX_DLL_FBK_COARSE] = ddl[i].dll_fbk << 4 | ddl[i].coarse_bank;
	v[RX_SEL_FAST] = ddl[i].sel_fast;

	if (rate < 160)
		v[RX_HSRX_CLK_DIV] = 1;
	else if (rate < 320)
		v[RX_HSRX_CLK_DIV] = 2;
	else if (rate < 640)
		v[RX_HSRX_CLK_DIV] = 3;
	else if (rate < 1280)
		v[RX_HSRX_CLK_DIV] = 4;
	else if (rate < 2560)
		v[RX_HSRX_CLK_DIV] = 5;
	else
		v[RX_HSRX_CLK_DIV] = 6;

	thssettle = 115 + 1000 * 6 / rate;
	thssettle = (thssettle - t_dco - (itminrx + 3) * t_dco - 2 * t_dco) / t_dco - 1;
	if (thssettle < 0)
		return -EINVAL;
	v[RX_HS_THSSETTLE] = thssettle;
	v[RX_FJUMP_DESKEW] = ddl[i].max_phase / 10 / 4;
	v[RX_EYE_OPEN_DESKEW] = ddl[i].max_phase * 4 / 10 / 2;

	return 0;
}

static bool mt6382_rx_cond(u8 cond, u32 rate)
{
	switch (cond) {
	case RX_GT_1500:
		return rate > 1500;
	case RX_LE_1500:
		return rate <= 1500;
	case RX_GT_2500:
		return rate > 2500;
	case RX_LE_2500:
		return rate <= 2500;
	default:
		return true;
	}
}

static int mt6382_rx_phy_setup(struct mt6382 *ctx)
{
	static const u32 lane_con[] = {
		OCLA_LANEC_CON, OCLA_LANE0_CON, OCLA_LANE1_CON,
		OCLA_LANE2_CON, OCLA_LANE3_CON,
	};
	struct regmap *map = ctx->regmap;
	u32 rate = ctx->tm.rx_rate;
	u32 vars[RX_NUM_VARS];
	unsigned int i;
	int ret;

	ret = mt6382_rx_values(rate, vars);
	if (ret)
		return ret;

	/* startup_seq_common(), mtk_disp_bdg.c:3837-3839 */
	ret = regmap_write(map, OCLA_LANE_SWAP, 0x30213102);
	for (i = 0; i < ARRAY_SIZE(lane_con) && !ret; i++)	/* mipi_rx_enable() */
		ret = regmap_write(map, lane_con[i], 0);
	for (i = 0; i < ARRAY_SIZE(lane_con) && !ret; i++)	/* mipi_rx_set_forcerxmode() */
		ret = regmap_set_bits(map, lane_con[i], OCLA_FORCE_RX_MODE);

	for (i = 0; i < ARRAY_SIZE(mt6382_rx_startup) && !ret; i++) {
		const struct mt6382_rx_write *w = &mt6382_rx_startup[i];
		u32 val = w->val & BIT(31) ? vars[w->val & ~BIT(31)] : w->val;

		if (!mt6382_rx_cond(w->cond, rate))
			continue;
		if (w->mask == U32_MAX)
			ret = regmap_write(map, MIPI_RX_PHY(w->reg), val);
		else
			ret = regmap_update_bits(map, MIPI_RX_PHY(w->reg), w->mask,
						 (val << __ffs(w->mask)) & w->mask);
	}

	return ret;
}

/*
 * The bridge side of the timing, from the panel mode and lane rate, as
 * bdg_tx_phy_config(), bdg_tx_ps_ctrl(), bdg_tx_vdo_timing_set() and
 * bdg_dsi_line_timing_dphy_setting() compute it; then the AP side as
 * DSI_Config_VDO_Timing_with_DSC() (mtk_dsi.c:3362-3520) does: the AP's line
 * must take as long as the bridge's.
 */
static int mt6382_compute_timing(struct mt6382_timing *t)
{
	u32 ui, cycle, hs_trail, bpp_byte = 3, total, ap_words, ap_cycles;

	if (!t->tx_rate || t->tx_rate > 2500 || t->lanes < 1 || t->lanes > 4)
		return -EINVAL;

	/* bdg_tx_phy_config(), mtk_disp_bdg.c:1332-1410, in TX byte clocks */
	ui = 1000 / t->tx_rate;
	cycle = 8000 / t->tx_rate;
	t->lpx = max(DIV_ROUND_UP(60, cycle), 2U);
	t->hs_prpr = DIV_ROUND_UP(64 + 5 * ui, cycle) + 1;
	t->hs_zero = DIV_ROUND_UP(200 + 10 * ui, cycle);
	t->hs_zero = t->hs_zero > t->hs_prpr ? t->hs_zero - t->hs_prpr : t->hs_zero;
	t->hs_zero = max_t(u8, t->hs_zero, 1);
	hs_trail = 80 + 4 * ui;
	t->hs_trail = hs_trail > cycle ? DIV_ROUND_UP(hs_trail, cycle) + 1 : 2;
	t->da_hs_exit = 2 * t->lpx;
	t->ta_go = 4 * t->lpx;
	t->ta_get = 5 * t->lpx;
	t->ta_sure = 3 * t->lpx / 2;
	t->clk_hs_prpr = DIV_ROUND_UP(80, cycle);
	t->clk_zero = DIV_ROUND_UP(400, cycle) - t->clk_hs_prpr;
	t->clk_zero = max_t(u8, t->clk_zero, 1);
	t->clk_trail = max(DIV_ROUND_UP(100, cycle) + 1, 2U);
	t->clk_hs_exit = 2 * t->lpx;
	t->clk_hs_post = DIV_ROUND_UP(96 + 52 * ui, cycle);
	t->data_phy_cycle = t->da_hs_exit + 1 + t->lpx + t->hs_prpr + t->hs_zero + 1;
	/*
	 * Xiaomi's adjustment for gold's panel (HQ-299527, :1405-1408), which LK
	 * also makes: its PHY_TIMECON0/2 read back 0x0c120b09/0x0d2e0100.
	 */
	t->hs_trail -= 1;
	t->clk_trail -= 3;

	/*
	 * bdg_tx_vdo_timing_set(), :1591-1674: porches as for uncompressed
	 * RGB888, minus the packet overheads and the HS entry and exit.
	 */
	if (t->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		t->hsa_byte = t->hsa * bpp_byte - 10;
		t->hbp_byte = t->hbp * bpp_byte - 10;
		t->hfp_byte = t->hfp * bpp_byte - 12;
	} else if (t->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		t->hsa_byte = 0;
		t->hbp_byte = (t->hbp + t->hsa) * bpp_byte - 10;
		t->hfp_byte = t->hfp * bpp_byte - 12 - 6;
	} else {
		t->hsa_byte = 0;
		t->hbp_byte = (t->hbp + t->hsa) * bpp_byte - 10;
		t->hfp_byte = t->hfp * bpp_byte - 12;
	}
	t->bllp_byte = 16 * t->lanes;
	if (t->hfp_byte > t->data_phy_cycle * t->lanes)
		t->hfp_byte -= t->data_phy_cycle * t->lanes;
	else
		t->hfp_byte = 4;

	/* bdg_tx_ps_ctrl(), :1529-1589: the DSC stream, a third of RGB888 */
	t->ps_wc = DIV_ROUND_UP(t->hactive, 3) * bpp_byte;

	/* bdg_dsi_line_timing_dphy_setting(), :2034-2097 */
	if (t->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		total = 4 + (4 + t->hbp_byte + 2) + (4 + t->ps_wc + 2) +
			(4 + t->bllp_byte + 2) + (4 + t->hfp_byte + 2);
	else if (t->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		total = 4 + (4 + t->hsa_byte + 2) + 4 + (4 + t->hbp_byte + 2) +
			(4 + t->ps_wc + 2) + (4 + t->hfp_byte + 2);
	else
		total = 4 + (4 + t->hbp_byte + 2) + (4 + t->ps_wc + 2) +
			(4 + t->hfp_byte + 2);
	total += t->data_phy_cycle * t->lanes;
	t->line_cycle = DIV_ROUND_UP(total, t->lanes);

	/* the bridge's pipe must keep up: :2099-2112 */
	if (t->hactive * 1000 / MT6382_MM_CLK_MHZ > t->line_cycle * 8000 / t->tx_rate)
		return -ERANGE;

	/*
	 * The AP side. MediaTek's DSI host writes its line as byte counts
	 * (mtk_dsi.c:3435-3500); a mainline host derives them from the mode,
	 * so give it a mode whose line, at the RX rate, is as many byte clocks
	 * as stock's: ap_tx_line_cycle.
	 */
	t->rx_rate = t->tx_rate * MT6382_RXTX_RATIO / 100;
	ap_words = DIV_ROUND_UP(t->line_cycle * t->lanes * MT6382_RXTX_RATIO, 100);
	ap_cycles = DIV_ROUND_UP(ap_words, t->lanes);
	t->ap_htotal = DIV_ROUND_UP(ap_cycles * t->lanes, bpp_byte);

	return 0;
}

/* set_LDO_on(), mtk_disp_bdg.c:404-449 */
static int mt6382_ldo_on(struct mt6382 *ctx)
{
	struct regmap *map = ctx->regmap;
	u32 st, trim1, trim2, trim3;
	int ret;

	ret = regmap_set_bits(map, SYS_LDO_CTRL0, PHYLDO_MASKB);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_LDO_CTRL0, LDO_TRIM_BY_EFUSE);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(map, SYS_LDO_STATUS, st,
				       (st & 0x300) == 0x300, 10, 1000);
	if (ret) {
		dev_err(ctx->dev, "LDO not ready: 0x%x\n", st);
		return ret;
	}

	/* trim the LDO from the efuse when it has been fused */
	ret = regmap_read(map, EFUSE_STATUS, &st);
	if (!ret && (st & 0x7fffff)) {
		ret = regmap_read(map, EFUSE_TRIM1, &trim1);
		if (!ret)
			ret = regmap_read(map, EFUSE_TRIM2, &trim2);
		if (!ret)
			ret = regmap_read(map, EFUSE_TRIM3, &trim3);
		if (ret)
			return ret;
		dev_dbg(ctx->dev, "efuse 0x%x, trim 0x%x 0x%x 0x%x\n",
			st, trim1 & 0x3f, trim2 & 0xf, trim3 & 0xf);
		/* stock tests TRIM2 twice and not TRIM3 (:436) */
		if ((trim1 & 0x3f) || (trim2 & 0xf))
			ret = regmap_set_bits(map, SYS_LDO_CTRL0, LDO_TRIM_BY_EFUSE);
	}
	if (ret)
		return ret;

	ret = regmap_clear_bits(map, SYS_LDO_CTRL1, PHYLDO1_LP_EN);
	if (ret)
		return ret;
	return regmap_set_bits(map, SYS_LDO_CTRL1, PHYLDO2_EN);
}

/*
 * set_mtcmos_on(), ana_macro_on() with the 405 MHz MM clock, set_subsys_on();
 * mtk_disp_bdg.c:459-482, 615-667, 729-753
 */
static int mt6382_power_on(struct mt6382 *ctx)
{
	struct regmap *map = ctx->regmap;
	u32 val;
	int ret;

	/* set_mtcmos_on() */
	ret = regmap_set_bits(map, SYS_PWR_CTRL, PWR_ON);
	if (!ret)
		ret = regmap_set_bits(map, SYS_PWR_CTRL, PWR_ON_2ND);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_RST_CTRL, SW_RST_EN_DISP_PWR_WRAP);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_PWR_CTRL, PISO_EN);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_PWR_CTRL, DISP_PWR_CLK_DIS);
	if (!ret)
		ret = regmap_set_bits(map, SYS_PWR_CTRL, SYSBUF_SRAM_SLEEP_B);
	if (!ret)
		ret = regmap_set_bits(map, SYS_PWR_CTRL, GCE_SRAM_SLEEP_B);
	if (!ret)
		ret = regmap_read(map, SYS_PWR_CTRL, &val);
	if (!ret)
		ret = regmap_write(map, SYS_PWR_CTRL, (val | 0x00880000) & 0xffbbefff);
	if (ret)
		return ret;

	/* ana_macro_on(): the main PLL, then the clock muxes */
	ret = regmap_write(map, AP_MAINPLL_CON3, 3);
	if (ret)
		return ret;
	udelay(1);
	ret = regmap_write(map, AP_MAINPLL_CON3, 1);
	if (!ret)
		ret = regmap_write(map, AP_MAINPLL_CON0, 0xff000000);
	if (!ret)
		ret = regmap_write(map, AP_MAINPLL_CON1, 0x000fc000);
	if (ret)
		return ret;
	udelay(2);
	ret = regmap_write(map, AP_MAINPLL_CON0, 0xff000001);
	if (ret)
		return ret;
	usleep_range(25, 50);
	ret = regmap_write(map, AP_MAINPLL_CON0, 0xff800001);
	if (!ret)
		ret = regmap_write(map, CK_CLK_CFG_0_CLR, 0xffffffff);
	/* bits 17:16: MM clock 1 = 270, 2 = 405, 3 = 540 MHz */
	if (!ret)
		ret = regmap_write(map, CK_CLK_CFG_0_SET, 3 << 24 | 2 << 16 | 1 << 8 | 1);
	if (!ret)
		ret = regmap_write(map, CK_CLK_CFG_UPDATE, BIT(4) | BIT(3) | BIT(1) | BIT(0));
	if (ret)
		return ret;
	udelay(6);

	/* set_subsys_on(): clocks on, DCM off */
	ret = regmap_read(map, GCE_CTL_INT0, &val);
	if (!ret)
		ret = regmap_write(map, GCE_CTL_INT0, val & 0x0000ffff);
	if (!ret)
		ret = regmap_write(map, MM_CG_CON0, 0);
	if (!ret)
		ret = regmap_clear_bits(map, EFUSE_DCM_ON, 0x7);
	if (!ret)
		ret = regmap_set_bits(map, GCE_CTL_INT0, 0x0000ffff);
	if (!ret)
		ret = regmap_write(map, MM_HW_DCM_1ST_DIS0, 0xffffffff);
	if (!ret)
		ret = regmap_write(map, MM_HW_DCM_2ND_DIS0, 0xffffffff);
	if (!ret)	/* 26 MHz to the display subsystem */
		ret = regmap_write(map, SYS_DISP_MISC1, 1);

	return ret;
}

/* bdg_mipi_tx_dphy_clk_setting(), mtk_disp_bdg.c:934-1235, no lane swap or SSC */
static int mt6382_tx_pll_on(struct mt6382 *ctx)
{
	static const u32 sw_ctl_en[] = {
		MIPITX_D0_SW_CTL_EN, MIPITX_D1_SW_CTL_EN, MIPITX_D2_SW_CTL_EN,
		MIPITX_D3_SW_CTL_EN, MIPITX_CK_SW_CTL_EN,
	};
	struct regmap *map = ctx->regmap;
	u32 rate = ctx->tm.tx_rate, ratio, posdiv, pcw, rem;
	unsigned int i;
	int ret;

	if (rate >= 2000) {
		ratio = 1;
		posdiv = 0;
	} else if (rate >= 1000) {
		ratio = 2;
		posdiv = 1;
	} else if (rate >= 500) {
		ratio = 4;
		posdiv = 2;
	} else if (rate > 250) {
		ratio = 8;
		posdiv = 3;
	} else if (rate >= 125) {
		ratio = 16;
		posdiv = 4;
	} else {
		return -EINVAL;
	}

	/* dsi_get_pcw(), :915-932: 26 MHz reference, 7.24 fixed point */
	rem = rate * ratio % 26;
	pcw = (rate * ratio / 26) << 24 |
	      ((256 * rem / 26) & 0xff) << 16 |
	      ((256 * (256 * rem % 26) / 26) & 0xff) << 8 |
	      ((256 * (256 * (256 * rem % 26) % 26) / 26) & 0xff);

	ret = regmap_set_bits(map, MIPITX_CK_CKMODE_EN, MIPITX_CKMODE_EN);
	if (!ret)
		ret = regmap_write(map, MIPITX_PLL_CON4, 0x00ff12e0);
	if (!ret)
		ret = regmap_write(map, MIPITX_LANE_CON, 0x3fff0180);
	if (ret)
		return ret;
	udelay(2);
	ret = regmap_write(map, MIPITX_LANE_CON, 0x3fff0080);
	for (i = 0; i < ARRAY_SIZE(sw_ctl_en) && !ret; i++)
		ret = regmap_clear_bits(map, sw_ctl_en[i], MIPITX_SW_CTL_EN);
	if (!ret)
		ret = regmap_set_bits(map, MIPITX_PLL_PWR, MIPITX_PLL_SDM_PWR_ON);
	if (ret)
		return ret;
	udelay(2);
	ret = regmap_clear_bits(map, MIPITX_PLL_PWR, MIPITX_PLL_SDM_ISO_EN);
	if (!ret)
		ret = regmap_write(map, MIPITX_PLL_CON0, pcw);
	if (!ret)
		ret = regmap_update_bits(map, MIPITX_PLL_CON1, MIPITX_PLL_POSDIV,
					 FIELD_PREP(MIPITX_PLL_POSDIV, posdiv));
	if (!ret)
		ret = regmap_clear_bits(map, MIPITX_PLL_CON2, MIPITX_PLL_SDM_SSC_EN);
	if (!ret)
		ret = regmap_set_bits(map, MIPITX_PLL_CON1, MIPITX_PLL_EN);
	if (ret)
		return ret;
	usleep_range(50, 100);
	ret = regmap_set_bits(map, MIPITX_SW_CTRL_CON4, MIPITX_SW_ANA_CK_EN);
	if (ret)
		return ret;
	usleep_range(20, 40);	/* bdg_tx_init(), :3636 */

	return 0;
}

/* bdg_tx_init() after the PLL, mtk_disp_bdg.c:3625-3649 */
static int mt6382_tx_setup(struct mt6382 *ctx)
{
	const struct mt6382_timing *t = &ctx->tm;
	struct regmap *map = ctx->regmap;
	u32 mode = t->mode_flags, rw_times, w, txrx;
	int ret;

	/* bdg_tx_phy_config(), :1412-1472 */
	ret = regmap_update_bits(map, TX_PHY_TIMECON0,
				 TX_LPX | TX_HS_PRPR | TX_HS_ZERO | TX_HS_TRAIL,
				 FIELD_PREP(TX_LPX, t->lpx) |
				 FIELD_PREP(TX_HS_PRPR, t->hs_prpr) |
				 FIELD_PREP(TX_HS_ZERO, t->hs_zero) |
				 FIELD_PREP(TX_HS_TRAIL, t->hs_trail));
	if (!ret)
		ret = regmap_update_bits(map, TX_PHY_TIMECON1,
					 TX_TA_GO | TX_TA_SURE | TX_TA_GET | TX_DA_HS_EXIT,
					 FIELD_PREP(TX_TA_GO, t->ta_go) |
					 FIELD_PREP(TX_TA_SURE, t->ta_sure) |
					 FIELD_PREP(TX_TA_GET, t->ta_get) |
					 FIELD_PREP(TX_DA_HS_EXIT, t->da_hs_exit));
	if (!ret)
		ret = regmap_update_bits(map, TX_PHY_TIMECON2,
					 TX_CLK_ZERO | TX_CLK_TRAIL,
					 FIELD_PREP(TX_CLK_ZERO, t->clk_zero) |
					 FIELD_PREP(TX_CLK_TRAIL, t->clk_trail));
	if (!ret)
		ret = regmap_update_bits(map, TX_PHY_TIMECON3,
					 TX_CLK_HS_PRPR | TX_CLK_HS_POST | TX_CLK_HS_EXIT,
					 FIELD_PREP(TX_CLK_HS_PRPR, t->clk_hs_prpr) |
					 FIELD_PREP(TX_CLK_HS_POST, t->clk_hs_post) |
					 FIELD_PREP(TX_CLK_HS_EXIT, t->clk_hs_exit));

	/*
	 * bdg_tx_txrx_ctrl(), :1480-1527. Xiaomi forces HSTX_CKLP_EN on for
	 * gold (HQ-305688); here it follows the panel's clock flag.
	 */
	txrx = FIELD_PREP(TX_LANE_NUM, GENMASK(t->lanes - 1, 0));
	if (mode & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		txrx |= TX_HSTX_CKLP_EN;
	if (mode & MIPI_DSI_MODE_NO_EOT_PACKET)
		txrx |= TX_DIS_EOT;
	if (!(mode & MIPI_DSI_MODE_VIDEO))
		txrx |= TX_EXT_TE_EN;
	if (!ret)
		ret = regmap_update_bits(map, TX_TXRX_CON, TX_LANE_NUM | TX_HSTX_CKLP_EN |
					 TX_DIS_EOT | TX_EXT_TE_EN, txrx);
	if (!ret)
		ret = regmap_set_bits(map, TX_INTEN, TX_INT_BUFFER_UNDERRUN);

	/* bdg_tx_ps_ctrl(), :1563-1586 */
	w = t->ps_wc / 3;
	if (!ret)
		ret = regmap_update_bits(map, TX_VACT_NL, TX_VACT_NL_MASK, t->vactive);
	if (!ret)
		ret = regmap_update_bits(map, TX_PSCON,
					 TX_CUSTOM_HEADER | TX_PS_WC | TX_PS_SEL,
					 FIELD_PREP(TX_PS_WC, t->ps_wc) |
					 FIELD_PREP(TX_PS_SEL, TX_PS_SEL_COMPRESSION));
	if (!ret)
		ret = regmap_update_bits(map, TX_SIZE_CON, TX_WIDTH | TX_HEIGHT,
					 FIELD_PREP(TX_WIDTH, w) |
					 FIELD_PREP(TX_HEIGHT, t->vactive));

	/* bdg_tx_vdo_timing_set(), :1664-1671 */
	if (!ret)
		ret = regmap_write(map, TX_VSA_NL, t->vsa);
	if (!ret)
		ret = regmap_write(map, TX_VBP_NL, t->vbp);
	if (!ret)
		ret = regmap_write(map, TX_VFP_NL, t->vfp);
	if (!ret)
		ret = regmap_write(map, TX_HSA_WC, t->hsa_byte);
	if (!ret)
		ret = regmap_write(map, TX_HBP_WC, t->hbp_byte);
	if (!ret)
		ret = regmap_write(map, TX_HFP_WC, t->hfp_byte);
	if (!ret)
		ret = regmap_write(map, TX_BLLP_WC, t->bllp_byte);

	/* bdg_tx_buf_rw_set(), :1676-1715, video mode */
	rw_times = DIV_ROUND_UP(w * t->vactive * 3, 9);
	if (!ret)
		ret = regmap_write(map, TX_BUF_RW_TIMES, rw_times);
	if (!ret)
		ret = regmap_set_bits(map, TX_BUF_CON0, TX_ANTI_LATENCY_BUF_EN);

	/* bdg_tx_enable_hs_clk(), then bdg_tx_set_mode(NULL): command mode */
	if (!ret)
		ret = regmap_set_bits(map, TX_PHY_LCCON, TX_LC_HS_TX_EN);
	if (!ret)
		ret = regmap_update_bits(map, TX_MODE_CON, TX_MODE, TX_MODE_CMD);
	if (!ret)
		ret = regmap_clear_bits(map, MM_MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL);

	return ret;
}

/* bdg_dsc_init(), mtk_disp_bdg.c:3961-4088, from the panel's DSC config */
static int mt6382_dsc_setup(struct mt6382 *ctx)
{
	const struct drm_dsc_config *dsc = &ctx->dsc;
	struct regmap *map = ctx->regmap;
	u32 width = ctx->tm.hactive, height = ctx->tm.vactive;
	u32 delay, slices_v, pps[20] = { }, range;
	unsigned int i;
	int ret;

	if (dsc->pic_width != width || dsc->pic_height != height)
		return -EINVAL;

	ret = regmap_set_bits(map, DSC_CON, DSC_CON_UFOE_SEL);
	if (!ret)	/* empty flag always high */
		ret = regmap_update_bits(map, DSC_CON, DSC_CON_EMPTY_FLAG_SEL,
					 FIELD_PREP(DSC_CON_EMPTY_FLAG_SEL, 1));
	if (!ret)	/* output buffer as for FHD(+) */
		ret = regmap_write(map, DSC_OBUF, 0x800002c2);
	if (ret)
		return ret;

	delay = DIV_ROUND_UP((128 + (dsc->initial_xmit_delay + 2) / 3) * 3,
			     dsc->slice_width);
	ret = regmap_write(map, DSC_MODE, (dsc->slice_count - 1) |
			   min(delay, 15U) << 8 | BIT(16));
	if (!ret)
		ret = regmap_write(map, DSC_PIC_W, (DIV_ROUND_UP(width, 3) - 1) << 16 | width);
	slices_v = DIV_ROUND_UP(height, dsc->slice_height);
	if (!ret)
		ret = regmap_write(map, DSC_PIC_H,
				   (slices_v * dsc->slice_height - 1) << 16 | (height - 1));
	if (!ret)
		ret = regmap_write(map, DSC_SLICE_W,
				   (DIV_ROUND_UP(dsc->slice_width, 3) - 1) << 16 |
				   dsc->slice_width);
	if (!ret)
		ret = regmap_write(map, DSC_SLICE_H,
				   (dsc->slice_width % 3) << 30 | (slices_v - 1) << 16 |
				   (dsc->slice_height - 1));
	if (!ret)
		ret = regmap_write(map, DSC_CHUNK_SIZE, dsc->slice_chunk_size);
	if (!ret)
		ret = regmap_write(map, DSC_BUF_SIZE,
				   dsc->slice_chunk_size * dsc->slice_height);
	if (!ret)
		ret = regmap_write(map, DSC_PAD, DIV_ROUND_UP(dsc->slice_chunk_size, 3) * 3 -
				   dsc->slice_chunk_size);
	if (!ret)	/* the stock default; gold's panel gives the same */
		ret = regmap_write(map, DSC_CFG, 0x22);
	if (!ret)
		ret = regmap_update_bits(map, DSC_SHADOW, DSC_VERSION_MINOR,
					 FIELD_PREP(DSC_VERSION_MINOR,
						    dsc->dsc_version_minor == 2 ? 2 : 1));
	if (ret)
		return ret;

	/* the PPS, in MediaTek's register layout (:4041-4085) */
	pps[0] = dsc->line_buf_depth | dsc->bits_per_component << 4 |
		 dsc->bits_per_pixel << 8 | dsc->convert_rgb << 18 |
		 dsc->block_pred_enable << 19;
	pps[1] = dsc->initial_xmit_delay | dsc->initial_dec_delay << 16;
	pps[2] = dsc->initial_scale_value | dsc->scale_increment_interval << 16;
	pps[3] = dsc->scale_decrement_interval | dsc->first_line_bpg_offset << 16;
	pps[4] = dsc->nfl_bpg_offset | dsc->slice_bpg_offset << 16;
	pps[5] = dsc->initial_offset | dsc->final_offset << 16;
	pps[6] = dsc->flatness_min_qp | dsc->flatness_max_qp << 8 |
		 dsc->rc_model_size << 16;
	pps[7] = dsc->rc_edge_factor | dsc->rc_quant_incr_limit0 << 8 |
		 dsc->rc_quant_incr_limit1 << 16 | dsc->rc_tgt_offset_high << 24 |
		 dsc->rc_tgt_offset_low << 28;
	/* stock writes these as constants for 8 bpc, 8 bpp; they are the tables */
	for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i++)
		pps[8 + i / 4] |= dsc->rc_buf_thresh[i] << (8 * (i % 4));
	for (i = 0; i < DSC_NUM_BUF_RANGES; i++) {
		range = dsc->rc_range_params[i].range_min_qp |
			dsc->rc_range_params[i].range_max_qp << 5 |
			dsc->rc_range_params[i].range_bpg_offset << 10;
		pps[12 + i / 2] |= range << (16 * (i % 2));
	}
	for (i = 0; i < ARRAY_SIZE(pps) && !ret; i++)
		ret = regmap_write(map, DSC_PPS(i), pps[i]);

	return ret;
}

/* mipi_dsi_rx_mac_init(), mtk_disp_bdg.c:4091-4264, before the PHY wait */
static int mt6382_rx_mac_setup(struct mt6382 *ctx)
{
	/* (offset from DSI2_BASE, value); in source order */
	static const u16 mac_init[][2] = {
		{ 0x000, 0 },		/* VERSION */
		{ 0x008, U16_MAX },	/* N_LANES: lanes - 1 */
		{ 0x00c, 0 },		/* INT_ST_MAIN */
		{ 0x010, 4 },		/* EOTP_CFG: HS RX EoTp */
		{ 0x014, 0 },		/* CTRL_CFG */
		{ 0x020, 0 },		/* FIFO_STATUS */
		{ 0x10c, 0 },		/* PHY_MODE: D-PHY */
		{ 0x118, 0 },		/* PHY_TEST_CTRL0 */
		{ 0x11c, 0 },		/* PHY_TEST_CTRL1 */
		{ 0x124, 0 },		/* PHY_DATA_STATUS */
		{ 0x140, 0 },		/* LPTXRDY_TO_CNT */
		{ 0x144, 0 },		/* LPTX_TO_CNT */
		{ 0x148, 0 },		/* HSRX_TO_CNT */
	};
	/* status, mask, force for PHY_FATAL ... RX_TRIGGERS (:4152-4205) */
	static const u16 int_groups[] = {
		0x200, 0x210, 0x220, 0x230, 0x240, 0x250, 0x260, 0x280, 0x2b0, 0x2d0,
	};
	const struct mt6382_timing *t = &ctx->tm;
	struct regmap *map = ctx->regmap;
	u32 t_hact_ipi, t_hact_ppi, delay, val;
	unsigned int i;
	int ret;

	ret = regmap_write(map, SYS_DISP_MISC0, 1);
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x008, 3);		/* N_LANES */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x100, 1);		/* PHY_SHUTDOWNZ */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x104, 1);		/* PHY_RSTZ */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x004, 0);		/* SOFT_RSTN */
	if (ret)
		return ret;
	udelay(1);
	ret = regmap_write(map, DSI2_BASE + 0x004, 1);

	for (i = 0; i < ARRAY_SIZE(mac_init) && !ret; i++) {
		val = mac_init[i][1] == U16_MAX ? t->lanes - 1 : mac_init[i][1];
		ret = regmap_write(map, DSI2_BASE + mac_init[i][0], val);
	}
	for (i = 0; i < ARRAY_SIZE(int_groups) && !ret; i++) {
		ret = regmap_write(map, DSI2_BASE + int_groups[i], 0);
		if (!ret)
			ret = regmap_write(map, DSI2_BASE + int_groups[i] + 4, 0xffffffff);
		if (!ret)
			ret = regmap_write(map, DSI2_BASE + int_groups[i] + 8, 0);
	}

	/* DDI (command) path: VC 0-3, clock manager 0x37 for the 405 MHz MM clock */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x300, 0);	/* DDI_RDY_TO_CNT */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x304, 0);	/* DDI_RESP_TO_CNT */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x310, 0xf);	/* DDI_VALID_VC_CFG */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x314, 0x37);	/* DDI_CLK_MGR_CFG */

	/* IPI (video) path: mode, VC 0, and when to start sending the line */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x400, 0);	/* IPI_MODE_CFG */
	if (!ret && !(t->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE))
		ret = regmap_write(map, DSI2_BASE + 0x400, 1);
	if (ret)
		return ret;

	t_hact_ipi = t->hactive * 1000 / MT6382_MM_CLK_MHZ;
	t_hact_ppi = (6 + t->hactive * 3) * 8000 / t->rx_rate / t->lanes;
	if (t_hact_ppi > t_hact_ipi)
		delay = ((t_hact_ppi - t_hact_ipi) * MT6382_MM_CLK_MHZ +
			 20 * 8000 * MT6382_MM_CLK_MHZ / t->rx_rate) / 1000 + 4;
	else
		delay = 20 * 8000 * MT6382_MM_CLK_MHZ / t->rx_rate / 1000 + 4;

	ret = regmap_write(map, DSI2_BASE + 0x404, 0);	/* IPI_VALID_VC_CFG */
	if (!ret)
		ret = regmap_write(map, DSI2_BASE + 0x408, delay);	/* IPI_TX_DELAY */

	return ret;
}

/* Wait until a register reads a value six times in a row (:4243-4260, :3909-3936) */
static int mt6382_poll_stable(struct mt6382 *ctx, u32 reg, u32 mask)
{
	unsigned int i, seen = 0;
	u32 val;
	int ret;

	for (i = 0; i < 5000; i++) {
		ret = regmap_read(ctx->regmap, reg, &val);
		if (ret)
			return ret;
		if ((val & mask) == mask && ++seen > 5)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

/*
 * Everything bdg_common_init() and mipi_dsi_rx_mac_init() do, from reset to
 * a TX in command mode with the RX on the DDI path. The AP's DSI host must
 * already be in LP-11 (check_stopstate(), which mtk_dsi.c:4889 calls after
 * powering on its PHY).
 */
static int mt6382_bring_up(struct mt6382 *ctx)
{
	struct regmap *map = ctx->regmap;
	int ret;

	/* bdg_tx_pull_6382_reset_pin(), :370-386 */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10, 20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10, 20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10, 20);
	ctx->state = MT6382_OFF;

	ret = mt6382_spi_setup_slave(ctx);
	if (ret)
		return ret;

	/* bdg_clk_buf_nfc(1), :6005: pass the 26 MHz on to the NFC controller */
	ret = regmap_write(map, SYS_CKBUF_CTRL, FIELD_PREP(CKBUF_NFC_TERM, 2) |
			   CKBUF_NFC_CK_OUT_EN);
	if (!ret)
		ret = mt6382_ldo_on(ctx);
	if (!ret)
		ret = mt6382_power_on(ctx);
	if (ret)
		return ret;
	ctx->spi_fast = true;	/* spislv_switch_speed_hz(), :5466 */

	/* :5470-5520 */
	ret = regmap_write(map, SYS_RST_DG_CTRL, 0);	/* reset de-glitch off */
	if (!ret)	/* GPIO12 as the IRQ output */
		ret = regmap_update_bits(map, GPIO_MODE1, GPIO_MODE1_GPIO12,
					 FIELD_PREP(GPIO_MODE1_GPIO12, 1));
	if (!ret)	/* release the RX MAC reset */
		ret = regmap_write(map, SYS_DISP_MISC0, 1);
	if (!ret)
		ret = regmap_write(map, SYS_DISP_MISC0, 0);
	if (!ret)
		ret = regmap_write(map, MM_DDI_POST_CTRL, 0x00001100);
	if (!ret)	/* TE out bypassed, TE in from the panel */
		ret = regmap_update_bits(map, GPIO_MODE1, GPIO_MODE1_TE,
					 FIELD_PREP(GPIO_MODE1_TE, 0x31));
	if (!ret)
		ret = regmap_set_bits(map, MM_MIPI_RX_POST_CTRL, MIPI_RX_MODE_SEL);
	if (!ret)
		ret = regmap_write(map, RDMA_SIZE_CON_0, ctx->tm.hactive);
	if (!ret)
		ret = regmap_write(map, RDMA_SIZE_CON_1, ctx->tm.vactive);
	if (!ret)
		ret = regmap_write(map, RDMA_FIFO_CON, 0xf30 << 16);
	if (!ret)
		ret = regmap_write(map, RDMA_STALL_CG_CON, 0);
	if (!ret)
		ret = regmap_update_bits(map, RDMA_SRAM_SEL, RDMA_SRAM_SEL_SEL, 0);
	if (!ret)
		ret = regmap_set_bits(map, RDMA_GLOBAL_CON, RDMA_ENGINE_EN);
	if (!ret)
		ret = regmap_write(map, MUTEX_INTEN, BIT(16));
	if (!ret)
		ret = regmap_write(map, MUTEX0_MOD0, 0x1f);
	if (!ret)	/* SOF and EOF from the DSI TX */
		ret = regmap_write(map, MUTEX0_CTL, BIT(0) | BIT(6));
	if (!ret)
		ret = regmap_set_bits(map, MUTEX0_EN, MUTEX0_EN_EN);
	if (ret)
		return ret;

	ret = mt6382_tx_pll_on(ctx);
	if (!ret)
		ret = mt6382_tx_setup(ctx);
	if (!ret)
		ret = regmap_write(map, TX_RESYNC_CON, 0x50007);
	if (!ret)
		ret = mt6382_dsc_setup(ctx);
	if (!ret)
		ret = regmap_set_bits(map, DSC_CON, DSC_CON_PT_MEM_EN);
	if (!ret)
		ret = regmap_set_bits(map, DSC_CON, DSC_CON_EN);
	if (!ret)
		ret = mt6382_rx_phy_setup(ctx);
	if (!ret)	/* :5555: from here on at the NFC controller's request (0x12) */
		ret = regmap_write(map, SYS_CKBUF_CTRL, FIELD_PREP(CKBUF_NFC_TERM, 2) |
				   CKBUF_EXT_CLK_REQ_EN);
	if (!ret)	/* output_debug_signal(), :5368: outside its debug #ifdef */
		ret = regmap_write(map, GPIO_MODE1, 0x31111111);
	if (!ret)
		ret = mt6382_rx_mac_setup(ctx);
	if (ret)
		return ret;

	ret = mt6382_poll_stable(ctx, OCLA_PHY_READY, BIT(0));
	if (ret) {
		dev_err(ctx->dev, "RX PHY not ready\n");
		return ret;
	}

	/* check_stopstate(), :3909-3936: the AP's lane 0 in LP-11 */
	ret = mt6382_poll_stable(ctx, OCLA_LANE0_STOPSTATE, BIT(0));
	if (ret) {
		dev_err(ctx->dev, "no LP-11 from the DSI host\n");
		return ret;
	}
	ret = regmap_clear_bits(map, OCLA_LANEC_CON, OCLA_FORCE_RX_MODE);
	if (!ret)
		ret = regmap_clear_bits(map, OCLA_LANE0_CON, OCLA_FORCE_RX_MODE);
	if (!ret)
		ret = regmap_clear_bits(map, OCLA_LANE1_CON, OCLA_FORCE_RX_MODE);
	if (!ret)
		ret = regmap_clear_bits(map, OCLA_LANE2_CON, OCLA_FORCE_RX_MODE);
	if (!ret)
		ret = regmap_clear_bits(map, OCLA_LANE3_CON, OCLA_FORCE_RX_MODE);
	if (!ret)
		ctx->state = MT6382_CMD;

	return ret;
}

/* set_ana_mipi_dsi_off(), :782-913: the TX lanes to ULPS, then its PHY off */
static int mt6382_tx_off(struct mt6382 *ctx)
{
	/* in stock's order */
	static const u32 lanes[] = {
		MIPITX_D2_SW_CTL_EN, MIPITX_D0_SW_CTL_EN, MIPITX_CK_SW_CTL_EN,
		MIPITX_D1_SW_CTL_EN, MIPITX_D3_SW_CTL_EN,
	};
	struct regmap *map = ctx->regmap;
	unsigned int i, n;
	u32 val;
	int ret;

	ret = regmap_clear_bits(map, TX_PHY_LCCON, TX_LC_HS_TX_EN);
	if (!ret)
		ret = regmap_clear_bits(map, TX_PHY_LD0CON, TX_L0_ULPM_EN);
	if (!ret)
		ret = regmap_clear_bits(map, TX_PHY_LCCON, TX_LC_ULPM_EN);
	if (!ret)
		ret = regmap_set_bits(map, TX_PHY_LD0CON, TX_LX_ULPM_AS_L0);
	if (!ret)
		ret = regmap_set_bits(map, TX_PHY_LD0CON, TX_L0_ULPM_EN);
	if (!ret)
		ret = regmap_set_bits(map, TX_PHY_LCCON, TX_LC_ULPM_EN);
	if (ret)
		return ret;

	/* stock goes on when the lanes do not report ULPS (:813-821) */
	if (regmap_read_poll_timeout(map, TX_INTSTA, val, val & TX_INT_SLEEPIN_ULPS,
				     10, 10000))
		dev_warn(ctx->dev, "TX did not enter ULPS: 0x%x\n", val);

	/* LP-00 under software control */
	for (i = 0; i < ARRAY_SIZE(lanes); i++)
		for (n = 1; n <= 4 && !ret; n++)
			ret = regmap_clear_bits(map, MIPITX_SW_LPTX(lanes[i], n), BIT(0));
	for (i = 0; i < ARRAY_SIZE(lanes) && !ret; i++)
		ret = regmap_set_bits(map, lanes[i], MIPITX_SW_CTL_EN);
	if (!ret)
		ret = regmap_update_bits(map, TX_TXRX_CON, TX_LANE_NUM, 0);
	if (!ret)
		ret = regmap_clear_bits(map, MIPITX_PLL_CON1, MIPITX_PLL_EN);
	if (!ret)
		ret = regmap_set_bits(map, MIPITX_PLL_PWR, MIPITX_PLL_SDM_ISO_EN);
	if (!ret)
		ret = regmap_clear_bits(map, MIPITX_PLL_PWR, MIPITX_PLL_SDM_PWR_ON);
	if (!ret)
		ret = regmap_set_bits(map, MIPITX_LANE_CON, MIPITX_PAD_TIEL_SEL);
	if (!ret)
		ret = regmap_clear_bits(map, MIPITX_LANE_CON, MIPITX_BG_LPF_EN);
	if (!ret)
		ret = regmap_clear_bits(map, MIPITX_LANE_CON, MIPITX_BG_CORE_EN);

	return ret;
}

/*
 * ana_macro_off() with set_pll_off(), then set_mtcmos_off() and set_LDO_off();
 * :451-457, 484-507, 562-613, 669-689
 */
static int mt6382_power_off(struct mt6382 *ctx)
{
	struct regmap *map = ctx->regmap;
	u32 val;
	int ret;

	/* the RX D-PHY */
	ret = regmap_write(map, DSI2_PHY_SHUTDOWNZ, 0);
	if (!ret)
		ret = regmap_write(map, DSI2_PHY_RSTZ, 0);
	/* the clock muxes back to 26 MHz */
	if (!ret)
		ret = regmap_write(map, CK_CLK_CFG_0_CLR, 3 << 24 | 3 << 16 | 1 << 8 | 3);
	if (!ret)
		ret = regmap_write(map, CK_CLK_CFG_UPDATE, BIT(4) | BIT(3) | BIT(1) | BIT(0));
	if (ret)
		return ret;
	udelay(6);

	/* set_pll_off() */
	ret = regmap_clear_bits(map, AP_PLL_CON3, 0x1e);
	if (!ret)
		ret = regmap_set_bits(map, AP_PLL_CON5, 0x000020a0);
	if (!ret)
		ret = regmap_clear_bits(map, AP_PLL_CON5, ~0xfffcb4af);
	if (!ret)
		ret = regmap_set_bits(map, AP_PLLON_CON0, 0x07ffffff);
	if (!ret)
		ret = regmap_set_bits(map, AP_PLLON_CON1, 0x07fffe00);
	if (!ret)
		ret = regmap_clear_bits(map, AP_PLLON_CON1, 0x000001ff);
	if (!ret)
		ret = regmap_clear_bits(map, AP_MAINPLL_CON0, ~0xff7ffffe);
	if (ret)
		return ret;
	udelay(1);
	ret = regmap_set_bits(map, AP_MAINPLL_CON3, 3);
	if (ret)
		return ret;
	udelay(1);
	ret = regmap_clear_bits(map, AP_MAINPLL_CON3, 1);
	if (ret)
		return ret;

	/* set_mtcmos_off() */
	ret = regmap_read(map, SYS_PWR_CTRL, &val);
	if (!ret)
		ret = regmap_write(map, SYS_PWR_CTRL, (val & 0x00000fff) | 0x00661000);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_PWR_CTRL, ~0xffddffff);
	if (!ret)
		ret = regmap_set_bits(map, SYS_PWR_CTRL, DISP_PWR_CLK_DIS);
	if (!ret)
		ret = regmap_set_bits(map, SYS_PWR_CTRL, PISO_EN);
	if (!ret)
		ret = regmap_set_bits(map, SYS_RST_CTRL, SW_RST_EN_DISP_PWR_WRAP);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_PWR_CTRL, PWR_ON_2ND);
	if (!ret)
		ret = regmap_clear_bits(map, SYS_PWR_CTRL, PWR_ON);

	/* set_LDO_off() */
	if (!ret)
		ret = regmap_clear_bits(map, SYS_LDO_CTRL1, PHYLDO2_EN);
	if (!ret)
		ret = regmap_set_bits(map, SYS_LDO_CTRL1, PHYLDO1_LP_EN);

	return ret;
}

/* bdg_tx_start(), :1829-1841 */
static int mt6382_tx_start(struct mt6382 *ctx)
{
	int ret;

	ret = regmap_clear_bits(ctx->regmap, TX_START, TX_START_START);
	if (!ret)
		ret = regmap_set_bits(ctx->regmap, TX_START, TX_START_START);
	return ret;
}

/* --------------------------------------------------------------------------
 * Panel commands through the bridge's DSI TX
 */

/* bdg_tx_wait_for_idle(), :2007-2032 */
static int mt6382_tx_wait_idle(struct mt6382 *ctx)
{
	u32 val;

	return regmap_read_poll_timeout(ctx->regmap, TX_INTSTA, val,
					!(val & TX_INT_BUSY), 1, 5000);
}

/*
 * In command mode, through the TX's command queue as LK sends the panel's
 * init sequence (dbg_set_cmdq_V2(), :3540-3611, with force_update).
 */
static int mt6382_send_cmdq(struct mt6382 *ctx, const struct mipi_dsi_packet *pkt,
			    bool hs)
{
	u32 words[TX_CMDQ_WORDS] = { }, con;
	unsigned int i, n;
	int ret;

	if (mipi_dsi_packet_format_is_long(pkt->header[0])) {
		n = 1 + DIV_ROUND_UP(pkt->payload_length, 4);
		if (n > ARRAY_SIZE(words))
			return -EINVAL;
		words[0] = TX_CMDQ_TYPE_LONG | pkt->header[0] << 8 |
			   pkt->header[1] << 16 | pkt->header[2] << 24;
		for (i = 0; i < pkt->payload_length; i++)
			words[1 + i / 4] |= pkt->payload[i] << (8 * (i % 4));
		con = TX_CMDQ_SIZE_SEL | n;
	} else {
		n = 1;
		words[0] = pkt->header[0] << 8 | pkt->header[1] << 16 |
			   pkt->header[2] << 24;
		con = n;
	}
	if (hs)
		words[0] |= TX_CMDQ_HS;

	ret = mt6382_tx_wait_idle(ctx);
	for (i = 0; i < n && !ret; i++)
		ret = regmap_write(ctx->regmap, TX_CMDQ(i), words[i]);
	if (!ret)
		ret = regmap_write(ctx->regmap, TX_CMDQ_CON, con);
	if (!ret)
		ret = mt6382_tx_start(ctx);
	if (!ret)
		ret = mt6382_tx_wait_idle(ctx);

	return ret;
}

/* --------------------------------------------------------------------------
 * Video, through the SoC's DSI host
 */

/*
 * A bridge register write through the SoC's DSI host: a DCS long write on
 * virtual channel 1 with the address and the value, which the RX takes as a
 * write to its own registers (mipi_dsi_write_6382(), mtk_dsi.c:5170, data
 * type 0x79), in high speed (mtk_mipi_dsi_write_6382(): "AP read/write 6382
 * configs only support hs"). The host stays in command mode afterwards
 * unless @last, so that a sequence of them goes out between two frames.
 */
static int mt6382_seq_write(struct mt6382 *ctx, u32 reg, u32 val, bool last)
{
	struct mipi_dsi_device *d = ctx->cfg_dsi;
	struct mipi_dsi_msg msg = {
		.channel = d->channel,
		.type = MIPI_DSI_DCS_LONG_WRITE,
		.flags = last ? 0 : MIPI_DSI_MSG_HOLD_CMD_MODE,
		.tx_len = 8,
	};
	u8 buf[8];
	ssize_t ret;

	put_unaligned_le32(reg, buf);
	put_unaligned_le32(val, buf + 4);
	msg.tx_buf = buf;
	ret = d->host->ops->transfer(d->host, &msg);
	return ret < 0 ? ret : 0;
}

/*
 * MediaTek's sequence around a command while video runs, and to restart the
 * TX (mtk_dsi_stop_vdo_mode(), the command, mtk_dsi_start_vdo_mode(), in one
 * GCE packet; mtk_drm_setbacklight() with is_bdg_supported()). At the end
 * of the host's frame the host goes to command mode; through it the TX
 * stops, is reset and goes to command mode, and the RX to the command path;
 * the command goes out of the TX's queue; then the TX goes back to video and
 * the RX to the video path, the TX is started, and the host's video starts
 * again right after, from its interrupt handler.
 *
 * The TX's line is a little shorter than the host's (mtk_dsi.c:3434 rounds
 * the host's line up to whole words), so at the end of each frame the TX
 * waits for the host's data. Freshly reset and started a few microseconds
 * ahead of the host's frame, it waits from the first frame on; started at
 * another point it shows the wrong lines until it has caught up, or drifts
 * round the frame for seconds.
 *
 * A TX fresh from bring-up has only sent commands, and stock starts it
 * without the stop and the reset (mtk_output_dsi_enable(): bdg_tx_set_mode()
 * and bdg_tx_start(), then the host's video): reset at that point, it stays
 * busy for good and sends nothing, video or commands. So it only gets the
 * second half here. Without @pkt this only restarts the TX.
 */
static int mt6382_host_seq(struct mt6382 *ctx, const struct mipi_dsi_packet *pkt)
{
	/* a command only comes while video runs; never drop one */
	bool stop = ctx->state == MT6382_VIDEO || pkt;
	u32 com, mode, post;
	int ret, ret2;

	ret = regmap_read(ctx->regmap, TX_COM_CON, &com);
	if (!ret)
		ret = regmap_read(ctx->regmap, TX_MODE_CON, &mode);
	if (!ret)
		ret = regmap_read(ctx->regmap, MM_MIPI_RX_POST_CTRL, &post);
	if (ret)
		return ret;
	mode &= ~TX_MODE;
	if (ctx->tm.mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		mode |= TX_MODE_BURST;
	else if (ctx->tm.mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		mode |= TX_MODE_SYNC_PULSE;
	else
		mode |= TX_MODE_SYNC_EVENT;

	if (stop) {
		ret = mt6382_seq_write(ctx, TX_START, 0, false);
		udelay(25);
		if (!ret)
			ret = mt6382_seq_write(ctx, TX_COM_CON, com | TX_DSI_RESET, false);
		if (!ret)
			ret = mt6382_seq_write(ctx, TX_COM_CON, com & ~TX_DSI_RESET, false);
		if (!ret)
			ret = mt6382_seq_write(ctx, TX_MODE_CON, mode & ~TX_MODE, false);
		if (!ret)
			ret = mt6382_seq_write(ctx, MM_MIPI_RX_POST_CTRL,
					       post & ~MIPI_RX_MODE_SEL, false);
		if (!ret && pkt)
			ret = mt6382_send_cmdq(ctx, pkt, true);
		udelay(25);
	}

	/* the restart, even after a failure, or the host stays in command mode */
	ret2 = mt6382_seq_write(ctx, TX_MODE_CON, mode, false);
	if (!ret2)
		ret2 = mt6382_seq_write(ctx, MM_MIPI_RX_POST_CTRL,
					post | MIPI_RX_MODE_SEL, false);
	if (!ret2)
		ret2 = mt6382_seq_write(ctx, TX_START, 0, false);
	if (!ret2)
		ret2 = mt6382_seq_write(ctx, TX_START, TX_START_START, true);
	if (!ret2)
		ctx->state = MT6382_VIDEO;

	return ret ?: ret2;
}

/*
 * Start the TX with the host's video. A TX fresh from bring-up only gets
 * the start (mt6382_host_seq()) and then runs on its own: ahead of the
 * host's data, it drifts round the frame, about a millisecond a second,
 * instead of waiting for the data at the end of each frame. Stock never
 * leaves it so: right after the screen comes on it sets the backlight, and
 * that command goes through the whole sequence, TX reset included, a frame
 * after bdg_tx_start(). Do the same once the TX has sent its first frame;
 * it then stays with the host.
 */
static int mt6382_start_video(struct mt6382 *ctx)
{
	bool fresh = ctx->state != MT6382_VIDEO;
	u32 sta;
	int ret;

	ret = mt6382_host_seq(ctx, NULL);
	if (ret || !fresh)
		return ret;

	/* the status bit only latches while the interrupt is enabled */
	ret = regmap_set_bits(ctx->regmap, TX_INTEN, TX_INT_VM_VFP_STR);
	if (!ret)
		ret = regmap_clear_bits(ctx->regmap, TX_INTSTA, TX_INT_VM_VFP_STR);
	if (!ret)
		ret = regmap_read_poll_timeout(ctx->regmap, TX_INTSTA, sta,
					       sta & TX_INT_VM_VFP_STR, 500, 50000);
	if (ret) {
		dev_warn(ctx->dev, "no frame from the TX after its start: %d\n", ret);
		return ret;
	}

	return mt6382_host_seq(ctx, NULL);
}

/* --------------------------------------------------------------------------
 * DSI host for the panel
 */

static int mt6382_attach_ap_dsi(struct mt6382 *ctx)
{
	const struct mipi_dsi_device_info info = {
		.type = "mt6382",
		.channel = 0,
		.node = ctx->dev->of_node,
	};
	const struct mipi_dsi_device_info cfg_info = {
		.type = "mt6382-cfg",
		.channel = 1,
	};
	struct device_node *host_node;
	struct mipi_dsi_device *dsi, *cfg;
	struct mipi_dsi_host *host;
	int ret;

	host_node = of_graph_get_remote_node(ctx->dev->of_node, 0, -1);
	if (!host_node)
		return -ENODEV;

	host = of_find_mipi_dsi_host_by_node(host_node);
	of_node_put(host_node);
	if (!host)
		return -EPROBE_DEFER;

	/* not attached: only for transfers, in high speed (no MODE_LPM) */
	cfg = mipi_dsi_device_register_full(host, &cfg_info);
	if (IS_ERR(cfg))
		return PTR_ERR(cfg);

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		mipi_dsi_device_unregister(cfg);
		return PTR_ERR(dsi);
	}

	dsi->lanes = ctx->tm.lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	dsi->hs_rate = (unsigned long)ctx->tm.rx_rate * 1000000;

	/*
	 * The host removes the devices on it when it goes away
	 * (mipi_dsi_host_unregister()), possibly before us: keep them
	 * until we are done with them.
	 *
	 * Known before the attach: attaching can complete the display
	 * pipeline, whose driver then binds and enables it before
	 * mipi_dsi_attach() returns (mediatek-drm with its fbdev client), and
	 * our enable needs the configuration device by then.
	 */
	get_device(&dsi->dev);
	get_device(&cfg->dev);
	mutex_lock(&ctx->lock);
	ctx->ap_dsi = dsi;
	ctx->cfg_dsi = cfg;
	mutex_unlock(&ctx->lock);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		mutex_lock(&ctx->lock);
		ctx->ap_dsi = NULL;
		ctx->cfg_dsi = NULL;
		mutex_unlock(&ctx->lock);
		put_device(&dsi->dev);
		put_device(&cfg->dev);
		mipi_dsi_device_unregister(dsi);
		mipi_dsi_device_unregister(cfg);
		return ret;
	}

	return 0;
}

static void mt6382_detach_ap_dsi(struct mt6382 *ctx)
{
	struct mipi_dsi_device *dsi, *cfg;

	mutex_lock(&ctx->lock);
	dsi = ctx->ap_dsi;
	cfg = ctx->cfg_dsi;
	ctx->ap_dsi = NULL;
	ctx->cfg_dsi = NULL;
	mutex_unlock(&ctx->lock);
	if (!dsi)
		return;

	if (device_is_registered(&dsi->dev)) {
		mipi_dsi_detach(dsi);
		mipi_dsi_device_unregister(dsi);
	}
	if (device_is_registered(&cfg->dev))
		mipi_dsi_device_unregister(cfg);
	put_device(&dsi->dev);
	put_device(&cfg->dev);
}

static int mt6382_host_attach(struct mipi_dsi_host *host, struct mipi_dsi_device *dev)
{
	struct mt6382 *ctx = host_to_mt6382(host);
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int ret;

	if (ctx->panel_dsi)
		return -EBUSY;
	if (dev->lanes < 1 || dev->lanes > 4 || dev->format != MIPI_DSI_FMT_RGB888 ||
	    !(dev->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		dev_err(ctx->dev, "only RGB888 video mode panels are supported\n");
		return -EINVAL;
	}
	if (!dev->dsc || dev->dsc->slice_count < 1 || dev->dsc->slice_count > 2 ||
	    dev->dsc->bits_per_component != 8) {
		dev_err(ctx->dev, "only 8 bpc DSC panels with 1 or 2 slices are supported\n");
		return -EINVAL;
	}

	ret = drm_of_find_panel_or_bridge(ctx->dev->of_node, 1, 0, &panel, &bridge);
	if (ret)
		return ret;
	if (panel) {
		bridge = drm_panel_bridge_add_typed(panel, DRM_MODE_CONNECTOR_DSI);
		drm_panel_put(panel);
		if (IS_ERR(bridge))
			return PTR_ERR(bridge);
	}

	mutex_lock(&ctx->lock);
	ctx->panel_dsi = dev;
	ctx->next_bridge = bridge;
	ctx->next_is_panel = panel;
	ctx->dsc = *dev->dsc;
	ctx->tm.lanes = dev->lanes;
	ctx->tm.mode_flags = dev->mode_flags;
	ctx->tm.tx_rate = dev->hs_rate ? dev->hs_rate / 1000000 :
			  MT6382_DEFAULT_TX_RATE_MBPS;
	ctx->tm.rx_rate = ctx->tm.tx_rate * MT6382_RXTX_RATIO / 100;
	mutex_unlock(&ctx->lock);

	drm_bridge_add(&ctx->bridge);

	ret = mt6382_attach_ap_dsi(ctx);
	if (ret) {
		drm_bridge_remove(&ctx->bridge);
		if (panel)
			drm_panel_bridge_remove(bridge);
		ctx->panel_dsi = NULL;
		ctx->next_bridge = NULL;
		return dev_err_probe(ctx->dev, ret, "cannot attach to the DSI host\n");
	}

	return 0;
}

static int mt6382_host_detach(struct mipi_dsi_host *host, struct mipi_dsi_device *dev)
{
	struct mt6382 *ctx = host_to_mt6382(host);

	if (ctx->panel_dsi != dev)
		return -EINVAL;

	mt6382_detach_ap_dsi(ctx);
	drm_bridge_remove(&ctx->bridge);
	if (ctx->next_is_panel)
		drm_panel_bridge_remove(ctx->next_bridge);

	mutex_lock(&ctx->lock);
	ctx->panel_dsi = NULL;
	ctx->next_bridge = NULL;
	mutex_unlock(&ctx->lock);

	return 0;
}

static ssize_t mt6382_host_transfer(struct mipi_dsi_host *host,
				    const struct mipi_dsi_msg *msg)
{
	struct mt6382 *ctx = host_to_mt6382(host);
	struct mipi_dsi_packet pkt;
	u32 mode;
	int ret;

	/* reads would need the TX's BTA and RX bypass (bdg_set_dcs_read_cmd()) */
	if (msg->rx_len)
		return -EOPNOTSUPP;

	ret = mipi_dsi_create_packet(&pkt, msg);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	if (ctx->state == MT6382_OFF) {
		ret = -ENODEV;
		goto out;
	}

	ret = regmap_read(ctx->regmap, TX_MODE_CON, &mode);
	if (ret)
		goto out;

	/*
	 * In command mode, before the video starts, the panel's init
	 * sequence goes out of the TX's queue as LK sends it. While video
	 * runs, a command goes in MediaTek's sequence through the host; the
	 * TX's VM_CMD never completes, and MediaTek's kernel never uses it.
	 */
	if (!(mode & TX_MODE))
		ret = mt6382_send_cmdq(ctx, &pkt, !(msg->flags & MIPI_DSI_MSG_USE_LPM));
	else if (ctx->host_streaming)
		ret = mt6382_host_seq(ctx, &pkt);
	else
		ret = -EBUSY;
out:
	mutex_unlock(&ctx->lock);

	return ret ? ret : msg->tx_len;
}

static const struct mipi_dsi_host_ops mt6382_host_ops = {
	.attach = mt6382_host_attach,
	.detach = mt6382_host_detach,
	.transfer = mt6382_host_transfer,
};

/* --------------------------------------------------------------------------
 * DRM bridge
 */

static int mt6382_bridge_attach(struct drm_bridge *bridge, struct drm_encoder *encoder,
				enum drm_bridge_attach_flags flags)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);

	return drm_bridge_attach(encoder, ctx->next_bridge, bridge, flags);
}

static void mt6382_timing_from_mode(struct mt6382 *ctx, struct mt6382_timing *t,
				    const struct drm_display_mode *mode)
{
	*t = ctx->tm;
	t->hactive = mode->hdisplay;
	t->hfp = mode->hsync_start - mode->hdisplay;
	t->hsa = mode->hsync_end - mode->hsync_start;
	t->hbp = mode->htotal - mode->hsync_end;
	t->vactive = mode->vdisplay;
	t->vfp = mode->vsync_start - mode->vdisplay;
	t->vsa = mode->vsync_end - mode->vsync_start;
	t->vbp = mode->vtotal - mode->vsync_end;
}

static enum drm_mode_status mt6382_bridge_mode_valid(struct drm_bridge *bridge,
						     const struct drm_display_info *info,
						     const struct drm_display_mode *mode)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);
	struct mt6382_timing t;

	if (mode->hdisplay != ctx->dsc.slice_width * ctx->dsc.slice_count)
		return MODE_H_ILLEGAL;
	if (mode->vdisplay % ctx->dsc.slice_height)
		return MODE_V_ILLEGAL;

	mt6382_timing_from_mode(ctx, &t, mode);
	if (mt6382_compute_timing(&t))
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

/*
 * The SoC's DSI host gets a mode with the panel's vertical timing and a line
 * as long, at the bridge's RX rate, as the bridge's TX line: stock sends
 * HSA 4 and HBP 4 bytes and makes up the rest with HFP (mtk_dsi.c:3427-3460).
 */
static bool mt6382_bridge_mode_fixup(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);
	/* 4 * 3 - 10 = 2 bytes each, which a host sending whole words makes 4 */
	const u32 hsa = 4, hbp = 4;
	struct mt6382_timing t;

	mt6382_timing_from_mode(ctx, &t, mode);
	if (mt6382_compute_timing(&t))
		return false;

	adjusted->hsync_start = t.ap_htotal - hsa - hbp;
	adjusted->hsync_end = adjusted->hsync_start + hsa;
	adjusted->htotal = adjusted->hsync_end + hbp;
	/* the clock that makes the host send RGB888 at rx_rate per lane */
	adjusted->clock = DIV_ROUND_UP(t.rx_rate * 1000 * t.lanes, 24);

	return true;
}

static void mt6382_bridge_atomic_pre_enable(struct drm_bridge *bridge,
					    struct drm_atomic_commit *state)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);
	const struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
	int ret;

	crtc = drm_atomic_get_new_crtc_for_encoder(state, bridge->encoder);
	if (!crtc)
		return;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	mutex_lock(&ctx->lock);
	mt6382_timing_from_mode(ctx, &ctx->tm, &crtc_state->mode);
	ret = mt6382_compute_timing(&ctx->tm);
	if (!ret) {
		/* the panel's DSC config, completed as the encoder needs it */
		ctx->dsc.pic_width = ctx->tm.hactive;
		ctx->dsc.pic_height = ctx->tm.vactive;
		/* RGB in, with the colour transform (rct_on in stock) */
		ctx->dsc.convert_rgb = true;
		drm_dsc_set_const_params(&ctx->dsc);
		drm_dsc_set_rc_buf_thresh(&ctx->dsc);
		ret = drm_dsc_setup_rc_params(&ctx->dsc, DRM_DSC_1_1_PRE_SCR);
	}
	if (!ret) {
		ctx->dsc.initial_scale_value = drm_dsc_initial_scale_value(&ctx->dsc);
		ctx->dsc.line_buf_depth = ctx->dsc.bits_per_component + 1;
		ret = drm_dsc_compute_rc_parameters(&ctx->dsc);
	}
	if (!ret)
		ret = mt6382_bring_up(ctx);
	if (ret) {
		dev_err(ctx->dev, "bring-up failed: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		ctx->state = MT6382_OFF;
	}
	mutex_unlock(&ctx->lock);
}

static void mt6382_bridge_atomic_enable(struct drm_bridge *bridge,
					struct drm_atomic_commit *state)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);
	u32 clk, data, rdma, tx;
	int ret;

	mutex_lock(&ctx->lock);
	ctx->host_streaming = true;
	/*
	 * Start the TX with the host's next frame: MediaTek's kernel starts it
	 * each time the host starts streaming, and a TX that does not start
	 * with the host's frame runs out of data towards the bottom of it.
	 */
	if (ctx->state != MT6382_OFF) {
		ret = mt6382_start_video(ctx);
		if (ret)
			dev_err(ctx->dev, "cannot start video: %d\n", ret);
	}

	/*
	 * The DSI host enabled before us and streams by now. Without its HS
	 * clock the RX passes nothing on, the RDMA and DSC stay idle and the TX
	 * never reaches a VFP, so the panel stays black and commands sent
	 * while video runs time out.
	 */
	if (ctx->state == MT6382_VIDEO) {
		msleep(40);	/* two frames at 60 Hz */
		if (!regmap_read(ctx->regmap, DSI2_PHY_CLK_STATUS, &clk) &&
		    !regmap_read(ctx->regmap, DSI2_PHY_DATA_STATUS, &data) &&
		    !regmap_read(ctx->regmap, RDMA_INTSTA, &rdma) &&
		    !regmap_read(ctx->regmap, TX_INTSTA, &tx) &&
		    !(clk & DSI2_PHY_CLK_ACTIVE_HS))
			dev_warn(ctx->dev,
				 "no HS clock from the DSI host: PHY clk 0x%x data 0x%x, RDMA 0x%x, TX 0x%x\n",
				 clk, data, rdma, tx);
	}
	mutex_unlock(&ctx->lock);
}

/*
 * The host powers off before our post_disable (pre_enable_prev_first), so
 * no restart through it may be pending by then.
 */
static void mt6382_bridge_atomic_disable(struct drm_bridge *bridge,
					 struct drm_atomic_commit *state)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);

	mutex_lock(&ctx->lock);
	ctx->host_streaming = false;
	mutex_unlock(&ctx->lock);
}

static void mt6382_bridge_atomic_post_disable(struct drm_bridge *bridge,
					      struct drm_atomic_commit *state)
{
	struct mt6382 *ctx = bridge_to_mt6382(bridge);

	int ret = 0;

	/*
	 * bdg_common_deinit(), :5569-5597: block by block, out of reset, so
	 * that the clock buffer keeps passing the 26 MHz on to the NFC
	 * controller. The next enable starts from a reset pulse anyway.
	 */
	mutex_lock(&ctx->lock);
	if (ctx->state != MT6382_OFF) {
		ret = regmap_clear_bits(ctx->regmap, TX_START, TX_START_START);
		ctx->spi_fast = false;	/* spislv_switch_speed_hz(), :5581 */
		if (!ret)
			ret = mt6382_tx_off(ctx);
		if (!ret)
			ret = mt6382_power_off(ctx);
		if (ret)
			dev_err(ctx->dev, "power-down failed: %d\n", ret);
	}
	ctx->state = MT6382_OFF;
	mutex_unlock(&ctx->lock);
}

static const struct drm_bridge_funcs mt6382_bridge_funcs = {
	.attach = mt6382_bridge_attach,
	.mode_valid = mt6382_bridge_mode_valid,
	.mode_fixup = mt6382_bridge_mode_fixup,
	.atomic_pre_enable = mt6382_bridge_atomic_pre_enable,
	.atomic_enable = mt6382_bridge_atomic_enable,
	.atomic_disable = mt6382_bridge_atomic_disable,
	.atomic_post_disable = mt6382_bridge_atomic_post_disable,
	.atomic_create_state = drm_atomic_helper_bridge_create_state,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
};

/* --------------------------------------------------------------------------
 * Probe
 */

/* TEST: does the TX go through frames (VFP start) and does it underrun? */
static int mt6382_frames_show(struct seq_file *m, void *unused)
{
	struct mt6382 *ctx = m->private;
	u32 a = 0, b = 0;

	mutex_lock(&ctx->lock);
	if (ctx->state != MT6382_OFF) {
		regmap_set_bits(ctx->regmap, TX_INTEN, BIT(10));
		regmap_write(ctx->regmap, TX_INTSTA, TX_INT_BUSY);
		msleep(60);
		regmap_read(ctx->regmap, TX_INTSTA, &a);
		regmap_write(ctx->regmap, TX_INTSTA, TX_INT_BUSY);
		msleep(60);
		regmap_read(ctx->regmap, TX_INTSTA, &b);
	}
	mutex_unlock(&ctx->lock);
	seq_printf(m, "state %d vfp %d/%d underrun %d/%d intsta %08x %08x\n", ctx->state,
		   !!(a & BIT(10)), !!(b & BIT(10)), !!(a & BIT(12)), !!(b & BIT(12)), a, b);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_frames);

/*
 * TEST: phase between the SoC's DSI host and the TX. The host's DSI_STATE_DBG7
 * (0x14013164) is read directly; its run-length encoding over 40 ms goes out
 * first. Then, 30 times 100 ms apart: the start of the host's shortest
 * vertical state (taken as VSA) and, over SPI, the TX's next VFP start
 * (INTSTA bit 10). The difference drifts if the two lines differ in length.
 */
static u64 mt6382_ap_vsa(void __iomem *ap, u64 limit_ns)
{
	u64 t0 = ktime_get_ns(), start = t0, now;
	u32 prev = readl(ap + AP_DBG7) >> 8, v;

	for (;;) {
		now = ktime_get_ns();
		if (now - t0 > limit_ns)
			return 0;
		v = readl(ap + AP_DBG7) >> 8;
		if (v == prev)
			continue;
		/* a run shorter than 12 us that is not the first one seen */
		if (start != t0 && now - start < 12000)
			return start;
		prev = v;
		start = now;
	}
}

static int mt6382_phase_show(struct seq_file *m, void *unused)
{
	struct mt6382 *ctx = m->private;
	void __iomem *ap = ioremap(AP_DSI_BASE, 0x1000);
	u32 runs_v[64], runs_us[64], v, prev, sta;
	u64 t0, start, now, vsa, tx;
	unsigned int n = 0, i;

	if (!ap)
		return -ENOMEM;

	t0 = start = ktime_get_ns();
	prev = readl(ap + AP_DBG7);
	while (n < ARRAY_SIZE(runs_v) && (now = ktime_get_ns()) - t0 < 40000000) {
		v = readl(ap + AP_DBG7);
		if (v == prev)
			continue;
		runs_v[n] = prev;
		runs_us[n++] = (now - start) / 1000;
		prev = v;
		start = now;
	}
	seq_puts(m, "ap dbg7 runs (value:us):");
	for (i = 0; i < n; i++)
		seq_printf(m, " %x:%u", runs_v[i], runs_us[i]);
	seq_putc(m, '\n');

	mutex_lock(&ctx->lock);
	if (ctx->state == MT6382_OFF)
		goto out;
	regmap_set_bits(ctx->regmap, TX_INTEN, BIT(10));
	for (i = 0; i < 30; i++) {
		vsa = mt6382_ap_vsa(ap, 40000000);
		regmap_write(ctx->regmap, TX_INTSTA, TX_INT_BUSY);
		tx = 0;
		while (ktime_get_ns() - vsa < 40000000) {
			if (regmap_read(ctx->regmap, TX_INTSTA, &sta))
				break;
			if (sta & BIT(10)) {
				tx = ktime_get_ns();
				break;
			}
		}
		seq_printf(m, "%s%lld", i ? " " : "phase us:",
			   vsa && tx ? (s64)(tx - vsa) / 1000 : -1LL);
		msleep(100);
	}
	seq_putc(m, '\n');
out:
	mutex_unlock(&ctx->lock);
	iounmap(ap);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_phase);

/* TEST: "addr" reads, "addr val" writes one bridge register; read shows the last */
static u32 mt6382_dbg_addr, mt6382_dbg_val;
static ssize_t mt6382_reg_dbg_write(struct file *f, const char __user *ubuf, size_t len,
				    loff_t *ppos)
{
	struct mt6382 *ctx = file_inode(f)->i_private;
	char buf[40] = { };
	int n, ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	n = sscanf(buf, "%x %x", &mt6382_dbg_addr, &mt6382_dbg_val);
	if (n == 2)
		ret = regmap_write(ctx->regmap, mt6382_dbg_addr, mt6382_dbg_val);
	else if (n == 1)
		ret = regmap_read(ctx->regmap, mt6382_dbg_addr, &mt6382_dbg_val);
	else
		return -EINVAL;
	return ret ? ret : len;
}

static ssize_t mt6382_reg_dbg_read(struct file *f, char __user *ubuf, size_t len, loff_t *ppos)
{
	char buf[40];
	int n = scnprintf(buf, sizeof(buf), "%05x %08x\n", mt6382_dbg_addr, mt6382_dbg_val);

	return simple_read_from_buffer(ubuf, len, ppos, buf, n);
}

static const struct file_operations mt6382_reg_dbg_fops = {
	.write = mt6382_reg_dbg_write,
	.read = mt6382_reg_dbg_read,
};

static void mt6382_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}



static int mt6382_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct clk_hw *ckbuf;
	struct mt6382 *ctx;
	const char *name;
	int ret;

	ctx = devm_drm_bridge_alloc(dev, struct mt6382, bridge, &mt6382_bridge_funcs);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dev = dev;
	ctx->spi = spi;
	spi_set_drvdata(spi, ctx);
	mutex_init(&ctx->lock);
	mutex_init(&ctx->spi_lock);

	ctx->txbuf = devm_kzalloc(dev, 16, GFP_KERNEL);
	ctx->rxbuf = devm_kzalloc(dev, 16, GFP_KERNEL);
	ctx->rs_tx = devm_kzalloc(dev, 2, GFP_KERNEL);
	ctx->rs_rx = devm_kzalloc(dev, 2, GFP_KERNEL);
	if (!ctx->txbuf || !ctx->rxbuf || !ctx->rs_tx || !ctx->rs_rx)
		return -ENOMEM;

	/* spi_slave.c:430: mode 0, LSB first */
	spi->mode = (spi->mode & ~SPI_MODE_X_MASK) | SPI_MODE_0 | SPI_LSB_FIRST;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up SPI\n");

	ctx->refclk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(ctx->refclk))
		return dev_err_probe(dev, PTR_ERR(ctx->refclk), "no reference clock\n");

	/* asserting it would stop a bridge the bootloader left running */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "no reset GPIO\n");

	ctx->regmap = devm_regmap_init(dev, NULL, ctx, &mt6382_regmap_config);
	if (IS_ERR(ctx->regmap))
		return dev_err_probe(dev, PTR_ERR(ctx->regmap), "cannot init regmap\n");

	ctx->state = MT6382_OFF;

	/*
	 * The 26 MHz passed on to an NFC controller: the clock buffer runs at
	 * the controller's request, from bring-up on and while powered down
	 * after the panel is off.
	 */
	name = devm_kasprintf(dev, GFP_KERNEL, "%s-ckbuf", dev_name(dev));
	if (!name)
		return -ENOMEM;
	ckbuf = devm_clk_hw_register_fixed_factor_index(dev, name, 0, 0, 1, 1);
	if (IS_ERR(ckbuf))
		return dev_err_probe(dev, PTR_ERR(ckbuf), "cannot register the NFC clock\n");
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, ckbuf);
	if (ret)
		return dev_err_probe(dev, ret, "cannot provide the NFC clock\n");

	ctx->bridge.of_node = dev->of_node;
	ctx->bridge.pre_enable_prev_first = true;

	ctx->host.dev = dev;
	ctx->host.ops = &mt6382_host_ops;

	{
		struct dentry *d = debugfs_create_dir("mt6382", NULL);

		debugfs_create_file("frames", 0400, d, ctx, &mt6382_frames_fops);
		debugfs_create_file("phase", 0400, d, ctx, &mt6382_phase_fops);
		debugfs_create_file("reg", 0600, d, ctx, &mt6382_reg_dbg_fops);
		ret = devm_add_action_or_reset(dev, mt6382_debugfs_remove, d);
		if (ret)
			return ret;
	}

	return mipi_dsi_host_register(&ctx->host);
}

static void mt6382_remove(struct spi_device *spi)
{
	struct mt6382 *ctx = spi_get_drvdata(spi);

	mipi_dsi_host_unregister(&ctx->host);
}

static const struct of_device_id mt6382_of_match[] = {
	{ .compatible = "mediatek,mt6382" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6382_of_match);

static const struct spi_device_id mt6382_spi_ids[] = {
	{ "mt6382" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, mt6382_spi_ids);

static struct spi_driver mt6382_driver = {
	.driver = {
		.name = "mediatek-mt6382",
		.of_match_table = mt6382_of_match,
	},
	.id_table = mt6382_spi_ids,
	.probe = mt6382_probe,
	.remove = mt6382_remove,
};
module_spi_driver(mt6382_driver);

MODULE_DESCRIPTION("MediaTek MT6382 DSI to DSI bridge with DSC");
MODULE_LICENSE("GPL");
