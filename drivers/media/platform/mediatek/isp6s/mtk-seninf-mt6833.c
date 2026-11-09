// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6833 sensor interface (SENINF)
 *
 * Six CSI-2 receivers (seninf1-6), each with a test pattern generator,
 * feed six muxes (FIFOs), whose output any of 13 cam_mux outputs can take
 * towards the camera engines: cam_mux 0-2 the raw ISPs, 3 and up the
 * camera DMA engines (camsv).
 *
 * The media graph follows MediaTek's MT8365 ISP 3.0 SENINF series (v5):
 * this driver owns the media device, with a sink pad per receiver and a
 * source pad per cam_mux, binds the sensors on its sink ports and the
 * camsv instances on its source ports. Without the streams API, the
 * receiver feeding the outputs is the one whose link from its sensor is
 * enabled. Registers and their order are those of MediaTek's camera HAL
 * for this SoC (libcam.halsensor.so, SeninfDrvImp), which match the
 * SENINF code in MediaTek's MT8188 camera series, except that cam_mux
 * sources are selected in CAM_MUX_CTRL (0x400), not CHK_CTL_0.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#define SENINF_NUM_INPUTS		6
#define SENINF_NUM_OUTPUTS		13
#define SENINF_NUM_PADS			(SENINF_NUM_INPUTS + SENINF_NUM_OUTPUTS)
#define SENINF_NUM_CSI			3
#define SENINF_MAX_LANES		4

/* top, in seninf1's page */
#define SENINF_TOP_MUX_CTRL(m)		(0x010 + 4 * ((m) / 4))
#define SENINF_TOP_MUX_SHIFT(m)		(8 * ((m) % 4))
#define SENINF_TOP_PHY_CTRL_CSI(n)	(0x060 + 4 * (n))
#define  TOP_PHY_CTRL_DPHY_EN		BIT(0)
#define  TOP_PHY_CTRL_CPHY_EN		BIT(1)

/* cam_mux */
#define SENINF_CAM_MUX_CTRL(c)		(0x400 + 4 * ((c) / 4))
#define SENINF_CAM_MUX_SHIFT(c)		(8 * ((c) % 4))
#define SENINF_CAM_MUX_EN		0x410
#define SENINF_CAM_MUX_OPT(c)		(0x420 + 4 * (c))
#define  CAM_MUX_OPT_VC_EN		BIT(0)
#define  CAM_MUX_OPT_DT_EN		BIT(1)
#define  CAM_MUX_OPT_VC_SEL		GENMASK(12, 8)
#define  CAM_MUX_OPT_DT_SEL		GENMASK(21, 16)
#define SENINF_CAM_MUX_IRQ_STATUS	0x4a8
#define SENINF_CAM_MUX_CHK_CTL_0(c)	(0x500 + 0x10 * (c))
#define  CAM_MUX_CHK_PIX_MODE		GENMASK(1, 0)
#define SENINF_CAM_MUX_CHK_CTL_1(c)	(0x504 + 0x10 * (c))
#define SENINF_CAM_MUX_CHK_RES(c)	(0x508 + 0x10 * (c))

/* per receiver i, at 0x1000 * i */
#define SENINF_CTRL			0x200
#define  SENINF_CTRL_EN			BIT(0)
#define SENINF_CSI2_CTRL		0x210
#define  SENINF_CSI2_CTRL_EN		BIT(0)
#define SENINF_TESTMDL_CTRL		0x220
#define  SENINF_TESTMDL_EN		BIT(0)
#define SENINF_TM_CTL			0x608
#define  TM_CTL_EN			BIT(0)
#define  TM_CTL_PAT			GENMASK(8, 4)
#define SENINF_TM_SIZE			0x60c
#define SENINF_TM_CLK			0x610
#define SENINF_TM_DUM			0x618
#define SENINF_CSI2_EN			0xa00
#define SENINF_CSI2_OPT			0xa04
#define  CSI2_OPT_CPHY_SEL		BIT(0)
#define  CSI2_OPT_GENERIC_LONG_PKT	BIT(3)
#define SENINF_CSI2_HDR_MODE_0		0xa08
#define SENINF_CSI2_RESYNC_MERGE	0xa10
#define  CSI2_RESYNC_MERGE_EN		BIT(8)
#define SENINF_CSI2_DI_CTRL(s)		(0xa20 + 4 * (s))
#define  CSI2_DI_CTRL_EN		0x11
#define  CSI2_DI_CTRL_VC		GENMASK(12, 8)
#define  CSI2_DI_CTRL_DT		GENMASK(21, 16)
#define SENINF_CSI2_CH_CTRL(c)		(0xa60 + 4 * (c))
#define  CSI2_CH_CTRL_S0		BIT(8)
#define SENINF_CSI2_DBG_CTRL		0xae0
#define  CSI2_DBG_CTRL_BIT17		BIT(17)

/* per mux m, at 0x1000 * m */
#define SENINF_MUX_CTRL_0		0xd00
#define  MUX_CTRL_0_EN			BIT(0)
#define  MUX_CTRL_0_IRQ_SW_RST		BIT(1)
#define  MUX_CTRL_0_SW_RST		BIT(2)
#define SENINF_MUX_CTRL_1		0xd04
#define  MUX_CTRL_1_SRC			GENMASK(3, 0)
#define  MUX_SRC_TEST_MODEL		1
#define  MUX_SRC_MIPI			8
#define  MUX_CTRL_1_PIX_MODE		GENMASK(9, 8)
#define SENINF_MUX_OPT			0xd08
#define  MUX_OPT_HSYNC_POL		BIT(16)
#define  MUX_OPT_VSYNC_POL		BIT(17)
#define SENINF_MUX_IRQ_STATUS		0xd18

/* the HAL's test pattern: colour bars, clock / 8, 100 dummy pixels and lines */
#define SENINF_TM_PATTERN		0xc
#define SENINF_TM_CLK_DIV		7
#define SENINF_TM_DUMMY			((100 << 16) | 100)

#define SENINF_DEFAULT_WIDTH		1600
#define SENINF_DEFAULT_HEIGHT		1200
#define SENINF_DEFAULT_CODE		MEDIA_BUS_FMT_Y10_1X10

static unsigned int cam_mux_extra;
module_param(cam_mux_extra, uint, 0644);
MODULE_PARM_DESC(cam_mux_extra,
		 "Bit mask of cam_mux outputs to drive along with the streaming one (bring-up)");

static unsigned int tm_clk = SENINF_TM_CLK_DIV;
module_param(tm_clk, uint, 0644);
MODULE_PARM_DESC(tm_clk, "Test pattern clock divider (bring-up)");

static unsigned int tm_dummy = SENINF_TM_DUMMY;
module_param(tm_dummy, uint, 0644);
MODULE_PARM_DESC(tm_dummy, "Test pattern TM_DUM: dummy lines << 16 | dummy pixels (bring-up)");

static bool cam_mux_filter = true;
module_param(cam_mux_filter, bool, 0644);
MODULE_PARM_DESC(cam_mux_filter, "Let cam_mux pass only the stream's VC and DT (bring-up)");

struct mtk_seninf_format {
	u32 code;
	u8 dt;
};

static const struct mtk_seninf_format mtk_seninf_formats[] = {
	{ MEDIA_BUS_FMT_Y10_1X10, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_Y8_1X8, MIPI_CSI2_DT_RAW8 },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, MIPI_CSI2_DT_RAW8 },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, MIPI_CSI2_DT_RAW8 },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, MIPI_CSI2_DT_RAW8 },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, MIPI_CSI2_DT_RAW8 },
};

struct mtk_seninf_input {
	unsigned int id;		/* receiver: seninf<id + 1>, also its pad */
	struct phy *phy;
	unsigned int csi;		/* CSI port pair */
	unsigned int lanes;
	struct v4l2_subdev *sensor;
	struct media_pad *sensor_pad;
};

struct mtk_seninf {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev sd;
	struct media_pad pads[SENINF_NUM_PADS];
	struct v4l2_async_notifier notifier;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *test_pattern;

	struct mtk_seninf_input inputs[SENINF_NUM_INPUTS];
	struct mtk_seninf_input *streaming;	/* receiver in use */
	unsigned int streaming_outputs;		/* cam_mux mask */
	bool streaming_tpg;
};

struct mtk_seninf_asc {
	struct v4l2_async_connection asc;
	unsigned int port;
};

static inline struct mtk_seninf *to_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_seninf, sd);
}

static inline bool mtk_seninf_is_sink(unsigned int pad)
{
	return pad < SENINF_NUM_INPUTS;
}

static const struct mtk_seninf_format *mtk_seninf_find_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_seninf_formats); i++)
		if (mtk_seninf_formats[i].code == code)
			return &mtk_seninf_formats[i];

	return NULL;
}

static void mtk_seninf_update(struct mtk_seninf *priv, u32 reg, u32 mask, u32 val)
{
	u32 tmp = readl(priv->base + reg);

	writel((tmp & ~mask) | (val & mask), priv->base + reg);
}

static void mtk_seninf_dump(struct mtk_seninf *priv, const char *what,
			    unsigned int start, unsigned int end)
{
	unsigned int off;

	for (off = start; off < end; off += 0x20)
		dev_info(priv->dev, "%s %04x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 what, off,
			 readl(priv->base + off), readl(priv->base + off + 0x4),
			 readl(priv->base + off + 0x8), readl(priv->base + off + 0xc),
			 readl(priv->base + off + 0x10), readl(priv->base + off + 0x14),
			 readl(priv->base + off + 0x18), readl(priv->base + off + 0x1c));
}

static void mtk_seninf_dump_state(struct mtk_seninf *priv, unsigned int id,
				  unsigned int outputs)
{
	unsigned int r = 0x1000 * id, c;

	for (c = 0; c < SENINF_NUM_OUTPUTS; c++)
		if (outputs & BIT(c))
			dev_info(priv->dev, "cam_mux %u: ctrl %08x opt %08x chk %08x %08x res %08x\n",
				 c, readl(priv->base + SENINF_CAM_MUX_CTRL(c)),
				 readl(priv->base + SENINF_CAM_MUX_OPT(c)),
				 readl(priv->base + SENINF_CAM_MUX_CHK_CTL_0(c)),
				 readl(priv->base + SENINF_CAM_MUX_CHK_CTL_1(c)),
				 readl(priv->base + SENINF_CAM_MUX_CHK_RES(c)));
	dev_info(priv->dev, "cam_mux en %08x irq %08x; mux%u irq %08x\n",
		 readl(priv->base + SENINF_CAM_MUX_EN),
		 readl(priv->base + SENINF_CAM_MUX_IRQ_STATUS), id + 1,
		 readl(priv->base + r + SENINF_MUX_IRQ_STATUS));
	mtk_seninf_dump(priv, "top", 0x000, 0x080);
	mtk_seninf_dump(priv, "cam_mux", 0x400, 0x440);
	mtk_seninf_dump(priv, "rx", r + 0x200, r + 0x240);
	mtk_seninf_dump(priv, "tm", r + 0x600, r + 0x640);
	mtk_seninf_dump(priv, "csi2", r + 0xa00, r + 0xb00);
	mtk_seninf_dump(priv, "mux", r + 0xd00, r + 0xd40);
}

/* -----------------------------------------------------------------------------
 * Hardware
 */

static void mtk_seninf_csi2_setup(struct mtk_seninf *priv,
				  struct mtk_seninf_input *input, u8 dt)
{
	unsigned int r = 0x1000 * input->id;
	unsigned int i;

	mtk_seninf_update(priv, r + SENINF_CSI2_DBG_CTRL, CSI2_DBG_CTRL_BIT17,
			  CSI2_DBG_CTRL_BIT17);
	mtk_seninf_update(priv, r + SENINF_CSI2_RESYNC_MERGE, CSI2_RESYNC_MERGE_EN,
			  CSI2_RESYNC_MERGE_EN);
	mtk_seninf_update(priv, r + SENINF_CSI2_OPT,
			  CSI2_OPT_CPHY_SEL | CSI2_OPT_GENERIC_LONG_PKT,
			  dt >= 0x10 && dt <= 0x17 ? CSI2_OPT_GENERIC_LONG_PKT : 0);
	writel(GENMASK(input->lanes - 1, 0), priv->base + r + SENINF_CSI2_EN);
	writel(0, priv->base + r + SENINF_CSI2_HDR_MODE_0);

	for (i = 0; i < 8; i++)
		writel(0, priv->base + r + SENINF_CSI2_DI_CTRL(i));
	writel(FIELD_PREP(CSI2_DI_CTRL_DT, dt) | FIELD_PREP(CSI2_DI_CTRL_VC, 0) |
	       CSI2_DI_CTRL_EN, priv->base + r + SENINF_CSI2_DI_CTRL(0));
	for (i = 0; i < 4; i++)
		writel(0, priv->base + r + SENINF_CSI2_CH_CTRL(i));
	writel(CSI2_CH_CTRL_S0, priv->base + r + SENINF_CSI2_CH_CTRL(0));

	mtk_seninf_update(priv, r + SENINF_CSI2_CTRL, SENINF_CSI2_CTRL_EN,
			  SENINF_CSI2_CTRL_EN);
	mtk_seninf_update(priv, r + SENINF_CTRL, SENINF_CTRL_EN, SENINF_CTRL_EN);
	mtk_seninf_update(priv, SENINF_TOP_PHY_CTRL_CSI(input->csi),
			  TOP_PHY_CTRL_DPHY_EN | TOP_PHY_CTRL_CPHY_EN,
			  TOP_PHY_CTRL_DPHY_EN);
}

static void mtk_seninf_tpg_setup(struct mtk_seninf *priv, unsigned int id,
				 const struct v4l2_mbus_framefmt *fmt)
{
	unsigned int r = 0x1000 * id;

	mtk_seninf_update(priv, r + SENINF_TM_CTL, TM_CTL_PAT | TM_CTL_EN,
			  FIELD_PREP(TM_CTL_PAT, SENINF_TM_PATTERN));
	writel((fmt->height << 16) | fmt->width, priv->base + r + SENINF_TM_SIZE);
	writel(tm_clk, priv->base + r + SENINF_TM_CLK);
	writel(tm_dummy, priv->base + r + SENINF_TM_DUM);
	mtk_seninf_update(priv, r + SENINF_TM_CTL, TM_CTL_EN, TM_CTL_EN);
	mtk_seninf_update(priv, r + SENINF_TESTMDL_CTRL, SENINF_TESTMDL_EN,
			  SENINF_TESTMDL_EN);
	mtk_seninf_update(priv, r + SENINF_CTRL, SENINF_CTRL_EN, SENINF_CTRL_EN);
}

static void mtk_seninf_mux_setup(struct mtk_seninf *priv, unsigned int id, bool tpg)
{
	unsigned int m = id;	/* mux m takes receiver m */
	unsigned int r = 0x1000 * m;

	mtk_seninf_update(priv, r + SENINF_MUX_CTRL_1,
			  MUX_CTRL_1_SRC | MUX_CTRL_1_PIX_MODE,
			  FIELD_PREP(MUX_CTRL_1_SRC, tpg ? MUX_SRC_TEST_MODEL : MUX_SRC_MIPI));
	mtk_seninf_update(priv, r + SENINF_MUX_OPT,
			  MUX_OPT_HSYNC_POL | MUX_OPT_VSYNC_POL, 0);
	mtk_seninf_update(priv, r + SENINF_MUX_CTRL_0,
			  MUX_CTRL_0_IRQ_SW_RST | MUX_CTRL_0_SW_RST,
			  MUX_CTRL_0_IRQ_SW_RST | MUX_CTRL_0_SW_RST);
	udelay(1);
	mtk_seninf_update(priv, r + SENINF_MUX_CTRL_0,
			  MUX_CTRL_0_IRQ_SW_RST | MUX_CTRL_0_SW_RST, 0);
	mtk_seninf_update(priv, r + SENINF_MUX_CTRL_0, MUX_CTRL_0_EN, MUX_CTRL_0_EN);
	writel(~0U, priv->base + r + SENINF_MUX_IRQ_STATUS);

	mtk_seninf_update(priv, SENINF_TOP_MUX_CTRL(m), 0xf << SENINF_TOP_MUX_SHIFT(m),
			  id << SENINF_TOP_MUX_SHIFT(m));
}

static void mtk_seninf_cam_mux_setup(struct mtk_seninf *priv, unsigned int c,
				     unsigned int m, const struct v4l2_mbus_framefmt *fmt,
				     u8 dt, bool tpg)
{
	u32 opt = 0;

	mtk_seninf_update(priv, SENINF_CAM_MUX_CTRL(c), 0xf << SENINF_CAM_MUX_SHIFT(c),
			  m << SENINF_CAM_MUX_SHIFT(c));
	if (!tpg && cam_mux_filter)
		opt = CAM_MUX_OPT_VC_EN | CAM_MUX_OPT_DT_EN |
		      FIELD_PREP(CAM_MUX_OPT_VC_SEL, 0) |
		      FIELD_PREP(CAM_MUX_OPT_DT_SEL, dt);
	writel(opt, priv->base + SENINF_CAM_MUX_OPT(c));
	mtk_seninf_update(priv, SENINF_CAM_MUX_CHK_CTL_0(c), CAM_MUX_CHK_PIX_MODE, 0);
	writel((fmt->height << 16) | fmt->width,
	       priv->base + SENINF_CAM_MUX_CHK_CTL_1(c));
	mtk_seninf_update(priv, SENINF_CAM_MUX_EN, BIT(c), BIT(c));
	writel(3 << (2 * c), priv->base + SENINF_CAM_MUX_IRQ_STATUS);
}

static void mtk_seninf_hw_stop(struct mtk_seninf *priv, unsigned int id,
			       unsigned int outputs, struct mtk_seninf_input *input)
{
	unsigned int r = 0x1000 * id;

	mtk_seninf_update(priv, SENINF_CAM_MUX_EN, outputs, 0);
	mtk_seninf_update(priv, r + SENINF_MUX_CTRL_0, MUX_CTRL_0_EN, 0);
	mtk_seninf_update(priv, r + SENINF_TM_CTL, TM_CTL_EN, 0);
	mtk_seninf_update(priv, r + SENINF_TESTMDL_CTRL, SENINF_TESTMDL_EN, 0);
	mtk_seninf_update(priv, r + SENINF_CTRL, SENINF_CTRL_EN, 0);
	mtk_seninf_update(priv, r + SENINF_CSI2_CTRL, SENINF_CSI2_CTRL_EN, 0);
	writel(0, priv->base + r + SENINF_CSI2_EN);
	if (input)
		mtk_seninf_update(priv, SENINF_TOP_PHY_CTRL_CSI(input->csi),
				  TOP_PHY_CTRL_DPHY_EN | TOP_PHY_CTRL_CPHY_EN, 0);
}

/* -----------------------------------------------------------------------------
 * Streaming
 */

static struct mtk_seninf_input *mtk_seninf_active_input(struct mtk_seninf *priv)
{
	unsigned int i;

	for (i = 0; i < SENINF_NUM_INPUTS; i++) {
		struct mtk_seninf_input *input = &priv->inputs[i];

		if (input->sensor && media_pad_remote_pad_first(&priv->pads[i]))
			return input;
	}

	return NULL;
}

static int mtk_seninf_phy_start(struct mtk_seninf *priv,
				struct mtk_seninf_input *input,
				const struct mtk_seninf_format *fi)
{
	union phy_configure_opts opts = { };
	s64 link_freq;
	int ret;

	link_freq = v4l2_get_link_freq(input->sensor_pad, 0, 0);
	if (link_freq <= 0) {
		dev_err(priv->dev, "%s: no link frequency\n", input->sensor->name);
		return link_freq ? link_freq : -EINVAL;
	}

	opts.mipi_dphy.lanes = input->lanes;
	opts.mipi_dphy.hs_clk_rate = link_freq * 2;
	ret = phy_configure(input->phy, &opts);
	if (ret)
		return ret;

	ret = phy_power_on(input->phy);
	if (ret)
		return ret;

	dev_dbg(priv->dev, "seninf%u: %u lanes at %lld bps, DT 0x%02x\n",
		input->id + 1, input->lanes, link_freq * 2, fi->dt);

	return 0;
}

static int mtk_seninf_enable_streams(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state, u32 pad,
				     u64 streams_mask)
{
	struct mtk_seninf *priv = to_seninf(sd);
	unsigned int c = pad - SENINF_NUM_INPUTS;
	struct mtk_seninf_input *input;
	const struct v4l2_mbus_framefmt *fmt;
	const struct mtk_seninf_format *fi;
	unsigned int outputs, id, i;
	bool tpg;
	int ret;

	if (mtk_seninf_is_sink(pad))
		return -EINVAL;

	/* one stream at a time for now */
	if (priv->streaming_outputs)
		return -EBUSY;

	tpg = priv->test_pattern->val;
	input = mtk_seninf_active_input(priv);
	if (!tpg && !input) {
		dev_err(priv->dev, "no sensor link enabled\n");
		return -ENOLINK;
	}

	fmt = v4l2_subdev_state_get_format(state, pad);
	fi = mtk_seninf_find_format(fmt->code);
	if (!fi)
		return -EINVAL;

	id = input ? input->id : 0;
	outputs = BIT(c) | (cam_mux_extra & GENMASK(SENINF_NUM_OUTPUTS - 1, 0));

	ret = pm_runtime_resume_and_get(priv->dev);
	if (ret)
		return ret;

	dev_info(priv->dev, "start: seninf%u %s, %ux%u code 0x%04x, cam_mux mask 0x%04x\n",
		 id + 1, tpg ? "test pattern" : input->sensor->name,
		 fmt->width, fmt->height, fmt->code, outputs);

	if (tpg) {
		mtk_seninf_tpg_setup(priv, id, fmt);
	} else {
		mtk_seninf_csi2_setup(priv, input, fi->dt);
		ret = mtk_seninf_phy_start(priv, input, fi);
		if (ret)
			goto err_stop;
	}

	mtk_seninf_mux_setup(priv, id, tpg);
	for (i = 0; i < SENINF_NUM_OUTPUTS; i++)
		if (outputs & BIT(i))
			mtk_seninf_cam_mux_setup(priv, i, id, fmt, fi->dt, tpg);

	if (!tpg) {
		ret = v4l2_subdev_enable_streams(input->sensor,
						 input->sensor_pad->index, BIT(0));
		if (ret) {
			dev_err(priv->dev, "failed to start %s: %d\n",
				input->sensor->name, ret);
			phy_power_off(input->phy);
			goto err_stop;
		}
	}

	priv->streaming = input;
	priv->streaming_outputs = outputs;
	priv->streaming_tpg = tpg;

	return 0;

err_stop:
	mtk_seninf_hw_stop(priv, id, outputs, tpg ? NULL : input);
	pm_runtime_put(priv->dev);
	return ret;
}

static int mtk_seninf_disable_streams(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state, u32 pad,
				      u64 streams_mask)
{
	struct mtk_seninf *priv = to_seninf(sd);
	struct mtk_seninf_input *input = priv->streaming;
	unsigned int id = input ? input->id : 0;

	if (!priv->streaming_outputs)
		return 0;

	if (!priv->streaming_tpg) {
		v4l2_subdev_disable_streams(input->sensor, input->sensor_pad->index,
					    BIT(0));
	}

	mtk_seninf_dump_state(priv, id, priv->streaming_outputs);
	mtk_seninf_hw_stop(priv, id, priv->streaming_outputs,
			   priv->streaming_tpg ? NULL : input);
	if (!priv->streaming_tpg)
		phy_power_off(input->phy);

	priv->streaming = NULL;
	priv->streaming_outputs = 0;
	pm_runtime_put(priv->dev);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Subdev operations
 */

static const struct v4l2_mbus_framefmt mtk_seninf_default_fmt = {
	.code = SENINF_DEFAULT_CODE,
	.width = SENINF_DEFAULT_WIDTH,
	.height = SENINF_DEFAULT_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

static int mtk_seninf_init_state(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	unsigned int i;

	for (i = 0; i < SENINF_NUM_PADS; i++)
		*v4l2_subdev_state_get_format(state, i) = mtk_seninf_default_fmt;

	return 0;
}

static int mtk_seninf_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(mtk_seninf_formats))
		return -EINVAL;

	code->code = mtk_seninf_formats[code->index].code;

	return 0;
}

static int mtk_seninf_set_fmt(struct v4l2_subdev *sd,
			      const struct v4l2_subdev_client_info *ci,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = to_seninf(sd);
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming_outputs)
		return -EBUSY;

	if (!mtk_seninf_find_format(fmt->format.code))
		fmt->format.code = SENINF_DEFAULT_CODE;
	fmt->format.width = clamp(fmt->format.width, 16U, 8192U);
	fmt->format.height = clamp(fmt->format.height, 16U, 8192U);
	fmt->format.field = V4L2_FIELD_NONE;

	format = v4l2_subdev_state_get_format(state, fmt->pad);
	*format = fmt->format;

	/* The receiver passes frames through: outputs take the input's format. */
	if (mtk_seninf_is_sink(fmt->pad))
		for (i = SENINF_NUM_INPUTS; i < SENINF_NUM_PADS; i++)
			*v4l2_subdev_state_get_format(state, i) = fmt->format;

	return 0;
}

static int mtk_seninf_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops mtk_seninf_ctrl_ops = {
	.s_ctrl = mtk_seninf_s_ctrl,
};

static const char * const mtk_seninf_test_pattern_menu[] = {
	"Disabled",
	"Colour bars",
};

static const struct v4l2_subdev_core_ops mtk_seninf_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mtk_seninf_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops mtk_seninf_pad_ops = {
	.enum_mbus_code = mtk_seninf_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mtk_seninf_set_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
	.enable_streams = mtk_seninf_enable_streams,
	.disable_streams = mtk_seninf_disable_streams,
};

static const struct v4l2_subdev_ops mtk_seninf_subdev_ops = {
	.core = &mtk_seninf_core_ops,
	.video = &mtk_seninf_video_ops,
	.pad = &mtk_seninf_pad_ops,
};

static const struct v4l2_subdev_internal_ops mtk_seninf_internal_ops = {
	.init_state = mtk_seninf_init_state,
};

/* Only one receiver feeds the outputs: allow one enabled sensor link. */
static int mtk_seninf_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote, u32 flags)
{
	struct mtk_seninf *priv = to_seninf(media_entity_to_v4l2_subdev(entity));
	unsigned int i;

	if (!mtk_seninf_is_sink(local->index) || !(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	if (priv->streaming_outputs)
		return -EBUSY;

	for (i = 0; i < SENINF_NUM_INPUTS; i++)
		if (i != local->index && media_pad_remote_pad_first(&priv->pads[i]))
			return -EBUSY;

	return 0;
}

static const struct media_entity_operations mtk_seninf_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_setup = mtk_seninf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Async notifier
 */

static int mtk_seninf_notifier_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *sd,
				     struct v4l2_async_connection *asc)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf, notifier);
	struct mtk_seninf_asc *masc = container_of(asc, struct mtk_seninf_asc, asc);
	unsigned int port = masc->port;
	int ret;

	if (mtk_seninf_is_sink(port)) {
		struct mtk_seninf_input *input = &priv->inputs[port];
		int pad;

		pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode,
						  MEDIA_PAD_FL_SOURCE);
		if (pad < 0)
			return pad;

		/* disabled: userspace picks the sensor (or the test pattern) */
		ret = v4l2_create_fwnode_links_to_pad(sd, &priv->pads[port], 0);
		if (ret)
			return ret;

		input->sensor = sd;
		input->sensor_pad = &sd->entity.pads[pad];
	} else {
		ret = v4l2_create_fwnode_links_to_pad(&priv->sd, &sd->entity.pads[0],
						      MEDIA_LNK_FL_IMMUTABLE |
						      MEDIA_LNK_FL_ENABLED);
		if (ret)
			return ret;
	}

	dev_dbg(priv->dev, "%s bound to port %u\n", sd->name, port);

	return 0;
}

static void mtk_seninf_notifier_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *sd,
				       struct v4l2_async_connection *asc)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf, notifier);
	struct mtk_seninf_asc *masc = container_of(asc, struct mtk_seninf_asc, asc);

	if (mtk_seninf_is_sink(masc->port)) {
		priv->inputs[masc->port].sensor = NULL;
		priv->inputs[masc->port].sensor_pad = NULL;
	}
}

static int mtk_seninf_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf, notifier);

	return v4l2_device_register_subdev_nodes(&priv->v4l2_dev);
}

static const struct v4l2_async_notifier_operations mtk_seninf_notifier_ops = {
	.bound = mtk_seninf_notifier_bound,
	.unbind = mtk_seninf_notifier_unbind,
	.complete = mtk_seninf_notifier_complete,
};

/*
 * Receiver i takes its lanes from: i = 0, 2, 4: the A half of CSI port pair
 * i / 2, or both halves with more than 2 lanes; i = 1, 3, 5: the B half.
 */
static int mtk_seninf_parse_input(struct mtk_seninf *priv, unsigned int port,
				  struct fwnode_handle *ep)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct mtk_seninf_input *input = &priv->inputs[port];
	char name[8];
	int ret;

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		return dev_err_probe(priv->dev, ret, "port %u: bad endpoint\n", port);

	input->id = port;
	input->csi = port / 2;
	input->lanes = vep.bus.mipi_csi2.num_data_lanes;
	if (!input->lanes || input->lanes > SENINF_MAX_LANES ||
	    (input->lanes > 2 && (port & 1)))
		return dev_err_probe(priv->dev, -EINVAL,
				     "port %u: %u data lanes not supported\n",
				     port, input->lanes);

	if (input->lanes > 2)
		snprintf(name, sizeof(name), "csi%u", input->csi);
	else
		snprintf(name, sizeof(name), "csi%u%c", input->csi,
			 port & 1 ? 'b' : 'a');

	input->phy = devm_phy_get(priv->dev, name);
	if (IS_ERR(input->phy))
		return dev_err_probe(priv->dev, PTR_ERR(input->phy),
				     "port %u: no PHY %s\n", port, name);

	return 0;
}

static int mtk_seninf_async_register(struct mtk_seninf *priv)
{
	struct device *dev = priv->dev;
	struct fwnode_handle *ep;
	int ret;

	v4l2_async_nf_init(&priv->notifier, &priv->v4l2_dev);
	priv->notifier.ops = &mtk_seninf_notifier_ops;

	fwnode_graph_for_each_endpoint(dev_fwnode(dev), ep) {
		struct fwnode_handle *remote;
		struct fwnode_endpoint fwep;
		struct mtk_seninf_asc *masc;

		ret = fwnode_graph_parse_endpoint(ep, &fwep);
		if (ret)
			goto err;
		if (fwep.port >= SENINF_NUM_PADS) {
			ret = -EINVAL;
			goto err;
		}

		remote = fwnode_graph_get_remote_endpoint(ep);
		if (!remote)
			continue;
		fwnode_handle_put(remote);

		if (mtk_seninf_is_sink(fwep.port)) {
			ret = mtk_seninf_parse_input(priv, fwep.port, ep);
			if (ret)
				goto err;
		}

		masc = v4l2_async_nf_add_fwnode_remote(&priv->notifier, ep,
						       struct mtk_seninf_asc);
		if (IS_ERR(masc)) {
			ret = PTR_ERR(masc);
			goto err;
		}
		masc->port = fwep.port;
	}

	ret = v4l2_async_nf_register(&priv->notifier);
	if (ret)
		goto err_cleanup;

	return 0;

err:
	fwnode_handle_put(ep);
err_cleanup:
	v4l2_async_nf_cleanup(&priv->notifier);
	return ret;
}

static int mtk_seninf_v4l2_register(struct mtk_seninf *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct device *dev = priv->dev;
	unsigned int i;
	int ret;

	priv->mdev.dev = dev;
	strscpy(priv->mdev.model, "mtk-camsys-mt6833", sizeof(priv->mdev.model));
	media_device_init(&priv->mdev);

	priv->v4l2_dev.mdev = &priv->mdev;
	ret = v4l2_device_register(dev, &priv->v4l2_dev);
	if (ret)
		goto err_media;

	v4l2_subdev_init(sd, &mtk_seninf_subdev_ops);
	sd->internal_ops = &mtk_seninf_internal_ops;
	sd->dev = dev;
	sd->fwnode = dev_fwnode(dev);
	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &mtk_seninf_entity_ops;
	strscpy(sd->name, dev_name(dev), sizeof(sd->name));
	v4l2_set_subdevdata(sd, priv);

	for (i = 0; i < SENINF_NUM_PADS; i++)
		priv->pads[i].flags = mtk_seninf_is_sink(i) ? MEDIA_PAD_FL_SINK :
							      MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, SENINF_NUM_PADS, priv->pads);
	if (ret)
		goto err_v4l2;

	v4l2_ctrl_handler_init(&priv->ctrls, 1);
	priv->test_pattern =
		v4l2_ctrl_new_std_menu_items(&priv->ctrls, &mtk_seninf_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(mtk_seninf_test_pattern_menu) - 1,
					     0, 0, mtk_seninf_test_pattern_menu);
	if (priv->ctrls.error) {
		ret = priv->ctrls.error;
		goto err_ctrls;
	}
	sd->ctrl_handler = &priv->ctrls;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_ctrls;

	ret = v4l2_device_register_subdev(&priv->v4l2_dev, sd);
	if (ret)
		goto err_subdev;

	ret = mtk_seninf_async_register(priv);
	if (ret)
		goto err_unreg_subdev;

	ret = media_device_register(&priv->mdev);
	if (ret)
		goto err_notifier;

	return 0;

err_notifier:
	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
err_unreg_subdev:
	v4l2_device_unregister_subdev(sd);
err_subdev:
	v4l2_subdev_cleanup(sd);
err_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrls);
	media_entity_cleanup(&sd->entity);
err_v4l2:
	v4l2_device_unregister(&priv->v4l2_dev);
err_media:
	media_device_cleanup(&priv->mdev);
	return ret;
}

static int mtk_seninf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_seninf *priv;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	dev_set_drvdata(dev, priv);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->num_clks = devm_clk_bulk_get_all(dev, &priv->clks);
	if (priv->num_clks < 0)
		return dev_err_probe(dev, priv->num_clks, "failed to get the clocks\n");

	for (i = 0; i < priv->num_clks; i++)
		dev_info(dev, "clock %s: %lu Hz\n", priv->clks[i].id,
			 clk_get_rate(priv->clks[i].clk));

	devm_pm_runtime_enable(dev);

	ret = mtk_seninf_v4l2_register(priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register\n");

	return 0;
}

static void mtk_seninf_remove(struct platform_device *pdev)
{
	struct mtk_seninf *priv = dev_get_drvdata(&pdev->dev);

	media_device_unregister(&priv->mdev);
	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_subdev_cleanup(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrls);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_device_unregister(&priv->v4l2_dev);
	media_device_cleanup(&priv->mdev);
}

static int mtk_seninf_runtime_suspend(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	return 0;
}

static int mtk_seninf_runtime_resume(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(priv->num_clks, priv->clks);
}

static const struct dev_pm_ops mtk_seninf_pm_ops = {
	RUNTIME_PM_OPS(mtk_seninf_runtime_suspend, mtk_seninf_runtime_resume, NULL)
};

static const struct of_device_id mtk_seninf_of_match[] = {
	{ .compatible = "mediatek,mt6833-seninf" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_match);

static struct platform_driver mtk_seninf_driver = {
	.probe = mtk_seninf_probe,
	.remove = mtk_seninf_remove,
	.driver = {
		.name = "mtk-seninf-mt6833",
		.of_match_table = mtk_seninf_of_match,
		.pm = pm_ptr(&mtk_seninf_pm_ops),
	},
};
module_platform_driver(mtk_seninf_driver);

MODULE_DESCRIPTION("MediaTek MT6833 sensor interface driver");
MODULE_LICENSE("GPL");
