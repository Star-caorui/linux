// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6833 camera DMA engine (camsv)
 *
 * A camsv instance takes one SENINF cam_mux output, crops it in its timing
 * generator (TG), packs it and writes it to memory through its IMGO DMA.
 * The driver exposes a subdev between the SENINF and a capture video node.
 *
 * The V4L2 structure follows MediaTek's MT8365 ISP 3.0 camsv series (v5);
 * the register programming is that of MediaTek's camera HAL for this SoC
 * (libimageio_plat_drv.so: CAMSV_TOP_CTRL, CAMSV_TG_CTRL, DMAO_CAMSV_B,
 * CAMSV_BUF_CTRL) and of the stock kernel's camera_isp.c.
 *
 * Buffers are handed to the DMA one frame ahead: at the start of a frame
 * the driver writes the address for the next one, which the double buffered
 * register takes at the end of the frame (an address written at the end of
 * a frame, in the done interrupt, only takes effect a frame later). A
 * scratch buffer takes the frames when userspace has none queued.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define CAMSV_MODULE_EN			0x010
#define  MODULE_EN_TG			BIT(0)
#define  MODULE_EN_PAK			BIT(2)
#define  MODULE_EN_PAK_SEL		BIT(3)
#define  MODULE_EN_IMGO			BIT(4)
#define  MODULE_EN_DB			BIT(30)
#define CAMSV_FMT_SEL			0x014
#define CAMSV_INT_EN			0x018
#define CAMSV_INT_STATUS		0x01c
#define  INT_VS1			BIT(0)
#define  INT_TG_ERR			BIT(4)
#define  INT_TG_GBERR			BIT(5)
#define  INT_SOF			BIT(6)
#define  INT_HW_PASS1_DON		BIT(11)
#define  INT_SW_PASS1_DON		BIT(12)
#define  INT_IMGO_ERR			BIT(16)
#define  INT_IMGO_OVERRUN		BIT(17)
#define  INT_ERRORS			(INT_TG_ERR | INT_TG_GBERR | INT_IMGO_ERR | \
					 INT_IMGO_OVERRUN)
#define CAMSV_SW_CTL			0x020
#define  SW_CTL_IMGO_RST_TRIG		BIT(0)
#define  SW_CTL_IMGO_RST_ST		BIT(1)
#define  SW_CTL_SW_RST			BIT(2)
#define CAMSV_IMGO_FBC_INC		0x02c
#define  IMGO_FBC_RCNT_INC		BIT(0)
#define CAMSV_CLK_EN			0x030
#define  CLK_EN_TG_DP			BIT(0)
#define  CLK_EN_PAK_DP			BIT(2)
#define  CLK_EN_DMA_DP			BIT(15)
#define CAMSV_PAK			0x04c
#define CAMSV_TG_SEN_MODE		0x130
#define  TG_SEN_MODE_CMOS_EN		BIT(0)
#define  TG_SEN_MODE_DBL_DATA_BUS	BIT(1)
#define  TG_SEN_MODE_DBL_DATA_BUS1	BIT(13)
#define  TG_SEN_MODE_TIME_STP_EN	BIT(16)
#define  TG_SEN_MODE_DS			GENMASK(18, 17)
#define CAMSV_TG_VF_CON			0x134
#define  TG_VF_CON_VFDATA_EN		BIT(0)
#define  TG_VF_CON_SINGLE_MODE		BIT(1)
#define CAMSV_TG_SEN_GRAB_PXL		0x138
#define CAMSV_TG_SEN_GRAB_LIN		0x13c
#define CAMSV_TG_FRMSIZE_ST		0x168
#define CAMSV_TG_INTER_ST		0x16c
#define CAMSV_TG_1A4			0x1a4
#define CAMSV_IMGO_BASE_ADDR		0x420
#define CAMSV_IMGO_BASE_ADDR_HI		0x428
#define CAMSV_IMGO_XSIZE		0x430
#define CAMSV_IMGO_YSIZE		0x434
#define CAMSV_IMGO_STRIDE		0x438
#define  IMGO_STRIDE_BUS		GENMASK(19, 16)
#define  IMGO_STRIDE_BIT27		BIT(27)
#define CAMSV_IMGO_CON			0x43c
#define CAMSV_IMGO_CON2			0x440
#define CAMSV_IMGO_CON3			0x444
#define CAMSV_IMGO_CROP			0x448
#define CAMSV_FH_EN			0x800
#define CAMSV_FBC_IMGO_CTL1		0xa00
#define  FBC_CTL1_BIT10			BIT(10)
#define  FBC_CTL1_FBC_MODE		BIT(11)
#define  FBC_CTL1_FBC_EN		BIT(15)
#define  FBC_CTL1_NUM			GENMASK(31, 24)
#define CAMSV_FBC_IMGO_CTL2		0xa04

#define CAMSV_INT_EN_VAL		(INT_VS1 | INT_SOF | INT_HW_PASS1_DON | \
					 INT_SW_PASS1_DON | INT_ERRORS)

#define CAMSV_PAD_SINK			0
#define CAMSV_PAD_SOURCE		1
#define CAMSV_NUM_PADS			2

#define CAMSV_MIN_SIZE			16
#define CAMSV_MAX_WIDTH			8192
#define CAMSV_MAX_HEIGHT		8192

static bool unpacked;
module_param(unpacked, bool, 0644);
MODULE_PARM_DESC(unpacked, "Write 10-bit pixels unpacked, 16 bits each (bring-up)");

/*
 * How to get 16-bit pixels (the HAL never does it). Measured with the
 * SC202CS gradient: 0 packer off: IMGO_ERR, nothing written; 1 PAK 0xc:
 * 16-bit words holding the top 8 bits; 2 PAK 0xb: IMGO_ERR; 3 packer on
 * with PAK_SEL: 16-bit words holding the 10 bits shifted left by 4.
 */
static unsigned int unpacked_mode = 3;
module_param(unpacked_mode, uint, 0644);
MODULE_PARM_DESC(unpacked_mode, "How the unpacked mode is set up (bring-up)");

static int pak_override = -1;
module_param(pak_override, int, 0644);
MODULE_PARM_DESC(pak_override, "PAK value instead of the format's (bring-up)");

static int fmt_sel_override = -1;
module_param(fmt_sel_override, int, 0644);
MODULE_PARM_DESC(fmt_sel_override, "FMT_SEL value instead of the format's (bring-up)");

static unsigned int done_irq = 12;
module_param(done_irq, uint, 0644);
MODULE_PARM_DESC(done_irq, "Interrupt status bit that ends a frame: 11 HW_PASS1_DON, 12 SW_PASS1_DON (bring-up)");

static bool fbc = true;
module_param(fbc, bool, 0644);
MODULE_PARM_DESC(fbc, "Use the frame buffer control counters (bring-up)");

struct mtk_camsv_format {
	u32 code;
	u32 fourcc;		/* packed */
	u32 fourcc_unpacked;
	u8 bpp;
	u8 fmt_sel;
	u8 pak;
};

static const struct mtk_camsv_format mtk_camsv_formats[] = {
	{ MEDIA_BUS_FMT_Y10_1X10, V4L2_PIX_FMT_Y10P, V4L2_PIX_FMT_Y10, 10, 1, 9 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10P, V4L2_PIX_FMT_SBGGR10, 10, 1, 9 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10P, V4L2_PIX_FMT_SGBRG10, 10, 1, 9 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10P, V4L2_PIX_FMT_SGRBG10, 10, 1, 9 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10P, V4L2_PIX_FMT_SRGGB10, 10, 1, 9 },
	{ MEDIA_BUS_FMT_Y8_1X8, V4L2_PIX_FMT_GREY, 0, 8, 0, 8 },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, 0, 8, 0, 8 },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_PIX_FMT_SGBRG8, 0, 8, 0, 8 },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_PIX_FMT_SGRBG8, 0, 8, 0, 8 },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_PIX_FMT_SRGGB8, 0, 8, 0, 8 },
};

struct mtk_camsv_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct mtk_camsv {
	struct device *dev;
	void __iomem *regs;
	int irq;
	struct clk_bulk_data *clks;
	int num_clks;

	struct v4l2_subdev sd;
	struct media_pad sd_pads[CAMSV_NUM_PADS];
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	struct mutex lock;		/* video node and queue */
	struct media_pipeline pipe;

	struct v4l2_pix_format fmt;
	const struct mtk_camsv_format *fi;

	spinlock_t irqlock;		/* buffers, counters */
	struct list_head buffers;	/* queued, not yet given to the DMA */
	struct mtk_camsv_buffer *cur;	/* being written (NULL: scratch) */
	struct mtk_camsv_buffer *next;	/* latched at the end of this frame */
	bool cur_valid;
	void *scratch;
	dma_addr_t scratch_addr;
	size_t scratch_size;
	unsigned int sequence;
	unsigned int irq_count[32];
	u32 irq_last;
	u32 int_en;

	struct media_pad *seninf_pad;
};

static inline struct mtk_camsv *sd_to_camsv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_camsv, sd);
}

static inline u32 mtk_camsv_fourcc(const struct mtk_camsv_format *fi)
{
	return unpacked && fi->fourcc_unpacked ? fi->fourcc_unpacked : fi->fourcc;
}

static const struct mtk_camsv_format *mtk_camsv_find_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_camsv_formats); i++)
		if (mtk_camsv_formats[i].code == code)
			return &mtk_camsv_formats[i];

	return NULL;
}

static const struct mtk_camsv_format *mtk_camsv_find_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_camsv_formats); i++)
		if (mtk_camsv_fourcc(&mtk_camsv_formats[i]) == fourcc)
			return &mtk_camsv_formats[i];

	return NULL;
}

static bool mtk_camsv_is_unpacked(const struct mtk_camsv_format *fi)
{
	return unpacked && fi->fourcc_unpacked;
}

/* bytes of pixel data in a line */
static u32 mtk_camsv_line_bytes(const struct mtk_camsv_format *fi, u32 width)
{
	if (mtk_camsv_is_unpacked(fi))
		return width * 2;

	return DIV_ROUND_UP(width * fi->bpp, 8);
}

static void mtk_camsv_set_bits(struct mtk_camsv *cam, u32 reg, u32 bits)
{
	writel(readl(cam->regs + reg) | bits, cam->regs + reg);
}

static void mtk_camsv_clear_bits(struct mtk_camsv *cam, u32 reg, u32 bits)
{
	writel(readl(cam->regs + reg) & ~bits, cam->regs + reg);
}

static void mtk_camsv_dump(struct mtk_camsv *cam, unsigned int start, unsigned int end)
{
	unsigned int off;

	for (off = start; off < end; off += 0x20)
		dev_info(cam->dev, "%04x: %08x %08x %08x %08x %08x %08x %08x %08x\n", off,
			 readl(cam->regs + off), readl(cam->regs + off + 0x4),
			 readl(cam->regs + off + 0x8), readl(cam->regs + off + 0xc),
			 readl(cam->regs + off + 0x10), readl(cam->regs + off + 0x14),
			 readl(cam->regs + off + 0x18), readl(cam->regs + off + 0x1c));
}

/* -----------------------------------------------------------------------------
 * Hardware
 */

static void mtk_camsv_reset(struct mtk_camsv *cam)
{
	u32 val;

	writel(0, cam->regs + CAMSV_SW_CTL);
	writel(SW_CTL_IMGO_RST_TRIG, cam->regs + CAMSV_SW_CTL);
	if (readl_poll_timeout_atomic(cam->regs + CAMSV_SW_CTL, val,
				      (val & (SW_CTL_IMGO_RST_TRIG | SW_CTL_IMGO_RST_ST)) ==
				      (SW_CTL_IMGO_RST_TRIG | SW_CTL_IMGO_RST_ST), 1, 1000))
		dev_warn(cam->dev, "DMA reset timed out (SW_CTL %08x)\n", val);
	writel(SW_CTL_SW_RST, cam->regs + CAMSV_SW_CTL);
	writel(0, cam->regs + CAMSV_SW_CTL);
}

/*
 * Hand a buffer to the DMA: latched at the next start of frame. The
 * register takes the low 32 bits of the IOVA; the M4U region's upper bits
 * come from the larb (its BANK_SEL). That holds because every buffer is
 * mapped by the DMA API on this device (vb2-dma-contig, imported dma-bufs
 * and the scratch buffer), so its IOVA lies in larb13's 4 GiB region
 * (8-12 GiB) and never crosses a 4 GiB boundary.
 */
static void mtk_camsv_queue_addr(struct mtk_camsv *cam, dma_addr_t addr)
{
	writel(lower_32_bits(addr), cam->regs + CAMSV_IMGO_BASE_ADDR);
	if (fbc)
		mtk_camsv_set_bits(cam, CAMSV_IMGO_FBC_INC, IMGO_FBC_RCNT_INC);
}

/* give the DMA the buffer for the next frame; called with irqlock held */
static void mtk_camsv_queue_next(struct mtk_camsv *cam)
{
	struct mtk_camsv_buffer *buf;

	buf = list_first_entry_or_null(&cam->buffers, struct mtk_camsv_buffer, list);
	if (buf) {
		list_del(&buf->list);
		mtk_camsv_queue_addr(cam, vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0));
	} else {
		mtk_camsv_queue_addr(cam, cam->scratch_addr);
	}
	cam->next = buf;
}

static void mtk_camsv_hw_start(struct mtk_camsv *cam)
{
	const struct mtk_camsv_format *fi = cam->fi;
	u32 width = cam->fmt.width, height = cam->fmt.height;
	unsigned long flags;
	u32 module_en, pak;

	mtk_camsv_reset(cam);

	module_en = MODULE_EN_TG | MODULE_EN_PAK;
	pak = fi->pak;
	if (mtk_camsv_is_unpacked(fi)) {
		switch (unpacked_mode) {
		case 0:
			module_en &= ~MODULE_EN_PAK;
			break;
		case 1:
			pak = 0xc;
			break;
		case 2:
			pak = 0xb;
			break;
		default:
			module_en |= MODULE_EN_PAK_SEL;
			break;
		}
	}
	writel(module_en, cam->regs + CAMSV_MODULE_EN);
	mtk_camsv_clear_bits(cam, 0x05c, BIT(0));
	mtk_camsv_clear_bits(cam, 0x058, BIT(7));
	if (pak_override >= 0)
		pak = pak_override;
	writel(fmt_sel_override >= 0 ? fmt_sel_override : fi->fmt_sel,
	       cam->regs + CAMSV_FMT_SEL);
	cam->int_en = CAMSV_INT_EN_VAL;
	writel(cam->int_en, cam->regs + CAMSV_INT_EN);
	writel(pak, cam->regs + CAMSV_PAK);
	writel((readl(cam->regs + 0x2d0) & ~GENMASK(20, 16)) | (0xe << 16),
	       cam->regs + 0x2d0);

	/* written by the HAL, meaning unknown; the frame header DMA stays off */
	writel(0, cam->regs + CAMSV_FH_EN);
	mtk_camsv_clear_bits(cam, 0x7b0, BIT(31));
	writel(~0U, cam->regs + 0x7c4);
	mtk_camsv_set_bits(cam, 0x418, 0x61000000);

	/* TG: one pixel per clock, continuous, grab the whole frame */
	mtk_camsv_clear_bits(cam, CAMSV_TG_SEN_MODE, TG_SEN_MODE_CMOS_EN |
			     TG_SEN_MODE_DBL_DATA_BUS | TG_SEN_MODE_DBL_DATA_BUS1 |
			     TG_SEN_MODE_DS);
	mtk_camsv_set_bits(cam, CAMSV_TG_SEN_MODE, TG_SEN_MODE_TIME_STP_EN);
	writel(0, cam->regs + CAMSV_TG_1A4);
	mtk_camsv_clear_bits(cam, CAMSV_TG_VF_CON, TG_VF_CON_VFDATA_EN |
			     TG_VF_CON_SINGLE_MODE);
	writel(width << 16, cam->regs + CAMSV_TG_SEN_GRAB_PXL);
	writel(height << 16, cam->regs + CAMSV_TG_SEN_GRAB_LIN);

	/* IMGO DMA */
	writel(mtk_camsv_line_bytes(fi, width) - 1, cam->regs + CAMSV_IMGO_XSIZE);
	writel(height - 1, cam->regs + CAMSV_IMGO_YSIZE);
	writel(cam->fmt.bytesperline | IMGO_STRIDE_BIT27 |
	       FIELD_PREP(IMGO_STRIDE_BUS, 1), cam->regs + CAMSV_IMGO_STRIDE);
	writel(0, cam->regs + CAMSV_IMGO_CROP);
	writel(0x80000080, cam->regs + CAMSV_IMGO_CON);
	writel(0x00400030, cam->regs + CAMSV_IMGO_CON2);
	writel(0x00200010, cam->regs + CAMSV_IMGO_CON3);

	writel(0, cam->regs + CAMSV_IMGO_BASE_ADDR_HI);
	writel(0, cam->regs + CAMSV_FBC_IMGO_CTL1);
	if (fbc)
		writel(FBC_CTL1_FBC_MODE, cam->regs + CAMSV_FBC_IMGO_CTL1);

	mtk_camsv_set_bits(cam, CAMSV_TG_SEN_MODE, TG_SEN_MODE_CMOS_EN);
	mtk_camsv_set_bits(cam, CAMSV_MODULE_EN, MODULE_EN_IMGO);
	if (fbc) {
		mtk_camsv_clear_bits(cam, CAMSV_FBC_IMGO_CTL1, FBC_CTL1_NUM);
		mtk_camsv_set_bits(cam, CAMSV_FBC_IMGO_CTL1, FBC_CTL1_FBC_EN);
		mtk_camsv_set_bits(cam, CAMSV_FBC_IMGO_CTL1, FBC_CTL1_BIT10);
	}

	/* the first buffer, before double buffering latches the registers */
	spin_lock_irqsave(&cam->irqlock, flags);
	cam->cur = NULL;
	cam->cur_valid = false;
	mtk_camsv_queue_next(cam);
	spin_unlock_irqrestore(&cam->irqlock, flags);

	mtk_camsv_set_bits(cam, CAMSV_CLK_EN, CLK_EN_TG_DP | CLK_EN_PAK_DP | CLK_EN_DMA_DP);
	mtk_camsv_clear_bits(cam, CAMSV_MODULE_EN, MODULE_EN_DB);
	mtk_camsv_set_bits(cam, CAMSV_MODULE_EN, MODULE_EN_DB);
	mtk_camsv_set_bits(cam, CAMSV_TG_VF_CON, TG_VF_CON_VFDATA_EN);
}

static void mtk_camsv_hw_stop(struct mtk_camsv *cam)
{
	unsigned int i;

	mtk_camsv_clear_bits(cam, CAMSV_TG_VF_CON, TG_VF_CON_VFDATA_EN);

	dev_info(cam->dev, "stop: frames %u, status %08x, FRMSIZE_ST %08x, TG_INTER_ST %08x, FBC %08x %08x\n",
		 cam->sequence, cam->irq_last,
		 readl(cam->regs + CAMSV_TG_FRMSIZE_ST),
		 readl(cam->regs + CAMSV_TG_INTER_ST),
		 readl(cam->regs + CAMSV_FBC_IMGO_CTL1),
		 readl(cam->regs + CAMSV_FBC_IMGO_CTL2));
	for (i = 0; i < 32; i++)
		if (cam->irq_count[i])
			dev_info(cam->dev, "interrupt bit %u: %u\n", i, cam->irq_count[i]);
	mtk_camsv_dump(cam, 0x000, 0x1c0);
	mtk_camsv_dump(cam, 0x2c0, 0x2e0);
	mtk_camsv_dump(cam, 0x400, 0x460);
	mtk_camsv_dump(cam, 0x7a0, 0x7e0);
	mtk_camsv_dump(cam, 0x800, 0x820);
	mtk_camsv_dump(cam, 0xa00, 0xa20);

	/* let a frame in flight end, then stop and reset the DMA */
	msleep(50);
	mtk_camsv_clear_bits(cam, CAMSV_TG_SEN_MODE, TG_SEN_MODE_CMOS_EN);
	writel(0, cam->regs + CAMSV_INT_EN);
	mtk_camsv_clear_bits(cam, CAMSV_MODULE_EN, MODULE_EN_IMGO);
	writel(0, cam->regs + CAMSV_FBC_IMGO_CTL1);
	mtk_camsv_reset(cam);
	readl(cam->regs + CAMSV_INT_STATUS);
}

static irqreturn_t mtk_camsv_irq(int irq, void *data)
{
	struct mtk_camsv *cam = data;
	struct mtk_camsv_buffer *buf;
	unsigned long status;
	unsigned int bit;

	status = readl(cam->regs + CAMSV_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	spin_lock(&cam->irqlock);

	cam->irq_last = status;
	for_each_set_bit(bit, &status, 32)
		cam->irq_count[bit]++;

	/*
	 * A stuck DMA raises its error bits again for every pixel it cannot
	 * write, millions of interrupts a second: report each error once and
	 * mask it for the rest of the stream.
	 */
	if (status & cam->int_en & INT_ERRORS) {
		dev_err(cam->dev, "error interrupt %08lx, masking %08lx\n", status,
			status & cam->int_en & INT_ERRORS);
		cam->int_en &= ~(status & INT_ERRORS);
		writel(cam->int_en, cam->regs + CAMSV_INT_EN);
	}

	/* the end of the frame being written: hand its buffer back */
	if ((status & BIT(done_irq)) && cam->cur_valid) {
		buf = cam->cur;
		if (buf) {
			buf->vb.sequence = cam->sequence;
			buf->vb.vb2_buf.timestamp = ktime_get_ns();
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
		cam->cur = NULL;
		cam->cur_valid = false;
		cam->sequence++;
	}

	/* a frame starts in the buffer given last; give the next one */
	if (status & INT_SOF) {
		if (cam->cur_valid && cam->cur)	/* its end was missed */
			vb2_buffer_done(&cam->cur->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		cam->cur = cam->next;
		cam->cur_valid = true;
		mtk_camsv_queue_next(cam);
	}

	spin_unlock(&cam->irqlock);

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 * vb2 queue
 */

static int mtk_camsv_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vq);

	if (*num_planes)
		return sizes[0] < cam->fmt.sizeimage ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = cam->fmt.sizeimage;

	return 0;
}

static int mtk_camsv_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < cam->fmt.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, cam->fmt.sizeimage);

	return 0;
}

static void mtk_camsv_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_camsv_buffer *buf =
		container_of(to_vb2_v4l2_buffer(vb), struct mtk_camsv_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&cam->irqlock, flags);
	list_add_tail(&buf->list, &cam->buffers);
	spin_unlock_irqrestore(&cam->irqlock, flags);
}

static void mtk_camsv_return_buffers(struct mtk_camsv *cam, enum vb2_buffer_state state)
{
	struct mtk_camsv_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&cam->irqlock, flags);
	if (cam->cur_valid && cam->cur)
		vb2_buffer_done(&cam->cur->vb.vb2_buf, state);
	if (cam->next && cam->next != cam->cur)
		vb2_buffer_done(&cam->next->vb.vb2_buf, state);
	cam->cur = NULL;
	cam->next = NULL;
	cam->cur_valid = false;
	list_for_each_entry_safe(buf, tmp, &cam->buffers, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&cam->irqlock, flags);
}

static int mtk_camsv_check_format(struct mtk_camsv *cam)
{
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(&cam->sd);
	fmt = v4l2_subdev_state_get_format(state, CAMSV_PAD_SOURCE);
	if (fmt->code != cam->fi->code || fmt->width != cam->fmt.width ||
	    fmt->height != cam->fmt.height) {
		dev_dbg(cam->dev, "format mismatch: %ux%u 0x%04x on the subdev\n",
			fmt->width, fmt->height, fmt->code);
		ret = -EPIPE;
	}
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int mtk_camsv_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vq);
	struct v4l2_subdev *seninf;
	int ret;

	ret = mtk_camsv_check_format(cam);
	if (ret)
		goto err_return;

	ret = video_device_pipeline_start(&cam->vdev, &cam->pipe);
	if (ret)
		goto err_return;

	cam->seninf_pad = media_pad_remote_pad_first(&cam->sd_pads[CAMSV_PAD_SINK]);
	if (!cam->seninf_pad) {
		ret = -ENOLINK;
		goto err_pipeline;
	}
	seninf = media_entity_to_v4l2_subdev(cam->seninf_pad->entity);

	cam->scratch_size = cam->fmt.sizeimage;
	cam->scratch = dma_alloc_coherent(cam->dev, cam->scratch_size,
					  &cam->scratch_addr, GFP_KERNEL);
	if (!cam->scratch) {
		ret = -ENOMEM;
		goto err_pipeline;
	}

	ret = pm_runtime_resume_and_get(cam->dev);
	if (ret)
		goto err_scratch;

	cam->sequence = 0;
	cam->irq_last = 0;
	memset(cam->irq_count, 0, sizeof(cam->irq_count));

	dev_info(cam->dev, "start: %ux%u %p4cc, %u bytes per line%s%s (mode %u)\n",
		 cam->fmt.width, cam->fmt.height, &cam->fmt.pixelformat,
		 cam->fmt.bytesperline, fbc ? ", FBC" : "",
		 mtk_camsv_is_unpacked(cam->fi) ? ", unpacked" : "", unpacked_mode);
	if (pak_override >= 0 || fmt_sel_override >= 0)
		dev_info(cam->dev, "PAK %d, FMT_SEL %d (overrides)\n", pak_override,
			 fmt_sel_override);

	mtk_camsv_hw_start(cam);
	enable_irq(cam->irq);

	ret = v4l2_subdev_enable_streams(seninf, cam->seninf_pad->index, BIT(0));
	if (ret) {
		dev_err(cam->dev, "failed to start %s: %d\n", seninf->name, ret);
		goto err_hw;
	}

	return 0;

err_hw:
	disable_irq(cam->irq);
	mtk_camsv_hw_stop(cam);
	pm_runtime_put(cam->dev);
err_scratch:
	dma_free_coherent(cam->dev, cam->scratch_size, cam->scratch, cam->scratch_addr);
	cam->scratch = NULL;
err_pipeline:
	video_device_pipeline_stop(&cam->vdev);
err_return:
	mtk_camsv_return_buffers(cam, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void mtk_camsv_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vq);
	struct v4l2_subdev *seninf = media_entity_to_v4l2_subdev(cam->seninf_pad->entity);

	v4l2_subdev_disable_streams(seninf, cam->seninf_pad->index, BIT(0));
	disable_irq(cam->irq);
	mtk_camsv_hw_stop(cam);
	pm_runtime_put(cam->dev);

	mtk_camsv_return_buffers(cam, VB2_BUF_STATE_ERROR);
	dma_free_coherent(cam->dev, cam->scratch_size, cam->scratch, cam->scratch_addr);
	cam->scratch = NULL;

	video_device_pipeline_stop(&cam->vdev);
}

static const struct vb2_ops mtk_camsv_vb2_ops = {
	.queue_setup = mtk_camsv_queue_setup,
	.buf_prepare = mtk_camsv_buf_prepare,
	.buf_queue = mtk_camsv_buf_queue,
	.start_streaming = mtk_camsv_start_streaming,
	.stop_streaming = mtk_camsv_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * Video node
 */

static void mtk_camsv_try_fmt(struct v4l2_pix_format *pix,
			      const struct mtk_camsv_format **fip)
{
	const struct mtk_camsv_format *fi;

	fi = mtk_camsv_find_fourcc(pix->pixelformat);
	if (!fi)
		fi = &mtk_camsv_formats[0];

	pix->pixelformat = mtk_camsv_fourcc(fi);
	pix->width = clamp(ALIGN(pix->width, 2), CAMSV_MIN_SIZE, CAMSV_MAX_WIDTH);
	pix->height = clamp(pix->height, CAMSV_MIN_SIZE, CAMSV_MAX_HEIGHT);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = ALIGN(mtk_camsv_line_bytes(fi, pix->width), 16);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->flags = 0;
	pix->priv = 0;

	if (fip)
		*fip = fi;
}

static int mtk_camsv_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct mtk_camsv *cam = video_drvdata(file);

	strscpy(cap->driver, "mtk-camsv", sizeof(cap->driver));
	strscpy(cap->card, dev_name(cam->dev), sizeof(cap->card));

	return 0;
}

static int mtk_camsv_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct mtk_camsv_format *fi;

	if (f->mbus_code) {
		fi = mtk_camsv_find_code(f->mbus_code);
		if (!fi || f->index)
			return -EINVAL;
	} else {
		if (f->index >= ARRAY_SIZE(mtk_camsv_formats))
			return -EINVAL;
		fi = &mtk_camsv_formats[f->index];
	}

	f->pixelformat = mtk_camsv_fourcc(fi);

	return 0;
}

static int mtk_camsv_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_camsv *cam = video_drvdata(file);

	f->fmt.pix = cam->fmt;

	return 0;
}

static int mtk_camsv_try_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	mtk_camsv_try_fmt(&f->fmt.pix, NULL);

	return 0;
}

static int mtk_camsv_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_camsv *cam = video_drvdata(file);

	if (vb2_is_busy(&cam->queue))
		return -EBUSY;

	mtk_camsv_try_fmt(&f->fmt.pix, &cam->fi);
	cam->fmt = f->fmt.pix;

	return 0;
}

static int mtk_camsv_enum_framesizes(struct file *file, void *fh,
				     struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !mtk_camsv_find_fourcc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = CAMSV_MIN_SIZE;
	fsize->stepwise.max_width = CAMSV_MAX_WIDTH;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = CAMSV_MIN_SIZE;
	fsize->stepwise.max_height = CAMSV_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;

	return 0;
}

static const struct v4l2_ioctl_ops mtk_camsv_ioctl_ops = {
	.vidioc_querycap = mtk_camsv_querycap,
	.vidioc_enum_fmt_vid_cap = mtk_camsv_enum_fmt,
	.vidioc_g_fmt_vid_cap = mtk_camsv_g_fmt,
	.vidioc_s_fmt_vid_cap = mtk_camsv_s_fmt,
	.vidioc_try_fmt_vid_cap = mtk_camsv_try_fmt_vid_cap,
	.vidioc_enum_framesizes = mtk_camsv_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations mtk_camsv_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int mtk_camsv_video_register(struct mtk_camsv *cam)
{
	struct video_device *vdev = &cam->vdev;
	struct vb2_queue *q = &cam->queue;
	int ret;

	cam->fmt.pixelformat = mtk_camsv_fourcc(&mtk_camsv_formats[0]);
	cam->fmt.width = 1600;
	cam->fmt.height = 1200;
	mtk_camsv_try_fmt(&cam->fmt, &cam->fi);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = cam->dev;
	q->drv_priv = cam;
	q->ops = &mtk_camsv_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mtk_camsv_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
			     V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	q->lock = &cam->lock;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	cam->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, &cam->vdev_pad);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), "%s capture", dev_name(cam->dev));
	vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;
	vdev->fops = &mtk_camsv_fops;
	vdev->ioctl_ops = &mtk_camsv_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &cam->lock;
	vdev->queue = q;
	vdev->v4l2_dev = cam->sd.v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	video_set_drvdata(vdev, cam);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	ret = media_create_pad_link(&cam->sd.entity, CAMSV_PAD_SOURCE, &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	video_unregister_device(vdev);
err_entity:
	media_entity_cleanup(&vdev->entity);
	return ret;
}

/* -----------------------------------------------------------------------------
 * Subdev
 */

static const struct v4l2_mbus_framefmt mtk_camsv_default_fmt = {
	.code = MEDIA_BUS_FMT_Y10_1X10,
	.width = 1600,
	.height = 1200,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

static int mtk_camsv_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	*v4l2_subdev_state_get_format(state, CAMSV_PAD_SINK) = mtk_camsv_default_fmt;
	*v4l2_subdev_state_get_format(state, CAMSV_PAD_SOURCE) = mtk_camsv_default_fmt;

	return 0;
}

static int mtk_camsv_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad == CAMSV_PAD_SOURCE) {
		if (code->index)
			return -EINVAL;
		code->code = v4l2_subdev_state_get_format(state, CAMSV_PAD_SINK)->code;
		return 0;
	}

	if (code->index >= ARRAY_SIZE(mtk_camsv_formats))
		return -EINVAL;
	code->code = mtk_camsv_formats[code->index].code;

	return 0;
}

static int mtk_camsv_set_fmt(struct v4l2_subdev *sd,
			     const struct v4l2_subdev_client_info *ci,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct mtk_camsv *cam = sd_to_camsv(sd);
	struct v4l2_mbus_framefmt *format;

	/* the source pad follows the sink pad */
	if (fmt->pad == CAMSV_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && vb2_is_busy(&cam->queue))
		return -EBUSY;

	if (!mtk_camsv_find_code(fmt->format.code))
		fmt->format.code = mtk_camsv_default_fmt.code;
	fmt->format.width = clamp(ALIGN(fmt->format.width, 2), CAMSV_MIN_SIZE,
				  CAMSV_MAX_WIDTH);
	fmt->format.height = clamp(fmt->format.height, CAMSV_MIN_SIZE, CAMSV_MAX_HEIGHT);
	fmt->format.field = V4L2_FIELD_NONE;

	format = v4l2_subdev_state_get_format(state, CAMSV_PAD_SINK);
	*format = fmt->format;
	*v4l2_subdev_state_get_format(state, CAMSV_PAD_SOURCE) = fmt->format;

	return 0;
}

static int mtk_camsv_registered(struct v4l2_subdev *sd)
{
	return mtk_camsv_video_register(sd_to_camsv(sd));
}

static void mtk_camsv_unregistered(struct v4l2_subdev *sd)
{
	struct mtk_camsv *cam = sd_to_camsv(sd);

	video_unregister_device(&cam->vdev);
	media_entity_cleanup(&cam->vdev.entity);
}

static const struct v4l2_subdev_pad_ops mtk_camsv_pad_ops = {
	.enum_mbus_code = mtk_camsv_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mtk_camsv_set_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
};

static const struct v4l2_subdev_ops mtk_camsv_subdev_ops = {
	.pad = &mtk_camsv_pad_ops,
};

static const struct v4l2_subdev_internal_ops mtk_camsv_internal_ops = {
	.init_state = mtk_camsv_init_state,
	.registered = mtk_camsv_registered,
	.unregistered = mtk_camsv_unregistered,
};

static const struct media_entity_operations mtk_camsv_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Probe
 */

static int mtk_camsv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_camsv *cam;
	int ret;

	cam = devm_kzalloc(dev, sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	cam->dev = dev;
	dev_set_drvdata(dev, cam);
	spin_lock_init(&cam->irqlock);
	INIT_LIST_HEAD(&cam->buffers);
	ret = devm_mutex_init(dev, &cam->lock);
	if (ret)
		return ret;

	/*
	 * The DMA goes through the M4U: with larb13's MMU bit clear for the
	 * port (physical addresses) it stalls and overruns at once. Its IOVAs
	 * are 34 bits, the region above 8 GiB for the camera larbs.
	 */
	if (!device_iommu_mapped(dev))
		return dev_err_probe(dev, -ENODEV, "not behind the M4U\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ret;

	cam->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cam->regs))
		return PTR_ERR(cam->regs);

	cam->irq = platform_get_irq(pdev, 0);
	if (cam->irq < 0)
		return cam->irq;
	ret = devm_request_irq(dev, cam->irq, mtk_camsv_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), cam);
	if (ret)
		return ret;

	cam->num_clks = devm_clk_bulk_get_all(dev, &cam->clks);
	if (cam->num_clks < 0)
		return dev_err_probe(dev, cam->num_clks, "failed to get the clocks\n");

	devm_pm_runtime_enable(dev);

	v4l2_subdev_init(&cam->sd, &mtk_camsv_subdev_ops);
	cam->sd.internal_ops = &mtk_camsv_internal_ops;
	cam->sd.dev = dev;
	cam->sd.owner = THIS_MODULE;
	cam->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	cam->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	cam->sd.entity.ops = &mtk_camsv_entity_ops;
	strscpy(cam->sd.name, dev_name(dev), sizeof(cam->sd.name));
	v4l2_set_subdevdata(&cam->sd, cam);

	cam->sd_pads[CAMSV_PAD_SINK].flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	cam->sd_pads[CAMSV_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&cam->sd.entity, CAMSV_NUM_PADS, cam->sd_pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(&cam->sd);
	if (ret)
		goto err_entity;

	ret = v4l2_async_register_subdev(&cam->sd);
	if (ret)
		goto err_subdev;

	return 0;

err_subdev:
	v4l2_subdev_cleanup(&cam->sd);
err_entity:
	media_entity_cleanup(&cam->sd.entity);
	return ret;
}

static void mtk_camsv_remove(struct platform_device *pdev)
{
	struct mtk_camsv *cam = dev_get_drvdata(&pdev->dev);

	v4l2_async_unregister_subdev(&cam->sd);
	v4l2_subdev_cleanup(&cam->sd);
	media_entity_cleanup(&cam->sd.entity);
}

static int mtk_camsv_runtime_suspend(struct device *dev)
{
	struct mtk_camsv *cam = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(cam->num_clks, cam->clks);

	return 0;
}

static int mtk_camsv_runtime_resume(struct device *dev)
{
	struct mtk_camsv *cam = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(cam->num_clks, cam->clks);
}

static const struct dev_pm_ops mtk_camsv_pm_ops = {
	RUNTIME_PM_OPS(mtk_camsv_runtime_suspend, mtk_camsv_runtime_resume, NULL)
};

static const struct of_device_id mtk_camsv_of_match[] = {
	{ .compatible = "mediatek,mt6833-camsv" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_camsv_of_match);

static struct platform_driver mtk_camsv_driver = {
	.probe = mtk_camsv_probe,
	.remove = mtk_camsv_remove,
	.driver = {
		.name = "mtk-camsv-mt6833",
		.of_match_table = mtk_camsv_of_match,
		.pm = pm_ptr(&mtk_camsv_pm_ops),
	},
};
module_platform_driver(mtk_camsv_driver);

MODULE_DESCRIPTION("MediaTek MT6833 camsv capture driver");
MODULE_LICENSE("GPL");
