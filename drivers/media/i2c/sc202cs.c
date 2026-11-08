// SPDX-License-Identifier: GPL-2.0-only
/*
 * SmartSens SC202CS 2 Mpixel monochrome image sensor driver
 *
 * Copyright (C) 2026 Rui Cao <i@udp0.com>
 *
 * The register settings come from the GPL sensor driver Xiaomi ships for the
 * Redmi Note 13 5G (MediaTek imgsensor, sc202cs_aac_depth_mipi_raw).
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SC202CS_REG_MODE_SELECT		CCI_REG8(0x0100)
#define SC202CS_MODE_STANDBY		0x00
#define SC202CS_MODE_STREAMING		0x01

#define SC202CS_REG_SOFTWARE_RESET	CCI_REG8(0x0103)
#define SC202CS_SOFTWARE_RESET		0x01

#define SC202CS_REG_CHIP_ID		CCI_REG16(0x3107)
#define SC202CS_CHIP_ID			0xeb52

#define SC202CS_REG_VTS			CCI_REG16(0x320e)
#define SC202CS_VTS_MAX			0xffff

#define SC202CS_REG_FLIP_MIRROR		CCI_REG8(0x3221)
#define SC202CS_MIRROR			(BIT(2) | BIT(1))
#define SC202CS_FLIP			(BIT(6) | BIT(5))

/* exposure in 1/16 lines, bits 19:0 */
#define SC202CS_REG_EXPOSURE		CCI_REG24(0x3e00)
#define SC202CS_EXPOSURE_MIN		2
#define SC202CS_EXPOSURE_MARGIN		4
#define SC202CS_EXPOSURE_STEP		1

/*
 * Gain: a coarse stage doubling per step (0x3e09: 0x00, 0x01, 0x03, ...) and a
 * fine stage from 1x to almost 2x in 1/128 steps (0x3e07: 0x80..0xff). The
 * control is the total gain in 1/128 steps, split between the two as the
 * vendor driver does; 16x is the vendor driver's limit.
 */
#define SC202CS_REG_FINE_GAIN		CCI_REG8(0x3e07)
#define SC202CS_REG_COARSE_GAIN		CCI_REG8(0x3e09)
#define SC202CS_GAIN_UNIT		128
#define SC202CS_GAIN_MIN		SC202CS_GAIN_UNIT
#define SC202CS_GAIN_MAX		(16 * SC202CS_GAIN_UNIT)
#define SC202CS_GAIN_STEP		1
#define SC202CS_GAIN_DEFAULT		SC202CS_GAIN_UNIT

#define SC202CS_REG_TEST_PATTERN	CCI_REG8(0x4501)
#define SC202CS_TEST_PATTERN_EN		BIT(3)

#define SC202CS_MCLK_FREQ		(24 * HZ_PER_MHZ)
#define SC202CS_LINK_FREQ		(360 * HZ_PER_MHZ)
#define SC202CS_DATA_LANES		1
#define SC202CS_BPP			10
/* one pixel per clock: 720 Mbit/s on one lane, 10 bits per pixel */
#define SC202CS_PIXEL_RATE		(SC202CS_LINK_FREQ * 2 * SC202CS_DATA_LANES / SC202CS_BPP)

#define SC202CS_NATIVE_WIDTH		1600
#define SC202CS_NATIVE_HEIGHT		1200

#define SC202CS_MBUS_CODE		MEDIA_BUS_FMT_Y10_1X10

static const s64 sc202cs_link_freq_menu[] = {
	SC202CS_LINK_FREQ,
};

static const char * const sc202cs_test_pattern_menu[] = {
	"Disabled",
	"Gradient",
};

static const char * const sc202cs_supply_names[] = {
	"dovdd",	/* digital I/O, 1.8 V */
	"avdd",		/* analog, 2.8 V */
};

#define SC202CS_NUM_SUPPLIES		ARRAY_SIZE(sc202cs_supply_names)

struct sc202cs_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct sc202cs {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[SC202CS_NUM_SUPPLIES];
	struct regulator *dvdd;		/* digital core, 1.2 V, if not made inside */

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

#define to_sc202cs(_sd)	container_of(_sd, struct sc202cs, sd)

/* 1600x1200, 30 fps, 24 MHz MCLK, 1 lane at 720 Mbit/s */
static const struct cci_reg_sequence sc202cs_1600x1200_regs[] = {
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x36e9), 0x24 },
	{ CCI_REG8(0x301f), 0x01 },
	{ CCI_REG8(0x3301), 0xff },
	{ CCI_REG8(0x3304), 0x68 },
	{ CCI_REG8(0x3306), 0x40 },
	{ CCI_REG8(0x3308), 0x08 },
	{ CCI_REG8(0x3309), 0xa8 },
	{ CCI_REG8(0x330b), 0xb0 },
	{ CCI_REG8(0x330c), 0x18 },
	{ CCI_REG8(0x330d), 0xff },
	{ CCI_REG8(0x330e), 0x20 },
	{ CCI_REG8(0x331e), 0x59 },
	{ CCI_REG8(0x331f), 0x99 },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x335e), 0x06 },
	{ CCI_REG8(0x335f), 0x08 },
	{ CCI_REG8(0x3364), 0x1f },
	{ CCI_REG8(0x337c), 0x02 },
	{ CCI_REG8(0x337d), 0x0a },
	{ CCI_REG8(0x338f), 0xa0 },
	{ CCI_REG8(0x3390), 0x01 },
	{ CCI_REG8(0x3391), 0x03 },
	{ CCI_REG8(0x3392), 0x1f },
	{ CCI_REG8(0x3393), 0xff },
	{ CCI_REG8(0x3394), 0xff },
	{ CCI_REG8(0x3395), 0xff },
	{ CCI_REG8(0x33a2), 0x04 },
	{ CCI_REG8(0x33ad), 0x0c },
	{ CCI_REG8(0x33b1), 0x20 },
	{ CCI_REG8(0x33b3), 0x38 },
	{ CCI_REG8(0x33f9), 0x40 },
	{ CCI_REG8(0x33fb), 0x48 },
	{ CCI_REG8(0x33fc), 0x0f },
	{ CCI_REG8(0x33fd), 0x1f },
	{ CCI_REG8(0x349f), 0x03 },
	{ CCI_REG8(0x34a6), 0x03 },
	{ CCI_REG8(0x34a7), 0x1f },
	{ CCI_REG8(0x34a8), 0x38 },
	{ CCI_REG8(0x34a9), 0x30 },
	{ CCI_REG8(0x34ab), 0xb0 },
	{ CCI_REG8(0x34ad), 0xb0 },
	{ CCI_REG8(0x34f8), 0x1f },
	{ CCI_REG8(0x34f9), 0x20 },
	{ CCI_REG8(0x3630), 0xa0 },
	{ CCI_REG8(0x3631), 0x92 },
	{ CCI_REG8(0x3632), 0x64 },
	{ CCI_REG8(0x3633), 0x43 },
	{ CCI_REG8(0x3637), 0x49 },
	{ CCI_REG8(0x363a), 0x85 },
	{ CCI_REG8(0x363c), 0x0f },
	{ CCI_REG8(0x3650), 0x31 },
	{ CCI_REG8(0x3670), 0x0d },
	{ CCI_REG8(0x3674), 0xc0 },
	{ CCI_REG8(0x3675), 0xa0 },
	{ CCI_REG8(0x3676), 0xa0 },
	{ CCI_REG8(0x3677), 0x92 },
	{ CCI_REG8(0x3678), 0x96 },
	{ CCI_REG8(0x3679), 0x9a },
	{ CCI_REG8(0x367c), 0x03 },
	{ CCI_REG8(0x367d), 0x0f },
	{ CCI_REG8(0x367e), 0x01 },
	{ CCI_REG8(0x367f), 0x0f },
	{ CCI_REG8(0x3698), 0x83 },
	{ CCI_REG8(0x3699), 0x86 },
	{ CCI_REG8(0x369a), 0x8c },
	{ CCI_REG8(0x369b), 0x94 },
	{ CCI_REG8(0x36a2), 0x01 },
	{ CCI_REG8(0x36a3), 0x03 },
	{ CCI_REG8(0x36a4), 0x07 },
	{ CCI_REG8(0x36ae), 0x0f },
	{ CCI_REG8(0x36af), 0x1f },
	{ CCI_REG8(0x36bd), 0x22 },
	{ CCI_REG8(0x36be), 0x22 },
	{ CCI_REG8(0x36bf), 0x22 },
	{ CCI_REG8(0x36d0), 0x01 },
	{ CCI_REG8(0x370f), 0x02 },
	{ CCI_REG8(0x3721), 0x6c },
	{ CCI_REG8(0x3722), 0x8d },
	{ CCI_REG8(0x3725), 0xc5 },
	{ CCI_REG8(0x3727), 0x14 },
	{ CCI_REG8(0x3728), 0x04 },
	{ CCI_REG8(0x37b7), 0x04 },
	{ CCI_REG8(0x37b8), 0x04 },
	{ CCI_REG8(0x37b9), 0x06 },
	{ CCI_REG8(0x37bd), 0x07 },
	{ CCI_REG8(0x37be), 0x0f },
	{ CCI_REG8(0x3901), 0x02 },
	{ CCI_REG8(0x3903), 0x40 },
	{ CCI_REG8(0x3905), 0x8d },
	{ CCI_REG8(0x3907), 0x00 },
	{ CCI_REG8(0x3908), 0x41 },
	{ CCI_REG8(0x391f), 0x41 },
	{ CCI_REG8(0x3933), 0x80 },
	{ CCI_REG8(0x3934), 0x02 },
	{ CCI_REG8(0x3937), 0x6f },
	{ CCI_REG8(0x393a), 0x01 },
	{ CCI_REG8(0x393d), 0x01 },
	{ CCI_REG8(0x393e), 0xc0 },
	{ CCI_REG8(0x39dd), 0x41 },
	{ CCI_REG8(0x3e00), 0x00 },
	{ CCI_REG8(0x3e01), 0x4d },
	{ CCI_REG8(0x3e02), 0xc0 },
	{ CCI_REG8(0x3e09), 0x00 },
	{ CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x61 },
};

static const struct sc202cs_mode sc202cs_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.hts = 1920,
		.vts = 1250,
		.regs = sc202cs_1600x1200_regs,
		.num_regs = ARRAY_SIZE(sc202cs_1600x1200_regs),
	},
};

static int sc202cs_set_gain(struct sc202cs *sc202cs, u32 gain)
{
	unsigned int coarse = 0;
	int ret = 0;

	/* coarse stage first: 2^coarse * fine / 128, fine in 128..255 */
	while (coarse < 5 && gain >= (SC202CS_GAIN_UNIT * 2) << coarse)
		coarse++;

	cci_write(sc202cs->regmap, SC202CS_REG_COARSE_GAIN, (1 << coarse) - 1, &ret);
	cci_write(sc202cs->regmap, SC202CS_REG_FINE_GAIN, gain >> coarse, &ret);

	return ret;
}

static int sc202cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc202cs *sc202cs = container_of(ctrl->handler, struct sc202cs, ctrl_handler);
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	s64 exposure_max;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&sc202cs->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = fmt->height + ctrl->val - SC202CS_EXPOSURE_MARGIN;
		ret = __v4l2_ctrl_modify_range(sc202cs->exposure,
					       sc202cs->exposure->minimum,
					       exposure_max,
					       sc202cs->exposure->step,
					       min(sc202cs->exposure->default_value,
						   exposure_max));
		if (ret)
			return ret;
	}

	/* The controls are written when the sensor is powered for streaming. */
	if (!pm_runtime_get_if_active(sc202cs->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(sc202cs->regmap, SC202CS_REG_EXPOSURE,
				ctrl->val << 4, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sc202cs_set_gain(sc202cs, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(sc202cs->regmap, SC202CS_REG_VTS,
				fmt->height + ctrl->val, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(sc202cs->regmap, SC202CS_REG_FLIP_MIRROR,
				(sc202cs->hflip->val ? SC202CS_MIRROR : 0) |
				(sc202cs->vflip->val ? SC202CS_FLIP : 0), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_update_bits(sc202cs->regmap, SC202CS_REG_TEST_PATTERN,
				      SC202CS_TEST_PATTERN_EN,
				      ctrl->val ? SC202CS_TEST_PATTERN_EN : 0, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sc202cs->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc202cs_ctrl_ops = {
	.s_ctrl = sc202cs_set_ctrl,
};

static int sc202cs_init_controls(struct sc202cs *sc202cs)
{
	struct v4l2_ctrl_handler *hdl = &sc202cs->ctrl_handler;
	const struct sc202cs_mode *mode = &sc202cs_modes[0];
	struct v4l2_fwnode_device_properties props;
	s64 exposure_max, hblank, vblank;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 11);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &sc202cs_ctrl_ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(sc202cs_link_freq_menu) - 1, 0,
				      sc202cs_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  SC202CS_PIXEL_RATE, SC202CS_PIXEL_RATE, 1,
			  SC202CS_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	ctrl = v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops, V4L2_CID_HBLANK,
				 hblank, hblank, 1, hblank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	sc202cs->vblank = v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops, V4L2_CID_VBLANK,
					    vblank, SC202CS_VTS_MAX - mode->height,
					    1, vblank);

	v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  SC202CS_GAIN_MIN, SC202CS_GAIN_MAX, SC202CS_GAIN_STEP,
			  SC202CS_GAIN_DEFAULT);

	exposure_max = mode->vts - SC202CS_EXPOSURE_MARGIN;
	sc202cs->exposure = v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      SC202CS_EXPOSURE_MIN, exposure_max,
					      SC202CS_EXPOSURE_STEP, exposure_max);

	sc202cs->hflip = v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);
	sc202cs->vflip = v4l2_ctrl_new_std(hdl, &sc202cs_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &sc202cs->hflip);

	v4l2_ctrl_new_std_menu_items(hdl, &sc202cs_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(sc202cs_test_pattern_menu) - 1,
				     0, 0, sc202cs_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(sc202cs->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &sc202cs_ctrl_ops, &props);
	if (ret)
		goto err_free;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	sc202cs->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void sc202cs_update_pad_format(const struct sc202cs_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = SC202CS_MBUS_CODE;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int sc202cs_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	const struct sc202cs_mode *mode = &sc202cs_modes[0];
	int ret;

	ret = pm_runtime_resume_and_get(sc202cs->dev);
	if (ret)
		return ret;

	ret = cci_write(sc202cs->regmap, SC202CS_REG_SOFTWARE_RESET,
			SC202CS_SOFTWARE_RESET, NULL);
	if (ret)
		goto err_put;
	/* the vendor driver waits 10 ms after the reset */
	fsleep(10 * USEC_PER_MSEC);

	ret = cci_multi_reg_write(sc202cs->regmap, mode->regs, mode->num_regs, NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(sc202cs->sd.ctrl_handler);
	if (ret)
		goto err_put;

	ret = cci_write(sc202cs->regmap, SC202CS_REG_MODE_SELECT,
			SC202CS_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	dev_err(sc202cs->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(sc202cs->dev);
	return ret;
}

static int sc202cs_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	int ret;

	ret = cci_write(sc202cs->regmap, SC202CS_REG_MODE_SELECT,
			SC202CS_MODE_STANDBY, NULL);
	if (ret)
		dev_err(sc202cs->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(sc202cs->dev);

	return ret;
}

static int sc202cs_set_pad_format(struct v4l2_subdev *sd,
				  const struct v4l2_subdev_client_info *ci,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	const struct sc202cs_mode *mode;

	mode = v4l2_find_nearest_size(sc202cs_modes, ARRAY_SIZE(sc202cs_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	sc202cs_update_pad_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int sc202cs_get_selection(struct v4l2_subdev *sd,
				 const struct v4l2_subdev_client_info *ci,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = SC202CS_NATIVE_WIDTH;
		sel->r.height = SC202CS_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc202cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = SC202CS_MBUS_CODE;

	return 0;
}

static int sc202cs_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(sc202cs_modes) ||
	    fse->code != SC202CS_MBUS_CODE)
		return -EINVAL;

	fse->min_width = sc202cs_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = sc202cs_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int sc202cs_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.width = sc202cs_modes[0].width,
			.height = sc202cs_modes[0].height,
		},
	};

	return sc202cs_set_pad_format(sd, NULL, state, &fmt);
}

static const struct v4l2_subdev_video_ops sc202cs_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc202cs_pad_ops = {
	.enum_mbus_code = sc202cs_enum_mbus_code,
	.enum_frame_size = sc202cs_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sc202cs_set_pad_format,
	.get_selection = sc202cs_get_selection,
	.enable_streams = sc202cs_enable_streams,
	.disable_streams = sc202cs_disable_streams,
};

static const struct v4l2_subdev_ops sc202cs_subdev_ops = {
	.video = &sc202cs_video_ops,
	.pad = &sc202cs_pad_ops,
};

static const struct v4l2_subdev_internal_ops sc202cs_internal_ops = {
	.init_state = sc202cs_init_state,
};

static const struct media_entity_operations sc202cs_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int sc202cs_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc202cs *sc202cs = to_sc202cs(sd);
	int ret;

	/* the vendor order: reset held, MCLK, DOVDD, AVDD (and DVDD), release */
	gpiod_set_value_cansleep(sc202cs->reset_gpio, 1);

	ret = clk_prepare_enable(sc202cs->mclk);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(SC202CS_NUM_SUPPLIES, sc202cs->supplies);
	if (ret)
		goto err_clk;

	if (sc202cs->dvdd) {
		ret = regulator_enable(sc202cs->dvdd);
		if (ret)
			goto err_supplies;
	}

	fsleep(USEC_PER_MSEC);
	gpiod_set_value_cansleep(sc202cs->reset_gpio, 0);
	fsleep(6 * USEC_PER_MSEC);

	return 0;

err_supplies:
	regulator_bulk_disable(SC202CS_NUM_SUPPLIES, sc202cs->supplies);
err_clk:
	clk_disable_unprepare(sc202cs->mclk);
	return ret;
}

static int sc202cs_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	gpiod_set_value_cansleep(sc202cs->reset_gpio, 1);
	if (sc202cs->dvdd)
		regulator_disable(sc202cs->dvdd);
	regulator_bulk_disable(SC202CS_NUM_SUPPLIES, sc202cs->supplies);
	clk_disable_unprepare(sc202cs->mclk);

	return 0;
}

static int sc202cs_check_hwcfg(struct sc202cs *sc202cs)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sc202cs->dev), NULL);
	if (!ep)
		return dev_err_probe(sc202cs->dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(sc202cs->dev, ret, "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != SC202CS_DATA_LANES) {
		ret = dev_err_probe(sc202cs->dev, -EINVAL,
				    "only %u data lane is supported\n",
				    SC202CS_DATA_LANES);
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(sc202cs->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       sc202cs_link_freq_menu,
				       ARRAY_SIZE(sc202cs_link_freq_menu),
				       &freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int sc202cs_identify(struct sc202cs *sc202cs)
{
	u64 id;
	int ret;

	ret = cci_read(sc202cs->regmap, SC202CS_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(sc202cs->dev, ret, "failed to read the chip ID\n");

	if (id != SC202CS_CHIP_ID)
		return dev_err_probe(sc202cs->dev, -ENODEV,
				     "unexpected chip ID 0x%04llx\n", id);

	return 0;
}

static int sc202cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc202cs *sc202cs;
	unsigned long freq;
	unsigned int i;
	int ret;

	sc202cs = devm_kzalloc(dev, sizeof(*sc202cs), GFP_KERNEL);
	if (!sc202cs)
		return -ENOMEM;

	sc202cs->dev = dev;
	v4l2_i2c_subdev_init(&sc202cs->sd, client, &sc202cs_subdev_ops);

	sc202cs->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sc202cs->regmap))
		return dev_err_probe(dev, PTR_ERR(sc202cs->regmap),
				     "failed to init CCI\n");

	sc202cs->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sc202cs->mclk))
		return dev_err_probe(dev, PTR_ERR(sc202cs->mclk),
				     "failed to get the clock\n");

	freq = clk_get_rate(sc202cs->mclk);
	if (freq != SC202CS_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "clock rate %lu Hz is not supported\n", freq);

	ret = sc202cs_check_hwcfg(sc202cs);
	if (ret)
		return ret;

	sc202cs->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sc202cs->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sc202cs->reset_gpio),
				     "failed to get the reset GPIO\n");

	for (i = 0; i < SC202CS_NUM_SUPPLIES; i++)
		sc202cs->supplies[i].supply = sc202cs_supply_names[i];

	ret = devm_regulator_bulk_get(dev, SC202CS_NUM_SUPPLIES, sc202cs->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the supplies\n");

	/* most boards leave the core supply to the sensor's own regulator */
	sc202cs->dvdd = devm_regulator_get_optional(dev, "dvdd");
	if (IS_ERR(sc202cs->dvdd)) {
		if (PTR_ERR(sc202cs->dvdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(sc202cs->dvdd),
					     "failed to get the dvdd supply\n");
		sc202cs->dvdd = NULL;
	}

	ret = sc202cs_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = sc202cs_identify(sc202cs);
	if (ret)
		goto err_power_off;

	ret = sc202cs_init_controls(sc202cs);
	if (ret)
		goto err_power_off;

	sc202cs->sd.state_lock = sc202cs->ctrl_handler.lock;
	sc202cs->sd.internal_ops = &sc202cs_internal_ops;
	sc202cs->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sc202cs->sd.entity.ops = &sc202cs_entity_ops;
	sc202cs->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sc202cs->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sc202cs->sd.entity, 1, &sc202cs->pad);
	if (ret)
		goto err_ctrls;

	ret = v4l2_subdev_init_finalize(&sc202cs->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&sc202cs->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register the subdev\n");
		goto err_pm;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sc202cs->sd);
err_entity:
	media_entity_cleanup(&sc202cs->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);
err_power_off:
	sc202cs_power_off(dev);
	return ret;
}

static void sc202cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);

	pm_runtime_disable(sc202cs->dev);
	if (!pm_runtime_status_suspended(sc202cs->dev)) {
		sc202cs_power_off(sc202cs->dev);
		pm_runtime_set_suspended(sc202cs->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(sc202cs_pm_ops, sc202cs_power_off,
				 sc202cs_power_on, NULL);

static const struct of_device_id sc202cs_of_match[] = {
	{ .compatible = "smartsens,sc202cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc202cs_of_match);

static struct i2c_driver sc202cs_driver = {
	.driver = {
		.name = "sc202cs",
		.pm = pm_ptr(&sc202cs_pm_ops),
		.of_match_table = sc202cs_of_match,
	},
	.probe = sc202cs_probe,
	.remove = sc202cs_remove,
};
module_i2c_driver(sc202cs_driver);

MODULE_DESCRIPTION("SmartSens SC202CS image sensor driver");
MODULE_AUTHOR("Rui Cao <i@udp0.com>");
MODULE_LICENSE("GPL");
