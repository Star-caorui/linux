// SPDX-License-Identifier: GPL-2.0-only
/*
 * GalaxyCore GC16B3 16 Mpixel image sensor driver
 *
 * Copyright (C) 2026 Rui Cao <i@udp0.com>
 *
 * The register settings come from the GPL sensor driver Xiaomi ships for the
 * Redmi Note 13 5G (MediaTek imgsensor, gc16b3_sunny_front_mipi_raw). The
 * register layout is the one of the GC08A3 (drivers/media/i2c/gc08a3.c).
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

#define GC16B3_REG_TEST_PATTERN_EN	CCI_REG8(0x008c)
#define GC16B3_REG_TEST_PATTERN_IDX	CCI_REG8(0x008d)
#define GC16B3_TEST_PATTERN_EN		0x01

#define GC16B3_REG_MODE_SELECT		CCI_REG8(0x0100)
#define GC16B3_MODE_STANDBY		0x80
#define GC16B3_MODE_STREAMING		0x01

#define GC16B3_REG_EXPOSURE		CCI_REG16(0x0202)
#define GC16B3_EXPOSURE_MIN		4
#define GC16B3_EXPOSURE_MARGIN		16
#define GC16B3_EXPOSURE_STEP		1

/* 1024 is 1x */
#define GC16B3_REG_ANALOGUE_GAIN	CCI_REG16(0x0204)
#define GC16B3_GAIN_MIN			1024
#define GC16B3_GAIN_MAX			(16 * 1024)
#define GC16B3_GAIN_STEP		1

/* the frame length is a multiple of two lines */
#define GC16B3_REG_VTS			CCI_REG16(0x0340)
#define GC16B3_VTS_MAX			0xfffe

#define GC16B3_REG_CHIP_ID		CCI_REG16(0x03f0)
#define GC16B3_CHIP_ID			0x16b3

#define GC16B3_MCLK_FREQ		(24 * HZ_PER_MHZ)
#define GC16B3_DATA_LANES		4

#define GC16B3_PIXEL_ARRAY_WIDTH	4640
#define GC16B3_PIXEL_ARRAY_HEIGHT	3488
#define GC16B3_ACTIVE_LEFT		16
#define GC16B3_ACTIVE_TOP		16
#define GC16B3_ACTIVE_WIDTH		4608
#define GC16B3_ACTIVE_HEIGHT		3456

/* the vendor settings read the array mirrored and flipped, starting with red */
#define GC16B3_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10

static const s64 gc16b3_link_freq_menu[] = {
	181200000,
};

/* 2304x1728 at 30 fps: 3672 x 1832 x 30.3 */
#define GC16B3_PIXEL_RATE		204000000

static const char * const gc16b3_test_pattern_menu[] = {
	"Disabled",
	"Solid Black",
	"Colour Bars",
};

static const u8 gc16b3_test_pattern_idx[] = {
	0x00,	/* unused */
	0x00,
	0x10,
};

static const char * const gc16b3_supply_names[] = {
	"dovdd",	/* digital I/O, 1.8 V */
	"dvdd",		/* digital core, 1.2 V */
	"avdd",		/* analog, 2.8 V */
};

#define GC16B3_NUM_SUPPLIES		ARRAY_SIZE(gc16b3_supply_names)

struct gc16b3_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct gc16b3 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[GC16B3_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
};

#define to_gc16b3(_sd)	container_of(_sd, struct gc16b3, sd)

/* common to all modes */
static const struct cci_reg_sequence gc16b3_common_regs[] = {
	{ CCI_REG8(0x0315), 0xd7 },
	{ CCI_REG8(0x03a2), 0x0f },
	{ CCI_REG8(0x0321), 0x10 },
	{ CCI_REG8(0x0c0c), 0x33 },
	{ CCI_REG8(0x0187), 0x40 },
	{ CCI_REG8(0x0188), 0x5f },
	{ CCI_REG8(0x0335), 0x51 },
	{ CCI_REG8(0x0336), 0x97 },
	{ CCI_REG8(0x0314), 0x11 },
	{ CCI_REG8(0x031a), 0x00 },
	{ CCI_REG8(0x0337), 0x05 },
	{ CCI_REG8(0x0316), 0x08 },
	{ CCI_REG8(0x0c0e), 0x40 },
	{ CCI_REG8(0x0c0d), 0xac },
	{ CCI_REG8(0x0334), 0x40 },
	{ CCI_REG8(0x031c), 0xe0 },
	{ CCI_REG8(0x0311), 0xf8 },
	{ CCI_REG8(0x0268), 0x03 },
	{ CCI_REG8(0x02d1), 0x19 },
	{ CCI_REG8(0x05a0), 0x0a },
	{ CCI_REG8(0x05c3), 0x50 },
	{ CCI_REG8(0x0217), 0x20 },
	{ CCI_REG8(0x0074), 0x0a },
	{ CCI_REG8(0x00a0), 0x04 },
	{ CCI_REG8(0x0057), 0x0c },
	{ CCI_REG8(0x0358), 0x05 },
	{ CCI_REG8(0x0059), 0x11 },
	{ CCI_REG8(0x0084), 0x90 },
	{ CCI_REG8(0x0087), 0x51 },
	{ CCI_REG8(0x0c08), 0x19 },
	{ CCI_REG8(0x02d0), 0x40 },
	{ CCI_REG8(0x0101), 0x03 },
	{ CCI_REG8(0x0af0), 0x00 },
	{ CCI_REG8(0x0c15), 0x05 },
	{ CCI_REG8(0x0c55), 0x05 },
	{ CCI_REG8(0x0244), 0x15 },
	{ CCI_REG8(0x0245), 0x15 },
	{ CCI_REG8(0x0348), 0x12 },
	{ CCI_REG8(0x0349), 0x30 },
	{ CCI_REG8(0x0342), 0x07 },
	{ CCI_REG8(0x0343), 0x4e },
	{ CCI_REG8(0x0219), 0x05 },
	{ CCI_REG8(0x0e0a), 0x01 },
	{ CCI_REG8(0x0e0b), 0x01 },
	{ CCI_REG8(0x0e01), 0x75 },
	{ CCI_REG8(0x0e03), 0x44 },
	{ CCI_REG8(0x0e04), 0x44 },
	{ CCI_REG8(0x0e05), 0x44 },
	{ CCI_REG8(0x0e06), 0x44 },
	{ CCI_REG8(0x0e36), 0x06 },
	{ CCI_REG8(0x0e34), 0xf8 },
	{ CCI_REG8(0x0e35), 0x34 },
	{ CCI_REG8(0x0e15), 0x5a },
	{ CCI_REG8(0x0e16), 0xaa },
	{ CCI_REG8(0x025c), 0xe0 },
	{ CCI_REG8(0x0c05), 0xbf },
	{ CCI_REG8(0x0c09), 0x20 },
	{ CCI_REG8(0x0c41), 0x0a },
	{ CCI_REG8(0x0c42), 0x00 },
	{ CCI_REG8(0x0c44), 0x00 },
	{ CCI_REG8(0x0c45), 0xdf },
	{ CCI_REG8(0x0e42), 0x0f },
	{ CCI_REG8(0x0e44), 0x04 },
	{ CCI_REG8(0x0e48), 0x00 },
	{ CCI_REG8(0x0e4f), 0x04 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0xe0 },
	{ CCI_REG8(0x02db), 0x01 },
	{ CCI_REG8(0x0b00), 0x0f },
	{ CCI_REG8(0x0b01), 0xa2 },
	{ CCI_REG8(0x0b02), 0x03 },
	{ CCI_REG8(0x0b03), 0x07 },
	{ CCI_REG8(0x0b04), 0x11 },
	{ CCI_REG8(0x0b05), 0x14 },
	{ CCI_REG8(0x0b06), 0x03 },
	{ CCI_REG8(0x0b07), 0x07 },
	{ CCI_REG8(0x0b08), 0xac },
	{ CCI_REG8(0x0b09), 0x0d },
	{ CCI_REG8(0x0b0a), 0x0c },
	{ CCI_REG8(0x0b0b), 0x07 },
	{ CCI_REG8(0x0b0c), 0x40 },
	{ CCI_REG8(0x0b0d), 0x34 },
	{ CCI_REG8(0x0b0e), 0x03 },
	{ CCI_REG8(0x0b0f), 0x07 },
	{ CCI_REG8(0x0b10), 0x80 },
	{ CCI_REG8(0x0b11), 0x1c },
	{ CCI_REG8(0x0b12), 0x03 },
	{ CCI_REG8(0x0b13), 0x07 },
	{ CCI_REG8(0x0b14), 0x10 },
	{ CCI_REG8(0x0b15), 0xfe },
	{ CCI_REG8(0x0b16), 0x03 },
	{ CCI_REG8(0x0b17), 0x07 },
	{ CCI_REG8(0x0b18), 0x00 },
	{ CCI_REG8(0x0b19), 0xfe },
	{ CCI_REG8(0x0b1a), 0x03 },
	{ CCI_REG8(0x0b1b), 0x07 },
	{ CCI_REG8(0x0b1c), 0x9f },
	{ CCI_REG8(0x0b1d), 0x1c },
	{ CCI_REG8(0x0b1e), 0x03 },
	{ CCI_REG8(0x0b1f), 0x07 },
	{ CCI_REG8(0x0b20), 0x00 },
	{ CCI_REG8(0x0b21), 0xfe },
	{ CCI_REG8(0x0b22), 0x03 },
	{ CCI_REG8(0x0b23), 0x07 },
	{ CCI_REG8(0x0b24), 0x00 },
	{ CCI_REG8(0x0b25), 0xfe },
	{ CCI_REG8(0x0b26), 0x03 },
	{ CCI_REG8(0x0b27), 0x07 },
	{ CCI_REG8(0x0b28), 0x80 },
	{ CCI_REG8(0x0b29), 0x1c },
	{ CCI_REG8(0x0b2a), 0x03 },
	{ CCI_REG8(0x0b2b), 0x07 },
	{ CCI_REG8(0x0b2c), 0x10 },
	{ CCI_REG8(0x0b2d), 0xfe },
	{ CCI_REG8(0x0b2e), 0x03 },
	{ CCI_REG8(0x0b2f), 0x07 },
	{ CCI_REG8(0x0b30), 0x00 },
	{ CCI_REG8(0x0b31), 0xfe },
	{ CCI_REG8(0x0b32), 0x03 },
	{ CCI_REG8(0x0b33), 0x07 },
	{ CCI_REG8(0x0b34), 0x9f },
	{ CCI_REG8(0x0b35), 0x1c },
	{ CCI_REG8(0x0b36), 0x03 },
	{ CCI_REG8(0x0b37), 0x07 },
	{ CCI_REG8(0x0b38), 0x48 },
	{ CCI_REG8(0x0b39), 0x80 },
	{ CCI_REG8(0x0b3a), 0x01 },
	{ CCI_REG8(0x0b3b), 0x07 },
	{ CCI_REG8(0x0b3c), 0x10 },
	{ CCI_REG8(0x0b3d), 0x84 },
	{ CCI_REG8(0x0b3e), 0x00 },
	{ CCI_REG8(0x0b3f), 0x07 },
	{ CCI_REG8(0x0b40), 0xb8 },
	{ CCI_REG8(0x0b41), 0x11 },
	{ CCI_REG8(0x0b42), 0x03 },
	{ CCI_REG8(0x0b43), 0x07 },
	{ CCI_REG8(0x0b44), 0x99 },
	{ CCI_REG8(0x0b45), 0x02 },
	{ CCI_REG8(0x0b46), 0x01 },
	{ CCI_REG8(0x0b47), 0x07 },
	{ CCI_REG8(0x0b48), 0xd9 },
	{ CCI_REG8(0x0b49), 0x02 },
	{ CCI_REG8(0x0b4a), 0x01 },
	{ CCI_REG8(0x0b4b), 0x07 },
	{ CCI_REG8(0x0b4c), 0x00 },
	{ CCI_REG8(0x0b4d), 0xfe },
	{ CCI_REG8(0x0b4e), 0x03 },
	{ CCI_REG8(0x0b4f), 0x07 },
	{ CCI_REG8(0x0b50), 0x06 },
	{ CCI_REG8(0x0b51), 0x14 },
	{ CCI_REG8(0x0b52), 0x03 },
	{ CCI_REG8(0x0b53), 0x07 },
	{ CCI_REG8(0x0b54), 0x2c },
	{ CCI_REG8(0x0b55), 0x0d },
	{ CCI_REG8(0x0b56), 0x0c },
	{ CCI_REG8(0x0b57), 0x07 },
	{ CCI_REG8(0x0b58), 0x00 },
	{ CCI_REG8(0x0b59), 0x34 },
	{ CCI_REG8(0x0b5a), 0x03 },
	{ CCI_REG8(0x0b5b), 0x07 },
	{ CCI_REG8(0x0b5c), 0xe0 },
	{ CCI_REG8(0x0b5d), 0x1c },
	{ CCI_REG8(0x0b5e), 0x03 },
	{ CCI_REG8(0x0b5f), 0x07 },
	{ CCI_REG8(0x0b60), 0x90 },
	{ CCI_REG8(0x0b61), 0x84 },
	{ CCI_REG8(0x0b62), 0x00 },
	{ CCI_REG8(0x0b63), 0x07 },
	{ CCI_REG8(0x0b64), 0x08 },
	{ CCI_REG8(0x0b65), 0x80 },
	{ CCI_REG8(0x0b66), 0x01 },
	{ CCI_REG8(0x0b67), 0x07 },
	{ CCI_REG8(0x0b68), 0x07 },
	{ CCI_REG8(0x0b69), 0xa2 },
	{ CCI_REG8(0x0b6a), 0x03 },
	{ CCI_REG8(0x0b6b), 0x07 },
	{ CCI_REG8(0x0aab), 0x01 },
	{ CCI_REG8(0x0af0), 0x02 },
	{ CCI_REG8(0x0aa8), 0xb0 },
	{ CCI_REG8(0x0aa9), 0x92 },
	{ CCI_REG8(0x0aaa), 0x1b },
	{ CCI_REG8(0x0264), 0x00 },
	{ CCI_REG8(0x0265), 0x04 },
	{ CCI_REG8(0x0266), 0x1e },
	{ CCI_REG8(0x0267), 0x10 },
	{ CCI_REG8(0x0041), 0x30 },
	{ CCI_REG8(0x0043), 0x00 },
	{ CCI_REG8(0x0044), 0x01 },
	{ CCI_REG8(0x005b), 0x02 },
	{ CCI_REG8(0x0047), 0xf0 },
	{ CCI_REG8(0x0048), 0x0f },
	{ CCI_REG8(0x004b), 0x0f },
	{ CCI_REG8(0x004c), 0x00 },
	{ CCI_REG8(0x024a), 0x02 },
	{ CCI_REG8(0x0249), 0x00 },
	{ CCI_REG8(0x024f), 0x0e },
	{ CCI_REG8(0x024e), 0x80 },
	{ CCI_REG8(0x0c12), 0xe6 },
	{ CCI_REG8(0x0c52), 0xe6 },
	{ CCI_REG8(0x0c10), 0x20 },
	{ CCI_REG8(0x0c11), 0x58 },
	{ CCI_REG8(0x0c50), 0x20 },
	{ CCI_REG8(0x0c51), 0x58 },
	{ CCI_REG8(0x0460), 0x08 },
	{ CCI_REG8(0x0462), 0x06 },
	{ CCI_REG8(0x0464), 0x04 },
	{ CCI_REG8(0x0466), 0x02 },
	{ CCI_REG8(0x0468), 0x10 },
	{ CCI_REG8(0x046a), 0x0e },
	{ CCI_REG8(0x046c), 0x0e },
	{ CCI_REG8(0x046e), 0x0c },
	{ CCI_REG8(0x0461), 0x03 },
	{ CCI_REG8(0x0463), 0x03 },
	{ CCI_REG8(0x0465), 0x03 },
	{ CCI_REG8(0x0467), 0x03 },
	{ CCI_REG8(0x0469), 0x04 },
	{ CCI_REG8(0x046b), 0x04 },
	{ CCI_REG8(0x046d), 0x04 },
	{ CCI_REG8(0x046f), 0x04 },
	{ CCI_REG8(0x0470), 0x04 },
	{ CCI_REG8(0x0472), 0x08 },
	{ CCI_REG8(0x0474), 0x0c },
	{ CCI_REG8(0x0476), 0x10 },
	{ CCI_REG8(0x0478), 0x06 },
	{ CCI_REG8(0x047a), 0x06 },
	{ CCI_REG8(0x047c), 0x08 },
	{ CCI_REG8(0x047e), 0x08 },
	{ CCI_REG8(0x0471), 0x04 },
	{ CCI_REG8(0x0473), 0x04 },
	{ CCI_REG8(0x0475), 0x04 },
	{ CCI_REG8(0x0477), 0x04 },
	{ CCI_REG8(0x0479), 0x03 },
	{ CCI_REG8(0x047b), 0x03 },
	{ CCI_REG8(0x047d), 0x03 },
	{ CCI_REG8(0x047f), 0x03 },
};

/* 2304x1728 (2x2 binning), 30 fps, 24 MHz MCLK, 4 lanes at 362.4 Mbit/s */
static const struct cci_reg_sequence gc16b3_2304x1728_regs[] = {
	{ CCI_REG8(0x0315), 0xd3 },
	{ CCI_REG8(0x03a2), 0x0f },
	{ CCI_REG8(0x0321), 0x10 },
	{ CCI_REG8(0x0c0c), 0x33 },
	{ CCI_REG8(0x0187), 0x40 },
	{ CCI_REG8(0x0188), 0x5f },
	{ CCI_REG8(0x0335), 0x59 },
	{ CCI_REG8(0x0336), 0x97 },
	{ CCI_REG8(0x0314), 0x11 },
	{ CCI_REG8(0x031a), 0x01 },
	{ CCI_REG8(0x0337), 0x05 },
	{ CCI_REG8(0x0316), 0x08 },
	{ CCI_REG8(0x0c0e), 0x41 },
	{ CCI_REG8(0x0c0d), 0xac },
	{ CCI_REG8(0x0334), 0x40 },
	{ CCI_REG8(0x031c), 0xe0 },
	{ CCI_REG8(0x0311), 0xf8 },
	{ CCI_REG8(0x0268), 0x03 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0241), 0xd4 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x04 },
	{ CCI_REG8(0x034a), 0x0d },
	{ CCI_REG8(0x034b), 0xb0 },
	{ CCI_REG8(0x0342), 0x07 },
	{ CCI_REG8(0x0343), 0x2c },
	{ CCI_REG8(0x0226), 0x00 },
	{ CCI_REG8(0x0227), 0x40 },
	{ CCI_REG8(0x0202), 0x06 },
	{ CCI_REG8(0x0203), 0x8a },
	{ CCI_REG8(0x0340), 0x07 },
	{ CCI_REG8(0x0341), 0x28 },
	{ CCI_REG8(0x0e24), 0x02 },
	{ CCI_REG8(0x0e25), 0x02 },
	{ CCI_REG8(0x0e2c), 0x08 },
	{ CCI_REG8(0x0e2d), 0x0c },
	{ CCI_REG8(0x0e37), 0x41 },
	{ CCI_REG8(0x0e38), 0x41 },
	{ CCI_REG8(0x0e17), 0x36 },
	{ CCI_REG8(0x0e18), 0x39 },
	{ CCI_REG8(0x0e19), 0x60 },
	{ CCI_REG8(0x0e1a), 0x62 },
	{ CCI_REG8(0x0e49), 0x3a },
	{ CCI_REG8(0x0e2b), 0x6c },
	{ CCI_REG8(0x0e0c), 0x28 },
	{ CCI_REG8(0x0e28), 0x28 },
	{ CCI_REG8(0x0210), 0xa3 },
	{ CCI_REG8(0x02b5), 0x84 },
	{ CCI_REG8(0x02b6), 0x72 },
	{ CCI_REG8(0x02b7), 0x0e },
	{ CCI_REG8(0x02b8), 0x05 },
	{ CCI_REG8(0x0c07), 0xec },
	{ CCI_REG8(0x0c46), 0xfe },
	{ CCI_REG8(0x0c47), 0x02 },
	{ CCI_REG8(0x0e43), 0x00 },
	{ CCI_REG8(0x0e45), 0x04 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0xe0 },
	{ CCI_REG8(0x0360), 0x01 },
	{ CCI_REG8(0x0360), 0x00 },
	{ CCI_REG8(0x0a67), 0x80 },
	{ CCI_REG8(0x0313), 0x00 },
	{ CCI_REG8(0x0ace), 0x08 },
	{ CCI_REG8(0x0a53), 0x04 },
	{ CCI_REG8(0x0a65), 0x05 },
	{ CCI_REG8(0x0a68), 0x11 },
	{ CCI_REG8(0x0a58), 0x00 },
	{ CCI_REG8(0x00a4), 0x00 },
	{ CCI_REG8(0x00a5), 0x01 },
	{ CCI_REG8(0x00a2), 0x00 },
	{ CCI_REG8(0x00a3), 0x00 },
	{ CCI_REG8(0x00ab), 0x00 },
	{ CCI_REG8(0x00ac), 0x00 },
	{ CCI_REG8(0x00a7), 0x0d },
	{ CCI_REG8(0x00a8), 0xb0 },
	{ CCI_REG8(0x00a9), 0x12 },
	{ CCI_REG8(0x00aa), 0x30 },
	{ CCI_REG8(0x0a85), 0x1e },
	{ CCI_REG8(0x0a86), 0xa8 },
	{ CCI_REG8(0x0a8a), 0x00 },
	{ CCI_REG8(0x0a8b), 0xe0 },
	{ CCI_REG8(0x0a8c), 0x1e },
	{ CCI_REG8(0x0a8d), 0x10 },
	{ CCI_REG8(0x0a90), 0x08 },
	{ CCI_REG8(0x0a91), 0x1c },
	{ CCI_REG8(0x0a92), 0x78 },
	{ CCI_REG8(0x0a71), 0xd2 },
	{ CCI_REG8(0x0a72), 0x12 },
	{ CCI_REG8(0x0a73), 0x60 },
	{ CCI_REG8(0x0a75), 0x41 },
	{ CCI_REG8(0x0a70), 0x87 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x0042), 0x00 },
	{ CCI_REG8(0x0056), 0x00 },
	{ CCI_REG8(0x0488), 0x06 },
	{ CCI_REG8(0x048a), 0x06 },
	{ CCI_REG8(0x048c), 0x06 },
	{ CCI_REG8(0x048e), 0x06 },
	{ CCI_REG8(0x05a0), 0x82 },
	{ CCI_REG8(0x05ac), 0x00 },
	{ CCI_REG8(0x05ad), 0x01 },
	{ CCI_REG8(0x0597), 0x6b },
	{ CCI_REG8(0x059a), 0x00 },
	{ CCI_REG8(0x059b), 0x00 },
	{ CCI_REG8(0x059c), 0x01 },
	{ CCI_REG8(0x05a3), 0x0a },
	{ CCI_REG8(0x05a4), 0x08 },
	{ CCI_REG8(0x05ab), 0x0a },
	{ CCI_REG8(0x05ae), 0x00 },
	{ CCI_REG8(0x0108), 0x48 },
	{ CCI_REG8(0x010b), 0x12 },
	{ CCI_REG8(0x01c1), 0x95 },
	{ CCI_REG8(0x01c2), 0x00 },
	{ CCI_REG8(0x0800), 0x05 },
	{ CCI_REG8(0x0801), 0x06 },
	{ CCI_REG8(0x0802), 0x0a },
	{ CCI_REG8(0x0803), 0x0d },
	{ CCI_REG8(0x0804), 0x12 },
	{ CCI_REG8(0x0805), 0x17 },
	{ CCI_REG8(0x0806), 0x22 },
	{ CCI_REG8(0x0807), 0x2e },
	{ CCI_REG8(0x0808), 0x5a },
	{ CCI_REG8(0x0809), 0x0e },
	{ CCI_REG8(0x080a), 0x32 },
	{ CCI_REG8(0x080b), 0x0e },
	{ CCI_REG8(0x080c), 0x33 },
	{ CCI_REG8(0x080d), 0x02 },
	{ CCI_REG8(0x080e), 0xb8 },
	{ CCI_REG8(0x080f), 0x03 },
	{ CCI_REG8(0x0810), 0x1d },
	{ CCI_REG8(0x0811), 0x00 },
	{ CCI_REG8(0x0812), 0xc0 },
	{ CCI_REG8(0x0813), 0x03 },
	{ CCI_REG8(0x0814), 0x1d },
	{ CCI_REG8(0x0815), 0x03 },
	{ CCI_REG8(0x0816), 0x1e },
	{ CCI_REG8(0x0817), 0x03 },
	{ CCI_REG8(0x0818), 0x1e },
	{ CCI_REG8(0x0819), 0x02 },
	{ CCI_REG8(0x081a), 0x08 },
	{ CCI_REG8(0x081b), 0x3e },
	{ CCI_REG8(0x081c), 0x02 },
	{ CCI_REG8(0x081d), 0x00 },
	{ CCI_REG8(0x081e), 0x00 },
	{ CCI_REG8(0x081f), 0x01 },
	{ CCI_REG8(0x0820), 0x01 },
	{ CCI_REG8(0x0821), 0x02 },
	{ CCI_REG8(0x0822), 0x06 },
	{ CCI_REG8(0x0823), 0x3e },
	{ CCI_REG8(0x0824), 0x02 },
	{ CCI_REG8(0x0825), 0x00 },
	{ CCI_REG8(0x0826), 0x00 },
	{ CCI_REG8(0x0827), 0x01 },
	{ CCI_REG8(0x0828), 0x01 },
	{ CCI_REG8(0x0829), 0x02 },
	{ CCI_REG8(0x082a), 0x02 },
	{ CCI_REG8(0x082b), 0x3e },
	{ CCI_REG8(0x082c), 0x02 },
	{ CCI_REG8(0x082d), 0x00 },
	{ CCI_REG8(0x082e), 0x00 },
	{ CCI_REG8(0x082f), 0x01 },
	{ CCI_REG8(0x0830), 0x01 },
	{ CCI_REG8(0x0831), 0x01 },
	{ CCI_REG8(0x0832), 0x1c },
	{ CCI_REG8(0x0833), 0x3e },
	{ CCI_REG8(0x0834), 0x02 },
	{ CCI_REG8(0x0835), 0x00 },
	{ CCI_REG8(0x0836), 0x00 },
	{ CCI_REG8(0x0837), 0x01 },
	{ CCI_REG8(0x0838), 0x01 },
	{ CCI_REG8(0x0839), 0x01 },
	{ CCI_REG8(0x083a), 0x16 },
	{ CCI_REG8(0x083b), 0x3e },
	{ CCI_REG8(0x083c), 0x02 },
	{ CCI_REG8(0x083d), 0x00 },
	{ CCI_REG8(0x083e), 0x00 },
	{ CCI_REG8(0x083f), 0x01 },
	{ CCI_REG8(0x0840), 0x01 },
	{ CCI_REG8(0x0841), 0x01 },
	{ CCI_REG8(0x0842), 0x10 },
	{ CCI_REG8(0x0843), 0x3e },
	{ CCI_REG8(0x0844), 0x02 },
	{ CCI_REG8(0x0845), 0x00 },
	{ CCI_REG8(0x0846), 0x00 },
	{ CCI_REG8(0x0847), 0x01 },
	{ CCI_REG8(0x0848), 0x01 },
	{ CCI_REG8(0x0849), 0x01 },
	{ CCI_REG8(0x084a), 0x08 },
	{ CCI_REG8(0x084b), 0x3e },
	{ CCI_REG8(0x084c), 0x02 },
	{ CCI_REG8(0x084d), 0x00 },
	{ CCI_REG8(0x084e), 0x00 },
	{ CCI_REG8(0x084f), 0x01 },
	{ CCI_REG8(0x0850), 0x01 },
	{ CCI_REG8(0x0851), 0x00 },
	{ CCI_REG8(0x0852), 0x1e },
	{ CCI_REG8(0x0853), 0x3e },
	{ CCI_REG8(0x0854), 0x02 },
	{ CCI_REG8(0x0855), 0x00 },
	{ CCI_REG8(0x0856), 0x00 },
	{ CCI_REG8(0x0857), 0x01 },
	{ CCI_REG8(0x0858), 0x01 },
	{ CCI_REG8(0x0859), 0x00 },
	{ CCI_REG8(0x085a), 0x14 },
	{ CCI_REG8(0x085b), 0x3e },
	{ CCI_REG8(0x085c), 0x02 },
	{ CCI_REG8(0x085d), 0x02 },
	{ CCI_REG8(0x085e), 0x00 },
	{ CCI_REG8(0x085f), 0x01 },
	{ CCI_REG8(0x0860), 0x01 },
	{ CCI_REG8(0x0861), 0x00 },
	{ CCI_REG8(0x0862), 0x0c },
	{ CCI_REG8(0x0863), 0x36 },
	{ CCI_REG8(0x0864), 0x02 },
	{ CCI_REG8(0x0865), 0x02 },
	{ CCI_REG8(0x0866), 0x00 },
	{ CCI_REG8(0x0867), 0x01 },
	{ CCI_REG8(0x0868), 0x01 },
	{ CCI_REG8(0x0869), 0x00 },
	{ CCI_REG8(0x086a), 0x00 },
	{ CCI_REG8(0x086b), 0x01 },
	{ CCI_REG8(0x086c), 0x00 },
	{ CCI_REG8(0x086d), 0x01 },
	{ CCI_REG8(0x086e), 0x00 },
	{ CCI_REG8(0x086f), 0x00 },
	{ CCI_REG8(0x0870), 0x01 },
	{ CCI_REG8(0x0871), 0x01 },
	{ CCI_REG8(0x0872), 0x62 },
	{ CCI_REG8(0x0873), 0x00 },
	{ CCI_REG8(0x0874), 0x02 },
	{ CCI_REG8(0x0875), 0x01 },
	{ CCI_REG8(0x0876), 0xf8 },
	{ CCI_REG8(0x0877), 0x00 },
	{ CCI_REG8(0x0878), 0x03 },
	{ CCI_REG8(0x0879), 0x02 },
	{ CCI_REG8(0x087a), 0xc0 },
	{ CCI_REG8(0x087b), 0x00 },
	{ CCI_REG8(0x087c), 0x04 },
	{ CCI_REG8(0x087d), 0x03 },
	{ CCI_REG8(0x087e), 0xeb },
	{ CCI_REG8(0x087f), 0x00 },
	{ CCI_REG8(0x0880), 0x05 },
	{ CCI_REG8(0x0881), 0x05 },
	{ CCI_REG8(0x0882), 0x7a },
	{ CCI_REG8(0x0883), 0x00 },
	{ CCI_REG8(0x0884), 0x06 },
	{ CCI_REG8(0x0885), 0x07 },
	{ CCI_REG8(0x0886), 0xe0 },
	{ CCI_REG8(0x0887), 0x10 },
	{ CCI_REG8(0x0888), 0x05 },
	{ CCI_REG8(0x0889), 0x0b },
	{ CCI_REG8(0x088a), 0x02 },
	{ CCI_REG8(0x088b), 0x10 },
	{ CCI_REG8(0x088c), 0x06 },
	{ CCI_REG8(0x088d), 0x0f },
	{ CCI_REG8(0x088e), 0x92 },
	{ CCI_REG8(0x088f), 0x14 },
	{ CCI_REG8(0x0890), 0xb6 },
	{ CCI_REG8(0x0891), 0x1f },
	{ CCI_REG8(0x0892), 0xab },
	{ CCI_REG8(0x0893), 0x1a },
	{ CCI_REG8(0x0894), 0x66 },
	{ CCI_REG8(0x0895), 0x01 },
	{ CCI_REG8(0x0896), 0x46 },
	{ CCI_REG8(0x0897), 0x02 },
	{ CCI_REG8(0x0898), 0x01 },
	{ CCI_REG8(0x0899), 0x01 },
	{ CCI_REG8(0x089a), 0x01 },
	{ CCI_REG8(0x089b), 0x03 },
	{ CCI_REG8(0x089c), 0x4c },
	{ CCI_REG8(0x089d), 0x04 },
	{ CCI_REG8(0x089e), 0xff },
	{ CCI_REG8(0x089f), 0xff },
	{ CCI_REG8(0x08a0), 0x99 },
	{ CCI_REG8(0x08a1), 0x02 },
	{ CCI_REG8(0x08a2), 0x02 },
	{ CCI_REG8(0x08a3), 0x04 },
	{ CCI_REG8(0x08a4), 0x02 },
	{ CCI_REG8(0x08a5), 0x0e },
	{ CCI_REG8(0x08a6), 0x02 },
	{ CCI_REG8(0x08a7), 0x03 },
	{ CCI_REG8(0x08a8), 0x40 },
	{ CCI_REG8(0x08a9), 0x04 },
	{ CCI_REG8(0x08aa), 0xff },
	{ CCI_REG8(0x08ab), 0xff },
	{ CCI_REG8(0x08ac), 0x00 },
	{ CCI_REG8(0x05ac), 0x01 },
	{ CCI_REG8(0x0207), 0xc4 },
	{ CCI_REG8(0x05a0), 0xc2 },
	{ CCI_REG8(0x01c0), 0x01 },
	{ CCI_REG8(0x0096), 0x81 },
	{ CCI_REG8(0x0097), 0x08 },
	{ CCI_REG8(0x0098), 0x87 },
	{ CCI_REG8(0x0204), 0x04 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0208), 0x01 },
	{ CCI_REG8(0x0209), 0x6f },
	{ CCI_REG8(0x0351), 0x00 },
	{ CCI_REG8(0x0352), 0x0c },
	{ CCI_REG8(0x0353), 0x00 },
	{ CCI_REG8(0x0354), 0x0c },
	{ CCI_REG8(0x034c), 0x09 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x06 },
	{ CCI_REG8(0x034f), 0xc0 },
	{ CCI_REG8(0x0180), 0x48 },
	{ CCI_REG8(0x0181), 0xf0 },
	{ CCI_REG8(0x0185), 0x01 },
	{ CCI_REG8(0x0103), 0x10 },
	{ CCI_REG8(0x0106), 0x39 },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0115), 0x20 },
	{ CCI_REG8(0x0121), 0x02 },
	{ CCI_REG8(0x0122), 0x03 },
	{ CCI_REG8(0x0123), 0x0a },
	{ CCI_REG8(0x0124), 0x00 },
	{ CCI_REG8(0x0125), 0x08 },
	{ CCI_REG8(0x0126), 0x04 },
	{ CCI_REG8(0x0128), 0xf0 },
	{ CCI_REG8(0x0129), 0x03 },
	{ CCI_REG8(0x012a), 0x02 },
	{ CCI_REG8(0x012b), 0x05 },
	{ CCI_REG8(0x0a70), 0x11 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x0a70), 0x00 },
	{ CCI_REG8(0x0070), 0x05 },
	{ CCI_REG8(0x0089), 0x03 },
	{ CCI_REG8(0x009b), 0x40 },
	{ CCI_REG8(0x00a4), 0x80 },
	{ CCI_REG8(0x00a0), 0x05 },
	{ CCI_REG8(0x00a6), 0x07 },
	{ CCI_REG8(0x0080), 0xd2 },
	{ CCI_REG8(0x00c1), 0x80 },
	{ CCI_REG8(0x00c2), 0x11 },
	{ CCI_REG8(0x024d), 0x01 },
	{ CCI_REG8(0x0084), 0x10 },
	{ CCI_REG8(0x0268), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
};

static const struct gc16b3_mode gc16b3_modes[] = {
	{
		.width = 2304,
		.height = 1728,
		.hts = 3672,
		.vts = 1832,
		.regs = gc16b3_2304x1728_regs,
		.num_regs = ARRAY_SIZE(gc16b3_2304x1728_regs),
	},
};

static int gc16b3_set_test_pattern(struct gc16b3 *gc16b3, u32 pattern)
{
	int ret = 0;

	if (pattern)
		cci_write(gc16b3->regmap, GC16B3_REG_TEST_PATTERN_IDX,
			  gc16b3_test_pattern_idx[pattern], &ret);
	cci_write(gc16b3->regmap, GC16B3_REG_TEST_PATTERN_EN,
		  pattern ? GC16B3_TEST_PATTERN_EN : 0, &ret);

	return ret;
}

static int gc16b3_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc16b3 *gc16b3 = container_of(ctrl->handler, struct gc16b3, ctrl_handler);
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	s64 exposure_max;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&gc16b3->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = fmt->height + ctrl->val - GC16B3_EXPOSURE_MARGIN;
		ret = __v4l2_ctrl_modify_range(gc16b3->exposure,
					       gc16b3->exposure->minimum,
					       exposure_max,
					       gc16b3->exposure->step,
					       min(gc16b3->exposure->default_value,
						   exposure_max));
		if (ret)
			return ret;
	}

	/* The controls are written when the sensor is powered for streaming. */
	if (!pm_runtime_get_if_active(gc16b3->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(gc16b3->regmap, GC16B3_REG_EXPOSURE, ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(gc16b3->regmap, GC16B3_REG_ANALOGUE_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(gc16b3->regmap, GC16B3_REG_VTS,
				fmt->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = gc16b3_set_test_pattern(gc16b3, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(gc16b3->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc16b3_ctrl_ops = {
	.s_ctrl = gc16b3_set_ctrl,
};

static int gc16b3_init_controls(struct gc16b3 *gc16b3)
{
	struct v4l2_ctrl_handler *hdl = &gc16b3->ctrl_handler;
	const struct gc16b3_mode *mode = &gc16b3_modes[0];
	struct v4l2_fwnode_device_properties props;
	s64 exposure_max, hblank, vblank;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &gc16b3_ctrl_ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(gc16b3_link_freq_menu) - 1, 0,
				      gc16b3_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &gc16b3_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  GC16B3_PIXEL_RATE, GC16B3_PIXEL_RATE, 1,
			  GC16B3_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	ctrl = v4l2_ctrl_new_std(hdl, &gc16b3_ctrl_ops, V4L2_CID_HBLANK,
				 hblank, hblank, 1, hblank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	gc16b3->vblank = v4l2_ctrl_new_std(hdl, &gc16b3_ctrl_ops, V4L2_CID_VBLANK,
					   vblank, GC16B3_VTS_MAX - mode->height,
					   2, vblank);

	v4l2_ctrl_new_std(hdl, &gc16b3_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC16B3_GAIN_MIN, GC16B3_GAIN_MAX, GC16B3_GAIN_STEP,
			  GC16B3_GAIN_MIN);

	exposure_max = mode->vts - GC16B3_EXPOSURE_MARGIN;
	gc16b3->exposure = v4l2_ctrl_new_std(hdl, &gc16b3_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC16B3_EXPOSURE_MIN, exposure_max,
					     GC16B3_EXPOSURE_STEP, exposure_max);

	v4l2_ctrl_new_std_menu_items(hdl, &gc16b3_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc16b3_test_pattern_menu) - 1,
				     0, 0, gc16b3_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(gc16b3->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &gc16b3_ctrl_ops, &props);
	if (ret)
		goto err_free;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	gc16b3->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void gc16b3_update_pad_format(const struct gc16b3_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = GC16B3_MBUS_CODE;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int gc16b3_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc16b3 *gc16b3 = to_gc16b3(sd);
	const struct gc16b3_mode *mode = &gc16b3_modes[0];
	int ret;

	ret = pm_runtime_resume_and_get(gc16b3->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(gc16b3->regmap, gc16b3_common_regs,
				  ARRAY_SIZE(gc16b3_common_regs), NULL);
	if (ret)
		goto err_put;

	ret = cci_multi_reg_write(gc16b3->regmap, mode->regs, mode->num_regs, NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(gc16b3->sd.ctrl_handler);
	if (ret)
		goto err_put;

	ret = cci_write(gc16b3->regmap, GC16B3_REG_MODE_SELECT,
			GC16B3_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	dev_err(gc16b3->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(gc16b3->dev);
	return ret;
}

static int gc16b3_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc16b3 *gc16b3 = to_gc16b3(sd);
	int ret;

	ret = cci_write(gc16b3->regmap, GC16B3_REG_MODE_SELECT,
			GC16B3_MODE_STANDBY, NULL);
	if (ret)
		dev_err(gc16b3->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(gc16b3->dev);

	return ret;
}

static int gc16b3_set_pad_format(struct v4l2_subdev *sd,
				 const struct v4l2_subdev_client_info *ci,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	const struct gc16b3_mode *mode;

	mode = v4l2_find_nearest_size(gc16b3_modes, ARRAY_SIZE(gc16b3_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	gc16b3_update_pad_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int gc16b3_get_selection(struct v4l2_subdev *sd,
				const struct v4l2_subdev_client_info *ci,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = GC16B3_ACTIVE_LEFT;
		sel->r.top = GC16B3_ACTIVE_TOP;
		sel->r.width = GC16B3_ACTIVE_WIDTH;
		sel->r.height = GC16B3_ACTIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GC16B3_PIXEL_ARRAY_WIDTH;
		sel->r.height = GC16B3_PIXEL_ARRAY_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc16b3_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = GC16B3_MBUS_CODE;

	return 0;
}

static int gc16b3_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(gc16b3_modes) ||
	    fse->code != GC16B3_MBUS_CODE)
		return -EINVAL;

	fse->min_width = gc16b3_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = gc16b3_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int gc16b3_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.width = gc16b3_modes[0].width,
			.height = gc16b3_modes[0].height,
		},
	};

	return gc16b3_set_pad_format(sd, NULL, state, &fmt);
}

static const struct v4l2_subdev_video_ops gc16b3_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc16b3_pad_ops = {
	.enum_mbus_code = gc16b3_enum_mbus_code,
	.enum_frame_size = gc16b3_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc16b3_set_pad_format,
	.get_selection = gc16b3_get_selection,
	.enable_streams = gc16b3_enable_streams,
	.disable_streams = gc16b3_disable_streams,
};

static const struct v4l2_subdev_ops gc16b3_subdev_ops = {
	.video = &gc16b3_video_ops,
	.pad = &gc16b3_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc16b3_internal_ops = {
	.init_state = gc16b3_init_state,
};

static const struct media_entity_operations gc16b3_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int gc16b3_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc16b3 *gc16b3 = to_gc16b3(sd);
	int ret;

	/* the vendor order: reset held, DOVDD, DVDD, AVDD, MCLK, release */
	gpiod_set_value_cansleep(gc16b3->reset_gpio, 1);

	ret = regulator_bulk_enable(GC16B3_NUM_SUPPLIES, gc16b3->supplies);
	if (ret)
		return ret;

	fsleep(USEC_PER_MSEC);

	ret = clk_prepare_enable(gc16b3->mclk);
	if (ret)
		goto err_supplies;

	fsleep(USEC_PER_MSEC);
	gpiod_set_value_cansleep(gc16b3->reset_gpio, 0);
	fsleep(8 * USEC_PER_MSEC);

	return 0;

err_supplies:
	regulator_bulk_disable(GC16B3_NUM_SUPPLIES, gc16b3->supplies);
	return ret;
}

static int gc16b3_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc16b3 *gc16b3 = to_gc16b3(sd);

	gpiod_set_value_cansleep(gc16b3->reset_gpio, 1);
	regulator_bulk_disable(GC16B3_NUM_SUPPLIES, gc16b3->supplies);
	clk_disable_unprepare(gc16b3->mclk);

	return 0;
}

static int gc16b3_check_hwcfg(struct gc16b3 *gc16b3)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(gc16b3->dev), NULL);
	if (!ep)
		return dev_err_probe(gc16b3->dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(gc16b3->dev, ret, "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC16B3_DATA_LANES) {
		ret = dev_err_probe(gc16b3->dev, -EINVAL,
				    "only %u data lanes are supported\n",
				    GC16B3_DATA_LANES);
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(gc16b3->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       gc16b3_link_freq_menu,
				       ARRAY_SIZE(gc16b3_link_freq_menu),
				       &freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int gc16b3_identify(struct gc16b3 *gc16b3)
{
	u64 id;
	int ret;

	ret = cci_read(gc16b3->regmap, GC16B3_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(gc16b3->dev, ret, "failed to read the chip ID\n");

	if (id != GC16B3_CHIP_ID)
		return dev_err_probe(gc16b3->dev, -ENODEV,
				     "unexpected chip ID 0x%04llx\n", id);

	return 0;
}

static int gc16b3_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc16b3 *gc16b3;
	unsigned long freq;
	unsigned int i;
	int ret;

	gc16b3 = devm_kzalloc(dev, sizeof(*gc16b3), GFP_KERNEL);
	if (!gc16b3)
		return -ENOMEM;

	gc16b3->dev = dev;
	v4l2_i2c_subdev_init(&gc16b3->sd, client, &gc16b3_subdev_ops);

	gc16b3->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(gc16b3->regmap))
		return dev_err_probe(dev, PTR_ERR(gc16b3->regmap),
				     "failed to init CCI\n");

	gc16b3->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc16b3->mclk))
		return dev_err_probe(dev, PTR_ERR(gc16b3->mclk),
				     "failed to get the clock\n");

	freq = clk_get_rate(gc16b3->mclk);
	if (freq != GC16B3_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "clock rate %lu Hz is not supported\n", freq);

	ret = gc16b3_check_hwcfg(gc16b3);
	if (ret)
		return ret;

	gc16b3->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc16b3->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc16b3->reset_gpio),
				     "failed to get the reset GPIO\n");

	for (i = 0; i < GC16B3_NUM_SUPPLIES; i++)
		gc16b3->supplies[i].supply = gc16b3_supply_names[i];

	ret = devm_regulator_bulk_get(dev, GC16B3_NUM_SUPPLIES, gc16b3->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the supplies\n");

	ret = gc16b3_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = gc16b3_identify(gc16b3);
	if (ret)
		goto err_power_off;

	ret = gc16b3_init_controls(gc16b3);
	if (ret)
		goto err_power_off;

	gc16b3->sd.state_lock = gc16b3->ctrl_handler.lock;
	gc16b3->sd.internal_ops = &gc16b3_internal_ops;
	gc16b3->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc16b3->sd.entity.ops = &gc16b3_entity_ops;
	gc16b3->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc16b3->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc16b3->sd.entity, 1, &gc16b3->pad);
	if (ret)
		goto err_ctrls;

	ret = v4l2_subdev_init_finalize(&gc16b3->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&gc16b3->sd);
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
	v4l2_subdev_cleanup(&gc16b3->sd);
err_entity:
	media_entity_cleanup(&gc16b3->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&gc16b3->ctrl_handler);
err_power_off:
	gc16b3_power_off(dev);
	return ret;
}

static void gc16b3_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc16b3 *gc16b3 = to_gc16b3(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc16b3->ctrl_handler);

	pm_runtime_disable(gc16b3->dev);
	if (!pm_runtime_status_suspended(gc16b3->dev)) {
		gc16b3_power_off(gc16b3->dev);
		pm_runtime_set_suspended(gc16b3->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc16b3_pm_ops, gc16b3_power_off,
				 gc16b3_power_on, NULL);

static const struct of_device_id gc16b3_of_match[] = {
	{ .compatible = "galaxycore,gc16b3" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc16b3_of_match);

static struct i2c_driver gc16b3_driver = {
	.driver = {
		.name = "gc16b3",
		.pm = pm_ptr(&gc16b3_pm_ops),
		.of_match_table = gc16b3_of_match,
	},
	.probe = gc16b3_probe,
	.remove = gc16b3_remove,
};
module_i2c_driver(gc16b3_driver);

MODULE_DESCRIPTION("GalaxyCore GC16B3 image sensor driver");
MODULE_AUTHOR("Rui Cao <i@udp0.com>");
MODULE_LICENSE("GPL");
