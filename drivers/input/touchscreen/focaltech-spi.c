// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech touchscreen controllers on SPI
 *
 * FocalTech's SPI protocol (its "v2", as in the FT3683G of the Xiaomi Redmi Note 13 5G): a transfer
 * starts with the register, a read (0xa0: 0x80 with CRC) or write (0x00) command and the 16-bit
 * length, then three dummy bytes. Status comes back in the fourth byte (0xa0 set: error); read data
 * follows the dummy bytes with a CRC-16 (CCITT, reflected, from 0xffff, low byte first). Touches are
 * read from register 0x01: after one byte, an event type and count byte. Type 0 is followed by six
 * bytes for each of up to ten points, with 12-bit coordinates in the panel's pixels; type 2 (the
 * FT3683G's) by two more bytes, then eight bytes for each point that changed, with 16-bit
 * coordinates in sixteenths of a pixel. Coordinates are reported in sixteenths of a pixel.
 */

#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define FTSPI_CMD_READ		0xa0
#define FTSPI_CMD_WRITE		0x00
#define FTSPI_STATUS_ERROR	0xa0
#define FTSPI_HEADER_LEN	4
#define FTSPI_DUMMY_LEN		3
#define FTSPI_CRC_LEN		2
#define FTSPI_RETRIES		3

#define FTSPI_REG_TOUCH		0x01
#define FTSPI_REG_CHIP_ID	0xa3
#define FTSPI_REG_CHIP_ID2	0x9f
#define FTSPI_REG_FW_VER	0xa6

#define FTSPI_MAX_POINTS	10
#define FTSPI_POINT_LEN		6
#define FTSPI_POINT_LEN_V2	8
#define FTSPI_TOUCH_LEN		(4 + FTSPI_MAX_POINTS * FTSPI_POINT_LEN_V2)

#define FTSPI_TYPE_V1		0
#define FTSPI_TYPE_V2		2
#define FTSPI_BUF_LEN		(FTSPI_HEADER_LEN + FTSPI_DUMMY_LEN + FTSPI_TOUCH_LEN + FTSPI_CRC_LEN)

#define FTSPI_EVENT_UP		1
#define FTSPI_ID_NONE		0x0f

static const char * const ftspi_supplies[] = { "avdd", "iovdd" };

struct ftspi {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset;
	u8 tx[FTSPI_BUF_LEN] ____cacheline_aligned;
	u8 rx[FTSPI_BUF_LEN] ____cacheline_aligned;
};

static int ftspi_transfer(struct ftspi *ts, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = ts->rx,
		.len = len,
	};
	int ret;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	/* the controller wants its chip select high for a while between transfers */
	usleep_range(150, 200);
	if (ret)
		return ret;

	return ts->rx[3] & FTSPI_STATUS_ERROR ? -EIO : 0;
}

static int ftspi_read(struct ftspi *ts, u8 reg, u8 *data, size_t len)
{
	size_t total = FTSPI_HEADER_LEN + FTSPI_DUMMY_LEN + len + FTSPI_CRC_LEN;
	u8 *d = ts->rx + FTSPI_HEADER_LEN + FTSPI_DUMMY_LEN;
	int i, ret = -EINVAL;

	if (total > FTSPI_BUF_LEN)
		return -EINVAL;

	for (i = 0; i < FTSPI_RETRIES; i++) {
		memset(ts->tx, 0, total);
		ts->tx[0] = reg;
		ts->tx[1] = FTSPI_CMD_READ;
		ts->tx[2] = len >> 8;
		ts->tx[3] = len;

		ret = ftspi_transfer(ts, total);
		if (ret)
			continue;
		if (crc_ccitt(0xffff, d, len) != get_unaligned_le16(d + len)) {
			ret = -EBADMSG;
			continue;
		}
		memcpy(data, d, len);
		return 0;
	}

	return ret;
}

static void ftspi_report(struct ftspi *ts, unsigned int id, bool down, unsigned int x,
			 unsigned int y, unsigned int major, unsigned int minor)
{
	input_mt_slot(ts->input, id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down);
	if (!down)
		return;

	touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major ?: 0x09);
	input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor ?: 0x09);
}

static irqreturn_t ftspi_irq(int irq, void *data)
{
	struct ftspi *ts = data;
	u8 buf[FTSPI_TOUCH_LEN];
	unsigned int type, count;
	int i;

	if (ftspi_read(ts, FTSPI_REG_TOUCH, buf, sizeof(buf)))
		return IRQ_HANDLED;

	/* all ones: the firmware is (re)starting */
	if (buf[1] == 0xff && buf[2] == 0xff && buf[3] == 0xff && buf[4] == 0xff)
		return IRQ_HANDLED;

	type = buf[1] >> 4;
	count = buf[1] & 0x0f;
	if (count > FTSPI_MAX_POINTS)
		return IRQ_HANDLED;

	switch (type) {
	case FTSPI_TYPE_V1:
		for (i = 0; i < FTSPI_MAX_POINTS; i++) {
			const u8 *p = buf + 2 + i * FTSPI_POINT_LEN;
			unsigned int id = p[2] >> 4;

			if (id >= FTSPI_MAX_POINTS)
				break;
			ftspi_report(ts, id, (p[0] >> 6) != FTSPI_EVENT_UP,
				     ((p[0] & 0x0f) << 8 | p[1]) * 16,
				     ((p[2] & 0x0f) << 8 | p[3]) * 16, p[5], p[5]);
		}
		break;
	case FTSPI_TYPE_V2:
		for (i = 0; i < count; i++) {
			const u8 *p = buf + 4 + i * FTSPI_POINT_LEN_V2;
			unsigned int id = p[2] >> 4;

			if (id >= FTSPI_MAX_POINTS)
				continue;
			ftspi_report(ts, id, (p[0] >> 6) != FTSPI_EVENT_UP,
				     (p[0] & 0x0f) << 12 | p[1] << 4 | p[4] >> 4,
				     (p[2] & 0x0f) << 12 | p[3] << 4 | (p[4] & 0x0f), p[5], p[6]);
		}
		break;
	default:
		/* gestures, pens and other messages */
		dev_dbg(&ts->spi->dev, "event type %u\n", type);
		return IRQ_HANDLED;
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

static int ftspi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ftspi *ts;
	u8 id[2], fw;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ftspi_supplies), ftspi_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	/* reset for 1 ms, then the firmware takes about 200 ms to come up */
	ts->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset))
		return dev_err_probe(dev, PTR_ERR(ts->reset), "failed to get the reset GPIO\n");
	if (ts->reset) {
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(ts->reset, 0);
	}
	msleep(200);

	ret = ftspi_read(ts, FTSPI_REG_CHIP_ID, &id[0], 1);
	if (!ret)
		ret = ftspi_read(ts, FTSPI_REG_CHIP_ID2, &id[1], 1);
	if (!ret)
		ret = ftspi_read(ts, FTSPI_REG_FW_VER, &fw, 1);
	if (ret)
		return dev_err_probe(dev, ret, "no answer to the chip ID read\n");
	dev_info(dev, "chip ID %02x%02x, firmware %02x\n", id[0], id[1], fw);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;
	ts->input->name = "FocalTech touchscreen";
	ts->input->id.bustype = BUS_SPI;

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 65535, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 65535, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);

	ret = input_mt_init_slots(ts->input, FTSPI_MAX_POINTS,
				  INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ftspi_irq, IRQF_ONESHOT,
					dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the interrupt\n");

	return input_register_device(ts->input);
}

static const struct of_device_id ftspi_of_match[] = {
	{ .compatible = "focaltech,ft3683g" },
	{ }
};
MODULE_DEVICE_TABLE(of, ftspi_of_match);

static const struct spi_device_id ftspi_ids[] = {
	{ "ft3683g" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ftspi_ids);

static struct spi_driver ftspi_driver = {
	.driver = {
		.name = "focaltech-spi",
		.of_match_table = ftspi_of_match,
	},
	.id_table = ftspi_ids,
	.probe = ftspi_probe,
};
module_spi_driver(ftspi_driver);

MODULE_DESCRIPTION("FocalTech touchscreen controllers on SPI");
MODULE_LICENSE("GPL");
