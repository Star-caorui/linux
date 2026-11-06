// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech touchscreen controllers on SPI
 *
 * FocalTech's SPI protocol (its "v2", as in the FT3683G of the Xiaomi Redmi Note 13 5G): a transfer
 * starts with the register, a read (0xa0: 0x80 with CRC) or write (0x00) command and the 16-bit
 * length, then three dummy bytes. Status comes back in the fourth byte (0xa0 set: error); read data
 * follows the dummy bytes with a CRC-16 (CCITT, reflected, from 0xffff, low byte first). Touches are
 * read from register 0x01: an event type and finger count byte after one byte, then six bytes for
 * each of up to ten points, with 12-bit coordinates in the panel's pixels.
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
#define FTSPI_TOUCH_LEN		(2 + FTSPI_MAX_POINTS * FTSPI_POINT_LEN)
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

static irqreturn_t ftspi_irq(int irq, void *data)
{
	struct ftspi *ts = data;
	u8 buf[FTSPI_TOUCH_LEN];
	int i;

	if (ftspi_read(ts, FTSPI_REG_TOUCH, buf, sizeof(buf)))
		return IRQ_HANDLED;

	/* 0xff: the firmware is (re)starting; other event types are gestures and pens */
	if (buf[1] == 0xff || buf[1] >> 4)
		return IRQ_HANDLED;

	for (i = 0; i < FTSPI_MAX_POINTS; i++) {
		const u8 *p = buf + 2 + i * FTSPI_POINT_LEN;
		unsigned int id = p[2] >> 4;
		bool down = (p[0] >> 6) != FTSPI_EVENT_UP;

		if (id == FTSPI_ID_NONE)
			break;
		if (id >= FTSPI_MAX_POINTS)
			continue;

		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down);
		if (!down)
			continue;

		touchscreen_report_pos(ts->input, &ts->prop,
				       (p[0] & 0x0f) << 8 | p[1],
				       (p[2] & 0x0f) << 8 | p[3], true);
		input_report_abs(ts->input, ABS_MT_PRESSURE, p[4] ?: 0x3f);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, p[5] ?: 0x09);
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

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 4095, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 4095, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);

	ret = input_mt_init_slots(ts->input, FTSPI_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
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
