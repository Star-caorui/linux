// SPDX-License-Identifier: GPL-2.0-only
/*
 * SI-IN SIA8159 analog input speaker amplifier
 *
 * The SIA8159 takes an analog line level input and is switched and set
 * up over I2C only. There is no public datasheet: the register values
 * below are the "playback" startup and shutdown lists and the trim
 * check of the vendor's parameter file (sipa.bin, firmware version
 * 1.0.4) as shipped with the Xiaomi Redmi Note 13 5G, applied the way
 * the vendor driver (sound/soc/codecs/sipa in Xiaomi's kernel sources)
 * applies them. What the individual registers do is not known.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define SIA8159_REG_CHIP_ID	0x00
#define SIA8159_REG_EN		0x05
#define SIA8159_REG_06		0x06
#define SIA8159_REG_TRIM0	0x20
#define SIA8159_REG_TRIM1	0x21
#define SIA8159_REG_TRIM2	0x22
#define SIA8159_MAX_REG		0x22

struct sia8159_reg_seq {
	u8 reg;
	u8 val;
	unsigned int delay_us;
};

/* vendor parameter file, SIA8159, "playback" scene, startup list */
static const struct sia8159_reg_seq sia8159_startup[] = {
	{ 0x01, 0x7c }, { 0x02, 0x21 }, { 0x05, 0x01 }, { 0x03, 0x70 },
	{ 0x04, 0xc9 }, { 0x06, 0x54 }, { 0x07, 0x7c }, { 0x08, 0x8c },
	{ 0x09, 0x0b }, { 0x0a, 0xe4 }, { 0x14, 0x80 },
};

/* vendor parameter file, SIA8159, shutdown list (the same for all scenes) */
static const struct sia8159_reg_seq sia8159_shutdown[] = {
	{ 0x05, 0x00, 1000 }, { 0x01, 0x41 }, { 0x02, 0x20, 1000 },
};

/* vendor parameter file, SIA8159, trim values for a failed CRC */
#define SIA8159_TRIM0_DEFAULT	0x76
#define SIA8159_TRIM1_DEFAULT	0x66
#define SIA8159_TRIM2_DEFAULT	0x70

struct sia8159 {
	struct device *dev;
	struct regmap *regmap;
};

static int sia8159_write_seq(struct sia8159 *sia,
			     const struct sia8159_reg_seq *seq, size_t num)
{
	size_t i;
	int ret;

	for (i = 0; i < num; i++) {
		ret = regmap_write(sia->regmap, seq[i].reg, seq[i].val);
		if (ret)
			return ret;
		if (seq[i].delay_us)
			fsleep(seq[i].delay_us);
	}

	return 0;
}

/* CRC-4/ITU (x^4 + x + 1, reflected), as the vendor driver computes it */
static u8 sia8159_crc4(const u8 *data, size_t len)
{
	u8 crc = 0;
	int i;

	while (len--) {
		crc ^= *data++;
		for (i = 0; i < 8; i++)
			crc = (crc & 1) ? (crc >> 1) ^ 0x0c : crc >> 1;
	}

	return crc;
}

/*
 * The trim bytes are TRIM2[7:4], four zero bits, TRIM1 and TRIM0, with a
 * CRC-4 of them in TRIM2[3:0]. Like the vendor driver, rewrite them after
 * every startup: from the vendor defaults if the CRC does not match, and
 * otherwise from what was read, TRIM2 adjusted as the vendor driver does.
 */
static int sia8159_check_trim(struct sia8159 *sia)
{
	unsigned int t0, t1, t2;
	u8 trim[3], t2_new;
	int ret;

	/* the vendor driver waits this long for the trim to be loaded */
	usleep_range(1000, 2000);

	ret = regmap_read(sia->regmap, SIA8159_REG_TRIM0, &t0);
	if (!ret)
		ret = regmap_read(sia->regmap, SIA8159_REG_TRIM1, &t1);
	if (!ret)
		ret = regmap_read(sia->regmap, SIA8159_REG_TRIM2, &t2);
	if (ret)
		return ret;

	trim[0] = t2 & 0xf0;
	trim[1] = t1;
	trim[2] = t0;

	if (sia8159_crc4(trim, sizeof(trim)) != (t2 & 0x0f)) {
		dev_dbg(sia->dev, "trim CRC mismatch (%02x %02x %02x), using defaults\n",
			t0, t1, t2);
		t0 = SIA8159_TRIM0_DEFAULT;
		t1 = SIA8159_TRIM1_DEFAULT;
		t2_new = SIA8159_TRIM2_DEFAULT;
	} else {
		t2_new = trim[0] >= 0x20 ? trim[0] - 0x20 : 0;
	}

	ret = regmap_update_bits(sia->regmap, SIA8159_REG_06, BIT(1), BIT(1));
	if (!ret)
		ret = regmap_write(sia->regmap, SIA8159_REG_TRIM0, t0);
	if (!ret)
		ret = regmap_write(sia->regmap, SIA8159_REG_TRIM1, t1);
	if (!ret)
		ret = regmap_write(sia->regmap, SIA8159_REG_TRIM2, t2_new);

	return ret;
}

static int sia8159_power_on(struct sia8159 *sia)
{
	int ret;

	ret = sia8159_write_seq(sia, sia8159_startup, ARRAY_SIZE(sia8159_startup));
	if (!ret)
		ret = sia8159_check_trim(sia);

	return ret;
}

static int sia8159_power_off(struct sia8159 *sia)
{
	return sia8159_write_seq(sia, sia8159_shutdown, ARRAY_SIZE(sia8159_shutdown));
}

static int sia8159_drv_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sia8159 *sia = snd_soc_component_get_drvdata(component);
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = sia8159_power_on(sia);
		if (ret) {
			dev_err(sia->dev, "failed to switch on: %d\n", ret);
			sia8159_power_off(sia);
		}
		return ret;
	case SND_SOC_DAPM_PRE_PMD:
		ret = sia8159_power_off(sia);
		if (ret)
			dev_err(sia->dev, "failed to switch off: %d\n", ret);
		return ret;
	default:
		return 0;
	}
}

static const struct snd_soc_dapm_widget sia8159_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_OUT_DRV_E("OUT DRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sia8159_drv_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route sia8159_dapm_routes[] = {
	{ "OUT DRV", NULL, "IN" },
	{ "OUT", NULL, "OUT DRV" },
};

static const struct snd_soc_component_driver sia8159_component_driver = {
	.dapm_widgets = sia8159_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sia8159_dapm_widgets),
	.dapm_routes = sia8159_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(sia8159_dapm_routes),
};

/* only the registers the vendor driver touches: the others are not known */
static bool sia8159_writeable_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0x01 && reg <= 0x0a) || reg == 0x14 ||
	       (reg >= SIA8159_REG_TRIM0 && reg <= SIA8159_REG_TRIM2);
}

static bool sia8159_readable_reg(struct device *dev, unsigned int reg)
{
	return reg == SIA8159_REG_CHIP_ID || sia8159_writeable_reg(dev, reg);
}

static const struct regmap_config sia8159_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SIA8159_MAX_REG,
	.writeable_reg = sia8159_writeable_reg,
	.readable_reg = sia8159_readable_reg,
	.cache_type = REGCACHE_NONE,
};

static bool sia8159_id_supported(unsigned int id)
{
	/* the chip ID ranges the vendor parameter file gives for SIA8159 */
	return (id >= 0x60 && id <= 0x68) || (id >= 0x6a && id <= 0x6f);
}

static int sia8159_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sia8159 *sia;
	unsigned int id;
	int ret;

	sia = devm_kzalloc(dev, sizeof(*sia), GFP_KERNEL);
	if (!sia)
		return -ENOMEM;

	sia->dev = dev;
	i2c_set_clientdata(client, sia);

	sia->regmap = devm_regmap_init_i2c(client, &sia8159_regmap_config);
	if (IS_ERR(sia->regmap))
		return dev_err_probe(dev, PTR_ERR(sia->regmap),
				     "failed to init regmap\n");

	ret = regmap_read(sia->regmap, SIA8159_REG_CHIP_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	if (!sia8159_id_supported(id))
		return dev_err_probe(dev, -ENODEV, "unsupported chip ID 0x%02x\n", id);

	/* start switched off, whatever the boot loader left */
	ret = sia8159_power_off(sia);
	if (ret)
		return dev_err_probe(dev, ret, "failed to switch off\n");

	return devm_snd_soc_register_component(dev, &sia8159_component_driver,
					       NULL, 0);
}

static void sia8159_i2c_shutdown(struct i2c_client *client)
{
	struct sia8159 *sia = i2c_get_clientdata(client);

	sia8159_power_off(sia);
}

static const struct of_device_id sia8159_of_match[] = {
	{ .compatible = "si-in,sia8159" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sia8159_of_match);

static const struct i2c_device_id sia8159_i2c_id[] = {
	{ "sia8159" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, sia8159_i2c_id);

static struct i2c_driver sia8159_i2c_driver = {
	.driver = {
		.name = "sia8159",
		.of_match_table = sia8159_of_match,
	},
	.probe = sia8159_i2c_probe,
	.shutdown = sia8159_i2c_shutdown,
	.id_table = sia8159_i2c_id,
};
module_i2c_driver(sia8159_i2c_driver);

MODULE_DESCRIPTION("SI-IN SIA8159 speaker amplifier driver");
MODULE_LICENSE("GPL");
