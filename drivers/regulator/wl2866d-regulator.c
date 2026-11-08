// SPDX-License-Identifier: GPL-2.0-only
/*
 * Will Semiconductor WL2866D camera PMIC: two DVDD and two AVDD LDOs behind I2C.
 *
 * Copyright (C) 2026 Rui Cao <i@udp0.com>
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define WL2866D_REG_DISCHARGE	0x02
#define WL2866D_REG_DVDD1	0x03
#define WL2866D_REG_DVDD2	0x04
#define WL2866D_REG_AVDD1	0x05
#define WL2866D_REG_AVDD2	0x06
#define WL2866D_REG_ENABLE	0x0e
#define WL2866D_REG_MAX		0x0f

enum wl2866d_ldo {
	WL2866D_DVDD1,
	WL2866D_DVDD2,
	WL2866D_AVDD1,
	WL2866D_AVDD2,
};

static const struct linear_range wl2866d_dvdd_range[] = {
	REGULATOR_LINEAR_RANGE(600000, 0x00, 0xc8, 6000),
};

static const struct linear_range wl2866d_avdd_range[] = {
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0xf8, 12500),
};

static const struct regulator_ops wl2866d_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_active_discharge = regulator_set_active_discharge_regmap,
};

/* The enable and discharge bits follow the LDOs' order: DVDD1, DVDD2, AVDD1, AVDD2. */
#define WL2866D_LDO(_id, _name, _supply, _vsel, _range, _n)		\
	[_id] = {							\
		.name = _name,						\
		.of_match = _name,					\
		.regulators_node = "regulators",			\
		.supply_name = _supply,					\
		.id = _id,						\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.ops = &wl2866d_ops,					\
		.linear_ranges = _range,				\
		.n_linear_ranges = ARRAY_SIZE(_range),			\
		.n_voltages = _n,					\
		.vsel_reg = _vsel,					\
		.vsel_mask = 0xff,					\
		.enable_reg = WL2866D_REG_ENABLE,			\
		.enable_mask = BIT(_id),				\
		.active_discharge_reg = WL2866D_REG_DISCHARGE,		\
		.active_discharge_mask = BIT(_id),			\
		.active_discharge_on = BIT(_id),			\
	}

static const struct regulator_desc wl2866d_regulators[] = {
	WL2866D_LDO(WL2866D_DVDD1, "dvdd1", "vin1", WL2866D_REG_DVDD1,
		    wl2866d_dvdd_range, 0xc9),
	WL2866D_LDO(WL2866D_DVDD2, "dvdd2", "vin1", WL2866D_REG_DVDD2,
		    wl2866d_dvdd_range, 0xc9),
	WL2866D_LDO(WL2866D_AVDD1, "avdd1", "vin2", WL2866D_REG_AVDD1,
		    wl2866d_avdd_range, 0xf9),
	WL2866D_LDO(WL2866D_AVDD2, "avdd2", "vin2", WL2866D_REG_AVDD2,
		    wl2866d_avdd_range, 0xf9),
};

static const struct regmap_config wl2866d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2866D_REG_MAX,
};

static int wl2866d_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { .dev = dev };
	struct regulator_dev *rdev;
	struct regmap *regmap;
	unsigned int val;
	int i, ret;

	regmap = devm_regmap_init_i2c(client, &wl2866d_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "failed to init regmap\n");

	/* The chip has no identification register: check that it answers. */
	ret = regmap_read(regmap, WL2866D_REG_ENABLE, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the enable register\n");

	config.regmap = regmap;
	for (i = 0; i < ARRAY_SIZE(wl2866d_regulators); i++) {
		rdev = devm_regulator_register(dev, &wl2866d_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "failed to register %s\n",
					     wl2866d_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "willsemi,wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static const struct i2c_device_id wl2866d_i2c_id[] = {
	{ "wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2866d_i2c_id);

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "wl2866d",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
	.id_table = wl2866d_i2c_id,
};
module_i2c_driver(wl2866d_driver);

MODULE_DESCRIPTION("Will Semiconductor WL2866D camera PMIC regulator driver");
MODULE_AUTHOR("Rui Cao <i@udp0.com>");
MODULE_LICENSE("GPL");
