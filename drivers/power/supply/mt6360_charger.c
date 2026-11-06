// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MT6360_PMU_CHG_CTRL1	0x311
#define MT6360_PMU_CHG_CTRL2	0x312
#define MT6360_PMU_CHG_CTRL3	0x313
#define MT6360_PMU_CHG_CTRL4	0x314
#define MT6360_PMU_CHG_CTRL5	0x315
#define MT6360_PMU_CHG_CTRL6	0x316
#define MT6360_PMU_CHG_CTRL7	0x317
#define MT6360_PMU_CHG_CTRL8	0x318
#define MT6360_PMU_CHG_CTRL9	0x319
#define MT6360_PMU_CHG_CTRL10	0x31A
#define MT6360_PMU_CHG_CTRL13	0x31D
#define MT6360_PMU_DEVICE_TYPE	0x322
#define MT6360_PMU_USB_STATUS1	0x327
#define MT6360_PMU_CHG_STAT	0x34A
#define MT6360_PMU_CHG_CTRL19	0x361
#define MT6360_PMU_FOD_STAT	0x3E7

/* MT6360_PMU_CHG_CTRL1 */
#define MT6360_FSLP_SHFT	(3)
#define MT6360_FSLP_MASK	BIT(MT6360_FSLP_SHFT)
#define MT6360_OPA_MODE_SHFT	(0)
#define MT6360_OPA_MODE_MASK	BIT(MT6360_OPA_MODE_SHFT)
/* MT6360_PMU_CHG_CTRL2 */
#define MT6360_IINLMTSEL_SHFT	(2)
#define MT6360_IINLMTSEL_MASK	GENMASK(3, 2)
#define MT6360_TE_EN_MASK	BIT(4)
#define MT6360_CHG_EN_MASK	BIT(0)
/* MT6360_PMU_CHG_CTRL3 */
#define MT6360_IAICR_SHFT	(2)
#define MT6360_IAICR_MASK	GENMASK(7, 2)
#define MT6360_ILIM_EN_MASK	BIT(0)
/* MT6360_PMU_CHG_CTRL4 */
#define MT6360_VOREG_SHFT	(1)
#define MT6360_VOREG_MASK	GENMASK(7, 1)
/* MT6360_PMU_CHG_CTRL5 */
#define MT6360_VOBST_MASK	GENMASK(7, 2)
/* MT6360_PMU_CHG_CTRL6 */
#define MT6360_VMIVR_SHFT      (1)
#define MT6360_VMIVR_MASK      GENMASK(7, 1)
/* MT6360_PMU_CHG_CTRL7 */
#define MT6360_ICHG_SHFT	(2)
#define MT6360_ICHG_MASK	GENMASK(7, 2)
/* MT6360_PMU_CHG_CTRL8 */
#define MT6360_IPREC_SHFT	(0)
#define MT6360_IPREC_MASK	GENMASK(3, 0)
/* MT6360_PMU_CHG_CTRL9 */
#define MT6360_IEOC_SHFT	(4)
#define MT6360_IEOC_MASK	GENMASK(7, 4)
/* MT6360_PMU_CHG_CTRL10 */
#define MT6360_OTG_OC_MASK	GENMASK(3, 0)
/* MT6360_PMU_CHG_CTRL13 */
#define MT6360_CHG_WDT_EN_MASK	BIT(7)
/* MT6360_PMU_DEVICE_TYPE */
#define MT6360_USBCHGEN_MASK	BIT(7)
/* MT6360_PMU_USB_STATUS1 */
#define MT6360_USB_STATUS_SHFT	(4)
#define MT6360_USB_STATUS_MASK	GENMASK(6, 4)
/* MT6360_PMU_CHG_STAT */
#define MT6360_CHG_STAT_SHFT	(6)
#define MT6360_CHG_STAT_MASK	GENMASK(7, 6)
#define MT6360_VBAT_LVL_MASK	BIT(5)
/* MT6360_PMU_CHG_CTRL19 */
#define MT6360_VINOVP_SHFT	(5)
#define MT6360_VINOVP_MASK	GENMASK(6, 5)
/* MT6360_PMU_FOD_STAT */
#define MT6360_CHRDET_EXT_MASK	BIT(4)

/* uV */
#define MT6360_VMIVR_MIN	3900000
#define MT6360_VMIVR_MAX	13400000
#define MT6360_VMIVR_STEP	100000
/* uA */
#define MT6360_ICHG_MIN		100000
#define MT6360_ICHG_MAX		5000000
#define MT6360_ICHG_STEP	100000
/* uV */
#define MT6360_VOREG_MIN	3900000
#define MT6360_VOREG_MAX	4710000
#define MT6360_VOREG_STEP	10000
/* uA */
#define MT6360_AICR_MIN		100000
#define MT6360_AICR_MAX		3250000
#define MT6360_AICR_STEP	50000
/* uA */
#define MT6360_IPREC_MIN	100000
#define MT6360_IPREC_MAX	850000
#define MT6360_IPREC_STEP	50000
/* uA */
#define MT6360_IEOC_MIN		100000
#define MT6360_IEOC_MAX		850000
#define MT6360_IEOC_STEP	50000

/* Battery temperature: how often to check it while powered, and the hysteresis in millicelsius */
#define MT6360_BAT_TEMP_POLL_MS		10000
#define MT6360_BAT_TEMP_HYST		2000

/*
 * Battery temperature zones, from the monitored battery's operating range (no charging outside
 * it) and alert temperatures (its alert charge current and voltage beyond them).
 */
enum mt6360_bat_temp_zone {
	MT6360_BAT_TEMP_COLD,
	MT6360_BAT_TEMP_COOL,
	MT6360_BAT_TEMP_NORMAL,
	MT6360_BAT_TEMP_WARM,
	MT6360_BAT_TEMP_HOT,
};

enum {
	MT6360_RANGE_VMIVR,
	MT6360_RANGE_ICHG,
	MT6360_RANGE_VOREG,
	MT6360_RANGE_AICR,
	MT6360_RANGE_IPREC,
	MT6360_RANGE_IEOC,
	MT6360_RANGE_MAX,
};

static const struct linear_range mt6360_chg_range[MT6360_RANGE_MAX] = {
	LINEAR_RANGE_IDX(MT6360_RANGE_VMIVR, 3900000, 0, 0x5F, 100000),
	LINEAR_RANGE_IDX(MT6360_RANGE_ICHG, 100000, 0, 0x31, 100000),
	LINEAR_RANGE_IDX(MT6360_RANGE_VOREG, 3900000, 0, 0x51, 10000),
	LINEAR_RANGE_IDX(MT6360_RANGE_AICR, 100000, 0, 0x3F, 50000),
	LINEAR_RANGE_IDX(MT6360_RANGE_IPREC, 100000, 0, 0x0F, 50000),
	LINEAR_RANGE_IDX(MT6360_RANGE_IEOC, 100000, 0, 0x0F, 50000),
};

struct mt6360_chg_info {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply_desc psy_desc;
	struct power_supply *psy;
	struct regulator_dev *otg_rdev;
	struct mutex chgdet_lock;
	u32 vinovp;
	bool pwr_rdy;
	bool bc12_en;
	int psy_usb_type;
	struct work_struct chrdet_work;
	u32 ichg_max;
	u32 cv_max;
	/* Protects the charge current, voltage and enable against the temperature work */
	struct mutex chg_lock;
	u32 ichg_req;
	u32 cv_req;
	u32 ichg_tlim;
	u32 cv_tlim;
	struct power_supply_battery_info *bat_info;
	struct iio_channel *bat_temp;
	struct delayed_work bat_temp_work;
	enum mt6360_bat_temp_zone bat_temp_zone;
	bool bat_temp_known;
};

enum mt6360_iinlmtsel {
	MT6360_IINLMTSEL_AICR_3250 = 0,
	MT6360_IINLMTSEL_CHG_TYPE,
	MT6360_IINLMTSEL_AICR,
	MT6360_IINLMTSEL_LOWER_LEVEL,
};

enum mt6360_pmu_chg_type {
	MT6360_CHG_TYPE_NOVBUS = 0,
	MT6360_CHG_TYPE_UNDER_GOING,
	MT6360_CHG_TYPE_SDP,
	MT6360_CHG_TYPE_SDPNSTD,
	MT6360_CHG_TYPE_DCP,
	MT6360_CHG_TYPE_CDP,
	MT6360_CHG_TYPE_DISABLE_BC12,
	MT6360_CHG_TYPE_MAX,
};

static int mt6360_get_chrdet_ext_stat(struct mt6360_chg_info *mci,
					     bool *pwr_rdy)
{
	int ret;
	unsigned int regval;

	ret = regmap_read(mci->regmap, MT6360_PMU_FOD_STAT, &regval);
	if (ret < 0)
		return ret;
	*pwr_rdy = (regval & MT6360_CHRDET_EXT_MASK) ? true : false;
	return 0;
}

static int mt6360_charger_get_online(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	int ret;
	bool pwr_rdy;

	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret < 0)
		return ret;
	val->intval = pwr_rdy ? true : false;
	return 0;
}

static int mt6360_charger_get_status(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	int status, ret;
	unsigned int regval;
	bool pwr_rdy;

	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret < 0)
		return ret;
	if (!pwr_rdy) {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		goto out;
	}

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_STAT, &regval);
	if (ret < 0)
		return ret;
	regval &= MT6360_CHG_STAT_MASK;
	regval >>= MT6360_CHG_STAT_SHFT;
	switch (regval) {
	case 0x0:
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 0x1:
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 0x2:
		status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		ret = -EIO;
	}
out:
	if (!ret)
		val->intval = status;
	return ret;
}

static int mt6360_charger_get_charge_type(struct mt6360_chg_info *mci,
					  union power_supply_propval *val)
{
	int type, ret;
	unsigned int regval;
	u8 chg_stat;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_STAT, &regval);
	if (ret < 0)
		return ret;

	chg_stat = (regval & MT6360_CHG_STAT_MASK) >> MT6360_CHG_STAT_SHFT;
	switch (chg_stat) {
	case 0x01: /* Charge in Progress */
		if (regval & MT6360_VBAT_LVL_MASK)
			type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case 0x00: /* Not Charging */
	case 0x02: /* Charge Done */
	case 0x03: /* Charge Fault */
	default:
		type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	val->intval = type;
	return 0;
}

static int mt6360_charger_get_ichg(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL7, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_ICHG_MASK) >> MT6360_ICHG_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_ICHG], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_max_ichg(struct mt6360_chg_info *mci,
				       union power_supply_propval *val)
{
	val->intval = mci->ichg_max;
	return 0;
}

static int mt6360_charger_get_cv(struct mt6360_chg_info *mci,
				 union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL4, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_VOREG_MASK) >> MT6360_VOREG_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_VOREG], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_max_cv(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	val->intval = mci->cv_max;
	return 0;
}

static int mt6360_charger_get_aicr(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL3, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_IAICR_MASK) >> MT6360_IAICR_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_AICR], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_mivr(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL6, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_VMIVR_MASK) >> MT6360_VMIVR_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_VMIVR], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_iprechg(struct mt6360_chg_info *mci,
				      union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL8, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_IPREC_MASK) >> MT6360_IPREC_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_IPREC], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_ieoc(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL9, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_IEOC_MASK) >> MT6360_IEOC_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_IEOC], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_health(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	static const int health[] = {
		[MT6360_BAT_TEMP_COLD] = POWER_SUPPLY_HEALTH_COLD,
		[MT6360_BAT_TEMP_COOL] = POWER_SUPPLY_HEALTH_COOL,
		[MT6360_BAT_TEMP_NORMAL] = POWER_SUPPLY_HEALTH_GOOD,
		[MT6360_BAT_TEMP_WARM] = POWER_SUPPLY_HEALTH_WARM,
		[MT6360_BAT_TEMP_HOT] = POWER_SUPPLY_HEALTH_HOT,
	};

	guard(mutex)(&mci->chg_lock);
	if (mci->bat_temp && mci->bat_temp_known)
		val->intval = health[mci->bat_temp_zone];
	else
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	return 0;
}

static int mt6360_charger_set_online(struct mt6360_chg_info *mci,
				     const union power_supply_propval *val)
{
	u8 force_sleep = val->intval ? 0 : 1;

	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL1,
				  MT6360_FSLP_MASK,
				  force_sleep << MT6360_FSLP_SHFT);
}

static int mt6360_charger_write_ichg(struct mt6360_chg_info *mci, u32 uA)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_ICHG], uA, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL7,
				  MT6360_ICHG_MASK,
				  sel << MT6360_ICHG_SHFT);
}

/* Charge at what was asked for, within the battery's and its temperature's limits */
static int mt6360_charger_apply_ichg(struct mt6360_chg_info *mci)
{
	return mt6360_charger_write_ichg(mci, min3(mci->ichg_req, mci->ichg_max, mci->ichg_tlim));
}

static int mt6360_charger_apply_cv(struct mt6360_chg_info *mci)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_VOREG],
					 min3(mci->cv_req, mci->cv_max, mci->cv_tlim), &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL4,
				  MT6360_VOREG_MASK,
				  sel << MT6360_VOREG_SHFT);
}

static int mt6360_charger_set_ichg(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	guard(mutex)(&mci->chg_lock);
	mci->ichg_req = max(val->intval, 0);
	return mt6360_charger_apply_ichg(mci);
}

static int mt6360_charger_set_cv(struct mt6360_chg_info *mci,
				 const union power_supply_propval *val)
{
	guard(mutex)(&mci->chg_lock);
	mci->cv_req = max(val->intval, 0);
	return mt6360_charger_apply_cv(mci);
}

static int mt6360_charger_set_aicr(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_AICR], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL3,
				  MT6360_IAICR_MASK,
				  sel << MT6360_IAICR_SHFT);
}

static int mt6360_charger_set_mivr(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_VMIVR], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL6,
				  MT6360_VMIVR_MASK,
				  sel << MT6360_VMIVR_SHFT);
}

static int mt6360_charger_set_iprechg(struct mt6360_chg_info *mci,
				      const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_IPREC], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL8,
				  MT6360_IPREC_MASK,
				  sel << MT6360_IPREC_SHFT);
}

static int mt6360_charger_set_ieoc(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_IEOC], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL9,
				  MT6360_IEOC_MASK,
				  sel << MT6360_IEOC_SHFT);
}

static int mt6360_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mt6360_chg_info *mci = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6360_charger_get_online(mci, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = mt6360_charger_get_status(mci, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = mt6360_charger_get_health(mci, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = mt6360_charger_get_charge_type(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6360_charger_get_ichg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = mt6360_charger_get_max_ichg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6360_charger_get_cv(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = mt6360_charger_get_max_cv(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6360_charger_get_aicr(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = mt6360_charger_get_mivr(mci, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6360_charger_get_iprechg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6360_charger_get_ieoc(mci, val);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = mci->psy_usb_type;
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static int mt6360_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct mt6360_chg_info *mci = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6360_charger_set_online(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6360_charger_set_ichg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6360_charger_set_cv(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6360_charger_set_aicr(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = mt6360_charger_set_mivr(mci, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6360_charger_set_iprechg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6360_charger_set_ieoc(mci, val);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int mt6360_charger_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property mt6360_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc mt6360_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= mt6360_charger_properties,
	.num_properties		= ARRAY_SIZE(mt6360_charger_properties),
	.get_property		= mt6360_charger_get_property,
	.set_property		= mt6360_charger_set_property,
	.property_is_writeable	= mt6360_charger_property_is_writeable,
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_SDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
				  BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
};

static const struct regulator_ops mt6360_chg_otg_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static const struct regulator_desc mt6360_otg_rdesc = {
	.of_match = "usb-otg-vbus-regulator",
	.name = "usb-otg-vbus",
	.ops = &mt6360_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4425000,
	.uV_step = 25000,
	.n_voltages = 57,
	.vsel_reg = MT6360_PMU_CHG_CTRL5,
	.vsel_mask = MT6360_VOBST_MASK,
	.enable_reg = MT6360_PMU_CHG_CTRL1,
	.enable_mask = MT6360_OPA_MODE_MASK,
};

static irqreturn_t mt6360_pmu_attach_i_handler(int irq, void *data)
{
	struct mt6360_chg_info *mci = data;
	int ret;
	unsigned int usb_status;
	int last_usb_type;

	mutex_lock(&mci->chgdet_lock);
	if (!mci->bc12_en) {
		dev_warn(mci->dev, "Received attach interrupt, bc12 disabled, ignore irq\n");
		goto out;
	}
	last_usb_type = mci->psy_usb_type;
	/* Plug in */
	ret = regmap_read(mci->regmap, MT6360_PMU_USB_STATUS1, &usb_status);
	if (ret < 0)
		goto out;
	usb_status &= MT6360_USB_STATUS_MASK;
	usb_status >>= MT6360_USB_STATUS_SHFT;
	switch (usb_status) {
	case MT6360_CHG_TYPE_NOVBUS:
		dev_dbg(mci->dev, "Received attach interrupt, no vbus\n");
		goto out;
	case MT6360_CHG_TYPE_UNDER_GOING:
		dev_dbg(mci->dev, "Received attach interrupt, under going...\n");
		goto out;
	case MT6360_CHG_TYPE_SDP:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6360_CHG_TYPE_SDPNSTD:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6360_CHG_TYPE_CDP:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case MT6360_CHG_TYPE_DCP:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	case MT6360_CHG_TYPE_DISABLE_BC12:
		dev_dbg(mci->dev, "Received attach interrupt, bc12 detect not enable\n");
		goto out;
	default:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		dev_dbg(mci->dev, "Received attach interrupt, reserved address\n");
		goto out;
	}

	dev_dbg(mci->dev, "Received attach interrupt, chg_type = %d\n", mci->psy_usb_type);
	if (last_usb_type != mci->psy_usb_type)
		power_supply_changed(mci->psy);
out:
	mutex_unlock(&mci->chgdet_lock);
	return IRQ_HANDLED;
}

static void mt6360_handle_chrdet_ext_evt(struct mt6360_chg_info *mci)
{
	int ret;
	bool pwr_rdy;

	mutex_lock(&mci->chgdet_lock);
	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret < 0)
		goto out;
	if (mci->pwr_rdy == pwr_rdy) {
		dev_dbg(mci->dev, "Received vbus interrupt, pwr_rdy is same(%d)\n", pwr_rdy);
		goto out;
	}
	mci->pwr_rdy = pwr_rdy;
	dev_dbg(mci->dev, "Received vbus interrupt, pwr_rdy = %d\n", pwr_rdy);
	if (pwr_rdy && mci->bat_temp)
		mod_delayed_work(system_wq, &mci->bat_temp_work, 0);
	if (!pwr_rdy) {
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		power_supply_changed(mci->psy);

	}
	ret = regmap_update_bits(mci->regmap,
				 MT6360_PMU_DEVICE_TYPE,
				 MT6360_USBCHGEN_MASK,
				 pwr_rdy ? MT6360_USBCHGEN_MASK : 0);
	if (ret < 0)
		goto out;
	mci->bc12_en = pwr_rdy;
out:
	mutex_unlock(&mci->chgdet_lock);
}

static void mt6360_chrdet_work(struct work_struct *work)
{
	struct mt6360_chg_info *mci = (struct mt6360_chg_info *)container_of(
				     work, struct mt6360_chg_info, chrdet_work);

	mt6360_handle_chrdet_ext_evt(mci);
}

static irqreturn_t mt6360_pmu_chrdet_ext_evt_handler(int irq, void *data)
{
	struct mt6360_chg_info *mci = data;

	mt6360_handle_chrdet_ext_evt(mci);
	return IRQ_HANDLED;
}

static int mt6360_chg_irq_register(struct platform_device *pdev)
{
	const struct {
		const char *name;
		irq_handler_t handler;
	} irq_descs[] = {
		{ "attach_i", mt6360_pmu_attach_i_handler },
		{ "chrdet_ext_evt", mt6360_pmu_chrdet_ext_evt_handler }
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(irq_descs); i++) {
		ret = platform_get_irq_byname(pdev, irq_descs[i].name);
		if (ret < 0)
			return ret;

		ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
						irq_descs[i].handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						irq_descs[i].name,
						platform_get_drvdata(pdev));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static u32 mt6360_vinovp_trans_to_sel(u32 val)
{
	u32 vinovp_tbl[] = { 5500000, 6500000, 11000000, 14500000 };
	int i;

	/* Select the smaller and equal supported value */
	for (i = 0; i < ARRAY_SIZE(vinovp_tbl)-1; i++) {
		if (val < vinovp_tbl[i+1])
			break;
	}
	return i;
}

static int mt6360_chg_init_setting(struct mt6360_chg_info *mci)
{
	int ret;
	u32 sel;

	sel = mt6360_vinovp_trans_to_sel(mci->vinovp);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL19,
				  MT6360_VINOVP_MASK, sel << MT6360_VINOVP_SHFT);
	if (ret)
		return dev_err_probe(mci->dev, ret, "%s: Failed to apply vinovp\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_DEVICE_TYPE,
				 MT6360_USBCHGEN_MASK, 0);
	if (ret)
		return dev_err_probe(mci->dev, ret, "%s: Failed to disable bc12\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
				 MT6360_IINLMTSEL_MASK,
				 MT6360_IINLMTSEL_AICR <<
					MT6360_IINLMTSEL_SHFT);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to switch iinlmtsel to aicr\n", __func__);
	usleep_range(5000, 6000);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL3,
				 MT6360_ILIM_EN_MASK, 0);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to disable ilim\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL10,
				 MT6360_OTG_OC_MASK, MT6360_OTG_OC_MASK);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to config otg oc to 3A\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL13,
				 MT6360_CHG_WDT_EN_MASK, 0);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to disable charger watchdog\n", __func__);
	return 0;
}

static enum mt6360_bat_temp_zone
mt6360_bat_temp_to_zone(const struct power_supply_battery_info *info, int temp)
{
	/* The battery gives its temperatures in degrees Celsius */
	if (info->temp_min > INT_MIN && temp < info->temp_min * 1000)
		return MT6360_BAT_TEMP_COLD;
	if (info->temp_alert_min > INT_MIN && temp < info->temp_alert_min * 1000)
		return MT6360_BAT_TEMP_COOL;
	if (info->temp_max < INT_MAX && temp > info->temp_max * 1000)
		return MT6360_BAT_TEMP_HOT;
	if (info->temp_alert_max < INT_MAX && temp > info->temp_alert_max * 1000)
		return MT6360_BAT_TEMP_WARM;
	return MT6360_BAT_TEMP_NORMAL;
}

/* How far a zone is from normal */
static int mt6360_bat_temp_distance(enum mt6360_bat_temp_zone zone)
{
	return abs((int)zone - MT6360_BAT_TEMP_NORMAL);
}

/* Leave a zone for one closer to normal only once past its boundary by the hysteresis */
static enum mt6360_bat_temp_zone mt6360_bat_temp_zone(struct mt6360_chg_info *mci, int temp)
{
	enum mt6360_bat_temp_zone cur = mci->bat_temp_zone;
	enum mt6360_bat_temp_zone zone = mt6360_bat_temp_to_zone(mci->bat_info, temp);

	if (!mci->bat_temp_known ||
	    mt6360_bat_temp_distance(zone) >= mt6360_bat_temp_distance(cur))
		return zone;

	temp += cur < MT6360_BAT_TEMP_NORMAL ? -MT6360_BAT_TEMP_HYST : MT6360_BAT_TEMP_HYST;
	zone = mt6360_bat_temp_to_zone(mci->bat_info, temp);
	if (mt6360_bat_temp_distance(zone) >= mt6360_bat_temp_distance(cur))
		return cur;
	return zone;
}

/*
 * Stop charging. As the stock kernel does against VSYS overshoot, take the charge current down to
 * 500 mA first, giving it 2 ms for each 50 mA to ramp.
 */
static int mt6360_charger_disable(struct mt6360_chg_info *mci)
{
	union power_supply_propval ichg;
	unsigned int regval;
	int ret;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL2, &regval);
	if (ret || !(regval & MT6360_CHG_EN_MASK))
		return ret;

	ret = mt6360_charger_get_ichg(mci, &ichg);
	if (ret)
		return ret;
	if (ichg.intval > 500000) {
		ret = mt6360_charger_write_ichg(mci, 500000);
		if (ret)
			return ret;
		msleep((ichg.intval - 500000) / 50000 * 2);
	}

	return regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2, MT6360_CHG_EN_MASK, 0);
}

/* Charge as the battery temperature allows; called with chg_lock held */
static int mt6360_charger_apply_bat_temp(struct mt6360_chg_info *mci)
{
	const struct power_supply_battery_info *info = mci->bat_info;
	int ret;

	mci->ichg_tlim = MT6360_ICHG_MAX;
	mci->cv_tlim = MT6360_VOREG_MAX;

	if (!mci->bat_temp_known)
		return mt6360_charger_disable(mci);

	switch (mci->bat_temp_zone) {
	case MT6360_BAT_TEMP_COLD:
	case MT6360_BAT_TEMP_HOT:
		return mt6360_charger_disable(mci);
	case MT6360_BAT_TEMP_COOL:
		if (info->alert_low_temp_charge_current_ua > 0)
			mci->ichg_tlim = info->alert_low_temp_charge_current_ua;
		if (info->alert_low_temp_charge_voltage_uv > 0)
			mci->cv_tlim = info->alert_low_temp_charge_voltage_uv;
		break;
	case MT6360_BAT_TEMP_WARM:
		if (info->alert_high_temp_charge_current_ua > 0)
			mci->ichg_tlim = info->alert_high_temp_charge_current_ua;
		if (info->alert_high_temp_charge_voltage_uv > 0)
			mci->cv_tlim = info->alert_high_temp_charge_voltage_uv;
		break;
	default:
		break;
	}

	ret = mt6360_charger_apply_cv(mci);
	if (!ret)
		ret = mt6360_charger_apply_ichg(mci);
	if (!ret)
		ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
					 MT6360_CHG_EN_MASK, MT6360_CHG_EN_MASK);
	return ret;
}

static void mt6360_bat_temp_work(struct work_struct *work)
{
	struct mt6360_chg_info *mci = container_of(to_delayed_work(work),
						   struct mt6360_chg_info, bat_temp_work);
	static const char * const names[] = {
		[MT6360_BAT_TEMP_COLD] = "cold, not charging",
		[MT6360_BAT_TEMP_COOL] = "cool",
		[MT6360_BAT_TEMP_NORMAL] = "normal",
		[MT6360_BAT_TEMP_WARM] = "warm",
		[MT6360_BAT_TEMP_HOT] = "hot, not charging",
	};
	enum mt6360_bat_temp_zone zone = MT6360_BAT_TEMP_NORMAL;
	bool known, changed = false, pwr_rdy;
	int temp = 0, ret;

	ret = iio_read_channel_processed(mci->bat_temp, &temp);
	known = ret >= 0;
	if (!known)
		dev_err_ratelimited(mci->dev, "Failed to read the battery temperature: %d\n", ret);

	scoped_guard(mutex, &mci->chg_lock) {
		if (known)
			zone = mt6360_bat_temp_zone(mci, temp);
		changed = known != mci->bat_temp_known || zone != mci->bat_temp_zone;
		if (changed) {
			mci->bat_temp_known = known;
			mci->bat_temp_zone = zone;
			if (known)
				dev_info(mci->dev, "Battery at %d mC: %s\n", temp, names[zone]);
			ret = mt6360_charger_apply_bat_temp(mci);
			if (ret)
				dev_err(mci->dev, "Failed to follow the battery temperature: %d\n",
					ret);
		}
	}

	if (changed)
		power_supply_changed(mci->psy);

	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret || pwr_rdy)
		queue_delayed_work(system_wq, &mci->bat_temp_work,
				   msecs_to_jiffies(MT6360_BAT_TEMP_POLL_MS));
}

/*
 * Charge the monitored battery as it asks: at most its constant charge current and voltage,
 * which also become the most userspace may set, with its precharge current, and ending at its
 * termination current.
 */
static int mt6360_chg_apply_battery_info(struct mt6360_chg_info *mci)
{
	struct power_supply_battery_info *info;
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_battery_info(mci->psy, &info);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return dev_err_probe(mci->dev, ret, "Failed to get battery info\n");

	mci->bat_info = info;

	if (info->constant_charge_voltage_max_uv > 0) {
		mci->cv_max = min(info->constant_charge_voltage_max_uv, MT6360_VOREG_MAX);
		val.intval = mci->cv_max;
		ret = mt6360_charger_set_cv(mci, &val);
		if (ret)
			return dev_err_probe(mci->dev, ret, "Failed to set charge voltage\n");
	}

	if (info->constant_charge_current_max_ua > 0) {
		mci->ichg_max = min(info->constant_charge_current_max_ua, MT6360_ICHG_MAX);
		val.intval = mci->ichg_max;
		ret = mt6360_charger_set_ichg(mci, &val);
		if (ret)
			return dev_err_probe(mci->dev, ret, "Failed to set charge current\n");
	}

	if (info->precharge_current_ua > 0) {
		val.intval = info->precharge_current_ua;
		ret = mt6360_charger_set_iprechg(mci, &val);
		if (ret)
			return dev_err_probe(mci->dev, ret, "Failed to set precharge current\n");
	}

	if (info->charge_term_current_ua > 0) {
		val.intval = info->charge_term_current_ua;
		ret = mt6360_charger_set_ieoc(mci, &val);
		if (!ret)
			ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
						 MT6360_TE_EN_MASK, MT6360_TE_EN_MASK);
		if (ret)
			return dev_err_probe(mci->dev, ret,
					     "Failed to set up termination\n");
	}

	return 0;
}

static int mt6360_charger_probe(struct platform_device *pdev)
{
	struct mt6360_chg_info *mci;
	struct power_supply_config charger_cfg = {};
	struct regulator_config config = { };
	int ret;

	mci = devm_kzalloc_obj(&pdev->dev, *mci);
	if (!mci)
		return -ENOMEM;

	mci->dev = &pdev->dev;
	mci->vinovp = 6500000;
	mci->ichg_max = MT6360_ICHG_MAX;
	mci->cv_max = MT6360_VOREG_MAX;
	mci->ichg_req = MT6360_ICHG_MAX;
	mci->ichg_tlim = MT6360_ICHG_MAX;
	mci->cv_req = MT6360_VOREG_MAX;
	mci->cv_tlim = MT6360_VOREG_MAX;
	mutex_init(&mci->chgdet_lock);
	ret = devm_mutex_init(&pdev->dev, &mci->chg_lock);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, mci);
	ret = devm_work_autocancel(&pdev->dev, &mci->chrdet_work, mt6360_chrdet_work);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to set delayed work\n");

	ret = device_property_read_u32(&pdev->dev, "richtek,vinovp-microvolt", &mci->vinovp);
	if (ret)
		dev_warn(&pdev->dev, "Failed to parse vinovp in DT, keep default 6.5v\n");

	mci->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!mci->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "Failed to get parent regmap\n");

	mci->bat_temp = devm_iio_channel_get(&pdev->dev, "battery-temp");
	if (IS_ERR(mci->bat_temp)) {
		ret = PTR_ERR(mci->bat_temp);
		if (ret != -ENODEV)
			return dev_err_probe(&pdev->dev, ret,
					     "Failed to get the battery temperature\n");
		mci->bat_temp = NULL;
	}

	ret = mt6360_chg_init_setting(mci);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to initial setting\n");

	memcpy(&mci->psy_desc, &mt6360_charger_desc, sizeof(mci->psy_desc));
	mci->psy_desc.name = dev_name(&pdev->dev);
	charger_cfg.drv_data = mci;
	charger_cfg.fwnode = dev_fwnode(&pdev->dev);
	mci->psy = devm_power_supply_register(&pdev->dev,
					      &mci->psy_desc, &charger_cfg);
	if (IS_ERR(mci->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(mci->psy),
				     "Failed to register power supply dev\n");

	ret = mt6360_chg_apply_battery_info(mci);
	if (ret)
		return ret;

	/* Without a battery to take the limits from, there is nothing to follow */
	if (!mci->bat_info)
		mci->bat_temp = NULL;

	if (mci->bat_temp) {
		ret = devm_delayed_work_autocancel(&pdev->dev, &mci->bat_temp_work,
						   mt6360_bat_temp_work);
		if (ret)
			return ret;
	}

	ret = mt6360_chg_irq_register(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register irqs\n");

	config.dev = &pdev->dev;
	config.regmap = mci->regmap;
	mci->otg_rdev = devm_regulator_register(&pdev->dev, &mt6360_otg_rdesc,
						&config);
	if (IS_ERR(mci->otg_rdev))
		return PTR_ERR(mci->otg_rdev);

	schedule_work(&mci->chrdet_work);
	if (mci->bat_temp)
		queue_delayed_work(system_wq, &mci->bat_temp_work, 0);

	return 0;
}

static const struct of_device_id __maybe_unused mt6360_charger_of_id[] = {
	{ .compatible = "mediatek,mt6360-chg", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6360_charger_of_id);

static const struct platform_device_id mt6360_charger_id[] = {
	{ .name = "mt6360-chg" },
	{ }
};
MODULE_DEVICE_TABLE(platform, mt6360_charger_id);

static struct platform_driver mt6360_charger_driver = {
	.driver = {
		.name = "mt6360-chg",
		.of_match_table = mt6360_charger_of_id,
	},
	.probe = mt6360_charger_probe,
	.id_table = mt6360_charger_id,
};
module_platform_driver(mt6360_charger_driver);

MODULE_AUTHOR("Gene Chen <gene_chen@richtek.com>");
MODULE_DESCRIPTION("MT6360 Charger Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
