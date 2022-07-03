// SPDX-License-Identifier: GPL-2.0-only
/*
 * Universal power supply monitor class
 *
 * Copyright © 2022 Linaro Ltd.
 * Author: Caleb Connolly
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/thermal.h>
#include <linux/fixp-arith.h>
#include "power_supply.h"

struct power_supply_cooling {
	struct power_supply *psy;
	unsigned int step;
};

static int power_supply_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return ret;

	/* Convert tenths of degree Celsius to milli degree Celsius. */
	*temp = val.intval * 100;

	return ret;
}

static struct thermal_zone_device_ops psy_tzd_ops = {
	.get_temp = power_supply_read_temp,
};

static int psy_register_thermal(struct power_supply *psy)
{
	int ret;

	if (psy->desc->no_thermal)
		return 0;

	/* Register battery zone device psy reports temperature */
	if (power_supply_has_property(psy->desc, POWER_SUPPLY_PROP_TEMP)) {
		psy->tzd = thermal_zone_device_register(psy->desc->name,
				0, 0, psy, &psy_tzd_ops, NULL, 0, 0);
		if (IS_ERR(psy->tzd))
			return PTR_ERR(psy->tzd);
		ret = thermal_zone_device_enable(psy->tzd);
		if (ret)
			thermal_zone_device_unregister(psy->tzd);
		return ret;
	}

	return 0;
}

static void psy_unregister_thermal(struct power_supply *psy)
{
	if (IS_ERR_OR_NULL(psy->tzd))
		return;
	thermal_zone_device_unregister(psy->tzd);
}

#define CHARGE_CNTL_STEP 500000
/* 2.5W or 500mA @ 5v */
#define POWER_STEP 25000

static int psy_get_current_now(struct power_supply *psy)
{
	int ret;

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	return -ENXIO;
}

static int psy_get_voltage_now(struct power_supply *psy)
{
	int ret;

	if (!vlim && power_supply_has_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val)
		if (ret)
			return ret;
		return val.intval;
	}

	if (!vlim && power_supply_has_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	return -ENXIO;
}

/* in milliwatts! */
static int psy_get_power_now(struct power_supply *psy, int *current,
			     int *voltage)
{
	int ret, current_now, voltage_now, power_now;

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_INPUT_POWER_LIMIT)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_INPUT_POWER_LIMIT, &val)
		if (ret)
			return ret;

		return val.intval / 1000;
	}

	voltage_now = psy_get_voltage_now(psy);
	current_now = psy_get_current_now(psy);

	if (current_now < 0)
		return current_now;

	power_now = current_now / 1000;

	if (voltage_now > 0) {
		power_now *= voltage_now / 1000;
	}

	if (current)
		*current = current_now;
	
	if (voltage)
		*voltage = voltage_now;

	return power_now;
}

static int psy_get_max_current(struct power_supply *psy)
{
	int ret;
	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	return -ENXIO;
}

static int psy_get_max_voltage(struct power_supply *psy)
{
	int ret;

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	if (!vlim && power_supply_has_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX, &val)
		if (ret)
			return ret;

		return val.intval;
	}

	/*
	 * This isn't ideal, but chances are devices that don't export a max voltage
	 * But do export the instaneous voltage are only designed to charge at one
	 * voltage.
	 */
	if (!vlim && power_supply_has_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val)
		if (ret)
			return ret;
		return val.intval;
	}

	return -ENXIO;
}

/* in milliwatts! */
static int psy_get_max_power(struct power_supply *psy, int *current,
			     int *voltage)
{
	int ret, current_max, voltage_max, power_max;

	if (power_supply_has_property(psy, POWER_SUPPLY_PROP_INPUT_POWER_LIMIT)) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_INPUT_POWER_LIMIT, &val)
		if (ret)
			return ret;

		return val.intval / 1000;
	}

	voltage_max = psy_get_max_voltage(psy);
	current_max = psy_get_max_current(psy);

	if (current_max < 0)
		return current_max;

	power_max = current_max / 1000;

	if (voltage_max > 0) {
		power_max *= voltage_max / 1000;
	}

	if (current)
		*current = current_now;
	
	if (voltage)
		*voltage = voltage_now;

	return power_max;
}

/* thermal cooling device callbacks */
static int ps_get_max_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct power_supply *psy;
	int ret, power_max;
	int step = CHARGE_CNTL_STEP;

	psy = tcd->devdata;

	power_max = psy_get_max_power(psy, NULL, NULL);

	*state = power_max / POWER_STEP;

	return ret;
}

static int ps_get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;
	unsigned long *max;

	ps_get_max_charge_cntl_limit(tcd, &max);

	psy = tcd->devdata;
	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
	if (ret)
		return ret;

	*state = val.intval;

	return ret;
}

static int ps_set_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = tcd->devdata;
	val.intval = state;
	ret = psy->desc->set_property(psy,
		POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);

	return ret;
}

static const struct thermal_cooling_device_ops psy_tcd_ops = {
	.get_max_state = ps_get_max_charge_cntl_limit,
	.get_cur_state = ps_get_cur_charge_cntl_limit,
	.set_cur_state = ps_set_cur_charge_cntl_limit,
};

static int psy_register_cooler(struct power_supply *psy)
{
	/* Register for cooling device if psy can control charging */
	if (power_supply_has_property(psy->desc, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT)
	 || power_supply_property_is_writeable(psy, POWER_SUPPLY_PROP_CURRENT_MAX)) {
		psy->tcd = thermal_cooling_device_register(
			(char *)psy->desc->name,
			psy, &psy_tcd_ops);
		return PTR_ERR_OR_ZERO(psy->tcd);
	}

	return 0;
}

static void psy_unregister_cooler(struct power_supply *psy)
{
	if (IS_ERR_OR_NULL(psy->tcd))
		return;
	thermal_cooling_device_unregister(psy->tcd);
}
