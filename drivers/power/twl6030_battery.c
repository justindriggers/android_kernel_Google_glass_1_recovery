/*
 * linux/drivers/power/twl6030_battery.c
 *
 * Based on: OMAP4:TWL6030 battery driver for Linux
 *
 * Copyright (C) 2012 Google, Inc.
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <asm/div64.h>

#define FG_REG_00	0x00
#define FG_REG_01	0x01
#define FG_REG_02	0x02
#define FG_REG_03	0x03
#define FG_REG_04	0x04
#define FG_REG_05	0x05
#define FG_REG_06	0x06
#define FG_REG_07	0x07
#define FG_REG_08	0x08
#define FG_REG_09	0x09
#define FG_REG_10	0x0A
#define FG_REG_11	0x0B

/* FG_REG_00 */
#define CC_ACTIVE_MODE_SHIFT	6
#define CC_AUTOCLEAR		(1 << 2)
#define CC_CAL_EN		(1 << 1)
#define CC_PAUSE		(1 << 0)

#define REG_TOGGLE1		0x90
#define FGDITHS			(1 << 7)
#define FGDITHR			(1 << 6)
#define FGS			(1 << 5)
#define FGR			(1 << 4)

/* TWL6030_GPADC_CTRL */
#define GPADC_CTRL_TEMP1_EN	(1 << 0)    /* input ch 1 */
#define GPADC_CTRL_TEMP2_EN	(1 << 1)    /* input ch 4 */
#define GPADC_CTRL_SCALER_EN	(1 << 2)    /* input ch 2 */
#define GPADC_CTRL_SCALER_DIV4	(1 << 3)
#define GPADC_CTRL_SCALER_EN_CH11	(1 << 4)    /* input ch 11 */
#define GPADC_CTRL_TEMP1_EN_MONITOR	(1 << 5)
#define GPADC_CTRL_TEMP2_EN_MONITOR	(1 << 6)
#define GPADC_CTRL_ISOURCE_EN		(1 << 7)

/* TWL6030/6032 BATTERY VOLTAGE GPADC CHANNELS */
#define TWL6030_GPADC_VBAT_CHNL	0x07

#define REG_MISC1		0xE4
#define VAC_MEAS		0x04
#define VBAT_MEAS		0x02
#define BB_MEAS			0x01

#define REG_USB_VBUS_CTRL_SET	0x04
#define VBUS_MEAS		0x01
#define REG_USB_ID_CTRL_SET	0x06
#define ID_MEAS			0x01

#define BBSPOR_CFG		0xE6
#define	BB_CHG_EN		(1 << 3)

/* sign extension needs a little care */
static __inline int sign_extend(int n, int num_bits)
{
	int shift = (int)(sizeof(int) * 8 - num_bits);
	return (n << shift) >> shift;
}

/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};

static struct wake_lock battery_wake_lock;

#define STATE_BATTERY   0
#define STATE_FULL      1

static const char *twl6030_state[] = {
	"BATTERY", "FULL",
};

/* change the order, not the length to keep this a power of 2 */
#define VOLTAGE_HISTORY_ORDER 3
#define VOLTAGE_HISTORY_LENGTH (1<<VOLTAGE_HISTORY_ORDER)

struct twl6030_battery_device_info {
	struct device		*dev;

	int voltage_mV;
	int voltage_history[VOLTAGE_HISTORY_LENGTH];
	int voltage_index;
	int current_uA;
	int current_avg_uA;
	int temp_C;
	int bat_health;
	int state;
	int vbus_online;

	unsigned long vbat_jiffies;
	unsigned long full_jiffies;
	unsigned long monitoring_interval_jiffies;

	int timer_n2;
	int timer_n1;
	s32 charge_n1;
	s32 charge_n2;
	s16 cc_offset;

	s64 capacity_offset;
	int capacity_uAh;
	int capacity_max_uAh;
	int trust_capacity;

	u8 gpadc_vbat_chnl;
	unsigned int min_vbus;

	struct twl4030_battery_platform_data *platform_data;

	unsigned int capacity;

	struct power_supply bat;
	struct power_supply usb;

	struct otg_transceiver *otg;
	struct notifier_block nb;

	struct work_struct calibration_work;
	struct delayed_work	monitor_work;

	struct workqueue_struct	*wq;
};

static int twl6030_get_gpadc_conversion(struct twl6030_battery_device_info *di,
		int channel_no)
{
	struct twl6030_gpadc_request req;
	int temp = 0;
	int ret;

	req.channels = (1 << channel_no);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0)
		return ret;

	if (req.rbuf[channel_no] > 0)
		temp = req.rbuf[channel_no];

	return temp;
}

static int is_battery_present(struct twl6030_battery_device_info *di)
{
	/* TODO */
	return 1;
}

static void twl6030_battery_current(struct twl6030_battery_device_info *di)
{
	int ret = 0;
	u16 read_value = 0;
	s16 temp = 0;
	int current_now = 0;

	/* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
								FG_REG_10, 2);
	if (ret < 0) {
		dev_dbg(di->dev, "failed to read FG_REG_10: current_now\n");
		return;
	}

	temp = sign_extend(read_value, 14);
	current_now = temp - di->cc_offset;

	/* current drawn per sec */
	current_now = current_now * fuelgauge_rate[0 /*di->fuelgauge_mode*/];
	/* current in mAmperes */
	current_now = (current_now * 3000) >> 14;
	/* current in uAmperes */
	current_now = current_now * 1000;
	di->current_uA = current_now;

	return;
}

static int twl6030_backupbatt_setup(void)
{
	int ret;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);
	if (ret)
		return ret;

	rd_reg |= BB_CHG_EN;
	ret = twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);

	return ret;
}

static int twl6030_battery_temp_setup(bool enable)
{
	int ret;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL_MODULE_MADC, &rd_reg, TWL6030_GPADC_CTRL);
	if (ret)
		return ret;

	if (enable)
		rd_reg |= (GPADC_CTRL_TEMP1_EN | GPADC_CTRL_TEMP2_EN |
			GPADC_CTRL_TEMP1_EN_MONITOR |
			GPADC_CTRL_TEMP2_EN_MONITOR | GPADC_CTRL_SCALER_DIV4);
	else
		rd_reg ^= (GPADC_CTRL_TEMP1_EN | GPADC_CTRL_TEMP2_EN |
			GPADC_CTRL_TEMP1_EN_MONITOR |
			GPADC_CTRL_TEMP2_EN_MONITOR | GPADC_CTRL_SCALER_DIV4);

	ret = twl_i2c_write_u8(TWL_MODULE_MADC, rd_reg, TWL6030_GPADC_CTRL);

	return ret;
}

static int twl6030_battery_voltage_setup(struct twl6030_battery_device_info *di)
{
	int ret;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, REG_MISC1);
	if (ret)
		return ret;

	rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS | BB_MEAS;
	ret = twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, REG_MISC1);
	if (ret)
		return ret;

	ret = twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_VBUS_CTRL_SET);
	if (ret)
		return ret;

	rd_reg = rd_reg | VBUS_MEAS;
	ret = twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_VBUS_CTRL_SET);
	if (ret)
		return ret;

	ret = twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_ID_CTRL_SET);
	if (ret)
		return ret;

	rd_reg = rd_reg | ID_MEAS;
	ret = twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_ID_CTRL_SET);
	if (ret)
		return ret;

	return ret;
}

static int twl6030_battery_current_setup(bool enable)
{
	int ret = 0;
	u8  reg = 0;

	/*
	 * Writing 0 to REG_TOGGLE1 has no effect, so
	 * can directly set/reset FG.
	 */
	if (enable)
		reg = FGDITHS | FGS;
	else
		reg = FGDITHR | FGR;

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, reg, REG_TOGGLE1);
	if (ret)
		return ret;

	return ret;
}

static enum power_supply_property twl6030_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int twl6030_calibrate_fuelgauge(struct twl6030_battery_device_info *di)
{
	s16 cc_offset;
	int n, ret;
	u8 reg;

	ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_AUTOCLEAR, FG_REG_00);
	if (ret)
		goto err;

	/* ensure autoclear completes before we calibrate */
	msleep(300);

	ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);
	if (ret)
		goto err;

	/* TODO: use IRQ to detect complete, plus timeout in case of failure */
	for (n = 0; n < 25; n++) {
		ret = twl_i2c_read_u8(TWL6030_MODULE_GASGAUGE, &reg, FG_REG_00);
		if (ret)
			goto err;
		if ((reg & CC_CAL_EN) == 0) break;
		msleep(100);
	}

	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &cc_offset, FG_REG_08, 2);
	if (ret < 0)
		goto err;

	cc_offset = sign_extend(cc_offset, 10);
	di->cc_offset = cc_offset;

	printk("battery: calibration took %d ms, offset = %d\n", n * 100, cc_offset);

	di->charge_n1 = 0;
	di->timer_n1 = 0;
	return 0;

err:
	return ret;
}

static int twl6030_estimate_capacity(struct twl6030_battery_device_info *di)
{
	if (di->voltage_mV < 3200) return 0;
	if (di->voltage_mV < 3500) return 5;
	if (di->voltage_mV < 3600) return 20;
	if (di->voltage_mV < 3700) return 50;
	if (di->voltage_mV < 3800) return 75;
	if (di->voltage_mV < 3900) return 90;
	return 95;
}

static int twl6030_get_battery_voltage(struct twl6030_battery_device_info *di)
{
	int v;

	v = twl6030_get_gpadc_conversion(di, di->gpadc_vbat_chnl);

	if (v <= 0)
		return di->voltage_mV;
	else
		return v;
}

static int twl6030_read_gasguage_regs(struct twl6030_battery_device_info *di)
{
	int ret;
	u32 data[2];

	/* must pause updates while reading multibyte registers to avoid bogus data */
	ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_PAUSE, FG_REG_00);
	if (ret < 0) {
		pr_err("twl6030: cannot pause gasgauge\n");
		goto done;
	}

	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, ((u8*) data) + 1, FG_REG_01, 7);
	if (ret < 0) {
		pr_err("twl6030: cannot read gasgauge\n");
		goto err;
	}

	di->timer_n1 = data[0] >> 8;	/* FG_REG_{01..03} is 24 bit unsigned sample counter */
	di->charge_n1 = data[1];	/* FG_REG_{04..07} is 32 bit signed accumulator */

err:
	ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, 0, FG_REG_00);
	if (ret < 0) {
		pr_err("twl6030: cannot resume gasgauge\n");
	}

done:
	return ret;
}

static void twl6030_update_voltage(struct twl6030_battery_device_info *di)
{
	int i;
	int index, q;
	long long total, denom;
	int curr_voltage = 0;

	curr_voltage = twl6030_get_gpadc_conversion(di, di->gpadc_vbat_chnl);
	if (curr_voltage <= 0)
		return;

	/*
	 * store the measured voltage in the history table. the index points
	 * to the most recent sample.
	 */
	di->voltage_index = (di->voltage_index + 1) & (VOLTAGE_HISTORY_LENGTH - 1);
	di->voltage_history[di->voltage_index] = curr_voltage;

	/* filter the cached voltage using a weighted average */
	q = VOLTAGE_HISTORY_LENGTH; // we need this many bits in the fraction

	index = di->voltage_index;
	total = 0;
	denom = 0;

	for (i=0; i < VOLTAGE_HISTORY_LENGTH; i++) {
		total += (long long) di->voltage_history[index] << (q - i); // convert to q, divide by 2^i
		denom += 1LL << (q - i); // denom += 1/(2^i), in q format

		index = (index + 1) & (VOLTAGE_HISTORY_LENGTH - 1);

		printk("battery voltage history[%d] = %d %s\n", i, di->voltage_history[i], i == di->voltage_index ? "*" : "");
	}

	/* divide by the sum of the weights to get the weighted average */
	total <<= q;
	total += denom >> 1; // round up mid value
	do_div(total, denom);

	/* convert back from q format and store value */
	di->voltage_mV = total >> q;
}

static void twl6030_read_fuelgauge(struct twl6030_battery_device_info *di)
{
	s32 samples;
	int ret, newcap;
	s64 cap, cur;
	u64 tmp;
	int statechanged = 0;

	/* save last reading before taking a new reading */
	di->charge_n2 = di->charge_n1;
	di->timer_n2 = di->timer_n1;

	/* update timer_n1 and charge_n1 */
	ret = twl6030_read_gasguage_regs(di);
	if (ret < 0)
		goto err;

	samples = di->timer_n1 - di->timer_n2;

	/* check for timer overflow */
	if (di->timer_n1 < di->timer_n2)
		samples = samples + (1 << 24);

	/* IACC (As) = ((ACCUM - OFFSET * COUNT) * 62 [mV] ) / (10 [Mohm] * 32768 [HZ] ) */
	/* (As * 62000) / 3600 -> mAh */

	/* TODO: ensure we do this once an hour (3600*4 samples) to avoid overflow */
	cap = (di->charge_n1 - ((s64) di->cc_offset) * ((s64) di->timer_n1)) * 62000LL;
	tmp = (cap < 0) ? -cap : cap;
	do_div(tmp, 1179648);
	cap = (cap < 0) ? -tmp : tmp;

	cur = di->charge_n1 - di->charge_n2 - (di->cc_offset * samples);
	cur *= (62LL * 4LL * 100000LL);
	tmp = (cur < 0) ? -cur : cur;
	do_div(tmp, samples);
	tmp >>= 15; /* / 32768 */
	cur = (cur < 0) ? -tmp : tmp;
	
	di->current_avg_uA = (int) cur;

	twl6030_update_voltage(di);

	/* detect charge termination */
	if ((di->voltage_mV > 4100) && (di->current_avg_uA < 50000) && (di->current_avg_uA > 0)) {
		if (di->trust_capacity && (di->capacity_uAh < di->capacity_max_uAh) && (di->current_avg_uA > 25000)) {
			/* if we are charging back to a full state with the CC,
			 * be a bit more aggressive than if we only have voltage
			 * to go by
			 */

			/* bump the full time out until the aggressive conditions are not met */
			di->full_jiffies = msecs_to_jiffies(120 * 1000) + jiffies;
		} else if (time_after_eq(jiffies, di->full_jiffies)) {
			di->full_jiffies = msecs_to_jiffies(120 * 1000) + jiffies;
			di->trust_capacity = 1;
			printk("battery: full state detected\n");

			/* calibration will zero the cc accumulator */
			twl6030_calibrate_fuelgauge(di);
			di->capacity_offset = di->capacity_max_uAh;
			cap = 0;

			di->state = STATE_FULL;
			statechanged = 1;
		}
	} else {
		/* bump the full time out as long as we aren't charging */
		di->full_jiffies = msecs_to_jiffies(120 * 1000) + jiffies;
	}

	cap += di->capacity_offset;

	/* limit capacity range to reasonable values */
	if (cap > ((s64) di->capacity_max_uAh))
		cap = di->capacity_max_uAh;
	if (cap < 0)
		cap = 0;

	di->capacity_uAh = cap;

	/* scale to percentage */
	newcap = cap;
	newcap = newcap / (di->capacity_max_uAh / 100);

	if (!di->trust_capacity) {
		/* if we haven't hit a known full charge state, we may
		 * have more capacity than measured by the CC, so use the
		 * CC measured capacity as the floor, but allow higher
		 * estimated capacities based on voltage
		 */
		ret = twl6030_estimate_capacity(di);
		if (ret > newcap)
			newcap = ret;
	}

	printk("battery: %lld uA  %lld uAh  %d mV  %d s  (%d%%) %s%s\n",
		cur, cap, di->voltage_mV, samples / 4, newcap,
		twl6030_state[di->state],
		di->trust_capacity ? " CC" : " EST");

	di->capacity = newcap;
	power_supply_changed(&di->bat);

	return;
err:
	pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_calibration_work(struct work_struct *work)
{
	struct twl6030_battery_device_info	*di =
		container_of(work, struct twl6030_battery_device_info, calibration_work);
	twl6030_calibrate_fuelgauge(di);
}

static void twl6030_monitor_work(struct work_struct *work)
{
	struct twl6030_battery_device_info *di = container_of(work,
			struct twl6030_battery_device_info, monitor_work.work);

	wake_lock(&battery_wake_lock);

	queue_delayed_work(di->wq, &di->monitor_work,
			di->monitoring_interval_jiffies);

	twl6030_read_fuelgauge(di);
	/* TODO: monitor battery temperature */

	wake_unlock(&battery_wake_lock);
}

#define to_twl6030_battery_device_info(x) container_of((x), \
		struct twl6030_battery_device_info, bat);

static int twl6030_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct twl6030_battery_device_info *di;

	di = to_twl6030_battery_device_info(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			twl6030_battery_current(di);
			if (di->current_uA > 0)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = twl6030_get_battery_voltage(di) * 1000;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			twl6030_battery_current(di);
			val->intval = di->current_uA;
			break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			val->intval = di->capacity_uAh;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = di->temp_C;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = di->current_avg_uA;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = di->bat_health;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = di->capacity;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int __devinit twl6030_battery_probe(struct platform_device *pdev)
{
	struct twl4030_battery_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_battery_device_info *di;
	int irq = -1;
	int ret;
	int i;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->platform_data = kmemdup(pdata, sizeof(*pdata), GFP_KERNEL);
	if (!di->platform_data) {
		kfree(di);
		return -ENOMEM;
	}

	di->monitoring_interval_jiffies =
		msecs_to_jiffies(pdata->monitoring_interval_seconds * 1000);
	di->capacity_max_uAh = 570000;

	di->full_jiffies = msecs_to_jiffies(120 * 1000) + jiffies;
	di->vbat_jiffies = jiffies;

	di->dev = &pdev->dev;
	di->bat.name = "twl6030_battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = twl6030_battery_props;
	di->bat.num_properties = ARRAY_SIZE(twl6030_battery_props);
	di->bat.get_property = twl6030_battery_get_property;
	di->bat_health = POWER_SUPPLY_HEALTH_GOOD;

	platform_set_drvdata(pdev, di);

	wake_lock_init(&battery_wake_lock, WAKE_LOCK_SUSPEND, "battery_wake_lock");

	di->wq = create_freezable_workqueue(dev_name(&pdev->dev));

	/* settings for temperature sensing */
	ret = twl6030_battery_temp_setup(true);
	if (ret)
		goto temp_setup_fail; 

	ret = power_supply_register(&pdev->dev, &di->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register main battery\n");
		goto batt_failed;
	}

	di->charge_n1 = 0;
	di->timer_n1 = 0;

	INIT_WORK(&di->calibration_work, twl6030_calibration_work);
	INIT_DELAYED_WORK_DEFERRABLE(&di->monitor_work, twl6030_monitor_work);

	ret = twl6030_battery_voltage_setup(di);
	if (ret)
		dev_err(&pdev->dev, "voltage measurement setup failed\n");

	ret = twl6030_battery_current_setup(true);
	if (ret)
		dev_err(&pdev->dev, "current measurement setup failed\n");

	di->gpadc_vbat_chnl = TWL6030_GPADC_VBAT_CHNL;
	di->voltage_mV = twl6030_get_gpadc_conversion(di, di->gpadc_vbat_chnl);
	dev_info(&pdev->dev, "Battery Voltage at Bootup is %d mV\n", di->voltage_mV);

	/* initialize the voltage history table */
	/* TODO: consider the best initial values for the table */
	for (i=0; i < VOLTAGE_HISTORY_LENGTH; i++)
		di->voltage_history[i] = di->voltage_mV;

	/* start with a rough estimate */
	di->capacity = twl6030_estimate_capacity(di);
	if (di->capacity < 5)
		di->capacity = 5;
	if (di->capacity > 50)
		di->capacity = 50;

	ret = twl6030_backupbatt_setup();
	if (ret)
		dev_err(&pdev->dev, "Backup Bat charging setup failed\n");

	queue_work(di->wq, &di->calibration_work);
	queue_delayed_work(di->wq, &di->monitor_work, 0);

	return 0;

	/* TODO: fix fail exit mess */
chg_irq_fail:
	power_supply_unregister(&di->bat);

batt_failed:
	if (irq != -1)
		free_irq(irq, di);

temp_setup_fail:
	wake_lock_destroy(&battery_wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

#ifdef CONFIG_PM
static int twl6030_battery_suspend(struct device *dev)
{
	int ret;

	/* TODO: schedule alarm */

	ret = twl6030_battery_current_setup(false);
	if (ret) {
		pr_err("%s: Current measurement setup failed (%d)!\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int twl6030_battery_resume(struct device *dev)
{
	int ret;

	ret = twl6030_battery_current_setup(true);
	if (ret) {
		pr_err("%s: Current measurement setup failed (%d)!\n",
				__func__, ret);
		return ret;
	}

	return 0;
}
#else
#define twl6030_battery_suspend	NULL
#define twl6030_battery_resume  NULL
#endif /* CONFIG_PM */

static const struct dev_pm_ops pm_ops = {
	.suspend    = twl6030_battery_suspend,
	.resume     = twl6030_battery_resume,
};

static struct platform_driver twl6030_battery_driver = {
	.probe      = twl6030_battery_probe,
	.driver     = {
		.name   = "twl6030_battery",
		.pm     = &pm_ops,
	},
};

static int __init twl6030_battery_init(void)
{
	return platform_driver_register(&twl6030_battery_driver);
}
module_init(twl6030_battery_init);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_battery");

