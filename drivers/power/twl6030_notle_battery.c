/*
 * linux/drivers/power/notle_battery.c
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
#include <linux/i2c/bq2415x.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <asm/div64.h>

#define CONTROLLER_INT_MASK	0x00
#define CONTROLLER_CTRL1	0x01
#define CONTROLLER_WDG		0x02
#define CONTROLLER_STAT1	0x03
#define CHARGERUSB_INT_STATUS	0x04
#define CHARGERUSB_INT_MASK	0x05
#define CHARGERUSB_STATUS_INT1	0x06
#define CHARGERUSB_STATUS_INT2	0x07
#define CHARGERUSB_CTRL1	0x08
#define CHARGERUSB_CTRL2	0x09
#define CHARGERUSB_CTRL3	0x0A
#define CHARGERUSB_STAT1	0x0B
#define CHARGERUSB_VOREG	0x0C
#define CHARGERUSB_VICHRG	0x0D
#define CHARGERUSB_CINLIMIT	0x0E
#define CHARGERUSB_CTRLLIMIT1	0x0F
#define CHARGERUSB_CTRLLIMIT2	0x10
#define ANTICOLLAPSE_CTRL1	0x11
#define ANTICOLLAPSE_CTRL2	0x12

/* TWL6032 registers 0xDA to 0xDE - TWL6032_MODULE_CHARGER */
#define CONTROLLER_CTRL2	0x00
#define CONTROLLER_VSEL_COMP	0x01
#define CHARGERUSB_VSYSREG	0x02
#define CHARGERUSB_VICHRG_PC	0x03
#define LINEAR_CHRG_STS		0x04

#define LINEAR_CHRG_STS_CRYSTL_OSC_OK	0x40
#define LINEAR_CHRG_STS_END_OF_CHARGE	0x20
#define LINEAR_CHRG_STS_VBATOV		0x10
#define LINEAR_CHRG_STS_VSYSOV		0x08
#define LINEAR_CHRG_STS_DPPM_STS	0x04
#define LINEAR_CHRG_STS_CV_STS		0x02
#define LINEAR_CHRG_STS_CC_STS		0x01

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

/* CONTROLLER_INT_MASK */
#define MVAC_FAULT		(1 << 7)
#define MAC_EOC			(1 << 6)
#define LINCH_GATED		(1 << 5)
#define MBAT_REMOVED		(1 << 4)
#define MFAULT_WDG		(1 << 3)
#define MBAT_TEMP		(1 << 2)
#define MVBUS_DET		(1 << 1)
#define MVAC_DET		(1 << 0)

/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_LINCH	(1 << 5)
#define CONTROLLER_CTRL1_EN_CHARGER	(1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)

/* CONTROLLER_STAT1 */
#define CONTROLLER_STAT1_EXTCHRG_STATZ	(1 << 7)
#define CONTROLLER_STAT1_LINCH_GATED	(1 << 6)
#define CONTROLLER_STAT1_CHRG_DET_N	(1 << 5)
#define CONTROLLER_STAT1_FAULT_WDG	(1 << 4)
#define CONTROLLER_STAT1_VAC_DET	(1 << 3)
#define VAC_DET	(1 << 3)
#define CONTROLLER_STAT1_VBUS_DET	(1 << 2)
#define VBUS_DET	(1 << 2)
#define CONTROLLER_STAT1_BAT_REMOVED	(1 << 1)
#define CONTROLLER_STAT1_BAT_TEMP_OVRANGE (1 << 0)

/* CHARGERUSB_INT_STATUS */
#define EN_LINCH		(1 << 4)
#define CURRENT_TERM_INT	(1 << 3)
#define CHARGERUSB_STAT		(1 << 2)
#define CHARGERUSB_THMREG	(1 << 1)
#define CHARGERUSB_FAULT	(1 << 0)

/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM		(1 << 3)
#define MASK_MCHARGERUSB_STAT		(1 << 2)
#define MASK_MCHARGERUSB_THMREG		(1 << 1)
#define MASK_MCHARGERUSB_FAULT		(1 << 0)

/* CHARGERUSB_STATUS_INT1 */
#define CHARGERUSB_STATUS_INT1_TMREG	(1 << 7)
#define CHARGERUSB_STATUS_INT1_NO_BAT	(1 << 6)
#define CHARGERUSB_STATUS_INT1_BST_OCP	(1 << 5)
#define CHARGERUSB_STATUS_INT1_TH_SHUTD	(1 << 4)
#define CHARGERUSB_STATUS_INT1_BAT_OVP	(1 << 3)
#define CHARGERUSB_STATUS_INT1_POOR_SRC	(1 << 2)
#define CHARGERUSB_STATUS_INT1_SLP_MODE	(1 << 1)
#define CHARGERUSB_STATUS_INT1_VBUS_OVP	(1 << 0)

/* CHARGERUSB_STATUS_INT2 */
#define ICCLOOP		(1 << 3)
#define CURRENT_TERM	(1 << 2)
#define CHARGE_DONE	(1 << 1)
#define ANTICOLLAPSE	(1 << 0)

/* CHARGERUSB_CTRL1 */
#define SUSPEND_BOOT	(1 << 7)
#define OPA_MODE	(1 << 6)
#define HZ_MODE		(1 << 5)
#define TERM		(1 << 4)

/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50	(0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100	(1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150	(2 << 5)
#define CHARGERUSB_CTRL2_VITERM_400	(7 << 5)

/* CHARGERUSB_CTRL3 */
#define VBUSCHRG_LDO_OVRD	(1 << 7)
#define CHARGE_ONCE		(1 << 6)
#define BST_HW_PR_DIS		(1 << 5)
#define AUTOSUPPLY		(1 << 3)
#define BUCK_HSILIM		(1 << 0)

/* CHARGERUSB_VOREG */
#define CHARGERUSB_VOREG_3P52		0x01
#define CHARGERUSB_VOREG_4P0		0x19
#define CHARGERUSB_VOREG_4P2		0x23
#define CHARGERUSB_VOREG_4P76		0x3F

/* CHARGERUSB_VICHRG */
#define CHARGERUSB_VICHRG_300		0x0
#define CHARGERUSB_VICHRG_500		0x4
#define CHARGERUSB_VICHRG_1500		0xE

/* CHARGERUSB_CINLIMIT */
#define CHARGERUSB_CIN_LIMIT_100	0x1
#define CHARGERUSB_CIN_LIMIT_300	0x5
#define CHARGERUSB_CIN_LIMIT_500	0x9
#define CHARGERUSB_CIN_LIMIT_NONE	0xF

/* CHARGERUSB_CTRLLIMIT1 */
#define VOREGL_4P16			0x21
#define VOREGL_4P56			0x35

/* CHARGERUSB_CTRLLIMIT2 */
#define CHARGERUSB_CTRLLIMIT2_1500	0x0E
#define		LOCK_LIMIT		(1 << 4)

/* ANTICOLLAPSE_CTRL2 */
#define BUCK_VTH_SHIFT			5

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

#define GPADC_ISOURCE_22uA		22
#define GPADC_ISOURCE_7uA		7

/* TWL6030/6032 BATTERY VOLTAGE GPADC CHANNELS */
#define TWL6030_GPADC_VBAT_CHNL	0x07
#define TWL6032_GPADC_VBAT_CHNL	0x12

/* TWL6030_GPADC_CTRL2 */
#define GPADC_CTRL2_CH18_SCALER_EN	BIT(2)

#define ENABLE_ISOURCE		0x80

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

#define STS_HW_CONDITIONS	0x21
#define STS_USB_ID		(1 << 2)	/* Level status of USB ID */

#define BATTERY_RESISTOR	10000
#define SIMULATOR_RESISTOR	5000
#define BATTERY_DETECT_THRESHOLD	((BATTERY_RESISTOR + SIMULATOR_RESISTOR) / 2)
#define CHARGING_CAPACITY_UPDATE_PERIOD	(1000 * 60 * 10)

/* To get VBUS input limit from twl6030_usb */
#if CONFIG_TWL6030_USB
extern unsigned int twl6030_get_usb_max_power(struct otg_transceiver *x);
#else
static inline unsigned int twl6030_get_usb_max_power(struct otg_transceiver *x)
{
	return 0;
};
#endif

/* sign extension needs a little care */
static __inline int sign_extend(int n, int num_bits)
{
	int shift = (int)(sizeof(int) * 8 - num_bits);
	return (n << shift) >> shift;
}

/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};
static struct wake_lock usb_wake_lock;
static struct wake_lock battery_wake_lock;

#define STATE_BATTERY		0 /* no wall power, charging disabled */
#define STATE_FAULT		1 /* charging off due to fault condition */
#define STATE_FULL		2 /* wall power but battery is charged */
#define STATE_USB		3 /* 500mA wall power, charging enabled */
#define STATE_AC		4 /* 1000mA wall power, charging enabled */

static const char *twl6030_state[] = {
	"BATTERY", "FAULT", "FULL", "USB", "AC"
};

#define is_powered(di)	(di->state > STATE_FAULT)
#define is_charging(di) (di->state > STATE_FULL)

/* change the order, not the length to keep this a power of 2 */
#define VOLTAGE_HISTORY_ORDER 3
#define VOLTAGE_HISTORY_LENGTH (1<<VOLTAGE_HISTORY_ORDER)

struct twl6030_bci_device_info {
	struct device		*dev;

	int			voltage_mV;
	int			voltage_history[VOLTAGE_HISTORY_LENGTH];
	int			voltage_index;
	int			current_uA;
	int			current_avg_uA;
	int			bat_health;
	int			state;
	int			vbus_online;

	unsigned long vbat_jiffies;
	unsigned long full_jiffies;

	int			timer_n2;
	int			timer_n1;
	s32			charge_n1;
	s32			charge_n2;
	s16			cc_offset;
	u8			usb_online;

	s64			capacity_offset;
	int			capacity_uAh;
	int			capacity_max_uAh;
	int			trust_capacity;

	u8			gpadc_vbat_chnl;
	u8			watchdog_duration;
	u16			monitoring_interval;
	unsigned int		min_vbus;

	struct			twl4030_bci_platform_data *platform_data;

	unsigned int		charger_incurrentmA;
	unsigned int		charger_outcurrentmA;
	unsigned long		usb_max_power;
	unsigned long		usb_event;

	unsigned int		capacity;

	struct power_supply	bat;
	struct power_supply	usb;

	struct otg_transceiver	*otg;
	struct notifier_block	nb;

	struct work_struct	charge_control_work;
	struct work_struct	charge_fault_work;
	struct work_struct	calibration_work;
	struct delayed_work	monitor_work;

	struct workqueue_struct	*wq;
	unsigned long		features;
};

static void twl6030_config_min_vbus_reg(struct twl6030_bci_device_info *di,
						unsigned int value)
{
	u8 rd_reg = 0;
	int ret;

	if (value > 4760 || value < 4200) {
		dev_dbg(di->dev, "invalid min vbus\n");
		return;
	}

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg,
					ANTICOLLAPSE_CTRL2);
	if (ret)
		goto err;
	rd_reg = rd_reg & 0x1F;
	rd_reg = rd_reg | (((value - 4200)/80) << BUCK_VTH_SHIFT);
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg,
					ANTICOLLAPSE_CTRL2);

	if (!ret)
		return;
err:
	pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_iterm_reg(struct twl6030_bci_device_info *di,
						unsigned int term_currentmA)
{
	int ret;

	if ((term_currentmA > 400) || (term_currentmA < 50)) {
		dev_dbg(di->dev, "invalid termination current\n");
		return;
	}

	term_currentmA = ((term_currentmA - 50)/50) << 5;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
						CHARGERUSB_CTRL2);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_voreg_reg(struct twl6030_bci_device_info *di,
							unsigned int voltagemV)
{
	int ret;

	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_VOREG);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_vichrg_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	int ret;

	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid charger_currentmA\n");
		return;
	}

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_VICHRG);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_cinlimit_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	int ret;

	if ((currentmA >= 50) && (currentmA <= 750)) {
		currentmA = (currentmA - 50) / 50;
	} else if (currentmA < 50) {
		dev_dbg(di->dev, "invalid input current limit\n");
		return;
	} else {
		/* This is no current limit */
		currentmA = 0x0F;
	}

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
					CHARGERUSB_CINLIMIT);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_limit1_reg(struct twl6030_bci_device_info *di,
							unsigned int voltagemV)
{
	int ret;

	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid max_charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_CTRLLIMIT1);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_config_limit2_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	int ret;

	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid max_charger_currentmA\n");
		return;
	}

	currentmA |= LOCK_LIMIT;
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_CTRLLIMIT2);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static int twl6030_set_watchdog(struct twl6030_bci_device_info *di, int val)
{
	di->watchdog_duration = val;
	return twl_i2c_write_u8(TWL6030_MODULE_CHARGER, val, CONTROLLER_WDG);
}


static const int vichrg[] = {
	300, 350, 400, 450, 500, 600, 700, 800,
	900, 1000, 1100, 1200, 1300, 1400, 1500, 300
};

/*
 * Return channel value
 * Or < 0 on failure.
 */
static int twl6030_get_gpadc_conversion(struct twl6030_bci_device_info *di,
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

static int is_battery_present(struct twl6030_bci_device_info *di)
{
	/* TODO */
	return 1;
}

static void twl6030_stop_usb_charger(struct twl6030_bci_device_info *di)
{
	int ret;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
	if (ret)
		pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
	/* printk("battery: CHARGER OFF\n"); */
}

static void twl6030_start_usb_charger(struct twl6030_bci_device_info *di, int mA)
{
	int ret;

	if (!is_battery_present(di)) {
		dev_err(di->dev, "BATTERY NOT DETECTED!\n");
		return;
	}

	if (mA < 50) {
		ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
		if (ret)
			goto err;
		return;
	}

	twl6030_config_vichrg_reg(di, di->charger_outcurrentmA);
	twl6030_config_cinlimit_reg(di, mA);
	twl6030_config_voreg_reg(di, di->platform_data->max_bat_voltagemV);
	twl6030_config_iterm_reg(di, di->platform_data->termination_currentmA);

	if (mA >= 50) {
		twl6030_set_watchdog(di, di->watchdog_duration);
		/* disable current termination, suspend mode, boost mode, etc */
		twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CHARGERUSB_CTRL1);
		ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, CONTROLLER_CTRL1_EN_CHARGER, CONTROLLER_CTRL1);
		if (ret)
			goto err;
	}
	/* printk("battery: CHARGER ON\n"); */
	return;

err:
	pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

/*
 * Interrupt service routine
 *
 * Attends to TWL 6030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */
static irqreturn_t twl6030charger_ctrl_interrupt(int irq, void *_di)
{
	struct twl6030_bci_device_info *di = _di;
	queue_work(di->wq, &di->charge_control_work);
	printk("battery: CHARGE CTRL IRQ\n");
	return IRQ_HANDLED;
}

static irqreturn_t twl6030charger_fault_interrupt(int irq, void *_di)
{
	struct twl6030_bci_device_info *di = _di;
	int ret;

	u8 usb_charge_sts, usb_charge_sts1, usb_charge_sts2;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
						CHARGERUSB_INT_STATUS);
	if (ret)
		goto err;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
						CHARGERUSB_STATUS_INT1);
	if (ret)
		goto err;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
						CHARGERUSB_STATUS_INT2);
	if (ret)
		goto err;

	printk("battery: CHARGE FAULT IRQ: STS %02x INT1 %02x INT2 %02x\n",
		usb_charge_sts, usb_charge_sts1, usb_charge_sts2);
err:
	queue_work(di->wq, &di->charge_fault_work);
	return IRQ_HANDLED;
}

static void twl6030battery_current(struct twl6030_bci_device_info *di)
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

static int twl6030backupbatt_setup(void)
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

static int twl6030battery_temp_setup(bool enable)
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

static int twl6030battery_voltage_setup(struct twl6030_bci_device_info *di)
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

static int twl6030battery_current_setup(bool enable)
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

static enum power_supply_property twl6030_bci_battery_props[] = {
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

static enum power_supply_property twl6030_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int twl6030_calibrate_fuelgauge(struct twl6030_bci_device_info *di)
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

static int twl6030_estimate_capacity(struct twl6030_bci_device_info *di)
{
	if (di->voltage_mV < 3200) return 0;
	if (di->voltage_mV < 3500) return 5;
	if (di->voltage_mV < 3600) return 20;
	if (di->voltage_mV < 3700) return 50;
	if (di->voltage_mV < 3800) return 75;
	if (di->voltage_mV < 3900) return 90;
	return 95;
}

static int twl6030_get_battery_voltage(struct twl6030_bci_device_info *di)
{
	int v;
	/* when the charger is enabled the voltage does not reflect the
	 * actual battery voltage, so use the cached voltage (sampled
	 * periodically with the charger disabled)
	 */
	if (is_charging(di))
		return di->voltage_mV;
	v = twl6030_get_gpadc_conversion(di, di->gpadc_vbat_chnl);
	if (v <= 0)
		return di->voltage_mV;
	else
		return v;
}

static int twl6030_read_gasguage_regs(struct twl6030_bci_device_info *di)
{
	int ret;
	u32 data[2];

#if 0
	s16 integ;

	ret = twl_i2c_read_u8(TWL6030_MODULE_GASGAUGE, (u8 *) data, FG_REG_00);
	if (ret < 0) {
		pr_err("twl6030: failed to read gasgauge config\n");
		goto done;
	}
#endif

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

#if 0
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &integ, FG_REG_10, 2);
	if (ret < 0) {
		pr_err("twl6030: failed to read integrator\n");
		goto err;
	}

	/* sign extend the value */
	integ = sign_extend(integ, 14);

	printk("battery: data[0] = %08x data[1] = %08x\n", data[0], data[1]);
	printk("battery: integ = %02x (%d)\n", (unsigned) integ, (int) integ);
#endif

err:
	ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, 0, FG_REG_00);
	if (ret < 0) {
		pr_err("twl6030: cannot resume gasgauge\n");
	}

done:
	return ret;
}

static void twl6030_update_voltage(struct twl6030_bci_device_info *di)
{
	int i;
	int ret;
	int index, q;
	long long total, denom;
	int curr_voltage;

	if (is_charging(di)) {
		if (time_after_eq(jiffies, di->vbat_jiffies)) {
			di->vbat_jiffies = msecs_to_jiffies(60 * 1000) + jiffies;

			twl6030_stop_usb_charger(di);

			msleep(200);

			ret = twl6030_get_gpadc_conversion(di, di->gpadc_vbat_chnl);
			if (ret > 0)
				curr_voltage = ret;

			twl6030_start_usb_charger(di, 500);
		} else {
			/* if no sample is taken don't bother recalculating the weighted average */
			return;
		}
	} else {
		di->vbat_jiffies = jiffies;
		ret = twl6030_get_gpadc_conversion(di, di->gpadc_vbat_chnl);
		if (ret > 0)
			curr_voltage = ret;
	}

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

static void twl6030_read_fuelgauge(struct twl6030_bci_device_info *di)
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
	/* TODO: make configurable */
	/* TODO: termination after X s even if other conditions not met */
	if (is_charging(di) && (di->voltage_mV > 4100) && (di->current_avg_uA < 50000)) {
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
			twl6030_stop_usb_charger(di);	
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

	printk("battery: %lld uA  %lld uAh  %d mV  %d s  (%d%%) %s%s%s\n",
		cur, cap, di->voltage_mV, samples / 4, newcap,
		twl6030_state[di->state],
		is_charging(di) ? " CHG" : "",
		di->trust_capacity ? " CC" : " EST");

	if ((newcap != di->capacity) || statechanged) {
		di->capacity = newcap;
		power_supply_changed(&di->bat);
	}

	if ((di->state == STATE_FULL) && (di->capacity < 95)) {
		printk("battery: drained from full to %d%%, charging again\n", di->capacity);
		di->state = STATE_USB;
		twl6030_start_usb_charger(di, 500);
		statechanged = 1;
	}

	return;
err:
	pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_determine_charge_state(struct twl6030_bci_device_info *di)
{
	u8 stat1;
	int newstate = STATE_BATTERY;

	/* TODO: i2c error -> fault? */
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &stat1, CONTROLLER_STAT1);

	/* TODO: why is STAT1.0 (BAT_TEMP_OVRANGE) always set? */
	/* printk("battery: determine_charge_state() stat1=%02x int1=%02x\n", stat1, int1); */
	
	if (stat1 & VBUS_DET) {
		/* dedicated charger detected by PHY? */
		if (di->usb_event == USB_EVENT_CHARGER)
			newstate = STATE_AC;
		else
			newstate = STATE_USB;

		if (!di->vbus_online) {
			di->vbus_online = 1;
			wake_lock(&usb_wake_lock);
		}
	} else {
		/* ensure we don't have a stale USB_EVENT_CHARGER should detect bounce */
		di->usb_event = USB_EVENT_NONE;

		if (di->vbus_online) {
			di->vbus_online = 0;
			/* give USB and userspace some time to react before suspending */
			wake_lock_timeout(&usb_wake_lock, HZ / 2);
		}
	}

	if (di->state == newstate)
		return;

	switch (newstate) {
	case STATE_FAULT:
	case STATE_BATTERY:
		if (is_charging(di))
			twl6030_stop_usb_charger(di);
		break;
	case STATE_USB:
	case STATE_AC:
		/* moving out of STATE_FULL should only happen on unplug
		 * or if we actually run down the battery capacity
		 */
		if (di->state == STATE_FULL) {
			newstate = STATE_FULL;
			break;
		}

		/* TODO: high current? */
		if (!is_charging(di))
			twl6030_start_usb_charger(di, 500);
		break;
	}

	if (di->state != newstate) {
		printk("battery: state %s -> %s\n",
			twl6030_state[di->state], twl6030_state[newstate]);
		di->state = newstate;
		power_supply_changed(&di->bat);
		power_supply_changed(&di->usb);
	}
}

static void twl6030_charge_control_work(struct work_struct *work)
{
	struct twl6030_bci_device_info	*di =
		container_of(work, struct twl6030_bci_device_info, charge_control_work);
	twl6030_determine_charge_state(di);
}

static void twl6030_charge_fault_work(struct work_struct *work)
{
	struct twl6030_bci_device_info	*di =
		container_of(work, struct twl6030_bci_device_info, charge_fault_work);

	if (is_charging(di))
		twl6030_start_usb_charger(di, 500);
	msleep(10);

	twl6030_determine_charge_state(di);
}

static void twl6030_calibration_work(struct work_struct *work)
{
	struct twl6030_bci_device_info	*di =
		container_of(work, struct twl6030_bci_device_info, calibration_work);
	twl6030_calibrate_fuelgauge(di);
}

static void twl6030_monitor_work(struct work_struct *work)
{
	struct twl6030_bci_device_info *di = container_of(work,
			struct twl6030_bci_device_info, monitor_work.work);

	wake_lock(&battery_wake_lock);

	/* pet the charger watchdog */
	if (is_charging(di))
		twl6030_set_watchdog(di, di->watchdog_duration);

	queue_delayed_work(di->wq, &di->monitor_work,
			msecs_to_jiffies(1000 * di->monitoring_interval));

	twl6030_read_fuelgauge(di);

	/* TODO: monitor battery temperature */

	twl6030_determine_charge_state(di);

	wake_unlock(&battery_wake_lock);
}

#define to_twl6030_bci_device_info(x) container_of((x), \
			struct twl6030_bci_device_info, bat);

#define to_twl6030_usb_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, usb);

static int twl6030_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_usb_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->vbus_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_gpadc_conversion(di, 10) * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int twl6030_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di;

	di = to_twl6030_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (di->state) {
		case STATE_USB:
		case STATE_AC:
		case STATE_FULL: /* TODO ?? */
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_battery_voltage(di) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		twl6030battery_current(di);
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = di->capacity_uAh;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 275; // simulate a healthy battery temp
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

static int twl6030_usb_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct twl6030_bci_device_info *di =
		container_of(nb, struct twl6030_bci_device_info, nb);

	di->usb_event = event;
	switch (event) {
	case USB_EVENT_VBUS:
		di->usb_online = *((unsigned int *) data);
		break;
	case USB_EVENT_ENUMERATED:
		di->usb_max_power = *((unsigned int *) data);
		break;
	case USB_EVENT_CHARGER:
	case USB_EVENT_NONE:
		break;
	case USB_EVENT_ID:
	default:
		return NOTIFY_OK;
	}

	if (di->usb_event != event) {
		di->usb_event = event;
		queue_work(di->wq, &di->charge_control_work);
	}

	return NOTIFY_OK;
}

static ssize_t show_vbus_voltage(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = twl6030_get_gpadc_conversion(di, 10);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_id_level(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = twl6030_get_gpadc_conversion(di, 14);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_regulation_voltage(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
			|| (val > di->platform_data->max_charger_voltagemV))
		return -EINVAL;
	di->platform_data->max_bat_voltagemV = val;
	twl6030_config_voreg_reg(di, val);

	return status;
}

static ssize_t show_regulation_voltage(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->platform_data->max_bat_voltagemV;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_termination_current(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 400))
		return -EINVAL;
	di->platform_data->termination_currentmA = val;
	twl6030_config_iterm_reg(di, val);

	return status;
}

static ssize_t show_termination_current(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->platform_data->termination_currentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_cin_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 1500))
		return -EINVAL;
	di->charger_incurrentmA = val;
	twl6030_config_cinlimit_reg(di, val);

	return status;
}

static ssize_t show_cin_limit(struct device *dev, struct device_attribute *attr,
								  char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->charger_incurrentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 300)
			|| (val > di->platform_data->max_charger_currentmA))
		return -EINVAL;
	di->charger_outcurrentmA = val;
	twl6030_config_vichrg_reg(di, val);

	return status;
}

static ssize_t show_charge_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->charger_outcurrentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_min_vbus(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
		return -EINVAL;
	di->min_vbus = val;
	twl6030_config_min_vbus_reg(di, val);

	return status;
}

static ssize_t show_min_vbus(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->min_vbus;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_model(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", di->trust_capacity ? 1 : 0);
}

static DEVICE_ATTR(model, S_IRUGO, show_model, NULL);
static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(id_level, S_IRUGO, show_id_level, NULL);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
		show_regulation_voltage, set_regulation_voltage);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		show_termination_current, set_termination_current);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO, show_cin_limit,
		set_cin_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO, show_charge_current,
		set_charge_current);
static DEVICE_ATTR(min_vbus, S_IWUSR | S_IRUGO, show_min_vbus, set_min_vbus);

static struct attribute *twl6030_bci_attributes[] = {
	&dev_attr_model.attr,
	&dev_attr_vbus_voltage.attr,
	&dev_attr_id_level.attr,
	&dev_attr_regulation_voltage.attr,
	&dev_attr_termination_current.attr,
	&dev_attr_cin_limit.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_min_vbus.attr,
	NULL,
};

static const struct attribute_group twl6030_bci_attr_group = {
	.attrs = twl6030_bci_attributes,
};

static char *twl6030_bci_supplied_to[] = {
	"twl6030_battery",
};

static int __devinit twl6030_bci_battery_probe(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_bci_device_info *di;
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

	di->state = STATE_BATTERY;

	di->monitoring_interval = 15;
	di->capacity_max_uAh = 570000;

	di->full_jiffies = msecs_to_jiffies(120 * 1000) + jiffies;
	di->vbat_jiffies = jiffies;

	di->dev = &pdev->dev;
	di->bat.name = "twl6030_battery";
	di->bat.supplied_to = twl6030_bci_supplied_to;
	di->bat.num_supplicants = ARRAY_SIZE(twl6030_bci_supplied_to);
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = twl6030_bci_battery_props;
	di->bat.num_properties = ARRAY_SIZE(twl6030_bci_battery_props);
	di->bat.get_property = twl6030_bci_battery_get_property;
	di->bat_health = POWER_SUPPLY_HEALTH_GOOD;

	di->usb.name = "twl6030_usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = twl6030_usb_props;
	di->usb.num_properties = ARRAY_SIZE(twl6030_usb_props);
	di->usb.get_property = twl6030_usb_get_property;

	platform_set_drvdata(pdev, di);

	wake_lock_init(&usb_wake_lock, WAKE_LOCK_SUSPEND, "usb_wake_lock");
	wake_lock_init(&battery_wake_lock, WAKE_LOCK_SUSPEND, "battery_wake_lock");

	di->wq = create_freezable_workqueue(dev_name(&pdev->dev));

	/* settings for temperature sensing */
	ret = twl6030battery_temp_setup(true);
	if (ret)
		goto temp_setup_fail; 
	ret = power_supply_register(&pdev->dev, &di->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register main battery\n");
		goto batt_failed;
	}

	ret = power_supply_register(&pdev->dev, &di->usb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb power supply\n");
		goto usb_failed;
	}

	di->charge_n1 = 0;
	di->timer_n1 = 0;

	INIT_WORK(&di->charge_control_work, twl6030_charge_control_work);
	INIT_WORK(&di->charge_fault_work, twl6030_charge_fault_work);
	INIT_WORK(&di->calibration_work, twl6030_calibration_work);
	INIT_DELAYED_WORK_DEFERRABLE(&di->monitor_work, twl6030_monitor_work);

	ret = twl6030battery_voltage_setup(di);
	if (ret)
		dev_err(&pdev->dev, "voltage measurement setup failed\n");

	ret = twl6030battery_current_setup(true);
	if (ret)
		dev_err(&pdev->dev, "current measurement setup failed\n");

	/* initialize for USB charging */
	twl6030_config_limit1_reg(di, pdata->max_charger_voltagemV);
	twl6030_config_limit2_reg(di, pdata->max_charger_currentmA);
	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
						CONTROLLER_INT_MASK);
	if (ret)
		goto init_failed;

	ret = twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			MASK_MCHARGERUSB_THMREG | MASK_MCURRENT_TERM,
			CHARGERUSB_INT_MASK);
	if (ret)
		goto init_failed;


	di->charger_outcurrentmA = di->platform_data->max_charger_currentmA;

	twl6030_set_watchdog(di, 32);

	di->nb.notifier_call = twl6030_usb_notifier_call;
	di->otg = otg_get_transceiver();
	if (di->otg) {
		ret = otg_register_notifier(di->otg, &di->nb);
		if (ret)
			dev_err(&pdev->dev, "otg register notifier failed %d\n", ret);
	} else
		dev_err(&pdev->dev, "otg_get_transceiver failed %d\n", ret);

	di->charger_incurrentmA = twl6030_get_usb_max_power(di->otg);
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

	ret = twl6030backupbatt_setup();
	if (ret)
		dev_err(&pdev->dev, "Backup Bat charging setup failed\n");

	/* request charger fault interruption */
	irq = platform_get_irq(pdev, 1);
	ret = request_threaded_irq(irq, NULL, twl6030charger_fault_interrupt,
		0, "twl_bci_fault", di);
	if (ret) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n", irq, ret);
		goto init_failed;
	}

	/* request charger ctrl interruption */
	irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(irq, NULL, twl6030charger_ctrl_interrupt,
		0, "twl_bci_ctrl", di);
	if (ret) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n", irq, ret);
		goto chg_irq_fail;
	}

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK, REG_INT_MSK_STS_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK, REG_INT_MSK_STS_C);

	ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	if (ret)
		dev_err(&pdev->dev, "could not create sysfs files\n");

	queue_work(di->wq, &di->calibration_work);
	queue_delayed_work(di->wq, &di->monitor_work, 0);
	return 0;

	/* TODO: fix fail exit mess */
chg_irq_fail:
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);
init_failed:
	power_supply_unregister(&di->usb);
usb_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	if (irq != -1)
		free_irq(irq, di);
temp_setup_fail:
	wake_lock_destroy(&usb_wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

#ifdef CONFIG_PM
static int twl6030_bci_battery_suspend(struct device *dev)
{
	int ret;

	/* TODO: schedule alarm */

	ret = twl6030battery_current_setup(false);
	if (ret) {
		pr_err("%s: Current measurement setup failed (%d)!\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int twl6030_bci_battery_resume(struct device *dev)
{
	int ret;

	ret = twl6030battery_current_setup(true);
	if (ret) {
		pr_err("%s: Current measurement setup failed (%d)!\n",
				__func__, ret);
		return ret;
	}

	return 0;
}
#else
#define twl6030_bci_battery_suspend	NULL
#define twl6030_bci_battery_resume	NULL
#endif /* CONFIG_PM */

static const struct dev_pm_ops pm_ops = {
	.suspend	= twl6030_bci_battery_suspend,
	.resume		= twl6030_bci_battery_resume,
};

static struct platform_driver twl6030_bci_battery_driver = {
	.probe		= twl6030_bci_battery_probe,
	.driver		= {
		.name	= "twl6030_bci",
		.pm	= &pm_ops,
	},
};

static int __init twl6030_battery_init(void)
{
	return platform_driver_register(&twl6030_bci_battery_driver);
}
module_init(twl6030_battery_init);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_bci");
