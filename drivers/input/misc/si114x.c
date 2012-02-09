/* Silicon Labs 1141/42/43 Proximity/ALS Android Driver
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#define DEBUG 1

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

#include <linux/slab.h>

#include <linux/i2c/si114x.h>

#define DRIVER_VERSION "0.1"
#define DEVICE_NAME "si114x"

#define INPUT_DEVICE_NAME_PS "si114x_ps"
#define INPUT_DEVICE_NAME_ALS "si114x_als"

#define SI114X_MAX_LED 3

/* TODO(cmanton) put this in platform data */
static int use_interrupts = 0;

struct si114x_led {
	int drive_strength;
	int enable;
};

/* Both ALS and PS input device structures */
struct si114x_input_dev {
	struct input_dev *input;
	struct input_polled_dev *input_poll;
#if 0
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct input_polled_dev *input_poll_dev_als;
	struct input_polled_dev *input_poll_dev_ps;
#endif
};

struct si114x_data {
	/* Device */
	struct i2c_client *client;
	struct early_suspend early_suspend;

	struct si114x_input_dev input_dev_als;
	struct si114x_input_dev input_dev_ps;
#if 0
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	struct input_polled_dev *input_poll_dev_als;
	struct input_polled_dev *input_poll_dev_ps;
#endif
	struct si114x_led led[SI114X_MAX_LED];

	struct si114x_platform_data *pdata;
};

static int set_led_drive_strength(struct si114x_data *si114x);

/* I2C Read */
static int _i2c_read_mult(struct si114x_data *si114x, char *rxData, int length)
{
	int i;
	struct i2c_msg data[] = {
		{
			.addr = si114x->client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = si114x->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (i = 0; i < I2C_RETRY; i++) {
		if (i2c_transfer(si114x->client->adapter, data, 2) > 0)
			break;
		/* Delay before retrying */
		dev_warn(&si114x->client->dev, "%s Retried read count:%d\n", __func__, i);
		mdelay(10);
	}

	if (i >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n", __func__);
		return -EIO;
	}
	return 0;
}

/* I2C Write */
static int _i2c_write_mult(struct si114x_data *si114x, char *txData, int length)
{
	int i;
	struct i2c_msg data[] = {
		{
			.addr = si114x->client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (i = 0; i < I2C_RETRY; i++) {
		if (i2c_transfer(si114x->client->adapter, data, 1) > 0)
			break;
		/* Delay before retrying */
		dev_warn(&si114x->client->dev, "%s Retried read count:%d\n", __func__, i);
		mdelay(10);
	}

	if (i >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}
	return 0;
}

static int _i2c_read_one(struct si114x_data *si114x, int addr, int *data)
{
	int rc = 0;
	uint8_t buffer[1];

	buffer[0] = SI411X_PART_ID;
	rc = _i2c_read_mult(si114x, buffer, sizeof(buffer));
	if (rc == 0) {
		*data = buffer[0];
	}
	return rc;
}

static int _i2c_write_one(struct si114x_data *si114x, int addr, int data)
{
	uint8_t buffer[2];

	buffer[0] = addr;
	buffer[1] = data;
	return _i2c_write_mult(si114x, buffer, sizeof(buffer));
}

/* Create and register interrupt driven input mechanism */
static int setup_als_input(struct si114x_data *si114x)
{
	int rc;
	struct i2c_client *client = si114x->client;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "%s: Unable to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_dev->name = INPUT_DEVICE_NAME_ALS;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Unable to register input device\n", __func__);
		goto err_als_register_input_device;
	}

	si114x->input_dev_als.input = input_dev;
	return rc;

 err_als_register_input_device:
	input_free_device(input_dev);
	return rc;
}

static void si114x_input_report_als_values(struct input_dev *input, uint16_t *value)
{
	input_report_abs(input, ABS_MISC, value[0]);
	input_sync(input);
}

static void si114x_input_report_ps_values(struct input_dev *input, uint16_t *value)
{
	input_report_abs(input, ABS_DISTANCE, value[0]);
	input_report_abs(input, ABS_DISTANCE, value[1]);
	input_report_abs(input, ABS_DISTANCE, value[2]);
	input_sync(input);
}

static void si114x_input_poll_als_func(struct input_polled_dev *dev)
{
	uint8_t buffer[2];
	struct si114x_data *si114x = dev->private;
	struct i2c_client *client = si114x->client;
	struct input_dev *input;
	uint16_t *data = (uint16_t *)buffer;

	input = si114x->input_dev_als.input_poll->input;
	dev_err(&client->dev, "%s: Polling ALS for measurement\n" , __func__);

	/* TODO(cmanton) locking ? */
	buffer[0] = SI411X_ALS_IR_DATA0;
	if (_i2c_read_mult(si114x, buffer, sizeof(buffer))) {
		dev_err(&client->dev, "%s: Unable to read als data\n", __func__);
		return;
	}
	si114x_input_report_als_values(input, data);
}

static void si114x_input_poll_ps_func(struct input_polled_dev *dev)
{
	uint8_t buffer[6];
	struct si114x_data *si114x = dev->private;
	struct i2c_client *client = si114x->client;
	struct input_dev *input;
	uint16_t *data = (uint16_t *)buffer;

	input = si114x->input_dev_ps.input_poll->input;
	dev_err(&client->dev, "%s: Polling PS for measurement\n" , __func__);

	/* TODO(cmanton) locking ? */
	buffer[0] = SI411X_PS1_DATA0;
	if (_i2c_read_mult(si114x, buffer, sizeof(buffer))) {
		dev_err(&client->dev, "%s: Unable to read als data\n", __func__);
		return;
	}
	si114x_input_report_ps_values(input, data);
}

static int si114x_input_als_open(struct input_dev *input)
{
	struct si114x_data *si114x = input_get_drvdata(input);
	struct i2c_client *client;

	printk("%s\n", __func__);
	return 0;
	client = si114x->client;

	dev_info(&client->dev, "%s: Input subsystem opened device for access\n", __func__);
	schedule_delayed_work(&si114x->input_dev_als.input_poll->work,
	                      msecs_to_jiffies(1000));
	return 0;
}

static void si114x_input_als_close(struct input_dev *input)
{
	struct si114x_data *si114x = input_get_drvdata(input);
	struct i2c_client *client;
	printk("%s\n", __func__);
	return;
	client = si114x->client;

	dev_info(&client->dev, "%s: Input subsystem closed device for access\n", __func__);
	cancel_delayed_work_sync(&si114x->input_dev_als.input_poll->work);
}

/* Create and register polled input mechanism */
static int setup_als_polled_input(struct si114x_data *si114x)
{
	int rc;
	struct i2c_client *client = si114x->client;
	struct input_polled_dev *input_poll;
	struct input_dev *input;

	input_poll = input_allocate_polled_device();
	if (!input_poll) {
		dev_err(&client->dev, "%s: Unable to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_poll->private = si114x;
	input_poll->poll = si114x_input_poll_als_func;
	/* TODO(cmanton) move knob elsewhere */
	input_poll->poll_interval = 1000;

	input = input_poll->input;

	input->name = INPUT_DEVICE_NAME_ALS;

	input->open = si114x_input_als_open;
	input->close = si114x_input_als_close;
	set_bit(EV_ABS, input->evbit);
	printk(KERN_ERR "%s open:%p close:%p\n", __func__, input->open, input->close);

	input->id.bustype = BUS_I2C;
	input->dev.parent = &si114x->client->dev;

	input_set_abs_params(input, ABS_MISC, ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	input_set_drvdata(input, input_poll);

	rc = input_register_polled_device(input_poll);
	if (rc) {
		dev_err(&si114x->client->dev,
		        "Unable to register input polled device %s\n",
		        input->name);
		goto err_als_register_input_device;
	}

	si114x->input_dev_als.input_poll = input_poll;

	return rc;

 err_als_register_input_device:
	input_free_polled_device(input_poll);

	return rc;
}

static int setup_ps_input(struct si114x_data *si114x)
{
	int rc;
	struct i2c_client *client = si114x->client;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "%s: Unable to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_dev->name = INPUT_DEVICE_NAME_PS;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(&client->dev, "%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}
	si114x->input_dev_ps.input = input_dev;

	return rc;

err_ps_register_input_device:
	input_free_device(input_dev);

	return rc;
}

static int si114x_input_ps_open(struct input_dev *input)
{
	struct si114x_data *si114x = input_get_drvdata(input);
	struct i2c_client *client;
	printk("%s\n", __func__);
	return 0;

	client = si114x->client;

	dev_info(&client->dev, "%s: Input subsystem opened device for access\n", __func__);

	schedule_delayed_work(&si114x->input_dev_ps.input_poll->work,
	                      msecs_to_jiffies(1000));
	return 0;
}

static void si114x_input_ps_close(struct input_dev *input)
{
	struct si114x_data *si114x = input_get_drvdata(input);
	struct i2c_client *client;
	printk("%s\n", __func__);
	return;

	client = si114x->client;

	dev_info(&client->dev, "%s: Input subsystem closed device for access\n", __func__);

	cancel_delayed_work_sync(&si114x->input_dev_ps.input_poll->work);
}

static int setup_ps_polled_input(struct si114x_data *si114x)
{
	int rc;
	struct i2c_client *client = si114x->client;
	struct input_polled_dev *input_poll;
	struct input_dev *input;

	input_poll = input_allocate_polled_device();
	if (!input_poll) {
		dev_err(&client->dev, "%s: Unable to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_poll->private = si114x;
	input_poll->poll = si114x_input_poll_ps_func;
	/* TODO(cmanton) move knob elsewhere */
	input_poll->poll_interval = 1000;

	input = input_poll->input;

	input->name = INPUT_DEVICE_NAME_PS;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &si114x->client->dev;

	set_bit(EV_ABS, input->evbit);

	input->open = si114x_input_ps_open;
	input->close = si114x_input_ps_close;
	printk(KERN_ERR "%s open:%p close:%p\n", __func__, input->open, input->close);

	input_set_abs_params(input, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	input_set_drvdata(input, input_poll);

	rc = input_register_polled_device(input_poll);
	if (rc) {
		dev_err(&si114x->client->dev,
		        "Unable to register input polled device %s\n",
		        input_poll->input->name);
		goto err_ps_register_input_device;
	}

	si114x->input_dev_ps.input_poll = input_poll;
	return rc;

err_ps_register_input_device:
	input_free_polled_device(si114x->input_dev_ps.input_poll);

	return rc;
}


static void si114x_early_suspend(struct early_suspend *h)
{
	printk(KERN_ERR "%s: Entering early suspend\n", __func__);
#if 0
	int rc = 0;
	struct si114x_data *si114x = sensor_info;

	if (si114x->is_suspend != 0) {
		dev_err(&si114x->i2c_client->dev, "%s Asked to suspend when already suspended\n", __func__);
		return;
	}
	si114x->is_suspend = 1;

	/* Save away the state of the devices at suspend point */
	si114x->als_suspend_enable_flag = si114x->als_enable_flag;
	si114x->ps_suspend_enable_flag = si114x->ps_enable_flag;

	/* Disable the devices for suspend if configured */
	if (si114x->disable_als_on_suspend && si114x->als_enable_flag) {
		ret += als_disable(si114x);
	}
	if (si114x->disable_ps_on_suspend && si114x->ps_enable_flag) {
		ret += ps_disable(si114x);
	}

	if (ret) {
		dev_err(&si114x->i2c_client->dev, "%s Unable to complete suspend\n", __func__);
	} else {
		dev_info(&si114x->i2c_client->dev, "%s Suspend completed\n", __func__);
	}
#endif
}

static void si114x_late_resume(struct early_suspend *h)
{
	printk(KERN_ERR "%s: Entering early suspend\n", __func__);
#if 0
	struct si114x_data *si114x = sensor_info;
	int ret = 0;

	if (si114x->is_suspend != 1) {
		dev_err(&si114x->i2c_client->dev, "%s Asked to resume when not suspended\n", __func__);
		return;
	}
	si114x->is_suspend = 0;

	/* If ALS was enbled before suspend, enable during resume */
	if (si114x->als_suspend_enable_flag) {
		ret += als_enable(si114x);
		si114x->als_suspend_enable_flag = 0;
	}

	/* If PS was enbled before suspend, enable during resume */
	if (si114x->ps_suspend_enable_flag) {
		ret += ps_enable(si114x);
		si114x->ps_suspend_enable_flag = 0;
	}

	if (ret) {
		dev_err(&si114x->i2c_client->dev, "%s Unable to complete resume\n", __func__);
	} else {
		dev_info(&si114x->i2c_client->dev, "%s Resume completed\n", __func__);
	}
#endif
}

static ssize_t psals_data_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	uint8_t buffer[8];
	struct si114x_data *si114x;
	uint16_t *data = (uint16_t *)buffer;

	si114x = dev_get_drvdata(dev);
	if (!si114x) {
		return -ENODEV;
	}

	buffer[0] = SI411X_ALS_IR_DATA0;
	if (_i2c_read_mult(si114x, buffer, sizeof(buffer))) {
		return -EIO;
	}
        return sprintf(buf, "%u %u %u %u\n", data[0], data[1], data[2], data[3]);
}
static DEVICE_ATTR(psals_data, 0444, psals_data_show, NULL);

static ssize_t led_drive_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	struct si114x_data *si114x;

	si114x = dev_get_drvdata(dev);
	if (!si114x) {
		return -ENODEV;
	}

	return sprintf(buf, "%u %u %u\n", si114x->led[0].drive_strength,
	               si114x->led[1].drive_strength,
	               si114x->led[2].drive_strength);
}

static ssize_t led_drive_store(struct device *dev, struct device_attribute *attr,
                               const char *buf, size_t count)
{
	int i;
	struct si114x_data *si114x;
	unsigned int drive_strength[SI114X_MAX_LED];

	si114x = dev_get_drvdata(dev);

	sscanf(buf, "%d %d %d", &drive_strength[0], &drive_strength[1], &drive_strength[2]);

	/* Validate parameters */
	for (i = 0; i < SI114X_MAX_LED; i++) {
		if (drive_strength[i] > 15) {
			return -EINVAL;
		}
	}

	/* Update local copies of parameters */
	for (i = 0; i < SI114X_MAX_LED; i++) {
		si114x->led[i].drive_strength = drive_strength[i];
	}

	/* Write parameters to device */
	return set_led_drive_strength(si114x);
}

static DEVICE_ATTR(led_drive, 0666, led_drive_show, led_drive_store);

/*
 * These sysfs routines are not used by the Android HAL layer
 * as they are located in the i2c bus device portion of the
 * sysfs tree.
 */
static void sysfs_register_bus_entry(struct si114x_data *si114x, struct i2c_client *client) {
	int rc = 0;

	/* Store the driver data into our private structure */
	dev_set_drvdata(&client->dev, si114x);
	printk(KERN_ERR "%s CMM Storing away data client->dev:%p si:%p\n",
	       __func__, &client->dev, si114x);

	rc += device_create_file(&client->dev, &dev_attr_psals_data);
	rc += device_create_file(&client->dev, &dev_attr_led_drive);

	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}

/*
 * These sysfs routines are exposed to the Android HAL layer as they are
 * created in the class/input portion of the sysfs tree.
 */
static void sysfs_register_class_input_entry_ps(struct si114x_data *si114x, struct device *dev) {
	int rc = 0;
	struct i2c_client *client = si114x->client;

	/* Store the driver data into our private structure */
//	dev_set_drvdata(dev, si114x);
//	input_poll_dev->private = si114x;
	printk(KERN_ERR "%s CMM Storing away data client->dev:%p si:%p\n",
	       __func__, dev, si114x);

	rc += device_create_file(dev, &dev_attr_psals_data);
	rc += device_create_file(dev, &dev_attr_led_drive);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}

static void sysfs_register_class_input_entry_als(struct si114x_data *si114x, struct device *dev) {
	int rc = 0;
	struct i2c_client *client = si114x->client;

	/* Store the driver data into our private structure */
//	dev_set_drvdata(dev, si114x);
	printk(KERN_ERR "%s CMM Storing away data client->dev:%p si:%p\n",
	       __func__, dev, si114x);

	rc += device_create_file(dev, &dev_attr_psals_data);
	rc += device_create_file(dev, &dev_attr_led_drive);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}

int set_measurement_rates(struct si114x_data *si114x) {
	int rc = 0;
	rc += _i2c_write_one(si114x, SI411X_MEAS_RATE, 0x84);
	rc += _i2c_write_one(si114x, SI411X_ALS_RATE, 0x08);
	rc += _i2c_write_one(si114x, SI411X_PS_RATE, 0x08);
	return rc;
}

static int set_led_drive_strength(struct si114x_data *si114x)
{
	int rc = 0;
	rc += _i2c_write_one(si114x, SI411X_PS_LED21, si114x->led[1].drive_strength << 4
	                     | si114x->led[0].drive_strength);
	rc += _i2c_write_one(si114x, SI411X_PS_LED3, si114x->led[2].drive_strength);
	return rc;
}

int _parm_write(struct si114x_data *si114x, int param_type, int param_addr, int param_data) {
	int rc = 0;
	int data = 0;
	uint8_t buffer[3];

	buffer[0] = 0x17;
	buffer[1] = param_data;
	buffer[2] = param_type | (param_addr & CMD_PARAM_ADDR_MASK);

	rc = _i2c_write_mult(si114x, buffer, sizeof(buffer));
	if (!rc) {
		return rc;
	}
	rc = _i2c_read_one(si114x, 0x20, &data);
	if (!rc) {
		return rc;
	}
	printk(KERN_ERR "%s Response value:0x%02x\n", __func__, data);
	return rc;
}

int _cmd_write(struct si114x_data *si114x, int cmd) {
	int data = 0;
	int rc = 0;
	rc = _i2c_write_one(si114x, 0x18, cmd);
	if (rc) {
		return rc;
	}
	rc = _i2c_read_one(si114x, 0x20, &data);
	if (!rc) {
		return rc;
	}
	printk(KERN_ERR "%s Response value:0x%02x\n", __func__, data);
	return rc;
}

int enable_channels(struct si114x_data *si114x) {
#if 0
  buffer[0] = 0x17;
  buffer[1] = 0x27; // enable 3 proximity channels + ambient IR
  buffer[2] = 0xA1; // set parameter 1
  I2CSendData(ALSPROX_I2C_ADDRESS, buffer, 3, 1);
#endif
	return _parm_write(si114x, CMD_PARAM_SET, PARAM_I2C_CHLIST, 0x27);
}

int turn_on_leds(struct si114x_data *si114x) {
	int rc = 0;
#if 0
  buffer[0] = 0x17;
  buffer[1] = 0x21; // turn on LEDs 1, 2 individually
  buffer[2] = 0xA2; // set parameter 2
  I2CSendData(ALSPROX_I2C_ADDRESS, buffer, 3, 1);
#endif
	rc += _parm_write(si114x, CMD_PARAM_SET, PARAM_PSLED12_SELECT, 0x21);
#if 0
  buffer[0] = 0x17;
  buffer[1] = 0x04; // turn on LED 3 individually
  buffer[2] = 0xA3; // set parameter 3
  I2CSendData(ALSPROX_I2C_ADDRESS, buffer, 3, 1);
#endif
	rc += _parm_write(si114x, CMD_PARAM_SET, PARAM_PSLED3_SELECT, 0x04);
	return rc;
}

int start_measurements(struct si114x_data *si114x) {
	return _cmd_write(si114x, CMD_PSALS_AUTO);
}

static int _check_part_id(struct si114x_data *si114x)
{
	int rc = 0;
	struct i2c_client *client = si114x->client;
	int data;

	rc = _i2c_read_one(si114x, SI411X_PART_ID, &data);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Unable to read part identifier\n",
		        __func__);
		return -EIO;
	}

	switch (data) {
		case PART_ID_SI1141:
			dev_err(&client->dev, "%s: Detected SI1141 part_id:%02x\n", __func__, data);
			break;

		case PART_ID_SI1142:
			dev_err(&client->dev, "%s: Detected SI1142 part_id:%02x\n", __func__, data);
			break;

		case PART_ID_SI1143:
			dev_err(&client->dev, "%s: Detected SI1143 part_id:%02x\n", __func__, data);
			break;

		default:
			dev_err(&client->dev, "%s: Unable to determine SI114x part_id:%02x\n", __func__, data);
			rc = -ENODEV;
	}
	return rc;
}

static int si114x_setup(struct si114x_data *si114x) {
	int rc = 0;

	/* Set defaults */
	si114x->led[0].enable = 1;
	si114x->led[1].enable = 1;
	si114x->led[2].enable = 1;

	si114x->led[0].drive_strength = 6;
	si114x->led[1].drive_strength = 6;
	si114x->led[2].drive_strength = 6;

	/* Set the proper operating hardware key */
	_i2c_write_one(si114x, SI411X_HW_KEY, HW_KEY);

	set_measurement_rates(si114x);

	enable_channels(si114x);

	turn_on_leds(si114x);

	start_measurements(si114x);

	return rc;
}

static int  __devinit si114x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct si114x_data *si114x = NULL;
	si114x = kzalloc(sizeof(struct si114x_data), GFP_KERNEL);
	if (!si114x)
	{
		dev_err(&client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	si114x->client = client;
	dev_err(&client->dev, "%s: Probing si114x..\n", __func__);

	if (_check_part_id(si114x)) {
		goto err_out;
	}

	/* Reset the device */
	_cmd_write(si114x, CMD_RESET);

	/* Register the sysfs inodes */
	sysfs_register_bus_entry(si114x, client);

	if (use_interrupts) {
		/* Setup the input subsystem for the ALS */
		if (setup_als_input(si114x)) {
			dev_err(&client->dev,"%s: Unable to allocate als input resource\n", __func__);
			goto err_out;
		}

		/* Setup the input subsystem for the PS */
		if (setup_ps_input(si114x)) {
			dev_err(&client->dev, "%s: Unable to allocate ps input resource\n", __func__);
			goto err_out;
		}
		//sysfs_register_class_input_entry_als(si114x, &si114x->input_dev_als.input->dev);
		// sysfs_register_class_input_entry_ps(si114x, &si114x->input_dev_ps.input->dev);
	} else {
		/* Setup the input subsystem for the ALS */
		if (setup_als_polled_input(si114x)) {
			dev_err(&client->dev,"%s: Unable to allocate als polled input resource\n", __func__);
			goto err_out;
		}

		/* Setup the input subsystem for the PS */
		if (setup_ps_polled_input(si114x)) {
			dev_err(&client->dev, "%s: Unable to allocate ps polled input resource\n", __func__);
			goto err_out;
		}
		// sysfs_register_class_input_entry_als(si114x, &si114x->input_dev_als.input_poll->input->dev);
		// sysfs_register_class_input_entry_ps(si114x, &si114x->input_dev_ps.input_poll->input->dev);
	}

	/* Setup the suspend and resume functionality */
	INIT_LIST_HEAD(&si114x->early_suspend.link);
	si114x->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	si114x->early_suspend.suspend = si114x_early_suspend;
	si114x->early_suspend.resume = si114x_late_resume;
	register_early_suspend(&si114x->early_suspend);

	if (si114x_setup(si114x)) {
		goto err_out;
	}

	/* Store the driver data into our private structure */
	dev_set_drvdata(&client->dev, si114x);
	printk(KERN_ERR "%s CMM Storing away data client->dev:%p si:%p\n",
	       __func__, &client->dev, si114x);
	return 0;

err_out:
	kfree(si114x);
	return -ENODEV;
}

static const struct i2c_device_id si114x_id[] = {
	{ DEVICE_NAME, 0 },
};

static struct i2c_driver si114x_driver = {
	.probe = si114x_probe,
	.id_table = si114x_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};

static int __init si114x_init(void)
{
	return i2c_add_driver(&si114x_driver);
}

static void __exit si114x_exit(void)
{
	i2c_del_driver(&si114x_driver);
}

module_init(si114x_init);
module_exit(si114x_exit);

MODULE_AUTHOR("cmanton@google.com");
MODULE_DESCRIPTION("SI114x Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
