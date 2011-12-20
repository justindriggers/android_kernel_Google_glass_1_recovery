/* Lite-On LTR-506ALS Android Driver
 *
 * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/i2c/ltr506als.h>

#define DRIVER_VERSION "1.0"
#define PARTID 0x90
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "ltr506als"

struct ltr506_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *workqueue;
	struct early_suspend early_suspend;
	struct wake_lock ps_wake_lock;

	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	int mode;

	/* ALS */
	int als_enable_flag;
	int als_suspend_enable_flag;
	int als_irq_flag;
	int als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;
	uint8_t als_resolution:3;
	uint8_t als_meas_rate:3;
	/* Flag to suspend ALS on suspend or not */
	int disable_als_on_suspend;
	int als_filter_interrupts;

	/* PS */
	int ps_enable_flag;
	int ps_suspend_enable_flag;
	int ps_irq_flag;
	int ps_opened;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;
	uint8_t ps_meas_rate:3;
	/* Flag to suspend PS on suspend or not */
	int disable_ps_on_suspend;
	int ps_filter_interrupts;

	/* LED */
	uint8_t led_pulse_freq:3;
	uint8_t led_duty_cyc:2;
	uint8_t led_peak_curr:3;
	uint8_t led_pulse_count;

	/* Interrupt */
	int irq;
	int gpio_int_no;
	int is_suspend;
};

struct ltr506_data *sensor_info;

/* I2C Read */
static int I2C_Read(char *rxData,
                    int length)
{
	int index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}
	return 0;
}

/* I2C Write */
static int I2C_Write(char *txData, int length)
{
	int index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}
	return 0;
}

/* Set register bit */
static int _ltr506_set_bit(struct i2c_client *client, u8 set, u8 cmd, u8 data)
{
	char buffer[2];
	u8 value;
	int ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

/* Set register field */
static int _ltr506_set_field(struct i2c_client *client, u8 mask, u8 cmd, u8 data)
{
	char buffer[2];
	u8 value;
	int ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	value = buffer[0] & ~(mask);
	value |= data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

/* Read ADC Value */
static uint16_t read_adc_value(struct ltr506_data *ltr506)
{
	int ret;
	uint16_t value;
	char buffer[2];

	switch (ltr506->mode) {
		case 0 :
			/* ALS */
			buffer[0] = LTR506_ALS_DATA_0;
			break;

		case 1 :
			/* PS */
			buffer[0] = LTR506_PS_DATA_0;
			break;
	}

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, sizeof(buffer));
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	dev_dbg(&ltr506->i2c_client->dev, "%s | mode-%d(0=als, 1=ps) : value = 0x%04X\n", __func__, ltr506->mode, value);

	switch (ltr506->mode) {
		case 0 :
			/* ALS */
			/* NOTE: Currently data structure is 16 bits so this
			 * check is here for completeness.
			 */
			if (value > ALS_MAX_MEASURE_VAL) {
				dev_err(&ltr506->i2c_client->dev,
				        "%s: PS Value Error: 0x%X\n", __func__,
				        value);
			}
			value &= ALS_VALID_MEASURE_MASK;
			break;

		case 1 :
			/* PS */
			if (value > PS_MAX_MEASURE_VAL) {
				dev_err(&ltr506->i2c_client->dev,
				        "%s: PS Value Error: 0x%X\n", __func__,
				        value);
			}
			value &= PS_VALID_MEASURE_MASK;
			break;
	}
	return value;
}

/* Set ALS range */
static int set_als_range(uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	buffer[0] = LTR506_ALS_THRES_UP_0;
	buffer[1] = ht & 0xFF;
	buffer[2] = (ht >> 8) & 0xFF;
	buffer[3] = lt & 0xFF;
	buffer[4] = (lt >> 8) & 0xFF;

	ret = I2C_Write(buffer, 5);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set als range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);
	return ret;
}

/* Set PS range */
static int set_ps_range(uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	buffer[0] = LTR506_PS_THRES_UP_0;
	buffer[1] = ht & 0xFF;
	buffer[2] = (ht >> 8) & 0x0F;
	buffer[3] = lt & 0xFF;
	buffer[4] = (lt >> 8) & 0x0F;

	ret = I2C_Write(buffer, 5);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set ps range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);
	return ret;
}

/* Report PS input event */
static void report_ps_input_event(struct ltr506_data *ltr506)
{
	int rc;
	uint16_t adc_value;
	int thresh_hi, thresh_lo, thresh_delta;

	ltr506->mode = 1;
	adc_value = read_adc_value(ltr506);

	input_report_abs(ltr506->ps_input_dev, ABS_DISTANCE, adc_value);
	input_sync(ltr506->ps_input_dev);

	if (!ltr506->ps_filter_interrupts) {
		return;
	}

	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	thresh_delta = (adc_value >> 10)+2;
	thresh_lo = adc_value - thresh_delta;
	thresh_hi = adc_value + thresh_delta;
	if (thresh_lo < PS_MIN_MEASURE_VAL)
		thresh_lo = PS_MIN_MEASURE_VAL;
	if (thresh_hi > PS_MAX_MEASURE_VAL)
		thresh_hi = PS_MAX_MEASURE_VAL;
	rc = set_ps_range((uint16_t)thresh_lo, (uint16_t)thresh_hi);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
	}
}

/* Report ALS input event and select range */
static void report_als_input_event(struct ltr506_data *ltr506)
{
	int rc;
	uint16_t adc_value;
	int thresh_hi, thresh_lo, thresh_delta;

	ltr506->mode = 0;
	adc_value = read_adc_value(ltr506);

	input_report_abs(ltr506->als_input_dev, ABS_MISC, adc_value);
	input_sync(ltr506->als_input_dev);

	if (!ltr506->als_filter_interrupts) {
		return;
	}
	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	thresh_delta = (adc_value >> 12)+2;
	thresh_lo = adc_value - thresh_delta;
	thresh_hi = adc_value + thresh_delta;
	if (thresh_lo < ALS_MIN_MEASURE_VAL)
		thresh_lo = ALS_MIN_MEASURE_VAL;
	if (thresh_hi > ALS_MAX_MEASURE_VAL)
		thresh_hi = ALS_MAX_MEASURE_VAL;
	rc = set_als_range((uint16_t)thresh_lo, (uint16_t)thresh_hi);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
	}
}

/* Work when interrupt */
static void ltr506_schedwork(struct work_struct *work)
{
	int ret;
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	buffer[0] = LTR506_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x0a;
	newdata = status & 0x05;

	if (!interrupt_stat) {
		/* There was an interrupt with no work to do */
		int i;
		u8 buf[40];
		dev_info(&ltr506->i2c_client->dev,"%s Unexpected received"
			 " interrupt with no work to do status:0x%02x\n",
			 __func__, status);
		buf[0] = 0x80;
		I2C_Read(buf, sizeof(buf));
		for (i = 0; i < sizeof(buf); i++) {
			dev_info(&ltr506->i2c_client->dev, "%s reg:0x%02x"
				 " val:0x%02x\n", __func__, 0x80+i, buf[i]);
		}
	} else {
		if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
			ltr506->ps_irq_flag = 1;
			report_ps_input_event(ltr506);
		}
		if ((interrupt_stat & 0x08) && (newdata & 0x04)) {
			ltr506->als_irq_flag = 1;
			report_als_input_event(ltr506);
		}
	}
	enable_irq(ltr506->irq);
}


static DECLARE_WORK(irq_workqueue, ltr506_schedwork);

/* IRQ Handler */
static irqreturn_t ltr506_irq_handler(int irq, void *data)
{
	struct ltr506_data *ltr506 = data;

	/* disable an irq without waiting */
	disable_irq_nosync(ltr506->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}

static int ltr506_gpio_irq(struct ltr506_data *ltr506)
{
	int rc = 0;

	rc = gpio_request(ltr506->gpio_int_no, DEVICE_NAME);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: GPIO %d Request Fail"
		        " (%d)\n", __func__, ltr506->gpio_int_no, rc);
		return rc;
	}

	rc = gpio_direction_input(ltr506->gpio_int_no);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Set GPIO %d as Input"
		        " Fail (%d)\n", __func__, ltr506->gpio_int_no, rc);
		goto out1;
	}

	/* Configure an active low trigger interrupt for the device */
	rc = request_irq(ltr506->irq, ltr506_irq_handler, IRQF_TRIGGER_LOW,
	                 DEVICE_NAME, ltr506);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Request IRQ (%d) for"
		        " GPIO %d Fail (%d)\n", __func__, ltr506->irq,
		        ltr506->gpio_int_no, rc);
		goto out1;
	}

	return rc;
out1:
	gpio_free(ltr506->gpio_int_no);

	return rc;
}

/* LED Setup */
static int ps_led_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[3];

	buffer[0] = LTR506_PS_LED;
	buffer[1] = (ltr506->led_pulse_freq << LED_PULSE_FREQ_SHIFT)
		| (ltr506->led_duty_cyc << LED_DUTY_CYC_SHIFT)
		| (ltr506->led_peak_curr << LED_PEAK_CURR_SHIFT);
	buffer[2] = ltr506->led_pulse_count;
	ret = I2C_Write(buffer, 3);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
	}

	return ret;

}

static int ps_meas_rate_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[2];

	buffer[0] = LTR506_PS_MEAS_RATE;
	buffer[1] = ltr506->ps_meas_rate;
	ret = I2C_Write(buffer, 2);
	if (ret < 0)
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

	return ret;
}

static int als_meas_rate_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[2];

	buffer[0] = LTR506_ALS_MEAS_RATE;
	buffer[1] = (ltr506->als_resolution << ADC_RESOLUTION_SHIFT)
		| (ltr506->als_meas_rate << ALS_MEAS_RATE_SHIFT);
	ret = I2C_Write(buffer, 2);
	if (ret < 0)
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

	return ret;
}

/* PS Enable */
static int ps_enable(struct ltr506_data *ltr506)
{
	int rc = 0;

	if (ltr506->ps_enable_flag) {
		dev_info(&ltr506->i2c_client->dev, "%s: already enabled\n", __func__);
		return 0;
	}

	/* Set thresholds where interrupt will *not* be generated */
	rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	/* Allows this interrupt to wake the system */
	rc = irq_set_irq_wake(ltr506->irq, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: IRQ-%d WakeUp Enable Fail...\n", __func__, ltr506->irq);
		return rc;
	}

	/* TODO(cmanton) Get values from platform options */
	ltr506->led_pulse_freq = 7;
	ltr506->led_duty_cyc = 1;
	ltr506->led_peak_curr = 0;
	ltr506->led_pulse_count = 127;

	rc = ps_led_setup(ltr506);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

	/* TODO(cmanton) Get values from platform options */
	ltr506->ps_meas_rate = 0x4;
	rc = ps_meas_rate_setup(ltr506);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_PS_CONTR, PS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	dev_info(&ltr506->i2c_client->dev, "%s Turned on proximity sensor\n", __func__);
	ltr506->ps_enable_flag = 1;
	return rc;
}

/* PS Disable */
static int ps_disable(struct ltr506_data *ltr506)
{
	int rc = 0;

	if (ltr506->ps_enable_flag == 0) {
		dev_info(&ltr506->i2c_client->dev, "%s: already disabled\n", __func__);
		return 0;
	}

	/* Don't allow this interrupt to wake the system anymore */
	rc = irq_set_irq_wake(ltr506->irq, 0);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: IRQ-%d WakeUp Disable Fail...\n", __func__, ltr506->irq);
		return rc;
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, CLR_BIT, LTR506_PS_CONTR, PS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	ltr506->ps_enable_flag = 0;
	return rc;
}

/* PS open fops */
static int ps_open(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	if (ltr506->ps_opened)
		return -EBUSY;

	ltr506->ps_opened = 1;

	return 0;
}

/* PS release fops */
static int ps_release(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	ltr506->ps_opened = 0;

	return ps_disable(ltr506);
}

/* PS IOCTL */
static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	struct ltr506_data *ltr506 = sensor_info;

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LTR506_IOCTL_PS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val)
				return ps_enable(ltr506);
			else
				return ps_disable(ltr506);
			break;
		case LTR506_IOCTL_PS_GET_ENABLED:
			return put_user(ltr506->ps_enable_flag, (unsigned long __user *)arg);
			break;
		default:
			pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			return -EINVAL;
	}
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
	.unlocked_ioctl = ps_ioctl
};

struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr506_ps",
	.fops = &ps_fops
};

static int als_enable(struct ltr506_data *ltr506)
{
	int rc = 0;

	/* if device not enabled, enable it */
	if (ltr506->als_enable_flag != 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS already enabled...\n", __func__);
		return rc;
	}
	/* TODO(cmanton) Put this value in the platform setup */
	ltr506->als_meas_rate = 0x3;
	rc = als_meas_rate_setup(ltr506);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	/* Set minimummax thresholds where interrupt will *not* be generated */
	rc = set_als_range(0x0, 0x0);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_ALS_CONTR, ALS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Enable Fail...\n", __func__);
		return rc;
	}
	dev_info(&ltr506->i2c_client->dev, "%s Turned on ambient light sensor\n", __func__);
	ltr506->als_enable_flag = 1;

	return rc;
}

static int als_disable(struct ltr506_data *ltr506)
{
	int rc = 0;
	if (ltr506->als_enable_flag != 1) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS already disabled...\n", __func__);
		return rc;
	}

	rc = _ltr506_set_bit(ltr506->i2c_client, CLR_BIT, LTR506_ALS_CONTR, ALS_MODE);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: ALS Disable Fail...\n", __func__);
		return rc;
	}
	ltr506->als_enable_flag = 0;

	return rc;
}

static int als_open(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;
	int rc = 0;

	if (ltr506->als_opened) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS already Opened...\n", __func__);
		rc = -EBUSY;
	}
	ltr506->als_opened = 1;
	return rc;
}

static int als_release(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	ltr506->als_opened = 0;
	return 0;
}

static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc, val;
	struct ltr506_data *ltr506 = sensor_info;

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LTR506_IOCTL_ALS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			/*pr_info("%s value = %d\n", __func__, val);*/
			rc = val ? als_enable(ltr506) : als_disable(ltr506);
			break;
		case LTR506_IOCTL_ALS_GET_ENABLED:
			val = ltr506->als_enable_flag;
			/*pr_info("%s enabled %d\n", __func__, val);*/
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
	.unlocked_ioctl = als_ioctl
};

static struct miscdevice als_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr506_ls",
	.fops = &als_fops
};

static ssize_t ps_adc_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	ltr506->mode = 1;
	value = read_adc_value(ltr506);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}


static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, NULL);

/* PS LED */
static ssize_t ps_led_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2] = {LTR506_PS_LED, 0};
	int led_pulse_freq, led_duty_cyc, led_peak_curr;

	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = buffer[0];

	led_pulse_freq = (value & LED_PULSE_FREQ) >> LED_PULSE_FREQ_SHIFT;
	led_duty_cyc = (value & LED_DUTY_CYC) >> LED_DUTY_CYC_SHIFT;
	led_peak_curr = (value & LED_PEAK_CURR) >> LED_PEAK_CURR_SHIFT;

	return sprintf(buf, "%d %d %d\n", led_pulse_freq, led_duty_cyc, led_peak_curr);
}

static ssize_t ps_led_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
	int ret = 0;
	struct ltr506_data *ltr506 = sensor_info;
	int led_pulse_freq, led_duty_cyc, led_peak_curr;

	sscanf(buf, "%d %d %d", &led_pulse_freq, &led_duty_cyc, &led_peak_curr);

	if (led_pulse_freq & ~(LED_PULSE_FREQ_BITS)
	    || led_duty_cyc & ~(LED_DUTY_CYC_BITS)
	    || led_peak_curr & ~(LED_PEAK_CURR_BITS)) {
		return -EINVAL;
	}

	ltr506->led_pulse_freq = led_pulse_freq;
	ltr506->led_duty_cyc = led_duty_cyc;
	ltr506->led_peak_curr = led_peak_curr;

	ret = ps_led_setup(ltr506);
	return (ret == 0) ? count : -EIO;
}

static DEVICE_ATTR(ps_led, 0666, ps_led_show, ps_led_store);

static ssize_t als_adc_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	ltr506->mode = 0;
	value = read_adc_value(ltr506);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static DEVICE_ATTR(als_adc, 0666, als_adc_show, NULL);

static ssize_t als_enable_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	buffer[0] = LTR506_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__,
		        buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "%d\n", (value & ALS_MODE) ? 1 : 0);

	return ret;
}

static ssize_t als_enable_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
	int rc = 0;
	int als_en;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_en);
	if ((als_en != 0) && (als_en != 1)) {
		return -EINVAL;
	}

	if (als_en && (ltr506->als_enable_flag == 0)) {
		rc = als_enable(ltr506);
	} else if (!als_en && (ltr506->als_enable_flag == 1)) {
		rc = als_disable(ltr506);
	}
	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_enable, 0666, als_enable_show, als_enable_store);


static ssize_t als_gain_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	int rc;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2];
	int als_gain;

	buffer[0] = LTR506_PS_CONTR;
	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return rc;
	}

	als_gain = (buffer[0] & ALS_GAIN) >> ALS_GAIN_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", als_gain);
}

static ssize_t als_gain_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
	int rc = 0;
	int als_gain;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_gain);
	if (als_gain & ~(ALS_GAIN_BITS)) {
		return -EINVAL;
	}

	rc = _ltr506_set_field(ltr506->i2c_client, ALS_GAIN, LTR506_ALS_CONTR,
	                       als_gain << ALS_GAIN_SHIFT);

	return (rc == 0) ? count : rc;
}
static DEVICE_ATTR(als_gain, 0666, als_gain_show, als_gain_store);

static ssize_t als_resolution_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_ALS_MEAS_RATE, 0 };
	int als_adc_resolution;

	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return rc;
	}

	value = buffer[0];
	als_adc_resolution = (value & ADC_RESOLUTION) >> ADC_RESOLUTION_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", als_adc_resolution);
}

static ssize_t als_resolution_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	struct ltr506_data *ltr506 = sensor_info;
	int als_adc_resolution;

	sscanf(buf, "%d", &als_adc_resolution);
	if (als_adc_resolution & ~(ADC_RESOLUTION_BITS)) {
		return -EINVAL;
	}

	ltr506->als_resolution = als_adc_resolution;

	rc = als_meas_rate_setup(ltr506);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_resolution, 0666, als_resolution_show, als_resolution_store);


static ssize_t als_meas_rate_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2] = { LTR506_ALS_MEAS_RATE, 0 };

	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__,
		        buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "%d\n", ((value & ALS_MEAS_RATE) >> 0));

	return ret;
}

static ssize_t als_meas_rate_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	int rc = 0;
	int als_meas_rate;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_meas_rate);
	if (als_meas_rate & ~(ALS_MEAS_RATE_BITS)) {
		return -EINVAL;
	}

	ltr506->als_meas_rate = als_meas_rate;
	rc = als_meas_rate_setup(ltr506);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(als_meas_rate, 0666, als_meas_rate_show, als_meas_rate_store);

static ssize_t ps_enable_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	buffer[0] = LTR506_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "%d\n", (value & PS_MODE) ? 1 : 0);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	int rc = 0;
	int ps_en;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_en);
	if ((ps_en != 0) && (ps_en != 1)) {
		return -EINVAL;
	}

	if (ps_en && (ltr506->ps_enable_flag == 0)) {
		rc = ps_enable(ltr506);
	} else if (!ps_en && (ltr506->ps_enable_flag == 1)) {
		rc = ps_disable(ltr506);
	}
	return (rc == 0)?count:rc;
}

static DEVICE_ATTR(ps_enable, 0666, ps_enable_show, ps_enable_store);

static ssize_t ps_gain_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	int rc;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2];
	int ps_gain;

	buffer[0] = LTR506_PS_CONTR;
	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return rc;
	}

	ps_gain = (buffer[0] & PS_GAIN) >> PS_GAIN_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", ps_gain);
}

static ssize_t ps_gain_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	int ps_gain;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_gain);
	if (ps_gain & ~(PS_GAIN_BITS)) {
		return -EINVAL;
	}

	rc = _ltr506_set_field(ltr506->i2c_client, PS_GAIN, LTR506_PS_CONTR,
	                       ps_gain << PS_GAIN_SHIFT);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_gain, 0666, ps_gain_show, ps_gain_store);

static ssize_t ps_pulse_cnt_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_PS_N_PULSES, 0 };

	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return rc;
	}

	value = buffer[0];

	return (rc < 0) ? rc : sprintf(buf, "%d\n", value);
}

static ssize_t ps_pulse_cnt_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	int ps_pulse_cnt;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_PS_N_PULSES, 0 };

	sscanf(buf, "%d", &ps_pulse_cnt);
	if (ps_pulse_cnt & ~(0xff)) {
		return -EINVAL;
	}

	buffer[1] = ps_pulse_cnt;
	rc = I2C_Write(buffer, 2);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_pulse_cnt, 0666, ps_pulse_cnt_show, ps_pulse_cnt_store);

static ssize_t ps_meas_rate_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	int rc;
	uint8_t value;
	struct ltr506_data *ltr506 = sensor_info;
	uint8_t buffer[2] = { LTR506_PS_MEAS_RATE, 0 };

	rc = I2C_Read(buffer, 1);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return rc;
	}

	value = (buffer[0] & PS_MEAS_RATE) >> PS_MEAS_RATE_SHIFT;

	return (rc < 0) ? rc : sprintf(buf, "%d\n", value);
}

static ssize_t ps_meas_rate_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc = 0;
	int ps_meas_rate;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_meas_rate);
	if (ps_meas_rate & ~(PS_MEAS_RATE_BITS)) {
		return -EINVAL;
	}

	ltr506->ps_meas_rate = ps_meas_rate;
	rc = ps_meas_rate_setup(ltr506);

	return (rc == 0) ? count : rc;
}

static DEVICE_ATTR(ps_meas_rate, 0666, ps_meas_rate_show, ps_meas_rate_store);


#define HOLE_REG_SPACE(a,b) buffer[0] = a; \
	I2C_Read(buffer, (b+1)-a); \
	for (i = 0; i < (b+1)-a; i++) { \
		buf += sprintf(buf, "0x%02x: 0x%02x\n", i+a, buffer[i]); \
	} \

static ssize_t dump_regs_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
	char *tmp_buf = buf;
	uint8_t buffer[64];
	int i;

	/* There are holes in the address space */
	HOLE_REG_SPACE(0x80, 0x9c);
	HOLE_REG_SPACE(0x9e, 0xa1);
	HOLE_REG_SPACE(0xa4, 0xa4);

	return strlen(tmp_buf);

}
static DEVICE_ATTR(dump_regs, 0666, dump_regs_show, NULL);

static ssize_t status_show(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
	int ret = 0;
	char *tmp_buf = buf;
	uint16_t min, max;
	uint8_t buffer[64];

	buffer[0] = LTR506_ALS_THRES_UP_0;

	ret = I2C_Read(buffer, 5);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return 0;
	}

	max = (u16)buffer[2] | ((u16)buffer[1] << 8);
	min = (u16)buffer[4] | ((u16)buffer[3] << 8);
	buf += sprintf(buf, "als min:%d max:%d\n", min, max);

	buffer[0] = LTR506_PS_THRES_UP_0;

	ret = I2C_Read(buffer, 5);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return 0;
	}

	max = (u16)buffer[2] | ((u16)buffer[1] << 8);
	min = (u16)buffer[4] | ((u16)buffer[3] << 8);
	buf += sprintf(buf, "ps min:%d max:%d\n", min, max);

	return strlen(tmp_buf);

}
static DEVICE_ATTR(status, 0666, status_show, NULL);



static ssize_t als_filter_interrupts_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct ltr506_data *ltr506 = sensor_info;
	return sprintf(buf, "%d\n", ltr506->als_filter_interrupts);
}

static ssize_t als_filter_interrupts_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc;
	int als_filter_interrupts;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &als_filter_interrupts);
	if (als_filter_interrupts != 0 && als_filter_interrupts != 1) {
		return -EINVAL;
	}

	rc = set_als_range(0,0);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
		return -EIO;
	}

	ltr506->als_filter_interrupts = als_filter_interrupts;

	return count;
}

static DEVICE_ATTR(als_filter_interrupts, 0666, als_filter_interrupts_show,
                   als_filter_interrupts_store);

static ssize_t ps_filter_interrupts_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct ltr506_data *ltr506 = sensor_info;
	return sprintf(buf, "%d\n", ltr506->ps_filter_interrupts);
}

static ssize_t ps_filter_interrupts_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
	int rc;
	int ps_filter_interrupts;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &ps_filter_interrupts);
	if (ps_filter_interrupts != 0 && ps_filter_interrupts != 1) {
		return -EINVAL;
	}

	rc = set_ps_range(0,0);
	if (rc < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
		return -EIO;
	}
	ltr506->ps_filter_interrupts = ps_filter_interrupts;

	return count;
}

static DEVICE_ATTR(ps_filter_interrupts, 0666, ps_filter_interrupts_show,
                   ps_filter_interrupts_store);

/*
 * These sysfs routines are not used by the Android HAL layer
 * as they are located in the i2c bus device portion of the
 * sysfs tree.
 */
static void sysfs_register_device(struct i2c_client *client) {
	int rc = 0;

	rc += device_create_file(&client->dev, &dev_attr_als_enable);
	rc += device_create_file(&client->dev, &dev_attr_ps_enable);
	rc += device_create_file(&client->dev, &dev_attr_dump_regs);
	rc += device_create_file(&client->dev, &dev_attr_status);
	rc += device_create_file(&client->dev, &dev_attr_als_filter_interrupts);
	rc += device_create_file(&client->dev, &dev_attr_ps_filter_interrupts);

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
static void sysfs_register_als_device(struct i2c_client *client, struct device *dev) {
	int rc = 0;
	rc += device_create_file(dev, &dev_attr_als_adc);
	rc += device_create_file(dev, &dev_attr_als_resolution);
	rc += device_create_file(dev, &dev_attr_als_enable);
	rc += device_create_file(dev, &dev_attr_als_gain);
	rc += device_create_file(dev, &dev_attr_als_meas_rate);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create als input sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created als input sysfs files\n", __func__);
	}
}

/*
 * These sysfs routines are exposed to the Android HAL layer as they are
 * created in the class/input portion of the sysfs tree.
 */
static void sysfs_register_ps_device(struct i2c_client *client, struct device *dev) {
	int rc = 0;

	rc += device_create_file(dev, &dev_attr_ps_adc);
	rc += device_create_file(dev, &dev_attr_ps_enable);
	rc += device_create_file(dev, &dev_attr_ps_gain);
	rc += device_create_file(dev, &dev_attr_ps_led);
	rc += device_create_file(dev, &dev_attr_ps_meas_rate);
	rc += device_create_file(dev, &dev_attr_ps_pulse_cnt);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create ps input sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created ps input sysfs files\n", __func__);
	}
}


static int als_setup(struct ltr506_data *ltr506)
{
	int ret;

	ltr506->als_input_dev = input_allocate_device();
	if (!ltr506->als_input_dev) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr506->als_input_dev->name = "ltr506_als";
	set_bit(EV_ABS, ltr506->als_input_dev->evbit);
	input_set_abs_params(ltr506->als_input_dev, ABS_MISC, ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr506->als_input_dev);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}

	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Register Misc Device Fail...\n", __func__);
		goto err_als_register_misc_device;
	}

	return ret;

err_als_register_misc_device:
	input_unregister_device(ltr506->als_input_dev);
err_als_register_input_device:
	input_free_device(ltr506->als_input_dev);

	return ret;
}

static int ps_setup(struct ltr506_data *ltr506)
{
	int ret;

	ltr506->ps_input_dev = input_allocate_device();
	if (!ltr506->ps_input_dev) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr506->ps_input_dev->name = "ltr506_ps";
	set_bit(EV_ABS, ltr506->ps_input_dev->evbit);
	input_set_abs_params(ltr506->ps_input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr506->ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Register Misc Device Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(ltr506->ps_input_dev);
err_ps_register_input_device:
	input_free_device(ltr506->ps_input_dev);

	return ret;
}

static int _check_part_id(struct ltr506_data *ltr506)
{
	int ret;
	u8 buffer[2];
	buffer[0] = LTR506_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Read failure :0x%02X",
		        __func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID) {
		dev_err(&ltr506->i2c_client->dev, "%s: Part failure miscompare"
		        " act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}
	return 0;
}

static int ltr506_setup(struct ltr506_data *ltr506)
{
	int ret = 0;

	/* Reset the devices */
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_ALS_CONTR, PS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&ltr506->i2c_client->dev, "%s: Reset ltr506 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr506) < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	ret = ltr506_gpio_irq(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s Requested interrupt\n", __func__);

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s: Set ltr506 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
	ret = _ltr506_set_bit(ltr506->i2c_client, CLR_BIT, LTR506_INTERRUPT, 0x08);
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_INTERRUPT, INTERRUPT_MODE);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s Enabled interrupt to device\n", __func__);

	/* Set ALS measurement gain */
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_ALS_CONTR, 0);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: ALS set gain fail...\n", __func__);
		goto err_out2;
	}

	/* Set PS measurement gain */
	ret = _ltr506_set_bit(ltr506->i2c_client, SET_BIT, LTR506_PS_CONTR, 0);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS set gain fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr506->i2c_client->dev, "%s: Set ltr506 gains\n", __func__);

	/* Turn on ALS and PS */
	ret = als_enable(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to enable ALS", __func__);
		goto err_out2;
	}

	ret = ps_enable(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to enable PS", __func__);
		goto err_out2;
	}

	return ret;

err_out2:
	free_irq(ltr506->irq, ltr506);
	gpio_free(ltr506->gpio_int_no);

err_out1:
	dev_err(&ltr506->i2c_client->dev, "%s Unable to setup device\n", __func__);
	return ret;
}

static void ltr506_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	struct ltr506_data *ltr506 = sensor_info;

	if (ltr506->is_suspend != 0) {
		dev_err(&ltr506->i2c_client->dev, "%s Asked to suspend when already suspended\n", __func__);
		return;
	}
	ltr506->is_suspend = 1;

	/* Save away the state of the devices at suspend point */
	ltr506->als_suspend_enable_flag = ltr506->als_enable_flag;
	ltr506->ps_suspend_enable_flag = ltr506->ps_enable_flag;

	/* Disable the devices for suspend if configured */
	if (ltr506->disable_als_on_suspend && ltr506->als_enable_flag) {
		ret += als_disable(ltr506);
	}
	if (ltr506->disable_ps_on_suspend && ltr506->ps_enable_flag) {
		ret += ps_disable(ltr506);
	}

	if (ret) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to complete suspend\n", __func__);
	} else {
		dev_info(&ltr506->i2c_client->dev, "%s Suspend completed\n", __func__);
	}
}

static void ltr506_late_resume(struct early_suspend *h)
{
	struct ltr506_data *ltr506 = sensor_info;
	int ret = 0;

	if (ltr506->is_suspend != 1) {
		dev_err(&ltr506->i2c_client->dev, "%s Asked to resume when not suspended\n", __func__);
		return;
	}
	ltr506->is_suspend = 0;

	/* If ALS was enbled before suspend, enable during resume */
	if (ltr506->als_suspend_enable_flag) {
		ret += als_enable(ltr506);
		ltr506->als_suspend_enable_flag = 0;
	}

	/* If PS was enbled before suspend, enable during resume */
	if (ltr506->ps_suspend_enable_flag) {
		ret += ps_enable(ltr506);
		ltr506->ps_suspend_enable_flag = 0;
	}

	if (ret) {
		dev_err(&ltr506->i2c_client->dev, "%s Unable to complete resume\n", __func__);
	} else {
		dev_info(&ltr506->i2c_client->dev, "%s Resume completed\n", __func__);
	}
}

static int  __devinit ltr506_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr506_data *ltr506;
	struct ltr506_platform_data *platdata;

	ltr506 = kzalloc(sizeof(struct ltr506_data), GFP_KERNEL);
	if (!ltr506)
	{
		dev_err(&ltr506->i2c_client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	sensor_info = ltr506;

	/* Set initial defaults */
	ltr506->als_enable_flag = 0;
	ltr506->ps_enable_flag = 0;

	/* Typically we want to restrict interrupts unless the sensor
	 * value has exceeded some envelope.
	 */
	ltr506->als_filter_interrupts = 1;
	ltr506->ps_filter_interrupts = 1;

	ltr506->i2c_client = client;
	ltr506->irq = client->irq;

	i2c_set_clientdata(client, ltr506);

	/* Parse the platform data */
	platdata = client->dev.platform_data;
	if (!platdata) {
		dev_err(&ltr506->i2c_client->dev, "%s: Platform Data assign Fail...\n", __func__);
		ret = -EBUSY;
		goto err_out;
	}

	ltr506->gpio_int_no = platdata->pfd_gpio_int_no;
	ltr506->adc_levels = platdata->pfd_levels;
	ltr506->default_ps_lowthresh = platdata->pfd_ps_lowthresh;
	ltr506->default_ps_highthresh = platdata->pfd_ps_highthresh;

	/* Configuration to set or disable devices upon suspend */
	ltr506->disable_als_on_suspend = platdata->pfd_disable_als_on_suspend;
	ltr506->disable_ps_on_suspend = platdata->pfd_disable_ps_on_suspend;

	if (_check_part_id(ltr506) < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev,"%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the PS */
	ret = ps_setup(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: PS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Create the workqueue for the interrup handler */
	ltr506->workqueue = create_singlethread_workqueue("ltr506_workqueue");
	if (!ltr506->workqueue) {
		dev_err(&ltr506->i2c_client->dev, "%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	/* Wake lock option for promity sensor */
	wake_lock_init(&(ltr506->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	/* Setup and configure both the ALS and PS on the ltr506 device */
	ret = ltr506_setup(ltr506);
	if (ret < 0) {
		dev_err(&ltr506->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto err_ltr506_setup;
	}

	/* Setup the suspend and resume functionality */
	INIT_LIST_HEAD(&ltr506->early_suspend.link);
	ltr506->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ltr506->early_suspend.suspend = ltr506_early_suspend;
	ltr506->early_suspend.resume = ltr506_late_resume;
	register_early_suspend(&ltr506->early_suspend);

	/* Register the sysfs files */
	sysfs_register_device(client);
	sysfs_register_als_device(client, &ltr506->als_input_dev->dev);
	sysfs_register_ps_device(client, &ltr506->ps_input_dev->dev);

	dev_dbg(&ltr506->i2c_client->dev, "%s: probe complete\n", __func__);
	return ret;

err_ltr506_setup:
	destroy_workqueue(ltr506->workqueue);
err_out:
	kfree(ltr506);
	return ret;
}

static const struct i2c_device_id ltr506_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};


static struct i2c_driver ltr506_driver = {
	.probe = ltr506_probe,
	.id_table = ltr506_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};

static int __init ltr506_init(void)
{
	return i2c_add_driver(&ltr506_driver);
}


static void __exit ltr506_exit(void)
{
	i2c_del_driver(&ltr506_driver);
}


	module_init(ltr506_init)
module_exit(ltr506_exit)

	MODULE_AUTHOR("Lite-On Technology Corp");
	MODULE_DESCRIPTION("LTR-506ALS Driver");
	MODULE_LICENSE("Dual BSD/GPL");
	MODULE_VERSION(DRIVER_VERSION);
