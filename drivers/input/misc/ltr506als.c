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
	int als_irq_flag;
	int als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;

	/* PS */
	int ps_enable_flag;
	int ps_irq_flag;
	int ps_opened;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;
	
	/* Interrupt */
	int irq;
	int gpio_int_no;
	int is_suspend;
};

struct ltr506_data *sensor_info;

/* I2C Read */
static int I2C_Read(char *rxData, int length)
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
		pr_alert("%s I2C Write Fail !!!!\n",	__func__);
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

	pr_info("%s\n", __func__);

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
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
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

/* Read ADC Value */
static uint16_t read_adc_value(struct ltr506_data *ltr506)
{
	uint16_t value, tmp_value, ret;
	char buffer[2];

	pr_info("%s\n", __func__);

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

	/* read 2 bytes from data regs */
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	tmp_value = buffer[1];
	pr_info("%s | mode-%d(0=als, 1=ps) : value = 0x%02X, tmp_value = 0x%02X\n", __func__, ltr506->mode, value, tmp_value);
		

	switch (ltr506->mode) {
		case 0 :
			/* ALS */
			value = value | tmp_value << 8;
			if (value > 0xFFFF) {
				pr_err("%s: ALS Value Error: 0x%X\n", __func__, value);
				return -1;
			}
		
			value = value & 0xFFFF;
			break;

		case 1 :
			/* PS */
			value = value | ((tmp_value & 0x0F) << 8);
			if (value > 0x0FFF) {
				pr_err("%s: PS Value Error: 0x%X\n", __func__, value);
				return -1;
			}
			value = value & 0x0FFF;
			break;
	}
	return value;
}

/* Set ALS range */
static int set_als_range(uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	pr_info("%s\n", __func__);

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
	
	return ret;
}

/* Set PS range */
static int set_ps_range(uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	pr_info("%s\n", __func__);

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
	
	return ret;
}

/* Report PS input event */
static void report_ps_input_event(struct ltr506_data *ltr506)
{
	uint16_t val;

	pr_info("%s\n", __func__);

	ltr506->mode = 1;
	val = read_adc_value(ltr506);

	input_report_abs(ltr506->ps_input_dev, ABS_DISTANCE, val);
	input_sync(ltr506->ps_input_dev);

	wake_lock_timeout(&(ltr506->ps_wake_lock), 2*HZ);
}

/* Report ALS input event and select range */
static void report_als_input_event(struct ltr506_data *ltr506)
{
	uint16_t adc_value;
	int level = 0, i, ret;

	pr_info("%s\n", __func__);

	ltr506->mode = 0;
	adc_value = read_adc_value(ltr506);

	for (i = 0; i < 5; i++) {
		if (adc_value <= (*(ltr506->adc_levels + i))) {
			level = i;
			if (*(ltr506->adc_levels + i))
				break;
		}
	}

	/* condition ? value if true : value if false */
	ret = set_als_range((i == 0) ? 0 :
			*(ltr506->adc_levels + (i - 1)) + 1,
		*(ltr506->adc_levels + i));
	if (ret < 0)
		pr_err("%s ALS Set Range Fail...\n", __func__);

	input_report_abs(ltr506->als_input_dev, ABS_MISC, level);
	input_sync(ltr506->als_input_dev);

}

/* Resume ALS operation */
static int als_resume_enable(struct ltr506_data *ltr506)
{
	int ret = -1;

	pr_info("%s\n", __func__);

	ret = _ltr506_set_bit(ltr506->i2c_client, 1, LTR506_ALS_CONTR, ALS_MODE);
	if (ret < 0) {
		pr_err("%s: ALS Resume Enable Fail...\n", __func__);
		return ret;
	}

	input_report_abs(ltr506->als_input_dev, ABS_MISC, -1);
	input_sync(ltr506->als_input_dev);

	report_als_input_event(ltr506);

	return ret;
}

/* Work when interrupt */
static void ltr506_schedwork(struct work_struct *work)
{
	uint8_t status, mode_als, mode_ps;
	uint8_t	interrupt_stat, newdata, ret;
	struct ltr506_data *ltr506 = sensor_info;
	char buffer[2];

	int value1;
	int value2;
	int retry_limit = 5;

	pr_info("%s\n", __func__);

	buffer[0] = LTR506_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];

	interrupt_stat = status & 10;
	newdata = status & 5;

	buffer[0] = LTR506_ALS_CONTR;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	mode_als = buffer[0];
	mode_ps = buffer[1];

	pr_info("Interrupt: %d", interrupt_stat);

	switch (interrupt_stat){
		case 0:
			/* No interrupt, continue to check */
			do {
				value1 = gpio_get_value(ltr506->gpio_int_no);
				irq_set_irq_type(ltr506->irq, value1 ?
					IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
				value2 = gpio_get_value(ltr506->gpio_int_no);
			} while ((value1 != value2) && (retry_limit-- > 0));
			break;

		case 2:
			/* PS interrupt */
			if (mode_ps & 0x02) {
				if ((newdata == 1) | (newdata == 5)) {
					ltr506->ps_irq_flag = 1;
					report_ps_input_event(ltr506);
				}
			}
			break;

		case 8:
			/* ALS interrupt */
			if (mode_als & 0x02) {
				if ((newdata == 4) | (newdata == 5)) {
					ltr506->als_irq_flag = 1;
					report_als_input_event(ltr506);
				}
			}
			break;

		case 10:
			/* Both interrupt */
			if (mode_ps & 0x02) {
				if ((newdata == 1) | (newdata == 5)) {
					ltr506->ps_irq_flag = 1;
					report_ps_input_event(ltr506);
				}
			}

			if (mode_als & 0x02) {
				if ((newdata == 4) | (newdata == 5)) {
					ltr506->als_irq_flag = 1;
					report_als_input_event(ltr506);
				}
			}
			break;
	}

	enable_irq(ltr506->irq);
}

static DECLARE_WORK(irq_workqueue, ltr506_schedwork);

/* IRQ Handler */
static irqreturn_t ltr506_irq_handler(int irq, void *data)
{
	struct ltr506_data *ltr506 = data;

	pr_info("%s\n", __func__);

	/* disable an irq without waiting */
	disable_irq_nosync(ltr506->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}

static int ltr506_gpio_irq(struct ltr506_data *ltr506)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = gpio_request(ltr506->gpio_int_no, DEVICE_NAME);
	if (ret < 0) {
		pr_err("%s: GPIO %d Request Fail (%d)\n", __func__, ltr506->gpio_int_no, ret);
		return ret;
	}

	ret = gpio_direction_input(ltr506->gpio_int_no);
	if (ret < 0) {
		pr_err("%s: Set GPIO %d as Input Fail (%d)\n", __func__, ltr506->gpio_int_no, ret);
		return ret;
	}

	ret = request_irq(ltr506->irq, ltr506_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ltr506);
	if (ret < 0) {
		pr_err("%s: Request IRQ (%d) for GPIO %d Fail (%d)\n", __func__, ltr506->irq, ltr506->gpio_int_no, ret);
		return ret;
	}

	return 0;
}

/* LED Setup */
static int ps_led_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[3];

	pr_info("%s\n", __func__);

	buffer[0] = LTR506_PS_LED;

	/* Default settings used for now. */
	buffer[1] = 0x6B;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
	}

	return ret;

}

static int ps_meas_rate_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[2];

	pr_info("%s\n", __func__);

	buffer[0] = LTR506_PS_MEAS_RATE;
	buffer[1] = 0x02;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) 
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

	return ret;
}

static int als_meas_rate_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[2];

	pr_info("%s\n", __func__);

	buffer[0] = LTR506_ALS_MEAS_RATE;
	buffer[1] = 0x02;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) 
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

	return ret;
}

/* PS Enable */
static int ps_enable(struct ltr506_data *ltr506)
{
	int ret;

	pr_info("%s\n", __func__);
	if (ltr506->ps_enable_flag) {
		pr_info("%s: already enabled\n", __func__);
		return 0;
	}

	/* dummy report */
	input_report_abs(ltr506->ps_input_dev, ABS_DISTANCE, -1);
	input_sync(ltr506->ps_input_dev);

	ret = _ltr506_set_bit(ltr506->i2c_client, 1, LTR506_PS_CONTR, PS_MODE);
	if (ret < 0) {
		pr_err("%s: PS Enable Fail...\n", __func__);
		return ret;
	}

	ret = ps_meas_rate_setup(ltr506);
	if (ret < 0) {
		pr_err("%s: PS MeasRate Setup Fail...\n", __func__);
		return ret;
	}


	msleep(WAKEUP_DELAY);
	report_ps_input_event(ltr506);

	ltr506->ps_enable_flag = 1;

	ret = irq_set_irq_wake(ltr506->irq, 1);
	if (ret < 0) {
		pr_err("%s: IRQ-%d WakeUp Enable Fail...\n", __func__, ltr506->irq);
		return ret;
	}

	return ret;
}

/* PS Disable */
static int ps_disable(struct ltr506_data *ltr506)
{
	int ret = -EIO;

	pr_info("%s\n", __func__);
	if (ltr506->ps_enable_flag == 0) {
		pr_info("%s: already disabled\n", __func__);
		return 0;
	}

	ret = irq_set_irq_wake(ltr506->irq, 0);
	if (ret < 0) {
		pr_err("%s: IRQ-%d WakeUp Disable Fail...\n", __func__, ltr506->irq);
		return ret;
	}

	ret = _ltr506_set_bit(ltr506->i2c_client, 0, LTR506_PS_CONTR, PS_MODE);
	if (ret < 0) {
		pr_err("%s: PS Disable Fail...\n", __func__);
		return ret;
	}

	irq_set_irq_type(ltr506->irq, IRQF_TRIGGER_LOW);

	ltr506->ps_enable_flag = 0;
	return ret;
}

/* PS open fops */ 
static int ps_open(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);

	if (ltr506->ps_opened)
		return -EBUSY;

	ltr506->ps_opened = 1;

	return 0;
}

/* PS release fops */ 
static int ps_release(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);

	ltr506->ps_opened = 0;

	return ps_disable(ltr506);
}

/* PS IOCTL */
static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s cmd %d\n", __func__, _IOC_NR(cmd));

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
	int ret;

	pr_info("%s\n", __func__);

	/* if device not enabled, enable it */
	if (ltr506->als_enable_flag == 0) {
		ret = _ltr506_set_bit(ltr506->i2c_client, 1, LTR506_ALS_CONTR, ALS_MODE);
		if (ret < 0) {
			pr_err("%s: ALS Enable Fail...\n", __func__);
			return ret;
		}

		ret = als_meas_rate_setup(ltr506);
		if (ret < 0) {
			pr_err("%s: ALS MeasRate Setup Fail...\n", __func__);
			return ret;
		}

		ltr506->als_enable_flag = 1;
	}

	if ((ltr506->als_enable_flag == 1) && (ltr506->is_suspend || ltr506->ps_irq_flag == 1)) {
		pr_err("%s: System is already Suspended, Object is NEAR...\n", __func__);
		ltr506->als_enable_flag = 1;
		return 0;
	}
	else if (ltr506->als_enable_flag != 0) {
		/* Dummy */
		msleep(WAKEUP_DELAY);
		input_report_abs(ltr506->als_input_dev, ABS_MISC, -1);
		input_sync(ltr506->als_input_dev);

		report_als_input_event(ltr506);
	}
	return 0;
}

static int als_disable(struct ltr506_data *ltr506)
{
	int ret;

	pr_info("%s\n", __func__);

	if (ltr506->is_suspend) {
		ltr506->als_enable_flag = 0;
		pr_err("%s: System is already Suspended...\n", __func__);
		return 0;
	}
	else {
		ret = irq_set_irq_wake(ltr506->irq, 0);
		if (ret < 0) {
			pr_err("%s: IRQ-%d WakeUp Disable Fail...\n", __func__, ltr506->irq);
			return ret;
		}
		irq_set_irq_type(ltr506->irq, IRQF_TRIGGER_LOW);
	}

	if (ltr506->als_enable_flag) {
		ret = _ltr506_set_bit(ltr506->i2c_client, 0, LTR506_ALS_CONTR, ALS_MODE);
		if (ret < 0) {
			pr_err("%s: ALS Disable Fail...\n", __func__);
			return ret;
		}
		else
			ltr506->als_enable_flag = 0;
	}
	return 0;
}

static int als_open(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;
	int rc = 0;

	pr_info("%s\n", __func__);
	if (ltr506->als_opened) {
		pr_err("%s: ALS already Opened...\n", __func__);
		rc = -EBUSY;
	}
	ltr506->als_opened = 1;
	return rc;
}

static int als_release(struct inode *inode, struct file *file)
{
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);
	ltr506->als_opened = 0;
	return 0;
}

static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc, val;
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s cmd %d\n", __func__, _IOC_NR(cmd));

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
	pr_info("%s\n", __func__);
	return 0;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if ((ps_en != 0) && (ps_en != 1))
		return -EINVAL;

	if (ps_en) {
		ps_enable(ltr506);
	} else {
		ps_disable(ltr506);
	}

	return count;
}

static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, ps_enable_store);

/* PS LED */
static ssize_t ps_led_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char buffer[2];
	uint8_t value, value1;
	int ret = 0;
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);

	buffer[0] = LTR506_PS_LED;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = buffer[0];
	value1 = buffer[1];
	pr_info("%s: PS_LED reg = 0x%x, PS_Pulses reg = 0x%x\n", __func__, value, value1);

	ltr506->led_pulse_freq = (value & 0xE0) >> 5;
	ltr506->led_duty_cyc = (value & 0x18) >> 3;
	ltr506->led_peak_curr = (value & 0x07);
	ltr506->led_pulse_count = value1;

	pr_info("%s: led_pulse_freq = 0x%x, led_duty_cyc = 0x%x, led_peak_curr = 0x%x, led_pulse_count = 0x%x\n", 
		__func__, ltr506->led_pulse_freq, ltr506->led_duty_cyc, ltr506->led_peak_curr, ltr506->led_pulse_count);

	return ret;
}

static ssize_t ps_led_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int param;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	sscanf(buf, "%d", &param);

	pr_info("%s: store value = %d\n", __func__, param);

	if (param == 1) {
		/* Need to create routine for setting LED */
		ret = ps_led_setup(ltr506); 
		if (ret < 0) {
			pr_err("%s : Set LED Fail...\n", __func__);
			return -1;
		}
	} else
		pr_info("%s: PS LED used with Default Settings.\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_led, 0666, ps_led_show, ps_led_store);

static ssize_t als_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret, i, level = -1;
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);

	ltr506->mode = 0;
	value = read_adc_value(ltr506);

	for (i = 0; i < 5; i++) {
		if (value <= (*(ltr506->adc_levels + i))) {
			level = i;
			if (*(ltr506->adc_levels + i))
				break;
		}
	}

	ret = sprintf(buf, "ADC[0x%02X] => level %d\n", value, level);

	return ret;
}

static DEVICE_ATTR(als_adc, 0666, als_adc_show, NULL);

static ssize_t als_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	/*struct ltr506_data *ltr506 = sensor_info;*/
	char buffer[2];

	pr_info("%s\n", __func__);

	buffer[0] = LTR506_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = buffer[0];

	ret = sprintf(buf, "ALS Enable = %d\n",
			(value & 0x02) ? 1 : 0);

	return ret;
}

static ssize_t als_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{

	int als_on;
	int ret;
	struct ltr506_data *ltr506 = sensor_info;

	pr_info("%s\n", __func__);

	als_on = -1;
	sscanf(buf, "%d", &als_on);

	if (als_on)
		ltr506->als_enable_flag = 1;
	else
		ltr506->als_enable_flag = 0;
		

	pr_info("%s: ltr506->als_enable_flag = %d!\n", __func__, ltr506->als_enable_flag);

	ret = _ltr506_set_bit(ltr506->i2c_client, (ltr506->als_enable_flag ? 1 : 0),	LTR506_ALS_CONTR, ALS_MODE);
	if (ret < 0)
		pr_err("%s: ALS Enable Fail...\n", __func__);

	return count;
}

static DEVICE_ATTR(als_on, 0666, als_enable_show, als_enable_store);

static int als_setup(struct ltr506_data *ltr506)
{
	int ret;

	pr_info("%s\n", __func__);

	ltr506->als_input_dev = input_allocate_device();
	if (!ltr506->als_input_dev) {
		pr_err("%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr506->als_input_dev->name = "ltr506_als";
	set_bit(EV_ABS, ltr506->als_input_dev->evbit);
	input_set_abs_params(ltr506->als_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(ltr506->als_input_dev);
	if (ret < 0) {
		pr_err("%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}

	ret = misc_register(&als_misc);
	if (ret < 0) {
		pr_err("%s: ALS Register Misc Device Fail...\n", __func__);
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

	pr_info("%s\n", __func__);

	ltr506->ps_input_dev = input_allocate_device();
	if (!ltr506->ps_input_dev) {
		pr_err("%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr506->ps_input_dev->name = "ltr506_ps";
	set_bit(EV_ABS, ltr506->ps_input_dev->evbit);
	input_set_abs_params(ltr506->ps_input_dev, ABS_DISTANCE, 0, 4095, 0, 0);

	ret = input_register_device(ltr506->ps_input_dev);
	if (ret < 0) {
		pr_err("%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		pr_err("%s: PS Register Misc Device Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

	err_ps_register_misc_device:
		input_unregister_device(ltr506->ps_input_dev);
	err_ps_register_input_device:
		input_free_device(ltr506->ps_input_dev);

	return ret;
}

static int ltr506_setup(struct ltr506_data *ltr506)
{
	int ret = 0;
	char buffer[3];

	pr_info("%s\n", __func__);

	ret = ltr506_gpio_irq(ltr506);
	if (ret < 0) {
		pr_alert("%s: GPIO Request Fail...\n", __func__);
		goto err_out;
	}

	msleep(PON_DELAY);

	/* Turn on ALS and PS */
	ret = als_enable(ltr506);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		goto err_configure;
	}

	ret = ps_enable(ltr506);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		goto err_configure;
	}

	/* Set thresholds */
	ret = set_als_range(0x0, 0x0);
	if (ret < 0) {
		pr_err("%s : ALS Thresholds Write Fail...\n", __func__);
		goto err_configure;
	}

	ltr506->ps_lowthresh = ltr506->default_ps_lowthresh;
	ltr506->ps_highthresh = ltr506->default_ps_highthresh;
	ret = set_ps_range(ltr506->ps_lowthresh, ltr506->ps_highthresh);
	if (ret < 0) {
		pr_err("%s : PS Thresholds Write Fail...\n", __func__);
		goto err_configure;
	}

	/* Set Persists */
	ret = _ltr506_set_bit(ltr506->i2c_client, 4, LTR506_INTERRUPT_PRST, ALS_INT_PRST);
	if (ret < 0) {
		pr_err("%s: ALS Set Persist Fail...\n", __func__);
		goto err_configure;
	}

	ret = _ltr506_set_bit(ltr506->i2c_client, 4, LTR506_INTERRUPT_PRST, PS_INT_PRST);
	if (ret < 0) {
		pr_err("%s: PS Set Persist Fail...\n", __func__);
		goto err_configure;
	}
	return ret;

	err_configure:
		free_irq(ltr506->irq, ltr506);
	err_out:
		gpio_free(ltr506->gpio_int_no);
		return ret;
}

static void ltr506_early_suspend(struct early_suspend *h)
{
	int ret;
	struct ltr506_data *ltr506 = sensor_info;
	pr_info("%s\n", __func__);

	ltr506->is_suspend = 1;

	if (ltr506->als_enable_flag) {
		ret = _ltr506_set_bit(ltr506->i2c_client, 0, LTR506_ALS_CONTR, ALS_MODE);
		if (ret < 0)
			pr_err("%s: Early Suspend Fail...\n", __func__);
		else
			ltr506->als_enable_flag = 0;
	}
}

static void ltr506_late_resume(struct early_suspend *h)
{
	struct ltr506_data *ltr506 = sensor_info;
	int ret;

	pr_info("%s: ltr506->als_enable_flag = %d, ltr506->ps_irq_flag = %d\n",
		__func__, ltr506->als_enable_flag, ltr506->ps_irq_flag);

	ltr506->is_suspend = 0;
	if (ltr506->als_enable_flag && (ltr506->ps_irq_flag == 0)) {
		ret = als_resume_enable(ltr506);
		if (ret < 0)
			pr_err("%s: Late Resume Fail...\n", __func__);
	}
}

static int  __devinit ltr506_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	char buffer[2];
	struct ltr506_data *ltr506;
	struct ltr506_platform_data *platdata;

	pr_info("%s\n", __func__);

	ltr506 = kzalloc(sizeof(struct ltr506_data), GFP_KERNEL);
	if (!ltr506)
	{
		pr_err("%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	ltr506->i2c_client = client;
	platdata = client->dev.platform_data;
	if (!platdata) {
		pr_err("%s: Platform Data assign Fail...\n", __func__);
		ret = -EBUSY;
		goto err_out;
	}

	ltr506->irq = client->irq;

	i2c_set_clientdata(client, ltr506);
	ltr506->gpio_int_no = platdata->pfd_gpio_int_no;
	ltr506->adc_levels = platdata->pfd_levels;
	ltr506->default_ps_lowthresh = platdata->pfd_ps_lowthresh;
	ltr506->default_ps_highthresh = platdata->pfd_ps_highthresh;

	sensor_info = ltr506;

	buffer[0] = LTR506_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		goto err_out;
	}

	if (buffer[0] != PARTID) {
		pr_err("%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	ltr506->als_enable_flag = 0;
	ret = als_setup(ltr506);
	if (ret < 0) {
		pr_err("%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}

	ltr506->ps_enable_flag = 0;
	ret = ps_setup(ltr506);
	if (ret < 0) {
		pr_err("%s: PS Setup Fail...\n", __func__);
		goto err_out;
	}

	ltr506->workqueue = create_singlethread_workqueue("ltr506_workqueue");
	if (!ltr506->workqueue) {
		pr_err("%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	wake_lock_init(&(ltr506->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	ret = ltr506_setup(ltr506);
	if (ret < 0) {
		pr_err("%s: Setup Fail...\n", __func__);
		goto err_ltr506_setup;
	}

	ltr506->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ltr506->early_suspend.suspend = ltr506_early_suspend;
	ltr506->early_suspend.resume = ltr506_late_resume;
	register_early_suspend(&ltr506->early_suspend);

	pr_info("%s: Probe End...\n", __func__);
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
	pr_alert("<<< %s: LTR-506ALS Driver Module LOADED >>>\n", __func__);
	return i2c_add_driver(&ltr506_driver);
}


static void __exit ltr506_exit(void)
{
	i2c_del_driver(&ltr506_driver);
	pr_alert(">>> %s: LTR-506ALS Driver Module REMOVED <<<\n", __func__);
}


module_init(ltr506_init)
module_exit(ltr506_exit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-506ALS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
