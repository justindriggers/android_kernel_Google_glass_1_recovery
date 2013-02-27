/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define FUNCTION_DATA f05_data
#define FUNCTION_NUMBER 0x05

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include "rmi_driver.h"
#include "rmi_f01.h"
#include "rmi_f05.h"

#define F01_RESET_MASK 0x01

/* character device name for fast image transfer (if enabled)
 * the device_register will add another '0' to this, making it
 * "f05rawsensor00"
 */
#define RAW_IMAGE_F05_CHAR_DEVICE_NAME "f05rawsensor0"

/** A note about RMI4 F05 register structure.
 *
 * Function 05 providces image reporting functions that allow direct visibility
 * into image sensing.  Only some devices support this feature.
 *
 * Function 05 image reporting provides direct access to low-level capacitance
 * data before processing and interpretation (for example, to derive finger or
 * palm status) for testing and debugging purposes.
 *
 * Controls for specific modes of operation are not contained within the image
 * reporting function. For example:
 *	Controls for algorithms used in image data processing and
 *	interpretation. Scaling sensitivities.
 *
 * The image data reported by Function 05 is a matrix of signed integer values,
 * corresponding to each intersection in the grid of transmitter and receiver
 * electrodes used for transcapacitive sensing.  It does not map these values
 * to an X, Y coordinate space.  It also reports button capacitances on an
 * additional last row (or rows).
 *
 * Operation in normal reporting modes after or during the use of Function 05
 * is not provided, nand may required a reset command to return the sensor to
 * normal operation.
 */

/**
 * @get_image - Setting this bit requests an image to be captured for reading
 * from F05 data registers.  The host should wait for the resulting interrupt
 * before attempting to read the data.  In order to ensure coherency of the
 * image during the reading process, reporting of a new image is stalled until
 * the get_image bit is again set to 1.
 *
 * @force_zero - Setting this bit requests a new baseline image to be taken
 * causing the delta image to be forced to zero(for all pexels).  The host
 * should wait (poll) for the force_zero bit to clear before attempting to next
 * read for report_data. The next baseline read from report_data is from the
 * frame following the forced zero of the delta image.
 */
struct f05_commands {
	u8 reserved1:2;
	u8 get_image:1;
	u8 reserved2:2;
	u8 force_zero:1;
	u8 reserved3:2;
} __attribute__((__packed__));

/**
 * @num_of_rx_electrodes - This 6-bit field reports the number of sensor
 * electrodes available in the design.
 * @num_of_tx_electrodes - This 6-bit field reports the number of drive
 * electrodes available in the design.
 * @has16_bit_delta - This bit field reports the size of the delta image when
 * report_mode = 2.
 *	if it is 0, a 1-byte delta image will be reported.
 *	if it is 1, a 2-byte delta iamge will be reported.
 * @izeOfF05ImageWindow - This field indicates the number of register bytes
 * mapped in f05 image data registers - f05_analog_data2.  The value is
 * 2 x num_of_rx_electrodes.
 */
struct f05_device_queries {
	/* query0 */
	u8 num_of_rx_electrodes:6;
	u8 reserved0:2;
	/* query1 */
	u8 num_of_tx_electrodes:6;
	u8 reserved1:2;
	/* query2 */
	u8 reserved2:8;
	/* query3 */
	u8 reserved3:7;
	u8 has16_bit_delta:1;
	/* query4 */
	u8 size_f05_image_window:8;
} __attribute__((__packed__));

/**
 * @no_auto_cal - When set, this bit prevents normal AutoCalibration.
 * AutoCalibration allows the sensor to adjust its operation to veriations
 * in temperature and environmental conditions and should not normally be
 * disabled.
 */
struct f05_device_controls {
	u8 reverved1:4;
	u8 no_auto_cal:1;
	u8 reverved2:3;
} __attribute__((__packed__));

/**
 * @report_index - the Baseline Image Line (report_mode=1) field specifies
 * the line number (transmitter electrode number, 0
 * through num_of_tx_electrodes) of the image data.
 * @reprotMode - Specifies the report mode for the data in the report_data registers.
 */
struct f05_device_data {
	u8 reserved;
	u8 report_index:6;
	u8 report_mode:2;
} __attribute__((__packed__));

/**
 * @get_image - Setting this bit request an image to be capture for reeading
 * from f05 data registers.  The host should wait for the resulting interrupt
 * before attempting to read the data.  In order to ensure coherency of the
 * image during the reading process, reporting of a new image is stalled until
 * the get_image bit is again set to '1'.
 * @force_zero - setting this bit requests a new baseline image to be taken
 * causing the delta image to be forced to zero(for all pixels). The host
 * should wait (poll) for the force_zero bit to clear before attempting to next
 * read from report_data.  The next baseline read from
 * the frame following the forced zero of the delta image.
 */
struct f05_device_commands {
	u8 reset1:2;
	u8 get_image:1;
	u8 reserved1:2;
	u8 force_zero:1;
	u8 reserved2:2;
};

struct f05_raw_data_char_dev {
	/* mutex for file operation*/
	struct mutex mutex_file_op;
	/* main char dev structure */
	struct cdev raw_data_dev;
	struct class *raw_data_device_class;
};

/**
 *
 * @f05_device_queries - F05 device specific query registers.
 * @f05_device_controls - F05 device specific control registers.
 * @f05_device_data - F05 device specific data registers.
 * @f05_device_commands - F05 device specific command registers.
 * @report_data - This replicated register contains on line of the image.
 * Each 1 - or 2 -byte value contains the baseline or delta image pixel data
 * for the corresponsding receiver number and the transmitter selected by
 * report_index.
 */
struct f05_data {
	struct f05_device_queries dev_query;
	struct f05_device_controls dev_control;
	struct f05_device_data dev_data;
	struct f05_device_commands dev_command;
	u8 *report_data;
	u8 *tmp_buffer;
	u8 report_size;
	u8 max_report_size;
	struct mutex status_mutex;
	struct mutex data_mutex;
	signed char status;
	struct f05_raw_data_char_dev raw_data_feed;
	struct rmi_function_container *fn_dev;
	int data_ready;
	wait_queue_head_t wq;

};



/*
 * SynSens_char_devnode - return device permission
 *
 * @dev: char device structure
 * @mode: file permission
 *
 */
static char *f05_char_devnode(struct device *dev, mode_t *mode)
{
	if (!mode)
		return NULL;
	/* rmi** */
	/**mode = 0666*/
	*mode = (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
	dev_dbg(dev, "%s: setting mode of %s to 0x%08x\n", __func__,
		RAW_IMAGE_F05_CHAR_DEVICE_NAME, *mode);
	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}


/*
 * f05_char_dev_cleanup - release memory or unregister driver
 * @data: instance data for a particular device.
 *
 */
static void f05_char_dev_cleanup(struct f05_raw_data_char_dev *data)
{
	dev_t devno;

	/* Get rid of our char dev entries */
	if (data) {
		devno = data->raw_data_dev.dev;

		if (data->raw_data_device_class)
			device_destroy(data->raw_data_device_class, devno);

		cdev_del(&data->raw_data_dev);

		/* cleanup_module is never called if registering failed */
		unregister_chrdev_region(devno, 1);
		pr_debug("%s: f05 character device is removed\n", __func__);
	}
}

static ssize_t f05_char_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct rmi_function_container *fn_dev;
	struct f05_data *f05 = NULL;
	struct f05_raw_data_char_dev *char_dev_container = NULL;
	ssize_t ret_value  = 0;

	if (count == 0) {
		pr_info("%s: count = %d -- no space to copy output to!!!\n",
			__func__, count);
		return -ENOMEM;
	}

	if (!filp) {
		pr_info("%s: called with NULL file pointer\n", __func__);
		return -EINVAL;
	}
	char_dev_container = filp->private_data;

	if (!char_dev_container) {
		pr_info("%s: called with NULL private_data\n", __func__);
		return -EINVAL;
	}

	f05 = container_of(char_dev_container, struct f05_data, raw_data_feed);

	fn_dev = f05->fn_dev;
	if (count < f05->report_size) {
		pr_info("%s: count = %d but need %d bytes -- not enough space\n",
			__func__, count, f05->report_size);
		return -ENOMEM;
	}

	mutex_lock(&f05->status_mutex);
	wait_event_timeout(f05->wq,
		(f05->data_ready != 0) && (f05->status == RUN_STATUS), 10);
	if (f05->status == IDLE_STATUS) {
		dev_err(&fn_dev->dev, "F05 status is in an abnormal state: 0x%x",
					f05->status);
		f05->data_ready = 0;
		mutex_unlock(&f05->status_mutex);
		return -ECONNRESET;
	}

	ret_value = copy_to_user((void __user *)buf,
				 (const void *)f05->report_data,
				 f05->report_size);

	*f_pos += f05->report_size;
	f05->data_ready = 0;
	mutex_unlock(&f05->status_mutex);
	return ret_value;
}

void do_stop(struct f05_data *f05)
{
	struct rmi_function_container *fn_dev;
	struct rmi_driver *driver;
	struct rmi_driver_data *driver_data;
	struct rmi_device_platform_data *pdata;
	struct rmi_function_container *f01_dev;
	int retval;

	fn_dev = f05->fn_dev;
	driver = fn_dev->rmi_dev->driver;
	driver_data = dev_get_drvdata(&fn_dev->rmi_dev->dev);
	pdata = to_rmi_platform_data(fn_dev->rmi_dev);
	f01_dev = driver_data->f01_container;

	mutex_lock(&f05->data_mutex);
	f05->status = IDLE_STATUS;
	f05->data_ready = 0;
	wake_up(&f05->wq);
	if (driver_data->irq_stored && driver->restore_irq_mask) {
		f05->dev_data.report_mode = 0;
		/* Write 0 to the Report Mode back to the first Block
		 * Data registers. */
		retval = rmi_write_block(fn_dev->rmi_dev,
				fn_dev->fd.data_base_addr,
				(u8 *) &f05->dev_data,
				sizeof(f05->dev_data));
		if (retval < 0) {
			dev_warn(&fn_dev->dev, "%s : Could not write report mode to 0x%x\n",
				__func__, fn_dev->fd.data_base_addr);
		}
		dev_dbg(&fn_dev->dev, "Restoring interupts!\n");
		driver->restore_irq_mask(fn_dev->rmi_dev);

		dev_dbg(&fn_dev->rmi_dev->dev, "Resetting...\n");
		retval = rmi_write(fn_dev->rmi_dev,
				f01_dev->fd.command_base_addr,
				F01_RESET_MASK);
		if (retval < 0)
			dev_warn(&fn_dev->rmi_dev->dev,
				 "WARNING - post-flash reset failed, code: %d.\n",
				 retval);
		msleep(pdata->reset_delay_ms);
		dev_dbg(&fn_dev->rmi_dev->dev, "Reset completed.\n");
	}
	mutex_unlock(&f05->data_mutex);
}

/*
 * SynSens_char_dev_write: - use to write data into RMI stream
 * First byte is indication of parameter to change
 *
 * @filep : file structure for write
 * @buf: user-level buffer pointer contains data to be written
 * @count: number of byte be be written
 * @f_pos: offset (starting register address)
 *
 * @return number of bytes written from user buffer (buf) if succeeds
 *         negative number if error occurs.
 */
static ssize_t f05_char_dev_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *f_pos)
{
	struct f05_data *f05 = NULL;
	struct f05_raw_data_char_dev *char_dev_container = NULL;
	struct rmi_function_container *fn_dev;
	struct rmi_driver *driver;
	struct rmi_driver_data *driver_data;
	char tmpbuf[2];
	char command;
	u8 val;
	int retval = 0;

	if (!filp) {
		dev_err(&fn_dev->dev, "%s: called with NULL file pointer\n", __func__);
		return -EINVAL;
	}
	char_dev_container = filp->private_data;

	if (!char_dev_container) {
		dev_err(&fn_dev->dev, "%s: called with NULL private_data\n", __func__);
		return -EINVAL;
	}
	f05 = container_of(char_dev_container, struct f05_data, raw_data_feed);
	fn_dev = f05->fn_dev;
	driver = fn_dev->rmi_dev->driver;
	driver_data = dev_get_drvdata(&fn_dev->rmi_dev->dev);

	retval = copy_from_user(tmpbuf, buf, count);
	command = tmpbuf[0];
	switch (command) {
	case F05_REPORT_SET_TYPE:
		if (count < 1) {
			dev_err(&fn_dev->dev, "%s : missing report type.\n",
						__func__);
			retval = -EINVAL;
			goto error_exit;
		}
		val = (u8)tmpbuf[1];
		dev_info(&fn_dev->dev, "%s: report type %d\n", __func__, val);
		if (val > 4) {
			dev_err(&fn_dev->dev, "%s : Report type %d is invalid.\n",
						__func__, val);
			retval = -EINVAL;
			goto error_exit;
		}
		mutex_lock(&f05->status_mutex);
		if (f05->status == IDLE_STATUS) {
			f05->dev_data.report_mode = val;
			/* Write the Report Mode back to the first Block
			 * Data registers. */
			retval = rmi_write_block(fn_dev->rmi_dev,
					fn_dev->fd.data_base_addr,
					(u8*) &f05->dev_data,
					sizeof(f05->dev_data));
			mutex_unlock(&f05->status_mutex);
			if (retval < 0) {
				dev_err(&fn_dev->dev, "%s : Could not write report mode to 0x%x\n",
					__func__, fn_dev->fd.data_base_addr);
				retval = -EINVAL;
				goto error_exit;
			}
		} else {
			dev_err(&fn_dev->dev, "%s : Report type cannot change in the middle of command\n",
				__func__);
			mutex_unlock(&f05->status_mutex);
			retval =  -EINVAL;
			goto error_exit;
		}
		break;
	case F05_REPORT_START:
		/* Overwrite and store interrupts */
		mutex_lock(&f05->status_mutex);
		if (f05->status == IDLE_STATUS) {
			if (driver->store_irq_mask) {
				driver->store_irq_mask(fn_dev->rmi_dev,
							fn_dev->irq_mask);
			}
			f05->status = RUN_STATUS;
			f05->dev_command.force_zero = 0;
			f05->dev_command.get_image = 1;
			retval = rmi_write_block(f05->fn_dev->rmi_dev,
					f05->fn_dev->fd.command_base_addr,
					(u8*) &f05->dev_command,
					sizeof(f05->dev_command));
			mutex_unlock(&f05->status_mutex);
			if (retval < 0) {
				dev_err(&fn_dev->dev, "%s : Could not write command to 0x%x\n", __func__,
					f05->fn_dev->fd.command_base_addr);
				goto error_exit;
			}
		} else {
			dev_err(&fn_dev->dev, "%s : cannot start report in the middle of command\n",
				__func__);
			mutex_unlock(&f05->status_mutex);
			retval =  -EINVAL;
			goto error_exit;
		}
		break;
	case F05_REPORT_STOP:
		/* Turn back on other interupts, if it
		 * appears that we turned them off. */
		do_stop(f05);
		break;
	case F05_REPORT_FORCE_ZERO:
		mutex_lock(&f05->status_mutex);
		if (f05->status == IDLE_STATUS) {
			f05->dev_command.get_image = 0;
			f05->dev_command.force_zero = 1;
			retval = rmi_write_block(f05->fn_dev->rmi_dev,
					f05->fn_dev->fd.command_base_addr,
					(u8*) &f05->dev_command,
					sizeof(f05->dev_command));
			mutex_unlock(&f05->status_mutex);
			if (retval < 0) {
				dev_err(&fn_dev->dev, "%s : Could not write command to 0x%x\n", __func__,
					f05->fn_dev->fd.command_base_addr);
				goto error_exit;
			}
		} else {
			dev_err(&fn_dev->dev, "%s : cannot change Force Zero in the middle of command\n",
				__func__);
			mutex_unlock(&f05->status_mutex);
			retval =  -EINVAL;
			goto error_exit;
		}
		break;
	}
	retval = count;
error_exit:
	return retval;
}


/*
 * SynSens_char_dev_open: - get a new handle for reading raw Touch Sensor images
 * @inp : inode struture
 * @filp: file structure for read/write
 *
 * @return 0 if succeeds
 */
static int f05_char_dev_open(struct inode *inp, struct file *filp)
{
	struct f05_raw_data_char_dev *my_dev ;

	my_dev = container_of(inp->i_cdev,
			      struct f05_raw_data_char_dev,
			      raw_data_dev);


	filp->private_data = my_dev;
	return 0;
}

/*
 *  SynSens_char_dev_release: - release an existing handle
 *  @inp: inode structure
 *  @filp: file structure for read/write
 *
 *  @return 0 if succeeds
 */
static int f05_char_dev_release(struct inode *inp, struct file *filp)
{
	struct f05_data *f05 = NULL;
	struct f05_raw_data_char_dev *char_dev_container = NULL;

	char_dev_container = filp->private_data;

	if (!char_dev_container) {
		dev_err(&f05->fn_dev->dev, "%s: called with NULL private_data\n", __func__);
		return -EINVAL;
	}
	f05 = container_of(char_dev_container, struct f05_data, raw_data_feed);
	do_stop(f05);
	return 0;
}

static const struct file_operations f05_char_dev_fops = {
	.owner =    THIS_MODULE,
	.write =    f05_char_dev_write,
	.read =     f05_char_dev_read,
	.open =     f05_char_dev_open,
	.release =  f05_char_dev_release,
};



static ssize_t rmi_fn_05_num_of_rx_electrodes_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function_container *fn_dev = to_rmi_function_container(dev);

	data = fn_dev->data;

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		data->dev_query.num_of_rx_electrodes);
}

static ssize_t rmi_fn_05_num_of_tx_electrodes_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function_container *fn_dev = to_rmi_function_container(dev);

	data = fn_dev->data;

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		data->dev_query.num_of_tx_electrodes);
}

static ssize_t rmi_fn_05_has_delta16_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function_container *fn_dev = to_rmi_function_container(dev);

	data = fn_dev->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		data->dev_query.has16_bit_delta);
}

static ssize_t rmi_fn_05_image_window_size_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function_container *fn_dev = to_rmi_function_container(dev);

	data = fn_dev->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		data->dev_query.size_f05_image_window);
}

static ssize_t  rmi_fn_05_no_auto_cal_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function_container *fn_dev = to_rmi_function_container(dev);

	data = fn_dev->data;

	return snprintf(buf, PAGE_SIZE,
			"%d\n", data->dev_control.no_auto_cal);
}

static ssize_t  rmi_fn_05_no_auto_cal_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct f05_data *data = NULL;
	unsigned long new_value;
	int retval;
	struct rmi_function_container *fn_dev = to_rmi_function_container(dev);

	data = fn_dev->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid no auto cal bit %s.", __func__, buf);
		return -EINVAL;
	}

	data->dev_control.no_auto_cal = new_value;
	retval = rmi_write_block(fn_dev->rmi_dev, fn_dev->fd.control_base_addr,
			(u8*) &data->dev_control,
			sizeof(data->dev_control));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write no auto cal bit.\n");
	return retval;
}

/* query register access */
static struct device_attribute dev_attr_num_of_rx_electrodes =
	__ATTR(num_of_rx_electrodes, RMI_RO_ATTR,
		rmi_fn_05_num_of_rx_electrodes_show, NULL);

static struct device_attribute dev_attr_num_of_tx_electrodes =
	__ATTR(num_of_tx_electrodes, RMI_RO_ATTR,
		rmi_fn_05_num_of_tx_electrodes_show, NULL);

static struct device_attribute dev_attr_has_delta16 =
	__ATTR(has_delta16, RMI_RO_ATTR,
		rmi_fn_05_has_delta16_show, NULL);

static struct device_attribute dev_attr_image_window_size =
	__ATTR(image_window_size, RMI_RO_ATTR,
		rmi_fn_05_image_window_size_show, NULL);

/* control register access */
static struct device_attribute dev_attr_no_auto_cal =
	__ATTR(no_auto_cal, RMI_RW_ATTR,
	       rmi_fn_05_no_auto_cal_show, rmi_fn_05_no_auto_cal_store);

static struct attribute *attrs[] = {
	&dev_attr_num_of_rx_electrodes.attr,
	&dev_attr_num_of_tx_electrodes.attr,
	&dev_attr_has_delta16.attr,
	&dev_attr_image_window_size.attr,
	&dev_attr_no_auto_cal.attr,
	NULL
};

static struct attribute_group fn05_attrs = GROUP(attrs);

/*store dynamically allocated major number of char device*/
static int rmi_f05_char_dev_major_num;

/*
 * f05_raw_data_char_dev_register - register char device
 * called from init
 *
 * @phy: a pointer to an rmi_phys_devices structure
 *
 * @return: zero if suceeds
 */
static int
f05_raw_data_char_dev_register(struct f05_data *f05)
{
	dev_t dev_no;
	int err;
	int result;
	struct device *device_ptr;
	struct f05_raw_data_char_dev *char_dev = &f05->raw_data_feed;

	if (!f05)
		dev_err(&f05->fn_dev->dev, "%s: No RMI F05 data structure instance\n", __func__);

	if (rmi_f05_char_dev_major_num) {
		dev_no = MKDEV(rmi_f05_char_dev_major_num, 0);
		result = register_chrdev_region(dev_no, 1,
					RAW_IMAGE_F05_CHAR_DEVICE_NAME);
	} else {
		result = alloc_chrdev_region(&dev_no, 0, 1,
					     RAW_IMAGE_F05_CHAR_DEVICE_NAME);
		/* let kernel allocate a major for us */
		rmi_f05_char_dev_major_num = MAJOR(dev_no);
	}

	if (result < 0)
		return result;


	mutex_init(&char_dev->mutex_file_op);


	/* initialize the device */
	cdev_init(&char_dev->raw_data_dev, &f05_char_dev_fops);

	char_dev->raw_data_dev.owner = THIS_MODULE;

	/* tell the linux kernel to add the device */
	err = cdev_add(&char_dev->raw_data_dev, dev_no, 1);

	if (err) {
		dev_err(&f05->fn_dev->dev, "%s: Error %d adding f05_raw_data_char_dev.\n",
			__func__, err);
		return err;
	}

	/* create device node */
	f05->raw_data_feed.raw_data_device_class =
		class_create(THIS_MODULE, RAW_IMAGE_F05_CHAR_DEVICE_NAME);

	if (IS_ERR(f05->raw_data_feed.raw_data_device_class)) {
		dev_err(&f05->fn_dev->dev, "%s: Failed to create /dev/%s.\n",
				__func__, RAW_IMAGE_F05_CHAR_DEVICE_NAME);

		f05_char_dev_cleanup(char_dev);
		return -ENODEV;
	}

	/* setup permission */
	f05->raw_data_feed.raw_data_device_class->devnode = f05_char_devnode;

	/* class creation */
	device_ptr = device_create(
		f05->raw_data_feed.raw_data_device_class,
		NULL, dev_no, NULL,
		RAW_IMAGE_F05_CHAR_DEVICE_NAME"%d",
		MINOR(dev_no));

	if (IS_ERR(device_ptr)) {
		dev_err(&f05->fn_dev->dev, "Failed to create raw_data_read device.\n");
		f05_char_dev_cleanup(char_dev);
		return -ENODEV;
	}

	return 0;
}


static int rmi_f05_initialize(struct rmi_function_container *fn_dev)
{
	struct f05_data *f05;
	u16 query_base_addr;
	u16 control_base_addr;
	u8 num_of_rx_electrodes;
	u8 num_of_tx_electrodes;
	int rc;
	struct rmi_device *rmi_dev = fn_dev->rmi_dev;

	f05 = devm_kzalloc(&fn_dev->dev, sizeof(struct f05_data), GFP_KERNEL);
	if (!f05) {
		dev_err(&fn_dev->dev, "Failed to allocate fn_05_data.\n");
		return -ENOMEM;
	}
	fn_dev->data = f05;
	f05->fn_dev = fn_dev;

	query_base_addr = fn_dev->fd.query_base_addr;
	control_base_addr = fn_dev->fd.control_base_addr;
	rc = rmi_read_block(rmi_dev, query_base_addr, (u8*) &f05->dev_query,
				sizeof(struct f05_device_queries));

	if (rc < 0) {
		dev_err(&fn_dev->dev, "Failed to read f05 query register.\n");
		return rc;
	}

	rc = rmi_read(rmi_dev, control_base_addr, (u8*) &f05->dev_control);
	if (rc < 0) {
		dev_err(&fn_dev->dev, "Failed to read f05 control register.\n");
		return rc;
	}

	num_of_rx_electrodes = f05->dev_query.num_of_rx_electrodes;
	num_of_tx_electrodes = f05->dev_query.num_of_tx_electrodes + 1;
	f05->max_report_size = num_of_tx_electrodes * num_of_rx_electrodes *  2;
	f05->report_data = devm_kzalloc(&fn_dev->dev, f05->max_report_size,
					GFP_KERNEL);
	f05->tmp_buffer = devm_kzalloc(&fn_dev->dev, f05->max_report_size,
					GFP_KERNEL);

	mutex_init(&f05->status_mutex);
	mutex_init(&f05->data_mutex);
	f05->status = IDLE_STATUS;
	f05->data_ready = 0;
	init_waitqueue_head(&f05->wq);
	f05_raw_data_char_dev_register(f05);
	return 0;
}


static int rmi_f05_probe(struct rmi_function_container *fn_dev)
{
	int rc;

	rc = rmi_f05_initialize(fn_dev);
	if (rc < 0)
		return rc;

	if (sysfs_create_group(&fn_dev->dev.kobj, &fn05_attrs) < 0) {
		dev_err(&fn_dev->dev, "Failed to create query sysfs files.");
		return -ENODEV;
	}
	return 0;
}

static int rmi_f05_remove(struct rmi_function_container *fn_dev)
{
	struct f05_data *f05 = fn_dev->data;
	sysfs_remove_group(&fn_dev->dev.kobj, &fn05_attrs);
	class_destroy(f05->raw_data_feed.raw_data_device_class);
	return 0;
}

static int rmi_f05_reset(struct rmi_function_container *fn_dev)
{
	do_stop(fn_dev->data);
	return 0;
}

int rmi_f05_attention(struct rmi_function_container *fn_dev, u8 *irq_bits)
{
	u8 row_size;
	int i;
	int retval;
	struct f05_data *data;
	u8 has16_bit_delta;
	u8 num_of_rx_electrodes;
	u8 num_of_tx_electrodes;
	u8 report_mode;
	u8 *tmp_ptr;
	u8 *stash;

	data = fn_dev->data;
	has16_bit_delta = data->dev_query.has16_bit_delta;
	num_of_rx_electrodes = data->dev_query.num_of_rx_electrodes;
	num_of_tx_electrodes = data->dev_query.num_of_tx_electrodes;
	report_mode = data->dev_data.report_mode;
	stash = data->report_data;
	switch (report_mode) {
	/* this is baseline capacitance image data */
	case 1:
		row_size = num_of_rx_electrodes * 2;
		break;
	/* this is delta image data */
	case 2:
		if (has16_bit_delta == 0)
			row_size = num_of_rx_electrodes;
		else
			row_size = num_of_rx_electrodes * 2;
		break;
	default:
		row_size = 0;
		break;
	}

	data->report_size = row_size * num_of_tx_electrodes;
	if (data->report_size == 0) {
		dev_err(&fn_dev->dev, "Invalid report type set in %s. This should never happen.\n",
			__func__);
		data->status = IDLE_STATUS;
		data->data_ready = 0;
		return -EINVAL;
	}
	tmp_ptr = data->tmp_buffer;
	for (i = 0; i < num_of_tx_electrodes; i++) {
		data->dev_data.report_index = i;
		retval = rmi_write_block(fn_dev->rmi_dev,
				fn_dev->fd.data_base_addr,
				(u8*) &data->dev_data,
				sizeof(data->dev_data));
		if (retval < 0) {
			dev_err(&fn_dev->dev, "Failed to write to report index!\n");
			goto error_exit;
		} else
			retval = rmi_read_block(fn_dev->rmi_dev,
				(fn_dev->fd.data_base_addr+2),
				tmp_ptr, row_size);
		if (retval < 0) {
			dev_err(&fn_dev->dev, "Failed to read report data\n");
			goto error_exit;
		}
		tmp_ptr += row_size;
	}
	mutex_lock(&data->status_mutex);
	data->report_data = data->tmp_buffer;
	data->tmp_buffer = stash;
	data->data_ready = 1;
	wake_up(&data->wq);
	if (data->status == RUN_STATUS) {
		data->dev_command.force_zero = 0;
		data->dev_command.get_image = 1;
		retval = rmi_write_block(data->fn_dev->rmi_dev,
				data->fn_dev->fd.command_base_addr,
				(u8*) &data->dev_command,
				sizeof(data->dev_command));
		if (retval < 0) {
			dev_err(&fn_dev->dev, "%s : Could not write command to 0x%x\n", __func__,
				data->fn_dev->fd.command_base_addr);
			goto error_exit;
		}
	}
	mutex_unlock(&data->status_mutex);
error_exit:
	return data->status;
}

static int f05_remove_device(struct device *dev)
{
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	return rmi_f05_remove(fc);
}

static __devinit int f05_probe(struct device *dev)
{
	struct rmi_function_container *fc;

	if (dev->type != &rmi_function_type) {
		dev_dbg(dev, "Not a function device.\n");
		return 1;
	}
	fc = to_rmi_function_container(dev);
	if (fc->fd.function_number != 0x05) {
		dev_dbg(dev, "Device is F%02X, not F%02X.\n",
			fc->fd.function_number, 0x05);
		return 1;
	}

	return rmi_f05_probe(fc);
}

static struct rmi_function_handler function_handler = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "rmi_f05",
		.bus = &rmi_bus_type,
		.probe = f05_probe,
		.remove = f05_remove_device,
	},
	.func = 0x05,
	.reset = rmi_f05_reset,
	.attention = rmi_f05_attention,
};

static int __init rmi_f05_module_init(void)
{
	int error;

	error = driver_register(&function_handler.driver);
	if (error < 0) {
		pr_err("%s: register driver failed!\n", __func__);
		return error;
	}

	return 0;
}

static void __exit rmi_f05_module_exit(void)
{
	driver_unregister(&function_handler.driver);
}


module_init(rmi_f05_module_init);
module_exit(rmi_f05_module_exit);

MODULE_AUTHOR("Vivian Ly <vly@synaptics.com>");
MODULE_DESCRIPTION("RMI F05 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
