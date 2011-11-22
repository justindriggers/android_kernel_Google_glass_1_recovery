/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/input.h>
#include <linux/slab.h>

#define QUERY_BASE_INDEX 1
#define MAX_LEN 256

/* data specific to fn $19 that needs to be kept around */
struct f19_data {
	bool *button_down;
	unsigned char button_count;
	unsigned char button_data_buffer_size;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	char input_name[MAX_LEN];
	char input_phys[MAX_LEN];
	struct input_dev *input;
};

static ssize_t rmi_f19_button_count_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_f19_buttonMap_show(struct device *dev,
				      struct device_attribute *attr, char *buf);

static ssize_t rmi_f19_buttonMap_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);

static struct device_attribute attrs[] = {
	__ATTR(button_count, RMI_RO_ATTR,
	       rmi_f19_button_count_show, rmi_store_error),	/* RO attr */
	__ATTR(buttonMap, RMI_RW_ATTR,
	       rmi_f19_buttonMap_show, rmi_f19_buttonMap_store)	/* RW attr */
};


static int rmi_f19_init(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_device_platform_data *pdata;
	struct f19_data *f19;
	struct input_dev *input_dev;
	u8 query_base_addr;
	int rc;
	int i;
	int attr_count = 0;

	dev_info(&fc->dev, "Intializing F19 values.");

	f19 = kzalloc(sizeof(struct f19_data), GFP_KERNEL);
	if (!f19) {
		dev_err(&fc->dev, "Failed to allocate function data.\n");
		return -ENOMEM;
	}
	pdata = to_rmi_platform_data(rmi_dev);
	query_base_addr = fc->fd.query_base_addr;

	/* initial all default values for f19 data here */
	rc = rmi_read(rmi_dev, query_base_addr+QUERY_BASE_INDEX,
		&f19->button_count);
	f19->button_count &= 0x1F;
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to read query register.\n");
		goto err_free_data;
	}

	/* Figure out just how much data we'll need to read. */
	f19->button_down = kcalloc(f19->button_count, sizeof(bool), GFP_KERNEL);
	if (!f19->button_down) {
		dev_err(&fc->dev, "Failed to allocate button state buffer.\n");
		rc = -ENOMEM;
		goto err_free_data;
	}

	f19->button_data_buffer_size = (f19->button_count + 7) / 8;
	f19->button_data_buffer =
	    kcalloc(f19->button_data_buffer_size,
		    sizeof(unsigned char), GFP_KERNEL);
	if (!f19->button_data_buffer) {
		dev_err(&fc->dev, "Failed to allocate button data buffer.\n");
		rc = -ENOMEM;
		goto err_free_data;
	}

	f19->button_map = kcalloc(f19->button_count,
				sizeof(unsigned char), GFP_KERNEL);
	if (!f19->button_map) {
		dev_err(&fc->dev, "Failed to allocate button map.\n");
		rc = -ENOMEM;
		goto err_free_data;
	}

	if (pdata) {
		if (pdata->button_map->nbuttons != f19->button_count) {
			dev_warn(&fc->dev,
				"Platformdata button map size (%d) != number "
				"of buttons on device (%d) - ignored.\n",
				pdata->button_map->nbuttons, f19->button_count);
		} else if (!pdata->button_map->map) {
			dev_warn(&fc->dev,
				 "Platformdata button map is missing!\n");
		} else {
			for (i = 0; i < pdata->button_map->nbuttons; i++)
				f19->button_map[i] = pdata->button_map->map[i];
		}
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&fc->dev, "Failed to allocate input device.\n");
		rc = -ENOMEM;
		goto err_free_data;
	}

	f19->input = input_dev;
	snprintf(f19->input_name, MAX_LEN, "%sfn%02x", dev_name(&rmi_dev->dev),
		fc->fd.function_number);
	input_dev->name = f19->input_name;
	snprintf(f19->input_phys, MAX_LEN, "%s/input0", input_dev->name);
	input_dev->phys = f19->input_phys;
	input_dev->dev.parent = &rmi_dev->dev;
	input_set_drvdata(input_dev, f19);

	/* Set up any input events. */
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	/* set bits for each button... */
	for (i = 0; i < f19->button_count; i++)
		set_bit(f19->button_map[i], input_dev->keybit);
	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to register input device.\n");
		goto err_free_input;
	}

	dev_dbg(&fc->dev, "Creating sysfs files.\n");
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev, "Failed to create sysfs file for %s.",
			     attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_free_data;
		}
	}
	fc->data = f19;
	return 0;

err_free_input:
	input_free_device(f19->input);

err_free_data:
	if (f19) {
		kfree(f19->button_down);
		kfree(f19->button_data_buffer);
		kfree(f19->button_map);
	}
	kfree(f19);
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);
	return rc;
}

int rmi_f19_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f19_data *f19 = fc->data;
	u8 data_base_addr = fc->fd.data_base_addr;
	int error;
	int button;

	/* Read the button data. */

	error = rmi_read_block(rmi_dev, data_base_addr, f19->button_data_buffer,
			f19->button_data_buffer_size);
	if (error < 0) {
		dev_err(&fc->dev, "%s: Failed to read button data registers.\n",
			__func__);
		return error;
	}

	/* Generate events for buttons that change state. */
	for (button = 0; button < f19->button_count;
	     button++) {
		int button_reg;
		int button_shift;
		bool button_status;

		/* determine which data byte the button status is in */
		button_reg = button / 7;
		/* bit shift to get button's status */
		button_shift = button % 8;
		button_status =
		    ((f19->button_data_buffer[button_reg] >> button_shift)
			& 0x01) != 0;

		/* if the button state changed from the last time report it
		 * and store the new state */
		if (button_status != f19->button_down[button]) {
			dev_dbg(&fc->dev, "%s: Button %d (code %d) -> %d.\n",
				__func__, button, f19->button_map[button],
				 button_status);
			/* Generate an event here. */
			input_report_key(f19->input, f19->button_map[button],
					 button_status);
			f19->button_down[button] = button_status;
		}
	}

	input_sync(f19->input); /* sync after groups of events */
	return 0;
}

static void rmi_f19_remove(struct rmi_function_container *fc)
{
	struct f19_data *data = fc->data;
	if (data) {
		kfree(data->button_down);
		kfree(data->button_data_buffer);
		kfree(data->button_map);
		input_unregister_device(data->input);
	}
	kfree(fc->data);
}

static struct rmi_function_handler function_handler = {
	.func = 0x19,
	.init = rmi_f19_init,
	.attention = rmi_f19_attention,
	.remove = rmi_f19_remove
};

static int __init rmi_f19_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s: register failed!\n", __func__);
		return error;
	}

	return 0;
}

static void rmi_f19_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}


static ssize_t rmi_f19_button_count_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f19_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->button_count);
}

static ssize_t rmi_f19_buttonMap_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{

	struct rmi_function_container *fc;
	struct f19_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	/* loop through each button map value and copy its
	 * string representation into buf */
	for (i = 0; i < data->button_count; i++) {
		/* get next button mapping value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", data->button_map[i]);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build button map buffer, "
				"code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;
}

static ssize_t rmi_f19_buttonMap_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_container *fc;
	struct f19_data *data;
	unsigned int button;
	int i;
	int retval = count;
	int button_count = 0;
	unsigned char temp_button_map[KEY_MAX];

	fc = to_rmi_function_container(dev);
	data = fc->data;

	/* Do validation on the button map data passed in.  Store button
	 * mappings into a temp buffer and then verify button count and
	 * data prior to clearing out old button mappings and storing the
	 * new ones. */
	for (i = 0; i < data->button_count && *buf != 0;
	     i++) {
		/* get next button mapping value and store and bump up to
		 * point to next item in buf */
		sscanf(buf, "%u", &button);

		/* Make sure the key is a valid key */
		if (button > KEY_MAX) {
			dev_err(dev,
				"%s: Error - button map for button %d is not a "
				"valid value 0x%x.\n",
				__func__, i, button);
			retval = -EINVAL;
			goto err_ret;
		}

		temp_button_map[i] = button;
		button_count++;

		/* bump up buf to point to next item to read */
		while (*buf != 0) {
			buf++;
			if (*(buf - 1) == ' ')
				break;
		}
	}

	/* Make sure the button count matches */
	if (button_count != data->button_count) {
		dev_err(dev,
		    "%s: Error - button map count of %d doesn't match device "
		     "button count of %d.\n", __func__, button_count,
		     data->button_count);
		retval = -EINVAL;
		goto err_ret;
	}

	/* Clear the key bits for the old button map. */
	for (i = 0; i < button_count; i++)
		clear_bit(data->button_map[i], data->input->keybit);

	/* Switch to the new map. */
	memcpy(data->button_map, temp_button_map,
	       data->button_count);

	/* Loop through the key map and set the key bit for the new mapping. */
	for (i = 0; i < button_count; i++)
		set_bit(data->button_map[i], data->input->keybit);

err_ret:
	return retval;
}
module_init(rmi_f19_module_init);
module_exit(rmi_f19_module_exit);

MODULE_AUTHOR("Eric Andersson <eric.andersson@unixphere.com>");
MODULE_DESCRIPTION("RMI F19 module");
MODULE_LICENSE("GPL");

