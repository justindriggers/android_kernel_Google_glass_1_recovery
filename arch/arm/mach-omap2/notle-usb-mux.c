/*
 * arch/arm/mach-omap2/notle-usb-mux.c
 *
 * Notle board USB MUX control driver.
 *
 * Copyright (c) 2013 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include "board-notle.h"
#include "notle-usb-mux.h"

enum usb_mux_mode {
	USB_MUX_MODE_FLOATING = 0,
	USB_MUX_MODE_USB      = 1,
	USB_MUX_MODE_TTY      = 2,
	USB_MUX_MODE_AUDIO    = 3,
};

struct usb_mux_device_info {
	struct device *dev;

	int gpio_cb0;
	int gpio_cb1;

	bool force_usb;
	bool usb_online;

	enum usb_mux_mode cur_mode;
	enum usb_mux_mode req_mode;

	struct otg_transceiver *otg;
	struct notifier_block nb;

	struct mutex lock;

	struct workqueue_struct *wq;
	struct work_struct work;
};

static struct usb_mux_device_info *usb_mux_di;

static const char *str_for_mode(enum usb_mux_mode mode)
{
	switch (mode) {
		case USB_MUX_MODE_FLOATING:
			return "floating";

		case USB_MUX_MODE_USB:
			return "usb";

		case USB_MUX_MODE_TTY:
			return "tty";

		case USB_MUX_MODE_AUDIO:
			return "audio";

		default:
			return "unknown";
	}
}

/*
 * Syncs the GPIO MUX control lines with the requested MUX mode.
 *
 * Call with di->lock mutex held.
 */
static void sync_usb_mux_mode(struct usb_mux_device_info *di)
{
	enum usb_mux_mode mode;

	if (di->force_usb || di->usb_online)
		mode = USB_MUX_MODE_USB;
	else
		mode = di->req_mode;

	dev_warn(di->dev, "Setting MUX mode to %s\n", str_for_mode(mode));

	switch (mode) {
		case USB_MUX_MODE_FLOATING:
			gpio_set_value(di->gpio_cb1, 0);
			gpio_set_value(di->gpio_cb0, 0);
			break;

		case USB_MUX_MODE_USB:
			gpio_set_value(di->gpio_cb1, 0);
			gpio_set_value(di->gpio_cb0, 1);
			break;

		case USB_MUX_MODE_TTY:
			gpio_set_value(di->gpio_cb1, 1);
			gpio_set_value(di->gpio_cb0, 0);
			break;

		case USB_MUX_MODE_AUDIO:
			gpio_set_value(di->gpio_cb1, 1);
			gpio_set_value(di->gpio_cb0, 1);
			break;

		default:
			dev_err(di->dev, "Invalid USB MUX mode requested: %d\n", mode);
			goto error;
	}

	di->cur_mode = mode;

error:
	return;
}

static void request_usb_mux_mode(struct usb_mux_device_info *di, enum usb_mux_mode mode)
{
	mutex_lock(&di->lock);

	di->req_mode = mode;
	sync_usb_mux_mode(di);

	mutex_unlock(&di->lock);
}

static enum usb_mux_mode get_usb_mux_mode(struct usb_mux_device_info *di)
{
	enum usb_mux_mode mode;

	mutex_lock(&di->lock);
	mode = di->cur_mode;
	mutex_unlock(&di->lock);

	return mode;
}

static void usb_mux_work(struct work_struct *work)
{
	struct usb_mux_device_info *di = container_of(work,
			struct usb_mux_device_info, work);

	mutex_lock(&di->lock);
	sync_usb_mux_mode(di);
	mutex_unlock(&di->lock);
}

/*
 * hack to allow charger to force usb mux to usb mode before turning on charger.
 * remove this when this driver takes over USB ID duties and have this driver call
 * a notifier chain that the charger will use to know when it's safe to engage
 */
void usb_mux_force(int use_usb)
{
	struct usb_mux_device_info *di = usb_mux_di;

	if (!di)
		return;

	mutex_lock(&di->lock);
	di->force_usb = !!use_usb;
	sync_usb_mux_mode(di);
	mutex_unlock(&di->lock);
}
EXPORT_SYMBOL(usb_mux_force);

static int usb_mux_usb_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct usb_mux_device_info *di =
		container_of(nb, struct usb_mux_device_info, nb);
	bool usb_online = di->usb_online;

	switch (event) {
		case USB_EVENT_VBUS:
		case USB_EVENT_ENUMERATED:
		case USB_EVENT_CHARGER:
			di->usb_online = true;
			break;

		case USB_EVENT_NONE:
			di->usb_online = false;
			break;

		case USB_EVENT_ID:
			return NOTIFY_OK; // don't react to these yet
	}

	if (usb_online != di->usb_online)
		queue_work(di->wq, &di->work);

	return NOTIFY_OK;
}

static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int status = count;
	struct usb_mux_device_info *di = dev_get_drvdata(dev);

	enum usb_mux_mode mode = USB_MUX_MODE_TTY;

	/* PARANOID: count should be at least 1, but make sure anyway */
	if (!count)
		count = 1;

	if (!strncmp(buf, "floating", count-1))
		mode = USB_MUX_MODE_FLOATING;
	else if (!strncmp(buf, "usb", count-1))
		mode = USB_MUX_MODE_USB;
	else if (!strncmp(buf, "tty", count-1))
		mode = USB_MUX_MODE_TTY;
	else if (!strncmp(buf, "audio", count-1))
		mode = USB_MUX_MODE_AUDIO;
	else
		status = -EINVAL;

	dev_warn(di->dev, "mode=%s status=%d count=%zu buf=%s\n", str_for_mode(mode),
			status, count, buf);

	if (status > 0)
		request_usb_mux_mode(di, mode);

	return status;
}

static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *val;
	struct usb_mux_device_info *di = dev_get_drvdata(dev);

	enum usb_mux_mode mode = get_usb_mux_mode(di);
	val = str_for_mode(mode);

	return sprintf(buf, "%s\n", val);
}

static DEVICE_ATTR(mux_mode, S_IWUGO | S_IRUGO, show_mode, set_mode);

static struct attribute *usb_mux_attributes[] = {
	&dev_attr_mux_mode.attr,
	NULL,
};

static const struct attribute_group usb_mux_attr_group = {
	.attrs = usb_mux_attributes,
};

static int usb_mux_probe(struct platform_device *pdev)
{
	int ret;
	int temp;
	struct usb_mux_platform_data *pdata = pdev->dev.platform_data;
	struct usb_mux_device_info *di = NULL;

	if (!pdata) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "platform_data is NULL!!\n");
		goto error;
	}

	di = kzalloc(sizeof(struct usb_mux_device_info), GFP_KERNEL);
	if (!di) {
		ret = -ENOMEM;
		goto error;
	}

	di->dev = &pdev->dev;

	di->gpio_cb0 = pdata->gpio_cb0;
	di->gpio_cb1 = pdata->gpio_cb1;

	ret = gpio_request_one(pdata->gpio_cb0, pdata->gpio_cb0_flags, pdata->gpio_cb0_label);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get usb_mux_cb0 gpio_%d\n", pdata->gpio_cb0);
		goto error;
	}

	ret = gpio_request_one(pdata->gpio_cb1, pdata->gpio_cb1_flags, pdata->gpio_cb1_label);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get usb_mux_cb1 gpio_%d\n", pdata->gpio_cb1);
		goto error;
	}

	/* read the current mux config (should match the gpio flags initial drive) */
	temp  = (!!gpio_get_value(di->gpio_cb0)) << 0;
	temp |= (!!gpio_get_value(di->gpio_cb1)) << 1;
	di->cur_mode = di->req_mode = temp;

	dev_warn(&pdev->dev, "Initial MUX mode is %s\n", str_for_mode(di->cur_mode));

	platform_set_drvdata(pdev, di);

	mutex_init(&di->lock);

	di->wq = create_freezable_workqueue(dev_name(&pdev->dev));
	INIT_WORK(&di->work, usb_mux_work);

	di->nb.notifier_call = usb_mux_usb_notifier_call;
	di->otg = otg_get_transceiver();
	if (di->otg) {
		ret = otg_register_notifier(di->otg, &di->nb);
		if (ret) {
			dev_err(&pdev->dev, "otg register notifier failed %d\n", ret);
			otg_put_transceiver(di->otg);
			goto error;
		}
	} else {
		dev_err(&pdev->dev, "otg_get_transceiver failed %d\n", ret);
		goto error;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &usb_mux_attr_group);
	if (ret)
		dev_err(&pdev->dev, "could not create sysfs files\n");

	usb_mux_di = di;

	return 0;

error:
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	dev_warn(&pdev->dev, "exiting probe with error: ret=%d\n", ret);

	return ret;
}

static int usb_mux_remove(struct platform_device *pdev)
{
	struct usb_mux_device_info *di = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &usb_mux_attr_group);

	otg_unregister_notifier(di->otg, &di->nb);
	otg_put_transceiver(di->otg);

	cancel_work_sync(&di->work);
	destroy_workqueue(di->wq);

	gpio_free(di->gpio_cb0);
	gpio_free(di->gpio_cb1);

	platform_set_drvdata(pdev, NULL);

	usb_mux_di = NULL;
	kfree(di);

	return 0;
}

static struct platform_driver usb_mux_driver = {
	.driver = {
		.name	= "usb_mux",
	},
	.probe		= usb_mux_probe,
	.remove		= usb_mux_remove,
};

static int __init usb_mux_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&usb_mux_driver);
	if (!ret) {
		pr_err("Failed to register USB MUX driver\n");
		goto error;
	}

error:
	return ret;
}

static void __exit usb_mux_driver_exit(void)
{
	platform_driver_unregister(&usb_mux_driver);
}

module_init(usb_mux_driver_init);
module_exit(usb_mux_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Corey Tabaka <eieio@google.com>");

