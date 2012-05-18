/* drivers/char/gps_elton.c
 *
 * Copyright (C) 2012 Google, Inc.
 *
 *  Elton CSR SiRFstar GSD4e driver.
 *
 * Driver used primarily for handling suspend/resume system events.
 */
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/slab.h>

/* kernel/timer.c */
extern void msleep(unsigned int msecs);

#define DRVNAME "gps_elton"

/* NOTE: This is from the notle board default configuration
  and must be kept in sync. */
#define GPIO_GPS_ON_OFF                 49
#define GPIO_GPS_RESET_N                52

/* Time to wait between toggling power pin. */
static const int wait_between_power_toggle_in_ms = 50;

MODULE_AUTHOR("Chris Manton <cmanton@google.com>");
MODULE_DESCRIPTION("Elton SiRF4e GPS Driver");
MODULE_LICENSE("GPL");

struct gps_elton_data {
	struct device *dev;
	struct class *class;
	struct early_suspend early_suspend;
	/* System flag synchronized with device indication chip low power mode. */
	int is_suspended;
};
static struct gps_elton_data *gps_elton = NULL;

static ssize_t gps_omap_suspend_show(struct device *dev,
                                     struct device_attribute *attr, char *buf);

static ssize_t gps_omap_suspend_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count);

static struct device_attribute attrs[] = {
	__ATTR(suspend, 0666,
	       gps_omap_suspend_show,
	       gps_omap_suspend_store),
};

static void _toggle_power(struct gps_elton_data * gps_elton)
{
	gpio_set_value(GPIO_GPS_ON_OFF, 1);
	msleep(wait_between_power_toggle_in_ms);
	gpio_set_value(GPIO_GPS_ON_OFF, 0);
}

static ssize_t gps_omap_suspend_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct gps_elton_data *gps_elton = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", gps_elton->is_suspended);
}

/* User manually specifying suspend or resume mode of GPS chip. */
static ssize_t gps_omap_suspend_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
	int data;
	struct gps_elton_data *gps_elton = dev_get_drvdata(dev);
	sscanf(buf, "%d", &data);

	/* Validate parameters. */
	if (data != 0 && data != 1) {
		return -EINVAL;
	}

	if (data == 1) {
		/* We want to suspend. */
		if (gps_elton->is_suspended) {
			/* But we are already suspended */
			dev_info(gps_elton->dev, "%s already suspended\n", __func__);
			return -EAGAIN;
		} else {
			_toggle_power(gps_elton);
			gps_elton->is_suspended = 1;
			dev_info(gps_elton->dev, "%s user suspended GPS chip\n", __func__);
		}
	} else {
		/* We want to resume. */
		if (gps_elton->is_suspended) {
			_toggle_power(gps_elton);
			gps_elton->is_suspended = 0;
			dev_info(gps_elton->dev, "%s user resumed GPS chip\n", __func__);
		} else {
			/* But we are already resumed. */
			dev_info(gps_elton->dev, "%s already resumed\n", __func__);
			return -EAGAIN;
		}
	}
	return count;
}

/* Kernel power manager specifying suspend or resume mode of GPS chip. */
static void gps_elton_early_suspend(struct early_suspend *h)
{
	if (gps_elton->is_suspended == 1) {
		dev_warn(gps_elton->dev, "%s already suspended\n", __func__);
		return;
	}
	_toggle_power(gps_elton);
	gps_elton->is_suspended = 1;
	dev_dbg(gps_elton->dev, "%s system suspended GPS chip\n", __func__);
}

static void gps_elton_late_resume(struct early_suspend *h)
{
	if (gps_elton->is_suspended == 0) {
		dev_warn(gps_elton->dev, "%s already resumed\n", __func__);
		return;
	}
	_toggle_power(gps_elton);
	gps_elton->is_suspended = 0;
	dev_dbg(gps_elton->dev, "%s system resumed GPS chip\n", __func__);
}

static int __init gps_elton_init(void)
{
	int rc = 0;
	int attr_count = 0;
	struct gps_elton_data *init_elton = NULL;
	struct device *init_dev = NULL;
	struct class *init_class = NULL;

	init_elton = kzalloc(sizeof(struct gps_elton_data), GFP_KERNEL);
	if (!init_elton) {
		dev_err(gps_elton->dev, "%s Unable to allocate memory for device\n", __func__);
		return -ENOMEM;
	}
	gps_elton = init_elton;

	init_class = class_create(THIS_MODULE, "gps");
	if (IS_ERR(init_class)) {
		rc = PTR_ERR(init_class);
		goto error_exit;
	}
	gps_elton->class = init_class;

	init_dev = device_create(gps_elton->class, NULL, MKDEV(0, 0), gps_elton, DRVNAME);
	if (IS_ERR(init_dev)){
		rc = PTR_ERR(init_dev);
		goto error_exit;
	}
	gps_elton->dev = init_dev;

	/* Setup the suspend and resume functionality */
	INIT_LIST_HEAD(&gps_elton->early_suspend.link);
	gps_elton->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	gps_elton->early_suspend.suspend = gps_elton_early_suspend;
	gps_elton->early_suspend.resume = gps_elton_late_resume;
	// TODO(cmanton) Don't suspend/resume for Russ's testing.
	// register_early_suspend(&gps_elton->early_suspend);

	/* This flag must be synchronized with the power pin toggles on the device.
	   Assume this init routine is being called on cold start and that the
	   device is currently suspended. */
	gps_elton->is_suspended = 1;

        /* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		rc = device_create_file(gps_elton->dev, &attrs[attr_count]);
		if (rc < 0) {
			dev_err(gps_elton->dev,
			        "%s: Failed to create sysfs file for %s.\n",
			        __func__, attrs[attr_count].attr.name);
			goto error_exit;
		}
	}

	dev_info(gps_elton->dev, "%s driver initialized successfully\n", __func__);
	return 0;

 error_exit:
	if (gps_elton->dev)
		device_destroy(gps_elton->class, gps_elton->dev->devt);

	if (gps_elton->class)
		class_destroy(gps_elton->class);

	if (gps_elton)
		kfree(gps_elton);

	gps_elton = NULL;
	return rc;
}

static void __exit gps_elton_cleanup(void)
{
	int attr_count = 0;

	unregister_early_suspend(&gps_elton->early_suspend);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		device_remove_file(gps_elton->dev, &attrs[attr_count]);
	}

	device_destroy(gps_elton->class, gps_elton->dev->devt);
	class_destroy(gps_elton->class);
}

module_init(gps_elton_init);
module_exit(gps_elton_cleanup);
