/*
 * Notle Panel support.
 *
 * Copyright (C) 2011 Google, Inc.
 * Author: John Hawley <madsci@google.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <video/omapdss.h>
#include <video/omap-panel-notle.h>

enum {
        NOTLE_I2C_FPGA  = 0,
        NOTLE_I2C_PANEL = 1,
};

struct init_register_value {
        u8 reg;
        u8 value;
};

static struct init_register_value panel_init_regs[] = {
  { 0x00, 0xC5 },
  { 0x01, 0xC3 },
  { 0x02, 0xC3 },
  { 0x13, 0x45 },
  { 0x14, 0x80 },
  { 0x15, 0xAA },
  { 0x16, 0xAA },
  { 0x17, 0x08 },
  { 0x18, 0x88 },
  { 0x19, 0x12 },
  { 0x1A, 0xE9 },
  { 0x21, 0x00 },
  { 0x22, 0x00 },
  { 0x23, 0x00 },
  { 0x24, 0xFF },
  { 0x25, 0xFF },
  { 0x26, 0xFF },
  { 0x27, 0x62 },
  { 0x28, 0x62 },
  { 0x29, 0x49 },
  { 0x2A, 0x9D },
  { 0x2B, 0x9D },
  { 0x2C, 0xB6 },
  { 0x2D, 0x80 },
  { 0x2E, 0x7B },
  { 0x2F, 0x67 },
  { 0x30, 0x7F },
  { 0x31, 0x84 },
  { 0x32, 0x98 },
  { 0x33, 0x95 },
  { 0x34, 0x90 },
  { 0x35, 0x79 },
  { 0x36, 0x6A },
  { 0x37, 0x6F },
  { 0x38, 0x86 },
  { 0x39, 0xA5 },
  { 0x3A, 0xA1 },
  { 0x3B, 0x85 },
  { 0x3C, 0x5A },
  { 0x3D, 0x5E },
  { 0x3E, 0x7A },
  { 0x3F, 0xB7 },
  { 0x40, 0xB3 },
  { 0x41, 0x90 },
  { 0x42, 0x48 },
  { 0x43, 0x4C },
  { 0x44, 0x6F },
  { 0x45, 0xC6 },
  { 0x46, 0xC8 },
  { 0x47, 0x93 },
  { 0x48, 0x39 },
  { 0x49, 0x37 },
  { 0x4A, 0x6C },
  { 0x4B, 0xFF },
  { 0x4C, 0xDC },
  { 0x4D, 0xC7 },
  { 0x4E, 0x00 },
  { 0x4F, 0x23 },
  { 0x50, 0x38 },
  { 0x00, 0x80 },
};

struct fpga_config {
  u8 config;
  u8 red_on_line;
  u8 green_on_line;
  u8 blue_on_line;
};

static struct fpga_config fpga_config = {
  .config = 0x00,
  .red_on_line = 0x01,
  .green_on_line = 0x02,
  .blue_on_line = 0x03,
};

struct panel_notle_i2c {
        struct i2c_client *fpga_client;
        struct i2c_client *panel_client;
};
static struct panel_notle_i2c *i2c_data;

struct panel_config {
        struct omap_video_timings timings;

        int acbi;        /* ac-bias pin transitions per interrupt */
        /* Unit: line clocks */
        int acb;        /* ac-bias pin frequency */

        enum omap_panel_config config;

        /* Delay in ms between DISPC dis/enable and display dis/enable */
        int power_on_delay;
        int power_off_delay;
};

/* Notle NHD Panel */
static struct panel_config notle_config = {
        .timings = {
                .x_res          = 640,
                .y_res          = 360,
                .pixel_clock    = 66400,

                .hfp            = 10,
                .hsw            = 68,
                .hbp            = 10,

                .vfp            = 5,
                .vsw            = 10,
                .vbp            = 5,
        },
        .acbi                   = 0x0,
        .acb                    = 0x0,
        .config                 = OMAP_DSS_LCD_TFT,
        .power_on_delay         = 0,
        .power_off_delay        = 0,
};

struct notle_drv_data {
        struct omap_dss_device *dssdev;
        struct panel_config *panel_config;
        struct kobject kobj;
};

static inline struct panel_notle_data
*get_panel_data(const struct omap_dss_device *dssdev) {
        return (struct panel_notle_data *) dssdev->data;
}

/* Local functions used by the sysfs interface */
static void panel_notle_power_off(struct omap_dss_device *dssdev);
static int panel_notle_power_on(struct omap_dss_device *dssdev);

/* Sysfs interface */
static ssize_t panel_notle_sysfs_reset(struct notle_drv_data *notle_data,
                                       const char *buf, size_t size) {
        panel_notle_power_off(notle_data->dssdev);
        msleep(100);
        panel_notle_power_on(notle_data->dssdev);
        return size;
}

/* Sysfs show functions */
static ssize_t panel_notle_pixel_clock_show(struct notle_drv_data *notle_data,
                                            char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        notle_data->dssdev->panel.timings.pixel_clock);
}

/* Sysfs store functions */
static ssize_t panel_notle_pixel_clock_store(struct notle_drv_data *notle_data,
                                             const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        notle_data->dssdev->panel.timings.pixel_clock = value;
        return size;
}

/* Sysfs attribute wrappers for show/store functions */
struct panel_notle_attribute {
        struct attribute attr;
        ssize_t (*show)(struct notle_drv_data *, char *);
        ssize_t (*store)(struct notle_drv_data *, const char *, size_t);
};

#define NOTLE_ATTR(_name, _mode, _show, _store) \
        struct panel_notle_attribute panel_notle_attr_##_name = \
        __ATTR(_name, _mode, _show, _store)

static NOTLE_ATTR(reset, S_IWUSR, NULL, panel_notle_sysfs_reset);
static NOTLE_ATTR(pixel_clock, S_IRUGO|S_IWUSR,
                  panel_notle_pixel_clock_show, panel_notle_pixel_clock_store);

static struct attribute *panel_notle_sysfs_attrs[] = {
        &panel_notle_attr_reset.attr,
        &panel_notle_attr_pixel_clock.attr,
        NULL,
};

static ssize_t panel_notle_attr_show(struct kobject *kobj, struct attribute *attr,
                                     char *buf) {
        struct notle_drv_data *panel_notle;
        struct panel_notle_attribute *panel_notle_attr;

        panel_notle = container_of(kobj, struct notle_drv_data, kobj);
        panel_notle_attr = container_of(attr, struct panel_notle_attribute, attr);

        if (!panel_notle_attr->show)
                return -ENOENT;

        return panel_notle_attr->show(panel_notle, buf);
}

static ssize_t panel_notle_attr_store(struct kobject *kobj, struct attribute *attr,
                                      const char *buf, size_t size) {
        struct notle_drv_data *panel_notle;
        struct panel_notle_attribute *panel_notle_attr;

        panel_notle = container_of(kobj, struct notle_drv_data, kobj);
        panel_notle_attr = container_of(attr, struct panel_notle_attribute, attr);

        if (!panel_notle_attr->store)
                return -ENOENT;

        return panel_notle_attr->store(panel_notle, buf, size);
}

static const struct sysfs_ops panel_notle_sysfs_ops = {
        .show = panel_notle_attr_show,
        .store = panel_notle_attr_store,
};

static struct kobj_type panel_notle_ktype = {
        .sysfs_ops = &panel_notle_sysfs_ops,
        .default_attrs = panel_notle_sysfs_attrs,
};

/* Utility functions */
static int panel_write_register(u8 reg, u8 value) {
        u8 buf[2];
        int r;
        struct i2c_msg msgs[1];

        if (!i2c_data || !i2c_data->panel_client) {
                printk(KERN_ERR "No I2C data set for Notle panel init\n");
                return -1;
        }

        buf[0] = reg;
        buf[1] = value;

        msgs[0].addr = i2c_data->panel_client->addr;
        msgs[0].flags = 0;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(i2c_data->panel_client->adapter, msgs, 1);
        if (r < 0) {
                return r;
        }

        return 0;
}

static int fpga_write_config(struct fpga_config *config) {
        int r;
        struct i2c_msg msgs[1];

        if (!i2c_data || !i2c_data->fpga_client) {
                printk(KERN_ERR "No I2C data set for Notle fpga init\n");
                return -1;
        }

        msgs[0].addr = i2c_data->fpga_client->addr;
        msgs[0].flags = 0;
        msgs[0].len = sizeof(*config);
        msgs[0].buf = (u8*)config;

        r = i2c_transfer(i2c_data->fpga_client->adapter, msgs, 1);
        if (r < 0) {
                return r;
        }

        return 0;
};

/* Functions to perform actions on the panel and DSS driver */
static int panel_notle_power_on(struct omap_dss_device *dssdev) {
        int i, r;
        struct panel_notle_data *panel_data = get_panel_data(dssdev);
        struct notle_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);
        struct panel_config *panel_config = drv_data->panel_config;

        if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
                return 0;
        }

        r = omapdss_dpi_display_enable(dssdev);
        if (r) {
                printk(KERN_ERR "Notle panel driver failed to enable DPI\n");
                goto err0;
        }

        if (panel_config->power_on_delay) {
                msleep(panel_config->power_on_delay);
        }

        if (panel_data->platform_enable) {
                r = panel_data->platform_enable(dssdev);
                if (r) {
                        printk(KERN_ERR "Notle panel driver failed to "
                               "platform_enable\n");
                        goto err1;
                }
        }

        for (i = 0; i < ARRAY_SIZE(panel_init_regs); ++i) {
                if(panel_write_register(panel_init_regs[i].reg,
                                        panel_init_regs[i].value)) {
                  printk(KERN_ERR "Failed to write panel config to Notle panel\n");
                  goto err2;
                }
        }

        if (fpga_write_config(&fpga_config)) {
                printk(KERN_ERR "Failed to write FPGA config for Notle panel\n");
                goto err2;
        }

        return 0;
err2:
        return 0;
err1:
        omapdss_dpi_display_disable(dssdev);
err0:
        return r;
}

static void panel_notle_power_off(struct omap_dss_device *dssdev) {
        struct panel_notle_data *panel_data = get_panel_data(dssdev);
        struct notle_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);
        struct panel_config *panel_config = drv_data->panel_config;

        if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
                return;
        }

        if (panel_data->platform_disable) {
                panel_data->platform_disable(dssdev);
        }

        if (panel_config->power_off_delay) {
                msleep(panel_config->power_off_delay);
        }

        omapdss_dpi_display_disable(dssdev);
}

static int panel_notle_probe(struct omap_dss_device *dssdev) {
        int r;
        struct panel_config *panel_config = &notle_config;
        struct notle_drv_data *drv_data = NULL;

        dev_dbg(&dssdev->dev, "probe\n");

        dssdev->panel.config = panel_config->config;
        dssdev->panel.timings = panel_config->timings;
        dssdev->panel.acb = panel_config->acb;
        dssdev->panel.acbi = panel_config->acbi;

        drv_data = kzalloc(sizeof(*drv_data), GFP_KERNEL);
        if (!drv_data) {
                return -ENOMEM;
        }

        drv_data->dssdev = dssdev;
        drv_data->panel_config = panel_config;

        dev_set_drvdata(&dssdev->dev, drv_data);

        r = kobject_init_and_add(&drv_data->kobj, &panel_notle_ktype,
                        &dssdev->manager->kobj, "panel-notle-dpi");
        if (r) {
                printk(KERN_WARNING "Notle panel failed to create sysfs directory\n");
        }

        return 0;
}

static void __exit panel_notle_remove(struct omap_dss_device *dssdev) {
        struct notle_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);

        dev_dbg(&dssdev->dev, "remove\n");

        kfree(drv_data);

        dev_set_drvdata(&dssdev->dev, NULL);
}

static int panel_notle_enable(struct omap_dss_device *dssdev) {
        int r = 0;

        r = panel_notle_power_on(dssdev);
        if (r)
                return r;

        dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

        return 0;
}

static int panel_notle_resume(struct omap_dss_device *dssdev) {
        int r = 0;

        r = panel_notle_power_on(dssdev);
        if (r)
                return r;

        dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

        return 0;
}

static void panel_notle_disable(struct omap_dss_device *dssdev) {
        panel_notle_power_off(dssdev);

        dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int panel_notle_suspend(struct omap_dss_device *dssdev) {
        panel_notle_power_off(dssdev);

        dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

        return 0;
}

static void panel_notle_set_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings) {
        dpi_set_timings(dssdev, timings);
}

static void panel_notle_get_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings) {
        *timings = dssdev->panel.timings;
}

static int panel_notle_check_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings) {
        return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver dpi_driver = {
        .probe                 = panel_notle_probe,
        .remove                = __exit_p(panel_notle_remove),

        .enable                = panel_notle_enable,
        .disable               = panel_notle_disable,
        .suspend               = panel_notle_suspend,
        .resume                = panel_notle_resume,

        .set_timings           = panel_notle_set_timings,
        .get_timings           = panel_notle_get_timings,
        .check_timings         = panel_notle_check_timings,

        .driver                = {
                .name                  = "panel_notle",
                .owner                 = THIS_MODULE,
        },
};

/* Functions to handle initialization of the i2c driver */
static int __devinit i2c_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
        if (i2c_data == NULL) {
                i2c_data = kzalloc(sizeof(*i2c_data), GFP_KERNEL);
                if (i2c_data == NULL) {
                        return -ENOMEM;
                }
        }

        i2c_set_clientdata(client, i2c_data);

        switch (id->driver_data) {
          case NOTLE_I2C_FPGA:
            /* panel-notle-fpga */
            i2c_data->fpga_client = client;
            break;
          case NOTLE_I2C_PANEL:
            /* panel-notle-panel */
            i2c_data->panel_client = client;
            break;
          default:
            printk(KERN_WARNING "Unrecognized i2c device in Notle panel driver\n");
            return -EINVAL;
        }

        return 0;
}

static int __devexit i2c_remove(struct i2c_client *client)
{
        struct panel_notle_i2c *drv_data = i2c_get_clientdata(client);

        i2c_set_clientdata(client, NULL);

        i2c_data = NULL;
        kfree(drv_data);

        return 0;
}

/* These are the I2C devices we support */
static const struct i2c_device_id i2c_idtable[] = {
        {"panel-notle-fpga", NOTLE_I2C_FPGA},
        {"panel-notle-panel", NOTLE_I2C_PANEL},
        {},
};

static struct i2c_driver i2c_driver = {
        .probe = i2c_probe,
        .remove = __exit_p(i2c_remove),
        .id_table = i2c_idtable,
        .driver = {
                   .name  = "panel-notle-i2c",
                   .owner = THIS_MODULE,
        },
};

static int __init panel_notle_drv_init(void) {
        int r = 0;
        r = i2c_add_driver(&i2c_driver);
        if (r < 0) {
                printk(KERN_WARNING "Notle panel i2c driver registration failed\n");
                goto err0;
        }

        r = omap_dss_register_driver(&dpi_driver);
        if (r < 0) {
                printk(KERN_WARNING "Notle panel dss driver registration failed\n");
                goto err1;
        }

        return 0;

err1:
        i2c_del_driver(&i2c_driver);
err0:
        return r;
}

static void __exit panel_notle_drv_exit(void) {
        omap_dss_unregister_driver(&dpi_driver);
        i2c_del_driver(&i2c_driver);
}

module_init(panel_notle_drv_init);
module_exit(panel_notle_drv_exit);



MODULE_DESCRIPTION("Notle FPGA and Panel Driver");
MODULE_LICENSE("GPL");
