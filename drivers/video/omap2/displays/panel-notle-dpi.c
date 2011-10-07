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

struct notle_panel_i2c {
        struct i2c_client *fpga_client;
        struct i2c_client *panel_client;
};
static struct notle_panel_i2c *i2c_data;

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
                .pixel_clock    = 76800,

                .hfp            = 48,
                .hsw            = 32,
                .hbp            = 80,

                .vfp            = 3,
                .vsw            = 4,
                .vbp            = 7,
        },
        .acbi                   = 0x0,
        .acb                    = 0x0,
        .config                 = OMAP_DSS_LCD_TFT,
        .power_on_delay         = 0,
        .power_off_delay        = 0,
};

struct panel_drv_data {
        struct omap_dss_device *dssdev;
        struct panel_config *panel_config;
};

static inline struct panel_notle_data
*get_panel_data(const struct omap_dss_device *dssdev) {
        return (struct panel_notle_data *) dssdev->data;
}

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
                printk(KERN_ERR "Failed I2C write to Notle panel\n");
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
                printk(KERN_ERR "Failed I2C write to Notle fpga\n");
                return r;
        }

        return 0;
};

static int notle_panel_power_on(struct omap_dss_device *dssdev) {
        int i, r;
        struct panel_notle_data *panel_data = get_panel_data(dssdev);
        struct panel_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);
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
                panel_write_register(panel_init_regs[i].reg,
                                     panel_init_regs[i].value);
        }

        /* TODO(madsci): Set up some sysfs attributes to control this config. */
        fpga_write_config(&fpga_config);

        return 0;
err1:
        omapdss_dpi_display_disable(dssdev);
err0:
        return r;
}

static void notle_panel_power_off(struct omap_dss_device *dssdev) {
        struct panel_notle_data *panel_data = get_panel_data(dssdev);
        struct panel_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);
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

static int notle_panel_probe(struct omap_dss_device *dssdev) {
        struct panel_config *panel_config = &notle_config;
        struct panel_drv_data *drv_data = NULL;

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

        return 0;
}

static void __exit notle_panel_remove(struct omap_dss_device *dssdev) {
        struct panel_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);

        dev_dbg(&dssdev->dev, "remove\n");

        kfree(drv_data);

        dev_set_drvdata(&dssdev->dev, NULL);
}

static int notle_panel_enable(struct omap_dss_device *dssdev) {
        int r = 0;

        r = notle_panel_power_on(dssdev);
        if (r)
                return r;

        dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

        return 0;
}

static int notle_panel_resume(struct omap_dss_device *dssdev) {
        int r = 0;

        r = notle_panel_power_on(dssdev);
        if (r)
                return r;

        dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

        return 0;
}

static void notle_panel_disable(struct omap_dss_device *dssdev) {
        notle_panel_power_off(dssdev);

        dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int notle_panel_suspend(struct omap_dss_device *dssdev) {
        notle_panel_power_off(dssdev);

        dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

        return 0;
}

static void notle_panel_set_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings) {
        dpi_set_timings(dssdev, timings);
}

static void notle_panel_get_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings) {
        *timings = dssdev->panel.timings;
}

static int notle_panel_check_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings) {
        return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver dpi_driver = {
        .probe                 = notle_panel_probe,
        .remove                = __exit_p(notle_panel_remove),

        .enable                = notle_panel_enable,
        .disable               = notle_panel_disable,
        .suspend               = notle_panel_suspend,
        .resume                = notle_panel_resume,

        .set_timings           = notle_panel_set_timings,
        .get_timings           = notle_panel_get_timings,
        .check_timings         = notle_panel_check_timings,

        .driver                = {
                .name                  = "notle_panel",
                .owner                 = THIS_MODULE,
        },
};

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
          case 0:
            /* panel-notle-fpga */
            i2c_data->fpga_client = client;
            break;
          case 1:
            /* panel-notle-panel */
            i2c_data->panel_client = client;
            break;
          default:
            printk(KERN_WARNING "Unrecognized i2c device in Notle panel driver\n");
            return -EINVAL;
        }

        return 0;
}

/* driver remove function */
static int __devexit i2c_remove(struct i2c_client *client)
{
        struct notle_panel_i2c *drv_data = i2c_get_clientdata(client);

        /* remove client data */
        i2c_set_clientdata(client, NULL);

        /* free private data memory */
        i2c_data = NULL;
        kfree(drv_data);

        return 0;
}

static const struct i2c_device_id i2c_idtable[] = {
        {"panel-notle-fpga", 0},
        {"panel-notle-panel", 1},
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

static int __init notle_panel_drv_init(void) {
        int r = 0;
        r = i2c_add_driver(&i2c_driver);
        if (r < 0) {
                printk(KERN_WARNING "Notle panel i2c driver registration failed\n");
                return r;
        }

        return omap_dss_register_driver(&dpi_driver);
}

static void __exit notle_panel_drv_exit(void) {
        omap_dss_unregister_driver(&dpi_driver);
        i2c_del_driver(&i2c_driver);
}

module_init(notle_panel_drv_init);
module_exit(notle_panel_drv_exit);

MODULE_DESCRIPTION("Notle FPGA and Panel Driver");
MODULE_LICENSE("GPL");
