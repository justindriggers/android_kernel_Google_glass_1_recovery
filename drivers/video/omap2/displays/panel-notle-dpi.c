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

#define LOG_TAG         "panel-notle: "

#define REG_DELAY       0xFF
#define MAX_BRIGHTNESS  0xFF

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
  { REG_DELAY, 0x0A },
  { 0x00, 0x80 },
};

static struct init_register_value panel_shutdown_regs[] = {
  { 0x00, 0x85 },
  { REG_DELAY, 0x0A },
  { 0x00, 0x80 },
};

/*
 * FPGA configuration options:
 *   TEST_MONO:      Turn on all backlight LEDs at full brightness
 *                   simultaneously.  This disables color sequencing.
 *   TEST_PATTERN:   Generate a checkerboard test pattern.  This
 *                   causes OMAP pixel data output to be ignored.
 *   LED_EN:         Enable/disable the backlight.
 *   CP_SEL:         Enable/disable the charge pump on the backlight LEDs.
 *   MONO:           Enable/disable monochrome mode.  The color mix is
 *                   determined by the current value of fpga_config fields.
 *                   Note that this disables color sequencing.
 */
#define FPGA_CONFIG_MASK          ((u8)0x1F)   /* Mask of all valid config bits */
#define FPGA_CONFIG_TEST_MONO     ((u8)0x01)   /* Enable monochrome test mode */
#define FPGA_CONFIG_TEST_PATTERN  ((u8)0x02)   /* Ouput test pattern */
#define FPGA_CONFIG_LED_EN        ((u8)0x04)   /* Enable LED backlight */
#define FPGA_CONFIG_CP_SEL        ((u8)0x08)   /* Chargepump select */
#define FPGA_CONFIG_MONO          ((u8)0x10)   /* Enable monochrome mode */

struct led_config {
  unsigned red_percent;     /* 100 * percent red in output (100 = 1% red) */
  unsigned green_percent;   /* 100 * percent green in output */
  unsigned blue_percent;    /* 100 * percent blue in output */
  unsigned brightness;      /* Total brightness, max of MAX_BRIGHTNESS */
};

struct fpga_config {
  u8 config;
  u16 red;
  u16 green;
  u16 blue;
  u8 revision;  /* Read-only */
};

/* Some reasonable defaults */
static struct led_config led_config = {
  .red_percent = 2353,    /* 23.53% Red by default */
  .green_percent = 4706,  /* 47.06% Green by default */
  .blue_percent = 2941,   /* 29.41% Blue by default */
  .brightness = 128,      /* 50% brightness by default */
};

static struct fpga_config fpga_config;

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
                .pixel_clock    = 59076,

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
        int enabled;
};

static inline struct panel_notle_data
*get_panel_data(const struct omap_dss_device *dssdev) {
        return (struct panel_notle_data *) dssdev->data;
}

/* Local functions used by the sysfs interface */
static void panel_notle_power_off(struct omap_dss_device *dssdev);
static int panel_notle_power_on(struct omap_dss_device *dssdev);
static int fpga_write_config(struct fpga_config *config);
static int fpga_read_config(struct fpga_config *config);
static void led_config_to_fpga_config(struct led_config *led,
                                      struct fpga_config *fpga);

/* Sysfs interface */
static ssize_t sysfs_reset(struct notle_drv_data *notle_data,
                           const char *buf, size_t size) {
        panel_notle_power_off(notle_data->dssdev);
        notle_data->dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

        msleep(100);
        if (!panel_notle_power_on(notle_data->dssdev)) {
          notle_data->dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
        }
        return size;
}
static ssize_t fpga_revision(struct notle_drv_data *notle_data, char *buf) {
        struct fpga_config config;
        if (fpga_read_config(&config)) {
          return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "0x%x\n", config.revision);
}
static ssize_t enabled_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        notle_data->enabled);
}
static ssize_t enabled_store(struct notle_drv_data *notle_data,
                             const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        value = !!value;

        if (value) {
          if (!panel_notle_power_on(notle_data->dssdev)) {
            notle_data->dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
          }
        } else {
          panel_notle_power_off(notle_data->dssdev);
          notle_data->dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
        }

        return size;
}
static ssize_t fpga_config_show(struct notle_drv_data *notle_data, char *buf) {
        struct fpga_config config;
        if (fpga_read_config(&config)) {
          return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "0x%x/%d/%d/%d\n",
                        config.config, config.red,
                        config.green, config.blue);
}
static ssize_t fpga_config_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int config;
        unsigned int red, green, blue;

        if (sscanf(buf, "0x%x/%u/%u/%u", &config, &red, &green, &blue) != 4) {
          printk(KERN_ERR LOG_TAG "Failed to fpga_config_store: malformed config: %s\n", buf);
          return -EINVAL;
        }

        if (((config & FPGA_CONFIG_MASK) != config) ||
            (red > 511) || (green > 511) || (blue > 511)) {
          printk(KERN_ERR LOG_TAG "Failed to fpga_config_store: invalid config: %s\n", buf);
          return -EINVAL;
        }

        fpga_config.config = (u8)config;
        fpga_config.red    = (u16)red;
        fpga_config.green  = (u16)green;
        fpga_config.blue   = (u16)blue;

        if (fpga_write_config(&fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to fpga_config_store: i2c write failed\n");
          return -EIO;
        }

        return size;
}
static ssize_t testpattern_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        !!(fpga_config.config & FPGA_CONFIG_TEST_PATTERN));
}
static ssize_t testpattern_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        if (value) {
          fpga_config.config |= FPGA_CONFIG_TEST_PATTERN;
        } else {
          fpga_config.config &= ~FPGA_CONFIG_TEST_PATTERN;
        }

        if (fpga_write_config(&fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to testpattern_store: i2c write failed\n");
          return -EIO;
        }

        return size;
}
static ssize_t testmono_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        !!(fpga_config.config & FPGA_CONFIG_TEST_MONO));
}
static ssize_t testmono_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        if (value) {
          fpga_config.config |= FPGA_CONFIG_TEST_MONO;
        } else {
          fpga_config.config &= ~FPGA_CONFIG_TEST_MONO;
        }

        if (fpga_write_config(&fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to testmono_store: i2c write failed\n");
          return -EIO;
        }

        return size;
}
static ssize_t mono_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        !!(fpga_config.config & FPGA_CONFIG_MONO));
}
static ssize_t mono_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        if (value) {
          fpga_config.config |= FPGA_CONFIG_MONO;
        } else {
          fpga_config.config &= ~FPGA_CONFIG_MONO;
        }

        if (fpga_write_config(&fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to mono_store: i2c write failed\n");
          return -EIO;
        }

        return size;
}
static ssize_t brightness_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n", led_config.brightness);
}
static ssize_t brightness_store(struct notle_drv_data *notle_data,
                                const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
          return r;

        if (value < 0 || value > MAX_BRIGHTNESS) {
          printk(KERN_ERR LOG_TAG "Failed to brightness_store: "
                 "invalid brightness: %i\n", value);
          return -EINVAL;
        }

        led_config.brightness = value;
        led_config_to_fpga_config(&led_config, &fpga_config);

        /*
         * Make sure we don't turn the backlight on just because brightness
         * was set.
         */
        if (notle_data->dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
          fpga_config.config &= ~FPGA_CONFIG_LED_EN;
        }

        /*
         * If the display is enabled, write the new FPGA config immediately,
         * otherwise it will be written when the display is enabled.
         */
        if (notle_data->enabled && fpga_write_config(&fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to brightness_store: i2c write failed\n");
          return -EIO;
        }

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

static NOTLE_ATTR(reset, S_IWUSR, NULL, sysfs_reset);
static NOTLE_ATTR(fpga_revision, S_IRUGO, fpga_revision, NULL);
static NOTLE_ATTR(enabled, S_IRUGO|S_IWUSR,
                  enabled_show, enabled_store);
static NOTLE_ATTR(fpga_config, S_IRUGO|S_IWUSR,
                  fpga_config_show, fpga_config_store);
static NOTLE_ATTR(testpattern, S_IRUGO|S_IWUSR,
                  testpattern_show, testpattern_store);
static NOTLE_ATTR(testmono, S_IRUGO|S_IWUSR,
                  testmono_show, testmono_store);
static NOTLE_ATTR(mono, S_IRUGO|S_IWUSR,
                  mono_show, mono_store);
static NOTLE_ATTR(brightness, S_IRUGO|S_IWUSR,
                  brightness_show, brightness_store);

static struct attribute *panel_notle_sysfs_attrs[] = {
        &panel_notle_attr_reset.attr,
        &panel_notle_attr_fpga_revision.attr,
        &panel_notle_attr_enabled.attr,
        &panel_notle_attr_fpga_config.attr,
        &panel_notle_attr_testpattern.attr,
        &panel_notle_attr_testmono.attr,
        &panel_notle_attr_mono.attr,
        &panel_notle_attr_brightness.attr,
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
static void led_config_to_fpga_config(struct led_config *led,
                                      struct fpga_config *fpga) {
        /* TODO(madsci): Move these to be configurable or in a better place. */
        const int total_lines = 380;  /* Display height + vertical blanking */
        const int red_max_mw = 41;    /* LED power when full on */
        const int green_max_mw = 62;
        const int blue_max_mw = 62;
        const int limit_mw = 40;      /* Max backlight power */

        fpga->red =   (total_lines *
                        (10000 - (
                          (3 * led->red_percent * led->brightness * limit_mw) /
                          (red_max_mw * MAX_BRIGHTNESS)))) /
                      10000;
        fpga->green = (total_lines *
                        (10000 - (
                          (3 * led->green_percent * led->brightness * limit_mw) /
                          (green_max_mw * MAX_BRIGHTNESS)))) /
                      10000;
        fpga->blue =  (total_lines *
                        (10000 - (
                          (3 * led->blue_percent * led->brightness * limit_mw) /
                          (blue_max_mw * MAX_BRIGHTNESS)))) /
                      10000;

        /* 0 is a special case for disabling the backlight LED */
        if (led->brightness == 0) {
          fpga->config &= ~FPGA_CONFIG_LED_EN;
        } else {
          fpga->config |= FPGA_CONFIG_LED_EN;
        }

        return;
}

static int panel_write_register(u8 reg, u8 value) {
        u8 buf[2];
        int r;
        struct i2c_msg msgs[1];

        if (!i2c_data || !i2c_data->panel_client) {
                printk(KERN_ERR LOG_TAG "No I2C data set in panel_write_register\n");
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

static int fpga_read_config(struct fpga_config *config) {
        int r;
        u8 buf[5];
        struct i2c_msg msgs[1];

        if (!i2c_data || !i2c_data->fpga_client) {
                printk(KERN_ERR LOG_TAG "No I2C data set in fpga_read_config\n");
                return -1;
        }

        msgs[0].addr = i2c_data->fpga_client->addr;
        msgs[0].flags = I2C_M_RD;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(i2c_data->fpga_client->adapter, msgs, 1);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to read FPGA config: i2c read failed\n");
                return r;
        }
        config->config   = buf[0];
        config->red      = (u16)buf[1] << 1;
        config->green    = (u16)buf[2] << 1;
        config->blue     = (u16)buf[3] << 1;
        config->revision = buf[4];

        return 0;
};

static int fpga_write_config(struct fpga_config *config) {
        int r;
        struct i2c_msg msgs[1];
        u8 buf[4] = {
          config->config,
          (u8)(config->red >> 1),
          (u8)(config->green >> 1),
          (u8)(config->blue >> 1)};

        if (!i2c_data || !i2c_data->fpga_client) {
                printk(KERN_ERR LOG_TAG "No I2C data set in fpga_write_config\n");
                return -1;
        }

        msgs[0].addr = i2c_data->fpga_client->addr;
        msgs[0].flags = 0;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(i2c_data->fpga_client->adapter, msgs, 1);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to write FPGA config: i2c write failed\n");
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

        printk(KERN_INFO LOG_TAG "Powering on\n");

        r = omapdss_dpi_display_enable(dssdev);
        if (r) {
          printk(KERN_ERR LOG_TAG "Failed to enable DPI\n");
          goto err0;
        }

        if (panel_config->power_on_delay) {
          msleep(panel_config->power_on_delay);
        }

        if (panel_data->platform_enable) {
          r = panel_data->platform_enable(dssdev);
          if (r) {
            printk(KERN_ERR LOG_TAG "Failed to platform_enable\n");
            goto err1;
          }
        }

        for (i = 0; i < ARRAY_SIZE(panel_init_regs); ++i) {
          if (panel_init_regs[i].reg == REG_DELAY) {
            if (panel_data->panel_enable) {
              r = panel_data->panel_enable();
              if (r) {
                printk(KERN_ERR LOG_TAG "Failed to panel_enable\n");
                goto err1;
              }
            }
            msleep(panel_init_regs[i].value);
            continue;
          }

          if (panel_write_register(panel_init_regs[i].reg,
                                   panel_init_regs[i].value)) {
            printk(KERN_ERR LOG_TAG "Failed to write panel config\n");
            goto err2;
          }
        }

        /* Enable LED backlight if we have nonzero brightness */
        if (led_config.brightness > 0) {
          fpga_config.config |= FPGA_CONFIG_LED_EN;
          if (fpga_write_config(&fpga_config)) {
            printk(KERN_ERR LOG_TAG "Failed to enable FPGA LED_EN\n");
          }
        }

        drv_data->enabled = 1;
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
        int i;

        if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
          return;
        }

        printk(KERN_INFO LOG_TAG "Powering off\n");

        /* Disable LED backlight */
        fpga_config.config &= ~FPGA_CONFIG_LED_EN;
        if (fpga_write_config(&fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to disable FPGA LED_EN\n");
        }

        for (i = 0; i < ARRAY_SIZE(panel_shutdown_regs); ++i) {
          if (panel_shutdown_regs[i].reg == REG_DELAY) {
            msleep(panel_shutdown_regs[i].value);
            continue;
          }

          if(panel_write_register(panel_shutdown_regs[i].reg,
                                  panel_shutdown_regs[i].value)) {
            printk(KERN_ERR LOG_TAG "Failed to shutdown panel\n");
          }
        }

        /* Disable DISP_ENB */
        if (panel_data->panel_disable) {
          panel_data->panel_disable();
        }

        /* Disable LCD_RST_N */
        if (panel_data->platform_disable) {
          panel_data->platform_disable(dssdev);
        }

        if (panel_config->power_off_delay) {
          msleep(panel_config->power_off_delay);
        }

        omapdss_dpi_display_disable(dssdev);
        drv_data->enabled = 0;
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
        drv_data->enabled = 0;

        dev_set_drvdata(&dssdev->dev, drv_data);
        led_config_to_fpga_config(&led_config, &fpga_config);

        r = kobject_init_and_add(&drv_data->kobj, &panel_notle_ktype,
                        &dssdev->manager->kobj, "panel-notle-dpi");
        if (r) {
                printk(KERN_WARNING LOG_TAG "Failed to create sysfs directory\n");
        }

        return 0;
}

static void __exit panel_notle_remove(struct omap_dss_device *dssdev) {
        struct notle_drv_data *drv_data = dev_get_drvdata(&dssdev->dev);

        dev_dbg(&dssdev->dev, "remove\n");

        kobject_del(&drv_data->kobj);
        kobject_put(&drv_data->kobj);
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
            printk(KERN_WARNING LOG_TAG "Unrecognized i2c device\n");
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
                printk(KERN_WARNING LOG_TAG "I2C driver registration failed\n");
                goto err0;
        }

        r = omap_dss_register_driver(&dpi_driver);
        if (r < 0) {
                printk(KERN_WARNING LOG_TAG "DSS driver registration failed\n");
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
