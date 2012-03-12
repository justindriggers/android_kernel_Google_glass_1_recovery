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
#include <linux/spi/spi.h>

#include <video/omapdss.h>
#include <video/omap-panel-notle.h>

#define LOG_TAG         "panel-notle: "

#define REG_DELAY       0xFF
#define MAX_BRIGHTNESS  0xFF

/* iCE40 registers */
#define ICE40_REVISION   0x00

#define ICE40_PIPELINE   0x01
#define ICE40_PIPELINE_AUTO     0x70
#define ICE40_PIPELINE_TESTPAT  0x07

#define ICE40_BACKLIGHT  0x10
#define ICE40_BACKLIGHT_SYNC    0xC0
#define ICE40_BACKLIGHT_MONO    0x20
#define ICE40_BACKLIGHT_LEDEN   0x10
#define ICE40_BACKLIGHT_CPSEL   0x08
#define ICE40_BACKLIGHT_FORCER  0x04
#define ICE40_BACKLIGHT_FORCEG  0x02
#define ICE40_BACKLIGHT_FORCEB  0x01

#define ICE40_LED_RED_H    0x11
#define ICE40_LED_RED_L    0x12
#define ICE40_LED_GREEN_H  0x13
#define ICE40_LED_GREEN_L  0x14
#define ICE40_LED_BLUE_H   0x15
#define ICE40_LED_BLUE_L   0x16

typedef enum {
        UNVERSIONED = 7,
        V1_DOG      = 7,
        V3_EMU      = 0,
        V4_FLY      = 4,
        V5_GNU      = 5,
        V6_HOG      = 6,
} notle_version;

enum {
        NOTLE_I2C_FPGA  = 0,
        NOTLE_I2C_PANEL = 1,
};

struct init_register_value {
        u8 reg;
        u8 value;
};

static const u8 ice40_regs[] = {
  ICE40_REVISION,
  ICE40_PIPELINE,
  ICE40_BACKLIGHT,
  ICE40_LED_RED_H,
  ICE40_LED_RED_L,
  ICE40_LED_GREEN_H,
  ICE40_LED_GREEN_L,
  ICE40_LED_BLUE_H,
  ICE40_LED_BLUE_L,
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

static notle_version version;

/*
 * Actel FPGA configuration options:
 *   TEST_MONO:      Turn on all backlight LEDs at full brightness
 *                   simultaneously.  This disables color sequencing.
 *   TEST_PATTERN:   Generate a checkerboard test pattern.  This
 *                   causes OMAP pixel data output to be ignored.
 *   LED_EN:         Enable/disable the backlight.
 *   CP_SEL:         Enable/disable the charge pump on the backlight LEDs.
 *   MONO:           Enable/disable monochrome mode.  The color mix is
 *                   determined by the current value of actel_fpga_config fields.
 *                   Note that this disables color sequencing.
 */
#define ACTEL_FPGA_CONFIG_MASK          ((u8)0x3F)   /* Mask of all valid config bits */
#define ACTEL_FPGA_CONFIG_TEST_MONO     ((u8)0x01)   /* Enable monochrome test mode */
#define ACTEL_FPGA_CONFIG_TEST_PATTERN  ((u8)0x02)   /* Ouput test pattern */
#define ACTEL_FPGA_CONFIG_LED_EN        ((u8)0x04)   /* Enable LED backlight */
#define ACTEL_FPGA_CONFIG_CP_SEL        ((u8)0x08)   /* Chargepump select */
#define ACTEL_FPGA_CONFIG_MONO          ((u8)0x10)   /* Enable monochrome mode */
#define ACTEL_FPGA_CONFIG_OVERDRIVE     ((u8)0x20)   /* Enable brightness overdrive */

struct led_config {
  unsigned red_percent;     /* 100 * percent red in output (100 = 1% red) */
  unsigned green_percent;   /* 100 * percent green in output */
  unsigned blue_percent;    /* 100 * percent blue in output */
  unsigned brightness;      /* Total brightness, max of MAX_BRIGHTNESS */
};

struct actel_fpga_config {
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
  .brightness = 0,        /* 0 brightness by default */
};

static struct actel_fpga_config actel_fpga_config;

static struct panel_notle_busses {
        struct i2c_client *actel_fpga_client;
        struct i2c_client *panel_client;
        struct spi_device *ice40_device;
} bus_data;

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
                .pixel_clock    = 85333,

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

/* Note that this enum and the string array below must match. */
typedef enum {
        TESTPATTERN_NONE = 0,
        TESTPATTERN_COARSE_CHECK,
        TESTPATTERN_FINE_CHECK,
        TESTPATTERN_COLORBARS,
        TESTPATTERN_ALIGNMENT,
        TESTPATTERN_CALIBRATION,
        TESTPATTERN_ALL_OFF,
        TESTPATTERN_ALL_ON,
} testpattern;
static const char* const testpattern_names[] = {
        "none",
        "coarse_checkerboard",
        "fine_checkerboard",
        "color_bars",
        "alignment",
        "calibration",
        "all_px_off",
        "all_px_on",
};

struct notle_drv_data {
        struct omap_dss_device *dssdev;
        struct panel_config *panel_config;
        struct kobject kobj;
        int enabled;
        testpattern pattern;
};

static inline const char* testpattern_name(testpattern pattern) {
        return testpattern_names[pattern];
}

static inline struct panel_notle_data
*get_panel_data(const struct omap_dss_device *dssdev) {
        return (struct panel_notle_data *) dssdev->data;
}

/* Local functions used by the sysfs interface */
static int fpga_rev = -1;
static void panel_notle_power_off(struct omap_dss_device *dssdev);
static int panel_notle_power_on(struct omap_dss_device *dssdev);
static int actel_fpga_write_config(struct actel_fpga_config *config);
static int actel_fpga_read_config(struct actel_fpga_config *config);
static void ice40_dump_regs(void);
static int ice40_read_register(u8 reg_addr);
static int ice40_write_register(u8 reg_addr, u8 reg_value);
static int ice40_set_backlight(int led_en, int r, int g, int b);
static int fpga_read_revision(void);
static void led_config_to_fpga_config(struct led_config *led,
                                      struct actel_fpga_config *fpga);
static void led_config_to_linecuts(struct omap_dss_device *dssdev,
                                   struct led_config *led, int *red_linecut,
                                   int *green_linecut, int *blue_linecut);

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
        int rev = fpga_read_revision();

        ice40_dump_regs();

        if (rev < 0) {
          printk(KERN_ERR LOG_TAG "Failed to read FPGA revision\n");
        } else {
          /*
           * Cache the fpga revision so we can still print this when
           * the panel is powered off.
           */
           fpga_rev = rev;
        }

        if (fpga_rev < 0) {
          printk(KERN_ERR LOG_TAG "No cached FPGA revision\n");
          return -EIO;
        }

        return snprintf(buf, PAGE_SIZE, "0x%02x\n", fpga_rev);
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

static u8 reg_addr = 0;
static ssize_t reg_addr_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "0x%02x\n",
                        reg_addr);
}
static ssize_t reg_addr_store(struct notle_drv_data *notle_data,
                              const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        reg_addr = (u8)(value & 0xff);

        return size;
}
static ssize_t reg_value_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "0x%02x\n",
                        ice40_read_register(reg_addr));
}
static ssize_t reg_value_store(struct notle_drv_data *notle_data,
                               const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        ice40_write_register(reg_addr, (u8)(value & 0xff));

        return size;
}
static ssize_t actel_fpga_config_show(struct notle_drv_data *notle_data, char *buf) {
        struct actel_fpga_config config;
        if (actel_fpga_read_config(&config)) {
          return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "0x%x/%d/%d/%d\n",
                        config.config, config.red,
                        config.green, config.blue);
}
static ssize_t actel_fpga_config_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int config;
        unsigned int red, green, blue;

        if (sscanf(buf, "0x%x/%u/%u/%u", &config, &red, &green, &blue) != 4) {
          printk(KERN_ERR LOG_TAG "Failed to actel_fpga_config_store: malformed config: %s\n", buf);
          return -EINVAL;
        }

        if (((config & ACTEL_FPGA_CONFIG_MASK) != config) ||
            (red > 511) || (green > 511) || (blue > 511)) {
          printk(KERN_ERR LOG_TAG "Failed to actel_fpga_config_store: invalid config: %s\n", buf);
          return -EINVAL;
        }

        actel_fpga_config.config = (u8)config;
        actel_fpga_config.red    = (u16)red;
        actel_fpga_config.green  = (u16)green;
        actel_fpga_config.blue   = (u16)blue;

        if (actel_fpga_write_config(&actel_fpga_config)) {
          printk(KERN_ERR LOG_TAG "Failed to actel_fpga_config_store: i2c write failed\n");
          return -EIO;
        }

        return size;
}
static ssize_t list_testpatterns(struct notle_drv_data *notle_data, char *buf) {
        int i;

        *buf = '\0';
        for (i = 0; i < sizeof(testpattern_names) / sizeof(testpattern_names[0]); ++i) {
                if (strlen(buf) + strlen(testpattern_names[i]) + 2 > PAGE_SIZE) {
                        return -EINVAL;
                }
                strcat(buf, testpattern_names[i]);
                strcat(buf, "\n");
        }
        return strlen(buf);
}
static ssize_t testpattern_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%s\n", testpattern_name(notle_data->pattern));
}
static ssize_t testpattern_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int i;
        char value[128];

        if (size > 128) return -EINVAL;

        sscanf(buf, "%s", value);

        for (i = 0; i < sizeof(testpattern_names) / sizeof(testpattern_names[0]); i++) {
                if (!strncmp(value, testpattern_names[i], size)) {
                        notle_data->pattern = (testpattern)i;
                }
        }

        switch (version) {
          case V4_FLY:
          case V5_GNU:
                if (notle_data->pattern == TESTPATTERN_NONE) {
                        actel_fpga_config.config &= ~ACTEL_FPGA_CONFIG_TEST_PATTERN;
                } else {
                        actel_fpga_config.config |= ACTEL_FPGA_CONFIG_TEST_PATTERN;
                }

                if (actel_fpga_write_config(&actel_fpga_config)) {
                        printk(KERN_ERR LOG_TAG "Failed to testpattern_store: "
                               "i2c write failed\n");
                        return -EIO;
                }
                break;
          case V6_HOG:

                i = ice40_read_register(ICE40_PIPELINE);
                if (i < 0) {
                        printk(KERN_ERR LOG_TAG "Failed to testpattern_store: "
                               "register read failed: %i\n", i);
                        return -EIO;
                }
                i = (i & ~ICE40_PIPELINE_TESTPAT) | notle_data->pattern;
                if ((i = ice40_write_register(ICE40_PIPELINE, (u8)(i & 0xff))) < 0) {
                        printk(KERN_ERR LOG_TAG "Failed to testpattern_store: "
                               "register write failed: %i\n", i);
                        return -EIO;
                }
                break;
          case V1_DOG:
          case V3_EMU:
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        return size;
}
static ssize_t testmono_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        !!(actel_fpga_config.config & ACTEL_FPGA_CONFIG_TEST_MONO));
}
static ssize_t testmono_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        if (value) {
          actel_fpga_config.config |= ACTEL_FPGA_CONFIG_TEST_MONO;
        } else {
          actel_fpga_config.config &= ~ACTEL_FPGA_CONFIG_TEST_MONO;
        }

        switch (version) {
          case V4_FLY:
          case V5_GNU:
                if (actel_fpga_write_config(&actel_fpga_config)) {
                        printk(KERN_ERR LOG_TAG "Failed to testmono_store: "
                               "i2c write failed\n");
                        return -EIO;
                }
                break;
          case V6_HOG:
                value = ice40_read_register(ICE40_BACKLIGHT);
                if (value < 0) {
                        printk(KERN_ERR LOG_TAG "Failed to testmono_store: "
                               "spi read failed: %i\n", value);
                        return -EIO;
                }
                if (actel_fpga_config.config & ACTEL_FPGA_CONFIG_TEST_MONO) {
                        value |= ICE40_BACKLIGHT_FORCER |
                                 ICE40_BACKLIGHT_FORCEG |
                                 ICE40_BACKLIGHT_FORCEB;
                } else {
                        value &= ~(ICE40_BACKLIGHT_FORCER |
                                   ICE40_BACKLIGHT_FORCEG |
                                   ICE40_BACKLIGHT_FORCEB);
                }
                value = ice40_write_register(ICE40_BACKLIGHT, (u8)(value & 0xff));
                if (value < 0) {
                        printk(KERN_ERR LOG_TAG "Failed to testmono_store: "
                               "spi write failed: %i\n", value);
                        return -EIO;
                }
                break;
          case V1_DOG:
          case V3_EMU:
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        return size;
}
static ssize_t forcer_show(struct notle_drv_data *notle_data, char *buf) {
        int val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        val = ice40_read_register(ICE40_BACKLIGHT);
        if (val < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forcer_store: "
                       "spi read failed: %i\n", val);
                return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "%d\n",
                !!(val & ICE40_BACKLIGHT_FORCER));
}
static ssize_t forcer_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        r = kstrtoint(buf, 0, &val);
        if (r)
                return r;

        r = ice40_read_register(ICE40_BACKLIGHT);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forcer_store: "
                       "spi read failed: %i\n", r);
                return -EIO;
        }

        if (val) {
                r |= ICE40_BACKLIGHT_FORCER;
        } else {
                r &= ~ICE40_BACKLIGHT_FORCER;
        }

        r = ice40_write_register(ICE40_BACKLIGHT, r);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forcer_store: "
                       "spi write failed: %i\n", r);
                return -EIO;
        }

        return size;
}
static ssize_t forceg_show(struct notle_drv_data *notle_data, char *buf) {
        int val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        val = ice40_read_register(ICE40_BACKLIGHT);
        if (val < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forceg_store: "
                       "spi read failed: %i\n", val);
                return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "%d\n",
                !!(val & ICE40_BACKLIGHT_FORCEG));
}
static ssize_t forceg_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        r = kstrtoint(buf, 0, &val);
        if (r)
                return r;

        r = ice40_read_register(ICE40_BACKLIGHT);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forceg_store: "
                       "spi read failed: %i\n", r);
                return -EIO;
        }

        if (val) {
                r |= ICE40_BACKLIGHT_FORCEG;
        } else {
                r &= ~ICE40_BACKLIGHT_FORCEG;
        }

        r = ice40_write_register(ICE40_BACKLIGHT, r);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forceg_store: "
                       "spi write failed: %i\n", r);
                return -EIO;
        }

        return size;
}
static ssize_t forceb_show(struct notle_drv_data *notle_data, char *buf) {
        int val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        val = ice40_read_register(ICE40_BACKLIGHT);
        if (val < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forceb_store: "
                       "spi read failed: %i\n", val);
                return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "%d\n",
                !!(val & ICE40_BACKLIGHT_FORCEB));
}
static ssize_t forceb_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        r = kstrtoint(buf, 0, &val);
        if (r)
                return r;

        r = ice40_read_register(ICE40_BACKLIGHT);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forceb_store: "
                       "spi read failed: %i\n", r);
                return -EIO;
        }

        if (val) {
                r |= ICE40_BACKLIGHT_FORCEB;
        } else {
                r &= ~ICE40_BACKLIGHT_FORCEB;
        }

        r = ice40_write_register(ICE40_BACKLIGHT, r);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to forceb_store: "
                       "spi write failed: %i\n", r);
                return -EIO;
        }

        return size;
}
static ssize_t cpsel_show(struct notle_drv_data *notle_data, char *buf) {
        int val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        val = ice40_read_register(ICE40_BACKLIGHT);
        if (val < 0) {
                printk(KERN_ERR LOG_TAG "Failed to cpsel_store: "
                       "spi read failed: %i\n", val);
                return -EIO;
        }
        return snprintf(buf, PAGE_SIZE, "%d\n",
                !!(val & ICE40_BACKLIGHT_CPSEL));
}
static ssize_t cpsel_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, val;

        if (version != V6_HOG) {
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                return -EINVAL;
        }

        r = kstrtoint(buf, 0, &val);
        if (r)
                return r;

        r = ice40_read_register(ICE40_BACKLIGHT);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to cpsel_store: "
                       "spi read failed: %i\n", r);
                return -EIO;
        }

        if (val) {
                r |= ICE40_BACKLIGHT_CPSEL;
        } else {
                r &= ~ICE40_BACKLIGHT_CPSEL;
        }

        r = ice40_write_register(ICE40_BACKLIGHT, r);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to cpsel_store: "
                       "spi write failed: %i\n", r);
                return -EIO;
        }

        return size;
}
static ssize_t mono_show(struct notle_drv_data *notle_data, char *buf) {
        int val;

        switch (version) {
            case V4_FLY:
            case V5_GNU:
              val = !!(actel_fpga_config.config & ACTEL_FPGA_CONFIG_MONO);
              break;
            case V6_HOG:
              val = ice40_read_register(ICE40_BACKLIGHT);
              if (val < 0) {
                printk(KERN_ERR LOG_TAG "Failed to read iCE40 register: "
                       "0x%02x\n", ICE40_BACKLIGHT);
                return -EIO;
              }
              val = !!(val & ICE40_BACKLIGHT_MONO);
              break;
            default:
              printk(KERN_ERR LOG_TAG "Unsupported Notle version: %d\n", version);
              return -EINVAL;
        }

        return snprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t mono_store(struct notle_drv_data *notle_data,
                                 const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;

        switch (version) {
            case V4_FLY:
            case V5_GNU:
              if (value) {
                actel_fpga_config.config |= ACTEL_FPGA_CONFIG_MONO;
              } else {
                actel_fpga_config.config &= ~ACTEL_FPGA_CONFIG_MONO;
              }
              if (actel_fpga_write_config(&actel_fpga_config)) {
                printk(KERN_ERR LOG_TAG "Failed to mono_store: i2c write failed\n");
                return -EIO;
              }
              break;
            case V6_HOG:
              r = ice40_read_register(ICE40_BACKLIGHT);
              if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to read iCE40 register: "
                       "0x%02x\n", ICE40_BACKLIGHT);
                return -EIO;
              }
              if (value) {
                r |= ICE40_BACKLIGHT_MONO;
              } else {
                r &= ~ICE40_BACKLIGHT_MONO;
              }
              r = ice40_write_register(ICE40_BACKLIGHT, r);
              if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to write iCE40 register: "
                       "0x%02x\n", ICE40_BACKLIGHT);
                return -EIO;
              }
              break;
            default:
              printk(KERN_ERR LOG_TAG "Unsupported Notle version: %d\n", version);
              return -EINVAL;
        }

        return size;
}
static ssize_t brightness_show(struct notle_drv_data *notle_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n", led_config.brightness);
}
static ssize_t brightness_store(struct notle_drv_data *notle_data,
                                const char *buf, size_t size) {
        int r, g, b, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
          return r;

        if (value < 0 || value > MAX_BRIGHTNESS) {
          printk(KERN_ERR LOG_TAG "Failed to brightness_store: "
                 "invalid brightness: %i\n", value);
          return -EINVAL;
        }

        led_config.brightness = value;

        /*
         * If the display is enabled, write the new FPGA config immediately,
         * otherwise it will be written when the display is enabled.
         */
        if (notle_data->enabled) {
          switch (version) {
            case V4_FLY:
            case V5_GNU:
              led_config_to_fpga_config(&led_config, &actel_fpga_config);
              if (actel_fpga_write_config(&actel_fpga_config)) {
                printk(KERN_ERR LOG_TAG "Failed to brightness_store: i2c write failed\n");
                return -EIO;
              }
              break;
            case V6_HOG:
              if (led_config.brightness) {
                led_config_to_linecuts(notle_data->dssdev, &led_config, &r, &g, &b);
                if (ice40_set_backlight(1, r, g, b)) {
                  printk(KERN_ERR LOG_TAG "Failed to brightness_store: spi write failed\n");
                }
              } else {
                if (ice40_set_backlight(0, -1, -1, -1)) {
                  printk(KERN_ERR LOG_TAG "Failed to brightness_store: spi write failed\n");
                }
              }
              break;
            default:
              printk(KERN_ERR LOG_TAG "Unsupported Notle version: %d\n", version);
              break;
          }
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
static NOTLE_ATTR(list_testpatterns, S_IRUGO, list_testpatterns, NULL);
static NOTLE_ATTR(enabled, S_IRUGO|S_IWUSR,
                  enabled_show, enabled_store);
static NOTLE_ATTR(reg_addr, S_IRUGO|S_IWUSR,
                  reg_addr_show, reg_addr_store);
static NOTLE_ATTR(reg_value, S_IRUGO|S_IWUSR,
                  reg_value_show, reg_value_store);
static NOTLE_ATTR(actel_fpga_config, S_IRUGO|S_IWUSR,
                  actel_fpga_config_show, actel_fpga_config_store);
static NOTLE_ATTR(testpattern, S_IRUGO|S_IWUSR,
                  testpattern_show, testpattern_store);
static NOTLE_ATTR(testmono, S_IRUGO|S_IWUSR,
                  testmono_show, testmono_store);
static NOTLE_ATTR(forcer, S_IRUGO|S_IWUSR,
                  forcer_show, forcer_store);
static NOTLE_ATTR(forceg, S_IRUGO|S_IWUSR,
                  forceg_show, forceg_store);
static NOTLE_ATTR(forceb, S_IRUGO|S_IWUSR,
                  forceb_show, forceb_store);
static NOTLE_ATTR(cpsel, S_IRUGO|S_IWUSR,
                  cpsel_show, cpsel_store);
static NOTLE_ATTR(mono, S_IRUGO|S_IWUSR,
                  mono_show, mono_store);
static NOTLE_ATTR(brightness, S_IRUGO|S_IWUSR,
                  brightness_show, brightness_store);

static struct attribute *panel_notle_sysfs_attrs[] = {
        &panel_notle_attr_reset.attr,
        &panel_notle_attr_fpga_revision.attr,
        &panel_notle_attr_list_testpatterns.attr,
        &panel_notle_attr_enabled.attr,
        &panel_notle_attr_reg_addr.attr,
        &panel_notle_attr_reg_value.attr,
        &panel_notle_attr_actel_fpga_config.attr,
        &panel_notle_attr_testpattern.attr,
        &panel_notle_attr_testmono.attr,
        &panel_notle_attr_forcer.attr,
        &panel_notle_attr_forceg.attr,
        &panel_notle_attr_forceb.attr,
        &panel_notle_attr_cpsel.attr,
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
static void led_config_to_linecuts(struct omap_dss_device *dssdev,
                                   struct led_config *led, int *red_linecut,
                                   int *green_linecut, int *blue_linecut) {
        int total_lines = dssdev->panel.timings.y_res +
            dssdev->panel.timings.vfp +
            dssdev->panel.timings.vsw +
            dssdev->panel.timings.vbp;
        struct panel_notle_data *panel_data = get_panel_data(dssdev);

        *red_linecut   = (total_lines *
                          (10000 - (
                           (3 * led->red_percent * led->brightness * panel_data->limit_mw) /
                           (panel_data->red_max_mw * MAX_BRIGHTNESS)))) /
                         10000;
        *green_linecut = (total_lines *
                          (10000 - (
                           (3 * led->green_percent * led->brightness * panel_data->limit_mw) /
                           (panel_data->green_max_mw * MAX_BRIGHTNESS)))) /
                         10000;
        *blue_linecut  = (total_lines *
                          (10000 - (
                           (3 * led->blue_percent * led->brightness * panel_data->limit_mw) /
                           (panel_data->blue_max_mw * MAX_BRIGHTNESS)))) /
                         10000;

        /*
         * This will cause a slight color shift at very dim brightness values,
         * but the altnerative is to cause a sudden color shift by dropping
         * the lowest LED entirely.  This is a side effect of the way the fpga
         * is implemented - there's no way to dim a color channel less than a
         * single line.
         */
        if (*red_linecut > dssdev->panel.timings.y_res - 2)
          *red_linecut = dssdev->panel.timings.y_res - 2;
        if (*green_linecut > dssdev->panel.timings.y_res - 2)
          *green_linecut = dssdev->panel.timings.y_res - 2;
        if (*blue_linecut > dssdev->panel.timings.y_res - 2)
          *blue_linecut = dssdev->panel.timings.y_res - 2;

        return;
}

static void led_config_to_fpga_config(struct led_config *led,
                                      struct actel_fpga_config *fpga) {
        int total_lines;
        int red_max_mw;    /* LED power when full on */
        int green_max_mw;
        int blue_max_mw;
        int limit_mw;      /* Max backlight power */

        /*
         * See arch/arm/mach-omap2/board-notle.c for the
         * enum with these values.
         */
        switch (version) {
          case V4_FLY:
            red_max_mw = 41;
            green_max_mw = 62;
            blue_max_mw = 62;
            limit_mw = 40;
            break;
          case V5_GNU:
            red_max_mw = 63;
            green_max_mw = 96;
            blue_max_mw = 96;
            limit_mw = 60;
            break;
          default:  /* Some reasonable defaults */
            printk(KERN_ERR LOG_TAG "Unsupported notle_version in "
                   "led_config_to_fpga_config: %d\n", version);
            return;
        };

        switch (fpga->revision & 0xF0) {
          case 0x10:
            total_lines = 380;  /* Display height + vertical blanking */
            break;
          case 0x20:
            total_lines = 358;  /* Display height - 2 */
            break;
          default:
            total_lines = 358;  /* Default to minimum of above cases */
            printk(KERN_ERR LOG_TAG "Unrecognized FPGA revision: 0x%02x\n", fpga->revision);
            break;
        }

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
          fpga->config &= ~ACTEL_FPGA_CONFIG_LED_EN;
        } else {
          fpga->config |= ACTEL_FPGA_CONFIG_LED_EN;
        }

        return;
}

static int panel_write_register(u8 reg, u8 value) {
        u8 buf[2];
        int r;
        struct i2c_msg msgs[1];

        if (!bus_data.panel_client) {
                printk(KERN_ERR LOG_TAG "No I2C data set in panel_write_register\n");
                return -1;
        }

        buf[0] = reg;
        buf[1] = value;

        msgs[0].addr = bus_data.panel_client->addr;
        msgs[0].flags = 0;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(bus_data.panel_client->adapter, msgs, 1);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to write 0x%02x to panel "
                       "register 0x%02x: %i\n", value, reg, r);
                return r;
        }

        return 0;
}

static int ice40_read_register(u8 reg_addr) {
  int val;

  if (!bus_data.ice40_device) {
    printk(KERN_ERR LOG_TAG "No iCE40 bus data set in ice40_read_register()\n");
    return -1;
  }
  val = spi_w8r8(bus_data.ice40_device, (reg_addr & 0x7f));
  return val;
}

static int ice40_write_register(u8 reg_addr, u8 reg_value) {
  u8 buf[] = {reg_addr | 0x80, reg_value};

  if (!bus_data.ice40_device) {
    printk(KERN_ERR LOG_TAG "No iCE40 bus data set in ice40_read_register()\n");
    return -1;
  }
  return spi_write(bus_data.ice40_device, buf, sizeof(buf));
}

static void ice40_dump_regs(void) {
  int i, val;

  printk(KERN_INFO LOG_TAG "*** iCE40 Register Dump ***\n");

  for (i = 0; i < sizeof(ice40_regs); ++i) {
    val = ice40_read_register(ice40_regs[i]);
    if (val < 0) {
      printk(KERN_INFO LOG_TAG "  0x%02x: FAILED\n", ice40_regs[i]);
    } else {
      printk(KERN_INFO LOG_TAG "  0x%02x: 0x%02x\n",
             ice40_regs[i], (u8)(val & 0xff));
    }
  }

  return;
}

/*
 * Set backlight parameters.  Pass -1 to any argument to ignore that value and
 * not set it in the relevant register.
 */
static int ice40_set_backlight(int led_en, int r, int g, int b) {
  int val;
  int ret = 0;

  ice40_read_register(ICE40_BACKLIGHT);

  if (r > -1) {
    ret |= ice40_write_register(ICE40_LED_RED_H, (r & 0xff00) >> 8);
    ret |= ice40_write_register(ICE40_LED_RED_L, (r & 0xff));
  }
  if (g > -1) {
    ret |= ice40_write_register(ICE40_LED_GREEN_H, (g & 0xff00) >> 8);
    ret |= ice40_write_register(ICE40_LED_GREEN_L, (g & 0xff));
  }
  if (b > -1) {
    ret |= ice40_write_register(ICE40_LED_BLUE_H, (b & 0xff00) >> 8);
    ret |= ice40_write_register(ICE40_LED_BLUE_L, (b & 0xff));
  }

  if (led_en > -1) {
    val = ice40_read_register(ICE40_BACKLIGHT);
    if (val < 0) {
      ret |= val;
    } else if (led_en) {
      val |= ICE40_BACKLIGHT_LEDEN;
    } else {
      val &= ~ICE40_BACKLIGHT_LEDEN;
    }
    ret |= ice40_write_register(ICE40_BACKLIGHT, val);
  }

  return ret;
}

static int fpga_read_revision(void) {
      struct actel_fpga_config actel_config;
      int r, rev = -1;

      switch (version) {
        case V4_FLY:
        case V5_GNU:
                if ((r = actel_fpga_read_config(&actel_config)) < 0) {
                        printk(KERN_ERR LOG_TAG "Failed to read actel FPGA config: %i\n", r);
                        break;
                }
                rev = actel_config.revision;
                break;
        case V6_HOG:
                if ((r = ice40_read_register(ICE40_REVISION)) < 0) {
                        printk(KERN_ERR LOG_TAG "Failed to read iCE40 FPGA config: %i\n", r);
                        break;
                }
                rev = r;

                break;
        case V1_DOG:
        case V3_EMU:
                printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                       version);
                break;
        }

        return rev;
}

static int actel_fpga_read_config(struct actel_fpga_config *config) {
        int r;
        u8 buf[5];
        struct i2c_msg msgs[1];

        if (!bus_data.actel_fpga_client) {
                printk(KERN_ERR LOG_TAG "No I2C data set in actel_fpga_read_config()\n");
                return -1;
        }

        msgs[0].addr = bus_data.actel_fpga_client->addr;
        msgs[0].flags = I2C_M_RD;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(bus_data.actel_fpga_client->adapter, msgs, 1);
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

static int actel_fpga_write_config(struct actel_fpga_config *config) {
        int r;
        struct i2c_msg msgs[1];
        u8 buf[4] = {
          config->config,
          (u8)(config->red >> 1),
          (u8)(config->green >> 1),
          (u8)(config->blue >> 1)};

        if (!bus_data.actel_fpga_client) {
                printk(KERN_ERR LOG_TAG "No I2C data set in actel_fpga_write_config\n");
                return -1;
        }

        msgs[0].addr = bus_data.actel_fpga_client->addr;
        msgs[0].flags = 0;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(bus_data.actel_fpga_client->adapter, msgs, 1);
        if (r < 0) {
                printk(KERN_ERR LOG_TAG "Failed to write FPGA config: i2c write failed\n");
                return r;
        }

        return 0;
};

/* Functions to perform actions on the panel and DSS driver */
static int panel_notle_power_on(struct omap_dss_device *dssdev) {
        int i, r, g, b;
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

          r = panel_write_register(panel_init_regs[i].reg,
                                   panel_init_regs[i].value);
          if (r) {
            printk(KERN_ERR LOG_TAG "Failed to write panel config via i2c: "
                   "%d\n", r);
            goto err1;
          }
        }

        /* Load defaults */
        switch (version) {
          case V6_HOG:
            ice40_write_register(ICE40_BACKLIGHT, ICE40_BACKLIGHT_CPSEL);
            break;
          default:
            break;
        }

        /* Enable LED backlight if we have nonzero brightness */
        if (led_config.brightness > 0) {
          switch (version) {
            case V4_FLY:
            case V5_GNU:
                  /*
                   * We need to read the fpga config first to get the revision,
                   * as it is required to do the led_config -> actel_fpga_config
                   * conversion.
                   */
                  if (fpga_read_revision() < 0) {
                    printk(KERN_ERR LOG_TAG "Failed to read FPGA revision, not "
                                            "enabling backlight\n");
                  } else {
                    led_config_to_fpga_config(&led_config, &actel_fpga_config);
                    actel_fpga_config.config |= ACTEL_FPGA_CONFIG_LED_EN;
                    if (actel_fpga_write_config(&actel_fpga_config)) {
                      printk(KERN_ERR LOG_TAG "Failed to enable FPGA LED_EN\n");
                    }
                  }
                  break;
            case V6_HOG:
                  led_config_to_linecuts(dssdev, &led_config, &r, &g, &b);
                  ice40_set_backlight(1, r, g, b);
                  break;
            default:
                  printk(KERN_ERR LOG_TAG "Unsupported Notle version: 0x%02x\n",
                         version);
                  break;
          }
        } else if ((version != V6_HOG) && (fpga_read_revision() < 0)) {
            printk(KERN_ERR LOG_TAG "Failed to read FPGA revision\n");
        }

        drv_data->enabled = 1;
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
        switch (version) {
          case V4_FLY:
          case V5_GNU:
            actel_fpga_config.config &= ~ACTEL_FPGA_CONFIG_LED_EN;
            if (actel_fpga_write_config(&actel_fpga_config)) {
              printk(KERN_ERR LOG_TAG "Failed to disable actel FPGA LED_EN\n");
            }
            break;
          case V6_HOG:
            /* Don't change the color mix, just disable the backlight. */
            if (ice40_set_backlight(0, -1, -1, -1)) {
              printk(KERN_ERR LOG_TAG "Failed to disable iCE40 FPGA LED_EN\n");
            }
            break;
          default:
            printk(KERN_ERR LOG_TAG "Unrecognized Notle version: %d\n", version);
            break;
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
        struct panel_notle_data *panel_data = get_panel_data(dssdev);
        struct notle_drv_data *drv_data = NULL;

        dev_dbg(&dssdev->dev, "probe\n");

        version = panel_data->notle_version;

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
                               const struct i2c_device_id *id) {
        switch (id->driver_data) {
          case NOTLE_I2C_FPGA:
            /* panel-notle-fpga */
            bus_data.actel_fpga_client = client;
            break;
          case NOTLE_I2C_PANEL:
            /* panel-notle-panel */
            bus_data.panel_client = client;
            break;
          default:
            printk(KERN_WARNING LOG_TAG "Unrecognized i2c device\n");
            return -EINVAL;
        }

        return 0;
}

static int __devexit i2c_remove(struct i2c_client *client) {
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

/* SPI Interface for iCE40 FPGA based systems */
static const struct spi_device_id spi_idtable[] = {
        {"ice40-spi", 0},
        {},
};

static int ice40_spi_probe(struct spi_device *spi) {
        spi->mode = SPI_MODE_3;
        spi->bits_per_word = 8;
        spi_setup(spi);

        bus_data.ice40_device = spi;
        return 0;
}

static int ice40_spi_remove(struct spi_device *spi) {
        return 0;
}

static int ice40_spi_suspend(struct spi_device *spi, pm_message_t mesg) {
        return 0;
}

static int ice40_spi_resume(struct spi_device *spi) {
        spi_setup(spi);
        return 0;
}

static struct spi_driver spi_driver = {
        .id_table             = spi_idtable,
        .probe                = ice40_spi_probe,
        .remove               = __devexit_p(ice40_spi_remove),
        .suspend              = ice40_spi_suspend,
        .resume               = ice40_spi_resume,
        .driver               = {
                .name             = "ice40-spi",
                .bus              = &spi_bus_type,
                .owner            = THIS_MODULE,
        },
};

static int __init panel_notle_drv_init(void) {
        int r = 0;
        r = i2c_add_driver(&i2c_driver);
        if (r < 0) {
                printk(KERN_WARNING LOG_TAG "I2C driver registration failed\n");
                goto err0;
        }

        r = spi_register_driver(&spi_driver);
        if (r < 0) {
                printk(KERN_WARNING LOG_TAG "SPI driver registration failed\n");
                goto err1;
        }

        r = omap_dss_register_driver(&dpi_driver);
        if (r < 0) {
                printk(KERN_WARNING LOG_TAG "DSS driver registration failed\n");
                goto err2;
        }

        return 0;

err2:
        spi_unregister_driver(&spi_driver);
err1:
        i2c_del_driver(&i2c_driver);
err0:
        return r;
}

static void __exit panel_notle_drv_exit(void) {
        omap_dss_unregister_driver(&dpi_driver);
        i2c_del_driver(&i2c_driver);
        spi_unregister_driver(&spi_driver);
}

module_init(panel_notle_drv_init);
module_exit(panel_notle_drv_exit);

MODULE_DESCRIPTION("Notle FPGA and Panel Driver");
MODULE_LICENSE("GPL");
