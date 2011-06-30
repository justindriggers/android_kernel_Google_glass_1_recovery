/*
 * Board support file for OMAP4430 based EltonBoard.
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: David Anders <x0132446@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>

#include <linux/i2c/l3g4200d.h>
#include <linux/i2c/lsm303dlhc.h>
#include <linux/i2c/ltr506.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include "../../../drivers/input/touchscreen/rmi_i2c.h"

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/dmtimer.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/dmtimer-pwm.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

#include "timer-gp.h"
#include "hsmmc.h"
#include "control.h"
#include "common-board-devices.h"
#include "mux.h"

#define MUX(x) OMAP4_CTRL_MODULE_PAD_##x##_OFFSET

#define GPIO_GREEN_LED                  7
#define MUX_GREEN_LED                   MUX(FREF_CLK4_REQ)
#define GPIO_YELLOW_LED                 8
#define MUX_YELLOW_LED                  MUX(FREF_CLK4_OUT)
#define GPIO_WL_RST_N                   43
#define MUX_WL_RST_N                    MUX(GPMC_A19)
#define GPIO_BT_RST_N                   151
#define MUX_BT_RST_N                    MUX(MCSPI4_CLK)
#define GPIO_WL_BT_REG_ON               48
#define MUX_WL_BT_REG_ON                MUX(GPMC_A24)
#define GPIO_GPS_ON_OFF                 49
#define MUX_GPS_ON_OFF                  MUX(GPMC_A25)
#define GPIO_GPS_RESET_N                52
#define MUX_GPS_RESET_N                 MUX(GPMC_NCS2)
#define GPIO_LCD_RESET_N                53
#define MUX_LCD_RESET_N                 MUX(GPMC_NCS3)
#define GPIO_AUDIO_POWERON              62
#define MUX_AUDIO_POWERON               MUX(GPMC_WAIT1)
#define GPIO_EN_10V                     84
#define MUX_EN_10V                      MUX(USBB1_ULPITLL_CLK)

// Notle v2 wifi
#define GPIO_BCM_WLAN_HOST_WAKE         170
#define MUX_BCM_WLAN_HOST_WAKE          MUX(USBB2_HSIC_STROBE)
#define GPIO_BCM_BT_HOST_WAKE           154
#define MUX_BCM_BT_HOST_WAKE            MUX(MCSPI4_CS0)
#define GPIO_BCM_WLAN_WAKE              97
#define MUX_BCM_WLAN_WAKE               MUX(USBB1_HSIC_STROBE)
#define GPIO_BCM_BT_WAKE                36
#define MUX_BCM_BT_WAKE                 MUX(GPMC_AD12)

#if 0
// Notle v1 wifi
#define GPIO_BCM_WLAN_HOST_WAKE         86
#define MUX_BCM_WLAN_HOST_WAKE          MUX(USBB1_ULPITLL_DIR)
#define GPIO_BCM_BT_HOST_WAKE           87
#define MUX_BCM_BT_HOST_WAKE            MUX(USBB1_ULPITLL_NXT)
#define GPIO_BCM_WLAN_WAKE              88
#define MUX_BCM_WLAN_WAKE               MUX(USBB1_ULPITLL_DAT0)
#define GPIO_BCM_BT_WAKE                91
#define MUX_BCM_BT_WAKE                 MUX(USBB1_ULPITLL_DAT3)
#endif

#define MUX_BACKLIGHT                   MUX(USBB1_ULPITLL_DAT5)
#define PWM_TIMER                       9

/*
#define GPIO_HUB_POWER		                1
#define ELTON_DVI_TFP410_POWER_DOWN_GPIO        53
#define GPIO_HUB_NRESET		                62
#define ELTON_EN_10V                            84
#define GPIO_BCM_BT_WAKE                        91
#define GPIO_BCM_BT_HOST_WAKE                   87
#define GPIO_BCM_WLAN_WAKE                      88
#define GPIO_BCM_WLAN_HOST_WAKE                 86
*/

static struct gpio_led gpio_leds[] = {
	{
		.name			= "notleboard::status1",
		.default_trigger	= "heartbeat",
		.gpio			= 7,
	},
	{
		.name			= "notleboard::status2",
		.default_trigger	= "mmc0",
		.gpio			= 8,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

/* The broadcom 4329 driver makes calls like this to get the irq for the device:

        wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "bcm4329_wlan_irq");
...
        *irq_flags_ptr = wifi_irqres->flags & IRQF_TRIGGER_MASK;
        return (int)wifi_irqres->start;

   Does this name go in the .name field of the resource or the platform_device?
*/

static struct resource notle_wifi_resources = {
        .name = "bcm4329_wlan_irq",
        .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
};

static struct platform_device notle_wifi_device = {
	.name	= "bcm4329_wlan",
	.id	= -1,
        .num_resources = 1,
        .resource = &notle_wifi_resources,
};

static void __init notle_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

static int display_set_config(void);

/* Display DVI */
static int notle_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 1);
        display_set_config();
	return 0;
}

static void notle_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 0);
}

/* Using generic display panel */
static struct panel_generic_dpi_data dvi_panel = {
	.name			= "generic-wingman",
	.platform_enable	= notle_enable_dvi,
	.platform_disable	= notle_disable_dvi,
};

struct omap_dss_device notle_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "notle_generic_panel",
	.driver_name		= "generic_dpi_panel",
	.data			= &dvi_panel,
	.phy.dpi.data_lines	= 24,
	.reset_gpio		= GPIO_LCD_RESET_N,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static struct i2c_client *himax_client;

static int display_set_config(void)
{
        int ret = 0;

        // Return a sensible error here:
        if (!himax_client)
                return -1;

#define DOG_HIMAX
#ifdef DOG_HIMAX
        // Himax HX7033BTLPA 320x240 LCOS module
        ret = i2c_smbus_write_byte_data(himax_client, 0x00, 0xD6);
        if (ret) {
                printk("\n\nun-successfully wrote display config!\n\n\n");
                dev_err(&himax_client->dev, "write fail: %d\n", ret);
                return ret;
        }
#endif
#ifdef LUMUS_HIMAX
        // Himax HX7027ATGFA 640x480 LCOS module
        ret = i2c_smbus_write_byte_data(himax_client, 0x00, 0x8);
        if (ret) {
                printk("\n\nun-successfully wrote display config!\n\n\n");
                dev_err(&himax_client->dev, "write fail: %d\n", ret);
                return ret;
        }
        // Turn off clk_pos and dither-enable bits:
        ret = i2c_smbus_write_byte_data(himax_client, 0x01, 0x10);
        if (ret) {
                printk("\n\nun-successfully wrote display config!\n\n\n");
                dev_err(&himax_client->dev, "write fail: %d\n", ret);
                return ret;
        }
#endif
#ifdef HIMAX
        // Himax HX7023ATEFA 320x240 LCOS module
        ret = i2c_smbus_write_byte_data(himax_client, 0x00, 0xd3);
        if (ret) {
                printk("\n\nun-successfully wrote display config!\n\n\n");
                dev_err(&himax_client->dev, "write fail: %d\n", ret);
                return ret;
        }
        // Turn off clk_pos:
        ret = i2c_smbus_write_byte_data(himax_client, 0x01, 0x60);
        if (ret) {
                printk("\n\nun-successfully wrote display config!\n\n\n");
                dev_err(&himax_client->dev, "write fail: %d\n", ret);
                return ret;
        }
#endif
#ifdef HDMI
#endif
        return ret;
}

static int __devinit himax_probe(struct i2c_client *client,
                                  const struct i2c_device_id *id)
{
        if (!i2c_check_functionality(client->adapter,
                                     I2C_FUNC_SMBUS_BYTE_DATA)) {
                dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
                return -EIO;
        }

        himax_client = client;
        return display_set_config();
}

static int __devexit himax_remove(struct i2c_client *client)
{
        himax_client = NULL;
        return 0;
}

static const struct i2c_device_id himax_id[] = {
        {"notle_himax", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, notle_id);

static struct i2c_driver notle_driver = {
        .driver = {
                .name = "notle_himax",
        },
        .probe = himax_probe,
        .remove = __devexit_p(himax_remove),
        .id_table = himax_id,
};

int __init notle_dvi_init(void)
{
	int r;

	/* Requesting TFP410 DVI GPIO and disabling it, at bootup */
	r = gpio_request_one(notle_dvi_device.reset_gpio,
				GPIOF_OUT_INIT_LOW, "DVI PD");
	if (r) {
		pr_err("Failed to get DVI powerdown GPIO\n");
                goto err;
        }

        r = gpio_request_one(GPIO_EN_10V, GPIOF_OUT_INIT_HIGH, "enable_10V");
        if (r) {
                pr_err("Failed to get enable_10V gpio\n");
                goto err;
        }

        i2c_add_driver(&notle_driver);
        notle_enable_dvi(&notle_dvi_device);

err:
	return r;
}

static struct omap_dss_device *notle_dss_devices[] = {
	&notle_dvi_device,
};

static struct omap_dss_board_info notle_dss_data = {
	.num_devices	= ARRAY_SIZE(notle_dss_devices),
	.devices	= notle_dss_devices,
	.default_device	= &notle_dvi_device,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	.mode			= MUSB_PERIPHERAL,
#else
	.mode			= MUSB_OTG,
#endif
	.power			= 100,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
        .phy_suspend    = omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_29_30,
                .no_off_init    = true,
	},
#ifdef NOTDEF
        /* XXX turn on DDR mode when we debug the problem */
                                  MMC_CAP_1_8V_DDR | MMC_CAP_BUS_WIDTH_TEST,
#endif
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA |  MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
                .no_off_init    = true,
	},
        {
                .name           = "bcm4329",
                .mmc            = 5,
                .caps           = MMC_CAP_4_BIT_DATA,
                    // TODO(abliss): | MMC_CAP_POWER_OFF_CARD, 
                .gpio_wp        = -EINVAL,
                .gpio_cd        = -EINVAL,
//                .ocr_mask       = MMC_VDD_165_195,
                .ocr_mask       = MMC_VDD_29_30,
                .nonremovable   = true,
        },
	{}	/* Terminator */
};

static struct regulator_consumer_supply notle_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};
static struct regulator_consumer_supply notle_vaux_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "%s: NULL platform data\n", __func__);
		return -EINVAL;
	}
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		 if (ret)
			dev_err(dev, "%s: Error card detect config(%d)\n",
				__func__, ret);
		 else
			pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed omap4_twl6030_hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++) {
		omap4_twl6030_hsmmc_set_late_init(c->dev);
        }

	return 0;
}

// Voltage for eMMC flash
static struct regulator_init_data notle_vaux1 = {
	.constraints = {
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = notle_vaux_supply,
};

// Voltage for sensors (mag, acc, gyro, light sensor, camera)
static struct regulator_init_data notle_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
                .always_on      = true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

// Voltage for display LED?
static struct regulator_init_data notle_vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

/* Voltage for SD card */
static struct regulator_init_data notle_vmmc = {
	.constraints = {
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = notle_vmmc_supply,
};

static struct regulator_init_data notle_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

/* Voltage for the touchpad */
static struct regulator_init_data notle_vusim = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data notle_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data notle_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data notle_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data notle_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data omap4_notle_clk32kg = {
        .constraints = {
               .valid_ops_mask         = REGULATOR_CHANGE_STATUS,
               .always_on              = true,
        },
};

static struct regulator_consumer_supply notle_vmmc5_supply[] = {
        {
                .supply = "vmmc",
                .dev_name = "omap_hsmmc.4",
        },
};

/* Voltage for bcm4329 card */
static struct regulator_init_data notle_vmmc5 = {
        .constraints = {
                .valid_ops_mask         = REGULATOR_CHANGE_STATUS,
                .always_on              = true,
        },
        .num_consumer_supplies  = 1,
        .consumer_supplies      = notle_vmmc5_supply,
};

static struct fixed_voltage_config notle_vwlan = {
        .supply_name = "vbcm4329",
        .microvolts = 3000000, /* 3 v */
        .gpio = GPIO_WL_BT_REG_ON,
        .startup_delay = 70000, /* 70 msec */
        .enable_high = 1,
        .enabled_at_boot = 0,
        .init_data = &notle_vmmc5,
};

static struct platform_device notle_vwlan_device = {
        .name = "reg-fixed-voltage",
        .id = -1,
        .dev = {
                .platform_data = &notle_vwlan,
        },
};

/*
 * Driver data struct for the pwm-backlight driver.  Fields:
 *
 *   max_brightness:  Determines the scale for the 'brightness' values.  A
 *     brightness of max_brightness results in a pwm duty cycle of (almost)
 *     always on.  The almost is because the dmtimer pwm support requires
 *     the duty cycle to be at least 1 clock cycle less than the period.
 *
 *   dft_brightness: Default brightness, this value will be used to
 *     initialize the brightness value.
 *
 *   lth_brightness: Lower threshold brightness, this determines the baseline
 *     for calculating duty cycles for the pwm output.  That is, a value of
 *     1 for brightness will result in a duty cycle of slightly over
 *     lth_brightness / max_brightness, and brightness values scale between
 *     lth_brightness and max_brightness linearly.
 *
 *   uth_brightness: Upper threshold brightness, this determines the maximum
 *     duty cycle of the pwm backlight.  That is, at a brightness of
 *     max_brightness, the duty cycle of the pwm will be
 *     uth_brightness / max_brightness.  If a value of 0 is given, the
 *     pwm will max at a 100% duty cycle (full on).
 *
 *   pwm_period_ns: Period of the pwm signal in nanoseconds.  This value
 *     (1009082) corresponds to approximately 991 Hz, where we seem to get
 *     minimal visible flickering of the backlight.
 */
static struct platform_pwm_backlight_data backlight_data = {
        .max_brightness = 0xff,
        .dft_brightness = 0x00,
        .lth_brightness = 0x00,
        .uth_brightness = 0x04,
        .pwm_period_ns = 1009082,
};

static struct platform_device backlight_device = {
        .name = "pwm-backlight",
        .id = -1,
        .dev = {
                .platform_data = &backlight_data,
        },
};

static struct platform_device *notle_devices[] __initdata = {
        &leds_gpio,
        &notle_vwlan_device,
};

static struct twl4030_bci_platform_data notle_bci_data = {
        .monitoring_interval            = 10,
        .max_charger_currentmA          = 1500,
        .max_charger_voltagemV          = 4560,
        .max_bat_voltagemV              = 4200,
        .low_bat_voltagemV              = 3300,
};

// Copied from board-4430sdp.c, but it doesn't seem to be used.
static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}

static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
/*
	.max_timeout	= 15000,
	.initial_vibrate = 0,
*/
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_madc_platform_data notle_gpadc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data notle_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &notle_vmmc,
	.vpp		= &notle_vpp,
	.vusim		= &notle_vusim,
	.vana		= &notle_vana,
	.vcxio		= &notle_vcxio,
	.vdac		= &notle_vdac,
	.vusb		= &notle_vusb,
	.vaux1		= &notle_vaux1,
	.vaux2		= &notle_vaux2,
	.vaux3		= &notle_vaux3,
        .clk32kg        = &omap4_notle_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* children */
        .codec          = &twl6040_codec,
	.bci            = &notle_bci_data,
	.madc           = &notle_gpadc_data,
};

static struct i2c_board_info __initdata notle_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_twldata,
	},
};

static struct rmi_i2c_platformdata synaptics_platformdata = {
       // same address here as in I2C_BOARD_INFO
       .i2c_address = 0x20,
       .irq = 0x00,
};

static struct i2c_board_info __initdata notle_i2c_3_boardinfo[] = {
        {
                I2C_BOARD_INFO("rmi4_ts", 0x20),
                .platform_data = &synaptics_platformdata,
        },
        {
                I2C_BOARD_INFO("stmpe32m28", 0x48),
        },
};

/*
 * i2c-4 
 */
static struct l3g4200d_gyr_platform_data notle_l3g4200d_data = {
        .min_interval = 1,                // Minimum poll interval in ms.
	.poll_interval = 10,              /* poll interval (in ms) to pass
                                             to kernel input_polled_dev. An
                                             appropriate sampling rate to
                                             set the hardware to will be
                                             chosen by the driver. */

        .fs_range = L3G4200D_GYR_FS_2000DPS,   /* full-scale range to set the
                                                 hardware to.  valid values are
                                                 250, 500, and 2000 (but you
                                                 must use the respective
                                                 constant here). */

      /* axis mapping, reorder these to change the order of the axes. */
	.axis_map_x = 1,
	.negate_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
};

static struct lsm303dlhc_acc_platform_data notle_lsm303dlh_acc_data = {
        .min_interval = 1,     // Minimum poll interval in ms.
        .poll_interval = 10,   // Poll interval in ms.

      /* axis mapping, reorder these to change the order of the axes. */
	.axis_map_x = 0,
	.negate_x = 1,
	.axis_map_y = 1,
	.negate_y = 1,
	.axis_map_z = 2,
        /* These need to be set or initialized to <0 or the driver croaks. */
        .gpio_int1 = -1,
        .gpio_int2 = -1,
	/* +-2G sensitivity.  Matches expectations of Android driver. */
	.g_range = LSM303DLHC_ACC_G_2G,
};

static struct lsm303dlhc_mag_platform_data notle_lsm303dlh_mag_data = {
        .h_range = LSM303DLHC_H_1_3G,

        .min_interval = 1,     // Minimum poll interval in ms.
        .poll_interval = 10,   // Poll interval in ms.

      /* axis mapping, reorder these to change the order of the axes. */
	.axis_map_x = 0,
	.negate_x = 1,
	.axis_map_y = 2,
	.negate_y = 1,
	.axis_map_z = 1,
};

static struct ltr506_als_platform_data notle_ltr506_als_data = {
};

static struct i2c_board_info __initdata notle_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO("l3g4200d_gyr", 0x68),
		.flags = I2C_CLIENT_WAKE,
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_l3g4200d_data,
	},
	{
		I2C_BOARD_INFO("lsm303dlhc_acc", 0x18),
		.flags = I2C_CLIENT_WAKE,
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_lsm303dlh_acc_data,
	},
	{
		I2C_BOARD_INFO("ltr506_als", 0x1d),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &notle_ltr506_als_data,
	},
	{
		I2C_BOARD_INFO("lsm303dlhc_mag", 0x1e),
		.flags = I2C_CLIENT_WAKE,
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_lsm303dlh_mag_data,
	},
	{
		I2C_BOARD_INFO("notle_himax", 0x48),
	},
};


static int __init notle_i2c_init(void)
{
	omap4_pmic_init("twl6030", &notle_twldata);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
			ARRAY_SIZE(notle_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, notle_i2c_4_boardinfo,
			ARRAY_SIZE(notle_i2c_4_boardinfo));
	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux empty_board_mux[] __initdata = {
        // XXX This omap4_mux_init() function seems ill-conceived.  It
        // writes this index to different mux settings by using
        // different bases.
        // aliased to gpio_wk2:
	// OMAP4_MUX(SIM_RESET, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),

        /* LCD_RESET_N - GPIO 53 */
        // OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
        // OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define emtpy_board_mux	NULL
#endif

static int omap_audio_init(void) {
	int r;

	/* Configuration of requested GPIO lines */

        r = gpio_request_one(GPIO_AUDIO_POWERON, GPIOF_OUT_INIT_HIGH,
                "audio_poweron");
        if (r) {
                pr_err("Failed to get audio_poweron gpio\n");
                goto error;
        }

        return 0;

error:
        return r;
}

static int omap_gps_init(void) {
	int r;

	/* Configuration of requested GPIO lines */

        r = gpio_request_one(GPIO_GPS_RESET_N, GPIOF_OUT_INIT_HIGH,
                "gps_reset_n");
        if (r) {
                pr_err("Failed to get gps_reset_n gpio\n");
                goto error;
        }

        /* The part needs to see a rising edge. */
        r = gpio_request_one(GPIO_GPS_ON_OFF, GPIOF_OUT_INIT_LOW,
                "gps_on_off");
        if (r) {
                pr_err("Failed to get gps_on_off gpio\n");
                goto error;
        }
	gpio_set_value(GPIO_GPS_ON_OFF, 1);

        return 0;

error:
        return r;
}

static int __init notle_wifi_init(void) {
        int r;

        if (!machine_is_notle())
                return 0;

        pr_info("%s()+", __func__);

        /* Configuration of requested GPIO lines */

        r = gpio_request_one(GPIO_BCM_WLAN_HOST_WAKE, GPIOF_IN, "wlan_host_wake");
        if (r) {
                pr_err("Failed to get wlan_host_wake gpio\n");
                goto error;
        }

        r = gpio_request_one(GPIO_BCM_WLAN_WAKE, GPIOF_IN, "wlan_wake");
        if (r) {
                pr_err("Failed to get wlan_wake gpio\n");
                goto error1;
        }

        r = gpio_request_one(GPIO_BCM_BT_HOST_WAKE, GPIOF_IN, "bt_host_wake");
        if (r) {
                pr_err("Failed to get bt_host_wake gpio\n");
                goto error2;
        }

        r = gpio_request_one(GPIO_BCM_BT_WAKE, GPIOF_OUT_INIT_HIGH, "bt_wake");
        gpio_set_value(GPIO_BCM_BT_WAKE, 1);
        if (r) {
                pr_err("Failed to get bt_wake gpio\n");
                goto error3;
        }

        r = gpio_request_one(GPIO_WL_RST_N, GPIOF_OUT_INIT_LOW, "wlan_reset");
        if (r) {
                pr_err("Failed to get wlan_reset gpio\n");
                goto error4;
        }

        r = gpio_request_one(GPIO_BT_RST_N, GPIOF_OUT_INIT_LOW, "bt_reset");
        if (r) {
                pr_err("Failed to get bt_reset gpio\n");
                goto error5;
        }

        gpio_set_value(GPIO_WL_RST_N, 1);
        gpio_set_value(GPIO_BT_RST_N, 1);

        r = gpio_to_irq(GPIO_BCM_WLAN_HOST_WAKE);
        if (r < 0) {
                pr_err("Failed to allocate irq for gpio %i\n", GPIO_BCM_WLAN_HOST_WAKE);
                goto error6;
        }

        notle_wifi_resources.start = notle_wifi_resources.end = r;

        r = gpio_to_irq(GPIO_BCM_BT_HOST_WAKE);
        if (r < 0) {
                pr_err("Failed to allocate irq for gpio %i\n", GPIO_BCM_BT_HOST_WAKE);
                goto error6;
        }
        // TODO(abliss): wire up BT host wake IRQ for power management
        r = platform_device_register(&notle_wifi_device);
        if (r) {
                pr_err("Failed to register platform device\n");
                goto error6;
        }

        pr_info("%s()-: 0", __func__);
        return 0;

error6:
        gpio_free(GPIO_BT_RST_N);
error5:
        gpio_free(GPIO_WL_RST_N);
error4:
        gpio_free(GPIO_BCM_BT_WAKE);
error3:
        gpio_free(GPIO_BCM_BT_HOST_WAKE);
error2:
        gpio_free(GPIO_BCM_WLAN_WAKE);
error1:
        gpio_free(GPIO_BCM_WLAN_HOST_WAKE);
error:
        pr_info("%s()-: %i", __func__, r);
        return r;
}

static void notle_dmtimer_pwm_enable(void) {
        unsigned int core_base_addr = 0xfc100000;

        // pwm timer output:
        __raw_writew(OMAP_MUX_MODE1, core_base_addr + MUX_BACKLIGHT);
}

static void notle_dmtimer_pwm_disable(void) {
        unsigned int core_base_addr = 0xfc100000;

        // pwm timer output:
        __raw_writew(OMAP_MUX_MODE7 | OMAP_PULL_ENA, core_base_addr + MUX_BACKLIGHT);
}

static struct dmtimer_pwm_ops notle_dmtimer_pwm_ops = {
  .enable       = notle_dmtimer_pwm_enable,
  .disable      = notle_dmtimer_pwm_disable,
};

static void __init notle_pwm_backlight_init(void) {
        int r;
        struct pwm_device *pwm;
        pwm = pwm_request_dmtimer(PWM_TIMER, "backlight",
                                  &notle_dmtimer_pwm_ops);

        if (!pwm) {
                pr_err("Failed to request backlight dmtimer\n");
                return;
        }

        backlight_data.pwm_id = pwm->pwm_id;
        r = platform_device_register(&backlight_device);
        if (r) {
                pr_err("Failed to register backlight platform device\n");
                pwm_free(pwm);
                return;
        }

        pr_info("Successfully initialized backlight\n");
        return;
}

static void __init my_mux_init(void) {
        // Move this code to board_mux constants when we're convinced it works:

        // Example code for writing mux values, bypassing omap4_mux_init code:
        unsigned int core_base_addr = 0xfc100000;
        unsigned int wkup_base_addr = 0xfc31e000;

        // gpio's in the first bank of 32 use the wkup base:
        // output gpio's:
        __raw_writew(OMAP_MUX_MODE3, wkup_base_addr + MUX_GREEN_LED);
        __raw_writew(OMAP_MUX_MODE3, wkup_base_addr + MUX_YELLOW_LED);

        // Others use the core base:
        // output gpio's:
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_GPS_ON_OFF);
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_GPS_RESET_N);
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_LCD_RESET_N);
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_AUDIO_POWERON);
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_EN_10V);
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_BCM_BT_WAKE);
        __raw_writew(OMAP_MUX_MODE3, core_base_addr + MUX_BT_RST_N);

        // Set display backlight to be pulled low when we start.
        __raw_writew(OMAP_MUX_MODE7 | OMAP_PULL_ENA, core_base_addr + MUX_BACKLIGHT);

        // input gpio's:
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                core_base_addr + MUX_BCM_WLAN_HOST_WAKE);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                core_base_addr + MUX_BCM_BT_HOST_WAKE);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                core_base_addr + MUX_BCM_WLAN_WAKE);
}

static void __init notle_init(void)
{
        int package = OMAP_PACKAGE_CBS;
        int err;

        if (omap_rev() == OMAP4430_REV_ES1_0)
                package = OMAP_PACKAGE_CBL;
        omap4_mux_init(empty_board_mux, empty_board_mux, package);
        my_mux_init();

        notle_i2c_init();
        notle_pwm_backlight_init();

        platform_add_devices(notle_devices, ARRAY_SIZE(notle_devices));
        omap_serial_init();
        omap4_twl6030_hsmmc_init(mmc);
        usb_musb_init(&musb_board_data);

        err = omap_audio_init();
        if (err) {
                pr_err("Audio initialization failed: %d\n", err);
        }

        err = omap_gps_init();
        if (err) {
                pr_err("GPS initialization failed: %d\n", err);
        }

        err = notle_wifi_init();
        if (err) {
                pr_err("Wifi initialization failed: %d\n", err);
        }

        err = notle_dvi_init();
        if (!err) {
                omap_display_init(&notle_dss_data);
        } else {
                pr_err("DVI initialization failed: %d\n", err);
        }
}

static void __init notle_map_io(void)
{
        omap2_set_globals_443x();
        omap44xx_map_common_io();
}

MACHINE_START(NOTLE, "OMAP4430")
	/* Maintainer: David Anders - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= notle_map_io,
	.init_early	= notle_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= notle_init,
	.timer		= &omap_timer,
MACHINE_END
