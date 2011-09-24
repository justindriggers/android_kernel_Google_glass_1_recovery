/*
 * Board support file for OMAP4430 based NotleBoard.
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

#include "board-notle.h"

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/memblock.h>
#include <linux/omapfb.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/reboot.h>
#include <linux/rfkill-gpio.h>

#ifdef CONFIG_INPUT_L3G4200D
#include <linux/i2c/l3g4200d.h>
#endif
#ifdef CONFIG_INPUT_LSM303DLHC
#include <linux/i2c/lsm303dlhc.h>
#endif
#ifdef CONFIG_INPUT_LTR506ALS
#include <linux/i2c/ltr506als.h>
#endif

#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#include <linux/mpu.h>
#endif

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#include "../../../drivers/input/touchscreen/rmi_i2c.h"
#endif

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/dmm.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/dmtimer.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <plat/vram.h>
#include <plat/omap-serial.h>
#include <plat/dmtimer-pwm.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <video/omap-panel-tc358762.h>

#include "timer-gp.h"
#include "omap4_ion.h"
#include "hsmmc.h"
#include "control.h"
#include "pm.h"
#include "common-board-devices.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "cm1_44xx.h"

#define GPIO_GREEN_LED                  7
#define MUX_GREEN_LED                   MUX(FREF_CLK4_REQ)
#define GPIO_YELLOW_LED                 8
#define MUX_YELLOW_LED                  MUX(FREF_CLK4_OUT)
#define GPIO_AUDIO_HEADSET              44
#define MUX_AUDIO_HEADSET               MUX(GPMC_A20)
#define GPIO_GPS_ON_OFF                 49
#define MUX_GPS_ON_OFF                  MUX(GPMC_A25)
#define GPIO_GPS_RESET_N                52
#define MUX_GPS_RESET_N                 MUX(GPMC_NCS2)
#define GPIO_LCD_RESET_N                53
#define MUX_LCD_RESET_N                 MUX(GPMC_NCS3)
#define GPIO_AUDIO_POWERON_DOG          62
#define MUX_AUDIO_POWERON_DOG           MUX(GPMC_WAIT1)
#define GPIO_AUDIO_POWERON_EMU          127
#define MUX_AUDIO_POWERON_EMU           MUX(HDQ_SIO)
#define GPIO_EN_10V                     84
#define MUX_EN_10V                      MUX(USBB1_ULPITLL_CLK)
#define GPIO_CAMERA_DOG                 94
#define MUX_CAMERA_DOG                  MUX(USBB1_ULPITLL_DAT6)
#define GPIO_CAMERA_EMU                 121
#define MUX_CAMERA_EMU                  MUX(ABE_DMIC_DIN2)
#define GPIO_TOUCHPAD_INT_N             32
#define MUX_TOUCHPAD_INT_N              MUX(GPMC_AD8)
#define MUX_BACKLIGHT                   MUX(USBB1_ULPITLL_DAT5)
#define PWM_TIMER                       9
// Wifi defines go in board-notle.h.
/*
#define GPIO_HUB_POWER		                1
#define NOTLE_DVI_TFP410_POWER_DOWN_GPIO        53
#define GPIO_HUB_NRESET		                62
#define NOTLE_EN_10V                            84
#define GPIO_BCM_BT_WAKE                        91
#define GPIO_BCM_BT_HOST_WAKE                   87
#define GPIO_BCM_WLAN_WAKE                      88
#define GPIO_BCM_WLAN_HOST_WAKE                 86
*/

#define PHYS_ADDR_SMC_SIZE      (SZ_1M * 3)
#define PHYS_ADDR_SMC_MEM       (0x80000000 + SZ_1G - PHYS_ADDR_SMC_SIZE)
#define OMAP_ION_HEAP_SECURE_INPUT_SIZE (SZ_1M * 30)
// NOTE(abliss): This should move to 105 to keep up with panda, but that seems
// to break currently.  When it does, we probably also need to change the value
// in omap4_ion.c.
#define PHYS_ADDR_DUCATI_SIZE   (SZ_1M * 103)
#define PHYS_ADDR_DUCATI_MEM    (PHYS_ADDR_SMC_MEM - PHYS_ADDR_DUCATI_SIZE - \
                                 OMAP_ION_HEAP_SECURE_INPUT_SIZE)

#ifndef CONFIG_NOTLE_I2C4_SENSORS
#define CONFIG_NOTLE_I2C4_SENSORS 0
#endif

extern int tuna_wlan_init(void);

// Notle board version detection gpios
#define GPIO_ID2                        42
#define MUX_ID2                         MUX(GPMC_A18)
#define GPIO_ID1                        40
#define MUX_ID1                         MUX(GPMC_A16)
#define GPIO_ID0                        34
#define MUX_ID0                         MUX(GPMC_AD10)

typedef enum {
        UNVERSIONED = 7,
        V1_DOG      = 7,
        V3_EMU      = 0,
        V4_FLY      = 1,
        V5_GNU      = 2
} notle_version;

/* Read board version from GPIO */
static void notle_version_init(void)
{
        int r;

        r = gpio_request_one(GPIO_ID2, GPIOF_IN, "id2");
        if (r) {
                pr_err("Failed to get gpio %d for id2 pin of board version\n", GPIO_ID2);
        }

        r = gpio_request_one(GPIO_ID1, GPIOF_IN, "id1");
        if (r) {
                pr_err("Failed to get gpio %d for id1 pin of board version\n", GPIO_ID1);
        }

        r = gpio_request_one(GPIO_ID0, GPIOF_IN, "id0");
        if (r) {
                pr_err("Failed to get gpio %d for id0 pin of board version\n", GPIO_ID0);
        }
}

static notle_version notle_version_read(void)
{
        unsigned int bVer = UNVERSIONED;

        bVer = gpio_get_value(GPIO_ID0) | (gpio_get_value(GPIO_ID1) << 1)
                       | (gpio_get_value(GPIO_ID2) << 2);

        return (notle_version) bVer;
}

static char * notle_version_str(notle_version board_ver)
{
        switch (board_ver)
        {
        case V1_DOG:
                return "V1 DOG or earlier";
        case V3_EMU:
                return "V3 EMU";
        case V4_FLY:
                return "V4 FLY";
        case V5_GNU:
                return "V5 GNU";
        }
        return "UNVERSIONED";
}

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

static void __init notle_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

static int display_set_config(void);

#ifndef CONFIG_OMAP2_DSS_DSI
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
#else

static struct tc358762_board_data dsi_panel = {
        .reset_gpio     = 53,
};

static struct omap_dss_device notle_dsi_device = {
        .name                   = "lcd",
        .driver_name            = "tc358762",
        .type                   = OMAP_DISPLAY_TYPE_DSI,
        .data                   = &dsi_panel,
        .phy.dsi                = {
                .type           = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
                .clk_lane       = 1,
                .clk_pol        = 0,
                .data1_lane     = 2,
                .data1_pol      = 0,
        },
        .clocks                 = {
                .dispc                  = {
                        .channel                = {
                                .lck_div                = 1,        /* Logic Clock = 172.8 MHz */
                                .pck_div                = 6,        /* Pixel Clock = 34.56 MHz */
                                .lcd_clk_src            = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
                        },
                        .dispc_fclk_src        = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
                },
                .dsi                    = {
                        .regm                   = 192,  /* DSI_PLL_REGM */
                        .regn                   = 8,   /* DSI_PLL_REGN */
                        .regm_dispc             = 6,    /* PLL_CLK1 (M4) */
                        .regm_dsi               = 5,    /* PLL_CLK2 (M5) */

                        .lp_clk_div             = 13,    /* LP Clock = 8.64 MHz */
                        .dsi_fclk_src           = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
                },
        },
        .panel                  = {
        },
        .channel                = OMAP_DSS_CHANNEL_LCD,
};
#endif

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

#ifdef CONFIG_OMAP2_DSS_DSI
int __init notle_dsi_init(void) {
        u32 reg;
        int r;

        /* Enable 2 lanes in DSI1 module, disable pull down */
        reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
        reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
        reg |= 0x3 << OMAP4_DSI1_LANEENABLE_SHIFT;
        reg &= ~OMAP4_DSI1_PIPD_MASK;
        reg &= ~OMAP4_DSI2_PIPD_MASK;
        reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
        reg |= 0x1f << OMAP4_DSI2_PIPD_SHIFT;
        pr_info("Writing 0x%08x to CONTROL_DSIPHY\n", reg);
        omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);


        /* Enable DISPLAY_ENB gpio */
        r = gpio_request_one(GPIO_EN_10V, GPIOF_OUT_INIT_HIGH, "enable_10V");
        if (r) {
                pr_err("Failed to get enable_10V gpio\n");
                goto err;
        }

        r = gpio_request_one(dsi_panel.reset_gpio, GPIOF_OUT_INIT_LOW, "display_reset");
        if (r) {
                pr_err("Failed to get display_reset gpio\n");
                goto err1;
        }

        return 0;
err1:
        gpio_free(GPIO_EN_10V);
err:
        return r;
}

#else
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
#endif

static struct omap_dss_device *notle_dss_devices[] = {
#ifndef CONFIG_OMAP2_DSS_DSI
        &notle_dvi_device,
#else
        &notle_dsi_device,
#endif
};

static struct omap_dss_board_info notle_dss_data = {
	.num_devices	= ARRAY_SIZE(notle_dss_devices),
	.devices	= notle_dss_devices,
#ifdef CONFIG_OMAP2_DSS_DSI
        .default_device = &notle_dsi_device,
#else
	.default_device	= &notle_dvi_device,
#endif
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
                .power_saving   = true,
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
                .power_saving   = true,
	},
        {
                .name           = "bcm4329",
                .mmc            = 5,
                .caps           = MMC_CAP_4_BIT_DATA,
                    // TODO(abliss): | MMC_CAP_POWER_OFF_CARD, 
                .gpio_wp        = -EINVAL,
                .gpio_cd        = -EINVAL,
                .ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
                .nonremovable   = true,
		.mmc_data	= &tuna_wifi_data,
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


static struct regulator_consumer_supply notle_cam2_supply[] = {
  {
    .supply = "cam2pwr",
  },
};

// Voltage for sensors (mag, acc, gyro, light sensor, camera)
static struct regulator_init_data notle_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		.always_on		= true,
	},
        .num_consumer_supplies = 1,
        .consumer_supplies = notle_cam2_supply,
};

static struct regulator_consumer_supply notle_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

// Voltage for display LED?
static struct regulator_init_data notle_vaux3 = {
	.constraints = {
#ifdef CONFIG_OMAP2_DSS_DSI
		.min_uV			= 1200000,
		.max_uV			= 1200000,
#else
		.min_uV			= 2800000,
		.max_uV			= 2800000,
#endif
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(notle_vcxio_supply),
	.consumer_supplies	= notle_vcxio_supply,
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
                /* Fixed voltage regulators do not have a set_voltage() hook
                 * therefore cannot have the voltage set. */
		.apply_uV		= false,
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
                /* Fixed voltage regulators do not have a set_voltage() hook
                 * therefore cannot have the voltage set. */
		.apply_uV		= false,
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
                /* Fixed voltage regulators do not have a set_voltage() hook
                 * therefore cannot have the voltage set. */
		.apply_uV		= false,
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

static struct omap_uart_port_info omap_serial_port_info[] = {
        { /* ttyO0 unused */
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        },
        { /* ttyO1 bluetooth */
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        },
        { /* ttyO2 console port */
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        },
        {  /* ttyO3 GPS */
                .use_dma        = 1,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                /* TODO(cmanton) Use interrupts for better (?) power management */
                .dma_rx_poll_rate = 100000,  // units in usec
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
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

/*
 * Driver data struct for the rfkill-gpio driver.  Fields:
 * @name:		name for the gpio rf kill instance
 * @reset_gpio:		GPIO which is used for reseting rfkill switch
 * @shutdown_gpio:	GPIO which is used for shutdown of rfkill switch
 * @power_clk_name:	[optional] name of clk to turn off while blocked
 */

static struct rfkill_gpio_platform_data bluetooth_rfkill_data = {
        .name = "bluetooth",
        .reset_gpio = GPIO_BT_RST_N,
        .shutdown_gpio = -1,
        .type = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bluetooth_rfkill_device = {
        .name = "rfkill_gpio",
        .id = -1,
        .dev = {
                .platform_data = &bluetooth_rfkill_data,
        },
};

// Translate hardware buttons to keys -- we have only one.  Note that
// GPIO_CAMERA_DOG may be overwritten by GPIO_CAMERA_EMU below.
static struct gpio_keys_button notle_button_table[] = {
    [0] = {
                .code   = KEY_CAMERA,           \
                .gpio   = GPIO_CAMERA_DOG,      \
                .desc   = "Camera",             \
                .type   = EV_KEY,               \
                .wakeup = 1,                    \
    },
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = notle_button_table,
	.nbuttons = ARRAY_SIZE(notle_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

static struct platform_device *notle_devices[] __initdata = {
        &leds_gpio,
        &gpio_keys,
};


static struct twl4030_bci_platform_data notle_bci_data = {
        .monitoring_interval            = 10,
        .max_charger_currentmA          = 1500,
        .max_charger_voltagemV          = 4560,
        .max_bat_voltagemV              = 4200,
        .low_bat_voltagemV              = 3300,
};


static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
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

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
static struct rmi_sensor_suspend_custom_ops synaptics_custom_ops = {
        .rmi_sensor_custom_suspend = 0,
        .rmi_sensor_custom_resume = 0,
        .delay_resume = 0,
};

static struct rmi_sensordata __initdata synaptics_sensordata = {
        .rmi_sensor_setup = 0,
        .rmi_sensor_teardown = 0,
        .attn_gpio_number = GPIO_TOUCHPAD_INT_N,
        .attn_polarity = 0,
        .custom_suspend_ops = &synaptics_custom_ops,
        .perfunctiondata = 0,
};

static struct rmi_i2c_platformdata __initdata synaptics_platformdata = {
       // same address here as in I2C_BOARD_INFO
       .i2c_address = 0x20,
//       .irq = 0x00,
        .sensordata = &synaptics_sensordata,
};
#endif

static struct i2c_board_info __initdata notle_i2c_3_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
        {
                I2C_BOARD_INFO("rmi4_ts", 0x20),
                .platform_data = &synaptics_platformdata,
        },
#endif
        {
                I2C_BOARD_INFO("stmpe32m28", 0x48),
        },
};

#if CONFIG_NOTLE_I2C4_SENSORS
/*
 * i2c-4 
 */
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
static struct mpu_platform_data mpu6050_data = {
        .int_config     = 0x10,
        .orientation    = { 1, 0, 0,
                            0, 1, 0,
                            0, 0, 1},
        .level_shifter  = 0,
        .accel          = {
                .get_slave_descr = mantis_get_slave_descr,
                .adapt_num = 4,
                .bus = EXT_SLAVE_BUS_SECONDARY,
                .address = 0x68,
                .orientation = { 1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1 },
        },
        .compass        = {
                .get_slave_descr = ak8975_get_slave_descr,
                .adapt_num = 4,
                .bus = EXT_SLAVE_BUS_SECONDARY,
                .address = 0x0C,
                .orientation = { 0, 1, 0,
                                 1, 0, 0,
                                 0, 0, -1 },
        },
};
#endif

#ifdef CONFIG_INPUT_L3G4200D
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
	.axis_map_x = 0,
	.negate_x = 1,
	.axis_map_y = 1,
	.negate_y = 1,
	.axis_map_z = 2,
};
#endif

#ifdef CONFIG_INPUT_LSM303DLHC
static struct lsm303dlhc_acc_platform_data notle_lsm303dlh_acc_data = {
        .min_interval = 1,     // Minimum poll interval in ms.
        .poll_interval = 10,   // Poll interval in ms.

      /* axis mapping, reorder these to change the order of the axes. */
	.axis_map_x = 1,
	.axis_map_y = 0,
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
	.axis_map_x = 2,
	.axis_map_y = 0,
	.negate_y = 1,
	.axis_map_z = 1,
};
#endif

#ifdef CONFIG_INPUT_LTR506ALS
static struct ltr506_platform_data notle_ltr506als_data = {
        .pfd_levels = { 0,0,0,0,0 },
        .pfd_als_lowthresh = 0,
        .pfd_als_highthresh = 0,

        /* PS */
        .pfd_ps_lowthresh = 0,
        .pfd_ps_highthresh = 0,

        /* Interrupt */
        .pfd_gpio_int_no = 0,
};
#endif

#endif //CONFIG_NOTLE_I2C4_SENSORS

static struct i2c_board_info __initdata notle_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO("tc358762-i2c", 0x0b),
	},
    // NOTE(abliss): currently, this i2c bus can support EITHER the sensors OR
    // the camera
#if CONFIG_NOTLE_I2C4_SENSORS
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
        {
		I2C_BOARD_INFO("mpu6050B1", 0x68),
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &mpu6050_data,
        },
#endif
#ifdef CONFIG_INPUT_L3G4200D
	{
		I2C_BOARD_INFO("l3g4200d_gyr", 0x68),
		.flags = I2C_CLIENT_WAKE,
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_l3g4200d_data,
	},
#endif
#ifdef CONFIG_INPUT_LSM303DLHC
	{
		I2C_BOARD_INFO("lsm303dlhc_acc", 0x18),
		.flags = I2C_CLIENT_WAKE,
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_lsm303dlh_acc_data,
	},
	{
		I2C_BOARD_INFO("lsm303dlhc_mag", 0x1e),
		.flags = I2C_CLIENT_WAKE,
	//	.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &notle_lsm303dlh_mag_data,
	},
#endif
#ifdef CONFIG_INPUT_LTR506ALS
	{
		I2C_BOARD_INFO("ltr506als", 0x1d),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &notle_ltr506als_data,
	},
#endif
	{
		I2C_BOARD_INFO("notle_himax", 0x48),
	},
#endif //CONFIG_NOTLE_I2C4_SENSORS
	{
		I2C_BOARD_INFO("ov9726", 0x10),
		.flags = I2C_CLIENT_WAKE,
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
        u32 omap4430_cm_clksel_dpll_abe_register;
        notle_version board_ver;
        int audio_power_on_gpio = GPIO_AUDIO_POWERON_EMU;

        /* Set the correct audio power on GPIO based on board revision */
        board_ver = notle_version_read();

        /* TODO(petermalkin) Whenever we get rid of i2c errors, remove the manual GPIO settings */
        /*      and put back the setting for the twl6030 driver to set poweron pin for audio    */
        /*      like this: audio_power_on_gpio = AUDIO_GPIO_62;                            */
        if (board_ver == V1_DOG) {
                __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_AUDIO_POWERON_DOG);
                r = gpio_request_one(GPIO_AUDIO_POWERON_DOG, GPIOF_OUT_INIT_HIGH, "audio_poweron");
        }
        twl6040_codec.audpwron_gpio = audio_power_on_gpio;

        /* GPIO that enables a MUX outside omap to use usb headset */
        r = gpio_request_one(GPIO_AUDIO_HEADSET, GPIOF_OUT_INIT_LOW,
                "gpio_audio_headset");
        if (r) {
                pr_err("Failed to get audio_headset gpio_%d\n", GPIO_AUDIO_HEADSET);
                goto error;
        }

        /* TODO(petermalkin): remove this line for the product compile. */
        /* Do not expose GPIO to /sys filesystem for security purposes. */
        r = gpio_export(GPIO_AUDIO_HEADSET, false);
        if (r) {
                pr_err("Unable to export audio_headset gpio_%d\n", GPIO_AUDIO_HEADSET);
        }

        /* Set ABE DPLL to a correct value so that Notle clock */
        /* does not mess up Phoenix audio */
        omap4430_cm_clksel_dpll_abe_register = OMAP4430_CM1_BASE + OMAP4430_CM1_CKGEN_INST + OMAP4_CM_CLKSEL_DPLL_ABE_OFFSET;
        omap_writel(0x82ee00, omap4430_cm_clksel_dpll_abe_register);

        omap_mux_init_signal("sys_nirq2.sys_nirq2", \
                OMAP_PIN_INPUT_PULLUP);
        return 0;

error:
        return r;
}

static int notle_gps_init(void) {
	int r;

	/* Configuration of requested GPIO lines */

        r = gpio_request_one(GPIO_GPS_RESET_N, GPIOF_OUT_INIT_HIGH,
                "gps_reset_n");
        if (r) {
                pr_err("Failed to get gps_reset_n gpio\n");
                goto error;
        }

        r = gpio_export(GPIO_GPS_RESET_N, false);
        if (r) {
                pr_err("Unable to export gps_reset_n gpio\n");
        }

        r = gpio_sysfs_set_active_low(GPIO_GPS_RESET_N, 0);
        if (r) {
                pr_err("Unable to set sysfs gps_reset_n active low\n");
        }

        /* Need a rising edge to turn device on. */
        r = gpio_request_one(GPIO_GPS_ON_OFF, GPIOF_OUT_INIT_LOW,
                "gps_on_off");
        if (r) {
                pr_err("Failed to get gps_on_off gpio\n");
                goto error;
        }

        r = gpio_export(GPIO_GPS_ON_OFF, false);
        if (r) {
                pr_err("Unable to export gps_on_off gpio\n");
        }

        return 0;

error:
        return r;
}

static int __init notle_gps_start(void) {
	gpio_set_value(GPIO_GPS_ON_OFF, 1);
        pr_info("Turning on GPS chip\n");
        return 0;
}
/* XXX Turning on GPS currently costs us ~50mA of current draw.
late_initcall(notle_gps_start);
*/

static int __init notle_touchpad_init(void) {
        int r;

        pr_info("%s()+", __func__);

        /* Configuration of requested GPIO line */

        r = gpio_request_one(GPIO_TOUCHPAD_INT_N, GPIOF_IN, "touchpad_int_n");
        if (r) {
                pr_err("Failed to get touchpad_int_n gpio\n");
        }
        return r;
}


#ifndef BACKLIGHT_HACK
static void notle_dmtimer_pwm_enable(void) {
        // pwm timer output:
        __raw_writew(OMAP_MUX_MODE1, CORE_BASE_ADDR + MUX_BACKLIGHT);
}

static void notle_dmtimer_pwm_disable(void) {
        // pwm timer output:
        __raw_writew(OMAP_MUX_MODE7 | OMAP_PULL_ENA, CORE_BASE_ADDR + MUX_BACKLIGHT);
}

static struct dmtimer_pwm_ops notle_dmtimer_pwm_ops = {
  .enable       = notle_dmtimer_pwm_enable,
  .disable      = notle_dmtimer_pwm_disable,
};

static int __init notle_pwm_backlight_init(void) {
        int r;
        struct pwm_device *pwm;
        pwm = pwm_request_dmtimer(PWM_TIMER, "backlight",
                                  &notle_dmtimer_pwm_ops);

        if (!pwm) {
                pr_err("Failed to request backlight dmtimer\n");
                return -1;
        }

        backlight_data.pwm_id = pwm->pwm_id;
        if (notle_version_read() == V3_EMU) {
          // Emu has a dimmer display; increase the max brightness.
          backlight_data.uth_brightness = 0x0F;
        }
        r = platform_device_register(&backlight_device);
        if (r) {
                pr_err("Failed to register backlight platform device\n");
                pwm_free(pwm);
                return -1;
        }

        pr_info("Successfully initialized backlight\n");
        return 0;
}

/*
   This code uses omap4 timers that are initialized as devices, so it must
   be called after device initialization is finished.
*/
late_initcall(notle_pwm_backlight_init);
#else
static int notle_backlight_hack(void) {
        // *** Adjust this value to set the backlight brightness. ***
        float brightness = .01;
        struct omap_dm_timer* notle_pwm_timer;
        const unsigned int sys_clock_hz = 19200000;
        // This wants to be high enough we don't see flicker,
        // but if it's too high, the display just blanks when
        // set to a 1% duty cycle.
        const unsigned int pwm_rate_hz = 991;
        unsigned int denom = sys_clock_hz / pwm_rate_hz;
        unsigned int num = denom * brightness;
        const unsigned int max = 0xffffffff;

        if (num > denom) num = denom;
        if (num == denom) {
                num = 0xfffffffe;
                denom = 0xffffffff;
        }

        notle_pwm_timer = omap_dm_timer_request_specific(PWM_TIMER);
        if (!notle_pwm_timer) {
                return -1;
        }
        omap_dm_timer_set_source(notle_pwm_timer, OMAP_TIMER_SRC_SYS_CLK);
        // pwm mode, turn it on, toggle, not pulse, and change on both
        // compare and overflow.
        omap_dm_timer_set_pwm(notle_pwm_timer, 1, 1,
                OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
        omap_dm_timer_set_load_start(notle_pwm_timer, 1, max - denom);
        omap_dm_timer_set_match(notle_pwm_timer, 1, max - num);

        __raw_writew(OMAP_MUX_MODE1, CORE_BASE_ADDR + MUX_BACKLIGHT);
        pr_info("Set brightness to %i via BACKLIGHT_HACK\n", (int)(brightness * 100));
        return 0;
}
late_initcall(notle_backlight_hack);

#endif

static void __init my_mux_init(void) {
        // Move this code to board_mux constants when we're convinced it works:

        // Example code for writing mux values, bypassing omap4_mux_init code:
        unsigned int wkup_base_addr = 0xfc31e000;

        // gpio's in the first bank of 32 use the wkup base:
        // output gpio's:
        __raw_writew(OMAP_MUX_MODE3, wkup_base_addr + MUX_GREEN_LED);
        __raw_writew(OMAP_MUX_MODE3, wkup_base_addr + MUX_YELLOW_LED);

        // Others use the core base:
        // output gpio's:
        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_AUDIO_HEADSET);
        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_GPS_ON_OFF);
        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_GPS_RESET_N);
        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_LCD_RESET_N);
        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_EN_10V);
        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_BT_RST_N);

        // Set display backlight to be pulled low when we start.
        __raw_writew(OMAP_MUX_MODE7 | OMAP_PULL_ENA, CORE_BASE_ADDR + MUX_BACKLIGHT);


        // board version gpio's:
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID2);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID1);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID0);

        // input gpio's:
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN,
                CORE_BASE_ADDR + MUX_BCM_WLAN_HOST_WAKE);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
            CORE_BASE_ADDR + MUX_CAMERA_EMU);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
            CORE_BASE_ADDR + MUX_CAMERA_DOG);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_TOUCHPAD_INT_N);


}

static int notle_notifier_call(struct notifier_block *this,
                               unsigned long code, void *cmd)
{
        void __iomem *sar_base;
        u32 v = OMAP4430_RST_GLOBAL_COLD_SW_MASK;

        sar_base = omap4_get_sar_ram_base();

        if (!sar_base)
                return notifier_from_errno(-ENOMEM);

        if ((code == SYS_RESTART) && (cmd != NULL)) {
                /* cmd != null; case: warm boot */
                if (!strcmp(cmd, "bootloader")) {
                        /* Save reboot mode in scratch memory */
                        strcpy(sar_base + 0xA0C, cmd);
                        v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
                } else if (!strcmp(cmd, "recovery")) {
                        /* Save reboot mode in scratch memory */
                        strcpy(sar_base + 0xA0C, cmd);
                        v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
                } else {
                        v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
                }
        }

        omap4_prm_write_inst_reg(0xfff, OMAP4430_PRM_DEVICE_INST,
                                 OMAP4_RM_RSTST);
        omap4_prm_write_inst_reg(v, OMAP4430_PRM_DEVICE_INST, OMAP4_RM_RSTCTRL);
        v = omap4_prm_read_inst_reg(WKUP_MOD, OMAP4_RM_RSTCTRL);

        return NOTIFY_DONE;
}

/* TODO: This is copied from panda board file.  What size should we use? */
#define NOTLE_FB_RAM_SIZE               SZ_16M /* 1920x1080*4 * 2 */
static struct omapfb_platform_data notle_fb_pdata = {
        .mem_desc = {
                .region_cnt = 1,
                .region = {
                        [0] = {
                                .size = NOTLE_FB_RAM_SIZE,
                        },
                },
        },
};

static struct notifier_block notle_reboot_notifier = {
        .notifier_call = notle_notifier_call,
};

static void __init notle_init(void)
{
        int package = OMAP_PACKAGE_CBS;
        int err;
        notle_version board_ver;

        if (omap_rev() == OMAP4430_REV_ES1_0)
                package = OMAP_PACKAGE_CBL;
        omap4_mux_init(empty_board_mux, empty_board_mux, package);
        my_mux_init();

        notle_version_init();
        board_ver = notle_version_read();
        printk("Notle board revision: %s", notle_version_str(board_ver));

        err = omap_audio_init();
        if (err) {
                pr_err("Audio initialization failed: %d\n", err);
        }

        register_reboot_notifier(&notle_reboot_notifier);
        notle_i2c_init();
        omap4_register_ion();

        if (board_ver == V3_EMU) {
          notle_button_table[0].gpio = GPIO_CAMERA_EMU;
        }

        platform_add_devices(notle_devices, ARRAY_SIZE(notle_devices));
        omap_serial_board_init(omap_serial_port_info);
        omap4_twl6030_hsmmc_init(mmc);
        usb_musb_init(&musb_board_data);
        omap_dmm_init();

        omap_vram_set_sdram_vram(NOTLE_FB_RAM_SIZE, 0);
        omapfb_set_platform_data(&notle_fb_pdata);

        err = notle_gps_init();
        if (err) {
                pr_err("GPS initialization failed: %d\n", err);
        }

        err = notle_wlan_init();
        if (err) {
                pr_err("Wifi initialization failed: %d\n", err);
        }

        err = platform_device_register(&bluetooth_rfkill_device);
        if (err) {
                pr_err("Failed to register BT rfkill device\n");
        }

        err = notle_touchpad_init();
        if (err) {
                pr_err("Touchpad initialization failed: %d\n", err);
        }

#ifdef CONFIG_OMAP2_DSS_DSI
        err = notle_dsi_init();
        if (!err) {
                omap_display_init(&notle_dss_data);
        } else {
                pr_err("DSI initialization failed: %d\n", err);
        }
#else
        err = notle_dvi_init();
        if (!err) {
                omap_display_init(&notle_dss_data);
        } else {
                pr_err("DVI initialization failed: %d\n", err);
        }

#endif
        omap_enable_smartreflex_on_init();
}

static void __init notle_map_io(void)
{
        omap2_set_globals_443x();
        omap44xx_map_common_io();
}

static void __init notle_reserve(void)
{
        /* do the static reservations first */
        memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
        memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
        /* ipu needs to recognize secure input buffer area as well */
        omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
                                    OMAP_ION_HEAP_SECURE_INPUT_SIZE);

#ifdef CONFIG_ION_OMAP
        omap_ion_init();
#else
        omap_reserve();
#endif
}

MACHINE_START(NOTLE, "OMAP4430")
	/* Maintainer: David Anders - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= notle_reserve,
	.map_io		= notle_map_io,
	.init_early	= notle_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= notle_init,
	.timer		= &omap_timer,
MACHINE_END
