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
#include <linux/hwspinlock.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/memblock.h>
#include <linux/omapfb.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/reboot.h>

#include <linux/i2c/l3g4200d.h>
#include <linux/i2c/lsm303dlhc.h>
#ifdef CONFIG_INPUT_LTR506ALS
#include <linux/i2c/ltr506als.h>
#endif
#ifdef CONFIG_INPUT_SI114X
#include <linux/i2c/si114x.h>
#endif
#include <linux/mpu.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#include "../../../drivers/input/touchscreen/rmi_i2c.h"
#endif
#ifdef CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C
#include <linux/rmi.h>
#endif

#ifdef CONFIG_INPUT_TOUCHPAD_FTK
#include <linux/i2c/ftk_patch.h>
#endif  /* CONFIG_INPUT_TOUCHPAD_FTK */

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
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
#include <plat/omap-pm.h>
#include <plat/omap-serial.h>
#include <plat/dmtimer-pwm.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <video/omap-panel-tc358762.h>
#include <video/omap-panel-notle.h>

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
#define GPIO_MPU9000_INT_TIMER          92
#define MUX_MPU9000_INT_TIMER           MUX(USBB1_ULPITLL_DAT4)
#define GPIO_MPU9000_INT                95
#define MUX_MPU9000_INT                 MUX(USBB1_ULPITLL_DAT7)
#define GPIO_PROX_INT                   90
#define MUX_PROX_INT_N                  MUX(USBB1_ULPITLL_DAT2)
#define GPIO_CAMERA_DOG                 94
#define MUX_CAMERA_DOG                  MUX(USBB1_ULPITLL_DAT6)
#define GPIO_CAMERA_EMU                 121
#define MUX_CAMERA_EMU                  MUX(ABE_DMIC_DIN2)
#define GPIO_TOUCHPAD_INT_N             32
#define MUX_TOUCHPAD_INT_N              MUX(GPMC_AD8)
#define MUX_BACKLIGHT                   MUX(USBB1_ULPITLL_DAT5)
#define GPIO_CAM_PWDN                   91
#define MUX_CAM_PWDN                    MUX(USBB1_ULPITLL_DAT3)

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
        V4_FLY      = 4,
        V5_GNU      = 5,
        V6_HOG      = 6,
} notle_version;

static notle_version NOTLE_VERSION = UNVERSIONED;

/* Read board version from GPIO.  Result in NOTLE_VERSION. */
static void notle_version_init(void)
{
        int r;

        // mux board version gpio's:
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID2);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID1);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID0);

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
        NOTLE_VERSION = gpio_get_value(GPIO_ID0) | (gpio_get_value(GPIO_ID1) << 1)
            | (gpio_get_value(GPIO_ID2) << 2);

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
        case V6_HOG:
                return "V6 HOG";
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

static struct i2c_client *himax_client;

static int write_dog_display_config(void)
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
        return ret;
}

static int notle_enable_dpi(struct omap_dss_device *dssdev) {
        gpio_set_value(dssdev->reset_gpio, 1);
        if (NOTLE_VERSION == V1_DOG) {
                write_dog_display_config();
        }
        return 0;
}

static void notle_disable_dpi(struct omap_dss_device *dssdev) {
        gpio_set_value(dssdev->reset_gpio, 0);
}

static int notle_enable_panel(void) {
        gpio_set_value(GPIO_EN_10V, 1);
        return 0;
};

static void notle_disable_panel(void) {
        gpio_set_value(GPIO_EN_10V, 0);
};

/* Using the panel-generic-dpi driver, we specify the panel name. */
static struct panel_generic_dpi_data dpi_panel = {
        .name                     = "generic-wingman",
        .platform_enable          = notle_enable_dpi,
        .platform_disable         = notle_disable_dpi,
};

/* Using the panel-notle-dpi driver, we only specify enable/disable. */
static struct panel_notle_data panel_notle = {
        .platform_enable          = notle_enable_dpi,
        .platform_disable         = notle_disable_dpi,
        .panel_enable             = notle_enable_panel,
        .panel_disable            = notle_disable_panel,
};

struct omap_dss_device panel_generic_dpi_device = {
        .type                     = OMAP_DISPLAY_TYPE_DPI,
        .name                     = "notle_generic_panel",
        .driver_name              = "generic_dpi_panel",
        .data                     = &dpi_panel,
        .phy.dpi.data_lines       = 24,
        .reset_gpio               = GPIO_LCD_RESET_N,
        .channel                  = OMAP_DSS_CHANNEL_LCD2,
};

struct omap_dss_device panel_notle_device = {
        .type                     = OMAP_DISPLAY_TYPE_DPI,
        .name                     = "notle_nhd_panel",
        .driver_name              = "panel_notle",
        .data                     = &panel_notle,
        .phy.dpi.data_lines       = 24,
        .reset_gpio               = GPIO_LCD_RESET_N,
        .channel                  = OMAP_DSS_CHANNEL_LCD2,
};

static struct tc358762_board_data dsi_panel = {
        .reset_gpio               = GPIO_LCD_RESET_N,
};

static struct omap_dss_device notle_dsi_device = {
        .name                     = "lcd",
        .driver_name              = "tc358762",
        .type                     = OMAP_DISPLAY_TYPE_DSI,
        .data                     = &dsi_panel,
        .phy.dsi                  = {
                .type             = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
                .clk_lane         = 1,
                .clk_pol          = 0,
                .data1_lane       = 2,
                .data1_pol        = 0,
        },
        .clocks                   = {
                .dispc                  = {
                        .channel                = {
                                .lck_div                = 1,        /* Logic Clock = 172.8 MHz */
                                .pck_div                = 8,        /* Pixel Clock = 34.56 MHz */
                                .lcd_clk_src            = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
                        },
                        .dispc_fclk_src         = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
                },
                .dsi                    = {
                        .regm                   = 142,  /* DSI_PLL_REGM */
                        .regn                   = 8,    /* DSI_PLL_REGN */
                        .regm_dispc             = 4,    /* PLL_CLK1 (M4) */
                        .regm_dsi               = 4,    /* PLL_CLK2 (M5) */

                        .lp_clk_div             = 18,    /* LP Clock = 8.64 MHz */
                        .dsi_fclk_src           = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
                },
        },
        .panel                  = {
        },
        .channel                = OMAP_DSS_CHANNEL_LCD,
};

static int __devinit himax_probe(struct i2c_client *client,
                                  const struct i2c_device_id *id)
{
        if (!i2c_check_functionality(client->adapter,
                                     I2C_FUNC_SMBUS_BYTE_DATA)) {
                dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
                return -EIO;
        }

        himax_client = client;
        return write_dog_display_config();
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

MODULE_DEVICE_TABLE(i2c, himax_id);

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

static struct i2c_driver himax_i2c_driver = {
        .driver = {
                .name = "notle_himax",
        },
        .probe = himax_probe,
        .remove = __devexit_p(himax_remove),
        .id_table = himax_id,
};

int __init notle_dpi_init(void)
{
        int r;

        switch (NOTLE_VERSION) {
          case V1_DOG:
            r = gpio_request_one(GPIO_EN_10V, GPIOF_OUT_INIT_HIGH, "enable_10V");
            if (r) {
                    pr_err("Failed to get enable_10V gpio\n");
                    goto err;
            }

            r = gpio_request_one(panel_generic_dpi_device.reset_gpio,
                                 GPIOF_OUT_INIT_LOW, "DVI PD");
            if (r) {
                    pr_err("Failed to get DVI powerdown GPIO\n");
                    goto err1;
            }
            i2c_add_driver(&himax_i2c_driver);
            notle_enable_dpi(&panel_generic_dpi_device);
            break;
          case V4_FLY:
          case V5_GNU:
          case V6_HOG:
            r = gpio_request_one(GPIO_EN_10V, GPIOF_OUT_INIT_LOW, "enable_10V");
            if (r) {
                    pr_err("Failed to get enable_10V gpio\n");
                    goto err;
            }

            r = gpio_request_one(panel_notle_device.reset_gpio,
                                 GPIOF_OUT_INIT_LOW, "DVI PD");
            if (r) {
                    pr_err("Failed to get DVI powerdown GPIO\n");
                    goto err1;
            }
            break;
          default:
            pr_err("Unrecognized Notle version initializing DPI\n");
            r = -1;
            goto err;
        }

        return 0;
err1:
        gpio_free(GPIO_EN_10V);
err:
        return r;
}

static struct omap_dss_device *dog_dss_devices[] = {
        &panel_generic_dpi_device,
};
static struct omap_dss_device *emu_dss_devices[] = {
        &notle_dsi_device,
};
static struct omap_dss_device *fly_dss_devices[] = {
        &panel_notle_device,
};


static struct omap_dss_board_info notle_dsi_dss_data = {
        .num_devices    = ARRAY_SIZE(emu_dss_devices),
        .devices        = emu_dss_devices,
        .default_device = &notle_dsi_device,
};

static struct omap_dss_board_info generic_dpi_dss_data = {
        .num_devices    = ARRAY_SIZE(dog_dss_devices),
        .devices        = dog_dss_devices,
        .default_device = &panel_generic_dpi_device,
};

static struct omap_dss_board_info panel_notle_dss_data = {
        .num_devices    = ARRAY_SIZE(fly_dss_devices),
        .devices        = fly_dss_devices,
        .default_device = &panel_notle_device,
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
                // TODO(rocky): Experiment with turning this off to see if
                // it improves hsmmc suspend/resume problem.  Measured
                // effect on current draw is zero.
                .power_saving   = false,
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
        /* This device is only present on Dog devices.  It will be blanked out
         * below (by setting .mmc to 0) for other versions.
         */
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_29_30,
                .no_off_init    = true,
                .power_saving   = true,
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
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,

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
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
        .num_consumer_supplies = 1,
        .consumer_supplies = notle_cam2_supply,
};

static struct regulator_consumer_supply notle_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

// Voltage for display
static struct regulator_init_data dog_vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data emu_vaux3 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 1200000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(notle_vcxio_supply),
	.consumer_supplies	= notle_vcxio_supply,
};

static struct regulator_init_data fly_vaux3 = {
	.constraints = {
		.min_uV			= 1500000,
		.max_uV			= 1500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data hog_vaux3 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 1200000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

#ifdef CONFIG_MMC_NOTLE
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
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = notle_vmmc_supply,
};
#else
/* Unused mmc slot 1 on standard notle */
static struct regulator_init_data notle_vmmc = {
	.constraints = {
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = notle_vmmc_supply,
};
#endif

/* unused */
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
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
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
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* unused */
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
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* 1.8V for "lots of important stuff" - clocks, plls, etc. */
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
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* A/D convertor? */
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
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply notle_vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

/* Powers OMAP's USB controller */
/* TODO(rocky): This still doesn't seem to get hooked up properly with the
 * twl6030 driver.  i.e. I still see these in the kernel log:
 * [   77.002502] suspend_set_state: VUSB: No configuration
 */
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
		.state_mem = {
			.enabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(notle_vusb_supply),
	.consumer_supplies      = notle_vusb_supply,
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
                .wer = 0,
        },
        { /* ttyO1 bluetooth */
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
                .wake_peer = bcm_bt_lpm_exit_lpm_locked,
                .rts_mux_driver_control = 1,
                .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
        },
        { /* ttyO2 console port */
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
                .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
        },
        {  /* ttyO3 GPS */
                .use_dma        = 1,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                /* TODO(cmanton) Use interrupts for better (?) power management */
                .dma_rx_poll_rate = 100000,  // units in usec
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
                .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
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

static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};


// Translate hardware buttons to keys -- we have only one.  Note that
// GPIO_CAMERA_EMU may be overwritten by GPIO_CAMERA_DOG below.
static struct gpio_keys_button notle_button_table[] = {
    [0] = {
                .code   = KEY_CAMERA,           \
                .gpio   = GPIO_CAMERA_EMU,      \
                .desc   = "Camera",             \
                .type   = EV_KEY,               \
                .wakeup = 1,                    \
                .active_low = 1,                \
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
        &bcm4330_bluetooth_device,
};


static struct twl4030_bci_platform_data notle_bci_data = {
        .monitoring_interval            = 10,
        .max_charger_currentmA          = 1500,
        .max_charger_voltagemV          = 4560,
        .max_bat_voltagemV              = 4200,
        .low_bat_voltagemV              = 3300,
};


static struct twl4030_codec_audio_data twl6040_audio = {
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
	.ep_step	= 0x0f,
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

static struct twl4030_platform_data dog_twldata = {
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
	.vaux3		= &dog_vaux3,
        .clk32kg        = &omap4_notle_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* children */
        .codec          = &twl6040_codec,
	.bci            = &notle_bci_data,
	.madc           = &notle_gpadc_data,
};

static struct twl4030_platform_data emu_twldata = {
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
	.vaux3		= &emu_vaux3,
        .clk32kg        = &omap4_notle_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* children */
        .codec          = &twl6040_codec,
	.bci            = &notle_bci_data,
	.madc           = &notle_gpadc_data,
};

static struct twl4030_platform_data fly_twldata = {
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
	.vaux3		= &fly_vaux3,
        .clk32kg        = &omap4_notle_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* children */
        .codec          = &twl6040_codec,
	.bci            = &notle_bci_data,
	.madc           = &notle_gpadc_data,
};

static struct twl4030_platform_data hog_twldata = {
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
	.vaux3		= &hog_vaux3,
        .clk32kg        = &omap4_notle_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* children */
        .codec          = &twl6040_codec,
	.bci            = &notle_bci_data,
	.madc           = &notle_gpadc_data,
};

#if defined(CONFIG_INPUT_TOUCHPAD_FTK) && defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C)
#error "Can only define either FTK or Synaptics touchpad"
#endif

#if defined(CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C) && defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C)
#error "Can only define touchpad (v2) or touchscreen (v1) Synaptics touchpad"
#endif

#ifdef CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C

static union rmi_f11_2d_ctrl0 f11_ctrl0 = {
	{
		/* ReportingMode = ‘000’: Continuous, when finger present.
		 * ReportingMode = ‘001’: Reduced reporting mode.
		 * ReportingMode = ‘010’: Finger-state change reporting mode.
		 * ReportingMode = ‘011’: Finger-presence change reporting mode. */
		.reporting_mode = 1,
		/* Enable filtering of the reported absolute positioning. */
		.abs_pos_filt = 0,
		/* Enable filtering of the reported relative positioning. */
		.rel_pos_filt = 0,
		/* Enable ballistics processing for relative finger motion. */
		.rel_ballistics = 0,
		.dribble = 0,
		.report_beyond_clip = 0,
	},
};

static union rmi_f11_2d_ctrl1 f11_ctrl1 = {
	{
		/* Specifies threshold at which a finger is considered a palm.
		 * Zero disables. */
		.palm_detect_thres = 0,
		/* Motion sensitivity.
		 * '00': Low
		 * '01': Medium
		 * '10': High
		 * '11': Infinite */
		.motion_sensitivity = 0,
		/* '0': Firmware determines tracked finger.
		 * '1': Host determines tracked finger. */
		.man_track_en = 0,
		/* Which finger being tracked.
		 * '0': Track finger 0
		 * '1': Track finger 1 */
		.man_tracked_finger = 0,
	},
};

static union rmi_f11_2d_ctrl2__3 f11_ctrl2__3 = {
	{
		.delta_x_threshold = 1,
		.delta_y_threshold = 1,
	},
};

static union rmi_f11_2d_ctrl4 f11_ctrl4 = {
	{
		/* Define velocity ballistic parameter to all relative
		 * motion events.  Zero disables. */
		.velocity = 0,
	},
};

static union rmi_f11_2d_ctrl5 f11_ctrl5 = {
	{
		/* Define acceleration ballistic parameter to all relative
		 * motion events.  Zero disables. */
		.acceleration = 0,
	},
};

static union rmi_f11_2d_ctrl6__7 f11_ctrl6__7 = {
	{
		/* Maximum sensor X position. */
		.sensor_max_x_pos = 0x0490,
	},
};

static union rmi_f11_2d_ctrl8__9 f11_ctrl8__9 = {
	{
		/* Maximum sensor Y position. */
		.sensor_max_y_pos = 0x00b8,
	},
};

static union rmi_f11_2d_ctrl10 f11_ctrl10 = {
	{
		/* These bits enable the feature iff the corresponding bit
		 * is set in the query register. */
		.single_tap_int_enable = 1,
		.tap_n_hold_int_enable = 1,
		.double_tap_int_enable = 1,
		.early_tap_int_enable = 1,
		.flick_int_enable = 1,
		.press_int_enable = 1,
		.pinch_int_enable = 1,
	},
};

static union rmi_f11_2d_ctrl11 f11_ctrl11 = {
	{
		/* These bits enable the feature iff the corresponding bit
		 * is set in the query register. */
		.palm_detect_int_enable = 1,
		.rotate_int_enable = 1,
		.touch_shape_int_enable = 1,
		.scroll_zone_int_enable = 1,
		.multi_finger_scroll_int_enable = 1,
	},
};

/* All the bits of this data structures must have accurate default
 * control data because of the way the rmi driver writes the control
 * registers to the device. */
static struct rmi_f11_2d_ctrl f11_ctrl = {
	.ctrl0 = &f11_ctrl0,
	.ctrl1 = &f11_ctrl1,
	.ctrl2__3 = &f11_ctrl2__3,
	.ctrl4 = &f11_ctrl4,
	.ctrl5 = &f11_ctrl5,
	.ctrl6__7 = &f11_ctrl6__7,
	.ctrl8__9 = &f11_ctrl8__9,
	.ctrl10 = &f11_ctrl10,
	.ctrl11 = &f11_ctrl11,
};

static struct rmi_device_platform_data synaptics_platformdata = {
	.driver_name = "rmi-generic",

	.irq = GPIO_TOUCHPAD_INT_N,
	.irq_polarity = RMI_IRQ_ACTIVE_LOW,
	.gpio_config = NULL,

	/* function handler pdata */
	.f11_ctrl = &f11_ctrl,
	.axis_align = {
		.swap_axes = false,
		.flip_x = false,
		.flip_y = true,
		.clip_X_low = 0,
		.clip_Y_low = 0,
		.clip_X_high = 0,
		.clip_Y_high = 0,
		.offset_X = 0,
		.offset_Y = 0,
		.rel_report_enabled = 0,
	},
	.button_map = NULL,
#ifdef CONFIG_PM
	.pm_data = NULL,
	.pre_suspend = NULL,
	.post_resume = NULL,
#endif
};
#ifndef RMI_F11_INDEX
#define RMI_F11_INDEX 0x11
#endif  // RMI_F11_INDEX
#endif

#ifdef CONFIG_INPUT_TOUCHPAD_FTK
static struct ftk_i2c_platform_data ftk_platformdata = {
  /* TODO(cmanton) Implement this based upon pending information from ST
   * For now do nothing */
  .power = NULL,
};
#endif  /* CONFIG_INPUT_TOUCHPAD_FTK */

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
static struct rmi_sensor_suspend_custom_ops synaptics_custom_ops = {
        .rmi_sensor_custom_suspend = 0,
        .rmi_sensor_custom_resume = 0,
        .delay_resume = 0,
};

static struct rmi_f11_functiondata synaptics_f11_data = {
        /* flip_X, flip_Y, and swap_axes will be set based on NOTLE_VERSION */
};

static struct rmi_functiondata synaptics_fndata = {
        .function_index     = RMI_F11_INDEX,
        .data               = &synaptics_f11_data,
};

static struct rmi_functiondata_list synaptics_fndatalist = {
        .count              = 1,
        .functiondata       = &synaptics_fndata,
};

static struct rmi_sensordata __initdata synaptics_sensordata = {
        .rmi_sensor_setup = 0,
        .rmi_sensor_teardown = 0,
        .attn_gpio_number = GPIO_TOUCHPAD_INT_N,
        .attn_polarity = 0,
        .custom_suspend_ops = &synaptics_custom_ops,
        .perfunctiondata = &synaptics_fndatalist,
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
#ifdef CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C
        {
                I2C_BOARD_INFO("rmi", 0x20),
                .platform_data = &synaptics_platformdata,
        },
#endif
#ifdef CONFIG_INPUT_TOUCHPAD_FTK
        {
                I2C_BOARD_INFO("ftk", 0x4b),
                .platform_data = &ftk_platformdata,
        },
#endif  /* CONFIG_TOUCHPAD_FTK */
        {
                I2C_BOARD_INFO("stmpe32m28", 0x48),
        },
};

/*
 * i2c-4 
 */
static struct mpu_platform_data mpu9150_data = {
        .int_config     = 0x10,
        .orientation    = { 0, 0, -1,
                            0, 1, 0,
                            1, 0, 0 },
        .level_shifter  = 1,
        /*
        .accel          = {
                .get_slave_descr = mantis_get_slave_descr,
                .adapt_num = 4,
                .bus = EXT_SLAVE_BUS_SECONDARY,
                .address = 0x68,
                .orientation = { 0, 0, -1,
                                 0, 1, 0,
                                 1, 0, 0 },
        },
        .compass        = {
                .get_slave_descr = ak8975_get_slave_descr,
                .adapt_num = 4,
                .bus = EXT_SLAVE_BUS_SECONDARY,
                .address = 0x0C,
                .orientation = { 0, 0, 1,
                                 1, 0, 0,
                                 0, 1, 0 },
        },
        */
};

/* compass */
static struct ext_slave_platform_data ak8975_compass_data = {
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.orientation = { 0, 0, 1,
			 1, 0, 0,
			 0, 1, 0 },
};


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

#ifdef CONFIG_INPUT_LTR506ALS
static struct ltr506_platform_data notle_ltr506als_data = {
	/* Interrupt */
	.pfd_gpio_int_no = GPIO_PROX_INT,

	/* Disable als on suspend flag */
	.pfd_disable_als_on_suspend = 1,

	/* ALS enable filtering interrupts
	 * by suppressing interrupts when measured value
	 * falls within some driver calculated threshold.  This
	 * can be manually set by setting a '0' here and writing
	 * to the sysfs threshold file.
	 * e.g.
	 *  echo "1000 1020" > /sys/class/input/input7/als_threshold
	 */
	.pfd_als_filter_interrupts = 1,

	/* ALS measurement repeat rate
	 * '000:  100ms
	 * '001:  200ms
	 * '010:  500ms
	 * '011: 1000ms
	 * '1xx: 2000ms */
	.pfd_als_meas_rate = 3,

	/* ALS gain.
	 * '00: 1 lux/count (1-64k lux)
	 * '01: 0.5 lux/count (0.5-32k lux)
	 * '10: 0.01 lux/count (0.02-640 lux)
	 * '11: 0.005 lux/count (0.01-32.0 lux) */
	.pfd_als_gain = 0,

	/* Disable ps on suspend flag */
	.pfd_disable_ps_on_suspend = 0,

	/* PS enable filtering interrupts
	 * by suppressing interrupts when measured value
	 * falls within some driver calculated threshold.  This
	 * can be manually set by setting a '0' here and writing
	 * to the sysfs threshold file.
	 * e.g.
	 *  echo "1000 1020" > /sys/class/input/input7/als_threshold
	 */
	.pfd_ps_filter_interrupts = 1,

	/* PS measurement repeate rate
	 * '000:   12.5ms (ALS auto-disabled)
	 * '001:   50ms
	 * '010:   70ms
	 * '011:  100ms
	 * '100:  200ms
	 * '101:  500ms
	 * '110: 1000ms
	 * '111: 2000ms */
	.pfd_ps_meas_rate = 4,

	/* PS gain.
	 * '00:  x8 gain
	 * '01: x16 gain
	 * '10: x32 gain
	 * '11: x64 gain */
	.pfd_ps_gain = 1,

	/* LED pulse frequency.
	 * '000: 30kHz
	 * '001: 40kHz
	 * '010: 50kHz
	 * '011: 60kHz
	 * '100: 70kHz
	 * '101: 80kHz
	 * '110: 90kHz
	 * '111: 100kHz */
	.pfd_led_pulse_freq = 7,

	/* LED Duty cycle.
	 * '00:  25%
	 * '01:  50%
	 * '10:  75%
	 * '11: 100% */
	.pfd_led_duty_cyc = 1,

	/* LED peak current.
	 * '000:   5mA
	 * '001:  10mA
	 * '010:  20mA
	 * '011:  50mA
	 * '1xx: 100mA */
	.pfd_led_peak_curr = 0,

	/* LED Pulse count. Number of LED pulses to be
	 * emitted for a measurement. */
	.pfd_led_pulse_count = 127,
};
#endif


static struct i2c_board_info __initdata notle_dog_i2c_4_boardinfo[] = {
        {
                I2C_BOARD_INFO("l3g4200d_gyr", 0x68),
                .flags = I2C_CLIENT_WAKE,
        //        .irq = OMAP44XX_IRQ_SYS_1N,
                .platform_data = &notle_l3g4200d_data,
        },
        {
                I2C_BOARD_INFO("lsm303dlhc_acc", 0x18),
                .flags = I2C_CLIENT_WAKE,
        //        .irq = OMAP44XX_IRQ_SYS_1N,
                .platform_data = &notle_lsm303dlh_acc_data,
        },
        {
                I2C_BOARD_INFO("lsm303dlhc_mag", 0x1e),
                .flags = I2C_CLIENT_WAKE,
        //        .irq = OMAP44XX_IRQ_SYS_1N,
                .platform_data = &notle_lsm303dlh_mag_data,
        },
#ifdef CONFIG_INPUT_LTR506ALS
        {
                I2C_BOARD_INFO("ltr506als", 0x3a),
                .flags = I2C_CLIENT_WAKE,
                .irq = OMAP_GPIO_IRQ(GPIO_PROX_INT),
                .platform_data = &notle_ltr506als_data,
        },
#endif
        {
                I2C_BOARD_INFO("notle_himax", 0x48),
        },
#ifdef CONFIG_INPUT_SI114X
        {
                I2C_BOARD_INFO("si114x", 0x5a),
//                .flags = I2C_CLIENT_WAKE,
 //               .irq = OMAP_GPIO_IRQ(GPIO_PROX_INT),
  //              .platform_data = &notle_ltr506als_data,
        },
#endif
        {
                I2C_BOARD_INFO("ov9726", 0x10),
                .flags = I2C_CLIENT_WAKE,
        },
};

static struct i2c_board_info __initdata notle_emu_i2c_4_boardinfo[] = {
        {
                I2C_BOARD_INFO("tc358762-i2c", 0x0b),
        },
        {
                I2C_BOARD_INFO("mpu6050", 0x68),
                .irq = OMAP_GPIO_IRQ(GPIO_MPU9000_INT),
                .platform_data = &mpu9150_data,
        },
        {
                I2C_BOARD_INFO("ak8975", 0xC),
                .irq = OMAP_GPIO_IRQ(GPIO_MPU9000_INT),
                .platform_data = &ak8975_compass_data,
        },
#ifdef CONFIG_INPUT_LTR506ALS
        {
                I2C_BOARD_INFO("ltr506als", 0x3a),
                .flags = I2C_CLIENT_WAKE,
                .irq = OMAP_GPIO_IRQ(GPIO_PROX_INT),
                .platform_data = &notle_ltr506als_data,
        },
#endif
        {
                I2C_BOARD_INFO("notle_himax", 0x48),
        },
        {
                I2C_BOARD_INFO("ov5650", 0x36),
                .flags = I2C_CLIENT_WAKE,
        },
};

static void __init notle_pmic_mux_init(void)
{

        omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
                                          OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
                                struct omap_i2c_bus_board_data *pdata)
{
       /* spinlock_id should be -1 for a generic lock request */
       if (spinlock_id < 0)
               pdata->handle = hwspin_lock_request();
       else
               pdata->handle = hwspin_lock_request_specific(spinlock_id);

       if (pdata->handle != NULL) {
               pdata->hwspin_lock_timeout = hwspin_lock_timeout;
               pdata->hwspin_unlock = hwspin_unlock;
       } else {
               pr_err("I2C hwspinlock request failed for bus %d\n", \
                                                               bus_id);
       }
}

static struct omap_i2c_bus_board_data __initdata notle_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata notle_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata notle_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata notle_i2c_4_bus_pdata;

static struct i2c_board_info __initdata notle_fly_i2c_4_boardinfo[] = {
        {
                I2C_BOARD_INFO("panel-notle-fpga", 0x0A),
        },
        {
                I2C_BOARD_INFO("panel-notle-panel", 0x49),
        },
        {
                I2C_BOARD_INFO("mpu6050", 0x68),
                .irq = OMAP_GPIO_IRQ(GPIO_MPU9000_INT),
                .platform_data = &mpu9150_data,
        },
        {
                I2C_BOARD_INFO("ak8975", 0xC),
                .irq = OMAP_GPIO_IRQ(GPIO_MPU9000_INT),
                .platform_data = &ak8975_compass_data,
        },
#ifdef CONFIG_INPUT_LTR506ALS
        {
                I2C_BOARD_INFO("ltr506als", 0x3a),
                .flags = I2C_CLIENT_WAKE,
                .irq = OMAP_GPIO_IRQ(GPIO_PROX_INT),
                .platform_data = &notle_ltr506als_data,
        },
#endif
        {
                I2C_BOARD_INFO("notle_himax", 0x48),
        },
        {
                I2C_BOARD_INFO("ov9726", 0x10),
                .flags = I2C_CLIENT_WAKE,
        },
};

static int __init notle_i2c_init(void)
{
        omap_i2c_hwspinlock_init(1, 0, &notle_i2c_1_bus_pdata);
        omap_i2c_hwspinlock_init(2, 1, &notle_i2c_2_bus_pdata);
        omap_i2c_hwspinlock_init(3, 2, &notle_i2c_3_bus_pdata);
        omap_i2c_hwspinlock_init(4, 3, &notle_i2c_4_bus_pdata);

        omap_register_i2c_bus_board_data(1, &notle_i2c_1_bus_pdata);
        omap_register_i2c_bus_board_data(2, &notle_i2c_2_bus_pdata);
        omap_register_i2c_bus_board_data(3, &notle_i2c_3_bus_pdata);
        omap_register_i2c_bus_board_data(4, &notle_i2c_4_bus_pdata);

        switch (NOTLE_VERSION) {
          case V1_DOG:
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
            synaptics_f11_data.flip_X = true;
            synaptics_f11_data.flip_Y = false;
            synaptics_f11_data.swap_axes = true;
#endif  /* CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C */

            omap4_pmic_init("twl6030", &dog_twldata);
            omap_register_i2c_bus(2, 400, NULL, 0);
            omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
                            ARRAY_SIZE(notle_i2c_3_boardinfo));
            omap_register_i2c_bus(4, 400, notle_dog_i2c_4_boardinfo,
                            ARRAY_SIZE(notle_dog_i2c_4_boardinfo));
            break;
          case V3_EMU:
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
            synaptics_f11_data.flip_X = false;
            synaptics_f11_data.flip_Y = true;
            synaptics_f11_data.swap_axes = true;
#endif  /* CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C */

            omap4_pmic_init("twl6030", &emu_twldata);
            omap_register_i2c_bus(2, 400, NULL, 0);
            omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
                            ARRAY_SIZE(notle_i2c_3_boardinfo));
            omap_register_i2c_bus(4, 400, notle_emu_i2c_4_boardinfo,
                            ARRAY_SIZE(notle_emu_i2c_4_boardinfo));
            break;
          case V4_FLY:
          case V5_GNU:
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
            synaptics_f11_data.flip_X = false;
            synaptics_f11_data.flip_Y = true;
            synaptics_f11_data.swap_axes = false;
#endif  /* CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C */
            omap4_pmic_init("twl6030", &fly_twldata);
            omap_register_i2c_bus(2, 400, NULL, 0);
            omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
                            ARRAY_SIZE(notle_i2c_3_boardinfo));
            omap_register_i2c_bus(4, 400, notle_fly_i2c_4_boardinfo,
                            ARRAY_SIZE(notle_fly_i2c_4_boardinfo));
            break;
          case V6_HOG:
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
            synaptics_f11_data.flip_X = false;
            synaptics_f11_data.flip_Y = true;
            synaptics_f11_data.swap_axes = false;
#endif  /* CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C */
            omap4_pmic_init("twl6030", &hog_twldata);
            omap_register_i2c_bus(2, 400, NULL, 0);
            omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
                            ARRAY_SIZE(notle_i2c_3_boardinfo));
            omap_register_i2c_bus(4, 400, notle_fly_i2c_4_boardinfo,
                            ARRAY_SIZE(notle_fly_i2c_4_boardinfo));
            break;
          default:
            pr_err("Unrecognized Notle version: %i\n", NOTLE_VERSION);
            return -1;
            break;
        }
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
        int audio_power_on_gpio = GPIO_AUDIO_POWERON_EMU;

        /* Set the correct audio power on GPIO based on board revision */


        /* TODO(petermalkin) Whenever we get rid of i2c errors, remove the manual GPIO settings */
        /*      and put back the setting for the twl6030 driver to set poweron pin for audio    */
        /*      like this: audio_power_on_gpio = AUDIO_GPIO_62;                            */
        if (NOTLE_VERSION == V1_DOG) {
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

/* XXX Turning on GPS currently costs us ~50mA of current draw.
static int __init notle_gps_start(void) {
	gpio_set_value(GPIO_GPS_ON_OFF, 1);
        pr_info("Turning on GPS chip\n");
        return 0;
}
late_initcall(notle_gps_start);
*/

static int __init notle_touchpad_init(void) {
        int r;

        pr_info("%s()+\n", __func__);

        /* Configuration of requested GPIO line */

        r = gpio_request_one(GPIO_TOUCHPAD_INT_N, GPIOF_IN, "touchpad_int_n");
        if (r) {
                pr_err("Failed to get touchpad_int_n gpio\n");
        }
        /* Allow this interrupt to wake the system */
        r = irq_set_irq_wake(gpio_to_irq(GPIO_TOUCHPAD_INT_N), 1);
        if (r) {
                pr_err("%s Unable to set irq to wake device\n", __FUNCTION__);
        }
        return r;
}

static int __init notle_imu_init(void) {
        int r;

        pr_info("%s()+\n", __func__);

        /* Configuration of requested GPIO line */

        r = gpio_request_one(GPIO_MPU9000_INT_TIMER, GPIOF_IN, "mpuirq_timer");
        if (r) {
                pr_err("Failed to get mpu9000_int_timer gpio\n");
        } else {
                pr_err("got the mpu9000 timer gpio!!!\n");
        }
        /*
        r = gpio_request_one(GPIO_MPU9000_INT, GPIOF_IN, "mpuirq");
        */
        r = gpio_request(GPIO_MPU9000_INT, "mpuirq");
        if (r) {
                pr_err("Failed to get mpu9000_int gpio\n");
        }
        r = gpio_direction_input(GPIO_MPU9000_INT);
        if (r) {
                pr_err("Failed to get mpu9000_int gpio\n");
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
#ifdef CONFIG_REMOTEPROC_WATCHDOG
        // the watchdog uses gpt9.
        pr_err("NOT setting up backlight pwm!  Turn off REMOTERPOC_WATCHDOG.\n");
        return -1;
#endif
        pwm = pwm_request_dmtimer(PWM_TIMER, "backlight",
                                  &notle_dmtimer_pwm_ops);

        if (!pwm) {
                pr_err("Failed to request backlight dmtimer\n");
                return -1;
        }

        backlight_data.pwm_id = pwm->pwm_id;
        if (NOTLE_VERSION == V3_EMU) {
          // Emu has a dimmer display; increase the max brightness.
          backlight_data.uth_brightness = 0xFF;
          // Since Emu has no status LEDs, start at ~half brightness so user
          // knows we're booting.
          backlight_data.dft_brightness = 0x80;
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
        int flags;
        // Move this code to board_mux constants when we're convinced it works:

        // Example code for writing mux values, bypassing omap4_mux_init code:
        unsigned int wkup_base_addr = 0xfc31e000;

        // gpio's in the first bank of 32 use the wkup base:
        // output gpio's:
        flags = OMAP_MUX_MODE3;
        __raw_writew(flags, wkup_base_addr + MUX_GREEN_LED);
        __raw_writew(flags, wkup_base_addr + MUX_YELLOW_LED);

        // Others use the core base:
        // output gpio's:
        __raw_writew(flags, CORE_BASE_ADDR + MUX_AUDIO_HEADSET);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_GPS_ON_OFF);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_GPS_RESET_N);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_LCD_RESET_N);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_EN_10V);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_BT_RST_N);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_CAM_PWDN);

        // Set display backlight to be pulled down when we start.
        flags = OMAP_MUX_MODE7 | OMAP_PULL_ENA;
        // Emu has no status LEDs, so for now we're flashing full brightness
        // as an early boot indication.
        if (NOTLE_VERSION == V3_EMU) {
                flags |= OMAP_PULL_UP;
        }
        __raw_writew(flags, CORE_BASE_ADDR + MUX_BACKLIGHT);


        flags = OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN;

        // input gpio's:
        __raw_writew(flags, CORE_BASE_ADDR + MUX_BCM_WLAN_HOST_WAKE);
        if (NOTLE_VERSION == V1_DOG) {
          __raw_writew(flags, CORE_BASE_ADDR + MUX_CAMERA_DOG);
        } else {
          __raw_writew(flags, CORE_BASE_ADDR + MUX_CAMERA_EMU);
        }
        __raw_writew(flags, CORE_BASE_ADDR + MUX_TOUCHPAD_INT_N);
        __raw_writew(flags, CORE_BASE_ADDR + MUX_PROX_INT_N);

        // Invensense part configured as push-pull.  Don't need omap to pullup.
        // TODO: (rocky) Do we want wakeup enabled?
        flags = OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN;
        if (NOTLE_VERSION != V1_DOG) {
          __raw_writew(flags, CORE_BASE_ADDR + MUX_MPU9000_INT);
          __raw_writew(flags, CORE_BASE_ADDR + MUX_MPU9000_INT_TIMER);
        }

}

#define TWL6030_PHOENIX_DEV_ON_REGISTER (0x25)
#define TWL6030_SW_RESET_BIT_MASK       (0x40)

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

        if (v == OMAP4430_RST_GLOBAL_COLD_SW_MASK) {

                /* Here we are certain we have no commands              */
                /* that need to be passed to the bootloader.            */
                /* Request a full power down / power up cycle from pmic */

                twl_i2c_write_u8(TWL_MODULE_RTC,
                                 TWL6030_SW_RESET_BIT_MASK,
                                 TWL6030_PHOENIX_DEV_ON_REGISTER);
        }

        /* if for some reason communication to pmic failed, */
        /* proceed to regular cold sw reset                 */

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

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *     EMIF1 - CS0 -   2 Gb
 *             CS1 -   2 Gb
 *     EMIF2 - CS0 -   2 Gb
 *             CS1 -   2 Gb
 *     --------------------
 *     TOTAL -         8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
        .cs0_device = &lpddr2_elpida_2G_S4_dev,
        .cs1_device = &lpddr2_elpida_2G_S4_dev
};


static void __init notle_init(void)
{
        int package = OMAP_PACKAGE_CBS;
        int err;
        int wifi_power_gpio;

        omap_emif_setup_device_details(&emif_devices, &emif_devices);

        if (omap_rev() == OMAP4430_REV_ES1_0)
                package = OMAP_PACKAGE_CBL;
        omap4_mux_init(empty_board_mux, empty_board_mux, package);
        notle_version_init();
        my_mux_init();
        notle_pmic_mux_init();

        printk("Notle board revision: %s(%d)", notle_version_str(NOTLE_VERSION), NOTLE_VERSION);

        err = omap_audio_init();
        if (err) {
                pr_err("Audio initialization failed: %d\n", err);
        }

        register_reboot_notifier(&notle_reboot_notifier);

#ifndef CONFIG_MMC_NOTLE
        // Disable support for sd card:
        mmc[2].mmc = 0;
#endif

        err = notle_imu_init();
        if (err) {
                pr_err("IMU initialization failed: %d\n", err);
        }
        notle_i2c_init();
        omap4_register_ion();

        if (NOTLE_VERSION == V1_DOG) {
          notle_button_table[0].gpio = GPIO_CAMERA_DOG;
        }

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

        switch (NOTLE_VERSION) {
          case V1_DOG:
          case V3_EMU:
          case V4_FLY:
          case V5_GNU:
            wifi_power_gpio = GPIO_WL_BT_REG_ON;
            break;
          case V6_HOG:
          default:
            wifi_power_gpio = GPIO_WL_RST_N;
        }
        err = notle_wlan_init(wifi_power_gpio);
        if (err) {
                pr_err("Wifi initialization failed: %d\n", err);
        }

        // Do this after the wlan_init, which inits the regulator shared
        // with the bluetooth device and muxes the bt signals.
        platform_add_devices(notle_devices, ARRAY_SIZE(notle_devices));

        err = notle_touchpad_init();
        if (err) {
                pr_err("Touchpad initialization failed: %d\n", err);
        }

        switch (NOTLE_VERSION) {
          case V1_DOG:
            err = notle_dpi_init();
            if (!err) {
                    omap_display_init(&generic_dpi_dss_data);
            } else {
                    pr_err("DPI initialization failed: %d\n", err);
            }
            break;
          case V3_EMU:
            err = notle_dsi_init();
            if (!err) {
                    omap_display_init(&notle_dsi_dss_data);
            } else {
                    pr_err("DSI initialization failed: %d\n", err);
            }
            break;
          case V4_FLY:
          case V5_GNU:
          case V6_HOG:
            err = notle_dpi_init();
            if (!err) {
                    panel_notle.notle_version = NOTLE_VERSION;
                    omap_display_init(&panel_notle_dss_data);
            } else {
                    pr_err("DPI initialization failed: %d\n", err);
            }
            break;
          default:
            pr_err("No display supported for Notle version: %s\n",
                   notle_version_str(NOTLE_VERSION));
            break;
        }

        omap_enable_smartreflex_on_init();
        omap_pm_enable_off_mode();
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
                                    OMAP4_ION_HEAP_SECURE_INPUT_SIZE);

#ifdef CONFIG_ION_OMAP
        omap_ion_init();
#endif
        omap_reserve();
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
