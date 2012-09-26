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
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/reboot.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>

#ifdef CONFIG_INPUT_LTR506ALS
#include <linux/i2c/ltr506als.h>
#endif
#ifdef CONFIG_INPUT_SI114X
#include <linux/i2c/si114x.h>
#endif
#include <linux/mpu.h>

#ifdef CONFIG_INPUT_GLASSHUB
#include <linux/i2c/glasshub.h>
#endif

#include <linux/mpu.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

/* Version 1 Touchpad driver */
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#include "../../../drivers/input/touchscreen/rmi_i2c.h"
#endif
/* Version 2 and 3 touchpad drivers. */
#if defined(CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C) || defined(CONFIG_RMI4_BUS)
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
#include <mach/omap4_ion.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system.h>

#include <plat/android-display.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include <plat/omap-serial.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <video/omap-panel-tc358762.h>
#include <video/omap-panel-notle.h>

#include "timer-gp.h"
#include "omap_ram_console.h"
#include "hsmmc.h"
#include "control.h"
#include "pm.h"
#include "common-board-devices.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "cm1_44xx.h"
#include "omap4-sar-layout.h"


#define PWM_TIMER                       9

static notle_version NOTLE_VERSION = UNVERSIONED;

// gpio pin assignment settings for various board types
// TODO(jscarr) get rid of common, or put in complete set
// HOG support is needed for a little longer, HOG & EVT1 were similar
static int notle_gpio_board_hog[GPIO_MAX_INDEX] = {
    [GPIO_MPU9000_INT_TIMER_INDEX] = GPIO_MPU9000_INT_TIMER_EVT1,
    [GPIO_MPU9000_INT_INDEX] = GPIO_MPU9000_INT_EVT1,
    [GPIO_USB_MUX_CB0_INDEX] = GPIO_USB_MUX_CB0_EVT1,
    [GPIO_USB_MUX_CB1_INDEX] = GPIO_USB_MUX_CB1,
    [GPIO_GPS_ON_OFF_INDEX] = GPIO_GPS_ON_OFF_EVT1,
    [GPIO_GPS_RESET_N_INDEX] = GPIO_GPS_RESET_N_EVT1,
    [GPIO_LCD_RST_N_INDEX] = GPIO_LCD_RST_N_EVT1,
    [GPIO_DISP_ENB_INDEX] = GPIO_DISP_ENB,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT1,
    [GPIO_CAM_PWDN_INDEX] = GPIO_CAM_PWDN_EVT1,
    [GPIO_TOUCHPAD_INT_N_INDEX] = GPIO_TOUCHPAD_INT_N_EVT1,
    [GPIO_PROX_INT_INDEX] = GPIO_PROX_INT_EVT1,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT1,
    [GPIO_BCM_BT_HOST_WAKE_INDEX] = GPIO_BCM_BT_HOST_WAKE_EVT1,
    [GPIO_BCM_WLAN_HOST_WAKE_INDEX] = GPIO_BCM_WLAN_HOST_WAKE_HOG,
};
static int notle_gpio_board_evt1[GPIO_MAX_INDEX] = {
    [GPIO_MPU9000_INT_TIMER_INDEX] = GPIO_MPU9000_INT_TIMER_EVT1,
    [GPIO_MPU9000_INT_INDEX] = GPIO_MPU9000_INT_EVT1,
    [GPIO_USB_MUX_CB0_INDEX] = GPIO_USB_MUX_CB0_EVT1,
    [GPIO_USB_MUX_CB1_INDEX] = GPIO_USB_MUX_CB1,
    [GPIO_GPS_ON_OFF_INDEX] = GPIO_GPS_ON_OFF_EVT1,
    [GPIO_GPS_RESET_N_INDEX] = GPIO_GPS_RESET_N_EVT1,
    [GPIO_LCD_RST_N_INDEX] = GPIO_LCD_RST_N_EVT1,
    [GPIO_DISP_ENB_INDEX] = GPIO_DISP_ENB,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT1,
    [GPIO_CAM_PWDN_INDEX] = GPIO_CAM_PWDN_EVT1,
    [GPIO_TOUCHPAD_INT_N_INDEX] = GPIO_TOUCHPAD_INT_N_EVT1,
    [GPIO_PROX_INT_INDEX] = GPIO_PROX_INT_EVT1,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT1,
    [GPIO_BCM_BT_HOST_WAKE_INDEX] = GPIO_BCM_BT_HOST_WAKE_EVT1,
    [GPIO_BCM_WLAN_HOST_WAKE_INDEX] = GPIO_BCM_WLAN_HOST_WAKE_EVT,
};
static int notle_gpio_board_evt2[GPIO_MAX_INDEX] = {
    [GPIO_MPU9000_INT_INDEX] = GPIO_MPU9000_INT_EVT2,
    [GPIO_USB_MUX_CB0_INDEX] = GPIO_USB_MUX_CB0_EVT2,
    [GPIO_USB_MUX_CB1_INDEX] = GPIO_USB_MUX_CB1,
    [GPIO_GPS_ON_OFF_INDEX] = GPIO_GPS_ON_OFF_EVT2,
    [GPIO_GPS_RESET_N_INDEX] = GPIO_GPS_RESET_N_EVT2,
    [GPIO_LCD_RST_N_INDEX] = GPIO_LCD_RST_N_EVT2,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT2,
    [GPIO_CAM_PWDN_INDEX] = GPIO_CAM_PWDN_EVT2,
    [GPIO_TOUCHPAD_INT_N_INDEX] = GPIO_TOUCHPAD_INT_N_EVT2,
    [GPIO_PROX_INT_INDEX] = GPIO_PROX_INT_EVT2,
    [GPIO_BT_RST_N_INDEX] = GPIO_BT_RST_N_EVT2,
    [GPIO_BCM_BT_HOST_WAKE_INDEX] = GPIO_BCM_BT_HOST_WAKE_EVT2,
    [GPIO_BCM_WLAN_HOST_WAKE_INDEX] = GPIO_BCM_WLAN_HOST_WAKE_EVT,
};

/* Read board version from GPIO.  Result in NOTLE_VERSION. */
static void notle_version_init(void)
{
        int r;

        // mux board version gpio's
        // use low level interface since omap_mux_init has not been called yet
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID2);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID1);
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_ID0);

        r = gpio_request_one(GPIO_BOARD_ID2, GPIOF_IN, "id2");
        if (r) {
                pr_err("Failed to get gpio %d for id2 pin of board version\n", GPIO_BOARD_ID2);
        }

        r = gpio_request_one(GPIO_BOARD_ID1, GPIOF_IN, "id1");
        if (r) {
                pr_err("Failed to get gpio %d for id1 pin of board version\n", GPIO_BOARD_ID1);
        }

        r = gpio_request_one(GPIO_BOARD_ID0, GPIOF_IN, "id0");
        if (r) {
                pr_err("Failed to get gpio %d for id0 pin of board version\n", GPIO_BOARD_ID0);
        }
        NOTLE_VERSION = gpio_get_value(GPIO_BOARD_ID0) | (gpio_get_value(GPIO_BOARD_ID1) << 1)
            | (gpio_get_value(GPIO_BOARD_ID2) << 2);

        gpio_free(GPIO_BOARD_ID0);
        gpio_free(GPIO_BOARD_ID1);
        gpio_free(GPIO_BOARD_ID2);

}


static char * notle_version_str(notle_version board_ver)
{
        switch (board_ver)
        {
        case V1_HOG:
                return "V1 HOG";
        case V1_EVT1:
                return "V1 EVT1";
        case V1_EVT2:
                return "V1 EVT2";
        default:
                return "UNVERSIONED";
        }
        return "UNVERSIONED";
}

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= 40000,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= false,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 80,
		},
	},
	{
		.pcb_temp_level			= 45000,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 37,
		},
	},
	{
		.pcb_temp_level			= 50000,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
	{
		.pcb_temp_level			= 60000,
		.max_opp			= 800000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 800000,
			.cooling_rate		= 600000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 19,
		},
	},
	{
		.pcb_temp_level			= 65000,
		.max_opp			= 800000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 800000,
			.cooling_rate		= 600000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 14,
		},
	},
	{
		.pcb_temp_level			= 90000,
		.max_opp			= 600000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 600000,
			.cooling_rate		= 300000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 1,
		},
	},
};

void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
		ARRAY_SIZE(omap4_duty_governor_pcb_sections));
}
#else
void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

int
notle_get_gpio(int gpio_index)
{
    int ret = -1;

    switch (NOTLE_VERSION) {
    case V1_HOG:
        ret = notle_gpio_board_hog[gpio_index];
        break;
    case V1_EVT1:
        ret = notle_gpio_board_evt1[gpio_index];
        break;
    case V1_EVT2:
        ret = notle_gpio_board_evt2[gpio_index];
        break;
    default:
        pr_err("No get_gpio for Notle version: %s\n",
               notle_version_str(NOTLE_VERSION));
        break;
    }
    // Special case gpio_wk0
    if (ret == 0 && NOTLE_VERSION != V1_HOG &&
            gpio_index != GPIO_BCM_WLAN_HOST_WAKE_INDEX) {
        pr_err("%s:Uninitialized index %d\n", __FUNCTION__, gpio_index);
        ret = -1;
    }
    return(ret);
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
        init_duty_governor();
}

static int notle_enable_dpi(struct omap_dss_device *dssdev) {
        gpio_set_value(dssdev->reset_gpio, 1);
        return 0;
}

static void notle_disable_dpi(struct omap_dss_device *dssdev) {
        gpio_set_value(dssdev->reset_gpio, 0);
}

static int notle_enable_panel(void) {
        if (NOTLE_VERSION == V1_EVT1 || NOTLE_VERSION == V1_HOG) {
            gpio_set_value(GPIO_DISP_ENB, 1);
        }
        return 0;
};

static void notle_disable_panel(void) {
        if (NOTLE_VERSION == V1_EVT1 || NOTLE_VERSION == V1_HOG) {
            gpio_set_value(GPIO_DISP_ENB, 0);
        }
};

/* Using the panel-notle-dpi driver, we only specify enable/disable. */
static struct panel_notle_data panel_notle = {
        .platform_enable          = notle_enable_dpi,
        .platform_disable         = notle_disable_dpi,
        .panel_enable             = notle_enable_panel,
        .panel_disable            = notle_disable_panel,
        .red_max_mw               = 63,
        .green_max_mw             = 192,
        .blue_max_mw              = 96,
        .limit_mw                 = 80,
};


// gpio line set in board specific code
struct omap_dss_device panel_notle_device = {
        .type                     = OMAP_DISPLAY_TYPE_DPI,
        .name                     = "notle_nhd_panel",
        .driver_name              = "panel_notle",
        .data                     = &panel_notle,
        .phy.dpi.data_lines       = 24,
        .channel                  = OMAP_DSS_CHANNEL_LCD2,
        .vsync_gpio               = -1,
        .panel = {
                .timings = {
                        .x_res = 640,
                        .y_res = 360,
                },
        },
};

static struct omap2_mcspi_device_config ice40_mcspi_config = {
        .turbo_mode                = 0,
        .single_channel            = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info ice40_spi_board_info[] __initdata = {
        [0] = {
                .modalias                = "ice40-spi",
                .bus_num                 = 1,
                .chip_select             = 0,
                .max_speed_hz            = 48000000,
                .controller_data         = &ice40_mcspi_config,
        },
};

int __init notle_dpi_init(void)
{
        int r;

        panel_notle_device.reset_gpio = notle_get_gpio(GPIO_LCD_RST_N_INDEX);
        if (NOTLE_VERSION == V1_EVT1 || NOTLE_VERSION == V1_HOG) {
            r = gpio_request_one(GPIO_DISP_ENB, GPIOF_OUT_INIT_LOW, "disp_enable");
            if (r) {
                    pr_err("Failed to get display enable gpio\n");
                    return r;
            }
        }

        r = gpio_request_one(panel_notle_device.reset_gpio,
                             GPIOF_OUT_INIT_HIGH, "panel_reset");
        if (r) {
                pr_err("Failed to get panel reset powerdown GPIO\n");
                if (NOTLE_VERSION == V1_EVT1 || NOTLE_VERSION == V1_HOG) {
                    gpio_free(GPIO_DISP_ENB);
                }
                return r;
        }

        if (NOTLE_VERSION == V1_HOG || NOTLE_VERSION == V1_EVT1 ||
            NOTLE_VERSION == V1_EVT2) {
          spi_register_board_info(ice40_spi_board_info,
                                  ARRAY_SIZE(ice40_spi_board_info));
        }

        return 0;
}

static struct omap_dss_device *panel_notle_dss_devices[] = {
        &panel_notle_device,
};

static struct omap_dss_board_info panel_notle_dss_data = {
        .num_devices    = ARRAY_SIZE(panel_notle_dss_devices),
        .devices        = panel_notle_dss_devices,
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
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA |  MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
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

#ifdef CONFIG_MMC_NOTLE
static struct regulator_consumer_supply notle_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};
#endif
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

static struct regulator_init_data notle_vaux3 = {
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
/* gpio_100 camera power-down */
static struct regulator_init_data notle_vmmc = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
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
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled        = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(notle_vusb_supply),
	.consumer_supplies      = notle_vusb_supply,
};

static struct regulator_consumer_supply notle_clk32kg_supply[] = {
	REGULATOR_SUPPLY("clk32kg", NULL),
};

static struct regulator_init_data omap4_notle_clk32kg = {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on              = true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(notle_clk32kg_supply),
	.consumer_supplies	= notle_clk32kg_supply,
};

// TODO(eieio): revisit these when we optimize sleep current

/* ttyO0 unused */
static struct omap_device_pad notle_uart1_pads[] __initdata = {
	{
        // fails on first with repeat
		//.name	= "mcspi1_cs2.uart1_cts",
		.name	= ".uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rts.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_tx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rx.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

/* ttyO1 bluetooth */
static struct omap_device_pad notle_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

/* ttyO2 console port */
static struct omap_device_pad notle_uart3_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart3_cts_rctx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rts_sd.uart3_rts_sd",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

/* ttyO3 GPS */
static struct omap_device_pad notle_uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_uart_port_info omap_serial_port_info[] __initdata = {
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
                .wer = 0,
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
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
                .wer = 0,
        },
};

void __init notle_serial_init(void)
{
	omap_serial_init_port_pads(0, notle_uart1_pads,
		ARRAY_SIZE(notle_uart1_pads), &omap_serial_port_info[0]);
	omap_serial_init_port_pads(1, notle_uart2_pads,
		ARRAY_SIZE(notle_uart2_pads), &omap_serial_port_info[1]);
	omap_serial_init_port_pads(2, notle_uart3_pads,
		ARRAY_SIZE(notle_uart3_pads), &omap_serial_port_info[2]);
	omap_serial_init_port_pads(3, notle_uart4_pads,
		ARRAY_SIZE(notle_uart4_pads), &omap_serial_port_info[3]);
}

static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};


// Translate hardware buttons to keys -- we have only one.  Note that
static struct gpio_keys_button notle_button_table[] = {
    [0] = {
                .code   = KEY_CAMERA,           \
                .gpio   = GPIO_CAMERA,          \
                .desc   = "Camera",             \
                .type   = EV_KEY,               \
                .wakeup = 1,                    \
                .debounce_interval = 30,        \
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

static struct platform_device notle_pcb_temp_sensor = {
	.name = "notle_pcb_sensor",
};

static int notle_batt_table[] = {
        /* adc code for temperature in degree C */
        929, 925, /* -2 ,-1 */
        920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
        875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
        816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
        748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
        671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
        591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
        511, 504, 496 /* 60 - 62 */
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
	.clk32kg	= &omap4_notle_clk32kg,
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

#endif  /* CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C */

#ifdef CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C
static struct rmi_device_platform_data synaptics_platformdata = {
	.driver_name = "rmi-generic",

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
#endif  /* CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C */

#ifdef CONFIG_RMI4_BUS
struct notle_gpio_data_s {
	int gpio_num;
	const char *name;
};

// Need to dynamically set gpio_num based on board type
static struct notle_gpio_data_s notle_touchpad_gpio_data = {
	.name = "touchpad",
};

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval = 0;
	struct notle_gpio_data_s *data = gpio_data;

	if (configure) {
		/* Enable the interrupt */
		enable_irq(gpio_to_irq(data->gpio_num));
		printk(KERN_INFO "%s Callback to setup touchpad gpio %d %s\n",
		       __func__, data->gpio_num, data->name);
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		        __func__, data->gpio_num);
	}
	return retval;
}

static struct rmi_device_platform_data synaptics_platformdata = {
	.driver_name = "rmi_generic",
	.sensor_name = "tm2240",

	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.level_triggered = true,
	.gpio_data = &notle_touchpad_gpio_data,
	.gpio_config = synaptics_touchpad_gpio_setup,

	.reset_delay_ms = 100,

        /* function handler pdata */
        .power_management = {
	        .nosleep = RMI_F01_NOSLEEP_OFF,
	        .wakeup_threshold = 0,
	        .doze_holdoff = 0,
	        .doze_interval = 0,
	        .allow_sensor_to_wake = 1,
	},

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
		.delta_X = 5,
		.delta_Y = 2,
		.rel_report_enabled = 0,

#ifdef CONFIG_RMI4_DEBUG
		.debugfs_flip = NULL,
		.debugfs_clip = NULL,
		.debugfs_offset = NULL,
		.debugfs_swap = NULL,
		.reg_debug_addr = 0,
		.reg_debug_size = 0,
#endif
		},

        .f19_button_map = NULL,
        .f1a_button_map = NULL,
        .gpioled_map = NULL,
        .f11_button_map = NULL,
        .f41_button_map = NULL,

#ifdef CONFIG_RMI4_FWLIB
        .firmware_name = "firmware_name",
#endif
#ifdef CONFIG_RMI4_F11_TYPEB
        .f11_type_b = false;
#endif
#ifdef  CONFIG_PM
        .pm_data = NULL,
        .pre_suspend = NULL,
        .post_suspend = NULL,
        .pre_resume = NULL,
        .post_resume = NULL,
#endif
};

#endif  /* CONFIG_RMI4_BUS */


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

// attn_gpio_number assignment is board specific
static struct rmi_sensordata __initdata synaptics_sensordata = {
        .rmi_sensor_setup = 0,
        .rmi_sensor_teardown = 0,
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
#if defined(CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C) || defined(CONFIG_RMI4_BUS)
        {
                I2C_BOARD_INFO("rmi", 0x20),
                .platform_data = &synaptics_platformdata,
        },
#endif  /* CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C || CONFIG_RMI4_BUS */
#ifdef CONFIG_INPUT_TOUCHPAD_FTK
        {
                I2C_BOARD_INFO("ftk", 0x4b),
                .platform_data = &ftk_platformdata,
        },
#endif  /* CONFIG_TOUCHPAD_FTK */
};

/*
 * i2c-4
 */
/* MPU */
static struct mpu_platform_data mpu9150_notle_data = {
        .int_config     = 0x10,
        .orientation    = { 0, 1, 0,
                            0, 0, 1,
                            1, 0, 0 },
        .level_shifter  = 1,
        .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
        .sec_slave_id   = COMPASS_ID_AK8975,
        .secondary_i2c_addr = 0x0C,
        .key = {221, 22, 205, 7, 217, 186, 151, 55,
            206, 254, 35, 144, 225, 102, 47, 50},
        .secondary_orientation = { 0, 0, 1,
                                   1, 0, 0,
                                   0, 1, 0 },
};

// gpio_int_no is board specific
#ifdef CONFIG_INPUT_LTR506ALS
static struct ltr506_platform_data notle_ltr506als_data = {
	/* Boolean to allow interrupt to wake device or not */
	.pfd_gpio_int_wake_dev = 0,

	/* Disable als on suspend flag */
	.pfd_disable_als_on_suspend = 0,

	/* ALS enable filtering interrupts
	 * by suppressing interrupts when measured value
	 * falls within some driver calculated threshold.  This
	 * can be manually set by setting a '0' here and writing
	 * to the sysfs threshold file.
	 * e.g.
	 *  echo "1000 1020" > /sys/class/input/input7/als_threshold
	 */
	.pfd_als_filter_interrupts = 0,

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
	.pfd_als_gain = 2,

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
	.pfd_ps_filter_interrupts = 0,

	/* PS measurement repeate rate
	 * '000:   12.5ms (ALS auto-disabled)
	 * '001:   50ms
	 * '010:   70ms
	 * '011:  100ms
	 * '100:  200ms
	 * '101:  500ms
	 * '110: 1000ms
	 * '111: 2000ms */
	.pfd_ps_meas_rate = 1,

	/* PS gain. NOTE(CMM) This is must write to '3'
	 * '00:  x8 gain
	 * '01: x16 gain
	 * '10: x32 gain
	 * '11: x64 gain ## */
	.pfd_ps_gain = 3,

	/* LED pulse frequency.
	 * '000: 30kHz
	 * '001: 40kHz
	 * '010: 50kHz
	 * '011: 60kHz
	 * '100: 70kHz
	 * '101: 80kHz
	 * '110: 90kHz
	 * '111: 100kHz */
	.pfd_led_pulse_freq = 3,

	/* LED Duty cycle. NOTE(CMM) This is must write to '1'
	 * '00:  25%
	 * '01:  50% ##
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


#ifdef CONFIG_INPUT_SI114X
// gpio interrupt line is board specific
static struct si114x_platform_data notle_si114x_data = {
	/* TODO(cmanton) Interrupts are not built into the driver yet */

	/* Rate of device timer waking itself up.
	 * 0x00: Never
	 * 0x84: Every 10ms
	 * 0x94: Every 20ms
	 * 0xb9: Every 100ms
	 * 0xdf: Every 496ms
	 * 0xff: Every 1984ms
	 */
	.pfd_meas_rate = SI114X_MEAS_RATE_10ms,

	/* Rate of ALS taking measurement every time device wakes up.
	 * 0x00: Never
	 * 0x08: Every time device awakens.
	 * 0x32: Every 10 times device awakens.
	 * 0x69: Every 100 times device awakens.
	 */
	.pfd_als_rate = SI114X_ALS_RATE_1x,

	/* Rate of PS taking measurement every time device wakes up.
	 * 0x00: Never
	 * 0x08: Every time device awakens.
	 * 0x32: Every 10 times device awakens.
	 * 0x69: Every 100 times device awakens.
	 */
        .pfd_ps_rate = SI114X_PS_RATE_1x,

	/* LED intensities */
        .pfd_ps_led1 = SI114X_LED_90,
        .pfd_ps_led2 = SI114X_LED_90,
        .pfd_ps_led3 = SI114X_LED_90,

        /* input subsystem poll interval in ms when using polling */
        .pfd_als_poll_interval = 1000,
        .pfd_ps_poll_interval = 12,
};
#endif  /* CONFIG_INPUT_SI114X */

#ifdef CONFIG_INPUT_GLASSHUB
static struct glasshub_platform_data notle_glasshub_data;
#endif  /* CONFIG_INPUT_GLASSHUB */

static struct i2c_board_info __initdata notle_i2c_4_boardinfo[] = {
        {
                I2C_BOARD_INFO("panel-notle-panel", 0x49),
        },
        {
                I2C_BOARD_INFO("mpu9150", 0x68),
                .platform_data = &mpu9150_notle_data,
        },
#ifdef CONFIG_INPUT_LTR506ALS
        {
                I2C_BOARD_INFO("ltr506als", 0x3a),
                .flags = I2C_CLIENT_WAKE,
                .platform_data = &notle_ltr506als_data,
        },
#endif
#ifdef CONFIG_INPUT_SI114X
        {
                I2C_BOARD_INFO("si114x", 0x5a),
                .platform_data = &notle_si114x_data,
        },
#endif

/* dls: slave address of 0x35 is chosen not to conflict and
 * easy to see on i2c bus. Can be changed by modifying the
 * code in the Glass hub MCU.
 */
#ifdef CONFIG_INPUT_GLASSHUB
        {
                I2C_BOARD_INFO("glasshub", 0x35),
                .platform_data = &notle_glasshub_data,
        },
#endif
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

static void __init notle_i2c_irq_fixup(void)
{
    int i;
    int gpio_mpu, gpio_prox, gpio_touchpad;
    struct i2c_board_info *pinfo;

    gpio_prox = notle_get_gpio(GPIO_PROX_INT_INDEX);
    gpio_mpu = notle_get_gpio(GPIO_MPU9000_INT_INDEX);
    gpio_touchpad = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);

    // Fix up the global device data structures
#ifdef CONFIG_TOUCHPAD_SYNAPTICS_RMI4_I2C
    synaptics_platformdata.irq = gpio_touchpad;
#endif

#ifdef CONFIG_RMI4_BUS
    synaptics_platformdata.attn_gpio = gpio_touchpad;
#endif

#ifdef CONFIG_INPUT_LTR506ALS
    notle_ltr506als_data.pfd_gpio_int_no = gpio_prox;
#endif

#ifdef CONFIG_INPUT_SI114X
    notle_si114x_data.pfd_gpio_int_no = gpio_prox;
#endif

#ifdef CONFIG_INPUT_GLASSHUB
notle_glasshub_data.gpio_int_no = gpio_prox;
#endif

#ifdef CONFIG_RMI4_BUS
    notle_touchpad_gpio_data.gpio_num = gpio_touchpad;
#endif

    // Now fixup the irqs set in the various 2c boardinfo structs
    pinfo = notle_i2c_4_boardinfo;
    for (i = 0; i < ARRAY_SIZE(notle_i2c_4_boardinfo); i++) {
        if (!strcmp("mpu9150", pinfo->type)  || !strcmp("ak8975", pinfo->type)) {
            pinfo->irq = OMAP_GPIO_IRQ(gpio_mpu);
        }
        if (!strcmp("ltr506als", pinfo->type) || !strcmp("glasshub", pinfo->type)) {
            pinfo->irq = OMAP_GPIO_IRQ(gpio_prox);
        }
        pinfo++;
    }
}

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
          case V1_HOG:
          case V1_EVT1:
          case V1_EVT2:
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
            synaptics_f11_data.flip_X = false;
            synaptics_f11_data.flip_Y = true;
            synaptics_f11_data.swap_axes = false;
            synaptics_sensordata.attn_gpio_number = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);
#endif  /* CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C */
            omap4_pmic_init("twl6030", &notle_twldata);
            notle_i2c_irq_fixup();
            omap_register_i2c_bus(2, 400, NULL, 0);
            omap_register_i2c_bus(3, 400, notle_i2c_3_boardinfo,
                            ARRAY_SIZE(notle_i2c_3_boardinfo));
            omap_register_i2c_bus(4, 400, notle_i2c_4_boardinfo,
                            ARRAY_SIZE(notle_i2c_4_boardinfo));
            break;
          default:
            pr_err("Unrecognized Notle version: %i\n", NOTLE_VERSION);
            return -1;
            break;
        }
        return 0;
}

#ifdef CONFIG_OMAP_MUX
// Board specific MUX settings
// HOG Core:
static struct omap_board_mux hog_board_mux[] __initdata = {
    OMAP4_MUX(GPMC_AD12,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_BT_WAKE
    OMAP4_MUX(GPMC_A19,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_RST_N
    OMAP4_MUX(GPMC_A20,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB0
    OMAP4_MUX(GPMC_A21,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB1
    OMAP4_MUX(GPMC_A24,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_BT_REG_ON
    OMAP4_MUX(GPMC_A25,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_ON_OFF
    OMAP4_MUX(GPMC_NCS2,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_RESET_N
    OMAP4_MUX(GPMC_NCS3,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // LCD_RST_N
    OMAP4_MUX(USBB1_ULPITLL_CLK,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // DISP_ENB
    OMAP4_MUX(USBB1_HSIC_STROBE,    OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),  // BCM_WLAN_HOST_WAKE_HOG
    OMAP4_MUX(USBB2_HSIC_STROBE,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_WLAN_WAKE
    OMAP4_MUX(MCSPI4_CLK,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BT_RST_N
    OMAP4_MUX(MCSPI4_CS0,           OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // BCM_BT_HOST_WAKE
    OMAP4_MUX(USBB1_ULPITLL_DAT3,   OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // CAM_PWDN
    OMAP4_MUX(USBB1_ULPITLL_DAT5,   OMAP_MUX_MODE7 | OMAP_PULL_ENA),    // BACKLIGHT XXX Remove? NC on EVT1
    OMAP4_MUX(ABE_DMIC_DIN2,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // CAMERA, TOP_SW
    OMAP4_MUX(GPMC_AD8,             OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // TOUCHPAD_INT_N
    OMAP4_MUX(USBB1_ULPITLL_DAT2,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // PROX_INT
    OMAP4_MUX(USBB1_ULPITLL_DAT7,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT
    OMAP4_MUX(USBB1_ULPITLL_DAT4,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT_TIMER
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
// HOG WakeUp:
static struct omap_board_mux hog_board_wkup_mux[] __initdata = {
    OMAP4_MUX(FREF_CLK4_REQ, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GREEN_LED
    OMAP4_MUX(FREF_CLK4_OUT, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // YELLOW_LED
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

// EVT1 Core:
static struct omap_board_mux evt1_board_mux[] __initdata = {
    OMAP4_MUX(GPMC_AD12,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_BT_WAKE
    OMAP4_MUX(GPMC_A19,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_RST_N
    OMAP4_MUX(GPMC_A20,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB0
    OMAP4_MUX(GPMC_A21,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB1
    OMAP4_MUX(GPMC_A24,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_BT_REG_ON
    OMAP4_MUX(GPMC_A25,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_ON_OFF
    OMAP4_MUX(GPMC_NCS2,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_RESET_N
    OMAP4_MUX(GPMC_NCS3,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // LCD_RST_N
    OMAP4_MUX(USBB1_ULPITLL_CLK,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // DISP_ENB
    OMAP4_MUX(USBB2_HSIC_STROBE,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_WLAN_WAKE
    OMAP4_MUX(MCSPI4_CLK,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BT_RST_N
    OMAP4_MUX(MCSPI4_CS0,           OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // BCM_BT_HOST_WAKE
    OMAP4_MUX(USBB1_ULPITLL_DAT3,   OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // CAM_PWDN
    OMAP4_MUX(USBB1_ULPITLL_DAT5,   OMAP_MUX_MODE7 | OMAP_PULL_ENA),    // BACKLIGHT XXX Remove? NC on EVT1
    OMAP4_MUX(ABE_DMIC_DIN2,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // CAMERA, TOP_SW
    OMAP4_MUX(GPMC_AD8,             OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // TOUCHPAD_INT_N
    OMAP4_MUX(USBB1_ULPITLL_DAT2,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // PROX_INT
    OMAP4_MUX(USBB1_ULPITLL_DAT7,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT
    OMAP4_MUX(USBB1_ULPITLL_DAT4,   OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT_TIMER
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
// EVT1 WakeUp:
static struct omap_board_mux evt1_board_wkup_mux[] __initdata = {
    OMAP4_MUX(SIM_IO,               OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),  // BCM_WLAN_HOST_WAKE
    OMAP4_MUX(FREF_CLK4_REQ,        OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GREEN_LED
    OMAP4_MUX(FREF_CLK4_OUT,        OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // YELLOW_LED
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

// EVT2 Core:
static struct omap_board_mux evt2_board_mux[] __initdata = {
    OMAP4_MUX(GPMC_AD12,            OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_BT_WAKE
    OMAP4_MUX(GPMC_A19,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_RST_N
    OMAP4_MUX(GPMC_A22,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB0
    OMAP4_MUX(GPMC_A21,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // USB_MUX_CB1
    OMAP4_MUX(GPMC_A24,             OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // WL_BT_REG_ON
    OMAP4_MUX(MCSPI1_CS2,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_ON_OFF
    OMAP4_MUX(MCSPI1_CS3,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // GPS_RESET_N
    OMAP4_MUX(USBB1_ULPITLL_DAT6,   OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // LCD_RST_N
    OMAP4_MUX(USBB2_HSIC_STROBE,    OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BCM_WLAN_WAKE
    OMAP4_MUX(MCSPI4_CS0,           OMAP_MUX_MODE3 | OMAP_PIN_INPUT),   // BCM_BT_HOST_WAKE
    OMAP4_MUX(ABE_MCBSP2_FSX,       OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // BT_RST_N
    OMAP4_MUX(SDMMC1_CLK,           OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),  // CAM_PWDN
    OMAP4_MUX(ABE_DMIC_DIN2,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // CAMERA, TOP_SW
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
// EVT2 WakeUp:
static struct omap_board_mux evt2_board_wkup_mux[] __initdata = {
    OMAP4_MUX(SIM_IO,               OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),  // BCM_WLAN_HOST_WAKE
    OMAP4_MUX(FREF_CLK3_REQ,        OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // CAMERA, TOP_SW
    OMAP4_MUX(SIM_CD,               OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN),    // TOUCHPAD_INT_N
    OMAP4_MUX(SIM_CLK,              OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),                     // PROX_INT
    OMAP4_MUX(SIM_PWRCTRL,          OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),           // MPU9000_INT_TIMER
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define emtpy_board_mux	NULL
#endif

static int omap_audio_init(void) {
        int r;
        u32 omap4430_cm_clksel_dpll_abe_register;
        int audio_power_on_gpio = GPIO_AUDIO_POWERON;
        int gpio_audio_headset;

        gpio_audio_headset = notle_get_gpio(GPIO_USB_MUX_CB0_INDEX);
        twl6040_codec.audpwron_gpio = audio_power_on_gpio;

        /* GPIO that enables a MUX outside omap to use usb headset */
        r = gpio_request_one(gpio_audio_headset, GPIOF_OUT_INIT_LOW,
                "gpio_audio_headset");
        if (r) {
                pr_err("Failed to get audio_headset gpio_%d\n", gpio_audio_headset);
                goto error;
        }

        /* TODO(petermalkin): remove this line for the product compile. */
        /* Do not expose GPIO to /sys filesystem for security purposes. */
        r = gpio_export(gpio_audio_headset, false);
        if (r) {
                pr_err("Unable to export audio_headset gpio_%d\n", gpio_audio_headset);
        }

        /* GPIO 45 needs export as per Russ request */
        r = gpio_request_one(GPIO_USB_MUX_CB1, GPIOF_OUT_INIT_HIGH,
                "gpio_45");
        if (r) {
                pr_err("Failed to get gpio_%d\n", GPIO_USB_MUX_CB1);
                goto error;
        }

        /* TODO(petermalkin): remove this line for the product compile. */
        /* Do not expose GPIO to /sys filesystem for security purposes. */
        r = gpio_export(GPIO_USB_MUX_CB1, false);
        if (r) {
                pr_err("Unable to export gpio_%d\n", GPIO_USB_MUX_CB1);
        }

        omap_mux_init_signal("sys_nirq2.sys_nirq2", \
                OMAP_PIN_INPUT_PULLUP);
        return 0;

error:
        return r;
}

static int notle_gps_init(void) {
	int r;
    int gpio_gps_reset, gpio_gps_on_off;

	/* Configuration of requested GPIO lines */
    gpio_gps_reset = notle_get_gpio(GPIO_GPS_RESET_N_INDEX);
    gpio_gps_on_off = notle_get_gpio(GPIO_GPS_ON_OFF_INDEX);

        r = gpio_request_one(gpio_gps_reset, GPIOF_OUT_INIT_HIGH,
                "gps_reset_n");
        if (r) {
                pr_err("Failed to get gps_reset_n gpio\n");
                goto error;
        }

        r = gpio_export(gpio_gps_reset, false);
        if (r) {
                pr_err("Unable to export gps_reset_n gpio\n");
        }

        r = gpio_sysfs_set_active_low(gpio_gps_reset, 0);
        if (r) {
                pr_err("Unable to set sysfs gps_reset_n active low\n");
        }

        /* Need a rising edge to turn device on. */
        r = gpio_request_one(gpio_gps_on_off, GPIOF_OUT_INIT_LOW,
                "gps_on_off");
        if (r) {
                pr_err("Failed to get gps_on_off gpio\n");
                goto error;
        }

        r = gpio_export(gpio_gps_on_off, false);
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
        int gpio_touchpad_int;

        pr_info("%s()+\n", __func__);

        /* Configuration of requested GPIO line */
        gpio_touchpad_int = notle_get_gpio(GPIO_TOUCHPAD_INT_N_INDEX);

        r = gpio_request_one(gpio_touchpad_int, GPIOF_IN, "touchpad_int_n");
        if (r) {
                pr_err("Failed to get touchpad_int_n gpio\n");
        }
        /* Allow this interrupt to wake the system */
        r = irq_set_irq_wake(gpio_to_irq(gpio_touchpad_int), 1);
        if (r) {
                pr_err("%s Unable to set irq to wake device\n", __FUNCTION__);
        }
        return r;
}

static int __init notle_imu_init(void) {
        int r;
        int gpio_mpu9000_int_timer, gpio_mpu9000_int;

        pr_info("%s()+\n", __func__);
        gpio_mpu9000_int = notle_get_gpio(GPIO_MPU9000_INT_INDEX);

        /* Configuration of requested GPIO line */

        if (NOTLE_VERSION == V1_EVT1 || NOTLE_VERSION == V1_HOG) {
            gpio_mpu9000_int_timer = notle_get_gpio(GPIO_MPU9000_INT_TIMER_INDEX);
            r = gpio_request_one(gpio_mpu9000_int_timer, GPIOF_IN, "mpuirq_timer");
            if (r) {
                    pr_err("Failed to get mpu9000_int_timer gpio\n");
            } else {
                    pr_err("got the mpu9000 timer gpio!!!\n");
            }
        }

        r = gpio_request(gpio_mpu9000_int, "mpuirq");
        if (r) {
                pr_err("Failed to get mpu9000_int gpio\n");
        }
        r = gpio_direction_input(gpio_mpu9000_int);
        if (r) {
                pr_err("Failed to get mpu9000_int gpio\n");
        }
        /* Allow this interrupt to wake the system */
        r = irq_set_irq_wake(gpio_to_irq(gpio_mpu9000_int), 1);
        if (r) {
                pr_err("%s Unable to set irq to wake device\n", __FUNCTION__);
        }
        return r;
}

#ifdef CONFIG_INPUT_GLASSHUB
static int __init notle_glasshub_init(void) {
        int r;

        pr_info("%s()+\n", __func__);

		notle_glasshub_data.irq = gpio_to_irq(notle_glasshub_data.gpio_int_no);

        /* Configuration of requested GPIO line */
        r = gpio_request_one(notle_glasshub_data.gpio_int_no, GPIOF_IN, "glasshub_int");
        if (r) {
                pr_err("Failed to get glasshub gpio\n");
        }

        return r;
}
#endif

#define TWL6030_SW_RESET_BIT_MASK       (1<<6)

static void notle_pm_restart(char str, const char *cmd)
{
	bool cold_reset = true;
	bool valid_reason = false;
	void __iomem *sar_base;

	sar_base = omap4_get_sar_ram_base();
	if (!sar_base)
		printk("Failed to get scratch memory base!\n");

	/* determine reset conditions */
	if (!cmd || !sar_base) {
		cold_reset = true;
		valid_reason = false;
	} else if (!strcmp(cmd, "bootloader")) {
		cold_reset = false;
		valid_reason = true;
	} else if (!strcmp(cmd, "recovery")) {
		cold_reset = false;
		valid_reason = true;
	} else if (!strcmp(cmd, "charger")) {
		cold_reset = false;
		valid_reason = true;
	} else {
		cold_reset = true;
		valid_reason = false;
	}

	/* record reset reason */
	if (sar_base && valid_reason)
		strcpy(sar_base + OMAP_REBOOT_REASON_OFFSET, cmd);

	/* choose reset path */
	if (cold_reset)
		twl_i2c_write_u8(TWL_MODULE_PM_MASTER, TWL6030_SW_RESET_BIT_MASK,
				TWL6030_PHOENIX_DEV_ON);
	else
		arm_machine_restart(str, cmd);
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
        u32 omap_reg;

        omap_emif_setup_device_details(&emif_devices, &emif_devices);

        if (omap_rev() == OMAP4430_REV_ES1_0)
                package = OMAP_PACKAGE_CBL;
        notle_version_init();
        switch (NOTLE_VERSION) {
        case V1_HOG:
            omap4_mux_init(hog_board_mux, hog_board_wkup_mux, package);
            break;

        case V1_EVT1:
            omap4_mux_init(evt1_board_mux, evt1_board_wkup_mux, package);

            // Additional mux/pad settings

            // The GPIOWK_IO_PWRDNZ bit needs to be set after muxing
            //and before you set it to input
            omap4_ctrl_wk_pad_writel(OMAP4_USIM_PWRDNZ_MASK,
                    OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);
            break;

        case V1_EVT2:
            omap4_mux_init(evt2_board_mux, evt2_board_wkup_mux, package);

            // Additional mux/pad settings

            // The GPIOWK_IO_PWRDNZ bit needs to be set after muxing
            //and before you set it to input
            omap4_ctrl_wk_pad_writel(OMAP4_USIM_PWRDNZ_MASK,
                    OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);

            // Camera power down on gpio_100 needs the correct voltage of 1.8V
            // wk1 needs correct magic bias settings
            omap_reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
            omap_reg |= OMAP4_MMC1_PBIASLITE_PWRDNZ_MASK | OMAP4_MMC1_PWRDNZ_MASK;
            omap_reg &= ~OMAP4_MMC1_PBIASLITE_VMODE_MASK;
            omap_reg |= OMAP4_USIM_PBIASLITE_PWRDNZ_MASK;
            omap_reg &= ~OMAP4_USIM_PBIASLITE_VMODE_MASK;
            omap4_ctrl_pad_writel(omap_reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
            break;

        default:
            pr_err("No mux init for Notle version: %s\n",
                   notle_version_str(NOTLE_VERSION));
            break;
        }
        notle_pmic_mux_init();

        printk("Notle board revision: %s(%d)", notle_version_str(NOTLE_VERSION), NOTLE_VERSION);

        err = omap_audio_init();
        if (err) {
                pr_err("Audio initialization failed: %d\n", err);
        }

        arm_pm_restart = notle_pm_restart;

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

        notle_serial_init();

        err = notle_wlan_init();
        if (err) {
                pr_err("Wifi initialization failed: %d\n", err);
        }

        omap4_twl6030_hsmmc_init(mmc);
        usb_musb_init(&musb_board_data);
        omap_dmm_init();

        omapfb_set_platform_data(&notle_fb_pdata);

        err = notle_gps_init();
        if (err) {
                pr_err("GPS initialization failed: %d\n", err);
        }

        // Do this after the wlan_init, which inits the regulator shared
        // with the bluetooth device and muxes the bt signals.
        platform_add_devices(notle_devices, ARRAY_SIZE(notle_devices));

        err = platform_device_register(&notle_pcb_temp_sensor);
        if (err) {
            pr_err("notle_pcb_temp_sensor registration failed: %d\n", err);
        }

        err = notle_touchpad_init();
        if (err) {
                pr_err("Touchpad initialization failed: %d\n", err);
        }

#ifdef CONFIG_INPUT_GLASSHUB
        err = notle_glasshub_init();
        if (err) {
                pr_err("Glass hub initialization failed: %d\n", err);
        }
#endif

        err = notle_dpi_init();
        if (!err) {
            panel_notle.notle_version = NOTLE_VERSION;
            omap_display_init(&panel_notle_dss_data);
        } else {
            pr_err("DPI initialization failed: %d\n", err);
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
	omap_init_ram_size();

#ifdef CONFIG_ION_OMAP
        omap_android_display_setup(&panel_notle_dss_data,
                                   NULL,
                                   NULL,
                                   &notle_fb_pdata,
                                   get_omap_ion_platform_data());
        omap_ion_init();
#else
        omap_android_display_setup(&panel_notle_dss_data,
                                   NULL,
                                   NULL,
                                   &notle_fb_pdata,
                                   NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

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
