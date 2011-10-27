/*
 * Copyright (C) 2011 Google, Inc.
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

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/if.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/random.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <plat/mmc.h>

#include "hsmmc.h"
#include "control.h"
#include "board-notle.h"

// NOTE(abliss): This file was ripped off of board-tuna-wifi.  I have kept the
// tuna name (and tuna's distinct gpio terminology) in many places, in order to
// simplify merges of future tuna improvements into this file.

#define GPIO_WLAN_PMENA		GPIO_WL_BT_REG_ON
#define GPIO_WLAN_IRQ		GPIO_BCM_WLAN_HOST_WAKE


// NOTE(abliss): These numbers were taken from tuna and I have no idea why they
// are what they are.
#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *tuna_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init tuna_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static struct resource tuna_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
                    // NOTE(abliss): interrupts not currently used; driver polls instead.
		.start		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.end		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};


static int tuna_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int tuna_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int tuna_wifi_status(struct device *dev)
{
	return tuna_wifi_cd;
}

struct mmc_platform_data tuna_wifi_data = {
        .ocr_mask		= MMC_VDD_165_195 | MMC_VDD_20_21,
	.built_in		= 1,
	.status			= tuna_wifi_status,
	.card_present		= 1,
	.register_status_notify	= tuna_wifi_status_register,
};

static int tuna_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	tuna_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int tuna_wifi_power_state;

struct fixed_voltage_data {
	struct regulator_desc desc;
	struct regulator_dev *dev;
	int microvolts;
	int gpio;
	unsigned startup_delay;
	bool enable_high;
	bool is_enabled;
};

static struct regulator_consumer_supply tuna_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data tuna_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &tuna_vmmc5_supply,
};

static struct fixed_voltage_config tuna_vwlan = {
	.supply_name = "vbcm4329",
	.microvolts = 2000000, /* 2.0V */
	.gpio = GPIO_WLAN_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &tuna_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &tuna_vwlan,
	},
};

static int tuna_wifi_power(int on)
{
        pr_info("%s: %d\n", __func__, on);
	mdelay(100);
	gpio_set_value(GPIO_WLAN_PMENA, on);
	mdelay(200);

	tuna_wifi_power_state = on;
	return 0;
}

static int tuna_wifi_reset_state;

static int tuna_wifi_reset(int on)
{
        pr_info("%s: %d\n", __func__, on);
	tuna_wifi_reset_state = on;
	mdelay(100);
        gpio_set_value(GPIO_WL_RST_N, on ? 0 : 1);
	mdelay(200);
	return 0;
}

static unsigned char tuna_mac_addr[IFHWADDRLEN] = { 0,0x90,0x4c,0,0,0 };

static int __init tuna_mac_addr_setup(char *str)
{
	char macstr[IFHWADDRLEN*3];
	char *macptr = macstr;
	char *token;
	int i = 0;

	if (!str)
		return 0;
	pr_err("wlan MAC = %s\n", str);
	if (strlen(str) >= sizeof(macstr))
		return 0;
	strcpy(macstr, str);

	while ((token = strsep(&macptr, ":")) != NULL) {
		unsigned long val;
		int res;

		if (i >= IFHWADDRLEN)
			break;
		res = strict_strtoul(token, 0x10, &val);
		if (res < 0)
			return 0;
		tuna_mac_addr[i++] = (u8)val;
	}

	return 1;
}

__setup("androidboot.macaddr=", tuna_mac_addr_setup);

static int tuna_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;
	if (!buf)
		return -EFAULT;

	if ((tuna_mac_addr[4] == 0) && (tuna_mac_addr[5] == 0)) {
		srandom32((uint)jiffies);
		rand_mac = random32();
		tuna_mac_addr[3] = (unsigned char)rand_mac;
		tuna_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		tuna_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, tuna_mac_addr, IFHWADDRLEN);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t tuna_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XY", 4},  /* universal */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},  /* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},  /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void *tuna_wifi_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(tuna_wifi_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode, tuna_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return &tuna_wifi_translate_custom_table[i];
	return &tuna_wifi_translate_custom_table[0];
}

static struct wifi_platform_data tuna_wifi_control = {
	.set_power      = tuna_wifi_power,
	.set_reset      = tuna_wifi_reset,
	.set_carddetect = tuna_wifi_set_carddetect,
	.mem_prealloc	= tuna_wifi_mem_prealloc,
	.get_mac_addr	= tuna_wifi_get_mac_addr,
	.get_country_code = tuna_wifi_get_country_code,
};

static struct platform_device tuna_wifi_device = {
        .name           = "bcm4329_wlan",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(tuna_wifi_resources),
        .resource       = tuna_wifi_resources,
        .dev            = {
                .platform_data = &tuna_wifi_control,
        },
};


static int __init notle_wlan_gpio(void) {
        int r;

        if (!machine_is_notle())
                return 0;

        pr_info("%s()+\n", __func__);

        /* Configuration of requested GPIO lines */
        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN,
                CORE_BASE_ADDR + MUX_BCM_WLAN_HOST_WAKE);
        r = gpio_request_one(GPIO_BCM_WLAN_HOST_WAKE, GPIOF_IN, "wlan_irq");
        if (r) {
                pr_err("Failed to get wlan_irq gpio\n");
                goto error;
        }

        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_BCM_WLAN_WAKE);
        r = gpio_request_one(GPIO_BCM_WLAN_WAKE, GPIOF_OUT_INIT_HIGH, "wlan_wake");
        if (r) {
                pr_err("Failed to get wlan_wake gpio\n");
                goto error1;
        }

        __raw_writew(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP,
                CORE_BASE_ADDR + MUX_BCM_BT_HOST_WAKE);
        r = gpio_request_one(GPIO_BCM_BT_HOST_WAKE, GPIOF_IN, "bt_irq");
        if (r) {
                pr_err("Failed to get bt_irq gpio\n");
                goto error2;
        }

        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_BCM_BT_WAKE);
        r = gpio_request_one(GPIO_BCM_BT_WAKE, GPIOF_OUT_INIT_HIGH, "bt_wake");
        if (r) {
                pr_err("Failed to get bt_wake gpio\n");
                goto error3;
        }

        __raw_writew(OMAP_MUX_MODE3, CORE_BASE_ADDR + MUX_WL_RST_N);
        r = gpio_request_one(GPIO_WL_RST_N, GPIOF_OUT_INIT_LOW, "wlan_reset");
        if (r) {
                pr_err("Failed to get wlan_reset gpio\n");
                goto error4;
        }
        gpio_set_value(GPIO_WL_RST_N, 1);

        pr_info("%s()-: 0\n", __func__);
        return 0;

error4:
        gpio_free(GPIO_BCM_BT_WAKE);
error3:
        gpio_free(GPIO_BCM_BT_HOST_WAKE);
error2:
        gpio_free(GPIO_BCM_WLAN_WAKE);
error1:
        gpio_free(GPIO_BCM_WLAN_HOST_WAKE);
error:
        pr_info("%s()-: %i\n", __func__, r);
        return r;
}

int __init notle_wlan_init(void)
{
	notle_wlan_gpio();
	tuna_init_wifi_mem();
	platform_device_register(&omap_vwlan_device);
	return platform_device_register(&tuna_wifi_device);
}
