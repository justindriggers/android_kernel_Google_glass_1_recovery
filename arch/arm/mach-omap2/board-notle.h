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

// Right now this file exists to coordinate between board-notle.c and
// board-notle-wifi.c

#ifndef _MACH_OMAP2_BOARD_NOTLE_H_
#define _MACH_OMAP2_BOARD_NOTLE_H_

#include "mux.h"

#define MUX(x) OMAP4_CTRL_MODULE_PAD_##x##_OFFSET

#define CORE_BASE_ADDR 0xfc100000
// Choose your board revision.
//#define NOTLE_VERSION_1
#define NOTLE_VERSION_2
//#define NOTLE_VERSION_3

#ifdef NOTLE_VERSION_1
#define GPIO_BCM_WLAN_HOST_WAKE         86
#define MUX_BCM_WLAN_HOST_WAKE          MUX(USBB1_ULPITLL_DIR)
#define GPIO_BCM_BT_HOST_WAKE           87
#define MUX_BCM_BT_HOST_WAKE            MUX(USBB1_ULPITLL_NXT)
#define GPIO_BCM_WLAN_WAKE              88
#define MUX_BCM_WLAN_WAKE               MUX(USBB1_ULPITLL_DAT0)
#define GPIO_BCM_BT_WAKE                91
#define MUX_BCM_BT_WAKE                 MUX(USBB1_ULPITLL_DAT3)
#endif // NOTLE_VERSION_1

#ifdef NOTLE_VERSION_2
// TODO: Figure out why the wlan_wake and wlan_host_wake gpio's
// are swapped compared to what the hardware docs imply them
// to be.
#define GPIO_BCM_WLAN_HOST_WAKE         97
#define MUX_BCM_WLAN_HOST_WAKE          MUX(USBB1_HSIC_STROBE)
#define GPIO_BCM_BT_HOST_WAKE           154
#define MUX_BCM_BT_HOST_WAKE            MUX(MCSPI4_CS0)
#define GPIO_BCM_WLAN_WAKE              170
#define MUX_BCM_WLAN_WAKE               MUX(USBB2_HSIC_STROBE)
#define GPIO_BCM_BT_WAKE                36
#define MUX_BCM_BT_WAKE                 MUX(GPMC_AD12)
#endif // NOTLE_VERSION_2

#define GPIO_WL_RST_N                   43
#define MUX_WL_RST_N                    MUX(GPMC_A19)
#define GPIO_BT_RST_N                   151
#define MUX_BT_RST_N                    MUX(MCSPI4_CLK)
#define GPIO_WL_BT_REG_ON               48
#define MUX_WL_BT_REG_ON                MUX(GPMC_A24)

extern struct mmc_platform_data tuna_wifi_data;
int notle_wlan_init(void);

#endif // _MACH_OMAP2_BOARD_NOTLE_H_
