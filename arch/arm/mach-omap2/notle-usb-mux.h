/*
 * arch/arm/mach-omap2/notle-usb-mux.h
 *
 * Notle board USB MUX control platform data.
 *
 * Copyright (c) 2013 Google, Inc.
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

#ifndef __ARCH_ARM_MACH_OMAP2_NOTLE_USB_MUX_H
#define __ARCH_ARM_MACH_OMAP2_NOTLE_USB_MUX_H

struct usb_mux_platform_data {
	int gpio_cb0;
	int gpio_cb0_flags;
	const char *gpio_cb0_label;

	int gpio_cb1;
	int gpio_cb1_flags;
	const char *gpio_cb1_label;
};

#endif

