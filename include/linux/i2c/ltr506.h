// Copyright 2011 Google Inc. All Rights Reserved.
// Author: cmanton@google.com (Chris Manton)

#ifndef	__LTR506_H__
#define	__LTR506_H__

#include	<linux/input.h>

#ifdef	__KERNEL__
struct ltr506_als_platform_data {
  /* CMM TBD */
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio_int1;
	int gpio_int2;
};

#endif	/* __KERNEL__ */

#endif	/* __LTR506_H__ */
