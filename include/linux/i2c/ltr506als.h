/* Lite-On LTR-506ALS Android Driver
 *
 * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef __LTR506_H
#define __LTR506_H

/* LTR-506 Registers */
#define LTR506_ALS_CONTR	0x80
#define LTR506_PS_CONTR		0x81
#define LTR506_PS_LED		0x82
#define LTR506_PS_N_PULSES	0x83
#define LTR506_PS_MEAS_RATE	0x84
#define LTR506_ALS_MEAS_RATE	0x85
#define LTR506_PART_ID		0x86
#define LTR506_MANUFACTURER_ID	0x87
#define LTR506_ALS_DATA_0	0x88
#define LTR506_ALS_DATA_1	0x89
#define LTR506_ALS_PS_STATUS	0x8A
#define LTR506_PS_DATA_0	0x8B
#define LTR506_PS_DATA_1	0x8C

#define LTR506_ALS_DATA_CH1_0		0x8D
#define LTR506_ALS_DATA_CH1_1		0x8E
#define LTR506_ALS_DATA_CH1_2		0x8F
#define LTR506_ALS_DATA_CH0_0		0x90
#define LTR506_ALS_DATA_CH0_1		0x91
#define LTR506_ALS_DATA_CH0_2		0x92
#define LTR506_ALS_COEFF1_DATA_0	0x93
#define LTR506_ALS_COEFF1_DATA_1	0x94
#define LTR506_ALS_COEFF2_DATA_0	0x95
#define LTR506_ALS_COEFF2_DATA_1	0x96
#define LTR506_ALS_IRF_CUT_OFF		0x97

#define LTR506_INTERRUPT	0x98
#define LTR506_PS_THRES_UP_0	0x99
#define LTR506_PS_THRES_UP_1	0x9A
#define LTR506_PS_THRES_LOW_0	0x9B
#define LTR506_PS_THRES_LOW_1	0x9C
#define LTR506_ALS_THRES_UP_0	0x9E
#define LTR506_ALS_THRES_UP_1	0x9F
#define LTR506_ALS_THRES_LOW_0	0xA0
#define LTR506_ALS_THRES_LOW_1	0xA1
#define LTR506_INTERRUPT_PRST	0xA4



/* Default Settings (Bitshift left: Setting << Bit Number) */
#define ADC_RESOLUTION		(4 << 5)
#define ALS_GAIN		(0 << 3)
#define ALS_SW_RESET		(0 << 2)
#define ALS_MODE		(0 << 1)
#define ALS_MEAS_RATE		(2 << 0)
#define ALS_INT_PRST		(0 << 0)

#define ALS_INT_FLAG		(0 << 3)
#define ALS_NEWDATA		(0 << 2)

#define PS_GAIN			(0 << 2)
#define PS_MODE			(0 << 1)
#define PS_MEAS_RATE		(3 << 0)
#define PS_INT_PRST		(0 << 4)

#define PS_INT_FLAG		(0 << 1)
#define PS_NEWDATA		(0 << 0)

#define INTERRUPT_MODE		(1 << 0)
#define INTERRUPT_POL		(0 << 2)
#define INTERRUPT_OUTPUT_MODE	(0 << 3)

#define LED_PULSE_FREQ		(3 << 5)
#define LED_DUTY_CYC		(1 << 3)
#define LED_PEAK_CURR		(3 << 0)

/* Power On response time in ms */
#define PON_DELAY	600
#define WAKEUP_DELAY	10

/* Interrupt vector number to use when probing IRQ number.
 * User changeable depending on sys interrupt.
 * For IRQ numbers used, see /proc/interrupts.
 */
#define GPIO_INT_NO	32

/* 
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR506_IOCTL_MAGIC      'c'

/* IOCTLs for ltr506 device */
#define LTR506_IOCTL_PS_ENABLE		_IOR(LTR506_IOCTL_MAGIC, 1, int *)
#define LTR506_IOCTL_PS_GET_ENABLED	_IOW(LTR506_IOCTL_MAGIC, 2, int *)
#define LTR506_IOCTL_ALS_ENABLE		_IOR(LTR506_IOCTL_MAGIC, 3, int *)
#define LTR506_IOCTL_ALS_GET_ENABLED	_IOW(LTR506_IOCTL_MAGIC, 4, int *)

struct ltr506_platform_data {
        /* ALS */
        uint16_t pfd_levels[5];
        uint16_t pfd_als_lowthresh;
        uint16_t pfd_als_highthresh;

        /* PS */
        uint16_t pfd_ps_lowthresh;
        uint16_t pfd_ps_highthresh;

        /* Interrupt */
        int pfd_gpio_int_no;
};

#endif
