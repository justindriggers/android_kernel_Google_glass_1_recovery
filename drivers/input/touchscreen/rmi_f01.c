/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $01 support for sensor
 * control and configuration.
 *
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
 *
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/param.h>

#include "rmi.h"
#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_sensor.h"
#include "rmi_function.h"
#include "rmi_f01.h"


#define RMI_REPORT_RATE_80 0
#define RMI_REPORT_RATE_40 (1 << 6)

/** Context data for each F01 we find.
 */
struct f01_instance_data {
	struct rmi_F01_control *controlRegisters;
	struct rmi_F01_data *dataRegisters;
	struct rmi_F01_query *queryRegisters;
};

/*.
 * The interrupt handler for Fn $01 doesn't do anything (for now).
 */
void FN_01_inthandler(struct rmi_function_info *rmifninfo,
	unsigned int assertedIRQs)
{
	struct f01_instance_data *instanceData = (struct f01_instance_data *) rmifninfo->fndata;

	printk(KERN_DEBUG "%s: Read device status.", __func__);

	if (rmi_read_multiple(rmifninfo->sensor, rmifninfo->funcDescriptor.dataBaseAddr,
		&instanceData->dataRegisters->deviceStatus, 1)) {
		printk(KERN_ERR "%s : Could not read F01 device status.\n",
			__func__);
	}

	/* TODO: Check for reset and handle appropriately.
	*/
}
EXPORT_SYMBOL(FN_01_inthandler);

/*
 * This reads in the function $01 source data.
 *
 */
void FN_01_attention(struct rmi_function_info *rmifninfo)
{
	struct f01_instance_data *instanceData = (struct f01_instance_data *) rmifninfo->fndata;

	/* TODO: Compute size to read and number of IRQ registers to processors
	* dynamically.  See comments in rmi.h. */
	if (rmi_read_multiple(rmifninfo->sensor, rmifninfo->funcDescriptor.dataBaseAddr+1,
		instanceData->dataRegisters->irqs, 1)) {
		printk(KERN_ERR "%s : Could not read interrupt status registers at 0x%02x\n",
			__func__, rmifninfo->funcDescriptor.dataBaseAddr);
		return;
	}
//	printk(KERN_INFO "%s: IRQ = 0x%02x", __func__, instanceData->dataRegisters->irqs[0]);

	if (instanceData->dataRegisters->irqs[0] & instanceData->controlRegisters->interruptEnable[0]) {
		/* call down to the sensors irq dispatcher to dispatch all enabled IRQs */
		rmifninfo->sensor->dispatchIRQs(rmifninfo->sensor,
			instanceData->dataRegisters->irqs[0]);
	}

}
EXPORT_SYMBOL(FN_01_attention);

int FN_01_config(struct rmi_function_info *rmifninfo)
{
	/* print info and do any source specific configuration. */
	int retval = 0;

	printk(KERN_DEBUG "%s: RMI4 function $01 config\n", __func__);

	/* On slow processors, we need to throttle back the rate at which
	* data updates become ready. */
#if 0
	/* TODO: This code gets invoked pointlessly on some systems, and causes
	 * the Synaptics device to stop working in those cases.  We need to figure
	 * out what's going on there.
	 */
	if (HZ < 500) {
		/* The default packet rate of 80 packets per
		* second is too fast (the Linux time slice for
		* sub-GHz processors is only 100 times per second).
		* So re-program it to 40 packets per second.
		*/
		/* TODO: We need to OR this in, rather than stomping on the other
		 * contents of the register.  It's OK for now, because this is
		 * early in the initializaton process and nobody else has had
		 * a chance to change this register, which defaults to 0.
		 */
		rmi_write(rmifninfo->sensor, rmifninfo->funcDescriptor.controlBaseAddr, RMI_REPORT_RATE_40);
		printk(KERN_INFO "%s: Throttled back reporting for slow CPU (%d HZ).", __func__, HZ);
	}
#endif

	return retval;
}
EXPORT_SYMBOL(FN_01_config);

/* Initialize any function $01 specific params and settings - input
 * settings, device settings, etc.
 */
int FN_01_init(struct rmi_function_device *function_device)
{
	pr_debug("%s: RMI4 function $01 init\n", __func__);

	return 0;
}
EXPORT_SYMBOL(FN_01_init);

int FN_01_detect(struct rmi_function_info *rmifninfo,
	struct rmi_function_descriptor *fndescr, unsigned int interruptCount)
{
	int i;
	int InterruptOffset;
	int retval = 0;
	struct f01_instance_data *instanceData;
	struct rmi_F01_control *controlRegisters;
	struct rmi_F01_data *dataRegisters;
	struct rmi_F01_query *queryRegisters;

	pr_debug("%s: RMI4 function $01 detect\n", __func__);

	/* Store addresses - used elsewhere to read data,
	* control, query, etc. */
	rmifninfo->funcDescriptor.queryBaseAddr = fndescr->queryBaseAddr;
	rmifninfo->funcDescriptor.commandBaseAddr = fndescr->commandBaseAddr;
	rmifninfo->funcDescriptor.controlBaseAddr = fndescr->controlBaseAddr;
	rmifninfo->funcDescriptor.dataBaseAddr = fndescr->dataBaseAddr;
	rmifninfo->funcDescriptor.interruptSrcCnt = fndescr->interruptSrcCnt;
	rmifninfo->funcDescriptor.functionNum = fndescr->functionNum;

	rmifninfo->numSources = fndescr->interruptSrcCnt;

	/* Set up context data. */
	instanceData = kzalloc(sizeof(*instanceData), GFP_KERNEL);
	if (!instanceData) {
		printk(KERN_ERR "%s: Error allocating memory for F01 context data.\n", __func__);
		return -ENOMEM;
	}
	queryRegisters = kzalloc(sizeof(*queryRegisters), GFP_KERNEL);
	if (!queryRegisters) {
		printk(KERN_ERR "%s: Error allocating memory for F01 query registers.\n", __func__);
		return -ENOMEM;
	}
	instanceData->queryRegisters = queryRegisters;
	retval = rmi_read_multiple(rmifninfo->sensor, rmifninfo->funcDescriptor.queryBaseAddr,
		(char *)instanceData->queryRegisters, sizeof(struct rmi_F01_query));
	if (retval) {
		printk(KERN_ERR "%s : Could not read F01 control registers at 0x%02x. Error %d.\n",
			__func__, rmifninfo->funcDescriptor.dataBaseAddr, retval);
	}
	printk(KERN_DEBUG "%s: RMI Protocol: %d.%d",
		__func__, queryRegisters->rmi_maj_ver, queryRegisters->rmi_min_ver);
	printk(KERN_DEBUG "%s: Manufacturer: %d %s", __func__,
		queryRegisters->mfgid, queryRegisters->mfgid == 1 ? "(Synaptics)" : "");
	printk(KERN_DEBUG "%s: Properties: 0x%x",
		__func__, queryRegisters->properties);
	printk(KERN_DEBUG "%s: Product Info: 0x%x 0x%x",
		__func__, queryRegisters->prod_info[0], queryRegisters->prod_info[1]);
	printk(KERN_DEBUG "%s: Date Code: Year : %d Month: %d Day: %d",
		__func__, queryRegisters->date_code[0], queryRegisters->date_code[1],
		queryRegisters->date_code[2]);
	printk(KERN_DEBUG "%s: Tester ID: %d", __func__, queryRegisters->tester_id);
	printk(KERN_DEBUG "%s: Serial Number: 0x%x",
		__func__, queryRegisters->serial_num);
	printk(KERN_DEBUG "%s: Product ID: %s", __func__, queryRegisters->prod_id);

	/* TODO: size of control registers needs to be computed dynamically.  See comment
	* in rmi.h. */
	controlRegisters = kzalloc(sizeof(*controlRegisters), GFP_KERNEL);
	if (!controlRegisters) {
		printk(KERN_ERR "%s: Error allocating memory for F01 control registers.\n", __func__);
		return -ENOMEM;
	}
	instanceData->controlRegisters = controlRegisters;
	retval = rmi_read_multiple(rmifninfo->sensor, rmifninfo->funcDescriptor.controlBaseAddr,
		(char *)instanceData->controlRegisters, sizeof(struct rmi_F01_control));
	if (retval) {
		printk(KERN_ERR "%s : Could not read F01 control registers at 0x%02x. Error %d.\n",
			__func__, rmifninfo->funcDescriptor.dataBaseAddr, retval);
	}

	/* TODO: size of data registers needs to be computed dynamically.  See comment
	* in rmi.h. */
	dataRegisters = kzalloc(sizeof(*dataRegisters), GFP_KERNEL);
	if (!dataRegisters) {
		printk(KERN_ERR "%s: Error allocating memory for F01 data registers.\n", __func__);
		return -ENOMEM;
	}
	instanceData->dataRegisters = dataRegisters;
	rmifninfo->fndata = instanceData;

	/* Need to get interrupt info to be used later when handling
	interrupts. */
	rmifninfo->interruptRegister = interruptCount/8;

	/* loop through interrupts for each source and or in a bit
	to the interrupt mask for each. */
	InterruptOffset = interruptCount % 8;

	for (i = InterruptOffset;
		i < ((fndescr->interruptSrcCnt & 0x7) + InterruptOffset);
		i++) {
			rmifninfo->interruptMask |= 1 << i;
	}

	return retval;
}
EXPORT_SYMBOL(FN_01_detect);
