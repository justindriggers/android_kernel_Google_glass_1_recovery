/**
 *
 * Synaptics Register Mapped Interface (RMI4) SPI Physical Layer Driver.
 * Copyright (C) 2008-2011, Synaptics Incorporated
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

#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include "rmi_spi.h"
#include "rmi_platformdata.h"
#include "rmi_drvr.h"

#define DRIVER_NAME "rmi4_ts"
#define DEVICE_NAME "rmi4_ts"

#define RMI_TDPB	65 /* 65 microseconds inter-byte delay between bytes for RMI chip*/
#define	SPI_BUFSIZ	32

static u8 *buf;

/**
 * This is the data kept on a per instance (client) basis.  This data is
 * always accessible by using the container_of() macro of the various elements
 * inside.
 */
struct instance_data {
	int instance_no;
	int irq;
	struct rmi_phys_driver rpd;
	struct spi_device *spidev;
	struct rmi_spi_platformdata *platformdata;
};


static int spi_xfer(struct spi_device *spi,
		const u8 *txbuf, unsigned n_tx,
		u8 *rxbuf, unsigned n_rx)
{
	static DECLARE_MUTEX(lock);

	int			status;
	struct spi_message	message;
	struct spi_transfer	x[2];
	u8			*local_buf;


	if ((n_tx + n_rx) > SPI_BUFSIZ)
		return -EINVAL;

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if (n_tx) {
		x[0].len = n_tx;
		x[0].delay_usecs = RMI_TDPB;
		spi_message_add_tail(&x[0], &message);
	}
	if (n_rx) {
#ifdef CONFIG_ARCH_OMAP
		x[1].len = n_rx-1;	/* since OMAP has one dummy byte. */
#else
		x[1].len = n_rx;
#endif
		x[1].delay_usecs = RMI_TDPB;
		spi_message_add_tail(&x[1], &message);
	}

	/* ... unless someone else is using the pre-allocated buffer */
	if (down_trylock(&lock)) {
		local_buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
		if (!local_buf)
			return -ENOMEM;
	} else
		local_buf = buf;

	memcpy(local_buf, txbuf, n_tx);


	x[0].tx_buf = local_buf;
	x[1].rx_buf = local_buf + n_tx;

	/* do the i/o */
	status = spi_sync(spi, &message);
	if (status == 0) {
		memcpy(rxbuf, x[1].rx_buf, n_rx);
		status = message.status;
	} else {
		printk(KERN_ERR "spi_sync fials!\n");
	}

	if (x[0].tx_buf == buf)
		up(&lock);
	else
		kfree(local_buf);

	return status;
}

/**
 * Read a single register through spi.
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_read(struct rmi_phys_driver *pd, unsigned short address, char *valp)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);

	char rxbuf[2];
	int retval;
	unsigned short addr = address;

	addr = ((addr & 0xff00) >> 8);
	address = ((address & 0x00ff) << 8);
	addr |= address;
	addr |= 0x80;		/* High bit set indicates read. */

	retval = spi_xfer(id->spidev, (u8 *)&addr, 2, rxbuf, 1);

	*valp = rxbuf[0];

	return retval;
}

/**
 * Same as rmi_spi_read, except that multiple bytes are allowed to be read.
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.  This
 * buffer must be at least size bytes long.
 * \param[in] size The number of bytes to be read.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_read_multiple(struct rmi_phys_driver *pd, unsigned short address,
	char *valp, int size)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	int retval;

	unsigned short addr = address;

	addr = ((addr & 0xff00) >> 8);
	address = ((address & 0x00ff) << 8);
	addr |= address;
	addr |= 0x80;		/* High bit set indicates read. */

	retval = spi_xfer(id->spidev, (u8 *)&addr, 2, valp, size);

	return retval;
}

/**
 * Write a single register through spi.
 * You can write multiple registers at once, but I made the functions for that
 * seperate for performance reasons.  Writing multiple requires allocation and
 * freeing.
 * \param[in] pd
 * \param[in] address The address at which to start the write.
 * \param[in] data The data to be written.
 * \return one upon success, something else upon error.
 */
static int
rmi_spi_write(struct rmi_phys_driver *pd, unsigned short address, char data)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	unsigned char txbuf[4];
	int retval;

	txbuf[2]  = data;
	txbuf[1]  = address;
	txbuf[0]  = address>>8;

	retval = spi_xfer(id->spidev, txbuf, 3, NULL, 0);
	return retval ? 0 : 1;
}

/**
 * Write multiple registers.
 * \param[in] pd
 * \param[in] address The address at which to start the write.
 * \param[in] valp A pointer to a buffer containing the data to be written.
 * \param[in] size The number of bytes to write.
 * \return one upon success, something else upon error.
 */
static int
rmi_spi_write_multiple(struct rmi_phys_driver *pd, unsigned short address,
	char *valp, int size)
{
	struct instance_data *id = container_of(pd, struct instance_data, rpd);
	unsigned char txbuf[32];
	int retval;
	int i;

	txbuf[1]  = address;
	txbuf[0]  = address>>8;

	for (i = 0; i < size; i++)
		txbuf[i + 2] = valp[i];

	retval = spi_xfer(id->spidev, txbuf, size+2, NULL, 0);

	return retval ? 0 : 1;
}

/**
 * This is the Interrupt Service Routine.  It just notifies the application
 * layer that attention is required.
 */
static irqreturn_t spi_attn_isr(int irq, void *info)
{
	struct instance_data *id = info;
	disable_irq(id->irq);
	if (id->rpd.attention)
		id->rpd.attention(&id->rpd, id->instance_no);
	return IRQ_HANDLED;
}


static int rmi_spi_probe(struct spi_device *spi)
{
	struct instance_data *id;
	int retval;
	int i;
	bool found;
	struct rmi_spi_platformdata *platformdata;
	struct rmi_sensordata *sensordata;

	printk(KERN_INFO "Probing RMI4 SPI device\n");

	found = false;

	spi->bits_per_word = 8;

	spi->mode = SPI_MODE_3;

	buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
	if (!buf) {
		printk(KERN_ERR "%s: Out of memory - can't allocate memory for spi buffer\n", __func__);
		return -ENOMEM;
	}

	retval = spi_setup(spi);
	if (retval < 0) {
		printk(KERN_ERR "%s: spi_setup failed.", __func__);
		return retval;
	}

	id = kzalloc(sizeof(*id), GFP_KERNEL);
	if (!id) {
		printk(KERN_ERR "%s: Out of memory - can't allocate memory for instance data.", __func__);
		return -ENOMEM;
	}

	id->spidev             = spi;
	id->rpd.name           = DRIVER_NAME;
	id->rpd.write          = rmi_spi_write;
	id->rpd.read           = rmi_spi_read;
	id->rpd.write_multiple = rmi_spi_write_multiple;
	id->rpd.read_multiple  = rmi_spi_read_multiple;
	id->rpd.module         = THIS_MODULE;
	id->rpd.polling_required = true; /* default to polling if irq not used */

	/* Loop through the client data and locate the one that was found. */

	platformdata = spi->dev.platform_data;
	if (platformdata == NULL) {
		printk(KERN_ERR "%s: CONFIGURATION ERROR - platform data is NULL.", __func__);
		return -EINVAL;
	}
	id->platformdata = platformdata;
	sensordata = platformdata->sensordata;

	/* Call the platform setup routine, to do any setup that is required before
	 * interacting with the device.
	 */
	if (sensordata && sensordata->rmi_sensor_setup) {
		retval = sensordata->rmi_sensor_setup();
		if (retval) {
			printk(KERN_ERR "%s: sensor setup failed with code %d.", __func__, retval);
			return retval;
		}
	}

	/* TODO: I think this if is no longer required. */
	if (platformdata->chip == RMI_SUPPORT) {
		id->instance_no = i;

		/* set the device name using the instance_no appended to DEVICE_NAME to make a unique name */
		dev_set_name(&spi->dev, "rmi4-spi%d", id->instance_no);
		/*
		* Determine if we need to poll (inefficient) or use interrupts.
		*/
		if (platformdata->irq) {
			int irqtype;

			id->irq = platformdata->irq;
			switch (platformdata->irq_type) {
			case IORESOURCE_IRQ_HIGHEDGE:
				irqtype = IRQF_TRIGGER_RISING;
				break;
			case IORESOURCE_IRQ_LOWEDGE:
				irqtype = IRQF_TRIGGER_FALLING;
				break;
			case IORESOURCE_IRQ_HIGHLEVEL:
				irqtype = IRQF_TRIGGER_HIGH;
				break;
			case IORESOURCE_IRQ_LOWLEVEL:
				irqtype = IRQF_TRIGGER_LOW;
				break;
			default:
				dev_warn(&spi->dev, "%s: Invalid IRQ flags in platform data.", __func__);
				kfree(id);
				return -ENXIO;
			}

			retval = request_irq(id->irq, spi_attn_isr, irqtype, "rmi_spi", id);
			if (retval) {
				dev_info(&spi->dev, "%s: Unable to get attn irq %d.  Reverting to polling.", __func__, id->irq);
				id->rpd.polling_required = true;
			} else {
				dev_dbg(&spi->dev, "%s: got irq", __func__);
				id->rpd.polling_required = false;
				id->rpd.irq = id->irq;
			}
		} else {
			id->rpd.polling_required = true;
			dev_info(&spi->dev, "%s: No IRQ info given. Polling required.", __func__);
		}
	}

	/* Store instance data for later access. */
	if (id)
		spi_set_drvdata(spi, id);

	/* Register the sensor driver - which will trigger a scan of the PDT. */
	retval = rmi_register_sensor(&id->rpd, platformdata->perfunctiondata);
	if (retval) {
		printk(KERN_ERR "rmi_register_phys_driver failed with code %d.", retval);
		if (id->irq)
			free_irq(id->irq, id);
		kfree(id);
		return retval;
	}

	printk(KERN_DEBUG "%s: Successfully Registered %s.", __func__, id->rpd.name);

	return 0;
}

static int rmi_spi_suspend(struct spi_device *spi, pm_message_t message)
{
	return 0;
}

static int rmi_spi_resume(struct spi_device *spi)
{
	return 0;
}

static int __devexit rmi_spi_remove(struct spi_device *spi)
{
	struct instance_data *id = spi_get_drvdata(spi);

	rmi_spi_suspend(spi, PMSG_SUSPEND);

	rmi_unregister_sensors(&id->rpd);

	if (id) {
		if (id->irq)
			free_irq(id->irq, id);
		kfree(id);
	}

	return 0;
}

static struct spi_driver rmi_spi_driver = {
	.driver = {
		.name  = "rmi_spi",
		.bus   = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe    = rmi_spi_probe,
	.remove   = __devexit_p(rmi_spi_remove),
	.suspend  = rmi_spi_suspend,
	.resume   = rmi_spi_resume,
};

/**
 * The Platform Driver probe function.  We just tell the spi subsystem about
 * ourselves in this call.
 */
static int
rmi_spi_plat_probe(struct platform_device *dev)
{
	struct rmi_spi_data *mid = dev->dev.platform_data;

	if (!mid) {
		printk(KERN_ERR "A platform device must contain rmi_spi_data\n");
		return -ENXIO;
	}

	return spi_register_driver(&rmi_spi_driver);
}

/**
 * Tell the spi subsystem that we're done.
 * \param[in] dev
 * \return Always returns 0.
 */
static int
rmi_spi_plat_remove(struct platform_device *dev)
{
	spi_unregister_driver(&rmi_spi_driver);
	return 0;
}

/**
 * Structure used to tell the Platform Driver subsystem about us.
 */
static struct platform_driver rmi_spi_platform_driver = {
	.driver		= {
		.name	= "rmi_spi_plat",
	},
	.probe		= rmi_spi_plat_probe,
	.remove		= rmi_spi_plat_remove,
};

static int __init rmi_spi_init(void)
{
	return platform_driver_register(&rmi_spi_platform_driver);
}
module_init(rmi_spi_init);

static void __exit rmi_spi_exit(void)
{
	kfree(buf);
	buf = NULL;
	platform_driver_unregister(&rmi_spi_platform_driver);
}
module_exit(rmi_spi_exit);

/** Standard driver module information - the author of the module.
 */
MODULE_AUTHOR("Synaptics, Inc.");
/** Standard driver module information - a summary description of this module.
 */
MODULE_DESCRIPTION("RMI4 Driver SPI Physical Layer");
/** Standard driver module information - the license under which this module
 * is included in the kernel.
 */
MODULE_LICENSE("GPL");

