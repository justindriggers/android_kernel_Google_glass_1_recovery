/* Driver for Glass low-power sensor hub
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/ctype.h>

/* Used to provide access via misc device */
#include <linux/miscdevice.h>

/* Used to provide user access to fileops API */
#include <asm-generic/uaccess.h>

/* download firmware */
#include <linux/firmware.h>

#include <linux/i2c/glasshub.h>

/* module debug flags */
#define DEBUG_FLASH_MODE 0

/* driver name/version */
#define DEVICE_NAME "glasshub"
#define DRIVER_VERSION "0.3"

/* minimum MCU version for this driver */
#define MINIMUM_MCU_VERSION	9

/* special app code to flash the bootloader */
#define BOOTLOADER_FLASHER	0xff

/* number of retries */
#define NUMBER_OF_I2C_RETRIES	3

/* 8-bit registers for the glass hub MCU */
#define REG_RESET			0
#define REG_STATUS			0
#define REG_PART_ID			1
#define REG_VERSION			2
#define REG_ENABLE_INT			3
#define REG_ENABLE_PASSTHRU		4
#define REG_ENABLE_DON_DOFF		5
#define REG_DON_DOFF			6
#define REG_ENABLE_WINK			7
#define REG_DON_DOFF_HYSTERESIS		8
#define REG_LED_DRIVE			9
#define REG_ERROR_CODE			10
#define REG_FLASH_DATA			11
#define REG_DETECTOR_GAIN		12
#define REG_PROX_PART_ID		13
#define REG_PROX_SEQ_ID			14
#define REG_WINK_INHIBIT		15
#define REG_WINK_STATUS			16
#define REG_DEBUG			17

/* 16-bit registers */
#define REG16_DETECTOR_BIAS		0x80
#define REG16_DON_DOFF_THRESH		0x81
#define REG16_MIN_PROX			0x82
#define REG16_PROX_RAW			0x83
#define REG16_PROX_DATA			0x84
#define REG16_ADDRESS			0x85

#define CMD_APP_VERSION			0xF6
#define CMD_BOOTLOADER_VERSION		0xF7
#define CMD_FLASH_STATUS		0xF8
#define CMD_FLASH			0xF9
#define CMD_BOOT			0xFA

/* interrupt sources */
#define IRQ_PASSTHRU			0b00000001
#define IRQ_WINK			0b00000010
#define IRQ_DON_DOFF			0b00000100

/* device ID */
#define GLASSHUB_PART_ID		0xbb

#define PROX_DATA_FIFO_SIZE		16

/* input device constants */
#define PS_MIN_VALUE			0
#define PS_MAX_VALUE			65535

/* bit fields for flags */
#define FLAG_WAKE_THREAD		0
#define FLAG_DEVICE_BOOTED		1
#define FLAG_FLASH_MODE			2
#define FLAG_SYSFS_CREATED		3

/* flags for device permissions */
#define DEV_MODE_RO (S_IRUSR | S_IRGRP)
#define DEV_MODE_WO (S_IWUSR | S_IWGRP)
#define DEV_MODE_RW (DEV_MODE_RO | DEV_MODE_WO)

/* firmware parameters */
#define FIRMWARE_PAGE_SIZE		64
#define FIRMWARE_TOTAL_SIZE		(8*1024)
#define FIRMWARE_TOTAL_PAGES		(FIRMWARE_TOTAL_SIZE / FIRMWARE_PAGE_SIZE)
#define FIRMWARE_NUM_DIRTY_BITS		((FIRMWARE_TOTAL_PAGES+ 31) / 32)
#define FIRMWARE_BASE_ADDRESS		0x8000
#define FIRMWARE_END_ADDRESS		(FIRMWARE_BASE_ADDRESS + FIRMWARE_TOTAL_SIZE - 1)

/* firmware state machine */
#define FW_STATE_START			0
#define FW_STATE_TYPE			1
#define FW_STATE_COUNT_HI		2
#define FW_STATE_COUNT_LO		3
#define FW_STATE_ADDR_7			4
#define FW_STATE_ADDR_6			5
#define FW_STATE_ADDR_5			6
#define FW_STATE_ADDR_4			7
#define FW_STATE_ADDR_3			8
#define FW_STATE_ADDR_2			9
#define FW_STATE_ADDR_1			10
#define FW_STATE_ADDR_0			11
#define FW_STATE_BYTE_HI		12
#define FW_STATE_BYTE_LO		13
#define FW_STATE_CHECKSUM_HI		14
#define FW_STATE_CHECKSUM_LO		15

/* location of calibration data */
#define CALIB_ADDRESS			0x9ff0

/* number of samples to take for calibration mean */
#define NUM_CALIBRATION_SAMPLES		3

/* number of samples in prox data buffer */
#define PROX_QUEUE_SZ			64
#define PROX_INTERVAL			(1000000000LL / 32)

/* values for flash_status */
#define FLASH_STATUS_OK			0
#define FLASH_STATUS_READY		1
#define FLASH_ERROR_DEV_REJECTED_PKT	-1
#define FLASH_ERROR_INVALID_S_RECORD	-2
#define FLASH_ERROR_ADDRESS_RANGE	-3
#define FLASH_ERROR_INVALID_HEX_DIGIT	-4
#define FLASH_ERROR_SREC_CHECKSUM_ERROR	-5
#define FLASH_ERROR_DEVICE_RESET_ERROR	-6
#define FLASH_ERROR_I2C_DEVICE_COMM	-7
#define FLASH_ERROR_NO_PAGES_FLASHED	-8

/*
 * Basic theory of operation:
 *
 * The glass hub device comes up in bootloader mode. This mode supports only 5 commands:
 *
 * BOOT		Jump to the application code stored in flash
 * FLASH	Download a 64-byte page of firmware to flash
 * FLASH_STATUS	Report status of last flash operation
 * APP_VERSION	Returns the 2-byte application version
 * BOOT_VERSION	Returns the 1-byte bootloader version
 *
 * It is now possible to flash the bootloader by loading special application
 * code. It reports a version number of 255.xxx which allows the driver to
 * identify it as special code to flash the bootloader. Flash operation
 * proceeds as normal except that at the point where the firmware image is to
 * be flashed, the driver boots into the special application code. Flashing
 * takes place as usual, the driver then resets into the new bootloader code
 * and reads the bootloader version. To enable this process from user space,
 * first download the special bootloader app code, then write a 1 to the
 * fw_update_enable sysfs node, download the bootloader app code, then write
 * a 255 to the fw_update_enable node. It is this last step that tells the
 * driver code that you want to write bootloader code instead of app code.
 * There is some risk that this operation could result in a bricked device,
 * since a failure to write the boot code is unrecoverable. In this case,
 * it will be necessary to attach a SWIM connector and use the debugger
 * to flash a new bootloader image.
 *
 * The application code can be flashed by a simple script. First, write a 1
 * to the enable_fw_update sysfs node. The driver will allocate a block of
 * memory to save the firmware image and clear the dirty bits for each page
 * of code. Next, write the .S19 the .S19 application code image in text
 * format (e.g. "cat") to the update_fw_data sysfs node. The driver will
 * validate the checksum of each line and store it in the binary code buffer,
 * setting a dirty bit for each 64-byte page of code it touches. Be aware
 * that it doesn't backfill missing data, i.e. if you touch a single byte on
 * a page, the rest of the page will be written with zeroes. Thus, it's
 * always a good idea to flash the entire image each time. Finally, write a 0
 * to the enable_fw_update sysfs node. This will cause the driver to write
 * each dirty page to the device. Note that the concept of a flash
 * programming mode is purely an artifact of the driver and not something the
 * device itself enforces or is even aware of.
 *
 * For normal operation, the driver will automatically boot the device to
 * start running application code the first time it receives any command that
 * requires the application code. From the application code, a RESET command
 * will cause the device to return to the bootloader. It is possible that a
 * bug in the application code will make it impossible to return to the
 * bootloader. In this case, it may be necessary to power-down the device to
 * force it back into the bootloader. This allows for recovery in case there
 * is a serious bug in the application code.
 */

struct glasshub_data {
	struct i2c_client *i2c_client;
	struct input_dev *ps_input_dev;
	const struct glasshub_platform_data *pdata;
	uint8_t *fw_image;
	uint32_t fw_dirty[FIRMWARE_NUM_DIRTY_BITS];
	int flash_status;
	int fw_state;
	int fw_rec_type;
	int fw_index;
	int fw_count;
	uint32_t fw_value;
	uint8_t fw_checksum;
	struct mutex device_lock;
	long long irq_timestamp;
	unsigned long sample_count;
	volatile unsigned long flags;
	uint8_t don_doff_state;
	uint8_t bootloader_version;
	uint8_t app_version_major;
	uint8_t app_version_minor;
};

struct glasshub_data *glasshub_private = NULL;

/*
 * Filesystem API is implemented with the kernel fifo.
 */
static DECLARE_WAIT_QUEUE_HEAD(glasshub_read_wait);
static struct glasshub_data_user glasshub_fifo_buffer[PROX_QUEUE_SZ];
static struct glasshub_data_user *glasshub_user_wr_ptr = NULL;
static struct glasshub_data_user *glasshub_user_rd_ptr = NULL;
static struct mutex glasshub_user_lock;
static atomic_t glasshub_opened = ATOMIC_INIT(0);

static void unregister_device_files(struct glasshub_data *glasshub);
static int register_device_files(struct glasshub_data *glasshub);

/*
 * I2C bus transaction read for consecutive data.
 * Returns 0 on success.
 */
static int _i2c_read(struct glasshub_data *glasshub, uint8_t *txData, int txLength,
		uint8_t *rxData,int rxLength)
{
	int i;
	struct i2c_msg data[] = {
		{
			.addr = glasshub->i2c_client->addr,
			.flags = 0,
			.len = txLength,
			.buf = txData,
		},
		{
			.addr = glasshub->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = rxLength,
			.buf = rxData,
		},
	};

	for (i = 0; i < NUMBER_OF_I2C_RETRIES; i++) {
		if (i2c_transfer(glasshub->i2c_client->adapter, data, 2) > 0)
			break;
		/* Delay before retrying */
		mdelay(10);
	}

	if (i >= NUMBER_OF_I2C_RETRIES) {
		dev_err(&glasshub->i2c_client->dev, "%s i2c read retry exceeded\n", __FUNCTION__);
		return -EIO;
	}
	return 0;
}

/*
 * I2C bus transaction to write consecutive data.
 * Returns 0 on success.
 */
static int _i2c_write_mult(struct glasshub_data *glasshub, uint8_t *txData, int length)
{
	int i;
	struct i2c_msg data[] = {
		{
			.addr = glasshub->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (i = 0; i < NUMBER_OF_I2C_RETRIES; i++) {
		if (i2c_transfer(glasshub->i2c_client->adapter, data, 1) > 0)
			break;
		/* Delay before retrying */
		mdelay(10);
	}

	if (i >= NUMBER_OF_I2C_RETRIES) {
		dev_err(&glasshub->i2c_client->dev, "%s i2c write retry exceeded\n", __FUNCTION__);
		return -EIO;
	}
	return 0;
}

/*
 * I2C bus transaction to read a register
 * Returns 0 on success.
 */
static int _i2c_read_reg(struct glasshub_data *glasshub, uint8_t reg, unsigned *value)
{
	int rc = 0;
	uint8_t buffer[2];
	*value = 0xffff;

	buffer[0] = reg;
	buffer[1] = 0;
	if (reg & 0x80) {
		rc = _i2c_read(glasshub, buffer, 1, buffer, 2);
		if (rc == 0) {
			*value = buffer[0] | (unsigned) buffer[1] << 8;
		}
	} else {
		rc = _i2c_read(glasshub, buffer, 1, buffer, 1);
		if (rc == 0) {
			*value = buffer[0];
		}
	}
	return rc;
}

/*
 * I2C bus transaction to read single byte.
 * Returns 0 on success.
 */
static int _i2c_read_reg8(struct glasshub_data *glasshub, uint8_t reg, uint8_t *data)
{
	int rc;
	unsigned value;
	rc = _i2c_read_reg(glasshub, reg, &value);
	*data = (uint8_t) value;
	return rc;
}

/*
 * I2C bus transaction to write register
 * Returns 0 on success.
 */
static int _i2c_write_reg(struct glasshub_data *glasshub, uint8_t reg, uint16_t data)
{
	uint8_t buffer[3];

	buffer[0] = reg;
	buffer[1] = data & 0xff;
	buffer[2] = (data >> 8) & 0xff;
	return _i2c_write_mult(glasshub, buffer, reg & 0x80 ? 3 : 2);
}

/* read an 8-bit value from glasshub memory */
static int read_glasshub_memory(struct glasshub_data *glasshub, uint16_t addr, unsigned *value)
{
	int rc;

	*value = 0;

	/* send address */
	rc = _i2c_write_reg(glasshub, REG16_ADDRESS, addr);
	if (rc) goto Error;

	/* read data */
	rc = _i2c_read_reg(glasshub, REG_FLASH_DATA, value);

Error:
	return rc;
}

static int write_glasshub_memory(struct glasshub_data *glasshub, uint16_t addr, uint8_t value)
{
	int rc;

	rc = _i2c_write_reg(glasshub, REG16_ADDRESS, addr);
	if (rc == 0) {
		_i2c_write_reg(glasshub, REG_FLASH_DATA, value);
	}
	return rc;
}

static int _check_part_id(struct glasshub_data *glasshub)
{
	int rc = 0;
	uint8_t data = 0;
	struct i2c_client *i2c_client = glasshub->i2c_client;

	rc = _i2c_read_reg8(glasshub, REG_PART_ID, &data);
	if (rc < 0) {
		dev_err(&i2c_client->dev, "%s: Unable to read part identifier\n", __FUNCTION__);
		return -EIO;
	}

	if (data != GLASSHUB_PART_ID) {
		dev_err(&i2c_client->dev, "%s Unexpected part ID = %u\n",
				__FUNCTION__, (unsigned)data);
		rc = -ENODEV;
	}

	return rc;
}

/* must hold the device lock */
int boot_device_l(struct glasshub_data *glasshub)
{
	int retry;
	uint8_t temp;
	int rc = 0;

	if (test_bit(FLAG_DEVICE_BOOTED, &glasshub->flags)) goto err_out;

	/* don't allow boot from flash mode, unless boot flasher app
	 * code has been loaded.
	 */
	if (glasshub->app_version_major != BOOTLOADER_FLASHER) {
		if (test_bit(FLAG_FLASH_MODE, &glasshub->flags)) {
			rc = -ENODEV;
			goto err_out;
		}
	}

	/* tell glass hub to boot */
	temp = CMD_BOOT;
	for (retry = 0; retry < 5; retry++) {
		rc = _i2c_write_mult(glasshub, &temp, sizeof(temp));
		msleep(100);
		if (rc == 0) break;
	}
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s Unable to boot glasshub device\n",
				__FUNCTION__);
		goto err_out;
	}

	/* if special boot flasher code, don't do anything else */
	if (glasshub->app_version_major == BOOTLOADER_FLASHER) return rc;

	/* verify part ID */
	if (_check_part_id(glasshub)) {
		rc = -ENODEV;
		goto err_out;
	}

	/* get current don/doff state */
	_i2c_read_reg8(glasshub, REG_DON_DOFF, &glasshub->don_doff_state);

	set_bit(FLAG_DEVICE_BOOTED, &glasshub->flags);

err_out:
	return rc;
}

/* must hold the device lock */
int reset_device_l(struct glasshub_data *glasshub, int force)
{
	int retry;
	uint8_t temp;
	int rc = 0;

	if (!force && !test_bit(FLAG_DEVICE_BOOTED, &glasshub->flags)) goto err_out;

	/* reset glass hub */
	for (retry = 0; retry < 5; retry++) {
		temp = REG_RESET;
		rc = _i2c_write_mult(glasshub, &temp, sizeof(temp));
		if (rc == 0) break;
	}
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s Unable to reset glasshub device\n",
				__FUNCTION__);
		goto err_out;
	}
	msleep(30);
	clear_bit(FLAG_DEVICE_BOOTED, &glasshub->flags);

err_out:
	return rc;
}

/* Main interrupt handler. We save a timestamp here and schedule
 * the threaded handler to run later, since we might have to
 * block on I/O requests from user space.
 */
static irqreturn_t glasshub_irq_handler(int irq, void *dev_id)
{
	struct glasshub_data *glasshub = (struct glasshub_data*) dev_id;

	/* if device is not booted, the interrupt must be someone else */
	if (!test_bit(FLAG_DEVICE_BOOTED, &glasshub->flags)) goto Handled;

	/* if threaded handler is already scheduled, don't schedule it again */
	if (test_and_set_bit(FLAG_WAKE_THREAD, &glasshub->flags)) goto Handled;

	/* save timestamp for threaded handler and schedule it */
	glasshub->irq_timestamp = read_robust_clock();
	return IRQ_WAKE_THREAD;

Handled:
	return IRQ_HANDLED;
}

/* Threaded interrupt handler. This is where the real work gets done.
 * We grab a mutex to prevent I/O requests from user space from
 * running concurrently. The timestamp for critical operations comes
 * from the main interrupt handler.
 */
static irqreturn_t glasshub_threaded_irq_handler(int irq, void *dev_id)
{
	struct glasshub_data *glasshub = (struct glasshub_data*) dev_id;
	int rc = 0;
	uint8_t status;
	uint16_t data[PROX_QUEUE_SZ];
	int proxCount = 0;
	int i;
	u64 timestamp;

	/* clear in-service flag */
	clear_bit(FLAG_WAKE_THREAD, &glasshub->flags);

	mutex_lock(&glasshub->device_lock);

	/* read the IRQ source */
	rc = _i2c_read_reg8(glasshub, REG_STATUS, &status);
	if (rc) goto Error;

	/* process don/doff */
	if (status & IRQ_DON_DOFF) {
		rc = _i2c_read_reg8(glasshub, REG_DON_DOFF, &glasshub->don_doff_state);
		if (rc) goto Error;
		dev_info(&glasshub->i2c_client->dev, "%s: don/doff state = %u\n",
				__FUNCTION__, glasshub->don_doff_state);
		sysfs_notify(&glasshub->i2c_client->dev.kobj, NULL, "don_doff");
	}

	/* process wink signal */
	if (status & IRQ_WINK) {
		dev_info(&glasshub->i2c_client->dev, "%s: wink signal received\n",
				__FUNCTION__);
		sysfs_notify(&glasshub->i2c_client->dev.kobj, NULL, "wink");
	}

	/* process prox data */
	while (status & IRQ_PASSTHRU) {
		unsigned value = 0;

		/* read value */
		rc = _i2c_read_reg(glasshub, REG16_PROX_DATA, &value);
		if (rc) goto Error;
		++glasshub->sample_count;

		/* Buffer up data (and drop data that exceeds our buffer
		 * length). Note that when the high bit is set, there is
		 * no more data. This mechanism saves us from doing another
		 * I2C bus transfer to check the status register.
		 */
		if (proxCount < PROX_QUEUE_SZ) {
			data[proxCount++] = (uint16_t) value & 0x7fff;
		}

		/* check for end of data */
		if (value & 0x8000) break;
	}

	/* pass prox data to user space */
	if (proxCount) {
		mutex_lock(&glasshub_user_lock);

		/* backdate timestamps */
		timestamp = glasshub->irq_timestamp - (proxCount - 1) * PROX_INTERVAL;

		for (i = 0; i < proxCount; i++) {
			glasshub_user_wr_ptr->value = data[i];
			glasshub_user_wr_ptr->timestamp = timestamp;
			timestamp += PROX_INTERVAL;
			glasshub_user_wr_ptr++;
			if (glasshub_user_wr_ptr == (glasshub_fifo_buffer + PROX_QUEUE_SZ)) {
				glasshub_user_wr_ptr = glasshub_fifo_buffer;
			}
		}

		/* wake up user space */
		wake_up_interruptible(&glasshub_read_wait);
		mutex_unlock(&glasshub_user_lock);
	}

	mutex_unlock(&glasshub->device_lock);
	return IRQ_HANDLED;

Error:
	mutex_unlock(&glasshub->device_lock);
	dev_err(&glasshub->i2c_client->dev, "%s: device read error\n", __FUNCTION__);
	return IRQ_HANDLED;
}

/* common routine to return an I2C register to userspace */
static ssize_t show_reg(struct device *dev, char *buf, uint8_t reg)
{
	unsigned value = 0xffff;
	struct glasshub_data *glasshub = dev_get_drvdata(dev);

	mutex_lock(&glasshub->device_lock);
	if (boot_device_l(glasshub) == 0) {
		_i2c_read_reg(glasshub, reg, &value);
	}
	mutex_unlock(&glasshub->device_lock);

	return sprintf(buf, "%u\n", value);
}

/* common routine to set an I2C register from userspace */
static ssize_t store_reg(struct device *dev, const char *buf, size_t count,
		uint8_t reg, uint16_t min, uint16_t max)
{
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	unsigned long value = 0;
	if (!kstrtoul(buf, 10, &value)) {
		value = (value < min) ? min : value;
		value = (value > max) ? max : value;
		mutex_lock(&glasshub->device_lock);
		if (boot_device_l(glasshub) == 0) {
			_i2c_write_reg(glasshub, reg, value);
		}
		mutex_unlock(&glasshub->device_lock);
	}
	return count;
}

/* show the application version number */
static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	return sprintf(buf, "%d.%d\n", glasshub->app_version_major, glasshub->app_version_minor);
}

/* show the bootloader version number */
static ssize_t bootloader_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", glasshub->bootloader_version);
}

/* show don/doff status */
static ssize_t don_doff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_DON_DOFF);
}

/* show don/doff enable status */
static ssize_t don_doff_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_ENABLE_DON_DOFF);
}

/* enable/disable don/doff */
static ssize_t don_doff_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_ENABLE_DON_DOFF, 0, 1);
}

/* show prox passthrough mode */
static ssize_t passthru_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_ENABLE_PASSTHRU);
}

/* enable/disable prox passthrough mode */
static ssize_t passthru_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_ENABLE_PASSTHRU, 0, 1);
}

/* read prox value */
static int read_prox_raw(struct glasshub_data *glasshub, uint16_t *pProxData)
{
	int rc;
	unsigned data;

	*pProxData = 0xffff;
	rc = _i2c_read_reg(glasshub, REG16_PROX_RAW, &data);
	if (rc == 0) {
		*pProxData = (uint16_t) data;
	}
	return rc;
}

/* show minimum prox value */
static ssize_t proxmin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG16_MIN_PROX);
}

/* show raw prox value */
static ssize_t proxraw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG16_PROX_RAW);
}

/* show raw IR value */
static ssize_t ir_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* TODO implement this!!! */
	unsigned data = 0xffff;
	return sprintf(buf, "%u\n", data);
}

/* show raw visible light value */
static ssize_t vis_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* TODO implement this!!! */
	unsigned data = 0xffff;
	return sprintf(buf, "%u\n", data);
}

/* show don/doff threshold value */
static ssize_t don_doff_threshold_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return show_reg(dev, buf, REG16_DON_DOFF_THRESH);
}

/* set don/doff threshold value */
static ssize_t don_doff_threshold_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG16_DON_DOFF_THRESH, 1, 5000);
}

/* show don/doff hysteresis value */
static ssize_t don_doff_hysteresis_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return show_reg(dev, buf, REG_DON_DOFF_HYSTERESIS);
}

/* set don/doff hysteresis value */
static ssize_t don_doff_hysteresis_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_DON_DOFF_HYSTERESIS, 1, 255);
}

/* show IR LED drive value */
static ssize_t led_drive_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_LED_DRIVE);
}

/* set IR LED drive value */
static ssize_t led_drive_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	return store_reg(dev, buf, count, REG_LED_DRIVE, 0, 7);
}

/* calibrate IR LED drive levels */
static ssize_t calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct glasshub_data *glasshub;
	uint32_t sum;
	uint16_t temp;
	uint16_t proxValues[8];
	int i, j;
	int rc;
	uint8_t led_drive = 0x04;
	uint8_t value = 0;
	uint16_t addr;

	glasshub = dev_get_drvdata(dev);
	if (!kstrtou8(buf, 10, &value) && value) {
		mutex_lock(&glasshub->device_lock);
		if (boot_device_l(glasshub)) {
			rc = -ENODEV;
			goto Unlock;
		}

		/* read current drive level */
		_i2c_read_reg8(glasshub, REG_LED_DRIVE, &led_drive);

		/* test all 7 LED levels */
		for (i = 0; i < 8; i++) {

			/* set LED drive level */
			_i2c_write_reg(glasshub, REG_LED_DRIVE, i);

			/* read multiple values and take the mean */
			sum = 0;
			for (j = 0; j < NUM_CALIBRATION_SAMPLES; j++) {

				/* allow charge pump to re-charge */
				msleep(35);

				/* read raw prox value */
				rc = read_prox_raw(glasshub, &temp);
				if (rc) break;

				/* read raw prox value and add to running sum */
				sum += temp;

			}

			/* break on error */
			if (rc) break;

			/* save mean value */
			proxValues[i] = sum / NUM_CALIBRATION_SAMPLES;
		}

		/* write calibration values to flash */
		if (rc == 0) {
			addr = CALIB_ADDRESS;
			for (i = 0; i < 8; i++) {

				/* store calibration data */
				write_glasshub_memory(glasshub, addr++, (uint8_t) proxValues[i]);
				write_glasshub_memory(glasshub, addr++, (uint8_t) (proxValues[i] >> 8));
			}
		}

		/* restore LED drive level */
		_i2c_write_reg(glasshub, REG_LED_DRIVE, led_drive);

Unlock:
		mutex_unlock(&glasshub->device_lock);
	}
	return count;
}

/* read calibration values */
static ssize_t calibration_values_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int rc;
	int i;
	unsigned temp;
	unsigned proxValues[8];
	uint16_t addr;

	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	memset(proxValues, 0, sizeof(proxValues));

	mutex_lock(&glasshub->device_lock);
	if (boot_device_l(glasshub)) goto Error;

	/* read calibration values from flash */
	addr = CALIB_ADDRESS;
	for (i = 0; i < 8; i++) {

		/* read LSB */
		rc = read_glasshub_memory(glasshub, addr++, &temp);
		if (rc) goto Error;
		proxValues[i] = temp;

		/* read MSB */
		rc = read_glasshub_memory(glasshub, addr++, &temp);
		if (rc) goto Error;
		proxValues[i] |= temp << 8;
	}

Error:
	mutex_unlock(&glasshub->device_lock);

	return sprintf(buf, "%u %u %u %u %u %u %u %u\n",
			proxValues[0], proxValues[1],
			proxValues[2], proxValues[3],
			proxValues[4], proxValues[5],
			proxValues[6], proxValues[7]);
}

/* return prox version */
static ssize_t prox_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t partId = 0xff;
	uint8_t seqId = 0xff;
	struct glasshub_data *glasshub = dev_get_drvdata(dev);

	mutex_lock(&glasshub->device_lock);
	if (boot_device_l(glasshub) == 0) {
		_i2c_read_reg8(glasshub, REG_PROX_PART_ID, &partId);
		_i2c_read_reg8(glasshub, REG_PROX_SEQ_ID, &seqId);
	}
	mutex_unlock(&glasshub->device_lock);

	return sprintf(buf, "0x%02x 0x%02x\n", partId, seqId);
}

/* show wink status (also clears the status) */
static ssize_t wink_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_WINK_STATUS);
}

/* show wink enable */
static ssize_t wink_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_ENABLE_WINK);
}

/* enable/disable wink */
static ssize_t wink_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_ENABLE_WINK, 0, 1);
}

/* show wink inhibit period */
static ssize_t wink_inhibit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_WINK_INHIBIT);
}

/* set wink inhibit period */
static ssize_t wink_inhibit_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_WINK_INHIBIT, 1, 255);
}

/* show detector gain */
static ssize_t detector_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_DETECTOR_GAIN);
}

/* set detector gain */
static ssize_t detector_gain_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_DETECTOR_GAIN, 0x01, 0x80);
}

/* show detector bias */
static ssize_t detector_bias_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG16_DETECTOR_BIAS);
}

/* set detector bias */
static ssize_t detector_bias_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG16_DETECTOR_BIAS, 1, 5000);
}

/* show last error code from device */
static ssize_t error_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_ERROR_CODE);
}

/* show debug value */
static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_reg(dev, buf, REG_DEBUG);
}

/* write debug value */
static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_reg(dev, buf, count, REG_DEBUG, 0, 255);
}

/* sysfs node for updating device firmware */
static ssize_t update_fw_data_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count);

static DEVICE_ATTR(update_fw_data, DEV_MODE_WO, NULL, update_fw_data_store);

/* hex converter for parsing .S19 files */
static int convert_hex(const char c, uint32_t *p)
{
	if (!isxdigit(c)) return -1;
	*p = (*p << 4) | (c <= '9' ? c - '0' : tolower(c) - 'a' + 10);
	return 0;
}

/* helper function to get app version */
static int get_app_version_l(struct glasshub_data *glasshub)
{
	int rc;
	uint8_t buffer[2];

	/* get current app version number */
	buffer[0] = CMD_APP_VERSION;
	rc = _i2c_read(glasshub, buffer, 1, buffer, sizeof(buffer));
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s Error getting firmware version: %d\n",
				__FUNCTION__, rc);
	} else {
		dev_info(&glasshub->i2c_client->dev, "Firmware version: %d.%d\n",
				buffer[0], buffer[1]);
		glasshub->app_version_major = buffer[0];
		glasshub->app_version_minor = buffer[1];
	}
	return rc;
}

/* helper function to exit flash programming mode */
static void exit_flash_mode_l(struct glasshub_data *glasshub)
{
	if (glasshub->fw_image) {
		kfree(glasshub->fw_image);
		glasshub->fw_image = NULL;
	}
	device_remove_file(&glasshub->i2c_client->dev, &dev_attr_update_fw_data);
	clear_bit(FLAG_FLASH_MODE, &glasshub->flags);

	/* get app version number */
	get_app_version_l(glasshub);

	/* register run mode sysfs files */
	if (glasshub->app_version_major != BOOTLOADER_FLASHER) {
		register_device_files(glasshub);
	}
}

/* sysfs node to download device firmware to be flashed */
static ssize_t update_fw_data_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	int i;
	int pageNum;

#if DEBUG_FLASH_MODE
	dev_info(&glasshub->i2c_client->dev, "%s Rx SREC %u bytes\n", __FUNCTION__, count);
#endif

	mutex_lock(&glasshub->device_lock);

	for (i = 0; i < count; i++) {

#if DEBUG_FLASH_MODE
		dev_dbg(&glasshub->i2c_client->dev,
				"%s SREC state = %d count = %d input = %c\n",
				__FUNCTION__, glasshub->fw_state, glasshub->fw_count, buf[i]);
#endif

		switch (glasshub->fw_state) {
			case FW_STATE_START:
				if (buf[i] == 'S') {
					glasshub->fw_state = FW_STATE_TYPE;
#if DEBUG_FLASH_MODE
					dev_dbg(&glasshub->i2c_client->dev,
							"%s Start of SREC\n", __FUNCTION__);
#endif
				}
				break;

			case FW_STATE_TYPE:
				if (buf[i] < '0' || buf[i] > '9') {
					glasshub->flash_status = FLASH_ERROR_INVALID_S_RECORD;
					goto err_out;
				}

				/* process only S1-S3 records */
				if (buf[i] >= '1' && buf[i] <= '3') {
#if DEBUG_FLASH_MODE
					dev_dbg(&glasshub->i2c_client->dev,
							"%s SREC type %c\n", __FUNCTION__, buf[i]);
#endif
					glasshub->fw_rec_type = buf[i];
					glasshub->fw_value = 0;
					glasshub->fw_checksum = 0;
					glasshub->fw_state = FW_STATE_COUNT_HI;
				}

				/* ignore other records */
				else {
					glasshub->fw_state = FW_STATE_START;
				}
				break;

			case FW_STATE_ADDR_6:
			case FW_STATE_ADDR_4:
			case FW_STATE_ADDR_2:
				if (convert_hex(buf[i], &glasshub->fw_value)) {
					glasshub->flash_status = FLASH_ERROR_INVALID_HEX_DIGIT;
					goto err_out;
				}
				glasshub->fw_checksum += glasshub->fw_value & 0xff;
				glasshub->fw_count--;
				glasshub->fw_state++;
				break;

			case FW_STATE_COUNT_HI:
			case FW_STATE_ADDR_7:
			case FW_STATE_ADDR_5:
			case FW_STATE_ADDR_3:
			case FW_STATE_ADDR_1:
			case FW_STATE_BYTE_HI:
			case FW_STATE_CHECKSUM_HI:
				if (convert_hex(buf[i], &glasshub->fw_value)) {
					glasshub->flash_status = FLASH_ERROR_INVALID_HEX_DIGIT;
					goto err_out;
				}
				glasshub->fw_state++;
				break;

			case FW_STATE_COUNT_LO:
				if (convert_hex(buf[i], &glasshub->fw_value)) {
					glasshub->flash_status = FLASH_ERROR_INVALID_HEX_DIGIT;
					goto err_out;
				}
				glasshub->fw_checksum += glasshub->fw_value & 0xff;
#if DEBUG_FLASH_MODE
				dev_dbg(&glasshub->i2c_client->dev, "%s SREC byte count %u\n",
						__FUNCTION__, glasshub->fw_value);
#endif

				/* adjust count for address and checksum bytes */
				glasshub->fw_count = glasshub->fw_value;
				glasshub->fw_value = 0;
				glasshub->fw_state = FW_STATE_ADDR_7 + 2 *
					('3' - glasshub->fw_rec_type);
				break;

			case FW_STATE_ADDR_0:
				if (convert_hex(buf[i], &glasshub->fw_value)) {
					glasshub->flash_status = FLASH_ERROR_INVALID_HEX_DIGIT;
					goto err_out;
				}
				glasshub->fw_checksum += glasshub->fw_value & 0xff;
#if DEBUG_FLASH_MODE
				dev_dbg(&glasshub->i2c_client->dev, "%s SREC address %04xh\n",
						__FUNCTION__, glasshub->fw_value);
#endif
				glasshub->fw_index = glasshub->fw_value - FIRMWARE_BASE_ADDRESS;
				glasshub->fw_count--;
				glasshub->fw_value = 0;
				glasshub->fw_state = FW_STATE_BYTE_HI;
				break;

			case FW_STATE_BYTE_LO:
				if (convert_hex(buf[i], &glasshub->fw_value)) {
					glasshub->flash_status = FLASH_ERROR_INVALID_HEX_DIGIT;
					goto err_out;
				}
				glasshub->fw_checksum += glasshub->fw_value & 0xff;

				/* validate address */
				if (glasshub->fw_index < 0 ||
						glasshub->fw_index > FIRMWARE_TOTAL_SIZE) {
					dev_err(&glasshub->i2c_client->dev,
							"%s Address out of range: address %u count=%u\n",
							__FUNCTION__, glasshub->fw_value,
							glasshub->fw_count);
					glasshub->flash_status = FLASH_ERROR_ADDRESS_RANGE;
					goto err_out;
				}

				/* store byte in image buffer */
				glasshub->fw_image[glasshub->fw_index] = glasshub->fw_value;
				glasshub->fw_value = 0;
				pageNum = glasshub->fw_index / FIRMWARE_PAGE_SIZE;
				glasshub->fw_dirty[pageNum / 32] |= (1 << (pageNum & 31));
				glasshub->fw_index++;
				glasshub->fw_state = FW_STATE_BYTE_HI;

				/* check for end of data */
				if (--glasshub->fw_count == 1) {
#if DEBUG_FLASH_MODE
					dev_dbg(&glasshub->i2c_client->dev,
							"%s SREC all bytes received\n",
							__FUNCTION__);
#endif
					glasshub->fw_state = FW_STATE_CHECKSUM_HI;
				}
				break;

			case FW_STATE_CHECKSUM_LO:
				if (convert_hex(buf[i], &glasshub->fw_value)) {
					glasshub->flash_status = FLASH_ERROR_INVALID_HEX_DIGIT;
					goto err_out;
				}
				if (glasshub->fw_value != (~glasshub->fw_checksum & 0xff)) {
					dev_err(&glasshub->i2c_client->dev,
							"%s SREC checksum mismatch %02xh != %02xh\n",
							__FUNCTION__,
							(unsigned)~glasshub->fw_checksum & 0xff,
							(unsigned)glasshub->fw_value);
					glasshub->flash_status = FLASH_ERROR_SREC_CHECKSUM_ERROR;
					goto err_out;
				}
#if DEBUG_FLASH_MODE
				dev_dbg(&glasshub->i2c_client->dev, "%s SREC checksum OK\n",
						__FUNCTION__);
#endif
				glasshub->fw_state = FW_STATE_START;
				break;
		}
	}

	mutex_unlock(&glasshub->device_lock);
	return count;

err_out:
	exit_flash_mode_l(glasshub);
	mutex_unlock(&glasshub->device_lock);
	dev_err(&glasshub->i2c_client->dev, "%s Not an SREC\n", __FUNCTION__);
	return -EINVAL;
}

/* sysfs node to enter/exit flash programming mode */
static ssize_t update_fw_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long value = 0;
	int rc = 0;
	int bootflasher = 0;
	int old_boot;
	struct glasshub_data *glasshub = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &value)) {
		goto err_out;
	}

#if DEBUG_FLASH_MODE
	dev_info(&glasshub->i2c_client->dev, "%s update_fw_enable = %lu\n", __FUNCTION__, value);
#endif

	mutex_lock(&glasshub->device_lock);

	/* bootloader version 3 does not support checksum */
	old_boot = (glasshub->bootloader_version < 4) ? 1 : 0;

	/* handle special bootflasher case */
	if ((value == BOOTLOADER_FLASHER) && (glasshub->app_version_major == BOOTLOADER_FLASHER)) {
		bootflasher = 1;
		old_boot = 0;
		value = 0;
	}

	/* enable firmware flash */
	if (value) {
		if (!test_bit(FLAG_FLASH_MODE, &glasshub->flags)) {
			/* allocate memory and clear dirty bits */
			if (!glasshub->fw_image) {
				glasshub->fw_image = kzalloc(FIRMWARE_TOTAL_SIZE, GFP_USER);
				if (!glasshub->fw_image) {
					dev_err(&glasshub->i2c_client->dev,
							"%s failed to allocate memory for firmware image\n",
							__FUNCTION__);
					goto unlock;
				}
				memset(glasshub->fw_dirty, 0, sizeof(glasshub->fw_dirty));
			}

			/* unregister device files */
			unregister_device_files(glasshub);

			rc = device_create_file(&glasshub->i2c_client->dev,
					&dev_attr_update_fw_data);
			if (rc) goto unlock;

			glasshub->fw_state = FW_STATE_START;
			set_bit(FLAG_FLASH_MODE, &glasshub->flags);
			glasshub->flash_status = FLASH_STATUS_READY;
		}
	} else {

		/* exit flash, write code to device */
		if (test_bit(FLAG_FLASH_MODE, &glasshub->flags)) {

			/* put device into bootloader mode */
			rc = reset_device_l(glasshub, 0);
			if (rc)  {
				glasshub->flash_status = FLASH_ERROR_DEVICE_RESET_ERROR;
				goto ExitFlashMode;
			}

			/* if flashing boot code, boot into application code */
			if (bootflasher) {
				rc = boot_device_l(glasshub);
				if (rc) {
					glasshub->flash_status = FLASH_ERROR_DEVICE_RESET_ERROR;
					goto ExitFlashMode;
				}
			}

			/* here's where we flash the image if it has dirty bits */
			if (glasshub->fw_image) {
				int pagesFlashed = 0;
				int page;
				uint8_t checksum;

#if DEBUG_FLASH_MODE
				dev_info(&glasshub->i2c_client->dev,
						"Update pages: %08x %08x %08x %08x\n",
						glasshub->fw_dirty[0],
						glasshub->fw_dirty[1],
						glasshub->fw_dirty[2],
						glasshub->fw_dirty[3]);
#endif

				for (page = 0; page < FIRMWARE_TOTAL_PAGES; page++) {
					/* flash only dirty pages */
					if (glasshub->fw_dirty[page / 32] &
							(1 << (page & 31))) {
						uint8_t buffer[FIRMWARE_PAGE_SIZE+3];
						int i, j;

						/* initalize command and page number */
						buffer[0] = CMD_FLASH;
						buffer[1] = page;

						/* seed checksum */
						checksum = 0xa5;

						/* copy firmware into buffer */
						for (i = 2, j = page * FIRMWARE_PAGE_SIZE;
								i < FIRMWARE_PAGE_SIZE + 2;
								i++, j++) {
							buffer[i] = glasshub->fw_image[j];
							checksum = (checksum << 1) ^ buffer[i] ^ (checksum >> 7);
						}

						/* add checksum to buffer */
						buffer[66] = checksum;

#if DEBUG_FLASH_MODE
						dev_info(&glasshub->i2c_client->dev,
								"%s: Flash page %d\n",
								__FUNCTION__, page);
#endif
						/* don't send checksum for older bootloader version */
						rc = _i2c_write_mult(glasshub, buffer,
								sizeof(buffer) - old_boot);
						if (rc) {
							dev_err(&glasshub->i2c_client->dev,
									"%s Unable to flash glasshub device\n",
									__FUNCTION__);
							glasshub->flash_status = FLASH_ERROR_I2C_DEVICE_COMM;
							goto ExitFlashMode;
						}

						/* new bootloader: check status for error */
						if (!old_boot) {
							buffer[0] = CMD_FLASH_STATUS;
							rc = _i2c_read(glasshub, buffer, 1, buffer, 1);
							if (rc) {
								dev_err(&glasshub->i2c_client->dev,
										"%s Error reading flash status\n",
										__FUNCTION__);
								glasshub->flash_status = FLASH_ERROR_I2C_DEVICE_COMM;
								goto ExitFlashMode;
							} else if (buffer[0] != 0) {
								dev_err(&glasshub->i2c_client->dev,
										"%s Device rejected packet: %u\n",
										__FUNCTION__, buffer[0]);
								glasshub->flash_status = FLASH_ERROR_DEV_REJECTED_PKT;
								goto ExitFlashMode;
							}
						}
						++pagesFlashed;
					}
				}
				dev_info(&glasshub->i2c_client->dev,
						"%s: %d code page(s) flashed\n",
						__FUNCTION__, pagesFlashed);
				glasshub->flash_status = pagesFlashed ? FLASH_STATUS_OK : FLASH_ERROR_NO_PAGES_FLASHED;
			}
ExitFlashMode:
			if (bootflasher) {
				uint8_t buffer[1];

				/* put device back into bootloader mode */
				rc = reset_device_l(glasshub, 1);
				if (rc)  {
					glasshub->flash_status = FLASH_ERROR_DEVICE_RESET_ERROR;
					goto ExitFlashMode;
				}

				/* get new bootloader version */
				buffer[0] = CMD_BOOTLOADER_VERSION;
				rc = _i2c_read(glasshub, buffer, 1, buffer, 1);
				if (rc) {
					glasshub->flash_status = FLASH_ERROR_DEVICE_RESET_ERROR;
					goto ExitFlashMode;
				}
				glasshub->bootloader_version = buffer[0];
			}
			clear_bit(FLAG_FLASH_MODE, &glasshub->flags);
		}
	}

unlock:
	if (!test_bit(FLAG_FLASH_MODE, &glasshub->flags) && glasshub->fw_image) {
		exit_flash_mode_l(glasshub);
	}
	mutex_unlock(&glasshub->device_lock);
err_out:
	return rc < 0 ? rc : count;
}

/* show state of firmware update flag */
static ssize_t update_fw_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	enable = test_bit(FLAG_FLASH_MODE, &glasshub->flags) ? 1 : 0;
	return sprintf(buf, "%d\n", enable);
}

/* show state of IRQ */
static ssize_t irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", gpio_get_value(glasshub->pdata->gpio_int_no));
}

/* show state of flash process */
static ssize_t flash_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int temp;
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	temp = glasshub->flash_status;
	glasshub->flash_status = 0;
	return sprintf(buf, "%d\n", temp);
}

/* show count of received samples */
static ssize_t sample_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct glasshub_data *glasshub = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", glasshub->sample_count);
}

static DEVICE_ATTR(update_fw_enable, DEV_MODE_RW, update_fw_enable_show, update_fw_enable_store);
static DEVICE_ATTR(version, DEV_MODE_RO, version_show, NULL);
static DEVICE_ATTR(bootloader_version, DEV_MODE_RO, bootloader_version_show, NULL);
static DEVICE_ATTR(passthru_enable, DEV_MODE_RW, passthru_enable_show, passthru_enable_store);
static DEVICE_ATTR(proxraw, DEV_MODE_RO, proxraw_show, NULL);
static DEVICE_ATTR(proxmin, DEV_MODE_RO, proxmin_show, NULL);
static DEVICE_ATTR(vis, DEV_MODE_RO, vis_show, NULL);
static DEVICE_ATTR(ir, DEV_MODE_RO, ir_show, NULL);
static DEVICE_ATTR(don_doff_enable, DEV_MODE_RW, don_doff_enable_show, don_doff_enable_store);
static DEVICE_ATTR(don_doff, DEV_MODE_RO, don_doff_show, NULL);
static DEVICE_ATTR(don_doff_threshold, DEV_MODE_RW, don_doff_threshold_show, don_doff_threshold_store);
static DEVICE_ATTR(don_doff_hysteresis, DEV_MODE_RW, don_doff_hysteresis_show, don_doff_hysteresis_store);
static DEVICE_ATTR(led_drive, DEV_MODE_RW, led_drive_show, led_drive_store);
static DEVICE_ATTR(calibrate, DEV_MODE_WO, NULL, calibrate_store);
static DEVICE_ATTR(calibration_values, DEV_MODE_RO, calibration_values_show, NULL);
static DEVICE_ATTR(prox_version, DEV_MODE_RO, prox_version_show, NULL);
static DEVICE_ATTR(wink, DEV_MODE_RO, wink_show, NULL);
static DEVICE_ATTR(wink_enable, DEV_MODE_RW, wink_enable_show, wink_enable_store);
static DEVICE_ATTR(wink_inhibit, DEV_MODE_RW, wink_inhibit_show, wink_inhibit_store);
static DEVICE_ATTR(detector_gain, DEV_MODE_RW, detector_gain_show, detector_gain_store);
static DEVICE_ATTR(detector_bias, DEV_MODE_RW, detector_bias_show, detector_bias_store);
static DEVICE_ATTR(debug, DEV_MODE_RW, debug_show, debug_store);
static DEVICE_ATTR(error_code, DEV_MODE_RO, error_code_show, NULL);
static DEVICE_ATTR(irq, DEV_MODE_RO, irq_show, NULL);
static DEVICE_ATTR(flash_status, DEV_MODE_RO, flash_status_show, NULL);
static DEVICE_ATTR(sample_count, DEV_MODE_RO, sample_count_show, NULL);

static struct attribute *attrs[] = {
	&dev_attr_passthru_enable.attr,
	&dev_attr_proxraw.attr,
	&dev_attr_proxmin.attr,
	&dev_attr_ir.attr,
	&dev_attr_vis.attr,
	&dev_attr_don_doff_enable.attr,
	&dev_attr_don_doff.attr,
	&dev_attr_don_doff_threshold.attr,
	&dev_attr_don_doff_hysteresis.attr,
	&dev_attr_led_drive.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_calibration_values.attr,
	&dev_attr_prox_version.attr,
	&dev_attr_wink.attr,
	&dev_attr_wink_enable.attr,
	&dev_attr_wink_inhibit.attr,
	&dev_attr_detector_gain.attr,
	&dev_attr_detector_bias.attr,
	&dev_attr_debug.attr,
	&dev_attr_error_code.attr,
	&dev_attr_irq.attr,
	&dev_attr_sample_count.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct attribute *bootmode_attrs[] = {
	&dev_attr_bootloader_version.attr,
	&dev_attr_version.attr,
	&dev_attr_update_fw_enable.attr,
	&dev_attr_flash_status.attr,
	NULL
};

static struct attribute_group bootmode_attr_group = {
	.attrs = bootmode_attrs,
};

/* register prox device */
static int register_ps_device(struct glasshub_data *glasshub)
{
	int rc;

	glasshub->ps_input_dev = input_allocate_device();
	if (!glasshub->ps_input_dev) {
		dev_err(&glasshub->i2c_client->dev, "%s: Failed to allocate prox input device\n",
				__func__);
		return -ENOMEM;
	}

	glasshub->ps_input_dev->name = "glasshub_ps";
	set_bit(EV_ABS, glasshub->ps_input_dev->evbit);
	input_set_abs_params(glasshub->ps_input_dev, ABS_DISTANCE, PS_MIN_VALUE, PS_MAX_VALUE, 0, 0);

	rc = input_register_device(glasshub->ps_input_dev);
	if (rc < 0) {
		dev_err(&glasshub->i2c_client->dev, "%s: Failed to register prox input device \n",
				__func__);
		goto free_device;
	}
	return rc;

free_device:
	input_free_device(glasshub->ps_input_dev);

	return rc;
}

/* unregister prox device */
static void unregister_ps_device(struct glasshub_data *glasshub)
{
	input_unregister_device(glasshub->ps_input_dev);
	input_free_device(glasshub->ps_input_dev);
}

/* register device files */
static int register_device_files(struct glasshub_data *glasshub)
{
	int rc = 0;
	unsigned app_version;

	/* if special boot flasher, don't register sysfs files */
	if (glasshub->app_version_major == BOOTLOADER_FLASHER) return rc;

	/* check app code version */
	app_version = ((unsigned) glasshub->app_version_major << 8) | glasshub->app_version_minor;

	/* make sure we have a supported firmware build on the MCU */
	if (app_version < MINIMUM_MCU_VERSION) {
		dev_info(&glasshub->i2c_client->dev,
				"%s: WARNING: MCU application code is down-rev: %u.%u\n",
				__FUNCTION__,
				glasshub->app_version_major,
				glasshub->app_version_minor);
		dev_info(&glasshub->i2c_client->dev,
				"%s: All functions except firmware update are disabled\n",
				__FUNCTION__);
		goto Exit;
	}

	/* are sysfs files already created? */
	if (test_and_set_bit(FLAG_SYSFS_CREATED, &glasshub->flags)) goto Exit;

	/* create attributes */
	rc = sysfs_create_group(&glasshub->i2c_client->dev.kobj, &attr_group);
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s Unable to create sysfs class files\n",
				__FUNCTION__);
	}

	/* register input devices */
	register_ps_device(glasshub);

Exit:
	return rc;
}

/* unregister device files */
static void unregister_device_files(struct glasshub_data *glasshub)
{
	/* are sysfs files created? */
	if (test_and_clear_bit(FLAG_SYSFS_CREATED, &glasshub->flags)) {
		sysfs_remove_group(&glasshub->i2c_client->dev.kobj,&attr_group);
		unregister_ps_device(glasshub);
	}
}

/* prox sensor open fops */
static int glasshub_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client;

	if (atomic_read(&glasshub_opened)) {
		return -EBUSY;
	}
	atomic_set(&glasshub_opened, 1);

	file->private_data = (void*)glasshub_private;
	client = glasshub_private->i2c_client;

	/* Set the read pointer equal to the write pointer */
	if (mutex_lock_interruptible(&glasshub_user_lock)) {
		dev_err(&client->dev, "%s: Unable to set read pointer\n", __func__);
		return -EAGAIN;
	}

	glasshub_user_rd_ptr = glasshub_user_wr_ptr;
	mutex_unlock(&glasshub_user_lock);
	return 0;
}

/* prox sensor release fops */
static int glasshub_release(struct inode *inode, struct file *file)
{
	struct glasshub_data *glasshub = (struct glasshub_data *)file->private_data;
	struct i2c_client *client = glasshub->i2c_client;

	if (!atomic_read(&glasshub_opened)) {
		dev_err(&client->dev, "%s: Device has not been opened\n", __func__);
		return -EBUSY;
	}
	atomic_set(&glasshub_opened, 0);
	return 0;
}

/* prox sensor IOCTL */
static long glasshub_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return -ENOSYS;
}

/* prox sensor read function */
static ssize_t glasshub_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int rc = 0;
	struct glasshub_data *glasshub = (struct glasshub_data *)file->private_data;
	struct i2c_client *client = glasshub->i2c_client;

	if (count == 0)
		return 0;

	if (count < sizeof(struct glasshub_data_user))
		return -EINVAL;

	if (count > PROX_QUEUE_SZ * sizeof(struct glasshub_data_user))
		count = PROX_QUEUE_SZ * sizeof(struct glasshub_data_user);

	if (mutex_lock_interruptible(&glasshub_user_lock)) {
			dev_err(&client->dev, "%s: Unable to acquire user read lock\n", __func__);
		return -EAGAIN;
	}

	if (glasshub_user_rd_ptr == glasshub_user_wr_ptr) {
		mutex_unlock(&glasshub_user_lock);
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		wait_event_interruptible(glasshub_read_wait,
				glasshub_user_rd_ptr != glasshub_user_wr_ptr);
		if (mutex_lock_interruptible(&glasshub_user_lock)) {
			dev_err(&client->dev, "%s: Unable to acquire user read lock after sleep\n",
					__func__);
			return -EAGAIN;
		}
	}

	while (glasshub_user_rd_ptr != glasshub_user_wr_ptr && count >=
			sizeof(struct glasshub_data_user)) {
		if (copy_to_user(buf, glasshub_user_rd_ptr,
						 sizeof(struct glasshub_data_user))) {
			rc = -EFAULT;
			break;
		}
		rc += sizeof(struct glasshub_data_user);
		buf += sizeof(struct glasshub_data_user);
		count -= sizeof(struct glasshub_data_user);

		glasshub_user_rd_ptr++;
		if (glasshub_user_rd_ptr == (glasshub_fifo_buffer + PROX_QUEUE_SZ)) {
			glasshub_user_rd_ptr = glasshub_fifo_buffer;
		}
	}

	mutex_unlock(&glasshub_user_lock);
	return rc;
}

static const struct file_operations glasshub_fops = {
	.owner = THIS_MODULE,
	.open = glasshub_open,
	.release = glasshub_release,
	.unlocked_ioctl = glasshub_ioctl,
	.read = glasshub_read,
};

struct miscdevice glasshub_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "glasshub",
	.fops = &glasshub_fops
};

static int glasshub_setup(struct glasshub_data *glasshub) {
	int rc = 0;
	uint8_t buffer;

	/* force glass hub reset */
	rc = reset_device_l(glasshub, 1);

	/* get bootloader version */
	buffer = CMD_BOOTLOADER_VERSION;
	rc = _i2c_read(glasshub, &buffer, sizeof(buffer), &buffer, sizeof(buffer));
	if (rc) goto err_out;
	glasshub->bootloader_version = buffer;

	/* get app version number */
	rc = get_app_version_l(glasshub);
	if (rc) goto err_out;

	/* check bootloader version */
	/* TODO: Abort on anything less than V3 once we retire Joey devices */
	if (glasshub->bootloader_version < 3) {
		dev_info(&glasshub->i2c_client->dev,
				"%s: WARNING: MCU bootloader is down-rev: %u\n",
				__FUNCTION__, glasshub->bootloader_version);

		/* this version is completely borked */
		if (glasshub->bootloader_version == 2) {
			rc = -EIO;
			goto err_out;
		}
	}

	/* request IRQ */
	rc = request_threaded_irq(glasshub->pdata->irq, glasshub_irq_handler,
			glasshub_threaded_irq_handler, IRQF_TRIGGER_FALLING,
			"glasshub_irq", glasshub);
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s request_threaded_irq failed\n",
				__FUNCTION__);
	}

	/* Allow this interrupt to wake the system */
	rc = irq_set_irq_wake(glasshub->pdata->irq, 1);
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s irq_set_irq_wake failed\n", __FUNCTION__);
	}

err_out:
	return rc;
}

static int __devinit glasshub_probe(struct i2c_client *i2c_client,
		const struct i2c_device_id *id)
{
	struct glasshub_data *glasshub = NULL;
	int rc;

	glasshub = kzalloc(sizeof(struct glasshub_data), GFP_KERNEL);
	if (!glasshub)
	{
		dev_err(&i2c_client->dev, "%s: Unable to allocate memory for driver structure\n",
				__FUNCTION__);
		return -ENOMEM;
	}

	/* initialize data structure */
	glasshub->i2c_client = i2c_client;
	glasshub->flags = 0;
	mutex_init(&glasshub->device_lock);

	/* Set platform defaults */
	glasshub->pdata = (const struct glasshub_platform_data *)i2c_client->dev.platform_data;
	if (!glasshub->pdata) {
		dev_err(&i2c_client->dev, "%s: Platform data has not been set\n", __FUNCTION__);
		goto err_out;
	}

	/* setup the device */
	dev_info(&i2c_client->dev, "%s: Probing glasshub...\n", __FUNCTION__);
	if (glasshub_setup(glasshub)) {
		goto err_out;
	}
	dev_info(&i2c_client->dev, "%s: Probe successful\n", __FUNCTION__);

	/* store driver data into device private structure */
	dev_set_drvdata(&i2c_client->dev, glasshub);
	glasshub_private = glasshub;

	/* create bootmode sysfs files */
	rc = sysfs_create_group(&glasshub->i2c_client->dev.kobj, &bootmode_attr_group);
	if (rc) {
		dev_err(&glasshub->i2c_client->dev, "%s Unable to create sysfs class files\n",
				__FUNCTION__);
		goto err_out;
	}

	/* create sysfs files unless running the special bootflasher app */
	if (glasshub->app_version_major != BOOTLOADER_FLASHER) {
		register_device_files(glasshub);
	}

	/* set misc driver */
	mutex_init(&glasshub_user_lock);
	glasshub_user_wr_ptr = glasshub_fifo_buffer;
	glasshub_user_rd_ptr = glasshub_fifo_buffer;
	misc_register(&glasshub_misc);

	return 0;

err_out:
	kfree(glasshub);
	return -ENODEV;
}

static const struct i2c_device_id glasshub_id[] = {
	{ DEVICE_NAME, 0 },
};

static struct i2c_driver glasshub_driver = {
	.probe = glasshub_probe,
	.id_table = glasshub_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};

static int __init glasshub_init(void)
{
	return i2c_add_driver(&glasshub_driver);
}

static void __exit glasshub_exit(void)
{
	i2c_del_driver(&glasshub_driver);
}

module_init(glasshub_init);
module_exit(glasshub_exit);

MODULE_AUTHOR("davidsparks@google.com");
MODULE_DESCRIPTION("Glass Hub Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
