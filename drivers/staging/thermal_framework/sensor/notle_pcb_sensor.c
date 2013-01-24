/*
 * PCB temperature sensor driver file
 *
 *  Copyright (C) 2012 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Patterened after TI's pcb_temp_sensor driver.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/thermal_framework.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/types.h>

#include <plat/common.h>
#include <linux/i2c/twl6030-gpadc.h>

#define PCB_REPORT_DELAY_MS	1000

/*
 * pcb_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 */
struct pcb_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
};
struct pcb_temp_sensor *temp_sensor;
struct pcb_sens notle_pcb_sensor;

#define TWL6030_ADC_START_VALUE 0
#define TWL6030_ADC_END_VALUE   1023
#define TWL6030_GPADC_CHANNEL     6

// TODO:(rocky) Should verify this table with actual thermal measurements.

/*
 * adc_to_temp table produced by this 'C' code:

#include <stdio.h>

// Resistance in kohms for temps from 125 to -40 every 5 degrees
// From NTC Thermistors data sheet for NCP**WF104 part
float resistance[] = {
2.522,
2.916,          // 120
3.380,
3.934,
4.594,
5.384,
6.337,
7.481,
8.873,
10.566,         // 80
12.635,
15.184,
18.323,
22.224,
27.091,
33.195,
40.904,
50.677,         // 40
63.167,
79.222,
100.000,
127.080,
162.651,
209.710,
272.500,
357.012,        // 0
471.632,
628.988,
846.579,
1151.037,
1581.881,
2197.225,
3088.599,
4397.119,       // -40
};

float temp(int index) {
  return 125.0 - 5.0 * index;
}

float adc_output(int index) {
  float res = resistance[index];
  // 31.6 kohm resister in series with the NTC part
  float normalized_voltage = res / (res + 31.6); // 0.0 => 1.0
  // 10 bit adc converter
  return normalized_voltage * 1024;
}

void output(int temp) {
  static int count = 0;
  // Output table values in mC
  if (count == 0) {
    printf("\t");
  } else {
    printf(" ");
  }
  printf("%d,", temp * 1000);
  if (++count >= 10) {
    count = 0;
    printf("\n");
  }
}

void main() {
  float adc_low = adc_output(0);
  float adc_high = adc_output(1);
  float reference_temp = 125.0;   // Temp corresponding to adc_low output
  int interpolated_temp;
  int next_index = 2;

  printf("static int adc_to_temp[] = {\n");
  // Generate temperature estimate for each adc output
  for (int adc = 0; adc < 1024; ++adc) {
    // Don't extrapolate
    if (adc < adc_low) {
      output(125);
      continue;
    }

    if (adc > adc_high) {
      // Don't extrapolate
      if (temp(next_index) < -40) {
        output(-40);
        continue;
      }
      adc_low = adc_high;
      adc_high = adc_output(next_index++);
      reference_temp = reference_temp - 5.0;
    }

    // Linearly interpolate between two calculated values
    interpolated_temp = reference_temp - (adc - adc_low) / (adc_high - adc_low) * 5.0 + 0.5;
    output(interpolated_temp);
  }
  printf("\n};\n");
}

*/
/*
 * Temperature values in milli degrees celsius ADC code values from 0 to 1023
 */
static int adc_to_temp[] = {
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000, 125000, 125000, 125000, 124000, 124000, 123000,
	123000, 123000, 122000, 122000, 121000, 121000, 120000, 120000, 119000, 119000,
	119000, 118000, 118000, 117000, 117000, 117000, 116000, 116000, 115000, 115000,
	115000, 114000, 114000, 114000, 113000, 113000, 113000, 112000, 112000, 112000,
	111000, 111000, 110000, 110000, 110000, 110000, 109000, 109000, 109000, 108000,
	108000, 108000, 107000, 107000, 107000, 106000, 106000, 106000, 106000, 105000,
	105000, 105000, 104000, 104000, 104000, 104000, 103000, 103000, 103000, 103000,
	102000, 102000, 102000, 102000, 101000, 101000, 101000, 101000, 100000, 100000,
	100000, 100000, 99000, 99000, 99000, 99000, 98000, 98000, 98000, 98000,
	98000, 97000, 97000, 97000, 97000, 96000, 96000, 96000, 96000, 95000,
	95000, 95000, 95000, 95000, 94000, 94000, 94000, 94000, 94000, 93000,
	93000, 93000, 93000, 93000, 92000, 92000, 92000, 92000, 92000, 91000,
	91000, 91000, 91000, 91000, 90000, 90000, 90000, 90000, 90000, 89000,
	89000, 89000, 89000, 89000, 89000, 88000, 88000, 88000, 88000, 88000,
	88000, 87000, 87000, 87000, 87000, 87000, 86000, 86000, 86000, 86000,
	86000, 86000, 85000, 85000, 85000, 85000, 85000, 85000, 84000, 84000,
	84000, 84000, 84000, 84000, 84000, 83000, 83000, 83000, 83000, 83000,
	83000, 82000, 82000, 82000, 82000, 82000, 82000, 81000, 81000, 81000,
	81000, 81000, 81000, 81000, 80000, 80000, 80000, 80000, 80000, 80000,
	80000, 79000, 79000, 79000, 79000, 79000, 79000, 79000, 78000, 78000,
	78000, 78000, 78000, 78000, 78000, 77000, 77000, 77000, 77000, 77000,
	77000, 77000, 76000, 76000, 76000, 76000, 76000, 76000, 76000, 75000,
	75000, 75000, 75000, 75000, 75000, 75000, 75000, 74000, 74000, 74000,
	74000, 74000, 74000, 74000, 74000, 73000, 73000, 73000, 73000, 73000,
	73000, 73000, 73000, 72000, 72000, 72000, 72000, 72000, 72000, 72000,
	72000, 71000, 71000, 71000, 71000, 71000, 71000, 71000, 71000, 70000,
	70000, 70000, 70000, 70000, 70000, 70000, 70000, 69000, 69000, 69000,
	69000, 69000, 69000, 69000, 69000, 69000, 68000, 68000, 68000, 68000,
	68000, 68000, 68000, 68000, 68000, 67000, 67000, 67000, 67000, 67000,
	67000, 67000, 67000, 66000, 66000, 66000, 66000, 66000, 66000, 66000,
	66000, 66000, 65000, 65000, 65000, 65000, 65000, 65000, 65000, 65000,
	65000, 64000, 64000, 64000, 64000, 64000, 64000, 64000, 64000, 64000,
	63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000,
	62000, 62000, 62000, 62000, 62000, 62000, 62000, 62000, 62000, 61000,
	61000, 61000, 61000, 61000, 61000, 61000, 61000, 61000, 61000, 60000,
	60000, 60000, 60000, 60000, 60000, 60000, 60000, 60000, 59000, 59000,
	59000, 59000, 59000, 59000, 59000, 59000, 59000, 59000, 58000, 58000,
	58000, 58000, 58000, 58000, 58000, 58000, 58000, 58000, 57000, 57000,
	57000, 57000, 57000, 57000, 57000, 57000, 57000, 57000, 56000, 56000,
	56000, 56000, 56000, 56000, 56000, 56000, 56000, 56000, 55000, 55000,
	55000, 55000, 55000, 55000, 55000, 55000, 55000, 55000, 54000, 54000,
	54000, 54000, 54000, 54000, 54000, 54000, 54000, 54000, 54000, 53000,
	53000, 53000, 53000, 53000, 53000, 53000, 53000, 53000, 53000, 52000,
	52000, 52000, 52000, 52000, 52000, 52000, 52000, 52000, 52000, 52000,
	51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000,
	50000, 50000, 50000, 50000, 50000, 50000, 50000, 50000, 50000, 50000,
	49000, 49000, 49000, 49000, 49000, 49000, 49000, 49000, 49000, 49000,
	49000, 48000, 48000, 48000, 48000, 48000, 48000, 48000, 48000, 48000,
	48000, 48000, 47000, 47000, 47000, 47000, 47000, 47000, 47000, 47000,
	47000, 47000, 46000, 46000, 46000, 46000, 46000, 46000, 46000, 46000,
	46000, 46000, 46000, 45000, 45000, 45000, 45000, 45000, 45000, 45000,
	45000, 45000, 45000, 45000, 44000, 44000, 44000, 44000, 44000, 44000,
	44000, 44000, 44000, 44000, 43000, 43000, 43000, 43000, 43000, 43000,
	43000, 43000, 43000, 43000, 43000, 42000, 42000, 42000, 42000, 42000,
	42000, 42000, 42000, 42000, 42000, 41000, 41000, 41000, 41000, 41000,
	41000, 41000, 41000, 41000, 41000, 41000, 40000, 40000, 40000, 40000,
	40000, 40000, 40000, 40000, 40000, 40000, 39000, 39000, 39000, 39000,
	39000, 39000, 39000, 39000, 39000, 39000, 39000, 38000, 38000, 38000,
	38000, 38000, 38000, 38000, 38000, 38000, 38000, 37000, 37000, 37000,
	37000, 37000, 37000, 37000, 37000, 37000, 37000, 36000, 36000, 36000,
	36000, 36000, 36000, 36000, 36000, 36000, 36000, 36000, 35000, 35000,
	35000, 35000, 35000, 35000, 35000, 35000, 35000, 35000, 34000, 34000,
	34000, 34000, 34000, 34000, 34000, 34000, 34000, 34000, 33000, 33000,
	33000, 33000, 33000, 33000, 33000, 33000, 33000, 33000, 32000, 32000,
	32000, 32000, 32000, 32000, 32000, 32000, 32000, 32000, 31000, 31000,
	31000, 31000, 31000, 31000, 31000, 31000, 31000, 31000, 30000, 30000,
	30000, 30000, 30000, 30000, 30000, 30000, 30000, 29000, 29000, 29000,
	29000, 29000, 29000, 29000, 29000, 29000, 28000, 28000, 28000, 28000,
	28000, 28000, 28000, 28000, 28000, 28000, 27000, 27000, 27000, 27000,
	27000, 27000, 27000, 27000, 27000, 26000, 26000, 26000, 26000, 26000,
	26000, 26000, 26000, 26000, 25000, 25000, 25000, 25000, 25000, 25000,
	25000, 25000, 25000, 24000, 24000, 24000, 24000, 24000, 24000, 24000,
	24000, 23000, 23000, 23000, 23000, 23000, 23000, 23000, 23000, 23000,
	22000, 22000, 22000, 22000, 22000, 22000, 22000, 22000, 21000, 21000,
	21000, 21000, 21000, 21000, 21000, 21000, 20000, 20000, 20000, 20000,
	20000, 20000, 20000, 20000, 19000, 19000, 19000, 19000, 19000, 19000,
	19000, 19000, 18000, 18000, 18000, 18000, 18000, 18000, 18000, 17000,
	17000, 17000, 17000, 17000, 17000, 17000, 17000, 16000, 16000, 16000,
	16000, 16000, 16000, 16000, 15000, 15000, 15000, 15000, 15000, 15000,
	15000, 14000, 14000, 14000, 14000, 14000, 14000, 14000, 13000, 13000,
	13000, 13000, 13000, 13000, 12000, 12000, 12000, 12000, 12000, 12000,
	12000, 11000, 11000, 11000, 11000, 11000, 11000, 10000, 10000, 10000,
	10000, 10000, 10000, 9000, 9000, 9000, 9000, 9000, 9000, 8000,
	8000, 8000, 8000, 8000, 7000, 7000, 7000, 7000, 7000, 7000,
	6000, 6000, 6000, 6000, 6000, 5000, 5000, 5000, 5000, 5000,
	4000, 4000, 4000, 4000, 4000, 3000, 3000, 3000, 3000, 3000,
	2000, 2000, 2000, 2000, 1000, 1000, 1000, 1000, 1000, 0,
	0, 0, 0, 0, 0, 0, 0, -1000, -1000, -1000,
	-1000, -2000, -2000, -2000, -2000, -3000, -3000, -3000, -4000, -4000,
	-4000, -4000, -5000, -5000, -5000, -6000, -6000, -6000, -7000, -7000,
	-7000, -8000, -8000, -8000, -9000, -9000, -9000, -10000, -10000, -11000,
	-11000, -11000, -12000, -12000, -13000, -13000, -14000, -14000, -14000, -15000,
	-16000, -16000, -17000, -17000, -18000, -18000, -19000, -19000, -20000, -21000,
	-21000, -22000, -23000, -23000, -24000, -25000, -26000, -27000, -28000, -29000,
	-30000, -31000, -32000, -33000, -35000, -36000, -38000, -40000, -40000, -40000,
	-40000, -40000, -40000, -40000,
};

static int adc_to_temp_conversion(int adc_val)
{
	// We see values > TWL6030_ADC_END_VALUE with very cold temps.
	// TODO:rocky
	// We should really calibrate the table with measured temps.
	if (adc_val > TWL6030_ADC_END_VALUE)
		adc_val = TWL6030_ADC_END_VALUE;
	if (adc_val < TWL6030_ADC_START_VALUE) {
		pr_err("%s:Temp read is invalid %i\n", __func__, adc_val);
		return -EINVAL;
	}

	return adc_to_temp[adc_val];
}

static int pcb_read_current_temp(void)
{
	int temp = 0;
	struct twl6030_gpadc_request req;
	int ret;

	req.channels = (1 << TWL6030_GPADC_CHANNEL);
	req.method = TWL6030_GPADC_SW2;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0) {
		pr_err("%s:TWL6030_GPADC conversion is invalid %d\n",
			__func__, ret);
		return -EINVAL;
	}
	temp = adc_to_temp_conversion(req.rbuf[TWL6030_GPADC_CHANNEL]);

	return temp;
}

/*
 * sysfs hook functions
 */
static int pcb_temp_sensor_read_temp(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	int temp = pcb_read_current_temp();

	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR(temperature, S_IRUGO, pcb_temp_sensor_read_temp,
			  NULL);

static struct attribute *pcb_temp_sensor_attributes[] = {
	&dev_attr_temperature.attr,
	NULL
};

static const struct attribute_group pcb_temp_sensor_group = {
	.attrs = pcb_temp_sensor_attributes,
};

static int __devinit pcb_temp_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;

	temp_sensor = kzalloc(sizeof(struct pcb_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	temp_sensor->pdev = pdev;
	temp_sensor->dev = &pdev->dev;

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	platform_set_drvdata(pdev, temp_sensor);

	ret = sysfs_create_group(&pdev->dev.kobj,
				 &pcb_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	dev_info(&pdev->dev, "%s\n", "notle_pcb_sensor");

	notle_pcb_sensor.update_temp = pcb_read_current_temp;
	omap4_duty_pcb_register(&notle_pcb_sensor);
	return 0;

sysfs_create_err:
	platform_set_drvdata(pdev, NULL);
	kfree(temp_sensor);
	return ret;
}

static int __devexit pcb_temp_sensor_remove(struct platform_device *pdev)
{
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &pcb_temp_sensor_group);
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_REMOVE);
	platform_set_drvdata(pdev, NULL);
	kfree(temp_sensor);

	return 0;
}

static int pcb_temp_sensor_runtime_suspend(struct device *dev)
{
	return 0;
}

static int pcb_temp_sensor_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pcb_temp_sensor_dev_pm_ops = {
	.runtime_suspend = pcb_temp_sensor_runtime_suspend,
	.runtime_resume = pcb_temp_sensor_runtime_resume,
};

static struct platform_driver pcb_temp_sensor_driver = {
	.probe = pcb_temp_sensor_probe,
	.remove = pcb_temp_sensor_remove,
	.driver = {
		.name = "notle_pcb_sensor",
		.pm = &pcb_temp_sensor_dev_pm_ops,
	},
};

int __init pcb_temp_sensor_init(void)
{
	return platform_driver_register(&pcb_temp_sensor_driver);
}

static void __exit pcb_temp_sensor_exit(void)
{
	platform_driver_unregister(&pcb_temp_sensor_driver);
}

module_init(pcb_temp_sensor_init);
module_exit(pcb_temp_sensor_exit);

MODULE_DESCRIPTION("Notle PCB Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Google Inc");
