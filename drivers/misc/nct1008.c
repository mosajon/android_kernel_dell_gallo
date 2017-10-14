/*
 * drivers/misc/nct1008.c
 *
 * Driver for NCT1008, temperature monitoring device from ON Semiconductors
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/hwmon-sysfs.h>
#include <linux/nct1008.h>

#define DRIVER_NAME "nct1008"
#define DEBUGLOG 0

#if DEBUGLOG
#define pr_db(fmt, ...) printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_db(fmt, ...) ((void)0)
#endif



/* Register Addresses */
#define LOCAL_TEMP_RD			0x00
#define EXT_HI_TEMP_RD			0x01
#define EXT_LO_TEMP_RD			0x10
#define STATUS_RD			0x02
#define CONFIG_RD			0x03

#define CONFIG_WR			0x09
#define CONV_RATE_WR			0x0A
#define LOCAL_TEMP_HI_LIMIT_WR		0x0B
#define EXT_TEMP_HI_LIMIT_HI_BYTE	0x0D
#define OFFSET_WR			0x11
#define EXT_THERM_LIMIT_WR		0x19
#define LOCAL_THERM_LIMIT_WR		0x20
#define THERM_HYSTERESIS_WR		0x21

/* Configuration Register Bits */
#define EXTENDED_RANGE_BIT		(0x1 << 2)
#define THERM2_BIT			(0x1 << 5)
#define STANDBY_BIT			(0x1 << 6)

/* Max Temperature Measurements */
#define EXTENDED_RANGE_OFFSET		64U
#define STANDARD_RANGE_MAX		127U
#define EXTENDED_RANGE_MAX		(150U + EXTENDED_RANGE_OFFSET)

struct nct1008_data {
	struct work_struct work;
	struct work_struct therm_work;
	struct i2c_client *client;
	struct mutex mutex;
	u8 config;
	void (*alarm_fn)(bool raised);
};

static inline u8 value_to_temperature(bool extended, u8 value)
{
	return extended ? (u8)(value - EXTENDED_RANGE_OFFSET) : value;
}

static inline u8 temperature_to_value(bool extended, u8 temp)
{
	return extended ? (u8)(temp + EXTENDED_RANGE_OFFSET) : temp;
}

/* NCT1008 attribute function */

static ssize_t set_limit(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	int value = simple_strtol(buf, NULL, 10);
	int err;

	pr_db("%s(%d), index: %d\n", __func__, __LINE__, attr->index);
	value = temperature_to_value(pdata->ext_range, value);
	pr_db("%s(%d), value = %d", __func__, __LINE__, value);
	switch (attr->index) {
	case 0:
		err = i2c_smbus_write_byte_data(client, LOCAL_THERM_LIMIT_WR, value);
		pr_db("%s(%d), write data to LOCAL_THERM_LIMIT: %d\n", __func__, __LINE__, value);
		break;
	case 1:
		err = i2c_smbus_write_byte_data(client, EXT_THERM_LIMIT_WR, value);
		pr_db("%s(%d), write data to EXT_THERM_LIMIT: %d\n", __func__, __LINE__, value);
		break;
	case 2:
		err = i2c_smbus_write_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR, value);
		pr_db("%s(%d), write data to LOCAL_TEMP_HI_LIMIT: %d\n", __func__, __LINE__, value);
		break;
	case 3:
		err = i2c_smbus_write_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE, value);
		pr_db("%s(%d), write data to EXT_TEMP_HI_LIMIT: %d\n", __func__, __LINE__, value);
		break;
	default:
		pr_db("%s(%d), NOT available...\n", __func__, __LINE__);
		break;
	}
	if (err < 0)
		return -1;

	return count;
}


static ssize_t show_limit(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	int temp_limit;

	pr_db("%s(%d), index: %d\n", __func__, __LINE__, attr->index);

	switch (attr->index) {
	case 0:
		temp_limit = i2c_smbus_read_byte_data(client, LOCAL_THERM_LIMIT_WR);
		pr_db("%s(%d), read data from LOCAL_THERM_LIMIT: %d\n", __func__, __LINE__, temp_limit);
		break;
	case 1:
		temp_limit = i2c_smbus_read_byte_data(client, EXT_THERM_LIMIT_WR);
		pr_db("%s(%d), read data from REMOTE_THERM_LIMIT: %d\n", __func__, __LINE__, temp_limit);
		break;
	case 2:
		temp_limit = i2c_smbus_read_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR);
		pr_db("%s(%d), read data from LOCAL_HIGH_LIMIT: %d\n", __func__, __LINE__, temp_limit);
		break;
	case 3:
		temp_limit = i2c_smbus_read_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE);
		pr_db("%s(%d), read data from REMOTE_HIGH_LIMIT: %d\n", __func__, __LINE__, temp_limit);
		break;
	default:
		pr_db("%s(%d), NOT available...\n", __func__, __LINE__);
		break;
	}

	return sprintf(buf, "%d\n", temp_limit);
}

static ssize_t show_temp(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	int temp;
	int temp_h, temp_l;

	pr_db("%s(%d), index: %d\n", __func__, __LINE__, attr->index);

	if (attr->index) {
		temp_l = i2c_smbus_read_byte_data(client, EXT_LO_TEMP_RD);
		temp_l = temp_l >> 6;
		switch (temp_l) {
		case 0:
			temp_l = 0;
			break;
		case 1:
			temp_l = 25;
			break;
		case 2:
			temp_l = 50;
			break;
		case 3:
			temp_l = 75;
			break;
		default:
			pr_db("%s(%d), read EXT_TEMP_LO temperature fail...");
			break;
		}
		pr_db("%s(%d), read data from EXT_LO_TEMP_RD: %d\n", __func__, __LINE__, temp_l);
		temp_h = i2c_smbus_read_byte_data(client, EXT_HI_TEMP_RD);
		pr_db("%s(%d), read data from EXT_HI_TEMP_RD: %d\n", __func__, __LINE__, temp_h);
		return sprintf(buf, "%d.%d\n", temp_h, temp_l);
	} else {
		temp = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
		pr_db("%s(%d), read data from LOCAL_TEMP_RD: %d\n", __func__, __LINE__, temp);
		return sprintf(buf, "%d\n", temp);
	}
}

static ssize_t nct1008_show_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	signed int temp_value = 0;
	u8 data = 0;

	if (!dev || !buf || !attr)
		return -EINVAL;

	data = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"temperature\n", __func__);
		return -EINVAL;
	}

	temp_value = (signed int)data;
	return sprintf(buf, "%d\n", temp_value);
}

static ssize_t nct1008_show_ext_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	signed int temp_value = 0;
	u8 data = 0;

	if (!dev || !buf || !attr)
		return -EINVAL;

	data = i2c_smbus_read_byte_data(client, EXT_HI_TEMP_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"ext_temperature\n", __func__);
		return -EINVAL;
	}

	temp_value = (signed int)data;

	data = i2c_smbus_read_byte_data(client, EXT_LO_TEMP_RD);

	return sprintf(buf, "%d.%d\n", temp_value, (25 * (data >> 6)));
}

static DEVICE_ATTR(temperature, S_IRUGO, nct1008_show_temp, NULL);
static DEVICE_ATTR(ext_temperature, S_IRUGO, nct1008_show_ext_temp, NULL);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(local_crit, S_IRUGO | S_IWUSR, show_limit, set_limit, 0);
static SENSOR_DEVICE_ATTR(remote_crit, S_IRUGO | S_IWUSR, show_limit, set_limit, 1);
static SENSOR_DEVICE_ATTR(local_high, S_IRUGO | S_IWUSR, show_limit, set_limit, 2);
static SENSOR_DEVICE_ATTR(remote_high, S_IRUGO | S_IWUSR, show_limit, set_limit, 3);

static struct attribute *nct1008_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_ext_temperature.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_local_crit.dev_attr.attr,
	&sensor_dev_attr_remote_crit.dev_attr.attr,
	&sensor_dev_attr_local_high.dev_attr.attr,
	&sensor_dev_attr_remote_high.dev_attr.attr,
	NULL
};

static const struct attribute_group nct1008_attr_group = {
	.attrs = nct1008_attributes,
};

static void nct1008_enable(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);

	i2c_smbus_write_byte_data(client, CONFIG_WR,
				  data->config & ~STANDBY_BIT);
}

static void nct1008_disable(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);

	i2c_smbus_write_byte_data(client, CONFIG_WR,
				  data->config | STANDBY_BIT);
}

static void nct1008_therm_work_func(struct work_struct *work)
{
	struct nct1008_data *data = container_of(work, struct nct1008_data, therm_work);
	struct nct1008_platform_data *pdata = data->client->dev.platform_data;
	int value = gpio_get_value(irq_to_gpio(pdata->therm_irq));
	char event[32] = {0};
	char *envp[] = {event, NULL};

	pr_db("%s(%d), therm work func, value: %d\n", __func__, __LINE__, value);
	if (value) {
		printk("send KOBJ THERM_EVENT\n");
		snprintf(event, sizeof(event), "EVENT=THERM_EVENT");
		kobject_uevent_env(&data->client->dev.kobj, KOBJ_CHANGE, envp);
	}
}

static void nct1008_work_func(struct work_struct *work)
{
	struct nct1008_data *data = container_of(work, struct nct1008_data, work);
	int irq = data->client->irq;
	int value = gpio_get_value(irq_to_gpio(irq));
	char event[32] = {0};
	char *envp[] = {event, NULL};

	pr_db("%s(%d), alert work func, value: %d\n", __func__, __LINE__, value);
	mutex_lock(&data->mutex);
	if (data->alarm_fn) {
		/* Therm2 line is active low */
		data->alarm_fn(!value);
	}

	if (!value) {
		printk("send KOBJ THERM2_EVENT\n");
		snprintf(event, sizeof(event), "EVENT=THERM2_EVENT");
		kobject_uevent_env(&data->client->dev.kobj, KOBJ_CHANGE, envp);
	}
	mutex_unlock(&data->mutex);
}

static irqreturn_t nct1008_therm_irq(int irq, void *dev_id)
{
	struct nct1008_data *data = dev_id;
	schedule_work(&data->therm_work);

	return IRQ_HANDLED;
}

static irqreturn_t nct1008_irq(int irq, void *dev_id)
{
	struct nct1008_data *data = dev_id;
	schedule_work(&data->work);

	return IRQ_HANDLED;
}

static int __devinit nct1008_configure_sensor(struct nct1008_data* data)
{
	struct i2c_client *client           = data->client;
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	u8 value;
	int err;

	if (!pdata || !pdata->supported_hwrev)
		return -ENODEV;

	/*
	 * Initial Configuration - device is placed in standby and
	 * ALERT/THERM2 pin is configured as THERM2
	 */
	data->config = value = pdata->ext_range ?
		(STANDBY_BIT | THERM2_BIT | EXTENDED_RANGE_BIT) :
		(STANDBY_BIT | THERM2_BIT);

	err = i2c_smbus_write_byte_data(client, CONFIG_WR, value);
	if (err < 0)
		goto error;

	/* Temperature conversion rate */
	err = i2c_smbus_write_byte_data(client, CONV_RATE_WR, pdata->conv_rate);
	if (err < 0)
		goto error;

	/* External temperature h/w shutdown limit */
	value = temperature_to_value(pdata->ext_range, pdata->shutdown_ext_limit);
	err = i2c_smbus_write_byte_data(client, EXT_THERM_LIMIT_WR, value);
	if (err < 0)
		goto error;

	/* Local temperature h/w shutdown limit */
	value = temperature_to_value(pdata->ext_range, pdata->shutdown_local_limit);
	err = i2c_smbus_write_byte_data(client, LOCAL_THERM_LIMIT_WR, value);
	if (err < 0)
		goto error;

	/* External Temperature Throttling limit */
	value = temperature_to_value(pdata->ext_range, pdata->throttling_ext_limit);
	err = i2c_smbus_write_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE, value);
	if (err < 0)
		goto error;

	/* Local Temperature Throttling limit */
	value = pdata->ext_range ? EXTENDED_RANGE_MAX : STANDARD_RANGE_MAX;
	err = i2c_smbus_write_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR, value);
	if (err < 0)
		goto error;

	/* Remote channel offset */
	err = i2c_smbus_write_byte_data(client, OFFSET_WR, pdata->offset);
	if (err < 0)
		goto error;

	/* THERM hysteresis */
	err = i2c_smbus_write_byte_data(client, THERM_HYSTERESIS_WR, pdata->hysteresis);
	if (err < 0)
		goto error;

	data->alarm_fn = pdata->alarm_fn;

	return 0;
error:
	return err;
}

static int __devinit nct1008_configure_irq(struct nct1008_data *data)
{
	struct nct1008_platform_data *pdata = data->client->dev.platform_data;
	int err = -1;

	pr_db("%s(%d), register alert irq %d\n", __func__, __LINE__, data->client->irq);
	INIT_WORK(&data->work, nct1008_work_func);
	err = request_irq(data->client->irq, nct1008_irq, IRQF_TRIGGER_RISING |
			 IRQF_TRIGGER_FALLING, DRIVER_NAME, data);
	if (err < 0) {
		printk("failed to request irq %d\n", data->client->irq);
		return err;
	}

	if (pdata->therm_irq) {
		pr_db("%s(%d), register therm irq %d\n", __func__, __LINE__, pdata->therm_irq);
		INIT_WORK(&data->therm_work, nct1008_therm_work_func);
		err = request_irq(pdata->therm_irq, nct1008_therm_irq, IRQF_TRIGGER_RISING |
				 IRQF_TRIGGER_FALLING, "temp_therm", data);
		if (err < 0) {
			printk("failed to request irq %d\n", pdata->therm_irq);
		}
	}
	return err;
}

static int __devinit nct1008_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct nct1008_data *data;
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	int err;

	data = kzalloc(sizeof(struct nct1008_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->mutex);

	err = nct1008_configure_sensor(data);	/* sensor is in standby */
	if (err < 0)
		goto error;

	err = nct1008_configure_irq(data);
	if (err < 0)
		goto error;

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &nct1008_attr_group);
	if (err < 0)
		goto error;

	dev_info(&client->dev, "%s: initialized\n", __func__);

	nct1008_enable(client);		/* sensor is running */

	schedule_work(&data->work);		/* check initial state */
	if (pdata->therm_irq)
		schedule_work(&data->therm_work);

	return 0;

error:
	kfree(data);
	return err;
}

static int __devexit nct1008_remove(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = data->client->dev.platform_data;

	if (pdata->therm_irq) {
		free_irq(pdata->therm_irq, data);
		cancel_work_sync(&data->therm_work);
	}
	free_irq(data->client->irq, data);
	cancel_work_sync(&data->work);
	sysfs_remove_group(&client->dev.kobj, &nct1008_attr_group);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int nct1008_suspend(struct i2c_client *client, pm_message_t state)
{
	int err = -1;
	/* in suspend mode, thermal sensor does not
	   need to enter standby mode and it can not disable irq.  */
	/* disable_irq(client->irq); */
	/* nct1008_disable(client); */
	printk("%s(%d), Set conversion rate to mode 0x00\n", __func__, __LINE__);
	err = i2c_smbus_write_byte_data(client, CONV_RATE_WR, 0x00);
	if (err < 0)
		printk("%s(%d), Set conversion rate to mode 0x00 failed\n", __func__, __LINE__);

	return 0;
}

static int nct1008_resume(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = client->dev.platform_data;
	int err = -1;

	/* nct1008_enable(client); */
	/* enable_irq(client->irq); */
	printk("%s(%d), Set conversion rate to 0x04\n", __func__, __LINE__);
	err = i2c_smbus_write_byte_data(client, CONV_RATE_WR, pdata->conv_rate);
	if (err < 0)
		printk("%s(%d), Set conversion rate to mode 0x04 failed\n", __func__, __LINE__);

	schedule_work(&data->work);
	if (pdata->therm_irq)
		schedule_work(&data->therm_work);

	return 0;
}
#endif

static const struct i2c_device_id nct1008_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct1008_id);

static struct i2c_driver nct1008_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe		= nct1008_probe,
	.remove		= __devexit_p(nct1008_remove),
	.id_table	= nct1008_id,
#ifdef CONFIG_PM
	.suspend	= nct1008_suspend,
	.resume		= nct1008_resume,
#endif
};

static int __init nct1008_init(void)
{
	return i2c_add_driver(&nct1008_driver);
}

static void __exit nct1008_exit(void)
{
	i2c_del_driver(&nct1008_driver);
}

MODULE_DESCRIPTION("Temperature sensor driver for OnSemi NCT1008");
MODULE_LICENSE("GPL");

module_init (nct1008_init);
module_exit (nct1008_exit);
