/*
 * drivers/power/bq20z45_battery.c
 *
 * Gas Gauge driver for TI's BQ20Z45
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 * Copyright (c) 2011, PEGATRON Corporation.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c/bq20z45_power.h>
#include <linux/irq.h>
#include <mach/gpio.h>

#include <linux/kthread.h>

enum {
	REG_MANUFACTURER_DATA,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CYCLE_COUNT,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_REMAIN_CAPACITY,
	REG_FCC_CAPACITY,
	REG_MAX
};

enum {
	REG_CHARGE_CURRENT = 14,
	REG_CHARGE_VOLTAGE = 15,
};

enum {
	PDA_PSY_OFFLINE = 0,
	PDA_PSY_ONLINE = 1,
	PDA_PSY_TO_CHANGE,
};

#define BATTERY_RETRY_MAX	5

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_INIT_DONE	0x80
#define BATTERY_DISCHARGING	0x40
#define BATTERY_FULL_CHARGED	0x20
#define BATTERY_FULL_DISCHARGED	0x10

#define KMAXTEMPERATURE_DISCHG 650
#define KMAXTEMPERATURE_CHG 850
#define KMINTEMPERATURE 0

#define KRECHARGETEMPH  400
#define KRECHARGETEMPL  50
static unsigned int cache_time = 10000;

#ifdef DEBUG
#define BATT_MSG(x...) pr_err("<BATT>" x)
#else
#define BATT_MSG(x...) do {} while (0)
#endif

#define temperature_cmp(temp, low, high) (((temp < high) && (temp > low)) ? 0 : 1)

#define BQ20Z45_DATA(_psp, _addr, _min_value, _max_value)	\
{							\
		.psp = POWER_SUPPLY_PROP_##_psp,		\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
}

#define BQ20Z45_DATAS(_addr, _min_value, _max_value)	\
{							\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
}


static int bq20z45_polling_thread(void *data);

static struct bq20z45_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq20z45_data[] = {
	[REG_MANUFACTURER_DATA] = BQ20Z45_DATA(PRESENT, 0x00, 0, 65535),
	[REG_TEMPERATURE]       = BQ20Z45_DATA(TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE]           = BQ20Z45_DATA(VOLTAGE_NOW, 0x09, 0, 20000),
	[REG_CURRENT]           = BQ20Z45_DATA(CURRENT_NOW, 0x0A, -32768, 32767),
	[REG_TIME_TO_EMPTY]     = BQ20Z45_DATA(TIME_TO_EMPTY_AVG, 0x12, 0, 65535),
	[REG_TIME_TO_FULL]      = BQ20Z45_DATA(TIME_TO_FULL_AVG, 0x13, 0, 65535),
	[REG_STATUS]            = BQ20Z45_DATA(STATUS, 0x16, 0, 65535),
	/*[REG_CYCLE_COUNT]       = BQ20Z75_DATA(CYCLE_COUNT, 0x17, 0, 65535),*/
	[REG_CAPACITY]          = BQ20Z45_DATA(CAPACITY, 0x0d, 0, 100),
	[REG_SERIAL_NUMBER]     = BQ20Z45_DATA(SERIAL_NUMBER, 0x1C, 0, 65535),
	[REG_CHARGE_CURRENT]     = BQ20Z45_DATAS(0x14, 0, 65535),
	[REG_CHARGE_VOLTAGE]     = BQ20Z45_DATAS(0x15, 0, 65535),
	[REG_REMAIN_CAPACITY]    = BQ20Z45_DATAS(0x0f, 0, 65535),
	[REG_FCC_CAPACITY]       = BQ20Z45_DATAS(0x10, 0, 65535),
};

static enum power_supply_property bq20z45_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	/*POWER_SUPPLY_PROP_CYCLE_COUNT,*/
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property bq20z45_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *power_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq20z45_properties_sequence[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

#define PROPERTY_SEQUENCE_SIZE            ARRAY_SIZE(bq20z45_properties_sequence)
#define GET_PROPERTY_SEQUENCE_START(psp) ((psp == bq20z45_properties_sequence[0]) ? 1 : 0)
#define GET_PROPERTY_SEQUENCE_END(psp)   ((psp == bq20z45_properties_sequence[PROPERTY_SEQUENCE_SIZE-1]) ? 1 : 0)

static struct bq20z45_status_backup
{
	bool ac_status;
	int batt_voltage;
	int batt_capacity;
	int batt_temperature;
	int manufactory_data;
	int register_status;
	int batt_status;
} batt_status_backup;

struct BQ20Z45_power_pdata *charger_data;

static struct bq20z45_device_info {
	struct i2c_client	*client;

	struct BQ20Z45_power_pdata *charger_pdata;
	struct task_struct *polling_thread;
	wait_queue_head_t wait_q_update;
	int batt_voltage;
	int batt_capacity;
	int batt_temperature;
	int charge_current;
	int charge_voltage;
	int manufactory_data;
	int register_status;
	int batt_status;
	bool state_changed;
	unsigned long update_time;
	atomic_t is_in_suspend;
	unsigned long ac_latest_update_time;

	bool ac_status;
	int sequence_index;
	atomic_t backup_lock;
	struct wake_lock wake_lock;
} *bq20z45_device;

static int bq20z45_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int bq20z45_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static ssize_t bq20z45_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);
static void bq20z45_battery_status_update(struct bq20z45_device_info *info);

enum supply_type {
	SUPPLY_TYPE_BATTERY = 0,
	SUPPLY_TYPE_AC,
};

static struct power_supply bq20z45_supply[] = {
	[SUPPLY_TYPE_BATTERY] = {
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq20z45_properties,
		.num_properties	= ARRAY_SIZE(bq20z45_properties),
		.get_property	= bq20z45_get_property,
	},
	[SUPPLY_TYPE_AC] = {
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= power_supplied_to,
		.num_supplicants= ARRAY_SIZE(power_supplied_to),
		.properties	= bq20z45_ac_properties,
		.num_properties = ARRAY_SIZE(bq20z45_ac_properties),
		.get_property	= bq20z45_ac_get_property,
	},
};

static s32 i2c_smbus_read_word_data_retry(struct i2c_client *client, u8 command)
{
	int retry = 0;
	s32 res = 0;
	for(;retry < BATTERY_RETRY_MAX; retry++) {
		res = i2c_smbus_read_word_data(client, command);
		if(res >= 0)
			break;
		printk("%s-retry:%d\n",__FUNCTION__,retry);
	}
	return res;
}

static s32 i2c_smbus_write_word_data_retry(struct i2c_client *client, u8 command, u16 value)
{
	int retry = 0;
	s32 res = 0;
	for(;retry < BATTERY_RETRY_MAX; retry++) {
		res = i2c_smbus_write_word_data(client, command, value);
		if(res >= 0)
			break;
		printk("%s-retry:%d\n",__FUNCTION__,retry);
	}
	return res;
}

static s32 i2c_smbus_read_byte_data_retry(struct i2c_client *client, u8 command)
{
	int retry = 0;
	s32 res = 0;
	for(;retry < BATTERY_RETRY_MAX; retry++) {
		res = i2c_smbus_read_byte_data(client, command);
		if(res >= 0)
			break;
		printk("%s-retry:%d\n",__FUNCTION__,retry);
	}
	return res;

}

static void check_property_sequence(enum power_supply_property batt_prop)
{
	BATT_MSG("+%s\n", __func__);

	if (batt_prop == bq20z45_properties_sequence[bq20z45_device->sequence_index])	{
		BATT_MSG("%s:sequence in : %d\n", __func__, bq20z45_device->sequence_index);
		bq20z45_device->sequence_index++;

		if (GET_PROPERTY_SEQUENCE_START(batt_prop))	{
			atomic_set(&bq20z45_device->backup_lock, 1);
		} else if (GET_PROPERTY_SEQUENCE_END(batt_prop)) {
			atomic_set(&bq20z45_device->backup_lock, 0);
			bq20z45_device->sequence_index = 0;
		}
	} else {
		BATT_MSG("%s:fail sequence : %d\n", __func__, bq20z45_device->sequence_index);
		atomic_set(&bq20z45_device->backup_lock, 0);
		bq20z45_device->sequence_index = 0;

		if (GET_PROPERTY_SEQUENCE_START(batt_prop))	{
			atomic_set(&bq20z45_device->backup_lock, 1);
			bq20z45_device->sequence_index++;
		}
	}

	BATT_MSG("-%s\n", __func__);
}

static int battery_get_charger_status(void)
{
	return bq20z45_device->charger_pdata->get_charger_status();
}

static int bq20z45_get_ac_status(void)
{
	return bq20z45_device->charger_pdata->is_ac_online();
}

static int bq20z45_is_nce_asserted(void)
{
	return gpio_get_value(bq20z45_device->charger_pdata->nce_gpio);
}

static int bq20z45_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	BATT_MSG("+%s: psp:%d\n", __func__, psp);

	check_property_sequence(psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = batt_status_backup.ac_status;
		break;
	default:
		dev_err(&bq20z45_device->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	BATT_MSG("-%s: psp:%d, val:%d\n", __func__, psp, val->intval);
	return 0;
}

static int bq20z45_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_STATUS);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {

		/* Mask the upper nibble of 2nd byte and
		 * lower byte of response then
		 * shift the result by 8 to get status*/
		ret &= 0x0F00;
		ret >>= 8;
		if (psp == POWER_SUPPLY_PROP_PRESENT) {
			if (ret == 0x0F)
				/* battery removed */
				val->intval = 0;
			else
				val->intval = 1;
		} else if (psp == POWER_SUPPLY_PROP_HEALTH) {
			if (ret == 0x09)
				val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			else if (ret == 0x0B)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (ret == 0x0C)
				val->intval = POWER_SUPPLY_HEALTH_DEAD;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
	} else {
		val->intval = 0;
	}

	return 0;
}

static int bq20z45_get_register_status(void)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		REG_STATUS);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for battery status "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq20z45_data[REG_STATUS].min_value &&
	    ret <= bq20z45_data[REG_STATUS].max_value) {

		/* Mask the upper nibble of 2nd byte and
		 * lower byte of response then
		 * shift the result by 8 to get status*/
		ret &= 0x00F0;
		ret >>= 8;
		return ret;
	}
	return 0;
}

static int bq20z45_get_manufacturer_data(void)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_STATUS);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {
		return ret;
	}
	return 0;
}

inline int bq20z45_manufacturer_data_2_status(int data)
{
	int ret = data;

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {

		/* Mask the upper nibble of 2nd byte and
		 * lower byte of response then
		 * shift the result by 8 to get status*/
		ret &= 0xFF00;
		ret >>= 8;
		return ret;
	}
	return 0;
}

inline int bq20z45_manufacturer_data_2_present(int data)
{
	int ret = data;

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {
		ret &= 0x0F00;
		ret >>= 8;
		if (ret == 0x0F)
		/* battery removed */
			return 0;
		else
			return 1;
	}

	return 0;
}

inline int bq20z45_manufacturer_data_2_health(int data)
{
	int ret = data;

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {

		ret &= 0x0F00;
		ret >>= 8;

		if (ret == 0x09)
			return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (ret == 0x0B)
			return POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (ret == 0x0C)
			return POWER_SUPPLY_HEALTH_DEAD;
		else
			return POWER_SUPPLY_HEALTH_GOOD;
	} else {
		return 0;
	}
}

static int bq20z45_get_manufacturer_status(void)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_STATUS);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq20z45_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq20z45_data[REG_MANUFACTURER_DATA].max_value) {

		/* Mask the upper nibble of 2nd byte and
		 * lower byte of response then
		 * shift the result by 8 to get status*/
		ret &= 0xFF00;
		ret >>= 8;
		return ret;
	}
	return 0;
}

static int bq20z45_get_operation_status(void)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		0x54);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}
	BATT_MSG(" opearation = %d\n", ret);

	return ret;
}

static int bq20z45_get_battery_charging_status(void)
{
	s32 ret;

	/* Write to ManufacturerAccess with
	 * ManufacturerAccess command and then
	 * read the status */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		0x55);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}
	BATT_MSG(" charging status = %d\n", ret);

	return ret;
}

static int get_psy_prop_status(int value)
{
	/*set default state*/
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int is_ac_present = bq20z45_device->ac_status;
	int ret_status = battery_get_charger_status();
	/* mask the upper byte and then find the
	 * actual status */
	value &= 0x00FF;
	/* check the error conditions
	 * lower nibble represent error
	 * 0 = no error, so check the remaining bits
	 * != 0 means error so return */
	if ((value & 0x000F) == 0x0F) {
		BATT_MSG("%s unknown battery status:%d\n", __func__, value);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (is_ac_present) {
		if (value & BATTERY_FULL_CHARGED) {
			status = POWER_SUPPLY_STATUS_FULL;
		} else if ((value & BATTERY_DISCHARGING) == 0) {
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (value & BATTERY_DISCHARGING) {
			/*battery status update has latency*/
			if(time_before(jiffies, bq20z45_device->ac_latest_update_time +
						msecs_to_jiffies(cache_time))) {
				status = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				status = POWER_SUPPLY_STATUS_DISCHARGING;
			}
		} else {
			status = POWER_SUPPLY_STATUS_UNKNOWN;
		}
	} else {
		if (value & BATTERY_FULL_DISCHARGED) {
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}

	BATT_MSG(" %s, status:%d, ac_present:%d, value:%d\n", __func__, status, is_ac_present, value);
	return status;
}

static int bq20z45_get_psp(int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret;
	int ret_capacity;

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[reg_offset].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, reg_offset);
		return -EINVAL;
	}

	if (reg_offset == REG_CURRENT) {
		if (ret & 0x8000) {
			BATT_MSG(" discharging, original current = %d \n", ret);
			ret -= 1;
			ret_capacity = (~ret); /*2's complement*/
			ret  = (-1)*(ret_capacity);
			BATT_MSG(" discharging, capacity_ret = %d \n", ret);
		}
	}

	if (ret >= bq20z45_data[reg_offset].min_value &&
	    ret <= bq20z45_data[reg_offset].max_value) {
		if (psp == POWER_SUPPLY_PROP_STATUS) {
			val->intval = get_psy_prop_status(ret);
		} else if (psp == POWER_SUPPLY_PROP_TEMP) {
			/* bq20z75 provides battery tempreture in 0.1°K so convert it to °C */
			val->intval = ret - 2731;
		} else if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
			val->intval = ret*1000;
		} else if (psp == POWER_SUPPLY_PROP_CURRENT_NOW) {
			val->intval = (ret & 0x8000) ? -(ret & 0x7FFF) : (ret & 0x7FFF);
		} else {
			val->intval = ret;
		}
	} else {
		/*out of range*/
		BATT_MSG(" %s , out of range\n", __func__);
		val->intval = 0;
		if (psp == POWER_SUPPLY_PROP_STATUS)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return 0;
}

static int bq20z45_get_capacity_psp(union power_supply_propval *val)
{
	s32 ret;
	s32 ret_fcc;
	s32 capacity_temp = 0;

	/*set default out parameter to 1*/
	val->intval = 1;

#ifdef CONFIG_MACH_GALLO
	ret = i2c_smbus_read_byte_data_retry(bq20z45_device->client,
		bq20z45_data[REG_CAPACITY].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CAPACITY);
		return -EINVAL;
	}

	/* bq20z75 spec says that this can be >100 %
	 * even if max value is 100 % */
#endif

	val->intval = ((ret >= 100) ? 100 : ret);

	return 0;
}

static char bq20z45_serial[9] = {'0'};
static int bq20z45_get_battery_serial_number(union power_supply_propval *val)
{
	int ret;

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_SERIAL_NUMBER].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_SERIAL_NUMBER);
		return -EINVAL;
	}

	ret = scnprintf(bq20z45_serial, sizeof(bq20z45_serial)-1, "%d", ret);
	if(val)
		val->strval = bq20z45_serial;

	return 0;
}


static char bq20z45_tech[5];
static int bq20z45_get_tech(u8 *s_tech)
{
	s32 ret;

	ret = i2c_smbus_read_block_data(bq20z45_device->client,
		0x22, s_tech);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, 0x22);
		return -EINVAL;
	}

	/* bq20z75 spec says that this can be >100 %
	 * even if max value is 100 % */
	BATT_MSG(" tech = %s\n", (char *)s_tech);

	return 0;
}

static int bq20z45_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	BATT_MSG("+%s: psp:%d\n", __func__, psp);

	check_property_sequence(psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq20z45_manufacturer_data_2_present(batt_status_backup.manufactory_data);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq20z45_manufacturer_data_2_health(batt_status_backup.manufactory_data);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (batt_status_backup.batt_status == POWER_SUPPLY_STATUS_FULL)
			val->intval = 100;
		else
			val->intval = batt_status_backup.batt_capacity;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		if (bq20z45_get_battery_serial_number(val))
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = batt_status_backup.batt_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_status_backup.batt_voltage * 1000; /* in micro voltage for framework */
				break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = batt_status_backup.batt_temperature;
		break;
	default:
		dev_err(&bq20z45_device->client->dev,
				"%s: INVALID property\n", __func__);
		return -EINVAL;
	}
	BATT_MSG("-%s: psp:%d, value:%d\n", __func__, psp, val->intval);
	return 0;
}


static ssize_t bq20z45_battery_store_ce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int state = 0;
	sscanf(buf, "%d", &state);
	if ((state != 0) && (state != 1)) {
		return 0;
	}
	gpio_direction_output(bq20z45_device->charger_pdata->nce_gpio, state);
	return count;
}

#define BQ20Z45_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO,},	\
	.show = bq20z45_battery_show_property,					\
	.store = NULL,								\
}

#define BQ20Z45_BATTERY_ATTR_ATS(_name, _store)                         \
{                                       \
    .attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR,},    \
	.show = bq20z45_battery_show_property,                  \
	.store = _store,                                \
}


static struct device_attribute bq20z45_battery_attrs[] = {
	BQ20Z45_BATTERY_ATTR(batt_vol),
	BQ20Z45_BATTERY_ATTR(batt_temp),
	BQ20Z45_BATTERY_ATTR(charging_source),
	BQ20Z45_BATTERY_ATTR(charging_enabled),
	BQ20Z45_BATTERY_ATTR(full_bat),
	BQ20Z45_BATTERY_ATTR(charge_current),
	BQ20Z45_BATTERY_ATTR(charge_voltage),
	BQ20Z45_BATTERY_ATTR(manufactory_status),
	BQ20Z45_BATTERY_ATTR(tech),
	BQ20Z45_BATTERY_ATTR_ATS(charger_ce, bq20z45_battery_store_ce),
	BQ20Z45_BATTERY_ATTR(lowbatt),
	BQ20Z45_BATTERY_ATTR(serial_number_test),
};

enum {
	BATT_VOL = 0,
	BATT_TEMP,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
	CHARGE_CURRENT,
	CHARGE_VOLTAGE,
	MANUFACTORY_STATUS,
	TECH,
	CHARGER_CE,
	LOWBATT,
	SERIAL_NUMBER_TEST,
};


static int bq20z45_battery_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(bq20z45_battery_attrs); i++) {
		rc = device_create_file(dev, &bq20z45_battery_attrs[i]);
		if (rc)
			goto bq20z45_attrs_failed;
	}
	goto succeed;
bq20z45_attrs_failed:
	while (i--)
		device_remove_file(dev, &bq20z45_battery_attrs[i]);
succeed:
	return rc;
}

static void bq20z45_battery_remove_attrs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(bq20z45_battery_attrs); i++)
		device_remove_file(dev, &bq20z45_battery_attrs[i]);
}

static ssize_t bq20z45_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - bq20z45_battery_attrs;

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       bq20z45_device->batt_voltage);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       bq20z45_device->batt_temperature);
		break;
	case CHARGE_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   bq20z45_device->charge_current);
		break;
	case CHARGE_VOLTAGE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   bq20z45_device->charge_voltage);
		break;
	case MANUFACTORY_STATUS:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   bq20z45_manufacturer_data_2_status(bq20z45_device->manufactory_data));
		break;
	case CHARGER_CE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       gpio_get_value(bq20z45_device->charger_pdata->nce_gpio));
		break;
	case LOWBATT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
					   !gpio_get_value(bq20z45_device->charger_pdata->lowbatt_gpio));
		break;
	case SERIAL_NUMBER_TEST:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", bq20z45_serial);
		break;

	default:
		i = -EINVAL;
	}

	BATT_MSG("%s:attr_off:%d, value:%s\n", __func__, off, buf);
	return i;
}

static int bq20z45_get_voltage(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_VOLTAGE].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_VOLTAGE);
		return -EINVAL;
	}
	if (ret <= bq20z45_data[REG_VOLTAGE].max_value &&
		ret >= bq20z45_data[REG_VOLTAGE].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read voltage failed!!!\n");
		return 0;
	}
}

static int bq20z45_get_charge_current(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_CHARGE_CURRENT].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CHARGE_CURRENT);
		return -EINVAL;
	}
	if (ret <= bq20z45_data[REG_CHARGE_CURRENT].max_value &&
		ret >= bq20z45_data[REG_CHARGE_CURRENT].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read charge current failed!!!\n");
		return 0;
	}
}

static int bq20z45_get_charge_voltage(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_CHARGE_VOLTAGE].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CHARGE_VOLTAGE);
		return -EINVAL;
	}
	if (ret <= bq20z45_data[REG_CHARGE_VOLTAGE].max_value &&
		ret >= bq20z45_data[REG_CHARGE_VOLTAGE].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read charge voltage failed!!!\n");
		return 0;
	}
}

static int bq20z45_get_temperature(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_TEMPERATURE].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_TEMPERATURE);
		return -EINVAL;
	}
	if (ret <= bq20z45_data[REG_TEMPERATURE].max_value &&
		ret >= bq20z45_data[REG_TEMPERATURE].min_value) {
		ret -= 2731;
		return ret;
	} else {
		pr_err("<BATT> read temperature failed!!!\n");
		return 0;
	}
}

static int bq20z45_get_current(void)
{
	s32 ret;
	s32 ret_capacity;
	ret = i2c_smbus_read_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_CURRENT].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CURRENT);
		return -EINVAL;
	}

	ret_capacity = (ret & 0x7FFF);

	if (ret & 0x8000) {
		ret_capacity -= 1;
		ret_capacity = (-1)*(ret_capacity);
		BATT_MSG(" discharging, capacity_ret = %d \n", ret);
	}
	if (ret_capacity <= bq20z45_data[REG_CURRENT].max_value &&
		ret_capacity >= bq20z45_data[REG_CURRENT].min_value) {
		return ret_capacity;
	} else {
		pr_err("<BATT> read current failed out of range!!!\n");
		return 0;
	}

}

static int bq20z45_get_capacity(void)
{
	s32 ret;

#ifdef CONFIG_MACH_GALLO
	ret = i2c_smbus_read_byte_data_retry(bq20z45_device->client,
		bq20z45_data[REG_CAPACITY].addr);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CAPACITY);
		return -EINVAL;
	}
	if (ret <= bq20z45_data[REG_CAPACITY].max_value &&
		ret >= bq20z45_data[REG_CAPACITY].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read capacity failed -- out of range ret = %d!!!\n", ret);
		return -EINVAL;
	}
#endif
}

static void bq20z45_battery_status_backup(struct bq20z45_device_info *batt_info)
{
	BATT_MSG("+%s: \n", __func__);

	batt_status_backup.ac_status = batt_info->ac_status;
	batt_status_backup.batt_voltage = batt_info->batt_voltage;
	batt_status_backup.batt_temperature = batt_info->batt_temperature;
	batt_status_backup.batt_capacity = batt_info->batt_capacity;
	batt_status_backup.manufactory_data = batt_info->manufactory_data;
	batt_status_backup.register_status = batt_info->register_status;
	batt_status_backup.batt_status = batt_info->batt_status;

	BATT_MSG("ac = %d, status = %d, voltage = %d, temperature = %d, capacity = %d\n",
		batt_status_backup.ac_status, batt_status_backup.batt_status, batt_status_backup.batt_voltage,
		batt_status_backup.batt_temperature, batt_status_backup.batt_capacity);
	BATT_MSG("-%s: \n", __func__);
}

static void bq20z45_battery_status_update(struct bq20z45_device_info *info)
{
	BATT_MSG("+%s: \n", __func__);

	info->batt_voltage = bq20z45_get_voltage();
	info->batt_temperature = bq20z45_get_temperature();
	info->batt_capacity = bq20z45_get_capacity();
	info->charge_current = bq20z45_get_charge_current();
	info->charge_voltage = bq20z45_get_charge_voltage();
	info->manufactory_data = bq20z45_get_manufacturer_data();
	info->register_status = bq20z45_get_register_status();
	info->ac_status = bq20z45_get_ac_status();
	info->batt_status = get_psy_prop_status(info->register_status);

	bq20z45_device->update_time = jiffies;

	BATT_MSG("+%s: backup_lock = %d\n", __func__, atomic_read(&bq20z45_device->backup_lock));

	if (!atomic_read(&bq20z45_device->backup_lock))	{
		BATT_MSG("-%s: backup_lock = %d\n", __func__, atomic_read(&bq20z45_device->backup_lock));
		bq20z45_battery_status_backup(info);
	}

	BATT_MSG("ac = %d, status = %d, voltage = %d, temperature = %d, capacity = %d, manufactory_data = %d\n",
		info->ac_status, info->batt_status, info->batt_voltage, info->batt_temperature,
		info->batt_capacity, info->manufactory_data);
	BATT_MSG("-%s: \n", __func__);
}

static int bq20z45_polling_thread(void *data)
{
	struct bq20z45_device_info *batt_info = data;

	BATT_MSG("+%s\n", __func__);

	while (1) {
		wait_event_timeout(batt_info->wait_q_update,
			batt_info->state_changed && !atomic_read(&bq20z45_device->is_in_suspend),
			batt_info->charger_pdata->interval_slow);

		if (!atomic_read(&bq20z45_device->is_in_suspend)) {
			bq20z45_battery_status_update(batt_info);

			if (batt_info->state_changed) {
				batt_info->state_changed = false;
				BATT_MSG("%s:staus change update\n", __func__);
	power_supply_changed(&bq20z45_supply[SUPPLY_TYPE_AC]);
			} else {
				BATT_MSG("%s:battery staus update\n", __func__);
	power_supply_changed(&bq20z45_supply[SUPPLY_TYPE_BATTERY]);
			}
		} else {
			BATT_MSG("%s:polling-suspend\n", __func__);
		}

		wake_lock_timeout(&bq20z45_device->wake_lock, HZ);
	};

	BATT_MSG("-%s\n", __func__);
	return 0;
}

static irqreturn_t state_change_isr(int irq, void *power_supply)
{
	BATT_MSG("%s irq:%d\n", __func__, irq);

	bq20z45_device->ac_latest_update_time = jiffies;
	bq20z45_device->state_changed = true;

	wake_up(&bq20z45_device->wait_q_update);

	return IRQ_HANDLED;
}

static int bq20z45_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc, i, flags;

	charger_data = client->dev.platform_data;
	if (!charger_data) {
		dev_dbg(&client->dev, "<BATT>  no platform data?\n");
		return -EINVAL;
	}

	bq20z45_device = kzalloc(sizeof(*bq20z45_device), GFP_KERNEL);
	if (!bq20z45_device)
		return -ENOMEM;

	memset(bq20z45_device, 0, sizeof(*bq20z45_device));
	init_waitqueue_head(&bq20z45_device->wait_q_update);

	bq20z45_device->client = client;
	bq20z45_device->update_time = jiffies;
	bq20z45_device->charger_pdata = charger_data;
	bq20z45_device->ac_latest_update_time = jiffies;

	flags = bq20z45_device->client->flags;
	bq20z45_device->client->flags &= ~I2C_M_IGNORE_NAK;
	bq20z45_device->client->flags = flags;
	i2c_set_clientdata(client, bq20z45_device);

	if (bq20z45_device->charger_pdata->is_ac_online == NULL) {
		BATT_MSG("%s is_ac_online = NULL !!!\n", __func__);
		rc =  -EINVAL;
		goto fail_init;
	}

	if (bq20z45_device->charger_pdata->get_charger_status == NULL) {
		BATT_MSG("%s get_charger_status = NULL !!!\n", __func__);
		rc =  -EINVAL;
		goto fail_init;
	}

	rc = bq20z45_get_battery_serial_number(0);

	if (rc < 0) {
		dev_err(&bq20z45_device->client->dev,
			"<BATT> %s: no battery present(%d)\n", __func__, rc);
		goto fail_init;
	}

	rc = request_irq(gpio_to_irq(bq20z45_device->charger_pdata->ac_present_gpio),
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"ac_present_irq", &bq20z45_supply[SUPPLY_TYPE_AC]);
	if (rc < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: request_irq failed(%d)\n", __func__, rc);
		goto fail_irq;
	}

	atomic_set(&bq20z45_device->backup_lock, 0);
	bq20z45_device->sequence_index = 0;

	/*charging in progress 01 --> charging complete 10 (stat2 falling)*/
	/*charging complete 10 --> charging suspend 11 (stat2 rising)*/
	rc = request_irq(gpio_to_irq(bq20z45_device->charger_pdata->stat2_gpio),
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"stat2_irq", &bq20z45_supply[SUPPLY_TYPE_BATTERY]);

	if (rc < 0) {
		pr_err("Failed to request stat1 or stat2 irq \n");
		goto fail_irq;
	}

	/*charging suspend 11 --> charging inprogress 01 (stat1 falling)*/
	/*charging in progress 01 --> charging suspend 11 (stat1 rising)*/
	rc = request_irq(gpio_to_irq(bq20z45_device->charger_pdata->stat1_gpio),
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"stat1_irq", &bq20z45_supply[SUPPLY_TYPE_BATTERY]);

	if (rc < 0) {
		pr_err("Failed to request stat1 or stat1 irq \n");
		goto fail_irq;
	}

	rc = request_irq(bq20z45_device->client->irq,
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_FALLING,
		"low_batt", &bq20z45_supply[SUPPLY_TYPE_BATTERY]);

	if (rc < 0) {
		pr_err("Failed to request low batt irq \n");
		goto fail_irq;
	}

	bq20z45_battery_status_update(bq20z45_device);

	for (i = 0; i < ARRAY_SIZE(bq20z45_supply); i++) {
		rc = power_supply_register(&client->dev, &bq20z45_supply[i]);
		if (rc) {
			dev_err(&bq20z45_device->client->dev,
				"%s: Failed to register power supply\n",
				 __func__);
			goto fail_power_register;
		}
	}

	bq20z45_battery_create_attrs(bq20z45_supply[SUPPLY_TYPE_BATTERY].dev);

	dev_info(&bq20z45_device->client->dev,
		"%s: battery driver registered\n", client->name);

	device_init_wakeup(&client->dev, 1);

	bq20z45_device->state_changed = true;
	bq20z45_device->polling_thread = kthread_create(bq20z45_polling_thread, bq20z45_device, "bq20z45_polling_thread");
	wake_up_process(bq20z45_device->polling_thread);
	wake_lock_init(&bq20z45_device->wake_lock, WAKE_LOCK_SUSPEND, "batterylock");

	return 0;

fail_power_register:
	while (i--)
		power_supply_unregister(&bq20z45_supply[i]);

fail_irq:
	free_irq(gpio_to_irq(bq20z45_device->charger_pdata->ac_present_gpio), bq20z45_device);
	free_irq(gpio_to_irq(bq20z45_device->charger_pdata->stat2_gpio), bq20z45_supply);

fail_init:
	kfree(bq20z45_device);
	bq20z45_device = NULL;
	return rc;
}

static int bq20z45_remove(struct i2c_client *client)
{
	struct bq20z45_device_info *bq20z45_device;
	int i;

	bq20z45_device = i2c_get_clientdata(client);

	bq20z45_battery_remove_attrs(bq20z45_supply[SUPPLY_TYPE_BATTERY].dev);

	for (i = 0; i < ARRAY_SIZE(bq20z45_supply); i++)
		power_supply_unregister(&bq20z45_supply[i]);

	free_irq(gpio_to_irq(bq20z45_device->charger_pdata->ac_present_gpio), bq20z45_device);
	free_irq(gpio_to_irq(bq20z45_device->charger_pdata->stat2_gpio), bq20z45_supply);
	free_irq(gpio_to_irq(bq20z45_device->charger_pdata->stat1_gpio), bq20z45_supply);
	free_irq(bq20z45_device->client->irq, bq20z45_supply);

	if (bq20z45_device) {
		kfree(bq20z45_device);
		bq20z45_device = NULL;
	}

	return 0;
}

#if defined(CONFIG_PM)
static int bq20z45_suspend(struct i2c_client *client,
	pm_message_t state)
{
	s32 ret;
	struct bq20z45_device_info *bq20z45_device;

	BATT_MSG("%s\n", __func__);

	bq20z45_device = i2c_get_clientdata(client);

	/* write to manufacture access with sleep command */
	ret = i2c_smbus_write_word_data_retry(bq20z45_device->client,
		bq20z45_data[REG_MANUFACTURER_DATA].addr,
		MANUFACTURER_ACCESS_SLEEP);
	if (ret < 0) {
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c write for %d failed\n",
			__func__, MANUFACTURER_ACCESS_SLEEP);
		return -EINVAL;
	}

	atomic_set(&bq20z45_device->is_in_suspend, 1);
	wake_up(&bq20z45_device->wait_q_update);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(gpio_to_irq(bq20z45_device->charger_pdata->ac_present_gpio));
		enable_irq_wake(gpio_to_irq(bq20z45_device->charger_pdata->stat2_gpio));
		enable_irq_wake(gpio_to_irq(bq20z45_device->charger_pdata->stat1_gpio));
		enable_irq_wake(bq20z45_device->client->irq);
	}

	return 0;
}

/* any smbus transaction will wake up bq20z75 */
static int bq20z45_resume(struct i2c_client *client)
{
	struct bq20z45_device_info *bq20z45_device;
	bq20z45_device = i2c_get_clientdata(client);

	BATT_MSG("%s\n", __func__);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(gpio_to_irq(bq20z45_device->charger_pdata->ac_present_gpio));
		disable_irq_wake(gpio_to_irq(bq20z45_device->charger_pdata->stat2_gpio));
		disable_irq_wake(gpio_to_irq(bq20z45_device->charger_pdata->stat1_gpio));
		disable_irq_wake(bq20z45_device->client->irq);
	}

	atomic_set(&bq20z45_device->is_in_suspend, 0);
	wake_up(&bq20z45_device->wait_q_update);

	return 0;
}
#endif

static const struct i2c_device_id bq20z45_id[] = {
	{ "bq20z45-battery", 0 },
	{},
};

static struct i2c_driver bq20z45_battery_driver = {
	.probe	= bq20z45_probe,
	.remove = bq20z45_remove,
#if defined(CONFIG_PM)
	.suspend = bq20z45_suspend,
	.resume = bq20z45_resume,
#endif
	.id_table = bq20z45_id,
	.driver = {
		.name	= "bq20z45-battery",
	},
};

static int __init bq20z45_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq20z45_battery_driver);
	if (ret)
		dev_err(&bq20z45_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(bq20z45_battery_init);

static void __exit bq20z45_battery_exit(void)
{
	i2c_del_driver(&bq20z45_battery_driver);
}
module_exit(bq20z45_battery_exit);

MODULE_AUTHOR("Pegatron Corporation");
MODULE_DESCRIPTION("BQ20z45 battery monitor driver");
MODULE_LICENSE("GPL");
