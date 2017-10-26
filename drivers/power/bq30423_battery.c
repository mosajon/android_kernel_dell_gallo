/*
 * drivers/power/bq30423_battery.c
 *
 * Gas Gauge driver for TI's BQ30423
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
#include <mach/gpio.h>
#include "../../arch/arm/mach-tegra/board-gallo.h"

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c/cg7216am.h>
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
	REG_MANUFACTURER_NAME,
	REG_CHARGE_CURRENT,
	REG_CHARGE_VOLTAGE,
	REG_MAX
};

#define BATTERY_RETRY_MAX	3
static unsigned i2c_access_timeout = 500; /* in ms */

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_INIT_DONE	0x80
#define BATTERY_DISCHARGING	0x40
#define BATTERY_FULL_CHARGED	0x20
#define BATTERY_FULL_DISCHARGED	0x10
#define BATTERY_STATUS_TCA	0x4000
#define BATTERY_STATUS_OTA	0x1000

#define BATTERY_TEMP_CRITICAL_HIGH	850

#define KMAXTEMPERATURE_DISCHG 650
#define KMAXTEMPERATURE_CHG 850
#define KMINTEMPERATURE 0

static unsigned int cache_time = 5000;

#define temperature_cmp(temp, low, high) (((temp < high) && (temp > low)) ? 0 : 1)

/* #define DEBUG */
#ifdef DEBUG
	#define BATT_MSG(x...) pr_err("<BATT>" x)
	#define LOG_FUNC() printk("<%s>\n", __FUNCTION__)
#else
	#define BATT_MSG(x...) do {} while (0);
	#define LOG_FUNC() do {} while (0)
#endif



#define BATT(x...) do {} while (0);

#define BQ30423_DATA(_psp, _addr, _min_value, _max_value)	\
{							\
		.psp = POWER_SUPPLY_PROP_##_psp,		\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
}

#define BQ30423_DATAS(_addr, _min_value, _max_value)	\
{							\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
}

static int bq30423_polling_thread(void *data);

static struct bq30423_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq30423_data[] = {
	[REG_MANUFACTURER_DATA] = BQ30423_DATA(PRESENT, 0x00, 0, 65535),
	[REG_TEMPERATURE]       = BQ30423_DATA(TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE]           = BQ30423_DATA(VOLTAGE_NOW, 0x09, 0, 65535),
	[REG_CURRENT]           = BQ30423_DATA(CURRENT_NOW, 0x0A, -32768, 32767),
	[REG_TIME_TO_EMPTY]     = BQ30423_DATA(TIME_TO_EMPTY_AVG, 0x12, 0, 65535),
	[REG_TIME_TO_FULL]      = BQ30423_DATA(TIME_TO_FULL_AVG, 0x13, 0, 65535),
	[REG_STATUS]            = BQ30423_DATA(STATUS, 0x16, 0, 65535),
	/*[REG_CYCLE_COUNT]       = BQ30423_DATA(CYCLE_COUNT, 0x17, 0, 65535),*/
	[REG_CAPACITY]          = BQ30423_DATA(CAPACITY, 0x0d, 0, 100),
	[REG_SERIAL_NUMBER]     = BQ30423_DATA(SERIAL_NUMBER, 0x1C, 0, 65535),
	[REG_MANUFACTURER_NAME]     = BQ30423_DATAS(0x20, 0, 0),
	[REG_CHARGE_CURRENT]     = BQ30423_DATAS(0x14, 0, 65535),
	[REG_CHARGE_VOLTAGE]     = BQ30423_DATAS(0x15, 0, 65535),
};

static enum power_supply_property bq30423_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	/*POWER_SUPPLY_PROP_CYCLE_COUNT,*/
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP
};

static enum power_supply_property bq30423_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property bq30423_properties_sequence[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

#define PROPERTY_SEQUENCE_SIZE            ARRAY_SIZE(bq30423_properties_sequence)
#define GET_PROPERTY_SEQUENCE_START(psp) ((psp == bq30423_properties_sequence[0]) ? 1 : 0)
#define GET_PROPERTY_SEQUENCE_END(psp)   ((psp == bq30423_properties_sequence[PROPERTY_SEQUENCE_SIZE-1]) ? 1 : 0)

static struct bq30423_status_backup
{
	bool ac_status;
	int batt_voltage;
	int batt_capacity;
	int batt_temperature;
	int manufactory_data;
	int register_status;
	int batt_status;
} batt_status_backup;

static char *power_supplied_to[] = {
	"battery",
};


struct charge_enable_timeout_struct {
	bool is_active;
	unsigned long timeout_jiffies;
};

static struct bq30423_device_info {
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
	unsigned long update_time;
	atomic_t is_in_suspend;
	unsigned long ac_latest_update_time;

	bool is_i2c_ok;
	bool is_i2c_ok_old;
	bool is_ac_changed;
	bool is_over_dischg_recovery_fail;
	bool ce_status;
	bool ce_status_must_update;
	bool is_first_boot;
	bool is_resume;
	char manufacture_name[I2C_SMBUS_BLOCK_MAX+1];
	struct charge_enable_timeout_struct dead_bat_recovery;
	struct charge_enable_timeout_struct over_discharge_bat_recovery;
	int default_timeout_dead_batt;
	int default_timeout_over_dischg_batt;

	bool ac_status;
	int sequence_index;
	atomic_t backup_lock;
} *bq30423_device;

static int bq30423_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int bq30423_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static ssize_t bq30423_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);
static void bq30423_battery_status_update(struct bq30423_device_info *info);
static int bq30423_get_battery_serial_number(union power_supply_propval *val);
static int bq30423_get_manufacture_name(char *name);
static int bq30423_get_voltage(void);
static int bq30423_get_temperature(void);
static int bq30423_get_register_status(void);
static s32 i2c_smbus_read_word_data_retry(struct i2c_client *client, u8 command);
static void start_over_discharge_bat_recovery(void);

enum supply_type {
	SUPPLY_TYPE_BATTERY = 0,
	SUPPLY_TYPE_AC,
};

static struct power_supply bq30423_supply[] = {
	[SUPPLY_TYPE_BATTERY] = {
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq30423_properties,
		.num_properties	= ARRAY_SIZE(bq30423_properties),
		.get_property	= bq30423_get_property,
	},
	[SUPPLY_TYPE_AC] = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = power_supplied_to,
		.num_supplicants = ARRAY_SIZE(power_supplied_to),
		.properties = bq30423_ac_properties,
		.num_properties = ARRAY_SIZE(bq30423_ac_properties),
		.get_property = bq30423_ac_get_property,
	},
};

static void check_property_sequence(enum power_supply_property batt_prop)
{
	BATT_MSG("+%s\n", __func__);

	if (batt_prop == bq30423_properties_sequence[bq30423_device->sequence_index])	{
		BATT_MSG("%s:sequence in : %d\n", __func__, bq30423_device->sequence_index);
		bq30423_device->sequence_index++;

		if (GET_PROPERTY_SEQUENCE_START(batt_prop))	{
			atomic_set(&bq30423_device->backup_lock, 1);
		} else if (GET_PROPERTY_SEQUENCE_END(batt_prop)) {
			atomic_set(&bq30423_device->backup_lock, 0);
			bq30423_device->sequence_index = 0;
		}
	} else {
		BATT_MSG("%s:fail sequence : %d\n", __func__, bq30423_device->sequence_index);
		atomic_set(&bq30423_device->backup_lock, 0);
		bq30423_device->sequence_index = 0;

		if (GET_PROPERTY_SEQUENCE_START(batt_prop))	{
			atomic_set(&bq30423_device->backup_lock, 1);
			bq30423_device->sequence_index++;
		}
	}

	BATT_MSG("-%s\n", __func__);
}

static bool is_support_smart_charging(void)
{
	/*
	printk(KERN_DEBUG "%s:  sb_imp = 0x%x \n", __func__, bq30423_device->charger_pdata->sb_imp);
	*/
	return (bq30423_device->charger_pdata->sb_imp != NULL);
}

static void set_ce_enable(bool enable)
{
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	LOG_FUNC();

	if (bq30423_device->ce_status_must_update || bq30423_device->ce_status != enable) {
		bq30423_device->ce_status_must_update = false;
		sb_impl->ce_enable(enable);
		bq30423_device->ce_status = enable;
	}
}

static bool is_dead_battery_recovery_active(void){
	struct charge_enable_timeout_struct *dead_recovery = &bq30423_device->dead_bat_recovery;

	if (dead_recovery->is_active) {
		return true;
	} else {
		return false;
	}
}

static bool is_battery_fine_for_charging(void){
	s32 temp, status;
	if (bq30423_device->charger_pdata->is_ac_online() == false)
		return false;

	temp = bq30423_device->batt_temperature;
	status = bq30423_device->register_status;

	if (status < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, REG_STATUS);
		return -EINVAL;
	}
	BATT_MSG("%s:  temp=%d, status=0x%04x  \n", __func__, temp, status);

	if (temp < BATTERY_TEMP_CRITICAL_HIGH &&
		(status & BATTERY_FULL_CHARGED) == 0 &&
		(status & BATTERY_STATUS_OTA) == 0 &&
		(status & BATTERY_STATUS_TCA) == 0) {

		return true;
	}

	return false;
}

static void start_dead_bat_recovery(void)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->dead_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	int timeout = 0;
	LOG_FUNC();

	if (!bq30423_device->is_i2c_ok) {
		printk(KERN_INFO "%s:  Dead battery recovery starting. \n", __func__);
		timeout = sb_impl->get_over_dischg_bat_recovery_timeout();
		if (timeout == 0) {
			printk(KERN_INFO "%s:  stop dead battery recovery because over-discharging timeout. \n", __func__);
			return;
		}

		timeout = sb_impl->get_dead_bat_recovery_timeout();
		if (timeout > 0) {
			sb_impl->store_dead_bat_recovery_timeout(timeout);
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_DEAD_BATTERY, 0);
			recovery->is_active = true;
			recovery->timeout_jiffies = jiffies + timeout*HZ;
			set_ce_enable(true);
		}
	}
}

static void end_dead_bat_recovery(bool is_need_re_count)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->dead_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	/* LOG_FUNC(); */

	if (recovery->is_active) {
		printk(KERN_INFO "%s:  Dead battery recovery ended. is_need_re_count = %d \n",
						 __func__, is_need_re_count);
		recovery->is_active = false;

		if (is_need_re_count) {
			sb_impl->store_dead_bat_recovery_timeout(bq30423_device->default_timeout_dead_batt);
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_DEAD_BATTERY, 0);
		} else {
			sb_impl->store_dead_bat_recovery_timeout(0);
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_DEAD_BATTERY, 1);
		}

	}
}

static void check_dead_bat_recovery(void)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->dead_bat_recovery;
	unsigned long remain_jiffies;
	int remain_sec;
	/* LOG_FUNC(); */

	if (recovery->is_active) {
		remain_jiffies = recovery->timeout_jiffies - jiffies;
		remain_sec = jiffies_to_msecs(remain_jiffies)/1000;
		printk(KERN_INFO "%s:  is_active=%d,  timeout=%d s, timeout_jiffies = %u, jiffies=%u \n",
				 __func__, recovery->is_active, remain_sec, (unsigned int)recovery->timeout_jiffies, (unsigned int)jiffies);

		if (bq30423_device->is_i2c_ok) {
			printk(KERN_INFO "%s:  Dead battery recovery successfully. \n", __func__);
			end_dead_bat_recovery(true);
		} else {
			if (time_after_eq(jiffies, recovery->timeout_jiffies)) {
				printk(KERN_INFO "%s:  Dead battery recovery fail. \n", __func__);
				set_ce_enable(false);
				end_dead_bat_recovery(false);
			}
		}
	}
}


static void suspend_dead_bat_recovery(void)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->dead_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	unsigned long remain_jiffies;
	int remain_sec;
	/* LOG_FUNC(); */

	if (recovery->is_active) {
		remain_jiffies = recovery->timeout_jiffies - jiffies;
		remain_sec = jiffies_to_msecs(remain_jiffies)/1000;
		sb_impl->store_dead_bat_recovery_timeout(remain_sec);
		printk(KERN_INFO "%s:  Dead battery: remain time=%d s \n", __func__, remain_sec);
	}
}

static void resume_dead_bat_recovery(void)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->dead_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	unsigned long remain_jiffies;
	int remain_sec;
	/* LOG_FUNC(); */

	if (recovery->is_active) {
		remain_sec =  sb_impl->get_dead_bat_recovery_timeout();
		remain_jiffies = remain_sec*HZ;
		if (remain_jiffies == 0) {
			printk(KERN_INFO "%s:  Dead battery recovery failed by MCU. \n", __func__);
			end_dead_bat_recovery(false);
		} else {
			recovery->timeout_jiffies = jiffies + remain_jiffies;
			printk(KERN_INFO "%s:  is_active=%d, remain time=%d \n",
					__func__, recovery->is_active, remain_sec);
		}
	}
}

static void start_over_discharge_bat_recovery(void)
{
	int bat_voltage = 0;
	struct charge_enable_timeout_struct *recovery = &bq30423_device->over_discharge_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	int timeout = 0;

	if (recovery->is_active) {
		return;
	}

	LOG_FUNC();
	if (bq30423_device->is_i2c_ok) {
		bat_voltage = bq30423_device->batt_voltage;

		if (bat_voltage < sb_impl->over_discharge_criteria) {
			printk(KERN_INFO "%s:  Over-Discharging battery recovery starting. bat_voltage=%d \n", __func__, bat_voltage);
			timeout = sb_impl->get_over_dischg_bat_recovery_timeout();
			if (timeout > 0) {
				sb_impl->store_over_dischg_bat_recovery_timeout(timeout);
				recovery->is_active = true;
				sb_impl->set_judgement_finished(JUDGEMENT_FOR_OVER_DISCHARGE_BATTERY, 0);
				recovery->timeout_jiffies = jiffies + timeout*HZ;
				set_ce_enable(true);
			}
		}
	}
}

static void end_over_discharge_bat_recovery(bool is_need_re_count)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->over_discharge_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	/* LOG_FUNC(); */
	if (recovery->is_active) {
		printk(KERN_INFO "%s:  Over-Discharging battery recovery ended. is_need_re_count = %d \n",
						 __func__, is_need_re_count);
		recovery->is_active = false;
		if (is_need_re_count) {
			sb_impl->store_over_dischg_bat_recovery_timeout(bq30423_device->default_timeout_over_dischg_batt);
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_OVER_DISCHARGE_BATTERY, 0);
		} else {
			sb_impl->store_over_dischg_bat_recovery_timeout(0);
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_OVER_DISCHARGE_BATTERY, 1);
		}
	}
}

static void check_over_discharge_bat_recovery(void)
{
	int bat_voltage = 0;
	struct charge_enable_timeout_struct *recovery = &bq30423_device->over_discharge_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	unsigned long remain_jiffies;
	int remain_sec;
	/* LOG_FUNC(); */

	if (!recovery->is_active) {
		return;
	}

	if (bq30423_device->is_i2c_ok) {
		bat_voltage = bq30423_device->batt_voltage;
		remain_jiffies = recovery->timeout_jiffies - jiffies;
		remain_sec = jiffies_to_msecs(remain_jiffies)/1000;

		printk(KERN_INFO "%s:  is_active=%d, criteria= %d, bat_voltage=%d,  timeout=%ds, timeout_jiffies = %u, jiffies=%u \n",
				__func__, recovery->is_active, sb_impl->over_discharge_criteria, bat_voltage, remain_sec,
				(unsigned int)recovery->timeout_jiffies, (unsigned int)jiffies);

		if (bat_voltage >= sb_impl->over_discharge_criteria) {
			printk(KERN_INFO "%s:  Over-discharging battery recovery successfully. \n", __func__);
			end_over_discharge_bat_recovery(true);
		} else {
			printk(KERN_INFO "%s: jiffies=%u, recovery->timeout_jiffies= %u \n",
					 __func__, (unsigned int)jiffies, (unsigned int)recovery->timeout_jiffies);

			if (time_after_eq(jiffies, recovery->timeout_jiffies)) {
				printk(KERN_INFO "%s:  Over-discharging battery recovery fail. \n", __func__);
				set_ce_enable(false);
				end_over_discharge_bat_recovery(false);
				bq30423_device->is_over_dischg_recovery_fail = true;
			}
		}
	} else {
		end_over_discharge_bat_recovery(true);
	}

}


static void suspend_over_dischg_bat_recovery(void)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->over_discharge_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	unsigned long remain_jiffies;
	int remain_sec;
	/* LOG_FUNC(); */

	if (recovery->is_active) {
		remain_jiffies = recovery->timeout_jiffies - jiffies;
		remain_sec = jiffies_to_msecs(remain_jiffies)/1000;
		sb_impl->store_over_dischg_bat_recovery_timeout(remain_sec);
		printk(KERN_INFO "%s: Over-discharging: remain time=%d s \n", __func__, remain_sec);
	}
}

static void resume_over_dischg_bat_recovery(void)
{
	struct charge_enable_timeout_struct *recovery = &bq30423_device->over_discharge_bat_recovery;
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
	unsigned long remain_jiffies;
	int remain_sec;
	/* LOG_FUNC(); */

	if (recovery->is_active) {
		remain_sec = sb_impl->get_over_dischg_bat_recovery_timeout();
		remain_jiffies = remain_sec * HZ;
		if (remain_jiffies == 0) {
			printk(KERN_INFO "%s:  Over-discharging recovery failed by MCU. \n", __func__);
			end_over_discharge_bat_recovery(false);
		} else {
			recovery->timeout_jiffies = jiffies + remain_jiffies;
			printk(KERN_INFO "%s:  is_active=%d, remain time=%d \n",
					__func__, recovery->is_active, remain_sec);
		}
	}
}

static bool validate_battery_vendor(void)
{
	struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;

	LOG_FUNC();

	if (bq30423_device->is_i2c_ok) {
		if (!sb_impl->validate_manufacture(bq30423_device->manufacture_name)) {
			return false;
		}
	}
	return true;
}

static s32 i2c_smbus_read_block_data_retry(struct i2c_client *client, u8 command, u8 *values)
{
	int retry = 0;
	s32 res = 0;
	unsigned long timeout, access_time;
	access_time = jiffies;
	timeout = jiffies + msecs_to_jiffies(i2c_access_timeout);

	for (retry = 0; retry < BATTERY_RETRY_MAX && time_before(access_time, timeout); retry++) {
		res = i2c_smbus_read_i2c_block_data(client, command, I2C_SMBUS_BLOCK_MAX, values);
		if (res >= 0)
			break;
		printk("%s-retry:%d\n", __FUNCTION__, retry);
		access_time = jiffies;
	}

	if (res < 0) {
		bq30423_device->is_i2c_ok = false;
	} else {
		bq30423_device->is_i2c_ok = true;
	}
	return res;
}

static s32 i2c_smbus_read_word_data_retry(struct i2c_client *client, u8 command)
{
	int retry = 0;
	s32 res = 0;
	unsigned long timeout, access_time;
	access_time = jiffies;
	timeout = jiffies + msecs_to_jiffies(i2c_access_timeout);

	for (retry = 0; retry < BATTERY_RETRY_MAX && time_before(access_time, timeout); retry++) {
		res = i2c_smbus_read_word_data(client, command);
		if(res >= 0)
			break;
		printk("%s-retry:%d\n",__FUNCTION__,retry);
		access_time = jiffies;
	}

	if (res < 0) {
		bq30423_device->is_i2c_ok = false;
	} else {
		bq30423_device->is_i2c_ok = true;
	}
	return res;
}

static s32 i2c_smbus_write_word_data_retry(struct i2c_client *client, u8 command, u16 value)
{
	int retry = 0;
	s32 res = 0;
	unsigned long timeout, access_time;
	access_time = jiffies;
	timeout = jiffies + msecs_to_jiffies(i2c_access_timeout);

	for (retry = 0; retry < BATTERY_RETRY_MAX  && time_before(access_time, timeout); retry++) {
		res = i2c_smbus_write_word_data(client, command, value);
		if(res >= 0)
			break;
		printk("%s-retry:%d\n",__FUNCTION__,retry);
		access_time = jiffies;
	}

	if (res < 0) {
		bq30423_device->is_i2c_ok = false;
	} else {
		bq30423_device->is_i2c_ok = true;
	}

	return res;
}

static s32 i2c_smbus_read_byte_data_retry(struct i2c_client *client, u8 command)
{
	int retry = 0;
	s32 res = 0;
	unsigned long timeout, access_time;
	access_time = jiffies;
	timeout = jiffies + msecs_to_jiffies(i2c_access_timeout);

	for (retry = 0; retry < BATTERY_RETRY_MAX && time_before(access_time, timeout); retry++) {
		res = i2c_smbus_read_byte_data(client, command);
		if(res >= 0)
			break;
		printk("%s-retry:%d\n",__FUNCTION__,retry);
		access_time = jiffies;
	}

	if (res < 0) {
		bq30423_device->is_i2c_ok = false;
	} else {
		bq30423_device->is_i2c_ok = true;
	}

	return res;
}

static int bq30423_get_ac_status(void)
{
	return bq30423_device->charger_pdata->is_ac_online();
}

static int bq30423_is_nce_asserted(void)
{
	return gpio_get_value(bq30423_device->charger_pdata->nce_gpio);
}

static int bq30423_ac_get_property(struct power_supply *psy,
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
		dev_err(&bq30423_device->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	BATT_MSG("-%s: psp:%d, val:%d\n", __func__, psp, val->intval);
	return 0;
}

static int bq30423_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq30423_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq30423_data[REG_MANUFACTURER_DATA].max_value) {

		ret &= 0x0F00;
		ret >>= 8;
		if (psp == POWER_SUPPLY_PROP_PRESENT) {
			if (ret == 0x0E)
				/* battery removed */
				val->intval = 0;
			else
				val->intval = 1;
		} else if (psp == POWER_SUPPLY_PROP_HEALTH) {
			if (ret == 0x0A)
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

static int bq30423_get_register_status(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_STATUS].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for battery status "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq30423_data[REG_STATUS].min_value &&
	    ret <= bq30423_data[REG_STATUS].max_value) {
		return ret;
	}
	return 0;
}

static int bq30423_get_manufacturer_data(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq30423_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq30423_data[REG_MANUFACTURER_DATA].max_value) {
		return ret;
	}
	return 0;
}

inline int manufacturer_data_2_status(int data)
{
	int ret = data;

	if (ret >= bq30423_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq30423_data[REG_MANUFACTURER_DATA].max_value) {

		/* Mask the upper nibble of 2nd byte and
		 * lower byte of response then
		 * shift the result by 8 to get status*/
		ret &= 0xFF00;
		ret >>= 8;
		return ret;
	}
	return 0;
}

inline int manufacturer_data_2_present(int data)
{
	int ret = data;

	if (ret >= bq30423_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq30423_data[REG_MANUFACTURER_DATA].max_value) {
		ret &= 0x0F00;
		ret >>= 8;
		if (ret == 0x0E)
			/* battery removed */
			return 0;
		else
			return 1;
	}

	return 0;
}

inline int manufacturer_data_2_health(int data)
{
	int ret = data;

	if (ret >= bq30423_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq30423_data[REG_MANUFACTURER_DATA].max_value) {

		ret &= 0x0F00;
		ret >>= 8;

		if (ret == 0x0A)
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

static int bq30423_get_manufacturer_status(void)
{
	int ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}

	if (ret >= bq30423_data[REG_MANUFACTURER_DATA].min_value &&
	    ret <= bq30423_data[REG_MANUFACTURER_DATA].max_value) {

		/* Mask the upper nibble of 2nd byte and
		 * lower byte of response then
		 * shift the result by 8 to get status*/
		ret &= 0xFF00;
		ret >>= 8;
		return ret;
	}
	return 0;
}

static int bq30423_get_pack_status(void)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data_retry(bq30423_device->client,
		bq30423_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}
	BATT_MSG(" opearation = %d\n", ret);

	return ret;
}

static int bq30423_get_battery_charging_status(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_MANUFACTURER_DATA].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for battery presence "
			"failed\n", __func__);
		return -EINVAL;
	}
	BATT_MSG(" charging status for FET= %d\n", (ret & 0xF000)>>12);
	BATT_MSG(" charging status for mode= %d\n", (ret & 0x0F00)>>8);
	BATT_MSG(" charging status for permanent failure= %d\n", (ret & 0x3000)>>12);
	BATT_MSG(" charging status for AC present= %d\n", (ret & 0x000F));

	return ret;
}

static int get_psy_prop_status(int value)
{
	/*set default state*/
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int is_ac_present = bq30423_device->ac_status;

	if (value == -EINVAL) {
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
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

			if (is_support_smart_charging()) {
				BATT_MSG("%s:  BATTERY_FULL_CHARGED. \n", __func__);
				set_ce_enable(false);
			}
		} else if ((value & BATTERY_DISCHARGING) == 0) {
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (value & BATTERY_DISCHARGING) {
			/*battery status update has latency*/
			if (time_before(jiffies, bq30423_device->ac_latest_update_time + msecs_to_jiffies(cache_time))) {
				printk(KERN_INFO "%s:  Use cache status, jiffies = %u,  ac_latest_update_time=%u \n", __func__,
						(unsigned int)jiffies, (unsigned int)bq30423_device->ac_latest_update_time);
				status = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				status = POWER_SUPPLY_STATUS_DISCHARGING;

				printk(KERN_ERR"%s:  AC in, but discharging \n", __func__);
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

	BATT_MSG(" %s, status:%d, ac_present:%d, value:0x%04x\n", __func__, status, is_ac_present, value);
	return status;
}

static int bq30423_get_psp(int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
	s32 ret_capacity;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[reg_offset].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, reg_offset);
		return -EINVAL;
	}

	if (reg_offset == REG_CURRENT) {
		ret_capacity = (ret & 0x7FFF);
		if (ret & 0x8000) {
			BATT_MSG(" discharging, capacity_ret = %d \n", ret);
			ret_capacity = (-1)*(ret_capacity);
		}
		ret = ret_capacity;
	}

	if (ret >= bq30423_data[reg_offset].min_value &&
	    ret <= bq30423_data[reg_offset].max_value) {
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

static int bq30423_get_capacity_psp(union power_supply_propval *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data_retry(bq30423_device->client,
		bq30423_data[REG_CAPACITY].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CAPACITY);
		return -EINVAL;
	}

	/* bq30423 spec says that this can be >100 %
	 * even if max value is 100 % */
	val->intval = ((ret >= 100) ? 100 : ret);

	return 0;
}

static char bq30423_serial[9] = {0};
static int bq30423_get_battery_serial_number(union power_supply_propval *val)
{
	int ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_SERIAL_NUMBER].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_SERIAL_NUMBER);
		return -EINVAL;
	}

	ret = scnprintf(bq30423_serial, sizeof(bq30423_serial)-1, "%d", ret);
	if(val)	
		val->strval = bq30423_serial;

	return 0;
}

static int bq30423_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	BATT_MSG("+%s: psp:%d\n", __func__, psp);

	if (!bq30423_device->is_i2c_ok) {
		dev_err(&bq30423_device->client->dev,
				"%s: i2c error, can't update property \n", __func__);
		if (psp == POWER_SUPPLY_PROP_STATUS) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
		return -ENODEV;
	}

	check_property_sequence(psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = manufacturer_data_2_present(batt_status_backup.manufactory_data);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = manufacturer_data_2_health(batt_status_backup.manufactory_data);
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
		dev_err(&bq30423_device->client->dev,
				"%s: INVALID property\n", __func__);
		return -EINVAL;

	}

	BATT_MSG("-%s: psp:%d, value:%d\n", __func__, psp, val->intval);
	return 0;
}

static ssize_t bq30423_battery_store_ce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int state = 0;
	sscanf(buf, "%d", &state);
	if ((state != 0) && (state != 1)) {
		pr_err("<BATT> %s input value is incorrect !!!\n", __func__);
		return 0;
	}
	if (bq30423_device->charger_pdata->nce_gpio != -1) {
		gpio_direction_output(bq30423_device->charger_pdata->nce_gpio, state);
	} else if (is_support_smart_charging()) {
		set_ce_enable(state);
	} else {
		pr_err("<BATT> %s can NOT set CE !!!\n", __func__);
		return 0;
	}

	return count;
}

#define bq30423_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO,},	\
	.show = bq30423_battery_show_property,					\
	.store = NULL,								\
}

#define bq30423_BATTERY_ATTR_ATS(_name, _store)                         \
{                                       \
    .attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR,},    \
	.show = bq30423_battery_show_property,                  \
	.store = _store,                                \
}

static struct device_attribute bq30423_battery_attrs[] = {
	bq30423_BATTERY_ATTR(batt_vol),
	bq30423_BATTERY_ATTR(batt_temp),
	bq30423_BATTERY_ATTR(charge_current),
	bq30423_BATTERY_ATTR(charge_voltage),
	bq30423_BATTERY_ATTR(manufactory_status),
	bq30423_BATTERY_ATTR_ATS(charger_ce, bq30423_battery_store_ce),
	bq30423_BATTERY_ATTR(manufacture_name),
};

enum {
	BATT_VOL = 0,
	BATT_TEMP,
	CHARGE_CURRENT,
	CHARGE_VOLTAGE,
	MANUFACTORY_STATUS,
	CHARGER_CE,
	MANUFACTURE_NAME,
};

static int bq30423_battery_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(bq30423_battery_attrs); i++) {
		rc = device_create_file(dev, &bq30423_battery_attrs[i]);
		if (rc)
			goto bq30423_attrs_failed;
	}
	goto succeed;
bq30423_attrs_failed:
	while (i--)
		device_remove_file(dev, &bq30423_battery_attrs[i]);
succeed:
	return rc;
}

static void bq30423_battery_remove_attrs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(bq30423_battery_attrs); i++)
		device_remove_file(dev, &bq30423_battery_attrs[i]);
}

static ssize_t bq30423_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - bq30423_battery_attrs;

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       bq30423_device->batt_voltage);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       bq30423_device->batt_temperature);
		break;
	case CHARGE_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   bq30423_device->charge_current);
		break;
	case CHARGE_VOLTAGE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   bq30423_device->charge_voltage);
		break;
	case MANUFACTORY_STATUS:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%08x\n",
				manufacturer_data_2_status(bq30423_device->manufactory_data));
		break;
	case CHARGER_CE:
		if (bq30423_device->charger_pdata->nce_gpio != -1) {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       gpio_get_value(bq30423_device->charger_pdata->nce_gpio));
		} else if (is_support_smart_charging()) {
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       bq30423_device->ce_status);
		} else {
			i = -EINVAL;
		}
		break;
	case MANUFACTURE_NAME:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", bq30423_device->manufacture_name);
		break;
	default:
		i = -EINVAL;
	}

	BATT_MSG("%s:attr_off:%d, value:%s\n", __func__, off, buf);
	return i;
}

static int bq30423_get_manufacture_name(char *name)
{
	s32 ret;
	int len = 0;
	char values[I2C_SMBUS_BLOCK_MAX+1];

	ret = i2c_smbus_read_block_data_retry(bq30423_device->client,
		bq30423_data[REG_MANUFACTURER_NAME].addr, values);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_MANUFACTURER_NAME);
		return -EINVAL;
	}

	len = values[0]; /* values[0] is the length of manufacture name */
	name[len] = '\0'; /* end of string */
	strncpy(name, values+1, len);

	dev_dbg(&bq30423_device->client->dev, "%s: name='%s' \n",
			 __func__, name);
	return ret;
}


static int bq30423_get_voltage(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_VOLTAGE].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_VOLTAGE);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_VOLTAGE].max_value &&
		ret >= bq30423_data[REG_VOLTAGE].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read voltage failed!!!\n");
		return 0;
	}
}

static int bq30423_get_charge_current(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_CHARGE_CURRENT].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CHARGE_CURRENT);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_CHARGE_CURRENT].max_value &&
		ret >= bq30423_data[REG_CHARGE_CURRENT].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read charge current failed!!!\n");
		return 0;
	}
}

static int bq30423_get_charge_voltage(void)
{
	s32 ret;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_CHARGE_VOLTAGE].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CHARGE_VOLTAGE);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_CHARGE_VOLTAGE].max_value &&
		ret >= bq30423_data[REG_CHARGE_VOLTAGE].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read charge voltage failed!!!\n");
		return 0;
	}
}

static int bq30423_get_temperature(void)
{
	s32 ret = 0;

	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_TEMPERATURE].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_TEMPERATURE);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_TEMPERATURE].max_value &&
		ret >= bq30423_data[REG_TEMPERATURE].min_value) {
		ret -= 2731;
		return ret;
	} else {
		pr_err("<BATT> read temperature failed!!!\n");
		return 0;
	}
}

static int bq30423_get_current(void)
{
	s32 ret;
	s32 ret_capacity;
	ret = i2c_smbus_read_word_data_retry(bq30423_device->client,
		bq30423_data[REG_CURRENT].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CURRENT);
		return -EINVAL;
	}

	ret_capacity = (ret & 0x7FFF);

	if (ret & 0x8000) {
		BATT_MSG(" discharging, capacity_ret = %d \n", ret);
		ret_capacity = (-1)*(ret_capacity);
	}
	if (ret_capacity <= bq30423_data[REG_CURRENT].max_value &&
		ret_capacity >= bq30423_data[REG_CURRENT].min_value) {
		return ret_capacity;
	} else {
		pr_err("<BATT> read current failed!!!\n");
		return 0;
	}
}

static int bq30423_get_capacity(void)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data_retry(bq30423_device->client,
		bq30423_data[REG_CAPACITY].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_CAPACITY);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_CAPACITY].max_value &&
		ret >= bq30423_data[REG_CAPACITY].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read capacity failed -- out of range ret = %d!!!\n", ret);
		return 0;
	}
}

static int bq30423_get_time_to_empty_avg(void)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data_retry(bq30423_device->client,
		bq30423_data[REG_TIME_TO_EMPTY].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__,REG_TIME_TO_EMPTY);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_TIME_TO_EMPTY].max_value &&
		ret >= bq30423_data[REG_TIME_TO_EMPTY].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read capacity failed -- out of range ret = %d!!!\n", ret);
		return 0;
	}
}

static int bq30423_get_time_to_full_avg(void)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data_retry(bq30423_device->client,
		bq30423_data[REG_TIME_TO_FULL].addr);
	if (ret < 0) {
		dev_err(&bq30423_device->client->dev, "%s: i2c read for %d "
			"failed\n", __func__, REG_TIME_TO_FULL);
		return -EINVAL;
	}
	if (ret <= bq30423_data[REG_TIME_TO_FULL].max_value &&
		ret >= bq30423_data[REG_TIME_TO_FULL].min_value) {
		return ret;
	} else {
		pr_err("<BATT> read capacity failed -- out of range ret = %d!!!\n", ret);
		return 0;
	}
}

static void bq30423_battery_status_backup(struct bq30423_device_info *batt_info)
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

static void bq30423_battery_status_update(struct bq30423_device_info *info)
{
	BATT_MSG("+%s: \n", __func__);

	info->batt_voltage = bq30423_get_voltage();
	info->batt_temperature = bq30423_get_temperature();
	info->batt_capacity = bq30423_get_capacity();
	info->charge_current = bq30423_get_charge_current();
	info->charge_voltage = bq30423_get_charge_voltage();
	info->manufactory_data = bq30423_get_manufacturer_data();
	info->register_status = bq30423_get_register_status();
	info->ac_status = bq30423_get_ac_status();
	info->batt_status = get_psy_prop_status(info->register_status);

	memset(info->manufacture_name, 0x00, sizeof(info->manufacture_name));
	bq30423_get_manufacture_name(info->manufacture_name);

	bq30423_device->update_time = jiffies;

	BATT_MSG("+%s: backup_lock = %d\n", __func__, atomic_read(&bq30423_device->backup_lock));

	if (!atomic_read(&bq30423_device->backup_lock))	{
		BATT_MSG("-%s: backup_lock = %d\n", __func__, atomic_read(&bq30423_device->backup_lock));
		bq30423_battery_status_backup(info);
	}

	BATT_MSG("ac = %d, status = %d,voltage = %d, temperature = %d, capacity = %d, manufactory_data = %d\n",
		info->ac_status, info->batt_status, info->batt_voltage, info->batt_temperature,
		info->batt_capacity, info->manufactory_data);
	BATT_MSG("-%s: \n", __func__);
}


static void bq30423_sb_main(void)
{
		if (is_support_smart_charging()) {
			struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;

			BATT_MSG("%s:  is_ac_changed=%d,  is_i2c_ok_old=%d, is_i2c_ok=%d, is_first_boot=%d \n", __func__,
					bq30423_device->is_ac_changed, bq30423_device->is_i2c_ok_old,
					bq30423_device->is_i2c_ok, bq30423_device->is_first_boot);
			if (bq30423_device->is_ac_changed ||
				bq30423_device->is_resume ||
				bq30423_device->is_i2c_ok_old != bq30423_device->is_i2c_ok) {
				bq30423_device->is_resume = false;
				if (bq30423_device->is_ac_changed) {
					bq30423_device->is_over_dischg_recovery_fail = false;
				}

				if (!bq30423_device->is_first_boot && bq30423_device->is_ac_changed) {
					sb_impl->store_dead_bat_recovery_timeout(bq30423_device->default_timeout_dead_batt);
					sb_impl->store_over_dischg_bat_recovery_timeout(bq30423_device->default_timeout_over_dischg_batt);
				}
				bq30423_device->is_first_boot = false;

				if (bq30423_device->charger_pdata->is_ac_online()) {
					if (!bq30423_device->is_over_dischg_recovery_fail) {
						start_dead_bat_recovery();
					}
					if (bq30423_device->is_ac_changed) {
						start_over_discharge_bat_recovery();
					}
				} else {
					BATT_MSG("%s:  AC is offline \n", __func__);
					set_ce_enable(false);
					end_dead_bat_recovery(true);
					end_over_discharge_bat_recovery(true);
				}

				bq30423_device->is_i2c_ok_old = bq30423_device->is_i2c_ok;
			}

			if (bq30423_device->charger_pdata->is_ac_online()) {
				if (is_dead_battery_recovery_active()) {
					check_dead_bat_recovery();
				} else {
					if (!bq30423_device->is_over_dischg_recovery_fail &&
						bq30423_device->is_i2c_ok &&
						validate_battery_vendor() &&
						is_battery_fine_for_charging()) {

						start_over_discharge_bat_recovery();
						check_over_discharge_bat_recovery();
						set_ce_enable(true);
					} else {
						end_over_discharge_bat_recovery(true);
						BATT_MSG("%s:  battery status is not good \n", __func__);
						set_ce_enable(false);
					}
				}
			}
		}
}


static int bq30423_polling_thread(void *data)
{
	struct bq30423_device_info *batt_info = data;

	BATT_MSG("+%s\n", __func__);

	while (1) {
		wait_event_timeout(batt_info->wait_q_update,
			batt_info->is_ac_changed && !atomic_read(&bq30423_device->is_in_suspend),
			batt_info->charger_pdata->interval_slow);

		if(!atomic_read(&bq30423_device->is_in_suspend)) {
			bq30423_battery_status_update(batt_info);

			bq30423_sb_main();
			if(batt_info->is_ac_changed) {
				BATT_MSG("%s:ac changed\n", __func__);
				batt_info->is_ac_changed = false;
				power_supply_changed(&bq30423_supply[SUPPLY_TYPE_AC]);
			} else {
				BATT_MSG("%s:battery staus update\n", __func__);

				power_supply_changed(&bq30423_supply[SUPPLY_TYPE_BATTERY]);
			}
		} else {
			BATT_MSG("%s:polling-suspend\n", __func__);
		}


	};

	BATT_MSG("-%s\n", __func__);
	return 0;
}

static irqreturn_t state_change_isr(int irq, void *power_supply)
{
	BATT_MSG("%s irq:%d\n", __func__, irq);

	bq30423_device->ac_latest_update_time = jiffies;
	bq30423_device->is_ac_changed = true;

	wake_up(&bq30423_device->wait_q_update);

	return IRQ_HANDLED;
}

static int bq30423_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc, i, flags;

	if (!client->dev.platform_data) {
		dev_dbg(&client->dev, "<BATT>  no platform data?\n");
		return -EINVAL;
	}

	bq30423_device = kzalloc(sizeof(*bq30423_device), GFP_KERNEL);
	if (!bq30423_device)
		return -ENOMEM;

	memset(bq30423_device, 0, sizeof(*bq30423_device));
	init_waitqueue_head(&bq30423_device->wait_q_update);

	bq30423_device->client = client;
	bq30423_device->update_time = jiffies;
	bq30423_device->charger_pdata = client->dev.platform_data;

	flags = bq30423_device->client->flags;
	bq30423_device->client->flags &= ~I2C_M_IGNORE_NAK;
	bq30423_device->client->flags = flags;
	i2c_set_clientdata(client, bq30423_device);

	if (bq30423_device->charger_pdata->is_ac_online == NULL) {
		BATT_MSG("%s is_ac_online = NULL !!!\n", __func__);
		rc =  -EINVAL;
		goto fail_init;
	}

	rc = bq30423_get_battery_serial_number(0);

	memset(bq30423_device->manufacture_name, 0x00, sizeof(bq30423_device->manufacture_name));
	bq30423_get_manufacture_name(bq30423_device->manufacture_name);

	if (rc < 0) {
		dev_err(&bq30423_device->client->dev,
			"<BATT> %s: no battery present(%d)\n", __func__, rc);

		if (!is_support_smart_charging())
			goto fail_init;
	}
	if (is_support_smart_charging()) {
		struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
		sb_impl->init();
		bq30423_device->default_timeout_dead_batt =
								sb_impl->get_dead_bat_recovery_timeout_default();
		bq30423_device->default_timeout_over_dischg_batt = sb_impl->get_over_dischg_bat_recovery_timeout_default();
	}

	bq30423_device->ac_latest_update_time = jiffies;
	bq30423_device->is_first_boot = true;
	rc = request_irq(gpio_to_irq(bq30423_device->charger_pdata->ac_present_gpio),
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"ac_present_irq", &bq30423_supply[SUPPLY_TYPE_AC]);
	if (rc < 0) {
		dev_err(&bq30423_device->client->dev,
			"%s: request_irq failed(%d)\n", __func__, rc);
		goto fail_irq;
	}

	atomic_set(&bq30423_device->backup_lock, 0);
	bq30423_device->sequence_index = 0;

	bq30423_battery_status_update(bq30423_device);

	for (i = 0; i < ARRAY_SIZE(bq30423_supply); i++) {
		rc = power_supply_register(&client->dev,
			&bq30423_supply[i]);
	if (rc) {
		dev_err(&bq30423_device->client->dev,
				"%s: Failed to register power supply\n",
				 __func__);
			goto fail_power_register;
		}
	}

	bq30423_battery_create_attrs(bq30423_supply[SUPPLY_TYPE_BATTERY].dev);
	dev_info(&bq30423_device->client->dev,
		"%s: battery driver registered\n", client->name);

	device_init_wakeup(&client->dev, 1);

	bq30423_device->is_ac_changed = true;
	bq30423_device->ce_status_must_update = true;
	bq30423_device->polling_thread = kthread_create(bq30423_polling_thread, bq30423_device, "bq30423_polling_thread");
	wake_up_process(bq30423_device->polling_thread);

	return 0;

fail_power_register:
	while (i--)
		power_supply_unregister(&bq30423_supply[i]);

fail_irq:
	free_irq(gpio_to_irq(bq30423_device->charger_pdata->ac_present_gpio), bq30423_device);
	/*
	free_irq(gpio_to_irq(bq30423_device->charger_pdata->stat2_gpio), bq30423_supply);
	*/

fail_init:
	kfree(bq30423_device);
	bq30423_device = NULL;
	return rc;
}

static int bq30423_remove(struct i2c_client *client)
{
	struct bq30423_device_info *bq30423_device;
	int i;

	bq30423_device = i2c_get_clientdata(client);

	bq30423_battery_remove_attrs(bq30423_supply[SUPPLY_TYPE_BATTERY].dev);

	for (i = 0; i < ARRAY_SIZE(bq30423_supply); i++)
		power_supply_unregister(&bq30423_supply[i]);

	free_irq(gpio_to_irq(bq30423_device->charger_pdata->ac_present_gpio), bq30423_device);
	/*
	free_irq(gpio_to_irq(bq30423_device->charger_pdata->stat2_gpio), bq30423_supply);
	*/

	if (bq30423_device) {
		kfree(bq30423_device);
		bq30423_device = NULL;
	}
	return 0;
}

#if defined(CONFIG_PM)

static void bq30423_shutdown(struct i2c_client *client)
{
	struct bq30423_device_info *bq30423_device;
	bq30423_device = i2c_get_clientdata(client);
	BATT_MSG("%s,\n", __func__);

	if (is_support_smart_charging()) {
		struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
		if (!sb_impl->validate_manufacture(bq30423_device->manufacture_name)) 	{
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_VENDOR_NAME, 1);
		} else {
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_VENDOR_NAME, 0);
		}
		suspend_dead_bat_recovery();
		suspend_over_dischg_bat_recovery();
	}
}

static int bq30423_suspend(struct i2c_client *client,
	pm_message_t state)
{
	struct bq30423_device_info *bq30423_device;
	bq30423_device = i2c_get_clientdata(client);

	BATT_MSG("%s,\n", __func__);
	atomic_set(&bq30423_device->is_in_suspend, 1);
	wake_up(&bq30423_device->wait_q_update);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(gpio_to_irq(bq30423_device->charger_pdata->ac_present_gpio));
		/*
		enable_irq_wake(gpio_to_irq(bq30423_device->charger_pdata->stat2_gpio));
		*/
	}
	if (is_support_smart_charging()) {
		struct smart_battery_charging_impl *sb_impl = bq30423_device->charger_pdata->sb_imp;
		if (!sb_impl->validate_manufacture(bq30423_device->manufacture_name)) 	{
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_VENDOR_NAME, 1);
		} else {
			sb_impl->set_judgement_finished(JUDGEMENT_FOR_VENDOR_NAME, 0);
		}
		suspend_dead_bat_recovery();
		suspend_over_dischg_bat_recovery();
	}

	return 0;
}

/* any smbus transaction will wake up bq30423 */
static int bq30423_resume(struct i2c_client *client)
{
	struct bq30423_device_info *bq30423_device;
	bq30423_device = i2c_get_clientdata(client);

	BATT_MSG("%s,\n", __func__);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(gpio_to_irq(bq30423_device->charger_pdata->ac_present_gpio));
		/*
		disable_irq_wake(gpio_to_irq(bq30423_device->charger_pdata->stat2_gpio));
		*/
	}

	if (is_support_smart_charging()) {
		bq30423_device->ce_status_must_update = true;
		resume_dead_bat_recovery();
		resume_over_dischg_bat_recovery();
		bq30423_device->is_resume = true;
	}

	if (bq30423_get_ac_status() != bq30423_device->ac_status) {
		BATT_MSG("%s:current_ac = %d, last_ac = %d\n", __func__, bq30423_get_ac_status(), bq30423_device->ac_status);
		bq30423_device->ac_latest_update_time = jiffies;
		bq30423_device->is_ac_changed = true;
	}

	atomic_set(&bq30423_device->is_in_suspend, 0);
	wake_up(&bq30423_device->wait_q_update);

	return 0;
}
#endif

static const struct i2c_device_id bq30423_id[] = {
	{ "bq30423-battery", 0 },
	{},
};

static struct i2c_driver bq30423_battery_driver = {
	.probe	= bq30423_probe,
	.remove = bq30423_remove,
#if defined(CONFIG_PM)
	.suspend = bq30423_suspend,
	.resume = bq30423_resume,
	.shutdown = bq30423_shutdown,
#endif
	.id_table = bq30423_id,
	.driver = {
		.name	= "bq30423-battery",
	},
};

static int __init bq30423_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq30423_battery_driver);
	if (ret)
		dev_err(&bq30423_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(bq30423_battery_init);

static void __exit bq30423_battery_exit(void)
{
	i2c_del_driver(&bq30423_battery_driver);
}
module_exit(bq30423_battery_exit);

MODULE_AUTHOR("Pegatron Corporation");
MODULE_DESCRIPTION("BQ30423 battery monitor driver");
MODULE_LICENSE("GPL");
