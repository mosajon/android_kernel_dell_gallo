/*
 * cg7216am.c  --  Cypress CG7216AM MCU driver
 *
 * Copyright 2011 Pegatron
 *
 * Author: Andrew Hsiao<andrew_hsiao@pegatron.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/cg7216am.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include "../../../arch/arm/mach-tegra/board-gallo.h"
#include <linux/miscdevice.h>

#define MCU_RETRY_MAX	3
#define MCU_WAKEUP_RETRY_MAX	5
#define MCU_WAKE_DELAY_TIME_MSEC	6
#define MCU_COMMAND_GAP_TIME_MSEC	1

/* Ex: 0x41 */
#define MCU_FW_STR_LEN 4

#define DEBUG

#ifdef DEBUG
#define LOG_FUNC() printk("%s\n", __FUNCTION__)
#define LOG_MSG(x...) pr_err(x)
#else
#define LOG_FUNC() do{}while(0)
#define LOG_MSG(x...) do {} while (0);
#endif

static struct miscdevice cg7216am_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cg7216am",
};

#define CG7216_FIRMWARE_UPDATE_EACH_WRITE 16

#define CG7216AM_REG_AMBER_LED_ON_OFF	0x0
/*OFF:0, ON:1*/
#define CG7216AM_REG_AMBER_LED_MODE	0x1
/*STEADY:0, BLINKING:1*/
#define CG7216AM_REG_AMBER_DURATION_ON	0x2
/*ms:0~0xFF*/
#define CG7216AM_REG_AMBER_DURATION_OFF	0x3
/*ms:0~0xFF*/
#define CG7216AM_REG_RED_LED_ON_OFF	0x10
/*OFF:0, ON:1*/
#define CG7216AM_REG_RED_LED_MODE	0x11
/*STEADY:0, BLINKING:1*/
#define CG7216AM_REG_RED_DURATION_ON	0x12
/*0.1s:0~0xFF*/
#define CG7216AM_REG_RED_DURATION_OFF	0x13
/*0.1s:0~0xFF*/
#define CG7216AM_REG_AMBER_THRESHOLD	0x20
/*BATTERY LEVEL: 0~0x64*/
#define CG7216AM_REG_RED_THRESHOLD	0x21
/*BATTERY LEVEL: 0~0x64*/
#define CG7216AM_REG_CPU_MODE		0x22
/*REF_GPIO:0, ON:1, OFF:2*/
#define CG7216AM_REG_GPIO_POLARITY	0x23
/*CPU_ON_BY_HIGH:0, CPU_ON_BY_LOW:1*/
#define CG7216AM_REG_MCU_MODE_DEBOUNCE	0x24
/*0.1s:0~0xFF*/
#define CG7216AM_REG_MCU_SUSPEND	0x25
/*NORMAL:0, SUSPEND:1*/
#define CG7216AM_REG_LED_OWNER		0x26
/*MCU:0, CPU:1*/
#define CG7216AM_REG_VERSION		0x30
/*VER:x*/
#define CG7216AM_REG_LOW_BAT_ALERT_1	0x40
/*CAPACITY_PERCENTAGE:0-0x64, defautl:5%*/
#define CG7216AM_REG_LOW_BAT_ALERT_2	0x41
/*CAPACITY_PERCENTAGE:0-0x64, default:10%*/
#define CG7216AM_REG_LOW_BAT_ALERT_3	0x42
/*CAPACITY_PERCENTAGE:0-0x64, default:15*/
#define CG7216AM_REG_LOW_TEMP_ALERT	0x50
/*TEMP:0~0xF, default:0*/
#define CG7216AM_REG_HIGH_TEMP_ALERT	0x51
/*TEMP:0~0xFF, default:0x37*/
#define CG7216AM_REG_LOW_TEMP_SHUTDOWN	0x52
/*TEMP:0:-3, default:0*/
#define CG7216AM_REG_HIGH_TEMP_ALERT_DISCHARGING	0x53
/*TEMP:0~0xFF, default:0x3C*/
#define CG7216AM_REG_HIGH_TEMP_SHUTDOWN_DISCHARGING	0x54
/*TEMP:0~0xFF, default:0x41*/
#define CG7216AM_REG_RECHARGE_TEMP_LOW	0x55
/*TEMP:0~0xFF, default:0x5*/
#define CG7216AM_REG_RECHARGE_TEMP_HIGH	0x56
/*TEMP:0~0xFF, default:0x32*/
#define CG7216AM_REG_WHITE_LED_ON_OFF	0x60
/*OFF:0, ON:1*/
#define CG7216AM_REG_WHITE_LED_MODE	0x61
/*STEADY:0, BLINKING:1*/
#define CG7216AM_REG_WHITE_DURATION_ON	0x62
/*ms:0~0xFF*/
#define CG7216AM_REG_WHITE_DURATION_OFF	0x63
/*ms:0~0xFF*/
#define CG7216AM_REG_GREEN_LED_ON_OFF	0x70
/*OFF:0, ON:1*/
#define CG7216AM_REG_GREEN_LED_MODE	0x71
/*STEADY:0, BLINKING:1*/
#define CG7216AM_REG_GREEN_DURATION_ON	0x72
/*ms:0~0xFF*/
#define CG7216AM_REG_GREEN_DURATION_OFF	0x73
/*ms:0~0xFF*/
#define CG7216AM_REG_DISABLE_CHARGING	0x80
/*enable charging: 0, disable charging: 1*/
#define CG7216AM_REG_DEAD_BAT_TIMEOUT	0x81
/* s:0~0xFF */
#define CG7216AM_REG_OVER_DISCHG_BAT_TIMEOUT	0x82
/* 10s:0~0xFF */

#define CG7216AM_REG_JUDGEMENT_FINISHED	0x83
/* 	bit 0: dead battery
	bit 1: over discharging battery
	bit 2: vendor name

	value: 0, un-finished
	value: 1, finished
 */

#define CG7216AM_DATA(_addr, _min_value, _max_value, _default_value)	\
[_addr] =							\
	{								\
			.addr = CG7216AM_REG_##_addr,			\
			.min_value = _min_value,			\
			.max_value = _max_value,			\
			.default_value = _default_value			\
	}

enum cg7216am_attribute
{
	AMBER_LED_ON_OFF,
	AMBER_LED_MODE,
	AMBER_DURATION_ON,
	AMBER_DURATION_OFF,
	RED_LED_ON_OFF,
	RED_LED_MODE,
	RED_DURATION_ON,
	RED_DURATION_OFF,
	AMBER_THRESHOLD,
	RED_THRESHOLD,
	CPU_MODE,
	GPIO_POLARITY,
	MCU_MODE_DEBOUNCE,
	MCU_SUSPEND,
	LED_OWNER,
	VERSION,
	LOW_BAT_ALERT_1,
	LOW_BAT_ALERT_2,
	LOW_BAT_ALERT_3,
	LOW_TEMP_ALERT,
	HIGH_TEMP_ALERT,
	LOW_TEMP_SHUTDOWN,
	HIGH_TEMP_ALERT_DISCHARGING,
	HIGH_TEMP_SHUTDOWN_DISCHARGING,
	RECHARGE_TEMP_LOW,
	RECHARGE_TEMP_HIGH,
	WHITE_LED_ON_OFF,
	WHITE_LED_MODE,
	WHITE_DURATION_ON,
	WHITE_DURATION_OFF,
	GREEN_LED_ON_OFF,
	GREEN_LED_MODE,
	GREEN_DURATION_ON,
	GREEN_DURATION_OFF,
	DISABLE_CHARGING,
	DEAD_BAT_TIMEOUT,
	OVER_DISCHG_BAT_TIMEOUT,
	JUDGEMENT_FINISHED,
};

static struct cg7216am_device_data {
	u8 addr;
	int min_value;
	int max_value;
	int default_value;
} cg7216am_data[] = {
	CG7216AM_DATA(AMBER_LED_ON_OFF, 0x0, 0x1, 0x1),
	CG7216AM_DATA(AMBER_LED_MODE, 0x0, 0x1, 0x1),
	CG7216AM_DATA(AMBER_DURATION_ON, 0x0, 0xFF, 0x1),
	CG7216AM_DATA(AMBER_DURATION_OFF, 0x0, 0xFF, 0x9),
	CG7216AM_DATA(RED_LED_ON_OFF, 0x0, 0x1, 0x0),
	CG7216AM_DATA(RED_LED_MODE, 0x0, 0x1, 0x1),
	CG7216AM_DATA(RED_DURATION_ON, 0x0, 0xFF, 0x1),
	CG7216AM_DATA(RED_DURATION_OFF, 0x0, 0xFF, 0x9),
	CG7216AM_DATA(AMBER_THRESHOLD, 0x0, 0x64, 0x25),
	CG7216AM_DATA(RED_THRESHOLD, 0x0, 0x64, 0xF),
	CG7216AM_DATA(CPU_MODE, 0x0, 0x2, 0x0),
	CG7216AM_DATA(GPIO_POLARITY, 0x0, 0x1, 0x1),
	CG7216AM_DATA(MCU_MODE_DEBOUNCE, 0x0, 0xFF, 0xA),
	CG7216AM_DATA(MCU_SUSPEND, 0x0, 0x1, 0x0),
	CG7216AM_DATA(LED_OWNER, 0x0, 0x1, 0x0),
	CG7216AM_DATA(VERSION, 0x0, 0xFF, 0x0),
	CG7216AM_DATA(LOW_BAT_ALERT_1, 0x0, 0x64, 0x5),
	CG7216AM_DATA(LOW_BAT_ALERT_2, 0x0, 0x64, 0xA),
	CG7216AM_DATA(LOW_BAT_ALERT_3, 0x0, 0x64, 0xF),
	CG7216AM_DATA(LOW_TEMP_ALERT, 0x0, 0xF, 0x0),
	CG7216AM_DATA(HIGH_TEMP_ALERT, 0x0, 0xFF, 0x37),
	CG7216AM_DATA(LOW_TEMP_SHUTDOWN, 0x0, 0x0, 0x0),
	CG7216AM_DATA(HIGH_TEMP_ALERT_DISCHARGING, 0x0, 0xFF, 0x3C),
	CG7216AM_DATA(HIGH_TEMP_SHUTDOWN_DISCHARGING, 0x0, 0xFF, 0x41),
	CG7216AM_DATA(RECHARGE_TEMP_LOW, 0x0, 0xFF, 0x5),
	CG7216AM_DATA(RECHARGE_TEMP_HIGH, 0x0, 0xFF, 0x32),
	CG7216AM_DATA(WHITE_LED_ON_OFF, 0x0, 0x1, 0x1),
	CG7216AM_DATA(WHITE_LED_MODE, 0x0, 0x1, 0x1),
	CG7216AM_DATA(WHITE_DURATION_ON, 0x0, 0xFF, 0x1),
	CG7216AM_DATA(WHITE_DURATION_OFF, 0x0, 0xFF, 0x5),
	CG7216AM_DATA(GREEN_LED_ON_OFF, 0x0, 0x1, 0x0),
	CG7216AM_DATA(GREEN_LED_MODE, 0x0, 0x1, 0x0),
	CG7216AM_DATA(GREEN_DURATION_ON, 0x0, 0xFF, 0x1),
	CG7216AM_DATA(GREEN_DURATION_OFF, 0x0, 0xFF, 0x5),
	CG7216AM_DATA(DISABLE_CHARGING, 0x0, 0x1, 0x1),
	CG7216AM_DATA(DEAD_BAT_TIMEOUT, 0x0, 0xFF, 0xB4), /* 180s */
	CG7216AM_DATA(OVER_DISCHG_BAT_TIMEOUT, 0x0, 0xFF, 0x78), /* 1200s = 20 mins */
	CG7216AM_DATA(JUDGEMENT_FINISHED, 0x0, 0xFF, 0xFF), /* all judgements are finished */
};

struct cg7216am_alert_condition
{
	u8 low_bat_alert_1;
	u8 low_bat_alert_2;
	u8 low_bat_alert_3;
	u8 low_temp_alert;
	u8 high_temp_alert;
	u8 low_temp_shutdown;
	u8 high_temp_alert_discharging;
	u8 high_temp_shutdown_discharging;
	u8 recharge_temp_low;
	u8 recharge_temp_high;
};

struct cg7216am_drvdata {
	int fw_version;
	struct bin_attribute fw_version_attr;
};

static void power_on_reset(struct i2c_client *i2c);
static void wakeup_mcu(struct i2c_client *i2c);

typedef int (*I2C_ACCESS_FUNC)(struct i2c_client *client, char *buf, int count);
static int i2c_access_imp(struct i2c_client *i2c, char *data, int len, I2C_ACCESS_FUNC i2c_func)
{
	int i, ret = 0;

	mdelay(MCU_COMMAND_GAP_TIME_MSEC);
	for(i = 0; i < MCU_RETRY_MAX; i++) {
		ret = i2c_func(i2c, data, len);
		if (ret >= 0)
			break;
		else {
			printk("%s, i2c err, ret:%d, retry:%d\n", __func__, ret, i);
			/*i2c transfer error*/
			if(i == MCU_RETRY_MAX) {
				power_on_reset(i2c);
			}
			wakeup_mcu(i2c);
		}
	}

	return ret;
}

static int read_data(struct i2c_client *i2c, char *data, int len)
{
	return i2c_access_imp(i2c, data, len, i2c_master_recv);
}


static int write_data(struct i2c_client *i2c, const char *data, int len)
{
	return i2c_access_imp(i2c, (char*)data, len, (I2C_ACCESS_FUNC)i2c_master_send);
}

static int write_reg(struct i2c_client *i2c, u8 addr, u8 data)
{
	unsigned char buf[] = {addr, data};
	return write_data(i2c, buf, sizeof(buf));
}

static int read_reg(struct i2c_client *i2c, u8 addr, u8* data)
{
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[2];
	int i, ret = 0;
	mdelay(MCU_COMMAND_GAP_TIME_MSEC);

	/*write*/
	msg[0].addr = i2c->addr;
	msg[0].flags = i2c->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (char *)&addr;

	/*read*/
	msg[1].addr = i2c->addr;
	msg[1].flags = i2c->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = (char *)data;

	for(i = 0; i < MCU_RETRY_MAX; i++) {
		ret = i2c_transfer(adap, msg, 2);
		if (ret >= 0)
			break;
	}

	return ret;
}

static int get_default_version(void)
{
	int ret = -EINVAL;
	struct board_version b_version;

	gallo_get_hw_info(&b_version);
	switch (b_version.hw_version) {
	case BOARD_VERSION_EVT1:
	{
		ret = 0x01;
		break;
	}
	case BOARD_VERSION_EVT2:
	{
		ret = 0x02;
		break;
	}
	case BOARD_VERSION_DVT:
	{
		ret = 0x03;
		break;
	}
	case BOARD_VERSION_DVT2:
	default:
	{
		ret = 0x21;
		break;
	}
	}
	return ret;

}

static int get_version(struct i2c_client *i2c)
{
	int retry = 0;
	int ret = -EINVAL;
	char data;
	int version = get_default_version();
	LOG_FUNC();

	for (; retry < MCU_RETRY_MAX; retry++) {
		ret = read_reg(i2c, cg7216am_data[VERSION].addr, &data);
		if (ret >= 0) {
			version = data;
			break;
		}
	}
	return version;
}

static ssize_t cg7216am_attr_get_version(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	char fw_str[MCU_FW_STR_LEN+1];
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct cg7216am_drvdata *devdata = i2c_get_clientdata(client);

	if (unlikely(!count))
		return count;

	if (off != 0)
		return 0;
	if (count != MCU_FW_STR_LEN)
		return 0;

	if (devdata) {
		sprintf(fw_str, "0x%02x", devdata->fw_version);
		memcpy(buf, fw_str, MCU_FW_STR_LEN);
	}
	return MCU_FW_STR_LEN;
}

static int set_alert_condition(struct cg7216am_alert_condition* alert_cond)
{
	return 0;
}


static void wakeup_mcu_imp(struct cg7216am_platform_data *pd)
{
	gpio_set_value(pd->gpio_sys_sus, pd->gpio_polarity_sys_sus);
	mdelay(MCU_WAKE_DELAY_TIME_MSEC);
	gpio_set_value(pd->gpio_sys_sus, !pd->gpio_polarity_sys_sus);
}

static void wakeup_mcu(struct i2c_client *i2c)
{
	struct cg7216am_drvdata *devdata = i2c_get_clientdata(i2c);
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;

	if (devdata && devdata->fw_version > 0x03) {
		dev_dbg(&i2c->dev, "%s: the firmware(0x%x) support suspend mode \n", __func__, devdata->fw_version);
		wakeup_mcu_imp(pd);
	}
}

static int suspend_mcu(struct i2c_client *i2c)
{
	int ret = 0;
	struct cg7216am_drvdata *devdata = i2c_get_clientdata(i2c);

	if (devdata && devdata->fw_version > 0x03) {
		ret = write_reg(i2c, cg7216am_data[MCU_SUSPEND].addr, 0x01);
	}
	return ret;
}

static int enter_mcu_mode(struct i2c_client *i2c,
	struct cg7216am_platform_data *pd, int is_mcu_mode)
{
	int ret = 0;

	LOG_FUNC();
	if(is_mcu_mode) {
		gpio_set_value(pd->gpio_sys_sus, pd->gpio_polarity_sys_sus);
	} else {
		gpio_set_value(pd->gpio_sys_sus, !pd->gpio_polarity_sys_sus);
		mdelay(MCU_WAKE_DELAY_TIME_MSEC);
		ret = suspend_mcu(i2c);
	}
	printk("%s, is_mcu_mode:%d, ret:%d\n", __func__, is_mcu_mode, ret);
	return ret;
}

static int set_led(struct i2c_client *i2c, enum cg7216am_led_mode mode,
	int on_off_reg_offset, int mode_reg_offset)
{
	int ret;
	LOG_FUNC();

	wakeup_mcu(i2c);
	ret = write_reg(i2c, cg7216am_data[LED_OWNER].addr, 0x01);

	switch(mode)
	{
	case MCU_LED_ON:
		/*Turn off blinking then trun on LED*/
		write_reg(i2c, cg7216am_data[mode_reg_offset].addr, 0x00);
		write_reg(i2c, cg7216am_data[on_off_reg_offset].addr, 0x01);
	break;
	case MCU_LED_OFF:
		/*Turn off LED then trun off BLINKING*/
		write_reg(i2c, cg7216am_data[mode_reg_offset].addr, 0x01);
		write_reg(i2c, cg7216am_data[on_off_reg_offset].addr, 0x00);
	break;
	case MCU_LED_BLINKING:
		/*Turn on blinking then turn on LED*/
		write_reg(i2c, cg7216am_data[mode_reg_offset].addr, 0x01);
		write_reg(i2c, cg7216am_data[on_off_reg_offset].addr, 0x01);
	break;
	}

	suspend_mcu(i2c);
	return -EINVAL;
}

int cg7216am_led_set(struct device *dev, int index, enum cg7216am_led_mode mode)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if(i2c == 0) {
		return -ENODEV;
	}
	switch(index)
	{
	case AMBER:
		return set_led(i2c, mode, AMBER_LED_ON_OFF, AMBER_LED_MODE);
	break;
	case WHITE:
		return set_led(i2c, mode, WHITE_LED_ON_OFF, WHITE_LED_MODE);
	break;
	case GREEN:
		return set_led(i2c, mode, GREEN_LED_ON_OFF, GREEN_LED_MODE);
	break;
	}
	return -ENODEV;
}
EXPORT_SYMBOL(cg7216am_led_set);


int cg7216am_battery_disable_charging(struct device *dev, bool disable)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0) {
		return -ENODEV;
	}

	wakeup_mcu(i2c);
	if (disable) {
		write_reg(i2c, cg7216am_data[DISABLE_CHARGING].addr, 0x01);
	} else {
		write_reg(i2c, cg7216am_data[DISABLE_CHARGING].addr, 0x00);
	}
	suspend_mcu(i2c);
	return -ENODEV;
}
EXPORT_SYMBOL(cg7216am_battery_disable_charging);

int cg7216am_get_dead_bat_remain_timeout(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret = -EINVAL;
	char data = 0;

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0) {
		return -ENODEV;
	}

	wakeup_mcu(i2c);
	ret = read_reg(i2c, cg7216am_data[DEAD_BAT_TIMEOUT].addr, &data);
	if (ret < 0) {
		LOG_MSG("i2c_transfer_error!! \n");
		data = 0;
	}
	suspend_mcu(i2c);
	return data;
}
EXPORT_SYMBOL(cg7216am_get_dead_bat_remain_timeout);

void cg7216am_store_dead_bat_remain_timeout(struct device *dev, int sec)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret = -EINVAL;

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0) {
		return;
	}

	wakeup_mcu(i2c);
	ret = write_reg(i2c, cg7216am_data[DEAD_BAT_TIMEOUT].addr, sec);
	if (ret < 0) {
		LOG_MSG("i2c_transfer_error!! \n");
	}
	suspend_mcu(i2c);
}
EXPORT_SYMBOL(cg7216am_store_dead_bat_remain_timeout);


int cg7216am_get_over_dischg_bat_remain_timeout(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret = -EINVAL;
	char data = 0;

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0) {
		return -ENODEV;
	}

	wakeup_mcu(i2c);
	ret = read_reg(i2c, cg7216am_data[OVER_DISCHG_BAT_TIMEOUT].addr, &data);
	if (ret < 0) {
		LOG_MSG("i2c_transfer_error!! \n");
		data = 0;
	}
	suspend_mcu(i2c);
	return data*10;
}
EXPORT_SYMBOL(cg7216am_get_over_dischg_bat_remain_timeout);

void cg7216am_store_over_dischg_bat_remain_timeout(struct device *dev, int sec)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret = -EINVAL;

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0) {
		return;
	}
	wakeup_mcu(i2c);
	ret = write_reg(i2c, cg7216am_data[OVER_DISCHG_BAT_TIMEOUT].addr, (sec+5)/10);
	if (ret < 0) {
		LOG_MSG("i2c_transfer_error!! \n");
	}
	suspend_mcu(i2c);
}
EXPORT_SYMBOL(cg7216am_store_over_dischg_bat_remain_timeout);

void cg7216am_set_judgement_finished(struct device *dev, enum cg7216am_finished_judgement item, bool done)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret = -EINVAL;
	char data = 0;

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0) {
		return;
	}
	wakeup_mcu(i2c);
	ret = read_reg(i2c, cg7216am_data[JUDGEMENT_FINISHED].addr, &data);
	LOG_MSG("before: update value to 0x%x \n", data);
	data &= ~(0x01 << item);
	data |= (done << item);

	LOG_MSG("after: update value to 0x%x \n", data);
	ret = write_reg(i2c, cg7216am_data[JUDGEMENT_FINISHED].addr, data);
	if (ret < 0) {
		LOG_MSG("i2c_transfer_error!! \n");
	}
	suspend_mcu(i2c);
}
EXPORT_SYMBOL(cg7216am_set_judgement_finished);

static int destroy_each_dev(struct device *each_dev, void *unused)
{
	LOG_FUNC();
	platform_device_unregister(to_platform_device(each_dev));
	return 0;
}

static int destroy_sub_devs(struct device *dev)
{
	LOG_FUNC();
	return device_for_each_child(dev, NULL, destroy_each_dev);
}

static int create_sub_devs(struct device *dev)
{
	int i, ret;
	struct cg7216am_platform_data *pd = dev->platform_data;
	struct cg7216am_subdev_info *each_dev;
	struct platform_device *pdev;

	LOG_FUNC();
	for(i=0; i < pd->num_subdevs; i++) {
		each_dev  = &pd->sub_devs[i];

		LOG_MSG("%s:name:%s, id:%d, num_subdevs:%d\n", __func__,
			each_dev->name, each_dev->id, pd->num_subdevs);

		pdev = platform_device_alloc(each_dev->name, each_dev->id);
		/*associate parent and platform_data for each sub_dev*/
		pdev->dev.parent = dev;
		pdev->dev.platform_data = each_dev->platform_data;
		ret = platform_device_add(pdev);
		if(ret) {
			destroy_sub_devs(dev);
			return ret;
		}
	}
	return ret;
}

struct ota_data {
	struct i2c_client *this_client;
	struct cg7216am_platform_data *pd;
	struct work_struct work;
	struct mutex lock;
};

static int update_firmware_imp(struct i2c_client *i2c, const char* firmware_buf,
	int row_count, int col_count)
{
	int ret, row;
	u8 ack;
	int i;
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;
	LOG_FUNC();

	for (i = 0; i < MCU_WAKEUP_RETRY_MAX; i++) {
		ret = read_data(i2c, &ack, sizeof(ack));
		if(ret < 0) {
				LOG_MSG("%s:wakeup mcu fail, ack=%#x, retry=%d\n", __func__, ack, i);
				wakeup_mcu_imp(pd);
		} else {
				LOG_MSG("%s:wakeup mcu ok, ack=%#x, retry=%d\n", __func__, ack, i);
				break;
		}
	}

	for(row = 0; row < row_count; row++) {
		ret = write_data(i2c, firmware_buf+row*col_count, col_count);
		if(ret < 0) {
			LOG_MSG("%s:i2c write err; row:%d, ret:%d\n", __func__, row, ret);
			return ret;
		}

		if((row%5) == 0) {
			msleep(100);
			read_data(i2c, &ack, sizeof(ack));
			msleep(1);
			if(ack != 0x20)
				LOG_MSG("%s:row[%d] ack=%#x\n", __func__, row, ack);
		} else
			msleep(10);

		if(row == row_count - 1) {
			msleep(1000);
			read_data(i2c, &ack, sizeof(ack));
			LOG_MSG("%s verify image: %#x\n", __func__, ack);
			if(ack == 0x21)
				return 0;
			else
				return -1;
		}
	}

	return -1;
}

static int update_firmware(struct i2c_client *i2c)
{
	const struct firmware *fw_entry;
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;
	char hw_ver[32] = {0};
	char event[32] = {0};
	char *envp[] = {event, NULL};
	int ret = -1;

	LOG_FUNC();
	sprintf(hw_ver, "%d", pd->hw_version);
	ret = request_firmware(&fw_entry, hw_ver, &i2c->dev);
	if( ret == 0) {
		int counts = fw_entry->size/CG7216_FIRMWARE_UPDATE_EACH_WRITE;
		LOG_MSG("%s:prepare to update firmeware: size:%d\n", __func__, fw_entry->size);
		ret = update_firmware_imp(i2c, fw_entry->data, counts, CG7216_FIRMWARE_UPDATE_EACH_WRITE);
		release_firmware(fw_entry);
		if(ret == 0) {
			snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_SUCCESS");
			kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
		} else {
			snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
			kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
		}
		return 0;
	}
	LOG_MSG("%s: request_firmware ret = %d!\n", __func__, ret);
	snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
	kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
	return -EFAULT;
}

static void ota_work_func(struct work_struct *work)
{
	struct ota_data *ota =
	    container_of(work, struct ota_data, work);

	LOG_FUNC();
	mutex_lock(&ota->lock);
	update_firmware(ota->this_client);
	mutex_unlock(&ota->lock);
}

static ssize_t request_ota_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t	status;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ota_data *ota = i2c_get_clientdata(i2c);

	LOG_FUNC();
	LOG_MSG("%s: i2c:%x\n", __func__, (unsigned int)i2c);

	if (sysfs_streq(buf, "1"))
		status = schedule_work(&ota->work);
	else
		status = -EINVAL;

	return status ? : size;
}

static /* const */ DEVICE_ATTR(request_ota, 0644, 0, request_ota_store);


static void power_on_reset(struct i2c_client *i2c)
{
	printk("%s\n", __func__);
}

static irqreturn_t cg7216am_interrupt_isr(int irq, void *devdata)
{
	return IRQ_HANDLED;
}


static __devinit int cg7216am_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	int ret;
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;
	struct ota_data *ota;
	struct cg7216am_drvdata *devdata;
	LOG_FUNC();

	if(i2c == 0 || i2c->dev.platform_data == 0) {
		printk("%s:invalid param\n", __func__);
		return -EINVAL;
	}

	power_on_reset(i2c);
	wakeup_mcu_imp(pd);

	if(pd->is_fw_update_mode) {
		ota = kzalloc(sizeof(struct ota_data ), GFP_KERNEL);
		if (!ota) {
			dev_err(&i2c->dev,
				"failed to allocate memory for module data\n");
			return -ENOMEM;
		}
		ota->this_client = i2c;
		ota->pd = i2c->dev.platform_data;
		mutex_init(&ota->lock);
		INIT_WORK(&ota->work, ota_work_func);
		i2c_set_clientdata(i2c, ota);

		printk("%s:firmware upgrade mode...:i2c:%x\n", __func__, (unsigned int)i2c);
		/*create attributes for firmware upgrade*/
		ret = device_create_file(&i2c->dev, &dev_attr_request_ota);
		return ret;
	}

	devdata = kzalloc(sizeof(struct cg7216am_drvdata), GFP_KERNEL);
	if (!devdata) {
		dev_err(&i2c->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	devdata->fw_version = get_version(i2c);
	i2c_set_clientdata(i2c, devdata);

	cg7216am_misc_device.parent = &i2c->dev;
	ret = misc_register(&cg7216am_misc_device);
	if (ret) {
		pr_err("%s: misc_register() register failed\n", __func__);
		goto fail0;
	}
	sysfs_bin_attr_init(&devdata->fw_version_attr);
	devdata->fw_version_attr.attr.name = "fw_version";
	devdata->fw_version_attr.attr.mode = S_IRUGO;
	devdata->fw_version_attr.size =  MCU_FW_STR_LEN;
	devdata->fw_version_attr.read = cg7216am_attr_get_version;

	ret = sysfs_create_bin_file(&i2c->dev.kobj, &devdata->fw_version_attr);
	if (ret) {
			sysfs_remove_bin_file(&i2c->dev.kobj,  &devdata->fw_version_attr);
		goto fail1;
	}

	ret = request_irq(gpio_to_irq(pd->gpio_mcu_int), cg7216am_interrupt_isr,
					IRQF_SHARED|IRQF_TRIGGER_FALLING, "cg7216am_int", devdata);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: Failed to request_irq(): gpio=%d, irq=%d \n",
		 __func__, pd->gpio_mcu_int, gpio_to_irq(pd->gpio_mcu_int));
		goto fail1;
	}

	device_init_wakeup(&i2c->dev, 1);
	dev_info(&i2c->dev,  "%s request_irq () ok,  gpio=%d, irq=%d \n", __func__,
			pd->gpio_mcu_int, gpio_to_irq(pd->gpio_mcu_int));

	create_sub_devs(&i2c->dev);
	printk("%s:mcu version:0x%x \n", __func__, devdata->fw_version);
	/*
	ret = write_reg(cg7216am_data[GPIO_POLARITY].addr, 0x01);
	*/
	WARN_ON(enter_mcu_mode(i2c, pd , 0) < 0);

	return ret;

fail1:
	misc_deregister(&cg7216am_misc_device);
fail0:
	i2c_set_clientdata(i2c, 0);
	kfree(devdata);
	return ret;
}

static __devexit int cg7216am_remove(struct i2c_client *i2c)
{
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;
	struct cg7216am_drvdata *devdata = i2c_get_clientdata(i2c);

	LOG_FUNC();
	sysfs_remove_bin_file(&i2c->dev.kobj,  &devdata->fw_version_attr);
	misc_deregister(&cg7216am_misc_device);
	destroy_sub_devs(&i2c->dev);

	device_init_wakeup(&i2c->dev, 0);
	free_irq(gpio_to_irq(pd->gpio_mcu_int), devdata);
	if (devdata) {
		kfree(devdata);
		devdata = NULL;
		i2c_set_clientdata(i2c, 0);
	}
	return 0;
}

static int cg7216am_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;
	LOG_FUNC();

	if (device_may_wakeup(&i2c->dev)) {
		enable_irq_wake(gpio_to_irq(pd->gpio_mcu_int));
	}

	WARN_ON(enter_mcu_mode(i2c, i2c->dev.platform_data, 1) < 0);
	return 0;
}

static int cg7216am_resume(struct i2c_client *i2c)
{
	struct cg7216am_platform_data *pd = i2c->dev.platform_data;
	LOG_FUNC();

	if (device_may_wakeup(&i2c->dev)) {
		disable_irq_wake(gpio_to_irq(pd->gpio_mcu_int));
	}

	WARN_ON(enter_mcu_mode(i2c, i2c->dev.platform_data, 0) < 0);
	return 0;
}

static const struct i2c_device_id i2c_id[] = {
	{ "cg7216am", 0 },
	{}
};

static struct i2c_driver cg7216am_driver = {
	.driver = {
		.name = "cg7216am",
		.owner = THIS_MODULE,
	},
	.probe		= cg7216am_probe,
	.remove		= __devexit_p(cg7216am_remove),
	.suspend	= cg7216am_suspend,
	.resume		= cg7216am_resume,
	.id_table	= i2c_id,
};

static int __init cg7216am_module_init(void)
{
	LOG_FUNC();
	return i2c_add_driver(&cg7216am_driver);
}
module_init(cg7216am_module_init);

static void __exit cg7216am_module_exit(void)
{
	LOG_FUNC();
	i2c_del_driver(&cg7216am_driver);
}
module_exit(cg7216am_module_exit);


MODULE_AUTHOR("Andrew Hsiao <Andrew_Hsiao@pegatroncorp.com>");
MODULE_DESCRIPTION("CG7216AM MCU Driver");
MODULE_LICENSE("GPL");
