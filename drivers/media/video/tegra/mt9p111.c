/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9P111 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/mt9p111.h>

#include "mt9p111_reg.h"

#define NULL_DEVICE 0
#define MT9P111_FLASH_LEVEL_OFF 0
#define MT9P111_FLASH_LEVEL_ON 1
#define MT9P111_FLASH_LEVEL_AF 1

enum {
	MT9P111_STATE_CONTEXT_A,
	MT9P111_STATE_CONTEXT_B,
	MT9P111_STATE_UNINITED,
};

struct sensor_info {
	int mode;
	struct i2c_client *i2c_client;
	struct mt9p111_platform_data *pdata;
};

static struct sensor_info *info;

static int mt9p111_state;

static int mt9p111_flash_mode;
static int mt9p111_flash_value;

static int mt9p111_af_pushed_to_infinity;

static int context_table[MT9P111_STATE_UNINITED];

#if 1
static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = ((client->addr) >> 1);
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = ((client->addr) >> 1);
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = (data[2] << 8) | data[3];

	return 0;
}
#endif

static int sensor_read_reg8(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = ((client->addr) >> 1);
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = ((client->addr) >> 1);
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];

	return 0;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = ((client->addr) >> 1);
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("mt9p111_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(20);
	} while (retry <= MT9P111_MAX_RETRIES);

	return err;
}

static int sensor_write_reg8(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = ((client->addr) >> 1);
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("mt9p111_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(20);
	} while (retry <= MT9P111_MAX_RETRIES);

	return err;
}

static int sensor_poll_reg8(struct i2c_client *client, u16 addr, u8 *val,
			    u8 expect_val, u16 delay_ms, u16 count)
{
	int err, i;
	for (i = 0; i < count; i++) {
		msleep(delay_ms);
		err = sensor_read_reg8(client, addr, val);
		if (err != 0)
			continue;
		if (*val == expect_val) {
			pr_info("%s: success on %d th\n", __func__, i);
			return 0;
		}
	}
	pr_info("%s: failed, last read 0x%x\n", __func__, *val);
	return -EINVAL;
}

static int sensor_pollnot_reg8(struct i2c_client *client, u16 addr, u8 *val,
			    u8 expect_not_val, u16 delay_ms, u16 count)
{
	int err, i;
	for (i = 0; i < count; i++) {
		msleep(delay_ms);
		err = sensor_read_reg8(client, addr, val);
		if (err != 0)
			continue;
		if (*val != expect_not_val) {
			pr_info("%s: success on %d th\n", __func__, i);
			return 0;
		}
	}
	pr_info("%s: failed\n", __func__);
	return -EINVAL;
}

static int sensor_poll_reg16(struct i2c_client *client, u16 addr, u16 *val,
			    u16 expect_val, u16 delay_ms, u16 count)
{
	int err, i;
	for (i = 0; i < count; i++) {
		msleep(delay_ms);
		err = sensor_read_reg(client, addr, val);
		if (err != 0)
			continue;
		if (*val == expect_val) {
			pr_info("%s: success on %d th\n", __func__, i);
			return 0;
		}
	}
	pr_info("%s: failed, last read 0x%x\n", __func__, *val);
	return -EINVAL;
}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg_v2 table[])
{
	int err;
	const struct sensor_reg_v2 *next;
	u16 val, read_val16;
	u8 read_val8;

	pr_info("mt9p111 %s\n", __func__);
	for (next = table; next->purpose != CMD_TABLE_END; next++) {
		if (next->purpose == CMD_WAIT_MS) {
			msleep(next->addr);
			continue;
		}

		val = next->val;
		if (next->purpose == CMD_REG16)
			err = sensor_write_reg(client, next->addr, val);
		else if (next->purpose == CMD_PGA1)
			err = sensor_write_reg(client, next->addr, val);
		else if (next->purpose == CMD_PGA2)
			err = sensor_write_reg(client, next->addr, val);
		else if (next->purpose == CMD_PGA3)
			err = sensor_write_reg(client, next->addr, val);
		else if (next->purpose == CMD_REG8)
			err = sensor_write_reg8(client, next->addr, val);
		else if (next->purpose == CMD_POLL8)
			err = sensor_poll_reg8(client, next->addr, &read_val8,
				val, 50, 6);
		else if (next->purpose == CMD_POLL16)
			err = sensor_poll_reg16(client, next->addr, &read_val16,
				val, 50, 6);
		else if (next->purpose == CMD_POLL8_NOT)
			err = sensor_pollnot_reg8(client, next->addr,
				&read_val8, val, 50, 6);
		else
			err = 0;
		if (err)
			return err;
	}
	return 0;
}

int mt9p111_write_table(struct i2c_client *client,
			      const struct sensor_reg_v2 table[])
{
	return sensor_write_table(client, table);
}

#if 0
int sensor_read_optm(struct i2c_client *client, u16 *status)
{
	int err;
	err = sensor_write_table(info->i2c_client, mt9p111_check_otp_lc);
	if (err)
		return err;
	err = sensor_read_reg(info->i2c_client, 0x380C, status);
	if (err)
		return err;
	pr_info("%s: R0x380C 0x%x\n", __func__, *status);
	return 0;
}
#endif

static int sensor_get_exposure_time(struct sensor_info *info,
	struct mt9p111_exposure *exposure)
{
	u16 time, line_length_pck;
	int err;
	err = sensor_read_reg(info->i2c_client, 0xA83A, &time);
	if (err)
		return err;
	err = sensor_read_reg(info->i2c_client, 0x300C, &line_length_pck);
	if (err)
		return err;
	if (!line_length_pck)
		line_length_pck = 1;
	exposure->exposure = time;
	exposure->line_per_second = 96000000 / line_length_pck;
	return 0;
}

static int sensor_should_flash(struct sensor_info *info)
{
	u16 status;
	int err;
	switch (mt9p111_flash_mode) {
	case MT9P111_FLASHMODE_ON:
		return 1;
	case MT9P111_FLASHMODE_TORCH:
	case MT9P111_FLASHMODE_OFF:
		return 0;
	case MT9P111_FLASHMODE_AUTO:
	/* TO-DO: If NULL_DEVICE is passed here,
	 * return the value got previously
	 */
		if (info == NULL_DEVICE)
			return 1;
		err = sensor_read_reg(info->i2c_client, 0xA83A, &status);
		if (err)
			return 0;
		pr_info("mt9p111 exposure 0x%x, target 0x%x\n", status,
			flash_target);
		if (status > flash_target)
			return 1;
		else
			return 0;
	default:
		return 0;
	}
	return 0;
}

static int sensor_set_led_level(struct sensor_info *info, int level)
{
	if (info->pdata && info->pdata->set_led_level) {
		info->pdata->set_led_level(level);
		mt9p111_flash_value = level;
	}
	return 0;
}

static int sensor_set_mode(struct sensor_info *info, struct mt9p111_mode *mode)
{
	int sensor_table, sensor_table_A;
	int err, board_info, i;
	u8 val;

	val = 0;
	pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);

	if (mode->xres == 2592 && mode->yres == 1944)
		sensor_table = SENSOR_MODE_2592x1944;
	else if (mode->xres == 1280 && mode->yres == 960)
		sensor_table = SENSOR_MODE_1280x960;
	else if (mode->xres == 1280 && mode->yres == 720)
		sensor_table = SENSOR_MODE_1280x720;
	else if (mode->xres == 800 && mode->yres == 600)
		sensor_table = SENSOR_MODE_800x600;
	else if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}
	/* If it's Preview: target mode should be in context A*/
	if (sensor_table != SENSOR_MODE_2592x1944)
		sensor_table_A = sensor_table;
	else if (mt9p111_state == MT9P111_STATE_UNINITED)
		sensor_table_A = SENSOR_MODE_640x480; /* Special case*/

	if (mt9p111_state == MT9P111_STATE_UNINITED) {
		if (info->pdata && info->pdata->get_board_info)
			board_info = info->pdata->get_board_info();
		mt9p111_check_if_2nd_source(info->i2c_client, board_info);
		for (i = 0; i < 3; i++) {
			if (i != 0) {
				info->pdata->power_off();
				info->pdata->power_on();
			}
			pr_info("%s mode_init\n", __func__);
			err = sensor_write_table(info->i2c_client,
				mt9p111_regs.init);
			if (err)
				continue;
			pr_info("%s mode_table %d\n", __func__, sensor_table_A);
			err = sensor_write_table(info->i2c_client,
				mode_table[sensor_table_A]);
			if (err)
				continue;
			pr_info("%s mode_init2\n", __func__);
			err = sensor_write_table(info->i2c_client,
				mt9p111_regs.init2);
			if (err)
				continue;
			pr_info("%s write lc\n", __func__);
			err = sensor_write_table(info->i2c_client,
				mt9p111_regs.lc);
			if (err)
				continue;
			pr_info("%s mode_init3\n", __func__);
			err = sensor_write_table(info->i2c_client,
				mt9p111_regs.init3);
			if (err)
				continue;
			pr_info("%s write lc late\n", __func__);
			mt9p111_check_otp_lc(info->i2c_client, board_info);
			err = sensor_write_table(info->i2c_client,
				mt9p111_regs.lc_late);
			if (err)
				continue;
			err = sensor_poll_reg8(info->i2c_client, 0x8405, &val,
				0x03, 50, 50);
			if (err)
				continue;
			else
				break;
		}
		if (err)
			return err;
		msleep(75);
		context_table[MT9P111_STATE_CONTEXT_A] = sensor_table_A;
		context_table[MT9P111_STATE_CONTEXT_B] = SENSOR_MODE_2592x1944;
	} else if (sensor_table != SENSOR_MODE_2592x1944) {
		/* Target resolution should be in context A*/
#if MT9P111_AUTO_FLASH
#else
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
#endif

		if (context_table[MT9P111_STATE_CONTEXT_A] != sensor_table_A) {
			/* Should modify resolution of context A*/
			pr_info("%s mode_table %d\n", __func__, sensor_table_A);
			err = sensor_write_table(info->i2c_client,
				mode_table[sensor_table_A]);
			if (err)
				return err;
			err = sensor_write_reg8(info->i2c_client, 0x8404, 0x06);
			if (err)
				return err;
			context_table[MT9P111_STATE_CONTEXT_A] = sensor_table_A;
		}
		if (mt9p111_state == MT9P111_STATE_CONTEXT_B) {
			/* Switching back from B to A */
			pr_info("%s mode__back_to_preview\n", __func__);
			err = sensor_write_table(info->i2c_client,
				mt9p111_regs.back_to_preview);
			if (err)
				return err;
		}
		err = sensor_poll_reg8(info->i2c_client, 0x8405, &val,
			0x03, 50, 50);
	}

	if (sensor_table == SENSOR_MODE_2592x1944) {
		/* Should be in Context B */
		pr_info("%s mode_table %d\n", __func__, SENSOR_MODE_2592x1944);
#if MT9P111_AUTO_FLASH
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
#else
		if (sensor_should_flash(info))
			sensor_set_led_level(info, MT9P111_FLASH_LEVEL_ON);
#endif
		err = sensor_write_table(info->i2c_client,
			mode_table[SENSOR_MODE_2592x1944]);
		if (err)
			return err;
		err = sensor_poll_reg8(info->i2c_client, 0x8405, &val, 0x07, 50,
			100);
	}

	if (sensor_table != SENSOR_MODE_2592x1944)
		mt9p111_state = MT9P111_STATE_CONTEXT_A;
	else
		mt9p111_state = MT9P111_STATE_CONTEXT_B;

	info->mode = sensor_table;

	pr_info("%s --\n", __func__);
	return 0;
}

static int sensor_set_item_effect(struct sensor_info *info, int value)
{
	u8 val;
	pr_info("%s %d\n", __func__, value);
	sensor_poll_reg8(info->i2c_client, 0x8405, &val, 0x03, 50, 50);
	switch (value) {
	case MT9P111_EFFECT_MONO:
		return sensor_write_table(info->i2c_client,
			mt9p111_effect_mono);
	case MT9P111_EFFECT_SEPIA:
		return sensor_write_table(info->i2c_client,
			mt9p111_effect_sepia);
	case MT9P111_EFFECT_NEGATIVE:
		return sensor_write_table(info->i2c_client,
			mt9p111_effect_negative);
	case MT9P111_EFFECT_SOLARIZE:
		return sensor_write_table(info->i2c_client,
			mt9p111_effect_solarize);
	case MT9P111_EFFECT_POSTERIZE:
		return sensor_write_table(info->i2c_client,
			mt9p111_effect_posterize);
	default:
		return sensor_write_table(info->i2c_client,
			mt9p111_effect_none);
	}
	return 0;
}

static int sensor_set_item_wb(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9P111_WB_SUNLIGHT:
	case MT9P111_WB_CLOUDY:
		return sensor_write_table(info->i2c_client,
			mt9p111_wb_sunlight);
	case MT9P111_WB_FLUORESCENT:
		return sensor_write_table(info->i2c_client,
			mt9p111_wb_fluorescent);
	case MT9P111_WB_INCANDESCENT:
		return sensor_write_table(info->i2c_client,
			mt9p111_wb_incandescent);
	default:
		return sensor_write_table(info->i2c_client,
			mt9p111_wb_auto);
	}
	return 0;
}

static int sensor_set_item_brightness(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9P111_BRIGHTNESS_P1:
		return sensor_write_table(info->i2c_client,
			mt9p111_brightness_p1);
	case MT9P111_BRIGHTNESS_P2:
		return sensor_write_table(info->i2c_client,
			mt9p111_brightness_p2);
	case MT9P111_BRIGHTNESS_N1:
		return sensor_write_table(info->i2c_client,
			mt9p111_brightness_n1);
	case MT9P111_BRIGHTNESS_N2:
		return sensor_write_table(info->i2c_client,
			mt9p111_brightness_n2);
	default:
		return sensor_write_table(info->i2c_client,
			mt9p111_brightness_0);
	}
	return 0;
}

static int sensor_set_item_scene(struct sensor_info *info, int value)
{
	u8 val;
	pr_info("%s %d\n", __func__, value);
	sensor_poll_reg8(info->i2c_client, 0x8405, &val, 0x03, 50, 50);
	switch (value) {
	case MT9P111_SCENE_ACTION:
		return sensor_write_table(info->i2c_client,
			mt9p111_scene_action);
	case MT9P111_SCENE_NIGHT:
		return sensor_write_table(info->i2c_client,
			mt9p111_scene_night);
	default:
		return sensor_write_table(info->i2c_client,
			mt9p111_scene_auto);
	}
	return 0;
}

static int sensor_set_item_flashmode(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9P111_FLASHMODE_TORCH:
		mt9p111_flash_mode = value;
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_ON);
		return sensor_write_table(info->i2c_client,
			mt9p111_flash_off);
	case MT9P111_FLASHMODE_ON:
		mt9p111_flash_mode = value;
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
		return sensor_write_table(info->i2c_client,
			mt9p111_flash_on);
	case MT9P111_FLASHMODE_OFF:
		mt9p111_flash_mode = value;
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
		return sensor_write_table(info->i2c_client,
			mt9p111_flash_off);
	case MT9P111_FLASHMODE_AUTO:
		mt9p111_flash_mode = value;
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
		return sensor_write_table(info->i2c_client,
			mt9p111_flash_auto);
	default:
		return 0;
	}
	return 0;
}

static int sensor_set_item_flashvalue(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
#if MT9P111_AUTO_FLASH
#else
	switch (value) {
	case 0:
		sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
		return 0;
	case 1:
	default:
		if (sensor_should_flash(info))
			sensor_set_led_level(info, MT9P111_FLASH_LEVEL_ON);
		return 0;
	}
#endif
	return 0;
}

static int sensor_set_effect(struct sensor_info *info,
				struct mt9p111_effect *effect)
{
	switch (effect->item) {
	case MT9P111_ITEM_EFFECT:
		return sensor_set_item_effect(info, effect->value);
	case MT9P111_ITEM_WB:
		return sensor_set_item_wb(info, effect->value);
	case MT9P111_ITEM_BRIGHTNESS:
		return sensor_set_item_brightness(info, effect->value);
	case MT9P111_ITEM_SCENE:
		return sensor_set_item_scene(info, effect->value);
	case MT9P111_ITEM_FLASHMODE:
		return sensor_set_item_flashmode(info, effect->value);
	case MT9P111_ITEM_FLASHVALUE:
		return sensor_set_item_flashvalue(info, effect->value);
	default:
		return -EINVAL;
	}
	return 0;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct sensor_info *info = file->private_data;

	pr_info("mt9p111 %s\n", __func__);
	switch (cmd) {
	case MT9P111_IOCTL_SET_MODE:
	{
		struct mt9p111_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct mt9p111_mode))) {
			return -EFAULT;
		}

		return sensor_set_mode(info, &mode);
	}
	case MT9P111_IOCTL_GET_STATUS:
	{
		__u8 data = 0;
		if (mt9p111_flash_value)
			data = 1;
		if (copy_to_user((void __user *)arg,
			&data, 1)) {
			pr_info("%s err on line %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case MT9P111_IOCTL_SET_EFFECT:
	{
		struct mt9p111_effect effect;
		if (copy_from_user(&effect,
				   (const void __user *)arg,
				   sizeof(struct mt9p111_effect))) {
			return -EFAULT;
		}
		return sensor_set_effect(info, &effect);
	}
	case MT9P111_IOCTL_TRIGGER_AF:
	{
		u16 status;
		if (copy_from_user(&status,
				   (const void __user *)arg, 2)) {
			pr_info("%s err on line %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		pr_info("mt9p111 trigger af %d\n", status);
		if (!status) {
			sensor_set_led_level(info, MT9P111_FLASH_LEVEL_OFF);
			if (mt9p111_af_pushed_to_infinity)
				return 0;
			err = sensor_write_table(info->i2c_client,
				mt9p111_focus_infinity);
			if (err)
				return err;
			mt9p111_af_pushed_to_infinity = 1;
			return 0;
		}
		err = sensor_write_table(info->i2c_client,
			mt9p111_focus_auto);
		if (err)
			return err;
		mt9p111_af_pushed_to_infinity = 0;
		if (sensor_should_flash(info))
			sensor_set_led_level(info, MT9P111_FLASH_LEVEL_AF);
		return sensor_write_reg8(info->i2c_client, 0xB006, 0x01);
	}
	case MT9P111_IOCTL_GET_AF_STATUS:
	{
		u16 status;
		pr_info("mt9p111 get af stat\n");
		err = sensor_read_reg(info->i2c_client, 0xB000, &status);
		if (err)
			return err;
		pr_info("mt9p111 get af stat 0x%x\n", status);
		if ((status & 0x10) == 0x10) {
			status = 1;
#if MT9P111_AUTO_FLASH
			if (sensor_should_flash(NULL_DEVICE))
				sensor_set_led_level(info,
					MT9P111_FLASH_LEVEL_OFF);
#endif
		}
		else
			status = 0;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			pr_info("%s err on line %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case MT9P111_IOCTL_GET_EXPOSURE_TIME:
	{
		struct mt9p111_exposure exposure;
		err = sensor_get_exposure_time(info, &exposure);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &exposure,
				 sizeof(struct mt9p111_exposure))) {
			pr_info("%s err on line %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case MT9P111_IOCTL_GET_MODEL_ID:
	{
		u16 model_id;
		err = sensor_read_reg(info->i2c_client, 0x3000, &model_id);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &model_id,
				 2)) {
			pr_info("%s err on line %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_info("mt9p111 %s\n", __func__);
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	mt9p111_state = MT9P111_STATE_UNINITED;
	mt9p111_flash_mode = MT9P111_FLASHMODE_OFF;
	mt9p111_flash_value = 0;
	mt9p111_af_pushed_to_infinity = 1;
	return 0;
}

int mt9p111_release(struct inode *inode, struct file *file)
{
	pr_info("mt9p111 %s\n", __func__);
	sensor_set_led_level(info, MT9P111_FLASHMODE_OFF);
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	mt9p111_state = MT9P111_STATE_UNINITED;
	mt9p111_af_pushed_to_infinity = 1;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = mt9p111_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MT9P111_NAME,
	.fops = &sensor_fileops,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("mt9p111 %s\n", __func__);

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!info) {
		pr_err("mt9p111_sensor : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("mt9p111_sensor : Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("mt9p111 %s\n", __func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ MT9P111_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = MT9P111_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info("mt9p111 %s\n", __func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info("mt9p111 %s\n", __func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

