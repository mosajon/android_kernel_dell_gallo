/*
 * fm34.c  --  FM34  DSP SoC Audio driver
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <sound/fm34.h>

#include <linux/kthread.h>
#include <linux/clk.h>
#include "../../../arch/arm/mach-tegra/clock.h"

#ifdef DEBUG
#define LOG_FUNC() printk("<%s>\n", __FUNCTION__)
#else
#define LOG_FUNC() do{}while(0)
#endif

#ifdef DEBUG
static void fm34_assert(const char *fcn, int line, const char *expr)
{
	if (in_interrupt())
		panic("ASSERTION FAILED IN INTERRUPT, %s:%d %s\n",
		      fcn, line, expr);
	else {
		int x;
		printk(KERN_ERR "ASSERTION FAILED, %s:%d %s\n",
		       fcn, line, expr);
		x = * (volatile int *) 0; /* force proc to exit */
	}
}
#define ASSERT(e)      ((e) ? (void) 0 : fm34_assert(__func__, __LINE__, #e))
#else
#define ASSERT(e)           ((void) 0)
#endif

#define I2C_MAX_RETRIES 2
#define FM34_INIT_MAX_RETRIES 2

static struct i2c_client *fm34_i2c;
static enum fm34_mode fm34_mode_current = FM34_ENABLE_EC;

static void fm34_set_power(unsigned int pwdn_pin, bool is_power_enable)
{
	LOG_FUNC();
	/* PWDN#:low->power_NOT_down->enable */
	gpio_set_value(pwdn_pin, is_power_enable);
	msleep(15);
}

static void fm34_set_hw_bypass(unsigned int bp_pin, bool is_hw_bypass)
{
	LOG_FUNC();
	/* PB#:low->hw_bypass*/
	gpio_set_value(bp_pin, !is_hw_bypass);
}

static int fm34_write_raw(struct i2c_client *client, char *buf, unsigned int len)
{
	int err;
	int retry=0;
	LOG_FUNC();
	if (!client->adapter)
		return -ENODEV;

	do {
		err = i2c_master_send(client, buf, len);
		if (err == len)
			return 0;
		retry++;
		printk("<%s>I2C write error:%d\n", __FUNCTION__, retry);
	} while (retry <= I2C_MAX_RETRIES);

	return err;
}

static int fm34_write_reg(struct i2c_client *client, unsigned short addr, unsigned short data)
{
	unsigned char cmd_buf[] = {0xFC, 0xF3, 0x3B, 0xFF, 0xFF, 0xFF, 0xFF};

	LOG_FUNC();

        cmd_buf[3] = (addr & 0xFF00) >> 8;
	cmd_buf[4] = addr & 0xFF;

	cmd_buf[5] = (data & 0xFF00) >> 8;
	cmd_buf[6] = data & 0xFF;

	return fm34_write_raw(client, cmd_buf, sizeof(cmd_buf));
}

static int fm34_read_reg(struct i2c_client *client, unsigned short addr, unsigned short *data)
{
	LOG_FUNC();
	return 0;
}

static int fm34_patch_and_init(struct i2c_client *i2c)
{
	int ret = 0;

	struct i2c_msg msg[128];
	int i = 0;

	struct fm34_platform_data *fm34_data;
	fm34_data = fm34_i2c->dev.platform_data;

	ASSERT(fm34_data->cmd_num <= ARRAY_SIZE(msg));

	for(i = 0; i < fm34_data->cmd_num; ++i) {
		msg[i].addr = i2c->addr;
		msg[i].flags = 0;
		msg[i].len = *(fm34_data->cmd_buf + (i * 9));
		msg[i].buf = fm34_data->cmd_buf + (i * 9) + 1;
	}

	LOG_FUNC();

	ret = i2c_transfer(i2c->adapter, (struct i2c_msg *)&msg, fm34_data->cmd_num);
	if (ret == fm34_data->cmd_num)
		ret = 0;

	return ret;
}

void fm34_set_mode(enum fm34_audio_capture_mode audio_capture_mode)
{
	struct fm34_platform_data *fm34_data;

	enum fm34_mode fm34_mode_new;

	fm34_data = fm34_i2c->dev.platform_data;

	LOG_FUNC();

	switch(audio_capture_mode) {
	case AUDIO_CAPTURE_MODE_NONE:
		fm34_mode_new = FM34_BY_PASS;
		break;
	case AUDIO_CAPTURE_MODE_INT_MIC_RECORD:
		fm34_mode_new = fm34_data->mode_int_mic_record;
		break;
	case AUDIO_CAPTURE_MODE_INT_MIC_INCALL:
		fm34_mode_new = fm34_data->mode_int_mic_incall;
		break;
	case AUDIO_CAPTURE_MODE_HS_MIC_RECORD:
		fm34_mode_new = fm34_data->mode_hs_mic_record;
		break;
	case AUDIO_CAPTURE_MODE_HS_MIC_INCALL:
		fm34_mode_new = fm34_data->mode_hs_mic_incall;
		break;
	default:
		printk("<%s>:Non support audio mode\n", __FUNCTION__);
		goto error;
	}

	if (fm34_mode_new != fm34_mode_current) {
		fm34_mode_current = fm34_mode_new;
	}

	switch (fm34_mode_new) {
	case FM34_ENABLE_EC:
		fm34_write_reg(fm34_i2c, 0x2300, 0x0000);
		break;
	case FM34_BY_PASS:
		fm34_write_reg(fm34_i2c, 0x2300, 0x0004);
		break;
	default:
		printk("<%s>:Non support mode\n", __FUNCTION__);
		goto error;
	}

error:
	return;
}
EXPORT_SYMBOL(fm34_set_mode);


void fm34_power_up(void)
{
	struct fm34_platform_data *fm34_data;
	unsigned short value = 0;

	struct clk* dap_mclk;

	int retry = 0;
	int ret = 0;

	LOG_FUNC();

	dap_mclk = tegra_get_clock_by_name("clk_dev1");
	if (!dap_mclk) {
		printk("<%s> cannot get DAP1 MCLK\n", __FUNCTION__);
		return;
	}
	clk_enable(dap_mclk);

	fm34_data = fm34_i2c->dev.platform_data;

	do {
		fm34_set_power(fm34_data->pin_id_pwdn, 1);
		gpio_set_value(fm34_data->pin_id_bp, 1);
		gpio_set_value(fm34_data->pin_id_rst, 1);
		msleep(10);
		gpio_set_value(fm34_data->pin_id_rst, 0);
		msleep(10);
		gpio_set_value(fm34_data->pin_id_rst, 1);
		msleep(10);

		ret = fm34_patch_and_init(fm34_i2c);
		if(ret != 0) {
			retry++;
			printk("<%s>FM34 patch and init error: %d\n", __FUNCTION__, retry);
			continue;
		}

		msleep(250);

		fm34_read_reg(fm34_i2c, 0x22FB, &value);

		break;
	} while (retry <= FM34_INIT_MAX_RETRIES);

	/* power down after setting */
	//gpio_set_value(fm34_data->pin_id_pwdn, 0);
}
EXPORT_SYMBOL(fm34_power_up);

static __devinit int fm34_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	int result = 0;
	struct fm34_platform_data *fm34_data;

	LOG_FUNC();

	if(i2c == 0 || i2c->dev.platform_data == 0)
		return -ENODEV;

	fm34_i2c = i2c;
	fm34_data = i2c->dev.platform_data;

	if(result < 0) {
		printk("<%s>Initial Failed\n", __FUNCTION__);
	}

	/*kernel_thread(fm34_power_up, 0, 0);*/
	fm34_power_up();

	return result;
}

static __devexit int fm34_i2c_remove(struct i2c_client *client)
{
	LOG_FUNC();
	return 0;
}

static int fm34_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct fm34_platform_data *fm34_data = client->dev.platform_data;;
	LOG_FUNC();
	fm34_set_power(fm34_data->pin_id_pwdn, 0);
	return 0;
}

static int fm34_resume(struct i2c_client *client)
{
	struct fm34_platform_data *fm34_data = client->dev.platform_data;;
	LOG_FUNC();
	fm34_set_power(fm34_data->pin_id_pwdn, 1);
	return 0;
}

static const struct i2c_device_id fm34_i2c_id[] = {
	{ "fm34", 0 },
	{}
};

static struct i2c_driver fm34_i2c_driver = {
	.driver = {
		.name = "fm34",
		.owner = THIS_MODULE,
	},
	.probe		= fm34_i2c_probe,
	.remove		= __devexit_p(fm34_i2c_remove),
	.suspend	= fm34_suspend,
	.resume		= fm34_resume,

	.id_table	= fm34_i2c_id,
};

static int __init fm34_module_init(void)
{
	LOG_FUNC();
	return i2c_add_driver(&fm34_i2c_driver);
}
device_initcall_sync(fm34_module_init);

static void __exit fm34_module_exit(void)
{
	LOG_FUNC();
	i2c_del_driver(&fm34_i2c_driver);
}
module_exit(fm34_module_exit);

MODULE_AUTHOR("Andrew Hsiao <Andrew_Hsiao@pegatroncorp.com>");
MODULE_DESCRIPTION("Audio DSP FM34 driver");
MODULE_LICENSE("GPL");
