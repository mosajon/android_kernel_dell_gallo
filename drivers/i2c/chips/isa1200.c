/*
 *	isa1200.c -	Haptic Motor
 *
 *	Copyright (C) 2009 Samsung Electronics
 *	Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under	the	terms of the GNU General Public	License	version	2 as
 * published by	the	Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/haptic.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/workqueue.h>
#include <linux/i2c/isa1200.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/slab_def.h>
#include <linux/slab_def.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t2.h>
#define	TEGRA_GPIO_PP0		   120
#define	TEGRA_GPIO_PP3		   123
#define	TEGRA_GPIO_PX7		   191

#include <asm/io.h>
#include "haptic.h"

#define DEBUG 1

struct isa1200_chip	{
	struct i2c_client *client;
	struct haptic_classdev cdev;
	struct work_struct work;
	struct hrtimer timer;
	struct task_struct *wait;

	unsigned int	len;			/* LDO enable */
	unsigned int	hen;			/* Haptic enable */

	int	enable;
	int	powered;
	int	timeout;
	int	direction;

	int	level;
	int	level_max;

	int	ldo_level;
};

static struct i2c_client *this_client;
const struct device *t_dev;
#define	BUFSIZE	2
#define I2C_RETRY_COUNT 5
static DECLARE_WAIT_QUEUE_HEAD(wq);

static int isa1200_i2c_read_byte(char *rxData, int length)
{
	int i;
	struct i2c_msg msgs[] =	{
		{
			.addr =	this_client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr =	this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
#if 0
	if (i2c_transfer(this_client->adapter, msgs,	2) < 0)	{
		pr_err("%s:	transfer error\n", __func__);
		return -EIO;
	} else
		return 0;
#endif
	for ( i = 0; i < I2C_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msgs,	2) < 0)	{
			pr_err("%s()+++: i2c transfer error\n", __func__);
		} else
			return 0;
	}
	return -EIO;
}

static int isa1200_i2c_write_byte(char *txData,	int	length)
{
	int i;
	struct i2c_msg msg[] = {
		{
			.addr =	this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
#if 0
	if (i2c_transfer(this_client->adapter, msg, 1) <	0) {
		pr_err("%s: transfer error:\n", __func__);
		return -EIO;
	} else
		return 0;
#endif
	for ( i = 0; i < I2C_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) <	0) {
			pr_err("%s()+++: i2c transfer error:\n", __func__);
		} else
			return 0;
	}
	return -EIO;
}
static int isa1200_read_reg(u8 reg)
{
	char buffer[BUFSIZE	+ 1];
	memset(buffer, 0, sizeof(buffer));
	buffer[0] =	reg;
	isa1200_i2c_read_byte(buffer, 1);
	return buffer[0];
}

static void	isa1200_write_reg(u8 reg, u8 value)
{
	char buffer[BUFSIZE	+ 1];
	memset(buffer, 0, sizeof(buffer));
	buffer[0] =	reg;
	buffer[1] =	value;
	isa1200_i2c_write_byte(buffer, 2);
}

static inline struct isa1200_chip *cdev_to_isa1200_chip(
		struct haptic_classdev *haptic_cdev) {
	return container_of(haptic_cdev, struct	isa1200_chip, cdev);
}

static int isa1200_chip_set_pwm_cycle(struct isa1200_chip *haptic)
{
	u16	duty = 0;
	dev_dbg(t_dev, "%s ++\n", __func__);
	if (haptic->direction ==	2)
		duty = 126 +	(u8)(haptic->level * 126	/ 125);
	else if (haptic->direction	== 1)
		duty = 126 -	(u8)(abs(haptic->level)	* 125 / 125);
	else
		duty = 126;
	isa1200_write_reg(ISA1200_HCTRL5, duty);
	return 0;
}



static void	isa1200_chip_power_on(struct isa1200_chip *haptic)
{
	u8 value = 0;
	static u8 odflag;
	dev_dbg(t_dev, "%s\n", __func__);

	if (abs(haptic->level) == 90)
		odflag = 1;
	else
		odflag = 0;
	if (haptic->powered)
		return;
	/*gpio_set_value(haptic->hen, 1);*/
	if (1 ==	odflag)	{
		value =	isa1200_read_reg(ISA1200_HCTRL0);
		if (haptic->direction ==	1) {
			value &= ~ISA1200_OVERHL;
			isa1200_write_reg(ISA1200_HCTRL0, value);
		} else if (haptic->direction	== 2) {
			value |= ISA1200_OVERHL;
			isa1200_write_reg(ISA1200_HCTRL0, value);
		}
		value |= ISA1200_OVEREN;
		value |= ISA1200_HAPDREN;
		isa1200_write_reg(ISA1200_HCTRL0, value);
	} else {
		value =	isa1200_read_reg(ISA1200_HCTRL0);
		value |= ISA1200_HAPDREN;
		isa1200_write_reg(ISA1200_HCTRL0, value);
	}
	haptic->powered	= 1;
	isa1200_chip_set_pwm_cycle(haptic);
	if (1 ==	odflag)	{
		value &= ~ISA1200_OVEREN;
		isa1200_write_reg(ISA1200_HCTRL0, value);
	}
}

static void	isa1200_chip_power_off(struct isa1200_chip *haptic)
{
	u8 value = 0;
	dev_dbg(t_dev, "%s\n", __func__);
	if (!haptic->powered)
		return;
	haptic->powered	= 0;
	value =	0x5B;
	isa1200_write_reg(ISA1200_HCTRL5, value);
	/*gpio_set_value(haptic->hen, 0);*/
	value =	isa1200_read_reg(ISA1200_HCTRL0);
	value &= ~ISA1200_HAPDREN;
	isa1200_write_reg(ISA1200_HCTRL0, value);
}
#if	0
static void	isa1200_chip_work(struct work_struct *work)
{
	struct isa1200_chip	*haptic;
	haptic = container_of(work,	struct isa1200_chip, work);
	if (haptic->enable) {
pr_db_isa("isa1200:levle:%d
		power:%d\n", haptic->level, haptic->powered);
		isa1200_chip_power_on(haptic);
	} else {
		isa1200_chip_power_off(haptic);
	}
}
#endif
static void	isa1200_chip_work(struct isa1200_chip *haptic)
{
	dev_dbg(t_dev, "%s\n", __func__);
	if (haptic->enable) {
		dev_dbg(t_dev, "%s:isa1200_chip_power_on()\n", __func__);
		isa1200_chip_power_on(haptic);
	} else {
		dev_dbg(t_dev, "%s:isa1200_chip_power_off()\n", __func__);
		isa1200_chip_power_off(haptic);
	}
}

static enum	hrtimer_restart	isa1200_chip_timer(struct hrtimer *timer)
{
	struct isa1200_chip	*haptic	= container_of(timer, struct isa1200_chip, timer);
	haptic->enable = 0;
	haptic->timeout	= 1;
	dev_dbg(t_dev, "%s\n", __func__);
	wake_up_interruptible(&wq);

	return HRTIMER_NORESTART;
}
/*
static void	isa1200_chip_set(struct	haptic_classdev	*haptic_cdev,
							   enum	haptic_value value)
{

	   struct isa1200_chip *haptic =
			   cdev_to_isa1200_chip(haptic_cdev);

	   pr_db_isa("isa1200_chip_set:set	value()=%d\n", value);

	   switch (value) {
	   case	HAPTIC_OFF:
			   haptic->enable =	0;
			   break;
	   case	HAPTIC_HALF:
	   case	HAPTIC_FULL:
	   default:
			   haptic->enable =	1;
			   break;
	   }

	   schedule_work(&haptic->work);
}

static enum	haptic_value isa1200_chip_get(struct haptic_classdev *haptic_cdev)
{

	   struct isa1200_chip *haptic =
			   cdev_to_isa1200_chip(haptic_cdev);

	   if (haptic->enable)
			   return HAPTIC_FULL;

	   return HAPTIC_OFF;
}
*/
static int wait_thread(void	*p)
{
	struct isa1200_chip	*haptic	= (struct isa1200_chip *)p;
	while (!kthread_should_stop()) {
		haptic->timeout	= 0;
		wait_event_interruptible(wq, (haptic->timeout == 1));
		isa1200_chip_power_off(haptic);
	}
	return 0;
}

static void	isa1200_config_i2c(void)
{
	u8 value, outtest =	0;
	value =	ISA1200_LDOADJ_35V;
	dev_dbg(t_dev, "%s++\n", __func__);
	isa1200_write_reg(ISA1200_SCTRL0, 0x0F);
	outtest	= isa1200_read_reg(ISA1200_SCTRL0);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	ISA1200_EXTCLKSEL |	ISA1200_BIT6_ON	| ISA1200_MOTTYP_ERM | 0x0;
	isa1200_write_reg(ISA1200_HCTRL1, 0xE0);
	outtest	= isa1200_read_reg(ISA1200_HCTRL1);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	0x00;
	isa1200_write_reg(ISA1200_HCTRL2, value);
	outtest	= isa1200_read_reg(ISA1200_HCTRL2);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	0x13;
	isa1200_write_reg(ISA1200_HCTRL3, value);
	outtest	= isa1200_read_reg(ISA1200_HCTRL3);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	0x00;
	isa1200_write_reg(ISA1200_HCTRL4, value);
	outtest	= isa1200_read_reg(ISA1200_HCTRL4);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	0x7E;
	isa1200_write_reg(ISA1200_HCTRL5, value);
	outtest	= isa1200_read_reg(ISA1200_HCTRL5);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	0xFC;
	isa1200_write_reg(ISA1200_HCTRL6, value);
	outtest	= isa1200_read_reg(ISA1200_HCTRL6);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
	value =	ISA1200_HAPDREN	| ISA1200_HAPDIGMOD_PWM_GEN	| ISA1200_PWMMOD_DIVIDER_256;
	isa1200_write_reg(ISA1200_HCTRL0, 0x11);
	outtest	= isa1200_read_reg(ISA1200_HCTRL0);
	dev_dbg(t_dev, "%s:rbValue=%x\n", __func__, outtest);
}

#define	ATTR_DEF_SHOW(name)	\
	static ssize_t isa1200_chip_show_##name(struct device *dev,	\
											struct device_attribute	*attr, char	*buf) \
	{ \
		struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);	\
		struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev); \
		\
		return sprintf(buf,	"%u\n",	haptic->name) +	1; \
	}

#define	ATTR_DEF_STORE(name) \
	static ssize_t isa1200_chip_store_##name(struct	device *dev, \
					struct device_attribute	*attr, \
					const char *buf, size_t	size) \
	{ \
		struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);	\
		struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev); \
		ssize_t	ret	= -EINVAL; \
		unsigned long val; \
		\
		ret	= strict_strtoul(buf, 10, &val); \
		if (ret	== 0) {	\
			ret	= size;	\
			haptic->name = val;	\
			isa1200_chip_work(haptic);\
		} \
		\
		return ret;	\
	}

ATTR_DEF_SHOW(enable);
ATTR_DEF_STORE(enable);
static DEVICE_ATTR(enable, 0644, isa1200_chip_show_enable,
				   isa1200_chip_store_enable);

static ssize_t isa1200_chip_show_level(struct device *dev,
									   struct device_attribute *attr, char *buf)
{
	struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);
	struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev);
	return sprintf(buf,	"%u\n",	abs(haptic->level))	+ 1;
}

static ssize_t isa1200_chip_store_level(struct device *dev,
										struct device_attribute	*attr,
										const char *buf, size_t	size)
{
	struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);
	struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev);
	ssize_t	ret	= -EINVAL;
	long val = 0;
	ret	= strict_strtol(buf, 10, &val);
	if (ret == 0) {
		ret	= size;
		if (haptic->level_max < abs(val))
			val	= haptic->level_max;
		haptic->level =	val + PWM_HAPTIC_SHIFT_LEVEL;
		isa1200_chip_set_pwm_cycle(haptic);
	}
	return ret;
}
static DEVICE_ATTR(level, 0644,	isa1200_chip_show_level,
				   isa1200_chip_store_level);


static ssize_t isa1200_chip_show_direction(struct device *dev,
				struct device_attribute	*attr, char	*buf)
{
	struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);
	struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev);
	return sprintf(buf,	"%u\n",	haptic->direction) + 1;
}

static ssize_t isa1200_chip_store_direction(struct device *dev,
				struct device_attribute	*attr,
				const char *buf, size_t	size)
{
	struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);
	struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev);
	ssize_t	ret	= -EINVAL;
	long val = 0;
	ret	= strict_strtol(buf, 10, &val);
	if (ret == 0) {
		ret	= size;
		switch (val)	{
		case 2:
			haptic->direction =	2;
			break;
		case 1:
			haptic->direction =	1;
			break;
		case 0:
			haptic->direction =	0;
			break;
		default:
			break;
		}
		isa1200_chip_set_pwm_cycle(haptic);
	}
	return ret;
}
static DEVICE_ATTR(direction, 0644,	isa1200_chip_show_direction,
				   isa1200_chip_store_direction);


ATTR_DEF_SHOW(level_max);
static DEVICE_ATTR(level_max, 0444,	isa1200_chip_show_level_max, NULL);

static ssize_t isa1200_chip_store_oneshot(struct device	*dev,
				struct device_attribute	*attr,
				const char *buf, size_t	size)
{
	struct haptic_classdev *haptic_cdev	= dev_get_drvdata(dev);
	struct isa1200_chip	*haptic	= cdev_to_isa1200_chip(haptic_cdev);
	ssize_t	ret	= -EINVAL;
	unsigned long val;
	ret	= strict_strtoul(buf, 10, &val);
	if (ret == 0) {
		hrtimer_cancel(&haptic->timer);
		ret	= size;
		if (0 ==	val) {
			isa1200_chip_power_off(haptic);
		} else {
			isa1200_chip_power_on(haptic);
			hrtimer_start(&haptic->timer,
						  ktime_set(val	/ 1000,	(val % 1000) * 1000000),
						  HRTIMER_MODE_REL);
		}
	}
	return ret;
}
static DEVICE_ATTR(oneshot,	0644, NULL,	isa1200_chip_store_oneshot);

static struct attribute	*haptic_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_level.attr,
	&dev_attr_level_max.attr,
	&dev_attr_oneshot.attr,
	&dev_attr_direction.attr,
	NULL,
};

static const struct	attribute_group	haptic_group = {
	.attrs = haptic_attributes,
};

static void	isa1200_setup(struct i2c_client	*client)
{
	dev_dbg(t_dev, "%s\n", __func__);
	isa1200_config_i2c();
}

static int __devinit isa1200_probe(struct i2c_client *client,
								   const struct	i2c_device_id *id)
{
	struct isa1200_chip	*chip;
	int	ret;
	pr_info("%s ++++\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))	{
		pr_err("isa1200	i2c	failed\n");
		return -ENODEV;
	}
	this_client	= client;
	t_dev = &this_client->dev;
	chip = kzalloc(sizeof(struct isa1200_chip),	GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->cdev.show_enable = isa1200_chip_show_enable;
	chip->cdev.store_enable	= isa1200_chip_store_enable;
	chip->cdev.store_oneshot = isa1200_chip_store_oneshot;
	chip->cdev.show_level =	isa1200_chip_show_level;
	chip->cdev.store_level = isa1200_chip_store_level;
	chip->cdev.show_level_max =	isa1200_chip_show_level_max;
	chip->cdev.show_direction =	isa1200_chip_show_direction;
	chip->cdev.store_direction = isa1200_chip_store_direction;
	chip->cdev.name	= "isa1200";
	chip->enable = 0;
	chip->level	= PWM_HAPTIC_DEFAULT_LEVEL;
	chip->level_max	= PWM_HAPTIC_MAX_LEVEL;
	chip->timeout =	0;
	chip->powered =	0;
	chip->direction	= 2;
	chip->len =	TEGRA_GPIO_PP0;
	chip->hen =	TEGRA_GPIO_PP3;
	/*
	gpio_request(chip->hen, "HAPTIC_HEN");
	gpio_direction_output(chip->hen, 1);
	gpio_export(chip->hen, true);
	delay(50);
	gpio_request(chip->len, "HAPTIC_LEN");
	gpio_direction_output(chip->len, 1);
	gpio_export(chip->len, true);
	*/
	tegra_gpio_enable(TEGRA_GPIO_PX7);
	ret	= gpio_request(TEGRA_GPIO_PX7, "SPI1_MISO");
	if (ret < 0)
		return ret;
	ret	= gpio_direction_output(TEGRA_GPIO_PX7, 1);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PX7);
		return ret;
	}
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPIF, TEGRA_TRI_NORMAL);

	tegra_gpio_enable(chip->hen);
	ret	= gpio_request(chip->hen, "HAPTIC_HEN");
	if (ret < 0)
		return ret;
	ret	= gpio_direction_output(chip->hen, 1);
	if (ret < 0)	{
		gpio_free(chip->hen);
		return ret;
	}
	udelay(50);
	tegra_gpio_enable(chip->len);
	ret	= gpio_request(chip->len, "HAPTIC_LEN");
	if (ret < 0)
		return ret;
	ret	= gpio_direction_output(chip->len, 1);
	if (ret < 0)	{
		gpio_free(chip->len);
		return ret;
	}
	/* register	our	new	haptic device */
	ret	= haptic_classdev_register(&client->dev, &chip->cdev);
	if (ret < 0)	{
		pr_err("haptic_classdev_register	failed\n");
		goto error_classdev;
	}
	ret	= sysfs_create_group(&chip->cdev.dev->kobj,	&haptic_group);
	if (ret)
		goto error_enable;
	hrtimer_init(&chip->timer, CLOCK_MONOTONIC,	HRTIMER_MODE_REL);
	chip->timer.function = isa1200_chip_timer;
	chip->wait = kthread_run(wait_thread, chip,	"wait");
	i2c_set_clientdata(client, chip);
	isa1200_setup(client);
	return 0;
error_enable:
	sysfs_remove_group(&chip->cdev.dev->kobj, &haptic_group);
error_classdev:
	haptic_classdev_unregister(&chip->cdev);
	/*error_pwm:
		   pwm_free(chip->pwm);*/
	kfree(chip);
	return ret;
}

static int __devexit isa1200_remove(struct i2c_client *client)
{
	struct isa1200_chip	*chip =	i2c_get_clientdata(client);
	pr_info("%s\n ++++", __func__);
	chip->timeout =	1;
	kthread_stop(chip->wait);
	hrtimer_cancel(&chip->timer);
	if (gpio_is_valid(chip->len))
		gpio_free(chip->len);
	sysfs_remove_group(&chip->cdev.dev->kobj, &haptic_group);
	haptic_classdev_unregister(&chip->cdev);
	kfree(chip);
	return 0;
}
#define CONFIG_PM 1
#if CONFIG_PM
static int isa1200_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct isa1200_chip	*chip =	i2c_get_clientdata(client);
	isa1200_chip_power_off(chip);

	ret	= gpio_direction_output(TEGRA_GPIO_PX7, 0);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PX7);
		return ret;
	}

	ret	= gpio_direction_output(TEGRA_GPIO_PP0, 0);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PP0);
		return ret;
	}
	udelay(50);
	ret	= gpio_direction_output(TEGRA_GPIO_PP3, 0);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PP3);
		return ret;
	}

	return 0;
}

static int isa1200_resume(struct i2c_client	*client)
{
	int ret;
	ret	= gpio_direction_output(TEGRA_GPIO_PX7, 1);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PX7);
		return ret;
	}

	ret	= gpio_direction_output(TEGRA_GPIO_PP0, 1);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PP0);
		return ret;
	}

	udelay(50);
	ret	= gpio_direction_output(TEGRA_GPIO_PP3, 1);
	if (ret < 0)	{
		gpio_free(TEGRA_GPIO_PP3);
		return ret;
	}
	isa1200_setup(client);
	return 0;
}
#else
#define	isa1200_suspend				   NULL
#define	isa1200_resume		   NULL
#endif

static const struct	i2c_device_id isa1200_id[] = {
	{ "isa1200", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isa1200_id);

static struct i2c_driver msm_isa1200_driver	= {
	.driver	= {
		.name	= "isa1200",
	},
	.probe			= isa1200_probe,
	 .remove		 = __devexit_p(isa1200_remove),
	  .suspend		  =	isa1200_suspend,
	   .resume		   = isa1200_resume,
		.id_table		= isa1200_id,
	 };

static int __init isa1200_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&msm_isa1200_driver);
}

static void	__exit isa1200_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&msm_isa1200_driver);
}

late_initcall(isa1200_init);
module_exit(isa1200_exit);

MODULE_AUTHOR("Kyungmin	Park <kyungmin.park@samsung.com>");
MODULE_DESCRIPTION("ISA1200	Haptic Motor driver");
MODULE_LICENSE("GPL");
