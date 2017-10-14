/*
 * tps6586x.c - 16-bit Led dimmer
 *
 * Copyright (C) 2008 Riku Voipio <riku.voipio@movial.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://www.nxp.com/acrobat/datasheets/TPS6586X_3.pdf
 *
 */


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/leds-pmic-tps6586x.h>
#include <linux/mfd/tps6586x.h>



#define TEGRA_GPIO_PR5		141

#define DEBUG 1

#define ldev_to_led(c)       container_of(c, struct pmic_tps6586x_led, ldev)


struct pmic_tps6586x_data {
	u8 num_leds;
	struct platform_device *client;
	struct pmic_tps6586x_led leds[TPS6586X_LEDS_MAX_NUM];
	struct mutex update_lock;
	struct input_dev    *idev;
	struct work_struct work;
};

static inline struct device *to_tps6586x_dev(struct device *dev)
{
	return dev->parent;
}

static void pmic_tps6586x_setled(struct pmic_tps6586x_led *led)
{
	int err;
	u8 buff[1];
	struct platform_device *client = led->client;
	struct pmic_tps6586x_data *data = dev_get_drvdata(&client->dev);
	struct device *tps_dev = to_tps6586x_dev(&client->dev);
	gpio_direction_output(TEGRA_GPIO_PR5, 1);
	mutex_lock(&data->update_lock);
	pr_err("pmic_tps6586x_setled(): id=%d ++ \n", led->id);
	if (led->id == 0) {
		/* red led, PMU: Red PWM pin */
		if (led->brightness == 0) {
			if (led->blink == 0) {
				buff[0] = 0xa0;
				tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
			} else {
				buff[0] = 0xf0;
				tps6586x_writes(tps_dev, 0x50,  sizeof(buff), buff);
				buff[0] = 0xbf;
				tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
				buff[0] = 0x00;
				tps6586x_writes(tps_dev, 0x53,  sizeof(buff), buff);
				buff[0] = 0x80;
				tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
			}
		} else {
			buff[0] = 0xff;
			tps6586x_writes(tps_dev, 0x50,  sizeof(buff), buff);
			buff[0] = 0xbf;
			tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
			buff[0] = 0x00;
			tps6586x_writes(tps_dev, 0x53,  sizeof(buff), buff);
			buff[0] = 0x80;
			tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
		}
	} else if (led->id == 1) {
		/* green led, PMU: Green PWM pin */
		if (led->brightness == 0) {
			if (led->blink == 0) {
				buff[0] &= 0xe0;
				tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
			} else {
				buff[0] = 0xf0;
				tps6586x_writes(tps_dev, 0x50,  sizeof(buff), buff);
				buff[0] = 0xa0;
				tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
				buff[0] = 0xdf;
				tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
			}
		} else {
			buff[0] = 0xff;
			tps6586x_writes(tps_dev, 0x50,  sizeof(buff), buff);
			buff[0] = 0xa0;
			tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
			buff[0] = 0xdf;
			tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
		}
	} else{
		/* blue led, PMU: Blue PWM pin */
		if (led->brightness == 0) {
			if (led->blink == 0) {
				buff[0] = 0x00;
				tps6586x_writes(tps_dev, 0x53,  sizeof(buff), buff);
			} else {
				buff[0] = 0xf0;
				tps6586x_writes(tps_dev, 0x50,  sizeof(buff), buff);
				buff[0] = 0xa0;
				tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
				buff[0] = 0xff;
				tps6586x_writes(tps_dev, 0x53,  sizeof(buff), buff);
				buff[0] = 0x80;
				tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
			}
		} else {
			buff[0] = 0xff;
			tps6586x_writes(tps_dev, 0x50,  sizeof(buff), buff);
			buff[0] = 0xa0;
			tps6586x_writes(tps_dev, 0x51,  sizeof(buff), buff);
			buff[0] = 0xff;
			tps6586x_writes(tps_dev, 0x53,  sizeof(buff), buff);
			buff[0] = 0x80;
			tps6586x_writes(tps_dev, 0x52,  sizeof(buff), buff);
		}

	}
	mutex_unlock(&data->update_lock);
}

static void pmic_tps6586x_set_brightness(struct led_classdev *led_cdev,
										 enum led_brightness value)
{
	struct pmic_tps6586x_led *led = ldev_to_led(led_cdev);
	led->brightness = value;
	schedule_work(&led->work);
}

static void pmic_tps6586x_set_blinking(struct led_classdev *led_cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct pmic_tps6586x_led *led = ldev_to_led(led_cdev);
	if (*delay_on > 0 && *delay_off > 0) {
		led->blink = 1;
	} else {
		led->blink = 0;
	}
	schedule_work(&led->work);
}

static void pmic_tps6586x_led_work(struct work_struct *work)
{
	struct pmic_tps6586x_led *led;
	led = container_of(work, struct pmic_tps6586x_led, work);
	pmic_tps6586x_setled(led);
}

static int pmic_tps6586x_configure(struct platform_device *pdev,
								    struct pmic_tps6586x_data *data, struct pmic_tps6586x_platform_data *pdata)
{
	int i, err = 0;

	data->num_leds = pdata->num_leds;
		for (i = 0; i < pdata->num_leds; i++) {
		struct pmic_tps6586x_led *led = &data->leds[i];
		struct pmic_tps6586x_led *pled = &pdata->leds[i];

		led->client = pdev;
		led->id = i;
		led->type = pled->type;

		switch (led->type) {
		case PMIC_TPS6586X_TYPE_LED:
			led->brightness = pled->brightness;
			led->name =  pled->name;
			led->ldev.name = led->name;

			led->ldev.brightness = LED_OFF;
			led->ldev.brightness_set = pmic_tps6586x_set_brightness;
			led->ldev.blink_set = pmic_tps6586x_set_blinking;

			INIT_WORK(&led->work, pmic_tps6586x_led_work);

			err = led_classdev_register(&pdev->dev, &led->ldev);

			if (err < 0) {
				dev_err(&pdev->dev,
						"couldn't register LED %s\n",
						led->name);
				goto exit;
			}

			/*pmic_tps6586x_setled(led);*/

			break;
		default:
			break;
		}
	}
	return 0;
exit:
	if (i > 0)
		for (i = i - 1; i >= 0; i--)
			switch (data->leds[i].type) {
			case PMIC_TPS6586X_TYPE_LED:
				led_classdev_unregister(&data->leds[i].ldev);
				cancel_work_sync(&data->leds[i].work);
				break;
			default:
				break;
			}

	return err;
}

static int pmic_tps6586x_probe(struct platform_device *pdev)
{
	struct pmic_tps6586x_data *data = NULL;
	struct pmic_tps6586x_platform_data *pmic_tps6586x_pdata = pdev->dev.platform_data;

	int err;
	if (!pmic_tps6586x_pdata)
		return -EIO;
	data = kzalloc(sizeof(struct pmic_tps6586x_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	dev_info(&pdev->dev, "setting platform data\n");
	dev_set_drvdata(&pdev->dev, data);
	data->client = pdev;
	mutex_init(&data->update_lock);

	gpio_request(TEGRA_GPIO_PR5, "LED_SWITCH");
	/*gpio_direction_output(TEGRA_GPIO_PR5, 1);
	gpio_export(TEGRA_GPIO_PR5, true);*/


	err = pmic_tps6586x_configure(pdev, data, pmic_tps6586x_pdata);
	if (err) {
		kfree(data);
		dev_set_drvdata(&pdev->dev, NULL);
	}
	return err;
}

static int pmic_tps6586x_remove(struct platform_device *pdev)
{
	struct pmic_tps6586x_data *data = dev_get_drvdata(&pdev->dev);
	int i;
	for (i = 0; i < data->num_leds; i++)
		switch (data->leds[i].type) {
		case PMIC_TPS6586X_TYPE_NONE:
			break;
		case PMIC_TPS6586X_TYPE_LED:
			led_classdev_unregister(&data->leds[i].ldev);
			cancel_work_sync(&data->leds[i].work);
			break;
		default:
			pr_err("pmic_tps6586x_remove(): not support \n");
		}
	kfree(data);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static struct platform_driver pmic_tps6586x_driver = {
	.driver	= {
		.name	= "tps6586x-leds",
		.owner	= THIS_MODULE,
	},
	.probe	= pmic_tps6586x_probe,
	.remove	= pmic_tps6586x_remove,
};


static int __init tps6586x_leds_init(void)
{
	printk(KERN_ERR "tps6586x_leds_init() ++ \n");
	return platform_driver_register(&pmic_tps6586x_driver);
}
module_init(tps6586x_leds_init);

static void __exit tps6586x_leds_exit(void)
{
	platform_driver_unregister(&pmic_tps6586x_driver);
}
module_exit(tps6586x_leds_exit);

MODULE_DESCRIPTION("TI TPS6586x leds driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-tps6586x")
