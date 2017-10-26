/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>

#include <asm/gpio.h>

#define LONG_PRESS_TIMEOUT 1000

struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct delayed_work dwork;
	bool is_long_press;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct gpio_button_data data[0];
};

struct gpio_button_data *p_vol_up_bdata;
struct gpio_button_data *p_vol_down_bdata;

#define TEGRA_GPIO_PQ4		132
#define TEGRA_GPIO_PQ5		133

static void gpio_keys_report_event(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, dwork.work);
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
	int code = button->code;

	if (!gpio_get_value(TEGRA_GPIO_PQ4) && !gpio_get_value(TEGRA_GPIO_PQ5)) {
		code = KEY_MENU;

		p_vol_up_bdata->is_long_press = true;
		p_vol_down_bdata->is_long_press = true;
	}
	else if (code == KEY_VOLUMEUP)
		code = KEY_HOME;
	else /* if (code == KEY_VOLUMEDOWN) */
		code = KEY_BACK;

	printk(KERN_INFO "[Keypad] Long press: %d, keycode: %d ---> %s (%d)\n", button->gpio, code, __FUNCTION__, __LINE__);

	input_event(input, type, code, !!state);
	input_sync(input);

	input_event(input, type, code, !state);
	input_sync(input);

	bdata->is_long_press = true;

	cancel_delayed_work(&p_vol_up_bdata->dwork);
	cancel_delayed_work(&p_vol_down_bdata->dwork);
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;

	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
	int code = button->code;

	BUG_ON(irq != gpio_to_irq(button->gpio));

	if (code != KEY_VOLUMEUP && code != KEY_VOLUMEDOWN) {
		input_event(input, type, code, !!state);
		input_sync(input);
		return IRQ_HANDLED;
	}

	if (state) {	/* press */
		cancel_delayed_work(&bdata->dwork);
		schedule_delayed_work(&bdata->dwork, msecs_to_jiffies(LONG_PRESS_TIMEOUT));
	} else {		/* release */
		if (bdata->is_long_press) {
			if (code == KEY_VOLUMEUP)
				code = KEY_HOME;
			else
				code = KEY_BACK;

			bdata->is_long_press = false;
		} else { 	/* this is short press */
			cancel_delayed_work(&bdata->dwork);

			input_event(input, type, code, !state);
			input_sync(input);

			input_event(input, type, code, !!state);
			input_sync(input);
		}
	}

	return IRQ_HANDLED;
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	ddata->input = input;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		int irq;
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;

		INIT_DELAYED_WORK(&bdata->dwork, gpio_keys_report_event);

		error = gpio_request(button->gpio, button->desc ?: "gpio_keys");
		if (error < 0) {
			pr_err("gpio-keys: failed to request GPIO %d,"
				" error %d\n", button->gpio, error);
			goto fail2;
		}

		error = gpio_direction_input(button->gpio);
		if (error < 0) {
			pr_err("gpio-keys: failed to configure input"
				" direction for GPIO %d, error %d\n",
				button->gpio, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			pr_err("gpio-keys: Unable to get irq number"
				" for GPIO %d, error %d\n",
				button->gpio, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		error = request_irq(irq, gpio_keys_isr,
				    IRQF_SHARED |
				    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				    button->desc ? button->desc : "gpio_keys",
				    bdata);
		if (error) {
			pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				irq, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);

		if (button->code == KEY_VOLUMEUP)
			p_vol_up_bdata = bdata;
		else if (button->code == KEY_VOLUMEDOWN)
			p_vol_down_bdata = bdata;
	}

	input_set_capability(input, EV_KEY, KEY_HOME);
	input_set_capability(input, EV_KEY, KEY_BACK);
	input_set_capability(input, EV_KEY, KEY_MENU);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		cancel_delayed_work_sync(&ddata->data[i].dwork);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		cancel_delayed_work_sync(&ddata->data[i].dwork);
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
		}
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				disable_irq_wake(irq);
			}
		}
	}

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
