/*
 * Copyright (C) 2010 Pegatron Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/i2c/bq20z45_power.h>
#include <linux/i2c/cg7216am.h>

/* #define DEBUG */

#ifdef DEBUG
#define LOG_FUNC() printk("<%s>\n", __FUNCTION__)
#define LOG_MSG(x...) pr_err(x)
#else
#define LOG_FUNC() do {} while (0)
#define LOG_MSG(x...) do {} while (0);
#endif

static int gallo_get_dead_bat_recovery_timeout(void);
static int gallo_get_over_dischg_bat_recovery_timeout(void);

char *gallo_battery_vendors[] = {
	"DP-ATL218",   /* DynaPack*/
	"Samsung SDI",  /* SDI */
};


#define GALLO_DEAD_BAT_RECOVERY_TIMEOUT_S (3*60)
#define GALLO_OVER_DISCHARGE_BAT_RECOVERY_TIMEOUT_S (20*60)
#define GALLO_OVER_DISCHARGE_CRITERIA (9300)

static struct platform_device *pdev_sb_impl;

static int __devinit cg7216am_battery_probe(struct platform_device *pdev)
{
	LOG_FUNC();
	pdev_sb_impl = pdev;
	return 0;
}

static int __devexit cg7216am_battery_remove(struct platform_device *pdev)
{
	LOG_FUNC();
	pdev_sb_impl = NULL;
	return 0;
}

static int cg7216am_battery_suspend(struct platform_device *pdev , pm_message_t state)
{
	LOG_FUNC();

	return 0;
}

static int cg7216am_battery_resume(struct platform_device *pdev)
{
	LOG_FUNC();
	return 0;
}

static struct platform_driver cg7216am_battery_platform_driver = {
	.driver	= {
		.name	= "cg7216am-battery",
		.owner	= THIS_MODULE,
	},
	.probe	= cg7216am_battery_probe,
	.remove	= __devexit_p(cg7216am_battery_remove),
	.suspend	= cg7216am_battery_suspend,
	.resume		= cg7216am_battery_resume,
};

static inline struct device *to_cg7216am_dev(struct device *dev)
{
	return dev->parent;
}

static bool gallo_sb_impl_init(void)
{
	int rc;
	LOG_FUNC();

	pdev_sb_impl = NULL;
	rc = platform_driver_register(&cg7216am_battery_platform_driver);
	if (rc < 0) {
		printk(KERN_ERR 	"%s: Failed to register MCU driver\n",  __func__);
	}
	return (rc >= 0);
}

static bool gallo_validate_battery_vendor(char *cur_vendor)
{
	int bat_vendor_num = ARRAY_SIZE(gallo_battery_vendors);
	int i = 0;

	if (strlen(cur_vendor) == 0) {
		return true;
	}

	for (i = 0; i < bat_vendor_num; i++) {
		if (!strcmp(gallo_battery_vendors[i], cur_vendor)) {
			LOG_MSG("%s.  valid battery vender = '%s' \n", __func__, cur_vendor);
			return true;
		}
	}
	printk(KERN_ERR"%s.  invalid battery vender = '%s' \n", __func__, cur_vendor);
	return false;
}

static int gallo_get_dead_bat_recovery_timeout_default(void)
{
	int rtn = 0;
	if (pdev_sb_impl == NULL) {
		return -ENODEV;
	}

	rtn = GALLO_DEAD_BAT_RECOVERY_TIMEOUT_S;
	printk(KERN_INFO "%s.  sec = %d \n", __func__, rtn);
	return rtn;
}

static int gallo_get_dead_bat_recovery_timeout(void)
{
	int sec = 0;
	if (pdev_sb_impl == NULL) {
		return -ENODEV;
	}

	sec = cg7216am_get_dead_bat_remain_timeout(
			to_cg7216am_dev(&pdev_sb_impl->dev));

	printk(KERN_INFO "%s.  sec = %d \n", __func__, sec);
	return sec;
}

static int gallo_get_over_dischg_bat_recovery_timeout_default(void)
{
	int rtn = 0;
	if (pdev_sb_impl == NULL) {
		return -ENODEV;
	}

	rtn = GALLO_OVER_DISCHARGE_BAT_RECOVERY_TIMEOUT_S;
	printk(KERN_INFO "%s.  sec = %d \n", __func__, rtn);
	return rtn;
}

static int gallo_get_over_dischg_bat_recovery_timeout(void)
{
	int sec = 0;
	if (pdev_sb_impl == NULL) {
		return -ENODEV;
	}

	sec = cg7216am_get_over_dischg_bat_remain_timeout(
			to_cg7216am_dev(&pdev_sb_impl->dev));

	printk(KERN_INFO "%s.  sec = %d \n", __func__, sec);
	return sec;
}

void gallo_store_dead_bat_recovery_timeout(int sec)
{
	if (pdev_sb_impl == NULL) {
		return;
	}

	printk(KERN_INFO "%s.  sec = %d \n", __func__, sec);
	cg7216am_store_dead_bat_remain_timeout(
			to_cg7216am_dev(&pdev_sb_impl->dev), sec);
}

void gallo_store_over_dischg_bat_recovery_timeout(int sec)
{
	if (pdev_sb_impl == NULL) {
		return;
	}

	printk(KERN_INFO "%s.  sec = %d \n", __func__, sec);
	cg7216am_store_over_dischg_bat_remain_timeout(
			to_cg7216am_dev(&pdev_sb_impl->dev), sec);
}

void gallo_set_judgement_finished(int item, bool done)
{
	if (pdev_sb_impl == NULL) {
		return;
	}

	printk(KERN_INFO "%s.  judgement = %d, done =%d \n", __func__, item, done);
	cg7216am_set_judgement_finished(
			to_cg7216am_dev(&pdev_sb_impl->dev), item, done);
}

static void gallo_ce_enable(bool enable)
{
	if (pdev_sb_impl == NULL) {
		return;
	}

	printk(KERN_INFO "%s: enable:%d\n", __func__, enable);

	cg7216am_battery_disable_charging(
			to_cg7216am_dev(&pdev_sb_impl->dev), !enable);
}


struct smart_battery_charging_impl gallo_sb_impl = {
	.init = gallo_sb_impl_init,
	.ce_enable = gallo_ce_enable,
	.validate_manufacture = gallo_validate_battery_vendor,
	.get_dead_bat_recovery_timeout_default = gallo_get_dead_bat_recovery_timeout_default,
	.get_dead_bat_recovery_timeout = gallo_get_dead_bat_recovery_timeout,
	.get_over_dischg_bat_recovery_timeout_default = gallo_get_over_dischg_bat_recovery_timeout_default,
	.get_over_dischg_bat_recovery_timeout = gallo_get_over_dischg_bat_recovery_timeout,
	.over_discharge_criteria = GALLO_OVER_DISCHARGE_CRITERIA,
	.store_dead_bat_recovery_timeout = gallo_store_dead_bat_recovery_timeout,
	.store_over_dischg_bat_recovery_timeout = gallo_store_over_dischg_bat_recovery_timeout,
	.set_judgement_finished = gallo_set_judgement_finished,
};


MODULE_DESCRIPTION("smart battery charging implement");
MODULE_LICENSE("GPL");

