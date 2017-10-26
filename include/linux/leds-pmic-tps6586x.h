/*
 * pca9532.h - platform data structure for pca9532 led controller
 *
 * Copyright (C) 2008 Riku Voipio <riku.voipio@movial.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://www.nxp.com/acrobat/datasheets/PCA9532_3.pdf
 *
 */

#ifndef __LINUX_LEDS_TPS6586X_H
#define __LINUX_LEDS_TPS6586X_H

#include <linux/leds.h>
#include <linux/workqueue.h>

#define TPS6586X_LEDS_MAX_NUM 5

enum pmic_tps6586x_type { PMIC_TPS6586X_TYPE_NONE, PMIC_TPS6586X_TYPE_LED };


struct pmic_tps6586x_led {
	u8 id;
	u8 brinking;
	struct platform_device *client;
	char *name;
	struct led_classdev ldev;
       struct work_struct work;
	enum pmic_tps6586x_type type;
	enum led_brightness brightness;
	u8 blink;
};

struct pmic_tps6586x_platform_data {
	u8 num_leds;
	struct pmic_tps6586x_led leds[TPS6586X_LEDS_MAX_NUM];
};

#endif /* __LINUX_LEDS_TPS6586X_H */

