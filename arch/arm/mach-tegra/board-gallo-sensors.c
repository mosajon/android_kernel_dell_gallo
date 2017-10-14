/*
 * arch/arm/mach-tegra/board-gallo-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/akm8975.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <mach/pinmux.h>

#ifdef CONFIG_VIDEO_YUV
#include <media/yuv_sensor.h>
#endif /* CONFIG_VIDEO_YUV */
#ifdef CONFIG_VIDEO_MT9D115
#include <media/mt9d115.h>
#endif /* CONFIG_VIDEO_T9D115 */
#ifdef CONFIG_VIDEO_MT9P111
#include <media/mt9p111.h>
#endif /* CONFIG_VIDEO_MT9P111 */
#include <linux/delay.h>
#include <generated/mach-types.h>
#include <linux/i2c/bq20z45_power.h>
#include "gpio-names.h"
#include "board.h"
#include "board-gallo.h"

#include <linux/i2c/cg7216am.h>
#include <linux/leds-cg7216am.h>
#include <linux/isl29018.h>

#define BQ24617_AC_PRESENT_GPIO	TEGRA_GPIO_PV3
#define BQ24617_NCE		TEGRA_GPIO_PR6

#define BQ24617_STAT1		TEGRA_GPIO_PQ3
#define BQ24617_STAT2		TEGRA_GPIO_PQ2
#define MCU_INTERRUPT		TEGRA_GPIO_PQ6
#define SYS_SUS			TEGRA_GPIO_PU4
/* For Gallo DVT2 use only*/
#define MCU_RESET_DVT2		TEGRA_GPIO_PP1
#define MCU_RESET_DVT3		TEGRA_GPIO_PC1
#define MCU_WAKE		TEGRA_GPIO_PU3
#define AP_CHG			TEGRA_GPIO_PQ3
#define AP_EOC			TEGRA_GPIO_PQ2

#define HW_LOW_BATTERY		TEGRA_GPIO_PW3
#define ISL29018_IRQ_GPIO	TEGRA_GPIO_PZ2
#define ISL29018_POWER_EN	TEGRA_GPIO_PV4
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define KXTF9_IRQ_GPIO		TEGRA_GPIO_PN4
#define KXTF9_PWR_SHDN		TEGRA_GPIO_PV7
#define MPU3050_IRQ_GPIO	TEGRA_GPIO_PZ4
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
#define YUV_SENSOR_OE_L_GPIO    TEGRA_GPIO_PL2
#define YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PL4
#define CAM_GPIO_RESET_L_GPIO	TEGRA_GPIO_PA3
#define NCT1008_THERM_GPIO	TEGRA_GPIO_PS2
#define NCT1008_THERM2_GPIO_EVT1	TEGRA_GPIO_PN6
#define NCT1008_THERM2_GPIO_EVT1_2	TEGRA_GPIO_PC7
#define LDS6202_IRQ_DVT2	TEGRA_GPIO_PQ7
#define LDS6202_IRQ_DVT3	TEGRA_GPIO_PQ1
#define LDS6202_RST	TEGRA_GPIO_PV0

#define	BATTERY_POLL_PERIOD	(3*HZ)
#define	BATTERY_STATUS_LATENCY	(1*HZ)

#ifndef CONFIG_BATTERY_BQ20Z45
#error "Gallo have to support BQ20Z45 currently"
#endif

#ifndef CONFIG_BATTERY_BQ30423
#error "Gallo have to support BQ30423 currently"
#endif

struct tegra_camera_gpios {
	int gpio;
	int value;
};

#define TEGRA_CAMERA_GPIO(_gpio, _value)	\
	{								\
		.gpio = _gpio,						\
		.value = _value,					\
	}

static struct tegra_camera_gpios gallo_camera_gpio_keys[] = {
{CAM2_PWR_DN_GPIO, 0},
{CAM2_RST_L_GPIO, 0},
{CAM2_LDO_SHUTDN_L_GPIO, 0},
{CAM3_PWR_DN_GPIO, 0},
{CAM3_RST_L_GPIO, 0},
{CAM3_LDO_SHUTDN_L_GPIO, 0},
{CAM3_VDD1V8_GPIO, 0},
{CAM3_VDDIO1V8_GPIO, 0},
{AVDD_DSI_CSI_ENB_GPIO, 0},
};

static struct pca953x_platform_data gallo_tca6416_data = {
	.gpio_base      = TEGRA_NR_GPIOS + 4,/* already requested by tps6586x */
};

static const struct i2c_board_info gallo_i2c3_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &gallo_tca6416_data,
	},
};

static struct board_version b_version;

#ifdef CONFIG_VIDEO_YUV
static int yuv_sensor_power_on(void)
{
	return 0;
}

static int yuv_sensor_power_off(void)
{
	return 0;
}

struct yuv_sensor_platform_data yuv_sensor_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};
#endif /* CONFIG_VIDEO_YUV */

#ifdef CONFIG_VIDEO_MT9D115
static int mt9d115_power_on(void)
{
	int ret, i;

	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 1);

	for (i = 0; i < ARRAY_SIZE(gallo_camera_gpio_keys); i++) {
		ret = gpio_request(gallo_camera_gpio_keys[i].gpio,
			__func__);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		gpio_direction_output(gallo_camera_gpio_keys[i].gpio,
			gallo_camera_gpio_keys[i].value);
	}
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);

	gpio_direction_output(CAM3_RST_L_GPIO, 1);
	msleep(20);

	gpio_direction_output(CAM3_VDDIO1V8_GPIO, 1);
	msleep(20);

	gpio_direction_output(CAM3_VDD1V8_GPIO, 1);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	msleep(20);

	gpio_direction_output(CAM3_RST_L_GPIO, 0);
	msleep(20);
	gpio_direction_output(CAM3_RST_L_GPIO, 1);
	msleep(20);

	gpio_direction_output(CAM3_LDO_SHUTDN_L_GPIO, 1);
	msleep(20);

	return 0;
fail:
	while (i--)
		gpio_free(gallo_camera_gpio_keys[i].gpio);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);
	return ret;
}

static int mt9d115_power_off(void)
{
	int i;

	gpio_direction_output(CAM3_RST_L_GPIO, 0);
	msleep(20);
	gpio_direction_output(CAM3_LDO_SHUTDN_L_GPIO, 0);
	gpio_direction_output(CAM3_VDD1V8_GPIO, 0);
	gpio_direction_output(CAM3_VDDIO1V8_GPIO, 0);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 0);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);

	for (i = 0; i < ARRAY_SIZE(gallo_camera_gpio_keys); i++)
		gpio_free(gallo_camera_gpio_keys[i].gpio);

	return 0;
}

struct mt9d115_platform_data mt9d115_data = {
	.power_on = mt9d115_power_on,
	.power_off = mt9d115_power_off,
};
#endif /* CONFIG_VIDEO_MT9D115 */

#ifdef CONFIG_VIDEO_MT9P111
static int mt9p111_power_on(void)
{
	int ret, i;

	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 1);

	for (i = 0; i < ARRAY_SIZE(gallo_camera_gpio_keys); i++) {
		ret = gpio_request(gallo_camera_gpio_keys[i].gpio,
			__func__);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		gpio_direction_output(gallo_camera_gpio_keys[i].gpio,
			gallo_camera_gpio_keys[i].value);
	}

	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);

	gpio_direction_output(CAM2_PWR_DN_GPIO, 0);
	msleep(20);

	gpio_direction_output(CAM2_LDO_SHUTDN_L_GPIO, 1);
	msleep(20);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	msleep(20);

	gpio_direction_output(CAM2_RST_L_GPIO, 1);

	return 0;
fail:
	while (i--)
		gpio_free(gallo_camera_gpio_keys[i].gpio);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);
	return ret;
}

static int mt9p111_power_off(void)
{
	int i;

	gpio_direction_output(CAM2_RST_L_GPIO, 0);
	msleep(20);
	gpio_direction_output(CAM2_LDO_SHUTDN_L_GPIO, 0);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 0);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);

	for (i = 0; i < ARRAY_SIZE(gallo_camera_gpio_keys); i++)
		gpio_free(gallo_camera_gpio_keys[i].gpio);

	return 0;
}

static int mt9p111_get_board_info(void)
{
	pr_info("%s board_info %x", __func__, b_version.hw_version);
	return b_version.hw_version;
}

static int mt9p111_set_led_level(int level)
{
	int level_l;
	level_l = level & 0x01;
	gpio_direction_output(TEGRA_GPIO_PD2, level_l);
	return 0;
}

struct mt9p111_platform_data mt9p111_data = {
	.power_on = mt9p111_power_on,
	.power_off = mt9p111_power_off,
	.get_board_info = mt9p111_get_board_info,
	.set_led_level = mt9p111_set_led_level,
};
#endif /* CONFIG_VIDEO_MT9P111 */

extern void tegra_throttling_enable(bool enable);

#ifdef CONFIG_INPUT_KXTF9
/* KIONIX KXTF9 Digital Tri-axis Accelerometer */
#include <linux/kxtf9.h>

struct kxtf9_platform_data kxtf9_data = {
	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= KXTF9_G_2G,
	.shift_adj	= SHIFT_ADJ_2G,

	.axis_map_x	= 1,
	.axis_map_y	= 0,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 1,

	.data_odr_init		= ODR800F,
	.ctrl_reg1_init		= KXTF9_G_2G | RES_12BIT | TDTE | WUFE | TPE,
	//.ctrl_reg1_init		= KXTF9_G_2G | RES_12BIT,
	//.int_ctrl_init		= KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init		= KXTF9_IEN | KXTF9_IEL,
	.tilt_timer_init	= 0x03,
	/*.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x16,
	.wuf_thresh_init	= 0x28,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xFF,
	.tdt_l_thresh_init	= 0x14,
	.tdt_tap_timer_init	= 0x53,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x10,
	.tdt_window_timer_init	= 0xA0,*/

	.gpio = KXTF9_IRQ_GPIO,
};
#endif /* CONFIG_INPUT_KXTF9 */

static int gallo_camera_init(void)
{
	int ret;
	tegra_gpio_enable(CAM_GPIO_RESET_L_GPIO);
	ret = gpio_request(CAM_GPIO_RESET_L_GPIO,
		"CAM_GPIO_RESET_L_GPIO");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio #%d\n",
			__func__, CAM_GPIO_RESET_L_GPIO);
	}
	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 0);

	tegra_gpio_enable(TEGRA_GPIO_PD2);
	ret = gpio_request(TEGRA_GPIO_PD2,
		"TEGRA_GPIO_PD2");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio #%d\n",
			__func__, TEGRA_GPIO_PD2);
	}
	gpio_direction_output(TEGRA_GPIO_PD2, 0);
	gpio_export(TEGRA_GPIO_PD2, false);

	tegra_gpio_enable(TEGRA_GPIO_PBB5);
	ret = gpio_request(TEGRA_GPIO_PBB5,
		"TEGRA_GPIO_PBB5");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio #%d\n",
			__func__, TEGRA_GPIO_PBB5);
	}
	gpio_direction_output(TEGRA_GPIO_PBB5, 0);
	gpio_export(TEGRA_GPIO_PBB5, false);

	return 0;
}

static int gallo_ls_power_on(void)
{
	gpio_direction_output(ISL29018_POWER_EN, 1);
	return 0;
}

static int gallo_ls_power_off(void)
{
	gpio_direction_output(ISL29018_POWER_EN, 0);
	return 0;
}

struct lightsensor_platform_data isl29018_data = {
	.power_on = gallo_ls_power_on,
	.power_off = gallo_ls_power_off,
};

static void gallo_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);

	tegra_gpio_enable(ISL29018_POWER_EN);
	gpio_request(ISL29018_POWER_EN, "LS_PWR_EN");
	gpio_direction_output(ISL29018_POWER_EN, 1);
	gpio_export(ISL29018_POWER_EN, false);
}

static void gallo_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}


static struct i2c_board_info gallo_lds6202_i2c[] = {
	{
		I2C_BOARD_INFO("lds6202", 0x2C),
		.irq = NULL,
	},
};

static void gallo_lds6202_init(void)
{
	if (b_version.hw_version >= BOARD_VERSION_DVT) {
		int LDS6202_IRQ = 0;
		if(b_version.hw_version > BOARD_VERSION_DVT2)
			LDS6202_IRQ = LDS6202_IRQ_DVT3;
		else
			LDS6202_IRQ = LDS6202_IRQ_DVT2;
		gallo_lds6202_i2c->irq = TEGRA_GPIO_TO_IRQ(LDS6202_IRQ);

		tegra_gpio_enable(LDS6202_IRQ);
		gpio_request(LDS6202_IRQ, "cap_int");
		gpio_direction_input(LDS6202_IRQ);

		tegra_gpio_enable(LDS6202_RST);
		gpio_request(LDS6202_RST, "cap_rst");
		gpio_direction_output(LDS6202_RST, 1);
		gpio_export(LDS6202_RST, true);

		i2c_register_board_info(4, gallo_lds6202_i2c,
					ARRAY_SIZE(gallo_lds6202_i2c));
	} else
		printk(KERN_ERR "LDS6202 cap sensor only apply to DVT or later.");
}

#ifdef CONFIG_INPUT_KXTF9
static void gallo_kxtf9_init(void)
{
	tegra_gpio_enable(KXTF9_IRQ_GPIO);
	gpio_request(KXTF9_IRQ_GPIO, "kionix_irq");
	gpio_direction_input(KXTF9_IRQ_GPIO);
}
#endif /* CONFIG_INPUT_KXTF9 */

static void gallo_bq20z45_init(void)
{
	int ret = 0;

	tegra_gpio_enable(BQ24617_AC_PRESENT_GPIO);
	tegra_gpio_enable(BQ24617_NCE);
	tegra_gpio_enable(BQ24617_STAT1);
	tegra_gpio_enable(BQ24617_STAT2);
	tegra_gpio_enable(HW_LOW_BATTERY);

	ret |= gpio_request(BQ24617_AC_PRESENT_GPIO, "ac_present");
	ret |= gpio_request(BQ24617_NCE, "chg_disable");
	ret |= gpio_request(BQ24617_STAT1, "charge_stat1");
	ret |= gpio_request(BQ24617_STAT2, "charge_stat2");
	ret |= gpio_request(HW_LOW_BATTERY, "low_battery");

	if(ret) {
		printk("%s:gpio request error-assert\n", __FUNCTION__);
		BUG_ON(ret);
	}

	gpio_direction_input(BQ24617_AC_PRESENT_GPIO);
	gpio_direction_output(BQ24617_NCE, 0);
	gpio_direction_input(BQ24617_STAT1);
	gpio_direction_input(BQ24617_STAT2);
	gpio_direction_input(HW_LOW_BATTERY);

	return;
}

static void gallo_cg7216am_gpio_init(void)
{
	/*
		Before GALLO_DVT, the CHG LED and EOC LED are controlled by HW.
		After GALLO_DVT, that are controlled by CPU/MCU.
	*/

	tegra_gpio_enable(SYS_SUS);
	gpio_request(SYS_SUS, "SYS_SUS");
	if (b_version.hw_version < BOARD_VERSION_DVT) {
		gpio_direction_output(SYS_SUS, 0);
	} else {
		gpio_direction_output(SYS_SUS, 1);
	}
	printk(KERN_INFO "%s: set SYS_SUS to %d ", __func__, gpio_get_value(SYS_SUS));

	if (b_version.hw_version >= BOARD_VERSION_DVT3) {
		tegra_gpio_enable(MCU_RESET_DVT3);
		gpio_request(MCU_RESET_DVT3, "MCU_RESET");
		gpio_direction_output(MCU_RESET_DVT3, 0);
	} else if (b_version.hw_version >= BOARD_VERSION_DVT2) {
		tegra_gpio_enable(MCU_RESET_DVT2);
		gpio_request(MCU_RESET_DVT2, "MCU_RESET");
		gpio_direction_output(MCU_RESET_DVT2, 0);
	} else {
		/* do nothing */
	}

	if (b_version.hw_version >= BOARD_VERSION_DVT2) {
		tegra_gpio_enable(MCU_WAKE);
		gpio_request(MCU_WAKE, "MCU_WAKE");
		gpio_direction_output(MCU_WAKE, 0);
	}

	tegra_gpio_enable(MCU_INTERRUPT);
	gpio_request(MCU_INTERRUPT, "MCU_INTERRUPT");
	gpio_direction_input(MCU_INTERRUPT);
}

static void gallo_nct1008_init(void)
{
	/* hardware version > EVT1 */
	if (b_version.hw_version > BOARD_VERSION_EVT1) {
		tegra_gpio_enable(NCT1008_THERM2_GPIO_EVT1_2);
		gpio_request(NCT1008_THERM2_GPIO_EVT1_2, "temp_alert");
		gpio_direction_input(NCT1008_THERM2_GPIO_EVT1_2);

		tegra_gpio_enable(NCT1008_THERM_GPIO);
		gpio_request(NCT1008_THERM_GPIO, "temp_therm");
		gpio_direction_input(NCT1008_THERM_GPIO);
	} else {
		tegra_gpio_enable(NCT1008_THERM2_GPIO_EVT1);
		gpio_request(NCT1008_THERM2_GPIO_EVT1, "temp_alert");
		gpio_direction_input(NCT1008_THERM2_GPIO_EVT1);
	}
}

static struct nct1008_platform_data gallo_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x04,
	.offset = 0,
	.hysteresis = 5,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.therm_irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM_GPIO),
	.alarm_fn = tegra_throttling_enable,
};

#define DEFINE_LED_ENTRY(index, led_name)			\
	{							\
		.led_id = index,				\
		.led_dev =					\
		{						\
			.name		= led_name,		\
			.brightness	= LED_OFF,		\
			.max_brightness = 1,			\
		},						\
	}

static struct cg7216am_led_device cg7216am_led_devs[] = {
	DEFINE_LED_ENTRY(GREEN, "green"),
	DEFINE_LED_ENTRY(WHITE, "white"),
	DEFINE_LED_ENTRY(AMBER, "amber"),
};

static struct cg7216am_led_platform_data cg7216am_led_platform_data = {
	.num_led	= ARRAY_SIZE(cg7216am_led_devs),
	.led_devs	= cg7216am_led_devs,
};

static struct cg7216am_subdev_info cg7216am_sub_devs[] = {
	{
		.id	= 0,
		.name	= "cg7216am-leds",
		.platform_data = &cg7216am_led_platform_data,
	},
	{
		.id	= 1,
		.name	= "cg7216am-battery",
	},
};

struct cg7216am_platform_data gallo_cg7216am_platform_data = {
	.hw_version = -1,
	.is_fw_update_mode = 0,
	.gpio_sys_sus		= SYS_SUS,
	.gpio_polarity_sys_sus = 1,
	.gpio_mcu_int	= MCU_INTERRUPT,
	.gpio_mcu_rst = -1,
	.num_subdevs		= ARRAY_SIZE(cg7216am_sub_devs),
	.sub_devs		= cg7216am_sub_devs,
};

struct cg7216am_platform_data gallo_cg7216am_ota_platform_data = {
	.hw_version = -1,
	.is_fw_update_mode = 0,
	.gpio_sys_sus		= SYS_SUS,
	.gpio_polarity_sys_sus = 1,
	.gpio_mcu_int	= MCU_INTERRUPT,
	.gpio_mcu_rst = -1,
	.num_subdevs		= ARRAY_SIZE(cg7216am_sub_devs),
	.sub_devs		= cg7216am_sub_devs,
};

const struct i2c_board_info gallo_cg7216am_i2c[] = {
	{
		I2C_BOARD_INFO("cg7216am", 0x10),
		.platform_data	= &gallo_cg7216am_platform_data,
	},
};

/* cg7216am use 0x60 slave address in OTA mode */
/* Slave Address in DVT: 0x60 */
/* Slave Address in DVT2: 0x77 */
static const struct i2c_board_info gallo_cg7216am_i2c_ota[] = {
	{
		I2C_BOARD_INFO("cg7216am", 0x60),
		.platform_data = &gallo_cg7216am_ota_platform_data,
	},
};

static const struct i2c_board_info gallo_cg7216am_i2c_ota_DVT2[] = {
	{
		I2C_BOARD_INFO("cg7216am", 0x77),
		.platform_data = &gallo_cg7216am_ota_platform_data,
	},
};

static const struct i2c_board_info gallo_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(ISL29018_IRQ_GPIO),
		.platform_data = &isl29018_data,
	},
	{
		I2C_BOARD_INFO("isa1200", 0x48),
	},
};

static int battery_get_ac_status(void)
{
	return gpio_get_value(BQ24617_AC_PRESENT_GPIO);
}

static int bq24617_get_charger_status(void)
{
	int stat1 = gpio_get_value(BQ24617_STAT1);
	int stat2 = gpio_get_value(BQ24617_STAT2);

	if (stat1 == 0) {
		if (stat2 == 0) {
			return CHARGING_SUSPEND;
		} else {
			return CHARGING_COMPLETE;
		}
	} else {
		if (stat2 == 0) {
			return CHARGING_IN_PROGRESS;
		} else {
			return CHARGING_NOT_PRESENT;
		}
	}
}

static struct BQ20Z45_power_pdata bq20z45_charger_platform = {
	.is_ac_online = battery_get_ac_status,
	.get_charger_status = bq24617_get_charger_status,
	.ac_present_gpio = BQ24617_AC_PRESENT_GPIO,
	.nce_gpio = BQ24617_NCE,
	.lowbatt_gpio = HW_LOW_BATTERY,
	.stat1_gpio = BQ24617_STAT1,
	.stat2_gpio = BQ24617_STAT2,
	.interval_slow = BATTERY_POLL_PERIOD,
	.interval_fast = BATTERY_STATUS_LATENCY,
};

static struct BQ20Z45_power_pdata bq30423_charger_platform = {
	.is_ac_online = battery_get_ac_status,
	.get_charger_status = bq24617_get_charger_status,
	.ac_present_gpio = BQ24617_AC_PRESENT_GPIO,
	.nce_gpio = -1,
	.stat1_gpio = -1,
	.stat2_gpio = -1,
	.interval_slow = BATTERY_POLL_PERIOD,
	.interval_fast = BATTERY_STATUS_LATENCY,
};

static const struct i2c_board_info gallo_EVT0_battery_info[] = {
	{
		I2C_BOARD_INFO("bq20z45-battery", 0x0B),
		.irq = TEGRA_GPIO_TO_IRQ(HW_LOW_BATTERY),
		.platform_data = &bq20z45_charger_platform,
	},
};

static const struct i2c_board_info gallo_EVT1_battery_info[] = {
	{
		I2C_BOARD_INFO("bq30423-battery", 0x0B),
		.irq = TEGRA_GPIO_TO_IRQ(HW_LOW_BATTERY),
		.platform_data = &bq30423_charger_platform,
	},
};

extern struct smart_battery_charging_impl gallo_sb_impl;
static void gallo_battery_charger_init(int is_recovery_mode)
{
	if ( b_version.hw_version == BOARD_VERSION_EVT0 ) {
		printk(KERN_INFO "%s. ,%d, EVT0\n", __func__, b_version.hw_version);

		i2c_register_board_info(2, gallo_EVT0_battery_info,
					ARRAY_SIZE(gallo_EVT0_battery_info));
	} else {
		printk(KERN_INFO "%s. ,%d, EVT??\n", __func__, b_version.hw_version);

		if (b_version.hw_version >= BOARD_VERSION_DVT3) {
			bq30423_charger_platform.sb_imp = &gallo_sb_impl;
		}

		i2c_register_board_info(2, gallo_EVT1_battery_info,
					ARRAY_SIZE(gallo_EVT1_battery_info));
	}
	if ( b_version.hw_version >= BOARD_VERSION_DVT2 ) {
		tegra_gpio_enable(AP_EOC);
		gpio_request(AP_EOC, "AP_EOC");
		gpio_direction_input(AP_EOC);

		tegra_gpio_enable(AP_CHG);
		gpio_request(AP_CHG, "AP_CHG");
		gpio_direction_input(AP_CHG);
	}
}

static struct pca954x_platform_mode gallo_pca9546_modes[] = {
	{ .adap_id = 6, }, /* REAR CAM1 */
	{ .adap_id = 7, }, /* REAR CAM2 */
	{ .adap_id = 8, }, /* FRONT CAM3 */
};

static struct pca954x_platform_data gallo_pca9546_data = {
	.modes	  = gallo_pca9546_modes,
	.num_modes      = ARRAY_SIZE(gallo_pca9546_modes),
};

static const struct i2c_board_info gallo_i2c3_board_info_pca9546[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &gallo_pca9546_data,
	},
};

static struct i2c_board_info gallo_i2c3_board_info[] = {
#ifdef CONFIG_VIDEO_YUV
	{
		I2C_BOARD_INFO(SENSOR_NAME, 0x3c),
		.platform_data = &yuv_sensor_data,
	},
#endif /* CONFIG_VIDEO_YUV */
#ifdef CONFIG_VIDEO_MT9D115
	{
		I2C_BOARD_INFO(MT9D115_NAME, 0x3c),
		.platform_data = &mt9d115_data,
	},
#endif /* CONFIG_VIDEO_MT9D115 */
#ifdef CONFIG_VIDEO_MT9P111
	{
		I2C_BOARD_INFO(MT9P111_NAME, 0x78),
		/* Use 0x78 instead of 0x78 >>1 to avoid conflict with 2M */
		.platform_data = &mt9p111_data,
	},
#endif /* CONFIG_VIDEO_MT9P111 */
};

static struct i2c_board_info gallo_dvt2_i2c3_board_info[] = {
#ifdef CONFIG_VIDEO_YUV
	{
		I2C_BOARD_INFO(SENSOR_NAME, 0x3c),
		.platform_data = &yuv_sensor_data,
	},
#endif /* CONFIG_VIDEO_YUV */
#ifdef CONFIG_VIDEO_MT9D115
	{
		I2C_BOARD_INFO(MT9D115_NAME, 0x3c),
		.platform_data = &mt9d115_data,
	},
#endif /* CONFIG_VIDEO_MT9D115 */
#ifdef CONFIG_VIDEO_MT9P111
	{
		I2C_BOARD_INFO(MT9P111_NAME, 0x7A),
		/* Use 0x7A instead of 0x7A >>1 to avoid conflict with 2M */
		.platform_data = &mt9p111_data,
	},
#endif /* CONFIG_VIDEO_MT9P111 */
};

static struct i2c_board_info gallo_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO_EVT1),
		.platform_data = &gallo_nct1008_pdata,
	},

#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
#endif
};

#ifdef CONFIG_MPU_SENSORS_MPU3050
#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data_EVT0 = {
	.int_config  = 0x90,
	.orientation = { 1, 0, 0, 0, -1, 0, 0, 0, -1 },  /* Orientation matrix for MPU on gallo */
	.level_shifter = 0,
	.accel = {
#ifdef CONFIG_MPU_SENSORS_KXTF9
	.get_slave_descr = get_accel_slave_descr,
	.irq = TEGRA_GPIO_TO_IRQ(KXTF9_IRQ_GPIO),
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 0,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x0F,
	.orientation = { 0, 1, 0, 1, 0, 0, 0, 0, -1 },  /* Orientation matrix for Kionix on gallo */
	},

	.compass = {
#ifdef CONFIG_MPU_SENSORS_AK8975
	.get_slave_descr = get_compass_slave_descr,
	/*.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),*/
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 4,            /* bus number 4 on gallo */
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x0C,
	.orientation = { 0, -1, 0, -1, 0, 0, 0, 0, -1 },  /* Orientation matrix for AKM on gallo */
	},
};

static struct mpu3050_platform_data mpu3050_data_EVT1 = {
	.int_config  = 0x90,
	.orientation = { 1, 0, 0, 0, -1, 0, 0, 0, -1 },  /* Orientation matrix for MPU on gallo */
	.level_shifter = 0,
	.accel = {
#ifdef CONFIG_MPU_SENSORS_KXTF9
	.get_slave_descr = get_accel_slave_descr,
	.irq = TEGRA_GPIO_TO_IRQ(KXTF9_IRQ_GPIO),
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 0,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x0F,
	.orientation = { -1, 0, 0, 0, 1, 0, 0, 0, -1 },  /* Orientation matrix for Kionix on gallo */
	},

	.compass = {
#ifdef CONFIG_MPU_SENSORS_AK8975
	.get_slave_descr = get_compass_slave_descr,
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 4,            /* bus number 4 on gallo */
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x0C,
	.orientation = { -1, 0, 0, 0, 1, 0, 0, 0, -1 },  /* Orientation matrix for AKM on gallo */
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(MPU3050_IRQ_GPIO),
		.platform_data = &mpu3050_data_EVT0,
	},
};

static void gallo_mpuirq_init(void)
{
	pr_info("*** MPU START *** gallo_mpuirq_init...\n");
	tegra_gpio_enable(MPU3050_IRQ_GPIO);
	gpio_request(MPU3050_IRQ_GPIO, SENSOR_MPU_NAME);
	gpio_direction_input(MPU3050_IRQ_GPIO);

	tegra_gpio_enable(KXTF9_IRQ_GPIO);
	gpio_request(KXTF9_IRQ_GPIO, "kxtf9");
	gpio_direction_input(KXTF9_IRQ_GPIO);
	pr_info("*** MPU END *** gallo_mpuirq_init...\n");

	if (b_version.hw_version >= BOARD_VERSION_DVT2) {
		tegra_gpio_enable(KXTF9_PWR_SHDN);
		gpio_request(KXTF9_PWR_SHDN, "kxtf9_PWR");
		gpio_direction_output(KXTF9_PWR_SHDN, 1);
	}
}
#endif

#ifdef CONFIG_INPUT_KXTF9
static struct i2c_board_info __initdata kxtf9_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		/*.irq = TEGRA_GPIO_TO_IRQ(KXTF9_IRQ_GPIO),*/
		.platform_data = &kxtf9_data,
	},
};
#endif /* CONFIG_INPUT_KXTF9 */

int __init gallo_sensors_init(int is_recovery_mode, bool is_wifi_sku)
{
	gallo_get_hw_info(&b_version);
	gallo_bq20z45_init();
	gallo_cg7216am_gpio_init();
	gallo_isl29018_init();
	if(!is_wifi_sku)
		gallo_lds6202_init();
	gallo_akm8975_init();
#ifdef CONFIG_INPUT_KXTF9
	gallo_kxtf9_init();
#endif /* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_MPU_SENSORS_MPU3050
	gallo_mpuirq_init();
#endif
	gallo_camera_init();
	gallo_nct1008_init();

#ifdef CONFIG_MPU_SENSORS_MPU3050

	if ( b_version.hw_version == BOARD_VERSION_EVT0 ) {
		mpu3050_i2c0_boardinfo[0].platform_data = &mpu3050_data_EVT0;
	}else if( b_version.hw_version == BOARD_VERSION_EVT1 ){
		mpu3050_i2c0_boardinfo[0].platform_data = &mpu3050_data_EVT1;
	}else{
		mpu3050_i2c0_boardinfo[0].platform_data = &mpu3050_data_EVT1;
	}

	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
		ARRAY_SIZE(mpu3050_i2c0_boardinfo));
#endif

	gallo_battery_charger_init(is_recovery_mode);

	i2c_register_board_info(0, gallo_i2c0_board_info,
		ARRAY_SIZE(gallo_i2c0_board_info));

	if (b_version.hw_version >= BOARD_VERSION_DVT3) {
		gallo_cg7216am_platform_data.gpio_mcu_rst = MCU_RESET_DVT3;
	} else if (b_version.hw_version >= BOARD_VERSION_DVT2) {
		gallo_cg7216am_platform_data.gpio_mcu_rst = MCU_RESET_DVT2;
	} else {
		gallo_cg7216am_platform_data.gpio_mcu_rst = -1;
	}

	if(is_recovery_mode) {
		gallo_cg7216am_ota_platform_data.is_fw_update_mode = 1;
		gallo_cg7216am_ota_platform_data.hw_version = b_version.hw_version;
               if (b_version.hw_version <= BOARD_VERSION_DVT) {
			i2c_register_board_info(0, gallo_cg7216am_i2c_ota,
				ARRAY_SIZE(gallo_cg7216am_i2c_ota));
               } else {
			i2c_register_board_info(0, gallo_cg7216am_i2c_ota_DVT2,
				ARRAY_SIZE(gallo_cg7216am_i2c_ota_DVT2));
               }
	}
	{
		if (b_version.hw_version < BOARD_VERSION_DVT) {
			gallo_cg7216am_platform_data.gpio_polarity_sys_sus = 1;
		} else {
			gallo_cg7216am_platform_data.gpio_polarity_sys_sus = 0;
		}

		i2c_register_board_info(0, gallo_cg7216am_i2c,
			ARRAY_SIZE(gallo_cg7216am_i2c));
	}

	if (b_version.hw_version >= BOARD_VERSION_DVT2) {
		i2c_register_board_info(3, gallo_dvt2_i2c3_board_info,
			ARRAY_SIZE(gallo_i2c3_board_info));
	} else {
		i2c_register_board_info(3, gallo_i2c3_board_info,
			ARRAY_SIZE(gallo_i2c3_board_info));
	}

	if (b_version.hw_version > BOARD_VERSION_EVT1)
		gallo_i2c4_board_info[0].irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO_EVT1_2);

	i2c_register_board_info(4, gallo_i2c4_board_info,
		ARRAY_SIZE(gallo_i2c4_board_info));


#ifdef CONFIG_INPUT_KXTF9
	i2c_kxtf9_setup();
#endif /* CONFIG_INPUT_KXTF9 */
	return 0;
}

int __init gallo_camera_late_init(void)
{
	int ret;
	struct regulator *cam_ldo6 = NULL;

	if (!machine_is_gallo())
		return 0;

	cam_ldo6 = regulator_get(NULL, "vdd_ldo6");
	if (IS_ERR_OR_NULL(cam_ldo6)) {
		pr_err("%s: Couldn't get regulator ldo6\n", __func__);
		return PTR_ERR(cam_ldo6);
	}

	ret = regulator_set_voltage(cam_ldo6, 1800*1000, 1800*1000);
	if (ret) {
		pr_err("%s: Failed to set ldo6 to 1.8v\n", __func__);
		goto fail;
	}

	ret = regulator_enable(cam_ldo6);
	if (ret) {
		pr_err("%s: Failed to enable ldo6\n", __func__);
		goto fail;
	}

	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 1);
	i2c_new_device(i2c_get_adapter(3), gallo_i2c3_board_info_tca6416);
	gpio_direction_output(CAM_GPIO_RESET_L_GPIO, 0);

	ret = regulator_disable(cam_ldo6);
	if (ret) {
		pr_err("%s: Failed to disable ldo6\n", __func__);
		goto fail;
	}

fail:
	regulator_put(cam_ldo6);
	return ret;
}

late_initcall(gallo_camera_late_init);
