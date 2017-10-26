/*
 * arch/arm/mach-tegra/board-gallo-panel.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <mach/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/tegra_cpufreq.h>

#include "devices.h"
#include "gpio-names.h"
#include "board.h"

#define gallo_pnl_pwr_enb	TEGRA_GPIO_PC6
#define gallo_bl_enb		TEGRA_GPIO_PD4
#define gallo_lvds_shutdown	TEGRA_GPIO_PB2
#define gallo_en_vdd_bl	TEGRA_GPIO_PW0
#define gallo_hdmi_hpd	TEGRA_GPIO_PN7
#define gallo_hdmi_enb	TEGRA_GPIO_PV5

/*panel power on sequence timing*/
#define gallo_pnl_to_lvds_ms	0
#define gallo_lvds_to_bl_ms		200

static struct regulator *gallo_hdmi_reg = NULL;
static struct regulator *gallo_hdmi_pll = NULL;
static bool backlight_on = true;


static int gallo_backlight_init(struct device *dev) {
	int ret;

	ret = gpio_request(gallo_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(gallo_bl_enb, 1);
	if (ret < 0)
		gpio_free(gallo_bl_enb);
	else
		tegra_gpio_enable(gallo_bl_enb);

	return ret;
};

static void gallo_backlight_exit(struct device *dev) {
	gpio_set_value(gallo_bl_enb, 0);
	gpio_free(gallo_bl_enb);
	tegra_gpio_disable(gallo_bl_enb);
}

static int gallo_backlight_notify(struct device *unused, int brightness)
{
	if (brightness && !backlight_on)
	{
		mdelay(150);
		backlight_on = true;
		pr_info("%s backlight_on=%d\n", __func__, backlight_on);
	}
	gpio_set_value(gallo_bl_enb, !!brightness);
	if (!brightness && backlight_on)
	{
		mdelay(100);
		backlight_on = false;
		pr_info("%s backlight_on=%d\n", __func__, backlight_on);
	}
	
	return brightness;
}

static int gallo_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data gallo_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 5000000,
	.init		= gallo_backlight_init,
	.exit		= gallo_backlight_exit,
	.notify		= gallo_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb   = gallo_disp1_check_fb,
};

static struct platform_device gallo_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &gallo_backlight_data,
	},
};

static int gallo_panel_enable(void)
{
	struct regulator *reg = regulator_get(NULL, "vdd_ldo4");

	regulator_enable(reg);
	regulator_put(reg);

	gpio_set_value(gallo_pnl_pwr_enb, 1);
	mdelay(gallo_pnl_to_lvds_ms);
	gpio_set_value(gallo_lvds_shutdown, 1);
	mdelay(gallo_lvds_to_bl_ms);
	gpio_set_value(gallo_en_vdd_bl, 1);
	return 0;
}

static int gallo_panel_disable(void)
{
	gpio_set_value(gallo_en_vdd_bl, 0);
	mdelay(100);
	gpio_set_value(gallo_lvds_shutdown, 0);
	mdelay(5);
	gpio_set_value(gallo_pnl_pwr_enb, 0);
	return 0;
}

static int gallo_hdmi_enable(void)
{
	if (!gallo_hdmi_reg) {
		gallo_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LD07 */
		if (IS_ERR_OR_NULL(gallo_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			gallo_hdmi_reg = NULL;
			return PTR_ERR(gallo_hdmi_reg);
		}
	}
	regulator_enable(gallo_hdmi_reg);

	if (!gallo_hdmi_pll) {
		gallo_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD08 */
		if (IS_ERR_OR_NULL(gallo_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			gallo_hdmi_pll = NULL;
			regulator_disable(gallo_hdmi_reg);
			gallo_hdmi_reg = NULL;
			return PTR_ERR(gallo_hdmi_pll);
		}
	}
	regulator_enable(gallo_hdmi_pll);
	return 0;
}

static int gallo_hdmi_disable(void)
{
	regulator_disable(gallo_hdmi_reg);
	regulator_disable(gallo_hdmi_pll);
	return 0;
}

static struct resource gallo_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource gallo_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode gallo_panel_modes[] = {
	{
		.pclk = 72072000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 58,
		.v_sync_width = 4,
		.h_back_porch = 58,
		.v_back_porch = 4,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 58,
		.v_front_porch = 4,
	},
};

static struct tegra_fb_data gallo_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 800,
	.bits_per_pixel	= 32,
};

static struct tegra_fb_data gallo_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1366,
	.yres		= 768,
	.bits_per_pixel	= 32,
};

static struct tegra_dc_out gallo_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.depth		= 18,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes	 	= gallo_panel_modes,
	.n_modes 	= ARRAY_SIZE(gallo_panel_modes),

	.enable		= gallo_panel_enable,
	.disable	= gallo_panel_disable,
};

static struct tegra_dc_out gallo_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= gallo_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= gallo_hdmi_enable,
	.disable	= gallo_hdmi_disable,
};

static struct tegra_dc_platform_data gallo_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &gallo_disp1_out,
	.fb		= &gallo_fb_data,
};

static struct tegra_dc_platform_data gallo_disp2_pdata = {
	.flags		= 0,
	.default_out	= &gallo_disp2_out,
	.fb		= &gallo_hdmi_fb_data,
};

static struct nvhost_device gallo_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= gallo_disp1_resources,
	.num_resources	= ARRAY_SIZE(gallo_disp1_resources),
	.dev = {
		.platform_data = &gallo_disp1_pdata,
	},
};

static int gallo_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &gallo_disp1_device.dev;
}

static struct nvhost_device gallo_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= gallo_disp2_resources,
	.num_resources	= ARRAY_SIZE(gallo_disp2_resources),
	.dev = {
		.platform_data = &gallo_disp2_pdata,
	},
};

static struct nvmap_platform_carveout gallo_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE,
		.size		= TEGRA_IRAM_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data gallo_nvmap_data = {
	.carveouts	= gallo_carveouts,
	.nr_carveouts	= ARRAY_SIZE(gallo_carveouts),
};

static struct platform_device gallo_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &gallo_nvmap_data,
	},
};

static struct platform_device *gallo_gfx_devices[] __initdata = {
	&gallo_nvmap_device,
	&tegra_grhost_device,
	&tegra_pwfm2_device,
	&gallo_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend gallo_panel_early_suspender;

static void gallo_panel_early_suspend(struct early_suspend *h)
{
	/*unsigned i;
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_POWERDOWN);*/
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);	
#ifdef CONFIG_CPU_FREQ
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
#endif
}

static void gallo_panel_late_resume(struct early_suspend *h)
{
	/*unsigned i;
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);*/
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_UNBLANK);
#ifdef CONFIG_CPU_FREQ
	cpufreq_restore_default_governor();
#endif
}
#endif

int __init gallo_panel_init(void)
{
	int err;
	struct resource *res;

	gpio_request(gallo_pnl_pwr_enb, "pnl_pwr_enb");
	gpio_direction_output(gallo_pnl_pwr_enb, 1);
	tegra_gpio_enable(gallo_pnl_pwr_enb);

	gpio_request(gallo_lvds_shutdown, "lvds_shdn");
	gpio_direction_output(gallo_lvds_shutdown, 1);
	tegra_gpio_enable(gallo_lvds_shutdown);

	gpio_request(gallo_en_vdd_bl, "en_vdd_bl");
	gpio_direction_output(gallo_en_vdd_bl, 1);
	tegra_gpio_enable(gallo_en_vdd_bl);

	tegra_gpio_enable(gallo_hdmi_enb);
	gpio_request(gallo_hdmi_enb, "hdmi_5v_en");
	gpio_direction_output(gallo_hdmi_enb, 1);

	tegra_gpio_enable(gallo_hdmi_hpd);
	gpio_request(gallo_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(gallo_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	gallo_panel_early_suspender.suspend = gallo_panel_early_suspend;
	gallo_panel_early_suspender.resume = gallo_panel_late_resume;
	gallo_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&gallo_panel_early_suspender);
#endif

	gallo_carveouts[1].base = tegra_carveout_start;
	gallo_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(gallo_gfx_devices,
				   ARRAY_SIZE(gallo_gfx_devices));


	res = nvhost_get_resource_byname(&gallo_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&gallo_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	if (!err)
		err = nvhost_device_register(&gallo_disp1_device);

	if (!err)
		err = nvhost_device_register(&gallo_disp2_device);

	return err;
}

