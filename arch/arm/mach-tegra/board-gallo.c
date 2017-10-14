/*
 * arch/arm/mach-tegra/board-gallo.c
 *
 * Copyright (c) 2010 - 2011, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>
#include <linux/mfd/tps6586x.h>
#include <linux/memblock.h>
#include <linux/leds.h>
#include <sound/fm34.h>

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#include <linux/i2c/atmel_maxtouch.h>
#endif
#include <linux/suspend.h>
#include <sound/wm8903.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/spdif.h>
#include <mach/audio.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/tegra_das.h>

#include "board.h"
#include "clock.h"
#include "board-gallo.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t2.h"
#include <linux/i2c/cg7216am.h>

extern struct cg7216am_platform_data gallo_cg7216am_platform_data;
extern const struct i2c_board_info gallo_cg7216am_i2c[];

#ifdef CONFIG_EEPROM_NV_ITEM
	#include <linux/i2c/eeprom_nv_item_common.h>
	#include <linux/i2c/eeprom_nv_item.h>
#endif

static struct board_version b_version;

static struct board_version hw_info = {
	.pj_id = -1,
	.hw_version = -1,
};

#define DAP2_FS		TEGRA_GPIO_PA2
#define DAP2_DIN	TEGRA_GPIO_PA4
#define DAP2_DOUT	TEGRA_GPIO_PA5
#define SPI2_MISO	TEGRA_GPIO_PX1
#define SPI2_CS0_N	TEGRA_GPIO_PX3
#define DEBUG_SWITCH	TEGRA_GPIO_PK5
#define DOCKING_INT	TEGRA_GPIO_PS5
#define USB_ULPI_OC	TEGRA_GPIO_PP2

static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "NVIDIA",
	.product = "Tegra 2",
	.nluns = 1,
};

static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTD_BASE),
		.mapbase	= TEGRA_UARTD_BASE,
		.irq		= INT_UARTD,
		.flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type           = PORT_TEGRA,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

static struct tegra_audio_platform_data tegra_spdif_pdata = {
	.dma_on = true,  /* use dma by default */
	.spdif_clk_rate = 5644800,
};

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "clk_dev2",
	.inf_type = TEGRA_USB_LINK_ULPI,
};

#ifdef CONFIG_BCM4329_RFKILL

static struct resource gallo_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4329_nreset_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device gallo_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(gallo_bcm4329_rfkill_resources),
	.resource       = gallo_bcm4329_rfkill_resources,
};

static noinline void __init gallo_bt_rfkill(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", gallo_bcm4329_rfkill_device.name, \
				"blink", NULL);

	platform_device_register(&gallo_bcm4329_rfkill_device);

	return;
}
#else
static inline void gallo_bt_rfkill(void) { }
#endif

#ifdef CONFIG_BT_BLUESLEEP
static noinline void __init tegra_setup_bluesleep(void)
{
	struct platform_device *pdev = NULL;
	struct resource *res;

	pdev = platform_device_alloc("bluesleep", 0);
	if (!pdev) {
		pr_err("unable to allocate platform device for bluesleep");
		return;
	}

	res = kzalloc(sizeof(struct resource) * 3, GFP_KERNEL);
	if (!res) {
		pr_err("unable to allocate resource for bluesleep\n");
		goto err_free_dev;
	}

	res[0].name   = "gpio_host_wake";
	res[0].start  = TEGRA_GPIO_PU6;
	res[0].end    = TEGRA_GPIO_PU6;
	res[0].flags  = IORESOURCE_IO;

	res[1].name   = "gpio_ext_wake";
	res[1].start  = TEGRA_GPIO_PU1;
	res[1].end    = TEGRA_GPIO_PU1;
	res[1].flags  = IORESOURCE_IO;

	res[2].name   = "host_wake";
	res[2].start  = gpio_to_irq(TEGRA_GPIO_PU6);
	res[2].end    = gpio_to_irq(TEGRA_GPIO_PU6);
	res[2].flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE;

	if (platform_device_add_resources(pdev, res, 3)) {
		pr_err("unable to add resources to bluesleep device\n");
		goto err_free_res;
	}

	if (platform_device_add(pdev)) {
		pr_err("unable to add bluesleep device\n");
		goto err_free_res;
	}

	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);

	return;

err_free_res:
	kfree(res);
err_free_dev:
	platform_device_put(pdev);
	return;
}
#else
static inline void tegra_setup_bluesleep(void) { }
#endif

static __initdata struct tegra_clk_init_table gallo_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uartd",	"pll_p",	216000000,	true},
	{ "uartc",	"pll_m",	600000000,	false},
	{ "blink",	"clk_32k",	32768,		false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "pwm",	"clk_m",	26000000,		false},
	{ "pll_a",	NULL,		56448000,	false},
	{ "pll_a_out0",	NULL,		11289600,	false},
	{ "clk_dev1",	"pll_a_out0",	0,		true},
	{ "i2s1",	"pll_a_out0",	11289600,	false},
	{ "i2s2",	"pll_a_out0",	11289600,	false},
	{ "audio",	"pll_a_out0",	11289600,	false},
	{ "audio_2x",	"audio",	22579200,	false},
	{ "spdif_out",	"pll_a_out0",	5644800,	false},
	{ "kbc",	"clk_32k",	32768,		true},
	{ NULL,		NULL,		0,		0},
};

#define USB_MANUFACTURER_NAME		"Dell Inc."
#define USB_PRODUCT_NAME		"Streak 10 Pro"
#define USB_PRODUCT_ID_MTP_ADB		0xB105
#define USB_PRODUCT_ID_MTP		0xB106
#define USB_PRODUCT_ID_RNDIS		0xB107
#define USB_VENDOR_ID			0x413C

static char *isn_number;
static bool is_wifi_sku = false;
static char *rck = 0;
static char *is_debug_mode;

static char *usb_functions_mtp_ums[] = { "mtp" };
static char *usb_functions_mtp_adb_ums[] = { "mtp", "adb" };
#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
static char *usb_functions_rndis[] = { "rndis" };
static char *usb_functions_rndis_adb[] = { "rndis", "adb" };
#endif
static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"mtp",
	"adb",
};

void gallo_i2c_bus_reset_devices_1(struct tegra_i2c_platform_data *pd, u16 client_addr);

static struct android_usb_product usb_products[] = {
	{
		.product_id     = USB_PRODUCT_ID_MTP,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_ums),
		.functions      = usb_functions_mtp_ums,
	},
	{
		.product_id     = USB_PRODUCT_ID_MTP_ADB,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb_ums),
		.functions      = usb_functions_mtp_adb_ums,
	},
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		.vendor_id      = USB_ACCESSORY_VENDOR_ID,
		.product_id     = USB_ACCESSORY_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory),
		.functions      = usb_functions_accessory,
	},
	{
		.vendor_id      = USB_ACCESSORY_VENDOR_ID,
		.product_id     = USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory_adb),
		.functions      = usb_functions_accessory_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id     = USB_PRODUCT_ID_RNDIS,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = USB_PRODUCT_ID_RNDIS,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
#endif
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id              = USB_VENDOR_ID,
	.product_id             = USB_PRODUCT_ID_MTP_ADB,
	.manufacturer_name      = USB_MANUFACTURER_NAME,
	.product_name           = USB_PRODUCT_NAME,
	.serial_number          = NULL,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data  = &andusb_plat,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	.ethaddr = {0, 0, 0, 0, 0, 0},
	.vendorID = USB_VENDOR_ID,
	.vendorDescr = USB_MANUFACTURER_NAME,
};

static struct platform_device rndis_device = {
	.name   = "rndis",
	.id     = -1,
	.dev    = {
		.platform_data  = &rndis_pdata,
	},
};
#endif

unsigned char	fm34_cmd_buf[][9] = {
	//patch
	{6, 0xFC, 0xF3, 0x6A, 0x2A, 0x00, 0x10, 0x00, 0x00},
	{6, 0xFC, 0xF3, 0x6A, 0x2A, 0x00, 0x20, 0x00, 0x00},
	{5, 0xFC, 0xF3, 0x68, 0x64, 0x04, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x51, 0x80, 0x00, 0xAC},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x52, 0x26, 0x7C, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x53, 0x34, 0x00, 0x0E},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x54, 0x19, 0x1D, 0x82},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x55, 0x19, 0x1C, 0xEF},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x56, 0x80, 0x4F, 0x6A},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x57, 0x26, 0x7A, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x58, 0x34, 0x00, 0x0E},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x59, 0x18, 0x2F, 0x80},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x5A, 0x0D, 0x02, 0x2A},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x5B, 0x18, 0x2B, 0xDF},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x5C, 0x95, 0x62, 0x06},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x5D, 0x95, 0x61, 0x46},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x5E, 0x40, 0xE5, 0xBA},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x5F, 0x83, 0xFD, 0x44},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x60, 0x26, 0xE2, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x61, 0x19, 0x06, 0x70},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x62, 0x93, 0xFD, 0x4A},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x63, 0x83, 0xFD, 0x5A},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x64, 0x23, 0xA2, 0x1F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x65, 0x93, 0xFD, 0x5A},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x66, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x67, 0x04, 0x00, 0x60},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x68, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x69, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x6A, 0x04, 0x00, 0x60},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x6B, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x6C, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x6D, 0x80, 0x7C, 0x31},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x6E, 0x40, 0x07, 0xE4},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x6F, 0x22, 0xE1, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x70, 0x19, 0x07, 0xA5},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x71, 0x22, 0x79, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x72, 0x19, 0x07, 0xA0},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x73, 0x80, 0x7C, 0x21},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x74, 0x22, 0xE1, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x75, 0x19, 0x07, 0xA5},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x76, 0x22, 0x79, 0x0F},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x77, 0x19, 0x07, 0xA0},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x78, 0x02, 0x80, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x79, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x7A, 0x00, 0x00, 0x00},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x7B, 0x34, 0x00, 0x0E},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x7C, 0x18, 0x3F, 0xAF},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x7D, 0x34, 0x00, 0x0E},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x7E, 0x19, 0x6C, 0xA4},
	{8, 0xFC, 0xF3, 0x0D, 0x10, 0x7F, 0x19, 0x66, 0x7F},
	{5, 0xFC, 0xF3, 0x68, 0x64, 0x00, 0x00, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xA2, 0x91, 0xCB, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xB2, 0x50, 0x51, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xA3, 0x82, 0xB5, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xB3, 0x02, 0xB8, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xA4, 0x82, 0xBC, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xB4, 0x50, 0x56, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xA5, 0x83, 0xF4, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xB5, 0x50, 0x5C, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xA6, 0x96, 0x66, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x3F, 0xB6, 0x50, 0x7D, 0x00},
	//init
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xF8, 0x80, 0x05, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xC8, 0x00, 0x09, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xEE, 0x00, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xF9, 0x08, 0x5F, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xFA, 0x24, 0x83, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x05, 0x00, 0x04, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x01, 0x00, 0x01, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x07, 0xF0, 0xF0, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x09, 0x08, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x0D, 0x01, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0xE8, 0x20, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x0C, 0x06, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xF2, 0x00, 0x44, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x57, 0x01, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x0A, 0x1A, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xC6, 0x00, 0x31, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xC7, 0x00, 0x50, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xD2, 0x82, 0x94, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xF6, 0x00, 0x00, 0x00},//turn off DAC
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x03, 0x01, 0x01, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x04, 0x03, 0x10, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x2F, 0x01, 0x20, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0x33, 0x02, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0xB3, 0x00, 0x0C, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0xD5, 0x20, 0x00, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x23, 0xCF, 0x01, 0x80, 0x00},
	{7, 0xFC, 0xF3, 0x3B, 0x22, 0xFB, 0x00, 0x00, 0x00},
	};

static struct fm34_platform_data fm34_dev_data = {
	.pin_id_bp	= DAP2_DIN,
	.pin_id_rst	= DAP2_DOUT,
	.pin_id_pwdn	= DAP2_FS,

	.mode_int_mic_record = FM34_BY_PASS,
	.mode_int_mic_incall = FM34_ENABLE_EC,
	.mode_hs_mic_record = FM34_BY_PASS,
	.mode_hs_mic_incall = FM34_BY_PASS,

	.cmd_num = ARRAY_SIZE(fm34_cmd_buf),
	.cmd_buf = (unsigned char *) fm34_cmd_buf,
};

static struct wm8903_platform_data wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0x83,           /* enable mic bias current */
	.micdet_delay = 0,
	.gpio_base = WM8903_GPIO_BASE,
	.gpio_cfg = {
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
		0,                     /* as output pin */
		WM8903_GPn_FN_GPIO_MICBIAS_CURRENT_DETECT
		<< WM8903_GP4_FN_SHIFT, /* as micbias current detect */
		WM8903_GPIO_NO_CONFIG,
	},

	.pin_id_en_mic_ext_n = SPI2_MISO,

	.int_mic_l  = true,
	.int_mic_r = true,
	.hs_mic_l = false,
	.hs_mic_r = true,

	.dacl_vol = 0xBE,
	.dacr_vol = 0xBE,
	.hpoutl_vol = 0x30,
	.hpoutr_vol = 0x30,
	.lineoutl_vol = 0x38,
	.lineoutr_vol = 0x38,
	.spkl_vol = 0x3C,
	.spkr_vol = 0x3C,

	.int_mic_adcl_vol = 0xDB,
	.int_mic_adcr_vol = 0xDB,
	.hs_mic_adcl_vol = 0xEF,
	.hs_mic_adcr_vol = 0xEF,
	.lin_vol = 0x1B,
	.rin_vol = 0x1B,

	.int_mic_record_drc = true,
	.int_mic_incall_drc = false,
	.hs_mic_record_drc = false,
	.hs_mic_incall_drc = false,

	.int_mic_record_drc0 = 0x89AF,
	.int_mic_record_drc1 = 0x3246,
	.int_mic_record_drc2 = 0x002C,
	.int_mic_record_drc3 = 0x01C0,
	.int_mic_incall_drc0 = 0x89AF,
	.int_mic_incall_drc1 = 0x3246,
	.int_mic_incall_drc2 = 0x002C,
	.int_mic_incall_drc3 = 0x01C0,
	.hs_mic_record_drc0 = 0x89AF,
	.hs_mic_record_drc1 = 0x3246,
	.hs_mic_record_drc2 = 0x002C,
	.hs_mic_record_drc3 = 0x01C0,
	.hs_mic_incall_drc0 = 0x89AF,
	.hs_mic_incall_drc1 = 0x3246,
	.hs_mic_incall_drc2 = 0x002C,
	.hs_mic_incall_drc3 = 0x01C0,

	.int_mic_record_hpf = true,
	.int_mic_incall_hpf = true,
	.hs_mic_record_hpf = true,
	.hs_mic_incall_hpf = true,

	.int_mic_record_hpf_cut = 0, /* HiFi Mode */
	.int_mic_incall_hpf_cut = 0, /* HiFi Mode */
	.hs_mic_record_hpf_cut = 0, /* HiFi Mode */
	.hs_mic_incall_hpf_cut = 0, /* HiFi Mode */
};

#ifdef CONFIG_EEPROM_NV_ITEM
static struct eeprom_nv_item_platform_data eeprom_nv_items = {
	.num_item = EEPROM_NV_ITEM_TOTAL,
	.items = {
		{
			.name = "eeprom_version",
			.start_byte = EEPROM_OFFSET_VERSION,
			.num_byte = EEPROM_SIZE_VERSION,
		},
		{
			.name = "g-sensor",
			.start_byte = EEPROM_OFFSET_GSENSOR,
			.num_byte = EEPROM_SIZE_GSENSOR,
		},
		{
			.name = "gyro",
			.start_byte = EEPROM_OFFSET_GYRO,
			.num_byte = EEPROM_SIZE_GYRO,
		},
		{
			.name = "bt",
			.start_byte = EEPROM_OFFSET_BT,
			.num_byte = EEPROM_SIZE_BT,
		},
		{
			.name = "cpu_type",
			.start_byte = EEPROM_OFFSET_CPU_TYPE,
			.num_byte = EEPROM_SIZE_CPU_TYPE,
		},
		{
			.name = "model_id",
			.start_byte = EEPROM_OFFSET_MODEL_ID,
			.num_byte = EEPROM_SIZE_MODEL_ID,
		},
		{
			.name = "sku",
			.start_byte = EEPROM_OFFSET_SKU,
			.num_byte = EEPROM_SIZE_SKU,
		},
		{
			.name = "sim_lock",
			.start_byte = EEPROM_OFFSET_SIM_LOCK,
			.num_byte = EEPROM_SIZE_SIM_LOCK,
		},
		{
			.name = "sw_version",
			.start_byte = EEPROM_OFFSET_SW_VERSION,
			.num_byte = EEPROM_SIZE_SW_VERSION,
		},
		{
			.name = "locale",
			.start_byte = EEPROM_OFFSET_LOCALE,
			.num_byte = EEPROM_SIZE_LOCALE,
		},
		{
			.name = "isn",
			.start_byte = EEPROM_OFFSET_ISN,
			.num_byte = EEPROM_SIZE_ISN,
		},
		{
			.name = "ota_version",
			.start_byte = EEPROM_OFFSET_OTA_VERSION,
			.num_byte = EEPROM_SIZE_OTA_VERSION,
		},
		{
			.name = "country_code",
			.start_byte = EEPROM_OFFSET_COUNTRY_CODE,
			.num_byte = EEPROM_SIZE_COUNTRY_CODE,
		},
		{
			.name = "service_tag",
			.start_byte = EEPROM_OFFSET_SERVICE_TAG,
			.num_byte = EEPROM_SIZE_SERVICE_TAG,
		},
		{
			.name = "start_usage",
			.start_byte = EEPROM_OFFSET_START_USAGE,
			.num_byte = EEPROM_SIZE_START_USAGE,
		},
		{
			.name = "ppid",
			.start_byte = EEPROM_OFFSET_PPID,
			.num_byte = EEPROM_SIZE_PPID,
		},
		{
			.name = "wifi_only",
			.start_byte = EEPROM_OFFSET_WIFI_ONLY,
			.num_byte = EEPROM_SIZE_WIFI_ONLY,
		},
	}
};
#endif

/* pwr i2c */
static struct i2c_board_info __initdata gallo_i2c_bus4_board_info[] = {

#ifdef CONFIG_EEPROM_NV_ITEM
	{
		I2C_BOARD_INFO("eeprom_nv_item", 0xA0>>1),
		.platform_data = &eeprom_nv_items,
	},
#endif

};


static struct tegra_ulpi_config gallo_ehci2_ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "clk_dev2",
};

static struct tegra_ehci_platform_data gallo_ehci2_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
	.phy_config = &gallo_ehci2_ulpi_phy_config,
	.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
};

static struct tegra_i2c_platform_data gallo_i2c1_platform_data_400k = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
	.timeout	= 150,
	.i2c_bus_reset_devices = gallo_i2c_bus_reset_devices_1,
};

static struct tegra_i2c_platform_data gallo_i2c1_platform_data_100k = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.slave_addr = 0x00FC,
	.timeout	= 150,
	.i2c_bus_reset_devices = gallo_i2c_bus_reset_devices_1,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data gallo_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 90000, 100000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.slave_addr = 0x00FC,
	.timeout	= 150,
};

static struct tegra_i2c_platform_data gallo_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
	.timeout	= 150,
};

static struct tegra_i2c_platform_data gallo_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
	.timeout	= 150,
};


static struct tegra_audio_platform_data tegra_audio_pdata[] = {
	/* For I2S1 */
	[0] = {
		.i2s_master	= true,
	.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 44100,
	.i2s_clk_rate	= 2822400,
	.dap_clk	= "clk_dev1",
	.audio_sync_clk = "audio_2x",
	.mode		= I2S_BIT_FORMAT_I2S,
		.fifo_fmt	= I2S_FIFO_PACKED,
	.bit_size	= I2S_BIT_SIZE_16,
		.i2s_bus_width = 32,
		.dsp_bus_width = 16,
		.en_dmic = false, /* by default analog mic is used */
	},
	/* For I2S2 */
	[1] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 8000,
		.dsp_master_clk = 8000,
		.i2s_clk_rate	= 2000000,
		.dap_clk	= "clk_dev1",
		.audio_sync_clk = "audio_2x",
		.mode		= I2S_BIT_FORMAT_DSP,
		.fifo_fmt	= I2S_FIFO_16_LSB,
		.bit_size	= I2S_BIT_SIZE_16,
		.i2s_bus_width = 32,
		.dsp_bus_width = 16,
	}
};

static struct tegra_das_platform_data tegra_das_pdata = {
	.dap_clk = "clk_dev1",
	.tegra_dap_port_info_table = {
		/* I2S1 <--> DAC1 <--> DAP1 <--> Hifi Codec */
		[0] = {
			.dac_port = tegra_das_port_i2s1,
			.dap_port = tegra_das_port_dap1,
			.codec_type = tegra_audio_codec_type_hifi,
			.device_property = {
				.num_channels = 2,
				.bits_per_sample = 16,
				.rate = 44100,
				.dac_dap_data_comm_format =
						dac_dap_data_format_all,
			},
		},
		[1] = {
			.dac_port = tegra_das_port_none,
			.dap_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		[2] = {
			.dac_port = tegra_das_port_none,
			.dap_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		/* I2S2 <--> DAC2 <--> DAP4 <--> BT SCO Codec */
		[3] = {
			.dac_port = tegra_das_port_i2s2,
			.dap_port = tegra_das_port_dap4,
			.codec_type = tegra_audio_codec_type_bluetooth,
			.device_property = {
				.num_channels = 1,
				.bits_per_sample = 16,
				.rate = 8000,
				.dac_dap_data_comm_format =
					dac_dap_data_format_dsp,
			},
		},
		[4] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
	},

	.tegra_das_con_table = {
		[0] = {
			.con_id = tegra_das_port_con_id_hifi,
			.num_entries = 2,
			.con_line = {
				[0] = {tegra_das_port_i2s1, tegra_das_port_dap1, true},
				[1] = {tegra_das_port_dap1, tegra_das_port_i2s1, false},
			},
		},
		[1] = {
			.con_id = tegra_das_port_con_id_bt_codec,
			.num_entries = 4,
			.con_line = {
				[0] = {tegra_das_port_i2s2, tegra_das_port_dap4, true},
				[1] = {tegra_das_port_dap4, tegra_das_port_i2s2, false},
				[2] = {tegra_das_port_i2s1, tegra_das_port_dap1, true},
				[3] = {tegra_das_port_dap1, tegra_das_port_i2s1, false},
			},
		},
	}
};

static const struct i2c_board_info gallo_i2c_fm34[] = {
	{
		I2C_BOARD_INFO("fm34", 0x60),
		.platform_data = &fm34_dev_data,
	},
};


static const struct i2c_board_info gallo_i2c_wm8903[] = {
	{
		I2C_BOARD_INFO("wm8903", 0x1a),
		.platform_data = &wm8903_pdata,
	},
};

static void gallo_audio_init(void)
{
	int is_recovery_mode = (rck==0)?0:1;

	if ( ! is_recovery_mode || hw_info.hw_version >= BOARD_VERSION_DVT2) {
		/* Prevent from conflict with MCU (fw: < 0x05) download mode slave address: 0x60. 
                   Therefore disable FM34 in recovery mode
                */

		/* FM34 gpio init */
		tegra_gpio_enable(fm34_dev_data.pin_id_bp);
		gpio_request(fm34_dev_data.pin_id_bp, "fm34_pin_bypass");
		gpio_direction_output(fm34_dev_data.pin_id_bp, 1);

		tegra_gpio_enable(fm34_dev_data.pin_id_rst);
		gpio_request(fm34_dev_data.pin_id_rst, "fm34_pin_reset");
		gpio_direction_output(fm34_dev_data.pin_id_rst, 1);

		tegra_gpio_enable(fm34_dev_data.pin_id_pwdn);
		gpio_request(fm34_dev_data.pin_id_pwdn, "fm34_pin_powerdown");
		gpio_direction_output(fm34_dev_data.pin_id_pwdn, 1);

		gpio_export(DAP2_FS, false);
		gpio_export(DAP2_DIN, false);
		gpio_export(DAP2_DOUT, false);
		i2c_register_board_info(0, gallo_i2c_fm34, ARRAY_SIZE(gallo_i2c_fm34));
	}

	/* WM8903 gpio init */
	tegra_gpio_enable(SPI2_MISO);
	gpio_request(SPI2_MISO, "en_ext_mic_n");
	gpio_direction_output(SPI2_MISO, 1);

	gpio_export(SPI2_MISO, false);

	i2c_register_board_info(0, gallo_i2c_wm8903, ARRAY_SIZE(gallo_i2c_wm8903));
}


static void gallo_i2c_init(int is_recovery_mode)
{
	if (is_recovery_mode || hw_info.hw_version <= BOARD_VERSION_DVT) {
		printk(KERN_INFO "%s. Set gen1 clock to 100k", __func__);
		tegra_i2c_device1.dev.platform_data = &gallo_i2c1_platform_data_100k;
	} else {
		printk(KERN_INFO "%s. Set gen1 clock to 400k", __func__);
		tegra_i2c_device1.dev.platform_data = &gallo_i2c1_platform_data_400k;
	}

	tegra_i2c_device2.dev.platform_data = &gallo_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &gallo_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &gallo_dvc_platform_data;

	gallo_audio_init();
	i2c_register_board_info(4, gallo_i2c_bus4_board_info, ARRAY_SIZE(gallo_i2c_bus4_board_info));

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);
}


#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button gallo_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PQ5, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PQ4, 0),
	[2] = GPIO_KEY(KEY_POWER, PV2, 1),
};

#define PMC_WAKE_STATUS 0x14
#define TEGRA_WAKE_GPIO_POWER	TEGRA_WAKE_GPIO_PV2

static int gallo_wakeup_key(void)
{
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	return status & TEGRA_WAKE_GPIO_POWER ? KEY_POWER : KEY_RESERVED;
}

static void gallo_wakeup_key_clear(void)
{
	writel(TEGRA_WAKE_GPIO_POWER, IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
}

static struct gpio_keys_platform_data gallo_keys_platform_data = {
	.buttons		= gallo_keys,
	.nbuttons		= ARRAY_SIZE(gallo_keys),
	.wakeup_key		= gallo_wakeup_key,
	.wakeup_key_clear	= gallo_wakeup_key_clear,
};

static struct platform_device gallo_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data	= &gallo_keys_platform_data,
	},
};

static void gallo_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gallo_keys); i++)
		tegra_gpio_enable(gallo_keys[i].gpio);
}
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *gallo_devices[] __initdata = {
	&tegra_uartb_device,
	&tegra_uartc_device,
	&pmu_device,
	&tegra_gart_device,
	&tegra_aes_device,
#ifdef CONFIG_KEYBOARD_GPIO
	&gallo_keys_device,
#endif
	&tegra_wdt_device,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_spdif_device,
	&tegra_avp_device,
	&tegra_camera,
	&tegra_das_device,
};

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
/* Atmel MaxTouch touchscreen              Driver data */
/*-----------------------------------------------------*/
/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */

#define MXT_IRQ_GPIO	TEGRA_GPIO_PV6
#define MXT_RESET_GPIO	TEGRA_GPIO_PU2
#define MXT_WAKE_GPIO	TEGRA_GPIO_PX2
#define MXT_PWR_EN_GPIO	TEGRA_GPIO_PP1

static u8 read_chg(void)
{
	return gpio_get_value(MXT_IRQ_GPIO);
}

static u8 valid_interrupt(void)
{
	return !read_chg();
}

static void mxt_init_platform_hw(void) {
	printk(KERN_INFO "[Touch] In %s.", __func__);

	gpio_set_value(MXT_WAKE_GPIO, 0);
	gpio_set_value(MXT_RESET_GPIO, 0);
	/* power on seq: RST->0, 90ns RST->1 */

	if (hw_info.hw_version != BOARD_VERSION_DVT2) {
		gpio_set_value(MXT_PWR_EN_GPIO, 0);
		msleep(10);
		gpio_set_value(MXT_PWR_EN_GPIO, 1);
		msleep(1);
	}
	gpio_set_value(MXT_RESET_GPIO, 1);
	msleep(120);
}

static void mxt_exit_platform_hw(void)
{
	printk(KERN_INFO "[Touch] In %s.", __func__);
}

static u8 T0_data[T24_SIZE] = {	/* T24 has the max size in zero objects */
	[0] = 0,	/* CTRL */
};

static u8 T7_data[] = {
	[0] = 30,	/* IDLEACQINT */
	[1] = 10,	/* ACTVACQINT */
	[2] = 15,	/* ACTV2IDLETO */
};

static u8 T8_data[] = {	/* << Gallo >> */
	[0] = 9,	/* CHRGTIME */
	[1] = 0,	/* Reserved - ATCHDRIFT */
	[2] = 20,	/* TCHDRIFT */
	[3] = 60,	/* DRIFTST */
	[4] = 0,	/* TCHAUTOCAL */
	[5] = 0,	/* Reserved - SYNC */
	[6] = 5,	/* ATCHCALST */
	[7] = 0,	/* ATCHCALSTHR */
	[8] = 50,	/* ATCHFRCCALTHR */
	[9] = 25,	/* ATCHFRCCALRATIO */
};

static u8 T9_data[] = {		/* << Gallo >> */
	[ 0] = 0x8F,			/* CTRL */
	[ 1] = 0x00,			/* XORIGIN */
	[ 2] = 0x00,			/* YORIGIN */
	[ 3] = 0x1C,			/* XSIZE */
	[ 4] = 0x2A,			/* YSIZE */
	[ 5] = 0x00,			/* AKSCFG */
	[ 6] = 0x10,			/* BLEN */
	[ 7] = 70,			/* TCHTHR */
	[ 8] = 0x03,			/* TCHDI */
	[ 9] = 0x01,			/* ORIENT */
	[10] = 0x00,			/* MRGTIMEOUT */
	[11] = 0x00,			/* MOVHYSTI */
	[12] = 0x03,			/* MOVHYSTN */
	[13] = 0x2E,			/* MOVFILTER */
	[14] = 0x0A,			/* NUMTOUCH */	/* Will be over write by Atmel_mxt_info.numtouch */
	[15] = 15,			/* MRGHYST */
	[16] = 14,			/* MRGTHR */
	[17] = 0x0A,			/* AMPHYST */
	[18] = ( 799 >> 0) & 0xFF,	/* XRANGE (low) */
	[19] = ( 799 >> 8) & 0xFF,	/* XRANGE (high) */
	[20] = (1279 >> 0) & 0xFF,	/* YRANGE (low) */
	[21] = (1279 >> 8) & 0xFF,	/* YRANGE (high) */
	[22] = 0x00,			/* XLOCLIP */
	[23] = 0x00,			/* XHICLOP */
	[24] = 0x00,			/* YLOCLIP */
	[25] = 0x00,			/* YHICLIP */
	[26] = 0x00,			/* XEDGECTRL */
	[27] = 0x00,			/* XEDGEDIST */
	[28] = 0x40,			/* YEDGECTRL */
	[29] = 0x00,			/* YEDGEDIST */
	[30] = 15,			/* JUMPLIMIT */
	[31] = 15,			/* TCHHYST */
	[32] = 0x00,			/* XPITCH */
	[33] = 0x00,			/* YPITCH */
};

static u8 T22_data[] = {	/* << Gallo >> */
	[ 0] = 0x05,	/* CTRL */
	[ 1] = 0x00,	/* Reserved */
	[ 2] = 0x00,	/* Reserved */
	[ 3] = 0x00,	/* Reserved */
	[ 4] = 0x00,	/* Reserved */
	[ 5] = 0x00,	/* Reserved */
	[ 6] = 0x00,	/* Reserved */
	[ 7] = 0x00,	/* Reserved */
	[ 8] = 40,	/* NOISETHR */
	[ 9] = 0x00,	/* Reserved */
	[10] = 0x00,	/* FREQHOPSCALE */
	[11] = 15,	/* FREQ[0] */
	[12] = 20,	/* FREQ[1] */
	[13] = 25,	/* FREQ[2] */
	[14] = 30,	/* FREQ[3] */
	[15] = 33,	/* FREQ[4] */
	[16] = 0x00,	/* Reserved */

};

static u8 T25_data[] = {
	[ 0] = 0x03,			/* CTRL */
	[ 1] = 0x00,			/* CMD */
	[ 2] = (9700 >> 0) & 0xFF,	/* HISIGLIM[0] (low) */
	[ 3] = (9700 >> 8) & 0xFF,	/* HISIGLIM[0] (high) */
	[ 4] = (7000 >> 0) & 0xFF,	/* LOSIGLIM[0] (low) */
	[ 5] = (7000 >> 8) & 0xFF,	/* LOSIGLIM[0] (high) */
};

static u8 T28_data_Sintek[] = {
	[ 0] = 0x00,	/* CTRL */
	[ 1] = 0x00,	/* CMD */
	[ 2] = 0x03,	/* Reserved */
	[ 3] = 0x08,	/* IDLEGCAFDEPTH */
	[ 4] = 0x08,	/* ACTVGCAFDEPTH */
	[ 5] = 0x3C,	/* VOLTAGE */
};

static u8 T28_data_AUO[] = {
	[ 0] = 0x00,	/* CTRL */
	[ 1] = 0x00,	/* CMD */
	[ 2] = 0x00,	/* Reserved */
	[ 3] = 0x08,	/* IDLEGCAFDEPTH */
	[ 4] = 0x10,	/* ACTVGCAFDEPTH */
	[ 5] = 0x3C,	/* VOLTAGE */
};

static u8 T41_data[] = {
	[ 0] = 0x01,	/* CTRL */
	[ 1] = 0x00,	/* Reserved */
	[ 2] = 0x00,	/* Reserved */
	[ 3] = 0x32,	/* LARGEOBJTHR */
	[ 4] = 0x05,	/* DISTANCETHR */
	[ 5] = 0x01,	/* SUPEXTTO */
	[ 6] = 0xAA,	/* THUMB FIXED */
};

static struct mxt_cfg_object cfg_object_Sintek[] = {
	{.type = 15,	.instance = 0,	.length = T15_SIZE,			.p_data = &T0_data[0]},
	{.type = 15,	.instance = 1,	.length = T15_SIZE,			.p_data = &T0_data[0]},
	{.type = 18,	.instance = 0,	.length = T18_SIZE,			.p_data = &T0_data[0]},
	{.type = 24,	.instance = 0,	.length = T24_SIZE,			.p_data = &T0_data[0]},
	{.type = 27,	.instance = 0,	.length = T27_SIZE,			.p_data = &T0_data[0]},
	{.type = 40,	.instance = 0,	.length = T40_SIZE,			.p_data = &T0_data[0]},
	{.type = 43,	.instance = 0,	.length = T43_SIZE,			.p_data = &T0_data[0]},

	{.type = 7,	.instance = 0,	.length = ARRAY_SIZE(T7_data),		.p_data = &T7_data[0]},
	{.type = 8,	.instance = 0,	.length = ARRAY_SIZE(T8_data),		.p_data = &T8_data[0]},
	{.type = 9,	.instance = 0,	.length = ARRAY_SIZE(T9_data),		.p_data = &T9_data[0]},
	{.type = 22,	.instance = 0,	.length = ARRAY_SIZE(T22_data),		.p_data = &T22_data[0]},
	{.type = 25,	.instance = 0,	.length = ARRAY_SIZE(T25_data),		.p_data = &T25_data[0]},
	{.type = 28,	.instance = 0,	.length = ARRAY_SIZE(T28_data_Sintek),	.p_data = &T28_data_Sintek[0]},
	{.type = 41,	.instance = 0,	.length = ARRAY_SIZE(T41_data),		.p_data = &T41_data[0]},
};

static struct mxt_cfg_object cfg_object_AUO[] = {
	{.type = 15,	.instance = 0,	.length = T15_SIZE,			.p_data = &T0_data[0]},
	{.type = 15,	.instance = 1,	.length = T15_SIZE,			.p_data = &T0_data[0]},
	{.type = 18,	.instance = 0,	.length = T18_SIZE,			.p_data = &T0_data[0]},
	{.type = 24,	.instance = 0,	.length = T24_SIZE,			.p_data = &T0_data[0]},
	{.type = 27,	.instance = 0,	.length = T27_SIZE,			.p_data = &T0_data[0]},
	{.type = 40,	.instance = 0,	.length = T40_SIZE,			.p_data = &T0_data[0]},
	{.type = 43,	.instance = 0,	.length = T43_SIZE,			.p_data = &T0_data[0]},

	{.type = 7,	.instance = 0,	.length = ARRAY_SIZE(T7_data),		.p_data = &T7_data[0]},
	{.type = 8,	.instance = 0,	.length = ARRAY_SIZE(T8_data),		.p_data = &T8_data[0]},
	{.type = 9,	.instance = 0,	.length = ARRAY_SIZE(T9_data),		.p_data = &T9_data[0]},
	{.type = 22,	.instance = 0,	.length = ARRAY_SIZE(T22_data),		.p_data = &T22_data[0]},
	{.type = 25,	.instance = 0,	.length = ARRAY_SIZE(T25_data),		.p_data = &T25_data[0]},
	{.type = 28,	.instance = 0,	.length = ARRAY_SIZE(T28_data_AUO), 	.p_data = &T28_data_AUO[0]},
	{.type = 41,	.instance = 0,	.length = ARRAY_SIZE(T41_data), 	.p_data = &T41_data[0]},
};

static struct mxt_cfg_object cfg_object_Cando[] = {
	{.type = 15,	.instance = 0,	.length = T15_SIZE,			.p_data = &T0_data[0]},
	{.type = 15,	.instance = 1,	.length = T15_SIZE,			.p_data = &T0_data[0]},
	{.type = 18,	.instance = 0,	.length = T18_SIZE,			.p_data = &T0_data[0]},
	{.type = 24,	.instance = 0,	.length = T24_SIZE,			.p_data = &T0_data[0]},
	{.type = 27,	.instance = 0,	.length = T27_SIZE,			.p_data = &T0_data[0]},
	{.type = 40,	.instance = 0,	.length = T40_SIZE,			.p_data = &T0_data[0]},
	{.type = 43,	.instance = 0,	.length = T43_SIZE,			.p_data = &T0_data[0]},

	{.type = 7,	.instance = 0,	.length = ARRAY_SIZE(T7_data),		.p_data = &T7_data[0]},
	{.type = 8,	.instance = 0,	.length = ARRAY_SIZE(T8_data),		.p_data = &T8_data[0]},
	{.type = 9,	.instance = 0,	.length = ARRAY_SIZE(T9_data),		.p_data = &T9_data[0]},
	{.type = 22,	.instance = 0,	.length = ARRAY_SIZE(T22_data),		.p_data = &T22_data[0]},
	{.type = 25,	.instance = 0,	.length = ARRAY_SIZE(T25_data), 	.p_data = &T25_data[0]},
	{.type = 28,	.instance = 0,	.length = ARRAY_SIZE(T28_data_AUO), 	.p_data = &T28_data_AUO[0]},
	{.type = 41,	.instance = 0,	.length = ARRAY_SIZE(T41_data), 	.p_data = &T41_data[0]},
};

struct mxt_cfg_data cfg_data[] = {
	[0] = {	/* Sintek */
		.cfg_objects = &cfg_object_Sintek[0],
		.cfg_objects_length = ARRAY_SIZE(cfg_object_Sintek),
		.cfg_checksum = {0x96, 0x2E, 0x82},
	      },
	[1] = {	/* AUO */
		.cfg_objects = &cfg_object_AUO[0],
		.cfg_objects_length = ARRAY_SIZE(cfg_object_AUO),
		.cfg_checksum = {0x96, 0x56, 0x82},
	      },
	[2] = {	/* Cando */
		.cfg_objects = &cfg_object_Cando[0],
		.cfg_objects_length = ARRAY_SIZE(cfg_object_Cando),
		.cfg_checksum = {0x96, 0x56, 0x82},
	      },
};

static struct mxt_platform_data Atmel_mxt_info = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch = 10,
	.init_platform_hw = &mxt_init_platform_hw,
	.exit_platform_hw = &mxt_exit_platform_hw,
	.max_x = 1279,
	.max_y = 799,
	.valid_interrupt = &valid_interrupt,
	.read_chg = &read_chg,

	.cfg_data = &cfg_data[0],
	.cfg_data_length = ARRAY_SIZE(cfg_data),
	.gpio_wake = MXT_WAKE_GPIO,
	.gpio_reset = MXT_RESET_GPIO,
};

static struct i2c_board_info __initdata i2c_info[] = {
	{
		I2C_BOARD_INFO("maXTouch", MXT_I2C_ADDRESS),
		.irq = TEGRA_GPIO_TO_IRQ(MXT_IRQ_GPIO),
		.platform_data = &Atmel_mxt_info,
	},
};

static int __init gallo_touch_init_atmel(void)
{
	/* config interrupt pin */
	if (hw_info.hw_version != BOARD_VERSION_DVT2) {
		tegra_gpio_enable(MXT_PWR_EN_GPIO);
		gpio_request(MXT_PWR_EN_GPIO, "Touch_PWR_EN");
		gpio_export(MXT_PWR_EN_GPIO, false);
		gpio_direction_output(MXT_PWR_EN_GPIO, 1);
		tegra_pinmux_set_tristate(TEGRA_PINGROUP_DAP3,	TEGRA_TRI_NORMAL);	/* MXT_PWR_EN_GPIO */
	}

	tegra_gpio_enable(MXT_IRQ_GPIO);
	tegra_gpio_enable(MXT_WAKE_GPIO);
	tegra_gpio_enable(MXT_RESET_GPIO);

	gpio_request(MXT_IRQ_GPIO, "Touch_IRQ");
	gpio_request(MXT_WAKE_GPIO, "Touch_Wake");
	gpio_request(MXT_RESET_GPIO, "Touch_Reset");

	gpio_export(MXT_IRQ_GPIO, false);
	gpio_export(MXT_WAKE_GPIO, false);
	gpio_export(MXT_RESET_GPIO, false);

	gpio_direction_input(MXT_IRQ_GPIO);
	gpio_direction_output(MXT_WAKE_GPIO, 0);
	gpio_direction_output(MXT_RESET_GPIO, 1);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_GPV,	TEGRA_TRI_NORMAL);	/* MXT_IRQ_GPIO */
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPIC,	TEGRA_TRI_NORMAL);	/* MXT_WAKE_GPIO */
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_GPU,	TEGRA_TRI_NORMAL);	/* MXT_RESET_GPIO */

	i2c_register_board_info(0, i2c_info, 1);

	return 0;
}
#endif /* CONFIG_TOUCHSCREEN_ATMEL_MT_T9 */

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_irq = TPS6586X_INT_BASE + TPS6586X_INT_USB_DET,
			.vbus_gpio = -1,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
	},
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
	},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
};

static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name, tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
		tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data), GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci_pdata[0],
				sizeof(struct tegra_ehci_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	kfree(pdev->dev.platform_data);
	pdev->dev.platform_data = NULL;
	platform_device_unregister(pdev);
}

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.host_register = &tegra_usb_otg_host_register,
	.host_unregister = &tegra_usb_otg_host_unregister,
};

static int __init gallo_gps_init(void)
{
	struct clk *clk32 = clk_get_sys(NULL, "blink");
	if (!IS_ERR(clk32)) {
		clk_set_rate(clk32,clk32->parent->rate);
		clk_enable(clk32);
	}

	tegra_gpio_enable(TEGRA_GPIO_PZ3);
	return 0;
}

static void gallo_power_off(void)
{
	int ret;

	ret = tps6586x_power_off();
	if (ret)
		pr_err("gallo: failed to power off\n");

	while(1);
}

static void __init gallo_power_off_init(void)
{
	pm_power_off = gallo_power_off;
}

#define SERIAL_NUMBER_LENGTH 20
static char usb_serial_num[SERIAL_NUMBER_LENGTH];
static void gallo_usb_init(void)
{
	char *src = NULL;
	int i;

	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));

        /* Pull high ulpi oc enable */
	if (b_version.hw_version >= BOARD_VERSION_DVT2) {
	        tegra_gpio_enable(USB_ULPI_OC);
		gpio_request(USB_ULPI_OC, "ulpi_oc_en");
		gpio_direction_output(USB_ULPI_OC, 1);
	}

	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	platform_device_register(&tegra_usb_fsg_device);
	platform_device_register(&androidusb_device);
	platform_device_register(&tegra_udc_device);

	tegra_ehci3_device.dev.platform_data=&tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);

	if(!is_wifi_sku){
		tegra_ehci2_device.dev.platform_data=&gallo_ehci2_ulpi_platform_data;
		platform_device_register(&tegra_ehci2_device);
	}

#ifdef CONFIG_USB_ANDROID_RNDIS
	src = usb_serial_num;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
	platform_device_register(&rndis_device);
#endif
}

#ifdef CONFIG_LEDS_GPIO
static struct gpio_led gallo_led_list = {
	.name = "amber",
	.gpio = LOW_ORG_LED_GPIO,
	.active_low = 0,
};

static struct gpio_led_platform_data gallo_led_data = {
	.num_leds = 1,
	.leds = &gallo_led_list,
};

static struct platform_device gallo_led_device = {
	 .name = "leds-gpio",
	 .id = -1,
	 .dev =  {
		.platform_data = &gallo_led_data,
	 },
};

static void gallo_led_init(void)
{
	platform_device_register(&gallo_led_device);
}
#endif

static int board_gpios[] = {
	TEGRA_GPIO_PR0,
	TEGRA_GPIO_PR1,
	TEGRA_GPIO_PR2,
	TEGRA_GPIO_PR3,
	TEGRA_GPIO_PR4,
};

static int unused_gpios[] = {
	/*group ata*/
	TEGRA_GPIO_PI3, /*GMI_CS6_N*/
	TEGRA_GPIO_PI4, /*GMI_RST_N*/
	/*group atd*/
	TEGRA_GPIO_PH0, /*GMI_AD8*/
	TEGRA_GPIO_PH2, /*GMI_AD10*/
	TEGRA_GPIO_PH3, /*GMI_AD11*/
	/*group ate*/
	TEGRA_GPIO_PH4, /*GMI_AD12*/
	TEGRA_GPIO_PH5, /*GMI_AD13*/
	TEGRA_GPIO_PH6, /*GMI_AD14*/
	TEGRA_GPIO_PH7, /*GMI_AD15*/
	/*group dta*/
	TEGRA_GPIO_PD5, /*VI_D1*/
	TEGRA_GPIO_PT4, /*VI_D0*/
	/*group dtb*/
	TEGRA_GPIO_PT2, /*VI_D10*/
	TEGRA_GPIO_PT3, /*VI_D11*/
	/*group dtc*/
	TEGRA_GPIO_PD6, /*VI_VSYNC*/
	TEGRA_GPIO_PD7, /*VI_HSYNC*/
	/*group dtd*/
	TEGRA_GPIO_PL0, /*VI_D2*/
	TEGRA_GPIO_PL1, /*VI_D3*/
	TEGRA_GPIO_PL2, /*VI_D4*/
	TEGRA_GPIO_PL3, /*VI_D5*/
	TEGRA_GPIO_PL4, /*VI_D6*/
	TEGRA_GPIO_PL5, /*VI_D7*/
	TEGRA_GPIO_PL6, /*VI_D8*/
	TEGRA_GPIO_PL7, /*VI_D9*/
	TEGRA_GPIO_PT0, /*VI_PCLK*/
	/*group gmc*/
	TEGRA_GPIO_PB1,	/*GMI_AD18*/
	TEGRA_GPIO_PK7,	/*GMI_AD19*/
	/*group gmd*/
	TEGRA_GPIO_PJ0,	/*GMI_CS0_N*/
	TEGRA_GPIO_PJ2,	/*GMI_CS1_N*/
	/*group gpu7*/
	TEGRA_GPIO_PU7,	/*JTAG_RTCK*/
	/*group ldi*/
	TEGRA_GPIO_PM6,	/*LCD_D22*/
	/*group lhp0*/
	TEGRA_GPIO_PM5,	/*LCD_D21*/
	/*group lhp1*/
	TEGRA_GPIO_PM2,	/*LCD_D18*/
	/*group lhp2*/
	TEGRA_GPIO_PM3,	/*LCD_D19*/
	/*group lm1*/
	TEGRA_GPIO_PW1,	/*LCD_M1*/
	/*group lpp*/
	TEGRA_GPIO_PM7,	/*LCD_D23*/
	/*group lpw1*/
	TEGRA_GPIO_PC1,	/*LCD_PWR1*/
	/*group lvp1*/
	TEGRA_GPIO_PM4,	/*LCD_D20*/
	/*group spid*/
	TEGRA_GPIO_PX4, /*SPI1_MOSI*/
	/*group spie*/
	TEGRA_GPIO_PX5, /*SPI1_SCK*/
	TEGRA_GPIO_PX6, /*SPI1_CS0_N*/
};

static int __init gallo_hw_info_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(board_gpios); i++) {
		tegra_gpio_enable(board_gpios[i]);
		ret = gpio_request(board_gpios[i], "HwInfo");
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, board_gpios[i]);
			goto fail;
		}
		gpio_direction_input(board_gpios[i]);
		gpio_export(board_gpios[i], false);
	}

	hw_info.pj_id = ((gpio_get_value(board_gpios[4]) << 1) |
				gpio_get_value(board_gpios[3]));

	hw_info.hw_version = ((gpio_get_value(board_gpios[2]) << 2) |
				(gpio_get_value(board_gpios[1]) << 1) |
				gpio_get_value(board_gpios[0]));

	pr_info("HwInfo : project=%d, version=%d\n",
				hw_info.pj_id, hw_info.hw_version);
	return 0;
fail:
	while (i--)
		gpio_free(board_gpios[i]);
	return ret;
}

void gallo_get_hw_info(struct board_version *bv)
{
	memcpy(bv, &hw_info, sizeof(*bv));
}

static int gallo_debug_switch_init(void)
{
	int ret;

	tegra_gpio_enable(DEBUG_SWITCH);

	ret = gpio_request(DEBUG_SWITCH, "DEBUG_SWITCH");

	if (ret < 0)
		return ret;

	if( is_debug_mode[0] == 'Y')
	{
		ret = gpio_direction_output(DEBUG_SWITCH, 1);
	}else{
		ret = gpio_direction_input(DEBUG_SWITCH);
	}
	gpio_export(DEBUG_SWITCH, true);

	if (ret < 0) {
		gpio_free(DEBUG_SWITCH);
		return ret;
	}

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_SLXK, TEGRA_TRI_NORMAL);

	return ret;
}

static int gallo_3G_power_enable(void)
{
	int ret;
/* gpio24 */
	tegra_gpio_enable(TEGRA_GPIO_PD0);
	ret = gpio_request(TEGRA_GPIO_PD0, "WWAN_DISABLE#");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(TEGRA_GPIO_PD0, 1);
	if (ret < 0) {
		gpio_free(TEGRA_GPIO_PD0);
		return ret;
	}
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_SLXK, TEGRA_TRI_NORMAL);
	gpio_free(TEGRA_GPIO_PD0);

/* gpio25 */
	tegra_gpio_enable(TEGRA_GPIO_PD1);

	ret = gpio_request(TEGRA_GPIO_PD1, "WWAN_3V3_POWER_ENABLE");
		if (ret < 0)
		return ret;
	ret = gpio_direction_output(TEGRA_GPIO_PD1, 1);
	if (ret < 0) {
		gpio_free(TEGRA_GPIO_PD1);
		return ret;
	}
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_SLXA, TEGRA_TRI_NORMAL);
	gpio_free(TEGRA_GPIO_PD1);
	return ret;
}

/*Switch GPIO*/
static struct gpio_switch_platform_data dock_switch_data = {
       .name = "dock",
       .gpio = NULL,
};

static struct platform_device dock_switch_device = {
       .name             = "switch-gpio",
       .dev = {
		   .platform_data = &dock_switch_data,
       }
};

static int gallo_docking_init(void)
{

/* Set gpio 149 as input, need to register interrupt function later */
	if (b_version.hw_version >= BOARD_VERSION_DVT2) {
		tegra_gpio_enable(DOCKING_INT);
		gpio_request(DOCKING_INT, "docking_int");
		gpio_direction_input(DOCKING_INT);
	}

	if (b_version.hw_version > BOARD_VERSION_DVT2)
		dock_switch_data.gpio = TEGRA_GPIO_PQ7;
	else
		dock_switch_data.gpio = TEGRA_GPIO_PQ1;

	platform_device_register(&dock_switch_device);

	return 0;
}

static void gallo_configure_unused_pins(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(unused_gpios); i++) {
		gpio_request(unused_gpios[i], "unused");
		tegra_gpio_enable(unused_gpios[i]);
		gpio_direction_input(unused_gpios[i]);
	}

	if (hw_info.hw_version < BOARD_VERSION_DVT2) {

		gpio_request(TEGRA_GPIO_PV7, "unused");
		tegra_gpio_enable(TEGRA_GPIO_PV7);
		gpio_direction_input(TEGRA_GPIO_PV7);
	}
}

static void __init tegra_gallo_init(void)
{
	char serial[20];
	int is_recovery_mode = (rck==0)?0:1;

	tegra_common_init();
	tegra_clk_init_from_table(gallo_clk_init_table);
	gallo_pinmux_init(is_wifi_sku);
	gallo_hw_info_init();

	gallo_get_hw_info(&b_version);
	gallo_i2c_init(is_recovery_mode);

	snprintf(serial, sizeof(serial), "%llx", tegra_chip_uid());

    system_serial_low = (unsigned int)(tegra_chip_uid() & 0xFFFFFFFF);
    system_serial_high = (unsigned int)((tegra_chip_uid() >> 32ull) & 0xFFFFFFFF);

    pr_info("tegra_chip_uid() = %s\n", serial);

	if (isn_number[0] == 'O') {
		andusb_plat.serial_number = kstrdup(serial, GFP_KERNEL);
	} else if (isn_number[0] == 'X' || isn_number[0] == 0xff) {
		andusb_plat.serial_number = "DELL_TABLET_DEVELOP";
	} else{
		andusb_plat.serial_number = "DELL_TABLET_NONE";
	}

	tegra_i2s_device1.dev.platform_data = &tegra_audio_pdata[0];
	tegra_i2s_device2.dev.platform_data = &tegra_audio_pdata[1];
	tegra_spdif_device.dev.platform_data = &tegra_spdif_pdata;
	if (is_tegra_debug_uartport_hs() == true)
		platform_device_register(&tegra_uartd_device);
	else
		platform_device_register(&debug_uart);
	tegra_das_device.dev.platform_data = &tegra_das_pdata;

	platform_add_devices(gallo_devices, ARRAY_SIZE(gallo_devices));

	gallo_sdhci_init(is_recovery_mode);
	gallo_regulator_init();

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	pr_info("Initializing Atmel touch driver\n");
	gallo_touch_init_atmel();
#endif

#ifdef CONFIG_KEYBOARD_GPIO
	gallo_keys_init();
#endif
#ifdef CONFIG_KEYBOARD_TEGRA
	gallo_kbc_init();
#endif
#ifdef CONFIG_LEDS_GPIO
	gallo_led_init();
#endif
	gallo_wired_jack_init();
	gallo_usb_init();
	gallo_gps_init();
	gallo_panel_init();
	gallo_sensors_init(is_recovery_mode, is_wifi_sku);
	gallo_bt_rfkill();
	gallo_power_off_init();
	gallo_emc_init();
#ifdef CONFIG_BT_BLUESLEEP
	tegra_setup_bluesleep();
#endif
	gallo_3G_power_enable();
	gallo_docking_init();
	gallo_configure_unused_pins();
	gallo_debug_switch_init();

	/*disabe consol switch when suspend/resume*/
	pm_set_vt_switch(0);
}


void gallo_i2c_bus_reset_devices_1(struct tegra_i2c_platform_data *pd, u16 client_addr)
{
	printk("i2c_bus_reset_devices:0x%x!!\n", client_addr);
	/*reset uP*/
	if (gallo_cg7216am_platform_data.gpio_mcu_rst != -1) {
		printk("Reset mcu by gpio:%d!!\n",  gallo_cg7216am_platform_data.gpio_mcu_rst);
		gpio_direction_output(gallo_cg7216am_platform_data.gpio_mcu_rst, 1); /* high active */
		mdelay(1);
		gpio_direction_output(gallo_cg7216am_platform_data.gpio_mcu_rst, 0);
	}

	/*reset Gyro/Gsensor*/
	/*reset FM34*/
	/*reset WM8903*/
	/*reset Touch*/
	mxt_init_platform_hw();
}

static int __init isn_check(char *options)
{
	isn_number = options;
	return 1;
}
__setup("isn=", isn_check);

static int __init sku_check(char *options)
{
	if(0 == strcmp(options, "wifi-only"))
	   is_wifi_sku = true;
	return 1;
}
__setup("androidboot.carrier=", sku_check);

static int __init rck_check(char *options)
{
	rck = options;
	return 1;
}
__setup("RCK=", rck_check);

static int __init debug_mode_check(char *options)
{
	is_debug_mode = options;
	return 1;
}
__setup("debug=", debug_mode_check);

int __init tegra_gallo_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_gallo_protected_aperture_init);

void __init tegra_gallo_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M, SZ_16M);
}

MACHINE_START(GALLO, "gallo")
	.boot_params    = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_gallo_init,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_gallo_reserve,
	.timer          = &tegra_timer,
MACHINE_END
