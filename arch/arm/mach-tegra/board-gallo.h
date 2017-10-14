/*
 * arch/arm/mach-tegra/board-gallo.h
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_TEGRA_BOARD_GALLO_H
#define _MACH_TEGRA_BOARD_GALLO_H

int gallo_charge_init(void);
int gallo_regulator_init(void);
int gallo_sdhci_init(int is_recovery_mode);
int gallo_pinmux_init(bool is_wifi_sku);
int gallo_panel_init(void);
int gallo_wired_jack_init(void);
int gallo_sensors_init(int is_recovery_mode, bool is_wifi_sku);
int gallo_kbc_init(void);
int gallo_vibrator_init(void);
int gallo_emc_init(void);

/* external gpios */

/* TPS6586X gpios */
#define TPS6586X_GPIO_BASE	TEGRA_NR_GPIOS
#define LOW_ORG_LED_GPIO	TPS6586X_GPIO_BASE /* gpio1 */
#define AVDD_DSI_CSI_ENB_GPIO	(TPS6586X_GPIO_BASE + 1) /* gpio2 */

/* TCA6416 gpios */
#define TCA6416_GPIO_BASE	(TEGRA_NR_GPIOS + 4)
#define CAM1_PWR_DN_GPIO	(TCA6416_GPIO_BASE + 0) /* gpio0 */
#define CAM1_RST_L_GPIO		(TCA6416_GPIO_BASE + 1) /* gpio1 */
#define CAM1_AF_PWR_DN_L_GPIO	(TCA6416_GPIO_BASE + 2) /* gpio2 */
#define CAM1_LDO_SHUTDN_L_GPIO	(TCA6416_GPIO_BASE + 3) /* gpio3 */
#define CAM2_PWR_DN_GPIO	(TCA6416_GPIO_BASE + 4) /* gpio4 */
#define CAM2_RST_L_GPIO		(TCA6416_GPIO_BASE + 5) /* gpio5 */
#define CAM2_AF_PWR_DN_L_GPIO	(TCA6416_GPIO_BASE + 6) /* gpio6 */
#define CAM2_LDO_SHUTDN_L_GPIO	(TCA6416_GPIO_BASE + 7) /* gpio7 */
#define CAM3_PWR_DN_GPIO	(TCA6416_GPIO_BASE + 8) /* gpio8 */
#define CAM3_RST_L_GPIO		(TCA6416_GPIO_BASE + 9) /* gpio9 */
#define CAM3_AF_PWR_DN_L_GPIO	(TCA6416_GPIO_BASE + 10) /* gpio10 */
#define CAM3_LDO_SHUTDN_L_GPIO	(TCA6416_GPIO_BASE + 11) /* gpio11 */
#define CAM2_VDDIOTX_GPIO	(TCA6416_GPIO_BASE + 12)
#define CAM2_MVDD_GPIO		(TCA6416_GPIO_BASE + 13)
#define CAM3_VDD1V8_GPIO	(TCA6416_GPIO_BASE + 14)
#define CAM3_VDDIO1V8_GPIO	(TCA6416_GPIO_BASE + 15)
#define CAM_I2C_MUX_RST_GPIO	(TCA6416_GPIO_BASE + 15) /* gpio15 */

/* WM8903 gpios */
#define WM8903_GPIO_BASE	(TEGRA_NR_GPIOS + 32)
#define WM8903_GP1		(WM8903_GPIO_BASE + 0)
#define WM8903_GP2		(WM8903_GPIO_BASE + 1)
#define WM8903_GP3		(WM8903_GPIO_BASE + 2)
#define WM8903_GP4		(WM8903_GPIO_BASE + 3)
#define WM8903_GP5		(WM8903_GPIO_BASE + 4)

/* Interrupt numbers from external peripherals */
#define TPS6586X_INT_BASE	TEGRA_NR_IRQS
#define TPS6586X_INT_END	(TPS6586X_INT_BASE + 32)

/********************************************
*             | ID4 | ID3 | ID2 | ID1 | ID0
* ------------+-----+-----+-----+-----+-----
*  GALLO-EVT0 |  0  |  0  |  0  |  0  |  0
*  GALLO-EVT1 |  0  |  0  |  0  |  0  |  1
*  GALLO-EVT2 |  0  |  0  |  0  |  1  |  1
*  GALLO-DVT  |  0  |  0  |  1  |  0  |  0
*  GALLO-DVT2 |  0  |  0  |  1  |  0  |  1
*  GALLO-DVT3 |  0  |  0  |  1  |  1  |  0
*********************************************/

enum project_id {
	PROJECT_UNKNOWN = -1,
	GALLO,
};

enum hw_version {
	BOARD_VERSION_UNKNOWN = -1,
	BOARD_VERSION_EVT0 = 0,
	BOARD_VERSION_EVT1 = 1,
	BOARD_VERSION_EVT2 = 3,
	BOARD_VERSION_DVT = 4,
	BOARD_VERSION_DVT2 = 5,
	BOARD_VERSION_DVT3 = 6,
};

struct board_version {
	u8 pj_id;
	u8 hw_version;
};

void gallo_get_hw_info(struct board_version *);
#endif
