#ifndef ___MT9P111_REG_H__
#define ___MT9P111_REG_H__

#include <linux/uaccess.h>
#include <linux/i2c.h>

#define MT9P111_AUTO_FLASH 0

#define CMD_REG16 1
#define CMD_REG8 2
#define CMD_BITSET16 3
#define CMD_BITCLR16 4
#define CMD_WAIT_MS 5
#define CMD_TABLE_END 6
#define CMD_POLL8 7
#define CMD_POLL16 8
#define CMD_POLL8_NOT 9
#define CMD_PGA1 10
#define CMD_PGA2 11
#define CMD_PGA3 12

struct sensor_reg_v2 {
	u32 purpose;
	u16 addr;
	u16 val;
};

enum {
	SENSOR_MODE_2592x1944,
	SENSOR_MODE_1280x960,
	SENSOR_MODE_1280x720,
	SENSOR_MODE_800x600,
	SENSOR_MODE_640x480,
};

extern struct mt9p111_reg_t mt9p111_regs;
extern struct sensor_reg_v2 *mode_table[];
extern struct sensor_reg_v2 mt9p111_flash_auto[];
extern struct sensor_reg_v2 mt9p111_flash_on[];
extern struct sensor_reg_v2 mt9p111_flash_off[];
extern struct sensor_reg_v2 mt9p111_focus_infinity[];
extern struct sensor_reg_v2 mt9p111_focus_auto[];
extern struct sensor_reg_v2 mt9p111_scene_auto[];
extern struct sensor_reg_v2 mt9p111_scene_action[];
extern struct sensor_reg_v2 mt9p111_scene_night[];
extern struct sensor_reg_v2 mt9p111_wb_auto[];
extern struct sensor_reg_v2 mt9p111_wb_sunlight[];
extern struct sensor_reg_v2 mt9p111_wb_fluorescent[];
extern struct sensor_reg_v2 mt9p111_wb_incandescent[];
extern struct sensor_reg_v2 mt9p111_effect_none[];
extern struct sensor_reg_v2 mt9p111_effect_mono[];
extern struct sensor_reg_v2 mt9p111_effect_sepia[];
extern struct sensor_reg_v2 mt9p111_effect_negative[];
extern struct sensor_reg_v2 mt9p111_effect_solarize[];
extern struct sensor_reg_v2 mt9p111_effect_posterize[];
extern struct sensor_reg_v2 mt9p111_brightness_0[];
extern struct sensor_reg_v2 mt9p111_brightness_p1[];
extern struct sensor_reg_v2 mt9p111_brightness_p2[];
extern struct sensor_reg_v2 mt9p111_brightness_n1[];
extern struct sensor_reg_v2 mt9p111_brightness_n2[];

extern u16 flash_target;
/*
 * mt9p111_check_if_2nd_source
 * @client: I2c access to the module
 * @board_info: Additional board-level info, like something obtained from hwid
 *
 * Returns 0.
 *
 * This function will try to check if 2nd source module shoule be used,
 * and make any nessesary change to mt9p111_regs.
 */
extern int mt9p111_check_if_2nd_source(struct i2c_client *client,
	int board_info);
extern int mt9p111_check_otp_lc(struct i2c_client *client, int board_info);
extern int mt9p111_write_table(struct i2c_client *client,
	 const struct sensor_reg_v2 table[]);

struct mt9p111_reg_t {
	struct sensor_reg_v2 const *back_to_preview;
	struct sensor_reg_v2 const *init;
	struct sensor_reg_v2 const *init2;
	struct sensor_reg_v2 const *lc;
	struct sensor_reg_v2 const *init3;
	struct sensor_reg_v2 const *lc_late;
};

#endif  /* __MT9P111_REG_H__ */
