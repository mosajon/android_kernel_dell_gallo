#ifndef ___MT9P111_SENSOR_H__
#define ___MT9P111_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9P111_NAME	"mt9p111"
#define MT9P111_PATH     "/dev/mt9p111"

#define MT9P111_WAIT_MS       0
/* special number to indicate this is wait time require */
#define MT9P111_TABLE_END     1
/* special number to indicate this is end of table */
#define MT9P111_MAX_RETRIES   3 /* max counter for retry I2C access */

#define MT9P111_IOCTL_SET_MODE		_IOW('o', 1, struct mt9p111_mode)
#define MT9P111_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define MT9P111_IOCTL_TRIGGER_AF	_IOW('o', 3, __u32)
#define MT9P111_IOCTL_GET_AF_STATUS	_IOR('o', 4, __u16)
#define MT9P111_IOCTL_SET_EFFECT	_IOW('o', 5, struct mt9p111_effect)
#define MT9P111_IOCTL_GET_EXPOSURE_TIME	_IOR('o', 6, struct mt9p111_exposure)
#define MT9P111_IOCTL_GET_MODEL_ID	_IOR('o', 7, __u16)

#define MT9P111_ITEM_EFFECT		0
#define MT9P111_ITEM_WB			1
#define MT9P111_ITEM_BRIGHTNESS		2
#define MT9P111_ITEM_SCENE		3
#define MT9P111_ITEM_FLASHMODE		4
#define MT9P111_ITEM_MAX		5
#define MT9P111_ITEM_FLASHVALUE		6

#define MT9P111_EFFECT_NONE		0
#define MT9P111_EFFECT_MONO		1
#define MT9P111_EFFECT_SEPIA		2
#define MT9P111_EFFECT_NEGATIVE		3
#define MT9P111_EFFECT_SOLARIZE		4
#define MT9P111_EFFECT_POSTERIZE	5

#define MT9P111_WB_AUTO			0
#define MT9P111_WB_SUNLIGHT		1
#define MT9P111_WB_CLOUDY		2
#define MT9P111_WB_FLUORESCENT		3
#define MT9P111_WB_INCANDESCENT		4

#define MT9P111_SCENE_AUTO		0
#define MT9P111_SCENE_ACTION		1
#define MT9P111_SCENE_NIGHT		2

#define MT9P111_BRIGHTNESS_0		0
#define MT9P111_BRIGHTNESS_P1		1
#define MT9P111_BRIGHTNESS_P2		2
#define MT9P111_BRIGHTNESS_N1		3
#define MT9P111_BRIGHTNESS_N2		4

#define MT9P111_FLASHMODE_OFF		0
#define MT9P111_FLASHMODE_ON		1
#define MT9P111_FLASHMODE_AUTO		2
#define MT9P111_FLASHMODE_TORCH		3

struct mt9p111_mode {
	int xres;
	int yres;
};

struct mt9p111_effect {
	int item;
	int value;
};

struct mt9p111_exposure {
	int exposure;
	int line_per_second;
};

#ifdef __KERNEL__
struct mt9p111_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	int (*get_board_info)(void);
	int (*set_led_level) (int);
};
#endif /* __KERNEL__ */

#endif  /* __MT9P111_SENSOR_H__ */

