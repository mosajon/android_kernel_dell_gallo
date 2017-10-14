#ifndef __CG7216AM_H__
#define __CG7216AM_H__

struct device;

enum cg7216am_led_index
{
	AMBER,
	WHITE,
	GREEN
};

enum cg7216am_led_mode
{
	MCU_LED_ON,
	MCU_LED_OFF,
	MCU_LED_BLINKING
};

enum cg7216am_finished_judgement{
	JUDGEMENT_FOR_DEAD_BATTERY,
	JUDGEMENT_FOR_OVER_DISCHARGE_BATTERY,
	JUDGEMENT_FOR_VENDOR_NAME,
};

struct cg7216am_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

struct cg7216am_platform_data {
	int hw_version;
	int is_fw_update_mode;
	int gpio_sys_sus;
	int gpio_polarity_sys_sus;
	int gpio_mcu_int;
	int gpio_mcu_rst;
	int num_subdevs;
	struct cg7216am_subdev_info *sub_devs;
};

int cg7216am_led_set(struct device *cg7216am_dev, int index, enum cg7216am_led_mode mode);
int cg7216am_battery_disable_charging(struct device *cg7216am_dev, bool disable);

int cg7216am_get_dead_bat_remain_timeout(struct device *dev);
void cg7216am_store_dead_bat_remain_timeout(struct device *dev, int sec);
int cg7216am_get_over_dischg_bat_remain_timeout(struct device *dev);
void cg7216am_store_over_dischg_bat_remain_timeout(struct device *dev, int sec);
void cg7216am_set_judgement_finished(struct device *dev, enum cg7216am_finished_judgement, bool done);

#endif /* __CG7216AM_H__ */
