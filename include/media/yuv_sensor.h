#ifndef ___YUV_SENSOR_H__
#define ___YUV_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define SENSOR_NAME	"mt9d115"
#define SENSOR_PATH     "/dev/mt9d115"

#define SENSOR_WAIT_MS       0
/* special number to indicate this is wait time require */
#define SENSOR_TABLE_END     1
/* special number to indicate this is end of table */
#define SENSOR_MAX_RETRIES   3 /* max counter for retry I2C access */

#define SENSOR_IOCTL_SET_MODE		_IOW('o', 1, struct sensor_mode)
#define SENSOR_IOCTL_GET_STATUS		_IOR('o', 2, __u8)

struct sensor_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__
struct yuv_sensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __YUV_SENSOR_H__ */

