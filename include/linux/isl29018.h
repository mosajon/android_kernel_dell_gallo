#ifndef ___ISL29018_H__
#define ___ISL29018_H__

struct lightsensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif
