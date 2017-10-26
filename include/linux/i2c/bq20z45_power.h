

#ifndef __BQ20Z45_POWER_H__
#define __BQ20Z45_POWER_H__

#define PDA_POWER_AC_REMOVE  (0)
#define PDA_POWER_CHARGE_AC  (1 << 0)
#define PDA_POWER_CHARGE_USB (1 << 1)

enum ChargerStatus{
	CHARGING_NOT_PRESENT,
	CHARGING_IN_PROGRESS,
	CHARGING_COMPLETE,
	CHARGING_SUSPEND,
};

struct device;

struct smart_battery_charging_impl {
	bool (*init)(void);
	void (*ce_enable)(bool enable);
	bool (*validate_manufacture)(char *name);

	int (*get_dead_bat_recovery_timeout_default)(void);  /* in seconds */
	int (*get_dead_bat_recovery_timeout)(void);  /* in seconds */
	int (*get_over_dischg_bat_recovery_timeout_default)(void); /* in seconds */
	int (*get_over_dischg_bat_recovery_timeout)(void); /* in seconds */
	void (*store_dead_bat_recovery_timeout)(int sec);  /* in seconds */
	void (*store_over_dischg_bat_recovery_timeout)(int sec); /* in seconds */
	void (*set_judgement_finished)(int item, bool done);

	int over_discharge_criteria;
};

struct BQ20Z45_power_pdata {
	int (*is_ac_online)(void);
	int (*get_charger_status)(void);
	int ac_present_gpio;
	int nce_gpio;
	int lowbatt_gpio;
	int stat1_gpio;
	int stat2_gpio;

	unsigned int interval_fast;
	unsigned int interval_slow;

	struct smart_battery_charging_impl *sb_imp;
};

#endif /* __BQ20Z45_POWER_H__ */
