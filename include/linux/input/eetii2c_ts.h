#ifndef LINUX_INPUT_EETII2C_TS_H
#define LINUX_INPUT_EETII2C_TS_H

#define GF_EETII2C_ADDRESS 	(0x04)

struct eetii2c_ts_platform_data {
	bool (*guf_eetii2c_reset) (void);
	bool idle_support_mode;
	bool early_suspend_mode;
	int reset_pin;
	int wake_pin;
	int irq_flags;
	int gpio_int_number;
};

#endif /* LINUX_INPUT_EETII2C_TS_H */

