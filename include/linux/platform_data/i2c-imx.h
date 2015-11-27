/*
 * i2c.h - i.MX I2C driver header file
 *
 * Copyright (c) 2008, Darius Augulis <augulis.darius@gmail.com>
 *
 * This file is released under the GPLv2
 */

#ifndef __ASM_ARCH_I2C_H_
#define __ASM_ARCH_I2C_H_

/**
 * struct imxi2c_platform_data - structure of platform data for MXC I2C driver
 * @init:			Initialise gpio's and other board specific things
 * @exit:			Free everything initialised by @init
 * @bitrate:		Bus speed measured in Hz
 * @i2c_scl_pin:	GPIO pin number for the SCL pin
 * @i2c_sda_pin:	GPIO pin number for the SDA pin
 *
 **/

enum i2c_gpio_mode {
	i2c_gpio_mode_func,
	i2c_gpio_mode_gpio,
};

struct imxi2c_platform_data {
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
	int bitrate;
	void (*i2c_set_gpio_mode)( struct imxi2c_platform_data *, enum i2c_gpio_mode mode);
	unsigned int i2c_scl_pin;
	unsigned int i2c_sda_pin;
#ifdef CONFIG_OF
	struct pinctrl * pinctrl;
	struct pinctrl_state * pinctrl_state_i2c;
   	struct pinctrl_state * pinctrl_state_gpio;
#endif
};
#endif /* __ASM_ARCH_I2C_H_ */
