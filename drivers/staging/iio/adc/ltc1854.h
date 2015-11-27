/*
 * LTC1854/LTC1855/LTC1856 SPI ADC driver
 *
 * Copyright 2013 Clemens Terasa
 *
 * Licensed under the GPL-2 or later.
 */

struct ltc1854_platform_data {
	unsigned short convst_pin;
	unsigned short busy_neg_pin;
};

