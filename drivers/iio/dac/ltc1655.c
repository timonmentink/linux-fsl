/*
 * LTC1655  Digital to analog convertors spi driver
 *
 * Copyright 2014 Garz und Fricke
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/div64.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

enum{
	ID_LTC1655,		
};

/**
 * struct ltc1655_chip_info - chip specific information
 * @channels:		channel spec for the DAC
 * @vref_max_mv:	upper end of the scale voltage
 * @vref_min_mv:	lower end of the scale voltage
 */

struct ltc1655_chip_info {
	const struct iio_chan_spec	*channels;
	int vref_max_mv;
	int vref_min_mv;
};

/**
 * struct ltc1655_state - driver instance specific data
 * @indio_dev:		the industrial I/O device
 * @us:				spi_device
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:			supply regulator
 */

struct ltc1655_state {
	struct spi_device		*us;
	const struct ltc1655_chip_info	*chip_info;
	struct regulator		*reg_pos, *reg_neg;
	int current_out;
	int vref_max_mv;
	int vref_min_mv;
	int state_gpio_count;
	int * state_gpio;
	enum of_gpio_flags * state_gpio_flags;
	int enable_gpio;
	enum of_gpio_flags enable_gpio_flags;
};

static int ltc1655_spi_write(struct spi_device *spi, u16 val )
{
	unsigned char msg[2];
	msg[0] = val >> 8;
	msg[1] = val >> 0;

	return spi_write(spi, msg, 2);
}

static int ltc1655_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ltc1655_state *st = iio_priv(indio_dev);
	unsigned long scale_uv;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = st->current_out;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		scale_uv = ((st->vref_max_mv -  st->vref_min_mv )* 1000) >> chan->scan_type.realbits;
		*val =  scale_uv / 1000;
		*val2 = (scale_uv % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO;

	}
	return -EINVAL;
}

static int ltc1655_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ltc1655_state *st = iio_priv(indio_dev);
	const int upper =  (1 << (chan->scan_type.realbits - 1)) - 1;
	const int lower = -1 * (1 << (chan->scan_type.realbits - 1));
	
	int ret;
	u16 out;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if(val > upper)
			val = upper;
		else if(val < lower)
			val = lower;

		out = val - lower;

		ret = ltc1655_spi_write(st->us, out);
		if(0==ret)
			st->current_out = val;
		return ret;
	default:
		ret = -EINVAL;
	}

	return -EINVAL;
}

static ssize_t ltc1655_read_scale_factor_frac(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ltc1655_state *st = iio_priv(indio_dev);
	const unsigned int max_raw =  (1 << chan->scan_type.realbits) - 1;
	unsigned long frac;
	u64 bitsper1000V, bitsperV;
	
	bitsper1000V  = (u64) max_raw * 1000000;
	do_div( bitsper1000V, st->vref_max_mv - st->vref_min_mv );
	bitsperV = bitsper1000V;
	do_div( bitsperV, 1000);
	frac = bitsper1000V - bitsperV * 1000;

	return sprintf(buf, "%llu.%03lu\n", bitsperV, frac);
}

static ssize_t ltc1655_read_scale_factor(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ltc1655_state *st = iio_priv(indio_dev);
	const unsigned int max_raw =  (1 << chan->scan_type.realbits) - 1;
	unsigned long bitsperV;
	
	bitsperV = max_raw * 1000 / (st->vref_max_mv - st->vref_min_mv );;
	return sprintf(buf, "%lu\n", bitsperV );
}

static int ltc1655_check_communication( struct ltc1655_state *st)
{
	int status;
	u8 send[2] ={0x55, 0xaa};
	u8 result[2];

	status = spi_write_then_read(st->us, &send, 2, (u8 *) &result, 2);
	if( status != 0 || send[0] != result[0]  || send[1] != result[1])
		return 0;
	
	return 1;
}
static int ltc1655_getState( struct ltc1655_state *st)
{
	int i, value, act_low;
	
	if(!st) return -EINVAL;
	if( !st->state_gpio) return 1;	// State OK, we don't knw it better

	for(i = 0; i < st->state_gpio_count; i++ )
	{
		if( st->state_gpio[i] >= 0 )
		{
			value = !!gpio_get_value(st->state_gpio[i]);
			act_low = st->state_gpio_flags[i] && OF_GPIO_ACTIVE_LOW;
			pr_debug("%s gpio %d : %d %d\n",__func__, st->state_gpio[i], value , st->state_gpio_flags[i]);
			if(( value == act_low ))
				return 0;
		}
	}

	return 1;
}

static ssize_t ltc1655_read_dac_state(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ltc1655_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", ltc1655_getState(st));
}

static const struct iio_info ltc1655_info = {
	.write_raw = ltc1655_write_raw,
	.read_raw = ltc1655_read_raw,
	.driver_module = THIS_MODULE,
};

static const struct iio_chan_spec_ext_info ltc1655_ext_info[] = {
	{
		.name = "state",
		.read = ltc1655_read_dac_state,
		.shared = false,
	},
	{
		.name = "scale_factor",
		.read = ltc1655_read_scale_factor,
		.shared = false,
	},
	{
		.name = "scale_factor_frac",
		.read = ltc1655_read_scale_factor_frac,
		.shared = false,
	},


	{ },
};


const struct iio_chan_spec ltc1655_channel = { 
	.type = IIO_VOLTAGE, 
	.indexed = 1, 
	.output = 1, 
	.channel = 0, 
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), 
	.address = 0, 
	.scan_type = IIO_ST('u', 16, 16, 0), 
	.ext_info = ltc1655_ext_info,
};


static const struct ltc1655_chip_info ltc1655_chip_info = {
	.channels = &ltc1655_channel,
	.vref_max_mv = 10000,
	.vref_min_mv = -10000,
};


static int ltc1655_probe(struct spi_device *spi)
{
	struct ltc1655_state *st;
	struct iio_dev *indio_dev;
	int ret=0, voltage_uv_pos = 0, voltage_uv_neg = 0;
	struct device_node *np = spi->dev.of_node;

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st = iio_priv(indio_dev);
	st->reg_pos = regulator_get(&spi->dev, "vref-pos");
	if (!IS_ERR(st->reg_pos)) {
		ret = regulator_enable(st->reg_pos);
		if (ret)
			goto error_put_reg_pos;

		ret = regulator_get_voltage(st->reg_pos);
		if (ret < 0)
			goto error_disable_reg_pos;

		voltage_uv_pos = ret;
	}

	st->reg_neg = regulator_get(&spi->dev, "vref-neg");
	if (!IS_ERR(st->reg_neg)) {
		ret = regulator_enable(st->reg_neg);
		if (ret)
			goto error_put_reg_neg;

		ret = regulator_get_voltage(st->reg_neg);
		if (ret < 0)
			goto error_disable_reg_neg;

		voltage_uv_neg = ret;
	}
	else
	
	if( np != NULL)
	{
		st->state_gpio_count = of_gpio_named_count( np, "state-gpio");
		if(st->state_gpio_count > 0)
		{
			int i;
			st->state_gpio = devm_kzalloc( &spi->dev, st->state_gpio_count * sizeof(int), GFP_KERNEL);
			st->state_gpio_flags = devm_kzalloc( &spi->dev, st->state_gpio_count * sizeof(enum of_gpio_flags), GFP_KERNEL);
			if( !st->state_gpio || !st->state_gpio_flags)
			{
				dev_err(&spi->dev, "failed to get memory for state gpios, continue anyway ...\n");
				st->state_gpio_count = 0;
				if(st->state_gpio) devm_kfree( &spi->dev, st->state_gpio);
				if(st->state_gpio_flags) devm_kfree( &spi->dev, st->state_gpio_flags);
			}

			for(i = 0; i < st->state_gpio_count; i++)
			{
				st->state_gpio[i] = of_get_named_gpio_flags( np, "state-gpio", i, &(st->state_gpio_flags[i]));
			}
		}

		st->enable_gpio = of_get_named_gpio( np, "enable-gpio", 0);
		if (!gpio_is_valid(st->enable_gpio)) {
			dev_warn(&spi->dev, "failed to get enable gpio\n");
			st->enable_gpio = 0;
		}else
		{
			int error = devm_gpio_request_one(&spi->dev, st->enable_gpio, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "ltc1655 enable");
			if (error) {
				dev_err(&spi->dev, "request of gpio %d failed, %d\n",
					st->enable_gpio, error);
				st->enable_gpio = 0;
			}
		}
	}


	spi_set_drvdata(spi, indio_dev);
	st->chip_info = &ltc1655_chip_info;

	if (voltage_uv_pos)
		st->vref_max_mv = voltage_uv_pos / 1000;
	else
		st->vref_max_mv = st->chip_info->vref_max_mv;

	if (voltage_uv_neg)
		st->vref_min_mv = voltage_uv_neg / 1000;
	else
		st->vref_min_mv = st->chip_info->vref_min_mv;

	st->us = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ltc1655_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = 1;

	if(1 != ltc1655_check_communication( st))
		goto error_disable_reg_neg;

	if( ltc1655_write_raw( indio_dev, &ltc1655_channel, 0, 0, IIO_CHAN_INFO_RAW))
		goto error_disable_reg_neg;

	if(1 != ltc1655_getState( st))
		goto error_disable_reg_neg;


	// Finally enable the output
	if( st->enable_gpio)
	{
		msleep(10);
		gpio_set_value(st->enable_gpio, 1);
	}
	

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg_neg;

	return 0;

error_disable_reg_neg:
	if(st->state_gpio) devm_kfree( &spi->dev, st->state_gpio);
	if(st->state_gpio_flags) devm_kfree( &spi->dev, st->state_gpio_flags);

	if (!IS_ERR(st->reg_neg))
		regulator_disable(st->reg_neg);
error_put_reg_neg:
	if (!IS_ERR(st->reg_neg))
		regulator_put(st->reg_neg);
error_disable_reg_pos:
	if (!IS_ERR(st->reg_pos))
		regulator_disable(st->reg_pos);
error_put_reg_pos:
	if (!IS_ERR(st->reg_pos))
		regulator_put(st->reg_pos);
	iio_device_free(indio_dev);
error_ret:
	dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
	return ret;
}

static int ltc1655_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ltc1655_state *st = iio_priv(indio_dev);

	// Finally disable the output
	if( st->enable_gpio)
	{
		gpio_set_value(st->enable_gpio, 0);
		devm_gpio_free(&spi->dev, st->enable_gpio);
	}

	if(st->state_gpio) devm_kfree( &spi->dev, st->state_gpio);
	if(st->state_gpio_flags) devm_kfree( &spi->dev, st->state_gpio_flags);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg_pos)) {
		regulator_disable(st->reg_pos);
		regulator_put(st->reg_pos);
	}
	iio_device_free(indio_dev);

	return 0;
}

static const struct spi_device_id ltc1655_id[] = {
	{"ltc1655", ID_LTC1655},
	{}
};
MODULE_DEVICE_TABLE(spi, ltc1655_id);

static struct spi_driver ltc1655_driver = {
	.driver = {
		   .name = "ltc1655",
		   .owner = THIS_MODULE,
		   },
	.probe = ltc1655_probe,
	.remove = ltc1655_remove,
	.id_table = ltc1655_id,
};
module_spi_driver(ltc1655_driver);

MODULE_AUTHOR("Jonas Hoeppner <jonas.hoeppner@garz-fricke.com>");
MODULE_DESCRIPTION("LTC1655 16bit DAC spi driver");
MODULE_LICENSE("GPL v2");
