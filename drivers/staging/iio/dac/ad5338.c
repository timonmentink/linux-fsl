/*
 * Analog devices AD5337, AD5338, AD5339
 * Dual-Channel Digital to Analog Converters driver
 *
 * Copyright 2013 Clemens Terasa
 * Based on the AD5380 DAC driver
 *
 * Licensed under the GPL-2.
 */
#define DEBUG

#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AD5338_CLR_BIT		(1 << 13)

#define AD5338_PWR_DOWN_MODE_OFFSET	14
#define AD5338_PWR_DOWN_MODE_MASK	(3 << AD5338_PWR_DOWN_MODE_OFFSET)

/**
 * struct ad5338_chip_info - chip specific information
 * @channel_template:	channel specification template
 * @num_channels:	number of channels
*/

struct ad5338_chip_info {
	struct iio_chan_spec	channel_template;
	unsigned int		num_channels;
};

static const char * const ad5338_powerdown_modes[] = {
	"normal",
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"open_circuit",
};

/**
 * struct ad5338_state - driver instance specific data
 * @regmap:		regmap instance used by the device
 * @chip_info:		chip model specific constants, available modes etc
 * @vref_reg:		vref supply regulator
 */

struct ad5338_state {
	struct regmap			*regmap;
	const struct ad5338_chip_info	*chip_info;
	struct regulator		*vref_reg;
};

enum ad5338_type {
	ID_AD5337,
	ID_AD5338,
	ID_AD5338_1,
	ID_AD5339,
};

static int ad5338_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad5338_state *st = iio_priv(indio_dev);
	unsigned int mode;
	int ret;

	ret = regmap_read(st->regmap, chan->address, &mode);
	if (ret)
		return ret;

	mode &= AD5338_PWR_DOWN_MODE_MASK;
	mode >>= AD5338_PWR_DOWN_MODE_OFFSET;
	return mode;
}

static int ad5338_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad5338_state *st = iio_priv(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, chan->address,
		AD5338_PWR_DOWN_MODE_MASK,
		mode << AD5338_PWR_DOWN_MODE_OFFSET);

	return ret;
}

static const struct iio_enum ad5338_powerdown_mode_enum = {
	.items = ad5338_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5338_powerdown_modes),
	.get = ad5338_get_powerdown_mode,
	.set = ad5338_set_powerdown_mode,
};

static int ad5338_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	const unsigned int max_val = (1 << chan->scan_type.realbits);
	struct ad5338_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val >= max_val || val < 0)
			return -EINVAL;
		return regmap_update_bits(st->regmap,
				chan->address,
				(max_val - 1) << chan->scan_type.shift,
				val << chan->scan_type.shift);
	default:
		break;
	}
	return -EINVAL;
}

static int ad5338_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad5338_state *st = iio_priv(indio_dev);
	int scale_uv;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(st->regmap,
				chan->address,
				val);
		if (ret)
			return ret;
		*val >>= chan->scan_type.shift;
		*val &= (1 << chan->scan_type.realbits) - 1;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		scale_uv = regulator_get_voltage(st->vref_reg);
		if (scale_uv < 0)
			return scale_uv;

		*val =  scale_uv / 1000000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info ad5338_info = {
	.read_raw = ad5338_read_raw,
	.write_raw = ad5338_write_raw,
	.driver_module = THIS_MODULE,
};

static struct iio_chan_spec_ext_info ad5338_ext_info[] = {
	IIO_ENUM("powerdown_mode", true, &ad5338_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", &ad5338_powerdown_mode_enum),
	{ },
};

#define AD5338_CHANNEL(_bits) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_type = IIO_ST('u', (_bits), 16, 12 - (_bits)),	\
	.ext_info = ad5338_ext_info,				\
}

static const struct ad5338_chip_info ad5338_chip_info_tbl[] = {
	[ID_AD5337] = {
		.channel_template = AD5338_CHANNEL(8),
		.num_channels = 2,
	},
	[ID_AD5338] = {
		.channel_template = AD5338_CHANNEL(10),
		.num_channels = 2,
	},
	[ID_AD5338_1] = {
		.channel_template = AD5338_CHANNEL(10),
		.num_channels = 2,
	},
	[ID_AD5339] = {
		.channel_template = AD5338_CHANNEL(12),
		.num_channels = 2,
	},
};

static int ad5338_alloc_channels(struct iio_dev *indio_dev)
{
	struct ad5338_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *channels;
	unsigned int i;

	channels = kcalloc(st->chip_info->num_channels,
			   sizeof(struct iio_chan_spec), GFP_KERNEL);

	if (!channels)
		return -ENOMEM;

	for (i = 0; i < st->chip_info->num_channels; ++i) {
		channels[i] = st->chip_info->channel_template;
		channels[i].channel = i;
		channels[i].address = 1 << i;
	}

	indio_dev->channels = channels;

	return 0;
}

static int ad5338_probe(struct device *dev, struct regmap *regmap,
			enum ad5338_type type, const char *name)
{
	struct iio_dev *indio_dev;
	struct ad5338_state *st;
	int ret;

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(dev, "Failed to allocate iio device\n");
		ret = -ENOMEM;
		goto error_out;
	}

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->chip_info = &ad5338_chip_info_tbl[type];
	st->regmap = regmap;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &ad5338_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = st->chip_info->num_channels;

	ret = ad5338_alloc_channels(indio_dev);
	if (ret) {
		dev_err(dev, "Failed to allocate channel spec: %d\n", ret);
		goto error_free;
	}


	st->vref_reg = regulator_get(dev, "vref");
	if (IS_ERR(st->vref_reg)) {
		dev_err(dev, "Failed to get vref regulators!\n");
		ret = -ENOTSUPP;
		goto error_free;
	}

	ret = regulator_enable(st->vref_reg);
	if (ret) {
		dev_err(dev, "Failed to enable vref regulators: %d\n",
			ret);
		goto error_free_reg;
	}

	ret = regmap_write(st->regmap, 3, 0);
	if (ret) {
		dev_err(dev, "Failed to clear device registers: %d\n", ret);
		goto error_disable_reg;
	}

	ret = regmap_write(st->regmap, 3, AD5338_CLR_BIT);
	if (ret) {
		dev_err(dev, "Failed to set clear bit: %d\n", ret);
		goto error_disable_reg;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(dev, "Failed to register iio device: %d\n", ret);
		goto error_disable_reg;
	}

	dev_info(dev, "Registered as iio device: %d\n", indio_dev->id);
	return 0;

error_disable_reg:
	if (!IS_ERR(st->vref_reg))
		regulator_disable(st->vref_reg);
error_free_reg:
	if (!IS_ERR(st->vref_reg))
		regulator_put(st->vref_reg);

	kfree(indio_dev->channels);
error_free:
	iio_device_free(indio_dev);
error_out:

	return ret;
}

static int ad5338_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5338_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	kfree(indio_dev->channels);

	if (!IS_ERR(st->vref_reg)) {
		regulator_disable(st->vref_reg);
		regulator_put(st->vref_reg);
	}

	iio_device_free(indio_dev);

	return 0;
}

static bool ad5338_reg_false(struct device *dev, unsigned int reg)
{
	return false;
}

static bool ad5338_reg_writeable(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config ad5338_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = 3,
	.cache_type = REGCACHE_RBTREE,

	.volatile_reg = ad5338_reg_false,
	.readable_reg = ad5338_reg_false,
	.writeable_reg = ad5338_reg_writeable,
};

static int ad5338_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(i2c, &ad5338_regmap_config);

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return ad5338_probe(&i2c->dev, regmap, id->driver_data, id->name);
}

static int ad5338_i2c_remove(struct i2c_client *i2c)
{
	return ad5338_remove(&i2c->dev);
}

static const struct i2c_device_id ad5338_i2c_ids[] = {
	{ "ad5337", ID_AD5337 },
	{ "ad5338", ID_AD5338 },
	{ "ad5338-1", ID_AD5338_1 },
	{ "ad5339", ID_AD5339 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad5338_i2c_ids);

static struct i2c_driver ad5338_i2c_driver = {
	.driver = {
		   .name = "ad5338",
		   .owner = THIS_MODULE,
	},
	.probe = ad5338_i2c_probe,
	.remove = ad5338_i2c_remove,
	.id_table = ad5338_i2c_ids,
};

static inline int ad5338_i2c_register_driver(void)
{
	return i2c_add_driver(&ad5338_i2c_driver);
}

static inline void ad5338_i2c_unregister_driver(void)
{
	i2c_del_driver(&ad5338_i2c_driver);
}

static int __init ad5338_init(void)
{
	return ad5338_i2c_register_driver();
}
module_init(ad5338_init);

static void __exit ad5338_exit(void)
{
	ad5338_i2c_unregister_driver();

}
module_exit(ad5338_exit);

MODULE_AUTHOR("Clemens Terasa <BLmadman@gmx.de>");
MODULE_DESCRIPTION("Analog Devices AD5338 DAC");
MODULE_LICENSE("GPL v2");
