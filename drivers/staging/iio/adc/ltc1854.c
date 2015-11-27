/*
 * LTC1854/LTC1855/LTC1856 SPI ADC driver
 *
 * Copyright 2013 Clemens Terasa
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "ltc1854.h"

#define LTC1854_REG_SLEEP_BIT	(1 << 0)
#define LTC1854_REG_NAP_BIT	(1 << 1)
#define LTC1854_REG_GAIN_BIT (1 << 2)		// ltc1857 only
#define LTC1854_REG_UNI_BIT	(1 << 3)		// ltc1857 only
#define LTC1854_REG_ODD_BIT	(1 << 6)
#define LTC1854_REG_SGL_BIT	(1 << 7)
#define RES_MASK(bits)		((1 << (bits)) - 1)

struct ltc1854_state;
enum ltc1854_regulator {
	LTC1854_AVDD,
	LTC1854_OVDD,
	LTC1854_DVDD
};

static const char * const ltc1854_regulator_name[] = {
	[LTC1854_AVDD] = "Avdd",
	[LTC1854_OVDD] = "Ovdd",
	[LTC1854_DVDD] = "Dvdd",
};

struct ltc1854_chip_info {
	struct iio_chan_spec		channel_template;
	unsigned int			num_channels;
};

struct ltc1854_state {
	struct spi_device		*spi;
	const struct ltc1854_chip_info	*chip_info;
	struct regulator		*reg[3];
	unsigned short			convst_pin;
	unsigned short			busy_neg_pin;
	struct spi_transfer		xfer;
	struct spi_message		msg;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * Make the buffer large enough for one 16 bit sample and one 64 bit
	 * aligned 64 bit timestamp.
	 */
	unsigned char rx_data[ALIGN(2, sizeof(s64)) + sizeof(s64)]
			____cacheline_aligned;
	unsigned char tx_data[ALIGN(2, sizeof(s64))]
			____cacheline_aligned;
};

enum ltc1854_supported_device_ids {
	ID_LTC1854,
	ID_LTC1855,
	ID_LTC1856
};

static int ltc1854_scan_direct(struct ltc1854_state *st,
		struct iio_chan_spec const *chan)
{
	int ret;

	/* FIXME: 
	 * ltc1857 works well with this driver but has an input range selector. 
	 * The "UNI" and "GAIN" bit determine the input range - on ltc1854 they are "DONT CARE".
	 * So these bits can be safely set to select an input range of +-10 V on ltc1857.
	 */
	st->tx_data[0] = chan->address | LTC1854_REG_GAIN_BIT;
	st->tx_data[1] = 0;
	
	/* Busy waiting in case of busy ADC (unlikely) */
	while (!gpio_get_value(st->busy_neg_pin))
		cpu_relax();

	/* Simulate Mode 1 from LTC1854 spec */
	gpio_set_value(st->convst_pin, 0);
	ret = spi_sync(st->spi, &st->msg);
	gpio_set_value(st->convst_pin, 1);
	if (ret)
		return ret;
	udelay(4);

	/* Busy waiting in case of busy ADC (unlikely) */
	while (!gpio_get_value(st->busy_neg_pin))
		cpu_relax();

	st->tx_data[0] |= LTC1854_REG_NAP_BIT;
	gpio_set_value(st->convst_pin, 0);
	ret = spi_sync(st->spi, &st->msg);
	gpio_set_value(st->convst_pin, 1);
	if (ret)
		return ret;

	return be16_to_cpup((__be16 *)st->rx_data);
}

static int ltc1854_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;
	struct ltc1854_state *st = iio_priv(indio_dev);
	
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
			ret = ltc1854_scan_direct(st, chan);
		mutex_unlock(&indio_dev->mlock);

		if (ret < 0)
			return ret;
		*val = (ret >> chan->scan_type.shift) &
			RES_MASK(chan->scan_type.realbits);
		*val = sign_extend32(*val, chan->scan_type.realbits - 1);		
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* ADC with constant internal Vref: +-10 V*/
		*val =  20;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

#define LTC1854_CHAN(bits) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 0,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE)|	\
		BIT(IIO_CHAN_INFO_CALIBSCALE), 	\
	.scan_type = IIO_ST('s', (bits), 16, 16 - (bits)),	\
}								\

static int ltc1853_alloc_channels(struct iio_dev *indio_dev)
{
	struct ltc1854_state *st = iio_priv(indio_dev);
	int i;
	const struct ltc1854_chip_info *ci = st->chip_info;
	struct iio_chan_spec *channels;


	/* Also consider the differential channels */
	indio_dev->num_channels = 2 * ci->num_channels;
	channels = kcalloc(indio_dev->num_channels,
			   sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (IS_ERR_OR_NULL(channels))
		return -ENOMEM;

	/* Allocate the non-differential channels in ascending order first */
	for (i = 0; i < ci->num_channels; i++) {
		channels[i] = ci->channel_template;
		channels[i].channel = i;
		/* Address part in the first tx byte*/
		channels[i].address = LTC1854_REG_SGL_BIT |
			(i % 2) * LTC1854_REG_ODD_BIT |
			(i / 2) << 4;
		channels[i].scan_index = i;
	}

	/* Allocate the differential channels */
	for (i = 0; i < ci->num_channels; i++) {
		const unsigned int idx = i + ci->num_channels;
		channels[idx] = ci->channel_template;
		channels[idx].differential = 1;
		channels[idx].channel = i;
		/* Sequence: 1, 0, 3, 2, 5, 4, ... */
		channels[idx].channel2 = (i + 1) - (2 * (i % 2));
		/* Address part in the first tx byte*/
		channels[idx].address =	(i % 2) * LTC1854_REG_ODD_BIT |
			(i / 2)	<< 4;
		channels[idx].scan_index = idx;
	}

	indio_dev->channels = channels;

	return 0;
}

static const struct ltc1854_chip_info ltc1854_chip_info_tbl[] = {
	[ID_LTC1854] = {
		.channel_template = LTC1854_CHAN(12),
		.num_channels = 8,
	},
	[ID_LTC1855] = {
		.channel_template = LTC1854_CHAN(14),
		.num_channels = 8,
	},
	[ID_LTC1856] = {
		.channel_template = LTC1854_CHAN(16),
		.num_channels = 8,
	},
};

static const struct iio_info ltc1854_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ltc1854_read_raw,
};


static void ltc1854_regulators_free(struct ltc1854_state *st)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(st->reg); i++) {
		if (!IS_ERR_OR_NULL(st->reg[i])) {
			regulator_disable(st->reg[i]);
			regulator_put(st->reg[i]);
		}
		st->reg[i] = NULL;
	}
}

static int ltc1854_regulators_init(struct ltc1854_state *st)
{
	int i;
	int ret;
	for (i = 0; i < ARRAY_SIZE(st->reg); i++) {
		st->reg[i] = regulator_get(&st->spi->dev,
				ltc1854_regulator_name[i]);
		if (IS_ERR_OR_NULL(st->reg[i])) {
			dev_info(&st->spi->dev,
				 "Regulator %s not found.\n",
				 ltc1854_regulator_name[i]);
			st->reg[i] = NULL;
			continue;
		}

		ret = regulator_enable(st->reg[i]);
		if (ret) {
			dev_err(&st->spi->dev,
				"Failed to enable regulator %s.\n",
				ltc1854_regulator_name[i]);
			goto error_reg_en;
		}
	}
	return 0;
error_reg_en:
	ltc1854_regulators_free(st);
	return ret;
}

/**
 * ltc1854_trigger_handler - the trigger handler function
 * @irq: the interrupt number
 * @p: private data - always a pointer to the poll func.
 *
 * Based on the trigger handler in iio_simple_dummy_buffer.c
 */
static irqreturn_t ltc1854_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ltc1854_state *st = iio_priv(indio_dev);
	int len = 0;
	s16 *data;

	data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (data == NULL)
		goto done;

	if (!bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength)) {
		int i, j;
		for (i = 0, j = 0;
		     i < bitmap_weight(indio_dev->active_scan_mask,
				       indio_dev->masklength);
		     i++, j++) {
			j = find_next_bit(indio_dev->active_scan_mask,
					  indio_dev->masklength, j);

			mutex_lock(&indio_dev->mlock);
			data[i] = ltc1854_scan_direct(st,
					&indio_dev->channels[j]);
			mutex_unlock(&indio_dev->mlock);
			len += sizeof(*data);
		}
	}
	/* Store the timestamp at an 8 byte aligned offset */
	if (indio_dev->scan_timestamp)
		*(s64 *)((u8 *)data + ALIGN(len, sizeof(s64)))
			= iio_get_time_ns();
	iio_push_to_buffers(indio_dev, (u8 *)data);

	kfree(data);

done:
	/*
	 * Tell the core we are done with this trigger and ready for the
	 * next one.
	 */
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ltc1854_probe(struct spi_device *spi)
{
	struct ltc1854_state *st;
	struct iio_dev *indio_dev;
	int ret;
	struct ltc1854_platform_data *ltc1854_data;

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);
	if (IS_ERR_OR_NULL(spi->dev.platform_data)) {
		dev_err(&spi->dev, "No platform data supplied.\n");
		ret = -EINVAL;
		goto error_free_dev;
	}

	st->chip_info =
		&ltc1854_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	st->spi = spi;
	/* Using mode 1 is faster than mode 0 (LTC1854 spec p. 5 note 13) */
	spi->mode = SPI_MODE_1;

	ltc1854_data = spi->dev.platform_data;
	st->convst_pin = ltc1854_data->convst_pin;
	st->busy_neg_pin = ltc1854_data->busy_neg_pin;

	ret = gpio_request(st->convst_pin,
			spi_get_device_id(spi)->name);
	if (ret) {
		dev_err(&spi->dev, "Fail to request convst gpio PIN %d.\n",
			st->convst_pin);
		goto error_free_dev;
	}
	gpio_direction_output(st->convst_pin, 1);

	ret = gpio_request(st->busy_neg_pin,
			spi_get_device_id(spi)->name);
	if (ret) {
		dev_err(&spi->dev, "Fail to request busy_neg gpio PIN %d.\n",
			st->busy_neg_pin);
		goto error_free_gpio_convst;
	}
	gpio_direction_input(st->busy_neg_pin);


	/* Establish that the iio_dev is a child of the spi device */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc1854_info;

	ret = ltc1853_alloc_channels(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to allocate channels.\n");
		goto error_free_gpio_busy_neg;
	}

	ret = ltc1854_regulators_init(st);
	if (ret) {
		dev_err(&spi->dev, "Failed to activate regulators.\n");
		goto error_free_channels;
	}

	/* Setup default message */
	st->xfer.tx_buf = &st->tx_data;
	st->xfer.rx_buf = &st->rx_data;
	st->xfer.len = st->chip_info->channel_template
		.scan_type.storagebits / 8;
	spi_message_init(&st->msg);
	spi_message_add_tail(&st->xfer, &st->msg);

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
			&ltc1854_trigger_handler, NULL);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup triggered buffer.\n");
		goto error_free_reg;
	}


	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device.\n");
		goto error_free_buffer;
	}
	dev_info(&spi->dev, "Registered as iio device: %d\n", indio_dev->id);
	return 0;

error_free_buffer:
	iio_triggered_buffer_cleanup(indio_dev);
error_free_reg:
	ltc1854_regulators_free(st);
error_free_channels:
	kfree(indio_dev->channels);
error_free_gpio_busy_neg:
	gpio_free(st->busy_neg_pin);
error_free_gpio_convst:
	gpio_free(st->convst_pin);
error_free_dev:
	iio_device_free(indio_dev);
error_ret:
	dev_err(&spi->dev, "Probing failed: %d\n", ret);
	return ret;
}

static int ltc1854_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ltc1854_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	ltc1854_regulators_free(st);
	kfree(indio_dev->channels);
	gpio_free(st->busy_neg_pin);
	gpio_free(st->convst_pin);
	iio_device_free(indio_dev);

	return 0;
}

static const struct spi_device_id ltc1854_id[] = {
	{"ltc1854", ID_LTC1854},
	{"ltc1855", ID_LTC1855},
	{"ltc1856", ID_LTC1856},
	{}
};
MODULE_DEVICE_TABLE(spi, ltc1854_id);

static struct spi_driver ltc1854_driver = {
	.driver = {
		.name	= "ltc1854",
		.owner	= THIS_MODULE,
	},
	.probe		= ltc1854_probe,
	.remove		= ltc1854_remove,
	.id_table	= ltc1854_id,
};
module_spi_driver(ltc1854_driver);

MODULE_AUTHOR("Clemens Terasa <BLmadman@gmx.de>");
MODULE_DESCRIPTION("Linear Technology LTC1854/LTC1855/LTC1856 8-Channel ADC");
MODULE_LICENSE("GPL v2");
