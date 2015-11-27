/*
 * LM73 Sensor driver
 * Based on LM75
 *
 * Copyright (C) 2007, CenoSYS (www.cenosys.com).
 * Copyright (C) 2009, Bollore telecom (www.bolloretelecom.eu).
 *
 * Guillaume Ligneul <guillaume.ligneul@gmail.com>
 * Adrien Demarez <adrien.demarez@bolloretelecom.eu>
 * Jeremy Laine <jeremy.laine@bolloretelecom.eu>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
// #include <string.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#endif


/* Addresses scanned */
static const unsigned short normal_i2c[] = { 0x48, 0x49, 0x4a, 0x4c,
					0x4d, 0x4e, I2C_CLIENT_END };

/* LM73 registers */
#define LM73_REG_INPUT		0x00
#define LM73_REG_CONF		0x01
#define LM73_REG_MAX		0x02
#define LM73_REG_MIN		0x03
#define LM73_REG_CTRL		0x04
#define LM73_REG_ID		0x07

#define LM73_ID			0x9001 /* or 0x190 after a swab16() */
#define DRVNAME			"lm73"
#define LM73_TEMP_MIN		(-40)
#define LM73_TEMP_MAX		150

#define LM73_DIGITS_1			(3 + 1)	/* number of digits before the comma */
#define LM73_DIGITS_2			(2 + 1)	/* number of digits after the comma */
#define LM73_BUF_BEGIN_MINUS	0
#define LM73_BUF_BEGIN_PLUS		1
#define LM73_BUF_BEGIN_DIGIT	2

/*-----------------------------------------------------------------------*/

/* temperature alert interrupt */

static irqreturn_t temp_alert_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;

	printk(KERN_DEBUG "lm73 temp1_alrt interrupt\n");

	sysfs_notify(&client->dev.kobj, NULL, "temp1_alrt");
	kobject_uevent(&client->dev.kobj, KOBJ_CHANGE);

	return IRQ_HANDLED;
}

/*-----------------------------------------------------------------------*/

/* show and store functions */

static ssize_t set_temp(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);

	int buf_begin, i;
	char ctemp_1[LM73_DIGITS_1] = { 0 };
	char ctemp_2[LM73_DIGITS_2] = { 0 };
	char *ptemp_temp;
	long ltemp_1, ltemp_2;
	u16 utemp_raw = 0;

	if (*buf == '-')					/* negative temperature */
		buf_begin = LM73_BUF_BEGIN_MINUS;
	else if (*buf == '+')				/* positive temperature */
		buf_begin = LM73_BUF_BEGIN_PLUS;
	else if (*buf > 47 && *buf < 58)	/* positive temperature */
		buf_begin = LM73_BUF_BEGIN_DIGIT;
	else
		return -1;

	if (buf_begin != LM73_BUF_BEGIN_DIGIT)
		buf++;	/* jump to the first digit */

	/* copy every character before the comma from buf to ptemp_1 */
	ptemp_temp = ctemp_1;
	i = 0;
	while (1) {
		if (*buf > 47 && *buf < 58) {

			if (i >= (LM73_DIGITS_1 - 1))
				return -1;

			memcpy(ptemp_temp, buf, sizeof(char));
			ptemp_temp++;
			buf++;
			i++;
		}
		else
			break;
	}
	
	if (strict_strtol(ctemp_1, 10, &ltemp_1) < 0)
		return -1;

	/* is there a comma and more digits? */
	if (*buf == '.' && (*(buf+1) > 47 && *(buf+1) < 58)) {
		buf++;	/* jump to the digit after the comma */
		ptemp_temp = ctemp_2;
		i = 0;
		while (1) {
			if (*buf > 47 && *buf < 58) {

				if (i >= (LM73_DIGITS_2 - 1))
					return -1;

				memcpy(ptemp_temp, buf, sizeof(char));
				ptemp_temp++;
				buf++;
				i++;
			}
			else
				break;
		}
		
		if (strict_strtol(ctemp_2, 10, &ltemp_2) < 0)
			return -1;
	}
	
	utemp_raw = ltemp_1 << 7;
	
	if (ltemp_2 == 25)
		utemp_raw += 0x0020;
	else if (ltemp_2 == 50)
		utemp_raw += 0x0040;
	else if (ltemp_2 == 75)
		utemp_raw += 0x0060;
	
	if (buf_begin == LM73_BUF_BEGIN_MINUS)
		utemp_raw = (~utemp_raw) + 1; /* 2's complement */

	i2c_smbus_write_word_data(client, attr->index, swab16(utemp_raw));
	return count;
}

static ssize_t show_temp(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
	char sign;

	u16 utemp_raw = (u16) (swab16((i2c_smbus_read_word_data(client, attr->index)) & 0x0000FFFF));

	/* this function returns: +/- utemp_1 . utemp_2 */
	u16 utemp_1 = 0;
	u16 utemp_2 = 0;

	if ((utemp_raw & 0x8000) == 0x8000) {	/* negative temperature in 2's complement */
		sign = '-';
		utemp_raw = (~utemp_raw) + 1;
		utemp_1 = (utemp_raw >> 7);
		if ((utemp_raw & 0x0020) == 0x0020)	/* bit 5 -> 0.25 */
			utemp_2 += 25;
		if ((utemp_raw & 0x0040) == 0x0040)	/* bit 6 -> 0.5 */
			utemp_2 += 50;
	}
	else {	/* positive temperature */
		sign = '+';
		utemp_1 = (utemp_raw >> 7);
		if ((utemp_raw & 0x0020) == 0x0020)	/* bit 5 -> 0.25 */
			utemp_2 += 25;
		if ((utemp_raw & 0x0040) == 0x0040)	/* bit 6 -> 0.5 */
			utemp_2 += 50;
	}

	return sprintf(buf, "%c%d.%d\n", sign, utemp_1, utemp_2);
}

static ssize_t reset_alert(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
		
	i2c_smbus_write_byte_data(client, attr->index, 0x48);
	return count;
}

static ssize_t show_alert(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
	u8 uctrl_reg;
	u8 ualrt_stat;

	uctrl_reg = (u8)(i2c_smbus_read_byte_data(client, attr->index));
	ualrt_stat = (uctrl_reg >> 3) & 0x1;
		
	return sprintf(buf, "%d\n", ualrt_stat);;
}

/*-----------------------------------------------------------------------*/

/* sysfs attributes for hwmon */

static SENSOR_DEVICE_ATTR(temp1_max, 0666,
			show_temp, set_temp, LM73_REG_MAX);
static SENSOR_DEVICE_ATTR(temp1_min, 0666,
			show_temp, set_temp, LM73_REG_MIN);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,
			show_temp, NULL, LM73_REG_INPUT);
static SENSOR_DEVICE_ATTR(temp1_alrtrst, 0222,
			NULL, reset_alert, LM73_REG_CONF);
static SENSOR_DEVICE_ATTR(temp1_alrt, S_IRUGO,
			show_alert, NULL, LM73_REG_CTRL);


static struct attribute *lm73_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_alrtrst.dev_attr.attr,
	&sensor_dev_attr_temp1_alrt.dev_attr.attr,
	NULL
};

static const struct attribute_group lm73_group = {
	.attrs = lm73_attributes,
};

/*-----------------------------------------------------------------------*/

/* device probe and removal */

static int
lm73_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *hwmon_dev;
	int status, err;
	int ctrl;

	ctrl = i2c_smbus_read_byte_data(client, LM73_REG_CTRL);
	if (ctrl < 0)
		return ctrl;

	/* Register sysfs hooks */
	status = sysfs_create_group(&client->dev.kobj, &lm73_group);
	if (status)
		return status;

	hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(hwmon_dev)) {
		status = PTR_ERR(hwmon_dev);
		goto exit_remove;
	}
	i2c_set_clientdata(client, hwmon_dev);

	dev_info(&client->dev, "%s: sensor '%s'\n",
		 dev_name(hwmon_dev), client->name);
	
	/* configure temperature alert interrupt if available */
	if(client->irq)
	{
		irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);
		err = request_irq(client->irq, temp_alert_irq_handler, IRQF_SHARED,
			  "lm73 temp alert", client);
		if (err)
			goto exit_remove;
			
		dev_info(&client->dev, "%s: irq = %d \n", dev_name(hwmon_dev), client->irq);
	}

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &lm73_group);
	return status;
}

static int lm73_remove(struct i2c_client *client)
{
	struct device *hwmon_dev = i2c_get_clientdata(client);

	if (client->irq >= 0)
		free_irq(client->irq, client);
	hwmon_device_unregister(hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &lm73_group);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lm73_dt_ids[] = {
	{ .compatible = "national,lm73", 0 },
	{ /* sentinel */ }
};

#endif

static const struct i2c_device_id lm73_ids[] = {
	{ "lm73", 0 },
	{ /* LIST END */ }
};
MODULE_DEVICE_TABLE(i2c, lm73_ids);

/* Return 0 if detection is successful, -ENODEV otherwise */
static int lm73_detect(struct i2c_client *new_client,
			struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;
	int id, ctrl, conf;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	/*
	 * Do as much detection as possible with byte reads first, as word
	 * reads can confuse other devices.
	 */
	ctrl = i2c_smbus_read_byte_data(new_client, LM73_REG_CTRL);
	if (ctrl < 0 || (ctrl & 0x10))
		return -ENODEV;

	conf = i2c_smbus_read_byte_data(new_client, LM73_REG_CONF);
	if (conf < 0 || (conf & 0x0c))
		return -ENODEV;

	id = i2c_smbus_read_byte_data(new_client, LM73_REG_ID);
	if (id < 0 || id != (LM73_ID & 0xff))
		return -ENODEV;

	/* Check device ID */
	id = i2c_smbus_read_word_data(new_client, LM73_REG_ID);
	ctrl = i2c_smbus_read_byte_data(new_client, LM73_REG_CTRL);
	if ((id != LM73_ID) || (ctrl & 0x10))
		return -ENODEV;

	strlcpy(info->type, "lm73", I2C_NAME_SIZE);

	return 0;
}

static struct i2c_driver lm73_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "lm73",
		.of_match_table = lm73_dt_ids,
	},
	.probe		= lm73_probe,
	.remove		= lm73_remove,
	.id_table	= lm73_ids,
	.detect		= lm73_detect,
	.address_list	= normal_i2c,
};

/* module glue */

static int __init sensors_lm73_init(void)
{
	return i2c_add_driver(&lm73_driver);
}

static void __exit sensors_lm73_exit(void)
{
	i2c_del_driver(&lm73_driver);
}

MODULE_AUTHOR("Guillaume Ligneul <guillaume.ligneul@gmail.com>");
MODULE_DESCRIPTION("LM73 driver");
MODULE_LICENSE("GPL");

module_init(sensors_lm73_init);
module_exit(sensors_lm73_exit);
