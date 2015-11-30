/*
 * Touch Screen driver for SiS 9200 family I2C Touch panels
 *
 * Copyright (C) 2015 SiS, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <asm/unaligned.h>
#include <linux/input/mt.h>
#include <linux/crc-itu-t.h>
#include <linux/delay.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define SIS_I2C_NAME					"sis_i2c_ts"
#define MAX_FINGERS						10

/*Resolution mode*/
/*Constant value*/
#define SIS_MAX_X						4095
#define SIS_MAX_Y						4095

#define PACKET_BUFFER_SIZE				128

#define SIS_CMD_NORMAL					0x0

/* for new i2c format */
#define TOUCHDOWN						0x3
#define TOUCHUP							0x0
#define MAX_BYTE						64
#define PRESSURE_MAX					255

/*Resolution diagonal */
#define AREA_LENGTH_LONGER				5792
/*((SIS_MAX_X^2) + (SIS_MAX_Y^2))^0.5*/
#define AREA_LENGTH_SHORT				5792
#define AREA_UNIT						(5792/32)

#define P_BYTECOUNT						0
#define ALL_IN_ONE_PACKAGE				0x10
#define IS_TOUCH(x)						(x & 0x1)
#define IS_HIDI2C(x)					((x & 0xF) == 0x06)
#define IS_AREA(x)						((x >> 4) & 0x1)
#define IS_PRESSURE(x)				    ((x >> 5) & 0x1)
#define IS_SCANTIME(x)			        ((x >> 6) & 0x1)
#define NORMAL_LEN_PER_POINT			6
#define AREA_LEN_PER_POINT				2
#define PRESSURE_LEN_PER_POINT			1

#define TOUCH_FORMAT					0x1
#define HIDI2C_FORMAT					0x6
#define P_REPORT_ID						2
#define BYTE_BYTECOUNT					2
#define BYTE_ReportID					1
#define BYTE_CRC_HIDI2C					0
#define BYTE_CRC_I2C					2
#define BYTE_SCANTIME					2

#define MAX_SLOTS						15

struct sis_slot {
	int check_id;
	int id;
	unsigned short x, y;
	u16 pressure;
	u16 width;
	u16 height;
};

struct point {
	int id;
	unsigned short x, y;
	u16 pressure;
	u16 width;
	u16 height;
};

struct sistp_driver_data {
	int id;
	int fingers;
	struct point pt[MAX_FINGERS];
};

struct sis_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct sistp_driver_data *tp_info;
	int reset_gpio;
};

static void __devinit sis_ts_reset(struct i2c_client *client,
						struct sis_ts_data *ts)
{
	/* FIXME: I didn't find a reset pulse timing
	          specification */
	gpio_set_value(ts->reset_gpio, 0);
	udelay(100);
	gpio_set_value(ts->reset_gpio, 1);
}

/* Addresses to scan */
static void sis_tpinfo_clear(struct sistp_driver_data *tp_info, int max);

static int sis_cul_unit(u8 report_id)
{
	int ret = NORMAL_LEN_PER_POINT;

	if (report_id != ALL_IN_ONE_PACKAGE) {
		if (IS_AREA(report_id))
			ret += AREA_LEN_PER_POINT;
		if (IS_PRESSURE(report_id))
			ret += PRESSURE_LEN_PER_POINT;
	}

	return ret;
}

static int sis_readpacket(struct i2c_client *client, u8 cmd, u8 *buf)
{
	u8 tmpbuf[MAX_BYTE] = {0};
	int ret;
	int touchnum = 0;
	int p_count = 0;
	int touch_format_id = 0;
	int locate = 0;
	bool read_first = true;
	/*
	 * New i2c format
	 * buf[0] = Low 8 bits of byte count value
	 * buf[1] = High 8 bits of byte count value
	 * buf[2] = Report ID
	 * buf[touch num * 6 + 2 ] = Touch information;
	 * 1 touch point has 6 bytes, it could be none if no touch
	 * buf[touch num * 6 + 3] = Touch numbers
	 *
	 * One touch point information include 6 bytes, the order is
	 *
	 * 1. status = touch down or touch up
	 * 2. id = finger id
	 * 3. x axis low 8 bits
	 * 4. x axis high 8 bits
	 * 5. y axis low 8 bits
	 * 6. y axis high 8 bits
	 */
	do {
		if (locate >= PACKET_BUFFER_SIZE) {
			dev_err(&client->dev, "%s: Buf Overflow\n", __func__);
			return -EPERM;
		}
		ret = i2c_master_recv(client, tmpbuf, MAX_BYTE);

		if (ret < 0) {
			dev_err(&client->dev, "%s: i2c transfer error\n", __func__);
			return ret;
		}
		/* error package length of receiving data */
		else if (tmpbuf[P_BYTECOUNT] > MAX_BYTE) {
			dev_err(&client->dev, "%s: Error Bytecount\n", __func__);
			return -EPERM;
		}
		if (read_first) {
			/* access NO TOUCH event unless BUTTON NO TOUCH event */
			if (tmpbuf[P_BYTECOUNT] == 0)
				return 0;	/* touchnum is 0 */
		}
		
		/*
		 * skip parsing data when two devices are registered
		 * at the same slave address
		 * parsing data when P_REPORT_ID && 0xf is TOUCH_FORMAT
		 * or P_REPORT_ID is ALL_IN_ONE_PACKAGE
		 */
		touch_format_id = tmpbuf[P_REPORT_ID] & 0xf;
		if ((touch_format_id != TOUCH_FORMAT) &&
			(touch_format_id != HIDI2C_FORMAT) &&
			(tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE)) {
			dev_err(&client->dev, "sis_readpacket: Error Report_ID\n");
			return -EPERM;
		}
		p_count = (int) tmpbuf[P_BYTECOUNT] - 1;	/* start from 0 */
		if (tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE) {
			if (IS_TOUCH(tmpbuf[P_REPORT_ID])) {
				/* delete 2 byte crc */
				p_count -= BYTE_CRC_I2C;
			} else if (IS_HIDI2C(tmpbuf[P_REPORT_ID])) {
				p_count -= BYTE_CRC_HIDI2C;
			} else {	/* should not be happen */
				dev_err(&client->dev, "%s: delete crc error\n", __func__);
				return -EPERM;
			}
			if (IS_SCANTIME(tmpbuf[P_REPORT_ID]))
				p_count -= BYTE_SCANTIME;
		}
		/* For ALL_IN_ONE_PACKAGE */
		if (read_first) {
			touchnum = tmpbuf[p_count];
		} else {
			if (tmpbuf[p_count] != 0) {
				dev_err(&client->dev, "%s: get error package\n", __func__);
				return -EPERM;
			}
		}

		if ((touch_format_id != HIDI2C_FORMAT) &&
			(tmpbuf[P_BYTECOUNT] > 3)) {
			int crc_end = p_count + (IS_SCANTIME(
				tmpbuf[P_REPORT_ID]) * 2);
			u16 buf_crc = crc_itu_t(
				0, tmpbuf + 2, crc_end - 1);
			int l_package_crc = (IS_SCANTIME(
				tmpbuf[P_REPORT_ID]) * 2) + p_count + 1;
			u16 package_crc = get_unaligned_le16(
				&tmpbuf[l_package_crc]);

			if (buf_crc != package_crc) {
				dev_err(&client->dev, "%s: CRC Error\n", __func__);
				return -EPERM;
			}
		}

		memcpy(&buf[locate], &tmpbuf[0], MAX_BYTE);
		/* Buf_Data [0~63] [64~128] */
		locate += MAX_BYTE;
		read_first = false;
	} while (tmpbuf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE &&
			tmpbuf[p_count] > 5);
	return touchnum;
}

static int sis_parse_i2c_event(u8 *buf, u8 fingers,
			       struct sistp_driver_data *tp_info,
			       int *check_id, struct sis_slot *sisdata)
{
	int point_unit;
	u8 i = 0, pstatus = 0;
	u8 px = 0, py = 0;
	u8 p_area = 0;
	u8 p_preasure = 0;
	
	/* Parser and Get the sis data */
	point_unit = sis_cul_unit(buf[P_REPORT_ID]);
	tp_info->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);

	/* fingers 10 =  0 ~ 9 */
	for (i = 0; i < fingers; i++) {
		if ((buf[P_REPORT_ID] != ALL_IN_ONE_PACKAGE) && (i >= 5)) {
			/*Calc point status*/
			pstatus = BYTE_BYTECOUNT + BYTE_ReportID
					+ ((i - 5) * point_unit);
			pstatus += 64;
		} else {
			pstatus = BYTE_BYTECOUNT + BYTE_ReportID
					+ (i * point_unit);
					/* Calc point status */
		}
		px = pstatus + 2;	/* Calc point x_coord */
		py = px + 2;	/* Calc point y_coord */
		if ((buf[pstatus]) == TOUCHUP) {
			tp_info->pt[i].width = 0;
			tp_info->pt[i].height = 0;
			tp_info->pt[i].pressure = 0;
		} else if (buf[P_REPORT_ID] == ALL_IN_ONE_PACKAGE
					&& (buf[pstatus]) == TOUCHDOWN) {
			tp_info->pt[i].width = 1;
			tp_info->pt[i].height = 1;
			tp_info->pt[i].pressure = 1;
		} else if ((buf[pstatus]) == TOUCHDOWN) {
			p_area = py + 2;
			p_preasure = py + 2 + (IS_AREA(buf[P_REPORT_ID]) * 2);
			/* area */
			if (IS_AREA(buf[P_REPORT_ID])) {
				tp_info->pt[i].width = buf[p_area];
				tp_info->pt[i].height = buf[p_area + 1];
			} else {
				tp_info->pt[i].width = 1;
				tp_info->pt[i].height = 1;
			}
			/* pressure */
			if (IS_PRESSURE(buf[P_REPORT_ID]))
				tp_info->pt[i].pressure = (buf[p_preasure]);
			else
				tp_info->pt[i].pressure = 1;
		} else
			return -EPERM;

		tp_info->pt[i].id = (buf[pstatus + 1]);
		tp_info->pt[i].x = get_unaligned_le16(&buf[px]);
		tp_info->pt[i].y = get_unaligned_le16(&buf[py]);

		check_id[tp_info->pt[i].id] = 1;
		sisdata[tp_info->pt[i].id].id = tp_info->pt[i].id;
		sisdata[tp_info->pt[i].id].pressure = tp_info->pt[i].pressure;
		sisdata[tp_info->pt[i].id].x = tp_info->pt[i].x;
		sisdata[tp_info->pt[i].id].y = tp_info->pt[i].y;
		sisdata[tp_info->pt[i].id].width = tp_info->pt[i].width
						* AREA_UNIT;
		sisdata[tp_info->pt[i].id].height = tp_info->pt[i].height
						* AREA_UNIT;
	}
	return 0;
}

static irqreturn_t sis_ts_irq_handler(int irq, void *dev_id)
{
	struct sis_ts_data *ts = dev_id;
	struct sistp_driver_data *tp_info = ts->tp_info;
	int ret;
	u8 buf[PACKET_BUFFER_SIZE] = {0};
	u8 i = 0, fingers = 0;
	int check_id[MAX_FINGERS];
	static int pre_check_id[MAX_FINGERS];
	struct sis_slot sisdata[MAX_FINGERS];

	memset(check_id, 0, sizeof(int)*MAX_FINGERS);

	/*I2C or SMBUS block data read*/
	ret = sis_readpacket(ts->client, SIS_CMD_NORMAL, buf);
	/*Error Number*/
	if (ret < 0)
		goto out;
	/*access NO TOUCH event unless BUTTON NO TOUCH event*/
	else if (ret == 0) {
		fingers = 0;
		sis_tpinfo_clear(tp_info, MAX_FINGERS);
		goto type_b_report;
	}

	sis_tpinfo_clear(tp_info, MAX_FINGERS);
	fingers = ret;

	ret = sis_parse_i2c_event(buf, fingers, tp_info, check_id, sisdata);
	if (ret < 0)
		goto out;

type_b_report:
	for (i = 0; i < MAX_FINGERS; i++) {
		if ((check_id[i] != pre_check_id[i]) && (check_id[i] != 1))
			check_id[i] = -1;
	}

	for (i = 0; i < MAX_FINGERS; i++) {
		if (check_id[i] == 1) {
			input_mt_slot(ts->input_dev, i+1);
			if (sisdata[i].pressure > 0) {
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, true);
				input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE, sisdata[i].pressure);
				input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MAJOR, sisdata[i].width);
				input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MINOR, sisdata[i].height);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, sisdata[i].x);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, sisdata[i].y);
			} else {
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, false);
				check_id[i] = 0;			}
		} else if (check_id[i] == -1) {
			input_mt_slot(ts->input_dev, i+1);
			input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, false);
			check_id[i] = 0;
		}
		pre_check_id[i] = check_id[i];
	}

	input_sync(ts->input_dev);

out:
	return IRQ_HANDLED;
}

static void sis_tpinfo_clear(struct sistp_driver_data *tp_info, int max)
{
	int i = 0;

	for (i = 0; i < max; i++) {
		tp_info->pt[i].id = -1;
		tp_info->pt[i].x = 0;
		tp_info->pt[i].y = 0;
		tp_info->pt[i].pressure = 0;
		tp_info->pt[i].width = 0;
	}
	tp_info->id = 0x0;
	tp_info->fingers = 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sis_ts_of_match[] = {
	{ .compatible = "sis,sis_ts", },
	{ },
};
MODULE_DEVICE_TABLE(of, sis_ts_of_match);

static int __devinit sis_ts_probe_dt(struct i2c_client *client,
						struct sis_ts_data *ts)
{
	int error;
	struct device_node *np;
	const struct of_device_id *of_id;
	np = client->dev.of_node;
	int irq_gpio;
	
	if( !np)
		return -ENODEV;

	of_id = of_match_device(sis_ts_of_match, &client->dev);
	dev_dbg(&client->dev, "%s np: 0x%08x, of_id: 0x%08x\n", __func__,
				(unsigned int)np, (unsigned int)of_id);

	ts->reset_gpio = of_get_named_gpio( np, "reset-gpio", 0);
	if(ts->reset_gpio < 0) {
		dev_err(&client->dev, "get reset gpio from dt failed");
	   	return -EINVAL;
	}
	dev_dbg(&client->dev, "reset-gpio: %d\n", ts->reset_gpio);

	/* Hold controller in reset on start up */
	/* FIXME: Datasheet: POR time 5ms, p. 14, we assume here that 5ms is always
	   elapsed instead adding an addistional msleep */
	error = devm_gpio_request_one(&client->dev, ts->reset_gpio,
				GPIOF_OUT_INIT_LOW, "sis ts reset");
	if (error < 0) {
		dev_err(&client->dev, "gpio request for irq failed");
		return error;
	}

	irq_gpio = of_get_named_gpio( np, "irq-gpio", 0);
	if(irq_gpio < 0) {
		dev_err(&client->dev, "get irq gpio from dt failed");
		return -EINVAL;
	}
	dev_dbg(&client->dev, "irq-gpio: %d\n", irq_gpio);

	error = devm_gpio_request_one(&client->dev, irq_gpio,
				GPIOF_IN, "sis ts irq");
	if (error < 0) {
		dev_err(&client->dev, "gpio request for irq failed");
		return error;
	}
    
	client->irq = gpio_to_irq(irq_gpio);
	
	return 0;
}
#endif

static int sis_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct sis_ts_data *ts;

	dev_dbg(&client->dev, "%s\n", __func__);
	ts = devm_kzalloc(&client->dev, sizeof(struct sis_ts_data),
			  GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->tp_info = devm_kzalloc(&client->dev,
				   sizeof(struct sistp_driver_data),
				   GFP_KERNEL);
	if (!ts->tp_info) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	
#ifdef CONFIG_OF
	err = sis_ts_probe_dt(client, ts);
	if( err < 0)
	{
		dev_err(&client->dev,"  Error in device tree\n");
		return -EINVAL;
	}
	sis_ts_reset(client, ts);
#else
	err = 1;
#endif

	/* 1. Init necessary buffers */
	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* 2. Allocate input device */
	ts->input_dev = devm_input_allocate_device(&client->dev);
	if (!ts->input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}
	
	ts->input_dev->name = "sis_touch";

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
						0, PRESSURE_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
						0, AREA_LENGTH_LONGER, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR,
						0, AREA_LENGTH_SHORT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
						0, SIS_MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
						0, SIS_MAX_Y, 0, 0);
	input_mt_init_slots(ts->input_dev, MAX_SLOTS, INPUT_MT_DIRECT);

	/* add for touch keys */
	set_bit(KEY_COMPOSE, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	
	/* 3. Register input device to core */
	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: Unable to register %s input device\n", __func__,
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	/* 4. irq setup */
	err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					sis_ts_irq_handler,
					IRQF_TRIGGER_FALLING, client->name,
					ts);
	if (err < 0) {
		dev_err(&client->dev, " %s: Failed to request touchscreen IRQ\n", __func__);
		goto err_request_threaded_irq;	
	}

	return 0;
err_request_threaded_irq:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	kfree(ts->tp_info);
	kfree(ts);
err_alloc_data_failed:
	return err;
}

static int sis_ts_remove(struct i2c_client *client) {
	struct sis_ts_data *ts = i2c_get_clientdata(client);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts->tp_info);
	kfree(ts);
	return 0;
}

static int __maybe_unused sis_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
 
	disable_irq(client->irq);
	return 0;
}

static int __maybe_unused sis_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(sis_ts_pm, sis_ts_suspend, sis_ts_resume);

static const struct i2c_device_id sis_ts_id[] = {
	{ SIS_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sis_ts_id);

static struct i2c_driver sis_ts_driver = {
	.driver = {
		.name = SIS_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &sis_ts_pm,
	},
	.probe		= sis_ts_probe,
	.remove		= sis_ts_remove,
	.id_table	= sis_ts_id,
};
module_i2c_driver(sis_ts_driver);

MODULE_DESCRIPTION("SiS 9200 Family Touchscreen Driver"); 
MODULE_LICENSE("GPL v2");
