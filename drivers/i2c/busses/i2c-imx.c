/*
 *	Copyright (C) 2002 Motorola GSG-China
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
 *	USA.
 *
 * Author:
 *	Darius Augulis, Teltonika Inc.
 *
 * Desc.:
 *	Implementation of I2C Adapter/Algorithm Driver
 *	for I2C Bus integrated in Freescale i.MX/MXC processors
 *
 *	Derived from Motorola GSG China I2C example driver
 *
 *  Complete rewrite of the driver core by Marc-Oliver Westerburg, Garz & Fricke
 *  GmbH due to several subtle timing problems.
 *
 *	Copyright (C) 2005 Torsten Koschorrek <koschorrek at synertronixx.de
 *	Copyright (C) 2005 Matthias Blaschke <blaschke at synertronixx.de
 *	Copyright (C) 2007 RightHand Technologies, Inc.
 *	Copyright (C) 2008 Darius Augulis <darius.augulis at teltonika.lt>
 *	Copyright (C) 2012 Marc-Oliver Westerburg <westerburg at garz-fricke.com>
 *
 */

/** Includes *******************************************************************
*******************************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/atomic.h>

#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <linux/pinctrl/consumer.h>
#endif

#include <linux/platform_data/i2c-imx.h>
/* debug */
#define IMX_IIC_DEBUG 0
#if IMX_IIC_DEBUG
#undef dev_dbg
#define dev_dbg(def, format, arg...) printk(format, ## arg)
#endif

//#define CONFIG_I2C_DEBUG_BUS

// some debugging defines that
#undef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_STUCK
#undef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_ERROR
#undef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
#undef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_WAIT

#if ((defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_ERROR) || (defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER) || (defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_WAIT) || (defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_STUCK))
#define GUF_CAM_EN_LED1			(1 * 32 + 2)
#define GUF_I2C_DEBUG_SIGNAL	GUF_CAM_EN_LED1
#endif

/** Defines ********************************************************************
*******************************************************************************/

/* This will be the driver name the kernel reports */
#define DRIVER_NAME "imx-i2c"
#define USE_IRQ

/* Default value */
#define IMX_I2C_BIT_RATE	100000	/* 100kHz */

/* IMX I2C registers */
#define IMX_I2C_IADR	0x00	/* i2c slave address */
#define IMX_I2C_IFDR	0x04	/* i2c frequency divider */
#define IMX_I2C_I2CR	0x08	/* i2c control */
#define IMX_I2C_I2SR	0x0C	/* i2c status */
#define IMX_I2C_I2DR	0x10	/* i2c transfer data */

/* Bits of IMX I2C registers */
#define I2SR_RXAK	0x01
#define I2SR_IIF	0x02
#define I2SR_SRW	0x04
#define I2SR_IAL	0x10
#define I2SR_IBB	0x20
#define I2SR_IAAS	0x40
#define I2SR_ICF	0x80
#define I2CR_RSTA	0x04
#define I2CR_TXAK	0x08
#define I2CR_MTX	0x10
#define I2CR_MSTA	0x20
#define I2CR_IIEN	0x40
#define I2CR_IEN	0x80

/** Variables ******************************************************************
*******************************************************************************/

/*
 * sorted list of clock divider, register value pairs
 * taken from table 26-5, p.26-9, Freescale i.MX
 * Integrated Portable System Processor Reference Manual
 * Document Number: MC9328MXLRM, Rev. 5.1, 06/2007
 *
 * Duplicated divider values removed from list
 */

static u16 __initdata i2c_clk_div[50][2] = {
	{ 22,	0x20 }, { 24,	0x21 }, { 26,	0x22 }, { 28,	0x23 },
	{ 30,	0x00 },	{ 32,	0x24 }, { 36,	0x25 }, { 40,	0x26 },
	{ 42,	0x03 }, { 44,	0x27 },	{ 48,	0x28 }, { 52,	0x05 },
	{ 56,	0x29 }, { 60,	0x06 }, { 64,	0x2A },	{ 72,	0x2B },
	{ 80,	0x2C }, { 88,	0x09 }, { 96,	0x2D }, { 104,	0x0A },
	{ 112,	0x2E }, { 128,	0x2F }, { 144,	0x0C }, { 160,	0x30 },
	{ 192,	0x31 },	{ 224,	0x32 }, { 240,	0x0F }, { 256,	0x33 },
	{ 288,	0x10 }, { 320,	0x34 },	{ 384,	0x35 }, { 448,	0x36 },
	{ 480,	0x13 }, { 512,	0x37 }, { 576,	0x14 },	{ 640,	0x38 },
	{ 768,	0x39 }, { 896,	0x3A }, { 960,	0x17 }, { 1024,	0x3B },
	{ 1152,	0x18 }, { 1280,	0x3C }, { 1536,	0x3D }, { 1792,	0x3E },
	{ 1920,	0x1B },	{ 2048,	0x3F }, { 2304,	0x1C }, { 2560,	0x1D },
	{ 3072,	0x1E }, { 3840,	0x1F }
};

enum imx_i2c_type {
	IMX1_I2C,
	IMX21_I2C,
};

struct imx_i2c_struct {
	struct i2c_adapter				adapter;
	struct imxi2c_platform_data		platform;
	struct resource		*res;
	struct clk			*clk;
	void __iomem		*base;
#ifdef USE_IRQ
	int					irq;
	spinlock_t			lock;
	unsigned long		lock_flags;
#endif
	atomic_t			done;
	struct i2c_msg		*msgs;
	unsigned int		msg_ptr;
	int					msg_res;
	int					msg_idx;
	int					msg_num;

	unsigned int 		disable_delay;
	long				actual_rate;
	int					stopped;
	unsigned int		ifdr; /* IMX_I2C_IFDR */
	enum imx_i2c_type	devtype;

	unsigned int            cur_clk;
	unsigned int            bitrate;
};

static struct platform_device_id imx_i2c_devtype[] = {
	{
		.name = "imx1-i2c",
		.driver_data = IMX1_I2C,
	}, {
		.name = "imx21-i2c",
		.driver_data = IMX21_I2C,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, imx_i2c_devtype);

static const struct of_device_id i2c_imx_dt_ids[] = {
	{ .compatible = "fsl,imx1-i2c", .data = &imx_i2c_devtype[IMX1_I2C], },
	{ .compatible = "fsl,imx21-i2c", .data = &imx_i2c_devtype[IMX21_I2C], },
	{ /* sentinel */ }
};

static inline int is_imx1_i2c(struct imx_i2c_struct *i2c_imx)
{
	return i2c_imx->devtype == IMX1_I2C;
}

/** Functions for IMX I2C adapter driver ***************************************
*******************************************************************************/

#if ((defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_ERROR) || (defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_STUCK))
static void toggle_led(void)
{
	gpio_set_value(GUF_I2C_DEBUG_SIGNAL, 0);
	udelay(50);
	gpio_set_value(GUF_I2C_DEBUG_SIGNAL, 1);
}
#endif

#if ((defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER) || (defined TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_WAIT))
static void set_led(int state)
{
	gpio_set_value(GUF_I2C_DEBUG_SIGNAL, state);
}
#endif

static int i2c_imx_bus_busy(struct imx_i2c_struct *i2c_imx, int for_busy)
{
	unsigned int i;

	dev_dbg(&i2c_imx->adapter.dev, "<%s>\n", __func__);

#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_WAIT
	set_led(0);
#endif

	// we only wait for
	//   - bus-busy immediately after generating a start-condition
	//   - bus-free immediately after generating a stop-condition
	// Normally this should happen within 1 or 2 I2C clock cycles,
	// waiting for 10 cycles therefore should be more than sufficient.
	for (i = 10; i; i--)
	{
		unsigned int temp;

		// read current bus status
		temp = readb(i2c_imx->base + IMX_I2C_I2SR);

		// check for busy/not-busy conditions
		if (for_busy && (temp & I2SR_IBB))
			break;
		if (!for_busy && !(temp & I2SR_IBB))
			break;

		// delay for one I2C-cycle
		udelay(1000000/i2c_imx->actual_rate);	// one I2C clock-cylce
	}

	if (!i)
	{
		dev_dbg(&i2c_imx->adapter.dev,
			"<%s> I2C bus is busy\n", __func__);
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_ERROR
		toggle_led();
#endif
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_WAIT
		set_led(1);
#endif
		return -ETIMEDOUT;
	}


#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_WAIT
	set_led(1);
#endif
	return 0;
}

static int i2c_imx_start(struct imx_i2c_struct *i2c_imx, unsigned int kind)
{
	unsigned int temp = 0;
	int result = 0;

	dev_dbg(&i2c_imx->adapter.dev, "<%s>: stopped=%u kind=0x%x\n", __func__, i2c_imx->stopped, kind);

	// Start I2C transaction
	temp = readb(i2c_imx->base + IMX_I2C_I2CR);
	if (i2c_imx->stopped)
		temp |= I2CR_MSTA;
	else
		temp |= kind;
#ifdef USE_IRQ
	temp |= I2CR_IIEN;
#endif
	writeb(temp, i2c_imx->base + IMX_I2C_I2CR);
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
	set_led(0);
#endif

	// wait until bus is busy
	result = i2c_imx_bus_busy(i2c_imx, 1);
	if (result)
		return result;

	// there is still some setup to be done, if we generate a fresh start condition
	if (kind == I2CR_MSTA)
	{
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
		set_led(1);
#endif
		i2c_imx->stopped = 0;

		temp |= I2CR_MTX | I2CR_TXAK;
		writeb(temp, i2c_imx->base + IMX_I2C_I2CR);
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
		set_led(0);
#endif
	}
	return result;
}

static void i2c_imx_stop(struct imx_i2c_struct *i2c_imx)
{
	unsigned int temp = 0;

	// disable interrupt
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
	set_led(1);
#endif
#ifdef USE_IRQ
	temp = readb(i2c_imx->base + IMX_I2C_I2CR);
	temp &= ~(I2CR_IIEN);
	writeb(temp, i2c_imx->base + IMX_I2C_I2CR);
#endif
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
	set_led(0);
#endif

	if (!i2c_imx->stopped) {
		/* Stop I2C transaction */
		dev_dbg(&i2c_imx->adapter.dev, "<%s>\n", __func__);
		temp = readb(i2c_imx->base + IMX_I2C_I2CR);
		temp &= ~(I2CR_MSTA | I2CR_MTX | I2CR_IIEN);
		writeb(temp, i2c_imx->base + IMX_I2C_I2CR);
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
		set_led(1);
#endif
	}

	if (!i2c_imx->stopped) {
		i2c_imx_bus_busy(i2c_imx, 0);
		i2c_imx->stopped = 1;
	}
}

static void __init i2c_imx_set_clk(struct imx_i2c_struct *i2c_imx )
{
	unsigned int i2c_clk_rate;
	unsigned int div;
	int i;

	/* Divider value calculation */
	i2c_clk_rate = clk_get_rate(i2c_imx->clk);
	pr_debug( "i2c_clk_rate: %d", i2c_clk_rate);
	if (i2c_imx->cur_clk == i2c_clk_rate)
		return;
	else
		i2c_imx->cur_clk = i2c_clk_rate;

	div = (i2c_clk_rate + i2c_imx->bitrate - 1) / i2c_imx->bitrate;

	if (div < i2c_clk_div[0][0])
		i = 0;
	else if (div > i2c_clk_div[ARRAY_SIZE(i2c_clk_div) - 1][0])
		i = ARRAY_SIZE(i2c_clk_div) - 1;
	else
		for (i = 0; i2c_clk_div[i][0] < div; i++);

	/* Store divider value */
	i2c_imx->ifdr = i2c_clk_div[i][1];

	/*
	 * There dummy delay is calculated.
	 * It should be about one I2C clock period long.
	 * This delay is used in I2C bus disable function
	 * to fix chip hardware bug.
	 */
	i2c_imx->disable_delay = (500000U * i2c_clk_div[i][0]
		+ (i2c_clk_rate / 2) - 1) / (i2c_clk_rate / 2);

	i2c_imx->actual_rate = i2c_clk_rate / i2c_clk_div[i][0];

	/* dev_dbg() can't be used, because adapter is not yet registered */
#ifdef CONFIG_I2C_DEBUG_BUS
	printk(KERN_DEBUG "I2C: <%s> I2C_CLK=%d, REQ DIV=%d\n",
		__func__, i2c_clk_rate, div);
	printk(KERN_DEBUG "I2C: <%s> IFDR[IC]=0x%x, REAL DIV=%d\n",
		__func__, i2c_clk_div[i][1], i2c_clk_div[i][0]);
#endif
}

static irqreturn_t i2c_imx_xfer_complete(struct imx_i2c_struct *i2c_imx, int result)
{
	dev_dbg(&i2c_imx->adapter.dev, "<%s>(0x%x)\n", __func__, result);

	// save error result for driver and issue error message if appropriate
	i2c_imx->msg_res = (result & ~I2SR_ICF);
	if (i2c_imx->msg_res)
	{
		dev_dbg(&i2c_imx->adapter.dev, "<%s> error : 0x%x\n", __func__, i2c_imx->msg_res);
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_ERROR
		toggle_led();
#endif
	}

	// did a transaction finish (successfully or failed)?
	if (result)
	{
		// skip to next message
		i2c_imx->msg_idx++;

		// all messages handled?
		if (i2c_imx->msg_res || (i2c_imx->msg_idx == i2c_imx->msg_num))
		{
			// yes, signal stop condition and notify driver
			i2c_imx_stop(i2c_imx);
			atomic_set(&i2c_imx->done,1);
		}
		else
		{
			// no, more work to do

			// signal (repeated or regular) start condition
			if (i2c_imx_start(i2c_imx, (i2c_imx->msg_idx > 0) ? I2CR_RSTA : I2CR_MSTA))
				return i2c_imx_xfer_complete(i2c_imx, I2SR_IAL);

			// reset message pointer and result
			i2c_imx->msg_res = -1;
			i2c_imx->msg_ptr = 0;

			// write I2C slave address
			writeb((i2c_imx->msgs[i2c_imx->msg_idx].addr << 1) | ((i2c_imx->msgs[i2c_imx->msg_idx].flags & I2C_M_RD) ? 0x01: 0x00),
					i2c_imx->base + IMX_I2C_I2DR);
			dev_dbg(&i2c_imx->adapter.dev, "<%s> %u: ad: 0x%04x\n", __func__, i2c_imx->msg_idx, i2c_imx->msgs[i2c_imx->msg_idx].addr);
		}
	}

#ifdef USE_IRQ
	spin_unlock_irqrestore(&i2c_imx->lock, i2c_imx->lock_flags);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t i2c_imx_isr(int irq, void *dev_id)
{
	struct imx_i2c_struct *i2c_imx = dev_id;
	unsigned int temp;

#ifdef USE_IRQ
	spin_lock_irqsave(&i2c_imx->lock, i2c_imx->lock_flags);
#endif

	// if msg_idx == -1 we have just been called directly by the xfer()-function
	// generate a (pseudo) xfer-completion to initiate the first message transfer
	if (i2c_imx->msg_idx == -1)
		return i2c_imx_xfer_complete(i2c_imx, I2SR_ICF);

	// did we receive a real interrupt?
	temp = readb(i2c_imx->base + IMX_I2C_I2SR);
	if (temp & I2SR_IIF)
	{
		// clear interrupt flag
		temp &= ~I2SR_IIF;
		writeb(temp, i2c_imx->base + IMX_I2C_I2SR);

		dev_dbg(&i2c_imx->adapter.dev, "<%s> : 0x%x\n", __func__, temp);

		// handle arbitration lost
		if (temp & I2SR_IAL)
			return i2c_imx_xfer_complete(i2c_imx, I2SR_IAL);

		// handle NOT_ACK
		if ((readb(i2c_imx->base + IMX_I2C_I2CR) & I2CR_MTX) &&
			(temp & I2SR_RXAK) && !(i2c_imx->msgs[i2c_imx->msg_idx].flags & I2C_M_IGNORE_NAK))
			return i2c_imx_xfer_complete(i2c_imx, I2SR_RXAK);

		// handle byte xfer
		if (temp & I2SR_ICF)
		{
			if (i2c_imx->msgs[i2c_imx->msg_idx].flags & I2C_M_RD)
			{
				// handle read transactions

				// If we haven't received any data yet, i.e. we just sent out the
				// I2C address byte, we have to reconfigure the I2C controller
				if ((i2c_imx->msg_ptr == 0) && (i2c_imx->msg_res == -1))
				{
					// setup bus to read data
					temp = readb(i2c_imx->base + IMX_I2C_I2CR);
					temp &= ~I2CR_MTX;
					if (i2c_imx->msgs[i2c_imx->msg_idx].len - 1)
						temp &= ~I2CR_TXAK;
					writeb(temp, i2c_imx->base + IMX_I2C_I2CR);

					// dummy read to start read-cycle
					readb(i2c_imx->base + IMX_I2C_I2DR);

					// signal byte xfer done, but message still incomplete
					return i2c_imx_xfer_complete(i2c_imx, 0);
				}

				// We must generate STOP before read I2DR for last message byte
				// to prevent controller from generating another clock cycle
				if (i2c_imx->msg_ptr == (i2c_imx->msgs[i2c_imx->msg_idx].len - 1))
				{
					temp = readb(i2c_imx->base + IMX_I2C_I2CR);
					temp &= ~(I2CR_MSTA | I2CR_MTX);
					writeb(temp, i2c_imx->base + IMX_I2C_I2CR);
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_XFER
					set_led(1);
#endif
				}
				// We acknowledge only the last byte we read as per I2C spec.
				// This must be done before reading the next to last byte as
				// reading the I2DR register will initiate the read-cycle for the
				// final byte.
				else if (i2c_imx->msg_ptr == (i2c_imx->msgs[i2c_imx->msg_idx].len - 2))
				{
					temp = readb(i2c_imx->base + IMX_I2C_I2CR);
					temp |= I2CR_TXAK;
					writeb(temp, i2c_imx->base + IMX_I2C_I2CR);
				}

				// read byte from I2C controller (also initiating next read cycle on the bus)
				i2c_imx->msgs[i2c_imx->msg_idx].buf[i2c_imx->msg_ptr++] = readb(i2c_imx->base + IMX_I2C_I2DR);
				dev_dbg(&i2c_imx->adapter.dev, "<%s> : rd 0x%02x\n", __func__, i2c_imx->msgs[i2c_imx->msg_idx].buf[i2c_imx->msg_ptr-1]);

				// if we received all bytes we expected, signal message complete
				if (i2c_imx->msg_ptr == i2c_imx->msgs[i2c_imx->msg_idx].len)
					return i2c_imx_xfer_complete(i2c_imx, I2SR_ICF);
			}
			else
			{
				// if we already sent all bytes, signal message complete
				if (i2c_imx->msg_ptr == i2c_imx->msgs[i2c_imx->msg_idx].len)
					return i2c_imx_xfer_complete(i2c_imx, I2SR_ICF);

				// write next data byte initiating next write cycle on the bus)
				writeb(i2c_imx->msgs[i2c_imx->msg_idx].buf[i2c_imx->msg_ptr++], i2c_imx->base + IMX_I2C_I2DR);
				dev_dbg(&i2c_imx->adapter.dev, "<%s> : wr 0x%02x\n", __func__, i2c_imx->msgs[i2c_imx->msg_idx].buf[i2c_imx->msg_ptr-1]);
			}

			// signal byte xfer done, but message still incomplete
			return i2c_imx_xfer_complete(i2c_imx, 0);
		}

	}

#ifdef USE_IRQ
	spin_unlock_irqrestore(&i2c_imx->lock, i2c_imx->lock_flags);
#endif

	return IRQ_NONE;
}

static int i2c_imx_xfer(struct i2c_adapter *adapter,
						struct i2c_msg *msgs, int num)
{
	int result = 0;
	unsigned long timeout = 0;
	struct imx_i2c_struct *i2c_imx = i2c_get_adapdata(adapter);
	unsigned long lock_flags;

	dev_dbg(&i2c_imx->adapter.dev, "<%s> %u messages\n", __func__, num);

	// *******************************************************************
	// Check i2c_imx->done flag to make sure there is no transfer pending
	// *******************************************************************
	timeout = jiffies + 5 * HZ * i2c_imx->msg_num;
	while (atomic_read(&i2c_imx->done) == 0)
	{
		if (time_after(jiffies, timeout))
		{
			dev_err(&i2c_imx->adapter.dev, "<%s> timeout waiting for previous i2c transfer to complete\n", __func__);
			timeout = 0;
			break;
		}
#ifndef USE_IRQ
		if (IRQ_NONE == i2c_imx_isr(0, (void *)i2c_imx))
#endif
			udelay(1000000/i2c_imx->actual_rate);		// one I2C clock-cycle
	}


	// configure I2C-pins to GPIO-input to reliably determine bus-state
	if( i2c_imx->platform.i2c_set_gpio_mode)
	{
		i2c_imx->platform.i2c_set_gpio_mode(&i2c_imx->platform, i2c_gpio_mode_gpio);

		gpio_request(i2c_imx->platform.i2c_scl_pin, "i2c_scl");
		gpio_request(i2c_imx->platform.i2c_sda_pin, "i2c_sda");
		gpio_direction_input(i2c_imx->platform.i2c_scl_pin);
		gpio_direction_input(i2c_imx->platform.i2c_sda_pin);

		// see if some slave is still driving SDA low
		if (!gpio_get_value(i2c_imx->platform.i2c_sda_pin))
		{
			int i;
			// if so, try to get the slave unstuck by generating clock-cycles
#ifdef TOGGLE_GUF_I2C_DEBUG_SIGNAL_ON_STUCK
			toggle_led();
#endif
			gpio_direction_output(i2c_imx->platform.i2c_scl_pin, 1);

			for (i = 0; i < 16; i++)
			{
				udelay(10);
				gpio_set_value(i2c_imx->platform.i2c_scl_pin, 0);
				udelay(10);
				gpio_set_value(i2c_imx->platform.i2c_scl_pin, 1);
			}
		}

		// configure I2C-pins to functional mode
		gpio_free(i2c_imx->platform.i2c_sda_pin);
		gpio_free(i2c_imx->platform.i2c_scl_pin);

		i2c_imx->platform.i2c_set_gpio_mode( &i2c_imx->platform, i2c_gpio_mode_func);
	}
	// enable I2C controller clock
	clk_prepare_enable(i2c_imx->clk);
	writeb(i2c_imx->ifdr, i2c_imx->base + IMX_I2C_IFDR);

	// Enable I2C controller
	writeb(0, i2c_imx->base + IMX_I2C_I2SR);
	writeb(I2CR_IEN, i2c_imx->base + IMX_I2C_I2CR);

	// Wait controller to be stable
	udelay(50);

	// setup message struct to be used by ISR
#ifdef USE_IRQ
	spin_lock_irqsave(&i2c_imx->lock, lock_flags);
#endif
	atomic_set(&i2c_imx->done,0);
	i2c_imx->msgs = msgs;
	i2c_imx->msg_ptr = 0;
	i2c_imx->msg_num = num;
	i2c_imx->msg_idx = -1;
	i2c_imx->msg_res = -1;
#ifdef USE_IRQ
	spin_unlock_irqrestore(&i2c_imx->lock, lock_flags);
#endif

	// Trigger ISR manually to start processing
	i2c_imx_isr(0, (void *)i2c_imx);

	// The rest of the processing occurs in the interrupt handler.

	// Note: Using a wait_queue and wait_event_timeout() here to wait for the
	// ISR to finish would be nice, but doesn't seem to work stable with drivers
	// (like the DA905x) that make I2C communication in their probe-routine:
	//   - sometimes we just don't receive the event
	//   - the timeout doesn't seem to work at all in that cases
	// both together result in a completely hanging system during boot... :-(
	//
	// Using a completion would be nice, as well, but these these are based on
	// wait_queues...
	//
	timeout = jiffies + 5 * HZ * num;
	while (atomic_read(&i2c_imx->done) == 0)
	{
		if (time_after(jiffies, timeout))
		{
			timeout = 0;
			break;
		}
#ifndef USE_IRQ
		if (IRQ_NONE == i2c_imx_isr(0, (void *)i2c_imx))
#endif
			udelay(1000000/i2c_imx->actual_rate);		// one I2C clock-cycle
	}

	// Disable I2C controller
	writeb(0, i2c_imx->base + IMX_I2C_I2CR);
	clk_disable_unprepare(i2c_imx->clk);

	// handle return codes
	if (timeout == 0)
		result = -ETIMEDOUT;
	else if (i2c_imx->msg_res == I2SR_IAL)
		result = -EBUSY;
	else if (i2c_imx->msg_res == I2SR_RXAK)
		result = -EIO;
	else if (i2c_imx->msg_res)
		result = -EREMOTEIO;

	if( i2c_imx->platform.i2c_set_gpio_mode)
	{
		i2c_imx->platform.i2c_set_gpio_mode( &i2c_imx->platform, i2c_gpio_mode_gpio);
		gpio_request(i2c_imx->platform.i2c_scl_pin, "i2c_scl");
		gpio_request(i2c_imx->platform.i2c_sda_pin, "i2c_sda");
		gpio_direction_input(i2c_imx->platform.i2c_scl_pin);
		gpio_direction_input(i2c_imx->platform.i2c_sda_pin);
		gpio_free(i2c_imx->platform.i2c_sda_pin);
		gpio_free(i2c_imx->platform.i2c_scl_pin);
	}

	dev_dbg(&i2c_imx->adapter.dev, "<%s> exit with: %s: %d\n", __func__,
		(result < 0) ? "error" : "success msg",
			(result < 0) ? result : num);
	return (result < 0) ? result : num;
}

static u32 i2c_imx_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm i2c_imx_algo = {
	.master_xfer	= i2c_imx_xfer,
	.functionality	= i2c_imx_func,
};

#ifdef CONFIG_OF
static void i2c_set_pinctrl( struct imxi2c_platform_data * pdata, enum i2c_gpio_mode mode)
{
	int ret = 0;
	//pr_info("%s: %d\n", __func__, mode);
	switch( mode)
	{
		case i2c_gpio_mode_func:
			ret = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_state_i2c);
			break;
		case i2c_gpio_mode_gpio:
			ret = pinctrl_select_state(pdata->pinctrl, pdata->pinctrl_state_gpio);
			break;
	}
	if( ret)
		pr_warn("%s: set pinctrl to mode %d failed: %d\n", __func__, mode, ret	);
}
#endif

static int __init i2c_imx_probe(struct platform_device *pdev)
{
	struct imx_i2c_struct *i2c_imx;
	struct resource *res;
	struct imxi2c_platform_data *pdata = pdev->dev.platform_data;
	void __iomem *base;
	resource_size_t res_size;
#ifdef USE_IRQ
	int irq;
#endif
#ifdef CONFIG_OF
	const struct of_device_id *of_id; 
#endif
	int ret;

	pr_debug("%s: pdata: 0x%08x\n",__func__, (unsigned int) pdata);

	i2c_imx = devm_kzalloc(&pdev->dev, sizeof(struct imx_i2c_struct), GFP_KERNEL);
	if (!i2c_imx) {
		dev_err(&pdev->dev, "can't allocate interface\n");
		return -ENOMEM;
	}
	/* Setup i2c_imx driver structure */
	if( pdata) 	
		memcpy(&i2c_imx->platform, pdata, sizeof(struct imxi2c_platform_data));

	dev_dbg(&pdev->dev, "<%s>\n", __func__);

#ifdef CONFIG_OF
	of_id = of_match_device(i2c_imx_dt_ids, &pdev->dev);

	if( of_id != NULL)
	{
		struct imxi2c_platform_data * pd = &i2c_imx->platform;
		struct device_node *np = pdev->dev.of_node;

		pd->i2c_scl_pin = of_get_named_gpio( np, "gpio-scl", 0);
		if( pd->i2c_scl_pin < 0)
		{
			dev_warn(&pdev->dev, " scl gpio was not found in devicetree");
			goto no_pinctrl;			
		}
		pd->i2c_sda_pin	= of_get_named_gpio( np, "gpio-sda", 0);
		if( pd->i2c_sda_pin < 0)
		{
			dev_warn(&pdev->dev, " sda gpio was not found in devicetree");
			goto no_pinctrl;
		}

		pd->pinctrl = devm_pinctrl_get(&pdev->dev);
		if(!pd->pinctrl )
		{
			dev_warn(&pdev->dev, " pinctrl was not found in devicetree");
			goto no_pinctrl;
		}
		
		pd->pinctrl_state_i2c = pinctrl_lookup_state(pd->pinctrl, "default");
		if( IS_ERR(pd->pinctrl_state_i2c))
		{
			dev_warn(&pdev->dev, " pinctrl 'default' was not found in devicetree");
			goto no_pinctrl;
		}

		pd->pinctrl_state_gpio = pinctrl_lookup_state(pd->pinctrl, "gpio");
		if( IS_ERR(pd->pinctrl_state_gpio))
		{
			dev_warn(&pdev->dev, " pinctrl 'gpio' was not found in devicetree");
			goto no_pinctrl;
		}

		pr_debug("%s: state default 0x%08x state gpio 0x%08x\n", __func__, 
			(unsigned int)pd->pinctrl_state_i2c, (unsigned int) pd->pinctrl_state_gpio);
		
		pd->i2c_set_gpio_mode = i2c_set_pinctrl;
		i2c_set_pinctrl(pd, i2c_gpio_mode_func);

no_pinctrl:
		pdev->id_entry = of_id->data;
	}
#endif

	// Check if the given pins are working, at least now
	if( i2c_imx->platform.i2c_set_gpio_mode)
	{
		ret = gpio_request(i2c_imx->platform.i2c_scl_pin, "i2c_scl");
		if( ret)
		{
			dev_err(&pdev->dev, "failed to request i2c_scl gpio");
			goto fail0;
		}
		gpio_free(i2c_imx->platform.i2c_scl_pin);
		ret = gpio_request(i2c_imx->platform.i2c_sda_pin, "i2c_sda");
		if( ret)
		{
			dev_err(&pdev->dev, "failed to request i2c_sda gpio");
			goto fail0;
		}
		gpio_free(i2c_imx->platform.i2c_sda_pin);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get device resources\n");
		return -ENOENT;
	}
#ifdef USE_IRQ
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "can't get irq number\n");
		return -ENOENT;
	}
#endif

	if (i2c_imx->platform.init) {
		ret = i2c_imx->platform.init(&pdev->dev);
		if (ret)
			return ret;
	}

	res_size = resource_size(res);

	if (!devm_request_mem_region(&pdev->dev, res->start, res_size, DRIVER_NAME)) {
		ret = -EBUSY;
		goto fail0;
	}

	base = devm_ioremap(&pdev->dev, res->start, res_size);
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -EIO;
		goto fail0;
	}

	i2c_imx->devtype = pdev->id_entry->driver_data;

	strcpy(i2c_imx->adapter.name, pdev->name);
	i2c_imx->adapter.owner		= THIS_MODULE;
	i2c_imx->adapter.algo		= &i2c_imx_algo;
	i2c_imx->adapter.dev.parent	= &pdev->dev;
	i2c_imx->adapter.nr 		= pdev->id;
	i2c_imx->adapter.dev.of_node	= pdev->dev.of_node;
	atomic_set(&i2c_imx->done,1);// Not busy at the moment
	pr_debug( "i2c_imx_probe: i2c_imx->adapter.nr: %d\n", i2c_imx->adapter.nr);
#ifdef USE_IRQ
	i2c_imx->irq			= irq;
#endif
	i2c_imx->base			= base;
	i2c_imx->res			= res;

#ifdef USE_IRQ
	spin_lock_init(&i2c_imx->lock);
#endif

	/* Get I2C clock */
	i2c_imx->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c_imx->clk)) {
		ret = PTR_ERR(i2c_imx->clk);
		dev_err(&pdev->dev, "can't get I2C clock\n");
		goto fail0;
	}

#ifdef USE_IRQ
	/* Request IRQ */
	ret = devm_request_irq(&pdev->dev, i2c_imx->irq, i2c_imx_isr, 0,	pdev->name, i2c_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n", i2c_imx->irq);
		goto fail0;
	}
#endif

	/* Set up adapter data */
	i2c_set_adapdata(&i2c_imx->adapter, i2c_imx);

	/* Set up clock divider */
	i2c_imx->bitrate = IMX_I2C_BIT_RATE;
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &i2c_imx->bitrate);
	if (ret < 0 && pdata && pdata->bitrate)
		i2c_imx->bitrate = pdata->bitrate;
	i2c_imx_set_clk(i2c_imx);

	/* Set up chip registers to defaults */
	writeb(0, i2c_imx->base + IMX_I2C_I2CR);
	writeb(0, i2c_imx->base + IMX_I2C_I2SR);

	/* Add I2C adapter */
	ret = i2c_add_numbered_adapter(&i2c_imx->adapter);
	if (ret < 0) {
		dev_err(&pdev->dev, "registration failed\n");
		goto fail0;
	}
	of_i2c_register_devices(&i2c_imx->adapter);

	/* Set up platform driver data */
	platform_set_drvdata(pdev, i2c_imx);

#ifdef USE_IRQ
	dev_dbg(&i2c_imx->adapter.dev, "claimed irq %d\n", i2c_imx->irq);
#endif
	dev_dbg(&i2c_imx->adapter.dev, "device resources from 0x%x to 0x%x\n",
		i2c_imx->res->start, i2c_imx->res->end);
	dev_dbg(&i2c_imx->adapter.dev, "allocated %d bytes at 0x%x \n",
		res_size, i2c_imx->res->start);
	dev_dbg(&i2c_imx->adapter.dev, "adapter name: \"%s\"\n",
		i2c_imx->adapter.name);
	dev_dbg(&i2c_imx->adapter.dev, "IMX I2C adapter registered\n");

	return 0;   /* Return OK */

fail0:
	if (pdata && pdata->exit)
		pdata->exit(&pdev->dev);
	return ret; /* Return error number */
}

static int __exit i2c_imx_remove(struct platform_device *pdev)
{
	struct imx_i2c_struct *i2c_imx = platform_get_drvdata(pdev);
	struct imxi2c_platform_data *pdata = pdev->dev.platform_data;

	/* remove adapter */
	dev_dbg(&i2c_imx->adapter.dev, "adapter removed\n");
	i2c_del_adapter(&i2c_imx->adapter);
	platform_set_drvdata(pdev, NULL);

	/* setup chip registers to defaults */
	writeb(0, i2c_imx->base + IMX_I2C_IADR);
	writeb(0, i2c_imx->base + IMX_I2C_IFDR);
	writeb(0, i2c_imx->base + IMX_I2C_I2CR);
	writeb(0, i2c_imx->base + IMX_I2C_I2SR);

	/* Shut down hardware */
	if (pdata && pdata->exit)
		pdata->exit(&pdev->dev);

	return 0;
}

static struct platform_driver i2c_imx_driver = {
	.remove		= __exit_p(i2c_imx_remove),
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = i2c_imx_dt_ids,
	},
	.id_table	= imx_i2c_devtype,
};

static int __init i2c_adap_imx_init(void)
{
	return platform_driver_probe(&i2c_imx_driver, i2c_imx_probe);
}
subsys_initcall(i2c_adap_imx_init);

static void __exit i2c_adap_imx_exit(void)
{
	platform_driver_unregister(&i2c_imx_driver);
}
module_exit(i2c_adap_imx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Darius Augulis");
MODULE_DESCRIPTION("I2C adapter driver for IMX I2C bus");
MODULE_ALIAS("platform:" DRIVER_NAME);
