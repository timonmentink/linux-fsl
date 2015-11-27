/*
 *  Philips UCB1400 touchscreen driver
 *
 *  Author:	Nicolas Pitre
 *  Created:	September 25, 2006
 *  Copyright:	MontaVista Software, Inc.
 *
 * Spliting done by: Marek Vasut <marek.vasut@gmail.com>
 * If something doesnt work and it worked before spliting, e-mail me,
 * dont bother Nicolas please ;-)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This code is heavily based on ucb1x00-*.c copyrighted by Russell King
 * covering the UCB1100, UCB1200 and UCB1300..  Support for the UCB1400 has
 * been made separate from ucb1x00-core/ucb1x00-ts on Russell's request.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/ucb1400.h>
#include <linux/gpio.h>

/* Number of Touch-Up events before they are reported  */
#define TS_MIN_TOUCHUP			4

/* Maximum number of retries to enable the Touch IRQ */
#define ENABLE_IRQ_MAX_RETRIES 1000

/* Number of samples that are read for a single point */
#define NUMBER_SAMPLES_PER_POINT 10

/* Maximum allowed variance in the X/Y coordinate samples. */
#define MAX_DISTANCE			1

/* minumum number of consistent samples required to consider */
/* a coordinate valid */
#define MINIMUM_VALID_SAMPLES	4

/* threshold for pressure, under which the sample is ignored */
#define PRESSURE_THRESHOLD		200

/* constant for a sampled value to be ignored */
#define TOUCH_SAMPLE_IGNORE		0xFFFF

/* Number of retries if UCB write did not succeed */
#define UCB_COMM_RETRIES		10

static int adcsync;
static int ts_delay = 150; /* us */
static int ts_delay_pressure;	/* us */
static unsigned long debug = 0;

static ssize_t read_debug(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	snprintf(buf, 255, "%ld\n", debug);
	return strlen(buf)+1;
}

static ssize_t write_debug(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long val = 0;
	val = simple_strtoul(buf, NULL, 0);
	if(val >= 0 && val <= 1)
		debug = val;
	return strlen(buf)+1;
}

static DEVICE_ATTR(debug, S_IRUGO | S_IWUGO, read_debug, write_debug);

/* Switch to interrupt mode. */
static inline void ucb1400_ts_mode_int(struct snd_ac97 *ac97)
{
	ucb1400_reg_write(ac97, UCB_TS_CR,
			UCB_TS_CR_TSMX_POW | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
			UCB_TS_CR_MODE_INT);
}

static int filter_samples(int *samples)
{
    int i;
    int n_valid_samples = 0;
    int mean=0;
    int max_dist=0;
    int candidate_for_discard = -1;

    // compute mean value
    for (i=0; i<NUMBER_SAMPLES_PER_POINT; i++)
        if (samples[i] != UCB_ADC_DAT_MASK)
        {
            n_valid_samples++;
            mean += samples[i];
        }

    if (n_valid_samples < MINIMUM_VALID_SAMPLES)
        return TOUCH_SAMPLE_IGNORE;

	mean = mean / n_valid_samples;

    // look for the farthest position
    for (i=0; i<NUMBER_SAMPLES_PER_POINT; i++)
        if (samples[i] != UCB_ADC_DAT_MASK)
        {
            int dist = (mean - samples[i]) * (mean - samples[i]);
            if (dist > max_dist)
            {
                max_dist = dist;
                candidate_for_discard = i;
            }
        }

    if (max_dist > (MAX_DISTANCE*MAX_DISTANCE))
    {
        samples[candidate_for_discard] = UCB_ADC_DAT_MASK;
        return filter_samples(samples);
    }

	return mean;
}

/* Read an adc channel and apply filter (x-pos, y-pos, pressure) */
static inline unsigned int ucb1400_ts_read_adc_channel(struct ucb1400_ts *ucb, u16 channel, int adcsync)
{
	int i;
	int samples[NUMBER_SAMPLES_PER_POINT];

	for (i=0; i<NUMBER_SAMPLES_PER_POINT; i++)
		samples[i] = ucb1400_adc_read(ucb->ac97, channel, adcsync);

	return filter_samples(samples);
}

/*
 * Switch to pressure mode, and read pressure.  We don't need to wait
 * here, since both plates are being driven.
 */
static inline unsigned int ucb1400_ts_read_pressure(struct ucb1400_ts *ucb)
{
	int i=0;

	for (i = 0; i < UCB_COMM_RETRIES; i++)
	{
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMX_POW | UCB_TS_CR_TSPX_POW |
				UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
				UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);

		if (ucb1400_reg_read(ucb->ac97, UCB_TS_CR) == 
			(UCB_TS_CR_TSMX_POW | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA))
		{
			return ucb1400_ts_read_adc_channel(ucb, UCB_ADC_INP_TSPY, adcsync);
		}
	}

	return TOUCH_SAMPLE_IGNORE;
}

/*
 * Switch to X position mode and measure Y plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ucb1400_ts_read_xpos(struct ucb1400_ts *ucb)
{
	int i=0;

	for (i = 0; i < UCB_COMM_RETRIES; i++)
	{
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
				UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
				UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
				UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA);

		if (ucb1400_reg_read(ucb->ac97, UCB_TS_CR) == 
			(UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA))
		{
			udelay(ts_delay);
			return ucb1400_ts_read_adc_channel(ucb, UCB_ADC_INP_TSPY, adcsync);
		}
	}

	return TOUCH_SAMPLE_IGNORE;
}

/*
 * Switch to Y position mode and measure X plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ucb1400_ts_read_ypos(struct ucb1400_ts *ucb)
{
	int i=0;

	for (i = 0; i < UCB_COMM_RETRIES; i++)
	{
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
				UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
				UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
		ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
				UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
				UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA);

		if (ucb1400_reg_read(ucb->ac97, UCB_TS_CR) == 
			(UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA))
		{
			udelay(ts_delay);
			return ucb1400_ts_read_adc_channel(ucb, UCB_ADC_INP_TSPX, adcsync);
		}
	}

	return TOUCH_SAMPLE_IGNORE;
}

/*
 * Switch to X plate resistance mode.  Set MX to ground, PX to
 * supply.  Measure current.
 */
static inline unsigned int ucb1400_ts_read_xres(struct ucb1400_ts *ucb)
{
	ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
			UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	return ucb1400_adc_read(ucb->ac97, 0, adcsync);
}

/*
 * Switch to Y plate resistance mode.  Set MY to ground, PY to
 * supply.  Measure current.
 */
static inline unsigned int ucb1400_ts_read_yres(struct ucb1400_ts *ucb)
{
	ucb1400_reg_write(ucb->ac97, UCB_TS_CR,
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	return ucb1400_adc_read(ucb->ac97, 0, adcsync);
}

static inline int ucb1400_ts_pen_up(struct snd_ac97 *ac97)
{
	unsigned short val = ucb1400_reg_read(ac97, UCB_TS_CR);
	return val & (UCB_TS_CR_TSPX_LOW | UCB_TS_CR_TSMX_LOW);
}

static inline void ucb1400_ts_irq_enable(struct snd_ac97 *ac97)
{
	unsigned int i = 0;

	/* clear the TSPX interrupt status */
	ucb1400_reg_write(ac97, UCB_IE_CLEAR, UCB_IE_TSPX);
	/* reset INT clear/status register 0x62 */
	ucb1400_reg_write(ac97, UCB_IE_CLEAR, 0);

	/* There seems to be a problem enabling the IRQ...
	 * Make sure it is set, repeat setting it if necessary and bail out after
	 * ENABLE_IRQ_MAX_RETRIES.
	 */
	for (i=0; i < ENABLE_IRQ_MAX_RETRIES; i++) {
		/* enable falling edge interrupt of TSPX signal */
		ucb1400_reg_write(ac97, UCB_IE_FAL, UCB_IE_TSPX);

		/* check */
		if ( ucb1400_reg_read(ac97, UCB_IE_FAL) & UCB_IE_TSPX ) {
			break;
		} else {
			printk(KERN_ERR "ucb1400: failed to set touch IRQ. Retrying...\n");
			udelay(50);
		}
	}
	if ( i >= ENABLE_IRQ_MAX_RETRIES ) {
		printk(KERN_ERR "ucb1400: failed to set touch IRQ. Exceeded retries. \n");
	}
}

static inline void ucb1400_ts_irq_disable(struct snd_ac97 *ac97)
{
	ucb1400_reg_write(ac97, UCB_IE_FAL, 0);
}

static void ucb1400_ts_evt_add(struct input_dev *idev, u16 pressure, u16 x, u16 y)
{
	input_report_abs(idev, ABS_X, x);
	input_report_abs(idev, ABS_Y, y);
	input_report_abs(idev, ABS_PRESSURE, pressure);
	input_report_key(idev, BTN_TOUCH, 1);
	input_sync(idev);
}

static void ucb1400_ts_event_release(struct input_dev *idev)
{
	input_report_abs(idev, ABS_PRESSURE, 0);
	input_report_key(idev, BTN_TOUCH, 0);
	input_sync(idev);
}

static void ucb1400_handle_pending_irq(struct ucb1400_ts *ucb)
{
	unsigned int isr;

	isr = ucb1400_reg_read(ucb->ac97, UCB_IE_STATUS);
	ucb1400_reg_write(ucb->ac97, UCB_IE_CLEAR, isr);
	ucb1400_reg_write(ucb->ac97, UCB_IE_CLEAR, 0);

	if (isr & UCB_IE_TSPX)
		ucb1400_ts_irq_disable(ucb->ac97);
	else
		dev_dbg(&ucb->ts_idev->dev, "ucb1400: unexpected IE_STATUS = %#x\n", isr);
	enable_irq(ucb->irq);
}

static int ucb1400_ts_thread(void *_ucb)
{
	struct ucb1400_ts *ucb = _ucb;
	struct task_struct *tsk = current;
	int valid = 0;
	unsigned int touch_ups = 0;
	unsigned int old_x = 0;
	unsigned int old_y = 0;
	unsigned int old_p = 0;
	struct sched_param param = { .sched_priority = 1 };
	long timeout;

	pr_debug("<%s>\n", __func__);

	sched_setscheduler(tsk, SCHED_FIFO, &param);

	set_freezable();
	while (!kthread_should_stop()) {
		unsigned int x = 0;
		unsigned int y = 0;
		unsigned int p = 0;

		ucb->ts_restart = 0;

		/* disable falling edge interrupt of TSPX signal */
		if (ucb->irq_pending) {
			ucb->irq_pending = 0;
			ucb1400_handle_pending_irq(ucb);
		}

		/* enter interrupt mode */
		ucb1400_ts_mode_int(ucb->ac97);
		udelay(ts_delay);

		/* queue touch samples and report last one
		 * should a touch-up event occur, report that after
		 * TS_MIN_TOUCHUP touch-ups		 */
		if (ucb1400_ts_pen_up(ucb->ac97)) {
			/* enable falling edge interrupt of TSPX signal */
			ucb1400_ts_irq_enable(ucb->ac97);
			timeout = MAX_SCHEDULE_TIMEOUT;
			/*
			 * If we spat out a valid sample set last time,
			 * spit out a "pen off" sample here.
			 */
			if (valid) {
				if (touch_ups < TS_MIN_TOUCHUP) {
					// restore last (valid) values, but don't report current ones
					touch_ups++;
					x = old_x;
					y = old_y;
					p = old_p;
					/* set timeout */
					timeout = msecs_to_jiffies(5);
				} else {
					/* report touch-up */
					valid = 0;
					ucb1400_ts_event_release(ucb->ts_idev);
				}
			}

		} else {
			ucb1400_adc_enable(ucb->ac97);
			x = ucb1400_ts_read_xpos(ucb);
			y = ucb1400_ts_read_ypos(ucb);
			p = ucb1400_ts_read_pressure(ucb);
			ucb1400_adc_disable(ucb->ac97);

			/* Switch back to interrupt mode. */
			ucb1400_ts_mode_int(ucb->ac97);

			/* set timeout */
			timeout = msecs_to_jiffies(5);

			// We check here whether all samples are valid.
			// If the pressure is below a certain threshold, we
			// ignore the sample. If it has changed too much,
			// we ignore it as well.
			if ((x == TOUCH_SAMPLE_IGNORE) || (y == TOUCH_SAMPLE_IGNORE) ||
			    (p == TOUCH_SAMPLE_IGNORE) || (p!=0 && p<PRESSURE_THRESHOLD))
			{
				if(debug)
					pr_warning("ucb1400_ts: Discarding current sample (x: %d, y: %d, p: %d)", x, y, p);

				wait_event_freezable_timeout(ucb->ts_wait,
					ucb->irq_pending || ucb->ts_restart ||
					kthread_should_stop(), timeout);
				continue;
			}

			/* got valid touch-down */
			valid = 1;
			touch_ups = 0;

			/* report touch-down + coordinates */
			ucb1400_ts_evt_add(ucb->ts_idev, old_p, old_x, old_y);
		}
		/* store current touch sample */
		old_x = x;
		old_y = y;
		old_p = p;

		wait_event_freezable_timeout(ucb->ts_wait,
			ucb->irq_pending || ucb->ts_restart ||
			kthread_should_stop(), timeout);
	}

	/* Send the "pen off" if we are stopping with the pen still active */
	if (valid)
		ucb1400_ts_event_release(ucb->ts_idev);

	ucb->ts_task = NULL;
	return 0;
}

/*
 * A restriction with interrupts exists when using the ucb1400, as
 * the codec read/write routines may sleep while waiting for codec
 * access completion and uses semaphores for access control to the
 * AC97 bus.  A complete codec read cycle could take  anywhere from
 * 60 to 100uSec so we *definitely* don't want to spin inside the
 * interrupt handler waiting for codec access.  So, we handle the
 * interrupt by scheduling a RT kernel thread to run in process
 * context instead of interrupt context.
 */
static irqreturn_t ucb1400_hard_irq(int irqnr, void *devid)
{
	struct ucb1400_ts *ucb = devid;

	if (irqnr == ucb->irq) {
		disable_irq_nosync(ucb->irq);
		ucb->irq_pending = 1;
		wake_up(&ucb->ts_wait);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int ucb1400_ts_open(struct input_dev *idev)
{
	struct ucb1400_ts *ucb = input_get_drvdata(idev);
	int ret = 0;

	pr_debug("<%s>\n", __func__);
	BUG_ON(ucb->ts_task);

	ucb->ts_task = kthread_run(ucb1400_ts_thread, ucb, "UCB1400_ts");
	if (IS_ERR(ucb->ts_task)) {
		ret = PTR_ERR(ucb->ts_task);
		ucb->ts_task = NULL;
	}

	return ret;
}

static void ucb1400_ts_close(struct input_dev *idev)
{
	struct ucb1400_ts *ucb = input_get_drvdata(idev);
	pr_debug("<%s>\n", __func__);

	if (ucb->ts_task)
		kthread_stop(ucb->ts_task);

	ucb1400_ts_irq_disable(ucb->ac97);
	ucb1400_reg_write(ucb->ac97, UCB_TS_CR, 0);
}

#ifndef NO_IRQ
#define NO_IRQ	0
#endif

/*
 * Try to probe our interrupt, rather than relying on lots of
 * hard-coded machine dependencies.
 */
static int ucb1400_ts_detect_irq(struct ucb1400_ts *ucb)
{
	unsigned long mask, timeout;

	mask = probe_irq_on();

	/* Enable the ADC interrupt. */
	ucb1400_reg_write(ucb->ac97, UCB_IE_RIS, UCB_IE_ADC);
	ucb1400_reg_write(ucb->ac97, UCB_IE_FAL, UCB_IE_ADC);
	ucb1400_reg_write(ucb->ac97, UCB_IE_CLEAR, 0xffff);
	ucb1400_reg_write(ucb->ac97, UCB_IE_CLEAR, 0);

	/* Cause an ADC interrupt. */
	ucb1400_reg_write(ucb->ac97, UCB_ADC_CR, UCB_ADC_ENA);
	ucb1400_reg_write(ucb->ac97, UCB_ADC_CR, UCB_ADC_ENA | UCB_ADC_START);

	/* Wait for the conversion to complete. */
	timeout = jiffies + HZ/2;
	while (!(ucb1400_reg_read(ucb->ac97, UCB_ADC_DATA) &
						UCB_ADC_DAT_VALID)) {
		cpu_relax();
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "ucb1400: timed out in IRQ probe\n");
			probe_irq_off(mask);
			return -ENODEV;
		}
	}
	ucb1400_reg_write(ucb->ac97, UCB_ADC_CR, 0);

	/* Disable and clear interrupt. */
	ucb1400_reg_write(ucb->ac97, UCB_IE_RIS, 0);
	ucb1400_reg_write(ucb->ac97, UCB_IE_FAL, 0);
	ucb1400_reg_write(ucb->ac97, UCB_IE_CLEAR, 0xffff);
	ucb1400_reg_write(ucb->ac97, UCB_IE_CLEAR, 0);

	/* Read triggered interrupt. */
	ucb->irq = probe_irq_off(mask);
	if (ucb->irq < 0 || ucb->irq == NO_IRQ)
		return -ENODEV;

	return 0;
}

static int ucb1400_ts_probe(struct platform_device *dev)
{
	int error, x_res, y_res;
	u16 fcsr;
	struct ucb1400_ts *ucb = dev->dev.platform_data;

	pr_debug("<%s> IRQ: %d\n", __func__, ucb->irq);
	ucb->ts_idev = input_allocate_device();
	if (!ucb->ts_idev) {
		error = -ENOMEM;
		goto err;
	}

	/* Only in case the IRQ line wasn't supplied, try detecting it */
	if (ucb->irq < 0) {
		error = ucb1400_ts_detect_irq(ucb);
		if (error) {
			printk(KERN_ERR "UCB1400: IRQ probe failed\n");
			goto err_free_devs;
		}
	}
	init_waitqueue_head(&ucb->ts_wait);

	error = request_irq(ucb->irq, ucb1400_hard_irq, IRQF_TRIGGER_RISING,
				"UCB1400", ucb);
	if (error) {
		printk(KERN_ERR "ucb1400: unable to grab irq%d: %d\n",
				ucb->irq, error);
		goto err_free_devs;
	}
	printk(KERN_DEBUG "UCB1400: found IRQ %d\n", ucb->irq);

	input_set_drvdata(ucb->ts_idev, ucb);

	ucb->ts_idev->dev.parent	= &dev->dev;
	ucb->ts_idev->name		= "UCB1400 touchscreen interface";
	ucb->ts_idev->id.vendor		= ucb1400_reg_read(ucb->ac97,
						AC97_VENDOR_ID1);
	ucb->ts_idev->id.product	= ucb->id;
	ucb->ts_idev->open		= ucb1400_ts_open;
	ucb->ts_idev->close		= ucb1400_ts_close;
	ucb->ts_idev->evbit[0]		= BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	ucb->ts_idev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/*
	 * Enable ADC filter to prevent horrible jitter on Colibri.
	 * This also further reduces jitter on boards where ADCSYNC
	 * pin is connected.
	 */
	fcsr = ucb1400_reg_read(ucb->ac97, UCB_FCSR);
	ucb1400_reg_write(ucb->ac97, UCB_FCSR, fcsr | UCB_FCSR_AVE);

	ucb1400_adc_enable(ucb->ac97);
	x_res = ucb1400_ts_read_xres(ucb);
	y_res = ucb1400_ts_read_yres(ucb);
	ucb1400_adc_disable(ucb->ac97);
	printk(KERN_DEBUG "UCB1400: x/y = %d/%d\n", x_res, y_res);

	input_set_abs_params(ucb->ts_idev, ABS_X, 0, x_res, 0, 0);
	input_set_abs_params(ucb->ts_idev, ABS_Y, 0, y_res, 0, 0);
	input_set_abs_params(ucb->ts_idev, ABS_PRESSURE, 0, 0, 0, 0);

	error = input_register_device(ucb->ts_idev);
	if (error)
		goto err_free_irq;

	/* Create debug switch in sysfs. */
	device_create_file(&ucb->ts_idev->dev, &dev_attr_debug);

	return 0;

err_free_irq:
	free_irq(ucb->irq, ucb);
err_free_devs:
	input_free_device(ucb->ts_idev);
err:
	return error;

}

static int ucb1400_ts_remove(struct platform_device *dev)
{
	struct ucb1400_ts *ucb = dev->dev.platform_data;

	device_remove_file(&ucb->ts_idev->dev, &dev_attr_debug);
	free_irq(ucb->irq, ucb);
	input_unregister_device(ucb->ts_idev);
	return 0;
}

#ifdef CONFIG_PM
static int ucb1400_ts_resume(struct platform_device *dev)
{
	struct ucb1400_ts *ucb = dev->dev.platform_data;

	if (ucb->ts_task) {
		/*
		 * Restart the TS thread to ensure the
		 * TS interrupt mode is set up again
		 * after sleep.
		 */
		ucb->ts_restart = 1;
		wake_up(&ucb->ts_wait);
	}
	return 0;
}
#else
#define ucb1400_ts_resume NULL
#endif

static struct platform_driver ucb1400_ts_driver = {
	.probe	= ucb1400_ts_probe,
	.remove	= ucb1400_ts_remove,
	.resume	= ucb1400_ts_resume,
	.driver	= {
		.name	= "ucb1400_ts",
	},
};

static int __init ucb1400_ts_init(void)
{
	return platform_driver_register(&ucb1400_ts_driver);
}

static void __exit ucb1400_ts_exit(void)
{
	platform_driver_unregister(&ucb1400_ts_driver);
}

module_param(adcsync, bool, 0444);
MODULE_PARM_DESC(adcsync, "Synchronize touch readings with ADCSYNC pin.");

module_param(ts_delay, int, 0444);
MODULE_PARM_DESC(ts_delay, "Delay between panel setup and"
			    " position read. Default = 55us.");

module_param(ts_delay_pressure, int, 0444);
MODULE_PARM_DESC(ts_delay_pressure,
		"delay between panel setup and pressure read."
		"  Default = 0us.");

module_init(ucb1400_ts_init);
module_exit(ucb1400_ts_exit);

MODULE_DESCRIPTION("Philips UCB1400 touchscreen driver");
MODULE_LICENSE("GPL");
