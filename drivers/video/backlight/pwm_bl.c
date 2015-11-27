/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		*levels;
	unsigned int		max_level;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);

	int gpio_bl_power;
	int gpio_bl_power_act_low;
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	pr_debug("%s\n",__func__);
	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if( pb->gpio_bl_power)
	{
		int val = (brightness > 0 ) ? !pb->gpio_bl_power_act_low : pb->gpio_bl_power_act_low;
		pr_debug("%s Set %d to %d\n", __func__, pb->gpio_bl_power, val);
		gpio_set_value_cansleep(pb->gpio_bl_power, val);
	}

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	pr_debug("%s: Brightness: %d\n",__func__, brightness);
	pr_debug("%s: max: %d\n",__func__, max);
	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		int duty_cycle;

		pwm_enable(pb->pwm);

		if (pb->levels) {
			duty_cycle = pb->levels[brightness];
			max = pb->max_level;

			pr_debug("%s: max_level: %d\n",__func__, max);
		} else {
			duty_cycle = brightness;
		}

		pr_debug("%s: Duty input : %d\n",__func__, duty_cycle);
		pr_debug("%s: pb->lth_brightness: %d\n",__func__, pb->lth_brightness);
		pr_debug("%s: pb->period: %d\n",__func__, pb->period);
		pr_debug("%s: max: %d\n",__func__, max);
		
		{
			uint64_t temp = pb->period - pb->lth_brightness;
			temp = temp * duty_cycle;
			do_div( temp, max);
			if((temp >> 32) > 0)
			{
				pr_err( "%s: Error while scaling the pwm duty cycle\n", __func__);
				duty_cycle = max;
			}else
			{
				duty_cycle = temp;
				duty_cycle += pb->lth_brightness;
			}
		}

		pr_debug("%s: Duty scaled: %d\n",__func__, duty_cycle);
		pwm_config(pb->pwm, duty_cycle, pb->period);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	pr_debug("%s\n",__func__);
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	int length;
	u32 value, default_power;
	int ret;
	const char * bl_name;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	if( 0 == of_property_read_string(node, "sysfs-name", &bl_name ))
		dev_set_name(dev, bl_name);

	/* determine the number of brightness levels */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return -EINVAL;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

#ifdef DEBUG
		for(ret = 0; ret < data->max_brightness; ret++)
			pr_info("pwm %d 0x%04x\n", ret, data->levels[ret]);
#endif

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;
		ret = of_property_read_u32(node, "default-blank",
					   &default_power);
		if (!ret)
			data->dft_power = !!default_power;
		else
			data->dft_power = FB_BLANK_UNBLANK;

		data->dft_brightness = value;
		data->max_brightness--;
	}

	/*
	 * TODO: Most users of this driver use a number of GPIOs to control
	 *       backlight power. Support for specifying these needs to be
	 *       added.
	 */
	data->gpio_bl_power = of_get_named_gpio( node, "bl-pwr-gpio", 0);

	if( data->gpio_bl_power> 0)
	{
		if( 0 != of_property_read_u32( node, "bl-pwr-act-low", &data->gpio_bl_power_act_low))
			data->gpio_bl_power_act_low = 0;
		ret = gpio_request (data->gpio_bl_power, "backlight power");
		if(ret)
		{
			dev_err(dev, " Failed to request gpio %d: %d\n", data->gpio_bl_power, ret);
			data->gpio_bl_power = 0;
		}
		ret = gpio_direction_output (data->gpio_bl_power, data->gpio_bl_power_act_low );
		if(ret)
		{
			dev_err(dev, " Failed to set gpio %d to output: %d\n", data->gpio_bl_power, ret);
			data->gpio_bl_power = 0;
		}
	}else
	{
		data->gpio_bl_power = 0;
	}
	pr_debug("%s bl power gpio: %d, act low: %d\n", __func__, data->gpio_bl_power, data->gpio_bl_power_act_low);


	return 0;
}

static struct of_device_id pwm_backlight_of_match[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	unsigned int max;
	int ret;

	if (!data) {
		ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (data->levels) {
		pb->max_level = data->max_level ? data->max_level : data->levels[data->max_brightness];
		max = pb->max_level;
		pb->levels = data->levels;
	} else
		max = data->max_brightness;

	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;

	pb->gpio_bl_power = data->gpio_bl_power;
	pb->gpio_bl_power_act_low = data->gpio_bl_power_act_low;

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");

		pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
		if (IS_ERR(pb->pwm)) {
			dev_err(&pdev->dev, "unable to request legacy PWM\n");
			ret = PTR_ERR(pb->pwm);
			goto err_alloc;
		}
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");

	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data.
	 */
	if (data->pwm_period_ns > 0)
		pwm_set_period(pb->pwm, data->pwm_period_ns);

	pb->period = pwm_get_period(pb->pwm);
	pb->lth_brightness = data->lth_brightness * (pb->period / max);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_alloc;
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	bl->props.brightness = data->dft_brightness;
	bl->props.power = data->dft_power;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->exit)
		pb->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
			 pwm_backlight_resume);

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name		= "pwm-backlight",
		.owner		= THIS_MODULE,
		.pm		= &pwm_backlight_pm_ops,
		.of_match_table	= of_match_ptr(pwm_backlight_of_match),
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

