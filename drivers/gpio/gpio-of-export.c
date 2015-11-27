/* drivers/gpio/gpio-of-export.c
 *
 * Copyright (C) 2014 Garz und Fricke, <jonas.hoeppner@garz-fricke.com>
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "asm/io.h"

static int gpio_of_export_probe(struct platform_device *pdev)
{
	struct device_node *np, *cnp;
	np = pdev->dev.of_node;


#ifdef DEBUG
	{
		void * iomuxc;
		iomuxc = ioremap( 0x20e0000, 0x4000);
		if(iomuxc)
		{
		
			pr_err("%s 0x%p: 0x%08x 0x%p: 0x%08x \n", __func__, 
				(iomuxc + 0x174), *((u32*)( iomuxc + 0x174 )),
				(iomuxc + 0x488), *((u32*)( iomuxc + 0x488 ))
				);
			iounmap(iomuxc);
		}else
		{
			pr_err("%s Failed to remoap iomuxc register\n", __func__);
		}

	}
#endif

	if( !np)
		return -ENODEV;

	for_each_child_of_node(np, cnp) 
	{
		int count, i, flags,gpio, output, ret, changeable;
		const char * status;
		pr_debug("%s %s %s\n", __func__, of_node_full_name(np), of_node_full_name(cnp));

		if( !of_property_read_string(cnp, "status", &status  ))
		{
			if(!strcmp(status, "disabled"))
			{
				pr_debug("%s disabled\n", of_node_full_name(cnp));
				continue;
			}
		}

		output = !!of_get_property(cnp, "direction-output", 0);
		changeable = !!of_get_property(cnp, "direction-changeable", 0);

		count = of_gpio_count(cnp);
		pr_debug("%s %d gpios\n", of_node_full_name(cnp), count);

		

		for( i= 0; i < count; i++)
		{	
			enum of_gpio_flags flag = 0;
			gpio = of_get_gpio_flags(cnp,i, &flag);

			flags =	flag |	GPIOF_EXPORT;

			if( flag & OF_GPIO_ACTIVE_LOW)
				flags |= GPIOF_ACT_LOW;
			else
				flags &= ~GPIOF_ACT_LOW;

			if(output)
				flags &= ~GPIOF_DIR_IN;
			else
				flags |= GPIOF_DIR_IN;
			if(changeable)
				flags |= GPIOF_EXPORT_CHANGEABLE;
			else
				flags &= ~GPIOF_EXPORT_CHANGEABLE;
			
			pr_debug("%s % 3d 0x%01x 0x%01x\n", __func__, gpio, flag, flags);
			ret =  gpio_request_one(gpio, flags , "exported");

			if( ret)
			{
				dev_warn(&pdev->dev, "Failed to request gpio %d from %s", gpio, of_node_full_name(cnp));
				continue;
			}

			//\FIXME keep reference to free the gpios on exit
		}	
	}
	return 0;
}
static int gpio_of_export_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s\n", __func__);
	return 0;
}

static struct of_device_id gpio_of_export_dt_idtable[] = {
	{ .compatible = "gpio-of-export", .data = 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gpio_of_export_idtable);

static const struct platform_device_id gpio_of_export_id[] =
{
	{ "gpio-export", 0 },
	{ }
};
MODULE_DEVICE_TABLE (i2c, gpio_of_export_id);

static struct platform_driver gpio_of_export_driver =
{
	.driver = {
		.owner = THIS_MODULE,
		.name = "gpio-export",
		.of_match_table = gpio_of_export_dt_idtable,
	},
	.id_table = gpio_of_export_id,
	.probe    = gpio_of_export_probe,
	.remove   = gpio_of_export_remove,
};

static int __init gpio_of_export_init(void)
{
	return platform_driver_register (&gpio_of_export_driver);
}
module_init (gpio_of_export_init);

static void __exit gpio_of_export_exit (void)
{
	platform_driver_unregister (&gpio_of_export_driver);
}
module_exit (gpio_of_export_exit);

MODULE_AUTHOR ("Jonas Hoeppner <jonas.heoppner@garz-fricke.com>");
MODULE_DESCRIPTION ("gpio of export");
MODULE_LICENSE (GPL);
