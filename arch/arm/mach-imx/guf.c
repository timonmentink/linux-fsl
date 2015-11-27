/*
 * Copyright (C) 2013 Garz & Fricke GmbH
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/guf_xml_config.h>
#include <linux/memblock.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "common.h"
#include "hardware.h"
#include "mxc.h"
#include "iomux-v3.h"

#include "guf.h"


#define XML_BUFFER_SIZE 64*1024	 /* cur_tokenly not used, just to remember: Redboot allows 64k for the xml memory. */

static uint32_t phys_xml_parameter_address = 0;
static struct guf_xml_data config;

#ifdef CONFIG_FB
//	extern iomux_v3_cfg_t backlight_pwm_pad;
#endif


int guf_init_gpio_group(struct gpio *gpios, int num, bool bExport, bool bFixedDir)
{
	int i, err;
	err = gpio_request_array(gpios, num);
	if (!err & bExport)
	{
		for (i = 0; i < num; i++)
			gpio_export(gpios[i].gpio, bFixedDir ? 0 : 1);
	}

	return err;
}

extern int __init ip_auto_config_setup(char *addrs);
int guf_setup_network_from_xml(const struct guf_xml_data_network * config )
{
	char ip_string[128]; 
	/* Set IP configuration */ 
	if (config->dhcp) 
		sprintf(ip_string, "dhcp"); 
	else 
		sprintf(ip_string, "%s:%s:%s:%s:%s:eth0:off", config->ip_address, 
			config->gateway, config->gateway, config->netmask, config->hostname); 
	pr_debug("%s: ipstring from xml: %s\n", __func__, ip_string);
	ip_auto_config_setup(ip_string);
	return 0;
}

/******************************************************************************/
/* XML parameter setup                                                        */
/* Duplicated parameter name: xmlram and rbxmlram. rbxmlram is the parameter  */
/* used by redboot                                                            */
/* It was renamed for Flash-n-Go to xmlram. Functionality is the same         */
/******************************************************************************/
static int __init guf_get_xml_params(char *options)
{
	int ret;
	if (sscanf(options, "%x", &phys_xml_parameter_address) != 1) {
		pr_err("Garz & Fricke platform: no XML parameters given by the bootloader. Ignoring.\n");
		phys_xml_parameter_address = 0U;	/* to be sure */
		return 0;
	}
	pr_info("Garz & Fricke platform: xml data stored at 0x%08X\n", phys_xml_parameter_address);

	if( ( ret = memblock_remove( phys_xml_parameter_address , SZ_1M)))
	{
		pr_err("Garz & Fricke platform: Failed to reserve xml memory 0x%08X: %d\n", phys_xml_parameter_address, ret);
	}

	return 0;
}
early_param("xmlram", guf_get_xml_params);

static inline int __init guf_get_rbxml_params(char *options)
{
	return guf_get_xml_params(options);
}
early_param("rbxmlram", guf_get_rbxml_params);

/*******************************************************************/
/* Parse the complete xml configuration and store it in            */
/* the config struct                                               */
/*******************************************************************/
struct guf_xml_data * guf_parse_xml_parameters(void)
{
	int ret;
	char *buffer = NULL, *machine = NULL;
	struct ezxml *xml_cfg = NULL;


	if (phys_xml_parameter_address == 0) {
		pr_err("Garz & Fricke platform: xmlram parameter missing in kernel command line\n");
		goto out_error;
	}

	buffer = (char*)phys_to_virt(phys_xml_parameter_address);
	if(!buffer){
		pr_err("Garz & Fricke platform: Could not get virtual address for XML parameters.\n");
		goto out_error;
	}

	pr_info("%s 0x%08x 0x%08x\n", __func__, (unsigned int) phys_xml_parameter_address, (unsigned int) buffer);
#ifdef DEBUG
	{
		char tmp[50];
		memcpy(tmp, buffer,49);
		tmp[49] = 0;
		pr_debug("Buffer contains: %s\n", tmp);
	}
#endif

	xml_cfg = guf_get_config(buffer);
	if (xml_cfg == 0) {
		pr_err("Garz & Fricke platform: Could not get XML parameters\n");
		goto out_error;
	}

	if (get_network_configuration(xml_cfg, &config.network) != 0) {
		pr_err("Garz & Fricke platform: Could not get network configuration.\n");
		goto out_free;
	}

#ifdef CONFIG_FB
	/* Save default padctl for PWM pin before reading display settings */
	/* FIXME
	 * config.backlight.padctl = (backlight_pwm_pad & MUX_PAD_CTRL_MASK) >> MUX_PAD_CTRL_SHIFT;
	 * */

	if (cpu_is_imx6q() || cpu_is_imx6dl()) {
		machine = "mx6";
	} else if (cpu_is_mx53()) {
		machine = "mx53";
	} else if (cpu_is_mx35()) {
		machine = "mx35";
	}
	config.display_valid = get_display_configuration(xml_cfg, &config, machine);
	get_logo_license(xml_cfg, &config);
	get_touch_configuration(xml_cfg, &config);
#endif

	get_keypad_setting(xml_cfg, &config.keypad );
	guf_get_devtree_settings(xml_cfg, &config.devtree);

	guf_free_config(xml_cfg);
	if( ( ret = memblock_free( phys_xml_parameter_address , SZ_1M)))
	{
		pr_err("%s: failed to free xml memory block: %d\n", __func__, ret);
	}
	phys_xml_parameter_address = 0;

	return &config;

out_free:
	guf_free_config(xml_cfg);
out_error:
	pr_warn("%s: Error parsing the xml configuration\n", __func__);
	return 0;
}

/*******************************************************************/
/* Functions to use the loaded xml configuration                   */
/*******************************************************************/

unsigned int guf_get_number_of_lvds_channels(void)
{
	if (config.lvds.enable)
		return config.lvds.channels;
	else
		return 0;
}

guf_xml_data_touch_t guf_get_touch_xml_data(void)
{
	return config.touch;
}

int guf_xml_get_mac_address( const struct guf_xml_data_network * const network_config, u8 mac_addr[ETH_ALEN])
{
	if( 0 == network_config || !network_config->valid)
	{
		pr_warn("guf_xml: xml network config not valid yet\n");
		return -EPERM;
	}
	if( ! is_valid_ether_addr(network_config->mac_addr.sa_data))
	{
		pr_warn("guf_xml: mac address not valid\n");
		return -EINVAL;
	}
	if( mac_addr){
		int i;
		for(i = 0; i < 6; i++) {
			mac_addr[i] = network_config->mac_addr.sa_data[i];
		}
	}
	return 0;
}
