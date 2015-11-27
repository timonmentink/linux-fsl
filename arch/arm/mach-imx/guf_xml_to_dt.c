/*
 * Copyright (C) 2014 Garz & Fricke GmbH
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/guf_xml_config.h>
#include <linux/pwm_backlight.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/mxcfb.h>
#include <linux/platform_data/video-mx3fb.h>
#include <linux/ipu.h>
#include "guf.h"
#include "guf_xml_to_dt.h"

//==============================================
// Keypad command line
//==============================================
// There is a fourth option HDMI. It is not necessary to add it here
// because it's auto-detectable.
// But if GPIO or SPI is used it doesn't make sense to enable HDMI operation.
// Hence, HDMI is only active if KP_I2C is active.
#define KP_GPIO				0x01
#define KP_SPI				0x02
#define KP_I2C				0x04
#define KP_HDMI				0x08
#define KP_KEYPAD			0x10
#define KP_FROMCMDLINE	  0x1000

#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))


static const char * okay = "okay";
static const int okay_len = 5;
static uint32_t keypad = KP_GPIO | KP_I2C;

static void parse_keypad(char * options)
{
	char *option;
	while ((option = strsep(&options, ",")) != NULL)
	{
		if (strcmp(option, "gpio") == 0)
			keypad |= KP_GPIO;
		if (strcmp(option, "i2c") == 0)
			keypad |= KP_I2C;
		if (strcmp(option, "spi") == 0)
			keypad |= KP_SPI;
		if (strcmp(option, "hdmi_i2c") == 0)
			keypad |= KP_HDMI;
		if (strcmp(option, "keypad") == 0)
			keypad |= KP_KEYPAD;
	}
}
static int __init guf_parse_keypad(char *options)
{
	keypad = KP_FROMCMDLINE;

	parse_keypad(options);
	return 0;
}
__setup("keypad=", guf_parse_keypad);

//==============================================
// Device tree manipulation functions 
//==============================================
static int of_set_property( struct device_node * np, const char * propname, const void * value, size_t size)
{
	struct property *newpp;
	int r = 0 ;

	if(!np)
	{
		pr_err("%s, No device node given\n", __func__);
		return -EINVAL;
	}
#ifdef DEBUG
	{
		int i;
		struct property *prop;
		prop = of_find_property(np, propname, NULL);
		pr_info("%s: Old   %s %p", __func__, propname, prop);
		if( prop)
		{
			for( i = 0; i < prop->length; i++)
				pr_info(" 0x%08x ", ((u32*)prop->value)[i]);
		}
		pr_info("\n");
	}
#endif

	newpp = kzalloc(sizeof(*newpp) + size, GFP_KERNEL);
	if (!newpp)
	{
		pr_warn("%s, failed to alloc memory for property\n", __func__);
		r = -ENOMEM;
		goto out;
	}

	newpp->length = size;
	newpp->name = kstrdup(propname, GFP_KERNEL);
	if (!newpp->name) {
		pr_warn("%s, failed to kstrdupc\n", __func__);
		kfree(newpp);
		r = -ENOMEM;
		goto out;
	}
	newpp->value = kmemdup( value, size, GFP_KERNEL);
	if (!newpp->value) {
		pr_warn("%s, failed to kstrdupc\n", __func__);
		kfree(newpp);
		r = -ENOMEM;
		goto out;
	}

#ifdef DEBUG
	{
		int i;
		printk("%s: Write %s Size %d:", __func__, newpp->name, size );
		for( i = 0; i < size/sizeof(u32); i++)
			printk(" 0x%08x", ((u32*)newpp->value)[i]);
		printk("\n");
	}
#endif

	r = of_update_property(np, newpp);
	if(r)
		pr_warn("%s: Failed to update property\n", __func__);

out:
	return r;
}

#ifdef CONFIG_FB
static int of_set_property_u32( struct device_node * np, const char * name, u32 value)
{
	u32 val = cpu_to_be32( value);
	int r = of_set_property( np, name, &val, sizeof(val));
	pr_debug("Set %s to %d\n", name, value);
	if(r)
		pr_err("%s: Failed to set property %s in node %s \n", __func__, name, of_node_full_name(np));
	return r;
}

static void cpu_to_be32_array(u32 * val, int len)
{
	int i;
	for(i = 0; i < len; i++)
		val[i] = cpu_to_be32(val[i]);
}
#endif


/* Looking for the property propname in the node of compatible and sets it to newval.
   If expected_val is given, the property is checked for the previous value
   and fails if it does not match.

   \FIXME What happens if there are multiple nodes with the same compatible field?
*/
static int of_check_and_set_string(const char * compatible, const char * propname, 
	const char * expected_val, const char * newval)
{
	struct device_node *np;
	struct property *pp;
	int r = 0;

	if( !compatible || !propname || !newval)
		return -EINVAL;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
	{
		pr_warn("%s: Couldn't find node: %s\n", __func__, compatible);
		r = -ENODEV;
		goto out;
	}

	pp = of_find_property(np, propname, NULL);
	if(!pp || !pp->value)
	{
		pr_warn("%s: Couldn't find property '%s' in node: %s\n", __func__, propname, compatible);
		r = -ENODEV;
		goto put_node;
	}
	if(expected_val )
	{
		if(strncmp(pp->value, expected_val, pp->length))
		{
			pr_warn("%s: Property 'status' in node: %s, has value '%s', expecting '%s'\n", __func__, 
				compatible, (char*) pp->value, expected_val);
			r = -ENODEV;
			goto put_node;
		}
	}

	r = of_set_property(np, propname, newval, strlen(newval) + 1);

put_node:
	of_node_put(np);
out:
	return r;
}

/* Searches for the node with the given compatible flag, 
   and changes the attribute status from 'disabled' to 'okay'.
*/
static int enable_node_in_dt(const char * compatible)
{
	return of_check_and_set_string( compatible, "status", "disabled", "okay");
}


//=================================================================================
// External function to bring xml config to the devicetree
//=================================================================================


#ifdef CONFIG_FB
int set_backlight_settings_from_xml(guf_xml_data_backlight_t * xml_bl_setting)
{
	int r = 0, ret = 0;
	const char * compatible = "pwm-backlight";
	struct device_node *np, *pwm_np, *pwm_pinctl_np;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
	{
		pr_err("%s: Couldn't find node: %s\n", __func__, compatible);
		ret = -ENOENT;
		goto out;
	}

	pr_debug("%s: XML Frequency %d\n", __func__, xml_bl_setting->frequency);
	if (xml_bl_setting->frequency) {
		u32 pwm_period_ns = 1000000000u / xml_bl_setting->frequency;
		u32 pwms[3];
		r = of_property_read_u32_array( np, "pwms", pwms, 3);
		if(r)
		{
			pr_err("Failed to read pwm property from backlight\n");
			ret |= r;
		}
		else
		{
			pr_debug("%s: Read %0x %0x %0x\n", __func__, pwms[0], pwms[1], pwms[2]);
			pwms[2] = pwm_period_ns;
			pr_debug("%s: Set  %0x %0x %0x\n", __func__, pwms[0], pwms[1], pwms[2]);
			cpu_to_be32_array(pwms, 3);
			r = of_set_property(np, "pwms", pwms, 3 * sizeof(u32));
			if(r)
			{
				pr_err("Failed to set pwm property from backlight\n");
				ret |= r;
			}
		}
	}

	#define LUT_LEN 256
	if(xml_bl_setting->lut)
	{
		cpu_to_be32_array(xml_bl_setting->lut, LUT_LEN);
		r = of_set_property(np, "brightness-levels", xml_bl_setting->lut, LUT_LEN * sizeof(u32));
		if(r)
		{
			pr_err("Failed to set brightness-levels property from backlight\n");
			ret |= r;
		}
		r = of_set_property_u32( np, "default-brightness-level", xml_bl_setting->level_ac);
		if(r)
		{
			pr_err("Failed to set default-brightness-level property from backlight\n");
			ret |= r;
		}
	}

	pr_debug("%s: pwm_pad ctl 0x%05X\n", __func__, xml_bl_setting->padctl_from_xml );
	if(xml_bl_setting->padctl_from_xml)
	{
		#define PIN_CFG_LEN 6
		u32 pwm_pin[PIN_CFG_LEN];
		pwm_np = of_parse_phandle(np, "pwms", 0);
		if (!pwm_np) {
			pr_err( "failed to find pwm phandle, %s\n",	of_node_full_name(np));
			ret = -ENOENT;
			goto pwm_pad_ctl_fail;
		}
		pr_debug("%s: pwm: %s\n", __func__, of_node_full_name(pwm_np));

		pwm_pinctl_np = of_parse_phandle(pwm_np, "pinctrl-0", 0);
		if (!pwm_pinctl_np) {
			pr_err( "failed to find pwm pin ctl phandle, %s\n",	of_node_full_name(pwm_np));
			ret = -ENOENT;
			goto pwm_pad_put_pwm_np;
		}

		pr_debug("%s: pin ctl: %s\n", __func__, of_node_full_name(pwm_pinctl_np));

		r = of_property_read_u32_array( pwm_pinctl_np, "fsl,pins", pwm_pin, PIN_CFG_LEN);
		if(r)
		{
			pr_err("Failed to read pwm pin property from backlight\n");
			goto pwm_pad_put_ctl_np;
		}

		pr_debug("%s: Read  %0x %0x %0x %0x %0x %0x\n", __func__, pwm_pin[0], pwm_pin[1], pwm_pin[2], pwm_pin[3], pwm_pin[4], pwm_pin[5]);
		pwm_pin[5] = xml_bl_setting->padctl;

		pr_debug("%s: Write %0x %0x %0x %0x %0x %0x\n", __func__, pwm_pin[0], pwm_pin[1], pwm_pin[2], pwm_pin[3], pwm_pin[4], pwm_pin[5]);
		cpu_to_be32_array(pwm_pin, PIN_CFG_LEN);
		r = of_set_property(pwm_pinctl_np, "fsl,pins", pwm_pin, PIN_CFG_LEN * sizeof(u32));
		if(r)
		{
			pr_err("Failed to set fsl pins property from backlight\n");
			ret |= r;
		}

#ifdef DEBUG
		r = of_property_read_u32_array( pwm_pinctl_np, "fsl,pins", pwm_pin, PIN_CFG_LEN);
		if(r)
		{
			pr_err("Failed to read pwm pin property from backlight\n");
			goto pwm_pad_put_ctl_np;
		}
		pr_debug("%s: Read  %0x %0x %0x %0x %0x %0x\n", __func__, pwm_pin[0], pwm_pin[1], pwm_pin[2], pwm_pin[3], pwm_pin[4], pwm_pin[5]);
#endif

pwm_pad_put_ctl_np:
		of_node_put(pwm_pinctl_np);
pwm_pad_put_pwm_np:
		of_node_put(pwm_np);
pwm_pad_ctl_fail:;
	}

	of_node_put(np);
out:
	return r;
}

static const char * get_ipu_pix_fmt( int pix_fmt, int depth)
{
	/*
	RGB666		IPU_PIX_FMT_RGB666
	RGB565		IPU_PIX_FMT_RGB565
	RGB24		IPU_PIX_FMT_RGB24
	BGR24		IPU_PIX_FMT_BGR24
	GBR24		IPU_PIX_FMT_GBR24
	YUV444		IPU_PIX_FMT_YUV444
	LVDS666		IPU_PIX_FMT_LVDS666
	YUYV		IPU_PIX_FMT_YUYV
	UYVY		IPU_PIX_FMT_UYVY
	YVYV		IPU_PIX_FMT_YVYU
	VYUY		IPU_PIX_FMT_VYUY

	BGR666		IPU_PIX_FMT_BGR666
	*/

	switch(pix_fmt) 
	{
		case FORMAT_PIXEL_LVDS666:
		case FORMAT_PIXEL_RGB666:	
			return "RGB666";
		case FORMAT_PIXEL_BGR666:	
			return "BGR666";
		// Note: for 24bit framebuffers we use swapped R and B channels
		// compared to previous platforms...
		case FORMAT_PIXEL_RGB24:	
			if(depth == 24)
				return "BGR24";
			else
				return "RGB24";
		case FORMAT_PIXEL_BGR24:	
			if(depth == 24)
			   return "RGB24";
			else
			   return "BGR24";
		case FORMAT_PIXEL_YUV422:	
			return "YUV422P";
		default:
		case FORMAT_PIXEL_RGB565:	
			return "RGB565";
	}
}

int set_video_mode_from_xml(struct guf_xml_data * xml_config)
{
	int r = 0;
	const char * compatible = "fsl,imx6q-ldb";
    const char * compatible_fb = "fsl,mxc_sdc_fb";
	struct device_node *np;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct device_node *native_mode;
	struct device_node *fb_np;
	const char * if_pix_fmt;
	const char * tmp_str;
	guf_xml_data_lvds_t * lvds = &xml_config->lvds;
	u32 pixclock, fbsync;
	u32 power_on_off_timings[5];

	if(xml_config->display_valid)
	{
		pr_info("No display configured in xml settings, display stays disabled.\n");
		return 0;
	}

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
	{
		pr_err("%s: Couldn't find node: %s\n", __func__, compatible);
		r = -ENOENT;
		goto out;
	}
	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		pr_err( "failed to find display phandle, %s\n",	of_node_full_name(np));
		r = -ENOENT;
		goto put_node;
	}
	timings_np = of_find_node_by_name(display_np, "display-timings");
	if (!timings_np) {
		pr_err("%s: could not find display-timings node\n",	of_node_full_name(np));
		r = -ENOENT;
		goto put_display_node;
	}
	native_mode = of_parse_phandle(timings_np, "native-mode", 0);
	if (!native_mode) {
		pr_err("%s: could not find native-mode node\n", __func__);
		r = -ENOENT;
		goto put_display_node;
	}

	r |= of_set_property_u32( native_mode, "hsync-len", xml_config->hsync.width);
	r |= of_set_property_u32( native_mode, "vsync-len", xml_config->vsync.width);
	r |= of_set_property_u32( native_mode, "hfront-porch", xml_config->hsync.end_width);
	r |= of_set_property_u32( native_mode, "hback-porch", xml_config->hsync.start_width );
	r |= of_set_property_u32( native_mode, "vfront-porch", xml_config->vsync.end_width);
	r |= of_set_property_u32( native_mode, "vback-porch", xml_config->vsync.start_width );
	r |= of_set_property_u32( native_mode, "hactive", xml_config->display.xres);
	r |= of_set_property_u32( native_mode, "vactive", xml_config->display.yres);

	if (xml_config->display.pix_clk == 0) 
	{
		pixclock = 
					(   xml_config->display.xres 
					  + xml_config->hsync.start_width
					  + xml_config->hsync.width 
					  + xml_config->hsync.end_width )
				*   (   xml_config->display.yres 
				      + xml_config->vsync.start_width 
					  + xml_config->vsync.width
					  + xml_config->vsync.end_width ) 
				*       xml_config->display.refresh;
	} else {
		pixclock = xml_config->display.pix_clk;
	}
	pr_debug("%s %d Pixclk from xml: %d Hz, Refresh: %d Hz\n", __func__, __LINE__, pixclock, xml_config->display.refresh);

	r |= of_set_property_u32( native_mode, "clock-frequency", pixclock);

	/* change the interpratation of "polarity" and "select_enable" dependent
	   on "original_dc" */
	if (xml_config->display.original_dc != NULL)
	{
		if (!strcmp(xml_config->display.original_dc, "CLCDC")) {
			/* IPU compared to CLCDC uses inverted clock.select_enable and clock.polarity */
			xml_config->clock.select_enable = (xml_config->clock.select_enable == 0);
			xml_config->clock.polarity = (xml_config->clock.polarity == 0);
		}
	}
	else
	{
		pr_err("guf_xml: Your display settings are outdated, please update.\n");
	}

	fbsync =  
		(xml_config->clock.idle_enable ? FB_SYNC_CLK_IDLE_EN: 0) |
		(xml_config->data.oe_polarity ?  0: FB_SYNC_OE_LOW_ACT) |
		(xml_config->clock.polarity ? FB_SYNC_CLK_LAT_FALL : 0) |
		// (config.clock.polarity ? 0 : FB_SYNC_CLK_INVERT) | /* FIXME: Use this line on i.MX53 */
		(xml_config->clock.select_enable ? FB_SYNC_CLK_SEL_EN : 0) | 
		(xml_config->hsync.polarity ? 0: FB_SYNC_HOR_HIGH_ACT ) |
		(xml_config->vsync.polarity ? FB_SYNC_VERT_HIGH_ACT : 0) |
		(xml_config->data.polarity ? FB_SYNC_DATA_INVERT : 0);

	if(xml_config->clock.select_enable)
		pr_err("%s: Warning: clock select enable not supported by fb driver\n", __func__);

	pr_debug("%s %d: fb sync: 0x%08x - %d %d %d %d %d %d %d\n", __func__, __LINE__,fbsync,
		xml_config->clock.idle_enable,
		xml_config->data.oe_polarity ,
		xml_config->clock.polarity,
		xml_config->clock.select_enable,
		xml_config->hsync.polarity,
		xml_config->vsync.polarity,
		xml_config->data.polarity
			);
	r |= of_set_property_u32( native_mode, "fb-sync-flags", fbsync);

	if(lvds->enable)
	{
		if(lvds->mapping)
			r |= of_set_property(np, "mapping", lvds->mapping, strlen(lvds->mapping) + 1);
		if(lvds->mode)
			r |= of_set_property(np, "mode", lvds->mode, strlen(lvds->mode) + 1);
		r |= of_set_property_u32( np, "lvds-channels", lvds->channels);
		r |= of_set_property_u32( np, "sel6_8-polarity", lvds->sel6_8_polarity);
	}

	fb_np = NULL;
	while(1){
		fb_np = of_find_compatible_node( fb_np, NULL, compatible_fb);
		if (!fb_np)
		{
			pr_err("%s: Couldn't find node: %s\n", __func__, compatible_fb);
			r = -ENOENT;
			goto put_display_node;
		}
		if( of_property_read_string(fb_np, "disp_dev", &tmp_str))
			continue;
		if( strcmp( tmp_str, "ldb" ))		// sort out the hdmi channel
			continue;
		break;
	}

	r |= of_set_property_u32( fb_np, "default_bpp", xml_config->format.depth);
	if_pix_fmt = get_ipu_pix_fmt( xml_config->format.pixelformat, xml_config->format.depth);
	r |= of_set_property( fb_np, "interface_pix_fmt", if_pix_fmt, strlen(if_pix_fmt) + 1);

	power_on_off_timings[0] = xml_config->powerseq.poweron_to_signalon;
	power_on_off_timings[1] = xml_config->powerseq.poweron_to_backlighton;
	power_on_off_timings[2] = xml_config->powerseq.backlightoff_before_poweroff;
	power_on_off_timings[3] = xml_config->powerseq.signaloff_before_poweroff;
	power_on_off_timings[4] = xml_config->powerseq.poweroff_to_poweron;
	cpu_to_be32_array(power_on_off_timings, ARRAY_SIZE(power_on_off_timings));
	r |= of_set_property(display_np, "power-on-off-timings", power_on_off_timings, 
		sizeof(power_on_off_timings));
	pr_debug("%s: Set timings %d %d %d %d %d\n", __func__, 
		power_on_off_timings[0], 
		power_on_off_timings[1], 
		power_on_off_timings[2], 
		power_on_off_timings[3], 
		power_on_off_timings[4]); 
	
#if 0
	fb->name = xml_config->display.name;
#endif

	r |= of_set_property(np, "status", okay, okay_len);
	r |= of_set_property(fb_np, "status", okay, okay_len);

	of_node_put(fb_np);
put_display_node:
put_node:
	of_node_put(np);
out:
	return r;
}

int logo_license_from_dt_to_xml(guf_xml_data_logo_license_t * license, guf_xml_data_rotation_t * rotation)
{
	int r = 0;
	const char * compatible = "guf,png-logo";
	struct device_node *np;
	u32 rot;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
	{
		pr_err("%s: Couldn't find node: %s\n", __func__, compatible);
		r = -ENOENT;
		goto out;
	}

	if( rotation)
	{
		rot = rotation->angle;
		rot = cpu_to_be32(rot);
		r = of_set_property(np, "rotation", &rot, sizeof(u32));
		pr_err("XML -> DT %d rotation 0x%08x 0x%08x\n", r, rot, rotation->angle);
	}

	if( !license || !license->valid)
		goto out;

	r = of_set_property(np, "guf,logo-license", license->license, LICENSE_LENGTH * sizeof(u8));

	of_node_put(np);
out:
	return r;
}

/* touch dt node: defined in guf_xml_config.h
   */
static const char *guf_touch_dt_nodes[TOUCH_COUNT] =
{
#define TOUCH_TYPE( id, xml_name, dt_node ) [id] = dt_node,
	TOUCH_TYPES
#undef TOUCH_TYPE
};
int configure_touch_from_xml( const guf_xml_data_touch_t * xml_touch)	
{
	int r = -ENODEV;
	int touch_id = *xml_touch;
	if( touch_id < 0 || touch_id >= TOUCH_COUNT || !guf_touch_dt_nodes[touch_id])
		return -ENODEV;

	r = enable_node_in_dt(guf_touch_dt_nodes[touch_id]);
	if( r)
		pr_err("%s: Failed to enable touch with id %d: Not in devicetree.\n", __func__, touch_id);
	return r;
}
#endif

int set_mac_addr_from_xml(const struct guf_xml_data_network * const network_config)
{
	struct device_node *enet_np;
	unsigned char mac[ETH_ALEN];
	struct property *newmac;
	u8 *macaddr;
	char * compatible = "fsl,imx6q-fec";
	int r = 0;

	if(guf_xml_get_mac_address( network_config, mac))
	{
		pr_warn("%s: MAC address not valid", __func__);
		r = -EINVAL;
		goto out;
	}
	enet_np = of_find_compatible_node(NULL, NULL, compatible);
	if (!enet_np)
	{
		pr_warn("%s: Couldn't find enet node: %s\n", __func__, compatible);
		r = -ENODEV;
		goto out;
	}

	if (!of_get_mac_address(enet_np))
	{
		pr_warn("%s: Couldn't find mac address in node: %s\n", __func__, compatible);
		r = -ENODEV;
		goto put_enet_node;
	}

	newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
	if (!newmac)
	{
		pr_warn("%s, failed to alloc memory for mac\n", __func__);
		r = -ENOMEM;
		goto put_enet_node;
	}

	newmac->value = newmac + 1;
	newmac->length = 6;
	newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
	if (!newmac->name) {
		pr_warn("%s, failed to kstrdupc\n", __func__);
		kfree(newmac);
		r = -ENOMEM;
		goto put_enet_node;
	}

	macaddr = newmac->value;
	memcpy(macaddr, mac, 6 * sizeof(char));

	of_update_property(enet_np, newmac);
put_enet_node:
	of_node_put(enet_np);
out:
	return r;
}


int configure_keypad(char * keypad_from_xml)
{
	struct device_node * np, * cnp;

	pr_debug("%s 0x%x\n", __func__, keypad);

	if( keypad_from_xml)
	{
		if(keypad & KP_FROMCMDLINE){
			pr_warn("Keypad option from commandline overwrites setting from xml\n");
		}else{
			keypad = 0;
			parse_keypad(keypad_from_xml);
		}
	}

	np = of_find_compatible_node(NULL, NULL, "gpio-of-export");
	if( !np) 
	{
		pr_err("%s: failed to find node gpio-of-export\n", __func__);
		return -ENODEV;
	}
	keypad &= ~ KP_FROMCMDLINE;
	if (( keypad & KP_KEYPAD ) && (keypad & (~ KP_KEYPAD )))
	{
		// FIXME KEYPAD + I2C + SPI should work with less gpios for keypad
		pr_warn("Keypad option: both keypad and other function selected in command line, disabling everything except keypad.");
		keypad = KP_KEYPAD;
	}
	if (( keypad & KP_I2C ) && (keypad & KP_HDMI))
	{
		pr_warn("Keypad option: both I2C and HDMI selected in command line, disabling I2C.");
		keypad &= ~ KP_I2C;
	}

	if (keypad & KP_GPIO) 
	{
		pr_debug("%s Configuring KP_GPIO\n", __func__);
		cnp = of_get_child_by_name(np, "gpio-grp-keypad_changeable");
		if(cnp)
		{	
			of_set_property(cnp, "status", okay, okay_len);
			of_node_put(cnp);
		}

		if ((!(keypad & KP_I2C)) &&  (!(keypad & KP_HDMI)))
		{
			cnp = of_get_child_by_name(np, "gpio-grp-keypad_i2c");
			if(cnp)
			{	
				of_set_property(cnp, "status", okay, okay_len);
				of_node_put(cnp);
			}

		}
		if (!(keypad & KP_SPI))
		{
			cnp = of_get_child_by_name(np, "gpio-grp-keypad_spi");
			if(cnp)
			{	
				of_set_property(cnp, "status", okay, okay_len);
				of_node_put(cnp);
			}
		}
	}
	of_node_put(np);

	if (keypad & KP_I2C) {
		pr_debug("%s Configuring KP_I2C (HDMI)\n", __func__);
		np = of_find_node_by_path("/soc/aips-bus@02100000/i2c@021a4000");	//FIXME  may be its better to create an "alternative driver that gets the nodes to enable and disable set in dt ? */
		if(np) of_set_property(np, "status", okay, okay_len);
		of_node_put(np);

		// Enable HDMI only it KP_I2C is selected
		{
			const char * hdmi_nodes[] = {
				"/soc/hdmi_core@00120000",
				"/soc/hdmi_video@020e0000",
				"/soc/hdmi_audio@00120000",
				"/soc/hdmi_cec@00120000",
				"/sound-hdmi",
				"/fb@1",
			};
			int i;
			for(i =0; i < ARRAY_LENGTH(hdmi_nodes);i++)
			{
				pr_debug("%s Enable %d  %s\n",__func__, i, hdmi_nodes[i]);
				np = of_find_node_by_path(hdmi_nodes[i]);
				if(np) of_set_property(np, "status", okay, okay_len);
				else pr_info("%s: Node '%s' not found\n",__func__, hdmi_nodes[i]);
				of_node_put(np);
			}
		}
		
	}
	if (keypad & KP_SPI) {
		pr_info("%s Configuring KP_SPI\n", __func__);
		np = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02008000");	//FIXME  may be its better to create an "alternative driver that gets the nodes to enable and disable set in dt ? */
		if(np)
		{
			cnp = of_get_child_by_name(np, "spidev0");
			if( cnp)
			{
				of_set_property(cnp, "status", okay, okay_len);
				of_node_put(cnp);
			}
			cnp = of_get_child_by_name(np, "spidev1");
			if( cnp)
			{
				of_set_property(cnp, "status", okay, okay_len);
				of_node_put(cnp);
			}

			if (( keypad & KP_I2C ) || ( keypad & KP_HDMI )) {
				u32 num_cs = cpu_to_be32(2);
				of_set_property(np, "fsl,spi-num-chipselects", &num_cs, sizeof(u32));
			}else{
				cnp = of_get_child_by_name(np, "spidev2");
				if( cnp)
				{
					of_set_property(cnp, "status", okay, okay_len);
					of_node_put(cnp);
				}
			}
			of_set_property(np, "status", okay, okay_len);
		}
		of_node_put(np);
	}
	if (keypad & KP_HDMI) {
		// FIXME enable HDMI related nodes
	}
	if (keypad & KP_KEYPAD) {
		int r;
		r = enable_node_in_dt("gpio-matrix-keypad");
		if( r)
			pr_err("%s: Failed to enable gpio-matrix-keypad\n", __func__);

	}
	return  0;
}


/*************************************************************************/
/*************************************************************************/
int integrate_devtree_from_xml(const struct guf_xml_devtree * devtree)
{
	struct token{
		char * val;
		bool is_special;
		struct token * next;
		struct token * prev;
	} ;

	auto int parse_node( const struct device_node * np, const char * name, struct token ** tokens);
	bool is_special(char c)
	{
		switch(c){
		case '{': case '}': case '<': case '>': case '=': 
		case ';': case '[': case ']': case '"': case ':':
		case ',':
			return true;
		default:
			return false;
		}
	}
	bool is_space(char c)
	{
		switch(c){
		case ' ': case '\t': case '\n':	case '\r':
			return true;
		default:
			return false;
		}
	}
	enum{
		SUCCESS,
		UNEXPECTED_END_OF_DATA,
		SYNTAX_ERROR,
		FAILED_TO_FIND_NODE,
		TREE_ERROR,
		OUT_OF_MEM,
	};
	/* Copies the content of p to a new memregion, if alloc fails, p is not changed, otherwise freed */
	void * kremalloc( void * p, size_t oldsize, size_t newsize)
	{
		void * tmp = kmalloc( newsize, GFP_KERNEL);
		size_t smaller = newsize > oldsize ? oldsize : newsize;
		if( !tmp) return 0;
		if(p){
			memcpy( tmp, p, smaller);
			kfree(p);
		}
		return tmp;
	}
	int get_token(struct token ** tok, struct token ** tokens)
	{
		if(!*tokens)
			return UNEXPECTED_END_OF_DATA;
		*tok = *tokens;
		if(!(*tok)) 
			return UNEXPECTED_END_OF_DATA;
		*tokens = (*tokens)->next;
		//pr_info("xml %s %s\n", __func__, (*tok)->val?(*tok)->val:"null");
		return 0;
	}
	int get_name(char ** name, struct token ** tokens)
	{
		int ret;
		struct token * tok;
		if((ret = get_token(&tok, tokens))) return ret;
		if(tok->is_special)	return SYNTAX_ERROR;
		*name = tok->val;
		return 0;
	}
	int get_special(char * c, struct token ** tokens)
	{
		int ret;
		struct token * tok;
		if((ret = get_token(&tok, tokens))) return ret;
		if(!tok->is_special)	return SYNTAX_ERROR;
		*c = tok->val[0];
		return 0;
	}
	int get_name_and_special(char ** name, char * c, struct token ** tokens)
	{
		int ret;
		if((ret = get_name( name, tokens))) return ret;
		return get_special(c, tokens);
	}
	int parse_bytes( struct token ** tokens)
	{
		int ret;
		struct token * tok;
		pr_warn("%s %d Not implemented yet\n",__func__, __LINE__);
		while(true)
		{
			if((ret = get_token(&tok, tokens))) return ret;
			if(tok->is_special && tok->val[0] == '>')
				return 0;
			pr_debug("xml bytes %s\n", tok->val);
		}
	}
	int parse_string(struct token ** tokens)
	{
		int ret;
		struct token * tok;
		pr_warn("%s %d Not implemented yet\n",__func__, __LINE__);
		while(true)
		{
			if((ret = get_token(&tok, tokens))) return ret;
			if(tok->is_special && tok->val[0] == '"')
				return 0;
			pr_debug("xml String %s\n", tok->val);
		}
	}
	int parse_cell(struct token ** tokens, void ** tdata, size_t * tsize, size_t * csize)
	{
		int ret;
		u32	val, val_be32;
		struct token * tok;

		while(true)
		{
			if(( ret = get_token(&tok, tokens))) return ret;
			if(tok->is_special && tok->val[0] == '>')
				return 0;
			val = simple_strtoul(tok->val, 0, 0);
			val_be32 = cpu_to_be32( val);
			if( *tsize < *csize + sizeof(u32))
			{
				size_t newsize = *tsize * 2;
				void * tmp = kremalloc( *tdata, *tsize, newsize);
				if( !tmp)
				{
					kfree(*tdata);
					*tdata = 0;
					*tsize = 0;
					*csize = 0;
					return OUT_OF_MEM;
				}
				*tdata = tmp;
				*tsize = newsize;
			}
			memcpy( *tdata + *csize, (void *) &val_be32, sizeof(u32));
			*csize += sizeof(u32);

			pr_debug("xml Cell %s 0x%x %d\n", tok->val, val, *csize);
		}
	}
	int write_property( struct device_node * np, char * name, void * tdata, size_t csize)
	{
		struct property *prop;
		int ret;
		if(!( prop = kzalloc(sizeof(struct property) + csize, GFP_KERNEL))){
			ret = OUT_OF_MEM;
			goto free_data;
		}
		if( !( prop->name = kstrdup(name, GFP_KERNEL))){
			ret = OUT_OF_MEM;
			kfree(prop);
			goto free_data;
		}
		prop->value = prop + 1;	// we allocated the data memory directly after the struct
		memcpy( prop->value, tdata, csize);
		prop->length = csize;

		ret = of_update_property( np, prop);

		free_data:
			kfree(tdata);
		return ret;
	}


	int parse_property(struct token ** tokens, void ** tdata, size_t * tsize, size_t * csize)
	{
		int ret;
		char c;


		if(( ret = get_special(&c, tokens))) return ret;
		switch(c)
		{
		case '<':
			if((ret = parse_cell( tokens, tdata, tsize, csize))) return ret;
			break;
		case '"':
			if((ret = parse_string( tokens))) return ret;
			break;
		case '[':
			if((ret = parse_bytes( tokens))) return ret;
			break;
		}

		if(( ret = get_special(&c, tokens))) return ret;
		switch(c)
		{
		case ',':
			return parse_property( tokens, tdata, tsize, csize);
		case ';':
			return ret;
		}
		return SYNTAX_ERROR;
	}
	int parse_next_token(struct device_node * np, struct token ** tokens)
	{
		int ret;
		struct token * tok;
		char c;

		if(( ret = get_token( &tok, tokens))) return ret;

		if( tok->is_special && tok->val[0] == '}' ){
			if(( ret = get_special( &c, tokens))) return ret;
			if( c == ';') return 0;
			return SYNTAX_ERROR;
		}
		if( tok->is_special ) return SYNTAX_ERROR;

		if(( ret = get_special( &c, tokens))) return ret;

		switch(c){
		case '{':
			if(( ret = parse_node(np, tok->val, tokens))) return ret;
			break;
		case '=':{
			size_t tsize = sizeof(int) * 10, csize = 0;
			void * tdata = kremalloc( 0, 0, tsize);
			pr_debug("%s %d Open Prop: %s\n", __func__, __LINE__, tok->val);
			if(( ret = parse_property( tokens, &tdata, &tsize, &csize)	))
			{ 
				kfree(tdata); 
				return ret; 
			}
			pr_debug("%s %d Writing Prop: %s length %d\n", __func__, __LINE__, tok->val, csize);
			write_property( np, tok->val, tdata, csize);
			pr_debug("%s %d Close Prop: %s\n", __func__, __LINE__, tok->val);
			break;
			}
		}
		return parse_next_token( np, tokens);
	}
	int parse_node( const struct device_node * np, const char * name, struct token ** tokens)
	{
		int ret;
		struct device_node * pnp;
		// is a node
		pnp = of_get_child_by_name(np, name);
		if(!pnp) return FAILED_TO_FIND_NODE;

		// FIXME Create node if not available
		pr_debug("%s %d Opened Node: %s\n", __func__, __LINE__, name);

		ret = parse_next_token( pnp, tokens);

		of_node_put(pnp);

		pr_debug("%s %d Closed Node: %s %d\n", __func__, __LINE__, name, ret);

		return ret;
	}
	int parse_devtree( struct token ** tokens)
	{ 
		int ret;
		struct device_node * pnp;
		char * name, c;

		if(( ret = get_name_and_special(&name, &c, tokens))) return ret;

		if(c != '{') return SYNTAX_ERROR;

		pnp = of_find_node_by_name(NULL, name);
		if(!pnp)	pnp = of_find_node_by_path(name);
		if(!pnp) return FAILED_TO_FIND_NODE;

		pr_debug("%s %d Opened Node: %s\n", __func__, __LINE__, name);

		ret = parse_next_token( pnp, tokens);

		of_node_put(pnp);

		pr_debug("%s %d Closed Node: %s %d\n", __func__, __LINE__, name, ret);

		return ret;
	}

	int tokenize(const char * inputdata, struct token * dst)
	{
		__label__ out_of_memory_failure;
		enum{
			decode,
			decoding_word,
			decoding_string,
			decoding_string_escape,
			decoding_comment_begin,
			decoding_comment,
			decoding_comment_end,
		}decode_state = decode;
		int pos = 0;
		struct token * tok = dst;
		#define MAX_TOKEN_LEN 512
		char temp[MAX_TOKEN_LEN];
		char c;
		const char * cc = inputdata;

		void add_char(char c)
		{
			temp[pos++] = c;
			if( pos > MAX_TOKEN_LEN )
			{
				pr_err("%s: Error decoding device tree: Token length must not excceed %d bytes\n", __func__, MAX_TOKEN_LEN);
				goto out_of_memory_failure;
			}
		}
		void add_split(bool is_special )
		{
			struct token * next; 
			char * val;

			add_char(0);

			val = kstrdup(temp, GFP_KERNEL);
			if( !val){
				pr_err("%s: Error decoding device tree: Failed to allocate memory\n", __func__);
				goto out_of_memory_failure;
			};

			next = kmalloc( sizeof(struct token), GFP_KERNEL);
			if( !next){
				kfree(val);
				pr_err("%s: Error decoding device tree: Failed to allocate memory\n", __func__);
				goto out_of_memory_failure;
			};

			tok->next = next;
			next->prev = tok;
			tok = next;
			tok->val = val;
			tok->is_special = is_special;
			tok->next = 0;
			pos = 0;
		}
		void add_special(char c)
		{
			add_char(c);
			add_split(true);
		}

		for( c = *cc; c; cc++, c = *cc){
			next_try:
			switch(decode_state)
			{
			case decode:
				if(is_space(c))	continue;
				if( c == '/'){
					decode_state = decoding_comment_begin;
					continue;
				}
				if(is_special(c)){
					add_special(c);
					if( c == '"') 
						decode_state = decoding_string;
					continue;
				}

				decode_state = decoding_word;
				add_char(c);
				break;
			case decoding_word:
				if(is_space(c)){
					add_split(false);
					decode_state = decode;
					continue;
				}
				if(is_special(c) && c != ',' ){
					add_split(false);
					add_special(c);
					decode_state = decode;
					continue;
				}
				add_char(c);
				break;
			case decoding_string:
				if( c == '"')
				{
					add_split(false);
					add_special(c);
					decode_state = decode;
					continue;
				}
				if( c == '\\'){
					decode_state = decoding_string_escape;
					continue;
				}
				add_char(c);
				continue;
			case decoding_string_escape:
				decode_state = decoding_string;
				add_char(c);
				break;
			case decoding_comment_begin:
				if( c == '*' ){
					decode_state = decoding_comment;
				}else{
					add_char('/');
					decode_state = decoding_word;
					goto next_try;
				}
				break;
			case decoding_comment:
				if( c == '*' )
					decode_state = decoding_comment_end;
				break;
			case decoding_comment_end:
				if( c == '/' )
					decode_state = decode;
				else
					if( c == '*' )
						decode_state = decoding_comment_end;
					else
						decode_state = decoding_comment;
				break;
			}
		}

		add_char(0);

		return 0;

		out_of_memory_failure:
		return -1;
	}

	for( ; devtree; devtree=devtree->next)
	{
		struct token head;
		struct token * cur_token;
		int ret;

		if(( ret = tokenize( devtree->entry, &head)))
		{
			pr_err("%s: Error while tokenize the device tree string: %d\n", __func__, ret);
			goto error_free;
		}

		// dump result
#ifdef DEBUG
		for( cur_token = head.next; cur_token; cur_token = cur_token->next)
		{
			pr_debug("%s %s %s \n",__func__, cur_token->val, cur_token->is_special ? " *": "");
		}
#endif

		cur_token = head.next;
		if((ret = parse_devtree( &cur_token ))){
			pr_err("%s: Failed to decode devicetree entry\n", __func__); 
			switch(ret)
			{
			case UNEXPECTED_END_OF_DATA:
				pr_err("%s: Unexpected end of input data\n", __func__ ); 
				break;
			case SYNTAX_ERROR:
				pr_err("%s: Syntax error\n", __func__); 
				break;
			case FAILED_TO_FIND_NODE:
				pr_err("%s: Failed to find node in device tree\n", __func__); 
				break;
			case TREE_ERROR:
				pr_err("%s: Tree Error while parsing input data\n", __func__); 
				break;
			}
			pr_err("%s: Error happend before: %s\n", __func__, cur_token ? cur_token->val ?cur_token->val : "-no data-" : "null" );
		}

error_free:
		for( cur_token = head.next; cur_token; )
		{
			struct token * temp = cur_token;
			cur_token = cur_token->next;
			if(temp->val) kfree(temp->val);
			kfree(temp);
		}
	}
	return 0;
}

int update_devicetree_to_board_revision( int major, int minor)
{
	int ret = 0, i = 0;
	struct device_node * np, *cnp = NULL, * tnp;
	struct property *pp, *npp;
	const char * name = "hw_revision_fixup", *propname = "revisions";
	u32 _major, _minor;
	u32 overwrite[3];	// 3 fields: major, minor, phandle


	if( !( np = of_find_node_by_name(0, name)))	{
		pr_debug("%s: node '%s' not available\n", __func__, name);
		return -ENODEV;
	}
	while(1){	
		if( ( ret = of_property_read_u32_index(np, propname, i * 2, &_major)))	{
			if( ret == -EOVERFLOW && i > 0 ){// 
				ret = 0;
				pr_err("%s: Found no match for revision %d.%d, don't apply any changes\n", __func__, major, minor);
			}else
				pr_err("%s: Error while reading node '%s' property '%s'\n", __func__, name, propname);
			goto out;
		}
		if( ( ret = of_property_read_u32_index(np, propname, i * 2 + 1, &_minor)))	{
			pr_err("%s: Error while reading minor revision in node '%s' property '%s' for major %d ( line %d)\n", __func__, name, propname, _major, i);
			goto out;
		}
		if( _major == major && _minor == minor)
		{
			if( i == 0){
				pr_debug("%s: Board revision is default, don't apply any changes\n", __func__ );
				ret = 0; goto out;
			}else{
				pr_debug("%s: Apply board revision changes for %d.%d\n", __func__, _major, _minor );
				break;
			}
		}
		i++;
	}

	// Walk through all nodes, 
	while((cnp = of_get_next_child( np, cnp)))
	{
		//check the 'overwrite' property, 
		if(of_property_read_u32_array( cnp, "overwrite", overwrite, 3)){
		   pr_err("%s: %s has no valid 'overwrite' property\n", __func__, cnp->name); continue;;
		}

		if(!( overwrite[0] == _major && overwrite[1] == _minor)) continue;


		if(!( tnp = of_find_node_by_phandle( overwrite[2] ))){
		   pr_err("%s: %s Could not find node for handle %d\n", __func__, cnp->name, overwrite[2]);	
			continue;
		}

		// Loop through all properties and copy/move them to the target property
		for (pp = cnp->properties; pp; pp = pp->next) {
			if (0 == of_prop_cmp(pp->name, "overwrite")) continue;

			if( !(npp = kmemdup(pp, sizeof(struct property),GFP_KERNEL))){
				pr_err("%s: Failed to allocate memory for new property\n", __func__);
				goto put_node;
			}

			of_update_property(tnp, npp);
		}

put_node:
		of_node_put(tnp);
	}

out:
	of_node_put(np);

	return ret;
}

