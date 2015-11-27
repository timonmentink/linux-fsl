/*
 * ezxml - xml parser from eCos
 *
 * Copyright (C) 2004-2006 Aaron Voisine <aaron@voisine.org>
 * Copyright (C) 2005 eCosCentric Ltd
 * Copyright (C) 2010 Robert Schwebel <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#include <linux/guf_xml_config.h>
#include <linux/ezxml.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/errno.h>

/* touch names: these strings must match the attribute "name"
   of an XML touch configuration file */
static const char *guf_xml_data_touch_names[TOUCH_COUNT] =
{
#define TOUCH_TYPE( id, xml_name, dt_node ) [id] = xml_name,

	TOUCH_TYPES

#undef TOUCH_TYPE
};

struct ezxml* xml_find_setting(struct ezxml *xml_variable, const char *setting)
{
	struct ezxml *xml_setting;

	if (!xml_variable || !setting)
		return NULL;

	xml_setting = ezxml_child(xml_variable, "setting");
	if (!xml_setting)
		return NULL;

	for (; xml_setting; xml_setting = ezxml_next(xml_setting)) {
		if (ezxml_attr(xml_setting, "key")) {
			if (!strcmp(ezxml_attr(xml_setting, "key"), setting))
				return xml_setting;
		}
	}

	return NULL;
}

	struct ezxml*
guf_get_config(char *string)
{
	struct ezxml *xml_img = NULL;
	struct ezxml *xml_cfg = NULL;
	char * end;
	int len;

	if (!string)
		goto out_error;

	if (0 != memcmp(string, "<?xml", 5)) 
	{
		pr_err("Garz & Fricke platform: XML does not begin with tag\n");
		goto out_error;
	}

	end = strstr(string, "</configurationFile>");
	if( !end)
	{
		pr_err("Garz & Fricke platform: XML end tag couldn't be found\n");
		goto out_error;
	}
	len = end - string + 19;


	xml_img = ezxml_parse_str(string, len);
	if (!xml_img)
	{
		pr_err("Garz & Fricke platform: error while parsing XML tree\n");
		goto out_error;
	}

	xml_cfg = ezxml_child(xml_img, "variables");
	if (!xml_cfg)
	{
		pr_err("Garz & Fricke platform: could not find 'variables' in XML\n");
		goto out_free;
	}

	return xml_cfg;

out_free:
	ezxml_free(xml_img);

out_error:
	return NULL;
}

	void
guf_free_config(struct ezxml *xml_cfg)
{
	if (xml_cfg && xml_cfg->parent)
		ezxml_free(xml_cfg->parent);
}

static char * get_string_from_xml(struct ezxml * xml_cfg, const char * const key)
{
	struct ezxml *xml_parent = NULL;
	pr_debug("guf_xml: read key %s\n", key);
	xml_parent = xml_find_setting(xml_cfg, key);
	if (!xml_parent)
	{
		pr_warn("guf_xml: Key %s was not found.\n", key);
		return 0;
	}
	return (char*) ezxml_attr(xml_parent, "value");
}

/* vmallocs memory for the string src and copies it there, dest is set to the new location */
static int vmalloc_str( char ** dest, const char * const src)
{
	int len;
	char * temp;
	len = strlen(src);
	if( len > MAX_XML_STRING_LENGTH - 1)
		len = MAX_XML_STRING_LENGTH - 1;

	temp = vmalloc( len + 1);
	if( 0 == temp)
	{
		pr_err("guf_xml: failed to allocate %d bytes for xml string %s\n", len, src);
		return -ENOMEM;
	}
	memcpy(temp, src,len);
	temp[len] = 0;
	*dest = temp;
	pr_debug("guf_xml: %d, Dest: 0x%08X, %s, Src: 0x%08X %s\n", len, (unsigned int) *dest, *dest, (unsigned int) src, src);

	return 0;
}

static int xml_get_attr(struct ezxml *xml_child, char * tag, const char ** res)
{
	const char * xml_string = 0;
	if(!tag)
	{
		pr_err("%s: no tag \n", __func__);
		return -EINVAL;
	}
	if(!xml_child)	
	{
		pr_err("%s: No xml child given for tag '%s'\n", __func__,tag);
		return -EINVAL;
	}
	xml_string = ezxml_attr( xml_child, tag);
	if(!xml_string) 
	{
		pr_warn("%s: xml attr '%s' not found\n", __func__,tag);
		return -EINVAL;
	}
	*res = xml_string;
	return 0;
}

static int xml_read_u32( struct ezxml *xml_child, char * tag, unsigned long * res)
{
	const char * xml_string;
	int r;
	unsigned long v;
	r = xml_get_attr( xml_child, tag, &xml_string );
	if(r) return r;
	r = kstrtoul( xml_string, 0, &v);
	if(r) 
	{
		pr_warn("%s: xml attr '%s' could not be decoded\n", __func__,tag);
		return r;
	}
	pr_debug("%s: '%s' %lu\n",__func__, tag, v);
	*res = v;
	return 0;
}

int xml_get_str( struct ezxml *xml_child, char * tag, char ** res)
{
	const char * xml_string = 0;
	int r;
	r = xml_get_attr( xml_child, tag, &xml_string );
	if(r)
	{
		pr_warn("%s: Couldn't find tag '%s'\n", __func__, tag);
		return r;
	}
	if(!xml_string)
	{
		pr_warn("%s: Couldn't find tag '%s'\n", __func__, tag);
		return -EINVAL;
	}
	r = vmalloc_str( res, xml_string);
	if(r)
	{
		pr_err("%s: Failed to copy string for tag '%s': %s, %d\n", __func__, tag, xml_string, r);
		*res = "";
	}

	if(!r)
		pr_debug("%s: '%s' %s\n",__func__, tag, *res);
	return r;
}

/* Searches for the given key in the xml tree, allocates memory and copies the value to the given address */
static int cpy_string_from_xml(struct ezxml * xml_cfg, const char * const key, char ** dest)
{
	char * temp;

	temp = get_string_from_xml(xml_cfg, key);
	if( !temp)	return -EINVAL;
	pr_debug("guf_xml: key %s: %s\n", key, temp );

	return vmalloc_str( dest, temp);
}
static inline char to_lower_case(char c)
{
	if( c >= 'A' && c <= 'Z')
		c -= 'A' - 'a';
	return c;
}
/* Compares the string at a given xml key with cmp. Caseinsensitive.*/
static int cmp_string_with_xml(struct ezxml *xml_cfg, const char * key, const char * cmp, bool * dest)
{
	char * temp;
	int i;
	char a,b;	// supress unitialized warning
	*dest = false;

	temp = get_string_from_xml( xml_cfg, key);
	if(!temp)
	{
		pr_warn("guf_xml: failed to read %s\n", key);
		return 1;
	}
	i = 0;
	do{
		a = to_lower_case(temp[i]); b = to_lower_case(cmp[i]);
		//pr_debug("%d: %c - %c\n", i, a, b);
		if(a != b) break;
	} while( temp[i] != 0 && cmp[i] != 0 && ( ++i < MAX_XML_STRING_LENGTH ));
	if(a ==  b) 
		*dest = true;

	pr_debug("guf_xml: %s: %s\n", key, *dest? "true" : "false");
	return 0;
}

/* Parse GuF XML network */
int get_network_configuration(struct ezxml *xml_cfg, struct guf_xml_data_network * network)
{
	char *attr, *opt = NULL, *temp;
	int i = 0;

	cpy_string_from_xml( xml_cfg, "bootp_my_ip", &network->ip_address);
	cmp_string_with_xml( xml_cfg, "bootp", "true", &network->dhcp);
	cpy_string_from_xml( xml_cfg, "bootp_my_ip_mask", &network->netmask);
	cpy_string_from_xml( xml_cfg, "bootp_my_gateway_ip", &network->gateway);
	cpy_string_from_xml( xml_cfg, "eth0_name", &network->hostname);
	cpy_string_from_xml( xml_cfg, "bootp_my_ip_mask", &network->netmask);

	/* read the mac address */
	temp = get_string_from_xml( xml_cfg, "eth0_esa_data");
	attr = temp;
	while (attr && (attr != '\0') && i < 14) { //sa_data[14]
		opt = strsep(&attr, ":");
		network->mac_addr.sa_data[i] = (unsigned char) simple_strtoul(opt, NULL, 16);;
		i++;
	}

	network->valid = true;
	return 0;
}

// Parse all setting nodes with device tree information
int guf_get_devtree_settings(struct ezxml *xml_cfg, struct guf_xml_devtree ** devtree)
{
	struct ezxml *xml_setting;
	struct guf_xml_devtree * tail = 0;

	* devtree = 0;

	xml_setting = ezxml_child(xml_cfg, "setting");
	if (!xml_setting)
		return -1;

	for (; xml_setting; xml_setting = ezxml_next(xml_setting)) {
		pr_info("%s %s\n", __func__, ezxml_attr(xml_setting, "key"));
		if (ezxml_attr(xml_setting, "devicetree")) {
			const char * xml_value = ezxml_attr(xml_setting, "value");
			if( xml_value)
			{
				char * value;
				struct guf_xml_devtree * newentry = vmalloc(sizeof(struct guf_xml_devtree));
				if(!newentry) return ENOMEM;
				
				vmalloc_str( &value, xml_value);
				if(!value){
					vfree(newentry);
					return ENOMEM;
				}

				newentry->entry = value;
				newentry->next = 0;
				if(! *devtree)
				{
					*devtree = newentry;
				}else{
					tail->next = newentry;
				}
				tail = newentry;
			}
		}
	}
	return 0;
}

int get_keypad_setting(struct ezxml *xml_cfg, char ** keypad)
{
	*keypad = 0;
	return cpy_string_from_xml( xml_cfg, "keypad", keypad);
}

int get_display_configuration(struct ezxml * xml_cfg, struct guf_xml_data * config, char *machine)
{
	char *attr, *padctl_attr_name;
	struct ezxml *xml_parent = NULL;
	struct ezxml *xml_child = NULL;
	unsigned long val;

	xml_parent = ezxml_child(xml_cfg, "display");
	if (!xml_parent) {
		pr_warn("guf_xml:%s: No display configuration found!\n", __func__);
		goto out;
	}

	/* parse <display>-tag */
	xml_get_str(xml_parent, "name", &config->display.name);
	if(!xml_read_u32(xml_parent, "xres", &val)) config->display.xres  = val;
	if(!xml_read_u32(xml_parent, "yres", &val)) config->display.yres  = val;
	attr = (char *) ezxml_attr(xml_parent, "type");
	if (!strcmp(attr, "SDC")) {
		config->display.type = DISPLAY_TYPE_SDC;
		if(!xml_read_u32(xml_parent, "vidmem", &val)) config->display.vidmem_size  = val;
		if(!xml_read_u32(xml_parent, "refresh", &val)) config->display.refresh  = val;
	} else if (!strcmp(attr, "ADC")) {
		config->display.type = DISPLAY_TYPE_ADC;
	} else {
		pr_warn("guf_xml:%s: Wrong display configuration: type=%s\n",
			__func__, attr);
		goto out;
	}
	xml_get_str(xml_parent, "originalDC", &config->display.original_dc);
	config->display.pix_clk = 0;
	attr = (char *) ezxml_attr(xml_parent, "pix_clk");
	if (attr != NULL)
		config->display.pix_clk = simple_strtoul(attr, NULL, 0);

	/* parse <lvds>-tag */
	xml_child = ezxml_child(xml_parent, "lvds");
	if (!xml_child) {
		config->lvds.enable = 0;
	} else {
		if(!xml_read_u32(xml_child, "enable", &val)) 
			config->lvds.enable = val;
		if(!xml_read_u32(xml_child, "channels", &val)) 
			config->lvds.channels = val;
		if(!xml_read_u32(xml_child, "sel6_8_polarity", &val)) 
			config->lvds.sel6_8_polarity = !(val==0);

		xml_get_str( xml_child, "mapping", &config->lvds.mapping);
		xml_get_str( xml_child, "mode", &config->lvds.mode);
	}

	/* parse <mode>-tag */
	xml_child = ezxml_child(xml_parent, "mode");
	if (!xml_child) {
		pr_warn("guf_xml:%s: Wrong display configuration: No <mode> tag!\n",
			__func__);
		goto out;
	}
	if(!xml_read_u32(xml_child, "custom_panel", &val)) config->mode.custom_panel  = val;
	if(!xml_read_u32(xml_child, "type", &val)) config->mode.panel_type  = val;
	attr = (char *) ezxml_attr(xml_child, "id");
	if(!attr)
	{
		pr_warn("guf_xml:%s: Wrong display configuration: no mode id found\n", __func__);
		goto out;
	}
	if (!strcmp(attr, "Device")) {
		config->mode.id = MODE_ID_DEVICE;
	} else if (!strcmp(attr, "NTSC")) {
		config->mode.id = MODE_ID_NTSC;
	} else if (!strcmp(attr, "PAL")) {
		config->mode.id = MODE_ID_PAL;
	} else if (!strcmp(attr, "None")) {
		config->mode.id = MODE_ID_NONE;
	} else {
		pr_warn("guf_xml:%s: Wrong display configuration: mode=%s\n",
			__func__, attr);
		goto out;
	}

	/* parse <format>-tag */
	xml_child = ezxml_child(xml_parent, "format");
	if (!xml_child) {
		pr_warn("guf_xml:%s: Wrong display configuration: No <format> tag!\n",
			__func__);
		goto out;
	}
	if(!xml_read_u32(xml_child, "depth", &val)) config->format.depth  = val;
	if(!xml_read_u32(xml_child, "video_depth", &val)) config->format.video_depth  = val;
	if(!xml_read_u32(xml_child, "EGPEFormat", &val)) config->format.egpeformat  = val;
	attr = (char *) ezxml_attr(xml_child, "format");
	if (!strcmp(attr, "RGB666")) {
		config->format.pixelformat = FORMAT_PIXEL_RGB666;
	} else if (!strcmp(attr, "BGR666")) {
		config->format.pixelformat = FORMAT_PIXEL_BGR666;
	} else if (!strcmp(attr, "RGB565")) {
		config->format.pixelformat = FORMAT_PIXEL_RGB565;
	} else if (!strcmp(attr, "RGB24")) {
		config->format.pixelformat = FORMAT_PIXEL_RGB24;
	} else if (!strcmp(attr, "BGR24")) {
		config->format.pixelformat = FORMAT_PIXEL_BGR24;
	} else if (!strcmp(attr, "YUV422")) {
		config->format.pixelformat = FORMAT_PIXEL_YUV422;
	} else if (!strcmp(attr, "LVDS666")) {
		config->format.pixelformat = FORMAT_PIXEL_LVDS666;
	} else {
		pr_warn("guf_xml:%s: Wrong display configuration: format=%s\n",
			__func__, attr);
		goto out;
	}
	/* parse <rotation>-tag */
	xml_child = ezxml_child(xml_parent, "rotation");
	if (!xml_child) {
		pr_warn("guf_xml:%s: Wrong display configuration: No <rotation> tag!\n",
			__func__);
		goto out;
	}
	attr = (char *) ezxml_attr(xml_child, "value");
	if (!strcmp(attr, "0")) {
		config->rotation.angle = ROTATION_0;
	} else if (!strcmp(attr, "90")) {
		config->rotation.angle = ROTATION_90;
	} else if (!strcmp(attr, "180")) {
		config->rotation.angle = ROTATION_180;
	} else if (!strcmp(attr, "270")) {
		config->rotation.angle = ROTATION_270;
	} else {
		pr_warn("guf_xml:%s: Wrong display configuration: rotation=%s\n",
			__func__, attr);
		goto out;
	}

	/* special SDC settings */
	if (config->display.type == DISPLAY_TYPE_SDC)
	{
		/* parse <backlight>-tag */
		xml_child = ezxml_child(xml_parent, "backlight");
		if (!xml_child) {
			pr_warn("guf_xml:%s: Wrong display configuration: No <backlight> tag!\n",
				__func__);
			goto out;
		}
		if(!xml_read_u32(xml_child, "level_ac", &val)) config->backlight.level_ac  = val;
		if(!xml_read_u32(xml_child, "level_battery", &val)) config->backlight.level_battery  = val;
		attr = (char *) ezxml_attr(xml_child, "frequency");
		if (attr) {
			config->backlight.frequency = simple_strtoul(attr, NULL, 0);
		}
		config->backlight.lut = NULL;
		attr = (char *) ezxml_attr(xml_child, "lut");
		if (attr) {
			config->backlight.lut =
				(uint32_t *) vmalloc(256 * sizeof (uint32_t));
			if (!config->backlight.lut) {
				pr_warn("guf_xml:%s: Wrong display configuration: Unable "
					"to allocate backlight LUT!\n", __func__);
			} else {
				int i;
				for (i = 0; (i < 256) && (*attr); i++) {
					config->backlight.lut[i] = simple_strtoul(attr, &attr, 0);
					while (!isdigit(*attr) && *attr)
						attr++;
				}
				if (i != 256)
					pr_warn
						("%s: Wrong display configuration: Backlight"
						 " LUT incomplete!\n", __func__);
			}
		}
		
		config->backlight.padctl_from_xml = false;
		if (machine != NULL)
		{
			attr = NULL;
			padctl_attr_name = vmalloc(8 + strlen(machine));
			if (padctl_attr_name == NULL) {
				printk("%s: Could not allocate memory\n", __func__);
			} else {
				strcpy(padctl_attr_name, "padctl_");
				strcat(padctl_attr_name, machine);
				if(!xml_read_u32(xml_child, padctl_attr_name, &val)) 
				{
					config->backlight.padctl = val;
					config->backlight.padctl_from_xml = true;
				}
				vfree(padctl_attr_name);
			}
		}

		/* parse <hsync>-tag */
		xml_child = ezxml_child(xml_parent, "hsync");
		if (!xml_child) {
			pr_warn("guf_xml:%s: Wrong display configuration: No <hsync> tag!\n",
				__func__);
			goto out;
		}
		if(!xml_read_u32(xml_child, "width", &val)) config->hsync.width  = val;
		if(!xml_read_u32(xml_child, "start_width", &val)) config->hsync.start_width  = val;
		if(!xml_read_u32(xml_child, "end_width", &val)) config->hsync.end_width  = val;
		if(!xml_read_u32(xml_child, "polarity", &val)) config->hsync.polarity  = val;

		/* parse <vsync>-tag */
		xml_child = ezxml_child(xml_parent, "vsync");
		if (!xml_child) {
			pr_warn("guf_xml:%s: Wrong display configuration: No <vsync> tag!\n",
				__func__);
			goto out;
		}
		if(!xml_read_u32(xml_child, "width", &val)) config->vsync.width  = val;
		if(!xml_read_u32(xml_child, "start_width", &val)) config->vsync.start_width  = val;
		if(!xml_read_u32(xml_child, "end_width", &val)) config->vsync.end_width  = val;
		if(!xml_read_u32(xml_child, "polarity", &val)) config->vsync.polarity  = val;

		/* parse <clock>-tag */
		xml_child = ezxml_child(xml_parent, "clock");
		if (!xml_child) {
			pr_warn("guf_xml:%s: Wrong display configuration: No <clock> tag!\n",
				__func__);
			goto out;
		}
		if(!xml_read_u32(xml_child, "idle_enable", &val)) config->clock.idle_enable  = val;
		if(!xml_read_u32(xml_child, "select_enable", &val)) config->clock.select_enable  = val;
		if(!xml_read_u32(xml_child, "polarity", &val)) config->clock.polarity  = val;

		/* parse <data>-tag */
		xml_child = ezxml_child(xml_parent, "data");
		if (!xml_child) {
			pr_warn("guf_xml:%s: Wrong display configuration: No <data> tag!\n",
				__func__);
			goto out;
		}
		if(!xml_read_u32(xml_child, "mask_enable", &val)) config->data.mask_enable  = val;
		if(!xml_read_u32(xml_child, "polarity", &val)) config->data.polarity  = val;
		if(!xml_read_u32(xml_child, "oe_polarity", &val)) config->data.oe_polarity  = val;

		/* parse <power_sequence>-tag */
		xml_child = ezxml_child(xml_parent, "power_sequence");
		if (!xml_child) {
			pr_warn
				("%s: Wrong display configuration: No <power_sequence> tag!\n",
				 __func__);
			goto out;
		}
		config->powerseq.poweron_to_signalon = simple_strtol(ezxml_attr(xml_child, "PowerOnToSignalOn"), NULL, 0);
		config->powerseq.poweron_to_backlighton = simple_strtol(ezxml_attr(xml_child, "PowerOnToBacklightOn"), NULL, 0);
		config->powerseq.backlightoff_before_poweroff =	simple_strtol(ezxml_attr(xml_child, "BacklightOffBeforePowerOff"), NULL, 0);
		config->powerseq.signaloff_before_poweroff = simple_strtol(ezxml_attr(xml_child, "SignalOffBeforePowerOff"), NULL, 0);
		config->powerseq.poweroff_to_poweron = simple_strtol(ezxml_attr(xml_child, "PowerOffToPowerOn"), NULL, 0);
	}

	pr_info("guf_xml:%s: Using display configuration 0x%08X %s\n",
		__func__, (unsigned int)config->display.name, config->display.name);
	config->display.valid = true;
	return 0;

out:
	return 1;
}

int get_touch_configuration(struct ezxml * xml_cfg, struct guf_xml_data * config)
{
	int i;
	char *attr;
	struct ezxml *xml_parent = NULL;

	config->touch = TOUCH_NONE;

	xml_parent = ezxml_child(xml_cfg, "touch");
	if (!xml_parent) {
		pr_warn("guf_xml:%s: No touch configuration found!\n", __func__);
		goto out;
	}

	/* parse <touch>-tag */
	attr = (char *) ezxml_attr(xml_parent, "name");
	for (i=0; i<ARRAY_SIZE(guf_xml_data_touch_names); i++)
	{
		if (!strcmp(attr, guf_xml_data_touch_names[i]))
			config->touch = i;
	}
	if (config->touch == TOUCH_NONE)
	{
		pr_warn("guf_xml:%s: Touch configuration \"%s\" found but not supported\n",
			__func__, attr);
		goto out;
	}

	pr_info("guf_xml:%s: Using touch configuration %s\n",
		__func__, guf_xml_data_touch_names[config->touch]);

out:
	return 0;
}

int get_logo_license(struct ezxml * xml_cfg, struct guf_xml_data * config)
{
	struct ezxml *xml_parent = NULL;
	char *attr;
	int i;

	xml_parent = ezxml_child(xml_cfg, "logo-license");
	if (!xml_parent) {
		pr_warn("guf_xml:%s: No logo license configuration found!\n", __func__);
		config->logo_license.valid = 0;
		goto out;
	}

	/* parse <logo license> tag */
	attr = (char *) ezxml_attr(xml_parent, "license");
	for (i = 0; i < 56 && (*attr); i++)
	{
		while (!isdigit(*attr) && *attr)
			attr++;
		config->logo_license.license[i] = simple_strtoul(attr, &attr, 0);
	}

	if (i != 56) {
		pr_err("guf_xml:%s: Wrong logo license: Logo license incomplete!\n", __func__);
		config->logo_license.valid = 0;
	} else {
		config->logo_license.valid = 1;
	}

out:
	return 0;
}
