/*
 * guf_xml_config.h - Garz+Fricke Configuration Parser
 *
 * Copyright (C) 2009 Garz+Fricke
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

#ifndef GUF_XML_CONFIG_H
#define GUF_XML_CONFIG_H

#include <linux/ezxml.h>
#include <linux/socket.h>
#include <linux/types.h>

#define NAME_FIS_DIRECTORY 			"FIS directory"
#define NAME_REDUNDANT_FIS 			"Redundant FIS"
#define LENGTH_NAME_FIS_DIRECTORY 	13
#define LENGTH_NAME_REDUNDANT_FIS 	13

// The maximal length of one string parameter. just a guess.
#define MAX_XML_STRING_LENGTH 1024
/*
 * Display types:
 *
 * TEST: only used to test XML display configuration data
 * SDC : synchronous display controller
 * ADC : asynchronous display controller (currently not supported)
 */
#define DISPLAY_TYPE_TEST 0
#define DISPLAY_TYPE_SDC 1
#define DISPLAY_TYPE_ADC 2

/* kind of display device (currently on DEVICE is supported) */
#define MODE_ID_DEVICE 0
#define MODE_ID_NTSC 1
#define MODE_ID_PAL 2
#define MODE_ID_NONE 3

/* pixel format for Display Interface (DI) (YUV422 is currently not supported) */
#define FORMAT_PIXEL_RGB666 0
#define FORMAT_PIXEL_RGB565 1
#define FORMAT_PIXEL_RGB24 2
#define FORMAT_PIXEL_YUV422 3
#define FORMAT_PIXEL_LVDS666 4
#define FORMAT_PIXEL_BGR24 5
#define FORMAT_PIXEL_BGR666 6

/* allowed values for the <rotation> tag */
#define ROTATION_0 0
#define ROTATION_90 90
#define ROTATION_180 180
#define ROTATION_270 270


struct guf_xml_device_platform_data {
	struct mtd_info *mtd_fis_directory;
	struct mtd_info *mtd_redundant_fis;
};

/* general configuration information */
typedef struct
{
	char *name;	/* name of display configuration */
	uint32_t vidmem_size;	/* size of the reserved video memory */
	uint16_t xres;		/* horizontal resolution */
	uint16_t yres;		/* vertical resolution */
	uint8_t refresh;	/* vertical refresh rate of the display */
	uint8_t type;		/* type of the display configuration (test, sdc, adc) */
	char* original_dc; /* the display controller the display configuration was originally made for */
	uint32_t pix_clk;	/* optional fixed pixel clock */
	bool valid;
} guf_xml_data_display_t;

/* display-mode descriptor */
typedef struct
{
	uint8_t	id;		/* kind of connected display (Device, NTSC, PAL) */
	uint8_t	custom_panel;	/* Custom or hard-coded settings */
	uint8_t	panel_type;	/* Hard-coded display configuration */
} guf_xml_data_mode_t;

/* LVDS descriptor */
typedef struct
{
	uint8_t	enable;		/* LVDS enabled */
	uint8_t	channels;	/* number of LVDS channels */
	uint8_t sel6_8_polarity; /* Polarity of the 6/8 selection line 0 = default(3 channels low, 4 channels high), 1 = inverted*/
	char* mapping;	/* pixel mapping in 4-channel mode */
	char* mode;	/* LVDS mode */
} guf_xml_data_lvds_t;

/* display-format descriptor */
typedef struct
{
	uint8_t	depth;		/* bit-depth of the graphics mode */
	uint8_t	video_depth;	/* pixel depth for video plane */
	uint8_t	pixelformat;	/* pixel-format of the display */
	uint8_t	egpeformat;	/* pixel format for Windows-CE (for compatibility) */
} guf_xml_data_format_t;

/* orientation descriptor */
typedef struct
{
	uint16_t angle;		/* angle of rotation (0, 90, 180, or 270) */
} guf_xml_data_rotation_t;

/* brightness level of the backlight */
typedef struct
{
	uint8_t level_ac;	/* level on AC-power */
	uint8_t level_battery;	/* level on battery power */
	uint32_t *lut;		/* level look-up table */
	uint32_t padctl;	/* pad configuration for pwm pin */
	bool padctl_from_xml;	/* pad configuration for pwm pin */
	uint32_t frequency;	/* pwm frequency */
} guf_xml_data_backlight_t;

/* descriptor for HSYNC, VSYNC signals */
typedef struct
{
	uint16_t width;		/* width of synchronization signal */
	uint16_t start_width;	/* synchronization start */
	uint16_t end_width;	/* synchronization end */
	uint8_t polarity;	/* polarity of the signal */
} guf_xml_data_sync_t;

/* descriptor for the display clock signal */
typedef struct
{
	uint8_t	idle_enable;	/* enable clock during VSYNC */
	uint8_t	select_enable;	/* disable clock when no data output */
	uint8_t	polarity;	/* polarity of clock signal */
} guf_xml_data_clock_t;

/* descriptor for misc.-stuff */
typedef struct
{
	uint8_t	mask_enable;	/* mask data output */
	uint8_t	polarity;	/* polarity of data signals */
	uint8_t	oe_polarity;	/* polarity of output-enable signals */
} guf_xml_data_data_t;

/* power-sequence desriptor (all times in milliseconds) */
typedef struct
{
	int16_t	poweron_to_signalon;		/* time from LCD-on to signal on */
	int16_t	poweron_to_backlighton;		/* time from LCD-on to backlight on */
	int16_t	backlightoff_before_poweroff;	/* req. time to turn-off backlight before LCD-off */
	int16_t	signaloff_before_poweroff;	/* req. time to turn-off signal before LCD-off */
	int16_t	poweroff_to_poweron;		/* req. time display must remain off before powering it on again */
} guf_xml_data_powerseq_t;


#define TOUCH_TYPES \
		TOUCH_TYPE( TOUCH_UCB1400,		"UCB1400",				"" )\
		TOUCH_TYPE( TOUCH_IMX_ADC,		"iMX ADC",				"" )\
		TOUCH_TYPE( TOUCH_DA9052,		"DA9052",				"" )\
		TOUCH_TYPE( TOUCH_PINNACLE,		"Cirque Pinnacle",		"" )\
		TOUCH_TYPE( TOUCH_EDT_FT5X06,	"EDT FT5x06",			"edt,ft5x06" )\
		TOUCH_TYPE( TOUCH_SCN_0500133,	"SCN0500133",			"" )\
		TOUCH_TYPE( TOUCH_K2,			"Cirque K2",			"" )\
		TOUCH_TYPE( TOUCH_EETI_I2C,		"EETI I2C",				"eeti,eetii2c_ts" )\
		TOUCH_TYPE( TOUCH_EETI_EXC3000_I2C,		"EETI EXC3000 I2C",	"eeti,exc3000_ts" )\
		TOUCH_TYPE( TOUCH_PIXCIR,		"PIXCIR",				"auo,auo_pixcir_ts" )\
		TOUCH_TYPE( TOUCH_STMPE,		"STMPE",				"st,stmpe610" )\
		TOUCH_TYPE( TOUCH_WM9705,		"WM9705",				"" )\


/* touch type */
typedef enum
{
	TOUCH_NONE = -1,
#define TOUCH_TYPE( id, xml_name, dt_node ) id,
	TOUCH_TYPES
#undef TOUCH_TYPE
	TOUCH_COUNT,
} guf_xml_data_touch_t;

/* logo license type */
#define LICENSE_LENGTH 56
typedef struct
{
	uint8_t license[LICENSE_LENGTH];		/* license */
	uint32_t valid;
} guf_xml_data_logo_license_t;

struct guf_xml_data_network{
	struct sockaddr mac_addr;
	bool dhcp;
	char *ip_address;
	char *netmask;
	char *gateway;
	char *hostname;
	bool valid;
};
struct guf_xml_devtree {
	char * entry; // string containing the loaded devtree snippets
	struct guf_xml_devtree * next;
};

/* root descriptor of the display configuration */
struct guf_xml_data
{
	guf_xml_data_display_t display;
	guf_xml_data_lvds_t lvds;
	guf_xml_data_mode_t mode;
	guf_xml_data_format_t format;
	guf_xml_data_rotation_t rotation;
	guf_xml_data_backlight_t backlight;
	guf_xml_data_sync_t hsync;
	guf_xml_data_sync_t vsync;
	guf_xml_data_clock_t clock;
	guf_xml_data_data_t data;
	guf_xml_data_powerseq_t powerseq;
	guf_xml_data_touch_t touch;
	struct guf_xml_data_network network;
	guf_xml_data_logo_license_t logo_license;
	int display_valid;

	char * keypad;
	struct guf_xml_devtree * devtree;
} ;

extern struct ezxml* xml_find_setting(struct ezxml *xml_variable, const char* setting);
extern struct ezxml* guf_get_config(char* string);
extern void guf_free_config(struct ezxml *xml_cfg);
extern int get_network_configuration(struct ezxml *xml_cfg, struct guf_xml_data_network *network_config);
extern int get_display_configuration(struct ezxml * xml_cfg, struct guf_xml_data * config, char *machine);
extern int get_touch_configuration(struct ezxml *xml_cfg, struct guf_xml_data *config);
extern int get_logo_license(struct ezxml *xml_cfg, struct guf_xml_data *config);
extern int get_keypad_setting(struct ezxml *xml_cfg, char ** keypad);
extern int guf_get_devtree_settings(struct ezxml *xml_cfg, struct guf_xml_devtree ** devtree);

#endif
