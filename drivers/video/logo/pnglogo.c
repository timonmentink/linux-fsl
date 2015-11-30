/*
 * Copyright (C) 2013 Garz & Fricke GmbH, Inc. All Rights Reserved.
 *
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
 *
 * Author: Carsten Behling <carsten.behling@garz-fricke.com>
 *
 * Derived from Garz & Fricke RedBoot extension gflogo.c from
 * Marc-Oliver Westerburg <mow@garz-fricke.com>
 *
 */

#include <linux/linux_logo.h>
#include <linux/png.h>
#include <linux/fb.h>
#include <linux/ezxml.h>
#include <linux/vmalloc.h>
#include <linux/crc32.h>

#define NO_PROFILE_PNG_LOGO

#define LICENSE_DATA_LENGTH 52
#define LICENSE_CRC_LENGTH 4
#define LICENSE_LENGTH (LICENSE_CRC_LENGTH + LICENSE_DATA_LENGTH)

enum{
	LOGO_DEFAULT,
	LOGO_BLACK,
	LOGO_TEST,
	LOGO_PNG,
};
enum{
	LICENSE_NONE,
	LICENSE_GENERAL,
	LICENSE_EMBEDDED,
};

static unsigned int fb_show_logo_type = LOGO_DEFAULT;
static unsigned long fb_png_logo_start = 0;
static int license_type = LICENSE_NONE;

static int __init fb_early_png_logo(char *p)
{
	if(!memcmp(p, "black", 5)) {
		fb_show_logo_type = LOGO_BLACK;
	} else if(!memcmp(p, "test", 4)) {
		fb_show_logo_type = LOGO_TEST;
	} else {
		fb_png_logo_start = memparse(p, &p);
		if(fb_png_logo_start)
			fb_show_logo_type = LOGO_PNG;
	}
	return 0;
}
early_param("logo", fb_early_png_logo);

typedef struct {
	unsigned int	shown;		/* true when image has actually been displayed */
	unsigned int	crc;		/* checksum for validation */
	int				x_offs;		/* x offset where to start showing the image on screen */
	int				y_offs;		/* y offset where to start showing the image on screen */
	png_uint_32		width;		/* width of PNG image */
	int				rotate;		/* orientation */
	png_uint_32		height;		/* height of PNG image */
	int				interlace;	/* interlace-mode used by PNG image */
	int				channels;	/* number of channels; RGB:3, RGBA:4 */
	struct fb_info	*fb_info;	/* pointer to framebuffer driver */
	png_color_16p	bkg_color;	/* Background color of the image */
	int				all_read;	/* set to 1 in end call back */
} row_info_t;

/* offsets and increments to handle PNG's ADAM7 de-interlacing */
static const int col_start[7] = {0,4,0,2,0,1,0};
static const int col_incr[7] = {8,8,4,4,2,2,1};
static const int row_start[7] = {0,0,4,0,2,0,1};
static const int row_incr[7] = {8,8,8,4,4,2,2};
#ifdef DEBUG
static void dump_fbinfo(const struct fb_info * fb)
{
	if(!fb)
	{
		pr_info("%s fb = NULL\n", __func__);
		return;
	}
	pr_info("png_logo: screen format: xres %d, yres %d, bits_per_pixel %d, grayscale %d\n",
		fb->var.xres,
		fb->var.yres,
		fb->var.bits_per_pixel,
		fb->var.grayscale
		);
}

static void dump_row_info(const row_info_t * r)
{
	if(!r)
	{
		pr_info("%s r = NULL\n", __func__);
		return;
	}
	pr_info("png_logo: image format: x_offs %d, y_offs %d, width %u, heigth %u, rotate %d, ch %d\n",
		r->x_offs,
		r->y_offs,
		(unsigned int)r->width,
		(unsigned int)r->height,
		r->rotate,
		r->channels
		);
}
static void dump_logo(const struct linux_logo * logo)
{
	if(!logo)
	{
		pr_info("%s logo = NULL\n", __func__);
		return;
	}
	pr_info("png_logo: type %d, width %u, height %u, clutsize %u, clut 0x%0X, data 0x%0X\n",
		logo->type,
		logo->width,
		logo->height,
		logo->clutsize,
		(unsigned int)logo->clut,
		(unsigned int)logo->data);
}
/* Macro to dump useful information for debug*/

#define DUMP(i) \
	do{pr_info("png_logo: %d info_ptr->width %d, row_info_ptr->width %d\n",i, (int)info_ptr->width, (int)row_info_ptr->width);\
	dump_row_info(row_info_ptr);\
	dump_fbinfo(row_info_ptr->fb_info);\
	}while(0)

#else
#define DUMP(i)
#endif

static inline int
fb_read_pixel(struct fb_info *info, unsigned int x, unsigned int y, int rotate,
				unsigned char *red, unsigned char *green, unsigned char *blue)
{
	unsigned int color = 0;
	unsigned int temp = 0;
	int ret = 0;

	switch(rotate) {
		case FB_ROTATE_UR:
			x = info->var.xres - x;
			y = info->var.yres - y;
			break;
		case FB_ROTATE_CW:
			temp = info->var.yres - x;
			x = y;
			y = temp;
			break;
		case FB_ROTATE_CCW:
			temp = x;
			x = info->var.xres - y;
			y = temp;
			break;
		case FB_ROTATE_UD:
		default:
			break;
	}

	/* check screen boundaries */
	if(x >= info->var.xres || y >=  info->var.yres) {
		*red = *green = *blue = 0;
		return ret;
	}

	switch (info->var.bits_per_pixel) {
		case 8:
			color = (unsigned int)*((unsigned char *)info->screen_base +
				info->fix.line_length * y + x);
			break;
		case 16:
			color = (unsigned int)*((unsigned short *)info->screen_base +
				info->fix.line_length/2 * y + x);
			break;
		case 32:
			color = *((unsigned int *)info->screen_base +
				info->fix.line_length/4 * y + x);
			break;
		case 24:
			{
				unsigned char * pix = (unsigned char *)info->screen_base + info->fix.line_length * y + x * 3;
				color = pix[0] << 16 | pix[1] << 8 | pix[2] << 0;
			}
			break;
		default:
			pr_err("pnglogo - %s: %s: unsupported bpp: %d\n", __func__, info->fix.id, info->var.bits_per_pixel);
			ret = -ENODEV;
			break;
	}

	*red = (unsigned char)((color << (32 - info->var.red.offset - info->var.red.length)) >> 24);
	*green = (unsigned char)((color << (32 - info->var.green.offset - info->var.green.length)) >> 24);
	*blue = (unsigned char)((color << (32 - info->var.blue.offset - info->var.blue.length)) >> 24);

	return ret;
}

static inline int
fb_write_pixel(struct fb_info *info, unsigned int x, unsigned int y, int rotate,
				unsigned char red, unsigned char green, unsigned char blue)
{
	unsigned int color = 0;
	unsigned int temp = 0;

	switch(rotate) {
		case FB_ROTATE_UR:
			x = info->var.xres - x;
			y = info->var.yres - y;
			break;
		case FB_ROTATE_CW:
			temp = info->var.yres - x;
			x = y;
			y = temp;
			break;
		case FB_ROTATE_CCW:
			temp = x;
			x = info->var.xres - y;
			y = temp;
			break;
		case FB_ROTATE_UD:
		default:
			break;
	}

	/* check screen boundaries */
	if(x >= info->var.xres || y >=  info->var.yres) {
		return 0;
	}

	color = (((red >> (8 - info->var.red.length)) <<
			info->var.red.offset) +
				((green >> (8 - info->var.green.length)) <<
			info->var.green.offset) +
				((blue >> (8 - info->var.blue.length)) <<
			info->var.blue.offset));

	switch (info->var.bits_per_pixel) {
		case 8:
			*((unsigned char *)info->screen_base +
				info->fix.line_length * y + x) =
				(unsigned char)color;
			break;
		case 16:
			*((unsigned short *)info->screen_base +
				info->fix.line_length/2 * y + x) =
				(unsigned short)color;
			break;
		case 32:
			*((unsigned int *)info->screen_base +
				info->fix.line_length/4 * y + x) =
				(unsigned int)color;
			break;
		case 24:
			{
				unsigned char * pix = (unsigned char *) info->screen_base + info->fix.line_length * y + x * 3;
				pix[0] = color >> 16;
				pix[1] = color >> 8;
				pix[2] = color >> 0;
			}
			break;
		default:
			pr_err("pnglogo - %s: %s: unsupported bpp: %d\n", __func__, info->fix.id, info->var.bits_per_pixel);
			return -ENODEV;
	}

	return 0;
}

static inline int
fb_fill_block(struct fb_info *info, unsigned int x_off, unsigned int y_off,
				unsigned int width, unsigned int height, int rotate,
				unsigned char red, unsigned char green, unsigned char blue)
{

	unsigned int color = 0;
	unsigned int x = 0;
	unsigned int y = 0;
	unsigned int temp = 0;

	switch(rotate) {
		case FB_ROTATE_UR:
			x_off = info->var.xres - x_off;
			y_off = info->var.yres - y_off;
			x = info->var.xres - x;
			y = info->var.yres - y;
			break;
		case FB_ROTATE_CW:
			temp = info->var.yres - x_off;
			x_off = y_off;
			y_off = temp;
			temp = info->var.yres - x;
			x = y;
			y = temp;
			break;
		case FB_ROTATE_CCW:
			temp = x_off;
			x_off = info->var.xres - y_off;
			y_off = temp;
			temp = x;
			x = info->var.xres - y;
			y = temp;
			break;
		case FB_ROTATE_UD:
		default:
			break;
	}

	/* check screen boundaries */
	if(width + x_off > info->var.xres || height + y_off >  info->var.yres) {
		return 0;
	}

	for(x = x_off; x < width + x_off; x++) {
		for(y = y_off; y < height + y_off; y++) {
			color = (((red >> (8 - info->var.red.length)) <<
				info->var.red.offset) +
				   ((green >> (8 - info->var.green.length)) <<
				info->var.green.offset) +
				   ((blue >> (8 - info->var.blue.length)) <<
				info->var.blue.offset));
			switch (info->var.bits_per_pixel) {
				case 8:
					*((unsigned char *)info->screen_base +
						info->fix.line_length * y + x) =
						(unsigned char)color;
					break;
				case 16:
					*((unsigned short *)info->screen_base +
						info->fix.line_length/2 * y + x) =
						(unsigned short)color;
					break;
				case 32:
					*((unsigned int *)info->screen_base +
						info->fix.line_length/4 * y + x) =
						(unsigned int)color;
					break;
				case 24:
					{
						unsigned char * pix = (unsigned char *)info->screen_base + info->fix.line_length * y + x * 3;
						pix[0] = color >> 16;
						pix[1] = color >> 8;
						pix[2] = color >> 0;
					}
					break;
				default:
					pr_err("pnglogo - %s: %s: unsupported bpp: %d\n", __func__, info->fix.id, info->var.bits_per_pixel);
					return -ENODEV;
			}
		}
	}

	return 0;
}


static void fb_info_callback(png_structp png_ptr, png_infop info_ptr)
{
	int bit_depth;
	int color_type;
	png_color_16p bkg_color;
	row_info_t	*row_info_ptr = (row_info_t *)png_get_progressive_ptr(png_ptr);

	DUMP(0);
	/* read PNG image information */
	if(!png_get_IHDR(png_ptr, info_ptr,
				 &row_info_ptr->width,
				 &row_info_ptr->height,
				 &bit_depth, &color_type,
				 &row_info_ptr->interlace,
				 (int *)NULL,
				 (int *)NULL))
	{
		pr_warning("png_logo: Error reading IHDR imge info\n");
	}

	/* set up libpng transformations so that our fb_row_callback will
	   always receive data in RGB or RGBA format */

	/* usually we'll get 4 channels, while libpng can add a dummy alpha
	   channel if none is present, its ~10% faster if we handle this
	   ourselves */
	row_info_ptr->channels = 4;

	/* convert palette-images to RGB */
	if (color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_palette_to_rgb(png_ptr);

	/* ensure that gray-scale images have a bit_depth of at least 8 bpp */
	if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
		png_set_expand_gray_1_2_4_to_8(png_ptr);

	/* if image uses simple transparency, convert it to real RGBA pixels */
	if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
		png_set_tRNS_to_alpha(png_ptr);
	else {
		/* if image is palettized w.o. any transparency, we'll only get
		   RGB-pixels */
		if (color_type == PNG_COLOR_TYPE_PALETTE)
			row_info_ptr->channels = 3;
	}

	/* ensure we don't get more than 8 bpp per channel */
	if (bit_depth == 16)
		png_set_strip_16(png_ptr);

	/* for images with <8BPP unpack pixels to separate bytes */
	if (bit_depth < 8)
		png_set_packing(png_ptr);

	/* convert gray-scale images to RGB */
	if (color_type == PNG_COLOR_TYPE_GRAY ||
		color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
		png_set_gray_to_rgb(png_ptr);

	/* if alpha-channel is missing, we'll only get RGB-pixels */
	if (color_type == PNG_COLOR_TYPE_RGB ||
		color_type == PNG_COLOR_TYPE_GRAY)
		row_info_ptr->channels = 3;

	if(row_info_ptr->rotate == FB_ROTATE_UR ||
		row_info_ptr->rotate == FB_ROTATE_UD) {

		if(row_info_ptr->width > row_info_ptr->fb_info->var.xres)
			row_info_ptr->width = row_info_ptr->fb_info->var.xres;
		if(row_info_ptr->height > row_info_ptr->fb_info->var.yres)
			row_info_ptr->height = row_info_ptr->fb_info->var.yres;

		/* center image on screen */
		if (row_info_ptr->x_offs < 0)
			row_info_ptr->x_offs = (row_info_ptr->fb_info->var.xres - row_info_ptr->width) / 2;

		if (row_info_ptr->y_offs < 0)
			row_info_ptr->y_offs = (row_info_ptr->fb_info->var.yres - row_info_ptr->height) / 2;

		/* clip to borders */
		if ((row_info_ptr->x_offs + row_info_ptr->width) > row_info_ptr->fb_info->var.xres)
			row_info_ptr->width = row_info_ptr->fb_info->var.xres - row_info_ptr->x_offs;

		if ((row_info_ptr->y_offs + row_info_ptr->height) > row_info_ptr->fb_info->var.yres)
			row_info_ptr->height = row_info_ptr->fb_info->var.yres - row_info_ptr->y_offs;

	} else {
		if(row_info_ptr->width > row_info_ptr->fb_info->var.yres)
			row_info_ptr->width = row_info_ptr->fb_info->var.yres;
		if(row_info_ptr->height > row_info_ptr->fb_info->var.xres)
			row_info_ptr->height = row_info_ptr->fb_info->var.xres;

		/* center image on screen */
		if (row_info_ptr->x_offs < 0)
			row_info_ptr->x_offs = (row_info_ptr->fb_info->var.yres - row_info_ptr->width) / 2;

		if (row_info_ptr->y_offs < 0)
			row_info_ptr->y_offs = (row_info_ptr->fb_info->var.xres - row_info_ptr->height) / 2;

		/* clip to borders */
		if ((row_info_ptr->x_offs + row_info_ptr->width) > row_info_ptr->fb_info->var.xres)
			row_info_ptr->width = row_info_ptr->fb_info->var.yres - row_info_ptr->x_offs;
		if ((row_info_ptr->y_offs + row_info_ptr->height) > row_info_ptr->fb_info->var.yres)
			row_info_ptr->height = row_info_ptr->fb_info->var.yres - row_info_ptr->y_offs;
	}

	/* read background data from image (if present) and fill framebuffer */
	if (png_get_bKGD(png_ptr, info_ptr, &bkg_color)) {
		/* libpng already initializes RGB of the background color
		   even for palletized and gray-scale images.
		    - for palettized images BKG_RGB will already be 8 bit
		    - for RGB images BKG_RGB may be 8 or 16 bit
		    - for gray-scale images BKG_RGB may be 1, 2, 4, 8, or 16 bit
		   we'll have to convert to 8 bit manually:
		   Note: This conversion is equivalent to what png_do_expand() does */
		if (!(color_type & PNG_COLOR_MASK_PALETTE)) {
			switch (bit_depth) {
				case 1:
					bkg_color->red *= (png_uint_16)0xff;
					bkg_color->green *= (png_uint_16)0xff;
					bkg_color->blue *= (png_uint_16)0xff;
					break;
				case 2:
					bkg_color->red *= (png_uint_16)0x55;
					bkg_color->green *= (png_uint_16)0x55;
					bkg_color->blue *= (png_uint_16)0x55;
					break;
				case 4:
					bkg_color->red *= (png_uint_16)0x11;
					bkg_color->green *= (png_uint_16)0x11;
					bkg_color->blue *= (png_uint_16)0x11;
					break;
				case 16:
					bkg_color->red >>= 8;
					bkg_color->green >>= 8;
					bkg_color->blue >>= 8;
					break;
				case 8:
				default:
					break;
			}
		}
		row_info_ptr->bkg_color = bkg_color;
		if(license_type != LICENSE_NONE)
		{
			/* PNG format does not support transparent background colors
			   so a simple fill() will do: */
			if(fb_fill_block(row_info_ptr->fb_info, 0, 0, row_info_ptr->fb_info->var.xres,
				row_info_ptr->fb_info->var.yres, row_info_ptr->rotate,
				bkg_color->red, bkg_color->green, bkg_color->blue))
			{
				png_error(png_ptr, "Failed to write framebuffer");
			}
		}
	}

	/* tell libpng that we're done with setting up transformations */
	png_read_update_info(png_ptr, info_ptr);

#ifdef DEBUG
	dump_row_info(row_info_ptr);
#endif
	return;
}

static void fb_row_callback(png_structp png_ptr, png_bytep new_row,
				png_uint_32 y, int pass)
{
	/* Note: This routine has to deal with three different coordinate spaces:
	    - "sub-image space": coordinate space of the input data we receive
	      for non-interlaced images this is the same as (whole) image space,
	      for ADAM7 progressive images this is always a fractional image-space
	      of the whole image.
	    - "(whole-)image space": coordinate space of the whole PNG-file, i.e.
	    - "screen space": coordinate space of our output screen */
	int 		len;
	row_info_t	*row_info_ptr = (row_info_t *)png_get_progressive_ptr(png_ptr);
	struct fb_info *fb_info = row_info_ptr->fb_info;
	unsigned int x = 0;

	row_info_ptr->shown = 1;

	if( y < 10)
	pr_debug("pnglogo: +fb_row_callback(0x%08x, %2u, %u) (width: %u) ", (unsigned int)new_row,
		(unsigned int)y, (unsigned int)pass, (unsigned int)row_info_ptr->width);

	if (row_info_ptr->interlace == PNG_INTERLACE_NONE) {
		/* if image is not interlaced, we can handle it like the last
		   pass of interlaced image: the row we got contains a full set
		   of pixel data for the whole image width, i.e. in x-direction
		  for pass #6 sub-image space == (whole-)image space  */
		pass = 6;
	} else {
		/* for interlaced images we'll have to convert our input y-parameter,
		   which is in sub-image space to whole-image space */
		y = (row_incr[pass]*y) + row_start[pass];
	}

	if (y > row_info_ptr->height)
		return;

	/* draw pixels of the row */
	for(len = row_info_ptr->width - col_start[pass];
		len > 0;
		len -= col_incr[pass]) {
		/* does the pixel have a non-opaque alpha channel? */
		if ((row_info_ptr->channels == 4) && (new_row[3] != 0xff)) {
			unsigned char fb_r = 0;
			unsigned char fb_g = 0;
			unsigned char fb_b = 0;

			/* yep, read current destination pixel from framebuffer */
			if(fb_read_pixel(fb_info, x + row_info_ptr->x_offs,
				y + row_info_ptr->y_offs, row_info_ptr->rotate, &fb_r, &fb_g, &fb_b))
			{
				png_error(png_ptr, "Failed to read pixel from framebuffer");
			}

			/* composite new pixel with current pixel:
			   fb = img * alpha + fb * (1-alpha) */
			png_composite(new_row[0], new_row[0], new_row[3], fb_r);
			png_composite(new_row[1], new_row[1], new_row[3], fb_g);
			png_composite(new_row[2], new_row[2], new_row[3], fb_b);
		}
		/* write pixel to screen */
		if( fb_write_pixel(fb_info, x + row_info_ptr->x_offs,
			y + row_info_ptr->y_offs, row_info_ptr->rotate, new_row[0],
			new_row[1], new_row[2]))
		{
			png_error(png_ptr, "Failed to read pixel from framebuffer");
		}

		/* progress input and output data pointers */
		new_row += row_info_ptr->channels;
		x++;
	}

	if( y < 10)
	pr_debug("pnglogo: -fb_row_callback()\n");

	return;
}

static void fb_end_callback(png_structp png_ptr, png_infop info_ptr)
{
	row_info_t	*row_info_ptr = (row_info_t *)png_get_progressive_ptr(png_ptr);
	if (!row_info_ptr->shown) {
		pr_err("pnglogo: no boot logo license found\n");
	}

	row_info_ptr->all_read = 1;
	return;
}

static int fb_read_chunk_callback(png_structp png_ptr, png_unknown_chunkp chunk)
{
	row_info_t	*row_info_ptr = (row_info_t *)png_get_progressive_ptr(png_ptr);

	/* check for a PNG file related license */
	if (!memcmp(chunk->name, "igUf", 4) &&
		(chunk->size == sizeof(unsigned int)) &&
		!memcmp(chunk->data, &row_info_ptr->crc, sizeof(unsigned int))) {
		pr_info("png_logo: logo license in PNG file provided\n");

		png_set_progressive_read_fn(png_ptr, (png_voidp)row_info_ptr,
			fb_info_callback, fb_row_callback, fb_end_callback);

		license_type = LICENSE_EMBEDDED;

		return 1;
	}
	return 0;
}

#ifdef DEBUG
inline void dump_pixel(struct fb_info *info, int number)
{
	unsigned char r,g,b;
	static struct fb_info *my_info = NULL;
	if (info == NULL)
		info = my_info;
	else
		my_info = info;
	if (info)
	{
		fb_read_pixel(info, 0, 0, FB_ROTATE_UD, &r, &g, &b);
		printk("=============== pixel (%d, 0x%p) at 0/0: %u, %u, %u\n", number, info, r, g, b);
	} else {
		printk("=============== pixel (%d, NULL)\n", number);
	}
}
#else
#define dump_pixel(a,b ) do{}while(0)
#endif
static int fb_show_testlogo(struct fb_info *info, int rotate)
{
	int				xstart;
	int				ystart;
	int 			col_width;
	int 			col_height;
	int				num_cols;
	int 			c;
	int				i;
	int 			width;

	/* clear whole screen to black/white checkerboard */
	for (ystart = 0; ystart < info->var.yres; ystart++) {
		for (xstart = 0; xstart < info->var.xres; xstart++) {
			if((xstart ^ ystart) & 0x1){
				fb_write_pixel(info, xstart, ystart, rotate, 0x0, 0x0, 0x0);
			}else{
				fb_write_pixel(info, xstart, ystart, rotate, 0xff, 0xff, 0xff);
			}
		}
	}

	/* calculate number and width of color bars we can display */
	if (info->var.xres > 256) {
		xstart = (info->var.xres & 0xff) >> 1;
		col_width = ((info->var.xres - 2 * xstart) >> 8) & 0x3;
		num_cols = 256;
	} else {
		xstart = (info->var.xres & 0x7f) >> 1;
		col_width = 1;
		num_cols = 128;
	}

	/* calculate height of color bars we display */
	col_height = (info->var.xres - 20) >> 4;

	/* write 2x2 white test-pixel on the corners of the
	   color-ramp so that the test-pixel on the 'inside'
	   corner of the color ramp will get overwritten */
	fb_write_pixel(info, xstart - 1, 10 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart - 1, 10, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart, 10 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart, 10, rotate, 0xff, 0, 0xff);

	fb_write_pixel(info, xstart + col_width * 256 - 1, 10 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart + col_width * 256 - 1, 10, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart + col_width * 256 - 1, 10 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart + col_width * 256, 10, rotate, 0xff, 0, 0xff);

	fb_write_pixel(info, xstart + col_width * 256 - 1, 10 + col_height * 8 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart + col_width * 256 - 1, 10 + col_height * 8, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart + col_width * 256, 10 + col_height * 8 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart + col_width * 256, 10 + col_height * 8, rotate, 0xff, 0, 0xff);

	fb_write_pixel(info, xstart - 1, 10 + col_height * 8 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart - 1, 10 + col_height * 8, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart, 10 + col_height * 8 - 1, rotate, 0xff, 0, 0xff);
	fb_write_pixel(info, xstart, 10 + col_height * 8, rotate, 0xff, 0, 0xff);

	/* output color ramps; from top to bottom:
	   - black-to-white,
	   - black-to-red,
	   - black-to-green,
	   - black-to-blue
	   we leave a border of at least 10 pixels so we don't collide with our
	   corner pixels below, but we won't center the color ramps, n displays
	   wider than 256 (+20) pixels we display 256 shades, each one possibly
	   multiple pixels wide, on narrower displays we only show 128 shades
	   (each 1 pixel wide) */

	for (i = 0; i < num_cols; i++) {
		c = (num_cols == 256) ? i : (i << 1);
		fb_fill_block(info, xstart + i * col_width, 10 + 0 * col_height, col_width, col_height,
			rotate, c, c, c);
		fb_fill_block(info, xstart + i * col_width, 10 + 1 * col_height, col_width, col_height,
			rotate, c, 0, 0);
		fb_fill_block(info, xstart + i * col_width, 10 + 2 * col_height, col_width, col_height,
			rotate, 0, c, 0);
		fb_fill_block(info, xstart + i * col_width, 10 + 3 * col_height, col_width, col_height,
			rotate, 0, 0, c);
	}

	for (i = 0; i < 8; i++) {
		c = 1 << i;
		width = (num_cols >> 3)*col_width;
		fb_fill_block(info, xstart + i * width, 10 + 4 * col_height, width, col_height,
			rotate, c, c, c);
		fb_fill_block(info, xstart + i * width, 10 + 5 * col_height, width, col_height,
			rotate, c, 0, 0);
		fb_fill_block(info, xstart + i * width, 10 + 6 * col_height, width, col_height,
			rotate, 0, c, 0);
		fb_fill_block(info, xstart + i * width, 10 + 7 * col_height, width, col_height,
			rotate, 0, 0, c);
		fb_write_pixel(info, xstart +i * width, 10 +  8 * col_height, rotate, 0xff, 0, 0xff);
	}

	/* write test-pixel into all corners of the screen:
	   TOP-LEFT: red
	   TOP-RIGHT: green
BOTTOM_RIGHT: blue
BOTTOM_WHITE: white */
	for (i = 0; i < 3; i++) {
		fb_write_pixel(info, 0 + i, 0 + i, rotate, 0xff, 0, 0);
		fb_write_pixel(info, info->var.xres - 1 - i, 0 + i, rotate, 0, 0xff, 0);
		fb_write_pixel(info, info->var.xres - 1 - i, info->var.yres - 1 - i, rotate, 0, 0, 0xff);
		fb_write_pixel(info, 0 + i, info->var.yres - 1 - i, rotate, 0xff, 0xff, 0);
	}

	return 0;
}


static int check_generic_logo_license( const uint8_t * logo_license, int *license_type)
{
	z_stream 		zstream;
	unsigned int	crc;
	int ret = 1;
	#define	Z_STREAM_OUTBUFFER_SIZE 64
	char z_stream_buffer[Z_STREAM_OUTBUFFER_SIZE];

	if(!logo_license ){
		pr_info("png_logo: no generic logo license provided\n");
		return 0;
	}

	/* check for a generic logo license */
	zstream.avail_in = LICENSE_DATA_LENGTH;
	zstream.next_in = logo_license;
	zstream.avail_out = Z_STREAM_OUTBUFFER_SIZE;
	zstream.next_out = z_stream_buffer;
	zstream.workspace = vmalloc(zlib_inflate_workspacesize());
	if (zstream.workspace == NULL) {
		pr_err("pnglogo: internal zlib error\n");
		goto nothing_to_free;
	}
	if (zlib_inflateInit2(&zstream, 15+32) != Z_OK) {
		pr_err("pnglogo: zlib initialization error\n");
		goto free_zstream_workspace;
	}

	if (zlib_inflate(&zstream, Z_FINISH) != Z_STREAM_END) {
		pr_err("pnglogo: zlib inflate error\n");
		goto free_zstream_workspace;
	}
	pr_debug("png_logo: License expanded size: %0d\n", (int) zstream.total_out);

	crc = ~crc32(~0x0, z_stream_buffer, zstream.total_out);
	if (!memcmp(&crc, &logo_license[LICENSE_DATA_LENGTH], LICENSE_CRC_LENGTH)) {
		*license_type = LICENSE_GENERAL;
		ret = 0;
	}else{
		pr_info("png_logo: license  invalid\n");
	}

free_zstream_workspace:
	if(logo_license != NULL) {
		if(zstream.workspace)
			vfree(zstream.workspace);
	}
nothing_to_free:
	return ret;
}

#ifdef CONFIG_OF
#include <linux/of.h>
static uint8_t logo_license_dt[LICENSE_LENGTH];
static int logo_settings_from_dt(uint8_t ** license, int * rotation)
{
	const char * compatible = "guf,png-logo";
	struct device_node *np;
	int ret, ret2, rot;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
	{
		pr_err("pnglogo: Couldn't find node: %s\n", compatible);
		ret = -ENODEV;
		goto out;
	}
	
	ret = of_property_read_u8_array(np, "guf,logo-license", logo_license_dt, LICENSE_LENGTH );
	if(ret)
	{
		pr_warn("pnglogo: No generic logo license found in device tree\n");
		ret = -ENODEV;
	}else{
		*license = logo_license_dt;
	}
	ret2 = of_property_read_u32(np, "rotation", &rot );
	if(ret2)
	{
		pr_warn("pnglogo: No logo rotation found in device tree\n");
	}
	else{
		switch(rot)
		{
		case 90:
			*rotation = FB_ROTATE_CW;
			break;
		case 180:
			*rotation = FB_ROTATE_UR;
			break;
		case 270:
			*rotation = FB_ROTATE_CCW;
			break;
		case 0:
		default:
			*rotation = FB_ROTATE_UD;
			break;
		}
		pr_warn("pnglogo: rotation %d : %d from device tree\n", rot, *rotation);
	}

out:
	of_node_put(np);	
	if(ret)	
		return ret;
	return 0;
}
#endif


#ifdef PROFILE_PNG_LOGO
#include <linux/time.h>
#endif

void __init_refok fb_show_pnglogo(struct fb_info *info, int rotate,
		const struct linux_logo *logo, uint8_t *logo_license)
{
	struct linux_logo fb_png_logo;
	png_structp		png_ptr = NULL;
	png_infop		info_ptr = NULL;
	row_info_t		row_info;
	unsigned char 	*fb_png_logo_start_virt = NULL;
	int bytes_read;
	int buffersize = 128;
	uint8_t * lic = 0;
	const struct linux_logo *default_logo = logo;

	
#ifdef CONFIG_OF
	logo_settings_from_dt(&lic, &rotate);
	if(!logo_license && lic)
			logo_license = lic;
#endif

#ifdef PROFILE_PNG_LOGO
	struct timeval start, stop;
	int end_buffersize = 270000;

	buffersize = 64;

next_profile:
#endif

redraw_logo:

#ifdef DEBUG
	dump_fbinfo(info);
	dump_logo(logo);
#endif
	// Fill screen with black, always, so that it is black if we don't draw because of license issues.
	if(fb_fill_block(info, 0, 0, info->var.xres, info->var.yres, rotate, 0x0, 0x0, 0x0))
		goto out;

	switch(fb_show_logo_type)
	{
	case LOGO_BLACK:
		/* Its black anyway */
		pr_info("png_logo: blank screen\n");
		goto out;
	case LOGO_TEST:
		fb_show_testlogo(info, rotate);
		pr_info("png_logo: show test logo\n");
		goto out;
	case LOGO_PNG:
		if(!fb_png_logo_start)
		{
			pr_err("pnglogo: selected logopng but no address passed\n");
			goto out;
		}

		fb_png_logo_start_virt = (char*)phys_to_virt(fb_png_logo_start);
		if (fb_png_logo_start_virt == NULL) {
			pr_err("pnglogo: error converting physical to virtual address of passed logo\n");
			goto out;
		}
		fb_png_logo.data = fb_png_logo_start_virt;
		fb_png_logo.type = LINUX_LOGO_PNG;
		logo = &fb_png_logo;
		pr_info("png_logo: using logo passed from physical address 0x%08lx\n", fb_png_logo_start);
		break;
	case LOGO_DEFAULT:
	default:
		if(!logo)
		{
			pr_err("%s: Logo not set, using BLACK\n", __func__);
			fb_show_logo_type = LOGO_BLACK;
			goto out;
		}
		; // just continue, logo is already set by caller
	}

#ifdef DEBUG
	dump_logo(logo);
#endif
	if(license_type != LICENSE_GENERAL )	// the license from the xml file is only stored for one fb but valid for all
		if(check_generic_logo_license( logo_license, &license_type))
			goto out;

	row_info.x_offs = -1; /* center per default */
	row_info.y_offs = -1; /* center per default */
	row_info.shown = 0;
	row_info.crc = 0;
	row_info.width = 0;		/* fb_info_callback will overwrite this */
	row_info.rotate = rotate;
	row_info.fb_info = info;
	row_info.bkg_color = 0;

	/* initialize libpng */
	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL,
									 (png_error_ptr)NULL, (png_error_ptr)NULL);

	if (!png_ptr) {
		pr_err("pnglogo: error creating read struct\n");
		goto out;
	}

	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr) {
		pr_err("pnglogo: error_creating info struct\n");
		goto destroy_read_struct;
	}

	/* tell libpng about the data to be passed to the callback functions */
	png_set_progressive_read_fn(png_ptr, (png_voidp)&row_info,
		fb_info_callback,
		LICENSE_GENERAL == license_type ? fb_row_callback : NULL,	// Just don't draw when no license is available
		fb_end_callback);
	if( LICENSE_GENERAL != license_type)
		png_set_read_user_chunk_fn(png_ptr, NULL, fb_read_chunk_callback);
	png_set_keep_unknown_chunks(png_ptr, PNG_HANDLE_CHUNK_ALWAYS, NULL, 0);

	// set return point in case of errors
	if (setjmp(png_jmpbuf(png_ptr)))
	{
		static int once = 0;

		if(!once &&  default_logo != logo)
		{
			pr_warn("png_logo: passed logo has errors, fallback to default logo\n");
			//destroy_info_struct:
			if(png_ptr && info_ptr)
				png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
			//destroy_read_struct:
			if(png_ptr && info_ptr)
				png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
			// Get default logo
			logo = default_logo;
			fb_show_logo_type = LOGO_DEFAULT;
			goto redraw_logo;
		}else{
			// libpng will longjmp() here in case of errors
			pr_err("png_logo: ERROR PNG-file corrupted\n");
			goto destroy_info_struct;
		}
	}

#ifdef PROFILE_PNG_LOGO
	do_gettimeofday(&start);
#endif

	row_info.all_read = 0;
	bytes_read = 0;
	do{
		if (!row_info.crc)
			row_info.crc = ~crc32(~0x0, &logo->data[64], 64);

		png_process_data(png_ptr, info_ptr, (png_bytep)&logo->data[bytes_read], buffersize);
		bytes_read += buffersize;
	}while(!row_info.all_read);

destroy_info_struct:
	if(png_ptr && info_ptr)
		png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
destroy_read_struct:
	if(png_ptr && info_ptr)
		png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
out:

#ifdef PROFILE_PNG_LOGO
	do_gettimeofday(&stop);
	pr_info("png_logo profile: Buffersize 0x%05X, bytes: %d: %05d us\n",
		buffersize,
		bytes_read,
		(int) ((stop.tv_sec*1000+stop.tv_usec) - (start.tv_sec*1000+start.tv_usec) ));
	buffersize *= 2;
#ifdef DEBUG
	dump_logo(logo);
#endif
	if( buffersize < end_buffersize)
		goto next_profile;
#endif
	return;
}
