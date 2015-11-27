
/*
 *  Convert a logo in PNG format to a C character array suitable for inclusion
 *  in the Linux kernel
 *
 *  (C) Copyright 2013 by Carsten Behling <behlin_c@gmx.de>
 *
 *  Derived from pnmtologo.c by Geert Uytterhoeven <geert@linux-m68k.org>
 *
 *  --------------------------------------------------------------------------
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of the Linux
 *  distribution for more details.
 */

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *programname;
static const char *filename;
static const char *logoname = "png_logo";
static const char *outputname;
static FILE *out;
static int zeroterminated = 0;
static unsigned int size = 0;

static void die(const char *fmt, ...)
    __attribute__ ((noreturn)) __attribute ((format (printf, 1, 2)));
static void usage(void) __attribute ((noreturn));

static void write_header(void)
{
    /* open logo file */
    if (outputname) {
	out = fopen(outputname, "w");
	if (!out)
	    die("Cannot create file %s: %s\n", outputname, strerror(errno));
    } else {
	out = stdout;
    }

    fputs("/*\n", out);
    fputs(" *  DO NOT EDIT THIS FILE!\n", out);
    fputs(" *\n", out);
    fprintf(out, " *  It was automatically generated from %s\n", filename);
    fputs(" *\n", out);
    fprintf(out, " *  Linux logo %s\n", logoname);
    fputs(" */\n\n", out);
	fprintf(out, "#include <linux/linux_logo.h>\n\n");
    fprintf(out, "static unsigned char %s_data[] __initdata = {\n\t",
	    logoname);
}

static void write_footer(void)
{
    fputs("\n};\n\n", out);

    fprintf(out, "const struct linux_logo %s __initconst = {\n", logoname);
    fprintf(out, "\t.type\t\t= LINUX_LOGO_PNG,\n");
    fprintf(out, "\t.data\t\t= %s_data\n", logoname);
    fputs("};\n\n", out);

    /* close logo file */
    if (outputname)
		fclose(out);
}

static int checked_fgetc(FILE *f)
{
	int c = fgetc(f);
	if (c == EOF && zeroterminated) {
		zeroterminated = 0;
		return 0;
	}
	return c;
}

static void write_logo(void)
{
    FILE *fp;
	int c, col = 1;

	fp = fopen(filename, "rb");
	if (!fp)
		die("Cannot open file %s: %s\n", filename, strerror(errno));

	/* write file header */
    write_header();

	while ((c = checked_fgetc(fp)) != EOF) {
		if (col >= 78 - 6) {
			fputc('\n', out);
			fputc('\t', out);
			col = 1;
		}
		fprintf(out, "0x%.2x, ", c);
		col += 6;
		size++;
	}

    /* write logo structure and file footer */
    write_footer();
}

static void die(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);

    exit(1);
}

static void usage(void)
{
    die("\n"
	"Usage: %s [options] <filename>\n"
	"\n"
	"Valid options:\n"
	"    -h          : display this usage information\n"
	"    -n <name>   : specify logo name (default: linux_logo)\n"
	"    -o <output> : output to file <output> instead of stdout\n"
	"\n", programname);
}

int main(int argc, char *argv[])
{
    int opt;

    programname = argv[0];

    opterr = 0;
    while (1) {
		opt = getopt(argc, argv, "hn:o:");
		if (opt == -1)
			break;

		switch (opt) {
			case 'h':
				usage();
			break;

			case 'n':
				logoname = optarg;
			break;

			case 'o':
				outputname = optarg;
			break;

			default:
				usage();
			break;
		}
    }
    if (optind != argc-1)
	usage();

    filename = argv[optind];
	
	write_logo();
	
    exit(0);
}
