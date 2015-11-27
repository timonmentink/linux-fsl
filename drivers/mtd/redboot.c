/*
 * Parse RedBoot-style Flash Image System (FIS) tables and
 * produce a Linux partition array to match.
 *
 * Copyright © 2001      Red Hat UK Limited
 * Copyright © 2001-2010 David Woodhouse <dwmw2@infradead.org>
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/vmalloc.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/module.h>

#ifdef CONFIG_MTD_GUF_REDBOOT_PARSING
#include <linux/ezxml.h>
#endif

#define GUF_FISDIR_SIZE 0x10000
#define ACTIVE_FIS_DIRECTORY_SUFFIX " (ACTIVE)"
#define DRIVER_NAME "mtd_redboot"

#ifdef CONFIG_MTD_GUF_REDBOOT_PARSING
unsigned long current_fis_addr = 0, backup_fis_addr = 0, fis_length = 0;
#endif

struct fis_image_desc {
    unsigned char name[16];      // Null terminated name
    uint32_t	  flash_base;    // Address within FLASH of image
    uint32_t	  mem_base;      // Address in memory where it executes
    uint32_t	  size;          // Length of image
    uint32_t	  entry_point;   // Execution entry point
    uint32_t	  data_length;   // Length of actual data
    unsigned char _pad[256-(16+7*sizeof(uint32_t))];
    uint32_t	  desc_cksum;    // Checksum over image descriptor
    uint32_t	  file_cksum;    // Checksum over image data
};

struct fis_list {
	struct fis_image_desc *img;
	struct fis_list *next;
};

static int directory = CONFIG_MTD_REDBOOT_DIRECTORY_BLOCK;
module_param(directory, int, 0);

extern unsigned long crc32(const unsigned char *buf, unsigned int len);

static inline int redboot_checksum(struct fis_image_desc *img)
{
	/* RedBoot doesn't actually write the desc_cksum field yet AFAICT */
	return 1;
}

static int parse_redboot_partitions(struct mtd_info *master,
				    struct mtd_partition **pparts,
				    struct mtd_part_parser_data *data)
{
	int nrparts = 0;
	struct fis_image_desc *buf;
	struct mtd_partition *parts;
	struct fis_list *fl = NULL, *tmp_fl;
	int ret, i;
	size_t retlen;
	char *names;
	char *nullname;
	int namelen = 0;
	int nulllen = 0;
	int numslots;
	unsigned long offset;
#ifdef CONFIG_MTD_REDBOOT_PARTS_UNALLOCATED
	static char nullstring[] = "unallocated";
#endif

	if ( directory < 0 ) {
		offset = master->size + directory * master->erasesize;
		while (mtd_block_isbad(master, offset)) {
			if (!offset) {
			nogood:
				printk(KERN_NOTICE "Failed to find a non-bad block to check for RedBoot partition table\n");
				return -EIO;
			}
			offset -= master->erasesize;
		}
	} else {
		offset = directory * master->erasesize;
		while (mtd_block_isbad(master, offset)) {
			offset += master->erasesize;
			if (offset == master->size)
				goto nogood;
		}
	}
	buf = vmalloc(master->erasesize);

	if (!buf)
		return -ENOMEM;

	printk(KERN_NOTICE "Searching for RedBoot partition table in %s at offset 0x%lx\n",
	       master->name, offset);

	ret = mtd_read(master, offset, master->erasesize, &retlen,
		       (void *)buf);

	if (ret)
		goto out;

	if (retlen != master->erasesize) {
		ret = -EIO;
		goto out;
	}

	numslots = (master->erasesize / sizeof(struct fis_image_desc));
	for (i = 0; i < numslots; i++) {
		if (!memcmp(buf[i].name, "FIS directory", 14)) {
			/* This is apparently the FIS directory entry for the
			 * FIS directory itself.  The FIS directory size is
			 * one erase block; if the buf[i].size field is
			 * swab32(erasesize) then we know we are looking at
			 * a byte swapped FIS directory - swap all the entries!
			 * (NOTE: this is 'size' not 'data_length'; size is
			 * the full size of the entry.)
			 */

			/* RedBoot can combine the FIS directory and
			   config partitions into a single eraseblock;
			   we assume wrong-endian if either the swapped
			   'size' matches the eraseblock size precisely,
			   or if the swapped size actually fits in an
			   eraseblock while the unswapped size doesn't. */
			if (swab32(buf[i].size) == master->erasesize ||
			    (buf[i].size > master->erasesize
			     && swab32(buf[i].size) < master->erasesize)) {
				int j;
				/* Update numslots based on actual FIS directory size */
				numslots = swab32(buf[i].size) / sizeof (struct fis_image_desc);
				for (j = 0; j < numslots; ++j) {

					/* A single 0xff denotes a deleted entry.
					 * Two of them in a row is the end of the table.
					 */
					if (buf[j].name[0] == 0xff) {
				  		if (buf[j].name[1] == 0xff) {
							break;
						} else {
							continue;
						}
					}

					/* The unsigned long fields were written with the
					 * wrong byte sex, name and pad have no byte sex.
					 */
					swab32s(&buf[j].flash_base);
					swab32s(&buf[j].mem_base);
					swab32s(&buf[j].size);
					swab32s(&buf[j].entry_point);
					swab32s(&buf[j].data_length);
					swab32s(&buf[j].desc_cksum);
					swab32s(&buf[j].file_cksum);
				}
			} else if (buf[i].size < master->erasesize) {
				/* Update numslots based on actual FIS directory size */
				numslots = buf[i].size / sizeof(struct fis_image_desc);
			}
			break;
		}
	}
	if (i == numslots) {
		/* Didn't find it */
		printk(KERN_NOTICE "No RedBoot partition table detected in %s\n",
		       master->name);
		ret = 0;
		goto out;
	}

	for (i = 0; i < numslots; i++) {
		struct fis_list *new_fl, **prev;

		if (buf[i].name[0] == 0xff) {
			if (buf[i].name[1] == 0xff) {
				break;
			} else {
				continue;
			}
		}
		if (!redboot_checksum(&buf[i]))
			break;

		new_fl = kmalloc(sizeof(struct fis_list), GFP_KERNEL);
		namelen += strlen(buf[i].name)+1;
		if (!new_fl) {
			ret = -ENOMEM;
			goto out;
		}
		new_fl->img = &buf[i];
		if (data && data->origin)
			buf[i].flash_base -= data->origin;
		else
			buf[i].flash_base &= master->size-1;

		/* I'm sure the JFFS2 code has done me permanent damage.
		 * I now think the following is _normal_
		 */
		prev = &fl;
		while(*prev && (*prev)->img->flash_base < new_fl->img->flash_base)
			prev = &(*prev)->next;
		new_fl->next = *prev;
		*prev = new_fl;

		nrparts++;
	}
#ifdef CONFIG_MTD_REDBOOT_PARTS_UNALLOCATED
	if (fl->img->flash_base) {
		nrparts++;
		nulllen = sizeof(nullstring);
	}

	for (tmp_fl = fl; tmp_fl->next; tmp_fl = tmp_fl->next) {
		if (tmp_fl->img->flash_base + tmp_fl->img->size + master->erasesize <= tmp_fl->next->img->flash_base) {
			nrparts++;
			nulllen = sizeof(nullstring);
		}
	}
#endif
	parts = kzalloc(sizeof(*parts)*nrparts + nulllen + namelen, GFP_KERNEL);

	if (!parts) {
		ret = -ENOMEM;
		goto out;
	}

	nullname = (char *)&parts[nrparts];
#ifdef CONFIG_MTD_REDBOOT_PARTS_UNALLOCATED
	if (nulllen > 0) {
		strcpy(nullname, nullstring);
	}
#endif
	names = nullname + nulllen;

	i=0;

#ifdef CONFIG_MTD_REDBOOT_PARTS_UNALLOCATED
	if (fl->img->flash_base) {
	       parts[0].name = nullname;
	       parts[0].size = fl->img->flash_base;
	       parts[0].offset = 0;
		i++;
	}
#endif
	for ( ; i<nrparts; i++) {
		parts[i].size = fl->img->size;
		parts[i].offset = fl->img->flash_base;
		parts[i].name = names;

		strcpy(names, fl->img->name);
#ifdef CONFIG_MTD_REDBOOT_PARTS_READONLY
		if (!memcmp(names, "RedBoot", 8) ||
				!memcmp(names, "RedBoot config", 15) ||
				!memcmp(names, "FIS directory", 14)) {
			parts[i].mask_flags = MTD_WRITEABLE;
		}
#endif
		names += strlen(names)+1;

#ifdef CONFIG_MTD_REDBOOT_PARTS_UNALLOCATED
		if(fl->next && fl->img->flash_base + fl->img->size + master->erasesize <= fl->next->img->flash_base) {
			i++;
			parts[i].offset = parts[i-1].size + parts[i-1].offset;
			parts[i].size = fl->next->img->flash_base - parts[i].offset;
			parts[i].name = nullname;
		}
#endif
		tmp_fl = fl;
		fl = fl->next;
		kfree(tmp_fl);
	}
	ret = nrparts;
	*pparts = parts;
 out:
	while (fl) {
		struct fis_list *old = fl;
		fl = fl->next;
		kfree(old);
	}
	vfree(buf);
	return ret;
}

#ifdef CONFIG_MTD_GUF_REDBOOT_PARSING
static int parse_guf_redboot_partitions(struct mtd_info *master,
                             struct mtd_partition **pparts,
                             unsigned long fis_origin)
{
	int nrparts = 0;
	char *buf = NULL;
	struct mtd_partition *parts;
	int ret = 0, i = 0;
	size_t retlen;
	char *names;
	char *nullname;
	int namelen = 0;
	int activelen = strlen(ACTIVE_FIS_DIRECTORY_SUFFIX);
	unsigned long offset;
	unsigned long lowest_address = 0xFFFFFFFF, last_address = 0, current_address = 0;
	int generation = 0;
	unsigned long cksum_read = 0;
	unsigned long cksum_calc = 0;
	int xml_len = 0;

	struct ezxml *xml_img = 0;
	struct ezxml *xml_checktag, *xml_flash, *xml_cur_flash, *xml_cur_part = NULL, *xml_part;

	if(!current_fis_addr || !fis_length) {
		printk("Don't have FIS Directory information\n");
		goto out;
	}

	offset = current_fis_addr - CONFIG_MTD_GUF_REDBOOT_FLASH_BASE;

	buf = vmalloc(fis_length);
	if (!buf)
	{
		printk("%s: Buffer allocation failed (%s)\n", DRIVER_NAME, __func__);
		return -ENOMEM;
	}

	master->_read(master, offset,
			fis_length, &retlen, (void *)buf);

	if (retlen != fis_length) {
		printk("%s: Read Error Length Mismatch %u <-> %u (%s)\n", DRIVER_NAME, (unsigned int)retlen, (unsigned int)fis_length, __func__);
		ret = -EIO;
		goto out;
	}

	xml_len = strlen(buf)+1;
	memcpy(&cksum_read, buf + xml_len, sizeof(unsigned long));
	cksum_calc = crc32(buf, xml_len);
	if ((cksum_read != cksum_calc) ||
		memcmp(buf, "<?xml", 5))
	{
		printk("%s: XML-Data corrupted (%s)\n", DRIVER_NAME, __func__);
		ret = -EIO;
		goto out;
	}

		/* Found XML config */
		xml_img = ezxml_parse_str(buf, strlen(buf));
		if(!xml_img) {
			printk("NO XML image found!\n");
			goto out;
		}
		/* Does it contain a flash tag? */
		xml_checktag = ezxml_child(xml_img, "flash");
		if(!xml_checktag) {
			printk("No XML checktag\n");
			ezxml_free(xml_img);
			goto out_free_xml;
		}
		else {
			/* Now we can parse
			 * the flash partitions
			 */
			xml_flash = ezxml_child(xml_img,
					"flash");
			if(!xml_flash) {
				printk("No XML flash tag found\n");
				goto out_free_xml;
			}
			generation = simple_strtol(
					ezxml_attr(xml_flash,
						"generation"), NULL, 0);
			printk("Using FIS Dir at %lX with generation %d\n", current_fis_addr, generation);
		}

	if(!xml_img) {
		printk(KERN_NOTICE "No GUF RedBoot partition table detected in %s\n",
		       master->name);
		ret = 0;
		goto out;
	}

	xml_cur_flash = ezxml_child(xml_img,"flash");

	xml_part  = ezxml_child(xml_cur_flash, "partition");

	if(!xml_part) {
		printk("No XML partition tag found\n");
		goto out_free_xml;
	}

	printk("Calculating space for partition data...\n");
	do {
		namelen += strlen(ezxml_attr(xml_part, "name")) + 1;
		xml_part = ezxml_next(xml_part);
		nrparts++;
	} while(xml_part);
	printk("%u bytes needed for %u partitions\n", namelen, nrparts);

	parts = kzalloc((sizeof(*parts) * nrparts) + namelen + activelen, GFP_KERNEL);
	if (!parts) {
		printk("%s: Failed allocating space for partition data (%s)\n", DRIVER_NAME,  __func__);
		ret = -ENOMEM;
		goto out_free_xml;
	}
	nullname = (char *)&parts[nrparts];
	names = nullname;

	printk("Registering partitions...\n");
	do {
		xml_cur_part = NULL;
		lowest_address = 0xFFFFFFFF;

		/* Find lowest address */
		for(xml_part  = ezxml_child(xml_cur_flash, "partition");
				xml_part;
				xml_part = ezxml_next(xml_part)) {
			current_address = simple_strtoul(ezxml_attr(xml_part, "flash_base"), NULL, 0);
			if((current_address >= last_address) &&	(current_address < lowest_address)) {
				        lowest_address = current_address;
					xml_cur_part = xml_part;
				}
		}

		if(xml_cur_part) {
			parts[i].offset = simple_strtoul(ezxml_attr(xml_cur_part, "flash_base"), NULL, 0) - CONFIG_MTD_GUF_REDBOOT_FLASH_BASE;
			parts[i].size   = simple_strtoul(ezxml_attr(xml_cur_part, "flash_length"), NULL, 0);
			strcpy(names, (ezxml_attr(xml_cur_part, "name")));
			parts[i].name   = names;
			if(parts[i].offset == offset)
				strcat(parts[i].name, ACTIVE_FIS_DIRECTORY_SUFFIX);
		}

		names += strlen(names)+1;
		last_address = lowest_address + 1;
		i++;
	} while(xml_cur_part);
	printk("%u partitions registered\n", i);

	ret = nrparts;
	*pparts = parts;

out_free_xml:
	ezxml_free(xml_img);
out:
	vfree(buf);
	return ret;
}
 
static struct mtd_part_parser guf_redboot_parser = {
	.owner = THIS_MODULE,
	.parse_fn = parse_guf_redboot_partitions,
	.name = "GufRedBoot",
};

static int parse_guf_fis_table(char* str)
{
	char *opt = NULL;

	if(str) {
		opt = strsep(&str, ",");
		current_fis_addr = simple_strtoul(opt, NULL, 0);
	}

	if(str) {
		opt = strsep(&str, ",");
		backup_fis_addr = simple_strtoul(opt, NULL, 0);
	}

	if(str) {
		opt = strsep(&str, ",");
		fis_length = simple_strtoul(opt, NULL, 0);
	}

	return 0;
}

__setup("rbfis=",parse_guf_fis_table);
#endif

static struct mtd_part_parser redboot_parser = {
	.owner = THIS_MODULE,
	.parse_fn = parse_redboot_partitions,
	.name = "RedBoot",
};

/* mtd parsers will request the module by parser name */
MODULE_ALIAS("RedBoot");

static int __init redboot_parser_init(void)
{
#ifdef CONFIG_MTD_GUF_REDBOOT_PARSING
	return register_mtd_parser(&guf_redboot_parser);
#else
	return register_mtd_parser(&redboot_parser);
#endif

}

static void __exit redboot_parser_exit(void)
{
#ifdef CONFIG_MTD_GUF_REDBOOT_PARSING
	deregister_mtd_parser(&guf_redboot_parser);
#else
	deregister_mtd_parser(&redboot_parser);
#endif
}

module_init(redboot_parser_init);
module_exit(redboot_parser_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("Parsing code for RedBoot Flash Image System (FIS) tables");
