/*
 * guf_xml.c - Driver for Garz & Fricke XML to sysfs conversion
 *
 * (C) 2011 by Phillip Durdaut <phillip.durdaut@garz-fricke.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ezxml.h>
#include <linux/guf_xml_config.h>
#include <linux/mtd/mtd.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>

#include "crc32.h"

#define DRIVER_NAME 			"guf_xml"
#define ENTRY_TAG				"configurationFile"
#define DEBUG_MODE				0
#define MAX_PATH_LENGTH 		500		/* incl. terminating '\0' */
#define MAX_TAG_LENGTH 			50		/* incl. terminating '\0' */
#define MAX_GENERATION_LENGTH	10
#define ATTR_READ				0
#define ATTR_WRITE				1

struct guf_xml_multiple_tags {
	char *path;
	char *identifier;
};

struct guf_xml_tree_memory {
	int kobj_name_bytes_cnt;
	char *kobj_name_begin;
	char *kobj_name;
	int attr_group_bytes_cnt;
	struct attribute_group *attr_group_begin;
	struct attribute_group *attr_group;
	int attr_group_attrs_bytes_cnt;
	struct attribute **attr_group_attrs_begin;
	struct attribute **attr_group_attrs;
	int attr_name_bytes_cnt;
	char *attr_name_begin;
	char *attr_name;
	int kattr_bytes_cnt;
	struct kobj_attribute *kattr_begin;
	struct kobj_attribute *kattr;
};

static struct guf_xml_multiple_tags mtag_flash_partition = {
	.path = "/"ENTRY_TAG"/flash/partition",
	.identifier = "name",
};

static struct guf_xml_multiple_tags mtag_variables_setting = {
	.path = "/"ENTRY_TAG"/variables/setting",
	.identifier = "key",
};

static struct guf_xml_multiple_tags mtag_end_of_list = {
	.path = NULL,
	.identifier = NULL,
};

static struct guf_xml_multiple_tags *mtags[] = {
	&mtag_flash_partition,
	&mtag_variables_setting,
	/* another multiple tag goes here */
	&mtag_end_of_list,
};

static struct guf_xml_tree_memory tree_memory = {
	.kobj_name_bytes_cnt = 0,
	.kobj_name_begin = NULL,
	.kobj_name = NULL,
	.attr_group_bytes_cnt = 0,
	.attr_group_begin = NULL,
	.attr_group = NULL,
	.attr_group_attrs_bytes_cnt = 0,
	.attr_group_attrs_begin = NULL,
	.attr_group_attrs = NULL,
	.attr_name_bytes_cnt = 0,
	.attr_name_begin = NULL,
	.attr_name = NULL,
	.kattr_bytes_cnt = 0,
	.kattr_begin = NULL,
	.kattr = NULL,
};

int guf_xml_write_xml_to_partition(struct ezxml *xml_root);

static struct miscdevice guf_xml_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRIVER_NAME,
};

static struct mtd_info *mtd_fis_directory;	/* mtd information FIS directory */
static struct mtd_info *mtd_redundant_fis;	/* mtd information Redundant FIS */
static struct mtd_info *mtd_active;			/* points on the currently active device */
static struct mtd_info *mtd_backup;			/* points on the currently backup device */
static int eb_cnt;							/* number of eraseblocks */
static int pg_cnt;							/* pages per eraseblock */
static unsigned char *bbt;					/* bad block table */
static struct mutex mutex;
static struct kobject *kobj_root = NULL;

/*
 *  Makes mtd_backup become the new mtd_active.
 */
static void guf_xml_switch_mtd_devices(void)
{
	struct mtd_info *mtd_tmp;

	/* swap the mtd_active and mtd_backup pointers */
	mtd_tmp = mtd_active;
	mtd_active = mtd_backup;
	mtd_backup = mtd_tmp;
}

/*
 *  Checks whether the eraseblock with the given number in the given mtd
 *  device is a bad block or not.
 *  Returns 1 when block is bad and 0 when not.
 */
static int guf_xml_is_block_bad(struct mtd_info *mtd, int eb_num)
{
	loff_t addr = 0;

	if (!mtd) {
		printk(KERN_ERR "%s: mtd device uninitialized (%s)\n", DRIVER_NAME, __func__);
		return 1;
	}

	addr = eb_num * mtd->erasesize;

	/* NOR flash does not implement block_isbad */
	if (mtd->_block_isbad == NULL)
		return 0;

	if (mtd->_block_isbad(mtd, addr))
		return 1;
	else
		return 0;
}

/*
 *  Scans the given mtd device for bad blocks.
 *  Returns the number of bad blocks found and a negative number on failure.
 */
static int guf_xml_scan_for_bad_eraseblocks(struct mtd_info *mtd)
{
	int i;
	int	bad_block_cnt = 0;

	if (bbt == 0) {
		bbt = vmalloc(eb_cnt * sizeof(unsigned char));
		if (bbt == NULL) {
			printk(KERN_ERR "%s: failed to allocate memory (%s)\n", DRIVER_NAME, __func__);
			return -1;
		}
		memset(bbt, 0, eb_cnt * sizeof(unsigned char));		/* init memory */
	}

	if (DEBUG_MODE) printk(KERN_INFO "%s: scanning for bad eraseblocks\n", DRIVER_NAME);
	for (i = 0; i < eb_cnt; ++i) {
		bbt[i] = guf_xml_is_block_bad(mtd, i);
		if (bbt[i])
			bad_block_cnt++;
		cond_resched();
	}
	if (DEBUG_MODE) printk(KERN_INFO "%s: scanned %d eraseblocks, %d are bad\n", DRIVER_NAME, i, bad_block_cnt);

	return bad_block_cnt;
}

/*
 *  Erases the eraseblock with the given number in the given mtd device.
 *  Returns 0 on success and a negative number on failure.
 */
static int guf_xml_erase_eraseblock(struct mtd_info *mtd, int eb_num)
{
	int err = 0;
	struct erase_info ei;
	loff_t addr = 0;

	if (!mtd || !mtd->_erase) {
		printk(KERN_ERR "%s: mtd device uninitialized (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	addr = eb_num * mtd->erasesize;

	memset(&ei, 0, sizeof(struct erase_info));
	ei.mtd  = mtd;
	ei.addr = addr;
	ei.len  = mtd->erasesize;

	err = mtd->_erase(mtd, &ei);

	if (err || ei.state == MTD_ERASE_FAILED) {
		printk(KERN_ERR "%s: failure on erasing eraseblock %d (%s)\n", DRIVER_NAME, eb_num, __func__);
		return -1;
	}

	return 0;
}

/*
 *  Writes the given buffer to the eraseblock with the given number in the given mtd device.
 *  Returns 0 on success and a negative number on failure.
 */
static int guf_xml_write_eraseblock(struct mtd_info *mtd, int eb_num, char *buf)
{
	size_t written = 0;
	int err = 0;
	loff_t addr = 0;

	if (!mtd || !mtd->_write || !buf) {
		printk(KERN_ERR "%s: mtd device uninitialized (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	addr = eb_num * mtd->erasesize;

	buf += addr;
	err = mtd->_write(mtd, addr, mtd->erasesize, &written, buf);
	if (err || (written != mtd->erasesize)) {
		printk(KERN_ERR "%s: failure on writing on mtd device at 0x%llx (%s)\n", DRIVER_NAME, (long long)addr, __func__);
		return -1;
	}

	return 0;
}

/*
 *  Checks the given mtd device whether there are enough well-being blocks for writing the buffer.
 *  Returns 0 on success and a negative number on failure.
 */
static int guf_xml_write_mtd_device(struct mtd_info *mtd, char *buf, int buf_size)
{
	int bad_block_cnt;
	int available_size;
	int i;
	int err = 0;

	if (!mtd) {
		printk(KERN_ERR "%s: mtd device uninitialized (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	bad_block_cnt = guf_xml_scan_for_bad_eraseblocks(mtd);
	if (bad_block_cnt < 0) {
		printk(KERN_ERR "%s: failed to scan for bad blocks (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	available_size = (eb_cnt - bad_block_cnt) * mtd->erasesize;
	if (buf_size > available_size) {
		printk(KERN_ERR "%s: not enough space in flash (%d bytes free) for the data (%d bytes) (%s)\n", DRIVER_NAME, available_size, buf_size, __func__);
		return -1;
	}

	/* erase flash before writing on it */
	for (i = 0; i < eb_cnt; ++i) {
		if (bbt[i] == 0) {	/* only erase well-being blocks */
			err = guf_xml_erase_eraseblock(mtd, i);
			if (err) {
				printk(KERN_ERR "%s: failed to erase block %d (%s)\n", DRIVER_NAME, i, __func__);
				return -1;
			}
		}
	}

	/* write to flash and jump over bad blocks */
	for (i = 0; i < eb_cnt; ++i) {
		if (bbt[i] == 0) {	/* only write well-being blocks */

			err = guf_xml_write_eraseblock(mtd, i, buf);
			if (err) {
				printk(KERN_ERR "%s: failed to write to block %d (%s)\n", DRIVER_NAME, i, __func__);
				return -1;
			}
		}
	}

	return 0;
}

/*
 *  Reads the eraseblock with the given number in the given mtd device
 *  and writes the result in the buffer.
 *  Returns 0 on success and a negative number on failure.
 */
static int guf_xml_read_eraseblock(struct mtd_info *mtd, int eb_num, char *buf)
{
	size_t read = 0;
	int err = 0;
	loff_t addr = 0;

	if (!mtd || !mtd->_read || !buf) {
		printk(KERN_ERR "%s: mtd device uninitialized (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	addr = eb_num * mtd->erasesize;

	buf += addr;
	err = mtd->_read(mtd, addr, mtd->erasesize, &read, buf);
	if ((err) || (read != mtd->erasesize)) {
		printk(KERN_ERR "%s: failure on reading from mtd device at 0x%llx (%s), error: %d. (%d <-> %d)\n", DRIVER_NAME, (long long)addr, __func__, err, read, mtd->erasesize);
		return -1;
	}

	return 0;
}

/*
 *  Reads data from the given mtd device into a character string.
 *  When reading was successful a pointer to the character array is returned, otherwise NULL.
 *  The returned array has to be freed after using it!
 */
static char* guf_xml_read_partition_to_xml(struct mtd_info *mtd)
{
	char *buf = NULL;
	int bad_block_cnt;
	int i = 0;
	int err = 0;
	unsigned long cksum_read = 0;
	unsigned long cksum_calc = 0;
	int xml_len = 0;

	if (!mtd) {
		printk(KERN_ERR "%s: mtd device uninitialized (%s)\n", DRIVER_NAME, __func__);
		return NULL;
	}

	bad_block_cnt = guf_xml_scan_for_bad_eraseblocks(mtd);
	if (bad_block_cnt < 0) {
		printk(KERN_ERR "%s: failed to scan for bad blocks (%s)\n", DRIVER_NAME, __func__);
		return NULL;
	}

	buf = vmalloc(mtd->size);
	if (buf == NULL) {
		printk(KERN_ERR "%s: failed to allocate memory (%s)\n", DRIVER_NAME, __func__);
		return NULL;
	}
	memset(buf, 0, mtd->size);		/* init memory */

	/* read from flash and jump over bad blocks */
	for (i = 0; i < eb_cnt; ++i) {
		if (bbt[i] == 0) {	/* only read well-being blocks */
			if (guf_xml_read_eraseblock(mtd, i, buf))
				err++;
			}
		}

	// does the data start with an XML-header?
    if (memcmp(buf, "<?xml", 5))
    {
		printk(KERN_ERR "%s: no XML configuration found!!!\n", __func__);
				goto err_free;
	}

	// do we have a valid checksum for the XML data?
	xml_len = strlen(buf)+1;
	memcpy(&cksum_read, buf + xml_len, sizeof(unsigned long));
	cksum_calc = crc32(buf, xml_len);
	if (cksum_read != cksum_calc)
	{
		printk(KERN_ERR "%s: XML Checksum Error!!!\n", __func__);
    	goto err_free;
	}

	// if read-function detected any ECC errors in flash but XML checksum
	// is correct, we still have usable XML data but the flash area after the
	// XML-data got corrupted. Recover by writing the XML-configuration twice,
	// so both current and backup data get rewritten and all ECC-erros will
	// disappear.
	if (err)
	{
		struct ezxml *xml_root;

		printk("%s: found ECC-errors in unused space of the XML-partition. Fixing...\n", __func__);
		xml_root = ezxml_parse_str(buf, xml_len);

		if (xml_root == NULL) {
			printk(KERN_ERR "%s: failure on parsing xml data\n", __func__);
			goto err_free;
		}

		// re-write XML configuration
		if (guf_xml_write_xml_to_partition(xml_root))
			printk(KERN_ERR "%s: XML recovery step 1 failed\n", __func__);

		if (guf_xml_write_xml_to_partition(xml_root))
			printk(KERN_ERR "%s: XML recovery step 2 failed\n", __func__);

		// re-read XML-configuration so we return consistent data w.r.t generation count
		ezxml_free(xml_root);
		vfree(buf);
		return guf_xml_read_partition_to_xml(mtd);
	}

	return buf;

err_free:
	vfree(buf);
	printk(KERN_ERR "%s: unrecoverable error reading XML-data\n", __func__);
	return NULL;
}

/*
 *  Writes XML data to the backup MTD partition, including update of the
 *  generation number and appending the CRC32 checksum. Afterwards, the
 *  new partition is switched to active.
 *  When writing was successful, 0 is returned, else -1.
 */
int guf_xml_write_xml_to_partition(struct ezxml *xml_root)
{
	char *new_xml_string = NULL, *buffer = NULL;
	struct ezxml *xml_flash = NULL;
	int err = 0;
	int result = -1;
	unsigned int xml_len = 0;
	unsigned long checksum;
	int generation;
	char new_generation[MAX_GENERATION_LENGTH];

	/* get the current generation value*/
	xml_flash = ezxml_child(xml_root, "flash");
	if (xml_flash == NULL) {
		printk(KERN_ERR "%s: could not find any information about the flash device in xml data (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}
	generation = simple_strtol((char *)ezxml_attr(xml_flash, "generation"), NULL, 0);

	/* increase the generation tag by one */
	sprintf(new_generation, "%d", generation + 1);
	if (ezxml_set_attr(xml_flash, "generation", new_generation) == NULL) {
		printk(KERN_ERR "%s: could not set the generation attribute in xml data (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	new_xml_string = ezxml_toxml(xml_root);	/* new_xml_string has to be freed */
	if (!new_xml_string) {
		printk(KERN_ERR "%s: could not write xml data to string (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}
	xml_len = strlen(new_xml_string) + 1;

	/* copy xml string to new, bigger buffer, so that the checksum can be appended */
	buffer = kmalloc(xml_len + sizeof(unsigned long), GFP_KERNEL);
	if (!buffer) {
		printk(KERN_ERR "%s: could not allocate memory for xml string (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}
	memcpy(buffer, new_xml_string, xml_len);
	checksum = crc32((unsigned char *)new_xml_string, xml_len);
	memcpy(buffer + xml_len, &checksum, sizeof(unsigned long));

	err = guf_xml_write_mtd_device(mtd_backup, buffer, xml_len + sizeof(unsigned long));
	if (err) {
		printk(KERN_ERR "%s: failure on writing on mtd device (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	/* the just written mtd_backup should now become the new mtd_active */
	guf_xml_switch_mtd_devices();
	result = 0;

error:
	/* use kfree as new_xml_string was allocated by ezxml lib with kmalloc */
	if (buffer)
		kfree(buffer);
	if (new_xml_string);
		kfree(new_xml_string);
	return result;
}

/*
 *  Reads the flash generation attribute from the given mtd device.
 *  Returns the generation or an error code smaller than 0 on failure.
 */
static int guf_xml_get_mtd_generation(struct mtd_info *mtd)
{
	struct ezxml *xml_root = NULL;
	struct ezxml *xml_flash = NULL;
	char* xml_string = NULL;
	int generation = -1;

	/* read the xml data from the partition */
	xml_string = guf_xml_read_partition_to_xml(mtd);
	if (xml_string == NULL) {
		printk(KERN_ERR "%s: failure on reading xml partition (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	/* parse the xml data */
	xml_root = ezxml_parse_str(xml_string, strlen(xml_string));
	if (xml_root == NULL) {
		printk(KERN_ERR "%s: failure on parsing xml data (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	xml_flash = ezxml_child(xml_root, "flash");
	if (xml_flash == NULL) {
		printk(KERN_ERR "%s: could not find any information about the flash device in xml data (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	/* get the generation value*/
	generation = simple_strtol((char *)ezxml_attr(xml_flash, "generation"), NULL, 0);

error:
	if (xml_root)
	ezxml_free(xml_root);
	if (xml_string)
	vfree(xml_string);
	return generation;
}

/*
 *  Reads the flash generation attribute from both mtd devices to determine
 *  which one is the active one and which is the backup.
 *  Returns 0 on success or an error code smaller than 0 on failure.
 */
static int guf_xml_update_mtd_devices(void)
{
	int generation_fis_directory = guf_xml_get_mtd_generation(mtd_fis_directory);
	int generation_redundant_fis = guf_xml_get_mtd_generation(mtd_redundant_fis);

	if (generation_fis_directory < 0 || generation_redundant_fis < 0) {
		printk(KERN_ERR "%s: failure on reading mtd device generation (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	/* should not happen */
	if (generation_fis_directory == generation_redundant_fis) {
		printk(KERN_ERR "%s: both mtd devices have the same generation value (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	if (generation_fis_directory > generation_redundant_fis) {
		mtd_active = mtd_fis_directory;
		mtd_backup = mtd_redundant_fis;
	}
	else {
		mtd_active = mtd_redundant_fis;
		mtd_backup = mtd_fis_directory;
	}

	return 0;
}

/*
 *  Gets the path to the given tag.
 *  If the tag is 'rotation' the path will for example look like:
 *		/ENTRY_TAG/variables/display/rotation
 *  If the parameter tag is NULL the resulting path will look like:
 *  	/ENTRY_TAG/variables/display
 *  Returns a pointer to the path or NULL on failure.
 *  The returned path has to be freed after using it!
 */
static char* guf_xml_get_path_to_tag(struct kobject *youngest_kobj, char *tag)
{
	char *path = NULL;
	struct kobject *kobj = NULL;

	path = vmalloc(MAX_PATH_LENGTH * sizeof(char));
	if (path == NULL) {
		printk(KERN_ERR "%s: failed to allocate memory (%s)\n", DRIVER_NAME, __func__);
		return NULL;
	}
	memset(path, 0, MAX_PATH_LENGTH * sizeof(char));		/* init memory */

	if (tag != NULL) {
		if (strlen(tag)+2 > MAX_PATH_LENGTH) {
			printk(KERN_ERR "%s: path too long (%s)\n", DRIVER_NAME, __func__);
			goto err_free;
		}
		*path = '/';
		strcpy(path + 1, tag);
	}

	for (kobj = youngest_kobj; (kobj != NULL) && (strcmp(kobj->name, DRIVER_NAME) != 0); kobj = kobj->parent){
		if (strlen(path)+2+strlen(kobj->name) > MAX_PATH_LENGTH) {
			printk(KERN_ERR "%s: path too long (%s)\n", DRIVER_NAME, __func__);
			goto err_free;
		}
		memmove(path + strlen(kobj->name) + 1, path, strlen(path) * sizeof(char));
		memcpy(path + 1, kobj->name, strlen(kobj->name) * sizeof(char));
		*path = '/';
	}

	if (kobj == NULL) {
		printk(KERN_ERR "%s: invalid kernel object address (%s)\n", DRIVER_NAME, __func__);
		goto err_free;
	}
	return path;

err_free:
	vfree(path);
	return NULL;
}

/*
 *  Determines whether the given tag is a multiple tag.
 *  Returns 0 if it is not or the index of the multiple tag in the mtags pointer
 *  vector plus 1.
 */
static ssize_t guf_xml_is_multiple_tag(char *path_to_tag)
{
	int i;

	/* run through the mtags vector */
	for (i = 1; mtags[i-1]->path != NULL; i++) {
		if (strncmp(mtags[i-1]->path, path_to_tag, strlen(mtags[i-1]->path)) == 0) {
			return i;
		}
	}

	return 0;
}

/*
 *  Counts and returns the number of attributes (key-value-pairs) under the given tag.
 */
static int guf_xml_count_attrs(struct ezxml *xml)
{
	int cnt = 0;
	char **attr_name_or_value = NULL;

	if (!xml || !xml->attr)
		return 0;

	for (attr_name_or_value = xml->attr; *attr_name_or_value != NULL; attr_name_or_value++) {
		cnt++;
	}

	return (cnt/2);
}

/*
 *  Checks if the current tag is a multiple tag.
 *  If so the name of the kernel object (which equals later one sysfs directory) has to be changed
 *  because there can not exist files or directorys with the same name.
 *  In this case the name of the kobject will be <tagname>_<identifier> where
 *  identifier is specified in guf_xml_multiple_tags on top of this file.
 *  Returns a pointer to the (un)changed kernel object name or NULL on failure.
 */
static int guf_xml_change_kobj_name(struct kobject *kobj, char *kobj_name, char *tag_name,
	char *attr_name, char *attr_value)
{
	char *path = NULL;
	int mtag_no;

	path = guf_xml_get_path_to_tag(kobj, tag_name);
	if (path == NULL) {
		printk(KERN_ERR "%s: could not get path to xml tag (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	mtag_no = guf_xml_is_multiple_tag(path);
	if (mtag_no) {
		if (strcmp(attr_name, mtags[mtag_no-1]->identifier) == 0) {
			if (strlen(tag_name) + strlen(attr_value) + 2 >= MAX_TAG_LENGTH) {
				printk(KERN_ERR "%s: name too long (%s)\n", DRIVER_NAME, __func__);
				vfree(path);
				return -1;
			}
			sprintf(kobj_name, "%s_%s", tag_name, attr_value);
			kobj_name[strlen(tag_name) + 1 + strlen(attr_value)] = '\0';
		}
	}

	vfree(path);
	return 0;
}

/*
 *  Walks through the xml data and tries to find the attribute defined by path,
 *  attr, identifier and key. If it is found it depends on rw whether the attribute
 *  is read or written with value. On success 0 is returned and in case of rw = ATTR_READ
 *  presult points on a pointer which points on the value read.
 *  On failure -1 is returned.
 */
static int guf_xml_rw_xml_attr(struct ezxml *xml, int rw, char *path,
	char *attr, char *value, int multiple_tag, char *identifier, char *key, char **presult)
{
	int i, j;
	char *cur_tag;
	char *cur_value;

	cur_tag = vmalloc(MAX_TAG_LENGTH * sizeof(char));
	if (cur_tag == NULL) {
		printk(KERN_ERR "%s: failed to allocate memory (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}
	memset(cur_tag, 0, MAX_TAG_LENGTH * sizeof(char));		/* init memory */

	if (*path == '/')
		path++;
	if (strncmp(path, ENTRY_TAG, strlen(ENTRY_TAG)) == 0)
		path += strlen(ENTRY_TAG);
	if (*path == '/')
		path++;

	i = 0;
	j = 0;
	while (i <= strlen(path)) {

		if (path[i] == '/' || path[i] == '\0') {
			cur_tag[j] = '\0';
			j = 0;

			if (DEBUG_MODE) printk(KERN_INFO "%s: trying to jump to tag '%s'\n", DRIVER_NAME, cur_tag);

			xml = ezxml_child(xml, cur_tag);
			if (xml == NULL && multiple_tag == 0) {
				printk(KERN_ERR "%s: tag '%s' was not found (%s)\n", DRIVER_NAME, cur_tag, __func__);
				goto err_free;
			}

			if (multiple_tag) {
				xml = xml->child;
				break;
			}
		}
		else {
			cur_tag[j] = path[i];
			j++;
		}

		i++;
	}

	if (multiple_tag) {
		if (DEBUG_MODE) printk(KERN_INFO "%s: searching for '%s=%s'\n", DRIVER_NAME, identifier, key);

		i = 0;
		while (1) {

			xml = ezxml_idx(xml, !!i);

			if (xml == NULL) {
				printk(KERN_ERR "%s: could not find '%s=%s' (%s)\n", DRIVER_NAME, identifier, key, __func__);
				goto err_free;
			}

			cur_value = (char *)ezxml_attr(xml, identifier);
			if (DEBUG_MODE) printk(KERN_INFO "%s: current attribute is '%s'\n", DRIVER_NAME, cur_value);

			if (strcmp(cur_value, key) == 0)
				break;

			i++;
		}
	}

	switch(rw) {
		case ATTR_READ:
			*presult = (char *)ezxml_attr(xml, attr);
			if (!(*presult)) {
				printk(KERN_ERR "%s: could not find the attribute '%s' in xml data where '%s=%s' (%s)\n", DRIVER_NAME, attr, identifier, key, __func__);
				goto err_free;
			}
		break;

		case ATTR_WRITE:
			if (ezxml_set_attr(xml, attr, value) == NULL) {
				printk(KERN_ERR "%s: could not set the attribute '%s' in xml data struct with the id key '%s' (%s)\n", DRIVER_NAME, attr, key, __func__);
				goto err_free;
			}
		break;
	}

	vfree(cur_tag);
	return 0;

err_free:
	vfree(cur_tag);
	return -1;
}

/*
 *  Calls guf_xml_read_partition_to_xml for reading and parsing xml data and
 *  passes every received parameter to guf_xml_get_attr_from_xml_data.
 *  Returns the result of guf_xml_get_attr_from_xml_data or NULL on failure.
 *  The returned string in *ppresult has to be freed after using it!
 */
static int guf_xml_get_attr(char *path, char *attr, int multiple_tag, char *identifier, char *key, char **ppresult)
{
	struct ezxml *xml_root = NULL;
	char* xml_string = NULL;
	char* pattr;
	int ret = -1;

	/* read the xml data from the partition */
	xml_string = guf_xml_read_partition_to_xml(mtd_active);
	if (xml_string == NULL) {
		printk(KERN_ERR "%s: failure on reading xml partition (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	/* parse the xml data */
	xml_root = ezxml_parse_str(xml_string, strlen(xml_string));
	if (xml_root == NULL) {
		printk(KERN_ERR "%s: failure on parsing xml data (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	ret = guf_xml_rw_xml_attr(xml_root, ATTR_READ, path, attr, NULL, multiple_tag, identifier, key, &pattr);
	if (pattr != NULL)
	{
		/* we have to copy the attribute value, otherwise it will get lost on freeing xml_string */
		*ppresult = vmalloc(strlen(pattr) + 1);
		if (*ppresult == NULL) {
			printk(KERN_ERR "%s: could not allocate memory (%s)\n", DRIVER_NAME, __func__);
			ret = -1;
			goto error;
		}
		strcpy(*ppresult, pattr);
	}

error:
	if (xml_root)
	ezxml_free(xml_root);
	if (xml_string)
	vfree(xml_string);

	return ret;
}

/*
 *  Calls guf_xml_read_partition_to_xml for reading and parsing xml data and
 *  passes every received parameter to guf_xml_set_attr_from_xml_data.
 *  The xml data gets modified accordingly and after calculating the crc32 checksum
 *  the new buffer is written to the mtd device.
 *  Returns 0 when finished successfully or -1 on failure.
 */
static int guf_xml_set_attr(char *path, char *attr, char *value, int multiple_tag, char *identifier, char *key)
{
	char *xml_string = NULL;
	struct ezxml *xml_root = NULL;
	int err = 0;
	int ret = -1;

	/* read the xml data from the partition */
	xml_string = guf_xml_read_partition_to_xml(mtd_active);
	if (xml_string == NULL) {
		printk(KERN_ERR "%s: failure on reading xml partition (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	/* parse the xml data */
	xml_root = ezxml_parse_str(xml_string, strlen(xml_string));
	if (xml_root == NULL) {
		printk(KERN_ERR "%s: failure on parsing xml data (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	err = guf_xml_rw_xml_attr(xml_root, ATTR_WRITE, path, attr, value, multiple_tag, identifier, key, NULL);
	if (err) {
		printk(KERN_ERR "%s: failed to set xml attribute (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	err = guf_xml_write_xml_to_partition(xml_root);
	if (err) {
		printk(KERN_ERR "%s: failed to write new XML configuration (%s)\n", DRIVER_NAME, __func__);
		goto error;
	}

	ret = 0;
error:
	if (xml_root)
	ezxml_free(xml_root);
	if (xml_string)
	vfree(xml_string);
	return ret;
}

/*
 *  Determines the parameters passing to guf_xml_get_attr and writes the result in the output buffer.
 *  On success the total number of characters written is returned.
 *  On failure a negative error code is returned.
 */
static ssize_t guf_xml_sysfs_attr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *path = NULL;
	char *key = NULL;
	char *presult = NULL;
	int err = -EFAULT;
	int mtag_no;

	mutex_lock(&mutex);

	path = guf_xml_get_path_to_tag(kobj, NULL);
	if (path == NULL) {
		printk(KERN_ERR "%s: could not get path to xml tag (%s)\n", DRIVER_NAME, __func__);
		goto err_free;
	}

	mtag_no = guf_xml_is_multiple_tag(path);
	if (mtag_no) {
		key = path + strlen(mtags[mtag_no-1]->path) + 1;
		err = guf_xml_get_attr(mtags[mtag_no-1]->path, (char *)(attr->attr.name), 1, mtags[mtag_no-1]->identifier, key, &presult);
	}
	else {
		err = guf_xml_get_attr(path, (char *)attr->attr.name, 0, NULL, NULL, &presult);
	}

	if (err || !presult) {
		printk(KERN_ERR "%s: failed to get sysfs attribute (%s)\n", DRIVER_NAME, __func__);
		err = -EIO;
	}
	else {
	err = sprintf(buf, "%s\n", presult);
	}

err_free:
	if (presult)
		vfree(presult);
	if (path)
	vfree(path);
	mutex_unlock(&mutex);
	return err;
}

/*
 *  Determines the parameters passing to guf_xml_set_attr.
 *  On success the count parameter is returned.
 *  On failure a negative error code is returned.
 */
static ssize_t guf_xml_sysfs_attr_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	char *path = NULL;
	char *key = NULL;
	char *target_buf = (char *)buf;
	int err = 0;
	int mtag_no;

	mutex_lock(&mutex);

	path = guf_xml_get_path_to_tag(kobj, NULL);
	if (path == NULL) {
		printk(KERN_ERR "%s: could not get path to xml tag (%s)\n", DRIVER_NAME, __func__);
		count = -EFAULT;
		goto err_free;
	}

	/* delete the line feed at the end of the input */
	if (target_buf[strlen(target_buf) - 1] == '\n')
		target_buf[strlen(target_buf) - 1] = '\0';

	mtag_no = guf_xml_is_multiple_tag(path);
	if (mtag_no) {
		key = path + strlen(mtags[mtag_no-1]->path) + 1;
		err = guf_xml_set_attr(mtags[mtag_no-1]->path, (char *)(attr->attr.name), target_buf, 1, mtags[mtag_no-1]->identifier, key);
	}
	else {
		err = guf_xml_set_attr(path, (char *)attr->attr.name, target_buf, 0, NULL, NULL);
	}

	if (err) {
		printk(KERN_ERR "%s: failed to set sysfs attribute (%s)\n", DRIVER_NAME, __func__);
		count = -EIO;
	}

err_free:
	if (path)
	vfree(path);
	mutex_unlock(&mutex);
	return count;
}

/*
 *  Walks recursively through the xml data tree and calculates the amount of
 *  memory needed for creating the sysfs tree.
 *  Returns 0 when finished successfully or an error code smaller than 0 on failure.
 */
static int guf_xml_calc_space_for_tree(struct ezxml *xml)
{
	char **attr_name_or_value = NULL;
	char *cur_attr_name = NULL;
	char *cur_attr_value = NULL;
	int attr_cnt;

	if (xml == NULL)
		return -1;

	tree_memory.attr_group_bytes_cnt += sizeof(struct attribute_group);

	/* count attributes finding under the current tag */
	attr_cnt = guf_xml_count_attrs(xml);
	tree_memory.attr_group_attrs_bytes_cnt += (attr_cnt + 1) * sizeof(struct attribute *);

	/* go through the list of pointers pointing alternately on the attributes name
	   and the attributes value */
	for (attr_name_or_value = xml->attr; *attr_name_or_value != NULL; attr_name_or_value++) {

		/* attr_name_or_value points on a pointer pointing on the current attributes name */
		if (!cur_attr_name && !cur_attr_value) {
			cur_attr_name = *attr_name_or_value;
		}
		/* attr_name_or_value points on a pointer pointing on the current attributes value */
		else if (cur_attr_name && !cur_attr_value) {
			cur_attr_value = *attr_name_or_value;
		}

		if (cur_attr_name && cur_attr_value) {

			tree_memory.attr_name_bytes_cnt += (strlen(cur_attr_name) + 1) * sizeof(char);
			tree_memory.kattr_bytes_cnt += sizeof(struct kobj_attribute);

			cur_attr_name = NULL;
			cur_attr_value = NULL;
		}
	}

	tree_memory.kobj_name_bytes_cnt += MAX_TAG_LENGTH * sizeof(char);

	/* jump to next child tag if exist */
	if (xml->child != NULL)
		if (guf_xml_calc_space_for_tree(xml->child) < 0)
			return -1;

	/* jump to next sibling tag if exist */
	if (xml->ordered != NULL)
		if (guf_xml_calc_space_for_tree(xml->ordered) < 0)
			return -1;

	return 0;
}

/*
 *  Frees all the memory allocated for building the sysfs tree.
 */
static void guf_xml_free_space_of_tree(void)
{
	if (tree_memory.kobj_name_begin) {
		vfree(tree_memory.kobj_name_begin);
		tree_memory.kobj_name_begin = NULL;
	}

	if (tree_memory.attr_group_begin) {
		vfree(tree_memory.attr_group_begin);
		tree_memory.attr_group_begin = NULL;
	}

	if (tree_memory.attr_group_attrs_begin) {
		vfree(tree_memory.attr_group_attrs_begin);
		tree_memory.attr_group_attrs_begin = NULL;
	}

	if (tree_memory.attr_name_begin) {
		vfree(tree_memory.attr_name_begin);
		tree_memory.attr_name_begin = NULL;
	}

	if (tree_memory.kattr_begin) {
		vfree(tree_memory.kattr_begin);
		tree_memory.kattr_begin = NULL;
	}
}

/*
 *  Allocates the space in memory for creating the sysfs tree.
 *  Returns 0 when finished successfully or an error code smaller than 0 on failure.
 */
static int guf_xml_allocate_space_for_tree(void)
{
	tree_memory.kobj_name_begin = vmalloc(tree_memory.kobj_name_bytes_cnt);
	tree_memory.attr_group_begin = vmalloc(tree_memory.attr_group_bytes_cnt);
	tree_memory.attr_group_attrs_begin = vmalloc(tree_memory.attr_group_attrs_bytes_cnt);
	tree_memory.attr_name_begin = vmalloc(tree_memory.attr_name_bytes_cnt);
	tree_memory.kattr_begin = vmalloc(tree_memory.kattr_bytes_cnt);

	if (	!tree_memory.kobj_name_begin
		 || !tree_memory.attr_group_begin
		 || !tree_memory.attr_group_attrs_begin
		 || !tree_memory.attr_name_begin
		 || !tree_memory.kattr_begin			) {

		printk(KERN_ERR "%s: failed to allocate memory (%s)\n", DRIVER_NAME, __func__);
		guf_xml_free_space_of_tree();
		return -1;
	}

	/* init memory */
	memset(tree_memory.kobj_name_begin, 0, tree_memory.kobj_name_bytes_cnt);
	memset(tree_memory.attr_group_begin, 0, tree_memory.attr_group_bytes_cnt);
	memset(tree_memory.attr_group_attrs_begin, 0, tree_memory.attr_group_attrs_bytes_cnt);
	memset(tree_memory.attr_name_begin, 0, tree_memory.attr_name_bytes_cnt);
	memset(tree_memory.kattr_begin, 0, tree_memory.kattr_bytes_cnt);

	tree_memory.kobj_name = tree_memory.kobj_name_begin;
	tree_memory.attr_group = tree_memory.attr_group_begin;
	tree_memory.attr_group_attrs = tree_memory.attr_group_attrs_begin;
	tree_memory.attr_name = tree_memory.attr_name_begin;
	tree_memory.kattr = tree_memory.kattr_begin;

	return 0;
}

/*
 *  Walks recursively through the xml data tree and creates the sysfs tree.
 *  Returns 0 when finished successfully or an error code smaller than 0 on failure.
 */
static int guf_xml_walk_xml_tree(struct ezxml *xml, struct kobject *parent_kobj, struct ezxml *xml_root)
{
	struct kobject *kobj = NULL;
	char **attr_name_or_value = NULL;
	char *cur_attr_name = NULL;
	char *cur_attr_value = NULL;
	int attr_cnt;

	if (!xml)
		return 0;

	/* iterate over all sibling tags */
	for (; xml != NULL; xml = xml->ordered) {

		if (parent_kobj == NULL)
		return -1;

	/* count attributes finding under the current tag */
	attr_cnt = guf_xml_count_attrs(xml);

	/* save the name of the current tag as the xml data will be freed
   	   after building the sysfs tree is completed */
		if (strlen(xml->name)+1 >= tree_memory.kobj_name_bytes_cnt) {
			printk(KERN_ERR "%s: tag-name too long (%s)\n", DRIVER_NAME, __func__);
			return -1;
		}
	strcpy(tree_memory.kobj_name, xml->name);

	/* connect the current attribute group with the memory allocated for the single attributes */
	tree_memory.attr_group->attrs = tree_memory.attr_group_attrs;

	if (DEBUG_MODE) printk(KERN_INFO "\n<%s> (%d)\n", xml->name, attr_cnt);

	/* go through the list of pointers pointing alternately on the attributes name
	   and the attributes value */
		if (xml->attr)
	for (attr_name_or_value = xml->attr; *attr_name_or_value != NULL; attr_name_or_value++) {

		/* attr_name_or_value points on a pointer pointing on the current attributes name */
		if (!cur_attr_name && !cur_attr_value) {
			cur_attr_name = *attr_name_or_value;
		}
		/* attr_name_or_value points on a pointer pointing on the current attributes value */
		else if (cur_attr_name && !cur_attr_value) {
			cur_attr_value = *attr_name_or_value;
		}

		if (cur_attr_name && cur_attr_value) {

			if (DEBUG_MODE) printk(KERN_INFO "  %s = %s\n", cur_attr_name, cur_attr_value);

			/* save the name of the current attribute as the xml data will be freed
			   after building the sysfs tree is completed */
					if (strlen(cur_attr_name)+1 >= tree_memory.attr_name_bytes_cnt) {
						printk(KERN_ERR "%s: attr_name too long (%s)\n", DRIVER_NAME, __func__);
						continue;
					}
			strcpy(tree_memory.attr_name, cur_attr_name);

			tree_memory.kattr->attr.name = tree_memory.attr_name;
			tree_memory.kattr->attr.mode = 0666;
			tree_memory.kattr->show = guf_xml_sysfs_attr_show;
			tree_memory.kattr->store = guf_xml_sysfs_attr_store;

			*(tree_memory.attr_group_attrs) = &tree_memory.kattr->attr;

			if (guf_xml_change_kobj_name(parent_kobj, tree_memory.kobj_name, xml->name, tree_memory.attr_name, cur_attr_value) < 0) {
				printk(KERN_ERR "%s: failed to change the name of the kernel object (%s)\n", DRIVER_NAME, __func__);
				return -1;
			}

			/* make the tree memory pointers show on the next fields in allocated memory */
			tree_memory.kattr++;
			tree_memory.attr_name += strlen(cur_attr_name) + 1;
			tree_memory.attr_group_attrs++;

			cur_attr_name = NULL;
			cur_attr_value = NULL;
		}
	}

	/* create kobject and export it to sysfs */
	kobj = kobject_create_and_add(tree_memory.kobj_name, parent_kobj);
	if (kobj == NULL) {
		printk(KERN_ERR "%s: the adress of the kernel object can not be null (%s)\n", DRIVER_NAME, __func__);
		return -1;
	}

	/* export attributes to sysfs */
	if (sysfs_create_group(kobj, tree_memory.attr_group) != 0) {
		printk(KERN_ERR "%s: failure on creating sysfs attributes (%s)\n", DRIVER_NAME, __func__);
		goto err_sysfs_create_group;
	}

	/* make the tree memory pointers show on the next fields in allocated memory */
	tree_memory.kobj_name += MAX_TAG_LENGTH;
	tree_memory.attr_group++;

	/* jump to next child tag if exist */
	if (xml->child != NULL)
			if (guf_xml_walk_xml_tree(xml->child, kobj, xml_root) < 0)
			goto err_cleanup;

	}

	return 0;

err_sysfs_create_group:
	kobject_del(kobj);
	return -1;

err_cleanup:
	sysfs_remove_group(kobj, tree_memory.attr_group - 1);
	kobject_del(kobj);
	return -1;
}

/*
 *  Creates a kernel object called ENTRY_TAG and exports it to
 *  /sys/class/misc/guf_xml/ENTRY_TAG.
 *  Gets a pointer on the root element of the xml tree and starts the recursive
 *  function which reads the xml tree and builds the sysfs tree.
 */
static void guf_xml_create_sysfs_tree(void)
{
	struct ezxml *xml_root = NULL;
	char* xml_string = NULL;

	kobj_root = kobject_create_and_add(ENTRY_TAG, &guf_xml_miscdev.this_device->kobj);
	if (kobj_root == NULL) {
		printk(KERN_ERR "%s: failure on creating kernel object and adding it to sysfs (%s)\n", DRIVER_NAME, __func__);
		return;
	}

	/* read the xml data from the partition */
	xml_string = guf_xml_read_partition_to_xml(mtd_active);
	if (xml_string == NULL) {
		printk(KERN_ERR "%s: failure on reading xml partition (%s)\n", DRIVER_NAME, __func__);
		goto err_free_kobj;
	}

	/* parse the xml data */
	xml_root = ezxml_parse_str(xml_string, strlen(xml_string));
	if (xml_root == NULL) {
		printk(KERN_ERR "%s: failure on parsing xml data (%s)\n", DRIVER_NAME, __func__);
		goto err_free_string;
	}

	memset(&tree_memory, 0, sizeof(struct guf_xml_tree_memory));	/* erase all tree memory data */
	/* in standard configuration file xml_root->child points on <flash> tag */
	if (guf_xml_calc_space_for_tree(xml_root->child) < 0) {
		printk(KERN_ERR "%s: failure on calculating the needed memory for the sysfs tree (%s)\n", DRIVER_NAME, __func__);
		goto err_allocate_tree;
	}

	if (DEBUG_MODE) {
		printk(KERN_INFO "%s: kobj_name_bytes_cnt: %d bytes\n", DRIVER_NAME, tree_memory.kobj_name_bytes_cnt);
		printk(KERN_INFO "%s: attr_group_bytes_cnt: %d bytes\n", DRIVER_NAME, tree_memory.attr_group_bytes_cnt);
		printk(KERN_INFO "%s: attr_group_attrs_bytes_cnt: %d bytes\n", DRIVER_NAME, tree_memory.attr_group_attrs_bytes_cnt);
		printk(KERN_INFO "%s: attr_name_bytes_cnt: %d bytes\n", DRIVER_NAME, tree_memory.attr_name_bytes_cnt);
		printk(KERN_INFO "%s: kattr_bytes_cnt: %d bytes\n", DRIVER_NAME, tree_memory.kattr_bytes_cnt);
	}

	if (guf_xml_allocate_space_for_tree() < 0) {
		printk(KERN_ERR "%s: failure on allocating memory for the sysfs tree (%s)\n", DRIVER_NAME, __func__);
		goto err_allocate_tree;
	}

	/* in standard configuration file xml_root->child points on <flash> tag */
	if (guf_xml_walk_xml_tree(xml_root->child, kobj_root, xml_root) < 0) {
		printk(KERN_ERR "%s: failure on exporting xml data to sysfs (%s)\n", DRIVER_NAME, __func__);
		goto err_walk_tree;
	}

	ezxml_free(xml_root);
	vfree(xml_string);
	return;

err_walk_tree:
	guf_xml_free_space_of_tree();
err_allocate_tree:
	ezxml_free(xml_root);
err_free_string:
	vfree(xml_string);
err_free_kobj:
	kobject_del(kobj_root);
	return;
}

/*
 *  Registers this driver as a misc device, extracts the information about the
 *  mtd devices from platform data and starts building the sysfs tree.
 *  Returns 0 on success or an error code smaller than 0 on failure.
 */
static int guf_xml_probe(struct platform_device *pdev)
{
	int err = 0;
	uint64_t tmp_fis_directory_size;
	uint64_t tmp_redundant_fis_size;
	int fis_directory_eb_cnt;
	int fis_directory_pg_cnt;
	int redundant_fis_eb_cnt;
	int redundant_fis_pg_cnt;

	mutex_init(&mutex);

	/* register driver as a misc device */
	err = misc_register(&guf_xml_miscdev);
	if (err) {
		printk(KERN_ERR "%s: failed to register misc device (%s)\n", DRIVER_NAME, __func__);
		return err;
	}

	/* extract information about the mtd devices from platform data
	   that was filled in board init file */
	mtd_fis_directory = ((struct guf_xml_device_platform_data *)(pdev->dev.platform_data))->mtd_fis_directory;
	mtd_redundant_fis = ((struct guf_xml_device_platform_data *)(pdev->dev.platform_data))->mtd_redundant_fis;
	if (!mtd_fis_directory || !mtd_redundant_fis) {
		printk(KERN_ERR "%s: failed to get mount information (%s)\n", DRIVER_NAME, __func__);
		goto err_deregister;
	}

	/* check as division by zero is not allowed */
	if (mtd_fis_directory->writesize == 0 || mtd_redundant_fis->writesize == 0) {
		printk(KERN_ERR "%s: invalid page size (%s)\n", DRIVER_NAME, __func__);
		goto err_deregister;
	}

	tmp_fis_directory_size = mtd_fis_directory->size;
	do_div(tmp_fis_directory_size, mtd_fis_directory->erasesize);
	fis_directory_eb_cnt = tmp_fis_directory_size;
	fis_directory_pg_cnt = mtd_fis_directory->erasesize / mtd_fis_directory->writesize;

	tmp_redundant_fis_size = mtd_redundant_fis->size;
	do_div(tmp_redundant_fis_size, mtd_redundant_fis->erasesize);
	redundant_fis_eb_cnt = tmp_redundant_fis_size;
	redundant_fis_pg_cnt = mtd_redundant_fis->erasesize / mtd_redundant_fis->writesize;

	if (DEBUG_MODE) {
		printk(KERN_INFO "%s: partition name: %s\n", DRIVER_NAME, mtd_fis_directory->name);
		printk(KERN_INFO "%s: partition size: %llu bytes\n", DRIVER_NAME, (unsigned long long)mtd_fis_directory->size);
		printk(KERN_INFO "%s: eraseblock size: %u bytes\n", DRIVER_NAME, mtd_fis_directory->erasesize);
		printk(KERN_INFO "%s: page size: %u bytes\n", DRIVER_NAME, mtd_fis_directory->writesize);
		printk(KERN_INFO "%s: number of eraseblocks: %u\n", DRIVER_NAME, fis_directory_eb_cnt);
		printk(KERN_INFO "%s: pages per eraseblock: %u\n", DRIVER_NAME, fis_directory_pg_cnt);

		printk(KERN_INFO "%s: partition name: %s\n", DRIVER_NAME, mtd_redundant_fis->name);
		printk(KERN_INFO "%s: partition size: %llu bytes\n", DRIVER_NAME, (unsigned long long)mtd_redundant_fis->size);
		printk(KERN_INFO "%s: eraseblock size: %u bytes\n", DRIVER_NAME, mtd_redundant_fis->erasesize);
		printk(KERN_INFO "%s: page size: %u bytes\n", DRIVER_NAME, mtd_redundant_fis->writesize);
		printk(KERN_INFO "%s: number of eraseblocks: %u\n", DRIVER_NAME, redundant_fis_eb_cnt);
		printk(KERN_INFO "%s: pages per eraseblock: %u\n", DRIVER_NAME, redundant_fis_pg_cnt);
	}

	/* check equality of the both mtd devices */
	if (	mtd_fis_directory->size != mtd_redundant_fis->size
		||	mtd_fis_directory->erasesize != mtd_redundant_fis->erasesize
		||	mtd_fis_directory->writesize != mtd_redundant_fis->writesize
		||	fis_directory_eb_cnt != redundant_fis_eb_cnt
		||	fis_directory_pg_cnt != redundant_fis_pg_cnt) {

		printk(KERN_ERR "%s: mtd devices are not equal (%s)\n", DRIVER_NAME, __func__);
		goto err_deregister;
	}

	eb_cnt = fis_directory_eb_cnt;
	pg_cnt = fis_directory_pg_cnt;

	/* determine which mtd device is more up to date */
	err = guf_xml_update_mtd_devices();
	if (err) {
		printk(KERN_ERR "%s: failed to update mtd devices (%s)\n", DRIVER_NAME, __func__);
		return err;
	}

	/* start creating the sysfs tree */
	guf_xml_create_sysfs_tree();

	printk(KERN_INFO "%s: driver loaded\n", DRIVER_NAME);
	return 0;

err_deregister:
	misc_deregister(&guf_xml_miscdev);
	return -EFAULT;
}

/*
 *  Unregisters this driver as a misc device.
 *  Unbuild sysfs tree?
 *  Returns 0 on success or an error code smaller than 0 on failure.
 */
static int guf_xml_remove(struct platform_device *pdev)
{
	int err = 0;

	mutex_destroy(&mutex);

	err = misc_deregister(&guf_xml_miscdev);
	if (err) {
		printk(KERN_ERR "%s: failed to deregister misc device (%s)\n", DRIVER_NAME, __func__);
		return err;
	}

	printk(KERN_INFO "%s: driver removed\n", DRIVER_NAME);
	return 0;
}

static struct platform_driver guf_xml_driver = {
	.probe		= guf_xml_probe,
	.remove		= guf_xml_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init guf_xml_init(void)
{
	return platform_driver_register(&guf_xml_driver);
}
module_init(guf_xml_init);

static void __exit guf_xml_exit(void)
{
	platform_driver_unregister(&guf_xml_driver);
}
module_exit(guf_xml_exit);

MODULE_DESCRIPTION("Driver for Garz & Fricke XML to sysfs conversion");
MODULE_AUTHOR("Phillip Durdaut <phillip.durdaut@garz-fricke.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
