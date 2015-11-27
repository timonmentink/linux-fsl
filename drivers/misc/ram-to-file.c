/*
 * Copyright (C) 2015 Garz & Fricke GmbH. All Rights Reserved.
 *
 * RAM-to-file interface for the kernel command line. Provides given RAM
 * regions as files in the procfs filesystem.
 *
 * Syntax: ram-to-file=<address>,<length>,<name>
 *
 * Creates a virtual file /proc/<name> from which the RAM region at <address>
 * can be read.
 * The function "ram_to_file_reserve" has to be called from the board's
 * reserve-callback function in order to prevent the RAM regions being used and
 * eventually overwritten by the kernel.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <asm/io.h>

/*
 * Maximum number of RAM regions which can be provided in the procfs filesystem
 * This is a fixed number because we cannot allocate dynamic memory in the
 * early boot phase.
 */
#define MAX_FILES 10

/*
 * Buffer size for seq_file interface. Must be smaller than PAGE_SIZE.
 */
#define BUFSIZE	1024

struct ram_to_file_block
{
	struct list_head list;
	phys_addr_t phys;
	size_t length;
	char* name;
	char* buf;
};

struct ram_to_file_block ram_to_file_block_list[MAX_FILES];
struct ram_to_file_block *free_ram_to_file_block = ram_to_file_block_list;

bool ram_to_file_reserved = false;

/*
 * We have to use the seq_file interface for the procfs filesystem in order
 * to support file sizes larger than PAGE_SIZE.
 * See http://kernelnewbies.org/Documents/SeqFileHowTo for documentation.
 */
static void* ram_to_file_start(struct seq_file *s, loff_t *pos)
{
	struct ram_to_file_block *rtfb = (struct ram_to_file_block *)s->private;
	loff_t *spos;

	if ((*pos)*BUFSIZE >= rtfb->length)
		return NULL;

	spos = kmalloc(sizeof(loff_t), GFP_KERNEL);
	if (!spos)
		return NULL;

	*spos = (*pos)*BUFSIZE;
	return spos;
}

static void* ram_to_file_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct ram_to_file_block *rtfb = (struct ram_to_file_block *)s->private;
	loff_t *spos = v;

	*pos += 1;
	*spos += BUFSIZE;

	if (*spos >= rtfb->length)
		return NULL;

	return spos;
}

static void ram_to_file_stop(struct seq_file *s, void *v)
{
	kfree(v);
}

static int ram_to_file_show(struct seq_file *s, void *v)
{
	struct ram_to_file_block *rtfb = (struct ram_to_file_block *)s->private;
	loff_t *spos = v;
	int ret;
	size_t count = rtfb->length - *spos;
	if (count > BUFSIZE)
		count = BUFSIZE;
	ret = seq_write(s, rtfb->buf + *spos, count);
	return ret;
}

static struct seq_operations ram_to_file_seq_ops = {
	.start = ram_to_file_start,
	.next  = ram_to_file_next,
	.stop  = ram_to_file_stop,
	.show  = ram_to_file_show,
};

static int ram_to_file_open(struct inode *inode, struct file *file)
{
	int ret = seq_open(file, &ram_to_file_seq_ops);
	if (ret == 0) {
		struct seq_file *sq = file->private_data;
		sq->private = PDE_DATA(inode);
	}
	return ret;
}

static const struct file_operations ram_to_file_opts = {
	.owner = THIS_MODULE,
	.open = ram_to_file_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/**
 * ram_to_file_parse - parse kernel boot parameter "ram-to-file"
 * @options: string passed after parameter's "="
 *
 * This function parses a kernel boot parameter, which can be passed on the
 * kernel command line in the form "ram-to-file=<address>,<length>,<name>".
 * The parameter can occur multiple times. We cannot create the procfs files
 * immediately because this function is called very early in the boot process,
 * long before we can use vmalloc which is necessary for allocating the procfs
 * data structures. Thus, we store it in a list here for later initialization.
 */
static int __init ram_to_file_parse(char *options)
{
	struct ram_to_file_block *rtfb = free_ram_to_file_block;

	if (rtfb >= &ram_to_file_block_list[MAX_FILES])
		return 0;

	if (options == NULL || strlen(options) == 0)
		return 0;

	rtfb->phys = memparse(options, &options);
	if (*options != ',')
		return 0;

	options++;
	rtfb->length = memparse(options, &options);
	if (*options != ',')
		return 0;

	options++;
	rtfb->name = options;

	if (rtfb->phys <= 0 || rtfb->length <= 0 || rtfb->name == NULL || strlen(rtfb->name) == 0)
		return 0;

	free_ram_to_file_block++;
	return 0;
}
early_param("ram-to-file", ram_to_file_parse);

/**
 * ram_to_file_reserve - reserve parsed RAM regions
 *
 * This function must be called from the board's "reserve" callback function.
 * In order to avoid that the given RAM regions are used by the kernel we have
 * to exclude them from the kernel's memory management. The only point where
 * this is possible is the board's "reserve" callback function. If we do it
 * earlier, the memory is not even registered. If we do it later, the memory
 * will already be mapped.
 */
void ram_to_file_reserve(void)
{
	struct ram_to_file_block *rtfb = ram_to_file_block_list;

	while (rtfb < free_ram_to_file_block) {
		/*
		 * We have to align the size to 1 MB. I don't know where this comes
		 * from, determined it by try and error. If it is not aligned to this
		 * boundary, the kernel does not boot. Hopefully this works in all cases.
		 */
		memblock_remove(rtfb->phys, ALIGN(rtfb->length, SZ_1M));
		rtfb++;
	}
	ram_to_file_reserved = true;
}

/**
 * ram_to_file_init - create procfs files
 *
 * This function creates the procfs files for the parameters parsed by the
 * function "ram_to_file_parse".
 */
static int __init ram_to_file_init(void)
{
	struct ram_to_file_block *rtfb = ram_to_file_block_list;

	/* Check if RAM regions have been excluded from kernel memory space */
	if (!ram_to_file_reserved) {
		printk(KERN_ERR "ERROR: ram-to-file RAM regions have not been reserved, "
			"please call ram_to_file_reserve() from board's reserve-callback\n");
		return -EIO;
	}

	/* Create a procfs entry for each RAM-to-file block in the list */
	while (rtfb < free_ram_to_file_block) {
		if (request_mem_region(rtfb->phys, ALIGN(rtfb->length, SZ_1M), "ram-to-file") != NULL)
		{
			rtfb->buf = ioremap(rtfb->phys, ALIGN(rtfb->length, SZ_1M));
			if (rtfb->buf)
			{
				printk(KERN_INFO "ram-to-file: providing RAM region 0x%08x-0x%08x at /proc/%s\n",
					rtfb->phys, rtfb->phys + rtfb->length - 1, rtfb->name);
				proc_create_data(rtfb->name, 0, NULL, &ram_to_file_opts, rtfb);
			}
		}
		rtfb++;
	}

	return 0;
}
module_init(ram_to_file_init);
