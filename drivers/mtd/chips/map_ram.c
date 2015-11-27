/*
 * Common code to handle map devices which are simple RAM
 * (C) 2000 Red Hat. GPL'd.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>


static int mapram_read (struct mtd_info *, loff_t, size_t, size_t *, u_char *);
static int mapram_write (struct mtd_info *, loff_t, size_t, size_t *, const u_char *);
static int mapram_erase (struct mtd_info *, struct erase_info *);
static void mapram_nop (struct mtd_info *);
static struct mtd_info *map_ram_probe(struct map_info *map);
static unsigned long mapram_unmapped_area(struct mtd_info *, unsigned long,
					  unsigned long, unsigned long);


static struct mtd_chip_driver mapram_chipdrv = {
	.probe	= map_ram_probe,
	.name	= "map_ram",
	.module	= THIS_MODULE
};

int map_ram_detect(struct map_info *map, struct mtd_info *mtd)
{
	unsigned char preserve[6], read;
	int i,j, len;
	const loff_t addr[] = { 0, 2, 0xAAAC, 0x5554, map->size-2, map->size-1 };
	const unsigned char pattern[2] = {0xAA, 0x55};
	BUG_ON( ARRAY_SIZE(preserve) < ARRAY_SIZE(addr));

	pr_debug("%s Cached: 0x%p\n", __func__, map->cached);
	pr_debug("%s Virt: 0x%p Phys: 0x%p\n", __func__, (void*)map->virt, (void*)map->phys);

	for(j = 0; j < ARRAY_SIZE(addr); j++)
	{
		mapram_read(mtd, addr[j], 1, &len, &(preserve[j]));
		pr_debug("%s %d: 0x%08x: 0x%x\n", __func__, __LINE__, (unsigned int) addr[j], (unsigned int) preserve[j]);
	}

	for(i = 0; i < ARRAY_SIZE(pattern); i++)
	{
		for(j = 0; j < ARRAY_SIZE(addr); j++)
		{
			mapram_write(mtd, addr[j], 1, &len, &(pattern[i]));
		}
		for(j = 0; j < ARRAY_SIZE(addr); j++)
		{
			mapram_read( mtd, addr[j], 1, &len, &read);
			pr_debug("%s %d: 0x%08x: Wrote 0x%x Read: 0x%x\n", __func__, __LINE__, (unsigned int) addr[j], pattern[i], read);
			if( read != pattern[i])
			{
				pr_err("%s %d: 0x%08x: Wrote 0x%x Read: 0x%x: No SRAM detected.\n", __func__, __LINE__, (unsigned int) addr[j], pattern[i], read);
				return -ENODEV;
			}
		}
	}
	for(j = 0; j < ARRAY_SIZE(addr); j++)
	{
		// Restore content
		mapram_write(mtd, addr[j], 1, &len, &(preserve[j]));
	}
	for(j = 0; j < ARRAY_SIZE(addr); j++)
	{
		mapram_read(mtd, addr[j], 1, &len, &(preserve[j]));
		pr_debug("%s %d: 0x%08x: 0x%x\n", __func__, __LINE__, (unsigned int) addr[j], (unsigned int) preserve[j]);
	}
	return 0;
}

static struct mtd_info *map_ram_probe(struct map_info *map)
{
	struct mtd_info *mtd;

#if 0

	mapram_write(map, 0x55, 0);
	if (map_read8(map, 0) != 0x55)
		return NULL;

	map_write8(map, 0xAA, 0);
	if (map_read8(map, 0) != 0xAA)
		return NULL;

	/* Check the last byte is RAM */
	map_write8(map, 0x55, map->size-1);
	if (map_read8(map, map->size-1) != 0x55)
		return NULL;

	map_write8(map, 0xAA, map->size-1);
	if (map_read8(map, map->size-1) != 0xAA)
		return NULL;
#endif
	/* OK. It seems to be RAM. */

	mtd = kzalloc(sizeof(*mtd), GFP_KERNEL);
	if (!mtd)
		return NULL;

	map->fldrv = &mapram_chipdrv;
	mtd->priv = map;
	mtd->name = map->name;
	mtd->type = MTD_RAM;
	mtd->size = map->size;
	mtd->_erase = mapram_erase;
	mtd->_get_unmapped_area = mapram_unmapped_area;
	mtd->_read = mapram_read;
	mtd->_write = mapram_write;
	mtd->_sync = mapram_nop;
	mtd->flags = MTD_CAP_RAM;
	mtd->writesize = 1;

	mtd->erasesize = PAGE_SIZE;
 	while(mtd->size & (mtd->erasesize - 1))
		mtd->erasesize >>= 1;

	/* Check the first byte is RAM */
	if(map_ram_detect(map, mtd))
		goto no_device;

	__module_get(THIS_MODULE);
	return mtd;

no_device:
	kfree(mtd);
	return NULL;
}


/*
 * Allow NOMMU mmap() to directly map the device (if not NULL)
 * - return the address to which the offset maps
 * - return -ENOSYS to indicate refusal to do the mapping
 */
static unsigned long mapram_unmapped_area(struct mtd_info *mtd,
					  unsigned long len,
					  unsigned long offset,
					  unsigned long flags)
{
	struct map_info *map = mtd->priv;
	return (unsigned long) map->virt + offset;
}

static int mapram_read (struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;

	map_copy_from(map, buf, from, len);
	*retlen = len;
	return 0;
}

static int mapram_write (struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	struct map_info *map = mtd->priv;

	map_copy_to(map, to, buf, len);
	*retlen = len;
	return 0;
}

static int mapram_erase (struct mtd_info *mtd, struct erase_info *instr)
{
	/* Yeah, it's inefficient. Who cares? It's faster than a _real_
	   flash erase. */
	struct map_info *map = mtd->priv;
	map_word allff;
	unsigned long i;

	allff = map_word_ff(map);
	for (i=0; i<instr->len; i += map_bankwidth(map))
		map_write(map, allff, instr->addr + i);
	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);
	return 0;
}

static void mapram_nop(struct mtd_info *mtd)
{
	/* Nothing to see here */
}

static int __init map_ram_init(void)
{
	register_mtd_chip_driver(&mapram_chipdrv);
	return 0;
}

static void __exit map_ram_exit(void)
{
	unregister_mtd_chip_driver(&mapram_chipdrv);
}

module_init(map_ram_init);
module_exit(map_ram_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("MTD chip driver for RAM chips");
