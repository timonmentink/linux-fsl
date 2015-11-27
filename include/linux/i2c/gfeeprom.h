#ifndef _LINUX_GFEEPROM_H
#define _LINUX_GFEEPROM_H


#ifdef __KERNEL__
#include <linux/memory.h>
#endif /* __KERNEL__ */

#include <uapi/linux/i2c/gfeeprom.h>

#ifdef __KERNEL__
/*
 * As seen through Linux I2C, differences between the most common types of I2C
 * memory include:
 * - How much memory is available (usually specified in bit)?
 * - What write page size does it support?
 * - Special flags (16 bit addresses, read_only, world readable...)?
 *
 * If you set up a custom eeprom type, please double-check the parameters.
 * Especially page_size needs extra care, as you risk data loss if your value
 * is bigger than what the chip actually supports!
 */

struct gfeeprom_platform_data {
	u32		byte_len;		/* size (sum of all addr) */
	u16		page_size;		/* for writes */
	u8		flags;
	u8		bus_id;
#define AT24_FLAG_ADDR16	0x80	/* address pointer is 16 bit */
#define AT24_FLAG_READONLY	0x40	/* sysfs-entry will be read-only */
#define AT24_FLAG_IRUGO		0x20	/* sysfs-entry will be world-readable */
#define AT24_FLAG_TAKE8ADDR	0x10	/* take always 8 addresses (24c00) */

	void		(*setup)(struct memory_accessor*, void *context);
	void		*context;
};

#define GF_GLOBAL_PLATFORM_EEPROM_ADDRESS	0x50

struct guf_eeprom_memory_accessor
{
	struct memory_accessor mem_acc;
	void* cont;
};

#define GFEEPROM_MACC_MAJOR_VERSION_OFFS				  1
#define GFEEPROM_MACC_MINOR_VERSION_OFFS				  2
#define GFEEPROM_MACC_ARTICLE_NUMBER_OFFS				  3
#define GFEEPROM_MACC_SERIAL_NUMBER_OFFS				  4
#define GFEEPROM_MACC_COMPONENT_OFFS				  	  5
#define GFEEPROM_MACC_COMMENT_OFFS				  	  	  6
#define GFEEPROM_MACC_ADDRESS_OF_COMPONENT_OFFS	  	  	  7
#define GFEEPROM_MACC_PHYS_MEM_SIZE_OFFS	  	  	  	  8

bool gf_get_platform_hw_version(struct gfv_hw_version_data* hw_version);

#endif
#endif /* _LINUX_GFEEPROM_H */
