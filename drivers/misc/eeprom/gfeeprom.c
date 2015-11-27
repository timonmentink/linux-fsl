/*
 * at24.c - handle most I2C EEPROMs
 *
 * Copyright (C) 2005-2007 David Brownell
 * Copyright (C) 2008 Wolfram Sang, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/sysfs.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/memory.h>
#include <linux/crc32.h>
#include <linux/i2c/gfeeprom.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>

#endif

/*
 * I2C EEPROMs from most vendors are inexpensive and mostly interchangeable.
 * Differences between different vendor product lines (like Atmel AT24C or
 * MicroChip 24LC, etc) won't much matter for typical read/write access.
 * There are also I2C RAM chips, likewise interchangeable. One example
 * would be the PCF8570, which acts like a 24c02 EEPROM (256 bytes).
 *
 * However, misconfiguration can lose data. "Set 16-bit memory address"
 * to a part with 8-bit addressing will overwrite data. Writing with too
 * big a page size also loses data. And it's not safe to assume that the
 * conventional addresses 0x50..0x57 only hold eeproms; a PCF8563 RTC
 * uses 0x51, for just one example.
 *
 * Accordingly, explicit board-specific configuration data should be used
 * in almost all cases. (One partial exception is an SMBus used to access
 * "SPD" data for DRAM sticks. Those only use 24c02 EEPROMs.)
 *
 * So this driver uses "new style" I2C driver binding, expecting to be
 * told what devices exist. That may be in arch/X/mach-Y/board-Z.c or
 * similar kernel-resident tables; or, configuration data coming from
 * a bootloader.
 *
 * Other than binding model, current differences from "eeprom" driver are
 * that this one handles write access and isn't restricted to 24c02 devices.
 * It also handles larger devices (32 kbit and up) with two-byte addresses,
 * which won't work on pure SMBus systems.
 */

struct gfeeprom_data {
	struct gfeeprom_platform_data chip;
	struct memory_accessor macc;
	bool use_smbus;
	u8	bus_id;
	bool guf_hw_info_present;
	struct gfv_hw_version_data hw_version;
	bool hw_version_stale;
	/*
	 * Lock protects against activities from other Linux tasks,
	 * but not from changes by other I2C masters.
	 */
	struct mutex lock;
	struct bin_attribute bin;

	bool writable;
	u8 *writebuf;
	unsigned write_max;
	unsigned num_addresses;

	/* files corresponding to this device */
	spinlock_t open_lock;
	struct gfeeprom_file_data* gfeeprom_fdata_alloc[MAX_GFEEPROM_CHIPS];
	struct cdev cdev;
	/*
	 * Some chips tie up multiple I2C addresses; dummy devices reserve
	 * them for us, and we'll use them with SMBus calls.
	 */
	struct i2c_client *client[];
};

/*
 * Per fd structure used to track the gfeeprom data allocated to that dev file.
 */
struct gfeeprom_file_data {
	struct gfeeprom_data* 	gfeeprom_hw; // pointer to hardware device info
	u32	    				cnt;		 // usage count, to allow for shared accesses
	u32	    				excl;		 // open wanted exclusive access to this gfeeprom
	u32						slot;		 // which gfeeprom driver is opened
};

struct gf_platform_hw_version {
	struct gfv_hw_version_data version_data;
	bool valid_data;
};

static struct gf_platform_hw_version global_platform_data = {
	.valid_data = false
};

static struct class *gfeeprom_class;
/*
 * This parameter is to help this driver avoid blocking other drivers out
 * of I2C for potentially troublesome amounts of time. With a 100 kHz I2C
 * clock, one 256 byte read takes about 1/43 second which is excessive;
 * but the 1/170 second it takes at 400 kHz may be quite reasonable; and
 * at 1 MHz (Fm+) a 1/430 second delay could easily be invisible.
 *
 * This value is forced to be a power of two so that writes align on pages.
 */
static unsigned io_limit = 128;
module_param(io_limit, uint, 0);
MODULE_PARM_DESC(io_limit, "Maximum bytes per I/O (default 128)");

/*
 * It may be that the underlying I2C driver fails in xfers due to NACK,
 * OVRE or anything else. The number of xfer retries can be set before
 * a connection timeout due to a defect is assumed. 
 */
static unsigned xfer_retries = 10;
module_param(xfer_retries, uint, 0);
MODULE_PARM_DESC(xfer_retries, "Retries for xfers (default 10)");

/*
 * Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned write_timeout = 25;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

#define AT24_SIZE_BYTELEN 5
#define AT24_SIZE_FLAGS 8

#define AT24_BITMASK(x) (BIT(x) - 1)

/* create non-zero magic value for given eeprom parameters */
#define AT24_DEVICE_MAGIC(_len, _flags) 		\
	((1 << AT24_SIZE_FLAGS | (_flags)) 		\
	    << AT24_SIZE_BYTELEN | ilog2(_len))

static const struct i2c_device_id gfeeprom_ids[] = {
	{ "gfeeprom", 0 },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, at24_ids);

/*-------------------------------------------------------------------------*/

/*
 * This routine supports chips which consume multiple I2C addresses. It
 * computes the addressing information to be used for a given r/w request.
 * Assumes that sanity checks for offset happened at sysfs-layer.
 */
static struct i2c_client *at24_translate_offset(struct gfeeprom_data *at24,
		unsigned *offset)
{
	unsigned i;

	if (at24->chip.flags & AT24_FLAG_ADDR16) {
		i = *offset >> 16;
		*offset &= 0xffff;
	} else {
		i = *offset >> 8;
		*offset &= 0xff;
	}

	return at24->client[i];
}

static ssize_t at24_eeprom_read(struct gfeeprom_data *at24, char *buf,
		unsigned offset, size_t count)
{
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	struct i2c_client *client;
	int status, i;
	unsigned int retries = xfer_retries;

	memset(msg, 0, sizeof(msg));

	/*
	 * REVISIT some multi-address chips don't rollover page reads to
	 * the next slave address, so we may need to truncate the count.
	 * Those chips might need another quirk flag.
	 *
	 * If the real hardware used four adjacent 24c02 chips and that
	 * were misconfigured as one 24c08, that would be a similar effect:
	 * one "eeprom" file not four, but larger reads would fail when
	 * they crossed certain pages.
	 */

	/*
	 * Slave address and byte offset derive from the offset. Always
	 * set the byte address; on a multi-master board, another master
	 * may have changed the chip's "current" address pointer.
	 */
	client = at24_translate_offset(at24, &offset);

	if (count > io_limit)
		count = io_limit;

	if (at24->use_smbus) {
		/* Smaller eeproms can work given some SMBus extension calls */
		if (count > I2C_SMBUS_BLOCK_MAX)
			count = I2C_SMBUS_BLOCK_MAX;
	} else {
		/*
		 * When we have a better choice than SMBus calls, use a
		 * combined I2C message. Write address; then read up to
		 * io_limit data bytes. Note that read page rollover helps us
		 * here (unlike writes). msgbuf is u8 and will cast to our
		 * needs.
		 */
		i = 0;
		if (at24->chip.flags & AT24_FLAG_ADDR16)
			msgbuf[i++] = offset >> 8;
		msgbuf[i++] = offset;

		msg[0].addr = client->addr;
		msg[0].buf = msgbuf;
		msg[0].len = i;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = count;
	}

	/*
	 * Reads fail if the previous write didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	do {
		if (at24->use_smbus) {
			status = i2c_smbus_read_i2c_block_data(client, offset,
					count, buf);
		} else {
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
				status = count;
		}
		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);

		if (status == count) {
			return count;
		}
		
		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
		retries--;
	} while(retries);
	return -ETIMEDOUT;
}

static ssize_t at24_read(struct gfeeprom_data *at24,
		char *buf, loff_t off, size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Read data from chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&at24->lock);

	while (count) {
		ssize_t	status;

		status = at24_eeprom_read(at24, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&at24->lock);

	return retval;
}

/*
static ssize_t at24_bin_read(struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct gfeeprom_data *at24;

	at24 = dev_get_drvdata(container_of(kobj, struct device, kobj));
	return at24_read(at24, buf, off, count);
}
*/

/*
 * Note that if the hardware write-protect pin is pulled high, the whole
 * chip is normally write protected. But there are plenty of product
 * variants here, including OTP fuses and partial chip protect.
 *
 * We only use page mode writes; the alternative is sloooow. This routine
 * writes at most one page.
 */
static ssize_t at24_eeprom_write(struct gfeeprom_data *at24, const char *buf,
		unsigned offset, size_t count)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	ssize_t status;
	unsigned next_page;
	unsigned int retries = xfer_retries;
	

	/* Get corresponding I2C address and adjust offset */
	client = at24_translate_offset(at24, &offset);

	/* write_max is at most a page */
	if (count > at24->write_max)
		count = at24->write_max;

	/* Never roll over backwards, to the start of this page */
	next_page = roundup(offset + 1, at24->chip.page_size);
	if (offset + count > next_page)
		count = next_page - offset;

	/* If we'll use I2C calls for I/O, set up the message */
	if (!at24->use_smbus) {
		int i = 0;

		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = at24->writebuf;
		if (at24->chip.flags & AT24_FLAG_ADDR16)
			msg.buf[i++] = offset >> 8;

		msg.buf[i++] = offset;
		memcpy(&msg.buf[i], buf, count);
		msg.len = i + count;
	}

	/*
	 * Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	do {
		if (at24->use_smbus) {
			status = i2c_smbus_write_i2c_block_data(client,
					offset, count, buf);
			if (status == 0)
				status = count;
		} else {
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1)
				status = count;
		}
		//dev_info(&client->dev, "V: write %zu@%d --> %zd (%ld)\n", count, offset, status, jiffies);

		if (status == count) {
 			return count;
		}

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(write_timeout);
		retries--;
	} while (retries);

	return -ETIMEDOUT;
}

static ssize_t at24_write(struct gfeeprom_data *at24, const char *buf, loff_t off,
			  size_t count)
{
	ssize_t retval = 0;
	//struct i2c_client* 		client = at24_translate_offset(at24, (unsigned*)&off);

	if (unlikely(!count))
		return count;

	/*
	 * Write data to chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&at24->lock);

	while (count) {
		ssize_t	status;

		//dev_info(&(client->dev), "at24: 0x%X, buf: 0x%X, off: 0x%X, count: 0x%X\r\n", (unsigned int)at24, (unsigned int)buf, (unsigned int)off, count);

		status = at24_eeprom_write(at24, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&at24->lock);

	return retval;
}

/*
static ssize_t at24_bin_write(struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct gfeeprom_data *at24;

	at24 = dev_get_drvdata(container_of(kobj, struct device, kobj));
	return at24_write(at24, buf, off, count);
}
*/

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define ROM_HEAD_OFFSET		sizeof(u16) + sizeof(u8)
#define ROMSIZE_M24C		((1024 >> 3) - ROM_HEAD_OFFSET)
#define M24C_PAGE_SIZE		0x10
#define ROMSIZE_AT24C64C	(((1 << 16) >> 3) - ROM_HEAD_OFFSET)
#define AT24_PAGE_SIZE		0x20
#define EEPROM_MAX_PAGE_SIZE AT24_PAGE_SIZE
#define DUMMY_DATA			0xCCAA
#define EEPROM_TYPE_OFFSET	0x80

#define MAX_DEV_COUNT		0x07

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------
#define GET_ROM_SIZE(gfeeprom) (gfeeprom->chip.byte_len)
#define GET_PAGE_SIZE(gfeeprom) (gfeeprom->chip.page_size)
#define GET_ADDR_WIDTH(gfeeprom) ((gfeeprom->chip.flags & AT24_FLAG_ADDR16) ? EEPROM_ADDRSS_WIDTH_16BIT : EEPROM_ADDRSS_WIDTH_8BIT)
#define INVERT_ORDER(address) ((u16)(((u16)address) << 8 | ((u16)address) >> 8))
#define CREATE_EEPROM_TYPE_HEAD(eepromType) ((u16)((eepromType + EEPROM_TYPE_OFFSET) << 8 | (eepromType + EEPROM_TYPE_OFFSET)))

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Extern Variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Local Variables
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Local Functions
//------------------------------------------------------------------------------
static u32 wchar_to_char(char* szInput, u32 uiISize, char* szOutput, u32 uiOSize)
{
	u32 uiCharactersToCopy = min(uiISize, uiOSize);
	u32 uiCounter;

	if (!szInput || !szOutput)
		return ERROR_BAD_ARGUMENTS;

	for (uiCounter = 0; (uiCounter < uiCharactersToCopy) && (szInput[uiCounter] != L'\0'); uiCounter++)
		szOutput[uiCounter] = szInput[uiCounter];

	if (uiCounter == uiCharactersToCopy)
		szOutput[uiCounter-1] = '\0';
	else
		szOutput[uiCounter] = '\0';

	return ERROR_SUCCESS;
}

static u32 char_to_wchar(char* szInput, u32 uiISize, char* szOutput, u32 uiOSize)
{
	u32 uiCounter;
	u32 uiCharactersToCopy = min(uiISize, uiOSize);

	if (!szInput || !szOutput)
		return ERROR_BAD_ARGUMENTS;

	for (uiCounter = 0; (uiCounter < uiCharactersToCopy) && (szInput[uiCounter] != '\0'); uiCounter++)
		szOutput[uiCounter] = szInput[uiCounter];

	if (uiCounter == uiCharactersToCopy)
		szOutput[uiCounter-1] = '\0';
	else
		szOutput[uiCounter] = '\0';

	return ERROR_SUCCESS;
}


static u32 gfeeprom_gfhw_to_ehw_small(struct gfv_hw_version_data* lpInput, struct gfeeprom_hw_small_version_data* lpOutput)
{
	u32 dwReturn;

	if (!lpInput || !lpOutput)
		return ERROR_BAD_ARGUMENTS;

	lpOutput->size_of_struct = sizeof(struct gfeeprom_hw_small_version_data);
	dwReturn = wchar_to_char((char*)lpInput->major, GFV_NUMBER_LENGTH, (char*)lpOutput->major, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->minor, GFV_NUMBER_LENGTH, (char*)lpOutput->minor, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->article_number, GFV_ARTICLE_NUMBER_LENGTH, (char*)lpOutput->article_number, GFEEPROM_HW_SMALL_ARTICLE_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->component, GFV_COMPONENT_LENGTH, (char*)lpOutput->component, GFV_COMPONENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->comment, GFV_COMMENT_LENGTH, (char*)lpOutput->comment, GFV_COMPONENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	lpOutput->address_of_component = lpInput->address_of_component;
	lpOutput->phys_mem_size = lpInput->phys_mem_size;

	return ERROR_SUCCESS;
}

static u32 gfeeprom_small_ehw_to_gfhw(struct gfeeprom_hw_small_version_data* lpInput, struct gfv_hw_version_data* lpOutput)
{
	u32 dwReturn;

	if (!lpInput || !lpOutput)
		return ERROR_BAD_ARGUMENTS;

	lpOutput->size_of_struct = sizeof(struct gfv_hw_version_data);
	dwReturn = char_to_wchar((char*)lpInput->major, GFV_NUMBER_LENGTH, (char*)lpOutput->major, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->minor, GFV_NUMBER_LENGTH, (char*)lpOutput->minor, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->article_number, GFEEPROM_HW_SMALL_ARTICLE_NUMBER_LENGTH, (char*)lpOutput->article_number, GFV_ARTICLE_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	lpOutput->serial_numbers[0] = '\0';
	dwReturn = char_to_wchar((char*)lpInput->component, GFV_COMPONENT_LENGTH, (char*)lpOutput->component, GFV_COMPONENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->comment, GFV_COMPONENT_LENGTH, (char*)lpOutput->comment, GFV_COMMENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	lpOutput->address_of_component = lpInput->address_of_component;
	lpOutput->phys_mem_size = lpInput->phys_mem_size;
	return ERROR_SUCCESS;
}

static u32 gfeeprom_gfhw_to_ehw_normal(struct gfv_hw_version_data* lpInput, struct gfeeprom_hw_normal_version_data* lpOutput)
{
	u32 dwReturn;

	if (!lpInput || !lpOutput)
		return ERROR_BAD_ARGUMENTS;

	lpOutput->size_of_struct = sizeof(struct gfeeprom_hw_normal_version_data);
	dwReturn = wchar_to_char((char*)lpInput->major, GFV_NUMBER_LENGTH, (char*)lpOutput->major, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->minor, GFV_NUMBER_LENGTH, (char*)lpOutput->minor, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->article_number, GFV_ARTICLE_NUMBER_LENGTH, (char*)lpOutput->article_number, GFV_ARTICLE_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->serial_numbers, GFV_SERIAL_NUMBER_LENGTH, (char*)lpOutput->serial_numbers, GFV_SERIAL_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->component, GFV_COMPONENT_LENGTH, (char*)lpOutput->component, GFV_COMPONENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = wchar_to_char((char*)lpInput->comment, GFV_COMMENT_LENGTH, (char*)lpOutput->comment, GFV_COMMENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	lpOutput->address_of_component = lpInput->address_of_component;
	lpOutput->phys_mem_size = lpInput->phys_mem_size;
	return ERROR_SUCCESS;
}

static u32 gfeeprom_normal_ehw_to_gfhw(struct gfeeprom_hw_normal_version_data* lpInput, struct gfv_hw_version_data* lpOutput)
{
	u32 dwReturn;

	if (!lpInput || !lpOutput)
		return ERROR_BAD_ARGUMENTS;

	lpOutput->size_of_struct = sizeof(struct gfv_hw_version_data);
	dwReturn = char_to_wchar((char*)lpInput->major, GFV_NUMBER_LENGTH, (char*)lpOutput->major, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->minor, GFV_NUMBER_LENGTH, (char*)lpOutput->minor, GFV_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->article_number, GFV_ARTICLE_NUMBER_LENGTH, (char*)lpOutput->article_number, GFV_ARTICLE_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->serial_numbers, GFV_SERIAL_NUMBER_LENGTH, (char*)lpOutput->serial_numbers, GFV_SERIAL_NUMBER_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->component, GFV_COMPONENT_LENGTH, (char*)lpOutput->component, GFV_COMPONENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	dwReturn = char_to_wchar((char*)lpInput->comment, GFV_COMMENT_LENGTH, (char*)lpOutput->comment, GFV_COMMENT_LENGTH);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;
	lpOutput->address_of_component = lpInput->address_of_component;
	lpOutput->phys_mem_size = lpInput->phys_mem_size;

	return ERROR_SUCCESS;
}

static void gfeeprom_dump_gfhw(struct gfeeprom_data *gfeeprom, struct gfv_hw_version_data* lpGFHW)
{
	dev_dbg(&(gfeeprom->client[0]->dev), "dwSize: 0x%X\r\n", lpGFHW->size_of_struct);
	dev_dbg(&(gfeeprom->client[0]->dev), "major: %s\r\n", lpGFHW->major);
	dev_dbg(&(gfeeprom->client[0]->dev), "minor: %s\r\n", lpGFHW->minor);
	dev_dbg(&(gfeeprom->client[0]->dev), "article_number: %s\r\n", lpGFHW->article_number);
	dev_dbg(&(gfeeprom->client[0]->dev), "component: %s\r\n", lpGFHW->component);
	dev_dbg(&(gfeeprom->client[0]->dev), "comment: %s\r\n", lpGFHW->comment);
	dev_dbg(&(gfeeprom->client[0]->dev), "address_of_component: 0x%x\r\n", lpGFHW->address_of_component);
	dev_dbg(&(gfeeprom->client[0]->dev), "phys_mem_size: 0x%x\r\n", lpGFHW->phys_mem_size);
}

/*
static u32 EEPROM_ReadyTest(struct gfeeprom_data *gfeeprom)
{
	u32 		dwReturn;
	u8  		ucDummyByte;

	do
	{
		dwReturn = at24_read(gfeeprom, &ucDummyByte, 0, 1);
		uiCounter++;
	} while ((1 != dwReturn) && (uiCounter < READ_REPEATS_AFTER_WRITE));

	return dwReturn;
}
*/

static u32 gfeeprom_read_eeprom_head(struct gfeeprom_data *gfeeprom, u32 uiEEPROMPos)
{
	u32 		dwReturn;
	u16  		usEEPROMHead;
	struct i2c_client* client;

	dwReturn = at24_read(gfeeprom, (u8*)&usEEPROMHead, uiEEPROMPos, sizeof(u16));
 	client = at24_translate_offset(gfeeprom, &uiEEPROMPos);

	if (GFEEPROM_VALIDAT_ID(gfeeprom->bus_id, client->addr) != usEEPROMHead)
	{
		dev_dbg(&(client->dev), "ERROR: gfeeprom_read_eeprom_head: No valid EEPROM head at address 0x%02X on controller %d deteced.\r\n", client->addr << 1, gfeeprom->bus_id);
		return ERROR_NO_VOLUME_LABEL;
	}
	return ERROR_SUCCESS;
}

static u32 gfeeprom_write_eeprom_head(struct gfeeprom_data *gfeeprom, u32 uiEEPROMPos)
{
	u32		dwReturn;
	u16		usEEPROMHead;
	struct i2c_client* client = at24_translate_offset(gfeeprom, &uiEEPROMPos);

	usEEPROMHead = (u16)GFEEPROM_VALIDAT_ID(gfeeprom->bus_id, client->addr);
	dwReturn = at24_write(gfeeprom, (u8*)&usEEPROMHead, uiEEPROMPos, sizeof(u16));

/*
	dwReturn = EEPROM_ReadyTest(gfeeprom);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_err("ERROR: gfeeprom_write_eeprom_head: Code = 0x%x\r\n", dwReturn);
		return dwReturn;
	}
*/
	return ERROR_SUCCESS;
}

static u32 gfeeprom_write_page_mode(struct gfeeprom_data *gfeeprom, struct gfeeprom_area_head* lpEHead, u8* lpData, u32 uiSize, u32 uiOffset)
{
	u32						dwReturn;
	u8*						lpSource = NULL;
	bool					mem_allocated = false;
	struct i2c_client* 		client = at24_translate_offset(gfeeprom, &uiOffset);

	if (uiSize == 0)
		return ERROR_BAD_ARGUMENTS;

	//dev_info(&(client->dev), "gfeeprom_write_page_mode bus: 0x%X, address: 0x%X, offset: 0x%X, lpData: 0x%X, remaining bytes: %d\r\n", gfeeprom->bus_id, client->addr << 1, uiOffset, (unsigned int)lpData, uiSize);

	if ((uiOffset + uiSize + (lpEHead ? sizeof(struct gfeeprom_area_head) : 0)) > GET_ROM_SIZE(gfeeprom))
		return ERROR_BAD_ARGUMENTS;

	if (lpEHead)
	{
		//dev_info(&(client->dev), "lpEHead: 0x%X, uiOffset: 0x%X, size: 0x%X\r\n", (unsigned int)lpEHead, uiOffset, sizeof(struct gfeeprom_area_head));
		dwReturn = at24_write(gfeeprom, (u8*)lpEHead, uiOffset, sizeof(struct gfeeprom_area_head));
		if (sizeof(struct gfeeprom_area_head) != dwReturn)
		{
			dwReturn = ERROR_I2C_FAILURE;
			dev_err(&(client->dev), "ERROR: gfeeprom_write_page_mode(1): Code = 0x%x\r\n", dwReturn);
			return dwReturn;
		}
		uiOffset += sizeof(struct gfeeprom_area_head);
	}

	if (!lpData)
	{
		u32 i;

		lpSource = (u8*)kzalloc(uiSize, GFP_KERNEL);
		if (!lpSource)
		{
			dwReturn = ERROR_NOT_ENOUGH_MEMORY;
			dev_err(&(client->dev), "ERROR: gfeeprom_write_page_mode(2): Code = 0x%x\r\n", dwReturn);
			return dwReturn;
		}
		mem_allocated = true;
		for (i = 0; i < uiSize; i++)
			lpSource[i] = 0xFF;
	}
	else
	{
		lpSource = lpData;
	}

	dwReturn = at24_write(gfeeprom, lpSource, uiOffset, uiSize);
	if (uiSize != dwReturn)
	{
		dwReturn = ERROR_I2C_FAILURE;
		dev_err(&(client->dev), "ERROR: gfeeprom_write_page_mode(3): Code = 0x%x\r\n", dwReturn);
	}
	else
	{
		dwReturn = ERROR_SUCCESS;
	}

	if (mem_allocated)
		kfree(lpSource);
	return dwReturn;
}

static u32 gfeeprom_update_area_crc32(struct gfeeprom_data *gfeeprom, struct gfeeprom_area_head* lpEHead, u8* lpData, u32 uiSize, u32 uiOffset)
{
	u32							dwCRC32;
	u32							dwReturn;
	u32 						ulOffset;
	struct i2c_client* 			client = at24_translate_offset(gfeeprom, &uiOffset);

	if (!lpData || (uiSize == 0))
		return ERROR_BAD_ARGUMENTS;

	dev_dbg(&(client->dev), "gfeeprom_update_area_crc32\r\n");

	ulOffset = uiOffset + (((u32)&(lpEHead->crc32)) - (u32)lpEHead);
	dwCRC32 = crc32(0, lpData, uiSize);
	dwReturn = gfeeprom_write_page_mode(gfeeprom, NULL, (u8*)&dwCRC32, sizeof(u32), ulOffset);
	if (ERROR_SUCCESS != dwReturn)
		dev_warn(&(gfeeprom->client[0]->dev), "WARNING: gfeeprom_update_area_crc32: Code = 0x%x\r\n", dwReturn);
	return dwReturn;
}

static u32 gfeeprom_update_area_size(struct gfeeprom_data *gfeeprom, struct gfeeprom_area_head* lpEHead, u16 usSize, u32 uiOffset)
{
	u32				dwReturn;
	u32 			ulOffset;

	if (usSize == 0)
		return ERROR_BAD_ARGUMENTS;

	ulOffset = uiOffset + (((u32)&(lpEHead->length)) - (u32)lpEHead);
	dwReturn = gfeeprom_write_page_mode(gfeeprom, NULL, (u8*)&usSize, sizeof(u16), ulOffset);
	if (ERROR_SUCCESS != dwReturn)
		dev_warn(&(gfeeprom->client[0]->dev), "WARNING: gfeeprom_update_area_size: Code = 0x%x\r\n", dwReturn);
	return dwReturn;
}

//------------------------------------------------------------------------------
// external functions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static u32 gfeeprom_read_eeprom_area(struct gfeeprom_data *gfeeprom, enum gfeeprom_area_type areaType, u8* lpData, u16 usSize, u32* lpOffset, u16* area_size)
{
	u32 							dwReturn;
	struct gfeeprom_area_head		areaHead = {0,0,0};
	bool							bFound = false;
	u32								uiEEPROMPosition = sizeof(struct gfeeprom_head);

	dwReturn = gfeeprom_read_eeprom_head(gfeeprom, 0);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;

	for (; (uiEEPROMPosition < GET_ROM_SIZE(gfeeprom)) && !bFound;)
	{
		dwReturn = at24_read(gfeeprom, (u8*)&areaHead, uiEEPROMPosition, sizeof(struct gfeeprom_area_head));
		if (sizeof(struct gfeeprom_area_head) != dwReturn)
		{
			dwReturn = ERROR_EEPROM_HEAD_READ_FAILURE;
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(1): Code = 0x%x\r\n", dwReturn);
			return dwReturn;
		}

		uiEEPROMPosition += sizeof(struct gfeeprom_area_head);
		if (lpOffset)
			*lpOffset = uiEEPROMPosition + areaHead.length;

		if ((areaHead.type == areaType) && ((usSize < areaHead.length) || !lpData))
		{
			dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(2): The date size is to small to handle the EEPROM structure.\r\n");
			if (usSize >= sizeof(u16))
				*(u16*)lpData = areaHead.length;
			return ERROR_INSUFFICIENT_BUFFER;
		}

		if ((areaHead.type == areaType) && ((areaHead.length + uiEEPROMPosition) <= GET_ROM_SIZE(gfeeprom)))
		{
			dwReturn = at24_read(gfeeprom, lpData, uiEEPROMPosition, areaHead.length);
			if (areaHead.length != dwReturn)
			{
				dwReturn = ERROR_EEPROM_HEAD_READ_FAILURE;
				dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(2): Code = 0x%x\r\n", dwReturn);
				return dwReturn;
			}
			if (area_size)
				*area_size = areaHead.length;

			if (EEPROM_TYPE_EHW == areaHead.type)
			{
				u16 									usSizeBuffer;
				struct gfeeprom_hw_small_version_data 	areaData;
				struct i2c_client* 						client = at24_translate_offset(gfeeprom, &uiEEPROMPosition);

				memcpy(&areaData, lpData, sizeof(struct gfeeprom_hw_small_version_data));

				dev_dbg(&(gfeeprom->client[0]->dev), "gfeeprom_read_eeprom_area: Detected EEPROM HW structure on controller %d with eeprom address: 0x%02X.\r\n", gfeeprom->bus_id, client->addr << 1);
				if (ROMSIZE_M24C < GET_ROM_SIZE(gfeeprom))
					usSizeBuffer = sizeof(struct gfeeprom_hw_normal_version_data);
				else
					usSizeBuffer = sizeof(struct gfeeprom_hw_small_version_data);
				if (areaData.size_of_struct != usSizeBuffer)
				{
					dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(3): The EEPROM HW size isn't matching (0x%X <-> 0x%X). An old EEPROM structure detected, which isn't supported.\r\n",
								areaData.size_of_struct,
								usSizeBuffer);
					return ERROR_OLD_VERSION;
				}
				if (areaData.size_of_struct != areaHead.length)
				{
					dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(4): The date size of the list is different than the EEPROM HW reserved space.\r\n");
					return ERROR_LIST_INCONSISTENT;
				}
			}
			bFound = true;
		}
		else if (areaHead.type == areaType)
		{
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(5): Skipping type 0x%X because the size of 0x%X doesn't fit into EEPROM.\r\n", areaType, usSize);
			return ERROR_LIST_INCONSISTENT;
		}
		uiEEPROMPosition += areaHead.length;
	}

	if (!bFound)
	{
		dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(6): Mod %i not found\r\n", areaType);
		return ERROR_MOD_NOT_FOUND;
	}
	if (areaHead.crc32 != crc32(0, lpData, areaHead.length))
	{
		dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area(7): CRC32 checksum failure at mod %i. (0x%X <-> 0x%X)\r\n", areaType, areaHead.crc32, crc32(0, lpData, areaHead.length));
		return ERROR_CRC;
	}
	return ERROR_SUCCESS;
}

static u32 gfeeprom_get_used_eeprom_areas(struct gfeeprom_data *gfeeprom, u16* areas, u16 erea_count)
{
	u32 							dwReturn;
	struct gfeeprom_area_head		areaHead = {0,0,0};
	u32								uiEEPROMPosition = sizeof(struct gfeeprom_head);
	u8* 							lpData = NULL;
	u16								counter = 0;

	if (!areas)
		return ERROR_BAD_ARGUMENTS;
	*areas = 0;

	dwReturn = gfeeprom_read_eeprom_head(gfeeprom, 0);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;

	for (; uiEEPROMPosition < GET_ROM_SIZE(gfeeprom);)
	{
		dwReturn = at24_read(gfeeprom, (u8*)&areaHead, uiEEPROMPosition, sizeof(struct gfeeprom_area_head));
		if (sizeof(struct gfeeprom_area_head) != dwReturn)
		{
			dwReturn = ERROR_EEPROM_HEAD_READ_FAILURE;
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_count_eeprom_areas(1): Code = 0x%x\r\n", dwReturn);
			return dwReturn;
		}

		uiEEPROMPosition += sizeof(struct gfeeprom_area_head);
		if ((areaHead.length + uiEEPROMPosition) > GET_ROM_SIZE(gfeeprom))
		{
			dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_count_eeprom_areas(5): Skipping type 0x%X because the size of 0x%X doesn't fit into EEPROM.\r\n", areaHead.type, areaHead.length + uiEEPROMPosition);
			return ERROR_LIST_INCONSISTENT;
		}
		if (EEPROM_TYPE_EHW == areaHead.type)
		{
			uiEEPROMPosition += areaHead.length;
			continue;
		}
		lpData = kzalloc(areaHead.length, GFP_KERNEL);
		if (!lpData)
			return ERROR_NOT_ENOUGH_MEMORY;

		dwReturn = at24_read(gfeeprom, lpData, uiEEPROMPosition, areaHead.length);
		if (areaHead.length != dwReturn)
		{
			dwReturn = ERROR_EEPROM_HEAD_READ_FAILURE;
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_count_eeprom_areas(2): Code = 0x%x\r\n", dwReturn);
			kfree(lpData);
			return dwReturn;
		}
		if (areaHead.crc32 != crc32(0, lpData, areaHead.length))
		{
			dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_count_eeprom_areas(7): CRC32 checksum failure at mod %i. (0x%X <-> 0x%X)\r\n", areaHead.type, areaHead.crc32, crc32(0, lpData, areaHead.length));
			kfree(lpData);
			return ERROR_CRC;
		}
		if (0 == erea_count)
			*areas = *areas+1;
		else
		{
			if (counter < erea_count)
				areas[counter++] = areaHead.type;
			else
			{
				kfree(lpData);
				return ERROR_INSUFFICIENT_BUFFER;
			}
		}
		kfree(lpData);
		uiEEPROMPosition += areaHead.length;
	}

	return ERROR_SUCCESS;
}

static u32 gfeeprom_count_eeprom_areas(struct gfeeprom_data *gfeeprom, u16* areas)
{
	return gfeeprom_get_used_eeprom_areas(gfeeprom, areas, 0);
}

static u32 gfeeprom_write_eeprom_area(struct gfeeprom_data *gfeeprom, enum gfeeprom_area_type areaType, u8* lpData, u16 usSize, bool bForceWrite)
{
	u32 						dwReturn;
	struct gfeeprom_area_head	areaHead = {0,0,0};
	bool						bFound = false;
	u32							uiEEPROMPosition = sizeof(struct gfeeprom_head);
	u32							uiStartPosition = 0;
	struct i2c_client* 			client = at24_translate_offset(gfeeprom, &uiStartPosition);

	if (!lpData || (usSize == 0))
		return ERROR_BAD_ARGUMENTS;

	dwReturn = gfeeprom_read_eeprom_head(gfeeprom, 0);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_dbg(&(client->dev), "gfeeprom_write_eeprom_area: No EEPROM header found on controller %d address 0x%X, initialising eeprom... ", gfeeprom->bus_id, client->addr << 1);
		dwReturn = gfeeprom_write_eeprom_head(gfeeprom, 0);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_dbg(&(client->dev), " failed.\r\n");
			return dwReturn;
		}
		uiEEPROMPosition = sizeof(struct gfeeprom_head) + sizeof(struct gfeeprom_area_head);
		dev_dbg(&(client->dev), " done.\r\n");
	}
	else
	{
		while(!bFound)
		{
			dwReturn = at24_read(gfeeprom, (u8*)&areaHead, uiEEPROMPosition, sizeof(struct gfeeprom_area_head));
			if (sizeof(struct gfeeprom_area_head) != dwReturn)
			{
				dwReturn = ERROR_EEPROM_HEAD_READ_FAILURE;
				dev_err(&(client->dev), "ERROR: gfeeprom_write_eeprom_area(1): Code = 0x%x\r\n", dwReturn);
				return dwReturn;
			}
			uiEEPROMPosition += sizeof(struct gfeeprom_area_head);

			if (areaHead.type == areaType)
				bFound = true;
			else if (EEPROM_UNUSED_AREA == areaHead.type)
				break;
			else if ((uiEEPROMPosition + areaHead.length) < GET_ROM_SIZE(gfeeprom))
				uiEEPROMPosition += areaHead.length;
			else
				break;
		}
	}

	if (!bFound)
	{
		uiEEPROMPosition -= sizeof(struct gfeeprom_area_head);
		uiStartPosition = uiEEPROMPosition;
		client = at24_translate_offset(gfeeprom, &uiStartPosition);
		dev_dbg(&(client->dev), "gfeeprom_write_eeprom_area: Area type %d couldn't be found on controller %d at address 0x%X, appending area type at position: 0x%X.\r\n", areaType, gfeeprom->bus_id, client->addr << 1, uiEEPROMPosition);
		areaHead.type = areaType;
		areaHead.crc32 = 0;
		areaHead.length = usSize;
		dwReturn = gfeeprom_write_page_mode(gfeeprom, &areaHead, lpData, usSize, uiEEPROMPosition);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_err(&(client->dev), "ERROR: gfeeprom_write_eeprom_area(2): Area write failed, Code = 0x%x\r\n", dwReturn);
			return dwReturn;
		}
	}
	else
	{
		uiStartPosition = uiEEPROMPosition;
		client = at24_translate_offset(gfeeprom, &uiStartPosition);
		dev_dbg(&(client->dev), "gfeeprom_write_eeprom_area: Area type %d has been found on controller %d at address 0x%X, overwriting area type\r\n", areaType, gfeeprom->bus_id, client->addr << 1);

		if (usSize != areaHead.length)
		{
			dev_dbg(&(client->dev), "gfeeprom_write_eeprom_area: Detect old EEPROM area with different size, ");
			if (bForceWrite)
				dev_alert(&(client->dev), "forcing overwite of the area. All following areas will be lost!!!\r\n");
			else
			{
				dev_dbg(&(client->dev), "terminating write.\r\n");
				dev_dbg(&(client->dev), "ERROR: gfeeprom_write_eeprom_area: Area write failed, Code = 0x%x\r\n", ERROR_INSUFFICIENT_BUFFER);
				return ERROR_INSUFFICIENT_BUFFER;
			}
		}

		dwReturn = gfeeprom_write_page_mode(gfeeprom, NULL, lpData, usSize, uiEEPROMPosition);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_err(&(client->dev), "ERROR: gfeeprom_write_eeprom_area: Area write failed, Code = 0x%x\r\n", dwReturn);
			return dwReturn;
		}

		uiEEPROMPosition -= sizeof(struct gfeeprom_area_head);
		if (usSize != areaHead.length)
		{
			dev_alert(&(client->dev), "gfeeprom_write_eeprom_area: Updating the area head size to %d because of a forced write!\r\n", usSize);
			dwReturn = gfeeprom_update_area_size(gfeeprom, &areaHead, usSize, uiEEPROMPosition);
			if (ERROR_SUCCESS != dwReturn)
			{
				dev_err(&(client->dev), "ERROR: gfeeprom_write_eeprom_area: CRC32 update failed, Code = 0x%x\r\n", dwReturn);
				return dwReturn;
			}
		}
	}

	dwReturn = gfeeprom_update_area_crc32(gfeeprom, &areaHead, lpData, usSize, uiEEPROMPosition);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_err(&(client->dev), "ERROR: gfeeprom_write_eeprom_area: CRC32 update failed, Code = 0x%x\r\n", dwReturn);
		return dwReturn;
	}
	return ERROR_SUCCESS;
}

static u32 gfeeprom_program_hardware_version_information(struct gfeeprom_data *gfeeprom, struct gfv_hw_version_data* lpInput, bool bVerifyWrite, bool bForceWrite)
{
	u32 dwReturn = ERROR_SUCCESS;
	u8* lpData = NULL;
	u8* lpCompareData = NULL;
	u16 usSize;

	gfeeprom_dump_gfhw(gfeeprom, lpInput);

	if (ROMSIZE_M24C < GET_ROM_SIZE(gfeeprom))
	{
		usSize = sizeof(struct gfeeprom_hw_normal_version_data);
		lpData = kzalloc(usSize, GFP_KERNEL);
		lpCompareData = kzalloc(usSize, GFP_KERNEL);
		if (!lpData || !lpCompareData) {
			dwReturn = -ENOMEM;
			goto out;
		}

		dwReturn = gfeeprom_gfhw_to_ehw_normal(lpInput, (struct gfeeprom_hw_normal_version_data*)lpData);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_program_hardware_version_information: Code = 0x%x\r\n", dwReturn);
			goto out;
		}
	}
	else
	{
		usSize = sizeof(struct gfeeprom_hw_small_version_data);
		lpData = kzalloc(usSize, GFP_KERNEL);
		lpCompareData = kzalloc(usSize, GFP_KERNEL);
		if (!lpData || !lpCompareData) {
			dwReturn = -ENOMEM;
			goto out;
		}

		dwReturn = gfeeprom_gfhw_to_ehw_small(lpInput, (struct gfeeprom_hw_small_version_data*)lpData);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_program_hardware_version_information: Code = 0x%x\r\n", dwReturn);
			goto out;
		}
	}

	dwReturn = gfeeprom_write_eeprom_area(gfeeprom, EEPROM_TYPE_EHW, lpData, usSize, bForceWrite);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_program_hardware_version_information: Write failed with code  = 0x%x\r\n", dwReturn);
		goto out;
	}
	gfeeprom->hw_version_stale = true;

	if (bVerifyWrite)
	{
		dwReturn = gfeeprom_read_eeprom_area(gfeeprom, EEPROM_TYPE_EHW, lpCompareData, usSize, NULL, NULL);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_program_hardware_version_information: Read back failed with code = 0x%x\r\n", dwReturn);
			goto out;
		}

		if (memcmp(lpData, lpCompareData, usSize))
		{
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_read_eeprom_area: Data verification failed.\r\n");
			dwReturn = ERROR_INVALID_DATA;
			goto out;
		}

	}

out:
	if (lpData)
		kfree(lpData);
	if (lpCompareData)
		kfree(lpCompareData);

	return dwReturn;
}

static u32 gfeeprom_detect_available_eeprom(struct gfeeprom_data *gfeeprom)
{
	u32					dwReturn;
	u32					uiStartPosition = 0;
	struct i2c_client* 	client = at24_translate_offset(gfeeprom, &uiStartPosition);

	dwReturn = gfeeprom_read_eeprom_head(gfeeprom, 0);
	if (ERROR_SUCCESS != dwReturn)
		return dwReturn;

	dev_dbg(&(client->dev), "gfeeprom_detect_available_eeprom: Trying to detect the GF hardware desciption on controller %d at address 0x%02X...", gfeeprom->bus_id, client->addr << 1);

	if (ROMSIZE_M24C < GET_ROM_SIZE(gfeeprom))
	{
		struct gfeeprom_hw_normal_version_data eHW_Version;

		dwReturn = gfeeprom_read_eeprom_area(gfeeprom, EEPROM_TYPE_EHW, (u8*)&eHW_Version, sizeof(struct gfeeprom_hw_normal_version_data), NULL, NULL);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_info(&(client->dev), " not found.\r\n");
			return dwReturn;
		}

		if (eHW_Version.size_of_struct != sizeof(struct gfeeprom_hw_normal_version_data))
		{
			dev_err(&(client->dev), "ERROR: gfeeprom_detect_available_eeprom: No valid size of hardware struct found.\r\n");
			return ERROR_OLD_VERSION;
		}

		dwReturn = gfeeprom_normal_ehw_to_gfhw(&eHW_Version, &gfeeprom->hw_version);
		if (ERROR_SUCCESS != dwReturn)
			return dwReturn;
		gfeeprom->guf_hw_info_present = true;
	}
	else
	{
		struct gfeeprom_hw_small_version_data eHW_Version;

		dwReturn = gfeeprom_read_eeprom_area(gfeeprom, EEPROM_TYPE_EHW, (u8*)&eHW_Version, sizeof(struct gfeeprom_hw_small_version_data), NULL, NULL);
		if (ERROR_SUCCESS != dwReturn)
		{
			dev_info(&(client->dev), " not found.\r\n");
			return dwReturn;
		}

		if (eHW_Version.size_of_struct != sizeof(struct gfeeprom_hw_small_version_data))
		{
			dev_err(&(client->dev), "ERROR: gfeeprom_detect_available_eeprom: No valid size of hardware struct found.\r\n");
			return ERROR_OLD_VERSION;
		}

		dwReturn = gfeeprom_small_ehw_to_gfhw(&eHW_Version, &gfeeprom->hw_version);
		if (ERROR_SUCCESS != dwReturn)
			return dwReturn;
		gfeeprom->guf_hw_info_present = true;
	}

	gfeeprom_dump_gfhw(gfeeprom, &gfeeprom->hw_version);
	return ERROR_SUCCESS;
}

static u32 gfeeprom_delete_custom_area(struct gfeeprom_data *gfeeprom)
{
	u32 						dwReturn;
	u32							uiCounter;
	u32 						uiPos;
	u32 						uiPosOld;
	struct i2c_client* 			client = NULL;

	uiPosOld = 0;
	for (uiCounter = 0; uiCounter < GFEEPROM_MIN_AREA_TYPE_NUMBER; uiCounter++)
	{
		dwReturn = gfeeprom_read_eeprom_area(gfeeprom, uiCounter, NULL, 0, &uiPos, NULL);
		if ((ERROR_INSUFFICIENT_BUFFER == dwReturn) && (uiPos > uiPosOld))
			uiPosOld = uiPos;
	}

	client = at24_translate_offset(gfeeprom, &uiPosOld);
	dev_info(&(client->dev), "gfeeprom_delete_custom_area: start removing %i bytes of the custom area from position 0x%X to 0x%X.\r\n", GET_ROM_SIZE(gfeeprom) - uiPosOld, uiPosOld, GET_ROM_SIZE(gfeeprom));

	// Delete everything in the custom area
	dwReturn = gfeeprom_write_page_mode(gfeeprom, NULL, NULL, GET_ROM_SIZE(gfeeprom) - uiPosOld, uiPosOld);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_delete_custom_area: Code = 0x%x\r\n", dwReturn);
		return dwReturn;
	}
	return ERROR_SUCCESS;
}


static u32 gfeeprom_clean(struct gfeeprom_data *gfeeprom)
{
	u32 					dwReturn;

	// Delete everything
	dwReturn = gfeeprom_write_page_mode(gfeeprom, NULL, NULL, GET_ROM_SIZE(gfeeprom), 0);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_clean: Code = 0x%x\r\n", dwReturn);
		return dwReturn;
	}
	return ERROR_SUCCESS;
}

//------------------------------------------------------------------------------
//
//  Function:  OALIoCtlHalProgramEEPROMArea
//
//  This function is called to program a EEPROM with a GFSystemInformation on
//	one of the available EEPROMS.
//
//------------------------------------------------------------------------------
bool gfeeprom_ioctl_program_eeprom_area(struct gfeeprom_data *gfeeprom, struct gfeeprom_program_area* eeprom_area)
{
	u32					dwReturn;

	if (sizeof(struct gfeeprom_program_area) != eeprom_area->size_of_struct)
	{
		dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_program_eeprom_area: EEPROM info size isn't correct, wrong version used.\r\n");
		return false;
	}
	dwReturn = gfeeprom_program_hardware_version_information(gfeeprom,
															&(eeprom_area->hw_version),
															eeprom_area->verify,
															eeprom_area->force_write);
	if (ERROR_SUCCESS != dwReturn)
	{
		dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_program_eeprom_area: code : 0x%X\r\n", dwReturn);
		return false;
	}
	memcpy(&(gfeeprom->hw_version), &(eeprom_area->hw_version), sizeof(struct gfv_hw_version_data));
	return true;
}

int gfeeprom_ioctl_access_eeprom_area(struct gfeeprom_data *gfeeprom, struct gfeeprom_access* eeprom_access)
{
	u32 dwRC = ERROR_SUCCESS;

	if (sizeof(struct gfeeprom_access) != eeprom_access->size_of_struct)
	{
		dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area: EEPROM info size isn't correct, wrong version used.\r\n");
		return -EIO;
	}
	if (EEPROM_TYPE_EHW == eeprom_access->area_type)
	{
		dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area: No access to the hardware version area\r\n");
		return -EIO;
	}

	switch (eeprom_access->access_type)
	{
		case EEPROM_ACCESS_TYPE_READ:
		{
			u16 area_size = 0;
			dwRC = gfeeprom_read_eeprom_area(gfeeprom,
										 eeprom_access->area_type,
										 eeprom_access->data,
										 eeprom_access->data_length,
										 NULL,
										 &area_size);
			if(ERROR_SUCCESS != dwRC)
			{
				dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area failed, Error = %i\r\n", dwRC);
				if (ERROR_INSUFFICIENT_BUFFER == dwRC)
					return -EFBIG;
				return -EIO;
			}
			else
				dwRC = area_size;
			break;
		}

		case EEPROM_ACCESS_TYPE_WRITE:
			dwRC = gfeeprom_write_eeprom_area(gfeeprom,
										  eeprom_access->area_type,
										  eeprom_access->data,
										  eeprom_access->data_length,
										  eeprom_access->force_write);
			if(ERROR_SUCCESS != dwRC)
			{
				dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area failed, Error = %i\r\n", dwRC);
				return -EIO;
			}

			if (eeprom_access->verify_buffer && (eeprom_access->verify_buffer_length >= eeprom_access->data_length))
			{
				dwRC = gfeeprom_read_eeprom_area(gfeeprom,
											 eeprom_access->area_type,
											 eeprom_access->verify_buffer,
											 eeprom_access->data_length,
											 NULL,
											 NULL);
				if(ERROR_SUCCESS != dwRC)
				{
					dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area failed, Error = %i\r\n", dwRC);
					return -EIO;
				}

				if (memcmp(eeprom_access->data, eeprom_access->verify_buffer, eeprom_access->data_length))
				{
					dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area write verify failed, Error = %i\r\n", ERROR_INVALID_DATA);
					return -EIO;
				}
			}
			break;

		case EEPROM_ACCESS_TYPE_DELETE_ALL:
			dwRC = gfeeprom_delete_custom_area(gfeeprom);
			if(ERROR_SUCCESS != dwRC)
			{
				dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area failed, Error = %i\r\n", dwRC);
				return -EIO;
			}
			break;

		case EEPROM_ACCESS_TYPE_CLEAN:
			dwRC = gfeeprom_clean(gfeeprom);
			if(ERROR_SUCCESS != dwRC)
			{
				dev_dbg(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area failed, Error = %i\r\n", dwRC);
				return -EIO;
			}
			break;

		default:
			dev_err(&(gfeeprom->client[0]->dev), "ERROR: gfeeprom_ioctl_access_eeprom_area failed, Error = %i\r\n", dwRC);
			return -EIO;
	}
	return dwRC;
}

/*-------------------------------------------------------------------------*/
/*
 * This lets other kernel code access the eeprom data. For example, it
 * might hold a board's Ethernet address, or board-specific calibration
 * data generated on the manufacturing floor.
 */

static ssize_t gfeeprom_macc_read(struct memory_accessor *macc, char *buf, off_t offset, size_t count)
{
	ssize_t size = 0;
	struct gfeeprom_data *gfeeprom = container_of(macc, struct gfeeprom_data, macc);

	switch (offset)
	{
		case GFEEPROM_MACC_MAJOR_VERSION_OFFS:
			size = strlen(gfeeprom->hw_version.major);
			if (size < count)
				strcpy(buf, gfeeprom->hw_version.major);
			else
				size = 0;
			break;

		case GFEEPROM_MACC_MINOR_VERSION_OFFS:
			size = strlen(gfeeprom->hw_version.minor);
			if (size < count)
				strcpy(buf, gfeeprom->hw_version.minor);
			else
				size = 0;
			break;

		case GFEEPROM_MACC_ARTICLE_NUMBER_OFFS:
			size = strlen(gfeeprom->hw_version.article_number);
			if (size < count)
				strcpy(buf, gfeeprom->hw_version.article_number);
			else
				size = 0;
			break;

		case GFEEPROM_MACC_SERIAL_NUMBER_OFFS:
			size = strlen(gfeeprom->hw_version.serial_numbers);
			if (size < count)
				strcpy(buf, gfeeprom->hw_version.serial_numbers);
			else
				size = 0;
			break;

		case GFEEPROM_MACC_COMPONENT_OFFS:
			size = strlen(gfeeprom->hw_version.component);
			if (size < count)
				strcpy(buf, gfeeprom->hw_version.component);
			else
				size = 0;
			break;

		case GFEEPROM_MACC_COMMENT_OFFS:
			size = strlen(gfeeprom->hw_version.comment);
			if (size < count)
				strcpy(buf, gfeeprom->hw_version.comment);
			else
				size = 0;
			break;

		case GFEEPROM_MACC_ADDRESS_OF_COMPONENT_OFFS:
			if (sizeof(u32) < count)
			{
				memcpy(buf, &(gfeeprom->hw_version.address_of_component), sizeof(u32));
				size = sizeof(u32);
			}
			break;

		case GFEEPROM_MACC_PHYS_MEM_SIZE_OFFS:
			if (sizeof(u32) < count)
			{
				memcpy(buf, &(gfeeprom->hw_version.address_of_component), sizeof(u32));
				size = sizeof(u32);
			}
			break;
	}

	return size;
}

/*-------------------------------------------------------------------------*/
static int gfeeprom_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	int 	err = 0;
	struct 	gfeeprom_data* 			gfeeprom = NULL;
	struct 	gfeeprom_program_area*	eeprom_program = (struct gfeeprom_program_area*)arg;
	struct 	gfeeprom_access* 		eeprom_access = (struct gfeeprom_access*)arg;
	long   	nr = _IOC_NR(cmd);

	if (!arg)
		return -EINVAL;

	gfeeprom = i2c_get_clientdata(client);

	switch (nr)
	{
		case GFEEPROM_IOC_ACCESS_NUMBER:
			if ((_IOC_SIZE(cmd) != sizeof(struct gfeeprom_access)) ||
				(sizeof(struct gfeeprom_access) != eeprom_access->size_of_struct))
				return -EINVAL;

			if ((EEPROM_ACCESS_TYPE_WRITE == eeprom_access->access_type) &&
				(EEPROM_TYPE_EHW == eeprom_access->area_type))
				return -EINVAL;

			err = gfeeprom_ioctl_access_eeprom_area(gfeeprom, eeprom_access);
			break;

		case GFEEPROM_IOC_PROGRAM_HW_NUMBER:
			if (_IOC_SIZE(cmd) != sizeof(struct gfeeprom_program_area))
				return -EINVAL;

			err = gfeeprom_ioctl_program_eeprom_area(gfeeprom, eeprom_program);
			break;

		default:
			err = -ENOIOCTLCMD;
	}
	return err;
}

static ssize_t gfeeprom_read(struct file *fp, char __user *userbuf, size_t len, loff_t *off)
{
	struct gfeeprom_file_data* gfeeprom_fdata = NULL;
	struct gfeeprom_data* gfeeprom = NULL;
	struct gfeeprom_access eeprom_access;
	int err;
	int count;
	char *buf;

	if (!userbuf || !off || !len)
		return -EINVAL;

	gfeeprom_fdata = (struct gfeeprom_file_data*)fp->private_data;
	gfeeprom = gfeeprom_fdata->gfeeprom_hw;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (EEPROM_TYPE_EHW == *off)
	{
		/* re-read HW information if it has been changed in the meantime */
		if (gfeeprom->hw_version_stale)
		{
			gfeeprom_detect_available_eeprom(gfeeprom);
			gfeeprom->hw_version_stale = false;
		}

		if (!gfeeprom->guf_hw_info_present)
		{
			err = -ENOENT;
			goto free;
		}
		//dev_info(&gfeeprom->client[0]->dev, "gfeeprom_read: type=EEPROM_TYPE_EHW\n");
		if (sizeof(ROMSIZE_M24C < GET_ROM_SIZE(gfeeprom)))
		{
			if (len >= sizeof(struct gfeeprom_hw_normal_version_data))
			{
				if (gfeeprom_gfhw_to_ehw_normal(&(gfeeprom->hw_version), (struct gfeeprom_hw_normal_version_data*)buf) != ERROR_SUCCESS)
				{
					err = -EIO;
					goto free;
				}
				//dev_info(&gfeeprom->client[0]->dev, "gfeeprom_read: normal eeprom size returned\n");
				count = sizeof(struct gfeeprom_hw_normal_version_data);
				goto copy;
			}
			err = -ENOMEM;
			goto free;
		}
		else
		{
			if (len >= sizeof(struct gfeeprom_hw_small_version_data))
			{
				if (gfeeprom_gfhw_to_ehw_small(&(gfeeprom->hw_version), (struct gfeeprom_hw_small_version_data*)buf) != ERROR_SUCCESS)
				{
					err = -EIO;
					goto free;
				}
				//dev_info(&gfeeprom->client[0]->dev, "gfeeprom_read: small eeprom size returned\n");
				count = sizeof(struct gfeeprom_hw_small_version_data);
				goto copy;
			}
			err = -ENOMEM;
			goto free;
		}
	}

	dev_dbg(&gfeeprom->client[0]->dev, "gfeeprom_read: type=%u\n", (u16)*off);
	eeprom_access.size_of_struct = sizeof(struct gfeeprom_access);
	eeprom_access.access_type = EEPROM_ACCESS_TYPE_READ;
	eeprom_access.area_type = (u16)*off;
	eeprom_access.data = buf;
	eeprom_access.data_length = (u16)len;
	eeprom_access.verify_buffer = NULL;
	eeprom_access.verify_buffer_length = 0;

	count = gfeeprom_ioctl_access_eeprom_area(gfeeprom, &eeprom_access);
	if (!IS_ERR_VALUE(count))
		dev_dbg(&gfeeprom->client[0]->dev, "gfeeprom_read: size=%u\n", count);

copy:
	err = copy_to_user(userbuf, buf, len);
	if (err == ERROR_SUCCESS)
		err = count;
free:
	kfree(buf);
	return err;
}

static ssize_t gfeeprom_write(struct file *fp, const char __user *userbuf, size_t len, loff_t *off)
{
	ssize_t ret;
	struct gfeeprom_file_data* gfeeprom_fdata = NULL;
	struct gfeeprom_data* gfeeprom = NULL;
	struct gfeeprom_access eeprom_access;
	char *buf;

	if (!userbuf || !off || !len)
		return -EINVAL;

	gfeeprom_fdata = (struct gfeeprom_file_data*)fp->private_data;
	gfeeprom = gfeeprom_fdata->gfeeprom_hw;

	if (!gfeeprom->writable ||
		(EEPROM_TYPE_EHW == *off))
		return -EPERM;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = copy_from_user(buf, userbuf, len);
	if (ret < 0) {
		kfree(buf);
		return ret;
	}

	eeprom_access.size_of_struct = sizeof(struct gfeeprom_access);
	eeprom_access.access_type = EEPROM_ACCESS_TYPE_WRITE;
	eeprom_access.area_type = (u16)*off;
	eeprom_access.data = (u8*)buf;
	eeprom_access.data_length = (u16)len;
	eeprom_access.verify_buffer = kzalloc(eeprom_access.data_length, GFP_KERNEL);
	eeprom_access.verify_buffer_length = eeprom_access.data_length;

	ret = gfeeprom_ioctl_access_eeprom_area(gfeeprom, &eeprom_access);
	if (!IS_ERR_VALUE(ret))
		ret = eeprom_access.data_length;
	kfree(eeprom_access.verify_buffer);
	kfree(buf);
	return ret;
}

static long gfeeprom_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct 	gfeeprom_file_data* gfeeprom_fdata = NULL;
	struct 	gfeeprom_data* gfeeprom = NULL;
	long   	ret = 0;
	long   	nr = _IOC_NR(cmd);

	if (!arg)
		return -EINVAL;

	gfeeprom_fdata = (struct gfeeprom_file_data*)fp->private_data;
	gfeeprom = gfeeprom_fdata->gfeeprom_hw;

	switch (nr)
	{
		case GFEEPROM_IOC_ACCESS_NUMBER:
		{
			struct 	gfeeprom_access eeprom_access;
			char*	data_buffer = NULL;
			char*	verify_buffer = NULL;

			if (!(cmd & IOC_IN))
				return -EINVAL;

			if (_IOC_SIZE(cmd) != sizeof(struct gfeeprom_access))
				return -EINVAL;

            if (copy_from_user(&eeprom_access, (void __user *)arg, sizeof(struct gfeeprom_access)) != 0)
				return -EFAULT;

			if (EEPROM_ACCESS_TYPE_READ == eeprom_access.access_type)
				return -EINVAL;

			if ((EEPROM_ACCESS_TYPE_WRITE == eeprom_access.access_type) &&
				!gfeeprom->writable)
				return -EPERM;

			if ((EEPROM_ACCESS_TYPE_WRITE == eeprom_access.access_type) &&
				(EEPROM_TYPE_EHW != eeprom_access.area_type))
				return -EINVAL;

			if (eeprom_access.data_length) {
				data_buffer = kmalloc(eeprom_access.data_length, GFP_KERNEL);
				if (copy_from_user(data_buffer, (void __user *)eeprom_access.data, eeprom_access.data_length) != 0)
					return -EFAULT;
				eeprom_access.data = data_buffer;
			}

			if (eeprom_access.verify_buffer_length) {
				verify_buffer = kmalloc(eeprom_access.verify_buffer_length, GFP_KERNEL);
				eeprom_access.verify_buffer = verify_buffer;
			}

			ret = gfeeprom_ioctl_access_eeprom_area(gfeeprom, &eeprom_access);
			if (data_buffer)
				kfree(data_buffer);
			if (verify_buffer)
				kfree(verify_buffer);
			break;
		}

		case GFEEPROM_IOC_GET_CUSTOM_AREA_COUNT_NUMBER:
		{
			u16 erea_count = 0;

			if (!(cmd & IOC_OUT))
				return -EINVAL;

			if (_IOC_SIZE(cmd) != sizeof(u16))
				return -EINVAL;

			gfeeprom_count_eeprom_areas(gfeeprom, &erea_count);

            if (copy_to_user((void __user *)arg, &erea_count, sizeof(u16)) != 0)
				return -EFAULT;
			break;
		}

		case GFEEPROM_IOC_GET_USED_CUSTOM_AREA_NUMBER:
		{
			u16 erea_count = 0;
			u16* areas_buffer = NULL;
			struct gfeeprom_area_array* areas = (struct gfeeprom_area_array*) arg;

			if (!(cmd & IOC_OUT))
				return -EINVAL;

			if (_IOC_SIZE(cmd) != sizeof(struct gfeeprom_area_array))
				return -EINVAL;

			if (sizeof(struct gfeeprom_area_array) != areas->size_of_struct)
				return -EINVAL;

			ret = gfeeprom_count_eeprom_areas(gfeeprom, &erea_count);
			if (ret < 0)
				break;

			if ((erea_count > areas->elements) ||
				(!areas->areas))
				return -ENOSPC;

			areas_buffer = kmalloc(sizeof(u16)*erea_count, GFP_KERNEL);
			if (!areas_buffer)
			{
				ret = -ENOMEM;
				break;
			}

			gfeeprom_get_used_eeprom_areas(gfeeprom, areas_buffer, erea_count);

            if (copy_to_user(areas->areas, areas_buffer, sizeof(u16)*erea_count) != 0)
            {
				kfree(areas_buffer);
				return -EFAULT;
			}
			kfree(areas_buffer);
			break;
		}

		case GFEEPROM_IOC_PROGRAM_HW_NUMBER:
		{
			struct 	gfeeprom_program_area*	eeprom_program = (struct gfeeprom_program_area*)arg;

			if (_IOC_SIZE(cmd) != sizeof(struct gfeeprom_program_area))
				return -EINVAL;

			ret = gfeeprom_ioctl_program_eeprom_area(gfeeprom, eeprom_program);
			break;
		}

		default:
			ret = -ENOIOCTLCMD;
	}
	return ret;
}

static int gfeeprom_open(struct inode *ip, struct file *fp)
{
	int slot, error = 0;
	struct gfeeprom_file_data* gfeeprom_fdata = NULL;
	struct gfeeprom_data* gfeeprom = NULL;

	slot = iminor(ip) % MAX_GFEEPROM_CHIPS;
	gfeeprom = container_of(ip->i_cdev, struct gfeeprom_data, cdev);

	spin_lock(&gfeeprom->open_lock);
	if (gfeeprom->gfeeprom_fdata_alloc[slot] == NULL)
	{
		gfeeprom_fdata = kzalloc(sizeof(struct gfeeprom_file_data), GFP_KERNEL);
		if (!gfeeprom_fdata)
			error = -ENOMEM;
		else
		{
			gfeeprom_fdata->cnt = 1;
			gfeeprom_fdata->excl = fp->f_flags & O_EXCL;
			gfeeprom_fdata->slot = slot;
			gfeeprom_fdata->gfeeprom_hw = gfeeprom;
			gfeeprom->gfeeprom_fdata_alloc[slot] = gfeeprom_fdata;
		}
	}
	else
	{
		if (fp->f_flags & O_EXCL || gfeeprom->gfeeprom_fdata_alloc[slot]->excl)
			error = -EBUSY;
		else
			gfeeprom->gfeeprom_fdata_alloc[slot]->cnt++;
	}
	spin_unlock(&gfeeprom->open_lock);
	if (!error)
		fp->private_data = gfeeprom->gfeeprom_fdata_alloc[slot];
	return error;
}

static int gfeeprom_close(struct inode *ip, struct file *fp)
{
	int slot;
	struct gfeeprom_data* gfeeprom = NULL;

	slot = iminor(ip) % MAX_GFEEPROM_CHIPS;
	gfeeprom = container_of(ip->i_cdev, struct gfeeprom_data, cdev);

	spin_lock(&gfeeprom->open_lock);
	if (gfeeprom->gfeeprom_fdata_alloc[slot]->cnt == 1)
	{
		gfeeprom->gfeeprom_fdata_alloc[slot] = NULL;
		kfree(fp->private_data);
		fp->private_data = NULL;
	}
	else
		gfeeprom->gfeeprom_fdata_alloc[slot]->cnt--;
	spin_unlock(&gfeeprom->open_lock);

	return 0;
}

static const struct file_operations gfeeprom_fops = {
	.owner			= THIS_MODULE,
	.read			= gfeeprom_read,
	.write			= gfeeprom_write,
	.unlocked_ioctl = gfeeprom_ioctl,
	.open 			= gfeeprom_open,
	.release 		= gfeeprom_close
};

#ifdef CONFIG_OF
static const struct of_device_id gfeeprom_dt_ids[] = {
	{ .compatible = "guf,gfeeprom", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gfeeprom_dt_ids);

#endif

static int gfeeprom_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gfeeprom_platform_data 		chip;
	bool 								use_smbus = false;
	struct gfeeprom_data*				gfeeprom = NULL;
	int 								err, minor;
	unsigned 							i, num_addresses;
	struct device*						dev = NULL;
	dev_t 								dev_type;

#ifdef CONFIG_OF
	const struct of_device_id *of_id = of_match_device(gfeeprom_dt_ids, &client->dev);
	if( of_id == NULL)
	{
		dev_warn(&client->dev, "No device tree entry for gfeeprom found\n");
		if (client->dev.platform_data) {
			chip = *(struct gfeeprom_platform_data *)client->dev.platform_data;
		} else {
			err = -ENODEV;
			goto err_out;
		}
	}
	else
	{
		struct device_node *np = client->dev.of_node;
		int ret, val;
		memset( &chip, 0, sizeof(chip));

		ret = of_property_read_u32(np, "pagesize", &val);
		if (ret) {
			dev_err(&client->dev, "pagesize missing or invalid\n");
			return ret;
		}
		chip.page_size = val;
		ret = of_property_read_u32(np, "bytelen", &val);
		if (ret) {
			dev_err(&client->dev, "bytelen missing or invalid\n");
			return ret;
		}
		chip.byte_len = val;
		ret = of_property_read_u32(np, "bus-id", &val);
		if (ret) {
			dev_err(&client->dev, "bytelen missing or invalid\n");
			return ret;
		}
		chip.bus_id = val;
		ret = of_property_read_u32(np, "flags", &val);
		if (ret) {
			dev_err(&client->dev, "flags missing or invalid\n");
			return ret;
		}
		chip.flags = val;

	}

#else
	if (client->dev.platform_data) {
		chip = *(struct gfeeprom_platform_data *)client->dev.platform_data;
	} else {
		err = -ENODEV;
		goto err_out;
	}
#endif

	dev_dbg(&client->dev, "pagesize 0x%0x, bytelen %d, flags 0x%0x\n",  chip.page_size, chip.byte_len, chip.flags );

	if (!is_power_of_2(chip.byte_len))
		dev_warn(&client->dev, "byte_len looks suspicious (no power of 2)!\n");
	if (!is_power_of_2(chip.page_size))
		dev_warn(&client->dev, "page_size 0x%0x looks suspicious (no power of 2)!\n", chip.page_size);

	/* Use I2C operations unless we're stuck with SMBus extensions. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		if (chip.flags & AT24_FLAG_ADDR16) {
			err = -EPFNOSUPPORT;
			goto err_out;
		}
		if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
			err = -EPFNOSUPPORT;
			goto err_out;
		}
		use_smbus = true;
	}

	if (chip.flags & AT24_FLAG_TAKE8ADDR)
		num_addresses = 8;
	else
		num_addresses =	DIV_ROUND_UP(chip.byte_len, (chip.flags & AT24_FLAG_ADDR16) ? 65536 : 256);

	gfeeprom = kzalloc(sizeof(struct gfeeprom_data) + num_addresses * sizeof(struct i2c_client *), GFP_KERNEL);
	if (!gfeeprom)
	{
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&gfeeprom->lock);
	gfeeprom->use_smbus = use_smbus;
	gfeeprom->chip = chip;
	gfeeprom->num_addresses = num_addresses;

	/*
	 * Export the EEPROM bytes through sysfs, since that's convenient.
	 * By default, only root should see the data (maybe passwords etc)
	 */
	gfeeprom->bin.attr.name = "gfeeprom";
	gfeeprom->bin.attr.mode = chip.flags & AT24_FLAG_IRUGO ? S_IRUGO : S_IRUSR;
	gfeeprom->bin.size = chip.byte_len;
	gfeeprom->bus_id = chip.bus_id;
	gfeeprom->macc.read = gfeeprom_macc_read;

	gfeeprom->client[0] = client;

	gfeeprom->writable = !(chip.flags & AT24_FLAG_READONLY);
	if (gfeeprom->writable) {
		if (!use_smbus || i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {

			unsigned write_max = chip.page_size;

			// currently not supported
			//gfeeprom->macc.write = gfeeprom_macc_write;

			if (write_max > io_limit)
				write_max = io_limit;
			if (use_smbus && write_max > I2C_SMBUS_BLOCK_MAX)
				write_max = I2C_SMBUS_BLOCK_MAX;
			gfeeprom->write_max = write_max;

			/* buffer (data + address at the beginning) */
			gfeeprom->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
			if (!gfeeprom->writebuf) {
				err = -ENOMEM;
				goto err_struct;
			}
		} else {
			gfeeprom->writable = false;
			dev_warn(&client->dev, "cannot write due to controller restrictions.");
		}
	}

	// use dummy devices for multiple-address chips
	for (i = 1; i < num_addresses; i++) {
		gfeeprom->client[i] = i2c_new_dummy(client->adapter,
					client->addr + i);
		if (!gfeeprom->client[i]) {
			dev_err(&client->dev, "address 0x%02x unavailable\n",
					client->addr + i);
			err = -EADDRINUSE;
			goto err_clients;
		}
	}

	i2c_set_clientdata(client, gfeeprom);
	if (gfeeprom_detect_available_eeprom(gfeeprom) == ERROR_SUCCESS)
	{
		dev_info(&client->dev, "Detected hardware component: '%s' (%s), version: '%s.%s', comment: '%s'.\n",
			gfeeprom->hw_version.component, gfeeprom->hw_version.article_number,
			gfeeprom->hw_version.major, gfeeprom->hw_version.minor,
			(gfeeprom->hw_version.comment[0] != '\0') ? gfeeprom->hw_version.comment : "no comment");
		if (GF_GLOBAL_PLATFORM_EEPROM_ADDRESS == client->addr)
		{
			memcpy(&global_platform_data.version_data, &gfeeprom->hw_version, sizeof(struct gfv_hw_version_data));
			global_platform_data.valid_data = true;
		}
	}
	else
	{
		dev_info(&client->dev, "Uninitialzed EEPROM detected.\n");
	}

	dev_info(&client->dev, "%zu byte %s %s\n",
		gfeeprom->bin.size, client->name,
		gfeeprom->writable ? "(writable)" : "(read-only)");
	dev_dbg(&client->dev,
		"page_size %d, num_addresses %d, write_max %d%s\n",
		chip.page_size, num_addresses,
		gfeeprom->write_max,
		use_smbus ? ", use_smbus" : "");

	err = alloc_chrdev_region(&dev_type, (client->addr<<1), MAX_GFEEPROM_OPEN, GFEEPROM_NAME);
	if (!err)
	{
		unsigned int gfeeprom_major;

		gfeeprom_major = MAJOR(dev_type);
		cdev_init(&gfeeprom->cdev, &gfeeprom_fops);
		gfeeprom->cdev.owner = THIS_MODULE;
		err = cdev_add(&gfeeprom->cdev, dev_type, num_addresses);
		if (err) {
			dev_err(&client->dev, "Could not add cdev\n");
			unregister_chrdev_region(dev_type, MAX_GFEEPROM_OPEN);
			goto err_setup_file;
		}

		for (minor = client->addr; minor < (num_addresses + client->addr); minor++) {
			dev = device_create(gfeeprom_class, &client->dev,
								MKDEV(gfeeprom_major, (minor<<1)), NULL,
								"gfeeprom!bus%deeprom%02X", gfeeprom->bus_id, (minor<<1));
			if (IS_ERR(dev))
				dev_err(&client->dev, "Could not create device file node.\n");
		}
	}

err_setup_file:
	/* export data to kernel code */
	if (chip.setup)
		chip.setup(&gfeeprom->macc, chip.context);
	return 0;

err_clients:
	for (i = 1; i < num_addresses; i++)
		if (gfeeprom->client[i])
			i2c_unregister_device(gfeeprom->client[i]);

	kfree(gfeeprom->writebuf);
err_struct:
	kfree(gfeeprom);
err_out:
	dev_dbg(&client->dev, "probe error %d\n", err);
	return err;
}

static int gfeeprom_remove(struct i2c_client *client)
{
	struct gfeeprom_data *gfeeprom;
	int i;

	gfeeprom = i2c_get_clientdata(client);
	sysfs_remove_bin_file(&client->dev.kobj, &gfeeprom->bin);

	for (i = 1; i < gfeeprom->num_addresses; i++)
		i2c_unregister_device(gfeeprom->client[i]);

	kfree(gfeeprom->writebuf);
	kfree(gfeeprom);
	i2c_set_clientdata(client, NULL);
	return 0;
}


static struct i2c_driver gfeeprom_driver = {
	.driver = {
		.name = "gfeeprom",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gfeeprom_dt_ids,
#endif
	},
	.probe = gfeeprom_probe,
	.remove = gfeeprom_remove,
	.command = gfeeprom_command,
	.id_table = gfeeprom_ids
};

static int __init gfeeprom_init(void)
{
	int ret;

	gfeeprom_class = class_create(THIS_MODULE, "gfeeprom");
	if (IS_ERR(gfeeprom_class))
		return PTR_ERR(gfeeprom_class);

	io_limit = rounddown_pow_of_two(io_limit);
	ret = i2c_add_driver(&gfeeprom_driver);
	if (ret)
		goto class_destroy;
	return ret;

class_destroy:
	class_destroy(gfeeprom_class);
	return ret;
}
module_init(gfeeprom_init);

static void __exit gfeeprom_exit(void)
{
	i2c_del_driver(&gfeeprom_driver);
	class_destroy(gfeeprom_class);
}
module_exit(gfeeprom_exit);

bool gf_get_platform_hw_version(struct gfv_hw_version_data* hw_version)
{
	bool ret = false;
	if (hw_version && global_platform_data.valid_data)
	{
		memcpy(hw_version, &global_platform_data.version_data, sizeof(struct gfv_hw_version_data));
		ret = true;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gf_get_platform_hw_version);

MODULE_DESCRIPTION("Driver for Garz & Fricke EEPROMs");
MODULE_AUTHOR("Nils Grimm");
MODULE_LICENSE("GPL");
