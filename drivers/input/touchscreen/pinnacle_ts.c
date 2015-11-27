/*
 * Copyright (C) 2010 Carsten Behling, Garz & Fricke GmbH
 *
 * Description:	Cirque Pinnacle touchscreen
 *
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Based on:
 *  - eeti_ts.c
 *	Copyright (C) 2009 Daniel Mack <daniel@caiaq.de>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sysfs.h>
#include <linux/fb.h>
#include <linux/console.h>

/**
 *	sysfs_attr_init - initialize a dynamically allocated sysfs attribute
 *	@attr: struct attribute to initialize
 *
 *	Initialize a dynamically allocated struct attribute so we can
 *	make lockdep happy.  This is a new requirement for attributes
 *	and initially this is only needed when lockdep is enabled.
 *	Lockdep gives a nice error when your attribute is added to
 *	sysfs if you don't have this.
 */
#ifdef CONFIG_DEBUG_LOCK_ALLOC
#define sysfs_attr_init(attr)				\
do {							\
	static struct lock_class_key __key;		\
							\
	(attr)->key = &__key;				\
} while(0)
#else
#define sysfs_attr_init(attr) do {} while(0)
#endif

/**
 *	sysfs_bin_attr_init - initialize a dynamically allocated bin_attribute
 *	@attr: struct bin_attribute to initialize
 *
 *	Initialize a dynamically allocated struct bin_attribute so we
 *	can make lockdep happy.  This is a new requirement for
 *	attributes and initially this is only needed when lockdep is
 *	enabled.  Lockdep gives a nice error when your attribute is
 *	added to sysfs if you don't have this.
 */
#define sysfs_bin_attr_init(bin_attr) sysfs_attr_init(&(bin_attr)->attr)

//#include <mach/mx3fb.h>

#include <linux/input/pinnacle_ts.h>
#include "pinnacle_ts.h"

#undef TOGGLE_DEBUG_PIN_ON_CAL
#define TOGGLE_DEBUG_PIN_ON_CAL				0
#undef DEBUG
#define DEBUG 								0
#undef PRINT_RAW_TOUCH_COORDINATES
#define PRINT_RAW_TOUCH_COORDINATES			0
#undef DISABLE_DISPLAY_WHILE_CALIBRATING
#define DISABLE_DISPLAY_WHILE_CALIBRATING 	1
#undef DBG_CONFIG_FILE
#define DBG_CONFIG_FILE 					0

#define INVALID_ADC_VAL (MAX_ADC_VAL+1)

// all values in this block should be exported into SysFS because they have to be modified by the user/project/product
#define GUC_UP_HYSTERESIS 			(priv->ucUpHysteresis)
#define GUC_DOWN_HYSTERESIS 		(priv->ucDownHysteresis)
#define GUC_MINIMUM_VALID_SAMPLES 	(priv->ucMinimumValidSamples)
#define GUC_MINIMUM_VALID_HOLD 		(priv->ucMinimumValidHold)
#define GUC_MINIMUM_VALID_SUSTAIN	(priv->ucMinimumValidSustain)
#define GUC_MINIMUM_VALID_DECAY		(priv->ucMinimumValidDecay)
#define GUL_MAX_DISTANCE			(priv->ulMaxDistance)
#define GUL_MAX_X_POS				(priv->ulMaxXPos)
#define GUL_MIN_X_POS				(priv->ulMinXPos)
#define GUL_MAX_Y_POS				(priv->ulMaxYPos)
#define GUL_MIN_Y_POS				(priv->ulMinYPos)
#define GUL_MIN_CAL_DIFF_VALUE		(priv->ulMinCalDiffValue)
#define GUL_CHECK_MAX_VAL_AFTER_CAL	(priv->ulCheckMaxValAfterCal)

#if DEBUG
#define PRINTK(fmt, args...) printk(fmt, ## args)
#else
#define PRINTK(fmt, args...)
#endif

// helper functions
#define BUFFER_STRING_LENGTH	500
#define PARAMETERS_IN_STRING	18
#define SKIP_START_PARAMS		2
#define PARSE_STRING "a: 0x%x (wl: %u w: 0x%x 0x%x 0x%x 0x%x) (rl: %u r: 0x%x 0x%x 0x%x 0x%x) (v: 0x%x 0x%x 0x%x 0x%x) ev:%u s:%u i:0x%x"

static bool parse_config_slot(struct pinnacle_touch_config_slot* lpTouchConfig, char* szConfigString, u32 ulMaxStringSize)
{
	u32 ulParamCount = 0;
	u32 lpParam[PARAMETERS_IN_STRING];

	ulParamCount = sscanf(szConfigString, PARSE_STRING, &(lpParam[0]), &(lpParam[1]), &(lpParam[2]), &(lpParam[3]), &(lpParam[4]), &(lpParam[5]),
						  								 &(lpParam[6]), &(lpParam[7]), &(lpParam[8]), &(lpParam[9]), &(lpParam[10]), &(lpParam[11]),
						  								 &(lpParam[12]), &(lpParam[13]), &(lpParam[14]), &(lpParam[15]), &(lpParam[16]), &(lpParam[17]));
	if (ulParamCount != PARAMETERS_IN_STRING)
	{
#if DBG_CONFIG_FILE
		//char szOutputString[BUFFER_STRING_LENGTH];
		//
		//strcpy(szOutputString, szConfigString);
		//szOutputString[strlen(szConfigString)-2] = '\0'; // removing the '\n' at the end for the debug message output.
#endif
		PRINTK(KERN_DEBUG "ParseConfigSlot: no value found in the config line.\r\n");
		return false;
	}

	lpTouchConfig->usAddress = (u16)lpParam[0];
	lpTouchConfig->lpWriteData[0] = (u8)lpParam[2];
	lpTouchConfig->lpWriteData[1] = (u8)lpParam[3];
	lpTouchConfig->lpWriteData[2] = (u8)lpParam[4];
	lpTouchConfig->lpWriteData[3] = (u8)lpParam[5];
	lpTouchConfig->ucWriteDataCount = (u8)lpParam[1];
	lpTouchConfig->lpReadData[0] = (u8)lpParam[7];
	lpTouchConfig->lpReadData[1] = (u8)lpParam[8];
	lpTouchConfig->lpReadData[2] = (u8)lpParam[9];
	lpTouchConfig->lpReadData[3] = (u8)lpParam[10];
	lpTouchConfig->ucReadDataCount = (u8)lpParam[6];
	lpTouchConfig->lpDataMask[0] = (u8)lpParam[11];
	lpTouchConfig->lpDataMask[1] = (u8)lpParam[12];
	lpTouchConfig->lpDataMask[2] = (u8)lpParam[13];
	lpTouchConfig->lpDataMask[3] = (u8)lpParam[14];
	lpTouchConfig->bVerification = lpParam[15] ? true : false;
	lpTouchConfig->dwSleep = lpParam[16];
	lpTouchConfig->ucWaitForInterrupt = (u8)lpParam[17];

#if DBG_CONFIG_FILE
	printk("parse_config_slot: lpTouchConfig->usAddress:          0x%04X.\r\n", lpTouchConfig->usAddress);
	printk("                   lpTouchConfig->lpWriteData[0]:     0x%02X.\r\n", lpTouchConfig->lpWriteData[0]);
	printk("                   lpTouchConfig->lpWriteData[1]:     0x%02X.\r\n", lpTouchConfig->lpWriteData[1]);
	printk("                   lpTouchConfig->lpWriteData[2]:     0x%02X.\r\n", lpTouchConfig->lpWriteData[2]);
	printk("                   lpTouchConfig->lpWriteData[3]:     0x%02X.\r\n", lpTouchConfig->lpWriteData[3]);
	printk("                   lpTouchConfig->ucWriteDataCount:   0x%02X.\r\n", lpTouchConfig->ucWriteDataCount);
	printk("                   lpTouchConfig->lpReadData[0]:      0x%02X.\r\n", lpTouchConfig->lpReadData[0]);
	printk("                   lpTouchConfig->lpReadData[1]:      0x%02X.\r\n", lpTouchConfig->lpReadData[1]);
	printk("                   lpTouchConfig->lpReadData[2]:      0x%02X.\r\n", lpTouchConfig->lpReadData[2]);
	printk("                   lpTouchConfig->lpReadData[3]:      0x%02X.\r\n", lpTouchConfig->lpReadData[3]);
	printk("                   lpTouchConfig->ucReadDataCount:    0x%02X.\r\n", lpTouchConfig->ucReadDataCount);
	printk("                   lpTouchConfig->lpDataMask[0]:      0x%02X.\r\n", lpTouchConfig->lpDataMask[0]);
	printk("                   lpTouchConfig->lpDataMask[1]:      0x%02X.\r\n", lpTouchConfig->lpDataMask[1]);
	printk("                   lpTouchConfig->lpDataMask[2]:      0x%02X.\r\n", lpTouchConfig->lpDataMask[2]);
	printk("                   lpTouchConfig->lpDataMask[3]:      0x%02X.\r\n", lpTouchConfig->lpDataMask[3]);
	printk("                   lpTouchConfig->bVerification:      %s.\r\n", (char*)(lpTouchConfig->bVerification ? "true" : "false"));
	printk("                   lpTouchConfig->dwSleep:            0x%08X.\r\n", lpTouchConfig->dwSleep);
	printk("                   lpTouchConfig->ucWaitForInterrupt: 0x%02X.\r\n\r\n", lpTouchConfig->ucWaitForInterrupt);
#endif
	return true;
}

static bool setup_touch_config_slots(struct pinnacle_ts_priv *priv, struct pinnacle_touch_config_slot** lplpTouchConfigs, u32* lpReturnedElements)
{
	char					szBufferString[BUFFER_STRING_LENGTH];
	struct i2c_client*      client = priv->client;
	u8						ucElementPosition = 0;
	u32						ulElementCounter = 0;
	u32						ulPosCounterInString = 0;
	bool					bSuccess = true;
	u8*						lpPos;
	const struct firmware* 	fw_entry;
	int						iReturn;
	bool					bUsedCalFirmware = false;

	PRINTK("setup_touch_config_slots: try open config file...");
	iReturn = request_firmware(&fw_entry, PINNACLE_CAL_FIRMWARE, &client->dev);
	if (iReturn < 0)
	{
		PRINTK("setup_touch_config_slots: couldn't load '%s' firmware file because of error %d.\r\n", PINNACLE_CAL_FIRMWARE, iReturn);
		iReturn = request_firmware(&fw_entry, PINNACLE_STD_FIRMWARE, &client->dev);
		if (iReturn < 0)
		{
			printk(KERN_ERR "setup_touch_config_slots: couldn't load '%s' firmware file because of error %d.\r\n", PINNACLE_STD_FIRMWARE, iReturn);
			return false;
		}
	}
	else
	{
		bUsedCalFirmware = true;
	}
	//printk("successful.\r\n");

	// read the file one time for getting the memory size we have to allocate for the buffer
	// each line in the file is representing one TOUCH_CONFIG_SLOT, counting the lines.
	lpPos = (u8*)fw_entry->data;
	for (*lpReturnedElements = 0; ((lpPos - (u8*)fw_entry->data) < (fw_entry->size - 2)); lpPos++)
	{
		if (('a' == *lpPos) && (':' == *(lpPos + 1)) && (' ' == *(lpPos + 2)))
		{
			(*lpReturnedElements)++;
			lpPos += 2;
		}
	}
	if (SKIP_START_PARAMS >= *lpReturnedElements)
	{
		printk("setup_touch_config_slots: No elements in file.\r\n");
		bSuccess = false;
		goto EXIT;
	}
	(*lpReturnedElements) -= SKIP_START_PARAMS;
	//printk("setup_touch_config_slots: Found %u elements in file.\r\n", *lpReturnedElements);

	// reset the file position to the start and search to the first 'a:'
	lpPos = (u8*)fw_entry->data;
	for (; (ucElementPosition <= SKIP_START_PARAMS) && ((lpPos - (u8*)fw_entry->data) < (fw_entry->size - 2)); lpPos++)
	{
		if (('a' == *lpPos) && (':' == *(lpPos + 1)) && (' ' == *(lpPos + 2)))
		{
			ucElementPosition++;
			lpPos += 2;
		}
	}
	lpPos -= 3; // correcting the last increment, we want to stay at the firs 'a'.
	// allocate the data structure for storing the configuration into.
	*lplpTouchConfigs = (struct pinnacle_touch_config_slot*) kzalloc(sizeof(struct pinnacle_touch_config_slot) * (*lpReturnedElements), GFP_KERNEL);
	if (!*lplpTouchConfigs)
	{
		printk(KERN_ERR "setup_touch_config_slots: Couldn't allocate RAM for the config data structure.\r\n");
		bSuccess = false;
		goto EXIT;
	}
#if DBG_CONFIG_FILE
	printk("setup_touch_config_slots: Allocated %u bytes of ram space for storing the configuration data.\r\n", sizeof(struct pinnacle_touch_config_slot) * (*lpReturnedElements));
#endif

	// run again through the file and fill in the data structure
	for (ulElementCounter = 0, ulPosCounterInString = 0; (ulElementCounter < *lpReturnedElements) && ((lpPos - (u8*)fw_entry->data) < fw_entry->size); lpPos++)
	{
		if (ulPosCounterInString == (BUFFER_STRING_LENGTH-1))
		{
			printk(KERN_ERR "setup_touch_config_slots: Buffer string length isn't enough for handling string.\r\n");
			bSuccess = false;
			goto EXIT;
		}
		if (('\r' == *lpPos) || ('\n' == *lpPos))
		{
			lpPos++;
#if DBG_CONFIG_FILE
			char szOutputString[BUFFER_STRING_LENGTH];
#endif
			if (('\r' == *lpPos) || ('\n' == *lpPos))
				lpPos++;
			szBufferString[ulPosCounterInString] = '\0';
			ulPosCounterInString = 0;
#if DBG_CONFIG_FILE
			strcpy(szOutputString, szBufferString);
			szOutputString[strlen(szOutputString)-2] = '\0'; // removing the '\n' at the end for the debug message output.
			printk("SetupTouchConfigSlots: read '%s' out of file.\r\n", szBufferString);
#endif
			if (!parse_config_slot(&((*lplpTouchConfigs)[ulElementCounter++]), szBufferString, BUFFER_STRING_LENGTH))
			{
				printk("setup_touch_config_slots: Couldn't parse config file. Skipping line.\r\n");
				ulElementCounter--;	//in the next run sending back in the last not used element
			}
		}
		szBufferString[ulPosCounterInString++] = *lpPos;
	}

EXIT:
	if (!ulElementCounter || !bSuccess)
	{
		printk(KERN_ERR "setup_touch_config_slots: failed reading config file.\r\n");
		if (*lplpTouchConfigs)
		{
			kfree(*lplpTouchConfigs);
			*lplpTouchConfigs = NULL;
		}
		*lpReturnedElements = 0;
	}
	else
	{
#if DBG_CONFIG_FILE
		printk(KERN_DEBUG "setup_touch_config_slots: successful finished.\r\n.");
#endif
	}
	PRINTK(KERN_DEBUG "setup_touch_config_slots: releasing firmware.\r\n");
	release_firmware(fw_entry);
	if (bSuccess && bUsedCalFirmware) {
		priv->bControllerCalibrated = true;
	}
	return bSuccess;
}

// touch controller functions
static bool inreg(struct i2c_client *client, u8 address, u8* lpData)
{
    s32 value = i2c_smbus_read_byte_data(client, address | PINNACLE_READ);
    if (value < 0)
    {
		PRINTK(KERN_ERR "inreg: communication failure with error: %i.\r\n", value);
		return false;
	}

	*lpData = (u8) value;
	return true;
}

static bool outreg(struct i2c_client *client, u8 address, u8 value)
{
	s32 ret_value;
	u8	writes[PINNACLE_REG_WRITE_LENGTH+1];

	writes[0] = PINNACLE_WRITE | (address & HOSTREG__31);
	writes[1] = value;
	ret_value = i2c_master_send(client, writes, PINNACLE_REG_WRITE_LENGTH+1);
	if (ret_value  < 0)
	{
		PRINTK(KERN_ERR "outreg: communication failure with error: %i.\r\n", ret_value);
		return false;
	}
	return true;
}

static bool get_interrupt_source(struct i2c_client *client, u8* lpIntSource)
{
	if (!inreg(client, HOSTREG__STATUS1, lpIntSource))
		return false;
	//PRINTK("get_interrupt_source: i2c source: 0x%02X\r\n", *lpIntSource);
	return true;
}

static bool clear_interrupt_source(struct i2c_client *client, u8 ucIntSource)
{
	u8 ucRegBuffer = 0;

	if (!inreg(client, HOSTREG__STATUS1, &ucRegBuffer))
		return false;
	ucRegBuffer &= ~ucIntSource;
	if (!outreg(client, HOSTREG__STATUS1, ucRegBuffer))
		return false;
	return true;
}

static bool pinnacle_busy_wait_for_complete(struct i2c_client *client)
{
	u32 	ulCount = PINNACLE_COMMAND_COMPLETE_WAIT;
	u8 		ucData;
	bool	ret;

	while ((ret = inreg(client, HOSTREG__STATUS1, &ucData)) && !(ucData & HOSTREG__STATUS1__COMMAND_COMPLETE) && ulCount--)
	{
		PRINTK(KERN_INFO "pinnacle_busy_wait_for_complete: HOSTREG__STATUS1: 0x%02X\r\n", ucData);
		mdelay(1);
	}

	if (!ret || !ulCount)
	{
		PRINTK(KERN_ERR "pinnacle_busy_wait_for_complete, failed getting DR in time.\n");
		return false;
	}
	return clear_interrupt_source(client, HOSTREG__STATUS1__COMMAND_COMPLETE);
}

static bool inreg_ext(struct i2c_client *client, u16 address, u8* lpData)
{
	u32 ulCounter = PINNACLE_EXT_COMMAND_WAIT;
	u8 	ucRegBuffer = HOSTREG__EREG_AXS_CTRL_READ;

	// Set address
	if (!outreg(client, HOSTREG__EXT_REG_AXS_ADDR_HIGH, (u8)(address >> 8)))
		return false;
	if (!outreg(client, HOSTREG__EXT_REG_AXS_ADDR_LOW, (u8)address))
		return false;

	// Start read (without post increment)
	if (!outreg(client, HOSTREG__EXT_REG_AXS_CTRL, HOSTREG__EREG_AXS_CTRL_READ))
		return false;

	// Wait for completion
	while (ucRegBuffer && ulCounter--)
	{
		if (!inreg(client, HOSTREG__EXT_REG_AXS_CTRL, &ucRegBuffer))
			return false;
		if (ucRegBuffer)
			mdelay(1);
	}

	// Clear the Command Complete bit
	if (!clear_interrupt_source(client, HOSTREG__STATUS1__COMMAND_COMPLETE))
		return true;

	if (!ulCounter)
		return false;

	// return the value
	if (!inreg(client, HOSTREG__EXT_REG_AXS_VALUE, lpData))
		return false;
	return true;
}

static bool outreg_ext(struct i2c_client *client, u16 address, u8 value)
{
	u32 ulCounter = PINNACLE_EXT_COMMAND_WAIT;
	u8 	ucRegBuffer = HOSTREG__EREG_AXS_CTRL_WRITE;

	// set value
    if (!outreg(client, HOSTREG__EXT_REG_AXS_VALUE, value))
    	return false;
	// Set address
	if (!outreg(client, HOSTREG__EXT_REG_AXS_ADDR_HIGH, (u8)(address >> 8)))
		return false;
	if (!outreg(client, HOSTREG__EXT_REG_AXS_ADDR_LOW, (u8)address))
		return false;
	// Start write
	if (!outreg(client, HOSTREG__EXT_REG_AXS_CTRL, HOSTREG__EREG_AXS_CTRL_WRITE))
		return false;

	// Wait for completion
	while (ucRegBuffer && ulCounter--)
	{
		if (!inreg(client, HOSTREG__EXT_REG_AXS_CTRL, &ucRegBuffer))
			return false;
		if (ucRegBuffer)
			mdelay(1);
	}

	// Clear the Command Complete bit
	if (!clear_interrupt_source(client, HOSTREG__STATUS1__COMMAND_COMPLETE))
		return false;

	if (!ulCounter)
		return false;
	return true;
}

static bool readreg(struct i2c_client *client, u16 usAddress, u8* lpData)
{
	bool bReturn;

	if (usAddress > HOSTREG__31)
		bReturn = inreg_ext(client, usAddress, lpData);
	else
		bReturn = inreg(client, (u8)usAddress, lpData);
	PRINTK(KERN_DEBUG "readreg: read %s from address 0x%04X value 0x%02X\r\n", (char*)(bReturn ? "successful" : "failed"), usAddress, *lpData);
	return bReturn;
}

static bool writereg(struct i2c_client *client, u16 usAddress, u8 ucData)
{
	bool bReturn;

	if (usAddress > HOSTREG__31)
		bReturn = outreg_ext(client, usAddress, ucData);
	else
		bReturn = outreg(client, (u8)usAddress, ucData);
	PRINTK(KERN_DEBUG "writereg: wrote %s to address 0x%04X value 0x%02X\r\n", (char*)(bReturn ? "successful" : "failed"), usAddress, ucData);
	return bReturn;
}

static bool i2cReceiveData(struct pinnacle_ts_priv *priv, struct pinnacle_touch_data* sample)
{
	u8	ucRegWrite[PINNACLE_REG_WRITE_LENGTH];
	u8	ucTouchData[PINNACLE_TOUCH_DATA_LENGTH];
	u32 i;
	int ret_value;

	//PRINTK("++i2cReceiveData\r\n");

	ucRegWrite[0] = PINNACLE_READ | HOSTREG__PACKETBYTE_5;
	ret_value = i2c_master_send(priv->client, ucRegWrite, PINNACLE_REG_WRITE_LENGTH);
	if (ret_value < 0)
	{
		printk(KERN_ERR "Error, while trying to receive i2c data (1, %i).\r\n", ret_value);
		return false;
	}

	//PRINTK(KERN_DEBUG "received data, ");
	ret_value = i2c_master_recv(priv->client, ucTouchData, PINNACLE_TOUCH_DATA_LENGTH);
	if (ret_value < 0)
	{
		printk(KERN_ERR "Error, while trying to receive i2c data (2, %i).\r\n", ret_value);
		return false;
	}
	for (i = 0; i < PINNACLE_TOUCH_DATA_LENGTH; i++)
	{
		PRINTK(KERN_DEBUG "buf[%u]:0X%.2X ", i, ucTouchData[i]);
	}
	PRINTK(KERN_DEBUG "\r\n");

	// calculate values
	sample->usXPos = (u16) (((ucTouchData[PINNACLE_TOUCH_DATA_OFFS + 2] << 8) & 0x0F00) | ucTouchData[PINNACLE_TOUCH_DATA_OFFS + 0]);
	sample->usYPos = (u16) (((ucTouchData[PINNACLE_TOUCH_DATA_OFFS + 2] << 4) & 0x0F00) | ucTouchData[PINNACLE_TOUCH_DATA_OFFS + 1]);
	sample->usZPos = (u16) (ucTouchData[PINNACLE_TOUCH_DATA_OFFS + 3] & 0x7F);
	sample->usFlags = 0;

	if (ucTouchData[PINNACLE_TOUCH_DATA_OFFS + 3] & HOSTREG__PACKETBYTE_5_TOUCH)
		sample->usFlags |= PINNACLE_TOUCH_DATA_FLAGS_TOUCH;

	if (sample->usXPos >= GUL_MAX_X_POS)
		sample->usXPos = (u16)(GUL_MAX_X_POS - GUL_MIN_X_POS);
	else if (sample->usXPos < GUL_MIN_X_POS)
		sample->usXPos = 0;
	else
		sample->usXPos = sample->usXPos - (u16)GUL_MIN_X_POS;

	if (sample->usYPos >= GUL_MAX_Y_POS)
		sample->usYPos = (u16)(GUL_MAX_Y_POS - GUL_MIN_Y_POS);
	else if (sample->usYPos < GUL_MIN_Y_POS)
		sample->usYPos = 0;
	else
		sample->usYPos = sample->usYPos - (u16)GUL_MIN_Y_POS;
	PRINTK("received data: xpos = %u ypos = %u zpos = %u %s\r\n",
		sample->usXPos,
		sample->usYPos,
		sample->usZPos,
		(char*)((sample->usFlags & PINNACLE_TOUCH_DATA_FLAGS_TOUCH) ? "TOUCH DOWN" : "TOUCH UP"));

	if (!clear_interrupt_source(priv->client, HOSTREG__STATUS1__DATA_READY))
		return false;
	//PRINTK("--i2cReceiveData\r\n");
	return true;
}

static bool PenIsDown(struct pinnacle_ts_priv *priv, struct pinnacle_touch_data* sample)
{
	static bool bFirstRun = true;
	static bool bReturn = false;
	static unsigned int ulUpHysteresis = 0;
	static struct pinnacle_touch_data touchData;

	if (bFirstRun)
	{
		ulUpHysteresis = GUC_UP_HYSTERESIS;
		bFirstRun = false;
	}

	//PRINTK("++PenIsDown!\r\n");
	if (!i2cReceiveData(priv, sample))
		return false;

	if (!bReturn && (sample->usFlags & PINNACLE_TOUCH_DATA_FLAGS_TOUCH) && (PINNACLE_MIN_PRESSURE < sample->usZPos))
	{
		memcpy(&touchData, sample, sizeof(struct pinnacle_touch_data));
		bReturn = true;
	}
	else if (bReturn && (!(sample->usFlags & PINNACLE_TOUCH_DATA_FLAGS_TOUCH) || !(PINNACLE_MIN_PRESSURE < sample->usZPos)))
	{
		if (ulUpHysteresis)
		{
			memcpy(sample, &touchData, sizeof(struct pinnacle_touch_data));
			bReturn = true;
			ulUpHysteresis--;
		}
		else
		{
			bReturn = false;
		}
	}
	else
	{
		if (bReturn)
			memcpy(&touchData, sample, sizeof(struct pinnacle_touch_data));
		ulUpHysteresis = GUC_UP_HYSTERESIS;
	}
	//PRINTK("--PenIsDown!\r\n");
	return bReturn;
}

static bool check_cal_value(struct pinnacle_ts_priv *priv)
{
	static u16 		sOldValues[PINNACLE_CAL_REG_LENGTH];
	static bool 	oldInitialized = false;
	u32 			ulCounter;
	u16* 			lpValue;
	u32				uiDiff;

	//printk("++check_cal_value.\n");

	if (!oldInitialized)
		lpValue = sOldValues;
	else
		lpValue = priv->lpCalData;

	for (ulCounter = 0; ulCounter < PINNACLE_CAL_REG_LENGTH; ulCounter++)
	{
		if (!readreg(priv->client, (u16)(PINNACLE_CAL_REG_START + (ulCounter<<1)), (u8*)(&lpValue[ulCounter])+1))
		{
			printk(KERN_ERR "check_cal_value failed.\r\n");
			return false;
		}
		if (!readreg(priv->client, (u16)(PINNACLE_CAL_REG_START + (ulCounter<<1) + 1), (u8*)(&lpValue[ulCounter])))
		{
			printk(KERN_ERR "check_cal_value failed.\r\n");
			return false;
		}
		//printk("lpValue[%u]: 0x%04X.\n", ulCounter, lpValue[ulCounter]);
	}

	if (!oldInitialized)
	{
		oldInitialized = true;
		return false;
	}

	for (ulCounter = 0; ulCounter < PINNACLE_CAL_REG_LENGTH; ulCounter++)
	{
		uiDiff = (sOldValues[ulCounter] > priv->lpCalData[ulCounter]) ? (sOldValues[ulCounter] - priv->lpCalData[ulCounter]) : (priv->lpCalData[ulCounter] - sOldValues[ulCounter]);
		if (uiDiff > GUL_MIN_CAL_DIFF_VALUE)
		{
			printk(KERN_ERR "check_cal_value failed at %u 0x%04X <-> 0x%04X.\r\n", ulCounter, sOldValues[ulCounter], priv->lpCalData[ulCounter]);
			for (ulCounter = 0; ulCounter < PINNACLE_CAL_REG_LENGTH; ulCounter++)
				sOldValues[ulCounter] = priv->lpCalData[ulCounter];
			return false;
		}
	}
	printk("Valid hardware configuration found\r\n");
	priv->bControllerCalibrated = true;
	return true;
}

static bool initialize_iic_link(struct pinnacle_ts_priv *priv)
{
	struct pinnacle_touch_config_slot* lpTouchConfig = NULL;
	struct i2c_client *client = priv->client;
	u32 ulElements = 0;
	u32 i;
	u8 s;
	u32 ulCalErrorCnt = 0;
	bool bReturn = false;
	bool bZIdleModified = false;

#if TOGGLE_DEBUG_PIN_ON_CAL
#endif
	priv->ucDynMinimumValidSamples = GUC_MINIMUM_VALID_SAMPLES;
	priv->ucDynDownHysteresis = GUC_DOWN_HYSTERESIS;
	priv->ucValidSamples = GUC_MINIMUM_VALID_SAMPLES;

	PRINTK("Initializing the controller... ");
    if (!setup_touch_config_slots(priv, &lpTouchConfig, &ulElements))
    {
		printk(KERN_ERR "InitializeI2CLink, couldn't initialize config structure.\r\n");
		lpTouchConfig = NULL;
    	goto CleanUp;
	}
	//printk("found %u elements.\n", ulElements);

	for (i = 0; i < ulElements; i++)
	{
		if (lpTouchConfig[i].ucReadDataCount)
		{
			u8 ucRegBuffer;

			for (s = 0; s < lpTouchConfig[i].ucReadDataCount; s++)
			{
				if (!readreg(client, lpTouchConfig[i].usAddress + s, &ucRegBuffer))
				{
					PRINTK(KERN_WARNING "InitializeI2CLink, failed reading from address 0x%x.\r\n", lpTouchConfig[i].usAddress + s);
					goto CleanUp;
				}

				if (lpTouchConfig[i].bVerification && (lpTouchConfig[i].lpReadData[s] != ucRegBuffer))
				{
					PRINTK(KERN_WARNING "Verification of address 0x%04X failed at position %u:%u, read 0x%02X but expected 0x%02X\r\n",
						lpTouchConfig[i].usAddress, i, s, ucRegBuffer, lpTouchConfig[i].lpReadData[s]);
				}
				lpTouchConfig[i].lpReadData[s] = ucRegBuffer;
			}
		}

		if (lpTouchConfig[i].ucWriteDataCount)
		{
			if (lpTouchConfig[i].ucReadDataCount)
			{
				for (s = 0; s < lpTouchConfig[i].ucReadDataCount; s++)
					lpTouchConfig[i].lpWriteData[s] |= (lpTouchConfig[i].lpReadData[s] & lpTouchConfig[i].lpDataMask[s]);
			}

			for (s = 0; s < lpTouchConfig[i].ucWriteDataCount; s++)
			{
				if ((HOSTREG__CALCONFIG1 == (lpTouchConfig[i].usAddress + s)) &&
					(HOSTREG__CALCONFIG1__CALIBRATE & lpTouchConfig[i].lpWriteData[s]))
				{
#if TOGGLE_DEBUG_PIN_ON_CAL
					// to do
#endif
#if DISABLE_DISPLAY_WHILE_CALIBRATING
					if (0 == ulCalErrorCnt)
					{
						int i;

						console_lock();
						for (i = 0; i < num_registered_fb; i++) {
							fb_blank(registered_fb[i], FB_BLANK_POWERDOWN);
						}
						console_unlock();

						if (priv->pdata->guf_set_lcd_power)
							priv->pdata->guf_set_lcd_power(0);
						mdelay(500);
					}
#endif
				}
				else if (HOSTREG__ZIDLE == (lpTouchConfig[i].usAddress + s))
				{
					lpTouchConfig[i].lpWriteData[s] = (u8)(lpTouchConfig[i].lpWriteData[s] + GUC_UP_HYSTERESIS);
					PRINTK(KERN_INFO "InitializeI2CLink, corrected ZIDLE to %u.\r\n", lpTouchConfig[i].lpWriteData[s]);
					bZIdleModified = true;
				}

				if (!writereg(client, lpTouchConfig[i].usAddress + s, lpTouchConfig[i].lpWriteData[s]))
				{
					PRINTK(KERN_WARNING "InitializeI2CLink, failed writing value 0x%x to address 0x%x.\r\n", lpTouchConfig[i].lpWriteData[s], lpTouchConfig[i].usAddress + s);
					goto CleanUp;
				}
			}
		}

		if (lpTouchConfig[i].dwSleep)
			mdelay(lpTouchConfig[i].dwSleep);

		if (lpTouchConfig[i].ucWaitForInterrupt)
		{
			u8 ucStatus;

			if (!get_interrupt_source(client, &ucStatus))
			{
				PRINTK(KERN_WARNING "InitializeI2CLink, failed getting interrupt source (1).\r\n");
				goto CleanUp;
			}

			while (!(ucStatus & lpTouchConfig[i].ucWaitForInterrupt))
			{
				mdelay(1);
				if (!get_interrupt_source(client, &ucStatus))
				{
					PRINTK(KERN_WARNING "InitializeI2CLink, failed getting interrupt source (2).\r\n");
					goto CleanUp;
				}
			}

			if (!clear_interrupt_source(client, lpTouchConfig[i].ucWaitForInterrupt))
			{
				PRINTK(KERN_WARNING "InitializeI2CLink, failed cleating interrupt source 0x%x.\r\n", lpTouchConfig[i].ucWaitForInterrupt);
				goto CleanUp;
			}
			for (s = 0; s < lpTouchConfig[i].ucWriteDataCount; s++)
			{
				if ((HOSTREG__CALCONFIG1 == (lpTouchConfig[i].usAddress + s)) &&
					(HOSTREG__CALCONFIG1__CALIBRATE & lpTouchConfig[i].lpWriteData[s]))
				{
					if (GUL_CHECK_MAX_VAL_AFTER_CAL && !check_cal_value(priv))
					{
						if (GUL_CHECK_MAX_VAL_AFTER_CAL == ++ulCalErrorCnt)
						{
							PRINTK(KERN_ERR "Calibration matrix is corrupted and couldn't be corrected after %u tries. The touch is very likely not useable\r\n", GUL_CHECK_MAX_VAL_AFTER_CAL);
							ulCalErrorCnt = 0;
						}
						else
						{
							i--; //repeat the calibration step of the touch configuration;
						}
					}
					else
					{
						ulCalErrorCnt = 0;
					}
#if DISABLE_DISPLAY_WHILE_CALIBRATING
					if (0 == ulCalErrorCnt)
					{
						if (priv->pdata->guf_set_lcd_power)
							priv->pdata->guf_set_lcd_power(1);

						console_lock();
						for (i = 0; i < num_registered_fb; i++) {
							fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
						}
						console_unlock();
					}
#endif
#if TOGGLE_DEBUG_PIN_ON_CAL
					// to do
#endif
				}
			}
		}
	}

	if (!bZIdleModified)
	{
		PRINTK(KERN_INFO "InitializeI2CLink, standard initialization of ZIDLE to %u ", GUC_UP_HYSTERESIS + 2);
		if (!writereg(priv->client, HOSTREG__ZIDLE, GUC_UP_HYSTERESIS + 2))
		{
			PRINTK(KERN_INFO "failed.\r\n");
			goto CleanUp;
		}
		PRINTK(KERN_INFO "successful.\r\n");
	}

	bReturn = true;
CleanUp:
	PRINTK(KERN_DEBUG "done.\r\n");
#if TOGGLE_DEBUG_PIN_ON_CAL
	// to do
#endif
	if (lpTouchConfig)
		kfree(lpTouchConfig);
	pr_info("GUF-PINNACLE: %s initialization.\n",(char*)(bReturn ? "successfully finished" : "failed"));
	return bReturn;
}

static TOUCH_PANEL_SAMPLE_FLAGS pinnacle_ts_filter_samples(struct pinnacle_ts_priv *priv, u16* uiSamples, u32* pSample, bool bDynamicFilterUpdate)
{
    int i;
    int nValidSamples = 0;
    int mean=0;
    unsigned int maxDist=0;
    int CandidateForDiscard = -1;

    // compute mean value
    for (i=0; i < priv->ucDynDownHysteresis; i++)
	{
        if (uiSamples[i] != INVALID_ADC_VAL)
		{
            nValidSamples++;
            mean += uiSamples[i];
        }
	}

    if (nValidSamples < priv->ucDynMinimumValidSamples)
        return TouchSampleIgnore;

    mean /= nValidSamples;

    // look for the farthest position
    for (i=0; i < priv->ucDynDownHysteresis; i++)
	{
        if (uiSamples[i] != INVALID_ADC_VAL)
        {
            unsigned int dist = (mean - uiSamples[i]) * (mean - uiSamples[i]);
            if (dist > maxDist)
            {
                maxDist = dist;
                CandidateForDiscard = i;
            }
		}
	}

    if (maxDist > (GUL_MAX_DISTANCE*GUL_MAX_DISTANCE))
    {
        uiSamples[CandidateForDiscard] = INVALID_ADC_VAL;
        return pinnacle_ts_filter_samples(priv, uiSamples, pSample, bDynamicFilterUpdate);
    }

	if (bDynamicFilterUpdate)
	{
		if (priv->ucValidSamples == GUC_MINIMUM_VALID_HOLD)
		{
			if (priv->ucDynMinimumValidSamples > GUC_MINIMUM_VALID_SUSTAIN)
			{
				if (GUC_MINIMUM_VALID_DECAY > priv->ucDynMinimumValidSamples)
				{
					priv->ucDynMinimumValidSamples = priv->ucDynMinimumValidSamples - GUC_MINIMUM_VALID_DECAY;
					if (priv->ucDynMinimumValidSamples < GUC_MINIMUM_VALID_SUSTAIN)
						priv->ucDynMinimumValidSamples = GUC_MINIMUM_VALID_SUSTAIN;
				}
				else
				{
					priv->ucDynMinimumValidSamples = GUC_MINIMUM_VALID_SUSTAIN;
				}
			}
		}
		else
		{
			priv->ucValidSamples++;
		}
	}

	*pSample = mean;
	return TouchSampleValidFlag;
}

static TOUCH_PANEL_SAMPLE_FLAGS pinnacle_ts_sample_touch_screen(struct pinnacle_ts_priv *priv, struct pinnacle_touch_data* sample, u32 *x, u32 *y)
{
	u32 uiCounter;
    TOUCH_PANEL_SAMPLE_FLAGS TmpStateFlags;
    static u8 ucValidSampleBuffer = 0;

    PRINTK(KERN_DEBUG "++pinnacle_ts_sample_touch_screen\r\n");

    TmpStateFlags = TouchSampleDownFlag;
    if (ucValidSampleBuffer < priv->ucDynMinimumValidSamples)
    	ucValidSampleBuffer = priv->ucDynMinimumValidSamples;

    for (uiCounter = 1; uiCounter < priv->ucDynDownHysteresis; uiCounter++)
	{
    	priv->lpXSamples[uiCounter-1] = priv->lpXSamples[uiCounter];
    	priv->lpYSamples[uiCounter-1] = priv->lpYSamples[uiCounter];
	}
    priv->lpXSamples[priv->ucDynDownHysteresis-1] = sample->usXPos;
    priv->lpYSamples[priv->ucDynDownHysteresis-1] = sample->usYPos;
    memcpy(priv->lpBufferSamples, priv->lpXSamples, priv->ucDynDownHysteresis * sizeof(u16));
	TmpStateFlags |= pinnacle_ts_filter_samples(priv, priv->lpBufferSamples, x, false);
    if (!(TmpStateFlags & TouchSampleIgnore))
	{
    	memcpy(priv->lpBufferSamples, priv->lpYSamples, priv->ucDynDownHysteresis * sizeof(u16));
		TmpStateFlags |= pinnacle_ts_filter_samples(priv, priv->lpBufferSamples, y, true);
	}

	if (ucValidSampleBuffer > priv->ucDynMinimumValidSamples)
	{
		u32 uiCounter2 = ucValidSampleBuffer - priv->ucDynMinimumValidSamples;

		for (; uiCounter2; uiCounter2--)
		{
			for (uiCounter = 1; uiCounter < priv->ucDynDownHysteresis; uiCounter++)
			{
				priv->lpXSamples[uiCounter-1] = priv->lpXSamples[uiCounter];
				priv->lpYSamples[uiCounter-1] = priv->lpYSamples[uiCounter];
			}
		}
		priv->ucDynDownHysteresis = priv->ucDynDownHysteresis - (ucValidSampleBuffer - priv->ucDynMinimumValidSamples);
		ucValidSampleBuffer = priv->ucDynMinimumValidSamples;
	}

	PRINTK(KERN_DEBUG "--pinnacle_ts_sample_touch_screen, x = %u, y = %u, touch flags: 0x%X\r\n", *x, *y, TmpStateFlags);
    return (TmpStateFlags);
}

static void pinnacle_ts_read(struct work_struct *work)
{
	bool bTouchDown = false;
	u8 ints = 0;
	u32 x = 0;
	u32 y = 0;
	TOUCH_PANEL_SAMPLE_FLAGS StateFlags;
	struct pinnacle_touch_data sample;
	struct pinnacle_ts_priv *priv = container_of(work, struct pinnacle_ts_priv, work);

	//printk("++pinnacle_ts_read\r\n");
	if (get_interrupt_source(priv->client, &ints))
	{
		if (ints & HOSTREG__STATUS1__DATA_READY)
		{
			if (PenIsDown(priv, &sample))
			{
				//printk("pinnacle_ts_read, pen is down, filtering\r\n");
				StateFlags = pinnacle_ts_sample_touch_screen(priv, &sample, &x, &y);
				if (StateFlags & TouchSampleIgnore)
				{
					// resent the old value we didn't get a new value but keep the button down
					sample.usXPos = priv->usXBuffer;
					sample.usYPos = priv->usYBuffer;
					sample.usZPos = priv->usZBuffer;
				}
				else
				{
					priv->usXBuffer = (u16)x;
					sample.usXPos = (u16)x;
					priv->usYBuffer = (u16)y;
					sample.usYPos = (u16)y;
					priv->usZBuffer = sample.usZPos;
					bTouchDown = true;
					//printk("(%d/%d/%d/DOWN)\n", sample.usXPos, sample.usYPos, sample.usZPos);
				}
				if (bTouchDown || priv->bOldTouchDown)
					input_report_key(priv->input, BTN_TOUCH, 1);
			}
			else
			{
				u32 uiCounter;
				//printk("pinnacle_ts_read, pen is up, resetting filters\r\n");

				priv->ucValidSamples = 0;
				priv->ucDynMinimumValidSamples = GUC_MINIMUM_VALID_SAMPLES;
				priv->ucDynDownHysteresis = GUC_DOWN_HYSTERESIS;
				for (uiCounter = 0; uiCounter < GUC_DOWN_HYSTERESIS; uiCounter++)
				{
					priv->lpXSamples[uiCounter] = INVALID_ADC_VAL;
					priv->lpYSamples[uiCounter] = INVALID_ADC_VAL;
				}
				sample.usXPos = priv->usXBuffer;
				sample.usYPos = priv->usYBuffer;
				sample.usZPos = priv->usZBuffer;
				bTouchDown = false;
				//printk("(%d/%d/%d/UP)\n", sample.usXPos, sample.usYPos, sample.usZPos);
				if (priv->bOldTouchDown)
					input_report_key(priv->input, BTN_TOUCH, 0);
			}
			if (bTouchDown || priv->bOldTouchDown)
			{
				input_report_abs(priv->input, ABS_X, sample.usXPos);
				input_report_abs(priv->input, ABS_Y, sample.usYPos);
				input_report_abs(priv->input, ABS_PRESSURE, sample.usZPos);
				//input_report_abs(priv->input, ABS_RX, sample.usXPos);
				//input_report_abs(priv->input, ABS_RY, sample.usYPos);
				//input_report_abs(priv->input, ABS_RZ, sample.usZPos);
				//printk("sending touch\n");
				input_sync(priv->input);
			}
			priv->bOldTouchDown = bTouchDown;
			//udelay(10); //???
		}
		if (ints & ~HOSTREG__STATUS1__DATA_READY)
		{
			PRINTK("Received an additional interrupt besides a data interrupt (0x%X). Clearing it before enabling the interrupt again.\n", ints & ~HOSTREG__STATUS1__DATA_READY);
			if (!clear_interrupt_source(priv->client, ints & ~HOSTREG__STATUS1__DATA_READY))
			{
				PRINTK("Failed clearing the additional interrupts (0x%X). Give the system some time to life before enabling the interrupt again.\n", ints & ~HOSTREG__STATUS1__DATA_READY);
				msleep(100);
			}
		}
	}
	else
	{
		PRINTK("Couldn't receive interrupt status, give the system some time to life before enabling interrupt again.");
		msleep(100);
	}
	//printk("pinnacle_ts_read, enable irq.\r\n");
	enable_irq(priv->irq);
	//printk("--pinnacle_ts_read\r\n");
}

// driver functions
static irqreturn_t pinnacle_ts_isr(int irq, void *dev_id)
{
	int iReturn;
	struct pinnacle_ts_priv *priv = dev_id;

	//printk("++pinnacle_ts_isr!\r\n");
	 /* postpone I2C transactions as we are atomic */
	iReturn = schedule_work(&priv->work);
	if (iReturn < 0)
	{
		PRINTK(KERN_DEBUG "pinnacle_ts_isr, couldn't schedule worker thread because of error %d\r\n", iReturn);
	}
	disable_irq_nosync(priv->irq);
	if (priv->pdata->irq_flags & IRQF_SHARED)
		return IRQ_NONE;
	return IRQ_HANDLED;
}

static ssize_t pinnacle_ts_bin_read(struct file * f, struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	size_t uiCounter;
	struct pinnacle_ts_priv *priv = dev_get_drvdata(container_of(kobj, struct device, kobj));

	if (!priv)
		return -EIO;

	if (count & 0x01)
		return -EIO;

	if ((off + count) > (PINNACLE_CAL_REG_LENGTH * PINNACLE_CAL_REG_SIZE))
		return -EIO;

	//printk("pinnacle_ts_bin_read: readinfg %d bytes at offset %u from 0x%x.\n", count, (u32)off, (u32)priv->lpCalData);
	for (uiCounter = 0; uiCounter < (count >> 1); uiCounter++)
	{
		buf[(uiCounter<<1)] = *((char*)(priv->lpCalData) + (off + (uiCounter<<1) + 1));
		buf[(uiCounter<<1) + 1] = *((char*)(priv->lpCalData) + (off + (uiCounter<<1)));
	}
	return count;
}


static ssize_t pinnacle_ts_read_hysteresis_up(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ucUpHysteresis);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_hysteresis_up(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ucUpHysteresis = (u8)simple_strtoul(buf, NULL, 0);
	//printk("ucUpHysteresis: %u\n", priv->ucUpHysteresis);
	return strlen(buf)+1;
}
static DEVICE_ATTR(hysteresis_up, S_IRUGO | S_IWUGO, pinnacle_ts_read_hysteresis_up, pinnacle_ts_write_hysteresis_up);

static ssize_t pinnacle_ts_read_hysteresis_down(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ucDownHysteresis);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_hysteresis_down(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ucDownHysteresis = (u8)simple_strtoul(buf, NULL, 0);
	//printk("ucDownHysteresis: %u\n", priv->ucDownHysteresis);
	return strlen(buf)+1;
}
static DEVICE_ATTR(hysteresis_down, S_IRUGO | S_IWUGO, pinnacle_ts_read_hysteresis_down, pinnacle_ts_write_hysteresis_down);

static ssize_t pinnacle_ts_read_min_valid_samples(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ucMinimumValidSamples);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_min_valid_samples(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ucMinimumValidSamples = (u8)simple_strtoul(buf, NULL, 0);
	//printk("ucMinimumValidSamples: %u\n", priv->ucMinimumValidSamples);
	return strlen(buf)+1;
}
static DEVICE_ATTR(min_valid_samples, S_IRUGO | S_IWUGO, pinnacle_ts_read_min_valid_samples, pinnacle_ts_write_min_valid_samples);

static ssize_t pinnacle_ts_read_min_valid_hold(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ucMinimumValidHold);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_min_valid_hold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ucMinimumValidHold = (u8)simple_strtoul(buf, NULL, 0);
	//printk("ucMinimumValidHold: %u\n", priv->ucMinimumValidHold);
	return strlen(buf)+1;
}
static DEVICE_ATTR(min_valid_hold, S_IRUGO | S_IWUGO, pinnacle_ts_read_min_valid_hold, pinnacle_ts_write_min_valid_hold);

static ssize_t pinnacle_ts_read_min_valid_sustain(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ucMinimumValidSustain);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_min_valid_sustain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ucMinimumValidSustain = (u8)simple_strtoul(buf, NULL, 0);
	//printk("ucMinimumValidSustain: %u\n", priv->ucMinimumValidSustain);
	return strlen(buf)+1;
}
static DEVICE_ATTR(min_valid_sustain, S_IRUGO | S_IWUGO, pinnacle_ts_read_min_valid_sustain, pinnacle_ts_write_min_valid_sustain);

static ssize_t pinnacle_ts_read_min_valid_decay(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ucMinimumValidDecay);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_min_valid_decay(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ucMinimumValidDecay = (u8)simple_strtoul(buf, NULL, 0);
	//printk("ucMinimumValidDecay: %u\n", priv->ucMinimumValidDecay);
	return strlen(buf)+1;
}
static DEVICE_ATTR(min_valid_decay, S_IRUGO | S_IWUGO, pinnacle_ts_read_min_valid_decay, pinnacle_ts_write_min_valid_decay);

static ssize_t pinnacle_ts_read_max_distance(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulMaxDistance);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_max_distance(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulMaxDistance = simple_strtoul(buf, NULL, 0);
	//printk("ulMaxDistance: %u\n", priv->ulMaxDistance);
	return strlen(buf)+1;
}
static DEVICE_ATTR(max_distance, S_IRUGO | S_IWUGO, pinnacle_ts_read_max_distance, pinnacle_ts_write_max_distance);

static ssize_t pinnacle_ts_read_max_x(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulMaxXPos);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_max_x(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulMaxXPos = simple_strtoul(buf, NULL, 0);
	//printk("ulMaxXPos: %u\n", priv->ulMaxXPos);
	return strlen(buf)+1;
}
static DEVICE_ATTR(max_x, S_IRUGO | S_IWUGO, pinnacle_ts_read_max_x, pinnacle_ts_write_max_x);

static ssize_t pinnacle_ts_read_min_x(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulMinXPos);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_min_x(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulMinXPos = simple_strtoul(buf, NULL, 0);
	//printk("ulMinXPos: %u\n", priv->ulMinXPos);
	return strlen(buf)+1;
}
static DEVICE_ATTR(min_x, S_IRUGO | S_IWUGO, pinnacle_ts_read_min_x, pinnacle_ts_write_min_x);

static ssize_t pinnacle_ts_read_max_y(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulMaxYPos);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_max_y(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulMaxYPos = simple_strtoul(buf, NULL, 0);
	//printk("ulMaxYPos: %u\n", priv->ulMaxYPos);
	return strlen(buf)+1;
}
static DEVICE_ATTR(max_y, S_IRUGO | S_IWUGO, pinnacle_ts_read_max_y, pinnacle_ts_write_max_y);

static ssize_t pinnacle_ts_read_min_y(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulMinYPos);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_min_y(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulMinYPos = simple_strtoul(buf, NULL, 0);
	//printk("ulMinYPos: %u\n", priv->ulMinYPos);
	return strlen(buf)+1;
}
static DEVICE_ATTR(min_y, S_IRUGO | S_IWUGO, pinnacle_ts_read_min_y, pinnacle_ts_write_min_y);

static ssize_t pinnacle_ts_read_max_cal_diff(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulMinCalDiffValue);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_max_cal_diff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulMinCalDiffValue = simple_strtoul(buf, NULL, 0);
	//printk("ulMinCalDiffValue: %u\n", priv->ulMinCalDiffValue);
	return strlen(buf)+1;
}
static DEVICE_ATTR(max_cal_diff, S_IRUGO | S_IWUGO, pinnacle_ts_read_max_cal_diff, pinnacle_ts_write_max_cal_diff);

static ssize_t pinnacle_ts_read_max_cal_checks(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->ulCheckMaxValAfterCal);
	return strlen(buf)+1;
}
static ssize_t pinnacle_ts_write_max_cal_checks(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	if (priv->bDriverOpened)
		return -EIO;

	priv->ulCheckMaxValAfterCal = simple_strtoul(buf, NULL, 0);
	//printk("ulCheckMaxValAfterCal: %u\n", priv->ulCheckMaxValAfterCal);
	return strlen(buf)+1;
}
static DEVICE_ATTR(max_cal_checks, S_IRUGO | S_IWUGO, pinnacle_ts_read_max_cal_checks, pinnacle_ts_write_max_cal_checks);

static ssize_t pinnacle_ts_read_calibrated(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->bControllerCalibrated ? 1 : 0);
	return strlen(buf)+1;
}
static DEVICE_ATTR(calibrated, S_IRUGO, pinnacle_ts_read_calibrated, NULL);

static ssize_t pinnacle_ts_read_opened(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(to_input_dev(dev));

	snprintf(buf, 256, "%u\n", priv->bDriverOpened ? 1 : 0);
	return strlen(buf)+1;
}
static DEVICE_ATTR(opened, S_IRUGO, pinnacle_ts_read_opened, NULL);

static int pinnacle_ts_open(struct input_dev *dev)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(dev);
	u32 uiCounter;

	if (priv->bDriverOpened)
	{
		dev_err(&priv->client->dev, "device is already opened.\n");
		return -EEXIST;
	}

	priv->lpXSamples = (u16*) kzalloc(GUC_DOWN_HYSTERESIS * sizeof(u16), GFP_KERNEL);
	if (!priv->lpXSamples)
	{
		dev_err(&priv->client->dev, "failed to allocate driver data\n");
		return -EIO;
	}
	for (uiCounter = 0; uiCounter < GUC_DOWN_HYSTERESIS; uiCounter++)
		priv->lpXSamples[uiCounter] = INVALID_ADC_VAL;
	priv->lpYSamples = (u16*) kzalloc(GUC_DOWN_HYSTERESIS * sizeof(u16), GFP_KERNEL);
	if (!priv->lpYSamples)
	{
		dev_err(&priv->client->dev, "failed to allocate driver data\n");
		kfree(priv->lpXSamples);
		return -EIO;
	}
	for (uiCounter = 0; uiCounter < GUC_DOWN_HYSTERESIS; uiCounter++)
		priv->lpYSamples[uiCounter] = INVALID_ADC_VAL;
	priv->lpBufferSamples = (u16*) kzalloc(GUC_DOWN_HYSTERESIS * sizeof(u16), GFP_KERNEL);
	if (!priv->lpBufferSamples)
	{
		dev_err(&priv->client->dev, "failed to allocate driver data\n");
		kfree(priv->lpXSamples);
		kfree(priv->lpYSamples);
		return -EIO;
	}

	if (!priv->bDriverInitialized)
	{
		if (!pinnacle_busy_wait_for_complete(priv->client))
		{
			dev_err(&priv->client->dev, "failed receiving a DR.\n");
			return -EIO;
		}

		if (!initialize_iic_link(priv))
		{
			dev_err(&priv->client->dev, "failed to initialize touch controller.\n");
			return -EIO;
		}
		priv->bDriverInitialized = true;
	}

	priv->bDriverOpened = true;
	PRINTK(KERN_INFO "pinnacle_ts_open. priv->irq: 0x%X -> %d\r\n", priv->irq, gpio_get_value(priv->pdata->gpio_int_number));
	enable_irq(priv->irq);
	return 0;
}

static void pinnacle_ts_close(struct input_dev *dev)
{
	struct pinnacle_ts_priv *priv = input_get_drvdata(dev);

	PRINTK(KERN_DEBUG "++pinnacle_ts_close.\r\n");
	disable_irq(priv->irq);
	kfree(priv->lpXSamples);
	kfree(priv->lpYSamples);
	kfree(priv->lpBufferSamples);
	priv->bDriverOpened = false;
}

static int __devinit pinnacle_ts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct pinnacle_ts_priv *priv;
	struct input_dev *input = NULL;
	int err = -ENOMEM;

	// check for i2c functionality
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		dev_err(&client->dev, "iic functionality not supported\n");
		return -EIO;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
	{
		dev_err(&client->dev, "failed to allocate driver data\n");
		goto err0;
	}
	priv->ucUpHysteresis = PINNACLE_UP_HYSTERESIS;
	priv->ucDownHysteresis = PINNACLE_DOWN_HYSTERESIS;
	priv->ucMinimumValidSamples = PINNACLE_MINIMUM_VALID_SAMPLES;
	priv->ucMinimumValidHold = PINNACLE_MINIMUM_VALID_HOLD;
	priv->ucMinimumValidSustain = PINNACLE_MINIMUM_VALID_SUSTAIN;
	priv->ucMinimumValidDecay = PINNACLE_MINIMUM_VALID_DECAY;
	priv->ulMaxDistance = PINNACLE_MAX_DISTANCE;
	priv->ulMaxXPos = PINNACLE_MAX_X_POS;
	priv->ulMinXPos = PINNACLE_MIN_X_POS;
	priv->ulMaxYPos = PINNACLE_MAX_Y_POS;
	priv->ulMinYPos = PINNACLE_MIN_Y_POS;
	priv->ulMinCalDiffValue = PINNACLE_MIN_CAL_DIFF_VALUE;
	priv->ulCheckMaxValAfterCal = PINNACLE_CHECK_MAX_VAL_AFTER_CAL;
	priv->lpCalData = (u16*) kzalloc(PINNACLE_CAL_REG_LENGTH * PINNACLE_CAL_REG_SIZE, GFP_KERNEL);
	if (!priv->lpCalData)
	{
		dev_err(&client->dev, "failed to allocate driver data\n");
		goto err1;
	}
	//printk("allocated lpCalData: 0x%x.\n", (u32)priv->lpCalData);
	priv->ucDynDownHysteresis = 0;
	priv->ucDynMinimumValidSamples = 0;
	priv->ucValidSamples = 0;
	priv->bControllerCalibrated = false;
	priv->bDriverOpened = false;
	priv->bDriverInitialized = false;
	priv->bOldTouchDown = false;
	priv->usXBuffer = 0;
	priv->usYBuffer = 0;
	priv->usZBuffer = 0;
	mutex_init(&priv->mutex);

	input = input_allocate_device();
	if (!input)
	{
		dev_err(&client->dev, "failed to allocate input device.\n");
		goto err1;
	}

    if (device_create_file(&client->dev, &dev_attr_hysteresis_up) ||
        device_create_file(&client->dev, &dev_attr_hysteresis_down) ||
        device_create_file(&client->dev, &dev_attr_min_valid_samples) ||
        device_create_file(&client->dev, &dev_attr_min_valid_hold) ||
        device_create_file(&client->dev, &dev_attr_min_valid_sustain) ||
        device_create_file(&client->dev, &dev_attr_min_valid_decay) ||
        device_create_file(&client->dev, &dev_attr_max_distance) ||
        device_create_file(&client->dev, &dev_attr_max_x) ||
        device_create_file(&client->dev, &dev_attr_min_x) ||
        device_create_file(&client->dev, &dev_attr_max_y) ||
        device_create_file(&client->dev, &dev_attr_min_y) ||
        device_create_file(&client->dev, &dev_attr_max_cal_diff) ||
        device_create_file(&client->dev, &dev_attr_max_cal_checks) ||
        device_create_file(&client->dev, &dev_attr_calibrated) ||
        device_create_file(&client->dev, &dev_attr_opened))
    {
	   dev_err(&client->dev, "failed creating bin file in sysfs.\n");
       goto err1;
	}
	sysfs_bin_attr_init(&priv->sysfs_cal_data);
	priv->sysfs_cal_data.attr.name = "cal_data";
	priv->sysfs_cal_data.attr.mode = S_IRUGO;
	priv->sysfs_cal_data.read = pinnacle_ts_bin_read;
	priv->sysfs_cal_data.size = PINNACLE_CAL_REG_LENGTH * sizeof(u16);
	err = sysfs_create_bin_file(&client->dev.kobj, &priv->sysfs_cal_data);
	if (err)
	{
		dev_err(&client->dev, "failed creating bin file in sysfs.\n");
		goto err1;
	}

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS); //| BIT_MASK(EV_REL);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input, ABS_X, 0, GUL_MAX_X_POS, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, GUL_MAX_Y_POS, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, Z_MAX_COORDINATE, 0, 0);
	//input_set_abs_params(input, ABS_RX, 0, GUL_MAX_X_POS, 0, 0);
	//input_set_abs_params(input, ABS_RY, 0, GUL_MAX_Y_POS, 0, 0);
	//input_set_abs_params(input, ABS_RZ, 0, Z_MAX_COORDINATE, 0, 0);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = pinnacle_ts_open;
	input->close = pinnacle_ts_close;

	priv->client = client;
	priv->input = input;
	priv->irq = client->irq;
	priv->pdata = client->dev.platform_data;
	PRINTK("pinnacle_ts_probe: priv->irq: 0x%X, flags 0x%X.\n", priv->irq, (unsigned int)priv->pdata->irq_flags);

	INIT_WORK(&priv->work, pinnacle_ts_read);
	i2c_set_clientdata(client, priv);
	input_set_drvdata(input, priv);

	if (!priv->pdata->gufpinnacle_reset || !priv->pdata->gufpinnacle_reset())
	{
		dev_err(&client->dev, "failed reseting touch controller.\n");
		goto err1;
	}

	err = request_irq(priv->irq, pinnacle_ts_isr, priv->pdata->irq_flags, priv->client->name, priv);
	if (err)
	{
		dev_err(&client->dev, "unable to request touchscreen IRQ. error: %i\n", err);
		goto err1;
	}

	err = input_register_device(input);
	if (err)
	{
		dev_err(&client->dev, "failed to register device.\n");
		goto err1;
	}
	device_init_wakeup(&client->dev, 1);
	disable_irq(priv->irq);
	PRINTK("Pinnacle probing successful done.\n");
	return 0;

err1:
	if (input)
	input_free_device(input);
	i2c_set_clientdata(client, NULL);
	if (priv)
	{
		if (priv->lpXSamples)
			kfree(priv->lpXSamples);
		if (priv->lpYSamples)
			kfree(priv->lpYSamples);
		if (priv->lpBufferSamples)
			kfree(priv->lpBufferSamples);
		if (priv->lpCalData)
			kfree(priv->lpCalData);
	kfree(priv);
	}
err0:
	return err;
}

static int __devexit pinnacle_ts_remove(struct i2c_client *client)
{
	struct pinnacle_ts_priv *priv = i2c_get_clientdata(client);

	free_irq(priv->irq, priv);
	cancel_work_sync(&priv->work);
	i2c_set_clientdata(client, NULL);
	input_unregister_device(priv->input);
	kfree(priv->lpCalData);
	kfree(priv);
	return 0;
}

#ifdef CONFIG_PM
static int pinnacle_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pinnacle_ts_priv *priv = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		if (enable_irq_wake(priv->irq) < 0)
			printk(KERN_WARNING "pinnacle_ts_suspend: Couldn't enable irq wake.\r\n");
	return 0;
}

static int pinnacle_ts_resume(struct i2c_client *client)
{
	struct pinnacle_ts_priv *priv = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		if (disable_irq_wake(priv->irq) < 0)
			printk(KERN_WARNING "pinnacle_ts_suspend: Couldn't disable irq wake.\r\n");
	return 0;
}
#else
#define pinnacle_ts_suspend NULL
#define pinnacle_ts_resume NULL
#endif

static const struct i2c_device_id pinnacle_ts_id[] = {
	{ "pinnacle_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pinnacle_ts_id);

static struct i2c_driver pinnacle_ts_driver = {
	.driver = {
		.name = "pinnacle_ts",
	},
	.probe = pinnacle_ts_probe,
	.remove = pinnacle_ts_remove,
	.suspend = pinnacle_ts_suspend,
	.resume = pinnacle_ts_resume,
	.id_table = pinnacle_ts_id,
};

static int __init pinnacle_ts_init(void)
{
	return i2c_add_driver(&pinnacle_ts_driver);
}

static void __exit pinnacle_ts_exit(void)
{
	i2c_del_driver(&pinnacle_ts_driver);
}

MODULE_DESCRIPTION("Pinnacle Touchscreen driver");
MODULE_AUTHOR("Nils Grimm <nils.grimm@garz-fricke.de>");
MODULE_VERSION("2:6.33-SVN: $Revision: 1342 $");
MODULE_LICENSE("GPL");

module_init(pinnacle_ts_init);
module_exit(pinnacle_ts_exit);

