/*
 * Copyright (C) 2010 Nils Grimm, Garz & Fricke GmbH
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

 #ifndef __PINNACLE_TS_H
 #define __PINNACLE_TS_H

#define PINNACLE_COMMAND_COMPLETE_WAIT							1000
#define PINNACLE_EXT_COMMAND_WAIT								PINNACLE_COMMAND_COMPLETE_WAIT

#define PINNACLE_DEV_ADDRESS									(0x2A)
#define PINNACLE_REG_WRITE_LENGTH								0x1
#define PINNACLE_TOUCH_DATA_OFFS								0x0
#define PINNACLE_TOUCH_DATA_LENGTH								0x4
#define PINNACLE_CHIP_ID										0x07
#define PINNACLE_CHIP_VERSION									0x3A
#define	PINNACLE_PRODUCT_ID										0x00
#define PINNACLE_HCO0											0x00
#define PINNACLE_HCO1											0x40
#define PINNACLE_HCO2											0x00
#define PINNACLE_HCO3											0x00
#define PINNACLE_APPERTURE_SIZE									0x0B

#define PINNACLE_MIN_PRESSURE									0x15

//extended registers
#define HOSTREG__HCOs											0x48
#define HOSTREG__APTERTURE_SIZE									0xEE

/* Register Access Protocol (RAP) commands */
#define PINNACLE_READ                                           0xA0
#define PINNACLE_WRITE                                          0x80

#define PINNACLE_CAL_REG_START									0x1df
#define PINNACLE_CAL_REG_LENGTH									92
#define PINNACLE_CAL_REG_SIZE									sizeof(u16)

#define PINNACLE_STD_FIRMWARE									"pinnacle_ts_std.fw"
#define PINNACLE_CAL_FIRMWARE									"ts_cal.fw"

#define HOSTREG__0      (0x00)
#define HOSTREG__1      (0x01)
#define HOSTREG__2      (0x02)
#define HOSTREG__3      (0x03)
#define HOSTREG__4      (0x04)
#define HOSTREG__5      (0x05)
#define HOSTREG__6      (0x06)
#define HOSTREG__7      (0x07)
#define HOSTREG__8      (0x08)
#define HOSTREG__9      (0x09)
#define HOSTREG__10     (0x0A)
#define HOSTREG__11     (0x0B)
#define HOSTREG__12     (0x0C)
#define HOSTREG__13     (0x0D)
#define HOSTREG__14     (0x0E)
#define HOSTREG__15     (0x0F)
#define HOSTREG__16     (0x10)
#define HOSTREG__17     (0x11)
#define HOSTREG__18     (0x12)
#define HOSTREG__19     (0x13)
#define HOSTREG__20     (0x14)
#define HOSTREG__21     (0x15)
#define HOSTREG__22     (0x16)
#define HOSTREG__23     (0x17)
#define HOSTREG__24     (0x18)
#define HOSTREG__25     (0x19)
#define HOSTREG__26     (0x1A)
#define HOSTREG__27     (0x1B)
#define HOSTREG__28     (0x1C)
#define HOSTREG__29     (0x1D)
#define HOSTREG__30     (0x1E)
#define HOSTREG__31     (0x1F)
#define HOSTREG__CHIPID                                         HOSTREG__0
#define HOSTREG__VERSION                                        HOSTREG__1
#define HOSTREG__STATUS1                                        HOSTREG__2
#define HOSTREG__STATUS1__DATA_READY                            0x04
#define HOSTREG__STATUS1__COMMAND_COMPLETE                      0x08
#define HOSTREG__SYSCONFIG1                                     HOSTREG__3
#define HOSTREG__SYSCONFIG1__RESET                              0x01
#define HOSTREG__SYSCONFIG1__STANDBY                            0x02
#define HOSTREG__SYSCONFIG1__AUTO_SLEEP                         0x04
#define HOSTREG__SYSCONFIG1__TRACK_DISABLE                      0x08
#define HOSTREG__SYSCONFIG1__ANYMEAS_ENABLE                     0x10
#define HOSTREG__SYSCONFIG1__GPIO_CTRL_ENABLE                   0x20
#define HOSTREG__SYSCONFIG1__WAKEUP_TOGGLE                      0x40
#define HOSTREG__SYSCONFIG1__FORCE_WAKEUP                       0x80
#define HOSTREG__FEEDCONFIG1                                    HOSTREG__4
#define HOSTREG__FEEDCONFIG1__FEED_ENABLE                       0x01
#define HOSTREG__FEEDCONFIG1__DATA_TYPE__REL0_ABS1              0x02
#define HOSTREG__FEEDCONFIG1__FILTER_DISABLE                    0x04
#define HOSTREG__FEEDCONFIG1__X_AXIS_DISABLE                    0x08
#define HOSTREG__FEEDCONFIG1__Y_AXIS_DISABLE                    0x10
#define HOSTREG__FEEDCONFIG1__AXIS_FOR_Z__Y0_X1                 0x20
#define HOSTREG__FEEDCONFIG1__X_DATA_INVERT                     0x40
#define HOSTREG__FEEDCONFIG1__Y_DATA_INVERT                     0x80
#define HOSTREG__FEEDCONFIG2                                    HOSTREG__5
#define HOSTREG__FEEDCONFIG2__INTELLIMOUSE_MODE                 0x01
#define HOSTREG__FEEDCONFIG2__ALL_TAP_DISABLE                   0x02
#define HOSTREG__FEEDCONFIG2__SECONDARY_TAP_DISABLE             0x04
#define HOSTREG__FEEDCONFIG2__SCROLL_DISABLE                    0x08
#define HOSTREG__FEEDCONFIG2__GLIDE_EXTEND_DISABLE              0x10
#define HOSTREG__FEEDCONFIG2__PALM_BEFORE_Z_ENABLE              0x20
#define HOSTREG__FEEDCONFIG2__BUTNS_46_SCROLL_5_MIDDLE          0x40
#define HOSTREG__FEEDCONFIG2__SWAP_XY_RELATIVE                  0x80
#define HOSTREG__FEEDCONFIG3                                    HOSTREG__6
#define HOSTREG__FEEDCONFIG3__BTNS_456_TO_123_IN_REL            0x01
#define HOSTREG__FEEDCONFIG3__DISABLE_CROSS_RATE_SMOOTHING      0x02
#define HOSTREG__FEEDCONFIG3__DISABLE_PALM_NERD_MEAS            0x04
#define HOSTREG__FEEDCONFIG3__DISABLE_NOISE_AVOIDANCE           0x08
#define HOSTREG__FEEDCONFIG3__DISABLE_WRAP_LOCKOUT              0x10
#define HOSTREG__FEEDCONFIG3__DISABLE_DYNAMIC_EMI_ADJUST        0x20
#define HOSTREG__FEEDCONFIG3__DISABLE_HW_EMI_DETECT             0x40
#define HOSTREG__FEEDCONFIG3__DISABLE_SW_EMI_DETECT             0x80
#define HOSTREG__CALCONFIG1                                     HOSTREG__7
#define HOSTREG__CALCONFIG1__CALIBRATE                          0x01
#define HOSTREG__CALCONFIG1__SMALL_NERD_STILL_COMP_ENABLE       0x02
#define HOSTREG__CALCONFIG1__NERD_COMP_ENABLE                   0x04
#define HOSTREG__CALCONFIG1__TRACK_ERROR_COMP_ENABLE            0x08
#define HOSTREG__CALCONFIG1__DRIFT_USE_Y						0x10
#define HOSTREG__CALCONFIG1__TRACK_NO_PALM_COMP_ENABLE          0x20
#define HOSTREG__CALCONFIG1__CALIBRATION_MATRIX_DISABLE         0x40
#define HOSTREG__CALCONFIG1__DRIFT_COMP_ENABLE					0x80
#define HOSTREG__PS2AUX_CTRL                                    HOSTREG__8
#define HOSTREG__PS2AUX_CTRL__AUX_PRESENT               		0x80
#define HOSTREG__PS2AUX_CTRL__DISABLE_AA00_DETECT       		0x40
#define HOSTREG__PS2AUX_CTRL__SP_COORDINATE_DISABLE     		0x20
#define HOSTREG__PS2AUX_CTRL__GS_COORDINATE_DISABLE     		0x10
#define HOSTREG__PS2AUX_CTRL__SP_DISABLE                		0x08
#define HOSTREG__PS2AUX_CTRL__GS_DISABLE                		0x04
#define HOSTREG__PS2AUX_CTRL__SP_EXTENDED_MODE          		0x02
#define HOSTREG__PS2AUX_CTRL__CMD_PASSTHRU_ENABLE       		0x01
#define HOSTREG__SAMPLERATE                                     HOSTREG__9
#define HOSTREG__SAMPLERATE__10_SPS                             0x0A
#define HOSTREG__SAMPLERATE__20_SPS                             0x14
#define HOSTREG__SAMPLERATE__40_SPS                             0x28
#define HOSTREG__SAMPLERATE__60_SPS                             0x3C
#define HOSTREG__SAMPLERATE__80_SPS                             0x50
#define HOSTREG__SAMPLERATE__100_SPS                            0x64
#define HOSTREG__SAMPLERATE__200_SPS                            0xC8        // 200sps not supported
#define HOSTREG__ZIDLE                                          HOSTREG__10
#define HOSTREG__ZSCALER                                        HOSTREG__11
#define HOSTREG__SLEEP_INTERVAL                                 HOSTREG__12
#define HOSTREG__SLEEP_DELAY                                    HOSTREG__13
#define HOSTREG__DYNAMIC_EMI_ADJUST_THRESHOLD					HOSTREG__14
#define HOSTREG__PACKETBYTE_0									HOSTREG__15
#define HOSTREG__PACKETBYTE_1									HOSTREG__16
#define HOSTREG__PACKETBYTE_2									HOSTREG__17
#define HOSTREG__PACKETBYTE_3									HOSTREG__18
#define HOSTREG__PACKETBYTE_4									HOSTREG__19
#define HOSTREG__PACKETBYTE_5		                      		HOSTREG__20
#define HOSTREG__PACKETBYTE_5_TOUCH								0x80
#define HOSTREG__PACKETBYTE_6									HOSTREG__21
#define HOSTREG__PACKETBYTE_7                                   HOSTREG__22
#define HOSTREG__PACKETBYTE_8									HOSTREG__23
#define HOSTREG__PORTA_GPIO_CTRL                                HOSTREG__24
#define HOSTREG__PORTA_GPIO_DATA                                HOSTREG__25
#define HOSTREG__PORTB_GPIO_RSVD_0          					0x80
#define HOSTREG__PORTB_GPIO_RSVD_1          					0x40
#define HOSTREG__PORTB_GPIO_CTRL__PB2       					0x20
#define HOSTREG__PORTB_GPIO_CTRL__PB1       					0x10
#define HOSTREG__PORTB_GPIO_CTRL__PB0       					0x08
#define HOSTREG__PORTB_GPIO_DATA__PB2       					0x04
#define HOSTREG__PORTB_GPIO_DATA__PB1       					0x02
#define HOSTREG__PORTB_GPIO_DATA__PB0       					0x01
#define HOSTREG__PORTB_GPIO_CTRL_DATA                           HOSTREG__26
#define HOSTREG__EXT_REG_AXS_VALUE                              HOSTREG__27
#define HOSTREG__EXT_REG_AXS_ADDR_HIGH                          HOSTREG__28
#define HOSTREG__EXT_REG_AXS_ADDR_LOW                           HOSTREG__29
#define HOSTREG__EXT_REG_AXS_CTRL                               HOSTREG__30
#define HOSTREG__EREG_AXS_CTRL_RSVD_0                           0x80
#define HOSTREG__EREG_AXS_CTRL_RSVD_1                           0x40
#define HOSTREG__EREG_AXS_CTRL_RSVD_2                           0x20
#define HOSTREG__EREG_AXS_CTRL_RSVD_3                           0x10
#define HOSTREG__EREG_AXS_CTRL_INC_ADDR_WRITE                   0x08
#define HOSTREG__EREG_AXS_CTRL_INC_ADDR_READ                    0x04
#define HOSTREG__EREG_AXS_CTRL_WRITE                            0x02
#define HOSTREG__EREG_AXS_CTRL_READ                             0x01
#define HOSTREG__PRODUCT_ID                                     HOSTREG__31


#define PINNACLE_UP_HYSTERESIS 									5
#define PINNACLE_DOWN_HYSTERESIS 								5
#define PINNACLE_MINIMUM_VALID_SAMPLES 							3
#define PINNACLE_MINIMUM_VALID_HOLD 							2
#define PINNACLE_MINIMUM_VALID_SUSTAIN							1
#define PINNACLE_MINIMUM_VALID_DECAY							2
#define PINNACLE_MAX_DISTANCE									2
#define PINNACLE_MAX_X_POS										0x1000
#define PINNACLE_MIN_X_POS										0x46
#define PINNACLE_MAX_Y_POS										0x1000
#define PINNACLE_MIN_Y_POS										0x5a
#define PINNACLE_MIN_CAL_DIFF_VALUE								0x30
#define PINNACLE_CHECK_MAX_VAL_AFTER_CAL						15

struct pinnacle_direct_regs
{
	u8 ucChipID;
	u8 ucChipVersion;
	u8 ucStatus1;
	u8 ucSysConfig1;
	u8 ucFeedConfig1;
	u8 ucFeedConfig2;
	u8 ucFeedConfig3;
	u8 ucCalConfig1;
	u8 ucPs2AuxControl;
	u8 ucSampleRate;
	u8 ucZIdle;
	u8 ucZScaler;
	u8 ucSleepInterval;
	u8 ucSleepDelay;
	u8 ucDynamicEMIAdjustThreshold;
	u8 ucPacketu80;
	u8 ucPacketu81;
	u8 ucPacketu82;
	u8 ucPacketu83;
	u8 ucPacketu84;
	u8 ucPacketu85;
	u8 ucPacketu86;
	u8 ucPacketu87;
	u8 ucPacketu88;
	u8 ucPortAGPIOControl;
	u8 ucPortAGPIOData;
	u8 ucPortBGPIOCntrlData;
	u8 ucERA_Value;
	u8 ucERA_HighAddress;
	u8 ucERA_LowAddress;
	u8 ucERA_AccessControl;
	u8 ucProductId;
};

#define PINNACLE_TOUCH_DATA_FLAGS_TOUCH		0x8000
struct pinnacle_touch_data
{
	u16 usXPos;
	u16 usYPos;
	u16 usZPos;
	u16 usFlags;
};

#define MAX_ADC_VAL 0xFFF

#define MAX_CONFIG_DATA_PER_SLOT	4
struct pinnacle_touch_config_slot
{
	u16 	usAddress;
	u8 		lpWriteData[MAX_CONFIG_DATA_PER_SLOT];
	u8		ucWriteDataCount;
	u8 		lpReadData[MAX_CONFIG_DATA_PER_SLOT];
	u8		ucReadDataCount;
	u8 		lpDataMask[MAX_CONFIG_DATA_PER_SLOT];
	bool	bVerification;
	u16		dwSleep;
	u8		ucWaitForInterrupt;
};

// touch driver communications
#define TOUCH_MAX_REG_SIZE 0x10000

enum touch_commands
{
	TOUCH_CMD_READ_REG,
	TOUCH_CMD_WRITE_REG,
	TOUCH_CMD_RECALIBRATE,
	TOUCH_CMD_BLOCK_NORMAL_OP,
	TOUCH_CMD_ALLOW_NORMAL_OP,
	TOUCH_CMD_DUMP_REG_SET,
	TOUCH_CMD_PROGRAM_REG_SET
};

struct touch_rcv_read_reg
{
	u32  ulSizeOfStruct;
	u16  wRegister;
};

struct touch_snd_read_reg
{
	u32  ulSizeOfStruct;
	u16  wRegister;
	u16  wValue;
};

struct touch_rcv_write_reg
{
	u32  ulSizeOfStruct;
	u16  wRegister;
	u16  wValue;
	u32  ulSleepMsAfterWrite;
	u16  wWaitForInterruptAfterWrite;
};

struct touch_snd_reg_value_pair
{
	u16	wRegister;
	u16	wValue;
};

struct touch_snd_dump_reg_set
{
	u32	ulSizeOfStruct;
	u32	ulRegisterCount;
	struct touch_snd_reg_value_pair		regValues[TOUCH_MAX_REG_SIZE];
};

struct touch_rcv_dump_reg_set
{
	u32 		ulSizeOfStruct;
	u16 		wRegisterStart;
	u32		ulRegisterCount;
};

struct touch_rcv_prg_value
{
	u16 	wRegister;
	u16 	wValue;
	u32 	ulSleepMsAfterWrite;
	u16 	wWaitForInterruptAfterWrite;
};

struct touch_rcv_prg_reg_set
{
	u32 				ulSizeOfStruct;
	u32 				ulRegisterCount;
	struct touch_rcv_prg_value			regValues[TOUCH_MAX_REG_SIZE];
};

struct touch_snd_prg_value
{
	u32 ulSizeOfStruct;
	u32 ulRegisterCount;
};

struct touch_receive_api_package
{
	u32			ulSizeOfStruct;
	u32			ulCmdNumber;
	enum touch_commands 	touchCmd;
	union
	{
		struct touch_rcv_read_reg 		readReg;
		struct touch_rcv_write_reg		writeReg;
		struct touch_rcv_prg_reg_set 	prgRegSet;
		struct touch_rcv_dump_reg_set	dumpRegSet;
	} data;
};

struct touch_transmit_api_package
{
	u32			ulSizeOfStruct;
	u32			ulCmdNumber;
	enum touch_commands 	touchCmd;
	u32 			dwGetLastError;
	union
	{
		struct touch_snd_read_reg		readReg;
		struct touch_snd_dump_reg_set	dumpRegSet;
		struct touch_snd_prg_value		prgRegSet;
	} data;
};

/*
struct touch_communication_thread_parameter
{
	bool 				bRun;
	bool				bBlockNormalOp;
	struct mutex 		csComThread;
	u32		dwInterruptMask;
};
*/

enum enumTouchPanelSampleFlags
{
	TouchSampleValidFlag			= 0x01,		//@EMEM	The sample is valid.
	TouchSampleDownFlag				= 0x02,		//@EMEM	The finger/stylus is down.
	TouchSampleIsCalibratedFlag		= 0x04,		//@EMEM	The XY data has already been calibrated.
	TouchSamplePreviousDownFlag		= 0x08,		//@EMEM	The state of the previous valid sample.
    TouchSampleIgnore				= 0x10,     //@EMEM Ignore this sample.
};

typedef enum enumTouchPanelSampleFlags TOUCH_PANEL_SAMPLE_FLAGS;

struct pinnacle_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct mutex mutex;
	struct gufpinnacle_platform_data *pdata;
	struct 	bin_attribute sysfs_cal_data;
	int irq;
	bool  	bDriverOpened;
	bool  	bDriverInitialized;
	bool  	bControllerCalibrated;
	bool    bOldTouchDown;
	u8    	ucDynDownHysteresis;
	u8    	ucDynMinimumValidSamples;
	u8 	  	ucValidSamples;
	u16*  	lpXSamples;
	u16*  	lpYSamples;
	u16*  	lpBufferSamples;
	u16*  	lpCalData;
	u16 	usXBuffer;
	u16 	usYBuffer;
	u16 	usZBuffer;
	u8	  	ucUpHysteresis;							//default 5
	u8 	  	ucDownHysteresis;						//default 5
	u8	  	ucMinimumValidSamples;					//default 3
	u8	  	ucMinimumValidHold;						//default 2
	u8	  	ucMinimumValidSustain;					//default 1
	u8	  	ucMinimumValidDecay;					//default 2
	u32	  	ulMaxDistance;							//default 2
	u32   	ulMaxXPos;								//default 0x1000
	u32	  	ulMinXPos;								//default 0x46
	u32	  	ulMaxYPos;								//default 0x1000
	u32	  	ulMinYPos;								//default 0x5a
	u32	  	ulMinCalDiffValue;						//default 0x30
	u32	  	ulCheckMaxValAfterCal;					//default 15
};

#define X_SCALE_OFFSET			80
#define Y_SCALE_OFFSET			80
#if defined CONFIG_ANDROID
#define X_MAX_COORDINATE		1880
#define Y_MAX_COORDINATE		1330
#else
#define X_MAX_COORDINATE		1865
#define Y_MAX_COORDINATE		1365
#endif
#define Z_MAX_COORDINATE		0x7f
#define INTERRUPT_WAIT_TIMEOUT	1000

#endif