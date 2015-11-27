/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2009 Marc Kleine-Budde, Pengutronix
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * Copyright (C) 2011 Meprolight, Ltd.
 * Alex Gershgorin <alexg@meprolight.com>
 *
 * Modified from i.MX31 3-Stack Development System
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
 */

/*
 * This machine is known as:
 *  - i.MX35 3-Stack Development System
 *  - i.MX35 Platform Development Kit (i.MX35 PDK)
 */
#define DEBUG

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/memory.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mtd/physmap.h>
#include <linux/mfd/mc13892.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/spi.h>
#include <linux/kthread.h>
#include <linux/can/platform/flexcan.h>
#include <linux/i2c/gfeeprom.h>
#include <linux/power_supply.h>
#include <linux/power/gpio-charger.h>

#include <video/platform_lcd.h>
#include <linux/pwm_backlight.h>
#include <linux/fb.h>
#include <linux/lcd.h>
#include <linux/guf_xml_config.h>
#include <linux/cupid_imx_ac97.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/input/pinnacle_ts.h>
#include <linux/input/eetii2c_ts.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/memblock.h>

#include "hardware.h"
#include "common.h"
#include "iomux-mx35.h"
#include "devices-imx35.h"
#include "cpuidle.h"
#include "guf.h"

#define BLINK(a,b,c) \
	do{ \
		int __cnt = c; \
		while((--__cnt)) \
		{ \
			gpio_set_value(LED1_GPIO, 1); \
			msleep((a)); \
			gpio_set_value(LED1_GPIO, 0); \
			msleep((b)); \
		} \
	}while(0)

#define SION_BIT_ENABLED(iomux) \
	(iomux) | ((iomux_v3_cfg_t)IOMUX_CONFIG_SION << MUX_MODE_SHIFT)

/* FIXME: number of chips times GPIOs per chip */
#define NUM_GPIO_CHIPS 		3
#define NUM_GPIOS_PER_CHIP 	32

/******************************************************************************/
/* GPIO DEFINES                                                               */
/******************************************************************************/

/* two pins are controlling the PWR signals to the USB phys */
#define USBH_PWR	IMX_GPIO_NR(3, 3)
#define USBH_OC		IMX_GPIO_NR(3, 4)

#define USBOTG_PWR	IMX_GPIO_NR(3, 14)
#define USBOTG_OC	IMX_GPIO_NR(3, 15)

/* one pin is controlling the display */
#define LCD_ENA		IMX_GPIO_NR(3, 24)

/* Audio pins */
#define AC97_GPIO_TXFS	IMX_GPIO_NR(2, 31)
#define AC97_GPIO_TXD	IMX_GPIO_NR(2, 28)
#define AC97_GPIO_INT	IMX_GPIO_NR(2, 29)
#define SPEAKER_ON	IMX_GPIO_NR(3, 26)

/* LEDs */
#define LED1_GPIO	IMX_GPIO_NR(1, 14)

/* Flexcan / RS485 */
#define CAN_RS485_PWR_EN 	IMX_GPIO_NR(3, 25)
#define RS485_TX_EN			IMX_GPIO_NR(2, 22)

/* MDB */
#define MDB_WAKEUP_OUT	IMX_GPIO_NR(1, 15)

/* External touch interrupt lines */
#define TOUCH_INT		IMX_GPIO_NR(3, 29)
#define TOUCH_INT2		IMX_GPIO_NR(1, 8)
/* External touch reset lines */
#define TOUCH_RESET		IMX_GPIO_NR(3, 28)
#define TOUCH_RESET2	IMX_GPIO_NR(1, 7)
/* External touch I2C lines */
#define TOUCH_SCL		IMX_GPIO_NR(1, 12)
#define TOUCH_SDA		IMX_GPIO_NR(1, 13)

/* GPIOs on digital I/O connector (X14) */
#define DIG_IN1 	IMX_GPIO_NR(1, 0)
#define DIG_IN2 	IMX_GPIO_NR(1, 1)
#define DIG_OUT1 	IMX_GPIO_NR(1, 4)
#define DIG_OUT2 	IMX_GPIO_NR(1, 5)

/* GPIOs on keypad connector (X21) */
#define KP_ROW0 	IMX_GPIO_NR(2, 0)		// Pin 3
#define KP_COL0 	IMX_GPIO_NR(2, 12)		// Pin 4
#define KP_ROW1 	IMX_GPIO_NR(2, 1)		// Pin 5
#define KP_COL1 	IMX_GPIO_NR(2, 13)		// Pin 6
#define KP_ROW2 	IMX_GPIO_NR(2, 2)		// Pin 7
#define KP_COL2 	IMX_GPIO_NR(2, 14)		// Pin 8
#define KP_ROW3 	IMX_GPIO_NR(2, 3)		// Pin 9
#define KP_COL3 	IMX_GPIO_NR(2, 15)		// Pin 10
#define KP_ROW4 	IMX_GPIO_NR(2, 6)		// Pin 11
#define KP_COL4 	IMX_GPIO_NR(2, 17)		// Pin 12
#define KP_ROW5_DMA 	IMX_GPIO_NR(2, 11)	// Pin 13
#define KP_COL5_SS1 	IMX_GPIO_NR(2, 8)	// Pin 14
#define KP_ROW6_MISO 	IMX_GPIO_NR(2, 10)	// Pin 15
#define KP_COL6_MOSI 	IMX_GPIO_NR(2, 9)	// Pin 16
#define KP_ROW7_SLK 	IMX_GPIO_NR(2, 16)	// Pin 17
#define KP_COL7_SS0 	IMX_GPIO_NR(2, 7)	// Pin 18

/* SW_RESET# GPIO */
#define WDOG_B_RESET 	IMX_GPIO_NR(1, 6)

/******************************************************************************/
/* GPIO Names                                                                 */
/******************************************************************************/

/* These are pin names that will appear under /sys/class/gpio/ */
static const char * const guf_cupid_gpio_names[NUM_GPIO_CHIPS*NUM_GPIOS_PER_CHIP] = {
	[DIG_IN1]		= "dig_in1",
	[DIG_IN2]		= "dig_in2",
	[DIG_OUT1]		= "dig_out1",
	[DIG_OUT2]		= "dig_out2",
	[KP_ROW0]		= "keypad_pin3",
	[KP_COL0]		= "keypad_pin4",
	[KP_ROW1]		= "keypad_pin5",
	[KP_COL1]		= "keypad_pin6",
	[KP_ROW2]		= "keypad_pin7",
	[KP_COL2]		= "keypad_pin8",
	[KP_ROW3]		= "keypad_pin9",
	[KP_COL3]		= "keypad_pin10",
	[KP_ROW4]		= "keypad_pin11",
	[KP_COL4]		= "keypad_pin12",
	[KP_ROW5_DMA]	= "keypad_pin13",
	[KP_COL5_SS1]	= "keypad_pin14",
	[KP_ROW6_MISO]	= "keypad_pin15",
	[KP_COL6_MOSI]	= "keypad_pin16",
	[KP_ROW7_SLK]	= "keypad_pin17",
	[KP_COL7_SS0]	= "keypad_pin18",
	[MDB_WAKEUP_OUT]	= "mdb_wakeup_out",
};
	


/******************************************************************************/
/* PIN MULTIPLEXING                                                           */
/******************************************************************************/
static iomux_v3_cfg_t guf_cupid_dio_pads[] = {
	/* DIO */
	MX35_PAD_STXD5__GPIO1_0,
	MX35_PAD_SRXD5__GPIO1_1,
	SION_BIT_ENABLED(MX35_PAD_SCKR__GPIO1_4),
	SION_BIT_ENABLED(MX35_PAD_FSR__GPIO1_5),
};

static iomux_v3_cfg_t guf_cupid_uart1_pads[] = {
	/* UART1 */
	MX35_PAD_CTS1__UART1_CTS,
	MX35_PAD_RTS1__UART1_RTS,
	MX35_PAD_TXD1__UART1_TXD_MUX,
	MX35_PAD_RXD1__UART1_RXD_MUX,
};

static iomux_v3_cfg_t guf_cupid_uart2_pads[] = {
	/* UART2 */
	MX35_PAD_CTS2__UART2_CTS,
	MX35_PAD_RTS2__UART2_RTS,
	MX35_PAD_TXD2__UART2_TXD_MUX,
	MX35_PAD_RXD2__UART2_RXD_MUX,
	MX35_PAD_TX0__GPIO1_15,				// MDB_WAKEUP_OUT
};

static iomux_v3_cfg_t guf_cupid_uart3_pads[] = {
	/* UART3 */
	MX35_PAD_ATA_DATA11__UART3_TXD_MUX,
	MX35_PAD_ATA_DATA10__UART3_RXD_MUX,
	MX35_PAD_ATA_DATA9__GPIO2_22,		// RS485_TX_EN
};

static iomux_v3_cfg_t guf_cupid_flexcan_pads[] = {
	/* Flexcan1 */
	MX35_PAD_SD2_DATA3__CAN1_TXCAN,
	MX35_PAD_SD2_DATA2__CAN1_RXCAN,
	/* Flexcan2 */
	MX35_PAD_TX5_RX0__CAN2_TXCAN,
	MX35_PAD_TX4_RX1__CAN2_RXCAN,
};

static iomux_v3_cfg_t guf_cupid_can_rs485_pwr[] = {
	/* CAN/RS485_PWR_EN */
	MX35_PAD_LD19__GPIO3_25,
};

static iomux_v3_cfg_t guf_cupid_fec_pads[] = {
	/* FEC */
	MX35_PAD_FEC_TX_CLK__FEC_TX_CLK,
	MX35_PAD_FEC_RX_CLK__FEC_RX_CLK,
	MX35_PAD_FEC_RX_DV__FEC_RX_DV,
	MX35_PAD_FEC_COL__FEC_COL,
	MX35_PAD_FEC_RDATA0__FEC_RDATA_0,
	MX35_PAD_FEC_TDATA0__FEC_TDATA_0,
	MX35_PAD_FEC_TX_EN__FEC_TX_EN,
	MX35_PAD_FEC_MDC__FEC_MDC,
	MX35_PAD_FEC_MDIO__FEC_MDIO,
	MX35_PAD_FEC_TX_ERR__FEC_TX_ERR,
	MX35_PAD_FEC_RX_ERR__FEC_RX_ERR,
	MX35_PAD_FEC_CRS__FEC_CRS,
	MX35_PAD_FEC_RDATA1__FEC_RDATA_1,
	MX35_PAD_FEC_TDATA1__FEC_TDATA_1,
	MX35_PAD_FEC_RDATA2__FEC_RDATA_2,
	MX35_PAD_FEC_TDATA2__FEC_TDATA_2,
	MX35_PAD_FEC_RDATA3__FEC_RDATA_3,
	MX35_PAD_FEC_TDATA3__FEC_TDATA_3,
};

static iomux_v3_cfg_t guf_cupid_usbh1_pads[] = {
	/* USB H1 */
	MX35_PAD_MLB_CLK__GPIO3_3,
	MX35_PAD_MLB_DAT__GPIO3_4,
};

static iomux_v3_cfg_t guf_cupid_usb_otg_pads[] = {
	/* USB OTG */
	MX35_PAD_USBOTG_PWR__USB_TOP_USBOTG_PWR,
	MX35_PAD_USBOTG_OC__USB_TOP_USBOTG_OC,
};

static iomux_v3_cfg_t guf_cupid_esdhc_pads[] = {
	/* ESDHC1 */
	MX35_PAD_SD1_CMD__ESDHC1_CMD,
	MX35_PAD_SD1_CLK__ESDHC1_CLK,
	MX35_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX35_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX35_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX35_PAD_SD1_DATA3__ESDHC1_DAT3,
	MX35_PAD_ATA_DATA5__GPIO2_18, 		/* CD */
	MX35_PAD_ATA_DATA6__GPIO2_19, 		/* WP */
};

static iomux_v3_cfg_t guf_cupid_keypad_pads[] = {
	/* KEYPAD */
	MX35_PAD_SD2_CLK__GPIO2_1,
	MX35_PAD_SD2_CMD__GPIO2_0,
	MX35_PAD_SD2_DATA0__GPIO2_2,
	MX35_PAD_SD2_DATA1__GPIO2_3,
	MX35_PAD_ATA_CS0__GPIO2_6,
	MX35_PAD_ATA_CS1__GPIO2_7,
	MX35_PAD_ATA_DIOR__GPIO2_8,
	MX35_PAD_ATA_DIOW__GPIO2_9,
	MX35_PAD_ATA_DMACK__GPIO2_10,
	MX35_PAD_ATA_RESET_B__GPIO2_11,
	MX35_PAD_ATA_IORDY__GPIO2_12,
	MX35_PAD_ATA_DATA0__GPIO2_13,
	MX35_PAD_ATA_DATA1__GPIO2_14,
	MX35_PAD_ATA_DATA2__GPIO2_15,
	MX35_PAD_ATA_DATA3__GPIO2_16,
	MX35_PAD_ATA_DATA4__GPIO2_17,
};

static iomux_v3_cfg_t guf_cupid_led_pads[] = {
	/* LEDs */
	MX35_PAD_TX1__GPIO1_14,
};

static iomux_v3_cfg_t guf_cupid_display_pads[] = {
	/* Display */
	MX35_PAD_LD0__IPU_DISPB_DAT_0,
	MX35_PAD_LD1__IPU_DISPB_DAT_1,
	MX35_PAD_LD2__IPU_DISPB_DAT_2,
	MX35_PAD_LD3__IPU_DISPB_DAT_3,
	MX35_PAD_LD4__IPU_DISPB_DAT_4,
	MX35_PAD_LD5__IPU_DISPB_DAT_5,
	MX35_PAD_LD6__IPU_DISPB_DAT_6,
	MX35_PAD_LD7__IPU_DISPB_DAT_7,
	MX35_PAD_LD8__IPU_DISPB_DAT_8,
	MX35_PAD_LD9__IPU_DISPB_DAT_9,
	MX35_PAD_LD10__IPU_DISPB_DAT_10,
	MX35_PAD_LD11__IPU_DISPB_DAT_11,
	MX35_PAD_LD12__IPU_DISPB_DAT_12,
	MX35_PAD_LD13__IPU_DISPB_DAT_13,
	MX35_PAD_LD14__IPU_DISPB_DAT_14,
	MX35_PAD_LD15__IPU_DISPB_DAT_15,
	MX35_PAD_LD16__IPU_DISPB_DAT_16,
	MX35_PAD_LD17__IPU_DISPB_DAT_17,
	MX35_PAD_D3_HSYNC__IPU_DISPB_D3_HSYNC,
	MX35_PAD_D3_FPSHIFT__IPU_DISPB_D3_CLK,
	MX35_PAD_D3_DRDY__IPU_DISPB_D3_DRDY,
	MX35_PAD_D3_VSYNC__IPU_DISPB_D3_VSYNC,
	MX35_PAD_LD18__GPIO3_24,			/* LCD enable */
};

/*
 * The PWM backlight pin is configured in guf_setup_backlight_from_guf_xml()
 * after the XML configuration has been parsed, because the PADCTL can be
 * configured via /configurationFile/variables/display/backlight/padctl_mx35.
 */
iomux_v3_cfg_t backlight_pwm_pad = MX35_PAD_CSPI1_SS1__PWM_PWMO;


/******************************************************************************/
/* Watchdog                                                                    */
/******************************************************************************/
static iomux_v3_cfg_t guf_cupid_watchdog_pads[] = {
	/* SW_RESET# External WDOG reset */
	MX35_PAD_WDOG_RST__WDOG_WDOG_B
};

static int watchdog_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_watchdog_pads, ARRAY_SIZE(guf_cupid_watchdog_pads));
	imx35_add_imx2_wdt();
	return 0;
};

/******************************************************************************/
/* I2C / EEPROM / RTC                                                         */
/******************************************************************************/
static void i2c_1_set_gpio_mode( struct imxi2c_platform_data * pdata, enum i2c_gpio_mode mode)
{
	iomux_v3_cfg_t i2c_pads_func[] = {	MX35_PAD_I2C1_CLK__I2C1_SCL, MX35_PAD_I2C1_DAT__I2C1_SDA};
	iomux_v3_cfg_t i2c_pads_gpio[] = {	MX35_PAD_I2C1_CLK__GPIO2_24, MX35_PAD_I2C1_DAT__GPIO2_25};
	switch(mode)
	{
	case i2c_gpio_mode_func:
		mxc_iomux_v3_setup_multiple_pads(i2c_pads_func, ARRAY_SIZE(i2c_pads_func));
		break;
	case i2c_gpio_mode_gpio:
		mxc_iomux_v3_setup_multiple_pads(i2c_pads_gpio, ARRAY_SIZE(i2c_pads_gpio));
		break;
	}
}

static struct imxi2c_platform_data guf_cupid_i2c_1_pdata = {
	.bitrate = 400000,
	.i2c_scl_pin = IMX_GPIO_NR( 2, 24),
	.i2c_sda_pin = IMX_GPIO_NR( 2, 25),
	.i2c_set_gpio_mode = i2c_1_set_gpio_mode,
};

#ifdef CONFIG_EEPROM_GARZ_FRICKE
static struct guf_eeprom_memory_accessor gl_eeprom_acc;
static void guf_cupid_eeprom_setup(struct memory_accessor *mem_acc, void *context)
{
	gl_eeprom_acc.mem_acc.read = mem_acc->read;
	gl_eeprom_acc.mem_acc.write = mem_acc->write;
	gl_eeprom_acc.cont = context;
}

static struct gfeeprom_platform_data guf_cupid_eeprom_pdata = {
	.byte_len = (1 << 12),
	.page_size = 0x20,
	.flags = AT24_FLAG_ADDR16,
	.bus_id = 0,
	.context = &(guf_cupid_eeprom_pdata.bus_id),
	.setup = guf_cupid_eeprom_setup,
};
#else
static void guf_cupid_eeprom_setup(struct memory_accessor *macc, void *context) {
}

static struct at24_platform_data guf_cupid_eeprom_pdata = {
	.byte_len = 4096,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
	.setup = guf_cupid_eeprom_setup,
};
#endif

static void i2c_2_set_gpio_mode( struct imxi2c_platform_data * pdata,  enum i2c_gpio_mode mode)
{
	iomux_v3_cfg_t i2c_pads_func[] = {	MX35_PAD_I2C2_CLK__I2C2_SCL, MX35_PAD_I2C2_DAT__I2C2_SDA};
	iomux_v3_cfg_t i2c_pads_gpio[] = {	MX35_PAD_I2C2_CLK__GPIO2_26, MX35_PAD_I2C2_DAT__GPIO2_27};
	switch(mode)
	{
	case i2c_gpio_mode_func:
		mxc_iomux_v3_setup_multiple_pads(i2c_pads_func, ARRAY_SIZE(i2c_pads_func));
		break;
	case i2c_gpio_mode_gpio:
		mxc_iomux_v3_setup_multiple_pads(i2c_pads_gpio, ARRAY_SIZE(i2c_pads_gpio));
		break;
	}
}

static struct imxi2c_platform_data guf_cupid_i2c_2_pdata = {
	.bitrate = 400000,
	.i2c_scl_pin = IMX_GPIO_NR( 2, 26),
	.i2c_sda_pin = IMX_GPIO_NR( 2, 27),
	.i2c_set_gpio_mode = i2c_2_set_gpio_mode,
};

static void i2c_3_set_gpio_mode( struct imxi2c_platform_data * pdata,  enum i2c_gpio_mode mode)
{
	iomux_v3_cfg_t i2c_pads_func[] = {MX35_PAD_TX3_RX2__I2C3_SCL, MX35_PAD_TX2_RX3__I2C3_SDA};
	iomux_v3_cfg_t i2c_pads_gpio[] = {MX35_PAD_TX3_RX2__GPIO1_12, MX35_PAD_TX2_RX3__GPIO1_13};
	switch(mode)
	{
	case i2c_gpio_mode_func:
		mxc_iomux_v3_setup_multiple_pads(i2c_pads_func, ARRAY_SIZE(i2c_pads_func));
		break;
	case i2c_gpio_mode_gpio:
		mxc_iomux_v3_setup_multiple_pads(i2c_pads_gpio, ARRAY_SIZE(i2c_pads_gpio));
		break;
	}
}

static struct imxi2c_platform_data guf_cupid_i2c_3_pdata = {
	.bitrate = 400000,
	.i2c_scl_pin = IMX_GPIO_NR( 1, 12),
	.i2c_sda_pin = IMX_GPIO_NR( 1, 13),
	.i2c_set_gpio_mode = i2c_3_set_gpio_mode,
};

static struct i2c_board_info guf_cupid_i2c_1_devices[] = {
#ifdef CONFIG_EEPROM_GARZ_FRICKE
    {
		I2C_BOARD_INFO("gfeeprom", GF_GLOBAL_PLATFORM_EEPROM_ADDRESS), /* E0=0, E1=0, E2=0 */
		.platform_data = &guf_cupid_eeprom_pdata,
	},
#else
	{
		I2C_BOARD_INFO("24c32", 0x50),
		.platform_data = &guf_cupid_eeprom_pdata,
	},
#endif
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	}
	// The autoprobe didn't worked in tests, in the schematics the address changes over the versions so we just probe both, one will fail
	,	{	I2C_BOARD_INFO("lm73", 0x48), }
	,	{	I2C_BOARD_INFO("lm73", 0x49), }
};

static int cupid_i2c_init(void)
{
	i2c_register_board_info(0, guf_cupid_i2c_1_devices, ARRAY_SIZE(guf_cupid_i2c_1_devices));
	imx35_add_imx_i2c0(&guf_cupid_i2c_1_pdata);
	imx35_add_imx_i2c1(&guf_cupid_i2c_2_pdata);
	imx35_add_imx_i2c2(&guf_cupid_i2c_3_pdata);
	return 0;
}

/******************************************************************************/
/* TOUCHSCREEN                                                                */
/******************************************************************************/

static iomux_v3_cfg_t guf_cupid_external_touch_pads[] = {
	/* External Touch */
	MX35_PAD_LD22__GPIO3_28, 			// TOUCH_RESET
	MX35_PAD_LD23__GPIO3_29, 			// TOUCH_INT
	MX35_PAD_SCKT__GPIO1_7, 			// TOUCH_RESET2
	MX35_PAD_FST__GPIO1_8,  			// TOUCH_INT2
};

static struct edt_ft5x06_platform_data guf_cupid_edt_ft5x06_platform_data = {
	.reset_pin = TOUCH_RESET,
	.wake_pin = TOUCH_RESET2,
	.irq_pin = TOUCH_INT,
};

static struct i2c_board_info guf_cupid_touch_edt_info[] = {
	{
		I2C_BOARD_INFO("edt-ft5x06", GF_EDT_FT5X06_EEPROM_ADDRESS), /* E0=0, E1=0, E2=0 */
		.platform_data = &guf_cupid_edt_ft5x06_platform_data,
	}
};

static bool guf_cupid_pinnacle_lcd_power_control(bool bOn)
{
	if (bOn)
		gpio_set_value(LCD_ENA, 1);	// on
	else
		gpio_set_value(LCD_ENA, 0);	// off
	return true;
}


static bool guf_cupid_i2c_touchscreen_reset(void)
{
	if( guf_cupid_i2c_3_pdata.i2c_set_gpio_mode)
		guf_cupid_i2c_3_pdata.i2c_set_gpio_mode( &guf_cupid_i2c_3_pdata, i2c_gpio_mode_gpio);

	// switching off the pinnacle until we open the touch driver.
	gpio_request(guf_cupid_i2c_3_pdata.i2c_sda_pin, "I2C2_SDA");
	gpio_direction_output(guf_cupid_i2c_3_pdata.i2c_sda_pin, 0);
	gpio_request(guf_cupid_i2c_3_pdata.i2c_scl_pin, "I2C2_SCL");
	gpio_direction_output(guf_cupid_i2c_3_pdata.i2c_scl_pin, 0);
	gpio_set_value(TOUCH_RESET, 1);
	msleep(100);
	gpio_set_value(TOUCH_RESET, 0);
	msleep(50);

	// done with reset configure the I2C pins for controller usage.
	gpio_direction_input(guf_cupid_i2c_3_pdata.i2c_sda_pin);
	gpio_direction_input(guf_cupid_i2c_3_pdata.i2c_scl_pin);
	gpio_free(guf_cupid_i2c_3_pdata.i2c_sda_pin);
	gpio_free(guf_cupid_i2c_3_pdata.i2c_scl_pin);

	if( guf_cupid_i2c_3_pdata.i2c_set_gpio_mode)
		guf_cupid_i2c_3_pdata.i2c_set_gpio_mode( &guf_cupid_i2c_3_pdata, i2c_gpio_mode_func);
	return true;
};

static struct gufpinnacle_platform_data guf_cupid_pinnacle_pdata = {
	.guf_set_lcd_power = guf_cupid_pinnacle_lcd_power_control,
	.gufpinnacle_reset = guf_cupid_i2c_touchscreen_reset,
	.irq_flags = IRQF_TRIGGER_LOW,
	.gpio_int_number = TOUCH_INT
};

static struct i2c_board_info guf_cupid_touch_pinnacle_info[] = {
	{
		I2C_BOARD_INFO("pinnacle_ts", 0x2A),
		.platform_data = &guf_cupid_pinnacle_pdata,
	}
};

static struct eetii2c_ts_platform_data guf_cupid_eetii2c_pdata = {
#ifdef CONFIG_HAS_EARLYSUSPEND
	.early_suspend_mode = true,
#endif
	.guf_eetii2c_reset = guf_cupid_i2c_touchscreen_reset,
	.irq_flags = IRQF_DISABLED | IRQF_TRIGGER_HIGH,
	.gpio_int_number = TOUCH_INT,
	.reset_pin = TOUCH_RESET,
	.wake_pin = TOUCH_RESET2,
};

static struct i2c_board_info guf_cupid_touch_eetii2c_info[] = {
	{
		I2C_BOARD_INFO("eetii2c_ts", GF_EETII2C_ADDRESS),
		.platform_data = &guf_cupid_eetii2c_pdata,
	},
};

/******************************************************************************/
/* AUDIO / RTOUCH                                                                     */
/******************************************************************************/
static void guf_cupid_toggle_speaker(int enable)
{
	pr_debug("<%s> %d\n", __func__, enable);
	/* Toggle speaker */
	gpio_direction_output(SPEAKER_ON, 1);
	gpio_set_value(SPEAKER_ON, enable);
}

static void guf_cupid_ac97_warm_reset(struct snd_ac97 *ac97)
{
	iomux_v3_cfg_t txfs_gpio = MX35_PAD_STXFS4__GPIO2_31;
	iomux_v3_cfg_t txfs = MX35_PAD_STXFS4__AUDMUX_AUD4_TXFS;
	int ret;

	pr_debug("<%s>\n", __func__);
	ret = gpio_request(AC97_GPIO_TXFS, "SSI");
	if (ret) {
		printk("failed to get GPIO_TXFS: %d\n", ret);
		return;
	}

	mxc_iomux_v3_setup_pad(txfs_gpio);

	/* warm reset */
	gpio_direction_output(AC97_GPIO_TXFS, 1);
	udelay(2);
	gpio_set_value(AC97_GPIO_TXFS, 0);

	gpio_free(AC97_GPIO_TXFS);
	mxc_iomux_v3_setup_pad(txfs);
	msleep(2);
}

static iomux_v3_cfg_t guf_cupid_ssi_gpio[] = {
	MX35_PAD_STXD4__GPIO2_28,
	MX35_PAD_STXFS4__GPIO2_31,
};
static iomux_v3_cfg_t guf_cupid_ssi_func[] = {
	MX35_PAD_STXD4__AUDMUX_AUD4_TXD,
	MX35_PAD_STXFS4__AUDMUX_AUD4_TXFS,
};

static void guf_cupid_ac97_cold_reset(struct snd_ac97 *ac97)
{
	iomux_v3_cfg_t reset_gpio_low = MX35_PAD_ATA_DATA14__IPU_CSI_D_0_PD;
	iomux_v3_cfg_t reset_gpio_high = MX35_PAD_ATA_DATA14__IPU_CSI_D_0_PU;
	int ret;

	pr_debug("<%s>\n", __func__);

	ret = gpio_request(AC97_GPIO_TXFS, "SSI");
	if (ret)
		goto err1;

	ret = gpio_request(AC97_GPIO_TXD, "SSI");
	if (ret)
		goto err2;

	ret = gpio_request(SPEAKER_ON, "SSI");
	if (ret) {
		goto err3;
	}

	ret = mxc_iomux_v3_setup_multiple_pads(guf_cupid_ssi_gpio, ARRAY_SIZE(guf_cupid_ssi_gpio));
	if( ret ) pr_warn("Cupid platform: Failed setup pads in %s\n", __func__);

	gpio_direction_output(AC97_GPIO_TXFS, 1);
	gpio_set_value(AC97_GPIO_TXFS, 0);
	gpio_direction_output(AC97_GPIO_TXD, 1);
	gpio_set_value(AC97_GPIO_TXD, 0);

	udelay(20);
	/* cold reset */
	ret = mxc_iomux_v3_setup_pad(reset_gpio_low);
	if( ret ) pr_warn("Cupid platform: Failed setup pads in %s\n", __func__);
	udelay(20);
	ret = mxc_iomux_v3_setup_pad(reset_gpio_high);
	if( ret ) pr_warn("Cupid platform: Failed setup pads in %s\n", __func__);
	udelay(20);
	
	ret = mxc_iomux_v3_setup_multiple_pads(guf_cupid_ssi_func, ARRAY_SIZE(guf_cupid_ssi_func));
	if( ret ) pr_warn("Cupid platform: Failed setup pads in %s\n", __func__);

	gpio_request(AC97_GPIO_INT, "SSI");
	gpio_direction_input(AC97_GPIO_INT);

err3:
	gpio_free(AC97_GPIO_TXD);
err2:
	gpio_free(AC97_GPIO_TXFS);
err1:
	if (ret)
		printk("%s failed with %d\n", __func__, ret);
	mdelay(1);
}

static struct imx_ssi_platform_data cupid_ssi_pdata = {
	.ac97_reset = guf_cupid_ac97_cold_reset,
	.ac97_warm_reset = guf_cupid_ac97_warm_reset,
	.toggle_speaker = guf_cupid_toggle_speaker,
	.flags = IMX_SSI_USE_AC97,
};

static iomux_v3_cfg_t guf_cupid_ssi_pads[] = {
	/* SSI */
	MX35_PAD_STXFS4__AUDMUX_AUD4_TXFS,
	MX35_PAD_STXD4__AUDMUX_AUD4_TXD,
	MX35_PAD_SRXD4__AUDMUX_AUD4_RXD,
	MX35_PAD_SCK4__AUDMUX_AUD4_TXC,
	/* UCB1400/WM97xx IRQ */
	MX35_PAD_ATA_INTRQ__GPIO2_29,
	/* Speaker On */
	MX35_PAD_LD20__GPIO3_26,
};

static struct cupid_imx_ac97_pdata cupid_ac97_pdata = {
	.irq_gpio	= IMX_GPIO_NR( 2, 29 ),
};

static struct platform_device cupid_imx_ac97  = {
	.name = "imx-cupid_ac97",
	.id	= -1,
	.dev = {
		.platform_data = &cupid_ac97_pdata,
	},
};
static struct platform_device ac97_codec  = {
	.name = "ac97-codec",
	.id	= -1,
};

static int cupid_sound_rtouch_init(struct guf_xml_data * xml_config)
{
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_ssi_pads, ARRAY_SIZE(guf_cupid_ssi_pads));
	imx35_add_imx_ssi(0, &cupid_ssi_pdata);
	platform_device_register(&ac97_codec);
	platform_device_register(&cupid_imx_ac97);

	return 0;
}

static void guf_cupid_init_touch_driver(struct guf_xml_data * xml_config)
{

	guf_xml_data_touch_t touch = xml_config->touch;
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_external_touch_pads, ARRAY_SIZE(guf_cupid_external_touch_pads));

	switch(touch) {
	case TOUCH_PINNACLE:
		/* Configure external touch pins */
		gpio_request(TOUCH_INT, "CTOUCH");
		gpio_direction_input(TOUCH_INT);
		gpio_request(TOUCH_RESET, "CTOUCH");
		gpio_set_value(TOUCH_RESET, 0);
		gpio_direction_output(TOUCH_RESET, 1);
		guf_cupid_touch_pinnacle_info[0].irq = gpio_to_irq(TOUCH_INT),
		i2c_register_board_info(2, guf_cupid_touch_pinnacle_info, ARRAY_SIZE(guf_cupid_touch_pinnacle_info));
		break;
	case TOUCH_EETI_I2C:
		/* Configure external touch pins */
		gpio_request(TOUCH_RESET, "EETI_I2C_RST");
		gpio_direction_output(TOUCH_RESET, 0);
		gpio_request(TOUCH_INT, "EETI_I2C_INT");
		gpio_direction_input(TOUCH_INT);
		guf_cupid_touch_eetii2c_info[0].irq = gpio_to_irq(TOUCH_INT);
		i2c_register_board_info(2, guf_cupid_touch_eetii2c_info, ARRAY_SIZE(guf_cupid_touch_eetii2c_info));
		break;
	case TOUCH_EDT_FT5X06:
		/* Driver configures pins itself */
		guf_cupid_touch_edt_info[0].irq = gpio_to_irq(TOUCH_INT);
		i2c_register_board_info(2, guf_cupid_touch_edt_info, ARRAY_SIZE(guf_cupid_touch_edt_info));
		break;
	case TOUCH_UCB1400:
		cupid_ac97_pdata.enable_ucb1400_ts = true;
		break;
	case TOUCH_WM9705:
		cupid_ac97_pdata.enable_wm9705_ts = true;
		break;
	case TOUCH_DA9052:
	case TOUCH_IMX_ADC:
	default:
		printk("Warning: Selected touch not supported on this platform\n");
	case TOUCH_NONE:
		printk("Booting kernel without touch support\n");
		break;
	}
}

/******************************************************************************/
/* USB                                                                        */
/******************************************************************************/
static uint32_t otg_host_enabled=1;

static int __init guf_cupid_otg_host_enabled(char *options)
{
	if (sscanf(options, "%d", &otg_host_enabled) != 1) {
		pr_err("CUPID platform: wrong otg_host_enabled parameter syntax. Ignoring.\n");
		otg_host_enabled = 0U;	/* to be sure */
	} else {
		pr_info("CUPID platform: OTG Host enabled.\n");
	}
	return 0;
}
__setup("otg_host_enabled=", guf_cupid_otg_host_enabled);


static int guf_cupid_usbh1_phy_init(struct platform_device *pdev)
{
	int ret = 0;

	pr_debug("Cupid platform: %s: ID: %d\n", __func__, pdev->id);
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_usbh1_pads, ARRAY_SIZE(guf_cupid_usbh1_pads));
	
	ret = gpio_request(USBH_PWR, "USB");
	if (ret)
		goto usb_out;

	gpio_direction_output(USBH_PWR, 1);
	udelay(10);
	gpio_set_value(USBH_PWR, 0);
	
	ret = mx35_initialize_usb_hw(pdev->id, MXC_EHCI_IPPUE_DOWN | MXC_EHCI_INTERNAL_PHY);

usb_out:
	if (ret)
		pr_warn( "Cupid platform: %s failed with %d\n", __func__, ret);
	return ret;
}

static struct mxc_usbh_platform_data guf_cupid_usbh1_pdata = {
	.init = guf_cupid_usbh1_phy_init,
	.portsc = MXC_EHCI_MODE_SERIAL,
};

static int guf_cupid_usb_otg_init(struct platform_device *pdev)
{
	int ret;
	pr_debug("Cupid platform: %s, ID: %d\n", __func__, pdev->id);
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_usb_otg_pads, ARRAY_SIZE(guf_cupid_usb_otg_pads));

	ret = mx35_initialize_usb_hw(pdev->id, MXC_EHCI_INTERNAL_PHY | MXC_EHCI_INTERFACE_DIFF_UNI);
	if (ret)
		pr_warn( "Cupid platform: %s failed with %d\n", __func__, ret);
	return ret;
}
static int guf_cupid_usb_otg_device_init(struct platform_device *pdev)
{
#define USBCTRL_OTGBASE_OFFSET	0x600
#define USBCTRL_OTG_CTRL	0x0
#define USBCTRL_OTG_PHYCTRL	0x8
		/* UTMI phy control */
#define  USB_UTMI_PHY_CTRL_VEND_STATUS		 0x000000FF
#define  USB_UTMI_PHY_CTRL_VEND_DATA		 0x0000FF00
#define  USB_UTMI_PHY_CTRL_VEND_ADDR		 0x000F0000
#define  USB_UTMI_PHY_CTRL_VEND_LOAD		 0x00100000
#define  USB_UTMI_PHY_CTRL_LSFE				 0x00400000
#define  USB_UTMI_PHY_CTRL_EVDO				 0x00800000
#define  USB_UTMI_PHY_CTRL_USBEN			 0x01000000
#define  USB_UTMI_PHY_CTRL_RESET			 0x02000000
#define  USB_UTMI_PHY_CTRL_SUSP				 0x04000000
#define  USB_UTMI_PHY_CTRL_CLK_VLD			 0x08000000

	// FIXME This depends on the read out hardware revision in ptxdist and is not set for 1.1.1
	unsigned int v;
	pr_debug("Cupid platform: %s, ID: %d\n", __func__, pdev->id);
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_usb_otg_pads, ARRAY_SIZE(guf_cupid_usb_otg_pads));		

	v = readl(MX35_IO_ADDRESS(MX35_USB_BASE_ADDR + USBCTRL_OTGBASE_OFFSET + USBCTRL_OTG_PHYCTRL));
	pr_debug("%s: phyctrl 0x%08x\n", __func__, v);
	v |= USB_UTMI_PHY_CTRL_EVDO;
	pr_debug("%s: ctrl 0x%08x\n", __func__, v);
	writel(v, MX35_IO_ADDRESS(MX35_USB_BASE_ADDR + USBCTRL_OTGBASE_OFFSET + USBCTRL_OTG_PHYCTRL));
	return 0;
}

static struct mxc_usbh_platform_data guf_cupid_usb_otg_host_pdata = {
	.init = guf_cupid_usb_otg_init,
	.portsc = MXC_EHCI_MODE_UTMI,
};
static const struct fsl_usb2_platform_data guf_cupid_otg_device_pdata __initconst = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
	.workaround	= FLS_USB2_WORKAROUND_ENGCM09152,
	.init = guf_cupid_usb_otg_device_init,
};

static int cupid_usb_init(void)
{
	if (otg_host_enabled)
		imx35_add_mxc_ehci_otg(&guf_cupid_usb_otg_host_pdata);
	else
	{
		imx35_add_fsl_usb2_udc(&guf_cupid_otg_device_pdata);
	}

	imx35_add_mxc_ehci_hs(&guf_cupid_usbh1_pdata);

	return 0;
}

/******************************************************************************/
/* SRAM                                                                       */
/******************************************************************************/
struct platdata_mtd_ram guf_cupid_sram_pdata = {
	.mapname	= "SRAM",
	.bankwidth	= 1,
};

static struct resource guf_cupid_sram_resource = {
	.start = MX35_CS0_BASE_ADDR,
	.end   = MX35_CS0_BASE_ADDR + 0x80000 - 1,
	.flags = IORESOURCE_MEM,
};

static struct platform_device guf_cupid_sram = {
	.name		= "mtd-ram",
	.id		= 0,
	.resource	= &guf_cupid_sram_resource,
	.num_resources	= 1,
	.dev	= {
		.platform_data = &guf_cupid_sram_pdata,
	},
};

/******************************************************************************/
/* FLASH                                                                      */
/******************************************************************************/
static struct mxc_nand_platform_data guf_cupid_nand_pdata = {
	.width = 1,
	.hw_ecc = 1,
};

/******************************************************************************/
/* Fast Ethernet                                                              */
/*                                                                            */
/* struct fec_platform_data {                                                 */
/* 	phy_interface_t phy;                                                      */
/* 	unsigned char mac[ETH_ALEN];                                              */
/* };                                                                         */
/*****************************************************************************/
static struct fec_platform_data guf_cupid_fec_pdata = {
	.phy = PHY_INTERFACE_MODE_RMII,
};
static int guf_cupid_fec_init(const struct guf_xml_data_network * const network_config )
{
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_fec_pads, ARRAY_SIZE(guf_cupid_fec_pads));
	if(guf_xml_get_mac_address( network_config, guf_cupid_fec_pdata.mac))
	{
		pr_warn("Cupid platform: cupid: MAC address not valid");
	}
	imx35_add_fec(&guf_cupid_fec_pdata);
	return 0;
}

/******************************************************************************/
/* SERIAL                                                                     */
/******************************************************************************/

static struct imxuart_platform_data guf_cupid_uart1_pdata = {
	.flags = IMXUART_HAVE_RTSCTS, 
};

static void guf_cupid_rs485_tx_enable (struct device *dev, int enable)
{
	pr_debug("Cupid platform: rs485: tx_enable %d\n", enable);
	gpio_set_value(RS485_TX_EN, enable);
}

static struct imxuart_platform_data guf_cupid_uart2_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static struct imxuart_platform_data guf_cupid_rs485_pdata = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_RS485 | IMXUART_RS485_HALFD,
	.rs485_tx_enable = guf_cupid_rs485_tx_enable,
};


static inline void guf_cupid_serial_init(void)
{
	struct platform_device * uart2;
	/* UART1 */
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_uart1_pads, ARRAY_SIZE(guf_cupid_uart1_pads));
	imx35_add_imx_uart(0, &guf_cupid_uart1_pdata);

	/* UART2 MDB */
	gpio_request(MDB_WAKEUP_OUT, "MDB");
	gpio_direction_output(MDB_WAKEUP_OUT, 0);
	gpio_export(MDB_WAKEUP_OUT, 0);

	mxc_iomux_v3_setup_multiple_pads(guf_cupid_uart2_pads, ARRAY_SIZE(guf_cupid_uart2_pads));
	uart2 = imx35_add_imx_uart_named(1, "imx-mdb-guf", &guf_cupid_uart2_pdata);
	// Create a symbolic link to the MDB wakeup GPIO pin under the according TTY device
	gpio_export_link( &uart2->dev, "mdb_wakeup_out", MDB_WAKEUP_OUT);

	/* UART3  RS485 */
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_uart3_pads, ARRAY_SIZE(guf_cupid_uart3_pads));
	gpio_request(RS485_TX_EN, "RS485_tx_enable");
	gpio_direction_output(RS485_TX_EN, 0);
	imx35_add_imx_uart(2, &guf_cupid_rs485_pdata);
	
}

/******************************************************************************/
/* FLEXCAN                                                                    */
/******************************************************************************/
static const struct flexcan_platform_data guf_cupid_flexcan0_pdata = {
	.transceiver_switch = NULL,
};
static const struct flexcan_platform_data guf_cupid_flexcan1_pdata = {
	.transceiver_switch = NULL,
};

static int guf_cupid_power_can_rs485(void)
{
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_can_rs485_pwr, ARRAY_SIZE(guf_cupid_can_rs485_pwr));
	/* Request CAN_RS485_PWR_EN */
	gpio_request(CAN_RS485_PWR_EN, "CAN/RS485 POWER"); /* FIXME: Should we do that here ? */
	/* Enable CAN / RS458 PWR */
	gpio_direction_output(CAN_RS485_PWR_EN, 1);
	udelay(10);
	return 0;
}

static int guf_cupid_flexcan_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_flexcan_pads, ARRAY_SIZE(guf_cupid_flexcan_pads));

	imx35_add_flexcan(0, &guf_cupid_flexcan0_pdata);
	imx35_add_flexcan(1, &guf_cupid_flexcan1_pdata);
	return 0;
}

/******************************************************************************/
/* Digital I/O                                                                */
/******************************************************************************/

/* These GPIO pins will be visible via sysfs and have a fixed direction */
static struct gpio guf_cupid_ext_gpios[] = {
	{ DIG_IN1,			GPIOF_ACT_LOW|GPIOF_IN,					"DIG_IN1" },
	{ DIG_IN2,			GPIOF_ACT_LOW|GPIOF_IN,					"DIG_IN2" },
	{ DIG_OUT1,			GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"DIG_OUT1" },
	{ DIG_OUT2,			GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"DIG_OUT2" },
};

static int guf_cupid_set_gpio_names(void)
{
	int i;
	for (i = 0; i < (NUM_GPIO_CHIPS); i++){
		const char * const * names = &guf_cupid_gpio_names[ i * NUM_GPIOS_PER_CHIP];
		gpio_to_chip(i * NUM_GPIOS_PER_CHIP)->names  = names;
		pr_debug("gpio: set chipname %d, %0x\n", i, (int)names );
	}
	return 0;
}

static int guf_cupid_dio_init(void)
{
	int ret;

	ret = mxc_iomux_v3_setup_multiple_pads(guf_cupid_dio_pads, ARRAY_SIZE(guf_cupid_dio_pads));
	if( ret )
	{
		pr_warn("Cupid platform: cupid: Failed setup pads for external gpios\n");
		return ret;
	}

	ret = guf_init_gpio_group(guf_cupid_ext_gpios, ARRAY_SIZE(guf_cupid_ext_gpios), true, true);
	if( ret ) 
	{
		pr_warn("Cupid platform: cupid: Failed initialize external gpios\n");
		return ret;
	}
	return 0;
}

/******************************************************************************/
/* KEYPAD		                                                              */
/******************************************************************************/
static uint32_t keypad_enabled;

static int __init guf_cupid_keypad_enabled(char *options)
{
	if (sscanf(options, "%d", &keypad_enabled) != 1) {
		pr_err("Cupid platform: wrong keypad_enabled parameter syntax. Ignoring.\n");
		keypad_enabled = 0U;	/* to be sure */
	}
	return 0;
}

__setup("keypad_enabled=", guf_cupid_keypad_enabled);

static const unsigned int keypad_row_gpios[] = {
	KP_ROW0,
	KP_ROW1,
	KP_ROW2,
	KP_ROW3,
	KP_ROW4,
	KP_ROW5_DMA,
	KP_ROW6_MISO,
	KP_ROW7_SLK,
};

static const unsigned int keypad_col_gpios[] = {
	KP_COL0,
	KP_COL1,
	KP_COL2,
	KP_COL3,
	KP_COL4,
	KP_COL5_SS1,
	KP_COL6_MOSI,
	KP_COL7_SS0,
};

/* Test mapping, needs to be adapted to the actual keypad. 
   There is no dynamic configuration available yet	        */
static const uint32_t guf_cupid_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(0, 1, KEY_2),
	KEY(0, 2, KEY_3),
	KEY(1, 0, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(1, 2, KEY_6),
	KEY(2, 0, KEY_7),
	KEY(2, 1, KEY_8),
	KEY(2, 2, KEY_9),
	KEY(3, 0, KEY_A),
	KEY(3, 1, KEY_B),
	KEY(3, 2, KEY_C),
};

/* These GPIO pins will be visible via sysfs if the keypad is not enabled and have a configurable direction */
#define KP_GPIO_FLAGS (GPIOF_ACT_LOW|GPIOF_EXPORT_DIR_CHANGEABLE|GPIOF_OUT_INIT_HIGH)

static const struct matrix_keymap_data guf_cupid_keymap_data = {
	.keymap			= guf_cupid_keymap,
	.keymap_size	= ARRAY_SIZE(guf_cupid_keymap),
};

static struct matrix_keypad_platform_data guf_cupid_matrix_keypad_pdata = {
	.keymap_data 		= &guf_cupid_keymap_data,
	.row_gpios 			= keypad_row_gpios,
	.col_gpios 			= keypad_col_gpios,
	.num_row_gpios 		= ARRAY_SIZE(keypad_row_gpios),
	.num_col_gpios 		= ARRAY_SIZE(keypad_col_gpios),
	.col_scan_delay_us 	= 10,
	.debounce_ms 		= 100,
	.active_low			= 1,
	.wakeup 			= 1,
	.no_autorepeat 		= 1,
};

static struct platform_device guf_cupid_keypad_device = {
	.name	= "matrix-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &guf_cupid_matrix_keypad_pdata,
	},
};

static int export_kp_gpio(int gpio)
{
	int err;

	err = gpio_request_one(gpio, KP_GPIO_FLAGS, guf_cupid_gpio_names[gpio]);
	if( err)
	{
		pr_err("Cupid platform: Failed to request gpio %d, %d\n", gpio, err);
		return err;
	}
	return 0;
}

static int guf_cupid_keypad_init(void)
{
	int i, ret;
	ret = mxc_iomux_v3_setup_multiple_pads(guf_cupid_keypad_pads, ARRAY_SIZE(guf_cupid_keypad_pads));
	if( ret )
	{
		pr_warn("Cupid platform: cupid: Failed setup pads for external keypad\n");
		return ret;
	}

	if(keypad_enabled)
	{
		pr_info("Cupid platform: Keypad enabled\n");
		platform_device_register(&guf_cupid_keypad_device);
	}
	else
	{
		pr_info("Cupid platform: Keypad not enabled, export kp pins as gpios\n");
		for(i = 0; i < ARRAY_SIZE(keypad_col_gpios); i++)
			export_kp_gpio( keypad_col_gpios[i]);
		for(i = 0; i < ARRAY_SIZE(keypad_row_gpios); i++)
			export_kp_gpio( keypad_row_gpios[i]);
	}
	return 0;
}

/******************************************************************************/
/* LEDs                                                                       */
/******************************************************************************/
static const struct gpio_led guf_cupid_gpio_leds[] = {
	{
		.name			= "led1",
		.default_trigger= "none",
		.active_low		= 0,
		.gpio			= LED1_GPIO,
	},
};

static struct gpio_led_platform_data guf_cupid_gpio_led_info = {
	.leds		= guf_cupid_gpio_leds,
	.num_leds	= ARRAY_SIZE(guf_cupid_gpio_leds),
};


/******************************************************************************/
/* LCD POWER                                                                  */
/******************************************************************************/
static void guf_cupid_lcd_power_control(struct plat_lcd_data *pd, unsigned int power)
{
	if (power != 0)
		gpio_set_value(LCD_ENA, 1);	/* on */
	else
		gpio_set_value(LCD_ENA, 0);	/* off */
}

static struct plat_lcd_data guf_cupid_lcd_power_info = {
	.set_power = guf_cupid_lcd_power_control,
};

static struct platform_device guf_cupid_lcd_power = {
	.name	= "platform-lcd",
	.id	= -1,
	.dev	= {
		.platform_data	= &guf_cupid_lcd_power_info,
	},
};

/******************************************************************************/
/* BACKLIGHT                                                                  */
/******************************************************************************/
static struct platform_pwm_backlight_data guf_cupid_backlight_info = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns = 4000000,
	.levels = 0,
};

static struct platform_device guf_cupid_backlight_power = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data	= &guf_cupid_backlight_info,
	},
};
extern const struct imx_mxc_pwm_data imx35_mxc_pwm_data;

/******************************************************************************/
/* VIDEO                                                                      */
/******************************************************************************/
#ifdef CONFIG_LOGO_LINUX_PNG
static guf_xml_data_logo_license_t * cupid_logo_license;
static void cupid_show_logo(struct fb_info *fbi)
{
	pr_info("%s\n",__func__);
	//FIXME Rotation is missing
	fb_show_logo(fbi, 0, cupid_logo_license->valid ?cupid_logo_license->license: 0);
}
#endif

static struct fb_videomode guf_cupid_fb_mode;
static struct mx3fb_platform_data guf_cupid_mx3fb_pdata = {
	.mode		= &guf_cupid_fb_mode,
	.num_modes	= 1,
	.backlight_dev = &guf_cupid_backlight_power.dev,
	.lcd_dev = &guf_cupid_lcd_power.dev,
#ifdef CONFIG_LOGO_LINUX_PNG
	.fb_initialized = cupid_show_logo,
#endif
};

static uint32_t fixed_screen_address;
static uint32_t fixed_screen_size;

static int __init guf_cupid_fixed_screen(char *options)
{
	int display_enabled;

	if (sscanf(options, "%d,%x,%x", &display_enabled, &fixed_screen_address, &fixed_screen_size) != 3) {
		pr_err("Cupid platform: wrong vidmem parameter syntax. Ignoring.\n");
		fixed_screen_address = 0U;	/* to be sure */
		return 0;
	}
	/* nothing shown yet? e.g. no splash screen */
	if (display_enabled == 0)
		/* do not use any hard coded framebuffer address in this case */
		fixed_screen_address = 0U;
	else
		pr_info("Cupid platform: Using fixed framebuffer memory at %x\n",
			fixed_screen_address);

	return 0;
}
__setup("vidmem=", guf_cupid_fixed_screen);

static void guf_cupid_setup_fb_videomode( struct guf_xml_data * xml_config, struct mx3fb_platform_data * fb )
{
	struct fb_videomode * mode = ( struct fb_videomode * ) fb->mode;
	fb->name = xml_config->display.name;
	fb->bpp = xml_config->format.depth;
	mode->xres = xml_config->display.xres;
	mode->yres = xml_config->display.yres;
	if (xml_config->display.pix_clk == 0) 
	{
		mode->pixclock = 
					(   xml_config->display.xres 
					  + xml_config->hsync.start_width
					  + xml_config->hsync.width 
					  + xml_config->hsync.end_width )
				*   (   xml_config->display.yres 
				      + xml_config->vsync.start_width 
					  + xml_config->vsync.width
					  + xml_config->vsync.end_width ) 
				*       xml_config->display.refresh / 1000;
	} else {
		mode->pixclock = xml_config->display.pix_clk / 1000;
	}
	mode->hsync_len = xml_config->hsync.width;
	mode->vsync_len = xml_config->vsync.width;
	mode->left_margin = xml_config->hsync.start_width;
	mode->right_margin = xml_config->hsync.end_width;
	mode->upper_margin = xml_config->vsync.start_width;
	mode->lower_margin = xml_config->vsync.end_width;
	mode->refresh = xml_config->display.refresh;

	/* change the interpratation of "polarity" and "select_enable" dependent
	   on "original_dc" */
	if (xml_config->display.original_dc != NULL)
	{
		if (!strcmp(xml_config->display.original_dc, "CLCDC")) {
			/* IPU compared to CLCDC uses inverted clock.select_enable and clock.polarity */
			xml_config->clock.select_enable = (xml_config->clock.select_enable == 0);
			xml_config->clock.polarity = (xml_config->clock.polarity == 0);
		}
	}
	else
	{
		pr_err("Cupid platform: guf_xml: Your display settings are outdated, please update. System halted.\n");
		while(1);
	}
	mode->sync =  (xml_config->clock.idle_enable ? FB_SYNC_CLK_IDLE_EN: 0) |
		(xml_config->data.oe_polarity ? FB_SYNC_OE_ACT_HIGH : 0) |
		(xml_config->clock.polarity ? FB_SYNC_CLK_INVERT : 0) |
		(xml_config->clock.select_enable ? FB_SYNC_CLK_SEL_EN : 0) |
		(xml_config->hsync.polarity ? 0: FB_SYNC_HOR_HIGH_ACT ) |
		(xml_config->vsync.polarity ? FB_SYNC_VERT_HIGH_ACT : 0) |
		(xml_config->data.polarity ? FB_SYNC_DATA_INVERT : 0);
	mode->vmode = FB_VMODE_NONINTERLACED;
	mode->flag = 0;
	fb->num_modes = 1;

	/* If the bootloader provides a framebuffer, we will continue to use it */
	if (fixed_screen_address != 0U) {
		if( 0 == request_mem_region(fixed_screen_address, fixed_screen_size, "mx3_sdc_fb"))
		{
			pr_err("Cupid platform: failed to reserve frame buffer memory setup by the bootloader\n");
			//\FIXME Handle this
		}
		fb->fixed_screen_cpu = ioremap(fixed_screen_address, fixed_screen_size);
		fb->fixed_screen_dma = fixed_screen_address;
	}

	/* Set up the display power sequence */
	fb->poweron_to_signalon = xml_config->powerseq.poweron_to_signalon;
	fb->poweron_to_backlighton = xml_config->powerseq.poweron_to_backlighton;
	fb->backlightoff_before_poweroff = xml_config->powerseq.backlightoff_before_poweroff;
	fb->signaloff_before_poweroff = xml_config->powerseq.signaloff_before_poweroff;
	fb->poweroff_to_poweron = xml_config->powerseq.poweroff_to_poweron;


#ifdef CONFIG_FRAMEBUFFER_CONSOLE
#warning FBCON
	/* Adjust frambuffer console rotation */
	switch (xml_config->rotation.angle)
	{
		default:
		case ROTATION_0:
			fbcon_change_rotation(2);
		case ROTATION_90:
			fbcon_change_rotation(3);
		case ROTATION_180:
			fbcon_change_rotation(0);
		case ROTATION_270:
			fbcon_change_rotation(1);
	}
#endif
}

/******************************************************************************/
/* Board specific display initialization                                      */
/******************************************************************************/
int guf_cupid_display_init(struct guf_xml_data * xml_config)
{
	struct platform_device * pwm_device;
	struct platform_device * cupid_ipu_pdev;
	struct platform_device * cupid_fb_pdev;

	mxc_iomux_v3_setup_multiple_pads(guf_cupid_display_pads, ARRAY_SIZE(guf_cupid_display_pads));

	if( !xml_config->display.valid )
	{
		pr_info("Cupid platform: Booting kernel without display support\n");
		return 1;
	}

	//***************************************************************
	// LCD Power
	//***************************************************************
	gpio_request(LCD_ENA, "FB");
	platform_device_register(&guf_cupid_lcd_power);

	//***************************************************************
	// Backlight
	//***************************************************************
	pwm_device = imx_add_mxc_pwm(&imx35_mxc_pwm_data);
	guf_cupid_backlight_power.dev.parent = &pwm_device->dev;
	/* default brightness */
	guf_cupid_backlight_info.dft_brightness = xml_config->backlight.level_ac;
	/* configure backlight */
	if (xml_config->backlight.frequency) {
		guf_cupid_backlight_info.pwm_period_ns = 1000000000u / xml_config->backlight.frequency;
	}
	guf_cupid_backlight_info.levels = xml_config->backlight.lut; // set look-up table
	guf_cupid_backlight_info.max_level = 0xFFFF;
	guf_cupid_backlight_info.dft_brightness = xml_config->backlight.level_ac; // set level
	mxc_iomux_v3_setup_pad((backlight_pwm_pad & ~MUX_PAD_CTRL_MASK) | ((iomux_v3_cfg_t)xml_config->backlight.padctl << MUX_PAD_CTRL_SHIFT));
	platform_device_register(&guf_cupid_backlight_power);

	//***************************************************************
	// Video
	// Register IPU and framebuffer device 
	//***************************************************************
	cupid_ipu_pdev = imx35_add_ipu_core();
	guf_cupid_setup_fb_videomode( xml_config, &guf_cupid_mx3fb_pdata);
	guf_cupid_mx3fb_pdata.dma_dev = &cupid_ipu_pdev->dev;
	cupid_fb_pdev = imx35_add_mx3_sdc_fb(&guf_cupid_mx3fb_pdata);

	return 0;
}

/******************************************************************************/
/* SDHC                                                                       */
/******************************************************************************/
static struct regulator_consumer_supply vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx35.0"),
	REGULATOR_SUPPLY("vqmmc", "sdhci-esdhc-imx35.0"),
};

const struct esdhc_platform_data guf_cupid_esdhc_pdata = {
	.wp_gpio = IMX_GPIO_NR(2, 19),
	.cd_gpio = IMX_GPIO_NR(2, 18),
	.wp_type = ESDHC_WP_GPIO,
	.cd_type = ESDHC_CD_GPIO,
	.support_vsel = false,
};

int	cupid_sdhc_init(void)
{
	int ret;
	ret = mxc_iomux_v3_setup_multiple_pads(guf_cupid_esdhc_pads, ARRAY_SIZE(guf_cupid_esdhc_pads));
	if( ret )
	{
		pr_warn("Cupid platform: Failed setup pads for sdhc\n");
	}
	regulator_register_always_on(3, "3v", vmmc_consumers, ARRAY_SIZE(vmmc_consumers), 3000000);
	imx35_add_sdhci_esdhc_imx( 0, &guf_cupid_esdhc_pdata );
	return ret;
}

/******************************************************************************/
/* guf_xml in sys fs                                                          */
/******************************************************************************/
static int guf_xml_device_registered = 0;
static struct guf_xml_device_platform_data guf_xml_device_pdata = {
	.mtd_fis_directory = NULL,
	.mtd_redundant_fis = NULL,
};
struct platform_device guf_xml_device = {
	.name   = "guf_xml",
	.id     = -1,
};
static void cupid_guf_flash_add(struct mtd_info *mtd)
{
	if (strncmp(NAME_FIS_DIRECTORY, mtd->name, LENGTH_NAME_FIS_DIRECTORY) == 0) {
		pr_info("guf_xml: found partition %s\n", mtd->name);
		guf_xml_device_pdata.mtd_fis_directory = mtd;
	}

	if (strncmp(NAME_REDUNDANT_FIS, mtd->name, LENGTH_NAME_REDUNDANT_FIS) == 0) {
		pr_info("guf_xml: found partition %s\n", mtd->name);
		guf_xml_device_pdata.mtd_redundant_fis = mtd;
	}

	if (guf_xml_device_pdata.mtd_fis_directory && guf_xml_device_pdata.mtd_redundant_fis && !guf_xml_device_registered) {
		guf_xml_device.dev.platform_data = &guf_xml_device_pdata;
		platform_device_register(&guf_xml_device);
		guf_xml_device_registered = 1;
	}
}
static struct mtd_notifier cupid_guf_flash_notifier = {
	.add = cupid_guf_flash_add,
};


/******************************************************************************/
/* Board specific initialization                                              */
/******************************************************************************/
static void __init mx35_cupid_board_init(void)
{
	struct guf_xml_data * xml_config = 0;

	imx35_soc_init();

	/* RAM / FLASH */
	platform_device_register(&guf_cupid_sram);
	imx35_add_mxc_nand(&guf_cupid_nand_pdata);
	register_mtd_user(&cupid_guf_flash_notifier);

	xml_config = guf_parse_xml_parameters();

#ifdef CONFIG_LOGO_LINUX_PNG
	cupid_logo_license = &xml_config->logo_license;
#endif
	guf_cupid_init_touch_driver( xml_config);
	
	guf_cupid_set_gpio_names();

	/* DIO */
	guf_cupid_dio_init();
		
	/* Keypad */
	guf_cupid_keypad_init();
	
	/* LEDs */
	mxc_iomux_v3_setup_multiple_pads(guf_cupid_led_pads, ARRAY_SIZE(guf_cupid_led_pads));
	gpio_led_register_device(-1, &guf_cupid_gpio_led_info);
	
	/* FEC */
	guf_cupid_fec_init(xml_config?&xml_config->network:0);

	/* UART */
	guf_cupid_serial_init();	
	/* CAN */
	guf_cupid_flexcan_init();

	/* Power can and rs485 */
	guf_cupid_power_can_rs485();

	/* Display */
	guf_cupid_display_init(xml_config);

	/* USB */
	cupid_usb_init();

	/* I2C */
	cupid_i2c_init();
	
	/* SDHC */
	cupid_sdhc_init();

	watchdog_init();

	/* Audio */
	cupid_sound_rtouch_init(xml_config);
	
}

static void __init  mx35_cupid_init_late(void)
{
	//FIXME is this needed ??
	imx35_cpuidle_init();
}

static void __init guf_cupid_timer_init(void)
{
	mx35_clocks_init();
}

static void __init guf_cupid_reserve(void)
{
}

MACHINE_START(GUF_CUPID, "Garz & Fricke CUPID")
	/* Maintainer: Garz & Fricke GmbH */
	.atag_offset = 0x10000,
	.map_io = mx35_map_io,
	.init_early = imx35_init_early,
	.init_irq = mx35_init_irq,
	.handle_irq = imx35_handle_irq,
	.init_time = guf_cupid_timer_init,
	.init_machine = mx35_cupid_board_init,
	.init_late      = mx35_cupid_init_late,
	.reserve = guf_cupid_reserve,
	.restart	= mxc_restart,
MACHINE_END
