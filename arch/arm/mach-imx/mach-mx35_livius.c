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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/memblock.h>

#include "hardware.h"
#include "common.h"
#include "iomux-mx35.h"
#include "devices-imx35.h"
#include "../../../drivers/staging/iio/adc/ltc1854.h"
#include "cpuidle.h"
#include "guf.h"


#define SION_BIT_ENABLED(iomux) \
	(iomux) | ((iomux_v3_cfg_t)IOMUX_CONFIG_SION << MUX_MODE_SHIFT)

/* FIXME: number of chips times GPIOs per chip */
#define NUM_GPIO_CHIPS 		3
#define NUM_GPIOS_PER_CHIP 	32
	
/******************************************************************************/
/* GPIO DEFINES                                                               */
/******************************************************************************/

/* UART */
#define RS232_DRV_EN 		IMX_GPIO_NR(1, 19)
#define MUX_RS485_RS232		IMX_GPIO_NR(2, 21)
#define RS485_TX_EN			IMX_GPIO_NR(2, 22)
#define RS485_FD_EN			IMX_GPIO_NR(1, 30)
#define VCC_RS232_LWS_EN	IMX_GPIO_NR(3, 1) 	

/* Flexcan */
#define VCC_CAN_EN 		IMX_GPIO_NR(3, 2)

/* USB */
#define USBH_PWR		IMX_GPIO_NR(2, 26)

/* GSM */
#define GSM_EN			IMX_GPIO_NR(3, 4)

/* Analog IO */
#define ANALOG_OUT_EN	IMX_GPIO_NR(3, 0)

/* Digital IO */
#define DIG_IN1			IMX_GPIO_NR(1, 0)
#define DIG_IN2			IMX_GPIO_NR(1, 1)
#define DIG_IN3			IMX_GPIO_NR(1, 2)
#define DIG_IN4			IMX_GPIO_NR(1, 3)
#define DIG_OUT1		IMX_GPIO_NR(1, 4)
#define DIG_OUT2		IMX_GPIO_NR(1, 5)
#define DIG_OUT3		IMX_GPIO_NR(2, 3)
#define DIG_OUT4		IMX_GPIO_NR(2, 15)

/* Keyboard */
#define KB_COL0		IMX_GPIO_NR(2, 12)
#define KB_COL1		IMX_GPIO_NR(2, 13)
#define KB_COL2		IMX_GPIO_NR(2, 14)
#define KB_ROW0		IMX_GPIO_NR(2, 0)
#define KB_ROW1		IMX_GPIO_NR(2, 1)
#define KB_ROW2		IMX_GPIO_NR(2, 2)

/* LEDs */
#define KB_LED_EN	IMX_GPIO_NR(2, 10)
#define KB_LED1		IMX_GPIO_NR(1, 22)
#define KB_LED2		IMX_GPIO_NR(1, 23)
#define KB_LED3		IMX_GPIO_NR(1, 24)
#define KB_LED4		IMX_GPIO_NR(1, 25)
#define KB_LED5		IMX_GPIO_NR(1, 26)
#define KB_LED6		IMX_GPIO_NR(1, 27)
#define KB_LED7		IMX_GPIO_NR(1, 29)

/* Power Management */
#define VBATT_EN		IMX_GPIO_NR(3, 3)
#define VBAK5V_EN		IMX_GPIO_NR(2, 17)
#define VBAK5V_DIS		IMX_GPIO_NR(2, 16)
#define RUN_VBAK5V		IMX_GPIO_NR(2, 11) 
#define SHDN5V			IMX_GPIO_NR(2, 29)
#define RTC_INT			IMX_GPIO_NR(2, 31)
#define SW_RESET		IMX_GPIO_NR(1, 6)
#define CHARGE_STAT1	IMX_GPIO_NR(1, 8)
#define CHARGE_DONE		IMX_GPIO_NR(1, 9)
#define CHARGE_DISABLE	IMX_GPIO_NR(2, 30)

/******************************************************************************/
/* PIN MULTIPLEXING                                                           */
/******************************************************************************/
static iomux_v3_cfg_t guf_livius_dio_pads[] = {
	/* DIO */
	MX35_PAD_STXD5__GPIO1_0,
	MX35_PAD_SRXD5__GPIO1_1,
	MX35_PAD_SCK5__GPIO1_2,
	MX35_PAD_STXFS5__GPIO1_3,
	SION_BIT_ENABLED(MX35_PAD_SCKR__GPIO1_4),
	SION_BIT_ENABLED(MX35_PAD_FSR__GPIO1_5),
	SION_BIT_ENABLED(MX35_PAD_SD2_DATA1__GPIO2_3),
	SION_BIT_ENABLED(MX35_PAD_LD15__GPIO2_15),
};

static iomux_v3_cfg_t guf_livius_uart1_pads[] = {
	/* UART 1 */
	MX35_PAD_CTS1__UART1_CTS,
	MX35_PAD_RTS1__UART1_RTS,
	MX35_PAD_TXD1__UART1_TXD_MUX,
	MX35_PAD_RXD1__UART1_RXD_MUX,
	MX35_PAD_CSPI1_SS1__GPIO1_19,	/* RS232_DRV_EN */
	MX35_PAD_ATA_DATA8__GPIO2_21,	/* MUX_RS485_RS232 */
	MX35_PAD_ATA_DATA9__GPIO2_22,	/* RS485_TX_EN */
	MX35_PAD_CSI_HSYNC__GPIO1_30,	/* RS485_FD_EN */
	MX35_PAD_ATA_DA1__GPIO3_1,		/* VCC_RS232_LWS_EN */
};

static iomux_v3_cfg_t guf_livius_uart2_pads[] = {
	/* UART 2 */
	MX35_PAD_CTS2__UART2_CTS,
	MX35_PAD_RTS2__UART2_RTS,
	MX35_PAD_TXD2__UART2_TXD_MUX,
	MX35_PAD_RXD2__UART2_RXD_MUX,
	MX35_PAD_TX0__UART2_DCD,
	MX35_PAD_TX1__UART2_RI,
	MX35_PAD_TX4_RX1__UART2_DSR,
	MX35_PAD_TX5_RX0__UART2_DTR,
	MX35_PAD_MLB_CLK__GPIO3_3, 		/* VBATT_EN */
	MX35_PAD_MLB_DAT__GPIO3_4, 		/* GSM_EN */
};

static iomux_v3_cfg_t guf_livius_uart3_pads[] = {
	/* UART 3 */
	MX35_PAD_ATA_DATA11__UART3_TXD_MUX,
	MX35_PAD_ATA_DATA10__UART3_RXD_MUX,
};

static iomux_v3_cfg_t guf_livius_flexcan_pads[] = {
	/* Flexcan */
	MX35_PAD_SD2_DATA2__CAN1_RXCAN,
	MX35_PAD_SD2_DATA3__CAN1_TXCAN,
	MX35_PAD_ATA_DA2__GPIO3_2,		/* VCC_CAN_EN */	
};

static iomux_v3_cfg_t guf_livius_i2c1_pads_func[] = {
	/* I2C 1 */
	MX35_PAD_I2C1_CLK__I2C1_SCL,
	MX35_PAD_I2C1_DAT__I2C1_SDA
};

static iomux_v3_cfg_t guf_livius_i2c3_pads_func[] = {
	/* I2C 3 */
	MX35_PAD_TX3_RX2__I2C3_SCL,
	MX35_PAD_TX2_RX3__I2C3_SDA,
	MX35_PAD_ATA_DA0__GPIO3_0, 
};

static iomux_v3_cfg_t guf_livius_spi0_pads_func[] = {
	/* SPI 0 */
	MX35_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX35_PAD_CSPI1_MISO__CSPI1_MISO,
	MX35_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX35_PAD_CSPI1_SS0__CSPI1_SS0,
	MX35_PAD_CS5__GPIO1_21, 			/* CONVST */
	MX35_PAD_CSI_PIXCLK__GPIO1_31, 		/* NEG_BUSY (RDY?)*/
};

static iomux_v3_cfg_t guf_livius_fec_pads[] = {
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

static iomux_v3_cfg_t guf_livius_usbh_pads[] = {
	/* USB Host */
	MX35_PAD_I2C2_CLK__GPIO2_26,		/* USB Host PWR */
	MX35_PAD_I2C2_DAT__GPIO2_27,		/* USB Host Overcurrent */
};

static iomux_v3_cfg_t guf_livius_usb_otg_pads[] = {
	/* USB OTG */
	MX35_PAD_USBOTG_PWR__USB_TOP_USBOTG_PWR,
	MX35_PAD_USBOTG_OC__USB_TOP_USBOTG_OC,
};

static iomux_v3_cfg_t guf_livius_esdhc_pads[] = {
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

static iomux_v3_cfg_t guf_livius_keypad_pads[] = {
	/* KEYPAD */
	MX35_PAD_ATA_IORDY__GPIO2_12,
	MX35_PAD_ATA_DATA0__GPIO2_13,
	MX35_PAD_ATA_DATA1__GPIO2_14,
	MX35_PAD_SD2_CLK__GPIO2_1,
	MX35_PAD_SD2_CMD__GPIO2_0,
	MX35_PAD_SD2_DATA0__GPIO2_2,
};

static iomux_v3_cfg_t guf_livius_led_pads[] = {
	/* LEDs*/
	MX35_PAD_ATA_DMACK__GPIO2_10,		/* KB_LED_EN */
	MX35_PAD_CSI_VSYNC__GPIO1_29,
	MX35_PAD_CSI_D15__GPIO1_27,
	MX35_PAD_CSI_D14__GPIO1_26,
	MX35_PAD_CSI_D13__GPIO1_25,
	MX35_PAD_CSI_D12__GPIO1_24,
	MX35_PAD_CSI_D11__GPIO1_23,
	MX35_PAD_CSI_D10__GPIO1_22,
};

static iomux_v3_cfg_t guf_livius_pm_pads[] = {
	/* Power Management */
	SION_BIT_ENABLED(MX35_PAD_ATA_DATA4__GPIO2_17),
	SION_BIT_ENABLED(MX35_PAD_ATA_DATA3__GPIO2_16),
	SION_BIT_ENABLED(MX35_PAD_ATA_INTRQ__GPIO2_29),
	MX35_PAD_ATA_DMARQ__GPIO2_31,
	SION_BIT_ENABLED(MX35_PAD_WDOG_RST__GPIO1_6),
	MX35_PAD_FST__GPIO1_8,
	MX35_PAD_HCKT__GPIO1_9,	
	SION_BIT_ENABLED(MX35_PAD_ATA_BUFF_EN__GPIO2_30),
	MX35_PAD_LD11__GPIO2_11,
};

/******************************************************************************/
/* Power Management                                                           */
/******************************************************************************/

/* RTC wakeup input */
static const struct gpio_keys_button livius_buttons[] = {
	{
		.gpio		= RTC_INT,
		.code		= KEY_POWER,
		.desc		= "PWR",
		.active_low	= 1,
		.wakeup		= 1,
	},
};

static const struct gpio_keys_platform_data livius_button_data = {
	.buttons        = (struct gpio_keys_button *)livius_buttons,
	.nbuttons       = ARRAY_SIZE(livius_buttons),
};

static void guf_livius_pm_init(void)
{	
	mxc_iomux_v3_setup_multiple_pads(guf_livius_pm_pads, ARRAY_SIZE(guf_livius_pm_pads));
	imx_add_gpio_keys(&livius_button_data);
}

/******************************************************************************/
/* I2C 1 / EEPROM / RTC                                                       */
/******************************************************************************/
static struct imxi2c_platform_data guf_livius_i2c1_pdata = {
	.bitrate = 400000,
};

static struct guf_eeprom_memory_accessor guf_eeprom_acc;
static void guf_eeprom_setup(struct memory_accessor *mem_acc, void *context)
{
	guf_eeprom_acc.mem_acc.read = mem_acc->read;
	guf_eeprom_acc.mem_acc.write = mem_acc->write;
	guf_eeprom_acc.cont = context;
}

static struct gfeeprom_platform_data guf_eeprom_pdata = {
	.byte_len = (1 << 12),
	.page_size = 0x20,
	.flags = AT24_FLAG_ADDR16,
	.bus_id = 0,
	.context = &(guf_eeprom_pdata.bus_id),
	.setup = guf_eeprom_setup,
};

static struct i2c_board_info guf_livius_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("gfeeprom", GF_GLOBAL_PLATFORM_EEPROM_ADDRESS), /* EEPROM */
		.platform_data = &guf_eeprom_pdata,
	},
	{
		I2C_BOARD_INFO("pcf8563", 0x51),		/* RTC */
	}
};

static int guf_livius_i2c1_eeprom_rtc_init(struct platform_device *pdev)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_i2c1_pads_func, ARRAY_SIZE(guf_livius_i2c1_pads_func));
	i2c_register_board_info(0, guf_livius_i2c1_devices, ARRAY_SIZE(guf_livius_i2c1_devices));
	return 0;
}

/******************************************************************************/
/* I2C 3 / TempSensor / DAC                                                   */
/******************************************************************************/
static struct imxi2c_platform_data guf_livius_i2c3_pdata = {
	.bitrate = 400000,
};

static struct regulator_consumer_supply fixed_4v096_supplies[] = {
	REGULATOR_SUPPLY("vref", "2-000c"),
};

static struct i2c_board_info guf_livius_i2c3_devices[] = {
	{
		I2C_BOARD_INFO("lm73", 0x48),		/* TempSensor */
	},
	{
		I2C_BOARD_INFO("ad5338", 0x0C),		/* DAC */
	},
};

static int guf_livius_i2c3_temp_init(struct platform_device *pdev)
{
	int ret;
	mxc_iomux_v3_setup_multiple_pads(guf_livius_i2c3_pads_func, ARRAY_SIZE(guf_livius_i2c3_pads_func));
	
	ret = gpio_request(ANALOG_OUT_EN, "ANALOG_OUT_EN");
	if (ret) {
		printk(KERN_WARNING "%s failed with %d\n", __func__, ret);
		return ret;
	}
	gpio_direction_output(ANALOG_OUT_EN, 1);
	
	i2c_register_board_info(2, guf_livius_i2c3_devices, ARRAY_SIZE(guf_livius_i2c3_devices));
	
	regulator_register_always_on(0, "4v096", fixed_4v096_supplies, 1, 4096000);
	return 0;
}

/******************************************************************************/
/* SPI 0 / ADC                                                                */
/******************************************************************************/
static int guf_livius_spi0_chipselect[] = {
	MXC_SPI_CS(0),
};

static struct spi_imx_master guf_livius_spi0_pdata = {
	.chipselect = guf_livius_spi0_chipselect,
	.num_chipselect = sizeof(guf_livius_spi0_chipselect),
};

static struct regulator_consumer_supply fixed_5v_supplies[] = {
	REGULATOR_SUPPLY("Dvdd", "spi0.0"),
	REGULATOR_SUPPLY("Avdd", "spi0.0"),
};

static struct regulator_consumer_supply fixed_3v3_supplies[] = {
	REGULATOR_SUPPLY("Ovdd", "spi0.0"),
};

static struct ltc1854_platform_data ltc1854_data = {
	.convst_pin = IMX_GPIO_NR(1, 21),
	.busy_neg_pin = IMX_GPIO_NR(1, 31),
};

static struct spi_board_info ltc1854_dev __initdata = {
	.modalias	= "ltc1854",
	.max_speed_hz	= 20000000, /* Maximum according to spec */
	.bus_num	= 0,
	.chip_select	= 0,
	.platform_data	= &ltc1854_data,
};

static int guf_livius_spi0_init(struct platform_device *pdev)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_spi0_pads_func, ARRAY_SIZE(guf_livius_spi0_pads_func));
	spi_register_board_info(&ltc1854_dev, 1);
	
	regulator_register_always_on(1, "5v", fixed_5v_supplies, ARRAY_SIZE(fixed_5v_supplies), 5000000);
	regulator_register_always_on(2, "3v3", fixed_3v3_supplies, ARRAY_SIZE(fixed_3v3_supplies), 3300000);
	
	return 0;
}

/******************************************************************************/
/* USB                                                                        */
/******************************************************************************/
static int guf_livius_usbh_phy_init(struct platform_device *pdev)
{
	int ret = 0;

	mxc_iomux_v3_setup_multiple_pads(guf_livius_usbh_pads, ARRAY_SIZE(guf_livius_usbh_pads));
	
	ret = gpio_request(USBH_PWR, "USB");
	if (ret)
		goto usb_out;

	gpio_direction_output(USBH_PWR, 1);
	udelay(10);
	gpio_set_value(USBH_PWR, 0);
	
	ret = mx35_initialize_usb_hw(pdev->id, MXC_EHCI_IPPUE_DOWN | MXC_EHCI_INTERNAL_PHY);

usb_out:
	if (ret)
		printk(KERN_WARNING "%s failed with %d\n", __func__, ret);
	return ret;
}

static struct mxc_usbh_platform_data guf_livius_usbh_pdata = {
	.init = guf_livius_usbh_phy_init,
	.portsc = MXC_EHCI_MODE_SERIAL,
};

static int guf_livius_usb_otg_init(struct platform_device *pdev)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_usb_otg_pads, ARRAY_SIZE(guf_livius_usb_otg_pads));
	return 0;
}

static struct fsl_usb2_platform_data guf_livius_usb_otg_device_pdata = {
	.init = guf_livius_usb_otg_init,
	.operating_mode = FSL_USB2_DR_DEVICE,
	.phy_mode       = FSL_USB2_PHY_UTMI,
};

/******************************************************************************/
/* SRAM                                                                       */
/******************************************************************************/
struct platdata_mtd_ram guf_livius_sram_pdata = {
	.mapname	= "SRAM",
	.bankwidth	= 1,
};

static struct resource guf_livius_sram_resource = {
	.start = MX35_CS0_BASE_ADDR,
	.end   = MX35_CS0_BASE_ADDR + 0x80000 - 1,
	.flags = IORESOURCE_MEM,
};

static struct platform_device guf_livius_sram = {
	.name		= "mtd-ram",
	.id		= 0,
	.resource	= &guf_livius_sram_resource,
	.num_resources	= 1,
	.dev	= {
		.platform_data = &guf_livius_sram_pdata,
	},
};

/******************************************************************************/
/* FLASH                                                                      */
/******************************************************************************/
static struct mxc_nand_platform_data guf_livius_nand_pdata = {
	.width = 1,
	.hw_ecc = 1,
};

#if 0
/*
 * some devices will be configured at runtime by XML data from NAND
 * So, we must wait for the NAND to occur
 */
static void guf_livius_flash_add(struct mtd_info *mtd)
{
	/* Do nothing here */
}

static void guf_livius_flash_remove(struct mtd_info *mtd)
{
	/* nothing to do here */
}

static struct mtd_notifier guf_livius_flash_notifier = {
	.add = guf_livius_flash_add,
	.remove = guf_livius_flash_remove,
};
#endif

/******************************************************************************/
/* Fast Ethernet                                                              */
/******************************************************************************/
static int guf_livius_fec_init(struct platform_device *pdev)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_fec_pads, ARRAY_SIZE(guf_livius_fec_pads));
	return 0;
}

/******************************************************************************/
/* SERIAL                                                                     */
/******************************************************************************/
void guf_livius_rs485_tx_enable (int enable)
{
	gpio_set_value(RS485_TX_EN, enable);
}

void guf_livius_rs485_enable (int enable)
{
	gpio_set_value(MUX_RS485_RS232, enable);
}

void guf_livius_rs485_fd_enable (int enable)	/* low active full-duplex enable line */
{
	gpio_set_value(RS485_FD_EN, !enable);		
}

static struct imxuart_platform_data guf_livius_uart1_pdata = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_RS485, 
	.rs485_enable = guf_livius_rs485_enable,
	.rs485_tx_enable = guf_livius_rs485_tx_enable,
	.rs485_fd_enable = guf_livius_rs485_fd_enable,
};

static inline void guf_livius_serial_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_uart1_pads, ARRAY_SIZE(guf_livius_uart1_pads));
	mxc_iomux_v3_setup_multiple_pads(guf_livius_uart3_pads, ARRAY_SIZE(guf_livius_uart3_pads));
	
	gpio_request(RS232_DRV_EN, "UART");				/* power up the UART device */
	gpio_direction_output(RS232_DRV_EN, 1);
	
	imx35_add_imx_uart(0, &guf_livius_uart1_pdata);
	imx35_add_imx_uart(2, NULL);
}

/******************************************************************************/
/* UART2 / GSM                                                                */
/******************************************************************************/
static struct imxuart_platform_data guf_livius_uart2_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static int guf_livius_gsm_bootup(void *unused)
{
	// Pull down GSM module [ON/OFF] for five seconds by setting GSM_EN to 1
	gpio_set_value(GSM_EN, 1);
	msleep(5000);
	gpio_set_value(GSM_EN, 0);
	return 0;
	
	gpio_free(GSM_EN);
	gpio_free(VBATT_EN);
}

static inline void guf_livius_gsm_init(void)
{
	struct task_struct *thread;

	mxc_iomux_v3_setup_multiple_pads(guf_livius_uart2_pads, ARRAY_SIZE(guf_livius_uart2_pads));
	
	gpio_request(GSM_EN, "GSM");
	gpio_request(VBATT_EN, "GSM");
	gpio_direction_output(VBATT_EN, 1); /* Power up the GSM module */
	gpio_direction_output(GSM_EN, 0);

	printk(KERN_INFO "Initializing the GSM module\n");
	thread = kthread_create(guf_livius_gsm_bootup, NULL, "GSM_boot");
	if(thread)
		wake_up_process(thread);
}

/******************************************************************************/
/* FLEXCAN                                                                    */
/******************************************************************************/
static const struct flexcan_platform_data guf_livius_flexcan0_pdata = {
	.transceiver_switch = NULL,
};

static int guf_livius_flexcan_init(struct platform_device *pdev)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_flexcan_pads, ARRAY_SIZE(guf_livius_flexcan_pads));
	
	gpio_request(VCC_CAN_EN, "VCC_CAN_EN");	
	gpio_direction_output(VCC_CAN_EN, 1);	/* Power up CAN device */ 
	udelay(10);
	gpio_set_value(VCC_CAN_EN, 1);
	gpio_free(VCC_CAN_EN);
	
	return 0;
}

/******************************************************************************/
/* Digital I/O                                                                */
/******************************************************************************/
static const char* guf_livius_gpio_names[NUM_GPIO_CHIPS][NUM_GPIOS_PER_CHIP];

/* These are pin names that will appear under /sys/class/gpio/ */
const char* livius_gpio_names[96] = {
	[DIG_IN1]			= "dig_in1",
	[DIG_IN2]			= "dig_in2",
	[DIG_IN3]			= "dig_in3",
	[DIG_IN4]			= "dig_in4",
	[DIG_OUT1]			= "dig_out1",
	[DIG_OUT2]			= "dig_out2",
	[DIG_OUT3]			= "dig_out3",
	[DIG_OUT4]			= "dig_out4",
	[KB_LED_EN]			= "kb_led_enable",
};

/* These GPIO pins will be visible via sysfs and have a fixed direction */
static struct gpio ext_gpios[] = {
	{ DIG_IN1,			GPIOF_ACT_LOW|GPIOF_IN,					"DIG_IN1" },
	{ DIG_IN2,			GPIOF_ACT_LOW|GPIOF_IN,					"DIG_IN2" },
	{ DIG_IN3,			GPIOF_ACT_LOW|GPIOF_IN,					"DIG_IN3" },
	{ DIG_IN4,			GPIOF_ACT_LOW|GPIOF_IN,					"DIG_IN4" },
	{ DIG_OUT1,			GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"DIG_OUT1" },
	{ DIG_OUT2,			GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"DIG_OUT2" },
	{ DIG_OUT3,			GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"DIG_OUT3" },
	{ DIG_OUT4,			GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"DIG_OUT4" },
	{ KB_LED_EN,		GPIOF_ACT_LOW|GPIOF_OUT_INIT_HIGH,		"KB_LED_EN" },
};

static void set_gpio_names(const char** gpio_names)
{
	int i;
	struct gpio_chip *chip;

	for (i = 0; i < (NUM_GPIO_CHIPS*NUM_GPIOS_PER_CHIP); i++){
		unsigned int chip_num = i / NUM_GPIOS_PER_CHIP; 
		unsigned int gpio_num = i % NUM_GPIOS_PER_CHIP;	
		guf_livius_gpio_names[chip_num][gpio_num] = gpio_names[i];
		
		chip = gpio_to_chip(i);
		chip->names = guf_livius_gpio_names[chip_num];
	}
}

/******************************************************************************/
/* KEYPAD		                                                              */
/******************************************************************************/
static const unsigned int keypad_row_gpios[] = {
	KB_ROW0,
	KB_ROW1,
	KB_ROW2
};

static const unsigned int keypad_col_gpios[] = {
	KB_COL0,
	KB_COL1,
	KB_COL2
};

static const uint32_t guf_livius_keymap[] = {
	KEY(0, 0, KEY_ESC),
	KEY(0, 1, KEY_LEFT),
	KEY(0, 2, KEY_MENU),
	KEY(1, 0, KEY_UP),
	KEY(1, 1, KEY_ENTER),
	KEY(1, 2, KEY_DOWN),
	KEY(2, 0, KEY_BACKSPACE),
	KEY(2, 1, KEY_RIGHT),
	KEY(2, 2, KEY_HOME),
};

static const struct matrix_keymap_data guf_livius_keymap_data = {
	.keymap			= guf_livius_keymap,
	.keymap_size	= ARRAY_SIZE(guf_livius_keymap),
};

static struct matrix_keypad_platform_data guf_livius_matrix_keypad_pdata = {
	.keymap_data 		= &guf_livius_keymap_data,
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

static struct platform_device guf_livius_keypad_device = {
	.name	= "matrix-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &guf_livius_matrix_keypad_pdata,
	},
};

/******************************************************************************/
/* LEDs                                                                       */
/******************************************************************************/
static const struct gpio_led guf_livius_leds[] = {
	{
		.name			= "led_s1_red",
		.active_low		= 1,
		.gpio			= KB_LED1,
	},{
		.name			= "led_s1_green",
		.active_low		= 1,
		.gpio			= KB_LED2,
	},{
		.name			= "led_s2_red",
		.active_low		= 1,
		.gpio			= KB_LED3,
	},{
		.name			= "led_s2_green",
		.active_low		= 1,
		.gpio			= KB_LED4,
	},{
		.name			= "led_gsm_red",
		.active_low		= 1,
		.gpio			= KB_LED5,
	},{
		.name			= "led_pwr_red",
		.active_low		= 1,
		.gpio			= KB_LED6,
	},{
		.name			= "led_pwr_green",
		.active_low		= 1,
		.gpio			= KB_LED7,
	},
};

static const struct gpio_led_platform_data guf_livius_led_info = {
	.leds		= guf_livius_leds,
	.num_leds	= ARRAY_SIZE(guf_livius_leds),
};

/******************************************************************************/
/* ESDHC                                                                      */
/******************************************************************************/
static struct regulator_consumer_supply vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx35.0"),
	REGULATOR_SUPPLY("vqmmc", "sdhci-esdhc-imx35.0"),
};

static struct esdhc_platform_data guf_livius_esdhc_pdata = {
	.cd_gpio = IMX_GPIO_NR(2, 18),
	.wp_type = ESDHC_WP_NONE,
	.cd_type = ESDHC_CD_GPIO,
};

static int guf_livius_esdhc_init(struct platform_device *pdev)
{
	mxc_iomux_v3_setup_multiple_pads(guf_livius_esdhc_pads, ARRAY_SIZE(guf_livius_esdhc_pads));
	
	regulator_register_always_on(3, "3v", vmmc_consumers, ARRAY_SIZE(vmmc_consumers), 3000000);
	
	return 0;
}

/******************************************************************************/
/* Board specific initialization                                              */
/******************************************************************************/
static void __init mx35_livius_init(void)
{
	imx35_soc_init();

	
	/* RAM / FLASH */
	platform_device_register(&guf_livius_sram);
	imx35_add_mxc_nand(&guf_livius_nand_pdata);
	
	/* DIO */
	mxc_iomux_v3_setup_multiple_pads(guf_livius_dio_pads, ARRAY_SIZE(guf_livius_dio_pads));
	set_gpio_names(livius_gpio_names);
	guf_init_gpio_group(ext_gpios, ARRAY_SIZE(ext_gpios), true, true);
		
	/* Keypad */
	mxc_iomux_v3_setup_multiple_pads(guf_livius_keypad_pads, ARRAY_SIZE(guf_livius_keypad_pads));
	platform_device_register(&guf_livius_keypad_device);
	
	/* LEDs */
	mxc_iomux_v3_setup_multiple_pads(guf_livius_led_pads, ARRAY_SIZE(guf_livius_led_pads));
	gpio_led_register_device(-1, &guf_livius_led_info);
	
	/* FEC */
	guf_livius_fec_init(imx35_add_fec(NULL));
	
	/* UART */
	gpio_request(MUX_RS485_RS232, "RS485_enable");		
	gpio_direction_output(MUX_RS485_RS232, 0);		/* enable RS232-1 by default */ 
	gpio_request(RS485_TX_EN, "RS485_tx_enable");
	gpio_direction_output(RS485_TX_EN, 1);
	gpio_request(RS485_FD_EN, "RS485_fd_enable");	
	gpio_direction_output(RS485_FD_EN, 0);			/* default RS485 mode: full duplex */
	guf_livius_serial_init();	

	/* GSM */
	guf_livius_gsm_init();
	imx35_add_imx_uart(1, &guf_livius_uart2_pdata);
	
	guf_livius_i2c1_eeprom_rtc_init(imx35_add_imx_i2c0(&guf_livius_i2c1_pdata));
	guf_livius_i2c3_temp_init(imx35_add_imx_i2c2(&guf_livius_i2c3_pdata));
	guf_livius_spi0_init(imx35_add_spi_imx0(&guf_livius_spi0_pdata));
	
	imx35_add_fsl_usb2_udc(&guf_livius_usb_otg_device_pdata);
	guf_livius_esdhc_init(imx35_add_sdhci_esdhc_imx(0, &guf_livius_esdhc_pdata));
	imx35_add_mxc_ehci_hs(&guf_livius_usbh_pdata);
	
	guf_livius_flexcan_init(imx35_add_flexcan(0, &guf_livius_flexcan0_pdata));


	guf_livius_pm_init();
}

static void __init  mx35_livius_init_late(void)
{
	imx35_cpuidle_init();
}

static void __init guf_livius_timer_init(void)
{
	mx35_clocks_init();
}

static void __init guf_livius_reserve(void)
{
}

MACHINE_START(GUF_CUPID, "Garz & Fricke LIVIUS")
	/* Maintainer: Garz & Fricke GmbH */
	.atag_offset = 0x10000,
	.map_io = mx35_map_io,
	.init_early = imx35_init_early,
	.init_irq = mx35_init_irq,
	.handle_irq = imx35_handle_irq,
	.init_time = guf_livius_timer_init,
	.init_machine = mx35_livius_init,
	.init_late      = mx35_livius_init_late,
	.reserve = guf_livius_reserve,
	.restart	= mxc_restart,
MACHINE_END
