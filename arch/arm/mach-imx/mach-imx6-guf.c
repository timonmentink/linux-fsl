/*
 * Copyright 2011-2014 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/opp.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include "guf.h"
#include "guf_xml_to_dt.h"

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"
#include <asm/setup.h>

#include <linux/regulator/pfuze100.h>
#include <linux/regulator/machine.h>

#ifdef CONFIG_RAM_TO_FILE
extern void ram_to_file_reserve(void);
#endif


#define COMPATIBLE_MACHINES \
	MACHINE( "guf,imx6q-santaro",		SANTARO_X2_X4,	MXC_CPU_IMX6Q,  0 ) \
	MACHINE( "guf,imx6q-santoka",		SANTOKA_X2_X4,	MXC_CPU_IMX6Q,  0 ) \
	MACHINE( "guf,imx6q-sdc-cspu",		SDC_CSPU,		MXC_CPU_IMX6Q,	0 ) \
	MACHINE( "guf,imx6dl-santaro",		SANTARO_X1_X2L,	MXC_CPU_IMX6DL, 0 ) \
	MACHINE( "guf,imx6dl-santoka",		SANTOKA_X1_X2L,	MXC_CPU_IMX6DL, 0 ) \
	MACHINE( "guf,imx6dl-santino",		SANTINO,		MXC_CPU_IMX6DL, 0 ) \
	MACHINE( "guf,imx6dl-santino-lt",	SANTINO_LT,		MXC_CPU_IMX6DL, 0 ) \

enum guf_imx6_board_revision {
	V_UNKNOWN = -1,
	V1_0,
	V1_1,
	V1_2,
};
enum guf_imx6_board_ids {
	B_UNKNOWN = -1,
	B_SANTARO,
	B_SANTOKA,
	B_SANTINO,
	B_SANTINO_LT,
	B_SDC_CSPU,
};
static enum guf_imx6_board_revision board_revision = V_UNKNOWN;
static enum guf_imx6_board_ids board_id = B_UNKNOWN;


static const char *imx6q_dt_compat[] __initdata = {
#define MACHINE( compatible,a,b,c) compatible,
COMPATIBLE_MACHINES
#undef MACHINE
	NULL,
};
static const char guf_mxc_cpu_type[] = {
#define MACHINE( a,b, cpu_type,c) cpu_type,
COMPATIBLE_MACHINES
#undef MACHINE
};

enum{
#define MACHINE( a,id,b,c) id,
	COMPATIBLE_MACHINES
#undef MACHINE
	MACHINE_CNT
};
static int machine_id = -1;

#define MACHINE( a,id,b,c) int is_##id(void){ return machine_id == id;}
	COMPATIBLE_MACHINES
#undef MACHINE

#define BOARD_NAME "Garz & Fricke %s (i.MX6)"

static char board_name[64] = "Garz & Fricke i.MX6";

int is_SANTARO(void)
{
	return is_SANTARO_X1_X2L() || is_SANTARO_X2_X4();
}
int is_SANTOKA(void)
{
	return is_SANTOKA_X1_X2L() || is_SANTOKA_X2_X4();
}
/******************************************************************************/
/* BOARD REVISION                                                             */
/******************************************************************************/

#define PFID0					IMX_GPIO_NR(2, 0)
#define PFID1					IMX_GPIO_NR(2, 1)
#define PFID2					IMX_GPIO_NR(2, 2)
#define PFID3					IMX_GPIO_NR(2, 3)
#define PFID4					IMX_GPIO_NR(2, 4)
#define PFID5					IMX_GPIO_NR(2, 5)
#define PFID6					IMX_GPIO_NR(2, 6)
#define PFID7					IMX_GPIO_NR(2, 7)



static void guf_imx6_get_board_and_revision(void)
{
	int revision_bitmask;
	int board_bitmask;
	#define GPIO_2_DR 0x020A0000
	void * gpio2_dr;
	int value;

	if(!(gpio2_dr = ioremap( GPIO_2_DR, sizeof(int)))){
		pr_err("%s: Failed to get board revision, taking defaults\n", __func__);
		value = 0b11000111;
	}else{
		value = readl_relaxed( gpio2_dr) & 0xFF;
		iounmap(gpio2_dr);
	}

	revision_bitmask = (value >> 5) & 0x7;
	board_bitmask = (value >> 0) & 0x7;

	switch (board_bitmask) {
		case 0b111: board_id = B_SANTARO; printk("SANTARO"); break;
		case 0b101: board_id = B_SANTOKA; printk("SANTOKA"); break;
		case 0b100: board_id = B_SANTINO; printk("SANTINO"); break;
		case 0b011: board_id = B_SANTINO_LT; printk("SANTINO LT"); break;
		case 0b110: board_id = B_SDC_CSPU; printk("SDC-CSPU"); break;
		default: printk("Unknown Board 0x%x", board_bitmask);
	}
	switch (revision_bitmask) {
		case 0b111: board_revision = V1_0; break;
		case 0b110: board_revision = V1_1; break;
		case 0b101: board_revision = V1_2; break;
	}
	printk(" hardware revision ");
	if (board_revision >= 0)
		printk("1.%d\n", board_revision);
	else
		printk("unknown\n");
}


static struct guf_xml_data * xml_config = 0;
extern int spread_sprectrum_create_proc_fs_entry(void);
extern struct device platform_bus;

static inline void imx6q_enet_init(void)
{
	// configure REF_CLK for external source via GPIO_16
	{
		struct regmap * iomuxc_gpr;
		iomuxc_gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(iomuxc_gpr)) {
			pr_err("%s: failed to find fsl,imx6q-iomux-gpr regmap: %lu\n", __func__, PTR_ERR(iomuxc_gpr));
			goto iomuxc_gpr_failed;
		}

		regmap_update_bits( iomuxc_gpr, IOMUXC_GPR1, 
			IMX6Q_GPR1_ENET_CLK_SEL_MASK,
			IMX6Q_GPR1_ENET_CLK_SEL_PAD );

		iomuxc_gpr_failed:;
	}
}

#ifdef CONFIG_LOGO_GUF_SANTARO_DEFAULT
extern const struct linux_logo logo_guf_santaro_default_png;
extern const struct linux_logo logo_guf_santoka_default_png;
const struct linux_logo * imx6_get_guf_logo(void)
{
		/* Garz & Fricke SANTARO default Linux PNG logo */
	if( is_SANTARO())
		return &logo_guf_santaro_default_png;
	//if( is_SANTOKA())
		return &logo_guf_santoka_default_png;

}
#endif

static inline void imx6q_usb_init(void)
{
	// configure USB OTG SEL to use GPIO1
	{
		struct regmap * iomuxc_gpr;
		iomuxc_gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(iomuxc_gpr)) {
			pr_err("%s: failed to find fsl,imx6q-iomux-gpr regmap: %lu\n", __func__, PTR_ERR(iomuxc_gpr));
			goto iomuxc_gpr_failed;
		}

		regmap_update_bits( iomuxc_gpr, IOMUXC_GPR1, 
			IMX6Q_GPR1_USB_OTG_ID_SEL_MASK,
			IMX6Q_GPR1_USB_OTG_ID_SEL_GPIO_1 );

		iomuxc_gpr_failed:;
	}
}

static int imx6q_find_system_type_from_dt(void)
{
	struct device_node *root;
	const char * model;
	int ret, i;

	root = of_find_node_by_path("/");
	if( !root) 
		return -ENODEV;

	ret = of_property_read_string(root, "compatible", &model);
	
	if( ret) return ret;
	
	for(i = 0; i < MACHINE_CNT; i++)
		if( 0 == strcmp(model, imx6q_dt_compat[i]))
			{
				machine_id = i;
				break;
			}

	ret = of_property_read_string(root, "model", &model);
	if( ret) return ret;
	snprintf(board_name, sizeof(board_name),  BOARD_NAME, model);
	// early update of the mxc_cpu_type, is overwritten later from anatop register
	mxc_set_cpu_type(guf_mxc_cpu_type[i]);
	return 0;
}

static int imx6q_guf_update_dt_from_xml(void)
{
	int r = 0;

	int major = 1;
	int minor;
	switch(board_revision)
	{
	case V1_0: minor = 0; break;
	default:
	case V1_1: minor = 1; break;
	case V1_2: minor = 2; break;
	}
	r = update_devicetree_to_board_revision( major, minor);

#ifdef CONFIG_FB
	r |= set_video_mode_from_xml(xml_config);
	r |= set_backlight_settings_from_xml(&xml_config->backlight);
	r |= configure_touch_from_xml( &xml_config->touch);
	r |= logo_license_from_dt_to_xml(&xml_config->logo_license, &xml_config->rotation);
#endif

	r |= set_mac_addr_from_xml( &xml_config->network);
	r |= guf_setup_network_from_xml(&xml_config->network);
	r |= configure_keypad(xml_config->keypad);

	r |= integrate_devtree_from_xml(xml_config->devtree);

	if( r != 0)
		pr_err("%s: Failure occured while using settings from xml\n", __func__);
	
	return r;
}


static int __match_device_name(struct device *dev, void *data)
{
	char *name = (char*)data;
	return !strcmp(dev_name(dev), name);
}

static void __init imx6_guf_init_machine(void)
{
	struct device *parent, *child;

	guf_imx6_get_board_and_revision();
	if(xml_config)
		imx6q_guf_update_dt_from_xml();
	else
		pr_warn("%s: No xml config loaded\n",__func__);

	mxc_arch_reset_init_dt();
	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table, 0, parent);

	/* Create link to eMMC in /sys/devices/platform for Flash-N-Go System backward compatibility */
	child = device_find_child(parent, "soc.1", __match_device_name);
	child = device_find_child(child, "2100000.aips-bus", __match_device_name);
	child = device_find_child(child, "219c000.usdhc", __match_device_name);
	if (child) {
		if( sysfs_create_link(&platform_bus.kobj, &child->kobj, "sdhci-esdhc-imx.3"))
		{
			pr_err("%s: Failed to create sysfs entry for FNG System backward compatibility\n", __func__);
		}
	}

	imx6q_enet_init();
	/*imx6q_usb_init(); //TESTME this is from 3.0 kernel but everything worked without it ?*/
	imx_anatop_init();
	imx6_pm_init();
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_1GHZ		0x2
#define OCOTP_CFG3_SPEED_850MHZ		0x1
#define OCOTP_CFG3_SPEED_800MHZ		0x0

static void __init imx6q_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz; -- i.MX6Q only.
	 * 2b'10: 1000000000Hz;
	 * 2b'01: 850000000Hz; -- i.MX6Q Only, exclusive with 1GHz.
	 * 2b'00: 800000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_1P2GHZ)
			if (opp_disable(cpu_dev, 1200000000))
				pr_warn("failed to disable 1.2 GHz OPP\n");
	}
	if ((val & 0x3) < OCOTP_CFG3_SPEED_1GHZ)
		if (opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 1 GHz OPP\n");
	if (cpu_is_imx6q()) {
		if ((val & 0x3) < OCOTP_CFG3_SPEED_850MHZ ||
			(val & 0x3) == OCOTP_CFG3_SPEED_1GHZ)
			if (opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 850 MHz OPP\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init imx6_guf_init_late(void)
{
	struct regmap *gpr;
	int ret;

	/*
	 * Need to force IOMUXC irq pending to meet CCM low power mode
	 * restriction, this is recommended by hardware team.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_GINT_MASK,
			IMX6Q_GPR1_GINT_ASSERT);

	ret = spread_sprectrum_create_proc_fs_entry();
	if(ret)
		pr_warn("%s: Error while creating spread spectrum proc fs entry: %d\n",__func__, ret);

	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >	IMX_CHIP_REVISION_1_0))
	{
		imx6q_cpuidle_init();
	}

	if (IS_ENABLED(CONFIG_ARM_IMX6_CPUFREQ)) {
		imx6q_opp_init(&imx6q_cpufreq_pdev.dev);
		platform_device_register(&imx6q_cpufreq_pdev);
	}
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static void __init imx6q_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());

	xml_config = guf_parse_xml_parameters();

	if(imx6q_find_system_type_from_dt())
		pr_err("Machine does not seem to be compatible, continuing anyway ...\n");
}

static void __init imx6_guf_reserve(void)
{
#ifdef CONFIG_RAM_TO_FILE
	ram_to_file_reserve();
#endif
}


DT_MACHINE_START(IMX6_GUF, board_name)
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.init_time	= imx6q_timer_init,
	.init_machine	= imx6_guf_init_machine,
	.init_late      = imx6_guf_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= mxc_restart,
	.reserve = imx6_guf_reserve,
MACHINE_END
