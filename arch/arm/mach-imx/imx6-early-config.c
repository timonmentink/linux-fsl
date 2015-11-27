/* 
 * This file contains several helper functions, mostly implemeting 
 * configurations down by u-boot in the original freescale BSP.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>

#include "clk.h"
#include "common.h"
#include "hardware.h"
#include <asm/setup.h>

static void __iomem *anatop_base;

/* PLL2_528 defines */
#define ANADIG_PLL_528_DIV_SELECT		(1)
#define ANADIG_PLL_528_SYS_SS_STOP_OFFSET		(16)
#define ANADIG_PLL_528_SYS_SS_STOP_MASK			(0xFFFF << ANADIG_PLL_528_SYS_SS_STOP_OFFSET)
#define ANADIG_PLL_528_SYS_SS_STEP_OFFSET		(0)
#define ANADIG_PLL_528_SYS_SS_STEP_MASK			(0x7FFF)
#define ANADIG_PLL_528_SYS_SS_ENABLE			(1 << 15)
#define ANADIG_PLL_528_DENOM_MASK				(0x3FFFFFFF)

#define PLL_SETREG_OFFSET		0x4
#define PLL_CLRREG_OFFSET		0x8
#define PLL_TOGGLE_OFFSET		0x0C
#define PLL_NUM_DIV_OFFSET		0x10
#define PLL_DENOM_DIV_OFFSET		0x20
#define PLL_528_SS_OFFSET		0x10
#define PLL_528_NUM_DIV_OFFSET		0x20
#define PLL_528_DENOM_DIV_OFFSET	0x30

#define PLL2_528_OFFSET		0x30

/*
 * PLL2 spread spectrum can be configured via /proc/mx6_spread_sprectrum
 * using the following syntax:
 * 
 *    "<step>,<stop>,<denom>,<enabled>"  (e.g. "1,250,400,1")
 * 
 *    spread spectrum range  =  stop/denom    * 24MHz
 *    modulation frequency   =  step/(2*stop) * 24MHz
 *    frequency step         =  step/denom    * 24MHz
 */
#define PROC_FS_MAX_LEN 22
#define PROC_FS_NAME "mx6_spread_sprectrum"
#define FREF 24000000

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

int spread_spectrum_write_proc(struct file *filp,const char *buf,size_t count,loff_t *offp)
{
	int ret;
	char str[PROC_FS_MAX_LEN];
	unsigned int procfs_buffer_size = 0;
	int ints[5];

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN )
		procfs_buffer_size = PROC_FS_MAX_LEN;

	pr_debug("%s %d, %p %d\n", __func__, __LINE__, buf, count);
	if(copy_from_user(str, buf, procfs_buffer_size))
	{
		printk("spread_sprectrum_proc_write: proc_write failed at copy_from_user\n");
		return -EFAULT;
	}
	str[procfs_buffer_size-1] = '\0';

	get_options(str, 5, ints);
	if (ints[0] != 4)
	{
		ret = -EINVAL;
	}
	else
	{
		uint32_t sys_ss, denom, stop, step, enabled;

		step = ((ints[1] << ANADIG_PLL_528_SYS_SS_STEP_OFFSET) & ANADIG_PLL_528_SYS_SS_STEP_MASK) >> ANADIG_PLL_528_SYS_SS_STEP_OFFSET;
		stop = ((ints[2] << ANADIG_PLL_528_SYS_SS_STOP_OFFSET) & ANADIG_PLL_528_SYS_SS_STOP_MASK) >> ANADIG_PLL_528_SYS_SS_STOP_OFFSET;;
		denom = ints[3] & ANADIG_PLL_528_DENOM_MASK;
		enabled = ints[4];
		sys_ss = (step << ANADIG_PLL_528_SYS_SS_STEP_OFFSET) | (stop << ANADIG_PLL_528_SYS_SS_STOP_OFFSET);
		
		pr_debug("%s %d, %d %d %d %d %d\n", __func__, __LINE__, step, stop, denom, enabled, sys_ss);

		/* Disable spread spectrum mode */
		__raw_writel((__raw_readl(anatop_base + PLL2_528_OFFSET + PLL_528_SS_OFFSET) & ~ANADIG_PLL_528_SYS_SS_ENABLE), anatop_base + PLL2_528_OFFSET +  + PLL_528_SS_OFFSET);

		/* Write new values */
		__raw_writel(sys_ss, anatop_base + PLL2_528_OFFSET + PLL_528_SS_OFFSET);
		__raw_writel(denom, anatop_base + PLL2_528_OFFSET + PLL_528_DENOM_DIV_OFFSET);

		/* Enable spread spectrum mode */
		if (enabled)
			__raw_writel((__raw_readl(anatop_base + PLL2_528_OFFSET + PLL_528_SS_OFFSET) | ANADIG_PLL_528_SYS_SS_ENABLE), anatop_base + PLL2_528_OFFSET +  + PLL_528_SS_OFFSET);

		ret = procfs_buffer_size;
	}

	pr_debug("%s %d: ret %d\n", __func__, __LINE__, ret);
	return ret;
}

int spread_spectrum_read_proc(struct file *filp,char *buf,size_t count,loff_t *offp ) 
{
	static int state = 0;
	int ret;
	char buffer[PROC_FS_MAX_LEN];
	uint32_t sys_ss, denom, stop, step, enabled;

	pr_debug("%s %p %p %d %p\n", __func__, filp, buf, count, offp);
	if( state != 0)
	{
		state = 0;
		return 0;
	}

	sys_ss = __raw_readl(anatop_base + PLL2_528_OFFSET + PLL_528_SS_OFFSET);
	denom = __raw_readl(anatop_base + PLL2_528_OFFSET + PLL_528_DENOM_DIV_OFFSET);
	stop = (sys_ss & ANADIG_PLL_528_SYS_SS_STOP_MASK) >> ANADIG_PLL_528_SYS_SS_STOP_OFFSET;
	step = (sys_ss & ANADIG_PLL_528_SYS_SS_STEP_MASK) >> ANADIG_PLL_528_SYS_SS_STEP_OFFSET;
	enabled = (sys_ss & ANADIG_PLL_528_SYS_SS_ENABLE) ? 1 : 0;

	ret = snprintf(buffer,PROC_FS_MAX_LEN, "%u,%u,%u,%u\n", step, stop, denom, enabled);
	pr_debug("%s %d- %s\n", __func__, ret, buffer);

	if(ret < 0)	goto error;
	if(ret >= PROC_FS_MAX_LEN) ret = PROC_FS_MAX_LEN;

	count = ret;
	ret = copy_to_user(buf, buffer, count + 1);
	if(ret) goto error;

	state = 1;
	return count + 1;
error:
	pr_err("%s Error %d\n",__func__, ret);
	return ret;
}

static const struct file_operations spread_spectrum_fops = {
  .owner = THIS_MODULE,
  .read = spread_spectrum_read_proc,
  .write = spread_spectrum_write_proc,
};

int spread_sprectrum_create_proc_fs_entry(void)
{
	/* Create procfs entry for spread spectrum */
	struct proc_dir_entry *entry = proc_create(PROC_FS_NAME, 0666, NULL, &spread_spectrum_fops);

	if(!entry)
	{
		printk("Could not create /proc/%s\n", PROC_FS_NAME);
		return -ENODEV;
	}
	else
	{
		printk("/proc/%s created\n", PROC_FS_NAME);
	}
	return 0;
}

//================================================================================
//
//================================================================================

static void imx_reset_pfd(void __iomem * anatop_base)
{
	/*
	 * Per the IC design, we need to gate/ungate all the unused PFDs
	 * to make sure PFD is working correctly, otherwise, PFDs may not
	 * not output clock after reset.
	 */

	// Set gate for pfd 480
	writel_relaxed( (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7), anatop_base + 0xF0 + 0x4);
	// Set gate for pfd 528
	if(cpu_is_imx6q())
		writel_relaxed(  1 << 23 | 1 << 15 | 1 << 7, anatop_base + 0x100 + 0x4);
	else
		writel_relaxed( 1 << 15 | 1 << 7, anatop_base + 0x100 + 0x4);
	// Clear gate for pfd 480
	writel_relaxed( 1 << 31 | 1 << 23 | 1 << 15 | 1 << 7, anatop_base + 0xF0 + 0x8);
	// Clear gate for pfd 528
	if(cpu_is_imx6q())
		writel_relaxed( 1 << 23 | 1 << 15 | 1 << 7, anatop_base + 0x100 + 0x8);
	else
		writel_relaxed( 1 << 15 | 1 << 7, anatop_base + 0x100 + 0x8);

}
static void imx_set_pcie_phy_power_down(void __iomem * gpr)
{
	u32 val;
	
	val = readl(gpr + 0x4);
	val |= 0x1 << 18;
	writel(val, gpr + 0x4);

}
/*
 * Set the VDDSOC
 *
 * Mask out the REG_CORE[22:18] bits (REG2_TRIG) and set
 * them to the specified millivolt level.
 * Possible values are from 0.725V to 1.450V in steps of
 * 0.025V (25mV).
 */
static void set_vddsoc(u32 mv, void __iomem * anatop_base )
{
	u32 val, reg = readl(anatop_base + 0x140);

	if (mv < 725)
		val = 0x00;	/* Power gated off */
	else if (mv > 1450)
		val = 0x1F;	/* Power FET switched full on. No regulation */
	else
		val = (mv - 700) / 25;

	/*
	 * Mask out the REG_CORE[22:18] bits (REG2_TRIG)
	 * and set them to the calculated value (0.7V + val * 0.25V)
	 */
	reg = (reg & ~(0x1F << 18)) | (val << 18);
	writel(reg, anatop_base + 0x140);

	/* ROM may modify LDO ramp up time according to fuse setting for safe,
	 * we need to reset these settings to match the reset value: 0'b00
	 */
	reg = readl(anatop_base + 0x170);
	reg &= ~(0x3f << 24);
	writel(reg, anatop_base + 0x170);

}

void init_aips(void __iomem * aips1, void __iomem * aips2)
{

	/*
	 * Set all MPROTx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	writel(0x77777777, aips1  + 0x00);   // mprot0
	writel(0x77777777, aips1  + 0x04);  // mprot1
	writel(0x77777777, aips2  + 0x00);  // mprot0
	writel(0x77777777, aips2  + 0x04);  // mprot1

	/*
	 * Set all OPACRx to be non-bufferable, not require
	 * supervisor privilege level for access,allow for
	 * write access and untrusted master access.
	 */
	writel(0x00000000, aips1 + 0x10 );  // opacr0
	writel(0x00000000, aips1 + 0x14 );  // opacr1
	writel(0x00000000, aips1 + 0x18 );  // opacr2
	writel(0x00000000, aips1 + 0x1C );  // opacr3
	writel(0x00000000, aips1 + 0x20 );  // opacr4
	writel(0x00000000, aips2 + 0x10 );  // opacr0
	writel(0x00000000, aips2 + 0x14 );  // opacr1
	writel(0x00000000, aips2 + 0x18 );  // opacr2
	writel(0x00000000, aips2 + 0x1C );  // opacr3
	writel(0x00000000, aips2 + 0x20 );  // opacr4
}


static void imx_set_vddpu_power_down(void __iomem * anatop_base, void __iomem * gpc)
{
	u32 val;

	/* need to power down xPU in GPC before turn off PU LDO */
	val = readl(gpc + 0x260);
	writel(val | 0x1, gpc + 0x260);

	val = readl(gpc + 0x0);
	writel(val | 0x1, gpc + 0x0);
	while (readl(gpc + 0x0) & 0x1);

	/* disable VDDPU */
	val = 0x3e00;
	writel(val, anatop_base + 0x148);  // reg core clear
}

/* 
   This implements the fix mentionied in this thread:
   https://community.freescale.com/message/522033

   Actually this was a change introduced by freescale to u-boot for the 3.10.17 BSP, 
   which we missed because we don't use u-boot.
   "..., it will enable AXI cache and give IPU high priority to access AXI bus" 
   */
static void imx_set_ipu_bus_priority( void __iomem *gpr)
{

	//pr_info("%s %d: 0x%08X\n", __func__, __LINE__, readl(gpr + 0x10));
	//pr_info("%s %d: 0x%08X\n", __func__, __LINE__, readl(gpr + 0x18));
	//pr_info("%s %d: 0x%08X\n", __func__, __LINE__, readl(gpr + 0x1c));

	writel(0xF00000CF, gpr + 0x10);
	writel(0x007F007F, gpr + 0x18);
	writel(0x007F007F, gpr + 0x1c);

	//pr_info("%s %d: 0x%08X\n", __func__, __LINE__, readl(gpr + 0x10));
	//pr_info("%s %d: 0x%08X\n", __func__, __LINE__, readl(gpr + 0x18));
	//pr_info("%s %d: 0x%08X\n", __func__, __LINE__, readl(gpr + 0x1c));

}

/* This function sets the registers configured by u-boot in freescales setup.
   It is done here as it has to be done very early in the boot procedure
   */
void fixup_bootloader_settings(void )
{
	struct device_node *np, *child;
	void __iomem * anatop, * ccm, * gpr, *gpc, * aips1, * aips2;
	u32 val;

	early_pr_dbg("%s %d\n",__func__, __LINE__);

	// Map the needed memory areas
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-anatop");
	anatop = of_iomap(np, 0); WARN_ON(!anatop);
	anatop_base = anatop;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-iomuxc-gpr");	WARN_ON(!np);
	gpr = of_iomap(np, 0); 	WARN_ON(!gpr);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-gpc");	WARN_ON(!np);
	gpc = of_iomap(np, 0); WARN_ON(!gpc);

	np = of_find_compatible_node(NULL, NULL, "fsl,aips-bus"); 	WARN_ON(!np);
	child = of_get_child_by_name(np, "aipstz");  WARN_ON(!child);
	aips1 = of_iomap(child, 0); 	WARN_ON(!aips1);
	of_node_put(child);

	np = of_find_compatible_node(np, NULL, "fsl,aips-bus"); WARN_ON(!np);
	child = of_get_child_by_name(np, "aipstz"); WARN_ON(!child);
	aips2 = of_iomap(child, 0); WARN_ON(!aips2);
	of_node_put(child);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ccm");	WARN_ON(!np);
	ccm = of_iomap(np, 0); WARN_ON(!ccm);

	early_pr_dbg("%s %d\n",__func__, __LINE__);

	imx_set_ipu_bus_priority( gpr);

	// Set the registers 
	if( !cpu_is_imx6sl()){
		/* From 0001-ENGR00319415-pcie-random-link-down-issue-after-warm-.patch (uboot)
		There are about 0.02% percentage on some imx6q/dl/solo
		hw boards, random pcie link down when warm-reset is used.
		Make sure to clear the ref_ssp_en bit16 of gpr1 before
		warm-rst, and set ref_ssp_en after the pcie clks are
		stable to workaround it.

		rootcause:
		* gpr regisers wouldn't be reset by warm-rst, while the
		ref_ssp_en is required to be reset by pcie.
		(work-around in u-boot)
		* ref_ssp_en should be set after pcie clks are stable.
		(work-around in kernel)
		*/
		/* this bit is not used by imx6sx anymore */

		/*
		 * There are about 0.02% percentage, random pcie link down
		 * when warm-reset is used.
		 * clear the ref_ssp_en bit16 of gpr1 to workaround it.
		 * then warm-reset imx6q/dl/solo again.
		 */
		val = readl(gpr + 0x4);
		if (val & (0x1 << 16)) {
			val &= ~(0x1 << 16);
			writel(val, gpr + 0x4);
			//reset_cpu(0);
			mxc_restart(0, 0);
		}
	}
	
	/* Clear MMDC channel mask */
	writel(0, ccm + 0x04);  // CLKCTL_CCDR

	init_aips(aips1, aips2);

	set_vddsoc(1200, anatop);

	if( !cpu_is_imx6sl())
	{	/* Reset pfd is not needed for MX6SX */
		imx_reset_pfd(anatop);
		imx_set_pcie_phy_power_down(gpr);
	}

	imx_set_vddpu_power_down(anatop, gpc);

	// unmap the memory
	iounmap(ccm);
	iounmap(aips1);
	iounmap(aips2);
	iounmap(gpr);
	iounmap(gpc);
	early_pr_dbg("%s %d\n",__func__, __LINE__);
}

//================================================================================
//
//================================================================================
