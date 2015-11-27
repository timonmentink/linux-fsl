/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/module.h>
#include <asm/system_misc.h>

static int imx35_cpuidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	arm_pm_idle();
	return index;
}

static struct cpuidle_driver imx35_cpuidle_driver = {
	.name             = "imx35_cpuidle",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter            = imx35_cpuidle_enter,
		.exit_latency     = 2,
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "WAIT",
		.desc             = "Clock off",
	},
	.state_count = 1,
};

int __init imx35_cpuidle_init(void)
{
	return cpuidle_register(&imx35_cpuidle_driver, NULL);
}
