/*
 * linux/sound/soc.h -- ALSA SoC Layer
 *
 * Author:		Jonas Hoeppner
 * Copyright:	Garz und Fricke GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CUPID_IMX_AC97_H
#define __CUPID_IMX_AC97_H

struct cupid_imx_ac97_pdata{
	int irq_gpio;
	int enable_ucb1400_ts;
	int enable_wm9705_ts;
};

#endif
