/*
 * cupid-ac97.c -- SoC audio for imx_cupid
 *
 * Copyright 2010 Carsten Behling <carsten.behling@garz-fricke.de>
 * Copyright 2014 Jonas Hoeppner <jonas.hoeppner@garz-fricke.de>
 *
 * This code is based on code copyrighted by Freescale,
 * Liam Girdwood, Javier Martin and probably others.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <linux/ucb1400.h>
#include <linux/wm97xx.h>
#include <linux/cupid_imx_ac97.h>

#include "imx-ssi.h"
#include "imx-audmux.h"

#define DEBUG

static void imx_cupid_pen_irq_enable(struct wm97xx *wm, int enable)
{
	if (enable)
		enable_irq(wm->pen_irq);
	else
		disable_irq_nosync(wm->pen_irq);
}

static struct wm97xx_mach_ops imx_cupid_wm9705_mach_ops = {
	.acc_enabled	= 0,
	.irq_enable	= imx_cupid_pen_irq_enable,
};

static struct wm97xx_pdata imx_cupid_wm9705_pdata = {
	.mach_ops = &imx_cupid_wm9705_mach_ops,
};
static struct ucb1400_pdata imx_cupid_ucb1400_pdata = {
};

int imx_cupid_dai_link_init(struct snd_soc_pcm_runtime *rtd) 
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai * cpu_dai = rtd->cpu_dai;
	unsigned short vid1, vid2;
	pr_debug("snd: imx_cupid_dai_link_init: %s\n", codec->ac97?"AC97":"Not AC97");

	if(codec->ac97) {
		vid1 = codec->ac97->bus->ops->read(codec->ac97, AC97_VENDOR_ID1);
		vid2 = codec->ac97->bus->ops->read(codec->ac97, AC97_VENDOR_ID2);

		pr_debug("snd: VID %04x %04x\n", vid1, vid2);

		if(vid1 == WM97XX_ID1 && vid2 == WM9705_ID2) {
			pr_info("ac97: detected WM97XX codec\n");

			cpu_dai->ac97_pdata = &imx_cupid_wm9705_pdata;
		} else 
		if(vid1 == UCB_ID_1 && vid2 == UCB_ID_1400) {
			pr_info("ac97: dectected UCB1400 codec\n");
			pr_debug( "<%s> cpu dai: %s\n", __func__, rtd->cpu_dai->name);
			pr_debug( "<%s> codec dai: %s\n", __func__, rtd->codec_dai->name);
			pr_debug( "<%s> pdata: 0x%08x\n", __func__, (unsigned int)  &imx_cupid_ucb1400_pdata);

			cpu_dai->ac97_pdata = &imx_cupid_ucb1400_pdata;
		} else
		{
			pr_warn("<%s> Unknown AC97 Codec: VID %04x %04x, \n", __func__, vid1, vid2);
		}
	}
	return 0;
}

static struct snd_soc_dai_link imx_cupid_dai_ac97[] = {
	{
		.name        = "HiFi",
		.stream_name = "AC97 HiFi",
		.cpu_dai_name = "imx-ssi.0",
		.platform_name = "imx-ssi.0",
		.codec_name = "ac97-codec",
		.codec_dai_name = "ac97-hifi",
		.init		 = imx_cupid_dai_link_init,
	},
};

static struct snd_soc_card imx_cupid_snd_soc_card = {
	.name = "Cupid AC97",
	.owner = THIS_MODULE,
	.dai_link  = imx_cupid_dai_ac97,
	.num_links = ARRAY_SIZE(imx_cupid_dai_ac97),
};

static int __init imx_cupid_ac97_probe(struct platform_device *pdev)
{
	struct cupid_imx_ac97_pdata *pdata = pdev->dev.platform_data;
	
	int ret = 0;
	pr_debug("snd: imx_cupid_ac97_probe\n");

	imx_audmux_v2_configure_port(MX31_AUDMUX_PORT4_SSI_PINS_4, 
			  IMX_AUDMUX_V2_PTCR_SYN								// clocks equal for transmit and receive ( 4 wire)
			| IMX_AUDMUX_V2_PTCR_TFSEL(MX31_AUDMUX_PORT1_SSI0)		// Frame sync from port 0 ( ssi 0)
			| IMX_AUDMUX_V2_PTCR_TFSDIR,							// Frame sync is output
			  IMX_AUDMUX_V2_PDCR_RXDSEL(MX31_AUDMUX_PORT1_SSI0)	
		);	

	imx_audmux_v2_configure_port(MX31_AUDMUX_PORT1_SSI0, 
			  IMX_AUDMUX_V2_PTCR_SYN								// clocks equal for transmit and receive ( 4 wire)
			| IMX_AUDMUX_V2_PTCR_TCSEL(MX31_AUDMUX_PORT4_SSI_PINS_4)// TX Bitclk from port 4
			| IMX_AUDMUX_V2_PTCR_TCLKDIR,							// Frame sync is output
			  IMX_AUDMUX_V2_PDCR_RXDSEL(MX31_AUDMUX_PORT4_SSI_PINS_4)	
		);	

	if( pdata )
	{
		imx_cupid_wm9705_mach_ops.irq_gpio = pdata->irq_gpio;
		imx_cupid_wm9705_pdata.irq = gpio_to_irq(pdata->irq_gpio),
		imx_cupid_wm9705_pdata.enable_ts = pdata->enable_wm9705_ts;

		imx_cupid_ucb1400_pdata.irq_gpio = pdata->irq_gpio;
		imx_cupid_ucb1400_pdata.enable_ts = pdata->enable_ucb1400_ts;
	}

	imx_cupid_snd_soc_card.dev = &pdev->dev;

	ret =  snd_soc_register_card(&imx_cupid_snd_soc_card);
	if (ret) {
		pr_warn("Cupid platform: snd_soc_register_card failed: %d\n", ret);
	}
	return ret;
}

static int __exit imx_cupid_ac97_remove(struct platform_device * pdev)
{
	pr_err("Remove not implemented for imx_cupid_ac97_remove\n");
	return -1;
}

static const struct of_device_id imx_cupid_ac97_dt_ids[] = {
	{ .compatible = "guf,imx-audio-cupid_ac97", },
	{ /* sentinel */ }
};

static struct platform_driver imx_cupid_ac97_driver = {
	.probe = imx_cupid_ac97_probe,
	.remove = imx_cupid_ac97_remove,
	.driver = {
		.name = "imx-cupid_ac97",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_cupid_ac97_dt_ids,
	},
};
module_platform_driver(imx_cupid_ac97_driver);

MODULE_AUTHOR("Jonas Hoeppner <jonas.hoeppner@garz-fricke.de>");
MODULE_DESCRIPTION("CUPID ALSA SoC driver");
MODULE_LICENSE("GPL");
