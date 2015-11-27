/*
 * imx-ssi.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2009 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This code is based on code copyrighted by Freescale,
 * Liam Girdwood, Javier Martin and probably others.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 * The i.MX SSI core has some nasty limitations in AC97 mode. While most
 * sane processor vendors have a FIFO per AC97 slot, the i.MX has only
 * one FIFO which combines all valid receive slots. We cannot even select
 * which slots we want to receive. The WM9712 with which this driver
 * was developed with always sends GPIO status data in slot 12 which
 * we receive in our (PCM-) data stream. The only chance we have is to
 * manually skip this data in the FIQ handler. With sampling rates different
 * from 48000Hz not every frame has valid receive data, so the ratio
 * between pcm data and GPIO status data changes. Our FIQ handler is not
 * able to handle this, hence this driver only works with 48000Hz sampling
 * rate.
 * Reading and writing AC97 registers is another challenge. The core
 * provides us status bits when the read register is updated with *another*
 * value. When we read the same register two times (and the register still
 * contains the same value) these status bits are not set. We work
 * around this by not polling these bits but only wait a fixed delay.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/platform_data/asoc-imx-ssi.h>

#include "imx-ssi.h"

#define SSI_SACNT_DEFAULT (SSI_SACNT_AC97EN | SSI_SACNT_FV)

// after reset/power-up the AC97 codec needs some time before
// it is operational. This is signalled by the CODEC_READY bit
// in the TAG-slot sent by the codec. Unfortunately neither AC97
// spec nor USB1400 datasheet seem to specify how long the codec
// may take from power-on/reset until it is usable...
//
// (time-out given in number of 48KHz AC97 frames)
#define AC97_START_TIMEOUT	20000

// all AC97-controller actions performed below should execute
// within one single AC97 frame. A time-out of 2 frames (to
// compensate for frame synchronization) should therefore
// be more than enough...
//
// (time-out given in number of 48KHz AC97 frames)
#define AC97_POLL_TIMEOUT	2
// (one 48kHz Frame needs 20.83 us)
#define AC97_FRAME_TIME_USEC 21

/* Frame tags */
#define TAG_COMMAND_READ	0xC000
#define TAG_COMMAND_WRITE	0xE000
#define TAG_PCM_OUT			0x9800
#define TAG_VALID			0x8000

/*
 * SSI Network Mode or TDM slots configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct imx_ssi *ssi = snd_soc_dai_get_drvdata(cpu_dai);
	u32 sccr;
	pr_debug("<%s> tx %d rx %d slots %d, slotwidth %d\n", __func__, tx_mask, rx_mask, slots, slot_width);

	sccr = readl(ssi->base + SSI_STCCR);
	sccr &= ~SSI_STCCR_DC_MASK;
	sccr |= SSI_STCCR_DC(slots - 1);
	writel(sccr, ssi->base + SSI_STCCR);

	sccr = readl(ssi->base + SSI_SRCCR);
	sccr &= ~SSI_STCCR_DC_MASK;
	sccr |= SSI_STCCR_DC(slots - 1);
	writel(sccr, ssi->base + SSI_SRCCR);

	writel(tx_mask, ssi->base + SSI_STMSK);
	writel(rx_mask, ssi->base + SSI_SRMSK);

	return 0;
}

/*
 * SSI DAI format configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 * Note: We don't use the I2S modes but instead manually configure the
 * SSI for I2S because the I2S mode is only a register preset.
 */
static int imx_ssi_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct imx_ssi *ssi = snd_soc_dai_get_drvdata(cpu_dai);
	u32 strcr = 0, scr;

	pr_debug("<%s> 0x%08X\n", __func__, fmt);
	scr = readl(ssi->base + SSI_SCR) & ~(SSI_SCR_SYN | SSI_SCR_NET);

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data on rising edge of bclk, frame low 1clk before data */
		strcr |= SSI_STCR_TFSI | SSI_STCR_TEFS | SSI_STCR_TXBIT0;
		scr |= SSI_SCR_NET;
		if (ssi->flags & IMX_SSI_USE_I2S_SLAVE) {
			scr &= ~SSI_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_SLAVE;
		}
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data on rising edge of bclk, frame high with data */
		strcr |= SSI_STCR_TXBIT0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* data on rising edge of bclk, frame high with data */
		strcr |= SSI_STCR_TFSL | SSI_STCR_TXBIT0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* data on rising edge of bclk, frame high 1clk before data */
		strcr |= SSI_STCR_TFSL | SSI_STCR_TXBIT0 | SSI_STCR_TEFS;
		break;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		strcr |= SSI_STCR_TFSI;
		strcr &= ~SSI_STCR_TSCKP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		strcr &= ~(SSI_STCR_TSCKP | SSI_STCR_TFSI);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		strcr |= SSI_STCR_TFSI | SSI_STCR_TSCKP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		strcr &= ~SSI_STCR_TFSI;
		strcr |= SSI_STCR_TSCKP;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		/* Master mode not implemented, needs handling of clocks. */
		return -EINVAL;
	}

	strcr |= SSI_STCR_TFEN0;

	if (ssi->flags & IMX_SSI_NET)
		scr |= SSI_SCR_NET;
	if (ssi->flags & IMX_SSI_SYN)
		scr |= SSI_SCR_SYN;

	writel(strcr, ssi->base + SSI_STCR);
	writel(strcr, ssi->base + SSI_SRCR);
	writel(scr, ssi->base + SSI_SCR);

	return 0;
}

/*
 * SSI system clock configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct imx_ssi *ssi = snd_soc_dai_get_drvdata(cpu_dai);
	u32 scr;

	scr = readl(ssi->base + SSI_SCR);

	switch (clk_id) {
	case IMX_SSP_SYS_CLK:
		if (dir == SND_SOC_CLOCK_OUT)
			scr |= SSI_SCR_SYS_CLK_EN;
		else
			scr &= ~SSI_SCR_SYS_CLK_EN;
		break;
	default:
		return -EINVAL;
	}

	writel(scr, ssi->base + SSI_SCR);

	return 0;
}

/*
 * SSI Clock dividers
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	struct imx_ssi *ssi = snd_soc_dai_get_drvdata(cpu_dai);
	u32 stccr, srccr;

	stccr = readl(ssi->base + SSI_STCCR);
	srccr = readl(ssi->base + SSI_SRCCR);

	switch (div_id) {
	case IMX_SSI_TX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	case IMX_SSI_RX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	default:
		return -EINVAL;
	}

	writel(stccr, ssi->base + SSI_STCCR);
	writel(srccr, ssi->base + SSI_SRCCR);

	return 0;
}

/*
 * Should only be called when port is inactive (i.e. SSIEN = 0),
 * although can be called multiple times by upper layers.
 */
static int imx_ssi_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *ssi = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg, sccr;
	pr_debug("<%s>\n",__func__);

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = SSI_STCCR;
	else
		reg = SSI_SRCCR;

	if (ssi->flags & IMX_SSI_SYN)
		reg = SSI_STCCR;

	sccr = readl(ssi->base + reg) & ~SSI_STCCR_WL_MASK;

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sccr |= SSI_SRCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sccr |= SSI_SRCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sccr |= SSI_SRCCR_WL(24);
		break;
	}

	writel(sccr, ssi->base + reg);

	return 0;
}

static int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi = snd_soc_dai_get_drvdata(dai);
	unsigned int sier_bits, sier;
	unsigned int scr;
	pr_debug("<%s>\n",__func__);

	scr = readl(ssi->base + SSI_SCR);
	sier = readl(ssi->base + SSI_SIER);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (ssi->flags & IMX_SSI_DMA)
			sier_bits = SSI_SIER_TDMAE;
		else
			sier_bits = SSI_SIER_TIE | SSI_SIER_TFE0_EN;
	} else {
		if (ssi->flags & IMX_SSI_DMA)
			sier_bits = SSI_SIER_RDMAE;
		else
			sier_bits = SSI_SIER_RIE | SSI_SIER_RFF0_EN;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr |= SSI_SCR_TE;
		else
			scr |= SSI_SCR_RE;
		sier |= sier_bits;

		if (++ssi->enabled == 1)
			scr |= SSI_SCR_SSIEN;

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr &= ~SSI_SCR_TE;
		else
			scr &= ~SSI_SCR_RE;
		sier &= ~sier_bits;

		if (--ssi->enabled == 0)
			scr &= ~SSI_SCR_SSIEN;

		break;
	default:
		return -EINVAL;
	}

	if (!(ssi->flags & IMX_SSI_USE_AC97))
		/* rx/tx are always enabled to access ac97 registers */
		writel(scr, ssi->base + SSI_SCR);

	writel(sier, ssi->base + SSI_SIER);

	return 0;
}

static const struct snd_soc_dai_ops imx_ssi_pcm_dai_ops = {
	.hw_params	= imx_ssi_hw_params,
	.set_fmt	= imx_ssi_set_dai_fmt,
	.set_clkdiv	= imx_ssi_set_dai_clkdiv,
	.set_sysclk	= imx_ssi_set_dai_sysclk,
	.set_tdm_slot	= imx_ssi_set_dai_tdm_slot,
	.trigger	= imx_ssi_trigger,
};

static int imx_ssi_dai_probe(struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi = dev_get_drvdata(dai->dev);
	uint32_t val;
	pr_debug("<%s>\n", __func__);
	snd_soc_dai_set_drvdata(dai, ssi);

	val = SSI_SFCSR_TFWM0(ssi->dma_params_tx.maxburst) |
		SSI_SFCSR_RFWM0(ssi->dma_params_rx.maxburst);
	writel(val, ssi->base + SSI_SFCSR);

	/* Tx/Rx config */
	dai->playback_dma_data = &ssi->dma_params_tx;
	dai->capture_dma_data = &ssi->dma_params_rx;

	return 0;
}

static struct snd_soc_dai_driver imx_ssi_dai = {
	.probe = imx_ssi_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &imx_ssi_pcm_dai_ops,
};

static struct snd_soc_dai_driver imx_ac97_dai = {
	.probe = imx_ssi_dai_probe,
	.ac97_control = 1,
	.playback = {
		.stream_name = "AC97 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "AC97 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &imx_ssi_pcm_dai_ops,
};

static const struct snd_soc_component_driver imx_component = {
	.name		= DRV_NAME,
};
static struct imx_ssi *ac97_ssi;

static int is_timeout(struct timeval tv_start, int frames)
{
	struct timeval tv_cur;
	unsigned long usec_passed;

	do_gettimeofday(&tv_cur);
	usec_passed = (tv_cur.tv_sec - tv_start.tv_sec) * 1000000 + tv_cur.tv_usec - tv_start.tv_usec;
	if (usec_passed > frames * AC97_FRAME_TIME_USEC)
		return true;

	return false;
}


static int codec_ready(void)
{
	struct imx_ssi *imx_ssi = ac97_ssi;
	void __iomem *base = imx_ssi->base;
	struct timeval tv;

	// check if the codec is in a ready state
	// (signalled via AC97-TAG sent by codec)
	if (!(readl(base + SSI_SATAG) & 0x8000))
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_debug("codec NOT ready\n");
		spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
		return false;
	}

	// synchronize with AC97 frames
	// (wait for transmit frame sync)
	do_gettimeofday(&tv);
	while (!(readl(base + SSI_SISR) & SSI_SISR_TFS) && !is_timeout(tv, AC97_POLL_TIMEOUT))
		;

	return true;
}


static void setup_channel_to_ac97(struct imx_ssi *imx_ssi)
{
	void __iomem *base = imx_ssi->base;
	
	writel(0x0, base + SSI_SCR);
	writel(0x0, base + SSI_STCR);
	writel(0x0, base + SSI_SRCR);

	writel(SSI_SCR_SYN | SSI_SCR_NET, base + SSI_SCR);

	writel(SSI_SFCSR_RFWM0(8) |
		SSI_SFCSR_TFWM0(8) |
		SSI_SFCSR_RFWM1(8) |
		SSI_SFCSR_TFWM1(8), base + SSI_SFCSR);

	writel(SSI_STCCR_WL(16) | SSI_STCCR_DC(12), base + SSI_STCCR);
	writel(SSI_STCCR_WL(16) | SSI_STCCR_DC(12), base + SSI_SRCCR);

	writel(SSI_SCR_SYN | SSI_SCR_NET | SSI_SCR_SSIEN, base + SSI_SCR);
	writel(SSI_SOR_WAIT(3), base + SSI_SOR);

	writel(SSI_SCR_SYN | SSI_SCR_NET | SSI_SCR_SSIEN |
			SSI_SCR_TE | SSI_SCR_RE,
			base + SSI_SCR);

	writel(SSI_SACNT_DEFAULT, base + SSI_SACNT);
	writel(0xff, base + SSI_SACCDIS);
	writel(0x300, base + SSI_SACCEN);

}

static int imx_ssi_ac97_clear_mode(struct snd_ac97 *ac97)
{
	struct imx_ssi *imx_ssi = ac97_ssi;
	void __iomem *base = imx_ssi->base;
	unsigned int lreg;

	// clear all AC97 command bits in controller
	lreg = readl(base + SSI_SACNT);
	lreg &= ~(SSI_SACNT_RD | SSI_SACNT_WR);
	writel(lreg, base + SSI_SACNT);

	// verify that they have been cleared successfully
	if (readl(base + SSI_SACNT) & (SSI_SACNT_RD | SSI_SACNT_WR))
		return false;

	return true;
}

static int imx_ssi_ac97_wait_comm_ready(struct snd_ac97 *ac97)
{
	struct imx_ssi *imx_ssi = ac97_ssi;
	int res;
	struct timeval tv;

	// check that AC97 codec is ready
	do_gettimeofday(&tv);
	do
	{
		res = codec_ready();
	} while (!res && !is_timeout(tv, AC97_POLL_TIMEOUT));

	if (!res)
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: TIMEOUT - could not retrieve AC97 codec ready bit\n", __func__);
		spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
		return false;
	}

	// clear command bits in AC97 controller
	do_gettimeofday(&tv);
	do
	{
		res = imx_ssi_ac97_clear_mode(ac97);
	} while (!res && !is_timeout(tv, AC97_POLL_TIMEOUT));

	if (!res)
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: TIMEOUT - imx_ssi_ac97_clear_mode() failed\n", __func__);
		spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
		return false;
	}

	return true;
}

static int imx_ssi_ac97_issue_command(struct snd_ac97 *ac97, unsigned int command)
{
	struct imx_ssi *imx_ssi = ac97_ssi;
	void __iomem *base = imx_ssi->base;
	struct timeval tv;
	unsigned int lreg;
	int res = -1;
	int nTries = 0;
	int cmd_bit = (command == TAG_COMMAND_WRITE) ? SSI_SACNT_WR : SSI_SACNT_RD;

	// set command bit in AC97 controller
	do_gettimeofday(&tv);
	do
	{
		// set command bit in controller
		// Note: Once should be enough, but at least on i.MX35 occasionally
		// it is not. While writing the register twice is not a nice way either
		// it has shown to work running on three different systems (NESO, CUPID,
		// MC2) for three days and nights of continuous AC97 register read/write
		// commands without any problems.

		lreg = readl(base + SSI_SACNT);
		lreg |= cmd_bit;
		writel(lreg, base + SSI_SACNT);
		writel(lreg, base + SSI_SACNT);

		// verify that it has been set
		res = ((readl(base + SSI_SACNT) & cmd_bit) != 0);
	} while (!res && (!is_timeout(tv, AC97_POLL_TIMEOUT) || (nTries++ < 2)));

	if (!res)
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: TIMEOUT - setting write mode failed\n", __func__);
		spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
		return false;
	}

	// wait until controller clears command bit to signal successful send
	do_gettimeofday(&tv);
	while((readl(base + SSI_SACNT) & cmd_bit) && !is_timeout(tv, AC97_POLL_TIMEOUT))
		;

	if (readl(base + SSI_SACNT) & cmd_bit)
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: TIMEOUT - couldn't get cleared write/read bit\n", __func__);
		spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
		return false;
	}

	// if we just sent a read-command, we still have to wait for
	// the returned data from the AC97 codec...
	if (command == TAG_COMMAND_READ)
	{
		// do a dummy read of the SISR to clear synch-flags from
		// frame #n (in which we just sent the READ-command)
		readl(base + SSI_SISR);

		// wait for last receive time-slot of frame #n+1
		// (in this frame the codec will return our data)
		do_gettimeofday(&tv);
		while (!(readl(base + SSI_SISR) & SSI_SISR_RLS) && !is_timeout(tv, AC97_POLL_TIMEOUT))
			;

		// wait for last receive time-slot of frame #n+2
		// (currently frame #n+1 is not yet finished, so we
		// still have to wait until the controller actually
		// provides the read data...)
		do_gettimeofday(&tv);
		while (!(readl(base + SSI_SISR) & SSI_SISR_RLS) && !is_timeout(tv, AC97_POLL_TIMEOUT))
			;
	}

	return true;
}

static void imx_ssi_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
		unsigned short val)
{
	struct imx_ssi *imx_ssi = ac97_ssi;
	void __iomem *base = imx_ssi->base;
	unsigned int lreg;
	unsigned int lval;

	if (reg > 0x7f)
		return;

	pr_debug("%s: 0x%02x 0x%04x\n", __func__, reg, val);

	spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
	// wait until codec and controller are ready
	if (!imx_ssi_ac97_wait_comm_ready(ac97))
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: failed waiting for AC97 ready\n", __func__);
		return;
	}

	// write register address and data
	lreg = reg <<  12;
	writel(lreg, base + SSI_SACADD);

	lval = val << 4;
	writel(lval , base + SSI_SACDAT);

	// wait for command completion
	if (!imx_ssi_ac97_issue_command(ac97, TAG_COMMAND_WRITE))
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: failed issuing AC97 command\n", __func__);
		return;
	}
	spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);

}

static unsigned short imx_ssi_ac97_read(struct snd_ac97 *ac97,
		unsigned short reg)
{
	struct imx_ssi *imx_ssi = ac97_ssi;
	void __iomem *base = imx_ssi->base;

#ifdef DEBUG
	// Dump all ssi registers
	{
		pr_debug("<%s>0 %d, imx_ssi->base: 0x%08X\n", __func__, reg, base);
		int i;
		for( i = 0; i <= 0x58; i+=4)
		{
			void __iomem *reg = base + i;
			pr_debug("<%s> 0x%08X: 0x%08X\n",__func__, reg, readl(reg));
		}
	}
#endif

	unsigned short val = -1;
	unsigned int lreg;

	spin_lock_irqsave(&imx_ssi->lock, imx_ssi->lock_flags);
	// wait until codec and controller are ready
	if (!imx_ssi_ac97_wait_comm_ready(ac97))
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: failed waiting for AC97 ready\n", __func__);
		return -1;
	}

	// write register address...
	lreg = (reg & 0x7f) <<  12 ;
	writel(lreg, base + SSI_SACADD);

	// wait for command completion
	if (!imx_ssi_ac97_issue_command(ac97, TAG_COMMAND_READ))
	{
		spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
		pr_err("%s: failed issuing AC97 command\n", __func__);
		return -1;
	}

	// read back data
	val = (readl(base + SSI_SACDAT) >> 4) & 0xffff;

	spin_unlock_irqrestore(&imx_ssi->lock, imx_ssi->lock_flags);
	pr_debug("%s: 0x%02x 0x%04x\n", __func__, reg, val);

	return val;
}

static void imx_ssi_ac97_reset(struct snd_ac97 *ac97)
{
	struct imx_ssi *imx_ssi = ac97_ssi;

	if (imx_ssi->ac97_reset)
		imx_ssi->ac97_reset(ac97);
	/* First read sometimes fails, do a dummy read */
	imx_ssi_ac97_read(ac97, 0);
}

static void imx_ssi_ac97_warm_reset(struct snd_ac97 *ac97)
{
	struct imx_ssi *imx_ssi = ac97_ssi;

	if (imx_ssi->ac97_warm_reset)
		imx_ssi->ac97_warm_reset(ac97);

	/* First read sometimes fails, do a dummy read */
	imx_ssi_ac97_read(ac97, 0);
}

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read		= imx_ssi_ac97_read,
	.write		= imx_ssi_ac97_write,
	.reset		= imx_ssi_ac97_reset,
	.warm_reset	= imx_ssi_ac97_warm_reset
};
EXPORT_SYMBOL_GPL(soc_ac97_ops);

struct clk {
	const char		*name;
};
static int imx_ssi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct imx_ssi *ssi;
	struct imx_ssi_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;
	struct snd_soc_dai_driver *dai;

	pr_debug("<%s>\n", __func__);

	ssi = devm_kzalloc(&pdev->dev, sizeof(*ssi), GFP_KERNEL);
	if (!ssi)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, ssi);

	if (pdata) {
		ssi->ac97_reset = pdata->ac97_reset;
		ssi->ac97_warm_reset = pdata->ac97_warm_reset;
		ssi->toggle_speaker = pdata->toggle_speaker;
		ssi->flags = pdata->flags;
	}

	ssi->irq = platform_get_irq(pdev, 0);

	ssi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ssi->clk)) {
		ret = PTR_ERR(ssi->clk);
		dev_err(&pdev->dev, "Cannot get the clock: %d\n",
			ret);
		goto failed_clk;
	}
	pr_debug("SSI Clk: %s\n", ssi->clk->name);
	clk_prepare_enable(ssi->clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto failed_get_resource;
	}

	ssi->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ssi->base)) {
		ret = PTR_ERR(ssi->base);
		goto failed_register;
	}

	if (ssi->flags & IMX_SSI_USE_AC97) {
		if (ac97_ssi) {
			dev_err(&pdev->dev, "AC'97 SSI already registered\n");
			ret = -EBUSY;
			goto failed_register;
		}
		ac97_ssi = ssi;
		setup_channel_to_ac97(ssi);
		dai = &imx_ac97_dai;
	} else
		dai = &imx_ssi_dai;

	writel(0x0, ssi->base + SSI_SIER);

	ssi->dma_params_rx.addr = res->start + SSI_SRX0;
	ssi->dma_params_tx.addr = res->start + SSI_STX0;
	
	ssi->dma_params_tx.maxburst = 6;
	ssi->dma_params_rx.maxburst = 4;

	ssi->dma_params_tx.filter_data = &ssi->filter_data_tx;
	ssi->dma_params_rx.filter_data = &ssi->filter_data_rx;


	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx0");
	if (res) {
		pr_debug("<%s> DMA res tx: 0x%08x\n",__func__, res->start);
		imx_pcm_dma_params_init_data(&ssi->filter_data_tx, res->start,
			false);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx0");
	if (res) {
		pr_debug("<%s> DMA res rx: 0x%08x\n",__func__, res->start);
		imx_pcm_dma_params_init_data(&ssi->filter_data_rx, res->start,
			false);
	}

	platform_set_drvdata(pdev, ssi);

	ret = snd_soc_register_component(&pdev->dev, &imx_component, dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		goto failed_register;
	}

	ret = imx_pcm_fiq_init(pdev);
	if (ret)
		goto failed_pcm_fiq;

	ret = imx_pcm_dma_init(pdev, SND_DMAENGINE_PCM_FLAG_NO_RESIDUE |
				     SND_DMAENGINE_PCM_FLAG_NO_DT |
				     SND_DMAENGINE_PCM_FLAG_COMPAT,
				     IMX_SSI_DMABUF_SIZE);
	if (ret)
		goto failed_pcm_dma;

	return 0;

failed_pcm_dma:
	pr_debug("<%s>: failed_pcm_dma\n", __func__);
	imx_pcm_fiq_exit(pdev);
failed_pcm_fiq:
	pr_debug("<%s>: failed_pcm_fiq\n", __func__);
	snd_soc_unregister_component(&pdev->dev);

failed_get_resource:
failed_register:
	pr_debug("<%s>: failed_register\n", __func__);
	release_mem_region(res->start, resource_size(res));
	clk_disable_unprepare(ssi->clk);
failed_clk:
	pr_debug("<%s>: failed_clk\n", __func__);
	return ret;
}

static int imx_ssi_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct imx_ssi *ssi = platform_get_drvdata(pdev);

	imx_pcm_dma_exit(pdev);
	imx_pcm_fiq_exit(pdev);

	snd_soc_unregister_component(&pdev->dev);

	if (ssi->flags & IMX_SSI_USE_AC97)
		ac97_ssi = NULL;

	release_mem_region(res->start, resource_size(res));
	clk_disable_unprepare(ssi->clk);

	return 0;
}

static struct platform_driver imx_ssi_driver = {
	.probe = imx_ssi_probe,
	.remove = imx_ssi_remove,

	.driver = {
		.name = "imx-ssi",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(imx_ssi_driver);

/* Module information */
MODULE_AUTHOR("Sascha Hauer, <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("i.MX I2S/ac97 SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ssi");
