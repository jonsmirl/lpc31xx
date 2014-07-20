/*
 * Copyright 2014 Emilio López <emilio@elopez.com.ar>
 * Copyright 2014 Jon Smirl <jonsmirl@gmail.com>
 *
 * Based on the Allwinner SDK driver, released under the GPL.
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/regmap.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>

/* Codec register offsets and bit fields */
#define SUNXI_DAC_DPC		(0x00)
#define EN_DA				(31)
#define DAC_VERSION			(23) /* For CODEC OLD VERSION */
#define DVOL				(12)
#define SUNXI_DAC_FIFOC		(0x04)
#define DAC_FS				(29)
#define FIR_VERSION			(28)
#define SEND_LASAT			(26)
#define TX_FIFO_MODE			(24)
#define DAC_DRQ_CLR_CNT			(21)
#define TX_TRIG_LEVEL			(8)
#define DAC_MONO_EN			(6)
#define TX_SAMPLE_BITS			(5)
#define DAC_DRQ_EN			(4)
#define DAC_FIFO_FLUSH			(0)
#define SUNXI_DAC_FIFOS		(0x08)
#define SUNXI_DAC_TXDATA	(0x0c)
#define SUNXI_DAC_ACTL		(0x10)
#define DACAENR				(31)
#define DACAENL				(30)
#define MIXEN				(29)
#define DACPAS				(8)
#define MIXPAS				(7)
#define PAMUTE				(6)
#define PAVOL				(0)
#define SUNXI_DAC_TUNE		(0x14)
#define SUNXI_DAC_DEBUG		(0x18)
#define SUNXI_ADC_FIFOC		(0x1c)
#define EN_AD				(28)
#define RX_FIFO_MODE			(24)
#define RX_TRIG_LEVEL			(8)
#define ADC_MONO_EN			(7)
#define RX_SAMPLE_BITS			(6)
#define ADC_DRQ_EN			(4)
#define ADC_FIFO_FLUSH			(0)
#define SUNXI_ADC_FIFOS		(0x20)
#define SUNXI_ADC_RXDATA	(0x24)
#define SUNXI_ADC_ACTL		(0x28)
#define ADCREN				(31)
#define ADCLEN				(30)
#define PREG1EN				(29)
#define PREG2EN				(28)
#define VMICEN				(27)
#define ADCG				(20)
#define ADCIS				(17)
#define PA_EN				(4)
#define DDE				(3)
#define SUNXI_ADC_DEBUG		(0x2c)
#define SUNXI_DAC_TXCNT		(0x30)
#define SUNXI_ADC_RXCNT		(0x34)
#define SUNXI_AC_SYS_VERI	(0x38)
#define SUNXI_AC_MIC_PHONE_CAL	(0x3c)


#define DAIFMT_16BITS             (16)
#define DAIFMT_20BITS             (20)

#define DAIFMT_BS_MASK            (~(1<<16)) /* FIFO big small mode mask */
#define DAIFMT_BITS_MASK          (~(1<<5))  /* FIFO Bits select mask, not used yet */
#define SAMPLE_RATE_MASK          (~(7<<29)) /* Sample Rate slect mask*/

#define DAC_CHANNEL		  (6)

enum sunxi_device_id {SUN4A, SUN4I, SUN5I, SUN7I};

struct sunxi_priv {
	struct regmap *regmap;
	int irq;
	struct clk *clk_apb, *clk_pll2, *clk_module;

	enum sunxi_device_id id;

	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
};

static int i2s_play_start(struct sunxi_priv *priv)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
#endif

	/* flush TX FIFO */
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_FIFO_FLUSH, 0x1 << DAC_FIFO_FLUSH);

	/* enable DAC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_DRQ_EN, 0x1 << DAC_DRQ_EN);
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << PAMUTE, 0x1 << PAMUTE);

	return 0;
}

static int i2s_play_stop(struct sunxi_priv *priv)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 0, "audio_pa_ctrl");
#endif

	/* mute PA */
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << PAMUTE, 0x0 << PAMUTE);
	mdelay(5);

	/* disable DAC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_DRQ_EN, 0x0 << DAC_DRQ_EN);

	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAENL, 0x0 << DACAENL);
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAENR, 0x0 << DACAENR);

	return 0;
}

static int i2s_capture_start(struct sunxi_priv *priv)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
#endif

	/* enable ADC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DRQ_EN, 0x1 << ADC_DRQ_EN);

	return 0;
}

static int i2s_capture_stop(struct sunxi_priv *priv)
{
	/* disable ADC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DRQ_EN, 0x0 << ADC_DRQ_EN);

	/* enable mic1 PA */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << PREG1EN, 0x0 << PREG1EN);

	/* enable VMIC */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << VMICEN, 0x0 << VMICEN);
	if (priv->id == SUN7I) {
		/* FIXME - undocumented */
		regmap_update_bits(priv->regmap, SUNXI_DAC_TUNE, 0x3 << 8, 0x0 << 8);
	}

	/* enable ADC digital */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);

	/* set RX FIFO mode */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << RX_FIFO_MODE, 0x0 << RX_FIFO_MODE);

	/* flush RX FIFO */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_FIFO_FLUSH, 0x0 << ADC_FIFO_FLUSH);

	/* enable adc1 analog */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x3 << ADCLEN, 0x0 << ADCLEN);

	return 0;
}

static int sunxi_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			i2s_capture_start(priv);
		else
			i2s_play_start(priv);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			i2s_capture_stop(priv);
		else
			i2s_play_stop(priv);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sunxi_i2s_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 0x1 << EN_DA, 0x1 << EN_DA);
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_FIFO_FLUSH, 0x1 << DAC_FIFO_FLUSH);
		/* set TX FIFO send DRQ level */
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x3f << TX_TRIG_LEVEL, 0xf << TX_TRIG_LEVEL);
		if (substream->runtime->rate > 32000) {
			regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << FIR_VERSION, 0x0 << FIR_VERSION);
		} else {
			regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << FIR_VERSION, 0x1 << FIR_VERSION);
		}
		/* set TX FIFO MODE - 0 works for both 16 and 24 bits */
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << TX_FIFO_MODE, 0x0 << TX_FIFO_MODE);
		/* send last sample when DAC FIFO under run */
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << SEND_LASAT, 0x0 << SEND_LASAT);
		/* enable dac analog */
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAENL, 0x1 << DACAENL);
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAENR, 0x1 << DACAENR);
		/* enable DAC to PA */
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACPAS, 0x1 << DACPAS);
	} else {
		/* enable mic1 PA */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << PREG1EN, 0x1 << PREG1EN);
		/* mic1 gain 32dB */  /* FIXME - makes no sense */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x3 << 25, 0x1 << 25);
		/* enable VMIC */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << VMICEN, 0x1 << VMICEN);

		if (priv->id == SUN7I) {
			/* boost up record effect */
			regmap_update_bits(priv->regmap, SUNXI_DAC_TUNE, 0x3 << 8, 0x1 << 8);
		}

		/* enable ADC digital */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		/* set RX FIFO mode */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << RX_FIFO_MODE, 0x1 << RX_FIFO_MODE);
		/* flush RX FIFO */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_FIFO_FLUSH, 0x1 << ADC_FIFO_FLUSH);
		/* set RX FIFO rec drq level */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0xf << RX_TRIG_LEVEL, 0x7 << RX_TRIG_LEVEL);
		/* enable adc1 analog */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x3 << ADCLEN, 0x3 << ADCLEN);
	}

	return 0;
}

static int sunxi_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);
	int is_mono = !!(params_channels(params) == 1);
	int is_24bit = !!(hw_param_interval(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS)->min == 32);
	unsigned int rate = params_rate(params);
	unsigned int hwrate;

	switch (rate) {
	case 176400:
	case 88200:
	case 44100:
	case 33075:
	case 22050:
	case 14700:
	case 11025:
	case 7350:
	default:
		clk_set_rate(priv->clk_pll2, 22579200);
		clk_set_rate(priv->clk_module, 22579200);
		break;
	case 192000:
	case 96000:
	case 48000:
	case 32000:
	case 24000:
	case 16000:
	case 12000:
	case 8000:
		clk_set_rate(priv->clk_pll2, 24576000);
		clk_set_rate(priv->clk_module, 24576000);
		break;
	}

	switch (rate) {
	case 192000:
	case 176400:
		hwrate = 6;
		break;
	case 96000:
	case 88200:
		hwrate = 7;
		break;
	default:
	case 48000:
	case 44100:
		hwrate = 0;
		break;
	case 32000:
	case 33075:
		hwrate = 1;
		break;
	case 24000:
	case 22050:
		hwrate = 2;
		break;
	case 16000:
	case 14700:
		hwrate = 3;
		break;
	case 12000:
	case 11025:
		hwrate = 4;
		break;
	case 8000:
	case 7350:
		hwrate = 5;
		break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 7 << DAC_FS, hwrate << DAC_FS);
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 1 << DAC_MONO_EN, is_mono << DAC_MONO_EN);
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 1 << TX_SAMPLE_BITS, is_24bit << TX_SAMPLE_BITS);
		if (is_24bit)
			priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		else
			priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	} else  {
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 7 << 29, hwrate << 29);
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 1 << ADC_MONO_EN, is_mono << ADC_MONO_EN);
	}

	return 0;
}

static const struct snd_kcontrol_new sun7i_dac_ctls[] = {
	/*SUNXI_DAC_ACTL = 0x10,PAVOL*/
	SOC_SINGLE("Master Playback Volume", SUNXI_DAC_ACTL, 0, 0x3f, 0),
	SOC_SINGLE("Playback Switch", SUNXI_DAC_ACTL, 6, 1, 0), //全局输出开关
	SOC_SINGLE("FmL Switch", SUNXI_DAC_ACTL, 17, 1, 0), //Fm左开关
	SOC_SINGLE("FmR Switch", SUNXI_DAC_ACTL, 16, 1, 0), //Fm右开关
	SOC_SINGLE("LineL Switch", SUNXI_DAC_ACTL, 19, 1, 0), //Line左开关
	SOC_SINGLE("LineR Switch", SUNXI_DAC_ACTL, 18, 1, 0), //Line右开关
	SOC_SINGLE("Ldac Left Mixer", SUNXI_DAC_ACTL, 15, 1, 0),
	SOC_SINGLE("Rdac Right Mixer", SUNXI_DAC_ACTL, 14, 1, 0),
	SOC_SINGLE("Ldac Right Mixer", SUNXI_DAC_ACTL, 13, 1, 0),
	SOC_SINGLE("Mic Input Mux", SUNXI_DAC_ACTL, 9, 15, 0), //from bit 9 to bit 12.Mic（麦克风）输入静音
	SOC_SINGLE("MIC output volume", SUNXI_DAC_ACTL, 20, 7, 0),
	/*	FM Input to output mixer Gain Control
	* 	From -4.5db to 6db,1.5db/step,default is 0db
	*	-4.5db:0x0,-3.0db:0x1,-1.5db:0x2,0db:0x3
	*	1.5db:0x4,3.0db:0x5,4.5db:0x6,6db:0x7
	*/
	SOC_SINGLE("Fm output Volume", SUNXI_DAC_ACTL, 23, 7, 0),
	/*	Line-in gain stage to output mixer Gain Control
	*	0:-1.5db,1:0db
	*/
	SOC_SINGLE("Line output Volume", SUNXI_DAC_ACTL, 26, 1, 0),

	SOC_SINGLE("Master Capture Mute", SUNXI_ADC_ACTL, 4, 1, 0),
	SOC_SINGLE("Right Capture Mute", SUNXI_ADC_ACTL, 31, 1, 0),
	SOC_SINGLE("Left Capture Mute", SUNXI_ADC_ACTL, 30, 1, 0),
	SOC_SINGLE("Linein Pre-AMP", SUNXI_ADC_ACTL, 13, 7, 0),
	SOC_SINGLE("LINEIN APM Volume", SUNXI_AC_MIC_PHONE_CAL, 13, 0x7, 0),
	/* ADC Input Gain Control, capture volume
	* 000:-4.5db,001:-3db,010:-1.5db,011:0db,100:1.5db,101:3db,110:4.5db,111:6db
	*/
	SOC_SINGLE("Capture Volume", SUNXI_ADC_ACTL, 20, 7, 0),
	/*
	*	MIC2 pre-amplifier Gain Control
	*	00:0db,01:35db,10:38db,11:41db
	*/
	SOC_SINGLE("MicL Volume", SUNXI_ADC_ACTL, 25, 3, 0), //mic左音量
	SOC_SINGLE("MicR Volume", SUNXI_ADC_ACTL, 23, 3, 0), //mic右音量
	SOC_SINGLE("Mic2 Boost", SUNXI_ADC_ACTL, 29, 1, 0),
	SOC_SINGLE("Mic1 Boost", SUNXI_ADC_ACTL, 28, 1, 0),
	SOC_SINGLE("Mic Power", SUNXI_ADC_ACTL, 27, 1, 0),
	SOC_SINGLE("ADC Input Mux", SUNXI_ADC_ACTL, 17, 7, 0), //ADC输入静音
	SOC_SINGLE("Mic2 gain Volume", SUNXI_AC_MIC_PHONE_CAL, 26, 7, 0),
	/*
	*	MIC1 pre-amplifier Gain Control
	*	00:0db,01:35db,10:38db,11:41db
	*/
	SOC_SINGLE("Mic1 gain Volume", SUNXI_AC_MIC_PHONE_CAL, 29, 3, 0),
};

static int sunxi_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct snd_soc_card *card = snd_soc_dai_get_drvdata(dai);
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	snd_soc_dai_init_dma_data(dai, &priv->playback_dma_data, &priv->capture_dma_data);

	return 0;
}

static int sunxi_i2s_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	return clk_prepare_enable(priv->clk_module);
}

static void sunxi_i2s_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(priv->clk_module);
}


static int i2s_init(struct sunxi_priv *priv)
{
	/* enable DAC digital */
	regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 1 << EN_DA, 1 << EN_DA);

	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 1 << FIR_VERSION, 1 << FIR_VERSION);

	/* set digital volume to maximum */
	if (priv->id == SUN4A)
		regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 0x3F << DVOL, 0 << DVOL);

	/* PA mute */
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 1 << PAMUTE, 0 << PAMUTE);

	/* enable PA */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 1 << PA_EN, 1 << PA_EN);
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 3 << DAC_DRQ_CLR_CNT, 3 << DAC_DRQ_CLR_CNT);

	/* set volume */
	if (priv->id == SUN4A)
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x3f << PAVOL, 1 << PAVOL);
	else
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x3f << PAVOL, 0x30 << PAVOL);

	return 0;
}

static const struct snd_soc_dai_ops sunxi_i2s_dai_ops = {
	.startup = sunxi_i2s_startup,
	.shutdown = sunxi_i2s_shutdown,
	.trigger = sunxi_i2s_trigger,
	.hw_params = sunxi_i2s_hw_params,
	.prepare = sunxi_i2s_prepare,
};

static struct snd_soc_dai_driver sunxi_i2s_dai = {
	.probe = sunxi_i2s_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE),

		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_11025 |\
			 SNDRV_PCM_RATE_22050| SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
			 SNDRV_PCM_RATE_48000 |SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			 SNDRV_PCM_RATE_KNOT),
		.rate_min = 8000,
		.rate_max = 192000,
	},
	.ops = &sunxi_i2s_dai_ops,
};

static const struct snd_soc_component_driver sunxi_i2s_component = {
	.name = "sunxi-i2s",
};

static const struct regmap_config sunxi_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SUNXI_AC_MIC_PHONE_CAL,
};

static const struct snd_soc_dapm_widget i2s_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Mic Bias"),
	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_INPUT("MIC_IN"),
	SND_SOC_DAPM_INPUT("LINE_IN"),
};

static const struct of_device_id sunxi_i2s_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10a-i2s", .data = (void *)SUN4A},
	{ .compatible = "allwinner,sun4i-a10-i2s", .data = (void *)SUN4I},
	{ .compatible = "allwinner,sun5i-a13-i2s", .data = (void *)SUN5I},
	{ .compatible = "allwinner,sun7i-a20-i2s", .data = (void *)SUN7I},
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_i2s_of_match);

static int sunxi_i2s_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct sunxi_priv *priv;
	struct resource *res;
	void __iomem *base;
	int ret;

	if (!of_device_is_available(np))
		return -ENODEV;

	of_id = of_match_device(sunxi_i2s_of_match, dev);
	if (!of_id)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->id = (int)of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &sunxi_i2s_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->irq = irq_of_parse_and_map(np, 0);
	if (!priv->irq) {
		dev_err(dev, "no irq for node %s\n", np->full_name);
		return -ENXIO;
	}

	/* Get the clocks from the DT */
	priv->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->clk_apb)) {
		dev_err(dev, "failed to get apb clock\n");
		return PTR_ERR(priv->clk_apb);
	}
	priv->clk_pll2 = devm_clk_get(dev, "pll2");
	if (IS_ERR(priv->clk_pll2)) {
		dev_err(dev, "failed to get pll2 clock\n");
		return PTR_ERR(priv->clk_pll2);
	}
	priv->clk_module = devm_clk_get(dev, "i2s");
	if (IS_ERR(priv->clk_module)) {
		dev_err(dev, "failed to get i2s clock\n");
		return PTR_ERR(priv->clk_module);
	}

	/* Enable PLL2 on a basic rate */
	ret = clk_set_rate(priv->clk_pll2, 24576000);
	if (ret) {
		dev_err(dev, "failed to set i2s base clock rate\n");
		return ret;
	}
	if (clk_prepare_enable(priv->clk_pll2)) {
		dev_err(dev, "failed to enable pll2 clock\n");
		return -EINVAL;
	}

	/* Enable the bus clock */
	if (clk_prepare_enable(priv->clk_apb)) {
		dev_err(dev, "failed to enable apb clock\n");
		clk_disable_unprepare(priv->clk_pll2);
		return -EINVAL;
	}

	/* DMA configuration for TX FIFO */
	priv->playback_dma_data.addr = res->start + SUNXI_DAC_TXDATA;
	priv->playback_dma_data.maxburst = 4;
	priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	/* DMA configuration for RX FIFO */
	priv->capture_dma_data.addr = res->start + SUNXI_ADC_RXDATA;
	priv->capture_dma_data.maxburst = 4;
	priv->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	ret = devm_snd_soc_register_component(&pdev->dev, &sunxi_i2s_component, &sunxi_i2s_dai, 1);
	if (ret)
		goto err_clk_disable;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_clk_disable;

	i2s_init(priv);

	return 0;

err_clk_disable:
	clk_disable_unprepare(priv->clk_apb);
	clk_disable_unprepare(priv->clk_pll2);
	return ret;
}

static int sunxi_i2s_remove(struct platform_device *pdev)
{
	struct sunxi_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk_apb);
	clk_disable_unprepare(priv->clk_pll2);

	return 0;
}

static struct platform_driver sunxi_i2s_driver = {
	.driver = {
		.name = "sunxi-i2s",
		.owner = THIS_MODULE,
		.of_match_table = sunxi_i2s_of_match,
	},
	.probe = sunxi_i2s_probe,
	.remove = sunxi_i2s_remove,
};
module_platform_driver(sunxi_i2s_driver);

MODULE_DESCRIPTION("sunxi i2s ASoC driver");
MODULE_AUTHOR("Emilio López <emilio@elopez.com.ar>");
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
