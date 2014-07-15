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
#define DAC_EN				(31)
#define DIGITAL_VOL			(12)
#define DAC_VERSION			(23) /* For CODEC OLD VERSION */
#define SUNXI_DAC_FIFOC		(0x04)
#define LAST_SE				(26)
#define TX_FIFO_MODE			(24)
#define DRA_LEVEL			(21)
#define TX_TRI_LEVEL			(8)
#define DAC_MODE			(6) /* not yet in use */
#define TASR				(5) /* not yet in use */
#define DAC_DRQ				(4)
#define DAC_FIFO_FLUSH			(0)
#define SUNXI_DAC_FIFOS		(0x08)
#define SUNXI_DAC_TXDATA	(0x0c)
#define SUNXI_DAC_ACTL		(0x10)
#define VOLUME				(0)
#define PA_MUTE				(6)
#define MIXPAS				(7)
#define DACPAS				(8)
#define MIXEN				(29)
#define DACAEN_L			(30)
#define DACAEN_R			(31)
#define SUNXI_DAC_TUNE		(0x14)
#define SUNXI_DAC_DEBUG		(0x18)
#define SUNXI_ADC_FIFOC		(0x1c)
#define ADC_DIG_EN			(28)
#define RX_FIFO_MODE			(24)
#define RX_TRI_LEVEL			(8)
#define ADC_MODE			(7)
#define RASR				(6)
#define ADC_DRQ				(4)
#define ADC_FIFO_FLUSH			(0)
#define SUNXI_ADC_FIFOS		(0x20)
#define SUNXI_ADC_RXDATA	(0x24)
#define SUNXI_ADC_ACTL		(0x28)
#define ADC_LF_EN			(31)
#define ADC_RI_EN			(30)
#define ADC_EN				(30)
#define MIC1_EN				(29)
#define MIC2_EN				(28)
#define VMIC_EN				(27)
#define MIC_GAIN			(25)
#define ADC_SELECT			(17)
#define PA_ENABLE			(4)
#define HP_DIRECT			(3)
#define SUNXI_ADC_DEBUG		(0x2c)
#define SUNXI_DAC_TXCNT		(0x30)
#define SUNXI_ADC_RXCNT		(0x34)
#define SUNXI_BIAS_CRT		(0x38)
#define SUNXI_MIC_CRT		(0x3c)


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

static int codec_play_start(struct sunxi_priv *priv)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
#endif

	/* flush TX FIFO */
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_FIFO_FLUSH, 0x1 << DAC_FIFO_FLUSH);

	/* enable DAC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_DRQ, 0x1 << DAC_DRQ);
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << PA_MUTE, 0x1 << PA_MUTE);

	return 0;
}

static int codec_play_stop(struct sunxi_priv *priv)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 0, "audio_pa_ctrl");
#endif

	/* mute PA */
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << PA_MUTE, 0x0 << PA_MUTE);
	mdelay(5);

	/* disable DAC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_DRQ, 0x0 << DAC_DRQ);

	/* mute PA */
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << PA_MUTE, 0x0 << PA_MUTE);
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_L, 0x0 << DACAEN_L);
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_R, 0x0 << DACAEN_R);

	return 0;
}

static int codec_capture_start(struct sunxi_priv *priv)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
#endif

	/* enable ADC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DRQ, 0x1 << ADC_DRQ);

	return 0;
}

static int codec_capture_stop(struct sunxi_priv *priv)
{
	/* disable ADC DRQ */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DRQ, 0x0 << ADC_DRQ);

	/* enable mic1 PA */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << MIC1_EN, 0x0 << MIC1_EN);

	/* enable VMIC */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << VMIC_EN, 0x0 << VMIC_EN);
	if (priv->id == SUN7I) {
		regmap_update_bits(priv->regmap, SUNXI_DAC_TUNE, 0x3 << 8, 0x0 << 8);
	}

	/* enable ADC digital */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DIG_EN, 0x0 << ADC_DIG_EN);

	/* set RX FIFO mode */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << RX_FIFO_MODE, 0x0 << RX_FIFO_MODE);

	/* flush RX FIFO */
	regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_FIFO_FLUSH, 0x0 << ADC_FIFO_FLUSH);

	/* enable adc1 analog */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x3 << ADC_EN, 0x0 << ADC_EN);

	return 0;
}

static int sunxi_codec_trigger(struct snd_pcm_substream *substream, int cmd,
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
			codec_capture_start(priv);
		else
			codec_play_start(priv);
		break;		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			codec_capture_stop(priv);
		else
			codec_play_stop(priv);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sunxi_codec_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 0x1 << DAC_EN, 0x1 << DAC_EN);
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_FIFO_FLUSH, 0x1 << DAC_FIFO_FLUSH);
		/* set TX FIFO send DRQ level */
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x3f << TX_TRI_LEVEL, 0xf << TX_TRI_LEVEL);
		if (substream->runtime->rate > 32000) {
			regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << 28, 0x0 << 28);
		} else {
			regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << 28, 0x1 << 28);
		}
		/* set TX FIFO MODE */
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << TX_FIFO_MODE, 0x1 << TX_FIFO_MODE);
		/* send last sample when DAC FIFO under run */
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << LAST_SE, 0x0 << LAST_SE);
		/* enable dac analog */
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_L, 0x1 << DACAEN_L);
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_R, 0x1 << DACAEN_R);
		/* enable DAC to PA */
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << DACPAS, 0x1 << DACPAS);
	} else {
		/* enable mic1 PA */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << MIC1_EN, 0x1 << MIC1_EN);
		/* mic1 gain 32dB */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x3 << 25, 0x1 << 25);
		/* enable VMIC */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << VMIC_EN, 0x1 << VMIC_EN);

		if (priv->id == SUN7I) {
			/* boost up record effect */
			regmap_update_bits(priv->regmap, SUNXI_DAC_TUNE, 0x3 << 8, 0x1 << 8);
		}

		/* enable ADC digital */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DIG_EN, 0x1 << ADC_DIG_EN);
		/* set RX FIFO mode */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << RX_FIFO_MODE, 0x1 << RX_FIFO_MODE);
		/* flush RX FIFO */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_FIFO_FLUSH, 0x1 << ADC_FIFO_FLUSH);
		/* set RX FIFO rec drq level */
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 0xf << RX_TRI_LEVEL, 0x7 << RX_TRI_LEVEL);
		/* enable adc1 analog */
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x3 << ADC_EN, 0x3 << ADC_EN);
	}
	return 0;
}

static int sunxi_codec_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);
	unsigned int rate = params_rate(params);

	switch (params_rate(params)) {
	case 44100:
	case 22050:
	case 11025:
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

	switch (params_rate(params)) {
	default:
	case 44100:
		rate = 0;
		break;
	case 22050:
		rate = 2;
		break;
	case 11025:
		rate = 4;
		break;
	case 192000:
		rate = 6;
		break;
	case 96000:
		rate = 7;
		break;
	case 48000:
		rate = 0;
		break;
	case 32000:
		rate = 1;
		break;
	case 24000:
		rate = 2;
		break;
	case 16000:
		rate = 3;
		break;
	case 12000:
		rate = 4;
		break;
	case 8000:
		rate = 5;
		break;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 7 << 29, rate << 29);
		if (substream->runtime->channels == 1) {
			regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 1 << 6, 1 << 6);
		} else {
			regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 1 << 6, 0 << 6);
		}
	} else  {
		regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 7 << 29, rate << 29);
		if (substream->runtime->channels == 1) {
			regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 1 << 7, 1 << 7);
		} else {
			regmap_update_bits(priv->regmap, SUNXI_ADC_FIFOC, 1 << 7, 0 << 7);
		}
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
	SOC_SINGLE("LINEIN APM Volume", SUNXI_MIC_CRT, 13, 0x7, 0),
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
	SOC_SINGLE("Mic2 gain Volume", SUNXI_MIC_CRT, 26, 7, 0),
	/*
	*	MIC1 pre-amplifier Gain Control
	*	00:0db,01:35db,10:38db,11:41db
	*/
	SOC_SINGLE("Mic1 gain Volume", SUNXI_MIC_CRT, 29, 3, 0),
};


static int sunxi_codec_dai_probe(struct snd_soc_dai *dai)
{
	struct snd_soc_card *card = snd_soc_dai_get_drvdata(dai);
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	snd_soc_dai_init_dma_data(dai, &priv->playback_dma_data, &priv->capture_dma_data);

	return 0;
}

static int sunxi_codec_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = clk_prepare_enable(priv->clk_module);
	if (ret)
		return ret;

	return 0;
}

static void sunxi_codec_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct sunxi_priv *priv = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(priv->clk_module);
}


static  int codec_init(struct sunxi_priv *priv)
{
	/* enable DAC digital */
	regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 1 << DAC_EN, 1 << DAC_EN);

	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 1 << 28, 1 << 28);
	/* set digital volume to maximum */
	if (priv->id == SUN4A)
		regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 6 << DIGITAL_VOL, 0 << DIGITAL_VOL);
	/* PA mute */
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 1 << PA_MUTE, 0 << PA_MUTE);
	/* enable PA */
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 1 << PA_ENABLE, 1 << PA_ENABLE);
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 3 << DRA_LEVEL, 3 << DRA_LEVEL);
	/* set volume */
	if ((priv->id == SUN4I) || (priv->id == SUN4A)) {
		int rc;
		int device_lr_change = 0;
		if (priv->id == SUN4A) {
			regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x3f << VOLUME, 1 << VOLUME);
		} else {
 			regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x3f << VOLUME, 0x3b << VOLUME);
		}
#ifdef JDS
		rc = script_parser_fetch("audio_para", "audio_lr_change", &device_lr_change, 1);
		if (rc != SCRIPT_AUDIO_OK) {
			pr_err("No audio_lr_change in fex audio_para\n");
			return -1;
		}
		if (device_lr_change)
			regmap_update_bits(priv->regmap, SUNXI_DAC_DEBUG, 1 << DAC_CHANNEL, 1 << DAC_CHANNEL);
#endif
	} else {
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x3f << VOLUME, 0x3b << VOLUME);
	}
	return 0;
}

static const struct snd_soc_dai_ops sunxi_codec_dai_ops = {
	.startup = sunxi_codec_startup,
	.shutdown = sunxi_codec_shutdown,
	.trigger = sunxi_codec_trigger,
	.hw_params = sunxi_codec_hw_params,
	.prepare = sunxi_codec_prepare,
};

static struct snd_soc_dai_driver sunxi_codec_dai = {
	.probe = sunxi_codec_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,

		.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_11025 |\
			 SNDRV_PCM_RATE_22050| SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
			 SNDRV_PCM_RATE_48000 |SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			 SNDRV_PCM_RATE_KNOT),
		.rate_min = 8000,
		.rate_max = 192000,
	},
	.ops = &sunxi_codec_dai_ops,
};

static const struct snd_soc_component_driver sunxi_codec_component = {
	.name = "sunxi-codec",
};

static const struct regmap_config sunxi_codec_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SUNXI_MIC_CRT,
};

static const struct snd_soc_dapm_widget codec_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Mic Bias"),
	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_INPUT("MIC_IN"),
	SND_SOC_DAPM_INPUT("LINE_IN"),
};

static struct snd_soc_dai_link cdc_dai = {
	.name = "cdc",
	.stream_name = "CDC PCM",
	.codec_dai_name = "sunxi-codec-dai",
	.cpu_dai_name = "1c22c00.codec",
	.codec_name = "1c22c00.codec",
	.platform_name = "1c22c00.codec",
	//.init = tegra_wm8903_init,
	//.ops = &tegra_wm8903_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S,
};

static struct snd_soc_card snd_soc_sunxi_codec = {
	.name = "sunxi-codec",
	.owner = THIS_MODULE,
	.dai_link = &cdc_dai,
	.num_links = 1,
};

static struct snd_soc_codec_driver dummy_codec = {
	.controls = sun7i_dac_ctls,
	.num_controls = ARRAY_SIZE(sun7i_dac_ctls),
	.dapm_widgets = codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(codec_dapm_widgets),
};

#define STUB_RATES	SNDRV_PCM_RATE_8000_192000
#define STUB_FORMATS	(SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8 | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_U24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE | \
			SNDRV_PCM_FMTBIT_U32_LE | \
			SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

static struct snd_soc_dai_driver dummy_dai = {
	.name = "sunxi-codec-dai",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates 		= STUB_RATES,
		.formats 	= STUB_FORMATS,
	 },
};

static const struct of_device_id sunxi_codec_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10a-codec", .data = (void *)SUN4A},
	{ .compatible = "allwinner,sun4i-a10-codec", .data = (void *)SUN4I},
	{ .compatible = "allwinner,sun5i-a13-codec", .data = (void *)SUN5I},
	{ .compatible = "allwinner,sun7i-a20-codec", .data = (void *)SUN7I},
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_codec_of_match);

static int sunxi_codec_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_sunxi_codec;
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct sunxi_priv *priv;
	struct resource *res;
	void __iomem *base;
	int ret;

	if (!of_device_is_available(np))
		return -ENODEV;

	of_id = of_match_device(sunxi_codec_of_match, dev);
	if (!of_id)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, priv);

	priv->id = (int)of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					    &sunxi_codec_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->irq = irq_of_parse_and_map(np, 0);
	if (!priv->irq) {
		dev_err(dev, "no irq for node %s\n", np->full_name);
		return -ENXIO;
	}

	/* Clock */
	priv->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->clk_apb)) {
		dev_err(dev, "failed to get apb clock.\n");
		return PTR_ERR(priv->clk_apb);
	}
	priv->clk_pll2 = devm_clk_get(dev, "pll2");
	if (IS_ERR(priv->clk_pll2)) {
		dev_err(dev, "failed to get pll2 clock.\n");
		return PTR_ERR(priv->clk_pll2);
	}
	priv->clk_module = devm_clk_get(dev, "codec");
	if (IS_ERR(priv->clk_module)) {
		dev_err(dev, "failed to get codec clock.\n");
		return PTR_ERR(priv->clk_module);
	}

	ret = clk_set_rate(priv->clk_pll2, 24576000);
	if (ret) {
		dev_err(dev, "set codec base clock rate failed!\n");
		return ret;
	}
	if (clk_prepare_enable(priv->clk_pll2)) {
		dev_err(dev, "try to enable clk_pll2 failed\n");
		return -EINVAL;
	}
	if (clk_prepare_enable(priv->clk_apb)) {
		dev_err(dev, "try to enable clk_apb failed\n");
		return -EINVAL;
	}

	priv->playback_dma_data.addr = res->start + SUNXI_DAC_TXDATA;
	priv->playback_dma_data.maxburst = 4;
	priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	priv->capture_dma_data.addr = res->start + SUNXI_ADC_RXDATA;
	priv->capture_dma_data.maxburst = 4;
	priv->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	ret = snd_soc_register_codec(&pdev->dev, &dummy_codec, &dummy_dai, 1);

	ret = devm_snd_soc_register_component(&pdev->dev, &sunxi_codec_component, &sunxi_codec_dai, 1);
	if (ret)
		goto err_clk_disable;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_clk_disable;

	codec_init(priv);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto err_fini_utils;
	}

	ret = snd_soc_of_parse_audio_routing(card, "routing");
	if (ret)
		goto err;

	return 0;

err_fini_utils:
err:
err_clk_disable:
	clk_disable_unprepare(priv->clk_module);
	clk_disable_unprepare(priv->clk_apb);
	clk_disable_unprepare(priv->clk_pll2);
	return ret;
}

static int sunxi_codec_dev_remove(struct platform_device *pdev)
{
	struct sunxi_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk_module);
	clk_disable_unprepare(priv->clk_apb);
	clk_disable_unprepare(priv->clk_pll2);

	return 0;
}

static struct platform_driver sunxi_codec_driver = {
	.driver = {
		.name = "sunxi-codec",
		.owner = THIS_MODULE,
		.of_match_table = sunxi_codec_of_match,
	},
	.probe = sunxi_codec_probe,
	.remove = sunxi_codec_dev_remove,
};
module_platform_driver(sunxi_codec_driver);

MODULE_DESCRIPTION("sunxi codec ASoC driver");
MODULE_AUTHOR("Emilio López <emilio@elopez.com.ar>");
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
