/*
 * ALSA SoC SPDIF DIT driver
 *
 *  This driver is used by controllers which can operate in DIT (SPDI/F) where
 *  no codec is needed.  This file provides stub codec that can be used
 *  in these configurations. TI DaVinci Audio controller uses this driver.
 *
 * Author:      Steve Chen,  <schen@mvista.com>
 * Copyright:   (C) 2009 MontaVista Software, Inc., <source@mvista.com>
 * Copyright:   (C) 2009  Texas Instruments, India
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/of.h>
#include "sunxi-codec.h"

#define DRV_NAME "awstub"

#define STUB_RATES	SNDRV_PCM_RATE_8000_96000
#define STUB_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE)



static const struct snd_soc_dapm_widget dit_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Mic Bias"),
	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_INPUT("MIC_IN"),
	SND_SOC_DAPM_INPUT("LINE_IN"),
};

static struct snd_soc_codec_driver soc_codec_stub = {
	.dapm_widgets = dit_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dit_widgets),
};

#ifdef JDS
static struct snd_soc_codec_driver soc_codec_sun4a_codec = {
	.controls = sun4a_dac,
	.num_controls = ARRAY_SIZE(sun4a_dac),
};

static struct snd_soc_codec_driver soc_codec_sun4i_codec = {
	.controls = sun4i_dac,
	.num_controls = ARRAY_SIZE(sun4i_dac),
};
#endif

static struct snd_soc_dai_driver awstub_dai = {
	.name		= "awstub",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
};

static int stub_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_stub, &awstub_dai, 1);
}

static int stub_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id stub_dt_ids[] = {
	{ .compatible = "allwinner,stub", },
	{ }
};
MODULE_DEVICE_TABLE(of, stub_dt_ids);
#endif

static struct platform_driver stub_driver = {
	.probe		= stub_probe,
	.remove		= stub_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(stub_dt_ids),
	},
};

module_platform_driver(stub_driver);

MODULE_AUTHOR("Steve Chen <schen@mvista.com>");
MODULE_DESCRIPTION("AWStub codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
